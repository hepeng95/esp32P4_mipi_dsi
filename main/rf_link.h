/*
 * rf_link.h — ESP32-P4 侧 RF 数据链路层（UART 接收 + 协议解析 + 数据模型）
 *
 * 数据来源：ESP32-C5（驱动 nRF24L01 扫频 + 采集无人机 ASTM F3411 Remote ID），
 *           经 UART 统一打包发送给 ESP32-P4 进行显示。
 *
 * ============================================================================
 *  UART 帧协议（C5 -> P4，单向，9600~460800bps，建议 460800）
 * ============================================================================
 *  字节布局（小端）：
 *
 *    [0xAA][0x55][TYPE][LEN_LO][LEN_HI] [ PAYLOAD(LEN) ... ] [CRC8]
 *    |同步 |同步 |  1B  |  2B 长度     |  LEN 字节载荷        |  1B 校验
 *
 *  - SYNC     = 0xAA 0x55（两字节帧头，用于字节流对齐）
 *  - TYPE     = 1 字节消息类型（见下方 RF_MSG_TYPE_*）
 *  - LEN      = 2 字节小端，载荷长度（0..RF_MAX_PAYLOAD）
 *  - PAYLOAD  = LEN 字节，按 TYPE 解释
 *  - CRC8     = 对 [TYPE][LEN_LO][LEN_HI][PAYLOAD] 计算的 CRC-8
 *               多项式 0x07，初值 0x00，不反转（即 CRC-8/SMBUS 风格）
 *
 *  超时重同步：若字节间间隔 > 200ms 且未凑成完整一帧，状态机回到等同步。
 *  长度异常：若 LEN > RF_MAX_PAYLOAD，丢弃并回等同步。
 *
 * ============================================================================
 *  消息类型 TYPE
 * ============================================================================
 *  0x01  SPECTRUM     2.4G 扫频一帧
 *      PAYLOAD:  uint8 rssi[RF_NUM_CHANNELS]   // 125 字节，0..255，越大信号越强
 *                （约定：C5 对每信道取若干样本，RPD+LNA 增益估算占用度/功率，映射 0..255）
 *
 *  0x02  RID_RECORD   无人机 Remote ID 记录（C5 已解析 ASTM F3411 字段后下发）
 *      PAYLOAD 为定长结构 rf_rid_wire_t（见下），共 91 字节
 *      收到后：按 uas_id 查找/更新无人机列表，刷新 last_seen
 *
 *  0x03  RID_REMOVE   无人机消失（超时）
 *      PAYLOAD:  uint8 uas_id_type + char uas_id[20]   // 21 字节，指定要移除的无人机
 *
 *  0x04  RAW_PKT      原始 2.4G 抓包（调试 / raw 视图）
 *      PAYLOAD:  uint8 channel(0..124) + uint8 rssi + uint8 plen + uint8 data[plen]
 *                共 3 + plen 字节
 *
 *  0x10  STATUS       C5 心跳/状态
 *      PAYLOAD:  uint32 uptime_s + uint16 sweep_rate_hz + uint8 nrf_ok + uint8 ble_ok
 *                + uint8 drone_count + uint8 reserved + uint32 bytes_tx_total   // 14 字节
 *
 *  其余 TYPE 保留。未知类型：校验通过后忽略（仅计入统计）。
 *
 * ============================================================================
 *  C5 端实现要点
 * ============================================================================
 *  - nRF24L01：SPI 驱动，置 RX 模式，逐信道设置 RF_CH(0..124)，停留 ~200us 采样，
 *    估算 rssi（RPD 多次采样统计 + 可选多次回读），组成 125 字节后按 TYPE=0x01 发送。
 *    扫描率约 5~10Hz 即可（125 信道 * 200us ≈ 25ms + 开销）。
 *  - RID：用 ESP32-C5 的 WiFi 监听（NaN/Broadcast）或 BLE 扫描，解析 ASTM F3411 各消息，
 *    聚合成一条 rf_rid_wire_t 后按 TYPE=0x02 发送；位置变化时重发；超时发 TYPE=0x03。
 *  - 串口：建议 460800 8N1，TX/RX 交叉连接 P4 的 UART，共地。每帧带 CRC8。
 *  - 帧间无需间隔，靠 0xAA0x55 + 长度自对齐。
 */

#pragma once

/*
 * 依赖策略：
 *   - 所有共享类型（rf_rid_wire_t / rf_drone_t / rf_stats_t / rf_proto_hit_t 等）
 *     已抽到 rf_types.h。这里对外再 re-export（含入 rf_types.h），
 *     调用方只需 include 一个 rf_link.h 即可拿到全部业务 + 类型。
 *   - ESP 相关 esp_err_t 单独 include。
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "rf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 信道/频段参数（rf_types.h 已提供；保留宏重复定义保护） ------------ */
#ifndef RF_NUM_CHANNELS
#define RF_NUM_CHANNELS         125
#endif
#ifndef RF_CHAN_FREQ_MHZ_BASE
#define RF_CHAN_FREQ_MHZ_BASE   2400
#define RF_CHAN_FREQ_MHZ(ch)    (RF_CHAN_FREQ_MHZ_BASE + (ch))
#endif
#ifndef RF_SYNC0
#define RF_SYNC0                0xAA
#define RF_SYNC1                0x55
#define RF_MAX_PAYLOAD          512
#endif
/* enum rf_msg_type 已在 rf_types.h 定义 */
/* rf_rid_wire_t / rf_status_wire_t / rf_drone_t / RF_MAX_DRONES 已在 rf_types.h */
/* rf_stats_t / rf_debug_t / rf_scan_config_t / rf_band_t / RF_PROTOCOL_MAX_HITS /
   rf_proto_hit_t / 日志 / RID0 类型 均已在 rf_types.h 定义。

   下面保留所有 API 声明（业务/链路层）。*/

/* ============================================================================
 *  API
 * ============================================================================ */

/**
 * @brief 启动 RF 链路。
 *        - 若 CONFIG_EXAMPLE_RF_SIMULATE 打开，启动内置模拟数据任务（无需 C5 即可演示 UI）。
 *        - 若 UART TX/RX 引脚已配置（>=0），安装 UART 驱动并启动 RX 解析任务。
 *        可在 LVGL 初始化前后调用；本函数内部不触碰 LVGL。
 * @return ESP_OK / esp_err 错误码
 */
esp_err_t rf_link_start(void);

/**
 * @brief 运行时切换数据源（供 UI【模拟数据】/【串口实际数据】按钮调用）。
 * @param use_sim true=使用内置模拟数据；false=使用 UART 真实串口数据。
 *                切到 UART 时若驱动尚未安装会尝试一次初始化。
 */
void rf_link_set_source(bool use_sim);

/**
 * @brief 取当前数据源模式。
 * @return true=模拟数据；false=UART 真实数据。
 */
bool rf_link_get_source(void);

/**
 * @brief 开/关 UART 实时通信调试输出（写到 Serial Monitor 环形缓冲）。
 *        打开后，每笔 UART TX/RX bytes、每帧解析结果(OK/CRC/SYNC) 都会追加一行。
 *        关闭后立即停止写入。
 */
void rf_link_set_uart_debug_output(bool on);

/**
 * @brief 取当前 UART 调试输出开关状态。
 */
bool rf_link_get_uart_debug_output(void);

/**
 * @brief 运行时重新配置 UART 波特率（不卸载驱动，避免卡死）。
 *        切换后清空 RX FIFO、向日志面板写一条“BAUD CHANGE baud=XXX OK/ERR”。
 * @param baud 目标波特率，常用值: 9600 / 19200 / 57600 / 115200 / 230400 / 460800 / 921600。
 * @return ESP_OK 成功；否则返回 uart_set_baudrate/param_config 错误码。
 */
esp_err_t rf_link_set_uart_baud(int baud);

/**
 * @brief 取当前 UART 波特率（只读）。若 UART 尚未初始化则返回 0。
 */
int rf_link_get_uart_baud(void);

/**
 * @brief 一键自动探测波特率：依次尝试 115200 -> 460800 -> 921600 -> 9600 -> 230400，
 *        每个等待 probe_ms 毫秒，若期间 rx_frames 或 rx_bytes > 阈值即停留在此波特率。
 * @param probe_ms 每个候选值等待时长(ms)，建议 600~1200。
 * @return 命中的波特率；若都没命中返回 0 并保留最后一次尝试的波特率。
 */
int rf_link_autodetect_baud(uint32_t probe_ms);

/**
 * @brief 取最新一帧扫频 RSSI（拷贝，线程安全）。
 * @param rssi_out  输出缓冲，至少 RF_NUM_CHANNELS 字节；可为 NULL
 * @param sweep_cnt 输出：C5 侧累计扫描计数（每帧 +1），用于判断是否有新帧
 * @param age_ms    输出：距上一帧的 ms（本地计时），NULL 则忽略
 * @return true=有数据；false=从未收到任何扫频帧
 */
bool rf_link_get_spectrum(uint8_t rssi_out[RF_NUM_CHANNELS], uint32_t *sweep_cnt, uint32_t *age_ms);

/**
 * @brief 取无人机列表（拷贝，线程安全）。
 * @param out       输出数组，容量 >= max_count
 * @param max_count 调用方数组容量
 * @return 实际活动的无人机数量（<= RF_MAX_DRONES）
 */
int rf_link_get_drones(rf_drone_t *out, int max_count);

/**
 * @brief 取链路统计（拷贝，线程安全）。
 */
void rf_link_get_stats(rf_stats_t *out);

/**
 * @brief RID 列表的“世代号”：每当无人机增删/ID 变化时 +1。
 *        UI 可据此判断是否需要重建列表，避免每帧重绘闪烁。
 */
uint32_t rf_link_drones_generation(void);

/* ============================================================================
 *  调试接口
 * ============================================================================ */
/* RF_DEBUG_RAW_N / rf_debug_t 已在 rf_types.h 定义 */

/**
 * @brief 取调试信息（最近原始数据 + 帧统计），线程安全。
 */
void rf_link_get_debug(rf_debug_t *out);

/* ============================================================================
 *  命令发送（P4 -> C5）
 * ============================================================================ */
/* rf_scan_config_t / rf_band_t 已在 rf_types.h 定义 */

/**
 * @brief 发送扫描配置到 C5（APPLY 按钮用）
 */
void rf_link_send_scan_cfg(const rf_scan_config_t *cfg);

/**
 * @brief 发送频段模式切换命令
 */
void rf_link_send_band_mode(rf_band_t band);

/**
 * @brief 发送 RID 原始数据等级设置
 * @param level 0=关 1=仅 RID 2=所有 Beacon/ProbeResp 3=所有 Mgmt
 */
void rf_link_send_rid_raw_level(int level);

/**
 * @brief 发送信号增益命令到 C5：{cmd:gain,val:X}
 *        增益 1-10 倍，作为频谱图和瀑布图的显示增益系数。
 * @param gain 1-10
 */
void rf_link_send_gain(int gain);

/**
 * @brief 取最近一次 C5 确认的配置（$A 应答），线程安全。
 *        若从未收到过 $A，返回全 0。
 */
void rf_link_get_last_cfg(rf_scan_config_t *out);

/* ============================================================================
 *  协议分析仪（基于频谱带宽+中心频率特征）
 * ============================================================================ */
/* RF_PROTOCOL_MAX_HITS / rf_proto_hit_t 已在 rf_types.h 定义 */

/**
 * @brief 对当前频谱数据运行协议分析。
 * @param out       输出数组，至少 RF_PROTOCOL_MAX_HITS 个条目
 * @param out_max   数组容量
 * @return 实际检测到的信号数量
 */
int rf_link_analyze_spectrum(rf_proto_hit_t *out, int out_max);

/**
 * @brief 当前扫描配置显示字段，线程安全。
 *        基于最近一次 $D 帧解析的 s/e/r/st/sp 字段。
 * @param out 输出；若从未收到过数据，填默认 0,125,2,300,2
 */
void rf_link_get_scan_status(rf_scan_config_t *out);

/* ============================================================================
 *  串口行级日志环形缓冲（供 Serial Monitor 面板使用）
 * ============================================================================ */
/* RF_LOG_RING_LINES / RF_LOG_LINE_MAX / rf_log_line_type_t / rf_log_line_t /
   rf_log_stats_t 均已在 rf_types.h 定义 */

/**
 * @brief 读取日志环形缓冲（快照式，线程安全）。
 * @param out      调用方缓冲，至少 out_max 个条目
 * @param out_max  希望读取的条数
 * @param from_seq 入参：上次已经读到的 seq（total 值）；传入 0 表示从头读取。
 *                 出参：当前最新 total，供下次增量读取。
 * @return 实际复制到 out 的条数
 */
int rf_log_read(rf_log_line_t *out, int out_max, uint32_t *from_seq);

/**
 * @brief 取日志统计（累计写入/丢弃数，线程安全）。
 */
void rf_log_get_stats(rf_log_stats_t *out);

/* ============================================================================
 *  RID 原始 802.11 广播帧（$R0）
 * ============================================================================ */
/* RF_RID0_RING_MAX / RF_RID0_HEX_MAX / rf_rid0_frame_t 均已在 rf_types.h 定义 */

/**
 * @brief 读取 RID 原始帧环形缓冲（增量，线程安全）。
 * @param out       输出缓冲
 * @param out_max   容量
 * @param from_seq  入参：上次读到的 seq+1，0 表示从头取
 *                  出参：最新 seq+1，供下次继续
 * @return 实际取到的帧数
 */
int rf_link_read_rid0_frames(rf_rid0_frame_t *out, int out_max, uint32_t *from_seq);

#ifdef __cplusplus
}
#endif
