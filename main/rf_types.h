/*
 * rf_types.h — 共享数据类型（纯 C99，零依赖）
 *
 * 职责：
 *   - 定义信道/频段参数（nRF24L01 2.4G）
 *   - 定义二进制帧协议常量（UART 同步码、最大载荷、TYPE 枚举）
 *   - 定义线上 RID 定长结构（C5 -> P4 的打包格式）
 *   - 定义 P4 侧 UI 用到的无人机数据模型 rf_drone_t
 *   - 定义协议分析仪命中条目 rf_proto_hit_t
 *   - 定义串口行级日志 / RID0 原始帧 环形缓冲的槽类型
 *   - 定义调试诊断结构 rf_debug_t / 扫描配置 rf_scan_config_t / 频段模式
 *
 * 设计原则：
 *   - 只依赖 <stdint.h>/<stddef.h>/<stdbool.h>，不依赖任何厂商 SDK。
 *   - 凡是需要"移植就能用"的模块（rf_json / rid_registry / rf_proto /
 *     rf_ringbuf / rf_link）都从这里取类型；模块间不再相互 include 业务头。
 *
 * 移植：
 *   - 单独拷贝本文件即可在任何 C99 工程里拿到一致的类型定义。
 *   - 若要调整最大无人机数 / 频谱信道数 / 日志缓冲行宽等，改这里的宏即可
 *     （其它模块会自动跟随）。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 信道/频段参数（nRF24L01 2.4G） ------------------------------------- */
#define RF_NUM_CHANNELS         125     /* nRF24L01 信道数 (2400..2524 MHz) */
#define RF_CHAN_FREQ_MHZ_BASE   2400    /* 信道 0 = 2400 MHz */
#define RF_CHAN_FREQ_MHZ(ch)    (RF_CHAN_FREQ_MHZ_BASE + (ch))

/* ---- 二进制帧协议常量（UART: AA 55 TYPE LEN_LO LEN_HI PAYLOAD CRC8） ----- */
#define RF_SYNC0                0xAA
#define RF_SYNC1                0x55
#define RF_MAX_PAYLOAD          512

enum rf_msg_type {
    RF_MSG_SPECTRUM   = 0x01,   /* 125B 扫频 RSSI */
    RF_MSG_RID_RECORD = 0x02,   /* 91B 无人机 RID 记录 */
    RF_MSG_RID_REMOVE = 0x03,   /* 21B 移除无人机 */
    RF_MSG_RAW_PKT    = 0x04,   /* 原始 2.4G 抓包 */
    RF_MSG_STATUS     = 0x10,   /* C5 心跳/状态 */
};

/* ---- 线上 RID 结构（C5 下发的定长打包载荷，小端，packed） --------------- */
/* 对应 ASTM F3411-22 Remote ID；移植到其他 MCU 时结构体大小必须保持一致。 */
typedef struct __attribute__((packed)) {
    uint8_t  uas_id_type;        /* 1=Serial,2=ANSI/CTA,3=UTM-assigned,4=CAA-reg... */
    uint8_t  uas_id_len;
    char     uas_id[20];
    uint8_t  uas_type;
    uint8_t  op_desc_len;
    char     op_desc[23];
    uint8_t  operator_id_len;
    char     operator_id[20];
    int32_t  lat_e7;              /* UA 纬度 1e-7 度 */
    int32_t  lon_e7;              /* UA 经度 1e-7 度 */
    int16_t  alt_pressure_m;
    int16_t  alt_geodetic_m;
    int16_t  height_m;
    uint8_t  horiz_speed;         /* m/s */
    int8_t   vert_speed;          /* m/s, +升 -降 */
    uint16_t ground_track;        /* 0.01° (0..36000) */
    int32_t  operator_lat_e7;     /* 操作人纬度 1e-7 度 */
    int32_t  operator_lon_e7;     /* 操作人经度 1e-7 度 */
    uint32_t src_timestamp_ms;    /* C5 侧时间戳 ms */
    uint8_t  flags;
} rf_rid_wire_t;
_Static_assert(sizeof(rf_rid_wire_t) == 99, "rf_rid_wire_t must be 99 bytes on wire");

/* C5 STATUS 载荷（TYPE=0x10，14 字节） */
typedef struct __attribute__((packed)) {
    uint32_t uptime_s;
    uint16_t sweep_rate_hz;
    uint8_t  nrf_ok;
    uint8_t  ble_ok;
    uint8_t  drone_count;
    uint8_t  reserved;
    uint32_t bytes_tx_total;
} rf_status_wire_t;
_Static_assert(sizeof(rf_status_wire_t) == 14, "rf_status_wire_t must be 14 bytes");

/* ---- P4 侧无人机数据模型（UI 读取用，非 packed） ------------------------ */
#define RF_MAX_DRONES           16

typedef struct {
    bool     active;
    uint8_t  uas_id_type;
    char     uas_id[24];
    char     op_desc[28];
    char     operator_id[24];
    int32_t  lat_e7, lon_e7;                   /* UA 经纬度 1e-7 度 */
    int32_t  operator_lat_e7, operator_lon_e7; /* 操作人经纬度 1e-7 度 */
    int16_t  alt_pressure_m, alt_geodetic_m, height_m;
    uint8_t  horiz_speed;
    int8_t   vert_speed;
    uint16_t ground_track;
    uint32_t last_seen_ms;                     /* 本地 ms 时间戳 */
    uint8_t  flags;
} rf_drone_t;

/* ---- 协议分析仪命中条目（rf_proto 与 UI 共享） -------------------------- */
#define RF_PROTOCOL_MAX_HITS 8

typedef struct {
    int16_t  center_idx;    /* 中心通道下标 */
    uint16_t center_mhz;    /* 中心频率 MHz */
    uint8_t  bw_channels;   /* 跨越通道数 */
    uint8_t  peak_level;    /* 峰值强度 0..100 */
    uint8_t  avg_level;     /* 平均强度 */
    uint8_t  confidence;    /* 置信度 0..100 */
    uint8_t  proto_id;      /* 0=Unknown 1=WiFi 2=BLE 3=Zigbee 4=NRF24 5=BLE/Zigbee 6=宽带干扰 */
    char     label[28];     /* 显示标签 */
} rf_proto_hit_t;

/* ---- 扫描配置（P4->C5 反向命令共用） ------------------------------------ */
typedef struct {
    int  start_ch;    /* 起始通道 0..125 */
    int  end_ch;      /* 结束通道 0..125 */
    int  rate_idx;    /* 0=250K 1=1M 2=2M */
    int  settle_us;   /* 驻留时间 us */
    int  spp;         /* 采样次数 */
} rf_scan_config_t;

typedef enum { RF_BAND_24G = 0, RF_BAND_5G = 1 } rf_band_t;

/* ---- 链路统计 ----------------------------------------------------------- */
typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t crc_errors;
    uint32_t sync_errors;
    uint32_t spectrum_frames;
    uint32_t rid_frames;
    uint32_t last_status_ms;
    rf_status_wire_t last_status;
    bool     link_online;
    uint32_t sweep_rate_hz;
    /* Qt 上位机 JSON 协议统计 */
    uint32_t json_lines;
    uint32_t json_D;
    uint32_t json_W;
    uint32_t json_R;
    uint32_t json_R0;
    uint32_t json_A;
    /* 原始 RX 字节级诊断 */
    uint32_t raw_rx_peak_bytes;
    uint32_t raw_rx_total_chunks;
    uint32_t last_rx_byte_ms;
    uint32_t last_valid_frame_ms;
} rf_stats_t;

/* ---- UART 调试诊断 ------------------------------------------------------ */
#define RF_DEBUG_RAW_N 64
typedef struct {
    uint8_t  raw[RF_DEBUG_RAW_N];
    int      raw_len;
    uint8_t  last_frame_type;
    uint16_t last_frame_len;
    bool     last_crc_ok;
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t crc_errors;
    uint32_t sync_errors;
} rf_debug_t;

/* ---- 串口行级日志环形缓冲槽类型 ---------------------------------------- */
#define RF_LOG_RING_LINES   128
#define RF_LOG_LINE_MAX     192

typedef enum {
    RF_LOG_LINE_RX_LOG   = 0,
    RF_LOG_LINE_RX_DATA  = 1,
    RF_LOG_LINE_RX_WIFI5 = 2,
    RF_LOG_LINE_RX_ACK   = 3,
    RF_LOG_LINE_RX_RID   = 4,
    RF_LOG_LINE_RX_RID0  = 5,
    RF_LOG_LINE_TX_CMD   = 6,
} rf_log_line_type_t;

typedef struct {
    uint8_t  type;
    uint16_t len;
    char     text[RF_LOG_LINE_MAX];
} rf_log_line_t;

typedef struct {
    uint32_t total;
    uint32_t dropped;
} rf_log_stats_t;

/* ---- RID 原始 802.11 广播帧（$R0） ------------------------------------- */
#define RF_RID0_RING_MAX     64
#define RF_RID0_HEX_MAX     512

typedef struct {
    char     mac[20];
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  subtype;
    uint8_t  has_rid;
    uint16_t length;
    uint16_t hex_len;
    char     hex[RF_RID0_HEX_MAX];
    uint32_t seq;
} rf_rid0_frame_t;

#ifdef __cplusplus
}
#endif
