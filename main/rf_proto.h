/*
 * rf_proto.h — 2.4G 频谱协议分析仪（纯 C，零硬件/RTOS 依赖）
 *
 * 输入: 125 通道的 RSSI 样本 (0..255)，信道 i = 2400+i MHz
 * 输出: 最多 RF_PROTOCOL_MAX_HITS 个检测命中条目，含中心/带宽/峰值/置信度/协议名。
 *
 * 移植：rf_proto.h/c + rf_link.h（只取到 RF_NUM_CHANNELS、rf_proto_hit_t）即可。
 */
#pragma once

/*
 * 依赖策略（便于移植）：
 *   - 只通过 rf_types.h 拿到 RF_NUM_CHANNELS / rf_proto_hit_t / RF_PROTOCOL_MAX_HITS
 *   - 不 include rf_link.h，不依赖任何厂商 SDK。
 */
#include "rf_types.h"   /* RF_NUM_CHANNELS / rf_proto_hit_t / RF_PROTOCOL_MAX_HITS */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 纯函数版频谱分析：
 *        调用方自己取出 RSSI 数据后直接投喂，不依赖 rf_link 内部互斥。
 *        （rf_link_analyze_spectrum 是本函数的包装：它内部调 rf_link_get_spectrum -> 这里。）
 *
 * @param rssi      输入 RSSI 数组，至少 RF_NUM_CHANNELS 字节（0..255，越大越强）
 * @param sweep_cnt 扫描次数（0 视为无数据，返回 0）；可传 -1 强制分析
 * @param out       输出命中数组
 * @param out_max   输出容量
 * @return          命中数量（<= out_max）
 */
int rf_proto_analyze(const uint8_t rssi[RF_NUM_CHANNELS], int sweep_cnt,
                     rf_proto_hit_t *out, int out_max);

#ifdef __cplusplus
}
#endif
