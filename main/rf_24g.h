/*
 * rf_24g.h — 2.4GHz 频谱扫描 (nRF24L01 本地 SPI)
 *
 * 和 rf_5g (A5133 5.8G) 同架构：
 *   - Producer 任务扫 126 通道 nRF24L01 RPD, 写 s_rssi[] + version++
 *   - Consumer (rf_ui timer_cb) 读 version_old → memcpy → 读 version_new,
 *     若变了就重试 (最多 3 次), 零阻塞
 *
 * nRF24L01 RPD 只有 0/1 (载波检测), 所以每通道采 N 次, 命中率映射到 0~255
 *
 * 数据源优先级: rf_ui timer_cb 里 RF_BAND_24G 先试本模块; 若未启动再 fallback rf_link (C5 UART)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* nRF24L01 通道范围: 0~125 → 2400~2525 MHz, 126 通道 */
#define RF24_CHANNELS  126

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 nRF24L01 + 2.4G 扫描后台任务
 * @param priority 建议 3 (低于 LVGL / UART)
 */
esp_err_t rf_24g_start(int priority);

/** @brief 停止扫描任务 */
void rf_24g_stop(void);

/** @brief 本模块是否已启动 (rf_ui timer_cb 用来决定数据源) */
bool rf_24g_is_running(void);

/**
 * @brief 取最新 2.4G 扫描帧
 * @param[out] rssi_out  必须 >= RF24_CHANNELS (126) 字节, 值 0~255
 * @param[out] sweep_cnt 扫描帧序号 (递增)
 * @param[out] age_ms    距最新帧的毫秒数
 * @return true 有有效数据
 */
bool rf_24g_get_spectrum(uint8_t *rssi_out, uint32_t *sweep_cnt, uint32_t *age_ms);

#ifdef __cplusplus
}
#endif
