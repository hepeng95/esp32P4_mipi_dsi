/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * rf_5g.h — 5.8GHz RSSI 扫描层 (基于 A5133 SPI 驱动)
 *
 * 独立于 rf_link (2.4G UART)：P4 直接驱动 A5133 做 5.8G 扫频
 * 数据源接口 rf_5g_get_spectrum() 与 rf_link_get_spectrum() 签名一致，
 * rf_ui timer_cb 可按 s_active_band 在两者间切换。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "rf_types.h"

/* A5133 PLL_I 是 8-bit (CHN[7:0] = 0~255)
   FRF = 5725 + CHN×1MHz → 5725~5980 MHz, 共 256 通道 */
#define RF5G_CHANNELS   256

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 A5133 + 5.8G RSSI 扫描后台任务
 * @param priority 任务优先级 (建议 4, 低于 UART RX 5 和 LVGL 默认)
 */
esp_err_t rf_5g_start(int priority);

/** @brief 停止扫描任务 */
void rf_5g_stop(void);

/**
 * @brief 取最新 5.8G 扫描帧 (给 rf_ui timer_cb 调用, 和 rf_link_get_spectrum 签名一致)
 * @param[out] rssi_out  必须 >= RF5G_CHANNELS (256) 字节, 每字节 = 原始 RSSI 0~255
 * @param[out] sweep_cnt 扫描帧序号 (递增)
 * @param[out] age_ms    距最新帧的毫秒数
 * @return true 有有效数据
 */
bool rf_5g_get_spectrum(uint8_t *rssi_out, uint32_t *sweep_cnt, uint32_t *age_ms);

/** @brief 当前扫描正在运行 */
bool rf_5g_is_running(void);

/** 调试: 手动触发一次扫描 (非线程安全) */
void rf_5g_sweep_once(void);

#ifdef __cplusplus
}
#endif
