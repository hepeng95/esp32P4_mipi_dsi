/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * rf_gps.h — DX-GP22-A 北斗/GPS 双模 GNSS 模块驱动
 *
 * 模块特性 (典型值, 参考大夏龙雀产品系列):
 *   - GNSS: GPS + BDS (北斗), 支持 GLONASS/Galileo (固件版本相关)
 *   - 接口: UART, 默认 9600 8N1 (可通过 PMTK 指令改)
 *   - 输出: NMEA 0183, 典型语句 $GNGGA / $GNRMC / $GNGSV / $GPGST
 *
 * 引脚 (可改):
 *   ESP32-P4  UART2_TX  → GPIO6  (连 GPS 模块 RX)
 *   ESP32-P4  UART2_RX  → GPIO7  (连 GPS 模块 TX)
 *   GPS PPS (可选)      → GPIO5   (1Hz 定位脉冲, 不用 NC)
 *   GPS RST (可选)      → NC
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 引脚 & UART 配置 (可改) ======================== */
#define RF_GPS_UART_NUM       UART_NUM_2
/* ★ GPIO 避开 nRF24L01 SPI: MISO=4 MOSI=5 SCK=6 CSN=2 CE=7
 *   改到 GPIO 11/12 (ESP32-P4 通用 IO, 无冲突) */
#define RF_GPS_UART_TX        11      /* ESP → GPS RX */
#define RF_GPS_UART_RX        12      /* ESP ← GPS TX */
#define RF_GPS_UART_BAUD      9600    /* DX-GP22-A 默认 9600 */
#define RF_GPS_PPS_PIN        (-1)    /* PPS 不用写 -1; 若接 GPIO5 写 5 */

/* ======================== 数据结构 ======================== */

/* 定位状态 (来自 GGA) */
typedef enum {
    RF_GPS_FIX_NONE      = 0,    /* 未定位 */
    RF_GPS_FIX_AUTO      = 1,    /* GNSS 单点 */
    RF_GPS_FIX_DGPS      = 2,    /* DGPS / SBAS */
    RF_GPS_FIX_PPS       = 3,    /* PPS */
    RF_GPS_FIX_RTK_FLOAT = 4,    /* RTK float */
    RF_GPS_FIX_RTK_FIX   = 5,    /* RTK fix */
    RF_GPS_FIX_EST       = 6,    /* 估算 */
    RF_GPS_FIX_MANUAL    = 7,    /* 手动输入 */
    RF_GPS_FIX_SIM       = 8,    /* 模拟 */
} rf_gps_fix_t;

/* 完整定位快照 (所有时间戳都是模块输出的 UTC) */
typedef struct {
    int32_t  lat_e7;          /* 纬度  x1e7 度, 例 231429100 = 23.1429100 */
    int32_t  lon_e7;          /* 经度  x1e7 度, 例 1132602386 = 113.2602386 */
    int32_t  alt_cm;          /* 海拔 (椭球高) 厘米 */
    int32_t  alt_msl_cm;      /* 海拔 (MSL) 厘米, 没给则 0 */
    int32_t  hdop_x100;       /* HDOP x100 (例 120 = 1.20) */
    int32_t  vdop_x100;       /* VDOP x100 */
    int32_t  speed_x100;      /* 地面速度 x100 km/h, 0=无效 */
    int32_t  heading_x100;    /* 航向 x100 度, 0=无效 */
    uint8_t  num_sats;        /* 可见卫星数 (GGA) / 参与解算数 */
    rf_gps_fix_t fix;         /* 定位状态 */
    uint8_t  year;            /* UTC 年 (2000 + yr) */
    uint8_t  month;           /* UTC 月 */
    uint8_t  day;             /* UTC 日 */
    uint8_t  hour;            /* UTC 时 */
    uint8_t  minute;          /* UTC 分 */
    uint8_t  second;          /* UTC 秒 */
    uint8_t  cs_minute;       /* UTC 分 (完整度) */
    uint16_t cs_ms;           /* UTC ms */
    int64_t  ts_monotonic_ms; /* esp_timer 本地时间戳 (millis) */
    bool     valid;           /* 本帧是否有有效坐标 */
} rf_gps_state_t;

/* ======================== 公开 API ======================== */

/** 启动 GPS UART + NMEA 解析后台任务 */
esp_err_t rf_gps_start(int priority);

/** 停止 */
void rf_gps_stop(void);

/** 取最新定位快照 (线程安全)
 *  @return true=有有效定位, false=未定位或超时 */
bool rf_gps_get(rf_gps_state_t *out, int *age_ms);

/** 是否定位有效 (fix > RF_GPS_FIX_NONE, 且坐标新鲜) */
bool rf_gps_has_fix(void);

/** 发送一条 NMEA/PMTK 指令到 GPS 模块 (调试/配置用) */
esp_err_t rf_gps_send_cmd(const char *cmd);

#ifdef __cplusplus
}
#endif
