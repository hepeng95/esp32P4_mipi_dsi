/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * rf_5g.c — A5133 SPI 5.8G 扫描任务
 *
 * 跨核安全 (LVGL CPU0 ←→ 本任务 CPU1):
 *   完全不用 mutex, 改用 __atomic version counter + 重试读
 *   — Producer 写数据 → version++ (atomic release)
 *   — Consumer 读 version_old → memcpy 数据 → 读 version_new (atomic acquire)
 *     若 version_old != version_new, 重试 (最多 3 次, 保证 LVGL timer 不阻塞)
 *
 *   这样 LVGL 跨核调用 rf_5g_get_spectrum() 零阻塞, 不触发 FreeRTOS 跨核
 *   优先级继承断言 (xTaskPriorityDisinherit pxTCB==pxCurrentTCBs)
 */

#include "rf_5g.h"
#include "a5133.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include <stdatomic.h>

static const char *TAG = "rf_5g";

/* ====== 通道配置 ====== */
#define RF5G_CHANNELS      256    /* 5725 ~ 5980 MHz, 1MHz 步进 (A5133 PLL_I 8-bit 全范围) */
#define RF5G_BASE_KHZ      5725000
#define RF5G_STEP_KHZ      1000
#define RF5G_SETTLE_MS     1      /* PLL 稳定等待 */

/* ====== 任务参数 ====== */
#define RF5G_TASK_STACK     3072
#define RF5G_TASK_PRIORITY  3
#define RF5G_TASK_CORE      1      /* ★ CPU1 — 不抢占 LVGL */

/* ====== 无锁共享缓冲 ====== */
static TaskHandle_t  s_task_hdl      = NULL;
static volatile uint32_t s_version   = 0;    /* atomic counter */
static uint8_t       s_rssi[RF5G_CHANNELS];
static uint32_t      s_sweep_cnt     = 0;
static int64_t       s_last_tick_us  = 0;
static volatile bool s_running       = false;

/* ---- 扫描主循环 ---- */

static void rf_5g_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "A5133 + 5.8G scan task start (channel=0..%d, base=%lu MHz)",
             RF5G_CHANNELS - 1, (unsigned long)(RF5G_BASE_KHZ / 1000));

    esp_task_wdt_add(NULL);   /* 订阅 WDT, 自己喂狗 */

    if (a5133_init() != ESP_OK) {
        ESP_LOGE(TAG, "A5133 init FAILED, scan task abort");
        vTaskDelete(NULL);
        s_task_hdl = NULL;
        return;
    }

    int fail_cnt = 0;
    int sweep_cnt_local = 0;

    /* 预热: 先发一次 Strobe RX, 让 A5133 从 Standby 进 RX 模式 */
    a5133_set_standby();
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 启动后挂起 500ms — 让 app_main 有充足时间跑完 NRF24 init + rf_24g_start + return,
     * 否则 scan 任务 (prio 2) 会在 while(1) 里继续抢占 app_main (prio 1) */
    vTaskDelay(pdMS_TO_TICKS(500));

    while (s_running) {
        /* ====== 批量扫频 ====== */
        int64_t t0 = esp_timer_get_time();
        a5133_scan_all_channels(s_rssi, RF5G_CHANNELS);
        int64_t t1 = esp_timer_get_time();
        int64_t frame_us = t1 - t0;

        /* 提交一帧 */
        s_sweep_cnt++;
        sweep_cnt_local++;
        s_last_tick_us = t1;
        __atomic_store_n(&s_version, __atomic_load_n(&s_version, __ATOMIC_RELAXED) + 1, __ATOMIC_RELEASE);

        esp_task_wdt_reset();   /* 喂狗 (5s timeout, 帧间 54ms 远小于此) */

        /* ---- DEBUG: 暂时关掉 sweep 统计 printf, 只留 JSON (避免 USB 阻塞) ---- */
#if 1
        if (sweep_cnt_local % 20 == 0) {
            uint8_t rmin = 0xFF, rmax = 0;
            uint32_t rsum = 0, valid = 0, bad00 = 0, badFF = 0;
            for (int i = 0; i < RF5G_CHANNELS; i++) {
                uint8_t v = s_rssi[i];
                if (v == 0x00) bad00++;
                else if (v == 0xFF) badFF++;
                else {
                    if (v < rmin) rmin = v;
                    if (v > rmax) rmax = v;
                    rsum += v;
                    valid++;
                }
            }
            int avg = valid > 0 ? (int)(rsum / valid) : 0;
            int8_t min_dbm = a5133_rssi_to_dbm(rmin);
            int8_t max_dbm = a5133_rssi_to_dbm(rmax);
            ESP_LOGI(TAG, "sweep#%-4lu %ldms  valid=%3u min=%u(%ddBm) max=%u(%ddBm) avg=%u | 0x00=%3u 0xFF=%3u",
                     (unsigned long)s_sweep_cnt, (long)(frame_us / 1000),
                     valid, rmin, min_dbm, rmax, max_dbm, avg, bad00, badFF);
        }
#endif

        /* ---- JSON OUTPUT for PC DEBUG ----
         * ← 暂时注释掉: 1.2KB printf 占 USB 带宽, 每 10 帧还得 snprintf 256 次 */
#if 0
        if ((s_sweep_cnt % 10) == 0) {
            static char s_json_buf[2048];   /* BSS → 自动 PSRAM; 256ch 需 ~1.2KB */
            int pos = snprintf(s_json_buf, sizeof(s_json_buf),
                "#JSON#{\"n\":%lu,\"t\":%ld,\"c\":%d,\"d\":[",
                (unsigned long)s_sweep_cnt, (long)frame_us, RF5G_CHANNELS);
            for (int i = 0; i < RF5G_CHANNELS && pos < (int)sizeof(s_json_buf) - 16; i++) {
                pos += snprintf(s_json_buf + pos, sizeof(s_json_buf) - pos,
                                "%s%u", (i > 0) ? "," : "", s_rssi[i]);
            }
            snprintf(s_json_buf + pos, sizeof(s_json_buf) - pos, "]}\n");
            printf("%s", s_json_buf);   /* 仅 1 次 USB 阻塞 */
        }
#endif

        /* 错误计数 */
        int bad = 0;
        for (int i = 0; i < RF5G_CHANNELS; i++) {
            if (s_rssi[i] == 0x00 || s_rssi[i] == 0xFF) bad++;
        }
        if (bad >= RF5G_CHANNELS - 50) {
            fail_cnt++;
            if (fail_cnt > 20) {
                ESP_LOGW(TAG, "连续 %d 帧 RSSI 无效, 重新 init A5133", fail_cnt);
                a5133_deinit();
                vTaskDelay(pdMS_TO_TICKS(100));
                if (a5133_init() != ESP_OK) {
                    ESP_LOGE(TAG, "re-init FAILED, 暂停 1s 重试");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                fail_cnt = 0;
            }
        } else {
            fail_cnt = 0;
        }

        /* ← 去掉 vTaskDelay(pdMS_TO_TICKS(5)); 帧间不 sleep, 尽快扫下一帧 */
    }

    /* s_running=false, 优雅退出 */
    esp_task_wdt_delete(NULL);
    s_task_hdl = NULL;
    ESP_LOGI(TAG, "rf_5g task exit (flag)");
    vTaskDelete(NULL);
}

/* ---- 公共 API ---- */

esp_err_t rf_5g_start(int priority)
{
    if (s_task_hdl) { ESP_LOGW(TAG, "task already running"); return ESP_OK; }
    memset(s_rssi, 0, sizeof(s_rssi));
    s_sweep_cnt = 0;
    s_last_tick_us = 0;
    __atomic_store_n(&s_version, 0, __ATOMIC_RELEASE);
    s_running = true;

    BaseType_t ok = xTaskCreatePinnedToCore(rf_5g_task, "rf_5g",
                                            RF5G_TASK_STACK, NULL,
                                            (UBaseType_t)priority, &s_task_hdl,
                                            RF5G_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        s_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void rf_5g_stop(void)
{
    if (!s_task_hdl) return;
    s_running = false;
    /* 等任务自己退出 (扫完这一帧就好, ~50ms) */
    int wait = 100;
    while (s_task_hdl && wait-- > 0) vTaskDelay(pdMS_TO_TICKS(10));
    if (s_task_hdl) {
        ESP_LOGW(TAG, "rf_5g 没在规定时间内退出, 强制 delete");
        vTaskDelete(s_task_hdl);
        s_task_hdl = NULL;
    }
    a5133_deinit();
    ESP_LOGI(TAG, "rf_5g stopped gracefully");
}

/* 无锁读: 最多重试 3 次, 保证 LVGL timer 不阻塞 */
bool rf_5g_get_spectrum(uint8_t *rssi_out, uint32_t *sweep_cnt, uint32_t *age_ms)
{
    if (!s_task_hdl || s_last_tick_us == 0) return false;

    for (int attempt = 0; attempt < 3; attempt++) {
        uint32_t v1 = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
        memcpy(rssi_out, s_rssi, RF5G_CHANNELS);
        uint32_t cnt = s_sweep_cnt;
        int64_t age  = esp_timer_get_time() - s_last_tick_us;
        uint32_t v2 = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
        if (v1 == v2) {
            if (sweep_cnt) *sweep_cnt = cnt;
            if (age_ms)    *age_ms    = (uint32_t)(age / 1000);
            return true;
        }
    }
    /* 重试耗尽, 返回 partial (可能中间态, 但比返回 false 好) */
    uint32_t cnt = s_sweep_cnt;
    int64_t age  = esp_timer_get_time() - s_last_tick_us;
    if (sweep_cnt) *sweep_cnt = cnt;
    if (age_ms)    *age_ms    = (uint32_t)(age / 1000);
    return true;
}

bool rf_5g_is_running(void) { return s_task_hdl != NULL; }






