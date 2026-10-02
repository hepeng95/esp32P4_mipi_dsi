/*
 * rf_24g.c — nRF24L01 2.4G 扫描任务
 *
 * 无锁 atomic 模式, 和 rf_5g 完全一致:
 *   Producer 写 s_rssi[] → __atomic_store_n(&s_version, v+1, release)
 *   Consumer 读 version_old → memcpy → 读 version_new (acquire), 变了重试
 */

#include "rf_24g.h"
#include "nrf24l01.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <stdatomic.h>

static const char *TAG = "rf_24g";

/* ====== 通道 & 采样参数 ======
 * ★ NRF24L01 RPD 只是阈值检测 (PWR > -64dBm → 1), 非真正 RSSI
 *   3 samples × 100µs settle → 37.8ms/帧 ≈ 26 FPS (原 160ms → 6 FPS)
 *   hits 0~3 × 60 → 0/60/120/180 四档 (频谱显示足够) */
#define RF2G_CHANNELS       126     /* ch0~125 → 2400~2525 MHz */
#define RF2G_SAMPLES_PER_CH 3       /* 3 samples → hits 0~3 → ×60 得 4 档灰度 */
#define RF2G_SETTLE_US      100     /* nRF24L01 datasheet: RPD settling = 70µs, 取 100µs */

/* ====== 任务参数 ====== */
#define RF2G_TASK_STACK     4096
#define RF2G_TASK_PRIORITY  3
#define RF2G_TASK_CORE      1      /* ★ CPU1 — 不抢占 LVGL */

/* ====== 无锁共享缓冲 ====== */
static TaskHandle_t  s_task_hdl      = NULL;
static volatile uint32_t s_version   = 0;
static uint8_t       s_rssi[RF2G_CHANNELS];
static uint32_t      s_sweep_cnt     = 0;
static int64_t       s_last_tick_us  = 0;
static bool          s_running       = false;

/* 写侧: 发布新帧 */
static void publish_frame(void)
{
    s_sweep_cnt++;
    s_last_tick_us = esp_timer_get_time();
    __atomic_fetch_add(&s_version, 1, __ATOMIC_RELEASE);
}

/* 扫描主循环 */
static void rf_24g_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "nRF24L01 + 2.4G scan task start (%d ch, %dx samples/ch)",
             RF2G_CHANNELS, RF2G_SAMPLES_PER_CH);

    /* 启动后挂起 100ms — 让 app_main return 变成 IDLE */
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_task_wdt_add(NULL);   /* 订阅 WDT, 自己喂狗 */

    int64_t t0 = esp_timer_get_time();

    while (s_running) {
        /* ====== 极速扫 126 通道 ======
         * ★ IDLE watchdog 已关 (sdkconfig), 全速扫描!
         *   扫描 126×2×200µs=50ms/帧 ≈ 20Hz
         * ★ 每 16 通道 vTaskDelay(0) — 让高优先级 LVGL(prio 4) 跑 */
        for (int ch = 0; ch < RF2G_CHANNELS; ch++) {
            int hits = 0;
            for (int s = 0; s < RF2G_SAMPLES_PER_CH; s++) {
                if (nrf_detect_channel((uint8_t)ch, RF2G_SETTLE_US)) hits++;
            }
            /* ★ RPD 阈值检测: hits*60 → 0/60/120/180 四档 (3 samples) */
            s_rssi[ch] = (uint8_t)(hits * 60);
            /* 无 vTaskDelay — scan 在 CPU1, LVGL 在 CPU0, 不抢占 */
        }

        /* ====== 发布 ====== */
        publish_frame();
        esp_task_wdt_reset();   /* 喂狗 (5s timeout, 帧间 ≈58ms) */

        /* ---- DEBUG: 每 20 帧统计 ---- */
        if ((s_sweep_cnt % 20) == 0) {
            int nonzero = 0, max = 0, ch_max = 0;
            for (int i = 0; i < RF2G_CHANNELS; i++) {
                if (s_rssi[i] > 0) { nonzero++; if (s_rssi[i] > max) { max = s_rssi[i]; ch_max = i; } }
            }
            ESP_LOGI(TAG, "sweep#%-4lu non-zero=%d max=%u@ch%d  sample[32..37]=%u %u %u %u %u %u",
                     (unsigned long)s_sweep_cnt, nonzero, max, ch_max,
                     s_rssi[32], s_rssi[33], s_rssi[34], s_rssi[35], s_rssi[36], s_rssi[37]);
        }
    }

    /* s_running=false, 优雅退出 */
    esp_task_wdt_delete(NULL);
    s_task_hdl = NULL;
    ESP_LOGI(TAG, "rf_24g task exit (flag)");
    vTaskDelete(NULL);
}

/* ====== 对外 API ====== */

esp_err_t rf_24g_start(int priority)
{
    if (s_task_hdl) {
        ESP_LOGW(TAG, "扫描任务已在运行");
        return ESP_OK;
    }
    if (!nrf24l01_init_done()) {
        ESP_LOGE(TAG, "nRF24L01 未初始化, 请先调用 nrf24l01_init()");
        return ESP_ERR_INVALID_STATE;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(rf_24g_task, "rf_24g", RF2G_TASK_STACK, NULL,
                               (UBaseType_t)priority, &s_task_hdl, RF2G_TASK_CORE);
    if (!ok) {
        ESP_LOGE(TAG, "创建扫描任务失败");
        return ESP_ERR_NO_MEM;
    }
    s_running = true;
    ESP_LOGI(TAG, "rf_24g task started (prio %d)", priority);
    return ESP_OK;
}

void rf_24g_stop(void)
{
    s_running = false;
    /* 等任务自己退出 */
    int wait = 100;
    while (s_task_hdl && wait-- > 0) vTaskDelay(pdMS_TO_TICKS(10));
    if (s_task_hdl) {
        ESP_LOGW(TAG, "rf_24g 没及时退出, 强制 delete");
        vTaskDelete(s_task_hdl);
        s_task_hdl = NULL;
    }
    ESP_LOGI(TAG, "rf_24g stopped gracefully");
}

bool rf_24g_is_running(void)
{
    return s_running && s_task_hdl;
}

bool rf_24g_get_spectrum(uint8_t *rssi_out, uint32_t *sweep_cnt_out, uint32_t *age_ms_out)
{
    if (!rssi_out || !sweep_cnt_out || !age_ms_out) return false;

    for (int retry = 0; retry < 3; retry++) {
        uint32_t v_old = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
        memcpy(rssi_out, s_rssi, RF2G_CHANNELS);
        uint32_t v_new = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
        if (v_old == v_new) {
            *sweep_cnt_out = s_sweep_cnt;
            *age_ms_out = (uint32_t)((esp_timer_get_time() - s_last_tick_us) / 1000);
            return true;
        }
    }
    return false;   /* 3 次都遇到写侧正在更新, 放弃本帧 */
}


