/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * ESP32-P4 + ST7701S (480x854, MIPI DSI 2-lane) 板级初始化 & LVGL 宿主。
 * 仅保留 RF Monitor 实际使用的代码，剔除示例诊断/红屏测试/GPIO 自检等 demo 代码。
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7701.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_types.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst3530.h"
#include "rf_link.h"
#include "rf_ui.h"
#include "rf_map.h"
#include "rf_5g.h"
#include "rf_gps.h"
#include "nrf24l01.h"
#include "rf_24g.h"

static const char *TAG = "drf_app";

/* =========================================================================
 *  面板固定配置（ST7701S TK050F90H6，2 data lanes，DPI RGB888）
 *   lane_bitrate = 500Mbps, DPI clock = 33MHz, 60Hz 刷新率
 *   所有 EXAMPLE_* 前缀的宏沿用硬件约定，避免重新命名破坏项目常量。
 * ====================================================================== */
#define ST7701_DPI_CLK_MHZ         30     /* 30MHz → ~55Hz (稳定值) */
#define ST7701_H_RES               480
#define ST7701_V_RES               854
#define ST7701_HSYNC               10
#define ST7701_HBP                 30
#define ST7701_HFP                 80
#define ST7701_VSYNC               12
#define ST7701_VBP                 27
#define ST7701_VFP                 20
#define LCD_COLOR_FMT              LCD_COLOR_FMT_RGB565
#define LVGL_COLOR_FMT             LV_COLOR_FORMAT_RGB565
#define LCD_BITS_PER_PIXEL         16

#define MIPI_DSI_LANE_NUM          2
#define MIPI_DSI_LANE_BITRATE_MBPS 500   /* escape_clk_div = round(500/8/18)=4 ∈ [2,255] */

/* D-PHY 2.5V 供电（LDO3 接 VDD_MIPI_DPHY） */
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

/* GPIO：背光 / 复位 / 触摸（项目硬件引脚约定） */
#define PIN_NUM_BK_LIGHT           23
#define PIN_NUM_LCD_RST            16
#define BK_LIGHT_ON_LEVEL          1
#define BK_LIGHT_OFF_LEVEL         !BK_LIGHT_ON_LEVEL

/* 背光 PWM（LP3321 EN 引脚：10kHz，占空比 0% ~ 25% 覆盖用户可感知亮度范围） */
#define BK_LIGHT_LEDC_TIMER        LEDC_TIMER_0
#define BK_LIGHT_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define BK_LIGHT_LEDC_CHANNEL      LEDC_CHANNEL_0
#define BK_LIGHT_LEDC_DUTY_RES     LEDC_TIMER_10_BIT
#define BK_LIGHT_LEDC_FREQ_HZ      10000
#define BK_LIGHT_LEDC_DUTY_MAX     ((1 << 10) - 1)
#define BK_LIGHT_DUTY_MAX_PERCENT  25
#define BK_LIGHT_DUTY_FULL_ON      (BK_LIGHT_LEDC_DUTY_MAX * BK_LIGHT_DUTY_MAX_PERCENT / 100)
#define BK_LIGHT_DUTY_OFF          0

/* 触摸 CST3530（HYN main mode，官方 esp_lcd_touch_cst3530 组件） */
#define PIN_NUM_TOUCH_SCL          47
#define PIN_NUM_TOUCH_SDA          46
#define PIN_NUM_TOUCH_INT          GPIO_NUM_NC
#define PIN_NUM_TOUCH_RST          44
#define TOUCH_I2C_CLK_HZ           100000   /* 内部上拉弱，400kHz 易丢 ACK */

/* LVGL 运行时参数（pin CPU0、短临界区、用 vTaskDelay 让 IDLE 有机会喂狗） */
#define LVGL_DRAW_BUF_LINES        200  /* 200行×480×2=192KB, 平衡局部刷新效率和DSI传输压力 */
#define LVGL_TICK_PERIOD_MS        1                      /* 1ms tick（anim/indev 最小分辨） */
#define LVGL_TASK_STACK_SIZE       (16 * 1024)
#define LVGL_TASK_PRIORITY         4                      /* 高于扫描任务(3), 保证 UI 流畅 */
#define LVGL_TASK_CORE             0                      /* CPU0: 先测 tick fix, 再决定 core 分配 */
#define LVGL_TASK_MIN_DELAY_TICKS  pdMS_TO_TICKS(1)       /* 至少 1 tick，必让同核 IDLE 有机会跑 */
#define LVGL_TASK_MAX_DELAY_MS     8                      /* 8ms ≈ 120Hz 上限，避免 tick-less 长 sleep 卡顿 */

/* ST7701S 2-lane 初始化序列（0xCC=0x11 选 2 data lanes，屏厂提供时序） */
static const st7701_lcd_init_cmd_t st7701_init_cmds[] = {
    {0x01, (uint8_t []){0x00}, 1, 100},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xD1, (uint8_t []){0x11}, 1, 0},
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0xE9, 0x03}, 2, 0},
    {0xC1, (uint8_t []){0x12, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x37, 0x08}, 2, 0},
    {0xB0, (uint8_t []){0x00, 0x0E, 0x15, 0x0F, 0x11, 0x08, 0x08, 0x08, 0x08, 0x23, 0x04, 0x13, 0x12, 0x2B, 0x34, 0x1F}, 16, 0},
    {0xB1, (uint8_t []){0x00, 0x0E, 0x95, 0x0F, 0x13, 0x07, 0x09, 0x08, 0x08, 0x22, 0x04, 0x10, 0x0E, 0x2C, 0x34, 0x1F}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x4D}, 1, 0},
    {0xB1, (uint8_t []){0x13}, 1, 0},
    {0xB2, (uint8_t []){0x07}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x47}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x20}, 1, 0},
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xD0, (uint8_t []){0x88}, 1, 100},
    {0xE0, (uint8_t []){0x00, 0x00, 0x02}, 3, 0},
    {0xE1, (uint8_t []){0x0B, 0x00, 0x0D, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x00, 0x44, 0x44}, 11, 0},
    {0xE2, (uint8_t []){0x33, 0x33, 0x44, 0x44, 0x64, 0x00, 0x66, 0x00, 0x65, 0x00, 0x67, 0x00, 0x00}, 13, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x0C, 0x78, 0x3C, 0xA0, 0x0E, 0x78, 0x3C, 0xA0, 0x10, 0x78, 0x3C, 0xA0, 0x12, 0x78, 0x3C, 0xA0}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x0D, 0x78, 0x3C, 0xA0, 0x0F, 0x78, 0x3C, 0xA0, 0x11, 0x78, 0x3C, 0xA0, 0x13, 0x78, 0x3C, 0xA0}, 16, 0},
    {0xEB, (uint8_t []){0x02, 0x02, 0x39, 0x39, 0xEE, 0x44, 0x00}, 7, 0},
    {0xEC, (uint8_t []){0x00, 0x00}, 2, 0},
    {0xED, (uint8_t []){0xFF, 0xF1, 0x04, 0x56, 0x72, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0x27, 0x65, 0x40, 0x1F, 0xFF}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x36, (uint8_t []){0x00}, 1, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},   /* Sleep Out */
    {0x29, (uint8_t []){0x00}, 0, 120},   /* Display On */
};

#define ALIGN_UP(num, align)    (((num) + ((align) - 1)) & ~((align) - 1))
#define ALIGN_DOWN(num, align)  ((num) & ~((align) - 1))

/* LVGL flush 管线: DPI panel + DMA2D copy (异步)
   — LVGL 光栅化到 PSRAM draw buffer (PARTIAL 模式, 小块渲染 cache 命中率高)
   — flush_cb 调 draw_bitmap, DMA2D 异步拷贝到 DPI framebuffer
   — DMA2D 完成中断 → on_color_trans_done → flush_ready → 通知 LVGL 继续 */

static esp_lcd_touch_handle_t s_touch = NULL;

/* ============ BSP：背光 ============ */
static void bsp_init_backlight(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = BK_LIGHT_LEDC_MODE,
        .duty_resolution = BK_LIGHT_LEDC_DUTY_RES,
        .timer_num       = BK_LIGHT_LEDC_TIMER,
        .freq_hz         = BK_LIGHT_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num   = PIN_NUM_BK_LIGHT,
        .speed_mode = BK_LIGHT_LEDC_MODE,
        .channel    = BK_LIGHT_LEDC_CHANNEL,
        .timer_sel  = BK_LIGHT_LEDC_TIMER,
        .duty       = BK_LIGHT_DUTY_OFF,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    ESP_LOGI(TAG, "BL: PWM GPIO=%d freq=%dHz duty=0 (OFF)", PIN_NUM_BK_LIGHT, BK_LIGHT_LEDC_FREQ_HZ);
}

static void bsp_set_backlight_duty(uint32_t duty)
{
    ledc_set_duty(BK_LIGHT_LEDC_MODE, BK_LIGHT_LEDC_CHANNEL, duty);
    ledc_update_duty(BK_LIGHT_LEDC_MODE, BK_LIGHT_LEDC_CHANNEL);
}

static void bsp_set_backlight(bool on)
{
    bsp_set_backlight_duty(on ? BK_LIGHT_DUTY_FULL_ON : BK_LIGHT_DUTY_OFF);
}

/* 对外导出：背光亮度（占空比 0~100% 映射到 BK_LIGHT_DUTY_MAX_PERCENT） */
void app_backlight_set_percent(uint8_t percent)
{
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    uint32_t duty = (uint32_t)BK_LIGHT_DUTY_FULL_ON * percent / 100;
    bsp_set_backlight_duty(duty);
}

/* ============ BSP：DSI PHY 电源 ============ */
static void bsp_enable_dsi_phy_power(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t cfg = {
        .chan_id    = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    esp_err_t err = esp_ldo_acquire_channel(&cfg, &ldo);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "DSI PHY: LDO%d @ %dmV OK", MIPI_DSI_PHY_PWR_LDO_CHAN, MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV);
    } else {
        ESP_LOGE(TAG, "DSI PHY: LDO acquire FAIL: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(6));
}

/* ============ LVGL flush / tick / 宿主任务 ============ */
volatile uint32_t g_lvgl_flush_cnt = 0;       /* flush 调用次数（PARTIAL 模式下每帧多次） */
volatile uint32_t g_lvgl_frame_cnt = 0;       /* lv_timer_handler 调用次数 ≈ 实际帧数 */
volatile uint32_t g_lvgl_fps = 0;            /* 实际 FPS */
volatile uint32_t g_lvgl_flush_per_sec = 0;  /* 每秒 flush 次数（局部刷新指标） */

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);

    g_lvgl_flush_cnt++;
    /* flush_ready 在 DMA2D 完成中断 (on_color_trans_done) 里异步调用 */
}

static bool lvgl_flush_ready_cb(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    LV_UNUSED(panel);
    LV_UNUSED(edata);
    lv_display_t *d = (lv_display_t *)user_ctx;
    /* DMA2D 拷贝完成 → 通知 LVGL 可以复用 draw buffer */
    if (d) lv_display_flush_ready(d);
    return false;
}

static uint32_t lvgl_tick_get_cb(void)
{
    /* LVGL 9.4: SMP-safe tick — 直接读 esp_timer, 不经过 lv_tick_inc 的跨核自旋 */
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void lvgl_port_task(void *arg)
{
    LV_UNUSED(arg);
    ESP_LOGI(TAG, "LVGL task start (prio=%d core=%d stack=%d buf_lines=%d)",
             LVGL_TASK_PRIORITY, LVGL_TASK_CORE, LVGL_TASK_STACK_SIZE, LVGL_DRAW_BUF_LINES);

    uint32_t last_log_tick = xTaskGetTickCount();
    uint32_t last_frame_cnt = 0;
    uint32_t last_flush_cnt = 0;

    for (;;) {
        uint32_t flush_before = g_lvgl_flush_cnt;

        int32_t next_ms = lv_timer_handler();

        /* 只有本帧实际发生了 flush（有像素推送到屏幕）才算一帧 */
        if (g_lvgl_flush_cnt != flush_before) {
            g_lvgl_frame_cnt++;
        }

        if (next_ms <= 0) next_ms = 100;
        vTaskDelay(pdMS_TO_TICKS(next_ms));

        /* 每 1000ms 统计一次 FPS */
        uint32_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_log_tick) >= pdMS_TO_TICKS(1000)) {
            uint32_t elapsed = (now_tick - last_log_tick) * portTICK_PERIOD_MS;
            uint32_t frame_delta = g_lvgl_frame_cnt - last_frame_cnt;
            uint32_t fps = frame_delta * 1000 / (elapsed ? elapsed : 1);
            uint32_t flush_delta = g_lvgl_flush_cnt - last_flush_cnt;
            uint32_t flush_ps = flush_delta * 1000 / (elapsed ? elapsed : 1);

            g_lvgl_fps = fps;
            g_lvgl_flush_per_sec = flush_ps;

            last_frame_cnt = g_lvgl_frame_cnt;
            last_flush_cnt = g_lvgl_flush_cnt;
            last_log_tick = now_tick;
        }
    }
}

/* SD 卡 + 瓦片地图后台初始化任务（卡没插时不应阻塞 UI 启动） */
extern void rf_ui_on_map_ready(void);
static void rf_sd_init_task(void *arg)
{
    LV_UNUSED(arg);
    esp_err_t e = rf_map_init();
    if (e == ESP_OK) {
        rf_ui_on_map_ready();
    } else {
        ESP_LOGW(TAG, "SD mount failed (no card / no tiles dir): %s", esp_err_to_name(e));
    }
    vTaskDelete(NULL);
}

/* ============ LVGL 触摸读回调（CST3530 轮询） ============ */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    LV_UNUSED(indev);
    esp_lcd_touch_read_data(s_touch);
    uint16_t x[1], y[1], strength[1];
    uint8_t n = 0;
    if (esp_lcd_touch_get_coordinates(s_touch, x, y, strength, &n, 1) && n > 0) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* ============ 板级初始化 & LVGL ============ */
static esp_lcd_panel_handle_t init_st7701s_panel(esp_lcd_dsi_bus_handle_t bus,
                                                 esp_lcd_panel_io_handle_t dbi_io)
{
    /* DPI video 面板配置（RGB888，双帧缓冲，与屏厂 DPI 时序对齐） */
    esp_lcd_dpi_panel_config_t dpi = {
        .virtual_channel     = 0,
        .dpi_clk_src         = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz  = ST7701_DPI_CLK_MHZ,
        .pixel_format        = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .in_color_format     = LCD_COLOR_FMT,
        .out_color_format    = LCD_COLOR_FMT,
        .num_fbs             = 2,
        .video_timing = {
            .h_size            = ST7701_H_RES,
            .v_size            = ST7701_V_RES,
            .hsync_back_porch  = ST7701_HBP,
            .hsync_pulse_width = ST7701_HSYNC,
            .hsync_front_porch = ST7701_HFP,
            .vsync_back_porch  = ST7701_VBP,
            .vsync_pulse_width = ST7701_VSYNC,
            .vsync_front_porch = ST7701_VFP,
        },
#if CONFIG_EXAMPLE_USE_DMA2D_COPY_FRAME
        .flags = { .use_dma2d = true },
#endif
    };

    st7701_vendor_config_t vendor = {
        .mipi_config = { .dsi_bus = bus, .dpi_config = &dpi },
        .init_cmds         = st7701_init_cmds,
        .init_cmds_size    = sizeof(st7701_init_cmds) / sizeof(st7701_init_cmds[0]),
        .flags             = { .use_mipi_interface = 1 },
    };

    esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num  = PIN_NUM_LCD_RST,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = LCD_BITS_PER_PIXEL,
        .vendor_config   = &vendor,
    };

    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(dbi_io, &dev_cfg, &panel));
    return panel;
}

/* RST 硬复位（RST 低 ≥10ms，释放后 ≥120ms） */
static void panel_hw_reset(void)
{
    gpio_config_t rst = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_LCD_RST,
    };
    ESP_ERROR_CHECK(gpio_config(&rst));
    gpio_set_level(PIN_NUM_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(12));
    gpio_set_level(PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static lv_display_t *init_lvgl(esp_lcd_panel_handle_t panel)
{
    lv_init();
    lv_display_t *d = lv_display_create(ST7701_H_RES, ST7701_V_RES);
    lv_display_set_user_data(d, panel);
    lv_display_set_color_format(d, LVGL_COLOR_FMT);

    /* PARTIAL 模式: LVGL 光栅化到 PSRAM draw buffer, flush_cb DMA2D 拷贝到 DPI FB
       200 行 = 每帧约 4-5 次 flush, 平衡 cache 命中率和 flush 开销 */
    const size_t draw_sz = (size_t)ST7701_H_RES * LVGL_DRAW_BUF_LINES * LCD_BITS_PER_PIXEL / 8;
    void *b1 = heap_caps_aligned_calloc(64, 1, draw_sz, MALLOC_CAP_SPIRAM);
    void *b2 = heap_caps_aligned_calloc(64, 1, draw_sz, MALLOC_CAP_SPIRAM);
    assert(b1 && b2);
    ESP_LOGI(TAG, "LVGL: PARTIAL mode, %dx%d %dpx @ %dbpp, draw_buf=%zu bytes (lines=%d)",
             ST7701_H_RES, ST7701_V_RES, ST7701_H_RES*ST7701_V_RES, LCD_BITS_PER_PIXEL, draw_sz, LVGL_DRAW_BUF_LINES);

    lv_display_set_buffers(d, b1, b2, draw_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(d, lvgl_flush_cb);

    /* DMA2D 完成中断 → on_color_trans_done → flush_ready */
    esp_lcd_dpi_panel_event_callbacks_t cbs = { .on_color_trans_done = lvgl_flush_ready_cb };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel, &cbs, d));

    /* SMP-safe tick */
    lv_tick_set_cb(lvgl_tick_get_cb);

    BaseType_t core_ok = xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE,
                                                 NULL, LVGL_TASK_PRIORITY, NULL, LVGL_TASK_CORE);
    if (core_ok != pdPASS) {
        ESP_LOGW(TAG, "xTaskCreatePinnedToCore(core=%d) failed, fallback to xTaskCreate", LVGL_TASK_CORE);
        xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    }
    return d;
}

static void init_touch_cst3530(lv_display_t *disp)
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_panel_io_handle_t tp_io = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port               = I2C_NUM_0,
        .sda_io_num             = PIN_NUM_TOUCH_SDA,
        .scl_io_num             = PIN_NUM_TOUCH_SCL,
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt      = 7,
        .flags                  = { .enable_internal_pullup = true },
    };
    esp_err_t e = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (e != ESP_OK) { ESP_LOGE(TAG, "touch: i2c_new_master_bus failed: %s", esp_err_to_name(e)); return; }

    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_CST3530_CONFIG();
    tp_io_cfg.scl_speed_hz = TOUCH_I2C_CLK_HZ;
    tp_io_cfg.user_ctx    = i2c_bus;   /* HYN 层内部裸 I2C 需要 */
    e = esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_cfg, &tp_io);
    if (e != ESP_OK) { ESP_LOGE(TAG, "touch: new_panel_io_i2c failed: %s", esp_err_to_name(e)); return; }

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = ST7701_H_RES,
        .y_max = ST7701_V_RES,
        .rst_gpio_num = PIN_NUM_TOUCH_RST,
        .int_gpio_num = PIN_NUM_TOUCH_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags  = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
    };
    esp_lcd_touch_handle_t tp = NULL;
    e = esp_lcd_touch_new_i2c_cst3530(tp_io, &tp_cfg, &tp);
    if (e != ESP_OK) { ESP_LOGE(TAG, "touch: CST3530 init failed: %s", esp_err_to_name(e)); return; }
    s_touch = tp;

    /* NO LOCK: LVGL task 尚未启动, app_main 单线程里 init touch 安全 */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, disp);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);
    /* 缩短长按/手势判定，提高触摸反馈灵敏度 */
    lv_indev_set_long_press_time(indev, 300);
    lv_indev_set_long_press_repeat_time(indev, 80);

    ESP_LOGI(TAG, "CST3530 ready (SCL=%d SDA=%d RST=%d I2C=%dkHz)",
             PIN_NUM_TOUCH_SCL, PIN_NUM_TOUCH_SDA, PIN_NUM_TOUCH_RST, TOUCH_I2C_CLK_HZ / 1000);
}

/* ============ app_main ============ */
void app_main(void)
{
    ESP_LOGI(TAG, "DRF boot: panel=%dx%d RGB888 lanes=%d@%dMbps DPI=%dMHz",
             ST7701_H_RES, ST7701_V_RES, MIPI_DSI_LANE_NUM, MIPI_DSI_LANE_BITRATE_MBPS, ST7701_DPI_CLK_MHZ);

    /* 1) DSI PHY 2.5V + 背光 PWM 硬件 */
    bsp_enable_dsi_phy_power();
    bsp_init_backlight();

    /* 2) DSI bus + DBI IO（命令通道） */
    esp_lcd_dsi_bus_handle_t mipi_bus;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = MIPI_DSI_LANE_NUM,
        .lane_bit_rate_mbps = MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &mipi_bus));

    esp_lcd_panel_io_handle_t dbi_io;
    esp_lcd_dbi_io_config_t dbi = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_bus, &dbi, &dbi_io));

    /* 3) RST 硬复位 — 面板进入稳定状态（命令接口可用） */
    panel_hw_reset();

    /* 4) 面板驱动 + Init（发送 ST7701S 2-lane 初始化序列） */
    esp_lcd_panel_handle_t panel = init_st7701s_panel(mipi_bus, dbi_io);
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

    /* 5) LVGL（宿主任务） */
    lv_display_t *disp = init_lvgl(panel);

    /* 6) CST3530 触摸（I2C，挂 LVGL 输入设备） */
    init_touch_cst3530(disp);

    /* 7) 背光点亮（面板视频流已起，避免亮屏瞬间黑屏） */
    bsp_set_backlight(true);
    ESP_LOGI(TAG, "backlight ON duty=%d/%d (%d%% of max range)",
             BK_LIGHT_DUTY_FULL_ON, BK_LIGHT_LEDC_DUTY_MAX, BK_LIGHT_DUTY_MAX_PERCENT);

    /* 8) 构建 RF Monitor UI (优先显示开机自检页)
       LVGL 还没开始跑 lv_timer_handler, app_main 单线程里创建 UI 安全 (不需要锁) */
    rf_ui_create(disp);

    /* 9) RF 链路（UART 接 ESP32-C5）后台启动 */
    ESP_ERROR_CHECK(rf_link_start());

    /* 10) SD 卡 + 瓦片地图 (后台任务, 不阻塞 UI) */
    {
        TaskHandle_t sd_hdl = NULL;
        xTaskCreatePinnedToCore(rf_sd_init_task, "rf_sd", 4096, NULL, 1, &sd_hdl, 0);
    }

    /* 11) GPS (DX-GP22-A UART2) — 可选，低优先级后台运行 */
    {
        esp_err_t e = rf_gps_start(1);  /* prio 1, 不抢占 UI */
        if (e == ESP_OK) ESP_LOGI(TAG, "GPS DX-GP22-A started (prio 1, UART2 TX=6 RX=7 baud=9600)");
        else             ESP_LOGW(TAG, "GPS start FAILED: %s (skipping, GPS optional)", esp_err_to_name(e));
    }

    /* 12) ★ 交付级待机策略: 2.4G / 5.8G 扫频任务不主动启动
       — 用户进入 Spectrum 页时由 page_show() 按需启动 (rf_24g_start / rf_5g_start)
       — 主页时完全待机, 节省 CPU / 功耗 / 减少干扰
       — nRF24L01 硬件初始化也延迟到首次进入 2.4G 页时做 (减少启动时间) */
    {
        static const nrf_pin_config_t s_nrf_pins = {
            .mosi     = 5, .miso = 4, .sclk = 6, .csn = 2, .ce = 7,
            .spi_host = SPI2_HOST,
        };
        esp_err_t e1 = nrf24l01_init(&s_nrf_pins);
        if (e1 != ESP_OK) {
            ESP_LOGW(TAG, "NRF24L01 init FAILED: %s (2.4G fallback UART)", esp_err_to_name(e1));
        } else {
            ESP_LOGI(TAG, "NRF24L01 init OK (standby, will start on Spectrum page)");
        }
    }

    ESP_LOGI(TAG, "DRF ready — boot self-test running, RF scanners in standby");
}













