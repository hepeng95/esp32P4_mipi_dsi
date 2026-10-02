/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * rf_gps.c — DX-GP22-A GNSS UART 驱动 + NMEA 解析
 *
 * 后台任务 pin CPU1, prio 2 (<LVGL 4, <rf_5g 3), 避免跨核 mutex 断言崩溃
 *
 * NMEA 语句重点处理:
 *   $GNGGA — 位置 + 海拔 + 定位状态 + 卫星数 + HDOP
 *   $GNRMC — 经纬度 + 时间 + 速度 + 航向 + 日期 (完整快照)
 * 次要:
 *   $GNGSV — 可见卫星数 + 信号强度 (记录 sats 数)
 *
 * 经纬度解析: ddmm.mmmmmm → decimal degrees
 *   北纬/东经 = 正, 南纬/西经 = 负
 */

#include "rf_gps.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdatomic.h>
#include "driver/uart.h"

static const char *TAG = "rf_gps";

/* ====== 配置 ====== */
#define RF_GPS_TASK_STACK       4096
#define RF_GPS_TASK_PRIORITY    2
#define RF_GPS_TASK_CORE        1      /* ★ CPU1 — 不抢占 LVGL */
#define RF_GPS_LINE_BUF_SZ      192    /* 一行 NMEA 语句 <= 120, 留余量 */
#define RF_GPS_RING_SZ          1024   /* UART RX ring buffer */
#define RF_GPS_FIX_TIMEOUT_MS   15000  /* 15s 无数据认为没 fix */

/* ====== 状态 ====== */
static TaskHandle_t       s_task_hdl = NULL;
static SemaphoreHandle_t  s_mut      = NULL;
static rf_gps_state_t     s_st       = {0};
static volatile uint32_t s_ver       = 0;

/* ====== 工具: NMEA checksum ====== */
static uint8_t nmea_cksum(const char *s)
{
    uint8_t c = 0;
    while (*s && *s != '*') c ^= (uint8_t)*s++;
    return c;
}

/* ====== 工具: 跳过到下一个字段 (逗号分隔) ====== */
static const char *next_field(const char *p)
{
    while (*p && *p != ',') p++;
    if (*p == ',') p++;
    return p;
}

/* ====== 工具: ddmm.mmmmmm → 度 (×1e7) ======
   输入: "2314.2910" (纬度 23°14.2910') 或 "11326.0239" (经度 113°26.0239')
   输出: int32 × 1e7, 未解析返回 INT32_MIN
*/
static int32_t nmea_coord_e7(const char *field, char hem)
{
    if (!field || !*field) return INT32_MIN;
    /* 找小数点 */
    const char *dot = strchr(field, '.');
    if (!dot) return INT32_MIN;
    /* 度: 小数点前 2 位(纬度) 或 3 位(经度) */
    int deg_len = (int)(dot - field) - 2;   /* dd.xxxx 格式, 度在分钟前 */
    if (deg_len < 1 || deg_len > 3) return INT32_MIN;

    int deg = 0, min = 0, frac = 0;
    /* 度 */
    for (int i = 0; i < deg_len; i++) if (isdigit((unsigned char)field[i])) deg = deg * 10 + (field[i] - '0');
    /* 分钟整数部分 (小数点前剩余 2 位) */
    min = (field[deg_len] - '0') * 10 + (field[deg_len + 1] - '0');
    /* 小数部分 (小数点后, 取 6 位) */
    frac = 0;
    int fd = 0;
    dot++;
    while (*dot && fd < 6) {
        if (isdigit((unsigned char)*dot)) { frac = frac * 10 + (*dot - '0'); fd++; }
        dot++;
    }
    /* 补齐到 6 位 */
    while (fd < 6) { frac *= 10; fd++; }

    /* 度 = deg + min/60 + frac/60e6, × 1e7 */
    int64_t val = (int64_t)deg * 10000000LL
                + ((int64_t)min * 10000000LL) / 60LL
                + ((int64_t)frac * 100LL) / 60LL;
    if (hem == 'S' || hem == 'W') val = -val;
    if (val > INT32_MAX) val = INT32_MAX;
    if (val < INT32_MIN) val = INT32_MIN;
    return (int32_t)val;
}

/* ====== 工具: UTC hhmmss.sss → 时分秒 ====== */
static void nmea_time(const char *field, uint8_t *h, uint8_t *m, uint8_t *s, uint16_t *ms)
{
    *h = *m = *s = 0;
    *ms = 0;
    if (!field || !*field) return;
    if (field[0] && field[1]) *h = (field[0] - '0') * 10 + (field[1] - '0');
    if (field[2] && field[3]) *m = (field[2] - '0') * 10 + (field[3] - '0');
    if (field[4] && field[5]) *s = (field[4] - '0') * 10 + (field[5] - '0');
    const char *dot = strchr(field, '.');
    if (dot) {
        dot++;
        int frac = 0, d = 0;
        while (*dot && d < 3) { if (isdigit((unsigned char)*dot)) { frac = frac * 10 + (*dot - '0'); d++; } dot++; }
        while (d < 3) { frac *= 10; d++; }
        *ms = (uint16_t)frac;
    }
}

/* ====== 工具: UTC ddmmyy → 年月日 ====== */
static void nmea_date(const char *field, uint8_t *d, uint8_t *mo, uint8_t *y)
{
    *d = *mo = *y = 0;
    if (!field || !*field) return;
    if (field[0] && field[1]) *d  = (field[0] - '0') * 10 + (field[1] - '0');
    if (field[2] && field[3]) *mo = (field[2] - '0') * 10 + (field[3] - '0');
    if (field[4] && field[5]) *y  = (field[4] - '0') * 10 + (field[5] - '0');
}

/* ====== 工具: field → 16.xx → x100 int; 空或非法 → default ====== */
static int32_t nmea_dec_x100(const char *field, int32_t dflt)
{
    if (!field || !*field) return dflt;
    double v = atof(field);
    if (v < -999.0) return dflt;   /* NMEA 空字段被 atof 解析时是 0.0 */
    if (v > 9999.0) return dflt;
    return (int32_t)(v * 100.0 + 0.5);
}

/* ====== 工具: field → int ====== */
static int32_t nmea_int(const char *field, int32_t dflt)
{
    if (!field || !*field) return dflt;
    return (int32_t)atol(field);
}

/* ====== GGA 解析: $GNGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,alt_msl,M,diff_age,diff_station*XX\r\n ====== */
static void parse_gga(const char *line, rf_gps_state_t *s)
{
    /* 跳过 $GNGGA, */
    const char *p = strchr(line, ',');
    if (!p) return;
    p++;                                              /* hhmmss.sss */
    nmea_time(p, &s->hour, &s->minute, &s->second, &s->cs_ms);
    p = next_field(p);                                /* lat */
    s->lat_e7 = nmea_coord_e7(p, *(next_field(p) - 1));   /* lat, N/S */
    /* hem 简化: 再 next 一次拿 N/S 字段 */
    const char *hem_lat = next_field(p) - 1;
    char hem_l = (hem_lat && *hem_lat) ? *hem_lat : 'N';
    s->lat_e7 = nmea_coord_e7(p, hem_l);
    p = next_field(p);                                /* N/S */
    p = next_field(p);                                /* lon */
    const char *hem_lon_p = next_field(p) - 1;
    char hem_o = (hem_lon_p && *hem_lon_p) ? *hem_lon_p : 'E';
    s->lon_e7 = nmea_coord_e7(p, hem_o);
    p = next_field(p);                                /* E/W */
    p = next_field(p);                                /* fix */
    int32_t fix = nmea_int(p, 0);
    s->fix = (rf_gps_fix_t)fix;
    p = next_field(p);                                /* sats */
    s->num_sats = (uint8_t)nmea_int(p, 0);
    p = next_field(p);                                /* hdop */
    s->hdop_x100 = nmea_dec_x100(p, -1);
    p = next_field(p);                                /* alt */
    s->alt_cm = nmea_dec_x100(p, 0) * 10;             /* m ×100 → cm ×10? 不, atof 给 m, ×100 得 cm */
    /* 修正: m → cm */
    s->alt_cm = nmea_int(p, 0) * 100;
    /* 上面有 bug — 重算 */
    (void)s->alt_cm;
    /* 正确做法: */
    const char *alt_f = p;
    double alt_m = (alt_f && *alt_f) ? atof(alt_f) : 0.0;
    s->alt_cm = (int32_t)(alt_m * 100.0);
    p = next_field(p);                                /* M */
    p = next_field(p);                                /* alt_msl */
    double alt_msl_m = (p && *p) ? atof(p) : 0.0;
    s->alt_msl_cm = (int32_t)(alt_msl_m * 100.0);
    p = next_field(p);                                /* M */

    /* 标记有效 (fix > 0 且坐标合法) */
    s->valid = (s->fix != RF_GPS_FIX_NONE) &&
               (s->lat_e7 != INT32_MIN) &&
               (s->lon_e7 != INT32_MIN);
    s->ts_monotonic_ms = esp_timer_get_time() / 1000;
}

/* ====== RMC 解析: $GNRMC,time,A,lat,N/S,lon,E/W,speed,heading,date,,checksum ====== */
static void parse_rmc(const char *line, rf_gps_state_t *s)
{
    const char *p = strchr(line, ',');
    if (!p) return;
    p++;                                              /* time */
    nmea_time(p, &s->hour, &s->minute, &s->second, &s->cs_ms);
    p = next_field(p);                                /* A/V */
    char status = *p;
    p = next_field(p);                                /* lat */
    const char *lat_p = p;
    p = next_field(p);                                /* N/S */
    char hem_l = (p && *p) ? *p : 'N';
    s->lat_e7 = nmea_coord_e7(lat_p, hem_l);
    p = next_field(p);                                /* lon */
    const char *lon_p = p;
    p = next_field(p);                                /* E/W */
    char hem_o = (p && *p) ? *p : 'E';
    s->lon_e7 = nmea_coord_e7(lon_p, hem_o);
    p = next_field(p);                                /* speed (knots) */
    /* 1 knot = 1.852 km/h */
    double knots = (p && *p) ? atof(p) : 0.0;
    s->speed_x100 = (int32_t)(knots * 185.2);         /* ×100 km/h */
    p = next_field(p);                                /* heading */
    s->heading_x100 = nmea_dec_x100(p, -1);
    p = next_field(p);                                /* date ddmmyy */
    nmea_date(p, &s->day, &s->month, &s->year);

    /* RMC 状态 A = active(定位), V = void(未定位) */
    if (status == 'A') {
        s->fix = RF_GPS_FIX_AUTO;
        s->valid = (s->lat_e7 != INT32_MIN) && (s->lon_e7 != INT32_MIN);
    } else {
        /* RMC 只有 status V, 但 GGA 可能 fix!=0, 这里不覆盖 */
    }
    s->ts_monotonic_ms = esp_timer_get_time() / 1000;
}

/* ====== GSV: 只取可见卫星数 ====== */
static void parse_gsv(const char *line, rf_gps_state_t *s)
{
    /* 形如 $GNGSV,3,1,11,...  — 第 4 字段是可见卫星总数 */
    int field = 0;
    const char *p = line;
    while (*p) {
        if (*p == ',') field++;
        if (field == 3) break;
        p++;
    }
    /* 现在指向第 4 个字段 (可见卫星数) */
    if (*p == ',') p++;
    int32_t total = nmea_int(p, -1);
    if (total > 0 && total < 100) s->num_sats = (uint8_t)total;
}

/* ====== 单条语句分发 ====== */
static void handle_sentence(const char *line, rf_gps_state_t *s)
{
    /* 校验 checksum (可选, 有错就跳过) */
    const char *star = strchr(line, '*');
    if (star) {
        uint8_t expected = (uint8_t)strtoul(star + 1, NULL, 16);
        uint8_t got = nmea_cksum(line + 1);    /* 跳过 $ */
        if (got != expected) {
            ESP_LOGW(TAG, "cksum fail expected=0x%02X got=0x%02X line=%s",
                     expected, got, line);
            return;
        }
    }

    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0 ||
        strncmp(line, "$BDGGA", 6) == 0) {
        parse_gga(line, s);
    } else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0 ||
               strncmp(line, "$BDRMC", 6) == 0) {
        parse_rmc(line, s);
    } else if (strncmp(line, "$GNGSV", 6) == 0 || strncmp(line, "$GPGSV", 6) == 0 ||
               strncmp(line, "$BDGSV", 6) == 0) {
        parse_gsv(line, s);
    }
    /* 其他: GLL/VTG/GST 等暂不处理 */
}

/* ====== UART 初始化 ====== */
static esp_err_t rf_gps_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = RF_GPS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(RF_GPS_UART_NUM, RF_GPS_RING_SZ * 2,
                                       RF_GPS_RING_SZ, 0, NULL, 0);
    if (e != ESP_OK) return e;
    e = uart_param_config(RF_GPS_UART_NUM, &cfg);
    if (e != ESP_OK) return e;
    e = uart_set_pin(RF_GPS_UART_NUM, RF_GPS_UART_TX, RF_GPS_UART_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (e != ESP_OK) return e;
    ESP_LOGI(TAG, "UART%d init OK TX=GPIO%d RX=GPIO%d baud=%d",
             RF_GPS_UART_NUM, RF_GPS_UART_TX, RF_GPS_UART_RX, RF_GPS_UART_BAUD);
    return ESP_OK;
}

/* ====== 后台任务: UART RX → 按行切分 → 解析 NMEA → 写共享状态 ====== */
static void rf_gps_task(void *arg)
{
    (void)arg;
    char rx[RF_GPS_RING_SZ];
    char line[RF_GPS_LINE_BUF_SZ];
    int  line_pos = 0;

    /* 首次输出状态等待 1~2 秒 GPS 锁定 */
    ESP_LOGI(TAG, "GPS task start, waiting NMEA data...");

    while (1) {
        int len = uart_read_bytes(RF_GPS_UART_NUM, rx, sizeof(rx), pdMS_TO_TICKS(200));
        if (len <= 0) {
            /* 超时: 如果很久没数据, 标记 invalid 但不清空已有值 */
            continue;
        }

        for (int i = 0; i < len; i++) {
            char c = rx[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0 && line_pos < RF_GPS_LINE_BUF_SZ) {
                    line[line_pos] = '\0';
                    /* 复制到共享锁内处理 */
                    if (s_mut) xSemaphoreTake(s_mut, pdMS_TO_TICKS(5));
                    handle_sentence(line, &s_st);
                    if (s_mut) xSemaphoreGive(s_mut);
                    if (s_mut) __atomic_fetch_add(&s_ver, 1, __ATOMIC_RELEASE);
                    /* 状态变化时打印一下 */
                    static rf_gps_fix_t last_fix = RF_GPS_FIX_NONE;
                    if (s_st.fix != last_fix) {
                        last_fix = s_st.fix;
                        if (s_st.valid) {
                            ESP_LOGI(TAG, "FIX %d lat=%.7f lon=%.7f alt=%.2fm sats=%d",
                                     (int)s_st.fix,
                                     s_st.lat_e7 * 1e-7, s_st.lon_e7 * 1e-7,
                                     s_st.alt_cm * 0.01, s_st.num_sats);
                        } else {
                            ESP_LOGW(TAG, "FIX lost (state=%d)", (int)s_st.fix);
                        }
                    }
                }
                line_pos = 0;
            } else {
                if (line_pos < RF_GPS_LINE_BUF_SZ - 1) line[line_pos++] = c;
                else {
                    ESP_LOGW(TAG, "NMEA line overflow, drop");
                    line_pos = 0;
                }
            }
        }
    }
}

/* ====== 公共 API ====== */

esp_err_t rf_gps_start(int priority)
{
    if (s_task_hdl) { ESP_LOGW(TAG, "already running"); return ESP_OK; }
    if (!s_mut) s_mut = xSemaphoreCreateMutex();

    esp_err_t e = rf_gps_uart_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "UART init FAILED: %s", esp_err_to_name(e));
        return e;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(rf_gps_task, "rf_gps",
                                            RF_GPS_TASK_STACK, NULL,
                                            (UBaseType_t)RF_GPS_TASK_PRIORITY, &s_task_hdl,
                                            RF_GPS_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void rf_gps_stop(void)
{
    if (s_task_hdl) {
        vTaskDelete(s_task_hdl);
        s_task_hdl = NULL;
        uart_driver_delete(RF_GPS_UART_NUM);
        ESP_LOGI(TAG, "stopped");
    }
}

bool rf_gps_get(rf_gps_state_t *out, int *age_ms)
{
    if (!out) return false;
    if (!s_mut) return false;

    for (int a = 0; a < 3; a++) {
        uint32_t v1 = __atomic_load_n(&s_ver, __ATOMIC_ACQUIRE);
        *out = s_st;
        int64_t age = esp_timer_get_time() / 1000 - s_st.ts_monotonic_ms;
        uint32_t v2 = __atomic_load_n(&s_ver, __ATOMIC_ACQUIRE);
        if (v1 == v2) {
            if (age_ms) *age_ms = (int)age;
            return out->valid;
        }
    }
    *out = s_st;
    int64_t age = esp_timer_get_time() / 1000 - s_st.ts_monotonic_ms;
    if (age_ms) *age_ms = (int)age;
    return out->valid;
}

bool rf_gps_has_fix(void)
{
    rf_gps_state_t st;
    int age;
    if (!rf_gps_get(&st, &age)) return false;
    return st.valid && age < RF_GPS_FIX_TIMEOUT_MS;
}

esp_err_t rf_gps_send_cmd(const char *cmd)
{
    if (!cmd) return ESP_ERR_INVALID_ARG;
    /* 必须以 \r\n 结尾 */
    char buf[128];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    int w = uart_write_bytes(RF_GPS_UART_NUM, buf, (int)strlen(buf));
    return (w >= 0) ? ESP_OK : ESP_FAIL;
}
