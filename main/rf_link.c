/*
 * rf_link.c — RF 数据链路层实现（见 rf_link.h 协议说明）
 *
 * 职责：
 *   1) UART 接收 C5 发来的帧，做状态机对齐 / CRC8 校验 / 分发
 *   2) 维护线程安全的数据模型（扫频 RSSI、无人机列表、统计）
 *   3) 可选内置模拟数据源（CONFIG_EXAMPLE_RF_SIMULATE），无 C5 时驱动 UI 演示
 */

#include "rf_link.h"
#include "rf_json.h"
#include "rid_registry.h"
#include "rf_proto.h"
#include "rf_ringbuf.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <stdatomic.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>

static const char *TAG = "rf_link";

/* ============================================================================
 * 项目级硬件真值（不要被 sdkconfig 的历史默认值覆盖）：
 *   - 默认 UART LIVE 真实数据源（取消模拟数据显示），如需演示请通过 UI 切换。
 *   - UART: GPIO18(TX), GPIO17(RX), 115200 8N1（与 qt_ui/serial_manager.py 对齐）
 * ============================================================================ */

/* 强制关闭模拟：即使 sdkconfig 仍为 CONFIG_EXAMPLE_RF_SIMULATE=y，也在此强置为 0 */
#undef  CONFIG_EXAMPLE_RF_SIMULATE
#define CONFIG_EXAMPLE_RF_SIMULATE 0

/* 强制引脚 & 波特率：保证真实串口可用 */
#undef  CONFIG_EXAMPLE_RF_UART_PORT
#define CONFIG_EXAMPLE_RF_UART_PORT 1
#undef  CONFIG_EXAMPLE_RF_UART_TX_GPIO
#define CONFIG_EXAMPLE_RF_UART_TX_GPIO 40   /* P4 TX -> C5 RX */
#undef  CONFIG_EXAMPLE_RF_UART_RX_GPIO
#define CONFIG_EXAMPLE_RF_UART_RX_GPIO 42   /* P4 RX <- C5 TX */
#undef  CONFIG_EXAMPLE_RF_UART_BAUD
#define CONFIG_EXAMPLE_RF_UART_BAUD 115200  /* 与 qt_ui serial_manager 对齐 */

/* 运行时数据源开关：true=内置模拟生成；false=UART 真实数据 */
static volatile bool s_source_sim = (CONFIG_EXAMPLE_RF_SIMULATE) ? true : false;
/* 模拟任务句柄（用于挂起/恢复） */
static TaskHandle_t s_sim_sp_handle = NULL;
static TaskHandle_t s_sim_dr_handle = NULL;
/* UART 是否已初始化（只初始化一次） */
static bool s_uart_inited = false;
/* UART 实时通信调试输出开关（切到 UART 数据源时 UI 可自动打开） */
static volatile bool s_uart_debug_output = false;

/* ---- 数据存储（互斥保护） ------------------------------------------------ */
static StaticSemaphore_t s_mutex_storage;
static SemaphoreHandle_t  s_mtx;
static volatile uint32_t s_rf_version = 0;   // 每次写后 bump；LVGL 侧无锁读用

static uint8_t  s_rssi[RF_NUM_CHANNELS];
static uint32_t s_sweep_count = 0;          // C5 侧累计扫描计数（每帧+1）
static int64_t  s_last_spectrum_us = 0;     // 上一帧扫频的本地时间(us)
static uint32_t s_spectrum_rx_total = 0;   // 收到的扫频帧数（用于估算速率）

/* 无人机注册表（已封装，互斥在外层：调用方 xSemaphoreTake(s_mtx) 后操作） */
static rid_registry_t s_registry;

static rf_stats_t s_stats;

/* 最近一次解析到的扫描参数（来自 $D/spectrum 帧头部 meta） */
static rf_scan_config_t s_active_scan = {0, 125, 2, 300, 2};
/* 最近一次通过 $A 确认的配置 */
static rf_scan_config_t s_acked_cfg = {0, 125, 2, 300, 2};

/* 原始抓包环形缓冲（仅保留最近若干包供调试；UI 暂未使用，留接口） */
#define RF_RAW_RING_N 32
#define RF_RAW_PKT_MAX 64
typedef struct { uint8_t ch, rssi, len; uint8_t data[RF_RAW_PKT_MAX]; } rf_raw_t;
static rf_raw_t s_raw_ring[RF_RAW_RING_N];
static volatile int s_raw_head = 0;

/* 调试用：最近原始 UART 字节 + 帧信息 */
static struct {
    uint8_t  raw[RF_DEBUG_RAW_N];
    volatile int raw_len;
    volatile uint8_t last_frame_type;
    volatile uint16_t last_frame_len;
    volatile bool last_crc_ok;
} s_dbg;

/* ===== 日志行环形缓冲（Serial Monitor 用，基于 rf_ringbuf） ===== */
static rf_log_line_t s_log_ring[RF_LOG_RING_LINES];
static rf_ringbuf_t  s_log_rb = {
    .slots   = s_log_ring,
    .slot_sz = sizeof(rf_log_line_t),
    .capacity= RF_LOG_RING_LINES,
};

/* ===== RID 原始帧($R0) 环形缓冲（基于 rf_ringbuf） ===== */
static rf_rid0_frame_t s_rid0_ring[RF_RID0_RING_MAX];
static rf_ringbuf_t   s_rid0_rb = {
    .slots   = s_rid0_ring,
    .slot_sz = sizeof(rf_rid0_frame_t),
    .capacity= RF_RID0_RING_MAX,
};

/* ===== JSON 行级解析状态机（Qt 上位机同协议） ===== */
typedef struct {
    char     buf[2048];  /* 单行 JSON 文本缓冲（$D 126 点典型约 850 字节，留足余量） */
    uint16_t len;        /* 当前已写入的字符数 */
    uint8_t  overflow;   /* 1=该行超长已告警，避免一行多次 WARN */
} json_parser_t;
static json_parser_t s_jp;

/* 由于 send_json_line 与 uart_rx_task 早于以下函数定义，先前置声明 */
static void rf_log_push_impl(uint8_t type, const char *text, int len_override);
static void json_parser_feed(char ch);

/* ---- 辅助：UART 调试输出 helper（定义点保证位于 rf_log_push_impl 声明之后） ---- */

/* 辅助：字节数组 -> hex 字符串；写到 out，最多写入 out_sz-1，保证 \0。返回写入字符数(不含\0)。 */
static int bytes_to_hex(const uint8_t *d, int n, char *out, int out_sz)
{
    static const char hx[] = "0123456789ABCDEF";
    if (!d || !out || out_sz <= 1 || n <= 0) {
        if (out && out_sz >= 1) out[0] = '\0';
        return 0;
    }
    int w = 0, max = (out_sz - 1) / 3;
    if (n > max) n = max;
    for (int i = 0; i < n; i++) {
        out[w++] = hx[(d[i] >> 4) & 0x0F];
        out[w++] = hx[(d[i]) & 0x0F];
        if (i + 1 < n) out[w++] = ' ';
    }
    out[w] = '\0';
    return w;
}

/* UART 调试输出：TX 方向（一次写入的整段 bytes，一般已包含换行） */
static void uartdbg_tx_bytes(const uint8_t *d, int n)
{
    if (!s_uart_debug_output || !d || n <= 0) return;
    char line[RF_LOG_LINE_MAX];
    int w = snprintf(line, sizeof(line), "[TX] %d bytes: ", n);
    if (w < 0) w = 0;
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    int remain = (int)sizeof(line) - w - 1;
    if (remain > 0) {
        w += bytes_to_hex(d, (n > 48 ? 48 : n), line + w, remain + 1);
    }
    rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
}

/* UART 调试输出：RX 方向（收到的一段 bytes） */
static void uartdbg_rx_bytes(const uint8_t *d, int n)
{
    if (!s_uart_debug_output || !d || n <= 0) return;
    char line[RF_LOG_LINE_MAX];
    int w = snprintf(line, sizeof(line), "[RX] %d bytes: ", n);
    if (w < 0) w = 0;
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    int remain = (int)sizeof(line) - w - 1;
    if (remain > 0) {
        w += bytes_to_hex(d, (n > 48 ? 48 : n), line + w, remain + 1);
    }
    rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
}

/* UART 调试输出：帧解析结果摘要（凑成完整帧 / CRC 失败 / 同步错误） */
static void uartdbg_rx_frame(const char *tag, uint8_t type, uint16_t len, const uint8_t *head, int head_n)
{
    if (!s_uart_debug_output) return;
    char line[RF_LOG_LINE_MAX];
    int w = snprintf(line, sizeof(line), "[FRM] %s type=0x%02X len=%u", tag ? tag : "OK", type, len);
    if (w < 0) w = 0;
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    if (head && head_n > 0) {
        int remain = (int)sizeof(line) - w - 1;
        if (remain > 8) {
            w += snprintf(line + w, remain, "  head=");
            remain = (int)sizeof(line) - w - 1;
            if (remain > 0) {
                w += bytes_to_hex(head, head_n, line + w, remain + 1);
            }
        }
    }
    rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
}

/* 把某一步 UART 初始化/配置结果写一行到环形日志(不依赖 s_uart_debug_output 开关，常开用于上电自检) */
static void uart_push_log_res(const char *step, esp_err_t err, int port, int baud, int extra)
{
    char line[192];
    int w = snprintf(line, sizeof(line),
                     "[UART] %s: %s port=%d baud=%d extra=%d",
                     step, (err == ESP_OK) ? "OK" : esp_err_to_name(err),
                     port, baud, extra);
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
}

/* ---- CRC-8 (poly 0x07, init 0x00) ---------------------------------------- */
static uint8_t rf_crc8_update(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
        else            crc = (uint8_t)(crc << 1);
    }
    return crc;
}

/* ---- 内部：本地 ms 计时 -------------------------------------------------- */
static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ---- 帧分发处理 ---------------------------------------------------------- */
static void handle_spectrum(const uint8_t *p, uint16_t len)
{
    if (len != RF_NUM_CHANNELS) {
        ESP_LOGW(TAG, "spectrum bad len %u (exp %d)", len, RF_NUM_CHANNELS);
        return;
    }
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    memcpy(s_rssi, p, RF_NUM_CHANNELS);
    s_sweep_count++;
    s_last_spectrum_us = esp_timer_get_time();
    s_spectrum_rx_total++;
    s_stats.spectrum_frames = s_spectrum_rx_total;
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

static void handle_rid(const uint8_t *p, uint16_t len)
{
    if (len != sizeof(rf_rid_wire_t)) {
        ESP_LOGW(TAG, "rid bad len %u (exp %u)", len, (unsigned)sizeof(rf_rid_wire_t));
        return;
    }
    rf_rid_wire_t w;
    memcpy(&w, p, sizeof(w));
    int added = 0;
    int slot = -1;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    slot = rid_registry_upsert_wire(&s_registry, &w, now_ms(), &added);
    s_stats.rid_frames++;
    xSemaphoreGive(s_mtx);

    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);

    if (slot >= 0) {
        ESP_LOGI(TAG, "RID %s slot=%d id=%.20s UA lat=%.7f lon=%.7f  Op lat=%.7f lon=%.7f alt=%dm",
                 added ? "+add" : "upd", slot, w.uas_id,
                 w.lat_e7 / 1e7f, w.lon_e7 / 1e7f,
                 w.operator_lat_e7 / 1e7f, w.operator_lon_e7 / 1e7f,
                 (int)w.alt_geodetic_m);
    }
}

static void handle_rid_remove(const uint8_t *p, uint16_t len)
{
    if (len < 1 + 20) return;
    uint8_t id_type = p[0];
    const char *id = (const char *)(p + 1);
    /* wire 结构里 uas_id 是 char[20] (null padded)，所以这里最多比 20 少；
       但因为 rid_registry 使用 strncmp 比较，直接比较前 20 字节是等价的。 */
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    (void)rid_registry_remove_by_id(&s_registry, id_type, id);
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

static void handle_raw(const uint8_t *p, uint16_t len)
{
    if (len < 3) return;
    uint8_t plen = p[2];
    if ((uint16_t)(3 + plen) != len || plen > RF_RAW_PKT_MAX) return;
    int h = s_raw_head;
    s_raw_ring[h].ch = p[0];
    s_raw_ring[h].rssi = p[1];
    s_raw_ring[h].len = plen;
    memcpy(s_raw_ring[h].data, p + 3, plen);
    s_raw_head = (h + 1) % RF_RAW_RING_N;
}

static void handle_status(const uint8_t *p, uint16_t len)
{
    if (len != sizeof(rf_status_wire_t)) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    memcpy(&s_stats.last_status, p, sizeof(rf_status_wire_t));
    s_stats.last_status_ms = now_ms();
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

/* 分发已校验通过的帧 */
static void dispatch_frame(uint8_t type, const uint8_t *p, uint16_t len)
{
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_dbg.last_frame_type = type;
    s_dbg.last_frame_len  = len;
    s_dbg.last_crc_ok     = true;
    s_stats.last_valid_frame_ms = now_ms();
    uartdbg_rx_frame("OK", type, len, p, (len > 8 ? 8 : len));
    switch (type) {
        case RF_MSG_SPECTRUM:   handle_spectrum(p, len); break;
        case RF_MSG_RID_RECORD: handle_rid(p, len); break;
        case RF_MSG_RID_REMOVE: handle_rid_remove(p, len); break;
        case RF_MSG_RAW_PKT:    handle_raw(p, len); break;
        case RF_MSG_STATUS:     handle_status(p, len); break;
        default: break;
    }
}

/* ---- 字节级帧解析状态机 -------------------------------------------------- */
typedef enum { PS_SYNC0, PS_SYNC1, PS_TYPE, PS_LEN0, PS_LEN1, PS_PAYLOAD, PS_CRC } ps_t;

typedef struct {
    ps_t    st;
    uint8_t type;
    uint8_t crc;
    uint16_t len;
    uint16_t idx;
    uint8_t  buf[RF_MAX_PAYLOAD];
    int64_t  last_us;
} parser_t;

static void parser_reset(parser_t *pp)
{
    pp->st = PS_SYNC0;
    pp->idx = 0;
    pp->len = 0;
}

/* 喂一个字节；凑成完整帧时调用 dispatch_frame */
static void parser_feed(parser_t *pp, uint8_t b)
{
    int64_t tnow = esp_timer_get_time();
    // 超时重同步（帧中字节间隔 > 200ms）
    if (pp->st != PS_SYNC0 && (tnow - pp->last_us) > 200000) {
        s_stats.sync_errors++;
        parser_reset(pp);
    }
    pp->last_us = tnow;

    switch (pp->st) {
        case PS_SYNC0:
            if (b == RF_SYNC0) pp->st = PS_SYNC1;
            break;
        case PS_SYNC1:
            if (b == RF_SYNC1) { pp->st = PS_TYPE; pp->crc = 0; }
            else if (b == RF_SYNC0) { /* stay PS_SYNC1 */ }
            else parser_reset(pp);
            break;
        case PS_TYPE:
            pp->type = b;
            pp->crc = rf_crc8_update(pp->crc, b);
            pp->st = PS_LEN0;
            break;
        case PS_LEN0:
            pp->len = b;
            pp->crc = rf_crc8_update(pp->crc, b);
            pp->st = PS_LEN1;
            break;
        case PS_LEN1:
            pp->len |= (uint16_t)b << 8;
            pp->crc = rf_crc8_update(pp->crc, b);
            if (pp->len > RF_MAX_PAYLOAD) {
                ESP_LOGW(TAG, "frame len %u > max, resync", pp->len);
                s_stats.sync_errors++;
                parser_reset(pp);
            } else if (pp->len == 0) {
                pp->st = PS_CRC;
            } else {
                pp->idx = 0;
                pp->st = PS_PAYLOAD;
            }
            break;
        case PS_PAYLOAD:
            pp->buf[pp->idx++] = b;
            pp->crc = rf_crc8_update(pp->crc, b);
            if (pp->idx >= pp->len) pp->st = PS_CRC;
            break;
        case PS_CRC:
            if (b == pp->crc) {
                dispatch_frame(pp->type, pp->buf, pp->len);
            } else {
                s_stats.crc_errors++;
                s_dbg.last_frame_type = pp->type;
                s_dbg.last_frame_len  = pp->len;
                s_dbg.last_crc_ok     = false;
                ESP_LOGW(TAG, "CRC mismatch (type=0x%02X len=%u)", pp->type, pp->len);
                uartdbg_rx_frame("CRC_ERR", pp->type, pp->len, pp->buf, (pp->len > 8 ? 8 : pp->len));
            }
            parser_reset(pp);
            break;
    }
}

/* ---- UART RX 任务 --------------------------------------------------------- */
static parser_t s_uart_parser;

static void uart_rx_task(void *arg)
{
    ESP_LOGI(TAG, "UART RX task start port=%d baud=%d tx=%d rx=%d",
             (int)CONFIG_EXAMPLE_RF_UART_PORT, (int)CONFIG_EXAMPLE_RF_UART_BAUD,
             (int)CONFIG_EXAMPLE_RF_UART_TX_GPIO, (int)CONFIG_EXAMPLE_RF_UART_RX_GPIO);

    const int uart_port = CONFIG_EXAMPLE_RF_UART_PORT;
    const int buf_sz = 512;
    uint8_t *tmp = heap_caps_malloc(buf_sz, MALLOC_CAP_DEFAULT);
    if (!tmp) { ESP_LOGE(TAG, "rx buf alloc fail"); vTaskDelete(NULL); return; }

    parser_reset(&s_uart_parser);
    s_jp.len = 0;
    while (1) {
        int n = uart_read_bytes(uart_port, tmp, buf_sz, pdMS_TO_TICKS(20));
        if (n > 0) {
            s_stats.rx_bytes += n;
            s_stats.raw_rx_total_chunks++;
            if ((uint32_t)n > s_stats.raw_rx_peak_bytes) s_stats.raw_rx_peak_bytes = (uint32_t)n;
            s_stats.last_rx_byte_ms = now_ms();
            /* 保存最近 RF_DEBUG_RAW_N 字节原始数据供调试 */
            int copy = (n > RF_DEBUG_RAW_N) ? RF_DEBUG_RAW_N : n;
            memcpy(s_dbg.raw, tmp + n - copy, copy);
            s_dbg.raw_len = copy;
            uartdbg_rx_bytes(tmp, n);  /* 调试：记录本次 RX 原始字节 */
            for (int i = 0; i < n; i++) {
                uint8_t b = tmp[i];
                /* 二进制帧状态机（0xAA 0x55 头） */
                parser_feed(&s_uart_parser, b);
                /* 并列 Qt 上位机兼容的 JSON 行解析（$D/$W/$R/$R0/$A + LOG） */
                json_parser_feed((char)b);
            }
        }
        // 周期性维护：估算在线状态与扫频速率 + 注入诊断提示
        static uint32_t last_maint = 0;
        static uint32_t last_warn_rx_only = 0;
        uint32_t now = now_ms();
        if (now - last_maint > 1000) {
            last_maint = now;
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            // 3 秒内无任何帧则置离线
            uint32_t spec_local = s_spectrum_rx_total;
            static uint32_t last_spec = 0; static uint32_t last_t = 0;
            s_stats.sweep_rate_hz = (uint32_t)((uint64_t)(spec_local - last_spec) * 1000 / (now - last_t + 1));
            last_spec = spec_local; last_t = now;
            // 无数据阈值判断：最近 3s 无有效帧
            if (s_stats.last_valid_frame_ms != 0 &&
                now - s_stats.last_valid_frame_ms > 3000) {
                s_stats.link_online = false;
            }
            // 仅启动后曾收到字节也算 RX 路径正常
            xSemaphoreGive(s_mtx);

    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);

            /* 诊断：收到字节但 3s+ 没合成过有效帧 -> 串口 monitor 推一条提示 */
            if (s_uart_debug_output && !s_source_sim) {
                uint32_t rx_ms = s_stats.last_rx_byte_ms;
                uint32_t fr_ms = s_stats.last_valid_frame_ms;
                if (rx_ms != 0 && now - rx_ms <= 5000 &&
                    (fr_ms == 0 || now - fr_ms > 3000) &&
                    now - last_warn_rx_only > 5000) {
                    last_warn_rx_only = now;
                    char line[RF_LOG_LINE_MAX];
                    int w = snprintf(line, sizeof(line),
                                     "[WARN] RX bytes ok but NO valid frame. Check: baud=%d 8N1 TX/RX crossed GND common format=$D+LF or 0xAA55",
                                     (int)CONFIG_EXAMPLE_RF_UART_BAUD);
                    if (w < 0) w = 0;
                    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
                    rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
                }
            }
        }
    }
}

static esp_err_t init_uart(void)
{
    if (CONFIG_EXAMPLE_RF_UART_RX_GPIO < 0 && CONFIG_EXAMPLE_RF_UART_TX_GPIO < 0) {
        ESP_LOGW(TAG, "UART TX/RX GPIO not configured (-1), skip UART init (use sim only)");
        rf_log_push_impl(RF_LOG_LINE_RX_LOG,
                         "[UART] BOTH TX/RX GPIO=-1, init skipped.", -1);
        return ESP_ERR_INVALID_STATE;
    }
    const int port = CONFIG_EXAMPLE_RF_UART_PORT;
    const int tx = CONFIG_EXAMPLE_RF_UART_TX_GPIO;
    const int rx = CONFIG_EXAMPLE_RF_UART_RX_GPIO;
    const int baud = CONFIG_EXAMPLE_RF_UART_BAUD;

    /* ===== GPIO 预诊断 ===== */
    gpio_num_t rx_pin = (gpio_num_t)rx;
    gpio_num_t tx_pin = (gpio_num_t)tx;
    int rx_lvl = -1, tx_lvl = -1;
    bool rx_ok = false, tx_ok = false;
    if (rx >= 0 && GPIO_IS_VALID_GPIO(rx)) {
        /* RX 默认上拉：无信号时 idle 为高；能读到 idle 高说明引脚接到了真实线或浮空有上拉 */
        gpio_config_t gc = {
            .pin_bit_mask = (1ULL << rx_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t gr = gpio_config(&gc);
        rx_ok = (gr == ESP_OK);
        rx_lvl = (rx_ok) ? gpio_get_level(rx_pin) : -1;
    }
    if (tx >= 0 && GPIO_IS_VALID_GPIO(tx)) {
        gpio_config_t gc = {
            .pin_bit_mask = (1ULL << tx_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t gr = gpio_config(&gc);
        tx_ok = (gr == ESP_OK);
        tx_lvl = (tx_ok) ? gpio_get_level(tx_pin) : -1;
    }
    {
        char line[240];
        int w = snprintf(line, sizeof(line),
                         "[UART] PREINIT: port=%d baud=%d TX=%d(%s lv=%d) RX=%d(%s lv=%d) driver=installing",
                         port, baud,
                         tx, tx_ok ? "ok" : "invalid", tx_lvl,
                         rx, rx_ok ? "ok" : "invalid", rx_lvl);
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
        ESP_LOGI(TAG, "%s", line);
    }

    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err;
    err = uart_driver_install(port, 16384, 2048, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart install: %s", esp_err_to_name(err));
        uart_push_log_res("install", err, -1, -1, -1);
        return err;
    }
    err = uart_param_config(port, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart param: %s", esp_err_to_name(err));
        uart_push_log_res("param_config", err, -1, -1, -1);
        return err;
    }
    err = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart pin: %s", esp_err_to_name(err));
        uart_push_log_res("set_pin", err, -1, -1, -1);
        return err;
    }
    uart_set_rx_timeout(port, 1);   // 1 个字节超时即可吐出，降低 uart_read_bytes 延迟
    uart_disable_pattern_det_intr(port);
    uart_set_loop_back(port, false); // 先外部线模式

    uart_push_log_res("init", ESP_OK, port, baud, -1);

    /* ===== 硬件回环自检测：TX 内部短接到 RX，发 16 字节读回，验证 UART 驱动本身无死档 ===== */
    {
        uart_set_loop_back(port, true);
        /* 先 drain 干净 */
        uart_flush_input(port);
        const char pat[] = "UART_SELF_TEST_"; // 16B
        int wn = uart_write_bytes(port, pat, (uint32_t)sizeof(pat));
        esp_rom_delay_us(15000); // 115200 下 16B ~= 1.4ms, 留余量
        uint8_t rd[32];
        int rn = uart_read_bytes(port, rd, (uint32_t)sizeof(rd), pdMS_TO_TICKS(40));
        bool pass = (rn == (int)sizeof(pat)) && (memcmp(rd, pat, sizeof(pat)) == 0);
        uart_set_loop_back(port, false);
        uart_flush_input(port);

        char line[240];
        int w = snprintf(line, sizeof(line),
                         "[UART] LOOPBACK: wrote=%d read=%d match=%s -> %s",
                         wn, rn, pass ? "YES" : "NO",
                         pass ? "PASS (driver OK; no RX bytes => check cable/C5 baud/pinmux)"
                              : "FAIL (driver/GPIO path broken!)");
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
        ESP_LOGI(TAG, "%s", line);
    }

    /* ===== 对 C5 发送一条握手命令（mode=2 2.4G scan + rid_raw=0），唤醒对方 ===== */
    rf_link_send_band_mode(2);
    rf_link_send_rid_raw_level(0);
    {
        char line[180];
        int w = snprintf(line, sizeof(line),
                         "[UART] HANDSHAKE sent: mode=2.4G rid_raw=0. Waiting C5 $D/$R frames...");
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
    }

    xTaskCreatePinnedToCore(uart_rx_task, "rf_rx", 8192, NULL, 3, NULL, 1);  /* ★ CPU1 prio3, 不抢占 LVGL */
    ESP_LOGI(TAG, "UART RX started (baud=%d)", baud);
    return ESP_OK;
}

/* ---- 模拟数据源（无 C5 时驱动 UI） ---------------------------------------- */
#if CONFIG_EXAMPLE_RF_SIMULATE
static uint8_t clamp8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v); }

// 简单线性同余噪声（避免依赖 rand 的可重入问题）
static uint32_t lcg_state = 0x12345;
static int lcg_noise(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (int)(lcg_state >> 24) & 0x3F;  // 0..63
}

// 模拟一个高斯样式的凸起（中心 c0、幅度、宽度）
static int bump(int c, int c0, int amp, int width)
{
    int d = c - c0;
    int v = amp - (d * d) / width;   // 抛物线下降
    return v > 0 ? v : 0;
}

static void sim_spectrum_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "SIM spectrum task start (no C5, generating fake 2.4G data)");
    uint32_t t = 0;
    while (1) {
        if (!s_source_sim) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        t++;
        uint8_t rssi[RF_NUM_CHANNELS];
        // 慢漂移的噪声基底
        int floor = 18 + (int)(8 * sinf(t * 0.05f));
        // WiFi ch1/6/11 的典型占位
        int wifi6  = 60 + (int)(20 * sinf(t * 0.03f));
        int wifi11 = 50 + (int)(15 * sinf(t * 0.04f + 1.0f));
        // 蓝牙跳频窄峰
        int bt_c = (int)((t * 7) % RF_NUM_CHANNELS);
        // 移动的无人机下行链路信号
        int drone_c = 40 + (int)(35 * sinf(t * 0.08f));
        int drone_amp = 120 + (int)(30 * sinf(t * 0.2f));

        for (int c = 0; c < RF_NUM_CHANNELS; c++) {
            int v = floor + lcg_noise() / 2;
            v += bump(c, 37,  wifi6,  40);    // WiFi ch6 ~2437MHz
            v += bump(c, 62,  wifi11, 40);   // WiFi ch11 ~2462MHz
            v += bump(c, 1,  30, 20);        // WiFi ch1 ~2412MHz
            v += bump(c, bt_c, 90, 6);       // BT 窄峰
            v += bump(c, drone_c, drone_amp, 8); // 无人机信号
            rssi[c] = clamp8(v);
        }
        handle_spectrum(rssi, RF_NUM_CHANNELS);

        // 偶尔置 link_online + 假 status
        s_stats.link_online = true;
        if ((t % 4) == 0) {
            rf_status_wire_t st = {
                .uptime_s = t / 7,
                .sweep_rate_hz = 7,
                .nrf_ok = 1, .ble_ok = 1,
                .drone_count = 0,
                .reserved = 0,
                .bytes_tx_total = t * 130,
            };
            handle_status((const uint8_t *)&st, sizeof(st));
        }

        vTaskDelay(pdMS_TO_TICKS(150));  // ~7Hz 扫频
    }
}

// 模拟 2 架无人机
static void sim_drones_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "SIM drones task start");
    // 初始两条 RID 记录
    const char *id1 = "1587F3A2B9C04D5E";  // 假序列号
    const char *id2 = "UTM-0001234ABC";    // UTM 分配
    uint32_t t = 0;
    while (1) {
        if (!s_source_sim) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        t++;
        // 无人机1：固定区域，缓慢移动 + 高度摆动
        {
            rf_rid_wire_t w; memset(&w, 0, sizeof(w));
            w.uas_id_type = 1; // serial
            strncpy(w.uas_id, id1, sizeof(w.uas_id) - 1);
            strncpy(w.op_desc, "DJI Mavic 3 sim", sizeof(w.op_desc) - 1);
            strncpy(w.operator_id, "OP-CN-9001", sizeof(w.operator_id) - 1);
            // 北京附近 ~39.9, 116.4，漂移
            w.lat_e7 = (int32_t)((39.90 + 0.001 * sinf(t * 0.1f)) * 1e7);
            w.lon_e7 = (int32_t)((116.40 + 0.001 * cosf(t * 0.1f)) * 1e7);
            // 操作人位置（地面遥控点，相对 UA 偏南 ~50m）
            w.operator_lat_e7 = (int32_t)((39.8995) * 1e7);
            w.operator_lon_e7 = (int32_t)((116.3998) * 1e7);
            w.alt_geodetic_m = (int16_t)(120 + 20 * sinf(t * 0.2f));
            w.alt_pressure_m = w.alt_geodetic_m + 2;
            w.height_m = (int16_t)(80 + 15 * sinf(t * 0.2f + 1));
            w.horiz_speed = (uint8_t)(8 + (int)(3 * sinf(t * 0.3f)));
            w.vert_speed = (int8_t)(2 * sinf(t * 0.2f));
            w.ground_track = (uint16_t)((t * 10) % 36000);
            w.flags = 0x0F; // UA位置/操作员/描述/操作人位置 均有效
            handle_rid((const uint8_t *)&w, sizeof(w));
        }
        // 无人机2：航迹移动
        {
            rf_rid_wire_t w; memset(&w, 0, sizeof(w));
            w.uas_id_type = 3; // UTM-assigned
            strncpy(w.uas_id, id2, sizeof(w.uas_id) - 1);
            strncpy(w.op_desc, "Autopilot test craft", sizeof(w.op_desc) - 1);
            strncpy(w.operator_id, "OP-CN-7788", sizeof(w.operator_id) - 1);
            w.lat_e7 = (int32_t)((39.85 + 0.0008 * t) * 1e7);
            w.lon_e7 = (int32_t)((116.35 + 0.0008 * t) * 1e7);
            // 操作人位置（固定地面点，UA 北偏东方向飞行）
            w.operator_lat_e7 = (int32_t)((39.8480) * 1e7);
            w.operator_lon_e7 = (int32_t)((116.3520) * 1e7);
            w.alt_geodetic_m = (int16_t)(60 + 5 * sinf(t * 0.15f));
            w.alt_pressure_m = w.alt_geodetic_m + 3;
            w.height_m = (int16_t)(40 + 8 * sinf(t * 0.15f));
            w.horiz_speed = (uint8_t)(12);
            w.vert_speed = 0;
            w.ground_track = (uint16_t)9000; // 90.00 deg
            w.flags = 0x0F;
            handle_rid((const uint8_t *)&w, sizeof(w));
        }
        // 更新 status 里的 drone_count（直接改 copy）
        {
            // 统计活动数：用 registry 拷贝数
            rf_drone_t tmp[RF_MAX_DRONES];
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            int cnt = rid_registry_copy_active(&s_registry, tmp, RF_MAX_DRONES);
            s_stats.last_status.drone_count = (uint8_t)cnt;
            xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
#endif // CONFIG_EXAMPLE_RF_SIMULATE

/* ---- 对外 API ----------------------------------------------------------- */

/** 若 UART 未初始化，则尝试初始化（一次性）。 */
static void ensure_uart_inited(void)
{
    if (s_uart_inited) return;
    if (CONFIG_EXAMPLE_RF_UART_RX_GPIO < 0 && CONFIG_EXAMPLE_RF_UART_TX_GPIO < 0) return;
    esp_err_t err = init_uart();
    if (err == ESP_OK) s_uart_inited = true;
}

esp_err_t rf_link_start(void)
{
    s_mtx = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    if (!s_mtx) return ESP_ERR_NO_MEM;
    rid_registry_init(&s_registry);
    rf_ringbuf_init(&s_log_rb);
    rf_ringbuf_init(&s_rid0_rb);
    memset(&s_stats, 0, sizeof(s_stats));

    ESP_LOGI(TAG, "=== BUILD: " __DATE__ " " __TIME__ " ===");
    ESP_LOGI(TAG, "rf_link_start: simulate=%d uart_port=%d baud=%d tx=%d rx=%d",
             (int)CONFIG_EXAMPLE_RF_SIMULATE, (int)CONFIG_EXAMPLE_RF_UART_PORT,
             (int)CONFIG_EXAMPLE_RF_UART_BAUD,
             (int)CONFIG_EXAMPLE_RF_UART_TX_GPIO, (int)CONFIG_EXAMPLE_RF_UART_RX_GPIO);

#if CONFIG_EXAMPLE_RF_SIMULATE
    // 内置模拟数据源任务（默认按 s_source_sim 决定是否实际生成数据）
    // 模拟数据源任务（仅当 s_source_sim=1 时 CPU 有工作；放最低优先级避免抢占 LVGL/UART）
    xTaskCreatePinnedToCore(sim_spectrum_task, "rf_sim_sp", 4096, NULL, 2, &s_sim_sp_handle, 1);  /* ★ CPU1 */
    xTaskCreatePinnedToCore(sim_drones_task,   "rf_sim_dr", 4096, NULL, 2, &s_sim_dr_handle, 1);  /* ★ CPU1 */
    // 若运行时当前配置是 UART 数据源，则尝试初始化 UART（双任务常驻、由 s_source_sim 控制行为）
    if (!s_source_sim) {
        ensure_uart_inited();
    } else if (CONFIG_EXAMPLE_RF_UART_RX_GPIO >= 0) {
        /* 即使当前默认走模拟，也把 UART 启起来，便于中途切换到真实数据源 */
        ensure_uart_inited();
    }
#else
    // 仅真实 UART 接收
    ensure_uart_inited();
#endif
    return ESP_OK;
}

/**
 * @brief 运行时切换数据源
 * @param use_sim true=启用内置模拟数据；false=使用 UART(GPIO17/18) 真实串口数据
 */
void rf_link_set_source(bool use_sim)
{
    bool prev = s_source_sim;
    s_source_sim = use_sim;
    ESP_LOGI(TAG, "data source -> %s", use_sim ? "SIMULATE" : "UART");

    /* 切换源：清空旧残留，避免 UI 一直显示上一源的陈旧 RID / 频谱数据 */
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    rid_registry_reset(&s_registry);
    memset(s_rssi, 0, sizeof(s_rssi));
    s_sweep_count = 0;
    s_last_spectrum_us = 0;
    s_spectrum_rx_total = 0;
    /* 切源即清累计帧计数，方便排查新源是否有进数据 */
    s_stats.rx_bytes = 0;
    s_stats.rx_frames = 0;
    s_stats.crc_errors = 0;
    s_stats.sync_errors = 0;
    s_stats.spectrum_frames = 0;
    s_stats.rid_frames = 0;
    s_stats.json_lines = 0;
    s_stats.json_D = s_stats.json_W = s_stats.json_R = s_stats.json_R0 = s_stats.json_A = 0;
    s_stats.raw_rx_peak_bytes = 0;
    s_stats.raw_rx_total_chunks = 0;
    s_stats.last_status_ms = 0;
    s_stats.last_rx_byte_ms = 0;
    s_stats.last_valid_frame_ms = 0;
    memset(&s_stats.last_status, 0, sizeof(s_stats.last_status));
    s_stats.link_online = false;
    s_stats.sweep_rate_hz = 0;
    xSemaphoreGive(s_mtx);

    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);

    if (!use_sim) {
        ensure_uart_inited();
    }

    if (prev != use_sim) {
        char line[RF_LOG_LINE_MAX];
        int w = snprintf(line, sizeof(line),
                         "[SRC] switch to %s (UART port=%d baud=%d TX=%d RX=%d)",
                         use_sim ? "SIMULATE" : "UART(LIVE)",
                         (int)CONFIG_EXAMPLE_RF_UART_PORT, (int)CONFIG_EXAMPLE_RF_UART_BAUD,
                         (int)CONFIG_EXAMPLE_RF_UART_TX_GPIO, (int)CONFIG_EXAMPLE_RF_UART_RX_GPIO);
        if (w < 0) w = 0;
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
    }
}

bool rf_link_get_source(void)
{
    return s_source_sim;
}

void rf_link_set_uart_debug_output(bool on)
{
    s_uart_debug_output = on;
    ESP_LOGI(TAG, "UART debug output -> %s", on ? "ON" : "OFF");
    if (on) {
        char line[96];
        int w = snprintf(line, sizeof(line),
                         "[DBG] UART debug ON (port=%d, baud=%d, TX=%d, RX=%d)",
                         (int)CONFIG_EXAMPLE_RF_UART_PORT, (int)CONFIG_EXAMPLE_RF_UART_BAUD,
                         (int)CONFIG_EXAMPLE_RF_UART_TX_GPIO, (int)CONFIG_EXAMPLE_RF_UART_RX_GPIO);
        if (w < 0) w = 0;
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
    }
}

bool rf_link_get_uart_debug_output(void)
{
    return s_uart_debug_output;
}

/* 当前生效的运行时波特率（0 = UART 未 init 未通过 set_baud 改写过） */
static volatile int s_runtime_baud = 0;

esp_err_t rf_link_set_uart_baud(int baud)
{
    if (baud <= 0) return ESP_ERR_INVALID_ARG;
    ensure_uart_inited();
    const int port = CONFIG_EXAMPLE_RF_UART_PORT;
    /* 如果驱动还没装上（ensure 失败，GPIO -1 之类），返回 Invalid state 并写一行日志 */
    if (!s_uart_inited) {
        uart_push_log_res("set_baud SKIP(not_inited)", ESP_ERR_INVALID_STATE, port, baud, -1);
        return ESP_ERR_INVALID_STATE;
    }
    /* 使用 uart_set_baudrate 单独改；其它 8N1 参数保持 */
    esp_err_t err = uart_set_baudrate(port, baud);
    if (err == ESP_OK) {
        uart_flush_input(port);
        s_runtime_baud = baud;
        char line[128];
        int w = snprintf(line, sizeof(line),
                         "[UART] BAUD changed -> %d (RX flushed)", baud);
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
        /* 重置 rx 计数方便下一阶段观察 */
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        s_stats.rx_bytes = 0;
        s_stats.rx_frames = 0;
        s_stats.raw_rx_total_chunks = 0;
        s_stats.last_rx_byte_ms = 0;
        s_stats.last_valid_frame_ms = 0;
        xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
        /* 切完后再发一次握手命令，唤醒对方 2.4G 扫描 */
        rf_link_send_band_mode(2);
        rf_link_send_rid_raw_level(0);
    } else {
        uart_push_log_res("set_baud", err, port, baud, -1);
    }
    return err;
}

int rf_link_get_uart_baud(void)
{
    if (!s_uart_inited) return 0;
    if (s_runtime_baud > 0) return s_runtime_baud;
    return CONFIG_EXAMPLE_RF_UART_BAUD;
}

int rf_link_autodetect_baud(uint32_t probe_ms)
{
    static const int candidates[] = { 115200, 460800, 921600, 230400, 57600, 9600 };
    if (probe_ms < 300) probe_ms = 600;
    int hit = 0;
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        int b = candidates[i];
        rf_link_set_uart_baud(b);
        TickType_t start = xTaskGetTickCount();
        uint32_t frames0 = s_stats.rx_frames;
        uint32_t bytes0 = s_stats.rx_bytes;
        /* 等 probe_ms 期间观察是否有字节进来 */
        while ((uint32_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(probe_ms)) {
            vTaskDelay(pdMS_TO_TICKS(30));
            if (s_stats.rx_bytes > bytes0 + 8) break; /* 已经有字节进来，再留一点合成时间 */
        }
        vTaskDelay(pdMS_TO_TICKS(80)); /* 合成窗口 */
        uint32_t df = s_stats.rx_frames - frames0;
        uint32_t db = s_stats.rx_bytes - bytes0;
        char line[160];
        int w = snprintf(line, sizeof(line),
                         "[UART] PROBE baud=%-7d frames+%lu bytes+%lu %s",
                         b, (unsigned long)df, (unsigned long)db,
                         (df > 0 || db > 16) ? "<- MATCH" : "skip");
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
        if (df > 0 || db > 32) {
            hit = b;
            break;
        }
    }
    if (hit == 0) {
        rf_log_push_impl(RF_LOG_LINE_RX_LOG,
                         "[UART] PROBE done: NO match (check TX/RX wires or C5 is silent)", -1);
    } else {
        char line[128];
        int w = snprintf(line, sizeof(line),
                         "[UART] PROBE LOCKED -> baud=%d", hit);
        if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, w);
    }
    return hit;
}

bool rf_link_get_spectrum(uint8_t rssi_out[RF_NUM_CHANNELS], uint32_t *sweep_cnt, uint32_t *age_ms)
{
    for (int a = 0; a < 3; a++) {
        uint32_t v1 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        bool has = (s_sweep_count > 0);
        if (rssi_out && has) memcpy(rssi_out, s_rssi, RF_NUM_CHANNELS);
        uint32_t cnt = s_sweep_count;
        uint32_t age = s_last_spectrum_us ? (uint32_t)((esp_timer_get_time() - s_last_spectrum_us) / 1000) : 0xFFFFFFFFu;
        uint32_t v2 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        if (v1 == v2) {
            if (sweep_cnt) *sweep_cnt = cnt;
            if (age_ms)    *age_ms    = age;
            return has;
        }
    }
    if (sweep_cnt) *sweep_cnt = s_sweep_count;
    if (age_ms)    *age_ms    = s_last_spectrum_us ? (uint32_t)((esp_timer_get_time() - s_last_spectrum_us) / 1000) : 0xFFFFFFFFu;
    return s_sweep_count > 0;
}

int rf_link_get_drones(rf_drone_t *out, int max_count)
{
    uint32_t now = now_ms();

    /* 每次查询顺手清过期 (>60s 没更新) */
    xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50));
    rid_registry_purge_expired(&s_registry, now, 60000);
    int n = rid_registry_copy_active(&s_registry, out, max_count);
    xSemaphoreGive(s_mtx);

    /* 按 last_seen_ms 降序: 最新更新的排第一 */
    if (n > 1) {
        int cmp(const void *a, const void *b) {
            uint32_t va = ((const rf_drone_t *)a)->last_seen_ms;
            uint32_t vb = ((const rf_drone_t *)b)->last_seen_ms;
            /* 降序: vb - va, 防溢出 */
            if (vb > va) return  1;
            if (vb < va) return -1;
            return 0;
        }
        qsort(out, n, sizeof(rf_drone_t), cmp);
    }
    return n;
}

void rf_link_get_stats(rf_stats_t *out)
{
    for (int a = 0; a < 3; a++) {
        uint32_t v1 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        *out = s_stats;
        uint32_t since = now_ms() - out->last_status_ms;
        if (out->rx_frames == 0) out->link_online = false;
        else if (out->spectrum_frames == 0 && since > 3000) out->link_online = false;
        else out->link_online = true;
        uint32_t v2 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        if (v1 == v2) return;
    }
    *out = s_stats;
    uint32_t since = now_ms() - out->last_status_ms;
    if (out->rx_frames == 0) out->link_online = false;
    else if (out->spectrum_frames == 0 && since > 3000) out->link_online = false;
    else out->link_online = true;
}

uint32_t rf_link_drones_generation(void)
{
    return rid_registry_generation(&s_registry);  // generation counter is atomic inside rid_registry
}

void rf_link_get_debug(rf_debug_t *out)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    out->rx_bytes     = s_stats.rx_bytes;
    out->rx_frames    = s_stats.rx_frames;
    out->crc_errors   = s_stats.crc_errors;
    out->sync_errors  = s_stats.sync_errors;
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    out->raw_len         = s_dbg.raw_len;
    out->last_frame_type = s_dbg.last_frame_type;
    out->last_frame_len  = s_dbg.last_frame_len;
    out->last_crc_ok     = s_dbg.last_crc_ok;
    int n = s_dbg.raw_len;
    if (n > RF_DEBUG_RAW_N) n = RF_DEBUG_RAW_N;
    memcpy(out->raw, s_dbg.raw, n);
}

/* ---- 命令发送（P4 -> C5，JSON 行格式，与 Qt 上位机兼容） ---------------- */
static void send_json_line(const char *json_fmt, ...)
{
    /* 无 UART 引脚时（模拟模式）仅在日志里打印 */
    char buf[256];
    va_list ap;
    va_start(ap, json_fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, json_fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 3) n = sizeof(buf) - 3;
    buf[n++] = '\n';
    buf[n] = '\0';
    ESP_LOGI(TAG, "TX CMD: %s", buf);

    /* 写入 Serial Monitor TX 日志（去掉尾部换行） */
    {
        int ll = n - 1;
        if (ll > 0 && buf[ll - 1] == '\r') ll--;
        if (ll > 0) rf_log_push_impl(RF_LOG_LINE_TX_CMD, buf, ll);
    }

#if defined(CONFIG_EXAMPLE_RF_UART_RX_GPIO) && defined(CONFIG_EXAMPLE_RF_UART_TX_GPIO)
    if (CONFIG_EXAMPLE_RF_UART_TX_GPIO >= 0) {
        /* uart_write_bytes 线程安全（内部锁），无需额外互斥 */
        uartdbg_tx_bytes((const uint8_t *)buf, n);  /* 调试：字节级 TX 原始内容 */
        uart_write_bytes(CONFIG_EXAMPLE_RF_UART_PORT, buf, (size_t)n);
    }
#endif
}

void rf_link_send_scan_cfg(const rf_scan_config_t *cfg)
{
    if (!cfg) return;
    send_json_line("{\"cmd\":\"cfg\",\"s\":%d,\"e\":%d,\"r\":%d,\"st\":%d,\"sp\":%d}",
                   cfg->start_ch, cfg->end_ch, cfg->rate_idx,
                   cfg->settle_us, cfg->spp);
}

void rf_link_send_band_mode(rf_band_t band)
{
    send_json_line("{\"cmd\":\"mode\",\"band\":\"%s\"}",
                   (band == RF_BAND_5G) ? "5g" : "24g");
}

void rf_link_send_rid_raw_level(int level)
{
    int lv = (level < 0) ? 0 : ((level > 3) ? 3 : level);
    send_json_line("{\"cmd\":\"rid_raw\",\"level\":%d}", lv);
}

void rf_link_send_gain(int gain)
{
    int g = (gain < 1) ? 1 : ((gain > 10) ? 10 : gain);
    send_json_line("{\"cmd\":\"gain\",\"val\":%d}", g);
}

void rf_link_get_last_cfg(rf_scan_config_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    *out = s_acked_cfg;
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

void rf_link_get_scan_status(rf_scan_config_t *out)
{
    if (!out) return;
    for (int a = 0; a < 3; a++) {
        uint32_t v1 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        *out = s_active_scan;
        uint32_t v2 = __atomic_load_n(&s_rf_version, __ATOMIC_ACQUIRE);
        if (v1 == v2) return;
    }
    *out = s_active_scan;
}

/* ---- 协议分析仪（2.4G 频谱）：核心算法见 rf_proto.c ---- */
int rf_link_analyze_spectrum(rf_proto_hit_t *out, int out_max)
{
    if (!out || out_max <= 0) return 0;
    uint8_t rssi[RF_NUM_CHANNELS];
    uint32_t sweep = 0;
    if (!rf_link_get_spectrum(rssi, &sweep, NULL)) return 0;
    return rf_proto_analyze(rssi, (int)sweep, out, out_max);
}


/* ============================================================================
 *  日志环形缓冲（Serial Monitor：底层见 rf_ringbuf.c）
 * ============================================================================ */
static void rf_log_push_impl(uint8_t type, const char *text, int len_override)
{
    if (!text) return;
    int n = (len_override >= 0) ? len_override : (int)strlen(text);
    if (n < 0) n = 0;
    if (n >= RF_LOG_LINE_MAX) n = RF_LOG_LINE_MAX - 1;

    rf_log_line_t line;
    memset(&line, 0, sizeof(line));
    line.type = type;
    line.len  = (uint16_t)n;
    if (n > 0) memcpy(line.text, text, (size_t)n);
    line.text[n] = '\0';

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    (void)rf_ringbuf_push_copy(&s_log_rb, &line);
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

/* 用于 $D / $W 数据帧：缩成可读前缀，避免刷屏 */
static void log_push_data_line(rf_log_line_type_t type, const char *prefix,
                               const char *json, int s_val, int e_val, int r_val,
                               int st_val, int sp_val, int count_val)
{
    char short_line[RF_LOG_LINE_MAX];
    (void)prefix;
    int w = snprintf(short_line, sizeof(short_line),
        "v[%d] s=%d e=%d r=%d st=%d sp=%d (ch:%d)",
        s_val, s_val, e_val, r_val, st_val, sp_val, count_val);
    (void)json;
    if (w >= (int)sizeof(short_line)) w = (int)sizeof(short_line) - 1;
    rf_log_push_impl((uint8_t)type, short_line, w);
}

static void log_push_wifi5_line(rf_log_line_type_t type, int b_val, int n_val, int count_val)
{
    char short_line[RF_LOG_LINE_MAX];
    int w = snprintf(short_line, sizeof(short_line),
        "5G b=%d n=%d c[%d]", b_val, n_val, count_val);
    if (w >= (int)sizeof(short_line)) w = (int)sizeof(short_line) - 1;
    rf_log_push_impl((uint8_t)type, short_line, w);
}

int rf_log_read(rf_log_line_t *out, int out_max, uint32_t *from_seq)
{
    if (!out || !from_seq || out_max <= 0) return 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    int n = rf_ringbuf_read_seq(&s_log_rb, out, out_max, from_seq);
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    return n;
}

void rf_log_get_stats(rf_log_stats_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    rf_ringbuf_stat(&s_log_rb, &out->total, &out->dropped, NULL);
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

/* ============================================================================
 *  RID RAW ($R0) 环形缓冲（底层见 rf_ringbuf.c）
 * ============================================================================ */
static void rid0_push_impl(const rf_rid0_frame_t *f)
{
    if (!f) return;
    /* ringbuf 的 total 自增作为 seq；压入后读最新 seq 写回到 slot->seq */
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    (void)rf_ringbuf_push_copy(&s_rid0_rb, f);
    /* 回填 seq：上一次 push 的 slot = (head-1+capacity)%capacity */
    int cap = s_rid0_rb.capacity;
    int last = (s_rid0_rb.head - 1 + cap) % cap;
    rf_rid0_frame_t *slot = (rf_rid0_frame_t *)s_rid0_rb.slots + last;
    slot->seq = s_rid0_rb.total;
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
}

int rf_link_read_rid0_frames(rf_rid0_frame_t *out, int out_max, uint32_t *from_seq)
{
    if (!out || !from_seq || out_max <= 0) return 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    int n = rf_ringbuf_read_seq(&s_rid0_rb, out, out_max, from_seq);
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    return n;
}

/* ============================================================================
 *  极简 JSON 解析（零依赖，避免引入 cJSON 膨胀）
 *  仅支持:
 *    - 顶层对象
 *    - 键 -> 值 (int / bool / string / 数组(int) )
 *  返回 NULL 表示字段缺失或类型不匹配
 * ============================================================================ */
typedef struct {
    const char *p;
    const char *end;
} js_ctx_t;

static void js_skip_ws(js_ctx_t *c)
{
    while (c->p < c->end && (*c->p == ' ' || *c->p == '\t' || *c->p == '\r' || *c->p == '\n')) c->p++;
}

static int js_expect(js_ctx_t *c, char ch)
{
    js_skip_ws(c);
    if (c->p >= c->end || *c->p != ch) return -1;
    c->p++;
    return 0;
}

static int js_parse_int(js_ctx_t *c, int *out)
{
    js_skip_ws(c);
    if (c->p >= c->end) return -1;
    int sign = 1;
    if (*c->p == '-') { sign = -1; c->p++; }
    else if (*c->p == '+') { c->p++; }
    if (c->p >= c->end || !isdigit((unsigned char)*c->p)) return -1;
    long v = 0;
    while (c->p < c->end && isdigit((unsigned char)*c->p)) {
        v = v * 10 + (*c->p - '0');
        c->p++;
    }
    *out = (int)(sign * v);
    return 0;
}

static int js_parse_bool(js_ctx_t *c, int *out)
{
    js_skip_ws(c);
    if (c->p + 4 <= c->end && strncmp(c->p, "true", 4) == 0) { c->p += 4; *out = 1; return 0; }
    if (c->p + 5 <= c->end && strncmp(c->p, "false", 5) == 0) { c->p += 5; *out = 0; return 0; }
    return -1;
}

/* 返回 0=成功。*out_s 指向字符串内部, *out_len 为字符数 (不含引号) */
static int js_parse_string_ref(js_ctx_t *c, const char **out_s, int *out_len)
{
    js_skip_ws(c);
    if (c->p >= c->end || *c->p != '"') return -1;
    c->p++;
    const char *start = c->p;
    while (c->p < c->end && *c->p != '"') {
        if (*c->p == '\\' && c->p + 1 < c->end) c->p++;  /* 跳过转义 */
        c->p++;
    }
    if (c->p >= c->end) return -1;
    *out_s = start;
    *out_len = (int)(c->p - start);
    c->p++;
    return 0;
}

/* 跳过任意 JSON 值 */
static int js_skip_value(js_ctx_t *c)
{
    js_skip_ws(c);
    if (c->p >= c->end) return -1;
    char ch = *c->p;
    if (ch == '"') {
        const char *s; int l;
        return js_parse_string_ref(c, &s, &l);
    } else if (ch == '{' || ch == '[') {
        char open = ch, close = (ch == '{') ? '}' : ']';
        int depth = 1;
        c->p++;
        while (c->p < c->end && depth > 0) {
            if (*c->p == '"') {
                c->p++;
                while (c->p < c->end && *c->p != '"') {
                    if (*c->p == '\\' && c->p + 1 < c->end) c->p++;
                    c->p++;
                }
                if (c->p < c->end) c->p++;
            } else if (*c->p == open) { depth++; c->p++; }
            else if (*c->p == close) { depth--; c->p++; }
            else c->p++;
        }
        return (depth == 0) ? 0 : -1;
    } else {
        /* 数字 / true / false / null */
        while (c->p < c->end && *c->p != ',' && *c->p != ']' && *c->p != '}') c->p++;
        return 0;
    }
}

/* 在顶层对象中查找指定键，找到后把 ctx 定位到 value 的开始位置 */
static int js_find_key(js_ctx_t *c, const char *key, js_ctx_t *value_ctx)
{
    js_skip_ws(c);
    if (js_expect(c, '{') != 0) return -1;
    js_skip_ws(c);
    if (c->p < c->end && *c->p == '}') { c->p++; return -1; }
    while (1) {
        const char *ks; int kl;
        if (js_parse_string_ref(c, &ks, &kl) != 0) return -1;
        js_skip_ws(c);
        if (c->p >= c->end || *c->p != ':') return -1;
        c->p++;
        int match = (int)strlen(key) == kl && strncmp(ks, key, (size_t)kl) == 0;
        if (match) {
            *value_ctx = *c;
            return 0;
        } else {
            if (js_skip_value(c) != 0) return -1;
        }
        js_skip_ws(c);
        if (c->p >= c->end) return -1;
        if (*c->p == ',') { c->p++; continue; }
        if (*c->p == '}') { c->p++; return -1; }
        return -1;
    }
}

static int js_get_int(const char *json, int jlen, const char *key, int def)
{
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, key, &vctx) != 0) return def;
    int v = def;
    if (js_parse_int(&vctx, &v) != 0) return def;
    return v;
}

static int js_get_bool_int(const char *json, int jlen, const char *key, int def)
{
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, key, &vctx) != 0) return def;
    int v;
    if (js_parse_bool(&vctx, &v) == 0) return v;
    if (js_parse_int(&vctx, &v) == 0) return (v ? 1 : 0);
    return def;
}

static int js_get_cstr(const char *json, int jlen, const char *key, char *out, int out_sz)
{
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, key, &vctx) != 0) return -1;
    const char *s; int sl;
    if (js_parse_string_ref(&vctx, &s, &sl) != 0) return -1;
    int n = (sl < out_sz - 1) ? sl : (out_sz - 1);
    memcpy(out, s, (size_t)n);
    out[n] = '\0';
    return n;
}

/* 读取 int 数组（v 或 c），最多 out_max 个，返回复制个数 */
static int js_get_int_array(const char *json, int jlen, const char *key, int *out, int out_max)
{
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, key, &vctx) != 0) return 0;
    js_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return 0;
    vctx.p++;
    int cnt = 0;
    while (cnt < out_max) {
        js_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        if (*vctx.p == ']') { vctx.p++; break; }
        int val;
        if (js_parse_int(&vctx, &val) != 0) break;
        out[cnt++] = val;
        js_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        if (*vctx.p == ',') { vctx.p++; continue; }
        if (*vctx.p == ']') { vctx.p++; break; }
        break;
    }
    return cnt;
}

/* 解析无人机 $R 顶层数组 -> 列表长度 */
static int js_get_drone_count(const char *json, int jlen)
{
    /* 顶层对象，找 drones 数组，返回 length */
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, "drones", &vctx) != 0) return 0;
    js_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return 0;
    vctx.p++;
    int depth = 1;
    int count = 0;
    int in_obj = 0;
    while (vctx.p < vctx.end && depth > 0) {
        char ch = *vctx.p;
        if (ch == '"') {
            vctx.p++;
            while (vctx.p < vctx.end && *vctx.p != '"') {
                if (*vctx.p == '\\' && vctx.p + 1 < vctx.end) vctx.p++;
                vctx.p++;
            }
            if (vctx.p < vctx.end) vctx.p++;
            continue;
        }
        if (ch == '{') { depth++; in_obj++; vctx.p++; continue; }
        if (ch == '}') { depth--; if (in_obj > 0) { in_obj--; if (in_obj == 0) count++; } vctx.p++; continue; }
        if (ch == '[') { depth++; vctx.p++; continue; }
        if (ch == ']') { depth--; vctx.p++; continue; }
        vctx.p++;
    }
    return count;
}

/* ---- 子对象 JSON 查找工具（drone 数组里的第 i 个对象） ----------------------- */

/* 找 drones 数组第 idx 个对象的起止指针，成功返回 sub_start/sub_len，
   仅包含一个对象的内部 "{...}" 内容（不含包围括号）。 */
static int js_nth_drone_sub(const char *json, int jlen, int idx,
                            const char **sub_start, int *sub_len)
{
    js_ctx_t root = { json, json + jlen };
    js_ctx_t vctx;
    if (js_find_key(&root, "drones", &vctx) != 0) return -1;
    js_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return -1;
    vctx.p++;
    int depth = 1;
    int i = 0;
    int found = 0;
    while (vctx.p < vctx.end && depth > 0) {
        js_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        char ch = *vctx.p;
        if (ch == ']') { depth--; vctx.p++; continue; }
        if (ch == '{') {
            if (i == idx) {
                const char *s = vctx.p;   /* { */
                int obj_depth = 0;
                int in_str = 0;
                while (vctx.p < vctx.end) {
                    char cc = *vctx.p;
                    if (cc == '"') {
                        in_str = !in_str;
                        vctx.p++;
                        continue;
                    }
                    if (in_str) {
                        if (cc == '\\' && vctx.p + 1 < vctx.end) vctx.p++;
                        vctx.p++;
                        continue;
                    }
                    if (cc == '{') { obj_depth++; vctx.p++; continue; }
                    if (cc == '}') {
                        obj_depth--;
                        vctx.p++;
                        if (obj_depth == 0) {
                            *sub_start = s + 1;            /* 跳开头 { */
                            *sub_len = (int)(vctx.p - s) - 2;/* 跳尾 } */
                            return 0;
                        }
                        continue;
                    }
                    vctx.p++;
                }
                (void)found;
                return -1;
            } else {
                /* 跳过这个对象的 {...} */
                int obj_depth = 0;
                int in_str = 0;
                while (vctx.p < vctx.end) {
                    char cc = *vctx.p;
                    if (cc == '"') { in_str = !in_str; vctx.p++; continue; }
                    if (in_str) { if (cc == '\\' && vctx.p + 1 < vctx.end) vctx.p++; vctx.p++; continue; }
                    if (cc == '{') { obj_depth++; vctx.p++; continue; }
                    if (cc == '}') {
                        obj_depth--; vctx.p++;
                        if (obj_depth == 0) break;
                        continue;
                    }
                    vctx.p++;
                }
                i++;
                continue;
            }
        }
        /* 其它字符跳过 */
        if (ch == ',') vctx.p++;
        else if (ch == ']') depth--;
        else vctx.p++;
    }
    return -1;
}

/* 子对象版的取值 API（root 只是 JSON 对象的内容，不含外层 {}） */
#define JS_SUB_TMP_SZ 2048
static int js_sub_wrap(const char *s, int sl, char *tmp, int tmp_sz)
{
    int tl = sl + 2;
    if (tl <= 2) tl = 2;
    if (tl > tmp_sz) tl = tmp_sz;
    tmp[0] = '{';
    int cplen = tl - 2;
    if (cplen > 0) memcpy(tmp + 1, s, (size_t)cplen);
    tmp[tl - 1] = '}';
    return tl;
}
static int js_sub_get_int(const char *s, int sl, const char *key, int def)
{
    char tmp[JS_SUB_TMP_SZ];
    int tl = js_sub_wrap(s, sl, tmp, (int)sizeof(tmp));
    return js_get_int(tmp, tl, key, def);
}

static int js_sub_get_cstr(const char *s, int sl, const char *key, char *out, int out_sz)
{
    char tmp[JS_SUB_TMP_SZ];
    int tl = js_sub_wrap(s, sl, tmp, (int)sizeof(tmp));
    return js_get_cstr(tmp, tl, key, out, out_sz);
}

static int js_sub_get_bool(const char *s, int sl, const char *key, int def)
{
    char tmp[JS_SUB_TMP_SZ];
    int tl = js_sub_wrap(s, sl, tmp, (int)sizeof(tmp));
    return js_get_bool_int(tmp, tl, key, def);
}

/* 字符串 -> 数字（整数或浮点），遇到 null/true/false/非法首字符 返回 NaN 语义: saw_ok=0 */
static int js_parse_double_ctx(js_ctx_t *c, double *out)
{
    js_skip_ws(c);
    if (c->p >= c->end) return -1;
    const char *p = c->p;
    /* null */
    if (c->end - p >= 4 && strncmp(p, "null", 4) == 0) { c->p = p + 4; return -1; }
    int neg = 0;
    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') p++;
    double v = 0;
    int saw_digit = 0;
    while (p < c->end && *p >= '0' && *p <= '9') {
        v = v * 10.0 + (*p - '0');
        p++; saw_digit = 1;
    }
    if (p < c->end && *p == '.') {
        p++;
        double frac = 0.1;
        while (p < c->end && *p >= '0' && *p <= '9') {
            v += frac * (*p - '0');
            frac *= 0.1;
            p++; saw_digit = 1;
        }
    }
    if (saw_digit && p < c->end && (*p == 'e' || *p == 'E')) {
        p++;
        int eneg = 0;
        if (p < c->end && *p == '-') { eneg = 1; p++; }
        else if (p < c->end && *p == '+') p++;
        int expv = 0;
        while (p < c->end && *p >= '0' && *p <= '9') { expv = expv * 10 + (*p - '0'); p++; }
        double mul = 1.0;
        if (eneg) while (expv-- > 0) mul *= 0.1;
        else while (expv-- > 0) mul *= 10.0;
        v *= mul;
    }
    c->p = p;
    if (!saw_digit) return -1;
    *out = neg ? -v : v;
    return 0;
}

static double js_sub_get_double(const char *s, int sl, const char *key, double def)
{
    char tmp[JS_SUB_TMP_SZ];
    int tl = js_sub_wrap(s, sl, tmp, (int)sizeof(tmp));
    js_ctx_t root = { tmp, tmp + tl };
    js_ctx_t vctx;
    if (js_find_key(&root, key, &vctx) != 0) return def;
    double v = def;
    if (js_parse_double_ctx(&vctx, &v) != 0) return def;
    return v;
}

/* ============================================================================
 *  JSON 帧分发处理（与 Qt 上位机 serial_manager.py 对齐）
 * ============================================================================ */
static void json_handle_D(const char *json, int jlen)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_stats.json_lines++;
    s_stats.json_D++;
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_stats.last_valid_frame_ms = now_ms();
    xSemaphoreGive(s_mtx);

    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);

    int s = js_get_int(json, jlen, "s", 0);
    int e = js_get_int(json, jlen, "e", RF_NUM_CHANNELS - 1);
    int r = js_get_int(json, jlen, "r", 2);
    int st = js_get_int(json, jlen, "st", 300);
    int sp = js_get_int(json, jlen, "sp", 2);

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_active_scan.start_ch  = s;
    s_active_scan.end_ch    = e;
    s_active_scan.rate_idx  = r;
    s_active_scan.settle_us = st;
    s_active_scan.spp       = sp;
    xSemaphoreGive(s_mtx);

    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);

    int vals[RF_NUM_CHANNELS + 8];
    int count = js_get_int_array(json, jlen, "v", vals, RF_NUM_CHANNELS + 8);

    if (count <= 0) {
        char warn[160];
        int ww = snprintf(warn, sizeof(warn),
                          "$D missing 'v'[] or empty (s=%d e=%d). Dropped.", s, e);
        if (ww < 0) ww = 0;
        if (ww >= (int)sizeof(warn)) ww = (int)sizeof(warn) - 1;
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, warn, ww);
    }

    /* 压缩到 s_rssi[125]：如果 values 是针对 [s..e] 子区间则居中填充，否则按 0..124 映射 */
    uint8_t rssi[RF_NUM_CHANNELS];
    memset(rssi, 0, sizeof(rssi));
    if (count > 0) {
        int span = e - s + 1;
        if (span < 1) span = 1;
        for (int i = 0; i < count; i++) {
            int v = vals[i];
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            int idx;
            if (count <= span) {
                idx = s + i * span / count;
            } else {
                idx = s + i;
            }
            if (idx >= 0 && idx < RF_NUM_CHANNELS) {
                if ((uint8_t)v > rssi[idx]) rssi[idx] = (uint8_t)v;
            }
        }
    }
    handle_spectrum(rssi, RF_NUM_CHANNELS);

    /* 同时写日志：缩略版 */
    log_push_data_line(RF_LOG_LINE_RX_DATA, "$D", json, s, e, r, st, sp, count);
}

static void json_handle_W(const char *json, int jlen)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_stats.json_lines++;
    s_stats.json_W++;
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_stats.last_valid_frame_ms = now_ms();
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    int b = js_get_int(json, jlen, "b", 5);
    int n = js_get_int(json, jlen, "n", 25);
    int c_vals[64];
    int count = js_get_int_array(json, jlen, "c", c_vals, 64);
    log_push_wifi5_line(RF_LOG_LINE_RX_WIFI5, b, n, count);
}

static void json_handle_A(const char *json, int jlen)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_stats.json_lines++;
    s_stats.json_A++;
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_stats.last_valid_frame_ms = now_ms();
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    int ok = js_get_bool_int(json, jlen, "ok", 0);
    if (ok) {
        /* 最近一次 cfg 若仍在发送队列则记已接受（此处简化：用 active_scan 同步） */
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        s_acked_cfg = s_active_scan;
        xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    }
    char line[RF_LOG_LINE_MAX];
    int w = snprintf(line, sizeof(line), "$A {ok:%s}", ok ? "true" : "false");
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    rf_log_push_impl(RF_LOG_LINE_RX_ACK, line, w);
}

static void json_handle_R(const char *json, int jlen)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_stats.json_lines++;
    s_stats.json_R++;
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_stats.last_valid_frame_ms = now_ms();
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    /* 解析每个无人机 -> 转成 rf_rid_wire_t -> handle_rid() */
    int n = js_get_int(json, jlen, "n", 0);
    if (n <= 0) n = js_get_drone_count(json, jlen);
    if (n <= 0) {
        rf_log_push_impl(RF_LOG_LINE_RX_RID, "$R {drones: empty}", -1);
        return;
    }

    int parsed_ok = 0;
    for (int i = 0; i < n; i++) {
        const char *ds = NULL;
        int dl = 0;
        if (js_nth_drone_sub(json, jlen, i, &ds, &dl) != 0 || ds == NULL || dl <= 0) {
            ESP_LOGW(TAG, "$R drone[%d] sub-extract FAIL (n=%d)", i, n);
            continue;
        }
        if (dl >= JS_SUB_TMP_SZ - 2) {
            ESP_LOGW(TAG, "$R drone[%d] sub too long: dl=%d (wrapper can hold %dB)", i, dl, JS_SUB_TMP_SZ - 2);
        }
        rf_rid_wire_t w;
        memset(&w, 0, sizeof(w));
        /* 基础：先取到 mac / rssi 用于日志，mac 不存进 wire，但用 self_id / operator 做填充 */
        char mac_s[32];
        mac_s[0] = 0;
        js_sub_get_cstr(ds, dl, "mac", mac_s, (int)sizeof(mac_s));
        int rssi = js_sub_get_int(ds, dl, "rssi", 0);
        (void)rssi;
        /* uas_id -> id (优先) / 或 self_id 回退 */
        char id_val[32]; id_val[0] = 0;
        char id_type_s[16]; id_type_s[0] = 0;
        char ua_type_s[24]; ua_type_s[0] = 0;
        char standard_s[16]; standard_s[0] = 0;
        char status_s[16]; status_s[0] = 0;
        char self_id_s[64]; self_id_s[0] = 0;
        char operator_s[128]; operator_s[0] = 0;
        js_sub_get_cstr(ds, dl, "id", id_val, (int)sizeof(id_val));
        js_sub_get_cstr(ds, dl, "id_type", id_type_s, (int)sizeof(id_type_s));
        js_sub_get_cstr(ds, dl, "ua_type", ua_type_s, (int)sizeof(ua_type_s));
        js_sub_get_cstr(ds, dl, "standard", standard_s, (int)sizeof(standard_s));
        js_sub_get_cstr(ds, dl, "status", status_s, (int)sizeof(status_s));
        js_sub_get_cstr(ds, dl, "self_id", self_id_s, (int)sizeof(self_id_s));
        js_sub_get_cstr(ds, dl, "operator", operator_s, (int)sizeof(operator_s));

        /* id_type: "SerialNumber"/"CAA"/"UAOID"/"Specific" → 返回 1..5，否则 0 */
        uint8_t uas_id_type = 0;
        if      (id_type_s[0] == 'S' && id_type_s[1] == 'e') uas_id_type = 1;  /* SerialNumber */
        else if (id_type_s[0] == 'C')                        uas_id_type = 2;  /* CAA Assigned */
        else if (id_type_s[0] == 'U' && id_type_s[2] == 'O') uas_id_type = 3;  /* UAOID */
        else if (id_type_s[0] == 'S' && id_type_s[1] == 'p') uas_id_type = 4;  /* Specific Session ID */
        w.uas_id_type = uas_id_type;

        /* 优先用 id，否则使用 mac 作为 uas_id（保证能在 s_drones 中区分） */
        char *use_id = id_val;
        if (use_id[0] == 0) use_id = mac_s;
        int uas_id_len = (int)strlen(use_id);
        if (uas_id_len > (int)sizeof(w.uas_id) - 1) uas_id_len = (int)sizeof(w.uas_id) - 1;
        memcpy(w.uas_id, use_id, (size_t)uas_id_len);
        w.uas_id[uas_id_len] = 0;

        /* op_desc：standard + status + ua_type 组合（简化），
           否则直接塞 self_id，长度优先保存 readable 的内容 */
        const char *desc_src = (self_id_s[0] != 0) ? self_id_s : ua_type_s;
        int desc_len = (int)strlen(desc_src);
        if (desc_len > (int)sizeof(w.op_desc) - 1) desc_len = (int)sizeof(w.op_desc) - 1;
        if (desc_len > 0) memcpy(w.op_desc, desc_src, (size_t)desc_len);
        w.op_desc[desc_len] = 0;

        /* operator_id：直接塞 operator 字段 */
        int op_len = (int)strlen(operator_s);
        if (op_len > (int)sizeof(w.operator_id) - 1) op_len = (int)sizeof(w.operator_id) - 1;
        if (op_len > 0) memcpy(w.operator_id, operator_s, (size_t)op_len);
        w.operator_id[op_len] = 0;

        /* 坐标：lat / lon / alt_p / alt_g / height */
        double lat = js_sub_get_double(ds, dl, "lat", -99999.0);
        double lon = js_sub_get_double(ds, dl, "lon", -99999.0);
        double alt_p = js_sub_get_double(ds, dl, "alt_p", -1000.0);
        double alt_g = js_sub_get_double(ds, dl, "alt_g", -1000.0);
        double height = js_sub_get_double(ds, dl, "height", -1000.0);
        double speed  = js_sub_get_double(ds, dl, "speed", 9999.0);   /* 无效值大哨兵 */
        double vspeed = js_sub_get_double(ds, dl, "vspeed", 9999.0);
        double heading= js_sub_get_double(ds, dl, "heading", 9999.0);
        double op_lat = js_sub_get_double(ds, dl, "op_lat", -99999.0);
        double op_lon = js_sub_get_double(ds, dl, "op_lon", -99999.0);

        if (lat < -90.0 || lat > 90.0) { w.lat_e7 = 0; } else { w.lat_e7 = (int32_t)(lat * 1e7); }
        if (lon < -180.0 || lon > 180.0) { w.lon_e7 = 0; } else { w.lon_e7 = (int32_t)(lon * 1e7); }
        if (op_lat < -90.0 || op_lat > 90.0) { w.operator_lat_e7 = 0; } else { w.operator_lat_e7 = (int32_t)(op_lat * 1e7); }
        if (op_lon < -180.0 || op_lon > 180.0) { w.operator_lon_e7 = 0; } else { w.operator_lon_e7 = (int32_t)(op_lon * 1e7); }
        w.alt_pressure_m = (alt_p  < -900) ? -999.0f : (float)alt_p;
        w.alt_geodetic_m = (alt_g  < -900) ? -999.0f : (float)alt_g;
        w.height_m       = (height < -900) ? -999.0f : (float)height;
        w.horiz_speed    = (speed  >= 254.9f || speed < -1e6) ? -1.0f : (float)speed;
        w.vert_speed     = (vspeed >= 62.9f  || vspeed < -1e6) ? -1.0f : (float)vspeed;
        w.ground_track   = (heading >= 360.0f || heading < -1e6) ? -1.0f : (float)heading;

        /* flags：根据 status 设置 UA 状态位
           注意 UI 判断:
             - Pilot 坐标显示: d->flags & 0x08 (bit3)
             - 高度显示: 无条件
           所以这里 bit3 = 操作员坐标有效; */
        uint16_t flags = 0;
        if (standard_s[0]) {
            if (standard_s[0] == 'A' && standard_s[1] == 'S')    flags |= (1U << 0);   /* ASTM F3411 */
            else if (standard_s[0] == 'G')                        flags |= (1U << 1);   /* GB 42590 */
            else if (standard_s[0] == 'D')                        flags |= (1U << 2);   /* DJI DroneID */
        }
        if (status_s[0]) {
            if      (status_s[0] == 'A' && status_s[1] == 'i') flags |= (1U << 4);  /* Airborne */
            else if (status_s[0] == 'G')                       flags |= (1U << 5);  /* Ground */
            else if (status_s[0] == 'E')                       flags |= (1U << 6);  /* Emergency */
        }
        /* UI bit3: Pilot 坐标有效 (rf_ui.c line 1239: d->flags & 0x08) */
        if (w.operator_lat_e7 != 0 || w.operator_lon_e7 != 0) flags |= (1U << 3);
        if (w.height_m >= -900.0f)                            flags |= (1U << 9);
        w.flags = flags;

        ESP_LOGI(TAG, "$R drone[%d] dl=%d mac=%s id=%s id_type=%s(%u) "
                      "lat=%.7f->e7=%ld lon=%.7f->e7=%ld "
                      "op_lat=%.7f->e7=%ld op_lon=%.7f->e7=%ld flags=0x%04X",
                 i, dl, mac_s, use_id, id_type_s, (unsigned)uas_id_type,
                 lat, (long)w.lat_e7, lon, (long)w.lon_e7,
                 op_lat, (long)w.operator_lat_e7, op_lon, (long)w.operator_lon_e7,
                 (unsigned)w.flags);

        handle_rid((const uint8_t *)&w, (uint16_t)sizeof(w));
        parsed_ok++;
    }

    /* 构造一条日志行 */
    char logbuf[RF_LOG_LINE_MAX];
    int lw = snprintf(logbuf, sizeof(logbuf), "$R drones=%d parsed=%d", n, parsed_ok);
    if (lw >= (int)sizeof(logbuf)) lw = (int)sizeof(logbuf) - 1;
    rf_log_push_impl(RF_LOG_LINE_RX_RID, logbuf, lw);
}

static void json_handle_R0(const char *json, int jlen)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_stats.json_lines++;
    s_stats.json_R0++;
    s_stats.rx_frames++;
    s_stats.link_online = true;
    s_stats.last_valid_frame_ms = now_ms();
    xSemaphoreGive(s_mtx);
    __atomic_fetch_add(&s_rf_version, 1, __ATOMIC_RELEASE);
    rf_rid0_frame_t f;
    memset(&f, 0, sizeof(f));
    f.rssi = (int8_t)js_get_int(json, jlen, "rssi", 0);
    f.channel = (uint8_t)js_get_int(json, jlen, "ch", 0);
    f.subtype = (uint8_t)js_get_int(json, jlen, "subtype", 0xFF);
    f.has_rid = (uint8_t)js_get_bool_int(json, jlen, "rid", 0);
    f.length = (uint16_t)js_get_int(json, jlen, "len", 0);
    js_get_cstr(json, jlen, "mac", f.mac, (int)sizeof(f.mac));
    int hl = js_get_cstr(json, jlen, "hex", f.hex, (int)sizeof(f.hex));
    f.hex_len = (hl > 0) ? (uint16_t)hl : 0;
    rid0_push_impl(&f);

    /* 日志缩略版 */
    char line[RF_LOG_LINE_MAX];
    int w = snprintf(line, sizeof(line),
        "$R0 %s ch=%u rssi=%d subtype=0x%02X rid=%u len=%u hex=%uB",
        f.mac[0] ? f.mac : "?", (unsigned)f.channel,
        (int)f.rssi, (unsigned)f.subtype, (unsigned)f.has_rid,
        (unsigned)f.length, (unsigned)(f.hex_len / 2));
    if (w >= (int)sizeof(line)) w = (int)sizeof(line) - 1;
    rf_log_push_impl(RF_LOG_LINE_RX_RID0, line, w);
}

static void json_process_line(const char *line, int len)
{
    if (len <= 0) return;
    /* 去掉尾部 \r */
    if (len > 0 && line[len - 1] == '\r') len--;
    if (len <= 0) return;

    /* Qt 上位机命令格式：前缀 + 空格 + JSON:
       "$D {...}"  "$W {...}"  "$R {...}"  "$R0 {...}"  "$A {...}"
       其它：普通日志行
       注意：先判 $R0（4 字节前缀），避免被 $R（3 字节前缀）抢占。 */
    if (len >= 4 && line[0] == '$' && line[1] == 'R' && line[2] == '0' && line[3] == ' ') {
        json_handle_R0(line + 4, len - 4);
    } else if (len >= 3 && line[0] == '$' && line[2] == ' ') {
        const char *json = line + 3;
        int jlen = len - 3;
        char t2 = (char)line[1];
        if (t2 == 'D') {
            json_handle_D(json, jlen);
        } else if (t2 == 'W') {
            json_handle_W(json, jlen);
        } else if (t2 == 'A') {
            json_handle_A(json, jlen);
        } else if (t2 == 'R') {
            json_handle_R(json, jlen);
        } else {
            /* 其他前缀：写入日志 */
            rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, len);
        }
    } else {
        /* 未识别前缀（无 $/无空格/非上述字母）：写入日志行 */
        rf_log_push_impl(RF_LOG_LINE_RX_LOG, line, len);
    }
}

/* 喂一个字符给 JSON 行解析器。遇到 '\n' 时分发；保护缓冲超长行（>511 字节）。 */
static void json_parser_feed(char ch)
{
    if (ch == '\n') {
        s_jp.buf[s_jp.len] = '\0';
        if (s_jp.len > 0) {
            json_process_line(s_jp.buf, (int)s_jp.len);
        }
        s_jp.len = 0;
        s_jp.overflow = 0;
        return;
    }
    if (ch == '\r') return;
    if (s_jp.len + 1 >= (uint16_t)sizeof(s_jp.buf)) {
        if (!s_jp.overflow) {
            s_jp.overflow = 1;
            rf_log_push_impl(RF_LOG_LINE_RX_LOG,
                             "[WARN] JSON line too long, truncated.", -1);
        }
        /* 丢尾部字节，保持缓冲 '\0' 不越界 */
        return;
    }
    s_jp.buf[s_jp.len++] = ch;
}
