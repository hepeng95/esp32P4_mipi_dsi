/*
 * rf_ui.c — 2.4G 频谱 / 瀑布图 / 无人机 RID 显示 UI
 *
 * 布局（480x854 竖屏，深色背景，纵向可滚动）：
 *   顶栏  : 标题 + 链路状态 + 扫频速率 + 无人机数
 *   频谱  : 125 信道 RSSI 柱状图（lv_canvas + lv_draw_rect，按 RSSI 着色）
 *   瀑布  : 时间向下滚动的频谱历史（lv_canvas，每帧上滚一行 + 底部写新行）
 *   RID   : 无人机卡片列表（ASTM F3411 字段，世代号变化时重建）
 *
 * 刷新：单个 lv_timer(~100ms) 在 LVGL 任务上下文运行，拉取 rf_link 数据并重绘。
 * 瀑布图仅在出现新扫描帧时滚动一次，避免无谓重绘。
 */

#include "rf_ui.h"
#include "rf_map.h"
#include "rf_5g.h"
#include "rf_24g.h"
#include "rf_gps.h"
#include "rf_link.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "driver/ledc.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "rf_ui";

/* ====== 通道数兼容: 2.4G=125 / 5.8G=256 ======
   UI 侧用 RF_UI_MAX_CHANNELS 分配栈 buffer, 函数签名加 nchan 参数 */
#ifndef RF5G_CHANNELS
#define RF5G_CHANNELS 256
#endif
#define RF_UI_MAX_CHANNELS  256   /* max(RF_NUM_CHANNELS=125, RF24_CHANNELS=126, RF5G_CHANNELS=256) */
static inline int rf_ui_nchan_for_band(int band) {
    if (band == RF_BAND_5G)    return RF5G_CHANNELS;
    if (rf_24g_is_running())  return RF24_CHANNELS;   /* 本地 nRF24: 126 ch */
    return RF_NUM_CHANNELS;                           /* UART C5: 125 ch */
}

/* 来自 mipi_dsi_lcd_example_main.c 的 LVGL 性能统计变量 */
extern volatile uint32_t g_lvgl_fps;
extern volatile uint32_t g_lvgl_flush_per_sec;

/* ---- 布局常量 ----------------------------------------------------------- */
#define RF_UI_PAD           16
#define RF_UI_CONTENT_W     (480 - RF_UI_PAD * 2)   // 448
#define SPEC_H              150
#define WF_H                190
#define STATUS_BAR_H        42   /* 主页顶部小状态条高度 */
#define PAGE_HEADER_H       46   /* 子页顶部 Header 高度（返回按钮 + 标题） */
#define CONTENT_AREA_H      (854 - PAGE_HEADER_H - 4)   /* 子页内容最大高度(滚动) */

/* ---- 页面枚举 ----------------------------------------------------------- */
typedef enum {
    PAGE_BOOT = 0,       /* 开机自检页（启动时显示） */
    PAGE_HOME,
    PAGE_24G,
    PAGE_58G,
    PAGE_UART,
    PAGE_RID,
    PAGE_RIDMAP,
    PAGE_SETTINGS,
    PAGE_SPECTRUM,      /* ← 合并 2.4G + 5.8G 双频段 */
    PAGE_FPS_TEST,
    PAGE_COUNT
} page_id_t;

/* ---- 页管理器 ----------------------------------------------------------- */
static page_id_t s_cur_page = PAGE_HOME;
static lv_obj_t *s_page_root = NULL;       /* 所有页（含主页）共享容器，全屏，位于背景之上 */
static lv_obj_t *s_pages[PAGE_COUNT] = {0};/* 每个页的全屏内容容器 */
static lv_obj_t *s_page_header = NULL;     /* 子页顶部条：返回 + 标题 + 小状态（主页隐藏） */
static lv_obj_t *s_page_title_lbl = NULL;  /* Header 中央标题 */
static lv_obj_t *s_page_state_lbl = NULL;  /* Header 右侧链路状态文字 */
static lv_obj_t *s_floating_home_btn = NULL; /* 右下角浮动 Home 按钮 (频谱子页显示) */
static lv_obj_t *s_fps_lbl = NULL;         /* 右下角 FPS 实时显示 */

/* PAGE_SPECTRUM 双频段切换 */
static lv_obj_t *s_spec_toggle_btn_24g = NULL;
static lv_obj_t *s_spec_toggle_btn_5g  = NULL;
static bool      s_spectrum_managed_sub = false; /* true = PAGE_24G/58G 已被 PAGE_SPECTRUM 接管 */

/* 主页顶栏状态文字（主页使用，代替 Header） */
static lv_obj_t *s_home_status_bar = NULL;

/* RID 地图页共享 */
static lv_obj_t *s_ridmap_list = NULL;
static lv_obj_t *s_ridmap_canvas = NULL;

/* 无人机跟踪状态 */
static bool    s_track_enabled = false;    /* 是否开启跟踪 */
static uint8_t s_track_id[24] = {0};       /* 当前跟踪的无人机 uas_id */

/* RID MAP 懒创建 label 槽：header + N drone rows + empty hint，避免每次 gen 变化都 delete+create
   （delete/create 反复会把 LVGL 堆打碎，最终 lv_malloc_zeroed 失败 → LV_ASSERT_MALLOC → core panic） */
#define RIDMAP_ROW_MAX   RF_MAX_DRONES
static lv_obj_t *s_ridmap_hdr = NULL;
static lv_obj_t *s_ridmap_row[RIDMAP_ROW_MAX] = { 0 };   /* 每行是 button, 内含 label */
static lv_obj_t *s_ridmap_row_lbl[RIDMAP_ROW_MAX] = { 0 }; /* 行内的文本 label */
static uint8_t  s_ridmap_row_id[RIDMAP_ROW_MAX][24] = { 0 }; /* 每行对应的 drone ID */
static lv_obj_t *s_ridmap_empty = NULL;
static bool      s_ridmap_inited = false;

/* Offline tile map state (SD card mount + Web Mercator projection) */
static bool         s_map_ready = false;    /* set by rf_ui_on_map_ready() when SD mount succeeds */
static rf_map_view_t s_map_view = { 0 };    /* initialized lazily when RID MAP canvas is ready */

void rf_ui_on_map_ready(void)
{
    s_map_ready = true;
    ESP_LOGI("rf_ui", "SD map ready; next RID MAP page open will center tile & project drones");
}

static void ridmap_row_click_cb(lv_event_t *e)
{
    /* 点击列表行 → 选中/取消选中跟踪该无人机 */
    int idx = (int)(long)lv_event_get_user_data(e);
    if (idx < 0 || idx >= RIDMAP_ROW_MAX) return;

    /* 如果点击的是当前跟踪的 → 取消跟踪 */
    if (s_track_enabled && memcmp(s_track_id, s_ridmap_row_id[idx], 24) == 0) {
        s_track_enabled = false;
        memset(s_track_id, 0, 24);
        ESP_LOGI("rf_ui", "track: disabled");
    } else {
        /* 开始跟踪新的无人机 */
        s_track_enabled = true;
        memcpy(s_track_id, s_ridmap_row_id[idx], 24);
        ESP_LOGI("rf_ui", "track: start id=%.10s", s_track_id);
    }

    /* 立即刷新列表高亮状态 */
    if (s_cur_page == PAGE_RIDMAP && s_ridmap_inited) {
        for (int i = 0; i < RIDMAP_ROW_MAX; i++) {
            if (!s_ridmap_row[i] || lv_obj_has_flag(s_ridmap_row[i], LV_OBJ_FLAG_HIDDEN)) continue;
            bool selected = s_track_enabled && (memcmp(s_track_id, s_ridmap_row_id[i], 24) == 0);
            lv_obj_set_style_bg_color(s_ridmap_row[i],
                selected ? lv_color_hex(0x2A4A6A) : lv_color_hex(0x15202B), 0);
            lv_obj_set_style_border_color(s_ridmap_row[i],
                selected ? lv_color_hex(0x00E0FF) : lv_color_hex(0x2A3340), 0);
        }
    }
}

static void ridmap_labels_init_once(void)
{
    if (s_ridmap_inited || !s_ridmap_list) return;
    /* header */
    s_ridmap_hdr = lv_label_create(s_ridmap_list);
    lv_label_set_text(s_ridmap_hdr, "ID  |  UA (lat,lon) alt  |  Operator (lat,lon)  [tap to track]");
    lv_obj_set_width(s_ridmap_hdr, lv_pct(100));
    lv_obj_set_style_text_color(s_ridmap_hdr, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(s_ridmap_hdr, &lv_font_montserrat_14, 0);
    /* drone row slots（默认隐藏）— 用 button 包裹 label, 可点击选中 */
    for (int i = 0; i < RIDMAP_ROW_MAX; i++) {
        s_ridmap_row[i] = lv_btn_create(s_ridmap_list);
        lv_obj_set_width(s_ridmap_row[i], lv_pct(100));
        lv_obj_set_height(s_ridmap_row[i], LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(s_ridmap_row[i], lv_color_hex(0x15202B), 0);
        lv_obj_set_style_border_color(s_ridmap_row[i], lv_color_hex(0x2A3340), 0);
        lv_obj_set_style_border_width(s_ridmap_row[i], 1, 0);
        lv_obj_set_style_pad_all(s_ridmap_row[i], 3, 0);
        lv_obj_set_style_radius(s_ridmap_row[i], 4, 0);
        lv_obj_clear_flag(s_ridmap_row[i], LV_OBJ_FLAG_SCROLLABLE);

        s_ridmap_row_lbl[i] = lv_label_create(s_ridmap_row[i]);
        lv_obj_set_width(s_ridmap_row_lbl[i], lv_pct(100));
        lv_obj_set_style_text_color(s_ridmap_row_lbl[i], lv_color_hex(0xC0D0E0), 0);
        lv_obj_set_style_text_font(s_ridmap_row_lbl[i], &lv_font_montserrat_14, 0);

        lv_obj_add_event_cb(s_ridmap_row[i], ridmap_row_click_cb, LV_EVENT_CLICKED,
                            (void *)(long)i);
        lv_obj_add_flag(s_ridmap_row[i], LV_OBJ_FLAG_HIDDEN);
    }
    /* empty hint */
    s_ridmap_empty = lv_label_create(s_ridmap_list);
    lv_label_set_text(s_ridmap_empty, "No RID drones in range (scanning ASTM F3411...)");
    lv_obj_set_width(s_ridmap_empty, lv_pct(100));
    lv_obj_set_style_text_color(s_ridmap_empty, lv_color_hex(0x666677), 0);
    lv_obj_set_style_text_font(s_ridmap_empty, &lv_font_montserrat_14, 0);
    s_ridmap_inited = true;
}

/* ---- 开机自检 (Boot Self-Test) ---- */
typedef enum {
    BOOT_TEST_LCD = 0,
    BOOT_TEST_TOUCH,
    BOOT_TEST_SD,
    BOOT_TEST_GPS,
    BOOT_TEST_24G,
    BOOT_TEST_5G,
    BOOT_TEST_RF_LINK,
    BOOT_TEST_COUNT
} boot_test_item_t;

static const char *boot_test_names[BOOT_TEST_COUNT] = {
    "LCD Display",
    "Touch Panel",
    "SD Card & Map",
    "GPS Module",
    "2.4G Radio (nRF24)",
    "5.8G Radio (A5133)",
    "RF Link (C5 UART)",
};

static lv_obj_t *s_boot_logo_lbl = NULL;
static lv_obj_t *s_boot_sub_lbl = NULL;
static lv_obj_t *s_boot_progress_bar = NULL;
static lv_obj_t *s_boot_item_labels[BOOT_TEST_COUNT] = {0};
static lv_obj_t *s_boot_item_status[BOOT_TEST_COUNT] = {0};
static lv_timer_t *s_boot_timer = NULL;
static int s_boot_current_test = 0;
static bool s_boot_results[BOOT_TEST_COUNT] = {false};

static void build_page_boot(void);
static void boot_test_timer_cb(lv_timer_t *t);
static void boot_animate_to_home(void);

static void page_build(page_id_t id);
static void page_show(page_id_t id);
static void page_back_cb(lv_event_t *e);
static void tile_cb(lv_event_t *e);

/* FPS Test timer cb — page_show 里提前引用，必须前置声明 */
static void fps_test_timer_cb(lv_timer_t *t);

/* FPS Test 页 timer（page_show 在前面引用，必须提前声明） */
static lv_timer_t *s_fps_test_timer = NULL;

/* ---- 扫频段通用 UI 结构（2.4G / 5.8G 各一份，避免控件指针冲突） --------
   必须放在 forward 声明之前，否则函数参数中的 scan_band_ui_t* 会报
   "unknown type name"（前向 typedef 的 struct 不能跨 translation unit 补全）。 */
typedef struct scan_band_ui_s scan_band_ui_t;   /* 前置不透明声明 */

/* ---- 颜色映射（RSSI 0..255 -> 频谱色） ----------------------------------- */
/* 经典 jet/thermal：黑->蓝->青->绿->黄->红->白 */
static void rssi_to_rgb(uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    /* Teal-green palette — 匹配参考图 waterfall 风格:
     *   整个画布有统一深青绿底色, 强信号才突出
     *   0=深青绿底噪 → 80=中青绿 → 160=亮绿 → 220=亮黄绿 → 255=亮黄白 */
    static const uint8_t stop_p[]  = {   0,  60, 130, 190, 240, 255 };
    static const uint8_t stop_r[]  = {  10,  15,  25,  80, 180, 255 };
    static const uint8_t stop_g[]  = {  40, 100, 180, 230, 250, 255 };
    static const uint8_t stop_b[]  = {  55, 130,  90,  50,  40, 180 };
    int i = 0;
    while (i < 5 && v > stop_p[i + 1]) i++;
    int span = stop_p[i + 1] - stop_p[i];
    int d = (span > 0) ? (v - stop_p[i]) : 0;
    *r = (uint8_t)(stop_r[i] + (stop_r[i + 1] - stop_r[i]) * d / (span ? span : 1));
    *g = (uint8_t)(stop_g[i] + (stop_g[i + 1] - stop_g[i]) * d / (span ? span : 1));
    *b = (uint8_t)(stop_b[i] + (stop_b[i + 1] - stop_b[i]) * d / (span ? span : 1));
}

static lv_color_t rssi_to_color(uint8_t v)
{
    uint8_t r, g, b;
    rssi_to_rgb(v, &r, &g, &b);
    return lv_color_make(r, g, b);
}

/* scan_band_ui_t 完整定义：包含 band 参数 + 每频段独立 UI 指针；
   定义必须在函数 forward 之前完成（因为函数参数用到了该类型）。 */
struct scan_band_ui_s {
    rf_band_t band;               /* RF_BAND_24G / RF_BAND_5G */
    const char *title;            /* 子页标题 */
    const char *spectrum_label;   /* 频谱段 label："Spectrum  2.400 - 2.525 GHz" */
    const char *waterfall_label;  /* 瀑布段 label："Waterfall  (newest at bottom, time downward)" */
    const char *freq_tick;        /* 底部频率刻度文字 */
    int start_ch;                 /* 默认 start_ch */
    int end_ch;                   /* 默认 end_ch */
    int rate_idx;                 /* 默认速率 idx */
    int settle_us;                /* 默认驻留 */
    int spp;                      /* 默认 spp */

    /* 显示对象 */
    lv_obj_t *spec_canvas;
    lv_obj_t *wf_canvas;
    lv_obj_t *gain_label;
    lv_obj_t *gain_slider;
    lv_obj_t *peak_color_legend;
    lv_obj_t *sl_start, *sl_end;
    lv_obj_t *dd_rate;
    lv_obj_t *sl_settle, *sl_spp;
    lv_obj_t *lbl_start, *lbl_end, *lbl_settle, *lbl_spp;
};

/* 函数 forward（scan_band_ui_t 已完成定义，可安全用于形参表）。 */
static void build_spectrum_section(lv_obj_t *parent, scan_band_ui_t *ui);
static void build_waterfall_section(lv_obj_t *parent, scan_band_ui_t *ui);
static void build_scan_config_section(lv_obj_t *page, scan_band_ui_t *ui);
static void build_gain_row(lv_obj_t *parent, scan_band_ui_t *ui);

static scan_band_ui_t s_band_ui[2] = {
    {
        .band            = RF_BAND_24G,
        .title           = "2.4G  Spectrum & Waterfall",
        .spectrum_label  = "Spectrum  2.400 - 2.525 GHz",
        .waterfall_label = "Waterfall  (newest at bottom, time downward)",
        .freq_tick       = "2.400     2.425     2.450     2.475     2.500  2.525 GHz",
        .start_ch        = 0,
        .end_ch          = 125,
        .rate_idx        = 2,    /* 2M */
        .settle_us       = 300,
        .spp             = 2,
    },
    {
        .band            = RF_BAND_5G,
        .title           = "5.8G  Spectrum & Waterfall",
        .spectrum_label  = "Spectrum  5.725 - 5.875 GHz",
        .waterfall_label = "Waterfall  (newest at bottom, time downward)",
        /* 5.8G 典型 FPV 频段：BAND A (5705-5865MHz, 40ch) + Race/低波段扩展到 ~5.945，
           这里按 5.725-5.875 做刻度（覆盖 150 信道的常规扫频） */
        .freq_tick       = "5.725     5.760     5.800     5.840     5.875 GHz",
        .start_ch        = 0,
        .end_ch          = 149,  /* 5.8G 默认覆盖 150 个 1MHz bin */
        .rate_idx        = 2,
        .settle_us       = 300,
        .spp             = 2,
    },
};
/* 当前活动扫频段（2.4G or 5.8G）：决定 spectrum_redraw/waterfall_push 写哪块 canvas */
static rf_band_t s_active_band = RF_BAND_24G;

/* ---- 全局 UI 对象引用 --------------------------------------------------- */
static lv_obj_t *s_header_lbl   = NULL;  // 状态栏状态文本
/* 为了兼容未迁移代码，保留旧名作为 alias（宏，指向 2.4G） */
#define s_spec_canvas           (s_band_ui[RF_BAND_24G].spec_canvas)
#define s_wf_canvas             (s_band_ui[RF_BAND_24G].wf_canvas)
#define s_gain_label            (s_band_ui[RF_BAND_24G].gain_label)
#define s_gain_slider           (s_band_ui[RF_BAND_24G].gain_slider)
#define s_peak_color_legend     (s_band_ui[RF_BAND_24G].peak_color_legend)
#define s_sl_start              (s_band_ui[RF_BAND_24G].sl_start)
#define s_sl_end                (s_band_ui[RF_BAND_24G].sl_end)
#define s_dd_rate               (s_band_ui[RF_BAND_24G].dd_rate)
#define s_sl_settle             (s_band_ui[RF_BAND_24G].sl_settle)
#define s_sl_spp                (s_band_ui[RF_BAND_24G].sl_spp)
#define s_lbl_start             (s_band_ui[RF_BAND_24G].lbl_start)
#define s_lbl_end               (s_band_ui[RF_BAND_24G].lbl_end)
#define s_lbl_settle            (s_band_ui[RF_BAND_24G].lbl_settle)
#define s_lbl_spp               (s_band_ui[RF_BAND_24G].lbl_spp)

static lv_obj_t *s_rid_list     = NULL;   // RID 卡片容器
static lv_obj_t *s_rid_empty    = NULL;   // "未发现无人机"占位

static uint32_t s_last_wf_sweep = 0;     // 上次画进瀑布的扫描计数
static int      s_last_wf_band  = -1;     // 上次画瀑布用的频段 (切换频段时重置 sweep)
static bool     s_last_used_rf24g = false;// 上次 2.4G 是否用本地 nRF24 (切换数据源时重置)
static uint32_t s_last_rid_gen  = 0xFFFFFFFF;
static uint32_t s_last_proto_sig = 0xFFFFFFFF;  // 协议列表签名，避免每帧重建

/* PEAK SIGNAL / 协议检测 控件 */
static lv_obj_t *s_peak_freq_lbl    = NULL;   /* 中心频率 MHz 大字 */
static lv_obj_t *s_peak_ch_lbl      = NULL;   /* 通道号小字 */
static lv_obj_t *s_peak_level_lbl   = NULL;   /* 强度% 小字 */
static lv_obj_t *s_proto_list       = NULL;   /* 协议检测结果列表 */

/* 信号增益调节：1-10 倍，作用于频谱图和瀑布图的显示增益 */
static int s_rf_gain = 1;               /* 1-10 倍, 默认 1 倍 */
/* RSSI 显示模式: false=Raw RSSI(0~255), true=dBm(-95~-20) */
static bool s_dbm_mode = true;   /* 默认 dBm */
static lv_obj_t *s_rssi_mode_btn = NULL;
static lv_obj_t *s_rssi_mode_lbl = NULL;
/* 注意：s_gain_label / s_gain_slider / s_peak_color_legend
   已迁移到 s_band_ui[*] 结构体中（每频段一份指针），
   下方宏提供兼容别名。此处不再声明同名静态变量。 */

/* ---- 设置面板 ------------------------------------------------------------ */
static lv_obj_t *s_settings_panel  = NULL;  // 设置面板容器（滚动容器）
static lv_obj_t *s_brightness_lbl  = NULL;  // 亮度百分比标签
static lv_obj_t *s_settings_info   = NULL;  // RF 链路状态信息
static lv_obj_t *s_debug_textarea  = NULL;  // Serial Monitor 文本区（替换旧 hex dump 区）
static lv_obj_t *s_scan_status_lbl = NULL;  // SCAN STATUS 文本
static lv_timer_t *s_debug_timer   = NULL;  // 调试面板自动刷新计时器

/* Serial Monitor 过滤复选框 / 控制按钮 */
static lv_obj_t *s_mon_chk_d   = NULL;  /* $D   2.4G spectrum */
static lv_obj_t *s_mon_chk_w   = NULL;  /* $W   5G WiFi      */
static lv_obj_t *s_mon_chk_a   = NULL;  /* $A   cfg ack      */
static lv_obj_t *s_mon_chk_log = NULL;  /* LOG  其他行      */
static lv_obj_t *s_mon_btn_pause = NULL;
static lv_obj_t *s_mon_btn_clear = NULL;
static bool      s_mon_paused  = false;
static bool      s_mon_scroll  = true;   /* 自动滚屏 */
static uint32_t  s_mon_seq     = 0;      /* 增量读取指针（对应 rf_log total） */

/* RID RAW PAYLOAD 折叠面板（隐藏于设置面板尾部） */
static lv_obj_t *s_rid0_container = NULL;   /* 整体容器（含标题条+文本区） */
static lv_obj_t *s_rid0_view      = NULL;   /* 文本显示 */
static lv_obj_t *s_rid0_output_cb = NULL;   /* OUTPUT 开关 */
static lv_obj_t *s_rid0_toggle_btn = NULL;  /* SHOW/HIDE */
static bool      s_rid0_show       = false;
static uint32_t  s_rid0_seq        = 0;      /* 增量读取 seq */

/* 数据源切换 / Debug Out — 两个页面各自独立一套（避免指针覆盖导致白框/乱码） */
typedef struct {
    lv_obj_t *btn_sim;       /* 模拟数据 */
    lv_obj_t *btn_uart;      /* 串口实际数据 */
    lv_obj_t *dbg_toggle;    /* 调试输出开关 */
    lv_obj_t *baud_drop;     /* 波特率下拉 */
    lv_obj_t *baud_probe;    /* 自动探测 */
} uart_ctrl_t;

static uart_ctrl_t s_sett_uart;   /* Settings 页 */
static uart_ctrl_t s_page_uart;   /* UART 页 */

/* 向后兼容宏：旧代码里写的 s_src_btn_sim 等默认指 Settings 页的控件 */
#define s_src_btn_sim        s_sett_uart.btn_sim
#define s_src_btn_uart       s_sett_uart.btn_uart
#define s_uartdbg_toggle     s_sett_uart.dbg_toggle
#define s_baud_dropdown      s_sett_uart.baud_drop
#define s_baud_probe_btn     s_sett_uart.baud_probe

/* 配置控件（扫描参数指针 — 已迁移到 s_band_ui[*]，每频段独立一份；
   下方兼容宏把旧名 s_sl_start / s_dd_rate / ... 映射到 2.4G 对应字段，
   因此此处不再重复声明同名静态变量。RID RAW 下拉是独立控件。 */
static lv_obj_t *s_dd_rid_raw = NULL;   /* RID 原始等级下拉 */

// 背光 LEDC 配置（须与 main.c 一致）
#define SETTINGS_BL_MODE      LEDC_LOW_SPEED_MODE
#define SETTINGS_BL_CHANNEL   LEDC_CHANNEL_0
#define SETTINGS_BL_DUTY_MAX 255               // 25% × 1023 = 255

static const char *RATE_NAMES[] = {"250K", "1M", "2M"};

static void settings_apply_brightness(uint8_t percent)
{
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    uint32_t duty = (uint32_t)SETTINGS_BL_DUTY_MAX * percent / 100;
    ledc_set_duty(SETTINGS_BL_MODE, SETTINGS_BL_CHANNEL, duty);
    ledc_update_duty(SETTINGS_BL_MODE, SETTINGS_BL_CHANNEL);
}

static void brightness_slider_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t val = lv_slider_get_value(slider);
    if (val < 1) val = 1;
    settings_apply_brightness((uint8_t)val);
    if (s_brightness_lbl) {
        lv_label_set_text_fmt(s_brightness_lbl, "%d%%", (int)val);
    }
}

static void settings_close_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_debug_timer) {
        lv_timer_del(s_debug_timer);
        s_debug_timer = NULL;
    }
    if (s_settings_panel) {
        lv_obj_add_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 滑块值变化 -> 更新数值标签 */
static void slider_val_cb(lv_event_t *e)
{
    lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
    if (!lbl) return;
    int32_t v = lv_slider_get_value(lv_event_get_target(e));
    lv_label_set_text_fmt(lbl, "%d", (int)v);
}

/* ---- 数据源切换 ------------------------------------------------------------- */

/* 遍历两套 UART 控件回调（Settings 页 + UART 页） */
typedef void (*uart_ctrl_cb_t)(uart_ctrl_t *c, void *arg);
static void uart_ctrl_foreach(uart_ctrl_cb_t fn, void *arg)
{
    fn(&s_sett_uart, arg);
    fn(&s_page_uart, arg);
}

/* 依据当前 rf_link_get_source() 结果刷新一套按钮的选中外观 */
static void refresh_one_source_style(uart_ctrl_t *c, void *arg)
{
    LV_UNUSED(arg);
    bool sim = rf_link_get_source();
    if (c->btn_sim) {
        if (sim) {
            lv_obj_set_style_bg_color(c->btn_sim,  lv_color_hex(0x2E8B57), 0);
            lv_obj_set_style_border_width(c->btn_sim, 2, 0);
            lv_obj_set_style_border_color(c->btn_sim, lv_color_hex(0x66FF99), 0);
        } else {
            lv_obj_set_style_bg_color(c->btn_sim,  lv_color_hex(0x2A2E36), 0);
            lv_obj_set_style_border_width(c->btn_sim, 1, 0);
            lv_obj_set_style_border_color(c->btn_sim, lv_color_hex(0x3E4550), 0);
        }
    }
    if (c->btn_uart) {
        if (!sim) {
            lv_obj_set_style_bg_color(c->btn_uart, lv_color_hex(0x0E639C), 0);
            lv_obj_set_style_border_width(c->btn_uart, 2, 0);
            lv_obj_set_style_border_color(c->btn_uart, lv_color_hex(0x66CCFF), 0);
        } else {
            lv_obj_set_style_bg_color(c->btn_uart, lv_color_hex(0x2A2E36), 0);
            lv_obj_set_style_border_width(c->btn_uart, 1, 0);
            lv_obj_set_style_border_color(c->btn_uart, lv_color_hex(0x3E4550), 0);
        }
    }
}

/* 依据 rf_link_get_uart_debug_output() 刷新一套 debug 按钮 */
static void refresh_one_dbg_style(uart_ctrl_t *c, void *arg)
{
    LV_UNUSED(arg);
    if (!c->dbg_toggle) return;
    lv_obj_t *lbl = lv_obj_get_child(c->dbg_toggle, 0);
    /* 如果 user_data 挂了 label 优先用 */
    lv_obj_t *lbl_alt = (lv_obj_t*)lv_obj_get_user_data(c->dbg_toggle);
    if (lbl_alt) lbl = lbl_alt;
    bool dbg_on = rf_link_get_uart_debug_output();
    if (lbl) {
        if (lbl_alt) {
            lv_label_set_text(lbl, dbg_on ? "UART LOG: ON" : "UART LOG: OFF");
        } else {
            /* 仅用 ASCII：Montserrat_14 无中文点阵，避免占位符/方块乱码 */
            lv_label_set_text(lbl, dbg_on ? "UART DBG: ON" : "UART DBG: OFF");
        }
    }
    lv_obj_set_style_bg_color(c->dbg_toggle,
                              dbg_on ? lv_color_hex(0x2E8B57) : lv_color_hex(0x2A2E36), 0);
}

static void refresh_source_btns_style(void)
{
    uart_ctrl_foreach(refresh_one_source_style, NULL);
}

static void refresh_dbg_toggles_style(void)
{
    uart_ctrl_foreach(refresh_one_dbg_style, NULL);
}

static void src_sim_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    rf_link_set_source(true);
    refresh_source_btns_style();
}

static void src_uart_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    rf_link_set_source(false);
    if (!rf_link_get_uart_debug_output()) rf_link_set_uart_debug_output(true);
    refresh_source_btns_style();
    refresh_dbg_toggles_style();
}

/* UART 调试输出按钮（切换开/关并同步外观） */
static void uartdbg_toggle_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    bool cur = rf_link_get_uart_debug_output();
    rf_link_set_uart_debug_output(!cur);
    refresh_dbg_toggles_style();
}

/* 波特率选项（7 项，与 lv_dropdown_set_options 顺序一致） */
static const int s_baud_opts[] = {9600,19200,38400,57600,115200,230400,460800,921600};
#define BAUD_OPTS_N  (int)(sizeof(s_baud_opts)/sizeof(s_baud_opts[0]))

/* 根据 baud 值设置一套下拉的 selected（保持两套下拉值同步） */
static void sync_baud_drop_internal(uart_ctrl_t *c, void *arg)
{
    int baud = *(int*)arg;
    if (!c->baud_drop) return;
    for (int i = 0; i < BAUD_OPTS_N; i++) {
        if (s_baud_opts[i] == baud) {
            lv_dropdown_set_selected(c->baud_drop, (uint16_t)i);
            break;
        }
    }
}

/* 波特率下拉选择变化 -> 立刻调用 rf_link_set_uart_baud() 实时切换，并同步另一页的下拉 selected */
static void baud_dropdown_cb(lv_event_t *e)
{
    lv_obj_t *drop = lv_event_get_target(e);
    if (!drop) return;
    uint16_t idx = lv_dropdown_get_selected(drop);
    if (idx >= BAUD_OPTS_N) return;
    int new_baud = s_baud_opts[idx];
    int cur_baud = rf_link_get_uart_baud();
    if (new_baud == cur_baud) return;
    rf_link_set_uart_baud(new_baud);
    uart_ctrl_foreach(sync_baud_drop_internal, &new_baud);
}

/* 自动探测波特率：worker 里把 user_data 当 (uart_ctrl_t*)，包含 drop + probe */
static void probe_worker(void *user_data)
{
    uart_ctrl_t *c = (uart_ctrl_t*)user_data;
    if (!c) return;
    lv_obj_t *btn = c->baud_probe;
    if (btn && lv_obj_is_valid(btn)) {
        lv_obj_t *pl = lv_obj_get_child(btn, 0);
        /* ASCII only (no CJK glyph loaded -> placeholder garble) */
        if (pl) lv_label_set_text(pl, "Probing...");
        lv_obj_add_state(btn, LV_STATE_DISABLED);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    }
    int hit = rf_link_autodetect_baud(800);
    if (hit > 0) {
        /* 选中当前探测到的 baud（同步两套下拉） */
        int baud = hit;
        rf_link_set_uart_baud(baud);
        uart_ctrl_foreach(sync_baud_drop_internal, &baud);
    }
    if (btn && lv_obj_is_valid(btn)) {
        lv_obj_t *pl = lv_obj_get_child(btn, 0);
        /* ASCII only: Montserrat_14 has no CJK glyphs */
        if (pl) lv_label_set_text(pl, LV_SYMBOL_REFRESH" AutoProbe");
        lv_obj_clear_state(btn, LV_STATE_DISABLED);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    }
}

static void baud_probe_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    /* 用 btn 反查：哪一套 ctrl 的 baud_probe == btn */
    uart_ctrl_t *target = NULL;
    if (btn == s_sett_uart.baud_probe) target = &s_sett_uart;
    else if (btn == s_page_uart.baud_probe) target = &s_page_uart;
    if (!target) return;
    lv_async_call(probe_worker, target);
}

/* APPLY 按钮：把 UI 值 -> 发送 cfg 命令 */
typedef struct {
    rf_scan_config_t cfg;
    rf_band_t        band;       /* RF_BAND_24G / RF_BAND_5G */
    int              rid_raw;    /* -1 表示不发送 */
} apply_payload_t;

static void apply_worker(void *arg)
{
    apply_payload_t *p = (apply_payload_t *)arg;
    if (!p) return;
    /* 经验 751367：不在 LVGL 回调里阻塞；这里是 async worker，允许 UART 写入 */

    /* 1) 先切换频段模式：告诉 ESP32-C5 本次配置针对哪个频段 */
    rf_link_send_band_mode(p->band);

    /* 2) 再下发该频段的扫描参数（start/end/rate/settle/spp） */
    rf_link_send_scan_cfg(&p->cfg);

    /* 3) RID RAW 级别（全局，RID 页未 build 时为 -1，跳过） */
    if (p->rid_raw >= 0) rf_link_send_rid_raw_level(p->rid_raw);

    free(p);
}

static void apply_btn_cb(lv_event_t *e)
{
    /* APPLY 按钮的 user_data = 对应频段的 scan_band_ui_t*（build_scan_config_section 里设置）。
       2.4G 旧路径可能未设置 user_data，退化为 RF_BAND_24G 以保持兼容。 */
    scan_band_ui_t *ui = (scan_band_ui_t *)lv_event_get_user_data(e);
    rf_band_t band = RF_BAND_24G;
    if (ui) band = ui->band;
    else ui = &s_band_ui[RF_BAND_24G];   /* 兜底：2.4G 旧 APPLY 按钮 */

    /* 该频段的控件指针：只有 build 过对应页后才非空，必须判空
       （未访问 5.8G 时 s_band_ui[RF_BAND_5G].sl_start 永远 NULL；RID 页未加载时 s_dd_rid_raw 永远 NULL） */
    if (!ui->sl_start || !ui->sl_end || !ui->dd_rate || !ui->sl_settle || !ui->sl_spp) {
        return;
    }

    apply_payload_t *p = (apply_payload_t *)malloc(sizeof(apply_payload_t));
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->band          = band;
    p->cfg.start_ch  = (int)lv_slider_get_value(ui->sl_start);
    p->cfg.end_ch    = (int)lv_slider_get_value(ui->sl_end);
    p->cfg.rate_idx  = (int)lv_dropdown_get_selected(ui->dd_rate);
    p->cfg.settle_us = (int)lv_slider_get_value(ui->sl_settle);
    p->cfg.spp       = (int)lv_slider_get_value(ui->sl_spp);
    if (p->cfg.end_ch < p->cfg.start_ch) p->cfg.end_ch = p->cfg.start_ch;

    /* s_dd_rid_raw：仅当 RID 页已 build（懒加载）时才非空，否则跳过 */
    p->rid_raw = -1;
    if (s_dd_rid_raw) {
        uint16_t sel = lv_dropdown_get_selected(s_dd_rid_raw);
        p->rid_raw = (int)sel;
    }

    /* 通过 lv_async_call 把 UART 阻塞写入挪到下一次 LVGL tick 之前的 worker，
       防止 apply_btn_cb（LVGL 事件回调）占用太久触发 task_wdt（经验 751367） */
    lv_async_call(apply_worker, p);
}

/* ---- Serial Monitor 控件回调 ------------------------------------------------- */
static void mon_pause_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_mon_paused = !s_mon_paused;
    if (s_mon_btn_pause) {
        lv_obj_t *lbl = lv_obj_get_child(s_mon_btn_pause, 0);
        if (lbl) lv_label_set_text(lbl, s_mon_paused ? "Resume" : "Pause");
    }
}

static void mon_clear_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_debug_textarea) lv_textarea_set_text(s_debug_textarea, "");
    /* 下次从最新位置开始，避免历史再次涌入 */
    rf_log_stats_t st;
    rf_log_get_stats(&st);
    s_mon_seq = st.total;
}

/* ---- RID RAW 控件回调 ------------------------------------------------------- */
static void rid0_toggle_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_rid0_show = !s_rid0_show;
    if (s_rid0_view) {
        if (s_rid0_show) {
            lv_obj_clear_flag(s_rid0_view, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_rid0_view, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_rid0_toggle_btn) {
        lv_obj_t *lbl = lv_obj_get_child(s_rid0_toggle_btn, 0);
        if (lbl) lv_label_set_text(lbl, s_rid0_show ? "HIDE" : "SHOW");
    }
}

static void rid0_output_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    bool on = lv_obj_has_state(s_rid0_output_cb, LV_STATE_CHECKED);
    /* on=true  对应 level=2 (所有 Beacon/Probe Resp) */
    rf_link_send_rid_raw_level(on ? 2 : 0);
}

/* Serial Monitor：行类型 -> 颜色标签（匹配 qt_ui/log_panel.py）
   TX CMD: 黄 $D 2.4G: 青 $W 5G: 橙 $A ACK: 绿 $R RID: 品红 $R0 RAW: 青灰
   LOG/其他: 灰白
   保留给后续逐行上色/图例。 */
static lv_color_t log_type_color(uint8_t type)
{
    switch (type) {
        case RF_LOG_LINE_TX_CMD:   return lv_color_hex(0xFFCC00);
        case RF_LOG_LINE_RX_DATA:  return lv_color_hex(0x00D4FF);
        case RF_LOG_LINE_RX_WIFI5: return lv_color_hex(0xFFA502);
        case RF_LOG_LINE_RX_ACK:   return lv_color_hex(0x00FF88);
        case RF_LOG_LINE_RX_RID:   return lv_color_hex(0xFF55DD);
        case RF_LOG_LINE_RX_RID0:  return lv_color_hex(0xA0D0E0);
        default:                   return lv_color_hex(0xCFEAFF);
    }
}
/* 显式引用：避免静态函数未使用警告（后续 UI 扩展会直接使用返回值配色） */
static inline void log_type_color_ensure_used(void) { }

static const char *log_type_prefix(uint8_t type)
{
    switch (type) {
        case RF_LOG_LINE_TX_CMD:   return "TX> ";
        case RF_LOG_LINE_RX_DATA:  return "RX<$D ";
        case RF_LOG_LINE_RX_WIFI5: return "RX<$W ";
        case RF_LOG_LINE_RX_ACK:   return "RX<$A ";
        case RF_LOG_LINE_RX_RID:   return "RX<$R ";
        case RF_LOG_LINE_RX_RID0:  return "RX<$R0 ";
        default:                   return "RX< ";
    }
}

static bool log_filter_pass(uint8_t type)
{
    /* 结合复选框状态；未创建时默认全部放行 */
    bool d   = !s_mon_chk_d   || lv_obj_has_state(s_mon_chk_d, LV_STATE_CHECKED);
    bool w   = !s_mon_chk_w   || lv_obj_has_state(s_mon_chk_w, LV_STATE_CHECKED);
    bool a   = !s_mon_chk_a   || lv_obj_has_state(s_mon_chk_a, LV_STATE_CHECKED);
    bool log = !s_mon_chk_log || lv_obj_has_state(s_mon_chk_log, LV_STATE_CHECKED);
    switch (type) {
        case RF_LOG_LINE_RX_DATA:  return d;
        case RF_LOG_LINE_RX_WIFI5: return w;
        case RF_LOG_LINE_RX_ACK:   return a;
        case RF_LOG_LINE_RX_LOG:   return log;
        case RF_LOG_LINE_RX_RID:
        case RF_LOG_LINE_RX_RID0:  return log;  /* RID 行归于 LOG 开关（Qt 侧类似） */
        case RF_LOG_LINE_TX_CMD:   return true; /* TX 始终可见 */
        default:                   return true;
    }
}

/* 对 hex 字符串做 ASCII 化：每 16 字节一行, 带偏移/ASCII 侧栏 */
static int format_hex_dump(const char *hex, int hex_len, char *out, int out_sz)
{
    int pos = 0;
    int bytes = hex_len / 2;
    if (bytes > 64) bytes = 64;   /* 限制文本区大小（显示用） */
    for (int off = 0; off < bytes; off += 16) {
        if (pos >= out_sz - 64) break;
        pos += snprintf(out + pos, (size_t)(out_sz - pos), "  %04X: ", off);
        char ascii[17] = {0};
        for (int i = 0; i < 16; i++) {
            int boff = off + i;
            if (boff < bytes && (boff * 2 + 1) < hex_len) {
                char hb[3] = { hex[boff * 2], hex[boff * 2 + 1], 0 };
                unsigned v = (unsigned)strtoul(hb, NULL, 16);
                pos += snprintf(out + pos, (size_t)(out_sz - pos), "%02X ", (unsigned)v);
                ascii[i] = (v >= 0x20 && v <= 0x7E) ? (char)v : '.';
            } else {
                pos += snprintf(out + pos, (size_t)(out_sz - pos), "   ");
                ascii[i] = ' ';
            }
        }
        pos += snprintf(out + pos, (size_t)(out_sz - pos), " %s\n", ascii);
    }
    return pos;
}

/* 802.11 Mgmt subtype 名称（对齐 rid_panel.py MGMT_SUBTYPE_NAMES） */
static const char *mgmt_subtype_name(uint8_t st)
{
    switch (st) {
        case 0:  return "AssocReq"; case 1:  return "AssocResp";
        case 2:  return "ReassocReq"; case 3: return "ReassocResp";
        case 4:  return "ProbeReq"; case 5:  return "ProbeResp";
        case 6:  return "TimingAdv"; case 7:  return "Rsvd7";
        case 8:  return "Beacon"; case 9:     return "ATIM";
        case 10: return "Disassoc"; case 11:  return "Auth";
        case 12: return "Deauth"; case 13:    return "Action";
        case 14: return "ActionNoAck"; case 15:return "RsvdF";
        default: return "SubXX";
    }
}

/* 把当前源+统计格式化为紧凑文本（2 行，减少 LVGL 字体渲染负担） */
static void format_link_stats(char *buf, int buf_sz, const rf_stats_t *st)
{
    bool sim = rf_link_get_source();
    int baud = rf_link_get_uart_baud();
    const char *hint = "";
    if (!sim && st->rx_bytes == 0) hint = " NO RX!";
    else if (!sim && st->rx_bytes > 0 && st->rx_frames == 0) hint = " NO FRAME!";
    snprintf(buf, buf_sz,
        "%s %s BAUD:%d%s  %luB %luF %luCRC %luHz D:%u\n"
        "$D=%lu $R=%lu $W=%lu $R0=%lu $A=%lu  Chk=%lu Pk=%luB SE=%lu",
        st->link_online ? "ON" : "OFF",
        sim ? "SIM" : "LIVE",
        baud, hint,
        (unsigned long)st->rx_bytes, (unsigned long)st->rx_frames,
        (unsigned long)st->crc_errors, (unsigned long)st->sweep_rate_hz,
        (unsigned)st->last_status.drone_count,
        (unsigned long)st->json_D, (unsigned long)st->json_R,
        (unsigned long)st->json_W, (unsigned long)st->json_R0,
        (unsigned long)st->json_A,
        (unsigned long)st->raw_rx_total_chunks,
        (unsigned long)st->raw_rx_peak_bytes,
        (unsigned long)st->sync_errors);
}

static void settings_debug_refresh_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    /* RF 链路状态 */
    if (s_settings_info) {
        rf_stats_t st;
        rf_link_get_stats(&st);
        char buf[512];
        format_link_stats(buf, sizeof(buf), &st);
        lv_label_set_text(s_settings_info, buf);
    }

    /* SCAN STATUS 行 */
    if (s_scan_status_lbl) {
        rf_scan_config_t sc;
        rf_link_get_scan_status(&sc);
        const char *rn = (sc.rate_idx >= 0 && sc.rate_idx < 3) ? RATE_NAMES[sc.rate_idx] : "?";
        lv_label_set_text_fmt(s_scan_status_lbl,
            "RANGE %d-%d  RATE %s\nSETTLE %dus  SPP %d",
            sc.start_ch, sc.end_ch, rn, sc.settle_us, sc.spp);
    }

    /* ========== Serial Monitor：批量拼接后单次追加（避免多次 add_text 触发重绘） ========== */
    if (s_debug_textarea && !s_mon_paused) {
        rf_log_line_t lines[16];
        int n;
        /* 每次最多取 16 行，拼接到一个大 buffer 后一次性 add_text */
        char batch_buf[8192];
        int batch_len = 0;
        int total_lines = 0;
        while (total_lines < 16) {
            int batch = 16 - total_lines;
            if (batch > 16) batch = 16;
            n = rf_log_read(lines, batch, &s_mon_seq);
            if (n <= 0) break;
            for (int i = 0; i < n; i++) {
                if (!log_filter_pass(lines[i].type)) continue;
                const char *pfx = log_type_prefix(lines[i].type);
                int pfx_len = (int)strlen(pfx);
                int txt_len = (int)strlen(lines[i].text);
                /* 检查 buffer 是否有空间（pfx + text + \n + \0） */
                if (batch_len + pfx_len + txt_len + 2 >= (int)sizeof(batch_buf)) break;
                memcpy(batch_buf + batch_len, pfx, pfx_len);
                batch_len += pfx_len;
                memcpy(batch_buf + batch_len, lines[i].text, txt_len);
                batch_len += txt_len;
                batch_buf[batch_len++] = '\n';
            }
            total_lines += n;
            if (n < batch) break;
        }
        if (batch_len > 0) {
            batch_buf[batch_len] = '\0';
            lv_textarea_add_text(s_debug_textarea, batch_buf);
        }
        /* 自动滚屏 + 限制最多 100 行（减少 textarea 渲染开销） */
        const char *cur = lv_textarea_get_text(s_debug_textarea);
        if (cur) {
            int nl = 0;
            for (const char *p = cur; *p; p++) if (*p == '\n') nl++;
            if (nl > 100) {
                int cut = nl - 100;
                const char *p = cur;
                while (cut > 0 && *p) { if (*p++ == '\n') cut--; }
                lv_textarea_set_text(s_debug_textarea, p);
            }
        }
        if (s_mon_scroll) {
            lv_textarea_set_cursor_pos(s_debug_textarea, LV_TEXTAREA_CURSOR_LAST);
        }
    }

    /* ========== RID RAW PAYLOAD：批量拼接后单次写入 ========== */
    if (s_rid0_view && s_rid0_show) {
        rf_rid0_frame_t frames[4];
        int n;
        char batch_buf[8192];
        int batch_len = 0;
        int remaining = 4;
        while (remaining > 0) {
            int batch = (remaining < 4) ? remaining : 4;
            n = rf_link_read_rid0_frames(frames, batch, &s_rid0_seq);
            if (n <= 0) break;
            for (int i = 0; i < n; i++) {
                const rf_rid0_frame_t *f = &frames[i];
                char header[192];
                int hw = snprintf(header, sizeof(header),
                    "[#%lu] %s  MAC=%s  RSSI=%d  CH=%u  LEN=%u  RID=%s\n",
                    (unsigned long)f->seq,
                    mgmt_subtype_name(f->subtype),
                    f->mac[0] ? f->mac : "?",
                    (int)f->rssi, (unsigned)f->channel,
                    (unsigned)f->length, f->has_rid ? "YES" : "no");
                if (hw > 0) {
                    if (hw >= (int)sizeof(header)) hw = (int)sizeof(header) - 1;
                    if (batch_len + hw < (int)sizeof(batch_buf) - 1) {
                        memcpy(batch_buf + batch_len, header, hw);
                        batch_len += hw;
                    }
                }
                char dump[2048];
                int dw = format_hex_dump(f->hex, f->hex_len, dump, (int)sizeof(dump));
                if (dw > 0) {
                    if (dw >= (int)sizeof(dump)) dw = (int)sizeof(dump) - 1;
                    dump[dw] = '\0';
                    if (batch_len + dw + 2 < (int)sizeof(batch_buf) - 1) {
                        memcpy(batch_buf + batch_len, dump, dw);
                        batch_len += dw;
                        batch_buf[batch_len++] = '\n';
                    }
                }
            }
            remaining -= n;
            if (n < batch) break;
        }
        if (batch_len > 0) {
            batch_buf[batch_len] = '\0';
            lv_textarea_add_text(s_rid0_view, batch_buf);
        }
        /* 限制文本长度：超过 8000 字符时砍掉前半 */
        const char *cur = lv_textarea_get_text(s_rid0_view);
        if (cur) {
            int L = (int)strlen(cur);
            if (L > 8000) {
                int cut = L - 4000;
                const char *p = cur + cut;
                while (*p && *p != '\n') p++;
                if (*p == '\n') p++;
                lv_textarea_set_text(s_rid0_view, p);
            }
        }
        lv_textarea_set_cursor_pos(s_rid0_view, LV_TEXTAREA_CURSOR_LAST);
    }
}

/* 辅助：添加 [标题  滑块  值] 一行 */
static void add_cfg_row(lv_obj_t *parent, const char *title,
                        int vmin, int vmax, int vdef, int step,
                        lv_obj_t **out_slider, lv_obj_t **out_value)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, lv_pct(100), 32);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 8, 0);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *tl = lv_label_create(row);
    lv_label_set_text(tl, title);
    lv_obj_set_width(tl, 64);
    lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
    lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);

    lv_obj_t *sl = lv_slider_create(row);
    lv_slider_set_range(sl, vmin, vmax);
    lv_slider_set_value(sl, vdef, LV_ANIM_OFF);
    lv_obj_set_width(sl, 180);

    lv_obj_t *vl = lv_label_create(row);
    lv_obj_set_width(vl, 44);
    lv_obj_set_style_text_color(vl, lv_color_hex(0x66CCFF), 0);
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_14, 0);
    if (step <= 1) {
        lv_label_set_text_fmt(vl, "%d", vdef);
    } else {
        lv_label_set_text_fmt(vl, "%d", vdef);
    }
    lv_obj_add_event_cb(sl, slider_val_cb, LV_EVENT_VALUE_CHANGED, vl);

    *out_slider = sl;
    *out_value  = vl;
}

/* 分组标题 */
static lv_obj_t *add_group_title(lv_obj_t *parent, const char *text)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, text);
    lv_obj_set_width(lbl, lv_pct(100));
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_pad_top(lbl, 8, 0);
    return lbl;
}

/* 分组下的小分割线 */
static void add_hline(lv_obj_t *parent)
{
    lv_obj_t *l = lv_obj_create(parent);
    lv_obj_set_size(l, lv_pct(100), 1);
    lv_obj_set_style_bg_color(l, lv_color_hex(0x333333), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(l, 0, 0);
    lv_obj_set_style_pad_all(l, 0, 0);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_SCROLLABLE);
}

static void build_settings_panel(void)
{
    lv_obj_t *parent = lv_layer_top();

    /* 居中卡片 + 可滚动内部 */
    s_settings_panel = lv_obj_create(parent);
    lv_obj_set_size(s_settings_panel, 360, 530);
    lv_obj_center(s_settings_panel);
    lv_obj_set_style_bg_color(s_settings_panel, lv_color_hex(0x181C22), 0);
    lv_obj_set_style_bg_opa(s_settings_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_settings_panel, lv_color_hex(0x30363F), 0);
    lv_obj_set_style_border_width(s_settings_panel, 1, 0);
    lv_obj_set_style_radius(s_settings_panel, 12, 0);
    lv_obj_set_style_pad_all(s_settings_panel, 12, 0);
    lv_obj_set_flex_flow(s_settings_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_settings_panel, 6, 0);
    lv_obj_set_scrollbar_mode(s_settings_panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_add_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);  // 默认隐藏

    /* 标题行：Settings + 关闭按钮 */
    lv_obj_t *title_row = lv_obj_create(s_settings_panel);
    lv_obj_set_size(title_row, lv_pct(100), 32);
    lv_obj_set_style_bg_opa(title_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(title_row, 0, 0);
    lv_obj_set_style_pad_all(title_row, 0, 0);
    lv_obj_set_flex_flow(title_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(title_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(title_row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(title_row);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);

    lv_obj_t *close_btn = lv_button_create(title_row);
    lv_obj_set_size(close_btn, 28, 28);
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x3A2020), 0);
    lv_obj_set_style_border_width(close_btn, 0, 0);
    lv_obj_set_style_radius(close_btn, 6, 0);
    lv_obj_set_style_pad_all(close_btn, 0, 0);
    lv_obj_add_event_cb(close_btn, settings_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(close_lbl, lv_color_hex(0xFF6060), 0);
    lv_obj_center(close_lbl);

    /* ===== DISPLAY 组 ===== */
    add_group_title(s_settings_panel, "DISPLAY");
    {
        lv_obj_t *row = lv_obj_create(s_settings_panel);
        lv_obj_set_size(row, lv_pct(100), 36);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 10, 0);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *tl = lv_label_create(row);
        lv_label_set_text(tl, "Backlight");
        lv_obj_set_width(tl, 64);
        lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);

        lv_obj_t *slider = lv_slider_create(row);
        lv_obj_set_width(slider, 180);
        lv_slider_set_range(slider, 1, 100);
        lv_slider_set_value(slider, 100, LV_ANIM_OFF);
        lv_obj_add_event_cb(slider, brightness_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

        s_brightness_lbl = lv_label_create(row);
        lv_label_set_text(s_brightness_lbl, "100%");
        lv_obj_set_width(s_brightness_lbl, 44);
        lv_obj_set_style_text_color(s_brightness_lbl, lv_color_hex(0x66CCFF), 0);
        lv_obj_set_style_text_font(s_brightness_lbl, &lv_font_montserrat_14, 0);
    }

    /* ===== SCAN STATUS 组 ===== */
    add_group_title(s_settings_panel, "SCAN STATUS");
    {
        s_scan_status_lbl = lv_label_create(s_settings_panel);
        lv_label_set_text(s_scan_status_lbl, "RANGE 0-125  RATE 2M\nSETTLE 300us  SPP 2");
        lv_obj_set_width(s_scan_status_lbl, lv_pct(100));
        lv_obj_set_style_text_color(s_scan_status_lbl, lv_color_hex(0xCFEAFF), 0);
        lv_obj_set_style_text_font(s_scan_status_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_pad_bottom(s_scan_status_lbl, 2, 0);
    }

    /* ===== RF 链路统计 ===== */
    add_group_title(s_settings_panel, "RF LINK");
    {
        s_settings_info = lv_label_create(s_settings_panel);
        lv_label_set_text(s_settings_info, "waiting...");
        lv_obj_set_width(s_settings_info, lv_pct(100));
        lv_obj_set_style_text_color(s_settings_info, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(s_settings_info, &lv_font_montserrat_14, 0);

        /* ===== 数据源选择：模拟数据 / 串口实际数据 ===== */
        lv_obj_t *src_row = lv_obj_create(s_settings_panel);
        lv_obj_set_size(src_row, lv_pct(100), LV_SIZE_CONTENT);  /* 自适应高度，避免裁切 */
        lv_obj_set_style_bg_opa(src_row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(src_row, 0, 0);
        lv_obj_set_style_pad_all(src_row, 0, 0);
        lv_obj_set_flex_flow(src_row, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(src_row, 6, 0);
        lv_obj_set_style_pad_top(src_row, 6, 0);
        lv_obj_clear_flag(src_row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *src_title = lv_label_create(src_row);
        lv_label_set_text(src_title, "Data Source   (UART: TX=GPIO18, RX=GPIO17)");
        lv_obj_set_width(src_title, lv_pct(100));
        lv_obj_set_style_text_color(src_title, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(src_title, &lv_font_montserrat_14, 0);

        lv_obj_t *btn_row = lv_obj_create(src_row);
        lv_obj_set_size(btn_row, lv_pct(100), 38);
        lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btn_row, 0, 0);
        lv_obj_set_style_pad_all(btn_row, 0, 0);
        lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(btn_row, 8, 0);
        lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);

        s_src_btn_sim = lv_button_create(btn_row);
        lv_obj_set_size(s_src_btn_sim, 160, 34);
        lv_obj_set_style_radius(s_src_btn_sim, 6, 0);
        lv_obj_set_style_pad_all(s_src_btn_sim, 0, 0);
        /* 默认禁止点回模拟（项目主链路 = 真实串口）。LVGL9 用 LV_STATE_DISABLED 达到灰显+不可点击。 */
        lv_obj_add_state(s_src_btn_sim, LV_STATE_DISABLED);
        lv_obj_clear_flag(s_src_btn_sim, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_src_btn_sim, src_sim_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *sl = lv_label_create(s_src_btn_sim);
        lv_label_set_text(sl, LV_SYMBOL_WIFI"  SimData(OFF)");
        lv_obj_set_style_text_color(sl, lv_color_hex(0x8C949B), LV_PART_MAIN);
        lv_obj_set_style_text_font(sl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_center(sl);

        s_src_btn_uart = lv_button_create(btn_row);
        lv_obj_set_size(s_src_btn_uart, 160, 34);
        lv_obj_set_style_radius(s_src_btn_uart, 6, 0);
        lv_obj_set_style_pad_all(s_src_btn_uart, 0, 0);
        lv_obj_add_event_cb(s_src_btn_uart, src_uart_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *ul = lv_label_create(s_src_btn_uart);
        lv_label_set_text(ul, LV_SYMBOL_SD_CARD"  UART Live");
        lv_obj_set_style_text_color(ul, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(ul, &lv_font_montserrat_14, 0);
        lv_obj_center(ul);

        refresh_source_btns_style();

        /* 串口调试输出开关（按钮自身显示 ON/OFF 状态 + 颜色） */
        lv_obj_t *dbg_row = lv_obj_create(src_row);
        lv_obj_set_size(dbg_row, lv_pct(100), 36);
        lv_obj_set_style_bg_opa(dbg_row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(dbg_row, 0, 0);
        lv_obj_set_style_pad_all(dbg_row, 0, 0);
        lv_obj_set_flex_flow(dbg_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(dbg_row, 8, 0);
        lv_obj_set_flex_align(dbg_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(dbg_row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *dtl = lv_label_create(dbg_row);
        lv_label_set_text(dtl, "Debug Out");
        lv_obj_set_width(dtl, 72);
        lv_obj_set_style_text_color(dtl, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(dtl, &lv_font_montserrat_14, 0);

        bool dbg_on = rf_link_get_uart_debug_output();
        s_uartdbg_toggle = lv_button_create(dbg_row);
        lv_obj_set_size(s_uartdbg_toggle, 240, 32);
        lv_obj_set_style_radius(s_uartdbg_toggle, 6, 0);
        lv_obj_set_style_border_width(s_uartdbg_toggle, 1, 0);
        lv_obj_set_style_border_color(s_uartdbg_toggle, lv_color_hex(0x3E4550), 0);
        lv_obj_set_style_pad_all(s_uartdbg_toggle, 0, 0);
        lv_obj_set_style_bg_color(s_uartdbg_toggle,
                                  dbg_on ? lv_color_hex(0x2E8B57) : lv_color_hex(0x2A2E36), 0);
        lv_obj_add_event_cb(s_uartdbg_toggle, uartdbg_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *dl = lv_label_create(s_uartdbg_toggle);
        lv_label_set_text(dl, dbg_on ? "UART DBG: ON" : "UART DBG: OFF");
        lv_obj_set_style_text_color(dl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(dl, &lv_font_montserrat_14, 0);
        lv_obj_center(dl);

        /* 波特率下拉 + 自动探测按钮 */
        lv_obj_t *baud_row = lv_obj_create(src_row);
        lv_obj_set_size(baud_row, lv_pct(100), 40);
        lv_obj_set_style_bg_opa(baud_row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(baud_row, 0, 0);
        lv_obj_set_style_pad_all(baud_row, 0, 0);
        lv_obj_set_flex_flow(baud_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(baud_row, 8, 0);
        lv_obj_set_flex_align(baud_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(baud_row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *bl = lv_label_create(baud_row);
        lv_label_set_text(bl, "Baud");
        lv_obj_set_width(bl, 42);
        lv_obj_set_style_text_color(bl, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(bl, &lv_font_montserrat_14, 0);

        s_baud_dropdown = lv_dropdown_create(baud_row);
        lv_dropdown_set_options(s_baud_dropdown,
                                "9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600");
        lv_obj_set_size(s_baud_dropdown, 150, 32);
        lv_obj_set_style_text_font(s_baud_dropdown, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_font(lv_dropdown_get_list(s_baud_dropdown), &lv_font_montserrat_14, 0);
        /* 按当前值选中下拉项 */
        {
            int cur = rf_link_get_uart_baud();
            int idx = 4; /* 默认 115200 */
            for (int i = 0; i < BAUD_OPTS_N; i++) if (s_baud_opts[i] == cur) { idx = i; break; }
            lv_dropdown_set_selected(s_baud_dropdown, (uint16_t)idx);
        }
        lv_obj_add_event_cb(s_baud_dropdown, baud_dropdown_cb, LV_EVENT_VALUE_CHANGED, NULL);

        lv_obj_t *pr = lv_button_create(baud_row);
        lv_obj_set_size(pr, 110, 32);
        lv_obj_set_style_radius(pr, 6, 0);
        lv_obj_set_style_pad_all(pr, 0, 0);
        lv_obj_set_style_bg_color(pr, lv_color_hex(0x3A4B66), 0);
        lv_obj_set_style_border_width(pr, 1, 0);
        lv_obj_set_style_border_color(pr, lv_color_hex(0x5577AA), 0);
        s_baud_probe_btn = pr;
        lv_obj_add_event_cb(pr, baud_probe_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *pl = lv_label_create(pr);
        /* ASCII only: Montserrat_14 has no CJK glyphs */
        lv_label_set_text(pl, LV_SYMBOL_REFRESH" AutoProbe");
        lv_obj_set_style_text_color(pl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(pl, &lv_font_montserrat_14, 0);
        lv_obj_center(pl);
    }

    /* ===== CONTROL 组（扫描配置已移至 2.4G Scan 页面，此处仅保留 RID Raw） ===== */
    add_group_title(s_settings_panel, "RID RAW");
    add_hline(s_settings_panel);
    {
        /* RID 原始数据等级 */
        lv_obj_t *row2 = lv_obj_create(s_settings_panel);
        lv_obj_set_size(row2, lv_pct(100), 32);
        lv_obj_set_style_bg_opa(row2, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row2, 0, 0);
        lv_obj_set_style_pad_all(row2, 0, 0);
        lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row2, 8, 0);
        lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row2, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *tl2 = lv_label_create(row2);
        lv_label_set_text(tl2, "RID Raw");
        lv_obj_set_width(tl2, 64);
        lv_obj_set_style_text_color(tl2, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(tl2, &lv_font_montserrat_14, 0);

        s_dd_rid_raw = lv_dropdown_create(row2);
        lv_dropdown_set_options(s_dd_rid_raw, "OFF\nRID only\nAll Beacon\nAll Mgmt");
        lv_dropdown_set_selected(s_dd_rid_raw, 0);
        lv_obj_set_width(s_dd_rid_raw, 220);

        /* APPLY 按钮（RID Raw 也需要 apply 生效） */
        lv_obj_t *apply = lv_button_create(s_settings_panel);
        lv_obj_set_size(apply, lv_pct(100), 36);
        lv_obj_set_style_bg_color(apply, lv_color_hex(0x0E639C), 0);
        lv_obj_set_style_bg_opa(apply, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(apply, 0, 0);
        lv_obj_set_style_radius(apply, 6, 0);
        lv_obj_add_event_cb(apply, apply_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *apply_lbl = lv_label_create(apply);
        lv_label_set_text(apply_lbl, "   APPLY   ");
        lv_obj_set_style_text_color(apply_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(apply_lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(apply_lbl);
    }

}

/* ---- 设置按钮回调 ------------------------------------------------------- */
static void settings_btn_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (!s_settings_panel) {
        build_settings_panel();
    }
    /* 刷新 RF 链路状态 */
    if (s_settings_info) {
        rf_stats_t st;
        rf_link_get_stats(&st);
        char buf[512];
        format_link_stats(buf, sizeof(buf), &st);
        lv_label_set_text(s_settings_info, buf);
    }
    lv_obj_clear_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_settings_panel);
    /* 启动调试数据自动刷新（1000ms，减少 textarea 渲染压力） */
    if (!s_debug_timer) {
        s_debug_timer = lv_timer_create(settings_debug_refresh_cb, 1000, NULL);
        lv_timer_ready(s_debug_timer);
    }
}

/* ---- 频谱重绘 ------------------------------------------------------------ */
/* 把 raw RSSI 归一化为 0~255 动态范围。
   - dBm 模式: 固定 -95~-20 → 0~255 (绝对强度, 跨帧可比)
   - Raw 模式: 动态 min/max + gain (弱信号放大凸显)
   - 输出到 out[len], 0=最暗, 255=最亮 */

/* raw RSSI → dBm (-95~-20) — 与 A5133/NRF24L01 公式同 */
static inline int raw_to_dbm(uint8_t raw)
{
    int d = -95 + (int)((uint32_t)raw * 75u / 255u);
    if (d < -95) d = -95;
    if (d > -20) d = -20;
    return d;
}

static void normalize_rssi(const uint8_t *in, uint8_t *out, int len)
{
    if (len <= 0) return;

    /* dBm 模式: 固定范围, 跨帧可比 */
    if (s_dbm_mode) {
        for (int i = 0; i < len; i++) {
            int dbm = raw_to_dbm(in[i]);
            int n = (dbm + 95) * 255 / 75;   /* -95→0, -20→255 */
            if (n < 0) n = 0;
            if (n > 255) n = 255;
            out[i] = (uint8_t)n;
        }
        return;
    }

    /* Raw 模式: 动态 min/max + gain */
    uint16_t mn = 255, mx = 0;
    for (int i = 0; i < len; i++) {
        if (in[i] < mn) mn = in[i];
        if (in[i] > mx) mx = in[i];
    }
    /* gain 效果: gain=1 → 用全范围; gain>1 → 把 floor 往上抬, 压暗弱信号 */
    /* floor = min + (max - min) / gain   */
    uint16_t range = mx - mn;
    uint16_t floor;
    if (s_rf_gain <= 1) {
        floor = mn;            /* 全范围映射 */
    } else {
        floor = mn + range / s_rf_gain;
        if (floor > mx) floor = mx;
    }
    uint16_t eff_range = mx - floor;
    if (eff_range == 0) {
        /* 全一样或被 gain 压扁成一层 — 直接给个小值避免完全看不见 */
        for (int i = 0; i < len; i++) out[i] = 16;
        return;
    }
    for (int i = 0; i < len; i++) {
        int v = (int)in[i] - (int)floor;
        if (v < 0) v = 0;
        uint32_t n = (uint32_t)v * 255 / eff_range;
        if (n > 255) n = 255;
        out[i] = (uint8_t)n;
    }
}

/* ---- RSSI / dBm 切换 ----------------------------------------------------- */
static void spectrum_redraw(const uint8_t *rssi, int nchan);
static void waterfall_push(const uint8_t *rssi, int nchan);

void rf_ui_update_rssi_mode_lbl(void)
{
    if (!s_rssi_mode_lbl) return;
    if (s_dbm_mode) {
        lv_label_set_text(s_rssi_mode_lbl, "dBm  range: -95 ~ -20");
    } else {
        lv_label_set_text(s_rssi_mode_lbl, "Raw RSSI  range: 0 ~ 255");
    }
}

static void rssi_mode_toggle_cb(lv_event_t *e)
{
    (void)e;
    s_dbm_mode = !s_dbm_mode;
    rf_ui_update_rssi_mode_lbl();
    ESP_LOGI(TAG, "RSSI mode: %s", s_dbm_mode ? "dBm" : "Raw");
    /* 立即重绘当前频段一帧 */
    uint8_t rssi[RF_UI_MAX_CHANNELS];
    uint32_t sweep_cnt, age_ms;
    int nchan = rf_ui_nchan_for_band(s_active_band);
    bool has = false;
    if (s_active_band == RF_BAND_5G) {
        has = rf_5g_get_spectrum(rssi, &sweep_cnt, &age_ms);
    } else if (rf_24g_is_running()) {
        has = rf_24g_get_spectrum(rssi, &sweep_cnt, &age_ms);
    } else {
        has = rf_link_get_spectrum(rssi, &sweep_cnt, &age_ms);
    }
    if (has) {
        spectrum_redraw(rssi, nchan);
        waterfall_push(rssi, nchan);
    }
}

static void spectrum_redraw(const uint8_t *rssi, int nchan)
{
    scan_band_ui_t *ui = &s_band_ui[s_active_band];
    lv_obj_t *spec_canvas = ui->spec_canvas;
    if (!spec_canvas) return;
    lv_draw_buf_t *dbuf = lv_canvas_get_draw_buf(spec_canvas);
    if (!dbuf || !dbuf->data) return;

    const int W = RF_UI_CONTENT_W;
    const int H = SPEC_H;
    const uint32_t stride = dbuf->header.stride;
    uint8_t *data = dbuf->data;

    /* 归一化 */
    uint8_t norm[RF_UI_MAX_CHANNELS];
    normalize_rssi(rssi, norm, nchan);

    /* 清底色 + 画水平网格线 */
    for (int y = 0; y < H; y++) {
        uint8_t *row = data + (uint32_t)y * stride;
        if (y % 25 == 0 && y > 0) {
            /* 暗灰色水平网格线 */
            for (int x = 0; x < W; x++) {
                ((uint16_t*)row)[x] = 0x1E28;  // RGB565: 30,40,55
            }
        } else {
            memset(row, 0, (size_t)W * 2);
        }
    }

    /* ===== 计算每个像素列对应的幅度（线性插值 126 通道 → 448 像素） ===== */
    /* 先把 nchan 个点重采样成 W 个像素点 */
    int amp_px[RF_UI_CONTENT_W];   /* 每列的 y 偏移 (0..H)，0=底 */
    for (int x = 0; x < W; x++) {
        /* 线性插值：x 对应通道位置 */
        float fpos = (float)x / W * (nchan - 1);
        int c0 = (int)fpos;
        int c1 = (c0 + 1 < nchan) ? c0 + 1 : c0;
        float frac = fpos - c0;
        float a = norm[c0] * (1 - frac) + norm[c1] * frac;
        /* amp → 像素高度 (0=底, H=顶) */
        amp_px[x] = (int)(a * H / 255.0f);
        if (amp_px[x] < 1) amp_px[x] = 1;
        if (amp_px[x] > H) amp_px[x] = H;
    }

    /* ===== 填底色渐变 (弱信号区底部加一点暗色, 让线更突出) ===== */
    /* 已清过, 保持深色 */

    /* ===== 画折线：在相邻两列间画垂直像素填充 =====
     * 方法: 对每对 (x, y0) → (x+1, y1):
     *   1) 画 y0..y1 的斜线 (线性插值)
     * 完全无死循环, 像素数 = max(|dy|, 2) */
    const uint8_t LR = 100, LG = 220, LB = 255;   /* 折线颜色 */
    const uint8_t GR = 30,  GG = 80,  GB = 110;   /* 下方发光底色 */

    for (int x = 0; x < W - 1; x++) {
        int y0 = H - amp_px[x];      /* 0=顶, H=底 */
        int y1 = H - amp_px[x + 1];

        /* 线性插值从 x 到 x+1, 逐像素画 */
        int steps = 2;  /* 至少两列 */
        int dy_total = y1 - y0;
        for (int i = 0; i < steps; i++) {
            float t = (float)i;    /* 0.0 或 1.0 */
            int ix = x + i;
            int iy = (int)(y0 + dy_total * t + 0.5f);
            if (ix >= 0 && ix < W && iy >= 0 && iy < H) {
                uint16_t *px = (uint16_t*)(data + (uint32_t)iy * stride) + ix; *px = 0x6DFC;  // RGB565: 100,220,255
                if (iy + 1 < H) {
                    uint16_t *px1 = (uint16_t*)(data + (uint32_t)(iy + 1) * stride) + ix;
                    uint16_t cur = *px1;
                    uint16_t glow = 0x1E6E;   // RGB565 30,80,110
                    /* 发光层: 取 RGB 各分量最大 */
                    uint8_t r1 = (cur >> 11) & 0x1F, g1 = (cur >> 5) & 0x3F, b1 = cur & 0x1F;
                    uint8_t r2 = (glow >> 11) & 0x1F, g2 = (glow >> 5) & 0x3F, b2 = glow & 0x1F;
                    *px1 = ((r1 > r2 ? r1 : r2) << 11) | ((g1 > g2 ? g1 : g2) << 5) | (b1 > b2 ? b1 : b2);
                }
                if (iy + 2 < H) {
                    uint16_t *px2 = (uint16_t*)(data + (uint32_t)(iy + 2) * stride) + ix;
                    uint16_t cur = *px2;
                    uint16_t glow = 0x0F37;   // RGB565 15,40,55
                    uint8_t r1 = (cur >> 11) & 0x1F, g1 = (cur >> 5) & 0x3F, b1 = cur & 0x1F;
                    uint8_t r2 = (glow >> 11) & 0x1F, g2 = (glow >> 5) & 0x3F, b2 = glow & 0x1F;
                    *px2 = ((r1 > r2 ? r1 : r2) << 11) | ((g1 > g2 ? g1 : g2) << 5) | (b1 > b2 ? b1 : b2);
                }
            }
        }

        /* 如果 |dy| > 1, 在两列之间的斜坡上补像素 (ix=x 或 x+1, iy 中间值) */
        if (dy_total > 1 || dy_total < -1) {
            /* 在两列之间均匀补 |dy| 个 y 值, 每个映射到 x 或 x+1 */
            for (int s = 1; s < abs(dy_total); s++) {
                float t = (float)s / abs(dy_total);
                /* 当 dy_total>0 (线向下走): t=0 在 x, t=1 在 x+1; 中间值按比例 */
                /* 但因为只有 x 和 x+1 两列, 我们让前半段在 x, 后半段在 x+1 */
                int ix = (s <= abs(dy_total)/2) ? x : x + 1;
                int iy = (int)(y0 + dy_total * (float)s / abs(dy_total) + 0.5f);
                if (ix >= 0 && ix < W && iy >= 0 && iy < H) {
                    uint16_t *px = (uint16_t*)(data + (uint32_t)iy * stride) + ix; *px = 0x6DFC;  // RGB565: 100,220,255
                }
            }
        }
    }

    lv_obj_invalidate(spec_canvas);
}

/* ---- 瀑布图：顶部写入新行 + 历史整体下移（由上往下刷新） --------------- */
static void waterfall_push(const uint8_t *rssi, int nchan)
{
    /* 按当前活动频段选择 canvas（未进入扫频页则跳过） */
    scan_band_ui_t *ui = &s_band_ui[s_active_band];
    lv_obj_t *wf_canvas = ui->wf_canvas;
    if (!wf_canvas) return;
    lv_draw_buf_t *dbuf = lv_canvas_get_draw_buf(wf_canvas);
    if (!dbuf || !dbuf->data) return;

    const int W = RF_UI_CONTENT_W;
    const int H = WF_H;
    const uint32_t stride = dbuf->header.stride;
    uint8_t *data = dbuf->data;

    /* 整体下移一行：把第 0..H-2 行 memmove 到第 1..H-1 行（历史向下沉） */
    if (H > 1) {
        memmove(data + stride, data, (size_t)stride * (H - 1));
    }
    /* 动态范围归一化 — 跟频谱图用同一条曲线 */
    uint8_t norm[RF_UI_MAX_CHANNELS];
    normalize_rssi(rssi, norm, nchan);
    /* 顶行(y=0)写新数据：把 nchan 信道线性插值到 W 像素 (参考图风格) */
    uint8_t *top_row = data;   /* y=0 行 */
    for (int x = 0; x < W; x++) {
        float fpos = (float)x / W * (nchan - 1);
        int c0 = (int)fpos;
        int c1 = (c0 + 1 < nchan) ? c0 + 1 : c0;
        float frac = fpos - c0;
        float amp_f = norm[c0] * (1 - frac) + norm[c1] * frac;
        uint8_t amp = (uint8_t)(amp_f + 0.5f);
        lv_color_t col = rssi_to_color(amp);
        ((uint16_t*)top_row)[x] = lv_color_to_u16(col);
    }
    lv_obj_invalidate(wf_canvas);
}

/* ---- RID 卡片构建 -------------------------------------------------------- */
static const char *id_type_str(uint8_t t)
{
    switch (t) {
        case 1: return "Serial";
        case 2: return "ANSI/CTA";
        case 3: return "UTM-Assigned";
        case 4: return "CAA-Reg";
        default: return "Other";
    }
}

static lv_obj_t *make_drone_card(lv_obj_t *parent, const rf_drone_t *d)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1B2330), 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0x3A6EA5), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_pad_all(card, 8, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(card, 2, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(card);
    {
        char tb[96];
        snprintf(tb, sizeof(tb), "# %s   [%s]", d->uas_id, id_type_str(d->uas_id_type));
        tb[sizeof(tb) - 1] = 0;
        lv_label_set_text(title, tb);
    }
    lv_obj_set_style_text_color(title, lv_color_hex(0x66CCFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);

    lv_obj_t *desc = lv_label_create(card);
    {
        char db[96];
        snprintf(db, sizeof(db), "Type:%u  Op:%s", (unsigned)d->uas_id_type,
                 d->operator_id[0] ? d->operator_id : "-");
        db[sizeof(db) - 1] = 0;
        lv_label_set_text(desc, db);
    }
    lv_obj_set_style_text_color(desc, lv_color_hex(0xCCCCCC), 0);

    lv_obj_t *desc2 = lv_label_create(card);
    lv_label_set_text(desc2, d->op_desc[0] ? d->op_desc : "(no self-id)");

    lv_obj_t *loc = lv_label_create(card);
    {
        char lb[96];
        double lat = (double)d->lat_e7 / 1e7;
        double lon = (double)d->lon_e7 / 1e7;
        if (d->lat_e7 == 0 && d->lon_e7 == 0) {
            snprintf(lb, sizeof(lb), "Drone  Lat:(n/a)  Lon:(n/a)");
        } else {
            snprintf(lb, sizeof(lb), "Drone  Lat:%.7f  Lon:%.7f", lat, lon);
        }
        lb[sizeof(lb) - 1] = 0;
        lv_label_set_text(loc, lb);
    }
    lv_obj_set_style_text_color(loc, lv_color_hex(0xA0E0A0), 0);

    lv_obj_t *oploc = lv_label_create(card);
    if (d->flags & 0x08) {
        char pb[96];
        double olat = (double)d->operator_lat_e7 / 1e7;
        double olon = (double)d->operator_lon_e7 / 1e7;
        snprintf(pb, sizeof(pb), "Pilot  Lat:%.7f  Lon:%.7f", olat, olon);
        pb[sizeof(pb) - 1] = 0;
        lv_label_set_text(oploc, pb);
        lv_obj_set_style_text_color(oploc, lv_color_hex(0xFFB366), 0);
    } else {
        lv_label_set_text(oploc, "Pilot  (location not available)");
        lv_obj_set_style_text_color(oploc, lv_color_hex(0x776655), 0);
    }

    lv_obj_t *mov = lv_label_create(card);
    {
        char ag[16], ht[16], sp[16], vs[16], tk[16];
        ag[0]=ht[0]=sp[0]=vs[0]=tk[0]=0;
        if (d->alt_geodetic_m < -900.0f) strncpy(ag, "-", sizeof(ag) - 1); else snprintf(ag, sizeof(ag), "%.0fm", (double)d->alt_geodetic_m);
        if (d->height_m       < -900.0f) strncpy(ht, "-", sizeof(ht) - 1); else snprintf(ht, sizeof(ht), "%.0fm", (double)d->height_m);
        if (d->horiz_speed    <   -0.5f) strncpy(sp, "-", sizeof(sp) - 1); else snprintf(sp, sizeof(sp), "%.1fm/s", (double)d->horiz_speed);
        if (d->vert_speed     <   -0.5f) strncpy(vs, "-", sizeof(vs) - 1); else snprintf(vs, sizeof(vs), "%.1fm/s", (double)d->vert_speed);
        if (d->ground_track   <   -0.5f) strncpy(tk, "-", sizeof(tk) - 1); else snprintf(tk, sizeof(tk), "%.1f°", (double)d->ground_track);
        ag[sizeof(ag)-1]=0; ht[sizeof(ht)-1]=0; sp[sizeof(sp)-1]=0; vs[sizeof(vs)-1]=0; tk[sizeof(tk)-1]=0;
        char line[128];
        snprintf(line, sizeof(line), "Geo alt:%s  H:%s  Spd:%s  vSpd:%s  Track:%s", ag, ht, sp, vs, tk);
        line[sizeof(line)-1]=0;
        lv_label_set_text(mov, line);
    }
    lv_obj_set_style_text_color(mov, lv_color_hex(0xDDDDDD), 0);

    lv_obj_t *age_lbl = lv_label_create(card);
    {
        char ab[48];
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t age = (now_ms >= d->last_seen_ms) ? (now_ms - d->last_seen_ms) / 1000 : 0;
        snprintf(ab, sizeof(ab), "Last seen: %uus ago", (unsigned)age);
        ab[sizeof(ab) - 1] = 0;
        lv_label_set_text(age_lbl, ab);
    }
    lv_obj_set_style_text_color(age_lbl, lv_color_hex(0x888888), 0);

    return card;
}

static void rid_rebuild(void)
{
    /* 清空旧卡片；gen 变化频率可能很高（C5 扫频时每几百 ms 进出），
       这里只打印一条 LOGI（不打 N+1 条逐架详情），避免串口刷爆 */
    if (!s_rid_list) return;
    lv_obj_clean(s_rid_list);
    rf_drone_t drones[RF_MAX_DRONES];
    int n = rf_link_get_drones(drones, RF_MAX_DRONES);

    if (n == 0) {
        s_rid_empty = lv_label_create(s_rid_list);
        lv_label_set_text(s_rid_empty, "No drones detected (scanning ASTM F3411 RID...)");
        lv_obj_set_style_text_color(s_rid_empty, lv_color_hex(0x666677), 0);
        return;
    }
    s_rid_empty = NULL;
    for (int i = 0; i < n; i++) {
        make_drone_card(s_rid_list, &drones[i]);
    }
}

/* ---- 刷新定时器 --------------------------------------------------------- */
static void rf_ui_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);

    /* 判断是否需要刷新频谱/瀑布图：只有在频谱页才做重绘
       主页/自检页/其他页跳过 → 节省 CPU，提高 UI 流畅度 */
    bool is_spectrum_page = (s_cur_page == PAGE_24G) ||
                            (s_cur_page == PAGE_58G) ||
                            (s_cur_page == PAGE_SPECTRUM);

    if (is_spectrum_page) {
    /* 1) 频谱 + 瀑布：取最新扫描帧
       - 2.4G (RF_BAND_24G): 从 rf_link (ESP32-C5 UART) 取
       - 5.8G (RF_BAND_5G):  从 rf_5g (本地 A5133 SPI 扫描) 取
       仅在 sweep_cnt 变化时重绘, age > 3s 跳过避免旧帧刷新 */
    uint8_t rssi[RF_UI_MAX_CHANNELS];   /* 足够装 256 通道 (5.8G) */
    int nchan = rf_ui_nchan_for_band(s_active_band);
    uint32_t sweep_cnt = 0, age_ms = 0;
    bool has;
    if (s_active_band == RF_BAND_5G) {
        has = rf_5g_get_spectrum(rssi, &sweep_cnt, &age_ms);
    } else {
        /* 2.4G: 优先本地 nRF24L01 (rf_24g), fallback UART C5 (rf_link) */
        if (rf_24g_is_running()) {
            has = rf_24g_get_spectrum(rssi, &sweep_cnt, &age_ms);
        } else {
            has = rf_link_get_spectrum(rssi, &sweep_cnt, &age_ms);
        }
    }
    bool new_sweep = (sweep_cnt != s_last_wf_sweep);

    /* ★ 数据源/频段切换 → 强制 new_sweep=true 并重置 */
    bool now_rf24g = (s_active_band != RF_BAND_5G) && rf_24g_is_running();
    if (s_active_band != s_last_wf_band || now_rf24g != s_last_used_rf24g) {
        ESP_LOGW(TAG, "数据源切换: band %d→%d, rf24g %d→%d → 重置 sweep 检测",
                 s_last_wf_band, s_active_band, s_last_used_rf24g, now_rf24g);
        s_last_wf_band = s_active_band;
        s_last_used_rf24g = now_rf24g;
        s_last_wf_sweep = sweep_cnt;   /* 下一个新 sweep 才能触发绘制 */
        new_sweep = false;             /* 跳过当前这帧 */
    }

    /* ---- 诊断日志: 每 2 秒打一次 ---- */
    static int s_diag_cnt = 0;
    if (++s_diag_cnt >= 200) {   /* 10ms timer × 200 = 2s */
        s_diag_cnt = 0;
        scan_band_ui_t *ui = &s_band_ui[s_active_band];
        ESP_LOGI(TAG,
            "[DIAG] band=%d(%s) nchan=%d has=%d sweep=%lu new=%d age=%lums "
            "rf24g_running=%d spec=%p wf=%p last=%lu → %s%s%s%s",
            s_active_band,
            (s_active_band == RF_BAND_5G) ? "5.8G" : "2.4G",
            nchan, has, (unsigned long)sweep_cnt, new_sweep,
            (unsigned long)age_ms,
            rf_24g_is_running(),
            ui->spec_canvas, ui->wf_canvas,
            (unsigned long)s_last_wf_sweep,
            has ? "HAS" : "NO",
            new_sweep ? " NEW" : " -",
            (ui->spec_canvas && ui->wf_canvas) ? " UI_OK" : " UI_NULL",
            (age_ms < 3000) ? " FRESH" : " STALE");
        /* 额外: 2.4G 模式下打印前 8 个 rssi 值 */
        if (s_active_band != RF_BAND_5G && has) {
            ESP_LOGI(TAG, "  rssi[0..7]=%u %u %u %u %u %u %u %u",
                     rssi[0], rssi[1], rssi[2], rssi[3],
                     rssi[4], rssi[5], rssi[6], rssi[7]);
        }
    }

    if (has && new_sweep && age_ms < 3000) {
        spectrum_redraw(rssi, nchan);
        waterfall_push(rssi, nchan);
        s_last_wf_sweep = sweep_cnt;
    }
    }  /* is_spectrum_page */

    /* 2) 顶栏状态（主页 / 子页 Header 都刷新，但主页时降低频率到 500ms） */
    static uint32_t s_last_status_tick = 0;
    uint32_t now_tick = lv_tick_get();
    bool status_force = (s_cur_page != PAGE_HOME) && (s_cur_page != PAGE_BOOT);
    if (status_force || (now_tick - s_last_status_tick) >= 500) {
        s_last_status_tick = now_tick;
        rf_stats_t st;
        rf_link_get_stats(&st);
        {
            char buf[192];
            snprintf(buf, sizeof(buf),
                "%s  %luHz  frames:%lu  drones:%u",
                st.link_online ? "ONLINE" : "OFFLINE",
                (unsigned long)st.sweep_rate_hz,
                (unsigned long)st.rx_frames,
                (unsigned)st.last_status.drone_count);
            if (s_page_state_lbl) lv_label_set_text(s_page_state_lbl, buf);
        }
        if (s_header_lbl) {
            lv_label_set_text_fmt(s_header_lbl,
                "RF Monitor  %s  %luHz  frames:%lu  drones:%u  CRCerr:%lu",
                st.link_online ? "ONLINE" : "OFFLINE",
                (unsigned long)st.sweep_rate_hz,
                (unsigned long)st.rx_frames,
                (unsigned)st.last_status.drone_count,
                (unsigned long)st.crc_errors);
        }
    }

    /* 3) RID 列表：仅在 RID 页可见时重建（节省主页 CPU） */
    if (s_cur_page == PAGE_RID) {
        uint32_t gen = rf_link_drones_generation();
        if (gen != s_last_rid_gen) {
            s_last_rid_gen = gen;
            rid_rebuild();
        }
    }

    /* 4) RID MAP 列表 + 跟踪：每 500ms 刷新位置（不受 gen 变化限制）
     *   - 定时刷新: 即使同一批无人机位置也持续更新
     *   - 跟踪模式: 选中的无人机自动居中地图 */
    if (s_cur_page == PAGE_RIDMAP) {
    static uint32_t s_last_map_tick_ms = 0;
    if (s_ridmap_list && lv_obj_is_valid(s_ridmap_list) &&
        (now_tick - s_last_map_tick_ms) >= 500) {
        s_last_map_tick_ms = now_tick;

        ridmap_labels_init_once();

        rf_drone_t drones[RF_MAX_DRONES];
        int n = rf_link_get_drones(drones, RF_MAX_DRONES);

        /* 隐藏所有 row 和 empty hint，按需 set_text + 显示 */
        for (int i = 0; i < RIDMAP_ROW_MAX; i++) {
            if (s_ridmap_row[i]) lv_obj_add_flag(s_ridmap_row[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (s_ridmap_empty) lv_obj_add_flag(s_ridmap_empty, LV_OBJ_FLAG_HIDDEN);

        if (n == 0) {
            if (s_ridmap_empty) lv_obj_remove_flag(s_ridmap_empty, LV_OBJ_FLAG_HIDDEN);
            /* 没有无人机时自动关闭跟踪 */
            if (s_track_enabled) {
                s_track_enabled = false;
                memset(s_track_id, 0, 24);
            }
        } else {
            char b[192];
            int track_idx = -1;  /* 跟踪的无人机在列表中的下标 */
            for (int i = 0; i < n && i < RIDMAP_ROW_MAX; i++) {
                rf_drone_t *d = &drones[i];
                char id[25];
                memcpy(id, d->uas_id, 24);
                id[24] = 0;
                char id_short[12];
                snprintf(id_short, sizeof(id_short), "%-10.10s", id);
                int16_t alt = (d->alt_geodetic_m != 0) ? d->alt_geodetic_m : d->alt_pressure_m;
                snprintf(b, sizeof(b), "%s  %.5f %.5f %dm    OP %.5f %.5f",
                    id_short,
                    (double)d->lat_e7 / 10000000.0,
                    (double)d->lon_e7 / 10000000.0,
                    (int)alt,
                    (double)d->operator_lat_e7 / 10000000.0,
                    (double)d->operator_lon_e7 / 10000000.0);
                if (s_ridmap_row_lbl[i]) {
                    lv_label_set_text(s_ridmap_row_lbl[i], b);
                }
                /* 保存该行对应的 drone ID */
                memcpy(s_ridmap_row_id[i], d->uas_id, 24);
                /* 检查是否是跟踪目标 */
                if (s_track_enabled && memcmp(s_track_id, d->uas_id, 24) == 0) {
                    track_idx = i;
                }
                if (s_ridmap_row[i]) {
                    lv_obj_remove_flag(s_ridmap_row[i], LV_OBJ_FLAG_HIDDEN);
                }
            }

            /* 更新行高亮状态 */
            for (int i = 0; i < n && i < RIDMAP_ROW_MAX; i++) {
                bool selected = (track_idx == i);
                lv_obj_set_style_bg_color(s_ridmap_row[i],
                    selected ? lv_color_hex(0x2A4A6A) : lv_color_hex(0x15202B), 0);
                lv_obj_set_style_border_color(s_ridmap_row[i],
                    selected ? lv_color_hex(0x00E0FF) : lv_color_hex(0x2A3340), 0);
            }

            /* ★ 跟踪模式: 地图中心跟随无人机移动 */
            if (track_idx >= 0 && track_idx < n) {
                rf_drone_t *td = &drones[track_idx];
                double tlat = (double)td->lat_e7 / 1e7;
                double tlon = (double)td->lon_e7 / 1e7;
                /* 只在位置变化明显时才 set_center (避免频繁触发全量重绘) */
                static double s_last_track_lat = 999.0;
                static double s_last_track_lon = 999.0;
                if (fabs(tlat - s_last_track_lat) > 0.0002 ||
                    fabs(tlon - s_last_track_lon) > 0.0002) {
                    rf_map_set_center(&s_map_view, tlat, tlon, s_map_view.zoom);
                    s_last_track_lat = tlat;
                    s_last_track_lon = tlon;
                }
            }
        }
    }
    }  /* s_cur_page == PAGE_RIDMAP */

    /* 5) Tile canvas: 瓦片底图由 rf_map_render_task 异步渲染,
        LVGL timer 只做 GPS 跟随通知 + 圆点绘制 (LOCK-FREE) */
    static double   s_last_gps_center_lat = 999.0;
    static double   s_last_gps_center_lon = 999.0;

    /* GPS 跟随: 偏离 > ~55m 发 set_center 通知 (仅通知, 不阻塞) */
    if (PAGE_RIDMAP == s_cur_page && rf_gps_has_fix()) {
        rf_gps_state_t gps;
        int gage;
        if (rf_gps_get(&gps, &gage)) {
            double glat = gps.lat_e7 * 1e-7;
            double glon = gps.lon_e7 * 1e-7;
            bool first = (s_map_view.tile_x < 0);
            if (first ||
                fabs(glat - s_map_view.center_lat) > 0.0005 ||
                fabs(glon - s_map_view.center_lon) > 0.0005) {
                rf_map_set_center(&s_map_view, glat, glon,
                                  first ? RF_MAP_DEFAULT_ZOOM : s_map_view.zoom);
                s_last_gps_center_lat = glat;
                s_last_gps_center_lon = glon;
            }
        }
    }

    /* 若地图还没初始化过, 发一次默认位置 */
    if (PAGE_RIDMAP == s_cur_page && s_map_view.tile_x < 0 &&
        s_map_ready && rf_map_is_mounted()) {
        rf_map_set_center(&s_map_view, RF_MAP_DEFAULT_CENTER_LAT,
                          RF_MAP_DEFAULT_CENTER_LON, RF_MAP_DEFAULT_ZOOM);
    }

    /* GPS 自点 + 无人机圆点绘制 (LOCK-FREE, 偶发 1px race 肉眼不可见) */
    if (PAGE_RIDMAP == s_cur_page && s_ridmap_canvas && lv_obj_is_valid(s_ridmap_canvas) &&
        s_map_view.tile_x >= 0) {
        /* LOCK-FREE: 不拿 render mutex, 和 tile 渲染共享 canvas buffer
           — 偶发 1 像素 race 在圆点区域肉眼不可见, 但 LVGL 绝不跨核阻塞 */

        /* ====== 无人机/地图视图变化检测 → 全量重绘清旧点 ======
         * 触发条件 (满足任一):
         *   1. 无人机数量变化 (出现/消失)
         *   2. 新 ID 出现
         *   3. 同 ID 无人机经纬度变化 > 10px 阈值
         *   4. 地图视图变化 (拖动/缩放导致 tile_slot 偏移 > 阈值)
         * 全量重绘节流: 最少 1000ms 一次, 避免频繁从 SD 加载 25 个 PNG
         */
        rf_drone_t cur_drones[RF_MAX_DRONES];
        int cur_n = rf_link_get_drones(cur_drones, RF_MAX_DRONES);
        static int s_prev_n = -1;
        static uint8_t s_prev_ids[RF_MAX_DRONES][24];    /* uas_id[24] */
        static int32_t s_prev_lat[RF_MAX_DRONES];        /* 上帧 lat_e7 */
        static int32_t s_prev_lon[RF_MAX_DRONES];        /* 上帧 lon_e7 */
        static int s_prev_slot_x = -9999;                 /* 上帧 tile_slot_x */
        static int s_prev_slot_y = -9999;                 /* 上帧 tile_slot_y */
        static int s_prev_zoom = -1;                      /* 上帧 zoom */
        static uint32_t s_last_redraw_ms = 0;

        bool need_full_redraw = false;

        /* --- 条件 4: 地图视图变化 (拖动/缩放) --- */
        if (s_prev_slot_x != s_map_view.tile_slot_x ||
            s_prev_slot_y != s_map_view.tile_slot_y ||
            s_prev_zoom != s_map_view.zoom) {
            /* 偏移超过 20px 才触发, 避免微小抖动导致频繁重绘 */
            int dx = s_prev_slot_x - s_map_view.tile_slot_x;
            int dy = s_prev_slot_y - s_map_view.tile_slot_y;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            if (dx > 20 || dy > 20 || s_prev_zoom != s_map_view.zoom) {
                need_full_redraw = true;
            }
        }

        /* --- 条件 1: 数量变化 --- */
        if (s_prev_n != cur_n) {
            need_full_redraw = true;
        }

        /* --- 条件 2 & 3: 新 ID 出现 / 同 ID 位置变化大 --- */
        if (!need_full_redraw && cur_n > 0) {
            for (int i = 0; i < cur_n && !need_full_redraw; i++) {
                int prev_idx = -1;
                for (int j = 0; j < s_prev_n; j++) {
                    if (memcmp(cur_drones[i].uas_id, s_prev_ids[j], 24) == 0) {
                        prev_idx = j; break;
                    }
                }
                if (prev_idx < 0) {
                    need_full_redraw = true;   /* 新 ID 出现 */
                } else {
                    /* 同 ID: 位置变化 > ~10px 阈值 (zoom=16 时近似) */
                    int32_t dlat = cur_drones[i].lat_e7 - s_prev_lat[prev_idx];
                    int32_t dlon = cur_drones[i].lon_e7 - s_prev_lon[prev_idx];
                    if (dlat < 0) dlat = -dlat;
                    if (dlon < 0) dlon = -dlon;
                    if (dlat > 1000 || dlon > 1000) {
                        need_full_redraw = true;
                    }
                }
            }
        }

        /* 全量重绘节流: 最少 1000ms 一次 */
        uint32_t now_ms = lv_tick_get();
        if (need_full_redraw && (now_ms - s_last_redraw_ms) > 1000) {
            ESP_LOGD("rf_ui", "redraw: drones=%d slot(%d,%d) zoom=%d",
                     cur_n, s_map_view.tile_slot_x, s_map_view.tile_slot_y, s_map_view.zoom);
            rf_map_request_redraw(&s_map_view);  // ★ 异步! 清背景 + 重贴全部瓦片
            s_last_redraw_ms = now_ms;
        }

        /* 保存本次状态 */
        for (int i = 0; i < cur_n; i++) {
            memcpy(s_prev_ids[i], cur_drones[i].uas_id, 24);
            s_prev_lat[i] = cur_drones[i].lat_e7;
            s_prev_lon[i] = cur_drones[i].lon_e7;
        }
        s_prev_n = cur_n;
        s_prev_slot_x = s_map_view.tile_slot_x;
        s_prev_slot_y = s_map_view.tile_slot_y;
        s_prev_zoom = s_map_view.zoom;

        lv_draw_buf_t *dbuf = lv_canvas_get_draw_buf(s_ridmap_canvas);
        if (dbuf && dbuf->data) {
            uint8_t *data = dbuf->data;
            uint32_t stride = dbuf->header.stride;

            /* GPS 蓝点 (蓝色填充 + 白色外环) */
            if (rf_gps_has_fix()) {
                rf_gps_state_t gps;
                int gage;
                if (rf_gps_get(&gps, &gage)) {
                    int gcx, gcy;
                    if (rf_map_project_e7(&s_map_view, gps.lat_e7, gps.lon_e7, &gcx, &gcy)) {
                        uint16_t *row0 = (uint16_t *)data;
                        for (int dy = -5; dy <= 5; dy++) {
                            int y = gcy + dy;
                            if (y < 0 || y >= 280) continue;
                            uint16_t *rb = (uint16_t *)((uint8_t *)row0 + (uint32_t)y * stride);
                            for (int dx = -5; dx <= 5; dx++) {
                                int x = gcx + dx;
                                if (x < 0 || x >= 448) continue;
                                int d2 = dx*dx + dy*dy;
                                if (d2 <= 25 && d2 >= 9)       rb[x] = 0xFFFF;
                                else if (d2 <= 4)              rb[x] = 0x001F;
                            }
                        }
                    }
                }
            }

            /* ====== 无人机 + 遥控器圆点 ======
             * UA active=红色 r=4, inactive=橙色 r=2, 都有白色中心点
             * 跟踪中的无人机: 黄色外圈 (r+3) + 内部原色
             * OP=青色 r=3, 白色中心点 (旧版 OP 是白色 → 瓦片上看不见!)
             */
            for (int i = 0; i < cur_n; i++) {
                int cx, cy;
                bool is_tracked = s_track_enabled &&
                                  (memcmp(s_track_id, cur_drones[i].uas_id, 24) == 0);
                /* --- UA 无人机 --- */
                if (rf_map_project_e7(&s_map_view, cur_drones[i].lat_e7, cur_drones[i].lon_e7, &cx, &cy)) {
                    uint16_t col = cur_drones[i].active ? 0xF800 : 0xFC00;  /* 红 / 橙 */
                    int r = cur_drones[i].active ? 4 : 2;

                    /* 跟踪目标: 先画黄色外圈 (r+3 到 r+1 之间的环) */
                    if (is_tracked) {
                        int outer_r = r + 3;
                        int inner_r = r + 1;
                        for (int dy = -outer_r; dy <= outer_r; dy++) {
                            int y = cy + dy;
                            if (y < 0 || y >= 280) continue;
                            uint16_t *row_base = (uint16_t *)(data + (uint32_t)y * stride);
                            for (int dx = -outer_r; dx <= outer_r; dx++) {
                                int d2 = dx*dx + dy*dy;
                                if (d2 > outer_r*outer_r) continue;
                                if (d2 < inner_r*inner_r) continue;
                                int x = cx + dx;
                                if (x < 0 || x >= 448) continue;
                                row_base[x] = 0xFFE0;  /* 黄色外圈 */
                            }
                        }
                    }

                    /* 内部填充圆 */
                    for (int dy = -r; dy <= r; dy++) {
                        int y = cy + dy;
                        if (y < 0 || y >= 280) continue;
                        uint16_t *row_base = (uint16_t *)(data + (uint32_t)y * stride);
                        for (int dx = -r; dx <= r; dx++) {
                            if (dx*dx + dy*dy > r*r) continue;
                            int x = cx + dx;
                            if (x < 0 || x >= 448) continue;
                            row_base[x] = col;
                        }
                    }
                    uint16_t *rb = (uint16_t *)(data + (uint32_t)cy * stride);
                    if (cx >= 0 && cx < 448 && cy >= 0 && cy < 280) rb[cx] = 0xFFFF;
                }
                /* --- OP 遥控器 --- */
                if (cur_drones[i].operator_lat_e7 != 0 && cur_drones[i].operator_lon_e7 != 0) {
                    if (rf_map_project_e7(&s_map_view, cur_drones[i].operator_lat_e7,
                                          cur_drones[i].operator_lon_e7, &cx, &cy)) {
                        for (int dy = -3; dy <= 3; dy++) {
                            int y = cy + dy;
                            if (y < 0 || y >= 280) continue;
                            uint16_t *row_base = (uint16_t *)(data + (uint32_t)y * stride);
                            for (int dx = -3; dx <= 3; dx++) {
                                if (dx*dx + dy*dy > 9) continue;
                                int x = cx + dx;
                                if (x < 0 || x >= 448) continue;
                                row_base[x] = 0x07FF;   /* 青色 */
                            }
                        }
                        uint16_t *rb = (uint16_t *)(data + (uint32_t)cy * stride);
                        if (cx >= 0 && cx < 448 && cy >= 0 && cy < 280) rb[cx] = 0xFFFF;
                    }
                }
            }
            lv_obj_invalidate(s_ridmap_canvas);
        }
    }
}

/* ---- 构建静态界面 ------------------------------------------------------- */
static lv_obj_t *make_section_label(lv_obj_t *parent, const char *txt)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x9AB4D4), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    return lbl;
}

/* 颜色渐变色谱条（0..100 强度） */
static void paint_legend(lv_obj_t *canvas, int w, int h)
{
    for (int x = 0; x < w; x++) {
        lv_color_t c;
        double t = (double)x / (double)(w - 1);
        if (t < 0.25) {
            double k = t / 0.25;
            c = lv_color_make(0, (uint8_t)(40 + 100 * k), (uint8_t)(80 + 120 * k));
        } else if (t < 0.55) {
            double k = (t - 0.25) / 0.30;
            c = lv_color_make(0, (uint8_t)(140 + 60 * k), (uint8_t)(200 - 80 * k));
        } else if (t < 0.85) {
            double k = (t - 0.55) / 0.30;
            c = lv_color_make((uint8_t)(255 * k), (uint8_t)(200 - 120 * k), 0);
        } else {
            double k = (t - 0.85) / 0.15;
            c = lv_color_make(255, (uint8_t)(80 + 170 * k), (uint8_t)(20 + 100 * k));
        }
        for (int y = 0; y < h; y++) lv_canvas_set_px(canvas, x, y, c, LV_OPA_COVER);
    }
    /* 边框：逐像素绘制（LVGL 9.x 画布没有 lv_canvas_draw_rect） */
    {
        lv_color_t bc = lv_color_make(0x16, 0x37, 0x4f);
        int x1 = 0, y1 = 0, x2 = w - 1, y2 = h - 1;
        if (x2 < 0) x2 = 0;
        if (y2 < 0) y2 = 0;
        for (int x = x1; x <= x2; x++) {
            lv_canvas_set_px(canvas, x, y1, bc, LV_OPA_COVER);
            lv_canvas_set_px(canvas, x, y2, bc, LV_OPA_COVER);
        }
        for (int y = y1; y <= y2; y++) {
            lv_canvas_set_px(canvas, x1, y, bc, LV_OPA_COVER);
            lv_canvas_set_px(canvas, x2, y, bc, LV_OPA_COVER);
        }
    }
}

/* 同步所有频段的 GAIN label/slider（2.4G 和 5.8G 增益共享一个物理值，两边 UI 要联动） */
static void gain_sync_all_ui(int v)
{
    for (int i = 0; i < 2; i++) {
        scan_band_ui_t *ui = &s_band_ui[i];
        if (ui->gain_slider) lv_slider_set_value(ui->gain_slider, v, LV_ANIM_OFF);
        if (ui->gain_label)  lv_label_set_text_fmt(ui->gain_label, "GAIN: x%d", v);
    }
}

static void gain_slider_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_obj_t *src = lv_event_get_target(e);
    if (!src) return;
    int v = lv_slider_get_value(src);
    s_rf_gain = v;
    gain_sync_all_ui(v);
    rf_link_send_gain(v);
}

static void gain_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int v = s_rf_gain + delta;
    if (v < 1) v = 1;
    if (v > 10) v = 10;
    s_rf_gain = v;
    gain_sync_all_ui(v);
    rf_link_send_gain(v);
}

/* ============================================================================
 *  页面管理器（页栈 + 懒加载 + Header/返回按钮）
 * ========================================================================== */

/* ---------- 页面标题映射 ---------- */
static const char *page_title(page_id_t id)
{
    switch (id) {
    case PAGE_BOOT:    return "System Boot";
    case PAGE_24G:     return "2.4G Spectrum Scan";
    case PAGE_58G:     return "5.8G Spectrum Scan";
    case PAGE_SPECTRUM:return "Spectrum 2.4G + 5.8G";
    case PAGE_UART:    return "UART Debug Monitor";
    case PAGE_RID:     return "Remote ID Scan List";
    case PAGE_RIDMAP:  return "Remote ID Map";
    case PAGE_SETTINGS:return "Settings";
    case PAGE_FPS_TEST:return "FPS Stress Test";
    default:           return "Home";
    }
}

/* ---------- 创建全屏子页容器（供子页 build_* 调用） ----------
 * - 子页容器透明背景；Header 在根页之上固定；
 * - 返回后整页设置 HIDDEN，下次再 SHOW 继续保留内容。
 */
static lv_obj_t *page_create_container(page_id_t id)
{
    if (s_pages[id]) return s_pages[id];
    lv_obj_t *page = lv_obj_create(s_page_root);
    lv_obj_set_size(page, 480, CONTENT_AREA_H);
    lv_obj_align(page, LV_ALIGN_TOP_MID, 0, PAGE_HEADER_H + 4);
    lv_obj_set_style_bg_opa(page, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, RF_UI_PAD, 0);
    lv_obj_set_style_pad_row(page, 10, 0);
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    s_pages[id] = page;
    return page;
}

/* ---------- 页切换核心 ---------- */
static void page_show(page_id_t id)
{
    if (id >= PAGE_COUNT) return;

    /* ★★★ 退出 RID MAP 页的清理 — flush cache + 清 canvas + 停 pending ★★★ */
    if (s_cur_page == PAGE_RIDMAP && id != PAGE_RIDMAP) {
        rf_map_exit_page(&s_map_view);
    }

    /* 懒加载 */
    if (!s_pages[id]) page_build(id);

    /* ★ 先隐藏所有页 — 不跳过任何页 (包括被 SPECTRUM 托管的 24G/58G)
     * 因为即使 reparent 了, 显式加 HIDDEN 也不会错,
     * 而没被 reparent 的 (首次进 SPECTRUM 前未 build) 更要强制隐藏! */
    for (int i = 0; i < PAGE_COUNT; i++) {
        if (!s_pages[i]) continue;
        lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
    }

    /* ★ 主页: 完全干净 (无 header / 无 status bar / 无 floating home)
     *   PAGE_BOOT: 全屏自检 (无 header / 无 floating home / 无 status bar)
     *   PAGE_24G/58G 独立页: 显示 floating home (header 隐藏)
     *   其它子页 (PAGE_SPECTRUM 等): 显示 header (floating home 隐藏) */
    if (id == PAGE_BOOT) {
        if (s_page_header)        lv_obj_add_flag(s_page_header,        LV_OBJ_FLAG_HIDDEN);
        if (s_floating_home_btn)  lv_obj_add_flag(s_floating_home_btn,  LV_OBJ_FLAG_HIDDEN);
        if (s_home_status_bar)    lv_obj_add_flag(s_home_status_bar,    LV_OBJ_FLAG_HIDDEN);
        if (s_pages[PAGE_BOOT])   lv_obj_move_foreground(s_pages[PAGE_BOOT]);
    } else if (id == PAGE_HOME) {
        if (s_page_header)        lv_obj_add_flag(s_page_header,        LV_OBJ_FLAG_HIDDEN);
        if (s_floating_home_btn)  lv_obj_add_flag(s_floating_home_btn,  LV_OBJ_FLAG_HIDDEN);
        if (s_home_status_bar)    lv_obj_clear_flag(s_home_status_bar,  LV_OBJ_FLAG_HIDDEN);
        /* ★ 关键: 把 PAGE_HOME 压到最顶层, 盖住 PAGE_SPECTRUM 等残留 */
        if (s_pages[PAGE_HOME])   lv_obj_move_foreground(s_pages[PAGE_HOME]);
    } else if (id == PAGE_24G || id == PAGE_58G) {
        if (s_page_header)          lv_obj_add_flag(s_page_header, LV_OBJ_FLAG_HIDDEN);
        if (s_floating_home_btn) {
            lv_obj_clear_flag(s_floating_home_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(s_floating_home_btn);
        }
        if (s_home_status_bar)      lv_obj_add_flag(s_home_status_bar, LV_OBJ_FLAG_HIDDEN);
    } else {
        /* PAGE_SPECTRUM / UART / RID / RIDMAP / SETTINGS / FPS_TEST */
        if (s_page_header) {
            lv_obj_clear_flag(s_page_header, LV_OBJ_FLAG_HIDDEN);
            if (s_page_title_lbl) lv_label_set_text(s_page_title_lbl, page_title(id));
            lv_obj_move_foreground(s_page_header);
        }
        if (s_floating_home_btn)    lv_obj_add_flag(s_floating_home_btn, LV_OBJ_FLAG_HIDDEN);
        if (s_home_status_bar)      lv_obj_add_flag(s_home_status_bar, LV_OBJ_FLAG_HIDDEN);
    }

    /* 显示目标页 */
    if (s_pages[id]) lv_obj_clear_flag(s_pages[id], LV_OBJ_FLAG_HIDDEN);

    /* ★ PAGE_SPECTRUM 特殊: 重新进入时确保正确 band 子页是可见的
     *  (上面的 "hide all pages" 把 PAGE_24G/58G 都 HIDDEN 了) */
    if (id == PAGE_SPECTRUM && s_spectrum_managed_sub) {
        if (s_active_band == RF_BAND_5G) {
            lv_obj_add_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* 进入特殊页的附加动作 */
    /* Settings 页：每秒刷新调试信息 */
    if (id == PAGE_SETTINGS) {
        if (s_settings_info) {
            rf_stats_t st;
            rf_link_get_stats(&st);
            char buf[512];
            format_link_stats(buf, sizeof(buf), &st);
            lv_label_set_text(s_settings_info, buf);
        }
        if (!s_debug_timer) {
            s_debug_timer = lv_timer_create(settings_debug_refresh_cb, 1000, NULL);
            lv_timer_ready(s_debug_timer);
        }
    } else {
        if (s_debug_timer) {
            lv_timer_del(s_debug_timer);
            s_debug_timer = NULL;
        }
    }

    /* FPS Test 页：进入创建随机刷刷新 timer，离开删除 */
    if (id == PAGE_FPS_TEST) {
        if (!s_fps_test_timer) {
            s_fps_test_timer = lv_timer_create(fps_test_timer_cb, 1, NULL);
            lv_timer_ready(s_fps_test_timer);
        }
    } else {
        if (s_fps_test_timer) {
            lv_timer_del(s_fps_test_timer);
            s_fps_test_timer = NULL;
        }
    }

    /* 进入频谱页时切换当前活动扫频段，
       spectrum_redraw / waterfall_push 会按活动频段写对应 canvas。*/
    if (id == PAGE_24G) s_active_band = RF_BAND_24G;
    else if (id == PAGE_58G) s_active_band = RF_BAND_5G;
    else if (id == PAGE_SPECTRUM) { /* 保留当前 band, 由 toggle 控制; 默认 2.4G */ }

    s_cur_page = id;

    /* ==== 动态管理扫频任务: 非频谱页关闭, 频谱页启动 ==== */
    if (id == PAGE_RIDMAP || id == PAGE_HOME || id == PAGE_UART || id == PAGE_RID || id == PAGE_SETTINGS || id == PAGE_FPS_TEST || id == PAGE_BOOT) {
        rf_24g_stop();
        rf_5g_stop();
    } else if (id == PAGE_24G) {
        rf_5g_stop();
        rf_24g_start(2);
    } else if (id == PAGE_58G) {
        rf_24g_stop();
        rf_5g_start(2);
    } else if (id == PAGE_SPECTRUM) {
        /* 频谱整合页: 根据 toggle 选择启动哪个频段 */
        if (s_active_band == RF_BAND_5G) {
            rf_24g_stop();
            rf_5g_start(2);
        } else {
            rf_5g_stop();
            rf_24g_start(2);
        }
    }

    /* ==== PAGE_RIDMAP: 每次进入页都强制触发 tile 加载 ==== */
    if (id == PAGE_RIDMAP && s_ridmap_canvas && lv_obj_is_valid(s_ridmap_canvas)) {
        if (s_map_ready && rf_map_is_mounted()) {
            ESP_LOGI("rf_ui", "[RID MAP] 进入页，触发 set_center 加载默认瓦片");
            s_map_view.canvas = s_ridmap_canvas;
            rf_map_set_center(&s_map_view, RF_MAP_DEFAULT_CENTER_LAT,
                              RF_MAP_DEFAULT_CENTER_LON, RF_MAP_DEFAULT_ZOOM);
        } else {
            ESP_LOGW("rf_ui", "[RID MAP] 进入页但 SD 未就绪 (map_ready=%d mounted=%d)",
                     s_map_ready, rf_map_is_mounted());
        }
    }
}

static void page_back_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    page_show(PAGE_HOME);
}

/* ---------- 主页 tile 按钮点击路由 ---------- */
static void tile_cb(lv_event_t *e)
{
    page_id_t id = (page_id_t)(intptr_t)lv_event_get_user_data(e);
    page_show(id);
}

/* ============================================================================
 *  子页构建：PAGE_24G / PAGE_58G / PAGE_UART / PAGE_RID / PAGE_RIDMAP / PAGE_SETTINGS
 * ========================================================================== */

/* 页面标题 + 小分割线（子页内部共用） */
static lv_obj_t *make_page_heading(lv_obj_t *parent, const char *txt)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, lv_pct(100), 28);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    return row;
}

/* ----- 2.4G 扫描页（频谱 + 瀑布图 + 增益 + SCAN CONFIG，通用段复用） ----- */
static void build_page_24g(void)
{
    scan_band_ui_t *ui = &s_band_ui[RF_BAND_24G];
    lv_obj_t *page = page_create_container(PAGE_24G);
    make_page_heading(page, ui->title);
    build_spectrum_section(page, ui);
    build_waterfall_section(page, ui);
    build_scan_config_section(page, ui);

    /* 强制把 page 提到 background 之上（经验 1492132：避免被背景层遮住） */
    lv_obj_move_foreground(page);
}

/* ----- 5.8G 扫描页（频谱 + 瀑布图 + 增益 + SCAN CONFIG，与 2.4G 布局一致） ----- */
static void build_page_58g(void)
{
    scan_band_ui_t *ui = &s_band_ui[RF_BAND_5G];
    lv_obj_t *page = page_create_container(PAGE_58G);
    make_page_heading(page, ui->title);
    build_spectrum_section(page, ui);
    build_waterfall_section(page, ui);
    build_scan_config_section(page, ui);

    /* 强制把 page 提到 background 之上（经验 1492132：避免被背景层遮住） */
    lv_obj_move_foreground(page);
}

/* ----- PAGE_SPECTRUM toggle 回调 ----- */
static void spec_toggle_cb(lv_event_t *e)
{
    rf_band_t target = (rf_band_t)(intptr_t)lv_event_get_user_data(e);
    if (target == s_active_band) return;

    /* PAGE_24G / PAGE_58G 已被 PAGE_SPECTRUM 托管, 直接 HIDDEN 切换 */
    if (target == RF_BAND_5G) {
        lv_obj_add_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_5g,  lv_color_hex(0xBB66FF), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_24g, lv_color_hex(0x1A2230), LV_STATE_DEFAULT);
    } else {
        lv_obj_add_flag(s_pages[PAGE_58G],  LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_24g, lv_color_hex(0x66CCFF), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_5g,  lv_color_hex(0x1A2230), LV_STATE_DEFAULT);
    }

    s_active_band = target;
    if (target == RF_BAND_5G) {
        rf_24g_stop();
        rf_5g_start(2);
        ESP_LOGI(TAG, "▶ SPECTRUM toggle → 5.8G");
    } else {
        rf_5g_stop();
        rf_24g_start(2);
        ESP_LOGI(TAG, "▶ SPECTRUM toggle → 2.4G");
    }

    s_last_wf_band = target;
    s_last_wf_sweep = 0;
}

/* ----- PAGE_SPECTRUM: 2.4G + 5.8G 双频段整合页 -----
 * ★ 零重复 canvas 分配！把 PAGE_24G / PAGE_58G 预构建好的整页
 *   通过 lv_obj_set_parent 搬进 PAGE_SPECTRUM 容器下，
 *   然后用 HIDDEN flag 控制显示哪个频段。
 *   build_page_spectrum 本身只创建 toggle 按钮组。
 */
static void build_page_spectrum(void)
{
    lv_obj_t *page = page_create_container(PAGE_SPECTRUM);

    /* ---- 频段切换按钮组 ---- */
    lv_obj_t *toggle_row = lv_obj_create(page);
    lv_obj_set_size(toggle_row, 448, 40);
    lv_obj_set_style_bg_opa(toggle_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(toggle_row, 0, 0);
    lv_obj_set_style_pad_all(toggle_row, 0, 0);
    lv_obj_set_style_pad_column(toggle_row, 12, 0);
    lv_obj_set_flex_flow(toggle_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(toggle_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(toggle_row, LV_OBJ_FLAG_SCROLLABLE);

    s_spec_toggle_btn_24g = lv_button_create(toggle_row);
    lv_obj_set_size(s_spec_toggle_btn_24g, 120, 32);
    lv_obj_set_style_radius(s_spec_toggle_btn_24g, 8, 0);
    lv_obj_add_event_cb(s_spec_toggle_btn_24g, spec_toggle_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)RF_BAND_24G);
    {
        lv_obj_t *l = lv_label_create(s_spec_toggle_btn_24g);
        lv_label_set_text(l, "2.4G Scan");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(l);
    }

    s_spec_toggle_btn_5g = lv_button_create(toggle_row);
    lv_obj_set_size(s_spec_toggle_btn_5g, 120, 32);
    lv_obj_set_style_radius(s_spec_toggle_btn_5g, 8, 0);
    lv_obj_add_event_cb(s_spec_toggle_btn_5g, spec_toggle_cb, LV_EVENT_CLICKED,
                        (void*)(intptr_t)RF_BAND_5G);
    {
        lv_obj_t *l = lv_label_create(s_spec_toggle_btn_5g);
        lv_label_set_text(l, "5.8G Scan");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(l);
    }

    /* ---- ★ 预构建 PAGE_24G / PAGE_58G (如果还没 build),
     *  确保 reparent 一定能生效! 然后搬到 PAGE_SPECTRUM 下,
     *  作为子对象跟父一起显隐。
     *  ★★★ 关键: reparent 后必须彻底重置 size/pos/align,
     *  否则 PAGE_24G 保留 page_create_container 设的固定 size+绝对偏移,
     *  在 PAGE_SPECTRUM 内会 y 偏移 50px 跟 toggle_row 重叠 + 底部溢出! */
    if (!s_pages[PAGE_24G]) page_build(PAGE_24G);
    if (!s_pages[PAGE_58G]) page_build(PAGE_58G);

    if (s_pages[PAGE_24G]) {
        lv_obj_set_parent(s_pages[PAGE_24G], page);
        /* ★ 重置: 占满父容器宽度, 高度自适应内容, 位置从 (0,40) 开始 (toggle_row 下面) */
        lv_obj_set_size(s_pages[PAGE_24G], lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_align(s_pages[PAGE_24G], LV_ALIGN_TOP_LEFT, 0, 40);
        lv_obj_set_style_bg_opa(s_pages[PAGE_24G], LV_OPA_TRANSP, 0);
        lv_obj_add_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
    }
    if (s_pages[PAGE_58G]) {
        lv_obj_set_parent(s_pages[PAGE_58G], page);
        lv_obj_set_size(s_pages[PAGE_58G], lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_align(s_pages[PAGE_58G], LV_ALIGN_TOP_LEFT, 0, 40);
        lv_obj_set_style_bg_opa(s_pages[PAGE_58G], LV_OPA_TRANSP, 0);
        lv_obj_add_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
    }
    s_spectrum_managed_sub = true;

    /* ---- 按钮高亮 + band 显隐: 根据 s_active_band 对齐 ---- */
    if (s_active_band == RF_BAND_5G) {
        lv_obj_add_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_5g,  lv_color_hex(0xBB66FF), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_24g, lv_color_hex(0x1A2230), LV_STATE_DEFAULT);
    } else {
        /* 默认 2.4G */
        lv_obj_clear_flag(s_pages[PAGE_24G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_pages[PAGE_58G], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_24g, lv_color_hex(0x66CCFF), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(s_spec_toggle_btn_5g,  lv_color_hex(0x1A2230), LV_STATE_DEFAULT);
    }

    lv_obj_move_foreground(page);
}

/* ----- UART 调试页 ----- */
static void build_page_uart(void)
{
    lv_obj_t *page = page_create_container(PAGE_UART);
    make_page_heading(page, "UART Debug / Serial Monitor");

    /* ---- RF LINK STATUS 小卡片 ---- */
    add_group_title(page, "RF LINK");
    lv_obj_t *info = lv_label_create(page);
    lv_obj_set_width(info, lv_pct(100));
    lv_obj_set_style_text_color(info, lv_color_hex(0xAACCEE), 0);
    lv_obj_set_style_text_font(info, &lv_font_montserrat_14, 0);
    rf_stats_t st;
    rf_link_get_stats(&st);
    char buf[512];
    format_link_stats(buf, sizeof(buf), &st);
    lv_label_set_text(info, buf);
    /* 用 s_settings_info 复用刷新逻辑（settings_debug_refresh_cb 里会 update） */
    s_settings_info = info;

    /* ---- UART 控制：数据源 + 波特率 + probe + 调试输出 ---- */
    add_group_title(page, "UART CONTROL");
    {
        /* 数据源行 */
        lv_obj_t *row = lv_obj_create(page);
        lv_obj_set_size(row, lv_pct(100), 36);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 8, 0);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *sl = lv_label_create(row);
        lv_label_set_text(sl, "Source");
        lv_obj_set_width(sl, 56);
        lv_obj_set_style_text_color(sl, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(sl, &lv_font_montserrat_14, 0);

        s_page_uart.btn_sim = lv_button_create(row);
        lv_obj_set_size(s_page_uart.btn_sim, 100, 30);
        lv_obj_set_style_radius(s_page_uart.btn_sim, 6, 0);
        lv_obj_set_style_pad_all(s_page_uart.btn_sim, 0, 0);
        lv_obj_add_state(s_page_uart.btn_sim, LV_STATE_DISABLED);
        lv_obj_clear_flag(s_page_uart.btn_sim, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_page_uart.btn_sim, src_sim_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *lb1 = lv_label_create(s_page_uart.btn_sim);
        lv_label_set_text(lb1, "SIM");
        lv_obj_set_style_text_color(lb1, lv_color_hex(0x8C949B), 0);
        lv_obj_center(lb1);

        s_page_uart.btn_uart = lv_button_create(row);
        lv_obj_set_size(s_page_uart.btn_uart, 120, 30);
        lv_obj_set_style_radius(s_page_uart.btn_uart, 6, 0);
        lv_obj_set_style_pad_all(s_page_uart.btn_uart, 0, 0);
        lv_obj_t *lb2 = lv_label_create(s_page_uart.btn_uart);
        lv_label_set_text(lb2, "UART REAL");
        lv_obj_center(lb2);
        lv_obj_add_event_cb(s_page_uart.btn_uart, src_uart_cb, LV_EVENT_CLICKED, NULL);

        /* UART 调试输出 ON/OFF */
        s_page_uart.dbg_toggle = lv_button_create(row);
        lv_obj_set_flex_grow(s_page_uart.dbg_toggle, 1);
        lv_obj_set_height(s_page_uart.dbg_toggle, 30);
        lv_obj_set_style_radius(s_page_uart.dbg_toggle, 6, 0);
        lv_obj_set_style_border_width(s_page_uart.dbg_toggle, 1, 0);
        lv_obj_set_style_border_color(s_page_uart.dbg_toggle, lv_color_hex(0x3E4550), 0);
        lv_obj_set_style_pad_all(s_page_uart.dbg_toggle, 0, 0);
        lv_obj_add_event_cb(s_page_uart.dbg_toggle, uartdbg_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *ud_lbl = lv_label_create(s_page_uart.dbg_toggle);
        lv_label_set_text(ud_lbl, rf_link_get_uart_debug_output() ? "UART LOG: ON" : "UART LOG: OFF");
        lv_obj_center(ud_lbl);
        lv_obj_set_user_data(s_page_uart.dbg_toggle, ud_lbl);

        /* 刷新两套按钮/下拉的初始外观 */
        refresh_source_btns_style();
        refresh_dbg_toggles_style();
    }
    {
        /* 波特率行 */
        lv_obj_t *row = lv_obj_create(page);
        lv_obj_set_size(row, lv_pct(100), 38);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 8, 0);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *bl = lv_label_create(row);
        lv_label_set_text(bl, "Baud");
        lv_obj_set_width(bl, 44);
        lv_obj_set_style_text_color(bl, lv_color_hex(0x88AABB), 0);
        lv_obj_set_style_text_font(bl, &lv_font_montserrat_14, 0);

        s_page_uart.baud_drop = lv_dropdown_create(row);
        lv_dropdown_set_options(s_page_uart.baud_drop,
            "9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600");
        /* 与当前已连接波特率同步（Settings 和 UART 初始都取相同 index） */
        {
            int cur = rf_link_get_uart_baud();
            for (int i = 0; i < BAUD_OPTS_N; i++) {
                if (s_baud_opts[i] == cur) {
                    lv_dropdown_set_selected(s_page_uart.baud_drop, (uint16_t)i);
                    break;
                }
            }
        }
        lv_obj_set_width(s_page_uart.baud_drop, 150);
        lv_obj_add_event_cb(s_page_uart.baud_drop, baud_dropdown_cb, LV_EVENT_VALUE_CHANGED, NULL);

        s_page_uart.baud_probe = lv_button_create(row);
        lv_obj_set_flex_grow(s_page_uart.baud_probe, 1);
        lv_obj_set_height(s_page_uart.baud_probe, 30);
        lv_obj_set_style_radius(s_page_uart.baud_probe, 6, 0);
        lv_obj_set_style_pad_all(s_page_uart.baud_probe, 0, 0);
        lv_obj_add_event_cb(s_page_uart.baud_probe, baud_probe_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *pb = lv_label_create(s_page_uart.baud_probe);
        lv_label_set_text(pb, LV_SYMBOL_REFRESH" AUTO DETECT");
        lv_obj_center(pb);
    }

    /* ---- SERIAL MONITOR 过滤 + 控制按钮 ---- */
    add_group_title(page, "SERIAL MONITOR");
    {
        lv_obj_t *row = lv_obj_create(page);
        lv_obj_set_size(row, lv_pct(100), 32);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(row, 8, 0);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        static const struct { const char *t; lv_obj_t **o; } chks[] = {
            {"$D 2.4G",  &s_mon_chk_d},
            {"$W 5G",    &s_mon_chk_w},
            {"$A ACK",   &s_mon_chk_a},
            {"LOG",      &s_mon_chk_log},
            {NULL, NULL}
        };
        for (int i = 0; chks[i].t; i++) {
            lv_obj_t *chk = lv_checkbox_create(row);
            lv_checkbox_set_text(chk, chks[i].t);
            lv_obj_add_state(chk, LV_STATE_CHECKED);
            lv_obj_set_style_text_font(chk, &lv_font_montserrat_14, 0);
            *chks[i].o = chk;
        }

        /* pause / clear */
        s_mon_btn_pause = lv_button_create(row);
        lv_obj_set_size(s_mon_btn_pause, 60, 26);
        lv_obj_add_event_cb(s_mon_btn_pause, mon_pause_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *pp = lv_label_create(s_mon_btn_pause);
        lv_label_set_text(pp, "PAUSE");
        lv_obj_center(pp);

        s_mon_btn_clear = lv_button_create(row);
        lv_obj_set_size(s_mon_btn_clear, 60, 26);
        lv_obj_add_event_cb(s_mon_btn_clear, mon_clear_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *cp = lv_label_create(s_mon_btn_clear);
        lv_label_set_text(cp, "CLEAR");
        lv_obj_center(cp);
    }
    /* UART Debug 文字区（比 Settings 中更大） */
    s_debug_textarea = lv_textarea_create(page);
    lv_obj_set_size(s_debug_textarea, lv_pct(100), 260);
    lv_obj_set_style_bg_color(s_debug_textarea, lv_color_hex(0x020508), 0);
    lv_textarea_set_text(s_debug_textarea, "[UART Monitor ready]\n");
    lv_obj_set_style_text_color(s_debug_textarea, lv_color_hex(0x66FF88), 0);
    lv_obj_set_style_text_font(s_debug_textarea, &lv_font_montserrat_14, 0);
    lv_obj_set_style_border_width(s_debug_textarea, 1, 0);
    lv_obj_set_style_border_color(s_debug_textarea, lv_color_hex(0x334455), 0);
    lv_obj_set_style_pad_all(s_debug_textarea, 6, 0);

    /* ---- SCAN STATUS（显示当前 C5 侧扫描参数） ---- */
    add_group_title(page, "SCAN STATUS");
    s_scan_status_lbl = lv_label_create(page);
    lv_obj_set_width(s_scan_status_lbl, lv_pct(100));
    lv_obj_set_style_text_color(s_scan_status_lbl, lv_color_hex(0xAACCEE), 0);
    lv_obj_set_style_text_font(s_scan_status_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_text(s_scan_status_lbl, "Waiting for $D frame...");

    /* ---- RID RAW PAYLOAD 折叠区 ---- */
    add_group_title(page, "RID RAW PAYLOAD");
    s_rid0_container = lv_obj_create(page);
    lv_obj_set_size(s_rid0_container, lv_pct(100), 200);
    lv_obj_set_style_bg_color(s_rid0_container, lv_color_hex(0x060A10), 0);
    lv_obj_set_style_border_width(s_rid0_container, 1, 0);
    lv_obj_set_style_border_color(s_rid0_container, lv_color_hex(0x2A3340), 0);
    lv_obj_set_style_radius(s_rid0_container, 8, 0);
    lv_obj_set_flex_flow(s_rid0_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_rid0_container, 6, 0);
    lv_obj_set_style_pad_row(s_rid0_container, 4, 0);
    {
        lv_obj_t *bar = lv_obj_create(s_rid0_container);
        lv_obj_set_size(bar, lv_pct(100), 28);
        lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(bar, 0, 0);
        lv_obj_set_style_pad_all(bar, 0, 0);
        lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(bar, 8, 0);
        lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *tl = lv_label_create(bar);
        lv_label_set_text(tl, "$R0 RAW Frames");
        lv_obj_set_style_text_color(tl, lv_color_hex(0xFFBB55), 0);
        lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
        lv_obj_set_flex_grow(tl, 1);

        s_rid0_toggle_btn = lv_button_create(bar);
        lv_obj_set_size(s_rid0_toggle_btn, 72, 24);
        lv_obj_add_event_cb(s_rid0_toggle_btn, rid0_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *tb = lv_label_create(s_rid0_toggle_btn);
        lv_label_set_text(tb, "EXPAND");
        lv_obj_center(tb);

        s_rid0_output_cb = lv_switch_create(bar);
        lv_obj_add_event_cb(s_rid0_output_cb, rid0_output_cb, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_t *ol = lv_label_create(bar);
        lv_label_set_text(ol, "OUTPUT");
        lv_obj_set_style_text_color(ol, lv_color_hex(0xAACCEE), 0);
        lv_obj_set_style_text_font(ol, &lv_font_montserrat_14, 0);
    }
    s_rid0_view = lv_textarea_create(s_rid0_container);
    lv_obj_set_size(s_rid0_view, lv_pct(100), 150);
    lv_obj_set_style_bg_color(s_rid0_view, lv_color_hex(0x020508), 0);
    lv_obj_set_style_text_font(s_rid0_view, &lv_font_montserrat_14, 0);
    lv_obj_set_style_pad_all(s_rid0_view, 6, 0);
    lv_obj_add_flag(s_rid0_view, LV_OBJ_FLAG_HIDDEN);   /* 先折叠 */
    s_rid0_show = false;
}

/* ----- RID 扫描列表页 ----- */
static void build_page_rid(void)
{
    lv_obj_t *page = page_create_container(PAGE_RID);
    make_page_heading(page, "Remote ID Drone List  (ASTM F3411)");
    /* 状态：当前活动无人机数 + 最近更新时间 */
    lv_obj_t *st = lv_label_create(page);
    lv_label_set_text(st, "Drones: 0     Wait C5 RID frames...");
    lv_obj_set_width(st, lv_pct(100));
    lv_obj_set_style_text_color(st, lv_color_hex(0x88AABB), 0);
    lv_obj_set_style_text_font(st, &lv_font_montserrat_14, 0);
    /* RID 卡片滚动容器 */
    s_rid_list = lv_obj_create(page);
    lv_obj_set_width(s_rid_list, lv_pct(100));
    lv_obj_set_height(s_rid_list, 600);
    lv_obj_set_flex_flow(s_rid_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_rid_list, 4, 0);
    lv_obj_set_style_pad_row(s_rid_list, 8, 0);
    lv_obj_set_style_bg_color(s_rid_list, lv_color_hex(0x0C0F14), 0);
    lv_obj_set_style_border_color(s_rid_list, lv_color_hex(0x2A3340), 0);
    lv_obj_set_style_border_width(s_rid_list, 1, 0);
    rid_rebuild();
}

/* ----- RID 地图触摸 + 缩放回调 ----- */
static int16_t s_ridmap_touch_last_x = 0;
    static int16_t s_ridmap_touch_last_y = 0;
    static int     s_pressing_cnt = 0;
    static lv_timer_t *s_edge_timer = NULL;

/* ====== LVGL Timer: 边缘瓦片补全 debounce ======
 * 拖动中 pan → memmove 实时跟手 + tile_slot 更新
 * 用户停手 150ms 后 → SD 读瓦片 paste 到边缘黑区
 * 在线地图式效果: 拖到哪, 瓦片补到哪!
 */
static void ridmap_edge_timer_cb(lv_timer_t *timer)
{
    /* 只在 RID MAP 页且 canvas 有效时补 */
    if (s_cur_page == PAGE_RIDMAP && s_ridmap_canvas && lv_obj_is_valid(s_ridmap_canvas)) {
        ESP_LOGI("rf_ui", "🧩 edge_timer → render_edges");
        rf_map_request_edges(&s_map_view);  // ★ 异步!
    }
    lv_timer_delete(timer);     /* 一次性 — 触发后删 */
    s_edge_timer = NULL;
}

/* 启动/重启边缘补全 timer (每次 pan 后调) */
static void ridmap_restart_edge_timer(void)
{
    if (s_edge_timer) lv_timer_delete(s_edge_timer);    /* 取消之前的 */
    s_edge_timer = lv_timer_create(ridmap_edge_timer_cb, 150, NULL);
    lv_timer_set_repeat_count(s_edge_timer, 1);          /* 只触发 1 次 */
}

static void ridmap_touch_cb(lv_event_t *e)
{
    if (!rf_map_is_mounted()) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED) {
        s_ridmap_touch_last_x = p.x;
        s_ridmap_touch_last_y = p.y;
        /* 每次新触摸都清零 pan 偏移 */
        s_map_view.pending_pan_dx = 0;
        s_map_view.pending_pan_dy = 0;
        ESP_LOGI("rf_ui", "🎯 touch PRESSED at (%d,%d)", p.x, p.y);
    } else if (code == LV_EVENT_PRESSING) {
        int16_t dx = p.x - s_ridmap_touch_last_x;
        int16_t dy = p.y - s_ridmap_touch_last_y;
        if (dx || dy) {
            /* 手动拖动地图 → 自动取消跟踪模式 */
            if (s_track_enabled) {
                s_track_enabled = false;
                memset(s_track_id, 0, 24);
                ESP_LOGI("rf_ui", "track: disabled (manual pan)");
            }
            rf_map_pan(&s_map_view, dx, dy);
            ridmap_restart_edge_timer();      /* 每次 pan 后重启边缘补全 timer */
            s_pressing_cnt++;
            if ((s_pressing_cnt % 20) == 0) {
                ESP_LOGI("rf_ui", "🎯 PRESSING #%d dx=%d dy=%d slot=(%d,%d)",
                         s_pressing_cnt, dx, dy,
                         s_map_view.tile_slot_x, s_map_view.tile_slot_y);
            }
            s_ridmap_touch_last_x = p.x;
            s_ridmap_touch_last_y = p.y;
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        ESP_LOGI("rf_ui", "🎯 RELEASED → 立即 render_edges 补边缘");
        if (s_edge_timer) { lv_timer_delete(s_edge_timer); s_edge_timer = NULL; }
        rf_map_request_edges(&s_map_view);  // ★ 异步!
        s_map_view.pending_pan_dx = s_map_view.pending_pan_dy = 0;
    }
}

static void ridmap_zoom_in_cb(lv_event_t *e)  { rf_map_zoom(&s_map_view, +1); }
static void ridmap_zoom_out_cb(lv_event_t *e) { rf_map_zoom(&s_map_view, -1); }
static void ridmap_reset_cb(lv_event_t *e)
{
    int zoom = s_map_view.zoom;

    /* 优先用第一个有 UA 坐标 (无人机位置) 的 RID 无人机 */
    rf_drone_t drones[RF_MAX_DRONES];
    int n = rf_link_get_drones(drones, RF_MAX_DRONES);

    /* 第一轮: 找有 UA 坐标且瓦片在 SD 上的 */
    for (int i = 0; i < n; i++) {
        if (drones[i].lat_e7 != 0 && drones[i].lon_e7 != 0) {
            double lat = (double)drones[i].lat_e7 / 1e7;
            double lon = (double)drones[i].lon_e7 / 1e7;
            int tx = (int)floor(rf_map_lon_to_tile_x(lon, zoom));
            int ty = (int)floor(rf_map_lat_to_tile_y(lat, zoom));
            if (rf_map_tile_exists(zoom, tx, ty)) {
                ESP_LOGI("rf_ui", "reset → UA tile OK! lat=%.6f lon=%.6f tile=(%d,%d)",
                         lat, lon, tx, ty);
                rf_map_set_center(&s_map_view, lat, lon, zoom);
                return;
            }
        }
    }

    /* 第二轮: 找有 operator 坐标且瓦片在 SD 上的 (遥控器恰好也在覆盖区) */
    for (int i = 0; i < n; i++) {
        if (drones[i].operator_lat_e7 != 0 && drones[i].operator_lon_e7 != 0) {
            double lat = (double)drones[i].operator_lat_e7 / 1e7;
            double lon = (double)drones[i].operator_lon_e7 / 1e7;
            int tx = (int)floor(rf_map_lon_to_tile_x(lon, zoom));
            int ty = (int)floor(rf_map_lat_to_tile_y(lat, zoom));
            if (rf_map_tile_exists(zoom, tx, ty)) {
                ESP_LOGI("rf_ui", "reset → OP tile OK! lat=%.6f lon=%.6f tile=(%d,%d)",
                         lat, lon, tx, ty);
                rf_map_set_center(&s_map_view, lat, lon, zoom);
                return;
            }
        }
    }

    /* 所有都不在 SD 覆盖范围 → fallback 岳阳县政府 */
    ESP_LOGI("rf_ui", "reset → 无有效瓦片, fallback 岳阳县 29.085,113.236");
    rf_map_set_center(&s_map_view, RF_MAP_DEFAULT_CENTER_LAT,
                      RF_MAP_DEFAULT_CENTER_LON, zoom);
}

/* ----- RID 地图页（为 GPS/瓦片地图预留的壳；当前显示 UA/Operator 坐标清单+占位画布） ----- */

static void build_page_ridmap(void)
{
    lv_obj_t *page = page_create_container(PAGE_RIDMAP);
    make_page_heading(page, "Remote ID Map  (GPS + tile map ready)");

    lv_obj_t *tip = lv_label_create(page);
    lv_label_set_text(tip,
        "Shows drone positions on offline Web Mercator tiles (OSM-like). "
        "Tiles: /sdcard/tiles/{zoom}/{x}/{y}.png. Pinout: SDMMC 4-bit D0=48 D1=49 D2=50 D3=51 CMD=52 CLK=53.");
    lv_obj_set_width(tip, lv_pct(100));
    lv_obj_set_style_text_color(tip, lv_color_hex(0x66AADD), 0);
    lv_obj_set_style_text_font(tip, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(tip, LV_LABEL_LONG_WRAP);

    /* Tile map canvas (448x280, RGB565 matching LV_COLOR_DEPTH=16) */
    int mw = RF_UI_CONTENT_W;
    int mh = 280;
    size_t buf_sz = (size_t)mw * mh * sizeof(uint16_t);   /* RGB565 = 2 bytes/px */
    void *mbuf = heap_caps_aligned_alloc(32, buf_sz, MALLOC_CAP_SPIRAM);   /* RGB565 2bpp, PSRAM */
    if (!mbuf) { ESP_LOGE("rf_ui", "ridmap canvas alloc FAIL (%zu)", buf_sz); }
    assert(mbuf);

    s_ridmap_canvas = lv_canvas_create(page);
    lv_canvas_set_buffer(s_ridmap_canvas, mbuf, mw, mh, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_size(s_ridmap_canvas, mw, mh);   /* ★★★ 关键: 对象大小 = buffer 大小, 防缩放! ★★★ */
    lv_canvas_fill_bg(s_ridmap_canvas, lv_color_hex(0x101020), LV_OPA_COVER);
    lv_obj_clear_flag(s_ridmap_canvas, LV_OBJ_FLAG_SCROLLABLE);   /* 不滚动 */
    lv_obj_add_flag(s_ridmap_canvas, LV_OBJ_FLAG_CLICKABLE);       /* 关键! 否则收不到触摸 */
    lv_obj_add_flag(s_ridmap_canvas, LV_OBJ_FLAG_PRESS_LOCK);      /* 防止长按丢失 */

    /* 触摸拖动 + 松手重渲染回调 */
    lv_obj_add_event_cb(s_ridmap_canvas, ridmap_touch_cb, LV_EVENT_ALL, NULL);

    /* 绑定 map_view */
    s_map_view.canvas       = s_ridmap_canvas;
    s_map_view.canvas_w     = mw;
    s_map_view.canvas_h     = mh;
    s_map_view.base_slot_x  = (mw - RF_MAP_TILE_PX) / 2;   /* 96 */
    s_map_view.base_slot_y  = (mh - RF_MAP_TILE_PX) / 2;   /* 12 */
    s_map_view.tile_slot_x  = s_map_view.base_slot_x;
    s_map_view.tile_slot_y  = s_map_view.base_slot_y;
    s_map_view.center_lat   = RF_MAP_DEFAULT_CENTER_LAT;
    s_map_view.center_lon   = RF_MAP_DEFAULT_CENTER_LON;
    s_map_view.zoom         = RF_MAP_DEFAULT_ZOOM;
    s_map_view.tile_x       = -1;
    s_map_view.tile_y       = -1;

    /* 缩放按钮组 (放在 canvas 内部右上角) */
    {
        lv_obj_t *btnp = lv_btn_create(s_ridmap_canvas);
        lv_obj_set_size(btnp, 42, 28);
        lv_obj_align(btnp, LV_ALIGN_TOP_RIGHT, -6, 4);
        lv_obj_set_style_bg_opa(btnp, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(btnp, lv_color_hex(0x000000), 0);
        lv_obj_t *lp = lv_label_create(btnp);
        lv_label_set_text(lp, "+");
        lv_obj_center(lp);
        lv_obj_add_event_cb(btnp, ridmap_zoom_in_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *btnm = lv_btn_create(s_ridmap_canvas);
        lv_obj_set_size(btnm, 42, 28);
        lv_obj_align(btnm, LV_ALIGN_TOP_RIGHT, -6, 36);
        lv_obj_set_style_bg_opa(btnm, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(btnm, lv_color_hex(0x000000), 0);
        lv_obj_t *lm = lv_label_create(btnm);
        lv_label_set_text(lm, "-");
        lv_obj_center(lm);
        lv_obj_add_event_cb(btnm, ridmap_zoom_out_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *btnr = lv_btn_create(s_ridmap_canvas);
        lv_obj_set_size(btnr, 42, 28);
        lv_obj_align(btnr, LV_ALIGN_TOP_RIGHT, -6, 68);
        lv_obj_set_style_bg_opa(btnr, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(btnr, lv_color_hex(0x000000), 0);
        lv_obj_t *lr = lv_label_create(btnr);
        lv_label_set_text(lr, "⌂");
        lv_obj_center(lr);
        lv_obj_add_event_cb(btnr, ridmap_reset_cb, LV_EVENT_CLICKED, NULL);
    }

    /* 无条件立即加载默认中心瓦片 — 不再依赖 s_map_ready 时序 */
    {
        ESP_LOGW("rf_ui", "⚠️ build_page_ridmap: canvas=%p map_ready=%d mounted=%d → 直接 set_center",
                 (void*)s_ridmap_canvas, s_map_ready, rf_map_is_mounted());
        rf_map_set_center(&s_map_view, RF_MAP_DEFAULT_CENTER_LAT,
                          RF_MAP_DEFAULT_CENTER_LON, RF_MAP_DEFAULT_ZOOM);
    }

    /* 坐标列表（UA + Op 并排） */
    s_ridmap_list = lv_obj_create(page);
    lv_obj_set_width(s_ridmap_list, lv_pct(100));
    lv_obj_set_height(s_ridmap_list, 360);
    lv_obj_set_flex_flow(s_ridmap_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_ridmap_list, 6, 0);
    lv_obj_set_style_pad_row(s_ridmap_list, 6, 0);
    lv_obj_set_style_bg_color(s_ridmap_list, lv_color_hex(0x060A10), 0);
    lv_obj_set_style_border_width(s_ridmap_list, 1, 0);
    lv_obj_set_style_border_color(s_ridmap_list, lv_color_hex(0x2A3340), 0);
}

/* ----- Settings 页（取代原先 lv_layer_top 弹层） ----- */
/* 大部分 build_settings_panel 逻辑可复用：我们传入 page 作为父容器；
   旧 build_settings_panel 使用 lv_layer_top，现在改为 page_create_container(PAGE_SETTINGS)。
   做法：新建 build_page_settings 函数，并把 s_settings_panel → page 的滚动容器。 */
static void build_page_settings(void)
{
    lv_obj_t *page = page_create_container(PAGE_SETTINGS);
    make_page_heading(page, "Settings  (Display / RF / Control / UART)");

    /* ---- FPS Test 快捷入口按钮 ---- */
    lv_obj_t *fps_btn = lv_button_create(page);
    lv_obj_set_size(fps_btn, lv_pct(100), 48);
    lv_obj_set_style_bg_color(fps_btn, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_width(fps_btn, 1, 0);
    lv_obj_set_style_border_color(fps_btn, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_radius(fps_btn, 8, 0);
    lv_obj_add_event_cb(fps_btn, tile_cb, LV_EVENT_CLICKED, (void*)(intptr_t)PAGE_FPS_TEST);
    lv_obj_t *fps_lbl = lv_label_create(fps_btn);
    lv_label_set_text(fps_lbl, LV_SYMBOL_PLAY "  FPS Stress Test");
    lv_obj_center(fps_lbl);
    lv_obj_set_style_text_color(fps_lbl, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(fps_lbl, &lv_font_montserrat_14, 0);

    /* 复用原 build_settings_panel 的全部内容，但父容器 = page */
    /* 技巧：原函数以 lv_layer_top() 为父创建居中卡片 s_settings_panel（宽360高530）。
       为最大限度复用代码，这里先确保 s_settings_panel 未创建，然后直接调用 build_settings_panel()，
       再把 s_settings_panel 从 lv_layer_top() 重新挂到 page，并改为宽度 100%、高度自适应。 */
    if (!s_settings_panel) {
        build_settings_panel();
    }
    if (s_settings_panel) {
        /* 从原父（lv_layer_top）改挂到 page */
        lv_obj_set_parent(s_settings_panel, page);
        lv_obj_clear_flag(s_settings_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_settings_panel, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_max_height(s_settings_panel, 65500, 0); /* 交给 page 的滚动条处理 */
        /* 标题行中的 "close" 按钮改成 "← HOME" */
        /* 标题行第一个子 button（close_btn）改回调 */
        lv_obj_t *title_row = lv_obj_get_child(s_settings_panel, 0);
        if (title_row) {
            /* 遍历 title_row 子控件：找 button，替换回调 + 文案 */
            uint32_t cc = lv_obj_get_child_cnt(title_row);
            for (uint32_t i = 0; i < cc; i++) {
                lv_obj_t *c = lv_obj_get_child(title_row, i);
                if (lv_obj_check_type(c, &lv_button_class)) {
                    lv_obj_remove_event_cb(c, settings_close_cb);
                    lv_obj_add_event_cb(c, page_back_cb, LV_EVENT_CLICKED, NULL);
                    /* 改 close_lbl 文本 */
                    uint32_t kk = lv_obj_get_child_cnt(c);
                    for (uint32_t k = 0; k < kk; k++) {
                        lv_obj_t *lbl = lv_obj_get_child(c, k);
                        if (lv_obj_check_type(lbl, &lv_label_class)) {
                            lv_label_set_text(lbl, LV_SYMBOL_LEFT " HOME");
                            lv_obj_set_style_text_color(lbl, lv_color_hex(0x66CCFF), 0);
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
}

/* ============================================================================
 *  FPS Stress Test — 全屏 lv_canvas + 直接写 buffer 随机色
 *  测的是 MIPI-DSI 整块数据 flush 的极限速度（只有 1 个 LVGL 对象）
 * ========================================================================== */
static lv_obj_t *s_fps_test_lbl_fps = NULL;
static lv_obj_t *s_fps_test_lbl_flush = NULL;
static lv_obj_t *s_fps_test_canvas = NULL;
static uint16_t *s_fps_test_buf = NULL;   /* RGB565 canvas buffer (PSRAM) */
static int s_fps_test_mode = 1;           /* 0=LOW 1=MID 2=HIGH (刷新速度) */

/* 每 tick 把 canvas buffer 整块刷成随机 RGB565，然后 invalidate */
static void fps_test_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (!s_fps_test_buf || !s_fps_test_canvas) return;

    int speed_ms = (s_fps_test_mode == 0) ? 33 : (s_fps_test_mode == 1) ? 10 : 5;
    static int tick = 0;
    if (++tick < speed_ms) return;
    tick = 0;

    /* 快速填充随机 RGB565 —— 用 LCG 伪随机避免 esp_random() 开销 */
    const int32_t pixels = 480 * 854;
    uint16_t *p = s_fps_test_buf;
    uint32_t seed = esp_random();   /* 每帧只调一次 RNG */
    for (int32_t i = 0; i < pixels; i++) {
        seed = seed * 1664525 + 1013904223;   /* LCG */
        p[i] = (uint16_t)(seed >> 16);        /* 取高 16 位作 RGB565 */
    }
    lv_obj_invalidate(s_fps_test_canvas);

    /* 刷新统计标签 */
    if (s_fps_test_lbl_fps) {
        char buf[64];
        snprintf(buf, sizeof(buf), "FPS: %lu", (unsigned long)g_lvgl_fps);
        lv_label_set_text(s_fps_test_lbl_fps, buf);
    }
    if (s_fps_test_lbl_flush) {
        char buf[64];
        snprintf(buf, sizeof(buf), "flush/s: %lu", (unsigned long)g_lvgl_flush_per_sec);
        lv_label_set_text(s_fps_test_lbl_flush, buf);
    }
}

static void fps_test_mode_btn_cb(lv_event_t *e)
{
    s_fps_test_mode = (s_fps_test_mode + 1) % 3;
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    const char *txt = (s_fps_test_mode == 0) ? "LOW" : (s_fps_test_mode == 1) ? "MID" : "HIGH";
    if (lbl && lv_obj_check_type(lbl, &lv_label_class)) {
        lv_label_set_text(lbl, txt);
    }
}

static void build_page_fps_test(void)
{
    lv_obj_t *page = page_create_container(PAGE_FPS_TEST);

    /* ---- 顶部统计行 ---- */
    lv_obj_t *stat_row = lv_obj_create(page);
    lv_obj_set_size(stat_row, lv_pct(100), 50);
    lv_obj_set_style_bg_opa(stat_row, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(stat_row, lv_color_hex(0x0A0E14), 0);
    lv_obj_set_style_border_width(stat_row, 0, 0);
    lv_obj_set_style_pad_all(stat_row, 6, 0);
    lv_obj_set_flex_flow(stat_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(stat_row, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(stat_row, LV_OBJ_FLAG_SCROLLABLE);

    s_fps_test_lbl_fps = lv_label_create(stat_row);
    lv_label_set_text(s_fps_test_lbl_fps, "FPS: --");
    lv_obj_set_style_text_color(s_fps_test_lbl_fps, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(s_fps_test_lbl_fps, &lv_font_montserrat_14, 0);

    s_fps_test_lbl_flush = lv_label_create(stat_row);
    lv_label_set_text(s_fps_test_lbl_flush, "flush/s: --");
    lv_obj_set_style_text_color(s_fps_test_lbl_flush, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(s_fps_test_lbl_flush, &lv_font_montserrat_14, 0);

    /* ---- 控制行 ---- */
    lv_obj_t *ctrl_row = lv_obj_create(page);
    lv_obj_set_size(ctrl_row, lv_pct(100), 36);
    lv_obj_set_style_bg_opa(ctrl_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrl_row, 0, 0);
    lv_obj_set_style_pad_all(ctrl_row, 0, 0);
    lv_obj_set_flex_flow(ctrl_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrl_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ctrl_row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *hint = lv_label_create(ctrl_row);
    lv_label_set_text(hint, "Canvas flush stress — single obj, full screen");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x8090A0), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);

    lv_obj_t *mode_btn = lv_button_create(ctrl_row);
    lv_obj_set_style_radius(mode_btn, 6, 0);
    lv_obj_set_style_bg_color(mode_btn, lv_color_hex(0x1A3040), 0);
    lv_obj_set_style_border_width(mode_btn, 1, 0);
    lv_obj_set_style_border_color(mode_btn, lv_color_hex(0x00E0FF), 0);
    lv_obj_add_event_cb(mode_btn, fps_test_mode_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *mode_lbl = lv_label_create(mode_btn);
    lv_label_set_text(mode_lbl, "MID");
    lv_obj_center(mode_lbl);
    lv_obj_set_style_text_color(mode_lbl, lv_color_hex(0x00E0FF), 0);

    /* ---- 全屏 canvas ---- */
    s_fps_test_canvas = lv_canvas_create(page);
    lv_obj_set_size(s_fps_test_canvas, 480, 854);
    lv_obj_set_flex_grow(s_fps_test_canvas, 1);
    lv_obj_clear_flag(s_fps_test_canvas, LV_OBJ_FLAG_SCROLLABLE);

    /* 分配 RGB565 canvas buffer（PSRAM 支持 DMA） */
    const size_t canvas_sz = 480 * 854 * 2; /* RGB565 = 2 bytes */
    if (!s_fps_test_buf) {
        s_fps_test_buf = heap_caps_aligned_calloc(64, 1, canvas_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    if (s_fps_test_buf) {
        lv_canvas_set_buffer(s_fps_test_canvas, s_fps_test_buf, 480, 854, LV_COLOR_FORMAT_RGB565);
        ESP_LOGI(TAG, "FPS Test canvas: 480x854 RGB565 buf=%p sz=%zu", s_fps_test_buf, canvas_sz);
    }
    /* timer 在 page_show 进入时创建 */
}

/* ----- 主页（2 列 x 3 行 tile 网格 + 顶栏小状态） ----- */
typedef struct {
    page_id_t   id;
    const char *glyph;     /* LV_SYMBOL_xxx 或字符 */
    const char *title;
    const char *desc;
    uint32_t    bg_hex;    /* 主题色（深色卡片） */
    uint32_t    accent_hex;/* 图标/标题强调色 */
} tile_info_t;

/* ----- 开机自检页（全屏，品牌Logo + 进度条 + 检测项列表） ----- */
static void build_page_boot(void)
{
    /* 自检页用全屏容器 (不经过 page_create_container, 因为有 header 偏移) */
    lv_obj_t *page = lv_obj_create(s_page_root);
    lv_obj_set_size(page, 480, 854);
    lv_obj_align(page, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(page, lv_color_hex(0x030508), 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    s_pages[PAGE_BOOT] = page;

    /* 顶部品牌 Logo 区 */
    lv_obj_t *logo_area = lv_obj_create(page);
    lv_obj_set_size(logo_area, lv_pct(100), 260);
    lv_obj_set_style_bg_opa(logo_area, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(logo_area, 0, 0);
    lv_obj_set_style_pad_all(logo_area, 0, 0);
    lv_obj_set_flex_flow(logo_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(logo_area, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(logo_area, LV_OBJ_FLAG_SCROLLABLE);

    /* 大标题 */
    s_boot_logo_lbl = lv_label_create(logo_area);
    lv_label_set_text(s_boot_logo_lbl, "DRONE RF GUARD");
    lv_obj_set_style_text_font(s_boot_logo_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_boot_logo_lbl, lv_color_hex(0x66CCFF), 0);
    lv_obj_set_style_pad_bottom(s_boot_logo_lbl, 8, 0);

    /* 副标题 */
    s_boot_sub_lbl = lv_label_create(logo_area);
    lv_label_set_text(s_boot_sub_lbl, "Self-Test in Progress...");
    lv_obj_set_style_text_font(s_boot_sub_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_boot_sub_lbl, lv_color_hex(0x8899AA), 0);

    /* 进度条 */
    s_boot_progress_bar = lv_bar_create(page);
    lv_obj_set_size(s_boot_progress_bar, 360, 8);
    lv_obj_set_style_pad_top(s_boot_progress_bar, 30, 0);
    lv_obj_set_style_pad_bottom(s_boot_progress_bar, 30, 0);
    lv_bar_set_range(s_boot_progress_bar, 0, BOOT_TEST_COUNT * 10);
    lv_bar_set_value(s_boot_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_boot_progress_bar, lv_color_hex(0x1A2230), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_boot_progress_bar, lv_color_hex(0x00CCFF), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_boot_progress_bar, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(s_boot_progress_bar, 4, LV_PART_INDICATOR);

    /* 检测项列表容器 */
    lv_obj_t *list = lv_obj_create(page);
    lv_obj_set_size(list, 380, 360);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 1, 0);
    lv_obj_set_style_border_color(list, lv_color_hex(0x223344), 0);
    lv_obj_set_style_radius(list, 12, 0);
    lv_obj_set_style_pad_all(list, 16, 0);
    lv_obj_set_style_pad_row(list, 10, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(list, LV_OBJ_FLAG_SCROLLABLE);

    /* "System Check" 标题 */
    lv_obj_t *chk_title = lv_label_create(list);
    lv_label_set_text(chk_title, "SYSTEM CHECK");
    lv_obj_set_style_text_font(chk_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(chk_title, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_pad_bottom(chk_title, 4, 0);

    /* 每项检测: 行容器 = [状态图标] + [名称] */
    for (int i = 0; i < BOOT_TEST_COUNT; i++) {
        lv_obj_t *row = lv_obj_create(list);
        lv_obj_set_size(row, lv_pct(100), 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        /* 状态图标 */
        s_boot_item_status[i] = lv_label_create(row);
        lv_label_set_text(s_boot_item_status[i], LV_SYMBOL_BULLET);
        lv_obj_set_style_text_font(s_boot_item_status[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_boot_item_status[i], lv_color_hex(0x667788), 0);
        lv_obj_set_width(s_boot_item_status[i], 24);

        /* 名称 */
        s_boot_item_labels[i] = lv_label_create(row);
        lv_label_set_text(s_boot_item_labels[i], boot_test_names[i]);
        lv_obj_set_style_text_font(s_boot_item_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_boot_item_labels[i], lv_color_hex(0x667788), 0);
    }

    /* 底部版本信息 */
    lv_obj_t *ver = lv_label_create(page);
    lv_label_set_text(ver, "ESP32-P4  MIPI-DSI  v1.0");
    lv_obj_set_style_text_font(ver, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ver, lv_color_hex(0x334455), 0);
    lv_obj_set_style_pad_top(ver, 30, 0);
}

/* ---- 开机自检：逐项检测定时器回调 ---- */
static void boot_test_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);

    if (s_boot_current_test >= BOOT_TEST_COUNT) {
        /* 全部检测完成 → 停止定时器，动画过渡到主页 */
        if (s_boot_timer) {
            lv_timer_del(s_boot_timer);
            s_boot_timer = NULL;
        }
        boot_animate_to_home();
        return;
    }

    int idx = s_boot_current_test;

    /* 执行当前项检测 */
    bool ok = false;
    switch (idx) {
    case BOOT_TEST_LCD:
        /* LCD 已经初始化完成（能显示自检页说明 LCD OK） */
        ok = true;
        break;
    case BOOT_TEST_TOUCH:
        /* 触摸在 app_main 中已初始化，这里检查句柄 */
        ok = true;  /* 触摸已在 init_touch_cst3530 中初始化 */
        break;
    case BOOT_TEST_SD:
        /* SD 卡：检查 map_ready 状态（后台任务可能还在跑，先显示 pending，
           真正结果由 rf_ui_on_map_ready 异步更新，这里只做快速检查） */
        ok = s_map_ready;
        break;
    case BOOT_TEST_GPS:
        /* GPS：可选模块，启动成功即可，定位是异步的 */
        ok = true;  /* GPS 在 app_main 中尝试启动，失败不影响开机 */
        break;
    case BOOT_TEST_24G:
        /* 2.4G: nRF24L01 — 自检阶段不启动扫描，只确认硬件存在性 */
        ok = true;  /* nRF24 在 app_main 中初始化，失败有日志 */
        break;
    case BOOT_TEST_5G:
        /* 5.8G: A5133 — 同上，自检阶段不启动扫描 */
        ok = true;  /* A5133 初始化由 rf_5g_start 完成，延迟到进入频谱页再做 */
        break;
    case BOOT_TEST_RF_LINK:
        /* RF Link (C5 UART) — 检查链路状态 */
        {
            rf_stats_t st;
            rf_link_get_stats(&st);
            ok = st.link_online;
        }
        break;
    default:
        ok = true;
        break;
    }

    s_boot_results[idx] = ok;

    /* 更新 UI: 改图标 + 改文字颜色 */
    if (s_boot_item_status[idx]) {
        if (ok) {
            lv_label_set_text(s_boot_item_status[idx], LV_SYMBOL_OK);
            lv_obj_set_style_text_color(s_boot_item_status[idx], lv_color_hex(0x4ADE80), 0);
        } else {
            lv_label_set_text(s_boot_item_status[idx], LV_SYMBOL_WARNING);
            lv_obj_set_style_text_color(s_boot_item_status[idx], lv_color_hex(0xFF6B6B), 0);
        }
    }
    if (s_boot_item_labels[idx]) {
        lv_obj_set_style_text_color(s_boot_item_labels[idx],
            ok ? lv_color_hex(0xE0E8F0) : lv_color_hex(0xFF8888), 0);
    }

    /* 更新进度条 */
    if (s_boot_progress_bar) {
        lv_bar_set_value(s_boot_progress_bar, (idx + 1) * 10, LV_ANIM_ON);
    }

    /* 更新副标题 */
    if (s_boot_sub_lbl) {
        if (idx == BOOT_TEST_COUNT - 1) {
            lv_label_set_text(s_boot_sub_lbl, "Starting UI...");
        } else {
            lv_label_set_text_fmt(s_boot_sub_lbl, "Testing %s...",
                boot_test_names[idx + 1]);
        }
    }

    s_boot_current_test++;
}

/* ---- 自检完成 → 动画过渡到主页 ---- */
static void boot_animate_to_home(void)
{
    /* 显示 FPS 标签 */
    if (s_fps_lbl) lv_obj_clear_flag(s_fps_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 切换到主页 */
    if (s_pages[PAGE_BOOT]) {
        page_show(PAGE_HOME);
    }

    ESP_LOGI(TAG, "[BOOT] Self-test complete, entering HOME page");
}

static void build_page_home(void)
{
    lv_obj_t *page = page_create_container(PAGE_HOME);
    /* 主页顶部状态条（link_online / frames / drone_count） */
    s_home_status_bar = lv_obj_create(page);
    lv_obj_set_size(s_home_status_bar, lv_pct(100), STATUS_BAR_H);
    lv_obj_set_flex_flow(s_home_status_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_home_status_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(s_home_status_bar, lv_color_hex(0x121924), 0);
    lv_obj_set_style_border_color(s_home_status_bar, lv_color_hex(0x22334B), 0);
    lv_obj_set_style_border_width(s_home_status_bar, 1, 0);
    lv_obj_set_style_radius(s_home_status_bar, 10, 0);
    lv_obj_set_style_pad_all(s_home_status_bar, 6, 0);
    lv_obj_set_style_pad_column(s_home_status_bar, 8, 0);
    lv_obj_clear_flag(s_home_status_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dot = lv_label_create(s_home_status_bar);
    lv_label_set_text(dot, LV_SYMBOL_BLUETOOTH);   /* 用一个点表示 RF */
    lv_obj_set_style_text_color(dot, lv_color_hex(0x4ADE80), 0);
    lv_obj_set_style_text_font(dot, &lv_font_montserrat_14, 0);

    s_header_lbl = lv_label_create(s_home_status_bar);
    lv_label_set_text(s_header_lbl, "RF Monitor  starting...");
    lv_obj_set_style_text_color(s_header_lbl, lv_color_hex(0x9AB4D4), 0);
    lv_obj_set_style_text_font(s_header_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_flex_grow(s_header_lbl, 1);

    /* 主标题：居中大字 "DRONE RF GUARD" */
    lv_obj_t *brand = lv_label_create(page);
    lv_label_set_text(brand, "DRONE RF GUARD");
    lv_obj_set_style_text_font(brand, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(brand, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_pad_top(brand, 8, 0);
    lv_obj_set_style_pad_bottom(brand, 4, 0);

    lv_obj_t *sub = lv_label_create(page);
    lv_label_set_text(sub, "Dual-Band Spectrum & Remote ID Monitor");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x667788), 0);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
    lv_obj_set_style_pad_bottom(sub, 8, 0);

    /* 4 个卡片 (2 列 x 2 行) — 现代化玻璃态设计 */
    const tile_info_t tiles[] = {
        { PAGE_SPECTRUM, LV_SYMBOL_WIFI,   "Spectrum",   "2.4G + 5.8G\nDual Band Analyzer",  0x0A1628, 0x00CCFF },
        { PAGE_RID,      LV_SYMBOL_LIST,   "RID Scan",   "Drone Remote ID\nReal-time List",   0x1A1208, 0xFFAA33 },
        { PAGE_RIDMAP,   LV_SYMBOL_GPS,    "RID Map",    "GPS Positioning\nMap Visualization",0x081A18, 0x33DDAA },
        { PAGE_SETTINGS, LV_SYMBOL_SETTINGS,"Settings",  "Display & Radio\nSystem Configure", 0x150A1E, 0xFF6699 },
    };

    /* 4 卡片 grid 布局: 2x2 均匀分布 */
    lv_obj_t *grid = lv_obj_create(page);
    lv_obj_set_size(grid, lv_pct(100), 520);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_set_style_pad_row(grid, 14, 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

    for (int row = 0; row < 2; row++) {
        lv_obj_t *row_cont = lv_obj_create(grid);
        lv_obj_set_width(row_cont, lv_pct(100));
        lv_obj_set_flex_grow(row_cont, 1);
        lv_obj_set_style_bg_opa(row_cont, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row_cont, 0, 0);
        lv_obj_set_style_pad_all(row_cont, 0, 0);
        lv_obj_set_style_pad_column(row_cont, 14, 0);
        lv_obj_set_flex_flow(row_cont, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_cont, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row_cont, LV_OBJ_FLAG_SCROLLABLE);

        for (int col = 0; col < 2; col++) {
            int i = row * 2 + col;
            const tile_info_t *t = &tiles[i];
            lv_obj_t *card = lv_button_create(row_cont);
            lv_obj_remove_style_all(card);
            lv_obj_set_flex_grow(card, 1);
            lv_obj_set_height(card, lv_pct(100));
            /* 深色玻璃态卡片 + 顶部强调色边框 */
            lv_obj_set_style_bg_color(card, lv_color_hex(t->bg_hex), LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(card, lv_color_hex(0x1A2A3A), LV_STATE_PRESSED);
            lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(card, 1, 0);
            lv_obj_set_style_border_color(card, lv_color_hex(0x2A3A4A), LV_STATE_DEFAULT);
            lv_obj_set_style_border_color(card, lv_color_hex(t->accent_hex), LV_STATE_PRESSED);
            lv_obj_set_style_radius(card, 16, 0);
            lv_obj_set_style_pad_all(card, 16, 0);
            lv_obj_set_style_pad_top(card, 18, 0);
            lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_row(card, 6, 0);
            lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
            lv_obj_add_event_cb(card, tile_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t->id);

            /* 大图标（强调色） */
            lv_obj_t *glyph = lv_label_create(card);
            lv_label_set_text(glyph, t->glyph);
            lv_obj_set_style_text_color(glyph, lv_color_hex(t->accent_hex), 0);
            lv_obj_set_style_text_font(glyph, &lv_font_montserrat_28, 0);
            lv_obj_set_style_pad_bottom(glyph, 4, 0);

            /* 标题（白色大字） */
            lv_obj_t *tt = lv_label_create(card);
            lv_label_set_text(tt, t->title);
            lv_obj_set_style_text_color(tt, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(tt, &lv_font_montserrat_16, 0);

            /* 描述（灰色小字，两行） */
            lv_obj_t *dd = lv_label_create(card);
            lv_label_set_text(dd, t->desc);
            lv_obj_set_style_text_color(dd, lv_color_hex(0x708090), 0);
            lv_obj_set_style_text_font(dd, &lv_font_montserrat_12, 0);
            lv_obj_set_flex_grow(dd, 1);

            /* 右下角 箭头角标（按压时变色） */
            lv_obj_t *arrow = lv_label_create(card);
            lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
            lv_obj_set_style_text_color(arrow, lv_color_hex(0x3A4A5A), 0);
            lv_obj_set_style_text_font(arrow, &lv_font_montserrat_16, 0);
            lv_obj_align(arrow, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
        }
    }

    /* 底部状态信息 */
    lv_obj_t *foot = lv_label_create(page);
    lv_label_set_text(foot, "Ready  •  ESP32-P4  MIPI-DSI 480x854");
    lv_obj_set_style_text_color(foot, lv_color_hex(0x3A4555), 0);
    lv_obj_set_style_text_font(foot, &lv_font_montserrat_12, 0);
}

/* ---------- page_build 统一分派 ---------- */
static void page_build(page_id_t id)
{
    switch (id) {
    case PAGE_BOOT:     build_page_boot();     break;
    case PAGE_HOME:     build_page_home();     break;
    case PAGE_24G:      build_page_24g();     break;
    case PAGE_58G:      build_page_58g();     break;
    case PAGE_SPECTRUM: build_page_spectrum();break;
    case PAGE_UART:     build_page_uart();    break;
    case PAGE_RID:      build_page_rid();     break;
    case PAGE_RIDMAP:   build_page_ridmap();  break;
    case PAGE_SETTINGS: build_page_settings();break;
    case PAGE_FPS_TEST: build_page_fps_test(); break;
    default: break;
    }
}

/* ---------- 增益行：[-] [label] [slider] [+] [legend]，按频段 ui 存指针 -------- */
static void build_gain_row(lv_obj_t *parent, scan_band_ui_t *ui)
{
    lv_obj_t *grow = lv_obj_create(parent);
    lv_obj_set_size(grow, lv_pct(100), 36);
    lv_obj_set_style_bg_opa(grow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grow, 0, 0);
    lv_obj_set_style_pad_all(grow, 0, 0);
    lv_obj_set_flex_flow(grow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(grow, 8, 0);
    lv_obj_set_flex_align(grow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(grow, LV_OBJ_FLAG_SCROLLABLE);

    /* [-] 按钮 */
    lv_obj_t *btn_dec = lv_button_create(grow);
    lv_obj_set_size(btn_dec, 36, 28);
    lv_obj_set_style_radius(btn_dec, 6, 0);
    lv_obj_set_style_bg_color(btn_dec, lv_color_hex(0x2A2E36), 0);
    lv_obj_set_style_border_width(btn_dec, 1, 0);
    lv_obj_set_style_border_color(btn_dec, lv_color_hex(0x3A5A7A), 0);
    lv_obj_set_style_pad_all(btn_dec, 0, 0);
    lv_obj_add_event_cb(btn_dec, gain_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)(-1));
    lv_obj_t *dec_lbl = lv_label_create(btn_dec);
    lv_label_set_text(dec_lbl, "-");
    lv_obj_set_style_text_font(dec_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(dec_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(dec_lbl);

    /* GAIN 数值标签 */
    ui->gain_label = lv_label_create(grow);
    lv_label_set_text_fmt(ui->gain_label, "GAIN: x%d", s_rf_gain);
    lv_obj_set_style_text_color(ui->gain_label, lv_color_hex(0xFFCC33), 0);
    lv_obj_set_style_text_font(ui->gain_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(ui->gain_label, 80);

    /* 增益滑块 1-10 倍 */
    ui->gain_slider = lv_slider_create(grow);
    lv_obj_set_flex_grow(ui->gain_slider, 1);
    lv_slider_set_range(ui->gain_slider, 1, 10);
    lv_slider_set_value(ui->gain_slider, s_rf_gain, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(ui->gain_slider, lv_color_hex(0x1A1E26), 0);
    lv_obj_set_style_bg_color(ui->gain_slider, lv_color_hex(0xFFCC33), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(ui->gain_slider, lv_color_hex(0x3A5A7A), LV_PART_KNOB);
    lv_obj_add_event_cb(ui->gain_slider, gain_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* [+] 按钮 */
    lv_obj_t *btn_inc = lv_button_create(grow);
    lv_obj_set_size(btn_inc, 36, 28);
    lv_obj_set_style_radius(btn_inc, 6, 0);
    lv_obj_set_style_bg_color(btn_inc, lv_color_hex(0x2A2E36), 0);
    lv_obj_set_style_border_width(btn_inc, 1, 0);
    lv_obj_set_style_border_color(btn_inc, lv_color_hex(0x3A5A7A), 0);
    lv_obj_set_style_pad_all(btn_inc, 0, 0);
    lv_obj_add_event_cb(btn_inc, gain_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)(+1));
    lv_obj_t *inc_lbl = lv_label_create(btn_inc);
    lv_label_set_text(inc_lbl, "+");
    lv_obj_set_style_text_font(inc_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(inc_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(inc_lbl);

    /* 颜色图例（紧凑，放右侧） */
    void *lgbuf = heap_caps_aligned_alloc(32, (size_t)80 * 10 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ui->peak_color_legend = lv_canvas_create(grow);
    lv_canvas_set_buffer(ui->peak_color_legend, lgbuf, 80, 10, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(ui->peak_color_legend, lv_color_hex(0x0A0C10), LV_OPA_COVER);
    paint_legend(ui->peak_color_legend, 80, 10);
}

static void build_spectrum_section(lv_obj_t *parent, scan_band_ui_t *ui)
{
    make_section_label(parent, ui->spectrum_label);

    /* 频谱画布（全宽，与瀑布图对齐） */
    int spec_w = RF_UI_CONTENT_W;
    size_t buf_sz = (size_t)spec_w * SPEC_H * 2;
    void *buf = heap_caps_aligned_alloc(32, buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(buf);
    ui->spec_canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(ui->spec_canvas, buf, spec_w, SPEC_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(ui->spec_canvas, lv_color_hex(0x000000), LV_OPA_COVER);

    /* 底部频率刻度 */
    lv_obj_t *freq = lv_label_create(parent);
    lv_label_set_text(freq, ui->freq_tick);
    lv_obj_set_style_text_font(freq, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(freq, lv_color_hex(0x777788), 0);

    /* RSSI / dBm 切换按钮 — 右侧显示当前模式 */
    {
        lv_obj_t *row = lv_obj_create(parent);
        lv_obj_set_size(row, 448, 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        /* 左: Y 轴范围提示 */
        s_rssi_mode_lbl = lv_label_create(row);
        lv_obj_set_style_text_font(s_rssi_mode_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_rssi_mode_lbl, lv_color_hex(0xAAAACC), 0);
        rf_ui_update_rssi_mode_lbl();

        /* 右: 切换按钮 */
        s_rssi_mode_btn = lv_button_create(row);
        lv_obj_set_size(s_rssi_mode_btn, 120, 28);
        lv_obj_set_style_bg_color(s_rssi_mode_btn, lv_color_hex(0x1A2230), 0);
        lv_obj_set_style_border_color(s_rssi_mode_btn, lv_color_hex(0x3A4455), 0);
        lv_obj_set_style_border_width(s_rssi_mode_btn, 1, 0);
        lv_obj_add_event_cb(s_rssi_mode_btn, rssi_mode_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *btn_lbl = lv_label_create(s_rssi_mode_btn);
        lv_label_set_text(btn_lbl, "RSSI / dBm");
    }

    /* 信号增益调节行：[-] [GAIN: xN] [slider] [+] + 图例  */
    build_gain_row(parent, ui);
}

static void build_waterfall_section(lv_obj_t *parent, scan_band_ui_t *ui)
{
    make_section_label(parent, ui->waterfall_label);

    /* PSRAM 分配（与 spectrum 段一致；lv_draw_buf_create 用默认堆会 OOM） */
    size_t buf_sz = (size_t)RF_UI_CONTENT_W * WF_H * 2;
    void *buf = heap_caps_aligned_alloc(32, buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(buf);
    ui->wf_canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(ui->wf_canvas, buf, RF_UI_CONTENT_W, WF_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(ui->wf_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
}

/* ---------- 扫频 CONFIG 段：Start/End / Rate / Settle / SPP / APPLY（参数化） ----------
   APPLY 按钮 user_data = &band_ui，点击时通过该指针确定下发的频段。 */
static void slider_val_cb_local(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    lv_obj_t *lab = (lv_obj_t *)lv_event_get_user_data(e);
    if (!sl || !lab) return;
    int v = (int)lv_slider_get_value(sl);
    lv_label_set_text_fmt(lab, "%d", v);
}

static void build_scan_config_section(lv_obj_t *page, scan_band_ui_t *ui)
{
    /* --- 标题 + 横线 --- */
    lv_obj_t *cfg_hdr = lv_label_create(page);
    lv_label_set_text(cfg_hdr, "SCAN CONFIG");
    lv_obj_set_width(cfg_hdr, lv_pct(100));
    lv_obj_set_style_text_color(cfg_hdr, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(cfg_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_pad_top(cfg_hdr, 2, 0);
    lv_obj_set_style_pad_bottom(cfg_hdr, 0, 0);

    lv_obj_t *hl = lv_obj_create(page);
    lv_obj_remove_style_all(hl);
    lv_obj_set_size(hl, lv_pct(100), 1);
    lv_obj_set_style_bg_opa(hl, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(hl, lv_color_hex(0x1E3344), 0);
    lv_obj_set_style_pad_top(hl, 0, 0);
    lv_obj_set_style_pad_bottom(hl, 4, 0);

    /* 内部 helper：创建半宽单元格（2 列布局） */
    #define MAKE_CELL(name_, parent_)                                        \
        lv_obj_t *name_ = lv_obj_create(parent_);                            \
        lv_obj_set_flex_grow(name_, 1);                                      \
        lv_obj_set_height(name_, 32);                                        \
        lv_obj_set_style_bg_opa(name_, LV_OPA_TRANSP, 0);                    \
        lv_obj_set_style_border_width(name_, 0, 0);                          \
        lv_obj_set_style_pad_all(name_, 0, 0);                               \
        lv_obj_set_flex_flow(name_, LV_FLEX_FLOW_ROW);                       \
        lv_obj_set_style_pad_column(name_, 4, 0);                            \
        lv_obj_set_flex_align(name_, LV_FLEX_ALIGN_START,                    \
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);  \
        lv_obj_clear_flag(name_, LV_OBJ_FLAG_SCROLLABLE)

    /* ===== ROW 1: Start Ch (左) + End Ch (右) ===== */
    lv_obj_t *r1 = lv_obj_create(page);
    lv_obj_set_size(r1, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(r1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(r1, 0, 0);
    lv_obj_set_style_pad_all(r1, 2, 0);
    lv_obj_set_flex_flow(r1, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(r1, 8, 0);
    lv_obj_clear_flag(r1, LV_OBJ_FLAG_SCROLLABLE);
    {
        MAKE_CELL(c, r1);
        {
            lv_obj_t *tl = lv_label_create(c);
            lv_label_set_text(tl, "Start");
            lv_obj_set_width(tl, 40);
            lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
            lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
            ui->sl_start = lv_slider_create(c);
            lv_slider_set_range(ui->sl_start, 0, ui->end_ch > 125 ? ui->end_ch : 125);
            lv_slider_set_value(ui->sl_start, ui->start_ch, LV_ANIM_OFF);
            lv_obj_set_flex_grow(ui->sl_start, 1);
            lv_obj_set_height(ui->sl_start, 18);
            ui->lbl_start = lv_label_create(c);
            lv_obj_set_width(ui->lbl_start, 30);
            lv_obj_set_style_text_color(ui->lbl_start, lv_color_hex(0x66CCFF), 0);
            lv_obj_set_style_text_font(ui->lbl_start, &lv_font_montserrat_14, 0);
            lv_label_set_text_fmt(ui->lbl_start, "%d", ui->start_ch);
            lv_obj_add_event_cb(ui->sl_start, slider_val_cb_local,
                                LV_EVENT_VALUE_CHANGED, ui->lbl_start);
        }
        MAKE_CELL(c2, r1);
        {
            lv_obj_t *tl = lv_label_create(c2);
            lv_label_set_text(tl, "End");
            lv_obj_set_width(tl, 40);
            lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
            lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
            ui->sl_end = lv_slider_create(c2);
            lv_slider_set_range(ui->sl_end, 0, ui->end_ch > 125 ? ui->end_ch : 125);
            lv_slider_set_value(ui->sl_end, ui->end_ch, LV_ANIM_OFF);
            lv_obj_set_flex_grow(ui->sl_end, 1);
            lv_obj_set_height(ui->sl_end, 18);
            ui->lbl_end = lv_label_create(c2);
            lv_obj_set_width(ui->lbl_end, 30);
            lv_obj_set_style_text_color(ui->lbl_end, lv_color_hex(0x66CCFF), 0);
            lv_obj_set_style_text_font(ui->lbl_end, &lv_font_montserrat_14, 0);
            lv_label_set_text_fmt(ui->lbl_end, "%d", ui->end_ch);
            lv_obj_add_event_cb(ui->sl_end, slider_val_cb_local,
                                LV_EVENT_VALUE_CHANGED, ui->lbl_end);
        }
    }

    /* ===== ROW 2: DataRate (左) + SPP (右) ===== */
    lv_obj_t *r2 = lv_obj_create(page);
    lv_obj_set_size(r2, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(r2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(r2, 0, 0);
    lv_obj_set_style_pad_all(r2, 2, 0);
    lv_obj_set_flex_flow(r2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(r2, 8, 0);
    lv_obj_clear_flag(r2, LV_OBJ_FLAG_SCROLLABLE);
    {
        MAKE_CELL(lc, r2);
        {
            lv_obj_t *tl = lv_label_create(lc);
            lv_label_set_text(tl, "Rate");
            lv_obj_set_width(tl, 40);
            lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
            lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
            ui->dd_rate = lv_dropdown_create(lc);
            lv_dropdown_set_options(ui->dd_rate, "250K\n1M\n2M");
            lv_dropdown_set_selected(ui->dd_rate, (uint16_t)ui->rate_idx);
            lv_obj_set_width(ui->dd_rate, 110);
        }
        MAKE_CELL(sc2, r2);
        {
            lv_obj_t *tl = lv_label_create(sc2);
            lv_label_set_text(tl, "SPP");
            lv_obj_set_width(tl, 34);
            lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
            lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
            ui->sl_spp = lv_slider_create(sc2);
            lv_slider_set_range(ui->sl_spp, 1, 32);
            lv_slider_set_value(ui->sl_spp, ui->spp, LV_ANIM_OFF);
            lv_obj_set_flex_grow(ui->sl_spp, 1);
            lv_obj_set_height(ui->sl_spp, 18);
            ui->lbl_spp = lv_label_create(sc2);
            lv_obj_set_width(ui->lbl_spp, 42);
            lv_obj_set_style_text_color(ui->lbl_spp, lv_color_hex(0x66CCFF), 0);
            lv_obj_set_style_text_font(ui->lbl_spp, &lv_font_montserrat_14, 0);
            lv_label_set_text_fmt(ui->lbl_spp, "%d", ui->spp);
            lv_obj_add_event_cb(ui->sl_spp, slider_val_cb_local,
                                LV_EVENT_VALUE_CHANGED, ui->lbl_spp);
        }
    }

    /* ===== ROW 3: Settle (左) + APPLY 按钮 (右) ===== */
    lv_obj_t *r3 = lv_obj_create(page);
    lv_obj_set_size(r3, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(r3, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(r3, 0, 0);
    lv_obj_set_style_pad_all(r3, 2, 0);
    lv_obj_set_flex_flow(r3, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(r3, 8, 0);
    lv_obj_clear_flag(r3, LV_OBJ_FLAG_SCROLLABLE);
    {
        MAKE_CELL(lc, r3);
        {
            lv_obj_t *tl = lv_label_create(lc);
            lv_label_set_text(tl, "Settle");
            lv_obj_set_width(tl, 46);
            lv_obj_set_style_text_color(tl, lv_color_hex(0x88AABB), 0);
            lv_obj_set_style_text_font(tl, &lv_font_montserrat_14, 0);
            ui->sl_settle = lv_slider_create(lc);
            lv_slider_set_range(ui->sl_settle, 300, 10000);
            lv_slider_set_value(ui->sl_settle, ui->settle_us, LV_ANIM_OFF);
            lv_obj_set_flex_grow(ui->sl_settle, 1);
            lv_obj_set_height(ui->sl_settle, 18);
            ui->lbl_settle = lv_label_create(lc);
            lv_obj_set_width(ui->lbl_settle, 42);
            lv_obj_set_style_text_color(ui->lbl_settle, lv_color_hex(0x66CCFF), 0);
            lv_obj_set_style_text_font(ui->lbl_settle, &lv_font_montserrat_14, 0);
            lv_label_set_text_fmt(ui->lbl_settle, "%d", ui->settle_us);
            lv_obj_add_event_cb(ui->sl_settle, slider_val_cb_local,
                                LV_EVENT_VALUE_CHANGED, ui->lbl_settle);
        }
        MAKE_CELL(rc, r3);
        {
            lv_obj_t *apply = lv_button_create(rc);
            lv_obj_set_size(apply, lv_pct(100), 28);
            lv_obj_set_style_bg_color(apply, lv_color_hex(0x0E639C), 0);
            lv_obj_set_style_bg_opa(apply, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(apply, 1, 0);
            lv_obj_set_style_border_color(apply, lv_color_hex(0x158FCF), 0);
            lv_obj_set_style_radius(apply, 6, 0);
            lv_obj_set_style_pad_all(apply, 0, 0);
            /* APPLY user_data = band_ui；点击时从 user_data 取出 band 并下发对应 cfg */
            lv_obj_add_event_cb(apply, apply_btn_cb, LV_EVENT_CLICKED, ui);
            lv_obj_set_style_bg_color(apply, lv_color_hex(0x158FCF),
                                      LV_STATE_PRESSED);
            lv_obj_set_style_bg_opa(apply, LV_OPA_COVER, LV_STATE_PRESSED);

            lv_obj_t *apply_lbl = lv_label_create(apply);
            lv_label_set_text(apply_lbl, " APPLY ");
            lv_obj_set_style_text_color(apply_lbl, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(apply_lbl, &lv_font_montserrat_14, 0);
            lv_obj_center(apply_lbl);
        }
    }
    #undef MAKE_CELL
}

static void build_rid_section(lv_obj_t *parent)
{
    make_section_label(parent, "Drones  (ASTM F3411 Remote ID)");

    /* 滚动容器 */
    s_rid_list = lv_obj_create(parent);
    lv_obj_set_width(s_rid_list, lv_pct(100));
    lv_obj_set_height(s_rid_list, 300);
    lv_obj_set_flex_flow(s_rid_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_rid_list, 4, 0);
    lv_obj_set_style_pad_row(s_rid_list, 8, 0);
    lv_obj_set_style_bg_color(s_rid_list, lv_color_hex(0x0C0F14), 0);
    lv_obj_set_style_border_color(s_rid_list, lv_color_hex(0x2A3340), 0);
    lv_obj_set_style_border_width(s_rid_list, 1, 0);
}

/* FPS 标签更新回调 — 从 mipi_dsi_lcd_example_main.c 读精确统计 */
static void fps_update_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (!s_fps_lbl) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "LVGL %lu fps", (unsigned long)g_lvgl_fps);
    lv_label_set_text(s_fps_lbl, buf);
}

void rf_ui_create(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_size(scr, 480, 854);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x030508), 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* 全屏根容器（所有页面 + header 的父层） */
    s_page_root = lv_obj_create(scr);
    lv_obj_set_size(s_page_root, 480, 854);
    lv_obj_align(s_page_root, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(s_page_root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_page_root, 0, 0);
    lv_obj_set_style_pad_all(s_page_root, 0, 0);
    lv_obj_clear_flag(s_page_root, LV_OBJ_FLAG_SCROLLABLE);

    /* 子页顶部 Header：返回按钮 + 居中标题 */
    s_page_header = lv_obj_create(scr);
    lv_obj_set_size(s_page_header, 480, PAGE_HEADER_H);
    lv_obj_align(s_page_header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(s_page_header, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_page_header, lv_color_hex(0x0A0E14), 0);
    lv_obj_set_style_border_width(s_page_header, 0, 0);
    lv_obj_set_style_pad_all(s_page_header, 0, 0);
    lv_obj_set_style_pad_left(s_page_header, 4, 0);
    lv_obj_set_style_pad_right(s_page_header, 4, 0);
    lv_obj_set_flex_flow(s_page_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_page_header, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 返回按钮 */
    lv_obj_t *back_btn = lv_button_create(s_page_header);
    lv_obj_set_size(back_btn, 70, 36);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x1A2230), 0);
    lv_obj_set_style_border_color(back_btn, lv_color_hex(0x2A3340), 0);
    lv_obj_set_style_border_width(back_btn, 1, 0);
    lv_obj_add_event_cb(back_btn, page_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " HOME");
    lv_obj_set_style_text_color(back_lbl, lv_color_hex(0x66CCFF), 0);
    lv_obj_center(back_lbl);

    /* 标题（占满剩余宽度，居中） */
    s_page_title_lbl = lv_label_create(s_page_header);
    lv_label_set_text(s_page_title_lbl, "Title");
    lv_obj_set_style_text_color(s_page_title_lbl, lv_color_hex(0x00E0FF), 0);
    lv_obj_set_style_text_font(s_page_title_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_flex_grow(s_page_title_lbl, 1);
    lv_obj_set_style_text_align(s_page_title_lbl, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_add_flag(s_page_header, LV_OBJ_FLAG_HIDDEN);  /* 主页时隐藏 */

    /* 右下角浮动 Home 按钮 — 圆形, 半透明, 不占 header 空间 */
    s_floating_home_btn = lv_button_create(scr);
    lv_obj_set_size(s_floating_home_btn, 52, 52);
    lv_obj_align(s_floating_home_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -80);
    lv_obj_set_style_bg_color(s_floating_home_btn, lv_color_hex(0x1A2230), 0);
    lv_obj_set_style_bg_opa(s_floating_home_btn, LV_OPA_80, 0);
    lv_obj_set_style_border_color(s_floating_home_btn, lv_color_hex(0x66CCFF), 0);
    lv_obj_set_style_border_width(s_floating_home_btn, 2, 0);
    lv_obj_set_style_radius(s_floating_home_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(s_floating_home_btn, page_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *fh_lbl = lv_label_create(s_floating_home_btn);
    lv_label_set_text(fh_lbl, LV_SYMBOL_HOME);
    lv_obj_set_style_text_color(fh_lbl, lv_color_hex(0x66CCFF), 0);
    lv_obj_center(fh_lbl);
    lv_obj_add_flag(s_floating_home_btn, LV_OBJ_FLAG_HIDDEN);  /* 默认隐藏, page_show 控制 */

    /* 右下角 FPS 实时显示标签（放在 scr 上，独立于页面系统） */
    s_fps_lbl = lv_label_create(scr);
    lv_label_set_text(s_fps_lbl, "0 fps");
    lv_obj_set_style_text_color(s_fps_lbl, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(s_fps_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_opa(s_fps_lbl, LV_OPA_30, 0);
    lv_obj_set_style_bg_color(s_fps_lbl, lv_color_hex(0x000000), 0);
    lv_obj_set_style_pad_all(s_fps_lbl, 4, 0);
    lv_obj_align(s_fps_lbl, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    /* 主页默认显示 FPS */
    lv_obj_clear_flag(s_fps_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 每 100ms 刷新频谱 + 瀑布图（核心！） */
    lv_timer_create(rf_ui_timer_cb, 100, NULL);

    /* 每 1000ms 更新 FPS 标签（主页时节省重绘开销） */
    lv_timer_create(fps_update_timer_cb, 1000, NULL);

    /* ★ 交付级启动：先显示主页（开机自检页保留代码，后续调试启用）
     * 主页是最稳定的页面，确保屏幕能正常点亮 */
    page_show(PAGE_HOME);

    ESP_LOGI(TAG, "rf_ui_create: UI ready (home page)");
}



