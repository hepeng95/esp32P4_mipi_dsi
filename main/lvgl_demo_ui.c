/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"
#include "esp_log.h"

// 多功能插件演示 UI（无触摸，自动动画演示各类 LVGL 控件）：
//   - Tabview 分页：Widgets / Chart / Meter
//   - Widgets 页：标签、按钮、开关、复选框、进度条、滑块
//   - Chart 页：折线图，数据由定时器自动滚动刷新
//   - Meter 页：圆形仪表，指针由定时器自动扫动
//   通过 lv_timer 周期性更新数值，即使没有触摸也能看到界面“动起来”。

static const char *TAG = "demo_ui";

static const lv_font_t *font_normal = &lv_font_montserrat_16;
static const lv_font_t *font_large  = &lv_font_montserrat_24;

// 需要被定时器动态更新的对象
static lv_obj_t *s_bar        = NULL;   // 进度条
static lv_obj_t *s_slider     = NULL;   // 滑块
static lv_obj_t *s_chart      = NULL;   // 折线图
static lv_chart_series_t *s_ser = NULL; // 折线图数据序列
static lv_obj_t *s_scale      = NULL;   // 刻度（仪表外观）
static lv_obj_t *s_needle     = NULL;   // 仪表指针（用线模拟）
static lv_obj_t *s_arc        = NULL;   // 圆弧仪表
static lv_obj_t *s_status     = NULL;   // 底部状态标签

static int32_t s_tick = 0;

/* ---- 定时器：自动刷新各控件数值 ------------------------------------------- */

static void demo_anim_timer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    s_tick++;

    // 进度条 0~100 循环
    int32_t bar_val = (s_tick * 2) % 100;
    if (s_bar) {
        lv_bar_set_value(s_bar, bar_val, LV_ANIM_ON);
    }

    // 滑块正弦摆动
    int32_t sv = 50 + (lv_trigo_sin((int16_t)(s_tick * 4)) * 45 / 32767);
    if (s_slider) {
        lv_slider_set_value(s_slider, sv, LV_ANIM_ON);
    }

    // 圆弧仪表 0~100 正弦摆动
    if (s_arc) {
        int32_t av = 50 + (lv_trigo_sin((int16_t)(s_tick * 6 + 4000)) * 50 / 32767);
        lv_arc_set_value(s_arc, av);
    }

    // 折线图滚动追加新点（正弦 + 偏置）
    if (s_chart && s_ser) {
        int32_t y = 50 + (lv_trigo_sin((int16_t)(s_tick * 20)) * 45 / 32767);
        lv_chart_set_next_value(s_chart, s_ser, y);
    }

    if (s_status) {
        lv_label_set_text_fmt(s_status, "Auto demo running... t=%d  bar=%d%%", (int)s_tick, (int)bar_val);
    }
}

/* ---- 各分页构建 ----------------------------------------------------------- */

static void build_widgets_tab(lv_obj_t *tab)
{
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab, 10, 0);

    lv_obj_t *title = lv_label_create(tab);
    lv_label_set_text(title, "Widgets");
    lv_obj_set_style_text_font(title, font_large, 0);

    // 按钮
    lv_obj_t *btn = lv_button_create(tab);
    lv_obj_set_width(btn, LV_PCT(80));
    lv_obj_t *btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "Button");
    lv_obj_center(btn_lbl);

    // 开关
    lv_obj_t *sw = lv_switch_create(tab);
    lv_obj_add_state(sw, LV_STATE_CHECKED);

    // 复选框
    lv_obj_t *cb = lv_checkbox_create(tab);
    lv_checkbox_set_text(cb, "Enable feature");
    lv_obj_add_state(cb, LV_STATE_CHECKED);

    // 进度条
    s_bar = lv_bar_create(tab);
    lv_obj_set_size(s_bar, LV_PCT(80), 16);
    lv_bar_set_range(s_bar, 0, 100);
    lv_bar_set_value(s_bar, 30, LV_ANIM_OFF);

    // 滑块
    s_slider = lv_slider_create(tab);
    lv_obj_set_width(s_slider, LV_PCT(80));
    lv_slider_set_range(s_slider, 0, 100);
    lv_slider_set_value(s_slider, 50, LV_ANIM_OFF);

    // 旋转加载动画
    lv_obj_t *spinner = lv_spinner_create(tab);
    lv_obj_set_size(spinner, 60, 60);
    lv_spinner_set_anim_params(spinner, 1000, 200);
}

static void build_chart_tab(lv_obj_t *tab)
{
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *title = lv_label_create(tab);
    lv_label_set_text(title, "Chart");
    lv_obj_set_style_text_font(title, font_large, 0);

    s_chart = lv_chart_create(tab);
    lv_obj_set_size(s_chart, LV_PCT(90), 260);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_point_count(s_chart, 30);
    lv_chart_set_update_mode(s_chart, LV_CHART_UPDATE_MODE_SHIFT);
    s_ser = lv_chart_add_series(s_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);

    // 初始填充
    for (int i = 0; i < 30; i++) {
        lv_chart_set_next_value(s_chart, s_ser, 50);
    }
}

static void build_meter_tab(lv_obj_t *tab)
{
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab, 16, 0);

    lv_obj_t *title = lv_label_create(tab);
    lv_label_set_text(title, "Meter");
    lv_obj_set_style_text_font(title, font_large, 0);

    // 用圆弧 arc 作为仪表（LVGL v9 常用，无需额外 meter 组件）
    s_arc = lv_arc_create(tab);
    lv_obj_set_size(s_arc, 220, 220);
    lv_arc_set_rotation(s_arc, 135);
    lv_arc_set_bg_angles(s_arc, 0, 270);
    lv_arc_set_range(s_arc, 0, 100);
    lv_arc_set_value(s_arc, 50);
    lv_obj_remove_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);  // 纯展示，不接受输入

    lv_obj_t *hint = lv_label_create(tab);
    lv_label_set_text(hint, "Auto sweeping gauge");
    lv_obj_set_style_text_font(hint, font_normal, 0);
}

/* ---- 入口 ----------------------------------------------------------------- */

void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                          LV_THEME_DEFAULT_DARK, font_normal);

    lv_obj_t *scr = lv_display_get_screen_active(disp);

    // Tabview（顶部标签栏）
    lv_obj_t *tv = lv_tabview_create(scr);
    lv_tabview_set_tab_bar_size(tv, 48);
    lv_obj_set_size(tv, LV_PCT(100), LV_PCT(100));

    lv_obj_t *tab1 = lv_tabview_add_tab(tv, "Widgets");
    lv_obj_t *tab2 = lv_tabview_add_tab(tv, "Chart");
    lv_obj_t *tab3 = lv_tabview_add_tab(tv, "Meter");

    build_widgets_tab(tab1);
    build_chart_tab(tab2);
    build_meter_tab(tab3);

    // 底部状态条（叠在最上层）
    s_status = lv_label_create(scr);
    lv_label_set_text(s_status, "Auto demo running...");
    lv_obj_set_style_text_color(s_status, lv_color_hex(0x88DD88), 0);
    lv_obj_set_style_text_font(s_status, font_normal, 0);
    lv_obj_align(s_status, LV_ALIGN_BOTTOM_MID, 0, -4);

    // 定时器：每 200ms 自动刷新数值，让界面在无触摸下也“动起来”
    lv_timer_create(demo_anim_timer_cb, 200, NULL);

    ESP_LOGI(TAG, "Multi-widget demo UI created (tabview + auto animation)");
}
