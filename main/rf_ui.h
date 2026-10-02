/*
 * rf_ui.h — 多页 RF 监测 UI（竖屏 480x854）
 *
 * 页面结构：
 *   - HOME         主页（2 列 x 3 行，6 个功能入口卡片）
 *   - PAGE_24G     2.4G 扫描（频谱 + 瀑布图 + 增益）
 *   - PAGE_58G     5.8G 扫描（占位，接入数据后可显示）
 *   - PAGE_UART    UART 调试（Serial Monitor + 过滤）
 *   - PAGE_RID     RID 扫描列表（无人机卡片 + 状态）
 *   - PAGE_RIDMAP  RID 地图（后续扩展 GPS / 瓦片地图，当前显示坐标列表）
 *   - PAGE_SET     设置（显示 / 扫描参数 / UART / 控制 / RAW 等）
 *
 * 依赖：LVGL 9.x + rf_link 数据模型。
 * 刷新：单 lv_timer（250ms）运行在 LVGL task 上下文，不另起任务。
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 构建并显示多页 RF UI。必须在 LVGL 初始化后调用。
 * @param disp 已创建的 LVGL display
 */
void rf_ui_create(lv_display_t *disp);

#ifdef __cplusplus
}
#endif
