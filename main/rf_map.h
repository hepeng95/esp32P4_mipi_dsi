/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * rf_map — Offline tile map + geo → canvas projection
 *
 * Hard constraints from project memory:
 *   - ESP32-P4 has SDMMC host + GPIO matrix + 4-bit SDMMC (fastest for tile PNGs)
 *   - Tile path on SD card: /sdcard/tiles/{zoom}/{x}/{y}.png   (OSM Web Mercator)
 *   - Tile size: 256x256 px, zoom 17 = ~1.2 m/px (street level for typical DJI RID ops)
 *   - SD pins (4-bit SDMMC): D0=49, D1=48, D2=53, D3=52, CMD=51, CLK=50
 *     GPIO8 is free (user may repurpose for WP/CD or second card).
 *   - Canvas in RID MAP page: 448x280 RGB888
 *
 * Usage:
 *   rf_map_init()                 — mount SD card + load 1 tile centered at rf_map_set_center()
 *   rf_map_set_center(lat, lon, z)— switch center tile (default zoom=17, first call required)
 *   rf_map_project(lat, lon, &cx, &cy) — drone lat/lon → canvas pixel (returns false if out-of-tile)
 *   rf_map_canvas_overlay()       — returns lv_img_dsc_t* for the center tile (draw_img to canvas)
 */
#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- SD pinout (4-bit SDMMC mode) --- */
#define RF_MAP_SD_CLK     50
#define RF_MAP_SD_CMD     51
#define RF_MAP_SD_D0      49
#define RF_MAP_SD_D1      48
#define RF_MAP_SD_D2      53
#define RF_MAP_SD_D3      52
#define RF_MAP_SD_MOUNT   "/sdcard"
#define RF_MAP_TILE_DIR   RF_MAP_SD_MOUNT "/MAP/TILES"

/* Default center — SD 卡瓦片实际覆盖区域 (湖南省岳阳市岳阳县东部)
   lat=29.085, lon=113.236, Web Mercator zoom=16
   XYZ tile_x=53381, tile_y=27229 → /sdcard/MAP/TILES/16/27229/53381.PNG */
#define RF_MAP_DEFAULT_CENTER_LAT   29.085
#define RF_MAP_DEFAULT_CENTER_LON   113.236
#define RF_MAP_DEFAULT_ZOOM         16

/* Single tile we keep in RAM (PSRAM). 256x256 PNG at zoom 17 = ~100KB on disk → ~192KB raw RGB888 in PSRAM. */
#define RF_MAP_TILE_PX  256

/* Canvas slot (the center tile is drawn here). Caller sets these when building RID MAP page. */
typedef struct {
    lv_obj_t *canvas;            /* 地图 canvas */
    int       canvas_w;          /* canvas 像素宽 (448) */
    int       canvas_h;          /* canvas 像素高 (280) */
    int       tile_slot_x;       /* 当前瓦片 paste 位置 (pan 时动态偏移) */
    int       tile_slot_y;
    int       base_slot_x;       /* 居中基准 (canvas_w - 256)/2 */
    int       base_slot_y;
    double    center_lat;        /* deg */
    double    center_lon;        /* deg */
    int       zoom;              /* web-mercator tile zoom */
    int       tile_x;            /* current center tile x */
    int       tile_y;            /* current center tile y */
    int       pending_pan_dx;    /* 累积拖动偏移 (px), 超过 128 触发重渲染 */
    int       pending_pan_dy;
} rf_map_view_t;

esp_err_t rf_map_init(void);
void      rf_map_deinit(void);
bool      rf_map_is_mounted(void);

/* ★ 退出 RID MAP 页: flush cache (省 PSRAM) + 清 canvas + 停 pending 请求 */
void      rf_map_exit_page(rf_map_view_t *view);

/* Load/switch center tile. Sets view->tile_x, tile_y and tile_img (cached). */
esp_err_t rf_map_set_center(rf_map_view_t *view, double lat, double lon, int zoom);

/* Render multi-tile to fully cover canvas (pan/zoom 后重绘). */
esp_err_t rf_map_render_multi(rf_map_view_t *view);

/* 边缘瓦片补全 (不清背景, 只 paste 瓦片覆盖边缘露出的黑区). 在线地图式 */
esp_err_t rf_map_render_edges(rf_map_view_t *view);

/* ★ 异步边缘补全请求 — 不阻塞调用方 (触摸回调里用这个!) */
void rf_map_request_edges(rf_map_view_t *view);

/* ★ 异步全量重绘请求 — 清除所有旧点 (无人机位置变化时用这个!) */
void rf_map_request_redraw(rf_map_view_t *view);

/* 快速检查瓦片是否存在 (不读内容, 用于选择中心时防空白) */
bool rf_map_tile_exists(int zoom, int tx, int ty);

/* Web Mercator 投影工具 (公开, reset/zoom 里校验瓦片用) */
double rf_map_lon_to_tile_x(double lon_deg, int zoom);
double rf_map_lat_to_tile_y(double lat_deg, int zoom);

/* 触摸拖动: 更新 tile_slot 偏移 + 重算 center_lat/lon, 不立即重读 SD */
void rf_map_pan(rf_map_view_t *view, int dx_px, int dy_px);

/* 缩放: zoom += delta, 范围 [12, 19], 立即重渲染 */
void rf_map_zoom(rf_map_view_t *view, int delta);

/* Project a lat/lon onto the canvas slot. Returns false if point falls outside the tile. */
bool rf_map_project(const rf_map_view_t *view, double lat, double lon,
                    int *canvas_x_out, int *canvas_y_out);

/* Convenience: convert 1e-7 degree fixed point from rf_drone_t directly. */
static inline bool rf_map_project_e7(const rf_map_view_t *view,
                                      int32_t lat_e7, int32_t lon_e7,
                                      int *cx, int *cy)
{
    return rf_map_project(view, lat_e7 * 1e-7, lon_e7 * 1e-7, cx, cy);
}

/* 返回 canvas 写入互斥锁 — LVGL timer 画 GPS/无人机圆点时用 */
SemaphoreHandle_t rf_map_canvas_mutex(void);

#ifdef __cplusplus
}
#endif
