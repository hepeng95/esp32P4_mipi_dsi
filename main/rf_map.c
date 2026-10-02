/*
 * rf_map.c — 极简同步版 v3
 *
 * 设计原则:
 *   1. 同步渲染 (不搞异步 task, 逻辑清晰不迷路)
 *   2. 简单数据模型: tile_x/y = 中心瓦片, tile_slot_x/y = 瓦片左上角在 canvas 上的位置
 *   3. pan 实时 memmove + slot 偏移 + shift 进位
 *   4. SD 卡 SPI3_HOST (避免和 NRF24L01 SPI2_HOST 冲突)
 *   5. LRU tile cache (8 槽)
 *
 * 初始化链:
 *   rf_map_init() → mount SD (SPI3_HOST) → cache init → tile_exists check
 *   rf_map_set_center() → 算 tile_x/y/slot → sync render 3x3 grid
 *
 * 拖动链:
 *   rf_map_pan(dx,dy) → memmove canvas → slot += dx,dy → shift 检查
 *   松手 → rf_map_render_edges() → sync re-render 3x3
 */

#include "rf_map.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#include "stb_image.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "rf_map";

/* ====== 全局状态 ====== */
static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static SemaphoreHandle_t s_mut = NULL;

/* Tile Cache */
#define TC_SIZE 8
typedef struct {
    int zoom, tx, ty;
    int w, h;
    uint16_t *rgb;
    uint32_t age;
    bool used;
} tc_t;
static tc_t s_tc[TC_SIZE];
static uint32_t s_tc_age = 0;

/* ====== Web Mercator ====== */
static inline double d2r(double d) { return d * M_PI / 180.0; }
double rf_map_lon_to_tile_x(double lon, int z) {
    return (lon + 180.0) / 360.0 * (double)(1 << z);
}
double rf_map_lat_to_tile_y(double lat, int z) {
    if (lat >  85.051129) lat =  85.051129;
    if (lat < -85.051129) lat = -85.051129;
    double t = tan(d2r(lat)) + 1.0 / cos(d2r(lat));
    return (1.0 - log(t) / M_PI) / 2.0 * (double)(1 << z);
}
static void make_path(int z, int tx, int ty, char *out, size_t sz) {
    snprintf(out, sz, RF_MAP_TILE_DIR "/%d/%d/%d.PNG", z, ty, tx);
}
bool rf_map_tile_exists(int z, int tx, int ty) {
    char p[256]; make_path(z, tx, ty, p, sizeof(p));
    struct stat st; return stat(p, &st) == 0 && st.st_size > 0;
}

/* ====== 文件读取 ====== */
static uint8_t *read_psram(const char *path, size_t *osz) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 2000000) { fclose(f); return NULL; }
    uint8_t *b = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    if (!b) { fclose(f); return NULL; }
    size_t got = fread(b, 1, sz, f); fclose(f);
    if (got != (size_t)sz) { heap_caps_free(b); return NULL; }
    *osz = sz; return b;
}

/* ====== Cache ====== */
static int tc_lookup(int z, int tx, int ty) {
    for (int i = 0; i < TC_SIZE; i++)
        if (s_tc[i].used && s_tc[i].zoom==z && s_tc[i].tx==tx && s_tc[i].ty==ty) {
            s_tc[i].age = ++s_tc_age; return i;
        }
    return -1;
}
static void tc_insert(int z, int tx, int ty, uint16_t *rgb, int w, int h) {
    int v = -1, ma = 0x7FFFFFFF;
    for (int i = 0; i < TC_SIZE; i++) {
        if (!s_tc[i].used) { v = i; break; }
        if (s_tc[i].age < ma) { ma = s_tc[i].age; v = i; }
    }
    if (s_tc[v].rgb) heap_caps_free(s_tc[v].rgb);
    s_tc[v] = (tc_t){ .zoom=z, .tx=tx, .ty=ty, .w=w, .h=h,
                      .rgb=rgb, .age=++s_tc_age, .used=true };
}
static void tc_flush(void) {
    for (int i = 0; i < TC_SIZE; i++) {
        if (s_tc[i].rgb) heap_caps_free(s_tc[i].rgb);
        memset(&s_tc[i], 0, sizeof(tc_t));
    }
}

/* ====== Paste tile → canvas ====== */
static void paste(const uint16_t *src, int tw, int th,
                  rf_map_view_t *v, int sx, int sy) {
    if (!v->canvas || !lv_obj_is_valid(v->canvas)) return;
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(v->canvas);
    if (!db || !db->data) return;
    uint8_t *buf = (uint8_t *)db->data;
    uint32_t stride = db->header.stride;

    /* 边界裁剪 */
    int ox=0, oy=0, dx=sx, dy=sy, cw=tw, ch=th;
    if (dx<0) { ox=-dx; cw+=dx; dx=0; }
    if (dy<0) { oy=-dy; ch+=dy; dy=0; }
    if (dx+cw > v->canvas_w)  cw = v->canvas_w - dx;
    if (dy+ch > v->canvas_h)  ch = v->canvas_h - dy;
    if (cw<=0 || ch<=0) return;

    for (int y = 0; y < ch; y++) {
        const uint16_t *sr = src + (oy+y)*tw + ox;
        uint16_t *dr = (uint16_t*)(buf + (uint32_t)(dy+y)*stride);
        memcpy(dr+dx, sr, cw*2);
    }
}

/* ====== 加载一个 tile (cache→SD→paste) ====== */
static bool load_tile(rf_map_view_t *v, int tx, int ty, int slot_x, int slot_y) {
    int ci = tc_lookup(v->zoom, tx, ty);
    if (ci >= 0) {
        paste(s_tc[ci].rgb, s_tc[ci].w, s_tc[ci].h, v, slot_x, slot_y);
        return true;
    }
    char path[256]; make_path(v->zoom, tx, ty, path, sizeof(path));
    size_t fsz = 0;
    uint8_t *raw = read_psram(path, &fsz);
    if (!raw) return false;
    int w=0, h=0, ch=0;
    unsigned char *rgba = stbi_load_from_memory(raw, (int)fsz, &w, &h, &ch, 4);
    heap_caps_free(raw);
    if (!rgba) return false;
    int n = w*h;
    uint16_t *rgb = heap_caps_malloc((size_t)n*2, MALLOC_CAP_SPIRAM);
    if (!rgb) { stbi_image_free(rgba); return false; }
    for (int i = 0; i < n; i++) {
        uint8_t r=rgba[i*4], g=rgba[i*4+1], b=rgba[i*4+2];
        rgb[i] = ((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3);
    }
    stbi_image_free(rgba);
    tc_insert(v->zoom, tx, ty, rgb, w, h);
    paste(rgb, w, h, v, slot_x, slot_y);
    return true;
}

/* ====== 渲染 3×3 grid ====== */
static void render_grid(rf_map_view_t *v) {
    if (!v->canvas || !lv_obj_is_valid(v->canvas)) return;
    static const struct {int dx,dy;} o[9] = {
        {-1,-1},{0,-1},{1,-1},
        {-1, 0},{0, 0},{1, 0},
        {-1, 1},{0, 1},{1, 1},
    };
    int ok=0, fail=0;
    for (int i = 0; i < 9; i++) {
        vTaskDelay(1);
        if (load_tile(v,
                      v->tile_x+o[i].dx, v->tile_y+o[i].dy,
                      v->tile_slot_x+o[i].dx*RF_MAP_TILE_PX,
                      v->tile_slot_y+o[i].dy*RF_MAP_TILE_PX)) ok++;
        else fail++;
    }
    ESP_LOGI(TAG, "render z=%d tile=(%d,%d) slot=(%d,%d) ok=%d fail=%d",
             v->zoom, v->tile_x, v->tile_y, v->tile_slot_x, v->tile_slot_y, ok, fail);
    lv_obj_invalidate(v->canvas);
}

/* ====== SD 挂载 (SPI3_HOST, 避开 NRF SPI2_HOST) ====== */
static esp_err_t mount_sd(void) {
    if (s_mounted) return ESP_OK;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    spi_bus_config_t bc = {
        .mosi_io_num = RF_MAP_SD_CMD,
        .miso_io_num = RF_MAP_SD_D0,
        .sclk_io_num = RF_MAP_SD_CLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t e = spi_bus_initialize(host.slot, &bc, SDSPI_DEFAULT_DMA);
    if (e != ESP_OK) { ESP_LOGE(TAG, "spi_bus_initialize FAIL: %s", esp_err_to_name(e)); return e; }
    sdspi_device_config_t dc = SDSPI_DEVICE_CONFIG_DEFAULT();
    dc.host_id = host.slot; dc.gpio_cs = RF_MAP_SD_D3;
    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed=false, .max_files=4, .allocation_unit_size=16*1024,
    };
    for (int a = 0; a < 3; a++) {
        e = esp_vfs_fat_sdspi_mount(RF_MAP_SD_MOUNT, &host, &dc, &mc, &s_card);
        if (e == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (e != ESP_OK) {
        spi_bus_free(SPI3_HOST);
        ESP_LOGE(TAG, "SD mount FAIL: %s", esp_err_to_name(e)); return e;
    }
    s_mounted = true;
    ESP_LOGI(TAG, "SD mount OK card=%.1fGB SPI3_HOST", s_card->csd.capacity*0.000512);
    return ESP_OK;
}

/* ====== 公开接口 ====== */
bool rf_map_is_mounted(void) { return s_mounted; }
SemaphoreHandle_t rf_map_canvas_mutex(void) { return s_mut; }

esp_err_t rf_map_init(void) {
    s_mut = xSemaphoreCreateMutex();
    esp_err_t e = mount_sd();
    if (e != ESP_OK) return e;
    /* 列出可用 zoom */
    DIR *d = opendir(RF_MAP_TILE_DIR);
    if (d) {
        struct dirent *de;
        while ((de = readdir(d)))
            if (de->d_name[0] >= '0' && de->d_name[0] <= '9')
                ESP_LOGI(TAG, "zoom %s", de->d_name);
        closedir(d);
    }
    return ESP_OK;
}

void rf_map_deinit(void) {
    tc_flush();
    if (s_card) { esp_vfs_fat_sdcard_unmount(RF_MAP_SD_MOUNT, s_card); spi_bus_free(SPI3_HOST); s_card=NULL; }
    s_mounted = false;
}

/* ====== set_center: 地理中心 → tile + slot ======
 * tile_slot = canvas_center - frac*256 → 瓦片中心对准 canvas 中心 */
esp_err_t rf_map_set_center(rf_map_view_t *v, double lat, double lon, int zoom) {
    if (!v || !s_mounted) return ESP_ERR_INVALID_ARG;
    v->center_lat = lat; v->center_lon = lon; v->zoom = zoom;
    double fx = rf_map_lon_to_tile_x(lon, zoom);
    double fy = rf_map_lat_to_tile_y(lat, zoom);
    v->tile_x = (int)floor(fx);
    v->tile_y = (int)floor(fy);
    double fx_frac = fx - v->tile_x;
    double fy_frac = fy - v->tile_y;
    v->tile_slot_x = v->canvas_w/2 - (int)(fx_frac * RF_MAP_TILE_PX);
    v->tile_slot_y = v->canvas_h/2 - (int)(fy_frac * RF_MAP_TILE_PX);
    v->pending_pan_dx = v->pending_pan_dy = 0;
    ESP_LOGI(TAG, "set_center %.5f,%.5f z=%d → tile=(%d,%d) slot=(%d,%d)",
             lat, lon, zoom, v->tile_x, v->tile_y, v->tile_slot_x, v->tile_slot_y);
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
    render_grid(v);
    if (s_mut) xSemaphoreGive(s_mut);
    return ESP_OK;
}

/* ====== pan: memmove + slot 偏移 + shift ====== */
void rf_map_pan(rf_map_view_t *v, int dx, int dy) {
    if (!v || !v->canvas || !lv_obj_is_valid(v->canvas)) return;
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(v->canvas);
    if (!db || !db->data) return;
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    uint8_t *buf = (uint8_t*)db->data;
    uint32_t stride = db->header.stride;
    int w=v->canvas_w, h=v->canvas_h;
    const uint16_t BG = 0x1020;

    /* Y 整行 */
    if (dy>0 && dy<h) {
        memmove(buf+(uint32_t)dy*stride, buf, (uint32_t)(h-dy)*stride);
        for (int y=0;y<dy;y++) { uint16_t *r=(uint16_t*)(buf+(uint32_t)y*stride); for (int x=0;x<w;x++) r[x]=BG; }
    } else if (dy<0 && -dy<h) {
        int a=-dy; memmove(buf, buf+(uint32_t)a*stride, (uint32_t)(h-a)*stride);
        uint8_t *e=buf+(uint32_t)(h-a)*stride;
        for (int y=0;y<a;y++) { uint16_t *r=(uint16_t*)(e+(uint32_t)y*stride); for (int x=0;x<w;x++) r[x]=BG; }
    }
    /* X 逐行 */
    if (dx>0 && dx<w) {
        uint32_t mb=(uint32_t)(w-dx)*2;
        for (int y=0;y<h;y++) { uint8_t *row=buf+(uint32_t)y*stride; memmove(row+dx*2,row,mb); uint16_t *rp=(uint16_t*)row; for (int x=w-dx;x<w;x++) rp[x]=BG; }
    } else if (dx<0 && -dx<w) {
        int a=-dx; uint32_t mb=(uint32_t)(w-a)*2;
        for (int y=0;y<h;y++) { uint8_t *row=buf+(uint32_t)y*stride; memmove(row,row+(uint32_t)a*2,mb); uint16_t *rp=(uint16_t*)row; for (int x=0;x<a;x++) rp[x]=BG; }
    }

    v->tile_slot_x += dx;
    v->tile_slot_y += dy;
    if (s_mut) xSemaphoreGive(s_mut);

    /* shift: slot 偏离 center-128 超过 ±256 → tile 进位 */
    int tx = v->canvas_w/2 - RF_MAP_TILE_PX/2;   /* 96 (target) */
    int ty = v->canvas_h/2 - RF_MAP_TILE_PX/2;   /* 12 */
    int sx = (int)round((double)(v->tile_slot_x - tx) / 256.0);
    int sy = (int)round((double)(v->tile_slot_y - ty) / 256.0);
    if (sx || sy) {
        if (sx) { v->tile_x -= sx; v->tile_slot_x -= sx*256; }
        if (sy) { v->tile_y -= sy; v->tile_slot_y -= sy*256; }
        ESP_LOGI(TAG, "🆙 shift (%+d,%+d) → tile=(%d,%d) slot=(%d,%d)",
                 sx, sy, v->tile_x, v->tile_y, v->tile_slot_x, v->tile_slot_y);
    }
}

/* ====== zoom: 保持地理中心不变 ====== */
void rf_map_zoom(rf_map_view_t *v, int delta) {
    if (!v) return;
    int nz = v->zoom + delta;
    if (nz<10 || nz>19 || nz==v->zoom) return;
    double fx = rf_map_lon_to_tile_x(v->center_lon, nz);
    double fy = rf_map_lat_to_tile_y(v->center_lat, nz);
    int tx=(int)floor(fx), ty=(int)floor(fy);
    if (!rf_map_tile_exists(nz, tx, ty)) {
        ESP_LOGW(TAG, "zoom blocked: %d/%d/%d missing", nz, ty, tx); return;
    }
    v->zoom=nz; v->tile_x=tx; v->tile_y=ty;
    v->tile_slot_x = v->canvas_w/2 - (int)((fx-tx)*RF_MAP_TILE_PX);
    v->tile_slot_y = v->canvas_h/2 - (int)((fy-ty)*RF_MAP_TILE_PX);
    v->pending_pan_dx = v->pending_pan_dy = 0;
    ESP_LOGI(TAG, "zoom → z=%d tile=(%d,%d) slot=(%d,%d)", nz, tx, ty, v->tile_slot_x, v->tile_slot_y);
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
    render_grid(v);
    if (s_mut) xSemaphoreGive(s_mut);
}

/* ====== 投影 ====== */
bool rf_map_project(const rf_map_view_t *v, double lat, double lon,
                    int *cxo, int *cyo) {
    if (!v || !cxo || !cyo || v->zoom<=0) return false;
    double fx = rf_map_lon_to_tile_x(lon, v->zoom);
    double fy = rf_map_lat_to_tile_y(lat, v->zoom);
    int px = v->tile_slot_x + (int)((fx - v->tile_x) * RF_MAP_TILE_PX);
    int py = v->tile_slot_y + (int)((fy - v->tile_y) * RF_MAP_TILE_PX);
    if (px<0||px>=v->canvas_w||py<0||py>=v->canvas_h) return false;
    *cxo = px; *cyo = py; return true;
}

/* ====== 同步 edges 补全 (松手后调用) ====== */
esp_err_t rf_map_render_edges(rf_map_view_t *v) {
    if (!v) return ESP_ERR_INVALID_ARG;
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    render_grid(v);
    if (s_mut) xSemaphoreGive(s_mut);
    return ESP_OK;
}

esp_err_t rf_map_render_multi(rf_map_view_t *v) {
    if (!v) return ESP_ERR_INVALID_ARG;
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
    render_grid(v);
    if (s_mut) xSemaphoreGive(s_mut);
    return ESP_OK;
}

/* 异步请求包装 (保留接口但内部同步) */
void rf_map_request_edges(rf_map_view_t *v) { rf_map_render_edges(v); }
void rf_map_request_redraw(rf_map_view_t *v) { rf_map_render_multi(v); }

void rf_map_exit_page(rf_map_view_t *v) {
    ESP_LOGI(TAG, "🧹 exit_page");
    tc_flush();
    if (v && v->canvas && lv_obj_is_valid(v->canvas))
        lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
}
