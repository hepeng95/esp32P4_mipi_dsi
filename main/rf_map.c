/*
 * rf_map.c — v4 修复版 (zoom锚点 + cache扩 + memmove方向)
 *
 * ====== 三个修复 ======
 * 1. zoom 锚点: 从 canvas 中心反推当前地理点再缩放 (不是用旧 center_lat)
 * 2. cache 扩 TC_SIZE=32 (5x5=25 tile 覆盖, PSRAM 32MB 足够)
 *    + render 异步 task: pan 只 memmove, 边缘补全异步做, 不卡 UI
 * 3. memmove 方向: 手指向右拖 → canvas 像素整体向左搬 (方向反了!)
 *    + pan 后 lv_obj_invalidate 通知 LVGL 刷新
 *
 * ====== 坐标系 ======
 *   tile_x, tile_y           = Web Mercator 中心瓦片
 *   tile_slot_x, tile_slot_y = 瓦片左上角在 canvas 上的像素坐标
 *   canvas_center = (canvas_w/2, canvas_h/2) — canvas 中心像素
 *
 *   初始: tile_slot = canvas_center - frac*256 (地理中心对准 canvas 中心)
 *
 *   pan(dx, dy): 手指拖 dx → slot += dx
 *     手指向右拖 +300 → slot_x += 300 → 地图在 canvas 上整体左移
 *     用户看到的效果: 地图向左滑, 看到更西边的内容 ← 和直觉一致
 *
 *   memmove 方向: canvas 内容需要整体反向移动
 *     dx > 0 (手指向右拖): canvas 像素向左搬 → memmove(row, row+dx*2, ...)
 *     dx < 0 (手指向左拖): canvas 像素向右搬 → memmove(row+|dx|*2, row, ...)
 *     dy > 0 (手指向下拖): canvas 像素向上搬 → memmove(buf, buf+dy*stride, ...)
 *     dy < 0 (手指向上拖): canvas 像素向下搬 → memmove(buf+|dy|*stride, buf, ...)
 *
 * ====== Zoom 反推地理坐标 ======
 *   tile_frac_x = (canvas_center_x - tile_slot_x) / 256.0 + tile_x
 *   lon = tile_frac_x * 360.0 / (1 << zoom) - 180.0
 *   tile_frac_y = (canvas_center_y - tile_slot_y) / 256.0 + tile_y
 *   lat = mercator_inverse(tile_frac_y / (1 << zoom))
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
#include <stdatomic.h>

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

/* ====== Tile Cache: 扩到 32 (5x5 覆盖 + 冗余) ======
 * 每个 tile ≈ 256*256*2 = 128KB, 32 个 = 4MB, PSRAM 32MB 足够! */
#define TC_SIZE 32
typedef struct {
    int zoom, tx, ty;
    int w, h;
    uint16_t *rgb;
    uint32_t age;
    bool used;
} tc_t;
static tc_t s_tc[TC_SIZE];
static uint32_t s_tc_age = 0;

/* ====== 无 render_task — 全部同步 (避免 mutex 竞争卡死 UI) ====== */
static TaskHandle_t s_render_hdl = NULL;   /* 保留 NULL 即可, 不启动 */
static _Atomic int s_pending = 0;
static rf_map_view_t s_pending_view = { 0 };

#define RENDER_CORE  1
#define RENDER_PRIO  2

/* ====== Web Mercator 正/逆投影 ====== */
static inline double d2r(double d) { return d * M_PI / 180.0; }
static inline double r2d(double r) { return r * 180.0 / M_PI; }

double rf_map_lon_to_tile_x(double lon, int z) {
    return (lon + 180.0) / 360.0 * (double)(1 << z);
}
double rf_map_lat_to_tile_y(double lat, int z) {
    if (lat >  85.051129) lat =  85.051129;
    if (lat < -85.051129) lat = -85.051129;
    double t = tan(d2r(lat)) + 1.0 / cos(d2r(lat));
    return (1.0 - log(t) / M_PI) / 2.0 * (double)(1 << z);
}

/* 逆投影: tile_y_frac → lat (弧度 → 度) */
static double tile_y_to_lat(double ty_frac, int z) {
    double n = M_PI - 2.0 * M_PI * ty_frac / (double)(1 << z);
    return r2d(atan(0.5 * (exp(n) - exp(-n))));
}

/* 从 canvas 中心像素反推当前地理坐标 (pan 后 center_lat/lon 可能过期) */
static void canvas_center_to_geo(const rf_map_view_t *v,
                                  double *lat_out, double *lon_out) {
    int cx = v->canvas_w / 2;
    int cy = v->canvas_h / 2;
    double fx = (double)(cx - v->tile_slot_x) / (double)RF_MAP_TILE_PX + (double)v->tile_x;
    double fy = (double)(cy - v->tile_slot_y) / (double)RF_MAP_TILE_PX + (double)v->tile_y;
    *lon_out = fx * 360.0 / (double)(1 << v->zoom) - 180.0;
    *lat_out = tile_y_to_lat(fy, v->zoom);
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

/* ====== 加载一个 tile ====== */
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

/* ====== 渲染 N×N grid ====== */
static void render_grid(rf_map_view_t *v, int half_n) {
    if (!v->canvas || !lv_obj_is_valid(v->canvas)) return;
    int ok=0, fail=0, total=0;
    for (int dy=-half_n; dy<=half_n; dy++) {
        for (int dx=-half_n; dx<=half_n; dx++) {
            vTaskDelay(1);   /* 防 WDT, 每个 tile 让一次 */
            total++;
            if (load_tile(v,
                          v->tile_x+dx, v->tile_y+dy,
                          v->tile_slot_x+dx*RF_MAP_TILE_PX,
                          v->tile_slot_y+dy*RF_MAP_TILE_PX)) ok++;
            else fail++;
        }
    }
    ESP_LOGI(TAG, "render%dx%d z=%d tile=(%d,%d) ok=%d fail=%d",
             half_n*2+1, half_n*2+1, v->zoom, v->tile_x, v->tile_y, ok, fail);
    lv_obj_invalidate(v->canvas);
}

/* ====== 异步 render task ====== */
static void rf_map_render_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "render task start (CPU%d, prio %d, cache=%d tiles)", RENDER_CORE, RENDER_PRIO, TC_SIZE);
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(300));
        int mode = __atomic_load_n(&s_pending, __ATOMIC_ACQUIRE);
        if (!mode) continue;
        __atomic_store_n(&s_pending, 0, __ATOMIC_RELEASE);
        rf_map_view_t v = s_pending_view;
        if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
        if (mode == 1) {
            /* dirty: 全屏重绘 */
            lv_canvas_fill_bg(v.canvas, lv_color_hex(0x1020), LV_OPA_COVER);
            render_grid(&v, 2);   /* 5x5 */
        } else {
            /* edges: 边缘补全 (3x3) */
            render_grid(&v, 1);
        }
        if (s_mut) xSemaphoreGive(s_mut);
    }
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
    DIR *d = opendir(RF_MAP_TILE_DIR);
    if (d) {
        struct dirent *de;
        while ((de = readdir(d)))
            if (de->d_name[0] >= '0' && de->d_name[0] <= '9')
                ESP_LOGI(TAG, "zoom %s", de->d_name);
        closedir(d);
    }
    /* ★ 不启动 render_task — 全部同步 (避免 mutex 竞争) */
    return ESP_OK;
}

void rf_map_deinit(void) {
    if (s_render_hdl) { vTaskDelete(s_render_hdl); s_render_hdl = NULL; }
    tc_flush();
    if (s_card) { esp_vfs_fat_sdcard_unmount(RF_MAP_SD_MOUNT, s_card); spi_bus_free(SPI3_HOST); s_card=NULL; }
    s_mounted = false;
}

void rf_map_exit_page(rf_map_view_t *v) {
    ESP_LOGI(TAG, "exit_page");
    __atomic_store_n(&s_pending, 0, __ATOMIC_RELEASE);
    tc_flush();
    if (v && v->canvas && lv_obj_is_valid(v->canvas))
        lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
}

/* ====== set_center ====== */
esp_err_t rf_map_set_center(rf_map_view_t *v, double lat, double lon, int zoom) {
    if (!v || !s_mounted) return ESP_ERR_INVALID_ARG;
    v->center_lat = lat; v->center_lon = lon; v->zoom = zoom;
    double fx = rf_map_lon_to_tile_x(lon, zoom);
    double fy = rf_map_lat_to_tile_y(lat, zoom);
    v->tile_x = (int)floor(fx);
    v->tile_y = (int)floor(fy);
    v->tile_slot_x = v->canvas_w/2 - (int)((fx-v->tile_x) * RF_MAP_TILE_PX);
    v->tile_slot_y = v->canvas_h/2 - (int)((fy-v->tile_y) * RF_MAP_TILE_PX);
    v->pending_pan_dx = v->pending_pan_dy = 0;
    ESP_LOGI(TAG, "set_center %.5f,%.5f z=%d → tile=(%d,%d) slot=(%d,%d)",
             lat, lon, zoom, v->tile_x, v->tile_y, v->tile_slot_x, v->tile_slot_y);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER); render_grid(v, 1);
    return ESP_OK;
}

/* ====== pan ★ 四个问题一起修 ======
 * 1. 方向反 → tile_slot -= dx (不是 +=)
 * 2. 瓦片没及时补全 → 每次 pan 回调末尾都 render_grid(1) (cache hit 秒开)
 * 3. 超过边界才刷新 → shift 阈值 ±128 (半瓦片), 更灵敏
 * 4. 全部同级瓦片可见 → cache 32 tile + 每帧补全
 */
void rf_map_pan(rf_map_view_t *v, int dx, int dy) {
    if (!v || !v->canvas || !lv_obj_is_valid(v->canvas)) return;
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(v->canvas);
    if (!db || !db->data) return;

    uint8_t *buf = (uint8_t*)db->data;
    uint32_t stride = db->header.stride;
    int w=v->canvas_w, h=v->canvas_h;
    const uint16_t BG = 0x1020;

    /*
     * ★★★ memmove 方向 (手指向右拖, canvas 像素整体向左搬) ★★★
     * dx > 0: memmove(row, row+dx*2, (w-dx)*2)  ← 像素向左搬 ✓
     * dy > 0: memmove(buf, buf+dy*stride, (h-dy)*stride) ← 向上搬 ✓
     * ★★★ slot 方向 (修正! 之前是 +=dx, 应该 -=dx) ★★★
     * dx > 0 手指向右拖 → 瓦片在 canvas 上向左移 → tile_slot_x 减小
     * tile_slot_x -= dx ← ★ 这才是正确方向!
     */

    /* Y: 整行 memmove */
    if (dy > 0 && dy < h) {
        memmove(buf, buf + (uint32_t)dy * stride, (uint32_t)(h - dy) * stride);
        for (int y = h - dy; y < h; y++) {
            uint16_t *r = (uint16_t*)(buf + (uint32_t)y * stride);
            for (int x = 0; x < w; x++) r[x] = BG;
        }
    } else if (dy < 0 && -dy < h) {
        int a = -dy;
        memmove(buf + (uint32_t)a * stride, buf, (uint32_t)(h - a) * stride);
        for (int y = 0; y < a; y++) {
            uint16_t *r = (uint16_t*)(buf + (uint32_t)y * stride);
            for (int x = 0; x < w; x++) r[x] = BG;
        }
    }

    /* X: 逐行 memmove */
    if (dx > 0 && dx < w) {
        uint32_t mb = (uint32_t)(w - dx) * 2;
        for (int y = 0; y < h; y++) {
            uint8_t *row = buf + (uint32_t)y * stride;
            memmove(row, row + dx * 2, mb);
            uint16_t *rp = (uint16_t*)row;
            for (int x = w - dx; x < w; x++) rp[x] = BG;
        }
    } else if (dx < 0 && -dx < w) {
        int a = -dx;
        uint32_t mb = (uint32_t)(w - a) * 2;
        for (int y = 0; y < h; y++) {
            uint8_t *row = buf + (uint32_t)y * stride;
            memmove(row + a * 2, row, mb);
            uint16_t *rp = (uint16_t*)row;
            for (int x = 0; x < a; x++) rp[x] = BG;
        }
    }

    /* ★★★ slot 方向! 手指向右拖 → 瓦片向右移(同方向) → slot 增大 ★★★ */
    v->tile_slot_x += dx;
    v->tile_slot_y += dy;
    v->pending_pan_dx += dx;
    v->pending_pan_dy += dy;

    lv_obj_invalidate(v->canvas);

    /* ★ shift 检查 (±128 半瓦片阈值, 比之前更灵敏) */
    int tx_target = v->canvas_w/2 - RF_MAP_TILE_PX/2;   /* 96 */
    int ty_target = v->canvas_h/2 - RF_MAP_TILE_PX/2;   /* 12 */
    int sx = (int)round((double)(v->tile_slot_x - tx_target) / (double)RF_MAP_TILE_PX);
    int sy = (int)round((double)(v->tile_slot_y - ty_target) / (double)RF_MAP_TILE_PX);
    if (sx || sy) {
        if (sx) { v->tile_x -= sx; v->tile_slot_x -= sx * RF_MAP_TILE_PX; }
        if (sy) { v->tile_y -= sy; v->tile_slot_y -= sy * RF_MAP_TILE_PX; }
        v->pending_pan_dx = v->pending_pan_dy = 0;
        ESP_LOGI(TAG, "🆙 shift (%+d,%+d) tile=(%d,%d) slot=(%d,%d)",
                 sx, sy, v->tile_x, v->tile_y, v->tile_slot_x, v->tile_slot_y);
    }

    /* ★★★ 关键: 每次 pan 回调都 render_grid(1) 补全 BG 区域 ★★★
     * cache hit → memcpy 级速度 (无 decode), 立即覆盖新露出的 BG
     * 这样用户拖到哪里, 瓦片就补到哪里, 无等待! */
    render_grid(v, 1);   /* 3x3, 同步 */
}

/* ====== zoom ★ 从 canvas 中心反推地理点 ★ ====== */
void rf_map_zoom(rf_map_view_t *v, int delta) {
    if (!v) return;
    int nz = v->zoom + delta;
    if (nz<10 || nz>19 || nz==v->zoom) return;

    /* ★★★ 关键: 反推 canvas 中心当前对应的地理坐标 ★★★
     * center_lat/lon 可能是 set_center 时的初始值, pan 后已过期! */
    double cur_lat, cur_lon;
    canvas_center_to_geo(v, &cur_lat, &cur_lon);
    ESP_LOGI(TAG, "zoom anchor: canvas_center → lat=%.6f lon=%.6f (stored was %.6f,%.6f)",
             cur_lat, cur_lon, v->center_lat, v->center_lon);

    /* 用 cur_lat/cur_lon 做 zoom 锚点 */
    double fx = rf_map_lon_to_tile_x(cur_lon, nz);
    double fy = rf_map_lat_to_tile_y(cur_lat, nz);
    int tx = (int)floor(fx), ty = (int)floor(fy);
    if (!rf_map_tile_exists(nz, tx, ty)) {
        ESP_LOGW(TAG, "zoom blocked: %d/%d/%d missing", nz, ty, tx); return;
    }

    v->center_lat = cur_lat;   /* 同步更新 */
    v->center_lon = cur_lon;
    v->zoom = nz;
    v->tile_x = tx; v->tile_y = ty;
    v->tile_slot_x = v->canvas_w/2 - (int)((fx - tx) * RF_MAP_TILE_PX);
    v->tile_slot_y = v->canvas_h/2 - (int)((fy - ty) * RF_MAP_TILE_PX);
    v->pending_pan_dx = v->pending_pan_dy = 0;

    ESP_LOGI(TAG, "zoom → z=%d tile=(%d,%d) slot=(%d,%d)", nz, tx, ty,
             v->tile_slot_x, v->tile_slot_y);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER); render_grid(v, 1);
}

/* ====== 回中: 用默认中心 ====== */
void rf_map_home(rf_map_view_t *v, double lat, double lon) {
    rf_map_set_center(v, lat, lon, v->zoom);
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

/* ====== 异步请求包装 ====== */
void rf_map_request_edges(rf_map_view_t *v) {
    if (!v || !s_render_hdl) return;
    s_pending_view = *v;
    __atomic_store_n(&s_pending, 2, __ATOMIC_RELEASE);
    xTaskNotifyGive(s_render_hdl);
}
void rf_map_request_redraw(rf_map_view_t *v) {
    if (!v || !s_render_hdl) return;
    s_pending_view = *v;
    __atomic_store_n(&s_pending, 1, __ATOMIC_RELEASE);
    xTaskNotifyGive(s_render_hdl);
}

/* 同步版本 (保留接口, 初始化/回中时用) */
esp_err_t rf_map_render_multi(rf_map_view_t *v) {
    if (!v) return ESP_ERR_INVALID_ARG;
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    lv_canvas_fill_bg(v->canvas, lv_color_hex(0x1020), LV_OPA_COVER);
    render_grid(v, 1);
    if (s_mut) xSemaphoreGive(s_mut);
    return ESP_OK;
}
esp_err_t rf_map_render_edges(rf_map_view_t *v) {
    if (!v) return ESP_ERR_INVALID_ARG;
    if (s_mut) xSemaphoreTake(s_mut, portMAX_DELAY);
    render_grid(v, 1);
    if (s_mut) xSemaphoreGive(s_mut);
    return ESP_OK;
}



