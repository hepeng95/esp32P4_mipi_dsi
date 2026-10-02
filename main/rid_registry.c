/*
 * rid_registry.c — 无人机 Remote ID 注册表实现（见 rid_registry.h）
 * 纯 C99，无 malloc，零 ESP/LVGL/FreeRTOS 依赖。
 */
#include "rid_registry.h"

#include <string.h>
#include <limits.h>

#ifndef UINT32_MAX
#define UINT32_MAX  ((uint32_t)0xFFFFFFFFUL)
#endif

static bool ids_equal(uint8_t a_type, const char *a, uint8_t b_type,
                      const char *b, size_t id_sz)
{
    if (a_type != b_type) return false;
    /* 逐字比较到 id_sz（两边都是定长，末尾 '\0' 可作为区分） */
    for (size_t i = 0; i < id_sz; i++) {
        if (a[i] != b[i]) return false;
        if (a[i] == '\0') return true;   /* 后面都为 0 无需再比 */
    }
    return true;
}

static int find_slot(const rid_registry_t *r, uint8_t id_type, const char *id, size_t id_sz)
{
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS; i++) {
        if (!r->slots[i].active) continue;
        if (ids_equal(r->slots[i].uas_id_type, r->slots[i].uas_id,
                      id_type, id, id_sz)) return i;
    }
    return -1;
}

static int alloc_slot(rid_registry_t *r)
{
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS; i++) {
        if (!r->slots[i].active) return i;
    }
    /* 满：踢掉最久未见 */
    int worst = 0;
    uint32_t oldest = UINT32_MAX;
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS; i++) {
        if (r->slots[i].last_seen_ms < oldest) {
            oldest = r->slots[i].last_seen_ms;
            worst = i;
        }
    }
    r->slots[worst].active = false;
    return worst;
}

static void safe_cpy_n(char *dst, size_t dst_sz, const char *src, size_t src_sz)
{
    size_t n = (src_sz < dst_sz - 1) ? src_sz : (dst_sz - 1);
    memcpy(dst, src, n);
    dst[n] = '\0';
}

void rid_registry_init(rid_registry_t *r)
{
    if (!r) return;
    memset(r, 0, sizeof(*r));
    r->gen = 0;
}

void rid_registry_reset(rid_registry_t *r)
{
    if (!r) return;
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS; i++) {
        r->slots[i].active = false;
    }
    r->gen++;
}

int rid_registry_copy_active(const rid_registry_t *r, rf_drone_t *out, int out_max)
{
    if (!r || !out || out_max <= 0) return 0;
    int cnt = 0;
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS && cnt < out_max; i++) {
        if (r->slots[i].active) {
            out[cnt++] = r->slots[i];
        }
    }
    return cnt;
}

uint32_t rid_registry_generation(const rid_registry_t *r)
{
    return r ? r->gen : 0;
}

int rid_registry_upsert_wire(rid_registry_t *r, const rf_rid_wire_t *w,
                             uint32_t now_ms, int *added)
{
    if (!r || !w) return -2;
    int slot = find_slot(r, w->uas_id_type, w->uas_id, sizeof(w->uas_id));
    bool is_new = (slot < 0);
    if (is_new) {
        slot = alloc_slot(r);
        memset(&r->slots[slot], 0, sizeof(rf_drone_t));
    }
    rf_drone_t *d = &r->slots[slot];
    d->active = true;
    d->uas_id_type = w->uas_id_type;
    safe_cpy_n(d->uas_id, sizeof(d->uas_id), w->uas_id, sizeof(w->uas_id));
    safe_cpy_n(d->op_desc, sizeof(d->op_desc), w->op_desc, sizeof(w->op_desc));
    safe_cpy_n(d->operator_id, sizeof(d->operator_id), w->operator_id, sizeof(w->operator_id));
    d->lat_e7 = w->lat_e7;
    d->lon_e7 = w->lon_e7;
    d->operator_lat_e7 = w->operator_lat_e7;
    d->operator_lon_e7 = w->operator_lon_e7;
    d->alt_pressure_m = w->alt_pressure_m;
    d->alt_geodetic_m = w->alt_geodetic_m;
    d->height_m      = w->height_m;
    d->horiz_speed   = w->horiz_speed;
    d->vert_speed    = w->vert_speed;
    d->ground_track  = w->ground_track;
    d->last_seen_ms  = now_ms;
    d->flags         = w->flags;
    r->gen++;
    if (added) *added = is_new ? 1 : 0;
    return slot;
}

int rid_registry_upsert_drone(rid_registry_t *r, const rf_drone_t *in, int *added)
{
    if (!r || !in) return -2;
    int slot = find_slot(r, in->uas_id_type, in->uas_id, sizeof(in->uas_id));
    bool is_new = (slot < 0);
    if (is_new) {
        slot = alloc_slot(r);
        memset(&r->slots[slot], 0, sizeof(rf_drone_t));
    }
    r->slots[slot] = *in;
    r->slots[slot].active = true;
    r->gen++;
    if (added) *added = is_new ? 1 : 0;
    return slot;
}

int rid_registry_remove_by_id(rid_registry_t *r, uint8_t id_type, const char *uas_id)
{
    if (!r || !uas_id) return 0;
    int slot = find_slot(r, id_type, uas_id, sizeof(r->slots[0].uas_id));
    if (slot < 0) return 0;
    r->slots[slot].active = false;
    r->gen++;
    return 1;
}

int rid_registry_purge_expired(rid_registry_t *r, uint32_t now_ms, uint32_t ttl_ms)
{
    if (!r) return 0;
    int removed = 0;
    for (int i = 0; i < RID_REGISTRY_MAX_SLOTS; i++) {
        if (!r->slots[i].active) continue;
        uint32_t delta;
        if (now_ms >= r->slots[i].last_seen_ms) delta = now_ms - r->slots[i].last_seen_ms;
        else                                    delta = (UINT32_MAX - r->slots[i].last_seen_ms) + now_ms + 1;
        if (delta > ttl_ms) {
            r->slots[i].active = false;
            removed++;
        }
    }
    if (removed > 0) r->gen++;
    return removed;
}
