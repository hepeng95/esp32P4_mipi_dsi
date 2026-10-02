/*
 * rid_registry.h — 无人机 Remote ID 注册表（纯 C99，零 ESP/LVGL/FreeRTOS 依赖）
 *
 * 职责：
 *   - 管理容量为 RID_REGISTRY_MAX_SLOTS 的固定槽数组
 *   - 按 uas_id_type + uas_id 匹配：存在则 update，不存在则 allocate 新槽
 *   - 支持 remove / reset / active 过期清理
 *   - 每次增删改 bump "generation"，调用方据此判断是否需要刷新 UI
 *   - 所有写操作均由调用方负责加锁（本模块故意不带互斥，方便在无 OS 环境也能使用）
 *
 * 移植注意：
 *   - 直接 rid_registry.h + rid_registry.c 拷贝即可
 *   - 调用方需自行提供 uint32_t 毫秒时间戳（例如 rid_registry_set_now_ms(now_ms)），
 *     或通过 rid_registry_upsert_by_wire 的 now_ms 参数直接传入
 */
#pragma once

/*
 * 依赖策略（便于移植）：
 *   - 只通过 rf_types.h 拿到 rf_drone_t / rf_rid_wire_t / RF_MAX_DRONES
 *   - 不 include rf_link.h（rf_link 是"链路+业务"模块，可能带 ESP/FreeRTOS 依赖）
 */
#include "rf_types.h"   /* rf_drone_t / rf_rid_wire_t / RF_MAX_DRONES */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RID_REGISTRY_MAX_SLOTS
#define RID_REGISTRY_MAX_SLOTS  RF_MAX_DRONES
#endif

typedef struct {
    rf_drone_t slots[RID_REGISTRY_MAX_SLOTS];
    uint32_t   gen;          /* 每一次增/删/改 +1，用于 UI diff */
} rid_registry_t;

/* ---------- 生命周期 ---------- */
void rid_registry_init(rid_registry_t *r);
void rid_registry_reset(rid_registry_t *r);    /* 清空所有槽，gen++ */

/* ---------- 查询 ---------- */
/* 返回活动无人机数量并拷贝到 out（最多 out_max 条）。
 * 与 rf_link_get_drones 语义完全一致，便于替换。 */
int  rid_registry_copy_active(const rid_registry_t *r, rf_drone_t *out, int out_max);
/* 返回当前世代号 */
uint32_t rid_registry_generation(const rid_registry_t *r);

/* ---------- 增删改（写操作会 gen++） ---------- */
/* 按 wire 插入/更新无人机。now_ms 是当前本地毫秒时间戳。
 * 返回 >=0 = 使用的槽号；<0 = 失败（槽满）。*added 非 NULL 时填 1=新增 0=更新。 */
int rid_registry_upsert_wire(rid_registry_t *r, const rf_rid_wire_t *w,
                             uint32_t now_ms, int *added);

/* 与 upsert 类似，但接受已经是 rf_drone_t 字段格式的更新（用于 JSON 解析直接构造 drone）。 */
int rid_registry_upsert_drone(rid_registry_t *r, const rf_drone_t *d, int *added);

/* 按 uas_id_type+uas_id 移除指定无人机。成功返回 1，找不到返回 0。 */
int  rid_registry_remove_by_id(rid_registry_t *r, uint8_t id_type, const char *uas_id);

/* 清理距 now_ms 超过 ttl_ms 的活动无人机；返回清理数量 */
int  rid_registry_purge_expired(rid_registry_t *r, uint32_t now_ms, uint32_t ttl_ms);

#ifdef __cplusplus
}
#endif
