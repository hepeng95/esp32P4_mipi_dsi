/*
 * rf_ringbuf.h — 通用定长环形缓冲（纯 C，零依赖）
 *
 * 提供两种使用方式：
 *   1) RF_RINGBUF_DECLARE_T(T,N) 生成一个类型化的 ringbuf T_ringbuf_N_t，适合 POD
 *   2) 用通用 rf_ringbuf_t（void* slots + slot_sz + memcpy），无需宏
 *
 * 两个抽象都支持：
 *   - push（覆盖式：满则覆盖最老，返回是否丢弃过）
 *   - read_by_seq（基于 "累计写入数 total" 作为 seq 增量读取）
 *   - 没有互斥：调用方负责加锁（方便在无 OS / 单消费者等各种场景用）
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================ 通用（void* 槽位）版本 ================================= */
typedef struct {
    void     *slots;         /* 数组首地址 */
    size_t    slot_sz;       /* 单槽字节数 */
    int       capacity;      /* 槽数 */
    int       head;          /* 下一个写入位置 */
    int       count;         /* 当前有效条目数（<= capacity） */
    uint32_t  total;         /* 累计写入（单调，做 seq 用） */
    uint32_t  dropped;       /* 因满丢弃数 */
} rf_ringbuf_t;

/* 初始化：slots/slot_sz/capacity 先填好，其它字段归零 */
static inline void rf_ringbuf_init(rf_ringbuf_t *rb)
{
    if (!rb) return;
    rb->head = 0;
    rb->count = 0;
    rb->total = 0;
    rb->dropped = 0;
}

/* 压入一条（memcpy 方式。满则覆盖最老）。返回 1=本次丢弃了最老；0=正常。 */
int  rf_ringbuf_push_copy(rf_ringbuf_t *rb, const void *item);

/* 按 seq 增量读取：返回从 *from_seq 起的 <=out_max 条，复制到 out，*from_seq 推进到新 total。 */
int  rf_ringbuf_read_seq(const rf_ringbuf_t *rb, void *out, int out_max, uint32_t *from_seq);

/* seq 快照：若只想看最新 N 条里 [total-N, total) 区间，可据此计算。 */
static inline void rf_ringbuf_stat(const rf_ringbuf_t *rb,
                                   uint32_t *out_total, uint32_t *out_dropped,
                                   int *out_count)
{
    if (!rb) return;
    if (out_total)   *out_total   = rb->total;
    if (out_dropped) *out_dropped = rb->dropped;
    if (out_count)   *out_count   = rb->count;
}

#ifdef __cplusplus
}
#endif
