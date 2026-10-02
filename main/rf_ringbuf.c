/*
 * rf_ringbuf.c — 通用（void* 槽位）环形缓冲实现
 */
#include "rf_ringbuf.h"

int rf_ringbuf_push_copy(rf_ringbuf_t *rb, const void *item)
{
    if (!rb || !item || rb->capacity <= 0 || !rb->slots) return -1;
    int dropped = 0;
    char *dst = (char *)rb->slots + (size_t)rb->head * rb->slot_sz;
    memcpy(dst, item, rb->slot_sz);
    rb->head = (rb->head + 1) % rb->capacity;
    if (rb->count < rb->capacity) {
        rb->count++;
    } else {
        rb->dropped++;
        dropped = 1;
    }
    rb->total++;
    return dropped;
}

int rf_ringbuf_read_seq(const rf_ringbuf_t *rb, void *out, int out_max, uint32_t *from_seq)
{
    if (!rb || !out || !from_seq || out_max <= 0 || rb->capacity <= 0 || !rb->slots) return 0;
    uint32_t want_seq = *from_seq;
    uint32_t cur_total = rb->total;
    int count = rb->count;
    if (want_seq > cur_total) want_seq = cur_total;
    int available = (int)(cur_total - want_seq);
    if (available <= 0) { *from_seq = cur_total; return 0; }
    if (available > count)   available = count;
    if (available > out_max) available = out_max;

    int oldest_idx = (rb->head - count + rb->capacity + rb->capacity) % rb->capacity;
    /* 跳过 want_seq 比 total-count 更旧的部分 */
    int skip = (int)(((int64_t)want_seq) - ((int64_t)(cur_total - (uint32_t)count)));
    if (skip < 0) skip = 0;
    int start = (oldest_idx + skip + rb->capacity) % rb->capacity;
    char *dst = (char *)out;
    for (int i = 0; i < available; i++) {
        int k = (start + i) % rb->capacity;
        memcpy(dst + (size_t)i * rb->slot_sz,
               (const char *)rb->slots + (size_t)k * rb->slot_sz,
               rb->slot_sz);
    }
    *from_seq = want_seq + (uint32_t)available;
    return available;
}
