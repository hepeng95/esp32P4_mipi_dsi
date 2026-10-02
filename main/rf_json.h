/*
 * rf_json.h — 极简 JSON 查找/解析工具（纯 C99，零依赖）
 *
 * 支持:顶层对象 { key:value, ... } 的按键查找。
 * value 支持: int / bool / string / null / double / 嵌套对象 / 数组(长度、子对象定位、int 数组)。
 *
 * 不做语法树构建；直接对原字符串区间匹配，没有任何 malloc。
 * 设计目标: 小、快、在 MCU 上没有任何堆开销。
 *
 * 在本工程里由 rf_link.c 的 JSON 帧解析 ($D / $W / $R / $A / $R0) 调用。
 * 移植: 直接 rf_json.h + rf_json.c 两个文件一起拷贝走即可，无需其它依赖。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *p;
    const char *end;
} rf_json_ctx_t;

/* ---------- 基础工具（内部使用，但对外暴露以便扩展） ---------- */
void         rf_json_skip_ws(rf_json_ctx_t *c);
int          rf_json_expect(rf_json_ctx_t *c, char ch);  /* 0=ok -1=not found */
int          rf_json_parse_int(rf_json_ctx_t *c, int *out);
int          rf_json_parse_bool(rf_json_ctx_t *c, int *out);
int          rf_json_parse_double_ctx(rf_json_ctx_t *c, double *out);
/* *out_s 指向原字符串内部, *out_len = 字符数（不含引号） */
int          rf_json_parse_string_ref(rf_json_ctx_t *c, const char **out_s, int *out_len);
/* 跳过一个值(任意 JSON value: object/array/string/num/bool/null) */
int          rf_json_skip_value(rf_json_ctx_t *c);

/* ---------- 按键查找（顶层对象，非递归） ---------- */
/* 在 c 里查找 key 的值，值区间填到 value_ctx。返回 0=found -1=missing */
int rf_json_find_key(const rf_json_ctx_t *c, const char *key, rf_json_ctx_t *value_ctx);

/* ---------- 对外常用 get_* ---------- */
/* 所有函数入参 json+jlen 指一个完整的 JSON 对象字符串（可以不含外层 {}, 但对 find_key 必须是 {}） */
int      rf_json_get_int(const char *json, int jlen, const char *key, int def);
int      rf_json_get_bool(const char *json, int jlen, const char *key, int def);
double   rf_json_get_double(const char *json, int jlen, const char *key, double def);
/* 把键值复制到 out，null / 缺失 返回 -1 且 out[0]='\0' */
int      rf_json_get_cstr(const char *json, int jlen, const char *key, char *out, int out_sz);
/* 解析整型数组，返回实际填充数量；缺失则返回 0 */
int      rf_json_get_int_array(const char *json, int jlen, const char *key, int *out, int out_max);

/* ---------- 数组/子对象工具 ---------- */
/* 返回 "drones":[] 的长度；数组不存在/非数组返回 0 */
int rf_json_array_len(const char *json, int jlen, const char *key);
/* 便捷名：兼容旧调用 js_get_drone_count → rf_json_array_len(json,jlen,"drones") */
int rf_json_get_drone_count(const char *json, int jlen);

/* 把数组中第 idx 个子对象 { } 区间返回（子对象起止 = ctx->p 到 ctx->end，不含外层括号）
 * 返回 0=ok；其他=越界/不是对象 */
int rf_json_nth_object(const char *json, int jlen, const char *key, int idx, rf_json_ctx_t *sub);

/* 在一个子对象串（由 rf_json_nth_object 返回的 sub）里按 key 读字段的便捷包装 */
int      rf_json_sub_get_int(const rf_json_ctx_t *sub, const char *key, int def);
int      rf_json_sub_get_bool(const rf_json_ctx_t *sub, const char *key, int def);
double   rf_json_sub_get_double(const rf_json_ctx_t *sub, const char *key, double def);
int      rf_json_sub_get_cstr(const rf_json_ctx_t *sub, const char *key, char *out, int out_sz);

#ifdef __cplusplus
}
#endif
