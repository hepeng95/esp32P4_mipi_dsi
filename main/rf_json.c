/*
 * rf_json.c — 极简 JSON 查找/解析工具实现（见 rf_json.h）
 * 纯 C99，仅依赖 <string.h>/<stdlib.h>/<ctype.h>/<stdint.h>/<stddef.h>
 */
#include "rf_json.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>

#ifndef RF_JSON_MIN
#define RF_JSON_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

void rf_json_skip_ws(rf_json_ctx_t *c)
{
    while (c->p < c->end && (*c->p == ' ' || *c->p == '\t' || *c->p == '\r' || *c->p == '\n')) c->p++;
}

int rf_json_expect(rf_json_ctx_t *c, char ch)
{
    rf_json_skip_ws(c);
    if (c->p >= c->end || *c->p != ch) return -1;
    c->p++;
    return 0;
}

int rf_json_parse_int(rf_json_ctx_t *c, int *out)
{
    rf_json_skip_ws(c);
    if (c->p >= c->end) return -1;
    int sign = 1;
    if (*c->p == '-') { sign = -1; c->p++; }
    else if (*c->p == '+') { c->p++; }
    if (c->p >= c->end || !isdigit((unsigned char)*c->p)) return -1;
    long v = 0;
    while (c->p < c->end && isdigit((unsigned char)*c->p)) {
        v = v * 10 + (*c->p - '0');
        c->p++;
    }
    *out = (int)(sign * v);
    return 0;
}

int rf_json_parse_bool(rf_json_ctx_t *c, int *out)
{
    rf_json_skip_ws(c);
    if (c->end - c->p >= 4 && strncmp(c->p, "true", 4) == 0) { c->p += 4; *out = 1; return 0; }
    if (c->end - c->p >= 5 && strncmp(c->p, "false", 5) == 0) { c->p += 5; *out = 0; return 0; }
    return -1;
}

int rf_json_parse_double_ctx(rf_json_ctx_t *c, double *out)
{
    rf_json_skip_ws(c);
    if (c->p >= c->end) return -1;
    const char *p = c->p;
    if (c->end - p >= 4 && strncmp(p, "null", 4) == 0) { c->p = p + 4; return -1; }
    int neg = 0;
    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') p++;
    double v = 0;
    int saw_digit = 0;
    while (p < c->end && *p >= '0' && *p <= '9') {
        v = v * 10.0 + (*p - '0');
        p++; saw_digit = 1;
    }
    if (p < c->end && *p == '.') {
        p++;
        double frac = 0.1;
        while (p < c->end && *p >= '0' && *p <= '9') {
            v += frac * (*p - '0');
            frac *= 0.1;
            p++; saw_digit = 1;
        }
    }
    if (saw_digit && p < c->end && (*p == 'e' || *p == 'E')) {
        p++;
        int eneg = 0;
        if (p < c->end && *p == '-') { eneg = 1; p++; }
        else if (p < c->end && *p == '+') p++;
        int expv = 0;
        while (p < c->end && *p >= '0' && *p <= '9') { expv = expv * 10 + (*p - '0'); p++; }
        double mul = 1.0;
        if (eneg) while (expv-- > 0) mul *= 0.1;
        else      while (expv-- > 0) mul *= 10.0;
        v *= mul;
    }
    c->p = p;
    if (!saw_digit) return -1;
    *out = neg ? -v : v;
    return 0;
}

int rf_json_parse_string_ref(rf_json_ctx_t *c, const char **out_s, int *out_len)
{
    rf_json_skip_ws(c);
    if (c->p >= c->end || *c->p != '"') return -1;
    c->p++;
    const char *start = c->p;
    while (c->p < c->end && *c->p != '"') {
        if (*c->p == '\\' && c->p + 1 < c->end) c->p++;
        c->p++;
    }
    if (c->p >= c->end) return -1;
    *out_s = start;
    *out_len = (int)(c->p - start);
    c->p++;
    return 0;
}

int rf_json_skip_value(rf_json_ctx_t *c)
{
    rf_json_skip_ws(c);
    if (c->p >= c->end) return -1;
    char ch = *c->p;
    if (ch == '"') {
        const char *s; int l;
        return rf_json_parse_string_ref(c, &s, &l);
    } else if (ch == '{' || ch == '[') {
        char open = ch, close = (ch == '{') ? '}' : ']';
        int depth = 1;
        c->p++;
        while (c->p < c->end && depth > 0) {
            if (*c->p == '"') {
                c->p++;
                while (c->p < c->end && *c->p != '"') {
                    if (*c->p == '\\' && c->p + 1 < c->end) c->p++;
                    c->p++;
                }
                if (c->p < c->end) c->p++;
            } else if (*c->p == open) { depth++; c->p++; }
            else if (*c->p == close) { depth--; c->p++; }
            else c->p++;
        }
        return (depth == 0) ? 0 : -1;
    } else {
        while (c->p < c->end && *c->p != ',' && *c->p != ']' && *c->p != '}') c->p++;
        return 0;
    }
}

int rf_json_find_key(const rf_json_ctx_t *c, const char *key, rf_json_ctx_t *value_ctx)
{
    rf_json_ctx_t root = *c;
    rf_json_skip_ws(&root);
    if (rf_json_expect(&root, '{') != 0) return -1;
    rf_json_skip_ws(&root);
    if (root.p < root.end && *root.p == '}') { root.p++; return -1; }
    size_t klen = strlen(key);
    while (1) {
        const char *ks; int kl;
        if (rf_json_parse_string_ref(&root, &ks, &kl) != 0) return -1;
        rf_json_skip_ws(&root);
        if (root.p >= root.end || *root.p != ':') return -1;
        root.p++;
        if ((size_t)kl == klen && strncmp(ks, key, klen) == 0) {
            *value_ctx = root;
            return 0;
        } else {
            if (rf_json_skip_value(&root) != 0) return -1;
        }
        rf_json_skip_ws(&root);
        if (root.p >= root.end) return -1;
        if (*root.p == ',') { root.p++; continue; }
        if (*root.p == '}') { root.p++; return -1; }
        return -1;
    }
}

/* ================= 顶层 get 系列 ============================================ */

int rf_json_get_int(const char *json, int jlen, const char *key, int def)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return def;
    int v = def;
    if (rf_json_parse_int(&vctx, &v) != 0) return def;
    return v;
}

int rf_json_get_bool(const char *json, int jlen, const char *key, int def)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return def;
    int v;
    if (rf_json_parse_bool(&vctx, &v) == 0) return v;
    if (rf_json_parse_int(&vctx, &v) == 0) return (v ? 1 : 0);
    return def;
}

double rf_json_get_double(const char *json, int jlen, const char *key, double def)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return def;
    double v = def;
    if (rf_json_parse_double_ctx(&vctx, &v) != 0) return def;
    return v;
}

int rf_json_get_cstr(const char *json, int jlen, const char *key, char *out, int out_sz)
{
    if (!out || out_sz <= 0) return -1;
    out[0] = '\0';
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return -1;
    const char *s; int sl;
    if (rf_json_parse_string_ref(&vctx, &s, &sl) != 0) return -1;
    int n = RF_JSON_MIN(sl, out_sz - 1);
    if (n > 0) memcpy(out, s, (size_t)n);
    out[n] = '\0';
    return n;
}

int rf_json_get_int_array(const char *json, int jlen, const char *key, int *out, int out_max)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return 0;
    rf_json_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return 0;
    vctx.p++;
    int cnt = 0;
    while (out && cnt < out_max) {
        rf_json_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        if (*vctx.p == ']') { vctx.p++; break; }
        int val;
        if (rf_json_parse_int(&vctx, &val) != 0) break;
        out[cnt++] = val;
        rf_json_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        if (*vctx.p == ',') { vctx.p++; continue; }
        if (*vctx.p == ']') { vctx.p++; break; }
        break;
    }
    return cnt;
}

/* ================= 数组 / 子对象工具 ======================================== */

int rf_json_array_len(const char *json, int jlen, const char *key)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return 0;
    rf_json_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return 0;
    vctx.p++;
    int depth = 1, count = 0, in_obj = 0;
    while (vctx.p < vctx.end && depth > 0) {
        char ch = *vctx.p;
        if (ch == '"') {
            vctx.p++;
            while (vctx.p < vctx.end && *vctx.p != '"') {
                if (*vctx.p == '\\' && vctx.p + 1 < vctx.end) vctx.p++;
                vctx.p++;
            }
            if (vctx.p < vctx.end) vctx.p++;
            continue;
        }
        if (ch == '{') { depth++; in_obj++; vctx.p++; continue; }
        if (ch == '}') {
            depth--;
            if (in_obj > 0) { in_obj--; if (in_obj == 0) count++; }
            vctx.p++; continue;
        }
        if (ch == '[') { depth++; vctx.p++; continue; }
        if (ch == ']') { depth--; vctx.p++; continue; }
        vctx.p++;
    }
    return count;
}

int rf_json_get_drone_count(const char *json, int jlen)
{
    return rf_json_array_len(json, jlen, "drones");
}

/* 返回对象内容的起止（剥掉外层 {}），填入 *sub */
static int skip_brace_object(rf_json_ctx_t *ctx, rf_json_ctx_t *sub)
{
    rf_json_skip_ws(ctx);
    if (ctx->p >= ctx->end || *ctx->p != '{') return -1;
    const char *s = ctx->p + 1;  /* 跳 { */
    int obj_depth = 1;
    int in_str = 0;
    ctx->p++;
    while (ctx->p < ctx->end) {
        char cc = *ctx->p;
        if (cc == '"') { in_str = !in_str; ctx->p++; continue; }
        if (in_str) { if (cc == '\\' && ctx->p + 1 < ctx->end) ctx->p++; ctx->p++; continue; }
        if (cc == '{') { obj_depth++; ctx->p++; continue; }
        if (cc == '}') {
            obj_depth--;
            ctx->p++;
            if (obj_depth == 0) {
                sub->p = s;
                sub->end = ctx->p - 1;  /* 跳尾 } */
                return 0;
            }
            continue;
        }
        ctx->p++;
    }
    return -1;
}

int rf_json_nth_object(const char *json, int jlen, const char *key, int idx, rf_json_ctx_t *sub)
{
    rf_json_ctx_t root = { json, json + jlen };
    rf_json_ctx_t vctx;
    if (rf_json_find_key(&root, key, &vctx) != 0) return -1;
    rf_json_skip_ws(&vctx);
    if (vctx.p >= vctx.end || *vctx.p != '[') return -1;
    vctx.p++;
    int i = 0;
    while (vctx.p < vctx.end) {
        rf_json_skip_ws(&vctx);
        if (vctx.p >= vctx.end) break;
        char ch = *vctx.p;
        if (ch == ']') return -1;
        if (ch == '{') {
            rf_json_ctx_t obj_ctx;
            if (skip_brace_object(&vctx, &obj_ctx) != 0) return -1;
            if (i == idx) { *sub = obj_ctx; return 0; }
            i++;
            continue;
        }
        /* 非对象值，跳过直到 , 或 ] */
        if (ch == ',') vctx.p++;
        else if (rf_json_skip_value(&vctx) != 0) return -1;
    }
    return -1;
}

/* ================ 子对象包装 API（调用方传入 sub -> 先包装 {} 再按键查） ===== */
#define RF_JSON_SUB_TMP_SZ 2048

static int sub_wrap(const rf_json_ctx_t *sub, char *tmp, int tmp_sz)
{
    int sl = (int)(sub->end - sub->p);
    if (sl < 0) sl = 0;
    int tl = sl + 2;
    if (tl > tmp_sz) tl = tmp_sz;
    if (tl < 2) tl = 2;
    tmp[0] = '{';
    int cplen = tl - 2;
    if (cplen > 0) memcpy(tmp + 1, sub->p, (size_t)cplen);
    tmp[tl - 1] = '}';
    return tl;
}

int rf_json_sub_get_int(const rf_json_ctx_t *sub, const char *key, int def)
{
    char tmp[RF_JSON_SUB_TMP_SZ];
    int tl = sub_wrap(sub, tmp, (int)sizeof(tmp));
    return rf_json_get_int(tmp, tl, key, def);
}

int rf_json_sub_get_bool(const rf_json_ctx_t *sub, const char *key, int def)
{
    char tmp[RF_JSON_SUB_TMP_SZ];
    int tl = sub_wrap(sub, tmp, (int)sizeof(tmp));
    return rf_json_get_bool(tmp, tl, key, def);
}

double rf_json_sub_get_double(const rf_json_ctx_t *sub, const char *key, double def)
{
    char tmp[RF_JSON_SUB_TMP_SZ];
    int tl = sub_wrap(sub, tmp, (int)sizeof(tmp));
    return rf_json_get_double(tmp, tl, key, def);
}

int rf_json_sub_get_cstr(const rf_json_ctx_t *sub, const char *key, char *out, int out_sz)
{
    char tmp[RF_JSON_SUB_TMP_SZ];
    int tl = sub_wrap(sub, tmp, (int)sizeof(tmp));
    return rf_json_get_cstr(tmp, tl, key, out, out_sz);
}
