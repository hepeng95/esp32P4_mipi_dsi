/*
 * rf_proto.c — 2.4G 频谱协议分析仪实现（见 rf_proto.h）。纯 C，零依赖。
 */
#include "rf_proto.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define ABS(x)  ((x) >= 0 ? (x) : -(x))

static const int16_t wifi24_centers[][2] = {
    {1,2412},{2,2417},{3,2422},{4,2427},{5,2432},
    {6,2437},{7,2442},{8,2447},{9,2452},{10,2457},
    {11,2462},{12,2467},{13,2472}
};
#define N_WIFI24 (int)(sizeof(wifi24_centers) / sizeof(wifi24_centers[0]))

static const int16_t zigbee_centers[][2] = {
    {11,2405},{12,2410},{13,2415},{14,2420},{15,2425},
    {16,2430},{17,2435},{18,2440},{19,2445},{20,2450},
    {21,2455},{22,2460},{23,2465},{24,2470},{25,2475},{26,2480}
};
#define N_ZIGBEE (int)(sizeof(zigbee_centers) / sizeof(zigbee_centers[0]))

static void find_nearest_wifi24(int mhz, int *out_ch, int *out_dist)
{
    int best_ch = -1, best_d = 9999;
    for (int i = 0; i < N_WIFI24; i++) {
        int d = ABS(mhz - (int)wifi24_centers[i][1]);
        if (d < best_d) { best_d = d; best_ch = (int)wifi24_centers[i][0]; }
    }
    *out_ch = best_ch; *out_dist = best_d;
}

static void find_nearest_zigbee(int mhz, int *out_ch, int *out_dist)
{
    int best_ch = -1, best_d = 9999;
    for (int i = 0; i < N_ZIGBEE; i++) {
        int d = ABS(mhz - (int)zigbee_centers[i][1]);
        if (d < best_d) { best_d = d; best_ch = (int)zigbee_centers[i][0]; }
    }
    *out_ch = best_ch; *out_dist = best_d;
}

static int ble_mhz_by_ch(int ble_ch)
{
    if (ble_ch <= 10) return 2404 + 2 * ble_ch;
    if (ble_ch <= 36) return 2428 + 2 * (ble_ch - 11);
    if (ble_ch == 37) return 2402;
    if (ble_ch == 38) return 2426;
    return 2480;
}

static void find_nearest_ble(int mhz, int *out_ch, int *out_dist)
{
    int best_ch = -1, best_d = 9999;
    for (int c = 0; c < 40; c++) {
        int m = ble_mhz_by_ch(c);
        int d = ABS(mhz - m);
        if (d < best_d) { best_d = d; best_ch = c; }
    }
    *out_ch = best_ch; *out_dist = best_d;
}

int rf_proto_analyze(const uint8_t rssi[RF_NUM_CHANNELS], int sweep_cnt,
                     rf_proto_hit_t *out, int out_max)
{
    if (!out || out_max <= 0 || !rssi) return 0;
    if (sweep_cnt == 0) return 0;   /* 无数据 */

    uint8_t level[RF_NUM_CHANNELS];
    for (int i = 0; i < RF_NUM_CHANNELS; i++) {
        uint32_t l = (uint32_t)rssi[i] * 100 / 255;
        level[i] = (l > 100) ? 100 : (uint8_t)l;
    }

    /* 找信号段：阈值 8，允许 2 通道间隙 */
    #define THRESH   8
    #define MIN_GAP  2
    typedef struct { int s, e, pi, pv; } seg_t;
    seg_t segs[RF_PROTOCOL_MAX_HITS];
    int seg_n = 0;

    int in_seg = 0, gap = 0;
    int seg_s = 0, seg_pi = 0, seg_pv = 0;
    for (int i = 0; i < RF_NUM_CHANNELS; i++) {
        if (level[i] >= THRESH) {
            if (!in_seg) {
                in_seg = 1; seg_s = i; seg_pi = i; seg_pv = level[i];
            } else {
                if (level[i] > seg_pv) { seg_pv = level[i]; seg_pi = i; }
            }
            gap = 0;
        } else {
            if (in_seg) {
                gap++;
                if (gap >= MIN_GAP) {
                    if (seg_n < RF_PROTOCOL_MAX_HITS) {
                        segs[seg_n].s = seg_s;
                        segs[seg_n].e = i - gap;
                        segs[seg_n].pi = seg_pi;
                        segs[seg_n].pv = seg_pv;
                        seg_n++;
                    }
                    in_seg = 0; gap = 0;
                }
            }
        }
    }
    if (in_seg && seg_n < RF_PROTOCOL_MAX_HITS) {
        segs[seg_n].s = seg_s;
        segs[seg_n].e = RF_NUM_CHANNELS - 1;
        segs[seg_n].pi = seg_pi;
        segs[seg_n].pv = seg_pv;
        seg_n++;
    }

    /* 合并邻近小段：间距 <= 8 则合并（WiFi 可能被噪声分裂） */
    {
        int w = 0;
        for (int r = 0; r < seg_n; r++) {
            if (w > 0 && segs[r].s - segs[w - 1].e <= 8) {
                if (segs[r].pv > segs[w - 1].pv) {
                    segs[w - 1].pi = segs[r].pi;
                    segs[w - 1].pv = segs[r].pv;
                }
                segs[w - 1].e = segs[r].e;
            } else {
                segs[w++] = segs[r];
            }
        }
        seg_n = w;
    }
    if (seg_n > out_max) seg_n = out_max;

    for (int k = 0; k < seg_n; k++) {
        int s = segs[k].s, e = segs[k].e;
        int bw_ch = e - s + 1;
        int ci = (s + e) / 2;
        int cmhz = 2400 + ci;

        int lo = (ci - 15 < 0) ? 0 : ci - 15;
        int hi = (ci + 15 >= RF_NUM_CHANNELS) ? RF_NUM_CHANNELS - 1 : ci + 15;
        int spread_cnt = 0;
        int eff_bw = 1;
        for (int i = lo; i <= hi; i++) {
            if (level[i] >= 4) spread_cnt++;
        }
        for (int d = 1; d <= 15; d++) {
            int ip = ci + d;
            if (ip <= hi && level[ip] >= 4) eff_bw++; else break;
        }
        for (int d = 1; d <= 15; d++) {
            int im = ci - d;
            if (im >= lo && level[im] >= 4) eff_bw++; else break;
        }

        int avg = 0;
        for (int j = s; j <= e; j++) avg += level[j];
        avg = (e - s + 1 > 0) ? avg / (e - s + 1) : 0;

        int wch, wd, zch, zd, bch, bd;
        find_nearest_wifi24(cmhz, &wch, &wd);
        find_nearest_zigbee(cmhz, &zch, &zd);
        find_nearest_ble(cmhz, &bch, &bd);

        int wide   = (spread_cnt >= 6 || eff_bw >= 5);
        int narrow = (spread_cnt <= 3 && eff_bw <= 3);
        int pv = segs[k].pv;

        uint8_t proto = 0;
        uint8_t conf  = 30;
        char lbl[64]  = {0};

        if (wide && wd <= 6) {
            proto = 1;
            conf = (pv >= 25 && wd <= 3) ? 90 : (pv >= 15 ? 75 : 55);
            snprintf(lbl, sizeof(lbl), "WiFi ch%d (%d%% bw~%d)", wch, pv, eff_bw);
        } else if (narrow && bd <= 1 && zd > 1) {
            proto = 2; conf = 75;
            snprintf(lbl, sizeof(lbl), "BLE ch%d (%d%%)", bch, pv);
        } else if (narrow && zd <= 1 && bd > 1) {
            proto = 3; conf = 75;
            snprintf(lbl, sizeof(lbl), "Zigbee ch%d (%d%%)", zch, pv);
        } else if (narrow && zd <= 1 && bd <= 1) {
            proto = 5; conf = 50;
            snprintf(lbl, sizeof(lbl), "BLE/Z %dMHz (%d%%)", cmhz, pv);
        } else if (narrow) {
            proto = 4; conf = 60;
            snprintf(lbl, sizeof(lbl), "NRF24 %dMHz (%d%%)", cmhz, pv);
        } else if (wide) {
            if (wd <= 10) {
                proto = 1; conf = 50;
                snprintf(lbl, sizeof(lbl), "WiFi? ch%d (%d%%)", wch, pv);
            } else {
                proto = 6; conf = 35;
                snprintf(lbl, sizeof(lbl), "WB intf %dMHz", cmhz);
            }
        } else {
            if (wd <= 3) {
                proto = 1; conf = 50;
                snprintf(lbl, sizeof(lbl), "WiFi? ch%d (%d%%)", wch, pv);
            } else if (bd <= 1) {
                proto = 2; conf = 45;
                snprintf(lbl, sizeof(lbl), "BLE? ch%d (%d%%)", bch, pv);
            } else if (zd <= 1) {
                proto = 3; conf = 45;
                snprintf(lbl, sizeof(lbl), "Zigbee? ch%d (%d%%)", zch, pv);
            } else {
                proto = 0; conf = 30;
                snprintf(lbl, sizeof(lbl), "? %dMHz (%d%%)", cmhz, pv);
            }
        }

        out[k].center_idx  = (int16_t)ci;
        out[k].center_mhz  = (uint16_t)cmhz;
        out[k].bw_channels = (uint8_t)bw_ch;
        out[k].peak_level  = (uint8_t)pv;
        out[k].avg_level   = (uint8_t)avg;
        out[k].confidence  = conf;
        out[k].proto_id    = proto;
        snprintf(out[k].label, sizeof(out[k].label), "%s", lbl);
    }
    return seg_n;
}
