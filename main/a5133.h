/*
 * a5133.h — A5133 5.8GHz FSK Transceiver 驱动
 *
 * 基于 A5133 Datasheet v0.7 (Preliminary), Nov. 2021
 *   https://amiccom.com.tw
 *
 * ========= SPI 协议 (Page 51-52) =========
 *   地址字节 (8 bits):
 *     Bit 7  = 0 → Control Register 访问
 *              1 → Strobe Command
 *     Bit 6  = 0 → Write
 *              1 → Read
 *     Bit[5:0] = 寄存器地址 (6 bits, 范围 0x00~0x3F)
 *   Strobe Command 格式: A7=1, A6..A4=命令, A3..A0=x (don't care)
 *
 * ========= Strobe Commands (Page 53) =========
 *   Sleep    = 0x80 (1000 xxxx)
 *   Idle     = 0x90 (1001 xxxx)
 *   Standby  = 0xA0 (1010 xxxx)
 *   PLL      = 0xB0 (1011 xxxx)
 *   RX       = 0xC0 (1100 xxxx)
 *   TX       = 0xD0 (1101 xxxx)
 *   FIFO_WRST= 0xE0 (1110 xxxx)
 *   FIFO_RRST= 0xF0 (1111 xxxx)
 *
 * ========= 关键寄存器 (Page 13-15) =========
 *   00h = Mode (W: RESETN, R: 状态标志 HECF/FECF/CRCF/CER/XER/PLLER/TRSR/TRER)
 *   01h = Mode Control (DDPC/ARSSI/AIF/DFCD/WORE/FMT/FMS/ADCM)
 *   0Eh = PLL I (CHN[7:0] — channel number)
 *   0Fh = PLL II (RRC[1:0]/IP8)
 *   10h = PLL III (BIP[7:0])
 *   11h = PLL IV (BFP[15:8])
 *   12h = PLL V  (BFP[7:0])
 *   13h = Channel Group I (CHGL[7:0])
 *   14h = Channel Group II (CHGH[7:0])
 *   1Eh = RSSI Threshold / ADC — 读 RSSI 从 ADC[7:0] (Page 14)
 *         Write: RTH[7:0]
 *         Read:  ADC[7:0] = RSSI 值 (8 bits, 约 -95 ~ -20 dBm)
 *   1Fh = ADC Control (AVSEL[1:0]/MVSEL[1:0]/RADC/FSARS/XADS/CDM)
 *
 * ========= 频率设置 =========
 *   FRXLO (RX LO) = FIF × (CHN+1)  — FIF=4MHz (4Mbps) 或 2MHz (2Mbps)
 *   CHN = (目标频率 - 5725 MHz) / CHSP - 1
 *   Channel step = 1 MHz 默认
 *   例: 5800 MHz → CHN = (5800-5725)/1 - 1 = 74 = 0x4A
 *
 * ========= PA/LNA 方向控制 (如果有外部功放) =========
 *   PA_EN = 1 → TX; LNA_EN = 1 → RX (需硬件确认)
 *
 * ========= 引脚 =========
 *   SPI:  SCK=39, SDIO=34 (MOSI/MISO 共用, 3-wire half-duplex), SCS=41
 *   CTL:  PA_EN=27, LNA_EN=10, CTR=28 (注: GPIOs 可能被模块拉低 — 需实测)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 引脚分配 ======================== */

#define A5133_PIN_SCLK       39
#define A5133_PIN_SDIO       34    /* 模块 SDIO — 3 线 SPI 双向 */
#define A5133_PIN_CS         41    /* 模块 SCS, 低有效 */

/* PA/LNA/CTR — ★ 避开 nRF24L01 SPI: MISO=4 MOSI=5 SCK=6
 *   改到 GPIO 9/10 避免冲突 (硬件若没接 PA/LNA 则无影响) */
#define A5133_PIN_PA_EN      27
#define A5133_PIN_LNA_EN     10
#define A5133_PIN_CTR        28

/* ======================== SPI 命令构造 (按数据手册) ======================== */

/* 地址字节:
 *   bit7 = 0 → 寄存器访问
 *   bit6 = 0 → 写, 1 → 读
 *   bit[5:0] = 寄存器地址 (6 bits)
 */
#define A5133_SPI_ADDR(addr, read)  ((uint8_t)(((read) ? 0x40 : 0x00) | ((addr) & 0x3F)))

/* Strobe Command:
 *   bit7 = 1, bit6..4 = 命令, bit3..0 = don't care
 */
#define A5133_STROBE_SLEEP     0x80   /* 1000 xxxx */
#define A5133_STROBE_IDLE      0x90   /* 1001 xxxx */
#define A5133_STROBE_STANDBY   0xA0   /* 1010 xxxx */
#define A5133_STROBE_PLL       0xB0   /* 1011 xxxx */
#define A5133_STROBE_RX        0xC0   /* 1100 xxxx */
#define A5133_STROBE_TX        0xD0   /* 1101 xxxx */
#define A5133_STROBE_FIFO_WRST 0xE0   /* 1110 xxxx */
#define A5133_STROBE_FIFO_RRST 0xF0   /* 1111 xxxx */

/* ======================== 寄存器地址 (按数据手册) ======================== */

#define A5133_REG_MODE          0x00   /* W=RESETN (全 0 触发软件复位), R=状态标志 */
#define A5133_REG_MODE_CTRL     0x01   /* DDPC/ARSSI/AIF/DFCD/WORE/FMT/FMS/ADCM */
#define A5133_REG_CALC          0x02
#define A5133_REG_FIFO_I        0x03   /* FEP[7:0] = FIFO end pointer */
#define A5133_REG_FIFO_II       0x04
#define A5133_REG_FIFO_DATA     0x05   /* FIFO 数据端口 */
#define A5133_REG_ID_DATA       0x06
#define A5133_REG_RC_OSC_I      0x07
#define A5133_REG_RC_OSC_II     0x08
#define A5133_REG_RC_OSC_III    0x09
#define A5133_REG_CKO_PIN       0x0A
#define A5133_REG_GIO1_PIN      0x0B
#define A5133_REG_GIO2_PIN      0x0C
#define A5133_REG_DATA_RATE     0x0D
#define A5133_REG_PLL_I         0x0E   /* CHN[7:0] — Channel Number */
#define A5133_REG_PLL_II        0x0F
#define A5133_REG_PLL_III       0x10
#define A5133_REG_PLL_IV        0x11
#define A5133_REG_PLL_V         0x12
#define A5133_REG_CHAN_GROUP_I  0x13
#define A5133_REG_CHAN_GROUP_II 0x14
#define A5133_REG_TX_I          0x15
#define A5133_REG_TX_II         0x16
#define A5133_REG_DELAY_I       0x17
#define A5133_REG_DELAY_II      0x18
#define A5133_REG_RX            0x19
#define A5133_REG_RX_GAIN_I     0x1A
#define A5133_REG_RX_GAIN_II    0x1B
#define A5133_REG_RX_GAIN_III   0x1C
#define A5133_REG_RX_GAIN_IV   0x1D
#define A5133_REG_RSSI_ADC      0x1E   /* ★ 读 RSSI: R=ADC[7:0], W=RTH[7:0] */
#define A5133_REG_ADC_CTRL      0x1F

/* ======================== 公开 API ======================== */

esp_err_t a5133_init(void);
void      a5133_deinit(void);

/* SPI 低层: 写/读 Strobe */
void      a5133_send_strobe(uint8_t strobe);

/* 寄存器读写 */
uint8_t   a5133_read_reg(uint8_t addr);
void      a5133_write_reg(uint8_t addr, uint8_t val);

/* 模式切换 — 发 Strobe 命令 */
void      a5133_set_rx(void);
void      a5133_set_tx(void);
void      a5133_set_standby(void);
void      a5133_set_sleep(void);
void      a5133_set_pll(void);
void      a5133_reset(void);

/* 频率设置: freq_mhz 目标频率 (MHz), 例 5800 */
void      a5133_set_freq_mhz(uint16_t freq_mhz);

/* RSSI 读取: 0x00~0xFF, 范围约 -95 ~ -20 dBm */
uint8_t   a5133_read_rssi_raw(void);
int8_t    a5133_read_rssi_dbm(void);

/* ======= 快速扫频通道 (给 rf_5g scan loop 用) =======
 * a5133_scan_all_channels — 批量扫描, Standby 只做 1 次, 最快 (~36ms/256ch)
 * a5133_fast_scan_channel — 单通道 (兼容旧代码)
 */
void      a5133_scan_all_channels(uint8_t *rssi_buf, int num_channels);
uint8_t   a5133_fast_scan_channel(uint8_t chn);
int8_t    a5133_rssi_to_dbm(uint8_t raw);   /* 线性映射 -95~-20 dBm */

#ifdef __cplusplus
}
#endif
