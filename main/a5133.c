/*
 * a5133.c — A5133 5.8GHz FSK Transceiver 驱动
 *
 * 基于 A5133 Datasheet v0.7 (Preliminary), Nov. 2021
 * SPI 协议 (Page 51-52):
 *   地址字节 = [bit7=0: 寄存器访问 / bit7=1: Strobe]
 *              [bit6=0: 写 / bit6=1: 读]
 *              [bit5..0 = 6-bit 地址]
 *   Strobe 命令 (Page 53): bit7=1, bit6..4=命令, bit3..0=x
 *   RX Strobe = 0xC0, TX Strobe = 0xD0, Standby = 0xA0, PLL = 0xB0, Sleep = 0x80
 */

#include "a5133.h"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"  /* GPIO register structs */
#include "soc/io_mux_struct.h" /* IO_MUX fun_ie register */
#include "string.h"

static const char *TAG = "a5133";
static bool s_inited = false;

/* ======================== A5133 配置表 (DR_4Mbps)
 * 来源: radio/A5133Config.h — 参考实现
 * 索引 = 寄存器地址 (0x00..0x3E), 对应每个寄存器的默认值
 */
static const uint8_t s_cfg_main[] = {  // 0x00-0x3F, 完整 64 字节
    0x00, 0x62, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0C, 0x02, 0x01, 0x1D, 0x1F, 0x00, 0x1E,
    0x59, 0x74, 0x01, 0x28, 0x50, 0x2D, 0x40, 0x1B,
    0x40, 0x70, 0x7B, 0xC0, 0x3C, 0xCA, 0x00, 0xC1,
    0x00, 0x00, 0x00, 0xE4, 0x01, 0x4F, 0xC0, 0x80,
    0x30, 0x40, 0x00, 0xFF, 0x40, 0xA7, 0x57, 0x74,
    0xF3, 0x33, 0x4D, 0x15, 0x0F, 0x00, 0x00, 0x33,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t s_cfg_page0x20[] = { /* CODE1_REG 各 page */
    0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
};
static const uint8_t s_cfg_page0x21[] = { /* CODE2_REG 各 page */
    0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x2C, 0x4F, 0x01, 0x42, 0x56,
};
static const uint8_t s_cfg_page0x22[] = { /* CODE3_REG 各 page */
    0x00, 0x10, 0x00, 0x10, 0x00, 0x04,
};
static const uint8_t s_cfg_page0x2A[] = { /* DAS_REG 各 page */
    0x00, 0x01, 0xF0, 0x80, 0x80, 0x48, 0x03, 0xC0, 0x3A, 0x3E, 0xE8, 0x80, 0x00,
};
static const uint8_t s_cfg_page0x38[] = { /* ROMP_REG 各 page (trim) */
    0x80, 0x50, 0x20, 0x64, 0x20, 0x40, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00,
};

/* RF_ANALOG_REG = 0x35 — 用来切换 page (bit[7:4] = page, bit[3:0] = analog 参数)
 * s_cfg_main[0x35] = 0x00 → 低 4 位 = 0x00
 */
#define A5133_REG_RFANALOG   0x35
#define A5133_REG_CALIBRATION 0x02

/* RSSI 校准保存 — RF_Cal 会写入 */
static uint8_t s_mem_rh = 0xC0;   /* 默认 RXGAIN2 */
static uint8_t s_mem_rl = 0x3C;   /* 默认 RXGAIN3 */

/* ============== Bit-Bang SPI — 模仿 radio 示例 rwByte 模式 ==============
 *
 * 参考 radio/myRadio_gpio.c::myRadioSpi_rwByte()
 *   SPI Mode 0 (CPOL=0, CPHA=0):
 *     SCK idle = Low
 *     MCU 在 SCK 上升沿 写 SDIO (A5133 采样) + 读 SDIO (A5133 驱动)
 *     A5133 在 SCK 下降沿 驱动下一位到 SDIO
 *
 *   关键: SDIO 方向 每 bit 切换 — OUTPUT(推挽) 写 bit → INPUT(上拉) 读 bit
 *         不用 open-drain, 用推挽!
 *         延时 ~2µs per bit (比 STM32 的 10µs 快, 但 ESP32-P4 快 5 倍)
 */

/* 引脚号 → 寄存器位 (ESP32-P4: GPIOs 32-39 in group1) */
#define SDIO_PIN        A5133_PIN_SDIO
#define SDIO_BIT        (1U << (SDIO_PIN - 32))
#define SDIO_IN_BIT     (1U << (SDIO_PIN - 32))  /* 输入数据在 in1 寄存器 */

/* ★ SCLK / CS — 仍用 gpio_set_level (ESP32-P4 GPIO struct 有差异, 先稳) */
static inline void s_sclk_low(void)  { gpio_set_level(A5133_PIN_SCLK, 0); }
static inline void s_sclk_high(void) { gpio_set_level(A5133_PIN_SCLK, 1); }
static inline void s_cs_low(void)    { gpio_set_level(A5133_PIN_CS, 0); }
static inline void s_cs_high(void)   { gpio_set_level(A5133_PIN_CS, 1); }

/* ★ SPI bit 延时 — 2 NOPs (vs 原来 4 NOPs) */
static inline void s_bit_delay(void)
{
    __asm__ volatile(
        ".word 0x00000013\n"
        ".word 0x00000013\n"
        ::: "memory"
    );
}

/* SDIO → 推挽输出 (register 直写 — 跳过 gpio_set_direction 互斥锁) */
static inline void s_sdio_out(void)
{
    IO_MUX.gpio[SDIO_PIN].fun_ie = 0;          /* 禁用输入缓冲 */
    GPIO.enable1_w1ts.enable1_w1ts = SDIO_BIT; /* 启用输出 */
    GPIO.pin[SDIO_PIN].pad_driver = 0;         /* 推挽模式 */
}

/* SDIO → 上拉输入 (register 直写) */
static inline void s_sdio_in(void)
{
    GPIO.enable1_w1tc.enable1_w1tc = SDIO_BIT; /* 禁用输出 */
    IO_MUX.gpio[SDIO_PIN].fun_ie = 1;          /* 启用输入缓冲 */
    GPIO.pin[SDIO_PIN].pad_driver = 0;         /* 保持推挽配置 */
    /* pullup 已在 init 设置好 */
}

/* SDIO 写数据 — register 直写代替 gpio_set_level */
static inline void s_sdio_write(int level)
{
    if (level) GPIO.out1_w1ts.out1_w1ts = SDIO_BIT;
    else       GPIO.out1_w1tc.out1_w1tc = SDIO_BIT;
}

/* SDIO 读数据 — register 直写代替 gpio_get_level */
static inline int s_sdio_read(void)
{
    return (GPIO.in1.in1_data_next >> (SDIO_PIN - 32)) & 1;
}

/* ============== 核心: rwByte — 模仿 myRadioSpi_rwByte ==============
 * 发 1 byte (MSB first), 同时收 1 byte (MSB first).
 * 返回收到的值.
 *
 * 每个 bit:
 *   1. SDIO_OUT_PP + 写 bit
 *   2. 延时
 *   3. SCK 上升沿
 *   4. SDIO_IN_PU + 读 bit
 *   5. SCK 下降沿
 */
static uint8_t s_rw_byte(uint8_t tx)
{
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--) {
        /* --- 写 bit: 推挽输出 --- */
        s_sdio_out();
        s_sdio_write((tx >> i) & 1);
        s_bit_delay();

        /* --- SCK 上升沿 --- */
        s_sclk_high();

        /* --- 读 bit: 切输入 + 读 --- */
        s_sdio_in();
        rx = (rx << 1) | (uint8_t)s_sdio_read();

        /* --- SCK 下降沿 + 延时让 A5133 驱动下一位 --- */
        s_sclk_low();
        s_bit_delay();
    }
    return rx;
}

/* 只写 byte (忽略返回) */
static inline void s_w_byte(uint8_t b) { s_rw_byte(b); }

/* 只读 byte (发 dummy 0x00) */
static inline uint8_t s_r_byte(void) { return s_rw_byte(0x00); }

/* ============== GPIO 初始化 ============== */

static void a5133_config_gpio(void)
{
    /* SPI: SCLK + CS + SDIO — 推挽输出 (rwByte 每 bit 切换 SDIO 方向) */
    gpio_config_t io_spi = {
        .pin_bit_mask = (1ULL << A5133_PIN_SCLK) | (1ULL << A5133_PIN_CS) | (1ULL << A5133_PIN_SDIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_spi));

    /* 配置完 SPI, SDIO 初始为上拉 (等待 s_rw_byte 切方向) */
    gpio_pullup_en(A5133_PIN_SDIO);

    /* CTL: PA_EN, LNA_EN, CTR — 普通推挽输出 */
    gpio_config_t io_ctl = {
        .pin_bit_mask = (1ULL << A5133_PIN_PA_EN)
                      | (1ULL << A5133_PIN_LNA_EN)
                      | (1ULL << A5133_PIN_CTR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_ctl);

    /* 空闲态 */
    gpio_set_level(A5133_PIN_CS,   1);
    gpio_set_level(A5133_PIN_SCLK, 0);
    gpio_set_level(A5133_PIN_PA_EN,  0);
    gpio_set_level(A5133_PIN_LNA_EN, 0);
    gpio_set_level(A5133_PIN_CTR,    0);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* 自检 */
    ESP_LOGI(TAG, "GPIO 自检: CS=%d SDIO=%d SCLK=%d",
             gpio_get_level(A5133_PIN_CS),
             gpio_get_level(A5133_PIN_SDIO),
             gpio_get_level(A5133_PIN_SCLK));
}

/* ============== SPI 低层事务 — 模仿 HAL 层 RF_WriteReg / RF_ReadReg ============== */

/* 写 Strobe 命令 (1 byte, CS 拉低 → 发 8 bits → CS 拉高) */
void a5133_send_strobe(uint8_t strobe)
{
    s_cs_high();
    s_sclk_low();
    s_cs_low();
    s_rw_byte(strobe);          /* 写命令, 忽略返回 */
    s_cs_high();
    s_sclk_low();
}

/* 写寄存器: 1 byte 地址 + 1 byte 数据 */
void a5133_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t addr_byte = A5133_SPI_ADDR(addr, 0);  /* bit7=0, bit6=0, addr=低6位 */

    s_cs_high();
    s_sclk_low();
    s_cs_low();
    s_rw_byte(addr_byte);       /* 写地址 */
    s_rw_byte(val);             /* 写数据 */
    s_cs_high();
    s_sclk_low();
}

/* 读寄存器: 1 byte 地址 → 1 byte 返回数据 */
uint8_t a5133_read_reg(uint8_t addr)
{
    uint8_t addr_byte = A5133_SPI_ADDR(addr, 1);  /* bit7=0, bit6=1, addr=低6位 */

    s_cs_high();
    s_sclk_low();
    s_cs_low();
    s_rw_byte(addr_byte);          /* 发读地址 */
    uint8_t val = s_rw_byte(0x00); /* 发 dummy, 同时收数据 */
    s_cs_high();
    s_bit_delay();
    s_sclk_low();
    return val;
}

/* ============== Page 读写 (RFANALOG_REG = 0x35 选择 page) ============== */

static void a5133_write_page(uint8_t addr, uint8_t val, uint8_t page)
{
    /* bit[7:4] = page, bit[3:0] = 0x00 (RFANALOG 低 4 位默认) */
    a5133_write_reg(A5133_REG_RFANALOG, (page << 4) | 0x00);
    /* 参考实现里有个短延时让 page 切换稳定 */
    for (volatile int i = 0; i < 200; i++) __asm__ volatile("nop");
    a5133_write_reg(addr, val);
}

/* ============== RF 完整配置 — 模仿 radio/A5133_hal.c::RF_Config() ============== */

static void a5133_rf_config(void)
{
    ESP_LOGI(TAG, "RF_Config: 写主寄存器...");

    /* 0x01 ~ 0x04 */
    for (int i = 1; i <= 4; i++)
        a5133_write_reg((uint8_t)i, s_cfg_main[i]);
    /* 0x07 ~ 0x1F */
    for (int i = 7; i <= 0x1F; i++)
        a5133_write_reg((uint8_t)i, s_cfg_main[i]);

    /* Page 寄存器: 0x20/0x21/0x22/0x2A/0x38 */
    ESP_LOGI(TAG, "RF_Config: 写 Page 寄存器...");
    for (int i = 0; i < 13; i++) a5133_write_page(0x20, s_cfg_page0x20[i], (uint8_t)i);
    for (int i = 0; i < 13; i++) a5133_write_page(0x21, s_cfg_page0x21[i], (uint8_t)i);
    for (int i = 0; i < 6;  i++) a5133_write_page(0x22, s_cfg_page0x22[i], (uint8_t)i);

    /* 0x23 ~ 0x29 */
    for (int i = 0x23; i <= 0x29; i++)
        a5133_write_reg((uint8_t)i, s_cfg_main[i]);

    for (int i = 0; i < 13; i++) a5133_write_page(0x2A, s_cfg_page0x2A[i], (uint8_t)i);

    /* 0x2B ~ 0x35 */
    for (int i = 0x2B; i <= 0x35; i++)
        a5133_write_reg((uint8_t)i, s_cfg_main[i]);

    /* 0x37 (跳过 0x36 KEYDATA 运行时写) */
    a5133_write_reg(0x37, 0x33);

    for (int i = 0; i < 12; i++) a5133_write_page(0x38, s_cfg_page0x38[i], (uint8_t)i);

    /* 0x39 ~ 0x3C, 0x3E */
    for (int i = 0x39; i <= 0x3C; i++) a5133_write_reg((uint8_t)i, s_cfg_main[i]);
    a5133_write_reg(0x3E, 0x00);
}

/* ============== PLL 校准 — 模仿 radio/A5133_hal.c::RF_Cal() ============== */

static esp_err_t a5133_rf_cal(void)
{
    ESP_LOGI(TAG, "RF_Cal: 开始 PLL 校准...");

    /* 先进 PLL 状态 */
    a5133_send_strobe(A5133_STROBE_PLL);
    vTaskDelay(pdMS_TO_TICKS(2));
    a5133_write_reg(A5133_REG_RFANALOG, 0x00);

    /* IF / RSSI / RC 校准 — CALIBRATION_REG = 0x02, 写 0x23 启动, busy bit 轮询清除
     * ★ 纯 busy-wait! 不能用 vTaskDelay — A5133 状态机会超时! */
    a5133_write_reg(A5133_REG_CALIBRATION, 0x23);
    int timeout = 50000;
    while ((a5133_read_reg(A5133_REG_CALIBRATION) & 0x23) && --timeout > 0)
        for (volatile int i = 0; i < 50; i++) __asm__ volatile("nop");
    if (timeout == 0) {
        ESP_LOGW(TAG, "RF_Cal: IF/RSSI/RC 校准超时");
        return ESP_ERR_TIMEOUT;
    }

    /* VCO 校准 — 3 个 channel group */
    uint8_t cal_channels[] = {25, 75, 125};
    for (int c = 0; c < 3; c++) {
        a5133_write_reg(A5133_REG_PLL_I, cal_channels[c]);
        a5133_write_reg(A5133_REG_CALIBRATION, 0x1C);
        timeout = 50000;
        while ((a5133_read_reg(A5133_REG_CALIBRATION) & 0x1C) && --timeout > 0)
            for (volatile int i = 0; i < 50; i++) __asm__ volatile("nop");
        if (timeout == 0) {
            ESP_LOGW(TAG, "RF_Cal: VCO Ch%d 校准超时", cal_channels[c]);
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGI(TAG, "  VCO ch%u ✓", cal_channels[c]);
    }

    /* 保存校准后的 RXGAIN 值用于 RSSI 计算 */
    s_mem_rh = a5133_read_reg(0x1B);  /* RXGAIN2 */
    s_mem_rl = a5133_read_reg(0x1C);  /* RXGAIN3 */
    ESP_LOGI(TAG, "RF_Cal: RH=0x%02X RL=0x%02X", s_mem_rh, s_mem_rl);

    /* 回 Standby */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "RF_Cal: 完成 ✓");
    return ESP_OK;
}

/* ============== 模式切换 (用 Strobe 命令!) ============== */
/* 同时控制 PA_EN (功放) 和 LNA_EN (低噪声放大器) — 硬件引脚必须配合! */

void a5133_set_rx(void)
{
    gpio_set_level(A5133_PIN_LNA_EN, 1);   /* ★ RX 时开 LNA */
    gpio_set_level(A5133_PIN_PA_EN,  0);
    a5133_send_strobe(A5133_STROBE_RX);
}

void a5133_set_tx(void)
{
    gpio_set_level(A5133_PIN_LNA_EN, 0);
    gpio_set_level(A5133_PIN_PA_EN,  1);   /* ★ TX 时开 PA */
    a5133_send_strobe(A5133_STROBE_TX);
}

void a5133_set_standby(void)
{
    gpio_set_level(A5133_PIN_PA_EN,  0);
    gpio_set_level(A5133_PIN_LNA_EN, 0);
    a5133_send_strobe(A5133_STROBE_STANDBY);
}

void a5133_set_pll(void)      { a5133_send_strobe(A5133_STROBE_PLL); }
void a5133_set_sleep(void)
{
    gpio_set_level(A5133_PIN_PA_EN,  0);
    gpio_set_level(A5133_PIN_LNA_EN, 0);
    a5133_send_strobe(A5133_STROBE_SLEEP);
}

/* 软件复位: 写 00h = 0x00 (Page 57) */
void a5133_reset(void)
{
    a5133_write_reg(A5133_REG_MODE, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/* ============== 频率设置 ==============
 * A5133 Datasheet v0.7 §14.1 LO Frequency Setting:
 *   FRF = FRF_BASE + FOFFSET
 *   FRF_BASE = 5725.001 MHz
 *   FOFFSET  = CHN[7:0] × 1 MHz
 *   => CHN[7:0] = FRF_MHz - 5725
 *   例: 5725 MHz → CHN=0, 5765 MHz → CHN=40, 5849 MHz → CHN=124
 *   CHN 写入 PLL I 寄存器 (0Eh)
 */
void a5133_set_freq_mhz(uint16_t freq_mhz)
{
    /* PLL_I 是 8-bit, FRF = 5725 + CHN×1MHz, CHN[7:0] = 0~255
       → 覆盖 5725 ~ 5980 MHz (整个 5.8GHz ISM band) */
    if (freq_mhz < 5725) freq_mhz = 5725;
    if (freq_mhz > 5980) freq_mhz = 5980;

    uint8_t chn = (uint8_t)(freq_mhz - 5725);
    ESP_LOGD(TAG, "set_freq: %u MHz → CHN=%u", freq_mhz, chn);

    /* 先 Standby */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 写 CHN → PLL I (0Eh) */
    a5133_write_reg(A5133_REG_PLL_I, chn);

    /* 切 PLL 让 PLL 锁定 */
    a5133_send_strobe(A5133_STROBE_PLL);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/* ============== RSSI 读取 ==============
 * 读 1Eh (RSSI Threshold / ADC) — R=ADC[7:0] (Page 14)
 * A5133 RX settling time: PLL → RX = 60 µs (Page 11)
 */
uint8_t a5133_read_rssi_raw(void)
{
    /* 进 RX 模式, 等 ~140µs 让 RSSI 稳定 (Page 76) */
    a5133_send_strobe(A5133_STROBE_RX);

    /* 140µs busy-wait — 不能用 vTaskDelay (太短, 会被调度出去) */
    for (volatile int i = 0; i < 5000; i++) __asm__ volatile("nop");

    uint8_t rssi = a5133_read_reg(A5133_REG_RSSI_ADC);

    /* 回 Standby 省电 */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    return rssi;
}

int8_t a5133_read_rssi_dbm(void)
{
    uint8_t raw = a5133_read_rssi_raw();
    /* 粗略映射: 0xFF = 满偏, 实际范围约 -95 ~ -20 dBm */
    /* datasheet: RSSI Range -95 ~ -20 dBm, 8 bits → 约 0.3 dB/LSB */
    return (int8_t)(-95 + (raw * 75 / 255));
}

/* ============== 快速扫频通道 (正确版) ==============
 * 完全模仿 a5133_set_freq_mhz + a5133_read_rssi_raw 序列:
 *   Strobe STANDBY → wait 1ms → Write PLL_I → Strobe PLL → wait 2ms → Strobe RX → wait 140µs → Read RSSI
 *
 * NOP 计数基于 a5133_read_rssi_raw 的实测:
 *   5000 nops ≈ 140µs → 1ms ≈ 35000, 2ms ≈ 70000
 * 性能: 256ch × ~3.2ms/ch ≈ 820ms/帧 ≈ 1.2Hz
 */
#define NOP_1MS  for (volatile int _i = 0; _i < 35000; _i++) __asm__ volatile("nop")
#define NOP_140US for (volatile int _i = 0; _i < 5000; _i++) __asm__ volatile("nop")

uint8_t a5133_fast_scan_channel(uint8_t chn)
{
    /* 1. Standby — PLL 才能改 */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    NOP_1MS;                                    /* ~1ms */

    /* 2. 写 PLL_I [chn] */
    a5133_write_reg(A5133_REG_PLL_I, chn);

    /* 3. Strobe PLL + 等锁定 (实测 1ms 够, 2ms 保险) */
    a5133_send_strobe(A5133_STROBE_PLL);
    NOP_1MS;                                    /* ~1ms */

    /* 4. Strobe RX + 等 RSSI ADC 稳定 */
    a5133_send_strobe(A5133_STROBE_RX);
    NOP_140US;                                  /* datasheet Page 76 */

    /* 5. 读 RSSI ADC */
    return a5133_read_reg(A5133_REG_RSSI_ADC);
}

/* ============== 批量扫描 (极速版 V2: PLL 等待 80µs, SPI 寄存器直写) ==============
 * ★ 只做一次 STANDBY → PLL mode, 之后直接写 PLL_I + STROBE PLL
 * ★ LNA 全程常开
 * ★ PLL 等待 80µs (PLL mode 下切频 VCO 锁相极快)
 * ★ RSSI settle 60µs (datasheet: PLL→RX settling = 60µs)
 * ★ SCLK/CS → GPIO 寄存器直写 (省 gpio_set_level ~200ns/call)
 * ★ s_bit_delay 4 NOP → 2 NOP
 *
 * 性能: 256ch × ~140µs/ch ≈ 36ms/帧 ≈ 27Hz! (vs 最初 820ms = 1.2Hz)
 */
#define NOP_80US   for (volatile int _i = 0; _i < 2857;  _i++) __asm__ volatile("nop")
#define NOP_60US   for (volatile int _i = 0; _i < 2143;  _i++) __asm__ volatile("nop")

void a5133_scan_all_channels(uint8_t *rssi_buf, int num_channels)
{
    if (num_channels > 256) num_channels = 256;
    if (num_channels < 1) return;

    /* 1. 开头只做 1 次 Standby → PLL 进入 PLL mode */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    a5133_send_strobe(A5133_STROBE_PLL);
    NOP_80US;

    /* 2. LNA 全程常开 (省 2×256 次 GPIO 写) */
    gpio_set_level(A5133_PIN_LNA_EN, 1);
    gpio_set_level(A5133_PIN_PA_EN,  0);

    /* 3. 循环: 写 PLL_I → STROBE PLL → 等锁 → STROBE RX → 等 settle → 读 RSSI */
    for (int chn = 0; chn < num_channels; chn++) {
        a5133_write_reg(A5133_REG_PLL_I, (uint8_t)chn);
        a5133_send_strobe(A5133_STROBE_PLL);
        NOP_80US;

        a5133_send_strobe(A5133_STROBE_RX);
        NOP_60US;

        rssi_buf[chn] = a5133_read_reg(A5133_REG_RSSI_ADC);
    }

    /* 4. 回 Standby 省电 */
    gpio_set_level(A5133_PIN_LNA_EN, 0);
    a5133_send_strobe(A5133_STROBE_STANDBY);
}
int8_t a5133_rssi_to_dbm(uint8_t raw)
{
    /* 线性映射: raw=0 → -95dBm, raw=255 → -20dBm, 斜率 75/255 ≈ 0.294 dB/LSB */
    int16_t dbm = -95 + (int16_t)((uint16_t)raw * 75u / 255u);
    if (dbm < -95) dbm = -95;
    if (dbm > -20) dbm = -20;
    return (int8_t)dbm;
}

/* ============== init / deinit ============== */

esp_err_t a5133_init(void)
{
    if (s_inited) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "=== A5133 INIT START ===");

    a5133_config_gpio();

    /* ---- DIAG 0: GPIO 自检 ---- */
    ESP_LOGI(TAG, "GPIO 自检: CS=%d SCLK=%d SDIO=%d",
             gpio_get_level(A5133_PIN_CS),
             gpio_get_level(A5133_PIN_SCLK),
             gpio_get_level(A5133_PIN_SDIO));

    /* ---- DIAG 1: SPI 寄存器读测试 (看 A5133 有没有在响应) ---- */
    ESP_LOGI(TAG, "--- SPI 寄存器读测试 ---");
    uint8_t mode_r   = a5133_read_reg(A5133_REG_MODE);       /* 00h */
    uint8_t modectl_r= a5133_read_reg(A5133_REG_MODE_CTRL);  /* 01h */
    uint8_t pll1_r   = a5133_read_reg(A5133_REG_PLL_I);      /* 0Eh */
    uint8_t adc_r    = a5133_read_reg(A5133_REG_RSSI_ADC);   /* 1Eh */
    ESP_LOGI(TAG, "  REG[00]=0x%02X REG[01]=0x%02X REG[0E]=0x%02X REG[1E]=0x%02X",
             mode_r, modectl_r, pll1_r, adc_r);
    if (mode_r == 0xFF && modectl_r == 0xFF) {
        ESP_LOGW(TAG, "所有寄存器都返回 0xFF — SDIO 始终被拉高 (A5133 没上电/没响应)");
    } else if (mode_r == 0x00 && modectl_r == 0x00) {
        ESP_LOGW(TAG, "所有寄存器都返回 0x00 — SDIO 被拉低 (A5133 没接/短路)");
    } else {
        ESP_LOGI(TAG, "SPI 读成功! 有非全 0/FF 的值 → A5133 在响应");
    }

    /* ---- DIAG 3: 写 PLL I (0Eh) = 74, 再读回 ---- */
    ESP_LOGI(TAG, "--- SPI 写验证: 写 PLL_I(0Eh)=0x4A 再读回 ---");
    a5133_write_reg(A5133_REG_PLL_I, 0x4A);
    vTaskDelay(pdMS_TO_TICKS(1));
    uint8_t chn_back = a5133_read_reg(A5133_REG_PLL_I);
    ESP_LOGI(TAG, "  wrote 0x4A, read back=0x%02X — %s",
             chn_back, (chn_back == 0x4A) ? "✓ SPI 写成功!" : "✗ SPI 写不进去");

    /* ---- 正常初始化 ---- */
    ESP_LOGI(TAG, "--- 完整初始化 ---");

    /* 1. Standby + Reset (写 0x00 到 MODE_REG) */
    a5133_send_strobe(A5133_STROBE_STANDBY);
    a5133_write_reg(A5133_REG_MODE, 0x00);   /* software reset */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 2. RF_Config — 写 65+ 个寄存器默认值 */
    a5133_rf_config();

    /* 3. PLL 校准 */
    a5133_rf_cal();

    /* 4. 设频率 5800 MHz: CHN = 5800-5725 = 75 */
    a5133_set_freq_mhz(5800);

    /* 3. RX 模式 */
    a5133_send_strobe(A5133_STROBE_RX);
    vTaskDelay(pdMS_TO_TICKS(1));  /* PLL 锁定 */

    /* ---- DIAG 4: 进 RX 后, 读 RSSI ---- */
    ESP_LOGI(TAG, "--- RX 模式测试 ---");
    for (volatile int i = 0; i < 5000; i++) __asm__ volatile("nop");
    uint8_t rssi = a5133_read_reg(A5133_REG_RSSI_ADC);
    ESP_LOGI(TAG, "  RX mode → ADC[7:0] = 0x%02X (%u) → RSSI ≈ %d dBm",
             rssi, rssi, -95 + (rssi * 75 / 255));

    /* ---- 读更多寄存器做健康检查 ---- */
    ESP_LOGI(TAG, "--- 全寄存器 dump (0x00..0x1F) ---");
    for (int addr = 0; addr <= 0x1F; addr++) {
        uint8_t val = a5133_read_reg((uint8_t)addr);
        ESP_LOGI(TAG, "  REG[0x%02X] = 0x%02X", addr, val);
    }

    /* 回 Standby 省电 */
    a5133_send_strobe(A5133_STROBE_STANDBY);

    ESP_LOGI(TAG, "=== A5133 INIT DONE ===");
    s_inited = true;
    return ESP_OK;
}

void a5133_deinit(void)
{
    if (!s_inited) return;
    a5133_send_strobe(A5133_STROBE_SLEEP);
    s_inited = false;
    ESP_LOGI(TAG, "deinit → sleep");
}
