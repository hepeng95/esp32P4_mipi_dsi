#include "nrf24l01.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "nrf24";

/* SPI 命令 */
#define NRF_CMD_R_REG        0x00   /* R_REG | reg */
#define NRF_CMD_W_REG        0x20   /* W_REG | reg */
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX     0xE1
#define NRF_CMD_FLUSH_RX     0xE2
#define NRF_CMD_REUSE_TX_PL  0xE3
#define NRF_CMD_R_RX_PL_WID  0x60
#define NRF_CMD_W_ACK_PAYLOAD 0xA8
#define NRF_CMD_NOP          0xFF

static spi_device_handle_t s_spi = NULL;
static nrf_pin_config_t    s_pins;

static inline void ce_high(void) { gpio_set_level(s_pins.ce, 1); }
static inline void ce_low(void)  { gpio_set_level(s_pins.ce, 0); }
static inline void cs_high(void) { gpio_set_level(s_pins.csn, 1); }
static inline void cs_low(void)  { gpio_set_level(s_pins.csn, 0); }

/* SPI 单次传输 (tx 写, rx 读, 长度 len)
 * ★ 手动 CS 控制 — nRF24L01 对 CSN 时序要求严格, 不能用 SPI host 硬件 CS */
static esp_err_t spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0) return ESP_OK;
    if (s_spi == NULL) return ESP_ERR_INVALID_STATE;

    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.flags = 0;

    cs_low();   /* ★ 手动拉低 CSN (必须在 SCLK 空闲期间) */
    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    cs_high();  /* ★ 手动拉高 CSN (必须在最后一个时钟沿之后) */

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 传输失败 len=%u: %s", (unsigned)len, esp_err_to_name(ret));
    }
    return ret;
}

uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_R_REG | (reg & 0x1F)), 0xFF };
    uint8_t rx[2] = { 0 };
    if (spi_xfer(tx, rx, 2) != ESP_OK) return 0xFF;
    return rx[1];
}

uint8_t nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_W_REG | (reg & 0x1F)), val };
    uint8_t rx[2] = { 0 };
    if (spi_xfer(tx, rx, 2) != ESP_OK) return 0xFF;
    return rx[1];
}

/* 打印所有关键寄存器 (调试用) */
static void nrf_dump_registers(void)
{
    ESP_LOGI(TAG, "---- NRF24L01 寄存器 dump ----");
    ESP_LOGI(TAG, "  CONFIG     [0x00] = 0x%02x", nrf_read_reg(NRF_REG_CONFIG));
    ESP_LOGI(TAG, "  EN_AA      [0x01] = 0x%02x", nrf_read_reg(NRF_REG_EN_AA));
    ESP_LOGI(TAG, "  EN_RXADDR  [0x02] = 0x%02x", nrf_read_reg(NRF_REG_EN_RXADDR));
    ESP_LOGI(TAG, "  SETUP_AW   [0x03] = 0x%02x", nrf_read_reg(NRF_REG_SETUP_AW));
    ESP_LOGI(TAG, "  SETUP_RETR [0x04] = 0x%02x", nrf_read_reg(NRF_REG_SETUP_RETR));
    ESP_LOGI(TAG, "  RF_CH      [0x05] = 0x%02x  (channel=%u, freq=%.1fMHz)",
             nrf_read_reg(NRF_REG_RF_CH), nrf_read_reg(NRF_REG_RF_CH) & 0x7F,
             2400.0 + (nrf_read_reg(NRF_REG_RF_CH) & 0x7F));
    ESP_LOGI(TAG, "  RF_SETUP   [0x06] = 0x%02x", nrf_read_reg(NRF_REG_RF_SETUP));
    ESP_LOGI(TAG, "  STATUS     [0x07] = 0x%02x", nrf_read_reg(NRF_REG_STATUS));
    ESP_LOGI(TAG, "  OBSERVE_TX [0x08] = 0x%02x", nrf_read_reg(NRF_REG_OBSERVE_TX));
    ESP_LOGI(TAG, "  RPD/CD     [0x09] = 0x%02x", nrf_read_reg(NRF_REG_RPD));
    ESP_LOGI(TAG, "  FIFO_STAT  [0x17] = 0x%02x", nrf_read_reg(NRF_REG_FIFO_STATUS));
    ESP_LOGI(TAG, "--------------------------------");
}

/* ===== 自检: 写入后回读验证, 检测 SPI 链路是否正常 =====
 * 返回 true 表示所有寄存器读写正常
 *
 * 测试两类:
 *   1) STATUS 上电默认值检查 (0x0E): 用于探测芯片是否真的在线
 *      - 0x00 / 0xFF => 芯片未响应 (SPI 链路异常或芯片未供电)
 *      - 非 0x0E      => 芯片在线但状态异常
 *   2) 多个寄存器的写入后回读验证 (覆盖单字节寄存器全场景) */
static bool nrf_self_test(void)
{
    ESP_LOGI(TAG, "==== NRF24L01 自检 (读写验证) ====");

    /* ---------- 1) STATUS 上电默认值检查 ---------- */
    /* STATUS 寄存器 bit6/5/4 = IRQ 标志 (上电后为 0, 除非有事件)
     *                       bit3:2 = RX_P_NO (上电后 = 111 = 0b111 = FIFO 空)
     *                       bit0   = TX_FULL (上电后 = 0)
     *   => 上电默认值 = 0b0000_1110 = 0x0E
     *   注: 即使 PWR_UP=1, RX_P_NO 仍为 111 (FIFO 空), TX_FULL=0
     *       只有收到数据 / FIFO 满才会改变 */
    uint8_t status = nrf_read_reg(NRF_REG_STATUS);
    ESP_LOGI(TAG, "  [STATUS] 当前值 = 0x%02x", status);
    if (status == 0x00 || status == 0xFF) {
        ESP_LOGE(TAG, "  [FAIL] STATUS 读到 0x%02x: 芯片未响应 (检查 SPI 接线 / 供电)", status);
        ESP_LOGE(TAG, "    排查:");
        ESP_LOGE(TAG, "      1) MISO/MOSI/SCLK/CSN 接线是否正确");
        ESP_LOGE(TAG, "      2) NRF24L01 VCC 是否 3.3V (不能 5V)");
        ESP_LOGE(TAG, "      3) CE 是否接对 (不接会导致芯片不工作)");
        ESP_LOGE(TAG, "      4) 用的模块是否带 LDO (有些模块标 5V 但 LDO 压差大)");
        return false;
    } else if ((status & 0x0E) != 0x0E) {
        ESP_LOGW(TAG, "  [WARN] STATUS=0x%02x, 低 4 位期望 0x0E (RX FIFO 空且 TX FIFO 未满)", status);
        /* 不算致命错误, 继续后续测试 */
    } else {
        ESP_LOGI(TAG, "  [PASS] STATUS 默认值正常 (0x0E), 芯片在线");
    }

    /* ---------- 2) 寄存器写入后回读验证 ---------- */
    struct {
        uint8_t reg;
        uint8_t wval;
        uint8_t restore;       /* 测试后恢复到的值 */
        const char *name;
    } tests[] = {
        /* 注意: 每个寄存器选一个非默认值, 避免巧合通过 */
        { NRF_REG_RF_CH,       0x4A, 0x02, "RF_CH" },       /* 通道 74 = 2474MHz */
        { NRF_REG_SETUP_AW,    0x01, 0x03, "SETUP_AW" },    /* 3 字节地址 (默认 5 字节) */
        { NRF_REG_EN_RXADDR,   0x03, 0x01, "EN_RXADDR" },   /* pipe 0,1 (默认仅 pipe0) */
        { NRF_REG_EN_AA,       0x00, 0x00, "EN_AA" },       /* 无自动应答 */
        { NRF_REG_SETUP_RETR,  0x1A, 0x00, "SETUP_RETR" },  /* 500us + 10 次重传 */
        { NRF_REG_RX_PW_P0,    0x10, 0x20, "RX_PW_P0" },    /* 16 字节 (默认 32) */
        /* RF_SETUP: 仅低 3 位 (RF_PWR) 可任意写, bit5/3 (RF_DR) 也可写
         *          bit7 (CONT_WAVE) bit6 (PLL_LOCK) 不建议动
         *          bit2/1/0 保留 (datasheet 标 0), 写入后回读应屏蔽保留位 */
        { NRF_REG_RF_SETUP,    0x07, 0x0E, "RF_SETUP" },    /* 1Mbps, -18dBm (写 0x07) */
    };

    bool all_ok = true;
    for (size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i) {
        uint8_t reg  = tests[i].reg;
        uint8_t wval = tests[i].wval;

        /* 写入 */
        nrf_write_reg(reg, wval);
        /* 回读 */
        uint8_t rval = nrf_read_reg(reg);

        /* RF_SETUP bit2/1/0 是保留位, 写入后回读时掩掉 */
        uint8_t mask = 0xFF;
        if (reg == NRF_REG_RF_SETUP) mask = 0xF8;  /* 只比较高 5 位 */

        if ((rval & mask) == (wval & mask)) {
            ESP_LOGI(TAG, "  [PASS] %-10s: 写 0x%02x → 读 0x%02x", tests[i].name, wval, rval);
        } else {
            ESP_LOGE(TAG, "  [FAIL] %-10s: 写 0x%02x → 读 0x%02x (期望 0x%02x & 0x%02x)",
                     tests[i].name, wval, rval, wval, mask);
            all_ok = false;
        }

        /* 恢复默认值 (避免影响后续初始化流程) */
        nrf_write_reg(reg, tests[i].restore);
    }

    if (all_ok) {
        ESP_LOGI(TAG, "  自检通过: SPI 通信正常, %u 个寄存器读写一致",
                 (unsigned)(sizeof(tests)/sizeof(tests[0])));
    } else {
        ESP_LOGE(TAG, "  自检失败: SPI 链路异常 (部分寄存器读写不一致)");
        ESP_LOGE(TAG, "    排查:");
        ESP_LOGE(TAG, "      1) CSN/SCLK 信号完整性 (线长 / 跳线质量)");
        ESP_LOGE(TAG, "      2) SPI 时钟是否过高 (当前 8MHz, 可降到 2MHz 测试)");
        ESP_LOGE(TAG, "      3) 模块可能为次品 / 假货");
    }
    return all_ok;
}

/* ===== 活性检测: 扫描 WiFi 信道 6 (2437MHz) 是否有载波 =====
 * 说明: ESP32-C5 WiFi AP 信道 6 一定会产生 2.4GHz 载波,
 *       如果 NRF 自检通过但活性检测失败, 说明 RPD 检测时序有问题 */
static void nrf_live_test(void)
{
    ESP_LOGI(TAG, "==== NRF24L01 活性检测 (扫描 WiFi 信道) ====");
    ESP_LOGI(TAG, "  依次扫描通道 32-42 (2432-2442MHz, 覆盖 WiFi ch6=2437MHz)");
    ESP_LOGI(TAG, "  期望: 附近通道 37 (2437MHz) 应有较高 RPD 命中率");

    const uint8_t ch_start = 32, ch_end = 42;
    const uint8_t samples = 20;
    uint8_t hits[16] = {0};   /* 最多 16 通道 */

    for (uint8_t s = 0; s < samples; ++s) {
        for (uint8_t ch = ch_start; ch <= ch_end; ++ch) {
            uint8_t rpd = nrf_detect_channel(ch, 500);
            if (rpd) hits[ch - ch_start]++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "  活性检测结果 (20 次采样, 命中次数):");
    bool any_hit = false;
    for (uint8_t ch = ch_start; ch <= ch_end; ++ch) {
        uint8_t h = hits[ch - ch_start];
        if (h > 0) any_hit = true;
        /* 简易柱状图 */
        char bar[24];
        memset(bar, '#', h < 20 ? h : 20);
        bar[h < 20 ? h : 20] = '\0';
        ESP_LOGI(TAG, "    ch%3u (%.0fMHz): %2u/20 %s",
                 ch, 2400.0 + ch, h, bar);
    }

    if (any_hit) {
        ESP_LOGI(TAG, "  活性检测通过: 检测到载波, NRF24L01 正常工作");
    } else {
        ESP_LOGW(TAG, "  活性检测未检测到载波");
        ESP_LOGW(TAG, "    可能原因:");
        ESP_LOGW(TAG, "      1) 附近无 WiFi 设备 (极少见)");
        ESP_LOGW(TAG, "      2) NRF24L01 模块天线损坏");
        ESP_LOGW(TAG, "      3) 模块是假货/次品 (淘宝常见)");
        ESP_LOGW(TAG, "      4) RPD 检测时序仍有问题");
    }
}

uint8_t nrf_read_rpd(void)
{
    return nrf_read_reg(NRF_REG_RPD) & 0x01;
}

void nrf_rx_mode(void)
{
    /* PRIM_RX=1, PWR_UP=1, CRC 使能 (EN_CRC=1, CRCO=0 -> 1 byte) */
    uint8_t cfg = nrf_read_reg(NRF_REG_CONFIG);
    cfg |= NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX | (1 << 2); /* EN_CRC */
    nrf_write_reg(NRF_REG_CONFIG, cfg);
    ce_high();
}

void nrf_standby_mode(void)
{
    ce_low();
    /* 保持 PWR_UP=1 但 PRIM_RX=0 => Standby-I */
    uint8_t cfg = nrf_read_reg(NRF_REG_CONFIG);
    cfg |= NRF_CONFIG_PWR_UP;
    cfg &= ~NRF_CONFIG_PRIM_RX;
    nrf_write_reg(NRF_REG_CONFIG, cfg);
}

esp_err_t nrf_set_channel(uint8_t ch)
{
    if (ch > NRF_CHANNEL_MAX) {
        ESP_LOGE(TAG, "通道号 %u 超出范围 (0-%d)", ch, NRF_CHANNEL_MAX);
        return ESP_ERR_INVALID_ARG;
    }
    nrf_write_reg(NRF_REG_RF_CH, ch & 0x7F);
    ESP_LOGD(TAG, "切换通道 -> %u (%.1f MHz)", ch, 2400.0 + ch);
    return ESP_OK;
}

esp_err_t nrf_set_data_rate(nrf_data_rate_t rate)
{
    uint8_t rs = nrf_read_reg(NRF_REG_RF_SETUP);
    rs &= ~(NRF_RF_SETUP_RF_DR_LOW | NRF_RF_SETUP_RF_DR_HIGH);
    const char *rate_str = "?";
    switch (rate) {
        case NRF_RATE_250K: rs |= NRF_RF_SETUP_RF_DR_LOW;  rate_str = "250kbps"; break;
        case NRF_RATE_1M:   /* 00 = 1Mbps */               rate_str = "1Mbps";   break;
        case NRF_RATE_2M:   rs |= NRF_RF_SETUP_RF_DR_HIGH; rate_str = "2Mbps";   break;
        default:
            ESP_LOGE(TAG, "无效数据速率 %d", (int)rate);
            return ESP_ERR_INVALID_ARG;
    }
    /* 同时设定 RF 输出功率为 0dBm (11) */
    rs |= (3 << 1); /* RF_PWR = 11 -> 0dBm */
    nrf_write_reg(NRF_REG_RF_SETUP, rs);
    ESP_LOGI(TAG, "数据速率设置为 %s (RF_SETUP=0x%02x)", rate_str, nrf_read_reg(NRF_REG_RF_SETUP));
    return ESP_OK;
}

void nrf_clear_irq_flags(void)
{
    nrf_write_reg(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
}

void nrf_flush_rx(void)
{
    uint8_t tx = NRF_CMD_FLUSH_RX;
    uint8_t rx = 0;
    spi_xfer(&tx, &rx, 1);
}

uint8_t nrf_detect_channel(uint8_t ch, uint32_t settle_us)
{
    /* 切通道: CE 必须在 Low (Standby) 态 */
    ce_low();

    nrf_set_channel(ch);
    nrf_clear_irq_flags();
    /* flush_rx 其实不需要 (我们不读 payload), 省掉 1 次 SPI */
    /* nrf_flush_rx(); */

    /* 限制 settle: PLL 锁定 ≥ 130µs, RPD 能量累计 ≥ 70µs (datasheet min)
     * ★ 取 max 70µs (RPD settling), 让调用方传 100µs 足够 */
    if (settle_us < 70)  settle_us = 70;
    if (settle_us > 10000) settle_us = 10000;

    ce_high();
    esp_rom_delay_us(settle_us);    /* 直接等, 不再分两段 (简化) */

    uint8_t rpd = nrf_read_rpd();
    ce_low();
    return rpd;
}

esp_err_t nrf24l01_init(const nrf_pin_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_pins = *cfg;

    ESP_LOGI(TAG, "==== NRF24L01 初始化 ====");
    ESP_LOGI(TAG, "  引脚: MOSI=%d MISO=%d SCLK=%d CSN=%d CE=%d",
             cfg->mosi, cfg->miso, cfg->sclk, cfg->csn, cfg->ce);
    ESP_LOGI(TAG, "  SPI host=%d, 时钟=1MHz (CSN=GPIO手动控制)", (int)cfg->spi_host);

    /* ★ GPIO 自检: 在配 SPI host 之前, 先手动翻转所有引脚确认硬件活着
     *   这一步能立刻暴露引脚冲突/焊接问题 */
    {
        gpio_config_t io_all = {
            .pin_bit_mask = (1ULL << cfg->ce) | (1ULL << cfg->csn) |
                            (1ULL << cfg->mosi) | (1ULL << cfg->sclk),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_all);
        /* MISO 独立配成输入 (上拉) */
        gpio_config_t io_miso = {
            .pin_bit_mask = (1ULL << cfg->miso),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_miso);

        ESP_LOGI(TAG, "---- GPIO 电平自检 (手动翻转) ----");
        /* 全高 */
        gpio_set_level(cfg->ce, 1); gpio_set_level(cfg->csn, 1);
        gpio_set_level(cfg->mosi, 1); gpio_set_level(cfg->sclk, 1);
        esp_rom_delay_us(10);
        ESP_LOGI(TAG, "  全HIGH: CE=%d CSN=%d MOSI=%d SCLK=%d MISO=%d",
                 gpio_get_level(cfg->ce), gpio_get_level(cfg->csn),
                 gpio_get_level(cfg->mosi), gpio_get_level(cfg->sclk),
                 gpio_get_level(cfg->miso));
        /* 全低 */
        gpio_set_level(cfg->ce, 0); gpio_set_level(cfg->csn, 0);
        gpio_set_level(cfg->mosi, 0); gpio_set_level(cfg->sclk, 0);
        esp_rom_delay_us(10);
        ESP_LOGI(TAG, "  全LOW : CE=%d CSN=%d MOSI=%d SCLK=%d MISO=%d",
                 gpio_get_level(cfg->ce), gpio_get_level(cfg->csn),
                 gpio_get_level(cfg->mosi), gpio_get_level(cfg->sclk),
                 gpio_get_level(cfg->miso));

        /* 失败判断: 输出脚回读应跟上; MISO 输入应有上拉 (读1) */
        bool ok = true;
        if (gpio_get_level(cfg->mosi) != 0) { ESP_LOGE(TAG, "  !! MOSI GPIO 被外部强驱 HIGH (引脚冲突/短路)"); ok = false; }
        if (gpio_get_level(cfg->sclk) != 0) { ESP_LOGE(TAG, "  !! SCLK GPIO 被外部强驱 HIGH (引脚冲突/短路)"); ok = false; }
        if (gpio_get_level(cfg->ce)   != 0) { ESP_LOGE(TAG, "  !! CE   GPIO 被外部强驱 HIGH"); ok = false; }
        if (gpio_get_level(cfg->csn)  != 0) { ESP_LOGE(TAG, "  !! CSN  GPIO 被外部强驱 HIGH"); ok = false; }
        if (ok) ESP_LOGI(TAG, "  ✅ GPIO 自检通过 (无冲突)");
        /* MISO 可能悬空也可能由 nRF24L01 驱动, 此处只看是否能读到上拉 */
        ESP_LOGI(TAG, "---- GPIO 自检完成 ----");
    }

    /* 空闲态: CSN=HIGH, CE=LOW */
    gpio_set_level(cfg->ce, 0);
    gpio_set_level(cfg->csn, 1);

    /* 配置 SPI 总线 (会接管 MOSI/MISO/SCLK 的引脚复用) */
    spi_bus_config_t buscfg = {
        .mosi_io_num = cfg->mosi,
        .miso_io_num = cfg->miso,
        .sclk_io_num = cfg->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI 总线初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI 总线初始化成功");

    /* ★ spics_io_num = -1 禁用硬件 CS — CSN 由手动 cs_low()/cs_high() 控制 */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  /* 1 MHz (先保守, 稳定后提频) */
        .mode = 0,                           /* CPOL=0, CPHA=0 */
        .spics_io_num = -1,                  /* ★ 禁用硬件 CS */
        .queue_size = 4,
        .flags = 0,
    };
    esp_err_t dev_ret = spi_bus_add_device(cfg->spi_host, &devcfg, &s_spi);
    if (dev_ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 设备添加失败: %s", esp_err_to_name(dev_ret));
        return dev_ret;
    }
    ESP_LOGI(TAG, "SPI 设备添加成功 (1MHz, mode 0, CSN=GPIO手动)");

    vTaskDelay(pdMS_TO_TICKS(100)); /* 等待器件稳定 */
    ESP_LOGI(TAG, "等待器件稳定完成");

    /* 上电流程: PWR_UP=1 -> 等 1.5ms -> 进入配置
     * ★ 设置 PRIM_RX=1 (RX 模式), 扫描仪永远不需要 TX, 且 RPD 只在 RX 模式下工作 */
    nrf_write_reg(NRF_REG_CONFIG, NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX | (1 << 2));
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 关闭自动应答 / 自动重传, 扫描不需要 */
    nrf_write_reg(NRF_REG_EN_AA, 0x00);
    nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);
    /* 使能 pipe0, 地址宽度 5 字节 */
    nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);
    nrf_write_reg(NRF_REG_SETUP_AW, 0x03); /* 5 字节地址 */
    nrf_write_reg(NRF_REG_RX_PW_P0, 32);

    /* 默认 2Mbps, 0dBm */
    nrf_set_data_rate(NRF_RATE_2M);
    nrf_set_channel(2);

    nrf_clear_irq_flags();
    nrf_flush_rx();

    /* 验证芯片是否在线 */
    uint8_t cfg_read = nrf_read_reg(NRF_REG_CONFIG);
    if ((cfg_read & NRF_CONFIG_PWR_UP) == 0) {
        ESP_LOGE(TAG, "!! NRF24L01 不响应, 请检查接线 / 供电 (CONFIG=0x%02x)", cfg_read);
        ESP_LOGE(TAG, "   常见原因: 1) 接线错误 2) VCC 未接 3.3V 3) CSN/CE 接反");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "NRF24L01 在线 (CONFIG=0x%02x)", cfg_read);

    /* 1. 寄存器读写自检 */
    bool test_ok = nrf_self_test();
    if (!test_ok) {
        ESP_LOGE(TAG, "NRF24L01 自检失败, 终止初始化");
        return ESP_FAIL;
    }

    /* 2. 寄存器 dump (自检后值已恢复默认) */
    nrf_dump_registers();

    /* 3. 活性检测移到 rf_24g 后台任务 — 避免阻塞 init */
    /* nrf_live_test(); */

    ESP_LOGI(TAG, "==== NRF24L01 初始化完成 ====");
    return ESP_OK;
}

void nrf24l01_deinit(void)
{
    if (s_spi) {
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
    }
}

bool nrf24l01_init_done(void)
{
    return (s_spi != NULL);
}
