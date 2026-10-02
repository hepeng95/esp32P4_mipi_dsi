#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NRF24L01 寄存器地址 */
#define NRF_REG_CONFIG       0x00
#define NRF_REG_EN_AA        0x01
#define NRF_REG_EN_RXADDR    0x02
#define NRF_REG_SETUP_AW     0x03
#define NRF_REG_SETUP_RETR   0x04
#define NRF_REG_RF_CH        0x05
#define NRF_REG_RF_SETUP     0x06
#define NRF_REG_STATUS       0x07
#define NRF_REG_OBSERVE_TX   0x08
#define NRF_REG_RPD          0x09  /* nRF24L01+ 才有 RPD, nRF24L01 用 CD */
#define NRF_REG_RX_ADDR_P0   0x0A
#define NRF_REG_RX_PW_P0     0x11
#define NRF_REG_FIFO_STATUS  0x17
#define NRF_REG_FEATURE      0x1D

/* CONFIG 位 */
#define NRF_CONFIG_PWR_UP    (1 << 1)
#define NRF_CONFIG_PRIM_RX   (1 << 0)

/* STATUS 位 */
#define NRF_STATUS_RX_DR     (1 << 6)
#define NRF_STATUS_TX_DS     (1 << 5)
#define NRF_STATUS_MAX_RT    (1 << 4)

/* RF_SETUP 数据速率位 */
#define NRF_RF_SETUP_RF_DR_LOW  (1 << 5)   /* 1 => 250kbps (与 RF_DR_HIGH 配合) */
#define NRF_RF_SETUP_RF_DR_HIGH (1 << 3)   /* 1 => 2Mbps (RF_DR_LOW=0) */

/* FIFO_STATUS 位 */
#define NRF_FIFO_RX_EMPTY   (1 << 6)

/* NRF24L01+ 通道范围 */
#define NRF_CHANNEL_MIN     0
#define NRF_CHANNEL_MAX     125   /* 2400 ~ 2525 MHz */
#define NRF_CHANNEL_COUNT   126

/* 数据速率枚举 */
typedef enum {
    NRF_RATE_250K = 0,
    NRF_RATE_1M   = 1,
    NRF_RATE_2M   = 2
} nrf_data_rate_t;

/* 引脚配置 */
typedef struct {
    int mosi;   /* SPI MOSI */
    int miso;   /* SPI MISO */
    int sclk;   /* SPI SCLK */
    int csn;    /* 片选 */
    int ce;     /* Chip Enable */
    int spi_host; /* SPI_HOST / HSPI_HOST / FSPI_HOST */
} nrf_pin_config_t;

esp_err_t nrf24l01_init(const nrf_pin_config_t *cfg);
void     nrf24l01_deinit(void);

/** @brief nRF24L01 是否已初始化成功 (SPI 设备已挂载) */
bool     nrf24l01_init_done(void);

/* 基本寄存器读写 */
uint8_t nrf_read_reg(uint8_t reg);
uint8_t nrf_write_reg(uint8_t reg, uint8_t val);

/* 读取 RPD/CD 标志 (1 = 检测到载波, 0 = 未检测) */
uint8_t nrf_read_rpd(void);

/* 切换到 RX 模式 (CE 高) */
void nrf_rx_mode(void);
/* 退出 RX 模式 (CE 低) */
void nrf_standby_mode(void);

/* 设置通道 (0~125) */
esp_err_t nrf_set_channel(uint8_t ch);

/* 设置数据速率 */
esp_err_t nrf_set_data_rate(nrf_data_rate_t rate);

/* 清除 STATUS 中断标志 */
void nrf_clear_irq_flags(void);

/* 刷新 RX FIFO */
void nrf_flush_rx(void);

/* 单通道快速载波检测: 切换通道 -> RX 模式 -> 等待 -> 读 RPD -> 退出 */
uint8_t nrf_detect_channel(uint8_t ch, uint32_t settle_us);

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_H */
