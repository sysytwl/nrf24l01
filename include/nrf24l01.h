#ifndef NRF24L01_H
#define NRF24L01_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 寄存器地址
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA        0x01
#define NRF24_REG_EN_RXADDR    0x02
#define NRF24_REG_SETUP_AW     0x03
#define NRF24_REG_SETUP_RETR   0x04
#define NRF24_REG_RF_CH        0x05
#define NRF24_REG_RF_SETUP     0x06
#define NRF24_REG_STATUS       0x07
#define NRF24_REG_OBSERVE_TX   0x08
#define NRF24_REG_RPD          0x09
#define NRF24_REG_RX_ADDR_P0   0x0A
#define NRF24_REG_RX_ADDR_P1   0x0B
#define NRF24_REG_RX_ADDR_P2   0x0C
#define NRF24_REG_RX_ADDR_P3   0x0D
#define NRF24_REG_RX_ADDR_P4   0x0E
#define NRF24_REG_RX_ADDR_P5   0x0F
#define NRF24_REG_TX_ADDR      0x10
#define NRF24_REG_RX_PW_P0     0x11
#define NRF24_REG_RX_PW_P1     0x12
#define NRF24_REG_RX_PW_P2     0x13
#define NRF24_REG_RX_PW_P3     0x14
#define NRF24_REG_RX_PW_P4     0x15
#define NRF24_REG_RX_PW_P5     0x16
#define NRF24_REG_FIFO_STATUS  0x17
#define NRF24_REG_DYNPD        0x1C
#define NRF24_REG_FEATURE      0x1D

// 命令字节
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_FLUSH_RX      0xE2
#define NRF24_CMD_REUSE_TX_PL   0xE3
#define NRF24_CMD_R_RX_PL_WID   0x60
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_NOP           0xFF

// 配置寄存器位
#define NRF24_CONFIG_PRIM_RX    (1<<0)
#define NRF24_CONFIG_PWR_UP     (1<<1)
#define NRF24_CONFIG_CRCO       (1<<2)
#define NRF24_CONFIG_EN_CRC     (1<<3)
#define NRF24_CONFIG_MASK_MAX_RT (1<<4)
#define NRF24_CONFIG_MASK_TX_DS (1<<5)
#define NRF24_CONFIG_MASK_RX_DR (1<<6)
#define NRF24_REG_RPD         0x09

// 状态寄存器位
#define NRF24_STATUS_TX_FULL    (1<<0)
#define NRF24_STATUS_RX_P_NO    (0x0E) // 掩码位置3:1
#define NRF24_STATUS_MAX_RT     (1<<4)
#define NRF24_STATUS_TX_DS      (1<<5)
#define NRF24_STATUS_RX_DR      (1<<6)

// FIFO 状态寄存器位
#define NRF24_FIFO_TX_REUSE     (1<<6)
#define NRF24_FIFO_TX_FULL      (1<<5)
#define NRF24_FIFO_TX_EMPTY     (1<<4)
#define NRF24_FIFO_RX_FULL      (1<<1)
#define NRF24_FIFO_RX_EMPTY     (1<<0)

// 默认配置
#define NRF24_PIPE_COUNT        6
#define NRF24_ADDR_WIDTH        5
#define NRF24_MAX_PAYLOAD       32

// 数据速率
typedef enum {
    NRF24_DR_1Mbps = 0,
    NRF24_DR_2Mbps = 1,
    NRF24_DR_250Kbps = 2
} nrf24_datarate_t;

// 射频输出功率
typedef enum {
    NRF24_PA_MIN = 0,   // -18 dBm
    NRF24_PA_LOW,       // -12 dBm
    NRF24_PA_HIGH,      // -6 dBm
    NRF24_PA_MAX        // 0 dBm
} nrf24_pa_level_t;

// 自动重传延时
typedef enum {
    NRF24_ARD_250us = 0,
    NRF24_ARD_500us,
    NRF24_ARD_750us,
    NRF24_ARD_1000us,
    NRF24_ARD_1250us,
    NRF24_ARD_1500us,
    NRF24_ARD_1750us,
    NRF24_ARD_2000us,
    NRF24_ARD_2250us,
    NRF24_ARD_2500us,
    NRF24_ARD_2750us,
    NRF24_ARD_3000us,
    NRF24_ARD_3250us,
    NRF24_ARD_3500us,
    NRF24_ARD_3750us,
    NRF24_ARD_4000us
} nrf24_ard_t;

// 自动重传次数
typedef enum {
    NRF24_ARC_DISABLED = 0,
    NRF24_ARC_1,
    NRF24_ARC_2,
    NRF24_ARC_3,
    NRF24_ARC_4,
    NRF24_ARC_5,
    NRF24_ARC_6,
    NRF24_ARC_7,
    NRF24_ARC_8,
    NRF24_ARC_9,
    NRF24_ARC_10,
    NRF24_ARC_11,
    NRF24_ARC_12,
    NRF24_ARC_13,
    NRF24_ARC_14,
    NRF24_ARC_15
} nrf24_arc_t;

// 地址宽度
typedef enum {
    NRF24_AW_3BYTES = 1,
    NRF24_AW_4BYTES = 2,
    NRF24_AW_5BYTES = 3
} nrf24_addr_width_t;

// 主设备句柄
typedef struct {
    spi_device_handle_t spi;        // SPI设备句柄
    gpio_num_t ce_pin;              // CE引脚
    gpio_num_t irq_pin;             // IRQ引脚（可选，可设为-1）
    uint8_t rx_pipe_addr[NRF24_PIPE_COUNT][NRF24_ADDR_WIDTH]; // 各管道接收地址（仅低字节有效）
    uint8_t tx_addr[NRF24_ADDR_WIDTH];                         // 发送地址
} nrf24l01_t;

/**
 * @brief 初始化nRF24L01+设备
 *
 * @param dev 设备句柄指针（将填充）
 * @param host SPI主机（SPI2_HOST或SPI3_HOST）
 * @param clk_speed SPI时钟速度（Hz），建议不超过10MHz
 * @param cs_pin 片选引脚
 * @param ce_pin CE引脚
 * @param irq_pin IRQ引脚（可设为-1不使用）
 * @return esp_err_t
 */
esp_err_t nrf24l01_init(nrf24l01_t *dev, gpio_num_t ce_pin, gpio_num_t irq_pin);

/**
 * @brief 设置RF通道
 *
 * @param dev 设备句柄
 * @param channel 通道号（0-125）
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_channel(nrf24l01_t *dev, uint8_t channel);

/**
 * @brief 设置数据速率
 *
 * @param dev 设备句柄
 * @param rate 数据速率
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_datarate(nrf24l01_t *dev, nrf24_datarate_t rate);

/**
 * @brief 设置发射功率
 *
 * @param dev 设备句柄
 * @param pa_level 功率等级
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_pa_level(nrf24l01_t *dev, nrf24_pa_level_t pa_level);

/**
 * @brief 设置自动重传参数
 *
 * @param dev 设备句柄
 * @param ard 重传延时
 * @param arc 重传次数
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_retransmit(nrf24l01_t *dev, nrf24_ard_t ard, nrf24_arc_t arc);

/**
 * @brief 设置接收地址（管道0）
 *
 * @param dev 设备句柄
 * @param addr 5字节地址（LSB先发送）
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_rx_addr_p0(nrf24l01_t *dev, const uint8_t *addr);

/**
 * @brief 设置接收地址（管道1）
 *
 * @param dev 设备句柄
 * @param addr 5字节地址（LSB先发送）
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_rx_addr_p1(nrf24l01_t *dev, const uint8_t *addr);

/**
 * @brief 设置接收管道2-5地址（低字节）
 *
 * @param dev 设备句柄
 * @param pipe 管道号（2-5）
 * @param addr_byte 单字节地址
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_rx_addr_p2_5(nrf24l01_t *dev, uint8_t pipe, uint8_t addr_byte);

/**
 * @brief 设置发送地址
 *
 * @param dev 设备句柄
 * @param addr 5字节地址（LSB先发送）
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_tx_addr(nrf24l01_t *dev, const uint8_t *addr);

/**
 * @brief 设置接收管道有效载荷宽度
 *
 * @param dev 设备句柄
 * @param pipe 管道号（0-5）
 * @param width 有效载荷字节数（1-32）
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_rx_payload_width(nrf24l01_t *dev, uint8_t pipe, uint8_t width);

/**
 * @brief 使能自动应答（Enhanced ShockBurst）
 *
 * @param dev 设备句柄
 * @param pipe 管道号（0-5）
 * @param enable true=使能
 * @return esp_err_t
 */
esp_err_t nrf24l01_enable_auto_ack(nrf24l01_t *dev, uint8_t pipe, bool enable);

/**
 * @brief 使能动态有效载荷长度
 *
 * @param dev 设备句柄
 * @param pipe 管道号（0-5）
 * @param enable true=使能
 * @return esp_err_t
 */
esp_err_t nrf24l01_enable_dyn_payload(nrf24l01_t *dev, uint8_t pipe, bool enable);

/**
 * @brief 设置工作模式为接收模式
 *
 * @param dev 设备句柄
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_rx_mode(nrf24l01_t *dev);

/**
 * @brief 设置工作模式为发送模式（不立即发送，需调用发送函数）
 *
 * @param dev 设备句柄
 * @return esp_err_t
 */
esp_err_t nrf24l01_set_tx_mode(nrf24l01_t *dev);

/**
 * @brief 发送数据（阻塞，等待发送完成或超时）
 *
 * @param dev 设备句柄
 * @param data 数据缓冲区
 * @param len 数据长度（1-32）
 * @param timeout_ms 超时时间（毫秒）
 * @return esp_err_t 成功返回ESP_OK，失败返回错误码
 */
esp_err_t nrf24l01_send(nrf24l01_t *dev, const uint8_t *data, size_t len, int timeout_ms);

/**
 * @brief 检查是否有数据可读
 *
 * @param dev 设备句柄
 * @param pipe 输出参数：收到数据的管道号
 * @return true 有数据，false 无数据
 */
bool nrf24l01_data_ready(nrf24l01_t *dev, uint8_t *pipe);

/**
 * @brief 接收数据（非阻塞，如果有数据则读取）
 *
 * @param dev 设备句柄
 * @param data 接收缓冲区（至少32字节）
 * @param len 输出参数：实际接收到的数据长度
 * @param pipe 输出参数：接收管道号
 * @return esp_err_t ESP_OK表示成功读取数据，ESP_ERR_NOT_FOUND表示无数据
 */
esp_err_t nrf24l01_receive(nrf24l01_t *dev, uint8_t *data, size_t *len, uint8_t *pipe);

/**
 * @brief 获取状态寄存器值
 *
 * @param dev 设备句柄
 * @return uint8_t 状态寄存器
 */
uint8_t nrf24l01_get_status(nrf24l01_t *dev);

/**
 * @brief 清空中断标志（写1清除）
 *
 * @param dev 设备句柄
 * @param flags 要清除的位（RX_DR, TX_DS, MAX_RT）
 */
void nrf24l01_clear_irq(nrf24l01_t *dev, uint8_t flags);

/**
 * @brief 使能模块（PWR_UP）
 *
 * @param dev 设备句柄
 * @return esp_err_t
 */
esp_err_t nrf24l01_power_up(nrf24l01_t *dev);

/**
 * @brief 关闭模块（PWR_DOWN）
 *
 * @param dev 设备句柄
 * @return esp_err_t
 */
esp_err_t nrf24l01_power_down(nrf24l01_t *dev);

/**
 * @brief 打印当前配置（调试用）
 *
 * @param dev 设备句柄
 */
void nrf24l01_dump_registers(nrf24l01_t *dev);

/**
 * @brief 检测接收功率（RPD），判断信号强度是否高于 -64dBm
 *
 * @param dev 设备句柄
 * @return true 检测到信号强度 > -64dBm
 * @return false 未检测到信号或信号强度 < -64dBm
 */
bool nrf24l01_test_rpd(nrf24l01_t *dev);

#ifdef __cplusplus
}
#endif

#endif // NRF24L01_H