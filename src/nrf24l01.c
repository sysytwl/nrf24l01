#include "nrf24l01.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static const char *TAG = "nRF24L01";

// 内部辅助函数
static esp_err_t nrf24l01_write_register(nrf24l01_t *dev, uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { NRF24_CMD_W_REGISTER | reg, value };
    spi_transaction_t t = {
        .length = 2 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t nrf24l01_write_register_multi(nrf24l01_t *dev, uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_W_REGISTER | reg;
    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    // 分配临时缓冲区包含命令和数据
    uint8_t *tx_buf = malloc(1 + len);
    if (!tx_buf) return ESP_ERR_NO_MEM;
    tx_buf[0] = cmd;
    memcpy(tx_buf + 1, data, len);
    t.tx_buffer = tx_buf;
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    free(tx_buf);
    return ret;
}

static esp_err_t nrf24l01_read_register(nrf24l01_t *dev, uint8_t reg, uint8_t *value) {
    uint8_t tx_data[2] = { NRF24_CMD_R_REGISTER | reg, NRF24_CMD_NOP };
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {
        .length = 2 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    return ret;
}

static esp_err_t nrf24l01_read_register_multi(nrf24l01_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_R_REGISTER | reg;
    uint8_t *tx_buf = malloc(1 + len);
    uint8_t *rx_buf = malloc(1 + len);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    tx_buf[0] = cmd;
    memset(tx_buf + 1, NRF24_CMD_NOP, len); // 填充NOP用于时钟
    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, rx_buf + 1, len);
    }
    free(tx_buf);
    free(rx_buf);
    return ret;
}

static esp_err_t nrf24l01_write_cmd(nrf24l01_t *dev, uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t nrf24l01_write_cmd_with_data(nrf24l01_t *dev, uint8_t cmd, const uint8_t *data, size_t len) {
    uint8_t *tx_buf = malloc(1 + len);
    if (!tx_buf) return ESP_ERR_NO_MEM;
    tx_buf[0] = cmd;
    memcpy(tx_buf + 1, data, len);
    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx_buf,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    free(tx_buf);
    return ret;
}

static void nrf24l01_ce_high(nrf24l01_t *dev) {
    if (dev->ce_pin != GPIO_NUM_NC) {
        gpio_set_level(dev->ce_pin, 1);
    }
}

static void nrf24l01_ce_low(nrf24l01_t *dev) {
    if (dev->ce_pin != GPIO_NUM_NC) {
        gpio_set_level(dev->ce_pin, 0);
    }
}

esp_err_t nrf24l01_init(nrf24l01_t *dev, spi_host_device_t host, int clk_speed,
                        gpio_num_t cs_pin, gpio_num_t ce_pin, gpio_num_t irq_pin) {
    esp_err_t ret;
    memset(dev, 0, sizeof(nrf24l01_t));
    dev->ce_pin = ce_pin;
    dev->irq_pin = irq_pin;

    // 配置CE引脚
    if (ce_pin != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << ce_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) return ret;
        nrf24l01_ce_low(dev);
    }

    // 配置IRQ引脚（如果使用）
    if (irq_pin != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << irq_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE, // 可根据需要选择
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) return ret;
    }

    // 添加SPI设备
    spi_device_interface_config_t devcfg = {
        .mode = 0,                  // CPOL=0, CPHA=0
        .clock_speed_hz = clk_speed,
        .spics_io_num = cs_pin,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX, // nRF24L01+ 使用半双工，但这里仅用MOSI+MISO同时传输命令字节，后续可能半双工优化，简单起见使用全双工模式，但nRF24L01+实际上支持全双工SPI读写
    };
    ret = spi_bus_add_device(host, &devcfg, &dev->spi);
    if (ret != ESP_OK) return ret;

    // 延时，确保芯片上电稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取状态寄存器，验证通信
    uint8_t status;
    ret = nrf24l01_read_register(dev, NRF24_REG_STATUS, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STATUS register");
        return ret;
    }
    ESP_LOGI(TAG, "STATUS = 0x%02x", status);

    // 默认配置：关断模式，CRC使能，1字节CRC，关闭所有中断掩码
    ret = nrf24l01_write_register(dev, NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO);
    if (ret != ESP_OK) return ret;

    // 设置地址宽度为5字节
    ret = nrf24l01_write_register(dev, NRF24_REG_SETUP_AW, NRF24_AW_5BYTES);
    if (ret != ESP_OK) return ret;

    // 设置RF通道为2 (2400+2 MHz)
    ret = nrf24l01_set_channel(dev, 2);
    if (ret != ESP_OK) return ret;

    // 设置数据速率1Mbps，功率0dBm
    ret = nrf24l01_set_datarate(dev, NRF24_DR_1Mbps);
    if (ret != ESP_OK) return ret;
    ret = nrf24l01_set_pa_level(dev, NRF24_PA_MAX);
    if (ret != ESP_OK) return ret;

    // 默认自动重传：500us延时，重传3次
    ret = nrf24l01_set_retransmit(dev, NRF24_ARD_500us, NRF24_ARC_3);
    if (ret != ESP_OK) return ret;

    // 清空FIFO
    nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_TX);
    nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_RX);

    // 清空中断
    nrf24l01_clear_irq(dev, 0xFF);

    // 默认接收所有管道，但关闭所有管道（先禁用）
    ret = nrf24l01_write_register(dev, NRF24_REG_EN_RXADDR, 0);
    if (ret != ESP_OK) return ret;

    // 默认设置每个管道的有效载荷宽度为0（禁用）
    for (int i = 0; i < NRF24_PIPE_COUNT; i++) {
        nrf24l01_write_register(dev, NRF24_REG_RX_PW_P0 + i, 0);
    }

    // 关闭自动应答和动态载荷（默认）
    nrf24l01_write_register(dev, NRF24_REG_EN_AA, 0);
    nrf24l01_write_register(dev, NRF24_REG_DYNPD, 0);
    nrf24l01_write_register(dev, NRF24_REG_FEATURE, 0);

    ESP_LOGI(TAG, "nRF24L01+ initialized");
    return ESP_OK;
}

esp_err_t nrf24l01_set_channel(nrf24l01_t *dev, uint8_t channel) {
    if (channel > 125) return ESP_ERR_INVALID_ARG;
    return nrf24l01_write_register(dev, NRF24_REG_RF_CH, channel & 0x7F);
}

esp_err_t nrf24l01_set_datarate(nrf24l01_t *dev, nrf24_datarate_t rate) {
    uint8_t rf_setup;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_RF_SETUP, &rf_setup);
    if (ret != ESP_OK) return ret;
    rf_setup &= ~((1<<3) | (1<<5)); // 清除RF_DR_LOW和RF_DR_HIGH位
    switch (rate) {
        case NRF24_DR_1Mbps:
            // RF_DR_HIGH=0, RF_DR_LOW=0
            break;
        case NRF24_DR_2Mbps:
            rf_setup |= (1<<3); // RF_DR_HIGH=1
            break;
        case NRF24_DR_250Kbps:
            rf_setup |= (1<<5); // RF_DR_LOW=1
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return nrf24l01_write_register(dev, NRF24_REG_RF_SETUP, rf_setup);
}

esp_err_t nrf24l01_set_pa_level(nrf24l01_t *dev, nrf24_pa_level_t pa_level) {
    uint8_t rf_setup;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_RF_SETUP, &rf_setup);
    if (ret != ESP_OK) return ret;
    rf_setup &= ~((1<<1) | (1<<2)); // 清除RF_PWR位
    rf_setup |= (pa_level & 0x03) << 1;
    return nrf24l01_write_register(dev, NRF24_REG_RF_SETUP, rf_setup);
}

esp_err_t nrf24l01_set_retransmit(nrf24l01_t *dev, nrf24_ard_t ard, nrf24_arc_t arc) {
    uint8_t value = ((ard & 0x0F) << 4) | (arc & 0x0F);
    return nrf24l01_write_register(dev, NRF24_REG_SETUP_RETR, value);
}

esp_err_t nrf24l01_set_rx_addr_p0(nrf24l01_t *dev, const uint8_t *addr) {
    memcpy(dev->rx_pipe_addr[0], addr, NRF24_ADDR_WIDTH);
    return nrf24l01_write_register_multi(dev, NRF24_REG_RX_ADDR_P0, addr, NRF24_ADDR_WIDTH);
}

esp_err_t nrf24l01_set_rx_addr_p1(nrf24l01_t *dev, const uint8_t *addr) {
    memcpy(dev->rx_pipe_addr[1], addr, NRF24_ADDR_WIDTH);
    return nrf24l01_write_register_multi(dev, NRF24_REG_RX_ADDR_P1, addr, NRF24_ADDR_WIDTH);
}

esp_err_t nrf24l01_set_rx_addr_p2_5(nrf24l01_t *dev, uint8_t pipe, uint8_t addr_byte) {
    if (pipe < 2 || pipe > 5) return ESP_ERR_INVALID_ARG;
    dev->rx_pipe_addr[pipe][0] = addr_byte; // 仅存储低字节，其他字节与P1相同
    return nrf24l01_write_register(dev, NRF24_REG_RX_ADDR_P0 + pipe, addr_byte);
}

esp_err_t nrf24l01_set_tx_addr(nrf24l01_t *dev, const uint8_t *addr) {
    memcpy(dev->tx_addr, addr, NRF24_ADDR_WIDTH);
    return nrf24l01_write_register_multi(dev, NRF24_REG_TX_ADDR, addr, NRF24_ADDR_WIDTH);
}

esp_err_t nrf24l01_set_rx_payload_width(nrf24l01_t *dev, uint8_t pipe, uint8_t width) {
    if (pipe > 5) return ESP_ERR_INVALID_ARG;
    if (width > 32) return ESP_ERR_INVALID_ARG;
    return nrf24l01_write_register(dev, NRF24_REG_RX_PW_P0 + pipe, width);
}

esp_err_t nrf24l01_enable_auto_ack(nrf24l01_t *dev, uint8_t pipe, bool enable) {
    if (pipe > 5) return ESP_ERR_INVALID_ARG;
    uint8_t en_aa;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_EN_AA, &en_aa);
    if (ret != ESP_OK) return ret;
    if (enable) {
        en_aa |= (1 << pipe);
    } else {
        en_aa &= ~(1 << pipe);
    }
    return nrf24l01_write_register(dev, NRF24_REG_EN_AA, en_aa);
}

esp_err_t nrf24l01_enable_dyn_payload(nrf24l01_t *dev, uint8_t pipe, bool enable) {
    if (pipe > 5) return ESP_ERR_INVALID_ARG;
    // 首先使能FEATURE.EN_DPL
    uint8_t feature;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_FEATURE, &feature);
    if (ret != ESP_OK) return ret;
    feature |= (1<<2); // EN_DPL
    ret = nrf24l01_write_register(dev, NRF24_REG_FEATURE, feature);
    if (ret != ESP_OK) return ret;

    uint8_t dynpd;
    ret = nrf24l01_read_register(dev, NRF24_REG_DYNPD, &dynpd);
    if (ret != ESP_OK) return ret;
    if (enable) {
        dynpd |= (1 << pipe);
    } else {
        dynpd &= ~(1 << pipe);
    }
    return nrf24l01_write_register(dev, NRF24_REG_DYNPD, dynpd);
}

esp_err_t nrf24l01_set_rx_mode(nrf24l01_t *dev) {
    uint8_t config;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    config |= NRF24_CONFIG_PRIM_RX | NRF24_CONFIG_PWR_UP;
    ret = nrf24l01_write_register(dev, NRF24_REG_CONFIG, config);
    if (ret != ESP_OK) return ret;
    nrf24l01_ce_high(dev); // 开始监听
    vTaskDelay(pdMS_TO_TICKS(1)); // 稳定时间
    return ESP_OK;
}

esp_err_t nrf24l01_set_tx_mode(nrf24l01_t *dev) {
    uint8_t config;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    config &= ~NRF24_CONFIG_PRIM_RX;
    config |= NRF24_CONFIG_PWR_UP;
    ret = nrf24l01_write_register(dev, NRF24_REG_CONFIG, config);
    if (ret != ESP_OK) return ret;
    nrf24l01_ce_low(dev); // 确保CE低，准备发送
    return ESP_OK;
}

esp_err_t nrf24l01_send(nrf24l01_t *dev, const uint8_t *data, size_t len, int timeout_ms) {
    if (len == 0 || len > NRF24_MAX_PAYLOAD) return ESP_ERR_INVALID_ARG;

    // 确保处于发送模式（PRIM_RX=0, PWR_UP=1）
    uint8_t config;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    if (config & NRF24_CONFIG_PRIM_RX) {
        // 当前为接收模式，需切换到发送模式
        ret = nrf24l01_set_tx_mode(dev);
        if (ret != ESP_OK) return ret;
    }

    // 清空TX FIFO（可选，但为了可靠）
    nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_TX);

    // 写入发送载荷
    ret = nrf24l01_write_cmd_with_data(dev, NRF24_CMD_W_TX_PAYLOAD, data, len);
    if (ret != ESP_OK) return ret;

    // 触发发送：CE高 >10us
    nrf24l01_ce_high(dev);
    esp_rom_delay_us(15); // 至少10us
    nrf24l01_ce_low(dev);

    // 等待发送完成或超时
    TickType_t start = xTaskGetTickCount();
    uint8_t status;
    while (1) {
        status = nrf24l01_get_status(dev);
        if (status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) {
            break;
        }
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGW(TAG, "Send timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 清除中断标志
    nrf24l01_clear_irq(dev, status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));

    if (status & NRF24_STATUS_MAX_RT) {
        ESP_LOGW(TAG, "Max retries exceeded");
        return ESP_FAIL; // 重传超时
    }

    return ESP_OK;
}

bool nrf24l01_data_ready(nrf24l01_t *dev, uint8_t *pipe) {
    uint8_t status = nrf24l01_get_status(dev);
    if (status & NRF24_STATUS_RX_DR) {
        if (pipe) {
            *pipe = (status & NRF24_STATUS_RX_P_NO) >> 1;
        }
        return true;
    }
    return false;
}

esp_err_t nrf24l01_receive(nrf24l01_t *dev, uint8_t *data, size_t *len, uint8_t *pipe) {
    if (!nrf24l01_data_ready(dev, pipe)) {
        return ESP_ERR_NOT_FOUND;
    }

    // 读取有效载荷长度（如果动态载荷使能）
    uint8_t payload_len = NRF24_MAX_PAYLOAD;
    // 检查FEATURE.EN_DPL是否使能
    uint8_t feature;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_FEATURE, &feature);
    if (ret == ESP_OK && (feature & (1<<2))) {
        // 动态载荷使能，使用R_RX_PL_WID读取长度
        uint8_t cmd = NRF24_CMD_R_RX_PL_WID;
        uint8_t rx_len[2];
        spi_transaction_t t = {
            .length = 2 * 8,
            .tx_buffer = &cmd,
            .rx_buffer = rx_len,
        };
        ret = spi_device_transmit(dev->spi, &t);
        if (ret == ESP_OK) {
            payload_len = rx_len[1];
            if (payload_len > NRF24_MAX_PAYLOAD) {
                ESP_LOGE(TAG, "Invalid payload length %d", payload_len);
                // 清空无效载荷
                nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_RX);
                return ESP_FAIL;
            }
        }
    }

    // 读取载荷
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
    uint8_t *rx_buf = malloc(1 + payload_len);
    if (!rx_buf) return ESP_ERR_NO_MEM;
    uint8_t *tx_buf = malloc(1 + payload_len);
    if (!tx_buf) {
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    tx_buf[0] = cmd;
    memset(tx_buf + 1, NRF24_CMD_NOP, payload_len);
    spi_transaction_t t = {
        .length = (1 + payload_len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, rx_buf + 1, payload_len);
        *len = payload_len;
    }
    free(tx_buf);
    free(rx_buf);

    // 清除RX_DR中断
    nrf24l01_clear_irq(dev, NRF24_STATUS_RX_DR);
    return ret;
}

uint8_t nrf24l01_get_status(nrf24l01_t *dev) {
    uint8_t cmd = NRF24_CMD_NOP;
    uint8_t status;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .rx_buffer = &status,
    };
    spi_device_transmit(dev->spi, &t);
    return status;
}

void nrf24l01_clear_irq(nrf24l01_t *dev, uint8_t flags) {
    uint8_t status = nrf24l01_get_status(dev);
    // 写1清除对应位
    nrf24l01_write_register(dev, NRF24_REG_STATUS, status | flags);
}

esp_err_t nrf24l01_power_up(nrf24l01_t *dev) {
    uint8_t config;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    config |= NRF24_CONFIG_PWR_UP;
    return nrf24l01_write_register(dev, NRF24_REG_CONFIG, config);
}

esp_err_t nrf24l01_power_down(nrf24l01_t *dev) {
    uint8_t config;
    esp_err_t ret = nrf24l01_read_register(dev, NRF24_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    config &= ~NRF24_CONFIG_PWR_UP;
    return nrf24l01_write_register(dev, NRF24_REG_CONFIG, config);
}

void nrf24l01_dump_registers(nrf24l01_t *dev) {
    uint8_t val;
    const char *reg_names[] = {
        "CONFIG", "EN_AA", "EN_RXADDR", "SETUP_AW", "SETUP_RETR",
        "RF_CH", "RF_SETUP", "STATUS", "OBSERVE_TX", "RPD",
        "RX_ADDR_P0", "RX_ADDR_P1", "RX_ADDR_P2", "RX_ADDR_P3",
        "RX_ADDR_P4", "RX_ADDR_P5", "TX_ADDR", "RX_PW_P0",
        "RX_PW_P1", "RX_PW_P2", "RX_PW_P3", "RX_PW_P4", "RX_PW_P5",
        "FIFO_STATUS", "", "", "", "DYNPD", "FEATURE"
    };
    for (int i = 0; i <= 0x1D; i++) {
        if (i >= 0x18 && i <= 0x1B) continue; // 保留
        if (nrf24l01_read_register(dev, i, &val) == ESP_OK) {
            ESP_LOGI(TAG, "0x%02X %-12s = 0x%02X", i, reg_names[i], val);
        }
    }
}