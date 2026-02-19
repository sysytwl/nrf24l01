#include "nrf_rx.h"
#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

// SPI引脚配置（根据实际接线修改） 
#define PIN_CS           GPIO_NUM_15
#define PIN_CE           GPIO_NUM_NC  // 修改：使用实际的GPIO引脚而不是NC
#define PIN_IRQ          GPIO_NUM_NC  // 不使用可设为-1

// 地址示例（5字节，LSB先发送）
static const uint8_t tx_addr[5] = {0x00, 0x00, 0x00, 0x00, 0x01};
static const uint8_t rx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

nrf24l01_t nrf;

void nrf_rx_init(void) {
    memset(&nrf, 0, sizeof(nrf));

    // 初始化SPI总线
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_12,
        .mosi_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 64,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    // 添加SPI设备
    spi_device_interface_config_t devcfg = {
        .mode = 0, // CPOL=0, CPHA=0
        .clock_speed_hz = 8*1000*1000,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &nrf.spi));

    ESP_ERROR_CHECK(nrf24l01_init(&nrf, PIN_CE, PIN_IRQ));

    // 设置发送和接收地址
    nrf24l01_set_tx_addr(&nrf, tx_addr);
    nrf24l01_set_rx_addr_p0(&nrf, rx_addr);
    nrf24l01_set_rx_addr_p1(&nrf, rx_addr);
    
    // 设置RF通道
    nrf24l01_set_channel(&nrf, 125);
    
    // 设置数据速率和功率
    nrf24l01_set_datarate(&nrf, NRF24_DR_1Mbps);
    nrf24l01_set_pa_level(&nrf, NRF24_PA_MAX);

    // 切换到接收模式
    nrf24l01_set_rx_mode(&nrf);
    
    ESP_LOGI("nrf_rx", "Receiver initialized and in RX mode");
}

void nrf_rx_tick(uint8_t *data){
    uint8_t pipe;
    if (nrf24l01_data_ready(&nrf, &pipe)) {
        size_t len;
        if (nrf24l01_receive(&nrf, data, &len, &pipe) == ESP_OK) {
            ESP_LOGI("nrf_rx", "Received %d bytes on pipe %d", len, pipe);
            ESP_LOG_BUFFER_HEX("nrf_rx", data, len);
            
            // 检查是否为有效数据
            if(len > 0) {
                // 确保字符串以null结尾
                if(len < 32) {
                    data[len] = '\0';
                    ESP_LOGI("nrf_rx", "Data as string: %s", data);
                }
            }
        } else {
            ESP_LOGE("nrf_rx", "Failed to receive data");
        }
    }
}