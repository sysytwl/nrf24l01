#include "nrf_tx.h"
#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

// SPI引脚配置（根据实际接线修改） 
#define PIN_CS           GPIO_NUM_15
#define PIN_CE           GPIO_NUM_23  // 修改：使用实际的GPIO引脚而不是NC
#define PIN_IRQ          GPIO_NUM_NC  // 不使用可设为-1

// 地址示例（5字节，LSB先发送）
static const uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t rx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

nrf24l01_t nrf;

void nrf_tx_init(void) {
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

    // 切换到发送模式
    nrf24l01_set_tx_mode(&nrf);
    
    ESP_LOGI("nrf_tx", "Transmitter initialized and in TX mode");
}

void nrf_tx(uint8_t *data){
    // 创建要发送的数据
    uint8_t send_data[20];
    strcpy((char*)send_data, "Hello nRF24L01+!");
    
    esp_err_t ret;
    do {
        ret = nrf24l01_send(&nrf, send_data, 32, 1000);

        // 清除中断标志
        //nrf24l01_clear_irq(&nrf, 0xFF);
    } while (ret != ESP_OK);

}