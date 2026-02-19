#include "nrf_tx.h"
#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"



// SPI引脚配置（根据实际接线修改） 
#define PIN_CS           GPIO_NUM_15
#define PIN_CE           GPIO_NUM_NC
#define PIN_IRQ          GPIO_NUM_NC  // 不使用可设为-1

// 地址示例（5字节，LSB先发送）
static const uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t rx_addr[5] = {0x00, 0x00, 0x00, 0x00, 0x01};

nrf24l01_t nrf;
void nrf_tx_init(void) {
    memset(&nrf, 0, sizeof(nrf));

    // 初始化SPI总线
    spi_bus_config_t buscfg;
        memset(&buscfg, 0, sizeof(spi_bus_config_t));
        buscfg.miso_io_num = GPIO_NUM_12;
        buscfg.mosi_io_num = GPIO_NUM_13;
        buscfg.sclk_io_num = GPIO_NUM_14;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = 64;
        buscfg.intr_flags = 0;
        buscfg.data_io_default_level = false;
        buscfg.data4_io_num = GPIO_NUM_NC;
        buscfg.data5_io_num = GPIO_NUM_NC;
        buscfg.data6_io_num = GPIO_NUM_NC;
        buscfg.data7_io_num = GPIO_NUM_NC;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    // 添加SPI设备
    spi_device_interface_config_t devcfg;
        memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
        devcfg.mode = 0;// CPOL=0, CPHA=0
        devcfg.clock_speed_hz = 8*1000*1000;
        devcfg.spics_io_num = PIN_CS;
        devcfg.queue_size = 7;
        //devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &nrf.spi));

    ESP_ERROR_CHECK(nrf24l01_init(&nrf, SPI2_HOST, 0, PIN_CS, PIN_CE, PIN_IRQ));

    // 设置地址
    nrf24l01_set_tx_addr(&nrf, tx_addr);
    nrf24l01_set_rx_addr_p0(&nrf, rx_addr);
    nrf24l01_set_rx_addr_p1(&nrf, rx_addr); // 示例使用相同地址
    // 启用管道0自动应答
    nrf24l01_enable_auto_ack(&nrf, 0, true);
    // 设置管道0有效载荷宽度（固定长度）
    nrf24l01_set_rx_payload_width(&nrf, 0, 32);
    // 设置RF通道
    nrf24l01_set_channel(&nrf, 125);
    // 设置数据速率和功率
    nrf24l01_set_datarate(&nrf, NRF24_DR_1Mbps);
    nrf24l01_set_pa_level(&nrf, NRF24_PA_MAX);

    nrf24l01_set_tx_mode(&nrf);
}

void nrf_tx(uint8_t *data){
    uint8_t send_data[32] = "Hello nRF24L01+!";
    esp_err_t ret = nrf24l01_send(&nrf, send_data, strlen((char*)send_data) + 1, 1000);
    if (ret == ESP_OK) {
        ESP_LOGI("TAG", "Send OK");
    } else {
        ESP_LOGE("TAG", "Send failed");
    }
}