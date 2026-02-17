# nrf24l01 driver for esp-idf
1. the driver is in nrf24l01.x file
2. other code is for the esp-fpv-cam 

## code example
``` cpp 

#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "NRF_TEST";

// SPI引脚配置（根据实际接线修改）
#define SPI_HOST        SPI2_HOST
#define SPI_CLK_SPEED    8000000  // 8MHz
#define PIN_CS           GPIO_NUM_5
#define PIN_CE           GPIO_NUM_18
#define PIN_IRQ          GPIO_NUM_19  // 不使用可设为-1

// 地址示例（5字节，LSB先发送）
static const uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t rx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

void app_main(void) {
    // 初始化SPI总线（假设已经初始化，也可在此完成）
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED));

    nrf24l01_t nrf;
    ESP_ERROR_CHECK(nrf24l01_init(&nrf, SPI_HOST, SPI_CLK_SPEED, PIN_CS, PIN_CE, PIN_IRQ));

    // 设置地址
    nrf24l01_set_tx_addr(&nrf, tx_addr);
    nrf24l01_set_rx_addr_p0(&nrf, rx_addr);
    nrf24l01_set_rx_addr_p1(&nrf, rx_addr); // 示例使用相同地址
    // 启用管道0自动应答
    nrf24l01_enable_auto_ack(&nrf, 0, true);
    // 设置管道0有效载荷宽度（固定长度）
    nrf24l01_set_rx_payload_width(&nrf, 0, 32);
    // 设置RF通道
    nrf24l01_set_channel(&nrf, 100);
    // 设置数据速率和功率
    nrf24l01_set_datarate(&nrf, NRF24_DR_1Mbps);
    nrf24l01_set_pa_level(&nrf, NRF24_PA_MAX);

    // 示例：发送模式
    nrf24l01_set_tx_mode(&nrf);
    uint8_t send_data[32] = "Hello nRF24L01+!";
    esp_err_t ret = nrf24l01_send(&nrf, send_data, strlen((char*)send_data) + 1, 1000);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Send OK");
    } else {
        ESP_LOGE(TAG, "Send failed");
    }

    // 切换到接收模式
    nrf24l01_set_rx_mode(&nrf);
    while (1) {
        uint8_t pipe;
        if (nrf24l01_data_ready(&nrf, &pipe)) {
            uint8_t recv_data[32];
            size_t len;
            if (nrf24l01_receive(&nrf, recv_data, &len, &pipe) == ESP_OK) {
                ESP_LOGI(TAG, "Received %d bytes from pipe %d: %s", len, pipe, recv_data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

```

## code example with rx irq
``` cpp

#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "NRF_IRQ";
static nrf24l01_t *g_nrf_dev = NULL;
static TaskHandle_t g_rx_task_handle = NULL;

// 中断服务程序
static void IRAM_ATTR nrf_irq_handler(void *arg) {
    // 发送通知给接收任务（不执行 SPI 操作）
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(g_rx_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// 接收任务
void nrf_rx_task(void *pvParameters) {
    nrf24l01_t *dev = (nrf24l01_t *)pvParameters;
    uint8_t data[32];
    size_t len;
    uint8_t pipe;

    while (1) {
        // 等待通知（无限期）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 检查是否有数据（由于中断可能因多个原因触发，最好再检查一下 FIFO 状态）
        // 注意：这里在任务上下文中，可以安全调用 SPI 函数
        while (nrf24l01_data_ready(dev, &pipe)) {
            esp_err_t ret = nrf24l01_receive(dev, data, &len, &pipe);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "IRQ: Received %d bytes from pipe %d: %.*s", len, pipe, len, data);
                // 处理数据...
            } else {
                ESP_LOGE(TAG, "Receive failed");
            }
        }

        // 可选：清除 IRQ 标志（驱动中的 nrf24l01_receive 已经清除了 RX_DR，但如果有其他中断源可能还需要清除）
        // nrf24l01_clear_irq(dev, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    }
}

void app_main(void) {
    // 初始化 SPI 总线（略，同之前）
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    // 初始化 nRF24L01+ 设备（传入 IRQ 引脚）
    nrf24l01_t nrf;
    ESP_ERROR_CHECK(nrf24l01_init(&nrf, SPI2_HOST, 8000000, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19));
    g_nrf_dev = &nrf; // 保存全局指针供 ISR 使用（如果 ISR 需要，但此处不需要直接操作 nrf）

    // 设置地址等（同之前）
    uint8_t tx_addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    uint8_t rx_addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    nrf24l01_set_tx_addr(&nrf, tx_addr);
    nrf24l01_set_rx_addr_p0(&nrf, rx_addr);
    nrf24l01_enable_auto_ack(&nrf, 0, true);
    nrf24l01_set_rx_payload_width(&nrf, 0, 32);
    nrf24l01_set_channel(&nrf, 100);
    nrf24l01_set_datarate(&nrf, NRF24_DR_1Mbps);
    nrf24l01_set_pa_level(&nrf, NRF24_PA_MAX);

    // 创建接收任务
    xTaskCreate(nrf_rx_task, "nrf_rx", 4096, &nrf, 10, &g_rx_task_handle);

    // 配置 IRQ 引脚中断
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_19), // 假设 IRQ 引脚是 GPIO19
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      // nRF24L01+ IRQ 是开漏，需要上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,        // 下降沿触发（IRQ 低电平有效）
    };
    gpio_config(&io_conf);

    // 安装 GPIO 中断服务（如果尚未安装）
    gpio_install_isr_service(0);

    // 添加 ISR 处理函数
    gpio_isr_handler_add(GPIO_NUM_19, nrf_irq_handler, NULL);

    // 设置为接收模式
    nrf24l01_set_rx_mode(&nrf);

    ESP_LOGI(TAG, "IRQ receiver started");

    // 主循环可做其他事情
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

```

## code example with tx irq
``` cpp

#include "nrf24l01.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "NRF_IRQ_TX";

// 设备句柄
static nrf24l01_t g_nrf;
// 任务句柄
static TaskHandle_t g_tx_task_handle = NULL;
static TaskHandle_t g_rx_task_handle = NULL;
// 用于发送同步的队列（或任务通知）
static QueueHandle_t g_tx_event_queue = NULL;

// 自定义事件类型
typedef enum {
    TX_EVENT_SUCCESS,   // 发送成功
    TX_EVENT_MAX_RT,    // 最大重传
    TX_EVENT_RX_DATA    // 收到数据（由接收任务处理）
} nrf_event_t;

// 发送任务参数结构
typedef struct {
    uint8_t data[32];
    size_t len;
    uint8_t retry_count;
    uint8_t channel;    // 当前使用信道
} tx_params_t;

// 中断服务程序
static void IRAM_ATTR nrf_irq_handler(void *arg) {
    // 发送通知给接收任务（接收任务负责读取状态并分发）
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(g_rx_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// 接收任务：处理所有中断事件（RX, TX, MAX_RT）
void nrf_event_task(void *pvParameters) {
    nrf24l01_t *dev = (nrf24l01_t *)pvParameters;
    uint8_t status;
    uint8_t data[32];
    size_t len;
    uint8_t pipe;

    while (1) {
        // 等待中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 读取状态寄存器（同时清除高三位中断标志？注意读 STATUS 不会清除，需要写清除）
        status = nrf24l01_get_status(dev);
        ESP_LOGD(TAG, "IRQ: STATUS=0x%02x", status);

        // 处理 RX 数据就绪
        if (status & NRF24_STATUS_RX_DR) {
            // 可能有多个数据包，循环读取
            while (nrf24l01_data_ready(dev, &pipe)) {
                if (nrf24l01_receive(dev, data, &len, &pipe) == ESP_OK) {
                    ESP_LOGI(TAG, "RX pipe %d: %.*s", pipe, len, data);
                    // 检查是否为切换信道命令
                    if (len >= 2 && data[0] == 0xAA) { // 自定义命令字节
                        uint8_t new_ch = data[1];
                        ESP_LOGI(TAG, "Received channel switch command -> %d", new_ch);
                        nrf24l01_set_channel(dev, new_ch);
                        // 可选：保存当前信道
                    }
                }
            }
            // 清除 RX_DR 标志（receive 已清除，但为防万一）
            nrf24l01_clear_irq(dev, NRF24_STATUS_RX_DR);
        }

        // 处理 TX 发送成功
        if (status & NRF24_STATUS_TX_DS) {
            ESP_LOGD(TAG, "TX_DS event");
            nrf24l01_clear_irq(dev, NRF24_STATUS_TX_DS);
            // 通知发送任务成功
            if (g_tx_task_handle) {
                xTaskNotify(g_tx_task_handle, TX_EVENT_SUCCESS, eSetValueWithOverwrite);
            }
        }

        // 处理最大重传
        if (status & NRF24_STATUS_MAX_RT) {
            ESP_LOGW(TAG, "MAX_RT event");
            nrf24l01_clear_irq(dev, NRF24_STATUS_MAX_RT);
            // 通知发送任务失败
            if (g_tx_task_handle) {
                xTaskNotify(g_tx_task_handle, TX_EVENT_MAX_RT, eSetValueWithOverwrite);
            }
        }
    }
}

// 发送任务（演示连续发送，并在 MAX_RT 时切换信道并发送切换命令）
void nrf_tx_task(void *pvParameters) {
    nrf24l01_t *dev = (nrf24l01_t *)pvParameters;
    uint32_t notification_value;
    uint8_t current_channel = 100; // 初始信道
    uint8_t tx_data[32] = "Hello with IRQ";
    size_t tx_len = strlen((char*)tx_data) + 1;
    int max_retries = 3; // 最大重传尝试（不同于 nRF 的 ARC，这里是软件重试次数）

    // 先设置为发送模式（但发送函数会确保模式正确，这里可以提前设好）
    nrf24l01_set_tx_mode(dev);

    while (1) {
        // 准备数据：正常数据包
        memcpy(tx_data, "Hello IRQ", 10); // 示例数据

        // 发送数据（非阻塞？这里使用阻塞发送但利用中断等待结果）
        // 先清空 TX FIFO
        nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_TX);
        // 写入载荷
        nrf24l01_write_cmd_with_data(dev, NRF24_CMD_W_TX_PAYLOAD, tx_data, tx_len);
        // 启动发送：CE 脉冲
        nrf24l01_ce_high(dev);
        esp_rom_delay_us(15);
        nrf24l01_ce_low(dev);

        // 等待中断通知（发送结果）
        if (xTaskNotifyWait(0, ULONG_MAX, &notification_value, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (notification_value == TX_EVENT_SUCCESS) {
                ESP_LOGI(TAG, "Send successful on channel %d", current_channel);
                // 发送成功，延时后继续发送下一个包
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            } else if (notification_value == TX_EVENT_MAX_RT) {
                ESP_LOGE(TAG, "MAX_RT occurred on channel %d", current_channel);
                // 达到硬件重传上限，执行软件重试策略
                if (--max_retries > 0) {
                    // 切换信道（循环 0-125）
                    current_channel = (current_channel + 1) % 126;
                    nrf24l01_set_channel(dev, current_channel);
                    ESP_LOGI(TAG, "Switched to channel %d and retry", current_channel);

                    // 发送切换信道命令给接收端（使用新信道）
                    uint8_t cmd[2] = {0xAA, current_channel}; // 命令+新信道号
                    // 注意：发送命令前要确保设备处于发送模式（当前模式可能还是RX？但我们刚可能因为MAX_RT未改变模式，最好检查）
                    nrf24l01_set_tx_mode(dev);
                    nrf24l01_write_cmd(dev, NRF24_CMD_FLUSH_TX);
                    nrf24l01_write_cmd_with_data(dev, NRF24_CMD_W_TX_PAYLOAD, cmd, sizeof(cmd));
                    nrf24l01_ce_high(dev);
                    esp_rom_delay_us(15);
                    nrf24l01_ce_low(dev);

                    // 再等待这个命令的发送结果（超时较短）
                    uint32_t cmd_notif;
                    if (xTaskNotifyWait(0, ULONG_MAX, &cmd_notif, pdMS_TO_TICKS(500)) == pdTRUE) {
                        if (cmd_notif == TX_EVENT_SUCCESS) {
                            ESP_LOGI(TAG, "Channel switch command sent");
                        } else {
                            ESP_LOGE(TAG, "Failed to send channel switch command");
                        }
                    }
                    // 然后重新发送原始数据（循环开始处会重新发送）
                    continue; // 回到循环开头，重新发送原始数据
                } else {
                    ESP_LOGE(TAG, "Max software retries exhausted, giving up");
                    // 可以进入错误处理或休眠
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    max_retries = 3; // 重置重试计数
                }
            }
        } else {
            ESP_LOGW(TAG, "Send timeout (no IRQ)");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 避免疯狂重试
    }
}

void app_main(void) {
    // 初始化 SPI 总线（与之前相同）
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    // 初始化 nRF24L01+
    ESP_ERROR_CHECK(nrf24l01_init(&g_nrf, SPI2_HOST, 8000000, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19));

    // 设置地址（与接收端匹配）
    uint8_t tx_addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    uint8_t rx_addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    nrf24l01_set_tx_addr(&g_nrf, tx_addr);
    nrf24l01_set_rx_addr_p0(&g_nrf, rx_addr);
    nrf24l01_enable_auto_ack(&g_nrf, 0, true);
    nrf24l01_set_rx_payload_width(&g_nrf, 0, 32);
    nrf24l01_set_channel(&g_nrf, 100);
    nrf24l01_set_datarate(&g_nrf, NRF24_DR_1Mbps);
    nrf24l01_set_pa_level(&g_nrf, NRF24_PA_MAX);

    // 创建事件队列（可选，这里使用任务通知）
    g_tx_event_queue = xQueueCreate(5, sizeof(uint32_t)); // 如果要用队列

    // 创建接收事件处理任务（高优先级，以便快速响应中断）
    xTaskCreatePinnedToCore(nrf_event_task, "nrf_evt", 4096, &g_nrf, 15, &g_rx_task_handle, 0);

    // 创建发送任务
    xTaskCreatePinnedToCore(nrf_tx_task, "nrf_tx", 4096, &g_nrf, 10, &g_tx_task_handle, 1);

    // 配置 IRQ 引脚中断
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_19),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_19, nrf_irq_handler, NULL);

    // 启动接收模式（事件任务会处理 RX，但发送任务会临时切换模式）
    nrf24l01_set_rx_mode(&g_nrf);

    ESP_LOGI(TAG, "System started, waiting for IRQ...");

    // 主循环空闲
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

```