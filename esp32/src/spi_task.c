#include "spi_task.h"
#include "state_manager.h"
#include "events.h"
#include "shared_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#else
#define RCV_HOST    SPI2_HOST
#endif

static const char* TAG = "SpiTask";

#define GPIO_HANDSHAKE      2
#define GPIO_MOSI           6
#define GPIO_MISO           5
#define GPIO_SCLK           4
#define GPIO_CS             7

#define BLINK_GPIO 8

static bool g_led_state = false;

void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

static void blink_led(void)
{
    g_led_state = !g_led_state;
    gpio_set_level(BLINK_GPIO, g_led_state);
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void spi_slave_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    gpio_config(&io_conf);
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
}

static void prvSpiTask(void* pvParameters) {
    esp_err_t ret;

    ESP_LOGI(TAG, "SPIT");

    configure_led();

    spi_slave_init();

    WORD_ALIGNED_ATTR uint8_t rx_buf[4]; // 4-byte receive buffer
    spi_slave_transaction_t trans = {0};
    trans.length = sizeof(rx_buf) * 8;
    trans.rx_buffer = rx_buf;

    for(;;) {
        memset(rx_buf, 0, sizeof(rx_buf));
        
        ret = spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received %d bytes from SPI", trans.trans_len / 8);

            // Process only if the received data is 4 bytes
            if (trans.trans_len == 32) {
                Event_t newEvent;
                newEvent.eType = EVT_SPI_DATA_RECEIVED;

                newEvent.xSensorData.fTemperature = (float)rx_buf[0] + ((float)rx_buf[1] / 100.0f);
                newEvent.xSensorData.fHumidity = (float)rx_buf[2] + ((float)rx_buf[3] / 100.0f);

                ESP_LOGI(TAG, "Parsed Data: T=%.2f, H=%.2f", newEvent.xSensorData.fTemperature, newEvent.xSensorData.fHumidity);

                if (xQueueSend(g_xEventQueue, &newEvent, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to send event to queue.");
                }
            }
            blink_led();
        }
    }
}

BaseType_t SpiTask_CreateTask(void) {
    return xTaskCreate(
        prvSpiTask,                     // Task function
        "Spi_Task",                     // Task name
        4096,                           // Stack size
        NULL,                           // Parameters
        10,                             // Priority
        NULL                            // Task handle
    );
}
