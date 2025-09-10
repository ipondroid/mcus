
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "esp_log.h"

#include "state_manager.h"
#include "spi_task.h"
#include "ble_task.h"
#include "wifi_task.h"
#include "mqtt_task.h"
#include "events.h"
#include "commands.h"

// static const char* TAG = "AppMain";
#define TAG "AppMain"

QueueHandle_t g_xEventQueue = NULL;
QueueHandle_t g_xBleCommandQueue = NULL;

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    g_xEventQueue = xQueueCreate(10, sizeof(Event_t));
    g_xBleCommandQueue = xQueueCreate(5, sizeof(Command_t));

    StateManager_CreateTask();
    SpiTask_CreateTask();
    BleTask_CreateTask();
    WiFiTask_CreateTask();
    MqttTask_CreateTask();

    ESP_LOGI(TAG, "Tasks and Queues created.");
}
