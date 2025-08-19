#include "state_manager.h"
#include "system_state.h"
#include "events.h"
#include "commands.h"
#include "shared_types.h"
#include "ble_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "StateManager";

SystemState_t g_SystemState;

static TaskHandle_t g_xStateManagerTaskHandle = NULL;

static void prvHandleEvent(Event_t* pxEvent);

static void prvStateManagerTask(void* pvParameters) {
    Event_t xReceivedEvent;

    ESP_LOGI(TAG, "Task started");

    memset(&g_SystemState, 0, sizeof(SystemState_t));
    g_SystemState.eSystemMode = MODE_INITIALIZING;

    g_SystemState.eSystemMode = MODE_ADVERTISING;
    ESP_LOGI(TAG, "System mode: ADVERTISING");

    for (;;) {
        if (xQueueReceive(g_xEventQueue, &xReceivedEvent, portMAX_DELAY) == pdPASS) {
            prvHandleEvent(&xReceivedEvent);
        }
    }
}

static void prvHandleEvent(Event_t* pxEvent) {
    ESP_LOGI(TAG, "Event received: %d", pxEvent->eType);

    switch (pxEvent->eType) {
        case EVT_SPI_DATA_RECEIVED: {
            g_SystemState.fLastTemperature = pxEvent->xSensorData.fTemperature;
            g_SystemState.fLastHumidity = pxEvent->xSensorData.fHumidity;
            ESP_LOGI(TAG, "State updated: T=%.2f, H=%.2f", g_SystemState.fLastTemperature, g_SystemState.fLastHumidity);

            ESP_LOGI(TAG, "bIsBleClientConnected: %d", g_SystemState.bIsBleClientConnected);
            if (g_SystemState.bIsBleClientConnected) {
                Command_t bleCommand;
                bleCommand.eType = CMD_BLE_NOTIFY_DATA;
                bleCommand.xSensorData = pxEvent->xSensorData;

                if (xQueueSend(g_xBleCommandQueue, &bleCommand, 0) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to send command to BleTask");
                }
            }
            break;
        }
        case EVT_BLE_CLIENT_CONNECTED:
            g_SystemState.eSystemMode = MODE_CONNECTED;
            g_SystemState.bIsBleClientConnected = true;
            ESP_LOGI(TAG, "System mode: CONNECTED");
            break;
        case EVT_BLE_CLIENT_DISCONNECTED:
            g_SystemState.eSystemMode = MODE_ADVERTISING;
            g_SystemState.bIsBleClientConnected = false;
            ESP_LOGI(TAG, "System mode: ADVERTISING");
            break;
        default:
            break;
    }
}

BaseType_t StateManager_CreateTask(void) {
    return xTaskCreate(
        prvStateManagerTask,            // Task function
        "StateManager",                 // Task name
        4096,                           // Stack size
        NULL,                           // Parameters
        5,                              // Priority
        &g_xStateManagerTaskHandle      // Task handle
    );
}
