#include "state_manager.h"
#include "system_state.h"
#include "events.h"
#include "commands.h"
#include "shared_types.h"
#include "ble_task.h"
#include "mqtt_task.h"
#include "wifi_task.h"
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
                BleTransmissionMode_t transmission_mode = get_ble_transmission_mode();
                if (transmission_mode == BLE_TRANSMISSION_MODE_REALTIME) {
                    // Send notification only if CCCD is enabled (realtime mode)
                    Command_t bleCommand;
                    bleCommand.eType = CMD_BLE_NOTIFY_DATA;
                    bleCommand.xSensorData = pxEvent->xSensorData;

                    if (xQueueSend(g_xBleCommandQueue, &bleCommand, 0) != pdPASS) {
                        ESP_LOGE(TAG, "Failed to send command to BleTask");
                    }
                    ESP_LOGI(TAG, "BLE data sent in realtime mode");
                } else {
                    ESP_LOGI(TAG, "BLE in on-request mode, data stored but not sent");
                }
            }

            // Send to MQTT if connected
            if (mqtt_is_connected()) {
                esp_err_t ret = mqtt_publish_sensor_data(&pxEvent->xSensorData);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Sensor data sent to MQTT broker");
                } else {
                    ESP_LOGE(TAG, "Failed to send sensor data to MQTT broker");
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
        case EVT_WIFI_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            break;
        case EVT_WIFI_DISCONNECTED:
            ESP_LOGI(TAG, "WiFi disconnected");
            break;
        case EVT_MQTT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case EVT_MQTT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            break;
        case EVT_MQTT_DATA_SENT:
            ESP_LOGI(TAG, "MQTT data sent successfully");
            break;
            
        case EVT_WIFI_CONFIG_RECEIVED:
            ESP_LOGI(TAG, "WiFi configuration received");
            break;
            
        case EVT_WIFI_CONFIG_CONNECT_REQ: {
            ESP_LOGI(TAG, "WiFi connection request: SSID=%s", pxEvent->xWiFiConfig.ssid);
            esp_err_t ret = wifi_connect_with_credentials(pxEvent->xWiFiConfig.ssid, pxEvent->xWiFiConfig.password);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start WiFi connection: %s", esp_err_to_name(ret));
            }
            break;
        }
        
        case EVT_WIFI_CONFIG_DISCONNECT_REQ:
            ESP_LOGI(TAG, "WiFi disconnect request");
            esp_err_t ret = wifi_disconnect();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
            }
            break;
            
        case EVT_MQTT_CONFIG_RECEIVED:
            ESP_LOGI(TAG, "MQTT configuration received");
            break;
            
        case EVT_MQTT_CONFIG_SET_REQ: {
            ESP_LOGI(TAG, "MQTT configuration set request: broker=%s, client_id=%s", 
                     pxEvent->xMqttConfig.broker_uri, pxEvent->xMqttConfig.client_id);
            
            set_mqtt_config(&pxEvent->xMqttConfig);
            
            esp_err_t ret = mqtt_reconfigure(&pxEvent->xMqttConfig);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reconfigure MQTT client: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "MQTT client reconfigured successfully");
            }
            break;
        }
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
