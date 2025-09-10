#include "mqtt_task.h"
#include "events.h"
#include "wifi_task.h"
#include "ble_task.h"
#include "cJSON.h"
#include <time.h>
#include <sys/time.h>

static const char *TAG = "MQTT_Task";
static esp_mqtt_client_handle_t client = NULL;
static bool mqtt_connected = false;
static MqttConfigData_t current_mqtt_config;

extern QueueHandle_t g_xEventQueue;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        
        // Send MQTT connected event
        Event_t mqttEvent;
        mqttEvent.eType = EVT_MQTT_CONNECTED;
        xQueueSend(g_xEventQueue, &mqttEvent, pdMS_TO_TICKS(10));
        
        // Publish device status
        mqtt_publish_device_status("online");
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        
        // Send MQTT disconnected event
        Event_t mqttDisconnectEvent;
        mqttDisconnectEvent.eType = EVT_MQTT_DISCONNECTED;
        xQueueSend(g_xEventQueue, &mqttDisconnectEvent, pdMS_TO_TICKS(10));
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        
        // Send MQTT data sent event
        Event_t mqttSentEvent;
        mqttSentEvent.eType = EVT_MQTT_DATA_SENT;
        xQueueSend(g_xEventQueue, &mqttSentEvent, pdMS_TO_TICKS(10));
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
        
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_init(void)
{
    // Get MQTT configuration from BLE task
    get_mqtt_config(&current_mqtt_config);
    
    // Use fallback values if configuration is empty
    if (strlen(current_mqtt_config.broker_uri) == 0) {
        strcpy(current_mqtt_config.broker_uri, MQTT_BROKER_URI);
    }
    if (strlen(current_mqtt_config.username) == 0) {
        strcpy(current_mqtt_config.username, MQTT_USERNAME);
    }
    if (strlen(current_mqtt_config.password) == 0) {
        strcpy(current_mqtt_config.password, MQTT_PASSWORD);
    }
    if (strlen(current_mqtt_config.client_id) == 0) {
        strcpy(current_mqtt_config.client_id, MQTT_CLIENT_ID);
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = current_mqtt_config.broker_uri,
        .credentials.username = current_mqtt_config.username,
        .credentials.authentication.password = current_mqtt_config.password,
        .credentials.client_id = current_mqtt_config.client_id,
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 5000,
        .network.timeout_ms = 5000,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    ESP_LOGI(TAG, "MQTT initialized with broker: %s", current_mqtt_config.broker_uri);
}

esp_err_t mqtt_reconfigure(const MqttConfigData_t* new_config)
{
    if (new_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Reconfiguring MQTT with broker: %s", new_config->broker_uri);
    
    // Stop current client if running
    if (client != NULL) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
        mqtt_connected = false;
    }

    // Update current configuration
    memcpy(&current_mqtt_config, new_config, sizeof(MqttConfigData_t));

    // Create new client with updated configuration
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = current_mqtt_config.broker_uri,
        .credentials.username = current_mqtt_config.username,
        .credentials.authentication.password = current_mqtt_config.password,
        .credentials.client_id = current_mqtt_config.client_id,
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 5000,
        .network.timeout_ms = 5000,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client with new configuration");
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // Start the new client if WiFi is connected
    if (wifi_is_connected()) {
        esp_err_t ret = esp_mqtt_client_start(client);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start MQTT client with new configuration: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "MQTT client started with new configuration");
    }
    
    return ESP_OK;
}

bool mqtt_is_connected(void)
{
    return mqtt_connected;
}

static uint64_t get_timestamp_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

esp_err_t mqtt_publish_sensor_data(const SensorData_t* sensor_data)
{
    if (!mqtt_is_connected() || sensor_data == NULL) {
        return ESP_FAIL;
    }

    cJSON *json = cJSON_CreateObject();
    cJSON *device_id = cJSON_CreateString(current_mqtt_config.client_id);
    cJSON *timestamp = cJSON_CreateNumber(get_timestamp_ms());
    cJSON *data = cJSON_CreateObject();
    cJSON *temperature = cJSON_CreateNumber(sensor_data->fTemperature);
    cJSON *humidity = cJSON_CreateNumber(sensor_data->fHumidity);

    cJSON_AddItemToObject(json, "device_id", device_id);
    cJSON_AddItemToObject(json, "timestamp", timestamp);
    cJSON_AddItemToObject(data, "temperature", temperature);
    cJSON_AddItemToObject(data, "humidity", humidity);
    cJSON_AddItemToObject(json, "data", data);

    char *json_string = cJSON_Print(json);
    if (json_string == NULL) {
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_SENSOR_DATA, json_string, 0, 1, 0);
    
    ESP_LOGI(TAG, "Published sensor data: %s", json_string);
    
    free(json_string);
    cJSON_Delete(json);

    return (msg_id != -1) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_publish_device_status(const char* status)
{
    if (!mqtt_is_connected() || status == NULL) {
        return ESP_FAIL;
    }

    cJSON *json = cJSON_CreateObject();
    cJSON *device_id = cJSON_CreateString(current_mqtt_config.client_id);
    cJSON *timestamp = cJSON_CreateNumber(get_timestamp_ms());
    cJSON *status_obj = cJSON_CreateString(status);

    cJSON_AddItemToObject(json, "device_id", device_id);
    cJSON_AddItemToObject(json, "timestamp", timestamp);
    cJSON_AddItemToObject(json, "status", status_obj);

    char *json_string = cJSON_Print(json);
    if (json_string == NULL) {
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_DEVICE_STATUS, json_string, 0, 1, 0);
    
    ESP_LOGI(TAG, "Published device status: %s", json_string);
    
    free(json_string);
    cJSON_Delete(json);

    return (msg_id != -1) ? ESP_OK : ESP_FAIL;
}

static void prvMqttTask(void* pvParameters)
{
    ESP_LOGI(TAG, "MQTT Task started");
    
    while (!wifi_is_connected()) {
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    esp_err_t ret = esp_mqtt_client_start(client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    while(1) {
        if (wifi_is_connected() && !mqtt_is_connected()) {
            ESP_LOGW(TAG, "WiFi connected but MQTT disconnected, attempting reconnection...");
            esp_mqtt_client_reconnect(client);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void MqttTask_CreateTask(void)
{
    mqtt_init();
    
    xTaskCreate(prvMqttTask, "mqtt_task", 8192, NULL, 3, NULL);
}
