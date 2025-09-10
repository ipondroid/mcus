#include "wifi_task.h"
#include "events.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "wifi_config.h"
#include "commands.h"

#include <string.h>

static const char *TAG = "WiFi_Task";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

extern QueueHandle_t g_xEventQueue;
extern QueueHandle_t g_xBleCommandQueue;

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");

        Event_t wifiEvent;
        wifiEvent.eType = EVT_WIFI_DISCONNECTED;
        xQueueSend(g_xEventQueue, &wifiEvent, pdMS_TO_TICKS(10));

        Command_t bleCommand;
        bleCommand.eType = CMD_WIFI_STATUS_UPDATE;
        bleCommand.status = 0; // Disconnected
        xQueueSend(g_xBleCommandQueue, &bleCommand, pdMS_TO_TICKS(10));

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        Event_t wifiEvent;
        wifiEvent.eType = EVT_WIFI_CONNECTED;
        xQueueSend(g_xEventQueue, &wifiEvent, pdMS_TO_TICKS(10));

        Command_t bleCommand;
        bleCommand.eType = CMD_WIFI_STATUS_UPDATE;
        bleCommand.status = 1; // Connected
        xQueueSend(g_xBleCommandQueue, &bleCommand, pdMS_TO_TICKS(10));
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {0};
    char saved_ssid[MAX_SSID_LEN] = {0};
    char saved_password[MAX_PASSWORD_LEN] = {0};

    if (wifi_config_exists() && wifi_config_load(saved_ssid, saved_password) == ESP_OK) {
        ESP_LOGI(TAG, "Loading saved WiFi configuration: %s", saved_ssid);
        strncpy((char*)wifi_config.sta.ssid, saved_ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char*)wifi_config.sta.password, saved_password, sizeof(wifi_config.sta.password) - 1);
    } else {
        ESP_LOGI(TAG, "No saved WiFi config found, using default");
        strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    }

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

bool wifi_is_connected(void)
{
    if (s_wifi_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t wifi_connect_with_credentials(const char* ssid, const char* password)
{
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", ssid);

    esp_err_t ret = wifi_config_save(ssid, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_wifi_stop();

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    s_retry_num = 0;

    if (s_wifi_event_group) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t wifi_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting from WiFi");

    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
    }

    return ret;
}

uint8_t wifi_get_connection_status(void)
{
    return wifi_is_connected() ? 1 : 0;
}

static void prvWiFiTask(void* pvParameters)
{
    ESP_LOGI(TAG, "WiFi Task started");

    wifi_init_sta();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, attempting reconnection...");
            s_retry_num = 0;
            esp_wifi_connect();
        }
    }
}

void WiFiTask_CreateTask(void)
{
    xTaskCreate(prvWiFiTask, "wifi_task", 4096, NULL, 4, NULL);
}
