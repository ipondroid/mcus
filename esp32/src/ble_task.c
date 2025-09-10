#include "ble_task.h"
#include "state_manager.h"
#include "events.h"
#include "commands.h"
#include "shared_types.h"
#include "wifi_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"

#include <string.h>

static const char* TAG = "BleTask";

static float g_temperature = 0.0f;
static float g_humidity = 0.0f;
static SemaphoreHandle_t g_sensor_mutex = NULL;

// WiFi configuration storage
static char g_wifi_ssid[32] = {0};
static char g_wifi_password[64] = {0};
static uint8_t g_wifi_status = 0; // 0=disconnected, 1=connected
static SemaphoreHandle_t g_wifi_mutex = NULL;

// MQTT configuration storage
static char g_mqtt_broker_uri[128] = {0};
static char g_mqtt_username[32] = {0};
static char g_mqtt_password[64] = {0};
static char g_mqtt_client_id[32] = {0};
static SemaphoreHandle_t g_mqtt_mutex = NULL;

// BLE transmission mode management
static BleTransmissionMode_t g_ble_transmission_mode = BLE_TRANSMISSION_MODE_ON_REQUEST;
static SemaphoreHandle_t g_ble_mode_mutex = NULL;

#define GATTS_TAG "TH_SR"

static const uint16_t gatts_env_sensing_service_uuid = 0x181A; // UUID for the Environmental Sensing Service
static const uint16_t gatts_char_temperature_uuid = 0x2A6E; // UUID for the Temperature Measurement Characteristic
static const uint16_t gatts_char_humidity_uuid = 0x2A6F; // UUID for the Humidity Measurement Characteristic

// WiFi Configuration Service
// Service UUID: 6ba1c7a0-8123-4567-9abc-def012345678
static uint8_t wifi_config_service_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xa0, 0xc7, 0xa1, 0x6b
};

// SSID Write Characteristic UUID: 6ba1c7a1-8123-4567-9abc-def012345678
static uint8_t wifi_ssid_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xa1, 0xc7, 0xa1, 0x6b
};

// Password Write Characteristic UUID: 6ba1c7a2-8123-4567-9abc-def012345678
static uint8_t wifi_password_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xa2, 0xc7, 0xa1, 0x6b
};

// Status Read Characteristic UUID: 6ba1c7a3-8123-4567-9abc-def012345678
static uint8_t wifi_status_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xa3, 0xc7, 0xa1, 0x6b
};

// Command Write Characteristic UUID: 6ba1c7a4-8123-4567-9abc-def012345678
static uint8_t wifi_command_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xa4, 0xc7, 0xa1, 0x6b
};

// MQTT Broker Configuration Characteristic UUIDs
// MQTT Broker URI: 6ba1c7b0-8123-4567-9abc-def012345678
static uint8_t mqtt_broker_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xb0, 0xc7, 0xa1, 0x6b
};

// MQTT Username: 6ba1c7b1-8123-4567-9abc-def012345678
static uint8_t mqtt_username_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xb1, 0xc7, 0xa1, 0x6b
};

// MQTT Password: 6ba1c7b2-8123-4567-9abc-def012345678
static uint8_t mqtt_password_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xb2, 0xc7, 0xa1, 0x6b
};

// MQTT Client ID: 6ba1c7b3-8123-4567-9abc-def012345678
static uint8_t mqtt_client_id_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xb3, 0xc7, 0xa1, 0x6b
};

// MQTT Command: 6ba1c7b4-8123-4567-9abc-def012345678
static uint8_t mqtt_command_char_uuid[16] = {
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x67, 0x45, 0x23, 0x81, 0xb4, 0xc7, 0xa1, 0x6b
};
#define GATTS_DESCR_UUID     0x3333
#define GATTS_NUM_HANDLE     4

#define GATTS_DEVICE_NAME    "TH_DEV"
#define PROFILE_NUM          2
#define ENV_APP_ID           0x55
#define WIFI_APP_ID          0x56
#define ENV_SVC_INST_ID      0
#define WIFI_SVC_INST_ID     1

enum {
    ENV_PROFILE_IDX,
    WIFI_PROFILE_IDX,
};

// Environmental Sensing Service indices
enum {
    ENV_IDX_SVC,
    ENV_IDX_CHAR_TEMP,
    ENV_IDX_CHAR_TEMP_VAL,
    ENV_IDX_CHAR_TEMP_CFG,
    ENV_IDX_CHAR_HUMID,
    ENV_IDX_CHAR_HUMID_VAL,
    ENV_IDX_CHAR_HUMID_CFG,
    ENV_IDX_NB,
};

// WiFi Configuration Service indices
enum {
    WIFI_IDX_SVC,
    WIFI_IDX_CHAR_SSID,
    WIFI_IDX_CHAR_SSID_VAL,
    WIFI_IDX_CHAR_PASSWORD,
    WIFI_IDX_CHAR_PASSWORD_VAL,
    WIFI_IDX_CHAR_STATUS,
    WIFI_IDX_CHAR_STATUS_VAL,
    WIFI_IDX_CHAR_COMMAND,
    WIFI_IDX_CHAR_COMMAND_VAL,
    WIFI_IDX_CHAR_MQTT_BROKER,
    WIFI_IDX_CHAR_MQTT_BROKER_VAL,
    WIFI_IDX_CHAR_MQTT_USERNAME,
    WIFI_IDX_CHAR_MQTT_USERNAME_VAL,
    WIFI_IDX_CHAR_MQTT_PASSWORD,
    WIFI_IDX_CHAR_MQTT_PASSWORD_VAL,
    WIFI_IDX_CHAR_MQTT_CLIENT_ID,
    WIFI_IDX_CHAR_MQTT_CLIENT_ID_VAL,
    WIFI_IDX_CHAR_MQTT_COMMAND,
    WIFI_IDX_CHAR_MQTT_COMMAND_VAL,
    WIFI_IDX_NB,
};  

static uint8_t adv_config_done       = 0;
uint16_t env_handle_table[ENV_IDX_NB];
uint16_t wifi_handle_table[WIFI_IDX_NB];

// 0000181A-0000-1000-8000-00805F9B34FB
static uint8_t service_uuid[16] = {
    0xFB, 0x34, 0x9B, 0x5F,
    0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00,
    0x1A, 0x18, 0x00, 0x00
};

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  char_prop_read               = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t  char_prop_write              = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t  char_prop_read_notify        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

uint16_t cccd_value = 0;

#define SENSOR_NOTIFY_INTERVAL_MS  10000 // 10 seconds

uint16_t notify_conn_id = 0xFFFF;
esp_gatt_if_t notify_gatts_if = 0;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0xA0,
    .adv_int_max         = 0x190,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


static void env_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void wifi_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static struct gatts_profile_inst profile_table[PROFILE_NUM] = {
    [ENV_PROFILE_IDX] = {
        .gatts_cb = env_profile_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
    [WIFI_PROFILE_IDX] = {
        .gatts_cb = wifi_profile_handler, 
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// Environmental Sensing Service
static const esp_gatts_attr_db_t env_gatt_db[ENV_IDX_NB] = {
    [ENV_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(gatts_env_sensing_service_uuid), (uint8_t *)&gatts_env_sensing_service_uuid}},

    [ENV_IDX_CHAR_TEMP] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    [ENV_IDX_CHAR_TEMP_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_char_temperature_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), 0, NULL}},

    [ENV_IDX_CHAR_TEMP_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    [ENV_IDX_CHAR_HUMID] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    [ENV_IDX_CHAR_HUMID_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_char_humidity_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t)*2, 0, NULL}},

    [ENV_IDX_CHAR_HUMID_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},
};

// WiFi Configuration Service
static const esp_gatts_attr_db_t wifi_gatt_db[WIFI_IDX_NB] = {
    [WIFI_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(wifi_config_service_uuid), sizeof(wifi_config_service_uuid), wifi_config_service_uuid}},

    [WIFI_IDX_CHAR_SSID] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_SSID_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, wifi_ssid_char_uuid, ESP_GATT_PERM_WRITE,
            32, 0, NULL}},

    [WIFI_IDX_CHAR_PASSWORD] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_PASSWORD_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, wifi_password_char_uuid, ESP_GATT_PERM_WRITE,
            64, 0, NULL}},

    [WIFI_IDX_CHAR_STATUS] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read}},

    [WIFI_IDX_CHAR_STATUS_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, wifi_status_char_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), 0, NULL}},

    [WIFI_IDX_CHAR_COMMAND] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_COMMAND_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, wifi_command_char_uuid, ESP_GATT_PERM_WRITE,
            sizeof(uint8_t), 0, NULL}},

    [WIFI_IDX_CHAR_MQTT_BROKER] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_MQTT_BROKER_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, mqtt_broker_char_uuid, ESP_GATT_PERM_WRITE,
            128, 0, NULL}},

    [WIFI_IDX_CHAR_MQTT_USERNAME] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_MQTT_USERNAME_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, mqtt_username_char_uuid, ESP_GATT_PERM_WRITE,
            32, 0, NULL}},

    [WIFI_IDX_CHAR_MQTT_PASSWORD] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_MQTT_PASSWORD_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, mqtt_password_char_uuid, ESP_GATT_PERM_WRITE,
            64, 0, NULL}},

    [WIFI_IDX_CHAR_MQTT_CLIENT_ID] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_MQTT_CLIENT_ID_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, mqtt_client_id_char_uuid, ESP_GATT_PERM_WRITE,
            32, 0, NULL}},

    [WIFI_IDX_CHAR_MQTT_COMMAND] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_IDX_CHAR_MQTT_COMMAND_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, mqtt_command_char_uuid, ESP_GATT_PERM_WRITE,
            sizeof(uint8_t), 0, NULL}},
};


static void init_sensor_mutex(void) {
    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor mutex");
    }
}

static void init_wifi_mutex(void) {
    g_wifi_mutex = xSemaphoreCreateMutex();
    if (g_wifi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi mutex");
    }
}

static void init_mqtt_mutex(void) {
    g_mqtt_mutex = xSemaphoreCreateMutex();
    if (g_mqtt_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT mutex");
    }
    
    // Set default MQTT configuration
    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strcpy(g_mqtt_broker_uri, "mqtt://192.168.1.100:1883");
        strcpy(g_mqtt_username, "iot_user");
        strcpy(g_mqtt_password, "iot_password");
        strcpy(g_mqtt_client_id, "esp32_001");
        xSemaphoreGive(g_mqtt_mutex);
    }
}

static void init_ble_mode_mutex(void) {
    g_ble_mode_mutex = xSemaphoreCreateMutex();
    if (g_ble_mode_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE mode mutex");
    }
}

void get_mqtt_config(MqttConfigData_t* config) {
    if (config == NULL) return;
    
    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(config->broker_uri, g_mqtt_broker_uri, sizeof(config->broker_uri) - 1);
        strncpy(config->username, g_mqtt_username, sizeof(config->username) - 1);
        strncpy(config->password, g_mqtt_password, sizeof(config->password) - 1);
        strncpy(config->client_id, g_mqtt_client_id, sizeof(config->client_id) - 1);
        xSemaphoreGive(g_mqtt_mutex);
    }
}

void set_mqtt_config(const MqttConfigData_t* config) {
    if (config == NULL) return;
    
    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(g_mqtt_broker_uri, config->broker_uri, sizeof(g_mqtt_broker_uri) - 1);
        strncpy(g_mqtt_username, config->username, sizeof(g_mqtt_username) - 1);
        strncpy(g_mqtt_password, config->password, sizeof(g_mqtt_password) - 1);
        strncpy(g_mqtt_client_id, config->client_id, sizeof(g_mqtt_client_id) - 1);
        xSemaphoreGive(g_mqtt_mutex);
    }
}

void set_wifi_status(uint8_t status) {
    if (g_wifi_mutex != NULL && xSemaphoreTake(g_wifi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_wifi_status = status;
        xSemaphoreGive(g_wifi_mutex);
    }
}

uint8_t get_wifi_status(void) {
    uint8_t status = 0;
    if (g_wifi_mutex != NULL && xSemaphoreTake(g_wifi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status = g_wifi_status;
        xSemaphoreGive(g_wifi_mutex);
    }
    return status;
}

void set_temperature(float temperature) {
    if (g_sensor_mutex != NULL && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_temperature = temperature;
        xSemaphoreGive(g_sensor_mutex);
    }
}

float get_temperature(void) {
    float temp = 0.0f;
    if (g_sensor_mutex != NULL && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        temp = g_temperature;
        xSemaphoreGive(g_sensor_mutex);
    }
    return temp;
}

void set_humidity(float humidity) {
    if (g_sensor_mutex != NULL && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_humidity = humidity;
        xSemaphoreGive(g_sensor_mutex);
    }
}

float get_humidity(void) {
    float humid = 0.0f;
    if (g_sensor_mutex != NULL && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        humid = g_humidity;
        xSemaphoreGive(g_sensor_mutex);
    }
    return humid;
}

BleTransmissionMode_t get_ble_transmission_mode(void) {
    BleTransmissionMode_t mode = BLE_TRANSMISSION_MODE_ON_REQUEST;
    if (g_ble_mode_mutex != NULL && xSemaphoreTake(g_ble_mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        mode = g_ble_transmission_mode;
        xSemaphoreGive(g_ble_mode_mutex);
    }
    return mode;
}

void set_ble_transmission_mode(BleTransmissionMode_t mode) {
    if (g_ble_mode_mutex != NULL && xSemaphoreTake(g_ble_mode_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_ble_transmission_mode = mode;
        xSemaphoreGive(g_ble_mode_mutex);
        ESP_LOGI(TAG, "BLE transmission mode set to: %s", 
                 mode == BLE_TRANSMISSION_MODE_REALTIME ? "REALTIME" : "ON_REQUEST");
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            }else{
                ESP_LOGI(GATTS_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            }
            else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        default:
            break;
    }
}

static void env_profile_handler(esp_gatts_cb_event_t event, 
                               esp_gatt_if_t gatts_if, 
                               esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "ENV GATT server register, status %d, app_id %d, gatts_if %d", 
                     param->reg.status, param->reg.app_id, gatts_if);
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(GATTS_DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) {
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_ble_gatts_create_attr_tab(env_gatt_db, gatts_if, ENV_IDX_NB, ENV_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            if (param->read.need_rsp == false) {
                ESP_LOGI(GATTS_TAG, "No response needed for read event");
            } else {
                ESP_LOGI(GATTS_TAG, "Response needed for read event");
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                if (param->read.handle == env_handle_table[ENV_IDX_CHAR_TEMP_VAL]) {
                    float t = get_temperature();
                    int16_t temp = (uint16_t)(t * 100);
                    rsp.attr_value.len = sizeof(temp);
                    memcpy(rsp.attr_value.value, &temp, sizeof(temp));
                    ESP_LOGI(GATTS_TAG, "GATT read [CHAR_TEMP_VAL] Temperature: (%.2f)", t);
                } else if (param->read.handle == env_handle_table[ENV_IDX_CHAR_HUMID_VAL]) {
                    float h = get_humidity();
                    uint16_t humid = (uint16_t)(h * 100);
                    rsp.attr_value.len = sizeof(humid);
                    memcpy(rsp.attr_value.value, &humid, sizeof(humid));
                    ESP_LOGI(GATTS_TAG, "GATT read [CHAR_HUMID_VAL] Humidity: (%.2f)", h);
                } else {
                    rsp.attr_value.len = 0;
                }

                esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TAG, "ENV ESP_GATTS_WRITE_EVT");
            
            // CCCD handling for dual transmission mode
            if (param->write.handle == env_handle_table[ENV_IDX_CHAR_TEMP_CFG] || 
                param->write.handle == env_handle_table[ENV_IDX_CHAR_HUMID_CFG]) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                
                if (descr_value == 0x0001) {
                    ESP_LOGI(GATTS_TAG, "Notification enabled - switching to REALTIME mode");
                    set_ble_transmission_mode(BLE_TRANSMISSION_MODE_REALTIME);
                    
                    notify_conn_id = param->write.conn_id;
                    notify_gatts_if = gatts_if;
                    
                    // Send current sensor data immediately upon enabling notifications
                    float temp = get_temperature();
                    float humid = get_humidity();
                    if (temp != 0.0f || humid != 0.0f) {
                        int16_t temp_value = (int16_t)(temp * 100);
                        uint16_t humid_value = (uint16_t)(humid * 100);
                        
                        esp_ble_gatts_send_indicate(notify_gatts_if, notify_conn_id,
                                                    env_handle_table[ENV_IDX_CHAR_TEMP_VAL],
                                                    sizeof(temp_value), (uint8_t *)&temp_value, false);
                        esp_ble_gatts_send_indicate(notify_gatts_if, notify_conn_id,
                                                    env_handle_table[ENV_IDX_CHAR_HUMID_VAL],
                                                    sizeof(humid_value), (uint8_t *)&humid_value, false);
                        ESP_LOGI(GATTS_TAG, "Sent initial values: temp=%.2f, humid=%.2f", temp, humid);
                    }
                    
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(GATTS_TAG, "Indication enabled - switching to REALTIME mode");
                    set_ble_transmission_mode(BLE_TRANSMISSION_MODE_REALTIME);
                    notify_conn_id = param->write.conn_id;
                    notify_gatts_if = gatts_if;
                    
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(GATTS_TAG, "Notification disabled - switching to ON_REQUEST mode");
                    set_ble_transmission_mode(BLE_TRANSMISSION_MODE_ON_REQUEST);
                    notify_conn_id = 0xFFFF;
                    notify_gatts_if = 0;
                }
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_MTU_EVT:
        case ESP_GATTS_CONF_EVT:
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            esp_ble_gap_update_conn_params(&conn_params);

            Event_t newEvent = (Event_t){ .eType = EVT_BLE_CLIENT_CONNECTED };
            xQueueSend(g_xEventQueue, &newEvent, 0);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ENV ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            
            // Reset transmission mode to on-request when client disconnects
            set_ble_transmission_mode(BLE_TRANSMISSION_MODE_ON_REQUEST);
            notify_conn_id = 0xFFFF;
            notify_gatts_if = 0;
            
            newEvent = (Event_t){ .eType = EVT_BLE_CLIENT_DISCONNECTED };
            xQueueSend(g_xEventQueue, &newEvent, 0);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(GATTS_TAG, "ENV ESP_GATTS_CREAT_ATTR_TAB_EVT");
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != ENV_IDX_NB) {
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) doesn't equal to ENV_IDX_NB(%d)", 
                         param->add_attr_tab.num_handle, ENV_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
                memcpy(env_handle_table, param->add_attr_tab.handles, sizeof(env_handle_table));
                esp_ble_gatts_start_service(env_handle_table[ENV_IDX_SVC]);
            }
            break;
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

// WiFi Configuration Service handler
static void wifi_profile_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "WIFI GATT server register, status %d, app_id %d, gatts_if %d", 
                     param->reg.status, param->reg.app_id, gatts_if);
            esp_ble_gatts_create_attr_tab(wifi_gatt_db, gatts_if, WIFI_IDX_NB, WIFI_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            if (param->read.need_rsp == false) {
                ESP_LOGI(GATTS_TAG, "No response needed for read event");
            } else {
                ESP_LOGI(GATTS_TAG, "Response needed for read event");
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                if (param->read.handle == wifi_handle_table[WIFI_IDX_CHAR_STATUS_VAL]) {
                    uint8_t status = get_wifi_status();
                    rsp.attr_value.len = sizeof(status);
                    memcpy(rsp.attr_value.value, &status, sizeof(status));
                    ESP_LOGI(GATTS_TAG, "GATT read [CHAR_WIFI_STATUS_VAL] WiFi Status: %d", status);
                } else {
                    rsp.attr_value.len = 0;
                }

                esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TAG, "WIFI ESP_GATTS_WRITE_EVT");
            
            // WiFi Configuration Handling
            if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_SSID_VAL]) {
                if (param->write.len < sizeof(g_wifi_ssid)) {
                    if (g_wifi_mutex != NULL && xSemaphoreTake(g_wifi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_wifi_ssid, 0, sizeof(g_wifi_ssid));
                        memcpy(g_wifi_ssid, param->write.value, param->write.len);
                        g_wifi_ssid[param->write.len] = '\0';
                        xSemaphoreGive(g_wifi_mutex);
                        ESP_LOGI(GATTS_TAG, "WiFi SSID received: %s", g_wifi_ssid);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_PASSWORD_VAL]) {
                if (param->write.len < sizeof(g_wifi_password)) {
                    if (g_wifi_mutex != NULL && xSemaphoreTake(g_wifi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_wifi_password, 0, sizeof(g_wifi_password));
                        memcpy(g_wifi_password, param->write.value, param->write.len);
                        g_wifi_password[param->write.len] = '\0';
                        xSemaphoreGive(g_wifi_mutex);
                        ESP_LOGI(GATTS_TAG, "WiFi password received (length: %d)", param->write.len);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_COMMAND_VAL]) {
                if (param->write.len == 1) {
                    uint8_t command = param->write.value[0];
                    ESP_LOGI(GATTS_TAG, "WiFi command received: %d", command);
                    
                    // Send WiFi configuration event
                    Event_t wifi_event;
                    if (command == 1) { // Connect command
                        wifi_event.eType = EVT_WIFI_CONFIG_CONNECT_REQ;
                        if (g_wifi_mutex != NULL && xSemaphoreTake(g_wifi_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            strncpy(wifi_event.xWiFiConfig.ssid, g_wifi_ssid, sizeof(wifi_event.xWiFiConfig.ssid) - 1);
                            strncpy(wifi_event.xWiFiConfig.password, g_wifi_password, sizeof(wifi_event.xWiFiConfig.password) - 1);
                            wifi_event.xWiFiConfig.connect_cmd = command;
                            xSemaphoreGive(g_wifi_mutex);
                        }
                    } else if (command == 0) { // Disconnect command
                        wifi_event.eType = EVT_WIFI_CONFIG_DISCONNECT_REQ;
                        wifi_event.xWiFiConfig.connect_cmd = command;
                    }
                    xQueueSend(g_xEventQueue, &wifi_event, pdMS_TO_TICKS(10));
                }
            }
            // MQTT Configuration Handling
            else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_MQTT_BROKER_VAL]) {
                if (param->write.len < sizeof(g_mqtt_broker_uri)) {
                    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_mqtt_broker_uri, 0, sizeof(g_mqtt_broker_uri));
                        memcpy(g_mqtt_broker_uri, param->write.value, param->write.len);
                        g_mqtt_broker_uri[param->write.len] = '\0';
                        xSemaphoreGive(g_mqtt_mutex);
                        ESP_LOGI(GATTS_TAG, "MQTT Broker URI received: %s", g_mqtt_broker_uri);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_MQTT_USERNAME_VAL]) {
                if (param->write.len < sizeof(g_mqtt_username)) {
                    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_mqtt_username, 0, sizeof(g_mqtt_username));
                        memcpy(g_mqtt_username, param->write.value, param->write.len);
                        g_mqtt_username[param->write.len] = '\0';
                        xSemaphoreGive(g_mqtt_mutex);
                        ESP_LOGI(GATTS_TAG, "MQTT Username received: %s", g_mqtt_username);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_MQTT_PASSWORD_VAL]) {
                if (param->write.len < sizeof(g_mqtt_password)) {
                    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_mqtt_password, 0, sizeof(g_mqtt_password));
                        memcpy(g_mqtt_password, param->write.value, param->write.len);
                        g_mqtt_password[param->write.len] = '\0';
                        xSemaphoreGive(g_mqtt_mutex);
                        ESP_LOGI(GATTS_TAG, "MQTT password received (length: %d)", param->write.len);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_MQTT_CLIENT_ID_VAL]) {
                if (param->write.len < sizeof(g_mqtt_client_id)) {
                    if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memset(g_mqtt_client_id, 0, sizeof(g_mqtt_client_id));
                        memcpy(g_mqtt_client_id, param->write.value, param->write.len);
                        g_mqtt_client_id[param->write.len] = '\0';
                        xSemaphoreGive(g_mqtt_mutex);
                        ESP_LOGI(GATTS_TAG, "MQTT Client ID received: %s", g_mqtt_client_id);
                    }
                }
            } else if (param->write.handle == wifi_handle_table[WIFI_IDX_CHAR_MQTT_COMMAND_VAL]) {
                if (param->write.len == 1) {
                    uint8_t command = param->write.value[0];
                    ESP_LOGI(GATTS_TAG, "MQTT command received: %d", command);
                    
                    // Send MQTT configuration event
                    Event_t mqtt_event;
                    if (command == 1) { // Set configuration command
                        mqtt_event.eType = EVT_MQTT_CONFIG_SET_REQ;
                        if (g_mqtt_mutex != NULL && xSemaphoreTake(g_mqtt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            strncpy(mqtt_event.xMqttConfig.broker_uri, g_mqtt_broker_uri, sizeof(mqtt_event.xMqttConfig.broker_uri) - 1);
                            strncpy(mqtt_event.xMqttConfig.username, g_mqtt_username, sizeof(mqtt_event.xMqttConfig.username) - 1);
                            strncpy(mqtt_event.xMqttConfig.password, g_mqtt_password, sizeof(mqtt_event.xMqttConfig.password) - 1);
                            strncpy(mqtt_event.xMqttConfig.client_id, g_mqtt_client_id, sizeof(mqtt_event.xMqttConfig.client_id) - 1);
                            xSemaphoreGive(g_mqtt_mutex);
                        }
                        xQueueSend(g_xEventQueue, &mqtt_event, pdMS_TO_TICKS(10));
                    }
                }
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_MTU_EVT:
        case ESP_GATTS_CONF_EVT:
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            // Connection handling is done in env_profile_handler
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "WIFI ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(GATTS_TAG, "WIFI ESP_GATTS_CREAT_ATTR_TAB_EVT");
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != WIFI_IDX_NB) {
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) doesn't equal to WIFI_IDX_NB(%d)", 
                         param->add_attr_tab.num_handle, WIFI_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
                memcpy(wifi_handle_table, param->add_attr_tab.handles, sizeof(wifi_handle_table));
                esp_ble_gatts_start_service(wifi_handle_table[WIFI_IDX_SVC]);
            }
            break;
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            // Assign gatts_if based on app_id
            if (param->reg.app_id == ENV_APP_ID) {
                profile_table[ENV_PROFILE_IDX].gatts_if = gatts_if;
                ESP_LOGI(GATTS_TAG, "ENV app registered, gatts_if %d", gatts_if);
            } else if (param->reg.app_id == WIFI_APP_ID) {
                profile_table[WIFI_PROFILE_IDX].gatts_if = gatts_if;
                ESP_LOGI(GATTS_TAG, "WIFI app registered, gatts_if %d", gatts_if);
            }
        } else {
            ESP_LOGE(GATTS_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_table[idx].gatts_if) {
                if (profile_table[idx].gatts_cb) {
                    profile_table[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void ble_init() {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    // Register Environmental Sensing Service app
    ret = esp_ble_gatts_app_register(ENV_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "ENV app register error, error code = %x", ret);
        return;
    }

    // Register WiFi Configuration Service app
    ret = esp_ble_gatts_app_register(WIFI_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "WIFI app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

static void prvBleTask(void* pvParameters) {
    Command_t xReceivedCommand;
    ESP_LOGI(TAG, "BLET");

    init_sensor_mutex();
    init_wifi_mutex();
    init_mqtt_mutex();
    init_ble_mode_mutex();
    wifi_config_init();
    ble_init();

    for (;;) {
        if (xQueueReceive(g_xBleCommandQueue, &xReceivedCommand, portMAX_DELAY) == pdPASS) {
            if (xReceivedCommand.eType == CMD_BLE_NOTIFY_DATA) {
                SensorData_t* pData = &xReceivedCommand.xSensorData;
                ESP_LOGI(TAG, "Received CMD_BLE_NOTIFY_DATA: Temperature=%.2f, Humidity=%.2f",
                         pData->fTemperature, pData->fHumidity);
                         
                // Update stored sensor values
                set_humidity(pData->fHumidity);
                set_temperature(pData->fTemperature);
                
                // Send BLE notification if in realtime mode and connection is active
                BleTransmissionMode_t mode = get_ble_transmission_mode();
                if (mode == BLE_TRANSMISSION_MODE_REALTIME && 
                    notify_conn_id != 0xFFFF && notify_gatts_if != 0) {
                    
                    int16_t temp_value = (int16_t)(pData->fTemperature * 100);
                    uint16_t humid_value = (uint16_t)(pData->fHumidity * 100);
                    
                    esp_ble_gatts_send_indicate(notify_gatts_if, notify_conn_id,
                                                env_handle_table[ENV_IDX_CHAR_TEMP_VAL],
                                                sizeof(temp_value), (uint8_t *)&temp_value, false);
                    esp_ble_gatts_send_indicate(notify_gatts_if, notify_conn_id,
                                                env_handle_table[ENV_IDX_CHAR_HUMID_VAL],
                                                sizeof(humid_value), (uint8_t *)&humid_value, false);
                    ESP_LOGI(TAG, "BLE notification sent: temp=%.2f, humid=%.2f", 
                             pData->fTemperature, pData->fHumidity);
                } else if (mode == BLE_TRANSMISSION_MODE_ON_REQUEST) {
                    ESP_LOGI(TAG, "Data stored for on-request access: temp=%.2f, humid=%.2f", 
                             pData->fTemperature, pData->fHumidity);
                }
                
            } else if (xReceivedCommand.eType == CMD_WIFI_STATUS_UPDATE) {
                uint8_t status = xReceivedCommand.status;
                ESP_LOGI(TAG, "Received CMD_WIFI_STATUS_UPDATE: Status=%d", status);
                set_wifi_status(status);
            }
        }
    }
}

BaseType_t BleTask_CreateTask(void) {
    return xTaskCreate(prvBleTask, "Ble_Task", 4096, NULL, 5, NULL);
}
