
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

#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"


#define SENSOR_TAG "SENSOR"

static float g_temperature = 0.0f;
static float g_humidity = 0.0f;
static SemaphoreHandle_t g_sensor_mutex = NULL;

static void init_sensor_mutex(void) {
    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        ESP_LOGE(SENSOR_TAG, "Failed to create sensor mutex");
    }
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

#define GATTS_TAG "TH_SR"

static const uint16_t gatts_env_sensing_service_uuid = 0x181A; // UUID for the Environmental Sensing Service
static const uint16_t gatts_char_temperature_uuid = 0x2A6E; // UUID for the Temperature Measurement Characteristic
static const uint16_t gatts_char_humidity_uuid = 0x2A6F; // UUID for the Humidity Measurement Characteristic
#define GATTS_DESCR_UUID     0x3333
#define GATTS_NUM_HANDLE     4

#define GATTS_DEVICE_NAME    "TH_DEV"
#define PROFILE_NUM          1
#define PROFILE_APP_IDX      0
#define APP_ID               0x55
#define SVC_INST_ID          0

enum {
    IDX_SVC,
    IDX_CHAR_TEMP,
    IDX_CHAR_TEMP_VAL,
    IDX_CHAR_TEMP_CFG,
    IDX_CHAR_HUMID,
    IDX_CHAR_HUMID_VAL,
    IDX_CHAR_HUMID_CFG,
    HRS_IDX_NB,
};

static uint8_t adv_config_done       = 0;
uint16_t handle_table[HRS_IDX_NB];

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
// static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t  char_prop_read_notify        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint8_t temperature_value[2]          = {0x00, 0x00};
static uint8_t humidity_value[2]             = {0x11, 0x22};

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

uint16_t cccd_value = 0;

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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst profile_table[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(gatts_env_sensing_service_uuid), (uint8_t *)&gatts_env_sensing_service_uuid}},

    // Temperature Characteristic Declaration
    [IDX_CHAR_TEMP] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    // Temperature Characteristic Value
    [IDX_CHAR_TEMP_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_char_temperature_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), 0, NULL}},

    // Client Characteristic Configuration Descriptor (CCCD)
    [IDX_CHAR_TEMP_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},

    // Humidity Characteristic Declaration
    [IDX_CHAR_HUMID] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    // Humidity Characteristic Value
    [IDX_CHAR_HUMID_VAL] = {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_char_humidity_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t)*2, 0, NULL}},

    // Client Characteristic Configuration Descriptor (CCCD)
    [IDX_CHAR_HUMID_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}},
};

#define SENSOR_NOTIFY_INTERVAL_MS  10000 // 10 seconds

extern uint16_t handle_table[HRS_IDX_NB];
uint16_t notify_conn_id = 0xFFFF;
esp_gatt_if_t notify_gatts_if = 0;
// bool notify_enabled = false;

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


static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(GATTS_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            if (param->read.need_rsp == false) {
                ESP_LOGI(GATTS_TAG, "No response needed for read event");
            } else {
                // ESP_LOGI(GATTS_TAG, "Response needed for read event");
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                if (param->read.handle == handle_table[IDX_CHAR_TEMP_VAL]) {
                    float t = get_temperature();
                    int16_t temp = (uint16_t)(t * 100);
                    rsp.attr_value.len = sizeof(temp);
                    memcpy(rsp.attr_value.value, &temp, sizeof(temp));
                    ESP_LOGI(GATTS_TAG, "GATT READ [CHAR_TEMP_VAL] Temperature: (%.2f)", t);
                } else if (param->read.handle == handle_table[IDX_CHAR_HUMID_VAL]) {
                    float h = get_humidity();
                    uint16_t humid = (uint16_t)(h * 100);
                    rsp.attr_value.len = sizeof(humid);
                    memcpy(rsp.attr_value.value, &humid, sizeof(humid));
                    ESP_LOGI(GATTS_TAG, "GATT READ [CHAR_HUMID_VAL] Humidity: (%.2f)", h);
                } else {
                    rsp.attr_value.len = 0;
                }

                esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            }
            
       	    break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT");
            // cccd
            if (param->write.handle == handle_table[IDX_CHAR_TEMP_CFG]) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(GATTS_TAG, "Notification enabled");
                    // Notification enabled
                    // notify_enabled = true;
                    notify_conn_id = param->write.conn_id;
                    notify_gatts_if = gatts_if;
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(GATTS_TAG, "Indication enabled");
                    // Indication enabled
                    // notify_enabled = true;
                    // ... (Indication)
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(GATTS_TAG, "Notification disabled");
                    // Disabled
                    // notify_enabled = false;
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
            // ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            esp_ble_gap_update_conn_params(&conn_params);

            // start_sensor_notify_timer();
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            // ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            // notify_enabled = false;
            // stop_sensor_notify_timer();
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            // ESP_LOGI(GATTS_TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT");
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
                esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            }
            break;
        }
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
            profile_table[PROFILE_APP_IDX].gatts_if = gatts_if;
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

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#else
#define RCV_HOST    SPI2_HOST
#endif

#define GPIO_HANDSHAKE      2
#define GPIO_MOSI           6
#define GPIO_MISO           5
#define GPIO_SCLK           4
#define GPIO_CS             7

#define SPI_TAG "TH_SPI"

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

void ble_init() {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

void spi_task(void *param) {
    size_t buf_size = 4; // 4 bytes for temperature and humidity
    uint8_t *rx_buf = (uint8_t *)spi_bus_dma_memory_alloc(RCV_HOST, buf_size, 0);
    assert(rx_buf);
    spi_slave_transaction_t trans = {0};

    trans.length = 4 * 8; // bits
    trans.rx_buffer = rx_buf;
    while (1) {
        memset(rx_buf, 0, buf_size);

        if (spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY) == ESP_OK) {
            ESP_LOGI(SPI_TAG, " ");
            ESP_LOGI(SPI_TAG, "SPI [%02x %02x %02x %02x]", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

            float t = rx_buf[0] + rx_buf[1] / 100.f;
            float h = rx_buf[2] + rx_buf[3] / 100.f;
            ESP_LOGI(SPI_TAG, "SPI Received: T=%.2f, H=%.2f", t, h);
            set_temperature(t);
            set_humidity(h);
            // send_sensor_notification(NULL);
        }
        blink_led();
    }

    free(rx_buf);
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize sensor mutex
    init_sensor_mutex();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    configure_led();

    spi_slave_init();

    ble_init();

    xTaskCreate(spi_task, "spi_task", 4096, NULL, 5, NULL);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms delay
    }
}
