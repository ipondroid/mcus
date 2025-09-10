#include "wifi_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "WiFiConfig";
static bool g_initialized = false;

esp_err_t wifi_config_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret == ESP_OK) {
        g_initialized = true;
        ESP_LOGI(TAG, "WiFi configuration module initialized");
    } else {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t wifi_config_save(const char* ssid, const char* password)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "WiFi config module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) >= MAX_SSID_LEN || strlen(password) >= MAX_PASSWORD_LEN) {
        ESP_LOGE(TAG, "SSID or password too long");
        return ESP_ERR_INVALID_SIZE;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save SSID
    ret = nvs_set_str(nvs_handle, NVS_WIFI_SSID_KEY, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving SSID: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save password
    ret = nvs_set_str(nvs_handle, NVS_WIFI_PASSWORD_KEY, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving password: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Save configured flag
    ret = nvs_set_u8(nvs_handle, NVS_WIFI_CONFIGURED_KEY, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving configured flag: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi configuration saved successfully");
        ESP_LOGI(TAG, "SSID: %s", ssid);
    }
    
cleanup:
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t wifi_config_load(char* ssid, char* password)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "WiFi config module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password buffer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint8_t configured = 0;
    ret = nvs_get_u8(nvs_handle, NVS_WIFI_CONFIGURED_KEY, &configured);
    if (ret != ESP_OK || configured != 1) {
        ESP_LOGI(TAG, "WiFi configuration not found or not configured");
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }
    
    // Load SSID
    size_t ssid_len = MAX_SSID_LEN;
    ret = nvs_get_str(nvs_handle, NVS_WIFI_SSID_KEY, ssid, &ssid_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error loading SSID: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Load password
    size_t password_len = MAX_PASSWORD_LEN;
    ret = nvs_get_str(nvs_handle, NVS_WIFI_PASSWORD_KEY, password, &password_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error loading password: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "WiFi configuration loaded successfully");
    ESP_LOGI(TAG, "SSID: %s", ssid);
    
cleanup:
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t wifi_config_clear(void)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "WiFi config module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Erase entire namespace
    ret = nvs_erase_all(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error erasing WiFi config: %s", esp_err_to_name(ret));
    } else {
        ret = nvs_commit(nvs_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "WiFi configuration cleared successfully");
        } else {
            ESP_LOGE(TAG, "Error committing clear operation: %s", esp_err_to_name(ret));
        }
    }
    
    nvs_close(nvs_handle);
    return ret;
}

bool wifi_config_exists(void)
{
    if (!g_initialized) {
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return false;
    }
    
    uint8_t configured = 0;
    ret = nvs_get_u8(nvs_handle, NVS_WIFI_CONFIGURED_KEY, &configured);
    nvs_close(nvs_handle);
    
    return (ret == ESP_OK && configured == 1);
}
