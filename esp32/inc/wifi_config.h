#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "esp_err.h"
#include <stdbool.h>

// NVS key definitions
#define NVS_WIFI_NAMESPACE      "wifi_config"
#define NVS_WIFI_SSID_KEY       "ssid"
#define NVS_WIFI_PASSWORD_KEY   "password"
#define NVS_WIFI_CONFIGURED_KEY "configured"

// WiFi credentials maximum length
#define MAX_SSID_LEN     32
#define MAX_PASSWORD_LEN 64

esp_err_t wifi_config_save(const char* ssid, const char* password);
esp_err_t wifi_config_load(char* ssid, char* password);
esp_err_t wifi_config_clear(void);
bool wifi_config_exists(void);
esp_err_t wifi_config_init(void);

#endif // WIFI_CONFIG_H
