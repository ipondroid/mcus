#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define WIFI_SSID       "WIFI_SSID"
#define WIFI_PASSWORD   "WIFI_PASSWORD"
#define WIFI_MAXIMUM_RETRY  5

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_init_sta(void);
void WiFiTask_CreateTask(void);
bool wifi_is_connected(void);

esp_err_t wifi_connect_with_credentials(const char* ssid, const char* password);
esp_err_t wifi_disconnect(void);
uint8_t wifi_get_connection_status(void);

#endif // WIFI_TASK_H
