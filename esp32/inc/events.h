#ifndef EVENTS_H
#define EVENTS_H

#include <stdint.h>
#include "shared_types.h"

typedef enum {
    EVT_SPI_DATA_RECEIVED,

    EVT_BLE_CLIENT_CONNECTED,
    EVT_BLE_CLIENT_DISCONNECTED,
    EVT_BLE_NOTIFICATION_ENABLED,
    EVT_BLE_NOTIFICATION_DISABLED,
    EVT_BLE_READ_REQUEST,

    EVT_WIFI_CONNECTED,
    EVT_WIFI_DISCONNECTED,

    EVT_MQTT_CONNECTED,
    EVT_MQTT_DISCONNECTED,
    EVT_MQTT_DATA_SENT,

    EVT_WIFI_CONFIG_RECEIVED,
    EVT_WIFI_CONFIG_CONNECT_REQ,
    EVT_WIFI_CONFIG_DISCONNECT_REQ,

    EVT_MQTT_CONFIG_RECEIVED,
    EVT_MQTT_CONFIG_SET_REQ,

} EventType_t;

typedef struct {
    char ssid[32];
    char password[64];
    uint8_t connect_cmd;  // 1=connect, 0=disconnect
} WiFiConfigData_t;

typedef struct {
    EventType_t   eType;
    union {
        SensorData_t     xSensorData;
        WiFiConfigData_t xWiFiConfig;
        MqttConfigData_t xMqttConfig;
    };
} Event_t;


#endif // EVENTS_H
