#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include "shared_types.h"

typedef enum {
    CMD_BLE_NOTIFY_DATA,
    CMD_SPI_SEND_DATA,
    
    // WiFi Configuration Commands
    CMD_WIFI_CONNECT,
    CMD_WIFI_DISCONNECT,
    CMD_WIFI_STATUS_UPDATE,
} CommandType_t;

typedef struct {
    char ssid[32];
    char password[64];
} WiFiCredentials_t;

typedef struct {
    CommandType_t eType;
    union {
        SensorData_t      xSensorData;  // Sensor data for BLE commands
        WiFiCredentials_t xWiFiCreds;   // WiFi credentials for WiFi commands
        uint8_t           status;
    };
} Command_t;


#endif // COMMANDS_H
