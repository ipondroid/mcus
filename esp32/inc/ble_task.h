#ifndef BLE_TASK_H
#define BLE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "shared_types.h"

// BLE transmission mode enumeration
typedef enum {
    BLE_TRANSMISSION_MODE_ON_REQUEST,  // CCCD = 0x0000 (Notification disabled)
    BLE_TRANSMISSION_MODE_REALTIME     // CCCD = 0x0001 (Notification enabled)
} BleTransmissionMode_t;

// BLE command queue handle
extern QueueHandle_t g_xBleCommandQueue;

// WiFi configuration through BLE
extern QueueHandle_t g_xEventQueue;

BaseType_t BleTask_CreateTask(void);

void get_mqtt_config(MqttConfigData_t* config);

void set_mqtt_config(const MqttConfigData_t* config);

BleTransmissionMode_t get_ble_transmission_mode(void);

void set_ble_transmission_mode(BleTransmissionMode_t mode);

#endif // BLE_TASK_H
