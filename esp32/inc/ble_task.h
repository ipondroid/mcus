#ifndef BLE_TASK_H
#define BLE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t g_xBleCommandQueue;

BaseType_t BleTask_CreateTask(void);

#endif // BLE_TASK_H
