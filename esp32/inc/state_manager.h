#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t g_xEventQueue;

BaseType_t StateManager_CreateTask(void);

#endif // STATE_MANAGER_H
