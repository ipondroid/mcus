#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "FreeRTOS.h"
#include "queue.h"

// Event queue handle (created in main.c and assigned here)
extern QueueHandle_t g_xEventQueue;

BaseType_t StateManager_CreateTask(void);

#endif // STATE_MANAGER_H
