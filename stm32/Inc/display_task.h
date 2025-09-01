#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "queue.h"

extern I2C_HandleTypeDef hi2c1;

extern QueueHandle_t g_xDisplayCommandQueue;

BaseType_t DisplayTask_CreateTask(void);

#endif // DISPLAY_TASK_H
