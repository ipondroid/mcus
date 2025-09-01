#ifndef SPI_TASK_H
#define SPI_TASK_H

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "queue.h"

extern SPI_HandleTypeDef hspi2;

extern QueueHandle_t g_xSpiCommandQueue;

BaseType_t SpiTask_CreateTask(void);

#endif // SPI_TASK_H
