#ifndef CAN_TASK_H
#define CAN_TASK_H

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "queue.h"

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t inUse;
} CANMsgBuffer_t;

extern CAN_HandleTypeDef hcan1;

extern QueueHandle_t g_xCanCommandQueue;

BaseType_t CanTask_CreateTask(void);

#endif // CAN_TASK_H
