#include "can_task.h"
#include "state_manager.h"
#include "events.h"
#include "commands.h"
#include "sensor_task.h" // for SensorData_t
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define CAN_MSG_POOL_SIZE 4

static CANMsgBuffer_t g_CANMsgPool[CAN_MSG_POOL_SIZE];

// CAN task handle
static TaskHandle_t g_xCanTaskHandle = NULL;

// CAN transmit function
static void prvCanTransmit(SensorData_t* pData) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8];
    uint8_t intPart, fractPart;
    static uint8_t canMsgCount = 0;

    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = 0x030;
    txHeader.ExtId = 0x03;
    txHeader.TransmitGlobalTime = DISABLE;

    txHeader.DLC = 5;
    intPart = (uint8_t)pData->fHumidity;
    fractPart = (uint8_t)(pData->fHumidity*100 - intPart*100);
    txData[0] = intPart;
    txData[1] = fractPart;
    intPart = (uint8_t)pData->fTemperature;
    fractPart = (uint8_t)(pData->fTemperature*100 - intPart*100);
    txData[2] = intPart;
    txData[3] = fractPart;
    txData[4] = ++canMsgCount;

    // printf("CANT: T:%.2f, H:%.2f\n\r", pData->fTemperature, pData->fHumidity);
    printf("CANT [%02d %02d %02d %02d %02d]\n\r",
        txData[0], txData[1], txData[2], txData[3], txData[4]);

    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
        printf("CANT: Failed to transmit CAN message.\n\r");
    }

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

static void prvCanTask(void* pvParameters) {
    Command_t xReceivedCommand;
    printf("CANT\n\r");

    for (;;) {
        if (xQueueReceive(g_xCanCommandQueue, &xReceivedCommand, portMAX_DELAY) == pdPASS) {
            if (xReceivedCommand.eType == CMD_CAN_SEND_DATA) {
                prvCanTransmit((SensorData_t*)xReceivedCommand.pPayload);

                if (xReceivedCommand.pPayload != NULL) {
                    vPortFree(xReceivedCommand.pPayload);
                }
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        CANMsgBuffer_t* pBuffer = NULL;
        for (int i=0; i < CAN_MSG_POOL_SIZE; i++) {
            if (!g_CANMsgPool[i].inUse) {
                g_CANMsgPool[i].inUse = 1;
                pBuffer = &g_CANMsgPool[i];
                break;
            }
        }

        if (pBuffer != NULL) {
            memcpy(pBuffer->data, rxData, sizeof(rxData));
            pBuffer->header = rxHeader;
            
            Event_t newEvent = {
                .eType = EVT_CAN_MSG_RECEIVED,
                .pPayload = pBuffer,
                .ulDataSize = sizeof(CANMsgBuffer_t)
            };
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(g_xEventQueue, &newEvent, &xHigherPriorityTaskWoken) != pdPASS) {
                pBuffer->inUse = 0;
                // error count
            }
            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        // buffer full error count
    }
}

BaseType_t CanTask_CreateTask(void) {
    return xTaskCreate(
        prvCanTask,                     // Task function
        "CAN_Task",                     // Task name
        configMINIMAL_STACK_SIZE * 2,   // Stack size
        NULL,                           // Parameters
        tskIDLE_PRIORITY + 3,           // Priority
        &g_xCanTaskHandle               // Task handle
    );
}

