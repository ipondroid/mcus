#include "state_manager.h"
#include "system_state.h"
#include "events.h"
#include "commands.h"
#include "sensor_task.h"
#include "spi_task.h"
#include "can_task.h"
#include "display_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

SystemState_t g_SystemState;

static TaskHandle_t g_xStateManagerTaskHandle = NULL;

static void prvHandleEvent(Event_t* pxEvent);
static void prvHandleSensorDataReady(Event_t* pxEvent);
static void prvHandleCANReceivedData(Event_t* pxEvent);

static void prvStateManagerTask(void* pvParameters) {
    Event_t xReceivedEvent;

    printf("SMT\n\r");

    g_SystemState.eSystemMode = MODE_INITIALIZING;

    // ... Other initializations

    g_SystemState.eSystemMode = MODE_NORMAL_OPERATION;

    for (;;) {
        if (xQueueReceive(g_xEventQueue, &xReceivedEvent, portMAX_DELAY) == pdPASS) {
            prvHandleEvent(&xReceivedEvent);
        }
    }
}

static void prvHandleEvent(Event_t* pxEvent) {
    printf("SMT EV%d\n\r", pxEvent->eType);

    switch (g_SystemState.eSystemMode) {
        case MODE_NORMAL_OPERATION:
            switch (pxEvent->eType) {
                case EVT_SENSOR_DATA_READY:
                    prvHandleSensorDataReady(pxEvent);
                    break;
                case EVT_CAN_MSG_RECEIVED:
                    prvHandleCANReceivedData(pxEvent);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    if (pxEvent->pPayload != NULL) {
        vPortFree(pxEvent->pPayload);
    }
}

static void prvHandleSensorDataReady(Event_t* pxEvent) {
    SensorData_t* pSensorData = (SensorData_t*)pxEvent->pPayload;

    if (pSensorData->ucSensorId != 0) {
        // We only handle sensor ID 0 (DHT22) for now
        pxEvent->pPayload = NULL;
        return;
    }

    g_SystemState.fLastTemperature = pSensorData->fTemperature;
    g_SystemState.fLastHumidity = pSensorData->fHumidity;

    SensorData_t* pDataForDisplay = (SensorData_t*)pvPortMalloc(sizeof(SensorData_t));
    SensorData_t* pDataForCan = (SensorData_t*)pvPortMalloc(sizeof(SensorData_t));
    SensorData_t* pDataForSpi = (SensorData_t*)pvPortMalloc(sizeof(SensorData_t));

    if (pDataForDisplay == NULL || pDataForCan == NULL || pDataForSpi == NULL) {
        printf("SMT: Failed to allocate memory for command payloads.\n\r");
        if(pDataForDisplay) vPortFree(pDataForDisplay);
        if(pDataForCan) vPortFree(pDataForCan);
        if(pDataForSpi) vPortFree(pDataForSpi);
        return;
    }

    memcpy(pDataForDisplay, pSensorData, sizeof(SensorData_t));
    memcpy(pDataForCan, pSensorData, sizeof(SensorData_t));
    memcpy(pDataForSpi, pSensorData, sizeof(SensorData_t));

    Command_t cmd;

    // Send command to Display Task
    cmd = (Command_t){ .eType = CMD_DISPLAY_UPDATE, .pPayload = pDataForDisplay };
    if (xQueueSend(g_xDisplayCommandQueue, &cmd, 0) != pdPASS) {
        vPortFree(pDataForDisplay);
    }

    // Send command to CAN Task
    cmd = (Command_t){ .eType = CMD_CAN_SEND_DATA, .pPayload = pDataForCan };
    if (xQueueSend(g_xCanCommandQueue, &cmd, 0) != pdPASS) {
        vPortFree(pDataForCan);
    }

    // Send command to SPI Task
    cmd = (Command_t){ .eType = CMD_SPI_SEND_DATA, .pPayload = pDataForSpi };
    if (xQueueSend(g_xSpiCommandQueue, &cmd, 0) != pdPASS) {
        vPortFree(pDataForSpi);
    }

    pxEvent->pPayload = NULL;
}

static void prvHandleCANReceivedData(Event_t* pxEvent) {
    printf("SMT CR [");
    CANMsgBuffer_t* pCANMsgData = (CANMsgBuffer_t*)pxEvent->pPayload;
    uint8_t length = pCANMsgData->header.DLC-1;

    for (int i=0; i<length; i++) {
        printf("%02x ", pCANMsgData->data[i]);
    }
    printf("%02x]\n\r", pCANMsgData->data[length]);
    pCANMsgData->inUse = 0;

    pxEvent->pPayload = NULL;
}

BaseType_t StateManager_CreateTask(void) {
    return xTaskCreate(
        prvStateManagerTask,
        "StateManager",
        configMINIMAL_STACK_SIZE * 4,
        NULL,
        tskIDLE_PRIORITY + 2,
        &g_xStateManagerTaskHandle
    );
}
