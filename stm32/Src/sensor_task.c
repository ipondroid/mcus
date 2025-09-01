#include "sensor_task.h"
#include "state_manager.h"
#include "events.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

#include "dht22.h"

static void prvGenericSensorTask(void* pvParameters);
static uint8_t prvReadSensorData(const SensorConfig_t* pConfig, SensorData_t* pData);
static void prvSendSensorEvent(const SensorData_t* pData, EventType_t eEventType);

static void prvGenericSensorTask(void* pvParameters) {
    SensorTaskContext_t* pContext = (SensorTaskContext_t*)pvParameters;

    if (pContext == NULL) {
        printf("ST: Invalid task context\n\r");
        vTaskDelete(NULL);
        return;
    }

    SensorConfig_t* pConfig = &pContext->xConfig;
    printf("ST: Starting ID %d, type %d\n\r", pConfig->ucSensorId, pConfig->eType);

    memset(&pContext->xLastReading, 0, sizeof(SensorData_t));
    pContext->xLastReading.ucSensorId = pConfig->ucSensorId;
    pContext->xLastReading.eType = pConfig->eType;
    pContext->xLastReading.eStatus = SENSOR_STATUS_IDLE;

    // Initial delay to allow other tasks to start
    vTaskDelay(pdMS_TO_TICKS(1000 + (pConfig->ucSensorId * 100))); // Stagger startup

    uint32_t ulAdaptiveInterval = pConfig->ulReadingIntervalMs;

    for (;;) {
        // Check task state and respond to dynamic controls
        TaskState_t eTaskState;
        if (TaskManager_GetTaskState(pConfig->ucSensorId, &eTaskState) == TASK_MANAGER_OK) {
            if (eTaskState == TASK_STATE_DORMANT || eTaskState == TASK_STATE_SUSPENDED) {
                // Task is dormant/suspended, wait longer
                vTaskDelay(pdMS_TO_TICKS(ulAdaptiveInterval * 2));
                continue;
            }
        }

        // Update metrics for task management
        SensorMetrics_t xMetrics = {
            .ulDataAge = xTaskGetTickCount() - pContext->ulLastReadTime,
            .ulErrorRate = pContext->ulErrorCount > 0 ?
                          (pContext->ulErrorCount * 100) / (pContext->ulErrorCount + 1) : 0,
            .ulImportanceScore = (pConfig->eType == SENSOR_TYPE_DHT22) ? 100 : 50,
            .ucDemandLevel = 128, // Default medium demand
            .ulSystemLoad = 0
        };

        TaskManager_UpdateMetrics(pConfig->ucSensorId, &xMetrics);

        // Calculate adaptive reading interval
        ulAdaptiveInterval = TaskManager_CalculateAdaptiveInterval(pConfig->ucSensorId, &xMetrics);

        // Wait for the adaptive reading interval
        vTaskDelay(pdMS_TO_TICKS(ulAdaptiveInterval));

        SensorData_t sensorData;
        memset(&sensorData, 0, sizeof(SensorData_t));
        sensorData.ucSensorId = pConfig->ucSensorId;
        sensorData.eType = pConfig->eType;
        sensorData.ulTimestamp = xTaskGetTickCount();
        sensorData.eStatus = SENSOR_STATUS_READING;

        uint8_t ucRetries = 0;
        uint8_t ucSuccess = 0;

        while (ucRetries <= pConfig->ucMaxRetries && !ucSuccess) {
            if (prvReadSensorData(pConfig, &sensorData)) {
                ucSuccess = 1;
                sensorData.eStatus = SENSOR_STATUS_READY;
                pContext->ulErrorCount = 0;

                printf("ST: Sensor %d - T:%.2f, H:%.2f, P:%.2f\n\r",
                       sensorData.ucSensorId,
                       sensorData.fTemperature,
                       sensorData.fHumidity,
                       sensorData.fPressure);
            } else {
                ucRetries++;
                sensorData.eStatus = SENSOR_STATUS_ERROR;
                pContext->ulErrorCount++;

                if (ucRetries <= pConfig->ucMaxRetries) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }

        if (!ucSuccess) {
            printf("ST: Sensor %d read failed after %d retries\n\r",
                   pConfig->ucSensorId, pConfig->ucMaxRetries);

            prvSendSensorEvent(&sensorData, EVT_SENSOR_READ_ERROR);

            if (pContext->ulErrorCount > 5) {
                TaskManager_TransitionState(pConfig->ucSensorId, TASK_STATE_ERROR);
            }
        } else {
            memcpy(&pContext->xLastReading, &sensorData, sizeof(SensorData_t));
            pContext->ulLastReadTime = sensorData.ulTimestamp;

            prvSendSensorEvent(&sensorData, EVT_SENSOR_DATA_READY);

            TaskManager_TransitionState(pConfig->ucSensorId, TASK_STATE_RUNNING);
        }
    }
}

static uint8_t prvReadSensorData(const SensorConfig_t* pConfig, SensorData_t* pData) {
    if (pConfig == NULL || pData == NULL) {
        return 0;
    }

    const SensorDriver_t* pDriver = SensorInterface_GetDriver(pConfig->eType);
    if (pDriver == NULL || pDriver->pfRead == NULL) {
        printf("ST: No driver available for sensor type %d\n\r", pConfig->eType);
        return 0;
    }

    return pDriver->pfRead(pConfig, pData);
}

static void prvSendSensorEvent(const SensorData_t* pData, EventType_t eEventType) {
    if (pData == NULL) {
        return;
    }

    SensorData_t* pEventData = (SensorData_t*)pvPortMalloc(sizeof(SensorData_t));
    if (pEventData == NULL) {
        printf("ST: Failed to allocate memory for event data\n\r");
        return;
    }

    memcpy(pEventData, pData, sizeof(SensorData_t));

    Event_t newEvent = {
        .eType = eEventType,
        .pPayload = pEventData
    };

    if (xQueueSend(g_xEventQueue, &newEvent, pdMS_TO_TICKS(10)) != pdPASS) {
        printf("ST: Failed to send sensor event\n\r");
        vPortFree(pEventData);
    }
}

BaseType_t SensorTask_CreateSensorTask(const SensorConfig_t* pConfig, SensorTaskContext_t** pTaskContext) {
    if (pConfig == NULL) {
        return pdFAIL;
    }

    SensorTaskContext_t* pContext = (SensorTaskContext_t*)pvPortMalloc(sizeof(SensorTaskContext_t));
    if (pContext == NULL) {
        printf("ST: Failed to allocate task context\n\r");
        return pdFAIL;
    }

    memset(pContext, 0, sizeof(SensorTaskContext_t));
    memcpy(&pContext->xConfig, pConfig, sizeof(SensorConfig_t));

    char taskName[16];
    snprintf(taskName, sizeof(taskName), "Sensor_%d", pConfig->ucSensorId);

    BaseType_t xResult = xTaskCreate(
        prvGenericSensorTask,
        taskName,
        pConfig->usStackSize,
        pContext,
        pConfig->uxTaskPriority,
        &pContext->xTaskHandle
    );

    if (xResult != pdPASS) {
        printf("ST: Failed to create sensor task ID %d\n\r", pConfig->ucSensorId);
        vPortFree(pContext);
        return pdFAIL;
    }

    if (pTaskContext != NULL) {
        *pTaskContext = pContext;
    }

    printf("ST: Created sensor task ID %d\n\r", pConfig->ucSensorId);
    return pdPASS;
}

BaseType_t SensorTask_DestroyTask(SensorTaskContext_t* pTaskContext) {
    if (pTaskContext == NULL) {
        return pdFAIL;
    }

    if (pTaskContext->xTaskHandle != NULL) {
        vTaskDelete(pTaskContext->xTaskHandle);
        pTaskContext->xTaskHandle = NULL;
    }

    vPortFree(pTaskContext);

    return pdPASS;
}

BaseType_t SensorTask_GetLastReading(const SensorTaskContext_t* pTaskContext, SensorData_t* pData) {
    if (pTaskContext == NULL || pData == NULL) {
        return pdFAIL;
    }

    if (pTaskContext->xLastReading.eStatus != SENSOR_STATUS_READY) {
        return pdFAIL;
    }

    memcpy(pData, &pTaskContext->xLastReading, sizeof(SensorData_t));
    return pdPASS;
}
