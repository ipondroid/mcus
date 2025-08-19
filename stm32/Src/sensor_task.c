#include "sensor_task.h"
#include "state_manager.h"
#include "events.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "dht22.h"

static TaskHandle_t g_xSensorTaskHandle = NULL;

static void prvSensorTask(void* pvParameters) {
    printf("SST\n\r");

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a moment for other tasks to get ready

    for (;;) {
        // Attempt to read sensor data every 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));

        SensorData_t* pSensorData = (SensorData_t*)pvPortMalloc(sizeof(SensorData_t));
        if (pSensorData == NULL) {
            printf("SST: Failed to allocate memory for sensor data\n\r");
            continue; // to the next cycle
        }

        if (DHT_GetData((DHT_DataTypedef *)pSensorData) == 0) { // Use virtual data
            printf("SST: Failed to read from DHT22 sensor.\n\r");
            vPortFree(pSensorData);
        }
        else {
            printf("SST: T:%.2f, H:%.2f\n\r", pSensorData->fTemperature, pSensorData->fHumidity);

            Event_t newEvent = {
                .eType = EVT_SENSOR_DATA_READY,
                .pPayload = pSensorData
            };

            if (xQueueSend(g_xEventQueue, &newEvent, pdMS_TO_TICKS(10)) != pdPASS) {
                printf("SST: Failed to send event to queue.\n\r");
                vPortFree(pSensorData);
            }
        }
    }
}

BaseType_t SensorTask_CreateTask(void) {
    return xTaskCreate(
        prvSensorTask,                  // Task function
        "Sensor_Task",                  // Task name
        configMINIMAL_STACK_SIZE * 2,   // Stack size
        NULL,                           // Parameters
        tskIDLE_PRIORITY + 1,           // Priority
        &g_xSensorTaskHandle            // Task handle
    );
}

