#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "sensor_interface.h"

/* Task parameters */
#define SENSOR_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
#define SENSOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

typedef struct {
    TaskHandle_t xTaskHandle;
    SensorConfig_t xConfig;
    SensorData_t xLastReading;
    uint32_t ulErrorCount;
    uint32_t ulLastReadTime;
} SensorTaskContext_t;

BaseType_t SensorTask_CreateSensorTask(const SensorConfig_t* pConfig, SensorTaskContext_t** pTaskContext);

BaseType_t SensorTask_CreateTask(void);

BaseType_t SensorTask_DestroyTask(SensorTaskContext_t* pTaskContext);

BaseType_t SensorTask_GetLastReading(const SensorTaskContext_t* pTaskContext, SensorData_t* pData);

#endif // SENSOR_TASK_H
