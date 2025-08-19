#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "FreeRTOS.h"

// Sensor data struct definition
typedef struct {
    float fTemperature;
    float fHumidity;
} SensorData_t;

BaseType_t SensorTask_CreateTask(void);

#endif // SENSOR_TASK_H
