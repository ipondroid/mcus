/*
 * sensor_manager.h
 *
 * Multi-sensor management system
 */

#ifndef INC_SENSOR_MANAGER_H_
#define INC_SENSOR_MANAGER_H_

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "sensor_interface.h"
#include "sensor_task.h"

#define SENSOR_MANAGER_MAX_SENSORS    8

typedef enum {
    SENSOR_MANAGER_OK = 0,
    SENSOR_MANAGER_ERROR,
    SENSOR_MANAGER_FULL,
    SENSOR_MANAGER_NOT_FOUND,
    SENSOR_MANAGER_INVALID_PARAM
} SensorManagerStatus_t;

typedef struct {
    uint8_t ucIsActive;
    SensorConfig_t xConfig;
    SensorTaskContext_t* pTaskContext;
    uint32_t ulLastUpdateTime;
    uint32_t ulTotalReadings;
    uint32_t ulErrorCount;
} SensorRegistryEntry_t;

typedef struct {
    SensorRegistryEntry_t axRegistry[SENSOR_MANAGER_MAX_SENSORS];
    uint8_t ucActiveSensorCount;
    SemaphoreHandle_t xMutex;
    uint8_t ucInitialized;
} SensorManagerContext_t;

extern SensorManagerContext_t g_xSensorManager;

SensorManagerStatus_t SensorManager_Init(void);

SensorManagerStatus_t SensorManager_Deinit(void);

SensorManagerStatus_t SensorManager_RegisterSensor(const SensorConfig_t* pConfig);

SensorManagerStatus_t SensorManager_UnregisterSensor(uint8_t ucSensorId);

SensorManagerStatus_t SensorManager_GetSensorData(uint8_t ucSensorId, SensorData_t* pData);

SensorManagerStatus_t SensorManager_GetAllSensorData(SensorData_t* pDataArray, uint8_t ucArraySize, uint8_t* pActualCount);

SensorManagerStatus_t SensorManager_GetSensorStats(uint8_t ucSensorId, uint32_t* pTotalReadings, uint32_t* pErrorCount, uint32_t* pLastUpdateTime);

SensorManagerStatus_t SensorManager_GetActiveSensors(uint8_t* pSensorIds, uint8_t ucArraySize, uint8_t* pActualCount);

uint8_t SensorManager_IsSensorActive(uint8_t ucSensorId);

SensorManagerStatus_t SensorManager_ResetSensorErrorCount(uint8_t ucSensorId);

SensorManagerStatus_t SensorManager_GetGlobalStats(uint8_t* pActiveSensors, uint32_t* pTotalReadings, uint32_t* pTotalErrors);

#endif /* INC_SENSOR_MANAGER_H_ */
