/*
 * sensor_manager.c
 *
 * Multi-sensor management system implementation
 */

#include "sensor_manager.h"
#include "events.h"
#include <string.h>
#include <stdio.h>

SensorManagerContext_t g_xSensorManager = {0};

static SensorRegistryEntry_t* prvFindSensorEntry(uint8_t ucSensorId);
static SensorManagerStatus_t prvValidateConfig(const SensorConfig_t* pConfig);
static void prvUpdateSensorStats(SensorRegistryEntry_t* pEntry, uint8_t ucSuccess);

SensorManagerStatus_t SensorManager_Init(void) {
    if (g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_OK; // Already initialized
    }

    memset(g_xSensorManager.axRegistry, 0, sizeof(g_xSensorManager.axRegistry));
    g_xSensorManager.ucActiveSensorCount = 0;
    printf("\n\r");

    g_xSensorManager.xMutex = xSemaphoreCreateMutex();
    if (g_xSensorManager.xMutex == NULL) {
        printf("SM: Failed to create mutex\n\r");
        return SENSOR_MANAGER_ERROR;
    }

    g_xSensorManager.ucInitialized = 1;
    printf("SM: initialized\n\r");

    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_Deinit(void) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_OK;
    }

    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS; i++) {
        if (g_xSensorManager.axRegistry[i].ucIsActive) {
            SensorManager_UnregisterSensor(g_xSensorManager.axRegistry[i].xConfig.ucSensorId);
        }
    }

    if (g_xSensorManager.xMutex != NULL) {
        vSemaphoreDelete(g_xSensorManager.xMutex);
        g_xSensorManager.xMutex = NULL;
    }

    g_xSensorManager.ucInitialized = 0;
    printf("SM: deinitialized\n\r");

    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_RegisterSensor(const SensorConfig_t* pConfig) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_ERROR;
    }

    if (pConfig == NULL || prvValidateConfig(pConfig) != SENSOR_MANAGER_OK) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    if (prvFindSensorEntry(pConfig->ucSensorId) != NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_ERROR;
    }

    SensorRegistryEntry_t* pEntry = NULL;
    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS; i++) {
        if (!g_xSensorManager.axRegistry[i].ucIsActive) {
            pEntry = &g_xSensorManager.axRegistry[i];
            break;
        }
    }

    if (pEntry == NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_FULL;
    }

    memcpy(&pEntry->xConfig, pConfig, sizeof(SensorConfig_t));
    pEntry->ucIsActive = 1;
    pEntry->pTaskContext = NULL;
    pEntry->ulLastUpdateTime = 0;
    pEntry->ulTotalReadings = 0;
    pEntry->ulErrorCount = 0;

    BaseType_t xResult = SensorTask_CreateSensorTask(pConfig, &pEntry->pTaskContext);
    if (xResult != pdPASS) {
        pEntry->ucIsActive = 0;
        xSemaphoreGive(g_xSensorManager.xMutex);
        printf("SM: Failed to create task for sensor %d\n\r", pConfig->ucSensorId);
        return SENSOR_MANAGER_ERROR;
    }

    g_xSensorManager.ucActiveSensorCount++;
    xSemaphoreGive(g_xSensorManager.xMutex);

    printf("SM: Registered sensor ID %d, type %d\n\r", pConfig->ucSensorId, pConfig->eType);
    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_UnregisterSensor(uint8_t ucSensorId) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    SensorRegistryEntry_t* pEntry = prvFindSensorEntry(ucSensorId);
    if (pEntry == NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_NOT_FOUND;
    }

    if (pEntry->pTaskContext != NULL) {
        SensorTask_DestroyTask(pEntry->pTaskContext);
        pEntry->pTaskContext = NULL;
    }

    pEntry->ucIsActive = 0;
    memset(&pEntry->xConfig, 0, sizeof(SensorConfig_t));
    pEntry->ulLastUpdateTime = 0;
    pEntry->ulTotalReadings = 0;
    pEntry->ulErrorCount = 0;

    g_xSensorManager.ucActiveSensorCount--;
    xSemaphoreGive(g_xSensorManager.xMutex);

    printf("SM: Unregistered sensor ID %d\n\r", ucSensorId);
    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_GetSensorData(uint8_t ucSensorId, SensorData_t* pData) {
    if (!g_xSensorManager.ucInitialized || pData == NULL) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    SensorRegistryEntry_t* pEntry = prvFindSensorEntry(ucSensorId);
    if (pEntry == NULL || pEntry->pTaskContext == NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_NOT_FOUND;
    }

    // Get last reading from sensor task
    BaseType_t xResult = SensorTask_GetLastReading(pEntry->pTaskContext, pData);
    if (xResult == pdPASS) {
        pEntry->ulLastUpdateTime = xTaskGetTickCount();
        pEntry->ulTotalReadings++;
        prvUpdateSensorStats(pEntry, 1);
    } else {
        prvUpdateSensorStats(pEntry, 0);
    }

    xSemaphoreGive(g_xSensorManager.xMutex);

    return (xResult == pdPASS) ? SENSOR_MANAGER_OK : SENSOR_MANAGER_ERROR;
}

SensorManagerStatus_t SensorManager_GetAllSensorData(SensorData_t* pDataArray, uint8_t ucArraySize, uint8_t* pActualCount) {
    if (!g_xSensorManager.ucInitialized || pDataArray == NULL || pActualCount == NULL) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    *pActualCount = 0;

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS && *pActualCount < ucArraySize; i++) {
        SensorRegistryEntry_t* pEntry = &g_xSensorManager.axRegistry[i];
        if (pEntry->ucIsActive && pEntry->pTaskContext != NULL) {
            if (SensorTask_GetLastReading(pEntry->pTaskContext, &pDataArray[*pActualCount]) == pdPASS) {
                (*pActualCount)++;
            }
        }
    }

    xSemaphoreGive(g_xSensorManager.xMutex);
    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_GetSensorStats(uint8_t ucSensorId, uint32_t* pTotalReadings, uint32_t* pErrorCount, uint32_t* pLastUpdateTime) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    SensorRegistryEntry_t* pEntry = prvFindSensorEntry(ucSensorId);
    if (pEntry == NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_NOT_FOUND;
    }

    if (pTotalReadings) *pTotalReadings = pEntry->ulTotalReadings;
    if (pErrorCount) *pErrorCount = pEntry->ulErrorCount;
    if (pLastUpdateTime) *pLastUpdateTime = pEntry->ulLastUpdateTime;

    xSemaphoreGive(g_xSensorManager.xMutex);
    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_GetActiveSensors(uint8_t* pSensorIds, uint8_t ucArraySize, uint8_t* pActualCount) {
    if (!g_xSensorManager.ucInitialized || pSensorIds == NULL || pActualCount == NULL) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    *pActualCount = 0;

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS && *pActualCount < ucArraySize; i++) {
        if (g_xSensorManager.axRegistry[i].ucIsActive) {
            pSensorIds[*pActualCount] = g_xSensorManager.axRegistry[i].xConfig.ucSensorId;
            (*pActualCount)++;
        }
    }

    xSemaphoreGive(g_xSensorManager.xMutex);
    return SENSOR_MANAGER_OK;
}

uint8_t SensorManager_IsSensorActive(uint8_t ucSensorId) {
    if (!g_xSensorManager.ucInitialized) {
        return 0;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    SensorRegistryEntry_t* pEntry = prvFindSensorEntry(ucSensorId);
    uint8_t ucResult = (pEntry != NULL) ? 1 : 0;

    xSemaphoreGive(g_xSensorManager.xMutex);
    return ucResult;
}

SensorManagerStatus_t SensorManager_ResetSensorErrorCount(uint8_t ucSensorId) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    SensorRegistryEntry_t* pEntry = prvFindSensorEntry(ucSensorId);
    if (pEntry == NULL) {
        xSemaphoreGive(g_xSensorManager.xMutex);
        return SENSOR_MANAGER_NOT_FOUND;
    }

    pEntry->ulErrorCount = 0;
    xSemaphoreGive(g_xSensorManager.xMutex);

    return SENSOR_MANAGER_OK;
}

SensorManagerStatus_t SensorManager_GetGlobalStats(uint8_t* pActiveSensors, uint32_t* pTotalReadings, uint32_t* pTotalErrors) {
    if (!g_xSensorManager.ucInitialized) {
        return SENSOR_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xSensorManager.xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SENSOR_MANAGER_ERROR;
    }

    if (pActiveSensors) *pActiveSensors = g_xSensorManager.ucActiveSensorCount;

    uint32_t ulTotalReadings = 0, ulTotalErrors = 0;
    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS; i++) {
        if (g_xSensorManager.axRegistry[i].ucIsActive) {
            ulTotalReadings += g_xSensorManager.axRegistry[i].ulTotalReadings;
            ulTotalErrors += g_xSensorManager.axRegistry[i].ulErrorCount;
        }
    }

    if (pTotalReadings) *pTotalReadings = ulTotalReadings;
    if (pTotalErrors) *pTotalErrors = ulTotalErrors;

    xSemaphoreGive(g_xSensorManager.xMutex);
    return SENSOR_MANAGER_OK;
}

static SensorRegistryEntry_t* prvFindSensorEntry(uint8_t ucSensorId) {
    for (uint8_t i = 0; i < SENSOR_MANAGER_MAX_SENSORS; i++) {
        if (g_xSensorManager.axRegistry[i].ucIsActive &&
            g_xSensorManager.axRegistry[i].xConfig.ucSensorId == ucSensorId) {
            return &g_xSensorManager.axRegistry[i];
        }
    }
    return NULL;
}

static SensorManagerStatus_t prvValidateConfig(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    if (pConfig->eType >= SENSOR_TYPE_MAX) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    if (pConfig->ulReadingIntervalMs < 100 || pConfig->ulReadingIntervalMs > 3600000) {
        return SENSOR_MANAGER_INVALID_PARAM; // 100ms to 1 hour
    }

    if (pConfig->ucMaxRetries > 10) {
        return SENSOR_MANAGER_INVALID_PARAM;
    }

    return SENSOR_MANAGER_OK;
}

static void prvUpdateSensorStats(SensorRegistryEntry_t* pEntry, uint8_t ucSuccess) {
    if (pEntry == NULL) return;

    if (!ucSuccess) {
        pEntry->ulErrorCount++;
    }
}
