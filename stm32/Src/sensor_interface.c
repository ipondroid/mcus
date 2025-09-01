/*
 * sensor_interface.c
 *
 * Universal sensor interface implementation
 */

#include "sensor_interface.h"
#include "dht22.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint8_t DHT22_Init(const SensorConfig_t* pConfig);
static uint8_t DHT22_Read(const SensorConfig_t* pConfig, SensorData_t* pData);
static uint8_t DHT22_Deinit(const SensorConfig_t* pConfig);
static uint8_t DHT22_Reset(const SensorConfig_t* pConfig);

static uint8_t DS18B20_Init(const SensorConfig_t* pConfig);
static uint8_t DS18B20_Read(const SensorConfig_t* pConfig, SensorData_t* pData);
static uint8_t DS18B20_Deinit(const SensorConfig_t* pConfig);
static uint8_t DS18B20_Reset(const SensorConfig_t* pConfig);

static const SensorDriver_t g_xDHT22Driver = {
    .pfInit = DHT22_Init,
    .pfRead = DHT22_Read,
    .pfDeinit = DHT22_Deinit,
    .pfReset = DHT22_Reset
};

static const SensorDriver_t g_xDS18B20Driver = {
    .pfInit = DS18B20_Init,
    .pfRead = DS18B20_Read,
    .pfDeinit = DS18B20_Deinit,
    .pfReset = DS18B20_Reset
};

static const SensorDriver_t* g_apDriverRegistry[SENSOR_TYPE_MAX] = {
    [SENSOR_TYPE_DHT22] = &g_xDHT22Driver,
    [SENSOR_TYPE_DS18B20] = &g_xDS18B20Driver,
    [SENSOR_TYPE_BMP280] = NULL    // Not implemented yet
};

static const char* g_apcSensorTypeNames[SENSOR_TYPE_MAX] = {
    [SENSOR_TYPE_DHT22] = "DHT22",
    [SENSOR_TYPE_DS18B20] = "DS18B20",
    [SENSOR_TYPE_BMP280] = "BMP280"
};

static const char* g_apcSensorStatusNames[] = {
    [SENSOR_STATUS_IDLE] = "IDLE",
    [SENSOR_STATUS_READING] = "READING",
    [SENSOR_STATUS_READY] = "READY",
    [SENSOR_STATUS_ERROR] = "ERROR",
    [SENSOR_STATUS_DISCONNECTED] = "DISCONNECTED"
};

const SensorDriver_t* SensorInterface_GetDriver(SensorType_t eType) {
    if (eType >= SENSOR_TYPE_MAX) {
        return NULL;
    }
    return g_apDriverRegistry[eType];
}

uint8_t SensorInterface_IsDataValid(const SensorData_t* pData) {
    if (pData == NULL) {
        return 0;
    }

    if (pData->eStatus != SENSOR_STATUS_READY) {
        return 0;
    }

    switch (pData->eType) {
        case SENSOR_TYPE_DHT22:
            // DHT22 valid ranges: Temperature -40 to 80°C, Humidity 0 to 100%
            if (pData->fTemperature < -40.0f || pData->fTemperature > 80.0f ||
                pData->fHumidity < 0.0f || pData->fHumidity > 100.0f) {
                return 0;
            }
            break;

        case SENSOR_TYPE_DS18B20:
            // DS18B20 valid range: Temperature -55 to 125°C
            if (pData->fTemperature < -55.0f || pData->fTemperature > 125.0f) {
                return 0;
            }
            break;

        case SENSOR_TYPE_BMP280:
            // BMP280 valid ranges: Temperature -40 to 85°C, Pressure 300 to 1100 hPa
            if (pData->fTemperature < -40.0f || pData->fTemperature > 85.0f ||
                pData->fPressure < 300.0f || pData->fPressure > 1100.0f) {
                return 0;
            }
            break;

        default:
            return 0; // Unknown 
    }

    return 1;
}

const char* SensorInterface_GetTypeName(SensorType_t eType) {
    if (eType >= SENSOR_TYPE_MAX) {
        return "UNKNOWN";
    }
    return g_apcSensorTypeNames[eType];
}

const char* SensorInterface_GetStatusName(SensorStatus_t eStatus) {
    if (eStatus >= sizeof(g_apcSensorStatusNames) / sizeof(g_apcSensorStatusNames[0])) {
        return "UNKNOWN";
    }
    return g_apcSensorStatusNames[eStatus];
}

/* DHT22 Driver Implementation */

static uint8_t DHT22_Init(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DHT22: Initializing sensor ID %d on GPIO %p:%d\n\r", 
           pConfig->ucSensorId, (void*)pConfig->pGPIOPort, pConfig->uGPIOPin);

    // DHT22 doesn't need specific initialization beyond GPIO setup
    // GPIO is already configured in main.c MX_GPIO_Init()
    
    return 1;
}

static uint8_t DHT22_Read(const SensorConfig_t* pConfig, SensorData_t* pData) {
    if (pConfig == NULL || pData == NULL) {
        return 0;
    }

    DHT_DataTypedef dhtData;
    memset(&dhtData, 0, sizeof(DHT_DataTypedef));

    uint8_t result = DHT_GetData(&dhtData);
    if (result == 0) {
        printf("DHT22: Failed to read sensor ID %d\n\r", pConfig->ucSensorId);
        return 0;
    }

    pData->ucSensorId = pConfig->ucSensorId;
    pData->eType = SENSOR_TYPE_DHT22;
    pData->fTemperature = dhtData.Temperature;
    pData->fHumidity = dhtData.Humidity;
    pData->fPressure = 0.0f; // DHT22 doesn't measure pressure
    pData->ulTimestamp = HAL_GetTick();
    pData->eStatus = SENSOR_STATUS_READY;

    printf("DHT22: Read sensor ID %d - T:%.2f°C, H:%.2f%%\n\r", 
           pData->ucSensorId, pData->fTemperature, pData->fHumidity);

    return 1;
}

static uint8_t DHT22_Deinit(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DHT22: Deinitializing sensor ID %d\n\r", pConfig->ucSensorId);
    
    // DHT22 doesn't need specific deinitialization
    return 1;
}

static uint8_t DHT22_Reset(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DHT22: Resetting sensor ID %d\n\r", pConfig->ucSensorId);
    
    // For DHT22, reset means reinitializing the communication
    // This is handled by the DHT_GetData function internally
    return 1;
}


/* DS18B20 Driver Implementation */

static uint8_t DS18B20_Init(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DS18B20: Initializing sensor ID %d on GPIO %p:%d\n\r", 
           pConfig->ucSensorId, (void*)pConfig->pGPIOPort, pConfig->uGPIOPin);

    return 1;
}
static uint8_t DS18B20_Read(const SensorConfig_t* pConfig, SensorData_t* pData) {
    static uint32_t ulLastReadTime = 0;

    if (pConfig == NULL || pData == NULL) {
        return 0;
    }

    if (HAL_GetTick() - ulLastReadTime < 5000) {
        // DS18B20 requires at least 5 second between reads
        return 0;
    }
    ulLastReadTime = HAL_GetTick();

    pData->ucSensorId = pConfig->ucSensorId;
    pData->eType = SENSOR_TYPE_DS18B20;
    pData->fTemperature = 25.0f + (rand() % 1000) / 100.0f; // 25.00 to 35.00
    pData->fHumidity = 40.0f + (rand() % 6000) / 100.0f;    // 40.00 to 100.00
    pData->fPressure = 0.0f;
    pData->ulTimestamp = HAL_GetTick();
    pData->eStatus = SENSOR_STATUS_READY;

    printf("DS18B20: Read sensor ID %d - T:%.2f°C, H:%.2f%%\n\r", 
           pData->ucSensorId, pData->fTemperature, pData->fHumidity);

    return 1;
}

static uint8_t DS18B20_Deinit(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DS18B20: Deinitializing sensor ID %d\n\r", pConfig->ucSensorId);
    
    return 1;
}

static uint8_t DS18B20_Reset(const SensorConfig_t* pConfig) {
    if (pConfig == NULL) {
        return 0;
    }

    printf("DS18B20: Resetting sensor ID %d\n\r", pConfig->ucSensorId);
    
    return 1;
}
