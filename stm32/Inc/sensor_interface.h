/*
 * sensor_interface.h
 *
 * Universal sensor interface for multi-sensor support
 */

#ifndef INC_SENSOR_INTERFACE_H_
#define INC_SENSOR_INTERFACE_H_

#include "FreeRTOS.h"
#include "main.h"

typedef enum {
    SENSOR_TYPE_DHT22 = 0,
    SENSOR_TYPE_DS18B20,
    SENSOR_TYPE_BMP280,
    SENSOR_TYPE_MAX
} SensorType_t;

typedef enum {
    SENSOR_STATUS_IDLE = 0,
    SENSOR_STATUS_READING,
    SENSOR_STATUS_READY,
    SENSOR_STATUS_ERROR,
    SENSOR_STATUS_DISCONNECTED
} SensorStatus_t;

typedef struct {
    uint8_t ucSensorId;
    SensorType_t eType;
    GPIO_TypeDef* pGPIOPort;
    uint16_t uGPIOPin;
    uint32_t ulReadingIntervalMs;
    uint8_t ucMaxRetries;
    UBaseType_t uxTaskPriority;
    uint16_t usStackSize;
} SensorConfig_t;

typedef struct {
    uint8_t ucSensorId;
    SensorType_t eType;
    float fTemperature;
    float fHumidity;
    float fPressure;
    uint32_t ulTimestamp;
    SensorStatus_t eStatus;
} SensorData_t;

typedef struct {
    uint8_t (*pfInit)(const SensorConfig_t* pConfig);
    uint8_t (*pfRead)(const SensorConfig_t* pConfig, SensorData_t* pData);
    uint8_t (*pfDeinit)(const SensorConfig_t* pConfig);
    uint8_t (*pfReset)(const SensorConfig_t* pConfig);
} SensorDriver_t;

const SensorDriver_t* SensorInterface_GetDriver(SensorType_t eType);
uint8_t SensorInterface_IsDataValid(const SensorData_t* pData);
const char* SensorInterface_GetTypeName(SensorType_t eType);
const char* SensorInterface_GetStatusName(SensorStatus_t eStatus);

#endif /* INC_SENSOR_INTERFACE_H_ */
