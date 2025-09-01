#ifndef EVENTS_H
#define EVENTS_H

#include <stdint.h>

// Defines all possible events in the system.
typedef enum {
    // CAN related events
    EVT_CAN_MSG_RECEIVED,
    EVT_CAN_BUS_OFF,

    // SPI related events
    EVT_SPI_MSG_RECEIVED,
    EVT_SPI_LINK_DOWN,

    // Sensor related events
    EVT_SENSOR_DATA_READY,
    EVT_SENSOR_READ_ERROR,
    EVT_SENSOR_DISCONNECTED,
    EVT_SENSOR_RECONNECTED,
    EVT_SENSOR_CALIBRATED,
    EVT_SENSOR_OVERRANGE,

    // UI related events (e.g., button input)
    EVT_USER_BUTTON_PRESSED,

    // System events
    EVT_SYSTEM_ERROR,
    EVT_LOW_MEMORY_WARNING,
} EventType_t;

// Event source identification
typedef enum {
    EVENT_SOURCE_UNKNOWN = 0,
    EVENT_SOURCE_CAN,
    EVENT_SOURCE_SPI,
    EVENT_SOURCE_SENSOR,
    EVENT_SOURCE_SENSOR_MANAGER,
    EVENT_SOURCE_TASK_MANAGER,
    EVENT_SOURCE_UI,
    EVENT_SOURCE_SYSTEM
} EventSource_t;

typedef struct {
    uint8_t ucSensorId;
    uint8_t ucSensorType;
    uint32_t ulTimestamp;
    uint32_t ulSequenceNumber;
} SensorEventInfo_t;

typedef struct {
    uint32_t ulErrorCode;
    uint32_t ulMemoryFree;
    uint32_t ulTimestamp;
} SystemEventInfo_t;

typedef struct {
    EventType_t   eType;
    EventSource_t eSource;
    void*         pPayload;
    uint32_t      ulDataSize;

    // Extended event information based on event type
    union {
        SensorEventInfo_t xSensorInfo;  // For sensor-related events
        SystemEventInfo_t xSystemInfo;  // For system events
        uint32_t ulGenericInfo;         // For other events
    } xEventInfo;
} Event_t;


#endif // EVENTS_H
