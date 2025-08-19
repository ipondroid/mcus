#ifndef EVENTS_H
#define EVENTS_H

#include <stdint.h>
#include "shared_types.h"

typedef enum {
    EVT_SPI_DATA_RECEIVED,

    EVT_BLE_CLIENT_CONNECTED,
    EVT_BLE_CLIENT_DISCONNECTED,
    EVT_BLE_NOTIFICATION_ENABLED,
    EVT_BLE_NOTIFICATION_DISABLED,
    EVT_BLE_READ_REQUEST,

} EventType_t;

typedef struct {
    EventType_t   eType;
    SensorData_t  xSensorData;
} Event_t;


#endif // EVENTS_H
