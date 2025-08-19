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

    // UI related events (e.g., button input)
    EVT_USER_BUTTON_PRESSED,

} EventType_t;

typedef struct {
    EventType_t eType;      // Event type
    void*       pPayload;   // Data
    uint32_t    ulDataSize; // Payload data size
} Event_t;


#endif // EVENTS_H
