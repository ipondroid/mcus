#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_INITIALIZING,      // Initializing
    MODE_NORMAL_OPERATION,  // Normal operation
    MODE_LOW_POWER,         // Low power mode
    MODE_ERROR              // System error state
} SystemMode_t;

typedef struct {
    SystemMode_t eSystemMode;

    bool bIsSpiConnected;       // Whether SPI is connected with ESP32
    bool bIsCanActive;        // Whether the CAN bus is active

    float fLastTemperature;
    float fLastHumidity;

    uint32_t ulLastCanRxTimestamp;
    uint32_t ulLastSpiRxTimestamp;

} SystemState_t;

// Global system state
extern SystemState_t g_SystemState;

#endif // SYSTEM_STATE_H
