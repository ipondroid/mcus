#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_INITIALIZING,
    MODE_NORMAL_OPERATION,
    MODE_LOW_POWER,
    MODE_ERROR
} SystemMode_t;

typedef struct {
    SystemMode_t eSystemMode;

    bool bIsSpiConnected;
    bool bIsCanActive;

    float fLastTemperature;
    float fLastHumidity;

    uint32_t ulLastCanRxTimestamp;
    uint32_t ulLastSpiRxTimestamp;

} SystemState_t;

extern SystemState_t g_SystemState;

#endif // SYSTEM_STATE_H
