#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_INITIALIZING,      // Initializing
    MODE_ADVERTISING,       // Advertising via BLE
    MODE_CONNECTED,         // Connected with a BLE client
    MODE_ERROR              // System error state
} SystemMode_t;

typedef struct {
    SystemMode_t eSystemMode;

    bool bIsBleClientConnected;
    uint16_t u16BleConnId; // Connection ID
    void*    pGattsIf;     // GATT server interface

    float fLastTemperature;
    float fLastHumidity;

} SystemState_t;

extern SystemState_t g_SystemState;

#endif // SYSTEM_STATE_H
