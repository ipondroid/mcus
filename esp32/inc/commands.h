#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include "shared_types.h"

typedef enum {
    CMD_BLE_NOTIFY_DATA,
    CMD_SPI_SEND_DATA,
} CommandType_t;

typedef struct {
    CommandType_t eType;
    SensorData_t  xSensorData;
} Command_t;


#endif // COMMANDS_H
