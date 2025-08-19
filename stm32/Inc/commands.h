#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

// Defines the type of commands that the StateManager will issue to the Worker Tasks.
typedef enum {
    CMD_SPI_SEND_DATA,      // Command to send data via SPI
    CMD_CAN_SEND_DATA,      // Command to send data via CAN
    CMD_DISPLAY_UPDATE,     // Command to update the display
} CommandType_t;

// This is the struct that will be passed through the command queue.
typedef struct {
    CommandType_t eType;        // Command type
    void*         pPayload;     // Data to be passed along with the command
    uint32_t      ulDataSize;   // Payload data size
} Command_t;


#endif // COMMANDS_H
