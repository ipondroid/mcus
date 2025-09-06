#include "spi_task.h"
#include "state_manager.h"
#include "events.h"
#include "commands.h"
#include "sensor_interface.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static TaskHandle_t g_xSpiTaskHandle = NULL;

static void prvSpiTask(void* pvParameters) {
    Command_t xReceivedCommand;
    uint8_t txData[4];
    uint8_t intPart, fractPart;

    printf("SPIT\n\r");

    for (;;) {
        if (xQueueReceive(g_xSpiCommandQueue, &xReceivedCommand, portMAX_DELAY) == pdPASS) {
            if (xReceivedCommand.eType == CMD_SPI_SEND_DATA) {
                // printf("SPIT: Received CMD_SPI_SEND_DATA\n\r");

                SensorData_t *sensorData = (SensorData_t *)xReceivedCommand.pPayload;
                intPart = (uint8_t)sensorData->fTemperature;
                fractPart = (uint8_t)(sensorData->fTemperature*100 - intPart*100);
                txData[0] = intPart;
                txData[1] = fractPart;
                intPart = (uint8_t)sensorData->fHumidity;
                fractPart = (uint8_t)(sensorData->fHumidity*100 - intPart*100);
                txData[2] = intPart;
                txData[3] = fractPart;

                printf("SPIT [%02x %02x %02x %02x]\n\r", txData[0], txData[1], txData[2], txData[3]);

                HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET);

                HAL_SPI_Transmit(&hspi2, txData, 4, HAL_MAX_DELAY);

                HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);

                if (xReceivedCommand.pPayload != NULL) {
                    vPortFree(xReceivedCommand.pPayload);
                }
            }
        }
    }
}

BaseType_t SpiTask_CreateTask(void) {
    return xTaskCreate(
        prvSpiTask,
        "SPI_Task",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY + 2,
        &g_xSpiTaskHandle
    );
}

