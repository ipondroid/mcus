#include "display_task.h"
#include "state_manager.h"
#include "commands.h"
#include "sensor_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#include "4ilo/4ilo_ssd1306.h"

static TaskHandle_t g_xDisplayTaskHandle = NULL;

extern void Error_Handler(void);

static void prvDisplayTask(void* pvParameters) {
    Command_t xReceivedCommand;
    char tempStr[20];
    char humStr[20];

    printf("DST\n\r");

    if (ssd1306_Init(&hi2c1) != 0) {
        Error_Handler();
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    for (;;) {
        if (xQueueReceive(g_xDisplayCommandQueue, &xReceivedCommand, portMAX_DELAY) == pdPASS) {
            if (xReceivedCommand.eType == CMD_DISPLAY_UPDATE) {
                SensorData_t* pData = (SensorData_t*)xReceivedCommand.pPayload;

                snprintf(tempStr, sizeof(tempStr), "T: %.2f C", pData->fTemperature);
                snprintf(humStr, sizeof(humStr), "H: %.2f %%", pData->fHumidity);

                ssd1306_Fill(Black);
                ssd1306_UpdateScreen(&hi2c1);

                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(tempStr, Font_7x10, White);
                ssd1306_SetCursor(0, 12);
                ssd1306_WriteString(humStr, Font_7x10, White);
                ssd1306_UpdateScreen(&hi2c1);

                if (xReceivedCommand.pPayload != NULL) {
                    vPortFree(xReceivedCommand.pPayload);
                }
            }
        }
    }
}

BaseType_t DisplayTask_CreateTask(void) {
    return xTaskCreate(
        prvDisplayTask,
        "Display_Task",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY + 2,
        &g_xDisplayTaskHandle
    );
}

