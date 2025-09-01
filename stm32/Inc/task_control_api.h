#ifndef TASK_CONTROL_API_H
#define TASK_CONTROL_API_H

#include "task_manager.h"
#include "priority_manager.h"

typedef enum {
    CONTROL_CMD_SUSPEND_SENSOR,
    CONTROL_CMD_RESUME_SENSOR,
    CONTROL_CMD_BOOST_PRIORITY,
    CONTROL_CMD_SET_DEMAND_LEVEL,
    CONTROL_CMD_EMERGENCY_MODE,
    CONTROL_CMD_NORMAL_MODE
} ControlCommand_t;

typedef struct {
    ControlCommand_t eCommand;
    uint8_t ucSensorId;
    uint32_t ulParameter1;
    uint32_t ulParameter2;
} ControlRequest_t;

TaskManagerStatus_t TaskControl_ProcessCommand(const ControlRequest_t* pRequest);

TaskManagerStatus_t TaskControl_SuspendSensor(uint8_t ucSensorId);
TaskManagerStatus_t TaskControl_ResumeSensor(uint8_t ucSensorId);
TaskManagerStatus_t TaskControl_BoostSensorPriority(uint8_t ucSensorId, uint32_t ulDurationMs);
TaskManagerStatus_t TaskControl_SetDemandLevel(uint8_t ucSensorId, uint8_t ucDemandLevel);

TaskManagerStatus_t TaskControl_EmergencyMode(void);
TaskManagerStatus_t TaskControl_NormalMode(void);

TaskManagerStatus_t TaskControl_GetSystemStatus(uint8_t* pActiveTasks, uint32_t* pSystemLoad);

#endif // TASK_CONTROL_API_H