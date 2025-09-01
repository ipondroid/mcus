#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "events.h"
#include "sensor_interface.h"
#include <stdint.h>

#define TASK_MANAGER_MAX_SENSORS 8
#define TASK_MANAGER_STATE_QUEUE_SIZE 16
#define TASK_MANAGER_PRIORITY_BOOST_MAX 3

typedef enum {
    TASK_MANAGER_OK = 0,
    TASK_MANAGER_ERROR,
    TASK_MANAGER_INVALID_PARAM,
    TASK_MANAGER_NOT_FOUND,
    TASK_MANAGER_ALREADY_EXISTS,
    TASK_MANAGER_RESOURCE_EXHAUSTED
} TaskManagerStatus_t;

typedef enum {
    TASK_STATE_DORMANT,      
    TASK_STATE_READY,        
    TASK_STATE_RUNNING,      
    TASK_STATE_SUSPENDED,    
    TASK_STATE_BLOCKED,      
    TASK_STATE_ERROR         
} TaskState_t;

typedef enum {
    TASK_TRIGGER_TIMER,      
    TASK_TRIGGER_EVENT,      
    TASK_TRIGGER_CONDITION,  
    TASK_TRIGGER_DEMAND      
} TaskTriggerType_t;

typedef enum {
    PRIORITY_CONDITION_ERROR_RATE,     
    PRIORITY_CONDITION_DATA_FRESHNESS, 
    PRIORITY_CONDITION_SYSTEM_LOAD,    
    PRIORITY_CONDITION_USER_DEMAND,    
    PRIORITY_CONDITION_SENSOR_TYPE     
} PriorityCondition_t;

typedef struct {
    uint32_t ulDataAge;           
    uint32_t ulErrorRate;         
    uint32_t ulImportanceScore;   
    uint8_t ucDemandLevel;        
    uint32_t ulSystemLoad;        
} SensorMetrics_t;

typedef struct {
    PriorityCondition_t eCondition;
    uint32_t ulThreshold;
    UBaseType_t uxPriorityBoost;
    uint32_t ulDurationMs;
    uint8_t ucActive;
} PriorityRule_t;

typedef struct {
    uint8_t ucSensorId;
    TaskState_t eCurrentState;
    TaskState_t eDesiredState;
    UBaseType_t uxCurrentPriority;
    UBaseType_t uxBasePriority;
    UBaseType_t uxMaxPriority;
    TaskTriggerType_t eTriggerType;
    uint32_t ulStateChangeTime;
    uint32_t ulLastActivityTime;
    uint32_t ulPriorityBoostExpiry;
    SensorMetrics_t xMetrics;
    TaskHandle_t xTaskHandle;
    uint8_t ucActive;
} TaskControlBlock_t;

typedef struct {
    uint8_t ucSensorId;
    TaskState_t eNewState;
    EventType_t eTriggerEvent;
    uint32_t ulTimestamp;
} StateChangeRequest_t;

typedef struct {
    TaskControlBlock_t axTasks[TASK_MANAGER_MAX_SENSORS];
    QueueHandle_t xStateChangeQueue;
    SemaphoreHandle_t xTaskManagerMutex;
    PriorityRule_t axPriorityRules[8];
    uint8_t ucActiveTasks;
    uint8_t ucActivePriorityRules;
    uint32_t ulSystemLoadMetric;
    uint8_t ucInitialized;
} TaskManagerContext_t;

typedef void (*TaskEventHandler_t)(uint8_t ucSensorId, const Event_t* pEvent);

TaskManagerStatus_t TaskManager_Init(void);
TaskManagerStatus_t TaskManager_Deinit(void);

TaskManagerStatus_t TaskManager_RegisterTask(uint8_t ucSensorId, TaskHandle_t xTaskHandle, 
                                           UBaseType_t uxBasePriority, UBaseType_t uxMaxPriority);
TaskManagerStatus_t TaskManager_UnregisterTask(uint8_t ucSensorId);

TaskManagerStatus_t TaskManager_TransitionState(uint8_t ucSensorId, TaskState_t eNewState);
TaskManagerStatus_t TaskManager_SuspendTask(uint8_t ucSensorId);
TaskManagerStatus_t TaskManager_ResumeTask(uint8_t ucSensorId);
TaskManagerStatus_t TaskManager_BlockTask(uint8_t ucSensorId);

TaskManagerStatus_t TaskManager_AdjustPriority(uint8_t ucSensorId, UBaseType_t uxNewPriority);
TaskManagerStatus_t TaskManager_BoostPriority(uint8_t ucSensorId, uint32_t ulDurationMs);
TaskManagerStatus_t TaskManager_RestoreBasePriority(uint8_t ucSensorId);

TaskManagerStatus_t TaskManager_UpdateMetrics(uint8_t ucSensorId, const SensorMetrics_t* pMetrics);
TaskManagerStatus_t TaskManager_GetTaskState(uint8_t ucSensorId, TaskState_t* pState);
TaskManagerStatus_t TaskManager_GetTaskMetrics(uint8_t ucSensorId, SensorMetrics_t* pMetrics);

TaskManagerStatus_t TaskManager_ProcessStateChanges(void);
TaskManagerStatus_t TaskManager_HandleEvent(const Event_t* pEvent);

TaskManagerStatus_t TaskManager_RegisterEventHandler(EventType_t eEventType, TaskEventHandler_t pfnHandler);

uint32_t TaskManager_CalculateAdaptiveInterval(uint8_t ucSensorId, const SensorMetrics_t* pMetrics);
uint8_t TaskManager_ShouldActivateTask(uint8_t ucSensorId);

const char* TaskState_GetName(TaskState_t eState);
const char* TaskTrigger_GetName(TaskTriggerType_t eTrigger);

#endif // TASK_MANAGER_H