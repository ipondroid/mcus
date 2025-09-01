#include "task_manager.h"
#include "sensor_manager.h"
#include <string.h>
#include <stdio.h>

static TaskManagerContext_t g_xTaskManager = {0};
static TaskEventHandler_t g_apfnEventHandlers[EVT_SYSTEM_ERROR + 1] = {NULL};

typedef struct {
    TaskState_t eFromState;
    TaskState_t eToState;
    EventType_t eTriggerEvent;
    uint32_t ulConditionMask;
} StateTransition_t;

static const StateTransition_t g_axStateTransitions[] = {
    {TASK_STATE_RUNNING, TASK_STATE_DORMANT, EVT_SENSOR_DISCONNECTED, 0},
    {TASK_STATE_DORMANT, TASK_STATE_READY, EVT_SENSOR_RECONNECTED, 0},
    {TASK_STATE_RUNNING, TASK_STATE_ERROR, EVT_SENSOR_READ_ERROR, 0},
    {TASK_STATE_RUNNING, TASK_STATE_SUSPENDED, EVT_LOW_MEMORY_WARNING, 0}
};



static TaskControlBlock_t* prvFindTaskControlBlock(uint8_t ucSensorId);
static TaskManagerStatus_t prvValidateTransition(TaskState_t eCurrentState, TaskState_t eNewState);
static void prvExecuteStateTransition(uint8_t ucSensorId, TaskState_t eNewState);

TaskManagerStatus_t TaskManager_Init(void) {
    if (g_xTaskManager.ucInitialized) {
        return TASK_MANAGER_OK;
    }

    memset(&g_xTaskManager, 0, sizeof(TaskManagerContext_t));

    g_xTaskManager.xStateChangeQueue = xQueueCreate(TASK_MANAGER_STATE_QUEUE_SIZE, sizeof(StateChangeRequest_t));
    if (g_xTaskManager.xStateChangeQueue == NULL) {
        return TASK_MANAGER_ERROR;
    }

    g_xTaskManager.xTaskManagerMutex = xSemaphoreCreateMutex();
    if (g_xTaskManager.xTaskManagerMutex == NULL) {
        vQueueDelete(g_xTaskManager.xStateChangeQueue);
        return TASK_MANAGER_ERROR;
    }

    printf("TM: Task Manager initialized\\n\\r");
    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_Deinit(void) {
    if (!g_xTaskManager.ucInitialized) {
        return TASK_MANAGER_OK;
    }

    if (g_xTaskManager.xStateChangeQueue != NULL) {
        vQueueDelete(g_xTaskManager.xStateChangeQueue);
    }

    if (g_xTaskManager.xTaskManagerMutex != NULL) {
        vSemaphoreDelete(g_xTaskManager.xTaskManagerMutex);
    }

    g_xTaskManager.ucInitialized = 0;
    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_RegisterTask(uint8_t ucSensorId, TaskHandle_t xTaskHandle, 
                                           UBaseType_t uxBasePriority, UBaseType_t uxMaxPriority) {
    if (!g_xTaskManager.ucInitialized || xTaskHandle == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_xTaskManager.xTaskManagerMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return TASK_MANAGER_ERROR;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock != NULL && pTaskBlock->ucActive) {
        xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);
        return TASK_MANAGER_ALREADY_EXISTS;
    }

    if (pTaskBlock == NULL) {
        for (uint8_t i = 0; i < TASK_MANAGER_MAX_SENSORS; i++) {
            if (!g_xTaskManager.axTasks[i].ucActive) {
                pTaskBlock = &g_xTaskManager.axTasks[i];
                break;
            }
        }
    }

    if (pTaskBlock == NULL) {
        xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);
        return TASK_MANAGER_RESOURCE_EXHAUSTED;
    }

    memset(pTaskBlock, 0, sizeof(TaskControlBlock_t));
    pTaskBlock->ucSensorId = ucSensorId;
    pTaskBlock->eCurrentState = TASK_STATE_READY;
    pTaskBlock->eDesiredState = TASK_STATE_READY;
    pTaskBlock->uxCurrentPriority = uxBasePriority;
    pTaskBlock->uxBasePriority = uxBasePriority;
    pTaskBlock->uxMaxPriority = uxMaxPriority;
    pTaskBlock->eTriggerType = TASK_TRIGGER_TIMER;
    pTaskBlock->ulStateChangeTime = xTaskGetTickCount();
    pTaskBlock->ulLastActivityTime = xTaskGetTickCount();
    pTaskBlock->ulPriorityBoostExpiry = 0;
    pTaskBlock->xTaskHandle = xTaskHandle;
    pTaskBlock->ucActive = 1;

    g_xTaskManager.ucActiveTasks++;

    xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);

    printf("TM: Registered task for sensor %d\\n\\r", ucSensorId);
    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_TransitionState(uint8_t ucSensorId, TaskState_t eNewState) {
    if (!g_xTaskManager.ucInitialized) {
        return TASK_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xTaskManager.xTaskManagerMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return TASK_MANAGER_ERROR;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);
        return TASK_MANAGER_NOT_FOUND;
    }

    if (prvValidateTransition(pTaskBlock->eCurrentState, eNewState) != TASK_MANAGER_OK) {
        xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);
        return TASK_MANAGER_INVALID_PARAM;
    }

    TaskState_t eOldState = pTaskBlock->eCurrentState;
    prvExecuteStateTransition(ucSensorId, eNewState);
    
    xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);

    printf("TM: Sensor %d state: %s -> %s\\n\\r", 
           ucSensorId, TaskState_GetName(eOldState), TaskState_GetName(eNewState));

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_SuspendTask(uint8_t ucSensorId) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    if (pTaskBlock->xTaskHandle != NULL) {
        vTaskSuspend(pTaskBlock->xTaskHandle);
        pTaskBlock->eCurrentState = TASK_STATE_SUSPENDED;
        pTaskBlock->ulStateChangeTime = xTaskGetTickCount();
    }

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_ResumeTask(uint8_t ucSensorId) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    if (pTaskBlock->xTaskHandle != NULL && pTaskBlock->eCurrentState == TASK_STATE_SUSPENDED) {
        vTaskResume(pTaskBlock->xTaskHandle);
        pTaskBlock->eCurrentState = TASK_STATE_RUNNING;
        pTaskBlock->ulStateChangeTime = xTaskGetTickCount();
    }

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_AdjustPriority(uint8_t ucSensorId, UBaseType_t uxNewPriority) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    if (uxNewPriority > pTaskBlock->uxMaxPriority) {
        uxNewPriority = pTaskBlock->uxMaxPriority;
    }

    if (pTaskBlock->xTaskHandle != NULL) {
        vTaskPrioritySet(pTaskBlock->xTaskHandle, uxNewPriority);
        pTaskBlock->uxCurrentPriority = uxNewPriority;
    }

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_BoostPriority(uint8_t ucSensorId, uint32_t ulDurationMs) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    UBaseType_t uxBoostedPriority = pTaskBlock->uxBasePriority + TASK_MANAGER_PRIORITY_BOOST_MAX;
    if (uxBoostedPriority > pTaskBlock->uxMaxPriority) {
        uxBoostedPriority = pTaskBlock->uxMaxPriority;
    }

    TaskManager_AdjustPriority(ucSensorId, uxBoostedPriority);
    pTaskBlock->ulPriorityBoostExpiry = xTaskGetTickCount() + pdMS_TO_TICKS(ulDurationMs);

    printf("TM: Boosted sensor %d priority to %d for %dms\\n\\r", 
           ucSensorId, (int)uxBoostedPriority, (int)ulDurationMs);

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_UpdateMetrics(uint8_t ucSensorId, const SensorMetrics_t* pMetrics) {
    if (pMetrics == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    memcpy(&pTaskBlock->xMetrics, pMetrics, sizeof(SensorMetrics_t));
    pTaskBlock->ulLastActivityTime = xTaskGetTickCount();

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_HandleEvent(const Event_t* pEvent) {
    if (pEvent == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    if (pEvent->eType <= EVT_SYSTEM_ERROR && g_apfnEventHandlers[pEvent->eType] != NULL) {
        g_apfnEventHandlers[pEvent->eType](pEvent->xEventInfo.xSensorInfo.ucSensorId, pEvent);
    }

    for (uint8_t i = 0; i < sizeof(g_axStateTransitions) / sizeof(StateTransition_t); i++) {
        if (g_axStateTransitions[i].eTriggerEvent == pEvent->eType) {
            uint8_t ucSensorId = pEvent->xEventInfo.xSensorInfo.ucSensorId;
            TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
            
            if (pTaskBlock != NULL && pTaskBlock->ucActive && 
                pTaskBlock->eCurrentState == g_axStateTransitions[i].eFromState) {
                
                StateChangeRequest_t xRequest = {
                    .ucSensorId = ucSensorId,
                    .eNewState = g_axStateTransitions[i].eToState,
                    .eTriggerEvent = pEvent->eType,
                    .ulTimestamp = xTaskGetTickCount()
                };
                
                xQueueSend(g_xTaskManager.xStateChangeQueue, &xRequest, 0);
            }
        }
    }

    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_ProcessStateChanges(void) {
    StateChangeRequest_t xRequest;
    
    while (xQueueReceive(g_xTaskManager.xStateChangeQueue, &xRequest, 0) == pdTRUE) {
        TaskManager_TransitionState(xRequest.ucSensorId, xRequest.eNewState);
    }
    
    return TASK_MANAGER_OK;
}

uint32_t TaskManager_CalculateAdaptiveInterval(uint8_t ucSensorId, const SensorMetrics_t* pMetrics) {
    if (pMetrics == NULL) {
        return 5000; // Default 5 seconds
    }

    uint32_t ulBaseInterval = 5000;
    uint32_t ulInterval = ulBaseInterval;

    if (pMetrics->ulErrorRate > 50) {
        ulInterval *= 2; // High error rate increases interval
    } else if (pMetrics->ulErrorRate < 5) {
        ulInterval = ulInterval * 80 / 100; // Low error rate decreases interval
    }

    if (pMetrics->ucDemandLevel > 200) {
        ulInterval = ulInterval * 50 / 100; // High demand decreases interval
    } else if (pMetrics->ucDemandLevel < 50) {
        ulInterval *= 2; // Low demand increases interval
    }

    if (ulInterval < 1000) ulInterval = 1000;   // Minimum 1 second
    if (ulInterval > 30000) ulInterval = 30000; // Maximum 30 seconds

    return ulInterval;
}

TaskManagerStatus_t TaskManager_RegisterEventHandler(EventType_t eEventType, TaskEventHandler_t pfnHandler) {
    if (eEventType > EVT_SYSTEM_ERROR || pfnHandler == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    g_apfnEventHandlers[eEventType] = pfnHandler;
    return TASK_MANAGER_OK;
}

static TaskControlBlock_t* prvFindTaskControlBlock(uint8_t ucSensorId) {
    for (uint8_t i = 0; i < TASK_MANAGER_MAX_SENSORS; i++) {
        if (g_xTaskManager.axTasks[i].ucActive && 
            g_xTaskManager.axTasks[i].ucSensorId == ucSensorId) {
            return &g_xTaskManager.axTasks[i];
        }
    }
    return NULL;
}

static TaskManagerStatus_t prvValidateTransition(TaskState_t eCurrentState, TaskState_t eNewState) {
    if (eCurrentState == eNewState) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    switch (eCurrentState) {
        case TASK_STATE_DORMANT:
            return (eNewState == TASK_STATE_READY) ? TASK_MANAGER_OK : TASK_MANAGER_INVALID_PARAM;
        
        case TASK_STATE_READY:
        case TASK_STATE_RUNNING:
            return TASK_MANAGER_OK; // Can transition to any state
        
        case TASK_STATE_SUSPENDED:
            return (eNewState == TASK_STATE_READY || eNewState == TASK_STATE_RUNNING || eNewState == TASK_STATE_ERROR) 
                   ? TASK_MANAGER_OK : TASK_MANAGER_INVALID_PARAM;
        
        case TASK_STATE_ERROR:
            return (eNewState == TASK_STATE_READY || eNewState == TASK_STATE_DORMANT) 
                   ? TASK_MANAGER_OK : TASK_MANAGER_INVALID_PARAM;
        
        default:
            return TASK_MANAGER_INVALID_PARAM;
    }
}

static void prvExecuteStateTransition(uint8_t ucSensorId, TaskState_t eNewState) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL) return;

    pTaskBlock->eCurrentState = eNewState;
    pTaskBlock->ulStateChangeTime = xTaskGetTickCount();

    switch (eNewState) {
        case TASK_STATE_SUSPENDED:
            TaskManager_SuspendTask(ucSensorId);
            break;
        
        case TASK_STATE_RUNNING:
        case TASK_STATE_READY:
            if (pTaskBlock->eCurrentState == TASK_STATE_SUSPENDED) {
                TaskManager_ResumeTask(ucSensorId);
            }
            break;
        
        case TASK_STATE_ERROR:
            TaskManager_AdjustPriority(ucSensorId, 1); // 최저 우선순위
            break;
        
        default:
            break;
    }
}

TaskManagerStatus_t TaskManager_RestoreBasePriority(uint8_t ucSensorId) {
    return TaskManager_AdjustPriority(ucSensorId, 
                                     prvFindTaskControlBlock(ucSensorId)->uxBasePriority);
}

TaskManagerStatus_t TaskManager_UnregisterTask(uint8_t ucSensorId) {
    if (!g_xTaskManager.ucInitialized) {
        return TASK_MANAGER_ERROR;
    }

    if (xSemaphoreTake(g_xTaskManager.xTaskManagerMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return TASK_MANAGER_ERROR;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);
        return TASK_MANAGER_NOT_FOUND;
    }

    pTaskBlock->ucActive = 0;
    pTaskBlock->xTaskHandle = NULL;
    g_xTaskManager.ucActiveTasks--;

    xSemaphoreGive(g_xTaskManager.xTaskManagerMutex);

    printf("TM: Unregistered task for sensor %d\\n\\r", ucSensorId);
    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_BlockTask(uint8_t ucSensorId) {
    return TaskManager_TransitionState(ucSensorId, TASK_STATE_BLOCKED);
}

TaskManagerStatus_t TaskManager_GetTaskState(uint8_t ucSensorId, TaskState_t* pState) {
    if (pState == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    *pState = pTaskBlock->eCurrentState;
    return TASK_MANAGER_OK;
}

TaskManagerStatus_t TaskManager_GetTaskMetrics(uint8_t ucSensorId, SensorMetrics_t* pMetrics) {
    if (pMetrics == NULL) {
        return TASK_MANAGER_INVALID_PARAM;
    }

    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return TASK_MANAGER_NOT_FOUND;
    }

    memcpy(pMetrics, &pTaskBlock->xMetrics, sizeof(SensorMetrics_t));
    return TASK_MANAGER_OK;
}

uint8_t TaskManager_ShouldActivateTask(uint8_t ucSensorId) {
    TaskControlBlock_t* pTaskBlock = prvFindTaskControlBlock(ucSensorId);
    if (pTaskBlock == NULL || !pTaskBlock->ucActive) {
        return 0;
    }

    // Check if task should be activated based on metrics
    if (pTaskBlock->xMetrics.ucDemandLevel > 200) {
        return 1; // High demand
    }

    if (pTaskBlock->xMetrics.ulErrorRate < 10 && pTaskBlock->xMetrics.ulDataAge > 10000) {
        return 1; // Low error rate and stale data
    }

    return 0;
}

const char* TaskState_GetName(TaskState_t eState) {
    switch (eState) {
        case TASK_STATE_DORMANT: return "DORMANT";
        case TASK_STATE_READY: return "READY";
        case TASK_STATE_RUNNING: return "RUNNING";
        case TASK_STATE_SUSPENDED: return "SUSPENDED";
        case TASK_STATE_BLOCKED: return "BLOCKED";
        case TASK_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const char* TaskTrigger_GetName(TaskTriggerType_t eTrigger) {
    switch (eTrigger) {
        case TASK_TRIGGER_TIMER: return "TIMER";
        case TASK_TRIGGER_EVENT: return "EVENT";
        case TASK_TRIGGER_CONDITION: return "CONDITION";
        case TASK_TRIGGER_DEMAND: return "DEMAND";
        default: return "UNKNOWN";
    }
}