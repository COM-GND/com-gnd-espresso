#ifndef FREERTOSDEBUG_H
#define FREERTOSDEBUG_H

#define configUSE_TRACE_FACILITY 1
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

void vTaskGetRunTimeStats(signed char *pcWriteBuffer);
void vAFunction( void );

// The TaskStatus_t definition
// typedef struct xTASK_STATUS
// {
//    /* The handle of the task to which the rest of the information in the
//    structure relates. */
//    TaskHandle_t xHandle;

//    /* A pointer to the task's name.  This value will be invalid if the task was
//    deleted since the structure was populated! */
//    const signed char *pcTaskName;

//    /* A number unique to the task. */
//    UBaseType_t xTaskNumber;

//    /* The state in which the task existed when the structure was populated. */
//    eTaskState eCurrentState;

//    /* The priority at which the task was running (may be inherited) when the
//    structure was populated. */
//    UBaseType_t uxCurrentPriority;

//    /* The priority to which the task will return if the task's current priority
//    has been inherited to avoid unbounded priority inversion when obtaining a
//    mutex.  Only valid if configUSE_MUTEXES is defined as 1 in
//    FreeRTOSConfig.h. */
//    UBaseType_t uxBasePriority;

//    /* The total run time allocated to the task so far, as defined by the run
//    time stats clock.  Only valid when configGENERATE_RUN_TIME_STATS is
//    defined as 1 in FreeRTOSConfig.h. */
//    unsigned long ulRunTimeCounter;

//    /* Points to the lowest address of the task's stack area. */
//    StackType_t *pxStackBase;

//    /* The minimum amount of stack space that has remained for the task since
//    the task was created.  The closer this value is to zero the closer the task
//    has come to overflowing its stack. */
//    uint8_t usStackHighWaterMark;
// } TaskStatus_t;

#endif