#include "freertos-debug-utils.h"
// this requires some modification to the build
// see https://docs.platformio.org/en/latest/frameworks/espidf.html#configuration-for-4-0
// current version of platform io does not support menuconfig
// leaving this here for now when the support lands in a release. 

// https://www.freertos.org/uxTaskGetSystemState.html

void vTaskGetRunTimeStats( char *pcWriteBuffer )
{
TaskStatus_t *pxTaskStatusArray;
volatile UBaseType_t uxArraySize, x;
double ulTotalRunTime, ulStatsAsPercentage;

   /* Make sure the write buffer does not contain a string. */
   *pcWriteBuffer = 0x00;

   /* Take a snapshot of the number of tasks in case it changes while this
   function is executing. */
   uxArraySize = uxTaskGetNumberOfTasks();

   /* Allocate a TaskStatus_t structure for each task.  An array could be
   allocated statically at compile time. */
   pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL )
   {
      /* Generate raw status information about each task. */
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 (uint32_t *)&ulTotalRunTime );

      /* For percentage calculations. */
      ulTotalRunTime /= 100UL;

      /* Avoid divide by zero errors. */
      if( ulTotalRunTime > 0 )
      {
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
         for( x = 0; x < uxArraySize; x++ )
         {
            /* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage =
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            if( ulStatsAsPercentage > 0UL )
            {
               sprintf( pcWriteBuffer, "%stt%utt%lf%%rn",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                                 ulStatsAsPercentage );
            }
            else
            {
               /* If the percentage is zero here then the task has
               consumed less than 1% of the total run time. */
               sprintf( pcWriteBuffer, "%stt%utt<1%%rn",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter );
            }

            pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
         }
      }

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
}

// https://www.freertos.org/vTaskGetInfo.html
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#_CPPv412vTaskGetInfo12TaskHandle_tP12TaskStatus_t10BaseType_t10eTaskState
// void vAFunction( void )
// {
// TaskHandle_t xHandle;
// TaskStatus_t xTaskDetails;

//     /* Obtain the handle of a task from its name. */
//     xHandle = xTaskGetHandle( "Task_Name" );

//     /* Check the handle is not NULL. */
//     configASSERT( xHandle );

//     /* Use the handle to obtain further information about the task. */
//     vTaskGetInfo( /* The handle of the task being queried. */
//                   xHandle,
//                   /* The TaskStatus_t structure to complete with information
//                   on xTask. */
//                   &xTaskDetails,
//                   /* Include the stack high water mark value in the
//                   TaskStatus_t structure. */
//                   pdTRUE,
//                   /* Include the task state in the TaskStatus_t structure. */
//                   eInvalid );
// }
 

