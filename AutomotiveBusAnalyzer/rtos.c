// FreeRTOS includes
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <led.h>
#include "FreeRTOSConfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <ring.h>
#include "rtos.h"
#include "board.h"

#include "tasks/task_CAN.h"
#include "tasks/task_LOG.h"

enum
{
  tsk_IDLE_PRIORITY = tskIDLE_PRIORITY,
  tsk_INIT_PRIORITY,
  tsk_USB_PRIORITY,
  tsk_CAN_PRIORITY = tsk_USB_PRIORITY,
  tsk_LOG_PRIORITY = tsk_USB_PRIORITY,
};

//---------------------------------------------------------------------------------------------------------------------
#define TASK_INIT_STACK_SIZE 4096
#define TASK_CAN_STACK_SIZE 1024
#define TASK_USB_STACK_SIZE 1024
#define TASK_LOG_STACK_SIZE 1024

COMPILER_SECTION(".data_TCM") StaticTask_t xInitTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxInitTaskStack[ TASK_INIT_STACK_SIZE ];

COMPILER_SECTION(".data_TCM") StaticTask_t xCanTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxCanTaskStack[ TASK_CAN_STACK_SIZE ];

COMPILER_SECTION(".data_TCM") StaticTask_t xUsbTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxUsbTaskStack[ TASK_USB_STACK_SIZE ];

COMPILER_SECTION(".data_TCM") StaticTask_t xLogTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxLogTaskStack[ TASK_LOG_STACK_SIZE ];

TaskHandle_t hCanTask;
TaskHandle_t hUsbTask;
TaskHandle_t hUsbRxTask;
TaskHandle_t hLogTask;

#define CONSOLE_BUFFER_LINE_LEN 80u
char consoleBuffer[64][CONSOLE_BUFFER_LINE_LEN];
uint8_t cb_head = 0;
uint8_t cb_tail = 0;
uint32_t g_twinkle = 0;

//---------------------------------------------------------------------------------------------------------------------
#ifdef DEBUG
extern void dbgprintf(const char * format, ...)
{
  if(RING_SPACE(cb_head, cb_tail, 16))
  {
    va_list args;
    va_start(args, format);
    vsnprintf(consoleBuffer[cb_head], CONSOLE_BUFFER_LINE_LEN, format, args);
    RING_INC(cb_head, 16u);
    va_end(args);
  }
}
#endif


//---------------------------------------------------------------------------------------------------------------------
static void _rtos_can_task(void* params)
{
  (void)params;
  rtos_can_task();
  vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------
static void _rtos_usb_task(void* params)
{
  (void)params;
  rtos_usb_task();
  vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------
static void _rtos_log_task(void* params)
{
  (void)params;
  rtos_log_task();
  vTaskDelete(NULL);
}

static void _rtos_init_task(void* params)
{
    (void)params;
    uint32_t board = init_task_callback();

    hLogTask = xTaskCreateStatic(_rtos_log_task, "LOG", TASK_LOG_STACK_SIZE, NULL, tsk_LOG_PRIORITY, uxLogTaskStack, &xLogTaskTCB);
    hCanTask = xTaskCreateStatic(_rtos_can_task, "CAN", TASK_CAN_STACK_SIZE, NULL, tsk_CAN_PRIORITY, uxCanTaskStack, &xCanTaskTCB);
    hUsbTask = xTaskCreateStatic(_rtos_usb_task, "USB", TASK_USB_STACK_SIZE, NULL, tsk_USB_PRIORITY, uxUsbTaskStack, &xUsbTaskTCB);

    rtos_usb_task_rx();

    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------
void rtos_run(void)
{
    hUsbRxTask = xTaskCreateStatic(_rtos_init_task, "INIT", TASK_INIT_STACK_SIZE, NULL, tsk_INIT_PRIORITY, uxInitTaskStack, &xInitTaskTCB);

    // run scheduler
    vTaskStartScheduler();
}


//---------------------------------------------------------------------------------------------------------------------
extern void vApplicationMallocFailedHook(void)
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void*)NULL);
}


//---------------------------------------------------------------------------------------------------------------------
extern void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT((volatile void*)NULL);
}


//---------------------------------------------------------------------------------------------------------------------
extern void vApplicationIdleHook(void)
{
    static TickType_t initialTick = 0;
    static TickType_t lastTwinkle = 0;
    static uint32_t stackTick = 0;
    //volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    //xFreeHeapSpace = xPortGetFreeHeapSize();

    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    //(void) xFreeHeapSpace;

    while(RING_CNT(cb_head, cb_tail, 16))
    {
      printf(consoleBuffer[cb_tail]);
      RING_INC(cb_tail, 16);
    }

    TickType_t ticks = xTaskGetTickCount();
    if( (ticks - initialTick) > 1000 )
    {
      printf("\r\nWarn: Not idle for 1000 ticks\r\n");
    }
    
    if(g_twinkle)
    {
      if(ticks - lastTwinkle > 125)
      {
        lastTwinkle = ticks;
        g_twinkle--;
        (g_twinkle & 1) ? Board_SetLed(LED_USB, LED_TRI_R, LED_PRIO_LOW) : Board_SetLed(LED_USB, LED_TRI_OFF, LED_PRIO_CLEAR);
      }
      
      if(g_twinkle == 0) {
        // When twinkle is done, return to LED_TRI_B. When additional data is sent, it will turn to green eventually.
        // It cannot be off as we have received data already => configured. An error would be masked by twinkle and will be cleared
        // by the next successful data transmission
        Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_CLEAR);
      }
    }

    initialTick = ticks;

#if INCLUDE_uxTaskGetStackHighWaterMark
  stackTick++;
  if(stackTick > 10000000)
  {
    printf("CAN Stack max usage: %lu byte\r\n", uxTaskGetStackHighWaterMark(hCanTask));
    printf("USB Stack max usage: %lu byte\r\n", uxTaskGetStackHighWaterMark(hUsbTask));
    printf("LOG Stack max usage: %lu byte\r\n", uxTaskGetStackHighWaterMark(hLogTask));
    printf("INIT Stack max usage: %lu byte\r\n", uxTaskGetStackHighWaterMark(hUsbRxTask));
    stackTick = 0;
  }
#endif

}


//---------------------------------------------------------------------------------------------------------------------
extern void vApplicationTickHook(void)
{
    TimeTick_Tick();
}


//---------------------------------------------------------------------------------------------------------------------
#if configSUPPORT_STATIC_ALLOCATION
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
COMPILER_SECTION(".data_TCM") StaticTask_t xIdleTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an implementation
of vApplicationGetIdleTaskMemory() to provide the memory that is used by the Idle task. */
extern void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                           StackType_t **ppxIdleTaskStackBuffer,
                                           uint32_t *pulIdleTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
#endif


//---------------------------------------------------------------------------------------------------------------------
#if configSUPPORT_STATIC_ALLOCATION && configUSE_TIMERS
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
COMPILER_SECTION(".data_TCM") StaticTask_t xTimerTaskTCB;
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(8) StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
extern void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                            StackType_t **ppxTimerTaskStackBuffer,
                                            uint32_t *pulTimerTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
