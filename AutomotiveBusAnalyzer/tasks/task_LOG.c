#include <board.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <message_buffer.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "tasks/task_LOG.h"
#include "rtos.h"

static volatile uint8_t start_logger = 0;
static volatile uint8_t stop_logger = 0;
static uint8_t logger_running = 0;
static StaticSemaphore_t sdMutexBuffer;
static SemaphoreHandle_t sdMutex;
static StreamBufferHandle_t logBufferHandle;
static StaticStreamBuffer_t  logStaticBuffer;
COMPILER_SECTION (".ram_nocache") COMPILER_ALIGNED(4) static FIL logFile;
COMPILER_SECTION (".ram_nocache") static uint8_t rxLogBuffer[16384*2];  // Buffer for writing on SD-CARD
static uint8_t logMessageBuffer[16384*4];   // Buffer for collecting data

char* componentToStr[] =
{
  "RTOS",
  "USB",
  "CAN",
  "LIN",
  "ETH",
  "SENT",
  "CXPI",
  "LOGC_MAX"
};

void notifyToggleLogger(void)
{
  /* Set a notify variable */
  if(logger_running) {
    stop_logger = 1;
  }
  else {
    start_logger = 1;
  }
      
  if(hLogTask != 0)
  {
    /* Unblock the task */
    xTaskNotifyFromISR(hLogTask, 0, eNoAction, 0);
  }    
}

static void startLogger(void)
{
  /* Check if SD card is present before enabling logger */
  uint8_t value = 0;
  uint8_t ret = Board_ReadGpio(DP_HSMCI, GPIO_SDDETECT, &value);
  if(ret != BSP_OK || value != 0u)
  {
    return;
  }

  int8_t retries = 10;
  char filename[32];
  
  while(logger_running == 0 && retries > 0)
  {
    sprintf(filename, "log_%08lx.txt", TRNG->TRNG_ODATA);  
    if(FR_OK == f_open(&logFile, filename, FA_READ | FA_OPEN_EXISTING))
    {
      f_close(&logFile);
      --retries;
    }
    else
    {
      if(FR_OK == f_open(&logFile, filename, FA_WRITE | FA_CREATE_ALWAYS))
      {
        logger_running = 1;
        
        /* Unblock the task */
        xTaskNotifyFromISR(hLogTask, 0, eNoAction, 0);
      }
      else
      {
        --retries;
      }
    }
  }
  
  if(logger_running == 0)
  {
    dbgprintf("Failed to open logfile, check SD-Card file system!\r\n");
    Board_SetLed(LED_SD, LED_TRI_R, LED_PRIO_HIGH);
  }
}

void logString(logc component, const char* string)
{
  if(logger_running)
  {
    UINT len = strlen(string);
    UINT wlen = 0;
    volatile UINT ret = 0;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    ret = f_write(&logFile, componentToStr[component], strlen(componentToStr[component]), &wlen);
    ret = f_write(&logFile, string, len, &wlen);
    ret = f_sync(&logFile);
    ret = ret;
    xSemaphoreGive(sdMutex);
  }
}

void logRaw(const uint8_t* data, uint32_t len)
{
//  return;
//  xSemaphoreTake(sdMutex, portMAX_DELAY);
  if(logger_running)
  {
    if(0 == xStreamBufferSend(logBufferHandle, data, len, 0))
    {
      dbgprintf("log buffer full, dropping message\r\n");
    }
  }  
  
  /*
  if(xStreamBufferBytesAvailable(logBufferHandle) >= 4096) {
    extern TaskHandle_t hLogTask;
    xTaskNotify(hLogTask, 0, eNoAction);
  }
  */
  
//  xSemaphoreGive(sdMutex);
}

void logData(logc component, uint8_t busId, uint32_t messageId, uint32_t len, const uint8_t* data)
{
  char line[64];

  int buffsize = sizeof(line);

  int ctr = snprintf(line, buffsize, "\r\n%s %hhu %08lx, %lu ", componentToStr[component], busId, messageId, len);
  if(ctr)
  {
    while(len-- && (buffsize - ctr) > 0)
    {
      int rc = snprintf(line + ctr, buffsize - ctr, "%02hhx", *data++);

      if(rc)
      {
        ctr += rc;
      }
      else
      {
        break;
      }
    }

    if(len == 0xFFFFFFFF)
    {
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      if(0 == xStreamBufferSend(logBufferHandle, line, strlen(line), 0))
      {
        dbgprintf("log buffer full, dropping message\r\n");
      }
      xSemaphoreGive(sdMutex);
    }
  }
}

void init_log_task(void)
{
  sdMutex = xSemaphoreCreateMutexStatic(&sdMutexBuffer);
  //logBufferHandle = xMessageBufferCreateStatic(sizeof(logMessageBuffer), logMessageBuffer, &logStaticBuffer);
  
  logBufferHandle = xStreamBufferCreateStatic(sizeof(logMessageBuffer), sizeof(rxLogBuffer), logMessageBuffer, &logStaticBuffer);
  
  memset(&logFile, 0, sizeof(logFile));
}

uint32_t bytesXfered = 0;

void rtos_log_task(void)
{
  uint32_t startTick = xTaskGetTickCount();
  
  while(1)
  {
    if(logger_running)
    {
      //uint32_t bytesReceived = xMessageBufferReceive(logBufferHandle, rxLogBuffer, sizeof(rxLogBuffer), 250);
      uint32_t bytesReceived = xStreamBufferReceive(logBufferHandle, rxLogBuffer, sizeof(rxLogBuffer), 1000);
      
      if(bytesReceived > 0)
      {
        // Todo: xMessageBuffer will only take small messages and provide them to FatFS.
        // FatFS will collect up to 512 byte and trigger a write.
        // That's very ineffecient, the write queue size should be at least 16kb in size.
        UINT bytesWritten = 0;
        f_write(&logFile, rxLogBuffer, bytesReceived, &bytesWritten);
        //bytesWritten = bytesReceived;

        bytesXfered += bytesWritten;
        uint32_t now = xTaskGetTickCount();
        
        if(now-startTick > 1000) {
          dbgprintf("Transfered %d bytes\r\n", bytesXfered);
          startTick = now;
          bytesXfered = 0;
        }
        
        
        if(bytesWritten != bytesReceived)
        {
          printf("Writing message to SD card failed!");
          asm volatile("bkpt #0");
        }
      }
      else
      {
        f_sync(&logFile);
        dbgprintf("Bytes in stream: %u\r\n", xStreamBufferBytesAvailable(logBufferHandle));
        bytesXfered = 0;
      }
      
      if(stop_logger) {
        f_close(&logFile);
        logger_running = 0;
        stop_logger = 0;
      }          
    }
    else
    {
      /* */
      xTaskNotifyWait(0, 0, 0, portMAX_DELAY);
      if(start_logger) {
        startLogger();
        start_logger = 0;
      }
    }
  }
}

uint32_t get_fattime(void)
{
  uint32_t ul_time;
  
  #ifdef USE_RTC
  uint32_t ul_hour, ul_minute, ul_second;
  uint32_t ul_year, ul_month, ul_day, ul_week;

  /* Retrieve date and time */
  //rtc_get_time(RTC, &ul_hour, &ul_minute, &ul_second);
  //rtc_get_date(RTC, &ul_year, &ul_month, &ul_day, &ul_week);
  #endif
  ul_time = ((2013 - 1980) << 25)
  | (4 << 21)
  | (15 << 16)
  | (12 << 11)
  | (10 << 5)
  | (10 << 0);

  return ul_time;
}
