/*
 * ABAControl.c
 *
 * Created: 23.02.2018 07:59:06
 *  Author: M43734
 */ 
 
#include <board.h>
#include <config.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <USBD.h>
#include "Bldr_Protocol.h"
#include "ABAControl.h"
#include "aba.api.pb.h"
#include "returnQueue.h"
#include "tasks/task_CAN.h"
#include "versions.h"

typedef void(*tAppCall)(void);

#define CTRL_QUEUE_DEPTH (4u)
static uint8_t ctrl_ucosFree[CTRL_QUEUE_DEPTH * sizeof(struct QueueMsgObject*)];
static struct QueueMsgObject ctrl_msgObjects[CTRL_QUEUE_DEPTH];
static StaticQueue_t xQueueCtrlToUsbFree;
static QueueHandle_t xQueueHandleCtrlToUsbFree;

void init_aba_api(void)
{
  /* Initialize all FreeRTOS objects before they are used */
  xQueueHandleCtrlToUsbFree = xQueueCreateStatic(CTRL_QUEUE_DEPTH, sizeof(struct QueueMsgObject*), ctrl_ucosFree, &xQueueCtrlToUsbFree);
  
  for(uint8_t i=0; i<CTRL_QUEUE_DEPTH; i++)
  {
    struct QueueMsgObject* temp = &ctrl_msgObjects[i];
    temp->returnQueue = xQueueHandleCtrlToUsbFree;
    xQueueSendToBack(xQueueHandleCtrlToUsbFree, &temp, 0);
  }
}

static void returnTimestamp(void)
{
  struct QueueMsgObject* msg = 0;  
  while(pdFALSE == xQueueReceive(xQueueHandleCtrlToUsbFree, &msg, portMAX_DELAY))
  {
    vTaskDelay(1); // xQueueReceive shall block, so this should never be executed
  }
  msg->uco.header.type = UCO_API;
  msg->uco.header.command = aba_api_eCommand_CMD_TIMESTAMP;
  msg->uco.header.payloadlength = 0;
  
  aba_api_Timestamp ts;
  ts.timestamp = getCurrentTCTime();

  /* Queue message should never block */
  queueMessage(msg, aba_api_Timestamp_fields, &ts);
}

static void returnVersion(void)
{
  struct QueueMsgObject* msg = 0;  
  while(pdFALSE == xQueueReceive(xQueueHandleCtrlToUsbFree, &msg, portMAX_DELAY))
  {
    vTaskDelay(1); // xQueueReceive shall block, so this should never be executed
  }
  msg->uco.header.type = UCO_API;
  msg->uco.header.command = aba_api_eCommand_CMD_GETVERSION;
  msg->uco.header.payloadlength = 0;

  extern BOARD_ID board;

  aba_api_Versions vs;
  vs.api = 	aba_api_eVersion_VER_API;
  vs.bootloader = ((struct bRamLayout*)BRAM_ADDR)->bootloader_ver;
  vs.can = aba_api_eVersion_VER_CAN;
  vs.firmware = ABA_FIRMWARE_VERSION;
  vs.hardware = board;
  vs.hash = GIT_SHORT_HASH;

  /* Queue message should never block */
  queueMessage(msg, aba_api_Versions_fields, &vs);
}

static void twinkle()
{
  extern uint32_t g_twinkle;
  if(g_twinkle == 0) {
    g_twinkle = 20;
  } 
}

int32_t uco_ApiHandler(struct USBCMDObject* uco)
{
  // pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
  struct bRamLayout* BRAM = (struct bRamLayout*)BRAM_ADDR;

  switch(uco->header.command)
  {
    case aba_api_eCommand_CMD_SAMBA:
      BRAM->bootloader_req = SAMBA_REQUEST;
      USBD_Disconnect();
      NVIC_SystemReset();
    break;
    
    case aba_api_eCommand_CMD_BOOTLOADER:
      BRAM->bootloader_req = BOOTLOADER_REQUEST;
      USBD_Disconnect();
      NVIC_SystemReset();
    break;
    
    case aba_api_eCommand_CMD_RESET:
      USBD_Disconnect();
      NVIC_SystemReset();
    break;

    case aba_api_eCommand_CMD_GETVERSION:
      returnVersion();
    break;

    case aba_api_eCommand_CMD_TIMESTAMP:
      returnTimestamp();
    break;
    
    case aba_api_eCommand_CMD_TWINKLE:
      twinkle();
    break;
  }

  return 0;
}
