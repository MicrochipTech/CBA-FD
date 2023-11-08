/*
 * task_CAN.c
 *
 * Created: 19.02.2018 12:59:37
 *  Author: M43734
 */ 

#include <board.h>
#include <config.h>
#include <FreeRTOS.h>
#include <task.h>
#include <returnQueue.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <mcan.h>
#include <assert.h>
#include <pmc.h>

#include "rtos.h"
#include "tasks/task_CAN.h"
#include "tasks/task_CAN_cfg.h"
#include "mcan_config.h"
#include "tc.h"
#include "fatfs/ff.h"
#include "tasks/task_LOG.h"

extern TaskHandle_t hCanTask;
extern FIL logFile;
extern uint8_t logger_running;
static void logCanMessage(aba_can_CanMessage* msg);

#define CAN0_SHIFT_TX 0
#define CAN0_SHIFT_RX 8
#define CAN1_SHIFT_TX 16
#define CAN1_SHIFT_RX 24

#define CAN_ILS1_IRQ (MCAN_ILS_BOL_Msk | MCAN_ILS_PEAL_Msk | MCAN_ILS_PEDL_Msk | MCAN_ILS_TEFLL_Msk)

#define CAN_QUEUE_DEPTH 20
static uint8_t can_ucosFree[CAN_QUEUE_DEPTH * sizeof(struct QueueMsgObject*)];
static struct QueueMsgObject can_msgObjects[CAN_QUEUE_DEPTH];
static StaticQueue_t xQueueCanToUsbFree;
static QueueHandle_t xQueueHandleCanToUsbFree;

COMPILER_SECTION(".data_TCM") volatile uint64_t canRxTimeEnhanced[_aba_can_eCanDevice_ARRAYSIZE][MCAN0_NMBR_RX_FIFO0_ELMTS];
COMPILER_SECTION(".data_TCM") volatile uint64_t canTxTimeEnhanced[_aba_can_eCanDevice_ARRAYSIZE][MCAN0_NMBR_TX_EVT_FIFO_ELMTS];
COMPILER_SECTION(".data_TCM") volatile struct canMsgWaiting_t canMsgs;
uint32_t canTimerCounter = 0;
uint32_t frameCounter = 0;
uint32_t eventCounter = 0;
uint32_t errorLostCounter = 0;
uint32_t errorCounter = 0;
uint32_t init_status[_aba_can_eCanDevice_ARRAYSIZE];

#ifdef DEBUG
uint32_t mcan0isrctr = 0;
uint32_t mcan1isrctr = 0;
uint32_t mcan0isrrx = 0;
uint32_t mcan0isrtx = 0;
uint32_t mcan1isrrx = 0;
uint32_t mcan1isrtx = 0;
uint32_t canTxCtr = 0;
#endif

static const sdCanCfg_t* pSdCanCfg = 0;
//---------------------------------------------------------------------------------------------------------------------

//static void processCanMessages(const MCan_ConfigType* mcfg, uint8_t* rxCnt, uint8_t* txCnt);
static void processCanMessages(const MCan_ConfigType* mcfg, volatile uint8_t* rxCnt, volatile uint8_t* txCnt, uint8_t rxTail, uint8_t evtTail);
static int32_t canReadRegister(aba_can_DbgReadRegister* ac);
static int32_t canPhyControl(const aba_can_CanDeviceWrapper* ac);
static int32_t canGetBitrate(aba_can_eCanDevice device, aba_can_eCanCommands command);
static int32_t canGetMode(aba_can_eCanDevice device, aba_can_eCanCommands command);
/**
 * The handler for decoding the command id and calling the appropriate function
 * @param uco A pointer to the USB command object
 */
int32_t uco_CanHandler(struct USBCMDObject* uco)
{
  int32_t rc = -1;
  pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);

  switch(uco->header.command)
  {
    case aba_can_eCanCommands_SET_BITRATE:
    {
      aba_can_CanSpeed ac;
      if(pb_decode(&istream, aba_can_CanSpeed_fields, &ac)) {
        rc = canSetBitrate(&ac);
        canGetBitrate(ac.device, aba_can_eCanCommands_SET_BITRATE); //backwards compatibility to OptoLyzer
        canGetBitrate(ac.device, aba_can_eCanCommands_GET_BITRATE);
      }
    }
    break;

    case aba_can_eCanCommands_GET_BITRATE:
    {
      aba_can_CanDeviceWrapper ac;
      if(pb_decode(&istream, aba_can_CanDeviceWrapper_fields, &ac)) {
        rc = canGetBitrate(ac.device, uco->header.command);
      }
    }
    break;

    case aba_can_eCanCommands_TX_MSG:
    {
      aba_can_CanMessage ac;
      if(pb_decode(&istream, aba_can_CanMessage_fields, &ac)) {
        //while(rc < 0)
        {
          rc = canSendFrame(&ac);
        }
#ifdef DEBUG
        canTxCtr++;
#endif
      }
    }
    break;

    case aba_can_eCanCommands_SET_MODE:
    {
      aba_can_CanMode ac;
      if(pb_decode(&istream, aba_can_CanMode_fields, &ac)) {
        rc = canSetMode(&ac);
        canGetMode(ac.device, aba_can_eCanCommands_SET_MODE); //backwards compatibility to OptoLyzer
        canGetMode(ac.device, aba_can_eCanCommands_GET_MODE);
      }
    }
    break;
    
    case aba_can_eCanCommands_GET_MODE:
    {
      aba_can_CanDeviceWrapper ac;
      if(pb_decode(&istream, aba_can_CanDeviceWrapper_fields, &ac)) {
        rc = canGetMode(ac.device, uco->header.command);
      }
    }
    break;
    
    case aba_can_eCanCommands_RESET:
    {
      aba_can_CanDeviceWrapper ac;
      if(pb_decode(&istream, aba_can_CanDeviceWrapper_fields, &ac)) {
        rc = canReset(&ac);
      }
    }
    break;

    case aba_can_eCanCommands_DBG_READ_REG:
    {
      aba_can_DbgReadRegister ac;
      if(pb_decode(&istream, aba_can_DbgReadRegister_fields, &ac)) {
        rc = canReadRegister(&ac);
      }
    }
    break;
    
    case aba_can_eCanCommands_PHY_MAINTAIN:
    {
      aba_can_CanDeviceWrapper ac;
      if(pb_decode(&istream, aba_can_CanDeviceWrapper_fields, &ac)) {
        rc = canPhyControl(&ac);
      }
    }
    break;

  }

  return rc;
}

static int32_t canReadRegister(aba_can_DbgReadRegister* ac)
{
  if((ac->addr >= (uint32_t)MCAN0) && ac->addr <= ((uint32_t)MCAN0 + sizeof(Mcan)))
  {
    ac->value = *(uint32_t*)ac->addr;  
  }
  else if((ac->addr >= (uint32_t)MCAN1) && ac->addr <= ((uint32_t)MCAN1 + sizeof(Mcan)))
  {
    ac->value = *(uint32_t*)ac->addr;
  }
  else
  {
    ac->value = 0x55555555;
  }

  struct QueueMsgObject* msg = 0;
  /* Queue Receive may block when all buffers are in use */
  if(pdTRUE == xQueueReceive(xQueueHandleCanToUsbFree, &msg, portMAX_DELAY))
  {
    msg->uco.header.type = UCO_CAN;
    msg->uco.header.command = aba_can_eCanCommands_DBG_READ_REG;
    
    if(queueMessage(msg, aba_can_DbgReadRegister_fields, ac))
    {
      return 0;
    }      
  }

  return 1;
}

int32_t canGetBitrate(aba_can_eCanDevice device, aba_can_eCanCommands command)
{
  if(command != aba_can_eCanCommands_GET_BITRATE && command != aba_can_eCanCommands_SET_BITRATE )
    return 1;

  aba_can_CanSpeed ac;
  
  /* Select MCAN device */
  Mcan* dev = (device == aba_can_eCanDevice_MCAN0) ? MCAN0 : MCAN1;

  ac.nseg1  = 1u + ((dev->MCAN_NBTP & MCAN_NBTP_NTSEG1_Msk) >> MCAN_NBTP_NTSEG1_Pos);
  ac.nseg2  = 1u + ((dev->MCAN_NBTP & MCAN_NBTP_NTSEG2_Msk) >> MCAN_NBTP_NTSEG2_Pos);
  ac.nscale = 1u + ((dev->MCAN_NBTP & MCAN_NBTP_NBRP_Msk)   >> MCAN_NBTP_NBRP_Pos);
  ac.nsjw   = 1u + ((dev->MCAN_NBTP & MCAN_NBTP_NSJW_Msk)   >> MCAN_NBTP_NSJW_Pos);
  
  ac.dseg1  = 1u + ((dev->MCAN_DBTP & MCAN_DBTP_DTSEG1_Msk) >> MCAN_DBTP_DTSEG1_Pos);
  ac.dseg2  = 1u + ((dev->MCAN_DBTP & MCAN_DBTP_DTSEG2_Msk) >> MCAN_DBTP_DTSEG2_Pos);
  ac.dscale = 1u + ((dev->MCAN_DBTP & MCAN_DBTP_DBRP_Msk)   >> MCAN_DBTP_DBRP_Pos);
  ac.dsjw   = 1u + ((dev->MCAN_DBTP & MCAN_DBTP_DSJW_Msk)   >> MCAN_DBTP_DSJW_Pos);
  ac.tdc    = ((dev->MCAN_DBTP & MCAN_DBTP_TDC_Msk)  >> MCAN_DBTP_TDC_Pos);
  ac.tdcf   = ((dev->MCAN_TDCR & MCAN_TDCR_TDCF_Msk) >> MCAN_TDCR_TDCF_Pos);
  ac.tdco   = ((dev->MCAN_TDCR & MCAN_TDCR_TDCO_Msk) >> MCAN_TDCR_TDCO_Pos);

  ac.device = device;  

  struct QueueMsgObject* msg = 0;
  /* Queue Receive may block when all buffers are in use */
  if(pdTRUE == xQueueReceive(xQueueHandleCanToUsbFree, &msg, portMAX_DELAY))
  {
     msg->uco.header.type = UCO_CAN;
     msg->uco.header.command = command;
    
     if(queueMessage(msg, aba_can_CanSpeed_fields, &ac))
     {
       return 0;
     }       
  }
  return 1; 
}

/**
 * Set the CAN controller bitrate
 * @param istream The protobuf stream containing the bitrate information
 * @see aba_can_eCanCommands_SET_BITRATE
 * @see aba_can_CanSpeed_fields
 */
int32_t canSetBitrate(const aba_can_CanSpeed* ac)
{
  /* Select MCAN device */
  Mcan* dev = (ac->device == aba_can_eCanDevice_MCAN0) ? MCAN0 : MCAN1;
  
  dbgprintf("CAN2.0[%u] TSEG1: %u, TSEG2: %u, SJW: %u\r\n", ac->device, ac->nseg1, ac->nseg2, ac->nsjw);
  dbgprintf("CAN-FD[%u] TSEG1: %u, TSEG2: %u, SJW: %u\r\n", ac->device, ac->dseg1, ac->dseg2, ac->dsjw);

  MCAN_EnterInitMode(dev);

  /* Set Nominal Baudrate */
  dev->MCAN_NBTP =
    MCAN_NBTP_NTSEG2(ac->nseg2 - 1u) |
    MCAN_NBTP_NTSEG1(ac->nseg1 - 1u) |
    MCAN_NBTP_NBRP(ac->nscale - 1u) |
    MCAN_NBTP_NSJW(ac->nsjw - 1u);
  
  /* Set Data Baudrate */
  dev->MCAN_DBTP =
    MCAN_DBTP_DTSEG2(ac->dseg2 - 1u) |
    MCAN_DBTP_DTSEG1(ac->dseg1 - 1u) |
    MCAN_DBTP_DBRP(ac->dscale - 1u) |
    MCAN_DBTP_DSJW(ac->dsjw - 1u) |
    (ac->tdc ? MCAN_DBTP_TDC_ENABLED : MCAN_DBTP_TDC_DISABLED);
    
  dev->MCAN_TDCR =
    MCAN_TDCR_TDCO(ac->tdco) |
    MCAN_TDCR_TDCF(ac->tdcf);
  
  /* Enabling CAN will clear bus-off */
  /*
  dev->MCAN_IR = 0xFFFFFFFFu;

  memset(canRxTimeEnhanced, 0xFF, sizeof(canRxTimeEnhanced));
  memset(canTxTimeEnhanced, 0xFF, sizeof(canTxTimeEnhanced));
  MCAN_Enable(dev);
  */

  return 0;
}

/**
 * Send a CAN frame to the bus
 * @param istream The protobuf stream containing the CAN frame information
 * @see aba_can_eCanCommands_TX_MSG
 * @see aba_can_CanMessage
 */
int32_t canSendFrame(aba_can_CanMessage* ac)
{
  uint8_t mode = 0;
  uint16_t retries = (uint16_t)ac->timestamp;
  uint32_t dlc = ac->dlc;
  MCan_IdType type = CAN_STD_ID;

  /* Select MCAN device */
  const MCan_ConfigType* conf = (ac->device == aba_can_eCanDevice_MCAN0) ? &mcan0Config : &mcan1Config;

  if(init_status[ac->device] == STATUS_BUS_OFF) {
    return -1;
  }

  /* Select the correct frame type */
  switch(ac->frameType)
  {
    case aba_can_eCanFrameType_TYPE_FDBRS_XTD:    mode++;
    case aba_can_eCanFrameType_TYPE_FD_XTD:       mode++;
    case aba_can_eCanFrameType_TYPE_CLASSIC_XTD:  type = CAN_EXT_ID;
    break;

    case aba_can_eCanFrameType_TYPE_FDBRS_STD:    mode++;
    case aba_can_eCanFrameType_TYPE_FD_STD:       mode++;
    case aba_can_eCanFrameType_TYPE_CLASSIC_STD:  type = CAN_STD_ID;
    break;
    
    default:
    break;
  }
  
  /* Calculate the DLC bits from the payload length */
  if(dlc > 8u)
  {
    if(dlc <= 12u) {dlc = CAN_DLC_12;}
    else if(dlc <= 16u) {dlc = CAN_DLC_16;}
    else if(dlc <= 20u) {dlc = CAN_DLC_20;}
    else if(dlc <= 24u) {dlc = CAN_DLC_24;}
    else if(dlc <= 32u) {dlc = CAN_DLC_32;}
    else if(dlc <= 48u) {dlc = CAN_DLC_48;}
    else if(dlc <= 64u) {dlc = CAN_DLC_64;}
  }

  /* Check if there is a empty slot in both Tx and Tx Event FIFO */
#if 0  
  while(( 32u - (conf->pMCan->MCAN_TXFQS & MCAN_TXFQS_TFFL_Msk)) >=   // Used slots
        ( 32u - (conf->pMCan->MCAN_TXEFS & MCAN_TXEFS_EFFL_Msk))      // Free slots
  )
#else
  while(( (conf->pMCan->MCAN_TXFQS & MCAN_TXFQS_TFFL_Msk)) <=   // Used slots
        ( (conf->pMCan->MCAN_TXEFS & MCAN_TXEFS_EFFL_Msk))      // Free slots
  )
#endif
  {
    vTaskDelay(1);

    if(retries == 0)
    {
      struct QueueMsgObject* msg = 0; 
      if(pdTRUE == xQueueReceive(xQueueHandleCanToUsbFree, &msg, 0))
      {
        msg->uco.header.type = UCO_CAN;
        msg->uco.header.command = aba_can_eCanCommands_TX_TIMEOUT;
        ac->timestamp = getCurrentTCTime();
        queueMessage(msg, aba_can_CanMessage_fields, ac);
      }    
      
      return -1;
    }
    else
    {
      retries--;
    }
  }

  uint32_t rc = MCAN_AddToTxFifoQ(conf, ac->canId, type, dlc, ac->message.bytes, mode);
  return (rc == 255u) ? -1 : 0;
}

int32_t canGetMode(aba_can_eCanDevice device, aba_can_eCanCommands command)
{
  if(command != aba_can_eCanCommands_GET_MODE && command != aba_can_eCanCommands_SET_MODE)
    return 1;
    
  aba_can_CanMode ac;
  Mcan* dev = (device == aba_can_eCanDevice_MCAN0) ? MCAN0 : MCAN1;
  
  ac.autoRetryEnabled = (dev->MCAN_CCCR & MCAN_CCCR_DAR_Msk) ? 0 : 1;
  ac.device = device;

  if(dev->MCAN_CCCR & MCAN_CCCR_FDOE_Msk)
  {
    if(dev->MCAN_CCCR & MCAN_CCCR_NISO_Msk)
    {
      ac.mode = aba_can_eCanMode_MODE_FDNISO;
    }
    else
    {
      ac.mode = aba_can_eCanMode_MODE_FDISO;
    }
  }
  else
  {
    ac.mode = aba_can_eCanMode_MODE_CLASSIC;
  }
  
  if(dev->MCAN_CCCR & MCAN_CCCR_INIT_Msk)
  {
    ac.specialMode = aba_can_eCanSpecialMode_MODE_INIT;
  }
  else if(dev->MCAN_CCCR & (MCAN_CCCR_TEST_Msk | MCAN_CCCR_MON_Msk))
  {
    ac.specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_INT;
  }
  else if(dev->MCAN_CCCR & MCAN_CCCR_TEST_Msk)
  {
    ac.specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_EXT;
  }
  else if(dev->MCAN_CCCR & MCAN_CCCR_MON_Msk)
  {
    ac.specialMode = aba_can_eCanSpecialMode_MODE_LISTEN;
  }   
  else
  {
    ac.specialMode = aba_can_eCanSpecialMode_MODE_NORMAL; 
  }
  
  struct QueueMsgObject* msg = 0;
  /* Queue Receive may block when all buffers are in use */
  if(pdTRUE == xQueueReceive(xQueueHandleCanToUsbFree, &msg, portMAX_DELAY))
  {
    msg->uco.header.type = UCO_CAN;
    msg->uco.header.command = command;
    
    if(queueMessage(msg, aba_can_CanMode_fields, &ac))
    {
      return 0;
    }
  }
  return 1;
}  

int32_t canSetMode(const aba_can_CanMode* ac)
{
  Mcan* dev = 0;
  if(ac->device == aba_can_eCanDevice_MCAN0)
  {
    dev = MCAN0;
    Board_SetLed(LED_CAN0, LED_TRI_G, LED_PRIO_CLEAR);
    init_status[aba_can_eCanDevice_MCAN0] = STATUS_INIT_DONE;
    canMsgs.msgInst[aba_can_eCanDevice_MCAN0] = 0;
  }
  else
  {
    dev = MCAN1;
    Board_SetLed(LED_CAN1, LED_TRI_G, LED_PRIO_CLEAR);
    init_status[aba_can_eCanDevice_MCAN1] = STATUS_INIT_DONE;
    canMsgs.msgInst[aba_can_eCanDevice_MCAN1] = 0;
  }
  
  MCAN_EnterInitMode(dev);
  if(0 == ac->autoRetryEnabled)
  {
    dev->MCAN_CCCR |= MCAN_CCCR_DAR_Msk;
  }
  else
  {
    dev->MCAN_CCCR &= ~MCAN_CCCR_DAR_Msk;
  }

  switch(ac->mode)
  {
    default:
    case aba_can_eCanMode_MODE_FDISO:
      dev->MCAN_CCCR &= ~(MCAN_CCCR_NISO_Msk);
      dev->MCAN_CCCR |= (MCAN_CCCR_FDOE_Msk | MCAN_CCCR_BRSE_Msk);
      break;
    case aba_can_eCanMode_MODE_FDNISO:
      dev->MCAN_CCCR |= (MCAN_CCCR_FDOE_Msk | MCAN_CCCR_BRSE_Msk | MCAN_CCCR_NISO_Msk);
      break;
    case aba_can_eCanMode_MODE_CLASSIC:
      dev->MCAN_CCCR &= ~(MCAN_CCCR_FDOE_Msk | MCAN_CCCR_BRSE_Msk | MCAN_CCCR_NISO_Msk);
      break;
  }

  switch(ac->specialMode)
  {
    default:
    case aba_can_eCanSpecialMode_MODE_NORMAL:
      dbgprintf("CAN INIT: Normal Mode\r\n");
      MCAN_SetModeNormal(dev);
    break;

    case aba_can_eCanSpecialMode_MODE_LISTEN:
      dbgprintf("CAN INIT: Listen Mode\r\n");
      MCAN_SetModeListenOnly(dev);
    break;

    case aba_can_eCanSpecialMode_MODE_LOOPBACK_INT:
      dbgprintf("CAN INIT: Loopback Mode\r\n");
      MCAN_SetModeLoopbackInt(dev);
    break;

    case aba_can_eCanSpecialMode_MODE_LOOPBACK_EXT:
      dbgprintf("CAN INIT: External Loopback Mode\r\n");
      MCAN_SetModeLoopbackExt(dev);
    break;
  }

  memset((void*)&canRxTimeEnhanced[0][0], 0xFF, sizeof(canRxTimeEnhanced));
  memset((void*)&canTxTimeEnhanced[0][0], 0xFF, sizeof(canTxTimeEnhanced));
  
  dev->MCAN_IR = 0xFFFFFFFFu;
  MCAN_Enable(dev);
  
  return 0;
}

int32_t canReset(const aba_can_CanDeviceWrapper* ac)
{
  Mcan* dev = 0;

  if(ac->device == aba_can_eCanDevice_MCAN0)
  {
    dev = MCAN0;
    Board_SetLed(LED_CAN0, LED_TRI_OFF, LED_PRIO_CLEAR);
    init_status[aba_can_eCanDevice_MCAN0] = STATUS_RESET;
  }
  else
  {
    dev = MCAN1;
    Board_SetLed(LED_CAN1, LED_TRI_OFF, LED_PRIO_CLEAR);
    init_status[aba_can_eCanDevice_MCAN1] = STATUS_RESET;
  }

  MCAN_EnterInitMode(dev);

  return 0;
}

static int32_t canPhyControl(const aba_can_CanDeviceWrapper* ac)
{
  Board_WriteGpio((ac->device == aba_can_eCanDevice_MCAN0) ? DP_CAN0 : DP_CAN1, GPIO_CANSTBY, ac->flag);
 
  return 0;
}

//---------------------------------------------------------------------------------------------------------------------

COMPILER_SECTION(".code_TCM")
static uint8_t MCAN_AdjustFifo0Timing(const MCan_ConfigType* mcfg, uint16_t time_high, uint16_t time_low)
{
  uint32_t element_size = mcfg->rxFifo0ElmtSize & 0x1Fu;      // Get the element size from mcan1Config
  uint32_t* pThisRxBuf = mcfg->msgRam.pRxFifo0;               // Get the Rx buffer address from mcan1Config
  uint8_t fill_level = MCAN_GetFillLevelFifo0(mcfg->pMCan);   // Fill level of Fifo0, the task will only read this number of frames
  uint8_t idx = (MCAN_GetTailFifo0(mcfg->pMCan) + fill_level - 1u) % mcfg->nmbrFifo0Elmts;
  uint16_t rxts_ref = time_low;                                     // Overflow detection
  uint8_t cnt = fill_level;
  
  while(cnt--)                                                      // Loop all messages from newest to oldest
  {
    uint32_t* buf = pThisRxBuf + (idx * element_size);              // Set the buffer address
    buf++;
    uint16_t rxts = (uint16_t)(*buf & 0x0000FFFFu);                 // Get the Rx timestamp 16-bit LSB

    if(rxts > rxts_ref)                                             // If the time value of the older frame is larger than of the last frame a overflow happened
    {
      time_high--;                                                  // Decrease the 16-bit MSB counter
    }
    rxts_ref = rxts;                                                // Assign 16-bit LSB value to reference

    uint64_t tmpTime = (uint64_t)((uint64_t)canTimerCounter << 32) | ((uint32_t)(time_high << 16u)) | rxts;
    canRxTimeEnhanced[mcfg->instanceId][idx] = tmpTime; // Assign 16-bit MSB value to array

    if(idx == 0) {                                                  // Handle wrapping of the Fifo
      idx = mcfg->nmbrFifo0Elmts;
    }
    idx--;
  }

  return fill_level;
}

COMPILER_SECTION(".code_TCM")
static uint8_t MCAN_AdjustTxEventFifoTiming(const MCan_ConfigType* mcfg, uint16_t time_high, uint16_t time_low)
{
  uint32_t* pTxEventBuf = mcfg->msgRam.pTxEvtFifo;
  uint8_t fill_level = MCAN_GetFillLevelTxEventFifo(mcfg->pMCan);
  uint8_t idx = (MCAN_GetTailFifoEvent(mcfg->pMCan) + fill_level - 1u) % mcfg->nmbrTxEvtFifoElmts;
  uint16_t txts_ref = time_low;                                     // Overflow detection
  uint8_t cnt = fill_level;
  
  while(cnt--)                                                      // Loop all messages from newest to oldest
  {
    uint32_t* buf = pTxEventBuf + (idx * (MCAN_TX_EVENT_ELMT_SZ/4u));    // Set the buffer address
    buf++;
    uint16_t txts = (uint16_t)(*buf & 0x0000FFFFu);                 // Get the Rx timestamp 16-bit LSB

    if(txts > txts_ref)                                             // If the time value of the older frame is larger than of the last frame a overflow happened
    {
      time_high--;                                                  // Decrease the 16-bit MSB counter
    }
    txts_ref = txts;                                                // Assign 16-bit LSB value to reference

    uint64_t tmpTime = (uint64_t)((uint64_t)canTimerCounter << 32) | ((uint32_t)(time_high << 16u)) | txts;
    canTxTimeEnhanced[mcfg->instanceId][idx] = tmpTime; // Assign 16-bit MSB value to array

    if(idx == 0) {                                                  // Handle wrapping of the Fifo
      idx = mcfg->nmbrTxEvtFifoElmts;
    }
    idx--;
  }

  return fill_level;
}

COMPILER_SECTION(".code_TCM")
static void processError(const MCan_ConfigType* conf)
{
  struct QueueMsgObject* msg = 0;
  if(pdTRUE == xQueueReceiveFromISR(xQueueHandleCanToUsbFree, &msg, 0))
  {
    assert(msg != 0);
		
    aba_can_CanError cs;
    cs.timestamp = (uint64_t)canTimerCounter << 32 | TC0->TcChannel[1].TC_CV << 16 | TC0->TcChannel[0].TC_CV;
    
    msg->uco.header.type = UCO_CAN;
    msg->uco.header.command = aba_can_eCanCommands_GET_ERRORCNT;
    
    uint32_t psr = conf->pMCan->MCAN_PSR;
    cs.busOff = (psr & MCAN_PSR_BO_Msk) >> MCAN_PSR_BO_Pos;
    cs.lec = (psr & (MCAN_PSR_LEC_Msk | MCAN_PSR_DLEC_Msk));
    cs.tec = (conf->pMCan->MCAN_ECR & MCAN_ECR_TEC_Msk) >> MCAN_ECR_TEC_Pos;
    cs.device = conf->instanceId == 0 ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;

    if(conf->pMCan->MCAN_ECR & MCAN_ECR_RP_Msk)
    {
      cs.rec = 128u;
    }      
    else
    {
      cs.rec = (conf->pMCan->MCAN_ECR & MCAN_ECR_REC_Msk) >> MCAN_ECR_REC_Pos;
    }      
    
    errorCounter++;
    queueMessageFromISR(msg, aba_can_CanError_fields, &cs);
  }
  else
  {
    errorLostCounter++;
  }
}

COMPILER_SECTION(".code_TCM")
void MCAN0_Handler(void)
{
  uint16_t time_low = TC0->TcChannel[0].TC_CV & UINT16_MAX;     // Current time 16-bit LSB
  uint16_t time_high = TC0->TcChannel[1].TC_CV & UINT16_MAX;    // Current time 16-bit MSB
  uint32_t ir = MCAN0->MCAN_IR;
  MCAN0->MCAN_IE &= ~(MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk);
  MCAN0->MCAN_IR = MCAN_IR_RF0N_Msk | MCAN_IR_TEFN_Msk;

  if(ir & MCAN_IR_RF0N_Msk)
  {
    canMsgs.msgQue[MCAN0RX] = MCAN_AdjustFifo0Timing(&mcan0Config, time_high, time_low);
#ifdef DEBUG
    mcan0isrrx += canMsgs.msgQue[MCAN0RX];
#endif
  }
  
  if(ir & MCAN_IR_TEFN_Msk)
  {
    canMsgs.msgQue[MCAN0TX] = MCAN_AdjustTxEventFifoTiming(&mcan0Config, time_high, time_low);
#ifdef DEBUG
    mcan0isrtx += canMsgs.msgQue[MCAN0TX];
#endif
  }  

#ifdef DEBUG
  mcan0isrctr++;
#endif

  /* Notify task about new messages */
  if(canMsgs.msgInst[0]) {
    xTaskNotifyFromISR(hCanTask, canMsgs.msgTotal, eSetValueWithOverwrite, NULL);
  }
  else {
    /* If an element is queued into the FIFO after an interrupt has been raised (means this ISR is executed)
       and the fifo processor gets all elements + the recently queued element, there will be another IRQ
       where the fifo processor will read back 0 elements (as it already has been processed)!
       Clearing the IR bit after reading all elements would avoid this scenario, however if an element is enqueued
       between processing the last element and clearing the IR bit, this element will stay in the FIFO and won't generate
       an IRQ. It will be stuck until the next message is queued into the FIFO, creating another IRQ.
    */
    MCAN0->MCAN_IE |= (MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk);
  }
}

volatile uint8_t lastMsgCnt = 0;

COMPILER_SECTION(".code_TCM")
void MCAN1_Handler(void)
{
  uint16_t time_low = TC0->TcChannel[0].TC_CV & UINT16_MAX;     // Current time 16-bit LSB
  uint16_t time_high = TC0->TcChannel[1].TC_CV & UINT16_MAX;    // Current time 16-bit MSB
  uint32_t ir = MCAN1->MCAN_IR;
  MCAN1->MCAN_IE &= ~(MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk);
  MCAN1->MCAN_IR = MCAN_IR_RF0N_Msk | MCAN_IR_TEFN_Msk;

  if(ir & MCAN_IR_RF0N_Msk)
  {
    canMsgs.msgQue[MCAN1RX] = MCAN_AdjustFifo0Timing(&mcan1Config, time_high, time_low);
#ifdef DEBUG
    mcan1isrrx += canMsgs.msgQue[MCAN1RX];
#endif
  }

  if(ir & MCAN_IR_TEFN_Msk)
  {
    canMsgs.msgQue[MCAN1TX] = MCAN_AdjustTxEventFifoTiming(&mcan1Config, time_high, time_low);
#ifdef DEBUG
    mcan1isrtx += canMsgs.msgQue[MCAN1TX];
#endif
  } 
  
#ifdef DEBUG
  mcan1isrctr++;
#endif

  /* Notify task about new messages */
  if(canMsgs.msgInst[1]) {
    xTaskNotifyFromISR(hCanTask, canMsgs.msgTotal, eSetValueWithOverwrite, NULL);
  }
  else {
    /* If an element is queued into the FIFO after an interrupt has been raised (means this ISR is executed)
    and the fifo processor gets all elements + the recently queued element, there will be another IRQ
    where the fifo processor will read back 0 elements (as it already has been processed)!
    Clearing the IR bit after reading all elements would avoid this scenario, however if an element is enqueued
    between processing the last element and clearing the IR bit, this element will stay in the FIFO and won't generate
    an IRQ. It will be stuck until the next message is queued into the FIFO, creating another IRQ.
    */
    MCAN1->MCAN_IE |= (MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk);
  }
}

/**
 * MCAN0 secondary interrupt service routine
 */
COMPILER_SECTION(".code_TCM")
void MCAN0_Line1_Handler(void)
{
  MCAN0->MCAN_IE &= ~CAN_ILS1_IRQ;
  Board_SetLed(LED_CAN0, LED_TRI_R, LED_PRIO_HIGH);
  
  if(MCAN0->MCAN_IR & MCAN_IR_BO_Msk)
  {
    init_status[aba_can_eCanDevice_MCAN0] = STATUS_BUS_OFF;
  }
  else
  {
    init_status[aba_can_eCanDevice_MCAN0] = STATUS_MINOR_ERROR;
  }
  
  if(MCAN0->MCAN_IR & MCAN_IR_TEFL_Msk)
  {
    asm volatile ("bkpt #0");
  }

  processError(&mcan0Config);
  MCAN0->MCAN_IR = CAN_ILS1_IRQ;
  MCAN0->MCAN_IE |= CAN_ILS1_IRQ;
}

/**
 * MCAN1 secondary interrupt service routine
 */
COMPILER_SECTION(".code_TCM")
void MCAN1_Line1_Handler(void)
{
  MCAN1->MCAN_IE &= ~CAN_ILS1_IRQ;
  Board_SetLed(LED_CAN1, LED_TRI_R, LED_PRIO_HIGH);
  
  if(MCAN1->MCAN_IR & MCAN_IR_BO_Msk)
  {
    init_status[aba_can_eCanDevice_MCAN1] = STATUS_BUS_OFF;
  }
  else
  {
    init_status[aba_can_eCanDevice_MCAN1] = STATUS_MINOR_ERROR;
  }
  
  if(MCAN1->MCAN_IR & MCAN_IR_TEFL_Msk)
  {
    asm volatile ("bkpt #0");
    MCAN1->MCAN_IR = MCAN_IR_TEFL_Msk;
  }
  
  processError(&mcan1Config);
  MCAN1->MCAN_IR = CAN_ILS1_IRQ;
  MCAN1->MCAN_IE |= CAN_ILS1_IRQ;
}

/**
 * FreeRTOS initialization task for CAN. Initialize all FreeRTOS structures within the init task,
 * before the actual CAN task is started.
 */
void init_can_task(const uintptr_t* sdCfg)
{
  if(sdCfg) {
    pSdCanCfg = (sdCanCfg_t*)sdCfg;
  }

  /* Initialize all FreeRTOS objects before they are used */
  xQueueHandleCanToUsbFree = xQueueCreateStatic(CAN_QUEUE_DEPTH, sizeof(struct QueueMsgObject*), can_ucosFree, &xQueueCanToUsbFree);
 
  for(uint8_t i=0; i<CAN_QUEUE_DEPTH; i++)
  {
    struct QueueMsgObject* temp = &can_msgObjects[i];
    temp->returnQueue = xQueueHandleCanToUsbFree;
    xQueueSendToBack(xQueueHandleCanToUsbFree, &temp, 0);
  }

  memset(init_status, 0, sizeof(init_status));
  canMsgs.msgTotal = 0;

  /* Enable TC0_CH1 and TC0_CH1 with timer chaining */  
  PMC_ConfigurePCKn(6u, PMC_PCK_CSS_MAIN_CLK, 11u);
  PMC_EnablePeripheral(ID_TC0);
  PMC_EnablePeripheral(ID_TC1);
  canTimerCounter = 0;

  TC_DisableClock(TC0, TC_CH0);
  TC_DisableClock(TC0, TC_CH1);

  TC0->TcChannel[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_ASWTRG_SET;
  TC0->TcChannel[0].TC_RA = 0x7FFF;
  TC0->TcChannel[0].TC_RC = 0xFFFF;
  TC0->TcChannel[1].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_XC1;
  TC0->TcChannel[1].TC_IER = TC_IER_COVFS_Msk;
  TC0->TC_BMR = TC_BMR_TC1XC1S_TIOA0;

  dummyRead32(TC0->TcChannel[1].TC_SR);
  
  NVIC_ClearPendingIRQ(TC1_IRQn);
  NVIC_EnableIRQ(TC1_IRQn);
  
  TC_EnableClock(TC0, TC_CH0);
  TC_EnableClock(TC0, TC_CH1);
  TC_SyncStart(TC0);
}

void TC1_Handler(void)
{
  volatile uint32_t status = TC0->TcChannel[1].TC_SR;

  if(status & TC_SR_COVFS_Msk)
  {
    // Happens once every 71 min
    canTimerCounter++;
  }
}

uint64_t getCurrentTCTime(void)
{
  /**
    In addition to barriers, the CPSID instruction is also self synchronizing within the instruction stream,
    and ensures that the effect of setting PRIMASK and/or FAULTMASK is visible on the following instruction,
    thus ensuring that the appropriate interrupt/exceptions are disabled
  **/
  portDISABLE_INTERRUPTS();
  uint64_t ret = (uint64_t)canTimerCounter << 32 | TC0->TcChannel[1].TC_CV << 16 | TC0->TcChannel[0].TC_CV;
  portENABLE_INTERRUPTS();
  return ret;
}

__STATIC_INLINE void flagToFrametype(Mailbox64Type* mailBox, aba_can_CanMessage* cs)
{
  if(mailBox->info.flags & CAN_FLAG_EXTENDED)
  {
    if(mailBox->info.flags & CAN_FLAG_BRS) {
      cs->frameType = aba_can_eCanFrameType_TYPE_FDBRS_XTD;
    }
    else if(mailBox->info.flags & CAN_FLAG_FD) {
      cs->frameType = aba_can_eCanFrameType_TYPE_FD_XTD;
    }
    else {
      cs->frameType = aba_can_eCanFrameType_TYPE_CLASSIC_XTD;
    }
  }
  else
  {
    if(mailBox->info.flags & CAN_FLAG_BRS) {
      cs->frameType = aba_can_eCanFrameType_TYPE_FDBRS_STD;
    }
    else if(mailBox->info.flags & CAN_FLAG_FD) {
      cs->frameType = aba_can_eCanFrameType_TYPE_FD_STD;
    }
    else {
      cs->frameType = aba_can_eCanFrameType_TYPE_CLASSIC_STD;
    }
  }
}

/**
 * FreeRTOS CAN receive task
 */
void rtos_can_task()
{
  struct canMsgWaiting_t msgCnt;
  
  /* CAN basic initialization */
  MCAN_Init(&mcan1Config);
  //MCAN_InitFdEnable(&mcan1Config);
  //MCAN_InitFdBitRateSwitchEnable(&mcan1Config);
  MCAN1->MCAN_TXEFC |= MCAN_TXEFC_EFWM(16);

  MCAN_ConfigRxFifoFilter(&mcan1Config, CAN_FIFO_0, 0u, 0u, CAN_STD_ID, CAN_11_BIT_ID_MASK);
  MCAN_ConfigRxFifoFilter(&mcan1Config, CAN_FIFO_0, 1u, 0u, CAN_EXT_ID, CAN_29_BIT_ID_MASK);

  mcan1Config.pMCan->MCAN_ILE = MCAN_ILE_EINT0_Msk | MCAN_ILE_EINT1_Msk;
  mcan1Config.pMCan->MCAN_ILS = MCAN_ILS_EPL_Msk | MCAN_ILS_EWL_Msk | MCAN_ILS_BOL_Msk | MCAN_ILS_PEAL_Msk | MCAN_ILS_PEDL_Msk | MCAN_ILS_TEFLL_Msk;
  mcan1Config.pMCan->MCAN_IE = MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk | CAN_ILS1_IRQ;
  mcan1Config.pMCan->MCAN_TSCC = MCAN_TSCC_TSS_EXT_TIMESTAMP;

  MCAN_Init(&mcan0Config);
  //MCAN_InitFdEnable(&mcan0Config);
  //MCAN_InitFdBitRateSwitchEnable(&mcan0Config);

  MCAN_ConfigRxFifoFilter(&mcan0Config, CAN_FIFO_0, 0u, 0u, CAN_STD_ID, CAN_11_BIT_ID_MASK);
  MCAN_ConfigRxFifoFilter(&mcan0Config, CAN_FIFO_0, 1u, 0u, CAN_EXT_ID, CAN_29_BIT_ID_MASK);

  mcan0Config.pMCan->MCAN_ILE = MCAN_ILE_EINT0_Msk | MCAN_ILE_EINT1_Msk;
  mcan0Config.pMCan->MCAN_ILS = MCAN_ILS_EPL_Msk | MCAN_ILS_EWL_Msk | MCAN_ILS_BOL_Msk | MCAN_ILS_PEAL_Msk | MCAN_ILS_PEDL_Msk | MCAN_ILS_TEFLL_Msk;
  mcan0Config.pMCan->MCAN_IE = MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk | CAN_ILS1_IRQ;
  mcan0Config.pMCan->MCAN_TSCC = MCAN_TSCC_TSS_EXT_TIMESTAMP;
  
  if(pSdCanCfg) {
    canSetBitrate(&pSdCanCfg->sdInitCanSpeed[0]);
    canSetMode(&pSdCanCfg->sdInitCanMode[0]);
    canSetBitrate(&pSdCanCfg->sdInitCanSpeed[1]);
    canSetMode(&pSdCanCfg->sdInitCanMode[1]);
  }

  /*
   * CAN has not started yet! And remains in init mode until speed has been set
   */
  while(1)
  {
    /* Wait for a signal from the ISR */
    BaseType_t ret = xTaskNotifyWait(0, UINT32_MAX, &msgCnt.msgTotal, 500);
    uint8_t enableMcan0Int = 0;
    uint8_t enableMcan1Int = 0;
    
    /* Reset CAN status LEDs */
    if(ret == pdFALSE)
    {
      if(init_status[aba_can_eCanDevice_MCAN0] == STATUS_INIT_DONE) {
        Board_SetLed(LED_CAN0, LED_TRI_B, LED_PRIO_CLEAR);
      }
      
      if(init_status[aba_can_eCanDevice_MCAN1] == STATUS_INIT_DONE) {
        Board_SetLed(LED_CAN1, LED_TRI_B, LED_PRIO_CLEAR);
      }
      
      if(init_status[aba_can_eCanDevice_MCAN0] == STATUS_MINOR_ERROR) {
        init_status[aba_can_eCanDevice_MCAN0] = STATUS_INIT_DONE;
      }

      if(init_status[aba_can_eCanDevice_MCAN1] == STATUS_MINOR_ERROR) {
        init_status[aba_can_eCanDevice_MCAN1] = STATUS_INIT_DONE;
      }
      
#ifdef DEBUG
      /* It shall never happen that the service routine is notified with 0 messages waiting */
      if(canMsgs.msgTotal) {
        asm volatile ("bkpt #0");
      }
#endif      
      continue;
    }
  
    while(canMsgs.msgTotal)
    {
      /* Set CAN status LEDs if messages are pending */
      if(canMsgs.msgInst[0]) {
        Board_SetLed(LED_CAN0, LED_TRI_G, LED_PRIO_CLEAR);
        enableMcan0Int = 1;
      }
      
      if(canMsgs.msgInst[1]) {
        Board_SetLed(LED_CAN1, LED_TRI_G, LED_PRIO_CLEAR);
        enableMcan1Int = 1;
      }
      
      uint8_t mcan0RxTail = MCAN_GetTailFifo0(mcan0Config.pMCan);
      uint8_t mcan0TxTail = MCAN_GetTailFifoEvent(mcan0Config.pMCan);
      uint8_t mcan1RxTail = MCAN_GetTailFifo0(mcan1Config.pMCan);
      uint8_t mcan1TxTail = MCAN_GetTailFifoEvent(mcan1Config.pMCan);
      
      uint64_t mcan0Rx = canRxTimeEnhanced[0][mcan0RxTail];
      uint64_t mcan0Tx = canTxTimeEnhanced[0][mcan0TxTail];
      uint64_t mcan1Rx = canRxTimeEnhanced[1][mcan1RxTail];
      uint64_t mcan1Tx = canTxTimeEnhanced[1][mcan1TxTail];

      if(canMsgs.msgInst[0] && canMsgs.msgInst[1])
      {
        if( (canMsgs.msgQue[MCAN0RX] && mcan0Rx < min(mcan1Rx, mcan1Tx)) || (canMsgs.msgQue[MCAN0TX] && mcan0Tx < min(mcan1Rx, mcan1Tx)) )
        {
          processCanMessages(&mcan0Config, &canMsgs.msgQue[MCAN0RX], &canMsgs.msgQue[MCAN0TX], mcan0RxTail, mcan0TxTail);
        }
        else
        {
          processCanMessages(&mcan1Config, &canMsgs.msgQue[MCAN1RX], &canMsgs.msgQue[MCAN1TX], mcan1RxTail, mcan1TxTail);
        }
      }
      else if(canMsgs.msgInst[0])
      {
        processCanMessages(&mcan0Config, &canMsgs.msgQue[MCAN0RX], &canMsgs.msgQue[MCAN0TX], mcan0RxTail, mcan0TxTail);
      }
      else if(canMsgs.msgInst[1])
      {
        processCanMessages(&mcan1Config, &canMsgs.msgQue[MCAN1RX], &canMsgs.msgQue[MCAN1TX], mcan1RxTail, mcan1TxTail);
      }
    }
    
    /* Interrupts have been disabled in the ISR, so re-enable them */
    if(enableMcan0Int || canMsgs.msgInst[0]  == 0) {
      mcan0Config.pMCan->MCAN_IE |= MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk;
    }
    
    if(enableMcan1Int || canMsgs.msgInst[1] == 0) {
      mcan1Config.pMCan->MCAN_IE |= MCAN_IE_RF0NE_Msk | MCAN_IE_TEFNE_Msk;
    }
    
#ifdef DEBUG
    /* At least one of the interrupts must have been enabled after processing is done */
    if(!enableMcan0Int && !enableMcan1Int) {
      //asm volatile ("bkpt #0");
    }
#endif
  }
}

static void processCanMessages(const MCan_ConfigType* mcfg, volatile uint8_t* rxCnt, volatile uint8_t* txCnt, uint8_t rxTail, uint8_t evtTail)
{
  int fill_level = 0;
  aba_can_CanMessage cs;
  struct MailBox64Tag mailbox;
  
  struct QueueMsgObject* msg = 0;
  /* Queue Receive may block when all buffers are in use */
  while(pdFALSE == xQueueReceive(xQueueHandleCanToUsbFree, &msg, portMAX_DELAY))
  {
    vTaskDelay(1); // xQueueReceive shall block, so this should never be executed
  }
  msg->uco.header.type = UCO_CAN;
  
  if(*rxCnt > 0 && (canRxTimeEnhanced[mcfg->instanceId][rxTail] < canTxTimeEnhanced[mcfg->instanceId][evtTail] || *txCnt == 0))
  {
    // Rx first
    msg->uco.header.command = aba_can_eCanCommands_RX_MSG;
    cs.timestamp = canRxTimeEnhanced[mcfg->instanceId][rxTail];                                   // Timestamps are concatenated
    fill_level = MCAN_GetRxFifoBuffer(mcfg, CAN_FIFO_0, &mailbox);
    assert(fill_level != 0);
    canRxTimeEnhanced[mcfg->instanceId][rxTail] = UINT64_MAX;

    (*rxCnt)--;
    frameCounter++;
  }
  else if(*txCnt != 0)
  {
    // Tx First
    msg->uco.header.command = aba_can_eCanCommands_TX_MSG;
    cs.timestamp = canTxTimeEnhanced[mcfg->instanceId][evtTail];                                   // Timestamps are concatenated
    fill_level = MCAN_GetTxEventFifoBuffer(mcfg, &mailbox);
    assert(fill_level != 0);
    canTxTimeEnhanced[mcfg->instanceId][evtTail] = UINT64_MAX;
    
    (*txCnt)--;
    eventCounter++;
  }
  else
  {
    /* It shall not be possible to call processCanMessages with both rxCnt and txCnt being 0 */
    asm volatile ("bkpt #0");
  }
  
  cs.device = mcfg->instanceId == 0 ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;
  cs.canId = mailbox.info.id;
  cs.dlc = mailbox.info.length;                                            // DLC
  cs.message.size = cs.dlc;                                                   // Message size is the length of the message in byte
  memcpy(cs.message.bytes, mailbox.data, cs.message.size);
  flagToFrametype(&mailbox, &cs);
  
  /* Queue message should never block */
  queueMessage(msg, aba_can_CanMessage_fields, &cs);
  logCanMessage(&cs);

  //taskYIELD();
}

//uint32_t msglog = 0;

static void logCanMessage(aba_can_CanMessage* msg)
{
  
  //logData(CAN, msg->device, msg->canId, msg->dlc, &msg->message);
  logRaw((const uint8_t*)msg, sizeof(*msg));
  /*
  if(logger_running) {
    if(msglog < 10000) {
      UINT bw = 0;
      f_write(&logFile, msg, sizeof(*msg), &bw);
      msglog++;
    }
    else
    {
      f_close(&logFile);
      asm volatile("bkpt #0");
    }      
  }
  */
}  
