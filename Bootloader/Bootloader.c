/*
 * Bootloader.c
 *
 * Created: 10.10.2017 09:18:06
 *  Author: M43734
 */ 

#include "sam.h"
#include "Bootloader.h"
#include "efc.h"
#include "CRC.h"
#include "mpu.h"
#include "flashd.h"
#include "timetick.h"
#include "config.h"
#include <string.h>

#include <USBDescriptors.h>
#include <USBRequests.h>
#include <USBD.h>
#include <USBDDriver.h>

#define DATABUFFERSIZE                  1024
#define ABADriverDescriptors_BULKOUT    1
#define ABADriverDescriptors_BULKIN     2

/** USB standard device driver. */
USBDDriver usbdDriver;
extern const USBDDriverDescriptors ABADriverDescriptors;
COMPILER_ALIGNED(32) static uint8_t gRxBuffer[DATABUFFERSIZE];
COMPILER_ALIGNED(32) static uint8_t gTxBuffer[DATABUFFERSIZE];

uint8_t writeBuffer[IFLASH_PAGE_SIZE];

static uint8_t FlashSectorErase(uint32_t StartAddr, uint32_t EndAddr);
static void _UsbDataSent(void *unused, uint8_t status, uint32_t transferred, uint32_t remaining);

static uint32_t flashEraseProgress = 0;
static uint32_t eraseStartAddr = 0;
static uint32_t eraseEndAddr = 0;
volatile uint8_t eraseFlash = 0;

WEAK void bootloaderStatusIndicator(BSTATUS status)
{
}

void SysTick_Handler(void)
{
  TimeTick_Tick();
}

void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
  USBDDriver_RequestHandler(&usbdDriver, request);
}

#if 0
static uint8_t isBufferEmpty(const uint8_t* buffer, uint32_t len)
{
  uint8_t ret = 1u;
  
  for(uint32_t i=0; i<len; i++)
  {
    if(buffer[i] != 0xFFU) {
      ret = 0;
      break;
    }
  }
  
  return ret;
}
#endif

/* Todo: Currently the state-machine is running in the ISR context. It should run as task to allow asynchronous operations */
static void bootloaderSM(struct sbAction* action, uint32_t bytesReceived)
{
  /* Always initialize the reply packet */
  struct sbReply* reply = (struct sbReply*)(gTxBuffer);
  reply->magic = 0x52E54C59U;

  switch(action->action)
  {
    case QUERYVERSION:
      reply->action = QUERYVERSION;
      reply->reply = QUERYVERSION;
      reply->reserved = BOOTLDR_VERSION_1;
      USBD_Write(ABADriverDescriptors_BULKIN, gTxBuffer, sizeof(struct sbReply), (TransferCallback)_UsbDataSent, 0);
    break;

    /* Save the starting address of the flash erase event */
    case ERASESTART:
      if(eraseFlash == 0)
      {
        flashEraseProgress = 0;
        eraseStartAddr = action->address;
        dbgprintf("Received erase start address: %lx\r\n", action->address);
      }
      else
      {
        dbgprintf("Erase still in progress\r\n");
      }
    break;
    
    /* Save the ending address of the flash erase event and trigger the erase action */
    case ERASEFLASH:
      if(eraseFlash == 0)
      {
        reply->action = ERASESTATUS;
        reply->reply = ERASEFLASH;
        reply->reserved = 0;
        USBD_Write(ABADriverDescriptors_BULKIN, gTxBuffer, sizeof(struct sbReply), (TransferCallback)_UsbDataSent, 0);
        
        dbgprintf("Received erase end address: %lx\r\n", action->address);
        eraseEndAddr = action->address;
        eraseFlash = 1;
      }
      else
      {
        dbgprintf("Erase still in progress\r\n");
      }
    break;

    /* Reply the current erase percentage */
    case ERASESTATUS:
      reply->action = ERASESTATUS;
      reply->reserved = flashEraseProgress;
      reply->reply = ERROR_OK;
      USBD_Write(ABADriverDescriptors_BULKIN, gTxBuffer, sizeof(struct sbReply), (TransferCallback)_UsbDataSent, 0);
    break;

    /* Receive the first 256 bytes of a sector to write */
    case SENDDATA:
      memcpy(&writeBuffer[0], &action->data[0], IFLASH_PAGE_SIZE / 2);
      dbgprintf("Received 0x%lx byte\r\n", bytesReceived);
    break;

    /* Receive the second 256 bytes of the sector */
    case WRITEDATA:
      memcpy(&writeBuffer[IFLASH_PAGE_SIZE / 2], &action->data[0], IFLASH_PAGE_SIZE / 2);
      dbgprintf("Received data for address 0x%lx\r\n", action->address);
      
      /* USB already does a CRC16, however as our buffer is split, make sure it has the correct content! */
      if(action->checksum != crc16(writeBuffer, sizeof(writeBuffer)))
      {
        dbgprintf("Checksum error!\r\n");
        reply->action = ERROR_CHECKSUM;
      }
      else
      {
        //if(isBufferEmpty(writeBuffer, sizeof(writeBuffer)) != 0)
        {
          __disable_irq();
          FLASHD_Unlock(action->address, action->address + IFLASH_PAGE_SIZE, 0, 0);
          FLASHD_Write( action->address, writeBuffer,      IFLASH_PAGE_SIZE);
          FLASHD_Lock(  action->address, action->address + IFLASH_PAGE_SIZE, 0, 0 );
          __enable_irq();
        }        
        reply->action = ERROR_OK;
        bootloaderStatusIndicator(BS_FLASHING);
      }

      reply->magic    = 0x52E54C59U;
      reply->reply    = action->action;
      reply->reserved = 0;
      
      USBD_Write(ABADriverDescriptors_BULKIN, gTxBuffer, sizeof(struct sbReply), 0, 0);
    break;

    case TWINKLE:
      bootloaderStatusIndicator(BS_TWINKLE);
    break;

    /* System reset */
    case RESET:
      dbgprintf("Performing system reset\r\n");
      NVIC_SystemReset();
    default:
    break;
  }
}

static void UsbDataReceived(void* user_data, uint8_t status, uint32_t received, uint32_t remaining)
{
  if(status == USBD_STATUS_SUCCESS)
  {
    dbgprintf("Received 0x%lx byte, 0x%lx byte remaining\r\n", received, remaining);

    if(received == sizeof(struct sbAction))
    {
      struct sbAction* action = (struct sbAction*)(gRxBuffer);
      if(action->magic == MAGIC_ACTION)
      {  
        /* Todo: Save the packet, call state-machine and USBD_Read from main loop */
        bootloaderSM(action, received);
      }
    }
  }
  else {
     dbgprintf("_UsbDataReceived error\r\n");
  }

  // Read next data packet
  USBD_Read(ABADriverDescriptors_BULKOUT, gRxBuffer, sizeof(gRxBuffer), UsbDataReceived, 0);
}

static void _UsbDataSent(void *unused, uint8_t status, uint32_t transferred, uint32_t remaining)
{
  dbgprintf("Transfered 0x%lx byte, 0x%lx byte remaining\r\n", transferred, remaining);
}

static uint8_t FlashSectorErase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t ucError = 0;

  StartAddr -= StartAddr % IFLASH_SECTOR_SIZE;
  if(EndAddr - StartAddr < IFLASH_SECTOR_SIZE) {
    EndAddr = StartAddr + IFLASH_SECTOR_SIZE;
  }

  uint32_t StartAddrTemp = StartAddr;

  /* Unlock the region */
  __disable_irq();
  ucError = FLASHD_Unlock(StartAddr, EndAddr, 0, 0);
  __enable_irq();
  if( ucError ) {
    dbgprintf("FLASHD_Unlock fail: %hhu\r\n", ucError);
    return ucError;
  }

  uint32_t length = EndAddr - StartAddr;
  if(length < IFLASH_SECTOR_SIZE) {
    length = IFLASH_SECTOR_SIZE;
  }
  
  float fpercent = 100.f / (length / IFLASH_SECTOR_SIZE);
  float percent = 0.f;

  dbgprintf("Erasing\r\n");

  /* Erase the region by Erase Sector command */
  for( ; StartAddrTemp < EndAddr; ) {
    __disable_irq();
    ucError = FLASHD_EraseSector(StartAddrTemp);
    __enable_irq();
    if( ucError ) {
      dbgprintf("EraseSector@%08lx fail: %hhu\r\n", StartAddrTemp, ucError);
      return ucError;
    }
    
    StartAddrTemp += IFLASH_SECTOR_SIZE;
    flashEraseProgress = (uint32_t)percent;
    dbgprintf("\r%ld %%", (int32_t)percent);
    bootloaderStatusIndicator(BS_ERASING);
    percent += fpercent;
  }

  flashEraseProgress = 100;
  dbgprintf("\rDone!\r\n");

  /* Lock the region */
  ucError = FLASHD_Lock(StartAddr, EndAddr, 0, 0 );
  if( ucError ) {
    dbgprintf("FLASHD_lock fail: %hhu\r\n", ucError);
    return ucError;
  }

  return ucError;
}

void Bootloader(void)
{
  /* Update internal flash Region to Full Access*/
  MPU_UpdateRegions(MPU_DEFAULT_IFLASH_REGION, IFLASH_START_ADDRESS,
	MPU_AP_FULL_ACCESS | INNER_NORMAL_WB_NWA_TYPE(NON_SHAREABLE) | MPU_CalMPURegionSize(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) |
	MPU_REGION_ENABLE);

  NVIC_SetPriority(USBHS_IRQn, 2);

  memset(&usbdDriver, 0, sizeof(USBDDriver));

  USBDDriver_Initialize(&usbdDriver, &ABADriverDescriptors, 0);
  USBD_Init();

  Wait(10);

  USBD_Connect();

  while(USBD_GetState() < USBD_STATE_CONFIGURED);
    
  // One call to read is enough, the subsequent call will be done from the ISR
  USBD_Read(ABADriverDescriptors_BULKOUT, gRxBuffer, sizeof(gRxBuffer), UsbDataReceived, 0);
    
  while(1)
  {
    if(eraseFlash) {
      FlashSectorErase(eraseStartAddr, eraseEndAddr);
      eraseFlash = 0;
    }

    bootloaderStatusIndicator(BS_IDLE); 
  }

}