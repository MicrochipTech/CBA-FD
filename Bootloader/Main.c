/*
 * Bootloader.c
 */ 


#include <sam.h>
#include <board.h>
#include <wdt.h>
#include <twid.h>
#include <pio.h>
#include <pmc.h>
#include <timetick.h>
#include <stdio.h>
#include <string.h>

#include "Bootloader.h"
#include "CRC.h"
#include "config.h"
#include "USB_Descriptor.h"
#include "versions.h"

typedef void(*tAppCall)(void);

extern void Dummy_Handler(void);

struct bRamLayout* BRAM = (struct bRamLayout*)BRAM_ADDR;

uint8_t macAddr[6];

#define BLINK_DLY 500u
#define BLINK_SLOPE 10u
#define BOOTLOADER_BUTTON_TIME 1000u

/*
 * @brief Check if the provided stack address is located in SRAM
 *
 * Check if the provided address is mapped to the SRAM address specified by the linker script.
 * @param stackAddressU32 The stack address to verify.
 * @return 0 if the address is invalid and 1 otherwise.
 */
uint8_t isStackAddrValid(uint32_t stackAddressU32)
{
  extern uint32_t _ram_start_;
  extern uint32_t _ram_end_;
  extern uint32_t _dtcm_start_;
  extern uint32_t _dtcm_end_;
  
  return
   (stackAddressU32 >= (uint32_t)&_ram_start_ && stackAddressU32 <= (uint32_t)&_ram_end_)  ||
   (stackAddressU32 >= (uint32_t)&_dtcm_start_ && stackAddressU32 <= (uint32_t)&_dtcm_end_)
   ? 1U : 0;
}

/*
 * @brief Check if the provided initialization vector address is located in FLASH
 *
 * Check if the provided address is mapped to the FLASH address specified by the linker script.
 * @param vtorAddressU32 The vtor address to verify.
 * @return 0 if the address is invalid and 1 otherwise.
 */
uint8_t isVtorAddrValid(uint32_t vtorAddressU32)
{
  extern uint32_t _appl_start_;
  extern uint32_t _appl_end_;

  return (vtorAddressU32 >= (uint32_t)&_appl_start_ && vtorAddressU32 <= (uint32_t)&_appl_end_) ? 1U : 0;
}

/*
 * @brief Verify the checksum of the flashed application located in FLASH
 * 
 * Calculate and verify the checksum of the whole application section in FLASH.
 * @return 0 on checksum missmatch
 * @return 1 if checksum matched or SKIP_CHECKSUM was set
 */
uint8_t applicationChecksum()
{
  if(BRAM->skip_checksum == SKIP_CHECKSUM) {
    return 1;
  }

  extern uint32_t _appl_start_;
  extern uint32_t _appl_end_;
  extern uint32_t _app_chkSumAddr;
  extern uint32_t _app_chkSumLen;

  if((uint32_t)_app_chkSumLen > (uint32_t)&_appl_end_ - (uint32_t)&_appl_start_) {
    return 0;
  }

  crc32c_init_table();

  uint32_t embeddedCRC = _app_chkSumAddr;
  uint32_t calculatedCRC = crc32((const uint8_t*)&_appl_start_, (uint32_t)_app_chkSumLen);

  if(embeddedCRC == calculatedCRC) {
    return 1;
  }
  else {
    dbgprintf("Checksum missmatch!\r\n");
    return 0;
  }  
}

/*
 * @brief Check if the application has requested to reboot into bootloader mode.
 *
 * @return 0 if there was no request and 1 otherwise.
 */
uint8_t bootloaderRequested(uint32_t request)
{
  uint8_t ret = 0;

  if(BRAM->bootloader_req == request) {
    ret = 1;
  }

  return ret;
}

static void run_bootloader(void)
{
  memset(BRAM, 0, sizeof(*BRAM));

  /* Initialize MAC address and serial number */
  Board_ReadMacAddress(macAddr, 6);

  char serialString[13];
  sprintf(serialString, "%02x%02x%02x%02x%02x%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  extern unsigned char productSerialString[];
  for(uint32_t i=0; i<sizeof(serialString)-1u; i++)
  {
    productSerialString[(i * 2u) + 2u] = serialString[i];
  }
  productSerialString[0] = 2u + 2u * strlen(serialString);

  Bootloader();
  NVIC_SystemReset();
}

void startSamba(void)
{
  TimeTick_Disable();
  memset(BRAM, 0, sizeof(*BRAM));
  uint32_t samba_stackAddr = *(uint32_t*)0x00800000u;
  uint32_t samba_vtorAddr = *(uint32_t*)0x00800004u;
  SCB->VTOR = (0x00800000u & SCB_VTOR_TBLOFF_Msk);
  __set_MSP(samba_stackAddr);
  __set_PSP(samba_stackAddr);
  SCB_DisableICache();
  ((tAppCall)samba_vtorAddr)();
  Dummy_Handler();
}

void bootloaderStatusIndicator(BSTATUS status)
{
  /* This is a unspecific state state-machine.
   * It is called on every event which may occur cyclic or sporadic.
   * Therefore some extended status handling is required.
   * Having a cyclic invocation would require an interrupt, which will be
   * disabled during erase/flashing.
   */
  
  uint32_t tick = GetTicks();
  static uint32_t lastTick = 0;
  static uint32_t onOffCycle = 0;
  static uint32_t twinkleCtr = 0;
  static BSTATUS prevStatus = BS_IDLE;
  
  if(status == BS_IDLE && twinkleCtr) {
    /* Change request to idle, but there are still twinkles left => stay in BS_TWINKLE */
    status = BS_TWINKLE;
  }
  else if(status == BS_IDLE && prevStatus == BS_ERROR) {
    /* Change request to idle, but there was an error previously => stay in BS_ERROR */
    status = BS_ERROR;
  }
  
  switch(status)
  {
    case BS_IDLE:
      if(tick - lastTick >= BLINK_DLY)
      {

        if(onOffCycle++) {
          Board_SetLed(LED_USB, LED_TRI_OFF, LED_PRIO_CLEAR);
          onOffCycle = 0;
        }
        else {
          Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_NONE);
        }
        lastTick = tick;
      }      
    break;
    
    case BS_ERROR:
      Board_SetLed(LED_USB, LED_TRI_R, LED_PRIO_HIGH);
    break;
    
    case BS_TWINKLE:
      if(twinkleCtr)
      {
        if(tick - lastTick >= (BLINK_DLY >> 2))
        {
          if(onOffCycle++) {
            Board_SetLed(LED_USB, LED_TRI_OFF, LED_PRIO_CLEAR);
            onOffCycle = 0;
          }
          else {
            Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_LOW);
          }
          twinkleCtr--;
          lastTick = tick;
        }
      }
      else
      {
        Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_LOW);
        twinkleCtr = 20;
      }      
    break;
    
    case BS_ERASING:    
    case BS_FLASHING:
      /* Erasing/flashing will clear all other LEDs.
       * In case of an error, the state-machine needs to be called with BS_ERROR and USB_R will be set.
       */
      if(onOffCycle++) {
        Board_SetLed(LED_USB, LED_TRI_OFF, LED_PRIO_CLEAR);
        onOffCycle = 0;  
      }
      else {
        Board_SetLed(LED_USB, LED_TRI_G, LED_PRIO_HIGH);
      }
      
      lastTick = tick;
      
    break;
  }
  
  prevStatus = status;
}

int main(void)
{
  /* Addresses are imported from linker script */
  extern uint32_t _app_stackAddr;
  extern uint32_t _app_vtorAddr;

  SystemInit();
  WDT_Disable(WDT);
  SCB_EnableICache();
  TimeTick_Configure();
  
  /* Disable Chip Erase */
  MATRIX->MATRIX_WPMR  = MATRIX_WPMR_WPKEY_PASSWD;
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12_Msk;
  MATRIX->MATRIX_WPMR  = MATRIX_WPMR_WPKEY_PASSWD | MATRIX_WPMR_WPEN;
  
  /* Detect the board and store it in BRAM */
  BOARD_ID board;
  Board_Detect(&board);
  Board_Initialize();
  setUsbIds(USBD_PID_BOOTLOADER, 0x0001);
  BRAM->board_ver = board;

  /* Check if SDSTOP is pressed and stay in bootloader if so */
  /* Errata? When the pin is configured as input with weak_pulldown and debounce, it will report a high level for the first debounce period */
  uint32_t requestCtr = 0;
  uint8_t buttonPushed = 0;
  do 
  {
    Wait(1u);
    requestCtr++;
    Board_ReadGpio(DP_TRIGGER, TRG_SDSTOP, &buttonPushed);
  } while (buttonPushed == 1u);

  /* SAM-BA was requested */
  if(bootloaderRequested(SAMBA_REQUEST))
  {
    dbgprintf("Starting SAM-BA bootloader\r\n");
    startSamba();
  }
    
  /* Initialize the SAM system */
  if(bootloaderRequested(BOOTLOADER_REQUEST) || !applicationChecksum() || BOOTLOADER_BUTTON_TIME <= requestCtr)
  {
    SCB_EnableDCache();
    dbgprintf("Starting MBA bootloader\r\n");
    run_bootloader();
  }

  /* No bootloader was requested, check if there's an valid application */
  if(isStackAddrValid(_app_stackAddr) && isVtorAddrValid(_app_vtorAddr))
  {
    TimeTick_Disable();
    __set_MSP(_app_stackAddr);
    __set_PSP(_app_stackAddr);

    tAppCall application = (tAppCall)_app_vtorAddr;
    BRAM->bootloader_ver = BOOTLOADER_FIRMWARE_VERSION;
    SCB_DisableICache();
    application();
  }
  else
  {
    run_bootloader();
  }

  Dummy_Handler();
}
