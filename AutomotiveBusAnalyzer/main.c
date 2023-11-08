/*
 * AutomotiveBusAnalyzer.c
 */ 

#include <string.h>
#include <board.h>
#include <led.h>
#include <wdt.h>
#include <pmc.h>
#include <timetick.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <assert.h>
#include <pio.h>
#include <xdmad.h>
#include <sdmmc.h>
#include <sdmmc_cmd.h>
#include <mcid.h>
#include <ff.h>
#include <settings.h>

#include "config.h"
#include "rtos.h"
#include "tasks/task_CAN.h"
#include "tasks/task_LOG.h"
#include "tasks/task_USB.h"
#include "ABAControl.h"

#include "mpu.h"

/** XDMAC driver */
static sXdmad dmaDrv;
static sMcid mciDrv;
extern sSdCard sdDrv[BOARD_NUM_MCI];

/** I2C standard device driver */
uint8_t serialNr[16];
uint8_t macAddr[6];

static uint8_t sdcard_avail = 0;
sdCfg_t* settings = 0;
BOARD_ID board = BOARD_UNKNOWN;

COMPILER_SECTION(".ram_nocache") COMPILER_ALIGNED(4) static FATFS fs;
COMPILER_SECTION(".ram_nocache") COMPILER_ALIGNED(4) static DIR dir;

static const char *hwString[] = {"Unknown", "V71 Xplained Ultra", "ABA Revision 0", "ABA Revision 1", "CBA Revision 1", "\0\0"};

void XDMAC_Handler(void)
{
  XDMAD_Handler(&dmaDrv);
}

COMPILER_SECTION(".code_TCM")
void HSMCI_Handler(void)
{
  MCID_Handler(&mciDrv);
}

static void initFs(void)
{
  const TCHAR Drv_Num = DEV_MMC;
  
  memset(&fs, 0, sizeof(fs));
  memset(&dir, 0, sizeof(dir));
  
  SD_Init(&sdDrv[0]);

  if(FR_OK == f_mount(&fs, &Drv_Num, 1))
  {
    /* Open root directory */
    if(FR_OK == f_opendir(&dir, "0:"))
    {
      /* Mark SD card as available for the logger */
      sdcard_avail = 1;
    }
  }
}

static void readSdConfig(void)
{
  FIL file;
  
  memset(&file, 0, sizeof(file));

  if(sdcard_avail)
  {
    /* Open ABA config file */
    if(FR_OK == f_open(&file, "ABA.json", FA_READ))
    {
      UINT fSize = f_size(&file);
      if(fSize <= 2048)
      {
        /* Allocate buffer on stack (init task should have at least 4kb */
        char jsonBuffer[2048];
        memset(jsonBuffer, 0, sizeof(jsonBuffer));
        UINT bytesRead = 0;
        if(FR_OK == f_read(&file, jsonBuffer, fSize, &bytesRead))
        {
          jsonBuffer[bytesRead] = 0;
          settings = parseSettings(jsonBuffer, bytesRead);
        }
        else {
          printf("Error: Failed to open ABA.json\r\n\n");
        }
      }
      else {
        printf("Error: Cannot parse ABA.json, file is to large...\r\n");
      }
    }
  }
}

/**
 * Call all init tasks here
 */
uint32_t init_task_callback(void)
{
  board = Board_Initialize();
  for(uint32_t i=0; i<16; i++) {
    MPU->RNR = i;
    printf("Region [%lu], ", i);
    printf("RBAR=0x%08lx, ", MPU->RBAR);
    printf("RASR=0x%08lx\r\n", MPU->RASR);
  }  
  
  printf("-- Automotive Bus Analyzer Core Project --\r\n");
  printf("-- Compiled: %s %s With %s--\r\n", __DATE__, __TIME__ ,	COMPILER_NAME);
  printf("-- Detected HW: %s\r\n", hwString[board]);
  while(board == BOARD_UNKNOWN);
   
  /* Priorities higher than 4 are not supported by current FreeRTOS configuration */
  NVIC_SetPriority(USBHS_IRQn, 5);
  NVIC_SetPriority(MCAN0_INT0_IRQn, 4);
  NVIC_SetPriority(MCAN0_INT1_IRQn, 4);
  NVIC_SetPriority(MCAN1_INT0_IRQn, 4);
  NVIC_SetPriority(MCAN1_INT1_IRQn, 4);
  NVIC_SetPriority(HSMCI_IRQn, 4);
  NVIC_SetPriority(USART0_IRQn, 4);
  NVIC_SetPriority(ISI_IRQn, 4);
  NVIC_SetPriority(PIOA_IRQn, 5);
  
  /* Initialize MAC address and serial number */
  Board_ReadSerial(serialNr, 16u);
  Board_ReadMacAddress(macAddr, 6);

  char serialString[13];
  sprintf(serialString, "%02x%02x%02x%02x%02x%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  extern unsigned char productSerialString[];
  for(uint32_t i=0; i<sizeof(serialString)-1; i++)
  {
    productSerialString[(i * 2) + 2] = serialString[i];
  }
  productSerialString[0] = 2 + 2 * strlen(serialString);
  

  PMC_EnablePeripheral(ID_TRNG);
  TRNG->TRNG_CR = TRNG_CR_KEY_PASSWD | TRNG_CR_ENABLE_Msk;
  XDMAD_Initialize(&dmaDrv, 0);
  MCID_Init(&mciDrv, HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0);
  SDD_Initialize(&sdDrv[0], &mciDrv, 0);

  NVIC_ClearPendingIRQ(HSMCI_IRQn);
  NVIC_EnableIRQ(HSMCI_IRQn);
  NVIC_ClearPendingIRQ(XDMAC_IRQn);
  NVIC_EnableIRQ(XDMAC_IRQn);

  settings = getSettings();
  memset(settings, 0, sizeof(*settings));

  /* Check if a SD card is inserted */
  uint8_t value = 0, ret = 0;
  ret = Board_ReadGpio(DP_HSMCI, GPIO_SDDETECT, &value);
  if(ret == BSP_OK && value == 0u) {
    /* Read config from SD card if available */
    initFs();
    readSdConfig();
    Board_SetLed(LED_SD, LED_TRI_B, LED_PRIO_NONE);
  }

  init_log_task();
  if((settings->initialized && settings->logging)) {
    notifyToggleLogger();
  }
  init_aba_api();
  init_can_task(settings->canCfg);
  init_usb_task();
  
  return (uint32_t)board;
}

int main(void)
{
  SystemCoreClockUpdate();
  
  WDT_Disable(WDT);
  
  SCB_EnableICache();
  SCB_EnableDCache();

  rtos_run();
}
