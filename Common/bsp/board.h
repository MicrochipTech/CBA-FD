#pragma once

/*
#if 0
#include "board_rev1.h"
#else
#include "board_rev2.h"
#endif
*/

#include <sam.h>
#include <stdbool.h>
#include <stdint.h>
#include <compiler.h>
#include <pio.h>

void SystemInit(void);
void SystemCoreClockUpdate(void);
void sysclk_enable_usb(void);
void sysclk_disable_usb(void);

extern uint32_t SystemCoreClock;

#define BOARD_MCK                 150000000u
#define TWI_CLOCK                 400000u

#define EUI_MAGIC                 0xFCu
#define EUI_ADDRESS48             0x9Au
#define AT24MAC_ADDRESS_XPLAINED  0x5Fu
#define AT24MAC_ADDRESS           0x58u // 0101000 

#define AT24MAC_DEVICE_ADDRESS_BYTE_EEPROM  (0xA0>>1)
#define AT24MAC_DEVICE_ADDRESS_BYTE_EUI     (0xB0>>1)
#define AT24MAC_DEVICE_ADDRESS_BYTE_SER     (0xB0>>1)

#define AT32MAC_WORD_ADDRESS_BYTE_EEPROM    0x00
#define AT32MAC_WORD_ADDRESS_BYTE_EUI48     0x9A
#define AT32MAC_WORD_ADDRESS_BYTE_SER       0x90

#define GPIO_MAX_CNT              6u      // Hard-coded 6 GPIOs as they are the same on all ABA boards
#define GPIO_PIN_MASK             0x3Fu   // First 6 pins of PIOE are used
#define TRIGGER_PIN_MASK          0x0Cu

#define BOARD_FATAL_ERROR_TWI_READ_EUI_MAGIC "FATAL ERROR: TWI read EUI magic.\r\n"

extern bool board_fatal_error;
extern const char *p_board_fatal_error_description;

typedef enum _BSPCODE
{
  BSP_OK,
  BSP_ERR,
  BSP_ERR_PARAM,
  BSP_ERR_NA,
  BSP_ERR_CFG,
  BSP_ERR_BUSY
} BSPCODE;
/*
typedef enum _LED_PIN
{
  LED_PIN_START,
  LED_PIN_NONE = LED_PIN_START,
  LED_PIN_USB_R,
  LED_PIN_USB_G,
  LED_PIN_USB_B,
  LED_PIN_CAN1_R,
  LED_PIN_CAN1_G,
  LED_PIN_CAN1_B,
  LED_PIN_CAN0_R,
  LED_PIN_CAN0_G,
  LED_PIN_CAN0_B,
  LED_PIN_SD_R,
  LED_PIN_SD_G,
  LED_PIN_SD_B,
  LED_PIN_LIN_STAT,
  LED_PIN_LIN_DATA,
  LED_PIN_ENET_STAT,
  LED_PIN_ENET_DATA,
  LED_PIN_MAX
} __LEDS;
*/
typedef enum _LEDS
{
  LED_MIN = 0,
  LED_USB = LED_MIN,
  LED_CAN0,
  LED_CAN1,
  LED_SD,
  LED_MAX
} LEDS;

typedef enum _LED_TRI
{
  LED_TRI_MIN = 0,
  LED_TRI_OFF = LED_TRI_MIN,
  LED_TRI_R,
  LED_TRI_G,
  LED_TRI_B,
  LED_TRI_MAX
} LED_TRI;

typedef enum _LED_PRIO
{
  LED_PRIO_NONE,
  LED_PRIO_LOW,
  LED_PRIO_HIGH,
  LED_PRIO_CLEAR
} LED_PRIO;

typedef enum _GPIOS
{
  GPIO_SDDETECT = 0,
  GPIO_PICRST = 5,
  GPIO_LININH = 2,
  GPIO_LINEN = 3,
  GPIO_0 = 0,
  GPIO_1 = 1,
  GPIO_2 = 2,
  GPIO_3 = 3,
  GPIO_4 = 4,
  GPIO_5 = 5,
  GPIO_CANSTBY = 2
} GPIOS;

typedef enum _TRIGGERS
{
  TRG_1 = 0,
  TRG_2,
  TRG_EN,
  TRG_SDSTOP,
  TRG_MAX
} TRIGGERS;


typedef enum _CONSOLE
{
  CONSOLE_NONE,
  CONSOLE_XPLAINED,
  CONSOLE_CLICK,
  CONSOLE_ABA,
  CONSOLE_CBA = CONSOLE_ABA,
  CONSOLE_MAX
} CONSOLE;

typedef enum _DEVICE_PIN
{
  DP_ATMAC,
  DP_VERSION,
  DP_CONSOLE,
  DP_CAN0,
  DP_CAN1,
  DP_HSMCI,
  DP_LIN,
  DP_ETHERNET,
  DP_MIKROBUS0,
  DP_MIKROBUS1,
  DP_DSPIC,
  DP_GPIO,
  DP_TRIGGER,
  DP_MAX_CFG,
  DP_GPIO_DYNAMIC = 0,
  DP_GPIO_TRIGGER,
  DP_MAX_GPIO
} DEVICE_PIN;

typedef enum _BOARD_ID
{
  BOARD_UNKNOWN,
  BOARD_V71XPLAINED,
  BOARD_CBAREV1
} BOARD_ID;

typedef struct
{
  const Pin* pins;
  uint8_t count;
} pincount_t;

typedef struct
{
  pincount_t peripheral[DP_MAX_CFG];
  pincount_t leds[LED_MAX];
//  pincount_t dynamic[DP_MAX_GPIO];
} PinCfg_t;

/*
typedef struct  
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
} LedRgb_t;

typedef struct
{
    LedRgb_t rgb[LED_MAX];
} LedCfg_t;
*/

BOARD_ID Board_Initialize(void);
int8_t Board_Detect(BOARD_ID *p_board);
uint8_t Board_Configure(DEVICE_PIN device);
uint8_t Board_ReadSerial(uint8_t* buffer, uint8_t len);
uint8_t Board_ReadEeprom(uint8_t* buffer, uint8_t address, uint8_t len);
uint8_t Board_WriteEeprom(uint8_t* buffer, uint8_t address, uint8_t len);
uint8_t Board_ReadMacAddress(uint8_t* macAddress, uint8_t len);
uint8_t Board_ReadGpio(DEVICE_PIN device, uint8_t pin, uint8_t* value_out);
uint8_t Board_WriteGpio(DEVICE_PIN device, uint8_t pin, uint8_t value);

uint8_t Board_SetLed(LEDS led, LED_TRI color, LED_PRIO prio);

/* Deprecated */
//uint8_t Board_LedSet(__LEDS l);
//uint8_t Board_LedClear(__LEDS l);