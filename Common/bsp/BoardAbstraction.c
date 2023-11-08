#include <board.h>
#include <stdint.h>
#include <pio.h>
#include <string.h>
#include <twid.h>
#include <pmc.h>
#include <USB_Descriptor.h>

static void initCbaRev1(void);
static void initXplainedUltra(void);

Twid twid;
uint8_t at24Address = 0;
CONSOLE console = CONSOLE_NONE;
uint8_t ledPriorities[LED_MAX];

#define RED   0u
#define GREEN 1u
#define BLUE  2u
#define PIO_IGNORE { 0, 0, 0, 0 }

#define PinConfigure(x, y) \
{ \
  PinCfg.peripheral[x].pins = &y[0]; \
  PinCfg.peripheral[x].count = sizeof(y) / sizeof(Pin); \
}

#define LedConfigure(x, y) \
{ \
  PinCfg.leds[x].pins = &y[0]; \
  PinCfg.leds[x].count = sizeof(y) / sizeof(Pin); \
}

enum
{
  BRD_ID1,
  BRD_ID2,
  BRD_ID3,
  BRD_ID4
};

PinCfg_t PinCfg;

/* ATMAC is present on every Xplained/ABA/CBA board */
static const Pin _PINS_ATMAC[] =
{
  { PIOA, PIO_PA3A_TWIHS0_TWD0,   MUX_PA3A_TWIHS0_TWD0,   PIO_DEFAULT },    /* TWD0       */
  { PIOA, PIO_PA4A_TWIHS0_TWCK0,  MUX_PA4A_TWIHS0_TWCK0,  PIO_DEFAULT }     /* TWCK0      */
};

/* Version pins are available starting with ABA/CBA Rev.1 */
static const Pin _PINS_VERSION[] =
{
  { PIOC, PIO_PC10,               PIO_INPUT,              PIO_PULLDOWN },   /* BRD_ID1    */
  { PIOD, PIO_PD13,               PIO_INPUT,              PIO_PULLDOWN },   /* BRD_ID2    */
  { PIOC, PIO_PC9,                PIO_INPUT,              PIO_PULLDOWN },   /* BRD_ID3    */
  { PIOC, PIO_PC8,                PIO_INPUT,              PIO_PULLDOWN }    /* BRD_ID4    */
};

/* HSMCI are fixed function pins present on all boards */
static const Pin _PINS_HSMCI[] = 
{
  { PIOA, PIO_PA22,               PIO_INPUT,              PIO_DEFAULT },    /* DETECT     */
  { PIOA, PIO_PA25D_HSMCI_MCCK,   MUX_PA25D_HSMCI_MCCK,   PIO_DEFAULT },    /* MCCK       */
  { PIOA, PIO_PA26C_HSMCI_MCDA2,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA2      */
  { PIOA, PIO_PA27C_HSMCI_MCDA3,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA3      */
  { PIOA, PIO_PA28C_HSMCI_MCCDA,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCCDA      */
  { PIOA, PIO_PA30C_HSMCI_MCDA0,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA0      */
  { PIOA, PIO_PA31C_HSMCI_MCDA1,  PIO_PERIPH_C,           PIO_DEFAULT }     /* MCDA1      */
};

static const Pin _PINS_GPIO[] =
{
  { PIOE, PIO_PE0,                PIO_INPUT,              PIO_DEFAULT },    /* GPIO0      */
  { PIOE, PIO_PE1,                PIO_INPUT,              PIO_DEFAULT },    /* GPIO1      */
  { PIOE, PIO_PE2,                PIO_INPUT,              PIO_DEFAULT },    /* GPIO2      */
  { PIOE, PIO_PE3,                PIO_INPUT,              PIO_DEFAULT },    /* GPIO3      */
  { PIOE, PIO_PE4,                PIO_INPUT,              PIO_DEFAULT },    /* GPIO4      */
  { PIOE, PIO_PE5,                PIO_INPUT,              PIO_DEFAULT }     /* GPIO5      */
};

bool board_fatal_error = false;
const char *p_board_fatal_error_description;

int8_t Board_Detect(BOARD_ID *p_board)
{
  int8_t ret;
  uint8_t euiDummy[1];

  memset(&PinCfg, 0, sizeof(PinCfg));
  memset(&ledPriorities, 0, sizeof(ledPriorities));
  PinConfigure(DP_ATMAC, _PINS_ATMAC);
  PinConfigure(DP_VERSION, _PINS_VERSION);
  PIO_Configure(PinCfg.peripheral[DP_ATMAC].pins, PinCfg.peripheral[DP_ATMAC].count);

  PMC_EnablePeripheral(ID_TWIHS0);
  
  int twi_retries = 20;
  
  while(twi_retries--)
  {
    TWI_ConfigureMaster(TWIHS0, TWI_CLOCK, BOARD_MCK);
    TWID_Initialize(&twid, TWIHS0);

    euiDummy[0] = 0;
    ret = TWID_Read(&twid, AT24MAC_DEVICE_ADDRESS_BYTE_EUI, AT32MAC_WORD_ADDRESS_BYTE_EUI48, 1u, euiDummy, 1u, 0);
    if(0 != ret) {
      board_fatal_error = true;
      p_board_fatal_error_description = BOARD_FATAL_ERROR_TWI_READ_EUI_MAGIC;
      at24Address = 0;
      if(p_board) *p_board = BOARD_UNKNOWN;
      //return -1;
      continue;
    }
    else {
      break;
    }
  }
  
  if(board_fatal_error)
    return -1;
  
  if(euiDummy[0] == EUI_MAGIC)
  {
    at24Address = AT24MAC_ADDRESS;
    PIO_Configure(PinCfg.peripheral[DP_VERSION].pins, PinCfg.peripheral[DP_VERSION].count);
    
    uint8_t brdId = 0;
    brdId |= PIO_Get(&PinCfg.peripheral[DP_VERSION].pins[BRD_ID1]) << 0;
    brdId |= PIO_Get(&PinCfg.peripheral[DP_VERSION].pins[BRD_ID2]) << 1;
    brdId |= PIO_Get(&PinCfg.peripheral[DP_VERSION].pins[BRD_ID3]) << 2;
    brdId |= PIO_Get(&PinCfg.peripheral[DP_VERSION].pins[BRD_ID4]) << 3;
  
    switch(brdId)
    {
      case 8: { if(p_board) *p_board = BOARD_CBAREV1; break; }
      default: break;
    }
  }
  else
  {
    euiDummy[0] = 0;
    TWID_Read(&twid, AT24MAC_ADDRESS_XPLAINED, EUI_ADDRESS48, 1u, euiDummy, 1u, 0);
    if(euiDummy[0] == EUI_MAGIC)
    {
      at24Address = AT24MAC_ADDRESS_XPLAINED;
      if(p_board) *p_board = BOARD_V71XPLAINED;
    }
  }

  return 0;
}

BOARD_ID Board_Initialize(void)
{
  BOARD_ID board = BOARD_UNKNOWN;
  
  Board_Detect(&board);
  switch(board)
  {
    case BOARD_CBAREV1:
      console = CONSOLE_ABA;
      setUsbIds(USBD_PID_CBA, 0x0002);
      initCbaRev1();
    break;

    case BOARD_V71XPLAINED:
      console = CONSOLE_XPLAINED;
      setUsbIds(USBD_PID_CBA, 0x0000);
      initXplainedUltra();
    break;
  
    default:
    break;
  }

  if(board != BOARD_UNKNOWN)
  {
    // Disable the MATRIX registers write protection => change PB04 configuration
    MATRIX->MATRIX_WPMR  = MATRIX_WPMR_WPKEY_PASSWD;
    MATRIX->CCFG_SYSIO  |= CCFG_SYSIO_SYSIO4;
    MATRIX->MATRIX_WPMR  = MATRIX_WPMR_WPKEY_PASSWD | MATRIX_WPMR_WPEN;   
    
    /* Set the debounce filter */
    PIO_SetDebounceCutoff(PIOA, 100);
    PIO_SetDebounceCutoff(PIOB, 100);
    PIO_SetDebounceCutoff(PIOC, 100);
    PIO_SetDebounceCutoff(PIOD, 100);
    PIO_SetDebounceCutoff(PIOE, 100);

    for(int i = DP_VERSION + 1; i < DP_MAX_CFG; i++)
    {
      if(PinCfg.peripheral[i].count > 0)
      {
        PIO_Configure(PinCfg.peripheral[i].pins, PinCfg.peripheral[i].count);
      }
    }

    for(int i = LED_MIN; i < LED_MAX; i++)
    {
      if(PinCfg.leds[i].count > 0)
      {
        PIO_Configure(PinCfg.leds[i].pins, PinCfg.leds[i].count);
      }
    }
  }

  return board;
}

static void initCbaRev1()
{
  static const Pin _PINS_LED_USB[] =
  {
    { PIOD, PIO_PD5,                PIO_OUTPUT_1,           PIO_HIGH_Z  },    /* USB_R   */
    { PIOD, PIO_PD4,                PIO_OUTPUT_1,           PIO_DEFAULT },    /* USB_G   */
    { PIOC, PIO_PC23,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* USB_B   */
  };
  
  static const Pin _PINS_LED_CAN0[] =
  {
    { PIOC, PIO_PC16,               PIO_OUTPUT_1,           PIO_HIGH_Z  },    /* CAN0_R  */
    { PIOA, PIO_PA1,                PIO_OUTPUT_1,           PIO_DEFAULT },    /* CAN0_G  */
    { PIOD, PIO_PD11,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* CAN0_B  */
  };
  
  static const Pin _PINS_LED_CAN1[] =
  {
    { PIOC, PIO_PC17,               PIO_OUTPUT_1,           PIO_HIGH_Z  },    /* CAN1_R  */
    { PIOA, PIO_PA0,                PIO_OUTPUT_1,           PIO_DEFAULT },    /* CAN1_G  */
    { PIOD, PIO_PD10,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* CAN1_B  */
  };
  
  static const Pin _PINS_LED_SD[] =
  {
    { PIOD, PIO_PD19,               PIO_OUTPUT_1,           PIO_HIGH_Z  },    /* SD_R       */
    { PIOA, PIO_PA12,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* SD_G       */
    { PIOD, PIO_PD18,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* SD_B       */
  };

  static const Pin _PINS_MIKROBUS0[] =
  {
    { PIOD, PIO_PD30,               PIO_INPUT,              PIO_DEFAULT },    /* AFE0-AD0    */
    { PIOD, PIO_PD26,               PIO_OUTPUT_0,           PIO_DEFAULT },    /* RST_MBUS    */
    { PIOD, PIO_PD25B_SPI0_NPCS1,   MUX_PD25B_SPI0_NPCS1,   PIO_DEFAULT },    /* SPI0_NPCS1  */
    { PIOD, PIO_PD22B_SPI0_SPCK,    MUX_PD22B_SPI0_SPCK,    PIO_DEFAULT },    /* SPI0_SPCK   */
    { PIOD, PIO_PD21B_SPI0_MOSI,    MUX_PD21B_SPI0_MOSI,    PIO_DEFAULT },    /* SPI0_MISO   */
    { PIOD, PIO_PD20B_SPI0_MISO,    MUX_PD20B_SPI0_MISO,    PIO_DEFAULT },    /* SPI0_MOSI   */        
    { PIOD, PIO_PD25A_PWM0_PWML1,   MUX_PD25A_PWM0_PWML1,   PIO_DEFAULT },    /* PWMC0_PWML0 */
    { PIOA, PIO_PA14,               PIO_INPUT,              PIO_DEFAULT },    /* WKUP8       */
    { PIOD, PIO_PD15B_USART2_RXD2,  MUX_PD15B_USART2_RXD2,  PIO_DEFAULT },    /* RXD2        */
    { PIOD, PIO_PD16B_USART2_TXD2,  MUX_PD16B_USART2_TXD2,  PIO_DEFAULT },    /* TXD2        */
    { PIOD, PIO_PD28C_TWIHS2_TWCK2, MUX_PD28C_TWIHS2_TWCK2, PIO_DEFAULT },    /* TWCK2       */
    { PIOD, PIO_PD27C_TWIHS2_TWD2,  MUX_PD27C_TWIHS2_TWD2,  PIO_DEFAULT }     /* TWD2        */
  };

  static const Pin _PINS_MIKROBUS1[] =
  {
    { PIOC, PIO_PC30,               PIO_INPUT,              PIO_DEFAULT },    /* AFE1-AD5    */
    { PIOD, PIO_PD2,                PIO_OUTPUT_0,           PIO_DEFAULT },    /* RST_MBUS_1  */
    { PIOC, PIO_PD1C_SPI1_NPCS2,    MUX_PD1C_SPI1_NPCS2,    PIO_DEFAULT },    /* SPI1_NPCS2  */
    { PIOC, PIO_PC24C_SPI1_SPCK,    MUX_PC24C_SPI1_SPCK,    PIO_DEFAULT },    /* SPI1_SPCK   */
    { PIOC, PIO_PC27C_SPI1_MOSI,    MUX_PC27C_SPI1_MOSI,    PIO_DEFAULT },    /* SPI1_MISO   */
    { PIOC, PIO_PC26C_SPI1_MISO,    MUX_PC26C_SPI1_MISO,    PIO_DEFAULT },    /* SPI1_MOSI   */
    { PIOA, PIO_PA20B_PWM0_PWML1,   MUX_PA20B_PWM0_PWML1,   PIO_DEFAULT },    /* PWMC0_PWML1 */
    { PIOA, PIO_PA2,                PIO_INPUT,              PIO_DEFAULT },    /* WKUP2       */
    { PIOA, PIO_PA9A_UART0_URXD0,   MUX_PA9A_UART0_URXD0,   PIO_DEFAULT },    /* RXD0        */
    { PIOA, PIO_PA10A_UART0_UTXD0,  MUX_PA10A_UART0_UTXD0,  PIO_DEFAULT },    /* TXD0        */
    { PIOB, PIO_PB5A_TWIHS1_TWCK1,  MUX_PB5A_TWIHS1_TWCK1,  PIO_DEFAULT },    /* TWCK1       */
    { PIOB, PIO_PB4A_TWIHS1_TWD1,   MUX_PB4D_USART1_TXD1,   PIO_DEFAULT }     /* TWD1        */
  };
  
  static const Pin _PINS_CAN0[] =
  {
    { PIOB, PIO_PB3A_MCAN0_CANRX0,  MUX_PB3A_MCAN0_CANRX0,  PIO_DEFAULT },    /* CANRX0      */
    { PIOB, PIO_PB2A_MCAN0_CANTX0,  MUX_PB2A_MCAN0_CANTX0,  PIO_DEFAULT },    /* CANTX0      */
    { PIOA, PIO_PA5,                PIO_OUTPUT_0,           PIO_DEFAULT }     /* STBY0       */
  };

  static const Pin _PINS_CAN1[] =
  {
    { PIOC, PIO_PC12C_MCAN1_CANRX1, MUX_PC12C_MCAN1_CANRX1, PIO_DEFAULT },    /* CANRX1     */
    { PIOC, PIO_PC14C_MCAN1_CANTX1, MUX_PC14C_MCAN1_CANTX1, PIO_DEFAULT },    /* CANTX1     */
    { PIOD, PIO_PD29,               PIO_OUTPUT_0,           PIO_DEFAULT }     /* STBY1      */
  };

  static const Pin _PINS_TRIGGER[]=
  {
    { PIOC, PIO_PC3,      PIO_OUTPUT_0,     PIO_DEFAULT                 },   /* TRG1       */
    { PIOC, PIO_PC2,      PIO_INPUT,        PIO_PULLDOWN | PIO_DEBOUNCE },   /* TRG2       */
    { PIOC, PIO_PC4,      PIO_OUTPUT_0,     PIO_DEFAULT                 },   /* ~3STATE2   */
    { PIOA, PIO_PA23,     PIO_INPUT,        PIO_PULLDOWN | PIO_DEBOUNCE }    /* SD_STOP    */
  };

  static const Pin _PINS_CONSOLE[] =
  {
    { PIOA, PIO_PA9A_UART0_URXD0,   MUX_PA9A_UART0_URXD0,   PIO_DEFAULT },    /* URXD0      */
    { PIOA, PIO_PA10A_UART0_UTXD0,  MUX_PA10A_UART0_UTXD0,  PIO_DEFAULT }     /* UTXD0      */
  };

  PinConfigure(DP_CAN0, _PINS_CAN0);
  PinConfigure(DP_CAN1, _PINS_CAN1);
  PinConfigure(DP_TRIGGER, _PINS_TRIGGER);
  PinConfigure(DP_CONSOLE, _PINS_CONSOLE);
  PinConfigure(DP_HSMCI, _PINS_HSMCI);
  PinConfigure(DP_MIKROBUS0, _PINS_MIKROBUS0);
  PinConfigure(DP_MIKROBUS1, _PINS_MIKROBUS1);
  
  LedConfigure(LED_USB, _PINS_LED_USB);
  LedConfigure(LED_CAN0, _PINS_LED_CAN0);
  LedConfigure(LED_CAN1, _PINS_LED_CAN1);
  LedConfigure(LED_SD, _PINS_LED_SD);
}

static void initXplainedUltra()
{
#if 0
  static const Pin _PINS_LED[] =
  {
      PIO_IGNORE,
    { PIOA, PIO_PA23,               PIO_OUTPUT_1,           PIO_DEFAULT },    /* USB_STAT - LED0  */
    { PIOC, PIO_PC9,                PIO_OUTPUT_1,           PIO_DEFAULT }     /* USB_DATA - LED1  */
  };
  PinCfg.peripheral[DP_LED].pins = _PINS_LED;
  PinCfg.peripheral[DP_LED].count = sizeof(_PINS_LED) / sizeof(Pin);
#endif

  static const Pin _PINS_CAN0[] =
  {
    { PIOB, PIO_PB2A_MCAN0_CANTX0,  MUX_PB2A_MCAN0_CANTX0,  PIO_DEFAULT },    /* CANTX0     */
    { PIOB, PIO_PB3A_MCAN0_CANRX0,  MUX_PB3A_MCAN0_CANRX0,  PIO_DEFAULT },    /* CANRX0     */
  };
  PinCfg.peripheral[DP_CAN0].pins = _PINS_CAN0;
  PinCfg.peripheral[DP_CAN0].count = sizeof(_PINS_CAN0) / sizeof(Pin);

  static const Pin _PINS_CAN1[] =
  {
    { PIOC, PIO_PC12C_MCAN1_CANRX1, MUX_PC12C_MCAN1_CANRX1, PIO_DEFAULT },    /* CANRX1     */
    { PIOC, PIO_PC14C_MCAN1_CANTX1, MUX_PC14C_MCAN1_CANTX1, PIO_DEFAULT }     /* CANTX1     */
  };
  PinCfg.peripheral[DP_CAN1].pins = _PINS_CAN1;
  PinCfg.peripheral[DP_CAN1].count = sizeof(_PINS_CAN1) / sizeof(Pin);

  static const Pin _PINS_CONSOLE[] =
  {
    { PIOA, PIO_PA21A_USART1_RXD1,  MUX_PA21A_USART1_RXD1,  PIO_DEFAULT },    /* RXD1       */
    { PIOB, PIO_PB4D_USART1_TXD1,   MUX_PB4D_USART1_TXD1,   PIO_DEFAULT }     /* TXD1       */
  };
  PinCfg.peripheral[DP_CONSOLE].pins = _PINS_CONSOLE;
  PinCfg.peripheral[DP_CONSOLE].count = sizeof(_PINS_CONSOLE) / sizeof(Pin);

  static const Pin _PINS_LIN[] =
  {
    { PIOB, PIO_PB0C_USART0_RXD0,   MUX_PB0C_USART0_RXD0,   PIO_DEFAULT },    /* RXD0       */
    { PIOB, PIO_PB1C_USART0_TXD0,   MUX_PB1C_USART0_TXD0,   PIO_DEFAULT }     /* TXD0       */
  };
  PinCfg.peripheral[DP_LIN].pins = _PINS_LIN;
  PinCfg.peripheral[DP_LIN].count = sizeof(_PINS_LIN) / sizeof(Pin);

  static const Pin _PINS_HSMCI[] =
  {
    { PIOD, PIO_PD18,               PIO_INPUT,              PIO_DEFAULT },    /* DETECT     */
    { PIOA, PIO_PA25D_HSMCI_MCCK,   MUX_PA25D_HSMCI_MCCK,   PIO_DEFAULT },    /* MCCK       */
    { PIOA, PIO_PA26C_HSMCI_MCDA2,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA2      */
    { PIOA, PIO_PA27C_HSMCI_MCDA3,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA3      */
    { PIOA, PIO_PA28C_HSMCI_MCCDA,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCCDA      */
    { PIOA, PIO_PA30C_HSMCI_MCDA0,  PIO_PERIPH_C,           PIO_DEFAULT },    /* MCDA0      */
    { PIOA, PIO_PA31C_HSMCI_MCDA1,  PIO_PERIPH_C,           PIO_DEFAULT }     /* MCDA1      */
    };
  PinCfg.peripheral[DP_HSMCI].pins = _PINS_HSMCI;
  PinCfg.peripheral[DP_HSMCI].count = sizeof(_PINS_HSMCI) / sizeof(Pin);
}

uint8_t Board_SetLed(LEDS led, LED_TRI color, LED_PRIO prio)
{
  uint8_t ret = BSP_OK;
  
  if(prio >= ledPriorities[led])
  {
    ledPriorities[led] = prio;
    switch(color)
    {
      case LED_TRI_OFF:
        PIO_SetHighZ  (&PinCfg.leds[ led ].pins[ RED   ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ GREEN ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ BLUE  ]);
      break;

      case LED_TRI_R:
        PIO_Clear     (&PinCfg.leds[ led ].pins[ RED   ]);
        PIO_ClearHighZ(&PinCfg.leds[ led ].pins[ RED   ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ GREEN ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ BLUE  ]);
      break;
      
      case LED_TRI_G:
        PIO_SetHighZ  (&PinCfg.leds[ led ].pins[ RED   ]);
        PIO_Clear     (&PinCfg.leds[ led ].pins[ GREEN ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ BLUE  ]);
      break;
      
      case LED_TRI_B:
        PIO_SetHighZ  (&PinCfg.leds[ led ].pins[ RED   ]);
        PIO_Set       (&PinCfg.leds[ led ].pins[ GREEN ]);
        PIO_Clear     (&PinCfg.leds[ led ].pins[ BLUE  ]);
      break;
      
      default:
        ret = BSP_ERR;
      break;
    }
  }
  else
  {
    ret = BSP_ERR;
  }
  
  if(prio == LED_PRIO_CLEAR)
  {
    ledPriorities[led] = LED_PRIO_NONE;
  }
  
  return ret;
}

uint8_t Board_ReadEeprom(uint8_t* buffer, uint8_t address, uint8_t len)
{
  uint8_t ret = BSP_ERR_PARAM;
  
  //if(at24Address != 0 && buffer != 0 && len >= 1 && len <= 0x80u)
  if(buffer != 0 && len >= 1 && len <= 0x80u)
  {
    uint16_t eeprom_addr = at24Address & ~0x8;
    TWID_Read(&twid, eeprom_addr, address, 1u, buffer, len, 0);
    ret = BSP_OK;
  }
  
  return ret;
}

uint8_t Board_WriteEeprom(uint8_t* buffer, uint8_t address, uint8_t len)
{
  uint8_t ret = BSP_ERR_PARAM;
  
  //if(at24Address != 0 && buffer != 0 && len >= 1 && len <= 16u)
  if(buffer != 0 && len >= 1 && len <= 16u)
  {
    uint16_t eeprom_addr = at24Address & ~0x8;
    TWID_Write(&twid, eeprom_addr, address, 1u, buffer, len, 0);
    ret = BSP_OK;
  }
  
  return ret;  
}

uint8_t Board_ReadSerial(uint8_t* buffer, uint8_t len)
{
  uint8_t ret = BSP_ERR_PARAM;
  
  // Serial of AT24MAC is 16 byte max
  if(at24Address != 0 && buffer != 0 && len >= 16)
  {
    if(len > 16) {
      len = 16;
    }
    TWID_Read(&twid, at24Address, 0x80u, 1u, buffer, len, 0);
    ret = BSP_OK;
  }

  return ret;
}

uint8_t Board_ReadMacAddress(uint8_t* macAddress, uint8_t len)
{
  uint8_t ret = BSP_ERR_PARAM;
  
  // MAC address is always 6 byte
  if(at24Address != 0 && macAddress != 0 && (len == 6u || len == 8u))
  {
    memset(macAddress, 0, len);
    TWID_Read(&twid, at24Address, ((len == 6u) ? 0x9Au : 0x98u), 1u, macAddress, len, 0);
    ret = BSP_OK;
  }

  return ret;
}

uint8_t Board_ReadGpio(DEVICE_PIN device, uint8_t pin, uint8_t* value_out)
{
  uint8_t ret = BSP_OK;

  if(value_out)
  {
    if(PinCfg.peripheral[device].count >= pin)
    {
      const Pin* p = &PinCfg.peripheral[device].pins[pin];
      *value_out = PIO_Get(p);
    }
    else
    {
      ret = BSP_ERR_NA;
    }
  }
  else
  {
    ret = BSP_ERR_PARAM;
  }

  return ret;
}

uint8_t Board_WriteGpio(DEVICE_PIN device, uint8_t pin, uint8_t value)
{
  uint8_t ret = BSP_OK;

  if(PinCfg.peripheral[device].count >= pin)
  {
    const Pin* p = &PinCfg.peripheral[device].pins[pin];
    
    if(value == 0)
    {
      if(p->type == PIO_OUTPUT_0)
      {
        // Value 0, Output normal => clear
        PIO_Clear(p);
      }
      else if(p->type == PIO_OUTPUT_1)
      {
        // Value 0, Output inverted => set
        PIO_Set(p);
      }
      else
      {
        // Pin not an output
        ret = BSP_ERR_CFG;        
      }
    }
    else if(value == 1u)
    {
      // Value 1, Output normal => set
      if(p->type == PIO_OUTPUT_0)
      {
        PIO_Set(p);
      }
      else if(p->type == PIO_OUTPUT_1)
      {
        // Value 1, Output inverted => clear
        PIO_Clear(p);
      }
      else
      {
        // Pin not an output
        ret = BSP_ERR_CFG;
      }
    }
    else
    {
      // Value for output is 0 (disable) or 1 (enable)
      ret = BSP_ERR_PARAM;
    }
  }
  else
  {
    // Pin not available
    ret = BSP_ERR_NA;
  }
  
  return ret;
}
