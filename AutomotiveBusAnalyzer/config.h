/*
 * config.h
 *
 * Created: 19.02.2018 13:17:44
 *  Author: M43734
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include "compiler.h"
#include "USB_Commands.h"

#define BRAM_ADDR 0x40074000U

#define ABADriverDescriptors_BULKOUT        1
#define ABADriverDescriptors_BULKIN         2

#define USB_RX_SIZE                         1024
#define USB_TX_SIZE                         8192

typedef int32_t (*tUcoHandler)(struct USBCMDObject* uco);

#ifdef DEBUG
extern void dbgprintf(const char * format, ...);
#else
#define dbgprintf(x, ...) {}
#endif

#endif /* CONFIG_H_ */