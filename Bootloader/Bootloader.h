/*
 * Bootloader.h
 *
 * Created: 10.10.2017 09:18:16
 *  Author: M43734
 */ 


#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include "compiler.h"
#include "Bldr_Protocol.h"


typedef enum _BSTATUS
{
  BS_IDLE,
  BS_ERROR,
  BS_TWINKLE,
  BS_ERASING,
  BS_FLASHING  
} BSTATUS;

void Bootloader(void);
void bootloaderStatusIndicator(BSTATUS status);

#endif /* BOOTLOADER_H_ */