/*
 * task_CAN.h
 *
 * Created: 19.02.2018 12:59:48
 *  Author: M43734
 */ 


#pragma once

#include "compiler.h"
#include "USB_Commands.h"

void init_can_task();
int32_t uco_CanHandler(struct USBCMDObject* uco);
void rtos_can_task();
uint64_t getCurrentTCTime();

struct canMsgWaiting_t
{
  union
  {
    uint32_t msgTotal;
    uint16_t msgInst[2];
    uint8_t msgQue[4];
  };
};

#define MCAN0RX 0
#define MCAN0TX 1
#define MCAN1RX 2
#define MCAN1TX 3

enum
{
  STATUS_RESET = 0,
  STATUS_INIT_DONE,
  STATUS_MINOR_ERROR,
  STATUS_BUS_OFF
};

