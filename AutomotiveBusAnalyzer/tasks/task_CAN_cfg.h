/*
 * task_CAN_cfg.h
 *
 * Created: 19.02.2018 12:59:48
 *  Author: M43734
 */ 


#pragma once

#include <stdint.h>
#include "aba.can.pb.h"

typedef struct 
{
  aba_can_CanSpeed sdInitCanSpeed[_aba_can_eCanDevice_ARRAYSIZE];
  aba_can_CanMode sdInitCanMode[_aba_can_eCanDevice_ARRAYSIZE];
} sdCanCfg_t;

/* Required for SD-Card initialization */
int32_t canSetBitrate(const aba_can_CanSpeed* ac);
int32_t canSendFrame(aba_can_CanMessage* ac);
int32_t canSetMode(const aba_can_CanMode* ac);
int32_t canReset(const aba_can_CanDeviceWrapper* ac);

