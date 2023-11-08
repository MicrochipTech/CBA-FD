/*                                                                              */
/*                                                                              */
/* © 2016 Microchip Technology Inc. and its subsidiaries.                       */
/*                                                                              */
/* Subject to your compliance with these terms, you may use Microchip software  */
/* and any derivatives exclusively with Microchip products. It is your          */
/* responsibility to comply with third party license terms applicable to your   */
/* use of third party software (including open source software) that may        */
/* accompany Microchip software.                                                */
/*                                                                              */
/* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER       */
/* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED */
/* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A           */
/* PARTICULAR PURPOSE.                                                          */
/*                                                                              */
/* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,    */
/* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND        */
/* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS    */
/* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE       */
/* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN  */
/* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, */
/* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.                  */
/*                                                                              */
/*                                                                              */

#pragma once

#include <stdint.h>
#include "USB_Commands.h"
#include "AbaHandle.h"

typedef struct
{
	uint16_t      bitRate;
	uint8_t       preScaler;
	uint8_t       phase1Seg;
	uint8_t       phase2Seg;
	uint8_t       sjw;
	uint8_t       tdc;
} BitTimeSettings_t;

// Todo: Maybe move canSpeeds and canFdSpeeds to MbaInterface.h
enum canSpeeds
{
	CAN_SPEED_100,
	CAN_SPEED_125,
	CAN_SPEED_250,
	CAN_SPEED_500,
	CAN_SPEED_1000,
	CAN_SPEED_INVALID,
	CAN_SPEED_MAX
};

enum canFdSpeeds
{
	CANFD_SPEED_1000,
	CANFD_SPEED_2000,
	CANFD_SPEED_3077,
	CANFD_SPEED_4000,
	CANFD_SPEED_5000,
	CANFD_SPEED_6667,
	CANFD_SPEED_8000,
	CANFD_SPEED_INVALID,
	CANFD_SPEED_MAX,
};

bool CAN_SetBitTiming(USBCMDObject* uco, CAN_BUS canBus, CanBitrate* bitrate);
bool CAN_GetBitTiming(USBCMDObject* uco, CAN_BUS canBus);
bool CAN_SetSpeed(USBCMDObject* uco, CAN_BUS canBus, canSpeeds canSpeed, canFdSpeeds canFdSpeed);
bool CAN_SetMode(USBCMDObject* uco, CAN_BUS canBus, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled);
bool CAN_GetMode(USBCMDObject* uco, CAN_BUS canBus);
bool CAN_ProcessMessage(const USBCMDObject* uco, AbaMessage** msg);
bool CAN_ProcessTxMessage(USBCMDObject* uco, const CanFrame* frame);
bool CAN_ProcessTxMessage(USBCMDObject* uco, CAN_BUS canBus, const uint8_t* data, uint32_t canId, uint8_t dlc, uint8_t flags, uint16_t timeout);
bool CAN_Reset(USBCMDObject* uco, CAN_BUS canBus);
bool CAN_PhyMaintain(USBCMDObject* uco, CAN_BUS canBus, uint32_t enable);

/* Helper methods that may be removed */
bool CAN_ParseSpeed(const USBCMDObject* uco, uint8_t& canSpeed, uint8_t& canFdSpeed);
bool CAN_GetSpeed(struct USBCMDObject* uco, CAN_BUS canBus);
bool CAN_GetErrorCounters(USBCMDObject* uco);
bool CAN_DbgReadReg(USBCMDObject* uco, uint32_t address);
