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

#include "MbaInterface.h"

namespace aba::can
{
	int32_t sendFrame(mba_handle_t* aba_device, CAN_BUS busId, uint32_t canId, const uint8_t* payload, uint8_t dlc, uint8_t flags, uint16_t timeout);
    int32_t sendFrameBulk(mba_handle_t* aba_device, const CanFrame* frame_ptr, uint32_t frame_cnt);
    int32_t setBitTiming(mba_handle_t* mba_device, CAN_BUS busId, const CanBitrate* bitrate);
    int32_t getBitTiming(mba_handle_t* mba_device, CAN_BUS busId);
    int32_t setSpeed(mba_handle_t* aba_device, CAN_BUS busId, uint32_t canSpeed, uint32_t canFdSpeed);
	int32_t setMode(mba_handle_t* aba_device, CAN_BUS busId, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled);
    int32_t getMode(mba_handle_t* mba_device, CAN_BUS busId);
	int32_t reset(mba_handle_t* aba_device, CAN_BUS busId);
	int32_t dbgReadReg(mba_handle_t* aba_device, uint32_t address);
    int32_t phyMaintain(mba_handle_t* device, CAN_BUS busId, uint32_t enable);
}
