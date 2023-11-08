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

/*////////////////////////////////////////////////////////////////////////////////

This file contains the USB part of the CAN connection

////////////////////////////////////////////////////////////////////////////////*/

#include <libusb.h>

#include "AbaHandle.h"
#include "CanInterface.h"
#include "AbaUSB.h"
#include "AbaCAN.h"

namespace aba::can
{
    static const BitTimeSettings_t canSettings[CAN_SPEED_MAX] =
    {
        {
            /* bitrate   = */ 100u,
            /* preScaler = */ 5u,
            /* phase1Seg = */ 63u,
            /* phase2Seg = */ 16u,
            /* sjw       = */ 16u,
            /* tdc       = */ 0u
        },
        {
            /* bitrate   = */ 125u,
            /* preScaler = */ 4u,
            /* phase1Seg = */ 63u,
            /* phase2Seg = */ 16u,
            /* sjw       = */ 16u,
            /* tdc       = */ 0u
        },
        {
            /* bitrate   = */ 250u,
            /* preScaler = */ 2u,
            /* phase1Seg = */ 63u,
            /* phase2Set = */ 16u,
            /* sjw       = */ 16u,
            /* tdc       = */ 0u
        },
        {
            /* bitrate   = */ 500u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 63u,
            /* phase2Set = */ 16u,
            /* sjw       = */ 16u,
            /* tdc       = */ 0u
        },
        {
            /* bitrate   = */ 1000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 31u,
            /* phase2Set = */ 8u,
            /* sjw       = */ 8u,
            /* tdc       = */ 0u
        },
        {
            /* bitrate   = */ 0u,
            /* preScaler = */ 0u,
            /* phase1Seg = */ 0u,
            /* phase2Set = */ 0u,
            /* sjw       = */ 0u,
            /* tdc       = */ 0u
        }
    };

    static const BitTimeSettings_t canFdSettings[CANFD_SPEED_MAX] =
    {
        {
            /* bitrate   = */ 1000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 31u,
            /* phase2Set = */ 8u,
            /* sjw       = */ 8u,
            /* tdc       = */ 32u
        },
        {
            /* bitrate   = */ 2000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 15u,
            /* phase2Set = */ 4u,
            /* sjw       = */ 4u,
            /* tdc       = */ 16u
        },
        {
            /* bitrate   = */ 3077u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 9u,
            /* phase2Set = */ 3u,
            /* sjw       = */ 3u,
            /* tdc       = */ 10u
        },
        {
            /* bitrate   = */ 4000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 7u,
            /* phase2Set = */ 2u,
            /* sjw       = */ 2u,
            /* tdc       = */ 8u
        },
        {
            /* bitrate   = */ 5000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 5u,
            /* phase2Set = */ 2u,
            /* sjw       = */ 2u,
            /* tdc       = */ 6u
        },
        {
            /* bitrate   = */ 6667u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 4u,
            /* phase2Set = */ 1u,
            /* sjw       = */ 1u,
            /* tdc       = */ 5u
        },
        {
            /* bitrate   = */ 8000u,
            /* preScaler = */ 1u,
            /* phase1Seg = */ 3u,
            /* phase2Set = */ 1u,
            /* sjw       = */ 1u,
            /* tdc       = */ 4u
        },
        {
            /* bitrate   = */ 0u,
            /* preScaler = */ 0u,
            /* phase1Seg = */ 0u,
            /* phase2Set = */ 0u,
            /* sjw       = */ 0u,
            /* tdc       = */ 0u
        }
    };

	int32_t sendFrame(mba_handle_t* aba_device, CAN_BUS busId, uint32_t canId, uint8_t* payload, uint8_t dlc, uint8_t flags, uint16_t timeout)
	{
		int32_t ret = E_ERR;
		USBCMDObject* uco = new USBCMDObject();
		if (CAN_ProcessTxMessage(uco, busId, payload, canId, dlc, flags, timeout))
		{
			ret = aba::usb::transferSubmit(aba_device, uco);
		}
		return ret;
	}

    int32_t sendFrameBulk(mba_handle_t* aba_device, CanFrame* frame_ptr, uint32_t frame_cnt)
    {
        if (frame_cnt > 32) {
            return E_ARG;
        }

        std::string buf;
        uint32_t ctr = 0;
        while (ctr < frame_cnt)
        {
            USBCMDObject* uco = new USBCMDObject();
            CAN_ProcessTxMessage(uco, &frame_ptr[ctr++]);
            buf.append((const char*)uco, (sizeof(USBCMDHeader) + uco->header.payloadlength));
            delete uco;
        }

        return aba::usb::transferSubmit(aba_device, buf);
    }

    int32_t setBitTiming(mba_handle_t* mba_device, CAN_BUS busId, CanBitrate* bitrate)
    {
        int32_t ret = E_ERR;
        USBCMDObject* uco = new USBCMDObject();

        if (CAN_SetBitTiming(uco, busId, bitrate))
        {
            if (busId < CAN_BUS_MAX)
            {
                mba_device->m_canBitrate[busId] = *bitrate;
            }
            ret = aba::usb::transferSubmit(mba_device, uco);
        }
        return ret;
    }

    int32_t getBitTiming(mba_handle_t* mba_device, CAN_BUS busId)
    {
        int32_t ret = E_ERR;
        USBCMDObject* uco = new USBCMDObject();

        if (CAN_GetBitTiming(uco, busId))
        {
            ret = aba::usb::transferSubmit(mba_device, uco);
        }
        return ret;
    }

	int32_t setSpeed(mba_handle_t* aba_device, CAN_BUS busId, uint32_t canSpeed, uint32_t canFdSpeed)
	{
		canSpeeds _canSpeed;
		canFdSpeeds _canFdSpeed;
        CanBitrate bitrate;

		switch (canSpeed)
		{
		case 100:  _canSpeed = canSpeeds::CAN_SPEED_100;  break;
		case 125:  _canSpeed = canSpeeds::CAN_SPEED_125;  break;
		case 250:  _canSpeed = canSpeeds::CAN_SPEED_250;  break;
		case 500:
        default:   _canSpeed = canSpeeds::CAN_SPEED_500;  break;
		case 1000: _canSpeed = canSpeeds::CAN_SPEED_1000; break;
		}
		
		switch (canFdSpeed)
		{
		case 1000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_1000; break;
		case 2000:
        default:   _canFdSpeed = canFdSpeeds::CANFD_SPEED_2000; break;
		case 3000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_3077; break;
		case 4000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_4000; break;
		case 5000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_5000; break;
		case 6000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_6667; break;
		case 8000: _canFdSpeed = canFdSpeeds::CANFD_SPEED_8000; break;
		}

        /* CAN2.0 Settings */
        bitrate.nseg1  = canSettings[_canSpeed].phase1Seg;
        bitrate.nseg2  = canSettings[_canSpeed].phase2Seg;
        bitrate.nsjw   = canSettings[_canSpeed].sjw;
        bitrate.nscale = canSettings[_canSpeed].preScaler;
        
        /* CAN-FD Settings */
        bitrate.dseg1  = canFdSettings[_canFdSpeed].phase1Seg;
        bitrate.dseg2  = canFdSettings[_canFdSpeed].phase2Seg;
        bitrate.dsjw   = canFdSettings[_canFdSpeed].sjw;
        bitrate.dscale = canFdSettings[_canFdSpeed].preScaler;
        bitrate.tdc    = (canFdSettings[_canFdSpeed].tdc != 0u) ? 1u : 0u;
        bitrate.tdco   = canFdSettings[_canFdSpeed].tdc;
        bitrate.tdcf   = 0u;

        return setBitTiming(aba_device, busId, &bitrate);
	}

	int32_t setMode(mba_handle_t* mba, CAN_BUS busId, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled)
	{
		int32_t ret = E_ERR;
		USBCMDObject* uco = new USBCMDObject();
		if (CAN_SetMode(uco, busId, canMode, testMode, autoRetryEnabled))
		{
            if (busId < CAN_BUS_MAX)
            {
                mba->m_canMode[busId].mode = canMode;
                mba->m_canMode[busId].testMode = testMode;
                mba->m_canMode[busId].retryEnabled = autoRetryEnabled;
            }
			ret = aba::usb::transferSubmit(mba, uco);
		}
		return ret;
	}

    int32_t getMode(mba_handle_t* mba_device, CAN_BUS busId)
    {
        int32_t ret = E_ERR;
        USBCMDObject* uco = new USBCMDObject();
        if (CAN_GetMode(uco, busId))
        {
            ret = aba::usb::transferSubmit(mba_device, uco);
        }
        return ret;
    }

	int32_t reset(mba_handle_t* aba_device, CAN_BUS busId)
	{
		int32_t ret = E_ERR;
		USBCMDObject* uco = new USBCMDObject();
		if (CAN_Reset(uco, busId))
		{
			ret = aba::usb::transferSubmit(aba_device, uco);
		}
		return ret;
	}

	int32_t dbgReadReg(mba_handle_t* aba_device, uint32_t address)
	{
		int32_t ret = E_ERR;
		USBCMDObject* uco = new USBCMDObject();
		if (CAN_DbgReadReg(uco, address))
		{
			ret = aba::usb::transferSubmit(aba_device, uco);
		}
		return ret;
	}

    int32_t phyMaintain(mba_handle_t* device, CAN_BUS busId, uint32_t enable)
    {
        int32_t ret = E_ERR;
        USBCMDObject* uco = new USBCMDObject();
        if (CAN_PhyMaintain(uco, busId, enable))
        {
            ret = aba::usb::transferSubmit(device, uco);
        }
        return ret;
    }
}
