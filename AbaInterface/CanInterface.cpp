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

This file contains the CAN specific protobuf code

////////////////////////////////////////////////////////////////////////////////*/

#include <assert.h>
#include "CanInterface.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "aba.can.pb.h"

bool encodeOutputStream(USBCMDObject* uco, const pb_msgdesc_t* fields, const void* src_struct)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(uco->data, sizeof(uco->data));
    bool status = pb_encode(&ostream, fields, src_struct);
    uco->header.payloadlength = static_cast<uint16_t>(ostream.bytes_written);
    return status;
}

template<aba_can_eCanCommands CMD>
inline bool CAN_SimpleRequest(USBCMDObject* uco, CAN_BUS canBus, uint32_t flag = 0)
{
    assert(uco != nullptr);

    uco->header.type = UCO_CAN;
    uco->header.command = CMD;

    aba_can_CanDeviceWrapper cs;
    cs.device = (canBus == CAN_BUS_0) ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;
    cs.flag = flag;

    return encodeOutputStream(uco, aba_can_CanDeviceWrapper_fields, &cs);
}

bool CAN_GetBitTiming(USBCMDObject* uco, CAN_BUS canBus)
{
    return CAN_SimpleRequest<aba_can_eCanCommands_GET_BITRATE>(uco, canBus);
}

bool CAN_GetMode(USBCMDObject* uco, CAN_BUS canBus)
{
    return CAN_SimpleRequest<aba_can_eCanCommands_GET_MODE>(uco, canBus);
}

bool CAN_GetErrorCounters(USBCMDObject* uco, CAN_BUS canBus)
{
    return CAN_SimpleRequest<aba_can_eCanCommands_GET_ERRORCNT>(uco, canBus);
}

bool CAN_Reset(USBCMDObject* uco, CAN_BUS canBus)
{
    return CAN_SimpleRequest<aba_can_eCanCommands_RESET>(uco, canBus);
}

bool CAN_PhyMaintain(USBCMDObject* uco, CAN_BUS canBus, uint32_t enable)
{
    return CAN_SimpleRequest<aba_can_eCanCommands_PHY_MAINTAIN>(uco, canBus, enable);
}

bool CAN_SetMode(USBCMDObject* uco, CAN_BUS canBus, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled)
{
	uco->header.type = UCO_CAN;
	uco->header.command = aba_can_eCanCommands_SET_MODE;

	aba_can_CanMode cs;
	cs.device = (canBus == CAN_BUS_0) ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;
	cs.autoRetryEnabled = autoRetryEnabled;

	switch (canMode)
	{
	case CAN_MODE_CLASSIC:
		cs.mode = aba_can_eCanMode_MODE_CLASSIC;
		break;
	case CAN_MODE_FDNISO:
		cs.mode = aba_can_eCanMode_MODE_FDNISO;
		break;
	case CAN_MODE_FDISO:
	default:
		cs.mode = aba_can_eCanMode_MODE_FDISO;
		break;
	}

	switch (testMode)
	{
	case CAN_TESTMODE_NORMAL:
	default:
		cs.specialMode = aba_can_eCanSpecialMode_MODE_NORMAL;
		break;
	case CAN_TESTMODE_LISTENONLY:
		cs.specialMode = aba_can_eCanSpecialMode_MODE_LISTEN;
		break;
	case CAN_TESTMODE_LOOPBACK:
		cs.specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_INT;
		break;
	case CAN_TESTMODE_EXTLOOPBACK:
		cs.specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_EXT;
		break;
	}

    return encodeOutputStream(uco, aba_can_CanMode_fields, &cs);
}

bool CAN_SetBitTiming(USBCMDObject* uco, CAN_BUS canBus, CanBitrate* bitrate)
{
    assert(uco != nullptr);

    uco->header.type = UCO_CAN;
    uco->header.command = aba_can_eCanCommands_SET_BITRATE;

    aba_can_CanSpeed cs;
    cs.device = (canBus == CAN_BUS_0) ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;

    /* CAN2.0 Settings */
    cs.nseg1  = bitrate->nseg1;
    cs.nseg2  = bitrate->nseg2;
    cs.nsjw   = bitrate->nsjw;
    cs.nscale = bitrate->nscale;

    cs.dseg1  = bitrate->dseg1;
    cs.dseg2  = bitrate->dseg2;
    cs.dsjw   = bitrate->dsjw;
    cs.dscale = bitrate->dscale;
    cs.tdc    = bitrate->tdc;
    cs.tdco   = bitrate->tdco;
    cs.tdcf   = bitrate->tdcf;

    return encodeOutputStream(uco, aba_can_CanSpeed_fields, &cs);
}

/**
 * Unpack a CAN frame received from the ABA
 * @param uco Pointer to an USBCMDObject which holds the received payload data.
 * @param frame Pointer to a CanFrame
 * @return True or False
 */
//bool CAN_ProcessMessage(const USBCMDObject* uco, uint8_t& command, uint8_t** outBuffer, uint32_t& bufferSize)
bool CAN_ProcessMessage(const USBCMDObject* uco, AbaMessage** msg)
{
	assert(uco != nullptr);
	assert(msg != nullptr);

	bool status = false;
	
	if (uco->header.command == aba_can_eCanCommands_RX_MSG ||
		uco->header.command == aba_can_eCanCommands_TX_MSG)
	{
		aba_can_CanMessage cs;
		pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
		status = pb_decode(&istream, aba_can_CanMessage_fields, &cs);

		if (status)
		{
			uint8_t command = (uco->header.command == aba_can_eCanCommands_RX_MSG ? CAN_MSG_RX : CAN_MSG_TX);
			*msg = new AbaMessage(UCO_CAN, command, 0, sizeof(CanFrame));
			CanFrame* frame = reinterpret_cast<CanFrame*>((*msg)->message.data);
			frame->busId = cs.device;
			frame->dlc = cs.dlc;
			frame->id = cs.canId;
			frame->timestamp = cs.timestamp;

			uint32_t flags = CANFRAME_FLAG_NONE;
			switch (cs.frameType)
			{
			case aba_can_eCanFrameType_TYPE_FDBRS_XTD:
				flags |= CANFRAME_FLAG_BRS;
			case aba_can_eCanFrameType_TYPE_FD_XTD:
				flags |= CANFRAME_FLAG_FD;
			case aba_can_eCanFrameType_TYPE_CLASSIC_XTD:
				flags |= CANFRAME_FLAG_EXTENDED;
				break;

			case aba_can_eCanFrameType_TYPE_FDBRS_STD:
				flags |= CANFRAME_FLAG_BRS;
			case aba_can_eCanFrameType_TYPE_FD_STD:
				flags |= CANFRAME_FLAG_FD;
			case aba_can_eCanFrameType_TYPE_CLASSIC_STD:
			default:
				break;
			};
			frame->flags = flags;

			memcpy(frame->data, cs.message.bytes, cs.dlc);
		}
	}
	else if (uco->header.command == aba_can_eCanCommands_GET_ERRORCNT)
	{
		aba_can_CanError cs;
		pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
		status = pb_decode(&istream, aba_can_CanError_fields, &cs);

		if (status)
		{
			CanError err;
			err.busId = cs.device;
			err.busOff = cs.busOff;
			err.rec = cs.rec;
			err.tec = cs.tec;
			err.lec = (cs.lec >> 0) & 0x7;
			err.dlec = (cs.lec >> 8) & 0x7;
			err.timestamp = cs.timestamp;

			*msg = new AbaMessage(uco->header.type, CAN_MSG_ERR, &err, sizeof(err));
		}
	}
	else if (uco->header.command == aba_can_eCanCommands_DBG_READ_REG)
	{
		aba_can_DbgReadRegister cs;
		pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
		status = pb_decode(&istream, aba_can_DbgReadRegister_fields, &cs);

		if (status)
		{
			*msg = new AbaMessage(uco->header.type, CAN_MSG_DBG, &cs, sizeof(aba_can_DbgReadRegister));
		}
	}
    else if (uco->header.command == aba_can_eCanCommands_GET_BITRATE)
    {
        aba_can_CanSpeed cs;
        pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
        status = pb_decode(&istream, aba_can_CanSpeed_fields, &cs);

        if (status)
        {
            CanBitrate rate;
            rate.busId = (cs.device == aba_can_eCanDevice_MCAN0) ? CAN_BUS_0 : CAN_BUS_1;
            rate.dscale = cs.dscale;
            rate.dseg1 = cs.dseg1;
            rate.dseg2 = cs.dseg2;
            rate.dsjw = cs.dsjw;
            rate.nscale = cs.nscale;
            rate.nseg1 = cs.nseg1;
            rate.nseg2 = cs.nseg2;
            rate.nsjw = cs.nsjw;
            rate.tdc = cs.tdc;
            rate.tdcf = cs.tdcf;
            rate.tdco = cs.tdco;

            *msg = new AbaMessage(uco->header.type, CAN_MSG_BITRATE, &rate, sizeof(rate));
        }
    }
    else if (uco->header.command == aba_can_eCanCommands_GET_MODE)
    {
        aba_can_CanMode cs;
        pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
        status = pb_decode(&istream, aba_can_CanMode_fields, &cs);

        if (status)
        {
            CanMode mode;
            
            switch (cs.mode)
            {
            case aba_can_eCanMode_MODE_CLASSIC:
                mode.mode = CAN_MODE_CLASSIC;
                break;
            case aba_can_eCanMode_MODE_FDNISO:
                mode.mode = CAN_MODE_FDNISO;
                break;
            case aba_can_eCanMode_MODE_FDISO:
                mode.mode = CAN_MODE_FDISO;
                break;
            default:
                mode.mode = UINT8_MAX;
                break;
            }
            
            switch (cs.specialMode)
            {
            case aba_can_eCanSpecialMode_MODE_NORMAL:
                mode.testMode = CAN_TESTMODE_NORMAL;
                break;
            case aba_can_eCanSpecialMode_MODE_LISTEN:
                mode.testMode = CAN_TESTMODE_LISTENONLY;
                break;
            case aba_can_eCanSpecialMode_MODE_LOOPBACK_INT:
                mode.testMode = CAN_TESTMODE_LOOPBACK;
                break;
            case aba_can_eCanSpecialMode_MODE_LOOPBACK_EXT:
                mode.testMode = CAN_TESTMODE_EXTLOOPBACK;
                break;
            default:
                mode.testMode = UINT8_MAX;
                break;
            }

            mode.busId = (cs.device == aba_can_eCanDevice_MCAN0) ? CAN_BUS_0 : CAN_BUS_1;
            mode.retryEnabled = cs.autoRetryEnabled;

            *msg = new AbaMessage(uco->header.type, CAN_MSG_MODE, &mode, sizeof(mode));
        }
    }

	return status;
}

/**
 * Send a CAN frame to the ABA
 * @param uco Pointer to an USBCMDObject which will receive the payload data.
 * @param data The payload of the CAN frame.
 * @param canId The CAN frame ID.
 * @param dlc The length of the payload in range of 0-64 byte. It's the task of the ABA to convert to the correct protocol DLC value.
 * @param flags A combination of CANFRAME_FLAG
 * @see CANFRAME_FLAG
 * @return True or False
 */
bool CAN_ProcessTxMessage(USBCMDObject* uco, CAN_BUS canBus, const uint8_t* data, uint32_t canId, uint8_t dlc, uint8_t flags, uint16_t timeout)
{
	assert(uco != nullptr);

	uco->header.type = UCO_CAN;
	uco->header.command = aba_can_eCanCommands_TX_MSG;

	bool status = true;
	aba_can_CanMessage cs;
	cs.canId = canId;
	cs.dlc = dlc;
	cs.device = (canBus == CAN_BUS_0) ? aba_can_eCanDevice_MCAN0 : aba_can_eCanDevice_MCAN1;

	/* Process the flags and translate the enum */
	switch (flags)
	{
	case CANFRAME_FLAG_BRS | CANFRAME_FLAG_FD | CANFRAME_FLAG_EXTENDED:
		cs.frameType = aba_can_eCanFrameType_TYPE_FDBRS_XTD;
		break;
	case CANFRAME_FLAG_FD | CANFRAME_FLAG_EXTENDED:
		cs.frameType = aba_can_eCanFrameType_TYPE_FD_XTD;
		break;
	case CANFRAME_FLAG_EXTENDED:
		cs.frameType = aba_can_eCanFrameType_TYPE_CLASSIC_XTD;
		break;
	case CANFRAME_FLAG_BRS | CANFRAME_FLAG_FD:
		cs.frameType = aba_can_eCanFrameType_TYPE_FDBRS_STD;
		break;
	case CANFRAME_FLAG_FD:
		cs.frameType = aba_can_eCanFrameType_TYPE_FD_STD;
		break;
	case CANFRAME_FLAG_NONE:
		cs.frameType = aba_can_eCanFrameType_TYPE_CLASSIC_STD;
		break;
	default:
		status = false;
		break;
	}

	/* Check the DLC not exceeding 8 byte for CAN2.0 and 64 byte for CAN-FD */
	if ((dlc > 8u) && ((flags & CANFRAME_FLAG_FD) == 0x0)) {
		status = false;
	}
	if (dlc > 64u) {
		status = false;
	}

	/* Encode the message */
	if (status == true)
	{
		cs.timestamp = timeout;
		cs.message.size = dlc;
		memcpy(&cs.message.bytes[0], data, dlc);

        status = encodeOutputStream(uco, aba_can_CanMessage_fields, &cs);
	}

	return status;
}

bool CAN_ProcessTxMessage(USBCMDObject* uco, const CanFrame* frame)
{
	return CAN_ProcessTxMessage(uco, frame->busId ? CAN_BUS_1 : CAN_BUS_0, frame->data, frame->id, frame->dlc, frame->flags, static_cast<uint16_t>(frame->timestamp));
}

bool CAN_DbgReadReg(USBCMDObject* uco, uint32_t address)
{
	assert(uco != nullptr);

	uco->header.type = UCO_CAN;
	uco->header.command = aba_can_eCanCommands_DBG_READ_REG;

	aba_can_DbgReadRegister cs;
	cs.addr = address;
	cs.value = 0x55555555;

    return encodeOutputStream(uco, aba_can_DbgReadRegister_fields, &cs);
}
