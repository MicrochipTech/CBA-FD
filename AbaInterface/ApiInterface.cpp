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

This file contains the API specific protobuf code

////////////////////////////////////////////////////////////////////////////////*/

#include <assert.h>
#include "USB_Commands.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "aba.api.pb.h"
#include "ApiInterface.h"

bool API_ResetDevice(USBCMDObject* uco, MBA_RESET resetMode)
{
	uco->header.type = UCO_API;
	uco->header.command = (resetMode == MBA_RESET_BOOTLOADER) ? aba_api_eCommand_CMD_BOOTLOADER : aba_api_eCommand_CMD_RESET;
	uco->header.payloadlength = 0;

	return true;
}

bool API_GetFirmwareVersion(USBCMDObject* uco)
{
    uco->header.type = UCO_API;
    uco->header.command = aba_api_eCommand_CMD_GETVERSION;
    uco->header.payloadlength = 0;

    return true;
}

bool API_Twinkle(USBCMDObject* uco)
{
    uco->header.type = UCO_API;
    uco->header.command = aba_api_eCommand_CMD_TWINKLE;
    uco->header.payloadlength = 0;

    return true;
}

bool API_GetTimestamp(USBCMDObject* uco)
{
    uco->header.type = UCO_API;
    uco->header.command = aba_api_eCommand_CMD_TIMESTAMP;
    uco->header.payloadlength = 0;
    
    return true;
}

bool API_ProcessMessage(const USBCMDObject* uco, AbaMessage** msg)
{
	uint8_t type = 0;
    bool status = false;

	switch (uco->header.command)
	{
	case aba_api_eCommand_CMD_TIMESTAMP:
    {
        aba_api_Timestamp ts;
        pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
        status = pb_decode(&istream, aba_api_Timestamp_fields, &ts);
        if (status)
        {
            *msg = new AbaMessage(UCO_API, MBA_TIMESTAMP, &ts.timestamp, sizeof(ts.timestamp));
        }
        break;
    }
    case aba_api_eCommand_CMD_GETVERSION:
    {
        aba_api_Versions vs;
        pb_istream_t istream = pb_istream_from_buffer(uco->data, uco->header.payloadlength);
        status = pb_decode(&istream, aba_api_Versions_fields, &vs);
        if (status)
        {
            mba_version_t versions;
            versions.ver_api = vs.api;
            versions.ver_bootloader = vs.bootloader;
            versions.ver_firmware = vs.firmware;
            versions.ver_hardware = vs.hardware;
            versions.ver_hash = vs.hash;
            *msg = new AbaMessage(UCO_API, MBA_VERSION, &versions, sizeof(versions));
        }
        break;
    }
	default:
		type = MBA_RESERVED;
        *msg = new AbaMessage(UCO_API, uco->header.command, 0, 0);
        break;
	}
	
	return status;
}
