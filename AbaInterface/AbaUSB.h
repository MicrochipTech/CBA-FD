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

#define USB_VENDOR_ID 0x04D8u

#define PID_ABA 0x0AB1u
#define PID_CBA 0x0AB2u

#include "MbaInterface.h"
#include "USB_Commands.h"
#include <string>

namespace aba::usb
{
    enum USBEVENT
    {
        CONNECT,
        DISCONNECT
    };

	void unload();
	int32_t transferSubmit(mba_handle_t* aba_device, USBCMDObject* uco);
    int32_t transferSubmit(mba_handle_t* aba_device, std::string buffer);
	uint32_t enumDevices(mba_device_t** device_list);
	int32_t openDevice(mba_handle_t** aba_handle, const mba_serial_t& serial);
	int32_t closeDevice(mba_handle_t* aba_handle);
    int32_t reconnectDevice(mba_handle_t* mba);
	uint8_t startReceive(mba_handle_t* aba_handle);
}
