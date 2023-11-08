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

#include <libusb.h>
#include <string.h>
#include <mutex>
#include "AbaUSB.h"
#include "AbaHandle.h"
#include "CanInterface.h"
#include "ApiInterface.h"
#include "USB_Commands.h"

#include <map>
#include <thread>
#include <stdio.h>

mba_handle_t::mba_handle_t()
{
	memset(this, 0, sizeof(*this));
	m_usbTxMutex = new std::mutex();
    m_connection_status = CON_STATUS::DISCONNECTED;
    
    // Todo: m_canBusCount is limited to CAN_BUS_MAX, but it should be queried from the hardware
    m_canBusCount = CAN_BUS_MAX;
    m_canBitrate = new CanBitrate[m_canBusCount];
    m_canMode = new CanMode[m_canBusCount];
}

mba_handle_t::~mba_handle_t()
{
	shutdown();

    delete[]m_canBitrate;
    delete[]m_canMode;
}

void mba_handle_t::shutdown()
{
	std::unique_lock<std::mutex> lock(*m_usbTxMutex);

    m_connection_status = CON_STATUS::DISCONNECTED;

	if (m_usbThread != nullptr)
	{
		m_runThread = 0;
		m_usbThread->join();
		delete m_usbThread;
		m_usbThread = nullptr;
	}

	if (m_usbCtx)
	{
		if (m_usbHandle != nullptr)
		{
			libusb_release_interface(m_usbHandle, 0);
			libusb_close(m_usbHandle);
            m_usbHandle = nullptr;
		}
		libusb_exit(m_usbCtx);
		m_usbCtx = nullptr;
	}

	lock.unlock();
	delete this->m_usbTxMutex;

    // m_xfer is freed by libusb
}

AbaMessage::AbaMessage()
{
	memset(&this->message, 0, sizeof(this->message));
}

AbaMessage::~AbaMessage()
{
	if (this->message.data != nullptr)
	{
		delete[] this->message.data;
	}
}

AbaMessage::AbaMessage(uint8_t _messageType, uint8_t _command, const void* _data, uint32_t _data_length)
{
    this->message.messageType = _messageType;
    this->message.command = _command;
    this->message.data_length = _data_length;
    this->message.data = new uint8_t[_data_length];
    if (_data != nullptr)
    {
        memcpy(this->message.data, _data, _data_length);
    }
    this->message.reserved = 0;
    this->message.instance = 0;
}

