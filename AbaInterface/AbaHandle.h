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

#include <MbaInterface.h>
#include <thread>
#include <mutex>

struct libusb_context;
struct libusb_device_handle;
struct libusb_transfer;
struct libusb_device_descriptor;

enum CON_STATUS
{
    DISCONNECTED,
    CONNECTED
};

struct mba_handle_t
{
private:
	void shutdown();

public:
	mba_handle_t();
	~mba_handle_t();

	uint32_t m_instanceId;
	uint32_t m_deviceType;
	mba_serial_t m_serial;

	libusb_device_handle* m_usbHandle;
	libusb_context* m_usbCtx;
	libusb_transfer* m_xfer;

	std::thread* m_usbThread;
	std::mutex* m_usbTxMutex;
	tCallback m_userCb;
	void* m_userCbData;
	volatile uint32_t m_runThread;
    mba_statistics_t m_stats;
    CON_STATUS m_connection_status;

    uint8_t m_canBusCount;
	uint8_t m_cxpiBusCount;
    CanBitrate* m_canBitrate;
    CanMode* m_canMode;
 
	uint8_t m_rxBuffer[8192];
};

class AbaMessage
{
public:
	AbaMessage();
	AbaMessage(uint8_t _messageType, uint8_t _command, const void* _data, uint32_t _data_length);
	~AbaMessage();

	mba_message_t message;
};

mba_handle_t* getDeviceHandle(const mba_serial_t& serial);
void invoke_callback(mba_handle_t* mba, AbaMessage* msg);

/* When the library is used for the first time after loading, LibraryInit shall be called to
   initialize OS depending functions. LibraryInit is not multithreading safe. */
int LibraryInit();

/* When the library is unloaded gracefully, unload() shall be invoked which then calls LibraryUnload
   to free all OS depending functions. It may be executed from OS specific context with certain backdraws.
   LibraryUnload is not multithreading safe. */
int LibraryUnload();
