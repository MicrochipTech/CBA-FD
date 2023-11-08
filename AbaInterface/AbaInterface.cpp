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

This file contains the library exported functions

////////////////////////////////////////////////////////////////////////////////*/

#include "AbaHandle.h"
#include "AbaUSB.h"
#include "AbaCAN.h"
#include "AbaApi.h"
#include <mutex>
#include <deque>
#include <set>
#include <tuple>
#include <thread>
#include <condition_variable>
#include <list>
#include <cstring>

using namespace aba::usb;

std::set<mba_handle_t*> deviceHandles;
std::deque<AbaMessage*> messageQueue;
std::mutex initMutex;
std::mutex cbMutex;
tCallback userCallback = nullptr;
std::thread cbThread;
std::condition_variable dataAvail;
size_t callbackQueueLimit = 2000000;

mba_handle_t* getDeviceHandle(const mba_serial_t& serial)
{
    mba_handle_t* ret = nullptr;
    for (auto mba : deviceHandles)
    {
        if (0 == strcmp(mba->m_serial.octet, serial.octet))
        {
            ret = mba;
        }
    }
    return ret;
}

void invoke_callback(mba_handle_t* mba, AbaMessage* msg)
{
	std::lock_guard<std::mutex> lock(cbMutex);

	if (userCallback != nullptr)
	{
		if (messageQueue.size() < callbackQueueLimit)
		{
			mba->m_stats.messagesReceived++;
			msg->message.instance = mba->m_instanceId;
			messageQueue.push_back(msg);
			dataAvail.notify_one();
		}
		else
		{
			mba->m_stats.messagesDropped++;
		}
	}
}

void callback_thread(void* user_data)
{
	while (userCallback)
	{
		std::unique_lock<std::mutex> lock(cbMutex);
		dataAvail.wait(lock);

		tCallback cb = userCallback;

		while (messageQueue.size() > 0)
		{
			if (cb)
			{
				AbaMessage* element = messageQueue.front();
				messageQueue.pop_front();
				lock.unlock();
				cb(&element->message, user_data);
				delete element;
				lock.lock();
			}
		}
		//lock.unlock();
	}
}

#if 0
void reconnect_device(mba_serial_t serial)
{
    for (auto mba : deviceHandles)
    {
        if (0 == strcmp((const char*)mba->m_serial.octet, (const char*)serial.octet))
        {
            reconnectDevice(mba);
            break;
        }
    }
}
#endif

/**
 * Verify a mba_handle_t is valid
 * @param handle Pointer to a mba_handle_t
 * @param deviceType @ref MBA_DEVICE
 * @result E_OK if all parameters are valid
 * @result E_ARG if handle pointer or deviceType are invalid
 * @result E_DEV if the specified device does not support this operation
 */
int32_t verifyParameter(mba_handle_t* handle, uint32_t deviceType)
{
	if (nullptr == handle) {
		return E_ARG;
	}

	if (nullptr == handle->m_usbHandle) {
		return E_ARG;
	}

	if ((handle->m_deviceType & deviceType) == 0)
	{
		return E_DEV;
	}

    if (handle->m_connection_status != CON_STATUS::CONNECTED)
    {
        return E_DISC;
    }

	return E_OK;
}

extern "C"
{
	DLL_EXPORT void unload()
	{
		std::lock_guard<std::mutex> lock(initMutex);

        // Unload OS specific stuff
        LibraryUnload();

		cbMutex.lock();
		userCallback = nullptr;
		cbMutex.unlock();

		dataAvail.notify_one();
		if (cbThread.joinable()) {
			cbThread.join();
		}
			
		for(auto aba_device : deviceHandles) {
			aba::usb::closeDevice(aba_device);
		}
		deviceHandles.clear();
		messageQueue.clear();

		aba::usb::unload();
	}

	DLL_EXPORT int32_t enumDevices(mba_device_t** device_list)
	{
		if (nullptr == device_list) {
			return E_ARG;
		}

		return aba::usb::enumDevices(device_list);
	}

	DLL_EXPORT int32_t openDevice(mba_handle_t** aba_device, const mba_serial_t* serial)
	{
		std::lock_guard<std::mutex> lock(initMutex);

		if (nullptr == aba_device || nullptr == serial) {
			return E_ARG;
		}

        // Load OS specific stuff
        LibraryInit();

		int32_t ret = E_ERR;
		int32_t dev = aba::usb::openDevice(aba_device, *serial);

		if (dev >= E_OK)
		{
			deviceHandles.insert(*aba_device);
			ret = aba::usb::startReceive(*aba_device);
			if (ret == E_OK) {
				ret = dev;
			}
		}

		return ret;
	}

	DLL_EXPORT int32_t closeDevice(mba_handle_t* aba_device)
	{
		std::lock_guard<std::mutex> lock(initMutex);
		
		if (aba_device)
		{
			aba::usb::closeDevice(aba_device);
			deviceHandles.erase(aba_device);
			return E_OK;
		}
		
		return E_ARG;
	}

	DLL_EXPORT int32_t registerCallback(tCallback callback, void* user_data)
	{
		std::lock_guard<std::mutex> lock(initMutex);

		if (nullptr == callback) {
			return E_ARG;
		}

        if (userCallback != nullptr) {
            return E_DEV;
        }

		userCallback = callback;
		cbThread = std::thread(callback_thread, user_data);
		
		return E_OK;
	}

	DLL_EXPORT int32_t unregisterCallback()
	{
		std::lock_guard<std::mutex> lock(initMutex);
		userCallback = nullptr;

		dataAvail.notify_one();
		if (cbThread.joinable())
		cbThread.join();
		messageQueue.clear();
		
		return E_OK;
	}

	DLL_EXPORT int32_t MBA_Reset(mba_handle_t* mba_device, MBA_RESET resetMode)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::api::resetDevice(mba_device, resetMode);
	}

    DLL_EXPORT int32_t MBA_GetVersion(mba_handle_t* mba_device)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::api::getFirmwareVersion(mba_device);
    };

    DLL_EXPORT int32_t MBA_Twinkle(mba_handle_t* mba_device)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::api::twinkle(mba_device);
    }

	DLL_EXPORT int32_t CAN_SendFrame(mba_handle_t* mba_device, CAN_BUS busId, uint32_t canId, uint8_t* payload, uint8_t dlc, uint8_t flags, uint16_t timeout)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::can::sendFrame(mba_device, busId, canId, payload, dlc, flags, timeout);
	}

    DLL_EXPORT int32_t CAN_SendFramesBulk(mba_handle_t* mba_device, CanFrame* frames, uint32_t count)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::can::sendFrameBulk(mba_device, frames, count);
    }

	DLL_EXPORT int32_t CAN_SetSpeed(mba_handle_t* mba_device, CAN_BUS busId, uint32_t canSpeed, uint32_t canFdSpeed)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::can::setSpeed(mba_device, busId, canSpeed, canFdSpeed);
	}

    DLL_EXPORT int32_t CAN_SetBitTiming(mba_handle_t* mba_device, CAN_BUS busId, CanBitrate* bitrate)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::can::setBitTiming(mba_device, busId, bitrate);
    }

    DLL_EXPORT int32_t CAN_GetBitTiming(mba_handle_t* mba_device, CAN_BUS busId)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::can::getBitTiming(mba_device, busId);
    }

	DLL_EXPORT int32_t CAN_SetMode(mba_handle_t* mba_device, CAN_BUS busId, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::can::setMode(mba_device, busId, canMode, testMode, autoRetryEnabled);
	}

    DLL_EXPORT int32_t CAN_GetMode(mba_handle_t* mba_device, CAN_BUS busId)
    {
        int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
        return (E_OK != rc) ? rc : aba::can::getMode(mba_device, busId);
    }

	DLL_EXPORT int32_t CAN_Reset(mba_handle_t* mba_device, CAN_BUS busId)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::can::reset(mba_device, busId);
	}

	DLL_EXPORT int32_t CAN_ReadRegister(mba_handle_t* mba_device, uint32_t address)
	{
		int32_t rc = verifyParameter(mba_device, DEVICE_ABA | DEVICE_CBA);
		return (E_OK != rc) ? rc : aba::can::dbgReadReg(mba_device, address);
	}
}
