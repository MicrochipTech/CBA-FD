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

This file contains the libusb connection handling

////////////////////////////////////////////////////////////////////////////////*/

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
#include <assert.h>

#ifndef sprintf_s
#define sprintf_s(buf, size, ...) snprintf(buf, size, __VA_ARGS__)
#endif

namespace aba::usb
{
	uint8_t instanceCtr = 0;
    mba_device_t* aba_device_list = nullptr;

    /* The deviceMapping map is only for tracking the order in which devices have been opened.
     * Every time a MBA hardware instance is opened, it will get assigned a unique ID for this session (lifetime of the loaded driver library).
     * Closing and opening or reconnecting will assign each MBA hardware the same ID as when it was opened the first time.
     * Therefore this mapping stays the same even if devices are disconnected, it's not allowed to remove entries during runtime.
     */
	auto handleCompare = [](const mba_serial_t& ah1, const mba_serial_t& ah2) {
		auto ret = memcmp(&ah1.octet, &ah2, sizeof(mba_serial_t));
		if (ret < 0)
			ret = 0;
		return ret;
	};
	std::map<mba_serial_t, uint8_t, decltype(handleCompare)> deviceMapping(handleCompare);
    

	/* Local function definitions */
    void usbThread(mba_handle_t* aba_handle);
    int hotplug_callback(libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data);

    /* Functions */
	int32_t closeDevice(mba_handle_t* mba)
	{
        if (nullptr != mba) {
            delete mba;
            return E_OK;
        }
        return E_ERR;
	}

	void usbThread(mba_handle_t* mba)
	{
		timeval tv;
		tv.tv_usec = 100 * 1000u;
		tv.tv_sec = 0;
		int completed = 0;

		while (mba->m_runThread)
		{
			auto ret = libusb_handle_events_timeout_completed(mba->m_usbCtx, &tv, &completed);
		}
	}

	void unload()
	{
		if (aba_device_list)
		{
			delete[]aba_device_list;
			aba_device_list = nullptr;
		}
	}

	int hotplug_callback(libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data)
	{
		
		return 0;
	}


	int32_t transferSubmit(mba_handle_t* aba_device, USBCMDObject* uco)
	{
		std::unique_lock<std::mutex> lock(*aba_device->m_usbTxMutex);

		int32_t transfered = 0;
		int32_t length = (sizeof(uco->header) + uco->header.payloadlength);
		int rc = libusb_bulk_transfer(aba_device->m_usbHandle, 0x01u, reinterpret_cast<uint8_t*>(uco), length, &transfered, 2000);
		delete uco;

		if (rc == 0 && length == transfered)
			return E_OK;

		return E_ERR;
	}

    int32_t transferSubmit(mba_handle_t* aba_device, std::string buffer)
    {
        std::unique_lock<std::mutex> lock(*aba_device->m_usbTxMutex);

        int32_t transfered = 0;
        int32_t length = static_cast<int32_t>(buffer.length());
        int rc = libusb_bulk_transfer(aba_device->m_usbHandle, 0x01u, reinterpret_cast<uint8_t*>(buffer.data()), length, &transfered, 2000);

        if (rc == 0 && length == transfered)
            return E_OK;

        return E_ERR;
    }

	static void USB_ProcessError(libusb_transfer_status status, AbaMessage** msg)
	{
		uint32_t usb_status = static_cast<uint32_t>(status);
		*msg = new AbaMessage(UCO_API, MBA_USBERROR, &usb_status, sizeof(usb_status));
	}

	static void LIBUSB_CALL cb_dataReceived(struct libusb_transfer* xfr)
	{
		mba_handle_t* aba_handle = reinterpret_cast<mba_handle_t*>(xfr->user_data);
		uint8_t* pRdBuffer = xfr->buffer;
		uint32_t elementsPerTransfer = 0;

		if (xfr->status == LIBUSB_TRANSFER_COMPLETED)
		{
			if (/*aba_handle->m_userCb &&*/pRdBuffer)
			{
				int32_t bytesLeft = xfr->actual_length;
				while (bytesLeft > 0)
				{
					uint8_t command = 0;
					AbaMessage* msg = 0;
					USBCMDObject* uco = (USBCMDObject*)pRdBuffer;
					switch (uco->header.type)
					{
					case UCO_API:
					{
						if (API_ProcessMessage(uco, &msg)) {
							invoke_callback(aba_handle, msg);
						}
						break;
					}
					case UCO_CAN:
					{
						if (CAN_ProcessMessage(uco, &msg)) {
							invoke_callback(aba_handle, msg);
						}
						break;
					}
					default:
						break;
					}

					elementsPerTransfer++;
					bytesLeft -= sizeof(uco->header) + uco->header.payloadlength;
					pRdBuffer += sizeof(uco->header) + uco->header.payloadlength;
				}

				//if(xfr->actual_length)
				//	printf("Received %lu elements, total size %lu\r\n", elementsPerTransfer, xfr->actual_length);

				libusb_free_transfer(xfr);
				auto nxfr = libusb_alloc_transfer(0);
				aba_handle->m_xfer = nxfr;
				libusb_fill_bulk_transfer(nxfr, aba_handle->m_usbHandle, 0x82, aba_handle->m_rxBuffer, sizeof(aba_handle->m_rxBuffer), cb_dataReceived, aba_handle, 2000);
				libusb_submit_transfer(nxfr);
			}
		}
		else if (xfr->status == LIBUSB_TRANSFER_TIMED_OUT)
		{
			libusb_free_transfer(xfr);
			auto nxfr = libusb_alloc_transfer(0);
			aba_handle->m_xfer = nxfr;
			libusb_fill_bulk_transfer(nxfr, aba_handle->m_usbHandle, 0x82, aba_handle->m_rxBuffer, sizeof(aba_handle->m_rxBuffer), cb_dataReceived, aba_handle, 2000);
			libusb_submit_transfer(nxfr);
		}
		else
		{
			// Send control message via callback and terminate?...
			AbaMessage* msg = 0;
			USB_ProcessError(xfr->status, &msg);
			invoke_callback(aba_handle, msg);
		}
	}

	uint8_t startReceive(mba_handle_t* aba_handle)
	{
		uint8_t ret = E_INIT;
		
		if (aba_handle->m_usbThread != nullptr) {
			return ret;
		}
		aba_handle->m_runThread = 1;
		aba_handle->m_usbThread = new std::thread(usbThread, aba_handle);

		auto xIn = libusb_alloc_transfer(0);
		aba_handle->m_xfer = xIn;

		libusb_fill_bulk_transfer(xIn, aba_handle->m_usbHandle, 0x82, aba_handle->m_rxBuffer, sizeof(aba_handle->m_rxBuffer), cb_dataReceived, aba_handle, 2000);

        assert(xIn->dev_handle != 0);
        assert(aba_handle->m_usbHandle != 0);

		if (LIBUSB_SUCCESS == libusb_submit_transfer(xIn)) {
			ret = E_OK;
		}
		return ret;
	}

    int32_t reconnectDevice(mba_handle_t* mba_handle)
    {
        // Todo: Requires mutex
        if (nullptr == mba_handle) {
            return LIBUSB_ERROR_NO_DEVICE;
        }

        int rc = LIBUSB_ERROR_NOT_FOUND;
        mba_serial_t tmpSerial;

        if (nullptr != mba_handle->m_usbThread)
        {
            mba_handle->m_runThread = false;
            mba_handle->m_usbThread->join();
            delete mba_handle->m_usbThread;
            mba_handle->m_usbThread = nullptr;
        }

        // Release interface may fail as the device has disconnected
        if (mba_handle->m_usbHandle != nullptr) {
            libusb_release_interface(mba_handle->m_usbHandle, 0x0);
            libusb_close(mba_handle->m_usbHandle);
            mba_handle->m_usbHandle = nullptr;
        }

        /* Enumerate all connected USB devices */
        libusb_device** libusb_device_list = NULL;
        ssize_t usb_count = libusb_get_device_list(mba_handle->m_usbCtx, &libusb_device_list);

        for (ssize_t i = 0; i < usb_count; i++)
        {
            libusb_device_descriptor desc;
            libusb_get_device_descriptor(libusb_device_list[i], &desc);

            if (USB_VENDOR_ID == desc.idVendor && (PID_ABA == desc.idProduct || PID_CBA == desc.idProduct))
            {
                if (LIBUSB_SUCCESS == libusb_open(libusb_device_list[i], &mba_handle->m_usbHandle))
                {
                    memset(&tmpSerial, 0, sizeof(mba_serial_t));
                    libusb_get_string_descriptor_ascii(mba_handle->m_usbHandle, desc.iSerialNumber, (unsigned char*)tmpSerial.octet, sizeof(mba_serial_t));

                    if (0 == memcmp(&tmpSerial, &mba_handle->m_serial, sizeof(mba_serial_t)))
                    {
                        // Claiming the interface must work however
                        rc = libusb_claim_interface(mba_handle->m_usbHandle, 0x0);
                        if (rc < E_OK) {
                            libusb_close(mba_handle->m_usbHandle);
                            break;
                        }
                        else {
                            mba_handle->m_connection_status = CON_STATUS::CONNECTED;
                            break;
                        }
                    }
                }
            }
        }
        
        return (rc == E_OK) ? mba_handle->m_instanceId : rc;
    }

	int32_t openDevice(mba_handle_t** aba_handle, const mba_serial_t& serial)
	{
        // Todo: Requires mutex now
		int32_t rc = E_NODEV;
        mba_serial_t tmpSerial;

        /* Check if this device has already been opened */
        if(nullptr != getDeviceHandle(serial))
        {
            return E_DEV;
        }

		/* Allocate a new mba_handle_t */
		mba_handle_t* newHandle = new mba_handle_t();
		if (nullptr == newHandle) {
			return E_ERR;
		}

		/* Initialize libusb */
		if (libusb_init(&newHandle->m_usbCtx) != LIBUSB_SUCCESS) {
			delete newHandle;
			return E_ERR;
		}

		/* Enumerate all connected USB devices */
		libusb_device** libusb_device_list = NULL;
		libusb_device_handle** dev_handle = &newHandle->m_usbHandle;
		ssize_t usb_count = libusb_get_device_list(newHandle->m_usbCtx, &libusb_device_list);

		for (ssize_t i = 0; i < usb_count; i++)
		{
			libusb_device_descriptor desc;
			libusb_get_device_descriptor(libusb_device_list[i], &desc);

			if (USB_VENDOR_ID == desc.idVendor && (PID_ABA == desc.idProduct || PID_CBA == desc.idProduct))
			{
				/* Query and compare serial number */
				if (LIBUSB_SUCCESS == libusb_open(libusb_device_list[i], dev_handle))
				{
					switch (desc.idProduct)
					{
					case PID_ABA:
						newHandle->m_deviceType = DEVICE_ABA;
#ifdef TEMP_CBA_FIX                        
                        if (desc.bcdDevice == 0x1)
                            newHandle->m_deviceType = DEVICE_CBA;
#endif
                        break;
					case PID_CBA:
						newHandle->m_deviceType = DEVICE_CBA;
						break;
					}

					memset(&tmpSerial, 0, sizeof(mba_serial_t));
					libusb_get_string_descriptor_ascii(*dev_handle, desc.iSerialNumber, (unsigned char*)tmpSerial.octet, sizeof(mba_serial_t));

					if (0 == strcmp(tmpSerial.octet, serial.octet))
					{
						rc = libusb_claim_interface(*dev_handle, 0x0);
						if (rc < E_OK) {
							libusb_close(*dev_handle);
                            break;
						}
						else
						{
							/* Insert the device into the static mapping list, so it will retain its number on open/close calls */
							auto mappedId = deviceMapping.find(tmpSerial);
							if (mappedId == deviceMapping.end())
							{
								deviceMapping.insert({ tmpSerial, instanceCtr });
								newHandle->m_instanceId = instanceCtr;
								instanceCtr++;
							}
							else {
								newHandle->m_instanceId = mappedId->second;
							}

							newHandle->m_serial = tmpSerial;
                            newHandle->m_connection_status = CON_STATUS::CONNECTED;
							rc = E_OK;
						}
						break;
					}
				}
			}
		}

		libusb_free_device_list(libusb_device_list, 1);

		if (rc < 0) {
			delete newHandle;
		}
		else {
			*aba_handle = newHandle;
		}

		return (rc == E_OK) ? newHandle->m_instanceId : rc;
	}

	uint32_t enumDevices(mba_device_t** device_list)
	{
		size_t aba_count = 0;
		libusb_context* ctx;
		libusb_device** libusb_device_list = NULL;

		if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
			return E_ERR;
		}

		ssize_t usb_count = libusb_get_device_list(ctx, &libusb_device_list);

		/* Enumerate all connected USB devices */
		for (ssize_t i = 0; i < usb_count; i++)
		{
			libusb_device_descriptor desc;
			libusb_get_device_descriptor(libusb_device_list[i], &desc);

			if (USB_VENDOR_ID == desc.idVendor && (PID_ABA == desc.idProduct || PID_CBA == desc.idProduct))
			{
				/* If Vendor and Product ID match, increase device counter */
				aba_count++;
			}
		}

		/* Valid devices are connected */
		if (aba_count > 0)
		{
			if (aba_device_list)
			{
				/* If a list was populated previously clear it */
				delete[]aba_device_list;
				aba_device_list = nullptr;
			}

			/* Create new list */
			aba_device_list = new mba_device_t[aba_count + 1];
			if (nullptr == aba_device_list) {
				usb_count = 0;
				aba_count = 0;
			}
			memset(aba_device_list, 0, sizeof(mba_device_t) * (aba_count + 1));

			uint32_t ctr = 0;
			for (uint32_t i = 0; i < usb_count; i++)
			{
				auto device = libusb_device_list[i];
				libusb_device_descriptor desc;

				libusb_get_device_descriptor(device, &desc);

				if (USB_VENDOR_ID == desc.idVendor && (PID_ABA == desc.idProduct || PID_CBA == desc.idProduct))
				{
					/* Add a device to the list */
					switch (desc.idProduct)
					{
					case PID_ABA:
						aba_device_list[ctr].deviceType = DEVICE_ABA;
						break;
					case PID_CBA:
						aba_device_list[ctr].deviceType = DEVICE_CBA;
						break;
					}
					memset(&aba_device_list[ctr].serial, 0, sizeof(mba_serial_t));

					/* Query serial number */
                    libusb_device_handle* dev_handle = nullptr;
                    int rc = libusb_open(libusb_device_list[i], &dev_handle);
					if (LIBUSB_SUCCESS == rc)
					{
                        rc = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, (unsigned char*)aba_device_list[ctr].serial.octet, sizeof(mba_serial_t));
                        if (LIBUSB_SUCCESS > rc)
                        {
                            aba_device_list[ctr].deviceType = DEVICE_INVALID;
                            sprintf_s(aba_device_list[ctr].serial.octet, sizeof(mba_serial_t), "Error: failed to read serial from device (%d)", rc);
                        }
                        libusb_close(dev_handle);
					}
                    /* If either open device or query descriptor failed return DEVICE_INVALID */
                    else
                    {
                        aba_device_list[ctr].deviceType = DEVICE_INVALID;
                        sprintf_s(aba_device_list[ctr].serial.octet, sizeof(mba_serial_t), "Error: libusb_open failed (%d)", rc);
                    }

					ctr++;
				}
			}

			*device_list = aba_device_list;
		}

		libusb_free_device_list(libusb_device_list, 1);
		libusb_exit(ctx);

		return static_cast<uint32_t>(aba_count);
	}
}
