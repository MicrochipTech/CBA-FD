#include <libusb.h>
#include <MbaInterface.h>
#include <AbaHandle.h>
#include <AbaUSB.h>
#include <AbaCAN.h>
#include <string.h>

int hotplug_callback(struct libusb_context* ctx, struct libusb_device* dev, libusb_hotplug_event event, void* user_data)
{
    mba_serial_t tmpSerial;
    struct libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev, &desc);

    memset(&tmpSerial, 0, sizeof(mba_serial_t));
    
    auto rc = libusb_open(dev, &dev_handle);

    libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, (unsigned char*)tmpSerial.octet, sizeof(mba_serial_t));

    libusb_close(dev_handle);

    auto mba_handle = getDeviceHandle(tmpSerial);
    return aba::usb::reconnectDevice(mba_handle);
}

int hotplug_windows(mba_serial_t& serial, aba::usb::USBEVENT event)
{
    auto mba = getDeviceHandle(serial);

    if (nullptr != mba)
    {
        if (event == aba::usb::CONNECT)
        {
            if (E_OK == aba::usb::reconnectDevice(mba))
            {
                aba::usb::startReceive(mba);

//WES                uint32_t i = 0;
//WES                for (i = 0; i < mba->m_canBusCount; i++)
//WES                {
//WES                    aba::can::setBitTiming(mba, static_cast<CAN_BUS>(i), &mba->m_canBitrate[i]);
//WES                    aba::can::setMode(mba, static_cast<CAN_BUS>(i), mba->m_canMode[i].mode, mba->m_canMode[i].testMode, mba->m_canMode[i].retryEnabled);
//WES                }
                invoke_callback(mba, new AbaMessage(MBA_CORE, MBA_DEVICERECONNECTED, 0, 0));
                return 0;
            }
            else
            {
                // There is no detailed error handling, as usually another CONNECT message will be sent when the device is ready.
            }
            return 0;
        }
        else
        {
            mba->m_connection_status = CON_STATUS::DISCONNECTED;
            invoke_callback(mba, new AbaMessage(MBA_CORE, MBA_DEVICEDISCONNECTED, 0, 0));
        }
    }
    return 0;
}