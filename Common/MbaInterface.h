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

/** @file MbaInterface.h */ 

#pragma once

#include "compiler_w.h"

enum CANFRAME_FLAG
{
    CANFRAME_FLAG_NONE      = 0x0u,     ///< No flags
    CANFRAME_FLAG_EXTENDED  = 0x1u,     ///< Frame has extended bit set
    CANFRAME_FLAG_FD        = 0x2u,     ///< Frame has FD bit set
    CANFRAME_FLAG_BRS       = 0x4u      ///< Frame has BRS bit set
};

enum CAN_MODE
{
    CAN_MODE_CLASSIC,                   ///< Classic CAN
    CAN_MODE_FDNISO,                    ///< CAN-FD using Bosch-CRC
    CAN_MODE_FDISO,                     ///< CAN-FD using ISO-CRC
};

enum CAN_TESTMODE
{
    CAN_TESTMODE_INIT,                  ///< Module init (enters after powercycle)
    CAN_TESTMODE_NORMAL,                ///< Testmode disabled
    CAN_TESTMODE_LISTENONLY,            ///< Listen only, do send ACKs or error frames
    CAN_TESTMODE_LOOPBACK,              ///< Local loopback mode
    CAN_TESTMODE_EXTLOOPBACK,           ///< Extern loopback mode enhances local loopback by enabling frame transmission to the bus. However RX is still disabled and it will not receive ACK or react on error frames.
};

enum CAN_BUS
{
    CAN_BUS_0 = 0,                      ///< CAN bus 0
    CAN_BUS_1,                          ///< CAN bus 1
    CAN_BUS_MAX                         ///< Maximum number of CAN busses
};

#if 0
enum CAN_SPEED
{
    CAN_SPEED_100 = 100,                ///< Nominal CAN speed 100kBit/s
    CAN_SPEED_125 = 125,                ///< Nominal CAN speed 125kBit/s
    CAN_SPEED_250 = 250,                ///< Nominal CAN speed 250kBit/s
    CAN_SPEED_500 = 500,                ///< Nominal CAN speed 500kBit/s
    CAN_SPEED_1000 = 1000               ///< Nominal CAN speed 1000kBit/s
};

enum CAN_FD_SPEED
{
    CAN_FD_SPEED_1000 = 1000,           ///< Data CAN speed 1000kBit/s
    CAN_FD_SPEED_2000 = 2000,           ///< Data CAN speed 2000kBit/s
    CAN_FD_SPEED_3077 = 3077,           ///< Data CAN speed 3077kBit/s
    CAN_FD_SPEED_4000 = 4000,           ///< Data CAN speed 4000kBit/s
    CAN_FD_SPEED_5000 = 5000,           ///< Data CAN speed 5000kBit/s
    CAN_FD_SPEED_6667 = 6667,           ///< Data CAN speed 6667kBit/s
    CAN_FD_SPEED_8000 = 8000            ///< Data CAN speed 8000kBit/s
};
#endif

enum CAN_MSG
{
    CAN_MSG_RX = 0,                     ///< This message describes a successfully received CAN frame. See @ref CanFrame.
    CAN_MSG_TX,                         ///< This message describes a successfully transmitted CAN frame. See @ref CanFrame.
    CAN_MSG_ERR,                        ///< This message describes an error on the bus. See @ref CanError.
    CAN_MSG_DBG,                        ///< This message describes a register read. Reserved.
    CAN_MSG_BITRATE,                    ///< This message describes the CAN bitrate setting. See @ref CanBitrate.
    CAN_MSG_MODE
};

enum MBA_CORE
{
    MBA_RESERVED = 0,                   ///< Reserved
    MBA_TIMESTAMP,                      ///< Get the current time of the MBA. Callback is invoked with data containing a uint64_t timestamp in usec since powerup of the MBA.
    MBA_VERSION,                        ///< Get the software version of the MBA. Callback is invoked with data containing a @ref mba_version_t pointer.
    MBA_USBERROR,                       ///< Libusb error container. Callback is invoked with data containing a libusb_transfer_status enum value.
    MBA_DEVICEDISCONNECTED,             ///< When a hotplug disconnect has happened. Callback is invoked with data set to 0.
    MBA_DEVICERECONNECTED               ///< When a previously configured device has been disconnected and is reconnecting now. Callback is invoked with data set to 0.
};

enum MBA_RESET
{
    MBA_RESET_APPLICATION = 0,          ///< Reset MBA
    MBA_RESET_BOOTLOADER                ///< Reset MBA and stay in bootloader
};

enum MBA_DEVICE
{
    DEVICE_INVALID = 0,                 ///< Unknown device
    DEVICE_ABA,                         ///< Device is of type MBA_ABA
    DEVICE_CBA                          ///< Device is of type MBA_CBA
};

enum MBA_CMD
{
    MBA_CORE = 0,                       ///< MBA command, see @ref MBA_MSG
    MBA_CAN,                            ///< CAN message, see @ref CAN_MSG
};

struct mba_handle_t;

COMPILER_PACK_SET(1)
struct CanFrame
{
    uint32_t busId;                     ///< @ref CAN_BUS
    uint32_t id;                        ///< CAN frame id, will be truncated to 11-bit or 28-bit depending on extended flag
    uint64_t timestamp;                 ///< MBA timestamp set on event (read-only)
    uint8_t dlc;                        ///< Encoded CAN data length, max. 8 byte for non-CAN-FD frames
    uint8_t flags;                      ///< @ref CANFRAME_FLAG
    uint8_t data[64];                   ///< CAN frame payload, will truncated to dlc
};

struct CanError
{
    uint32_t busId;                     ///< @ref CAN_BUS
    uint8_t busOff;                     ///< Bus off indicator
    uint8_t tec;                        ///< Transmit error counter
    uint8_t rec;                        ///< Receive error counter
    uint8_t lec;                        ///< Error code set during arbitration phase. Refer to V71 datasheet for additional information.
    uint8_t dlec;                       ///< Error code set during adata phase. Refer to V71 datasheet for additional information.
    uint64_t timestamp;                 ///< MBA timestamp set on event (read-only)
};

struct CanBitrate
{
    uint32_t busId;                     ///< @ref CAN_BUS read-only value
    uint8_t nsjw;                       ///< Synchronization jump width of arbitration phase
    uint8_t nseg1;                      ///< Segment1 of arbitration phase
    uint8_t nseg2;                      ///< Segment2 of arbitration phase
    uint8_t nscale;                     ///< Clock prescaler for CAN2.0 and arbitration phase
    uint8_t dsjw;                       ///< Synchronization jump width of data phase
    uint8_t dseg1;                      ///< Segment1 of data phase
    uint8_t dseg2;                      ///< Segment2 of data phase
    uint8_t dscale;                     ///< Clock prescaler for CAN-FD data phase
    uint8_t tdc;                        ///< Transmitter delay compensation
    uint8_t tdcf;                       ///< Transmitter delay compensation filter
    uint8_t tdco;                       ///< Transmitter delay compensation offset
};

struct CanMode
{
    uint32_t busId;                     ///< @ref CAN_BUS read-only value
    uint8_t mode;                       ///< @ref CAN_MODE
    uint8_t testMode;                   ///< @ref CAN_TESTMODE
    uint8_t retryEnabled;               ///< If retries are enabled, a CAN frame will be (re-) transmitted until it's acknowledged by other bus participants or the MBA goes into bus-off mode.
};

struct mba_message_t
{
    uint8_t instance;                   ///< When multiple MBA devices are connected via the interface, they will be numbered in the order they have been opened. If a device has been opened and closed, it will get the same id reassigned.
    uint8_t messageType;                ///< @ref MBA_CMD
    uint8_t command;                    ///< Depending on messageType
    uint8_t reserved;                   ///< Reserved
    uint8_t* data;                      ///< Pointer to the data, content depending on messageType and command
    uint32_t data_length;               ///< Length of the data
};

struct mba_statistics_t
{
    uint32_t messagesReceived;          ///< Number of frames the driver library received via USB 
    uint32_t messagesDropped;           ///< Number of frames the driver library dropped due to buffer overrun
};

struct mba_version_t
{
    uint32_t ver_firmware;              ///< Firmware version
    uint32_t ver_api;                   ///< Protobuf version
    uint32_t ver_can;					// 	
    uint32_t ver_bootloader;            ///< Bootloader version
    uint32_t ver_hardware;              ///< Hardware version
    uint32_t ver_hash;                  ///< Firmware hash
};

struct mba_serial_t
{
    char octet[40];                    ///< Alphanumeric serial number of the device 
};

struct mba_device_t
{
    uint32_t deviceType;                ///< See @ref MBA_DEVICE
    mba_serial_t serial;                ///< See @ref mba_serial_t
};
COMPILER_PACK_RESET()

enum E_ERROR
{
    E_OK    =  0,                       ///< All okay
    E_ERR   = -1,                       ///< Error
    E_INIT  = -2,                       ///< Device not initialized
    E_ARG   = -3,                       ///< Wrong argument
    E_DEV   = -4,                       ///< Called function on wrong device
    E_NODEV = -5,                       ///< No device available
    E_MEM   = -6,                       ///< Memory allocation error
    E_DISC  = -7                        ///< Device has been disconnected (without being closed before)
};

/**
 * Definition of the callback function invoked by the driver library.
 * 1. Evaluate message->instance which specifies a unique id based on the order the MBA device handles have been created.
 * 2. Evaluate message->messageType to see if this message is a generic MBA_CORE or MBA_CAN message. See @ref MBA_CMD
 * 3. Evaluate message->command, note that this is depending on the messageType. See @ref MBA_CORE and @ref CAN_MSG.
 * 4. Parse the message->data to the according structure e.g. @ref CanFrame, @ref CanError, etc. Verify the size of the structure matches message->data_length.
 *
 * @param message See @ref mba_message_t
 * @param user_data The pointer specified by @ref registerCallback
 */
typedef void(*tCallback)(const mba_message_t* message, void* user_data);

#ifdef __cplusplus
extern "C" {
#endif
    /**
    * Unload the MBA interface library.
    * On Windows based devices, unload is implicitly called by when the library is unloaded.
    */
    DLL_EXPORT void unload();

    /**
    * Enumerate all connected MBA devices.
    * A pointer to the list will be stored in device_list parameter. The memory is owned by the library and will be freed when unload() is invoked.
    * @param device_list Pointer to a mba_device_t pointer, which will point to the device list after the function returns successful
    * @result Number of devices available in the list
    * @result 0 if no device is available
    * @result E_ARG if devive_list pointer is invalid
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t enumDevices(mba_device_t** device_list);

    /**
    * Open a MBA device based on the serial number.
    * A pointer to the instance will be stored in mba_device parameter.
    * @param mba_device Pointer to a mba_handle_t pointer, which will point to the device handle if the function returns successful
    * @param serial Pointer to a MBA serial returned by @ref enumDevices
    * @result E_OK on success
    * @result E_ARG mba_device or serial pointer are invalid
    * @result E_INIT if the device has been opened already
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t openDevice(mba_handle_t** mba_device, const mba_serial_t* serial);

    /**
    * Close a MBA device.
    * @param mba_device Pointer to a mba_handle_t that has been created by calling @ref openDevice
    * @result See @ref E_ERROR
    * @result E_OK on success
    * @result E_ARG if mba_device pointer is invalid.
    */
    DLL_EXPORT int32_t closeDevice(mba_handle_t* mba_device);

    /**
    * Register the callback handler for the MBA library. Only a single callback can be registered.
    * If multiple MBA devices are used, the instance id is provided to the callback function.
    * @param callback Pointer to the callback function that is invoked by the MBA library on any event
    * @param user_data Pointer to user defined data that is passed to the callback on invokation
    * @result See @ref E_ERROR
    * @result E_OK on success
    * @result E_ARG if callback pointer is invalid
    */
    DLL_EXPORT int32_t registerCallback(tCallback callback, void* user_data);

    /**
    * Remove the callback.
    * If the device is not closed, messages may be cached and delivered when a new callback is registered.
    */
    DLL_EXPORT int32_t unregisterCallback();

    /**
    * Send a CAN frame.
    * If a CAN-FD shall be transmitted, it will automatically padded to the next matching frame size, e.g. sending 60 byte will create a 64 byte frame.
    * If the internal CAN Tx fifo of the MBA is full, it will wait for timeout ms and notify via callback if the message still could not be placed in the fifo.
    * When a message has been transmitted successfully or transmission resulted in an error, the callback will be invoked asynchronously.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @param canId CAN frame identifier
    * @param payload Pointer to the CAN frame payload
    * @param dlc Length of the CAN frame payload 
    * @param flags Flags of the CAN frame, see @ref CANFRAME_FLAG
    * @param timeout Transmit timeout in millisecond, maximum value of 1000
    * @result E_OK on success
    * @result E_ARG if mba_device or payload pointer are invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_SendFrame(mba_handle_t* mba_device, CAN_BUS busId, uint32_t canId, uint8_t* payload, uint8_t dlc, uint8_t flags, uint16_t timeout);

    /**
    * Send multiple CAN frames using a single USB transaction.
    * Refer to @ref CanFrame for member variables. \b busId must be set for every frame to select the CAN bus. Timeout must be set for every frame using the \b timestamp member.
    * The callback will be invoked accordingly, please refer to @CAN_SendFrame for additional information.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param frames Pointer to an array of @ref CanFrame
    * @param count Number of array entries
    * @result E_OK on success
    * @result E_ARG if mba_device or frames pointer are invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_SendFramesBulk(mba_handle_t* mba_device, CanFrame* frames, uint32_t count);

    /**
    * Set the CAN speed of a channel.
    * If invalid settings are supplied, the speed will default to 500/2000 kBit/s. Sample point is always set to 80%.
    * After invokation, the bit-timing will be returned asynchronously by the callback, see @CAN_GetBitTiming.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @param canSpeed CAN speed, available values are 100, 125, 250, 500 and 1000 kBit/s
    * @param canFdSpeed CAN-FD speed, available values are 1000, 2000, 3077, 4000, 5000, 6667, 8000 kBit/s
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_SetSpeed(mba_handle_t* mba_device, CAN_BUS busId, uint32_t canSpeed, uint32_t canFdSpeed);

    /**
    * Set the CAN bit-timing of a channel by providing a low-level configuration.
    * There is no validity checking for the provided parameters
    * After the timing has been applied to the hardware, the applied bit-timing will be returned asynchronously by the callback.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @param bitrate Pointer to @ref CanBitrate object
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_SetBitTiming(mba_handle_t* mba_device, CAN_BUS busId, CanBitrate* bitrate);

    /**
    * Get the CAN bit-timing of a channel.
    * After invokation, the bit-timing will be returned asynchronously by the callback.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_GetBitTiming(mba_handle_t* mba_device, CAN_BUS busId);

    /**
    * Set the CAN operation mode of a channel.
    * After the modes have been applied to the hardware, the applied settings will be returned asynchronously by the callback.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @param canMode @ref CAN_MODE
    * @param testMode @ref CAN_TESTMODE
    * @param autoRetryEnabled When enabled, MBA will retry to transmit a message when arbitration was lost
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_SetMode(mba_handle_t* mba_device, CAN_BUS busId, uint8_t canMode, uint8_t testMode, uint8_t autoRetryEnabled);

    /**
    * Get the CAN operation mode of a channel.
    * After invokation, the current settings will be returned asynchronously by the callback.
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_GetMode(mba_handle_t* mba_device, CAN_BUS busId);

    /**
    * Reset a CAN bus to init mode
    * @param mba_device Handle to the device created by @ref openDevice
    * @param busId Index of the CAN bus
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_Reset(mba_handle_t* mba_device, CAN_BUS busId);

    /**
    * Read a register of the CAN peripheral
    * Please refer to SAMV71 datasheet for details
    * @param mba_device Handle to the device created by @ref openDevice
    * @param address Absolute 32-bit address of the register. MBA will only return valid values if address points to CAN register space.
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_DEV when trying to send a CAN frame on incapable hardware
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t CAN_ReadRegister(mba_handle_t* mba_device, uint32_t address);

    /**
    * Reset the MBA
    * @param mba_device Handle to the device created by @ref openDevice
    * @param resetMode @ref MBA_RESET
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t MBA_Reset(mba_handle_t* mba_device, MBA_RESET resetMode);

    /**
    * Query the MBA firmware version
    * @param mba_device Handle to the device created by @ref openDevice
    * @result E_OK on success
    * @result E_ARG if mba_device is invalid
    * @result E_ERR if USB communication failed
    */
    DLL_EXPORT int32_t MBA_GetVersion(mba_handle_t* mba_device);

#ifdef __cplusplus
}
#endif
