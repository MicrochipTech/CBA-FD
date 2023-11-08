/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/

#include <USBD_Config.h>
#include <USBDescriptors.h>
#include <USBDDriver.h>

#include "USB_Descriptor.h"
#include "compiler.h"

/** Device product ID. */
#define ABADriverDescriptors_PRODUCTID       USBD_PID_RESERVED
/** Device vendor ID (Atmel). */
#define ABADriverDescriptors_VENDORID        USBD_VID_MICROCHIP
/** Device release number. */
#define ABADriverDescriptors_RELEASE         USBD_RELEASE_RESERVED

#define ABADeviceDescriptor_CLASS            0x00
#define ABADeviceDescriptor_SUBCLASS         0x00
#define ABADeviceDescriptor_PROTOCOL         0x00

#define ABADriverDescriptors_BULKOUT        1
#define ABADriverDescriptors_BULKIN         2

typedef struct _ABADriverConfigurationDescriptors {

  /** Standard configuration descriptor. */
  USBConfigurationDescriptor configuration;
  /** Communication interface descriptor. */
  USBInterfaceDescriptor  communication;
  /** Data OUT endpoint descriptor. */
  USBEndpointDescriptor dataOut;
  /** Data IN endpoint descriptor. */
  USBEndpointDescriptor dataIn;

} __attribute__ ((__packed__)) ABADriverConfigurationDescriptors;

static USBDeviceDescriptor deviceDescriptor = {

  sizeof(USBDeviceDescriptor),
  USBGenericDescriptor_DEVICE,
  USBDeviceDescriptor_USB2_00,
  ABADeviceDescriptor_CLASS,
  ABADeviceDescriptor_SUBCLASS,
  ABADeviceDescriptor_PROTOCOL,
  64u, /* CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0), */
  ABADriverDescriptors_VENDORID,
  ABADriverDescriptors_PRODUCTID,
  ABADriverDescriptors_RELEASE,
  0, /* Index of manufacturer description */
  1, /* Index of product description */
  2, /* Index of serial number description */
  1  /* One possible configuration */
};

void setUsbIds(uint16_t pid, uint16_t release)
{
  deviceDescriptor.idProduct = pid;
  deviceDescriptor.bcdDevice = release;
}

const ABADriverConfigurationDescriptors configurationDescriptorsFS = {

  /* Standard configuration descriptor */
  {
    sizeof(USBConfigurationDescriptor),
    USBGenericDescriptor_CONFIGURATION,
    sizeof(ABADriverConfigurationDescriptors),
    1, /* There is one interface in this configuration */
    1, /* This is configuration #1 */
    0, /* No string descriptor for this configuration */
    USBD_BMATTRIBUTES,
    USBConfigurationDescriptor_POWER(100)
  },
  /* Communication class interface standard descriptor */
  {
    sizeof(USBInterfaceDescriptor),
    USBGenericDescriptor_INTERFACE,
    0, /* This is interface #0 */
    0, /* This is alternate setting #0 for this interface */
    2, /* This interface uses 2 endpoints */
    ABADeviceDescriptor_CLASS,
    ABADeviceDescriptor_SUBCLASS,
    ABADeviceDescriptor_PROTOCOL,
    0  /* No string descriptor for this interface */
  },
  /* Bulk-OUT endpoint standard descriptor */
  {
    sizeof(USBEndpointDescriptor),
    USBGenericDescriptor_ENDPOINT,
    USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT, ABADriverDescriptors_BULKOUT),
    USBEndpointDescriptor_BULK,
    USBEndpointDescriptor_MAXBULKSIZE_FS,
    0 /* Must be 0 for full-speed bulk endpoints */
  },
  /* Bulk-IN endpoint descriptor */
  {
    sizeof(USBEndpointDescriptor),
    USBGenericDescriptor_ENDPOINT,
    USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN, ABADriverDescriptors_BULKIN),
    USBEndpointDescriptor_BULK,
    USBEndpointDescriptor_MAXBULKSIZE_FS,
    0 /* Must be 0 for full-speed bulk endpoints */
  }
};

const ABADriverConfigurationDescriptors configurationDescriptorsHS = {

  /* Standard configuration descriptor */
  {
    sizeof(USBConfigurationDescriptor),
    USBGenericDescriptor_CONFIGURATION,
    sizeof(ABADriverConfigurationDescriptors),
    1, /* There is one interface in this configuration */
    1, /* This is configuration #1 */
    0, /* No string descriptor for this configuration */
    USBConfigurationDescriptor_SELFPOWERED_NORWAKEUP,
    USBConfigurationDescriptor_POWER(100)
  },
  /* Communication class interface standard descriptor */
  {
    sizeof(USBInterfaceDescriptor),
    USBGenericDescriptor_INTERFACE,
    0, /* This is interface #0 */
    0, /* This is alternate setting #0 for this interface */
    2, /* This interface uses 2 endpoints */
    ABADeviceDescriptor_CLASS,
    ABADeviceDescriptor_SUBCLASS,
    ABADeviceDescriptor_PROTOCOL,
    0  /* No string descriptor for this interface */
  },
  /* Bulk-OUT endpoint standard descriptor */
  {
    sizeof(USBEndpointDescriptor),
    USBGenericDescriptor_ENDPOINT,
    USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT, ABADriverDescriptors_BULKOUT),
    USBEndpointDescriptor_BULK,
    USBEndpointDescriptor_MAXBULKSIZE_HS,
    0 /* Must be 0 for full-speed bulk endpoints */
  },
  /* Bulk-IN endpoint descriptor */
  {
    sizeof(USBEndpointDescriptor),
    USBGenericDescriptor_ENDPOINT,
    USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN, ABADriverDescriptors_BULKIN),
    USBEndpointDescriptor_BULK,
    USBEndpointDescriptor_MAXBULKSIZE_HS,
    0 /* Must be 0 for full-speed bulk endpoints */
  }
};

/** Language ID string descriptor */
const unsigned char languageIdStringDescriptor[] = {

  USBStringDescriptor_LENGTH(1),
  USBGenericDescriptor_STRING,
  USBStringDescriptor_ENGLISH_US
};

/** Product string descriptor */
const unsigned char productStringDescriptor[] = {

  USBStringDescriptor_LENGTH(10),
  USBGenericDescriptor_STRING,
  USBStringDescriptor_UNICODE('M'),
  USBStringDescriptor_UNICODE('B'),
  USBStringDescriptor_UNICODE('A'),
  USBStringDescriptor_UNICODE('n'),
  USBStringDescriptor_UNICODE('a'),
  USBStringDescriptor_UNICODE('l'),
  USBStringDescriptor_UNICODE('y'),
  USBStringDescriptor_UNICODE('z'),
  USBStringDescriptor_UNICODE('e'),
  USBStringDescriptor_UNICODE('r')
};

COMPILER_ALIGNED(32) unsigned char productSerialString[] = {
  USBStringDescriptor_LENGTH(20),
  USBGenericDescriptor_STRING,
  USBStringDescriptor_UNICODE('0'),
  USBStringDescriptor_UNICODE('1'),
  USBStringDescriptor_UNICODE('2'),
  USBStringDescriptor_UNICODE('3'),
  USBStringDescriptor_UNICODE('4'),
  USBStringDescriptor_UNICODE('5'),
  USBStringDescriptor_UNICODE('6'),
  USBStringDescriptor_UNICODE('7'),
  USBStringDescriptor_UNICODE('8'),
  USBStringDescriptor_UNICODE('9'),
  USBStringDescriptor_UNICODE('M'),
  USBStringDescriptor_UNICODE('B'),
  USBStringDescriptor_UNICODE('A'),
  USBStringDescriptor_UNICODE('S'),
  USBStringDescriptor_UNICODE('E'),
  USBStringDescriptor_UNICODE('R'),
  USBStringDescriptor_UNICODE('I'),
  USBStringDescriptor_UNICODE('A'),
  USBStringDescriptor_UNICODE('L'),
  USBStringDescriptor_UNICODE('0')
};

/** List of string descriptors used by the device */
const unsigned char *stringDescriptors[] = {

  languageIdStringDescriptor,
  productStringDescriptor,
  productSerialString
};

/** Device qualifier descriptor (Necessary to pass USB test). */
static const USBDeviceQualifierDescriptor qualifierDescriptor = {

  sizeof(USBDeviceQualifierDescriptor),
  USBGenericDescriptor_DEVICEQUALIFIER,
  USBDeviceDescriptor_USB2_00,
  ABADeviceDescriptor_CLASS,
  ABADeviceDescriptor_SUBCLASS,
  ABADeviceDescriptor_PROTOCOL,
  64,
  1, // Device has one possible configuration.
  0x00
};

/** List of standard descriptors for the serial driver. */
const USBDDriverDescriptors ABADriverDescriptors = {

  &deviceDescriptor,
  (USBConfigurationDescriptor *) &(configurationDescriptorsFS),
  &qualifierDescriptor,
  0,
  0, /* No high-speed device descriptor (uses FS one) */
  (USBConfigurationDescriptor *) &(configurationDescriptorsHS),
  &qualifierDescriptor,
  0,
  stringDescriptors,
  3 /* 3 string descriptors in list */
};
