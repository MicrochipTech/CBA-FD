#pragma once

#include <stdint.h>

#define USBD_VID_MICROCHIP          0x04D8  /**< Vendor ID: Microchip */
#define USBD_PID_RESERVED           0xFFFF
#define USBD_RELEASE_RESERVED       0x0000  /**< Release: 0.01 */

#define USBD_PID_BOOTLOADER         0x0AB0
#define USBD_PID_ABA                0x0AB1
#define USBD_PID_CBA                0x0AB1

void setUsbIds(uint16_t pid, uint16_t release);
