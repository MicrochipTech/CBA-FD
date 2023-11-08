#pragma once

#include <stdint.h>
#include "libusb.h"

#ifdef __cplusplus
extern "C" {
#endif

int sendFirmware(char* file, uint32_t flashAddress, libusb_device_handle* dev_handle);

#ifdef __cplusplus
}
#endif
