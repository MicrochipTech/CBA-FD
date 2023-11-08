#pragma once

#include "AbaUSB.h"
#include "MbaInterface.h"

typedef void (*notificationCallback)(aba::usb::USBEVENT event, mba_serial_t& serial, void* user_data);

int hotplug_windows(mba_serial_t& serial, aba::usb::USBEVENT event);