#pragma once

#include <string>
#include <stdint.h>

struct settings_t
{
    std::string ack;
    uint32_t dspeed0;
    uint32_t nspeed0;
    uint32_t dspeed1;
    uint32_t nspeed1;
    std::string pipe;
    std::string iface;
};
