#pragma once

#include "settings.hpp"

struct sPipe
{
    bool init;
    void* handle;
    int file;
};

uint32_t write_pipe(sPipe& pipe, const void* ptr, size_t size);
bool open_pipe(const settings_t& set, sPipe& pipe);
