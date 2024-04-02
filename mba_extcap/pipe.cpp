#include "pipe.hpp"
#include <Windows.h>

uint32_t write_pipe(sPipe& pipe, const void* ptr, size_t size)
{
    DWORD cbWritten = 0;

    if (pipe.init)
    {
        WriteFile(pipe.handle, ptr, static_cast<DWORD>(size), &cbWritten, NULL);
    }

    return static_cast<uint32_t>(cbWritten);
}

bool open_pipe(const settings_t& set, sPipe& pipe)
{
    pipe.handle = CreateFileA(set.pipe.c_str(), GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

    if (pipe.handle == INVALID_HANDLE_VALUE)
    {
        return false;
    }
    
    pipe.init = true;
    return true;
}
