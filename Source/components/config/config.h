#pragma once

#include <stdint.h>

// May use other format in future (e.g. GUID)
typedef uint8_t TDeviceId;

struct SRuntimeConfig
{
    TDeviceId deviceId;

    // Anchor refresh time interval
    int32_t refreshInt;
};

SRuntimeConfig ConfigRead();
void ConfigWrite(SRuntimeConfig cfg);
