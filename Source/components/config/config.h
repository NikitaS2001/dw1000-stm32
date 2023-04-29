#pragma once

#include "dwm1000/frame/dwm_frame.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    TDeviceAddr deviceId;

    // Aggregate (sum of) transmitter and receiver antenna delays
    uint32_t aggAntDelay;
} SRuntimeConfig;

SRuntimeConfig ConfigRead(void);
void ConfigWrite(SRuntimeConfig cfg);

#ifdef __cplusplus
}
#endif
