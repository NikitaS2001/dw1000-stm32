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

    // Anchor refresh time interval
    int32_t refreshInt;
} SRuntimeConfig;

SRuntimeConfig ConfigRead(void);
void ConfigWrite(SRuntimeConfig cfg);

#ifdef __cplusplus
}
#endif
