#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// May use other format in future (e.g. GUID)
typedef uint8_t TDeviceId;

typedef struct
{
    TDeviceId deviceId;

    // Anchor refresh time interval
    int32_t refreshInt;
} SRuntimeConfig;

SRuntimeConfig ConfigRead(void);
void ConfigWrite(SRuntimeConfig cfg);

#ifdef __cplusplus
}
#endif
