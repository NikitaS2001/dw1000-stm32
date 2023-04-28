#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint16_t TDeviceAddr;

TDeviceAddr ReadPanId(void);

TDeviceAddr ReadDeviceAddress(void);
void WriteDeviceAddress(TDeviceAddr addr);

#ifdef __cplusplus
}
#endif
