#include "dwm_addr.h"

#include "deca_lib/deca_device_api.h"
#include "deca_lib/deca_regs.h"

TDeviceAddr ReadPanId(void)
{
    uint32_t panAddrReg = dwt_read32bitreg(PANADR_ID);
    return (panAddrReg & PANADR_PAN_ID_MASK) >> 2;
}

TDeviceAddr ReadDeviceAddress(void)
{
    uint32_t panAddrReg = dwt_read32bitreg(PANADR_ID);
    return panAddrReg & PANADR_SHORT_ADDR_MASK;
}

void WriteDeviceAddress(TDeviceAddr addr)
{
    dwt_setaddress16(addr);
}
