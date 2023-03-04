#include "config.h"
#include "memory/flash.h"

SRuntimeConfig ConfigRead(void)
{
    SRuntimeConfig cfg;
    FlashRead(USER_FLASH_BASE, (uint16_t*)&cfg, sizeof(cfg) / 2);
    return cfg;
}

void ConfigWrite(SRuntimeConfig cfg)
{
    FlashWrite(USER_FLASH_BASE, (uint16_t*)&cfg, sizeof(cfg) / 2);
}
