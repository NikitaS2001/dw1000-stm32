#include "flash.h"
#include <stm32f10x_flash.h>

#define FLASH_SIZE 512
#if FLASH_SIZE < 256
    #define SECTOR_SIZE           1024    // bytes
#else
    #define SECTOR_SIZE           2048    // bytes
#endif

uint16_t FLASH_ReadHalfWord(uint32_t address)
{
    return *(__IO uint16_t*)address;
}

uint32_t FLASH_ReadWord(uint32_t address)
{
    uint32_t temp1, temp2;
    temp1 = *(__IO uint16_t*)address;
    temp2 = *(__IO uint16_t*)(address+2);
    return (temp2 << 16) + temp1;
}

void FlashRead(uint32_t startAddress, uint16_t* readData, uint16_t countToRead)
{
    if (startAddress < FLASH_BASE || (startAddress + countToRead * 2) >= (FLASH_BASE + 1024 * FLASH_SIZE))
    {
        // Out of bounds
        return;
    }

    for (uint16_t dataIndex = 0; dataIndex < countToRead; dataIndex++)
    {
        readData[dataIndex] = FLASH_ReadHalfWord(startAddress + dataIndex * 2);
    }
}

void FlashWrite(uint32_t startAddress, uint16_t* writeData, uint16_t countToWrite)
{
    if (startAddress < FLASH_BASE || (startAddress + countToWrite * 2) >= (FLASH_BASE + 1024 * FLASH_SIZE))
    {
        // Out of bounds
        return;
    }

    FLASH_Unlock();

    uint32_t offsetAddress=startAddress-FLASH_BASE;
    uint32_t sectorPosition=offsetAddress/SECTOR_SIZE;
    uint32_t sectorStartAddress=sectorPosition*SECTOR_SIZE+FLASH_BASE;
    FLASH_ErasePage(sectorStartAddress); // Erase full sector before write

    for (uint16_t dataIndex = 0; dataIndex < countToWrite; dataIndex++)
    {
        FLASH_ProgramHalfWord(startAddress+dataIndex*2,writeData[dataIndex]);
    }

    FLASH_Lock();
}
