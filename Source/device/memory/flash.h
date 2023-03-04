#ifndef __FLASH_H
#define __FLASH_H

#include <stm3210e_eval.h>

void FLASH_ReadMoreData(uint32_t startAddress, uint16_t* readData, uint16_t countToRead);

void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#endif // __FLASH_H
