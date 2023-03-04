#pragma once

#include <stm3210e_eval.h>

#ifdef __cplusplus
extern "C" {
#endif

void FLASH_ReadMoreData(uint32_t startAddress, uint16_t* readData, uint16_t countToRead);
void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#ifdef __cplusplus
}
#endif
