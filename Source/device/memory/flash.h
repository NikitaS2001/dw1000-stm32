#pragma once

#include <stm3210e_eval.h>

void FlashRead(uint32_t startAddress, uint16_t* readData, uint16_t countToRead);
void FlashWrite(uint32_t startAddress, uint16_t* writeData, uint16_t countToWrite);
