/*
 * Copyright (c) 2016, Robosavvy Ltd.
 * All rights reserved.
 * Author: Vitor Matos
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils/std/concurrent_queue.h"

extern "C"
{
  #include <stm32f10x.h>
  #include <stm32f10x_usart.h>
}

#include <stdbool.h>
#include <stdint.h>

std::concurrent_queue<uint8_t> rxQueue;
std::concurrent_queue<uint8_t> txQueue;

extern "C"
{

// UART interrupt handler
// For each received byte, pushes it into the buffer.
// For each transmitted byte, read the next available from the buffer.
void USART2_IRQHandler()
{
    // TX the next available char on the buffer
    if (USART_GetITStatus(USART2, USART_IT_TC))
    {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        if (txQueue.size() > 0)
            USART_SendData(USART2, txQueue.pop());
    }

    // RX and copy the data to buffer
    if (USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        rxQueue.push((uint8_t)USART_ReceiveData(USART2));
    }
}

}
