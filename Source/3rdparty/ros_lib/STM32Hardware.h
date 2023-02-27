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

//*****************************************************************************
//
// Bare minimum hardware resources allocated for rosserials communication.
// * One UART port, interrupt driven transfers
// * Two RingBuffers of TX_BUFFER_SIZE and RX_BUFFER_SIZE
// * Systick Interrupt handler
//
//*****************************************************************************

#ifndef ROS_LIB_STM32_HARDWARE_H
#define ROS_LIB_STM32_HARDWARE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
extern "C"
{
  #include <stm32_eval.h>
  #include <stm32f10x.h>
  #include <stm32f10x_usart.h>
	#include "deca_port.h"
  #include "ringbuf.h"
}

#define SYSTICKHZ  1000UL

#ifndef ROSSERIAL_BAUDRATE
#define ROSSERIAL_BAUDRATE 57600
#endif

extern tRingBufObject rxBuffer;
extern tRingBufObject txBuffer;
extern uint8_t ui8rxBufferData[RX_BUFFER_SIZE];
extern uint8_t ui8txBufferData[TX_BUFFER_SIZE];

class STM32Hardware
{
  public:
    STM32Hardware() {}

    void init()
    {
      // Attach to COM2 (USART2)
      USART_InitTypeDef USART_InitStructure;
      USART_InitStructure.USART_BaudRate = ROSSERIAL_BAUDRATE;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

      STM_EVAL_COMInit(COM2, &USART_InitStructure);
      
      // Configure interrupts
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
      USART_ITConfig(USART2, USART_IT_TC, ENABLE);
      
      NVIC_InitTypeDef NVIC_InitStructure;
      NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure); 

      // Initialize UART buffers.
      RingBufInit(&rxBuffer, ui8rxBufferData, RX_BUFFER_SIZE);
      RingBufInit(&txBuffer, ui8txBufferData, TX_BUFFER_SIZE);
    }

    // read a byte from the serial port. -1 = failure
    int read()
    {
      if (RingBufUsed(&rxBuffer))
      {
        return RingBufReadOne(&rxBuffer);
      }
      else
      {
        return -1;
      }
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
      const bool bRingBufWasEmpty = RingBufEmpty(&txBuffer);
      RingBufWrite(&txBuffer, data, length);
  
      // Trigger sending buffer if not already sending
      if(bRingBufWasEmpty)
      {
        USART_SendData(USART2, (uint8_t)RingBufReadOne(&txBuffer));
      }
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
      return portGetTickCount();
    }

    // System frequency
    uint32_t getSysCoreClock(void)
    {
      return SystemCoreClock;
    }
};
#endif  // ROS_LIB_STM32_HARDWARE_H
