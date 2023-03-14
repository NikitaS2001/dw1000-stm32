#pragma once

#include "utils/std/concurrent_queue.h"

extern "C"
{
    #include <stm32_eval.h>
    #include <stm32f10x.h>
    #include <stm32f10x_usart.h>
}

#include <FreeRTOS.h>
#include <task.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef ROSSERIAL_BAUDRATE
#define ROSSERIAL_BAUDRATE 57600
#endif

extern std::concurrent_queue<uint8_t> rxQueue;
extern std::concurrent_queue<uint8_t> txQueue;

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
    }

    // read a byte from the serial port. -1 = failure
    int read()
    {
        if (rxQueue.size() > 0)
        {
            return rxQueue.pop();
        }
        else
        {
            return -1;
        }
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
        const bool bTxBufWasEmpty = txQueue.size() == 0;
        for (int i = 0; i < length; ++i)
        {
            txQueue.push(data[i]);
        }

        // Trigger sending buffer if not already sending
        if(bTxBufWasEmpty)
        {
            USART_SendData(USART2, txQueue.pop());
        }
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
        return xTaskGetTickCount();
    }

    // System frequency
    uint32_t getSysCoreClock(void)
    {
        return SystemCoreClock;
    }
};
