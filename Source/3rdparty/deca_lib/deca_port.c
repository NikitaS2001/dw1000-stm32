/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "deca_port.h"
#include "deca_sleep.h"

#define rcc_init(x)                 RCC_Configuration(x)
#define systick_init(x)             SysTick_Configuration(x)
#define rtc_init(x)                 RTC_Configuration(x)
#define interrupt_init(x)           NVIC_Configuration(x)
#define usart_init(x)               USART_Configuration(x)
#define spi_init(x)                 SPI_Configuration(x)
#define gpio_init(x)                GPIO_Configuration(x)
#define ethernet_init(x)            No_Configuration(x)
#define fs_init(x)                  No_Configuration(x)
#define usb_init(x)                 No_Configuration(x)
#define touch_screen_init(x)        No_Configuration(x)

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

/* Internal functions prototypes. */
static void spi_peripheral_init(void);

int No_Configuration(void)
{
    return -1;
}

unsigned long portGetTickCnt(void)
{
    return time32_incr;
}

int SysTick_Configuration(void)
{
    if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
    {
        /* Capture error */
        while (1);
    }
    NVIC_SetPriority (SysTick_IRQn, 5);

    return 0;
}

void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}


int NVIC_DisableDECAIRQ(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MPW3 IRQ polarity is high by default
    EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_NOIRQ;
    EXTI_Init(&EXTI_InitStructure);

    return 0;
}


int NVIC_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable GPIO used as DECA IRQ for interrupt
    GPIO_InitStructure.GPIO_Pin = DECAIRQ;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;  //IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
    GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);

    /* Connect EXTI Line to GPIO Pin */
    GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MPW3 IRQ polarity is high by default
    EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
    EXTI_Init(&EXTI_InitStructure);

    /* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DECAIRQ_EXTI_USEIRQ;

    NVIC_Init(&NVIC_InitStructure);

    /* Enable the RTC Interrupt */
    //NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    //NVIC_Init(&NVIC_InitStructure);

    return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;
    /* Check the parameters */
    assert_param(IS_GET_EXTI_LINE(EXTI_Line));

    enablestatus =  EXTI->IMR & EXTI_Line;
    if (enablestatus != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

int USART_Configuration(void)
{
    return 0;
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
    uint16_t tmpreg = 0;

    /* Get the SPIx CR1 value */
    tmpreg = SPIx->CR1;

    /*clear the scaling bits*/
    tmpreg &= 0xFFC7;

    /*set the scaling bits*/
    tmpreg |= scalingfactor;

    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void)
{
    SPI_ChangeRate(SPI_BaudRatePrescaler_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void)
{
    SPI_ChangeRate(SPI_BaudRatePrescaler_4);
}

void SPI_ConfigFastRate(uint16_t scalingfactor)
{
    SPI_InitTypeDef SPI_InitStructure;

    SPI_I2S_DeInit(SPIx);

    // SPIx Mode setup
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx, &SPI_InitStructure);

    // Enable SPIx
    SPI_Cmd(SPIx, ENABLE);
}

int SPI_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_I2S_DeInit(SPIx);

    // SPIx Mode setup
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
    SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALER;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx, &SPI_InitStructure);

    // SPIx SCK and MOSI pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx MISO pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx CS pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);

    // Disable SPIx SS Output
    SPI_SSOutputCmd(SPIx, DISABLE);

    // Enable SPIx
    SPI_Cmd(SPIx, ENABLE);

    // Set CS high
    GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

    return 0;
}


int SPI2_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_I2S_DeInit(SPIy);

    // SPIy Mode setup
    //SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;     //
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPIy_PRESCALER;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIy, &SPI_InitStructure);

    // SPIy SCK and MOSI pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_SCK | SPIy_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    // SPIy MISO pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_MISO;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    // SPIy CS pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_CS_GPIO, &GPIO_InitStructure);

    // Disable SPIy SS Output
    SPI_SSOutputCmd(SPIy, DISABLE);

    // Enable SPIy
    SPI_Cmd(SPIy, ENABLE);

    // Set CS high
    GPIO_SetBits(SPIy_CS_GPIO, SPIy_CS);

    return 0;
}

int GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure all unused GPIO port pins in Analog Input mode (floating input
    * trigger OFF), this will reduce the power consumption and increase the device
    * immunity against EMI/EMC */

    // Enable GPIOs clocks
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
        ENABLE);

    // Set all GPIO pins as analog inputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    //Enable GPIO used for User button
    GPIO_InitStructure.GPIO_Pin = TA_BOOT1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TA_BOOT1_GPIO, &GPIO_InitStructure);

    //Enable GPIO used for Response Delay setting
    GPIO_InitStructure.GPIO_Pin = TA_RESP_DLY | TA_SW1_3 | TA_SW1_4 | TA_SW1_5 | TA_SW1_6 | TA_SW1_7 | TA_SW1_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TA_RESP_DLY_GPIO, &GPIO_InitStructure);

    //Enable GPIO used for SW1 switch setting
    GPIO_InitStructure.GPIO_Pin = TA_SW1_3 | TA_SW1_4 | TA_SW1_6 | TA_SW1_7 | TA_SW1_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TA_SW1_GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TA_SW1_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TA_SW1_GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_1|GPIO_Pin_2;              //INX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //�����趨������ʼ��GPIOB.5
    // Disable GPIOs clocks
    //RCC_APB2PeriphClockCmd(
    //                  RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
    //                  RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
    //                  RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
    //                  DISABLE);

    // Enable GPIO used for LEDs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);
GPIO_SetBits(GPIOC,GPIO_Pin_13);//change by johhn
    return 0;
}


void reset_DW1000(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO used for DW1000 reset
    GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

    //drive the RSTn pin low
    GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

    //put the pin back to tri-state ... as input
    GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

    deca_sleep(2);
}


void setup_DW1000RSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if(enable)
    {
        // Enable GPIO used as DECA IRQ for interrupt
        GPIO_InitStructure.GPIO_Pin = DECARSTIRQ;
        //GPIO_InitStructure.GPIO_Mode =    GPIO_Mode_IPD;  //IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(DECARSTIRQ_GPIO, &GPIO_InitStructure);

        /* Connect EXTI Line to GPIO Pin */
        GPIO_EXTILineConfig(DECARSTIRQ_EXTI_PORT, DECARSTIRQ_EXTI_PIN);

        /* Configure EXTI line */
        EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MP IRQ polarity is high by default
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        /* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

        /* Enable and set EXTI Interrupt to the lowest priority */
        NVIC_InitStructure.NVIC_IRQChannel = DECARSTIRQ_EXTI_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

        NVIC_Init(&NVIC_InitStructure);
    }
    else
    {
        //put the pin back to tri-state ... as input
        GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

        /* Configure EXTI line */
        EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MP IRQ polarity is high by default
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
        EXTI_Init(&EXTI_InitStructure);
    }
}

int ETH_GPIOConfigure(void)
{
    return 0;
}

int is_button_low(uint16_t GPIOpin)
{
    int result = 1;

    if (GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))
        result = 0;

    return result;
}

//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin)
{
    int result = 1;

    if(GPIOpin == TA_SW1_5)
    {
        if (GPIO_ReadInputDataBit(TA_SW1_GPIOA, GPIOpin))
            result = 0;
    }
    else
    {
        if (GPIO_ReadInputDataBit(TA_SW1_GPIOC, GPIOpin))
            result = 0;
    }

    return result;
}


void led_off (led_t led)
{
    switch (led)
    {
        case LED_PC6:
            GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            break;
        case LED_PC7:
            GPIO_ResetBits(GPIOC, GPIO_Pin_7);
            break;
        case LED_PC8:
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
            break;
        case LED_PC9:
            GPIO_ResetBits(GPIOC, GPIO_Pin_9);
            break;
        case LED_ALL:
            GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
            break;
        default:
            // do nothing for undefined led number
            break;
    }
}

void led_on (led_t led)
{
    switch (led)
    {
        case LED_PC6:
            GPIO_SetBits(GPIOC, GPIO_Pin_6);
            break;
        case LED_PC7:
            GPIO_SetBits(GPIOC, GPIO_Pin_7);
            break;
        case LED_PC8:
            GPIO_SetBits(GPIOC, GPIO_Pin_8);
            break;
        case LED_PC9:
            GPIO_SetBits(GPIOC, GPIO_Pin_9);
            break;
        case LED_ALL:
            GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
            break;
        default:
            // do nothing for undefined led number
            break;
    }
}


/**
  * @brief  Configures COM port.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
#include "stm32_eval.h"
void usartinit(void)
{
    USART_InitTypeDef USART_InitStructure;
    //GPIO_InitTypeDef GPIO_InitStructure;

    /* USARTx configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM1, &USART_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 55;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART_puts(uint8_t *s,uint8_t len)
{
    int i;
    for(i=0; i<len; i++)
    {
        putchar(s[i]);
    }
}

int is_IRQ_enabled(void)
{
    return ((   NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]
                & (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_peripheral_init()
 *
 * @brief Initialise all SPI peripherals at once.
 *
 * @param none
 *
 * @return none
 */
static void spi_peripheral_init(void)
{
    spi_init();//for dwm1000
}

/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef  I2C_InitStructure;
static void MX_I2C1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure I2C1 pins: PB6->SCL and PB7->SDA */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x30;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void)
{
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
        ENABLE);

    /* Enable SPI1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* Enable SPI2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    //rcc_init();

    gpio_init();
    systick_init();
    spi_peripheral_init();

    usartinit();
#ifdef STM32F10X_MD
    MX_I2C1_Init();
#endif
}


