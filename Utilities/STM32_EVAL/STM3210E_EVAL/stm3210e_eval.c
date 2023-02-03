/**
  ******************************************************************************
  * @file    stm3210e_eval.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file provides
  *            - set of firmware functions to manage Leds, push-button and COM ports
  *            - low level initialization functions for SD card (on SDIO), SPI serial
  *              flash (sFLASH) and temperature sensor (LM75)
  *          available on STM3210E-EVAL evaluation board from STMicroelectronics.  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm3210e_eval.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_dma.h"


#include "deca_types.h"
#include <string.h>
#include "port.h"
#include "deca_sleep.h"
#include "deca_regs.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM3210E_EVAL
  * @{
  */ 
      
/** @defgroup STM3210E_EVAL_LOW_LEVEL 
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports, SD card on SDIO, serial flash (sFLASH), serial EEPROM (sEE) 
  *        and temperature sensor (LM75) available on STM3210E-EVAL evaluation 
  *        board from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_Variables
  * @{
  */ 
GPIO_TypeDef* GPIO_PORT[LEDnum] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDnum] = {LED1_PIN, LED2_PIN, LED3_PIN,
                                 LED4_PIN};
const uint32_t GPIO_CLK[LEDnum] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_GPIO_PORT, TAMPER_BUTTON_GPIO_PORT, 
                                      KEY_BUTTON_GPIO_PORT, RIGHT_BUTTON_GPIO_PORT,
                                      LEFT_BUTTON_GPIO_PORT, UP_BUTTON_GPIO_PORT,
                                      DOWN_BUTTON_GPIO_PORT, SEL_BUTTON_GPIO_PORT}; 

const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN, TAMPER_BUTTON_PIN, 
                                      KEY_BUTTON_PIN, RIGHT_BUTTON_PIN,
                                      LEFT_BUTTON_PIN, UP_BUTTON_PIN,
                                      DOWN_BUTTON_PIN, SEL_BUTTON_PIN}; 

const uint32_t BUTTON_CLK[BUTTONn] = {WAKEUP_BUTTON_GPIO_CLK, TAMPER_BUTTON_GPIO_CLK,
                                      KEY_BUTTON_GPIO_CLK, RIGHT_BUTTON_GPIO_CLK,
                                      LEFT_BUTTON_GPIO_CLK, UP_BUTTON_GPIO_CLK,
                                      DOWN_BUTTON_GPIO_CLK, SEL_BUTTON_GPIO_CLK};

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {WAKEUP_BUTTON_EXTI_LINE,
                                            TAMPER_BUTTON_EXTI_LINE, 
                                            KEY_BUTTON_EXTI_LINE,
                                            RIGHT_BUTTON_EXTI_LINE,
                                            LEFT_BUTTON_EXTI_LINE,
                                            UP_BUTTON_EXTI_LINE,
                                            DOWN_BUTTON_EXTI_LINE,
                                            SEL_BUTTON_EXTI_LINE};

const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {WAKEUP_BUTTON_EXTI_PORT_SOURCE,
                                              TAMPER_BUTTON_EXTI_PORT_SOURCE, 
                                              KEY_BUTTON_EXTI_PORT_SOURCE,
                                              RIGHT_BUTTON_EXTI_PORT_SOURCE,
                                              LEFT_BUTTON_EXTI_PORT_SOURCE,
                                              UP_BUTTON_EXTI_PORT_SOURCE,
                                              DOWN_BUTTON_EXTI_PORT_SOURCE,
                                              SEL_BUTTON_EXTI_PORT_SOURCE};
								 
const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {WAKEUP_BUTTON_EXTI_PIN_SOURCE,
                                             TAMPER_BUTTON_EXTI_PIN_SOURCE, 
                                             KEY_BUTTON_EXTI_PIN_SOURCE,
                                             RIGHT_BUTTON_EXTI_PIN_SOURCE,
                                             LEFT_BUTTON_EXTI_PIN_SOURCE,
                                             UP_BUTTON_EXTI_PIN_SOURCE,
                                             DOWN_BUTTON_EXTI_PIN_SOURCE,
                                             SEL_BUTTON_EXTI_PIN_SOURCE}; 

const uint16_t BUTTON_IRQn[BUTTONn] = {WAKEUP_BUTTON_EXTI_IRQn, TAMPER_BUTTON_EXTI_IRQn, 
                                       KEY_BUTTON_EXTI_IRQn, RIGHT_BUTTON_EXTI_IRQn,
                                       LEFT_BUTTON_EXTI_IRQn, UP_BUTTON_EXTI_IRQn,
                                       DOWN_BUTTON_EXTI_IRQn, SEL_BUTTON_EXTI_IRQn};

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};
 
const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};

/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];     
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4 
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];    
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button  
  *     @arg BUTTON_KEY: Key Push Button 
  *     @arg BUTTON_RIGHT: Joystick Right Push Button 
  *     @arg BUTTON_LEFT: Joystick Left Push Button 
  *     @arg BUTTON_UP: Joystick Up Push Button 
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_APB2PeriphClockCmd(BUTTON_CLK[Button] | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);


  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    GPIO_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if(Button != BUTTON_WAKEUP)
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    }
    else
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:    
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button  
  *     @arg BUTTON_KEY: Key Push Button 
  *     @arg BUTTON_RIGHT: Joystick Right Push Button 
  *     @arg BUTTON_LEFT: Joystick Left Push Button 
  *     @arg BUTTON_UP: Joystick Up Push Button 
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button    
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable UART clock */
  if (COM == COM1)
  {
    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE); 
  }
  else
  {
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);
	
	
	
	
	
	NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//选择分组方式0
  /* 使能 USART1 中断 */

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 55;

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//change by johhn
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}


#define QUERY_CMD		0x01 	/* 查询命令 */
#define EXECUTE_CMD		0x02	/* 执行命令 */
#define SET_CMD			0x03	/* 设置命令 */
#define ARGC_LIMIT 250//change by johhn
#define NULL  0//change by johhn


typedef struct {
	char *cmd;	/* AT指令 */
	int (*deal_func)(int opt, int argc, char *argv[]);	
}at_cmd_t;
#define ANCHOR_MAX_NUM 3
extern int Anthordistance_count[ANCHOR_MAX_NUM];
extern int Anthordistance[ANCHOR_MAX_NUM];
uint8 SWITCH_DIS=0;
extern int ANCHOR_REFRESH_COUNT_set;
int AT_CmdFunc_ondis(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
	}
	else if(opt==SET_CMD)
	{
		
		char *get=NULL;
		get=strtok(argv[0],",");
		uint16_t Set=atoi(get);
		
		if(Set==1)
		{
			SWITCH_DIS=1;
		}
		else if(Set==0)
		{
			SWITCH_DIS=0;
		}
		else
		{
			return -1;
		}
	
	}
	else if(opt==QUERY_CMD)
	{
		
		
	}
	
	
	return 1;
}
int AT_CmdFunc_interval(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
	}
	else if(opt==SET_CMD)
	{
		char *get=NULL;
		get=strtok(argv[0],",");
		uint16_t Set=atoi(get);
		if(Set>=5&& Set<=50)
		{
			ANCHOR_REFRESH_COUNT_set=Set;
		}
		else
		{
			return -1;
		}
	
	}
	else if(opt==QUERY_CMD)
	{
		
		
	}
	
	
	return 1;
}

int AT_CmdFunc_version(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
	}
	else if(opt==SET_CMD)
	{
		
	
	}
	else if(opt==QUERY_CMD)
	{
		printf("AIT-BU01-DB V100 T2020-5-17\r\n");
	}
	
	
	return 1;
}
int AT_CmdFunc_RST(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		printf("\r\nOK\r\n");
		NVIC_SystemReset();
	}
	else if(opt==SET_CMD)
	{
		
		
	}
	else if(opt==QUERY_CMD)
	{
		
	
	}
	
	
	return 1;
}





int AT_CmdFunc_TemHum(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		float get=0;
		
		SHT20_read_tem(&get);
		printf("T:%f\r\n",get);
		
		get=0;
		SHT20_read_hum(&get);
		printf("H:%f\r\n",get);
		
	}
	else if(opt==SET_CMD)
	{
		
	}
	else if(opt==QUERY_CMD)
	{
	
	}
	
	
	return 1;
}


int AT_CmdFunc_XYZ(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		float speed=0;
		drv_lis2dh12_get_angle(&speed);
		
		printf("angle:%f\r\n",speed);
	}
	else if(opt==SET_CMD)
	{
		
	}
	else if(opt==QUERY_CMD)
	{
		
	}
	
	
	return 1;
}


UserSet UserSetNow={
		1,
		0
};
#define FLASH_SIZE 512
#if FLASH_SIZE<256
  #define SECTOR_SIZE           1024    //字节
#else
  #define SECTOR_SIZE           2048    //字节
#endif

//读取指定地址的半字(16位数据)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
	return*(__IO uint16_t*)address;
}
 
//读取指定地址的全字(32位数据)
uint32_t FLASH_ReadWord(uint32_t address)
{
	uint32_t temp1,temp2;
	temp1=*(__IO uint16_t*)address;
	temp2=*(__IO uint16_t*)(address+2);
	return (temp2<<16)+temp1;
}


void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead)
{
	uint16_t dataIndex;
	for(dataIndex=0;dataIndex<countToRead;dataIndex++)
	{
		readData[dataIndex]=FLASH_ReadHalfWord(startAddress+dataIndex*2);
	}
}
void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
{
	if(startAddress< FLASH_BASE || (startAddress+countToWrite*2)>= (FLASH_BASE+1024*FLASH_SIZE) )
	{
		return;
	}
	FLASH_Unlock();
	uint32_t offsetAddress=startAddress-FLASH_BASE;
	uint32_t sectorPosition=offsetAddress/SECTOR_SIZE;
	uint32_t sectorStartAddress=sectorPosition*SECTOR_SIZE+FLASH_BASE;
	FLASH_ErasePage(sectorStartAddress);//擦除这个扇区
	uint16_t dataIndex;
	for(dataIndex=0;dataIndex<countToWrite;dataIndex++)
	{
	FLASH_ProgramHalfWord(startAddress+dataIndex*2,writeData[dataIndex]);
	}
	FLASH_Lock();//上锁写保护
}

int AT_CmdFunc_ANCHOR_TAG(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
		
			uint16_t write=66;
			FLASH_WriteMoreData(USER_FLASH_BASE,&write,1);
		
		
	}
	else if(opt==SET_CMD)
	{
		char *get=NULL;
		get=strtok(argv[0],",");
		uint16_t Set=atoi(get);
		uint16_t write[3]={0};
		
		if(Set==1)
		{
			//uint16_t write=1;
			write[0]=1;
			//FLASH_WriteMoreData(USER_FLASH_BASE,&write,1);
		}
		else if(Set==0)
		{
			write[0]=0;
			//uint16_t write=0;
			//FLASH_WriteMoreData(USER_FLASH_BASE,&write,1);
			
		}
		else
		{
			return -1;
		}
		
		get=NULL;
		get=strtok(NULL,",");
		Set=atoi(get);
		if(Set>=0 && Set<=255)
		{
			write[1]=Set;
			
		}
		else
		{
			return -1;
		}
		
		FLASH_WriteMoreData(USER_FLASH_BASE,write,2);
		
	}
	else if(opt==QUERY_CMD)
	{
		uint16_t buff[3]={0};
		FLASH_ReadMoreData(USER_FLASH_BASE,buff,3);
	
		printf("buff0:%d\r\n",buff[0]);
		printf("buff1:%d\r\n",buff[1]);
		printf("buff2:%d\r\n",buff[2]);
		
	}
	
	
	return 1;
}
int8 PairGpioCheck(uint32 GDMx1,uint32 GDPx1,uint32 GDMy1,uint32 GDPy1)
{
	uint32 GDMx=GDMx1;
	uint32 GDPx=GDPx1;
	uint32 GDMy=GDMy1;
	uint32 GDPy=GDPy1;
	uint8 get=0xff;

	
	dwt_setGPIOdirection(GDMx,0);
		dwt_setGPIOvalue(GDMx,GDPx);
		dwt_setGPIOdirection(GDMy,GDPy);
		get=dwt_getgpiovalue(GDPy);
		if(get==1)
		{
			//OLED_ShowString(0,3," IO 0-2 1 INIT success");
		}
		else
		{
		
			return 0;
		}
		
	dwt_setGPIOvalue(GDMx,0);
	get=dwt_getgpiovalue(GDPy);
		if(get==0)
		{
			//OLED_ShowString(0,3," IO 0-2 0 INIT success");
		}
		else
		{
		
			return 0;
		}
		
		
		
		dwt_setGPIOdirection(GDMy,0);
		dwt_setGPIOvalue(GDMy,GDPy);
		dwt_setGPIOdirection(GDMx,GDPx);
		get=dwt_getgpiovalue(GDPx);
			if(get==1)
		{
			//OLED_ShowString(0,3," IO 2-0 1 INIT success");
		}
		else
		{
		
			return 0;
		}
		
		
		dwt_setGPIOvalue(GDMy,0);
		get=dwt_getgpiovalue(GDPx);
			if(get==0)
		{
			//OLED_ShowString(0,3," IO 2-0 0 INIT success");
		}
		else
		{
		
			return 0;
		}
		return 1;
		
	


}
int buo1Gpio()
{
#if 1		
		
			uint8 get=0xff;
	 /* see NOTE 1: 1st enable GPIO clocks */
		dwt_enablegpioclocks();
		dwt_enablegpioclocks1();//io7 geneal gpio
		
		
		get=PairGpioCheck(GDM4,GDP4,GDM6,GDP6);
		if(get==0)
		{
			//OLED_ShowString(0,3," IO 4 INIT FAIL");
			printf("IO 4-6 fail\r\n");
			return 0;
		}
		else
		{
			printf("IO 4-6 success\r\n");
			//OLED_ShowString(0,3," IO 4-6 test ok");
		}
		
		get=PairGpioCheck(GDM5,GDP5,GDM7,GDP7);
		if(get==0)
		{
			//OLED_ShowString(0,0," IO 5 INIT FAIL");
			printf("IO 5-7 fail\r\n");
			return 0;
		}
		else
		{
				printf("IO 5-7 success\r\n");
			//OLED_ShowString(0,0," IO 5-7 test ok");
		}
		
		
		
#endif
		
		


		
		
}

int8 PairGpioCheckDB(GPIO_TypeDef *GPIOX,uint16_t pinx,GPIO_TypeDef *GPIOY,uint16_t piny)
{
	uint8 get=0xff;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pinx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOX, &GPIO_InitStructure);
	GPIO_SetBits(GPIOX,pinx);
	
	
	memset(&GPIO_InitStructure,0,sizeof(GPIO_InitTypeDef));
	GPIO_InitStructure.GPIO_Pin = piny;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOY, &GPIO_InitStructure);
	
	get=GPIO_ReadInputDataBit(GPIOY,piny);
	if(get==1)
	{
			//printf("PB IO 1-11 OK\r\n");
	}
	else
	{
		return 0;
		//printf("PB IO 1-11 fail \r\n");
	}
	
	GPIO_ResetBits(GPIOX,pinx);
	get=GPIO_ReadInputDataBit(GPIOY,piny);
	if(get==0)
	{
			//printf("PB IO 1-11 OK\r\n");
	}
	else
	{
		return 0;
		//printf("PB IO 1-11 fail \r\n");
	}
	
	
	memset(&GPIO_InitStructure,0,sizeof(GPIO_InitTypeDef));
	GPIO_InitStructure.GPIO_Pin = piny;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOY, &GPIO_InitStructure);
	GPIO_SetBits(GPIOY,piny);

	
	memset(&GPIO_InitStructure,0,sizeof(GPIO_InitTypeDef));	
	GPIO_InitStructure.GPIO_Pin = pinx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOX, &GPIO_InitStructure);
	
	get=GPIO_ReadInputDataBit(GPIOX,pinx);
	if(get==1)
	{
			//printf("PB IO 1-11 OK\r\n");
	}
	else
	{
		return 0;
		//printf("PB IO 1-11 fail \r\n");
	}
	
	GPIO_ResetBits(GPIOY,piny);
	get=GPIO_ReadInputDataBit(GPIOX,pinx);
	if(get==0)
	{
			//printf("PB IO 1-11 OK**************************\r\n");
	}
	else
	{
		return 0;
		//printf("PB IO 1-11 fail********************* \r\n");
	}
	
	return 1;
}
int buo1DbGpio()
{
	
		//GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  
//PB4默认用作调试口，如果用作普通的IO，需要加上以下两句 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	
	uint8 get=0xff;
	
	get=PairGpioCheckDB(GPIOB,GPIO_Pin_8,GPIOB,GPIO_Pin_9);
	if(get==0)
	{
		printf(" PB8-PB9 fail\r\n");
		return 0;
	}
	else
	{
		printf("PB8-PB9 success\r\n");
	}
		
	get=PairGpioCheckDB(GPIOA,GPIO_Pin_8,GPIOB,GPIO_Pin_15);
	if(get==0)
	{
		printf(" PA8-PB15 fail\r\n");
		return 0;
	}
	else
	{
		printf("PA8-PB15 success\r\n");
	}
	
	
	get=PairGpioCheckDB(GPIOB,GPIO_Pin_1,GPIOB,GPIO_Pin_11);//
	if(get==0)
	{
		printf(" PB1-PB11 fail\r\n");
		return 0;
	}
	else
	{
		printf("PB1-PB11 success\r\n");
	}
	
	get=PairGpioCheckDB(GPIOA,GPIO_Pin_3,GPIOB,GPIO_Pin_10);
	if(get==0)
	{
		printf(" PA3-PB10 fail\r\n");
		return 0;
	}
	else
	{
		printf("PA3-PB10 success\r\n");
	}
	
	
	
	get=PairGpioCheckDB(GPIOB,GPIO_Pin_3,GPIOB,GPIO_Pin_5);
	if(get==0)
	{
		printf(" PB3-PB5 fail\r\n");
		return 0;
	}
	else
	{
		printf("PB3-PB5 success\r\n");
	}
	
	
	get=PairGpioCheckDB(GPIOA,GPIO_Pin_15,GPIOB,GPIO_Pin_4);
	if(get==0)
	{
		printf(" PA15-PB4 fail\r\n");
		return 0;
	}
	else
	{
		printf("PA15-PB4 success\r\n");
	}

	

		
	return 1;
}

int AT_CmdFunc_ANCHOR_GPIOTEST(int opt,int argc, char *argv[])
{
	
	if(opt==EXECUTE_CMD)
	{
		//GPIO_SetBits(GPIOA,GPIO_Pin_1);
		if(buo1Gpio()==0)
		{
			return -1;
		
		}
		if(buo1DbGpio()==0)
		{
			return -1;
		
		}
		
	}
	else if(opt==SET_CMD)
	{
		
	}
	else if(opt==QUERY_CMD)
	{
	
		
	}
	
	
	return 1;
}

int AT_CmdFunc_FACTORY(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
		
		
		uint16_t write[3]={1,0,0xff};//MODE ID SAVE
		FLASH_WriteMoreData(0x08010000,write,3);
		
		
	}
	else if(opt==SET_CMD)
	{
		//uint16_t buff[3]={0};
		//FLASH_ReadMoreData(0x08010000,buff,3);
		//printf("buff0:%d\r\n",buff[0]);
		//printf("buff1:%d\r\n",buff[1]);
		//printf("buff2:%d\r\n",buff[2]);
	}
	else if(opt==QUERY_CMD)
	{
		//extern const int Filter_D_set;
	//Filter_D_set=5;
		
	}
	
	
	return 1;
}

int AT_CmdFunc_AT(int opt,int argc, char *argv[])
{
	//phy_printf("aliGenie set opt:%d\r\n",opt);
	if(opt==EXECUTE_CMD)
	{
		
		
		
		//uint16_t write[3]={1,0,0xff};//MODE ID SAVE
		//FLASH_WriteMoreData(0x08010000,write,3);
		
		
	}
	else if(opt==SET_CMD)
	{
		//uint16_t buff[3]={0};
		//FLASH_ReadMoreData(0x08010000,buff,3);
		//printf("buff0:%d\r\n",buff[0]);
		//printf("buff1:%d\r\n",buff[1]);
		//printf("buff2:%d\r\n",buff[2]);
	}
	else if(opt==QUERY_CMD)
	{
		//extern const int Filter_D_set;
	//Filter_D_set=5;
		
	}
	
	
	return 1;
}
at_cmd_t at_table[] = {
	
	{"AT+switchdis",  AT_CmdFunc_ondis},
	{"AT+interval",  AT_CmdFunc_interval},
	{"AT+version",  AT_CmdFunc_version},
	{"AT+RST",  AT_CmdFunc_RST},
	{"AT+tem_hum",  AT_CmdFunc_TemHum},
	{"AT+xyz",  AT_CmdFunc_XYZ},
	{"AT+anchor_tag",  AT_CmdFunc_ANCHOR_TAG},
	{"AT+GPIOTEST",  AT_CmdFunc_ANCHOR_GPIOTEST},
	{"AT+FACTORY",  AT_CmdFunc_FACTORY},
	{"AT",  AT_CmdFunc_AT},
	
};

#define AT_TABLE_SIZE	(sizeof(at_table) / sizeof(at_cmd_t))
/*
 * @brief 字符串拆分解析处理
 * @return 检测归类的参数个数
 **/
int string_split(char *strp, uint32_t strsize, char ch, char *argv[], uint32_t argcM )
{
	int ch_index = 0;
	int argc_index = 0;
	//uint8_t spliflag = 0;
	uint8_t splitflag = 0;
	
	if ((!strsize) || (!argcM)) return 0;

	argv[argc_index++] = &strp[ch_index];
	for (ch_index = 0; ch_index < strsize; ch_index++) {
		if (strp[ch_index] == 'ch') {
			strp[ch_index] = '\0';
#if 0	/* 处理连续出现ch情况 */
			if (1 == splitflag) {
				argv[argc_index++] = &strp[ch_index];
			}
#endif			
			splitflag = 1;
		} else if (splitflag == 1) {
			splitflag = 0;
			argv[argc_index++] = &strp[ch_index];
			if (argc_index >= argcM) break;
		} else {
			splitflag = 0;
		}
	}
	
	return argc_index;
}

#define respond_error()	printf("\r\nERROR\r\n")
#define respond_ok()		printf("\r\nOK\r\n");

int at_cmd_parse(uint8_t *pdata, uint16_t size)
{
	int ret  = -1;
	char *ptr = NULL;
	int argc = ARGC_LIMIT;
	uint16_t offset = 0;
	int index = 0;
	char *argv[ARGC_LIMIT] = { (char *)0 };

	if (strstr((const char *)pdata, "AT") == NULL) goto at_end;
	for (index = 0; index < AT_TABLE_SIZE; index++) {
		ptr = strstr((const char *)pdata, at_table[index].cmd);
		if (ptr != NULL) {
			ptr += strlen(at_table[index].cmd);
			offset = ptr - (char *)pdata;
			break;
		}
	}
	if (index >= AT_TABLE_SIZE) goto at_end;

	/* 解析查询命令 */
	if ((ptr[0] == '?') && (ptr[1] == '\r') && (ptr[2] == '\n')) {
		if (NULL != at_table[index].deal_func) {
			ret = at_table[index].deal_func(QUERY_CMD, argc, argv);
		}
	} else if ((ptr[0] == '\r') && (ptr[1] == '\n')) { /* 解析执行命令 */
		if (NULL != at_table[index].deal_func) {
			ret = at_table[index].deal_func(EXECUTE_CMD, argc, argv);
		}
	} else if (ptr[0] == '=') { /* 解析设置命令 */
		ptr += 1;
		argc = string_split((char*)ptr, size - offset, ',', argv, argc);
		if (NULL != at_table[index].deal_func) {
			ret = at_table[index].deal_func(SET_CMD, argc, argv);
		}
	} else {
		ret = -1;
	}

at_end:
	if (-1 == ret) respond_error();
	else respond_ok();

	return ret;
}

#define AT_RX_TIMEOUT	200	/* ms */
#define AT_RX_BUF_SIZE	512	/* bytes */
uint32 HAL_GetTick()
{
	return portGetTickCount();
}
static uint8_t cmdbuf[AT_RX_BUF_SIZE];
int at_cmd_recv(uint8_t data)
{
	
	int ret = -1;
	static uint16_t index = 0;
	static uint32_t tick = 0;
	static uint8_t flag = 0;

	if (((HAL_GetTick() - tick) > AT_RX_TIMEOUT) || (index >= AT_RX_BUF_SIZE	)) {
		index = 0;
		flag = 0;
	}
	tick = HAL_GetTick();
	cmdbuf[index++] = data;
	if ((data == '\n') && flag==1 ) {
		ret = at_cmd_parse(cmdbuf, index);
		
		flag = 0;
		index = 0;
	} if (data == '\r' ) {
		flag = 1;
	}else {
		flag = 0;
	}

	return ret;
 }//change by johhn

void USART1_IRQHandler(void)
{
	//printf("hello dwm1000!\r\n");
	char buff;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//如果寄存器中有数据

  {

    /* Read one byte from the receive data register */
		buff=0;
		buff = USART_ReceiveData(USART1);
		#define TAG
		#ifdef TAG
		at_cmd_recv(buff);
		#endif
		//printf("hello dwm1000  %c!\r\n",buff);

  }
	

}
/**
  * @brief  DeInitializes the SDIO interface.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);
  
  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();
  
  /*!< Disable the SDIO AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, DISABLE);

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /*!< Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< GPIOC and GPIOD Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | SD_DETECT_GPIO_CLK, ENABLE);

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /*!< Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);
  
  /*!< Enable the SDIO AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

  /*!< Enable the DMA2 Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{

  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /*!< DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);  
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /*!< DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE); 
}

/**
  * @brief  Returns the DMA End Of Transfer Status.
  * @param  None
  * @retval DMA SDIO Channel Status.
  */
uint32_t SD_DMAEndOfTransferStatus(void)
{
  return (uint32_t)DMA_GetFlagStatus(DMA2_FLAG_TC4);
}

/**
  * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< Disable the sFLASH_SPI  */
  SPI_Cmd(sFLASH_SPI, DISABLE);
  
  /*!< DeInitializes the sFLASH_SPI */
  SPI_I2S_DeInit(sFLASH_SPI);
  
  /*!< sFLASH_SPI Periph clock disable */
  RCC_APB2PeriphClockCmd(sFLASH_SPI_CLK, DISABLE);
  
  /*!< Configure sFLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MISO_PIN;
  GPIO_Init(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MOSI_PIN;
  GPIO_Init(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = sFLASH_CS_PIN;
  GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< sFLASH_SPI_CS_GPIO, sFLASH_SPI_MOSI_GPIO, sFLASH_SPI_MISO_GPIO 
       and sFLASH_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(sFLASH_CS_GPIO_CLK | sFLASH_SPI_MOSI_GPIO_CLK | sFLASH_SPI_MISO_GPIO_CLK |
                         sFLASH_SPI_SCK_GPIO_CLK, ENABLE);

  /*!< sFLASH_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(sFLASH_SPI_CLK, ENABLE);
  
  /*!< Configure sFLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MOSI_PIN;
  GPIO_Init(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = sFLASH_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  DeInitializes the LM75_I2C.
  * @param  None
  * @retval None
  */
void LM75_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< Disable LM75_I2C */
  I2C_Cmd(LM75_I2C, DISABLE);
  /*!< DeInitializes the LM75_I2C */
  I2C_DeInit(LM75_I2C);
  
  /*!< LM75_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(LM75_I2C_CLK, DISABLE);
    
  /*!< Configure LM75_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LM75_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure LM75_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SDA_PIN;
  GPIO_Init(LM75_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure LM75_I2C pin: SMBUS ALERT */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SMBUSALERT_PIN;
  GPIO_Init(LM75_I2C_SMBUSALERT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the LM75_I2C..
  * @param  None
  * @retval None
  */
void LM75_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< LM75_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(LM75_I2C_CLK, ENABLE);
    
  /*!< LM75_I2C_SCL_GPIO_CLK, LM75_I2C_SDA_GPIO_CLK 
       and LM75_I2C_SMBUSALERT_GPIO_CLK Periph clock enable */
  RCC_APB2PeriphClockCmd(LM75_I2C_SCL_GPIO_CLK | LM75_I2C_SDA_GPIO_CLK |
                         LM75_I2C_SMBUSALERT_GPIO_CLK, ENABLE);
  
  /*!< Configure LM75_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(LM75_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure LM75_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SDA_PIN;
  GPIO_Init(LM75_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure LM75_I2C pin: SMBUS ALERT */
  GPIO_InitStructure.GPIO_Pin = LM75_I2C_SMBUSALERT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(LM75_I2C_SMBUSALERT_GPIO_PORT, &GPIO_InitStructure); 
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
