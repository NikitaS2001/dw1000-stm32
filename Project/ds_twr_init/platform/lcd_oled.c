#include "lcd_oled.h"
#include "stdlib.h"
#include "lcd_oledfont.h"
#include "deca_sleep.h"

#include <math.h>
//OLED memory
//The storage format is as follows.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127



///I2C related
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : delay
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
static void I2C_delay(uint16_t cnt)
{
    while(cnt--);
}

#define IIC_TIMEOUT 100
int IIc_flag = 1 ;
/*******************************************************************************
* Function Name  : I2C_AcknowledgePolling
* Description    : Waiting for the right to control the I2C bus. Assessment of employment status
* Input          : - I2Cx:I2C register base address
*                  - I2C_Addr:slave address
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
static void I2C_AcknowledgePolling(I2C_TypeDef *I2Cx,uint8_t I2C_Addr)
{
    vu16 SR1_Tmp;
    uint16_t timeout = 10000;
    timeout = IIC_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2Cx, ENABLE);
        SR1_Tmp = I2C_ReadRegister(I2Cx, I2C_Register_SR1);
        I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);

    }
    while(!(I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0x0002)&&(timeout--));
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2Cx, ENABLE);
}


/*******************************************************************************
* Function Name: I2C_WriteOneByte
* Description : Write a byte of data through the specified I2C interface
* Input : - I2Cx: I2C register base address
                   - I2C_Addr: slave device address
                   - addr: pre-written byte address
                   - value: write data
* Output : None
* Return : return 0 successfully
* Attention : None
*******************************************************************************/
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{
    uint16_t timeout = 0;
    /* start bit */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) &&(timeout--));
    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);

    if(timeout == 0)
        IIc_flag = 0;

    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)&&(timeout--));
    /*sending address*/
    I2C_SendData(I2Cx, addr);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout--));
    /*write a byte*/
    I2C_SendData(I2Cx, value);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&&(timeout--));
    /*stop bit*/
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /*stop bit flag*/
		timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)&&(timeout--) );

    //I2C_AcknowledgePolling(I2Cx,I2C_Addr);
		I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    I2C_delay(80);
    return 0;
}


uint8_t I2C_ReadOneByte(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte)
{
    uint8_t ReceiveData = 0;
		
		int16_t timeout = 0;
		timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) &&(timeout--));
    I2C_GenerateSTART(I2Cx, ENABLE);
		
		timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) &&(timeout--) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
		timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) ); // wait send Address Device

    I2C_SendData(I2Cx, AddressByte);
		timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) &&(timeout--) ); // wait send Address Byte

    I2C_GenerateSTART(I2Cx, ENABLE);
		timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout--) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
		timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)&&(timeout--) ); // wait Send Address Device As Receiver

		timeout = IIC_TIMEOUT;
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) &&(timeout--) ); // wait Receive a Byte
    ReceiveData = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    I2C_GenerateSTOP(I2Cx, ENABLE);
		timeout = IIC_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) &&(timeout--) ); // wait Generate Stop Condition

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    return ReceiveData;
};


/*******************************************************************************
* Function Name : I2C_Write
* Description : Write multiple bytes of data through the specified I2C interface
* Input : - I2Cx: I2C register base address
                   - I2C_Addr: slave device address
                   - addr: pre-written byte address
                   - *buf: pre-written data storage location
                   - num: the number of bytes written
* Output : None
* Return : return 0 successfully
* Attention : None
*******************************************************************************/
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num)
{
    uint8_t err=0;

    while(num--)
    {
        if(I2C_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++))
        {
            err++;
        }
    }
    if(err)
        return 1;
    else
        return 0;
}


/*
//send data
//slaveAddr: The 7-bit address of the slave machine, the direction of the eighth bit will be added automatically
//pdata: the address of the data to be sent
//len: the length of the data to be sent
*/
uint8_t hal_i2c1_send_data1(I2C_TypeDef *I2Cx,uint8_t I2C_Addr, uint8_t pdata)
{
		
		uint16_t timeout = 0;
    /* start bit */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) &&(timeout--));
	//printf("\r\nstart\r\n");
	
    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
    if(timeout == 0)
        IIc_flag = 0;
    timeout = IIC_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) );//wake up device
		//printf("\r\nweak up\r\n");
		
    /*send data*/
			
		I2C_SendData(I2Cx,pdata);
		timeout = IIC_TIMEOUT;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) &&(timeout--) );
		
		//printf("\r\nsend data\r\n");


    /*stop bit*/
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /*stop bit flag*/
		timeout = IIC_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) &&(timeout--));

		//I2C_AcknowledgePolling(I2Cx,I2C_Addr);
		I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
    I2C_delay(9000);
		//I2C_delay(0xfff0);
		//printf("\r\nstop\r\n");
    return 0;

}
uint8_t hal_i2c1_send_data(I2C_TypeDef *I2Cx,uint8_t I2C_Addr, uint8_t *pdata,uint32_t u32_len)
{
		 /*send data*/
		for(uint32_t u32_count=0;u32_count<(u32_len);u32_count++){
				
			
			hal_i2c1_send_data1(I2Cx,I2C_Addr,*pdata);
			pdata++;
			I2C_delay(10000);
			//I2C_delay(0xfff0);
			//printf("\r\nsend ok\r\n");
		};

}
int hal_i2c1_read_data(I2C_TypeDef *I2Cx, unsigned char AddressDevice, uint8_t *pdata,uint32_t u32_len)
{
	uint8_t ReceiveData = 0;
	int timeout = 0;
	timeout = IIC_TIMEOUT;
		//printf("\r\nread start\r\n");
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) &&(timeout--) );
    I2C_GenerateSTART(I2Cx, ENABLE);
		timeout = IIC_TIMEOUT;
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) &&(timeout--) ); // wait Generate Start
		//printf("\r\nread start\r\n");

		timeout = IIC_TIMEOUT;
    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) &&(timeout--) ); // wait send Address Device
	//printf("\r\nread weak up timeout:%d\r\n",timeout);
	if(timeout<=0)
	{
		I2C_GenerateSTOP(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
		//printf("\r\nread weak up timeout:%d\r\n",timeout);
		return -1;
	}
	
		timeout = IIC_TIMEOUT;
    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)&&(timeout--) ); // wait Generate Start

		timeout = IIC_TIMEOUT;
    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)&&(timeout--) ); // wait Send Address Device As Receiver
		//printf("\r\nread rec\r\n");
		
    /*data acceptance*/
		for(uint32_t u32_count=0;u32_count<(u32_len-1);u32_count++){
				timeout = IIC_TIMEOUT;
			while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) &&(timeout--) ); // wait Receive a Byte
			*pdata = I2C_ReceiveData(I2Cx);
			pdata++;
			
			I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next); // 
			I2C_AcknowledgeConfig(I2Cx, ENABLE);// disable acknowledge
			//printf("\r\nread Ack\r\n");
		};

    

		*pdata = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
		//printf("\r\nread no Ack\r\n");

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge
		//printf("\r\nread ok\r\n");
    return ReceiveData;

}


#define SHT20_ADDR					0x80//0x40
#define SHT20_WRITE_REG				0xE6	//write register instruction
#define SHT20_READ_REG				0xE7	//read register instruction
#define TRIG_TEMP_MEASUREMENT_POLL	0xF3	//trigger temperature measurement
#define TRIG_HUMI_MEASUREMENT_POLL	0xF5	//trigger humidity measurement
#define TRIG_RESET					0xFE	//reset command

//reset the sensor
void SHT20_init(void){
	
	uint8_t u8_cmd_buf[2]={0},u8_reg=0;

	//reset
	u8_cmd_buf[0]=TRIG_RESET;	 //send reset command
	
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);

	//deca_sleep(100);	//reset time is less than 15ms
	I2C_delay(9000);
	
	//read register
	u8_cmd_buf[0]=SHT20_READ_REG;	 //send read register command
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);

	
	hal_i2c1_read_data(I2C1,SHT20_ADDR,&u8_reg,1);
	//printf("u8_reg:%X\r\n",u8_reg);
	

	//printf("SHT20_WRITE_REG\r\n");
	//write register
	//u8_cmd_buf[0]=SHT20_WRITE_REG;	 //send write register command
	//u8_cmd_buf[1]=0x02;				 //register value
	//hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,2);
	I2C_WriteOneByte(I2C1,0x80,SHT20_WRITE_REG,0x2);
	
	
	//read register
	u8_cmd_buf[0]=SHT20_READ_REG;	 //send read register command
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);
	
	hal_i2c1_read_data(I2C1,SHT20_ADDR,&u8_reg,1);
	//printf("u8_reg1:%X\r\n",u8_reg);
	

	return;
}
//read temperature
int SHT20_read_tem(float *p_temperature){
	
	uint8_t u8_cmd_buf[2]={0},u8_data_buf[3]={0},u8_count=0;

	u8_cmd_buf[0]=TRIG_TEMP_MEASUREMENT_POLL;	 //trigger humidity measurement
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);
	
int h_ret=1;
	u8_count=0;
	while(1)
		{
			I2C_delay(0xfffe);
            //keep reading until the read succeeds or times out
			h_ret=hal_i2c1_read_data(I2C1,SHT20_ADDR,u8_data_buf,3);
			if(0==h_ret){
				//the answer is successful and jump out of the loop
				break;
			}
			
			if(u8_count>20){
			printf("SHT20_read_tem time out 03");
			return -1;
			}
		u8_count++;
	}

	u8_data_buf[1] &= ~0x0003;
	
	//data processing
	*p_temperature=(u8_data_buf[0]<<8)|u8_data_buf[1];
	*p_temperature=((*p_temperature)*0.00268127)-46.85;

	return 0;
}

//read humidity
int SHT20_read_hum(float *p_humidity){
	
	uint8_t u8_cmd_buf[2]={0},u8_data_buf[3]={0},u8_count=0;

	u8_cmd_buf[0]=TRIG_HUMI_MEASUREMENT_POLL;	 //trigger humidity measurement
	hal_i2c1_send_data(I2C1,SHT20_ADDR,u8_cmd_buf,1);
	

	u8_count=0;
	int h_ret=1;
	while(1){
		I2C_delay(0xfffe);
        //keep reading until the read succeeds or times out
		h_ret=hal_i2c1_read_data(I2C1,SHT20_ADDR,u8_data_buf,3);
		if(0==h_ret){
			//the answer is successful and jump out of the loop
			break;
		}
		if(u8_count>20){
			printf(0,"SHT20_read_tem time out 03");
			return -1;
		}
		u8_count++;
	}

	u8_data_buf[1] &= ~0x0003;
	
	//data processing
	*p_humidity=(u8_data_buf[0]<<8)|u8_data_buf[1];
	*p_humidity=((*p_humidity)*0.00190735)-6;

	return 0;
}


#define LIS2DH12_FROM_FS_2g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 1.0f
#define LIS2DH12_FROM_FS_4g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 2.0f
#define LIS2DH12_FROM_FS_8g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 4.0f
#define LIS2DH12_FROM_FS_16g_HR_TO_mg(lsb) (float)((int16_t)lsb>>4)*12.0f
#define LIS2DH12_FROM_LSB_TO_degC_HR(lsb)  (float)((int16_t)lsb>>6)/4.0f+25.0f

/* iic communication related operations */
int32_t drv_lis2dh12_iic_write_byte(uint8_t addr, uint8_t data)
{
	I2C_WriteOneByte(I2C1,0x32,addr,data);
	return 0;

}
int32_t drv_lis2dh12_iic_read_byte(uint8_t AddressByte, uint8_t* data)
{
	uint8_t ReceiveData = 0;
	I2C_TypeDef *I2Cx=I2C1;
	uint8_t AddressDevice=0x32;

    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address Device

    I2C_SendData(I2Cx, AddressByte);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) ); // wait send Address Byte

    I2C_GenerateSTART(I2Cx, ENABLE);
    while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

    I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ); // wait Send Address Device As Receiver

    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ); // wait Receive a Byte
    ReceiveData = I2C_ReceiveData(I2Cx);

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition

    I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

		*data=ReceiveData;
    //return ReceiveData;
		
	//*data=I2C_ReadOneByte(I2C1,0x32,addr);
	return 0;
}

#define lis2dh12_iic_write_byte drv_lis2dh12_iic_write_byte
/* lis2dh12 init configure detection thresholds and interrupts */
int32_t drv_lis2dh12_init()
{
/* Initialization of sensor */
	lis2dh12_iic_write_byte(0x20, 0x37);	/* CTRL_REG1(20h): Turn off the sensor, set to enter power-down mode ODR 25HZ */
//	lis2dh12_iic_write_byte(0x20, 0x57);	/* CTRL_REG1(20h): Turn off the sensor, set to enter low power consumption mode ODR 100HZ */
	lis2dh12_iic_write_byte(0x21, 0x03);	/* CTRL_REG2(21h): IA1, IA2 enable high-pass filter bc */
	lis2dh12_iic_write_byte(0x22, 0xc0);    /* CTRL_REG3(22h): 0x80 Enable click interrupt to INT_1 INT_2 */
	lis2dh12_iic_write_byte(0x23, 0x88);    /* CTRL_REG4(23h): enable fast, data update, full scale +/-2G, very precision mode */
//	lis2dh12_iic_write_byte(0x25, 0x00);    /* CTRL_REG6(25h): High level (rising edge) triggers interrupt */
	
	/* INT1 toggle detection, interrupt*/ //0x6a
	lis2dh12_iic_write_byte(0x30, 0x7f);  /* INT1_CFG(30h): Enable, 6D X/Y/Z cross threshold interrupt */
//	lis2dh12_iic_write_byte(0x30, 0x4f);  /* INT1_CFG(30h): Enable, 6D X/Y either exceeds the threshold interrupt */
//  lis2dh12_iic_write_byte(0x31, 0x20);  /* INT1_SRC(31h): Set interrupt source */
 
	lis2dh12_iic_write_byte(0x32, 0x02);  	/* INT1_THS(32h): Set interrupt threshold 0x10: 16*2(FS) 0x20: 32*16(FS) */
 
//	lis2dh12_iic_write_byte(0x33, 0x02);  	/* INT1_DURATION(33h): 1LSB=1/ODR If ODR=25HZ, then 1LSB=40ms, set the delay to 1s, corresponding to 25->0x19 */
//	lis2dh12_iic_write_byte(0x33, 0x03);  /* INT1_DURATION(33h): 1LSB=1/ODR If ODR=50HZ, then 1LSB=20ms, set the delay to 1s, corresponding to 50->0x32 */
  lis2dh12_iic_write_byte(0x33, 0x03);  	/* INT1_DURATION(33h): 1LSB=1/ODR If ODR=100HZ, then 1LSB=10ms, set the delay to 1s, corresponding to 100->0x64 */
 
	
//	/* INT2 click interrupt */
////	lis2dh12_iic_write_byte(0x24, 0x01);	/* CTRL_REG5(24h):  */
//	lis2dh12_iic_write_byte(0x25, 0xa0);  /f* CTRL_REG6(25h): Click interrupt on INT2 pin */
//
//	lis2dh12_iic_write_byte(0x38, 0x15);	/* CLICK_CFG (38h): Click recognition interrupt enable */
////	lis2dh12_iic_write_byte(0x39, 0x10);
//	lis2dh12_iic_write_byte(0x3a, 0x7f);  /* CLICK_THS (3Ah): click threshold */
//	lis2dh12_iic_write_byte(0x3b, 0xff);  /* TIME_LIMIT (3Bh): Time limit window 6 ODR 1LSB=1/ODR 1LSB=1/100HZ, 10ms, set delay 1s, corresponding to 100¡ª>0x64*/
//	lis2dh12_iic_write_byte(0x3c, 0xff);  /* TIME_LATENCY (3Ch): Interrupt level duration 1 ODR=10ms */
//	lis2dh12_iic_write_byte(0x3d, 0x01);  /* TIME_WINDOW (3Dh):  click time window */
	
	/* Start sensor */
//	lis2dh12_iic_write_byte(0x20, 0x37);
	lis2dh12_iic_write_byte(0x20, 0x5f);  /* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */
	


}
int32_t drv_lis2dh12_initqq()
{
	/* Initialization of sensor */
	drv_lis2dh12_iic_write_byte(0x21, 0x01); /* CTRL_REG2(21h): Filtered data and High-pass filter selection */
	
		
	drv_lis2dh12_iic_write_byte(0x22, 0x40); /* CTRL_REG3(22h): IA1 interrupt on INT1 pin */
	
	
	drv_lis2dh12_iic_write_byte(0x23, 0x80); /* CTRL_REG4(23h): Set Full-scale to +/-2g */

	
	/* Wakeup recognition enable */ 
	drv_lis2dh12_iic_write_byte(0x30, 0x2a); /* INT1_CFG(30h): INT1 Configuration */
	
	drv_lis2dh12_iic_write_byte(0x32, 0x36); /* INT1_THS(32h): INT1 Threshold set */

	
	drv_lis2dh12_iic_write_byte(0x33, 0x0);/* INT1_DURATION(33h): INT1 Duration set */
	
	
	/* Start sensor */
	drv_lis2dh12_iic_write_byte(0x20, 0x57);/* CTRL_REG1(20h): Start sensor at ODR 100Hz Low-power mode */ 
		
	
	return 0;
}

/* Get the current tilt angle */
int drv_lis2dh12_get_angle( float *get)
{
	;
	float acc_x, acc_y, acc_z, acc_g;
	float angle_x, angle_y, angle_z, angle_xyz;
	int8_t data[6]={0};
	uint8_t i;
	
	for (i=0; i<6; i++)
	{
		drv_lis2dh12_iic_read_byte(0x28 + i, data+i);
		//printf("i:%d data:%d\r\n",i,data[i]);
	}
		
	
	/* x, y, z axis acceleration */
	acc_x = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)data));
	acc_y = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+2)));
	acc_z = (LIS2DH12_FROM_FS_2g_HR_TO_mg(*(int16_t*)(data+4)));
	
	printf("acc_x:%f\r\n",acc_x);
	printf("acc_y:%f\r\n",acc_y);
	printf("acc_z:%f\r\n",acc_z);
	
	acc_x = abs(acc_x);
	acc_y = abs(acc_y);
	acc_z = abs(acc_z);

	

	/* acceleration of gravity */
	acc_g = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
	//printf("acc_g:%f\r\n",acc_g);
	
	//return acc_y;
	if (acc_z > acc_g)
		acc_z = acc_g;

	/* angle_z/90 = asin(acc_z/acc_g)/¦Ð/2 */
	angle_z = asin(acc_z/acc_g) * 2 / 3.14 * 90;
	angle_z = 90 - angle_z;
	if(angle_z < 0)
		angle_z = 0;
	*get=angle_z;
	return 0;
}


//STM32F103C8T6 IIC OLED
#ifdef STM32F10X_MD
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    if(IIc_flag == 1)
    {
        if(cmd)
        {
            I2C_WriteOneByte(I2C1,0x78,0x40,dat);
        }
        else
        {
            I2C_WriteOneByte(I2C1,0x78,0x00,dat);
        }
    }
}

#else
//STM32F103RCT6 SPI OLED
///I2C related end


#if OLED_MODE==1

void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    DATAOUT(dat);
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    OLED_WR_Clr();
    OLED_WR_Set();
    OLED_CS_Set();
    OLED_DC_Set();
}
#else

void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    uint8_t i;
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    for(i=0; i<8; i++)
    {
        OLED_SCLK_Clr();
        if(dat&0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat<<=1;
    }
    OLED_CS_Set();
    OLED_DC_Set();
}
#endif
#endif
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}
//Turn on the OLED display
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//Turn off the OLED display
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//Screen clearing function
void OLED_Clear(void)
{
    uint8_t i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //Setting page address (0~7)
        OLED_WR_Byte (0x00,OLED_CMD);      //Set display position¡ªcolumn low address
        OLED_WR_Byte (0x10,OLED_CMD);      //Set display position¡ªcolumn height address
        for(n=0; n<128; n++)OLED_WR_Byte(0,OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';
    if(x>Max_Column-1)
    {
        x=0;
        y=y+2;
    }
    if(SIZE ==16)
    {
        OLED_Set_Pos(x,y);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
    }
    else
    {
        OLED_Set_Pos(x,y+1);
        for(i=0; i<6; i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);

    }
}
//m^n
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}

void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size/2)*t,y,' ');
                continue;
            }
            else enshow=1;

        }
        OLED_ShowChar(x+(size/2)*t,y,temp+'0');
    }
}
//display a character string
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        OLED_ShowChar(x,y,chr[j]);
        x+=8;
        if(x>120)
        {
            x=0;
            y+=2;
        }
        j++;
    }
}

/* Function description: Display the starting point coordinates (x, y) of BMP picture 128¡Á64,
    the range of x is 0~127, and the range of y is 0~7 of the page*/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
    unsigned int j=0;
    unsigned char x,y;

    if(y1%8==0) y=y1/8;
    else y=y1/8+1;
    for(y=y0; y<y1; y++)
    {
        OLED_Set_Pos(x0,y);
        for(x=x0; x<x1; x++)
        {
            OLED_WR_Byte(BMP[j++],OLED_DATA);
        }
    }
}


//initialize SSD1306
void OLED_Init(void)
{
#ifdef STM32F10X_HD
//STM32F103RC SPI OLED
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //Enable A port clock
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //push-pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//Speed 50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);    //Initialize GPIO 3,6
    GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15);

    OLED_RST_Set();
    deca_sleep(100);
    OLED_RST_Clr();
    deca_sleep(200);
    OLED_RST_Set();
#endif

    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);
}
