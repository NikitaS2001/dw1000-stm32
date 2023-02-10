/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"
#include "lcd_oled.h"
#include "trilateration.h"
#include <math.h>
#include "kalman.h"
#include "AT24C02.h"
#include "stm32_eval.h"
#include "lib.h"


extern char dist_str[16];
extern uint8_t TAG_ID;
extern uint8_t MASTER_TAG;
extern uint8_t SLAVE_TAG_START_INDEX;
extern uint8_t ANCHOR_IND; 
extern uint8_t ANCHOR_IND; 
extern uint8 Semaphore[MAX_SLAVE_TAG];

vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];

int Anthordistance_count[ANCHOR_MAX_NUM];

int ANCHOR_REFRESH_COUNT_set=5;
#define ANCHOR_REFRESH_COUNT ANCHOR_REFRESH_COUNT_set

/* Private macro ---------- ---------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}

void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}
void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}


void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
	frame_seq_nb=0;//change by johhn
    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;//基站收到标签的信息，里面有TAG_ID,在基站回复标签的时候，也需要指定TAG_ID,只有TAG_ID一致才做处理

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);//这个后加的，默认tx后应该自动切换rx，但是目前debug 发现并没有自动打开，这里强制打开rx
				uint32 tick1=portGetTickCount();
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        {
									if((portGetTickCount() - tick1) > 350)
									{
										break;
									}

				};
        GPIO_SetBits(GPIOA,GPIO_Pin_1);

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)//检测TAG_ID
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                //TODO maybe need longer time
                //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*2);
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { 
										if((portGetTickCount() - tick1) > 500)
									{
										break;//change by johhn
									}
								
								};

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                       // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        Anthordistance_count[rx_buffer[12]] ++;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >=ANCHOR_REFRESH_COUNT )
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
									//clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
										Anchor_Index++;
                                    }
                                    break;
                                }
								Anchor_Index++;
                            }
                        }
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            // sprintf(dist_str, "%08x",status_reg);
            // OLED_ShowString(0, 2,"           ");
            // OLED_ShowString(0, 2,dist_str);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}

double final_distance =  0;

int main(void)
{
    uint8 anthor_index = 0;
    uint8 tag_index = 0;

    uint8 Semaphore_Enable = 0 ;
    uint8 Waiting_TAG_Release_Semaphore = 0;
    int8 frame_len = 0;
		//lable1:
    /* Start with board specific hardware init. */
    peripherals_init();
	
    printf("hello dwm1000!\r\n");

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */

	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();

		
    if(dwt_initialise(DWT_LOADUCODE) == -1)
    {
        printf("dwm1000 init fail!\r\n");
        OLED_ShowString(0,0,"INIT FAIL");
        while (1)
        {
					  //STM_EVAL_LEDOn(LED1);
					GPIO_SetBits(GPIOC,GPIO_Pin_13);
            deca_sleep(1000);
            //STM_EVAL_LEDOff(LED1);
					GPIO_ResetBits(GPIOC,GPIO_Pin_13);
            deca_sleep(1000);
					
 
        }
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);
    dwt_setleds(1);
		
    /* Apply default antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    OLED_ShowString(0,0,"INIT PASS");

    printf("init pass!\r\n");
		printf("AIT-BU01-DB V100 T2020-5-17\r\n");
		
		
    AnchorList[0].x =0.12;
    AnchorList[0].y =0.34;
    AnchorList[0].z =0;

    AnchorList[1].x =0.25;
    AnchorList[1].y =0;
    AnchorList[1].z =0;

    AnchorList[2].x =0;
    AnchorList[2].y =0;
    AnchorList[2].z =0;
    int rx_ant_delay =32880;
    int index = 0 ;
		
	extern UserSet UserSetNow;
	uint16_t buff[3]={1,0,0xff};//默认值
	FLASH_ReadMoreData(USER_FLASH_BASE,buff,3);
	if(buff[0]==1)
	{
		UserSetNow.ANCHOR_TAG=1;
	}
	else if(buff[0]==0)
	{
		UserSetNow.ANCHOR_TAG=0;
	}
	else
	{
		UserSetNow.ANCHOR_TAG=1;
	}
	
//#ifdef ANTHOR
	
if(UserSetNow.ANCHOR_TAG==1)
{
	if(buff[1]>=0 && buff[1]<=255)
	{
		UserSetNow.ID=buff[1];
		ANCHOR_IND=UserSetNow.ID;
	}
	printf("device:anchor ID:%d\r\n",ANCHOR_IND);
	
			    Anchor_Array_Init();
    /* Loop forever initiating ranging exchanges. */
    OLED_ShowString(0,0,"DS TWR ANTHOR");
    //OLED_ShowString(0,2,"Distance:");

    //KalMan_PramInit();
		ANTHOR_MEASURE();

	
}

//#endif

//#ifdef TAG

if(UserSetNow.ANCHOR_TAG==0)
{
	
	if(buff[1]>=0 && buff[1]<=255)
	{
		UserSetNow.ID=buff[1];
		TAG_ID=UserSetNow.ID;
		MASTER_TAG=TAG_ID;
	}
	
	printf("device:TAG ID:%d\r\n",UserSetNow.ID);
	if(TAG_ID == MASTER_TAG)
    {
        OLED_ShowString(0,0,"DS MASTER TAG:");
    }
    else
    {
        OLED_ShowString(0,0,"DS SLAVE TAG:");
    }
		
    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    

    //OLED_ShowString(0,2,"Distance:"); chnage by johhn

    if(TAG_ID ==  MASTER_TAG)
    {
        Semaphore_Enable = 1 ;
        Semaphore_Init();
        Waiting_TAG_Release_Semaphore = 0;
    }
    else
    {
        Semaphore_Enable = 0 ;
    }
    //Master TAG0
			
		TAG_MEASURE();
  


}
		
//#endif
}

#define Filter_N 5  //max filter use in this system
const int Filter_D_set=5;
#define Filter_D Filter_D_set  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

static void distance_mange(void)
{
    // if(GetLocation(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0])) != -1)
    // {
    // printf("Tag Location:x=%3.2fm y=%3.2fm z=%3.2fm\r\n",tag_best_solution.x,tag_best_solution.y,tag_best_solution.z);
    // sprintf(dist_str, "x:%3.2f y:%3.2f",tag_best_solution.x,tag_best_solution.y);
    // OLED_ShowString(0,0,dist_str);
    //  }
    //GetLocation2(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location( &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location2( &(AnchorList[0]), &(Anthordistance[0]));
    // OLED_ShowString(0, 2,"pass");

    //printf("Count:%d %d %d \r\n",Anthordistance[0]/Anthordistance_count[0],Anthordistance[1]/Anthordistance_count[1],Anthordistance[2]/Anthordistance_count[2]);
   // Anthordistance[0] =filter((int)(Anthordistance[0]/Anthordistance_count[0]),0);
   // Anthordistance[1] =filter((int)(Anthordistance[1]/Anthordistance_count[1]),1);
   // Anthordistance[2] = filter((int)(Anthordistance[2]/Anthordistance_count[2]),2);
    //printf("Count:%d %d %d \r\n",Anthordistance_count[0],Anthordistance_count[1],Anthordistance_count[2]);
    //printf("Count:%d %d %d \r\n",Anthordistance[0],Anthordistance[1],Anthordistance[2]);
    //printf(" \r\n");

    {
        int Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            }
			Anchor_Index++;
        }
    }

    compute_angle_send_to_anthor0(Anthordistance[0], Anthordistance[1],Anthordistance[2]);

		
		for(int j=0;j<ANCHOR_MAX_NUM;j++)
		{
			if(Anthordistance_count[j]>0)
			{
				 sprintf(dist_str, "an%d:%3.2fm",j,(float)Anthordistance[j]/1000);
					printf("%s\r\n",dist_str);
			}
		
		}
    if(Anthordistance_count[0]>0)
    {
        sprintf(dist_str, "an0:%3.2fm", (float)Anthordistance[0]/1000);
				//printf("%s\r\n",dist_str);
        OLED_ShowString(0, 2," 		   ");
        OLED_ShowString(0, 2,dist_str);
    }


    if(Anthordistance_count[1]>0)
    {
        sprintf(dist_str, "an1:%3.2fm", (float)Anthordistance[1]/1000);
				//printf("%s\r\n",dist_str);
        OLED_ShowString(0, 4,"		 ");
        OLED_ShowString(0, 4,dist_str);
    }


    if(Anthordistance_count[2]>0)
    {
        sprintf(dist_str, "%3.2fm", (float)Anthordistance[2]/1000);
				//AnthordistanceBuff[0]=Anthordistance[0];
				//printf("an2:%s\r\n",dist_str);
			/*
			float get=(float)Anthordistance[2]/1000;
				if(get >0.98 && get<1.20)
				{
					OLED_ShowString(0, 6,"success");
				
				}		
				else
				{
				
					OLED_ShowString(0, 6,"fail");
				
				}*/
					
        OLED_ShowString(0, 6,"		 ");
        OLED_ShowString(0, 6,dist_str);
    }
    // printf("Distance:%d,   %d,    %d mm\r\n",(int)((float)Anthordistance[0]/Anthordistance_count[0]),(int)((float)Anthordistance[1]/Anthordistance_count[1]),(int)((float)Anthordistance[2]/Anthordistance_count[2]));
}





#define DISTANCE3 0.27

//**************************************************************//
//distance1 anthor0 <--> TAG  mm
//distance2 anthor1 <--> TAG  mm
//distance3 anthor2 <--> TAG  mm
//**************************************************************//
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3)
{
    static int framenum = 0 ;

#if 0 //compute angle for smartcar
    float dis3_constans = DISTANCE3;
    float cos = 0;
    float angle = 0 ;
    float dis1 = (float)distance1/1000; //m
    float dis2 = (float)distance2/1000;  //m

    if(dis1 + dis3_constans < dis2 || dis2+dis3_constans < dis1)
    {
        //printf("ERROR!\r\n");
        //return;
    }
    cos = (dis1*dis1 + dis3_constans* dis3_constans - dis2*dis2)/(2*dis1*dis3_constans);
    angle  = acos(cos)*180/3.1415926;
    printf("cos = %f, arccos = %f\r\n",cos,angle);
    sprintf(dist_str, "angle: %3.2f m", angle);
    OLED_ShowString(0, 6,"            ");
    OLED_ShowString(0, 6,dist_str);

    if(dis1 > 1)
    {
        if(angle > 110)
        {
            printf("turn right\r\n");
            angle_msg[10] = 'R';
        }
        else if(angle < 75)
        {
            printf("turn left\r\n");
            angle_msg[10] = 'L';
        }
        else
        {
            printf("forward\r\n");
            angle_msg[10] = 'F';
        }
    }
    else
    {
        printf("stay here\r\n");
        angle_msg[10] = 'S';
    }
    angle_msg[LOCATION_FLAG_IDX] = 0;

#else
    //location
    {
        uint8 len = 0;
        angle_msg[LOCATION_FLAG_IDX] = 1;

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'm';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'r';

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = TAG_ID;//TAG ID

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\n';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\r';


        angle_msg[LOCATION_INFO_LEN_IDX] = len;
        //MAX LEN
        if(LOCATION_INFO_START_IDX + len -2 >ANGLE_MSG_MAX_LEN)
        {
            while(1);//toggle LED
        }
        //USART_puts((char*)dist_str,16);

    }
#endif
    //only anthor0 recive angle message
    angle_msg[ALL_MSG_SN_IDX] = framenum;
    angle_msg[ALL_MSG_TAG_IDX] = TAG_ID;

    dwt_writetxdata(sizeof(angle_msg), angle_msg, 0);
    dwt_writetxfctrl(sizeof(angle_msg), 0);

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE );
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };

    framenum++;

}

static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* 清SR寄存器中的TC标志 */

    USART_ClearFlag(EVAL_COM1,USART_FLAG_TC);
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM1, (uint8_t) ch);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}
