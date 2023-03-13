#include "comm/comm.h"

#include "config/config.h"
#include "kalman/kalman.h"
#include "math/vector.h"
#include "shell/shell.h"

#include "at24c16/AT24C02.h"
#include "dwm1000/dwm1000.h"
#include "memory/flash.h"

#include "deca_lib/deca_device_api.h"
#include "deca_lib/deca_port.h"
#include "deca_lib/deca_regs.h"

#include "ros_lib/dwm1000_msgs/BeaconDataArray.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <stm32f10x.h>
#include <stm32_eval.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

uint8 SWITCH_DIS = 1;

extern uint8_t TAG_ID;
extern uint8_t MASTER_TAG;
extern uint8_t SLAVE_TAG_START_INDEX;
extern uint8_t ANCHOR_IND; 
extern uint8_t ANCHOR_IND; 
extern uint8 Semaphore[MAX_SLAVE_TAG];

vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];

int Anthordistance_count[ANCHOR_MAX_NUM];

int ANCHOR_REFRESH_COUNT_set=5;
#define ANCHOR_REFRESH_COUNT ANCHOR_REFRESH_COUNT_set

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

extern "C" int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}

extern "C" void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    frame_seq_nb=0;

    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    
        /* Write frame data to DW1000 and prepare transmission. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);
        uint32 tick1=portGetTickCount();
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        {
            if((portGetTickCount() - tick1) > 350)
            {
                break;
            }

        };

        GPIO_SetBits(GPIOA, GPIO_Pin_1);

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

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
            {
                continue;
            }

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
                        break;
                    }
                }

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
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // vTaskDelay(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}

void MainTask(void* pvParameters)
{
    printf("Initializing DWM1000...\r\n");

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */

    reset_DW1000();

    /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();

    if (dwt_initialise(DWT_LOADUCODE) == -1)
    {
        printf("DWM1000 init is failed!\r\n");

        // Blink LED when init is failed
        while (1)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            vTaskDelay(1000);

            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            vTaskDelay(1000);
        }
    }

    spi_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);
    dwt_setleds(1);

    /* Apply default antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    printf("Init pass!\r\n");

    SRuntimeConfig cfg = ConfigRead();

#ifdef BEACON

    printf("Device type: BEACON (anchor)\r\n");
    printf("Device id: %d\r\n", cfg.deviceId);

    ANCHOR_IND = cfg.deviceId;

    Anchor_Array_Init();

    /* Loop forever initiating ranging exchanges. */
    //KalMan_PramInit();
    ANTHOR_MEASURE();

#else

    printf("Device type: SENSOR (tag)\r\n");
    printf("Device id: %d\r\n", cfg.deviceId);

    TAG_ID = cfg.deviceId;
    MASTER_TAG = TAG_ID;

    /* Set expected response's delay and timeout.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    
    if(TAG_ID == MASTER_TAG)
    {
        Semaphore_Init();
    }

    //Master TAG0
    TAG_MEASURE();

#endif

    vTaskDelete(NULL);	
}

int main(void)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    ShellInit();

    xTaskCreate(
        MainTask,
        "MainTask",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

#ifndef BEACON

    xTaskCreate(
        CommTask,
        "CommTask",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

#endif

    vTaskStartScheduler();
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
    dwm1000_msgs::BeaconDataArray::_beacons_type beaconDataArray[ANCHOR_MAX_NUM];
    dwm1000_msgs::BeaconDataArray beaconDataPacket;
    beaconDataPacket.beacons = beaconDataArray;
    beaconDataPacket.beacons_length = 0;

    for (int Anchor_Index = 0; Anchor_Index < ANCHOR_MAX_NUM; Anchor_Index++)
    {
        if(Anthordistance_count[Anchor_Index] > 0 )
        {
            int distance = filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            Anthordistance[Anchor_Index] = distance;

            float distMeters = (float)distance / 1000;

            dwm1000_msgs::BeaconDataArray::_beacons_type& beaconData = beaconDataArray[beaconDataPacket.beacons_length++];
            beaconData.id = Anchor_Index;
            beaconData.dist = distMeters;

            // printf("Got distance (id = %d): %3.2f meters\r\n", Anchor_Index, distMeters);
        }
    }

    CommSendBeaconDataArray(&beaconDataPacket);
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
