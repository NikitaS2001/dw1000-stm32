#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "at24c16/AT24C02.h"

#include "deca_lib/deca_device_api.h"
#include "deca_lib/deca_port.h"
#include "deca_lib/deca_regs.h"

#include <stm32f10x.h>
#include <stm32_eval.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Default communication configuration. */
static dwt_config_t config =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Example application name and version to display on LCD screen. TAG DELAY */
#define RNG_DELAY_MS 5

/* Default antenna delay values for 64 MHz PRF. */
//#define TX_ANT_DLY 16436
//#define RX_ANT_DLY 16436
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

typedef enum
{
    MSG_POLL = 0x21,
    MSG_RESP = 0x10,
    MSG_FINAL = 0x23,
    MSG_DISTANCE = 0xAA,
    MSG_ANGLE = 0xFE,
    MSG_SEM_RELEASE = 0xE0,
    MSG_TAG_STAT = 0xE1,
    MSG_MASTER_SEM_RELEASE = 0xE2,
    MSG_TAG_STAT_RESP = 0xE3,
    MSG_MASTER_SEM_RELEASE_CONFIRM = 0xE4,
} dwt_frame_type_t;

typedef struct
{
    uint8 magic[] = {0x41, 0x88};
    uint8 seqNumber;
    uint8 tagIndex;
    uint8 manuf[] = {0xDE, 'W', 'A', 'V', 'E'};
    dwt_frame_type_t type;
} dwt_frame_header_t;

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] =                        {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] =                        {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] =                       {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] =                       {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0, 0, 0, 0};
static uint8 tx_poll_msg[] =                        {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] =                        {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] =                       {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 angle_msg[] =                          {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 Semaphore_Release[] =                  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
static uint8 Tag_Statistics[] =                     {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
static uint8 Master_Release_Semaphore[] =           {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
static uint8 Tag_Statistics_response[] =            {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
static uint8 Master_Release_Semaphore_comfirm[] =   {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};


/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_semaphore = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. */
#define FINAL_RX_TIMEOUT_UUS 3300

/* Delay between frames, in UWB microseconds. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2800 //2700 will fail
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Speed of light in air, in metres per second. */
#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299702547
#endif

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
//char dist_str[16] = {0};
/* Declaration of static functions. */
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void compute_angle_send_to_anthor0(int distance1, int distance2, int distance3);
void USART_puts(uint8_t *s,uint8_t len);

#define MAX_SLAVE_TAG 0x1
#define ANCHOR_MAX_NUM 45

// Enable flag
extern uint8 SWITCH_DIS;

// Tag work loop
void TAG_MEASURE(void);

// Anchor work loop
void ANTHOR_MEASURE(void);

#ifdef __cplusplus
}
#endif
