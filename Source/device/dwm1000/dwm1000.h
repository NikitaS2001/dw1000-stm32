#pragma once

#include "deca_lib/deca_device_api.h"
#include <stdint.h>

#ifndef DWM_VAR_TRX_DLY
#define DWM_VAR_TRX_DLY 0
#endif

#if (DWM_VAR_TRX_DLY == 0)

/**
 * The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 * but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 * range measurements.
 * 
 * Default antenna delay values for 64 MHz PRF:
 */
#define TX_ANT_DLY 16466
#define RX_ANT_DLY 16466

#endif // DWM_VAR_TRX_DLY

/**
 * UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 us and 1 us = 499.2 * 128 dtu.
 * */
#define UUS_TO_DWT_TIME 65536

/**
 * Delay between frames, in UWB microseconds.
 * 
 * This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature.
 * 
 * Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 * and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 * details about the timings involved in the ranging process.
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150

/**
 * This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration.
 */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 11200

/**
 * This is the delay from the end of the frame transmission to the enable of the receiver,
 * as programmed for the DW1000's wait for response feature.
 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/**
 * Receive response timeout, in UWB microseconds.
 *
 * This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 * is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 * 110k data rate used (around 30 ms).
 */
#define RESP_RX_TIMEOUT_UUS 27000

/**
 * Receive final frame timeout, in UWB microseconds.
 * 
 * This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 * is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 * 110k data rate used (around 35 ms).
 */
#define FINAL_RX_TIMEOUT_UUS 33000

/**
 * Receive distance frame timeout, in UWB microseconds.
 */
#define DIST_RX_TIMEOUT_UUS 33000

/**
 * Preamble timeout, in multiple of PAC size.
 *
 * The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 * out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 * recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 * length) for more challenging longer range, NLOS or noisy environments.
 */
#define PRE_TIMEOUT 8

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#ifdef __cplusplus
extern "C"
{
#endif

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

/**
 * Low power:  reduce reflections due to large metal structures
 * 0x0E082848,  //16M prf power
 * 0x25456585   //64M prf power
 *
 * High power:
 * 0x0E08281F,  //16M prf power
 * 0x2545651F   //64M prf power
 */
static dwt_txconfig_t configTx = {
    0xC2,           //Delay
    0x2545651F      //Power
};

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void);

#ifdef __cplusplus
}
#endif
