#pragma once

#include "dwm1000/regs/dwm_addr.h"

#include "deca_lib/deca_device_api.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint8_t TSeqNumber;

typedef struct
{
    TDeviceAddr sourceAddr;
    TDeviceAddr destinationAddr;
    TSeqNumber sequenceNumber;
} SDwmFrameHdr;

typedef struct
{
    SDwmFrameHdr hdr;
    void* data;
    uint32_t size;
} SDwmFrame;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DwmFrameRead()
 *
 * @brief This call turns on the receiver immediately
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or it times out (SFD, Preamble or Frame).
 *
 * @param outFrame - frame pointer to read
 * @param rxTimeout - RX timeout value (0 - no timeout)
 *
 * @return DWT_SUCCESS for success, or DWT_ERROR for error
 */
int DwmFrameRead(SDwmFrame* outFrame, uint32_t rxTimeout);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DwmFrameWrite()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below.
 * The passed payload is packet into a MAC frame and then sent using dwt_starttx() with appropriate mode.
 *
 * @param frame - frame to write
 * @param txDelay - a delayed transmission timestamp (zero to send immediately)
 *
 * @return DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int DwmFrameWrite(const SDwmFrame* frame, uint32_t txDelay);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn DwmFrameReadAfterWrite()
 *
 * @brief This call initiates the transmission with a required response, then turns on the receiver immediately.
 * The passed payload is packet into a MAC frame and then sent using dwt_starttx() with appropriate mode.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or it times out (SFD, Preamble or Frame).
 *
 * @param frame - frame to write
 * @param outFrame - frame pointer to read
 * @param txDelay - a delayed transmission timestamp (zero to send immediately)
 * @param rxDelay - a reception delay after transmission is done
 * @param rxTimeout - RX timeout value (0 - no timeout)
 *
 * @return DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int DwmFrameReadAfterWrite(const SDwmFrame* frame, SDwmFrame* outFrame, uint32_t txDelay, uint32_t rxDelay, uint32_t rxTimeout);

#ifdef __cplusplus
}
#endif
