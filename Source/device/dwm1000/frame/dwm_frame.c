#include "dwm_frame.h"

#include "deca_lib/deca_regs.h"
#include "deca_lib/deca_types.h"
#include "stm_lib/stm32f10x_conf.h"

#include <FreeRTOS.h>
#include <stdio.h>
#include <string.h>

/** MAC header Frame type field values */
// Beacon
#define DWM_MAC_FRAME_TYPE_BEACON   0x00 // 0b000
// Data
#define DWM_MAC_FRAME_TYPE_DATA     0x01 // 0b001
// Acknowledgement
#define DWM_MAC_FRAME_TYPE_ACK      0x02 // 0b010
// MAC command
#define DWM_MAC_FRAME_TYPE_MACCMD   0x03 // 0b011

/** MAC header Destination/Source addressing mode field values */
// No address or PAN ID is present in the frame
#define DWM_MAC_ADDR_MODE_NONE      0x00 // 0b00
// Reserved
//#define DWM_MAC_ADDR_RESERVED     0x01 // 0b01
// The address field is a short (16-bit) address
#define DWM_MAC_ADDR_MODE_SHORT     0x02 // 0b10
// The address field is an extended (64-bit) address
#define DWM_MAC_ADDR_MODE_EXT       0x03 // 0b11

// MAC Header frame control field type (two octets)
typedef uint16_t TDwmMhrFcf;

/**
 * @brief MAC Header frame control field structure
 * This must be packed to TDwmMhrFcf (@see MakeFrameControlField)
 */
typedef struct
{
    uint16_t frameType;
    uint8_t securityEnabled;
    uint8_t framePending;
    uint8_t ackRequest;
    uint8_t panIdCompress;
    uint8_t destAddrMode;
    uint8_t frameVersion;
    uint8_t srcAddrMode;
} SDwmMhrFcf;

/**
 * @brief MAC Header (MHR) variables sizes structure
 * This structure is used to calculate a header full size
 * for example depending on a frame control field values
 */
typedef struct
{
    uint8_t frameCtl;
    uint8_t seqNumber;
    uint8_t destPanId;
    uint8_t destAddr;
    uint8_t srcPanId;
    uint8_t srcAddr;
    uint8_t auxSecHeader;
} SDwmMhrSkel;

static SDwmMhrSkel DwmMakeMhrSkel(const SDwmMhrFcf* fcf) 
{
    assert_param(fcf != NULL);
    SDwmMhrSkel res;

    /** Frame Control */
    res.frameCtl = sizeof(TDwmMhrFcf);

    /** Sequence Number */
    res.seqNumber = sizeof(TSeqNumber);

    /** Destination PAN Identifier */
    if (fcf->destAddrMode == DWM_MAC_ADDR_MODE_NONE)
    {
        res.destPanId = 0;
    }
    else
    {
        // Set when destination address is set
        res.destPanId = 2;
    }

    /** Destination Address */
    switch (fcf->destAddrMode)
    {
    case DWM_MAC_ADDR_MODE_SHORT:
        res.destAddr = 2;
        break;
    case DWM_MAC_ADDR_MODE_EXT:
        res.destAddr = 8;
        break;
    case DWM_MAC_ADDR_MODE_NONE:
    default:
        res.destAddr = 0;
        break;
    }

    /** Source PAN Identifier */
    if (fcf->srcAddrMode == DWM_MAC_ADDR_MODE_NONE)
    {
        res.srcPanId = 0;
    }
    else if (fcf->panIdCompress > 0 && fcf->destAddrMode != DWM_MAC_ADDR_MODE_NONE)
    {
        // Not set when both addresses are set and PAN ID compression is enabled
        res.srcPanId = 0;
    }
    else
    {
        // Set when source address is set
        res.srcPanId = 2;
    }

    /** Source Address */
    switch (fcf->srcAddrMode)
    {
    case DWM_MAC_ADDR_MODE_SHORT:
        res.srcAddr = 2;
        break;
    case DWM_MAC_ADDR_MODE_EXT:
        res.srcAddr = 8;
        break;
    case DWM_MAC_ADDR_MODE_NONE:
    default:
        res.srcAddr = 0;
        break;
    }

    /** Auxiliary Security Header */
    // Security is not supported
    res.auxSecHeader = 0;

    return res;
}

static TDwmMhrFcf DwmPackFrameControlField(const SDwmMhrFcf* fcf)
{
    assert_param(fcf != NULL);

    TDwmMhrFcf res = 0;

    res |= fcf->frameType         << 0;
    res |= fcf->securityEnabled   << 3;
    res |= fcf->framePending      << 4;
    res |= fcf->ackRequest        << 5;
    res |= fcf->panIdCompress     << 6;
    res |= fcf->destAddrMode      << 10;
    res |= fcf->frameVersion      << 12;
    res |= fcf->srcAddrMode       << 14;

    return res;
}

static SDwmMhrFcf DwmUnpackFrameControlField(TDwmMhrFcf fcf)
{
    SDwmMhrFcf res = {
        .frameType = fcf & 0x0007,
        .securityEnabled = (fcf >> 3) & 0x01,
        .framePending = (fcf >> 4) & 0x01,
        .ackRequest = (fcf >> 5) & 0x01,
        .panIdCompress = (fcf >> 6) & 0x01,
        .destAddrMode = (fcf >> 10) & 0x03,
        .frameVersion = (fcf >> 12) & 0x03,
        .srcAddrMode = (fcf >> 14) & 0x03,
    };
    return res;
}

static void DwmPackFrame(const SDwmFrame* frame, void** ppvOutPacket, uint32_t* pOutPacketSize)
{
    assert_param(frame != NULL);
    assert_param(ppvOutPacket != NULL);
    assert_param(pOutPacketSize != NULL);

    SDwmMhrFcf fcf = {
        .frameType = DWM_MAC_FRAME_TYPE_DATA,
        .framePending = 0,
        .ackRequest = 0,
        .panIdCompress = 1,
        .destAddrMode = DWM_MAC_ADDR_MODE_SHORT,
        .frameVersion = 0,
        .srcAddrMode = DWM_MAC_ADDR_MODE_SHORT,
    };

    const SDwmMhrSkel mhrSkel = DwmMakeMhrSkel(&fcf);

    uint8_t mhrSize = 0;
    mhrSize += mhrSkel.frameCtl;
    mhrSize += mhrSkel.seqNumber;
    mhrSize += mhrSkel.destPanId;
    mhrSize += mhrSkel.destAddr;
    mhrSize += mhrSkel.srcPanId;
    mhrSize += mhrSkel.srcAddr;
    mhrSize += mhrSkel.auxSecHeader;

    // +2 for MAC footer (CRC value)
    uint32_t packetSize = mhrSize + frame->size + 2;
    uint8_t* packet = pvPortMalloc(packetSize);

    uint8_t* packetPtr = packet;

    /** WRITE MAC HEADER */

    // Write frame control
    *((TDwmMhrFcf*)packetPtr) = DwmPackFrameControlField(&fcf);
    packetPtr += mhrSkel.frameCtl;

    // Write sequence number
    *packetPtr = frame->hdr.sequenceNumber;
    packetPtr += mhrSkel.seqNumber;

    // Write destination PAN ID
    if (mhrSkel.destPanId > 0)
    {
        // PAN IDs are not implemented so use zeros
        memset(packetPtr, 0, mhrSkel.destPanId);
        packetPtr += mhrSkel.destPanId;
    }

    // Write destination address
    if (mhrSkel.destAddr > 0)
    {
        // Extended addresses are not supported
        assert_param(mhrSkel.destAddr == sizeof(frame->hdr.destinationAddr));
        memcpy(packetPtr, &frame->hdr.destinationAddr, mhrSkel.destAddr);
        packetPtr += mhrSkel.destAddr;
    }

    // Write source PAN ID
    if (mhrSkel.srcPanId > 0)
    {
        // PAN IDs are not implemented so use zeros
        memset(packetPtr, 0, mhrSkel.srcPanId);
        packetPtr += mhrSkel.srcPanId;
    }

    // Write source address
    if (mhrSkel.srcAddr > 0)
    {
        // Extended addresses are not supported
        assert_param(mhrSkel.srcAddr == sizeof(frame->hdr.sourceAddr));
        memcpy(packetPtr, &frame->hdr.sourceAddr, mhrSkel.srcAddr);
        packetPtr += mhrSkel.srcAddr;
    }

    // Write Aux Security header
    if (mhrSkel.auxSecHeader > 0)
    {
        // Security is not implemented so use zeros
        memset(packetPtr, 0, mhrSkel.auxSecHeader);
        packetPtr += mhrSkel.auxSecHeader;
    }

    /** WRITE PAYLOAD */
    memcpy(packetPtr, frame->data, frame->size);
    packetPtr += frame->size;

    // Don't write footer, because it will be filled automatically with 16-bit CRC on transmission

    // Commit packet
    *ppvOutPacket = packet;
    *pOutPacketSize = packetSize;
}

static void DwmUnpackFrame(const void* pvPacket, uint32_t packetSize, SDwmFrame* outFrame)
{
    assert_param(pvPacket != NULL);
    assert_param(outFrame != NULL);

    assert_param(packetSize > sizeof(TDwmMhrFcf));

    // Parse frame control field
    SDwmMhrFcf fcf = DwmUnpackFrameControlField(*(TDwmMhrFcf*)pvPacket);
    const SDwmMhrSkel mhrSkel = DwmMakeMhrSkel(&fcf);

    uint8_t mhrSize = 0;
    mhrSize += mhrSkel.frameCtl;
    mhrSize += mhrSkel.seqNumber;
    mhrSize += mhrSkel.destPanId;
    mhrSize += mhrSkel.destAddr;
    mhrSize += mhrSkel.srcPanId;
    mhrSize += mhrSkel.srcAddr;
    mhrSize += mhrSkel.auxSecHeader;

    // +2 for MAC footer (CRC value)
    // Frames without payload are forbidden
    const uint32_t macSize = mhrSize + 2;
    assert_param(packetSize > macSize);

    SDwmFrame tempFrame;

    // Addresses must present in the packet
    // Extended addresses are not supported
    assert_param(mhrSkel.destAddr == sizeof(tempFrame.hdr.destinationAddr));
    assert_param(mhrSkel.srcAddr == sizeof(tempFrame.hdr.sourceAddr));

    // Must be >0
    const uint32_t payloadSize = packetSize - macSize;

    tempFrame.data = pvPortMalloc(payloadSize);
    tempFrame.size = payloadSize;

    uint8_t* packetPtr = (uint8_t*)pvPacket;

    packetPtr += mhrSkel.frameCtl;
    tempFrame.hdr.sequenceNumber = *(TSeqNumber*)packetPtr;

    packetPtr += mhrSkel.seqNumber;
    packetPtr += mhrSkel.destPanId;
    tempFrame.hdr.destinationAddr = *(TDeviceAddr*)packetPtr;

    packetPtr += mhrSkel.destAddr;
    packetPtr += mhrSkel.srcPanId;
    tempFrame.hdr.sourceAddr = *(TDeviceAddr*)packetPtr;

    packetPtr += mhrSkel.srcAddr;
    packetPtr += mhrSkel.auxSecHeader;
    memcpy(tempFrame.data, packetPtr, payloadSize);

    *outFrame = tempFrame;
}

static int DwmFrameWriteInternal(const SDwmFrame* frame, uint32_t txDelay, uint8_t bRespExpected)
{
    assert_param(frame != NULL);

    uint8_t* packet = NULL;
    uint32_t packetSize = 0;
    DwmPackFrame(frame, (void**)&packet, &packetSize);

    assert_param(packet != NULL);
    assert_param(packetSize != 0);

    int result = DWT_ERROR;

    do
    {
        if (dwt_writetxdata(packetSize, packet, 0) != DWT_SUCCESS)
        {
            break;
        }

        if (dwt_writetxfctrl(packetSize, 0) != DWT_SUCCESS)
        {
            break;
        }

        uint8_t txMode = 0;
        if (txDelay > 0)
        {
            dwt_setdelayedtrxtime(txDelay);
            txMode |= DWT_START_TX_DELAYED;
        }
        else
        {
            txMode |= DWT_START_TX_IMMEDIATE;
        }

        if (bRespExpected)
        {
            txMode |= DWT_RESPONSE_EXPECTED;
        }

        if (dwt_starttx(txMode) != DWT_SUCCESS)
        {
            break;
        }

        result = DWT_SUCCESS;
        
    } while (0);

    vPortFree(packet);

    if (result == DWT_SUCCESS)
    {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        /* Clear good TX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    }

    return result;
}

int DwmFrameReadInternal(SDwmFrame* outFrame)
{
    int result = DWT_ERROR;

    /**
     * Poll until a packet is properly received or an error/timeout occurs.
     * 
     * We use polled mode of operation here to keep as simple as possible but RXFCG and error/timeout
     * status events can be used to generate interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
     * 
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register,
     * we can use this simplest API function to access it. */
    uint32_t statusReg;
    do
    {
        statusReg = dwt_read32bitreg(SYS_STATUS_ID);
    } while ( !((statusReg) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) );

    if (statusReg & SYS_STATUS_RXFCG)
    {
        /* A packet has been received, copy it to local buffer. */
        size_t packetSize = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        uint8_t* packetBuf = pvPortMalloc(packetSize);

        dwt_readrxdata(packetBuf, packetSize, 0);

        DwmUnpackFrame(packetBuf, packetSize, outFrame);
        result = DWT_SUCCESS;

        vPortFree(packetBuf);

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialize LDE operation. */
        dwt_rxreset();
    }

    return result;
}

int DwmFrameRead(SDwmFrame* outFrame, uint32_t rxTimeout)
{
    assert_param(outFrame != NULL);

    dwt_setrxtimeout(rxTimeout);

    /**
     * Activate reception immediately.
     * Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
     * optimize system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.). */
    if (dwt_rxenable(0) != DWT_SUCCESS)
    {
        return DWT_ERROR;
    }

    return DwmFrameReadInternal(outFrame);
}

int DwmFrameWrite(const SDwmFrame* frame, uint32_t txDelay)
{
    assert_param(frame != NULL);
    return DwmFrameWriteInternal(frame, txDelay, 0);
}

int DwmFrameReadAfterWrite(const SDwmFrame* frame, SDwmFrame* outFrame, uint32_t txDelay, uint32_t rxDelay, uint32_t rxTimeout)
{
    assert_param(frame != NULL);
    assert_param(outFrame != NULL);

    /**
     * Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_setrxaftertxdelay(rxDelay);
    dwt_setrxtimeout(rxTimeout);

    if (DwmFrameWriteInternal(frame, txDelay, 1) != DWT_SUCCESS)
    {
        return DWT_ERROR;
    }

    return DwmFrameReadInternal(outFrame);
}
