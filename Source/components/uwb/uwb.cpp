#include "uwb.h"

#include "config/config.h"
#include "telem/telem.h"
#include "shell/shell.h"

#include "dwm1000/dwm1000.h"
#include "dwm1000/frame/dwm_frame.h"

#include "deca_lib/deca_port.h"
#include "deca_lib/deca_regs.h"

#include <FreeRTOS.h>
#include <task.h>

#include <string.h>

enum EUwbMsgType
{
    MSG_DWT_POLL,
    MSG_DWT_RESP,
    MSG_DWT_FINAL,
    MSG_DWT_DIST,
};

struct SUwbPayload
{
    EUwbMsgType type;
    uint8_t payload[16];
};

struct SUwbPacket
{
    SDwmFrameHdr frameHdr;
    SUwbPayload payload;
};

#define DECLARE_MSG(type, name, params)                     \
struct name                                                 \
{                                                           \
    name() { memset(this, 0, sizeof(name)); }               \
    params                                                  \
    static const EUwbMsgType s_type = type;                 \
};

#define DECLARE_MSG_NO_PARAMS(type, name) DECLARE_MSG(type, name, )

DECLARE_MSG_NO_PARAMS(MSG_DWT_POLL, SUwbPollMsg);
DECLARE_MSG_NO_PARAMS(MSG_DWT_RESP, SUwbRespMsg);
DECLARE_MSG(MSG_DWT_FINAL, SUwbFinalMsg,
    uint32_t pollTxTs;
    uint32_t respRxTs;
    uint32_t finalTxTs;
);
DECLARE_MSG(MSG_DWT_DIST, SUwbDistMsg,
    double distance;
);

static uint32_t GetAntennaTxDelay()
{
#if DWM_VAR_TRX_DLY
    SRuntimeConfig cfg = ConfigRead();
    return cfg.aggAntDelay / 2;
#else
    return TX_ANT_DLY;
#endif // DWM_VAR_TRX_DLY
}

static uint32_t GetAntennaRxDelay()
{
#if DWM_VAR_TRX_DLY
    SRuntimeConfig cfg = ConfigRead();
    return cfg.aggAntDelay / 2;
#else
    return RX_ANT_DLY;
#endif // DWM_VAR_TRX_DLY
}

static void UpdateAntennaDelays()
{
    dwt_setrxantennadelay(GetAntennaTxDelay());
    dwt_settxantennadelay(GetAntennaRxDelay());
}

static const char* UwbPacketTypeToString(EUwbMsgType type)
{
    switch (type)
    {
    case MSG_DWT_POLL:      return "MSG_DWT_POLL";
    case MSG_DWT_RESP:      return "MSG_DWT_RESP";
    case MSG_DWT_FINAL:     return "MSG_DWT_FINAL";
    case MSG_DWT_DIST:      return "MSG_DWT_DIST";
    default:                return "UNKNOWN";
    }
}

template<typename T>
static T UwbUnpackPayload(const SUwbPayload& pld)
{
    assert_param(pld.type == T::s_type);

    T res;
    if (sizeof(T) > 0)
    {
        assert_param(sizeof(T) <= sizeof(res.payload));
        memcpy(&res, pld.payload, sizeof(T));
    }

    return res;
}

template<typename T>
static SUwbPayload UwbPackPayload(const T& msg)
{
    SUwbPayload res;
    res.type = T::s_type,

    memset(res.payload, 0, sizeof(res.payload));

    if (sizeof(T) > 0)
    {
        assert_param(sizeof(T) <= sizeof(res.payload));
        memcpy(res.payload, &msg, sizeof(T));
    }

    return res;
}

template<typename T>
static SUwbPayload UwbPackPayload()
{
    return UwbPackPayload(T());
}

static bool UwbRead(SUwbPacket& outPacket, uint32_t rxTimeout = 0)
{
    SDwmFrame frame;
    if (DwmFrameRead(&frame, rxTimeout) != DWT_SUCCESS)
    {
        return false;
    }

    outPacket.frameHdr = frame.hdr;
    memcpy(&outPacket.payload, frame.data, sizeof(outPacket.payload));

    vPortFree(frame.data);

    return true;
}

static bool UwbWrite(const SUwbPacket& packet, uint32_t txDelay = 0)
{
    SDwmFrame frame;
    frame.hdr = packet.frameHdr;
    frame.data = (void*)&packet.payload;
    frame.size = sizeof(packet.payload);

    return DwmFrameWrite(&frame, txDelay) == DWT_SUCCESS;
}

static bool UwbReadAfterWrite(const SUwbPacket& packet, SUwbPacket& outPacket, uint32_t txDelay = 0, uint32_t rxDelay = 0, uint32_t rxTimeout = 0)
{
    SDwmFrame frame;
    frame.hdr = packet.frameHdr;
    frame.data = (void*)&packet.payload;
    frame.size = sizeof(packet.payload);

    SDwmFrame outFrame;
    if (DwmFrameReadAfterWrite(&frame, &outFrame, txDelay, rxDelay, rxTimeout) != DWT_SUCCESS)
    {
        return false;
    }

    outPacket.frameHdr = outFrame.hdr;
    memcpy(&outPacket.payload, outFrame.data, sizeof(outPacket.payload));

    vPortFree(outFrame.data);

    return true;
}

static void AnchorLoop()
{
    SHELL_LOG("[UWB] Device type: ANCHOR\r\n");

    while (true)
    {
        /* Set configuration which is not preserved in deep sleep */
        UpdateAntennaDelays();

        SUwbPacket pollPacket;

        if (!UwbRead(pollPacket))
        {
            continue;
        }

        if (pollPacket.payload.type != MSG_DWT_POLL)
        {
            SHELL_LOG("[UWB] Could not read poll packet\r\n");
            continue;
        }

        uint64_t pollRxTs = get_rx_timestamp_u64();

        SUwbPacket respPacket;
        respPacket.frameHdr.sourceAddr = ReadDeviceAddress();
        respPacket.frameHdr.destinationAddr = pollPacket.frameHdr.sourceAddr;
        respPacket.frameHdr.sequenceNumber = pollPacket.frameHdr.sequenceNumber;
        respPacket.payload = UwbPackPayload<SUwbRespMsg>();

        SUwbPacket finalPacket;
        bool bFinalPacketResult = UwbReadAfterWrite(
            respPacket,
            finalPacket,
            0,
            RESP_TX_TO_FINAL_RX_DLY_UUS,
            FINAL_RX_TIMEOUT_UUS
            );

        if (!bFinalPacketResult || finalPacket.payload.type != MSG_DWT_FINAL)
        {
            SHELL_LOG("[UWB] Could not read final packet\r\n");
            continue;
        }

        uint64_t respTxTs = get_tx_timestamp_u64();
        uint64_t finalRxTs = get_rx_timestamp_u64();

        SUwbFinalMsg finalMsg = UwbUnpackPayload<SUwbFinalMsg>(finalPacket.payload);

        uint32_t pollRxTs32 = (uint32_t)pollRxTs;
        uint32_t respTxTs32 = (uint32_t)respTxTs;
        uint32_t finalRxTs32 = (uint32_t)finalRxTs;

        double Ra = (double)(finalMsg.respRxTs - finalMsg.pollTxTs);
        double Rb = (double)(finalRxTs32 - respTxTs32);
        double Da = (double)(finalMsg.finalTxTs - finalMsg.respRxTs);
        double Db = (double)(respTxTs32 - pollRxTs32);
        int64_t tofDtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

        double tof = tofDtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        SUwbDistMsg distMsg;
        distMsg.distance = distance;

        SUwbPacket distPacket;
        distPacket.frameHdr.sourceAddr = ReadDeviceAddress();
        distPacket.frameHdr.destinationAddr = finalPacket.frameHdr.sourceAddr;
        distPacket.frameHdr.sequenceNumber = finalPacket.frameHdr.sequenceNumber;
        distPacket.payload = UwbPackPayload(distMsg);

        if (!UwbWrite(distPacket))
        {
            SHELL_LOG("[UWB] Could not send distance packet\r\n");
        }
    }
}

// TODO: move to other file
static void PollAnchor(TDeviceAddr addr)
{
    static TSeqNumber s_seqNum = 0;

    SUwbPacket pollPacket;
    pollPacket.frameHdr.sourceAddr = ReadDeviceAddress();
    pollPacket.frameHdr.destinationAddr = addr;
    pollPacket.frameHdr.sequenceNumber = s_seqNum++;
    pollPacket.payload = UwbPackPayload<SUwbPollMsg>();

    SUwbPacket respPacket;
    bool bRespResult = UwbReadAfterWrite(
        pollPacket,
        respPacket,
        0,
        POLL_TX_TO_RESP_RX_DLY_UUS,
        RESP_RX_TIMEOUT_UUS
        );

    if (!bRespResult || respPacket.payload.type != MSG_DWT_RESP)
    {
        return;
    }

    /* Retrieve poll transmission and response reception timestamp. */
    uint64_t pollTxTs = get_tx_timestamp_u64();
    uint64_t respRxTs = get_rx_timestamp_u64();

    /**
     * Compute final message transmission time.
     * 
     * As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
     * register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
     * response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
     * lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits. */
    uint32_t finalTxTime = (respRxTs + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
    uint32_t finalTxTs = (((uint64_t)(finalTxTime & 0xFFFFFFFEUL)) << 8) + GetAntennaTxDelay();

    SUwbFinalMsg finalMsg;
    finalMsg.pollTxTs = pollTxTs;
    finalMsg.respRxTs = respRxTs;
    finalMsg.finalTxTs = finalTxTs;

    SUwbPacket finalPacket;
    finalPacket.frameHdr.sourceAddr = ReadDeviceAddress();
    finalPacket.frameHdr.destinationAddr = addr;
    finalPacket.frameHdr.sequenceNumber = s_seqNum++;
    finalPacket.payload = UwbPackPayload(finalMsg);

    SUwbPacket distPacket;
    bool bDistResult = UwbReadAfterWrite(
        finalPacket,
        distPacket,
        finalTxTime,
        0,
        DIST_RX_TIMEOUT_UUS
        );

    if (!bDistResult || distPacket.payload.type != MSG_DWT_DIST)
    {
        return;
    }

    SUwbDistMsg distMsg = UwbUnpackPayload<SUwbDistMsg>(distPacket.payload);

    // SHELL_LOG("[UWB] GOT DIST (0x%04X) (%3.3f)\r\n", addr, (float)distMsg.distance);

    dwm1000::BeaconData beaconData;
    beaconData.id = addr;
    beaconData.dist = distMsg.distance;

    TelemSendBeaconData(beaconData);
}

static void TagLoop()
{
    SHELL_LOG("[UWB] Device type: TAG\r\n");

    while (true)
    {
        /* Set configuration which is not preserved in deep sleep */
        UpdateAntennaDelays();

        for (int32_t i = 1; i <= 10; ++i)
        {
            PollAnchor((TDeviceAddr)i);
        }
    }
}

void UwbTask(void* pvParameters)
{
    SRuntimeConfig cfg = ConfigRead();

    dwt_setpanid(0);
    WriteDeviceAddress(cfg.deviceId);

    uint32_t txDly = GetAntennaTxDelay();
    uint32_t rxDly = GetAntennaRxDelay();

    SHELL_LOG("[UWB] Device id: %d (0x%04X:0x%04X)\r\n", cfg.deviceId, ReadPanId(), ReadDeviceAddress());
    SHELL_LOG("[UWB] Antenna TRX delay is '%d'\r\n", txDly + rxDly);
    SHELL_LOG("\tTX: %d\r\n", txDly);
    SHELL_LOG("\tRX: %d\r\n", rxDly);

    dwt_enableframefilter(DWT_FF_DATA_EN);

#ifdef BEACON
    AnchorLoop();
#else
    TagLoop();
#endif

    vTaskDelete( NULL );
}
