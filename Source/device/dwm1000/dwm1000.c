#include "dwm1000.h"

#define LOWORD(l)           ((WORD)(((DWORD_PTR)(l)) & 0xffff))
#define HIWORD(l)           ((WORD)((((DWORD_PTR)(l)) >> 16) & 0xffff))
#define HIDWORD(dw, hw)     LOWORD(dw) | (hw << 16)
#define LODWORD(dw, lw)     (HIWORD(dw) << 16) | lw

uint64 get_tx_timestamp_u64(void)
{
    __int64 v4; // r4
    int i; // r0
    uint32 v7[6]; // [sp+0h] [bp-18h] BYREF

    v7[0] = 0;
    v7[1] = 0;
    v4 = 0LL;
    dwt_readtxtimestamp(v7);

    for ( i = 4; i >= 0; --i )
    {
        HIDWORD(v4, v4 >> 24);
        LODWORD(v4, *((uint8*)v7 + i) | ((uint32)v4 << 8));
    }

    return v4;
}

uint64 get_rx_timestamp_u64(void)
{
    __int64 v4; // r4
    int i; // r0
    uint32 v7[6]; // [sp+0h] [bp-18h] BYREF

    v7[0] = 0;
    v7[1] = 0;
    v4 = 0LL;
    dwt_readrxtimestamp(v7);

    for ( i = 4; i >= 0; --i )
    {
        HIDWORD(v4, v4 >> 24);
        LODWORD(v4, *((uint8*)v7 + i) | ((uint32)v4 << 8));
    }

    return v4;
}

static void final_msg_get_ts(const uint8* ts_field, uint32* ts)
{
    *ts = 0;
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += (ts_field[i] << (8 * i));
    }
}

static void final_msg_set_ts(uint8* ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

void TAG_MEASURE(void)
{
    int i; // r6
    int v1; // r7
    int v2; // r9
    int v3; // r8
    int v4; // r0
    int v5; // r0
    bool v6; // cc
    int v7; // r0
    int v8; // r0
    int32 v9; // r0
    int v10; // r0
    int32 v11; // r1
    int v12; // r2
    int v13; // r8
    int v14; // r0
    int32 v15; // r1
    unsigned __int16 v16; // r0
    int v17; // r8
    int v18; // r0
    int v19; // r0
    unsigned __int16 v20; // r0

    i = 0;
    v1 = 0;
    v2 = 0;
    while ( 1 )
    {
        while ( !SWITCH_DIS ) {};

        if ( v1 == 1 )
        {
            v3 = 1073809408;
            GPIO_ResetBits(1073809408, 2);
            v4 = GPIO_ResetBits(1073809408, 4);
            Tag_Measure_Dis(v4);
            v1 = 0;
            if ( TAG_ID != MASTER_TAG )
            {
                byte_856 = frame_seq_nb;
                byte_857 = TAG_ID;
                dwt_writetxdata(13, &Semaphore_Release, 0);
                dwt_writetxfctrl(13, 0);
                v5 = dwt_starttx(0);
                tick = portGetTickCnt(v5);
                while ( 1 )
                {
                    v7 = dwt_read32bitoffsetreg(15, 0) << 24;
                    if ( v7 < 0 )
                        break;
                    v6 = (int32)(portGetTickCnt(v7) - tick) > 0xC8;
LABEL_7:
                    if ( v6 )
LABEL_61:
                        NVIC_SystemReset();
                }
                GPIO_SetBits(v3, 4);
            }
        }
        if ( TAG_ID == MASTER_TAG )
        {
            if ( !((int (*)(void))Sum_Tag_Semaphore_request)() )
            {
                for ( i = (uint8)SLAVE_TAG_START_INDEX; !i; i = 1 )
                {
                    v3 = 0;
                    byte_863 = 0;
                    byte_864 = 0;
                    dwt_writetxdata(13, &Tag_Statistics, 0);
                    dwt_writetxfctrl(13, 0);
                    v8 = dwt_starttx(2);
                    tick = portGetTickCnt(v8);
                    while ( 1 )
                    {
                        v10 = dwt_read32bitoffsetreg(15, 0);
                        status_reg = v10;
                        if ( (v10 & 0x2427D000) != 0 )
                            break;
                        v9 = portGetTickCnt(v10) - tick;
                        v6 = v9 > 0xC8;
                        if ( v9 > 0xC8 )
                            goto LABEL_7;
                    }
                    if ( (status_reg & 0x4000) != 0 )
                    {
                        dwt_write32bitoffsetreg(15, 0, 16512);
                        v11 = dwt_read32bitoffsetreg(16, 0) & 0x7F;
                        if ( v11 <= 0x18 )
                            dwt_readrxdata(&rx_buffer, v11, 0);
                        byte_794 = 0;
                        v12 = (uint8)byte_795;
                        if ( !byte_795 )
                        {
                            byte_795 = 0;
                            v13 = v12;
                            if ( !memcmp(&rx_buffer, &Tag_Statistics_response, 0xAu) )
                            {
                                Semaphore[v13] = 1;
                                GPIO_SetBits(1073809408, 4);
                            }
                        }
                    }
                    else
                    {
                        dwt_write32bitoffsetreg(15, 0, 606572544);
                    }
                }
                for ( i = (uint8)SLAVE_TAG_START_INDEX; !i; i = 1 ) {};
            }
            if ( v2 )
                goto LABEL_42;
            v14 = ((int (*)(void))Sum_Tag_Semaphore_request)();
            if ( v14 )
            {
                Semaphore[0] = 0;
                for ( i = (uint8)SLAVE_TAG_START_INDEX; !i; i = 1 )
                {
                    v14 = (uint8)Semaphore[0];
                    if ( Semaphore[0] == 1 )
                    {
                        dwt_setrxaftertxdelay(150);
                        dwt_setrxtimeout(2700);
                        byte_870 = 0;
                        byte_871 = 0;
                        dwt_writetxdata(13, &Master_Release_Semaphore, 0);
                        dwt_writetxfctrl(13, 0);
                        dwt_starttx(2);
                        do
                        status_reg = dwt_read32bitoffsetreg(15, 0);
                        while ( (status_reg & 0x2427D000) == 0 );
                        if ( (status_reg & 0x4000) != 0 )
                        {
                            GPIO_SetBits(1073809408, 2);
                            dwt_write32bitoffsetreg(15, 0, 16512);
                            v15 = dwt_read32bitoffsetreg(16, 0) & 0x7F;
                            if ( v15 <= 0x18 )
                                dwt_readrxdata(&rx_buffer, v15, 0);
                            v14 = (int)&rx_buffer;
                            byte_794 = 0;
                            if ( !byte_795 )
                            {
                                byte_795 = 0;
                                GPIO_SetBits(1073809408, 8);
                                v14 = memcmp(&rx_buffer, &Master_Release_Semaphore_comfirm, 0xAu);
                                if ( !v14 )
                                {
                                    v2 = 1;
LABEL_42:
                                    dwt_setrxtimeout(13500);
                                    dwt_rxenable(0);
                                    do
                                        status_reg = dwt_read32bitoffsetreg(15, 0);
                                    while ( (status_reg & 0x2427D000) == 0 );
                                    if ( (status_reg & 0x4000) != 0 )
                                    {
                                        dwt_write32bitoffsetreg(15, 0, 0x4000);
                                        v16 = (char)dwt_read32bitoffsetreg(16, 0);
                                        dwt_readrxdata(&rx_buffer, v16, 0);
                                        byte_794 = 0;
                                        v17 = (uint8)byte_795;
                                        byte_795 = 0;
                                        v14 = memcmp(&rx_buffer, &Semaphore_Release, 0xAu);
                                        if ( !v14 )
                                        {
                                            v14 = (int)Semaphore;
                                            if ( Semaphore[v17] == 1 )
                                            {
                                                Semaphore[v17] = 0;
                                                v14 = v2 - 1;
                                                v2 = (uint8)(v2 - 1);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        v2 = (uint8)(v2 - 1);
                                        Semaphore[i] = 0;
                                        v14 = dwt_write32bitoffsetreg(15, 0, 606572544);
                                    }
                                    break;
                                }
                            }
                        }
                        else
                        {
                            Semaphore[0] = 0;
                            v14 = dwt_write32bitoffsetreg(15, 0, 606572544);
                        }
                    }
                }
            }
            if ( !Sum_Tag_Semaphore_request(v14) )
            {
                v1 = 1;
                v2 = 0;
            }
        }
        else
        {
            dwt_setrxtimeout(0);
            v18 = dwt_rxenable(0);
            tick = portGetTickCnt(v18);
            while ( 1 )
            {
                v19 = dwt_read32bitoffsetreg(15, 0);
                status_reg = v19;
                if ( (v19 & 0x2427D000) != 0 )
                    break;
                if ( (int32)(portGetTickCnt(v19) - tick) > 0xC8 )
                goto LABEL_61;
            }
            if ( (status_reg & 0x4000) != 0 )
            {
                dwt_write32bitoffsetreg(15, 0, 16512);
                v20 = (char)dwt_read32bitoffsetreg(16, 0);
                dwt_readrxdata(&rx_buffer, v20, 0);
                byte_794 = 0;
                if ( byte_795 == TAG_ID )
                {
                    byte_795 = 0;
                    if ( !memcmp(&rx_buffer, &Tag_Statistics, 0xAu) )
                    {
                        word_87D = *(_WORD *)&frame_seq_nb;
                        dwt_writetxdata(13, &Tag_Statistics_response, 0);
                        dwt_writetxfctrl(13, 0);
                        dwt_starttx(0);
                        GPIO_SetBits(1073809408, 4);
                    }
                    if ( !memcmp(&rx_buffer, &Master_Release_Semaphore, 0xAu) )
                    {
                        word_88A = *(_WORD *)&frame_seq_nb;
                        dwt_writetxdata(13, &Master_Release_Semaphore_comfirm, 0);
                        dwt_writetxfctrl(13, 0);
                        dwt_starttx(0);
                        while ( (dwt_read32bitoffsetreg(15, 0) & 0x80) == 0 )
                        ;
                        v1 = 1;
                        GPIO_SetBits(1073809408, 2);
                    }
                }
            }
            else
            {
                dwt_write32bitoffsetreg(15, 0, 606572544);
            }
        }
    }
}

void ANTHOR_MEASURE(void)
{
    bool v0; // nf
    unsigned __int16 v1; // r0
    int v2; // r5
    int32 v3; // r1
    double v4; // r0
    double v5; // r0
    double v6; // r0
    double v7; // r0
    double v8; // r0
    int v9; // r0
    int32 v10; // [sp+0h] [bp-40h]
    int32 v11; // [sp+4h] [bp-3Ch]
    int32 v12; // [sp+8h] [bp-38h]
    int32 v13; // [sp+Ch] [bp-34h]
    int32 v14; // [sp+10h] [bp-30h]
    int32 v15; // [sp+14h] [bp-2Ch]
    int32 v16; // [sp+18h] [bp-28h]
    int v17; // [sp+1Ch] [bp-24h] BYREF
    int32 v18; // [sp+20h] [bp-20h]
    int v19; // [sp+24h] [bp-1Ch]
    int v20; // [sp+28h] [bp-18h]
    int v21; // [sp+30h] [bp-10h] BYREF
    int v22; // [sp+34h] [bp-Ch] BYREF
    int32 v23; // [sp+38h] [bp-8h]

    while ( 1 )
    {
        dwt_setrxtimeout(0);
        dwt_rxenable(0);
        do
            status_reg = dwt_read32bitoffsetreg(15, 0);
        while ( (status_reg & 0x2427D000) == 0 );
        v0 = (status_reg & 0x4000) != 0;
LABEL_4:
        if ( !v0 )
        {
            dwt_write32bitoffsetreg(15, 0, 606572544);
            continue;
        }
            dwt_write32bitoffsetreg(15, 0, 16512);
            v1 = (char)dwt_read32bitoffsetreg(16, 0);
            dwt_readrxdata(&rx_buffer, v1, 0);
        if ( (uint8)byte_794 % 0x2Du != ANCHOR_IND )
            continue;
        v23 = (uint8)byte_794 % 0x2Du;
        v2 = (uint8)byte_795;
        byte_794 = 0;
        byte_795 = 0;
        if ( !memcmp(&rx_buffer, &rx_poll_msg, 0xAu) )
        {
            *(uint64 *)&poll_rx_ts = get_rx_timestamp_u64();
            dwt_setrxaftertxdelay(500);
            dwt_setrxtimeout(3300);
            byte_801 = frame_seq_nb;
            byte_802 = v2;
            dwt_writetxdata(15, &tx_resp_msg, 0);
            dwt_writetxfctrl(15, 0);
            dwt_starttx(2);
            do
                status_reg = dwt_read32bitoffsetreg(15, 0);
            while ( (status_reg & 0x2427D000) == 0 );
            v0 = (status_reg & 0x4000) != 0;
            if ( (status_reg & 0x4000) == 0 )
                goto LABEL_4;
            dwt_write32bitoffsetreg(15, 0, 16512);
            v3 = dwt_read32bitoffsetreg(16, 0) & 0x7F;
            if ( v3 <= 0x18 )
                dwt_readrxdata(&rx_buffer, v3, 0);
            byte_794 = 0;
            if ( (uint8)byte_795 == v2 )
            {
                byte_795 = 0;
                if ( !memcmp(&rx_buffer, &rx_final_msg, 0xAu) )
                {
                    *(uint64 *)&resp_tx_ts = get_tx_timestamp_u64();
                    *(uint64 *)&final_rx_ts = get_rx_timestamp_u64();
                    final_msg_get_ts(&byte_79C, &v21);
                    final_msg_get_ts(&unk_7A0, &v22);
                    final_msg_get_ts(&unk_7A4, &v17);
                    v4 = (double)(int32)(v22 - v21);
                    v15 = HIDWORD(v4);
                    v16 = LODWORD(v4);
                    v5 = (double)(int32)(final_rx_ts - resp_tx_ts);
                    v13 = HIDWORD(v5);
                    v14 = LODWORD(v5);
                    v6 = (double)(int32)(v17 - v22);
                    v11 = HIDWORD(v6);
                    v12 = LODWORD(v6);
                    v7 = (double)(int32)(resp_tx_ts - poll_rx_ts);
                    v18 = HIDWORD(v7);
                    v10 = LODWORD(v7);
                    v8 = COERCE_DOUBLE(__PAIR64__(v15, v16))
                        + COERCE_DOUBLE(__PAIR64__(v13, v14))
                        + COERCE_DOUBLE(__PAIR64__(v11, v12))
                        + v7;
                    v19 = HIDWORD(v8);
                    v20 = LODWORD(v8);
                    tof = (double)(__int64)((COERCE_DOUBLE(__PAIR64__(v15, v16)) * COERCE_DOUBLE(__PAIR64__(v13, v14))
                                            - COERCE_DOUBLE(__PAIR64__(v11, v12)) * COERCE_DOUBLE(__PAIR64__(v18, v10)))
                                            / v8)
                        * 1.56500401e-11;
                    distance = tof * 299702547.0;
                    *(float *)&v8 = tof * 299702547.0;
                    distance = distance - dwt_getrangebias((uint8)config, LODWORD(v8), (uint8)byte_7E9);
                    v9 = (int)(distance * 100.0);
                    byte_830 = v9 / 100;
                    byte_831 = v9 % 100;
                    byte_832 = v23;
                    byte_828 = frame_seq_nb;
                    byte_829 = v2;
                    dwt_writetxdata(15, &distance_msg, 0);
                    dwt_writetxfctrl(15, 0);
                    dwt_starttx(0);
                }
            }
        }
        else if ( !(memcmp(&rx_buffer, &angle_msg, 0xAu) | (uint8)ANCHOR_IND) )
        {
            if ( byte_79D == 1 )
            {
                byte_795 = v2;
                USART_puts(&unk_79F, (uint8)byte_79E);
            }
            else
            {
                putc((uint8)byte_79C, (FILE *)&_stdout);
            }
        }
    }
}
