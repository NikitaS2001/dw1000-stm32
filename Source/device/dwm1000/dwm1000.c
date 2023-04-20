/**
 * DECOMPILED FROM DWM LIB (dwm1000.o) 
 * DON'T COMPILE
 */

#include "dwm1000.h"

#define LOWORD(l)           ((WORD)(((DWORD_PTR)(l)) & 0xffff))
#define HIWORD(l)           ((WORD)((((DWORD_PTR)(l)) >> 16) & 0xffff))
#define HIDWORD(dw, hw)     LOWORD(dw) | (hw << 16)
#define LODWORD(dw, lw)     (HIWORD(dw) << 16) | lw

#define byte_7F3 rx_poll_msg[0]
#define byte_7F4 rx_poll_msg[1]
#define byte_7F5 rx_poll_msg[2]
#define byte_7F6 rx_poll_msg[3]
#define byte_7F7 rx_poll_msg[4]
#define byte_7F8 rx_poll_msg[5]
#define byte_7F9 rx_poll_msg[6]
#define byte_7FA rx_poll_msg[7]
#define byte_7FB rx_poll_msg[8]
#define byte_7FC rx_poll_msg[9]
#define byte_7FD rx_poll_msg[10]
#define byte_7FE rx_poll_msg[11]

#define byte_7FF tx_resp_msg[0]
#define byte_800 tx_resp_msg[1]
#define byte_801 tx_resp_msg[2]
#define byte_802 tx_resp_msg[3]
#define byte_803 tx_resp_msg[4]
#define byte_804 tx_resp_msg[5]
#define byte_805 tx_resp_msg[6]
#define byte_806 tx_resp_msg[7]
#define byte_807 tx_resp_msg[8]
#define byte_808 tx_resp_msg[9]
#define byte_809 tx_resp_msg[10]
#define byte_80A tx_resp_msg[11]
#define byte_80B tx_resp_msg[12]
#define byte_80C tx_resp_msg[13]
#define byte_80D tx_resp_msg[14]

#define byte_80E rx_final_msg[0]
#define byte_80F rx_final_msg[1]
#define byte_810 rx_final_msg[2]
#define byte_811 rx_final_msg[3]
#define byte_812 rx_final_msg[4]
#define byte_813 rx_final_msg[5]
#define byte_814 rx_final_msg[6]
#define byte_815 rx_final_msg[7]
#define byte_816 rx_final_msg[8]
#define byte_817 rx_final_msg[9]
#define byte_818 rx_final_msg[10]
#define byte_819 rx_final_msg[11]
#define byte_81A rx_final_msg[12]
#define byte_81B rx_final_msg[13]
#define byte_81C rx_final_msg[14]
#define byte_81D rx_final_msg[15]
#define byte_81E rx_final_msg[16]
#define byte_81F rx_final_msg[17]
#define byte_820 rx_final_msg[18]
#define byte_821 rx_final_msg[19]
#define byte_822 rx_final_msg[20]
#define byte_823 rx_final_msg[21]
#define byte_824 rx_final_msg[22]
#define byte_825 rx_final_msg[23]

#define byte_826 distance_msg[0]
#define byte_827 distance_msg[1]
#define byte_828 distance_msg[2]
#define byte_829 distance_msg[3]
#define byte_82A distance_msg[4]
#define byte_82B distance_msg[5]
#define byte_82C distance_msg[6]
#define byte_82D distance_msg[7]
#define byte_82E distance_msg[8]
#define byte_82F distance_msg[9]
#define byte_830 distance_msg[10]
#define byte_831 distance_msg[11]
#define byte_832 distance_msg[12]
#define byte_833 distance_msg[13]
#define byte_834 distance_msg[14]

#define byte_835 tx_poll_msg[0]
#define byte_836 tx_poll_msg[1]
#define byte_837 tx_poll_msg[2]
#define byte_838 tx_poll_msg[3]
#define byte_839 tx_poll_msg[4]
#define byte_83A tx_poll_msg[5]
#define byte_83B tx_poll_msg[6]
#define byte_83C tx_poll_msg[7]
#define byte_83D tx_poll_msg[8]
#define byte_83E tx_poll_msg[9]
#define byte_83F tx_poll_msg[10]
#define byte_840 tx_poll_msg[11]

#define byte_841 rx_resp_msg[0]
#define byte_842 rx_resp_msg[1]
#define byte_843 rx_resp_msg[2]
#define byte_844 rx_resp_msg[3]
#define byte_845 rx_resp_msg[4]
#define byte_846 rx_resp_msg[5]
#define byte_847 rx_resp_msg[6]
#define byte_848 rx_resp_msg[7]
#define byte_849 rx_resp_msg[8]
#define byte_84A rx_resp_msg[9]
#define byte_84B rx_resp_msg[10]
#define byte_84C rx_resp_msg[11]
#define byte_84D rx_resp_msg[12]
#define byte_84E rx_resp_msg[13]
#define byte_84F rx_resp_msg[14]

#define byte_850 tx_final_msg[0]
#define byte_851 tx_final_msg[1]
#define byte_852 tx_final_msg[2]
#define byte_853 tx_final_msg[3]
#define byte_854 tx_final_msg[4]
#define byte_855 tx_final_msg[5]
#define byte_856 tx_final_msg[6]
#define byte_857 tx_final_msg[7]
#define byte_858 tx_final_msg[8]
#define byte_859 tx_final_msg[9]
#define byte_85A tx_final_msg[10]
#define byte_85B tx_final_msg[11]
#define byte_85C tx_final_msg[12]
#define byte_85D tx_final_msg[13]
#define byte_85E tx_final_msg[14]
#define byte_85F tx_final_msg[15]
#define byte_860 tx_final_msg[16]
#define byte_861 tx_final_msg[17]
#define byte_862 tx_final_msg[18]
#define byte_863 tx_final_msg[19]
#define byte_864 tx_final_msg[20]
#define byte_865 tx_final_msg[21]
#define byte_866 tx_final_msg[22]
#define byte_867 tx_final_msg[23]

#define byte_868 angle_msg[0]
#define byte_869 angle_msg[1]
#define byte_86A angle_msg[2]
#define byte_86B angle_msg[3]
#define byte_86C angle_msg[4]
#define byte_86D angle_msg[5]
#define byte_86E angle_msg[6]
#define byte_86F angle_msg[7]
#define byte_870 angle_msg[8]
#define byte_871 angle_msg[9]
#define byte_872 angle_msg[10]
#define byte_873 angle_msg[11]
#define byte_874 angle_msg[12]
#define byte_875 angle_msg[13]
#define byte_876 angle_msg[14]
#define byte_877 angle_msg[15]
#define byte_878 angle_msg[16]
#define byte_879 angle_msg[17]
#define byte_87A angle_msg[18]
#define byte_87B angle_msg[19]
#define byte_87C angle_msg[20]
#define byte_87D angle_msg[21]
#define byte_87E angle_msg[22]
#define byte_87F angle_msg[23]
#define byte_880 angle_msg[24]
#define byte_881 angle_msg[25]
#define byte_882 angle_msg[26]
#define byte_883 angle_msg[27]
#define byte_884 angle_msg[28]
#define byte_885 angle_msg[29]
#define byte_886 angle_msg[30]

#define byte_887 Semaphore_Release[0]
#define byte_888 Semaphore_Release[1]
#define byte_889 Semaphore_Release[2]
#define byte_88A Semaphore_Release[3]
#define byte_88B Semaphore_Release[4]
#define byte_88C Semaphore_Release[5]
#define byte_88D Semaphore_Release[6]
#define byte_88E Semaphore_Release[7]
#define byte_88F Semaphore_Release[8]
#define byte_890 Semaphore_Release[9]
#define byte_891 Semaphore_Release[10]
#define byte_892 Semaphore_Release[11]
#define byte_893 Semaphore_Release[12]

#define byte_894 Tag_Statistics[0]
#define byte_895 Tag_Statistics[1]
#define byte_896 Tag_Statistics[2]
#define byte_897 Tag_Statistics[3]
#define byte_898 Tag_Statistics[4]
#define byte_899 Tag_Statistics[5]
#define byte_89A Tag_Statistics[6]
#define byte_89B Tag_Statistics[7]
#define byte_89C Tag_Statistics[8]
#define byte_89D Tag_Statistics[9]
#define byte_89E Tag_Statistics[10]
#define byte_89F Tag_Statistics[11]
#define byte_8A0 Tag_Statistics[12]

#define byte_8A1 Master_Release_Semaphore[0]
#define byte_8A2 Master_Release_Semaphore[1]
#define byte_8A3 Master_Release_Semaphore[2]
#define byte_8A4 Master_Release_Semaphore[3]
#define byte_8A5 Master_Release_Semaphore[4]
#define byte_8A6 Master_Release_Semaphore[5]
#define byte_8A7 Master_Release_Semaphore[6]
#define byte_8A8 Master_Release_Semaphore[7]
#define byte_8A9 Master_Release_Semaphore[8]
#define byte_8AA Master_Release_Semaphore[9]
#define byte_8AB Master_Release_Semaphore[10]
#define byte_8AC Master_Release_Semaphore[11]
#define byte_8AD Master_Release_Semaphore[12]

#define byte_8AE Tag_Statistics_response[0]
#define byte_8AF Tag_Statistics_response[1]
#define byte_8B0 Tag_Statistics_response[2]
#define byte_8B1 Tag_Statistics_response[3]
#define byte_8B2 Tag_Statistics_response[4]
#define byte_8B3 Tag_Statistics_response[5]
#define byte_8B4 Tag_Statistics_response[6]
#define byte_8B5 Tag_Statistics_response[7]
#define byte_8B6 Tag_Statistics_response[8]
#define byte_8B7 Tag_Statistics_response[9]
#define byte_8B8 Tag_Statistics_response[10]
#define byte_8B9 Tag_Statistics_response[11]
#define byte_8BA Tag_Statistics_response[12]

#define byte_8BB Master_Release_Semaphore_comfirm[0]
#define byte_8BC Master_Release_Semaphore_comfirm[1]
#define byte_8BD Master_Release_Semaphore_comfirm[2]
#define byte_8BE Master_Release_Semaphore_comfirm[3]
#define byte_8BF Master_Release_Semaphore_comfirm[4]
#define byte_8C0 Master_Release_Semaphore_comfirm[5]
#define byte_8C1 Master_Release_Semaphore_comfirm[6]
#define byte_8C2 Master_Release_Semaphore_comfirm[7]
#define byte_8C3 Master_Release_Semaphore_comfirm[8]
#define byte_8C4 Master_Release_Semaphore_comfirm[9]
#define byte_8C5 Master_Release_Semaphore_comfirm[10]
#define byte_8C6 Master_Release_Semaphore_comfirm[11]
#define byte_8C7 Master_Release_Semaphore_comfirm[12]

uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; --i)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; --i)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
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
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
            Tag_Measure_Dis();
            v1 = 0;
            if ( TAG_ID != MASTER_TAG )
            {
                tx_final_msg[6] = frame_seq_nb;
                tx_final_msg[7] = TAG_ID;
                dwt_writetxdata(sizeof(Semaphore_Release), &Semaphore_Release, 0);
                dwt_writetxfctrl(sizeof(Semaphore_Release), 0);
                v5 = dwt_starttx(DWT_START_TX_IMMEDIATE);
                tick = portGetTickCnt(v5);
                while ( 1 )
                {
                    v7 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0) << 24;
                    if ( v7 < 0 )
                        break;
                    v6 = (int32)(portGetTickCnt(v7) - tick) > 0xC8;
LABEL_7:
                    if ( v6 )
LABEL_61:
                        NVIC_SystemReset();
                }
                GPIO_SetBits(GPIOA, GPIO_Pin_2);
            }
        }
        if ( TAG_ID == MASTER_TAG )
        {
            if ( !((int (*)(void))Sum_Tag_Semaphore_request)() )
            {
                for ( i = (uint8)SLAVE_TAG_START_INDEX; !i; i = 1 )
                {
                    v3 = 0;
                    tx_final_msg[19] = 0;
                    tx_final_msg[20] = 0;
                    dwt_writetxdata(13, &Tag_Statistics, 0);
                    dwt_writetxfctrl(13, 0);
                    v8 = dwt_starttx(DWT_RESPONSE_EXPECTED);
                    tick = portGetTickCnt(v8);
                    while ( 1 )
                    {
                        v10 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
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
                        dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 16512);
                        v11 = dwt_read32bitoffsetreg(RX_FINFO_ID, 0) & 0x7F;
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
                                GPIO_SetBits(GPIOA, GPIO_Pin_2);
                            }
                        }
                    }
                    else
                    {
                        dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 606572544);
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
                        angle_msg[8] = 0;
                        angle_msg[9] = 0;
                        dwt_writetxdata(13, &Master_Release_Semaphore, 0);
                        dwt_writetxfctrl(13, 0);
                        dwt_starttx(2);
                        do
                            status_reg = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
                        while ( (status_reg & 0x2427D000) == 0 );
                        if ( (status_reg & 0x4000) != 0 )
                        {
                            GPIO_SetBits(GPIOA, GPIO_Pin_1);
                            dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 16512);
                            v15 = dwt_read32bitoffsetreg(RX_FINFO_ID, 0) & 0x7F;
                            if ( v15 <= 0x18 )
                                dwt_readrxdata(&rx_buffer, v15, 0);
                            v14 = (int)&rx_buffer;
                            byte_794 = 0;
                            if ( !byte_795 )
                            {
                                byte_795 = 0;
                                GPIO_SetBits(GPIOA, GPIO_Pin_3);
                                v14 = memcmp(&rx_buffer, &Master_Release_Semaphore_comfirm, 0xAu);
                                if ( !v14 )
                                {
                                    v2 = 1;
LABEL_42:
                                    dwt_setrxtimeout(13500);
                                    dwt_rxenable(0);
                                    do
                                        status_reg = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
                                    while ( (status_reg & 0x2427D000) == 0 );
                                    if ( (status_reg & 0x4000) != 0 )
                                    {
                                        dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 0x4000);
                                        v16 = (char)dwt_read32bitoffsetreg(RX_FINFO_ID, 0);
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
                                        v14 = dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 606572544);
                                    }
                                    break;
                                }
                            }
                        }
                        else
                        {
                            Semaphore[0] = 0;
                            v14 = dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 606572544);
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
                v19 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
                status_reg = v19;
                if ( (v19 & 0x2427D000) != 0 )
                    break;
                if ( (int32)(portGetTickCnt(v19) - tick) > 0xC8 )
                goto LABEL_61;
            }
            if ( (status_reg & 0x4000) != 0 )
            {
                dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 16512);
                v20 = (char)dwt_read32bitoffsetreg(RX_FINFO_ID, 0);
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
                        GPIO_SetBits(GPIOA, GPIO_Pin_2);
                    }
                    if ( !memcmp(&rx_buffer, &Master_Release_Semaphore, 0xAu) )
                    {
                        word_88A = *(_WORD *)&frame_seq_nb;
                        dwt_writetxdata(13, &Master_Release_Semaphore_comfirm, 0);
                        dwt_writetxfctrl(13, 0);
                        dwt_starttx(0);
                        while ( (dwt_read32bitoffsetreg(SYS_STATUS_ID, 0) & 0x80) == 0 )
                        ;
                        v1 = 1;
                        GPIO_SetBits(GPIOA, GPIO_Pin_1);
                    }
                }
            }
            else
            {
                dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 606572544);
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
            status_reg = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
        while ( (status_reg & 0x2427D000) == 0 );
        v0 = (status_reg & SYS_STATUS_RXFCG) != 0;
LABEL_4:
        if ( !v0 )
        {
            dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 606572544);
            continue;
        }
        dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 16512);
        v1 = (char)dwt_read32bitoffsetreg(RX_FINFO_ID, 0);
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
            tx_resp_msg[2] = frame_seq_nb;
            tx_resp_msg[3] = v2;
            dwt_writetxdata(15, &tx_resp_msg, 0);
            dwt_writetxfctrl(15, 0);
            dwt_starttx(2);
            do
                status_reg = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
            while ( (status_reg & 0x2427D000) == 0 );
            v0 = (status_reg & 0x4000) != 0;
            if ( (status_reg & 0x4000) == 0 )
                goto LABEL_4;
            dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, 16512);
            v3 = dwt_read32bitoffsetreg(RX_FINFO_ID, 0) & 0x7F;
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
                    v4 = (double)(v22 - v21);
                    v5 = (double)(final_rx_ts - resp_tx_ts);
                    v6 = (double)(v17 - v22);
                    v7 = (double)(resp_tx_ts - poll_rx_ts);
                    v8 = v4 + v5 + v6 + v7;
                    tof = ((v4 * v5 - v6 * v7) / v8) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;
                    *(float *)&v8 = tof * SPEED_OF_LIGHT;
                    distance = distance - dwt_getrangebias((uint8)config, LODWORD(v8), (uint8)config.prf);
                    v9 = (int)(distance * 100.0);
                    distance_msg[2] = frame_seq_nb;
                    distance_msg[3] = v2;
                    distance_msg[10] = v9 / 100;
                    distance_msg[11] = v9 % 100;
                    distance_msg[12] = v23;
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
                putc((uint8)byte_79C, stdout);
            }
        }
    }
}
