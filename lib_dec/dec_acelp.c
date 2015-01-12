/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <memory.h>
#include <assert.h>
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "rom_basop_util.h"

#define _1_CODE 0x200       /*codebook excitation Q9 */

static void D_ACELP_decode_arithtrack(Word16 v[], Word32 s, Word16 p, Word16 trackstep, Word16 tracklen);

/*---------------------------------------------------------------------*
* Function D_ACELP_indexing()
*
*---------------------------------------------------------------------*/

void D_ACELP_indexing(
    Word16 code[],
    PulseConfig config,
    Word16 num_tracks,
    Word16 index[]
)
{
    Word16 track, pulses, k, pulsestrack[4];
    Word32 s;
    Word16 trackpos;
    UWord16 *idxs;
    UWord32 idxs32[4], index_n[NB_TRACK_FCB_4T];
    Word16 restpulses, wordcnt, wordcnt32;

    wordcnt = shr(add(config.bits,15),4);     /* ceil(bits/16) */

    /* check if some tracks have more pulses */
    restpulses = s_and((Word16)config.nb_pulse, sub(num_tracks, 1));

    /* cast to short */
    idxs= (UWord16 *)idxs32;
    FOR (k=0; k<wordcnt; k++)
    {
        idxs[k] = index[k];
        move16();
    }
    idxs[wordcnt] = 0;
    move16();

    /*init 32bits wordcnt*/
    wordcnt32 = shr(add(wordcnt,1),1);

    IF (restpulses)
    {
        /* check if we need to code track positions */
        SWITCH (config.codetrackpos)
        {
        case TRACKPOS_FREE_THREE:
            /* Find track with less pulses */
            trackpos = s_and(idxs[0], 3);
            longshr(idxs32,2,wordcnt32);

            /* set number of pulses per track */
            set16_fx(pulsestrack, add(shr((Word16)config.nb_pulse,2),1),4);
            cast16();
            pulsestrack[trackpos] = sub(pulsestrack[trackpos],1);    /* this one has less pulses */  move16();
            BREAK;
        case TRACKPOS_FREE_ONE:
            /* Find track with more pulses */
            trackpos = s_and(idxs[0], 3);
            longshr(idxs32,2,wordcnt32);

            /* set number of pulses per track */
            set16_fx(pulsestrack, shr((Word16)config.nb_pulse,2),4);
            cast16();
            pulsestrack[trackpos] = add(pulsestrack[trackpos],1);    /* this one has less pulses */ move16();
            BREAK;
        case TRACKPOS_FIXED_EVEN:
            /* Pulses on even tracks */
            pulsestrack[0] = shr(add((Word16)config.nb_pulse,1), 1);
            cast16();
            move16();
            pulsestrack[1] = 0;
            move16();
            pulsestrack[2] = shr((Word16)config.nb_pulse, 1);
            cast16();
            move16();
            pulsestrack[3] = 0;
            move16();
            BREAK;
        case TRACKPOS_FIXED_FIRST:
            /* set number of pulses per track */
            set16_fx(pulsestrack, shr((Word16)config.nb_pulse,2),4);
            cast16();
            FOR (k=0; k<restpulses; k++)
            {
                pulsestrack[k] = add(pulsestrack[k],1);
                move16();
            }
            BREAK;
        case TRACKPOS_FIXED_FIRST_TWO:
            /* 1000, 0100 */
            trackpos = s_and(idxs[0], 1);
            longshr(idxs32,1,wordcnt32);

            /* set number of pulses per track */
            set16_fx(pulsestrack, shr((Word16)config.nb_pulse,2),4);
            cast16();
            pulsestrack[trackpos] = add(pulsestrack[trackpos],1);
            move16();
            BREAK;

        case TRACKPOS_FIXED_TWO:
            /* 1100, 0110, 0011, 1001 */
            /* Find track with less pulses */
            trackpos = s_and(idxs[0], 3);
            longshr(idxs32,2,wordcnt32);

            /* set number of pulses per track */
            set16_fx(pulsestrack, shr((Word16)config.nb_pulse,2),4);
            cast16();
            pulsestrack[trackpos] = add(pulsestrack[trackpos],1);
            move16();
            trackpos = add(trackpos,1);
            trackpos = s_and(trackpos,3);
            pulsestrack[trackpos] = add(pulsestrack[trackpos],1);
            move16();
            BREAK;
        default:
            assert(0);
            BREAK;
        }
    }
    ELSE
    {
        /* set number of pulses per track */
        set16_fx(pulsestrack, shr((Word16)config.nb_pulse,2),4);
        cast16();
    }

    IF (sub(config.bits, 43) == 0)
    {
        D_ACELP_decode_43bit(idxs, code, pulsestrack);
    }
    ELSE
    {
        fcb_pulse_track_joint_decode(idxs, wordcnt, index_n, pulsestrack, num_tracks);
        FOR (track = num_tracks - 1; track >=1; track--)
        {
            pulses = pulsestrack[track];
            move16();

            IF (pulses)
            {
                /* divide by number of possible states: rest is actual state and
                 * the integer part goes to next track */
                s = index_n[track];
                /* decode state to actual pulse positions on track */
                D_ACELP_decode_arithtrack(code+track, s, pulses, num_tracks, 16);
            }
            ELSE    /* track is empty */
            {
                FOR (k=track; k < 16*num_tracks; k+=num_tracks)
                {
                    code[k] = 0;
                    move16();
                }
            }
        }

        s = L_add(index_n[0], 0);
        pulses = pulsestrack[0];
        move16();

        if (s >= pulsestostates[16][pulses-1])
        {
            /* fatal error. Input bit-stream does not adhere to standard. */
            assert(0);
        }

        IF (pulses)
        {
            D_ACELP_decode_arithtrack(code, s, pulses, num_tracks, 16);
        }
        ELSE {/* track is empty */
            FOR (k=0; k < 16*num_tracks; k+=num_tracks)
            {
                code[k] = 0;
                move16();
            }
        }
    }
}

static void D_ACELP_decode_arithtrack(Word16 v[], Word32 s, Word16 p, Word16 trackstep, Word16 tracklen)
{
    Word16 k, idx;

    /*initialy s was UWords32 but it seems that s is never greater then 0x80000000*/
    /*this assumption reduces complexity but if it is not true than exit*/
    assert(s >= 0);

    FOR (k=(tracklen)-1; k>= 0; k--)
    {
        idx = imult1616(k,trackstep);
        v[idx] = 0;          /* default: there is no pulse here */   move16();

        FOR(; p; p--)  /* one pulse placed, so one less left */
        {
            IF (L_sub(s, pulsestostates[k][p-1]) < 0)
            {
                BREAK;
            }

            s = L_sub(s, pulsestostates[k][p-1]);

            IF (v[idx] != 0)   /* there is a pulse here already = sign is known */
            {
                if (v[idx] > 0)
                {
                    v[idx] = add(v[idx],_1_CODE); /* place one more pulse here */     move16();
                }
                if (v[idx] <= 0)
                {
                    v[idx] = sub(v[idx],_1_CODE); /* place one more pulse here */     move16();
                }
            }
            ELSE      /* this is the first pulse here -> determine sign */
            {
                v[idx] = +_1_CODE; /* place a negative pulse here */              move16();
                if (L_and(s, 0x1) != 0)
                {
                    v[idx] = -_1_CODE; /* place a negative pulse here */           move16();
                }
                s = L_lshr(s,1);
            }
        }
    }
}

void fcb_pulse_track_joint_decode(UWord16 *idxs, Word16 wordcnt, UWord32 *index_n, Word16 *pulse_num, Word16 track_num)
{
    Word16 hi_to_low[10] = { 0, 0, 0, 3, 9, 5, 3, 1, 8, 8};
    Word16 low_len[10]   = {0, 0, 8, 5, 7, 11, 13, 15, 16, 16};
    Word32 low_mask[10] = {0, 0, 255, 31, 127, 2047, 8191, 32767, 65535, 65535};
    Word16 indx_fact[10] = {0, 0, 2, 172, 345, 140, 190, 223, 463, 1732};
    Word16 index_len[3] = {0, 5, 9};
    UWord32 index_mask[3] = {0, 31, 511};

    UWord32 index;
    Word32 indx_tmp;
    Word16 indx_flag, indx_flag_1;
    Word16 track, track_num1, pulse_num0, pulse_num1;
    Word32 div_tmp;
    Word16 indx_flag_2;

    indx_flag = 0;
    move16();
    indx_flag_1 = 0;
    move16();
    indx_flag_2 = 0;
    move16();

    FOR (track = 0; track < track_num; track++)
    {
        indx_flag = add(indx_flag, shr(pulse_num[track], 2));
        indx_flag_1 = add(indx_flag_1, shr(pulse_num[track], 1));
        indx_flag_2 = add(indx_flag_2, shr(pulse_num[track], 3));
    }

    hi_to_low[4] = 1;
    move16();
    if (sub(indx_flag, track_num) >= 0)
    {
        hi_to_low[4] = 9;
        move16();
    }

    hi_to_low[7] = 1;
    move16();
    if (sub(indx_flag_2, 1) >= 0)
    {
        hi_to_low[7] = 9;
        move16();
    }

    IF (sub(indx_flag_1, track_num) >= 0)
    {
        IF (sub(indx_flag, track_num) >= 0)
        {
            index = L_deposit_l(0);
            IF (sub(indx_flag_2, 1) >= 0)
            {
                FOR (track = sub(wordcnt, 1); track >= 6; track--)
                {
                    index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
                }
                index_n[3] = L_add(L_lshl(idxs[5], 8), L_and(L_lshr(idxs[4], 8), 0xFF));
                move32();
                index_n[2] = L_and(L_add(L_lshl(idxs[4], 16), idxs[3]), 0xFFFFFF);
                move32();
                index_n[1] = L_add(L_lshl(idxs[2], 8), L_and(L_lshr(idxs[1], 8), 0xFF));
                move32();
                index_n[0] = L_and(L_add(L_lshl(idxs[1], 16), idxs[0]), 0xFFFFFF);
                move32();
            }
            ELSE
            {
                FOR (track = (wordcnt-1); track >= track_num; track--)
                {
                    index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
                }
                FOR (track = 0; track < track_num; track++)
                {
                    index_n[track] = (UWord32) idxs[track];
                    move32();
                }
            }
        }
        ELSE
        {
            IF (sub(track_num, 4) == 0)
            {
                index = L_deposit_l(0);
                FOR (track = (wordcnt-1); track >= 2; track--)
                {
                    index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
                }

                index_n[3] = L_and((Word32) idxs[1], 0xFF);
                move32();
                index_n[2] = L_lshr((Word32) idxs[1], 8);
                move32();
                index_n[1] = L_and((Word32) idxs[0], 0xFF);
                move32();
                index_n[0] = L_lshr((Word32) idxs[0], 8);
                move32();
            }
            ELSE
            {
                index = L_deposit_l(0);
                FOR (track = (wordcnt-1); track >= 3; track--)
                {
                    index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
                }
                index = L_add(L_lshl(index, 8), L_and((Word32) idxs[2], 0xFF));

                index_n[4] = L_lshr((Word32) idxs[2], 8);
                move32();
                index_n[3] = L_and((Word32) idxs[1], 0xFF);
                move32();
                index_n[2] = L_lshr((Word32) idxs[1], 8);
                move32();
                index_n[1] = L_and((Word32) idxs[0], 0xFF);
                move32();
                index_n[0] = L_lshr((Word32) idxs[0], 8);
                move32();
            }
        }

        track_num1 = sub(track_num, 1);
        pulse_num1 = pulse_num[track_num1];
        move16();
        index = L_add(L_lshl(index, hi_to_low[pulse_num1]), L_lshr(index_n[track_num1], low_len[pulse_num1]));
        FOR (track = (track_num-1); track > 0; track--)
        {
            track_num1 = sub(track, 1);
            pulse_num0 = pulse_num[track_num1];
            move16();
            pulse_num1 = pulse_num[track];
            move16();
            index = L_add(L_lshl(index, hi_to_low[pulse_num0]), L_lshr(index_n[track_num1], low_len[pulse_num0]));

            iDiv_and_mod_32(index, indx_fact[pulse_num1], &div_tmp, &indx_tmp, 0);
            index_n[track] = L_add(L_and(index_n[track], low_mask[pulse_num1]), L_lshl(indx_tmp, low_len[pulse_num1]));
            index = L_add(div_tmp, 0);
        }
        pulse_num1 = pulse_num[0];
        move16();
        index_n[0] = L_add(L_and(index_n[0], low_mask[pulse_num1]), L_lshl(index, low_len[pulse_num1]));
        move32();
    }
    ELSE
    {
        IF (sub(track_num, 4) == 0)
        {
            index = L_deposit_l(0);
            FOR (track = (wordcnt-1); track >= 0; track--)
            {
                index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
            }
        }
        ELSE
        {
            index = L_deposit_l(0);
            FOR (track = (wordcnt-1); track > 0; track--)
            {
                index = L_add(L_lshl(index, 16), (UWord32) idxs[track]);
            }
            pulse_num1 = pulse_num[4];
            index_n[4] = L_and(index, index_mask[pulse_num1]);
            move32();
            index = L_lshr(index, index_len[pulse_num1]);
            index = L_add(L_lshl(index, 16), (UWord32) idxs[0]);
        }
        FOR (track = 3; track > 0; track--)
        {
            pulse_num1 = pulse_num[track];
            move16();
            index_n[track] = L_and(index, index_mask[pulse_num1]);
            move32();
            index = L_lshr(index, index_len[pulse_num1]);
        }
        index_n[0] = index;
        move32();
    }

    return;
}

