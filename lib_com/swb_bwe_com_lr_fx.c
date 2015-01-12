/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <math.h>
#include "options.h"
#include "cnst_fx.h"

#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"        /* required for wmc_tool */
#include "basop_mpy.h"

/*-------------------------------------------------------------------*
 * GetPredictedSignal()
 *
 * Routine for calculating the predicted signal
 *-------------------------------------------------------------------*/
void GetPredictedSignal_fx(
    const Word16 *predBuf_fx,          /* i:  Q8     */
    Word32 *L_outBuf,            /* o:  Q9     */
    const Word16 lag_fx,               /* i:  Q0     */
    const Word16 fLen_fx,              /* i:  Q0     */
    const Word16 lagGains_fx,          /* i:  Qgain  */
    const Word16 Qgain                 /* i:  Q0     */
)
{
    Word16 i;
    const Word16 *p_predBuf;
    Word32 *p_L_outBuf;

    p_predBuf  = predBuf_fx + lag_fx;
    p_L_outBuf = L_outBuf;

    FOR (i=0; i<fLen_fx; i++)
    {
        /* Q8 x Q0 --> Q9, 9+7-16=Q0 */
        *p_L_outBuf++ = L_shr(L_mult(*p_predBuf++, lagGains_fx), Qgain);
        move32();
    }
}

/*-------------------------------------------------------------------*
 * est_freq_har_decis_fx()
 *
 * Harmonic frequency decision matrix
 *-------------------------------------------------------------------*/
static
void est_freq_har_decis_fx(
    Word16 *har_freq_est1,       /* o: harmonic analysis 1 */
    Word16 *har_freq_est2,       /* o: harmonic analysis 2 */
    Word16 sharp,                /* i: pka-avg for group 1 */
    Word16 sharp1,               /* i: pka-avg for group 2 */
    Word16 hfe_est_countk1,      /* i: group pks count 1   */
    Word16 hfe_est_countk2,      /* i: group pks count 2   */
    Word16 k,                    /* i: group count         */
    Word16 k1,                   /* i:                     */
    Word16 k2,                   /* i:                     */
    Word16 *prev_frm_hfe2        /* i: harmonic estimation */
)
{
    Word16 temp_hfe2 = 0;
    Word16 har_freq_est2_2;
    Word16 prev_frm_hfe2_2;

    IF( k != 0 )
    {
        *har_freq_est1 = div_s_ss(sharp, k);
    }

    test();
    test();
    IF( sub(k1, 1) > 0 )
    {
        *har_freq_est2 = div_s_ss(sharp1, k1);
    }
    ELSE IF( sub(k1, 2) < 0 && ( k2 != 0 || sub(k, 1) > 0) )
    {
        *har_freq_est2 = *har_freq_est1;
        move16();
    }
    ELSE
    {
        test();
        test();
        test();
        IF((hfe_est_countk1 != 0 || hfe_est_countk2 != 0) && (k1 == 0 && k2 == 0))
        {
            *har_freq_est2 = (*har_freq_est1);
            move16();
        }
        ELSE
        {
            *har_freq_est2 = shl(*har_freq_est1, 1);
            move16();
        }
    }

    /* Consider Estimation Error upto 200Hz */
    test();
    test();
    test();
    har_freq_est2_2 = shl(*har_freq_est2, 1);
    prev_frm_hfe2_2 = shl(*prev_frm_hfe2, 1);
    IF( *prev_frm_hfe2 != 0 && ( sub(abs_s(sub(*prev_frm_hfe2, *har_freq_est2)), 10) < 0 || sub(abs_s(sub(*prev_frm_hfe2, har_freq_est2_2)),10) < 0 ) )
    {
        *har_freq_est2 = *prev_frm_hfe2;
        move16();
    }
    ELSE IF(*prev_frm_hfe2 != 0 && sub(abs_s(sub(*har_freq_est2, prev_frm_hfe2_2)), 10) < 0)
    {
        *har_freq_est2 = prev_frm_hfe2_2;
        move16();
    }
    ELSE
    {
        temp_hfe2 = shr(add(*prev_frm_hfe2, *har_freq_est2), 1);
        move16();

        IF( sub(abs_s(sub(temp_hfe2, *prev_frm_hfe2)), 2) < 0 )
        {
            temp_hfe2 =*prev_frm_hfe2;
            move16();
            *har_freq_est2 = temp_hfe2;
            move16();
        }
    }

    test();
    test();
    if( sub(*har_freq_est2, *har_freq_est1) < 0 && (  sub(k, 1) > 0 && sub(k1, 2) < 0 ) )
    {
        *har_freq_est2 = *har_freq_est1;
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * har_est_fx()
 *
 * Harmonic Structure analysis using LF spectrum
 *--------------------------------------------------------------------------*/

Word16 har_est_fx(
    Word32 L_spectra[],                   /* i  : coded spectrum                 */
    Word16 N,                             /* i  : length of the desired spectrum */
    Word16 *har_freq_est1,                /* i/o: Estimation harmonics 1         */
    Word16 *har_freq_est2,                /* o  : Estimation harmonics 2         */
    Word16 *flag_dis,                     /* i/o: flag for BWE reconstruction    */
    Word16 *prev_frm_hfe2,                /* i/o: Estimated harmonic update      */
    const Word16 subband_search_offset[], /* i  : Subband Search range           */
    const Word16 sbWidth[],               /* i  : Subband Search range           */
    Word16 *prev_stab_hfe2                /* i/o: Estimated harmonic position    */
)
{
    Word32 L_peak;
    Word32 L_input_abs[L_FRAME32k], L_blk_peak[30];
    Word32 L_blk_peak_te[30];
    Word32 L_blk_peak_max;
    Word32 *p_L_blk_peak, *pm1_L_blk_peak;

    Word16 i, j, q, k , k1, k2;
    Word16 blk_end,blk_st;
    Word16 peak_pos,blk_peak_pos[30], diff_peak_pos[30], sharp, sharp1, sharp2;
    Word16 min_har_pos;
    Word16 blk_peak_pos_te[30];
    Word16 temp;
    Word16 hfe_est_countk,hfe_est_countk1,hfe_est_countk2;
    Word16 r1, r2, r3;
    Word16 start_pos;
    Word16 blk_peak_pos_max;

    Word16 nlags, nlags_half, ct_hfsb2,sum_diff;
    Word16 blk_peak_pos_hfsb2[30],diff_peak_pos_hfsb2[30];
    Word16 rem_hfe2 ,q_diffpos_hfe2, diff_posmax_hfe2, q_diffpos_prevhfe2;

    Word16 blk_end_LEN;

    Word16 *p_blk_peak_pos, *pm1_blk_peak_pos;
    Word16 *p_diff_peak_pos,*pm1_diff_peak_pos;
    Word16 blk_peak_max_idx, blk_peak_pos_max_diff, diff_peak_pos_te[30];
    Word16 thr1, thr2;

    set32_fx(L_input_abs, 0x0L, L_FRAME32k);
    set32_fx(L_blk_peak, 0x0L, 30);
    set16_fx(blk_peak_pos, 0, 30);
    set16_fx(blk_peak_pos_te, 0, 30);

    rem_hfe2 = 0;
    move16();
    q_diffpos_hfe2 = 0;
    move16();
    diff_posmax_hfe2 = 0;
    move16();
    q_diffpos_prevhfe2 = 0;
    move16();

    set16_fx(diff_peak_pos,0,30);

    r1 = SWB_HAR_RAN1;
    move16();
    r2 = SWB_HAR_RAN2;
    move16();
    r3 = SWB_HAR_RAN3;
    move16();
    start_pos = r1;
    move16();

    /* Copy the abs values of LF spectrum*/
    FOR ( i = start_pos; i < N; i++)
    {
        L_input_abs[i] = L_abs(L_spectra[i]);
        move32();
    }

    blk_end = div_s_ss(N, LR_BLK_LEN);
    blk_st = div_s_ss(start_pos, LR_BLK_LEN);

    /*if( N/(LR_BLK_LEN) - blk_end > 0.0f) */
    blk_end_LEN = i_mult(blk_end, LR_BLK_LEN);
    if( sub(N, blk_end_LEN) > 0 )
    {
        blk_end = add(blk_end, 1);
    }

    /* initialization of over buffer for fractional point */
    temp = i_mult(blk_end, LR_BLK_LEN);
    FOR(i=N; i<temp; i++)
    {
        L_input_abs[i] = L_deposit_l(0);
    }

    q = start_pos;
    move16();

    /* Block Processing, to detect the spectral peaks*/
    FOR(i = blk_st; i < blk_end; i++)
    {
        L_peak = L_deposit_l(0);
        peak_pos = 0;
        move16();

        FOR(j = 0; j < LR_BLK_LEN; j ++)
        {
            IF ( L_sub(L_input_abs[q], L_peak) > 0x0L )
            {
                L_peak = L_input_abs[q];
                move32();
                peak_pos = q;
                move16();
            }

            test();
            test();
            test();
            IF( sub(i, blk_st) > 0 && L_input_abs[q] != 0x0L && L_sub(L_input_abs[q], L_peak) == 0 && sub(sub(peak_pos, blk_peak_pos[i-1]), LR_HLF_PK_BLK_LEN) < 0 )
            {
                L_peak = L_input_abs[q];
                move32();
                peak_pos = q;
                move16();
            }
            q = add(q, 1);
        }

        L_blk_peak[i] = L_peak;
        move32();
        blk_peak_pos[i] = peak_pos;
        move16();
    }

    p_L_blk_peak   = &L_blk_peak[blk_st];
    pm1_L_blk_peak = &L_blk_peak[sub(blk_st, 1)];
    p_blk_peak_pos   = &blk_peak_pos[blk_st];
    pm1_blk_peak_pos = &blk_peak_pos[sub(blk_st, 1)];
    FOR(i = blk_st; i < blk_end; i++)
    {
        IF( sub(i, blk_st) > 0 )
        {

            test();
            IF( *p_blk_peak_pos != 0 && *pm1_blk_peak_pos != 0 )
            {
                IF( sub(sub(*p_blk_peak_pos, *pm1_blk_peak_pos), LR_LOWBAND_DIF_PK_LEN) < 0 )
                {
                    IF( L_sub(*p_L_blk_peak, *pm1_L_blk_peak) > 0 )
                    {
                        *pm1_L_blk_peak   = L_deposit_l(0);
                        *pm1_blk_peak_pos = 0;
                        move16();
                    }
                    ELSE
                    {
                        *p_L_blk_peak = *pm1_L_blk_peak;
                        move32();
                        *p_blk_peak_pos = *pm1_blk_peak_pos;
                        move16();
                        *pm1_L_blk_peak = L_deposit_l(0);
                        *pm1_blk_peak_pos = 0;
                        move16();
                    }
                }
            }
        }
        p_L_blk_peak++;
        pm1_L_blk_peak++;

        p_blk_peak_pos++;
        pm1_blk_peak_pos++;
    }

    /* peak counts in each group */
    j = 0;
    move16();
    hfe_est_countk = 0;
    move16();
    hfe_est_countk1 = 0;
    move16();
    hfe_est_countk2 = 0;
    move16();
    FOR(i = blk_st; i < blk_end; i++)
    {
        IF(blk_peak_pos[i] != 0 )
        {
            blk_peak_pos_te[j] = blk_peak_pos[i];
            move16();
            IF( sub(blk_peak_pos[i], r2) < 0 )
            {
                hfe_est_countk = add(hfe_est_countk, 1);
            }
            ELSE IF( sub(blk_peak_pos[i], r3) < 0 )
            {
                hfe_est_countk1 = add(hfe_est_countk1, 1);
            }
            ELSE
            {
                hfe_est_countk2 = add(hfe_est_countk2, 1);
            }
            L_blk_peak_te[j] = L_blk_peak[i];
            move32();
            j = add(j, 1);
        }
    }

    min_har_pos = SWB_HAR_RAN1;
    move16();
    temp = 0;
    move16();
    L_blk_peak_max = L_blk_peak_te[0];
    move32();
    blk_peak_pos_max = blk_peak_pos_te[0] ;
    move16();
    blk_peak_max_idx = 0;
    move16();


    p_diff_peak_pos   = &diff_peak_pos[1];
    pm1_diff_peak_pos = &diff_peak_pos[1-1];
    FOR(i = 1; i < j; i++)
    {
        *pm1_diff_peak_pos = sub(blk_peak_pos_te[i], blk_peak_pos_te[sub(i,1)]);
        if( sub(*pm1_diff_peak_pos, min_har_pos) <= 0 )
        {
            min_har_pos = *pm1_diff_peak_pos;
            move16();
        }

        IF( L_sub(L_blk_peak_te[sub(i,1)],  L_blk_peak_max) > 0x0L )
        {
            L_blk_peak_max = L_blk_peak_te[sub(i,1)];
            move32();
            blk_peak_pos_max = blk_peak_pos_te[sub(i,1)];
            move16();
            blk_peak_max_idx = sub(i,1);
        }

        temp = add(temp, 1);

        p_diff_peak_pos++;
        pm1_diff_peak_pos++;
    }
    blk_peak_pos_max_diff = diff_peak_pos[blk_peak_max_idx];
    move16();

    /* Decision for BWE reconstruction */
    test();
    test();
    test();
    IF((sub(hfe_est_countk, 2) < 0 && sub(hfe_est_countk1, 2) < 0 && sub(hfe_est_countk2, 2) <0  ) || sub(min_har_pos, SWB_HAR_RAN1) >= 0 )
    {
        *flag_dis = 0;
        move16();
        test();
        test();
        test();
        if( (sub(hfe_est_countk, 1) == 0 && sub(hfe_est_countk1, 1) == 0) && (sub(hfe_est_countk2, 1) == 0 || hfe_est_countk2 == 0) )
        {
            *flag_dis = 1;
            move16();
        }
    }
    thr1 = add(blk_peak_pos_max_diff, LR_LOWBAND_DIF_PK_LEN);
    FOR(i=0; i<temp; i++)
    {
        if( sub(thr1, diff_peak_pos[i]) < 0 )
        {
            diff_peak_pos[i] = 0;
            move16();
        }
    }
    Copy(diff_peak_pos,diff_peak_pos_te,temp);
    set16_fx(diff_peak_pos,-1,temp);
    j=0;
    FOR(i=0; i<temp; i++)
    {
        IF(diff_peak_pos_te[i] != 0)
        {
            diff_peak_pos[j] = diff_peak_pos_te[i];
            move16();
            j = add(j, 1);
        }
    }
    temp = j;
    move16();

    /* harmonic estimation analysis to perform BWE Reconstruction */
    IF( *flag_dis )
    {
        sharp = 0;
        move16();
        k = 0;
        move16();
        k1 = 0;
        move16();
        sharp1 = 0;
        move16();
        sharp2 = 0;
        move16();
        k2 = 0;
        move16();

        q = 1;
        move16();
        thr1 = add(min_har_pos, LR_LOWBAND_DIF_PK_LEN);
        thr2 = add(min_har_pos, shl(LR_LOWBAND_DIF_PK_LEN, 1));
        FOR(i=0; i<temp; i++)
        {
            test();
            test();
            IF( sub(diff_peak_pos[i], thr1) <= 0 && diff_peak_pos[i] > 0)
            {
                sharp = add(sharp, diff_peak_pos[i]);
                k = add(k, 1);
            }
            ELSE IF( sub(diff_peak_pos[i], thr2) <= 0 && diff_peak_pos[i] > 0)
            {
                sharp1 = add(sharp1, diff_peak_pos[i]);
                k1 = add(k1, 1);
            }
            ELSE IF ( diff_peak_pos[i] > 0 )
            {
                sharp2 = add(sharp2, diff_peak_pos[i]);
                k2 = add(k2, 1);
            }
            q = add(q, 1);
        }

        est_freq_har_decis_fx(har_freq_est1,har_freq_est2,sharp,sharp1,hfe_est_countk1,hfe_est_countk2,k,k1,k2,prev_frm_hfe2);

        blk_peak_pos_max = blk_peak_pos_te[sub(temp,1)];
        move16();

        test();
        test();
        IF((*prev_stab_hfe2) > 0 && (*prev_frm_hfe2) > 0 && *prev_stab_hfe2 < N)
        {
            rem_hfe2 = sub(*har_freq_est2, extract_h(L_shl(L_mult(div_s_ss(*har_freq_est2, *prev_frm_hfe2), *prev_frm_hfe2), 15)));
            diff_posmax_hfe2 = abs_s(sub(blk_peak_pos_max, *prev_stab_hfe2));
            IF( rem_hfe2 == 0 )
            {
                test();
                IF( sub(diff_posmax_hfe2, 9) < 0 || *har_freq_est2 == 0 )
                {
                    blk_peak_pos_max = *prev_stab_hfe2;
                    move16();
                }
                ELSE
                {
                    q_diffpos_hfe2 = div_s_ss(diff_posmax_hfe2, *har_freq_est2);
                    q_diffpos_prevhfe2 = div_s_ss(diff_posmax_hfe2, *prev_frm_hfe2);
                    test();
                    IF( sub(q_diffpos_hfe2, 10) < 0 || sub(q_diffpos_prevhfe2, 10) < 0)
                    {
                        blk_peak_pos_max = *prev_stab_hfe2;
                        move16();
                    }
                    ELSE
                    {
                        *prev_stab_hfe2 = blk_peak_pos_max;
                        move16();
                    }
                }
            }
            ELSE
            {
                *prev_stab_hfe2 = blk_peak_pos_max;
                move16();
            }
        }
        ELSE
        {
            *prev_stab_hfe2 = blk_peak_pos_max;
            move16();
        }

        test();
        if( *har_freq_est1 == 0 || *har_freq_est2 == 0 )
        {
            *flag_dis = 0;
            move16();
        }
    }

    IF( *flag_dis == 0 )
    {
        IF( *prev_frm_hfe2 != 0 )
        {
            *har_freq_est2 = *prev_frm_hfe2;
            move16();
        }
        ELSE
        {
            nlags = shl(1, bits_lagIndices_mode0_Har_fx[0]);
            nlags_half = shr(nlags, 1);
            ct_hfsb2 = 0;
            move16();
            FOR(i = 0; i < j; i++)
            {
                test();
                IF( sub(blk_peak_pos_te[i], sub(subband_search_offset[0], nlags_half)) >= 0
                &&
                sub(blk_peak_pos_te[i], add(add(subband_search_offset[0], sbWidth[0]), nlags_half)) < 0)
                {
                    blk_peak_pos_hfsb2[ct_hfsb2] = blk_peak_pos_te[i];
                    move16();
                    ct_hfsb2 = add(ct_hfsb2, 1);
                    move16();
                }
            }

            IF( sub(ct_hfsb2, 1) > 0 )
            {
                sum_diff = 0;
                move16();
                FOR(i=1; i<ct_hfsb2; i++)
                {
                    diff_peak_pos_hfsb2[i-1] = sub(blk_peak_pos_hfsb2[i], blk_peak_pos_hfsb2[i-1]);
                    sum_diff = add(sum_diff, diff_peak_pos_hfsb2[i-1]);
                }
                *har_freq_est2 = div_s_ss(sum_diff, ct_hfsb2);
            }
            ELSE
            {
                *har_freq_est2 = min_har_pos;
                move16();
            }
        }
    }


    return blk_peak_pos_max;
}

void genhf_noise_fx(
    Word16 noise_flr_fx[],             /* i  : Qss smoothed non tonal                           */ /* sspectra_diff_fx:Qss */
    Word16 Qss,                        /* i  : Q0  Q value                                      */
    Word32 L_xSynth_har[],             /* o  : QsL hf non tonal components                      */ /* xSynth_har:QsL */
    Word16 QsL,                        /* i  : Q0  Q value                                      */
    Word16 *predBuf_fx,                /* i  : Qss smoothed tonal compone                       */ /* sspectra:Qss */
    Word16 bands,                      /* i  : Q0  total number of subbands in a frame          */
    Word16 harmonic_band,              /* i  : Q0  Number of LF harmonic frames                 */
    Word16 har_freq_est2,              /* i  : Q0  harmonic signal parameter                    */
    Word16 pos_max_hfe2,               /* i  : Q0  last pulse in core coder                     */
    Word16 *pul_res,                   /* o  : Q0  pulse resolution                             */
    GainItem_fx pk_sf_fx[],            /* o  :     representative region                        */
    const Word16 fLenLow,                    /* i  : Q0  low frequency length                         */
    const Word16 fLenHigh,                   /* i  : Q0  high frequency length                        */
    const Word16 sbWidth[],                  /* i  : Q0  bandwidth for high bands                     */
    const Word16 lagIndices[],               /* i  : Q0  correlation indices for most representative  */
    const Word16 subband_offsets[],          /* i  : Q0  band offsets for HF reconstruction           */
    const Word16 subband_search_offset[]     /* i  : Q0  most representative regions offsets in LF    */
)
{
    Word16 k,j,ii,st_pos,dst_pos;
    Word16 nlags[NB_SWB_SUBBANDS_HAR_SEARCH_SB];
    Word32 L_tmpbuf[L_FRAME32k];
    Word16 hfband_end[NB_SWB_SUBBANDS];
    Word16 rem_hfe,/*last_peakpos,*/ temp_last_peakpos,i,l,pos,res;
    Word16 hf_pulse_peaks_fx[160],pulse_peak_sb_fx[320]; /* Qss */
    Word16 st_last_peakpos;
    Word16 tmp_fx;

    set32_fx(L_tmpbuf, 0x0L, L_FRAME32k);
    FOR(k=0; k<3; k++)
    {
        hfband_end[k] = add(fLenLow, subband_offsets[k+1]);
        move16();
    }
    hfband_end[3] = add(fLenLow, fLenHigh);
    move16();

    tmp_fx = sub(sub(fLenLow, pos_max_hfe2), 1);
    rem_hfe = div_s_ss(tmp_fx, har_freq_est2);

    st_last_peakpos = add(pos_max_hfe2, i_mult(rem_hfe, har_freq_est2));

    temp_last_peakpos = st_last_peakpos;
    move16();
    i = 0;
    move16();

    FOR(k=0; k<2; k++)
    {
        nlags[k] = shl(1, bits_lagIndices_mode0_Har_fx[k]);

        l = 0;
        move16();
        WHILE( sub(st_last_peakpos, add(fLenLow,subband_offsets[k])) < 0)
        {
            st_last_peakpos = add(st_last_peakpos, har_freq_est2);
        }
        st_last_peakpos = sub(st_last_peakpos, har_freq_est2);

        IF( k==0 )
        {
            st_pos = add(sub(subband_search_offset[k], shr(nlags[k], 1)), lagIndices[k]);

            /*Copy the LF Smoothed Noise to the HF*/
            FOR(j =0; j<sbWidth[k]; j++)
            {
                L_xSynth_har[j] = L_shl(L_deposit_l(noise_flr_fx[st_pos+j]), sub(QsL, Qss));

                L_tmpbuf[j] = L_xSynth_har[j];
                move32();
                IF( predBuf_fx[st_pos+j] != 0x0)
                {
                    hf_pulse_peaks_fx[l] = predBuf_fx[st_pos+j];
                    move16(); /* Qss */
                    l = add(l, 1);
                }
            }
        }
        ELSE
        {
            st_pos = sub(add(subband_search_offset[k], shr(nlags[k], 1)), lagIndices[k]);
            dst_pos = sub(st_pos, sbWidth[k]);
            ii = sbWidth[k-1];
            move16();
            /*Copy the LF Smoothed Noise floor to the HF*/
            FOR(j=st_pos; j>(dst_pos); j--)
            {
                IF (sub(ii, add(sbWidth[k],sbWidth[k-1])) >= 0)
                {
                    BREAK;
                }

                /*xSynth_har[ii] = noise_flr[j];*/
                L_xSynth_har[ii] = L_shl(L_deposit_l(noise_flr_fx[j]), sub(QsL, Qss));
                L_tmpbuf[ii] = L_xSynth_har[ii];
                move32();
                IF( predBuf_fx[j] != 0x0 )
                {
                    hf_pulse_peaks_fx[l] = predBuf_fx[j];
                    move16();
                    l = add(l, 1);
                }
                ii = add(ii, 1);
            }
        }
        pos = 0;
        move16();
        FOR(j = 0; j< l; j++)
        {
            st_last_peakpos = add(st_last_peakpos, har_freq_est2);
            IF( sub(st_last_peakpos, hfband_end[k]) <  0 )
            {
                pk_sf_fx[k*8+pos].nmrValue_fx = hf_pulse_peaks_fx[j];
                move16(); /* Qss */
                pk_sf_fx[k*8+pos].gainIndex_fx = sub(st_last_peakpos, fLenLow);
                move16();
                pul_res[k] = add(pul_res[k], 1);
                move16();
                pulse_peak_sb_fx[i] = hf_pulse_peaks_fx[j];
                move16(); /* Qss */
                i = add(i, 1);
                pos = add(pos, 1);
            }
        }
        st_last_peakpos = temp_last_peakpos;
        move16();
    }
    res = sub(i, 1);
    l = 1;
    move16();
    ii = sub(sub(hfband_end[k-1], fLenLow), 1);
    tmp_fx = sub(bands, harmonic_band);
    FOR(; k<tmp_fx; k++)
    {
        Word16 tmp2;

        tmp2 = (sub(hfband_end[k], fLenLow));
        FOR( j=sub(hfband_end[k-1], fLenLow); j<tmp2; j++ )
        {
            L_xSynth_har[j] = L_tmpbuf[ii];
            move32();
            L_tmpbuf[j] = L_xSynth_har[j];
            move32();
            ii = sub(ii, 1);
        }
        pos = 0;
        move16();
        WHILE( sub(st_last_peakpos, hfband_end[k-1]) < 0 )
        {
            st_last_peakpos = add(st_last_peakpos, har_freq_est2);
        }
        WHILE( sub(st_last_peakpos, hfband_end[k]) < 0 && sub(pul_res[k], pul_res[2-l]) < 0 && sub(l, 2) <= 0 )
        {
            test();
            test();
            pk_sf_fx[k*8+pos].nmrValue_fx = pulse_peak_sb_fx[res];
            move16(); /* Qss */
            pk_sf_fx[k*8+pos].gainIndex_fx = sub(st_last_peakpos, fLenLow);
            move16();
            pul_res[k] = add(pul_res[k], 1);
            move16();
            res = sub(res, 1);
            pos = add(pos, 1);
            st_last_peakpos = add(st_last_peakpos, har_freq_est2);
        }
        l = add(l ,1);
    }

    return;
}

/*-------------------------------------------------------------------*
 * SmoothSpec()
 *
 * Smoothes specified samples using moving average method. The number
 * of points in the average is given by 'span'. Note that current
 * implementation does not accept 'span' to be smaller than 'fLen'.
 *-------------------------------------------------------------------*/
static
void SmoothSpec_fx(
    Word16 *inBuf,      /* (i) : Input spectrum       Q8 */
    Word16 *outBuf,     /* (o) : Smoothed spectrum    Q8 */
    Word16 num_subband  /* (i) : subband number          */
)
{
    Word16 i, tmp;
    Word16 span1;           /* */
    Word16 nItems, inItems; /* inverse */
    Word32 L_sum;             /* */
    Word16 *oldPtr, *newPtr;/* */
    Word16    hi, lo;
    /* ========  Q8  ======== */
    span1 = shr(MA_LEN, 1);

    /*-- First sample. --*/
    L_sum = L_deposit_l(*inBuf);
    *outBuf++ = *inBuf;
    move16();

    oldPtr = inBuf;
    newPtr = inBuf + 2;

    /*-- Handle start. --*/
    inBuf++;
    L_sum = L_mac0(L_sum, 0x0001, *inBuf);

    /* nItems = 3 --> inItems = 1/3 = 0.33f, 85(Q8) */
    /* 1/3 = 0.3333f -> 0x2AAA Q15 */
    inItems = 0x2AAA;
    move16();
    FOR(i = 1; i < span1; i++)
    {
        L_sum = L_mac0(L_sum, 0x0001, *newPtr++);

        lo = L_Extract_lc(L_sum, &hi);
        *outBuf++ = round_fx( L_shl(Mpy_32_16(hi, lo, inItems), 16) ); /* Q(8+15+1-16)=Q8 -> Q(8+16-16)=Q8 */

        L_sum = L_mac0(L_sum, 0x0001, *newPtr++);
        /* nItems += 2,
         * only used value is 5 -->
         * inItems = 1/5 = 0.2f, 51(Q8)
         */
        /* 1/5 = 0.2f -> 0x1999 Q15 */
        inItems = 0x1999;
        move16();
        inBuf++;
    }

    inBuf++;
    L_sum = L_mac0(L_sum, 0x0001, *newPtr++);

    lo = L_Extract_lc(L_sum, &hi);
    /* 4681 (in Q15) = 0.1428 = 1/7 */
    *outBuf++ = round_fx(L_shl(Mpy_32_16(hi, lo, 4681), 16)); /* Q(8+15+1-16)=Q8 -> Q(8+16-16)=Q8 */
    i = add(i, 1);

    /*-- Moving average. --*/
    tmp = sub(num_subband, span1);
    FOR( ; i < tmp; i++)
    {
        L_sum = L_mac0(L_sum, 0x0001, *newPtr++);
        L_sum = L_msu0(L_sum, 0x0001, *oldPtr++);

        lo = L_Extract_lc(L_sum, &hi);
        /* 4681 (in Q15) = 0.1428 = 1/7 */
        *outBuf++ = round_fx(L_shl(Mpy_32_16(hi, lo, 4681), 16)); /* Q(8+15+1-16)=Q8 -> Q(8+16-16)=Q8 */
        inBuf++;
    }

    /*-- Handle end. --*/
    /* nItems = span - 2; (nItems = 5, so we can maintain inItems = 1/5 = 0.2f from above) */
    nItems = sub(MA_LEN, 2);
    L_sum = L_msu0(L_sum, 0x0001, *oldPtr++);

    tmp = sub(num_subband, 1);
    FOR( ; i < tmp; i++)
    {
        L_sum = L_msu0(L_sum, 0x0001, *oldPtr++);

        lo = L_Extract_lc(L_sum, &hi);
        *outBuf++ = round_fx( L_shl(Mpy_32_16(hi, lo, inItems), 16)); /* Q(8+15+1-16)=Q8 -> Q(8+16-16)=Q8 */

        /* nItems -= 2; */
        nItems = sub(nItems, 2);

        /* 1.0f -> 0x7fff Q15 */
        inItems = 0x7fff;
        move16();
        if(sub(nItems, 3) == 0)
        {
            /* 1/3 = 0.333f -> 0x2AAA Q15 */
            inItems = 0x2AAA;
            move16();
        }
        L_sum = L_msu0(L_sum, 0x0001, *oldPtr++);

        inBuf++;
    }

    /*-- Last sample. --*/
    *outBuf = *inBuf;
    move16();
}

/*-------------------------------------------------------------------*
 * SpectrumSmoothing()
 *
 * Smoothing of the low-frequency envelope
 *-------------------------------------------------------------------*/

void SpectrumSmoothing_fx(
    const Word32 *L_inBuf,                   /* i  : Qs  Low band MDCT             */
    Word16 *outBuf_fx,                 /* o  : Qss output                    */
    Word16 *Qss,                       /* o  : Q0  Q value of output vector  */
    const Word16 fLen,                       /* i  : Q0  length                    */
    const Word16 th_cut_fx                   /* i  : Qss threshold of cut          */
)
{
    /* internal variable */
    Word16 i,j,k;

    Word16 num_subband_smooth_fx;
    Word16 num_subband_smooth_pre_fx;

    Word16 exp_normd;
    Word16 exp_shift;

    Word16 max_val_norm_fx;

    Word16 Qmax_val_norm[L_FRAME32k/L_SB];

    Word32 L_inBuf_abs;
    Word32 L_inBuf_pss[L_FRAME32k];
    Word32 L_max_val[L_FRAME32k/L_SB];
    Word16 outBuf_pss_fx[L_FRAME32k];
    Word32 L_outBuf_pss[L_FRAME32k];

    Word16 m, n;
    Word16 cnt_zero_cont;
    Word16 n_list[BANDS_MAX];
    Word16 reset_flag;
    Word16 pp, pk;
    Word16 exp_norm;

    *Qss = 10;

    num_subband_smooth_pre_fx = mult(fLen, 21845);  /* 1/L_SB = 1/12 = 21845(Q18)  Q = exp_normn-18 */
    num_subband_smooth_fx = shr(num_subband_smooth_pre_fx, 18-15);
    IF( sub(num_subband_smooth_pre_fx, shl(num_subband_smooth_fx, 18-15)) != 0 )
    {
        num_subband_smooth_fx++;
    }

    FOR( i=0; i<fLen; i++ )
    {
        L_inBuf_pss[i] = L_inBuf[i];
        move32();
        outBuf_pss_fx[i] = 0;
        move16();
        L_outBuf_pss[i] = L_deposit_l(0);
    }

    FOR( i=fLen; i<fLen + (num_subband_smooth_fx * L_SB - fLen); i++ )
    {
        L_inBuf_pss[i] = L_deposit_l(0);
        outBuf_pss_fx[i] = 0;
        move16();
        L_outBuf_pss[i] = L_deposit_l(0);
    }

    j = 0;
    FOR ( i=0; i<num_subband_smooth_fx; i++ )
    {
        L_max_val[i] = L_deposit_l(0);
        FOR( k=0; k<L_SB; k++ )
        {
            L_inBuf_abs = L_abs(L_inBuf_pss[j]);
            if( L_sub(L_max_val[i], L_inBuf_abs) < 0 )
            {
                L_max_val[i] = L_inBuf_abs;
                move32();
            }

            j++;
        }
    }

    /* convert to maximum amplitude frequency log scale envelope */
    j = 0;
    FOR ( i=0; i<num_subband_smooth_fx; i++ )
    {
        /* max_val_norm = 10.0f / (max_val[i] + 0.001f); */
        /* 10.0f : 0x2800, Q10 */
        IF(L_sub(L_max_val[i], 0x1L) > 0)
        {
            exp_normd = norm_l(L_max_val[i]);
            max_val_norm_fx = div_s(0x2800, round_fx(L_shl(L_max_val[i], exp_normd))); /* Q10-(Qs+exp_normd-16) */
            Qmax_val_norm[i] =  sub(10-12+16+15, exp_normd);
            move16(); /* 10 - (12+exp_normd-16) +15 */;
        }
        ELSE
        {
            max_val_norm_fx = 0;
            move16();
            Qmax_val_norm[i] = 0;
            move16();
        }

        exp_shift = sub(*Qss, add(Qmax_val_norm[i], -19));
        FOR( k = 0; k < L_SB; k++ )
        {
            exp_norm = norm_l(L_inBuf_pss[j]);
            L_outBuf_pss[j] = Mult_32_16(L_shl(L_inBuf_pss[j], exp_norm), max_val_norm_fx);
            move32();
            outBuf_pss_fx[j] = round_fx( L_shl(L_outBuf_pss[j], sub(exp_shift, exp_norm)) );
            j++;
        }
    }

    k = 0;
    move16();
    m = 0;
    move16();
    n = 0;
    move16();
    reset_flag = 0;
    move16();
    n_list[0] = 0;
    move16();
    FOR( j=0; j<num_subband_smooth_fx; j++ )
    {
        cnt_zero_cont = 0;
        move16();
        FOR( i=0; i<L_SB; i++ )
        {
            cnt_zero_cont = add(cnt_zero_cont, 1);
            if( outBuf_pss_fx[k] != 0 )
            {
                cnt_zero_cont = 0;
                move16();
            }
            k = add(k, 1);
        }

        IF( cnt_zero_cont != 0 )
        {
            test();
            IF( sub(j, div_s_ss(subband_search_offsets_fx[0], L_SB)) > 0 && reset_flag == 0 )
            {
                n = 0;
                move16();
                reset_flag = 1;
                move16();
            }
            n_list[n] = j;
            move16();
            n = add(n, 1);
        }

        test();
        if( sub(reset_flag, 1) == 0 && sub(n, 1) == 0 )
        {
            m = 0;
            move16();
        }

        pk = sub(k, L_SB);
        IF( sub(cnt_zero_cont, mult_r(L_SB, 24576)) > 0 )  /* cnt_zero_cont > 3*L_SB/4 */
        {
            pp = round_fx(L_shl(L_mult(n_list[m], L_SB), 15));
            FOR( i=0; i<L_SB; i++)
            {
                if( outBuf_pss_fx[pk+i] == 0 )
                {
                    outBuf_pss_fx[pk+i] = shr(outBuf_pss_fx[pp+i], 1);
                    move16();
                }
            }
            m = add(m, 1);
        }
    }

    FOR( i=0; i<fLen; i++ )
    {
        outBuf_fx[i] = 0x0;
        move16();
        if( sub(abs_s(outBuf_pss_fx[i]), th_cut_fx) > 0 )
        {
            outBuf_fx[i] = outBuf_pss_fx[i];
            move16();
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * Get20Log10Spec()
 *
 * Calculates 20*log10() for the specified samples. Input and output buffers can be the same.
 *-------------------------------------------------------------------*/

void Get20Log10Spec_fx(
    const Word32 *L_inBuf,     /* i  : input         Q_inBuf   */ /* L_inBuf >=0, so L_abs is omitted. */
    Word16 *outBuf_fx,   /* o  : output        Q7        */
    const Word16 fLen,         /* i  : loop length             */
    const Word16 Q_inBuf       /* i  : Qvalue of L_inBuf       */
)
{
    Word16 i;
    Word16 exp, frac;

    Word32 L_tmp;

    Word32 L_lamda;
    Word16 Q_inBuf_1;

    Q_inBuf_1 = sub(Q_inBuf, 1);
    L_lamda = L_shl(1L, Q_inBuf_1);  /* +1 : Q_inBuf -> Q_inBuf-1 for overflow problem */

    FOR( i = 0; i < fLen; i++)
    {
        /*outBuf++ = (float) (20.0f * log10(fabs(*inBuf + 1.0))); */
        L_tmp = L_add(L_shr(*L_inBuf++, 1), L_lamda);
        exp = 31;
        move16();
        if (L_tmp != 0x0L)
        {
            exp = norm_l(L_tmp);
        }
        frac = Log2_norm_lc(L_shl(L_tmp, exp));
        exp = sub(30, exp);
        exp = sub(exp, Q_inBuf_1);
        L_tmp = L_Comp(exp, frac);

        L_tmp = Mpy_32_16_1(L_tmp, 24660);   /* 6.0206 in Q12 */
        L_tmp = L_shl(L_tmp, 2+8);     /* Q7 */
        *outBuf_fx++ = round_fx(L_tmp);
    }

    return;
}

void convert_lagIndices_pls2smp_fx(
    Word16 lagIndices_in_fx[],
    Word16 nBands_search_fx,
    Word16 lagIndices_out_fx[],
    const Word16 sspectra_fx[],
    const Word16 sbWidth_fx[],
    const Word16 fLenLow_fx
)
{
    Word16 sb;
    Word16 i, cnt;

    FOR( sb = 0; sb < nBands_search_fx; sb++ )
    {
        cnt = 0;
        move16();
        i   = 0;
        move16();

        WHILE( sub(cnt, lagIndices_in_fx[sb]) <= 0 )
        {
            if( sspectra_fx[subband_search_offsets_fx[sb]+i] != 0 )
            {
                cnt = add(cnt, 1);
            }

            i = add(i, 1);

            IF( sub(add(subband_search_offsets_fx[sb], add(i, sbWidth_fx[sb])) , fLenLow_fx) >= 0)
            {
                BREAK;
            }
        }

        lagIndices_out_fx[sb] = add(sub(i, 1), subband_search_offsets_fx[sb]);
        move16();
    }

    return;
}

Word16 get_usebit_npswb_fx(
    Word16 hqswb_clas_fx
)
{
    Word16 i;
    Word16 bits;
    Word16 up_lmt;
    const Word16 *bits_req;

    up_lmt   = 0;
    move16();
    bits_req = bits_lagIndices_fx;
    move16();
    bits = 0;
    move16();

    IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
    {
        up_lmt = NB_SWB_SUBBANDS;
        move16();
        bits_req = bits_lagIndices_fx;
        move16();
    }
    ELSE IF ( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        up_lmt = NB_SWB_SUBBANDS_HAR_SEARCH_SB;
        move16();
        bits_req =bits_lagIndices_mode0_Har_fx;
        move16();
        bits = 2;
        move16(); /*noise gain*/
    }

    FOR( i = 0; i < up_lmt; i++ )
    {
        bits = add(bits, bits_req[i]);
        move16();
    }

    return bits;
}

void SpectrumSmoothing_nss_fx(
    const Word32 *L_inBuf,                   /* i  : lowband MDCT              */
    Word16 *outBuf_fx,                 /* o  : output                    */
    Word16 *Qss,                       /* o  : Q value of output vector  */
    const Word16 fLen                        /* i  : length                    */
)
{
    /* internal variable */
    Word16 i,k;

    Word16 inBuf_fx[L_FRAME32k];
    Word16 Qm;
    Word32 L_tmp[L_FRAME32k];

    Word16 num_subband_smooth_fx;
    Word16 exp_tmp;

    Word16 inBufw_fx[L_FRAME32k+L_SB_NSS];
    Word16 outBufw_fx[L_FRAME32k+L_SB_NSS];
    Word32 L_outBufw[L_FRAME32k+L_SB_NSS];
    Word16 Qo[NUM_SUBBAND_SMOOTH_MAX];

    Word16 avg_val_fx;
    Word32 L_avg_val;
    Word16 r0_fx;
    Word32 L_r0;
    Word32 L_temp;
    Word16 temp_fx;

    Word16 max_peak_fx;

    Word16 smr_fx;
    Word32 L_smr;

    Word32 L_temp_sum_1[NUM_SUBBAND_SMOOTH_MAX];
    Word32 L_temp_sum_2[NUM_SUBBAND_SMOOTH_MAX];
    Word32 L_temp_sum_3[NUM_SUBBAND_SMOOTH_MAX];


    Word16 temp_sum_smooth_fx[NUM_SUBBAND_SMOOTH_MAX];
    Word16 temp_sum_div_fx[NUM_SUBBAND_SMOOTH_MAX];
    Word16 Qsumdiv[NUM_SUBBAND_SMOOTH_MAX];
    Word32 L_temp1;
    Word16 temp_hi;
    Word16 temp_lo;


    Word16 avg_val2_fx;
    Word32 L_avg_val2;
    Word16 Qavg_val;
    Word16 Qsmr;
    Word16 exp, frac;

    Word16 clip_cof_fx;

    Word16 thre_fx, thre_fx_neg;
    Word16 thre_min_fx;

    Word16 temp_sum_log_fx[NUM_SUBBAND_SMOOTH_MAX];
    Word16 exp_norm;
    Word16 exp_normn;
    Word16 exp_normd;
    Word16 exp_shift;

    L_tmp[0] = L_deposit_l(0);
    FOR(i=0; i<fLen; i++)
    {
        L_tmp[0] = L_or(L_tmp[0], L_abs(L_inBuf[i]));
    }
    exp_norm = norm_l(L_tmp[0]);
    Qm = sub(exp_norm, 4);                       /* Qm = sub(add(12, exp_norm), 16); */

    FOR(i=0; i<fLen; i++)
    {
        L_tmp[i] = L_shl(L_inBuf[i], exp_norm); /* Q(12+exp_norm) */ move32();
        inBuf_fx[i] = round_fx(L_tmp[i]);      /* Qm */
    }

    num_subband_smooth_fx = shr(fLen, 3); /* L_SB_NSS=8 shr(target, 3); */

    /* buffer copy for fractional point */
    FOR( i = 0; i < fLen; i++ )
    {
        inBufw_fx[i] = inBuf_fx[i];
        move16();
        outBufw_fx[i] = 0;
        move16();
    }

    /* initialization of over buffer for fractional point */
    k = add(fLen, L_SB_NSS);
    FOR( i = fLen; i < k; i++ )
    {
        inBufw_fx[i] = 0;
        move16();
        outBufw_fx[i] = 0;
        move16();
    }

    L_avg_val = L_deposit_l(0);
    FOR( i = 0; i < fLen; i++ )
    {
        L_r0 = L_abs(L_deposit_l(inBufw_fx[i]));
        L_avg_val = L_add(L_avg_val, L_r0);
    }
    exp_normn = norm_l(L_avg_val);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(fLen);
    avg_val_fx = div_l(L_shl(L_avg_val, exp_normn), shl(fLen, exp_normd)); /* (Qs+exp_norm+exp_normn) - (exp_normd) - 15 */
    Qavg_val = sub(add(Qm, sub(exp_normn, exp_normd)), 1);

    max_peak_fx = 0;
    move16();
    FOR( i = 0; i < fLen; i++ )
    {
        r0_fx = abs_s(inBufw_fx[i]);
        if( sub(max_peak_fx, r0_fx) < 0 )
        {
            max_peak_fx = r0_fx;
            move16();  /* Qm */
        }
    }

    /*smr = 10.0f * (float)log10( max_peak/(avg_val + 1.0e-20) + 1.0e-20 ); */
    exp_normn = norm_s(max_peak_fx);
    exp_normn = sub(exp_normn, 1);

    avg_val_fx = s_max(avg_val_fx, 0x1);
    exp_normd  = norm_s(avg_val_fx);

    smr_fx = div_s( shl(max_peak_fx, exp_normn), shl( avg_val_fx, exp_normd)); /* Q(exp_normn-exp_normd+15) */
    exp_tmp = sub(exp_normn, exp_normd);
    L_smr = L_deposit_h(smr_fx); /* Q+16 -> Q(exp_normn-exp_normd+15+16) */
    Qsmr=add(sub(add(Qm,exp_tmp), Qavg_val), 31);

    L_temp = L_add(L_shr(L_smr, 1), 0x1L); /* add minimum value */
    exp = norm_l(L_temp);
    frac = Log2_norm_lc(L_shl(L_temp, exp));
    exp = sub(30, exp);
    exp = sub(exp, sub(Qsmr, 1));
    L_temp = L_Comp(exp, frac);

    L_temp = Mpy_32_16_1(L_temp, 12330);   /* 3.0103 in Q12 */
    L_temp = L_shl(L_temp, 2+8);     /* Q7 */
    smr_fx = round_fx(L_temp);

    FOR( i = 0; i < num_subband_smooth_fx; i++ )
    {
        L_temp_sum_1[i] = L_deposit_l(0);
        L_temp_sum_2[i] = L_deposit_l(0);

        FOR( k = 0; k < L_SB_NSS_HALF; k++ )
        {
            L_temp_sum_1[i] = L_add( L_temp_sum_1[i], extract_l(abs_s( inBufw_fx[k+L_SB_NSS*i])) ); /* Qm */ move32();
        }

        FOR( k = L_SB_NSS_HALF; k < L_SB_NSS; k++ )
        {
            L_temp_sum_2[i] = L_add( L_temp_sum_2[i], extract_l(abs_s( inBufw_fx[k+L_SB_NSS*i])) ); /* Qm */ move32();
        }

        L_temp_sum_1[i] = L_shr(L_temp_sum_1[i], 2); /* *0.25   guarantee low-side 16bit for L_temp_sum_* */ move32();
        L_temp_sum_2[i] = L_shr(L_temp_sum_2[i], 2); /* *0.25 */  move32();
        L_temp_sum_3[i] = L_mult(extract_l(L_temp_sum_1[i]), extract_l(L_temp_sum_2[i])); /* Qm*2+1 */  move32();

        IF( L_temp_sum_3[i] == 0 )
        {
            L_temp_sum_3[i] = L_shl(L_add(L_temp_sum_1[i], L_temp_sum_2[i]), add(Qm, 1)); /*Q(Qm+Qm+1) */ move32();
        }
    }

    exp_norm = add(shl(Qm, 1), 1);
    Get20Log10Spec_fx(L_temp_sum_3, temp_sum_log_fx, num_subband_smooth_fx, exp_norm);

    /* temp_sum_log_fx // *0.5  Q7 -> Q8 (not change) */
    SmoothSpec_fx(temp_sum_log_fx, temp_sum_smooth_fx, num_subband_smooth_fx);

    FOR (i = 0; i < num_subband_smooth_fx; i++)
    {
        L_temp1 = L_mult(temp_sum_smooth_fx[i], 1360); /* Q8+Q13+1=Q22, 1360(Q13) = 0.1660 = 3.321928(log2^10) * 0.05 */
        L_temp1 = L_shr(L_temp1, 6);                   /* Q22 -> Q16 */
        L_temp1 = L_negate(L_temp1);
        temp_lo = L_Extract_lc(L_temp1, &temp_hi);
        Qsumdiv[i] = sub(14, temp_hi);
        temp_sum_div_fx[i] = extract_l(Pow2(14, temp_lo)); /* Qsumdiv[i] */
        exp_norm = norm_s(temp_sum_div_fx[i]);
        temp_sum_div_fx[i] = shl(temp_sum_div_fx[i], exp_norm);
        move16();
        Qsumdiv[i] = add(Qsumdiv[i], exp_norm);
        move16();
    }

    *Qss = 31;
    move16();
    FOR (i = 0; i < num_subband_smooth_fx; i++)
    {
        Qo[i] = add(add(Qm, Qsumdiv[i]), 1);
        L_temp1 = 0x0L;
        FOR (k = 0; k < L_SB_NSS; k++)
        {
            L_outBufw[k + L_SB_NSS * i] = L_mult(inBufw_fx[k + L_SB_NSS * i], temp_sum_div_fx[i]);
            move32();
            L_temp1 = L_or(L_temp1, L_abs(L_outBufw[k + L_SB_NSS * i]));
        }
        exp_norm = 31;
        if (L_temp1 != 0x0L )
        {
            exp_norm = norm_l(L_temp1);
        }
        FOR (k = 0; k < L_SB_NSS; k++)
        {
            L_outBufw[k + L_SB_NSS * i] = L_shl(L_outBufw[k + L_SB_NSS * i], exp_norm);
            move32();
        }
        Qo[i] = add(Qo[i], exp_norm);
        move16();
        *Qss = s_min(*Qss, Qo[i]);
        move16();
    }

    FOR (i = 0; i < num_subband_smooth_fx; i++)
    {
        exp_shift = sub(*Qss, Qo[i]);
        FOR (k = 0; k < L_SB_NSS; k++)
        {
            L_outBufw[k + L_SB_NSS * i] = L_shl(L_outBufw[k + L_SB_NSS * i], exp_shift);
            move16();
            outBufw_fx[k + L_SB_NSS * i] = round_fx(L_outBufw[k + L_SB_NSS * i]);
        }
    }
    *Qss = sub(*Qss, 16);

    L_avg_val2 = L_deposit_l(0);
    FOR( i = 0; i < fLen; i++ )
    {
        L_r0 = L_abs(L_deposit_l(outBufw_fx[i]));
        L_avg_val2 = L_add(L_avg_val2, L_r0);     /* Qss */
    }

    exp_normn = norm_l(L_avg_val2);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(fLen);
    temp_fx = div_l(L_shl(L_avg_val2, exp_normn), shl(fLen, exp_normd)); /* Q(obw+exp_normn - exp_normd) - 1 */
    avg_val2_fx = shr(temp_fx, sub(sub(exp_normn, exp_normd), 1)); /* Qss */

    /*clip_cof = smr - 16.0f; */
    clip_cof_fx = sub(smr_fx, 2048);  /* 2048: 16.0f (Q7) */
    if( clip_cof_fx < 0 )
    {
        clip_cof_fx = 0;
        move16();
    }
    /*clip_cof += 2.5f; */
    clip_cof_fx = add(clip_cof_fx, 320); /* 320: 2.5f (Q7) */

    thre_fx = round_fx(L_shl(L_mult(avg_val2_fx, clip_cof_fx), 8)); /* Q(Qss+7+1) -> Qss */
    thre_fx_neg = negate(thre_fx);
    thre_min_fx = shr(avg_val2_fx, 2); /* *0.25f  // Qss */

    FOR(i = 0; i < fLen; i++)
    {
        IF( sub(abs_s(outBufw_fx[i]), thre_fx) > 0 )
        {
            temp_fx = thre_fx;
            move16();
            if(outBufw_fx[i] < 0)
            {
                temp_fx = thre_fx_neg;
                move16();
            }
            outBufw_fx[i] = temp_fx;
            move16();
        }

        if( sub(abs_s(outBufw_fx[i]), thre_min_fx) < 0 )
        {
            outBufw_fx[i] = 0;
            move16();
        }
    }

    FOR(i = 0; i < fLen; i++)
    {
        outBuf_fx[i] = outBufw_fx[i];
        move16(); /* Qss */
    }

    return;
}

/*-------------------------------------------------------------------*
 * return_bits_normal2
 *
 * arrange bit_budget when HQ_NORMAL
 *-------------------------------------------------------------------*/

void return_bits_normal2_fx(
    Word16 *bit_budget_fx,         /* i/o : bit budget                          */
    const Word16 p2a_flags_fx[],         /* i   : HF tonal indicator                  */
    const Word16 bands_fx,               /* i   : Total number of Subbands in a frame */
    const Word16 bits_lagIndices_fx[]    /* i   : bits for lagIndices                 */
)
{
    Word16 i;
    const Word16 *p_p2a_flags_fx;

    p_p2a_flags_fx = &p2a_flags_fx[sub(bands_fx, NB_SWB_SUBBANDS)];
    FOR( i=0 ; i < NB_SWB_SUBBANDS; i++ )
    {
        if( sub(*p_p2a_flags_fx++, 1) == 0 )
        {
            *bit_budget_fx = add(*bit_budget_fx, bits_lagIndices_fx[i]);
            move16();
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * preset_hq2_swb
 *
 * preset before swb_bwe_{enc,dec}_lr
 *-------------------------------------------------------------------*/

void preset_hq2_swb_fx
(
    const Word16 hqswb_clas_fx,     /* i   : HQ2 class information               */
    const Word16 band_end_fx[],     /* i   : band end of each SB                 */
    Word16 *har_bands_fx,     /* i/o : Number of LF harmonic bands         */
    Word16 p2a_bands_fx,      /* i   : flag for peakness                   */
    const Word16 length_fx,         /* i   : processed band length               */
    const Word16 bands_fx,          /* i   : Total number of Subbands in a frame */
    Word16 *lowlength_fx,     /* o   : lowband length                      */
    Word16 *highlength_fx,    /* o   : highband length                     */
    Word32 L_m[]              /* o   : MDCT                                */
)
{
    IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        *har_bands_fx = add(sub(bands_fx, p2a_bands_fx), 1);
        move16();
        *lowlength_fx = add(band_end_fx[*har_bands_fx-1], 1);
        move16();
    }
    ELSE
    {
        *lowlength_fx = add(band_end_fx[bands_fx-NB_SWB_SUBBANDS-1], 1);
        move16();
    }

    *highlength_fx = sub(length_fx, *lowlength_fx);
    move16();

    set32_fx( L_m, 0, length_fx );

    return;
}

/*-------------------------------------------------------------------*
 * preset_hq2_swb
 *
 * post process after swb_bwe_{enc,dec}_lr
 *-------------------------------------------------------------------*/

void post_hq2_swb_fx
(
    const Word32 L_m[],             /* i   : input_signal                        */
    const Word16 lowlength_fx,      /* i   : lowband length                      */
    const Word16 highlength_fx,     /* i   : highband length                     */
    const Word16 hqswb_clas_fx,     /* i   : HQ2 class information               */
    const Word16 har_bands_fx,      /* i   : Number of LF harmonic bands         */
    const Word16 bands_fx,          /* i   : Total number of Subbands in a frame */
    const Word16 p2a_flags_fx[],    /* i   : HF tonal indicator                  */
    const Word16 band_start_fx[],   /* i   : band start of each SB               */
    const Word16 band_end_fx[],     /* i   : band end of each SB                 */
    Word32 L_y2[],            /* o   : output signal                       */
    Word16 npulses_fx[]       /* i/o : Number of coded spectrum            */
)
{
    Word16  i, k;

    /* copy the scratch buffer to the output */
    Copy32( &L_m[lowlength_fx], &L_y2[lowlength_fx], highlength_fx );

    IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        k = har_bands_fx;
        move16();
    }
    ELSE
    {
        k = sub(bands_fx, NB_SWB_SUBBANDS);
        move16();
    }

    FOR( ; k<bands_fx; k++ )
    {
        test();
        IF( p2a_flags_fx[k] == 0 && npulses_fx[k] == 0 )
        {
            FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
            {
                if( L_y2[i] != 0 )
                {
                    npulses_fx[k] = add(npulses_fx[k], 1);
                    move16();
                }
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * GetSynthesizedSpecThinOut()
 *
 * Synthesize the spectrum in generic subband coding
 *--------------------------------------------------------------------------*/

void GetSynthesizedSpecThinOut_fx(
    const Word16 *predBuf_fx,        /* i  : Qss: prediction buffer (i.e., lowband)  */
    const Word16 Qss,                /* i  :      Q value of input vector            */
    Word32 *L_outBuf,          /* o  : QsL: synthesized spectrum               */
    Word16 QsL,                /* o  :      Q value of synthesized spectrum    */
    const Word16 nBands_fx,          /* i  : Q0: number of subbands calculated       */
    const Word16 *sbWidth_fx,        /* i  : Q0: subband lengths                     */
    const Word16 *lagIndices_fx,     /* i  : Q0: lowband index for each subband      */
    const Word16 *lagGains_fx,       /* i  : Qgain: lagGain for each subband         */
    const Word16 *QlagGains_fx,      /* i  : Q0: Q value of lagGains_fx              */
    const Word16 predBufLen_fx       /* i  : Q0: lowband length                      */
)
{
    Word16 i, sb;
    Word16 fLen_fx, lag_fx;

    const Word16 *ptr_predBuf_fx;
    Word32 *ptr_L_outBuf;
    Word32 *ptr_L_in_outBuf;

    Word16 exp_shift;

    ptr_L_in_outBuf = L_outBuf;
    ptr_L_outBuf = L_outBuf;

    FOR( sb = 0; sb < nBands_fx; sb++ )
    {
        fLen_fx = sbWidth_fx[sb];
        lag_fx = lagIndices_fx[sb];

        if( sub(add(lag_fx , fLen_fx) ,predBufLen_fx) > 0 )
        {
            /* should never happen */
            lag_fx = sub(predBufLen_fx, fLen_fx);
        }
        ptr_predBuf_fx = predBuf_fx + lag_fx;


        exp_shift = sub(add(add(Qss, QlagGains_fx[sb]), 1), QsL);

        FOR (i=0; i<fLen_fx; i++)
        {
            *ptr_L_outBuf++ = L_shr(L_mult(*ptr_predBuf_fx++, lagGains_fx[sb]), exp_shift);
            move32(); /* Qss+QlagGains+1 -> QsL */
        }
    }

    ptr_L_outBuf = ptr_L_in_outBuf;

    return;
}

/*--------------------------------------------------------------------------*
 * div_s_ss
 *
 * compute division with Word16 Q0.   ex.   10/2 -> 5
 *--------------------------------------------------------------------------*/

Word16 div_s_ss(     /* o: result of division (Word16 Q0) */
    const Word16 n,  /* i: numerator   (Word16 Q0         */
    const Word16 d   /* i: denominator (Word16 Q0)        */
)
{
    Word16 norm_n, norm_d;
    Word16 ns, ds;
    Word16 res;

    test();
    IF ( n == 0 || d == 0 )
    {
        return 0;
    }

    norm_n = norm_s(n);
    norm_n = sub(norm_n, 1);
    ns = shl(n, norm_n);

    norm_d = norm_s(d);
    ds = shl(d, norm_d);

    res = shr(div_s(ns, ds), add(sub(norm_n, norm_d), 15));

    return res;
}

void hf_parinitiz_fx(
    const Word32 L_total_brate,
    const Word16 hqswb_clas_fx,
    Word16 lowlength_fx,
    Word16 highlength_fx,
    Word16 wBands_fx[],
    const Word16 **subband_search_offset_fx,
    const Word16 **subband_offsets_fx,
    Word16 *nBands_fx,
    Word16 *nBands_search_fx,
    Word16 *swb_lowband_fx,
    Word16 *swb_highband_fx
)
{
    *swb_lowband_fx = lowlength_fx;
    move16();
    *swb_highband_fx = highlength_fx;
    move16();

    IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        /* Mode dependent initializations (performed every frame in case mode-switching implemented) */
        *nBands_fx = NB_SWB_SUBBANDS_HAR;
        move16();
        *nBands_search_fx = NB_SWB_SUBBANDS_HAR_SEARCH_SB;
        move16();

        IF ( L_sub(L_total_brate, HQ_13k20) == 0 )
        {
            wBands_fx[0] = SWB_SB_BW_LEN0_12KBPS_HAR;
            move16();
            wBands_fx[1] = SWB_SB_BW_LEN1_12KBPS_HAR;
            move16();
            wBands_fx[2] = SWB_SB_BW_LEN2_12KBPS_HAR;
            move16();
            wBands_fx[3] = SWB_SB_BW_LEN3_12KBPS_HAR;
            move16();
            *subband_offsets_fx = subband_offsets_sub5_13p2kbps_Har_fx;
            move16();
            *subband_search_offset_fx = subband_search_offsets_13p2kbps_Har_fx;
            move16();
        }
        ELSE
        {
            wBands_fx[0] = SWB_SB_BW_LEN0_16KBPS_HAR;
            move16();
            wBands_fx[1] = SWB_SB_BW_LEN1_16KBPS_HAR;
            move16();
            wBands_fx[2] = SWB_SB_BW_LEN2_16KBPS_HAR;
            move16();
            wBands_fx[3] = SWB_SB_BW_LEN3_16KBPS_HAR;
            move16();
            *subband_offsets_fx = subband_offsets_sub5_16p4kbps_Har_fx;
            move16();
            *subband_search_offset_fx = subband_search_offsets_16p4kbps_Har_fx;
            move16();
        }
    }
    ELSE
    {
        /* Mode-dependent initializations (performed every frame in case mode-switching implemented) */
        *nBands_fx = NB_SWB_SUBBANDS;
        move16();
        *nBands_search_fx = NB_SWB_SUBBANDS;
        move16();

        IF ( L_sub(L_total_brate, HQ_13k20) == 0 )
        {
            wBands_fx[0] = SWB_SB_LEN0_12KBPS;
            move16();
            wBands_fx[1] = SWB_SB_LEN1_12KBPS;
            move16();
            wBands_fx[2] = SWB_SB_LEN2_12KBPS;
            move16();
            wBands_fx[3] = SWB_SB_LEN3_12KBPS;
            move16();
            *subband_offsets_fx = subband_offsets_12KBPS;
            move16();
        }
        ELSE
        {
            wBands_fx[0] = SWB_SB_LEN0_16KBPS;
            move16();
            wBands_fx[1] = SWB_SB_LEN1_16KBPS;
            move16();
            wBands_fx[2] = SWB_SB_LEN2_16KBPS;
            move16();
            wBands_fx[3] = SWB_SB_LEN3_16KBPS;
            move16();
            *subband_offsets_fx = subband_offsets_16KBPS;
            move16();
        }
    }

    return;
}

void GetlagGains_fx(
    const Word16 *predBuf_fx,      /* i: Qss  Low freq. Smoothed Spectrum */
    const Word16 Qss,              /* i: Q0   Q value of predBuf          */
    const Word32 *L_band_energy,   /* i: Qbe  Band Energy                 */
    const Word16 Qbe,              /* i: Q0   Q value of band energy      */
    const Word16 nBands,           /* i: Q0   number of SWB subbands      */
    const Word16 *sbWidth,         /* i: Q0   width of SWB subbands       */
    const Word16 *lagIndices,      /* i: Q0   lagIndices                  */
    const Word16 predBufLen,       /* i: Q0   length of predBuf           */
    Word16 *lagGains_fx,     /* o: QlagGains lagGains               */
    Word16 *QlagGains        /* o: Q0   Q value of lagGains         */
)
{
    Word16 i;
    Word16 sb, fLen, lag;

    Word32 L_outBuf[L_FRAME32k];

    Word16 *ptr_ssBuf_fx;
    Word32 L_lagEnergy;
    Word16 Qene;

    Word16 temp_lo_fx, temp_hi_fx;
    Word16 pow_fx;
    Word16 Qpow;
    Word16 exp_normd, exp_normn;
    Word16 Qdiv;
    Word16 exp_norm;

    Word16 temp_fx;
    Word32 L_temp;

    Word16 ssBuf_fx[L_FRAME32k];
    Word16 exp_norm_ss;

    exp_norm_ss = 2;
    move16();
    FOR (i=0; i<predBufLen; i++)
    {
        ssBuf_fx[i] = shr(predBuf_fx[i], exp_norm_ss);
        move16(); /* Qss+exp_norm_ss */
    }

    FOR( sb = 0; sb < nBands; sb++ )
    {
        fLen = sbWidth[sb];
        move16();
        lag = lagIndices[sb];
        move16();

        IF( sub(add(lag, fLen),predBufLen) > 0 )
        {
            /* should never happen */
            lag = sub(predBufLen, fLen);
            move16();
        }

        GetPredictedSignal_fx( predBuf_fx, L_outBuf, lag, fLen, 0x7fff, 15);

        ptr_ssBuf_fx = ssBuf_fx + lag;
        L_lagEnergy = L_deposit_l(0);
        FOR( i = 0; i < fLen; i++)
        {
            L_lagEnergy = L_mac(L_lagEnergy, *ptr_ssBuf_fx, *ptr_ssBuf_fx); /* (Qss-exp_norm_ss)*2+1 */
            ptr_ssBuf_fx++;
        }
        Qene = add(shl(sub(Qss, exp_norm_ss), 1), 1);

        IF( L_lagEnergy != 0x0L )
        {
            /* lagGains[sb] = (float)sqrt( pow(2.0f, band_energy[sb]) / lagEnergy ); */
            /* Pow part  (pow(2.0f, band_energy) ) */
            L_temp = L_shr(L_band_energy[sb], sub(Qbe, 16)); /* Qbe -> Q16 */
            temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
            Qpow = sub(14, temp_hi_fx);
            pow_fx = extract_l(Pow2(14, temp_lo_fx));       /* Qpow */

            /* Div part  ( pow (2.0f, band_energy[i])/lagEenegy ) */
            exp_normn = norm_s(pow_fx);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_l(L_lagEnergy);
            temp_fx = div_s( shl( pow_fx, exp_normn), extract_h(L_shl(L_lagEnergy, exp_normd)));
            Qdiv = add(sub(add(Qpow, exp_normn) , add(Qene, exp_normd)), 31);

            exp_norm = norm_s(temp_fx);
            temp_fx = shl(temp_fx, exp_norm);
            Qdiv = add(Qdiv, exp_norm);

            /* Sqrt part sqrt(pow (2.0f, band_energy[i])/lagEnergy) */
            QlagGains[sb] = add(Qdiv, 16);
            IF ( s_and(Qdiv, 1) == 0 ) /* Qdiv % 2 == 0 */
            {
                L_temp = Sqrt_l(L_shr(L_deposit_h(temp_fx),1), &exp_norm);
                L_temp = L_shr(L_temp, exp_norm);
                QlagGains[sb] = sub(shr(QlagGains[sb], 1), 1);
                move16();
                lagGains_fx[sb] = round_fx(L_temp);
            }
            ELSE
            {
                L_temp = Sqrt_l(L_deposit_h(temp_fx), &exp_norm);
                L_temp = L_shr(L_temp, exp_norm);
                QlagGains[sb] = shr(QlagGains[sb], 1);
                move16();
                lagGains_fx[sb] = round_fx(L_temp);
            }
        }
        ELSE
        {
            /* lagGains[sb] = 0.0f; */
            lagGains_fx[sb] = 0;
            move16();
            QlagGains[sb] = 15;
            move16();
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * noise_extr_corcod()
 * Spectrum normalization for the core coder
 *--------------------------------------------------------------------------*/

void noise_extr_corcod_fx(
    Word32 L_spectra[],               /* i  : QsL core coder                                  */
    const Word32 L_spectra_ni[],            /* i  : QsL core coder with sparse filling              */
    Word16 sspectra_fx[],             /* o  : Qss Smoothed tonal information from core coder  */
    Word16 sspectra_diff_fx[],        /* o  : Qss non tonal infomration for gap filling       */
    Word16 sspectra_ni_fx[],          /* o  : Qss smoothed core coder                         */
    const Word16 fLenLow_fx,                /* i  : Q0  low frequency bands width                   */
    Word16 prev_hqswb_clas_fx,        /* i  : Q0  classification information                  */
    Word16 *prev_ni_ratio_fx,         /* i  : Q15 noise paraemeter                            */
    Word16 *Qss                       /* o  : Q0  Q value for sspectra_*_fx                   */
)
{
    Word16 i,pulse_num_fx;
    Word32 L_spectra_diff[L_FRAME32k]; /* QsL */
    Word16 Qss_s, Qss_d;
    Word16 ni_ratio_fx, ni_ratio_cur_fx, br_adj_fx; /* Q15 */
    Word16 tmp_fx;
    Word16 exp_normn, exp_normd, exp_shift;
    Word16 exp_norm;

    /*Spectrum Smoothing for tonal signals*/
    SpectrumSmoothing_nss_fx( L_spectra, sspectra_fx, &Qss_s, fLenLow_fx );
    Copy(sspectra_fx, sspectra_ni_fx, fLenLow_fx);
    tmp_fx = 0;
    FOR(i=0; i<fLenLow_fx; i++)
    {
        tmp_fx |= abs_s(sspectra_fx[i]);
        logic16();
    }
    exp_norm = norm_s(tmp_fx);
    FOR(i=0; i<fLenLow_fx; i++)
    {
        sspectra_fx[i] = shl(sspectra_fx[i], exp_norm);
        move16();
    }
    Qss_s = add(Qss_s, exp_norm);

    /*noise extraction*/
    FOR(i=0; i<fLenLow_fx; i++)
    {
        L_spectra_diff[i] = L_sub(L_spectra_ni[i], L_spectra[i]);
        move32();
    }
    SpectrumSmoothing_nss_fx( L_spectra_diff, sspectra_diff_fx, &Qss_d, fLenLow_fx );
    tmp_fx = 0;
    FOR(i=0; i<fLenLow_fx; i++)
    {
        tmp_fx |= abs_s(sspectra_diff_fx[i]);
        logic16();
    }
    exp_norm = norm_s(tmp_fx);
    FOR(i=0; i<fLenLow_fx; i++)
    {
        sspectra_diff_fx[i] = shl(sspectra_diff_fx[i], exp_norm);
        move16();
    }
    Qss_d = add(Qss_d, exp_norm);

    IF ( sub(Qss_s, Qss_d) < 0 )
    {
        *Qss = Qss_s;
        move16();
        exp_shift = sub(Qss_d, *Qss);
        FOR(i=0; i<fLenLow_fx; i++)
        {
            sspectra_diff_fx[i] = shr(sspectra_diff_fx[i], exp_shift);
            move16();
        }
    }
    ELSE
    {
        *Qss = Qss_d;
        move16();
        exp_shift = sub(Qss_s, *Qss);
        FOR(i=0; i<fLenLow_fx; i++)
        {
            sspectra_fx[i] = shr(sspectra_fx[i], exp_shift);
            move16();
            sspectra_ni_fx[i] = shr(sspectra_ni_fx[i], exp_shift);
            move16();
        }
    }

    /*Smoothing the noise components*/
    br_adj_fx = 29491;  /* br_adj = 0.9f;  Q15 */

    pulse_num_fx = 0;
    move16();
    FOR(i=0; i<fLenLow_fx; i++)
    {
        if(L_spectra[i] != 0x0L)
        {
            pulse_num_fx = add(pulse_num_fx, 1);
        }
    }

    ni_ratio_cur_fx = 0x0;
    move16();
    IF( pulse_num_fx != 0 )
    {
        /*ni_ratio_cur = (fLenLow-pulse_num)/(fLenLow+0.0f);*/
        tmp_fx = sub(fLenLow_fx, pulse_num_fx);
        exp_normn = norm_s(tmp_fx);
        exp_normn = sub(exp_normn, 1);
        exp_normd = norm_s(fLenLow_fx);
        ni_ratio_cur_fx = div_s(shl(tmp_fx, exp_normn), shl(fLenLow_fx, exp_normd)); /* exp_normn - exp_normd + 15 */
        ni_ratio_cur_fx = shl(ni_ratio_cur_fx, sub(exp_normn, exp_normd));  /* 15 - (exp_normn - exp-normd + 15) */

        /*ni_ratio_cur *= br_adj;*/
        ni_ratio_cur_fx = mult_r(ni_ratio_cur_fx, br_adj_fx);
    }

    IF( sub(prev_hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        IF( sub(ni_ratio_cur_fx, *prev_ni_ratio_fx) > 0 )
        {
            /* 0.8: 26214(Q15) 0.2: 6554(Q15) */
            ni_ratio_fx = mac_r(L_mult(ni_ratio_cur_fx, 26214), *prev_ni_ratio_fx, 6554);
        }
        ELSE
        {
            /* 0.6: 19661(Q15) 0.4: 13107(Q15) */
            ni_ratio_fx = mac_r(L_mult(ni_ratio_cur_fx, 19661), *prev_ni_ratio_fx, 13107);
        }
    }
    ELSE
    {
        /* 0.7: 22938(Q15) */
        ni_ratio_fx = mult_r(ni_ratio_cur_fx, 22938);
    }
    *prev_ni_ratio_fx = ni_ratio_fx;
    move16();

    FOR(i=0; i<fLenLow_fx; i++)
    {
        sspectra_diff_fx[i] = mult_r(sspectra_diff_fx[i], ni_ratio_fx);
        move16();
        sspectra_ni_fx[i] = add(sspectra_fx[i], sspectra_diff_fx[i]);
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * ton_ene_est()
 * band energies for missing bands in the core coder
 *--------------------------------------------------------------------------*/

void ton_ene_est_fx(
    Word32 L_xSynth_har[],         /* i  : QsL  buffer with non tonal compoents   */
    Word16 QsL,                    /* i  : Q0   Q value for xSynth_har            */
    Word32 L_be_tonal[],           /* o  : QbeL tonal energy of the missing bands */
    Word16 *QbeL,                  /* o  : Q0   Q value for be_tonal              */
    Word32 L_band_energy[],        /* i  : Qbe  subband energies                  */
    Word16 Qbe,                    /* i  : Q0   Q value for band_energy           */
    const Word16 band_start[],           /* i  : Q0   subband start indices             */
    const Word16 band_end[],             /* i  : Q0   subband end indices               */
    const Word16 band_width[],           /* i  : Q0   subband widths                    */
    const Word16 fLenLow,                /* i  : Q0   low frequency width               */
    const Word16 fLenHigh,               /* i  : Q0   High frequency width              */
    Word16 bands,                  /* i  : Q0   total subbands                    */
    Word16 har_bands,              /* i  : Q0   total number of harmonics bands   */
    Word16 ni_lvl_fx,              /* i  : Q11  noise enve for the hf bands       */
    GainItem_fx pk_sf_fx[],          /* i  :                                        */
    Word16 Qss,                    /* i  : Q0   Q value for GainItem_fx->nmrValue */
    Word16 *pul_res                /* i  : Q0   tonal resolution                  */
)
{
    Word16 i, j, k;
    Word16 exp_norm;
    Word16 Inv_band_width_fx[BANDS_MAX];
    Word16 QInvBW[BANDS_MAX];

    Word16 xSynth_har_fx[L_FRAME32k];
    Word16 QxSynth;  /* Q value for xSynth_har_fx */
    Word16 QxSynth_sft;  /* Q value for xSynth_har_fx */

    Word16 sb_ton_loc_fx[SWB_HAR_RAN1]; /* Q0 */
    Word16 sb_ton_fx[SWB_HAR_RAN1];     /* Qss */
    Word16 ni_gain_fx[NB_SWB_SUBBANDS];
    Word16 Qni_gain;
    Word16 avg_pe_fx[NB_SWB_SUBBANDS];
    Word16 Qavg_pe[NB_SWB_SUBBANDS];
    Word16 QsN; /* Q value for xSynth_har after multipy ni_lvl */
    Word16 exp_safe; /* overflow prevent shift */

    Word16 pos,count_pos_st,count_pos_end;
    Word16 pul_res_bnd[NB_SWB_SUBBANDS];
    Word16 peak_fx[NB_SWB_SUBBANDS]; /* Qss */

    Word32 L_E;
    Word16 QE;
    Word16 temp_lo_fx, temp_hi_fx;
    Word32 L_temp;
    Word16 exp_pow;
    Word32 L_band_energy_Linear[BANDS_MAX];

    Word16 exp_normd, exp_normn;

    Word16 E_r_fx;
    Word16 QE_r; /* Q value for E_r_fx */

    Word16 exp_shift;

    Word16 E_r_shift_fx;
    Word16 fac_fx;
    Word16 Qtemp;
    Word16 temp2_fx, Qtemp2;
    Word16 temp_fx;

    *QbeL = 3;
    move16();
    Qni_gain = 8;
    move16();

    FOR(k=0; k<bands; k++)
    {
        exp_norm = norm_s(band_width[k]);
        exp_norm = sub(exp_norm, 1);
        Inv_band_width_fx[k] = div_s(0x1fff, shl(band_width[k], exp_norm)); /* */
        QInvBW[k] = sub(28, exp_norm); /* 13-exp_norm+15 */ move16();
    }

    set16_fx(sb_ton_loc_fx, -1, SWB_HAR_RAN1);
    set16_fx(ni_gain_fx,0,NB_SWB_SUBBANDS);
    set16_fx(avg_pe_fx,0,NB_SWB_SUBBANDS);
    set16_fx(Qavg_pe,0,NB_SWB_SUBBANDS);
    set16_fx(sb_ton_fx,0,NB_SWB_SUBBANDS);
    set16_fx(peak_fx,0,NB_SWB_SUBBANDS);
    FOR( i=0; i<fLenHigh; i++)
    {
        L_xSynth_har[i] = Mult_32_16(L_xSynth_har[i], ni_lvl_fx); /* QsL(=12)+11-15=8 */ move32();
    }
    QsN = sub(add(QsL, 11), 15);

    exp_safe = 4; /*move16();*/
    norm_vec_32_16_scale_fx(L_xSynth_har, QsN, fLenHigh, xSynth_har_fx, &QxSynth, exp_safe);

    pos = 0;
    move16();
    i = sub(bands, har_bands);
    FOR(k=0; k<i; k++)
    {
        FOR(j=0; j<pul_res[k]; j++)
        {
            sb_ton_loc_fx[pos] = pk_sf_fx[k*8+j].gainIndex_fx;
            move16();
            sb_ton_fx[pos] = pk_sf_fx[k*8+j].nmrValue_fx;
            move16();
            pos = add(pos, 1);
        }
    }
    k = 0;
    move16();
    pos = 0;
    move16();
    DO
    {
        count_pos_st = pos;
        WHILE(sb_ton_loc_fx[pos] <=(band_end[k+har_bands]-fLenLow) && sb_ton_loc_fx[pos]>=0 )
        {
            pos = add(pos, 1);
        }
        count_pos_end = pos;
        move16();
        pul_res_bnd[k] = sub(count_pos_end, count_pos_st);
        move16();
        if(pul_res_bnd[k] > 0)
        {
            peak_fx[k] =  abs_s(sb_ton_fx[count_pos_st]);
            move16();
        }
        k = add(k, 1);
    } WHILE( sub(k, NB_SWB_SUBBANDS) < 0 );

    k = 0;
    move16();
    /*energy calculation for tonal components*/
    FOR(i=har_bands; i<bands; i++)
    {
        L_E = sum2_fx(&xSynth_har_fx[sub(band_start[i], fLenLow)], band_width[i]);
        QE = add(shl(QxSynth, 1),1); /* QxSynth*2+1 */

        /* E_r = (float) E/(float)pow(2.0f,band_energy[i]); -> E * pow(2.0f, -band_energy) */
        /* Pow Part */
        L_temp = L_shr(L_band_energy[i], sub(Qbe, 16));
        temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);

        exp_pow = sub(14, temp_hi_fx);
        L_band_energy_Linear[i] = Pow2(14, temp_lo_fx);
        move32();      /* Qexp_pow */
        L_band_energy_Linear[i] = L_shl(L_band_energy_Linear[i], sub(*QbeL, exp_pow));

        /* Div Part */
        E_r_fx = 0x7fff;
        move16();
        QE_r = 0;
        move16();
        IF(L_band_energy_Linear[i] != 0x0L)
        {
            exp_normd = norm_l(L_E);
            exp_normd = sub(exp_normd, 1);
            exp_normn = norm_l(L_band_energy_Linear[i]);

            E_r_fx = div_s( extract_h(L_shl(L_E, exp_normd)), extract_h(L_shl(L_band_energy_Linear[i], exp_normn)));
            /* QE_r = (QE-16) - (QbeL+exp_normn-16) + 15; */
            QE_r = add(sub(add(QE, exp_normd) , add(*QbeL, exp_normn)), 15);
        }

        L_E = L_shl(L_E, sub(*QbeL, QE));
        QE = *QbeL;

        /* 0.06=15729(Q18) */
        exp_shift = sub(18, QE_r);
        E_r_shift_fx = shl(E_r_fx, exp_shift);

        IF ( sub(E_r_shift_fx, 15729) < 0 ) /* E_r < 0.06  */
        {
            /* avg_pe[k] = (float) sqrt(pow(2.0f,band_energy[i])/band_width[i]); */
            /* Pre SQRT part */
            /*L_temp = Mpy_32_16_1(L_band_energy_Linear[i], Inv_band_width_fx[i]);*/ /* QbeL + QInvBW -15 */
            L_temp = Mult_32_16(L_band_energy_Linear[i], Inv_band_width_fx[i]); /* QbeL + QInvBW -15 */
            Qtemp = sub(add(*QbeL, QInvBW[i]), 15);

            sqrt_32n_16_fx(L_temp, Qtemp, &avg_pe_fx[k], &Qavg_pe[k]);

            fac_fx = 19661; /* 0.6(Q15) */
            IF(pul_res_bnd[k] != 0)
            {
                /* Div Part */
                L_temp = Mult_32_16(L_E, Inv_band_width_fx[i]); /* QE+exp_norm+QInvBW[i]+1 */
                Qtemp = sub(add(QE, QInvBW[i]), 15);

                /* SQRT Part */
                sqrt_32n_16_fx(L_temp, Qtemp, &temp2_fx, &Qtemp2);

                /* Div Part */
                exp_normd = norm_s(temp2_fx);
                exp_normd = sub(exp_normd, 1);
                exp_normn = norm_s(peak_fx[k]);

                fac_fx = div_s(shl(temp2_fx, exp_normd), shl(peak_fx[k], exp_normn));
                fac_fx = shl(fac_fx, sub(add(Qss, exp_normn), add(Qtemp2, exp_normd))); /* Qtemp2+exp_normd-(Qss+exp_normn)+15 -> 15*/

            }

            ni_gain_fx[k] = mult_r(avg_pe_fx[k], fac_fx); /* Qavg_pe[k] */ move16();

            L_temp = L_mult(ni_gain_fx[k], ni_gain_fx[k]);
            L_temp = Mult_32_16(L_temp, E_r_fx);

            /* 0.12f: 257698038 (Q31) */
            if( L_sub(L_shl(L_temp,sub(31, add(add(shl(Qavg_pe[k], 1), QE_r), 1-15))), 257698038) >= 0 )
            {
                ni_gain_fx[k] = mult_r(1638, ni_gain_fx[k]); /* 0.05 : 1638(Q15) */ move16();
            }
            Overflow = 0;
            move16();
            ni_gain_fx[k] = shl(ni_gain_fx[k], sub(Qni_gain, Qavg_pe[k]));
            ni_gain_fx[k] = s_max(ni_gain_fx[k], (short)(1.4*pow(2,Qni_gain)));  /* 1.4 -> 22938(Q14) */

            exp_shift = QsL-(QxSynth+Qni_gain+1); /* QsL - (QxSynth+Qni_gain+1) */
            FOR(j=band_start[i]; j<=band_end[i]; j++)
            {
                L_xSynth_har[j-fLenLow] = L_shl(L_mult(xSynth_har_fx[j-fLenLow], ni_gain_fx[k]), exp_shift); /* QsL - (QxSynth+Qni_gain+1) */ move32();
            }

            exp_safe = 4; /* move16(); */
            norm_vec_32_16_scale_fx(&L_xSynth_har[band_start[i]-fLenLow], QsL, band_width[i], &xSynth_har_fx[band_start[i]-fLenLow], &QxSynth_sft, exp_safe);

            L_E = sum2_fx(&xSynth_har_fx[sub(band_start[i], fLenLow)], band_width[i]);
            QE = add(shl(QxSynth_sft, 1),1);

            L_E = L_shl(L_E, sub(*QbeL, QE));
            QE = *QbeL;
        }
        ELSE
        {
            /* Q8 -> Q12 */
            FOR(j=band_start[i]; j<=band_end[i]; j++)
            {
                L_xSynth_har[j-fLenLow] = L_shl(L_xSynth_har[j-fLenLow], 4); /* Q8(12+11-15) -> Q12 */ move32();
            }
        }

        k = add(k, 1);

        L_be_tonal[i] = L_sub(L_band_energy_Linear[i], L_E);

        IF ( L_be_tonal[i] < 0x0L )
        {
            L_E = L_deposit_l(0);
            FOR(j=(band_start[i]-fLenLow); j<=(band_end[i]-fLenLow); j++)
            {
                temp_fx = round_fx(L_shl(L_xSynth_har[j], (*QbeL+7)/2));  /* (12+x-16)*2+1 => QbeL */
                L_xSynth_har[j] = L_shr(L_xSynth_har[j], 2); /* 1/4 */ move32();
                L_E = L_mac( L_E, temp_fx, temp_fx);
            }

            L_E = L_shr(L_E, 4); /* 1/4 */
            L_be_tonal[i] = L_sub(L_band_energy_Linear[i], L_E);
            move32();
        }
    }

    return;
}


/*--------------------------------------------------------------------------*
 * Gettonl_scalfact()
 * Gap filling for the core coder
 *--------------------------------------------------------------------------*/
void Gettonl_scalfact_fx
(
    Word32 *L_outBuf,                /* i/o: QsL synthesized spectrum                        */
    Word16 QsL,                      /* i  : Q0  Q value for outBuf                          */
    const Word32 *L_codbuf,                /* i  : QsL core coder                                  */
    const Word16 fLenLow,                  /* i  : Q0  lowband length                              */
    const Word16 fLenHigh,                 /* i  : Q0  highband length                             */
    const Word16 harmonic_band,            /* i  : Q0  total number of Low frequency bands         */
    const Word16 bands,                    /* i  : Q0  total number of subbands in a frame         */
    Word32 *L_band_energy,           /* i  : Qbe band energy of each subband                 */
    Word16 Qbe,                      /* i  : Q0  Q value for band_energy                     */
    const Word16 *band_start,              /* i  : Q0  subband start indices                       */
    const Word16 *band_end,                /* i  : Q0  subband end indices                         */
    const Word16 p2aflags[],               /* i  : Q0  missing bands in the core coder             */
    Word32 L_be_tonal[],             /* i  : QbeL tonal energy                               */
    Word16 QbeL,                     /* i  : Q0  Q value for be_tonal                        */
    GainItem_fx *pk_sf_fx,                /* i  :     toanl information for Sparse filling        */
    Word16 Qss,                      /* i  : Q0  Q value for pk_sf.nmrValue                  */
    Word16 *pul_res_pk               /* i  : Q0  pulse resolution information                */
)
{
    Word16 i, j, tmp;
    Word16 sb_ton_fx[SWB_HAR_RAN1]; /* Qss */
    Word16 sb_ton_loc_fx[SWB_HAR_RAN1]; /* Q0 */
    Word32 L_est_ton_ene[NB_SWB_SUBBANDS]; /* QetEne */
    Word16 QetEne;
    Word16 band_sf_fx[SWB_HAR_RAN1]; /* Qton_sf */
    Word16 pos_fx, k_fx, pos_tmp_fx; /* Q0 */
    Word16 exp_safe;
    Word16 temp_fx;
    Word16 Qtemp;

    Word16 band_pos_fx;
    Word16 count_pos_st_fx, count_pos_end_fx;

    Word16 exp_normd, exp_normn;

    Word16 ton_sf_fx; /* Qton_sf */
    Word16 Qton_sf;

    Word16 step_fx; /* Q15 */
    Word32 L_temp;

    Word16 enrd_r_fx; /* Q15 */

    Word32 L_band_energy_Linear[BANDS_MAX]; /* QbeL */
    Word16 temp_hi_fx, temp_lo_fx;
    Word16 exp_pow;
    Word16 exp_shift;

    Word16 Qbsf2[SWB_HAR_RAN1];

    Qton_sf = sub(sub(QsL, Qss), 1);

    enrd_r_fx = 29491;
    move16(); /* 0.9: 29491.2(Q15) */

    set32_fx(L_est_ton_ene, 0x0L, NB_SWB_SUBBANDS);
    set16_fx(sb_ton_loc_fx, -1,SWB_HAR_RAN1);

    /* Get the tonal information for sparse filling  */
    pos_fx = 0;
    move16();
    FOR(k_fx=0; k_fx<sub(bands, harmonic_band); k_fx++)
    {
        FOR(j=0; j<pul_res_pk[k_fx]; j++)
        {
            sb_ton_loc_fx[pos_fx] = pk_sf_fx[k_fx*8+j].gainIndex_fx;
            move16();
            sb_ton_fx[pos_fx] = pk_sf_fx[k_fx*8+j].nmrValue_fx;
            move16();
            pos_fx = add(pos_fx, 1);
        }
    }
    k_fx = 0;
    move16();
    pos_fx = 0;
    move16();
    pos_tmp_fx = 0;
    move16();

    DO
    {
        band_pos_fx = add(k_fx, harmonic_band);
        count_pos_st_fx = pos_fx;
        move16();
        WHILE(sub(sb_ton_loc_fx[pos_fx], sub(band_end[band_pos_fx], fLenLow)) <= 0 && sb_ton_loc_fx[pos_fx] >= 0 )
        {
            test();
            pos_fx = add(pos_fx, 1);
        }
        count_pos_end_fx = pos_fx;
        move16();

        exp_safe = 2; /* move16(); */
        QetEne = add(shl(sub(Qss, exp_safe), 1), 1);
        L_temp = L_add(L_est_ton_ene[k_fx], 0);
        FOR(i=count_pos_st_fx; i<count_pos_end_fx; i++)
        {
            temp_fx = shr(sb_ton_fx[i], exp_safe);
            L_temp = L_mac(L_temp, temp_fx, temp_fx);
        }
        L_est_ton_ene[k_fx] = L_temp;
        move32();

        IF (L_est_ton_ene[k_fx] <= 0x0L)
        {
            /* 1.0 */
            L_est_ton_ene[k_fx] = L_deposit_l(32767);
            QetEne=15;
        }

        /*ton_sf= (float) sqrt(be_tonal[band_pos]/est_ton_ene[k]);*/  /* be_tonal: QbeL,  est_ton_ene: (Qss-exp_safe)*2+1 */
        /* Div Part */
        exp_normd = norm_l(L_be_tonal[band_pos_fx]);
        exp_normd = sub(exp_normd, 1);
        exp_normn = norm_l(L_est_ton_ene[k_fx]);

        ton_sf_fx = 0x0;
        move16();
        IF ( L_be_tonal[band_pos_fx] > 0x0L )
        {
            ton_sf_fx = div_l(L_shl(L_be_tonal[band_pos_fx], exp_normd), extract_h(L_shl(L_est_ton_ene[k_fx], exp_normn))); /* QbeL+exp_normd - (QetEne+exp_normn-16) - 1 */
        }

        /* Sqrt Part */
        /*sqrt_32n_16_fx(L_deposit_h(ton_sf_fx), (QbeL+exp_normd)-(QetEne+exp_normn-16)-1+16, &ton_sf_fx, &Qton_sf);*/
        sqrt_32n_16_fx(L_deposit_h(ton_sf_fx), add(sub(add(QbeL, exp_normd), add(QetEne, exp_normn)), 31), &ton_sf_fx, &Qton_sf);

        FOR(i=count_pos_st_fx; i<count_pos_end_fx; i++)
        {
            band_sf_fx[pos_tmp_fx] = ton_sf_fx;
            move16();
            Qbsf2[pos_tmp_fx] = Qton_sf;
            move16();
            pos_tmp_fx = add(pos_tmp_fx, 1);
        }
        k_fx = add(k_fx, 1);
    } WHILE(sub(k_fx, NB_SWB_SUBBANDS) < 0);

    /* Gap filling for the core coder  */
    /* 0.077=20185(Q18) */
    L_temp = L_mult(20185, fLenHigh); /* 18+0+1= 19 */
    exp_normn = norm_l(L_temp);

    step_fx = div_s(0x3fff, extract_h(L_shl(L_temp, exp_normn)));
    step_fx = shl(step_fx,  sub(exp_normn, 11)); /* 15 - (14-(19+exp_normn-16)+15) */ /* Q15 */

    pos_tmp_fx = 0;
    move16();
    tmp = sub(bands, harmonic_band);
    FOR(k_fx=0; k_fx<tmp; k_fx++)
    {
        band_pos_fx = add(k_fx, harmonic_band);

        IF ( L_be_tonal[band_pos_fx] > 0x0L )
        {
            /* enrd_r *=(float)sqrt(be_tonal[band_pos]/pow(2.0f,band_energy[band_pos])); */
            /* Pow Part */
            L_temp = L_shr(L_band_energy[band_pos_fx], sub(Qbe, 16));
            temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
            exp_pow = sub(14, temp_hi_fx);
            L_band_energy_Linear[band_pos_fx] = Pow2(14, temp_lo_fx);
            move32();      /* Qexp_pow */
            L_band_energy_Linear[band_pos_fx] = L_shl(L_band_energy_Linear[band_pos_fx], sub(QbeL, exp_pow));

            /* Div Part */
            exp_normd = norm_l(L_be_tonal[band_pos_fx]);
            exp_normd = sub(exp_normd, 1);
            exp_normn = norm_l(L_band_energy_Linear[band_pos_fx]);
            temp_fx = div_l(L_shl(L_be_tonal[band_pos_fx], exp_normd), extract_h(L_shl(L_band_energy_Linear[band_pos_fx], exp_normn))); /* QbeL+exp_normd-(QbeL+exp_normn-16)-1 */
            sqrt_32n_16_fx(L_deposit_h(temp_fx), add(sub(exp_normd, exp_normn), 31), &temp_fx, &Qtemp);
            enrd_r_fx = extract_h(L_shl(L_mult(enrd_r_fx, temp_fx), sub(15, Qtemp)));

            enrd_r_fx = sub(enrd_r_fx, step_fx);
        }
        ELSE
        {
            enrd_r_fx = 0x0;
            move16();
        }

        IF(sub(p2aflags[band_pos_fx], 1) == 0)
        {
            FOR(i= band_start[band_pos_fx]; i<=band_end[band_pos_fx]; i++)
            {
                L_outBuf[i-fLenLow] = L_codbuf[i];
                move32();
            }
        }
        ELSE
        {
            pos_fx = 0;
            move16();
            pos_fx = add(pos_fx, pos_tmp_fx);
            exp_shift = sub(sub(QsL, Qss), 1);
            FOR(j=0; j<pul_res_pk[k_fx]; j++)
            {
                /*outBuf[pk_sf[k*8+j].gainIndex] = pk_sf[k*8+j].nmrValue*band_sf[pos]*enrd_r;*/
                /* outBuf:QsL , pk_sf.nmrValue:Qss, band_sf:Qbsf2(Qton_sf), enrd_r:Q15 */
                L_outBuf[pk_sf_fx[k_fx*8+j].gainIndex_fx] = L_shl(Mult_32_16(L_mult(band_sf_fx[pos_fx], enrd_r_fx), pk_sf_fx[k_fx*8+j].nmrValue_fx), sub(exp_shift ,Qbsf2[pos_fx]) );
                move32(); /* QsL - (Qbsf2[pos_fx]+Qss+16-15) */

                pos_fx = add(pos_fx, 1);
            }
        }
        pos_tmp_fx = add(pos_tmp_fx, pul_res_pk[k_fx]);

    }

    return;
}

void updat_prev_frm_fx(
    Word32 L_y2[],                   /* i/o: core coder buffer                 */
    Word32 L_t_audio[],              /*   o: core coder buffer                 */
    Word32 L_bwe_br,                 /*   i: core bitrate                      */
    Word16 length,                   /*   i: frame length coded bw             */
    const Word16 inner_frame,              /*   i: input frame length                */
    Word16 bands,                    /*   i: sub band resolution               */
    Word16 bwidth,                   /*   i: NB/WB/SWB indicator               */
    const Word16 is_transient,             /*   i: signal class information          */
    Word16 hqswb_clas,               /*   i: signal class information          */
    Word16 *prev_hqswb_clas,         /*   o: update signal class information   */
    Word16 prev_SWB_peak_pos[],      /*   o: update core coder last coded peaks*/
    Word16 prev_SWB_peak_pos_tmp[],  /*   o: update core coder last coded peaks*/
    Word16 prev_npulses[],           /*   o: update pulse resolution           */
    Word16 prev_npulses_tmp[],       /*   i: update pulse resolution           */
    Word16 *prev_frm_hfe2,           /*   o: update harmonics                  */
    Word16 *prev_stab_hfe2,          /*   o: update harmonics                  */
    Word16 bws_cnt                   /*   i: band width detector               */
)
{
    Word16 i,k,k1,k2,j;
    Word16 length1, length2, length3;

    /* Copy the coded MDCT coefficient to the output buffer */
    IF ( !is_transient )
    {
        /* Copy the scratch buffer to the output */
        Copy32( L_y2, L_t_audio, length);

        /* If the input frame is larger than coded bandwidth, zero out uncoded MDCT coefficients */
        IF ( sub(inner_frame, length) > 0 )
        {
            set32_fx( L_t_audio, 0x0L, sub(inner_frame, length) );
        }
    }
    ELSE /* transient frame */
    {
        test();
        IF( sub(inner_frame, length) == 0 || bws_cnt > 0)
        {
            /* Copy the scratch buffer to the output */
            Copy32( L_y2, L_t_audio, length);
        }
        ELSE
        {
            /* un-collapse transient frame and interleave zeros */
            FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                /* length/NUM_TIME_SWITCHING_BLOCKS */
                length1 = shr(length, 2); /* length/NUM_TIME_SWITCHING_BLOCKS */
                /* inner_frame/NUM_TIME_SWITCHING_BLOCKS */
                length2 = shr(inner_frame, 2); /* inner_frame/NUM_TIME_SWITCHING_BLOCKS */
                /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */
                length3 = shr(sub(inner_frame, length), 2); /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */

                k1 = 0;
                move16();
                k2 = 0;
                move16();

                /* un-collapse transient frame and interleave zeros */
                FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
                {
                    /* k1 = i*length/NUM_TIME_SWITCHING_BLOCKS; */
                    /* k2 = i*inner_frame/NUM_TIME_SWITCHING_BLOCKS; */

                    Copy32( L_y2 + k1, L_t_audio + k2, length1 );
                    set32_fx( L_t_audio + k2 + length1, 0x0L, length3);

                    k1 = add(k1, length1);
                    k2 = add(k2, length2);
                }
            }
        }
    }

    /* update */
    test();
    test();
    IF( (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) && sub(bwidth, SWB) == 0 )
    {
        *prev_hqswb_clas = hqswb_clas;
        move16();
        IF( sub(hqswb_clas, HQ_HARMONIC) != 0 )
        {
            *prev_frm_hfe2 = 0;
            move16();
            *prev_stab_hfe2 = 0;
            move16();
        }
    }
    ELSE
    {
        *prev_hqswb_clas = is_transient;
        move16();
    }

    test();
    test();
    test();
    IF( (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) && sub(bwidth, SWB) == 0 && sub(hqswb_clas, HQ_NORMAL) == 0 )
    {
        j = 0;
        move16();
        FOR(k=sub(bands,NI_USE_PREV_SPT_SBNUM); k<bands; k++)
        {
            prev_SWB_peak_pos[j] = prev_SWB_peak_pos_tmp[j];
            move16();
            j = add(j, 1);
        }
        FOR(k=0; k<bands; k++)
        {
            prev_npulses[k] = prev_npulses_tmp[k];
            move16();
        }
    }

    return;
}


/*-------------------------------------------------------------------*
 * sqrt_32n_16_fx()
 *
 * Routine for Sqrt
 *-------------------------------------------------------------------*/
void sqrt_32n_16_fx(
    Word32 L_in,              /* i  : input vector (Word32)       */
    Word16 Qin,               /* i  : Q value for L_in            */
    Word16 *out_fx,           /* o  : sqrt input vector (Word16)  */
    Word16 *Qout              /* o  : Q value for out_fx          */
)
{
    Word16 exp_norm;
    Word32 L_in_t;
    Word16 Qin_t;

    /* Input Normalization */
    exp_norm = norm_l(L_in);
    L_in_t = L_shl(L_in, exp_norm);
    Qin_t = add(Qin, exp_norm);

    /* SQRT part */
    IF ( s_and(Qin_t, 1) == 0 )
    {
        L_in_t = Sqrt_l(L_shr(L_in_t,1), &exp_norm);
        L_in_t = L_shr(L_in_t, exp_norm);
        *Qout = sub(shr(Qin_t, 1), 1);
    }
    ELSE
    {
        L_in_t = Sqrt_l(L_in_t, &exp_norm);
        L_in_t = L_shr(L_in_t, exp_norm);
        *Qout = shr(Qin_t, 1);
    }
    *out_fx = round_fx(L_in_t);
}

/*-------------------------------------------------------------------*
 * norm_vec_32_16_scale_fx()
 *
 * Routine for normilization and convert Word32 to Word16
 *-------------------------------------------------------------------*/
void norm_vec_32_16_scale_fx(
    Word32 *L_vec_in,     /* i  : input vector                                        */
    Word16 Qin,           /* i  : Q value for input vector                            */
    Word16 length_fx,     /* i  :vector size                                          */
    Word16 *vec_out_fx,   /* o  : output vectror                                      */
    Word16 *Qout,         /* o  : Q value for output vectro                           */
    Word16 exp_safe       /* i  : suppress left shift: for prevend overflow on sum    */
)
{
    Word16 i;
    Word32 L_temp;
    Word16 exp_norm, exp_shift;

    L_temp = L_deposit_l(0);
    FOR(i=0; i<length_fx; i++)
    {
        L_temp = L_or(L_temp, L_abs(L_vec_in[i]));
    }
    exp_norm = norm_l(L_temp);

    exp_shift = sub(exp_norm, exp_safe);
    *Qout = sub(add(Qin, exp_shift), 16);
    FOR(i=0; i<length_fx; i++)
    {
        vec_out_fx[i] = round_fx(L_shl(L_vec_in[i], exp_shift));
    }
}

void get_sigma_fx_har(
    const Word32 L_x_abs[],     /* i: Qi     absolute input    */
    const Word16 Qi,            /* i: Q0     Q value of x_abs  */
    const Word16 avg_fx,        /* i: Qavg   average of x_abs  */
    const Word16 Qavg,          /* i: Q0     Q value of avg    */
    const Word16 length_fx,     /* i: Q0     length            */
    Word16 *sigma_fx,     /* o: Qsigma sigma             */
    Word16 *Qsigma        /* o: Q0     Q value of sigma  */
)
{
    Word16 i;
    Word32 L_d;
    Word16 d_fx;

    Word16 length1_fx;
    Word16 exp_normd;
    Word16 exp_normn;
    Word16 exp_shift;

    Word16 exp_norm;

    Word32 L_temp;

    Word16 temp_fx;

    Word32 L_tmp;
    Word32 L_x_abs_sh[L_FRAME32k];
    Word16 Qd;

    Word16 exp_safe;

    exp_safe = 4;
    move16();  /* max 103 < 2^7  -> 4+4=8 */

    L_tmp = L_deposit_l(0);
    FOR( i=0; i<length_fx; i++ )
    {
        L_tmp = L_or(L_tmp, L_x_abs[i]);
    }
    exp_norm = norm_l(L_tmp);
    exp_norm = sub(exp_norm, exp_safe);
    FOR( i=0; i<length_fx; i++ )
    {
        L_x_abs_sh[i] = L_shl(L_x_abs[i], exp_norm);
    }

    L_d = L_deposit_l(0);
    FOR( i=0; i<length_fx; i++ )
    {
        temp_fx = extract_h(L_x_abs_sh[i]);
        L_d = L_mac(L_d, temp_fx, temp_fx); /* (Qi+exp_norm-16)*2+1 */
    }
    Qd = add(shl(sub(add(Qi, exp_norm), 16), 1), 1);

    /* d /= (length-1);  */
    length1_fx = sub(length_fx, 1);

    /* d /= (length-1);  */
    exp_normn = norm_l(L_d);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(length1_fx);

    exp_shift = sub(sub(exp_normn, exp_normd), 1);
    d_fx = div_l(L_shl(L_d, exp_normn), shl(length1_fx, exp_normd)); /* Qabs*2+1+exp_normn - exp_normd - 1 */
    L_d = L_shr(L_deposit_l(d_fx), exp_shift); /* Qabs*2+1 + 16 */
    /* Qd = ((Qd+exp_normn) - exp_normd-exp_shift -1); */
    Qd = sub(sub(add(Qd, exp_normn), add(exp_normd, exp_shift)), 1);

    /* d -= avg*avg; */
    L_tmp = L_mult(avg_fx, avg_fx); /* Qavg*2+1 */
    L_tmp = L_shr(L_tmp, sub(add(shl(Qavg,1),1), Qd));
    L_d = L_sub(L_d, L_tmp);

    exp_norm = norm_l(L_d);
    L_d = L_shl(L_d, exp_norm);
    Qd = add(Qd, exp_norm);

    /* sigma = (float)sqrt(d); */
    IF ( s_and(Qd, 1) == 0 )/* Qd % 2 == 0 */
    {
        L_temp = Sqrt_l(L_shr(L_d, 1), &exp_norm);
        L_temp = L_shr(L_temp, exp_norm);
        *Qsigma = sub(shr(Qd, 1), 1);
        *sigma_fx = round_fx(L_temp);
    }
    ELSE
    {
        L_temp = Sqrt_l(L_d, &exp_norm);
        L_temp = L_shr(L_temp, exp_norm);
        *Qsigma = shr(Qd, 1);
        *sigma_fx = round_fx(L_temp);
    }

    return;
}

void FindNBiggest2_simple_fx_har(
    const Word32 *L_inBuf,        /* i  : input buffer (searched)                     */
    const Word16 Qabs_in,         /* i  : Q value of input buffer                     */
    GainItem_fx *pk_sf_fx,       /* o  : N biggest components found                  */
    const Word16 nIdx_fx,         /* i  : search length                               */
    Word16 *n_fx,           /* i  : number of components searched (N biggest)   */
    Word16 n_nbiggestsearch /* i  :                                             */
)
{
    Word16 j;
    Word32 L_avg_in;
    Word32 L_abs_in[400];
    Word32 L_abs_in_sft[400];
    Word16 avg_in_fx;
    Word32 L_max_in;
    Word16 Qavg_in;

    Word16 exp_normd;
    Word16 exp_normn;

    Word16 temp_fx;

    Word16 sigma_fx;
    Word16 Qsigma;

    Word16 peak_cnt_fx;
    Word32 L_thr;
    Word32 L_temp;

    L_max_in = L_deposit_l(0);
    L_avg_in = L_deposit_l(0);

    FOR (j = 0; j < nIdx_fx; j++)
    {
        L_abs_in[j] = L_abs(L_inBuf[j]); /* Qabs_in */
        L_abs_in_sft[j] = L_shr(L_abs_in[j], 8); /* 8 is safe shift */

        if( L_sub(L_max_in, L_abs_in_sft[j]) < 0 )
        {
            L_max_in = L_abs_in_sft[j];
            move32();
        }

        L_avg_in = L_add(L_avg_in, L_abs_in_sft[j]);
    }

    /*avg_in /= (float)nIdx;*/
    exp_normn = norm_l(L_avg_in);
    exp_normn = sub(exp_normn, 1);
    L_avg_in = L_shl(L_avg_in, exp_normn);
    exp_normd = norm_s(nIdx_fx);
    temp_fx = shl(nIdx_fx, exp_normd);
    avg_in_fx = div_l(L_avg_in, temp_fx);
    Qavg_in = sub(add(sub(Qabs_in, 8), exp_normn) , add(exp_normd, 1));

    peak_cnt_fx = 0;
    move16();
    IF( L_sub(L_max_in, 0x1) <= 0 )
    {
        FOR (j = 0; j < n_nbiggestsearch; j++)
        {
            pk_sf_fx[peak_cnt_fx].nmrValue_fx = 0x0;
            move16();
            pk_sf_fx[peak_cnt_fx].gainIndex_fx = j;
            move16();
            peak_cnt_fx = add(peak_cnt_fx, 1);
        }
    }

    get_sigma_fx_har( L_abs_in, Qabs_in, avg_in_fx, Qavg_in, nIdx_fx, &sigma_fx, &Qsigma );

    temp_fx = mult_r(sigma_fx, 18841);  /* 18841 = 1.15(Q14)  Qsigma + Q14 + 1 */
    L_thr = L_add(extract_l(avg_in_fx), L_shr(extract_l(temp_fx), sub(sub(Qsigma,1), Qavg_in)) );
    L_thr = L_shr(L_thr, sub(Qavg_in, Qabs_in));

    IF( sub(peak_cnt_fx, n_nbiggestsearch) < 0 )
    {
        FOR (j = 0; j < nIdx_fx; j++)
        {
            IF(L_sub(L_abs_in[j], L_thr) > 0 )
            {
                pk_sf_fx[peak_cnt_fx].nmrValue_fx = round_fx(L_abs_in[j]);  /* Qabs_in-16 */
                pk_sf_fx[peak_cnt_fx].gainIndex_fx = j;
                move16();
                L_abs_in[j] = L_deposit_l(0);
                peak_cnt_fx = add(peak_cnt_fx, 1);
                move16();
            }

            IF( sub(peak_cnt_fx, n_nbiggestsearch) == 0 )
            {
                BREAK;
            }
        }
    }

    /* thr *= (0.3f / n_nbiggestsearch) * peak_cnt + 0.7f; */
    /* 0.3=19661(Q16) */
    temp_fx = div_s_ss(19661, n_nbiggestsearch);
    L_temp = L_mult(temp_fx, peak_cnt_fx); /* 16+0+1 */
    temp_fx = add(round_fx(L_shl(L_temp, 14)), 22938);  /* shift: 17+14-16 -> 15 */ /* 0.7(22937.6:Q15)*/
    L_thr = Mult_32_16(L_thr, temp_fx);

    IF( sub(peak_cnt_fx, n_nbiggestsearch) < 0 )
    {
        FOR (j = 0; j < nIdx_fx; j++)
        {
            IF( L_sub(L_abs_in[j], L_thr) > 0 )
            {
                pk_sf_fx[peak_cnt_fx].nmrValue_fx = round_fx(L_abs_in[j]);  /* Qabs_in-16 */
                pk_sf_fx[peak_cnt_fx].gainIndex_fx = j;
                move16();
                L_abs_in[j] = L_deposit_l(0);
                peak_cnt_fx = add(peak_cnt_fx, 1);
                move16();
            }

            IF( sub(peak_cnt_fx, n_nbiggestsearch) == 0 )
            {
                BREAK;
            }
        }
    }

    /* thr *= (0.6f / n_nbiggestsearch) * peak_cnt + 0.3f; */
    /* 0.6=19661(Q15) */
    temp_fx = div_s_ss(19661, n_nbiggestsearch);
    L_temp = L_mult(temp_fx, peak_cnt_fx); /* 15+0+1 */
    temp_fx = add(round_fx(L_shl(L_temp, 15)), 9830);  /* shift: 16+15-16 -> 15 */ /* 0.3(9830.4:Q15)*/

    L_thr = Mult_32_16(L_thr, temp_fx);
    IF( sub(peak_cnt_fx, n_nbiggestsearch) < 0 )
    {
        FOR (j = 0; j < nIdx_fx; j++)
        {
            IF( L_sub(L_abs_in[j], L_thr) > 0 )
            {
                pk_sf_fx[peak_cnt_fx].nmrValue_fx = round_fx(L_abs_in[j]);  /* Qabs_in-16 */
                pk_sf_fx[peak_cnt_fx].gainIndex_fx = j;
                move16();
                L_abs_in[j] = L_deposit_l(0);
                peak_cnt_fx = add(peak_cnt_fx, 1);
                move16();
            }

            IF( sub(peak_cnt_fx, n_nbiggestsearch) == 0 )
            {
                BREAK;
            }
        }
    }

    *n_fx = peak_cnt_fx;
    move16();
}

/*--------------------------------------------------------------------------*
 * spectrumsmooth_noiseton()
 * Spectrum normalization for the the core coder
 *--------------------------------------------------------------------------*/
Word16 spectrumsmooth_noiseton_fx(   /* o  : Qss  ss_min                                      */
    Word32 L_spectra[],        /* i  : Qs   core coder                                  */
    /*Word16 Qs,*/               /* i  : Q0   Q value for spectra, spectra_ni             */
    const Word32 L_spectra_ni[],     /* i  : Qs   core coder with sparse filling              */
    Word16 sspectra_fx[],      /* o  : Qss  Smoothed tonal information from core coder  */
    Word16 sspectra_diff_fx[], /* o  : Qss  non tonal infomration for gap filling       */
    Word16 sspectra_ni_fx[],   /* o  : Qss  smoothed core coder                         */
    Word16 *Qss,               /* o  : Q0   Q value for sspectra*                       */
    const Word16 fLenLow_fx,         /* i  : Q0   low frequency boundaries                    */
    Word16 *ni_seed_fx         /* io : Q0   random seed                                 */
)
{
    Word16 i;
    Word32 L_spectra_diff[L_FRAME32k];
    Word16 ni_ratio_fx;           /* Q15 */
    Word16 ss_min_fx;             /* Q10 */
    Word16 cut_sig_th_fx;         /* Q10 */
    Word16 cut_ni_th_fx;          /* Q10 */
    Word16 pcnt_fx, sign_fx;
    Word16 exp_normn, exp_normd;

    Word16 ratio_fx;
    Word32 L_temp;
    Word32 L_spectra_rm[L_FRAME32k];
    Word32 L_cut_input=410;


    /*Get the pulse resolution for the core coder*/
    pcnt_fx = 0;
    move16();
    FOR(i=0; i<fLenLow_fx; i++)
    {
        if(L_spectra[i] != 0x0L)
        {
            pcnt_fx = add(pcnt_fx, 1);
        }
    }

    /*ni_ratio = 4.0f*(pcnt)/(fLenLow+0.0f);*/
    exp_normn = norm_s(pcnt_fx);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(fLenLow_fx);

    ni_ratio_fx = div_s(shl(pcnt_fx, exp_normn), shl(fLenLow_fx, exp_normd)); /* exp_normn - exp_normd + 15 - 2 */

    ni_ratio_fx = shl(ni_ratio_fx, add(sub(exp_normd, exp_normn),2) );  /* 15 - (exp_normn-exp_normd+15-2) */

    /*ni_ratio = min(0.9f, ni_ratio);*/
    ni_ratio_fx = s_min(29491, ni_ratio_fx);  /* 0.9: 29491(Q15) */
    Overflow = 0;
    move16();

    ss_min_fx = mult_r(ni_ratio_fx, 10240); /* Q15+Q10-15 = Q10 */ /* 10.0: 10240(Q10) */
    cut_sig_th_fx = shr(ss_min_fx, 2); /* 1/4 */
    cut_sig_th_fx = s_max(973, cut_sig_th_fx); /* 0.95: 972.8(Q10) */

    /*core coder normalization for gap filling*/
    FOR(i=0; i<fLenLow_fx; i++)
    {
        L_spectra_rm[i] = L_deposit_l(0);
        if( L_sub(L_abs(L_spectra[i]), L_cut_input) >= 0x0L )
        {
            L_spectra_rm[i] = L_spectra[i];
            move32();
        }
    }
    SpectrumSmoothing_fx( L_spectra_rm, sspectra_fx, Qss, fLenLow_fx, cut_sig_th_fx);

    /*Extract noise informaton from the core coder*/
    Copy(sspectra_fx, sspectra_ni_fx, fLenLow_fx);
    FOR(i=0; i<fLenLow_fx; i++)
    {
        L_spectra_diff[i] = L_sub(L_spectra_ni[i], L_spectra[i]);
        move32();
    }
    cut_ni_th_fx = 0x0;
    move16();
    /*normalize sparse filled components*/
    FOR(i=0; i<fLenLow_fx; i++)
    {
        L_spectra_rm[i] = L_deposit_l(0);
        if( L_sub(L_abs(L_spectra_diff[i]), L_cut_input) >= 0x0L )
        {
            L_spectra_rm[i] = L_spectra_diff[i];
            move32();
        }
    }
    SpectrumSmoothing_fx( L_spectra_rm, sspectra_diff_fx, Qss, fLenLow_fx, cut_ni_th_fx);

    /*Normalized corecoder for Gap filling */
    /* ratio = 1 - ss_min/10.0 */
    ratio_fx = sub(0x7fff, shl(mult_r(ss_min_fx, 3277), 15-10)); /* Q15 */
    FOR(i=0; i<fLenLow_fx; i++)
    {
        sign_fx = 0;
        move16();
        if(sspectra_fx[i] < 0)
        {
            sign_fx = 1;
            move16();
        }
        IF( sub(abs_s(sspectra_fx[i]), ss_min_fx ) > 0)
        {
            /*sspectra[i] = sign*((10-ss_min)/10.0f*(float)fabs(sspectra[i])+ss_min);*/
            sspectra_fx[i] = add(mult_r(ratio_fx, abs_s(sspectra_fx[i])), ss_min_fx);
            IF(sign_fx != 0)
            {
                sspectra_fx[i] = negate(sspectra_fx[i]);
                move16();
            }
        }
        sspectra_ni_fx[i] = add(sspectra_fx[i], mult_r(sspectra_diff_fx[i], ni_ratio_fx));
        move16();

        IF( sspectra_ni_fx[i] == 0x0)
        {
            /*sspectra_ni[i] = 0.5f*10.0f*ni_ratio* own_random(ni_seed)/32768.0f;*/
            L_temp = L_mult(ni_ratio_fx, 20480);                       /* ni_ratio*5 */
            L_temp = Mult_32_16(L_temp, Random(ni_seed_fx));           /* Q28 */
            sspectra_ni_fx[i] = round_fx(L_shr(L_temp, 2));            /* Qss */
        }
    }

    return(ss_min_fx);
}

/*--------------------------------------------------------------------------*
 * noiseinj_hf()
 * level adjustments for the missing bands in the core coder
 *--------------------------------------------------------------------------*/
void noiseinj_hf_fx(
    Word32 L_xSynth_har[],          /* i/o : Qs   gap filled information            */
    Word16 Qs,                      /* i   : Q0   Q value for xSynth_har            */
    Word32 L_th_g[],                /* i   : Qs   level adjustment information      */
    Word32 L_band_energy[],         /* i   : Qbe  subband energies                  */
    Word16 Qbe,                     /* i   : Q0   Q value for band_energy           */
    Word16 *prev_En_sb_fx,          /* i/o : QsEn smoothed sqrt band Energies       */
    const Word16 p2a_flags_fx[],          /* i   : Q0   Missing bands in the core coder   */
    const Word16 BANDS_fx,                /* i   : Q0   total bands                       */
    const Word16 band_start_fx[],         /* i   : Q0   band start indices                */
    const Word16 band_end_fx[],           /* i   : Q0   band end indices                  */
    const Word16 fLenLow_fx,              /* i   : Q0   low frequency bandwidth           */
    const Word16 fLenHigh_fx              /* i   : Q0   SWB frequency bandwidth           */
)
{
    Word16 k,i;

    Word16 *p_prev_En_sb_fx;
    Word16 QbeL=7; /* Don't need 3, because this E only use under th samples */ /* QsEn=3 */

    Word16 map_pulse_t_fx[L_FRAME32k];
    Word16 map_pulse_fx[L_FRAME32k];

    Word16 QsEn=4; /* kiken */
    Word32 L_En[NB_SWB_SUBBANDS];
    Word32 *p_L_En;
    Word16 QE;
    Word16 sqrt_En_fx[NB_SWB_SUBBANDS]; /* QsEn */
    Word16 *p_sqrt_En_fx; /* QsEn */

    Word16 Enn_sm_sb_fx[NB_SWB_SUBBANDS];
    Word16 *p_Enn_sm_sb_fx;

    Word16 exp_safe;
    Word16 xSynth_har_fx[L_FRAME32k];
    Word16 QxSynth;

    Word16 Qtemp;

    Word16 ni_scale_fx; /* Q14 */

    Word16 temp_fx;
    Word32 L_temp;

    Word16 exp_normn, exp_normd;
    Word16 div_fx;
    Word16 Qdiv;

    set16_fx(map_pulse_t_fx, 0, band_end_fx[BANDS_fx-1]+1);
    set16_fx(map_pulse_fx, 0, band_end_fx[BANDS_fx-1]+1);

    /*level adjust the missing bands in the core coder */
    exp_safe = 4; /*move16();*/
    norm_vec_32_16_scale_fx(L_xSynth_har, Qs, fLenHigh_fx, xSynth_har_fx, &QxSynth, exp_safe);
    QE = add(shl(QxSynth, 1),1);

    p_L_En = L_En;
    p_sqrt_En_fx = sqrt_En_fx;
    FOR(k=sub(BANDS_fx, NB_SWB_SUBBANDS); k<BANDS_fx; k++)
    {
        *p_L_En = L_deposit_l(0);
        IF(p2a_flags_fx[k] == 0)
        {
            FOR(i=band_start_fx[k]; i<=band_end_fx[k]; i++)
            {
                IF( L_sub(L_abs(L_xSynth_har[i-fLenLow_fx]), L_th_g[k-(BANDS_fx-NB_SWB_SUBBANDS)] ) <= 0x0L )
                {
                    *p_L_En = L_mac(*p_L_En,    xSynth_har_fx[i-fLenLow_fx], xSynth_har_fx[i-fLenLow_fx]);
                }
                ELSE
                {
                    map_pulse_t_fx[i] = 1;
                    move16();
                }
            }
            *p_L_En = L_shl(*p_L_En, sub(QbeL, QE));
            move32();
            /**p_L_En = (float)sqrt(*p_En);*/
            sqrt_32n_16_fx(*p_L_En, QbeL, p_sqrt_En_fx, &Qtemp);
            *p_sqrt_En_fx = shl(*p_sqrt_En_fx, sub(QsEn, Qtemp)); /* -> Q2 */ move16();
        }
        p_L_En++;
        p_sqrt_En_fx++;
    }

    p_sqrt_En_fx = sqrt_En_fx;
    p_Enn_sm_sb_fx = Enn_sm_sb_fx;
    p_prev_En_sb_fx = prev_En_sb_fx;
    FOR(k=BANDS_fx-NB_SWB_SUBBANDS; k<BANDS_fx; k++)
    {
        *p_Enn_sm_sb_fx = prev_En_sb_fx[k-(BANDS_fx-NB_SWB_SUBBANDS)];
        move16(); /* QsEn */
        IF(p2a_flags_fx[k] == 0)
        {
            L_temp = Mult_32_16(L_band_energy[k], 26214); /* 0.8: 26214(Q15) */
            temp_fx = round_fx(L_shl(L_temp, sub(QsEn, sub(Qbe,16))));
            IF( sub(*p_prev_En_sb_fx, temp_fx) < 0 )
            {
                /**p_Enn_sm_sb = (0.15f*(*p_En)) + (0.85f*prev_En_sb[k-(BANDS-NB_SWB_SUBBANDS)]);*/
                /* 0.15: 4915.2(Q15) 0.85: 27852.80(Q15) */
                *p_Enn_sm_sb_fx = round_fx(L_mac(L_mult(*p_sqrt_En_fx, 4915), *p_prev_En_sb_fx, 27853));
            }
            ELSE
            {
                /**p_Enn_sm_sb = (0.8f*(*p_En)) + (0.2f*prev_En_sb[k-(BANDS-NB_SWB_SUBBANDS)]);*/
                /* 0.8: 26214.4(Q15) 0.2:6553.6(Q15) */
                *p_Enn_sm_sb_fx = round_fx(L_mac(L_mult(*p_sqrt_En_fx, 26214), *p_prev_En_sb_fx, 6554));
            }
        }

        p_Enn_sm_sb_fx++;
        p_sqrt_En_fx++;
        p_prev_En_sb_fx++;
    }

    p_sqrt_En_fx = sqrt_En_fx;
    p_Enn_sm_sb_fx = Enn_sm_sb_fx;
    p_prev_En_sb_fx = prev_En_sb_fx;
    map_pulse_fx[fLenLow_fx] = (map_pulse_t_fx[fLenLow_fx] | map_pulse_t_fx[fLenLow_fx+1]);
    logic16();
    move16();
    FOR(i=fLenLow_fx+1; i<band_end_fx[BANDS_fx-1]; i++)
    {
        map_pulse_fx[i] = ( map_pulse_t_fx[i-1] | map_pulse_t_fx[i] | map_pulse_t_fx[i+1] );
        logic16();
        logic16();
        move16();
    }
    map_pulse_fx[i] = (map_pulse_t_fx[i-1] | map_pulse_t_fx[i]);
    logic16();
    move16();

    FOR(k=BANDS_fx-NB_SWB_SUBBANDS; k<BANDS_fx; k++)
    {
        test();
        IF(p2a_flags_fx[k] == 0 && *p_sqrt_En_fx != 0x0)
        {
            /*ni_scale = sqrt((*p_Enn_sm_sb)/(*p_En));*/
            /* Div Part */
            exp_normn = norm_s(*p_Enn_sm_sb_fx);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_s(*p_sqrt_En_fx);
            div_fx = div_s(shl(*p_Enn_sm_sb_fx, exp_normn), shl(*p_sqrt_En_fx, exp_normd));
            Qdiv = add(sub(exp_normn, exp_normd), 15);

            /* SQRT Part */
            sqrt_32n_16_fx(L_deposit_h(div_fx), add(Qdiv, 16), &ni_scale_fx, &Qtemp);
            ni_scale_fx = shl(ni_scale_fx, sub(14, Qtemp));

            ni_scale_fx = s_min(20408, ni_scale_fx); /* 1.25=20408.0(Q14) */

            ni_scale_fx = s_max(12288, ni_scale_fx); /* 0.75=12288.0(Q14) */

            ni_scale_fx = mult_r(ni_scale_fx, 26214); /* 0.8=26214.4(Q15)   -> Q14 */
            FOR(i=band_start_fx[k]; i<=band_end_fx[k]; i++)
            {
                IF( L_sub(L_abs(L_xSynth_har[i-fLenLow_fx]), L_th_g[k-(BANDS_fx-NB_SWB_SUBBANDS)]) <= 0x0L )
                {
                    IF(map_pulse_fx[i] == 0)
                    {
                        L_xSynth_har[i-fLenLow_fx] = L_shl(Mult_32_16(L_xSynth_har[i-fLenLow_fx], ni_scale_fx), 1);  /* Q12+Q14-15-1 = Q12 */ move32();
                    }
                }
            }
            *p_prev_En_sb_fx = *p_Enn_sm_sb_fx;
            move16();
        }
        p_Enn_sm_sb_fx++;
        p_sqrt_En_fx++;
        p_prev_En_sb_fx++;
    }

    return;
}
