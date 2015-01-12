/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

#include "prot_fx.h"
#include "rom_com_fx.h"




/*--------------------------------------------------------------------------*
 * Regression_Anal()
 *
 *
 *--------------------------------------------------------------------------*/
static void Regression_Anal_fx(
    const Word32 *values_fx,    /* i : Previous values                  */
    Word32 *r_p_fx,       /* o : Output r[a b] array : y=ax+b     */
    const Word16 num_pgf    /* i : Number of previous good frame    */
)
{
    Word16 i;
    Word16 tmp;
    Word32 L_tmp1, L_tmp2;
    Word16 aindex_fx[MAX_PGF+1];
    Word32 b_p_fx[MAX_PGF+1];

    /* Initialize */
    FOR (i=0; i<num_pgf+1; i++)
    {
        aindex_fx[i] = 0;
        move16();
        b_p_fx[i] = 0;
        move16();
    }

    /* [aindex[0] aindex[1]][r[0]]=[b[0]]*/
    /* [aindex[1] aindex[2]][r[1]] [b[1]]*/
    /* r[0] is the y-intercept(initial value). r[1] is slope*/

    /* r[0] = (b[0]a[2]-a[1]b[1])/(a[0]a[2]-a[1]a[1])
       r[1] = (b[0]a[1]-a[0]b[1])/(a[1]a[1]-a[0]a[2]) */
    /*aindex[0]=num_pgf; */
    aindex_fx[0] = num_pgf;
    move16();
    FOR (i=1; i<num_pgf+1; i++)
    {
        aindex_fx[1] = add(aindex_fx[1], i);
        aindex_fx[2] = add(aindex_fx[2], i_mult(i, i));
    }
    /* Calculate b[] */
    FOR (i=0; i<num_pgf; i++)
    {
        b_p_fx[0] = L_add(b_p_fx[0], L_shr(values_fx[i], 2));/*10 */
        b_p_fx[1] = L_add(b_p_fx[1], Mult_32_16(values_fx[i], shl(sub(num_pgf, i), 13)));/*10 */
    }

    tmp = sub(i_mult(aindex_fx[0], aindex_fx[2]), i_mult(aindex_fx[1], aindex_fx[1]));
    tmp = inv_tbl_fx[tmp];
    move16();/*15 */
    L_tmp1 = L_sub(Mult_32_16(b_p_fx[0], shl(aindex_fx[2], 10)), Mult_32_16(b_p_fx[1], shl(aindex_fx[1], 10)));/*5 */
    L_tmp2 = L_sub(Mult_32_16(b_p_fx[1], shl(aindex_fx[0], 10)), Mult_32_16(b_p_fx[0], shl(aindex_fx[1], 10)));/*5 */
    r_p_fx[0] = Mult_32_16(L_tmp1, tmp);
    r_p_fx[1] = Mult_32_16(L_tmp2, tmp);

    return;
}

static void FEC_scaling_fx(
    Word32 *old_coeffs_fx,            /* i/o  : Pointer to old MDCT coeffs.                    */
    Word32 *t_audio_q_fx,             /* o  : MDCT coeffs. (for synthesis)                     */
    Word16 *Norm_gain_fx,             /* i  : Gain for Norm of each band                       */
    Word16 *HQ_FEC_seed,           /* i/o  : Seed for Ransom number Generator               */
    Word16 nb_sfm,                 /* i  : Number of sub-band                               */
    const Word16 *start_band,
    const Word16 *end_band,
    Word16 freqOffset,
    Word16 output_frame            /* i  : Frame size                                       */

)
{
    Word16 i, j;

    FOR (i = 0; i < RANDOM_START; i++)
    {
        FOR (j = start_band[i]+freqOffset; j <= end_band[i]+freqOffset; j++)
        {
            t_audio_q_fx[j] = Mult_32_16(old_coeffs_fx[j], Norm_gain_fx[i]);/*12 */
        }
    }

    FOR (i = RANDOM_START; i < nb_sfm; i++)
    {
        FOR (j = start_band[i]+freqOffset; j <= end_band[i]+freqOffset; j++)
        {
            IF (Random( HQ_FEC_seed )<0)
            {
                t_audio_q_fx[j] = Mult_32_16(L_negate(old_coeffs_fx[j]), Norm_gain_fx[i]);/*12*/
            }
            ELSE
            {
                t_audio_q_fx[j] = Mult_32_16(old_coeffs_fx[j], Norm_gain_fx[i]);/*12*/
            }
        }
    }

    FOR (i = end_band[nb_sfm-1]+freqOffset; i < output_frame+freqOffset; i++)
    {
        IF (Random( HQ_FEC_seed )<0)
        {
            t_audio_q_fx[i] = Mult_32_16(L_negate(old_coeffs_fx[i]), Norm_gain_fx[nb_sfm-1]);/*12*/
        }
        ELSE
        {
            t_audio_q_fx[i] = Mult_32_16(old_coeffs_fx[i], Norm_gain_fx[nb_sfm-1]);/*12*/
        }
    }

    return;
}

void HQ_FEC_processing_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                          */
    Word32 *t_audio_q_fx,             /* o  : MDCT coeffs. (for synthesis)              Q12       */
    Word16 is_transient,              /* i  : Old flag for transient                           */
    Word32 ynrm_values_fx[][MAX_PGF], /* i  : Old average Norm values for each group of bands Q12 */
    Word32 r_p_values_fx[][MAX_ROW],  /* i  : Computed y-intercept and slope by Regression    Q5 */
    Word16 num_Sb,                    /* i  : Number of sub-band group                         */
    Word16 nb_sfm,                    /* i  : Number of sub-band                               */
    Word16 *Num_bands_p,              /* i  : Number of coeffs. for each sub-band              */
    Word16 output_frame,              /* i  : Frame size                                       */
    const Word16 *sfm_start,                /* i  : Start of bands                                   */
    const Word16 *sfm_end,                  /* i  : End of bands                                     */
    Word16 output_frame_org           /* i  : Original Frame size                              */
)
{
    Word16 i, j, k;
    Word16 mute_start,num_pgf;
    Word16 Num_sb_bwe, core_end_freq;
    Word16 sfm;

    Word16 tmp_fx, exp1, exp2;
    Word32 norm_p_fx[MAX_SB_NB];
    Word32 *norm_values_fx, *r_p_fx;
    Word16 energy_diff_fx;

    core_end_freq = 0;
    move16();

    /* Decide the start frame number for adaptive muting */
    /* Normalized energy difference between the current frame and the moving average */
    tmp_fx = abs_s(sub(st_fx->energy_MA_Curr_fx[1], st_fx->energy_MA_Curr_fx[0]));
    exp1 = sub(norm_s(tmp_fx), 1);
    st_fx->energy_MA_Curr_fx[0] = s_max(st_fx->energy_MA_Curr_fx[0],1);
    move16();
    exp2 = norm_s(st_fx->energy_MA_Curr_fx[0]);
    tmp_fx = div_s(shl(tmp_fx, exp1), shl(st_fx->energy_MA_Curr_fx[0], exp2));/*15 + exp1 - exp2*/
    energy_diff_fx = shl(tmp_fx, sub(sub(exp2, exp1), 5));/*10*/
    test();
    IF ((sub(energy_diff_fx, 1024) < 0) && (is_transient==0)) /* First erasure frame */
    {
        mute_start = 5;
        move16();
    }
    ELSE
    {
        mute_start = 2;
        move16();
    }
    test();
    test();
    if( sub(st_fx->prev_old_bfi_fx, 1) == 0 && sub(st_fx->nbLostCmpt, 1) == 0 && sub(output_frame_org, L_FRAME8k) == 0 )
    {
        st_fx->nbLostCmpt = add(st_fx->nbLostCmpt, 1);
    }

    /* Frequency-domain FEC */
    IF ( sub(st_fx->nbLostCmpt, 1) == 0 ) /* First erasure frame */
    {
        IF ( is_transient == 0 )
        {
            IF (sub(energy_diff_fx, 1024) < 0)
            {
                FOR (i=0; i < output_frame; i++)
                {
                    t_audio_q_fx[i] = st_fx->old_coeffs_fx[i];
                    move32();
                }
            }
            ELSE
            {
                FOR (i=0; i < output_frame; i++)
                {
                    st_fx->old_coeffs_fx[i] = Mult_32_16(st_fx->old_coeffs_fx[i], 23170);/*23170, 3dB, Q15*/
                    t_audio_q_fx[i] = st_fx->old_coeffs_fx[i];
                    move32();
                }
            }

            /* Sign prediction in 4-dim bands up to 1.6 kHz*/
            IF (st_fx->old_is_transient_fx[1] == 0)
            {
                IF (st_fx->old_is_transient_fx[2] == 0)
                {
                    FOR (sfm = 0; sfm < HQ_FEC_SIGN_SFM; sfm++)
                    {
                        IF (sub(st_fx->prev_sign_switch_fx[sfm], HQ_FEC_SIGN_THRES) >= 0)
                        {
                            FOR (i = 0; i < HQ_FEC_BAND_SIZE; i++)
                            {
                                t_audio_q_fx[i+sfm*HQ_FEC_BAND_SIZE] = L_negate(t_audio_q_fx[i+sfm*HQ_FEC_BAND_SIZE]);
                                move32();
                            }
                        }
                    }
                }
                ELSE
                {
                    FOR (sfm = 0; sfm < HQ_FEC_SIGN_SFM; sfm++)
                    {
                        IF (sub(st_fx->prev_sign_switch_fx[sfm], HQ_FEC_SIGN_THRES_TRANS) >= 0)
                        {
                            FOR (i = 0; i < HQ_FEC_BAND_SIZE; i++)
                            {
                                t_audio_q_fx[i+sfm*HQ_FEC_BAND_SIZE] = L_negate(t_audio_q_fx[i+sfm*HQ_FEC_BAND_SIZE]);
                                move32();
                            }
                        }
                    }
                }
            }
            ELSE
            {
                FOR (i = RANDOM_START*8; i < output_frame; i++)
                {
                    IF(Random(&st_fx->HQ_FEC_seed_fx) < 0)
                    {
                        t_audio_q_fx[i] = L_negate(t_audio_q_fx[i]);
                        move32();
                    }
                }
            }
        }
        ELSE
        {
            IF( st_fx->old_is_transient_fx[1] )      /* hangover */
            {
                FOR (i=0; i < output_frame; i++)
                {
                    st_fx->old_coeffs_fx[i] = Mult_32_16(st_fx->old_coeffs_fx[i], 23170);
                    t_audio_q_fx[i] = st_fx->old_coeffs_fx[i];
                    move32();
                }
            }
            ELSE
            {
                FOR (i = 0; i < RANDOM_START*8; i++)
                {
                    st_fx->old_coeffs_fx[i] = Mult_32_16(st_fx->old_coeffs_fx[i], 23170);
                    t_audio_q_fx[i] = st_fx->old_coeffs_fx[i];
                    move32();
                }

                FOR (i = RANDOM_START*8; i < output_frame; i++)
                {
                    st_fx->old_coeffs_fx[i] = Mult_32_16(st_fx->old_coeffs_fx[i], 23170);
                    IF (Random(&st_fx->HQ_FEC_seed_fx) < 0)
                    {
                        t_audio_q_fx[i] = L_negate(st_fx->old_coeffs_fx[i]);
                        move32();
                    }
                    ELSE
                    {
                        t_audio_q_fx[i] = st_fx->old_coeffs_fx[i];
                        move32();
                    }
                }
            }
        }
    }
    ELSE /* st_fx->nbLostCmpt > 1 */
    {
        test();
        IF( sub(energy_diff_fx, 1024) < 0 && is_transient == 0 )
        {
            num_pgf = 4;
            move16();
        }
        ELSE
        {
            num_pgf = 2;
            move16();
        }

        Num_sb_bwe = num_Sb;
        move16();
        core_end_freq = output_frame;
        move16();

        IF ( sub(st_fx->nbLostCmpt, 2) == 0 )
        {

            FOR ( i=0; i<Num_sb_bwe; i++ )
            {
                norm_values_fx = &ynrm_values_fx[i][0];  /*Q12*/
                r_p_fx = &r_p_values_fx[i][0];      /*Q5*/
                Regression_Anal_fx(norm_values_fx, r_p_fx, num_pgf);
            }
        }

        /* Fade-out Norm by the result of Regression */
        IF( sub(st_fx->nbLostCmpt, mute_start) >= 0 )
        {
            /* Scaling */
            FOR ( i=0; i < output_frame; i++ )
            {
                st_fx->old_coeffs_fx[i] = Mult_32_16(st_fx->old_coeffs_fx[i], 23170);
                move32();
            }
        }

        k = 0;
        move16();
        FOR ( i=0; i<Num_sb_bwe; i++)
        {
            norm_values_fx = &ynrm_values_fx[i][0];
            r_p_fx =  &r_p_values_fx[i][0];

            /* Predict the average energy of each sub-band using Regression */
            /* Linear Regression */
            IF ( r_p_fx[1] > 0 )
            {
                r_p_fx[1] = L_deposit_l(0);
                norm_p_fx[i] = L_shr(norm_values_fx[0], 7);/*5*/  move32();
            }
            ELSE
            {
                norm_p_fx[i] = L_add(r_p_fx[0], L_mult0(extract_l(r_p_fx[1]), sub(add(st_fx->nbLostCmpt, num_pgf), 1)));
            }

            test();
            IF ( norm_values_fx[0] != 0 && norm_p_fx[i] > 0 ) /* Avoid negative value of the predicted energy */
            {
                exp1 = sub(norm_l(norm_p_fx[i]), 1);
                exp2 = norm_l(norm_values_fx[0]);
                tmp_fx = div_s(extract_h(L_shl(norm_p_fx[i], exp1)),
                               extract_h(L_shl(norm_values_fx[0], exp2)));/*15 + (5 + exp1 - 16) - (12 + exp2 - 16)*/
                tmp_fx = shl(tmp_fx, add(6, sub(exp2, exp1)));/*14*/

                if (sub(tmp_fx, 16384) > 0)
                {
                    tmp_fx = 16384;
                    move16();
                }

                FOR ( j=0; j<Num_bands_p[i]; j++ )
                {
                    st_fx->Norm_gain_fx[k++] = shl(tmp_fx, 1);
                    move16();
                }
            }
            ELSE
            {
                /* Scale down the last gain with the fixed gain(-3dB) */
                FOR (j=0; j<Num_bands_p[i]; j++)
                {
                    st_fx->Norm_gain_fx[k] = mult_r(st_fx->Norm_gain_fx[k], 23170);
                    move16();
                    k++;
                }
            }
        }

        /* Scaling for core band */
        FEC_scaling_fx( st_fx->old_coeffs_fx, t_audio_q_fx, st_fx->Norm_gain_fx, &st_fx->HQ_FEC_seed_fx, nb_sfm, sfm_start, sfm_end,
                        0, core_end_freq );

    }


    return;
}

void HQ_FEC_Mem_update_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure            */
    Word32 *t_audio_q_fx,            /*Q12*/
    Word32 *normq_fx,              /*Q14*/
    Word16 *ynrm,
    Word16 *Num_bands_p,
    Word16 is_transient,
    Word16 hqswb_clas,
    Word16 c_switching_flag,
    Word16 nb_sfm,
    Word16 num_Sb,
    Word16 *mean_en_high_fx,          /*Q5*/
    Word16 hq_core_type,                    /* i : normal or low-rate MDCT(HQ) core */
    Word16 output_frame
)
{
    Word16 Min_ind;
    Word32 Min_value;
    Word16 Max_ind;
    Word16 stat_mode_curr;

    Word16 i, j, k;
    Word16 offset;
    Word16 exp, exp1, exp2, tmp_fx;
    Word32* norm_values_fx;
    Word32 L_tmp, tmp_energy_fx = 0, Max_coeff_fx;
    Word32 en_high_fx[MAX_SB_NB];


    IF (is_transient)
    {
        set16_fx(st_fx->prev_sign_switch_2_fx, 0, HQ_FEC_SIGN_SFM);
        set16_fx(st_fx->prev_sign_switch_fx, 0, HQ_FEC_SIGN_SFM);
    }
    ELSE
    {
        FOR (j = 0; j < HQ_FEC_SIGN_SFM; j++)
        {
            st_fx->prev_sign_switch_fx[j] = st_fx->prev_sign_switch_2_fx[j];
            move16();
            st_fx->prev_sign_switch_2_fx[j] = 0;
            move16();

            FOR (i = 0; i < HQ_FEC_BAND_SIZE; i++)
            {
                test();
                test();
                test();
                IF ((st_fx->old_coeffs_fx[i+j*HQ_FEC_BAND_SIZE]>0 && t_audio_q_fx[i+j*HQ_FEC_BAND_SIZE]<0)
                || (st_fx->old_coeffs_fx[i+j*HQ_FEC_BAND_SIZE]<0 && t_audio_q_fx[i+j*HQ_FEC_BAND_SIZE]>0))
                {
                    st_fx->prev_sign_switch_fx[j] = add(st_fx->prev_sign_switch_fx[j], 1);
                    move16();
                    st_fx->prev_sign_switch_2_fx[j] = add(st_fx->prev_sign_switch_2_fx[j], 1);
                    move16();
                }
            }
        }
    }

    IF(sub(output_frame, L_FRAME8k) == 0)
    {
        /* if LR MDCT core is used, recalculate norms from decoded MDCT spectrum (using code from hq_hr_enc_fx()) */
        test();
        IF ( ( sub(hqswb_clas, HQ_HVQ) == 0 ) || ( sub(hq_core_type, LOW_RATE_HQ_CORE) == 0 ) )
        {
            /* First group */
            logqnorm_fx(t_audio_q_fx, 12, ynrm, 32, WID_G1, (hqswb_clas == HQ_HVQ));
            j = ynrm[0];
            move16();
            offset = WID_G1;
            move16();

            FOR ( i=1; i<SFM_G1; i++ )
            {
                logqnorm_fx(&t_audio_q_fx[offset], 12, &ynrm[i], 40, WID_G1, (hqswb_clas == HQ_HVQ));
                offset += WID_G1;
                move16();
            }

            /* Second group */
            FOR ( i=SFM_G1; i<SFM_G1+2; i++ )
            {
                logqnorm_fx(&t_audio_q_fx[offset], 12, &ynrm[i], 40, WID_G2, (hqswb_clas == HQ_HVQ));
                offset += WID_G2;
                move16();
            }
        }

        /* Memory update for the LGF log2 Norm */
        FOR (i = 0; i < nb_sfm; i++)
        {
            normq_fx[i] = dicn_fx[ynrm[i]];
            move32();
        }
        k=0;
        move16();
        FOR (i=0; i<num_Sb; i++)
        {
            norm_values_fx = &st_fx->ynrm_values_fx[i][0];
            Copy32(norm_values_fx, &norm_values_fx[1], MAX_PGF-1);

            L_tmp = L_deposit_l(0);
            FOR (j=0; j<Num_bands_p[i]; j++)
            {
                L_tmp = L_add(L_tmp, L_shr(normq_fx[k++], 3));/*11*/
            }
            tmp_fx = shl(inv_tbl_fx[Num_bands_p[i]], 1);/*16*/
            norm_values_fx[0] = Mult_32_16(L_tmp, tmp_fx);/*11 + 16 - 15*/  move32();
            tmp_energy_fx = L_add(tmp_energy_fx, L_shr(L_tmp, 3));/*8*/
        }
        test();
        test();
        IF((c_switching_flag)||((sub(st_fx->last_core_fx, ACELP_CORE) == 0)&&(sub(st_fx->core_fx, HQ_CORE) == 0)))
        {
            FOR (i=0; i<MAX_SB_NB; i++)
            {
                FOR (j=1; j<MAX_PGF; j++)
                {
                    st_fx->ynrm_values_fx[i][j]=st_fx->ynrm_values_fx[i][0];
                    move32();
                }
            }
        }
        set16_fx(st_fx->Norm_gain_fx, 32767, NB_SFM);/*15*/
        /* st->energy_MA_Curr[1]=Energy of the current frame */
        tmp_fx = inv_tbl_fx[nb_sfm];
        move16();/*15*/
        L_tmp = Mult_32_16(tmp_energy_fx, tmp_fx);/*8 + 15 - 15*/

        st_fx->energy_MA_Curr_fx[1] = extract_h(L_shl(L_tmp, 16-8));
        /* Moving Average */
        st_fx->energy_MA_Curr_fx[0] = max(1,add(mult_r(26214, st_fx->energy_MA_Curr_fx[0]), mult_r(6554, st_fx->energy_MA_Curr_fx[1])));

        /*st->diff_energy = (float)fabs((st->energy_MA_Curr[1] - st->energy_MA_Curr[0])/st->energy_MA_Curr[0]); */
        st_fx->diff_energy_fx = abs_s(sub(st_fx->energy_MA_Curr_fx[1], st_fx->energy_MA_Curr_fx[0]));
        exp1 = sub(norm_l(st_fx->diff_energy_fx), 1);
        exp2 = norm_l(st_fx->energy_MA_Curr_fx[0]);
        st_fx->diff_energy_fx = div_s(extract_h(L_shl(st_fx->diff_energy_fx, exp1)), extract_h(L_shl(st_fx->energy_MA_Curr_fx[0], exp2)));
        exp = add(15, sub(exp1, exp2));
        st_fx->diff_energy_fx = shl(st_fx->diff_energy_fx, sub(11, exp));/*11*/

        /* Classify the stationary mode : 12%  */
        IF (sub(st_fx->diff_energy_fx, ED_THRES_12P_fx) < 0)
        {
            stat_mode_curr = 1;
            move16();
        }
        ELSE
        {
            stat_mode_curr = 0;
            move16();
        }

        /* Apply Hysteresis to prevent frequent mode changing */
        IF(sub(st_fx->stat_mode_old_fx, stat_mode_curr) == 0)
        {
            st_fx->stat_mode_out_fx = stat_mode_curr;
            move16();
        }

        st_fx->stat_mode_old_fx = stat_mode_curr;
        move16();

        /* Find max. band index (Minimum value means maximum energy) */
        Min_ind=0;
        move16();
        Min_value = L_deposit_l(100);
        FOR (i=0; i<num_Sb; i++)
        {
            IF(L_sub(Min_value, ynrm[i]) > 0)
            {
                Min_value = ynrm[i];
                move16();
                Min_ind = i;
                move16();
            }
        }

        /* Find max. coeff in band 0 */
        Max_ind = 0;
        move16();
        IF (Min_ind == 0)
        {
            Max_coeff_fx = L_deposit_l(0);
            FOR (i=0; i<8; i++)
            {
                L_tmp = L_abs(t_audio_q_fx[i]);
                IF (L_sub(Max_coeff_fx, L_tmp) < 0)
                {
                    Max_coeff_fx = L_add(L_tmp, 0);
                    Max_ind = i;
                    move16();
                }
            }
        }

        /* Find energy difference from band 16 */
        k=1;
        move16();

        FOR (i=k; i<num_Sb; i++)
        {
            en_high_fx[i] = L_deposit_l(0);
            FOR (j=0; j<2; j++)
            {
                /*en_high[i] += 0.5f*st->ynrm_values[i][j+1];*/
                en_high_fx[i] = L_add(en_high_fx[i], L_shr(st_fx->ynrm_values_fx[i][j+1], 1));/*Q12*/ move32();
            }
        }

        *mean_en_high_fx = 0;
        move16();
        FOR (i=k; i<num_Sb; i++)
        {
            /* *mean_en_high += (float)(en_high[i]/st->ynrm_values[i][0]);*/
            exp1 = sub(norm_l(en_high_fx[i]), 1);
            exp2 = norm_l(st_fx->ynrm_values_fx[i][0]);
            tmp_fx = div_s(extract_h(L_shl(en_high_fx[i], exp1)), extract_h(L_shl(st_fx->ynrm_values_fx[i][0], exp2)));
            exp = add(15, sub(exp1, exp2));
            *mean_en_high_fx = add(*mean_en_high_fx, shr(tmp_fx, sub(exp, 5)));
        }
        *mean_en_high_fx = mult(*mean_en_high_fx, inv_tbl_fx[num_Sb-k]);

        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF ((sub(Min_ind, 5) < 0) && (sub(abs_s(sub(Min_ind, st_fx->old_Min_ind_fx)), 2) < 0) &&(sub(st_fx->diff_energy_fx, ED_THRES_90P_fx) < 0)&&(!st_fx->bfi_fx) && (!st_fx->prev_bfi_fx)&&(!st_fx->prev_old_bfi_fx)
            &&(!is_transient)&&(!st_fx->old_is_transient_fx[1]))
        {
            st_fx->phase_mat_flag_fx = 1;
            move16();
            test();
            if ((Min_ind == 0)&&(sub(Max_ind, 3) < 0))
            {
                st_fx->phase_mat_flag_fx = 0;
                move16();
            }
        }
        ELSE
        {
            st_fx->phase_mat_flag_fx = 0;
            move16();
        }

        st_fx->old_Min_ind_fx = Min_ind;
        move16();
    }

    FOR (i=0; i < L_FRAME8k; i++)
    {
        st_fx->old_coeffs_fx[i] = t_audio_q_fx[i];
        move32();
    }

    st_fx->old_is_transient_fx[2] = st_fx->old_is_transient_fx[1];
    move16();
    st_fx->old_is_transient_fx[1] = st_fx->old_is_transient_fx[0];
    move16();
    st_fx->old_is_transient_fx[0] = is_transient;
    move16();

    return;
}


static Word16 find_best_delay_fx(
    Word16 *mu_o_fx,
    Word16 *in_fx,
    Word16 mind1,
    Word16 maxd1,
    Word16 lin,
    Word16 delta,
    Word16 *false_flag
)
{
    Word16 i, d1, k;
    Word16 d1m = 0;

    Word16 tmp, exp1, exp2;
    Word32 L_tmp1, L_tmp2;

    Word32 min_sq_cross_fx, min_corr_fx;
    Word32 accA_fx, accB_fx;
    Word32 Rxy_fx[MAXDELAY_FEC], Ryy_fx[MAXDELAY_FEC];

    d1 = mind1;
    move16();
    FOR ( k = 0; k < (maxd1-mind1)/delta; k++ )
    {
        accA_fx = L_deposit_l(0);
        accB_fx = L_deposit_l(0);
        FOR ( i = 0; i < lin; i += delta )
        {
            accA_fx = L_add(accA_fx, L_shr(L_mult(mu_o_fx[d1+i], mu_o_fx[d1+i]),2));
            accB_fx = L_add(accB_fx, L_shr(L_mult(mu_o_fx[d1+i], in_fx[i]),2));
        }

        Rxy_fx[k] = accB_fx;
        move32();
        Ryy_fx[k] = accA_fx;
        move32();

        d1 = add(d1, delta);
    }

    /*  Obtain the best delay values */
    min_sq_cross_fx = 0x80000000;
    move32();
    min_corr_fx = L_deposit_l(0);
    exp2 = 0;
    move16();
    FOR ( d1 = 0; d1 < (maxd1-mind1)/delta; d1++ )
    {
        IF (L_abs(Rxy_fx[d1]) > L_abs(Ryy_fx[d1]))
        exp1 = norm_l(Rxy_fx[d1]);
        ELSE
        exp1 = norm_l(Ryy_fx[d1]);

        L_tmp1 = Mult_32_32(L_shl(Rxy_fx[d1], exp1), L_shl(min_corr_fx, exp2));
        L_tmp2 = Mult_32_32(L_shl(Ryy_fx[d1], exp1), L_shl(min_sq_cross_fx, exp2));
        IF ( L_sub(L_tmp1, L_tmp2) >= 0)
        {
            d1m = d1;
            move16();
            min_corr_fx = L_add(Ryy_fx[d1], 0);    /*12 + 12 - 31 */
            min_sq_cross_fx = L_add(Rxy_fx[d1], 0);  /*12 + 12 - 31 */
            exp2 = exp1;
            move16();
        }
    }

    test();
    IF((min_sq_cross_fx<=0) || (min_corr_fx<=0))
    {
        tmp = 0;
        move16(); /* If cross correlation is negative, the division per the energy will always be negative --> tmp will be < 8192, no need to do the division per say */
    }
    ELSE
    {
        /*d1m *= delta; */
        d1m = extract_l(L_mult0(d1m, delta));

        exp1 = sub(norm_l(min_sq_cross_fx), 1);
        exp2 = norm_l(min_corr_fx);
        L_tmp1 = L_shl(min_sq_cross_fx, exp1);
        L_tmp2 = L_shl(min_corr_fx, exp2);
        tmp = div_s(extract_h(L_tmp1), extract_h(L_tmp2));/*15 + exp1 - exp2 */
        tmp = shl(tmp, sub(exp2, add(exp1, 1)));/*14 */
    }

    *false_flag = 0;
    move16();
    test();
    if (sub(tmp, 8192) < 0 || sub(tmp, 24576) > 0)
    {
        *false_flag = 1;
        move16();
    }

    return d1m;

}

static Word16 Search_Max_Corr_fx(
    Word16 *mu_o_fx,            /* i : *old_auOut_2fr,  */
    Word16 old_Min_ind,          /* i : *old_auOut_2fr,  */
    const Word16 L               /* i : L/2            */
)
{
    Word16 pos;
    Word16 pos2,delta2;
    Word16 lin, delta;
    Word16 mind1,maxd1;
    Word16 false_flag;
    Word16 min_d1, max_d1;
    Word16 tmp1, tmp2;
    Word16 *in_fx;

    IF (old_Min_ind == 0)
    {
        /*lin = 8*L/20;               */ /* Basic size of the block for phase matching */
        lin = mult_r(L, 13107);      /* Basic size of the block for phase matching */
        /*min_dist = -1e35f; */
        mind1 = 0;
        move16();              /* min value of delay d1 to search for */
        /*maxd1 = 12*L/20;            */ /* max value of delay d1 to search for */
        maxd1 = mult_r(L, 19661);    /* max value of delay d1 to search for */

        /*in = mu_o + 2*L -lin;     */
        in_fx = mu_o_fx + 2*L - lin;
        move16();

        /* generate correlation */
        delta = 2;
        move16();
        delta2 = 1;
        move16();

        pos = find_best_delay_fx(mu_o_fx, in_fx, mind1, maxd1, lin, delta, &false_flag);

        IF (false_flag)
        {
            return 0;
        }

        tmp1 = add(mind1, sub(add(pos, 1), delta));
        tmp2 = add(mind1, add(pos, delta));
        min_d1 = s_max(mind1, tmp1);
        max_d1 = s_min(maxd1, tmp2);
        pos2 = find_best_delay_fx(mu_o_fx, in_fx, min_d1, max_d1, lin, delta2, &false_flag);
        IF (sub(mind1, tmp1) > 0)
        {
            pos = pos2;
            move16();
        }
        ELSE
        {
            pos = add(pos, add(sub(pos2, delta), 1));
        }

        pos = add(pos, add(lin, mind1));
    }
    ELSE
    {
        lin = mult_r(L, 9830);
        mind1 = mult_r(L, 14746);        /* min value of delay d1 to search for */
        maxd1 = mult_r(L, 22938);     /* max value of delay d1 to search for */
        in_fx = mu_o_fx + 2*L - lin;
        move16();

        /* generate correlation */
        delta = 2;
        move16();
        delta2 = 1;
        move16();

        pos = find_best_delay_fx(mu_o_fx, in_fx, mind1, maxd1, lin, delta, &false_flag);

        IF (false_flag)
        {
            return 0;
        }

        tmp1 = add(mind1, add(sub(pos, delta), 1));
        tmp2 = add(mind1, add(pos, delta));
        min_d1 = s_max(mind1, tmp1);
        max_d1 = s_min(maxd1, tmp2);
        pos2 = find_best_delay_fx(mu_o_fx, in_fx, min_d1, max_d1, lin, delta2, &false_flag);

        IF (sub(mind1, tmp1) > 0 )
        {
            pos = pos2;
            move16();
        }
        ELSE
        {
            pos = add(pos, add(sub(pos2, delta), 1));
        }
        pos = add(pos, add(lin, mind1));
    }

    return pos;
}

Word16 FEC_phase_matching_fx(
    Decoder_State_fx *st_fx,                    /* i  : Decoder State                           */
    Word32 *ImdctOut_fx,                        /* i  : input                                   */
    Word16 *auOut_fx,                           /* o  : output audio                            */
    Word16 *OldauOut_fx,
    Word16 OldauOut_pha_fx[2][N_LEAD_NB]
)
{
    Word16 i;
    Word16 pos, remain;
    Word16 ol_size;
    Word16 L_overlap, L;
    Word16 ImdctOutWin_fx[2*L_FRAME8k];
    Word16 OldauOutnoWin_fx[L_FRAME8k];
    Word16 OldauOut2_fx[L_FRAME8k];
    Word16 win_NB_fx[L_FRAME8k + 25];
    Word16 exp1, exp2, tmp;
    Word32 pow1_fx, pow2_fx;
    Word16 SmoothingWin_NB3_fx[24];

    L = L_FRAME8k;
    move16();
    ol_size = 2*L/20;
    move16();
    L_overlap = 3*L/20;
    move16();

    FOR (i=0; i<L_overlap; i++)
    {
        SmoothingWin_NB3_fx[i] = SmoothingWin_NB875_fx[i*3];
        move16();
    }

    FOR (i=0; i<L + 25; i++)
    {
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }
    set16_fx(ImdctOutWin_fx, 0, 2*L);

    pos = Search_Max_Corr_fx(st_fx->old_auOut_2fr_fx, st_fx->old_Min_ind_fx, L);

    IF (pos == 0)
    {
        return 1;
    }

    /* Repetition */
    remain = L+N_Z_L_NB - ((2*L)-pos);
    move16();
    Copy(&st_fx->old_auOut_2fr_fx[pos], &ImdctOutWin_fx[N_ZERO_NB], (2*L)-pos);

    /* OldauOut without windowing */
    FOR (i = N_ZERO_NB; i < L/2; i++)
    {
        OldauOutnoWin_fx[i-N_ZERO_NB] = extract_l( L_shr( L_negate(st_fx->oldIMDCTout_fx[L/2 - 1 - i]), 6 ) );
    }
    FOR(i = 0; i < L/2; i++)
    {
        OldauOutnoWin_fx[i+N_ZERO_O_NB] = extract_l( L_shr( L_negate(st_fx->oldIMDCTout_fx[i]), 6 ) );
    }

    Copy(OldauOutnoWin_fx, &ImdctOutWin_fx[N_ZERO_NB+(2*L)-pos], remain);

    pow1_fx = L_deposit_l(0);
    pow2_fx = L_deposit_l(0);
    FOR (i = 0; i < L; i++)
    {
        pow1_fx = L_add(pow1_fx, shr(abs_s(st_fx->old_auOut_2fr_fx[L+i]), 1));
        pow2_fx = L_add(pow2_fx, shr(abs_s(ImdctOutWin_fx[N_ZERO_NB+i]), 1));
    }

    exp1 = sub(norm_l(pow1_fx), 1);
    exp2 = norm_l(pow2_fx);
    tmp = div_s(extract_h(L_shl(pow1_fx, exp1)), extract_h(L_shl(pow2_fx, exp2)));/*15 + exp1 - exp2*/
    tmp = shl(tmp, sub(sub(exp2, exp1), 1));/*14*/
    FOR (i = N_ZERO_NB; i < 2*L; i++)
    {
        ImdctOutWin_fx[i] = shl(mult(ImdctOutWin_fx[i], tmp), 1);
    }
    Smoothing_vector_NB_fx(OldauOutnoWin_fx, &ImdctOutWin_fx[N_ZERO_NB], SmoothingWin_NB2_fx, auOut_fx, ol_size);

    FOR (i = 0; i < L/2; i++)
    {
        /*ImdctOutWin[3*L/2 + i] *= win_NB[L/2-i-1];*/
        ImdctOutWin_fx[3*L/2 + i] = mult(ImdctOutWin_fx[3*L/2 + i], win_NB_fx[L/2-i-1]);
        move16();
    }

    FOR (i = N_ZERO_NB; i < L/2; i++)
    {
        /*ImdctOutWin_fx[L + i] *= win_NB_fx[(L-1-i)];*/
        ImdctOutWin_fx[L + i] = mult(ImdctOutWin_fx[L + i], win_NB_fx[(L-1-i)]);
        move16();
    }

    Copy(&ImdctOutWin_fx[N_Z_L_O_NB], &OldauOut_pha_fx[0][0], N_LEAD_NB);
    Copy(&ImdctOutWin_fx[ol_size+N_ZERO_NB], &auOut_fx[ol_size], N_Z_L_NB-ol_size);
    Copy(&ImdctOutWin_fx[L], &auOut_fx[N_Z_L_NB], N_ZERO_NB);
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);

    FOR (i = 0; i < L/2; i++)
    {
        OldauOut2_fx[i] = extract_l( L_shr( L_negate(ImdctOut_fx[L/2 - 1 - i]), 6 ) );
        OldauOut2_fx[L/2+i] = extract_l( L_shr( L_negate(ImdctOut_fx[i]), 6 ) );
    }

    Smoothing_vector_NB_fx(&ImdctOutWin_fx[N_Z_L_O_NB], &OldauOut2_fx[N_ZERO_NB], SmoothingWin_NB3_fx, &OldauOut_pha_fx[1][0], L_overlap);

    FOR (i=L_overlap; i<N_LEAD_NB; i++)
    {
        OldauOut_pha_fx[1][i] = OldauOut2_fx[i+N_ZERO_NB];
        move32();
    }

    return 0;
}

void FEC_phase_matching_nextgood_fx(
    const Word32 *ImdctOut_fx,                /* i  : input                           */
    Word16 *auOut_fx,                         /* o  : output audio                    */
    Word16 *OldauOut_fx,                      /* i/o: audio from previous frame       */
    Word16 OldauOut_pha_fx[2][N_LEAD_NB],
    Word16 mean_en_high_fx                    /*Q5 */
)
{
    Word16 i;
    Word16 L_overlap, L;
    Word16 oldout_pha_idx;
    Word16 *OldOut_pha_fx;
    Word16 ImdctOutWin_fx[2*L_FRAME48k];
    Word16 win_NB_fx[L_FRAME8k + 25];

    L = L_FRAME8k;
    move16();
    FOR (i=0; i<L + 25; i++)
    {
        /* win_NB[i] = window_48kHz[i*6+3]; */
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }

    test();
    IF ((sub(mean_en_high_fx, 64) > 0)||(sub(mean_en_high_fx, 16) < 0))
    {
        oldout_pha_idx = 1;
        move16();
    }
    ELSE
    {
        oldout_pha_idx = 0;
        move16();
    }

    /* Overlapping with next good frame : Overlapping to remove the discontinuity */
    L_overlap = N_LEAD_NB;
    move16();
    OldOut_pha_fx = OldauOut_pha_fx[oldout_pha_idx];
    FOR (i = 0; i < N_LEAD_NB; i++)
    {
        /* OldOut_pha[i] *= SmoothingWin_NB875[L_overlap-i-1]; */
        OldOut_pha_fx[i] = mult(OldOut_pha_fx[i], SmoothingWin_NB875_fx[L_overlap-i-1]);
        move16();
    }

    IF (sub(oldout_pha_idx, 1) == 0)
    {
        /* Use phase matching and overlapping with the Oldauout*/
        /* Windowing */
        /*Windowing_1st_NB(ImdctOutWin, ImdctOut, win_NB, NULL, 0);*/
        /*Windowing_2nd_NB(ImdctOutWin, ImdctOut, win_NB); */
        Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, NULL, 0);
        Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);
    }
    ELSE
    {
        /* Only use phase matching */
        /* Windowing */
        Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, SmoothingWin_NB875_fx, 1);
        Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);
    }
    /* common_overlapping(auOut, ImdctOutWin, OldOut_pha, N_LEAD_NB, 0, N_LEAD_NB, L, N_ZERO_NB, 0, L);*/
    common_overlapping_fx(auOut_fx, ImdctOutWin_fx, OldOut_pha_fx, N_LEAD_NB, 0, N_LEAD_NB, L, N_ZERO_NB, 0);
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);

    return;
}

void FEC_phase_matching_burst_fx(
    const Word32 *ImdctOut_fx,              /* i  : input                           */
    Word16 *auOut_fx,                       /* o  : output audio                    */
    Word16 *OldauOut_fx,                    /* i/o: audio from previous frame       */
    Word16 OldauOut_pha_fx[2][N_LEAD_NB],
    Word16 *prev_oldauOut_fx                /* i : OldauOut from previous frame     */
)
{
    Word16 i;
    Word16 L_overlap;
    Word16 L;
    Word16 OldauOut2_fx[L_FRAME8k];
    Word16 ImdctOutWin_fx[2*L_FRAME8k];
    Word16 win_NB_fx[L_FRAME8k + 25];
    Word16 SmoothingWin_NB3_fx[24];

    L = L_FRAME8k;
    move16();
    L_overlap = 3*L/20;
    move16();

    FOR (i=0; i<L_overlap; i++)
    {
        SmoothingWin_NB3_fx[i] = SmoothingWin_NB875_fx[i*3];
        move16();
    }

    FOR (i=0; i<L + 25; i++)
    {
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }

    /* Windowing */
    Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, NULL, 0);
    Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);

    /* Repetition with old frame to reserve energy */
    /*common_overlapping(auOut, ImdctOutWin, prev_oldauOut, N_Z_L_NB, 0, N_Z_L_NB, L, N_ZERO_NB, 0, L);*/
    common_overlapping_fx(auOut_fx, ImdctOutWin_fx, prev_oldauOut_fx, N_Z_L_NB, 0, N_Z_L_NB, L, N_ZERO_NB, 0);

    /* data transition from OldauOut to auOut using smoothing win*/
    Smoothing_vector_NB_fx(OldauOut_pha_fx[0], auOut_fx, SmoothingWin_NB875_fx, auOut_fx, N_LEAD_NB);

    /* Update the OldauOut array for next overlapping */
    Copy(&ImdctOutWin_fx[N_Z_L_O_NB], &OldauOut_pha_fx[0][0], N_LEAD_NB);
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);
    Scaledown_fx(prev_oldauOut_fx, prev_oldauOut_fx, 23170, L);

    FOR (i = 0; i < L/2; i++)
    {
        /* OldauOut2[i] = -ImdctOut[L/2 - 1 - i];*/
        /* OldauOut2[L/2+i] = -ImdctOut[i];*/
        OldauOut2_fx[i] = extract_l( L_shr( L_negate(ImdctOut_fx[L/2 - 1 - i]), 6 ) );
        OldauOut2_fx[L/2+i] = extract_l( L_shr( L_negate(ImdctOut_fx[i]), 6 ) );
    }

    Smoothing_vector_NB_fx(&ImdctOutWin_fx[N_Z_L_O_NB], &OldauOut2_fx[N_ZERO_NB], SmoothingWin_NB3_fx, &OldauOut_pha_fx[1][0], L_overlap);

    FOR (i=L_overlap; i<N_LEAD_NB; i++)
    {
        /* OldauOut_pha[1][i] = OldauOut2[i+N_ZERO_NB]; */
        OldauOut_pha_fx[1][i] = OldauOut2_fx[i+N_ZERO_NB];
        move16();
    }

    return;
}

void Repetition_smoothing_nextgood_fx(
    const Word32 *ImdctOut_fx,          /* i  : input                           */
    Word16 *auOut_fx,                   /* o  : output audio                    */
    Word32 *OldImdctOut_fx,             /* i  : input                           */
    Word16 *OldauOut_fx,                /* i/o: audio from previous frame       */
    Word16 cur_data_use_flag,           /* i  : current imdct data use flag     */
    Word16 overlap_time
)
{
    Word16 i;
    Word16 L_overlap;
    Word16 ol_size;
    Word16 L;
    Word16 ImdctOutWin_fx[2*L_FRAME8k];
    Word16 win_NB_fx[L_FRAME8k + 25];

    L = L_FRAME8k;
    move16();

    FOR (i=0; i<L_FRAME8k + 25; i++)
    {
        /*win_NB[i] = window_48kHz[i*6+3];*/
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }

    FOR (i = N_ZERO_NB; i < L/2; i++)
    {
        /*OldauOut[i-N_ZERO_NB] = -OldImdctOut[L/2 - 1 - i];*/
        OldauOut_fx[i-N_ZERO_NB] = extract_l( L_shr( L_negate(OldImdctOut_fx[L/2 - 1 - i]), 6 )); /* Q6 -> Q0 */
    }
    FOR (i = 0; i < L/2; i++)
    {
        /*OldauOut[i+N_ZERO_O_NB] = -OldImdctOut[i];*/
        OldauOut_fx[i+N_ZERO_O_NB] = extract_l( L_shr( L_negate(OldImdctOut_fx[i]), 6 )); /* Q6 -> Q0 */
    }

    /* Overlapping with next good frame : Overlapping to remove the discontinuity */
    IF (cur_data_use_flag)
    {
        ol_size = N_LEAD_NB;
        move16();

        FOR (i = N_ZERO_NB; i < L/2; i++)
        {
            /* ImdctOutWin[i+L] = -ImdctOut[L/2 - 1 - i]; */
            ImdctOutWin_fx[i+L] = extract_l( L_shr( L_negate(ImdctOut_fx[L/2 - 1 - i]), 6 ) );
        }
        FOR (i = 0; i < L/2; i++)
        {
            ImdctOutWin_fx[i+3*L/2] = extract_l( L_shr( L_negate(ImdctOut_fx[i]), 6 ) );
        }

        /*a = (float)(1./(float)(ol_size)); y = ax */
        Smoothing_vector_scaledown_NB_fx(OldauOut_fx, &ImdctOutWin_fx[N_Z_L_O_NB], SmoothingWin_NB875_fx, OldauOut_fx, ol_size);

        /* Scale down the overlapped signal */
        Scaledown_fx(&ImdctOutWin_fx[ol_size+N_Z_L_O_NB], &OldauOut_fx[ol_size], 23170, N_Z_L_NB-ol_size);
    }

    L_overlap = overlap_time;
    move16();
    FOR (i = 0; i < L_overlap; i++)
    {
        /*OldauOut[i] *= SmoothingWin_NB875[L_overlap-i-1];*/
        OldauOut_fx[i] = mult(OldauOut_fx[i], SmoothingWin_NB875_fx[L_overlap-i-1]);
        move16();
    }
    FOR (i=L_overlap; i < L; i++)
    {
        OldauOut_fx[i] = 0;
        move16();
    }

    /* Windowing */
    /*Windowing_1st_NB(ImdctOutWin, ImdctOut, win_NB, SmoothingWin_NB875, 1);*/
    /*Windowing_2nd_NB(ImdctOutWin, ImdctOut, win_NB);*/
    Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, SmoothingWin_NB875_fx, 1);
    Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);

    /*v_add(&ImdctOutWin[N_ZERO_NB], OldauOut, auOut, L);*/
    /*mvr2r(&ImdctOutWin[L], OldauOut, L);*/
    FOR (i = 0; i < L; i++)
    {
        auOut_fx[i] = add(ImdctOutWin_fx[N_ZERO_NB + i], OldauOut_fx[i]);
        move16();
    }
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);

    return;
}

Word16 Repetition_smoothing_fx(
    const Word32 *ImdctOut_fx,        /* i  : input                           */
    Word16 *auOut_fx,                 /* o  : output audio                    */
    Word32 *OldImdctOut_fx,           /* i  : input                           */
    Word16 *OldauOut_fx,              /* i/o: audio from previous frame       */
    const Word16 L,                /* i  : length                          */
    Word16 *prev_oldauOut_fx,         /* i : OldauOut from previous frame     */
    Word16 overlap_time            /* i : overlap time                     */
)
{
    Word16 i;
    Word32 pow1_fx;
    Word32 pow2_fx;
    Word16 ImdctOutWin_fx[2*L_FRAME8k];
    Word16 OldauOutnoWin_fx[L_FRAME8k];
    Word16 win_NB_fx[L_FRAME8k + 25];

    FOR (i=0; i<L_FRAME8k + 25; i++)
    {
        /*win_NB[i] = window_48kHz[i*6+3];*/
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }

    /* Windowing */
    Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, NULL, 0);
    Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);

    /* Repetition with old frame to reserve energy */
    common_overlapping_fx( auOut_fx, ImdctOutWin_fx, prev_oldauOut_fx, N_Z_L_NB, 0, N_Z_L_NB, L, N_ZERO_NB, 0 );

    /* OldauOut without windowing */
    FOR (i = N_ZERO_NB; i < L/2; i++)
    {
        OldauOutnoWin_fx[i-N_ZERO_NB] = extract_l( L_shr( L_negate(OldImdctOut_fx[L/2 - 1 - i]), 6 ) );
    }
    FOR (i = 0; i < L/2; i++)
    {
        OldauOutnoWin_fx[i+N_ZERO_O_NB] = extract_l( L_shr( L_negate(OldImdctOut_fx[i]), 6 ) );
    }

    /* data transition from OldauOut to auOut using smoothing win*/
    Smoothing_vector_NB_fx(OldauOutnoWin_fx, auOut_fx, SmoothingWin_NB875_fx, auOut_fx, overlap_time);

    pow1_fx = L_deposit_l(0);
    pow2_fx = L_deposit_l(0);
    FOR (i = 0; i < 4*L/20; i++)
    {
        pow1_fx = L_add(pow1_fx, Mult_32_32(L_shl((Word32)auOut_fx[1*L/20 + i],6), L_shl((Word32)auOut_fx[1*L/20 + i],6)));
        pow2_fx = L_add(pow2_fx, Mult_32_32(L_shl((Word32)auOut_fx[N_LEAD_NB + i],6), L_shl((Word32)auOut_fx[N_LEAD_NB + i],6)));
    }


    IF (pow2_fx > L_add(pow1_fx, L_shl(pow1_fx, 1)))
    {
        return 1;
    }

    /* Update the OldauOut array for next overlapping */
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);
    Scaledown_fx(prev_oldauOut_fx, prev_oldauOut_fx, 23170, L);

    return 0;
}

void common_overlapping_fx(
    Word16 *auOut_fx,           /* i : Input                                   */
    Word16 *ImdctOutWin_fx,     /* o : Output                                  */
    Word16 *OldauOut_fx,        /* i : Window                                  */
    Word16 end1,                            /* i : Decay                                   */
    Word16 offset1,
    Word16 start2,
    Word16 end2,
    Word16 offset_i2,
    Word16 offset2
)
{
    Word16 i;

    /* Common Overlapping */
    FOR (i=0 ; i < end1; i++)
    {
        /*auOut_fx[i] = L_add(ImdctOutWin_fx[i+7*L/20], OldauOut_fx[i+offset1]);*/
        auOut_fx[i] = add(ImdctOutWin_fx[i+N_ZERO_NB], OldauOut_fx[i+offset1]);
        move16();
    }
    FOR (i=start2 ; i < end2; i++)
    {
        /*auOut_fx[i+offset2]  =  ImdctOutWin_fx[i+offset_i2];  move32();*/
        auOut_fx[i+offset2]  =  ImdctOutWin_fx[i+offset_i2];
        move16();
    }

    return;
}


void Smoothing_vector_NB_fx(
    const Word16 OldauOutnoWin_fx[],  /* i  : Input vector 1                                   */
    const Word16 ImdctOutWin_fx[],    /* i  : Input vector 2                                   */
    const Word16 SmoothingWin_fx[],   /* i  : Smoothing window                                   */
    Word16 auOut_fx[],          /* o  : Output vector that contains vector 1 .* vector 2 */
    const Word16 ol_size           /* i  : Overlap size                                    */
)
{
    Word16 i;
    Word16 weight_fx;

    FOR (i=0 ; i < ol_size; i++)
    {
        weight_fx = SmoothingWin_fx[i];
        move16();
        auOut_fx[i] = add(mult(OldauOutnoWin_fx[i], sub(32767, weight_fx)), mult(ImdctOutWin_fx[i], weight_fx));
        move16();
    }

    return;
}


void Windowing_1st_NB_fx(
    Word16 *ImdctOutWin_fx,                      /* o : Output                                  */
    const Word32 *ImdctOut_fx,                   /* i : Input                                   */
    const Word16 *win_fx,                        /* i : Window                                  */
    const Word16 *smoothingWin_fx,               /* i : Smoothing Window                        */
    Word16 smoothing_flag                        /* i : 1=Smoothing window, 0=Original window   */
)
{
    Word16 i;
    Word16 L;

    L = L_FRAME8k;
    move16();
    IF ( smoothing_flag == 0 )
    {
        FOR (i = N_ZERO_NB; i < L/2; i++)
        {
            /*ImdctOutWin[i] = ImdctOut[L/2 + i] * win[(2*L-1-i)-N_LEAD_O_NB];*/
            ImdctOutWin_fx[i] = extract_l( L_shr( Mult_32_16(ImdctOut_fx[L/2 + i], win_fx[(2*L-1-i)-N_LEAD_O_NB]), 6 ));
        }

        FOR (i = 0; i < N_ZERO_O_NB; i++)
        {
            /*ImdctOutWin[L/2 + i] = -ImdctOut[L - 1 - i] * win[(3*L/2-1-i)-N_LEAD_O_NB];*/
            /*ImdctOutWin[3*L/2 + i] = -ImdctOut[i] * win[(L/2-i-1)];*/
            ImdctOutWin_fx[L/2 + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[L - 1 - i]), win_fx[(3*L/2-1-i)-N_LEAD_O_NB]), 6 ));
            ImdctOutWin_fx[3*L/2 + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[i]), win_fx[(L/2-i-1)]), 6 ));
        }
    }
    ELSE
    {
        FOR (i = N_ZERO_NB; i < L/2; i++)
        {
            /*ImdctOutWin[i] = ImdctOut[L/2 + i] * smoothingWin[(i-N_ZERO_NB)];*/ /*win[(2*L-i)*decimate-1-decay-14*L_FRAME48k/20];*/
            ImdctOutWin_fx[i] = extract_l( L_shr( Mult_32_16(ImdctOut_fx[L/2 + i], smoothingWin_fx[(i-N_ZERO_NB)]), 6 )); /*win[(2*L-i)*decimate-1-decay-14*L_FRAME48k/20];*/
        }

        FOR (i = 0; i < N_ZERO_O_NB; i++)
        {
            /*ImdctOutWin[L/2 + i] = -ImdctOut[L - 1 - i] * smoothingWin[(i+N_ZERO_O_NB)];*/   /*win[(3*L/2-1-i)*decimate+decay-L_FRAME48k*14/20];*/
            /*ImdctOutWin[3*L/2 + i] = -ImdctOut[i] * win[(L/2-i-1)];*/
            ImdctOutWin_fx[L/2 + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[L - 1 - i]), smoothingWin_fx[(i+N_ZERO_O_NB)]), 6 ));   /*win[(3*L/2-1-i)*decimate+decay-L_FRAME48k*14/20];*/
            ImdctOutWin_fx[3*L/2 + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[i]), win_fx[(L/2-i-1)]), 6 ));
        }
    }

    return;
}
void Windowing_2nd_NB_fx(
    Word16 *ImdctOutWin_fx,         /* o : Output                   */
    const Word32 *ImdctOut_fx,      /* i : Input                    */
    const Word16 *win_fx           /* i : Window                   */
)
{
    Word16 i;
    Word16 L;

    L = L_FRAME8k;
    move16();
    FOR (i = N_ZERO_O_NB; i < L/2; i++)
    {
        /*ImdctOutWin[L/2 + i] = -ImdctOut[L - 1 - i];*/
        /*ImdctOutWin[3*L/2 + i] = -ImdctOut[i] * win[L/2-i-1];*/
        ImdctOutWin_fx[L/2 + i] = extract_l( L_shr( L_negate(ImdctOut_fx[L - 1 - i]), 6 ));
        ImdctOutWin_fx[3*L/2 + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[i]), win_fx[L/2-i-1]), 6 ));
    }

    FOR (i = 0; i < N_ZERO_NB; i++)
    {
        /*ImdctOutWin[L + i] = -ImdctOut[L/2 - 1 - i];*/
        ImdctOutWin_fx[L + i] = extract_l( L_shr( L_negate(ImdctOut_fx[L/2 - 1 - i]), 6 ));
    }

    FOR (i = N_ZERO_NB; i < L/2; i++)
    {
        /*ImdctOutWin[L + i] = -ImdctOut[L/2 - 1 - i] * win[L - 1 - i];*/
        ImdctOutWin_fx[L + i] = extract_l( L_shr( Mult_32_16(L_negate(ImdctOut_fx[L/2 - 1 - i]), win_fx[L - 1 - i]), 6 ));
    }

    return;
}

void Smoothing_vector_scaledown_NB_fx(
    const Word16 OldauOutnoWin_fx[],  /* i  : Input vector 1                                   */
    const Word16 ImdctOutWin_fx[],    /* i  : Input vector 2                                   */
    const Word16 SmoothingWin_fx[],   /* i  : Smoothing window                                   */
    Word16 auOut_fx[],          /* o  : Output vector that contains vector 1 .* vector 2 */
    const Word16 ol_size           /* i  : Overlap size                                    */

)
{
    Word16 i;
    Word16 weight_fx;

    FOR (i=0 ; i < ol_size; i++)
    {
        weight_fx = SmoothingWin_fx[i];
        move16();
        auOut_fx[i] = add(mult(OldauOutnoWin_fx[i], sub(32767, weight_fx)),
                          mult(ImdctOutWin_fx[i], mult_r(23170, weight_fx)));
        move16();
    }

    return;
}


void Scaledown_fx(
    Word16 x[],                 /* i  : Input vector                                  */
    Word16 y[],                 /* o  : Output vector that contains vector 1 .* vector 2 */
    Word16 scale_v,             /*Q15   */
    const Word16 N                    /* i  : Overlap size                                    */
)
{
    Word16 i;

    FOR (i=0; i<N; i++)
    {
        y[i] = mult(x[i], scale_v);
        move16();
    }

    return;
}

void time_domain_FEC_HQ_fx(
    Decoder_State_fx *st_fx,            /* i : Decoder State                        */
    Word32 *wtda_audio_fx,              /* i : input                                */
    Word16 *out_fx,                     /* o : output audio                         */
    Word16 mean_en_high_fx,             /* i : transient flag                       */
    const Word16 output_frame,
    Word16 *Q_synth
)
{
    IF(st_fx->Q_old_wtda != 0)
    {
        Scale_sig(st_fx->old_out_fx, output_frame, negate(st_fx->Q_old_wtda));
        st_fx->Q_old_wtda = 0;
        move16();
    }

    test();
    test();
    test();
    test();
    test();
    test();
    IF( (sub(st_fx->nbLostCmpt, 1) == 0)&&(sub(st_fx->phase_mat_flag_fx, 1) == 0)&&(sub(st_fx->phase_mat_next_fx, 0) == 0) )
    {
        IF (FEC_phase_matching_fx(st_fx, wtda_audio_fx, out_fx, st_fx->old_out_fx, st_fx->old_out_pha_fx) )
        {
            /* window_ola( wtda_audio, out, st->old_out, output_frame, 0, 0, 0 ); */
            window_ola_fx(wtda_audio_fx, out_fx, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame, ALDO_WINDOW, ALDO_WINDOW, 0, 0, 0 );
            st_fx->phase_mat_next_fx = 0;
            move16();
        }
        ELSE
        {
            st_fx->phase_mat_next_fx = 1;
            move16();
            *Q_synth = 0;
            move16();
        }
    }
    ELSE IF((sub(st_fx->prev_bfi_fx, 1) == 0)&&(st_fx->bfi_fx == 0) &&(sub(st_fx->phase_mat_next_fx, 1) == 0))
    {
        FEC_phase_matching_nextgood_fx( wtda_audio_fx, out_fx, st_fx->old_out_fx, st_fx->old_out_pha_fx, mean_en_high_fx);

        st_fx->phase_mat_next_fx = 0;
        move16();
        *Q_synth = 0;
        move16();
    }
    ELSE IF((sub(st_fx->prev_bfi_fx, 1) == 0)&&(sub(st_fx->bfi_fx, 1) == 0) &&(sub(st_fx->phase_mat_next_fx, 1) == 0))
    {
        FEC_phase_matching_burst_fx( wtda_audio_fx, out_fx, st_fx->old_out_fx, st_fx->old_out_pha_fx, st_fx->prev_oldauOut_fx);
        st_fx->phase_mat_next_fx = 1;
        move16();
        *Q_synth = 0;
        move16();
    }
    ELSE
    {
        /*n4 = (short)(N_LEAD_MDCT*(float)(output_frame/20));*/
        test();
        IF (st_fx->bfi_fx == 0 && sub(st_fx->prev_bfi_fx, 1) == 0)
        {
            test();
            IF((sub(st_fx->stat_mode_out_fx, 1) == 0) || (sub(st_fx->diff_energy_fx, ED_THRES_50P_fx) < 0))/* Q11 */
            {
                Repetition_smoothing_nextgood_fx( wtda_audio_fx, out_fx, st_fx->oldIMDCTout_fx, st_fx->old_out_fx, st_fx->old_bfi_cnt_fx> 1 ? 1:0, N_LEAD_NB);
                *Q_synth = 0;
                move16();
            }
            ELSE IF(sub(st_fx->old_bfi_cnt_fx, 1) > 0)
            {
                Next_good_after_burst_erasures_fx( wtda_audio_fx, out_fx, st_fx->old_out_fx, N_LEAD_NB );
                *Q_synth = 0;
                move16();
            }
            ELSE
            {
                /*window_ola( wtda_audio, out, st->old_out, output_frame, 0, 0, 0); */
                window_ola_fx( wtda_audio_fx, out_fx, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
                st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, st_fx->prev_bfi_fx, st_fx->oldHqVoicing_fx , st_fx->oldgapsynth_fx );
            }
        }
        ELSE /* if(st->bfi_fx == 1) */
        {
            test();
            IF( (sub(st_fx->stat_mode_out_fx, 1) == 0) || (sub(st_fx->diff_energy_fx, ED_THRES_50P_fx) < 0 ))
            {
                /* if( window_ola_bfi( wtda_audio, out, st->oldIMDCTout, st->old_out, output_frame, st->prev_oldauOut, N_LEAD_NB) ) */
                IF( Repetition_smoothing_fx( wtda_audio_fx, out_fx, st_fx->oldIMDCTout_fx, st_fx->old_out_fx, output_frame, st_fx->prev_oldauOut_fx, N_LEAD_NB) )
                {
                    /*window_ola( wtda_audio, out, st->old_out, output_frame, 0, 0, 0);*/
                    window_ola_fx( wtda_audio_fx, out_fx, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
                    st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, st_fx->prev_bfi_fx, st_fx->oldHqVoicing_fx , st_fx->oldgapsynth_fx );
                }
                ELSE
                {
                    *Q_synth = 0;
                    move16();
                }
            }
            ELSE
            {
                /*window_ola( wtda_audio, out, st->old_out, output_frame, 0, 0, 0 );*/
                window_ola_fx( wtda_audio_fx, out_fx, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
                st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, st_fx->prev_bfi_fx, st_fx->oldHqVoicing_fx , st_fx->oldgapsynth_fx );
            }
        }
        st_fx->phase_mat_next_fx = 0;
        move16();
    }

    return;
}

void Next_good_after_burst_erasures_fx(
    const Word32 *ImdctOut_fx,
    Word16 *auOut_fx,
    Word16 *OldauOut_fx,
    const Word16 ol_size
)
{
    Word16 i, L;
    Word16 ImdctOutWin_fx[2*L_FRAME48k];
    Word16 win_NB_fx[L_FRAME8k + 25];

    L = L_FRAME8k;
    move16();
    FOR (i=0; i<L + 25; i++)
    {
        win_NB_fx[i] = window_48kHz_fx16[i*6+3];
        move16();
    }

    /* Windowing */
    Windowing_1st_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx, NULL, 0);
    Windowing_2nd_NB_fx(ImdctOutWin_fx, ImdctOut_fx, win_NB_fx);

    /* Overlapping with next good frame : Overlapping to remove the discontinuity */
    Smoothing_vector_scaledown_NB_fx(&OldauOut_fx[N_ZERO_NB], &ImdctOutWin_fx[N_Z_L_O_NB], SmoothingWin_NB875_fx, &OldauOut_fx[N_ZERO_NB], ol_size);

    /* Scale down the overlapped signal */
    Scaledown_fx(&ImdctOutWin_fx[ol_size+N_Z_L_O_NB], &OldauOut_fx[ol_size+N_ZERO_NB], 23170, N_Z_L_NB-ol_size);

    /* Common Overlapping */
    common_overlapping_fx(auOut_fx, ImdctOutWin_fx, OldauOut_fx, N_Z_L_NB, N_ZERO_NB, 0, N_ZERO_NB, L, N_Z_L_NB);
    Copy(&ImdctOutWin_fx[L], OldauOut_fx, L);

    return;
}

