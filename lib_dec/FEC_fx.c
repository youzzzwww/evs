/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Common static table prototypes        */
#include "rom_dec_fx.h"    /* Decoder static table prototypes        */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/
static void pulseRes_preCalc(Word16* cond1, Word16* cond2, Word32* cond3 ,Word16 new_pit, Word16 Tc, Word16 L_frame);
void gain_dec_bfi_fx(Word16 *past_qua_en);
/*======================================================================*/
/* FUNCTION : FEC_exc_estim_fx()                    */
/*----------------------------------------------------------------------*/
/* PURPOSE :  Calculation of excitation signal                        */
/*                                    */
/*----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS :                        */
/* _ (Struct)  st_fx      : decoder static memory          */
/* _ (Word16) L_frame_fx    : length of the frame        Q0  */
/* _ (Word16) st_fx->lp_ener_fx  :  FEC - low-pass filtered energy   Q6  */
/* _ (Word16) st_fx->lp_gainc_fx  : FEC - low-pass filtered code gain Q3  */
/* _ (Word16) st_fx->old_pitch_buf  : FEC buffer of old subframe pitch valuesQ6*/
/* _ (Word16) st_fx->last_good    : FEC - clas of last good received    */
/* _ (Word16) st_fx->bfi_pitch_fx  : LP filter coefficient          */
/* _ (Word16) st_fx->upd_cnt_fx  : FEC counter of frames since last update*/
/* _ (Word16) st_fx->last_coder_type_fx: previous coder type        */
/* _ (Word16) st_fx->Last_GSC_pit_band_idx_fx: AC mode (GSC)Last pitch band index*/
/* _ (Word16) st_fx->stab_fac_fx    : LSF stability factor Q15      */
/* _ (Word16) st_fx->tilt_code    : tilt of code        Q15      */
/* _ (Word16) st_fx->last_voice_factor    : coding type        Q12  */
/* _ (Word16) st_fx->opt_AMR_WB_fx : coding type        Q12      */
/* _ (Word16) st_fx->lp_gainp_fx  : FEC -low-pass filtered pitch gain Q14  */
/* _ (Word16) st_fx->seed    :FEC-seed for random generator for excitation*/
/* _ (Word16) st_fx->opt_OMR_WB:flag indicating AMR-WB IO mode              */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                      */
/* _ (Word16[]) exc_fx               : adapt. excitation exc (Q_exc)       */
/* _ (Word16[]) exc2_fx              : adapt. excitation/total exc (Q_exc)   */
/* _ (Word16[]) bwe_exc_fx           : excitation for SWB TBE (Q_exc)     */
/* _ (Word16[]) pitch_buf_fx         : floating pitch values for each subframe Q6*/
/* _ (Word16[])  voice_factors_fx    : frame error rate        Q15     */
/* _ (Word16[])  FEC_pitch_fx(tmp_tc):    FEC pitch  Q6                 */
/*-----------------------------------------------------------------------*/

/* _ (Word16) st_fx->lp_gainp_fx  : FEC -low-pass filtered pitch gain Q14   */
/* _ (Word16) st_fx->seed    :FEC-seed for random generator for excitation*/
/* _ (Word16) st_fx->bfi_pitch_fx  : LP filter coefficient          */
/* _ (Word16) st_fx->lp_gainc_fx  : FEC - low-pass filtered code gain Q3  */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                           */
/* _ None                                 */
/*=======================================================================*/


void FEC_exc_estim_fx(
    Decoder_State_fx *st_fx,            /* i/o: Decoder static memory                        */
    const Word16 L_frame,           /* i  : length of the frame                          */
    Word16 *exc,              /* o  : pointer to excitation buffer (with past)     */
    Word16 *exc2,             /* o  : total excitation (for synthesis)             */
    Word16 exc_dct_in[],      /* o  : GSC excitation in DCT domain                 */
    Word16 *pitch_buf,        /* o  : Floating pitch   for each subframe           */
    Word16 *voice_factors,    /* o  : voicing factors                              */
    Word16 *tmp_tc,           /* o  : FEC pitch  Q6                                */
    Word16 *bwe_exc,          /* o  : excitation for SWB TBE                       */
    Word16 *lsf_new,          /* i  : ISFs at the end of the frame                 */
    Word16 *Q_exc,
    Word16 *tmp_noise         /* o  : long-term noise energy  Q0                   */
)
{

    Word16 exc2_buf[L_FRAME16k + MODE1_L_FIR_FER-1];
    Word16 gainCNG,new_pit /*Q0*/;  /* Q3*/
    Word16 exp;
    Word32 L_tmp,L_tmp2;
    Word16 tmp,tmp1,tmp16;
    Word16 delta;
    Word16 i,j;
    Word16 alpha;
    Word16 gain,gain_inov;
    Word16 Tc;
    Word16 *pt_exc,*pt1_exc;
    Word16 step;
    Word32 L_step;
    Word16 hp_filt[5];
    Word16 Diff_len,max_len,Len;
    Word16 last_bin_fx, nb_subfr;
    Word16 extrapolationFailed;
    Word16 cond1, cond2;
    Word32 cond3;
    Word32 predPitchLag;

    /* nb_subfr = L_frame/L_SUBFR */
    nb_subfr = shr(L_frame, 6);
    Diff_len = 0;     /* to avoid compilation flags */
    set16_fx( exc_dct_in, 0, L_FRAME16k );

    extrapolationFailed = 1;
    move16();

    gainCNG = 0;
    move16();
    IF(st_fx->lp_ener_fx != 0)
    {
        exp = norm_l(st_fx->lp_ener_fx);          /*lp_ener in Q6*/
        tmp = extract_h(L_shl(st_fx->lp_ener_fx,exp));
        exp = sub(exp, 30-6);

        tmp = div_s(16384, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp);

        gainCNG = round_fx(L_shl(L_tmp, sub(exp, 12))); /* In Q3 */

    }
    tmp1 = shl(st_fx->lp_gainc_fx,1);
    gainCNG = s_min(gainCNG , tmp1);
    set16_fx( exc_dct_in, 0, L_FRAME16k );

    /*-----------------------------------------------------------------*
     * pitch extrapolation
     *-----------------------------------------------------------------*/

    {
        Word32 *tmp_old_pitch /*15Q16*/;
        Word16 tmp_pitmin, tmp_pitmax;

        /*tmp_old_pitch = L_frame == L_FRAME ? &st_fx->old_pitch_buf_fx[2*NB_SUBFR-1] : &st_fx->old_pitch_buf_fx[2*NB_SUBFR16k-1];*/
        /*tmp_pitmin = L_frame == L_FRAME?PIT_MIN_DOUBLEEXTEND:PIT16k_MIN_EXTEND;*/
        /*tmp_pitmax = L_frame == L_FRAME?PIT_MAX:PIT16k_MAX;*/

        tmp_old_pitch = &st_fx->old_pitch_buf_fx[2*NB_SUBFR16k-1];
        tmp_pitmin = PIT16k_MIN_EXTEND;
        tmp_pitmax = PIT16k_MAX;
        IF(sub(L_frame, L_FRAME) == 0)
        {
            tmp_old_pitch = &st_fx->old_pitch_buf_fx[2*NB_SUBFR-1];
            tmp_pitmin = PIT_MIN_DOUBLEEXTEND;
            move16();
            tmp_pitmax = PIT_MAX;
            move16();
        }


        pitch_pred_linear_fit(
            st_fx->nbLostCmpt,
            st_fx->last_good_fx,
            st_fx->old_pitch_buf_fx,
            tmp_old_pitch,
            &predPitchLag,
            tmp_pitmin,
            tmp_pitmax,
            st_fx->mem_pitch_gain,
            0,
            0,
            &extrapolationFailed,
            nb_subfr);

        new_pit/*Q0 int*/ = shl(round_fx(predPitchLag),0);
    }



    /*-----------------------------------------------------------------*
     * estimate subframe pitch values for the FEC frame
     *-----------------------------------------------------------------*/

    /* initialize pitch to the long-term pitch */

    *tmp_tc = st_fx->bfi_pitch_fx;
    move16();    /*Q6*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        test();
        test();
        IF ( (sub(round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR-1], 6)), shl(mult(29491, st_fx->bfi_pitch_fx), 1)) < 0 &&
              sub(round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR-1], 6)), mult(19661, st_fx->bfi_pitch_fx)) > 0) ||      /* last pitch coherent with the past  */
             sub(st_fx->upd_cnt_fx, MAX_UPD_CNT) >= 0)      /* or last update too far in the past */
        {
            /* take the pitch value of last subframe of the previous frame */
            *tmp_tc = round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR-1], 6));
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        test();
        test();
        IF ( (sub(round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR16k-1], 6)), shl(mult(29491, st_fx->bfi_pitch_fx), 1)) < 0 &&
        sub(round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR16k-1], 6)), mult(19661, st_fx->bfi_pitch_fx)) > 0) ||      /* last pitch coherent with the past  */
        sub(st_fx->upd_cnt_fx, MAX_UPD_CNT) >= 0)      /* or last update too far in the past */
        {
            /* take the pitch value of last subframe of the previous frame */
            *tmp_tc = round_fx(L_shl(st_fx->old_pitch_buf_fx[2*NB_SUBFR16k-1], 6));
        }
    }

    /* convert pitch period */
    /* Tc = (short)(tmp_tc + 0.5f) */
    Tc = shr_r(*tmp_tc,6);

    /* estimate pitch values for all subframes */
    /*calculate conditions for Pulse resynchronization to take place*/
    pulseRes_preCalc( &cond1, &cond2, &cond3 , new_pit,  Tc,  L_frame);

    test();
    test();
    test();
    test();
    IF ((cond1 < 0 )
        && (new_pit > 0) && (cond2 != 0)
        && (cond3 > 0)
        && extrapolationFailed == 0
       )
    {
        tmp16 = *tmp_tc; /*Q6*/                               move16();
        IF(sub(nb_subfr,4)==0)
        {
            delta = shr(sub(shl(new_pit,6), *tmp_tc),2 ); /* 4 sub-frames */
        }
        ELSE
        {
            delta = mult_r(sub(shl(new_pit,6), *tmp_tc),6554); /* 5 sub-frames */
        }
        FOR (i = 0; i < nb_subfr; i++) /* subframe pitch values */
        {
            /* fT0 += delta */
            tmp16 = add(tmp16, delta);
            /* ptch_buf[i] = (short)(fT0 + 0.5) */
            pitch_buf[i] = shl(mult_r(tmp16, 512), 6);
            move16();
        }
    }
    ELSE
    {
        FOR (i = 0; i < nb_subfr; i++) /* subframe pitch values for bass postfilter */
        {
            pitch_buf[i] = *tmp_tc;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * estimate damping factor
     *-----------------------------------------------------------------*/

    /* rapid convergence to 0 */
    alpha = _ALPHA_VT_FX;
    move16();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( st_fx->last_coder_type_fx == UNVOICED && sub(st_fx->nbLostCmpt, 3) <= 0 )
    {
        /* last good frame was clearly unvoiced */
        alpha = _ALPHA_UU_FX;
        move16();
    }
    ELSE IF( sub(st_fx->last_coder_type_fx,AUDIO) == 0 ||  sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0 )
    {
        test();
        IF( st_fx->Last_GSC_pit_band_idx_fx > 0 && sub(st_fx->nbLostCmpt,1) > 0 )
        {
            alpha = 26214;
            move16();
        }
        ELSE IF( sub(st_fx->nbLostCmpt,5) <= 0 )
        {
            alpha = 32604;
            move16();
        }
        ELSE
        {
            alpha = 31130;
            move16();
        }
    }
    ELSE IF( sub(st_fx->last_good_fx,UNVOICED_CLAS) == 0 )
    {
        IF( sub(st_fx->nbLostCmpt,1) <= 0)
        {
            /* if stable, do not decrease the energy, pitch_gain = 0 */
            alpha = mac_r((1L<<16)*2*_ALPHA_U_FX, st_fx->stab_fac_fx, 32768-2*_ALPHA_U_FX);  /*st_fx->stab_fac_fx in Q15*/
        }
        ELSE IF ( sub(st_fx->nbLostCmpt,2) == 0 )
        {
            alpha =_ALPHA_S_FX;
            move16();  /* ALPHA_U*1.5f = 0.6 */
        }
        ELSE
        {
            alpha = _ALPHA_U_FX;
            move16(); /* 0.4 go rapidly to CNG gain, pitch gain = 0 */
        }
    }
    ELSE IF( sub(st_fx->last_good_fx,UNVOICED_TRANSITION) == 0 )
    {
        alpha = _ALPHA_UT_FX;
        move16();
    }
    ELSE IF( sub(st_fx->last_good_fx,ONSET) == 0 && sub(st_fx->nbLostCmpt,3) <= 0 && (sub(st_fx->last_coder_type_fx,GENERIC) == 0 || sub(st_fx->last_coder_type_fx,TRANSITION) == 0) )
    {
        alpha = 26214;
        move16();    /* mild convergence to 0 for the first 3 erased frames 0.8 in Q15 */
    }
    ELSE IF( ( sub(st_fx->last_good_fx,VOICED_CLAS) == 0 || sub(st_fx->last_good_fx,ONSET) == 0 ) && sub(st_fx->nbLostCmpt,3) <= 0 )
    {
        alpha = _ALPHA_V_FX;
        move16();   /* constant for the first 3 erased frames */
    }
    ELSE IF( sub(st_fx->last_good_fx,SIN_ONSET) == 0 )
    {
        alpha = _ALPHA_S_FX;
        move16();
    }
    test();
    test();
    IF( sub(st_fx->last_good_fx,VOICED_CLAS) >= 0 && sub(st_fx->last_good_fx,INACTIVE_CLAS) < 0 && sub(st_fx->last_coder_type_fx,AUDIO) != 0 )
    {
        IF( sub(st_fx->nbLostCmpt,1) == 0 ) /* if first erased frame in a block, reset harmonic gain */
        {
            /* move pitch gain towards 1 for voiced to remove energy fluctuations */
            /*gain = (float)sqrt( st_fx->lp_gainp );*/
            st_fx->lp_gainp_fx = s_max(st_fx->lp_gainp_fx, 1);
            exp = norm_s(st_fx->lp_gainp_fx);
            tmp = shl(st_fx->lp_gainp_fx, exp);
            tmp = div_s(16384, tmp);
            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp);

            gain = extract_h(L_shl(L_tmp, exp));

            gain = s_min(gain, 32113);  /*0.98 */
            gain = s_max(gain, 27853);  /*0.85 */

            alpha = mult_r(alpha, gain);
        }
        ELSE
        {
            /* st_fx->lp_gainp_fx is in Q14 when bfi_cnt > 1 to follow floating point because lp_gainp could be > than 1  */
            alpha = st_fx->lp_gainp_fx;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * construct the harmonic part of excitation
     *-----------------------------------------------------------------*/
    test();
    test();
    test();
    test();
    IF( (sub(st_fx->last_good_fx,UNVOICED_TRANSITION) >= 0 && sub(st_fx->last_good_fx,INACTIVE_CLAS) < 0) ||
        ( (sub(st_fx->last_coder_type_fx,AUDIO) == 0 || sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0) && st_fx->Last_GSC_pit_band_idx_fx > 0) )
    {

        pt_exc = exc;
        move16();
        pt1_exc = pt_exc - Tc;
        move16();

        IF (sub(st_fx->nbLostCmpt,1) == 0)
        {
            /* first pitch cycle is low-pass filtered */

            FOR (i = 0; i < Tc; i++) /* pitch cycle is first low-pass filtered */
            {
                /* *pt_exc++ = (0.18f * pt1_exc[-1] + 0.64f * pt1_exc[0] + 0.18f * pt1_exc[1]) */
                L_tmp = L_mult(5898, pt1_exc[-1]);
                L_tmp = L_mac(L_tmp, 20972, pt1_exc[0]);
                *pt_exc++ = mac_r(L_tmp, 5898, pt1_exc[1]);
                move16();
                pt1_exc++;
            }
        }

        /* last pitch cycle of the previous frame is repeatedly copied up to an extra subframe */

        tmp = (Word16)((exc + L_frame + L_SUBFR) - pt_exc);
        FOR (i = 0; i < tmp; i++)
        {
            *pt_exc++ = *pt1_exc++;
            move16();
        }

        IF (new_pit > 0 )
        {
            /*calculate conditions for Pulse resynchronization to take place*/
            pulseRes_preCalc( &cond1, &cond2, &cond3 , new_pit,  Tc,  L_frame);

            test();
            test();
            test();
            test();
            IF ((cond1 < 0 )
                && (new_pit > 0) && (cond2 != 0)
                && (cond3 > 0)
                && extrapolationFailed == 0
               )
            {
                Copy(exc,exc-L_frame-L_SUBFR,L_frame+L_SUBFR);
                PulseResynchronization(exc-L_frame-L_SUBFR, exc, L_frame, nb_subfr, L_deposit_h(Tc/*Q0*/)/*15Q16*/, L_deposit_h(new_pit/*Q0*/)/*15Q16*/);
            }
        }
        test();
        test();
        IF( sub(st_fx->last_good_fx,UNVOICED_TRANSITION) == 0  && ( sub(st_fx->last_coder_type_fx,GENERIC) == 0 || sub(st_fx->last_coder_type_fx,TRANSITION) == 0 ) )
        {
            /* start of the frame gain */
            gain = 0;
            move16();

            /* end of the frame gain */
            st_fx->lp_gainp_fx = 0;
            move16();
            step = 0;
            move16();
        }
        ELSE
        {

            /* start of the frame gain */
            gain = 16384;
            move16();

            /* end of the frame gain */
            test();

            IF(!(sub(st_fx->last_good_fx,VOICED_CLAS) >= 0 && sub(st_fx->last_good_fx,INACTIVE_CLAS) < 0 && sub(st_fx->last_coder_type_fx,AUDIO) != 0 && sub(st_fx->nbLostCmpt, 1) > 0 ))
            {
                st_fx->lp_gainp_fx = shr(alpha,1);    /* alpha in Q15 */
            }
            ELSE
            {
                st_fx->lp_gainp_fx = alpha;
                move16();    /* alpha in Q14 */
            }

            IF(sub(L_frame, L_FRAME) == 0)
            {
                step = shr(sub(gain,st_fx->lp_gainp_fx),8);
            }
            ELSE /*L_frame == L_FRAME16k*/
            {
                /*step = (1.0f/L_frame) * (gain - st_fx->lp_gainp);*/

                step = shr(mult_r(26214,sub(gain,st_fx->lp_gainp_fx)),8);  /*Q14*/
            }
        }

        FOR( i=0 ; i< L_frame; i++ )
        {
            /* exc[i] *= gain */
            exc[i] = round_fx(L_shl(L_mult(exc[i], gain), 1)); /* in Q_exc */
            /* gain -= step */
            gain = sub(gain, step);
        }
        test();
        test();
        IF( (sub(st_fx->last_coder_type_fx,AUDIO) == 0 || sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0) && st_fx->Last_GSC_pit_band_idx_fx > 0 )
        {
            Diff_len = mfreq_loc_div_25[st_fx->Last_GSC_pit_band_idx_fx];
            move16();

            /* Transform to frequency domain */
            edct_16fx( exc, exc_dct_in, st_fx->L_frame_fx, 5 );

            /* Reset unvaluable part of the adaptive (pitch) excitation contribution */
            max_len = sub(st_fx->L_frame_fx,Diff_len);
            Len = min( max_len, 80 );

            move16();     /*ptr init*/
            FOR( i=0; i<Len; i++ )
            {
                exc_dct_in[i + Diff_len] = mult_r(exc_dct_in[i + Diff_len],sm_table_fx[i]);
                move16();
            }

            FOR( ; i<max_len; i++ )
            {
                exc_dct_in[i + Diff_len] = 0;
                move16();
            }
            Diff_len = add(Diff_len,1);
        }
    }/* end of "if st_fx->last_good >= VOICED_TRANSITION" */

    /*-----------------------------------------------------------------*
    * Replicate the last spectrum in case the last good frame was coded by GSC
    *-----------------------------------------------------------------*/
    test();
    test();
    test();
    IF( sub(st_fx->last_coder_type_fx,AUDIO) == 0 || (sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0 && L_sub(st_fx->total_brate_fx,ACELP_24k40) <= 0 && !st_fx->Opt_AMR_WB_fx) )
    {
        st_fx->GSC_noisy_speech_fx = st_fx->Last_GSC_noisy_speech_flag_fx;
        move16();
        /* Replication of the last spectrum, with a slight downscaling of its dynamic */
        gsc_dec_fx( st_fx, exc_dct_in, st_fx->Last_GSC_pit_band_idx_fx, Diff_len, 0, st_fx->L_frame_fx/L_SUBFR, st_fx->last_coder_type_fx,  &last_bin_fx, lsf_new, NULL, st_fx->Q_exc );
        *tmp_noise = shr_r(st_fx->lp_gainc_fx,3);  /*Q0*/
        /* Transform back to time domain */
        edct_16fx( exc_dct_in, exc, st_fx->L_frame_fx, 5 );
    }
    ELSE
    {
        /*-----------------------------------------------------------------*
         * Construct the random part of excitation
         *-----------------------------------------------------------------*/

        /* generate the random part of the excitation */
        FOR (i=0; i<L_frame+MODE1_L_FIR_FER-1; i++)
        {
            /*Q-3*/
            exc2_buf[i] = shr(Random( &st_fx->seed_fx ),3);
            move16();
        }

        /* start of the frame gain */
        gain = st_fx->lp_gainc_fx;
        move16();

        test();
        test();
        test();
        IF(!(sub(st_fx->last_good_fx,VOICED_CLAS) >= 0 && sub(st_fx->last_good_fx,INACTIVE_CLAS) < 0 && sub(st_fx->last_coder_type_fx,AUDIO) != 0 && sub(st_fx->nbLostCmpt, 1) > 0 ))
        {
            /* Here alpha is in Q15 and lp_gainc_fx in Q3 */
            /*  st_fx->lp_gainc = alpha * st_fx->lp_gainc + (1.0f - alpha) * gainCNG; */
            L_tmp = L_mult(alpha, st_fx->lp_gainc_fx);

            st_fx->lp_gainc_fx = msu_r(L_tmp, add(alpha, -32768), gainCNG);
        }
        ELSE
        {  /* Here alpha is in Q14, but lp_gainc still in Q3 */
            /*  st_fx->lp_gainc = alpha * st_fx->lp_gainc + (1.0f - alpha) * gainCNG; */
            L_tmp = L_mult(alpha, st_fx->lp_gainc_fx);  /* Q14*Q3->Q18 */

            st_fx->lp_gainc_fx = round_fx(L_shl(L_msu(L_tmp, add(alpha, -16384), gainCNG),1));/* (Q14*Q3<<1)>>16 ->Q3 */
        }
        test();
        test();
        if( sub(st_fx->last_good_fx,UNVOICED_TRANSITION) == 0 && ( sub(st_fx->last_coder_type_fx,GENERIC) == 0 || sub(st_fx->last_coder_type_fx,TRANSITION) == 0 ) )
        {
            st_fx->lp_gainc_fx = gainCNG;
            move16();
        }

        /* linearly attenuate the gain throughout the frame */
        /* step = (1.0f/L_FRAME) * (gain - *lp_gainc); */
        step = sub(gain,st_fx->lp_gainc_fx); /* divide by L_FRAME done later */
        test();
        if(sub(L_frame,L_FRAME16k) == 0)
        {
            step = mult_r(step,26214);    /* L_frame16k-> L_frame and division by L_frame done later*/
        }

        /* calculate gain to normalize energy */
        pt_exc = exc2_buf + MODE1_L_FIR_FER/2;
        move16();

        /* To avoid saturation split the L_frame dot product into (L_frame/L_SUBFR) dot products
           and scale down before adding */
        /*  gain_inov = 1.0f / (float)sqrt( dotp( pt_exc, pt_exc, L_frame ) / L_frame + 0.01f ); */

        L_tmp = L_deposit_l(0);
        FOR (i = 0; i < 2; i++)
        {
            L_tmp2 = L_mult0(*pt_exc, *pt_exc);
            pt_exc++;
            FOR (j = 1; j < shr(L_frame,1); j++)
            {
                L_tmp2 = L_mac0(L_tmp2, *pt_exc, *pt_exc);  /* Q-5 */
                pt_exc++;
            }
            L_tmp = L_add(L_tmp, L_shr(L_tmp2, 1));         /* Q-7 */
        }
        test();
        if(sub(L_frame,L_FRAME16k) == 0)
        {
            L_tmp = Mult_32_16(L_tmp, 26214);   /* x0.8 to normalize to 256 samples */
        }
        exp = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, exp);    /* Normalize                  */
        exp = add(exp, 8-7);          /* Q0, 8 ->divide by 256      */
        exp = sub(31, exp);           /* For Denormalization in Q31 */
        L_tmp = Isqrt_lc(L_tmp, &exp);/* in Q(31-exp)                     */
        gain_inov = round_fx(L_tmp);

        /* attenuate somewhat on unstable unvoiced */
        test();
        test();
        if( (sub(st_fx->last_good_fx,UNVOICED_CLAS) == 0 || sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0) && sub(st_fx->last_coder_type_fx,UNVOICED) != 0 )
        {
            gain_inov = mult_r(gain_inov, 26214);
        }

        /* scaling of the random part of excitation */
        pt_exc = exc2_buf;
        move16();
        L_step = L_shr(L_mult(gain_inov, step), 8); /* here is the divide by L_FRAME */
        L_tmp2 = L_mult(gain_inov, gain);                          /* Q15 * Q3 -> Q3 */
        tmp = round_fx(L_tmp2);
        exp = add(add(exp, *Q_exc), 15);                   /* 3+Q_exc+15 -> Q_exc+18 */

        FOR( i=0; i< MODE1_L_FIR_FER/2; i++ )
        {
            /* non-causal ringing of the FIR filter */
            /**pt_exc++ *= (gain_inov * gain);*/
            L_tmp = L_mult(tmp, *pt_exc);              /* Q_exc+18 * Q-3 -> Q_exc+16 */
            *pt_exc++ = round_fx(L_shl(L_tmp, exp));
        }

        FOR( i=0; i< L_frame; i++ )
        {
            /* the inner part of the FIR filter */
            /* *pt_exc++ *= (gain_inov * gain); */
            L_tmp = L_mult(tmp, *pt_exc);
            *pt_exc++ = round_fx(L_shl(L_tmp, exp));
            /* gain -= step; */
            L_tmp2 = L_sub(L_tmp2, L_step);
            tmp = round_fx(L_tmp2);

        }

        FOR (i = 0 ; i < MODE1_L_FIR_FER/2; i++)       /* causal ringing of the FIR filter */
        {
            /* *pt_exc++ *= (gain_inov * gain) */
            L_tmp = L_mult(tmp, *pt_exc);
            *pt_exc++ = round_fx(L_shl(L_tmp, exp));
        }
    }

    /*-----------------------------------------------------------------*
     * Total excitation
     *-----------------------------------------------------------------*/
    test();
    test();
    test();
    test();
    IF( (sub(st_fx->last_coder_type_fx,AUDIO) == 0 || sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0) && L_sub(st_fx->total_brate_fx,ACELP_24k40) <= 0 && !st_fx->Opt_AMR_WB_fx)
    {
        /* For GSC - the excitation is already computed */
        Copy( exc, exc2, st_fx->L_frame_fx );
    }
    ELSE IF( sub(st_fx->last_good_fx,UNVOICED_TRANSITION) >= 0 && sub(st_fx->last_good_fx,INACTIVE_CLAS) < 0 )
    {
        /* For voiced and generic signals - prepare a HP filter for the random part of excitation */
        /* tmp = -(1-tilt_code) to correctly represent 1.0000 */
        tmp = add(st_fx->tilt_code_fx, -32768);
        move16();
        FOR (i = 0; i < MODE1_L_FIR_FER; i++)
        {
            hp_filt[i] = msu_r(0, tmp, h_high_fx[i]);
            move16();
        }

        /* HP filter the random part of the excitation and add the adaptive part */
        pt_exc = exc2_buf;
        move16();
        FOR( i=0; i< L_frame; i++ )
        {
            /* exc2[i] = exc[i] + dotp( &exc2_buf[i], hp_filt, MODE1_L_FIR_FER );*/
            L_tmp = L_mult(hp_filt[0], pt_exc[0]);
            FOR (j = 1; j < MODE1_L_FIR_FER; j++)
            {
                L_tmp = L_mac(L_tmp, hp_filt[j], pt_exc[j]);
            }
            exc2[i] = msu_r(L_tmp, -32768, exc[i]);
            move16();
            pt_exc++;
        }
    }
    ELSE
    {
        /* For purely unvoiced signals - just copy the unfiltered random part of the excitation */
        Copy( exc2_buf + MODE1_L_FIR_FER/2, exc, L_frame );
        Copy( exc2_buf + MODE1_L_FIR_FER/2, exc2, L_frame );
    }

    IF( sub(L_frame,L_FRAME) == 0 )
    {

        interp_code_5over2_fx( exc, bwe_exc, L_frame );
    }
    ELSE
    {
        interp_code_4over2_fx( exc, bwe_exc, L_frame );
    }
    test();
    IF( sub(st_fx->last_coder_type_fx,AUDIO) == 0 || sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0 )
    {
        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            set16_fx( voice_factors, 32767, NB_SUBFR );
        }
        ELSE
        {
            set16_fx( voice_factors, 32767, NB_SUBFR16k );
        }
    }
    ELSE
    {
        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            set16_fx( voice_factors, st_fx->last_voice_factor_fx, NB_SUBFR);             /* The factor of the last subframe is propagated forward */
        }
        ELSE
        {
            set16_fx( voice_factors, st_fx->last_voice_factor_fx, NB_SUBFR16k );           /* The factor of the last subframe is propagated forward */
        }
    }
    IF( st_fx->Opt_AMR_WB_fx )
    {
        gain_dec_bfi_fx(st_fx->past_qua_en_fx);
    }
    st_fx->bfi_pitch_fx = pitch_buf[(L_frame/L_SUBFR)-1];
    move16();
    st_fx->bfi_pitch_frame_fx = st_fx->L_frame_fx;
    move16();
    return;
}


/*calculates some conditions for Pulse resynchronization to take place*/
static void pulseRes_preCalc(Word16* cond1, Word16* cond2, Word32* cond3 ,Word16 new_pit, Word16 Tc, Word16 L_frame)
{
    Word16 tmp_pit, tmp_pit_e, tmp_frame, tmp_frame_e;
    Word32 tmp_pit2;

    tmp_pit = BASOP_Util_Divide1616_Scale(new_pit/*Q0*/,Tc/*Q0*/,&tmp_pit_e)/*Q15*/;
    tmp_frame = add(  extract_l(L_mult0(L_frame ,  FL2WORD16_SCALE(1.f/L_SUBFR,3))/*Q12*/) , FL2WORD16_SCALE(1.f,3) );/*Q12*/
    tmp_frame = BASOP_Util_Divide1616_Scale(FL2WORD16_SCALE(1.f,3),tmp_frame, &tmp_frame_e);/*Q15*/
    tmp_frame = shl(tmp_frame,add(tmp_frame_e,1));
    tmp_frame = sub(FL2WORD16(1.f), tmp_frame);/*Q15*/
    BASOP_SATURATE_WARNING_OFF
    /*To calc Q15 threshold, overflow may happen - do negation and compare with negated value to check also highest possible value*/
    tmp_pit = shl(negate(tmp_pit),tmp_pit_e);
    BASOP_SATURATE_WARNING_ON
    *cond1 = sub(tmp_pit, negate(tmp_frame));

    *cond2 = sub(Tc, new_pit);

    tmp_pit_e = BASOP_Util_Add_MantExp(new_pit,15-0,negate(Tc),15-0,&tmp_pit);/*Q15*/
    tmp_pit = abs_s(tmp_pit);
    tmp_pit2 = L_mult(Tc,FL2WORD16(0.15f));/*Q16*/
    BASOP_SATURATE_WARNING_OFF
    /*To calc Q15 threshold, overflow may happen - do negation and compare with negated value to check also highest possible value*/
    tmp_pit2 = L_shl(L_negate(tmp_pit2),sub(15-16,tmp_pit_e));
    BASOP_SATURATE_WARNING_ON
    *cond3 = L_sub(L_mult0(-1, tmp_pit),tmp_pit2);
}

/*-------------------------------------------------------------------*
 * gain_dec_bfi()
 *
 * Estimate past quantized gain prediction residual to be used in
 * next frame
 *-------------------------------------------------------------------*/

void gain_dec_bfi_fx(
    Word16 *past_qua_en    /* i/o: gain quantization memory (4 words)  */
)
{
    Word16   i;
    Word16 av_pred_en;
    Word32 Lav_pred_en;

    Lav_pred_en = L_mult(past_qua_en[0], 8192);
    FOR (i = 1; i < GAIN_PRED_ORDER; i++)
    {
        Lav_pred_en = L_mac(Lav_pred_en, past_qua_en[i], 8192);
    }

    /*av_pred_en = (float)(av_pred_en*(1.0f/(float)GAIN_PRED_ORDER)-3.0f);*/
    av_pred_en = sub(round_fx(Lav_pred_en),3<<10);

    /*if (av_pred_en < -14.0f)av_pred_en = -14.0f;*/
    av_pred_en = s_max(av_pred_en, -14<<10);


    FOR (i=GAIN_PRED_ORDER-1; i>0; i--)
    {
        past_qua_en[i] = past_qua_en[i-1];
        move16();
    }

    past_qua_en[0] = av_pred_en;
    move16();

    return;
}
