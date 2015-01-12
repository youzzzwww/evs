/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"

#include "prot_fx.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static Word16 SWB_BWE_encoding_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure   */
    Word16 *insig_fx,           /* i/o: delayed original input signal at 32kHz (might be rescaled)*/
    const Word16 *insig_lp_fx,        /* i  : delayed original lowband input signal at 32kHz */
    const Word16 *insig_hp_fx,        /* i  : delayed original highband input signal at 32kHz */
    const Word16 *synth_fx,           /* i  : delayed ACELP core synthesis at 12.8kHz */
    const Word16 *yos_fx,             /* i  : MDCT coefficients of the windowed original input signal at 32kHz */
    Word16 *SWB_fenv_fx,        /* o  : frequency-domain quantized BWE envelope */
    const Word16 tilt_nb_fx,          /* i  : SWB tilt */
    const Word16 st_offset,           /* i  : start frequency offset for BWE envelope */
    const Word16 coder_type,          /* i  : coding type                              */
    Word16 Q_insig_lp,
    Word16 Q_shb,
    Word16 Q_synth,
    Word16 Q_synth_lf
);

static void delay_input_signal_fx(
    Word16 *old_sig,
    Word16 *cur_sig,
    Word16 *new_sig,
    Word16 m1,
    Word16 m2,
    Word16 *Q_old,
    Word16 *Q_new
)
{
    Word16 i;
    Word16 max;
    Word16 max1_exp, max2_exp;

    max = abs_s(old_sig[0]);
    FOR(i=1; i<m1; i++)
    {
        max = s_max(max, abs_s(old_sig[i]));
    }
    IF(max == 0)
    {
        max1_exp = 15;
        move16();
    }
    ELSE
    {
        max1_exp = norm_s(max);
    }

    max = abs_s(new_sig[0]);
    FOR(i=1; i<m2; i++)
    {
        max = s_max(max, abs_s(new_sig[i]));
    }
    IF(max == 0)
    {
        max2_exp = 15;
        move16();
    }
    ELSE
    {
        max2_exp = norm_s(max);
    }

    IF(sub(add(max1_exp, *Q_old), add(max2_exp, *Q_new)) > 0)
    {
        Copy_Scale_sig(new_sig, new_sig, m2, max2_exp);
        Copy_Scale_sig(old_sig, old_sig, m1, sub(add(max2_exp, *Q_new), *Q_old));
        *Q_new = add(max2_exp, *Q_new);
    }
    ELSE IF(sub(add(max1_exp, *Q_old), add(max2_exp, *Q_new)) < 0)
    {
        Copy_Scale_sig(new_sig, new_sig, m2, sub(add(max1_exp, *Q_old), *Q_new));
        Copy_Scale_sig(old_sig, old_sig, m1, max1_exp);
        *Q_new = add(max1_exp, *Q_old);
    }
    ELSE
    {
        Copy_Scale_sig(new_sig, new_sig, m2, max2_exp);
        Copy_Scale_sig(old_sig, old_sig, m1, max1_exp);
        *Q_new = add(max1_exp, *Q_old);
    }
    *Q_old = *Q_new;
    move16();
    Copy(old_sig, cur_sig, m1);
    Copy( new_sig, &cur_sig[m1], sub(m2,m1) );
    Copy( new_sig + sub(m2,m1), old_sig, m1 );

    return;
}

/*-------------------------------------------------------------------*
 * wb_bwe_enc()
 *
 * WB BWE encoder
 *-------------------------------------------------------------------*/
void wb_bwe_enc_fx(
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure                  */
    const Word16 *new_wb_speech_fx,  /* i  : original input signal at 16kHz           */
    Word16 coder_type          /* i  : coding type                              */
)
{
    Word16 mode = 0;
    Word16 Sample_Delay_WB_BWE;
    Word16 old_input_fx[NS2SA(16000, DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME16k];
    Word32 yorig_32[L_FRAME16k];
    Word16 yorig_fx[L_FRAME16k];
    Word32 L_wtda_synth_fx[2*L_FRAME16k];
    Word16 *new_input_fx;                                /* pointer to original input signal         */
    Word16 scl, new_input_fx_exp;
    Word16 Q_synth;
    Word16 WB_fenv_fx[SWB_FENV];

    IF( L_sub(st_fx->total_brate_fx, ACELP_13k20) == 0 )
    {
        /*---------------------------------------------------------------------*
         * Delay the original input signal to be synchronized with ACELP core synthesis
         *---------------------------------------------------------------------*/
        set16_fx( old_input_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME16k );
        Sample_Delay_WB_BWE = NS2SA( 16000, DELAY_FD_BWE_ENC_12k8_NS);

        new_input_fx = old_input_fx + Sample_Delay_WB_BWE;
        Copy( st_fx->old_input_wb_fx, old_input_fx, Sample_Delay_WB_BWE );
        Copy( new_wb_speech_fx, new_input_fx, L_FRAME16k );
        Copy( old_input_fx + L_FRAME16k, st_fx->old_input_wb_fx, Sample_Delay_WB_BWE );

        /*---------------------------------------------------------------------*/
        /* WB BWE encoding                                                     */


        /* MDCT of the core synthesis signal */
        /*---------------------------------------------------------------------*/
        new_input_fx_exp = 0;
        move16();

        wtda_fx(old_input_fx, &new_input_fx_exp, L_wtda_synth_fx, st_fx->L_old_wtda_swb_fx,
                &st_fx->Q_old_wtda,ALDO_WINDOW,ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
                L_FRAME16k );

        /* DCT of the ACELP core synthesis */
        direct_transform_fx(L_wtda_synth_fx, yorig_32, 0, L_FRAME16k, &new_input_fx_exp);

        /* Convert to 16 Bits (Calc Shift Required to Stay within MAX_Q_NEW_INPUT) */
        scl = sub(16+8/*MAX_Q_NEW_INPUT*/, new_input_fx_exp);
        /* Possible to Upscale? */
        IF (scl > 0)
        {
            /* Yes */
            /* Calc Room to Upscale */
            Q_synth = Find_Max_Norm32(yorig_32, L_FRAME16k);

            /* Stay within MAX_Q_NEW_INPUT */
            scl = s_min(Q_synth, scl);
        }
        Copy_Scale_sig32_16(yorig_32, yorig_fx, L_FRAME16k, scl);
        Q_synth = add(sub(new_input_fx_exp, 16), scl) - 1;

        mode = WB_BWE_encoding_fx( coder_type, yorig_fx, WB_fenv_fx, st_fx, Q_synth, Q_synth);
        push_indice_fx(st_fx, IND_WB_CLASS, mode - 2, 1 );
    }

    st_fx->prev_mode_fx = mode;

    return;
}

/*-------------------------------------------------------------------*
* swb_bwe_enc()
*
* SWB BWE encoder (only for 32kHz signals)
*-------------------------------------------------------------------*/
void swb_bwe_enc_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    Word16 *old_input_12k8_fx,    /* i  : input signal @12.8kHz for SWB BWE       */
    Word16 *old_input_16k_fx,     /* i  : input signal @16kHz for SWB BWE         */
    const Word16 *old_syn_12k8_16k_fx,  /* i  : ACELP core synthesis at 12.8kHz or 16kHz */
    const Word16 *new_swb_speech_fx,    /* i  : original input signal at 32kHz           */
    Word16 *shb_speech_fx,        /* i  : SHB target signal (6-14kHz) at 16kHz     */
    const Word16 coder_type,            /* i  : coding type                              */
    Word16 Q_shb_speech,
    Word16 Q_slb_speech
)
{
    Word16 i;
    Word16 *new_input_fx;
    Word16 tmp, exp, exp1;
    Word16 frac;
    Word32 L_tmp;
    Word16 inner_frame;
    Word32 inner_Fs;
    Word32 L_old_input_fx[2*L_FRAME48k];
    Word32 yorig_32[L_FRAME48k];
    Word16 old_input_fx[NS2SA(48000, DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME48k];
    Word16 old_input_lp_fx[L_FRAME16k];
    Word16 new_input_hp_fx[L_FRAME16k];
    Word16 yorig_fx[L_FRAME48k];
    Word16 scl, new_input_fx_exp;
    Word16 max;
    Word16 Sample_Delay_SWB_BWE;
    Word16 Sample_Delay_HP;
    Word16 Sample_Delay_LP;
    Word16 idxGain = 0;

    Word16 Q_synth_hf, Q_synth, Q_shb;
    Word16 tilt_nb_fx;
    Word16 SWB_fenv_fx[SWB_FENV];
    Word32 ener_low_fx;
    Word32 energy_fbe_fb_fx = 0;
    Word16 fb_ener_adjust_fx;
    Word16 ener_adjust_quan_fx = 0;


    /*---------------------------------------------------------------------*
     * Delay the original input signal to be synchronized with ACELP core synthesis
     *---------------------------------------------------------------------*/
    IF( sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        inner_frame = L_FRAME48k;
        inner_Fs = 48000;
    }
    ELSE
    {
        inner_frame = L_FRAME32k;
        inner_Fs = 32000;
    }

    set16_fx( old_input_fx, 0, add(NS2SA(inner_Fs, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS), inner_frame) );

    IF( sub(st_fx->L_frame_fx, L_FRAME) == 0 )
    {
        Sample_Delay_SWB_BWE = NS2SA(inner_Fs, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS);
        Sample_Delay_HP      = NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS);
        Sample_Delay_LP      = NS2SA(12800, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS);

        delay_input_signal_fx( st_fx->old_input_lp_fx, old_input_lp_fx, &old_input_12k8_fx[L_INP_MEM], Sample_Delay_LP, L_FRAME, &st_fx->prev_Q_input_lp, &Q_slb_speech );
    }
    ELSE
    {
        Sample_Delay_SWB_BWE = NS2SA(inner_Fs, DELAY_FD_BWE_ENC_16k_NS + DELAY_FIR_RESAMPL_NS);
        Sample_Delay_HP      = NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_16k_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS);
        Sample_Delay_LP      = NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_16k_NS);

        delay_input_signal_fx( st_fx->old_input_lp_fx, old_input_lp_fx, &old_input_16k_fx[L_INP_MEM], Sample_Delay_LP, L_FRAME16k, &st_fx->prev_Q_input_lp, &Q_slb_speech );
    }

    Copy(st_fx->new_input_hp_fx, new_input_hp_fx, Sample_Delay_HP);
    Copy( shb_speech_fx, &new_input_hp_fx[Sample_Delay_HP], L_FRAME16k-Sample_Delay_HP );
    Copy( shb_speech_fx + L_FRAME16k-Sample_Delay_HP, st_fx->new_input_hp_fx, Sample_Delay_HP );
    new_input_fx = old_input_fx + Sample_Delay_SWB_BWE;
    Copy( st_fx->old_input_fx, old_input_fx, Sample_Delay_SWB_BWE );
    Copy( new_swb_speech_fx, new_input_fx, inner_frame );
    Copy( old_input_fx + inner_frame, st_fx->old_input_fx, Sample_Delay_SWB_BWE );
    /*----------------------------------------------------------------------*
    * Calculate tilt of the input signal and the ACELP core synthesis
    *----------------------------------------------------------------------*/

    /* tilt returned in Q24 goto to Q11 */
    tilt_nb_fx = round_fx(L_shl(calc_tilt_bwe_fx(old_input_lp_fx, Q_slb_speech, st_fx->L_frame_fx), 3));
    /*---------------------------------------------------------------------*
     * SWB BWE encoding
     * FB BWE encoding
     *---------------------------------------------------------------------*/
    new_input_fx_exp = 0;
    /* MDCT of the core synthesis signal */
    wtda_fx(old_input_fx, &new_input_fx_exp, L_old_input_fx, st_fx->L_old_wtda_swb_fx,
            &st_fx->Q_old_wtda, ALDO_WINDOW, ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
            inner_frame );

    /* DCT of the ACELP core synthesis */
    direct_transform_fx(L_old_input_fx, yorig_32, 0, inner_frame, &new_input_fx_exp);

    /* Convert to 16 Bits (Calc Shift Required to Stay within MAX_Q_NEW_INPUT) */
    scl = sub(16+8, new_input_fx_exp);
    /* Possible to Upscale? */
    IF (scl > 0)
    {
        /* Yes */
        /* Calc Room to Upscale */
        Q_synth = Find_Max_Norm32(yorig_32, inner_frame);
        /* Stay within MAX_Q_NEW_INPUT */
        scl = s_min(Q_synth, scl);
    }
    Copy_Scale_sig32_16(yorig_32, yorig_fx, inner_frame, scl);
    Q_synth = add(sub(new_input_fx_exp, 16), scl);

    max = 0;
    move16();
    Q_synth_hf = 0;
    move16();
    IF (sub(st_fx->L_frame_fx, L_FRAME16k) == 0)
    {
        scl = 300;
        move16();
    }
    ELSE
    {
        scl = 240;
        move16();
    }
    FOR(i=scl; i<inner_frame; i++)
    {
        max = s_max(max, abs_s(yorig_fx[i]));
    }

    IF(max == 0)
    {
        exp = 15;
        move16();
    }
    ELSE
    {
        exp = norm_s(max);
    }

    Copy_Scale_sig(&yorig_fx[scl], &yorig_fx[scl], sub(inner_frame, scl), exp);
    Q_synth_hf = add(exp, Q_synth);

    IF(sub(st_fx->last_extl_fx, SWB_BWE) == 0)
    {
        exp = norm_l(st_fx->EnergyLT_fx);
        IF(add(st_fx->prev_Q_new, exp) > shl(sub(Q_synth_hf, 4), 1))
        {
            Q_shb = sub(Q_synth_hf, 4);
            st_fx->EnergyLT_fx = L_shr(st_fx->EnergyLT_fx, sub(st_fx->prev_Q_new, shl(Q_shb, 1)));
        }
        ELSE
        {
            Q_shb = shr(add(st_fx->prev_Q_new, exp), 1);
            IF(s_and(exp, 0x0001) == 1)
            {
                exp = sub(exp, 1);
            }
            st_fx->EnergyLT_fx = L_shl(st_fx->EnergyLT_fx, exp);
        }
    }
    ELSE
    {
        Q_shb = sub(Q_synth_hf, 4);
    }
    Copy_Scale_sig(new_input_hp_fx, new_input_hp_fx, L_FRAME16k, sub(Q_shb, Q_shb_speech));
    /* SWB BWE encoding */
    IF (sub(st_fx->L_frame_fx, L_FRAME16k) == 0)
    {
        SWB_BWE_encoding_fx( st_fx, old_input_fx, old_input_lp_fx, new_input_hp_fx, old_syn_12k8_16k_fx, yorig_fx,
                             SWB_fenv_fx, tilt_nb_fx, 80, coder_type, Q_slb_speech, Q_shb, Q_synth_hf, Q_synth );

    }
    ELSE
    {
        SWB_BWE_encoding_fx( st_fx, old_input_fx, old_input_lp_fx, new_input_hp_fx, old_syn_12k8_16k_fx, yorig_fx,
        SWB_fenv_fx, tilt_nb_fx, 6, coder_type, Q_slb_speech, Q_shb, Q_synth_hf, Q_synth );
    }

    /* FB BWE encoding */
    IF ( sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        energy_fbe_fb_fx = L_deposit_l(0);
        FOR( i=FB_BAND_BEGIN; i<FB_BAND_END; i++)
        {
            tmp = shr(yorig_fx[i], 4);
            energy_fbe_fb_fx = L_mac0(energy_fbe_fb_fx, tmp, tmp); /*2*(Q_synth_hf-4) */
        }
        ener_low_fx = 0;
        move16();
        FOR( i=FB_BAND_BEGIN - FB_BAND_WIDTH; i<FB_BAND_BEGIN; i++)
        {
            tmp = shr(yorig_fx[i], 4);
            ener_low_fx = L_mac0(ener_low_fx, tmp, tmp); /*2*(Q_synth_hf-4) */
        }

        IF(energy_fbe_fb_fx != 0)
        {
            exp = norm_l(energy_fbe_fb_fx);
            frac = extract_h(L_shl(energy_fbe_fb_fx, exp));
            tmp = div_s(16384, frac);   /*15+14-(exp+2*(Q_synth_hf-4)-16) -->45-(exp+2*(Q_synth_hf-4)) */
            L_tmp = Mult_32_16(ener_low_fx, tmp);  /*45-(exp+2*(Q_synth_hf-4)) + 2*(Q_synth_hf-4) - 15 = 30-exp */
            exp1 = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp1);
            exp = 31-exp1-(30-exp);
            L_tmp = Isqrt_lc(L_tmp, &exp);  /*31-exp */
            fb_ener_adjust_fx = round_fx(L_shl(L_tmp, exp)); /*Q15 */

        }
        ELSE
        {
            fb_ener_adjust_fx = 0;
            move16();
        }

        fb_ener_adjust_fx = s_min(fb_ener_adjust_fx, 16384);  /*Q15 */
        idxGain = usquant_fx( fb_ener_adjust_fx, &ener_adjust_quan_fx, 0, 512, shl(1, NUM_BITS_FB_FRAMEGAIN) );
    }

    /* write FB BWE frame gain to the bitstream */
    IF( sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        push_indice_fx(st_fx, IND_FB_SLOPE, idxGain, NUM_BITS_FB_FRAMEGAIN );
    }

    return;
}
/*==========================================================================*/
/* FUNCTION      :   static Word16 WB_BWE_fenv_q_fx()            */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :   Scalar quantizer routine                */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                            */
/* Word16 *cb                  i:   quantizer codebook          Q10 */
/* Word16 cb_length            i:   length of codebook            */
/* Word16 cb_dim               i:   dimension of codebook          */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                            */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                          */
/* Word16 *x                   i/o: energy of WB envelop        Q10 */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                            */
/*           _ None                          */
/*--------------------------------------------------------------------------*/
/*                                      */
/*==========================================================================*/
static Word16 WB_BWE_fenv_q_fx(    /* o:   quantized gain index  */
    Word16 *x,                  /* i/o: energy of WB envelop  Q10*/
    const Word16 *cb,                 /* i:   quantizer codebook   Q10 */
    const Word16 cb_length,           /* i:   length of codebook    */
    const Word16 cb_dim               /* i:   dimension of codebook */
)
{
    Word16 i, j, indx = 0;
    Word32 dist, min_dist;
    const Word16 *pit = cb;/*Q10 */
    Word16 tmp;
    Word32 L_tmp;

    min_dist = L_add(MAX_32, 0);
    FOR (i=0; i<cb_length; i++ )
    {
        dist = L_deposit_l(0);
        FOR (j=0; j<cb_dim; j++)
        {
            tmp = sub(x[j], *pit);/*Q10 */
            L_tmp = L_mult0(tmp, tmp);/*Q(10+10)->Q20 */
            dist = L_add(dist, L_tmp);

            pit++;
        }

        IF( L_sub(dist,min_dist) < 0)
        {
            min_dist = L_add(dist, 0);
            indx = i;
            move16();
        }
    }

    FOR(j=0; j<cb_dim; j++)
    {
        x[j] = cb[cb_dim*indx+j];
        move16();
    }

    return (indx);
}

static void get_normalize_spec_fx(
    const Word16 core,                   /* i  : core selected            */
    const Word16 extl,                   /* i  : extension layer selected */
    const Word16 mode,                   /* i  : SHB BWE class            */
    const Word16 core_type,              /* i  : coding type              */
    const Word16 *org_fx,                /* i  : input spectrum           */
    Word16 *SWB_signal,            /* o  : output spectrum          */
    Word16 *prev_L_swb_norm,       /* i  : previous norm. len       */
    const Word16 offset ,                /* i  : frequency offset         */
    Word16 Q_new_lf
)
{
    Word16 n_freq, L_swb_norm;
    Word32 envelope[L_FRAME32k];
    Word16 frq_end;
    Word16 tmp;
    Word16 exp;
    Word32 L_tmp_m;

    set16_fx(SWB_signal, 0, add(HQ_GENERIC_HIGH0,offset) );
    calc_normal_length_fx(core, org_fx, mode, extl, &L_swb_norm, prev_L_swb_norm, Q_new_lf);
    test();
    IF( sub(extl,SWB_BWE) == 0 || sub(extl,FB_BWE) == 0 )
    {
        IF( sub(mode,HARMONIC) == 0 )
        {
            Copy( org_fx, &SWB_signal[add(240,offset)], 240 );
            Copy( &org_fx[128], &SWB_signal[add(480,offset)], 80 );
        }
        ELSE
        {
            Copy( &org_fx[112], &SWB_signal[add(240,offset)], 128 );
            Copy( &org_fx[112], &SWB_signal[add(368,offset)], 128 );
            Copy( &org_fx[176], &SWB_signal[add(496,offset)], 64 );
        }
        frq_end = add(560,offset);
    }
    ELSE IF (sub(extl,WB_BWE) == 0)
    {
        IF( core_type == 0 )
        {
            Copy(&org_fx[160], &SWB_signal[240], 80);
        }
        ELSE
        {
            Copy(&org_fx[80], &SWB_signal[240], 80);
        }
        frq_end = L_FRAME16k;
        move16();
    }
    ELSE
    {
        Copy( org_fx+HQ_GENERIC_OFFSET, SWB_signal+add(HQ_GENERIC_HIGH0,offset), HQ_GENERIC_LEN0 );
        Copy( org_fx+HQ_GENERIC_OFFSET, SWB_signal+add(HQ_GENERIC_HIGH1,offset), HQ_GENERIC_LEN0 );
        IF ( sub(offset,HQ_GENERIC_FOFFSET_24K4 ) == 0)
        {
            Copy( org_fx+HQ_GENERIC_LOW0,   SWB_signal+add(HQ_GENERIC_HIGH2,offset), sub(HQ_GENERIC_END_FREQ,HQ_GENERIC_HIGH2) );
        }
        frq_end = sub(L_FRAME32k, offset);
    }

    /* calculate envelope */
    calc_norm_envelop_fx( SWB_signal, envelope, L_swb_norm, sub(frq_end,offset), offset );

    /* Normalize with envelope */
    FOR( n_freq = add(swb_bwe_subband_fx[0],offset); n_freq<frq_end; n_freq++ )
    {
        IF(envelope[n_freq] != 0)
        {
            exp = norm_l(envelope[n_freq]);
            tmp = extract_h(L_shl(envelope[n_freq], exp));
            exp = sub(sub(30,exp), Q_new_lf);
            tmp = div_s(16384,tmp); /*Q(15+exp) */
            L_tmp_m = L_shr(L_mult0(SWB_signal[n_freq],tmp),add(exp,Q_new_lf)); /*Q15 */
            SWB_signal[n_freq] = extract_l(L_tmp_m); /*Q15 */
        }
        ELSE
        {
            SWB_signal[n_freq] = 1;
            move16();
        }
    }

    return;
}

/*-------------------------------------------------------------------*
* FD_BWE_class()
*
* classify signal of above 6.4kHz, can be used for WB/SWB switch
*-------------------------------------------------------------------*/
static Word16 FD_BWE_class_fx(     /* o  : FD BWE class        */
    const Word16 *fSpectrum,          /* i  : input spectrum      */
    const Word32 fGain,               /* i  : global gain         */
    const Word16 tilt_nb,             /* i  : BWE tilt            */
    Word16 Q_syn,
    Word16 Q_shb,
    Encoder_State_fx *st_fx             /* i/o: Encoder structure   */
)
{
    Word16 i, j, k, noise, sharpMod = 0;
    Word16 peak, mag;
    Word32 mean[20];
    Word16 sharpPeak;
    const Word16 *input_hi = 0;
    Word16 sharp;
    Word16 gain_tmp = 0;
    Word16 mode;
    Word32 L_meanH, L_mean_d, L_tmp;
    Word16 sharplimit;
    Word16 numsharp, num, den;
    Word16 numharmonic, tmp, expn, expd, scale;

    mode = NORMAL;
    move16();
    k = 0;
    move16();
    noise = 0;
    move16();
    sharpPeak = 0;
    move16();
    numsharp = 0;
    move16();
    numharmonic = 4;
    move16();
    sharplimit = 10;
    move16();

    L_mean_d = 0L;      /* to avoid compilation warnings */

    test();
    IF ( sub(st_fx->extl_fx, SWB_BWE) == 0 || sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        input_hi = &fSpectrum[256];
        move16();
        numsharp = NUM_SHARP;
        move16();

        test();
        test();
        test();
        IF ( ( sub(st_fx->last_extl_fx, SWB_BWE) == 0 && sub(st_fx->extl_fx, SWB_BWE) == 0 ) || ( sub(st_fx->last_extl_fx, FB_BWE) == 0 && sub(st_fx->extl_fx, FB_BWE) == 0 ) )
        {
            IF(st_fx->prev_global_gain_fx == 0)
            {
                gain_tmp = round_fx(L_shl(fGain, 30)); /*Q14 */
            }
            ELSE
            {
                expn = norm_l(fGain);
                num = extract_h(L_shl(fGain, expn));
                expn = sub(sub(30, expn), shl(Q_shb,1));

                expd = norm_l(st_fx->prev_global_gain_fx);
                den = extract_h(L_shl(st_fx->prev_global_gain_fx, expd));
                expd = sub(sub(30, expd), shl(st_fx->prev_Q_shb,1));

                scale = shr(sub(den, num), 15);
                num = shl(num, scale);
                expn = sub(expn, scale);

                tmp = div_s(num, den);
                expn = sub(expn, expd);
                gain_tmp = shl(tmp, sub(expn,1));/*Q14 */
            }
            test();
            IF (sub(st_fx->prev_mode_fx,TRANSIENT) == 0)
            {
                numharmonic = shl(numharmonic, 1);
            }
            ELSE IF (sub(st_fx->prev_mode_fx, NORMAL) == 0 || sub(st_fx->prev_mode_fx, NOISE) == 0)
            {
                numharmonic = add(shr(numharmonic, 1), numharmonic);
            }
        }
        ELSE
        {
            gain_tmp = 16384;
            move16();
            IF (sub(st_fx->prev_mode_fx, HARMONIC) == 0)
            {
                numharmonic = shr(numharmonic, 1);
                sharplimit = shr(sharplimit, 1);
            }
            ELSE
            {
                numharmonic = shl(numharmonic, 1);
                sharplimit = shl(sharplimit, 1);
            }
        }
    }
    ELSE IF (sub(st_fx->extl_fx, WB_BWE) == 0)
    {
        input_hi = &fSpectrum[224];
        move16();
        numsharp = 3;
        move16();

        IF (sub(st_fx->prev_mode_fx, HARMONIC) == 0)
        {
            numharmonic = shr(numharmonic, 2);
        }
        ELSE
        {
            numharmonic = shr(numharmonic, 1);
        }
        IF (sub(st_fx->last_extl_fx, WB_BWE) != 0)
        {
            IF (sub(st_fx->prev_mode_fx, HARMONIC) == 0)
            {
                sharplimit = shr(sharplimit, 1);
            }
            ELSE
            {
                sharplimit = shl(sharplimit, 1);
            }
        }
    }

    L_meanH = L_deposit_l(0);
    FOR(i = 0; i < numsharp; i ++)
    {
        peak = 0;
        move16();
        mean[i] = L_deposit_l(0);

        FOR(j = 0; j < SHARP_WIDTH; j ++)
        {
            mag = abs_s(*input_hi);
            IF (sub(mag, peak) > 0)
            {
                peak = mag;
                move16();/*Q_syn */
            }
            mean[i] = L_add(mean[i], mag);
            move32();/*Q_syn */
            input_hi ++;
        }

        L_meanH = L_add(L_meanH, mean[i]);/*Q_syn */

        IF(L_sub(mean[i], L_deposit_l(peak)) != 0)
        {
            L_tmp = L_sub(mean[i], peak);/*Q_syn */
            L_tmp = Mult_32_16(L_tmp, 16913); /* 1/31->Q19 -> Q_syn+19-15 */
            den = extract_l(L_shr(L_tmp, 4));  /*Q_syn */
            IF(den == 0)
            {
                den = 1;
                move16();
            }
            expd = norm_s(den);
            tmp = div_s(shl(1,sub(14,expd)), den); /*Q(29-expd-Q_syn) */
            L_tmp = L_mult(tmp, peak); /*Q(30-expd) */
            sharp = round_fx(L_shl(L_tmp, sub(expd, 4)));/*Q10 */
        }
        ELSE
        {
            sharp = 0;
            move16();
        }

        test();
        IF (sub(sharp, 4608) > 0 && sub(peak, shl(1, add(Q_syn, 3))) > 0)
        {
            k = add(k, 1);
            move16();
        }
        ELSE IF (sub(sharp, 3072) < 0)
        {
            noise = add(noise, 1);
            move16();
        }

        IF (sub(sharp, sharpPeak) > 0)
        {
            sharpPeak = sharp;
            move16();
        }
    }
    test();
    IF ( sub(st_fx->extl_fx, SWB_BWE) == 0 || sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        test();
        test();
        test();
        IF(sub(k, numharmonic) >= 0 && sub(gain_tmp, 8192) > 0 && sub(gain_tmp, 29491) < 0 && sub(sharpPeak, shl(sharplimit, 10)) > 0)
        {
            sharpMod = 1;
            move16();
        }
        ELSE
        {
            sharpMod = 0;
            move16();
        }

        L_meanH = Mult_32_16(L_meanH, 29127); /*Q_syn+8 */
        L_mean_d = 0;
        move16();
        FOR(i=0; i<NUM_SHARP; i++)
        {
            L_tmp = L_sub(L_shl(mean[i], 8-5), L_meanH); /*Q_syn+8 */
            L_mean_d = L_add(L_mean_d, L_abs(L_tmp)); /*Q_syn+8 */
        }
    }
    ELSE IF (sub(st_fx->extl_fx, WB_BWE) == 0)
    {
        test();
        IF (sub(k,numharmonic) >= 0 && sub(sharpPeak,shl(sharplimit, 10)) > 0)
        {
            sharpMod = 1;
            move16();
        }
        ELSE
        {
            sharpMod = 0;
            move16();
        }
    }

    test();
    test();
    IF (sharpMod && sub(st_fx->modeCount_fx, 12) < 0)
    {
        st_fx->modeCount_fx = add(st_fx->modeCount_fx, 1);
    }
    ELSE IF (sharpMod == 0 && st_fx->modeCount_fx > 0)
    {
        st_fx->modeCount_fx = sub(st_fx->modeCount_fx, 1);
    }

    IF (sub(st_fx->modeCount_fx, 2) >= 0)
    {
        sharpMod = 1;
        move16();
    }

    test();
    IF (sharpMod)
    {
        mode = HARMONIC;
        move16();
    }
    ELSE IF ( sub(st_fx->extl_fx, SWB_BWE) == 0 || sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        L_tmp = Mult_32_16(L_mean_d, 6827); /*Q_syn+8 ; 1/4.8 in Q15 */

        test();
        test();
        test();
        IF (sub(noise, 4) > 0 && (L_sub(L_tmp, L_meanH) < 0 ||  L_meanH == 0) && sub(tilt_nb, 10240) < 0)
        {
            mode = NOISE;
            move16();
        }
    }

    return (mode);
}
/*-------------------------------------------------------------------*
* freq_weights_fx()
*
*-------------------------------------------------------------------*/
static void freq_weights_fx(
    const Word16 Band_Ener[],            /* i  : Band energy              Q8 */
    const Word16 f_weighting[],         /* i  : weigting coefs.         Q15 */
    Word16 w[],                   /* o  : Freq. weighting         Q13 */
    const Word16 Nb                     /* i  : Number of bands             */
)
{
    Word16 i;
    Word16 tmp, tmp1, w1[SWB_FENV], w2[SWB_FENV];
    Word16 min_b, max_b;
    Word32 L_tmp;
    Word16 exp;

    /* Find Max band energy */
    min_b = Band_Ener[0];
    move16();
    max_b = Band_Ener[0];
    move16();
    FOR( i=1; i<Nb; i++ )
    {
        IF( sub(Band_Ener[i], min_b) < 0 )
        {
            min_b = Band_Ener[i];
            move16();/*Q8 */
        }

        IF( sub(Band_Ener[i],max_b) > 0 )
        {
            max_b = Band_Ener[i];
            move16();/*Q8 */
        }
    }

    /* Find weighting function */
    tmp = sub(max_b, min_b);/*Q8 */
    IF(tmp != 0)
    {
        exp = norm_s(tmp);
        tmp = div_s(shl(1,sub(14,exp)), tmp); /*(21-exp) */
        tmp = shl(tmp, sub(exp, 6)); /*Q15 */
    }
    ELSE
    {
        tmp = 32767;
        move16();
    }

    FOR( i=0; i<Nb; i++ )
    {
        tmp1 = sub(Band_Ener[i], min_b);
        L_tmp = L_mult(tmp1, tmp);/*Q24 */
        L_tmp = L_add(L_tmp, 16777216);/*Q24 */
        L_tmp = L_shl(L_tmp, 5);/*Q29 */
        w1[i] = round_fx(L_tmp);/*Q13 */
        w2[i] = f_weighting[i];
        move16();/*Q15                         */ /*1~0.75*/
        w[i] = mult_r(w1[i], w2[i]);
        move16();/*Q13 */
    }

    return;
}

/*-------------------------------------------------------------------*
* vqWithCand_w_fx()
*
*-------------------------------------------------------------------*/
static void vqWithCand_w_fx(
    const Word16 *x,              /* i  : input vector                             Q8 */
    const Word16 *E_ROM_dico,        /* i  : codebook                                 Q8 */
    const Word16 dim,            /* i  : codebook dimension                          */
    const Word16 E_ROM_dico_size,      /* i  : codebook size                               */
    Word16 *index,            /* o  : survivors indices                           */
    const Word16 surv,            /* i  : survivor number                             */
    Word32 dist_min[],        /* o  : minimum distortion                       Q5 */
    const Word16 *w,            /* i  : weighting                                Q13*/
    const Word16 flag            /* i  : flag indicationg weighted distortion metric */
)
{
    Word16 i, j, k, l;
    const Word16 *p_E_ROM_dico;
    Word16 dist, temp1;
    Word32 L_dist,L_tmp;

    IF( flag )
    {
        set32_fx( dist_min, MAX_32, surv );  /* FLT_MAX */

        FOR( i = 0; i < surv; i++ )
        {
            index[i] = i;
            move16();
        }

        p_E_ROM_dico = E_ROM_dico;
        move16();

        FOR( i = 0; i < E_ROM_dico_size; i++ )
        {
            dist = sub(x[0],*p_E_ROM_dico++);/*Q8 */
            L_dist = L_mult(dist,w[0]);/*Q22 */
            L_dist = Mult_32_16(L_dist,dist);/*Q15 */
            L_dist = L_shr(L_dist,10);/*Q5 */

            FOR( j = 1; j < dim; j++ )
            {
                temp1 = sub(x[j],*p_E_ROM_dico++);
                L_tmp = L_mult(temp1,w[j]);/*Q22 */
                L_tmp = Mult_32_16(L_tmp,temp1);/*Q15 */
                L_dist = L_add(L_dist,L_shr(L_tmp,10));/*Q5 */
            }

            FOR( k = 0; k < surv; k++ )
            {
                IF( L_sub(L_dist,dist_min[k]) < 0 )
                {
                    FOR( l = surv - 1; l > k; l-- )
                    {
                        dist_min[l] = dist_min[l - 1];
                        move32();
                        index[l] = index[l - 1];
                        move16();
                    }
                    dist_min[k] = L_dist;
                    move32();
                    index[k] = i;
                    move16();
                    BREAK;
                }
            }
        }
    }
    ELSE
    {
        set32_fx( dist_min, MAX_32, surv );  /* FLT_MAX */

        FOR (i = 0; i < surv; i++)
        {
            index[i] = i;
            move16();
        }

        p_E_ROM_dico = E_ROM_dico;
        move16();

        FOR( i = 0; i < E_ROM_dico_size; i++ )
        {
            dist = sub(x[0],*p_E_ROM_dico++);/*Q8 */
            L_dist = L_mult(dist,dist);/*Q17 */
            L_dist = L_shr(L_dist,12);/*Q5 */

            FOR( j = 1; j < dim; j++ )
            {
                temp1 = sub(x[j],*p_E_ROM_dico++);/*Q8 */
                L_tmp = L_mult(temp1,temp1);/*Q17 */
                L_dist = L_add(L_dist,L_shr(L_tmp,12));/*Q5 */
            }

            FOR( k = 0; k < surv; k++ )
            {
                IF( L_sub(L_dist,dist_min[k]) < 0 )
                {
                    FOR( l = surv - 1; l > k; l-- )
                    {
                        dist_min[l] = dist_min[l-1];
                        move32();
                        index[l] = index[l-1];
                        move16();
                    }
                    dist_min[k] = L_dist;
                    move32();
                    index[k] = i;
                    move16();
                    BREAK;
                }
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
* vqSimple_w_fx()
*
*-------------------------------------------------------------------*/

static Word16 vqSimple_w_fx(
    const Word16 *x,                 /* i  : input for quantizer                     Q8  */
    Word16 *y,                 /* i  : quantized value                         Q8  */
    const Word16 *cb,                /* i  : codebooks                               Q8  */
    const Word16 *w,                 /* i  : weight                                  Q13 */
    const Word16 dim,                /* i  : dimension                                   */
    const Word16 l,                  /* i  : number of candidates                        */
    const Word16 flag                /* i  : flag indicationg weighted distortion metric */
)
{
    Word16 i, j, index;
    const Word16 *cbP;
    Word16 dist, temp;
    Word32 L_dist,L_tmp,L_dist_min;

    index = 0;
    move16();
    L_dist_min = L_add(MAX_32, 0); /* FLT_MAX */
    cbP = cb;
    move16();
    IF( flag )
    {
        FOR( i = 0; i < l; i++ )
        {
            /*dist = x[0] - *cbP++; */
            /*dist *= (dist * w[0]); */
            dist = sub(x[0],*cbP++);/*Q8 */
            L_dist = L_mult(dist,w[0]);/*Q22 */
            L_dist = Mult_32_16(L_dist,dist);/*Q15 */
            L_dist = L_shr(L_dist,10);/*Q5 */

            FOR( j = 1; j < dim; j++ )
            {
                /*temp = x[j] - *cbP++; */
                /*dist += temp * temp * w[j]; */
                temp = sub(x[j],*cbP++);
                L_tmp = L_mult(temp,w[j]);/*Q22 */
                L_tmp = Mult_32_16(L_tmp,temp);/*Q15 */
                L_dist = L_add(L_dist,L_shr(L_tmp,10));/*Q5 */
            }
            IF (L_sub(L_dist, L_dist_min) < 0)
            {
                L_dist_min = L_add(L_dist, 0);/*Q5 */
                index = i;
                move16();
            }
        }
    }
    ELSE
    {
        FOR( i = 0; i < l; i++ )
        {
            /*dist = x[0] - *cbP++; */
            dist = sub(x[0],*cbP++);
            /*dist *= dist; */
            L_dist = L_mult(dist,dist);/*Q17 */
            L_dist = L_shr(L_dist,12);

            FOR( j = 1; j < dim; j++ )
            {
                /*temp = x[j] - *cbP++; */
                temp = sub(x[j] , *cbP++);
                /*dist += temp * temp; */
                L_tmp = L_mult(temp,temp);/*Q17 */
                L_dist = L_add(L_dist,L_shr(L_tmp,12));/*Q5 */
            }
            IF (L_sub(L_dist, L_dist_min) < 0)
            {
                L_dist_min = L_add(L_dist, 0);
                index = i;
                move16();
            }
        }
    }


    /* Reading the selected vector */
    Copy( &cb[index * dim], y, dim );

    return(index);
}



/*-------------------------------------------------------------------*
* MSVQ_Interpol_Tran_fx()
*
*-------------------------------------------------------------------*/
static void MSVQ_Interpol_Tran_fx(
    Word16 *SWB_env_energy,  /* i/o  : (original/quantized) energy   Q8 */
    Word16 *indice           /* o    : quantized index                  */
)

{
    Word16 k, n_band, candInd[N_CAND_TR], ind_tmp[2],tmp;
    Word16 env_temp11[SWB_FENV_TRANS/2], env_temp12[SWB_FENV_TRANS/2];
    Word16 tmp_q;
    Word16 quant_tmp[SWB_FENV_TRANS], quant_tmp2[SWB_FENV_TRANS];
    Word16 quant_select[SWB_FENV_TRANS];
    Word32 L_tmp, L_dist, L_minDist,distCand[N_CAND_TR];

    /* Extract target vector */
    FOR( n_band = 0; n_band < DIM_TR1; n_band++ )
    {
        env_temp11[n_band] = SWB_env_energy[2*n_band];
        move16();/*Q8 */
        env_temp12[n_band] = SWB_env_energy[2*n_band+1];
        move16();/*Q8 */
    }

    vqWithCand_w_fx( env_temp11, Env_TR_Cdbk1_fx, DIM_TR1, N_CB_TR1, candInd, N_CAND_TR, distCand, NULL, 0 );

    L_minDist = L_add(MAX_32, 0); /* FLT_MAX */

    FOR( k=0; k<N_CAND_TR; k++ )
    {
        FOR( n_band = 0; n_band < DIM_TR1; n_band++ )
        {
            quant_tmp[n_band] = Env_TR_Cdbk1_fx[add(shl(candInd[k],1), n_band)];
            move16();  /*DIM_TR1 == 2*/
        }

        FOR( n_band = 0; n_band < DIM_TR2-1; n_band++ )
        {
            /*quant_tmp2[n_band] = env_temp12[n_band] - ((quant_tmp[n_band]+quant_tmp[n_band+1])/2.f); */
            tmp = add(quant_tmp[n_band], quant_tmp[n_band+1]);/*Q8 */
            tmp = shr(tmp,1);
            quant_tmp2[n_band] = sub(env_temp12[n_band],tmp);
            move16();/*Q8 */
        }
        /*quant_tmp2[n_band] = env_temp12[n_band] - quant_tmp[n_band]; */
        quant_tmp2[n_band] = sub(env_temp12[n_band] , quant_tmp[n_band]);
        move16();/*Q8 */
        ind_tmp[0] = vqSimple_w_fx( quant_tmp2, quant_tmp2, Env_TR_Cdbk2_fx, NULL, DIM_TR2, N_CB_TR2,0 );
        move16();

        FOR( n_band = 0; n_band < DIM_TR1; n_band++ )
        {
            quant_select[n_band*2] = quant_tmp[n_band];
            move16();
        }

        FOR( n_band = 0; n_band < DIM_TR2-1; n_band++ )
        {
            /*quant_select[n_band*2+1] = ((quant_tmp[n_band]+quant_tmp[n_band+1])/2.f) + quant_tmp2[n_band]; */
            tmp = add(quant_tmp[n_band], quant_tmp[n_band+1]);/*Q8 */
            tmp = shr(tmp,1);
            quant_select[n_band*2+1] = add(tmp,quant_tmp2[n_band]);
            move16();
        }
        /*quant_select[n_band*2+1] = quant_tmp[n_band]+quant_tmp2[n_band]; */
        quant_select[n_band*2+1] = add(quant_tmp[n_band],quant_tmp2[n_band]);
        move16();

        L_dist = L_deposit_l(0);
        FOR( n_band = 0; n_band < SWB_FENV_TRANS; n_band++ )
        {
            /*tmp_q = SWB_env_energy[n_band] - quant_select[n_band]; */
            tmp_q = sub(SWB_env_energy[n_band] , quant_select[n_band]);
            /*dist += tmp_q*tmp_q; */
            L_tmp = L_mult(tmp_q,tmp_q);/*Q17 */
            L_dist = L_add(L_dist,L_shr(L_tmp,12));/*Q5 */
        }

        /* Check optimal candidate */
        IF (L_sub(L_dist, L_minDist) < 0)
        {
            L_minDist = L_add(L_dist, 0);
            indice[0] = candInd[k];
            move16();
            indice[1] = ind_tmp[0];
            move16();
        }
    }
    return;
}

/*-------------------------------------------------------------------*
* MSVQ_Interpol_fx()
*
*-------------------------------------------------------------------*/

static void msvq_interpol_fx(
    Word16 *SWB_env_energy,          /* i/o  : (original/quantized) energy   Q8*/
    Word16 *w_env,                   /* i/o  : weighting coffecients         Q13*/
    Word16 *indice                   /* o    : quantized index               */
)
{
    Word16 k, n_band,n_band2,n_band2p1, candInd[N_CAND], ind_tmp[4];
    Word16 tmp_q;
    Word16 env_temp11[SWB_FENV/2], env_temp12[SWB_FENV/2];
    Word16 quant_tmp[SWB_FENV], quant_tmp1[SWB_FENV], quant_tmp2[SWB_FENV];
    Word16 quant_select[SWB_FENV], w_env11[SWB_FENV/2], w_env12[SWB_FENV/2],tmp;
    Word32 L_tmp, distCand[N_CAND], L_dist, L_minDist;
    Word16 synth_energy[SWB_FENV];


    /* Extract target vector */
    FOR(n_band = 0; n_band < DIM11; n_band++)
    {
        n_band2 = shl(n_band,1);
        n_band2p1 =add(n_band2,1);
        env_temp11[n_band] = SWB_env_energy[n_band2];
        move16();/*Q8 */
        env_temp12[n_band] = SWB_env_energy[n_band2p1];
        move16();/*Q8 */

        w_env11[n_band] = w_env[n_band2];
        move16();/*Q13 */
        w_env12[n_band] = w_env[n_band2p1];
        move16();/*Q13 */
    }

    vqWithCand_w_fx( env_temp11, EnvCdbk11_fx, DIM11, N_CB11, candInd, N_CAND, distCand, w_env11, 1 );

    L_minDist = L_add(MAX_32, 0); /* FLT_MAX */

    FOR( k=0; k<N_CAND; k++ )
    {
        FOR( n_band = 0; n_band < DIM11; n_band++ )
        {
            quant_tmp1[n_band] = EnvCdbk11_fx[add(i_mult2(candInd[k], DIM11), n_band)];
            move16(); /*Q8 */
            quant_tmp2[n_band] = sub(env_temp11[n_band] , quant_tmp1[n_band]);
            move16(); /*Q8 */
        }

        ind_tmp[0] = vqSimple_w_fx( quant_tmp2, quant_tmp2, EnvCdbk1st_fx, w_env11, DIM1ST, N_CB1ST,  1 );
        ind_tmp[1] = vqSimple_w_fx( quant_tmp2+DIM1ST, quant_tmp2+DIM1ST, EnvCdbk2nd_fx, w_env11+DIM1ST, DIM2ND, N_CB2ND,  1 );

        /* Extract vector for odd position */
        FOR( n_band = 0; n_band < DIM11; n_band++ )
        {
            quant_tmp[n_band] = add(quant_tmp1[n_band] , quant_tmp2[n_band]);
            move16();
        }

        FOR( n_band = 0; n_band < DIM12-1; n_band++ )
        {
            tmp = add(quant_tmp[n_band], quant_tmp[n_band+1]);/*Q8 */
            tmp = shr(tmp,1);
            quant_tmp2[n_band] = sub(env_temp12[n_band],tmp);
            move16();/*Q8 */
        }

        /*quant_tmp2[n_band] = env_temp12[n_band]-quant_tmp[n_band]; */
        quant_tmp2[n_band] = sub(env_temp12[n_band],quant_tmp[n_band]);
        move16();/*Q8 */

        ind_tmp[2] = vqSimple_w_fx( quant_tmp2, quant_tmp2, EnvCdbk3rd_fx, w_env12, DIM3RD, N_CB3RD,  1 );
        move16();
        ind_tmp[3] = vqSimple_w_fx( quant_tmp2+DIM3RD, quant_tmp2+DIM3RD, EnvCdbk4th_fx, w_env12+DIM3RD, DIM4TH, N_CB4TH, 1 );
        move16();

        FOR( n_band = 0; n_band < DIM11; n_band++ )
        {
            quant_select[n_band*2] = quant_tmp[n_band];
            move16(); /*Q8 */
        }

        FOR( n_band = 0; n_band < DIM12-1; n_band++ )
        {
            tmp = add(quant_tmp[n_band],quant_tmp[n_band+1]);
            tmp = shr(tmp,1);
            quant_select[add(shl(n_band,1),1)] = add (tmp,quant_tmp2[n_band]);
            move16();/*Q8 */
        }
        quant_select[add(shl(n_band,1),1)] = add(quant_tmp[n_band] , quant_tmp2[n_band]);
        move16();/*Q8 */

        L_dist = L_deposit_l(0);
        FOR( n_band = 0; n_band < SWB_FENV; n_band++ )
        {
            tmp_q = sub(SWB_env_energy[n_band] , quant_select[n_band]); /*Q8 */
            L_tmp = L_mult(tmp_q,tmp_q);/*Q17 */
            L_tmp = Mult_32_16(L_tmp,w_env[n_band]);/*Q15 */
            L_dist = L_add(L_dist,L_shr(L_tmp,10));
        }

        /* Check optimal candidate */
        IF (L_sub(L_dist, L_minDist) < 0)
        {
            L_minDist = L_add(L_dist, 0);

            Copy( quant_select, synth_energy, SWB_FENV );

            indice[0] = candInd[k];
            move16();
            indice[1] = ind_tmp[0];
            move16();
            indice[2] = ind_tmp[1];
            move16();
            indice[3] = ind_tmp[2];
            move16();
            indice[4] = ind_tmp[3];
            move16();
        }
    }

    Copy( synth_energy, SWB_env_energy, SWB_FENV );

    return;
}

/*-------------------------------------------------------------------*
 * msvq_interpol_2_fx()
 *
 *-------------------------------------------------------------------*/
static void msvq_interpol_2_fx(
    Word16 *hq_generic_fenv,   /* i/o: (original/quantized) energy */
    const Word16 *w_env,             /* i  : weighting coffecients       */
    Word16 *indice,            /* o  : quantized index             */
    const Word16 nenv                /* i  : the number of envelopes     */
)
{
    Word16 k, n_band,n_band2, candInd[N_CAND], ind_tmp[4];
    Word16 tmp_q;
    Word16 env_temp11[SWB_FENV/2], env_temp12[SWB_FENV/2];
    Word16 quant_tmp[SWB_FENV], quant_tmp1[SWB_FENV], quant_tmp2[SWB_FENV];
    Word16 quant_select[SWB_FENV], w_env11[SWB_FENV/2], w_env12[SWB_FENV/2];
    Word32 L_tmp, distCand[N_CAND], L_dist, L_minDist;
    Word16 synth_energy[SWB_FENV];

    /* Extract target vector */
    FOR(n_band = 0; n_band < DIM11-1; n_band++)
    {
        n_band2 = shl(n_band,1);
        env_temp11[n_band] = hq_generic_fenv[n_band2];
        move16();/*Q8 */
        w_env11[n_band] = w_env[n_band2];
        move16();/*Q13 */
    }
    env_temp11[DIM11-1] = hq_generic_fenv[2*(DIM11-2)+1];
    move16();/*Q8 */
    w_env11[DIM11-1] = w_env[2*(DIM11-2)+1];
    move16();/*Q13 */

    env_temp12[0] = hq_generic_fenv[0];
    move16();/*Q8 */
    w_env12[0] = w_env[0];
    move16();/*Q13 */
    FOR(n_band = 1; n_band < DIM11-1; n_band++)
    {
        n_band2 = sub(shl(n_band,1),1);
        env_temp12[n_band] = hq_generic_fenv[n_band2/*2*n_band-1*/];
        move16();/*Q8 */
        w_env12[n_band] = w_env[n_band2/*2*n_band-1*/];
        move16();/*Q13 */
    }

    vqWithCand_w_fx( env_temp11, EnvCdbk11_fx, DIM11, N_CB11, candInd, N_CAND, distCand, w_env11, 1 );

    L_minDist = L_add(MAX_32, 0); /* FLT_MAX */
    FOR( k=0; k<N_CAND; k++ )
    {
        FOR( n_band = 0; n_band < DIM11; n_band++ )
        {
            quant_tmp1[n_band] = EnvCdbk11_fx[add(i_mult2(candInd[k], DIM11), n_band)];
            move16(); /*Q8 */
            quant_tmp2[n_band] = sub(env_temp11[n_band] , quant_tmp1[n_band]);
            move16(); /*Q8 */
        }

        ind_tmp[0] = vqSimple_w_fx( quant_tmp2, quant_tmp2, EnvCdbk1st_fx, w_env11, DIM1ST, N_CB1ST, 1 );
        ind_tmp[1] = vqSimple_w_fx( quant_tmp2+DIM1ST, quant_tmp2+DIM1ST, EnvCdbk2nd_fx, w_env11+DIM1ST, DIM2ND, N_CB2ND, 1 );

        /* Extract vector for odd position */
        FOR( n_band = 0; n_band < DIM11; n_band++ )
        {
            quant_tmp[n_band] = add(quant_tmp1[n_band] , quant_tmp2[n_band]);
            move16();
        }

        quant_tmp2[0] = sub(env_temp12[0] , quant_tmp[0]);
        move16();
        FOR( n_band = 1; n_band < DIM12-1; n_band++ )
        {
            tmp_q = add(quant_tmp[n_band-1], quant_tmp[n_band]);
            tmp_q = shr(tmp_q,1);
            quant_tmp2[n_band] = sub(env_temp12[n_band], tmp_q);
            move16();
        }

        ind_tmp[2] = vqSimple_w_fx( quant_tmp2, quant_tmp2, EnvCdbk3rd_fx, w_env12, DIM3RD, N_CB3RD, 1 );
        ind_tmp[3] = vqSimple_w_fx( quant_tmp2+DIM3RD, quant_tmp2+DIM3RD, EnvCdbk3rd_fx, w_env12+DIM3RD, DIM3RD, N_CB3RD, 1 );

        FOR( n_band = 0; n_band < DIM12-1; n_band++ )
        {
            quant_select[n_band*2] = quant_tmp[n_band];
            move16();/*Q8 */
        }
        quant_select[11] = quant_tmp[DIM12-1];
        move16();/*Q8 */

        quant_select[0] = add(quant_select[0],quant_tmp2[0]);
        move16();/*Q8 */
        FOR( n_band = 1; n_band < DIM12-1; n_band++ )
        {
            tmp_q = add(quant_tmp[n_band-1], quant_tmp[n_band]);
            tmp_q = shr(tmp_q,1);
            quant_select[sub(shl(n_band,1),1)] = add(quant_tmp2[n_band],tmp_q);
        }

        L_dist = L_deposit_l(0);
        FOR( n_band = 0; n_band < SWB_FENV-2; n_band++ )
        {
            tmp_q = sub(hq_generic_fenv[n_band] , quant_select[n_band]);/*Q8 */
            L_tmp = L_mult(tmp_q,tmp_q);/*Q17 */
            L_tmp = Mult_32_16(L_tmp,w_env[n_band]);/*Q15 */
            L_dist = L_add(L_dist,L_shr(L_tmp,10));
        }

        /* Check optimal candidate */
        IF( L_dist < L_minDist )
        {
            L_minDist = L_add(L_dist, 0);
            Copy( quant_select, synth_energy, SWB_FENV-2 );
            synth_energy[SWB_FENV-2] = 0;
            move16();
            synth_energy[SWB_FENV-1] = 0;
            move16();

            indice[0] = candInd[k];
            move16();
            indice[1] = ind_tmp[0];
            move16();
            indice[2] = ind_tmp[1];
            move16();
            indice[3] = ind_tmp[2];
            move16();
            indice[4] = ind_tmp[3];
            move16();
        }
    }

    Copy( synth_energy, hq_generic_fenv, nenv );

    return;
}


/*-------------------------------------------------------------------*
 * calculate_Tonality_fx()
 *
 * Calculate tonality
 *-------------------------------------------------------------------*/

static void calculate_Tonality_fx(
    const Word16 *org,               /* i  : MDCT coefficients of original              Q_new*/
    const Word16 *gen,               /* i  : MDCT coefficients of generated signal    Q15*/
    Word16 *SFM_org,           /* o  : Spectral Flatness results          Q12*/
    Word16 *SFM_gen,           /* o  : Spectral Flatness results                  Q12*/
    const Word16 length              /* i  : length for calculating tonality         */
)
{
    Word16 n_coeff;
    Word16 inv_len, max;
    Word16 exp,e_tmp,f_tmp;
    Word32 L_tmp,L_tmp2,L_am_org, L_am_gen,L_tmp1;
    Word16 org_spec[80], gen_spec[80];
    Word32 L_log_gm_org, L_log_gm_gen;
    Word16 l_shift;

    /* to reduce dynamic range of original spectrum */
    max = 0;
    move16();
    FOR ( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        org_spec[n_coeff] = abs_s(org[n_coeff]);
        move16();/*Q_new */
        /*test(); */
        /*if( sub(max, org_spec[n_coeff]) < 0) */
        /*{ */
        /*    max = org_spec[n_coeff];move16();//Q_new */
        /*} */
        max = s_max(max, org_spec[n_coeff]);
    }
    l_shift = norm_s(max);
    FOR( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        org_spec[n_coeff] = shl(org_spec[n_coeff],l_shift);
        move16();
        test();
        if (org_spec[n_coeff] == 0)
        {
            org_spec[n_coeff] = shl(1,l_shift);
            move16();
        }
    }

    max = 0;
    move16();
    FOR( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        gen_spec[n_coeff] = abs_s(gen[n_coeff]);
        move16();/*Q15 */
        /*test();
        if( sub(max,gen_spec[n_coeff]) < 0)
        {
            max = gen_spec[n_coeff];move16();
        }*/
        max = s_max(max, org_spec[n_coeff]);
    }
    l_shift = norm_s(max);
    FOR( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        gen_spec[n_coeff] = shl(gen_spec[n_coeff],l_shift);
        move16();
        test();
        if(gen_spec[n_coeff] == 0)
        {
            gen_spec[n_coeff] = shl(1,l_shift);
            move16();
        }
    }

    exp = norm_s(length);
    inv_len = div_s(shl(1,exp),shl(length,exp)); /*Q15 */

    L_am_org = L_deposit_l(0);
    L_am_gen = L_deposit_l(0);
    L_log_gm_org = 0;
    move16();
    L_log_gm_gen = 0;
    move16();

    FOR( n_coeff = 0; n_coeff<length; n_coeff++ )
    {
        L_am_org = L_add(L_am_org,L_deposit_l(org_spec[n_coeff]));/*Q10 */
        L_am_gen = L_add(L_am_gen,L_deposit_l(gen_spec[n_coeff]));/*Q10 */

        IF(org_spec[n_coeff] != 0)
        {
            L_tmp = L_deposit_h(org_spec[n_coeff]); /*Q26 */
            e_tmp = norm_l(L_tmp);
            f_tmp = Log2_norm_lc(L_shl(L_tmp,e_tmp));
            e_tmp = sub(sub(30,e_tmp),26);
            L_tmp = Mpy_32_16(e_tmp, f_tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
            L_log_gm_org = L_add(L_log_gm_org,L_tmp); /*Q14 */
        }

        IF(gen_spec[n_coeff] != 0)
        {
            L_tmp = L_deposit_h(gen_spec[n_coeff]); /*Q26 */
            e_tmp = norm_l(L_tmp);
            f_tmp = Log2_norm_lc(L_shl(L_tmp,e_tmp));
            e_tmp = sub(sub(30,e_tmp),26);
            L_tmp = Mpy_32_16(e_tmp, f_tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
            L_log_gm_gen = L_add(L_log_gm_gen,L_tmp); /*Q14 */
        }
    }

    IF(L_am_org != 0)
    {
        L_tmp = Mult_32_16(L_am_org,inv_len);/*Q10 */
        e_tmp = norm_l(L_tmp);
        f_tmp = Log2_norm_lc(L_shl(L_tmp, e_tmp));
        e_tmp = sub(sub(30,e_tmp),10);
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
    }
    ELSE
    {
        L_tmp1 = L_deposit_l(0);
    }

    L_tmp2 = Mult_32_16(L_log_gm_org,inv_len);/* Q14 */

    L_tmp = L_sub(L_tmp1,L_tmp2);
    move16(); /*Q14 */
    *SFM_org = round_fx(L_shl(L_tmp,14)); /*Q12 */
    *SFM_org = s_max( 0, s_min(*SFM_org, 24547) );
    move16();/*0.0001 and 5.993 in Q12 */

    IF(L_am_gen != 0)
    {
        L_tmp = Mult_32_16(L_am_gen,inv_len);/*Q10 */
        e_tmp = norm_l(L_tmp);
        f_tmp = Log2_norm_lc(L_shl(L_tmp, e_tmp));
        e_tmp = sub(sub(30,e_tmp),10);
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
    }
    ELSE
    {
        L_tmp1 = L_deposit_l(0);
    }

    L_tmp2 = Mult_32_16(L_log_gm_gen,inv_len);/* Q14 */

    L_tmp = L_sub(L_tmp1,L_tmp2);
    move16();/*Q14 */
    *SFM_gen = round_fx(L_shl(L_tmp,14)); /*Q12 */
    *SFM_gen = s_max( 0, s_min(*SFM_gen, 24547) );
    move16();/*0.0001 and 5.993 in Q12 */

    return;
}

/*-------------------------------------------------------------------*
 * energy_control_fx()
 *
 *-------------------------------------------------------------------*/

static void energy_control_fx(
    Encoder_State_fx *st_fx,         /* i/o: Encoder structure   */
    const Word16 core,               /* i  : core                */
    const Word16 mode,               /* i  : SHB BWE class       */
    const Word16 coder_type,         /* i  : coder type          */
    const Word16 *org_fx,               /* i  : input spectrum      */
    const Word16 offset,             /* i  : frequency offset    */
    Word16 *energy_factor_fx,     /* o  : energy factor       */
    Word16 Q_new_lf
)
{
    Word16 n_band,max_band,band_step;
    Word16 gamma_fx, core_type;
    Word16 SWB_signal_fx[L_FRAME32k], SFM_org_fx[SWB_FENV], SFM_gen_fx[SWB_FENV];


    IF ( sub(core,ACELP_CORE) == 0 )
    {
        gamma_fx = 11469;
        move16();/*.35 in Q15 */
        test();
        IF ( sub(coder_type,AUDIO) != 0 && L_sub(st_fx->total_brate_fx,ACELP_8k85)<0 )
        {
            core_type = 0;
            move16();
        }
        ELSE
        {
            core_type = 1;
            move16();
        }
        get_normalize_spec_fx(core, st_fx->extl_fx, mode, core_type, org_fx, SWB_signal_fx, &(st_fx->prev_L_swb_norm1_fx), offset, Q_new_lf);

        IF ( sub(st_fx->extl_fx,WB_BWE) == 0)
        {
            max_band = 4;
            move16();
            band_step = 2;
            move16();
        }
        ELSE
        {
            max_band = SWB_FENV;
            move16();
            band_step = 1;
            move16();
        }
    }
    ELSE  /* HQ core */
    {
        gamma_fx = 18022;
        move16();/*.55 in Q15 */
        get_normalize_spec_fx(core, st_fx->extl_fx, mode, -1, org_fx, SWB_signal_fx, &(st_fx->prev_L_swb_norm1_fx), offset, Q_new_lf);

        band_step = 1;
        move16();
        IF ( sub(offset,HQ_GENERIC_FOFFSET_32K) == 0 )
        {
            max_band = 12;
            move16();
        }
        ELSE
        {
            max_band = SWB_FENV;
            move16();
        }
    }

    FOR( n_band=0; n_band<max_band; )
    {
        calculate_Tonality_fx( org_fx+swb_bwe_subband_fx[n_band]+offset, SWB_signal_fx+swb_bwe_subband_fx[n_band]+offset,
                               &SFM_org_fx[n_band], &SFM_gen_fx[n_band], swb_bwe_subband_fx[n_band+band_step]-swb_bwe_subband_fx[n_band] );

        IF( sub(SFM_gen_fx[n_band],mult_r(24576,SFM_org_fx[n_band])) < 0 )
        {
            energy_factor_fx[n_band] = div_s(SFM_gen_fx[n_band],SFM_org_fx[n_band]);/*Q15 */
            IF( sub(energy_factor_fx[n_band],gamma_fx) < 0 )
            {
                energy_factor_fx[n_band] = gamma_fx;
                move16();
            }
        }
        ELSE
        {
            energy_factor_fx[n_band] = 32767;
            move16(); /* Q15 */
        }
        n_band = add(n_band, band_step);
    }
    return;
}

/*-------------------------------------------------------------------*
* WB_BWE_encoding()
*
* WB BWE main encoder
*-------------------------------------------------------------------*/
Word16 WB_BWE_encoding_fx(     /* o  : classification of wb signal            */
    const Word16 coder_type,       /* i  : coder type                             */
    const Word16 *yos_fx,          /* i  : MDCT coefficients of weighted original */
    Word16 *WB_fenv_fx,      /* i/o: energy of WB envelope                  */
    Encoder_State_fx *st_fx,           /* i/o: Encoder structure                      */
    Word16 Q_synth,
    Word16 Q_synth_lf
)
{
    Word16 mode;
    Word16 i, n_coeff, n_band;
    Word16 index;
    Word32 energy_fx;
    Word32 L_WB_fenv_fx[2];
    Word16 energy_factor_fx[4];
    Word16 ener_40, exp;
    Word32 L_tmp;
    Word16 tmp;

    n_band = 0;
    move16();
    FOR (i = 0; i < 2; i++)
    {
        energy_fx = L_deposit_l(0);
        FOR(n_coeff = swb_bwe_subband_fx[n_band]; n_coeff < swb_bwe_subband_fx[n_band+2]; n_coeff++)
        {
            energy_fx = L_add(energy_fx, L_shr(L_mult0(yos_fx[n_coeff], yos_fx[n_coeff]), 6));   /*2*Q_synth-6 */
        }

        L_WB_fenv_fx[i] = energy_fx;
        move32();           /*2*Q_synth-6 */
        n_band = add(n_band, 2);
    }
    mode = FD_BWE_class_fx(yos_fx, 0, 0, Q_synth, 0, st_fx);


    energy_control_fx( st_fx, ACELP_CORE, mode, coder_type, yos_fx, 0, energy_factor_fx, Q_synth_lf );

    FOR (i = 0; i < 2; i++)
    {
        ener_40 = mult_r(shr(energy_factor_fx[shl(i,1)],1), 26214);/*Q19 */
        L_tmp = Mult_32_16(L_WB_fenv_fx[i], ener_40);/*2*Q_synth-2 */
        IF(L_tmp)
        {
            exp = norm_l(L_tmp);
            tmp = Log2_norm_lc(L_shl(L_tmp, exp));
            /*exp = 30-exp-(2*Q_synth-2); */
            exp = sub(sub(30,exp),(sub(shl(Q_synth,1),2)));
            L_tmp = Mpy_32_16(exp, tmp, 32767); /* Q16 */
            WB_fenv_fx[i] = round_fx(L_shl(L_tmp, 10)); /*Q10 */
        }
        ELSE
        {
            WB_fenv_fx[i] = 0;
            move16();
        }
    }

    index = WB_BWE_fenv_q_fx( WB_fenv_fx, F_2_5_fx, 32, 2 );

    push_indice_fx(st_fx, IND_WB_FENV, index, 5 );

    return (mode);
}

/*-------------------------------------------------------------------*
 * SWB_BWE_encoding()
 *
 * SWB BWE encoder
 *-------------------------------------------------------------------*/
static Word16 SWB_BWE_encoding_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure   */
    Word16 *insig_fx,           /* i  : delayed original input signal at 32kHz */
    const Word16 *insig_lp_fx,        /* i  : delayed original lowband input signal at 32kHz */
    const Word16 *insig_hp_fx,        /* i  : delayed original highband input signal at 32kHz */
    const Word16 *synth_fx,           /* i  : delayed ACELP core synthesis at 12.8kHz */
    const Word16 *yos_fx,             /* i  : MDCT coefficients of the windowed original input signal at 32kHz */
    Word16 *SWB_fenv_fx,        /* o  : frequency-domain quantized BWE envelope */
    const Word16 tilt_nb_fx,          /* i  : SWB tilt */
    const Word16 st_offset,           /* i  : start frequency offset for BWE envelope */
    const Word16 coder_type,          /* i  : coding type                              */
    Word16 Q_insig_lp,
    Word16 Q_shb,
    Word16 Q_synth,
    Word16 Q_synth_lf
)
{
    Word16 IsTransient, mode;
    Word16 index;
    Word16 i, n_coeff, n_band, pos, indice[6];
    Word16 L;
    Word16 IsTransient_LF;

    Word16 tmp;
    Word32 energy_fx;
    Word16 tilt_fx;
    Word32 global_gain_fx;
    Word32 L_tmp;
    Word32 L_SWB_fenv_fx[SWB_FENV];
    Word16 SWB_tenv_fx[SWB_TENV];
    Word32 L_SWB_tenv, WB_tenv_syn_fx, WB_tenv_orig_fx;
    Word16 exp, expn, expd;
    Word16 num, den;
    Word16 scale;
    Word16 Rat_tenv_fx;
    Word16 SWB_tenv_tmp_fx[SWB_TENV];
    Word16 max_fx;
    Word16 energy_factor_fx[SWB_FENV], w_env_fx[SWB_FENV];

    IF( sub(st_fx->L_frame_fx, L_FRAME ) == 0)
    {
        L = L_SUBFR;
        move16();
    }
    ELSE
    {
        L = L_SUBFR16k;
        move16();
    }

    /* HF transient detect */
    IsTransient = detect_transient_fx( insig_hp_fx, L_FRAME16k, coder_type, Q_shb, st_fx);
    st_fx->prev_Q_new = shl(Q_shb, 1);

    /* LF transient detect */
    IsTransient_LF = 0;
    move16();
    FOR ( n_band = 0; n_band < 4; n_band++ )
    {
        tmp = i_mult2(n_band, L);
        energy_fx = L_deposit_l(0);
        FOR(i=0; i<L; i++)
        {
            energy_fx = L_add(energy_fx, L_shr(L_mult0(insig_lp_fx[i + tmp], insig_lp_fx[i + tmp]), 7));   /*2*Q_slb_speech - 7 */
        }

        IF(L_sub(Mult_32_16(energy_fx, 5958), st_fx->EnergyLF_fx) > 0)
        {
            IsTransient_LF = 1;
            move16();
        }

        st_fx->EnergyLF_fx = energy_fx;
        move32();
    }

    /* tilt returned in Q24 go to Q11 */
    tilt_fx = round_fx(L_shl(calc_tilt_bwe_fx(insig_fx, 0, L_FRAME32k), 3));

    test();
    test();
    IF( sub(IsTransient,1) == 0 && (sub(tilt_fx, 16384) > 0 || sub(st_fx->clas_fx,1) > 0) )
    {
        IsTransient = 0;
        move16();
        st_fx->TransientHangOver_fx = 0;
        move16();
    }

    IF( sub(IsTransient,1) == 0 )
    {
        mode = IsTransient;
        move16();
        push_indice_fx(st_fx, IND_SWB_CLASS, mode, 2 );

        /* Energy for the different bands and global energies */
        global_gain_fx = L_deposit_l(0);
        FOR (n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
        {
            energy_fx = L_deposit_l(0);
            FOR (n_coeff = swb_bwe_trans_subband_fx[n_band]+st_offset; n_coeff < swb_bwe_trans_subband_fx[n_band+1]+st_offset; n_coeff++)
            {
                L_tmp = L_shr(L_mult0(yos_fx[n_coeff], yos_fx[n_coeff]), 7); /*2*Q_synth-7 */
                energy_fx = L_add(L_tmp, energy_fx); /*2*Q_synth-7 */
            }
            global_gain_fx = L_add(global_gain_fx, L_shr(energy_fx, sub(sub(shl(Q_synth,1),7), shl(Q_shb,1))));  /*2*Q_shb */
            L_SWB_fenv_fx[n_band] = energy_fx;
            move32();
        }
        global_gain_fx = L_shr(global_gain_fx, 1); /*2*Q_shb  */

        FOR (n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
        {
            expd = norm_s(swb_bwe_trans_subband_width_fx[n_band]);
            tmp = div_s(shl(1,sub(14,expd)), swb_bwe_trans_subband_width_fx[n_band]);/*Q(29-expd) */
            L_tmp = Mult_32_16(L_SWB_fenv_fx[n_band], tmp);   /*2*Q_synth-7+29-expd - 15                */
            exp = norm_l(L_tmp);
            tmp = Log2_norm_lc(L_shl(L_tmp, exp));
            move16();
            exp = sub(sub(30, exp), sub(add(shl(Q_synth,1),7),expd));
            L_tmp = Mpy_32_16(exp, tmp, 24660);  /* Q14 */ /*10log10(2) in Q13 */
            tmp = round_fx(L_shl(L_tmp, 10));  /* Q8 */

            SWB_fenv_fx[n_band] = sub(tmp, Mean_env_tr_fx[n_band]);
            move16();/*Q8 */
        }

        WB_tenv_orig_fx = L_deposit_l(0);
        WB_tenv_syn_fx = L_deposit_l(1);
        FOR(n_band = 0; n_band < SWB_TENV; n_band++)
        {
            tmp = i_mult2(n_band, L_SUBFR16k);
            L_SWB_tenv = L_deposit_l(0);
            FOR(i = 0; i < L_SUBFR16k; i++)
            {
                L_SWB_tenv = L_add(L_SWB_tenv, L_mult0(insig_hp_fx[i + tmp], insig_hp_fx[i + tmp]));   /*2*Q_shb */
            }

            tmp = i_mult2(n_band, L);
            FOR(i=0; i<L; i++)
            {
                WB_tenv_syn_fx = L_add(WB_tenv_syn_fx, L_shr(L_mult0(synth_fx[i + tmp], synth_fx[i + tmp]), 7));   /*2*st_fx->Q_syn2 - 7 */
                WB_tenv_orig_fx = L_add(WB_tenv_orig_fx, L_shr(L_mult0(insig_lp_fx[i + tmp], insig_lp_fx[i + tmp]), 7));   /*2*Q_insig_lp - 7 */
            }

            L_tmp = Mult_32_16(L_SWB_tenv, INV_L_SUBFR16k_FX);/*2*Q_shb */
            SWB_tenv_fx[n_band] = 0;
            move16();
            IF(L_tmp != 0)
            {
                exp = norm_l(L_tmp);
                tmp = extract_h(L_shl(L_tmp, exp));
                exp = sub(exp, sub(30, 2*Q_shb));

                tmp = div_s(16384, tmp);
                L_tmp = L_deposit_h(tmp);
                L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */

                SWB_tenv_fx[n_band] = round_fx(L_shl(L_tmp, sub(exp, 12))); /*Q3           */
            }
        }

        IF(WB_tenv_orig_fx != 0)
        {
            expn = norm_l(WB_tenv_orig_fx);
            num = extract_h(L_shl(WB_tenv_orig_fx, expn));
            expn = sub(sub(30, expn), sub(shl(Q_insig_lp,1),7));

            expd = norm_l(WB_tenv_syn_fx);
            den = round_fx(L_shl(WB_tenv_syn_fx, expd));
            expd = sub(sub(30, expd), sub(shl(st_fx->Q_syn2, 1), 7));

            scale = shr(sub(den, num), 15);
            num   = shl(num, scale);
            expn  = sub(expn, scale);

            tmp  = div_s(num, den);
            expn = sub(expn, expd);

            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &expn); /*31-expn */

            Rat_tenv_fx = round_fx(L_shl(L_tmp, sub(expn, 1)));/*Q14 */
        }
        ELSE
        {
            Rat_tenv_fx = 16384;
            move16();
        }

        IF(sub(Rat_tenv_fx, 8192) < 0)
        {
            L_tmp = L_mult(Rat_tenv_fx, 19661);/*Q29 */
            Rat_tenv_fx = round_fx(L_shl(L_tmp, 2));/*Q15 */
        }
        ELSE IF (sub(Rat_tenv_fx, 16384)> 0)
        {
            Rat_tenv_fx = 32767;
            move16();
        }

        FOR(n_band = 0; n_band < SWB_TENV; n_band++)
        {
            SWB_tenv_fx[n_band] = mult_r(SWB_tenv_fx[n_band], Rat_tenv_fx);
            move16();/*Q3 */
        }

        max_fx = SWB_tenv_fx[0];
        move16();
        pos = 0;
        move16();
        FOR(n_band = 1; n_band < SWB_TENV; n_band++)
        {
            IF(sub(SWB_tenv_fx[n_band],max_fx) > 0)
            {
                max_fx = SWB_tenv_fx[n_band];
                move16();
                pos = n_band;
                move16();
            }
        }

        max_fx = SWB_tenv_fx[0];
        move16();
        FOR(n_band = 1; n_band < SWB_TENV; n_band++)
        {
            tmp = sub(mult_r(SWB_tenv_fx[n_band], 6554), SWB_tenv_fx[n_band-1]);
            IF(tmp > 0)
            {
                BREAK;
            }
        }

        IF(n_band < SWB_TENV)
        {
            energy_fx = L_deposit_l(0);
            FOR(n_band = (pos+1); n_band < SWB_TENV; n_band++)
            {
                energy_fx = L_add(energy_fx, SWB_tenv_fx[n_band]);/*Q3 */
            }

            IF(pos == sub(SWB_TENV, 1))
            {
                energy_fx = L_deposit_l(0);
            }
            ELSE
            {
                tmp = sub(SWB_TENV, pos+1);
                tmp = div_s(1, tmp); /*Q15 */
                energy_fx = Mult_32_16(energy_fx, tmp);/*Q3 */
            }

            FOR(n_band = 0; n_band < pos; n_band++)
            {
                SWB_tenv_fx[n_band] = mult_r(SWB_tenv_fx[n_band], 16384);
                move16();
            }

            /*SWB_tenv_fx[pos] = add(SWB_tenv_fx[pos], mult_r(SWB_tenv_fx[pos], 164));    move16();//Q3 */
            SWB_tenv_fx[pos] = round_fx(L_mac(L_mult(SWB_tenv_fx[pos],32767), SWB_tenv_fx[pos], 164)); /*Q3 */

            IF(L_sub(energy_fx, SWB_tenv_fx[pos]) < 0)
            {
                FOR(n_band = pos+1; n_band < SWB_TENV; n_band++)
                {
                    SWB_tenv_fx[n_band] = mult_r(SWB_tenv_fx[n_band], 29491);
                    move16();/*Q3 */
                }
            }
        }
        ELSE
        {
            FOR(n_band = 1; n_band < SWB_TENV; n_band++)
            {
                IF(sub(SWB_tenv_fx[n_band-1], SWB_tenv_fx[n_band]) > 0)
                {
                    /*SWB_tenv_fx[n_band-1] = add(mult_r(SWB_tenv_fx[n_band-1], 16384), mult_r(SWB_tenv_fx[n_band], 16384)); move16();//Q3 */
                    SWB_tenv_fx[n_band-1] = round_fx(L_mac(L_mult(SWB_tenv_fx[n_band-1], 16384), SWB_tenv_fx[n_band], 16384)); /*Q3 */
                }
                ELSE
                {
                    /*SWB_tenv_fx[n_band] = add(mult_r(SWB_tenv_fx[n_band-1], 16384), mult_r(SWB_tenv_fx[n_band], 16384)); move16();//Q3 */
                    SWB_tenv_fx[n_band] = round_fx(L_mac(L_mult(SWB_tenv_fx[n_band-1], 16384), SWB_tenv_fx[n_band], 16384)); /*Q3 */
                }
            }

            FOR(n_band = 0; n_band < SWB_TENV; n_band++)
            {
                SWB_tenv_fx[n_band] = mult_r(SWB_tenv_fx[n_band], 29491);
                move16();/*Q3 */
            }
        }

        test();
        test();
        IF(IsTransient_LF == 0 && sub(coder_type,INACTIVE) == 0 && sub(st_fx->TransientHangOver_fx,1) == 0)
        {
            FOR(n_band = 0; n_band < SWB_TENV; n_band++)
            {
                SWB_tenv_fx[n_band] = mult_r(SWB_tenv_fx[n_band], 16384);
                move16();
            }
            FOR(n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
            {
                SWB_fenv_fx[n_band] = mult_r(SWB_fenv_fx[n_band], 1638);
                move16();
            }
        }
        ELSE
        {
            SWB_fenv_fx[2] = mult_r(SWB_fenv_fx[2], 3277);
            move16();
            SWB_fenv_fx[3] = mult_r(SWB_fenv_fx[3], 1638);
            move16();
        }

        FOR(n_band = 0; n_band < SWB_TENV; n_band++)
        {
            IF(SWB_tenv_fx[n_band] == 0)
            {
                SWB_tenv_tmp_fx[n_band] = -32768;
                move16(); /*-16 in Q11 */
            }
            ELSE
            {
                L_tmp = L_deposit_h(SWB_tenv_fx[n_band]); /*Q19 */
                expn = norm_l(L_tmp);
                tmp = Log2_norm_lc(L_shl(L_tmp, expn));
                expn = sub(sub(30, expn), 19);
                L_tmp = Mpy_32_16(expn, tmp, 32767); /* Q16 */ /*1 in Q15 */
                SWB_tenv_tmp_fx[n_band] = round_fx(L_shl(L_tmp, 11)); /* Q11 */
            }

            IF (sub(SWB_tenv_tmp_fx[n_band], 30720) > 0)
            {
                index = 15;
                move16();
            }
            ELSE IF (SWB_tenv_tmp_fx[n_band] < 0)
            {
                index = 0;
                move16();
            }
            ELSE
            {
                index = shr(add(SWB_tenv_tmp_fx[n_band], 1024), 11);
            }

            push_indice_fx(st_fx, IND_SWB_TENV, index, 4 );
        }

        MSVQ_Interpol_Tran_fx(SWB_fenv_fx, indice);

        push_indice_fx(st_fx, IND_SWB_FENV, indice[0], 7 );
        push_indice_fx(st_fx, IND_SWB_FENV, indice[1], 6 );
    }
    ELSE
    {
        /* Energy for the different bands and global energies */
        global_gain_fx = L_deposit_l(0);
        FOR (n_band = 0; n_band < SWB_FENV; n_band++)
        {
            energy_fx = L_deposit_l(0);
            FOR (n_coeff = swb_bwe_subband_fx[n_band]+st_offset; n_coeff < swb_bwe_subband_fx[n_band+1]+st_offset; n_coeff++)
            {
                L_tmp = L_shr(L_mult0(yos_fx[n_coeff], yos_fx[n_coeff]), 5); /*2*Q_synth-5 */
                energy_fx = L_add(L_tmp, energy_fx); /*2*Q_synth-5 */
            }

            IF (sub(n_band, sub(SWB_FENV,2)) < 0)
            {
                global_gain_fx = L_add(global_gain_fx, L_shr(energy_fx, sub(2*Q_synth-5, 2*Q_shb)));  /*2*Q_shb */
            }
            L_SWB_fenv_fx[n_band] = energy_fx;
            move32();
        }

        global_gain_fx = L_shr(global_gain_fx, 1); /*2*Q_shb */
        mode = FD_BWE_class_fx(yos_fx, global_gain_fx, tilt_nb_fx, Q_synth, Q_shb, st_fx);
        push_indice_fx(st_fx, IND_SWB_CLASS, mode, 2 );

        energy_control_fx( st_fx, ACELP_CORE, mode, -1, yos_fx, st_offset, energy_factor_fx, Q_synth_lf );

        FOR (n_band = 0; n_band < SWB_FENV; n_band++)
        {
            L_tmp = Mult_32_16(L_SWB_fenv_fx[n_band],energy_factor_fx[n_band]);/*2*Q_synth-5 */
            L_tmp = Mult_32_16(L_tmp,swb_inv_bwe_subband_width_fx[n_band]);/*2*Q_synth-5 */

            IF(L_tmp != 0)
            {
                expn = norm_l(L_tmp);
                tmp = Log2_norm_lc(L_shl(L_tmp,expn));
                expn = sub(30,add(expn,sub(shl(Q_synth,1),5)));
                L_tmp = Mpy_32_16(expn, tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
                SWB_fenv_fx[n_band] = round_fx(L_shl(L_tmp, 10)); /* Q8 */
            }
            ELSE
            {
                SWB_fenv_fx[n_band] = 0;
                move16();
            }
        }

        IF( L_sub(st_fx->EnergyLT_fx, Mult_32_16(global_gain_fx, 546)) < 0 )
        {
            FOR (n_band = 0; n_band < SWB_FENV; n_band++)
            {
                SWB_fenv_fx[n_band] = mult_r(SWB_fenv_fx[n_band], 3277);
                move16();  /*Q8  */
            }
        }

        freq_weights_fx(SWB_fenv_fx, w_NOR_fx, w_env_fx, SWB_FENV);

        FOR (n_band = 0; n_band < SWB_FENV; n_band++)
        {
            SWB_fenv_fx[n_band] = sub(SWB_fenv_fx[n_band] , Mean_env_fx[n_band]);
            move16();
        }

        /* Energy VQ */
        msvq_interpol_fx(SWB_fenv_fx, w_env_fx, indice);

        push_indice_fx(st_fx, IND_SWB_FENV, indice[0], 5 );
        push_indice_fx(st_fx, IND_SWB_FENV, indice[1], 7 );
        push_indice_fx(st_fx, IND_SWB_FENV, indice[2], 6 );
        push_indice_fx(st_fx, IND_SWB_FENV, indice[3], 5 );
        push_indice_fx(st_fx, IND_SWB_FENV, indice[4], 6 );

    }
    st_fx->prev_mode_fx = mode;
    move16();
    st_fx->prev_global_gain_fx = global_gain_fx;
    move32();
    st_fx->prev_Q_shb = Q_shb;
    move16();

    return mode;
}

/*-------------------------------------------------------------------*
 * get_normalize_spec_fx_32()
 *
 *-------------------------------------------------------------------*/

static void get_normalize_spec_fx_32(
    const Word16 core,                   /* i  : core selected           : Q0  */
    const Word16 extl,                   /* i  : extension layer selected: Q0  */
    const Word16 mode,                   /* i  : SHB BWE class           : Q0  */
    const Word16 core_type,              /* i  : coding type             : Q0  */
    const Word32 *org_fx,         /* i  : input spectrum          : Q12 */
    Word32 *SWB_signal_fx,             /* o  : output spectrum         : Q20 */
    Word16 *prev_L_swb_norm,             /* i  : previous norm. len      : Q0  */
    const Word16 offset                  /* i  : frequency offset        : Q0  */
)
{
    Word16 n_freq, L_swb_norm;
    Word16 frq_end;
    Word16 exp1, exp2, tmp;
    Word32 L_tmp;
    Word32 envelope_fx[L_FRAME32k];

    set32_fx(SWB_signal_fx, 0, HQ_GENERIC_HIGH0+offset);
    calc_normal_length_fx_32(core, org_fx, mode, extl, &L_swb_norm, prev_L_swb_norm);
    test();
    IF(sub(extl , SWB_BWE) == 0 || sub( extl , FB_BWE)  == 0 )
    {
        IF ( sub(mode ,HARMONIC) == 0 )
        {
            Copy32(org_fx, &SWB_signal_fx[add(240,offset)], 240);
            Copy32(&org_fx[128], &SWB_signal_fx[add(480,offset)], 80);
        }
        ELSE
        {
            Copy32( &org_fx[112], &SWB_signal_fx[add(240,offset)], 128 );
            Copy32( &org_fx[112], &SWB_signal_fx[add(368,offset)], 128 );
            Copy32( &org_fx[176], &SWB_signal_fx[add(496,offset)], 64 );
        }
        frq_end = 560+offset;
        move16();
    }
    ELSE IF (sub(extl , WB_BWE)==0)
    {
        IF ( core_type == 0 )
        {
            Copy32(&org_fx[160], &SWB_signal_fx[240], 80);
        }
        ELSE
        {
            Copy32(&org_fx[80], &SWB_signal_fx[240], 80);
        }
        frq_end = L_FRAME16k;
        move16();
    }
    ELSE
    {
        Copy32( org_fx+HQ_GENERIC_OFFSET, SWB_signal_fx+HQ_GENERIC_HIGH0+offset, HQ_GENERIC_LEN0 );
        Copy32( org_fx+HQ_GENERIC_OFFSET, SWB_signal_fx+HQ_GENERIC_HIGH1+offset, HQ_GENERIC_LEN0 );
        IF ( sub(offset , HQ_GENERIC_FOFFSET_24K4) == 0 )
        {
            Copy32( org_fx+HQ_GENERIC_LOW0, SWB_signal_fx+HQ_GENERIC_HIGH2+offset, HQ_GENERIC_END_FREQ-HQ_GENERIC_HIGH2 );
        }
        frq_end = L_FRAME32k;
        move16();
    }

    /* calculate envelope */
    calc_norm_envelop_fx_32(SWB_signal_fx, envelope_fx, L_swb_norm, frq_end - offset, offset);

    /* Normalize with envelope */
    FOR ( n_freq = add(swb_bwe_subband_fx[0],offset); n_freq<frq_end; n_freq++ )
    {
        IF (envelope_fx[n_freq] != 0)
        {
            exp1 = norm_l(envelope_fx[n_freq]);
            exp2 = norm_l(SWB_signal_fx[n_freq]);
            tmp = extract_h(L_shl(envelope_fx[n_freq], exp1));/*12 + exp1 - 16 */
            tmp = div_s(16384, tmp);/*15 + 14 - (12 + exp1 - 16) */
            L_tmp = Mult_32_16(L_shl(SWB_signal_fx[n_freq], exp2), tmp);/*exp2 + 12 + (15 + 14 - (12 + exp1 - 16)) - 15 */
            SWB_signal_fx[n_freq] = L_shr(L_tmp, sub(10, sub(exp1, exp2)));/*20 */
        }
        ELSE
        {
            SWB_signal_fx[n_freq] = 0;
            move16();/*20 */
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * calculate_tonality_fx_32()
 *
 *-------------------------------------------------------------------*/
static void calculate_tonality_fx_32(
    const Word32 *org_fx,               /* i  : MDCT coefficients of original         : Q12  */
    const Word32 *gen_fx,               /* i  : MDCT coefficients of generated signal : Q12  */
    Word32 *SFM_org,              /* o  : Spectral Flatness results             : Q14  */
    Word32 *SFM_gen,              /* o  : Spectral Flatness results             : Q14  */
    const Word16 length                 /* i  : length for calculating tonality       : Q0   */
)
{
    Word16 n_coeff;
    Word32 am_org_fx, am_gen_fx, log_gm_org_sum_fx, log_gm_gen_sum_fx;
    Word16 exp, exp1, exp2, tmp;
    Word32 L_tmp, L_tmp1, L_tmp2;
    Word16 inv_len_fx;
    Word32 max_fx;
    Word32 org_spec_fx[80], gen_spec_fx[80];

    /* to reduce dynamic range of original spectrum */
    max_fx = 0;
    move16();
    FOR ( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        org_spec_fx[n_coeff] = L_abs(org_fx[n_coeff]);

        IF (L_sub(max_fx , org_spec_fx[n_coeff])<0)
        {
            max_fx = org_spec_fx[n_coeff];
            move16();
        }
    }
    max_fx = 0;
    move16();
    FOR ( n_coeff=0; n_coeff<length; n_coeff++ )
    {
        gen_spec_fx[n_coeff] = L_abs(gen_fx[n_coeff]);
        IF (L_sub(max_fx , gen_spec_fx[n_coeff])<0)
        {
            max_fx = gen_spec_fx[n_coeff];
            move16();
        }
    }

    exp = norm_s(length);
    inv_len_fx = div_s(shl(1, exp), shl(length, exp));/*15 */

    am_org_fx = 0;
    move16();
    am_gen_fx = 0;
    move16();
    log_gm_org_sum_fx = 0;
    move16();
    log_gm_gen_sum_fx = 0;
    move16();

    FOR ( n_coeff = 0; n_coeff<length; n_coeff++ )
    {
        am_org_fx = L_add(am_org_fx, org_spec_fx[n_coeff]);
        am_gen_fx = L_add(am_gen_fx, gen_spec_fx[n_coeff]);

        IF (org_spec_fx[n_coeff] != 0)
        {
            exp = norm_l(org_spec_fx[n_coeff]);
            tmp = Log2_norm_lc(L_shl(org_spec_fx[n_coeff], exp));/*15 */
            exp = sub(30, add(exp, 12));
            L_tmp = L_add(L_shl(exp, 16), L_shr(L_deposit_h(tmp), 15));/*16 */
            log_gm_org_sum_fx = L_add(log_gm_org_sum_fx, L_tmp);/*Q16 */
        }
        IF (gen_spec_fx[n_coeff] != 0)
        {
            exp = norm_l(gen_spec_fx[n_coeff]);
            tmp = Log2_norm_lc(L_shl(gen_spec_fx[n_coeff], exp));
            exp = sub(30, add(exp, 12));
            L_tmp = L_add(L_shl(exp, 16), L_shr(L_deposit_h(tmp), 15));/*16 */
            log_gm_gen_sum_fx = L_add(log_gm_gen_sum_fx, L_tmp);/*16 */
        }
    }

    IF (am_org_fx != 0)
    {
        exp1 = norm_l(am_org_fx);
        L_tmp1 = Mult_32_16(L_shl(am_org_fx, exp1), inv_len_fx);/*12 + exp1 + 15 - 15 */
        exp2 = norm_l(L_tmp1);/*12 + exp1 + exp2 */
        tmp = Log2_norm_lc(L_shl(L_tmp1, exp2));
        exp1 = sub(30, add(add(exp1, exp2), 12));
        L_tmp1 = Mpy_32_16(exp1, tmp, 24660);/*15 + 1 + 13 - 15  */
    }
    ELSE
    {
        L_tmp1 = 0;
        move16();
    }

    exp = norm_l(log_gm_org_sum_fx);
    L_tmp2 = Mult_32_16(L_shl(log_gm_org_sum_fx, exp), 24660);/*16 + exp + 13 - 15 */
    L_tmp2 = Mult_32_16(L_tmp2, inv_len_fx);/*14 + exp + 15 - 15 */
    L_tmp2 = L_shr(L_tmp2, exp);/*14 */
    L_tmp = L_sub(L_tmp1, L_tmp2);/*14 */

    *SFM_org = L_max(0, L_min(L_tmp, 98189));

    IF (am_gen_fx != 0)
    {
        exp1 = norm_l(am_gen_fx);
        L_tmp1 = Mult_32_16(L_shl(am_gen_fx, exp1), inv_len_fx);/*12 + exp1 + 15 - 15 */
        exp2 = norm_l(L_tmp1);/*12 + exp1 + exp2 */
        tmp = Log2_norm_lc(L_shl(L_tmp1, exp2));
        exp1 = sub(30, add(add(exp1, exp2), 12));
        L_tmp1 = Mpy_32_16(exp1, tmp, 24660);/*15 + 1 + 13 - 15  */
    }
    ELSE
    {
        L_tmp1 = 0;
        move16();
    }

    exp = norm_l(log_gm_gen_sum_fx);
    L_tmp2 = Mult_32_16(L_shl(log_gm_gen_sum_fx, exp), 24660);/*16 + exp + 13 - 15 */
    L_tmp2 = Mult_32_16(L_tmp2, inv_len_fx);/*14 + 15 - 15 */
    L_tmp2 = L_shr(L_tmp2, exp);/*14 */
    L_tmp = L_sub(L_tmp1, L_tmp2);/*14 */

    *SFM_gen = L_max(0, L_min(L_tmp, 98189));

    return;
}

/*-------------------------------------------------------------------*
 * energy_control_fx_32()
 *
 *-------------------------------------------------------------------*/
static void energy_control_fx_32(
    Encoder_State_fx *st_fx,                 /* i/o: encoder structure   */
    const Word16 core,               /* i  : core                : Q0  */
    const Word16 mode,               /* i  : SHB BWE class       : Q0  */
    const Word16 coder_type,         /* i  : SHB BWE class       : Q0  */
    const Word32 *org_fx,       /* i  : input spectrum      : Q12 */
    const Word16 offset,             /* i  : frequency offset    : Q0  */
    Word16 *energy_factor_fx   /* o  : energy factor       : Q15 */
)
{
    Word16 n_band;
    Word16 core_type;
    Word16 max_band=SWB_FENV,band_step=1;
    Word32 SWB_signal_fx[L_FRAME32k];
    Word32 SFM_org_fx[SWB_FENV], SFM_gen_fx[SWB_FENV];
    Word32 L_temp1, L_temp2;
    Word16 exp1, exp2, tmp1, tmp2, tmp;
    Word16 gamma_fx;

    IF ( core == ACELP_CORE )
    {
        gamma_fx = 11468;
        move16();
        test();
        IF ( sub(coder_type , AUDIO) != 0 && L_sub(st_fx->total_brate_fx , ACELP_8k00)<=0 )
        {
            core_type = 0;
            move16();
        }
        ELSE
        {
            core_type = 1;
            move16();
        }

        get_normalize_spec_fx_32(core, st_fx->extl_fx, mode, core_type, org_fx, SWB_signal_fx, &(st_fx->prev_L_swb_norm1_fx), offset );

        IF ( sub(st_fx->extl_fx , WB_BWE) == 0)
        {
            max_band = 4;
            move16();
            band_step = 2;
            move16();
        }
    }
    ELSE  /* HQ core */
    {
        gamma_fx = 18021;
        move16();
        get_normalize_spec_fx_32(core, -1, mode, -1, org_fx, SWB_signal_fx, &(st_fx->prev_L_swb_norm1_fx), offset );

        IF ( sub(offset , HQ_GENERIC_FOFFSET_32K) == 0 )
        {
            max_band = 12;
            move16();
        }
    }

    FOR ( n_band=0; n_band<max_band; n_band+=band_step )
    {
        calculate_tonality_fx_32(&org_fx[add(swb_bwe_subband_fx[n_band],offset)], &SWB_signal_fx[add(swb_bwe_subband_fx[n_band],offset)],
                                 &SFM_org_fx[n_band], &SFM_gen_fx[n_band], swb_bwe_subband_fx[add(n_band,band_step)]-swb_bwe_subband_fx[n_band]);

        L_temp1 = L_shl(SFM_gen_fx[n_band], 2);
        L_temp2 = L_add(SFM_org_fx[n_band], L_shl(SFM_org_fx[n_band], 1));
        IF (L_temp1 < L_temp2)
        {
            exp1 = sub(norm_l(SFM_gen_fx[n_band]), 1);
            exp2 = norm_l(SFM_org_fx[n_band]);
            tmp1 = extract_h(L_shl(SFM_gen_fx[n_band], exp1));
            tmp2 = extract_h(L_shl(SFM_org_fx[n_band], exp2));
            tmp = div_s(tmp1, tmp2);/*15 + (14 + exp1 ) - (14 + exp2) */
            energy_factor_fx[n_band] = shl(tmp, sub(exp2, exp1));/*15 */

            IF (sub(energy_factor_fx[n_band] , gamma_fx)<0)
            {
                energy_factor_fx[n_band] = gamma_fx;
                move16();
            }
        }
        ELSE
        {
            energy_factor_fx[n_band] = 32767;
            move16();/*15 */
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * decision_hq_generic_class_fx_32()
 *
 *-------------------------------------------------------------------*/
static Word16 decision_hq_generic_class_fx_32 (
    const Word32 *coefs_fx,           /* i: original MDCT spectrum                      : Q12   */
    const Word16 hq_generic_offset    /* i: frequency offset of high frequency spectrum : Q0    */
)
{
    Word16 i, k;
    Word16 nband;

    Word16 inv_band_fx;
    Word32 L_tmp, L_tmp1, L_tmp2;
    Word16 exp, tmp, tmp2;
    Word32 p_fx, a_fx;
    Word32 p2a_fx;
    Word32 avgp2a_fx;

    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4) == 0 )
    {
        nband = 10;
        move16();
        inv_band_fx = 3277;
        move16();/*15 */
    }
    ELSE
    {
        nband = 8;
        move16();
        inv_band_fx = 4096;
        move16();/*15 */
    }

    avgp2a_fx = L_deposit_l(0);
    FOR (k = 0; k < nband; k++)
    {
        a_fx = L_deposit_l(0);
        p_fx = L_deposit_l(0);
        tmp2 = add(swb_bwe_subband_fx[add(k,1)],hq_generic_offset);
        FOR (i = add(swb_bwe_subband_fx[k],hq_generic_offset); i < tmp2; i++)
        {
            exp = norm_l(coefs_fx[i]);
            tmp = extract_h(L_shl(coefs_fx[i], exp));/*12 + exp - 16 */
            L_tmp = L_mult0(tmp, tmp);/*2 * exp - 8 */
            L_tmp = L_shl(L_tmp, sub(14, shl(exp, 1)));/*6 */
            IF (L_sub(L_tmp , p_fx) > 0)
            {
                p_fx = L_add(L_tmp, 0);/*6 */
            }
            a_fx = L_add(a_fx, L_tmp);/*6 */
        }

        IF (a_fx > 0)
        {
            a_fx = Mult_32_16(a_fx, swb_inv_bwe_subband_width_fx[k]);/*6 */

            exp = norm_l(p_fx);
            tmp = Log2_norm_lc(L_shl(p_fx, exp));/*15 */
            exp = sub(30, add(exp, 6));
            L_tmp1 = L_add(L_deposit_h(exp), L_shr(L_deposit_h(tmp), 15));/*16 */

            exp = norm_l(a_fx);
            tmp = Log2_norm_lc(L_shl(a_fx, exp));
            exp = sub(30, add(exp, 6));
            L_tmp2 = L_add(L_deposit_h(exp), L_shr(L_deposit_h(tmp), 15));/*16 */

            p2a_fx = L_sub(L_tmp1, L_tmp2);/*16 */
            avgp2a_fx = L_add(avgp2a_fx, p2a_fx);/*16 */
        }
    }
    avgp2a_fx = Mult_32_16(avgp2a_fx, inv_band_fx);/*16 + 15 - 15 */
    IF (L_sub(avgp2a_fx , 187227)>0)/*8.6 / 10log10(2), Q16 */
    {
        return HQ_GENERIC_EXC1;
    }
    ELSE
    {
        return HQ_GENERIC_EXC0;
    }
}

/*-------------------------------------------------------------------*
 * hq_generic_encoding_fx()
 *
 *-------------------------------------------------------------------*/
void hq_generic_encoding_fx(
    const Word32 *coefs_fx,                     /* i  : MDCT coefficients of weighted original : Q12   */
    Word16 *hq_generic_fenv_fx,           /* i/o: energy of SWB envelope                 : Q3    */
    const Word16 hq_generic_offset,             /* i  : frequency offset for extracting energy : Q0    */
    Encoder_State_fx *st_fx,                      /* i/o: encoder state structure                        */
    Word16 *hq_generic_exc_clas           /* o  : bwe excitation class                   : Q0    */
)
{
    Word16 n_coeff, n_band;
    Word16 indice[HQ_GENERIC_NVQIDX];
    Word16 nenv;

    Word16 energy_factor_fx[SWB_FENV] = {0};
    Word16 cs, exp, tmp, tmp2;
    Word32 energy_fx;
    Word32 L_tmp, max_coefs_fx;
    Word16 w_env_fx[SWB_FENV];

    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4)<=0 )
    {
        nenv = SWB_FENV;
        move16();
    }
    ELSE
    {
        nenv = SWB_FENV-2;
        move16();
    }


    energy_control_fx_32(st_fx, HQ_CORE, -1, -1, coefs_fx, hq_generic_offset, energy_factor_fx);

    IF ( sub(st_fx->hq_generic_speech_class_fx , 1) == 0 )
    {
        push_indice_fx( st_fx, IND_HQ_SWB_EXC_SP_CLAS, 1, 1 );
        *hq_generic_exc_clas = HQ_GENERIC_SP_EXC;
        move16();
    }
    ELSE
    {
        *hq_generic_exc_clas = decision_hq_generic_class_fx_32(coefs_fx, hq_generic_offset);
        push_indice_fx( st_fx, IND_HQ_SWB_EXC_SP_CLAS, 0, 1 );
        push_indice_fx( st_fx, IND_HQ_SWB_EXC_CLAS, *hq_generic_exc_clas, 1 );
    }

    FOR ( n_band = 0; n_band < nenv; n_band++ )
    {
        energy_fx = L_deposit_l(0);
        max_coefs_fx = L_deposit_l(0);
        tmp2 = add(swb_bwe_subband_fx[n_band+1] , hq_generic_offset);
        FOR ( n_coeff = add(swb_bwe_subband_fx[n_band],hq_generic_offset); n_coeff < tmp2; n_coeff++ )
        {
            IF (L_sub(max_coefs_fx , L_abs(coefs_fx[n_coeff])) < 0)
            {
                max_coefs_fx = L_abs(coefs_fx[n_coeff]);
            }
        }
        cs = norm_l(max_coefs_fx);
        tmp2 = add(swb_bwe_subband_fx[n_band+1] , hq_generic_offset);
        FOR ( n_coeff = add(swb_bwe_subband_fx[n_band],hq_generic_offset); n_coeff < tmp2; n_coeff++ )
        {
            tmp = extract_h(L_shl(coefs_fx[n_coeff], cs));/*12 + cs - 16 */
            L_tmp = L_mult0(tmp, tmp);/*2*cs - 8 */
            energy_fx = L_add(energy_fx, L_shr(L_tmp, 5));
        }

        IF (energy_fx != 0)
        {
            L_tmp = Mult_32_16(energy_fx, energy_factor_fx[n_band]);/*2*cs - 13 */
            L_tmp = Mult_32_16(L_tmp, swb_inv_bwe_subband_width_fx[n_band]);/*2*cs - 13 + 15 - 15 */

            exp = norm_l(L_tmp);
            tmp = Log2_norm_lc(L_shl(L_tmp, exp));
            exp = sub(30, add(exp, 2*cs-13));

            L_tmp = Mpy_32_16(exp, tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
            hq_generic_fenv_fx[n_band] = round_fx(L_shl(L_tmp, 10));/*Q8 */
        }
        ELSE
        {
            hq_generic_fenv_fx[n_band] = 0;
            move16();
        }
    }

    IF ( sub(st_fx->bwidth_fx , FB) == 0 )
    {
        FOR ( n_band = 0; n_band < DIM_FB; n_band++ )
        {
            energy_fx = L_deposit_l(0);
            max_coefs_fx = L_deposit_l(0);
            tmp2 = fb_bwe_subband[add(n_band,1)];
            FOR ( n_coeff = fb_bwe_subband[n_band]; n_coeff < tmp2; n_coeff++ )
            {
                IF (L_sub(max_coefs_fx, L_abs(coefs_fx[n_coeff])) < 0)
                {
                    max_coefs_fx = L_abs(coefs_fx[n_coeff]);
                }
            }
            cs = norm_l(max_coefs_fx);
            tmp2 = fb_bwe_subband[add(n_band,1)];
            FOR ( n_coeff = fb_bwe_subband[n_band]; n_coeff < tmp2; n_coeff++ )
            {
                tmp = extract_h(L_shl(coefs_fx[n_coeff], cs));/*12 + cs - 16 */
                L_tmp = L_mult0(tmp, tmp);/*2*cs - 8 */
                energy_fx = L_add(energy_fx, L_shr(L_tmp, 5));
            }

            IF (energy_fx != 0)
            {
                L_tmp = Mult_32_16(energy_fx, fb_inv_bwe_subband_width_fx[n_band]);/*2*cs - 13 + 18 - 15 */

                exp = norm_l(L_tmp);
                tmp = Log2_norm_lc(L_shl(L_tmp, exp));
                exp = sub(30, add(exp, 2*cs-13));

                L_tmp = Mpy_32_16(exp, tmp, 24660); /* Q14 */ /*10log10(2) in Q13 */
                hq_generic_fenv_fx[add(n_band,nenv)] = round_fx(L_shl(L_tmp, 10));/*Q8 */
            }
            ELSE
            {
                hq_generic_fenv_fx[add(n_band,nenv)] = 0;
                move16();
            }
        }
    }

    freq_weights_fx(hq_generic_fenv_fx, w_NOR_fx, w_env_fx, nenv);

    FOR ( n_band = 0; n_band < nenv; n_band++ )
    {
        hq_generic_fenv_fx[n_band] = sub(hq_generic_fenv_fx[n_band], Mean_env_fx[n_band]);
    }

    IF ( st_fx->bwidth_fx == FB )
    {
        FOR ( n_band = 0; n_band < DIM_FB; n_band++ )
        {
            hq_generic_fenv_fx[add(n_band,nenv)] = sub(shr(hq_generic_fenv_fx[add(n_band,nenv)], 1), Mean_env_fb_fx[n_band]);
        }
    }


    /* Energy VQ */
    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4) <= 0 )
    {
        msvq_interpol_fx( hq_generic_fenv_fx, w_env_fx, indice );
    }
    ELSE
    {
        msvq_interpol_2_fx(hq_generic_fenv_fx, w_env_fx, indice, nenv);
    }

    IF ( sub(st_fx->bwidth_fx , FB) == 0 )
    {
        indice[5] = vqSimple_w_fx(hq_generic_fenv_fx+nenv, hq_generic_fenv_fx+nenv, EnvCdbkFB_fx, NULL, DIM_FB, N_CB_FB, 0);
    }

    push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[0], 5 );
    push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[1], 7 );
    push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[2], 6 );
    push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[3], 5 );

    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4) <= 0 )
    {
        push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[4], 6 );
    }
    ELSE
    {
        push_indice_fx( st_fx, IND_SWB_FENV_HQ, indice[4], 5 );
    }

    IF ( sub(st_fx->bwidth_fx , FB) == 0 )
    {
        push_indice_fx( st_fx, IND_FB_FENV_HQ, indice[5], 5 );
    }

    FOR ( n_band = 0; n_band < nenv; n_band++ )
    {
        tmp = add(hq_generic_fenv_fx[n_band], Mean_env_fx[n_band]);/*8 */
        L_tmp = L_mult(tmp, 21771);/*26 */
        L_tmp = L_shr(L_tmp, 10);/*16 */
        L_Extract(L_tmp, &exp, &tmp);/* */
        tmp = extract_l(Pow2(13, tmp));
        exp = sub(exp, 13);
        hq_generic_fenv_fx[n_band] = shl(tmp, add(exp, 1));/*1 */
    }


    IF ( sub(st_fx->bwidth_fx , FB) == 0 )
    {
        FOR ( n_band = 0; n_band < DIM_FB; n_band++ )
        {
            tmp = add(hq_generic_fenv_fx[add(n_band,nenv)], Mean_env_fb_fx[n_band]);/*7 */
            L_tmp = L_mult(tmp, 21771);/*25 */
            L_tmp = L_shr(L_tmp, 9);/*16 */
            L_Extract(L_tmp, &exp, &tmp);
            tmp = extract_l(Pow2(13, tmp));
            exp = sub(exp, 13);
            hq_generic_fenv_fx[add(n_band,nenv)] = shl(tmp, add(exp, 1));/*2 */
        }
    }

    return;
}


