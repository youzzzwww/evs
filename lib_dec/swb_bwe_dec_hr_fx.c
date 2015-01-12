/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "rom_dec_fx.h"     /* Static table prototypes                */
#include "cnst_fx.h"        /* Static table prototypes                */
#include "stl.h"

#define Q_GUARD 1
#define Q_32_BITS 14 /* scaling of 't_audio32' */
#define MAKE_PSEUDO_FLT(v,e) ((((Word32)(v))<<16) + (e))

/*-----------------------------------------------------------*
 * Gain_Dequant_HR()
 *
 * Returns decoded gain quantized between the specified
 * range using the specified number of levels.
 *-----------------------------------------------------------*/
/* It would be more efficient (PROM wise) to combine
   with gain_dequant_fx(). But for now the 'max' param has
   a larger allowed range here (Q13) than in gain_dequant_fx()
   where is is in Q15. This applies for the 'min' param too.
   Here it is Q6 in gain_dequant_fx() it is Q0. But merging the
   two functions would be less efficient (Performance Wise)
   since the function here doesn't use 'Log2_norm_lc' at all
   versus gain_dequant_fx() which does. */
static Word16 Gain_Dequant_HR(   /* o:   decoded gain (Q13)            */
    Word16 index,          /* i:   quantization index            */
    const Word16 min,            /* i:   value of lower limit (Q13)    */
    const Word16 bits,           /* i:   number of bits to dequantize  */
    Word16 *exp            /* o:   exponent of Decoded Gain      */
)
{
    Word32 L_mini, L_fact;
    Word16 gain;
    Word32 L_temp;
    Word16 exp1,exp2, p2_frac,p2_int;

    L_mini=0; /* no complexity counted, just to remove warning */
    L_fact=0; /* no complexity counted, just to remove warning */

    move32();
    move32();
    IF (sub(min, G_AVQ_MIN_FX) == 0)
    {
        L_mini = MAKE_PSEUDO_FLT(26214, 15); /* 0.8 in Q15 */
        L_fact = MAKE_PSEUDO_FLT(14145, 11); /* Log2(96) - Log2(0.8) in Q11 */
    }
    ELSE IF (sub(min, G_AVQ_MIN_DIV10_FX) == 0)
    {
        L_mini = MAKE_PSEUDO_FLT(20972, 18); /* 0.8*0.1 in Q18 */
        L_fact = MAKE_PSEUDO_FLT(20949, 11); /* Log2(96) - Log2(0.8*0.1) in Q11 */
    }
    ELSE IF (sub(min, G_CODE_MIN_FX) == 0)
    {
        L_mini = MAKE_PSEUDO_FLT(20972, 20); /* 0.02 in Q20 */
        L_fact = MAKE_PSEUDO_FLT(32628, 12); /* Log2(5) - Log2(0.02) in Q12 */
    }
    ELSE IF (sub(min, G_CODE_MIN_TC192_FX) == 0)
    {
        L_mini = MAKE_PSEUDO_FLT(19661, 15); /* 0.6 in Q15 */
        L_fact = MAKE_PSEUDO_FLT(24963, 12); /* Log2(41) - Log2(0.6) in Q12 */
    }
    ELSE IF (sub(min, MIN_GLOB_GAIN_BWE_HR_FX) == 0)
    {
        L_mini = MAKE_PSEUDO_FLT(24576, 13); /* 3.0 in Q13 */
        L_fact = MAKE_PSEUDO_FLT(30232, 12); /* Log2(500) - Log2(3) in Q12 */
    }
    /* levels = 1<<bits;*/
    /* c_min = (float)log10(min);*/
    /* c_mult = (float) ((levels-1)/(log10(max)-c_min));*/
    /* gain = (float)pow( 10.0, (((float)index)/c_mult) + c_min );*/

    L_temp = L_mult0(extract_h(L_fact), inv_tbl_2n_minus1[bits]);
    exp1 = norm_s(index);
    index = shl(index, exp1);
    /* inv_tbl has variable Q, with Q0 being at [2]*/
    /* So 'exp1 = sub(sub(15, extract_l(L_fact)), exp1) - (bits-2)' is written as:*/
    exp1 = sub(sub(sub(15+2, extract_l(L_fact)), exp1), bits);
    exp2 = norm_l(L_temp);
    L_temp = L_shl(L_temp, exp2);
    exp2 = sub(exp2, exp1);
    L_temp = Mult_32_16(L_temp, index);
    L_temp = L_shr(L_temp, exp2);

    p2_frac = L_Extract_lc(L_temp, &p2_int);
    L_temp = Pow2(30, p2_frac); /* 30 to get the most bits */
    exp1 = sub(30-16-15, p2_int);
    exp1 = add(exp1, extract_l(L_mini));

    gain = round_fx(L_temp);

    L_temp = L_mult(gain, extract_h(L_mini));
    exp2 = norm_l(L_temp);
    L_temp = L_shl(L_temp, exp2);

    gain = round_fx(L_temp);
    exp1 = add(exp1, exp2);

    *exp = exp1;
    move16();

    return gain;
}

/*-------------------------------------------------------------------*
 * TD_Postprocess()
 *
 * post processing in time domain for td/fd switching
 *-------------------------------------------------------------------*/

static Word16 TD_Postprocess(     /* o  : gain in Q15               */
    Word16 hb_synth_fx[],   /* i/o: high-band synthesis       */
    const Word16 hb_synth_fx_exp, /* i  : Q of high-band synthesis  */
    const Word16 input_frame,     /* i  : frame length              */
    const Word16 last_extl        /* i  : last extension layer      */
)
{
    Word16 i;
    Word16 pos, ind1, ind2;
    Word16 max_samp, temp, temp1, temp2, temp3;
    Word32 L_Energy, L_Energy2;

    max_samp = abs_s(hb_synth_fx[0]);
    pos = 0;
    move16();
    FOR( i = 1; i < input_frame; i++ )
    {
        temp = abs_s(hb_synth_fx[i]);
        if( sub(temp, max_samp) > 0 )
        {
            pos = i;
            move16();
        }
        max_samp = s_max(temp, max_samp);
    }

    IF( sub(pos, 160) < 0 )
    {
        L_Energy = Calc_Energy_Autoscaled(hb_synth_fx + sub(input_frame, 80), hb_synth_fx_exp, 80, &temp1 );
    }
    ELSE
    {
        L_Energy = Calc_Energy_Autoscaled(hb_synth_fx, hb_synth_fx_exp, 80, &temp1);
    }

    ind1 = s_max(0, sub(pos, 40));
    ind2 = s_min( input_frame, add(pos, 40) );
    temp3 = sub(ind2, ind1);

    L_Energy2 = Calc_Energy_Autoscaled(hb_synth_fx + ind1, hb_synth_fx_exp, temp3, &temp2);

    /* Float Code: "gain_flt = min( 1.0f, 1.0f/sqrt( 80*tmpF/(gain_flt*temp) ) )"
     * Multiply by 80 (eq to Mult by 1.25 and 64)
     * So Div by 2 to avoid overflow
     * Add 1/8
     * Adjust Exponent (-1 for Dib by2, -6 for Missing Mult by 64
     */
    L_Energy2 = L_add(L_shr(L_Energy2, 1), L_shr(L_Energy2, 3));
    temp2 = sub(temp2, 1+6);

    /* Normalize 'temp3' */
    temp = norm_s(temp3);
    temp3 = shl(temp3, temp);
    /* Adjust Exponent of Energy #1 */
    temp1 = add(temp1, temp);
    /* Mult by 'temp3' */
    L_Energy = Mult_32_16(L_Energy, temp3);
    L_Energy = L_max(L_Energy, 1);

    temp1 = sub(temp1, 15); /* because of Mpy_32_16_1 */

    L_Energy = Sqrt_Ratio32(L_Energy, temp1, L_Energy2, temp2, &temp);

    /* Put Back to Q31 (Let it saturate to 0.99999 in fx because we wanted to limit the gain to 1.0 anyways) */
    L_Energy = L_shl(L_Energy, temp);
    temp = round_fx(L_Energy);

    test();
    IF( sub(last_extl, SWB_BWE) == 0 || sub(last_extl, FB_BWE) == 0 )
    {
        FOR( i = ind1; i < input_frame; i++ )
        {
            hb_synth_fx[i] = mult_r(temp, hb_synth_fx[i]);
            move16();
        }
    }
    ELSE
    {
        FOR( i = ind1; i < ind2; i++ )
        {
            hb_synth_fx[i] = mult_r(temp, hb_synth_fx[i]);
            move16();
        }

        IF ( sub(ind2, input_frame) != 0 )
        {
            /* alpha_flt = (gain_flt > 0.5f) ? 1.0f : 0.5f;*/
            /* beta_flt = (alpha_flt - gain_flt)/sub(input_frame, ind2);*/
            temp2 = sub(16384, temp);
            if (temp2 < 0)
                temp2 = add(temp2, 16384);
            temp3 = sub(input_frame, ind2);
            /* Inverse 'temp3' */
            temp1 = norm_s(temp3);
            temp3 = shl(temp3, temp1);
            temp3 = div_s(16384, temp3);
            L_Energy2 = L_mult0(temp2, temp3);
            temp1 = add(temp1, 1); /* because we used 0.5 (16384) to inverse and not 1.0 (32768) */
            /* Back to Q31 */
            L_Energy2 = L_shr(L_Energy2, temp1);

            FOR( i = ind2; i < input_frame; i++ )
            {
                hb_synth_fx[i] = mult_r(round_fx(L_Energy), hb_synth_fx[i]);
                move16();
                L_Energy = L_add(L_Energy, L_Energy2);
            }
        }
    }

    return temp; /* in Q15 */
}

/*-------------------------------------------------------------------*
 * swb_bwe_dec_hr_fx()
 *
 * HR SWB BWE decoder
 *-------------------------------------------------------------------*/

Word16 swb_bwe_dec_hr_fx(            /* o  : Exponent of SHB synthesis   */
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure     */
    const Word16 *syn_12k8_16k_fx,/* i  : ACELP core synthesis @16kHz */
    const Word16 exp,             /* i  : Exponent of core synthesis  */
    Word16 *hb_synth_fx,    /* o  : SHB synthesis               */
    const Word16 output_frame,    /* i  : frame length                */
    const Word16 unbits,          /* i  : number of core unused bits  */
    const Word16 pitch_buf[]      /* i  : pitch buffer                */
)
{
    Word16 i, j, k, nBits, nBits_total, nBits_block, Nsv, Nsv2, width_noncoded;
    Word16 is_transient, tmpS, incr, IsTransient, pos;
    Word16 x_norm[NSV_MAX*(WIDTH_BAND+1)], x_norm1[NSV_MAX*(WIDTH_BAND+1)];
    Word32 t_audio32_tmp[L_FRAME48k];
    Word32 t_audio32[L_FRAME48k];
    Word16 t_audio_exp;
    Word16 en_band[N_BANDS_BWE_HR];
    Word16 ind1, ind2;
    Word32 L_EnergyLT, L_Energy;
    Word16 nq[NSV_MAX], nq2[NSV_MAX], nq_tmp[NSV_MAX];
    Word16 alpha;
    Word16 temp, temp2, temp3, temp4;
    Word16 en_noncoded, min_env;
    Word16 gain_fx = 0, gain2_fx, exp1, exp2;
    Word16 len;
    Word16 pitch;
    Word32 L_temp, L_temp2;
    Word32 L_tilt_wb;
    Word16 hb_synth_fx_exp;
    Word16 *ptr16;
    Word32 *ptr32;
    Word32 L_ener_all, L_ener_saved;
    Word16 ener_all_exp, ener_saved_exp;
    Word16 *t_audio, *t_audio_tmp;
    Word16 env = 0;
    Word16 exp_L, inv_L, frac;

    /* Use 't_audio32_tmp' Word32 Buffer as two Word16 Buffers to save local Stack. */
    /* There is no possible overlap so it is ok */
    t_audio = (Word16 *)&t_audio32_tmp[0];
    t_audio_tmp = (Word16 *)&t_audio32_tmp[L_FRAME48k/2];

    /*---------------------------------------------------------------------*
     * initializations
     *---------------------------------------------------------------------*/

    set16_fx(t_audio, 0, output_frame);
    set32_fx(t_audio32, 0, output_frame);
    exp2 = 0;
    move16();
    Nsv2 = 0;
    move16();
    /* only to suppress warnings (no complexity counted) */
    gain2_fx = 0;
    move16();
    ind2 = 0;
    move16();
    L_ener_saved = 0;
    move16();
    ener_saved_exp = 0;
    move16();

    /* reset memories in case that last frame was a different technology */
    test();
    IF( sub(st_fx->last_core_fx, HQ_CORE) == 0 || sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0 )
    {
        set16_fx( st_fx->old_out_hr_fx, 0, L_FRAME48k );
        st_fx->old_out_hr_exp_fx = 14;
        move16();
    }

    /* calculate SWB BWE bit-budget */
    /* nBits = st->extl_brate/50 + unbits */
    nBits = add(320,unbits);    /* st->extl_brate_fx is always 16kbps */
    nBits_total = nBits;
    move16();

    /*---------------------------------------------------------------------*
     * calculate tilt of the core synthesis
     *---------------------------------------------------------------------*/
    L_tilt_wb = calc_tilt_bwe_fx( syn_12k8_16k_fx, exp, L_FRAME16k );
    L_temp = L_mac(1L, 8192, pitch_buf[0]);
    FOR (i = 1; i < NB_SUBFR16k-1; i++)
    {
        L_temp = L_mac(L_temp, 8192, pitch_buf[i]);
    }
    pitch = mac_r(L_temp, 8192, pitch_buf[i]);
    /* pitch now in Q4 (Q6 div by 4) */

    /*---------------------------------------------------------------------*
     * FEC, or good frame decoding
     *---------------------------------------------------------------------*/

    IF( st_fx->bfi_fx )
    {
        is_transient = st_fx->old_is_transient_hr_bwe_fx;
        move16();

        /* Replication of the last spectrum, with an attenuation */
        test();
        test();
        IF( (sub(st_fx->clas_dec, VOICED_CLAS) == 0 || sub(st_fx->clas_dec, INACTIVE_CLAS) == 0) && sub(st_fx->nbLostCmpt, 3) <= 0 )
        {
            alpha = 26214; /* 0.80 */ move16();
            t_audio_exp = 0;
            move16();
        }
        ELSE IF ( is_transient )
        {
            alpha = 19661 /* 0.15 */;
            move16();
            t_audio_exp = 2;
            move16();
        }
        ELSE
        {
            alpha = 19661 /* 0.30 */;
            move16();
            t_audio_exp = 1;
            move16();
        }

        IF( is_transient )
        {
            /* output_frame == L_FRAME48k */
            tmpS = (2*END_FREQ_BWE_FULL_FB/50)/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_START_FREQ_COEF;
            move16();
            /* set BWE spectrum length */
            if( sub(output_frame, L_FRAME32k) == 0 )
            {
                tmpS = L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_START_FREQ_COEF;
                move16();
            }

            temp = shr(output_frame, 2);
            pos = NUM_TRANS_START_FREQ_COEF;
            move16();
            ind1 = 0;
            move16();
            /* reconstruct */
            len = 0;
            move16();
            /* Here t_audio was initialy filled with zeros */
            /* So, after the loop, the Q will be 'Q_32_Bits' */
            FOR( k=0; k<NUM_TIME_SWITCHING_BLOCKS; k++ )
            {
                temp4 = sub(st_fx->t_audio_prev_fx_exp[k], Q_32_BITS);
                temp4 = add(temp4, t_audio_exp);
                FOR( i=0; i<tmpS; i++ )
                {
                    L_temp = L_mult(alpha, st_fx->t_audio_prev_fx[i + ind1]);
                    L_temp = L_shr(L_temp, temp4);
                    t_audio32[pos + i] = L_temp;
                    move32();
                }
                ind1 = add(ind1, tmpS);
                pos = add(pos, temp);
            }
        }
        ELSE
        {
            /* output_frame == L_FRAME48k */
            tmpS = 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_START_FREQ_COEF;
            move16();
            /* set BWE spectrum length */
            if( sub(output_frame, L_FRAME32k) == 0 )
            {
                tmpS = L_FRAME32k - NUM_NONTRANS_START_FREQ_COEF;
                move16();
            }

            /* reconstruct */
            /* Here t_audio was initialy filled with zeros */
            /* So, after the loop, the Q will be 'Q_32_Bits' */
            temp4 = sub(st_fx->t_audio_prev_fx_exp[0], Q_32_BITS);
            temp4 = add(temp4, t_audio_exp);
            ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF];
            FOR( i=0; i<tmpS; i++ )
            {
                L_temp = L_mult(alpha, st_fx->t_audio_prev_fx[i]);
                L_temp = L_shr(L_temp, temp4);
                *ptr32++ = L_temp;
                move32();
            }
        }
        /* Save transform coefficients for the next frame (needed in case of frame erasures) */
        temp = NUM_NONTRANS_START_FREQ_COEF;
        move16(); /* not necessary but improves readability and allows a larger common code path */
        IF( output_frame == L_FRAME32k )
        {
            pos = L_FRAME32k - NUM_NONTRANS_START_FREQ_COEF;
        }
        ELSE  /* output_frame == L_FRAME48k */
        {
            pos = 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_START_FREQ_COEF;
        }
        temp4 = Find_Max_Norm32(t_audio32 + temp, pos);
        Copy_Scale_sig32_16(t_audio32 + temp, st_fx->t_audio_prev_fx, pos, temp4);
        st_fx->t_audio_prev_fx_exp[0] = add(Q_32_BITS, temp4);
        move16();

        st_fx->L_mem_EnergyLT_fx = Mul_flt32_Q15(st_fx->L_mem_EnergyLT_fx, &st_fx->mem_EnergyLT_fx_exp, alpha);
        move32();
        st_fx->mem_EnergyLT_fx_exp = add(st_fx->mem_EnergyLT_fx_exp, t_audio_exp);

        /* Set Exponent */
        t_audio_exp = Q_32_BITS;
        move16();
        exp_L = norm_s(output_frame);
        inv_L = div_s(shl(1,sub(14,exp_L)), output_frame); /*Q(29-exp_L)*/

        /*Q(st_fx->mem_EnergyLT_fx_exp+29-exp_L-15) -> Q(st_fx->mem_EnergyLT_fx_exp-exp_L+14)*/
        st_fx->L_mem_EnergyLT_fx = Mul_flt32_Q15(st_fx->L_mem_EnergyLT_fx, &st_fx->mem_EnergyLT_fx_exp, inv_L);
        move32();
        IF(st_fx->L_mem_EnergyLT_fx != 0)
        {
            exp1 = norm_l(st_fx->L_mem_EnergyLT_fx);
            frac = extract_h(L_shl(st_fx->L_mem_EnergyLT_fx, exp1));
            exp1 = sub(exp1,sub(16,sub(st_fx->mem_EnergyLT_fx_exp,exp_L)));

            temp = div_s(16384, frac);
            L_temp = L_deposit_h(temp);
            gain_fx = extract_l(L_shl(Isqrt_lc(L_temp, &exp1), sub(exp1, 2))); /*Q(31-exp + (exp-3)) -> Q13*/
        }

        env = 512;
        move16();
    }
    ELSE
    {
        /*---------------------------------------------------------------------*
         * get transient frame flag
         *---------------------------------------------------------------------*/

        is_transient = (Word16) get_next_indice_fx( st_fx, 1 );

        IF( is_transient )
        {
            nBits = -1;
            move16(); /* is_transient flag */
            nBits_block = shr(nBits_total, 2);
            nBits = add(nBits, s_and(nBits_total, 3));

            /* set width of noncoded (blind estimated) spectrum */
            test();
            IF( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) == 0 || sub(output_frame, L_FRAME32k) == 0 )
            {
                width_noncoded = L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF;
                move16();
                tmpS = L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF_EFF;
                move16();
            }
            ELSE  /* st->extl == FB_BWE_HIGHRATE */
            {
                width_noncoded = (2*END_FREQ_BWE_FULL_FB/50)/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF;
                move16();
                tmpS = (2*END_FREQ_BWE_FULL_FB/50)/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF_EFF;
                move16();
            }

            /*---------------------------------------------------------------------*
             * transient frames: processing in blocks (subframes)
             *---------------------------------------------------------------------*/
            len = 0;
            move16();
            FOR( k = 0; k < NUM_TIME_SWITCHING_BLOCKS; k++ )
            {
                nBits = add(nBits, nBits_block);

                /*---------------------------------------------------------------------*
                 * global gain and envelope decoding
                 *---------------------------------------------------------------------*/

                /* get global gain */
                ind1 = (Word16) get_next_indice_fx( st_fx, NBITS_GLOB_GAIN_BWE_HR );
                gain_fx = Gain_Dequant_HR( ind1, MIN_GLOB_GAIN_BWE_HR_FX, NBITS_GLOB_GAIN_BWE_HR, &exp1 );
                nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR);

                /* get energy per band */
                IF( k == 0 )
                {
                    ind1 = (Word16)get_next_indice_fx( st_fx, NBITS_ENVELOPE_BWE_HR_TR );
                    ind2 = ind1;
                    move16();
                    nBits = sub(nBits, NBITS_ENVELOPE_BWE_HR_TR);
                }
                ELSE
                {
                    ind1 = (Word16)get_next_indice_fx( st_fx, NBITS_ENVELOPE_BWE_HR_TR - 1 );
                    if( sub(ind2, 8) >= 0 )
                    {
                        ind1 = add(ind1, NUM_ENVLOPE_CODE_HR_TR2);
                    }
                    nBits = sub(nBits, NBITS_ENVELOPE_BWE_HR_TR - 1);
                }

                temp = shl(ind1, 1);
                en_band[0] = swb_hr_env_code3_fx[temp];
                move16();
                en_band[1] = swb_hr_env_code3_fx[add(temp, 1)];
                move16();

                /*env = add(shr(en_band[0], 1), shr(en_band[1], 1));*/
                env = mac_r(L_mult(en_band[0], 16384), en_band[1], 16384);

                /*---------------------------------------------------------------------*
                 * estimate energy of noncoded spectrum (14.4-20kHz)
                 *---------------------------------------------------------------------*/

                en_noncoded = en_band[N_BANDS_TRANS_BWE_HR-1];
                move16();

                IF( sub(st_fx->extl_fx, FB_BWE_HIGHRATE) == 0)
                {
                    ind1 = (Word16)get_next_indice_fx( st_fx, NBITS_HF_GAIN_BWE_HR );
                    nBits = sub(nBits, NBITS_HF_GAIN_BWE_HR);

                    IF (sub(ind1, 1) == 0)
                    {
                        en_noncoded = round_fx(L_mult0(en_noncoded, BWE_HR_TRANS_EN_LIMIT1_FX_Q16));
                    }

                    IF( sub(ind1, 2) == 0 )
                    {
                        en_noncoded = round_fx(L_mult0(en_noncoded, BWE_HR_TRANS_EN_LIMIT2_FX_Q16));
                    }

                    IF( sub(ind1, 3) == 0 )
                    {
                        en_noncoded = round_fx(L_mult0(en_noncoded, BWE_HR_TRANS_EN_LIMIT3_FX_Q16));
                    }
                }

                /*---------------------------------------------------------------------*
                 * AVQ decoding (dequantize normalized spectrum)
                 *---------------------------------------------------------------------*/

                Nsv = (NUM_TRANS_END_FREQ_COEF - NUM_TRANS_START_FREQ_COEF) / WIDTH_BAND;
                move16();
                AVQ_demuxdec_fx( st_fx, x_norm, &nBits, Nsv, nq );

                temp = add(len, NUM_TRANS_START_FREQ_COEF);
                /* 't_audio' in Q8 */
                t_audio_exp = 8;
                move16();
                FOR( i=0; i < Nsv*WIDTH_BAND; i++ )
                {
                    t_audio[temp + i] = shl(x_norm[i], t_audio_exp);
                    move16();
                }

                /* apply noise-fill */
                swb_hr_noise_fill_fx( is_transient, NUM_TRANS_START_FREQ_COEF, NUM_TRANS_END_FREQ_COEF,
                                      round_fx(L_shl(L_tilt_wb, 3)), /* Q(24+3-16) -> Q11 */
                                      pitch, nq, Nsv, &st_fx->bwe_highrate_seed_fx, t_audio+temp, t_audio_exp );

                /* Go from Q't_audio_exp' on 16 Bits to 'Q_32_BITS' on 32 bits */
                temp2 = i_mult2(WIDTH_BAND, Nsv);
                ptr16 = &t_audio[temp];
                move16();
                ptr32 = &t_audio32[temp];
                j = shl(1, sub(Q_32_BITS, t_audio_exp));
                FOR (i = 0; i< temp2; i++)
                {
                    /* put in 'Q_32_BITS' in a 32 Bits */
                    L_temp = L_mult0(*ptr16++, j);
                    *ptr32++ = L_temp;
                    move32();
                }

                /*---------------------------------------------------------------------*
                 * reconstruction
                 *---------------------------------------------------------------------*/

                temp = add(temp, NUM_TRANS_END_FREQ_COEF_EFF-NUM_TRANS_START_FREQ_COEF);
                pos = sub(temp, tmpS);
                ptr32 = &t_audio32[temp];
                /* reconstruct 14-16(20) kHz spectrum */
                FOR( j = 0; j < tmpS; j++ )
                {
                    *ptr32++ = L_shr(t_audio32[pos + j], 1);
                    move32();
                }

                temp = i_mult2(shr(output_frame, 2), k);

                temp2 = add(NUM_TRANS_START_FREQ_COEF, temp);
                ptr32 = &t_audio32[temp2];
                /* envelope denormalization */
                FOR( i=0; i<N_BANDS_TRANS_BWE_HR; i++ )
                {
                    FOR( j=0; j<WIDTH_TRANS_FREQ_COEF; j++ )
                    {
                        L_temp = Mult_32_16(*ptr32, en_band[i]);
                        L_temp = L_shl(L_temp, 6); /* by 6 because 'en_band' is in Q9 */
                        *ptr32++ = L_temp;
                        move32();
                    }
                }

                temp2 = add(NUM_TRANS_END_FREQ_COEF_EFF, temp);
                j = sub(tmpS, width_noncoded);
                ptr16 = &t_audio[add(temp2, j)];
                move16();
                ptr32 = &t_audio32[add(temp2, j)];
                /* envelope denormalization of 14.4-16(20) kHz spectrum */
                FOR( ; j < tmpS; j++ )
                {
                    L_temp = Mult_32_16(*ptr32, en_noncoded);
                    L_temp = L_shl(L_temp, 6); /* by 6 because 'en_noncoded' is in Q9 */
                    *ptr32++ = L_temp;
                    move32();
                }

                temp2 = add(NUM_TRANS_START_FREQ_COEF, temp);
                ptr16 = &t_audio[temp2];
                move16();
                ptr32 = &t_audio32[temp2];
                temp4 = i_mult2(NSV_OVERLAP, shr(WIDTH_BAND, 2));
                /* overlap region */
                FOR( i=0; i<temp4; i++ )
                {
                    L_temp = Mult_32_16(*ptr32, overlap_coefs_fx_0_9[shl(i, 2)]); /* overlap_coefs_fx_0_9 in Q15 */
                    *ptr32++ = L_temp;
                    move32();
                }

                temp2 = add(NUM_TRANS_START_FREQ_COEF, temp);
                ptr16 = &t_audio[temp2];
                move16();
                ptr32 = &t_audio32[temp2];
                temp4 = add(i_mult2(WIDTH_TRANS_FREQ_COEF, N_BANDS_TRANS_BWE_HR), width_noncoded);
                temp3 = sub(15, exp1);
                /* apply global gain */
                FOR( i=0; i<temp4; i++ )
                {
                    L_temp = Mult_32_16(*ptr32, gain_fx);
                    L_temp = L_shl(L_temp, temp3);
                    *ptr32++ = L_temp;
                    move32();
                }

                /* save transform coefficients for the next frame (needed in case of frame erasures) */
                temp = add(NUM_TRANS_START_FREQ_COEF, len);
                IF( output_frame == L_FRAME32k )
                {
                    pos = L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_START_FREQ_COEF;
                    move16();
                }
                ELSE  /* output_frame == L_FRAME48k */
                {
                    pos = (2*END_FREQ_BWE_FULL_FB/50)/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_START_FREQ_COEF;
                }
                temp4 = Find_Max_Norm32(t_audio32 + temp, pos);
                Copy_Scale_sig32_16(t_audio32 + temp, st_fx->t_audio_prev_fx + i_mult2(k, pos), pos, temp4);
                st_fx->t_audio_prev_fx_exp[k] = add(Q_32_BITS, temp4);
                move16();
                len = add(len, shr(output_frame, 2));
                /* attenuate HFs in case of band-width switching */
                IF( st_fx->bws_cnt1_fx > 0 )
                {
                    temp = shr(output_frame, 2);
                    temp = i_mult(k, temp);
                    temp = add(NUM_TRANS_START_FREQ_COEF, temp);
                    temp2 = i_mult(st_fx->bws_cnt1_fx, 1638);  /*Q15*/
                    FOR( i=0; i < L_FRAME32k/4; i++ )
                    {
                        t_audio[temp + i] = mult_r(t_audio[temp + i], temp2);
                        move16();
                    }
                }
            }
        }
        ELSE    /* !is_transient */
        {
            /* subtract one bit for is_transient flag */
            nBits = sub(nBits, 1);

            /*---------------------------------------------------------------------*
             * global gain and envelope decoding
             *---------------------------------------------------------------------*/

            /* get global gain */
            ind1 = (Word16) get_next_indice_fx( st_fx, NBITS_GLOB_GAIN_BWE_HR );
            gain_fx = Gain_Dequant_HR( ind1, MIN_GLOB_GAIN_BWE_HR_FX, NBITS_GLOB_GAIN_BWE_HR, &exp1 );

            /* get energy per band */
            ind1 = (Word16) get_next_indice_fx( st_fx, NBITS_ENVELOPE_BWE_HR1 );
            ind2 = (Word16) get_next_indice_fx( st_fx, NBITS_ENVELOPE_BWE_HR2 );

            temp = shl(ind1, 1);
            en_band[0] = swb_hr_env_code1_fx[temp];
            move16();
            en_band[1] = swb_hr_env_code1_fx[add(temp, 1)];
            move16();
            temp = shl(ind2, 1);
            en_band[2] = swb_hr_env_code2_fx[temp];
            move16();
            en_band[3] = swb_hr_env_code2_fx[add(temp, 1)];
            move16();

            /*env = add(add(shr(en_band[0], 2), shr(en_band[1], 2)), add(shr(en_band[2], 2), shr(en_band[3], 2)));*/
            env = mac_r(L_mac(L_mac(L_mult(en_band[0], 8192), en_band[1], 8192), en_band[2], 8192), en_band[3], 8192);

            /*---------------------------------------------------------------------*
             * choose sub-bands to be dequantized
             *---------------------------------------------------------------------*/

            /* find the subband with the min envelope */
            pos = 0;
            move16();
            min_env = en_band[0];
            move16();
            FOR (j = 1; j < N_BANDS_BWE_HR; j++)
            {
                if (sub(en_band[j], min_env) < 0)
                {
                    pos = j;
                    move16();
                }
                min_env = s_min(min_env, en_band[j]);
            }

            /* decide the spectrum to be dequantized */
            i = NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF;
            move16();
            IF( sub(nBits_total, NBITS_THRESH_BWE_HR) <= 0 )
            {
                i = sub(NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF - 64, shl(s_and(pos, 1), 3));
            }

            nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR + NBITS_ENVELOPE_BWE_HR1 + NBITS_ENVELOPE_BWE_HR2);

            /*---------------------------------------------------------------------*
             * estimate energy of noncoded spectrum (14.4-20kHz)
             *---------------------------------------------------------------------*/

            en_noncoded = mult_r(min_env, 16384);

            IF( sub(st_fx->extl_fx, FB_BWE_HIGHRATE) == 0 )
            {
                ind1 = (Word16) get_next_indice_fx( st_fx, NBITS_HF_GAIN_BWE_HR );
                nBits = sub(nBits, NBITS_HF_GAIN_BWE_HR);

                if (sub(ind1, 1) == 0)
                {
                    /* en_noncoded = BWE_HR_NONTRANS_EN_LIMIT1*(0.5*min_env) ==> 0.25*min_env */
                    en_noncoded = mult_r(min_env, BWE_HR_NONTRANS_EN_LIMIT2_FX_Q15/2);
                }

                IF (sub(ind1, 2) == 0)
                {
                    /* en_noncoded = 2.0*BWE_HR_NONTRANS_EN_LIMIT2*(0.5*min_env) ==> 1.2*min_env */
                    en_noncoded = round_fx(L_shl(L_mult(BWE_HR_NONTRANS_EN_LIMIT2_FX_Q14, min_env), 1));
                }

                if (sub(ind1, 3) == 0)
                {
                    /* en_noncoded = 2.0*BWE_HR_NONTRANS_EN_LIMIT3*(0.5*min_env) ==> 0.8*min_env */
                    en_noncoded = mult_r(BWE_HR_NONTRANS_EN_LIMIT3_FX_Q15, min_env);
                }
            }

            /*---------------------------------------------------------------------*
             * AVQ decoding (dequantize normalized spectrum)
             *---------------------------------------------------------------------*/

            /* Nsv = i / WIDTH_BAND */
            Nsv = shr(i, 3);
            AVQ_demuxdec_fx( st_fx, x_norm, &nBits, Nsv, nq );

            /*---------------------------------------------------------------------*
             * second stage decoding
             *---------------------------------------------------------------------*/
            IF( sub(nBits, 9 + NBITS_GLOB_GAIN_BWE_HR) >= 0 )
            {
                ind1 = (Word16) get_next_indice_fx( st_fx, NBITS_GLOB_GAIN_BWE_HR );
                gain2_fx = Gain_Dequant_HR( ind1, MIN_GLOB_GAIN_BWE_HR_FX, NBITS_GLOB_GAIN_BWE_HR, &exp2 );
                /* gain2_flt *= 0.0625f */
                exp2 = add(exp2, 4);

                /* calculate the number of subbands according to the rest bits */
                IF( sub(nBits, 396) > 0 )
                {
                    Nsv2 = 33;
                    move16();
                }
                ELSE
                {
                    /* Here what is acheived is an integer divide by 12 with truncation.    */
                    /* nBits/12                                                             */
                    Nsv2 = mult(nBits, 2731);
                    /* But, we have imprecision of the fraction so correction is necessary. */
                    /* We crosscheck if 'Nsv2' is either too high or too low.               */
                    /* Finally, the result must satisfy:                                    */
                    /*   Nsv2 * 12 <= nBits (Nsv2 is not too high) AND                      */
                    /*   nBits -  Nsv2 * 12 < 12 (Nsv2 is the highest divisor)              */
                    L_temp = L_msu0(L_deposit_l(nBits), 12, Nsv2);
                    if (L_sub(L_temp, 12L) >= 0)
                        Nsv2 = add(Nsv2, 1);
                    if (L_temp < 0)
                        Nsv2 = sub(Nsv2, 1);
                }

                nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR);
                AVQ_demuxdec_fx( st_fx, x_norm1, &nBits, Nsv2, nq2 );
            }

            /*---------------------------------------------------------------------*
             * dequantization
             *---------------------------------------------------------------------*/

            /* set 't_audio' exp */
            t_audio_exp = 10;
            move16();
            FOR( i = 0; i < Nsv*WIDTH_BAND; i++ )
            {
                t_audio_tmp[i] = shl(x_norm[i], t_audio_exp);
                move16();
            }

            Copy( nq, nq_tmp, Nsv );

            incr = 0;
            move16();
            ptr16 = x_norm1;
            temp2 = sub(sub(exp2, 15), t_audio_exp); /* go to Q't_audio' */
            FOR( i=0; i<Nsv; i++ )
            {
                test();
                IF( nq[i] == 0 && sub(incr, Nsv2) < 0 )
                {
                    temp = shl(i, 3);
                    FOR( j=0; j<WIDTH_BAND; j++ )
                    {
                        L_temp = L_mult(gain2_fx, *ptr16++);
                        L_temp = L_shr(L_temp, temp2); /* go to Q't_audio' */
                        t_audio_tmp[temp + j] = round_fx(L_temp);
                    }
                    /* 'nq[i] = add(nq[i], nq2[incr])' replaced by 'nq[i] = nq2[incr]' because 'nq[i] == 0' */
                    nq[i] = nq2[incr];
                    move16();
                    incr = add(incr, 1);
                }
            }

            temp3 = shl(-32768, temp2);
            FOR( i=0; incr<Nsv2; i++ )
            {
                IF( nq_tmp[i] != 0 )
                {
                    temp = shl(i, 3);
                    FOR( j=0; j<WIDTH_BAND; j++ )
                    {
                        L_temp = L_mult(gain2_fx, *ptr16++);
                        L_temp = L_msu(L_temp, t_audio_tmp[temp + j], temp3); /* go to -Q't_audio' */
                        L_temp = L_shr(L_temp, temp2); /* go to Q't_audio' */
                        t_audio_tmp[temp + j] = round_fx(L_temp);
                    }
                    nq[i] = add(nq[i], nq2[incr]);
                    move16();
                    incr = add(incr, 1);
                }
            }

            /*---------------------------------------------------------------------*
             * reorder the decoded spectrum
             *---------------------------------------------------------------------*/

            ptr16 = t_audio + NUM_NONTRANS_START_FREQ_COEF;

            IF( sub(nBits_total, NBITS_THRESH_BWE_HR) > 0 )
            {
                Copy( t_audio_tmp, ptr16, NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF );
                /* Update Maximum Written Location (from t_audio + NUM_NONTRANS_START_FREQ_COEF) */
                temp4 = NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF;
            }
            ELSE
            {
                ind1 = add(shl(pos, 6), i_mult2(shr(pos, 1), WIDTH_BAND));
                Copy( t_audio_tmp, ptr16, ind1 );

                /* Update Maximum Written Location (from t_audio + NUM_NONTRANS_START_FREQ_COEF) */
                temp4 = ind1;

                temp = add(pos, 1);
                ind2 = add(shl(temp, 6), i_mult2(shr(temp, 1), WIDTH_BAND));
                Copy( t_audio_tmp + ind1, ptr16 + ind2, NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF - ind2 );
                /* Update Maximum Written Location (from t_audio + NUM_NONTRANS_START_FREQ_COEF) */
                temp4 = s_max(temp4, NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF);

                /* reconstruct non-encoded subband */
                IF( sub(pos, 3) == 0 )
                {
                    Copy( t_audio + NUM_NONTRANS_START_FREQ_COEF + 128, ptr16 + 200, 72 );
                    /* Update Maximum Written Location (from t_audio + NUM_NONTRANS_START_FREQ_COEF) */
                    temp4 = s_max(temp4, 200+72);

                    Copy( nq + 16, nq + 25, 9 );
                }
                ELSE
                {
                    pos = s_and(pos, 1);
                    temp3 = add(64, shl(pos, 3));
                    Copy( ptr16 + ind2, ptr16 + ind1, temp3 );
                    /* Update Maximum Written Location (from t_audio + NUM_NONTRANS_START_FREQ_COEF) */
                    temp4 = s_max(temp4, add(ind1, temp3));

                    ind1 = shr(ind1, 3);
                    ind2 = shr(ind2, 3);

                    j = 33;
                    move16();
                    FOR( i=sub(Nsv, 1); i>=ind1; i-- )
                    {
                        nq[j] = nq[i];
                        move16();
                        j = sub(j, 1);
                    }

                    Copy( nq + ind2, nq + ind1, add(WIDTH_BAND, pos) );
                }
            }

            /* apply noise-fill */
            IF( sub(nBits, 200) < 0 )
            {
                swb_hr_noise_fill_fx( is_transient, NUM_NONTRANS_START_FREQ_COEF, NUM_NONTRANS_END_FREQ_COEF,
                round_fx(L_shl(L_tilt_wb, 3)), /* Q(24+3-16) -> Q11 */
                pitch, nq, Nsv, &st_fx->bwe_highrate_seed_fx, t_audio + NUM_NONTRANS_START_FREQ_COEF, t_audio_exp );
            }

            /* Go from Q't_audio_exp' on 16 Bits to Q16 on 32 bits */
            ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF];
            j = shl(1, sub(Q_32_BITS, t_audio_exp));
            FOR (i = 0; i < temp4; i++)
            {
                /* put in 'Q_32_BITS' in a 32 Bits */
                L_temp = L_mult0(*ptr16++, j);
                *ptr32++ = L_temp;
                move32();
            }

            /*---------------------------------------------------------------------*
             * reconstruction
             *---------------------------------------------------------------------*/

            /* smoothing 12.7-13.1kHz */
            test();
            IF( sub(pos, 3) == 0 && sub(nBits_total, 400) <= 0 )
            {
                ptr16 = &t_audio[NUM_NONTRANS_START_FREQ_COEF + 200 - WIDTH_BAND];
                L_temp = L_mac0(1L/* EPSILON */, *ptr16, *ptr16);
                FOR (i = 1; i < WIDTH_BAND; i++)
                {
                    ptr16++;
                    L_temp = L_mac0(L_temp, *ptr16, *ptr16);
                }
                ptr16++;
                L_temp2 = L_mac0(1L/* EPSILON */, *ptr16, *ptr16);
                FOR (i = 1; i < WIDTH_BAND; i++)
                {
                    ptr16++;
                    L_temp2 = L_mac0(L_temp2, *ptr16, *ptr16);
                }
                L_temp = Sqrt_Ratio32(L_temp, 0, L_temp2, 0, &temp);

                /* if 'temp' is < 0 then it is req to shift right before substracting 1.0 */
                temp2 = s_min(0, temp);
                L_temp = L_shl(L_temp, temp2);
                /* Energy_flt - i*(Energy_flt-1.0)/8.0 */
                L_temp2 = L_add(L_temp, L_shr(-2147483647L-1L, s_max(0, temp))); /* 1.0 in same Q as Sqrt minus the Guard */
                /* / 8.0 */
                L_temp2 = L_shr(L_temp2, 3+Q_GUARD);
                /* Add Guard */
                L_temp = L_shr(L_temp, Q_GUARD);
                /* Set Index */
                ptr16 = &t_audio[NUM_NONTRANS_START_FREQ_COEF + 200];
                ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF + 200];
                /* Set Exponent (relative to t_audio_exp (from 16 bits vector) */
                temp2 = add(sub(temp, temp2), sub(Q_GUARD-(16-Q_32_BITS), t_audio_exp));

                FOR( i=0; i<WIDTH_BAND; i++ )
                {
                    *ptr32++ = L_shl(Mult_32_16(L_temp, *ptr16++), temp2);
                    move32();

                    L_temp = L_sub(L_temp, L_temp2);
                }
            }

            /* reconstruct 14.4-16(20) kHz spectrum */
            width_noncoded = 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_END_FREQ_COEF;
            move16(); /* st->extl == FB_BWE_HIGHRATE */
            test();
            if( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) == 0 || sub(output_frame, L_FRAME32k) == 0 )
            {
                width_noncoded = L_FRAME32k - NUM_NONTRANS_END_FREQ_COEF;
                move16();
            }

            ptr16 = &t_audio[NUM_NONTRANS_END_FREQ_COEF - WIDTH_BAND];
            L_temp = L_mac0(1L/* EPSILON */, *ptr16, *ptr16);
            FOR (i = 1; i < WIDTH_BAND; i++)
            {
                ptr16++;
                L_temp = L_mac0(L_temp, *ptr16, *ptr16);
            }

            ptr16 = &t_audio[sub(NUM_NONTRANS_END_FREQ_COEF, width_noncoded)];
            L_temp2 = L_mac0(1L/* EPSILON */, *ptr16, *ptr16);
            FOR (i = 1; i < WIDTH_BAND; i++)
            {
                ptr16++;
                L_temp2 = L_mac0(L_temp2, *ptr16, *ptr16);
            }
            L_temp = Sqrt_Ratio32(L_temp, 0, L_temp2, 0, &temp);

            /* So part of the copy can be skipped because the loop that follows */
            /* will take the values from t_audio (16 bits) */
            /* Since 'width_noncoded' is always > WIDTH_BAND, we can substract it from the length */
            /* and adjust the offset accordingly */
            Copy32( t_audio32 + sub(NUM_NONTRANS_END_FREQ_COEF+WIDTH_BAND, width_noncoded),
                    t_audio32 + NUM_NONTRANS_END_FREQ_COEF+WIDTH_BAND, sub(width_noncoded, WIDTH_BAND) );

            /* smoothing 14.4-14.8kHz */
            ptr16 = &t_audio[sub(NUM_NONTRANS_END_FREQ_COEF, width_noncoded)];
            ptr32 = &t_audio32[NUM_NONTRANS_END_FREQ_COEF];
            temp = sub(temp, add(t_audio_exp, 16-Q_32_BITS));
            FOR( i=0; i<WIDTH_BAND; i++ )
            {
                L_temp2 = Mult_32_16(L_temp, *ptr16++);
                L_temp2 = L_shl(L_temp2, temp);
                *ptr32++ = L_temp2;
                move32();
            }

            /* envelope denormalization */
            ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF];
            FOR( i=0; i<N_BANDS_BWE_HR; i++ )
            {
                FOR( j=0; j<WIDTH_NONTRANS_FREQ_COEF; j++ )
                {
                    L_temp = Mult_32_16(*ptr32, en_band[i]);
                    L_temp = L_shl(L_temp, 6); /* Because 'en_band' is in Q9 */
                    *ptr32++ = L_temp;
                    move32();
                }
                temp = add(temp, WIDTH_NONTRANS_FREQ_COEF);
            }

            /* equalize 14.4-16(20) kHz spectrum */
            /* (en_band_flt[3] - j*(en_band_flt[3]/WIDTH_BAND - en_noncoded_flt/WIDTH_BAND)) */
            temp = norm_s(en_band[3]);
            L_temp = L_deposit_h(shl(en_band[3], temp));
            L_temp2 = L_mult(sub(en_band[3], en_noncoded), 32768/WIDTH_BAND);
            L_temp2 = L_shl(L_temp2, temp);
            ptr32 = &t_audio32[NUM_NONTRANS_END_FREQ_COEF];
            /* in L_temp/L_temp2, value in Q16+9+'temp' */
            temp = sub(15-9, temp); /* To go back to Q15, since 'en_band'/'en_noncoded' are in Q9 */
            FOR( j=0; j<WIDTH_BAND; j++ )
            {
                *ptr32 = L_shl(Mult_32_16(*ptr32, round_fx(L_temp)), temp);
                move32();
                ptr32++;
                L_temp = L_sub(L_temp, L_temp2);
            }
            ptr32 = &t_audio32[NUM_NONTRANS_END_FREQ_COEF + WIDTH_BAND];
            FOR( j=WIDTH_BAND; j<width_noncoded; j++ )
            {
                L_temp = Mult_32_16(*ptr32, en_noncoded);
                L_temp = L_shl(L_temp, 6); /* Because 'en_noncoded' is in Q9 */
                *ptr32++ = L_temp;
                move32();
            }

            /* Overlap region */
            ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF];
            FOR( i=0; i<NSV_OVERLAP*WIDTH_BAND; i++ )
            {
                L_temp = Mult_32_16(*ptr32, overlap_coefs_fx_0_9[i]);
                *ptr32++ = L_temp;
                move32();
            }

            /* Apply global gain */
            if( sub(nBits_total, NBITS_THRESH_BWE_HR) <= 0 )
            {
                gain_fx = mult_r(27853, gain_fx); /* 0.85 */
            }

            ptr32 = &t_audio32[NUM_NONTRANS_START_FREQ_COEF];
            temp = sub(15, exp1); /* From Exponent of 'gain_fx' in Q'exp1' to Q15 */
            temp2 = add(WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR, width_noncoded);
            FOR( i=0; i<temp2; i++ )
            {
                L_temp = Mult_32_16(*ptr32, gain_fx);
                L_temp = L_shl(L_temp, temp);
                *ptr32++ = L_temp;
                move32();
            }

            /* Save transform coefficients for the next frame (needed in case of frame erasures) */
            temp = NUM_NONTRANS_START_FREQ_COEF;
            move16(); /* not necessary but improves readability and allows a larger common code path */
            IF( output_frame == L_FRAME32k )
            {
                pos = L_FRAME32k - NUM_NONTRANS_START_FREQ_COEF;
            }
            ELSE  /* output_frame == L_FRAME48k */
            {
                pos = 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_START_FREQ_COEF;
            }
            temp4 = Find_Max_Norm32(t_audio32 + temp, pos);
            Copy_Scale_sig32_16(t_audio32 + temp, st_fx->t_audio_prev_fx, pos, temp4);
            st_fx->t_audio_prev_fx_exp[0] = add(Q_32_BITS, temp4);
            move16();
            /* attenuate HFs in case of band-width switching */
            IF( st_fx->bws_cnt1_fx > 0 )
            {
                temp = i_mult(st_fx->bws_cnt1_fx, 1638);  /*Q15*/
                FOR( i=0; i<L_FRAME32k; i++ )
                {
                    t_audio[NUM_NONTRANS_START_FREQ_COEF+i] = mult_r(t_audio[NUM_NONTRANS_START_FREQ_COEF+i], temp);
                    move16();
                }
            }
        }
    }

    IF (gain_fx < 0)
    {
        st_fx->prev_ener_shb_fx = extract_h(L_shr(L_mult0(32767, env), 7));
    }
    ELSE
    {
        st_fx->prev_ener_shb_fx = extract_h(L_shr(L_mult0(gain_fx, env), 7));
    }
    FOR(i=0; i<SWB_FENV; i++)
    {
        st_fx->prev_SWB_fenv_fx[i] = st_fx->prev_ener_shb_fx;
        move16();
    }

    /*---------------------------------------------------------------------*
     * iOLA and iMDCT
     *---------------------------------------------------------------------*/

    t_audio_exp = Q_32_BITS;

    Inverse_Transform( t_audio32, &t_audio_exp, t_audio32_tmp, is_transient, output_frame, output_frame );
    window_ola_fx( t_audio32_tmp, hb_synth_fx, &t_audio_exp, st_fx->old_out_hr_fx, &st_fx->old_out_hr_exp_fx, output_frame,
                   ALDO_WINDOW, ALDO_WINDOW, 0,0,0);
    hb_synth_fx_exp = t_audio_exp;
    move16();

    /*---------------------------------------------------------------------*
     * final adjustments
     *---------------------------------------------------------------------*/

    IF( !st_fx->bfi_fx )
    {
        IsTransient = 0;
        move16();
        L_EnergyLT = L_add(st_fx->L_mem_EnergyLT_fx, 0);
        temp4 = st_fx->mem_EnergyLT_fx_exp;
        move16();
        pos = 0;
        move16();

        ptr16 = hb_synth_fx;
        move16();

        len = shr(output_frame, 2); /* Divide Frame Len by 4, all (160, 320, 640, 960) are divisible by 4 */
        L_ener_all = L_deposit_l(0);
        ener_all_exp = 40;
        move16(); /* set to a high exponent */

        FOR( j=0; j<4; j++ )
        {
            L_Energy = Calc_Energy_Autoscaled(ptr16, hb_synth_fx_exp, len, &temp2);
            ptr16 += len;
            move16();
            /* Normalize Energy */
            temp = norm_l(L_Energy);
            L_Energy = L_shl(L_Energy, temp);
            /* Update Exponent of 'L_Energy' */
            temp2 = add(temp2, temp);

            /* Normalize Long Term Energy */
            temp = norm_l(L_EnergyLT);
            L_EnergyLT = L_shl(L_EnergyLT, temp);
            /* Calculate Exponent of Long Term Energy */
            temp = add(temp, temp4);

            /* Divide by 12.5 */
            L_temp = Mult_32_16(L_Energy, 20972); /* 20972 = 1/12.5*32768*8 (*8 to boost precision) */
            temp3 = norm_l(L_temp);
            L_temp = L_shl(L_temp, temp3);
            temp3 = add(add(temp2, temp3), 3);
            /* Energies are Strictly Positive Values and Normalized
               (compare exponent and value only if exponent is same) */
            /* Replaces: 'if (Energy_flt > 12.5f * EnergyLT_flt )' */
            temp3 = sub(temp3, temp);
            test();
            test();
            IF (temp3 < 0 || (L_sub(L_temp, L_EnergyLT) > 0 && temp3 == 0))
            {
                IsTransient = 1;
                move16();
                pos = j;
                move16();
                L_ener_saved = L_add(L_ener_all, 0);
                ener_saved_exp = ener_all_exp;
                move16();
            }

            L_ener_all = Add_flt32_flt32(L_Energy, temp2, L_ener_all, ener_all_exp, &ener_all_exp);
            /* 0.25f*Energy_flt */
            temp2 = add(temp2, 2);
            /* 0.75f*EnergyLT_flt */
            L_EnergyLT = L_sub(L_EnergyLT, L_shr(L_EnergyLT, 2));
            /* Exponent of 'L_EnergyLT' is 'temp' */
            /* Exponent of 'L_Energy' is 'temp2' */
            /* EnergyLT = 0.75f*EnergyLT + 0.25f*Energy */
            L_EnergyLT = Add_flt32_flt32(L_Energy, temp2, L_EnergyLT, temp, &temp4);
        }

        test();
        test();
        test();
        IF( IsTransient != 0 && pos > 0 && L_sub(L_tilt_wb, 16777216L*3/*tilt_wb in Q24*/) < 0 && sub(pitch, 500*16/*Q4*/) > 0 )
        {
            Nsv = i_mult2(pos, shr(output_frame, 2));

            gain_fx = 16384; /* sqrt(1.0) in Q14 */
            /* pos is 1,2 or 3 */
            temp3 = sub(pos, 2);
            if (temp3 == 0)
            {
                gain_fx = 23170;
                move16(); /* sqrt(2.0) in Q14 */
            }

            if (temp3 > 0)
            {
                gain_fx = 28378;
                move16(); /* sqrt(3.0) in Q14 */
            }
            exp1 = 14;
            move16();

            IF( sub(st_fx->last_extl_fx, st_fx->extl_fx) == 0 )
            {
                L_temp = Div_flt32_flt32( L_ener_saved, ener_saved_exp, st_fx->L_mem_EnergyLT_fx, st_fx->mem_EnergyLT_fx_exp, &temp2 );
                temp3 = sub(temp2, 1);
                L_temp2 = Isqrt_lc(L_temp, &temp3);
                temp3 = sub(sub(30+31-15+1, temp2), temp3);

                L_temp = Mult_32_16(L_temp2, gain_fx);
                temp = norm_l(L_temp);
                L_temp2 = L_shl(L_temp, temp);
                gain_fx = round_fx(L_temp2);
                exp1 = sub(add(temp3, temp), sub(31, exp1)); /* gain_fx is in Q14 */
            }

            L_temp = L_mult0(26214, gain_fx);
            /* +16: Because 26214 is 0.2 in Q16
             * -16: Because of round_fx
             * -15: To get exponent with ref to Q15
             *  +1: Because of L_mult'0'
             * and the normalization
             */
            exp2 = add(exp1, +16-16-15+1);
            temp = norm_l(L_temp);
            L_temp = L_shl(L_temp, temp);
            exp2 = add(exp2, temp);
            temp = round_fx(L_temp); /* Gain is in Q15+x */
            FOR( i=0; i<Nsv; i++ )
            {
                L_temp2 = L_mult(temp, hb_synth_fx[i]);
                L_temp2 = L_shr(L_temp2, exp2);
                hb_synth_fx[i] = round_fx(L_temp2);
            }

            len = shr(output_frame, 3); /* all frame length are divisible by 8 */
            /* This will yield
               3 for 960, 2 for 640, 1 for 320 & 0 for 160
               This is used as an index into a 1/output_frame table of 4 entries */
            temp = mult_r(output_frame, 102);

            ptr16 = &hb_synth_fx[Nsv];
            /* gain_flt * gain * 1/frame_len */
            L_temp2 = Mult_32_16(L_temp, swb_hr_inv_frm_len[temp]);
            /* Gain in Q30 */
            L_temp = L_shr(L_temp, add(exp2, 31-30));
            /* Adjust Exponent */
            exp2 = add(30-19-3-3, exp2); /* Exp Req to go to Q30 (-19: because 'swb_hr_inv_frm_len' is in Q19) */
            /* Put to Q30 */
            L_temp2 = L_shr(L_temp2, exp2);
            /* 1/frame_len in Q30 */
            L_temp2 = L_msu(L_temp2, swb_hr_inv_frm_len[temp], 32768L>>(19-15+1)); /* 19-15+1 to Bring to Q30 */
            FOR ( i = 0; i < len; i++ )
            {
                /* hb_synth[i+Nsv] *= (gain_flt - i*8.0f*(1.0f/output_frame*gain_flt - 1.0f/output_frame)) */
                *ptr16 = round_fx(L_shl(Mult_32_16(L_temp, *ptr16), 1));
                ptr16++;
                L_temp = L_sub(L_temp, L_temp2);
            }
        }

        st_fx->L_mem_EnergyLT_fx = L_EnergyLT;
        move32();
        st_fx->mem_EnergyLT_fx_exp = temp4;
        move16();
        st_fx->old_is_transient_hr_bwe_fx = is_transient;
        move16();
    }

    /* post-processing in case of TD/FD switching */
    test();
    IF( sub(st_fx->last_core_fx, HQ_CORE) == 0 || sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0 )
    {
        IF( L_sub(L_tilt_wb, 16777216L*3/*tilt_wb in Q24*/) < 0 )
        {
            temp = TD_Postprocess( hb_synth_fx, hb_synth_fx_exp, output_frame, st_fx->last_extl_fx );

            FOR( i=0; i<output_frame; i++ )
            {
                st_fx->old_out_hr_fx[i] = mult_r(st_fx->old_out_hr_fx[i], temp);
                move16();
            }
        }
    }

    return hb_synth_fx_exp;
}
