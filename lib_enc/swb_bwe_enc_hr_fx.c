/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"            /* Compilation switches                   */
#include "cnst_fx.h"            /* Common constants                       */
#include "rom_enc_fx.h"         /* Encoder static table prototypes        */
#include "rom_com_fx.h"         /* Static table prototypes                */
#include "prot_fx.h"            /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

/* Values that Allow to Use Left/Right Shifting instead of Division &
   'and' (&) instead of 'modulo' (%) which is also a costly division. */
#define NUM_TIME_SW_BLKS_SHIFT 2
#define NUM_TIME_SW_BLKS_MASK  (NUM_TIME_SWITCHING_BLOCKS-1)
#define WIDTH_BAND_SHIFT 3

/* Values for Max Scaling of Different Sub Function */
#define DETECT_TRANSIENT_MAX_Q (11+2)
#define               MAX_Q_NEW_INPUT 8
#define NON_TRANSIENT_RESCALE_Q_GUARD 4
#define     TRANSIENT_RESCALE_Q_GUARD 0
#define      SPECTRUM_RESCALE_Q_GUARD 1
#define               MAX_Q_UPSCALING 6
#define              MAX_AVQ_COD_Q_IN 14

/*-------------------------------------------------------------------*
 * en_band_quant_fx()
 *
 * Quantize the band envelop
 *-------------------------------------------------------------------*/

static Word16 en_band_quant_fx(/* o  : quantization index              */
    Word16 *en_band,     /* i/o: (un)quantized envelope value    */
    const Word16 *env_code,    /* i  : envelope codebook               */
    const Word16 N             /* i  : codebook dimension              */
)
{
    Word16 i, ind, tmp16;
    Word32 L_err, L_maxerr;

    L_maxerr = 2147483647L;
    move32();
    ind = 0;
    move16();

    FOR( i = 0; i < N; i++ )
    {
        /* This is More Efficient */
        tmp16 = sub(en_band[0], env_code[i*2]);
        L_err = L_mult0(tmp16, tmp16);
        tmp16 = sub(en_band[1], env_code[i*2+1]);
        L_err = L_mac0(L_err, tmp16, tmp16);
        /* L_err = 0; move32();
        FOR (j = 0; j < 2; j++)
        {
            tmp16 = sub(en_band[j], env_code[i*2+j]);
            L_err = L_mac0(L_err, tmp16, tmp16);
        } */
        if (L_sub(L_err, L_maxerr) < 0)
        {
            ind = i;
            move16();
        }
        L_maxerr = L_min(L_maxerr, L_err);
    }

    en_band[0] = env_code[2*ind];
    move16();
    en_band[1] = env_code[2*ind+1];
    move16();

    return( ind );
}

/*-------------------------------------------------------------------*
 * swb_bwe_enc_hr_fx()
 *
 * HR SWB BWE encoder
 *-------------------------------------------------------------------*/
void swb_bwe_enc_hr_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure     */
    Word16 *new_input_fx,    /* i  : input signal                */
    Word16 new_input_fx_exp, /* i  : Exponent of input signal    */
    const Word16 input_frame,      /* i  : frame length                */
    const Word16 coder_type,       /* i  : coding type                 */
    const Word16 unbits            /* i  : number of core unused bits  */
)
{
    Word16 i, j, k, nBits, nBits_total, nBits_block, Nsv, Nsv2, width_noncoded;
    Word16 is_transient, pos;
    Word16 x_norm_fx[NSV_MAX*(WIDTH_BAND+1)], x_norm1_fx[NSV_MAX*(WIDTH_BAND+1)];
    Word32 t_audio32[L_FRAME48k];
    Word16 *t_audio_fx, t_audio_fx_exp;
    Word16 *t_audio_tmp_fx; /* same exponent as 't_audio_fx' */
    Word16 en_band_fx[N_BANDS_BWE_HR]; /* in Q9 */
    Word16 gain1_fx, exp1, gain2_fx, exp2;
    Word32 L_gain_fx;
    Word16 ind1, ind2;
    Word16 nq[NSV_MAX], nq2[NSV_MAX];
    Word32 L_tmp, L_temp;
    Word16 temp, temp2;
    Word16 *ptr16;
    Word16 min_env_fx;
    Word32 L_en_noncoded_fx; /* in Q16 */
    Word16 en_noncoded_fx_exp;
    Word16 scl;
#if (N_BANDS_BWE_HR*WIDTH_NONTRANS_FREQ_COEF) > L_FRAME48k
    Word32 L_t_audio_tmp_fx[N_BANDS_BWE_HR*WIDTH_NONTRANS_FREQ_COEF];
#else
    Word32 L_t_audio_tmp_fx[L_FRAME48k];
#endif
    Word32 *ptr32;

    /*---------------------------------------------------------------------*
     * initializations
     *---------------------------------------------------------------------*/

    /* Use 32 Bits Buffer to Store Two 16 Bits Vectors in Order to Save Stack Space */
    t_audio_fx =     (Word16 *)&t_audio32[0];
    move16();
    t_audio_tmp_fx = (Word16 *)&t_audio32[L_FRAME48k/2];
    move16();

    ind2 = 0;
    move16();/* only to suppress warnings */
    Nsv2 = 0;
    move16();/* only to suppress warnings */
    L_en_noncoded_fx = 0;
    move16();/* only to suppress warnings */
    en_noncoded_fx_exp = 0;
    move16();/* only to suppress warnings */
    gain2_fx = 0;
    move16();

    /* reset memories in case that last frame was a different technology */
    test();
    IF( sub(st_fx->last_core_fx, HQ_CORE) == 0 || sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0 )
    {
        set16_fx( st_fx->L_old_wtda_hr_fx, 0, L_FRAME48k );
        st_fx->old_wtda_hr_fx_exp = 0;
        move16();
    }

    /* calculate SWB BWE bit-budget (extension layer bit-rate + AVQ unused bits from the core layer) */
    /* nBits = st->extl_brate/50 + unbits */
    nBits = add(320,unbits);    /* st->extl_brate_fx is always 16kbps */
    nBits_total = nBits;
    move16();

    /*---------------------------------------------------------------------*
     * detect transient frames
     *---------------------------------------------------------------------*/
    /* Calc Room for Energy */
    temp = norm_l(st_fx->EnergyLT_fx);
    /* Calc Max Exponent for 'new_input_fx' */
    temp = shr(temp, 1);
    temp = add(temp, st_fx->EnergyLT_fx_exp);
    temp = sub(temp, new_input_fx_exp);
    /* Do not Upscale */
    temp = min(0, temp);
    /* Limit Input to 'Q_DETECT_TRANSIENT_MAX_Q' (to avoid overflow in 'detect_transient_fx' for Energy Loop)*/
    exp1 = Find_Max_Norm16(new_input_fx, input_frame);
    exp1 = sub(exp1, 15-DETECT_TRANSIENT_MAX_Q);
    /* Downscale at Least by 'exp1' */
    temp = min(temp, exp1);
    /* Set Exponent of Input */
    exp1 = add(new_input_fx_exp, temp);

    Copy_Scale_sig(new_input_fx, t_audio_fx, input_frame, temp);
    /* Bring Energy in 2*Q'exp1' */
    st_fx->EnergyLT_fx = L_shl(st_fx->EnergyLT_fx, shl(sub(exp1, st_fx->EnergyLT_fx_exp), 1));

    is_transient = detect_transient_fx( t_audio_fx, input_frame, coder_type, exp1, st_fx );
    st_fx->EnergyLT_fx_exp = exp1;
    move16();

    push_indice_fx( st_fx, IND_HR_IS_TRANSIENT, is_transient, 1 );

    /*---------------------------------------------------------------------*
     * OLA and MDCT
     *---------------------------------------------------------------------*/

    /* To Bring Back to Q15 + 'new_input_fx_exp' */
    /* Save Exponent of Memory (relative to Q15) */
    st_fx->old_wtda_hr_fx_exp = new_input_fx_exp;
    move16();
    new_input_fx_exp=0;
    wtda_fx(new_input_fx, &new_input_fx_exp, L_t_audio_tmp_fx, st_fx->L_old_wtda_hr_fx,
            &st_fx->old_wtda_hr_fx_exp, ALDO_WINDOW, ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
            input_frame );

    direct_transform_fx(L_t_audio_tmp_fx, t_audio32, is_transient, input_frame, &new_input_fx_exp);

    /* Convert to 16 Bits (Calc Shift Required to Stay within MAX_Q_NEW_INPUT) */
    scl = sub(16+MAX_Q_NEW_INPUT, new_input_fx_exp);
    /* Possible to Upscale? */
    IF (scl > 0)
    {
        /* Yes */
        /* Calc Room to Upscale */
        t_audio_fx_exp = Find_Max_Norm32(t_audio32, input_frame);
        /* Stay within MAX_Q_NEW_INPUT */
        scl = s_min(t_audio_fx_exp, scl);
    }
    Copy_Scale_sig32_16(t_audio32, t_audio_fx, input_frame, scl);
    t_audio_fx_exp = add(sub(new_input_fx_exp, 16), scl);

    IF( is_transient )
    {
        nBits = -1;
        move16(); /* is_transient flag */
        /* 'nBits_block = nBits_total / NUM_TIME_SWITCHING_BLOCKS' */
        nBits_block = shr(nBits_total, NUM_TIME_SW_BLKS_SHIFT);
        /* 'nBits += nBits_total % NUM_TIME_SWITCHING_BLOCKS' */
        nBits = add(nBits, s_and(nBits_total, NUM_TIME_SW_BLKS_MASK));

        /* set width of noncoded (blind estimated) spectrum */
        IF( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) == 0)
        {
            width_noncoded = L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF;
            move16();
        }
        ELSE /* st->extl == FB_BWE_HIGHRATE */
        {
            width_noncoded = (2*END_FREQ_BWE_FULL_FB/50)/NUM_TIME_SWITCHING_BLOCKS - NUM_TRANS_END_FREQ_COEF;
            move16();
        }

        /* Find Max Scaling on Remaining Frequencies (after frequencies of non interest are zeroed) */
        temp = shr(input_frame, NUM_TIME_SW_BLKS_SHIFT);
        scl = 99;
        move16();
        ptr16 = &t_audio_fx[NUM_TRANS_START_FREQ_COEF];
        move16();
        FOR( k = 0; k < input_frame; k+=temp )
        {
            /* from t_audio_fx[k..NUM_TRANS_START_FREQ_COEF+k-1] will be zeroed out */
            /* AND */
            /* from t_audio_fx[k+L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS..k+switching_block_length-1] will be zeroed out */
            /* Find Max Scaling on Remaining Frequencies */
            scl = s_min(Find_Max_Norm16(ptr16+k, L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS-NUM_TRANS_START_FREQ_COEF), scl);
            IF (scl == 0)
            {
                BREAK;
            }
        }
        /* Need to Keep at Least some Guard Bits */
        scl = s_max(0, sub(scl, TRANSIENT_RESCALE_Q_GUARD));
        /* Reduce if 't_audio_fx_exp' is Q14 or more (AVQ Cod overflow with more than Q14 Input) */
        scl = sub(scl, s_max(0, sub(add(scl, t_audio_fx_exp), MAX_AVQ_COD_Q_IN)));
        /* Update Exponent of 't_audio_fx' */
        /* Here tne Signal is not Upscaled yet and some adjustement to 't_audio_fx_exp' will be used
           until the signal is indeed upscaled by 'scl'. That Occurs at the 'normalization with global gain'. */
        t_audio_fx_exp = add(t_audio_fx_exp, scl);

        /*---------------------------------------------------------------------*
         * transient frames: processing in blocks (subframes)
         *---------------------------------------------------------------------*/

        FOR( k = 0; k < NUM_TIME_SWITCHING_BLOCKS; k++ )
        {
            nBits = add(nBits, nBits_block);

            temp = i_mult2(k, shr(input_frame, NUM_TIME_SW_BLKS_SHIFT));

            /* Calculate Original Exponent (because the part of the signal that is used
               to Calculate the Energy is not yet Scaled) */
            j = sub(t_audio_fx_exp, scl);

            /* compute energy of noncoded (14.4-20kHz) spectrum */
            IF( sub(st_fx->extl_fx, FB_BWE_HIGHRATE) == 0)
            {
                L_tmp = Calc_Energy_Autoscaled(t_audio_fx + add(temp, NUM_TRANS_END_FREQ_COEF), j, width_noncoded, &temp2);
                L_en_noncoded_fx = Sqrt_Ratio32(L_tmp, temp2, L_deposit_l(width_noncoded), 0, &en_noncoded_fx_exp);
                en_noncoded_fx_exp = sub(31, en_noncoded_fx_exp);
            }

            /* keep only frequencies in interest */
            set16_fx( t_audio_fx + temp, 0, NUM_TRANS_START_FREQ_COEF );
            set16_fx( t_audio_fx + add(temp, L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS), 0, shr(sub(input_frame, L_FRAME32k), NUM_TIME_SW_BLKS_SHIFT) );

            /*---------------------------------------------------------------------*
             * global gain coding
             *---------------------------------------------------------------------*/

            /* compute and quantize global energy */
            /* Original float Code: 'gain = (float)sqrt( gain ) / (WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR)' */
            /* Put Divisor to Square to Move it Inside the Sqrt */
            /* So we can do 'sqrt( gain (WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR)^2)' */
            L_temp = L_mult0(WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR, WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR);
            L_tmp = Calc_Energy_Autoscaled(t_audio_fx + add(temp, NUM_TRANS_START_FREQ_COEF), j, WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR, &temp2);
            L_tmp = Sqrt_Ratio32(L_tmp, temp2, L_temp, /*L_temp is in Q0*/0, &exp2);
            /* Put in Q16 */
            L_gain_fx = L_shr(L_tmp, sub(31-16, exp2)); /* 31: 'L_tmp' is already in Q31 */

            ind1 = gain_quant_fx( &L_gain_fx, &gain1_fx, LG10_MIN_GLOB_GAIN_BWE_HR_Q14, LG10_MAX_GLOB_GAIN_BWE_HR_Q13, NBITS_GLOB_GAIN_BWE_HR, &exp1);

            push_indice_fx( st_fx, IND_HR_GAIN, ind1, NBITS_GLOB_GAIN_BWE_HR );
            nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR);

            /* normalization with global gain */
            ptr16 = &t_audio_fx[add(NUM_TRANS_START_FREQ_COEF, temp)];
            move16();
            temp2 = negate(exp1);
            gain2_fx = Invert16(gain1_fx, &temp2);

            /* Also Upscale by 'scl' */
            temp2 = sub(temp2, scl);
            FOR( i=0; i<WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR; i++ )
            {
                L_temp = L_mult(*ptr16, gain2_fx);
                L_temp = L_shr(L_temp, temp2);
                *ptr16++ = round_fx(L_temp);

            }
            FOR( ; i<L_FRAME32k/NUM_TIME_SWITCHING_BLOCKS-NUM_TRANS_START_FREQ_COEF; i++ )
            {
                temp2 = shl(*ptr16, scl);
                *ptr16++ = temp2;
                move16();
            }

            /*---------------------------------------------------------------------*
             * envelope coding
             *---------------------------------------------------------------------*/

            /* compute energy per band */
            ptr16 = &t_audio_fx[add(temp, NUM_TRANS_START_FREQ_COEF)];
            move16();
            FOR( i=0; i<N_BANDS_TRANS_BWE_HR; i++ )
            {
                L_temp = Calc_Energy_Autoscaled(ptr16, t_audio_fx_exp, WIDTH_TRANS_FREQ_COEF, &temp2);
                ptr16 += WIDTH_TRANS_FREQ_COEF;
                L_temp = Sqrt_Ratio32(L_temp, temp2, WIDTH_TRANS_FREQ_COEF, /*WIDTH_TRANS_FREQ_COEF is in Q0*/0, &temp2);
                en_band_fx[i] = round_fx(L_shr(L_temp, sub(15-9, temp2)));
            }

            /* Q energy per band */
            IF( k == 0 )
            {
                ind1 = en_band_quant_fx( en_band_fx, swb_hr_env_code3_fx, NUM_ENVLOPE_CODE_HR_TR );

                push_indice_fx( st_fx, IND_HR_ENVELOPE, ind1, NBITS_ENVELOPE_BWE_HR_TR );
                nBits = sub(nBits, NBITS_ENVELOPE_BWE_HR_TR);
                ind2 = ind1;
                move16();
            }
            ELSE
            {
                IF( sub(ind2, NUM_ENVLOPE_CODE_HR_TR2 ) < 0)
                {
                    ind1 = en_band_quant_fx( en_band_fx, swb_hr_env_code3_fx, NUM_ENVLOPE_CODE_HR_TR2 );
                    move16();
                }
                ELSE
                {
                    ind1 = en_band_quant_fx( en_band_fx, swb_hr_env_code3_fx + (NUM_ENVLOPE_CODE_HR_TR2*2), NUM_ENVLOPE_CODE_HR_TR2 );
                    move16();
                }

                push_indice_fx( st_fx, IND_HR_ENVELOPE, ind1, NBITS_ENVELOPE_BWE_HR_TR - 1 );
                nBits = sub(nBits, NBITS_ENVELOPE_BWE_HR_TR - 1);
            }

            /* normalize spectrum per bands */
            ptr16 = &t_audio_fx[add(temp, NUM_TRANS_START_FREQ_COEF)];
            move16();
            FOR( i = 0; i < N_BANDS_TRANS_BWE_HR; i++ )
            {
                temp2 = 9;
                move16();
                gain2_fx = Invert16(en_band_fx[i], &temp2);

                FOR( j = 0; j < WIDTH_TRANS_FREQ_COEF; j++ )
                {
                    L_temp = L_mult(*ptr16, gain2_fx);
                    L_temp = L_shr(L_temp, temp2);
                    *ptr16++ = round_fx(L_temp);
                }
            }

            /*---------------------------------------------------------------------*
             * estimate energy of noncoded spectrum (14.4-20kHz)
             *---------------------------------------------------------------------*/

            IF( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) != 0)
            {
                /* st->extl == FB_BWE_HIGHRATE */
                /* 'en_noncoded /= (gain * en_band[N_BANDS_TRANS_BWE_HR-1])' */
                /* Normalize 'L_en_noncoded_fx' */
                j = norm_l(L_en_noncoded_fx);
                L_en_noncoded_fx = L_shl(L_en_noncoded_fx, j);
                en_noncoded_fx_exp = add(en_noncoded_fx_exp, j);
                /* Calc Divisor */
                L_temp = L_mult0(gain1_fx, en_band_fx[N_BANDS_TRANS_BWE_HR-1]);
                /* Normalize Divisor */
                temp2 = norm_l(L_temp);
                L_temp = L_shl(L_temp, temp2);
                temp2 = sub(add(9, temp2), exp1); /* Q9 for 'en_band_fx' */
                j = Invert16(round_fx(L_temp), &temp2);
                L_temp = Mult_32_16(L_en_noncoded_fx, j);
                temp2 = add(temp2, en_noncoded_fx_exp);
                /* Put in Q16 */
                L_temp = L_shr(L_temp, temp2);

                IF( L_msu0(L_temp, BWE_HR_TRANS_EN_LIMIT1_FX_Q16, 1) < 0)
                {
                    ind1 = 1;
                    move16();
                    L_en_noncoded_fx = L_mult0(en_band_fx[N_BANDS_TRANS_BWE_HR-1], BWE_HR_TRANS_EN_LIMIT1_FX_Q16);
                }
                ELSE IF( L_msu0(L_temp, BWE_HR_TRANS_EN_LIMIT2_FX_Q16, 1) < 0)
                {
                    ind1 = 2;
                    move16();
                    L_en_noncoded_fx = L_mult0(en_band_fx[N_BANDS_TRANS_BWE_HR-1], BWE_HR_TRANS_EN_LIMIT2_FX_Q16);
                }
                ELSE IF( L_msu0(L_temp, BWE_HR_TRANS_EN_LIMIT3_FX_Q16, 1) < 0)
                {
                    ind1 = 3;
                    move16();
                    L_en_noncoded_fx = L_mult0(en_band_fx[N_BANDS_TRANS_BWE_HR-1], BWE_HR_TRANS_EN_LIMIT3_FX_Q16);
                }
                ELSE
                {
                    ind1 = 0;
                    move16();
                    L_en_noncoded_fx = L_deposit_h(en_band_fx[N_BANDS_TRANS_BWE_HR-1]); /* to Put in Q16+9 */
                }

                push_indice_fx( st_fx, IND_HR_HF_GAIN, ind1, NBITS_HF_GAIN_BWE_HR );
                nBits = sub(nBits, NBITS_HF_GAIN_BWE_HR);
            }
            ELSE
            {
                L_en_noncoded_fx = L_deposit_h(en_band_fx[N_BANDS_TRANS_BWE_HR-1]);
            }
            en_noncoded_fx_exp = 9+16;
            move16(); /* 9 for 'en_band_fx', 16 for 'BWE_HR_TRANS_EN_LIMIT...' */

            /*---------------------------------------------------------------------*
             * AVQ coding (quantize normalized spectrum)
             *---------------------------------------------------------------------*/

            Nsv = (NUM_TRANS_END_FREQ_COEF - NUM_TRANS_START_FREQ_COEF) / WIDTH_BAND;
            move16();
            AVQ_cod_fx(t_audio_fx + add(temp, NUM_TRANS_START_FREQ_COEF), x_norm_fx, nBits, Nsv, t_audio_fx_exp);
            AVQ_encmux_fx(st_fx, st_fx->extl_fx, x_norm_fx, &nBits, Nsv, nq);

        }
    }
    ELSE /* !is_transient */
    {
        /* subtract one bit for is_transient flag */
        nBits = sub(nBits, 1);

        /*---------------------------------------------------------------------*
         * processing of normal (non-transient) frames
         *---------------------------------------------------------------------*/

        /* set width of noncoded (blind estimated) spectrum */
        IF( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) == 0)
        {
            width_noncoded = L_FRAME32k - NUM_NONTRANS_END_FREQ_COEF;
            move16();
        }
        ELSE  /* st->extl == FB_BWE_HIGHRATE */
        {
            width_noncoded = 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_END_FREQ_COEF;
            move16();
        }

        /* compute energy of noncoded (14.4-20kHz) spectrum */
        IF( sub(st_fx->extl_fx, FB_BWE_HIGHRATE) == 0)
        {
            L_tmp = Calc_Energy_Autoscaled(t_audio_fx + NUM_NONTRANS_END_FREQ_COEF, t_audio_fx_exp, width_noncoded, &temp2);
            L_en_noncoded_fx = Sqrt_Ratio32(L_tmp, temp2, L_deposit_l(width_noncoded), 0, &en_noncoded_fx_exp);
            en_noncoded_fx_exp = sub(31, en_noncoded_fx_exp);
        }

        /* keep only frequencies in interest */
        set16_fx( t_audio_fx, 0, NUM_NONTRANS_START_FREQ_COEF );
        set16_fx( t_audio_fx + NUM_NONTRANS_END_FREQ_COEF, 0, sub(input_frame, NUM_NONTRANS_END_FREQ_COEF) );

        /*---------------------------------------------------------------------*
         * global gain coding
         *---------------------------------------------------------------------*/

        /* compute and quantize global gain */
        L_tmp = Calc_Energy_Autoscaled(t_audio_fx + NUM_NONTRANS_START_FREQ_COEF, t_audio_fx_exp, WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR, &temp2);
        /* Original float Code: 'gain = (float)sqrt( gain ) / (WIDTH_TRANS_FREQ_COEF*N_BANDS_TRANS_BWE_HR)' */
        /* Put Divisor to Square to Move it Inside the Sqrt */
        /* So we can do 'sqrt( gain (WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR)^2)' */
        L_temp = L_mult0(WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR, WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR);
        L_tmp = Sqrt_Ratio32(L_tmp, temp2, L_temp, /*L_temp is in Q0*/0, &exp2);
        /* Put in Q16 */
        L_gain_fx = L_shr(L_tmp, sub(31-16, exp2)); /* 31: 'L_tmp' is already in Q31 */
        ind1 = gain_quant_fx( &L_gain_fx, &gain1_fx, LG10_MIN_GLOB_GAIN_BWE_HR_Q14, LG10_MAX_GLOB_GAIN_BWE_HR_Q13, NBITS_GLOB_GAIN_BWE_HR, &exp1);

        push_indice_fx( st_fx, IND_HR_GAIN, ind1, NBITS_GLOB_GAIN_BWE_HR );
        nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR);

        /* normalization with global gain */
        ptr16 = &t_audio_fx[NUM_NONTRANS_START_FREQ_COEF];
        move16();
        /* Find Max Scaling on Remaining Frequencies */
        temp2 = Find_Max_Norm16(ptr16, NUM_NONTRANS_END_FREQ_COEF-NUM_NONTRANS_START_FREQ_COEF);
        temp2 = s_max(0, sub(temp2, NON_TRANSIENT_RESCALE_Q_GUARD));
        temp2 = s_min(temp2, MAX_Q_UPSCALING);
        t_audio_fx_exp = add(t_audio_fx_exp, temp2);
        temp2 = sub(temp2, exp1);
        temp = Invert16(gain1_fx, &temp2);

        FOR( i=0; i<WIDTH_NONTRANS_FREQ_COEF*N_BANDS_BWE_HR; i++ )
        {
            L_temp = L_mult(*ptr16, temp);
            L_temp = L_shr(L_temp, temp2);
            *ptr16++ = round_fx(L_temp);
        }

        /*---------------------------------------------------------------------*
         * envelope coding
         *---------------------------------------------------------------------*/

        /* compute energy per band */
        ptr16 = &t_audio_fx[NUM_NONTRANS_START_FREQ_COEF];
        move16();
        FOR( i=0; i<N_BANDS_BWE_HR; i++ )
        {
            L_temp = Calc_Energy_Autoscaled(ptr16, t_audio_fx_exp, WIDTH_NONTRANS_FREQ_COEF, &temp2);
            ptr16 += WIDTH_NONTRANS_FREQ_COEF;
            L_temp = Sqrt_Ratio32(L_temp, temp2, WIDTH_NONTRANS_FREQ_COEF, /*WIDTH_TRANS_FREQ_COEF is in Q0*/0, &temp2);
            en_band_fx[i] = round_fx(L_shr(L_temp, sub(15-9, temp2))); /* Put in Q9 */
        }

        /* Q energy per band */
        ind1 = en_band_quant_fx( en_band_fx, swb_hr_env_code1_fx, NUM_ENVLOPE_CODE_HR1 );
        ind2 = en_band_quant_fx( en_band_fx + 2, swb_hr_env_code2_fx, NUM_ENVLOPE_CODE_HR2 );

        push_indice_fx( st_fx, IND_HR_ENVELOPE, ind1, NBITS_ENVELOPE_BWE_HR1 );
        push_indice_fx( st_fx, IND_HR_ENVELOPE, ind2, NBITS_ENVELOPE_BWE_HR2 );

        nBits = sub(nBits, NBITS_ENVELOPE_BWE_HR1 + NBITS_ENVELOPE_BWE_HR2);

        /* normalize spectrum per bands */
        ptr32 = &L_t_audio_tmp_fx[0];
        ptr16 = &t_audio_fx[NUM_NONTRANS_START_FREQ_COEF];
        move16();
        FOR( i=0; i<N_BANDS_BWE_HR; i++ )
        {
            temp2 = 9;
            move16(); /* 'en_band_fx' is in Q9 */
            temp = Invert16(en_band_fx[i], &temp2);
            FOR( j=0; j<WIDTH_NONTRANS_FREQ_COEF; j++ )
            {
                L_temp = L_mult(*ptr16++, temp);
                L_temp = L_shr(L_temp, temp2);
                *ptr32++ = L_temp;
                move32();
            }
        }

        /* Find Max Scaling */
        L_temp = L_abs(L_t_audio_tmp_fx[0]);
        FOR (i = 1; i < N_BANDS_BWE_HR*WIDTH_NONTRANS_FREQ_COEF; i++)
        {
            L_temp = L_max(L_temp, L_t_audio_tmp_fx[i]);
        }
        temp = norm_l(L_temp);
        /* Keep some Guard and do not Downscale (and do not upscale too much) */
        temp = s_max(0, sub(temp, SPECTRUM_RESCALE_Q_GUARD));
        temp = s_min(temp, MAX_Q_UPSCALING);
        /* Reduce if 't_audio_fx_exp' is Q14 or more (AVQ Cod overflow with more than Q14 Input) */
        temp = sub(temp, s_max(0, sub(add(temp, t_audio_fx_exp), MAX_AVQ_COD_Q_IN)));
        /* Upscale and copy into 16 Bits 't_audio_fx' */
        Copy_Scale_sig32_16(L_t_audio_tmp_fx, &t_audio_fx[NUM_NONTRANS_START_FREQ_COEF],
                            N_BANDS_BWE_HR*WIDTH_NONTRANS_FREQ_COEF, temp);
        /* Adjust exponent of 't_audio_fx_exp' */
        t_audio_fx_exp = add(t_audio_fx_exp, temp);

        /*---------------------------------------------------------------------*
         * choose sub-bands to be quantized
         *---------------------------------------------------------------------*/

        /* find the subband with the min envelope */
        pos = 0;
        FOR (i=1; i<N_BANDS_BWE_HR; i++)
        {
            if (sub(en_band_fx[i], en_band_fx[pos]) < 0)
            {
                pos = i;
                move16();
            }
        }
        min_env_fx = en_band_fx[pos];
        move16();

        /* decide the spectrum to be quantized */
        IF( sub(nBits_total, NBITS_THRESH_BWE_HR) > 0)
        {
            i = NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF;
            move16();
            Copy( t_audio_fx + NUM_NONTRANS_START_FREQ_COEF, t_audio_tmp_fx, NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF );
        }
        ELSE
        {
            /* reorder the spectrum */
            ind1 = add(shl(pos, 6), shl(shr(pos, 1), WIDTH_BAND_SHIFT));
            Copy( t_audio_fx + NUM_NONTRANS_START_FREQ_COEF, t_audio_tmp_fx, ind1 );

            ind2 = add(pos, 1);
            ind2 = add(shl(ind2, 6), shl(shr(ind2, 1), WIDTH_BAND_SHIFT));
            Copy( t_audio_fx + add(NUM_NONTRANS_START_FREQ_COEF, ind2), t_audio_tmp_fx + ind1, sub(NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF, ind2) );

            i = sub(add(ind1, NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF), ind2);
        }

        /*---------------------------------------------------------------------*
         * estimate energy of noncoded spectrum (14.4-20kHz)
         *---------------------------------------------------------------------*/
        IF( sub(st_fx->extl_fx, SWB_BWE_HIGHRATE) != 0)
        {
            /* st->extl == FB_BWE_HIGHRATE */
            /* 'en_noncoded /= (gain * min_env)' */
            /* Normalize 'L_en_noncoded_fx' */
            temp = norm_l(L_en_noncoded_fx);
            L_en_noncoded_fx = L_shl(L_en_noncoded_fx, temp);
            en_noncoded_fx_exp = add(en_noncoded_fx_exp, temp);
            /* Calc Divisor */
            L_temp = L_mult0(gain1_fx, min_env_fx);
            /* Normalize Divisor */
            temp2 = norm_l(L_temp);
            L_temp = L_shl(L_temp, temp2);
            temp2 = sub(add(9, temp2), exp1); /* Q9 for 'min_env_fx', 'exp1' for 'gain1' */
            j = Invert16(round_fx(L_temp), &temp2);
            L_temp = Mult_32_16(L_en_noncoded_fx, j);
            temp2 = add(temp2, en_noncoded_fx_exp);
            /* Put in Q16 */
            L_temp = L_shr(L_temp, temp2);
            IF( L_msu(L_temp, BWE_HR_NONTRANS_EN_LIMIT1_FX_Q15, 1) < 0)
            {
                ind1 = 1;
                move16();
                /* 'en_noncoded = 0.5f * min_env * BWE_HR_NONTRANS_EN_LIMIT1' */
                L_en_noncoded_fx = L_mult0(min_env_fx, BWE_HR_NONTRANS_EN_LIMIT1_FX_Q15/2);
            }
            ELSE IF( L_msu(L_temp, BWE_HR_NONTRANS_EN_LIMIT2_FX_Q14, 2) > 0)
            {
                ind1 = 2;
                move16();
                /* 'min_env * BWE_HR_NONTRANS_EN_LIMIT2' */
                L_en_noncoded_fx = L_mult(min_env_fx, BWE_HR_NONTRANS_EN_LIMIT2_FX_Q14);
            }
            ELSE IF( L_msu(L_temp, BWE_HR_NONTRANS_EN_LIMIT3_FX_Q15, 1) > 0)
            {
                ind1 = 3;
                move16();
                L_en_noncoded_fx = L_mult0(min_env_fx, BWE_HR_NONTRANS_EN_LIMIT3_FX_Q15);
            }
            ELSE
            {
                ind1 = 0;
                move16();
                L_en_noncoded_fx = L_mult0(min_env_fx, 16384);
            }

            push_indice_fx( st_fx, IND_HR_HF_GAIN, ind1, NBITS_HF_GAIN_BWE_HR );
            nBits = sub(nBits, NBITS_HF_GAIN_BWE_HR);
        }
        ELSE
        {
            L_en_noncoded_fx = L_mult0(min_env_fx, 16384);
        }
        en_noncoded_fx_exp = 9+16-1;
        move16(); /* 9 for 'en_band_fx', 16 for 'BWE_HR_TRANS_EN_LIMIT...' & -1 becaues of 'L_mult0' */

        /*---------------------------------------------------------------------*
         * AVQ coding (quantize normalized spectrum)
         *---------------------------------------------------------------------*/

        Nsv = shr(i, WIDTH_BAND_SHIFT);
        AVQ_cod_fx(t_audio_tmp_fx/*same exponent as t_audio_fx*/, x_norm_fx, nBits, Nsv, t_audio_fx_exp);
        AVQ_encmux_fx(st_fx, st_fx->extl_fx, x_norm_fx, &nBits, Nsv, nq);

        /*---------------------------------------------------------------------*
         * second stage coding
         *---------------------------------------------------------------------*/

        IF( sub(nBits, 9 + NBITS_GLOB_GAIN_BWE_HR) >= 0 )
        {
            /* select spectrum of the second stage coding */
            ptr16 = &t_audio_fx[0];
            move16();
            FOR( i=0; i<Nsv; i++ )
            {
                IF( nq[i] == 0 )
                {
                    FOR( j=0; j<WIDTH_BAND; j++ )
                    {
                        *ptr16++ = t_audio_tmp_fx[i*WIDTH_BAND + j];
                        move16();
                    }
                }
            }
            temp2 = shl(1, t_audio_fx_exp);
            FOR( i=0; i<Nsv; i++ )
            {
                IF( nq[i] != 0 )
                {
                    FOR( j=0; j<WIDTH_BAND; j++ )
                    {
                        L_temp = L_deposit_l(t_audio_tmp_fx[i*WIDTH_BAND + j]);
                        L_temp = L_msu0(L_temp, temp2, x_norm_fx[i*WIDTH_BAND + j]);
                        *ptr16++ = extract_l(L_temp);
                    }
                }
            }

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

            /* second stage global gain estimation and coding */
            L_tmp = L_mult0(Nsv2, WIDTH_BAND);
            L_temp = Calc_Energy_Autoscaled(t_audio_fx, t_audio_fx_exp, extract_l(L_tmp), &temp2);
            /* Original Float Code: 'gain2 = (float)(16*sqrt( gain2 / (Nsv2*WIDTH_BAND) ))' */
            /* Or Instead: 'gain2 = (float)sqrt( gain2 / (Nsv2*WIDTH_BAND) * 256) )' */
            L_temp = Sqrt_Ratio32(L_temp, temp2, L_tmp, /*Log2 of 256*/8, &temp2);
            /* Put in Q16 */
            L_gain_fx = L_shr(L_temp, sub(31-16, temp2)); /* 31: 'L_temp' is already in Q31 */
            ind1 = gain_quant_fx( &L_gain_fx, &gain2_fx, LG10_MIN_GLOB_GAIN_BWE_HR_Q14, LG10_MAX_GLOB_GAIN_BWE_HR_Q13, NBITS_GLOB_GAIN_BWE_HR, &exp2);
            push_indice_fx( st_fx, IND_HR_GAIN, ind1, NBITS_GLOB_GAIN_BWE_HR );
            nBits = sub(nBits, NBITS_GLOB_GAIN_BWE_HR);

            /* normalize with global gain */
            temp2 = sub(4, exp2); /* 1/16 */
            temp = Invert16(gain2_fx, &temp2);
            FOR( i=0; i<Nsv2*WIDTH_BAND; i++ )
            {
                L_tmp = L_mult(temp, t_audio_fx[i]);
                L_tmp = L_shr(L_tmp, temp2);
                t_audio_fx[i] = round_fx(L_tmp);
            }

            set16_fx( nq2, 0, Nsv );

            AVQ_cod_fx( t_audio_fx, x_norm1_fx, nBits, Nsv2, t_audio_fx_exp );
            AVQ_encmux_fx( st_fx, st_fx->extl_fx, x_norm1_fx, &nBits, Nsv2, nq2 );
        }

    } /* 'ELSE' of ' IF( is_transient )' */

    /* write unused bits */
    WHILE( nBits > 0 )
    {
        i = min( nBits, 16 );
        push_indice_fx( st_fx, IND_UNUSED, 0, i );
        nBits = sub(nBits, i);
    }
    return;
}

