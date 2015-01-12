/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"


#include "stl.h"
#define               MAX_Q_NEW_INPUT 8
#define                     Q_WTDA_FX 13
#define                     Q_32_BITS 15

/*-------------------------------------------------------------------*
 * para_pred_bws()
 *
 * predict SWB parameters for bandwidth switching
 *-------------------------------------------------------------------*/
static
Word16 para_pred_bws_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure         */
    Word16 *signal_wb_fx,       /* i  : wideband frequency signal       */
    Word16 *SWB_fenv_fx,        /* o  : frequency-domain BWE envelope   */
    Word16 Q_syn
)
{
    Word16 i, j, k;
    Word16 mode;
    Word16 tmp, tmp_den, tmp_num;
    Word32 L_tmp, L_tmp_max;
    Word16 exp;
    Word16 *input_hi_fx;
    Word32 *mea;
    Word16 peak_fx, mag_fx;
    Word32 mean_fx[7], peak_32_fx;
    Word32 avrg1_fx, avrg2_fx, min_fx;
    Word16 att_fx;

    mode = NORMAL;
    move16();

    k = 0;
    move16();
    input_hi_fx = &signal_wb_fx[SHARP_WIDTH];
    move16();
    FOR(i = 0; i < 7; i ++)
    {
        peak_fx = 0;
        move16();
        mean_fx[i] = 0;
        move16();
        FOR(j = 0; j < SHARP_WIDTH; j ++)
        {
            mag_fx = abs_s(*input_hi_fx);
            peak_fx  = s_max(peak_fx ,mag_fx);
            /*IF (sub(mag_fx, peak_fx) > 0)  */
            /*{ */
            /*    peak_fx = mag_fx; */
            /*} */
            mean_fx[i] = L_add(mean_fx[i], L_deposit_l(mag_fx));
            move32();
            input_hi_fx ++;
        }

        IF(Q_syn < 11)
        {
            tmp = 1;
            move16();
        }
        ELSE
        {
            tmp = 0;
            move16();
            if(sub(shr(peak_fx, 3), shl(1, Q_syn)) > 0)
            {
                tmp = 1;
                move16();
            }
        }
        IF( tmp > 0)
        {
            L_tmp = L_msu0(Mult_32_16(L_shl(mean_fx[i], 10), 18432), peak_fx, 4544);
            if (L_tmp < 0)
            {
                k = add(k, 1);
            }
        }
    }

    avrg1_fx = L_deposit_l(0);
    avrg2_fx = L_deposit_l(0);
    FOR(i=1; i<4; i++)
    {
        avrg1_fx = L_add(avrg1_fx, mean_fx[i]);
        avrg2_fx = L_add(avrg2_fx, mean_fx[i+3]);
    }
    avrg1_fx = Mult_32_16(avrg1_fx, 10923);
    avrg2_fx = Mult_32_16(avrg2_fx, 10923); /* 1/3 -> Q15 -> 10923 */

    min_fx = L_add(2147483647, 0);     /*2^31 */
    peak_32_fx = L_deposit_l(0);
    FOR(i = 4; i < 7; i ++)
    {
        IF(L_sub(mean_fx[i], L_shl(avrg2_fx, 1)) > 0)
        {
            exp = norm_l(mean_fx[i]);
            IF(sub(exp, 16) < 0)
            {
                tmp_den = extract_l(L_shr(mean_fx[i], sub(16, exp)));  /*Qsyn - 16 + exp */
                tmp_num = extract_l(L_shr(avrg2_fx, sub(15, exp)));       /*//Qsyn - 16 + exp */
            }
            ELSE
            {
                tmp_den = extract_l(mean_fx[i]);
                tmp_num = extract_l(L_shl(avrg2_fx, 1));
            }

            tmp_den = div_s(1, tmp_den);

            tmp = i_mult(tmp_num, tmp_den);  /*Q15 */

            mean_fx[i] = Mult_32_16(mean_fx[i], tmp);
            move32();
        }
        min_fx = L_min(min_fx, mean_fx[i]);
        peak_32_fx = L_max(peak_32_fx, mean_fx[i]);
        /*IF(L_sub(mean_fx[i], min_fx) < 0) */
        /*{ */
        /*    min_fx = mean_fx[i]; */
        /*}   */
        /*IF(L_sub(mean_fx[i], peak_32_fx) > 0) */
        /*{ */
        /*    peak_32_fx = mean_fx[i]; */
        /*}  */
    }

    IF(sub(st_fx->tilt_wb_fx, 16384) > 0)
    {
        IF(sub(st_fx->tilt_wb_fx, 30720) > 0)
        {
            min_fx = peak_32_fx;
        }
        ELSE
        {
            tmp = extract_l(L_shr(L_mult0(st_fx->tilt_wb_fx, 17476), 14));  /*Q15 */
            min_fx = Mult_32_16(peak_32_fx, tmp);
        }
    }

    test();
    IF(peak_32_fx == 0 || min_fx == 0)
    {
        set16_fx(SWB_fenv_fx, 0, SWB_FENV);
    }
    ELSE
    {
        exp = norm_l(peak_32_fx);
        IF(sub(exp, 16) < 0)
        {
            tmp_den = extract_l(L_shr(peak_32_fx, sub(16, exp)));  /*Qsyn - 16 + exp */
            tmp = div_s(16384, tmp_den);     /*Q15+14 - (Qsyn - 16 + exp) */
            tmp_num = extract_l(L_shr(min_fx, sub(16, exp)));  /*Qsyn - 16 + exp  */

            tmp = extract_l(L_shr(L_mult0(tmp_num, tmp), 14));  /*Q15 */
        }
        ELSE
        {
            tmp_den = extract_l(peak_32_fx);  /*Qsyn  */
            exp = norm_s(tmp_den);
            tmp = div_s(shl(1, sub(14,exp)), tmp_den);   /*Q 29-exp - Qsyn */
            tmp_num = extract_l(min_fx);  /*Qsyn  */

            tmp = extract_l(L_shr(L_mult0(tmp_num, tmp), sub(14, exp)));  /*Q15 */
        }

        j = 0;
        move16();
        mea = &mean_fx[4];
        move16();
        L_tmp_max = L_shl(32767, add(Q_syn, 5));
        FOR(i = 0; i < SWB_FENV; i++)
        {
            IF(j == 5)
            {
                mea++;
                move16();
                j = 0;
                move16();
            }
            j++;
            move16();
            L_tmp = L_min(Mult_32_16(*mea, tmp), L_tmp_max);
            SWB_fenv_fx[i] = extract_l(L_shr(L_tmp, add(Q_syn, 5)));
        }
    }

    j = 0;
    move16();
    FOR(i = shr(SWB_FENV, 1); i < SWB_FENV; i++)
    {
        tmp = sub(32767, i_mult(j, 2341));
        move16();
        SWB_fenv_fx[i] = mult_r(SWB_fenv_fx[i], tmp);
        move16();
        j++;
        move16();
    }

    IF(L_sub(avrg1_fx, L_shl(avrg2_fx, 3)) > 0)
    {
        FOR(i = 0; i < SWB_FENV; i ++)
        {
            SWB_fenv_fx[i] = shr(SWB_fenv_fx[i], 1);
            move16();
        }
    }

    test();
    test();
    test();
    IF( (L_sub(st_fx->enerLH_fx, L_shr(st_fx->prev_enerLH_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLH_fx, 1), st_fx->prev_enerLH_fx) < 0) &&
        (L_sub(st_fx->enerLL_fx, L_shr(st_fx->prev_enerLL_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLL_fx, 1), st_fx->prev_enerLL_fx) < 0) )
    {
        FOR(i=0; i<SWB_FENV; i++)
        {
            test();
            IF(sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0 && sub(mult_r(SWB_fenv_fx[i], 16384),  st_fx->prev_SWB_fenv_fx[i]) > 0)
            {
                /*SWB_fenv_fx[i] = add(mult_r(SWB_fenv_fx[i], 3277), mult_r(st_fx->prev_SWB_fenv_fx[i], 29491)); */
                SWB_fenv_fx[i] = round_fx(L_mac(L_mult(SWB_fenv_fx[i], 3277), st_fx->prev_SWB_fenv_fx[i], 29491));
            }
            ELSE
            {
                /*SWB_fenv_fx[i] = add(mult_r(SWB_fenv_fx[i], st_fx->attenu_fx), mult_r(st_fx->prev_SWB_fenv_fx[i], sub(32767, st_fx->attenu_fx)));                     */
                SWB_fenv_fx[i] = round_fx(L_mac(L_mult(SWB_fenv_fx[i], st_fx->attenu_fx), st_fx->prev_SWB_fenv_fx[i], sub(32767, st_fx->attenu_fx)));
            }
        }

        IF(sub(st_fx->attenu_fx, 29491) < 0)
        {
            st_fx->attenu_fx = add(st_fx->attenu_fx, 1638);
            move16();
        }
    }
    ELSE
    {
        test();
        test();
        test();
        test();
        IF( L_sub(st_fx->core_brate_fx, st_fx->last_core_brate_fx) != 0 || (L_sub(st_fx->enerLH_fx, L_shr(st_fx->prev_enerLH_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLH_fx, 1), st_fx->prev_enerLH_fx) < 0) ||
        (L_sub(st_fx->enerLL_fx, L_shr(st_fx->prev_enerLL_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLL_fx, 1), st_fx->prev_enerLL_fx) < 0) )
        {
            FOR(i=0; i<SWB_FENV; i++)
            {
                if(sub(mult_r(SWB_fenv_fx[i], 16384),  st_fx->prev_SWB_fenv_fx[i]) > 0)
                {
                    SWB_fenv_fx[i] = st_fx->prev_SWB_fenv_fx[i];
                    move16();
                }
            }
        }

        FOR(i=0; i<SWB_FENV; i++)
        {
            /*SWB_fenv_fx[i] = add(mult_r(SWB_fenv_fx[i], 29491), mult_r(st_fx->prev_SWB_fenv_fx[i], 3277)); */
            SWB_fenv_fx[i] = round_fx(L_mac(L_mult(SWB_fenv_fx[i], 29491), st_fx->prev_SWB_fenv_fx[i], 3277));
        }
        st_fx->attenu_fx = 29491;
        move16();
    }

    if(sub(k, 3) > 0)
    {
        mode = HARMONIC;
        move16();
    }


    att_fx = i_mult(sub(N_WS2N_FRAMES, st_fx->bws_cnt_fx), 819);
    move16();/*15 */
    IF( sub(st_fx->L_frame_fx, L_FRAME16k) == 0 )
    {
        FOR( i = 0; i < 4; i++ )
        {
            SWB_fenv_fx[i] = mult_r(SWB_fenv_fx[i], att_fx);
            move16();  /*Q1 */
        }
    }

    FOR( i=4; i<SWB_FENV; i++ )
    {
        SWB_fenv_fx[i] = mult_r(SWB_fenv_fx[i], att_fx);
        move16();    /*Q1 */
    }

    return mode;
}

/*-------------------------------------------------------------------*
 * WB_BWE_gain_deq()
 *
 * Decoding of WB parameters
 *-------------------------------------------------------------------*/
Word16 WB_BWE_gain_deq_fx(
    Decoder_State_fx *st_fx ,    /* i/o: decoder state structure                 */
    Word16 *WB_fenv
)
{
    Word16 mode;
    Word16 index, index2;
    Word32 L_tmp;
    Word16 tmp, exp ,frac;

    index = (Word16) get_next_indice_fx(st_fx, 5 );
    mode = add(extract_l(get_next_indice_fx(st_fx, 1 )), 2);

    index2 = shl(index,1);
    L_tmp= L_shr(F_2_5_fx[index2],1);
    L_tmp = L_shl(L_tmp,6);

    frac = L_Extract_lc(L_tmp, &exp);
    tmp = extract_l(Pow2(14, frac));

    exp = sub(exp, 14);
    WB_fenv[0] = shl(tmp,add(exp,3));
    move16();


    L_tmp= L_shr(F_2_5_fx[add(index2, 1) ],1);
    L_tmp = L_shl(L_tmp,6);

    frac = L_Extract_lc(L_tmp, &exp);
    tmp = extract_l(Pow2(14, frac));

    exp = sub(exp, 14);
    WB_fenv[1] = shl(tmp,add(exp,3));
    move16();

    return (mode);
}

/*-------------------------------------------------------------------*
 * wb_bwe_dec()
 *
 * WB BWE decoder (only for 16kHz signals)
 *-------------------------------------------------------------------*/
Word16 wb_bwe_dec_fx(
    Word16 *synth_fx,            /* i/o: ACELP core synthesis/final synthesis    */
    Word16 *hb_synth_fx,         /* o  : SHB synthesis/final synthesis           */
    const Word16 output_frame,         /* i  : frame length                            */
    Word16 coder_type,           /* i  : coding type                             */
    Word16 *voice_factors_fx,    /* i  : voicing factors                         */
    const Word16 pitch_buf_fx[],       /* i  : pitch buffer                            */
    Decoder_State_fx *st_fx                /* i/o: decoder state structure                 */
    ,Word16 * Qpost
)
{
    Word16 mode;
    Word16 WB_fenv_fx[SWB_FENV];
    Word16 ysynth_fx[L_FRAME48k];
    Word16 Q_syn, exp, Q_syn_hb;
    Word32 L_wtda_synth_fx[2*L_FRAME48k], ysynth_32[L_FRAME48k], t_audio32_tmp[L_FRAME48k];
    Word16 scl, new_input_fx_exp;

    /* MDCT of the core synthesis signal */

    new_input_fx_exp = *Qpost;
    move16();
    wtda_fx(synth_fx, &new_input_fx_exp, L_wtda_synth_fx, st_fx->L_old_wtda_swb_fx,
            &st_fx->old_wtda_swb_fx_exp, ALDO_WINDOW, ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
            output_frame );
    *Qpost = sub(new_input_fx_exp,15);
    /* DCT of the ACELP core synthesis */
    direct_transform_fx(L_wtda_synth_fx, ysynth_32, 0, output_frame, &new_input_fx_exp);

    /* Convert to 16 Bits (Calc Shift Required to Stay within MAX_Q_NEW_INPUT) */
    scl = sub(16+MAX_Q_NEW_INPUT, new_input_fx_exp);
    /* Possible to Upscale? */
    IF (scl > 0)
    {
        /* Yes */
        /* Calc Room to Upscale */
        Q_syn = Find_Max_Norm32(ysynth_32, output_frame);
        /* Stay within MAX_Q_NEW_INPUT */
        scl = s_min(Q_syn, scl);
    }
    Copy_Scale_sig32_16(ysynth_32, ysynth_fx, output_frame, scl);
    Q_syn = add(sub(new_input_fx_exp, 16), scl);
    IF( !st_fx->bfi_fx )
    {
        IF( L_sub(st_fx->total_brate_fx, ACELP_13k20) == 0 )
        {
            /* de-quantization */
            mode = WB_BWE_gain_deq_fx(st_fx, WB_fenv_fx );
            st_fx->last_wb_bwe_ener_fx = mult_r(add(WB_fenv_fx[0], WB_fenv_fx[1]), 16384);
        }
        ELSE
        {
            if( sub(st_fx->last_extl_fx, WB_BWE) != 0 )
            {
                st_fx->prev_SWB_fenv_fx[0] = 0;
                move16();
            }

            mode = WB_BWE_gain_pred_fx( WB_fenv_fx, ysynth_fx, coder_type, st_fx->prev_coder_type_fx, st_fx->prev_SWB_fenv_fx[0],
            voice_factors_fx, pitch_buf_fx, st_fx->last_core_brate_fx, st_fx->last_wb_bwe_ener_fx, Q_syn);
            move16();
        }
    }
    ELSE
    {
        /* FEC */
        mode = NORMAL;
        move16();
        Copy( st_fx->prev_SWB_fenv_fx, WB_fenv_fx, 2 );
    }
    test();
    IF( sub(st_fx->last_extl_fx, WB_BWE) != 0 || st_fx->bfi_fx )
    {
        Copy( WB_fenv_fx, st_fx->prev_SWB_fenv_fx, 2 );
    }

    exp = norm_l(st_fx->prev_Energy_wb_fx);
    IF(sub(add(st_fx->prev_Q_synth, exp),Q_syn) > 0)
    {
        st_fx->prev_Energy_wb_fx = L_shr(st_fx->prev_Energy_wb_fx, sub(st_fx->prev_Q_synth, Q_syn));
    }
    ELSE
    {
        Q_syn = add(st_fx->prev_Q_synth, exp);
        st_fx->prev_Energy_wb_fx = L_shl(st_fx->prev_Energy_wb_fx, exp);
    }
    WB_BWE_decoding_fx( ysynth_fx, WB_fenv_fx, ysynth_32, L_FRAME16k, mode,
                        st_fx->last_extl_fx, &st_fx->prev_Energy_wb_fx, st_fx->prev_SWB_fenv_fx, &st_fx->prev_L_swb_norm_fx,
                        st_fx->extl_fx, coder_type, st_fx->total_brate_fx, &st_fx->Seed_fx, &st_fx->prev_flag_fx,
                        st_fx->prev_coder_type_fx, Q_syn, &Q_syn_hb );
    IF ( L_sub(st_fx->output_Fs_fx, 32000) == 0)
    {
        set32_fx( &ysynth_32[L_FRAME16k], 0, L_FRAME16k );
    }
    ELSE IF ( L_sub(st_fx->output_Fs_fx, 48000) == 0 )
    {
        set32_fx( &ysynth_32[L_FRAME16k], 0, L_FRAME32k );
    }
    Inverse_Transform( ysynth_32, &Q_syn_hb, t_audio32_tmp, 0, output_frame, output_frame );
    window_ola_fx( t_audio32_tmp, hb_synth_fx, &Q_syn_hb, st_fx->mem_imdct_fx, &st_fx->mem_imdct_exp_fx, output_frame,
                   ALDO_WINDOW,ALDO_WINDOW, 0,0,0);
    st_fx->prev_mode_fx = mode;
    st_fx->prev_Q_synth = Q_syn;
    return Q_syn_hb;
}
/*-------------------------------------------------------------------*
 * swb_bwe_gain_deq()
 *
 * Decoding of SWB parameters
 *-------------------------------------------------------------------*/
Word16 swb_bwe_gain_deq_fx(     /* o  : BWE class                           */
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure                 */
    const Word16 core,                /* i  : core                                */
    Word16 *SWB_tenv,           /* o  : Q0, time-domain BWE envelope        */
    Word16 *SWB_fenv,           /* o  : Q1, frequency-domain BWE envelope   */
    const Word16 hr_flag,             /* i  : high rate flag                      */
    const Word16 hqswb_clas           /* i  : HQ BWE class                        */
)
{
    Word16 index, mode, n_band;
    Word16 indice[6];
    Word16 quant_tmp[SWB_FENV/2], quant_tmp2[SWB_FENV/2];
    Word16 nb_bits[6];
    Word16 nenv;
    Word16 tmp,exp,frac;
    Word32 L_tmp;

    IF ( hqswb_clas > 0)
    {
        mode = (Word16)get_next_indice_fx( st_fx, 1 );
        IF (mode == 0)
        {
            mode = (Word16)get_next_indice_fx( st_fx, 1 );
        }
        ELSE
        {
            mode = HQ_GENERIC_SP_EXC;
            move16();
        }
    }
    ELSE
    {
        mode = (Word16)get_next_indice_fx( st_fx, 2 );
    }

    test();
    IF( sub(mode,1) == 0 && sub(core,ACELP_CORE) == 0 )
    {
        FOR( n_band = 0; n_band < SWB_TENV; n_band++ )
        {
            index = (Word16)get_next_indice_fx(st_fx, 4 );
            SWB_tenv[n_band] = shl(1, index);
            move16();
        }

        indice[0] = (Word16)get_next_indice_fx(st_fx, 7 );
        move16();
        indice[1] = (Word16)get_next_indice_fx(st_fx, 6 );
        move16();

        tmp = shl(indice[0],1);
        FOR(n_band = 0; n_band < DIM_TR1; n_band++)
        {
            /*Env_TR_Cdbk1_fx[ indice[0]*DIM_TR1+n_band]*/
            quant_tmp[2*n_band] = Env_TR_Cdbk1_fx[add(tmp,n_band)];/*Q8 */  move16();
        }

        /*tmp = indice[1]*DIM_TR2*/
        tmp = shl(indice[1],1);
        quant_tmp[1] = add(shr(add(quant_tmp[0], quant_tmp[2]),1), Env_TR_Cdbk2_fx[tmp]);
        move16();/*Q8 */
        quant_tmp[3] = add(quant_tmp[2],Env_TR_Cdbk2_fx[add(tmp,1)]);
        move16();/*Q8 */

        FOR(n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
        {
            tmp = add(quant_tmp[n_band], Mean_env_tr_fx[n_band]); /*Q8 */

            /*-----------------------------------------------------------------*
             * SWB_fenv[n_band] = pow(10.0, tmp/40)
             * = pow(2, 3.321928*tmp/40)
             * = pow(2, 0.0830482*tmp)
             *-----------------------------------------------------------------*/
            L_tmp = L_mult(tmp, 21771); /* 0.0830482 in Q18 -> Q27 */
            L_tmp = L_shr(L_tmp, 11); /* From Q27 to Q16 */
            frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
            tmp = extract_l(Pow2(13, frac));
            exp = sub(exp, 13);
            SWB_fenv[n_band] = shl(tmp, add(exp, 1));
            move16();/*Q1 */
        }
        /* in case of band-width switching, attenuate frame gain */
        IF( st_fx->bws_cnt1_fx > 0 )
        {
            tmp = i_mult(st_fx->bws_cnt1_fx, 1638);
            FOR(n_band = 0; n_band < SWB_TENV; n_band++)
            {
                SWB_tenv[n_band] = mult_r(SWB_tenv[n_band], tmp);
                move16();
            }

            FOR (n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
            {
                SWB_fenv[n_band] = mult_r(SWB_fenv[n_band], tmp);
                move16();
            }
        }
    }
    ELSE
    {
        nb_bits[0] = 5;
        move16();
        nb_bits[1] = 7;
        move16();
        nb_bits[2] = 6;
        move16();
        nb_bits[3] = 5;
        move16();

        IF ( sub(hr_flag,1) == 0 )
        {
            nb_bits[4] = 5;
            move16();
            nenv = SWB_FENV - 2;
            move16();
        }
        ELSE
        {
            nb_bits[4] = 6;
            move16();
            nenv = SWB_FENV;
            move16();
        }

        FOR (n_band = 0; n_band < 5; n_band++)
        {
            indice[n_band] = (Word16) get_next_indice_fx(st_fx, nb_bits[n_band] );
            move16();
        }

        IF ( sub(hqswb_clas,HQ_GEN_FB) == 0 )
        {
            indice[n_band] = (Word16) get_next_indice_fx(st_fx, 5 );
            move16();
        }

        Copy( &EnvCdbk11_fx[i_mult2(indice[0], DIM11)], quant_tmp, DIM11 );
        Copy( &EnvCdbk1st_fx[i_mult2(indice[1], DIM1ST)], quant_tmp2, DIM1ST );
        Copy( &EnvCdbk2nd_fx[i_mult2(indice[2], DIM2ND)], quant_tmp2+DIM1ST, DIM2ND );

        FOR( n_band = 0; n_band < DIM11-1; n_band++ )
        {
            quant_tmp[n_band] = add(quant_tmp[n_band], quant_tmp2[n_band]);
            move16();/*Q8 */
            SWB_fenv[n_band*2] = quant_tmp[n_band];
            move16(); /*Q8 */
        }

        IF ( sub(hr_flag,1) == 0 )
        {
            quant_tmp[6] = add(quant_tmp[6],quant_tmp2[6]);
            move16();/*Q8 */
            SWB_fenv[11] = quant_tmp[6];
            move16();

            Copy( &EnvCdbk3rd_fx[indice[3] * DIM3RD], quant_tmp2, DIM3RD );
            Copy( &EnvCdbk3rd_fx[indice[4] * DIM3RD], quant_tmp2+DIM3RD, DIM3RD );

            FOR(n_band = 0; n_band < 5; n_band++)
            {
                SWB_fenv[add(shl(n_band,1),1)] = add(shr(add(quant_tmp[n_band], quant_tmp[n_band+1]),1), quant_tmp2[n_band+1]);
                move16();/*Q8 */
            }

            SWB_fenv[0] = add(SWB_fenv[0], quant_tmp2[0]);
            move16();/*Q8 */
        }
        ELSE
        {
            quant_tmp[DIM11-1]=add(quant_tmp[DIM11-1],quant_tmp2[DIM11-1]);
            move16();/*Q8 */
            SWB_fenv[(DIM11-1)*2] = quant_tmp[DIM11-1];
            move16();

            Copy( &EnvCdbk3rd_fx[i_mult2(indice[3], DIM3RD)], quant_tmp2, DIM3RD );
            Copy( &EnvCdbk4th_fx[i_mult2(indice[4], DIM4TH)], quant_tmp2+DIM3RD, DIM4TH );

            FOR( n_band = 0; n_band < DIM12-1; n_band++ )
            {
                SWB_fenv[add(shl(n_band,1),1)] = add(shr(add(quant_tmp[n_band],quant_tmp[n_band+1]),1),quant_tmp2[n_band]);
                move16();/*Q8 */
            }

            SWB_fenv[n_band*2+1] = add(quant_tmp[n_band],quant_tmp2[n_band]);
            move16();/*Q8 */
        }

        FOR( n_band = 0; n_band < nenv; n_band++ )
        {
            tmp = add(SWB_fenv[n_band],Mean_env_fx[n_band]); /*Q8 */

            L_tmp = L_mult(tmp, 21771); /* 0.166096 in Q17 -> Q26 */
            L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
            frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */

            tmp = extract_l(Pow2(13, frac));/* Put 13 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */
            exp = sub(exp, 13);
            SWB_fenv[n_band] = shl(tmp, add(exp,1));
            move16();/*Q1 */
        }

        IF ( sub(hqswb_clas,HQ_GEN_FB) == 0 )
        {
            Copy( &EnvCdbkFB_fx[i_mult2(indice[5], DIM_FB)], &SWB_fenv[nenv], DIM_FB ); /*Q7 */

            FOR( n_band = 0; n_band < DIM_FB; n_band++ )
            {
                tmp = add(SWB_fenv[add(n_band,nenv)], Mean_env_fb_fx[n_band]);
                L_tmp = L_mult(tmp, 21771); /* 0.166096 in Q17 -> Q25 */
                L_tmp = L_shr(L_tmp, 9); /* From Q25 to Q16 */
                frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */

                tmp = extract_l(Pow2(13, frac));/* Put 13 as exponent so that */
                /* output of Pow2() will be: */
                /* 16384 < Pow2() <= 32767 */
                exp = sub(exp, 13);
                SWB_fenv[add(n_band,nenv)] = shl(tmp, add(exp,1));
                move16();
            }
        }

        /* in case of band-width switching, attenuate frame gain */
    }

    return mode;
}

/*-------------------------------------------------------------------*
 * swb_bwe_dec()
 *
 * SWB BWE decoder (only for 32kHz signals)
 *-------------------------------------------------------------------*/
Word16 swb_bwe_dec_fx(
    Decoder_State_fx *st_fx,              /* i/o: decoder state structure                 */
    Word16 *synth_fx,             /* i/o: ACELP core synthesis/final synthesis (might be rescaled inside wtda() )    */
    Word16 *hb_synth_fx,          /* o  : SHB synthesis/final synthesis           */
    const Word16 output_frame           /* i  : frame length                            */
    ,Word16 * Qpost
)
{
    Word16 i, l_subfr;
    Word16 mode;
    Word16 frica_flag = 0;
    Word16 idxGain;
    Word16 Q_syn, Q_syn_hb;
    Word16 ysynth_fx[L_FRAME48k];
    Word16 tmp;
    Word16 SWB_tenv_fx[SWB_TENV];
    Word32 L_wtda_synth_fx[2*L_FRAME48k], ysynth_32[L_FRAME48k];
    Word16 scl, new_input_fx_exp;
    Word32 t_audio32_tmp[L_FRAME48k];
    Word32 SWB_tenv_tmp_fx[SWB_TENV];
    Word32 L_tmp;
    Word16 exp, frac;
    Word16 fb_ener_adjust_fx = 0;
    Word16 SWB_fenv_fx[SWB_FENV];
    Word16 L;
    Word16 j = 0;
    Word16 ener_adjust_quan_fx;
    Word16 tmp2;
    /*---------------------------------------------------------------------*
     * SWB BWE decoding
     *---------------------------------------------------------------------*/
    /* windowing of the ACELP core synthesis */
    new_input_fx_exp = *Qpost;
    wtda_fx(synth_fx, &new_input_fx_exp, L_wtda_synth_fx, st_fx->L_old_wtda_swb_fx,
            &st_fx->old_wtda_swb_fx_exp,
            ALDO_WINDOW,
            ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
            output_frame );
    *Qpost = sub(new_input_fx_exp,15);
    /* DCT of the ACELP core synthesis */
    direct_transform_fx(L_wtda_synth_fx, ysynth_32, 0, output_frame, &new_input_fx_exp);

    /* Convert to 16 Bits (Calc Shift Required to Stay within MAX_Q_NEW_INPUT) */
    scl = sub(16+MAX_Q_NEW_INPUT, new_input_fx_exp);
    /* Possible to Upscale? */
    IF (scl > 0)
    {
        /* Yes */
        /* Calc Room to Upscale */
        Q_syn = Find_Max_Norm32(ysynth_32, output_frame);
        /* Stay within MAX_Q_NEW_INPUT */
        scl = s_min(Q_syn, scl);
    }
    Copy_Scale_sig32_16(ysynth_32, ysynth_fx, output_frame, scl);
    Q_syn = add(sub(new_input_fx_exp, 16), scl);

    IF( !st_fx->bfi_fx )
    {
        IF( st_fx->bws_cnt_fx > 0 )
        {
            /* estimate parameters */
            mode = para_pred_bws_fx( st_fx, ysynth_fx, SWB_fenv_fx, Q_syn );
            move16();
        }
        ELSE
        {
            /* de-quantization */
            mode = swb_bwe_gain_deq_fx(st_fx, ACELP_CORE, SWB_tenv_fx, SWB_fenv_fx, 0, -1 );
            move16();
        }

        L = SWB_FENV;
        move16();
        if(sub(mode, TRANSIENT) == 0)
        {
            L = SWB_FENV_TRANS;
            move16();
        }
        L_tmp = 0;
        move16();
        FOR(i=0; i<L; i++)
        {
            L_tmp = L_add(L_tmp, (Word32)SWB_fenv_fx[i]);
        }

        exp = norm_s(L);
        tmp = div_s(shl(1, sub(14,exp)), L); /*Q(29-exp) */

        L_tmp = Mult_32_16(L_tmp, tmp); /*Q(1+29-exp+1-16)->Q(15-exp) */
        st_fx->prev_ener_shb_fx = round_fx(L_shl(L_tmp, add(exp,2))); /*Q1 */
    }
    ELSE
    {
        /* SHB FEC */
        IF( sub(st_fx->prev_mode_fx, TRANSIENT) != 0 )
        {
            mode = st_fx->prev_mode_fx;
            move16();
        }
        ELSE
        {
            mode = NORMAL;
            move16();
        }

        Copy( st_fx->prev_SWB_fenv_fx, SWB_fenv_fx, SWB_FENV );
    }

    /* reconstruction of MDCT spectrum of the error signal */
    set32_fx( ysynth_32, 0, output_frame );

    test();
    test();
    IF( ( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->last_extl_fx, FB_BWE) != 0 ) || st_fx->bfi_fx )
    {
        Copy(SWB_fenv_fx, st_fx->prev_SWB_fenv_fx, SWB_FENV);
    }

    IF ( sub(st_fx->L_frame_fx, L_FRAME16k) == 0 )
    {
        SWB_BWE_decoding_fx( ysynth_fx, SWB_fenv_fx, ysynth_32, L_FRAME32k-80, mode, &frica_flag, &st_fx->prev_Energy_fx, st_fx->prev_SWB_fenv_fx,
                             &st_fx->prev_L_swb_norm_fx, st_fx->tilt_wb_fx, &st_fx->Seed_fx, 80, &st_fx->prev_weight_fx, st_fx->extl_fx, Q_syn );
    }
    ELSE
    {
        SWB_BWE_decoding_fx( ysynth_fx, SWB_fenv_fx, ysynth_32, L_FRAME32k-80, mode, &frica_flag, &st_fx->prev_Energy_fx, st_fx->prev_SWB_fenv_fx,
        &st_fx->prev_L_swb_norm_fx, st_fx->tilt_wb_fx, &st_fx->Seed_fx, 6, &st_fx->prev_weight_fx, st_fx->extl_fx, Q_syn );
    }

    test();
    IF ( sub(st_fx->prev_frica_flag_fx, 1) == 0 && frica_flag == 0 )
    {
        FOR( i = 0; i < L_SUBFR; i++ )
        {
            tmp = sub(32767, extract_l(L_mult0(i, 512))); /*Q15 */
            st_fx->mem_imdct_fx[i] = mult_r(st_fx->mem_imdct_fx[i], tmp);
            move16(); /*Q_synth */
        }

        FOR( ; i < output_frame; i++ )
        {
            st_fx->mem_imdct_fx[i] = 0;
            move16();
        }
    }

    /* decode information */
    IF ( sub(st_fx->extl_fx, FB_BWE) == 0 )
    {
        IF( !st_fx->bfi_fx )
        {
            idxGain = (Word16)get_next_indice_fx(st_fx, NUM_BITS_FB_FRAMEGAIN );
            fb_ener_adjust_fx = usdequant_fx(idxGain, FB_GAIN_QLOW_FX, FB_GAIN_QDELTA_FX); /*Q15 */
        }
        ELSE if( st_fx->bfi_fx )
        {
            fb_ener_adjust_fx = st_fx->prev_fb_ener_adjust_fx;
            move16();
        }

        st_fx->prev_fb_ener_adjust_fx = fb_ener_adjust_fx;
        move16();
        IF(sub(mode, TRANSIENT) == 0)
        {
            ener_adjust_quan_fx = shr(fb_ener_adjust_fx, 2);
            move16(); /*Q13*/
        }
        ELSE
        {
            IF(SWB_fenv_fx[7] != 0)
            {
                tmp = div_s(1, SWB_fenv_fx[7]);
                move16(); /*Q14*/
                ener_adjust_quan_fx = s_min(shr(i_mult(SWB_fenv_fx[13], tmp), 2), 32767);
                move16(); /*Q13*/
            }
            ELSE
            {
                ener_adjust_quan_fx = 0;
                move16(); /*Q13*/
            }
        }

        FOR( i = FB_BAND_BEGIN; i < FB_BAND_BEGIN+DE_OFFSET1; i++ )
        {
            tmp = sub(32767, i_mult(j, 1024));
            tmp = mult_r(tmp, ener_adjust_quan_fx); /*Q13*/

            tmp2 = i_mult(j, 256); /*Q13*/
            tmp2 = mult_r(tmp2, fb_ener_adjust_fx); /*Q13*/

            tmp = add(tmp, tmp2); /*Q13*/
            ysynth_32[i] = ysynth_32[i-FB_BAND_WIDTH];
            move16();
            ysynth_32[i] = L_shl(Mult_32_16(ysynth_32[i], tmp), 2);
            move32();/*15+Q_syn */
            j = add(j, 1);
        }

        FOR( ; i<FB_BAND_END; i++ )
        {
            ysynth_32[i] = ysynth_32[i-FB_BAND_WIDTH];
            move16();
            ysynth_32[i] = Mult_32_16(ysynth_32[i], fb_ener_adjust_fx);
            move32();/*15+Q_syn */
        }
    }

    Q_syn_hb = add(Q_syn, Q_32_BITS);
    Inverse_Transform( ysynth_32, &Q_syn_hb, t_audio32_tmp, 0, output_frame, output_frame );
    window_ola_fx( t_audio32_tmp, hb_synth_fx, &Q_syn_hb, st_fx->mem_imdct_fx, &st_fx->mem_imdct_exp_fx, output_frame,
                   ALDO_WINDOW, ALDO_WINDOW, 0,0,0);
    l_subfr = mult(output_frame, 8192);

    test();
    IF( sub(mode,TRANSIENT) == 0 )
    {
        FOR(i = 0; i < SWB_TENV; i++)
        {
            SWB_tenv_tmp_fx[i] = L_mult0(SWB_tenv_fx[i], 26214);
            move32();/*Q15 */
        }

        /* time envelope shaping when the current frame is TRANSIENT frame */
        time_envelop_shaping_fx( hb_synth_fx, SWB_tenv_tmp_fx, output_frame, &Q_syn_hb );
        Q_syn_hb = sub(Q_syn_hb, 3);

        st_fx->prev_td_energy_fx = SWB_tenv_fx[3];
        move16();
    }
    ELSE IF( sub(frica_flag, 1) == 0 && st_fx->prev_frica_flag_fx == 0 )
    {
        time_reduce_pre_echo_fx( synth_fx, hb_synth_fx, st_fx->prev_td_energy_fx, l_subfr, Q_syn, Q_syn_hb );
    }
    ELSE
    {
        tmp = i_mult2(3, l_subfr);
        L_tmp = L_deposit_l(0);
        FOR(i=0; i<l_subfr; i++)
        {
            L_tmp = L_mac0(L_tmp, hb_synth_fx[tmp+i], hb_synth_fx[tmp+i]); /*(2*Q_syn_hb) */
        }
        L_tmp = Mult_32_16(L_tmp, div_s(1, l_subfr)); /*(2*Q_syn_hb) */

        st_fx->prev_td_energy_fx = 0;
        move16();
        IF(L_tmp != 0)
        {
            exp = norm_l(L_tmp);
            frac = extract_h(L_shl(L_tmp, exp));
            exp = sub(exp, sub(30,shl(Q_syn_hb,1)));

            tmp = div_s(16384, frac);
            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */
            st_fx->prev_td_energy_fx = round_fx(L_shl(L_tmp, sub(exp,15))); /*Q0 */
        }
    }

    st_fx->prev_frica_flag_fx = frica_flag;
    move16();
    st_fx->prev_mode_fx = mode;
    move16();

    return Q_syn_hb;
}
