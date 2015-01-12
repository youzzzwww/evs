/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*BASOp version info: This file is up to date with trunk rev. 39929 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * decoder_acelp()
 *
 * Decode ACELP frame
 *-------------------------------------------------------------------*/

void decoder_acelp(
    Decoder_State_fx *st,
    Word16 coder_type,            /* i  : coder type                     */
    Word16 prm[],                 /* i  : parameters                     */
    Word16 A[],                   /* i  : coefficients NxAz[M+1]         */
    ACELP_config acelp_cfg,       /* i  : ACELP config                   */
    Word16 synth[],               /* i/o: synth[-2*LFAC..L_DIV]    Q0    */
    Word16 *pT,                   /* o  : pitch for all subframe   Q0    */
    Word16 *pgainT,               /* o  : pitch gain for all subfr 1Q14  */
    Word16 stab_fac,              /* i  : stability of isf               */
    Word16 *pitch_buffer,         /* o  : pitch values for each subfr.   */
    Word16 *voice_factors,        /* o  : voicing factors                */
    Word16 *bwe_exc               /* o  : excitation for SWB TBE         */
)
{
    Word16 i, j, i_subfr;
    Word16 T0, T0_frac, T0_min, T0_min_frac, T0_max, T0_max_frac, T0_res;
    Word16 tmp, tmp2, gain_pit/*Q14*/, Es_pred, tmp_deemph;
    Word32 Ltmp, Ltmp2, gain_code;
    Word16 code[L_SUBFR];
    Word16 mem_syn[M];
    Word16 *syn, syn_buf[M+L_DIV_MAX+L_DIV_MAX/2];
    Word16 *exc, exc_buf[L_EXC_MEM_DEC+L_DIV_MAX+1];
    Word16 exc2[L_FRAME_16k];
    Word16 *p_A;
    Word32 pitch_buf[NB_SUBFR16k];
    Word16 dummy_pitch_buf[NB_SUBFR16k];
    Word16 gain_inov;
    Word16 mem_back[M];
    Word16 update_flg;
    Word16 Q_mem_back;    /*Q format of mem_back*/
    Word16 h1[L_FRAME_16k/4+1];
    Word16 mem[M];
    Word16 *pA;
    PulseConfig config;
    Word16 weights[5]; /* Q15 */

    Word16 reScaleLen_fx;    /* rescaling length for the BWE buffers */
    Word16 reSampLen;

    /*Q formats of buffers   */
    Word16 prev_Q_syn;
    Word32 gain_code2=0;
    Word16 code2[L_SUBFR];
    Word16 dect_low;
    Word16 error = 0;
    Word16 gain_preQ = 0;                /* Gain of prequantizer excitation   */
    Word16 code_preQ[L_SUBFR];           /* Prequantizer excitation           */
    Word16 lp_flag;


    Word16 prev_gain_pit;
    Word16 tmp_noise;   /* Long term temporary noise energy */
    Word32 gain_code_tmp;
    Word16 gain_pit_tmp;
    Word32 gain_code_pre;
    gain_code_pre = 0;
    move16();


    set16_fx(code_preQ, 0, L_SUBFR);

    gain_inov = 0;    /* to avoid compilation warnings */
    T0 = 0;           /* to avoid compilation warnings */
    T0_frac = 0;      /* to avoid compilation warnings */
    T0_res = 0;       /* to avoid compilation warnings */
    move16();
    move16();
    move16();
    prev_Q_syn = st->prev_Q_syn = st->Q_syn;
    dect_low = 1;

    move16();
    move16();
    gain_pit = 0;
    gain_code = 0;
    move16();
    move16();
    move16();
    update_flg = 0;
    move16();
    gain_code2=0;

    move16();
    dect_low=1;

    prev_gain_pit = 0;
    tmp_noise = 0;

    IF(sub(st->nb_subfr,4)==0)
    {
        move16();
        move16();
        move16();
        move16();
        weights[0] = FL2WORD16(0.1f);
        weights[1] = FL2WORD16(0.2f);
        weights[2] = FL2WORD16(0.3f);
        weights[3] = FL2WORD16(0.4f);
    }
    ELSE  /*nb_subfr == 5*/
    {
        move16();
        move16();
        move16();
        move16();
        move16();
        weights[0] = FL2WORD16((float)1/15);
        weights[1] = FL2WORD16((float)2/15);
        weights[2] = FL2WORD16((float)3/15);
        weights[3] = FL2WORD16((float)4/15);
        weights[4] = FL2WORD16((float)5/15);
    }
    st->Mode2_lp_gainp = L_deposit_l(0);
    st->Mode2_lp_gainc = L_deposit_l(0);


    /*------------------------------------------------------------------------*
     * Previous frame is TCX                                                  *
     *------------------------------------------------------------------------*/


    /* Reset phase dispersion */
    IF ( st->last_core_bfi > ACELP_CORE )
    {
        st->dm_fx.prev_gain_code = L_deposit_l(0);
        set16_fx(st->dm_fx.prev_gain_pit, 0, 6);
        st->dm_fx.prev_state = 0;
        move16();
    }

    /* Update of synthesis filter memories in case of 12k8 core */
    test();
    test();
    IF ( st->prev_bfi_fx && st->last_con_tcx && sub(st->L_frame_fx, L_FRAME16k) < 0 )
    {
        synth_mem_updt2( st->L_frame_fx, L_FRAME16k, st->old_exc_fx, st->mem_syn_r, st->mem_syn2_fx, NULL, DEC );
    }

    /*------------------------------------------------------------------------*
     * Initialize buffers                                                     *
     *------------------------------------------------------------------------*/
    Copy( st->mem_syn2_fx, mem_back, M);
    move16();
    Q_mem_back = st->Q_syn;
    /* set ACELP synthesis memory */
    Copy( st->mem_syn2_fx, mem_syn, M );

    /* set excitation memory*/
    exc=exc_buf+L_EXC_MEM_DEC;
    Copy( st->old_exc_fx, exc_buf, L_EXC_MEM_DEC);
    *(exc+st->L_frame_fx) = 0;

    /* Init syn buffer */
    syn = syn_buf + M;
    Copy( st->mem_syn2_fx, syn_buf, M );

    /*------------------------------------------------------------------------*
     * Fast recovery flag
     *------------------------------------------------------------------------*/
    test();
    if(st->prev_bfi_fx && sub(coder_type,VOICED)==0)
    {
        /*Force BPF to be applied fully*/
        st->bpf_gain_param=3;
        move16();
    }

    /*------------------------------------------------------------------------*
     * - decode mean_ener_code for gain decoder (d_gain2.c)                   *
     *------------------------------------------------------------------------*/
    Es_pred = 0;
    move16();
    IF ( acelp_cfg.nrg_mode>0 )
    {
        d_gain_pred(acelp_cfg.nrg_mode, &Es_pred, &prm );
    }

    /*------------------------------------------------------------------------*
     *          Loop for every subframe in the analysis frame                 *
     *------------------------------------------------------------------------*
     *  To find the pitch and innovation parameters. The subframe size is     *
     *  L_subfr and the loop is repeated L_ACELP/L_subfr times.               *
     *     - compute impulse response of weighted synthesis filter (h1[])     *
     *     - compute the target signal for pitch search                       *
     *     - find the closed-loop pitch parameters                            *
     *     - encode the pitch delay                                           *
     *     - update the impulse response h1[] by including fixed-gain pitch   *
     *     - find target vector for codebook search                           *
     *     - correlation between target vector and impulse response           *
     *     - codebook search                                                  *
     *     - encode codebook address                                          *
     *     - VQ of pitch and codebook gains                                   *
     *     - find synthesis speech                                            *
     *     - update states of weighting filter                                *
     *------------------------------------------------------------------------*/

    p_A = A;

    FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
    {

        test();
        IF( sub(st->use_partial_copy,1)== 0 && sub(st->rf_frame_type,RF_NELP) == 0 )
        {
            IF( i_subfr == 0 )
            {
                decod_nelp_fx( st, UNVOICED, &tmp_noise, dummy_pitch_buf, exc, exc2, voice_factors, bwe_exc, &st->Q_exc, st->bfi_fx, pgainT );

                set32_fx(pitch_buf, L_SUBFR_Q16, NB_SUBFR);
                set16_fx(pitch_buffer, 4096, NB_SUBFR);  /* L_SUBFR_Q16 in Q6 */
            }
        }
        ELSE
        {

            /*-------------------------------------------------------*
             * - Decode adaptive codebook.                           *
             *-------------------------------------------------------*/

            test();
            IF( sub(st->use_partial_copy,1)== 0 && st->acelp_cfg.gains_mode[i_subfr/L_SUBFR] == 0 )
            {
                gain_pit = prev_gain_pit;
            }

            IF ( acelp_cfg.ltp_bits != 0 )
            {
                /*if( st->use_partial_copy
                    && st->rf_frame_type == RF_GENPRED
                    && ( i_subfr == L_SUBFR || i_subfr == 3*L_SUBFR ) )
                {
                    *pt_pitch = (float)T0 + (float)T0_frac/(float)T0_res;
                }
                else*/
                {
                    /* pitch lag decoding */
                    pitch_buf[i_subfr/L_SUBFR] = Mode2_pit_decode(  acelp_cfg.ltp_mode,
                    i_subfr,
                    L_SUBFR,
                    &prm,
                    &T0,
                    &T0_frac,
                    &T0_res,
                    &T0_min,
                    &T0_min_frac,
                    &T0_max,
                    &T0_max_frac,
                    st->pit_min,
                    st->pit_fr1,
                    st->pit_fr1b,
                    st->pit_fr2,
                    st->pit_max,
                    st->pit_res_max);
                }
                /* find pitch excitation */
                test();
                IF( sub(st->pit_res_max,6) == 0 && !(st->use_partial_copy) )
                {
                    IF ( sub(T0_res, shr(st->pit_res_max, 1)) == 0)
                    {
                        pred_lt4( &exc[i_subfr], &exc[i_subfr], T0, shl(T0_frac,1), L_SUBFR+1, pitch_inter6_2, PIT_L_INTERPOL6_2, PIT_UP_SAMP6 );
                    }
                    ELSE
                    {
                        pred_lt4( &exc[i_subfr],&exc[i_subfr],  T0, T0_frac, L_SUBFR+1, pitch_inter6_2, PIT_L_INTERPOL6_2, PIT_UP_SAMP6 );
                    }
                }
                ELSE
                {
                    IF ( sub(T0_res, shr(st->pit_res_max, 1)) == 0)
                    {
                        pred_lt4( &exc[i_subfr], &exc[i_subfr], T0, shl(T0_frac,1), L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );
                    }
                    ELSE
                    {
                        pred_lt4( &exc[i_subfr], &exc[i_subfr], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );
                    }
                }

                /* LP filtering of the adaptive excitation*/
                lp_flag = acelp_cfg.ltf_mode;
                move16();

                IF( sub(acelp_cfg.ltf_mode, NORMAL_OPERATION) == 0 )
                {
                    lp_flag = *prm;
                    move16();
                    prm++;
                }

                lp_filt_exc_dec_fx( st, MODE2, st->core_brate_fx, 0, coder_type, i_subfr, L_SUBFR, st->L_frame_fx, lp_flag, exc );

            }
            ELSE IF ( acelp_cfg.ltp_bits==0 )
            {
                /* No adaptive codebook (UC) */
                set16_fx(&exc[i_subfr], 0, L_SUBFR);

                T0 = L_SUBFR;
                T0_frac = 0;
                T0_res = 1;
                pitch_buf[i_subfr/L_SUBFR] = L_deposit_h(L_SUBFR);
            }

            IF( st->igf != 0 )
            {
                tbe_celp_exc(st->L_frame_fx, i_subfr, T0, T0_frac, &error, bwe_exc);
            }

            pitch_buffer[i_subfr/L_SUBFR] = shl(add(shl(T0,2),T0_frac), 4);

            /*-------------------------------------------------------*
             * - Decode innovative codebook.                         *
             *-------------------------------------------------------*/
            test();
            test();
            test();
            test();
            IF( sub(st->use_partial_copy,1)==0 &&
                ( sub(st->rf_frame_type,RF_ALLPRED) == 0 ||
                  ( sub(st->rf_frame_type,RF_GENPRED) == 0 &&
                    ( sub(i_subfr,L_SUBFR) == 0 || sub(i_subfr,3*L_SUBFR) == 0 )) ) )
            {
                set16_fx(code, 0, L_SUBFR);
            }
            ELSE
            {
                config = PulseConfTable[acelp_cfg.fixed_cdk_index[i_subfr/L_SUBFR]];
                D_ACELP_indexing( code, config, 4, prm );
                (prm) += 8;
                /*-------------------------------------------------------*
                 * - Add the fixed-gain pitch contribution to code[].    *
                 *-------------------------------------------------------*/

                E_UTIL_cb_shape( acelp_cfg.pre_emphasis, acelp_cfg.pitch_sharpening, acelp_cfg.phase_scrambling, acelp_cfg.formant_enh, acelp_cfg.formant_tilt,
                acelp_cfg.formant_enh_num, acelp_cfg.formant_enh_den, p_A, code, st->tilt_code_fx, extract_h(L_add(pitch_buf[i_subfr/L_SUBFR],26216)) );

            }
            /*-------------------------------------------------------*
             * - Generate Gaussian excitation                        *
             *-------------------------------------------------------*/
            test();
            IF( sub(acelp_cfg.gains_mode[i_subfr/L_SUBFR],7)==0 && !st->use_partial_copy )
            {
                gaus_L2_dec( code2, st->tilt_code_fx, p_A, acelp_cfg.formant_enh_num, &(st->seed_acelp) );
            }
            ELSE
            {
                gain_code2 = L_deposit_l(0);
                set16_fx(code2,0,L_SUBFR);
            }

            /*-------------------------------------------------*
             * - Decode codebooks gains.                       *
             *-------------------------------------------------*/
            IF( st->acelp_cfg.gains_mode[i_subfr/L_SUBFR] != 0 )
            {
                decode_acelp_gains( code,
                acelp_cfg.gains_mode[i_subfr/L_SUBFR],
                Es_pred,
                &gain_pit,
                &gain_code,
                &prm,
                &(st->past_gpit),
                &(st->past_gcode),
                &gain_inov,
                L_SUBFR,
                code2,
                &gain_code2
                                  );
            }
            IF(st->use_partial_copy && st->rf_frame_type == RF_ALLPRED) st->past_gcode = 0;
            IF(st->use_partial_copy && st->rf_frame_type == RF_NOPRED ) st->past_gpit = 67;

            dect_low=dect_low&(*(prm-1)==8);

            IF( st->igf != 0 )
            {
                /* Rescaling for 12.8k and 16k cores related to BWE */
                IF ( sub(st->L_frame_fx, L_FRAME) == 0 )
                {
                    /* 5/2 times resampled past memories*/
                    reScaleLen_fx = add(shl(i_subfr, 1), shr(i_subfr, 1));
                    reSampLen = (L_SUBFR * HIBND_ACB_L_FAC);
                }
                ELSE
                {
                    /* 2 times resampled past memories*/
                    reScaleLen_fx = shl(i_subfr, 1);
                    reSampLen = (L_SUBFR * 2);
                }

                Rescale_exc(NULL, &exc[i_subfr], &bwe_exc[reScaleLen_fx],
                            NULL, L_SUBFR, reSampLen, gain_code, &(st->Q_exc), st->Q_subfr,
                            exc2, i_subfr, GENERIC);
            }
            ELSE
            {
                Rescale_exc(NULL, &exc[i_subfr], NULL, NULL, L_SUBFR, 0,
                gain_code, &(st->Q_exc), st->Q_subfr, exc2, i_subfr, GENERIC);
            }

            /*----------------------------------------------------------*
             * Update parameters for the next subframe.                 *
             * - tilt of code: 0.0 (unvoiced) to 0.5 (voiced)           *
             *----------------------------------------------------------*/
            E_UTIL_voice_factor( exc,
            i_subfr,
            code,
            gain_pit,
            gain_code,
            &(st->voice_fac),
            &(st->tilt_code_fx),
            L_SUBFR,
            acelp_cfg.voice_tilt,
            st->Q_exc,
            0
                               );

            pgainT[i_subfr/L_SUBFR] = gain_pit;
            move16();

            /*-------------------------------------------------------*
             * - Find the total excitation.                          *
             *-------------------------------------------------------*/
            gain_code_tmp = gain_code;
            gain_pit_tmp = gain_pit;
            IF( i_subfr == 0 )
            {
                gain_code_pre = 0;
            }
            IF ( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->last_core_fx,ACELP_CORE) == 0 && ( sub(st->use_partial_copy,1) == 0|| sub(st->prev_use_partial_copy, 1) == 0))
            {
                IF( i_subfr > 0 && sub(gain_pit,20152) > 0 && sub(st->prev_tilt_code_dec_fx,6553) > 0 && sub(st->next_coder_type,VOICED) == 0
                &&( sub(st->use_partial_copy,1) == 0 || sub(st->prev_use_partial_copy, 1) ==0 ) )
                {
                    gain_pit = mult(gain_pit,sub(26214, mult(i_subfr,51)));
                }
                ELSE IF( !st->prev_use_partial_copy && sub(st->last_coder_type_fx,UNVOICED) == 0 && sub(st->next_coder_type,UNVOICED) != 0 && L_sub(gain_code,gain_code_pre)< 0)

                {
                    gain_code = 0;
                }
            }
            gain_code_pre = gain_code;
            st->tilt_code_dec_fx[i_subfr/L_SUBFR] = st->tilt_code_fx;


            tmp2 = shr(L_SUBFR, 1);
            FOR (j = 0; j < 2; j++)
            {
                FOR (i = sub(tmp2, shr(L_SUBFR, 1)); i < tmp2; i++)
                {
                    /* code in Q9, gain_pit in Q14, gain_code in Q16; exc Q_new */
                    Ltmp = Mpy_32_16_1(gain_code2, code2[i]);
                    Ltmp = L_shl(Ltmp, add(5,st->Q_exc));
                    Ltmp = L_mac(Ltmp, gain_pit, exc[i+i_subfr]);
                    exc2[i + i_subfr] = round_fx(L_shl(Ltmp, 1));

                    Ltmp2 = Mpy_32_16_1(gain_code, code[i]);
                    Ltmp2 = L_shl(Ltmp2, add(5,st->Q_exc));
                    Ltmp = L_add(Ltmp, Ltmp2);
                    BASOP_SATURATE_WARNING_OFF
                    Ltmp = L_shl(Ltmp, 1);       /* saturation can occur here */
                    BASOP_SATURATE_WARNING_ON
                    exc[i + i_subfr] = round_fx(Ltmp);
                }
                tmp2 = L_SUBFR;
                move16();
            }

            /*-----------------------------------------------------------------*
             * Prepare TBE excitation
             *-----------------------------------------------------------------*/
            gain_code = gain_code_tmp;
            gain_pit = gain_pit_tmp;
            IF( st->igf != 0 )
            {
                prep_tbe_exc_fx( st->L_frame_fx,
                                 i_subfr,
                                 gain_pit,
                                 gain_code,
                                 code,
                                 st->voice_fac,
                                 &voice_factors[i_subfr/L_SUBFR],
                                 bwe_exc,
                                 gain_preQ,
                                 code_preQ,
                                 st->Q_exc,
                                 T0,
                                 T0_frac,
                                 coder_type,
                                 st->core_brate_fx);
            }

            /*---------------------------------------------------------*
             * Enhance the excitation                                  *
             *---------------------------------------------------------*/
            E_UTIL_enhancer(  st->voice_fac,
                              stab_fac,
                              st->past_gcode,
                              gain_inov,
                              &(st->gc_threshold_fx),
                              code,
                              &exc2[i_subfr],
                              gain_pit,
                              &st->dm_fx.prev_gain_code,
                              st->dm_fx.prev_gain_pit,
                              &st->dm_fx.prev_state,
                              coder_type,
                              acelp_cfg.fixed_cdk_index[i_subfr/L_SUBFR],
                              L_SUBFR,
                              st->L_frame_fx,
                              st->Q_exc
                           );

        } /* !RF_NELP frame partial copy */
        /*----------------------------------------------------------*
         * - compute the synthesis speech                           *
         *----------------------------------------------------------*/
        rescale_mem(&st->Q_exc, &prev_Q_syn, &st->Q_syn, mem_syn, syn, M, i_subfr);

        E_UTIL_synthesis(sub(st->Q_exc,st->Q_syn), p_A, &exc2[i_subfr], &syn[i_subfr], L_SUBFR, mem_syn, 1, M);

        /*-----------------------------------------------------------------*
         * update lp_filtered gains for the case of frame erasure
         *-----------------------------------------------------------------*/

        st->Mode2_lp_gainp = L_add(st->Mode2_lp_gainp, L_mult0(st->past_gpit, weights[i_subfr/L_SUBFR])); /* 2Q29=1Q14*Q15 */       move32();
        st->Mode2_lp_gainc = L_add(st->Mode2_lp_gainc, Mpy_32_16_1(st->past_gcode, weights[i_subfr/L_SUBFR]));  /* 15Q16=15Q16*Q15 */      move32();

        /*----------------------------------------------------------*
         * - update pitch lag for guided ACELP                      *
         *----------------------------------------------------------*/
        test();
        if( st->enableGplc && sub( shr(i_subfr,6), sub(st->nb_subfr,1) )==0 )
        {
            st->T0_4th = T0;
            move16();
        }

        /*----------------------------------------------------------*
         * - Update LPC coeffs                                      *
         *----------------------------------------------------------*/
        p_A += (M+1);

        /* copy current gain for next subframe use, in case there is no explicit encoding */
        prev_gain_pit = gain_pit;
    } /* end of subframe loop */

    tmp = 0;
    move16();
    pA = A+(st->nb_subfr-1)*(M+1);
    set16_fx(h1, 0, add(L_SUBFR,1));
    set16_fx(mem, 0, M);
    h1[0] = 32768/32;
    move16();
    E_UTIL_synthesis(0, pA, h1, h1, L_SUBFR, mem, 0, M);     /* impulse response of LPC     */
    deemph_fx(h1, st->preemph_fac, L_SUBFR, &tmp);    /* impulse response of deemph  */
    /* impulse response level = gain introduced by synthesis+deemphasis */
    Ltmp = Dot_productSq16HQ( 0, h1, L_SUBFR, &st->last_gain_syn_deemph_e);
    st->last_gain_syn_deemph_e= add(st->last_gain_syn_deemph_e,10/*scaling of h1[0] and E_UTIL_synthesis * 2*/);
    st->last_gain_syn_deemph = round_fx(Sqrt32(Ltmp,&st->last_gain_syn_deemph_e));

    /* Do the classification */
    {
        Word16 pit16[NB_SUBFR16k];
        Word16 k;
        FOR(k = 0 ; k < st->nb_subfr ; k++)
        {
            pit16[k] = round_fx(L_shl(pitch_buf[k],6));/*Q6*/
        }

        FEC_clas_estim_fx(
            st,
            /*Opt_AMR_WB*/0, /*A*/
            st->L_frame_fx,
            &(st->clas_dec),
            st->core_ext_mode,
            pit16,
            &st->classifier_last_good,
            syn,
            &st->lp_ener_FER_fx,
            /**decision_hyst*/NULL,     /* i/o: hysteresis of the music/speech decision                               */
            /**UV_cnt*/ NULL,           /* i/o: number of consecutives frames classified as UV                        */
            /**LT_UV_cnt*/ NULL,        /* i/o: long term consecutives frames classified as UV                        */
            /**Last_ener*/ NULL,        /* i/o: last_energy frame                                                     */
            /**locattack*/ NULL,        /* i/o: detection of attack (mainly to localized speech burst)                */
            /**lt_diff_etot*/NULL,      /* i/o: long-term total energy variation                                      */
            /**amr_io_class*/ NULL,     /* i/o: classification for AMR-WB IO mode                                     */
            /*bitrate*/ 0  ,            /* i  : Decoded bitrate                                                       */
            &st->Q_syn,                 /* i  : Synthesis scaling                                                     */
            /**class_para*/ NULL,       /* o  : classification para. fmerit1                                          */
            st->mem_syn_clas_estim_fx,  /* i/o: memory of the synthesis signal for frame class estimation             */
            &st->classifier_Q_mem_syn,  /*i/o : exponent for memory of synthesis signal for frame class estimation    */
            st->pit_max,                /* i  : maximum pitch value, Q0                                               */
            FL2WORD16(-1.f),            /* i  : LTP Gain                                                              */
            0/*CLASSIFIER_ACELP*/,      /* i  : signal classifier mode                                                */
            0/*bfi*/,                   /* i  : bad frame indicator                                                   */
            M,                          /* i  : starting point for synth buffer                                       */
            add(M,
                add(st->L_frame_fx,
                    shr(st->L_frame_fx,1)))/*i   : length of synthesis buffer, relevant for rescaling                       */
        );
    }

    /* Update Pitch Lag memory */

    Copy32(&st->old_pitch_buf_fx[st->nb_subfr], st->old_pitch_buf_fx, st->nb_subfr);
    Copy32(pitch_buf, &st->old_pitch_buf_fx[st->nb_subfr], st->nb_subfr);


    {
        Word16 pBuf_scaleSyn[NB_SUBFR16k];

        FOR(i=0; i<(st->L_frame_fx/L_SUBFR); i++)
        {
            pBuf_scaleSyn[i] = round_fx(pitch_buf[i]);
        }

        Scale_sig(mem_back, M, sub(st->Q_syn,Q_mem_back));

        FEC_scale_syn_fx( st->L_frame_fx,
                          &update_flg,
                          st->clas_dec,
                          st->last_good_fx,
                          syn,
                          pBuf_scaleSyn,
                          st->enr_old_fx,
                          0,
                          coder_type,
                          st->prev_bfi_fx,
                          st->last_core_brate_fx,
                          exc,
                          exc2,
                          A,
                          &(st->old_enr_LP),
                          mem_back,
                          mem_syn,
                          st->Q_exc,
                          st->Q_syn);
    }

    /* update ACELP synthesis memory */
    Copy( mem_syn, st->mem_syn2_fx, M );
    Copy( syn+st->L_frame_fx-L_SYN_MEM, st->mem_syn_r, L_SYN_MEM );

    /* Deemphasis and output synth and ZIR */
    tmp_deemph = st->syn[M];
    move16();
    E_UTIL_deemph2(st->Q_syn, syn, st->preemph_fac, st->L_frame_fx, &tmp_deemph); /* tmp_deemph and syn in Q0 starting from here*/

    bufferCopyFx(syn+st->L_frame_fx- st->L_frame_fx/2, st->old_syn_Overl, shr(st->L_frame_fx, 1),0 /*Qf_syn*/, -1 /*Qf_old_xnq*/, 0 , 0 /*Q_old_xnq*/);

    Copy(syn+st->L_frame_fx-M-1, st->syn, 1+M); /*Q0*/

    Copy(syn, synth, st->L_frame_fx);

    Copy( syn, st->old_core_synth_fx, st->L_frame_fx );


    /* update old_Aq */
    Copy(exc_buf+st->L_frame_fx, st->old_exc_fx,  L_EXC_MEM_DEC);

    /* Output pitch parameters for bass post-filter */
    FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
    {
        *pT++ = round_fx(pitch_buf[i_subfr/L_SUBFR]);
    }


    st->tcxltp_last_gain_unmodified = 0;

    /*Update MODE1*/
    Copy( p_A-(M+1), st->old_Aq_12_8_fx, M+1 );
    st->old_Es_pred_fx = Es_pred;

    st->tcxltp_third_last_pitch = st->tcxltp_second_last_pitch;
    st->tcxltp_second_last_pitch = st->old_fpitch;
    st->old_fpitch = pitch_buf[(st->L_frame_fx/L_SUBFR) - 1];
    move32();


    return;
}

