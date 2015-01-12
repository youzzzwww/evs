/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_dec_fx.h"
#include "stl.h"


/*****************************************************
    calcGainc calculates st->lp_gainc
******************************************************/
static void calcGainc(Word16* exc, Word16 Q_exc, Word32 old_fpitch, Word16 L_subfr, Word32 lp_gainp, Word32* lp_gainc)
{
    Word32 L_c ;
    Word16 tmp16, tmp16_2, tmp16_3, tmp_e, tmp2_e, tmp_loop, i;
    Word32 L_acc, L_tmp;


    L_acc = L_deposit_l(0);
    L_c = L_deposit_l(0);
    Overflow = 0;
    Carry = 0;

    tmp16 = round_fx(old_fpitch);/*Q0*/
    tmp_loop = shl(L_subfr,1);
    BASOP_SATURATE_WARNING_OFF
    tmp16_2 = round_fx(L_shl(lp_gainp,2)); /*Q31->Q15, no severe saturation, because st->lp_gainp here is [0,1]*/
    BASOP_SATURATE_WARNING_ON

    FOR ( i=0; i< tmp_loop; i++ )
    {
        /*st->lp_gainc += ( exc[i-2*L_subfr] - st->Mode2_lp_gainp * exc[i-2*L_subfr-(int)(st->old_fpitch+0.5f)] ) *
                          ( exc[i-2*L_subfr] - st->Mode2_lp_gainp * exc[i-2*L_subfr-(int)(st->old_fpitch+0.5f)] );*/
        tmp16_3 = sub
                  (
                      exc[i-2*L_subfr]      /*Q1*/,
                      mult_r(tmp16_2       /*Q15*/, exc[i-2*L_subfr-tmp16]/*Q1*/)/*Q1*/
                  );
        L_acc = L_macNs(L_acc,tmp16_3,tmp16_3); /*Q3*/
        Overflow = 0;
        L_c = L_macNs(L_c,0,0); /*Accumulate Carrys*/
        Carry = 0;
    }
    L_tmp = norm_llQ31(L_c,L_acc,&tmp_e);/*Q3,norm,tmp_e*/
    tmp_e = add(tmp_e,31-(add(shl(Q_exc,1),1))); /*L_tmp is Q31, now*/
    tmp16 = BASOP_Util_Divide3216_Scale(L_tmp/*Q31,norm,tmp_e*/,shl(L_subfr,1)/*Q15,15*/,&tmp2_e)/*Q15,tmp2_e+tmp_e-15*/;
    tmp_e = sub(add(tmp2_e,tmp_e), 15);

    IF (tmp16 != 0)
    {
        tmp16 = Sqrt16(tmp16,&tmp_e); /*Q15,norm,tmp_e*/
    }
    *lp_gainc = L_shl(L_deposit_l(tmp16),add(tmp_e,1)); /*15Q16*/

}

static void calcGainc2(Word16 *exc, Word16 Q_exc, Word16 L_subfr, Word32* lp_gainc)
{
    Word16 i, cnt, tmp16 , tmp_e, tmp2_e;
    Word32 L_c, L_acc, L_tmp;


    Carry = 0;
    Overflow = 0;

    L_c = L_deposit_l(0);
    L_acc = L_deposit_l(0);

    cnt = shl(L_subfr,1);

    FOR (i=0; i < cnt; i++)
    {
        /* *gainc += ( exc[i-2*L_subfr] ) * ( exc[i-2*L_subfr]); */
        L_acc = L_macNs(L_acc, exc[i-2*L_subfr] /*Q1*/, exc[i-2*L_subfr] /*Q1*/); /*Q3*/
        Overflow = 0;
        L_c = L_macNs(L_c,0,0); /* Accumulate Carrys */
        Carry = 0;
    }

    L_tmp = norm_llQ31(L_c,L_acc,&tmp_e); /*Q3,norm,tmp_e*/
    tmp_e = add(tmp_e,31-(add(shl(Q_exc,1),1)));              /*L_tmp is Q31, now*/
    tmp16 = BASOP_Util_Divide3216_Scale(L_tmp/*Q31,norm,tmp_e*/,shl(L_subfr,1)/*Q15,15*/,&tmp2_e)/*Q15,tmp2_e+tmp_e-15*/;
    tmp_e = sub(add(tmp2_e,tmp_e), 15);

    IF ( tmp16 != 0 )
    {
        tmp16 = Sqrt16(tmp16,&tmp_e);     /*Q15,norm,tmp_e*/
    }
    *lp_gainc = L_shl(L_deposit_l(tmp16),add(tmp_e,1)); /*15Q16*/                   move32();

}

/******************************************************

con_tcx

\brief main function in time domain TCX concealment

*******************************************************/

void con_tcx(
    Word16 coder_type,            /*!< input: ACELP coder type          *//*Q0 */
    Word16 synth[],               /*!< i/o:   synth[]                   *//*Q0 */
    Word16 stab_fac,              /*!< input: stability of isf          *//*Q15*/
    Decoder_State_fx *st)
{
    Word16  i, s, c, L_frame, L_subfr, fLowPassFilter, T0;
    Word16  n, mem_syn_r_size_old, mem_syn_r_size_new;
    Word16  *noise;
    Word16  mem_syn[M], *syn;
    Word16  *exc, buf[OLD_EXC_SIZE_DEC+L_FRAME_MAX+L_FRAME_MAX/NB_SUBFR+1+L_FRAME_MAX/2];
    Word16  pre_emph_buf;
    Word16  hp_filt[L_FIR_FER2];
    Word16  alpha;
    Word16  tmp_deemph, gain, gainCNG, gain_inov;
    Word16  *pt_exc, *pt1_exc;
    Word16  Tc, tmpSeed;
    Word16  fUseExtrapolatedPitch;
    Word16  *ana_window;
    Word16  r_h[M+1], A_local[M+1], mem, r_l[M+1];
    PWord16  const *w;
    Word16  W1, W2, W12;
    Word16 Q_r;
    Word16 tmp16, tmp16_2, tmp_loop,  tmp_e, gain_tmp;
    Word16 gainCNG_e, noise_e, gain_inov_e ;/*Exponents for gainCNG, noise, gain_inov*/
    Word16 Q_syn; /*Q format of temporary synthesis buffer syn*/
    Word32 L_tmp, L_tmp2, step32_tmp;
    Word32 predPitchLag, pitch_buf[NB_SUBFR16k], step32, gain32;
    Word16  extrapolationFailed;
    Word16  gainSynthDeemph;
    Word16  gainSynthDeemph_e;
    Word32 old_pitch_buf[2*NB_SUBFR16k+2];
    Word16 Q_exc, new_Q, exp_scale;
    Word16 offset;

    /* inits */
    alpha = 0;
    move16();
    fUseExtrapolatedPitch = 0;
    move16();
    extrapolationFailed = 1;
    move16();

    noise_e = 0;
    move16();
    Q_syn = -1; /*Q format of temporary synthesis buffer syn*/                           move16();
    offset = 0;
    move16();

    /* Framing parameters */
    L_frame = st->L_frameTCX;
    move16();
    /* L_subfr = st->L_frameTCX/st->nb_subfr */
    L_subfr = mult_r(st->L_frameTCX,div_s(1,st->nb_subfr));
    assert( L_subfr == st->L_frameTCX/st->nb_subfr );
    move32();
    w = st->tcx_cfg.tcx_mdct_windowFB; /*pointer - no need to instrument*/
    W1 = st->tcx_cfg.tcx_mdct_window_lengthFB;
    move16();
    W2 = shr(st->tcx_cfg.tcx_mdct_window_lengthFB,1);
    W12 = shr(W1,1);

    /* take the previous frame last pitch */
    Tc = round_fx(st->old_fpitchFB);

    set16_fx(buf,0,shr(sizeof(buf),1)); /* initialize buf with 0 */

    c = BASOP_Util_Divide1616_Scale(
            s_max(L_frame,st->L_frame_fx),
            st->L_frame_fx,
            &s
        );

    FOR (i=0; i < (2*NB_SUBFR16k+2); i++)
    {
        old_pitch_buf[i] = L_shl(Mpy_32_16_1(st->old_pitch_buf_fx[i],c),s);
        move32();
    }

    /* set excitation memory*/
    exc = buf+OLD_EXC_SIZE_DEC;
    tmp_deemph = shl(synth[-1],0);

    pre_emph_buf = synth[-1];
    IF ( sub( st->nbLostCmpt , 1 ) == 0 )
    {
        /* apply pre-emphasis to the signal */
        mem = synth[-((shr(L_frame,1))+st->pit_max_TCX+M+M)-1];
        Q_exc = E_UTIL_f_preemph3(&(synth[-((shr(L_frame,1))+st->pit_max_TCX+2*M)]), st->preemph_fac, add(add(shr(L_frame,1),st->pit_max_TCX),shl(M,1)), &mem,1);
        st->Mode2_lp_gainc = L_deposit_l(0);

        st->Mode2_lp_gainp = get_gain2( synth-2*L_subfr, synth-2*L_subfr-Tc, shl(L_subfr,1) );
        move32();

        st->Mode2_lp_gainp = L_max(st->Mode2_lp_gainp,0);
        st->Mode2_lp_gainp = L_min(st->Mode2_lp_gainp,FL2WORD32_SCALE(1.0f,31-16));
        st->Mode2_lp_gainp = L_shl(st->Mode2_lp_gainp, 13);

        ana_window = buf;
        ham_cos_window(ana_window, mult(L_frame,FL2WORD16(0.75f)), shr(L_frame,2));

        /* Autocorrelation */
        autocorr_fx(&(synth[-L_frame-1]), M, r_h ,r_l , &Q_r , L_frame, ana_window, 0, 0);

        /* Lag windowing */
        lag_wind( r_h,r_l, M, st->sr_core, LAGW_STRONG );

        /* Levinson Durbin */
        E_LPC_lev_dur(r_h, r_l, A_local, NULL, M, NULL);

        /* copy for multiple frame loss */
        Copy(A_local, st->old_Aq_12_8_fx, M+1);

        /* Residu */
        assert((2*L_subfr+Tc+1+M) <= st->old_synth_lenFB);

        BASOP_SATURATE_WARNING_OFF /*saturation possible in case of spiky synthesis*/
        Residu3_fx(
            A_local,
            &(synth[-(2*L_subfr+Tc+1+M)]),            /*Qx   = Q0*/
            &(exc[-(2*L_subfr+Tc+1+M)]),              /*Qx+1 = Q1*/
            add(add(add(shl(L_subfr,1),Tc),1),M),
            1);
        BASOP_SATURATE_WARNING_ON
    }
    ELSE
    {
        /* apply pre-emphasis to the signal */
        mem = synth[-L_frame-1];
        Q_exc = E_UTIL_f_preemph3(&(synth[-L_frame]), st->preemph_fac, L_frame, &mem, 1);
        Copy(st->old_Aq_12_8_fx, A_local, M+1);

        offset = shr(L_frame,1);
        IF(sub(st->last_good_fx, UNVOICED_TRANSITION) >= 0 )
        {
            tmp16 = s_max(Tc - shr(L_frame,1), 0);
            Copy_Scale_sig(st->old_excFB_fx, &(exc[-tmp16]), offset+tmp16, Q_exc-st->Q_exc);
        }
        ELSE {
            Copy_Scale_sig(st->old_excFB_fx, &(exc[-2*L_subfr]), 2*L_subfr+offset, Q_exc-st->Q_exc);
        }
    }

    /*-----------------------------------------------------------------*
     * PLC: Construct the harmonic part of excitation
     *-----------------------------------------------------------------*/

    test();
    test();
    IF( sub(st->last_good_fx, UNVOICED_CLAS) > 0  && !(sub(st->last_good_fx, UNVOICED_TRANSITION) == 0 && sub(coder_type, GENERIC) == 0) )
    {
        IF ( sub(st->nbLostCmpt,1) == 0 )
        {
            calcGainc( exc, Q_exc, st->old_fpitchFB, L_subfr, st->Mode2_lp_gainp, &(st->Mode2_lp_gainc));
        }

        tmp16 = 0;
        move16();
        if (L_sub(st->sr_core , 25600) > 0)
        {
            tmp16 = 1;
            move16();
        }

        test();
        test();
        test();
        IF( ( sub(st->nbLostCmpt,1) == 0 ) && sub(st->rf_frame_type,RF_TCXFD) >= 0 && sub(st->rf_frame_type,RF_TCXTD2) <= 0 && st->use_partial_copy )
        {
            Word32 tcxltp_pitch_tmp = L_add(L_deposit_h(st->tcxltp_pitch_int), L_shl(L_deposit_l(div_s(st->tcxltp_pitch_fr,st->pit_res_max)),1)); /*15Q16*/
            Word16 scale_tmp = mult_r(st->L_frameTCX, getInvFrameLen(st->L_frame_fx)); /*getInvFrameLen()->9Q6*/
            Word16 tmp_shift = norm_s(scale_tmp);
            predPitchLag = L_shl(Mpy_32_16_1(tcxltp_pitch_tmp, shl(scale_tmp, tmp_shift)), sub(9, tmp_shift));

            T0 = round_fx(predPitchLag);

            test();
            test();
            test();
            if (   (T0 > 0)
                    && (sub(T0,Tc) != 0)
                    && (L_sub(L_deposit_h(abs_s(sub(T0,Tc)))/*Q16*/ , L_mult(FL2WORD16(.15f)/*Q15*/,Tc/*Q0*/) /*Q16*/ ) < 0)
               )
            {
                fUseExtrapolatedPitch = 1;
                move16();
            }
        }
        ELSE
        {

            pitch_pred_linear_fit(
                st->nbLostCmpt,
                st->last_good_fx,
                old_pitch_buf,
                &(st->old_fpitchFB),
                &predPitchLag,
                st->pit_min_TCX,
                st->pit_max_TCX,
                st->mem_pitch_gain,
                tmp16,
                st->plc_use_future_lag,
                &extrapolationFailed,
                st->nb_subfr
            );

            T0 = round_fx(predPitchLag);
            test();
            test();
            test();
            if (   (T0 > 0)
            && (sub(T0,Tc) != 0)
            && (L_sub(L_deposit_h(abs_s(sub(T0,Tc)))/*Q16*/ , L_mult(FL2WORD16(.15f)/*Q15*/,Tc/*Q0*/) /*Q16*/ ) < 0)
            && (extrapolationFailed == 0)
               )
            {
                fUseExtrapolatedPitch = 1;
                move16();
            }
        }


        fLowPassFilter = 0;
        move16();
        pt_exc = exc + offset;
        pt1_exc = pt_exc - Tc;

        if (fUseExtrapolatedPitch != 0)
        {
            pt_exc = buf;
        }
        test();
        IF( sub(st->stab_fac_fx ,FL2WORD16(1.f)) < 0 && sub(st->nbLostCmpt , 1) == 0 )
        {
            /* pitch cycle is first low-pass filtered */

            IF (L_sub(st->output_Fs_fx , 16000) <= 0)
            {
                FOR( i=0 ; i< Tc; i++ )
                {
                    move16();
                    *pt_exc++ = mac_r(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(
                                                            L_mult(FL2WORD16( 0.0053f), pt1_exc[-5]),
                                                            FL2WORD16( 0.0000f), pt1_exc[-4]),
                                                        FL2WORD16(-0.0440f), pt1_exc[-3]),
                                                        FL2WORD16( 0.0000f), pt1_exc[-2]),
                                                        FL2WORD16( 0.2637f), pt1_exc[-1]),
                                                        FL2WORD16( 0.5500f), pt1_exc[0] ),
                                                        FL2WORD16( 0.2637f), pt1_exc[1] ),
                                                        FL2WORD16( 0.0000f), pt1_exc[2] ),
                                                  FL2WORD16(-0.0440f), pt1_exc[3] ),
                                            FL2WORD16( 0.0000f), pt1_exc[4] ),
                                      FL2WORD16( 0.0053f), pt1_exc[5] );
                    pt1_exc++;
                }
            }
            ELSE IF (L_sub(st->output_Fs_fx , 25600) == 0)
            {
                FOR( i=0 ; i< Tc; i++ )
                {
                    move16();
                    *pt_exc++ = mac_r(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(
                                                            L_mult(FL2WORD16( 0.0056f), pt1_exc[-5]),
                                                            FL2WORD16( 0.0000f), pt1_exc[-4]),
                                                        FL2WORD16(-0.0464f), pt1_exc[-3]),
                                                        FL2WORD16( 0.0000f), pt1_exc[-2]),
                                                        FL2WORD16( 0.2783f), pt1_exc[-1]),
                                                        FL2WORD16( 0.5250f), pt1_exc[0] ),
                                                        FL2WORD16( 0.2783f), pt1_exc[1] ),
                                                        FL2WORD16( 0.0000f), pt1_exc[2] ),
                                                  FL2WORD16(-0.0464f), pt1_exc[3] ),
                                            FL2WORD16( 0.0000f), pt1_exc[4] ),
                                      FL2WORD16( 0.0056f), pt1_exc[5] );
                    pt1_exc++;
                }
            }
            ELSE /*(st->output_Fs_fx >= 32000)*/
            {
                FOR( i=0 ; i< Tc; i++ )
                {
                    move16();
                    *pt_exc++ = mac_r(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(L_mac(
                        L_mult(FL2WORD16(-0.0053f), pt1_exc[-5]),
                        FL2WORD16(-0.0037f), pt1_exc[-4]),
                    FL2WORD16(-0.0140f), pt1_exc[-3]),
                    FL2WORD16( 0.0180f), pt1_exc[-2]),
                    FL2WORD16( 0.2668f), pt1_exc[-1]),
                    FL2WORD16( 0.4991f), pt1_exc[0] ),
                    FL2WORD16( 0.2668f), pt1_exc[1] ),
                    FL2WORD16( 0.0180f), pt1_exc[2] ),
                    FL2WORD16(-0.0140f), pt1_exc[3] ),
                    FL2WORD16(-0.0037f), pt1_exc[4] ),
                    FL2WORD16(-0.0053f), pt1_exc[5] );
                    pt1_exc++;
                }
            }

            fLowPassFilter = 1;
            move16();
        }
        ELSE
        {
            /* copy the first pitch cycle without low-pass filtering */
            FOR( i=0 ; i < Tc; i++ )
            {
                *pt_exc++ = *pt1_exc++;
                move16();
            }
            fLowPassFilter = 1;
            move16();
        }

        if (fUseExtrapolatedPitch != 0)
        {
            pt1_exc = buf;
        }
        tmp16 =  add(sub(L_frame,imult1616(fLowPassFilter,Tc)),L_subfr);
        FOR (i = 0; i < tmp16; i++)
        {
            *pt_exc++ = *pt1_exc++;
            move16();
        }

        IF (fUseExtrapolatedPitch != 0)
        {
            get_subframe_pitch(st->nb_subfr,
                               st->old_fpitch,
                               /* predPitchLag * L_frame/st->L_frame, */
                               L_shr(Mpy_32_16_1(predPitchLag/*Q16*/,
                                                 mult_r(st->L_frame_fx/*Q0*/,
                                                         getInvFrameLen(L_frame)/*Q21*/
                                                       )/*Q6*/
                                                )/*Q7*/,
                                     7-16)/*Q16*/,
                               pitch_buf);

            PulseResynchronization(buf, exc, L_frame, st->nb_subfr, st->old_fpitchFB, predPitchLag);
        }
        ELSE
        {
            set32_fx( pitch_buf, st->old_fpitch, st->nb_subfr);
        }

        IF ( sub(st->nbLostCmpt , 1) == 0 )
        {
            pt_exc = exc+L_frame;
            if (T0 == 0)
            {
                pt1_exc = pt_exc -  Tc ;
            }
            if (T0 != 0)
            {
                pt1_exc = pt_exc - T0;
            }

            tmp_loop = shr(L_frame,1);
            FOR (i = 0; i < tmp_loop; i++)
            {
                *pt_exc++ = *pt1_exc++;
                move16();
            }
        }

        if (fUseExtrapolatedPitch != 0)
        {
            st->old_fpitchFB = predPitchLag;
            move16();
        }
        st->bpf_gain_param = 0;
        move16();

        /* PLC: calculate damping factor */
        alpha = Damping_fact(coder_type, st->nbLostCmpt, st->last_good_fx, stab_fac, &(st->Mode2_lp_gainp), 0);/*Q14*/

        IF ( sub(st->nbLostCmpt , 1) == 0 )
        {
            st->cummulative_damping = FL2WORD16(1.f);
            move16();
        }
        ELSE
        {
            {
                st->cummulative_damping = shl(mult_r(st->cummulative_damping/*Q15*/,alpha/*Q14*/),1)/*Q15*/;
            }
        }

        gain32 = L_add(FL2WORD32(1.f), 0); /*Q31*/
        gain = FL2WORD16(1.f);   /*Q15*/                                                   move16();
        if( sub(st->rf_frame_type, RF_TCXTD1) == 0 && sub(st->use_partial_copy, 1) == 0 )
        {
            gain32 = FL2WORD32(0.5f);
            gain = FL2WORD16(0.5f);
        }

        /*step = (1.0f/(L_frame+(L_frame/2))) * (gain - alpha);*/
        tmp16 = shr(imult1616(3,L_frame),1);
        tmp_e = norm_s(tmp16);
        tmp16 = shl(tmp16,tmp_e);
        tmp16 = div_s(FL2WORD16_SCALE(1.f,1),tmp16);/*Q15,1+tmp_e-15*/
        tmp16_2 = sub(shr(gain,1),alpha)/*Q14*/;
        step32 = L_shl(L_mult(tmp16,tmp16_2)/*Q30, 1+tmp_e-15*/,add(1-14,tmp_e))/*Q31*/;

        /* PLC: Apply fade out */
        tmp_loop = shr(imult1616(L_frame,3),1);
        FOR ( i=offset; i < tmp_loop; i++ )
        {
            exc[i] = mult_r(exc[i],round_fx(gain32))/*Q1*/;
            move16();
            gain32 = L_sub(gain32 , step32);
        }

        /* update old exc without random part */
        offset = s_max(Tc - shr(L_frame,1), 0);
        Copy(exc+L_frame-offset, st->old_excFB_fx, shr(L_frame,1)+offset);
        /* copy old_exc as 16kHz for acelp decoding */
        IF ( sub(st->nbLostCmpt, 1) == 0 )
        {
            lerp(exc - shr(L_frame,1), st->old_exc_fx, L_EXC_MEM_DEC, add(L_frame, shr(L_frame,1)));
        }
        ELSE
        {
            Copy(st->old_exc_fx+L_FRAME16k, st->old_exc_fx, L_FRAME16k/2);
            lerp(exc, st->old_exc_fx+L_FRAME16k/2, L_FRAME16k, L_frame);
        }
        st->Q_exc = Q_exc;
    }
    ELSE
    {
        /* No harmonic part */
        set16_fx(&exc[0], 0, add(L_frame,shr(L_frame,1)));
        IF ( sub(st->nbLostCmpt , 1) == 0 )
        {
            calcGainc2(&exc[0], Q_exc, L_subfr, &(st->Mode2_lp_gainc));
        }
        set32_fx( pitch_buf, L_deposit_h(L_SUBFR), st->nb_subfr);
        /* PLC: calculate damping factor */
        alpha = Damping_fact(coder_type, st->nbLostCmpt, st->last_good_fx, stab_fac, &(st->Mode2_lp_gainp), 0);/*Q14*/
    }

    /*-----------------------------------------------------------------*
     * Construct the random part of excitation
     *-----------------------------------------------------------------*/
    tmpSeed = st->seed_acelp;
    move16();
    noise = buf;
    noise_e = 1;/*set exponent of noise to 1*/                                           move16();

    tmp_loop = add(L_frame,L_FIR_FER2-1);
    FOR (i = 0; i < tmp_loop; i++)
    {
        tmpSeed = own_random2_fx(tmpSeed);
        noise[i] = shr(tmpSeed,noise_e);
        move16();
    }
    st->seed_acelp = tmpSeed;
    move16();

    tmp_loop = add(add(L_frame,shr(L_frame,1)) ,shl(L_FIR_FER2,1));
    FOR ( ; i < tmp_loop; i++)
    {
        tmpSeed = own_random2_fx(tmpSeed);
        noise[i] = shr(tmpSeed,noise_e);
        move16();
    }
    test();
    IF (sub(st->last_good_fx , VOICED_CLAS)==0 || sub(st->last_good_fx , ONSET)==0)
    {
        tmp16 = FL2WORD16(0.6f);
        move16();
        if ( L_sub(st->output_Fs_fx,16000) <= 0 )
        {
            tmp16 = FL2WORD16(0.2f);
            move16();
        }

        mem = noise[0];
        move16();
        preemph_copy_fx(&noise[1], &noise[1], tmp16, L_frame+(L_frame/2)+L_FIR_FER2, &mem);
    }
    /* high rate filter tuning */
    IF ( L_sub(st->output_Fs_fx,16000) <= 0 )
    {
        FOR( i=0; i< L_FIR_FER2; i++ )
        {
            hp_filt[i] = h_high3_16[i];
            move16();
        }
    }
    ELSE IF ( L_sub(st->output_Fs_fx,25600) == 0 )
    {
        FOR( i=0; i< L_FIR_FER2; i++ )
        {
            hp_filt[i] = h_high3_25_6[i];
            move16();
        }
    }
    ELSE /*(st->output_Fs_fx==32000)*/
    {
        FOR( i=0; i< L_FIR_FER2; i++ )
        {
            hp_filt[i] = h_high3_32[i];
            move16();
        }
    }
    IF ( sub(st->nbLostCmpt,1) == 0 )
    {
        highPassFiltering(st->last_good_fx, add(add(L_frame, shr(L_frame,1)),L_FIR_FER2), noise, hp_filt, L_FIR_FER2);
    }
    ELSE
    {
        IF(sub( st->last_good_fx , UNVOICED_TRANSITION) > 0 )
        {
            tmp_loop = add(add(L_frame,shr(L_frame,1)),L_FIR_FER2);
            gain_tmp = negate(add(-32768,st->cummulative_damping));/*Q15*/
            FOR( i=0 ; i < tmp_loop; i++ )
            {
                /*noise[i] = (1-st->cummulative_damping)*noise[i] + st->cummulative_damping*dot_product(&noise[i], hp_filt, L_FIR_FER2);*/
                move16();
                L_tmp2 = L_mac(0,      noise[i+L_FIR_FER2-11], hp_filt[0+L_FIR_FER2-11]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-10], hp_filt[0+L_FIR_FER2-10]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-9], hp_filt[0+L_FIR_FER2-9]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-8], hp_filt[0+L_FIR_FER2-8]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-7], hp_filt[0+L_FIR_FER2-7]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-6], hp_filt[0+L_FIR_FER2-6]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-5], hp_filt[0+L_FIR_FER2-5]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-4], hp_filt[0+L_FIR_FER2-4]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-3], hp_filt[0+L_FIR_FER2-3]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-2], hp_filt[0+L_FIR_FER2-2]);
                L_tmp2 = L_mac(L_tmp2, noise[i+L_FIR_FER2-1], hp_filt[0+L_FIR_FER2-1]);

                L_tmp2 = Mpy_32_16_1(L_tmp2, st->cummulative_damping/*Q15*/);/*Q0, noise_e*/
                noise[i] = mac_r(L_tmp2, gain_tmp,noise[i]);/*Q15, noise_e*/
            }
        }
    }

    /* PLC: [TCX: Fade-out] retrieve background level */
    tmp16 = 32767;
    move16();
    gainSynthDeemph = getLevelSynDeemph(&(tmp16),
                                        A_local,
                                        M,
                                        shr(L_frame,2),
                                        st->preemph_fac,
                                        1,
                                        &gainSynthDeemph_e);
    IF (0 != st->tcxonly)
    {
        /* gainCNG = st->conCngLevelBackgroundTrace/gainSynthDeemph; */
        BASOP_Util_Divide_MantExp(st->conCngLevelBackgroundTrace,
                                  st->conCngLevelBackgroundTrace_e,
                                  gainSynthDeemph, gainSynthDeemph_e,
                                  &gainCNG, &gainCNG_e);
    }
    ELSE
    {
        /* gainCNG = st->cngTDLevel/gainSynthDeemph; */
        BASOP_Util_Divide_MantExp(st->cngTDLevel,
        st->cngTDLevel_e,
        gainSynthDeemph, gainSynthDeemph_e,
        &gainCNG, &gainCNG_e);
    }

    gain32 = L_add(st->Mode2_lp_gainc, 0); /* start-of-the-frame gain - Q16*/
    if( sub(st->rf_frame_type, RF_TCXTD1) == 0 && sub(st->use_partial_copy, 1) == 0 )
    {
        gain32 = Mpy_32_16_1(gain32, FL2WORD16(0.7f));
    }
    L_tmp  = L_shl(gain32,1);

    IF (L_sub(L_shl(L_deposit_h(gainCNG),sub(gainCNG_e,31-16)/*Q16*/) , L_tmp) > 0)
    {
        gainCNG_e = sub(15+1,norm_l(L_tmp));
        gainCNG = extract_l(L_shr(L_tmp,gainCNG_e));/*Q15,gainCNG_e*/
        gainCNG_e = sub(gainCNG_e,1);
    }

    /* st->Mode2_lp_gainc = alpha * (st->Mode2_lp_gainc) + (1.0f - alpha) * gainCNG;*/  /* end-of-the-frame gain */

    L_tmp = Mpy_32_16_1(st->Mode2_lp_gainc,alpha)/*Q15*/;
    L_tmp2 = L_mult(sub(FL2WORD16_SCALE(1.f,1),alpha)/*Q14*/,gainCNG/*Q15,gainCNG_e*/);/*Q30,gainCNG_e*/
    st->Mode2_lp_gainc = BASOP_Util_Add_Mant32Exp(L_tmp,31-15,L_tmp2,add(gainCNG_e,31-30),&tmp_e);/*Q31*/   move32();
    st->Mode2_lp_gainc = L_shl(st->Mode2_lp_gainc,sub(tmp_e,31-16));
    move32();

    /* PLC: [TCX: Fade-out] Linearly attenuate the gain through the frame */
    /*step = (1.0f/L_frame) * (gain - (st->Mode2_lp_gainc));*/
    L_tmp = L_sub(gain32,st->Mode2_lp_gainc);/*Q16*/
    tmp_e = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp,tmp_e);/*Q16,-tmp_e*/
    step32 = Mpy_32_16_1(L_tmp/*Q16,-tmp_e*/,getInvFrameLen(L_frame)/*W16Q21*/);/*Q22,-tmp_e*/
    step32 = L_shl(step32,sub((25-22),tmp_e));/*Q25*/

    pt_exc = noise + L_FIR_FER2/2;

    /*gain_inov = 1.0f / (float)sqrt( dot_product( pt_exc, pt_exc, L_frame ) / L_frame + 0.01f );*//* normalize energy */
    L_tmp = Dot_productSq16HQ(0,pt_exc/*Q0,15+1*/,L_frame,&tmp_e)/*Q31,tmp_e+16+16*/;
    L_tmp = Mpy_32_16_1(L_tmp,getInvFrameLen(L_frame)/*W16Q21*/)/*W32Q37,tmp_e+16+16*//*Q5,tmp_e*/;
    tmp_e = add(tmp_e,31-5);/*-->Q31*/
    gain_inov = round_fx(ISqrt32(L_tmp,&tmp_e));/*Q15,tmp_e*/
    gain_inov_e = tmp_e;
    move16();
    test();
    test();
    IF (sub(st->last_good_fx , UNVOICED_CLAS) == 0  && sub(coder_type , UNVOICED) != 0)
    {
        gain_inov = mult_r(gain_inov,FL2WORD16(0.8f));
    }
    ELSE IF (!( sub(st->last_good_fx , UNVOICED_CLAS) == 0 || sub(st->last_good_fx , UNVOICED_TRANSITION) == 0 ))
    {
        /*gain_inov *= (1.1f- 0.75*st->lp_gainp);*/
        L_tmp = Mpy_32_16_1(L_sub(FL2WORD32_SCALE(1.1f,31-29), Mpy_32_16_1(st->Mode2_lp_gainp,24576))/*Q29*/,gain_inov/*Q15,gain_inov_e*/);/*Q29,gain_inov_e*/
        tmp_e = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp,tmp_e);
        gain_inov_e = add(sub(gain_inov_e,tmp_e),31-29);/*->Q31*/
        gain_inov = round_fx(L_tmp);/*Q15,gain_inov_e*/
    }

    st->Mode2_lp_gainp = L_shr(L_deposit_h(alpha/*Q14*/)/*Q14+16*/,1);/*Q29*/
    pt_exc = noise;            /* non-causal ringing of the FIR filter   */

    tmp_e = norm_l(gain32);
    tmp_e = sub(tmp_e,5); /*5 Bit additional Headroom for the gain - should be enough*/
    gain32 = L_shl(gain32,tmp_e);/*Q16,-tmp_e*/
    L_tmp = Mpy_32_16_1(gain32/*Q16,-tmp_e*/, gain_inov/*Q15,gain_inov_e*/)/*Q16,gain_inov_e-tmp_e*/;

    gain_tmp = round_fx(L_tmp);/*Q0, gain_inov_e-tmp_e*/

    FOR( i=0 ; i< L_FIR_FER2/2; i++ )
    {
        *pt_exc = mult_r(*pt_exc,gain_tmp);/*Q-15,noise_e+gain_inov_e-tmp_e*/              move16();
        pt_exc++;
    }
    tmp16 = add(L_frame,L_FIR_FER2/2);
    step32_tmp = L_shl(step32/*Q25*/,sub(tmp_e,(25-16)));/*Q16,-tmp_e*/
    FOR( i=0 ; i< tmp16; i++ )     /* Actual filtered random part of excitation */
    {
        *pt_exc = mult_r(*pt_exc, gain_tmp);
        move16();
        pt_exc++;
        gain32 = L_sub(gain32/*Q16,-tmp_e*/,step32_tmp);/*Q16,-tmp_e*/
        gain_tmp = mult_r(round_fx(gain32/*Q16,-tmp_e*/)/*Q0*/, gain_inov/*Q15,gain_inov_e*/)/*Q0,gain_inov_e-tmp_e*/;
    }
    tmp16 = shr(L_frame,1);
    FOR( i=0 ; i< tmp16; i++ )   /* causal ringing of the FIR filter */
    {
        *pt_exc = mult_r(*pt_exc , gain_tmp);
        move16();
        pt_exc++;
    }
    noise_e = add(sub(add(noise_e,gain_inov_e),tmp_e),15);/*--> noise is Q0, noise_e*/
    /*buf[0;L_FIR_FER2 + L_Frame + L_Frame/2] Q0, noise_e*/

    /*-----------------------------------------------------------------*
     * Construct the total excitation
     *-----------------------------------------------------------------*/

    IF( sub(st->last_good_fx , UNVOICED_TRANSITION) >= 0 )
    {
        tmp16 = add(L_frame,shr(L_frame,1));
        FOR( i=0 ; i< tmp16; i++ )
        {
            exc[i] = add(exc[i] , shl(noise[i+(L_FIR_FER2/2)],Q_exc+noise_e));/*Q1*/       move16();
        }
    }
    ELSE
    {
        bufferCopyFx(noise+L_FIR_FER2/2, exc, add(L_frame , shr(L_frame,1)),0/*Q_noise*/, noise_e, Q_exc, 0/*exc_e*/);
        Copy(exc+L_frame-2*L_subfr, st->old_excFB_fx, 2*L_subfr+shr(L_frame,1));
        /* copy old_exc as 16kHz for acelp decoding */
        IF ( sub(st->nbLostCmpt, 1) == 0 )
        {
            lerp(exc, st->old_exc_fx, L_EXC_MEM_DEC, add(L_frame, shr(L_frame,1)));
        }
        ELSE {
            Copy(st->old_exc_fx+L_FRAME16k, st->old_exc_fx, L_FRAME16k/2);
            lerp(exc, st->old_exc_fx+L_FRAME16k/2, L_FRAME16k, L_frame);
        }
        st->Q_exc = Q_exc;
    }
    /*buf[0;L_FIR_FER2 + L_Frame + L_Frame/2] Q0, noise_e*/
    /*buf[OLD_EXC_SIZE_DEC;3/2 L_frame] Q1*/

    /* Update Pitch Lag memory */
    Copy32( &st->old_pitch_buf_fx[st->nb_subfr], st->old_pitch_buf_fx, st->nb_subfr );
    Copy32( pitch_buf, &st->old_pitch_buf_fx[st->nb_subfr], st->nb_subfr );

    /*----------------------------------------------------------*
     * - compute the synthesis speech                           *
     *----------------------------------------------------------*/

    syn = buf + M;
    Copy(synth-M, buf, M);

    new_Q = sub(Q_exc, 3);
    new_Q = s_max(new_Q, -1);

    tmp16 = s_min(new_Q, st->prev_Q_syn);
    st->prev_Q_syn = new_Q;
    move16();

    exp_scale = sub(tmp16, Q_exc-1);
    Q_syn = tmp16;
    move16();

    Copy_Scale_sig(buf, mem_syn, M, exp_scale);

    tmp_deemph = shl(tmp_deemph,Q_syn);
    st->Q_syn = Q_syn;

    /*buf[OLD_EXC_SIZE_DEC;3/2 L_frame] Q1: exc*/
    /*buf[0;M] Q0: mem_syn*/

    E_UTIL_synthesis(
        sub(Q_exc, Q_syn),
        A_local,
        &exc[0],
        &syn[0],
        add(L_frame, shr(L_frame,1)),
        mem_syn,
        1,
        M);

    /*buf[OLD_EXC_SIZE_DEC;3/2 L_frame-1] Q1: exc*/
    /*buf[0;M-1] Q0: mem_syn*/
    /*buf[M;3/2 L_frame-1] Q-1: syn*/

    n = extract_h(L_mult(L_frame,FL2WORD16((float)N_ZERO_MDCT_NS/(float)FRAME_SIZE_NS)));

    /* update ACELP synthesis memory */
    mem_syn_r_size_old = shr(L_frame,4);         /* replace 1.25/20.0 by shr(4) */
    /* copy mem_syn as 16kHz */
    mem_syn_r_size_new = shr(L_FRAME16k,4);      /* replace 1.25/20.0 by shr(4) */

    Copy(syn+L_frame-L_SYN_MEM, st->mem_syn_r, L_SYN_MEM);
    lerp(st->mem_syn_r+L_SYN_MEM-mem_syn_r_size_old, st->mem_syn_r+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old);
    Copy(st->mem_syn_r+L_SYN_MEM-M, st->mem_syn2_fx, M);

    /* Deemphasis and output synth and ZIR */
    deemph_fx(syn, st->preemph_fac, add(L_frame,shr(L_frame,1)), &tmp_deemph);
    bufferCopyFx(syn+L_frame-M-1, st->syn, 1+M, Q_syn, 0, 0, 0);

    /* Update TCX-LTP */                                                                 test();
    if ( (st->tcxltp != 0) && (st->Mode2_lp_gainp == 0) )
    {
        st->tcxltp_last_gain_unmodified = 0;
        move16();
    }


    Copy(syn+L_frame-n, st->old_out_fx, sub(L_frame,n));

    FOR (i=0; i < W12; i++)
    {
        st->old_out_fx[i+n] = round_fx(Mpy_32_16_1(L_mult(w[i].v.re,w[i].v.re),st->old_out_fx[i+n]));
    }
    FOR ( ; i < W1; i++)
    {
        st->old_out_fx[i+n] = round_fx(Mpy_32_16_1(L_mult(w[W12-1-(i-W12)].v.im,w[W12-1-(i-W12)].v.im),st->old_out_fx[i+n]));
    }

    set16_fx(&st->old_out_fx[W1+n], 0, n);

    st->Q_old_wtda = Q_syn;
    move16();

    /* As long as there is no synth scaling factor introduced, which
    is given to the outside, there might occur overflows here */
    BASOP_SATURATE_WARNING_OFF
    bufferCopyFx(syn, synth, L_frame, Q_syn, 0, 0, 0);
    BASOP_SATURATE_WARNING_ON

    Copy_Scale_sig(syn+L_frame, st->syn_OverlFB, shr(L_frame,1), negate(Q_syn));

    bufferCopyFx(syn+L_frame, st->syn_Overl_TDACFB, shr(L_frame,1),Q_syn,0,-1,0);

    FOR (i=0 ; i < W12 ; i++)
    {
        buf[i] = mult_r(st->syn_Overl_TDACFB[i] , w[i].v.re);
        move16();
    }
    FOR (    ; i <W1 ; i++)
    {
        buf[i] = mult_r(st->syn_Overl_TDACFB[i],w[W12-1-(i-W12)].v.im);
        move16();
    }


    FOR (i=0; i<W2; i++)
    {
        st->syn_Overl_TDACFB[i] = add(buf[i] , buf[W1-1-i]);
        move16();
    }

    FOR (i=0; i<W2; i++)
    {
        st->syn_Overl_TDACFB[W2+i] = add(buf[W2+i] , buf[W1-1-W2-i]);
        move16();
    }

    FOR (i=0 ; i < W12 ; i++)
    {
        st->syn_Overl_TDACFB[i] = mult_r(st->syn_Overl_TDACFB[i],w[i].v.re);
        move16();
    }
    FOR (    ; i <W1 ; i++)
    {
        st->syn_Overl_TDACFB[i] = mult_r(st->syn_Overl_TDACFB[i],w[W12-1-(i-W12)].v.im);
        move16();
    }

    st->tcx_cfg.tcx_curr_overlap_mode = FULL_OVERLAP;

    synth[-1] = pre_emph_buf;
    move16();

    /* update memory for low band */
    lerp( syn+L_frame-shr(L_frame, 1), st->old_syn_Overl, shr(st->L_frame_fx, 1), shr(L_frame, 1) );
    Scale_sig(st->old_syn_Overl, shr(st->L_frame_fx, 1), sub(-1, Q_syn));
    lerp( st->syn_OverlFB, st->syn_Overl, shr(st->L_frame_fx, 1), shr(L_frame, 1) );
    lerp( st->syn_Overl_TDACFB, st->syn_Overl_TDAC, shr(st->L_frame_fx, 1), shr(L_frame, 1) );
    lerp( st->old_out_fx, st->old_out_LB_fx, st->L_frame_fx, L_frame );

    /* copy total excitation exc2 as 16kHz for acelp mode1 decoding */
    lerp(exc, st->old_exc2_fx, L_EXC_MEM, L_frame);
    lerp(syn, st->old_syn2_fx, L_EXC_MEM, L_frame);

    move32();
    L_tmp = L_mult(round_fx(L_shl(pitch_buf[st->nb_subfr-1]/*15Q16*/,4))/*Q4*/, round_fx(L_shl(L_mult(L_FRAME16k, getInvFrameLen(L_frame) )/*Q22*/,7) /*Q29*/)/*Q13*/) /*Q18*/  ;
    st->bfi_pitch_fx/*Q6*/ = round_fx(L_shr(L_tmp,18-6-16));
    move16();
    st->bfi_pitch_frame_fx = L_FRAME16k;
    move16();

    return;
}

