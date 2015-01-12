/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "options.h"
#include "prot_fx.h"

extern const Word16 T_DIV_L_Frame[];/*0Q15 * 2^-7 */


static void IMDCT(Word32 *x, Word16 x_e,
                  Word16 *old_syn_overl,
                  Word16 *syn_Overl_TDAC,
                  Word16 *xn_buf,
                  const Word16  *tcx_aldo_window_1,
                  const PWord16 *tcx_aldo_window_1_trunc,
                  const PWord16 *tcx_aldo_window_2,
                  const PWord16 *tcx_mdct_window_half,
                  const PWord16 *tcx_mdct_window_minimum,
                  const PWord16 *tcx_mdct_window_trans,
                  Word16 tcx_mdct_window_half_length,
                  Word16 tcx_mdct_window_min_length,
                  Word16 index,
                  Word16 left_rect,
                  Word16 tcx_offset,
                  Word16 overlap,
                  Word16 L_frame,
                  Word16 L_frameTCX,
                  Word16 L_spec_TCX5,
                  Word16 L_frame_glob,
                  Word16 Txnq_offset,
                  Word16 frame_cnt,
                  Word16 bfi,
                  Word16 *old_out,
                  Word16 *Q_old_wtda,
                  Decoder_State_fx *st
                  ,Word16 fullbandScale
                  ,Word16 *acelp_zir
                 );


void decoder_tcx(
    TCX_config *tcx_cfg,  /* input: configuration of TCX               */
    Word16 prm[],         /* input:  parameters                        */
    Word16 A[],           /* input:  coefficients NxAz[M+1]            */
    Word16 Aind[],        /* input: frame-independent coefficients Az[M+1] */
    Word16 L_frame_glob,  /* input:  frame length                      */
    Word16 L_frameTCX_glob,
    Word16 L_spec,
    Word16 synth[],        /* in/out: synth[-M-LFAC..L_frame]          */
    Word16 synthFB[],
    Decoder_State_fx *st,
    Word16 coder_type,     /* input : coder type                      */
    Word16 bfi,            /* input:  Bad frame indicator             */
    Word16 frame_cnt,      /* input: frame counter in the super frame */
    Word16 stab_fac,       /* input: stability of isf (1Q14)          */
    Word16 past_core_mode
)
{
    Word16 i, index, L_frame, tcx_offset;
    Word16 L_frameTCX, tcx_offsetFB;
    Word16 firstLine;
    Word16 gain_tcx, gain_tcx_e, fac_ns;
    Word16 Ap[M+2];
    Word32 x[N_MAX];
    Word16 x_e;
    Word16 *xn_buf;
    Word16 xn_bufFB[L_MDCT_OVLP_MAX+L_FRAME_PLUS+L_MDCT_OVLP_MAX];
    Word32 xn_buf32[N_MAX];
    Word16 overlap, Txnq_offset;
    Word16 overlapFB, Txnq_offsetFB;
    Word16 noiseFillingSize;
    Word16 noiseTransWidth;
    Word16 tnsSize; /* number of tns parameters put into prm   */
    Word8 fUseTns; /* flag that is set if TNS data is present */
    STnsData tnsData;
    Word16 left_rect;
    Word16 gainlpc2[FDNS_NPTS];
    Word16 gainlpc2_e[FDNS_NPTS];
    Word16 noiseTiltFactor;
    Word16 nf_seed;
    Word16 tmp1, tmp2, s, *tmpP16;
    Word8 tmp8;
    Word32 tmp32;
    Word16 gamma1;
    Word16 gamma;
    Word16 gainCompensate, gainCompensate_e;
    Word16 h1[L_FRAME_MAX/4+1];
    Word16 mem[M];
    Word16 temp_concealment_method = 0;
    Word16 arith_bits, signaling_bits;
    Word16 *prm_ltp, *prm_tns, *prm_hm, *prm_sqQ, *prm_target;
    Word16*pInfoTCXNoise;
    Word16 acelp_zir[L_FRAME_MAX/2];
    Word16 noise_filling_index;
    Word16 infoIGFStartLine;

    prm_target = NULL;       /* just to suppress MSVC warnigs */


    x_e = 0;          /* to avoid compilation warnings */
    nf_seed = 0;      /* to avoid compilation warnings */



    /* Overlay xn_buf32 with xn_buf */
    xn_buf = (Word16 *) xn_buf32;

    noiseTransWidth = MIN_NOISE_FILLING_HOLE;
    move16();
    tnsSize = 0;
    move16();
    fUseTns = 0;
    move16();
    gainCompensate = 32768/2;
    move16();
    gainCompensate_e = 1;
    move16();
    FOR (i=0 ; i < (L_MDCT_OVLP_MAX+L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2; i++)
    {
        xn_buf32[i] = L_deposit_l(0);
    }


    /* Init lengths */

    overlap    = tcx_cfg->tcx_mdct_window_length;
    move16();
    Txnq_offset=0;
    move16();
    overlapFB  = tcx_cfg->tcx_mdct_window_lengthFB;
    move16();
    Txnq_offsetFB = 0;
    move16();
    /* Modified the overlap to the delay in case of short blocks*/
    tcx_offset = tcx_cfg->tcx_offset;
    move16();
    tcx_offsetFB = tcx_cfg->tcx_offsetFB;
    move16();
    gamma1     = st->gamma;
    move16();

    if (st->enableTcxLpc != 0)
    {
        gamma1 = 0x7FFF;
        move16();
    }

    IF (bfi != 0)
    {
        /* PLC: [TCX: Memory update]
         * PLC: Init buffers */

        L_frame = L_frame_glob;
        move16();
        L_frameTCX = L_frameTCX_glob;
        move16();
        IF (st->L_frame_past != 0)
        {
            L_frame = st->L_frame_past;
            move16();
            L_frameTCX = st->L_frameTCX_past;
            move16();
        }

        left_rect = st->prev_widow_left_rect;
        move16();

        IF (left_rect != 0)
        {
            tcx_offset = tcx_cfg->lfacNext;
            move16();
            tcx_offsetFB = tcx_cfg->lfacNextFB;
            move16();
        }
    }
    ELSE
    {
        test();
        IF ( frame_cnt == 0 && st->last_core_fx == ACELP_CORE )
        {
            if (st->prev_bfi_fx == 0)
            {
                tcx_cfg->last_aldo = 0;
                move16();
            }

            /* if past frame is ACELP */
            L_frame = add(L_frame_glob, tcx_offset);
            L_frameTCX = add(L_frameTCX_glob, tcx_offsetFB);
            L_spec = add(L_spec, shr(st->tcx_cfg.tcx_coded_lines, 2));
            tcx_offset = 0;
            move16();
            tcx_offsetFB = 0;
            move16();
            IF (tcx_cfg->lfacNext<0)
            {
                L_frame = sub(L_frame, tcx_cfg->lfacNext);
                tcx_offset = tcx_cfg->lfacNext;
                move16();
                L_frameTCX = sub(L_frameTCX, tcx_cfg->lfacNextFB);
                tcx_offsetFB = tcx_cfg->lfacNextFB;
                move16();
            }
            left_rect = 1;
            move16();
            st->prev_widow_left_rect = 1;
            move16();
        }
        ELSE{

            L_frame = L_frame_glob;
            move16();
            L_frameTCX = L_frameTCX_glob;
            move16();
            left_rect = 0;
            move16();
            st->prev_widow_left_rect = 0;
            move16();
        }

        st->L_frame_past = L_frame;
        move16();
        st->L_frameTCX_past = L_frameTCX;
        move16();
    }

    test();
    IF ( (sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly))
    {
        IGFDecUpdateInfo(
            &st->hIGFDec,
            IGF_GRID_LB_SHORT
        );
    }
    ELSE
    {
        test();
        test();
        IF ((sub(st->last_core_fx, ACELP_CORE) == 0) || (left_rect && st->bfi_fx))
        {
            IGFDecUpdateInfo(
                &st->hIGFDec,
                IGF_GRID_LB_TRAN
            );
        }
        ELSE
        {
            IGFDecUpdateInfo(
                &st->hIGFDec,
                IGF_GRID_LB_NORM
            );
        }
    }

    IF (0 == st->igf)
    {
        IF (st->narrowBand == 0)
        {
            infoIGFStartLine = L_frame;
            move16();
        }
        ELSE
        {
            infoIGFStartLine  = L_frameTCX;
            move16();
        }
    }
    ELSE
    {
        infoIGFStartLine = s_min(st->hIGFDec.infoIGFStartLine,L_frameTCX);
        move16();
    }

    noiseFillingSize = L_spec;
    move16();
    if (st->igf != 0)
    {
        noiseFillingSize = st->hIGFDec.infoIGFStartLine;
        move16();
    }


    prm_ltp = &prm[1+NOISE_FILL_RANGES];
    move16();
    prm_tns = prm_ltp + LTPSIZE;
    move16();

    /*-----------------------------------------------------------*
     * Read TCX parameters                                       *
     *-----------------------------------------------------------*/

    index = 0;
    move16();

    IF (bfi == 0)
    {

        index = prm[0];
        move16();

        /* read noise level (fac_ns) */

        noise_filling_index = prm[1];
        move16();

        fac_ns = extract_l(L_shr(L_mult0(noise_filling_index, 0x6000), NBITS_NOISE_FILL_LEVEL));
    }
    ELSE
    {
        fac_ns = 0;
        move16();
    }

    /* read TNS data */
    test();
    IF ((bfi == 0) && (tcx_cfg->fIsTNSAllowed != 0))
    {
        cast16();
        fUseTns = (Word8)DecodeTnsData(tcx_cfg->pCurrentTnsConfig,
                                       prm_tns,
                                       &tnsSize,
                                       &tnsData);
    }
    ELSE
    {
        fUseTns = 0;
        move16();
    }

    prm_hm = prm_tns + tnsSize;
    move16();
    prm_sqQ = prm_hm + NPRM_CTX_HM;
    move16();

    /*-----------------------------------------------------------*
     * Spectrum data                                             *
     *-----------------------------------------------------------*/

    IF (bfi == 0)
    {

        /*-----------------------------------------------------------*
         * Context HM                                        *
         *-----------------------------------------------------------*/
        test();
        test();
        IF(tcx_cfg->ctx_hm != 0 && ( (st->last_core_fx != ACELP_CORE) || (frame_cnt > 0) ) )
        {
            st->last_ctx_hm_enabled = prm_hm[0];
            move16();
            {
                FOR (i = 0; i < L_spec; i++)    /* no context harmonic model, copy MDCT coefficients to x */
                {

                    x[i] = L_mult(prm_sqQ[i],
                                  1 << (30 - SPEC_EXP_DEC));
                    move32();
                }
            }
            x_e = SPEC_EXP_DEC;
            move16();
        }
        ELSE  /* tcx_cfg->ctx_hm == 0 */
        {

            IF (st->tcx_lpc_shaped_ari != 0)  /* low rates: new arithmetic coder */
            {
                prm_target = prm_sqQ;
                move16();
                prm_sqQ = prm_target + 1;
                move16();

                tmp8 = 1;
                move16();
                if (sub(st->last_core_fx, ACELP_CORE) == 0)
                {
                    tmp8 = 0;
                    move16();
                }

                tcx_arith_decode_envelope(
                    x, &x_e,
                    L_frame,
                    L_spec,
                    st,
                    Aind,
                    *prm_target,
                    prm_sqQ,
                    tmp8,
                    prm_hm, /* HM parameter area */
                    st->tcx_hm_LtpPitchLag,
                    &arith_bits,
                    &signaling_bits,
                    &nf_seed
                    ,(st->bwidth_fx > WB)?1:0
                );

                st->resQBits[frame_cnt] = sub(*prm_target, arith_bits);
                move16();
            }
            ELSE  /* TCX-only: old arithmetic coder */
            {
                FOR (i = 0; i < L_spec; i++)
                {

                    x[i] = L_mult(prm_sqQ[i],
                    1 << (30 - SPEC_EXP_DEC));
                    move32();
                }

                set32_fx(x+L_spec, 0, sub(L_frameTCX, L_spec));

                x_e = SPEC_EXP_DEC;
                move16();
            }

        }  /* else of if tcx_cfg->ctx_hm */
        tmp1 = s_max(L_frame, L_frameTCX);
        set32_fx(x+L_spec, 0, sub(tmp1, L_spec));


        /*-----------------------------------------------------------*
         * adaptive low frequency deemphasis.                        *
         *-----------------------------------------------------------*/

        weight_a_fx(A, Ap, gamma1, M);

        lpc2mdct(Ap, M, NULL, NULL, gainlpc2, gainlpc2_e);


        /* initialize LF deemphasis factors in xn_buf */
        tmp1 = s_max(L_spec, L_frameTCX);
        set16_fx(xn_buf, 0x4000, tmp1);

        IF (st->tcxonly == 0)
        {
            AdaptLowFreqDeemph( x, x_e, st->tcx_lpc_shaped_ari, gainlpc2, gainlpc2_e,
                                L_frame, xn_buf /* LF deemphasis factors */ );
        }
    }

    /* Global Gain */
    st->damping = 0;

    IF(bfi==0)
    {
        /*-----------------------------------------------------------*
         * Compute global gain                                       *
         *-----------------------------------------------------------*/

        tmp32 = L_shl(L_mult0(index, 0x797D), 7); /* 6Q25; 0x797D -> log2(10)/28 (Q18) */
        gain_tcx_e = add(extract_l(L_shr(tmp32, 25)), 1); /* get exponent */
        gain_tcx = round_fx(BASOP_Util_InvLog2(L_or(tmp32, 0xFE000000)));

        tmp1 = mult_r(shl(L_spec, 5), FL2WORD16(128.f/NORM_MDCT_FACTOR));
        s = 15-5-7;
        tmp1 = ISqrt16(tmp1, &s);

        gain_tcx = mult(gain_tcx, tmp1);
        gain_tcx_e = add(gain_tcx_e, s);

        st->old_gaintcx_bfi = gain_tcx;
        move16();
        st->old_gaintcx_bfi_e = gain_tcx_e;
        move16();

        st->cummulative_damping_tcx = FL2WORD16(1.0f);
        move16();
    }
    ELSE /* bfi = 1 */
    {
        /* PLC: [TCX: Fade-out]
         * derivation of damping factor */


        IF( st->use_partial_copy != 0 )
        {
            IF( sub(st->rf_frame_type, RF_TCXFD) == 0 )
            {
                tmp32 = L_shl(L_mult0(st->old_gaintcx_bfi, 0x797D), 7); /* 6Q25; 0x797D -> log2(10)/28 (Q18) */
                gain_tcx_e = add(extract_l(L_shr(tmp32, 25)), 1); /* get exponent */
                gain_tcx = round_fx(BASOP_Util_InvLog2(L_or(tmp32, 0xFE000000)));

                tmp1 = mult_r(shl(L_spec, 5), FL2WORD16(128.f/NORM_MDCT_FACTOR));
                s = 15-5-7;
                tmp1 = ISqrt16(tmp1, &s);

                gain_tcx = mult(gain_tcx, tmp1);
                gain_tcx_e = add(gain_tcx_e, s);

                st->old_gaintcx_bfi = gain_tcx;
                move16();
                st->old_gaintcx_bfi_e = gain_tcx_e;
                move16();
            }
            ELSE
            {
                gain_tcx = st->old_gaintcx_bfi;
                move16();
                gain_tcx_e = st->old_gaintcx_bfi_e;
                move16();
            }

            st->damping = FL2WORD16_SCALE(1.f,1); /*Q14*/                   move16();
        }
        ELSE
        {
            st->damping = Damping_fact(coder_type, st->nbLostCmpt, st->last_good_fx, stab_fac, &(st->Mode2_lp_gainp), st->last_core_fx);
            gain_tcx = st->old_gaintcx_bfi;
            move16();
            gain_tcx_e = st->old_gaintcx_bfi_e;
            move16();
        }

        st->cummulative_damping_tcx = shl(mult(st->cummulative_damping_tcx,st->damping),1);/*shl(Q15*Q14,1)=shl(Q14,1) = Q15*/

    }
    {

        IF(bfi)
        {
            gamma = gamma1;
            move16();
            if (st->envWeighted)
            {
                gamma = st->gamma;
                move16();
            }

            /* PLC: [TCX: Fade-out]
             * PLC: invert LPC weighting in case of PLC */
            IF (st->enableTcxLpc != 0)
            {
                gamma = add(mult_r(st->cummulative_damping_tcx,sub(st->gamma, FL2WORD16(1.0f))), FL2WORD16(1.0f));
            }
            ELSE
            {
                gamma = add(mult_r(st->cummulative_damping_tcx,sub(gamma1, FL2WORD16(1.0f))), FL2WORD16(1.0f));
            }
            weight_a_fx(A, Ap, gamma, M);

            lpc2mdct(Ap, M, NULL, NULL, gainlpc2, gainlpc2_e);

        }
        tmp2 = 0;
        move16();

        set16_fx(h1, 0, add(L_SUBFR,1));
        set16_fx(mem, 0, M);
        h1[0] = 32768/32;
        move16();
        E_UTIL_synthesis(0,Ap, h1, h1, L_SUBFR, mem, 0, M );
        deemph_fx(h1, st->preemph_fac, L_SUBFR, &tmp2);
        /* impulse response level = gain introduced by synthesis+deemphasis */
        test();
        IF (bfi==0)
        {
            /* st->last_gain_syn_deemph = (float)sqrt(dot_product( h1, h1, L_SUBFR)); */
            tmp32 = Dot_productSq16HQ( 0, h1, L_SUBFR, &st->last_gain_syn_deemph_e)/*Q15, st->last_gain_syn_deemph_e*/;
            st->last_gain_syn_deemph_e = add(st->last_gain_syn_deemph_e,10/*scaling of h1[0] and E_UTIL_synthesis * 2*/);
            tmp32 = Sqrt32(tmp32,&st->last_gain_syn_deemph_e);
            st->last_gain_syn_deemph = round_fx(tmp32);
            /*for avoiding compiler warnings*/
            st->gainHelper    = 32768/2;
            move16();
            st->gainHelper_e    = 1;
            move16();
            st->stepCompensate  = 0;
            move16();
            st->stepCompensate_e  = 0;
            move16();
        }
        /* not instrumenting the additional test() here seems to be common practice */
        ELSE IF (sub(TCX_20_CORE, st->core_fx)== 0 || sub(frame_cnt, 1) == 0 )
        {
            /* gainCompensate = st->last_gain_syn_deemph/(float)sqrt(dot_product( h1, h1, L_SUBFR)); */
            tmp32 = Dot_productSq16HQ( 0, h1, L_SUBFR, &gainCompensate_e)/*Q15, gainCompensate_e*/;
            gainCompensate_e = add(gainCompensate_e,10/*scaling of h1[0] and E_UTIL:synthesis*/);
            gainCompensate = round_fx(Sqrt32(tmp32,&gainCompensate_e))/*Q15, gainCompensate_e*/;
            BASOP_Util_Divide_MantExp (   st->last_gain_syn_deemph,
                                          st->last_gain_syn_deemph_e,
                                          gainCompensate,
                                          gainCompensate_e,
                                          &gainCompensate,
                                          &gainCompensate_e);

            tmp1 = T_DIV_L_Frame[L_shl(L_mac(-28000,st->L_frame_fx,95),1-15)];

            IF (sub(st->nbLostCmpt,1)==0)
            {
                /* stepCompensate = (1.f - gainCompensate)/st->L_frame_fx; */
                st->stepCompensate_e = BASOP_Util_Add_MantExp(
                                           tmp1,
                                           -7,
                                           negate(mult(gainCompensate,tmp1)),
                                           add(-7,gainCompensate_e),
                                           &st->stepCompensate);

                st->gainHelper = 32768/2;
                move16();
                st->gainHelper_e = 1;
                move16();
            }
            ELSE
            {
                /* stepCompensate = (st->last_concealed_gain_syn_deemph - gainCompensate)/st->L_frame_fx; */
                st->stepCompensate_e = BASOP_Util_Add_MantExp(
                    mult(tmp1,st->last_concealed_gain_syn_deemph),
                    add(-7, st->last_concealed_gain_syn_deemph_e),
                    negate(mult(tmp1,gainCompensate)),
                    add(-7, gainCompensate_e),
                    &st->stepCompensate);
                move16();
                move16();
                st->gainHelper = st->last_concealed_gain_syn_deemph;
                st->gainHelper_e = st->last_concealed_gain_syn_deemph_e;
            }
            move16();
            move16();
            st->last_concealed_gain_syn_deemph = gainCompensate;
            st->last_concealed_gain_syn_deemph_e = gainCompensate_e;
        }

    }


    /*-----------------------------------------------------------*
     * Residual inv. Q.                                          *
     *-----------------------------------------------------------*/
    test();
    IF ((bfi == 0) && (tcx_cfg->resq != 0))
    {

        IF (st->tcx_lpc_shaped_ari != 0)    /* new arithmetic coder */
        {

            Word16 *prm_resq;

            prm_resq = prm_sqQ
                       + *prm_target /* = targetBits */
                       - st->resQBits[frame_cnt];

            i = tcx_ari_res_invQ_spec(x, x_e, L_spec,
                                      prm_resq,
                                      st->resQBits[frame_cnt],
                                      0,
                                      tcx_cfg->sq_rounding,
                                      xn_buf /* LF deemphasis factors */ );
        }
        ELSE   /* old arithmetic coder */
        {
            i = tcx_res_invQ_gain(&gain_tcx, &gain_tcx_e,
            &prm_sqQ[L_spec],
            st->resQBits[frame_cnt]);

            tmpP16 = xn_buf;
            if (st->tcxonly != 0) tmpP16 = NULL;

            tcx_res_invQ_spec(x, x_e, L_spec,
            &prm_sqQ[L_spec],
            st->resQBits[frame_cnt],
            i,
            tcx_cfg->sq_rounding,
            tmpP16 /* LF deemphasis factors */ );
        }
    }
    test();
    IF (bfi == 0 && st->tcxonly != 0)
    {
        test();
        test();
        IF (st->tcxltp && (st->tcxltp_gain > 0) && !fUseTns)
        {

            PsychAdaptLowFreqDeemph(x, gainlpc2, gainlpc2_e, NULL);
        }
    }

    /* for FAC */

    test();
    IF (bfi == 0 && st->tcxonly == 0)
    {


        /* Replication of ACELP formant enhancement for low rates */
        IF (L_sub(st->total_brate_fx, ACELP_13k20) < 0 || st->rf_flag != 0 )
        {
            tcxFormantEnhancement(xn_buf, gainlpc2, gainlpc2_e, x, &x_e, L_frame, L_frameTCX);
        }
    }

    /*-----------------------------------------------------------*
     * Add gain to the lpc gains                                 *
     *-----------------------------------------------------------*/

    if(st->VAD==0 )
    {
        gain_tcx = mult_r(gain_tcx, tcx_cfg->na_scale);
    }

    i = norm_s(gain_tcx);
    gain_tcx = shl(gain_tcx, i);
    gain_tcx_e = sub(gain_tcx_e, i);
    FOR (i = 0; i < FDNS_NPTS; i++)
    {
        gainlpc2[i] = mult_r(gainlpc2[i], gain_tcx);
        move16();
    }


    /*-----------------------------------------------------------*
     * Noise filling.                                            *
     *-----------------------------------------------------------*/

    test();
    IF (bfi==0 && (fac_ns > 0))
    {

        tmp1 = 0;
        move16();
        test();
        if ( sub(st->bits_frame, 256) >= 0 && st->rf_flag == 0 )
        {
            tmp1 = 1;
            move16();
        }

        firstLine = tcxGetNoiseFillingTilt( A, M, L_frame, tmp1, &noiseTiltFactor );

        IF (st->tcxonly != 0)
        {
            tmp1 = 0;
            move16();
            test();
            test();
            if ((tcx_cfg->ctx_hm != 0) && (st->last_core_fx != ACELP_CORE) && (st->last_ctx_hm_enabled != 0))
            {
                tmp1 = FL2WORD16(0.3125f);
                move16();
            }
            noiseTransWidth = HOLE_SIZE_FROM_LTP(s_max(st->tcxltp_gain, tmp1));

            if (sub(L_frame, shr(st->L_frame_fx, 1)) == 0)
            {
                noiseTransWidth = 3;  /* minimum transition fading for noise filling in TCX-10 */   move16();
            }
        }


        IF (st->tcx_lpc_shaped_ari == 0)   /* old arithmetic coder */
        {
            /* noise filling seed */
            IF(bfi == 0)
            {
                tmp32 = L_deposit_l(0);
                FOR (i = 0; i < L_spec; i++)
                {
                    tmp32 = L_macNs(tmp32, abs_s(prm_sqQ[i]), i);
                }
                nf_seed = extract_l(tmp32);
            }
        }

        tmp1 = nf_seed;
        move16();
        if (bfi != 0)
        {
            tmp1 = st->seed_tcx_plc;
            move16();
        }

        pInfoTCXNoise = NULL;
        if (st->igf)
        {

            pInfoTCXNoise = st->hIGFDec.infoTCXNoise;
            move16();
        }
        tcx_noise_filling(x, x_e,
                          tmp1 /* seed */,
                          firstLine,
                          noiseFillingSize,
                          noiseTransWidth,
                          L_frame,
                          noiseTiltFactor,
                          fac_ns,
                          pInfoTCXNoise
                         );
        if (bfi == 0)
        {
            st->seed_tcx_plc = tmp1;
            move16();
        }
    }

    IF (st->enablePlcWaveadjust)
    {
        IF (bfi)
        {
            IF (sub(st->nbLostCmpt, 1) == 0)
            {
                st->plcInfo.concealment_method = TCX_NONTONAL;
                move16();
                /* tonal/non-tonal decision */
                test();
                test();
                IF (0 == sub(st->plcInfo.Transient[0],1)
                    && 0 == sub(st->plcInfo.Transient[1], 1)
                    && 0 == sub(st->plcInfo.Transient[2], 1))
                {
                    Word16 sum_word16 = 0;
                    move16();

                    FOR (i = 9; i >= 0; i--)
                    {
                        sum_word16 = add(sum_word16, st->plcInfo.TCX_Tonality[i]);
                    }

                    if(sub(sum_word16, 6) >= 0 )
                    {
                        st->plcInfo.concealment_method = TCX_TONAL;
                        move16();
                    }
                }

                if(st->tonal_mdct_plc_active)
                {
                    st->plcInfo.concealment_method = TCX_TONAL;
                    move16();
                }
            }

            if (sub(L_frameTCX, st->L_frameTCX) > 0)
            {
                st->plcInfo.concealment_method = TCX_TONAL;
                move16();
            }

            temp_concealment_method = st->plcInfo.concealment_method;
            move16();

            if (0 == sub(st->core_fx, TCX_10_CORE))
            {
                temp_concealment_method = TCX_TONAL;
                move16();
            }
        }
        /* get the starting location of the subframe in the frame */
        IF (0 ==sub(st->core_fx, TCX_10_CORE))
        {
            st->plcInfo.subframe_fx =extract_l( L_mult0(frame_cnt,L_frameTCX_glob));
        }
    }

    /* PLC: [TCX: Tonal Concealment] */
    /* PLC: [TCX: Fade-out]
     * PLC: Fade out to white noise */

    IF (bfi == 0)
    {
        TonalMDCTConceal_SaveFreqSignal(&st->tonalMDCTconceal,
                                        x, x_e,
                                        L_frameTCX,
                                        L_frame,
                                        gainlpc2, gainlpc2_e,
                                        gain_tcx_e);
    }
    ELSE
    {
        test();
        IF( !st->enablePlcWaveadjust || L_sub(temp_concealment_method, TCX_TONAL) == 0 )
        {
            Word16 f, tmp, noiseTiltFactor;

            /* set f to 1 to not fade out */
            /* set f to 0 to immediately switch to white noise */
            f = st->cummulative_damping_tcx;
            move16();
            if (0 != st->tcxonly)
            {
                f = FL2WORD16(1.0f);
                move16();
            }

            noiseTiltFactor = FL2WORD16(1.0f);
            move16();

            tmp = 0;
            move16();
            test();
            IF( sub(st->bits_frame, 256) >= 0 && st->rf_flag == 0)
            {
                tmp = 1;
                move16();
            }

            tcxGetNoiseFillingTilt( A, M, L_frame, tmp, &noiseTiltFactor );

            TonalMDCTConceal_InsertNoise(&st->tonalMDCTconceal,
                                         x,
                                         &x_e,
                                         st->tonal_mdct_plc_active,
                                         &st->seed_tcx_plc,
                                         noiseTiltFactor,
                                         f,
                                         infoIGFStartLine
                                        );
        }
    }


    IF (sub(L_spec, L_frame) < 0)
    {
        set32_fx(x+L_spec, 0, sub(L_frame,L_spec));
    }
    ELSE IF (sub(L_spec, L_frameTCX) > 0)
    {
        set32_fx(x+L_frameTCX, 0, sub(L_spec,L_frameTCX));
    }


    /*-----------------------------------------------------------*
     * Noise shaping in frequency domain (1/Wz)                  *
     *-----------------------------------------------------------*/
    test();
    IF(st->igf && ! bfi)
    {
        test();
        IF ( (sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly))
        {
            IGFDecCopyLPCFlatSpectrum(
                &st->hIGFDec,
                x,
                x_e,
                IGF_GRID_LB_SHORT
            );
        }
        ELSE
        {
            IF (sub(st->last_core_fx, ACELP_CORE) == 0)
            {
                IGFDecCopyLPCFlatSpectrum(
                    &st->hIGFDec,
                    x,
                    x_e,
                    IGF_GRID_LB_TRAN
                );
            }
            ELSE
            {
                IGFDecCopyLPCFlatSpectrum(
                    &st->hIGFDec,
                    x,
                    x_e,
                    IGF_GRID_LB_NORM
                );

            }
        }
    }

    /* LPC gains already available */
    test();
    test();
    IF(!st->enablePlcWaveadjust || !bfi || (L_sub(temp_concealment_method, TCX_TONAL) == 0))
    {
        x_e = add(x_e, gain_tcx_e);
        mdct_shaping(x, L_frame, gainlpc2, gainlpc2_e);
        IF ( bfi == 0 )
        {
            FOR (i = L_frame; i < L_spec; i++)
            {
                x[i] = L_shl(Mpy_32_16_1(x[i], gainlpc2[FDNS_NPTS-1]), gainlpc2_e[FDNS_NPTS-1]);
                move32();
            }
        }

        set32_fx(x+L_spec, 0, sub(L_frameTCX, L_spec));
        test();
        test();
        IF (( bfi != 0) && ( !st->enablePlcWaveadjust || L_sub(temp_concealment_method, TCX_TONAL) == 0 ))
        {
            scale_sig32(x+infoIGFStartLine, sub(L_spec, infoIGFStartLine), negate(gain_tcx_e));
        }
    }

    /* PLC: [TCX: Tonal Concealment] */
    IF (!bfi)
    {
        st->tonal_mdct_plc_active = 0;
        move16();
    }
    ELSE IF (st->tonal_mdct_plc_active)
    {
        test();
        IF( !st->enablePlcWaveadjust || temp_concealment_method )
        {
            TonalMDCTConceal_Apply(&st->tonalMDCTconceal, x, &x_e);
        }
    }

    tmp32 = L_deposit_h(0);
    if(st->tcxltp_last_gain_unmodified > 0)
    {
        tmp32 = L_add(st->old_fpitch, 0);
    }
    tmp8 = 0;
    move16();
    test();
    if(bfi && st->tonal_mdct_plc_active)
    {
        tmp8 = 1;
        move16();
    }
    TonalMDCTConceal_UpdateState(&st->tonalMDCTconceal,
                                 L_frameTCX,
                                 tmp32,
                                 bfi,
                                 tmp8);

    IF (st->enablePlcWaveadjust)
    {
        Word16 core;
        core = st->core_fx;
        move16();
        /* spectrum concealment */
        IF (bfi)
        {
            /* x_e =31-x_scale; */
            concealment_decode_fix(core, x, &x_e, &st->plcInfo);
        }
        /* update spectrum buffer, tonality flag, etc. */
        concealment_update_x(bfi, core, st->tonality_flag, x, &x_e, &st->plcInfo);
    }

    /*-----------------------------------------------------------*
    * IGF                                                       *
    *-----------------------------------------------------------*/
    test();
    test();
    IF (st->igf && !((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly)))
    {
        Word16 igfGridIdx;

        test();
        test();
        IF ((sub(st->last_core_fx, ACELP_CORE) == 0) || (left_rect && bfi))
        {
            /* packet loss after first TCX must be handled like transition frame */
            igfGridIdx = IGF_GRID_LB_TRAN;
        }
        ELSE
        {
            igfGridIdx = IGF_GRID_LB_NORM;
        }

        st->hIGFDec.igfData.igfInfo.nfSeed = extract_l(L_mac0(13849L, nf_seed, 31821));

        IGFDecApplyMono(
            &st->hIGFDec,
            x,
            &x_e,
            igfGridIdx,
            bfi
        );
    }
    test();
    test();
    IF (st->igf && ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly)))
    {
        st->hIGFDec.igfData.igfInfo.nfSeed = extract_l(L_mac0(13849L, nf_seed, 31821));
        IGFDecApplyMono(
            &st->hIGFDec,
            x,
            &x_e,
            IGF_GRID_LB_SHORT,
            bfi
        );
    }

    index = tcx_cfg->tcx_last_overlap_mode;  /* backup last TCX overlap mode */        move16();

    test();
    IF ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly != 0))
    {
        Word16 L = L_frameTCX;
        move16();

        test();
        test();
        test();
        if ((tcx_cfg->fIsTNSAllowed != 0 && fUseTns != 0 && bfi == 0) || (sub(L_spec, L_frameTCX) > 0))
        {
            L = L_spec;
            move16();
        }

        tcxInvertWindowGrouping(tcx_cfg,
                                xn_buf32,
                                x,
                                L,
                                fUseTns,
                                st->last_core_fx,
                                index,
                                frame_cnt,
                                bfi
                               );
    }

    /* normalize spectrum to minimize IMDCT noise */
    tmp1 = s_max(s_max(L_frame,L_frameTCX), L_spec);
    s = s_max(0, sub(getScaleFactor32(x, tmp1), 4)); /* Keep 4 bits headroom for TNS */
    Scale_sig32(x, tmp1, s);
    x_e = sub(x_e, s);

    /*-----------------------------------------------------------*
     * Temporal Noise Shaping Synthesis                          *
     *-----------------------------------------------------------*/

    test();
    test();
    IF ((tcx_cfg->fIsTNSAllowed != 0) && (fUseTns != 0) && bfi == 0 )
    {
        /* Apply TNS to get the reconstructed signal */
        test();
        test();
        SetTnsConfig(tcx_cfg, L_frame_glob == st->L_frame_fx, (st->last_core_fx == ACELP_CORE) && (frame_cnt == 0));

        ApplyTnsFilter(tcx_cfg->pCurrentTnsConfig, &tnsData, x, 0);

        test();
        IF ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (st->tcxonly != 0))
        {

            test();
            test();
            test();
            IF ((tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) ||
                ((tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP) && (frame_cnt == 0) && (index == 0))
               )
            {
                tmp1 = shr(tcx_cfg->tnsConfig[0][0].iFilterBorders[0], 1);
                /* undo rearrangement of LF sub-window lines for TNS synthesis filtering */
                IF (s_max(L_frameTCX, L_spec) > tcx_cfg->tnsConfig[0][0].iFilterBorders[0])
                {
                    tmp2 = shr(s_max(L_frameTCX, L_spec), 1);
                    Copy32(x+tmp1+8, x+tmp2+8, sub(tmp1, 8));
                    Copy32(x+8, x+tmp2, 8);
                    Copy32(x+16, x+8, sub(tmp1, 8));
                    set32_fx(x+tmp1, 0, sub(tmp2, tmp1));
                    set32_fx(x+tmp2+tmp1, 0, sub(tmp2, tmp1));
                }
                ELSE
                {
                    Copy32(x+8, xn_buf32, 8);
                    Copy32(x+16, x+8, sub(tmp1,8));
                    Copy32(xn_buf32, x+tmp1, 8);
                }

            }
        }
    }

    IF(st->igf)
    {
        test();
        IF(st->hIGFDec.flatteningTrigger != 0 && fUseTns == 0)
        {
            Word16 startLine = st->hIGFDec.infoIGFStartLine;
            Word16 endLine = st->hIGFDec.infoIGFStopLine;
            Word32 x_itf[N_MAX];
            Word16 j;

            const Word16* chk_sparse = st->hIGFDec.flag_sparse;
            const Word32* virtualSpec = st->hIGFDec.virtualSpec;

            const Word16 maxOrder = 8;
            Word16 curr_order = 0; /* not counted */
            Word16 A[ITF_MAX_FILTER_ORDER+1];
            Word16 Q_A;
            Word16 predictionGain = 0; /* not counted */

            move16();
            move16();

            move16();

            FOR (j = startLine; j < endLine; j++)
            {
                IF (sub(chk_sparse[j], 2) == 0)
                {
                    x_itf[j] = x[j];
                    move32();
                    x[j] = virtualSpec[j];
                    move32();
                }
            }

            ITF_Detect_fx(x, startLine, endLine, maxOrder, A, &Q_A, &predictionGain, &curr_order, shl(x_e, 1));

            s = getScaleFactor32(&x[startLine], sub(endLine, startLine));
            s = sub(s, 2);
            FOR(j = startLine; j < endLine; j++)
            {
                x[j] = L_shl(x[j], s);
                move32();
            }

            ITF_Apply_fx(x, startLine, endLine, A, Q_A, curr_order);

            FOR(j = startLine; j < endLine; j++)
            {
                x[j] = L_shr(x[j], s);
                move32();
            }

            FOR (j = startLine; j < endLine; j++)
            {
                if (sub(chk_sparse[j],2) == 0)
                {
                    x[j] = x_itf[j];
                    move32();
                }
            }
        }
    }

    /*-----------------------------------------------------------*
     * Compute inverse MDCT of x[].                              *
     *-----------------------------------------------------------*/


    Copy32(x, xn_buf32, s_max(s_max(L_frame,L_frameTCX), L_spec));

    IF(st->igf != 0)
    {
        set32_fx( xn_buf32+st->hIGFDec.infoIGFStartLine, 0, sub(L_frameTCX, st->hIGFDec.infoIGFStartLine) );
    }

    IMDCT(xn_buf32, x_e,
          st->old_syn_Overl,
          st->syn_Overl_TDAC,
          xn_buf,
          tcx_cfg->tcx_aldo_window_1,
          tcx_cfg->tcx_aldo_window_1_trunc,
          tcx_cfg->tcx_aldo_window_2,
          st->tcx_cfg.tcx_mdct_window_half,
          tcx_cfg->tcx_mdct_window_minimum,
          st->tcx_cfg.tcx_mdct_window_trans,
          st->tcx_cfg.tcx_mdct_window_half_length,
          tcx_cfg->tcx_mdct_window_min_length,
          index,
          left_rect,
          tcx_offset,
          overlap,
          L_frame,
          L_frameTCX,
          shr(s_max(L_frameTCX, L_spec), 1),
          L_frame_glob,
          Txnq_offset,
          frame_cnt,
          bfi,
          st->old_out_LB_fx,
          &st->Q_old_wtda_LB,
          st,
          0,
          acelp_zir
         );

    /* Generate additional comfort noise to mask potential coding artefacts */
    IF ( st->flag_cna != 0 )
    {
        generate_masking_noise_mdct (x,
                                     &x_e,
                                     st->hFdCngDec_fx->hFdCngCom,
                                     s_max(s_max(L_frame,L_frameTCX), L_spec)
                                    );
    }

    IMDCT(x, x_e,
          st->syn_OverlFB,
          st->syn_Overl_TDACFB,
          xn_bufFB,
          tcx_cfg->tcx_aldo_window_1_FB,
          tcx_cfg->tcx_aldo_window_1_FB_trunc,
          tcx_cfg->tcx_aldo_window_2_FB,
          tcx_cfg->tcx_mdct_window_halfFB,
          tcx_cfg->tcx_mdct_window_minimumFB,
          tcx_cfg->tcx_mdct_window_transFB,
          tcx_cfg->tcx_mdct_window_half_lengthFB,
          tcx_cfg->tcx_mdct_window_min_lengthFB,
          index,
          left_rect,
          tcx_offsetFB,
          overlapFB,
          L_frameTCX,
          L_frameTCX,
          shr(s_max(L_frameTCX, L_spec), 1),
          L_frameTCX_glob,
          Txnq_offsetFB,
          frame_cnt,
          bfi,
          st->old_out_fx,
          &st->Q_old_wtda,
          st,
          div_l(L_mult(FSCALE_DENOM, L_frameTCX_glob), L_frame_glob),
          acelp_zir
         );


    test();
    IF(st->enablePlcWaveadjust || L_sub(st->last_total_brate_fx, 48000) >= 0)
    {
        Word16 core;

        core = st->core_fx;
        move16();

        concealment_signal_tuning_fx(bfi, core, xn_bufFB, &st->plcInfo, st->nbLostCmpt, st->prev_bfi_fx,  /* waveform adjustment */
                                     st->tonalMDCTconceal.secondLastPcmOut,past_core_mode,st->tonalMDCTconceal.lastPcmOut, st);

        test();
        test();
        IF ((bfi || st->prev_bfi_fx) && st->plcInfo.concealment_method == TCX_NONTONAL)
        {
            lerp(xn_bufFB, xn_buf, L_frame_glob, L_frameTCX_glob);
        }

        /* update time-domain buffer */
        test();
        IF(bfi || sub(core, TCX_10_CORE) == 0)
        {

            IF (sub(core,TCX_20_CORE) == 0)
            {
                Copy(xn_bufFB, st->tonalMDCTconceal.lastPcmOut, L_frameTCX_glob);
            }
            ELSE
            {
                Copy(xn_bufFB, st->tonalMDCTconceal.lastPcmOut+L_mult0(frame_cnt,L_frameTCX_glob), L_frameTCX_glob);
            }
        }
    }


    /* PLC: [TCX: Tonal Concealment] */

    IF (!bfi)
    {
        {
            TonalMDCTConceal_SaveTimeSignal(&st->tonalMDCTconceal, xn_bufFB, L_frameTCX_glob);
        }
        st->second_last_tns_active = st->last_tns_active;
        st->last_tns_active = 0;
        move16();
        test();
        if ( tcx_cfg->fIsTNSAllowed && fUseTns)
        {
            st->last_tns_active = 1;
            move16();
        }

        st->tcxltp_third_last_pitch  = st->tcxltp_second_last_pitch;
        move32();
        st->tcxltp_second_last_pitch = st->old_fpitch;
        move32();
        st->old_fpitch = L_add(L_deposit_h(st->tcxltp_pitch_int), L_mult( st->tcxltp_pitch_fr, div_s(1,st->pit_res_max) /*Q16*/));
        st->old_fpitchFB = Mpy_32_16_1(st->old_fpitch/*Q16*/, mult_r(L_frameTCX/*Q0*/,getInvFrameLen(L_frame)/*Q21*/)/*Q6*/)/*Q7*/;
        st->old_fpitchFB = L_shr(st->old_fpitchFB,7-16);/*->Q16*/
    }


    /* Update old_syn_overl */
    IF (st->tcx_cfg.last_aldo == 0)
    {
        Copy(xn_buf+L_frame, st->old_syn_Overl+Txnq_offset, overlap);
        Copy(xn_bufFB+L_frameTCX, st->syn_OverlFB+Txnq_offsetFB, overlapFB);
        /* To be sure that sufficient overlap when going from TCX10 to TCX20 with asym windows */
        FOR(i=0; i<Txnq_offset; i++)
        {
            move16();
            move16();
            st->old_syn_Overl[i+overlap+Txnq_offset]=xn_buf[L_frame-i];
            st->old_syn_Overl[i]=0;
        }
        FOR(i=0; i<Txnq_offsetFB; i++)
        {
            move16();
            move16();
            st->syn_OverlFB[i+overlapFB+Txnq_offsetFB]=xn_bufFB[L_frameTCX-i];
            st->syn_OverlFB[i]=0;
        }
    }

    /* Output */
    Copy(xn_buf+shr(overlap,1)-tcx_offset, synth, L_frame_glob);
    Copy(xn_bufFB+shr(overlapFB,1)-tcx_offsetFB, synthFB, L_frameTCX_glob);

}



void decoder_tcx_post(Decoder_State_fx *st_fx,
                      Word16 *synth,
                      Word16 *synthFB,
                      Word16 *A,
                      Word16 bfi
                     )
{
    Word16 i;
    Word16 level_syn;
    Word32 step;
    Word16 gainCNG,gainCNG_e;
    Word16 tracingLevel;
    Word16 tracingLevel_e;
    Word16 xn_buf[L_FRAME_MAX];
    Word16 tmp1, tmp2, s;
    Word32 tmp32;
    Word32 tmp32_1, tmp32_2;

    /* TCX output */
    Copy( synth, xn_buf, st_fx->L_frame_fx );

    /* first TCX frame after ACELP; overwrite ltp initialization done during acelp PLC */
    test();
    test();
    if (!st_fx->bfi_fx && st_fx->prev_bfi_fx && sub(st_fx->last_core_fx,ACELP_CORE) == 0)
    {
        st_fx->tcxltp_last_gain_unmodified = 0;
        move16();
    }
    IF (bfi != 0 && st_fx->use_partial_copy == 0)
    {
        test();
        IF ( 0 == st_fx->enablePlcWaveadjust ||  sub(st_fx->plcInfo.concealment_method,TCX_TONAL) == 0 )
        {
            UWord32 dmy;
            tmp32_1 /*gainHelperFB*/    = L_shl_r(L_deposit_h(st_fx->gainHelper)    ,sub(st_fx->gainHelper_e,    31-28));/*Q28*/
            tmp32_2 /*stepCompensateFB*/= L_shl_r(L_deposit_h(st_fx->stepCompensate),sub(st_fx->stepCompensate_e,31-28));/*Q28*/

            Mpy_32_32_ss(tmp32_2/*Q28*/,
                         L_shl(L_mult0(st_fx->L_frame_fx,
                                       getInvFrameLen(st_fx->L_frameTCX)/*Q21*/)/*Q21*/,
                               8)/*Q29*/,
                         &tmp32_2,
                         &dmy ); /*Q26*/

            tmp32_2 = L_shl(tmp32_2,3-1); /*Q28*/

            FOR( i=0; i < st_fx->L_frameTCX; i++ )
            {
                tmp32 = L_shl(tmp32_1/*Q28*/,-(28-15)); /*16Q15*/
                synthFB[i] = round_fx(L_shl(Mpy_32_16_1(tmp32,synthFB[i]), 16));
                move16();
                tmp32_1 = L_sub(tmp32_1 , tmp32_2);
            }
        }
        tmp32_1 /*gainHelper*/    = L_shl_r(L_deposit_h(st_fx->gainHelper)    ,sub(st_fx->gainHelper_e,    31-28));/*Q28*/
        tmp32_2 /*stepCompensate*/= L_shl_r(L_deposit_h(st_fx->stepCompensate),sub(st_fx->stepCompensate_e,31-28));/*Q28*/
        FOR( i=0; i < st_fx->L_frame_fx; i++ )
        {
            tmp32 = L_shl(tmp32_1/*Q28*/,-(28-15)); /*16Q15*/
            xn_buf[i] = extract_l(Mpy_32_16_1(tmp32,xn_buf[i]));
            move16();
            tmp32_1 = L_sub(tmp32_1 , tmp32_2);
        }
    }

    /* PLC: [TCX: Fade-out]
     * PLC: estimate and update CNG energy */

    /* level_syn = (float)sqrt(( dot_product(synthFB, synthFB, L_frame)) / L_frame ); */
    s = sub(getScaleFactor16(synthFB, st_fx->L_frameTCX), 4);
    tmp32 = L_deposit_l(0);
    FOR (i = 0; i < st_fx->L_frameTCX; i++)
    {
        tmp1 = shl(synthFB[i], s);
        tmp32 = L_mac0(tmp32, tmp1, tmp1);
    }
    tmp32 = Mpy_32_16_1(tmp32, getInvFrameLen(st_fx->L_frameTCX));
    tmp2 = norm_l(tmp32);
    tmp1 = round_fx(L_shl(tmp32, tmp2));
    s = sub(sub(sub(1, shl(s, 1)), 6/*table lookup for inverse framelength*/), tmp2);
    tmp1 = Sqrt16(tmp1, &s);
    move16();
    level_syn = tmp1; /*Q0*/

    /* PLC: [TCX: Fade-out]
     * PLC: estimate and update CNG energy */

    tracingLevel=level_syn;
    move16();
    tracingLevel_e = add(s,15);

    test();
    test();
    IF (bfi == 0 && st_fx->tcxonly != 0 && sub(st_fx->clas_dec , UNVOICED_CLAS) == 0)
    {

        Word16 Qnew_levelBackgroundTrace;
        Qnew_levelBackgroundTrace = 0;
        move16();
        minimumStatistics(st_fx->conNoiseLevelMemory,                /*Q15*/
                          &st_fx->conNoiseLevelIndex,                /*Q0 */
                          &st_fx->conCurrLevelIndex,                 /*Q0 */
                          &st_fx->conCngLevelBackgroundTrace,        /*Q15*/
                          &st_fx->conLastFrameLevel,                 /*Q15*/
                          tracingLevel,                              /*Q15*/
                          st_fx->conNoiseLevelMemory_e,
                          st_fx->conCngLevelBackgroundTrace_e,
                          &Qnew_levelBackgroundTrace,
                          &st_fx->conLastFrameLevel_e,
                          tracingLevel_e                             /*scaling of tracingLevel*/
                         );

        /*note: All parameters being different from Q0 have to have the same Q-format*/

        st_fx->conCngLevelBackgroundTrace_e = Qnew_levelBackgroundTrace;
        move16();
    }

    /* PLC: [TCX: Fade-out]
     * PLC: fade-out in time domain */
    IF (bfi != 0)
    {
        Word32 conceal_eof_gain32;
        Word32 conceal_eof_gainFB;
        move16();
        move16();
        gainCNG = 1;
        gainCNG_e = 14+15+6; /*gainCNG is 2`097`152 - should be enough in case tracinglevel =~0 */
        IF (st_fx->tcxonly != 0)
        {
            /*gainCNG = st_fx->conCngLevelBackgroundTrace/(tracingLevel+0.01f);*/

            IF(tracingLevel != 0)
            {
                BASOP_Util_Divide_MantExp (
                    st_fx->conCngLevelBackgroundTrace,
                    st_fx->conCngLevelBackgroundTrace_e,
                    tracingLevel,
                    tracingLevel_e,
                    &gainCNG,
                    &gainCNG_e
                );
            }
        }
        ELSE
        {
            /*gainCNG = st_fx->cngTDLevel/(tracingLevel+0.01f);*/
            IF(tracingLevel != 0)
            {
                BASOP_Util_Divide_MantExp (
                    st_fx->cngTDLevel,
                    st_fx->cngTDLevel_e,
                    tracingLevel,
                    tracingLevel_e,
                    &gainCNG,
                    &gainCNG_e
                );
            }
        }

        if ((sub(st_fx->nbLostCmpt, 1) == 0))
        {
            st_fx->conceal_eof_gain = FL2WORD16(1.0f); /*Q15*/                        move16();
        }

        /* step = (st_fx->conceal_eof_gain - ( st_fx->conceal_eof_gain * st_fx->damping + gainCNG * (1 - st_fx->damping) )) / st_fx->L_frame_fx; */
        tmp2 = BASOP_Util_Add_MantExp(
                   mult_r(st_fx->conceal_eof_gain /*Q15*/,
                          st_fx->damping /*Q14*/),
                   15-14/*->Q15*/,
                   mult_r(gainCNG/*Q15*/,sub(0x4000,st_fx->damping/*Q14*/)) /*Q14*/,
                   add(gainCNG_e,15-14)/*->Q15*/,
                   &tmp1);
        tmp2 = BASOP_Util_Add_MantExp(st_fx->conceal_eof_gain,0,negate(tmp1),tmp2,&tmp1);
        step = L_shl(L_mult(tmp1, getInvFrameLen(st_fx->L_frame_fx)), sub(tmp2,6/*scaling from table lookup*/)); /*Q31*/
        step = L_max(0,step);
        {
            Word32 stepFB;
            UWord32 dmy;
            conceal_eof_gainFB = L_deposit_h(st_fx->conceal_eof_gain);
            Mpy_32_32_ss(step,L_shl(L_mult0(st_fx->L_frame_fx, getInvFrameLen(st_fx->L_frameTCX)),8),&stepFB ,&dmy );
            stepFB = L_shl(stepFB,3-1);
            FOR( i=0; i < st_fx->L_frameTCX; i++ )
            {
                synthFB[i] = mult_r(synthFB[i], round_fx(conceal_eof_gainFB));
                move16();
                conceal_eof_gainFB = L_sub(conceal_eof_gainFB, stepFB);
            }
        }
        conceal_eof_gain32 = L_deposit_h(st_fx->conceal_eof_gain); /*Q31*/
        FOR( i=0; i < st_fx->L_frame_fx; i++ )
        {
            xn_buf[i] = mult_r(xn_buf[i], st_fx->conceal_eof_gain);
            move16();
            conceal_eof_gain32 = L_sub(conceal_eof_gain32,step);
            st_fx->conceal_eof_gain = round_fx(conceal_eof_gain32);
        }
        test();
        IF ( 0 == st_fx->enablePlcWaveadjust ||  sub(st_fx->plcInfo.concealment_method,TCX_TONAL) == 0 )
        {
            st_fx->plcInfo.recovery_gain =  extract_h(L_shl(Mpy_32_16_1(conceal_eof_gainFB,
                                            st_fx->last_concealed_gain_syn_deemph),
                                            st_fx->last_concealed_gain_syn_deemph_e));/*Q31->Q15*/
        }
        ELSE
        {
            st_fx->plcInfo.recovery_gain = extract_h(conceal_eof_gainFB);
        }
        st_fx->plcInfo.step_concealgain_fx =
            round_fx(L_shl(L_mult0(
                               round_fx(step),
                               round_fx(L_shl(L_mult0(st_fx->L_frame_fx, getInvFrameLen(st_fx->L_frameTCX)),8))),3)); /*Q15*/
    }

    /*-----------------------------------------------------------*
     * Memory update                                             *
     *-----------------------------------------------------------*/

    /* Update synth, exc and old_Aq  */
    tcx_decoder_memory_update(xn_buf, /*Q0*/
                              synth,  /*Q0*/
                              st_fx->L_frame_fx,
                              A,
                              st_fx,
                              st_fx->syn, /*Q0*/
                              0
                             );


    /* PLC: [TCX: Memory update] */

    st_fx->old_pitch_buf_fx[0] = st_fx->old_pitch_buf_fx[st_fx->nb_subfr];
    move32();
    st_fx->old_pitch_buf_fx[1] = st_fx->old_pitch_buf_fx[st_fx->nb_subfr+1];
    move32();
    Copy32(&st_fx->old_pitch_buf_fx[st_fx->nb_subfr+2], &st_fx->old_pitch_buf_fx[2], st_fx->nb_subfr);
    set32_fx(&st_fx->old_pitch_buf_fx[st_fx->nb_subfr+2], st_fx->old_fpitch, st_fx->nb_subfr);

    st_fx->mem_pitch_gain[2*st_fx->nb_subfr+1]  = st_fx->mem_pitch_gain[st_fx->nb_subfr+1];
    move16();
    st_fx->mem_pitch_gain[2*st_fx->nb_subfr]  = st_fx->mem_pitch_gain[st_fx->nb_subfr];
    move16();

    FOR (i = 0; i < st_fx->nb_subfr; i++)
    {
        st_fx->mem_pitch_gain[2*st_fx->nb_subfr-1 - i]  = st_fx->mem_pitch_gain[st_fx->nb_subfr-1 - i];
        move16();
        st_fx->mem_pitch_gain[st_fx->nb_subfr-1 - i]  = st_fx->tcxltp_last_gain_unmodified;
        move16();
    }
}


static void IMDCT(Word32 *x, Word16 x_e,
                  Word16 *old_syn_overl,
                  Word16 *syn_Overl_TDAC,
                  Word16 *xn_buf,
                  const Word16  *tcx_aldo_window_1,
                  const PWord16 *tcx_aldo_window_1_trunc,
                  const PWord16 *tcx_aldo_window_2,
                  const PWord16 *tcx_mdct_window_half,
                  const PWord16 *tcx_mdct_window_minimum,
                  const PWord16 *tcx_mdct_window_trans,
                  Word16 tcx_mdct_window_half_length,
                  Word16 tcx_mdct_window_min_length,
                  Word16 index,
                  Word16 left_rect,
                  Word16 tcx_offset,
                  Word16 overlap,
                  Word16 L_frame,
                  Word16 L_frameTCX,
                  Word16 L_spec_TCX5,
                  Word16 L_frame_glob,
                  Word16 Txnq_offset,
                  Word16 frame_cnt,
                  Word16 bfi,
                  Word16 *old_out,
                  Word16 *Q_old_wtda,
                  Decoder_State_fx *st
                  ,Word16 fullbandScale
                  ,Word16 *acelp_zir
                 )
{
    const TCX_config *tcx_cfg = &st->tcx_cfg;
    Word16 tmp_offset;
    Word16 tmp1, tmp2, tmp3, *tmpP16;
    Word32 tmp32;
    Word8 tmp8;
    Word16 i;
    Word16 nz;
    Word16 aldo=0;

    /* number of zero for ALDO windows*/
    tmp32 = L_add(st->sr_core, 0);
    if (fullbandScale != 0)
    {
        tmp32 = L_add(st->output_Fs_fx, 0);
    }
    nz = NS2SA_fx2(tmp32, N_ZERO_MDCT_NS);

    tmp_offset = 0;
    move16();
    if (tcx_offset < 0)
    {
        tmp_offset = negate(tcx_offset);
    }

    test();
    IF ((sub(L_frameTCX, shr(st->L_frameTCX, 1)) == 0) && (st->tcxonly != 0))
    {
        /* Mode decision in PLC

        last OL   curr OL   left TCX-10  right TCX-10
        -------------------------------------------------------------
            0      0         2x TCX-5* 1x  TCX-10
            0      2         1x TCX-10 1x  TCX-10
            0      3         1x TCX-10 1x  TCX-10
            2      0         2x TCX-5  1x  TCX-10
            2      2         2x TCX-5  2x  TCX-5
            2      3         2x TCX-5  2x  TCX-5
            3      0         2x TCX-5  1x  TCX-10
            3      2         2x TCX-5  2x  TCX-5
            3      3         2x TCX-5  2x  TCX-5
        */
        test();
        test();
        test();
        test();
        test();
        test();
        IF ((bfi == 0 && tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) || (bfi!=0 && (tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) && (tcx_cfg->tcx_curr_overlap_mode != FULL_OVERLAP)))
        {
            /* minimum or half overlap, two transforms, grouping into one window */
            Word16 win[(L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2];
            Word16 w;
            Word16 L_win, L_ola;

            L_win = shr(L_frame, 1);
            L_ola = tcx_mdct_window_half_length;
            move16();
            if (sub(tcx_cfg->tcx_last_overlap_mode, MIN_OVERLAP) == 0)
            {
                L_ola = tcx_mdct_window_min_length;
                move16();
            }

            set16_fx(xn_buf, 0, add(tcx_offset,shr(L_ola,1)));  /* zero left end of buffer */
            set16_fx(win, 0, (L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2);

            FOR (w = 0; w < 2 ; w++)
            {

                TCX_MDCT_Inverse(x+L_mult0(w,L_spec_TCX5), sub(x_e,TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),win, L_ola, sub(L_win,L_ola), L_ola);

                tmp1 = left_rect;
                move16();
                tmp2 = tcx_cfg->tcx_last_overlap_mode;
                move16();
                tmp3 = st->last_core_bfi;
                move16();
                tmp8 = st->last_is_cng;
                move16();
                IF (w > 0)
                {
                    tmp1 = 0;
                    move16();
                    tmp2 = MIN_OVERLAP;
                    move16();
                    tmp3 = 1;
                    move16();
                    tmp8 = (Word8)0;
                    move16();
                }
                test();
                if(w == 0 && sub(index,2) == 0)
                {
                    tmp2 = MIN_OVERLAP;
                    move16();
                }
                IF (frame_cnt>0)
                {
                    tmp3 = 1;
                    move16();
                    tmp8 = (Word8)0;
                    move16();
                }

                tcx_windowing_synthesis_current_frame(win,
                                                      tcx_aldo_window_2,
                                                      tcx_mdct_window_half,
                                                      tcx_mdct_window_minimum,
                                                      L_ola,
                                                      tcx_mdct_window_half_length,
                                                      tcx_mdct_window_min_length,
                                                      tmp1,
                                                      tmp2,
                                                      acelp_zir,
                                                      st->old_syn_Overl,
                                                      syn_Overl_TDAC,
                                                      st->old_Aq_12_8_fx,
                                                      tcx_mdct_window_trans,
                                                      L_win,
                                                      tmp_offset,
                                                      bfi,
                                                      tmp3,
                                                      tmp8
                                                      ,fullbandScale
                                                     );

                IF (w > 0)
                {
                    tcx_windowing_synthesis_past_frame(xn_buf+tcx_offset-shr(L_ola , 1)+ imult1616(w,L_win),
                                                       tcx_aldo_window_1_trunc,
                                                       tcx_mdct_window_half,
                                                       tcx_mdct_window_minimum,
                                                       L_ola,
                                                       tcx_mdct_window_half_length,
                                                       tcx_mdct_window_min_length,
                                                       MIN_OVERLAP
                                                      );
                }
                /* add part of current sub-window overlapping with previous window */
                Vr_add(win,
                       xn_buf+tcx_offset-shr(L_ola,1)+w*L_win,  /*instrumented only shr because in fact, its only L_win+L_win+L_win...*/
                       xn_buf+tcx_offset-shr(L_ola,1)+w*L_win,
                       L_ola);
                /* copy new sub-window region not overlapping with previous window */
                Copy(
                    win+L_ola,
                    xn_buf+tcx_offset+shr(L_ola,1)+w*L_win,
                    L_win);
            }

            /* To assure that no garbage values are passed to overlap */
            set16_fx(xn_buf+L_frame+tcx_offset+shr(L_ola,1), 0, overlap-tcx_offset-shr(L_ola,1));
        }
        ELSE IF ( bfi == 0 && (frame_cnt == 0) && (tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP))
        {
            Word16 win[(L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2];
            Word16 L_win, L_ola, w;

            /* special overlap attempt, two transforms, grouping into one window */
            L_win = shr(L_frame, 1);
            L_ola = tcx_mdct_window_min_length;
            move16();

            set16_fx(win, 0, (L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2);

            /* 1st TCX-5 window, special MDCT with minimum overlap on right side */

            TCX_MDCT_Inverse(x, sub(x_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                             win + L_win,
                             0, sub(L_win, shr(L_ola, 1)), L_ola);

            set16_fx(xn_buf, 0, shr(overlap,1));
            /* copy new sub-window region not overlapping with previous window */
            Copy(win+L_win, xn_buf+shr(overlap,1), add(L_win,shr(L_ola,1))  );

            /* 2nd TCX-5 window, regular MDCT with minimum overlap on both sides */
            TCX_MDCT_Inverse(x + L_spec_TCX5, sub(x_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                             win,
                             L_ola, sub(L_win, L_ola), L_ola);

            tcx_windowing_synthesis_current_frame(win,
                                                  tcx_aldo_window_2,
                                                  tcx_mdct_window_half,
                                                  tcx_mdct_window_minimum,
                                                  L_ola,
                                                  tcx_mdct_window_half_length,
                                                  tcx_mdct_window_min_length,
                                                  0,  /* left_rect */
                                                  MIN_OVERLAP,  /* left_mode */
                                                  acelp_zir,
                                                  st->old_syn_Overl,
                                                  syn_Overl_TDAC,
                                                  st->old_Aq_12_8_fx,
                                                  tcx_mdct_window_trans,
                                                  L_win,
                                                  tmp_offset,
                                                  bfi,
                                                  1, /* st->last_core_bfi */
                                                  0  /* st->last_is_cng */
                                                  ,fullbandScale
                                                 );

            tmpP16 = xn_buf + add(sub(L_win, shr(L_ola, 1)), shr(overlap,1));

            tcx_windowing_synthesis_past_frame(tmpP16,
                                               tcx_aldo_window_1_trunc,
                                               tcx_mdct_window_half,
                                               tcx_mdct_window_minimum,
                                               L_ola,
                                               tcx_mdct_window_half_length,
                                               tcx_mdct_window_min_length,
                                               MIN_OVERLAP
                                              );

            /* add part of current sub-window overlapping with previous window */
            FOR (i = 0; i < L_ola; i++)
            {
                tmpP16[i] = add(tmpP16[i], win[i]);
                move16();
            }

            /* copy new sub-window region not overlapping with previous window */
            Copy(win + L_ola,
                 xn_buf + add(add(shr(overlap,1), shr(L_ola, 1)), L_win),
                 L_win);

            /* extra folding-out on left side of win, for perfect reconstruction */
            FOR (w = shr(overlap,1); w < overlap; w++)
            {
                xn_buf[overlap-1-w] = negate(xn_buf[w]);
                move16();
            }

            tcx_windowing_synthesis_current_frame(xn_buf,
                                                  tcx_aldo_window_2,
                                                  tcx_mdct_window_half,
                                                  tcx_mdct_window_minimum,
                                                  overlap,
                                                  tcx_mdct_window_half_length,
                                                  tcx_mdct_window_min_length,
                                                  left_rect,
                                                  0,  /* left_mode */
                                                  acelp_zir,
                                                  st->old_syn_Overl,
                                                  syn_Overl_TDAC,
                                                  st->old_Aq_12_8_fx,
                                                  tcx_mdct_window_trans,
                                                  shl(L_win,1),
                                                  tmp_offset,
                                                  bfi,
                                                  st->last_core_bfi,
                                                  st->last_is_cng,
                                                  fullbandScale
                                                 );

        }
        ELSE   /* default  i.e. maximum overlap, single transform, no grouping */
        {

            TCX_MDCT_Inverse(x, sub(x_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
            xn_buf,
            overlap, sub(L_frame, overlap), overlap);

            tmp1 = index;
            move16();
            test();
            test();
            test();
            if ( bfi==0 && (frame_cnt > 0) && (index == 0) && (st->last_core_fx != ACELP_CORE))
            {
                tmp1 = MIN_OVERLAP;
                move16();
            }

            tmp3 = st->last_core_bfi;
            move16();
            if (frame_cnt > 0)
            {
                tmp3 = 1;
                move16();
            }

            tmp8 = st->last_is_cng;
            move16();
            if (frame_cnt > 0)
            {
                tmp8 = 0;
                move16();
            }

            tcx_windowing_synthesis_current_frame(xn_buf,
            tcx_aldo_window_2,
            tcx_mdct_window_half,
            tcx_mdct_window_minimum,
            overlap,
            tcx_mdct_window_half_length,
            tcx_mdct_window_min_length,
            left_rect,
            tmp1,
            acelp_zir,
            st->old_syn_Overl,
            syn_Overl_TDAC,
            st->old_Aq_12_8_fx,
            tcx_mdct_window_trans,
            shr(L_frame_glob, 1),
            tmp_offset,
            bfi,
            tmp3,
            tmp8,
            fullbandScale
                                                 );
        }
    }
    ELSE   /* frame is TCX-20 or not TCX-only */
    {

        IF (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) != 0)
        {
            Word32 tmp_buf[L_FRAME_PLUS];
            Word16 Q;

            /* DCT */
            Q = sub(31, x_e);
            edct_fx(x, tmp_buf, L_frame, &Q);

            /* scale by sqrt(L / NORM_MDCT_FACTOR) */
            tmp1 = mult_r(shl(L_frame, 4), FL2WORD16(128.f / NORM_MDCT_FACTOR)); /* 4Q11 */
            tmp2 = 4;
            move16();
            tmp1 = Sqrt16(tmp1, &tmp2);

            FOR (i = 0; i < L_frame; i++)
            {
                tmp_buf[i] = Mpy_32_16_1(tmp_buf[i], tmp1);
                move32();
            }
            Q = sub(Q, tmp2);


            tmp1 = getScaleFactor16(st->old_out_fx, L_frame);
            *Q_old_wtda = add(*Q_old_wtda, tmp1);
            move16();
            FOR (i = 0; i < L_frame; i++)
            {
                old_out[i] = shl(old_out[i], tmp1);
                move16();
            }

            window_ola_fx(tmp_buf,
                          xn_buf,
                          &Q,
                          old_out,
                          Q_old_wtda,
                          L_frame,
                          tcx_cfg->tcx_last_overlap_mode,
                          tcx_cfg->tcx_curr_overlap_mode,
                          0,
                          0,
                          NULL);

            /* scale output */
            IF (Q <= 0)
            {
                FOR (i = 0; i < L_frame; i++)
                {
                    xn_buf[i] = shr(xn_buf[i], Q);
                    move16();
                }
            }
            ELSE
            {
                tmp1 = shr(0x4000, sub(Q,1));

                FOR (i = 0; i < L_frame; i++)
                {
                    xn_buf[i] = mult_r(xn_buf[i], tmp1);
                    move16();
                }
            }

            aldo = 1;
            move16();
        }
        ELSE
        {

            TCX_MDCT_Inverse(x, sub(x_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
            xn_buf,
            overlap, sub(L_frame, overlap), overlap);


            /*-----------------------------------------------------------*
             * Windowing, overlap and add                                *
             *-----------------------------------------------------------*/


            /* Window current frame */
            tmp3 = st->last_core_bfi;
            move16();
            if (frame_cnt > 0)
            {
                tmp3 = 1;
                move16();
            }

            tmp8 = st->last_is_cng;
            move16();
            if (frame_cnt > 0)
            {
                tmp8 = 0;
                move16();
            }

            tcx_windowing_synthesis_current_frame(  xn_buf,
            tcx_aldo_window_2,
            tcx_mdct_window_half,
            tcx_mdct_window_minimum,
            overlap,
            tcx_mdct_window_half_length,
            tcx_mdct_window_min_length,
            left_rect,
            tcx_cfg->tcx_last_overlap_mode,
            acelp_zir,
            st->old_syn_Overl,
            syn_Overl_TDAC,
            st->old_Aq_12_8_fx,
            tcx_mdct_window_trans,
            shr(L_frame_glob, 1),
            tmp_offset,
            bfi,
            tmp3,
            tmp8
            ,fullbandScale
                                                 );
        } /* TRANSITION_OVERLAP */
    } /* TCX-20 and TCX-only */

    /* Window and overlap-add past frame if past frame is TCX */
    test();
    IF ((frame_cnt != 0) || (st->last_core_bfi > ACELP_CORE))
    {
        test();
        test();
        IF (((sub(L_frameTCX, shr(st->L_frameTCX, 1)) == 0) && (st->tcxonly != 0)) || (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) == 0))
        {
            test();
            test();
            test();
            test();
            if ((bfi == 0) && (frame_cnt > 0) && (index == 0) &&
                    (tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP) && (st->last_core_fx != ACELP_CORE))
            {
                index = MIN_OVERLAP;   /* use minimum overlap between the two TCX-10 windows */         move16();
            }

            IF (tcx_cfg->last_aldo != 0)
            {
                Word16 tmp4;

                tmp2 = add(*Q_old_wtda, TCX_IMDCT_HEADROOM);
                tmp4 = sub(shr(overlap, 1), tcx_offset);

                FOR (i = 0; i < tmp4; i++)
                {
                    xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                    move16();
                }

                tmp1 = sub(overlap, tcx_mdct_window_min_length);
                FOR (i=0; i < tmp1; i++)
                {
                    xn_buf[i+tmp4] = shl(add(xn_buf[i+tmp4], shr(old_out[i+nz], tmp2)), TCX_IMDCT_HEADROOM);
                    move16();
                }

                /* fade truncated ALDO window */
                tmp1 = sub(overlap, shr(tcx_mdct_window_min_length, 1));
                FOR ( ; i < tmp1; i++)
                {
                    tmp3 = mult_r(shr(old_out[i+nz], tmp2), tcx_mdct_window_minimum[i-overlap+tcx_mdct_window_min_length].v.re);
                    xn_buf[i+tmp4] = shl(add(xn_buf[i+tmp4], tmp3), TCX_IMDCT_HEADROOM);
                    move16();
                }
                FOR ( ; i < overlap; i++)
                {
                    tmp3 = mult_r(shr(old_out[i+nz], tmp2), tcx_mdct_window_minimum[overlap-1-i].v.im);
                    xn_buf[i+tmp4] = shl(add(xn_buf[i+tmp4], tmp3), TCX_IMDCT_HEADROOM);
                    move16();
                }

                FOR (i = add(i, tmp4) ; i < L_frame; i++)
                {
                    xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                    move16();
                }
            }
            ELSE
            {
                tmp1 = index;
                move16();
                test();
                if ((index == 0) || (sub(tcx_cfg->tcx_last_overlap_mode, MIN_OVERLAP) == 0))
                {
                    tmp1 = tcx_cfg->tcx_last_overlap_mode;
                    move16();
                }

                tcx_windowing_synthesis_past_frame( old_syn_overl+Txnq_offset,
                tcx_aldo_window_1_trunc,
                tcx_mdct_window_half,
                tcx_mdct_window_minimum,
                overlap,
                tcx_mdct_window_half_length,
                tcx_mdct_window_min_length,
                tmp1
                                                  );

                BASOP_SATURATE_WARNING_OFF;
                IF ( bfi )
                {
                    tmp1 = sub(shr(overlap, 1), tcx_offset);
                    tmp3 = shr(tcx_mdct_window_half_length, 1);
                    FOR (i=0; i < tmp1; i++)
                    {
                        xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                        move16();
                    }
                    FOR (i = 0; i < tmp3; i++)
                    {
                        tmp2 = add(xn_buf[i+tmp1], mult_r(old_syn_overl[i+Txnq_offset], tcx_mdct_window_half[i].v.re));
                        xn_buf[i+tmp1] = shl(tmp2, TCX_IMDCT_HEADROOM);
                        move16();
                    }
                    FOR ( ; i < tcx_mdct_window_half_length; i++)
                    {
                        tmp2 = add(xn_buf[i+tmp1], mult_r(old_syn_overl[i+Txnq_offset], tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.im));
                        xn_buf[i+tmp1] = shl(tmp2, TCX_IMDCT_HEADROOM);
                        move16();
                    }
                    IF (sub(add(i, tmp1), L_frame) < 0)
                    {
                        FOR (i = add(i, tmp1); i < L_frame; i++)
                        {
                            xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                            move16();
                        }
                    }
                }
                ELSE IF (left_rect == 0)
                {
                    FOR (i=0; i<overlap; i++)
                    {

                        xn_buf[i] = shl(add(xn_buf[i], old_syn_overl[i+Txnq_offset]), TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    IF (sub(i, L_frame) < 0)
                    {
                        FOR ( ; i < L_frame; i++)
                        {
                            xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                            move16();
                        }
                    }
                }
                ELSE {
                    tmp1 = shr(overlap, 1);
                    FOR (i=0; i < tmp1; i++)
                    {
                        xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    tmpP16 = xn_buf + tmp1;
                    FOR (i=0; i < overlap; i++)
                    {
                        tmpP16[i] = shl(add(tmpP16[i], old_syn_overl[i]), TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    IF (sub(add(i, tmp1), L_frame) < 0)
                    {
                        FOR (i = add(i, tmp1); i < L_frame; i++)
                        {
                            xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                            move16();
                        }
                    }
                }
                BASOP_SATURATE_WARNING_ON;
            }
        }
        ELSE
        {
            IF (aldo == 0)
            {
                BASOP_SATURATE_WARNING_OFF;
                FOR (i = 0; i < L_frame; i++)
                {
                    xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                    move16();
                }
                BASOP_SATURATE_WARNING_ON;
            }
        }
    }
    ELSE
    {
        IF (aldo == 0)
        {
            BASOP_SATURATE_WARNING_OFF;
            FOR (i = 0; i < L_frame; i++)
            {
                xn_buf[i] = shl(xn_buf[i], TCX_IMDCT_HEADROOM);
                move16();
            }
            BASOP_SATURATE_WARNING_ON;
        }
    }

    test();
    test();
    test();
    IF ( (aldo == 0) &&
         ((sub(L_frameTCX, shr(st->L_frameTCX, 1)) == 0 && frame_cnt > 0) ||
          sub(L_frameTCX, shr(st->L_frameTCX, 1)) != 0) )
    {
        /*Compute windowed synthesis in case of switching to ALDO windows in next frame*/
        FOR (i = 0; i < nz; i++)
        {
            old_out[i] = shr(xn_buf[L_frame-nz+i], TCX_IMDCT_HEADROOM);
            move16();
        }
        Copy(xn_buf+L_frame, old_out+nz, overlap);
        set16_fx(old_out+nz+overlap, 0, nz);

        tcx_windowing_synthesis_past_frame( old_out+nz,
                                            tcx_aldo_window_1_trunc,
                                            tcx_mdct_window_half,
                                            tcx_mdct_window_minimum,
                                            overlap,
                                            tcx_mdct_window_half_length,
                                            tcx_mdct_window_min_length,
                                            tcx_cfg->tcx_curr_overlap_mode
                                          );

        /* If current overlap mode = FULL_OVERLAP -> ALDO_WINDOW */
        IF (sub(tcx_cfg->tcx_curr_overlap_mode, FULL_OVERLAP) == 0)
        {
            FOR (i=0; i<nz; i++)
            {
                old_out[nz+overlap+i] = shr(mult_r(xn_buf[L_frame-1-i], tcx_aldo_window_1[nz-1-i]), TCX_IMDCT_HEADROOM);
                move16();
            }
        }

        *Q_old_wtda = -TCX_IMDCT_HEADROOM;
        move16();
    }
    if (fullbandScale != 0)
    {
        st->tcx_cfg.last_aldo = aldo;
        move16();
    }

    /* Smoothing between the ACELP PLC and TCX Transition frame. Using the shape of the half overlap window for the crossfading. */
    test();
    test();
    test();
    IF (left_rect && (frame_cnt == 0) && (st->last_core_bfi == ACELP_CORE)
        && st->prev_bfi_fx)
    {

        IF (fullbandScale)
        {
            tmp1 = sub(shr(overlap, 1), tcx_offset);
            tmp3 = shr(tcx_mdct_window_half_length, 1);
            FOR (i = 0; i < tmp3; i++)
            {
                xn_buf[i+tmp1] = mult_r( xn_buf[i+tmp1], tcx_mdct_window_half[i].v.im );
                xn_buf[i+tmp1] = add( xn_buf[i+tmp1], mult_r( st->syn_OverlFB[i], mult_r( tcx_mdct_window_half[i].v.re, tcx_mdct_window_half[i].v.re ) ) );
                move16();
            }
            FOR ( ; i < tcx_mdct_window_half_length; i++)
            {
                xn_buf[i+tmp1] = mult_r( xn_buf[i+tmp1], tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.re );
                xn_buf[i+tmp1] = add( xn_buf[i+tmp1], mult_r( st->syn_OverlFB[i], mult_r( tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.im, tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.im ) ) );
                move16();
            }
        }
        ELSE
        {
            tmp1 = sub(shr(overlap, 1), tcx_offset);
            tmp3 = shr(tcx_mdct_window_half_length, 1);
            FOR (i = 0; i < tmp3; i++)
            {
                xn_buf[i+tmp1] = mult_r( xn_buf[i+tmp1], tcx_mdct_window_half[i].v.im );
                xn_buf[i+tmp1] = add( xn_buf[i+tmp1], mult_r( st->syn_Overl[i], mult_r( tcx_mdct_window_half[i].v.re, tcx_mdct_window_half[i].v.re ) ) );
                move16();
            }
            FOR ( ; i < tcx_mdct_window_half_length; i++)
            {
                xn_buf[i+tmp1] = mult_r( xn_buf[i+tmp1], tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.re );
                xn_buf[i+tmp1] = add( xn_buf[i+tmp1], mult_r( st->syn_Overl[i], mult_r( tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.im, tcx_mdct_window_half[tcx_mdct_window_half_length-1-i].v.im ) ) );
                move16();
            }
        }
    }
}

