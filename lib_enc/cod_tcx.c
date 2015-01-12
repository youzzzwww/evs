/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "stat_com.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "basop_mpy.h"



/* Up to the Autocorrelation it is the same code as in GetMDCT, with the difference in the parameters in the call to tcx_windowing_analysis */
void HBAutocorrelation(
    TCX_config *tcx_cfg,        /* input: configuration of TCX          */
    Word16 left_overlap_mode,   /* input: overlap mode of left window half   */
    Word16 right_overlap_mode,  /* input: overlap mode of right window half  */
    Word16 speech[],            /* input: speech[-LFAC..L_frame+LFAC]   */
    Word16 L_frame,             /* input: frame length                  */
    Word32 *r,                  /* output: autocorrelations vector */
    Word16 m                    /* input : order of LP filter      */
)
{
    Word16 i, j, left_overlap, right_overlap;
    Word16 len, norm, shift, fact;
    Word32 L_tmp, L_sum;
    Word16 y[L_MDCT_OVLP_MAX+L_FRAME_PLUS+L_MDCT_OVLP_MAX];


    /*-----------------------------------------------------------*
     * Windowing                                                 *
     *-----------------------------------------------------------*/

    WindowSignal(tcx_cfg, tcx_cfg->tcx_offset, left_overlap_mode, right_overlap_mode, &left_overlap, &right_overlap, speech, &L_frame, y, 0);

    /*-----------------------------------------------------------*
     * Autocorrelation                                           *
     *-----------------------------------------------------------*/

    len = add(L_frame, shr(add(left_overlap, right_overlap), 1));

    /* calculate shift */
    shift = 0;
    move16();
    L_sum = L_deposit_l(0);
    Overflow = 0;
    move16();
    FOR (i = 0; i < len; i+=1)
    {
        /* Test Addition */
        L_mac0(L_sum, y[i], y[i]);
        IF (Overflow)
        {
            Overflow = 0;
            move16();
            shift = 1;
            move16();
            L_tmp = L_msu0( 0, y[i], y[i] );
            L_tmp = L_shr( L_tmp, 1 );
            L_sum = L_add( L_shr( L_sub( L_sum, 1 ), 1 ), 1 );

            L_sum = L_sub( L_sum, L_tmp );

            FOR (j = add(i,1); j<len; j++)
            {
                L_tmp = L_msu0( 0, y[j], y[j] );
                L_tmp = L_shr( L_tmp, shift );

                /* Test Addition */
                L_sub( L_sum, L_tmp );
                IF ( Overflow  )
                {
                    Overflow = 0;
                    move16();
                    shift = add( shift, 1 );
                    L_tmp = L_shr( L_tmp, 1 );
                    L_sum = L_add( L_shr( L_sub( L_sum, 1 ), 1 ), 1 );
                }
                L_sum = L_sub( L_sum, L_tmp );
            }
            BREAK;
        }
        /* Perform Addition */
        L_sum = L_mac0( L_sum, y[i], y[i] );
    }

    /* scale signal to avoid overflow in autocorrelation */
    IF (shift > 0)
    {
        fact = lshr(-32768, shift);
        FOR (i = 0; i < len; i++)
        {
            y[i] = mult_r(y[i], fact);
            move16();
        }
    }

    /* Compute and normalize r[0] */
    L_sum = L_mac0(1, y[0], y[0]);
    FOR (i = 1; i < len; i++)
    {
        L_sum = L_mac0(L_sum, y[i], y[i]);
    }

    norm = norm_l(L_sum);
    L_sum = L_shl(L_sum, norm);
    r[0] = L_sum;
    move32();

    /* Compute r[1] to r[m] */
    FOR (i = 1; i <= m; i++)
    {
        L_sum = L_mult0(y[0],y[i]);
        FOR (j = 1; j < len - i; j++)
        {
            L_sum = L_mac0(L_sum, y[j], y[j + i]);
        }

        L_sum = L_shl(L_sum, norm);
        r[i] = L_sum;
        move32();
    }

}

void TNSAnalysis(
    TCX_config *tcx_cfg,    /* input: configuration of TCX */
    Word16 L_frame,         /* input: frame length */
    Word16 L_spec,
    Word16 tcxMode,         /* input: TCX mode for the frame/subframe - TCX20 | TCX10 | TCX 5 (meaning 2 x TCX 5) */
    Word8 isAfterACELP,     /* input: Flag indicating if the last frame was ACELP. For the second TCX subframe it should be 0  */
    Word32 spectrum[],      /* input: MDCT spectrum */
    STnsData * pTnsData,    /* output: Tns data */
    Word8 * pfUseTns,       /* output: Flag indicating if TNS is used */
    Word16 *predictionGain
)
{
    Word32 buff[8];
    Word16 tmp = 0; /* initialization only to avoid compiler warning, not counted */
    Word16 tmp2 = 0; /* initialization only to avoid compiler warning, not counted */


    /* Init TNS */
    *pfUseTns = 0;
    move16();

    IF (tcx_cfg->fIsTNSAllowed != 0)
    {
        tcx_cfg->pCurrentTnsConfig = &tcx_cfg->tnsConfig[sub(tcxMode, TCX_20) == 0][isAfterACELP];
        test();
        L_spec = tcx_cfg->pCurrentTnsConfig->iFilterBorders[0];
        move16();

        /*-----------------------------------------------------------*
         * Temporal Noise Shaping analysis                           *
         *-----------------------------------------------------------*/

        IF (sub(tcxMode, TCX_5) == 0)
        {
            tmp = shr(L_frame,2);

            /* rearrange LF sub-window lines prior to TNS analysis & filtering */
            tmp2 = shr(L_spec,1);

            IF (sub(tmp2, tmp) < 0)
            {
                Copy32(spectrum+8, spectrum+16, sub(tmp2, 8));
                Copy32(spectrum+tmp, spectrum+8, 8);
                Copy32(spectrum+tmp+8, spectrum+tmp2+8, sub(tmp2, 8));
            }
            ELSE
            {
                Copy32(spectrum+tmp, buff, 8);
                Copy32(spectrum+8, spectrum+16, sub(tmp, 8));
                Copy32(buff, spectrum+8, 8);
            }
        }

        move16();
        *pfUseTns = (Word8)DetectTnsFilt(tcx_cfg->pCurrentTnsConfig, spectrum, pTnsData, predictionGain);

        /* If TNS should be used then get the residual after applying it inplace in spectrum */
        IF (*pfUseTns != 0)
        {
            ApplyTnsFilter(tcx_cfg->pCurrentTnsConfig, pTnsData, spectrum, 1);
        }

        IF (sub(tcxMode, TCX_5) == 0)
        {
            /* undo rearrangement of LF sub-window lines prior to TNS analysis */
            IF (sub(tmp2, tmp) < 0)
            {
                Copy32(spectrum+tmp2+8, spectrum+tmp+8, sub(tmp2, 8));
                Copy32(spectrum+8, spectrum+tmp, 8);
                Copy32(spectrum+16, spectrum+8, sub(tmp2, 8));
                set32_fx(spectrum+tmp2, 0, sub(tmp,tmp2));
                set32_fx(spectrum+tmp+tmp2, 0, sub(tmp,tmp2));
            }
            ELSE
            {
                Copy32(spectrum+8, buff, 8);
                Copy32(spectrum+16, spectrum+8, sub(tmp, 8));
                Copy32(buff, spectrum+tmp, 8);
            }
        }
    }

}

void ShapeSpectrum(
    TCX_config *tcx_cfg,/*input: configuration of TCX*/
    Word16 A[],         /* input: quantized coefficients NxAz_q[M+1] */
    Word16 gainlpc[],   /* output: MDCT gains for the previous frame */
    Word16 gainlpc_e[], /* output: MDCT gains exponents */
    Word16 L_frame_glob,/* input: frame length             */
    Word16 L_spec,
    Word32 spectrum[],  /* i/o: MDCT spectrum */
    Word8 pfUseTns,     /* output: Flag indicating if TNS is used */
    Encoder_State_fx *st
)
{
    Word16 L_frame;
    Word16 Ap[M+2];
    Word16 gamma1;
    Word16 gainlpc_noinv[FDNS_NPTS];
    Word16 gainlpc_noinv_e[FDNS_NPTS];
    Word16 i;


    /*-----------------------------------------------------------*
     * Init                                                      *
     *-----------------------------------------------------------*/

    /* Init lengths */
    L_frame = L_frame_glob;
    move16();
    gamma1 = st->gamma;
    move16();
    if (st->enableTcxLpc != 0)
    {
        gamma1 = 0x7FFF;
        move16();
    }

    /* if past frame is ACELP */

    IF (st->last_core_fx == ACELP_CORE)
    {
        L_frame = add(L_frame, tcx_cfg->tcx_offset);
        L_spec = add(L_spec, shr(tcx_cfg->tcx_coded_lines, 2));
        if(tcx_cfg->lfacNext<0)
        {
            L_frame = sub(L_frame,tcx_cfg->lfacNext);
            move16();
        }
    }

    test();
    tcxGetNoiseFillingTilt(A,
                           M,
                           L_frame,
                           (L_sub(st->total_brate_fx, ACELP_13k20) >= 0 && st->rf_mode == 0 ),
                           &st->noiseTiltFactor);

    /* Calculate Spectrum Flatness Measure for the TCX Concealment */
    IF (st->enablePlcWaveadjust)
    {
        tcx_cfg->SFM2 = SFM_Cal(spectrum, s_min(200, L_frame));
    }

    /*-----------------------------------------------------------*
     * Pre-shaping in frequency domain using weighted LPC (Wz)   *
     *-----------------------------------------------------------*/

    weight_a_fx( A, Ap, gamma1, M );

    lpc2mdct( Ap, M, gainlpc_noinv, gainlpc_noinv_e, gainlpc, gainlpc_e );

    mdct_shaping( spectrum, L_frame, gainlpc_noinv, gainlpc_noinv_e );
    FOR (i = L_frame; i < L_spec; i++)
    {
        spectrum[i] = L_shl(Mpy_32_16_1(spectrum[i], gainlpc_noinv[FDNS_NPTS-1]), gainlpc_noinv_e[FDNS_NPTS-1]);
        move32();
    }

    test();
    test();
    test();
    IF( st->tcxonly && st->tcxltp && (st->tcxltp_gain > 0) && !pfUseTns )
    {
        PsychAdaptLowFreqEmph(spectrum, gainlpc, gainlpc_e);
    }

}


void QuantizeSpectrum(
    TCX_config *tcx_cfg,    /*input: configuration of TCX*/
    Word16 A[],             /* input: quantized coefficients NxAz_q[M+1] */
    Word16 Aqind[],         /* input: frame-independent quantized coefficients (M+1) */
    Word16 gainlpc[],       /* input: MDCT gains of the previous frame */
    Word16 gainlpc_e[],     /* input: MDCT gains exponents */
    Word16 synth[],
    Word16 L_frame_glob,    /* input: frame length             */
    Word16 L_frameTCX_glob,
    Word16 L_spec,
    Word16 nb_bits,         /*input: bit budget*/
    Word8 tcxonly,          /*input: only TCX flag*/
    Word32 spectrum[],      /* i/o: MDCT spectrum, input is shaped MDCT spectrum */
    Word16 *spectrum_e,     /* i/o: MDCT spectrum exponent */
    STnsData * pTnsData,    /* input: Tns data */
    Word8 fUseTns,          /* input: Flag indicating if TNS is used */
    Word16 tnsSize,         /* input: number of tns parameters put into prm */
    LPD_state *LPDmem,      /*i/o: memories*/
    Word16 prm[],           /* output: tcx parameters          */
    Word16 frame_cnt,       /* input: frame counter in the super_frame */
    Encoder_State_fx *st,
    CONTEXT_HM_CONFIG *hm_cfg
)
{
    Word16 i, L_frame, tcx_offset;
    Word16 stop;
    Word16 tmp1, tmp2, tmp3, tmp4, s;
    Word32 tmp32;
    Word8 tmp8;
    Word16 *tmpP16;
    Word16 L_frameTCX;
    Word16 fac_ns;
    Word16 nf_seed;
    Word32 ener;
    Word16 ener_e;
    Word16 gain_tcx, gain_tcx_e;
    Word16 sqBits;
    Word16 overlap, Txnq_offset;
    Word16 noiseFillingSize;
    Word16 noiseTransWidth;
    Word32 *OriginalSpectrum;
    Word16 OriginalSpectrum_e;
    Word16 ctxHmBits;
    Word16 resQBits;
    Word16 *signs;
    Word16 signaling_bits;
    Word16 *prm_ltp, *prm_tns, *prm_hm, *prm_lastnz, *prm_target;
    Word16 Aq_old[M+1];
    Word32 SFM;
    Word32 K, K2;
    Word16 aldo;       /* ALDO flag in current frame*/
    Word16 nz;         /* non-zero length in ALDO window*/


    /* Stack memory is split between encoder and internal decoder to reduce max
       stack memory usage. */
    {
        Word16 sqTargetBits;
        Word16 gain_tcx_opt, gain_tcx_opt_e;
        Word16 sqGain, sqGain_e;
        Word16 sqBits_noStop;
        Word16 nEncoded;
        Word16 maxNfCalcBw;
        Word16 PeriodicityIndex;
        Word16 NumIndexBits;
        Word16 nEncodedCtxHm, stopCtxHm, sqBitsCtxHm, Selector;
        Word16 lastnz, lastnzCtxHm;
        Word16 RelativeScore;
        Word32 x_orig[N_MAX];
        Word16 x_orig_e;
        Word16 resQTargetBits;
        Word16 xn_buf16[L_FRAME_PLUS];
        Word16 *sqQ;
        Word16 LtpPitchLag;


        sqGain = 0x4000;
        move16();
        sqGain_e = 1;
        move16();
        noiseTransWidth = MIN_NOISE_FILLING_HOLE;
        move16();
        resQTargetBits = 0;
        move16();

        /*-----------------------------------------------------------*
         * Init                                                      *
         *-----------------------------------------------------------*/

        /* Init lengths */
        L_frame = L_frame_glob;
        move16();
        L_frameTCX = L_frameTCX_glob;
        move16();
        overlap = tcx_cfg->tcx_mdct_window_length;
        move16();
        Txnq_offset=0;
        move16();
        aldo = 0;
        move16();
        nz = NS2SA_fx2(st->sr_core, N_ZERO_MDCT_NS);
        move16();
        /* Modified the overlap to the delay in case of short blocks*/
        tcx_offset = tcx_cfg->tcx_offset;
        move16();

        OriginalSpectrum = NULL;
        signs = NULL; /* silence warning */
        NumIndexBits = 0;
        move16();
        sqBits = 0;
        move16();
        ctxHmBits = 0;
        move16();
        resQBits = 0;
        move16();
        prm_ltp = &prm[1+NOISE_FILL_RANGES];
        move16();
        prm_tns = prm_ltp + LTPSIZE;
        move16();
        prm_hm = prm_tns + tnsSize;
        move16();
        prm_lastnz = prm_hm + 2;
        move16();
        sqQ = prm_hm + NPRM_CTX_HM;
        move16();

        /* if past frame is ACELP */

        IF (st->last_core_fx == ACELP_CORE)
        {
            tcx_cfg->last_aldo = 0;
            move16();

            L_frame = add(L_frame, tcx_offset);
            L_frameTCX = add(L_frameTCX, tcx_cfg->tcx_offsetFB);
            L_spec = add(L_spec, shr(tcx_cfg->tcx_coded_lines, 2));
            tcx_offset = 0;
            move16();
            IF(tcx_cfg->lfacNext<0)
            {
                L_frame = sub(L_frame,tcx_cfg->lfacNext);
                L_frameTCX = sub(L_frameTCX, tcx_cfg->lfacNextFB);
                tcx_offset = tcx_cfg->lfacNext;
                move16();
            }
            st->noiseLevelMemory = 0;
            move16();
        }


        E_LPC_f_lsp_a_conversion(st->lsp_old_fx, Aq_old, M);

        /* target bitrate for SQ */
        sqTargetBits = sub(nb_bits, 7 + NBITS_NOISE_FILL_LEVEL);

        /*Unquantized spectrum here*/
        IF (st->enablePlcWaveadjust)
        {


            SFM = SFM_Cal(spectrum, s_min(200, L_frame_glob));
            test();
            IF (sub(L_frame_glob, 256) <= 0)
            {
                K  = 0x33333333;
                move32();
                K2 = 0xCCCCCCD;
                move32();
            }
            ELSE IF (sub(L_frame_glob,320) == 0 || sub(L_frame_glob, 512)== 0 )
            {
                K  = 0x33333333;
                move32();
                K2 = 0xCCCCCCD;
                move32();
            }
            ELSE /*FrameSize_Core == 640*/
            {
                K  = 0x2CCCCCCD;
                move32();
                K2 = 0x51EB852;
                move32();
            }


            IF ( L_sub(SFM, K)<0 )
            {
                st->Tonal_SideInfo = 1;
                move16();
            }
            ELSE
            {
                st->Tonal_SideInfo = 0;
                move16();
            }

            if ( L_sub(tcx_cfg->SFM2, K2)< 0)
            {
                st->Tonal_SideInfo = 1;
                move16();
            }
        }

        /* Save pre-shaped spectrum*/
        Copy32(spectrum, x_orig, L_spec);
        x_orig_e = *spectrum_e;
        move16();

        /*-----------------------------------------------------------*
         * Bandwidth Limitation                                      *
         *-----------------------------------------------------------*/

        noiseFillingSize = L_spec;
        move16();
        IF (st->igf != 0)
        {
            noiseFillingSize = st->hIGFEnc.infoStartLine;
            move16();
        }
        ELSE
        {
            st->hIGFEnc.infoStopLine = noiseFillingSize;
            move16();
        }

        FOR (i=st->hIGFEnc.infoStopLine; i<L_frameTCX; i++)
        {
            spectrum[i] = L_deposit_l(0);
        }

        /*-----------------------------------------------------------*
         * Quantization                                              *
         *-----------------------------------------------------------*/

        IF (st->tcx_lpc_shaped_ari == 0)   /* old arithmetic coder */
        {

            /* Fast estimation of the scalar quantizer step size */
            test();
            IF ((tcx_cfg->ctx_hm != 0) && (st->last_core_fx != ACELP_CORE))
            {
                LtpPitchLag = -1;
                move16();

                test();
                IF ((tcxonly == 0) && (sub(st->tcxltp_pitch_int, st->L_frame_fx) < 0))
                {
                    tmp32 = L_shl(L_mult0(st->L_frame_fx, st->pit_res_max), 1+kLtpHmFractionalResolution+1);
                    tmp1 = add(imult1616(st->tcxltp_pitch_int, st->pit_res_max), st->tcxltp_pitch_fr);
                    LtpPitchLag = div_l(tmp32, tmp1);
                }

                ctxHmBits = add(ctxHmBits, 1);    /* ContextHM flag */
                sqTargetBits = sub(sqTargetBits, 1);  /* ContextHM flag */

                OriginalSpectrum = spectrum;
                OriginalSpectrum_e = *spectrum_e;
                move16();

                tmp1 = -1;
                move16();
                if (st->tcxltp != 0)
                {
                    tmp1 = st->tcxltp_gain;
                    move16();
                }
                PeriodicityIndex = SearchPeriodicityIndex(
                                       OriginalSpectrum,
                                       NULL,
                                       L_spec,
                                       sqTargetBits,
                                       LtpPitchLag,
                                       tmp1,
                                       &RelativeScore
                                   );

                ConfigureContextHm(
                    L_spec,
                    sqTargetBits,
                    PeriodicityIndex,
                    LtpPitchLag,
                    hm_cfg
                );

                tmp1 = 1;
                move16();
                if (sub(L_spec, 256) < 0)
                {
                    tmp1 = 0;
                    move16();
                }
                NumIndexBits = CountIndexBits( tmp1, PeriodicityIndex);



                /* Quantize original spectrum */

                sqGain = SQ_gain(OriginalSpectrum, OriginalSpectrum_e,
                                 shl(mult(LPDmem->tcx_target_bits_fac, sqTargetBits), 1),
                                 L_spec,
                                 &sqGain_e);

                tcx_scalar_quantization(OriginalSpectrum, OriginalSpectrum_e,
                                        sqQ,
                                        L_spec,
                                        sqGain, sqGain_e,
                                        tcx_cfg->sq_rounding,
                                        st->memQuantZeros,
                                        tcxonly);

                /* Estimate original bitrate */
                stop = 0;
                move16();

                sqBits = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                         L_spec,
                         &lastnz,
                         &nEncoded,
                         sqTargetBits,
                         &stop,
                         NULL);

                /* Estimate context mapped bitrate */

                stopCtxHm = 0;
                move16();

                /* Context Mapping */
                sqBitsCtxHm = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                              L_spec,
                              &lastnzCtxHm,
                              &nEncodedCtxHm,
                              sub(sqTargetBits, NumIndexBits),
                              &stopCtxHm,
                              hm_cfg
                                                                             );

                /* Decide whether or not to use context mapping */
                Selector = sub(s_max(stop, sqBits), add(s_max(stopCtxHm, sqBitsCtxHm), NumIndexBits));

                test();
                test();
                IF ((sub(Selector, 2) > 0) || ((sub(abs_s(Selector), 2) <= 0) &&
                                               (sub(kCtxHmOlRSThr, RelativeScore) < 0)))
                {
                    /* CtxHm is likely better */
                    sqTargetBits = sub(sqTargetBits, NumIndexBits);
                    ctxHmBits = add(ctxHmBits, NumIndexBits);
                    prm_hm[0] = 1;
                    move16();
                    prm_hm[1] = PeriodicityIndex;
                    move16();
                    *prm_lastnz = lastnzCtxHm;
                    move16();
                    sqBits_noStop = sqBits = sqBitsCtxHm;
                    move16();
                    move16();
                    nEncoded = nEncodedCtxHm;
                    move16();
                    stop = stopCtxHm;
                    move16();
                }
                ELSE   /* Original is better or not much difference */
                {
                    prm_hm[0] = 0;
                    move16();
                    prm_hm[1] = PeriodicityIndex;
                    move16();
                    *prm_lastnz = lastnz;
                    move16();
                    PeriodicityIndex = -1;
                    move16();

                    sqBits_noStop = sqBits;
                    move16();
                }


                if (stop != 0)
                {

                    sqBits = stop;
                    move16();
                }
            }
            ELSE   /* no context hm*/
            {
                PeriodicityIndex = -1;
                move16();

                sqGain = SQ_gain(spectrum, *spectrum_e,
                shl(mult(LPDmem->tcx_target_bits_fac, sqTargetBits), 1),
                L_spec,
                &sqGain_e);

                /* Quantize spectrum */

                tcx_scalar_quantization(spectrum, *spectrum_e,
                sqQ,
                L_spec,
                sqGain, sqGain_e,
                tcx_cfg->sq_rounding,
                st->memQuantZeros,
                tcxonly
                                       );

                /* Estimate bitrate */
                stop = 0;
                move16();
                sqBits = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                L_spec,
                prm_lastnz, /* lastnz */
                &nEncoded,
                sqTargetBits,
                &stop,
                NULL);

                sqBits_noStop = sqBits;
                move16();

                if (stop != 0)
                {
                    sqBits = stop;
                    move16();
                }
            } /* end of if (ctx_hm) */

            /* Adjust correction factor */
            tmp1 = sqBits;
            move16();

            if (s_and(L_spec, sub(L_spec, 1)) == 0)   /* power-of-2 */
            {
                tmp1 = add(tmp1, 1);
            }

            tmp1 = BASOP_Util_Divide1616_Scale(sqTargetBits, tmp1, &tmp2);
            BASOP_SATURATE_WARNING_OFF
            LPDmem->tcx_target_bits_fac = shl(mult(LPDmem->tcx_target_bits_fac, tmp1), tmp2);
            BASOP_SATURATE_WARNING_ON

            if (sub(LPDmem->tcx_target_bits_fac, 0x5000) > 0)
            {
                LPDmem->tcx_target_bits_fac = 0x5000;
                move16();
            }
            if (sub(LPDmem->tcx_target_bits_fac, 0x3000) < 0)
            {
                LPDmem->tcx_target_bits_fac = 0x3000;
                move16();
            }

            /* Refine quantizer step size with a rate-control-loop (optional) */
            sqBits = tcx_scalar_quantization_rateloop(spectrum, *spectrum_e,
                     sqQ,
                     L_spec,
                     &sqGain, &sqGain_e,
                     tcx_cfg->sq_rounding,
                     st->memQuantZeros,
                     prm_lastnz, /* lastnz */
                     sqTargetBits,
                     &nEncoded,
                     &stop,
                     sqBits_noStop,
                     sqBits,
                     tcx_cfg->tcxRateLoopOpt,
                     tcxonly,
                     PeriodicityIndex >= 0 ? hm_cfg : NULL
                                                     );

            IF (ctxHmBits > 0)   /* Mapping tool is enabled */
            {
                /* Truncate spectrum */
                set16_fx(sqQ+nEncoded, 0, sub(L_spec, nEncoded));

                IF (PeriodicityIndex >= 0)   /* Mapping is used */
                {
                    /* Estimate non-mapped bitrate */
                    stopCtxHm = 1;
                    move16();

                    sqBitsCtxHm = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                                  L_spec,
                                  &lastnz,
                                  &nEncodedCtxHm,
                                  sqTargetBits,
                                  &stopCtxHm,
                                  NULL);

                    /* Decide whether or not to revert mapping */
                    Selector = sub(sqBits, add(sqBitsCtxHm, NumIndexBits));

                    test();
                    IF (stopCtxHm == 0 && Selector > 0)   /* Non-mapped is better */
                    {
                        sqTargetBits = add(sqTargetBits, NumIndexBits);
                        ctxHmBits  = sub(ctxHmBits, NumIndexBits);
                        prm_hm[0] = 0;
                        move16();
                        *prm_lastnz = lastnz;
                        move16();
                        PeriodicityIndex = -1;
                        move16();
                        sqBits_noStop = sqBits = sqBitsCtxHm;
                        move16();
                        move16();
                        nEncoded = nEncodedCtxHm;
                        move16();
                        stop = stopCtxHm;
                        move16();
                    }
                }
                ELSE   /* Mapping is not used */
                {
                    /* Estimate mapped bitrate */
                    stopCtxHm = 1;
                    move16();
                    sqBitsCtxHm = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                    L_spec,
                    &lastnzCtxHm,
                    &nEncodedCtxHm,
                    sub(sqTargetBits, NumIndexBits),
                    &stopCtxHm,
                    hm_cfg
                                                                                 );

                    /* Decide whether or not to use mapping */
                    Selector = sub(sqBits, add(sqBitsCtxHm, NumIndexBits));

                    test();
                    IF (stopCtxHm == 0 && Selector > 0)   /* Mapped is better */
                    {
                        sqTargetBits = sub(sqTargetBits, NumIndexBits);
                        ctxHmBits  = add(ctxHmBits, NumIndexBits);
                        prm_hm[0] = 1;
                        move16();
                        *prm_lastnz = lastnzCtxHm;
                        move16();
                        PeriodicityIndex = prm_hm[1];
                        move16();
                        sqBits_noStop = sqBits = sqBitsCtxHm;
                        move16();
                        move16();
                        nEncoded = nEncodedCtxHm;
                        move16();
                        stop = stopCtxHm;
                        move16();
                    }
                }
            }

            /* Limit low sqGain for avoiding saturation of the gain quantizer*/
            tmp1 = mult_r(shl(L_spec, 5), FL2WORD16(128.f/NORM_MDCT_FACTOR));
            s = 15-5-7;
            tmp1 = ISqrt16(tmp1, &s);

            tmp2 = sub(sqGain_e, s);
            IF (tmp2 >= 0)
            {
                BASOP_SATURATE_WARNING_OFF;
                tmp2 = sub(sqGain, shr(tmp1, tmp2));
                BASOP_SATURATE_WARNING_ON;
            }
            ELSE
            {
                tmp2 = sub(shl(sqGain, s_max(-15, tmp2)), tmp1);
            }

            IF (tmp2 < 0)
            {
                sqGain = tmp1;
                sqGain_e = s;

                tcx_scalar_quantization( spectrum, *spectrum_e,
                                         sqQ,
                                         L_spec,
                                         sqGain, sqGain_e,
                                         tcx_cfg->sq_rounding,
                                         st->memQuantZeros,
                                         tcxonly
                                       );

                move16();
                stop=1;

                sqBits = ACcontextMapping_encode2_estimate_no_mem_s17_LC(sqQ,
                         L_spec,
                         prm_lastnz,
                         &nEncoded,
                         sqTargetBits,
                         &stop,
                         PeriodicityIndex >= 0 ? hm_cfg : NULL
                                                                        );
            }

            /* Truncate spectrum (for CBR) */
            IF (stop != 0)
            {
                set16_fx(sqQ+nEncoded, 0, sub(L_spec, nEncoded));
            }

            /* Save quantized Values */
            tmp32 = L_deposit_l(0);
            FOR(i=0; i<L_spec; i++)
            {
                spectrum[i] = L_mult(sqQ[i], 1 << (30 - SPEC_EXP_DEC));
                move32();
                /* noise filling seed */
                tmp32 = L_macNs(tmp32, abs_s(sqQ[i]), i);
            }
            *spectrum_e = SPEC_EXP_DEC;
            move16();

            nf_seed = extract_l(tmp32);

        }
        ELSE    /* low rates: new arithmetic coder */
        {
            AdaptLowFreqEmph(spectrum, *spectrum_e,
            NULL,
            0, 0,
            st->tcx_lpc_shaped_ari,
            gainlpc, gainlpc_e,
            L_frame
                            );

            prm_target = sqQ;
            move16();
            sqQ = prm_target + 1;
            move16();
            signs = hm_cfg->indexBuffer;
            move16();

            LtpPitchLag = -1;
            move16();

            IF (sub(st->tcxltp_pitch_int, st->L_frame_fx) < 0)
            {
                tmp32 = L_shl(L_mult0(st->L_frame_fx, st->pit_res_max), 1+kLtpHmFractionalResolution+1);
                tmp1 = add(imult1616(st->tcxltp_pitch_int, st->pit_res_max), st->tcxltp_pitch_fr);
                LtpPitchLag = div_l(tmp32, tmp1);
            }

            tmp8 = 1;
            move16();
            if (sub(st->last_core_fx, ACELP_CORE) == 0)
            {
                tmp8 = 0;
                move16();
            }

            tcx_arith_encode_envelope(
                spectrum,
                spectrum_e,
                signs,
                L_frame,
                L_spec,
                st,
                Aqind,
                sqTargetBits,
                sqQ,
                tmp8,
                prm_hm, /* HM parameter area */
                LtpPitchLag,
                &sqBits,
                &signaling_bits,
                &nf_seed
                ,(st->bwidth_fx > WB)?1:0
            );

            sqTargetBits = sub(sqTargetBits, signaling_bits);
            *prm_target = sqTargetBits;
            move16();
        }

        /*-----------------------------------------------------------*
         * Compute optimal TCX gain.                                 *
         *-----------------------------------------------------------*/
        /* initialize LF deemphasis factors in xn_buf */
        set16_fx(xn_buf16, 0x4000, L_spec);

        IF (tcxonly == 0)
        {

            AdaptLowFreqDeemph(spectrum, *spectrum_e,
                               st->tcx_lpc_shaped_ari,
                               gainlpc, gainlpc_e,
                               L_frame,
                               xn_buf16 /* LF deemphasis factors */
                              );
        }

        tcx_get_gain(x_orig, x_orig_e,
                     spectrum, *spectrum_e,
                     L_spec,
                     &gain_tcx_opt, &gain_tcx_opt_e,
                     &ener, &ener_e);

        IF (gain_tcx_opt <= 0)
        {
            gain_tcx_opt = sqGain;
            move16();
            gain_tcx_opt_e = sqGain_e;
            move16();
        }
        gain_tcx = gain_tcx_opt;
        move16();
        gain_tcx_e = gain_tcx_opt_e;
        move16();

        /* Save gains for FAC and Residual Q*/

        /*-----------------------------------------------------------*
         * Quantize TCX gain                                         *
         *-----------------------------------------------------------*/

        IF (L_sub(st->total_brate_fx, ACELP_13k20) >= 0 && st->rf_mode == 0 )
        {
            QuantizeGain(L_spec, &gain_tcx, &gain_tcx_e, &prm[0]);
        }


        /*-----------------------------------------------------------*
         * Residual Quantization                                     *
         *-----------------------------------------------------------*/

        IF (tcx_cfg->resq)
        {

            resQTargetBits = sub(sqTargetBits, sqBits);

            IF (st->tcx_lpc_shaped_ari)   /* new arithmetic coder */
            {
                Word16 *prm_resq;

                prm_resq = sqQ + sqTargetBits - resQTargetBits;

                resQBits = tcx_ari_res_Q_spec(x_orig, x_orig_e, signs, spectrum, *spectrum_e, L_spec, gain_tcx, gain_tcx_e,
                                              prm_resq,
                                              resQTargetBits,
                                              resQBits,
                                              tcx_cfg->sq_rounding,
                                              xn_buf16 /* LF deemphasis factors */ );

                /* Transmit zeros when there bits remain after RESQ */
                set16_fx(prm_resq+resQBits, 0, sub(resQTargetBits, resQBits));
            }
            ELSE   /* old arithmetic coder */
            {
                move16();
                tmpP16 = NULL;
                if (tcxonly == 0)
                {
                    move16();
                    tmpP16 = xn_buf16;
                }

                resQBits = tcx_res_Q_gain(gain_tcx_opt, gain_tcx_opt_e,
                &gain_tcx, &gain_tcx_e,
                &sqQ[L_spec],
                resQTargetBits);

                resQBits = tcx_res_Q_spec(x_orig, x_orig_e,
                spectrum, *spectrum_e,
                L_spec,
                gain_tcx, gain_tcx_e,
                &sqQ[L_spec],
                resQTargetBits,
                resQBits,
                tcx_cfg->sq_rounding,
                tmpP16 /* LF deemphasis factors */ );
            }


        }


        /*-----------------------------------------------------------*
         * ALFE tcx only bitrates                                    *
         *-----------------------------------------------------------*/

        IF (st->tcxonly != 0)
        {
            test();
            test();
            IF (st->tcxltp != 0 && (st->tcxltp_gain > 0) && fUseTns == 0)
            {

                PsychAdaptLowFreqDeemph(spectrum, gainlpc, gainlpc_e, NULL);
            }
        }

        /*-----------------------------------------------------------*
         * TCX SNR for Analysis purposes                             *
         *-----------------------------------------------------------*/

        {
            maxNfCalcBw = s_min(noiseFillingSize, round_fx(L_shl(L_mult(st->measuredBwRatio, L_frame), 1)));

            /*-----------------------------------------------------------*
             * Estimate and quantize noise factor                        *
             *-----------------------------------------------------------*/

            IF (L_sub(st->total_brate_fx, HQ_96k) >= 0)
            {
                fac_ns = 0;
                move16();
                prm[1] = 0;
                move16();
            }
            ELSE
            {
                /* noise filling start bin */
                i = shr(L_frame, 3);
                IF (L_sub(st->total_brate_fx, ACELP_13k20) >= 0 && st->rf_mode == 0 )
                {
                    i = idiv1616U(L_frame, 6);
                }

                IF (tcxonly)
                {
                    tmp1 = 0;
                    move16();
                    test();
                    test();
                    if ((tcx_cfg->ctx_hm != 0) && (st->last_core_fx != ACELP_CORE) && (prm_hm[0] != 0))
                    {
                        tmp1 = FL2WORD16(0.3125f);
                        move16();
                    }
                    noiseTransWidth = HOLE_SIZE_FROM_LTP(s_max(st->tcxltp_gain, tmp1));

                    if (sub(L_frame, shr(st->L_frame_fx, 1)) == 0)
                    {
                        /* minimum transition for noise filling in TCX-10 */
                        noiseTransWidth = 3;
                        move16();
                    }
                }

                tcx_noise_factor( x_orig, x_orig_e,
                                  spectrum,
                                  i,
                                  maxNfCalcBw,
                                  noiseTransWidth,
                                  L_frame,
                                  gain_tcx, gain_tcx_e,
                                  st->noiseTiltFactor,
                                  &fac_ns, &prm[NOISE_FILL_RANGES] );

                /* hysteresis for very tonal passages (more stationary noise filling level) */

                IF (sub(prm[NOISE_FILL_RANGES], 1) == 0)
                {
                    st->noiseLevelMemory = add(1, abs_s(st->noiseLevelMemory));   /* update counter */
                }
                ELSE {
                    test();
                    IF ((sub(prm[NOISE_FILL_RANGES], 2) == 0) &&
                    (sub(abs_s(st->noiseLevelMemory), 5) > 0))
                    {
                        /* reduce noise filling level by one step */
                        prm[NOISE_FILL_RANGES] = 1;
                        move16();
                        fac_ns = shr(0x6000, NBITS_NOISE_FILL_LEVEL);

                        /* signal that noise level is changed by inverting sign of level memory */
                        tmp1 = 5;
                        move16();
                        if (st->noiseLevelMemory >= 0)
                        {
                            tmp1 = sub(-1, st->noiseLevelMemory);
                        }
                        st->noiseLevelMemory = tmp1;
                    }
                    ELSE {
                        /* reset memory since level is too different */
                        st->noiseLevelMemory = 0;
                        move16();
                    }
                }

            } /* bitrate */
        }

        /* Free encoder specific stack to use it below for the internal TCX decoder. */
    }

    /*-----------------------------------------------------------*
     * Internal TCX decoder                                      *
     *-----------------------------------------------------------*/
    {
        /* Overlay of a 16-bit buffer xn_buf16 with a 32-bit buffer xn_buf32 */
        /* The size is determined by the requirements of the 16-bit buffer.  */
        Word32 xn_buf32[(L_MDCT_OVLP_MAX+L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2];
        Word16 *xn_buf16 = (Word16*)xn_buf32;

        /*Enable internal TCX decoder to run always to update LPD memory for rate switching */

        IF (tcxonly == 0)
        {
        }

        /*-----------------------------------------------------------*
         * Noise Filling.                                            *
         *-----------------------------------------------------------*/

        /* Replication of ACELP formant enhancement for low rates */
        IF ( L_sub(st->total_brate_fx, ACELP_13k20) < 0  || st->rf_mode != 0)
        {
            tcxFormantEnhancement(xn_buf16, gainlpc, gainlpc_e, spectrum, spectrum_e, L_frame, L_spec);
        }

        IF (fac_ns > 0)
        {
            tmp1 = 0;
            move16();
            if ( L_sub(st->total_brate_fx, ACELP_13k20) >= 0 && st->rf_mode == 0)
            {
                tmp1 = 1;
                move16();
            }

            i = tcxGetNoiseFillingTilt(A,
                                       M,
                                       L_frame,
                                       tmp1,
                                       &st->noiseTiltFactor);

            tcx_noise_filling(spectrum, *spectrum_e,
                              nf_seed /* seed */,
                              i,
                              noiseFillingSize,
                              noiseTransWidth,
                              L_frame,
                              st->noiseTiltFactor,
                              fac_ns,
                              NULL
                             );
        }

        IF (L_sub(st->total_brate_fx, ACELP_13k20) < 0 || st->rf_mode != 0)
        {
            /* partially recompute global gain (energy part), taking noise filling and formant enhancement into account */
            s = sub(getScaleFactor32(spectrum, L_spec), 4);
            tmp32 = L_deposit_l(1);

            FOR (i = 0; i < L_spec; i++)
            {
                tmp1 = round_fx(L_shl(spectrum[i], s));
                tmp32 = L_mac0(tmp32, tmp1, tmp1);
            }

            tmp1 = BASOP_Util_Divide3232_Scale(ener, tmp32, &tmp2);
            tmp2 = add(tmp2, sub(ener_e, add(shl(sub(*spectrum_e, s), 1), 1)));
            tmp1 = Sqrt16(tmp1, &tmp2);

            gain_tcx = mult(gain_tcx, tmp1);
            gain_tcx_e = add(gain_tcx_e, tmp2);

            QuantizeGain(L_spec, &gain_tcx, &gain_tcx_e, &prm[0]);
        }

        /*end of noise filling*/

        /*-----------------------------------------------------------*
         * Noise shaping in frequency domain (1/Wz)                  *
         *-----------------------------------------------------------*/

        /* LPC gains already available */
        mdct_shaping(spectrum, L_frame, gainlpc, gainlpc_e);

        /*-----------------------------------------------------------*
         * Apply gain                                                *
         *-----------------------------------------------------------*/
        IF (sub(st->tcx_cfg.coder_type, INACTIVE) == 0 )
        {
            gain_tcx = mult_r(gain_tcx, tcx_cfg->na_scale);
        }

        FOR (i = 0; i < L_spec; i++)
        {
            spectrum[i] = Mpy_32_16_1(spectrum[i], gain_tcx);
            move32();
        }
        *spectrum_e = add(*spectrum_e, gain_tcx_e);
        move16();

        stop = tcx_cfg->tcx_last_overlap_mode;   /* backup last TCX overlap mode */                         move16();

        test();
        IF ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (tcxonly != 0))
        {
            Word16 L = L_frame;
            move16();

            test();
            test();
            if (((tcx_cfg->fIsTNSAllowed != 0) && (fUseTns != 0)) || (sub(L_spec, L_frame) > 0))
            {
                L = L_spec;
                move16();
            }

            tcxInvertWindowGrouping(tcx_cfg,
                                    xn_buf32,
                                    spectrum,
                                    L,
                                    fUseTns,
                                    st->last_core_fx,
                                    stop,
                                    frame_cnt,
                                    0);
        }

        /*-----------------------------------------------------------*
         * Temporal Noise Shaping Synthesis                          *
         *-----------------------------------------------------------*/

        IF (tcx_cfg->fIsTNSAllowed != 0)
        {
            test();
            test();
            test();
            SetTnsConfig(tcx_cfg, sub(L_frame_glob, st->L_frame_fx) == 0, (st->last_core_fx == ACELP_CORE) && (frame_cnt == 0));

            /* Apply TNS to get the reconstructed signal */
            IF (fUseTns != 0)
            {
                ApplyTnsFilter(tcx_cfg->pCurrentTnsConfig, pTnsData, spectrum, 0);

                test();
                IF ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (tcxonly != 0))
                {
                    test();
                    test();
                    test();
                    IF ( (tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) ||
                         ((tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP) && (frame_cnt == 0) && (stop == 0)) )
                    {
                        tmp1 = shr(L_spec, 1);
                        /* undo rearrangement of LF sub-window lines for TNS synthesis filter */
                        assert(L_frame <= L_spec);
                        Copy32(spectrum+8, xn_buf32, 8);
                        Copy32(spectrum+16, spectrum+8, sub(tmp1,8));
                        Copy32(xn_buf32, spectrum+tmp1, 8);
                    }
                }
            }
        }

        {
            /* normalize spectrum to minimize IMDCT noise */
            s = getScaleFactor32(spectrum, L_frame);
            FOR (i = 0; i < L_frame; i++)
            {
                spectrum[i] = L_shl(spectrum[i], s);
                move32();
            }
            *spectrum_e = sub(*spectrum_e, s);

            /*-----------------------------------------------------------*
            * Compute inverse MDCT of spectrum[].                        *
            *-----------------------------------------------------------*/
            test();
            IF ((sub(L_frame, shr(st->L_frame_fx, 1)) == 0) && (tcxonly != 0))
            {
                IF (tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP)
                {
                    Word16 win[(L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2];
                    Word16 L_win, L_spec_TCX5, L_ola, w;

                    /* minimum or half overlap, two transforms, grouping into one window */
                    L_win = shr(L_frame, 1) /*(4 - tcx_cfg->tcx_last_overlap_mode)*/;
                    L_spec_TCX5 = shr(s_max(L_frame, L_spec), 1);
                    L_ola = tcx_cfg->tcx_mdct_window_half_length;
                    move16();
                    if ( sub(tcx_cfg->tcx_last_overlap_mode, MIN_OVERLAP) == 0 )
                    {
                        L_ola = tcx_cfg->tcx_mdct_window_min_length;
                        move16();
                    }

                    set16_fx(win, 0, (L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2);
                    set16_fx(xn_buf16, 0, add(tcx_offset, shr(L_ola, 1)));  /* zero left end of buffer */

                    FOR (w = 0; w < 2; w++)
                    {

                        IF (sub(tcx_cfg->tcx_last_overlap_mode, MIN_OVERLAP) == 0)
                        {
                            TCX_MDCT_Inverse(spectrum + L_mult0(w, L_spec_TCX5), sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                                             win,
                                             L_ola, sub(L_win, L_ola), L_ola);
                        }
                        ELSE
                        {
                            TCX_MDCT_Inverse(spectrum + L_mult0(w, L_spec_TCX5), sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                            win,
                            L_ola, sub(L_win, L_ola), L_ola);
                        }

                        tmp1 = tcx_cfg->tcx_last_overlap_mode;
                        move16();
                        test();
                        test();
                        if ((w > 0) || ((w == 0) && (sub(stop, 2) == 0)))
                        {
                            tmp1 = MIN_OVERLAP;
                            move16();
                        }

                        tmp2 = 0;
                        move16();
                        test();
                        if ((w == 0) && (st->last_core_fx == ACELP_CORE))
                        {
                            tmp2 = 1;
                            move16();
                        }

                        tmp3 = st->last_core_fx;
                        move16();
                        if (w > 0)
                        {
                            tmp3 = 1;
                            move16();
                        }

                        tmp4 = 0;
                        move16();
                        if (tcx_offset < 0)
                        {
                            tmp4 = negate(tcx_offset);
                        }

                        tcx_windowing_synthesis_current_frame(win,
                                                              tcx_cfg->tcx_aldo_window_2,
                                                              tcx_cfg->tcx_mdct_window_half,
                                                              tcx_cfg->tcx_mdct_window_minimum,
                                                              L_ola,
                                                              tcx_cfg->tcx_mdct_window_half_length,
                                                              tcx_cfg->tcx_mdct_window_min_length,
                                                              tmp2,
                                                              tmp1,
                                                              LPDmem->acelp_zir,
                                                              st->LPDmem.Txnq,
                                                              LPDmem->acelp_zir+shr(L_frame,1),
                                                              Aq_old,
                                                              tcx_cfg->tcx_mdct_window_trans,
                                                              L_win,
                                                              tmp4,
                                                              0,
                                                              tmp3,
                                                              0
                                                              ,0
                                                             );

                        tmp1 = add(tcx_offset, imult1616(w, L_win));
                        move16();
                        tmpP16 = xn_buf16 + sub(tmp1, shr(L_ola, 1));

                        IF (w > 0)
                        {
                            tcx_windowing_synthesis_past_frame(tmpP16,
                                                               tcx_cfg->tcx_aldo_window_1_trunc,
                                                               tcx_cfg->tcx_mdct_window_half,
                                                               tcx_cfg->tcx_mdct_window_minimum,
                                                               L_ola,
                                                               tcx_cfg->tcx_mdct_window_half_length,
                                                               tcx_cfg->tcx_mdct_window_min_length,
                                                               2
                                                              );
                        }
                        /* add part of current sub-window overlapping with previous window */
                        FOR (i = 0; i < L_ola; i++)
                        {
                            tmpP16[i] = add(tmpP16[i], win[i]);
                            move16();
                        }
                        /* copy new sub-window region not overlapping with previous window */
                        Copy(win + L_ola, xn_buf16 + add(tmp1, shr(L_ola, 1)), L_win);
                    }
                    /* To assure that no garbage values are copied to LPDmem->Txnq */
                    set16_fx(xn_buf16 + add(add(L_frame, tcx_offset), shr(L_ola, 1)),
                             0, sub(sub(overlap, tcx_offset), shr(L_ola, 1)));


                }
                ELSE IF ( s_and(frame_cnt == 0, (tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP)) )
                {
                    /* special overlap attempt, two transforms, grouping into one window */
                    Word16 win[(L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2];
                    Word16 L_win, L_spec_TCX5, L_ola, w;

                    L_win = shr(L_frame, 1);
                    L_spec_TCX5 = shr(s_max(L_frame, L_spec), 1);
                    L_ola = tcx_cfg->tcx_mdct_window_min_length;
                    move16();

                    set16_fx(win, 0, (L_FRAME_PLUS+L_MDCT_OVLP_MAX)/2);

                    /* Resize overlap (affect only asymmetric window)*/
                    overlap = st->tcx_cfg.tcx_mdct_window_delay;
                    /* 1st TCX-5 window, special MDCT with minimum overlap on right side */
                    TCX_MDCT_Inverse(spectrum, sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                                     win + L_win,
                                     0, sub(L_win, shr(L_ola, 1)), L_ola);

                    /* copy new sub-window region not overlapping with previous window */
                    Copy(win+L_win, xn_buf16+shr(overlap, 1), add(L_win, shr(L_ola, 1)));

                    /* 2nd TCX-5 window, regular MDCT with minimum overlap on both sides */

                    TCX_MDCT_Inverse(spectrum + L_spec_TCX5, sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                                     win,
                                     L_ola, sub(L_win, L_ola), L_ola);

                    tmp4 = 0;
                    move16();
                    if (tcx_offset <0)
                    {
                        tmp4 = negate(tcx_offset);
                    }
                    tcx_windowing_synthesis_current_frame(win,
                                                          tcx_cfg->tcx_aldo_window_2,
                                                          tcx_cfg->tcx_mdct_window_half,
                                                          tcx_cfg->tcx_mdct_window_minimum,
                                                          L_ola,
                                                          tcx_cfg->tcx_mdct_window_half_length,
                                                          tcx_cfg->tcx_mdct_window_min_length,
                                                          0,  /* left_rect */
                                                          2,  /* left_mode */
                                                          LPDmem->acelp_zir,
                                                          st->LPDmem.Txnq,
                                                          LPDmem->acelp_zir+shr(L_frame,1),
                                                          Aq_old,
                                                          tcx_cfg->tcx_mdct_window_trans,
                                                          L_win,
                                                          tmp4,
                                                          0,
                                                          1, /* not LPDmem->mode */
                                                          0
                                                          ,0
                                                         );


                    move16();
                    tmpP16 = xn_buf16 + add(sub(L_win, shr(L_ola, 1)), shr(overlap,1));

                    tcx_windowing_synthesis_past_frame(tmpP16,
                                                       tcx_cfg->tcx_aldo_window_1_trunc,
                                                       tcx_cfg->tcx_mdct_window_half,
                                                       tcx_cfg->tcx_mdct_window_minimum,
                                                       L_ola,
                                                       tcx_cfg->tcx_mdct_window_half_length,
                                                       tcx_cfg->tcx_mdct_window_min_length,
                                                       2
                                                      );

                    /* add part of current sub-window overlapping with previous window */
                    FOR (i = 0; i < L_ola; i++)
                    {
                        tmpP16[i] = add(tmpP16[i], win[i]);
                        move16();
                    }

                    /* copy new sub-window region not overlapping with previous window */
                    Copy(win + L_ola,
                         xn_buf16 + add(add(shr(overlap,1), shr(L_ola, 1)), L_win),
                         L_win);

                    /* extra folding-out on left side of win, for perfect reconstruction */
                    FOR (w = shr(overlap,1); w < overlap; w++)
                    {
                        xn_buf16[overlap-1-w] = negate(xn_buf16[w]);
                        move16();
                    }

                    tmp4 = 0;
                    move16();
                    if (tcx_offset < 0)
                    {
                        tmp4 = negate(tcx_offset);
                    }
                    tcx_windowing_synthesis_current_frame(xn_buf16,
                                                          tcx_cfg->tcx_aldo_window_2,
                                                          tcx_cfg->tcx_mdct_window_half,
                                                          tcx_cfg->tcx_mdct_window_minimum,
                                                          overlap,
                                                          tcx_cfg->tcx_mdct_window_half_length,
                                                          tcx_cfg->tcx_mdct_window_min_length,
                                                          st->last_core_fx==ACELP_CORE,
                                                          0,
                                                          LPDmem->acelp_zir,
                                                          st->LPDmem.Txnq,
                                                          LPDmem->acelp_zir+shr(L_frame,1),
                                                          Aq_old,
                                                          tcx_cfg->tcx_mdct_window_trans,
                                                          L_win,
                                                          tmp4,
                                                          0,
                                                          st->last_core_fx,
                                                          0
                                                          ,0
                                                         );
                }
                ELSE   /* default, i.e. maximum overlap, single transform, no grouping */
                {

                    TCX_MDCT_Inverse(spectrum, sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                    xn_buf16,
                    overlap, sub(L_frame, overlap), overlap);

                    tmp1 = stop;
                    move16();
                    test();
                    test();
                    if ((frame_cnt > 0) && (stop == 0) && (st->last_core_fx != ACELP_CORE))
                    {
                        tmp1 = 2;
                        move16();
                    }

                    tmp4 = 0;
                    move16();
                    if (tcx_offset <0)
                    {
                        tmp4 = negate(tcx_offset);
                    }
                    tcx_windowing_synthesis_current_frame(xn_buf16,
                    tcx_cfg->tcx_aldo_window_2,
                    tcx_cfg->tcx_mdct_window_half,
                    tcx_cfg->tcx_mdct_window_minimum,
                    overlap, /*tcx_cfg->tcx_mdct_window_length*/
                    tcx_cfg->tcx_mdct_window_half_length,
                    tcx_cfg->tcx_mdct_window_min_length,
                    st->last_core_fx==ACELP_CORE,
                    tmp1,
                    LPDmem->acelp_zir,
                    st->LPDmem.Txnq,
                    LPDmem->acelp_zir+shr(L_frame,1),
                    Aq_old,
                    tcx_cfg->tcx_mdct_window_trans,
                    shr(L_frame_glob, 1),
                    tmp4,
                    0,
                    st->last_core_fx,
                    0
                    ,0
                                                         );

                } /* tcx_last_overlap_mode > 0 */

            }
            ELSE   /* frame is TCX-20 or not TCX-only */
            {
                IF (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) != 0)
                {
                    Word32 tmp_buf[L_FRAME_PLUS];
                    Word16 Q;

                    /* DCT */
                    Q = sub(31, *spectrum_e);
                    edct_fx(spectrum, tmp_buf, L_frame, &Q);

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
                    st->Q_old_out = add(st->Q_old_out, tmp1);
                    move16();
                    FOR (i = 0; i < L_frame; i++)
                    {
                        st->old_out_fx[i] = shl(st->old_out_fx[i], tmp1);
                        move16();
                    }

                    window_ola_fx(tmp_buf,
                                  xn_buf16,
                                  &Q,
                                  st->old_out_fx,
                                  &st->Q_old_out,
                                  L_frame,
                                  tcx_cfg->tcx_last_overlap_mode,
                                  tcx_cfg->tcx_curr_overlap_mode,
                                  0,
                                  0,
                                  NULL);

                    /* scale output */
                    FOR (i = 0; i < L_frame; i++)
                    {
                        xn_buf16[i] = shr(xn_buf16[i], Q);
                        move16();
                    }

                    aldo = 1;
                    move16();
                }
                ELSE
                {

                    TCX_MDCT_Inverse(spectrum, sub(*spectrum_e, TCX_IMDCT_SCALE+TCX_IMDCT_HEADROOM),
                    xn_buf16,
                    overlap, sub(L_frame, overlap), overlap);

                    /*-----------------------------------------------------------*
                     * Windowing, overlap and add                                *
                     *-----------------------------------------------------------*/

                    /* Window current frame */
                    tmp4 = 0;
                    move16();
                    if (tcx_offset<0)
                    {
                        tmp4 = negate(tcx_offset);
                    }
                    tcx_windowing_synthesis_current_frame(xn_buf16,
                    tcx_cfg->tcx_aldo_window_2,
                    tcx_cfg->tcx_mdct_window_half,
                    tcx_cfg->tcx_mdct_window_minimum,
                    overlap, /*tcx_cfg->tcx_mdct_window_length*/
                    tcx_cfg->tcx_mdct_window_half_length,
                    tcx_cfg->tcx_mdct_window_min_length,
                    st->last_core_fx==ACELP_CORE,
                    tcx_cfg->tcx_last_overlap_mode, /*left mode*/
                    LPDmem->acelp_zir,
                    st->LPDmem.Txnq,
                    LPDmem->acelp_zir+shr(L_frame,1),
                    Aq_old,
                    tcx_cfg->tcx_mdct_window_trans,
                    shr(L_frame_glob, 1),
                    tmp4,
                    0,
                    st->last_core_fx,
                    0
                    ,0
                                                         );
                }
            } /* TCX-10 and TCX-only */

            /* Window and overlap-add past frame if past frame is TCX */
            test();
            test();
            test();
            IF ((st->last_core_fx > ACELP_CORE) && (((sub(L_frameTCX, shr(st->L_frameTCX, 1)) == 0) && (st->tcxonly != 0)) || (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) == 0)))
            {

                IF (tcx_cfg->last_aldo != 0)
                {
                    tmp2 = add(st->Q_old_out, TCX_IMDCT_HEADROOM);

                    tmp1 = sub(overlap, tcx_cfg->tcx_mdct_window_min_length);
                    FOR (i=0; i < tmp1; i++)
                    {
                        xn_buf16[i] = shl(add(xn_buf16[i], shr(st->old_out_fx[i+nz], tmp2)), TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    /* fade truncated ALDO window */
                    tmp1 = sub(overlap, shr(tcx_cfg->tcx_mdct_window_min_length, 1));
                    FOR ( ; i < tmp1; i++)
                    {
                        tmp3 = mult_r(shr(st->old_out_fx[i+nz], tmp2), tcx_cfg->tcx_mdct_window_minimum[i-overlap+tcx_cfg->tcx_mdct_window_min_length].v.re);
                        xn_buf16[i] = shl(add(xn_buf16[i], tmp3), TCX_IMDCT_HEADROOM);
                        move16();
                    }
                    FOR ( ; i < overlap; i++)
                    {
                        tmp3 = mult_r(shr(st->old_out_fx[i+nz], tmp2), tcx_cfg->tcx_mdct_window_minimum[overlap-1-i].v.im);
                        xn_buf16[i] = shl(add(xn_buf16[i], tmp3), TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    FOR ( ; i < L_frame; i++)
                    {
                        xn_buf16[i] = shl(xn_buf16[i], TCX_IMDCT_HEADROOM);
                        move16();
                    }
                }
                ELSE
                {
                    test();
                    test();
                    test();
                    if ((frame_cnt > 0) && (stop == 0) && (tcx_cfg->tcx_curr_overlap_mode == 0) && (st->last_core_fx != ACELP_CORE))
                    {
                        stop = 2;     /* use minimum overlap between the two TCX-10 windows */                  move16();
                    }

                    tmp1 = stop;
                    move16();
                    test();
                    if ((stop == 0) || (sub(tcx_cfg->tcx_last_overlap_mode, MIN_OVERLAP) == 0))
                    {
                        tmp1 = tcx_cfg->tcx_last_overlap_mode;
                        move16();
                    }

                    tcx_windowing_synthesis_past_frame( LPDmem->Txnq+Txnq_offset,
                    tcx_cfg->tcx_aldo_window_1_trunc,
                    tcx_cfg->tcx_mdct_window_half,
                    tcx_cfg->tcx_mdct_window_minimum,
                    overlap,
                    tcx_cfg->tcx_mdct_window_half_length,
                    tcx_cfg->tcx_mdct_window_min_length,
                    tmp1
                                                      );

                    BASOP_SATURATE_WARNING_OFF;
                    FOR (i=0; i<overlap; i++)
                    {
                        xn_buf16[i] = shl(add(xn_buf16[i], LPDmem->Txnq[i+Txnq_offset]), TCX_IMDCT_HEADROOM);
                        move16();
                    }

                    IF (sub(i, L_frame) < 0)
                    {
                        FOR ( ; i < L_frame; i++)
                        {
                            xn_buf16[i] = shl(xn_buf16[i], TCX_IMDCT_HEADROOM);
                            move16();
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
                        xn_buf16[i] = shl(xn_buf16[i], TCX_IMDCT_HEADROOM);
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
                    st->old_out_fx[i] = shr(xn_buf16[L_frame-nz+i], TCX_IMDCT_HEADROOM);
                    move16();
                }
                Copy(xn_buf16+L_frame, st->old_out_fx+nz, overlap);
                set16_fx(st->old_out_fx+nz+overlap, 0, nz);

                tcx_windowing_synthesis_past_frame( st->old_out_fx+nz,
                                                    tcx_cfg->tcx_aldo_window_1_trunc,
                                                    tcx_cfg->tcx_mdct_window_half,
                                                    tcx_cfg->tcx_mdct_window_minimum,
                                                    overlap,
                                                    tcx_cfg->tcx_mdct_window_half_length,
                                                    tcx_cfg->tcx_mdct_window_min_length,
                                                    tcx_cfg->tcx_curr_overlap_mode
                                                  );

                /* If current overlap mode = FULL_OVERLAP -> ALDO_WINDOW */
                IF (sub(tcx_cfg->tcx_curr_overlap_mode, FULL_OVERLAP) == 0)
                {
                    FOR (i=0; i<nz; i++)
                    {
                        st->old_out_fx[nz+overlap+i] = shr(mult_r(xn_buf16[L_frame-1-i], tcx_cfg->tcx_aldo_window_1[nz-1-i]), TCX_IMDCT_HEADROOM);
                        move16();
                    }
                    tcx_cfg->tcx_curr_overlap_mode = ALDO_WINDOW;
                    move16();
                }

                st->Q_old_out = -TCX_IMDCT_HEADROOM;
                move16();
            }
            tcx_cfg->last_aldo = aldo;
            move16();

            /* Update Txnq */
            IF (tcx_cfg->last_aldo == 0)
            {
                Copy(xn_buf16 + L_frame, LPDmem->Txnq+Txnq_offset, overlap);
                /* To be sure that sufficient overlap when going from TCX10 to TCX20 with asym windows */
                FOR(i=0; i<Txnq_offset; i++)
                {
                    move16();
                    move16();
                    LPDmem->Txnq[i+overlap+Txnq_offset]=xn_buf16[L_frame-i];
                    LPDmem->Txnq[i]=0;
                }
            }


            /* Output */
            Copy(xn_buf16+shr(overlap,1)-tcx_offset, synth, L_frame_glob);

        }

        /* Free internal TCX decoder stack memory */
    }

    /* Update L_frame_past */
    st->L_frame_past = L_frame;
    move16();

}


void coder_tcx(
    Word16 n,
    TCX_config *tcx_cfg,    /*input: configuration of TCX*/
    Word16 A[],             /* input: quantized coefficients NxAz_q[M+1] */
    Word16 Aqind[],         /* input: frame-independent quantized coefficients (M+1) */
    Word16 synth[],
    Word16 L_frame_glob,    /* input: frame length             */
    Word16 L_frameTCX_glob,
    Word16 L_spec,
    Word16 nb_bits,         /*input: bit budget*/
    Word8 tcxonly,          /*input: only TCX flag*/
    Word32 spectrum[],      /* i/o: MDCT spectrum */
    Word16 *spectrum_e,     /* i/o: MDCT spectrum exponent               */
    LPD_state *LPDmem,      /*i/o: memories*/
    Word16 prm[],           /* output: tcx parameters          */
    Encoder_State_fx *st,
    CONTEXT_HM_CONFIG *hm_cfg
)
{
    Word16 L_frame;
    Word16 left_overlap, right_overlap;
    Word16 tnsSize; /* number of tns parameters put into prm */
    Word16 tnsBits; /* number of tns bits in the frame */
    Word16 ltpBits;
    Word16 gainlpc[FDNS_NPTS], gainlpc_e[FDNS_NPTS];
    Word16 win[N_MAX+L_MDCT_OVLP_MAX];
    Word32 powerSpec[N_MAX];
    Word16 powerSpec_e;
    Word16 winMDST[N_MAX+L_MDCT_OVLP_MAX];
    Word16 *pWinMDST;


    left_overlap = right_overlap = -1;
    move16();
    move16();
    tnsSize = 0;
    move16();
    tnsBits = 0;
    move16();
    ltpBits = 0;
    move16();

    L_frame = L_frameTCX_glob;
    move16();

    /*-----------------------------------------------------------*
     * Windowing                                                 *
     *-----------------------------------------------------------*/
    IF (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) == 0)
    {

        WindowSignal(tcx_cfg,
                     tcx_cfg->tcx_offsetFB,
                     tcx_cfg->tcx_last_overlap_mode,
                     tcx_cfg->tcx_curr_overlap_mode,
                     &left_overlap, &right_overlap,
                     st->speech_TCX,
                     &L_frame,
                     win
                     ,1
                    );

        /*-----------------------------------------------------------*
         * Compute MDCT                                              *
         *-----------------------------------------------------------*/

        *spectrum_e = 16;
        move16();
        TCX_MDCT(win, spectrum, spectrum_e, left_overlap, sub(L_frame, shr(add(left_overlap, right_overlap), 1)), right_overlap);
    }
    ELSE
    {
        Word32 tmp_buf[L_FRAME_PLUS];
        Word16 Q, tmp1, tmp2, i;

        Q = 0;
        move16();

        wtda_fx(st->new_speech_TCX,
        &Q,
        tmp_buf,
        NULL,
        NULL,
        tcx_cfg->tcx_last_overlap_mode,
        tcx_cfg->tcx_curr_overlap_mode,
        L_frame);

        WindowSignal( tcx_cfg, tcx_cfg->tcx_offsetFB,
        tcx_cfg->tcx_last_overlap_mode == ALDO_WINDOW ? FULL_OVERLAP : tcx_cfg->tcx_last_overlap_mode,
        tcx_cfg->tcx_curr_overlap_mode == ALDO_WINDOW ? FULL_OVERLAP : tcx_cfg->tcx_curr_overlap_mode,
        &left_overlap, &right_overlap, st->speech_TCX, &L_frame, winMDST, 1 );

        /* scale by NORM_MDCT_FACTOR / L */
        tmp1 = mult_r(shl(L_frame, 4), FL2WORD16(128.f / NORM_MDCT_FACTOR)); /* 4Q11 */
        tmp2 = 4;
        move16();
        tmp1 = ISqrt16(tmp1, &tmp2);

        FOR (i = 0; i < L_frame; i++)
        {
            tmp_buf[i] = Mpy_32_16_1(tmp_buf[i], tmp1);
            move32();
        }
        Q = sub(Q, tmp2);

        /* DCT */
        edct_fx(tmp_buf, spectrum, L_frame, &Q);
        *spectrum_e = sub(31, Q);
    }


    /*-----------------------------------------------------------*
     * Attenuate upper end of NB spectrum,                       *
     * to simulate ACELP behavior                                *
     *-----------------------------------------------------------*/

    IF (st->narrowBand != 0)
    {
        attenuateNbSpectrum(L_frame, spectrum);
    }

    /*-----------------------------------------------------------*
     * Compute noise-measure flags for spectrum filling          *
     * and quantization (0: tonal, 1: noise-like).               *
     * Detect low pass if present.                               *
     *-----------------------------------------------------------*/

    pWinMDST = winMDST;
    move16();
    if (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP)  == 0)
    {
        pWinMDST = win;
        move16();
    }

    AnalyzePowerSpectrum(st,
                         L_frame*st->L_frame_fx/st->L_frameTCX,
                         L_frame,
                         left_overlap, right_overlap,
                         spectrum, *spectrum_e,
                         pWinMDST,
                         powerSpec, &powerSpec_e);
    IF (tcx_cfg->fIsTNSAllowed != 0)
    {
        test();
        test();
        SetTnsConfig(tcx_cfg, sub(L_frame_glob, st->L_frame_fx) == 0, st->last_core_fx == 0);

        TNSAnalysis(tcx_cfg, L_frame, L_spec, TCX_20, st->last_core_fx == 0, spectrum, st->tnsData, st->fUseTns
                    , &st->hIGFEnc.tns_predictionGain
                   );

    }
    ELSE
    {
        st->fUseTns[0] = st->fUseTns[1] = 0;
        move16();
        move16();
    }

    IF(st->igf)
    {
        ProcessIGF(&st->hIGFEnc, st, spectrum, spectrum_e, powerSpec, &powerSpec_e, 1, st->fUseTns[0], (st->last_core_fx == ACELP_CORE), 0);
    }

    ShapeSpectrum(tcx_cfg, A, gainlpc, gainlpc_e,
                  L_frame_glob,
                  L_spec,
                  spectrum,
                  st->fUseTns[0],
                  st
                 );
    if(st->igf)
    {
        nb_bits = sub(nb_bits, st->hIGFEnc.infoTotalBitsPerFrameWritten);
    }
    IF (tcx_cfg->fIsTNSAllowed != 0)
    {
        EncodeTnsData(tcx_cfg->pCurrentTnsConfig, st->tnsData, prm+1+NOISE_FILL_RANGES+LTPSIZE, &tnsSize, &tnsBits);
    }

    QuantizeSpectrum(tcx_cfg,
                     A,
                     Aqind,
                     gainlpc, gainlpc_e,
                     synth,
                     L_frame_glob,
                     L_frameTCX_glob,
                     L_spec,
                     sub(sub(nb_bits, tnsBits), ltpBits),
                     tcxonly,
                     spectrum, spectrum_e,
                     st->tnsData,
                     st->fUseTns[0],
                     tnsSize,
                     LPDmem,
                     prm,
                     n,
                     st,
                     hm_cfg
                    );

    LPDmem->nbits = add(LPDmem->nbits, add(tnsBits, ltpBits));

}


/*-------------------------------------------------------------------*
* coder_tcx_post()
*
*
*-------------------------------------------------------------------*/

void coder_tcx_post(Encoder_State_fx *st,
                    LPD_state *LPDmem,
                    TCX_config *tcx_cfg,
                    Word16 *synth,
                    const Word16 *A,
                    const Word16 *Ai,
                    Word16 *wsig,
                    Word16 Q_new,
                    Word16 shift
                   )
{
    Word16 xn_buf[L_FRAME_MAX];

    /* TCX output */
    Copy( synth, xn_buf, st->L_frame_fx );


    /*-----------------------------------------------------------*
     * Memory update                                             *
     *-----------------------------------------------------------*/

    /* Update LPDmem (Txnq,syn,syn_pe,old_exc,wsyn,Ai,Aq) */
    tcx_encoder_memory_update( wsig, xn_buf, st->L_frame_fx, Ai, A, tcx_cfg->preemph_fac, LPDmem, st, synth, Q_new, shift );

    return;
}
