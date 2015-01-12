/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"



/*--------------------------------------------------------------
 * Main Functions
 *--------------------------------------------------------------*/

void core_signal_analysis_high_bitrate( const Word16 *new_samples,   /*i: 0Q15*/
                                        const Word16 T_op[3],        /* i  : open-loop pitch values for quantiz. */
                                        const Word16 voicing[3],     /* i  : open-loop pitch gains               */
                                        const Word16 pitch[2],
                                        Word16 Aw[NB_SUBFR16k*(M+1)],/* i  : weighted A(z) unquant. for subframes*/
                                        Word16 lsp_new[],
                                        Word16 lsp_mid[],
                                        Encoder_State_fx *st,
                                        Word16 pTnsSize[],
                                        Word16 pTnsBits[],
                                        Word16 param_core[],
                                        Word16 *ltpBits,
                                        Word16 L_frame,
                                        Word16 L_frameTCX,
                                        Word32 **spectrum,
                                        Word16 *spectrum_e,
                                        Word16 *Q_new,
                                        Word16 *shift
                                      )
{
    const Word16 last_overlap = st->tcx_cfg.tcx_last_overlap_mode;
    const Word16 curr_overlap = st->tcx_cfg.tcx_curr_overlap_mode;
    Word16 i, frameno;
    Word16 L_subframe;
    Word16 left_overlap, right_overlap, folding_offset;
    Word32 buf[N_MAX]; /* Buffer for TCX20/TCX10 windowing, power spectrum */
    Word16 A[M+1];
    Word16 mdstWin[N_MAX+L_MDCT_OVLP_MAX]; /* Buffer for MDST windowing */
    Word16 * pMdstWin;
    Word16 lpc_left_overlap_mode, lpc_right_overlap_mode;
    Word16 * tcx20Win = (Word16*)buf;
    Word32 powerSpec[N_MAX];
    Word16 powerSpec_e;
    Word32 interleaveBuf[N_TCX10_MAX];
    Word16 *tcx5Win = (Word16*)interleaveBuf; /* Buffer for TCX5 windowing and interleaving. */
    Word16 r_h[NB_DIV][M+1], r_l[NB_DIV][M+1];
    Word32 r[M+1], epsP[M+1];
    Word16 *lsp[2];
    Word8  tmp8;
    Word16 alw_pitch_lag_12k8[2], alw_pitch_lag_12k8_wc;
    Word16 alw_voicing[2], alw_voicing_wc;
    Word16 nSubframes;
    Word16 overlap_mode[3];
    Word16 transform_type[2];
    Word16 tcx10SizeFB;
    Word16 tcx5SizeFB;
    Word16 tcx10Size;
    Word16 tmp, *tmpP16;
    Word32 *tmpP32;
    Word16 Q_exp;


    (void)Aw;
    (void)shift;

    left_overlap = -1;
    move16();
    right_overlap = -1;
    move16();

    tcx10SizeFB = shl(st->tcx_cfg.tcx5SizeFB, 1);
    tcx5SizeFB = st->tcx_cfg.tcx5SizeFB;
    move16();
    tcx10Size = shl(st->tcx_cfg.tcx5Size, 1);

    /*--------------------------------------------------------------*
    * Input Signal Processing: copy, HP filter, pre-emphasis
    *---------------------------------------------------------------*/

    /* Copy Samples */
    Copy(new_samples, st->new_speech_enc, L_frame );
    Scale_sig( st->new_speech_enc, L_frame, 1 );

    /*--------------------------------------------------------------*
    * TCX-LTP
    *---------------------------------------------------------------*/

    tmp8 = 0;
    move16();
    if(L_sub(st->sr_core, 25600) > 0)
    {
        tmp8 = 1;
        move16();
    }

    tcx_ltp_encode( st->tcxltp,
                    st->tcxonly,
                    st->tcxMode,
                    L_frame,
                    L_SUBFR,
                    st->speech_enc+st->encoderLookahead_enc,
                    st->speech_ltp+st->encoderLookahead_enc,
                    st->speech_enc+st->encoderLookahead_enc,
                    T_op[1],
                    &param_core[1+NOISE_FILL_RANGES],
                    ltpBits,
                    &st->tcxltp_pitch_int,
                    &st->tcxltp_pitch_fr,
                    &st->tcxltp_gain,
                    &st->tcxltp_pitch_int_past,
                    &st->tcxltp_pitch_fr_past,
                    &st->tcxltp_gain_past,
                    &st->tcxltp_norm_corr_past,
                    st->last_core_fx,
                    st->pit_min,
                    st->pit_fr1,
                    st->pit_fr2,
                    st->pit_max,
                    st->pit_res_max,
                    &st->transientDetection,
                    tmp8,
                    NULL,
                    M
                  );


    Copy(st->speech_enc+st->encoderLookahead_enc, st->new_speech_enc_pe, L_frame);

    Preemph_scaled(st->new_speech_enc_pe,      /* input: Q0, output: Q_new - 1 */
                   Q_new,
                   &(st->mem_preemph_enc),
                   st->Q_max_enc,
                   st->preemph_fac,
                   1,                           /* preemph_bits = output scaling */
                   0,
                   2,
                   L_frame,
                   st->coder_type_raw_fx,1
                  );

    Q_exp = sub(*Q_new, st->prev_Q_new);
    move16();

    /* Rescale Memory */
    Scale_sig(st->old_inp_16k_fx, L_INP_MEM, sub(*Q_new,st->Q_old));
    IF (Q_exp != 0)
    {
        Scale_sig(st->buf_speech_enc_pe, st->encoderPastSamples_enc+st->encoderLookahead_enc, Q_exp);
        Scale_sig(&(st->mem_wsp_enc), 1, Q_exp);
    }

    IF (sub(st->tcxMode,TCX_10) == 0)
    {
        Copy( &param_core[1+NOISE_FILL_RANGES], &param_core[NPRM_DIV+1+NOISE_FILL_RANGES], LTPSIZE );
    }

    lsp[0] = lsp_new;
    lsp[1] = lsp_mid;

    /*-------------------------------------------------------------------------*
    * Decision matrix for the transform and overlap length
    *--------------------------------------------------------------------------*/

    alw_pitch_lag_12k8[0] = pitch[0];
    move16();
    alw_pitch_lag_12k8[1] = pitch[1];
    move16();
    alw_voicing[0] = voicing[0];
    move16();
    alw_voicing[1] = voicing[1];
    move16();
    alw_pitch_lag_12k8_wc = s_min(alw_pitch_lag_12k8[0], alw_pitch_lag_12k8[1]);
    alw_voicing_wc = s_max(alw_voicing[0], alw_voicing[1]);
    overlap_mode[0] = last_overlap; /* Overlap between the last and the current frame */                          move16();

    IF (sub(st->tcxMode, TCX_20) == 0)
    {
        nSubframes = 1;
        move16();
        transform_type[0] = TCX_20;
        move16();
        overlap_mode[1] = curr_overlap; /* Overlap between the current and the next frame */                        move16();
        alw_pitch_lag_12k8[0] = alw_pitch_lag_12k8_wc;
        move16();
        alw_voicing[0] = alw_voicing_wc;
        move16();
    }
    ELSE
    {
        nSubframes = 2;
        move16();
        IF (sub(curr_overlap, FULL_OVERLAP) == 0)
        {
            transform_type[0] = TCX_5;
            move16();
            transform_type[1] = TCX_10;
            move16();

            overlap_mode[1] = MIN_OVERLAP; /* Overlap between 2nd and 3rd sub-frame */                                move16();
            if (sub(last_overlap, HALF_OVERLAP) == 0)
            {
                overlap_mode[1] = HALF_OVERLAP;
                move16();
            }
        }
        ELSE IF (sub(last_overlap, FULL_OVERLAP) == 0)
        {
            transform_type[0] = TCX_10;
            move16();
            transform_type[1] = TCX_5;
            move16();

            overlap_mode[1] = MIN_OVERLAP; /* Overlap between 1st and 2nd sub-frame */                                move16();
            if (sub(curr_overlap, HALF_OVERLAP) == 0)
            {
                overlap_mode[1] = HALF_OVERLAP;
                move16();
            }
        }
        ELSE
        {
            transform_type[0] = transform_type[1] = TCX_5;
            move16();
            move16();

            overlap_mode[1] = MIN_OVERLAP; /* Overlap between 2nd and 3rd sub-frame */                                move16();
            test();
            if (sub(last_overlap, HALF_OVERLAP) == 0 && sub(curr_overlap, HALF_OVERLAP) == 0)
            {
                overlap_mode[1] = HALF_OVERLAP;
                move16();
            }
        }
        overlap_mode[2] = curr_overlap; /* Overlap between the current and the next frame */                        move16();
    }
    IF (sub(transform_type[0], TCX_20) != 0)
    {
        IGFEncResetTCX10BitCounter(&st->hIGFEnc);
    }
    /*-------------------------------------------------------------------------*
    * Get MDCT output and TNS parameters. Apply TNS in the spectrum if needed
    *--------------------------------------------------------------------------*/

    FOR (frameno = 0; frameno < nSubframes; frameno++)
    {
        L_subframe = L_frameTCX;
        move16();
        if (sub(nSubframes, 1) != 0) L_subframe = shr(L_frameTCX, 1);

        test();
        IF ((sub(transform_type[frameno], TCX_20) != 0) || (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) == 0))
        {
            /* Windowing of the 2xTCX5 subframes or 1xTCX10 or 1xTCX20 */
            WindowSignal(&st->tcx_cfg,
                         st->tcx_cfg.tcx_offsetFB,
                         overlap_mode[frameno],
                         overlap_mode[frameno+1],
                         &left_overlap, &right_overlap,
                         &st->speech_TCX[frameno*tcx10SizeFB],
                         &L_subframe,
                         tcx20Win
                         ,1
                        );
        }

        IF (sub(transform_type[frameno], TCX_5) == 0)
        {
            folding_offset = shr(left_overlap, 1);

            /* Outter left folding */
            FOR (i = 0; i < folding_offset; i++)
            {
                tcx20Win[folding_offset+i] = sub(tcx20Win[folding_offset+i], tcx20Win[folding_offset-1-i]);
                move16();
            }
            /* Outter right folding */
            tmp = shr(right_overlap, 1);
            FOR (i = 0; i < tmp; i++)
            {
                tcx20Win[L_subframe+folding_offset-1-i] = add(tcx20Win[L_subframe+folding_offset-1-i], tcx20Win[L_subframe+folding_offset+i]);
                move16();
            }
            /* 2xTCX5 */
            L_subframe = tcx5SizeFB;
            move16();

            tmpP16 = tcx20Win;
            tmpP32 = spectrum[frameno];
            FOR (i = 0; i < 2; i++)
            {
                test();
                test();
                WindowSignal(&st->tcx_cfg,
                             folding_offset,
                             i == 0 ? RECTANGULAR_OVERLAP : MIN_OVERLAP,
                             sub(i, 1) == 0 ? RECTANGULAR_OVERLAP : MIN_OVERLAP,
                             &left_overlap, &right_overlap,
                             tmpP16,
                             &L_subframe,
                             tcx5Win
                             ,1
                            );

                spectrum_e[frameno] = 16;
                move16();
                TCX_MDCT(tcx5Win,
                         tmpP32,
                         &spectrum_e[frameno],
                         left_overlap,
                         sub(L_subframe, shr(add(left_overlap, right_overlap), 1)),
                         right_overlap);

                tmpP16 += tcx5SizeFB;
                tmpP32 += tcx5SizeFB;
            }
        }
        ELSE /* transform_type[frameno] != TCX_5 */
        {
            assert(transform_type[frameno] == TCX_10 || transform_type[frameno] == TCX_20);

            /* TCX20/TCX10 */
            spectrum_e[frameno] = 16;
            move16();
            test();
            IF ((sub(transform_type[frameno], TCX_20) == 0) && (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) != 0))
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
                overlap_mode[frameno],
                overlap_mode[frameno+1],
                L_frameTCX);

                WindowSignal(&st->tcx_cfg,
                st->tcx_cfg.tcx_offsetFB,
                overlap_mode[frameno] == ALDO_WINDOW ? FULL_OVERLAP : overlap_mode[frameno],
                overlap_mode[frameno+1] == ALDO_WINDOW ? FULL_OVERLAP : overlap_mode[frameno+1],
                &left_overlap, &right_overlap,
                &st->speech_TCX[frameno*tcx10SizeFB],
                &L_subframe,
                mdstWin
                ,1
                            );

                /* scale by NORM_MDCT_FACTOR / L */
                tmp1 = mult_r(shl(L_subframe, 4), FL2WORD16(128.f / NORM_MDCT_FACTOR)); /* 4Q11 */
                tmp2 = 4;
                move16();
                tmp1 = ISqrt16(tmp1, &tmp2);

                FOR (i = 0; i < L_subframe; i++)
                {
                    tmp_buf[i] = Mpy_32_16_1(tmp_buf[i], tmp1);
                    move32();
                }
                Q = sub(Q, tmp2);

                /* DCT */
                edct_fx(tmp_buf, spectrum[frameno], L_subframe, &Q);
                *spectrum_e = sub(31, Q);
            }
            ELSE
            {
                TCX_MDCT(tcx20Win,
                spectrum[frameno],
                &spectrum_e[frameno],
                left_overlap,
                sub(L_subframe, shr(add(left_overlap, right_overlap), 1)),
                right_overlap);
            }

            /* For TCX20 at bitrates up to 64 kbps we need the power spectrum */
            test();
            test();
            IF (sub(st->tcxMode, TCX_20) == 0 && ((L_sub(st->total_brate_fx, HQ_96k) < 0) || st->igf))
            {

                pMdstWin = tcx20Win;
                test();
                if (((sub(st->tcxMode, TCX_20) == 0) && (sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) != 0)))
                {
                    pMdstWin = mdstWin;
                }

                /* Compute noise-measure flags for spectrum filling and quantization */
                AnalyzePowerSpectrum(st,
                                     div_l(L_mult(L_subframe, st->L_frame_fx), st->L_frameTCX),
                                     L_subframe,
                                     left_overlap, right_overlap,
                                     spectrum[frameno], spectrum_e[frameno],
                                     pMdstWin,
                                     powerSpec, &powerSpec_e);
            }
        }

        test();
        test();
        TNSAnalysis(&st->tcx_cfg, L_frameTCX,
                    st->tcx_cfg.tcx_coded_lines,
                    transform_type[frameno], (frameno == 0) && (st->last_core_fx == ACELP_CORE),
                    spectrum[frameno], &st->tnsData[frameno], &st->fUseTns[frameno], NULL
                   );

        EncodeTnsData(st->tcx_cfg.pCurrentTnsConfig, &st->tnsData[frameno],
                      param_core+frameno*NPRM_DIV+1+NOISE_FILL_RANGES+LTPSIZE, pTnsSize+frameno, pTnsBits+frameno);

        IF (sub(transform_type[frameno], TCX_5) == 0)
        {
            /* group sub-windows: interleave bins according to their frequencies */
            FOR (i = 0; i < tcx5SizeFB; i++)
            {
                interleaveBuf[2*i] = spectrum[frameno][i];
                move32();
                interleaveBuf[2*i+1] = spectrum[frameno][tcx5SizeFB+i];
                move32();
            }
            Copy32(interleaveBuf, spectrum[frameno], tcx10SizeFB);
        }

        /*--------------------------------------------------------------*
        * LPC analysis
        *---------------------------------------------------------------*/


        lpc_left_overlap_mode = overlap_mode[frameno];
        move16();
        lpc_right_overlap_mode = overlap_mode[frameno+1];
        move16();
        if (sub(lpc_left_overlap_mode, ALDO_WINDOW) == 0)
        {
            lpc_left_overlap_mode = FULL_OVERLAP;
            move16();
        }
        if (sub(lpc_right_overlap_mode, ALDO_WINDOW) == 0)
        {
            lpc_right_overlap_mode = FULL_OVERLAP;
            move16();
        }

        HBAutocorrelation(&st->tcx_cfg,
                          lpc_left_overlap_mode,
                          lpc_right_overlap_mode,
                          &st->speech_enc_pe[frameno*tcx10Size],
                          shr(L_frame, sub(nSubframes, 1)),
                          r,
                          M);

        FOR (i=0; i <= M; i++)
        {
            move16();
            move16();
            r_l[frameno][i] = L_Extract_lc(r[i], &r_h[frameno][i]);
        }

        adapt_lag_wind( r_h[frameno], r_l[frameno], M, alw_pitch_lag_12k8[frameno], alw_voicing[frameno], st->sr_core );

        E_LPC_lev_dur(r_h[frameno], r_l[frameno], A, epsP, M, NULL);

        E_LPC_a_lsp_conversion(A, lsp[nSubframes-1-frameno], st->lspold_enc_fx, M );

        IF (st->igf)
        {
            ProcessIGF(&st->hIGFEnc, st, spectrum[frameno], &(spectrum_e[frameno]), powerSpec, &powerSpec_e, transform_type[frameno] == TCX_20, st->fUseTns[frameno], (st->last_core_fx == ACELP_CORE), frameno);
        }
    }

    /* Copy memory */
    mvr2r_Word16(lsp_new, st->lspold_enc_fx, M);


}

