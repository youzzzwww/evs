/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "basop_util.h"     /* Function prototypes                    */
#include "stl.h"            /* required for wmc_tool */

#include <assert.h>         /* Debug prototypes                       */

/*--------------------------------------------------------------------------*
 * evs_dec()
 *
 * Principal decoder routine
 *--------------------------------------------------------------------------*/

void evs_dec_fx(
    Decoder_State_fx *st_fx,             /* i/o  : Decoder state structure  */
    Word16 output_sp[],                  /* o    : output synthesis signal  */
    frameMode_fx frameMode               /* i    : Decoder frame mode       */
)
{
    Word16 i, j, output_frame, coder_type;
    Word16 sharpFlag;
    Word16 tmps, incr;
    Word16 core_switching_flag;
    Word16 unbits;
    Word16 hq_core_type;
    Word16 post_hq_delay;
    Word32 bwe_exc_extended_fx[L_FRAME32k+NL_BUFF_OFFSET];
    Word16 voice_factors_fx[NB_SUBFR16k];
    Word16 hb_synth_fx[L_FRAME48k];
    Word16 hb_synth_fx_exp;
    Word32 L_tmp;
    Word16 exp,fra;
    Word16 tmp_buffer_fx[L_FRAME48k];
    Word16 tmp16,tmp16_2;
    Word16 synth_fx[L_FRAME48k + HQ_DELTA_MAX*HQ_DELAY_COMP];
    Word16 fb_exc_fx[L_FRAME16k];
    Word16 pitch_buf_fx[NB_SUBFR16k];
    Word16 Q_fb_exc;
    Word16 old_syn_12k8_16k_fx[L_FRAME16k];
    Word16 sid_bw=-1;
    Word16 pcmbufFB[L_FRAME_MAX];
    Word32 workBuffer[128*3];
    Word16 delta;
    Word16 nab;
    Word16 concealWholeFrame;
    Word16 concealWholeFrameTmp = -1;
    Word16 delay_comp, delay_tdbwe;

    Word16 Qpostd;
    Word16 Q_synth;
    Word16 Qpostd_prev;

    Word32 *realBuffer[CLDFB_NO_COL_MAX], *imagBuffer[CLDFB_NO_COL_MAX];
    Word32 realBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], imagBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];

    delay_tdbwe = 0;                    /* for compiler warning*/
    Qpostd = 0;
    move16();   /* default and used for MODE2 */
    concealWholeFrame = -1;
    move16();

    /*------------------------------------------------------------------*
     * Initialization
     *-----------------------------------------------------------------*/
    FOR( i=0; i<CLDFB_NO_COL_MAX; i++ )
    {
        set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        realBuffer[i] = realBufferTmp[i];
        imagBuffer[i] = imagBufferTmp[i];
    }

    output_frame = st_fx->output_frame_fx;
    move16();

    core_switching_flag = 0;
    move16();
    sharpFlag = 0;
    move16();
    unbits = 0;
    move16();

    st_fx->use_partial_copy = 0;
    move16();
    st_fx->rf_flag = 0;
    move16();

    Qpostd_prev = st_fx->Qprev_synth_buffer_fx;
    move16();
    test();
    IF ( sub(st_fx->bfi_fx,1) == 0 || sub(st_fx->bfi_fx,2) == 0)
    {
        hq_core_type = st_fx->last_hq_core_type_fx;
        move16();
        coder_type = st_fx->last_coder_type_fx;
        move16();
    }
    ELSE
    {
        hq_core_type = -1;
        move16();
        coder_type = INACTIVE;
        move16();
    }

    IF( sub(st_fx->codec_mode, MODE2) == 0 )
    {
        set16_fx(voice_factors_fx,0,NB_SUBFR16k);
    }

    /* PLC: [TCX: Fade-out-recovery]
     * PLC: overlapping part needs to be attenuated for first good frame */      test();
    test();
    test();
    test();
    IF (!st_fx->bfi_fx
        && st_fx->prev_bfi_fx
        && (sub(st_fx->last_codec_mode, MODE2) == 0)
        && (sub(st_fx->last_core_bfi, TCX_20_CORE) == 0
            || sub(st_fx->last_core_bfi, TCX_10_CORE) == 0))
    {
        /* always attenuate old_out_fx */
        Word16  n, stepFB, g, n_e, stepFB_e, g_e, tmp16;

        n = NS2SA_fx2(st_fx->output_Fs_fx, N_ZERO_MDCT_NS);
        stepFB = st_fx->plcInfo.step_concealgain_fx;
        move16();
        stepFB_e = norm_s(stepFB);
        stepFB = shl(stepFB, stepFB_e); /*Q15,-stepFB_e*/
        n_e = norm_s(n);
        n = shl(n, n_e); /*Q0,-n_e*/
        tmp16 = mult_r(stepFB,n); /*Q0,-stepFB_e-n_e*/
        g_e = BASOP_Util_Add_MantExp(st_fx->plcInfo.recovery_gain, -15, tmp16, sub(negate(stepFB_e), n_e), &g); /*Q0,g_e*/
        n_e = norm_s(g);
        IF (sub(add(15,g_e), n_e) <= 0)
        {
            g = shl(g, add(15, g_e)); /*Q15*/
        }
        ELSE
        {
            g = FL2WORD16(1.0f); /*Q15*/
        }

        FOR (i = 0; i < st_fx->L_frameTCX; i++)
        {
            st_fx->old_out_fx[i] = mult_r( st_fx->old_out_fx[i], g );
        }
        /* attenuate PLC buffers, if no aldo window
           is used and if no sid or zero frame is received */
        IF ( 0 == st_fx->tcx_cfg.last_aldo )
        {
            Word32 f;
            Word16 s;
            Word16 tmp1;
            f = L_deposit_l(st_fx->conceal_eof_gain); /*Q15*/
            s = norm_l(f);
            s = sub(16,s);
            tmp1 = extract_l(L_shr_r(f,s));
            FOR( i=0; i < st_fx->tcx_cfg.tcx_mdct_window_lengthFB; i++ )
            {
                st_fx->syn_OverlFB[i] = shl(mult(tmp1,st_fx->syn_OverlFB[i]),s);
                move16();
            }
            s = norm_l(f);
            s = sub(16,s);
            tmp1 = extract_l(L_shr_r(f,s));
            FOR( i=0; i < st_fx->tcx_cfg.tcx_mdct_window_length; i++ )
            {
                st_fx->old_syn_Overl[i]   = shl(mult(tmp1,st_fx->old_syn_Overl[i]),s);
                move16();
            }
        }
    }

    set16_fx( hb_synth_fx, 0, output_frame );
    hb_synth_fx_exp = 0;
    move16();

    st_fx->rate_switching_reset = 0;
    move16();

    /*----------------------------------------------------------------*
     * Updates in case of AMR-WB IO mode -> EVS primary switching
     *----------------------------------------------------------------*/

    IF( sub(st_fx->last_core_fx,AMR_WB_CORE) == 0 )
    {
        updt_IO_switch_dec_fx( output_frame, st_fx );
    }

    IF( sub(frameMode,FRAMEMODE_MISSING) != 0 )  /* frame mode normal or future frame */
    {
        getPartialCopyInfo(st_fx, &coder_type, &sharpFlag);

        frameMode = st_fx->bfi_fx;
    }

    test();
    IF( sub(st_fx->rf_frame_type,RF_NO_DATA) == 0 && sub(st_fx->use_partial_copy,1)==0 )
    {
        /* the partial copy is a RF FRAME_NO_DATA frame and should follow the concealment path*/
        st_fx->bfi_fx = 1;
        move16();
        st_fx->codec_mode = st_fx->last_codec_mode;
        move16();
        frameMode = FRAMEMODE_MISSING;
        move16();
        st_fx->use_partial_copy = 0;
        move16();
    }

    /* if previous frame was concealed via ACELP, drop TCX partial copy info and continue ACELP concealment */
    test();
    test();
    test();
    IF( sub(st_fx->use_partial_copy,1)==0 && sub(st_fx->core_fx,TCX_20_CORE)==0 &&
        sub(st_fx->prev_bfi_fx,1)==0 && sub(st_fx->last_core_fx,ACELP_CORE)==0 )
    {
        st_fx->bfi_fx = 1;
        move16();
        st_fx->codec_mode = st_fx->last_codec_mode;
        move16();
        frameMode = FRAMEMODE_MISSING;
        move16();
        st_fx->use_partial_copy = 0;
        move16();
        st_fx->core_fx = ACELP_CORE;
        move16();
    }


    IF( sub(st_fx->codec_mode,MODE1) == 0 )
    {
        test();
        IF ( sub(st_fx->bfi_fx,1) == 0 || sub(st_fx->bfi_fx,2) == 0 )
        {
            st_fx->nbLostCmpt++;
            move16();
        }
        ELSE
        {
            st_fx->nbLostCmpt = 0;
            move16();
        }
        st_fx->enablePlcWaveadjust = 0;
        move16();

        /*------------------------------------------------------------------*
          * Decision matrix (selection of technologies)
          *-----------------------------------------------------------------*/

        IF ( sub(st_fx->bfi_fx,1) != 0 )
        {
            decision_matrix_dec_fx( st_fx, &coder_type, &sharpFlag, &hq_core_type, &core_switching_flag );
            st_fx->sr_core     = i_mult(st_fx->L_frame_fx,50);
            st_fx->fscale_old  = st_fx->fscale;
            st_fx->fscale      = sr2fscale(st_fx->sr_core);
        }

        /*---------------------------------------------------------------------*
         * Detect bandwidth switching
         *---------------------------------------------------------------------*/

        bandwidth_switching_detect_fx(st_fx);

        /*---------------------------------------------------------------------*
         * Preprocessing (preparing) for ACELP/HQ core switching
         *---------------------------------------------------------------------*/

        core_switching_pre_dec_fx( st_fx, output_frame );

        /*---------------------------------------------------------------------*
         * ACELP core decoding
         * HQ core decoding
         *---------------------------------------------------------------------*/
        IF ( sub(st_fx->core_fx,ACELP_CORE) == 0 )
        {
            /* ACELP core decoder */
            acelp_core_dec_fx( st_fx, synth_fx, bwe_exc_extended_fx, voice_factors_fx, old_syn_12k8_16k_fx, coder_type, sharpFlag, pitch_buf_fx, &unbits, &sid_bw );
            Qpostd = st_fx->Q_syn2;
            move16();
        }
        ELSE
        {
            hq_core_dec_fx( st_fx, synth_fx, &Q_synth, output_frame, hq_core_type, core_switching_flag );
            Qpostd = Q_synth;
            move16();
        }

        /*---------------------------------------------------------------------*
         * Postprocessing for ACELP/HQ core switching
         *---------------------------------------------------------------------*/

        core_switching_post_dec_fx( st_fx, synth_fx, output_frame, core_switching_flag, coder_type, &Qpostd );

        /*---------------------------------------------------------------------*
         * Pre-processing for bandwidth switching
         *---------------------------------------------------------------------*/

        bw_switching_pre_proc_fx( old_syn_12k8_16k_fx, st_fx );

        /*---------------------------------------------------------------------*
         * WB TBE decoding
         * WB BWE decoding
         *---------------------------------------------------------------------*/

        IF ( sub(st_fx->extl_fx,WB_TBE) == 0 )
        {
            /* WB TBE decoder */
            wb_tbe_dec_fx( st_fx, coder_type, bwe_exc_extended_fx, st_fx->Q_exc, voice_factors_fx, hb_synth_fx, &hb_synth_fx_exp );
        }
        ELSE IF ( sub(st_fx->extl_fx,WB_BWE) == 0 && st_fx->bws_cnt_fx == 0)
        {
            /* WB BWE decoder */
            hb_synth_fx_exp = wb_bwe_dec_fx( synth_fx, hb_synth_fx, output_frame, coder_type, voice_factors_fx, pitch_buf_fx, st_fx, &Qpostd );
        }

        /*---------------------------------------------------------------------*
         * SWB TBE decoding
         * SWB BWE decoding
         * FB TBE decoding
         *---------------------------------------------------------------------*/
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF ( sub(st_fx->extl_fx,SWB_TBE) == 0 || sub(st_fx->extl_fx,FB_TBE) == 0
             || (sub(coder_type,AUDIO) != 0 && sub(coder_type,INACTIVE) != 0 && L_sub(st_fx->core_brate_fx,SID_2k40) > 0 && sub(st_fx->core_fx,ACELP_CORE) == 0
                 && L_sub(st_fx->output_Fs_fx,32000) >= 0 && sub(st_fx->bwidth_fx,NB) > 0 && st_fx->bws_cnt_fx > 0 && !st_fx->ppp_mode_dec_fx) )
        {
            swb_tbe_dec_fx( st_fx, coder_type, bwe_exc_extended_fx, st_fx->Q_exc, voice_factors_fx,
                            old_syn_12k8_16k_fx, fb_exc_fx, &Q_fb_exc, hb_synth_fx, &hb_synth_fx_exp, pitch_buf_fx );

            /* FB TBE decoder/synthesis */
            test();
            IF ( sub(output_frame,L_FRAME48k) == 0 && sub(st_fx->extl_fx,FB_TBE) == 0 )
            {
                fb_tbe_dec_fx( st_fx, fb_exc_fx, Q_fb_exc, hb_synth_fx, hb_synth_fx_exp);
            }
        }
        ELSE IF( sub(st_fx->extl_fx,SWB_BWE) == 0 || sub(st_fx->extl_fx,FB_BWE) == 0 ||
                 (L_sub(st_fx->output_Fs_fx,32000) >= 0 && sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->bwidth_fx,NB) > 0 && st_fx->bws_cnt_fx > 0 && !st_fx->ppp_mode_dec_fx ) )
        {
            /* SWB BWE decoder */
            hb_synth_fx_exp = swb_bwe_dec_fx( st_fx, synth_fx, hb_synth_fx, output_frame, &Qpostd );
        }
        ELSE IF( sub(st_fx->extl_fx,SWB_BWE_HIGHRATE) == 0 || sub(st_fx->extl_fx,FB_BWE_HIGHRATE) == 0 )
        {
            hb_synth_fx_exp = swb_bwe_dec_hr_fx( st_fx, old_syn_12k8_16k_fx, Qpostd, hb_synth_fx, output_frame, unbits, pitch_buf_fx );
        }

        /*---------------------------------------------------------------------*
         * FEC - recovery after lost HQ core (smoothing of the BWE component)
         *---------------------------------------------------------------------*/

        test();
        test();
        IF ( st_fx->prev_bfi_fx && sub(st_fx->last_core_fx,HQ_CORE) == 0 && sub(st_fx->extl_fx,-1) != 0 )
        {
            /*tmp = FRAC_BWE_SMOOTH/output_frame;*/
            tmp16 = shr(410,shr(output_frame,8));
            if(sub(output_frame, L_FRAME48k)==0)
            {
                tmp16 = 68;
                move16();
            }
            /*output_frame/FRAC_BWE_SMOOTH*/
            j = shr(output_frame,1);
            tmp16_2 = 0;
            move16();
            FOR (i = 0; i < j; i++)
            {
                /*hb_synth[i] *= (i*tmp);*/
                hb_synth_fx[i] = mult_r(hb_synth_fx[i], tmp16_2);
                move16();
                tmp16_2 = add(tmp16_2, tmp16);
            }
        }

        /*---------------------------------------------------------------------*
         * SWB CNG
         *---------------------------------------------------------------------*/
        IF( sub(output_frame,L_FRAME32k) >= 0 )
        {
            /* SHB CNG decoder */
            swb_CNG_dec_fx( st_fx, synth_fx, hb_synth_fx, sid_bw, Qpostd );
            if( L_sub(st_fx->core_brate_fx, SID_2k40) <= 0 )
            {
                hb_synth_fx_exp = 0;
                move16();
            }
        }

        /*----------------------------------------------------------------*
         * Delay ACELP core synthesis to be synchronized with the components of bandwidth extension layers
         *----------------------------------------------------------------*/

        IF ( sub(output_frame,L_FRAME16k) >= 0 )
        {
            tmps = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS);

            exp=s_min(Qpostd, st_fx->Qprev_synth_buffer_fx);
            Scale_sig(synth_fx, output_frame, sub(exp,Qpostd));
            Qpostd=exp;
            move16();
            Scale_sig(st_fx->prev_synth_buffer_fx, tmps, sub(exp, st_fx->Qprev_synth_buffer_fx));
            st_fx->Qprev_synth_buffer_fx=exp;
            move16();
            Copy(synth_fx, tmp_buffer_fx, output_frame);
            Copy(st_fx->prev_synth_buffer_fx, synth_fx, tmps);
            Copy(tmp_buffer_fx, synth_fx + tmps, output_frame - tmps);
            Copy(tmp_buffer_fx + output_frame - tmps, st_fx->prev_synth_buffer_fx, tmps);
        }
        ELSE
        {
            exp=s_min(Qpostd, st_fx->Qprev_synth_buffer_fx);
            Scale_sig(synth_fx, output_frame, sub(exp,Qpostd));
            Qpostd=exp;
            move16();
            st_fx->Qprev_synth_buffer_fx=exp;
            move16();
        }

        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF (sub(st_fx->core_fx,ACELP_CORE) == 0
            && !st_fx->bfi_fx
            && st_fx->prev_bfi_fx
            && (sub(st_fx->last_codec_mode, MODE2) == 0)
            && (sub(st_fx->last_core_bfi, TCX_20_CORE) == 0 || sub(st_fx->last_core_bfi, TCX_10_CORE) == 0)
            && (st_fx->plcInfo.concealment_method == TCX_NONTONAL)
            && (L_sub(st_fx->plcInfo.nbLostCmpt, 4) < 0) )
        {
            waveform_adj2_fix(st_fx->tonalMDCTconceal.secondLastPcmOut,
                              synth_fx,
                              st_fx->plcInfo.data_noise,
                              &st_fx->plcInfo.outx_new_n1_fx,
                              &st_fx->plcInfo.nsapp_gain_fx,
                              &st_fx->plcInfo.nsapp_gain_n_fx,
                              &st_fx->plcInfo.recovery_gain,
                              st_fx->plcInfo.step_concealgain_fx,
                              st_fx->plcInfo.Pitch_fx,
                              st_fx->plcInfo.FrameSize,
                              1,
                              0,
                              add(extract_l(st_fx->plcInfo.nbLostCmpt), 1),
                              st_fx->bfi_fx);
        }

        /*----------------------------------------------------------------*
         * Addition of BWE components to the ACELP core synthesis
         *----------------------------------------------------------------*/

        test();
        test();
        IF ( sub(st_fx->extl_fx,-1) != 0 || (st_fx->bws_cnt_fx > 0 && sub(st_fx->core_fx,ACELP_CORE) == 0) )
        {
            /* Calculate an additional delay of extension layer components to be synchronized with ACELP synthesis */
            IF ( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
            {
                /* TBE on top of ACELP@12.8kHz */
                tmps = NS2SA_fx2( st_fx->output_Fs_fx, MAX_DELAY_TBE_NS - DELAY_SWB_TBE_12k8_NS );
            }
            ELSE
            {
                test();
                IF( sub(st_fx->extl_fx,SWB_BWE_HIGHRATE) == 0 || sub(st_fx->extl_fx,FB_BWE_HIGHRATE) == 0 )
                {
                    /* HR SWB BWE on top of ACELP@16kHz */
                    tmps = NS2SA_fx2( st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS );
                }
                ELSE
                {
                    /* TBE on top of ACELP@16kHz */
                    tmps = NS2SA_fx2( st_fx->output_Fs_fx, MAX_DELAY_TBE_NS - DELAY_SWB_TBE_16k_NS );
                }
            }

            /* Smooth transitions when switching between different technologies */
            test();
            test();
            test();
            test();
            IF ( (sub(st_fx->extl_fx,st_fx->last_extl_fx) != 0 || (sub(st_fx->extl_fx,st_fx->last_extl_fx) == 0 && sub((st_fx->core_fx ^ st_fx->last_core_fx),HQ_CORE) == 0))
                 && !(sub(st_fx->extl_fx,SWB_CNG) == 0 && sub(st_fx->last_extl_fx,SWB_TBE) == 0) )
            {
                /*incr = (short) ( L_FRAME / (tmps + 0.5f) );*/
                incr = idiv1616(L_FRAME*2, add(shl(tmps,1),1));
                tmp16 = 0;
                move16();
                FOR (i=0; i<tmps; i++)
                {
                    hb_synth_fx[i] = mult_r(hb_synth_fx[i], sin_table256_fx[tmp16]);
                    move16();
                    tmp16 = add(tmp16, incr);
                }
                set16_fx( st_fx->hb_prev_synth_buffer_fx, 0, tmps );
            }
            ELSE IF ( sub(tmps,st_fx->old_bwe_delay_fx) < 0 )
            {
                /* the previous frame was TBE on top of ACELP@16kHz and the current frame is TBE on top of ACELP@12.8kHz */
                /*incr = (short) ( L_FRAME / (tmps + 0.5f) );*/
                incr = idiv1616(L_FRAME*2, add(shl(tmps,1),1));
                tmp16 = 0;
                move16();
                FOR (i=0; i<tmps; i++)
                {
                    tmp_buffer_fx[i] = round_fx(L_mac(L_mult(st_fx->hb_prev_synth_buffer_fx[i], sin_table256_fx[255 - tmp16]),
                                                      st_fx->hb_prev_synth_buffer_fx[st_fx->old_bwe_delay_fx - 1 - i], sin_table256_fx[tmp16]));
                    tmp16 = add(tmp16, incr);
                }
                Copy(tmp_buffer_fx, st_fx->hb_prev_synth_buffer_fx, tmps);
            }
            ELSE IF (sub(tmps,st_fx->old_bwe_delay_fx) > 0)
            {
                /* the previous frame was TBE on top of ACELP@12.8kHz and the current frame is TBE on top of ACELP@16kHz */
                /*incr = (short)( L_FRAME  / (st->old_bwe_delay + 0.5f) );*/
                incr = idiv1616(L_FRAME*2, add(shl(st_fx->old_bwe_delay_fx,1),1));
                tmp16 = 0;
                move16();
                FOR (i=0; i<st_fx->old_bwe_delay_fx; i++)
                {
                    tmp_buffer_fx[i] = mult_r(st_fx->hb_prev_synth_buffer_fx[i], sin_table256_fx[255 - tmp16]);
                    move16();
                    tmp16 = add(tmp16, incr);

                }
                FOR (; i<tmps; i++)
                {
                    tmp_buffer_fx[i] = 0;
                    move16();
                }
                tmp16 = 0;
                FOR (i=0; i<st_fx->old_bwe_delay_fx; i++)
                {
                    /*tmp_buffer[tmps - 1 - i] += st->hb_prev_synth_buffer[st->old_bwe_delay - 1 - i] * sin_table256[i * incr];*/
                    tmp_buffer_fx[tmps - 1 - i] = round_fx(L_mac(L_mult(tmp_buffer_fx[tmps - 1 - i], 32767), st_fx->hb_prev_synth_buffer_fx[st_fx->old_bwe_delay_fx - 1 - i], sin_table256_fx[tmp16/*i * incr*/]));
                    tmp16 = add(tmp16, incr);
                }

                Copy( tmp_buffer_fx, st_fx->hb_prev_synth_buffer_fx, tmps );
            }
            /* Delay hb_synth */
            tmp16 = sub(hb_synth_fx_exp, st_fx->prev_hb_synth_fx_exp);
            IF ( tmp16 != 0 )
            {
                Scale_sig(st_fx->hb_prev_synth_buffer_fx, tmps, tmp16 );
            }
            Copy( hb_synth_fx, tmp_buffer_fx, output_frame );
            Copy( st_fx->hb_prev_synth_buffer_fx, hb_synth_fx, tmps );
            Copy( tmp_buffer_fx, hb_synth_fx + tmps, output_frame - tmps );
            Copy( tmp_buffer_fx + output_frame - tmps, st_fx->hb_prev_synth_buffer_fx, tmps );

            st_fx->old_bwe_delay_fx = tmps;
            move16();
            IF( st_fx->ppp_mode_dec_fx && sub(st_fx->L_frame_fx, st_fx->last_L_frame_fx) == 0 && (st_fx->bws_cnt_fx > 1 || st_fx->last_extl_fx != -1) )
            {
                Copy( st_fx->old_hb_synth_fx, hb_synth_fx, output_frame );
                IF(sub(st_fx->prev_hb_synth_fx_exp, 14) < 0)
                {
                    hb_synth_fx_exp = add(st_fx->prev_hb_synth_fx_exp, 1);
                }
                ELSE
                {
                    hb_synth_fx_exp = 14;
                }
            }
            ELSE
            {
                Copy( hb_synth_fx, st_fx->old_hb_synth_fx, output_frame );
            }

            add_vec_fx( synth_fx, Qpostd, hb_synth_fx, hb_synth_fx_exp, synth_fx, Qpostd, output_frame );

            /* SWB CNG/DTX - calculate SHB energy */
            test();
            IF ( sub(output_frame, L_FRAME32k) >= 0 && sub(st_fx->extl_fx, SWB_CNG) > 0 )
            {
                SWITCH (output_frame)
                {
                case L_FRAME8k:
                    tmp16 = 205;
                    BREAK; /*Q15*/
                case L_FRAME16k:
                    tmp16 = 102;
                    BREAK; /*Q15*/
                case L_FRAME32k:
                    tmp16 = 51;
                    BREAK; /*Q15*/
                case L_FRAME48k:
                    tmp16 = 34;
                    BREAK; /*Q15*/
                }

                L_tmp = L_deposit_l(1); /*Q2*hb_synth_fx_exp*/
                FOR ( i=0; i<output_frame; i++ )
                {
                    L_tmp = L_add(L_tmp, Mpy_32_16_1(L_mult0(hb_synth_fx[i], hb_synth_fx[i]), tmp16)); /*Q2*hb_synth_fx_exp*/
                }
                exp = norm_l(L_tmp);
                fra = Log2_norm_lc(L_shl(L_tmp, exp));
                exp = sub(sub(30, shl(hb_synth_fx_exp,1)), exp);
                L_tmp = Mpy_32_16(exp, fra, LG10);
                st_fx->last_shb_ener_fx = round_fx(L_shl(L_tmp, 10)); /*Q8*/
            }
        }
        st_fx->prev_hb_synth_fx_exp = hb_synth_fx_exp;
        move16();

        /* TCX-LTP Postfilter: used in MODE1 to update memories and to avoid discontinuities when the past frame was TCX */
        delta = NS2SA_fx2( st_fx->output_Fs_fx, TCXLTP_DELAY_NS );
        Scale_sig(st_fx->tcxltp_mem_in, delta, sub(Qpostd, Qpostd_prev));
        Scale_sig(st_fx->tcxltp_mem_out, output_frame, sub(Qpostd, Qpostd_prev));
        tcx_ltp_post( st_fx->tcxltp, ACELP_CORE, output_frame, st_fx->L_frame_past, 0, synth_fx, NULL,
                      delta, 0, 0, 0, 0, &st_fx->tcxltp_pitch_int_post_prev,
                      &st_fx->tcxltp_pitch_fr_post_prev, &st_fx->tcxltp_gain_post_prev,
                      &st_fx->tcxltp_filt_idx_prev, st_fx->pit_res_max,
                      &st_fx->pit_res_max_past,
                      0, 0, st_fx->tcxltp_mem_in,
                      st_fx->tcxltp_mem_out, st_fx->total_brate_fx );


        /* final output of synthesis signal */
        Copy( synth_fx, output_sp, output_frame );


    }
    ELSE /* MODE2 PART */
    {

        /* -------------------------------------------------------------- */
        /* CONCEALMENT                                                    */
        /* -------------------------------------------------------------- */

        concealWholeFrame = 0;
        move16();

        if( sub(frameMode, FRAMEMODE_NORMAL) == 0 )
        {
            st_fx->m_decodeMode = DEC_NO_FRAM_LOSS;
            move16();
        }

        IF( sub(frameMode, FRAMEMODE_MISSING) == 0 )
        {
            test();
            test();
            IF( st_fx->use_partial_copy && sub(st_fx->rf_frame_type, RF_TCXFD) >= 0 && sub(st_fx->rf_frame_type, RF_TCXTD2) <= 0)
            {
                st_fx->m_decodeMode = DEC_NO_FRAM_LOSS;
                move16();
            }
            ELSE
            {
                st_fx->m_decodeMode = DEC_CONCEALMENT_EXT;
                move16();
            }
        }

        SWITCH( st_fx->m_decodeMode )
        {
        case DEC_NO_FRAM_LOSS:
            BREAK;
        case DEC_CONCEALMENT_EXT:
            concealWholeFrame = 1;
            move16();
            BREAK;
        }

        concealWholeFrameTmp = concealWholeFrame;
        move16();

        /* -------------------------------------------------------------- */
        /* DECODE CORE                                                    */
        /* -------------------------------------------------------------- */

        dec_acelp_tcx_frame( st_fx, &coder_type, &concealWholeFrame, output_sp,
                             st_fx->p_bpf_noise_buf, pcmbufFB, bwe_exc_extended_fx, voice_factors_fx, pitch_buf_fx );

        IF( st_fx->igf )
        {

            /* TBE interface */
            test();
            test();
            IF( (st_fx->bfi_fx == 0 || st_fx->last_core_fx == ACELP_CORE) && st_fx->core_fx == ACELP_CORE )
            {
                test();
                test(), test();
                SWITCH (st_fx->bwidth_fx)
                {
                case WB:
                    st_fx->extl_fx = WB_TBE;
                    move16();
                    st_fx->extl_brate_fx = WB_TBE_0k35;
                    move32();
                    BREAK;

                case SWB:
                    st_fx->extl_fx = SWB_TBE;
                    move16();
                    st_fx->extl_brate_fx = SWB_TBE_1k6;
                    move32();
                    BREAK;

                case FB:
                    st_fx->extl_fx = FB_TBE;
                    move16();
                    st_fx->extl_brate_fx = FB_TBE_1k8;
                    move32();
                    BREAK;
                }
            }
            ELSE
            {
                st_fx->extl_fx = IGF_BWE;
                move16();
                st_fx->extl_brate_fx = L_deposit_l(0);
            }
            st_fx->core_brate_fx = L_sub(st_fx->total_brate_fx, st_fx->extl_brate_fx);

            st_fx->bws_cnt_fx = 0;
            move16();
            st_fx->bws_cnt1_fx = 0;
            move16();
            st_fx->tilt_wb_fx = 0;
            move16();

            IF( sub(st_fx->m_frame_type, ACTIVE_FRAME) == 0 )
            {
                test();
                test();
                IF( ( st_fx->bfi_fx == 0 || st_fx->last_core_fx == ACELP_CORE ) && st_fx->core_fx == ACELP_CORE )
                {
                    test();
                    IF( sub(st_fx->extl_fx, WB_TBE) == 0 )
                    {
                        wb_tbe_dec_fx( st_fx, coder_type, bwe_exc_extended_fx, st_fx->Q_exc, voice_factors_fx, hb_synth_fx, &hb_synth_fx_exp );
                        Copy( hb_synth_fx, st_fx->old_hb_synth_fx, L_FRAME32k );
                    }
                    ELSE IF( sub(st_fx->extl_fx, SWB_TBE) == 0 || sub(st_fx->extl_fx, FB_TBE) == 0 )
                    {
                        /* SWB TBE decoder */
                        swb_tbe_dec_fx( st_fx, coder_type, bwe_exc_extended_fx, st_fx->Q_exc, voice_factors_fx, st_fx->old_core_synth_fx,
                                        fb_exc_fx, &Q_fb_exc, hb_synth_fx, &hb_synth_fx_exp, pitch_buf_fx );

                        test();
                        IF( sub(st_fx->extl_fx, FB_TBE) == 0 && sub(output_frame, L_FRAME48k) == 0 )
                        {
                            fb_tbe_dec_fx( st_fx, fb_exc_fx, Q_fb_exc, hb_synth_fx, hb_synth_fx_exp );
                        }
                    }
                }
                ELSE
                {
                    IF( sub(st_fx->last_core_fx,ACELP_CORE) == 0 )
                    {
                        test();
                        test();
                        IF( (sub(st_fx->bwidth_fx, SWB) == 0 || sub(st_fx->bwidth_fx, FB) == 0) && sub(st_fx->last_codec_mode, MODE2) == 0)
                        {

                            GenTransition_fx( st_fx->syn_overlap_fx, st_fx->old_tbe_synth_fx, 2*NS2SA(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS), hb_synth_fx,
                            st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx, &(st_fx->syn_dm_phase_fx),
                            st_fx->output_Fs_fx, st_fx->int_3_over_2_tbemem_dec_fx, st_fx->rf_flag
                            , st_fx->total_brate_fx
                                            );

                            hb_synth_fx_exp = st_fx->prev_Q_bwe_syn2;
                            move16();
                        }

                        TBEreset_dec_fx( st_fx, st_fx->bwidth_fx );
                    }
                    ELSE IF ( sub(st_fx->last_codec_mode,MODE1)==0)
                    {
                        swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                                          st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx), &(st_fx->tbe_premph_fx), st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx) );

                        swb_tbe_reset_synth_fx( st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx );
                    }
                }
            }
        }

        /* -------------------------------------------------------------- */
        /* APPLY POSTPROC                                                 */
        /* -------------------------------------------------------------- */

        {
            Word16 timeIn_e;

            timeIn_e = 0;
            move16();

            nab = min( st_fx->cldfbAna_fx->no_channels, st_fx->cldfbSyn_fx->no_channels );
            st_fx->cldfbSyn_fx->lsb = s_min(st_fx->cldfbAna_fx->no_channels, st_fx->cldfbSyn_fx->no_channels);
            move16();
            st_fx->cldfbSyn_fx->usb = st_fx->cldfbSyn_fx->no_channels;
            move16();
            st_fx->cldfbAna_fx->lsb = st_fx->cldfbAna_fx->no_channels;
            move16();
            st_fx->cldfbAna_fx->usb = st_fx->cldfbAna_fx->no_channels;
            move16();

            test();
            test();
            test();
            test();
            IF ( st_fx->hFdCngDec_fx != NULL && (L_sub(st_fx->sr_core,8000) == 0 || L_sub(st_fx->sr_core,12800) == 0 || L_sub(st_fx->sr_core,16000) == 0) && L_sub(st_fx->total_brate_fx,ACELP_32k) <= 0 )
            {
                /***************************************
                 In CLDFB domain:
                 - perform noise estimation during active frames
                 - do CNG during inactive frames
                ****************************************/
                HANDLE_FD_CNG_DEC hFdCngDec = st_fx->hFdCngDec_fx;
                move16();

                noisy_speech_detection( st_fx->VAD && st_fx->m_frame_type==ACTIVE_FRAME, output_sp, st_fx->L_frame_fx, 0, hFdCngDec->msNoiseEst, hFdCngDec->msNoiseEst_exp,
                hFdCngDec->psize_shaping_norm, hFdCngDec->psize_shaping_norm_exp, hFdCngDec->nFFTpart_shaping,
                &(hFdCngDec->lp_noise), &(hFdCngDec->lp_speech), &(hFdCngDec->hFdCngCom->flag_noisy_speech) );

                hFdCngDec->hFdCngCom->likelihood_noisy_speech = mult_r(hFdCngDec->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.99));
                IF ( hFdCngDec->hFdCngCom->flag_noisy_speech != 0 )
                {
                    hFdCngDec->hFdCngCom->likelihood_noisy_speech = add(hFdCngDec->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.01));
                    move16();
                }
                st_fx->lp_noise = hFdCngDec->lp_noise;
                move32();

                ApplyFdCng( output_sp, 0, realBuffer, imagBuffer, &st_fx->scaleFactor.hb_scale, hFdCngDec, st_fx->m_frame_type, st_fx, concealWholeFrame
                            , 0
                          );

                /* Generate additional comfort noise to mask potential coding artefacts */
                test();
                IF( sub(st_fx->m_frame_type,ACTIVE_FRAME) == 0 && st_fx->flag_cna )
                {
                    generate_masking_noise( output_sp, 0, hFdCngDec->hFdCngCom, hFdCngDec->hFdCngCom->frameSize, 0 );
                }

                hFdCngDec->hFdCngCom->frame_type_previous = st_fx->m_frame_type;
                move16();
            }

            IF( sub(st_fx->m_frame_type,ACTIVE_FRAME) == 0 )
            {
                cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &st_fx->scaleFactor, output_sp, 0, CLDFB_NO_COL_MAX, workBuffer );
                st_fx->scaleFactor.hb_scale = st_fx->scaleFactor.lb_scale;
                move16();
            }
            ELSE
            {
                Word16 timeDomainBuffer[L_FRAME16k];
                Word16 A[M+1];

                Copy( st_fx->hFdCngDec_fx->hFdCngCom->timeDomainBuffer, timeDomainBuffer, st_fx->L_frame_fx );
                Copy( st_fx->hFdCngDec_fx->hFdCngCom->A_cng, A, M+1 );

                update_decoder_LPD_cng( st_fx, coder_type, timeDomainBuffer, A, st_fx->p_bpf_noise_buf );

                /* Generate additional comfort noise to mask potential coding artefacts */
                IF( st_fx->flag_cna )
                {
                    generate_masking_noise( timeDomainBuffer, 0, st_fx->hFdCngDec_fx->hFdCngCom, st_fx->hFdCngDec_fx->hFdCngCom->frameSize, 0 );
                }

                /* check if the CLDFB works on the right sample rate */
                IF( sub((st_fx->cldfbAna_fx->no_channels * st_fx->cldfbAna_fx->no_col),st_fx->L_frame_fx) != 0 )
                {
                    Word16 newCldfbBands = CLDFB_getNumChannels( (int)(st_fx->L_frame_fx * 50) );

                    resampleCldfb( st_fx->cldfbAna_fx, newCldfbBands, st_fx->L_frame_fx, 0 );
                    resampleCldfb( st_fx->cldfbBPF_fx, newCldfbBands, st_fx->L_frame_fx, 0 );
                }

                st_fx->cldfbSyn_fx->bandsToZero = 0;
                move16();
                test();
                IF ( sub( st_fx->bwidth_fx, NB ) == 0 && sub( st_fx->cldfbSyn_fx->no_channels, 10 ) > 0 )
                {
                    st_fx->cldfbSyn_fx->bandsToZero = sub( st_fx->cldfbSyn_fx->no_channels, 10 );
                }
                ELSE IF ( st_fx->hFdCngDec_fx->hFdCngCom->regularStopBand < st_fx->cldfbSyn_fx->no_channels )
                {
                    st_fx->cldfbSyn_fx->bandsToZero = st_fx->cldfbSyn_fx->no_channels - st_fx->hFdCngDec_fx->hFdCngCom->regularStopBand;
                }

                timeIn_e = 2;
                move16();
                Scale_sig(timeDomainBuffer, st_fx->L_frame_fx, timeIn_e);
                IF ( st_fx->p_bpf_noise_buf )
                {
                    Scale_sig(st_fx->p_bpf_noise_buf, st_fx->L_frame_fx, timeIn_e);
                }

                cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &st_fx->scaleFactor, timeDomainBuffer, negate(timeIn_e), CLDFB_NO_COL_MAX, workBuffer) ;
            }

            if( st_fx->flag_cna == 0 )
            {
                set16_fx( st_fx->hFdCngDec_fx->hFdCngCom->olapBufferSynth2, 0, st_fx->hFdCngDec_fx->hFdCngCom->fftlen );
            }

            IF( st_fx->p_bpf_noise_buf )
            {
                addBassPostFilterFx( st_fx->p_bpf_noise_buf, realBuffer, imagBuffer, st_fx->cldfbBPF_fx, workBuffer,
                negate(timeIn_e), CLDFB_NO_COL_MAX, st_fx->cldfbAna_fx->no_col, st_fx->cldfbAna_fx->no_channels, &st_fx->scaleFactor );

                IF( sub(st_fx->m_frame_type,ACTIVE_FRAME) != 0 )
                {
                    Scale_sig(st_fx->p_bpf_noise_buf, st_fx->L_frame_fx, negate(timeIn_e));
                }
            }

            IF (L_sub(st_fx->output_Fs_fx, 8000) > 0)
            {
                st_fx->tecDec_fx.cldfbExp = add(15, st_fx->scaleFactor.lb_scale);


                calcGainTemp_TBE_Fx( realBuffer, imagBuffer, st_fx->tecDec_fx.cldfbExp, st_fx->tecDec_fx.loBuffer,
                                     0,                               /*startPos,*//*!<  Start position of the current envelope. */
                                     st_fx->cldfbAna_fx->no_col,      /*stopPos,*/ /*!<  Stop position of the current envelope. */
                                     st_fx->cldfbAna_fx->no_channels, /*lowSubband*/
                                     st_fx->tecDec_fx.pGainTemp_m, st_fx->tecDec_fx.pGainTemp_e, st_fx->tec_flag
                                   );
            }

            /* set high band buffers to zero. Covering the current frame and the overlap area. */
            IF( sub(st_fx->m_frame_type,ACTIVE_FRAME) == 0 )
            {
                FOR( i = 0; i < 16; i++ )
                {
                    set32_fx( &realBuffer[i][nab], 0, sub(st_fx->cldfbSyn_fx->no_channels,nab) );
                    set32_fx( &imagBuffer[i][nab], 0, sub(st_fx->cldfbSyn_fx->no_channels,nab) );
                }
            }

            cldfbSynthesisFiltering(st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &st_fx->scaleFactor, output_sp, 0, CLDFB_NO_COL_MAX, workBuffer );
            /*CLDFB output always in Q0*/

            /* MODE1 MDCT to ACELP 2 transition */
            delay_comp = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS);
            Scale_sig(st_fx->delay_buf_out_fx, delay_comp, negate(st_fx->Q_old_postdec));
            st_fx->Q_old_postdec = 0;
            move16();
            delay_tdbwe= NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS- DELAY_CLDFB_NS);
            IF( sub(output_frame,L_FRAME16k) >= 0 )
            {
                Scale_sig(st_fx->prev_synth_buffer_fx, delay_tdbwe, sub(Qpostd, st_fx->Qprev_synth_buffer_fx));
            }

            test();
            IF( sub(st_fx->last_codec_mode,MODE1) == 0 && sub(st_fx->last_core_bfi,ACELP_CORE) > 0 )
            {
                Copy( st_fx->delay_buf_out_fx, output_sp, delay_comp);  /* copy the HQ/ACELP delay synchronization buffer at the beginning of ACELP frame */
                IF( sub(st_fx->core_fx,ACELP_CORE) == 0 )
                {
                    Word16 step, alpha, nz;

                    i = 15;
                    move16();
                    tmps = NS2SA_fx2(st_fx->output_Fs_fx, 3000000L);
                    nz = NS2SA_fx2(st_fx->output_Fs_fx, N_ZERO_MDCT_NS);
                    step = Inv16(tmps, &i);
                    step = shl(step, i);
                    alpha = 0;
                    move16();

                    test();
                    IF( st_fx->prev_bfi_fx && st_fx->HqVoicing_fx )
                    {
                        Copy( st_fx->fer_samples_fx, &st_fx->old_out_fx[nz], tmps);
                    }
                    ELSE
                    {
                        Scale_sig(st_fx->old_out_fx, nz+tmps, negate(st_fx->Q_old_wtda));
                    }
                    st_fx->Q_old_wtda = 0;
                    move16();

                    FOR (i = 0; i < tmps; i++)
                    {
                        output_sp[i+delay_comp] = msu_r(L_mult(output_sp[i+delay_comp], alpha), st_fx->old_out_fx[i+nz], add(alpha, -32768));
                        move16();
                        alpha = add(alpha, step);
                    }
                }
                ELSE
                {
                    IF( L_sub(st_fx->output_Fs_fx,8000) == 0 )
                    {
                        Copy(st_fx->delay_buf_out_fx, st_fx->FBTCXdelayBuf, delay_comp);
                    }
                    ELSE
                    {
                        Copy( st_fx->prev_synth_buffer_fx, st_fx->FBTCXdelayBuf, delay_tdbwe );
                        Copy( st_fx->delay_buf_out_fx, st_fx->FBTCXdelayBuf + delay_tdbwe, delay_comp );
                    }
                }
            }

            /* set delay compensation between HQ synthesis and ACELP synthesis */
            test();
            IF( sub(st_fx->core_fx,ACELP_CORE) == 0 && !(st_fx->con_tcx) )
            {
                set16_fx( st_fx->delay_buf_out_fx, 0, delay_comp );
                Copy( output_sp, st_fx->previoussynth_fx, output_frame );
            }
            ELSE
            {
                Copy( st_fx->old_synthFB_fx+st_fx->old_synth_lenFB-delay_comp, st_fx->delay_buf_out_fx, delay_comp );
                IF( L_sub(st_fx->output_Fs_fx, 8000) == 0 )
                {
                    Copy(st_fx->FBTCXdelayBuf, st_fx->previoussynth_fx, delay_comp);
                }
                ELSE
                {
                    Copy( st_fx->FBTCXdelayBuf + delay_tdbwe, st_fx->previoussynth_fx, delay_comp );
                }
                Copy(pcmbufFB, st_fx->previoussynth_fx + delay_comp, sub(output_frame, delay_comp));
            }
        }

        /* Delay compensation for TD-BWE*/
        IF( sub(output_frame,L_FRAME16k) >= 0 )
        {
            Copy( output_sp, tmp_buffer_fx, output_frame );
            Copy( st_fx->prev_synth_buffer_fx, output_sp, delay_tdbwe );
            Copy( tmp_buffer_fx, output_sp + delay_tdbwe, output_frame - delay_tdbwe );
            Copy( tmp_buffer_fx + output_frame - delay_tdbwe, st_fx->prev_synth_buffer_fx, delay_tdbwe );
        }

        test();
        IF( st_fx->igf != 0 && sub( st_fx->m_frame_type, ACTIVE_FRAME ) == 0 )
        {
            test();
            test();
            test();
            test();
            IF( st_fx->bfi_fx == 0 && sub(st_fx->core_fx, ACELP_CORE) == 0 && (st_fx->tec_flag != 0 || st_fx->tfa_flag != 0) &&  L_sub( st_fx->output_Fs_fx, 8000 ) > 0  )
            {
                tmp16 = 0;
                move16();
                if (sub(st_fx->tec_flag, 2) == 0)
                {
                    tmp16 = 1;
                    move16();
                }

                hb_synth_fx_exp = procTecTfa_TBE_Fx( hb_synth_fx, hb_synth_fx_exp, st_fx->tecDec_fx.pGainTemp_m, st_fx->tecDec_fx.pGainTemp_e,
                                                     st_fx->tfa_flag, st_fx->last_core_fx, shr(output_frame, 4 ), tmp16 );
            }

            test();
            test();
            test();
            test();
            test();
            test();
            IF( ( ( st_fx->bfi_fx == 0 || st_fx->last_core_fx == ACELP_CORE ) && st_fx->core_fx == ACELP_CORE ) ||
                ( ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 ) && ((sub(st_fx->bwidth_fx, SWB) == 0 || sub(st_fx->bwidth_fx, FB) == 0) && sub(st_fx->last_codec_mode, MODE2) == 0) ) )
            {
                add_vec_fx( output_sp, 0, hb_synth_fx, hb_synth_fx_exp, output_sp, 0, output_frame );
            }
        }


        IF( L_sub( st_fx->output_Fs_fx, 8000 ) == 0 )
        {
            tmps = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS );
        }
        ELSE
        {
            tmps = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS );
        }
        delta = NS2SA_fx2( st_fx->output_Fs_fx, TCXLTP_DELAY_NS );

        test();
        test();
        test();
        IF ( (st_fx->bfi_fx && sub(st_fx->last_core_fx, ACELP_CORE) > 0) || sub(st_fx->core_fx, ACELP_CORE) > 0)
        {
            test();
            test();
            test();
            test();
            IF ( sub(st_fx->last_core_bfi, ACELP_CORE) > 0 || (st_fx->bfi_fx && st_fx->last_core_fx > ACELP_CORE) || (st_fx->prev_bfi_fx && st_fx->last_con_tcx))
            {
                Copy(st_fx->FBTCXdelayBuf, output_sp, tmps);
                Copy(pcmbufFB, output_sp + tmps, sub(st_fx->L_frameTCX, tmps));
            }
            ELSE
            {
                Word16 step, alpha;

                i = 15;
                move16();
                step = Inv16(tmps, &i);
                step = shl(step, i);
                alpha = 0;
                move16();

                FOR (i = 0; i < tmps; i++)
                {
                    output_sp[i+tmps] = msu_r(L_mult(pcmbufFB[i], alpha), output_sp[i+tmps], add(alpha, -32768));
                    move16();
                    alpha = add(alpha, step);
                }
                Copy( pcmbufFB + tmps, output_sp + shl(tmps,1), sub(st_fx->L_frameTCX, shl(tmps,1)) );
            }

            Copy( pcmbufFB + st_fx->L_frameTCX - tmps, st_fx->FBTCXdelayBuf, tmps );

            test();
            IF( st_fx->bfi_fx && sub(st_fx->last_core_fx, ACELP_CORE) > 0 )
            {
                IF( L_sub(st_fx->output_Fs_fx, 8000) == 0 )
                {
                    Copy(st_fx->FBTCXdelayBuf, st_fx->delay_buf_out_fx, NS2SA(st_fx->output_Fs_fx, DELAY_CLDFB_NS));
                }
                ELSE
                {
                    Copy( st_fx->FBTCXdelayBuf, st_fx->prev_synth_buffer_fx, NS2SA(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS) );
                    Copy( st_fx->FBTCXdelayBuf + NS2SA(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS), st_fx->delay_buf_out_fx, NS2SA(st_fx->output_Fs_fx, DELAY_CLDFB_NS) );
                }
            }
        }
        ELSE IF( (sub(st_fx->last_codec_mode,MODE2)==0) && (sub(st_fx->last_core_fx, ACELP_CORE) > 0) )
        {
            Word16 step, alpha;

            Copy(st_fx->FBTCXdelayBuf, output_sp, delta);

            i = 15;
            move16();
            step = Inv16(sub(tmps,delta), &i);
            step = shl(step, i);
            alpha = 0;
            move16();

            FOR (i = delta; i < tmps; i++)
            {
                output_sp[i] = msu_r(L_mult(output_sp[i], alpha), st_fx->FBTCXdelayBuf[i], add(alpha, -32768));
                move16();
                alpha = add(alpha, step);
            }
        }

        Scale_sig(st_fx->tcxltp_mem_in, delta, sub(Qpostd, Qpostd_prev));
        Scale_sig(st_fx->tcxltp_mem_out, output_frame, sub(Qpostd, Qpostd_prev));
        test();
        tcx_ltp_post( st_fx->tcxltp, st_fx->core_fx, st_fx->L_frameTCX, st_fx->L_frame_fx, NS2SA_fx2( st_fx->output_Fs_fx, ACELP_LOOK_NS ) + tmps,
                      output_sp, st_fx->FBTCXdelayBuf, delta, st_fx->bfi_fx, st_fx->tcxltp_pitch_int, st_fx->tcxltp_pitch_fr, st_fx->tcxltp_gain,
                      &st_fx->tcxltp_pitch_int_post_prev, &st_fx->tcxltp_pitch_fr_post_prev, &st_fx->tcxltp_gain_post_prev, &st_fx->tcxltp_filt_idx_prev,
                      st_fx->pit_res_max,
                      &st_fx->pit_res_max_past,
                      st_fx->damping, L_sub(st_fx->total_brate_fx, 96000) >= 0, st_fx->tcxltp_mem_in, st_fx->tcxltp_mem_out, st_fx->total_brate_fx );

        if( sub(st_fx->last_codec_mode,MODE1) == 0 )
        {
            /*Update parameters*/
            st_fx->extl_fx = -1;
            move16();
        }

        Copy( output_sp, synth_fx, output_frame );

    }  /* end of MODE2 */


    /*----------------------------------------------------------------*
     * Save synthesis for HQ FEC
     *----------------------------------------------------------------*/
    post_hq_delay = NS2SA_fx2( st_fx->output_Fs_fx, POST_HQ_DELAY_NS );
    IF (sub(st_fx->codec_mode, MODE1) == 0)
    {
        Copy( st_fx->synth_history_fx+output_frame, st_fx->synth_history_fx, sub(shl(output_frame, 1), post_hq_delay));
        Copy_Scale_sig( synth_fx, st_fx->old_synthFB_fx+output_frame-post_hq_delay, output_frame,negate(Qpostd)); /* output_sp not initialized yet */
        IF( sub(output_frame, L_FRAME16k) >= 0 )
        {

            Copy_Scale_sig( st_fx->prev_synth_buffer_fx, st_fx->old_synthFB_fx+2*output_frame-NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS), NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS),negate(st_fx->Qprev_synth_buffer_fx));
        }
        IF( sub(st_fx->core_fx,ACELP_CORE) != 0 )
        {
            IF( sub(output_frame, L_FRAME16k) >= 0 )
            {

                Copy_Scale_sig( synth_fx+output_frame, st_fx->old_synthFB_fx+2*output_frame-NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS), NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS),negate(Qpostd));
                Copy_Scale_sig( st_fx->old_out_fx+NS2SA_fx2(st_fx->output_Fs_fx, N_ZERO_MDCT_NS), st_fx->old_synthFB_fx+2*output_frame, NS2SA_fx2(st_fx->output_Fs_fx, PH_ECU_LOOKAHEAD_NS), negate(st_fx->Q_old_wtda));
            }
            ELSE
            {

                Copy_Scale_sig( synth_fx+output_frame, st_fx->old_synthFB_fx+2*output_frame-NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS), NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS),negate(Qpostd));
                Copy_Scale_sig( st_fx->old_out_fx+NS2SA_fx2(st_fx->output_Fs_fx, N_ZERO_MDCT_NS), st_fx->old_synthFB_fx+2*output_frame-NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS), NS2SA_fx2(st_fx->output_Fs_fx, PH_ECU_LOOKAHEAD_NS), negate(st_fx->Q_old_wtda));
            }
        }
    }

    /*----------------------------------------------------------------*
     * HP filtering
     *----------------------------------------------------------------*/

    st_fx->Qprev_synth_buffer_fx=Qpostd;
    move16();
    Scale_sig32(st_fx->L_mem_hp_out_fx, 4, sub(Qpostd, Qpostd_prev));
    hp20(synth_fx, 1/*stride*/, output_frame, st_fx->L_mem_hp_out_fx, L_mult0(output_frame, 50));

    /*----------------------------------------------------------------*
     * Synthesis output
     *----------------------------------------------------------------*/

    IF( sub(st_fx->codec_mode,MODE1) == 0 )
    {
        /* final output of synthesis signal */
        syn_output_fx( st_fx->codec_mode, synth_fx, output_frame, output_sp, Qpostd  );
    }
    ELSE
    {
        Copy( synth_fx, output_sp, output_frame );
    }

    /*--------------------------------------------------------*
     * Updates
     *--------------------------------------------------------*/

    test();
    IF( st_fx->last_is_cng == 0 && sub(st_fx->codec_mode,MODE2) == 0 )
    {
        st_fx->bfi_fx = 0;
        move16();
        IF( st_fx->use_partial_copy && sub(st_fx->rf_frame_type, RF_TCXFD) >= 0 && sub(st_fx->rf_frame_type, RF_TCXTD2) <= 0)
        {
            if( sub(frameMode, FRAMEMODE_MISSING) == 0 )
            {
                st_fx->bfi_fx = 1;
                move16();
            }
        }
        ELSE IF( sub(st_fx->m_decodeMode, DEC_CONCEALMENT_EXT) == 0 )
        {
            st_fx->bfi_fx = 1;
            move16();
        }
        updt_dec_common_fx( st_fx, -1, output_sp );
    }
    ELSE
    {
        IF( sub(st_fx->codec_mode,MODE2) == 0 )
        {
            updt_dec_common_fx( st_fx, hq_core_type, output_sp );
        }
        ELSE
        {
            updt_dec_common_fx( st_fx, hq_core_type, synth_fx );
        }
    }
    IF( sub(st_fx->codec_mode,MODE2) == 0 )
    {
        test();
        IF(sub(st_fx->use_partial_copy,1)==0 && sub(st_fx->rf_frame_type,RF_NELP) == 0)
        {
            st_fx->last_nelp_mode_dec_fx = 1;
        }
        ELSE
        {
            st_fx->last_nelp_mode_dec_fx = 0;
        }
    }

    st_fx->prev_use_partial_copy = st_fx->use_partial_copy;
    move16();

    st_fx->prev_tilt_code_dec_fx = 0;
    move16();

    st_fx->prev_Q_exc = st_fx->Q_exc;
    move16();

    L_tmp = L_mult(st_fx->tilt_code_dec_fx[0], 4096);
    FOR( i=1; i<NB_SUBFR; i++ )
    {
        L_tmp = L_mac(L_tmp, st_fx->tilt_code_dec_fx[i], 4096);
    }
    st_fx->prev_tilt_code_dec_fx = round_fx(L_tmp);

    st_fx->prev_coder_type_fx = coder_type;
    move16();
    if( sub(st_fx->core_fx,HQ_CORE) == 0 )
    {
        st_fx->prev_coder_type_fx = GENERIC;
        move16();
    }

    test();
    IF ( L_sub(st_fx->core_brate_fx,SID_2k40) > 0 && sub(st_fx->first_CNG_fx,1) == 0 )
    {
        if( sub(st_fx->act_cnt_fx,BUF_DEC_RATE) >= 0 )
        {
            st_fx->act_cnt_fx = 0;
            move16();
        }

        st_fx->act_cnt_fx = add(st_fx->act_cnt_fx, 1);

        test();
        if( (sub(st_fx->act_cnt_fx,BUF_DEC_RATE)==0) && (st_fx->ho_hist_size_fx > 0) )
        {
            st_fx->ho_hist_size_fx = sub(st_fx->ho_hist_size_fx,1);
        }

        st_fx->act_cnt2_fx = add(st_fx->act_cnt2_fx,1);
        if( sub(st_fx->act_cnt2_fx,MIN_ACT_CNG_UPD) >= 0 )
        {
            st_fx->act_cnt2_fx = MIN_ACT_CNG_UPD;
            move16();
        }
    }
    test();
    test();
    if ( L_sub(st_fx->core_brate_fx,SID_2k40) <= 0 && st_fx->first_CNG_fx == 0 && sub(st_fx->cng_type_fx,LP_CNG) == 0 )
    {
        st_fx->first_CNG_fx = 1;
        move16();
    }

    /* update bandwidth switching parameters */
    IF( sub(st_fx->codec_mode, MODE1) == 0 )
    {
        updt_bw_switching_fx( st_fx, synth_fx, inner_frame_tbl, Qpostd );
    }
    ELSE
    {
        st_fx->last_bwidth_fx = st_fx->bwidth_fx;
        move32();
    }

    /* synchronisation of CNG seeds*/
    test();
    test();
    IF( st_fx->bfi_fx || (L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st_fx->core_brate_fx,SID_2k40) != 0) )
    {
        Random( &(st_fx->cng_seed_fx) );
        Random( &(st_fx->cng_ener_seed_fx) );
    }

    test();
    IF( st_fx->enablePlcWaveadjust && !concealWholeFrameTmp )
    {
        /* update the parameters used in waveform adjustment */
        concealment_update2_x( output_sp, &st_fx->plcInfo, st_fx->L_frameTCX );
    }

    if( st_fx->bfi_fx == 0 )
    {
        st_fx->last_total_brate_fx = st_fx->total_brate_fx;
        move32();
    }


    return;
}
