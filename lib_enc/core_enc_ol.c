/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"
#include "stl.h"
#include "rom_com_fx.h"



void closest_centroid_rf(const Word16 *data,            /* i  : input data */
                         const Word16 *weights,         /* i  : weights */
                         const Word16 *quantizer,       /* i  : quantizer table */
                         const Word16  centroids,       /* i  : number of centroids */
                         const Word16  length,          /* i  : dimension of quantiser */
                         Word16 *ind_vec);         /* o  : list of best match indice vectors */


/*-------------------------------------------------------------------*
 * core_encode_openloop()
 *
 * Open-loop core encoder
 *-------------------------------------------------------------------*/

void core_encode_openloop(
    Encoder_State_fx *st,                    /* i/o: encoder state structure             */
    const Word16 coder_type,             /* i  : coding type                         */
    const Word16 pitch[3],               /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],             /* i  : open-loop pitch gains               */
    const Word16 Aw[NB_SUBFR16k*(M+1)],  /* i  : weighted A(z) unquant. for subframes*/
    const Word16 *lsp_new,               /* i  : LSPs at the end of the frame        */
    const Word16 *lsp_mid,               /* i  : LSPs at the middle of the frame     */
    Word16 *pitch_buf,             /* i/o: floating pitch values for each subfr*/
    Word16 *voice_factors,         /* o  : voicing factors                     */
    Word16 *ptr_bwe_exc,           /* o  : excitation for SWB TBE              */
    const Word16 vad_hover_flag,
    Word16 Q_new,
    Word16 shift
)
{
    Word16 lsf_q[M], lsp_q[M], lspmid_q[M], lsf_tcx_q[M], lsp_tcx_q[M];
    Word16 lspq_ind[M];
    Word16 A_q_ind[M+1];
    Word16 tcx_lpc_cdk;
    Word16 A_w[M+1];
    Word16 A_q[NB_SUBFR16k*(M+1)];
    Word16 param_lpc[NPRM_LPC_NEW];
    Word16 nbits_lpc;
    Word16 param_core[2*NPRM_DIV];
    Word16 target_bits;
    Word16 stab_fac;
    Word32 spectrum_long[N_MAX]; /* MDCT output for a long block */
    Word16 spectrum_long_e;
    Word16 indexBuffer[N_MAX+1];
    CONTEXT_HM_CONFIG hm_cfg;
    Word16  bits_param_lpc[10], no_param_lpc;

    Word16 i;
    /* lsf quant parameters */
    Word16 lsp_q_rf[M];
    Word16 Aq_rf[NB_SUBFR*(M+1)];
    Word16 stab_fac_rf;
    Word16 *exc_rf;
    Word16 *syn_rf;
    Word16 tmp;
    Word16 rf_PLC_Mode;
    Word16 TD_mode;
    Word16 xsp[M], xsf[M];
    Word16 rf_mem_MA[M];

    Word16 exc_buf_rf[L_EXC_MEM + L_FRAME + 1];
    Word16 syn_buf_rf[M+L_FRAME16k+L_FRAME16k/2];

    Word16 w_rf[M+1], lsf_uq_rf[M+1];
    Word16 lsf_q_1st_rf[M+1], lsf_q_d_rf[M+1], lsf_q_rf[M+1];
    Word16 lsp_old_q_rf[M+1], lsf_old_q_rf[M+1];


    /* copy primary memories to use later during partial copy assembly */

    /* These primary memories are already scaled by (Q_new-st->prev_Q_new) inside pre_proc_fx()
       and copied to the partial frame at start of each frame to be used in computing partial copy params.
       But then they are discarded, not need for continuation
       The idea is to not maintain these memories seperately, and have the ability to generate the
       primary copy memories using the partial copy parameters at decode, so same in enc
    */

    /* back up the old LSPs and LSFs */
    Copy(st->lsp_old_fx, lsp_old_q_rf, M);
    Copy(st->lsf_old_fx, lsf_old_q_rf, M);

    /* back up old exc before primary encoding */
    set16_fx( exc_buf_rf, 0, (L_EXC_MEM+L_FRAME+1) );
    exc_rf = exc_buf_rf + L_EXC_MEM;
    Copy(st->LPDmem.old_exc, exc_buf_rf, L_EXC_MEM);

    /* back up old synthesis before primary encoding */
    set16_fx( syn_buf_rf, 0, (M+L_FRAME16k+L_FRAME16k/2) );
    syn_rf = syn_buf_rf + M;
    Copy(st->LPDmem.mem_syn, syn_buf_rf, M);

    /* back up syn2 mem */
    Copy(st->LPDmem.mem_syn2, st->rf_mem_syn2, M);

    /* back up LPD mem_w0 target generation memory */
    st->rf_mem_w0 = st->LPDmem.mem_w0;

    /* back up clip gain memory */
    Copy( st->clip_var_fx, st->rf_clip_var, 6 );

    /* back up tilt code */
    st->rf_tilt_code = st->LPDmem.tilt_code;

    /* back up dispMem */
    st->rf_dm_fx.prev_state = st->dm_fx.prev_state;
    st->rf_dm_fx.prev_gain_code = st->dm_fx.prev_gain_code;
    FOR(i=0; i<6; i++)
    {
        st->rf_dm_fx.prev_gain_pit[i] = st->dm_fx.prev_gain_pit[i];
    }

    /* back up gc_threshold for noise addition */
    st->rf_gc_threshold = st->LPDmem.gc_threshold;


    /* initialization */
    tcx_lpc_cdk = 0;
    move16();
    set16_fx( param_lpc, 0, NPRM_LPC_NEW );
    set16_fx( param_core, 0, 2*NPRM_DIV );
    Copy( st->tcxltp_param, &param_core[1+NOISE_FILL_RANGES], LTPSIZE );

    no_param_lpc = 0;
    move16();   /* avoid MSVC warnings */
    nbits_lpc = 0;
    move16();   /* avoid MSVC warnings */
    stab_fac = 0;
    move16();   /* avoid MSVC warnings */

    set32_fx(spectrum_long, 0, N_MAX);

    hm_cfg.indexBuffer = indexBuffer;
    move16();

    /*--------------------------------------------------------------*
    * LPC Quantization
    *---------------------------------------------------------------*/

    st->acelp_cfg.midLpc = st->acelp_cfg.midLpc_enable;
    move16();
    test();
    if ( (sub(st->lpcQuantization, 1) == 0) && (sub(coder_type, VOICED) == 0))
    {
        st->acelp_cfg.midLpc = 0;
        move16();
    }

    test();
    IF ( st->core_fx==ACELP_CORE || !st->enableTcxLpc )
    {
        IF (st->envWeighted)
        {
            /* Unweight the envelope */
            E_LPC_lsp_unweight(
                st->lsp_old_fx,
                st->lsp_old_fx,
                st->lsf_old_fx,
                st->inv_gamma,
                M
            );
            st->envWeighted = 0;
            move16();
        }

        IF(sub(st->core_fx,TCX_20_CORE)==0)
        {
            lpc_quantization( st, st->core_fx, st->lpcQuantization, st->lsf_old_fx, lsp_new, lsp_mid,
                              lsp_q, lsf_q, lspmid_q,  lspq_ind, st->clip_var_fx, st-> mem_MA_fx, st->mem_AR_fx,
                              st->narrowBand,  AUDIO, st->acelp_cfg.midLpc,  param_lpc, &nbits_lpc, bits_param_lpc, &no_param_lpc,
                              &(st->seed_acelp), st->Bin_E_fx, st->Bin_E_old_fx, add(Q_new, Q_SCALE - 2) );

        }
        ELSE
        {

            lpc_quantization(
                st,
                st->core_fx,
                st->lpcQuantization,
                st->lsf_old_fx,
                lsp_new,
                lsp_mid,
                lsp_q,
                lsf_q,
                lspmid_q,
                lspq_ind,
                st->clip_var_fx,
                st-> mem_MA_fx,
                st->mem_AR_fx,
                st->narrowBand,
                coder_type,
                st->acelp_cfg.midLpc,
                param_lpc,
                &nbits_lpc,
                bits_param_lpc,
                &no_param_lpc,
                &(st->seed_acelp),
                st->Bin_E_fx,
                st->Bin_E_old_fx,
                add(Q_new, Q_SCALE - 2));
        }

        /*-------------------------------------------------------------*
         * Rate switching: reset
         *---------------------------------------------------------------*/
        IF( st->rate_switching_reset!=0 )
        {
            Copy( lsp_q, st->lsp_old_fx, M );
            Copy( lsf_q, st->lsf_old_fx, M );
            Copy( lsp_q, lspmid_q, M );
        }

        stab_fac = lsf_stab_fx(lsf_q, st->lsf_old_fx, 0, st->L_frame_fx);
    }




    /*--------------------------------------------------------------*
    * Run ACELP
    *---------------------------------------------------------------*/
    IF ( st->core_fx==ACELP_CORE )
    {
        IF ( st->acelp_cfg.midLpc != 0 )
        {
            int_lsp4_fx( st->L_frame_fx, st->lsp_old_fx, lspmid_q, lsp_q, A_q, M, 0, 0 );
        }
        ELSE
        {
            int_lsp_fx( st->L_frame_fx, st->lsp_old_fx, lsp_q, A_q, M, 0, interpol_frac_fx, 0 );
        }

        /* Calculate target bits */
        target_bits = sub(sub(st->bits_frame_core, nbits_lpc), st->nb_bits_header_ace);

        if(sub(st->rf_mode,1) == 0)
        {
            /* joint bit allocation for redundant frame and TBE */
            /* calculate target bits for core coding */
            target_bits = sub(target_bits, st->rf_target_bits_write);
        }
        IF( st->igf != 0 )
        {
            target_bits = sub( target_bits, get_tbe_bits_fx( st->total_brate_fx, st->bwidth_fx, st->rf_mode ) );

        }

        if ( st->acelp_cfg.midLpc != 0)
        {
            target_bits = sub(target_bits, MIDLSF_NBITS );
        }

        if( st->plcExt.enableGplc )
        {
            target_bits = sub(target_bits, st->plcExt.nBits);
        }

        /* reset TBE buffers previous frame frame wasn't ACELP*/
        IF( sub( st->last_core_fx, ACELP_CORE ) != 0 )
        {
            TBEreset_enc_fx( st, st->bwidth_fx );
        }


        /* Run ACELP encoder */
        coder_acelp(
            &(st->acelp_cfg),
            coder_type,
            Aw,
            A_q,
            st->speech_enc_pe,
            st->synth,
            &(st->LPDmem),
            voicing,
            pitch,
            param_core,
            stab_fac,
            st,
            &st->plcExt,
            target_bits,
            Q_new,
            shift,
            pitch_buf,
            voice_factors,
            ptr_bwe_exc
        );


        st->glr_idx[0] = encSideSpecPowDiffuseDetector( st->plcExt.last_lsf_ref,
                         st->plcExt.last_lsf_con,
                         st->last_sr_core,
                         &(st->prev_lsf4_mean),
                         st->glr
                                                      );
        Copy(lsf_q, st->plcExt.last_lsf_ref, M);
        Copy(st->plcExt.lsf_con, st->plcExt.last_lsf_con, M);

        updateSpecPowDiffuseIdx(st);

        if(sub(st->last_stab_fac, FL2WORD16(0.02f)) > 0)
        {
            move16();
            st->glr_idx[0] = 0;
        }
        move16();
        st->last_stab_fac = stab_fac;

        move16();
        st->plcExt.LPDmem = &st->LPDmem;

        encoderSideLossSimulation( st, &st->plcExt, lsf_q, stab_fac, !(st->plcExt.enableGplc), st->L_frame_fx);

        st->tcxltp_norm_corr_past = voicing[1];
        move16();

        st->tcx_cfg.tcx_curr_overlap_mode = st->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;
        move16();

    }



    /*--------------------------------------------------------------*
    * Run TCX20
    *---------------------------------------------------------------*/

    IF ( sub(st->core_fx, TCX_20_CORE) == 0 )
    {
        IF (st->enableTcxLpc)
        {
            IF( sub(st->rf_mode,1)==0)
            {
                Copy(st->mem_MA_fx, rf_mem_MA, M);
            }

            tcx_lpc_cdk = tcxlpc_get_cdk(st->tcx_cfg.coder_type);

            /* Get the envelope corresponding to the current frame */
            E_LPC_int_lpc_tcx( st->lspold_enc_fx, lsp_new, A_q );

            /* Weight the envelope */
            weight_a_fx(A_q, A_q, st->gamma, M);

            /* Save the weighted envelope */
            Copy(A_q, A_w, M+1);

            /* Convert to xSP and xSF */
            E_LPC_a_lsp_conversion(A_q, xsp, lsp_new, M );
            E_LPC_lsp_lsf_conversion(xsp, xsf, M);

            /* Quantize */
            Q_lsf_tcxlpc(xsf,
                         lsf_tcx_q,
                         lspq_ind,
                         param_lpc, M, st->narrowBand, tcx_lpc_cdk, st-> mem_MA_fx
                         ,st->tcx_cfg.coder_type, st->Bin_E_fx, add(Q_new, Q_SCALE-2)
                        );

            /* Account for consumed bits */
            nbits_lpc = TCXLPC_NUMBITS;
            move16();
            if (param_lpc[0])
            {
                nbits_lpc = add(nbits_lpc, TCXLPC_IND_NUMBITS);
            }

            /* Convert quantized xSF to xSP and A */
            E_LPC_lsf_lsp_conversion(lsf_tcx_q, lsp_tcx_q, M);
            E_LPC_f_lsp_a_conversion(lsp_tcx_q, A_q, M);
        }
        ELSE
        {
            E_LPC_int_lpc_tcx(
                st->lsp_old_fx,
                lsp_q,
                A_q
            );
        }

        IF (st->tcx_lpc_shaped_ari != 0)
        {
            E_LPC_f_lsp_a_conversion(lspq_ind, A_q_ind, M);
        }

        /* Calculate target bits */
        target_bits = sub(sub(st->bits_frame_core, nbits_lpc), st->nb_bits_header_tcx);
        if(sub(st->rf_mode,1) == 0)
        {
            /* joint bit allocation for redundant frame and TBE */
            /* calculate target bits for core coding */
            target_bits = sub(target_bits, st->rf_target_bits_write);
        }
        IF (sub(st->mdct_sw, MODE1) == 0)
        {
            /* Account for core mode signaling bits difference: bandwidth and ACELP/TCX signaling bit are replaced */
            target_bits = add(target_bits, sub(add(FrameSizeConfig[st->frame_size_index].bandwidth_bits, 1), signalling_mode1_tcx20_enc(st, 0)));
        }
        ELSE
        if ( sub(st->mdct_sw_enable, MODE2) == 0 )
        {
            target_bits = sub(target_bits, 1);
        }

        if( st->plcExt.enableGplc )
        {
            target_bits = sub(target_bits, st->plcExt.nBits);
        }

        /* subtract bits for TCX overlap mode (1 bit: full, 2 bits: half or no overlap) */
        target_bits = sub(target_bits,1);
        test();
        if (sub(st->tcx_cfg.tcx_curr_overlap_mode, HALF_OVERLAP) == 0 || sub(st->tcx_cfg.tcx_curr_overlap_mode, MIN_OVERLAP) == 0)
        {
            target_bits = sub(target_bits,1);
        }

        target_bits = sub(target_bits, st->tcxltp_bits);


        coder_tcx(
            0,
            &(st->tcx_cfg),
            A_q,
            A_q_ind,
            st->synth,
            st->L_frame_fx,
            st->L_frameTCX,
            st->tcx_cfg.tcx_coded_lines,
            target_bits,
            st->tcxonly,
            spectrum_long,
            &spectrum_long_e,
            &(st->LPDmem),
            param_core,
            st,
            &hm_cfg
        );

        coder_tcx_post( st, &(st->LPDmem), &(st->tcx_cfg), st->synth, A_q, Aw, st->wspeech_enc, Q_new, shift );


        move16();
        st->plcExt.LPDmem = &(st->LPDmem);

        GplcTcxEncSetup(st, &st->plcExt, Q_new);

        IF (st->enableTcxLpc)
        {
            E_LPC_lsp_unweight(
                lsp_tcx_q,
                lsp_q,
                lsf_q,
                st->inv_gamma,
                M
            );
        }
        encoderSideLossSimulation( st, &st->plcExt, lsf_q, stab_fac, 1, st->L_frame_fx );

    }



    /* Update lsp/lsf memory */
    Copy( lsp_new, st->lspold_enc_fx, M );

    test();
    IF ( st->enableTcxLpc && st->core_fx != ACELP_CORE )
    {
        /* Update lsf/lsp memory */
        Copy(lsf_tcx_q, st->lsf_old_fx, M);
        Copy(lsp_tcx_q, st->lsp_old_fx, M);
        st->envWeighted = 1;
        move16();

        /* Update ACELP quantizer state */
        lsf_update_memory(st->narrowBand,
                          st->lsf_old_fx,
                          st-> mem_MA_fx,
                          st-> mem_MA_fx,
                          M);

        st->pstreaklen_fx = 0;
        st->streaklimit_fx = 32767;
        /* check resonance for pitch clipping algorithm */
        gp_clip_test_lsf_fx( st->lsf_old_fx, st->clip_var_fx, 0 );
        Copy(st->lsf_old_fx, st->mem_AR_fx, M);
    }
    ELSE
    {
        /* Update ISP/ISF memory */
        Copy(lsf_q, st->lsf_old_fx, M);
        Copy(lsp_q, st->lsp_old_fx, M);
    }

    /*--------------------------------------------------------------*
     * Update LP_CNG parameters
     *--------------------------------------------------------------*/

    test();
    IF( st->Opt_DTX_ON_fx != 0 && vad_hover_flag != 0 )
    {
        st->burst_ho_cnt_fx = add(st->burst_ho_cnt_fx,1);

        if( sub(st->burst_ho_cnt_fx,HO_HIST_SIZE) > 0 )
        {
            st->burst_ho_cnt_fx = HO_HIST_SIZE;
            move16();
        }
    }
    ELSE
    {
        st->burst_ho_cnt_fx = 0;
        move16();
    }

    IF( st->Opt_DTX_ON_fx != 0 )
    {
        /* update CNG parameters in active frames */
        IF ( sub(st->bwidth_fx,NB) == 0 && st->enableTcxLpc && st->core_fx != ACELP_CORE )
        {
            Word16 buf[L_LP], res[L_FRAME], A[M+1], r_l[M+1], r_h[M+1], lsptmp[M], Q_r, tmp;
            assert(st->L_frame_fx==L_FRAME);
            Copy(st->synth+L_FRAME-L_LP, buf, L_LP);
            tmp = st->synth[L_FRAME-L_LP-1];
            E_UTIL_f_preemph2(Q_new-1, buf, st->preemph_fac, L_LP, &tmp);
            autocorr_fx( buf, M, r_h, r_l, &Q_r, L_LP, Assym_window_W16fx, 0, 0 );
            lag_wind(r_h, r_l, M, INT_FS_FX, LAGW_WEAK);
            E_LPC_lev_dur(r_h, r_l, A, NULL, M, NULL);
            E_LPC_a_lsp_conversion(A, lsptmp, lsp_new, M);
            Residu3_fx(A, buf+L_LP-L_FRAME, res, L_FRAME, 1);
            cng_params_upd_fx( lsptmp, res, st->L_frame_fx, &st->ho_circ_ptr_fx,
                               st->ho_ener_circ_fx, &st->ho_circ_size_fx, st->ho_lsp_circ_fx,
                               Q_new, ENC, NULL, &st->cng_buf_cnt, st->cng_exc2_buf,
                               st->cng_Qexc_buf, st->cng_brate_buf, st->last_active_brate_fx );
        }
        ELSE
        {
            cng_params_upd_fx( lsp_new, st->LPDmem.old_exc+L_EXC_MEM-st->L_frame_fx,
            st->L_frame_fx, &st->ho_circ_ptr_fx, st->ho_ener_circ_fx,
            &st->ho_circ_size_fx, st->ho_lsp_circ_fx, Q_new, ENC, NULL,
            &st->cng_buf_cnt, st->cng_exc2_buf,
            st->cng_Qexc_buf, st->cng_brate_buf,
            st->last_active_brate_fx );
        }

        IF( sub(st->L_frame_fx,L_FRAME) == 0 )
        {
            /* store LSPs@16k, potentially to be used in CNG@16k */
            Copy( st->lsp_old16k_fx, &(st->ho_lsp_circ2_fx[(st->ho_circ_ptr_fx)*M]), M );
        }

        /* Set 16k LSP flag for CNG buffer */
        st->ho_16k_lsp_fx[st->ho_circ_ptr_fx] = 1;
        move16();
        if ( sub(st->L_frame_fx,L_FRAME) == 0 )
        {
            st->ho_16k_lsp_fx[st->ho_circ_ptr_fx] = 0;
            move16();
        }

        /* efficient DTX hangover control */
        IF ( sub(st->burst_ho_cnt_fx, 1) > 0 )
        {
            dtx_hangover_control_fx( st, lsp_new );
        }
    }

    /*--------------------------------------------------------------*
    * Adaptive Bass Post-filter
    *---------------------------------------------------------------*/

    test();
    IF (sub(st->core_fx, ACELP_CORE)>0 ||  (st->rate_switching_reset!=0))   /*TCX mode: copy values*/
    {
        set16_fx(st->mem_bpf.noise_buf, 0, 2*L_FILT16k);  /*TCX->no gain*/
        set16_fx(st->mem_bpf.error_buf, 0, L_FILT16k);  /*TCX->no gain*/
        st->bpf_gain_param=0;
    }
    ELSE IF (st->acelp_cfg.bpf_mode > 0)   /*ACELP: estimate bpf parameter with delay=0*/
    {

        /*Estimate bpf parameter*/
        bass_pf_enc(
            st->speech_enc,
            st->synth,
            st->bpf_T,
            st->bpf_gainT,
            st->L_frame_fx,
            L_SUBFR,
            &(st->bpf_gain_param),
            st->acelp_cfg.bpf_mode,
            &(st->mem_bpf)
        );
    }




    /*--------------------------------------------------------------*
      * Analysis Print Out
      *---------------------------------------------------------------*/


    /*--------------------------------------------------------------*
    * Generate Bitstream
    *---------------------------------------------------------------*/

    enc_prm( coder_type, param_core, param_lpc, st, st->L_frame_fx, &hm_cfg, bits_param_lpc, no_param_lpc );

    /* Channel-aware mode - encode partial copy */
    IF( sub(st->rf_mode,1)==0)
    {
        set16_fx(lsf_q_1st_rf, 0, M);
        IF (sub(st->core_fx, ACELP_CORE) == 0)
        {
            /* convert LSPs to LP coefficients */
            lsp2lsf_fx( lsp_new, lsf_uq_rf, M, st->sr_core );
            /*i: lsp_new Q15 */
            /*o: lsf_uq_rf Qx2.56*/

            /* first stage VQ, 8 bits; reuse TCX high rate codebook */
            st->rf_indx_lsf[0][0] = vlpc_1st_cod(lsf_uq_rf, lsf_q_1st_rf, w_rf, st->rf_mode);
            /*v_sub(lsf_uq_rf, lsf_q_1st_rf, lsf_q_d_rf, M);*/
            FOR (i=0; i<M; i++)
            {
                lsf_q_d_rf[i] = shl(mult_r(sub(lsf_uq_rf[i],lsf_q_1st_rf[i]), 25600),5);
                /*input value is in Qx2.56, convert to Q6 to match table, quantizer table kept at Q6 to avoid losing precision */
                /*Assume this difference data max range can be represented by Q6*/
            }
            /*o: lsf_q_1st_rf in Qx2.56*/
            /*o: lsf_q_d_rf   in Q6*/

            /* second stage vq  */
            closest_centroid_rf(lsf_q_d_rf, w_rf, lsf_q_diff_cb_8b_rf, (1<<8), M, &st->rf_indx_lsf[0][1]);
            /*i: lsf_q_d_rf   in Q6 */
            /*o: quantization index  Q0 */

            /* quantized lsf from two stages  */
            /*v_add(lsf_q_1st_rf, lsf_q_diff_cb_8b_rf + M * st->rf_indx_lsf[0][1], lsf_q_rf, M);*/
            FOR (i=0; i<M; i++)
            {
                tmp = lsf_q_diff_cb_8b_rf[i+ M*st->rf_indx_lsf[0][1]]; /*tmp = quantized lsf_q_d_rf in Q6*/
                tmp = shr(mult_r(tmp,20972),4); /* bring lsf_q_d_rf to Qx2.56 for addition */
                lsf_q_rf[i] = add(lsf_q_1st_rf[i], tmp);
            }

            v_sort(lsf_q_rf, 0, ORDER-1);
            reorder_lsf_fx( lsf_q_rf, LSF_GAP_FX, ORDER, st->sr_core );
        }
        ELSE
        {
            Word16 rf_tcx_lpc_cdk;

            rf_tcx_lpc_cdk = tcxlpc_get_cdk( GENERIC );
            /* Quantize */
            Q_lsf_tcxlpc(xsf,
            lsf_tcx_q,
            lspq_ind,
            param_lpc, M, st->narrowBand, rf_tcx_lpc_cdk, rf_mem_MA,
            GENERIC, st->Bin_E_fx, add(Q_new, Q_SCALE-2)
                        );

            /* VQ, 5+4+4 bits; reuse TCX low rate codebook */
            st->rf_indx_lsf[0][0] = param_lpc[1];
            st->rf_indx_lsf[0][1] = param_lpc[2];
            st->rf_indx_lsf[0][2] = param_lpc[3];
        }

        IF (sub(st->core_fx, ACELP_CORE) == 0)
        {
            /* current n-th ACELP frame and its corresponding partial copy */

            /*lsf2lsp( lsf_q_rf, lsp_q_rf, M, st->sr_core );*/
            E_LPC_lsf_lsp_conversion( lsf_q_rf, lsp_q_rf, M );
            /*i: lsf_q_rf in Qx2.56*/
            /*o: lsp_q_rf in Q15*/

            /* Interpolate LSPs and convert to LPC */
            int_lsp_fx( st->L_frame_fx, lsp_old_q_rf, lsp_q_rf, Aq_rf, M, 0, interpol_frac_fx, 0 );

            /* stability estimation  */
            stab_fac_rf = lsf_stab_fx( lsf_q_rf, lsf_old_q_rf, 0, st->L_frame_fx ); /*Q15*/

            /* Configure partial copy estimation of the current n-th frame to be packed in future with n+fec_offset frame  */
            /* o: rf_frame_type, o: rf_target_bits */
            BITS_ALLOC_ACELP_config_rf( coder_type, st->rf_tilt_buf, &st->rf_frame_type, &st->rf_target_bits, st->nb_subfr
                                        , st->rf_fec_indicator, pitch_buf
                                      );

            /* RF frame type in the buffer */
            st->rf_indx_frametype[0] = st->rf_frame_type;
            st->rf_targetbits_buff[0] = st->rf_target_bits;

            IF( sub(st->rf_frame_type,RF_NO_DATA) != 0 )
            {
                /* coder_acelp_rf does the partial copy encoding based on the rf frame type chosen for the RF encoding */
                coder_acelp_rf(&(st->acelp_cfg_rf), coder_type, Aw, Aq_rf,  st->speech_enc_pe, voicing, pitch,
                               stab_fac_rf, st, st->rf_target_bits, st->rf_frame_type, exc_rf, syn_rf, Q_new, shift);
            }
        }
        ELSE
        {
            st->rf_clas[0] = st->clas_fx;
            move16();
            st->rf_gain_tcx[0] = param_core[0];
            move16();

            /* attenuate somewhat the gain for onset when the correlation with previous frame is too low: avoid preecho */
            tmp = mult_r(shl(st->rf_gain_tcx[1], 1), FL2WORD16(0.8f));

            test();
            test();
            IF( (st->rf_gain_tcx[1] != 0) && (sub(st->rf_gain_tcx[0], tmp) > 0) && (sub(st->tcxltp_gain, FL2WORD16(0.2)) <= 0) )
            {
                st->rf_gain_tcx[0] = tmp;
                move16();

                if( sub(tmp, 127) > 0)
                {
                    st->rf_gain_tcx[0] = 127;
                    move16();
                }
            }

            /* get concealment decision*/
            rf_PLC_Mode = 0;
            move16();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF(
                (sub(st->core_fx, TCX_20_CORE) == 0)/*(st->core == TCX_20_CORE)*/
                && (sub(st->last_core_fx,TCX_20_CORE) == 0)/*&&(st->last_core == TCX_20_CORE)*/
                && (sub(st->rf_second_last_core, TCX_20_CORE) == 0)/*&& (st->rf_second_last_core == TCX_20_CORE)*/
                && ( (sub(st->tcxltp_pitch_int, shr(st->L_frame_fx, 1)) <= 0) || ( sub(st->tcxltp_gain, FL2WORD16(0.4f)) <= 0) )/*&& ((st->tcxltp_pitch_int <= 0.5f*st->L_frame) || ( st->tcxltp_gain <= 0.4f))*/
                && (sub(st->tcxltp_pitch_int, st->rf_tcxltp_pitch_int_past) == 0)/*&& (st->tcxltp_pitch_int == st->rf_tcxltp_pitch_int_past)*/
                && (st->rf_last_tns_active == 0)/*!st->rf_last_tns_active*/
                && (st->rf_second_last_tns_active == 0)/*!st->rf_second_last_tns_active*/
                && ( (st->tcx_cfg.fIsTNSAllowed & st->fUseTns[0]) == 0)/*!(st->tcx_cfg.fIsTNSAllowed & st->fUseTns[0])*/
            )
            {
                rf_PLC_Mode = 1;
                move16();
            }
            ELSE IF (st->last_core_fx != 0)
            {
                test();
                test();
                test();
                IF ( ((sub(st->clas_fx, UNVOICED_TRANSITION) <= 0) || (sub(st->last_clas_fx, UNVOICED_TRANSITION) <= 0) || (sub(st->tcxltp_gain, FL2WORD16(0.4f)) <= 0))
                     && sub(st->last_core_fx, -1) != 0 )
                {
                    rf_PLC_Mode = st->last_core_fx;
                    move16();
                }
            }

            /* call TD1 when the gain drop compare to previous frame*/
            test();
            test();
            test();
            test();
            IF( rf_PLC_Mode == 0 && st->rf_gain_tcx[1] != 0 &&
                ( (st->transientDetection.transientDetector.bIsAttackPresent != 0 && sub(st->rf_gain_tcx[0], mult_r(st->rf_gain_tcx[1], FL2WORD16(0.97f))) < 0) ||
                  sub(st->rf_gain_tcx[0], mult_r(st->rf_gain_tcx[1], FL2WORD16(0.90f))) < 0 )
              )
            {
                TD_mode = 0;
            }
            ELSE
            {
                TD_mode = 1;
            }

            /* updates */
            st->rf_tcxltp_pitch_int_past  = st->tcxltp_pitch_int;
            move16();
            st->rf_second_last_tns_active = st->rf_last_tns_active;
            move16();
            st->rf_last_tns_active        = (st->tcx_cfg.fIsTNSAllowed & st->fUseTns[0]);
            move16();
            st->rf_second_last_core       = st->last_core_fx;
            move16();

            st->rf_tcxltp_param[0] = st->tcxltp_param[1];
            move16();

            /* Configure partial copy estimation of the current n-th frame to be packed in future with n+fec_offset frame */
            /* o: rf_frame_type, o: rf_target_bits */
            BITS_ALLOC_TCX_config_rf( &st->rf_frame_type, &st->rf_target_bits, rf_PLC_Mode, coder_type, st->last_core_fx, TD_mode);

            /* RF frame type in the buffer */
            st->rf_indx_frametype[0] = st->rf_frame_type;
            move16();
            st->rf_targetbits_buff[0] = st->rf_target_bits;
            move16();

        }
    }



    return;
}


/*-------------------------------------------------------------------*
* closest_centroid_rf()
*
* Determine a set of closest VQ centroids for a given input
*-------------------------------------------------------------------*/
void closest_centroid_rf(
    const Word16 *data,            /* i  : input data Qx */
    const Word16 *weights,         /* i  : weights */
    const Word16 *quantizer,       /* i  : quantizer table Qx */
    const Word16  centroids,       /* i  : number of centroids */
    const Word16  length,          /* i  : dimension of quantiser */
    Word16 *ind_vec          /* o  : list of best match indice vectors */
)
{
    Word16 i,j;
    Word16 tmp, tmpL;
    Word32 werr, best_werr;
    Word32 L_tmp;


    ind_vec[0] = 0;
    move16();
    best_werr = MAX_32;
    move32();

    FOR( i = 0; i < centroids; i++ )
    {
        werr = L_deposit_l(0);
        tmpL = i_mult2(i, length);
        FOR( j = 0; j < length; j++ )
        {
            tmp = sub( data[j], quantizer[tmpL + j] );
            L_tmp = L_mult( tmp, tmp );
            werr = Madd_32_16( werr, L_tmp, weights[j] );
        }

        IF( werr < best_werr )
        {
            ind_vec[0] = i;
            best_werr = werr;
        }
    }

    return;
}


void core_acelp_tcx20_switching(
    Encoder_State_fx *st,            /* i/o: encoder state structure             */
    const Word16 vad_flag,
    Word16 sp_aud_decision0,
    Word16 non_staX,
    Word16 *pitch,         /* i  : open-loop pitch values for quantiz. */
    Word16 *pitch_fr,      /* i/o: fraction pitch values               */
    Word16 *voicing_fr,    /* i/o: fractional voicing values           */
    const Word16 currFlatness,   /* i  : flatness                            */
    const Word16 lsp_mid[M],     /* i  : LSPs at the middle of the frame     */
    const Word16 stab_fac,       /* i  : LP filter stability                 */
    Word16 Q_new,
    Word16 shift
)
{
    Word16 i, j, iter;
    Word16 xn_buf[L_MDCT_OVLP_MAX+L_FRAME_PLUS+L_MDCT_OVLP_MAX];
    Word16 Ap[M+1];
    Word16 gainlpc[FDNS_NPTS];
    Word16 gainlpc_e[FDNS_NPTS];
    Word32 en[N_MAX/4];
    Word32 ener, tmp32, fac, offset;
    Word16 ener_e;
    Word16 L_frame;
    Word16 overlap;
    Word16 x_e;
    Word16 tcx_offset;
    Word32 spectrum_long[N_MAX];
    Word32 *x = spectrum_long;
    Word32 target;
    Word32 tcx_snr;
    Word16 tmp16, s;
    Word16 L_frame_4;
    Word16 i2, T0;
    Word32 gain, signal, noise;
    Word16 A_q_tcx[(NB_SUBFR16k+1)*(M+1)];
    Word16 snr_tcx, snr_acelp, dsnr;

    /* Check minimum pitch for quantization */
    FOR( i = 0; i < 3; i++ )
    {
        tmp16 = pitch[i];
        move16();
        /* check minimum pitch for quantization */
        if (sub(tmp16, PIT_MIN_SHORTER) < 0)
        {
            tmp16 = shl(tmp16, 1);
        }

        /* convert pitch values to 16kHz domain */
        s = mult_r(tmp16, FL2WORD16(0.25f));
        if (sub(st->L_frame_fx, L_FRAME16k) == 0)
        {
            /*pitch[i] = (short)(pitch[i] * 1.25f + 0.5f);*/
            tmp16 = add(tmp16, s);
        }
        pitch[i] = tmp16;
        move16();
    }
    IF (st->narrowBand != 0)
    {
        pitchDoubling_det( st->wspeech_enc, pitch, pitch_fr, voicing_fr );
    }

    E_LPC_f_lsp_a_conversion(lsp_mid, A_q_tcx, M);


    /* LTP */

    tcx_ltp_encode( st->tcxltp,
                    st->tcxonly,
                    TCX_20,
                    st->L_frame_fx,
                    L_SUBFR,
                    st->speech_enc + st->encoderLookahead_enc,
                    st->speech_ltp + st->encoderLookahead_enc,
                    st->wspeech_enc + st->encoderLookahead_enc,
                    pitch[1],
                    st->tcxltp_param,
                    &st->tcxltp_bits,
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
                    0,
                    A_q_tcx,
                    M
                  );

    /* Force TCX when TCX20 in MODE1 is selected */
    IF ( sub(st->mdct_sw, MODE1) == 0 )
    {
        st->core_fx = TCX_20_CORE;
        move16();
    }
    ELSE
    {
        /*--------------------------------------------------------------*
        * Estimate TCX SNR
        *---------------------------------------------------------------*/

        L_frame = st->L_frame_fx;
        move16();
        tcx_offset = st->tcx_cfg.tcx_offset;
        move16();

        target = L_add(0x14C315C, 0); /* 1000.f * log2(10)/10 (15Q16) */
        test();
        if (L_sub(st->sr_core, 16000) == 0 || L_sub(st->sr_core, 12800) == 0)
        {
            target = L_add(0x11A5D28, 0); /* 850.f * log2(10)/10 (15Q16) */
        }
        if ( st->narrowBand != 0 )
        {
            target = L_add(0xA618AE, 0); /* 500f * log2(10)/10 (15Q16) */
        }

        IF (st->last_core_fx == ACELP_CORE)
        {
            L_frame = add(L_frame, tcx_offset);
            tcx_offset = s_min(st->tcx_cfg.lfacNext, 0);
            L_frame = sub(L_frame, tcx_offset);
        }
        L_frame_4 = shr(L_frame, 2);
        overlap = st->tcx_cfg.tcx_mdct_window_delay;
        move16();
        Copy(st->speech_ltp + sub(tcx_offset, shr(overlap, 1)), xn_buf, add(L_frame, overlap));

        tmp16 = shr(overlap, 1);
        IF (sub(st->last_core_fx,ACELP_CORE)==0)
        {
            IF (tcx_offset < 0)
            {
                set16_fx(xn_buf, 0, tmp16);
            }
        }
        ELSE
        {
            FOR (i = 0; i < tmp16; i++)
            {
                xn_buf[i] = mult_r(xn_buf[i], st->tcx_cfg.tcx_mdct_window[i].v.im);
                move16();
            }
            FOR ( ; i < overlap; i++)
            {
                xn_buf[i] = mult_r(xn_buf[i], st->tcx_cfg.tcx_mdct_window[overlap-1-i].v.re);
                move16();
            }
        }

        FOR (i = 0; i < tmp16; i++)
        {
            xn_buf[L_frame+i] = mult_r(xn_buf[L_frame+i], st->tcx_cfg.tcx_mdct_window[i].v.re);
            move16();
        }
        FOR ( ; i < overlap; i++)
        {
            xn_buf[L_frame+i] = mult_r(xn_buf[L_frame+i], st->tcx_cfg.tcx_mdct_window[overlap-1-i].v.im);
            move16();
        }

        x_e = 16;
        move16();
        TCX_MDCT(xn_buf, x, &x_e, overlap, sub(L_frame, overlap), overlap);
        tmp16 = mult_r(shl(L_frame, 5), FL2WORD16(16*0.0559017)); /* L_frame / sqrt(2*NORM_MDCT_FACTOR); Q9 */
        FOR (i = 0; i < L_frame; i++)
        {
            x[i] = Mpy_32_16_1(x[i], tmp16);
            move32();
        }
        x_e = add(x_e, 6);

        weight_a_fx(A_q_tcx, Ap, st->gamma, M);

        lpc2mdct(Ap, M, gainlpc, gainlpc_e, NULL, NULL);

        mdct_shaping(x, L_frame, gainlpc, gainlpc_e);

        IF ( st->narrowBand )
        {
            j = mult( L_frame, 20480 );
            set32_fx(&x[j], 0, sub(L_frame, j));
        }

        FOR (i = 0; i < L_frame_4; i++)
        {
            /* normalization */
            s = 31;
            move16();

            tmp16 = norm_l(x[0]);
            if (x[0] != 0) s = s_min(s, tmp16);

            tmp16 = norm_l(x[1]);
            if (x[1] != 0) s = s_min(s, tmp16);

            tmp16 = norm_l(x[2]);
            if (x[2] != 0) s = s_min(s, tmp16);

            tmp16 = norm_l(x[3]);
            if (x[3] != 0) s = s_min(s, tmp16);

            s = sub(s, 2);  /* 2 bits headroom */

            /* calc quadruple energy */
            ener = L_deposit_l(1);

            tmp16 = extract_h(L_shl(x[0], s));
            ener = L_mac(ener, tmp16, tmp16);

            tmp16 = extract_h(L_shl(x[1], s));
            ener = L_mac(ener, tmp16, tmp16);

            tmp16 = extract_h(L_shl(x[2], s));
            ener = L_mac(ener, tmp16, tmp16);

            tmp16 = extract_h(L_shl(x[3], s));
            ener = L_mac(ener, tmp16, tmp16);

            s = shl(sub(x_e, s), 1);

            tmp32 = L_add(BASOP_Util_Log2(ener), L_shl(L_deposit_l(s), 25)); /* log2, 6Q25 */
            tmp32 = L_shr(tmp32, 9); /* 15Q16 */
            en[i] = L_add(tmp32, 0x2FD5F); /* 0x2FD5F -> 9.f * log2(10)/10 (15Q16) */                       move32();

            x += 4;
        }

        fac = L_add(0x2A854B, 0); /* 0x2A854B -> 128.f * log2(10)/10 (15Q16) */
        offset = L_add(fac, 0);

        FOR (iter = 0; iter < 10; iter++)
        {
            fac = L_shr(fac, 1);
            offset = L_sub(offset, fac);
            ener = L_deposit_l(0);

            assert(L_frame_4 % 4 == 0);
            FOR (i=0; i < L_frame_4; i+=4)
            {
                tmp32 = L_sub(en[i], offset);

                if (L_sub(tmp32, 0xFF20) > 0) /* 0xFF20 -> 3.f * log2(10)/10 */
                {
                    ener = L_add(ener, tmp32);
                }

                tmp32 = L_sub(en[i+1], offset);

                if (L_sub(tmp32, 0xFF20) > 0) /* 0xFF20 -> 3.f * log2(10)/10 */
                {
                    ener = L_add(ener, tmp32);
                }

                tmp32 = L_sub(en[i+2], offset);

                if (L_sub(tmp32, 0xFF20) > 0) /* 0xFF20 -> 3.f * log2(10)/10 */
                {
                    ener = L_add(ener, tmp32);
                }

                tmp32 = L_sub(en[i+3], offset);

                if (L_sub(tmp32, 0xFF20) > 0) /* 0xFF20 -> 3.f * log2(10)/10 */
                {
                    ener = L_add(ener, tmp32);
                }

                IF (L_sub(ener, target) > 0)
                {
                    offset = L_add(offset, fac);
                    BREAK;
                }
            }
        }

        if (L_sub(offset, 0xAA153) <= 0) /* 0xAA153 -> 32.f * log2(10)/10 */
        {
            offset = L_add(0xFFD57AB5, 0); /* 0xFFD57AB5 -> -128.f * log2(10)/10; */
        }

        s = add(extract_h(offset), 1);
        offset = L_sub(L_and(offset, 0xFFFF), 0x10000);
        ener = BASOP_Util_InvLog2(L_shl(offset, 9));

        ener = Mpy_32_16_1(Mpy_32_16_1(ener, 0x78AE), getInvFrameLen(L_frame)); /* 0x78AE -> sqrt(2)/12 (Q18) */
        ener_e = sub(s, 9);

        tcx_snr = L_deposit_l(0);

        FOR (i = 0; i < st->L_frame_fx; i += L_SUBFR)
        {
            tmp32 = L_deposit_l(0);

            FOR (j = 0; j < L_SUBFR; j++)
            {
                tmp32 = L_mac0(tmp32, st->wspeech_enc[i+j], st->wspeech_enc[i+j]);
            }
            tmp32 = L_shr(BASOP_Util_Log2(tmp32), 9); /* 15Q16 */
            tmp32 = L_add(tmp32, L_sub( 0x1F0000, L_shl( L_deposit_h( add( Q_new, sub( shift, 1 ) ) ), 1 ) ) ); /* wspeech_enc scaling */
            if (L_sub(tmp32, 0xFFEC1185) < 0)
            {
                tmp32 = L_add(0, 0xFFEC1185); /* 0xFFEC1185 -> log2(1e-6) in 15Q16 */
            }

            tcx_snr = L_add(tcx_snr, tmp32);
        }
        tcx_snr = Mpy_32_16_1(tcx_snr, div_s(L_SUBFR, st->L_frame_fx));
        tcx_snr = L_sub(tcx_snr, L_shr(BASOP_Util_Log2(Mpy_32_16_1(ener, L_SUBFR)), 9));
        tcx_snr = L_sub(tcx_snr, L_deposit_h(add(ener_e, 15)));
        tcx_snr = L_shl(Mpy_32_16_1(tcx_snr, 0x6054), 2); /* 0x6054 -> 10/log2(10) (2Q13) */

        BASOP_SATURATE_WARNING_OFF
        snr_tcx = round_fx(L_shl(tcx_snr, 8)); /* 7Q8 */
        BASOP_SATURATE_WARNING_ON

        /*--------------------------------------------------------------*
        * Estimate ACELP SNR
        *---------------------------------------------------------------*/

        tmp32 = L_deposit_l(0);

        /*
          snr_acelp = 1/nSubFrames + sum( 10*log10( signal / (noise*0.055) )
          snr_acelp = sum( (log2(signal) - log2(noise)) * 10/log2(10) )/nSubFrames - 10*log10(0.055)
         */

        i2 = 0;
        move16();
        FOR (i = 0; i < st->L_frame_fx; i += L_SUBFR)
        {
            IF ( L_sub( st->sr_core, 16000 ) == 0 )
            {
                T0 = shr(add( add(pitch_fr[mult_r(i2,FL2WORD16((float)L_FRAME/(float)L_FRAME16k))], shr(pitch_fr[mult_r(i2,FL2WORD16((float)L_FRAME/(float)L_FRAME16k))], 2)) , (1 << 5) ), 6);
            }
            ELSE
            {
                T0 = shr(add( pitch_fr[i2] , (1 << 5) ), 6);
            }

            gain = get_gain( st->wspeech_enc + i, st->wspeech_enc + sub(i, T0), L_SUBFR );

            signal = L_deposit_l(1);
            noise = L_deposit_l(1);

            FOR (j = 0; j < L_SUBFR; j++)
            {
                signal = L_mac0(signal, st->wspeech_enc[i+j], st->wspeech_enc[i+j]);

                tmp16 = round_fx(L_shl(Mpy_32_16_r(gain, st->wspeech_enc[i+j-T0]), 15));
                tmp16 = sub(st->wspeech_enc[i+j], tmp16);
                noise = L_mac0(noise, tmp16, tmp16);
            }
            /* Assume always 4 sub frames. */
            /*assert( (st->L_frame_fx /  L_SUBFR) == 4);*/
            tmp32 = L_add(tmp32, Mpy_32_16_1(L_sub(BASOP_Util_Log2(signal), BASOP_Util_Log2(noise)), FL2WORD16_SCALE(3.0102999566398119521373889472449, 7-LD_DATA_SCALE+2)));
            i2 = add(i2, 1);
        }

        if(sub(st->L_frame_fx,L_FRAME16k) == 0)
        {
            tmp32 = Mpy_32_16_1(tmp32,FL2WORD16((float)L_FRAME/(float)L_FRAME16k));
        }

        offset = L_add(0, FL2WORD32_SCALE(-12.5963731051575616, 7)); /* 10*log10(0.055f) */
        if (L_sub(st->sr_core, 16000) == 0)
        {
            offset = L_add(0, FL2WORD32_SCALE(-10.362121726544446, 7)); /* 10*log10(0.092f) */
        }
        if (L_sub(st->sr_core, 12800) == 0)
        {
            offset = L_add(0, FL2WORD32_SCALE(-12.291479883578557, 7)); /* 10*log10(0.059f) */
        }
        if (st->narrowBand != 0)
        {
            offset = L_add(0, FL2WORD32_SCALE(-8.2390874094431865, 7)); /* 10*log10(0.15f) */
        }

        tmp32 = L_sub(tmp32, offset);

        snr_acelp = round_fx(tmp32); /* 7Q8 */

        /*--------------------------------------------------------------*
        * Switching Decision
        *---------------------------------------------------------------*/

        dsnr = 0;
        move16();
        /* hysteresis for very small SNR differences between ACELP and TCX */

        /* try to use TCX instead of ACELP on temporally stationary frames */
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        if ((sub(snr_acelp, snr_tcx) > 0) &&
                (sub(snr_acelp, add(snr_tcx, FL2WORD16_SCALE(2.0f, 7))) < 0) &&
                (sub(add(st->prevTempFlatness_fx, currFlatness), FL2WORD16_SCALE(3.25f, AVG_FLAT_E)) < 0 || sub(stab_fac, 0x7fff) == 0 || (L_sub(st->sr_core, 12800) == 0 && sub(sp_aud_decision0,1)==0 && sub(add(st->prevTempFlatness_fx, currFlatness), FL2WORD16_SCALE(20.f, AVG_FLAT_E)) < 0)) &&
                (sub(st->acelpFramesCount, 6) <= 0))
        {
            dsnr = FL2WORD16_SCALE(-2.0f, 7);
            move16();
        }

        /* try to use ACELP instead of TCX on transient and "buzzy" frames */
        test();
        test();
        test();
        if ((sub(snr_acelp, snr_tcx) < 0) &&
                (sub(snr_acelp, sub(snr_tcx, FL2WORD16_SCALE(2.0f, 7))) > 0) &&
                (sub(add(st->prevTempFlatness_fx, currFlatness), FL2WORD16_SCALE(3.25f, AVG_FLAT_E)) > 0) &&
                (sub(st->acelpFramesCount, 6) >= 0))
        {
            dsnr = FL2WORD16_SCALE(2.0f, 7);
            move16();
        }

        IF ( st->flag_noisy_speech_snr )
        {
            test();
            IF ( vad_flag || st->Opt_DTX_ON_fx )
            {
                dsnr = add(dsnr, FL2WORD16_SCALE(2.0f, 7));
            }
            ELSE
            {
                dsnr = sub(dsnr, FL2WORD16_SCALE(2.0f, 7));
            }
        }

        test();
        test();
        test();
        IF  (L_sub(st->sr_core, 12800) == 0 && sub(non_staX,FL2WORD16_SCALE(2.0f, 15-8)) < 0 && (sub(st->last_core_fx,ACELP_CORE)==0 || sub(st->last_core_fx,TCX_20_CORE)==0))
        {
            st->core_fx = st->last_core_fx;
        }
        ELSE
        IF ( sub(add(snr_acelp, dsnr), snr_tcx) > 0 )
        {
            st->core_fx = ACELP_CORE;
            move16();
            st->acelpFramesCount = add(st->acelpFramesCount, 1);
        }
        ELSE
        {
            st->core_fx = TCX_20_CORE;
            move16();
            st->acelpFramesCount = 0;
            move16();
        }


    }

    /* Fixed Decision (using -C) */
    test();
    if ( st->acelpEnabled != 0 && st->tcx20Enabled == 0 )
    {
        st->core_fx = ACELP_CORE;
        move16();
    }

    test();
    if ( st->acelpEnabled == 0 && st->tcx20Enabled != 0 )
    {
        st->core_fx = TCX_20_CORE;
        move16();
    }

    st->prevTempFlatness_fx = currFlatness;
    move16();
}




/*-------------------------------------------------------------------*
 * BITS_ALLOC_ACELP_config_rf()
 *
 * configure channel aware mode
 *-------------------------------------------------------------------*/
void BITS_ALLOC_ACELP_config_rf(const Word16 coder_type,
                                Word16 *tilt_code,
                                Word16 *rf_frame_type,
                                Word16 *rf_target_bits,
                                Word16 nb_subfr
                                , Word16 rf_fec_indicator
                                , Word16 *pitch_buf
                               )
{
    Word16 mean_tc, min_tilt_code, max_tilt_code;
    Word16 nrgMode, ltfMode, ltpMode, gainsMode;
    Word32 L_tmp;

    Word16 en_partial_red = 1;
    Word16 dpit1, dpit2, dpit3;

    /* Init */
    *rf_target_bits = 0;
    move16();

    /* ----------------------------------------*
     * RF frame type selection                 *
     *-----------------------------------------*/
    /* Mean tilt code estimation */
    mean_tc = 0;
    move16();
    /*mean_tc = mean_fx(tilt_code, nb_subfr);*/
    IF( sub(nb_subfr, 4) == 0 )
    {
        /* subframe 4 case */
        L_tmp = L_mult(tilt_code[0], 8192);
        L_tmp = L_mac(L_tmp, tilt_code[1], 8192 );
        L_tmp = L_mac(L_tmp, tilt_code[2], 8192 );
        mean_tc = mac_r(L_tmp, tilt_code[3], 8192 );  /* mean_tc in Q15 */
    }
    ELSE
    {
        /* subframe 5 case */
        L_tmp = L_mult(tilt_code[0], 6554);
        L_tmp = L_mac(L_tmp, tilt_code[1], 6554 );
        L_tmp = L_mac(L_tmp, tilt_code[2], 6554 );
        L_tmp = L_mac(L_tmp, tilt_code[3], 6554 );
        mean_tc = mac_r(L_tmp, tilt_code[4], 6554 );  /* mean_tc in Q15 */
    }

    /* Maximum tilt code estimation */
    max_tilt_code = tilt_code[0];
    move16();
    maximum_fx(tilt_code, nb_subfr, &max_tilt_code);

    /* Minimum tilt code estimation */
    min_tilt_code=tilt_code[0];
    move16();
    minimum_fx(tilt_code, nb_subfr, &min_tilt_code);

    /* Estimate the frame's criticality and decide
       whether to transmit partial redundancy information */
    dpit1 = abs_s( sub(pitch_buf[1], pitch_buf[0]));
    dpit2 = abs_s( sub(pitch_buf[2], pitch_buf[1]));
    dpit3 = abs_s( sub(pitch_buf[3], pitch_buf[2]));

    IF ( sub( rf_fec_indicator, 1 ) == 0 )
    {
        test();
        test();
        test();
        test();
        IF  ( sub( max_tilt_code, FL2WORD16(0.48f) ) > 0  && sub( dpit1, 0 )  <= 0 && sub( dpit2, 0 )  <= 0 && sub( dpit3, 0 )  <= 0 && sub(coder_type, VOICED ) == 0 )
        {
            en_partial_red = 0;
            move16();
        }
        ELSE IF  ( sub( max_tilt_code, FL2WORD16(0.47f) ) > 0  && sub( dpit1, 64 )  <= 0 && sub( dpit2, 64 )  <= 0 && sub( dpit3, 64 )  <= 0 && sub(coder_type, GENERIC ) == 0 )
        {
            en_partial_red = 0;
            move16();
        }
    }
    ELSE
    {
        test();
        test();
        test();
        test();
        IF  ( sub( max_tilt_code, FL2WORD16(0.47f) )> 0  && sub( dpit1, 16 )  <= 0 && sub( dpit2, 16 )  <= 0 && sub( dpit3, 16 )  <= 0 && sub(coder_type, VOICED ) == 0 )
        {
            en_partial_red = 0;
            move16();
        }
        ELSE IF ( sub( max_tilt_code, FL2WORD16(0.45f) ) > 0  && sub( dpit1, 80 )  <= 0 && sub( dpit2, 80 )  <= 0 && sub( dpit3, 80 )  <= 0 && sub(coder_type, GENERIC ) == 0 )
        {
            en_partial_red = 0;
            move16();
        }
    }

    /* ---------------------------------------------------------*
     * Identify number of bits required as per rf frame type    *
     * ---------------------------------------------------------*/

    /* rf_mode, 1 bit */
    *rf_target_bits = add(*rf_target_bits,1);

    /* rf_fec_offset 2 bits */
    *rf_target_bits = add(*rf_target_bits,2);

    /* rf_frame_type, 3 bits */
    *rf_target_bits = add(*rf_target_bits,3);

    /* LSF bits 8 + 8 bits */
    *rf_target_bits = add(*rf_target_bits,16);

    /* Intialize the RF mode frame type to all-pred */
    *rf_frame_type = RF_ALLPRED;

    test();
    IF( sub(coder_type,INACTIVE) == 0 ||  en_partial_red == 0)
    {
        *rf_frame_type = RF_NO_DATA;
    }
    ELSE IF ( sub(coder_type,UNVOICED) == 0 || sub(coder_type,INACTIVE) == 0)
    {
        *rf_frame_type = RF_NELP;
    }
    ELSE IF( sub(coder_type,GENERIC) == 0 && sub(max_tilt_code, FL2WORD16(0.05f)) <0 )
    {
        *rf_frame_type = RF_NOPRED;
    }
    ELSE IF( sub(coder_type,GENERIC) == 0 && sub(mean_tc,FL2WORD16(0.3f)) < 0)
    {
        *rf_frame_type = RF_GENPRED;
    }

    nrgMode = ACELP_NRG_MODE[1][1][*rf_frame_type];
    ltfMode = ACELP_LTF_MODE[1][1][*rf_frame_type];
    ltpMode = ACELP_LTP_MODE[1][1][*rf_frame_type];
    gainsMode = ACELP_GAINS_MODE[1][1][*rf_frame_type];

    /* Number of RF bits for different RF coder types */
    SWITCH (*rf_frame_type)
    {
    case RF_ALLPRED:
        /* Es_pred bits 3 bits, LTF: 1, pitch: 8,5,5,5, FCB: 0, gain: 7,0,7,0, Diff GFr: 4*/
        *rf_target_bits += (  ACELP_NRG_BITS[nrgMode]
                              + ACELP_LTF_BITS[ltfMode]
                              + ACELP_LTP_BITS_SFR[ltpMode][0] + ACELP_LTP_BITS_SFR[ltpMode][1] + ACELP_LTP_BITS_SFR[ltpMode][2] + ACELP_LTP_BITS_SFR[ltpMode][3]
                              + ACELP_GAINS_BITS[gainsMode] + ACELP_GAINS_BITS[gainsMode]
                              + 2 /*2 bits for PartialCopy GainFrame*/
                           );
        BREAK;

    case RF_NOPRED:
        /* Es_pred bits 3 bits, LTF: 0, pitch: 0, FCB: 7,7,7,7, gain: 6,0,6,0, Diff GFr: 2*/
        /*bits += (3 + 0 + 0 + 28 + 12 + 2); *//* 64 rf bits */
        *rf_target_bits += (  ACELP_NRG_BITS[nrgMode]
                              + ACELP_LTF_BITS[ltfMode]
                              + 28
                              + ACELP_GAINS_BITS[gainsMode] + ACELP_GAINS_BITS[gainsMode]
                              + 2 /*2 bits for PartialCopy GainFrame*/
                           );
        BREAK;

    case RF_GENPRED:
        /* Es_pred bits 3 bits, LTF: 0, pitch: 8,0,8,0, FCB: 6,7,5,5, gain: 5,0,5,0, Diff GFr: 0*/
        /*bits += (3 + 0 + 16 + 23 + 10 + 0);  */   /* 72 rf bits */
        *rf_target_bits += (  ACELP_NRG_BITS[nrgMode]
                              + ACELP_LTF_BITS[ltfMode]
                              + ACELP_LTP_BITS_SFR[ltpMode][0] + ACELP_LTP_BITS_SFR[ltpMode][1] + ACELP_LTP_BITS_SFR[ltpMode][2] + ACELP_LTP_BITS_SFR[ltpMode][3]
                              + 14
                              + ACELP_GAINS_BITS[gainsMode] + ACELP_GAINS_BITS[gainsMode]
                              + 2 /*2 bits for PartialCopy GainFrame*/
                           );
        BREAK;

    case RF_NELP:
        /* gain: 19, Diff GFr: 5 */
        /*bits += (19 + 5);    */
        *rf_target_bits +=  (19 + NUM_BITS_SHB_FRAMEGAIN);
        BREAK;

    case RF_NO_DATA:
        *rf_target_bits  = 6;
        BREAK;

    default:
        assert(!"RF_Frame_type does not belong to ACELP Partial copy frame types possible!");
        break;
    }

    return;

}

/*-------------------------------------------------------------------*
  * BITS_ALLOC_TCX_config_rf()
  *
  * configure channel aware mode
  *-------------------------------------------------------------------*/
void BITS_ALLOC_TCX_config_rf(
    Word16 *rf_frame_type,
    Word16 *rf_target_bits,
    Word16 PLC_Mode,
    Word16 coder_type,
    Word16 last_core,
    Word16 TD_mode
)
{
    Word16 bits;

    /* Init: rf_mode + rf_fec_offset + rf_frame_type */
    bits = 1 + 2 + 3;
    move16();

    test();
    IF( sub(coder_type, INACTIVE) == 0 || sub(last_core, ACELP_CORE) == 0 )
    {
        *rf_frame_type = RF_NO_DATA;
        move16();
    }
    ELSE
    {
        /* classification */
        bits = add(bits, 2);

        IF( PLC_Mode != 0 )
        {
            /* TCX global gain  = 7 bits */
            bits = add(bits, 7);
            *rf_frame_type = RF_TCXFD;
            move16();
        }
        ELSE
        {
            /* pitch and gain */
            /* LTP data */
            IF( TD_mode != 0)
            {
                bits = add(bits, 9);
                *rf_frame_type = RF_TCXTD2;
                move16();
            }
            ELSE
            {
                bits = add(bits, 9);
                *rf_frame_type = RF_TCXTD1;
                move16();
            }
        }

        if( sub(*rf_frame_type, RF_TCXFD) == 0 )
        {
            /* TCXFD: LSF bits 5 + 4 + 4 bits     */
            /* only embed LSF for FD concealment */
            bits = add(bits, TCXLPC_NUMBITS);
        }
    }

    *rf_target_bits = bits;
    move16();

    return;
}






