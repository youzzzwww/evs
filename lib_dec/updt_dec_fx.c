/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "prot_fx.h"    /* Function prototypes                    */
#include <assert.h>     /* Debug prototypes                       */
#include "stl.h"

/*-------------------------------------------------------------------*
 * updt_dec()
 *
 * Common updates (all frame types)
 *-------------------------------------------------------------------*/
void updt_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: state structure                          */
    const Word16 L_frame,           /* i  : length of the frame                      */
    const Word16 coder_type,        /* i  : coding type                              */
    const Word16 *old_exc_fx,       /* i  : buffer of excitation                     */
    const Word16 *pitch_buf_fx,     /* i  : floating pitch values for each subframe  */
    const Word16 Es_pred,           /* i  : predicited scaled innovation energy      */
    const Word16 *Aq,               /* i  : A(z) quantized for all subframes         */
    const Word16 *lsf_new_fx,       /* i  : current frame LSF vector                 */
    const Word16 *lsp_new_fx,       /* i  : current frame LSP vector                 */
    const Word16 voice_factors[],   /* i  : voicing factors                          */
    const Word16 *old_bwe_exc_fx,   /* i  : buffer of excitation                     */
    const Word16 *gain_buf          /* i  : Q14*/
)
{
    Word16 i, len;

    /* update old excitation buffer */
    Copy( &old_exc_fx[L_frame], st_fx->old_exc_fx, L_EXC_MEM_DEC );
    IF( !st_fx->Opt_AMR_WB_fx )
    {
        Copy( &old_bwe_exc_fx[L_FRAME32k], st_fx->old_bwe_exc_fx, PIT16k_MAX * 2 );
    }

    /* update old LSP and LSF vector */
    Copy( lsf_new_fx, st_fx->lsf_old_fx, M );
    Copy( lsp_new_fx, st_fx->lsp_old_fx, M );

    /* update last coding type */
    st_fx->last_coder_type_fx = coder_type;
    test();
    test();
    test();
    if( sub(coder_type,INACTIVE) == 0  || (sub(st_fx->bpf_off_fx,1) == 0 && sub(coder_type,AUDIO) != 0 && sub(coder_type,TRANSITION) != 0) )
    {
        st_fx->last_coder_type_fx = UNVOICED;
        move16();
    }
    test();
    test();
    if( (sub(coder_type,AUDIO) != 0 || st_fx->Last_GSC_noisy_speech_flag_fx != 0) &&  st_fx->Last_GSC_pit_band_idx_fx > 0 )
    {
        st_fx->Last_GSC_pit_band_idx_fx = 0;
        move16();   /*The temporal contribution of the GSC is meaningless after 1 frame lost for inactive & unvoiced content */
    }
    /* this ensures that st_fx->last_coder_type_fx is never set to INACTIVE in case of AVQ inactive because the FEC does not distinguish between GSC inactive and AVQ inactive */

    test();
    if( L_sub(st_fx->total_brate_fx,ACELP_24k40) > 0 && sub(coder_type,INACTIVE) == 0 )
    {
        st_fx->last_coder_type_fx = GENERIC;
        move16();
    }
    test();
    test();
    test();
    IF( sub(st_fx->Opt_AMR_WB_fx,1) == 0 && sub(coder_type,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,SID_1k75) != 0 && L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) != 0 )
    {
        /* overwrite previous coding type to help FEC */
        st_fx->last_coder_type_fx = UNVOICED;
        move16();
        st_fx->last_voice_factor_fx = voice_factors[NB_SUBFR-1];
        move16();
    }

    IF( !st_fx->Opt_AMR_WB_fx )
    {
        /* update voicing factor of TBE to help FEC */

        st_fx->last_voice_factor_fx = voice_factors[NB_SUBFR16k-1];
        move16();
        if( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            st_fx->last_voice_factor_fx = voice_factors[NB_SUBFR-1];
            move16();
        }
    }

    test();
    IF ( coder_type != AUDIO && coder_type != INACTIVE )
    {
        st_fx->noise_lev_fx = NOISE_LEVEL_SP3;
        move16();
        set16_fx( st_fx->old_y_gain_fx, 0, MBANDS_GN );

        FOR( i = 0; i < L_FRAME; i++ )
        {
            Word16 tmp_seed = st_fx->seed_tcx_fx;
            move16();
            st_fx->Last_GSC_spectrum_fx[i] = shr_r(Random( &tmp_seed ),5);
            move16(); /*Q10*/
        }
    }

    /* update last GSC SWB speech flag for FEC */
    st_fx->Last_GSC_noisy_speech_flag_fx = st_fx->GSC_noisy_speech_fx;
    move16();

    /* update counter for FEC pitch estimate */
    st_fx->upd_cnt_fx = add(st_fx->upd_cnt_fx,1);

    st_fx->upd_cnt_fx = s_min(st_fx->upd_cnt_fx, MAX_UPD_CNT);

    len = shr(L_frame,6);
    Copy32( &st_fx->old_pitch_buf_fx[len], st_fx->old_pitch_buf_fx, len );
    FOR ( i = 0; i < len; i++ )
    {
        st_fx->old_pitch_buf_fx[len+i] = L_mult0(pitch_buf_fx[i], 1<<10);
        move32();
    }
    Copy( &st_fx->mem_pitch_gain[2], &st_fx->mem_pitch_gain[L_frame/L_SUBFR+2], L_frame/L_SUBFR );
    IF (sub(L_frame , L_FRAME) == 0)
    {
        st_fx->mem_pitch_gain[2] = gain_buf[3];
        move16();
        st_fx->mem_pitch_gain[3] = gain_buf[2];
        move16();
        st_fx->mem_pitch_gain[4] = gain_buf[1];
        move16();
        st_fx->mem_pitch_gain[5] = gain_buf[0];
        move16();
    }
    ELSE
    {
        st_fx->mem_pitch_gain[2] = gain_buf[4];
        move16();
        st_fx->mem_pitch_gain[3] = gain_buf[3];
        move16();
        st_fx->mem_pitch_gain[4] = gain_buf[2];
        move16();
        st_fx->mem_pitch_gain[5] = gain_buf[1];
        move16();
        st_fx->mem_pitch_gain[6] = gain_buf[0];
        move16();
    }


    /* FEC - update adaptive LSF mean vector */
    Copy( st_fx->lsfoldbfi0_fx, st_fx->lsfoldbfi1_fx, M );
    Copy( lsf_new_fx, st_fx->lsfoldbfi0_fx, M );

    /* update of pitch and voicing information for HQ FEC */
    IF ( sub(st_fx->last_core_fx,HQ_CORE) != 0 )
    {
        st_fx->HqVoicing_fx = 1;
        move16();
        test();
        if( !st_fx->Opt_AMR_WB_fx && sub(coder_type,UNVOICED) == 0 )
        {
            st_fx->HqVoicing_fx = 0;
            move16();
        }
    }

    /* SC-VBR */
    st_fx->old_ppp_mode_fx = st_fx->last_ppp_mode_dec_fx;
    move16();
    st_fx->last_ppp_mode_dec_fx = st_fx->ppp_mode_dec_fx;
    move16();
    st_fx->last_nelp_mode_dec_fx = st_fx->nelp_mode_dec_fx;
    move16();
    st_fx->last_vbr_hw_BWE_disable_dec_fx = st_fx->vbr_hw_BWE_disable_dec_fx;
    move16();

    /*core switching updates*/
    Copy( &Aq[(st_fx->L_frame_fx/L_SUBFR-1)*(M+1)], st_fx->old_Aq_12_8_fx, M+1 );
    st_fx->old_Es_pred_fx = Es_pred;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * updt_IO_switch()
 *
 * Common updates for AMR-WB IO mode and EVS primary switching
 *-------------------------------------------------------------------*/
void updt_IO_switch_dec_fx(
    const Word16 output_frame,   /* i  : output frame length         */
    Decoder_State_fx *st_fx           /* o  : Decoder static variables structure */
)
{
    Word16 xsp_tmp[M];
    IF( sub(st_fx->last_core_fx,AMR_WB_CORE) == 0 )      /* switching to EVS primary mode */
    {
        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode mode LSF(LSP) */
        Copy( stable_LSP_fx, xsp_tmp, M );
        isf2lsf_fx( st_fx->lsf_old_fx, st_fx->lsf_old_fx, xsp_tmp);
        Copy( stable_LSP_fx, xsp_tmp, M );
        /*isp2lsp( st->lsp_old, st->lsp_old, xsp_tmp, M, grid100 );*/
        isp2lsp_fx( st_fx->lsp_old_fx, st_fx->lsp_old_fx, xsp_tmp, M);

        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode uses LSF(LSP) */
        Copy( stable_LSP_fx, xsp_tmp, M );
        /*isp2lsp( st->lspCNG, st->lspCNG, xsp_tmp, M, grid100 );*/
        isp2lsp_fx( st_fx->lspCNG_fx, st_fx->lspCNG_fx, xsp_tmp, M);

        st_fx->old_enr_index_fx = s_min(shl(st_fx->old_enr_index_fx,1), 127 );

        /* reset TD BWE buffers */
        set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX * 2 );
        set16_fx( st_fx->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );
        st_fx->bwe_non_lin_prev_scale_fx = 0;
        move16();
        st_fx->last_voice_factor_fx = 0;
        move16();

        wb_tbe_extras_reset_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx );
        wb_tbe_extras_reset_synth_fx( st_fx->state_lsyn_filt_shb_fx, st_fx->state_lsyn_filt_dwn_shb_fx, st_fx->state_32and48k_WB_upsample_fx
                                      ,st_fx->mem_resamp_HB_fx
                                    );

        IF(  sub(output_frame,L_FRAME32k) >= 0 )
        {
            swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                              st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &st_fx->tbe_demph_fx, &st_fx->tbe_premph_fx
                              ,st_fx->mem_stp_swb_fx,&(st_fx->gain_prec_swb_fx) );

            swb_tbe_reset_synth_fx( st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx );
        }

        IF( sub(output_frame,L_FRAME48k) == 0 )
        {
            st_fx->prev_fb_ener_adjust_fx = 0;
            move16();
            fb_tbe_reset_synth_fx( st_fx->fbbwe_hpf_mem_fx, &st_fx->prev_fbbwe_ratio_fx );
        }
        st_fx->prev_Energy_wb_fx = L_deposit_l(0);
        st_fx->prev_Energy_fx = 0;
        move16();
        st_fx->prev_td_energy_fx = 0;
        move16();
        st_fx->prev_weight_fx = 6554;
        move16();

        /* reset FD BWE buffers */
        st_fx->prev_mode_fx = NORMAL;
        move16();
        st_fx->prev_Energy_fx = 0;
        move16();
        st_fx->prev_L_swb_norm_fx = 8;
        move16();
        st_fx->prev_frica_flag_fx = 0;
        move16();
        set16_fx( st_fx->mem_imdct_fx, 0, L_FRAME48k );
        st_fx->prev_td_energy_fx = 0;
        move16();
        st_fx->prev_weight_fx = 0;
        move16();
        set16_fx(st_fx->L_old_wtda_swb_fx, 0, L_FRAME48k );
        /* HQ core buffers */
        set16_fx( st_fx->delay_buf_out_fx, 0, HQ_DELTA_MAX*HQ_DELAY_COMP );

        /* reset the unvoiced/audio signal improvement  memories */
        st_fx->seed_tcx_fx = 15687;
        move16();
        st_fx->UV_cnt_fx = 30;
        move16();
        st_fx->LT_UV_cnt_fx = (60<<6);
        move16();

        st_fx->use_acelp_preq = 0;
        move16();
    }
    ELSE                                    /* switching to AMR-WB IO mode */
    {
        /* ISF Q memories */
        set16_fx( st_fx->mem_MA_fx, 0, M );

        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode LSF(LSP) */
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsf2isf_fx( st_fx->lsf_old_fx, st_fx->lsf_old_fx, xsp_tmp, M);
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsp2isp_fx( st_fx->lsp_old_fx, st_fx->lsp_old_fx, xsp_tmp, M);

        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode LSF(LSP) */
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsp2isp_fx( st_fx->lspCNG_fx, st_fx->lspCNG_fx, xsp_tmp, M);

        st_fx->old_enr_index_fx = s_max(s_min(shr(st_fx->old_enr_index_fx, 1), 63 ),0);


        /* gain quantization memory */
        set16_fx( st_fx->past_qua_en_fx, -14<<10, GAIN_PRED_ORDER );

        /* HF synthesis memories */
        st_fx->Ng_ener_ST_fx = -51<<8;
        move16();

        hf_synth_amr_wb_reset_fx( &st_fx->seed2_fx, st_fx->mem_syn_hf_fx, st_fx->mem_hp_interp_fx,
        &st_fx->prev_r_fx, &st_fx->fmerit_w_sm_fx, st_fx->delay_syn_hf_fx, &st_fx->frame_count_fx,
        &st_fx->ne_min_fx, &st_fx->fmerit_m_sm_fx, &st_fx->voice_fac_amr_wb_hf,
        &st_fx->unvoicing_fx, &st_fx->unvoicing_sm_fx, &st_fx->unvoicing_flag_fx,
        &st_fx->voicing_flag_fx, &st_fx->start_band_old_fx, &st_fx->OptCrit_old_fx );

        /* reset the unvoiced/audio signal improvement memories */
        st_fx->seed_tcx_fx = 15687;
        move16();
        st_fx->UV_cnt_fx = 30;
        move16();
        st_fx->LT_UV_cnt_fx = 60<<6;
        move16();
        st_fx->Last_ener_fx = 0;
        move16();
        st_fx->lt_voice_fac_fx = 0;
        move16();
    }

    /* CNG - reset */
    st_fx->ho_hist_size_fx = 0;
    move16();

    /* LSF Q memories */
    Copy( UVWB_Ave_fx, st_fx->mem_AR_fx, M );

    /* FEC - update adaptive LSF mean vector */
    Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi0_fx, M );
    Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi1_fx, M );
    Copy( st_fx->lsf_old_fx, st_fx->lsf_adaptive_mean_fx, M );

    return;
}

/*-------------------------------------------------------------------*
 * updt_bw_switching()
 *
 * Updates for BW switching
 *-------------------------------------------------------------------*/

void updt_bw_switching_fx(
    Decoder_State_fx *st_fx,              /* i/o: decoder state structure                  */
    const Word16 *synth,                /* i  : float synthesis signal                   */
    const Word16 *inner_frame_tbl,      /* i  : HQ inner_frame signallisation table      */
    const Word16 Qpost
)
{
    test();
    IF(st_fx->output_Fs_fx == 32000 && st_fx->bwidth_fx == SWB)
    {
        st_fx->tilt_swb_fx = round_fx(L_shl(calc_tilt_bwe_fx(synth, Qpost, L_FRAME32k), 3));
    }

    st_fx->prev_enerLH_fx = st_fx->enerLH_fx;
    move32();
    st_fx->prev_enerLL_fx = st_fx->enerLL_fx;
    move32();
    st_fx->last_bwidth_fx = st_fx->bwidth_fx;
    move32();

    IF( sub(st_fx->core_fx, ACELP_CORE) == 0 )
    {
        st_fx->last_inner_frame_fx = L_FRAME32k;
        move16();
        test();
        if( sub(st_fx->bwidth_fx, WB) == 0 && st_fx->bws_cnt_fx == 0 )
        {
            st_fx->last_inner_frame_fx = L_FRAME16k;
            move16();
        }

        st_fx->prev_weight1_fx = 16384;
        move16();
        if(sub(st_fx->prev_mode_fx, HARMONIC) == 0)
        {
            st_fx->prev_weight1_fx = 6554;
            move16();
        }
    }
    ELSE
    {
        test();
        test();
        test();
        IF( !(sub(st_fx->last_inner_frame_fx, L_FRAME16k) >= 0 && sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME16k) <= 0 && st_fx->bws_cnt_fx > 0 && sub(st_fx->bws_cnt_fx, N_WS2N_FRAMES) < 0) )
        {
            st_fx->last_inner_frame_fx = inner_frame_tbl[st_fx->bwidth_fx];
            move16();
        }

        test();
        IF(sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME32k) >= 0 || L_sub(st_fx->core_brate_fx, HQ_16k40) <= 0)
        {
            st_fx->prev_weight1_fx = 16384;
            move16();
            test();
            if(sub(st_fx->prev_hqswb_clas_fx, HQ_HARMONIC) == 0 || sub(st_fx->prev_hqswb_clas_fx, HQ_HVQ) == 0)
            {
                st_fx->prev_weight1_fx = 6554;
                move16();
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * updt_dec_common()
 *
 * Common updates for MODE1 and MODE2
 *-------------------------------------------------------------------*/

void updt_dec_common_fx(
    Decoder_State_fx *st_fx,       /* i/o: decoder state structure     */
    Word16 hq_core_type_fx,      /* i  : HQ core type                */
    const Word16 *synth          /* i  : decoded synthesis           */
)
{

    st_fx->last_codec_mode = st_fx->codec_mode;
    move16();
    st_fx->last_extl_fx = st_fx->extl_fx;
    move16();
    st_fx->last_L_frame_fx = st_fx->L_frame_fx;
    move16();

    st_fx->prev_old_bfi_fx = st_fx->prev_bfi_fx;
    move16();
    st_fx->prev_bfi_fx = st_fx->bfi_fx;
    move16();
    st_fx->old_bfi_cnt_fx = st_fx->nbLostCmpt;
    move16();

    test();
    if (st_fx->use_partial_copy == 0 && st_fx->bfi_fx == 0)
    {
        st_fx->last_good_fx = st_fx->clas_dec;
        move16();
    }
    IF (st_fx->use_partial_copy)
    {
        st_fx->prev_rf_frame_type = st_fx->rf_frame_type;
    }
    ELSE
    {
        st_fx->prev_rf_frame_type = INACTIVE;
    }

    if (sub(st_fx->m_frame_type, ACTIVE_FRAME) == 0 && (st_fx->bfi_fx != 1 || st_fx->use_partial_copy != 0))
    {
        st_fx->rf_flag_last = st_fx->rf_flag;
    }

    IF( sub(st_fx->codec_mode,MODE1)==0 )
    {
        test();
        if( !st_fx->bfi_fx && st_fx->core_brate_fx > SID_2k40 )
        {
            move16();
            st_fx->last_active_brate_fx = st_fx->total_brate_fx;
        }

        move16();
        move16();
        st_fx->last_core_fx = st_fx->core_fx;
        st_fx->last_hq_core_type_fx = hq_core_type_fx;
    }
    ELSE IF( sub(st_fx->codec_mode,MODE2)==0 )
    {
        test();
        if ((!st_fx->bfi_fx) && (st_fx->last_is_cng==0))
        {
            move16();
            st_fx->last_active_brate_fx = st_fx->total_brate_fx;
        }
        /* INFO: moved from update_decoder_LPD_cng()  */
        if (sub(st_fx->m_frame_type,ACTIVE_FRAME)!=0)
        {
            move16();
            st_fx->last_is_cng = 1;
        }


        if (!st_fx->bfi_fx)
        {
            st_fx->last_core_fx = st_fx->core_fx;
        }
        move16();
        move16();
        st_fx->last_core_bfi = st_fx->core_fx; /* also required for clean channel decoding */
    }
    move16();
    st_fx->last_core_brate_fx = st_fx->core_brate_fx;

    /* save synthesis for core switching */
    /* save synthesis for core switching */
    Copy( synth + NS2SA_fx2(st_fx->output_Fs_fx,ACELP_LOOK_NS+DELAY_BWE_TOTAL_NS), st_fx->old_synth_sw_fx, NS2SA_fx2(st_fx->output_Fs_fx,FRAME_SIZE_NS-ACELP_LOOK_NS-DELAY_BWE_TOTAL_NS));

    test();
    test();
    test();
    IF( ((L_sub(st_fx->core_brate_fx,SID_2k40) <= 0) && sub(st_fx->cng_type_fx, FD_CNG) == 0)
        || (st_fx->tcxonly && sub(st_fx->codec_mode,MODE2)==0)
      )

    {
        /* reset LP memories */
        set16_fx( st_fx->mem_MA_fx,0, M );
        IF( L_sub(st_fx->sr_core,16000) == 0 )
        {
            Copy( GEWB2_Ave_fx, st_fx->mem_AR_fx, M );
        }
        ELSE
        {
            Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
        }
    }
    return;
}


void update_decoder_LPD_cng( Decoder_State_fx *st, Word16 coder_type, Word16 *timeDomainBuffer, Word16 *A, Word16 *bpf_noise_buf)
{
    Word16 i;
    Word16 lsp[M], lsf[M];
    Word16 *synth, synth_buf[M+1+L_FRAME_MAX+L_FRAME_MAX/2], tmp;
    Word16 buf_synth[OLD_SYNTH_SIZE_DEC+L_FRAME_MAX+M];
    Word16 pf_pitch[NB_SUBFR16k];
    Word16 pf_gain[NB_SUBFR16k];
    Word16 Qf_synth;
    Word16 pitch[NB_SUBFR16k];

    Qf_synth = add(0,0);

    /* LPC order */
    move16();

    /* LPC -> LSP */
    E_LPC_a_lsp_conversion( A, lsp, st->lsp_old_fx, M );

    /* LSP -> LSF */
    IF(sub(st->L_frame_fx, L_FRAME16k)== 0)
    {
        lsp2lsf_fx( lsp, lsf, M, INT_FS_16k_FX );
    }
    ELSE
    {
        E_LPC_lsp_lsf_conversion( lsp, lsf, M );
    }


    Copy( st->old_synth, buf_synth, st->old_synth_len );
    Copy( timeDomainBuffer, buf_synth+st->old_synth_len, st->L_frame_fx );

    /* Update synth memory */

    move16();
    synth = synth_buf + (1+M)  ;

    Copy( st->syn, synth_buf, 1+M );
    Copy( timeDomainBuffer, synth, st->L_frame_fx );
    Copy( synth+st->L_frame_fx-(1+M), st->syn, 1+M );
    Copy( st->old_synth+st->L_frame_fx, st->old_synth, st->old_synth_len-st->L_frame_fx );
    Copy( synth, st->old_synth+st->old_synth_len-st->L_frame_fx, st->L_frame_fx );
    Copy( synth+st->L_frame_fx-(st->L_frame_fx/2), st->old_syn_Overl, st->L_frame_fx/2 );

    st->tcxltp_last_gain_unmodified   = 0;

    /* Update pe-synth memory */
    move16();
    tmp = synth[-(1+M)];

    st->Q_syn = E_UTIL_f_preemph3( synth-M, st->preemph_fac, M+st->L_frame_fx, &tmp, 1 );
    st->prev_Q_syn = st->Q_syn = st->Q_syn - 1;
    Copy( synth+st->L_frame_fx-M, st->mem_syn2_fx, M );
    Copy( synth+st->L_frame_fx-L_SYN_MEM, st->mem_syn_r, L_SYN_MEM );

    /* Update excitation memory */
    IF( sub(st->L_frame_fx, L_EXC_MEM_DEC) < 0)
    {
        IF(sub(add(st->Q_syn,1),st->Q_exc) != 0)
        {
            Scale_sig(st->old_exc_fx, L_EXC_MEM_DEC, sub(add(st->Q_syn,1),st->Q_exc));
        }
        st->Q_exc = st->Q_syn + 1;
        Copy( st->old_exc_fx+st->L_frame_fx, st->old_exc_fx, sub(L_EXC_MEM_DEC,st->L_frame_fx) );
        Residu3_fx( A, synth, st->old_exc_fx+L_EXC_MEM_DEC-st->L_frame_fx, st->L_frame_fx, 1 );
    }
    ELSE
    {
        Residu3_fx( A, synth+st->L_frame_fx-L_EXC_MEM_DEC, st->old_exc_fx, L_EXC_MEM_DEC, 1 );
    }

    /* Update LPC-related memories */

    Copy( lsp, st->lsp_old_fx, M );
    Copy( lsf, st->lsf_old_fx, M );
    Copy( lsp, st->lspold_uw, M );
    Copy( lsf, st->lsfold_uw, M );
    move16();
    move16();
    st->envWeighted = 0;
    Copy( A, st->old_Aq_12_8_fx, M+1 );
    st->old_Es_pred_fx=0;

    /* Reset acelp memories */

    move16();
    st->dm_fx.prev_gain_code = L_deposit_l(0);
    set16_fx(st->dm_fx.prev_gain_pit, 0, 6);
    st->dm_fx.prev_state = 0;

    move16();
    move16();
    st->tilt_code_fx = TILT_CODE;
    st->gc_threshold_fx = 0;

    /* Update ace/tcx mode */
    st->core_fx = ACELP_CORE;
    move16();
    move16();
    move16();
    st->last_core_fx = ACELP_CORE;
    st->last_core_bfi = ACELP_CORE;
    st->last_is_cng = 1;
    move16();

    /* Reset TCX overlap */
    move16();
    move16();
    st->tcx_cfg.tcx_curr_overlap_mode = st->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;

    /* For BBWE and Postfilter */

    Copy( A, &(st->mem_Aq[0]), M+1 );
    Copy( A, &(st->mem_Aq[(M+1)]), M+1 );
    Copy( A, &(st->mem_Aq[2*(M+1)]), M+1 );
    Copy( A, &(st->mem_Aq[3*(M+1)]), M+1 );
    IF( sub(st->L_frame_fx, L_FRAME16k) == 0 )
    {
        Copy( A, &(st->mem_Aq[4*(M+1)]), M+1 );
    }

    /* Update for concealment */
    move16();
    move16();
    st->nbLostCmpt = 0;
    st->prev_old_bfi_fx = 0;

    FOR (i=0; i<M; i++)
    {
        move16();
        move16();
        move16();
        st->lsf_adaptive_mean[i] = add(    mult_r(st->lsfoldbfi1[i], FL2WORD16(1.0f/3.0f)),    add( mult_r(st->lsfoldbfi0[i], FL2WORD16(1.0f/3.0f)), mult_r(lsf[i], FL2WORD16(1.0f/3.0f))  ) );
        st->lsfoldbfi1[i] = st->lsfoldbfi0[i];
        st->lsfoldbfi0[i] = lsf[i];
    }

    set16_fx(pitch, shl(L_SUBFR, 6), NB_SUBFR16k);

    FEC_clas_estim_fx(
        st,
        /*Opt_AMR_WB*/0, /*A*/
        st->L_frame_fx,
        &(st->clas_dec),
        coder_type,
        pitch,
        &st->classifier_last_good,
        synth,
        &st->lp_ener_FER_fx,
        /**decision_hyst*/NULL,     /* i/o: hysteresis of the music/speech decision */               /*A*/
        /**UV_cnt*/ NULL,           /* i/o: number of consecutives frames classified as UV */         /*A*/
        /**LT_UV_cnt*/ NULL,        /* i/o: long term consecutives frames classified as UV */         /*A*/
        /**Last_ener*/ NULL,        /* i/o: last_energy frame                              */         /*A*/
        /**locattack*/ NULL,        /* i/o: detection of attack (mainly to localized speech burst) */ /*A*/
        /**lt_diff_etot*/NULL,      /* i/o: long-term total energy variation               */         /*A*/
        /**amr_io_class*/ NULL,     /* i/o: classification for AMR-WB IO mode       */                /*A*/
        /*bitrate*/ 0  ,            /* i  : Decoded bitrate                         */              /*A*/
        &Qf_synth,              /* i  : Synthesis scaling                       */
        /**class_para*/ NULL,       /* o  : classification para. fmerit1            */               /*A*/
        st->mem_syn_clas_estim_fx,  /* i/o: memory of the synthesis signal for frame class estimation */
        &st->classifier_Q_mem_syn, /*i/o : exponent for memory of synthesis signal for frame class estimation */ /*B*/
        st->pit_max,                /* i  : maximum pitch value, Q0                                               *//*B*/
        FL2WORD16(-1.f),            /* i  : LTP Gain                                                              *//*B*/
        0/*CLASSIFIER_ACELP*/,      /* i  : signal classifier mode                                                *//*B*/
        0/*bfi*/,                   /* i  : bad frame indicator                                                   *//*B*/
        add(1,M),                   /* i  : starting ppoint of synthesis buffer                                   *//*B*/
        add(add(M,1),
            add(st->L_frame_fx,
                shr(st->L_frame_fx,1)))/*i  : length of synthesis buffer, relevant for rescaling                    *//*B*/
    );

    /* Postfiltering */
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    pf_pitch[0] = pf_pitch[1] = pf_pitch[2] = pf_pitch[3] = pf_pitch[4] = L_SUBFR;
    pf_gain[0]  = pf_gain[1]  = pf_gain[2]  = pf_gain[3]  = pf_gain[4] = 0;
    st->bpf_gain_param=0;
    move16();

    post_decoder( st, coder_type, buf_synth, pf_gain, pf_pitch, timeDomainBuffer, bpf_noise_buf );

    return;
}

