/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"          /* Compilation switches                   */
#include "prot_fx.h"
#include "cnst_fx.h"          /* Common constants                       */
#include "rom_com_fx.h"       /* Static table prototypes                */
#include "stl.h"              /* required for wmc_tool */
#include "basop_util.h"

/*----------------------------------------------------------------------*
 * init_decoder()
 *
 * Initialization of static variables for the decoder
 *----------------------------------------------------------------------*/

void init_decoder_fx(
    Decoder_State_fx *st_fx  /* o:   Decoder static variables structure */
)
{
    Word16 i, j;
    Word16 f_db, e_db;
    Word32 L_tmp;
    Word16 newCldfbBands;

    /*-----------------------------------------------------------------*
     * ACELP core parameters
     *-----------------------------------------------------------------*/

    st_fx->codec_mode = MODE1;
    move16();
    st_fx->core_fx = ACELP_CORE;
    move16();
    st_fx->L_frame_fx  = L_FRAME;
    move16();
    st_fx->extl_fx  = -1;
    move16();
    st_fx->total_brate_fx = 8000;
    move16();
    st_fx->last_total_brate_fx = -1;
    move16();
    st_fx->core_brate_fx  = 8000;
    move16();
    st_fx->ini_frame_fx = 0;
    move16();
    st_fx->bwidth_fx = NB;
    move16();
    st_fx->extl_brate_fx = 0;
    move16();

    st_fx->last_coder_type_fx  = GENERIC;
    move16();
    st_fx->last_L_frame_fx  = st_fx->L_frame_fx;
    move16();
    st_fx->last_core_brate_fx  = st_fx->core_brate_fx ;
    move16();

    st_fx->last_core_fx = -1;
    move16();
    if ( st_fx->Opt_AMR_WB_fx )
    {
        st_fx->last_core_fx = AMR_WB_CORE;
        move16();
    }
    st_fx->last_extl_fx = st_fx->extl_fx;
    move16();
    st_fx->last_hq_core_type_fx = -1;
    move16();

    /* LSF initilaizations */
    Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );

    init_lvq_fx( st_fx->offset_scale1_fx, st_fx->offset_scale2_fx, st_fx->offset_scale1_p_fx, st_fx->offset_scale2_p_fx, st_fx->no_scales_fx, st_fx->no_scales_p_fx );

    set16_fx( st_fx->mem_MA_fx, 0, M );
    set16_fx( st_fx->mem_syn_hf_fx, 0, M16k );

    st_fx->dm_fx.prev_state = 0;
    move16();            /* This corresponds to st_fx->dispMem in FLP */
    st_fx->dm_fx.prev_gain_code = L_deposit_l(0);
    FOR(i=2; i<8; i++)
    {
        st_fx->dm_fx.prev_gain_pit[i-2] = 0;
        move16();
    }

    /* HF synth init */
    hf_synth_amr_wb_init_fx( &st_fx->prev_r_fx, &st_fx->fmerit_w_sm_fx,
                             &st_fx->frame_count_fx, &st_fx->ne_min_fx, &st_fx->fmerit_m_sm_fx, &st_fx->voice_fac_amr_wb_hf,
                             &st_fx->unvoicing_fx, &st_fx->unvoicing_sm_fx, &st_fx->unvoicing_flag_fx,
                             &st_fx->voicing_flag_fx, &st_fx->start_band_old_fx, &st_fx->OptCrit_old_fx );

    hf_synth_init_fx( st_fx->mem_hp400_fx, st_fx->mem_hf_fx );

    set16_fx( st_fx->mem_hp_interp_fx, 0, 2*L_FILT16k );

    set16_fx( st_fx->delay_syn_hf_fx, 0, NS2SA_fx2(16000,DELAY_CLDFB_NS) );

    st_fx->tilt_code_fx = 0;
    move16();
    st_fx->gc_threshold_fx = L_deposit_l(0);
    st_fx->last_good_fx = UNVOICED_CLAS;
    move16();
    st_fx->clas_dec = UNVOICED_CLAS;
    move16();

    st_fx->lp_gainp_fx = 0;
    move16();
    st_fx->lp_gainc_fx = 0;
    move16();

    set16_fx( st_fx->old_exc_fx, 0, L_EXC_MEM_DEC );

    /* AVQ pre-quantizer memory */
    st_fx->mem_preemp_preQ_fx = 0;
    move16();
    st_fx->last_nq_preQ_fx = 0;
    move16();
    st_fx->use_acelp_preq = 0;
    move16();

    st_fx->mem_deemph_fx = 0;
    move16();

    set16_fx( st_fx->mem_syn1_fx, 0, M );
    st_fx->mem_deemph_old_syn_fx = 0;
    move16();
    set16_fx( st_fx->mem_syn2_fx, 0, M );
    st_fx->stab_fac_fx = 0;
    move16();
    st_fx->stab_fac_smooth_fx = 0;
    move16();
    set16_fx( st_fx->agc_mem_fx, 0, 2 );
    set32_fx( st_fx->L_mem_hp_out_fx, 0, 5 );
    set16_fx( st_fx->mem_syn3_fx, 0, M );
    FOR (i=0; i<GAIN_PRED_ORDER; i++)
    {
        st_fx->past_qua_en_fx[i] = -14336;
        move16();  /* Q10; */  /* gain quantization memory (used also in AMR-WB IO mode) */
    }

    Copy( GEWB_Ave_fx, st_fx->lsf_old_fx, M );
    lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, INT_FS_FX);

    st_fx->mid_lsf_int_fx = 0;
    move16();
    st_fx->safety_net_fx = 0;
    move16();

    /* parameters for AC mode (GSC) */
    st_fx->seed_tcx_fx = 15687;
    move16();   /*check if it is Q0*/
    st_fx->GSC_noisy_speech_fx = 0;
    move16();
    st_fx->Last_GSC_noisy_speech_flag_fx = 0;
    move16();
    st_fx->cor_strong_limit_fx = 1;
    move16();
    set16_fx(st_fx->old_y_gain_fx, 0 , MBANDS_GN );
    st_fx->noise_lev_fx = NOISE_LEVEL_SP0;
    move16();
    set16_fx( st_fx->Last_GSC_spectrum_fx, 0, L_FRAME );
    st_fx->Last_GSC_pit_band_idx_fx = 0;
    move16();

    set16_fx( st_fx->lt_ener_per_band_fx, 4096, MBANDS_GN );/*Q12*/
    set16_fx( st_fx->last_exc_dct_in_fx, 0, L_FRAME);
    st_fx->last_ener_fx = 0;
    set16_fx( st_fx->last_bitallocation_band_fx, 0, MBANDS_GN );

    /* NB post-filter */
    Init_post_filter( &(st_fx->pfstat) );
    st_fx->psf_lp_noise_fx = 0;
    move16();

    /* FEC */
    st_fx->prev_bfi_fx = 0;
    move16();
    st_fx->lp_ener_FER_fx = 15360;
    move16();   /*60 in Q8*/
    st_fx->old_enr_LP = 0;
    move16();
    st_fx->lp_ener_fx = L_deposit_l(0);
    st_fx->enr_old_fx = L_deposit_l(0);
    st_fx->bfi_pitch_fx = L_SUBFR_Q6;
    move16();
    st_fx->bfi_pitch_frame_fx = L_SUBFR;
    move16();
    set16_fx( st_fx->mem_syn_clas_estim_fx, 0, L_SYN_MEM_CLAS_ESTIM );
    st_fx->last_con_tcx = 0;
    move16();

    FOR (i=0; i<2*NB_SUBFR16k; i++)
    {
        st_fx->old_pitch_buf_fx[i] = L_SUBFR<<16;
        move32();    /*15Q16*/
    }

    st_fx->upd_cnt_fx = MAX_UPD_CNT;
    move16();
    Copy( GEWB_Ave_fx, st_fx->lsfoldbfi0_fx, M );
    Copy( GEWB_Ave_fx, st_fx->lsfoldbfi1_fx, M );
    Copy( GEWB_Ave_fx, st_fx->lsf_adaptive_mean_fx, M );
    st_fx->seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->nbLostCmpt = 1;
    move16();
    st_fx->decision_hyst_fx = 0;
    move16();

    /* fast recovery */
    set16_fx( st_fx->old_exc2_fx, 0, L_EXC_MEM );
    set16_fx( st_fx->old_syn2_fx, 0, L_EXC_MEM );

    /* Stationary noise UV modification */
    st_fx->unv_cnt_fx = 0;
    move16();
    st_fx->ge_sm_fx = L_deposit_l(640);   /*Q(GE_SHIFT)*/
    st_fx->uv_count_fx = 0;
    move16();
    st_fx->act_count_fx = 3;
    move16();
    Copy( st_fx->lsp_old_fx, st_fx->lspold_s_fx, M );
    st_fx->noimix_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->min_alpha_fx = 32767;
    move16();   /*1; Q15*/
    st_fx->exc_pe_fx = 0;
    move16();

    st_fx->cng_ener_seed1_fx = RANDOM_INITSEED;
    move16();
    set32_fx(st_fx->lp_env_fx, 0, NUM_ENV_CNG);
    set16_fx(st_fx->exc_mem_fx, 0, 24);
    set16_fx(st_fx->exc_mem1_fx, 0, 30);
    set32_fx(st_fx->old_env_fx, 0, NUM_ENV_CNG);
    /* LD music post-filter */
    set16_fx(st_fx->dct_post_old_exc_fx, 0, DCT_L_POST-OFFSET2 );
    /*st->LDm_enh_min_ns_gain = (float)pow(10.0f, -12/20.0f);*/
    L_tmp = L_mult(((-12*256)), 5443);     /* *0.166096 in Q15 -> Q24 */
    L_tmp = L_shr(L_tmp, 8);                    /* From Q24 to Q16 */
    f_db = L_Extract_lc(L_tmp, &e_db);          /* Extract exponent */
    f_db = extract_l(Pow2(14, f_db));           /* Put 14 as exponent so that */
    e_db = add(e_db, 15-14);
    f_db = add(f_db, shr(1,add(e_db,1)));
    st_fx->LDm_enh_min_ns_gain_fx = shl(f_db, e_db);

    st_fx->LDm_last_music_flag_fx = 0;
    move16();
    set16_fx( st_fx->LDm_lt_diff_etot_fx, 0, MAX_LT );

    st_fx->LDm_thres_fx[0] = TH_0_MIN_FX;
    move16();
    st_fx->LDm_thres_fx[1] = TH_1_MIN_FX;
    move16();
    st_fx->LDm_thres_fx[2] = TH_2_MIN_FX;
    move16();
    st_fx->LDm_thres_fx[3] = TH_3_MIN_FX;
    move16();

    st_fx->LDm_nb_thr_1_fx = 0;
    move16();
    st_fx->LDm_nb_thr_3_fx = 0;
    move16();
    st_fx->LDm_mem_etot_fx = 0;
    move16();

    FOR (i = 0; i < VOIC_BINS_HR; i++)
    {
        st_fx->LDm_enh_lp_gbin_fx[i] = 16384;
        move16();
        st_fx->LDm_enh_lf_EO_fx [i]  = 328;
        move16();
    }

    FOR (i = 0; i < MBANDS_GN_LD; i++)
    {
        st_fx->LDm_enro_fx[i] = E_MIN_Q15;
        move16();
        st_fx->LDm_bckr_noise_fx[i] = E_MIN_Q15;
        move16();
    }

    st_fx->LDm_last_bfi_count_fx = NB_BFI_THR;
    move16();
    set16_fx(st_fx->filt_lfE_fx, 4096, DCT_L_POST);
    move16();
    st_fx->last_nonfull_music_fx = 0;
    move16();
    st_fx->Old_ener_Q  = 0;
    move16();

    /* CNG and DTX */
    st_fx->cng_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->cng_ener_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->old_enr_index_fx = -1;
    move16();
    st_fx->Enew_fx = L_deposit_l(0);
    st_fx->first_CNG_fx = 0;
    move16();
    Copy( st_fx->lsp_old_fx, st_fx->lspCNG_fx, M );
    st_fx->last_allow_cn_step_fx = 0;
    move16();
    st_fx->shb_cng_ener_fx = -1541;
    move16();
    st_fx->wb_cng_ener_fx = -1541;
    move16();
    st_fx->last_wb_cng_ener_fx = -1541;
    move16();
    st_fx->last_shb_cng_ener_fx = -1541;
    move16();
    st_fx->swb_cng_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->ho_hist_ptr_fx = -1;
    move16();
    set16_fx( st_fx->ho_lsp_hist_fx, 0, HO_HIST_SIZE*M );
    set32_fx( st_fx->ho_ener_hist_fx, 0, HO_HIST_SIZE );
    set32_fx( st_fx->ho_env_hist_fx, 0, HO_HIST_SIZE*NUM_ENV_CNG );
    st_fx->ho_hist_size_fx = 0;
    move16();
    st_fx->act_cnt_fx = 0;
    move16();
    st_fx->ho_circ_ptr_fx = -1;
    move16();
    set16_fx( st_fx->ho_lsp_circ_fx, 0, HO_HIST_SIZE*M );
    set32_fx( st_fx->ho_ener_circ_fx, 0, HO_HIST_SIZE );
    set32_fx( st_fx->ho_env_circ_fx, 0, HO_HIST_SIZE*NUM_ENV_CNG );
    st_fx->ho_circ_size_fx = 0;
    move16();
    set16_fx( st_fx->ho_16k_lsp_fx, 0, HO_HIST_SIZE );
    st_fx->CNG_mode_fx = -1;
    move16();
    st_fx->last_active_brate_fx = ACELP_7k20;
    move32();
    st_fx->last_CNG_L_frame_fx = L_FRAME;
    move16();
    st_fx->act_cnt2_fx = 0;
    move16();
    st_fx->num_ho_fx = 0;
    move16();
    st_fx->cng_type_fx = -1;
    move16();
    st_fx->last_cng_type_fx = -1;
    move16();

    FOR ( i=0; i<LPC_SHB_ORDER; i++ )
    {
        st_fx->lsp_shb_prev_fx[i] = lsp_shb_prev_tbl_fx[i];
        st_fx->lsp_shb_prev_prev_fx[i] = st_fx->lsp_shb_prev_fx[i];
    }

    st_fx->shb_dtx_count_fx = 0;
    move16();
    st_fx->last_vad_fx = 0;
    move16();
    st_fx->trans_cnt_fx = 0;
    move16();
    st_fx->burst_cnt_fx = 0;
    move16();
    st_fx->last_shb_ener_fx = 0;
    move16();

    /* HF (6-7kHz) BWE */
    st_fx->seed2_fx = RANDOM_INITSEED;
    move16();
    st_fx->Q_stat_noise_ge = GE_SHIFT;
    move16();
    st_fx->cngTDLevel = 0;
    move16();
    st_fx->cngTDLevel_e = 0;
    move16();

    /*-----------------------------------------------------------------*
     * HR SWB BWE parameters
     *-----------------------------------------------------------------*/

    set16_fx( st_fx->old_out_hr_fx, 0, L_FRAME48k );
    st_fx->old_out_hr_exp_fx = 14;
    move16();

    set16_fx( st_fx->t_audio_prev_fx, 0, 2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_START_FREQ_COEF );
    set16_fx( st_fx->t_audio_prev_fx_exp, 0, NUM_TIME_SWITCHING_BLOCKS ); /* one exp per switching block */
    st_fx->old_is_transient_hr_bwe_fx = 0;
    move16();
    st_fx->bwe_highrate_seed_fx = 12345;
    move16();

    st_fx->L_mem_EnergyLT_fx = L_deposit_h(16384);
    st_fx->mem_EnergyLT_fx_exp = 40;
    move16(); /* set to a high exponent */


    /*-----------------------------------------------------------------*
     * HQ core parameters
     *-----------------------------------------------------------------*/

    set16_fx( st_fx->old_out_fx, 0, L_FRAME48k );
    set16_fx( st_fx->old_out_LB_fx, 0, L_FRAME48k );
    set32_fx( st_fx->old_coeffs_fx, 0, L_FRAME8k );
    st_fx->Q_old_wtda = 15;
    move16();
    st_fx->Q_old_postdec = 0;
    move16();
    st_fx->Qprev_synth_buffer_fx = 15;
    move16();
    set16_fx( st_fx->old_is_transient_fx, 0, 3 );
    st_fx->old_bfi_cnt_fx = 0;
    move16();
    set16_fx(st_fx->old_auOut_2fr_fx, 0, L_FRAME8k*2);
    set16_fx(st_fx->old_out_pha_fx[0], 0, N_LEAD_NB);
    set16_fx(st_fx->old_out_pha_fx[1], 0, N_LEAD_NB);
    st_fx->prev_old_bfi_fx = 0;
    move16();
    st_fx->phase_mat_flag_fx = 0;
    move16();
    st_fx->phase_mat_next_fx = 0;
    move16();
    st_fx->old_Min_ind_fx = 0;
    move16();
    st_fx->diff_energy_fx = 0;
    move16();
    set32_fx( st_fx->oldIMDCTout_fx, 0, L_FRAME8k );
    set16_fx( st_fx->prev_oldauOut_fx, 0, L_FRAME8k );
    st_fx->stat_mode_out_fx = 0;
    move16();
    st_fx->stat_mode_old_fx = 0;
    move16();
    st_fx->oldHqVoicing_fx = 0;
    move16();

    FOR( i=0; i <MAX_SB_NB; i++ )
    {
        FOR( j=0; j<MAX_PGF; j++ )
        {
            st_fx->ynrm_values_fx[i][j] = 0;
            move16();
        }
        FOR( j=0; j<MAX_ROW; j++ )
        {
            st_fx->r_p_values_fx[i][j] = 0;
            move16();
        }
    }

    set16_fx(st_fx->Norm_gain_fx, 1, NB_SFM);  /*check if it is in Q0  */
    set16_fx(st_fx->energy_MA_Curr_fx, 100, 2);/*check if it is in Q0*/
    st_fx->HQ_FEC_seed_fx = RANDOM_INITSEED;
    move16();
    set16_fx( st_fx->delay_buf_out_fx, 0, HQ_DELTA_MAX*HQ_DELAY_COMP );
    set16_fx( st_fx->previoussynth_fx, 0, L_FRAME48k);
    set16_fx( st_fx->old_synth_sw_fx, 0, L_FRAME48k/2 );
    st_fx->manE_peak_mem = L_deposit_l(0);
    st_fx->expE_peak_mem = 32;
    move16();
    set16_fx( st_fx->prev_noise_level_fx, 0, 2 );
    set16_fx( st_fx->prev_R_fx, 0, SFM_N );
    set32_fx( st_fx->prev_coeff_out_fx, 0, L_FRAME32k );
    set16_fx(st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM);

    /* HQ GENERIC */
    st_fx->hq_generic_seed_fx = RANDOM_INITSEED;
    move16();

    st_fx->mem_norm_fx[0] = 31;
    move16();
    set16_fx(st_fx->mem_norm_fx+1, 39, NB_SFM-1);
    st_fx->mem_env_delta_fx = 0;
    move16();
    st_fx->no_att_hangover_fx = 0;
    move16();
    st_fx->energy_lt_fx = 2457600;
    move16();   /* 300 in Q13 */

    st_fx->HqVoicing_fx = 0;
    move16();
    set16_fx( st_fx->fer_samples_fx, 0, L_FRAME48k );
    set32_fx( st_fx->prev_env_fx, 0, SFM_N_WB );
    set32_fx( st_fx->prev_normq_fx, 0, SFM_N_WB );
    st_fx->prev_hqswb_clas_fx = HQ_NORMAL;
    move16();

    /* LRMDCT_DELTA_NOISE_INJECT_FX */
    set32_fx( st_fx->last_ni_gain_fx, 0x0L, BANDS_MAX );
    set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
    st_fx->last_max_pos_pulse_fx = 0;
    move16();
    st_fx->prev_frm_hfe2_fx = 0;
    move16();
    st_fx->prev_stab_hfe2_fx = 0;
    move16();
    st_fx->prev_ni_ratio_fx = 16384;
    move16(); /* 0.5 */
    set16_fx( st_fx->prev_En_sb_fx, 0, NB_SWB_SUBBANDS );

    /* pre-echo reduction */
    reset_preecho_dec_fx( st_fx );


    /*----------------------------------------------------------------------------------*
     * HQ FEC
     *----------------------------------------------------------------------------------*/
    st_fx->old_synthFB_fx = st_fx->synth_history_fx + st_fx->output_Fs_fx/8000*L_FRAME8k;
    st_fx->prev_good_synth_fx = st_fx->old_synthFB_fx + NS2SA_fx2(st_fx->output_Fs_fx, PH_ECU_LOOKAHEAD_NS);
    set16_fx( st_fx->X_sav_fx, 0, PH_ECU_SPEC_SIZE );
    st_fx->Q_X_sav = 0;
    move16();
    st_fx->num_p_fx = 0;
    move16();
    st_fx->ph_ecu_active_fx = 0;
    move16();
    st_fx->ni_seed_forfec = 0;
    move16();
    st_fx->last_fec_fx = 0;
    move16();
    st_fx->ph_ecu_HqVoicing_fx = 0;
    move16();
    set16_fx(st_fx->gapsynth_fx, 0, L_FRAME48k);
    st_fx->env_stab_fx = 0x6000;    /* 0.75 (Q15) */                           move16();
    st_fx->mem_norm_hqfec_fx[0] = 31;
    move16();
    set16_fx(st_fx->mem_norm_hqfec_fx+1, 39, SFM_N_ENV_STAB-1);
    st_fx->mem_env_delta_hqfec_fx = 0;
    move16();
    st_fx->env_stab_plc_fx = 0;
    move16();
    set16_fx( st_fx->env_stab_state_p_fx, INV_NUM_ENV_STAB_PLC_STATES, NUM_ENV_STAB_PLC_STATES );
    st_fx->envstabplc_hocnt_fx = 0;
    move16();
    set16_fx(st_fx->mag_chg_1st_fx, 32767, Lgw_max);
    set16_fx(st_fx->Xavg_fx, 0, Lgw_max);
    st_fx->beta_mute_fx = BETA_MUTE_FAC_INI;
    move16();
    set16_fx( st_fx->prev_sign_switch_fx, 0, HQ_FEC_SIGN_SFM );
    set16_fx( st_fx->prev_sign_switch_2_fx, 0, HQ_FEC_SIGN_SFM );
    /* st->ni_seed_forfec = 0; */
    st_fx->time_offs_fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * SWB BWE parameters
     *-----------------------------------------------------------------*/
    st_fx->old_wtda_wb_fx_exp = 0;
    move16();
    set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );
    set16_fx( st_fx->L_old_wtda_swb_fx, 0, L_FRAME48k );
    st_fx->old_wtda_swb_fx_exp = 0;
    move16();
    st_fx->mem_imdct_exp_fx = 0;
    move16();

    st_fx->prev_mode_fx = NORMAL;
    move16();
    set16_fx( st_fx->prev_SWB_fenv_fx, 0, SWB_FENV );
    st_fx->prev_Energy_fx = 0;
    move16();
    st_fx->prev_L_swb_norm_fx = 8;
    move16();
    st_fx->Seed_fx = 21211;
    move16();
    st_fx->prev_frica_flag_fx = 0;
    move16();
    set16_fx( st_fx->mem_imdct_fx, 0, L_FRAME48k );
    st_fx->prev_td_energy_fx = 0;
    move16();
    st_fx->prev_weight_fx = 6554; /*0.2 in Q15*/            move16();
    st_fx->prev_flag_fx = 0;
    move16();
    st_fx->prev_coder_type_fx = GENERIC;
    move16();
    st_fx->last_wb_bwe_ener_fx = 0;
    move16();
    st_fx->prev_hb_synth_fx_exp = 0;
    move16();
    st_fx->tilt_wb_fx = 0;
    move16();

    st_fx->prev_Energy_wb_fx = L_deposit_l(0);

    /*-----------------------------------------------------------------*
     * TBE parameters
     *-----------------------------------------------------------------*/

    InitSWBdecBuffer_fx( st_fx );
    ResetSHBbuffer_Dec_fx(st_fx);

    IF( L_sub(st_fx->output_Fs_fx,48000) == 0 )
    {
        set32_fx( st_fx->fbbwe_hpf_mem_fx[0], 0, 4 );
        set32_fx( st_fx->fbbwe_hpf_mem_fx[1], 0, 4 );
        set32_fx( st_fx->fbbwe_hpf_mem_fx[2], 0, 4 );
        set32_fx( st_fx->fbbwe_hpf_mem_fx[3], 0, 4 );
    }

    set16_fx( st_fx->mem_resamp_HB_fx, 0, 2*L_FILT16k );
    set16_fx( st_fx->mem_resamp_HB_32k_fx, 0, 2*L_FILT32k );
    set16_fx( st_fx->prev_synth_buffer_fx, 0, NS2SA_fx2(48000, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS) );
    set16_fx( st_fx->hb_prev_synth_buffer_fx, 0, NS2SA_fx2(48000, DELAY_BWE_TOTAL_NS) );
    st_fx->old_bwe_delay_fx = -1; /*Q0*/                    move16();

    st_fx->tilt_mem_fx = 0;
    move16();
    set16_fx( st_fx->prev_lsf_diff_fx, 16384, LPC_SHB_ORDER );
    st_fx->prev_tilt_para_fx = 0;
    move16();
    set16_fx( st_fx->cur_sub_Aq_fx, 0, M+1 );
    set16_fx( st_fx->int_3_over_2_tbemem_dec_fx, 0, INTERP_3_2_MEM_LEN );
    set16_fx( st_fx->interpol_3_2_cng_dec_fx, 0, INTERP_3_2_MEM_LEN );

    /* TD BWE post-processing */
    st_fx->ptr_mem_stp_swb_fx = st_fx->mem_stp_swb_fx + LPC_SHB_ORDER - 1;
    set16_fx(st_fx->mem_zero_swb_fx, 0, LPC_SHB_ORDER);

    FOR ( i=0; i<LPC_SHB_ORDER; i++ )
    {
        st_fx->swb_lsp_prev_interp_fx[i] = swb_lsp_prev_interp_init[i];
        move16();
    }

    st_fx->prev1_shb_ener_sf_fx = 32767; /* Q15*/           move16();
    st_fx->prev2_shb_ener_sf_fx = 32767; /* Q15*/           move16();
    st_fx->prev3_shb_ener_sf_fx = 32767; /* Q15*/           move16();
    st_fx->prev_res_shb_gshape_fx = 8192; /* 0.125 in Q14*/ move16();
    st_fx->prev_mixFactors_fx = 16384;    /* 0.5 in Q15*/   move16();
    st_fx->prev_ener_fx = 0;
    move16();
    st_fx->prev_GainShape_fx = 0;
    move16();
    set16_fx( st_fx->fb_state_lpc_syn_fx, 0, LPC_SHB_ORDER );
    st_fx->fb_tbe_demph_fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * WB/SWB bandwidth switching parameters
     *-----------------------------------------------------------------*/

    st_fx->tilt_swb_fx = 0;
    move16();
    st_fx->prev_ener_fx = L_deposit_l(0);
    st_fx->prev_ener_shb_fx = 0;
    move16();
    st_fx->prev_enerLH_fx = 0;
    move16();
    st_fx->enerLH_fx = L_deposit_l(0);
    st_fx->enerLL_fx = L_deposit_l(0);
    st_fx->prev_enerLL_fx = 0;
    move16();
    st_fx->prev_fractive_fx = 0;
    move16();
    st_fx->bws_cnt_fx = N_WS2N_FRAMES;
    move16();
    st_fx->bws_cnt1_fx = N_NS2W_FRAMES;
    move16();
    st_fx->attenu_fx = 3277;
    move16();
    st_fx->last_inner_frame_fx = L_FRAME8k;
    move16();
    st_fx->last_hq_tilt_fx = 0;
    move16();
    st_fx->last_bwidth_fx = 0;
    move16();
    st_fx->prev_weight1_fx = 16384;
    move16();

    /*-----------------------------------------------------------------*
     * channel-aware mode parameters
     *-----------------------------------------------------------------*/

    set16_fx( st_fx->tilt_code_dec_fx, 0, NB_SUBFR16k );
    set32_fx( st_fx->gain_code_fx, 0, NB_SUBFR16k );

    st_fx->use_partial_copy = 0;
    move16();
    st_fx->prev_use_partial_copy = 0;
    move16();
    st_fx->rf_flag = 0;
    move16();
    st_fx->rf_flag_last = 0;
    st_fx->prev_use_partial_copy = 0;
    move16();
    st_fx->prev_rf_frame_type = 0;
    move16();
    st_fx->next_coder_type = 0;
    move16();

    st_fx->rf_target_bits = 0;
    move16();

    FOR( i = 0; i < MAX_RF_FEC_OFFSET; i++)
    {
        st_fx->rf_indx_lsf[i][0] = 0;
        move16();
        st_fx->rf_indx_lsf[i][1] = 0;
        move16();
        st_fx->rf_indx_lsf[i][2] = 0;
        move16();
        st_fx->rf_indx_EsPred[i] = 0;
        move16();
        st_fx->rf_indx_nelp_fid[i] = 0;
        move16();
        st_fx->rf_indx_nelp_iG1[i] = 0;
        move16();
        st_fx->rf_indx_nelp_iG2[i][0] = 0;
        move16();
        st_fx->rf_indx_nelp_iG2[i][1] = 0;
        move16();
        st_fx->rf_indx_tbeGainFr[i] = 0;
        move16();

        FOR( j = 0; j < NB_SUBFR16k; j++)
        {
            st_fx->rf_indx_ltfMode[i][j] = 0;
            move16();
            st_fx->rf_indx_pitch[i][j] = 0;
            move16();
            st_fx->rf_indx_fcb[i][j] = 0;
            move16();
            st_fx->rf_indx_gain[i][j] = 0;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Improvement of unvoiced and audio signals in AMR-WB IO mode parameters
     *-----------------------------------------------------------------*/

    st_fx->UV_cnt_fx = 30;
    move16();
    st_fx->LT_UV_cnt_fx = (60<<6);
    move16();
    set16_fx( st_fx->lt_diff_etot_fx, 0, MAX_LT );
    st_fx->Last_ener_fx = 0;
    move16();
    set16_fx(st_fx->old_Aq_fx, 0, 4*(M+1));
    st_fx->old_Aq_fx[0] = 16384;
    move16();
    st_fx->old_Aq_fx[M+1] = 16384;
    move16();
    st_fx->old_Aq_fx[2*(M+1)] = 16384;
    move16();
    st_fx->old_Aq_fx[3*(M+1)] = 16384;
    move16();
    st_fx->lt_voice_fac_fx = 0;
    move16();


    /*-----------------------------------------------------------------*
     * Postfilters
     *-----------------------------------------------------------------*/

    bass_psfilter_init_fx( st_fx->pst_old_syn_fx, &(st_fx->pst_mem_deemp_err_fx), &(st_fx->pst_lp_ener_fx) );
    st_fx->bpf_off_fx = 0;
    move16();
    set16_fx( st_fx->Track_on_hist_fx, 0, L_TRACK_HIST );
    set16_fx( st_fx->vibrato_hist_fx, 0, L_TRACK_HIST );
    set16_fx( st_fx->mem_mean_pit_fx, 80, L_TRACK_HIST );
    st_fx->psf_att_fx = 32767;
    move16();

    /*-----------------------------------------------------------------*
     * FD BPF & resampling tools parameters
     *-----------------------------------------------------------------*/
    /* open analysis for max. SR 48kHz */
    openCldfb ( &st_fx->cldfbAna_fx, CLDFB_ANALYSIS, CLDFB_getNumChannels(48000), 320 );

    /* open analysis BPF for max. SR 16kHz */
    openCldfb ( &st_fx->cldfbBPF_fx, CLDFB_ANALYSIS, CLDFB_getNumChannels(16000), 320 );

    /* open synthesis for output SR */
    openCldfb ( &st_fx->cldfbSyn_fx, CLDFB_SYNTHESIS, CLDFB_getNumChannels(st_fx->output_Fs_fx), st_fx->output_frame_fx);

    st_fx->Ng_ener_ST_fx = -13056;
    move16();    /*-51 IN Q8*/
    st_fx->Last_frame_ener_fx = MAX_32;
    move32();
    st_fx->old_Es_pred_fx = 0;
    move16();
    set16_fx(st_fx->old_Aq_12_8_fx + 1, 0, M );
    st_fx->old_Aq_12_8_fx[0] = 4096;
    move16();            /*1 in Q12*/

    /*-----------------------------------------------------------------*
     * SC-VBR
     *-----------------------------------------------------------------*/

    st_fx->gainp_ppp_fx = 0;
    move16();
    st_fx->FadeScale_fx = 32767;
    move16();  /* Q15*/
    st_fx->last_ppp_mode_dec_fx = 0;
    move16();
    st_fx->old_ppp_mode_fx = 0;
    move16();
    st_fx->ppp_mode_dec_fx = 0;
    move16();
    st_fx->last_nelp_mode_dec_fx = 0;
    move16();
    st_fx->nelp_mode_dec_fx = 0;
    move16();
    st_fx->nelp_dec_seed_fx = 0;
    move16();
    st_fx->firstTime_voiceddec_fx = 1;
    move16();
    st_fx->prev_gain_pit_dec_fx = 0;
    move16();
    st_fx->prev_tilt_code_dec_fx = 0;
    move16();
    st_fx->vbr_hw_BWE_disable_dec_fx = 0;
    move16();
    st_fx->last_vbr_hw_BWE_disable_dec_fx = 0;
    move16();
    set16_fx( st_fx->old_hb_synth_fx, 0, L_FRAME48k );

    /* DTFS variables */
    set16_fx( st_fx->dtfs_dec_a_fx, 0, MAXLAG_WI );
    set16_fx( st_fx->dtfs_dec_b_fx, 0, MAXLAG_WI );
    st_fx->dtfs_dec_lag_fx = 0;
    move16();
    st_fx->dtfs_dec_nH_fx = 0;
    move16();
    st_fx->dtfs_dec_nH_4kHz_fx = 0;
    move16();
    st_fx->dtfs_dec_upper_cut_off_freq_of_interest_fx = 0;
    move16();
    st_fx->dtfs_dec_upper_cut_off_freq_fx = 0;
    move16();
    st_fx->ph_offset_D_fx = 0;
    move16();
    st_fx->lastLgainD_fx = 0;
    move16();
    st_fx->lastHgainD_fx = 0;
    move16();
    set16_fx( st_fx->lasterbD_fx, 0, NUM_ERB_WB );
    st_fx->dtfs_dec_Q = 0;
    move16();

    /* NELP decoder variables */
    set32_fx( st_fx->bp1_filt_mem_nb_dec_fx, 0, 14 );
    set16_fx( st_fx->bp1_filt_mem_wb_dec_fx, 0, 8 );
    set16_fx( st_fx->shape1_filt_mem_dec_fx, 0, 20 );
    set16_fx( st_fx->shape2_filt_mem_dec_fx, 0, 20 );
    set16_fx( st_fx->shape3_filt_mem_dec_fx, 0, 20 );

    /* IGF */
    st_fx->igf = 0;
    move16();
    set16_fx( (Word16*)&st_fx->hIGFDec, 0, (sizeof(st_fx->hIGFDec)+1)/sizeof(Word16) );
    st_fx->hIGFDec.igfData.igfInfo.nfSeed = 9733;
    move16();

    st_fx->enablePlcWaveadjust = 0;
    move16();

    /* Init Decoder */
    st_fx->sr_core = DEC_SR_CORE_DEFAULT;
    open_decoder_LPD( st_fx, st_fx->total_brate_fx,  st_fx->bwidth_fx);

    st_fx->m_decodeMode = DEC_NO_FRAM_LOSS;
    move16();
    st_fx->m_frame_type = ACTIVE_FRAME;
    move16();
    st_fx->m_old_frame_type = ACTIVE_FRAME;
    move16();


    newCldfbBands = CLDFB_getNumChannels(L_mult0(st_fx->L_frame_fx, 50));

    resampleCldfb( st_fx->cldfbAna_fx, newCldfbBands, st_fx->L_frame_fx, 1 );
    resampleCldfb( st_fx->cldfbBPF_fx, newCldfbBands, st_fx->L_frame_fx, 1 );

    /* Create FD_CNG instance */
    createFdCngDec(&st_fx->hFdCngDec_fx);

    /* Init FD-CNG */
    initFdCngDec( st_fx->hFdCngDec_fx, st_fx->cldfbSyn_fx->scale );

    st_fx->lp_noise = FL2WORD32_SCALE(-20.f,8);

    /*-----------------------------------------------------------------*
     * initialzie Q values
     *-----------------------------------------------------------------*/

    st_fx->memExp1 = 0;
    move16();
    st_fx->Q_syn2 = 0;
    move16();
    st_fx->Q_exc = 8;
    st_fx->prev_Q_exc = 0;
    move16();
    st_fx->Q_syn = 0;
    move16();
    st_fx->prev_Q_syn = 0;
    move16();

    FOR(i=0; i<L_Q_MEM; i++)
    {
        st_fx->Q_subfr[i] = 0;
        move16();
    }

    /* Previous frame LPC initialization for PPP */
    st_fx->prev_Q_synth = 0;
    move16();

    st_fx->prev_Q_exc_fr = 0;
    move16();
    st_fx->prev_Q_syn_fr = 0;
    move16();

    return;

}


/*----------------------------------------------------------------------*
 * reset_preecho_dec()
 *
 * Initialization of static variables for pre-echo
 *----------------------------------------------------------------------*/

void reset_preecho_dec_fx(
    Decoder_State_fx *st_fx        /* i/o: Decoder static variables structure            */
)
{
    st_fx->memfilt_lb_fx = 0;
    move16();
    st_fx->mean_prev_hb_fx = L_deposit_l(0);
    st_fx->smoothmem_fx = 32767;
    move16();
    st_fx->mean_prev_fx = L_deposit_l(0);
    st_fx->mean_prev_nc_fx = L_deposit_l(0);
    st_fx->wmold_hb_fx = 32767;
    move16();
    st_fx->prevflag_fx = 0;
    move16();
    st_fx->pastpre_fx = 0;
    move16();

    return;
}


/*----------------------------------------------------------------------*
 * destroy_decoder()
 *
 * Free memory which was allocated in init_decoder()
 *----------------------------------------------------------------------*/

void destroy_decoder(
    Decoder_State_fx *st_fx     /* o:   Decoder static variables structure */
)
{

    /* CLDFB BPF & resampling tools */

    /* delete analysis for max. SR 16kHz */
    deleteCldfb(&st_fx->cldfbAna_fx);

    /* delete analysis BPF for max. SR 16kHz */
    deleteCldfb(&st_fx->cldfbBPF_fx);

    /* delete synthesis for output SR */
    deleteCldfb(&st_fx->cldfbSyn_fx);

    deleteFdCngDec( &st_fx->hFdCngDec_fx );

    return;
}
