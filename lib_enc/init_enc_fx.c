/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"


/*-----------------------------------------------------------------------*
 * init_encoder_fx()
 *
 * Initialization of state variables
 *-----------------------------------------------------------------------*/
void init_encoder_fx(
    Encoder_State_fx *st_fx   /* i/o: Encoder static variables structure  */
)
{
    Word16 i;

    st_fx->nb_bits_tot_fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * ACELP core parameters
     *-----------------------------------------------------------------*/
    st_fx->last_core_fx = -1;
    move16();
    if ( st_fx->Opt_AMR_WB_fx )
    {
        st_fx->last_core_fx = AMR_WB_CORE;
        move16();
    }

    st_fx->L_frame_fx = L_FRAME;
    move16();
    st_fx->last_coder_type_fx = GENERIC;
    move16();
    st_fx->last_7k2_coder_type_fx = GENERIC;
    move16();
    st_fx->last_total_brate_fx = st_fx->total_brate_fx;
    move32();
    st_fx->last_core_brate_fx = st_fx->total_brate_fx;
    move32();
    st_fx->extl_fx = -1;
    move16();
    st_fx->last_extl_fx = -1;
    move16();
    st_fx->last_L_frame_fx = L_FRAME;
    move16();
    st_fx->rate_switching_reset = 0;
    move16();
    Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
    Copy( GEWB_Ave_fx, st_fx->lsfoldbfi0_fx, M );
    Copy( GEWB_Ave_fx, st_fx->lsfoldbfi1_fx, M );
    Copy( GEWB_Ave_fx, st_fx->lsf_adaptive_mean_fx, M );
    init_lvq_fx( st_fx->offset_scale1_fx, st_fx->offset_scale2_fx, st_fx->offset_scale1_p_fx, st_fx->offset_scale2_p_fx, st_fx->no_scales_fx, st_fx->no_scales_p_fx );
    st_fx->next_force_safety_net_fx = 0;

    st_fx->pstreaklen_fx = 0;
    move16();
    st_fx->streaklimit_fx = 32767;
    move16();  /*1;//Q15  */
    set16_fx( st_fx->mem_MA_fx, 0, M );

    init_gp_clip_fx( st_fx->clip_var_fx );
    pitch_ol_init_fx( &st_fx->old_thres_fx, &st_fx->old_pitch, &st_fx->delta_pit_fx, &st_fx->old_corr_fx ) ;

    hf_cod_init_fx( st_fx->mem_hp400_enc_fx, st_fx->mem_hf_enc_fx, st_fx->mem_syn_hf_enc_fx, st_fx->mem_hf2_enc_fx, &st_fx->gain_alpha_fx );

    st_fx->LPDmem.tilt_code = 0;
    move16();
    st_fx->LPDmem.gc_threshold = 0;
    move16();

    st_fx->clas_fx = UNVOICED_CLAS;
    move16();
    set16_fx( st_fx->old_inp_12k8_fx, 0, L_INP_MEM);                      /* memory of input signal at 12.8kHz */
    set16_fx( st_fx->old_wsp_fx, 0, L_WSP_MEM );
    set16_fx( st_fx->LPDmem.old_exc, 0, L_EXC_MEM );
    set16_fx( st_fx->old_wsp2_fx, 0, (L_WSP_MEM - L_INTERPOL)/OPL_DECIM );
    set16_fx( st_fx->old_inp_16k_fx, 0, L_INP_MEM );

    st_fx->mem_deemph_fx = 0;
    move16();
    st_fx->mem_preemph_fx = 0;
    move16();
    st_fx->mem_preemph16k_fx = 0;
    move16();
    st_fx->mem_preemph_enc = 0;
    move16();

    /* AVQ pre-quantizer memory */
    st_fx->mem_preemp_preQ_fx = 0;
    move16();
    st_fx->mem_deemp_preQ_fx = 0;
    move16();
    st_fx->last_nq_preQ_fx = 0;
    move16();
    st_fx->use_acelp_preq = 0;
    move16();

    /* (Decimated) Weighted Speech Memory */
    st_fx->mem_wsp_enc = 0;
    move16();

    set16_fx( st_fx->mem_decim16k_fx, 0, 2*L_FILT_MAX );
    st_fx->mem_wsp_fx = 0;
    move16();
    st_fx->LPDmem.mem_w0 = 0;
    move16();
    set16_fx( st_fx->LPDmem.mem_syn, 0, M );
    set16_fx( st_fx->mem_syn1_fx, 0, M );
    st_fx->mem_deemph_old_syn_fx = 0;
    move16();
    set16_fx( st_fx->LPDmem.mem_syn2, 0, M );
    set16_fx( st_fx->mem_decim_fx, 0, 2*L_FILT_MAX );
    set16_fx( st_fx->mem_decim2_fx, 0, 3 );
    set32_fx( st_fx->Bin_E_fx, 0, L_FFT );
    set16_fx( st_fx->lgBin_E_fx, 0, L_FFT );
    set32_fx( st_fx->Bin_E_old_fx, 0, L_FFT/2 );
    set16_fx( st_fx->LPDmem.mem_syn3, 0, M );

    st_fx->ini_frame_fx = 0;
    move16();
    st_fx->ee_old_fx = 640;
    move32();/*chk //10 in Q6 */
    st_fx->Nb_ACELP_frames_fx = 0;
    move16();
    st_fx->audio_frame_cnt_fx = AUDIO_COUNTER_INI;
    move16();/* Initializatin of the audio frame counter mildly into the audio mode      */

    /* adaptive lag window memory */
    st_fx->old_pitch_la = 0;
    move16();
    st_fx->old_voicing_la = 0;
    move16();
    set32_fx( st_fx->mem_hp20_in_fx, 0, 5 );

    st_fx->Last_Resort_fx = 0;
    st_fx->set_ppp_generic_fx = 0;

    st_fx->dm_fx.prev_state = 0;
    move16();       /* This corresponds to st_fx->dispMem in FLP */
    st_fx->dm_fx.prev_gain_code = 0;
    move32();

    FOR(i=2; i<8; i++)
    {
        st_fx->dm_fx.prev_gain_pit[i-2] = 0;
        move16();
    }

    st_fx->seed2_enc_fx = RANDOM_INITSEED;
    move16();


    st_fx->old_hpfilt_in_fx = 0;
    move16();
    st_fx->old_hpfilt_out_fx = 0;
    move16();
    st_fx->EnergyLT_fx = 0;
    move32();
    st_fx->prev_Q_new = 0;
    move16();


    FOR (i=0; i<GAIN_PRED_ORDER; i++)
    {
        st_fx->past_qua_en_fx[i] = -14336;   /* Q10gain quantization memory (used also in AMR-WB IO mode) */
    }

    IF( L_sub(st_fx->input_Fs_fx,8000) == 0 )
    {
        st_fx->min_band_fx = 1;
        move16();
        st_fx->max_band_fx = 16;
        move16();
    }
    ELSE
    {
        st_fx->min_band_fx = 0;
        move16();
        st_fx->max_band_fx = 19;
        move16();
    }

    FOR( i=0; i<NB_BANDS; i++ )
    {
        st_fx->fr_bands1_fx[i] = 1;
        move32();/*1e-5f; */
        st_fx->fr_bands2_fx[i] = 1;
        move32();/*1e-5f; */
        st_fx->ave_enr2_fx[i] = E_MIN_FX;
        move32(); /*Q7//E_MIN; */
    }
    IF ( st_fx->Opt_AMR_WB_fx )
    {
        Copy( mean_isf_amr_wb_fx, st_fx->lsf_old_fx, M );
        E_LPC_isf_isp_conversion( st_fx->lsf_old_fx, st_fx->lsp_old1_fx, M);
    }
    ELSE
    {
        Copy( GEWB_Ave_fx, st_fx->lsf_old_fx, M );
        lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old1_fx, M, INT_FS_FX);
    }

    Copy( st_fx->lsf_old_fx, st_fx->lsf_old1_fx, M );
    Copy( st_fx->lsp_old1_fx, st_fx->lsp_old_fx, M );
    Copy( st_fx->lsp_old_fx, st_fx->lsp_old16k_fx, M );
    Copy( st_fx->lsp_old_fx, st_fx->lspold_enc_fx, M );

    st_fx->stab_fac_fx = 0;
    move16();

    MDCT_selector_reset(st_fx);

    /* Bass post-filter memories - encoder side of MODE2 */
    st_fx->bpf_off_fx = 0;
    move16();

    /* TC mode */
    st_fx->tc_cnt_fx = 0;
    move16();
    st_fx->mCb1_fx = 0;
    move16();

    /* AC mode */
    st_fx->seed_tcx_fx = 15687;
    move16();
    st_fx->cor_strong_limit_fx = 1;
    move16();
    set16_fx( st_fx->last_exc_dct_in_fx, 0, L_FRAME );
    st_fx->last_ener_fx = 0;
    set16_fx( st_fx->last_bitallocation_band_fx, 0, MBANDS_GN );

    st_fx->mem_last_pit_band_fx = BAND1k2+1;
    move16();

    st_fx->old_dE1_fx = 0;
    move16();
    st_fx->old_ind_deltaMax_fx = 0;
    move32();
    set32_fx( st_fx->old_enr_ssf_fx, 0, 2*NB_SSF );
    st_fx->spike_hyst_fx = -1;
    move16();
    st_fx->music_hysteresis_fx = 0;
    move16();          /* Counter of frames after AUDIO frame to prevent UC */
    st_fx->last_harm_flag_acelp_fx = 0;
    move16();
    st_fx->GSC_noisy_speech_fx = 0;
    move16();

    /* speech/music classifier */
    st_fx->inact_cnt_fx = 0;
    move16();
    set16_fx(st_fx->past_dec_fx, 0, HANG_LEN );
    set16_fx(st_fx->past_dlp_fx, 0, HANG_LEN );
    set16_fx(st_fx->past_log_enr_fx, -1448, NB_BANDS_SPMUS);  /* log(E_MIN) in Q8 */

    st_fx->sp_mus_state_fx = -8;
    move16();
    st_fx->wdrop_fx = 0;
    move16();
    st_fx->wdlp_0_95_sp_fx = 0;
    move16();
    set16_fx( st_fx->last_lsp_fx, 0, M_LSP_SPMUS );
    st_fx->last_cor_map_sum_fx = 0;
    move16();
    st_fx->last_non_sta_fx = 0;
    move16();
    set32_fx( st_fx->past_PS_fx, 0, HIGHEST_FBIN );
    st_fx->past_ps_diff_fx = 0;
    move16();
    st_fx->past_epsP2_fx = 1024;
    move16();


    st_fx->gsc_thres_fx[0] = TH_0_MIN_FX;
    move16();
    st_fx->gsc_thres_fx[1] = TH_1_MIN_FX;
    move16();
    st_fx->gsc_thres_fx[2] = TH_2_MIN_FX;
    move16();
    st_fx->gsc_thres_fx[3] = TH_3_MIN_FX;
    move16();
    set16_fx(st_fx->gsc_lt_diff_etot_fx, 0, 40);
    st_fx->gsc_mem_etot_fx = 0;
    move16();
    st_fx->gsc_last_music_flag_fx = 0;
    move16();
    st_fx->gsc_last_bfi_count_fx = NB_BFI_THR;
    move16();

    st_fx->gsc_nb_thr_1_fx = 0;
    move16();
    st_fx->gsc_nb_thr_3_fx = 0;
    move16();
    st_fx->mold_corr_fx = 29491;
    move16();
    st_fx->lt_gpitch_fx = 0;
    move16();
    st_fx->mean_avr_dyn_fx  = 64;
    move16();/*Q7 */
    st_fx->last_sw_dyn_fx = 2560;
    move16();
    st_fx->pit_exc_hangover = 0;
    move16();
    st_fx->Last_pulse_pos_fx = 0;
    move16();

    /* speech/music classifier improvement */
    FOR ( i=0; i<BUF_LEN; i++ )
    {
        st_fx->buf_flux_fx[i] = -12800;
        move16();   /*-100.0 in Q7 */
        st_fx->buf_pkh_fx[i] = 0;
        move16();
        st_fx->buf_epsP_tilt_fx[i] = 0;
        move16();
        st_fx->buf_cor_map_sum_fx[i] = 0;
        move16();
        st_fx->buf_Ntonal_fx[i] = 0;
        move16();
        st_fx->buf_Ntonal2_fx[i] = 0;
        move16();
        st_fx->buf_Ntonal_lf_fx[i] = 0;
        move16();
    }

    set16_fx(st_fx->lpe_buf_fx, 0, HANG_LEN_INIT);
    set16_fx(st_fx->voicing_buf_fx, 0, HANG_LEN_INIT);
    st_fx->gsc_hangover_fx = 0;
    move16();
    set16_fx(st_fx->sparse_buf_fx, 0, HANG_LEN_INIT);
    set16_fx(st_fx->hf_spar_buf_fx, 0, HANG_LEN_INIT);
    st_fx->LT_sparse_fx = 0;
    move16();
    st_fx->gsc_cnt_fx = 0;
    move16();
    st_fx->last_vad_spa_fx = 0;
    move16();

    set16_fx( st_fx->old_Bin_E_fx, 0, 3*N_OLD_BIN_E );
    set16_fx( st_fx->buf_etot_fx, 0, 4 );
    set16_fx( st_fx->buf_dlp_fx, 0, 10 );

    st_fx->UV_cnt1_fx = 300;
    move16();
    st_fx->LT_UV_cnt1_fx = 16000;
    move16();   /*250.0f in Q6 */
    st_fx->onset_cnt_fx = 0;
    move16();
    st_fx->attack_hangover_fx = 0;
    move16();
    st_fx->dec_mov_fx = 0;
    move16();
    st_fx->dec_mov1_fx = 0;
    move16();
    st_fx->mov_log_max_spl_fx = 25600;
    move16();   /*200.0 in Q7 */
    st_fx->old_lt_diff_fx[0] = 0;
    move16();
    st_fx->old_lt_diff_fx[1] = 0;
    move16();

    /* GSC - pitch excitation parameters */
    st_fx->mem_w0_tmp_fx = 0;
    move16();
    set16_fx(st_fx->mem_syn_tmp_fx, 0, M);
    st_fx->high_stable_cor_fx = 0;
    move16();
    set16_fx(st_fx->var_cor_t_fx, 0, VAR_COR_LEN);

    st_fx->lps_fx = 0;
    move16();
    st_fx->lpm_fx = 0;
    move16();
    st_fx->Last_frame_ener_fx = MAX_32;
    move32();
    st_fx->lt_dec_thres_fx = 5120;
    move16(); /*10 in Q9 */
    st_fx->ener_RAT_fx = 0;
    move16();
    st_fx->mid_dyn_fx = 5120;
    move16(); /*40 -> Q7 */
    set16_fx( st_fx->old_y_gain_fx, 0, MBANDS_GN );
    st_fx->noise_lev_fx = NOISE_LEVEL_SP0;
    move16();
    st_fx->past_dyn_dec_fx = 0;
    move16();

    wb_vad_init_fx( &st_fx->nb_active_frames_fx, &st_fx->hangover_cnt_fx, &st_fx->lp_speech_fx, &st_fx->nb_active_frames_he_fx,
                    &st_fx->hangover_cnt_he_fx, &st_fx->bcg_flux_fx, &st_fx->soft_hangover_fx, &st_fx->voiced_burst_fx,
                    &st_fx->bcg_flux_init_fx, &st_fx->nb_active_frames_he1_fx, &st_fx->hangover_cnt_he1_fx,
                    &st_fx->L_vad_flag_reg_H_fx, &st_fx->L_vad_flag_reg_L_fx, &st_fx->L_vad_prim_reg_fx, &st_fx->vad_flag_cnt_50_fx,
                    &st_fx->vad_prim_cnt_16_fx, &st_fx->hangover_cnt_dtx_fx, &st_fx->hangover_cnt_music_fx );

    st_fx->nb_active_frames_HE_SAD_fx = 0;
    move16();
    st_fx->hangover_cnt_HE_SAD_fx = 0;
    move16();

    /* avoid uninitialized memory access */
    noise_est_init_fx( &st_fx->totalNoise_fx, &st_fx->first_noise_updt_fx, st_fx->bckr_fx, st_fx->enrO_fx,
                       st_fx->ave_enr_fx, &st_fx->pitO_fx, &st_fx->aEn_fx, &st_fx->harm_cor_cnt_fx,&st_fx->bg_cnt_fx,
                       &st_fx->lt_tn_track_fx, &st_fx->lt_tn_dist_fx, &st_fx->lt_Ellp_dist_fx,
                       &st_fx->lt_haco_ev_fx, &st_fx->low_tn_track_cnt_fx);

    st_fx->epsP_0_2_lp_fx       =  4096; /*1.0 Q12*/    move16();
    st_fx->epsP_0_2_ad_lp_fx    =  0;
    move16();
    st_fx->epsP_2_16_lp_fx      =  4096;
    move16();
    st_fx->epsP_2_16_lp2_fx     =  4096;
    move16();
    st_fx->epsP_2_16_dlp_lp2_fx =  0;
    move16();
    st_fx->lt_aEn_zero_fx       =  0;
    move16();




    st_fx->prim_act_quick_fx=0;
    move16();
    st_fx->prim_act_slow_fx=0;
    move16();
    st_fx->prim_act_fx=0;
    move16();
    st_fx->prim_act_quick_he_fx=0;
    move16();
    st_fx->prim_act_slow_he_fx=0;
    move16();
    st_fx->prim_act_he_fx=0;
    move16();
    st_fx->bckr_tilt_lt=0;
    move32();

    /* WB, SWB and FB bandwidth detector */
    st_fx->lt_mean_NB_fx  = 0;
    move16();
    st_fx->lt_mean_WB_fx  = 0;
    move16();
    st_fx->lt_mean_SWB_fx = 0;
    move16();
    st_fx->count_WB_fx  = BWD_COUNT_MAX;
    move16();
    st_fx->count_SWB_fx = BWD_COUNT_MAX;
    move16();
    st_fx->count_FB_fx  = BWD_COUNT_MAX;
    move16();
    st_fx->bwidth_fx = st_fx->max_bwidth_fx;
    move16();
    st_fx->last_input_bwidth_fx = st_fx->bwidth_fx;
    move16();
    st_fx->last_bwidth_fx = st_fx->bwidth_fx;
    move16();

    /* Tonal detector */
    FOR ( i=0; i<L_FFT/2; i++)
    {
        st_fx->old_S_fx[i] = 1;
        move16();
    }
    set16_fx( st_fx->cor_map_fx, 0, L_FFT/2 );
    st_fx->act_pred_fx = 32767;
    move16();
    st_fx->noise_char_fx = 0;
    move16();
    st_fx->multi_harm_limit_fx = THR_CORR_INIT_FX;
    move16();
    st_fx->coder_type_raw_fx = VOICED;
    st_fx->last_coder_type_raw_fx = st_fx->coder_type_raw_fx;

    /* Stationary noise UV modification  */
    st_fx->ge_sm_fx = 640;
    move32();/*Q(GE_SHIFT) */
    st_fx->uv_count_fx = 0;
    move16();
    st_fx->act_count_fx = 3;
    move16();
    Copy(st_fx->lsp_old_fx, st_fx->lspold_s_fx, M);
    st_fx->noimix_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->min_alpha_fx = 1;
    move16();
    st_fx->exc_pe_fx = 0;
    move16();

    /* CNG and DTX */
    st_fx->lp_noise_fx = 0;
    move16();
    Copy( st_fx->lsp_old1_fx, st_fx->lspCNG_fx, M );
    st_fx->cng_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->cng_ener_seed_fx = RANDOM_INITSEED;
    move16();
    st_fx->cng_ener_seed1_fx = RANDOM_INITSEED;
    st_fx->lp_ener_fx = 0;
    move32();
    st_fx->first_CNG_fx = 0;
    move16();
    st_fx->cnt_SID_fx = 0;
    move16();
    st_fx->max_SID_fx = 2;
    move16();
    st_fx->old_enr_index_fx = -1;
    move16();
    st_fx->Enew_fx = 0;
    move32();
    st_fx->VarDTX_cnt_voiced_fx = 0;
    move16();
    st_fx->lt_ener_voiced_fx = 0;
    move32();
    st_fx->VarDTX_cnt_noise_fx = 0;
    move16();
    st_fx->lt_ener_noise_fx = 0;
    move32();
    st_fx->lt_ener_last_SID_fx = 0;
    move32();
    if( st_fx->var_SID_rate_flag_fx )
    {
        st_fx->interval_SID_fx = 12;
        move16();
    }
    st_fx->lp_sp_enr_fx = 0;
    move16();
    st_fx->last_allow_cn_step_fx = 0;
    move16();

    st_fx->fd_cng_reset_flag = 0;
    move16();

    IF( st_fx->Opt_DTX_ON_fx )
    {
        st_fx->cng_hist_ptr_fx = -1;
        move16();
        set16_fx( st_fx->cng_lsp_hist_fx, 0, DTX_HIST_SIZE*M );
        set16_fx( st_fx->cng_ener_hist_fx, 0, DTX_HIST_SIZE );
        st_fx->cng_cnt_fx = 0;
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
    }
    set32_fx( st_fx->ho_env_circ_fx, 0, HO_HIST_SIZE*NUM_ENV_CNG );
    st_fx->active_fr_cnt_fx = 0;
    move16();
    st_fx->cng_type_fx = -1;
    move16();

    st_fx->CNG_mode_fx = -1;
    move16();
    st_fx->last_active_brate_fx = ACELP_7k20;
    move32();
    st_fx->last_CNG_L_frame_fx = L_FRAME;
    move16();
    set16_fx( st_fx->ho_16k_lsp_fx, 0, HO_HIST_SIZE );
    st_fx->act_cnt2_fx = 0;
    move16();
    st_fx->num_ho_fx = 0;
    move16();
    st_fx->hangover_terminate_flag_fx = 0;
    move16();

    st_fx->ho_circ_ptr_fx = -1;
    move16();
    set16_fx( st_fx->ho_lsp_circ_fx, 0, HO_HIST_SIZE*M );
    set32_fx( st_fx->ho_ener_circ_fx, 0, HO_HIST_SIZE );
    set32_fx( st_fx->ho_env_circ_fx, 0, HO_HIST_SIZE*NUM_ENV_CNG );
    st_fx->ho_circ_size_fx = 0;
    move16();
    st_fx->burst_ho_cnt_fx = 0;
    move16();
    st_fx->cng_buf_cnt = 0;
    move16();

    test();
    test();
    IF( st_fx->var_SID_rate_flag_fx || ((!st_fx->var_SID_rate_flag_fx) && (st_fx->interval_SID_fx >= DTX_HIST_SIZE)) )
    {
        st_fx->cng_hist_size_fx = DTX_HIST_SIZE;
        move16();
    }
    ELSE
    {
        st_fx->cng_hist_size_fx = st_fx->interval_SID_fx;
        move16();
    }
    set32_fx(st_fx->lp_env_fx, 0, 20);
    set32_fx(st_fx->cng_res_env_fx, 0, 20*8);
    set16_fx(st_fx->exc_mem_fx, 0, 24);
    set16_fx(st_fx->exc_mem1_fx, 0, 30);
    set16_fx(st_fx->exc_mem2_fx, 0, 30);
    set32_fx(st_fx->old_env_fx, 0, NUM_ENV_CNG);

    /* SWB CNG/DTX */
    st_fx->last_wb_cng_ener_fx = -1541;
    move16();   /* Q8 */
    st_fx->last_shb_cng_ener_fx = -1541;
    move16();   /* Q8 */
    st_fx->mov_wb_cng_ener_fx = -1541;
    move16();   /* Q8 */
    st_fx->mov_shb_cng_ener_fx = -1541;
    move16();   /* Q8 */
    st_fx->shb_cng_ini_cnt_fx = 1;
    move16();
    st_fx->shb_NO_DATA_cnt_fx = 0;
    move16();
    st_fx->last_SID_bwidth_fx = s_min(st_fx->max_bwidth_fx, SWB);
    st_fx->last_vad_fx = 0;
    move16();

    /* FEC */
    st_fx->last_clas_fx = UNVOICED_CLAS;
    move16();

    FOR (i=0; i<2*NB_SUBFR16k; i++)
    {
        st_fx->old_pitch_buf_fx[i] = L_SUBFR_Q6;
        move16();
    }
    st_fx->old_Es_pred_fx = 0;
    move16();
    set16_fx(st_fx->old_Aq_12_8_fx + 1, 0, M );
    st_fx->old_Aq_12_8_fx[0] = 4096;
    move16();

    /*-----------------------------------------------------------------*
     * CLDFB Analysis
     *-----------------------------------------------------------------*/

    /* open analysis for input SR */
    openCldfb ( &st_fx->cldfbAna_Fx, CLDFB_ANALYSIS, CLDFB_getNumChannels(st_fx->input_Fs_fx), st_fx->input_frame_fx );

    openCldfb ( &st_fx->cldfbSyn_Fx, CLDFB_SYNTHESIS, CLDFB_getNumChannels(16000), L_FRAME16k);

    st_fx->energyCoreLookahead_Fx = 0;
    move32();
    st_fx->sf_energyCoreLookahead_Fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * SC-VBR parameters
     *-----------------------------------------------------------------*/

    st_fx->nelp_enc_seed_fx = 0;
    move16();
    st_fx->last_nelp_mode_fx = 0;
    move16();
    st_fx->pppcountE_fx = 0;
    move16();
    st_fx->last_ppp_mode_fx = 0;
    move16();
    st_fx->last_last_ppp_mode_fx = 0;
    move16();
    st_fx->firstTime_voicedenc_fx = 1;
    move16();
    st_fx->prev_ppp_gain_pit_fx = 0;
    move16();
    st_fx->prev_tilt_code_fx = 0;
    move16();

    /* stable short pitch detection */
    st_fx->voicing0_sm_fx = 0;
    move16();
    st_fx->voicing_sm_fx = 0;
    move16();
    st_fx->LF_EnergyRatio_sm_fx = 128;
    move16();
    st_fx->predecision_flag_fx = 0;
    move16();
    st_fx->diff_sm_fx = 0;
    move32();
    st_fx->energy_sm_fx = 0;
    move32();

    st_fx->pattern_m_fx = 0;
    move16();
    st_fx->Last_Resort_fx = 0;
    move16();
    st_fx->Q_to_F_fx = 0;
    move16();

    st_fx->numactive_fx = 0;                /* keep the count of the frames inside current 600 frame bloack.*/
    st_fx->sum_of_rates_fx = 0;             /* sum of the rates of past 600 active frames*/
    st_fx->global_avr_rate_fx = 0;          /* global rate upto current time. recorded a (rate in kbps) *6000*/
    st_fx->frame_cnt_ratewin_fx = 0;        /* 600 active frame block count. Used to update the global rate*/
    st_fx->rate_control_fx = 0;
    move16();
    st_fx->SNR_THLD_fx = SNR_THLD_FX_Q8;
    move16();
    st_fx->mode_QQF_fx = 1;
    move16();


    /*-----------------------------------------------------------------*
     * SWB BWE parameters
     *-----------------------------------------------------------------*/

    set16_fx( st_fx->new_input_hp_fx, 0, NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS));
    set16_fx( st_fx->old_input_fx, 0, NS2SA(48000, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) );
    set16_fx( st_fx->old_input_wb_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_12k8_NS) );
    set16_fx( st_fx->old_input_lp_fx, 0, NS2SA(16000, ACELP_LOOK_NS + DELAY_SWB_TBE_16k_NS) );
    set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );

    st_fx->prev_mode_fx = NORMAL;
    move16();
    st_fx->modeCount_fx = 0;
    move16();
    st_fx->EnergyLF_fx = 0;
    move32();

    set16_fx( st_fx->L_old_wtda_swb_fx, 0, L_FRAME48k );

    st_fx->prev_Q_input_lp = 0;
    move16();

    st_fx->prev_global_gain_fx = 0;
    move32();
    st_fx->prev_Q_shb = 0;
    move16();
    st_fx->prev_L_swb_norm1_fx = 8;
    move16();
    st_fx->last_Opt_SC_VBR_fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * TBE parameters
     *-----------------------------------------------------------------*/

    InitSWBencBuffer_fx(st_fx);
    ResetSHBbuffer_Enc_fx(st_fx);
    set16_fx( st_fx->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k );
    set16_fx( st_fx->old_speech_wb_fx, 0, (L_LOOK_16k + L_SUBFR16k) * 5/16);
    set16_fx( st_fx->old_input_fhb_fx, 0, NS2SA(48000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) - L_FRAME48k/2);
    st_fx->cldfbHBLT = FL2WORD16_SCALE(1.0f, 15 - 13);

    st_fx->prev_gainFr_SHB_fx = 0;
    set16_fx(st_fx->lsp_shb_slow_interpl_fx, 0, LPC_SHB_ORDER);
    set16_fx(st_fx->lsp_shb_fast_interpl_fx, 0, LPC_SHB_ORDER);
    set16_fx( st_fx->shb_inv_filt_mem_fx, 0, LPC_SHB_ORDER );
    set16_fx( st_fx->lsp_shb_spacing_fx, 3277, 3);
    st_fx->prev_swb_GainShape_fx= 0;
    move16();
    st_fx->prev_frGainAtten_fx = 0;
    move16();
    st_fx->prev_wb_GainShape = 0;
    move16();
    set16_fx(st_fx->fb_state_lpc_syn_fx, 0, LPC_SHB_ORDER);
    st_fx->fb_tbe_demph_fx = 0;
    move16();
    st_fx->tilt_mem_fx = 0;
    move16();

    st_fx->prev_coder_type_fx = GENERIC;
    move16();
    set16_fx( st_fx->prev_lsf_diff_fx, 16384, LPC_SHB_ORDER );
    st_fx->prev_tilt_para_fx = 0;
    move16();
    set16_fx( st_fx->cur_sub_Aq_fx, 0, M+1 );

    /* TD BWE post-processing */
    st_fx->ptr_mem_stp_swb_fx = st_fx->mem_stp_swb_fx + LPC_SHB_ORDER - 1;
    set16_fx(st_fx->mem_zero_swb_fx, 0, LPC_SHB_ORDER);

    FOR ( i=0; i<LPC_SHB_ORDER; i++ )
    {
        st_fx->swb_lsp_prev_interp_fx[i] = swb_lsp_prev_interp_init[i];
    }

    set16_fx( st_fx->dec_2_over_3_mem_fx, 0, 12 );
    set16_fx( st_fx->dec_2_over_3_mem_lp_fx, 0, 6 );
    set16_fx( st_fx->old_fdbwe_speech_fx, 0, L_FRAME48k );

    /*-----------------------------------------------------------------*
     * HR SWB BWE parameters
     *-----------------------------------------------------------------*/
    set16_fx( st_fx->L_old_wtda_hr_fx, 0, L_FRAME48k );
    st_fx->old_wtda_hr_fx_exp = 0;
    move16();

    /*-----------------------------------------------------------------*
     * HQ core parameters
     *-----------------------------------------------------------------*/

    st_fx->input = st_fx->input_buff+L_FRAME48k+NS2SA(48000, DELAY_FIR_RESAMPL_NS);
    set16_fx( st_fx->input_buff+L_FRAME48k, 0, L_FRAME48k+NS2SA(48000, DELAY_FIR_RESAMPL_NS) );
    st_fx->old_input_signal_fx = st_fx->input - add(NS2SA_fx2(st_fx->input_Fs_fx, DELAY_FIR_RESAMPL_NS), st_fx->input_frame_fx);

    st_fx->Q_old_out = 0;
    move16();

    st_fx->Energy_Old_fx = 0;
    move16();
    st_fx->Q_old_wtda=15;
    move16();
    st_fx->EnergyLT_fx = 1;
    move32();
    st_fx->EnergyLT_fx_exp = 30;
    move16(); /* Set to a High Exponent so it is 1^-30 */
    st_fx->TransientHangOver_fx = 0;
    move16();

    set16_fx( st_fx->old_out_fx, 0, L_FRAME48k );
    st_fx->mode_count_fx = 0;
    move16();
    st_fx->mode_count1_fx = 0;
    move16();

    st_fx->hq_generic_speech_class_fx = 0;
    move16();
    st_fx->prev_Npeaks_fx = 0;
    set16_fx(st_fx->prev_peaks_fx, 0, HVQ_MAX_PEAKS);
    st_fx->hvq_hangover_fx = 0;
    st_fx->manE_peak_mem = 0;
    move32();
    st_fx->expE_peak_mem = 32;
    move16();

    st_fx->prev_hqswb_clas_fx = HQ_NORMAL;

    set16_fx(st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM);
    st_fx->clas_sec_old_fx   = 8192; /* 1.0f in Q13 */;
    st_fx->clas_final_old_fx = 1;    /* Q0 */
    st_fx->last_gain1 = 0;
    move32();
    st_fx->last_gain2 = 0;
    move32();
    /* speech/music classification */
    set16_fx(st_fx->lt_old_mode, 1, 3);
    st_fx->lt_voicing = FL2WORD16(0.5f);
    st_fx->lt_corr = FL2WORD16(0.5f);
    st_fx->lt_tonality = 0;
    move32();
    set16_fx(st_fx->lt_corr_pitch, 0, 3);
    st_fx->lt_hangover = 0;
    move16();
    st_fx->lowrate_pitchGain = 0;
    move16();


    st_fx->lt_music_hangover_fx = 0;
    move16();
    set16_fx(st_fx->tonality2_buf_fx,0,HANG_LEN_INIT);
    set16_fx(st_fx->tonality3_buf_fx,0,HANG_LEN_INIT);
    set16_fx(st_fx->LPCErr_buf_fx,0,HANG_LEN_INIT);
    st_fx->lt_music_state_fx    = 0;
    move16();
    st_fx->lt_speech_state_fx     = 0;
    move16();
    st_fx->lt_speech_hangover_fx  = 0;
    move16();
    st_fx->consec_inactive_fx = 0;
    move16();
    st_fx->spectral_tilt_reset_fx = 1;
    move16();
    st_fx->running_avg_fx = 0;
    move16();
    st_fx->ra_deltasum_fx = 0;
    move16();
    st_fx->trigger_SID_fx = 0;
    move16();
    st_fx->L_snr_sum_vad_fx = 0;
    move32();

    set16_fx( st_fx->prev_frm_index_fx, -1, NB_SWB_SUBBANDS_HAR_SEARCH_SB );
    st_fx->prev_frm_hfe2_fx = 0;
    move16();
    st_fx->prev_stab_hfe2_fx = 0;
    move16();
    st_fx->prev_ni_ratio_fx = 16384;
    move16(); /* 0.5 */
    set16_fx(st_fx->prev_En_sb_fx, 0, NB_SWB_SUBBANDS);
    set16_fx(st_fx->last_bitalloc_max_band_fx, 0, BANDS_MAX);
    set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
    set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
    st_fx->last_max_pos_pulse_fx = 0;
    move16();

    /*-----------------------------------------------------------------*
     * Channel-aware mode
     *-----------------------------------------------------------------*/


    test();
    test();
    test();
    IF( st_fx->Opt_RF_ON == 0 || (sub(st_fx->bwidth_fx,WB) != 0 && sub(st_fx->bwidth_fx,SWB) != 0) || L_sub(st_fx->total_brate_fx,ACELP_13k20) != 0 )
    {
        IF (sub(st_fx->Opt_RF_ON,1)==0 )
        {
            printf("\nWarning: Channel-aware mode only available for 13.2 kbps WB/SWB\n");
            printf("         Switched to normal mode!\n");
            st_fx->Opt_RF_ON = 0;
            move16();
        }
        st_fx->rf_mode = 0;
        move16();
    }
    ELSE
    {
        st_fx->rf_mode = st_fx->Opt_RF_ON;
        move16();
    }
    st_fx->rf_mode_last = st_fx->rf_mode;

    /* initialize RF indice buffers */
    st_fx->rf_frame_type = 0;
    move16();
    st_fx->rf_target_bits_write = 0;
    move16();
    {
        Word16 j;
        st_fx->rf_mem_w0 = 0;
        move16();
        set16_fx(st_fx->rf_clip_var, 0 ,6);
        st_fx->rf_tilt_code = 0;
        move16();
        set16_fx(st_fx->rf_mem_syn2, 0, M);
        st_fx->rf_dm_fx.prev_state = 0;
        move16();
        st_fx->rf_dm_fx.prev_gain_code = 0;
        move32();
        FOR(i=0; i<6; i++)
        {
            st_fx->rf_dm_fx.prev_gain_pit[i] = 0;
            move16();
        }
        st_fx->rf_gc_threshold = 0;
        move32();
        set16_fx(st_fx->rf_tilt_buf, 0, NB_SUBFR16k);

        st_fx->rf_target_bits = 0;
        move16();
        st_fx->rf_tcxltp_pitch_int_past = st_fx->L_frame_fx;
        move16();
        st_fx->rf_last_tns_active = 0;
        move16();
        st_fx->rf_second_last_tns_active = 0;
        move16();
        st_fx->rf_second_last_core = 0;
        move16();

        FOR( i = 0; i < MAX_RF_FEC_OFFSET; i++)
        {
            st_fx->rf_indx_frametype[i] = RF_NO_DATA;
            move16();
            st_fx->rf_targetbits_buff[i] = 6;
            move16();/* rf_mode: 1, rf_frame_type: 3, and fec_offset: 2 */
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

            st_fx->rf_clas[i] = UNVOICED_CLAS;
            move16();
            st_fx->rf_gain_tcx[i] = 0;
            move16();
            st_fx->rf_tcxltp_param[1] = 0;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * MODE2 initialization
     *-----------------------------------------------------------------*/

    st_fx->last_sr_core = i_mult2 (st_fx->last_L_frame_fx, 50);


    IF( sub(st_fx->codec_mode, MODE2) == 0 )
    {
        st_fx->igf = getIgfPresent( st_fx->total_brate_fx, st_fx->bwidth_fx, st_fx->rf_mode);
    }
    ELSE
    {
        st_fx->igf = 0;
        move16();
    }

    /* FD-CNG encoder */
    createFdCngEnc(&st_fx->hFdCngEnc_fx);
    initFdCngEnc(st_fx->hFdCngEnc_fx, st_fx->input_Fs_fx, st_fx->cldfbAna_Fx->scale);
    configureFdCngEnc( st_fx->hFdCngEnc_fx, st_fx->bwidth_fx, st_fx->rf_mode&&st_fx->total_brate_fx==13200?9600:st_fx->total_brate_fx );

    st_fx->last_totalNoise_fx = 0;
    move16();
    set16_fx( st_fx->totalNoise_increase_hist_fx, 0, TOTALNOISE_HIST_SIZE );
    st_fx->totalNoise_increase_len_fx = 0;
    move16();
    init_coder_ace_plus( st_fx, 0);

    InitTransientDetection( extract_l(Mult_32_16(st_fx->input_Fs_fx, 0x0290)),
                            NS2SA_fx2(st_fx->input_Fs_fx, DELAY_FIR_RESAMPL_NS),
                            &st_fx->transientDetection );

    reset_indices_enc_fx( st_fx );


    st_fx->Q_syn2 = 0;
    move16();
    st_fx->Q_syn = 0;
    move16();
    set16_fx(st_fx->Q_max, Q_MAX, L_Q_MEM);
    set16_fx(st_fx->Q_max_16k, Q_MAX, L_Q_MEM);
    st_fx->Q_old = 15;
    move16();
    st_fx->old_wsp_max = 0;
    move16();
    st_fx->old_wsp_shift = 0;
    move16();

    st_fx->vbr_generic_ho_fx = 0;
    move16();

    st_fx->sharpFlag = 0;
    move16();

    st_fx->Local_VAD = 0;

    set16_fx( st_fx->nelp_lp_fit_mem, 0, NELP_LP_ORDER*2 );

    return;
}

/*-----------------------------------------------------------------------*
 * destroy_encoder()
 *
 * Free memory which was allocated in init_encoder()
 *-----------------------------------------------------------------------*/

void destroy_encoder_fx(
    Encoder_State_fx *st_fx   /* i/o: Encoder static variables structure  */
)
{
    deleteCldfb( &st_fx->cldfbAna_Fx );
    deleteCldfb( &st_fx->cldfbSyn_Fx );

    deleteFdCngEnc( &st_fx->hFdCngEnc_fx );

    return;
}
