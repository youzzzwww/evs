/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * pre_proc()
 *
 * Pre-processing (spectral analysis, LP analysis, VAD,
 * OL pitch calculation, coder mode selection, ...)
 *--------------------------------------------------------------------*/

void pre_proc_fx(
    Encoder_State_fx *st,                    /* i/o: encoder state structure                  */
    const Word16 input_frame,              /* i  : frame length                             */
    const Word16 signal_in[],              /* i  : new samples                              */
    Word16 old_inp_12k8[],           /* i/o: buffer of old input signal               */
    Word16 old_inp_16k[],            /* i/o: buffer of old input signal @ 16kHz       */
    Word16 **inp,                    /* o  : ptr. to inp. signal in the current frame */
    Word16 *sp_aud_decision1,        /* o  : 1st stage speech/music classification    */
    Word16 *sp_aud_decision2,        /* o  : 2nd stage speech/music classification    */
    Word32 fr_bands[2*NB_BANDS],     /* o  : energy in frequency bands                */
    Word16 *vad_flag,
    Word16 *localVAD,
    Word16 *Etot,                    /* o  : total energy                             */
    Word32 *ener,                    /* o  : residual energy from Levinson-Durbin     */
    Word16 pitch[3],                 /* o  : open-loop pitch values for quantiz.      */
    Word16 voicing[3],               /* o  : OL maximum normalized correlation        */
    Word16 A[NB_SUBFR16k*(M+1)],     /* o  : A(z) unquantized for the 4 subframes     */
    Word16 Aw[NB_SUBFR16k*(M+1)],    /* o  : weighted A(z) unquantized for subframes  */
    Word16 epsP_h[M+1],              /* o  : LP prediction errors                     */
    Word16 epsP_l[M+1],              /* o  : LP prediction errors                     */
    Word32 epsP[M+1],                /* o  : LP prediction errors                     */
    Word16 lsp_new[M],               /* o  : LSPs at the end of the frame             */
    Word16 lsp_mid[M],               /* o  : LSPs in the middle of the frame          */
    Word16 *coder_type,              /* o  : coder type                               */
    Word16 *sharpFlag,               /* o  : formant sharpening flag                  */
    Word16 *vad_hover_flag,
    Word16 *attack_flag,             /* o  : flag signalling attack encoded by AC mode (GSC)    */
    Word16 *new_inp_resamp16k,       /* o  : new input signal @16kHz, non pre-emphasised, used by the WB TBE/BWE */
    Word16 *Voicing_flag,            /* o  : voicing flag for HQ FEC                  */

    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], /* o : cldfb real buffer */
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], /* o : cldfb imag buffer */
    CLDFB_SCALE_FACTOR *cldfbScale,                                 /* o : cldfb scale*/
    Word16 *old_exc,
    Word16 *hq_core_type,            /* o  : HQ core type                             */
    Word16 *Q_new,
    Word16 *shift,
    Word16 *Q_r
)
{
    Word16 delay;
    Word16 i;
    Word16 *inp_12k8, *new_inp_12k8, *inp_16k, *new_inp_16k;   /* pointers to current frame and new data */
    Word16 old_wsp[L_WSP], *wsp;                               /* weighted input signal buffer         */
    Word16 pitch_fr[NB_SUBFR];                                 /* fractional pitch values */
    Word16 voicing_fr[NB_SUBFR];                               /* fractional pitch gains               */
    Word32 lf_E[2*VOIC_BINS];                                  /* per bin spectrum energy in lf        */
    Word32 tmpN[NB_BANDS];                                     /* Temporary noise update               */
    Word32 tmpE[NB_BANDS];                                     /* Temporary averaged energy of 2 sf.   */
    Word32 ee[2];                                              /* Spectral tilt                        */
    Word16 corr_shift;                                         /* correlation shift                    */
    Word16 relE;                                               /* frame relative energy                */
    Word16 loc_harm;                                           /* harmonicity flag                     */
    Word16 cor_map_sum, sp_div;                                /* speech/music clasif. parameters      */
    Word32 PS[128];
    Word16 L_look;                                             /* length of look-ahead                 */

    Word16 Q_sp_div, Q_esp;
    Word16 localVAD_HE_SAD;                                     /* HE SAD parameters                    */
    Word16 snr_sum_he;                                          /* HE SAD parameters                    */

    Word16 vad_flag_cldfb;

    Word16 vad_flag_dtx;
    Word16 old_cor;
    Word16 uc_clas;
    Word32 hp_E[2];                                            /* Energy in HF                         */
    Word16 noisy_speech_HO, clean_speech_HO, NB_speech_HO;     /* SC-VBR HO flags                      */
    Word16 non_staX;                                           /* unbound non-stationarity for sp/mus clas. */
    Word32 sr_core_tmp;
    Word16 L_frame_tmp;
    Word16 Q_exp, Q_wsp_exp, Q_new_16k;
    Word16 shift_exp;
    Word16 Scale_fac[2];
    Word32 Le_min_scaled;
    Word16 excitation_max_test;
    Word16 lsf_new[M], stab_fac;
    Word32 enerBuffer[CLDFB_NO_CHANNELS_MAX];
    Word16 enerBuffer_exp; /*[CLDFB_NO_CHANNELS_MAX];*/
    Word16 realBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
    Word16 imagBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
    Word16 tmp_e;
    Word16 currFlatness;
    Word16 high_lpn_flag;
    Word16 cldfb_addition = add(0, 0);
    Word16 alw_pitch_lag_12k8[2];
    Word16 alw_voicing[2];
    Word16 flag_spitch;
    Word16 sf_energySum[CLDFB_NO_CHANNELS_MAX];
    Word32 L_tmp;
    UWord16 lsb;
    Word16 fft_buff[2*L_FFT];
    Word16 sp_floor;
    Word16 freqTable[2] = {20, 40};
    Word16 sp_aud_decision0;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    vad_flag_dtx = 0;
    move16();
    localVAD_HE_SAD = 0;
    move16();
    NB_speech_HO = 0;
    move16();
    clean_speech_HO = 0;
    move16();
    noisy_speech_HO = 0;
    move16();
    snr_sum_he = 0;
    move16();
    currFlatness = 0;
    move16();

    Q_new_16k = 0;
    move16();

    *vad_hover_flag = 0;
    move16();
    uc_clas = VOICED_CLAS;
    move16();
    *sp_aud_decision1 = 0;
    move16();
    *sp_aud_decision2 = 0;
    move16();
    *coder_type = GENERIC;
    move16();
    st->noise_lev_fx = NOISE_LEVEL_SP0;
    move16();
    *attack_flag = 0;
    move16();

    st->bump_up_fx = 0;
    move16();
    st->ppp_mode_fx = 0;
    move16();
    st->nelp_mode_fx = 0;
    move16();
    st->avoid_HQ_VBR_NB = 0;
    move16();

    L_look = L_LOOK_12k8;
    move16();           /* lookahead at 12.8kHz */

    new_inp_12k8 = old_inp_12k8 + L_INP_MEM;                    /* pointer to new samples of the input signal in 12.8kHz core */
    inp_12k8 = new_inp_12k8 - L_look;                           /* pointer to the current frame of input signal in 12.8kHz core */
    Copy( st->old_inp_12k8_fx, old_inp_12k8, L_INP_MEM );

    Copy( st->old_wsp_fx, old_wsp, L_WSP_MEM );
    wsp = old_wsp + L_WSP_MEM;                                  /* pointer to the current frame of weighted signal in 12.8kHz core */

    old_cor = st->old_corr_fx;
    move16();           /* save old_cor for speech/music classifier */

    st->rf_mode = st->Opt_RF_ON;

    /*--------------------------------------------------------------*
     * Cldfb analysis
     *---------------------------------------------------------------*/

    move32();
    move16();
    st->prevEnergyHF_fx = st->currEnergyHF_fx;
    tmp_e = st->currEnergyHF_e_fx;

    analysisCldfbEncoder_fx( st, signal_in, realBuffer, imagBuffer, realBuffer16, imagBuffer16, enerBuffer, &enerBuffer_exp, cldfbScale);
    cldfbScale->hb_scale = cldfbScale->lb_scale;

    /*----------------------------------------------------------------*
     * Change the sampling frequency to 12.8 kHz
     *----------------------------------------------------------------*/

    modify_Fs_fx( signal_in, input_frame, st->input_Fs_fx, new_inp_12k8, 12800, st->mem_decim_fx, (sub(st->max_bwidth_fx,NB) == 0) );
    Copy( new_inp_12k8, st->buf_speech_enc+L_FRAME32k, L_FRAME );
    Scale_sig( st->buf_speech_enc+L_FRAME32k, L_FRAME, 1 );
    /*------------------------------------------------------------------*
     * Perform fixed preemphasis (12.8 kHz signal) through 1 - g*z^-1
     *-----------------------------------------------------------------*/

    /* rf_mode: first time Q_new is computed here inside Preemph_scaled() for primary copy
                these are the same memories used in partial frame assembly as well */

    Preemph_scaled( new_inp_12k8, Q_new, &st->mem_preemph_fx, st->Q_max,
                    PREEMPH_FAC, 0, 1, L_Q_MEM, L_FRAME, st->last_coder_type_fx, 1 );

    Q_exp = sub(*Q_new, st->Q_old);
    st->prev_Q_old=st->Q_old;
    move16();
    st->Q_old = *Q_new;
    move16();

    /*------------------------------------------------------------------*
     * Scaling of memories
     *-----------------------------------------------------------------*/

    Le_min_scaled = Scale_mem_pre_proc( st->ini_frame_fx, Q_exp, Q_new, old_inp_12k8, &(st->mem_wsp_fx), st->enrO_fx, st->bckr_fx,
                                        st->ave_enr_fx, st->ave_enr2_fx, st->fr_bands1_fx, st->fr_bands2_fx, st->Bin_E_old_fx );

    /*-------------------------------------------------------------------------*
     * Spectral analysis
     *--------------------------------------------------------------------------*/

    analy_sp( inp_12k8, *Q_new, fr_bands, lf_E, Etot, st->min_band_fx, st->max_band_fx, Le_min_scaled, Scale_fac, st->Bin_E_fx, st->Bin_E_old_fx,
              PS, st->lgBin_E_fx, st->band_energies, fft_buff );

    st->band_energies_exp = sub(sub(WORD32_BITS-1,*Q_new),QSCALE);
    move16();

    /*----------------------------------------------------------------*
     * SAD (1-signal, 0-noise)
     *----------------------------------------------------------------*/

    noise_est_pre_fx( *Etot, st->ini_frame_fx,  &st->Etot_l_fx, &st->Etot_h_fx, &st->Etot_l_lp_fx, &st->Etot_last_fx,
                      &st->Etot_v_h2_fx, &st->sign_dyn_lp_fx, st->harm_cor_cnt_fx, &st->Etot_lp_fx );

    *vad_flag = wb_vad_fx( st, fr_bands, localVAD, &noisy_speech_HO, &clean_speech_HO, &NB_speech_HO,
                           &snr_sum_he, &localVAD_HE_SAD, &(st->flag_noisy_speech_snr), *Q_new );

    vad_flag_cldfb = vad_proc( &(st->vad_st),realBuffer, imagBuffer, cldfbScale->lb_scale, &cldfb_addition,
                               enerBuffer, enerBuffer_exp,st->cldfbAna_Fx->no_channels, *vad_flag );

    /* st->vad_without_hangover = *localVAD; */

    /* update wb_vad  primaryVAD  for the   CNG-SID  analysis */
    test();
    if( *vad_flag != 0 && vad_flag_cldfb == 0 )
    {
        *localVAD = 0;
        move16();
    }

    *vad_flag = vad_flag_cldfb;
    move16();

    /* apply DTX hangover for CNG analysis */
    vad_flag_dtx = dtx_hangover_addition_fx( st, *localVAD, *vad_flag, sub(st->lp_speech_fx,st->lp_noise_fx), cldfb_addition,  vad_hover_flag );

    /*----------------------------------------------------------------*
     * NB/WB/SWB/FB bandwidth detector
     *----------------------------------------------------------------*/

    FOR( i=0; i< CLDFB_NO_CHANNELS_MAX; i++ )
    {
        sf_energySum[i] = enerBuffer_exp;
        move16();
    }

    bw_detect_fx( st, signal_in, *localVAD, enerBuffer, sf_energySum );

    /*----------------------------------------------------------------*
     * Noise energy down-ward update and total noise energy estimation
     * Long-term energies and relative frame energy updates
     * Correlation correction as a function of total noise level
     *----------------------------------------------------------------*/

    noise_est_down_fx( fr_bands, st->bckr_fx,  tmpN, tmpE, st->min_band_fx, st->max_band_fx, &st->totalNoise_fx,
                       *Etot, &st->Etot_last_fx, &st->Etot_v_h2_fx,  *Q_new , Le_min_scaled );

    relE = sub(*Etot, st->lp_speech_fx); /* Q8 */    /* relE = *Etot - st->lp_speech;*/
    corr_shift = correlation_shift_fx( st->totalNoise_fx );

    /*----------------------------------------------------------------*
     * FD-CNG Noise Estimator
     *----------------------------------------------------------------*/

    test();
    IF( (L_sub(st->last_total_brate_fx,st->total_brate_fx) != 0) || (sub(st->last_bwidth_fx,st->bwidth_fx) != 0) )
    {
        configureFdCngEnc( st->hFdCngEnc_fx, st->bwidth_fx, (st->rf_mode && st->total_brate_fx==13200) ? 9600:st->total_brate_fx );
    }
    resetFdCngEnc ( st );
    perform_noise_estimation_enc ( st->band_energies, st->band_energies_exp, enerBuffer, enerBuffer_exp, st->hFdCngEnc_fx );

    /*-----------------------------------------------------------------*
     * Select SID or FRAME_NO_DATA frame if DTX enabled
     *-----------------------------------------------------------------*/

    dtx_fx(st, vad_flag_dtx, inp_12k8, *Q_new);

    /*----------------------------------------------------------------*
     * Adjust FD-CNG Noise Estimator
     *----------------------------------------------------------------*/
    test();
    IF ( st->hFdCngEnc_fx!=NULL && st->Opt_DTX_ON_fx )
    {
        AdjustFirstSID( st->hFdCngEnc_fx->hFdCngCom->npart, st->hFdCngEnc_fx->msPeriodog, st->hFdCngEnc_fx->msPeriodog_exp, st->hFdCngEnc_fx->energy_ho,
                        &st->hFdCngEnc_fx->energy_ho_exp,st->hFdCngEnc_fx->msNoiseEst, &st->hFdCngEnc_fx->msNoiseEst_exp, st->hFdCngEnc_fx->msNoiseEst_old,
                        &st->hFdCngEnc_fx->msNoiseEst_old_exp, &(st->hFdCngEnc_fx->hFdCngCom->active_frame_counter), st );
    }

    /*----------------------------------------------------------------*
    * Reconfigure MODE2
    *----------------------------------------------------------------*/

    IF ( sub(st->codec_mode,MODE2) == 0 )
    {
        SetModeIndex( st, st->total_brate_fx, st->bwidth_fx, *shift);
    }

    calcLoEnvCheckCorrHiLo_Fix( st->cldfbAna_Fx->no_col, freqTable, (st->tecEnc).loBuffer, (st->tecEnc).loTempEnv,
                                (st->tecEnc).loTempEnv_ns, (st->tecEnc).hiTempEnv, &((st->tecEnc).corrFlag) );
    /*---------------------------------------------------------------*
     * Time-domain transient detector
     *---------------------------------------------------------------*/

    /* Adjust prevEnergyHF and currEnergyHF to same exponent */
    i = sub(st->currEnergyHF_e_fx, tmp_e);

    /* If i > 0: currEnergyHF is higher => shr prevEnergyHF, exponent remains as is */
    st->prevEnergyHF_fx = L_shr(st->prevEnergyHF_fx, s_max(0, i));
    move32();

    /* If i < 0: currEnergyHF is lower => shr currEnergyHF, exponent changes to previous */
    st->currEnergyHF_fx = L_shl(st->currEnergyHF_fx, s_min(0, i));
    move32();

    if ( i < 0 )
    {
        st->currEnergyHF_e_fx = tmp_e;
        move16();
    }

    test();
    IF( st->tcx10Enabled || st->tcx20Enabled )
    {
        RunTransientDetection( signal_in, input_frame, &st->transientDetection);
        currFlatness = GetTCXAvgTemporalFlatnessMeasure( &st->transientDetection, NSUBBLOCKS, 0 );
    }

    /*----------------------------------------------------------------*
     * LP analysis
     *----------------------------------------------------------------*/

    alw_pitch_lag_12k8[0] = st->old_pitch_la;
    move16();
    alw_pitch_lag_12k8[1] = st->old_pitch_la;
    move16();
    alw_voicing[0] = st->old_voicing_la;
    move16();
    alw_voicing[1] = st->old_voicing_la;
    move16();

    analy_lp_fx( inp_12k8, L_FRAME, L_look, ener, A, epsP_h, epsP_l, lsp_new, lsp_mid, st->lsp_old1_fx, alw_pitch_lag_12k8, alw_voicing, 12800, *Q_new, Q_r );

    lsp2lsf_fx( lsp_new, lsf_new, M, INT_FS_FX );
    stab_fac = lsf_stab_fx( lsf_new, st->lsf_old1_fx, 0, L_FRAME );
    Copy( lsf_new, st->lsf_old1_fx, M );

    /*----------------------------------------------------------------*
     * Compute weighted input (for OL pitch analysis)
     * OL pitch analysis
     * 1/4 pitch precision improvement
     *----------------------------------------------------------------*/

    find_wsp( A, inp_12k8, wsp, &st->mem_wsp_fx, TILT_FAC_FX, L_FRAME, L_look, L_SUBFR, Aw, GAMMA1, NB_SUBFR );

    Q_wsp_exp = Q_exp;
    move16();
    Scale_wsp( wsp, &(st->old_wsp_max), shift, &Q_wsp_exp, &(st->old_wsp_shift), st->old_wsp2_fx,
               st->mem_decim2_fx, old_wsp, add(L_FRAME, L_look) );
    shift_exp=sub(Q_wsp_exp, Q_exp);

    IF( *vad_flag == 0 )
    {
        /* reset the OL pitch tracker memories during inactive frames */
        pitch_ol_init_fx( &st->old_thres_fx, &st->old_pitch, &st->delta_pit_fx, &st->old_corr_fx) ;
    }

    pitch_ol_fx( pitch, voicing, &st->old_pitch, &st->old_corr_fx, corr_shift, &st->old_thres_fx,
                 &st->delta_pit_fx, st->old_wsp2_fx, wsp, st->mem_decim2_fx, relE, st->clas_fx, st->bwidth_fx, st->Opt_SC_VBR_fx );

    /* Updates for adaptive lag window memory */
    st->old_pitch_la = pitch[2];
    move16();
    st->old_voicing_la = voicing[2];
    move16();

    /* Detection of very short stable pitch period (MODE1 bit-rates) */
    StableHighPitchDetect_fx( &flag_spitch, pitch, voicing, wsp, *localVAD, &st->voicing_sm_fx, &st->voicing0_sm_fx,
                              &st->LF_EnergyRatio_sm_fx, &st->predecision_flag_fx, &st->diff_sm_fx, &st->energy_sm_fx,*Q_new,st->lgBin_E_fx);

    /* 1/4 pitch precision improvement */
    IF( L_sub(st->total_brate_fx,ACELP_24k40) <= 0 )
    {
        /* 1/4 pitch precision improvement */
        pitch_ol2_fx( PIT_MIN_EXTEND, pitch[0], &pitch_fr[0], &voicing_fr[0], 0, wsp, 7 );
        pitch_ol2_fx( PIT_MIN_EXTEND, pitch[0], &pitch_fr[1], &voicing_fr[1], L_SUBFR, wsp, 7 );
        pitch_ol2_fx( PIT_MIN_EXTEND, pitch[1], &pitch_fr[2], &voicing_fr[2], 2*L_SUBFR, wsp, 7 );
        pitch_ol2_fx( PIT_MIN_EXTEND, pitch[1], &pitch_fr[3], &voicing_fr[3], 3*L_SUBFR, wsp, 7 );
    }
    ELSE
    {
        pitch_fr[0] = pitch[0];
        move16();
        pitch_fr[1] = pitch[0];
        move16();
        pitch_fr[2] = pitch[1];
        move16();
        pitch_fr[3] = pitch[1];
        move16();

        voicing_fr[0] = voicing[0];
        move16();
        voicing_fr[1] = voicing[0];
        move16();
        voicing_fr[2] = voicing[1];
        move16();
        voicing_fr[3] = voicing[1];
        move16();
    }

    /*------------------------------------------------------------------*
     * Update estimated noise energy and voicing cut-off frequency
     *-----------------------------------------------------------------*/

    noise_est_fx( st, tmpN, pitch, voicing, epsP_h,epsP_l, *Etot, relE, corr_shift, tmpE, fr_bands, &cor_map_sum,
                  &sp_div, &Q_sp_div, &non_staX , &loc_harm, lf_E, &st->harm_cor_cnt_fx ,st->Etot_l_lp_fx,
                  st->Etot_v_h2_fx ,&st->bg_cnt_fx, st->lgBin_E_fx,*Q_new, Le_min_scaled, &sp_floor );

    /*------------------------------------------------------------------*
     * Update parameters used in the VAD and DTX
     *-----------------------------------------------------------------*/

    vad_param_updt_fx( st, pitch, voicing, corr_shift, *vad_flag, A );

    /*-----------------------------------------------------------------*
     * Find spectral tilt
     * UC and VC frame selection
     *-----------------------------------------------------------------*/

    find_tilt_fx( fr_bands, st->bckr_fx, ee, pitch, voicing, lf_E, corr_shift, st->input_bwidth_fx,
                  st->max_band_fx, hp_E, st->codec_mode, *Q_new, &(st->bckr_tilt_lt), st->Opt_SC_VBR_fx );

    *coder_type = find_uv_fx( st, pitch_fr, voicing_fr, voicing, inp_12k8, *localVAD, ee, corr_shift,
                              relE, *Etot, hp_E, *Q_new, &flag_spitch, st->voicing_sm_fx );

    /*----------------------------------------------------------------*
    * channel aware mode configuration                                *
    *-----------------------------------------------------------------*/
    test();
    test();
    test();
    IF( !st->Opt_RF_ON)
    {
        st->rf_mode = 0;
        st->rf_target_bits_write = 0;
    }
    ELSE IF( st->rf_mode && L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st->core_brate_fx,SID_2k40) != 0 )
    {
        /* the RF config is for (n- fec_offset)th frame that will be packed along with the n-th frame bistream */
        st->rf_mode = 1;
        st->codec_mode = MODE2;

        st->rf_target_bits_write = st->rf_targetbits_buff[st->rf_fec_offset];
    }
    ELSE
    {
        st->rf_mode = 0;
        st->codec_mode = MODE1;
        st->rf_indx_frametype[0] = RF_NO_DATA;
        st->rf_targetbits_buff[0] = 6;  /* rf_mode: 1, rf_frame_type: 3, and fec_offset: 2 */
    }

    /*-----------------------------------------------------------------*
     * Signal classification for FEC
     * TC frame selection
     *-----------------------------------------------------------------*/

    st->clas_fx = signal_clas_fx( st, coder_type, voicing, inp_12k8, *localVAD, pitch, ee, relE, L_look, &uc_clas);

    st->Local_VAD = *localVAD;

    /*----------------------------------------------------------------*
     * Speech/music classification
     * AC frame selection
     *----------------------------------------------------------------*/

    FOR( i=0; i<M+1; i++ )
    {
        epsP[i] = L_Comp( epsP_h[i],epsP_l[i] );
        move32();
    }

    Q_esp = add(2*(*Q_new),add(Q_r[0],1));

    speech_music_classif_fx( st,
                             &sp_aud_decision0,
                             sp_aud_decision1, sp_aud_decision2, new_inp_12k8, inp_12k8, *vad_flag,
                             *localVAD, localVAD_HE_SAD, pitch, voicing, lsp_new, cor_map_sum, epsP, PS,
                             *Etot, old_cor, coder_type, attack_flag, non_staX, relE, Q_esp, *Q_new, &high_lpn_flag, flag_spitch);

    long_enr_fx( st,  *Etot, localVAD_HE_SAD , high_lpn_flag );  /* has to be after  after sp_music classfier */

    /*----------------------------------------------------------------*
     * Rewrite the VAD flag by VAD flag  with DTX hangover for further processing)
     *----------------------------------------------------------------*/

    if( st->Opt_DTX_ON_fx )
    {
        *vad_flag = vad_flag_dtx;
        move16();  /* flag now with the DTX-HO for use in further high rate encoding below  */
    }

    /*----------------------------------------------------------------*
     * Selection of internal ACELP Fs (12.8 kHz or 16 kHz)
     *----------------------------------------------------------------*/

    IF( sub(st->codec_mode,MODE1) == 0 )
    {
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) == 0 )
        {
            /* prevent "L_frame" changes in CNG segments */
            st->L_frame_fx = st->last_L_frame_fx;
            move16();
        }
        ELSE IF ( L_sub(st->core_brate_fx,SID_2k40) == 0 && st->first_CNG_fx && sub(st->act_cnt2_fx,MIN_ACT_CNG_UPD) < 0 )
        {
            /* prevent "L_frame" changes in SID frame after short segment of active frames */
            st->L_frame_fx = st->last_CNG_L_frame_fx;
            move16();
        }
        ELSE IF ( ( L_sub(st->core_brate_fx,SID_2k40) == 0 && L_sub(st->total_brate_fx,ACELP_9k60) >= 0 && ((sub(st->bwidth_fx,WB) == 0 && !( L_sub(st->total_brate_fx,ACELP_13k20) == 0 && sub(st->cng_type_fx,FD_CNG) == 0)) || (sub(st->cng_type_fx,LP_CNG) == 0 && sub(st->bwidth_fx,WB) > 0 && L_sub(st->total_brate_fx,ACELP_16k40) >= 0)) ) ||
                  ( L_sub(st->total_brate_fx,ACELP_24k40) > 0 && L_sub(st->total_brate_fx,HQ_96k) < 0 ) || ( L_sub(st->total_brate_fx,ACELP_24k40) == 0 && sub(st->bwidth_fx,WB) >= 0 ) )
        {
            st->L_frame_fx = L_FRAME16k;
            move16();
        }
        ELSE
        {
            st->L_frame_fx = L_FRAME;
            move16();
        }

        if( st->ini_frame_fx == 0 )
        {
            /* avoid switching of internal ACELP Fs in the very first frame */
            st->last_L_frame_fx = st->L_frame_fx;
            move16();
        }

        IF( sub(st->L_frame_fx,L_FRAME) == 0 )
        {
            st->gamma = GAMMA1;
            move16();
            st->preemph_fac = PREEMPH_FAC;
            move16();
        }
        ELSE
        {
            st->gamma = GAMMA16k;
            move16();
            st->preemph_fac = PREEMPH_FAC_16k;
            move16();
        }

        st->sr_core = L_mult0(50,st->L_frame_fx);
        st->encoderLookahead_enc = NS2SA_fx2(st->sr_core, ACELP_LOOK_NS);
        move16();
        st->encoderPastSamples_enc = shr(imult1616(st->L_frame_fx, 9), 4);
    }

    /*-----------------------------------------------------------------*
     * coder_type rewriting in case of switching
     * IC frames selection
     * enforce TC frames in case of switching
     *-----------------------------------------------------------------*/

    IF( sub(st->codec_mode,MODE1) == 0 )
    {
        /* enforce TRANSITION frames */
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( sub(st->last_L_frame_fx,st->L_frame_fx) != 0 && L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st->core_brate_fx,SID_2k40) != 0 && (sub(st->coder_type_raw_fx,VOICED) != 0) )
        {
            /* enforce TC frame in case of ACELP@12k8 <-> ACELP@16k core switching */
            *coder_type = TRANSITION;
            move16();
        }
        ELSE IF( sub(st->last_core_fx,HQ_CORE) == 0 || sub(st->last_core_fx,TCX_10_CORE) == 0 || sub(st->last_core_fx,TCX_20_CORE) == 0 )
        {
            /* enforce TC frame in case of HQ/TCX -> ACELP core switching */
            *coder_type = TRANSITION;
            move16();
        }
        ELSE IF( L_sub(st->last_core_brate_fx,SID_2k40) <= 0 && sub(st->cng_type_fx,FD_CNG) == 0 )
        {
            /* enforce TC frame in case of FD_CNG -> ACELP switching (past excitation not available) */
            *coder_type = TRANSITION;
            move16();
        }

        /* select INACTIVE frames */
        ELSE IF( L_sub(st->total_brate_fx,ACELP_24k40) <= 0 && *vad_flag == 0 )
        {
            /* inactive frames will be coded by GSC technology */
            /* except for the VBR mode. VBR mode uses NELP for that */
            test();
            IF ( !( sub(st->Opt_SC_VBR_fx, 1) == 0 &&  sub(vad_flag_dtx, 1 ) == 0 ) )
            {
                *coder_type = INACTIVE;
                move16();
                st->noise_lev_fx = NOISE_LEVEL_SP3;
                move16();
            }
        }
        ELSE IF( L_sub(st->total_brate_fx,ACELP_24k40) > 0 &&
                 ( (*vad_flag == 0 && st->bwidth_fx >= SWB && st->max_bwidth_fx >= SWB) || (*localVAD == 0 && (st->bwidth_fx <= WB || st->max_bwidth_fx <= WB)) )
               )
        {
            /* inactive frames will be coded by AVQ technology */
            *coder_type = INACTIVE;
            move16();
        }
    }
    ELSE
    {
        IF( !(*vad_flag) )
        {
            *coder_type = INACTIVE;
            move16();
        }
        ELSE IF( sub(*coder_type,GENERIC) > 0 )
        {
            *coder_type = GENERIC;
            move16();
        }
    }

    /*---------------------------------------------------------------*
     * SC-VBR - decision about PPP/NELP mode
     *---------------------------------------------------------------*/

    IF( st->Opt_SC_VBR_fx )
    {
        set_ppp_mode_fx( st, coder_type, noisy_speech_HO, clean_speech_HO, NB_speech_HO, *localVAD, localVAD_HE_SAD, vad_flag, pitch, *sp_aud_decision1 );
    }
    test();
    IF ( !st->Opt_AMR_WB_fx && !st->rf_mode )
    {
        test();
        test();
        IF ( L_sub(st->total_brate_fx,ACELP_13k20) == 0 || L_sub(st->total_brate_fx,ACELP_32k) == 0 )
        {
            st->mdct_sw_enable = MODE1;
            move16();
        }
        ELSE IF ( L_sub(ACELP_16k40,st->total_brate_fx) <= 0 && L_sub(st->total_brate_fx,ACELP_24k40) <= 0)
        {
            st->mdct_sw_enable = MODE2;
            move16();
        }
    }

    IF( sub(st->codec_mode,MODE1) == 0 )
    {
        /*---------------------------------------------------------------------*
         * Decision matrix (selection of technologies)
         *---------------------------------------------------------------------*/

        decision_matrix_enc_fx( st, *sp_aud_decision1, *sp_aud_decision2, *coder_type, *vad_flag, hq_core_type );

        /* HQ_CORE/TCX_20_CORE decision */
        IF ( sub(st->core_fx,HQ_CORE) == 0 ) /* Decision matrix decided for MDCT coding */
        {
            test();
            test();
            IF( (sub(st->bwidth_fx,SWB) == 0 || sub(st->bwidth_fx,FB) == 0) && L_sub(st->total_brate_fx,32000) == 0 )
            {
                /* Select MDCT Core */
                st->core_fx = mdct_classifier_fx(fft_buff,st,*vad_flag, enerBuffer);
            }
            test();
            IF( (L_sub(st->total_brate_fx,13200) == 0) && (sub(st->bwidth_fx,FB) != 0 ))
            {
                MDCT_selector( st, sp_floor, *Etot, cor_map_sum, voicing, enerBuffer, enerBuffer_exp, *vad_flag );
            }
        }
        ELSE
        {
            MDCT_selector_reset( st );
        }

        /* Switch to MODE2 if TCX_20_CORE */
        IF( sub(st->core_fx,TCX_20_CORE) == 0 )
        {
            st->codec_mode = MODE2;
            move16();
            IF( sub(st->last_codec_mode,MODE1) == 0 )
            {
                Word32 last_total_brate = L_add(st->last_total_brate_fx, 0);
                st->last_total_brate_fx = -1;
                move32();
                SetModeIndex( st, st->total_brate_fx, st->bwidth_fx, *shift );
                st->last_total_brate_fx = last_total_brate;
                move32();
            }
            ELSE
            {
                SetModeIndex( st, st->total_brate_fx, st->bwidth_fx, *shift);
                st->sr_core = getCoreSamplerateMode2( st->total_brate_fx, st->bwidth_fx, st->rf_mode);

                Mpy_32_16_ss(st->sr_core, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
                st->L_frame_fx = extract_l(L_shr(L_tmp, 3)); /* Q0 */
                st->encoderLookahead_enc = NS2SA_fx2(st->sr_core, ACELP_LOOK_NS);
                move16();
                st->encoderPastSamples_enc = shr(imult1616(st->L_frame_fx, 9), 4);
                assert(st->L_frame_fx == st->sr_core / 50);

                IF ( L_sub(st->sr_core,12800) == 0 )
                {
                    st->preemph_fac = PREEMPH_FAC;
                    move16();
                    st->gamma = GAMMA1;
                    move16();
                }
                ELSE
                {
                    st->preemph_fac = PREEMPH_FAC_16k;
                    move16();
                    st->gamma = GAMMA16k;
                    move16();
                }


                st->igf = getIgfPresent(st->total_brate_fx, st->bwidth_fx, st->rf_mode);
            }

            *coder_type = st->coder_type_raw_fx;
            move16();

            IF( *vad_flag == 0 )
            {
                *coder_type = INACTIVE;
                move16();
            }
            ELSE IF( sub((*coder_type),GENERIC) > 0 )
            {
                *coder_type = GENERIC;
                move16();
            }

            st->mdct_sw = MODE1;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Update of ACELP harmonicity counter (used in ACELP transform codebook @32kbps)
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    IF( L_sub(st->total_brate_fx, ACELP_32k) == 0 && sub(loc_harm,1) == 0 && sub(cor_map_sum,50<<8) > 0
        && sub(st->clas_fx, VOICED_CLAS)== 0 && sub(*coder_type,GENERIC) == 0 )
    {
        st->last_harm_flag_acelp_fx  = add(st->last_harm_flag_acelp_fx,1);
        st->last_harm_flag_acelp_fx = s_min(st->last_harm_flag_acelp_fx,10);
    }
    ELSE
    {
        st->last_harm_flag_acelp_fx = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Update audio frames counter (used for UV decision)
     *-----------------------------------------------------------------*/

    IF( sub(*coder_type,AUDIO) == 0 )
    {
        st->audio_frame_cnt_fx = add(st->audio_frame_cnt_fx,AUDIO_COUNTER_STEP);
    }
    ELSE IF (sub(*coder_type,INACTIVE) != 0)
    {
        st->audio_frame_cnt_fx = sub(st->audio_frame_cnt_fx,1);
    }

    st->audio_frame_cnt_fx = s_min(st->audio_frame_cnt_fx,AUDIO_COUNTER_MAX);
    st->audio_frame_cnt_fx = s_max(st->audio_frame_cnt_fx,0);

    /*-----------------------------------------------------------------*
     * Set formant sharpening flag
     *-----------------------------------------------------------------*/

    *sharpFlag = 0;
    move16();
    IF( sub(*coder_type,TRANSITION) == 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF( ( L_sub(st->total_brate_fx,ACELP_48k) > 0 && sub(st->bwidth_fx,SWB) < 0 ) ||                                     /* Deactivate for core bitrates higher than 48.0 kb/s */
            ( L_sub(st->total_brate_fx,ACELP_13k20) >= 0 && L_sub(st->total_brate_fx,ACELP_16k40) <= 0 ) ||                    /* Deactivate for bitrates <13.2, 16.4> kb/s (this is basically due to lack of signaling configurations */
            ( L_sub(st->total_brate_fx,ACELP_16k40) > 0 && sub(st->lp_noise_fx,FORMANT_SHARPENING_NOISE_THRESHOLD_FX  ) > 0 )  )  /* Deactivate for bitrates >= 24.4 kb/s if the long-term noise level exceeds 34 dB */
        {
            *sharpFlag= 0;
            move16();
        }
        ELSE
        {
            *sharpFlag= 1;
            move16();
        }
    }

    test();
    IF( sub(*coder_type,GENERIC) == 0 || sub(*coder_type,VOICED) == 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF( *vad_hover_flag ||
            ( L_sub(st->total_brate_fx,ACELP_48k) > 0 && sub(st->bwidth_fx,SWB) < 0 ) ||               /* Deactivate for core bitrates higher than 48.0 kb/s */
            ( L_sub(st->total_brate_fx,ACELP_13k20) >= 0 && sub(st->lp_noise_fx,FORMANT_SHARPENING_NOISE_THRESHOLD_FX) > 0   /* Deactivate for bitrates >= 13.2 kb/s if the long-term noise level exceeds 34 dB */
              && L_sub(st->total_brate_fx,CNA_MAX_BRATE) > 0 ) )
        {
            *sharpFlag = 0;
            move16();
        }
        ELSE
        {
            *sharpFlag = 1;
            move16();
        }
    }

    /* channel-aware mode - due to lack of signalling bit, sharpFlag is 1 always in RF mode */
    test();
    test();
    IF( sub(st->rf_mode,1)==0 && ( sub(*coder_type,VOICED) == 0 || sub(*coder_type,GENERIC) == 0 ) )
    {
        *sharpFlag = 1;
    }

    /*-----------------------------------------------------------------*
     * Set voicing flag for HQ FEC
     *-----------------------------------------------------------------*/

    *Voicing_flag = 0;
    move16();
    test();
    test();
    if ( *sp_aud_decision1 == 0 && ( sub(*coder_type,VOICED) == 0 || sub(*coder_type,GENERIC) == 0 ) )
    {
        *Voicing_flag = 1;
        move16();
    }

    /*---------------------------------------------------------------*
     * Preprocessing at other sampling frequency rate (16/25.6/32kHz)
     *----------------------------------------------------------------*/

    move16();
    sr_core_tmp = (sub(st->codec_mode,MODE1) == 0)? 16000 : L_max(16000,st->sr_core);      /* indicates the ACELP sampling rate for MODE2, 16 kHz for MODE1 */
    move16();
    L_frame_tmp = (sub(st->codec_mode,MODE1) == 0)? L_FRAME16k : s_max(L_FRAME16k,st->L_frame_fx);

    L_look = NS2SA_fx2(sr_core_tmp, ACELP_LOOK_NS);
    move16();      /* lookahead at other sampling rate (16kHz, 25.5kHz, 32kHz) */

    new_inp_16k = old_inp_16k + L_INP_MEM;                  /* pointer to new samples of the input signal in 16kHz core */
    inp_16k = new_inp_16k - L_look;                         /* pointer to the current frame of input signal in 16kHz core */
    Copy( st->old_inp_16k_fx, old_inp_16k, L_INP_MEM );     /* Note: The merory has been rescaled at the begining of the function*/

    /*---------------------------------------------------------------*
     * Change the sampling frequency to 16/25.6/32 kHz
     *----------------------------------------------------------------*/

    test();
    IF( L_sub(st->input_Fs_fx,sr_core_tmp) == 0 )
    {
        /* no resampling needed, only delay adjustement to account for the FIR resampling delay */
        delay = NS2SA_fx2(st->input_Fs_fx, DELAY_FIR_RESAMPL_NS);
        Copy_Scale_sig( st->mem_decim16k_fx + delay, new_inp_16k, delay, -1 );      /* Input in Q0 -> Output in Q-1 to mimic the resampling filter */
        Copy_Scale_sig( signal_in, new_inp_16k + delay, input_frame - delay, -1 );  /* Input in Q0 -> Output in Q-1 to mimic the resampling filter */
        Copy( signal_in + input_frame - shl(delay,1), st->mem_decim16k_fx, shl(delay,1) );  /* memory still in Q0 */
    }
    ELSE IF( L_sub(st->input_Fs_fx,32000) == 0 || L_sub(st->input_Fs_fx,48000) == 0 )
    {
        modify_Fs_fx( signal_in, input_frame, st->input_Fs_fx, new_inp_16k, sr_core_tmp, st->mem_decim16k_fx, 0 );
    }
    ELSE    /* keep memories up-to-date in case of bit-rate switching */
    {
        /* no resampling needed, only delay adjustement to account for the FIR resampling delay */
        delay = NS2SA_fx2(st->input_Fs_fx, DELAY_FIR_RESAMPL_NS);
        Copy( st->mem_decim16k_fx + delay, new_inp_16k, delay );
        Copy( signal_in, new_inp_16k + delay, sub(input_frame, delay) );
        Copy( signal_in + sub(input_frame,  shl(delay,1)), st->mem_decim16k_fx, shl(delay,1) );
    }

    IF( L_sub(sr_core_tmp,16000) == 0 )
    {
        /* save input resampled at 16kHz, non-preemhasised */
        Copy( new_inp_16k, new_inp_resamp16k, L_FRAME16k );
    }
    ELSE IF( L_sub(sr_core_tmp,16000) > 0 )
    {
        /* reset the buffer, the signal is needed for WB BWEs */
        set16_fx( new_inp_resamp16k, 0, L_FRAME16k );
    }

    /*------------------------------------------------------------------*
     * Perform fixed preemphasis (16kHz signal) through 1 - g*z^-1
     *-----------------------------------------------------------------*/

    test();
    test();
    IF( ((st->tcxonly == 0) ||  (sub(st->codec_mode,MODE1) == 0)) && L_sub(st->input_Fs_fx,8000) > 0 )
    {
        st->mem_preemph_enc = shl(new_inp_16k[sub(L_frame_tmp,1)],1);
    }

    test();
    IF( L_sub(st->input_Fs_fx,8000) > 0 && L_sub(sr_core_tmp,16000) == 0)
    {
        Preemph_scaled( new_inp_16k, &Q_new_16k, &(st->mem_preemph16k_fx), st->Q_max_16k, PREEMPH_FAC_16k, 0, 1, L_Q_MEM, L_FRAME16k, st->last_coder_type_fx, 1);
    }
    ELSE IF( L_sub(st->input_Fs_fx,8000) > 0 )    /* keep memory up-to-date in case of bit-rate switching */
    {
        st->mem_preemph16k_fx = new_inp_16k[sub(L_frame_tmp,1)];
        move16();
    }

    /*------------------------------------------------------------------*
     * Core-encoder memories scaling
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    test();
    IF( ( ((st->tcxonly == 0) || !(L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 || L_sub(st->core_brate_fx,SID_2k40) != 0)) && sub(st->L_frame_fx,L_FRAME16k) == 0 && sub(st->codec_mode,MODE2) == 0 ) ||
        ( sub(st->L_frame_fx,L_FRAME16k) == 0 && sub(st->codec_mode,MODE1) == 0 ) )
    {
        *Q_new = Q_new_16k;
        move16();
    }
    ELSE
    {
        IF( L_sub(st->input_Fs_fx,8000) > 0 && L_sub(sr_core_tmp,16000) == 0 )
        {
            Scale_sig(new_inp_16k, L_FRAME16k, sub(*Q_new,Q_new_16k));
        }
    }

    /* Above computed Q_new is used to scale primary copy exc and memory here by (Q_new, st->prev_Q_new) */

    Q_exp = sub(*Q_new, st->prev_Q_new);
    move16();
    Q_wsp_exp = add(Q_exp,shift_exp);

    Scale_mem_enc( Q_exp, old_inp_16k, old_exc, st->old_bwe_exc_fx, &(st->LPDmem.mem_w0), st->LPDmem.mem_syn,
                   st->LPDmem.mem_syn2, &st->mem_deemp_preQ_fx, st->last_exc_dct_in_fx, st->old_input_lp_fx );

    /*-----------------------------------------------------------------*
     * Redo LP analysis at 16kHz if ACELP@16k core was selected
     * update buffers
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    test();
    IF( ( ((st->tcxonly == 0) || !(L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 || L_sub(st->core_brate_fx,SID_2k40) != 0)) && sub(st->L_frame_fx,L_FRAME16k) == 0 && sub(st->codec_mode,MODE2) == 0 ) ||
        ( sub(st->L_frame_fx,L_FRAME16k) == 0 && sub(st->codec_mode,MODE1) == 0 ) )
    {
        /* update signal buffers */
        Copy( new_inp_resamp16k, st->buf_speech_enc+L_FRAME16k, L_FRAME16k );
        Scale_sig( st->buf_speech_enc+L_FRAME16k, L_FRAME16k, 1 );
        Copy( new_inp_16k, st->buf_speech_enc_pe+L_FRAME16k, L_FRAME16k );
        IF( Q_exp != 0 )
        {
            Scale_sig(st->buf_speech_enc_pe, st->encoderPastSamples_enc+st->encoderLookahead_enc, Q_exp);
            Scale_sig(&(st->mem_wsp_enc), 1, Q_exp);
        }

        /*--------------------------------------------------------------*
         * LPC analysis
         *---------------------------------------------------------------*/

        test();
        IF( sub(st->last_L_frame_fx,L_FRAME) == 0 && sub(st->codec_mode,MODE1) == 0 )
        {
            /* this is just an approximation, but it is sufficient */
            Copy( st->lsp_old1_fx, st->lspold_enc_fx, M );
        }

        analy_lp_fx( inp_16k, L_FRAME16k, L_look, ener, A, epsP_h, epsP_l, lsp_new, lsp_mid, st->lspold_enc_fx, pitch, voicing, 16000, *Q_new, Q_r );

        /*--------------------------------------------------------------*
        * Compute Weighted Input
        *---------------------------------------------------------------*/

        IF( sub(st->codec_mode,MODE2) == 0 )
        {
            find_wsp( A, st->speech_enc_pe, st->wspeech_enc, &st->mem_wsp_enc, PREEMPH_FAC_16k, L_FRAME16k, L_LOOK_16k, L_SUBFR, Aw, st->gamma, st->nb_subfr);

            /* This need to be harmonized with MODE2 */
            Scale_sig( st->wspeech_enc, L_FRAME16k+L_LOOK_16k, *shift );

        }
        ELSE
        {
            weight_a_subfr_fx( NB_SUBFR16k, A, Aw, GAMMA16k, M );
        }
    }
    ELSE
    {
        /* update signal buffers */
        Copy( new_inp_12k8, st->buf_speech_enc_pe+st->L_frame_fx, L_FRAME );
        Copy( st->buf_speech_enc+L_FRAME32k, st->buf_speech_enc+st->L_frame_fx, L_FRAME );

        if ( st->tcxonly == 0 )
        {
            Copy( wsp, st->buf_wspeech_enc+st->L_frame_fx+L_SUBFR, L_FRAME + L_LOOK_12k8 );
        }
        test();
        test();
        IF( sub(st->codec_mode,MODE2) == 0 && st->tcxonly == 0 && Q_exp != 0 )
        {
            Scale_sig( st->buf_speech_enc_pe, st->encoderPastSamples_enc+st->encoderLookahead_enc, Q_exp );
            Scale_sig( &(st->mem_wsp_enc), 1, Q_exp );
        }
    }

    excitation_max_test = -32768;
    move16();
    FOR( i = 0; i < L_EXC_MEM; i++ )
    {
        excitation_max_test = s_max(abs_s(old_exc[i]),excitation_max_test);
    }

    test();
    IF( sub(excitation_max_test,8192) > 0 && *shift == 0 )
    {
        excitation_max_test = 1;
        move16();
        *shift = -1;
        move16();
        st->old_wsp_shift=-1;
        move16();
        Scale_sig( old_wsp, L_WSP_MEM+L_FRAME+L_LOOK_12k8, -1 );
    }
    ELSE
    {
        excitation_max_test = 0;
        move16();
    }

    test();
    IF ( sub(st->codec_mode,MODE2) == 0 && st->tcxonly == 0 )
    {
        IF (Q_wsp_exp != 0)
        {
            Scale_sig(st->buf_wspeech_enc, st->L_frame_fx+L_SUBFR, Q_wsp_exp);
        }
        IF( sub(excitation_max_test,1) == 0 )
        {
            Scale_sig( st->buf_wspeech_enc, st->L_frame_fx+L_SUBFR+st->L_frame_fx+st->encoderLookahead_enc, -1 );
        }
    }

    /*-----------------------------------------------------------------*
     * ACELP/TCX20/HQ Switching Decision
     *-----------------------------------------------------------------*/

    IF ( sub(st->codec_mode,MODE2) == 0 )
    {
        test();
        test();
        IF((L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st->core_brate_fx,SID_2k40) != 0 && st->tcxonly == 0 ))
        {
            core_acelp_tcx20_switching( st,*vad_flag,
                                        sp_aud_decision0, non_staX,
                                        pitch, pitch_fr, voicing_fr, currFlatness, lsp_mid, stab_fac, *Q_new, *shift );
        }

        test();
        IF (sub(st->mdct_sw_enable,MODE2) == 0 && !st->rf_mode)
        {
            IF (sub(st->core_fx,TCX_20_CORE) == 0) /* Switching only possible from TCX_20 frames, not from TCX_10 frames */
            {
                /* Select MDCT Core */
                test();
                test();
                IF ((sub(st->bwidth_fx,SWB)==0 || sub(st->bwidth_fx,FB)==0) && L_sub(st->total_brate_fx,24400)==0)
                {
                    st->core_fx = mdct_classifier_fx(fft_buff,st,*vad_flag, enerBuffer);
                }
                test();
                IF ((L_sub(st->total_brate_fx,16400) == 0) && (sub(st->bwidth_fx,FB) !=0 ))
                {
                    MDCT_selector( st, sp_floor, *Etot, cor_map_sum, voicing, enerBuffer, enerBuffer_exp, *vad_flag );
                }
            }
            ELSE
            {
                MDCT_selector_reset( st );
            }

            /* Do the switching that was decided in the MDCT selector */
            test();
            IF( sub(st->core_fx,HQ_CORE) == 0 )
            {
                st->codec_mode = MODE1;
                move16();
                st->mdct_sw = MODE2;
                move16();
            }
            ELSE IF( sub(st->last_codec_mode,MODE1) == 0 && sub(st->last_core_fx,HQ_CORE) == 0 )
            {
                Word16 L_frame_old = st->last_L_frame_fx;
                move16();
                st->last_L_frame_fx = st->L_frame_fx;
                move16();
                SetModeIndex( st, st->total_brate_fx, st->bwidth_fx, *shift );
                st->last_L_frame_fx = L_frame_old;
                move16();
            }
        }

        /*--------------------------------------------------------------*
         * TCX mode decision
         *---------------------------------------------------------------*/

        SetTCXModeInfo( st, &st->transientDetection, &st->tcx_cfg.tcx_curr_overlap_mode );
    }

    /*-----------------------------------------------------------------*
     * Updates
     *-----------------------------------------------------------------*/

    /* update old weighted speech buffer - for OL pitch analysis */
    Copy( &old_wsp[L_FRAME], st->old_wsp_fx, L_WSP_MEM );

    /* update old input signal buffer */
    Copy( &old_inp_12k8[L_FRAME], st->old_inp_12k8_fx, L_INP_MEM );

    /* update old input signal @16kHz buffer */
    test();
    IF( L_sub(st->input_Fs_fx,8000)  > 0 && L_sub(sr_core_tmp,16000) == 0 )
    {
        Copy( &old_inp_16k[L_frame_tmp], st->old_inp_16k_fx, L_INP_MEM );
    }
    ELSE IF( L_sub(st->input_Fs_fx,8000) > 0 )
    {
        lerp( st->old_inp_12k8_fx+L_INP_MEM-L_INP_MEM*4/5, st->old_inp_16k_fx, L_INP_MEM, L_INP_MEM*4/5);
        Scale_sig(st->old_inp_16k_fx, L_INP_MEM, sub(*Q_new,st->Q_old));
    }

    test();
    test();
    IF( (L_sub(sr_core_tmp,16000) == 0) && st->tcxonly && sub(st->codec_mode,MODE2) == 0 )
    {
        /* copy input resampled at 16kHz, non-preemhasised */
        Copy( new_inp_resamp16k, new_inp_16k, L_FRAME16k );
    }

    /* update of old per-band energy spectrum */
    Copy32( fr_bands + NB_BANDS, st->enrO_fx, NB_BANDS );

    /* set the pointer of the current frame for the ACELP core */
    *inp = inp_16k;
    if ( sub(st->L_frame_fx,L_FRAME) == 0 )
    {
        *inp = inp_12k8;
    }


    return;
}
