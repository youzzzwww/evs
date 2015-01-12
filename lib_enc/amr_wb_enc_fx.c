/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>

#include "options.h"        /* Compilation switches                 */
#include "cnst_fx.h"        /* Common constants                     */
#include "rom_enc_fx.h"     /* Encoder static table prototypes      */
#include "rom_com_fx.h"     /* Static table prototypes              */
#include "prot_fx.h"        /* Function prototypes                  */
#include "stl.h"            /* required by wmc_tool                 */
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * amr_wb_enc()
 *
 * AMR-WB encoder
 *--------------------------------------------------------------------*/

void amr_wb_enc_fx(
    Encoder_State_fx *st,                      /* i/o: encoder state structure             */
    const Word16 input_sp[],               /* i  : input signal                        */
    const Word16 n_samples                 /* i  : number of input samples             */
)
{
    Word16 i, delay;
    Word16 old_inp[L_INP_12k8], *new_inp, *inp;                /* buffer of old input signal           */
    Word16 old_inp_16k[L_INP_12k8+L_SUBFR], *inp_16k, *new_inp_16k;/* buffer of old input signal @16kHz*/
    Word16 old_exc[L_EXC], *exc;                               /* excitation signal buffer             */
    Word16 old_wsp[L_WSP], *wsp;                               /* weighted input signal buffer         */
    Word16 input_frame;                                        /* frame length at input sampling freq. */
    Word32 fr_bands[2*NB_BANDS];                               /* energy in frequency bands            */
    Word32 lf_E[2*VOIC_BINS];                                  /* per bin spectrum energy in lf        */
    Word32 tmpN[NB_BANDS];                                     /* temporary noise update               */
    Word32 tmpE[NB_BANDS], PS[L_FFT/2];                        /* temporary averaged energy of 2 sf.   */
    Word16 corr_shift;                                         /* correlation shift                    */
    Word16 relE;                                               /* frame relative energy                */
    Word16 cor_map_sum, sp_div;
    Word16 vad_flag;
    Word16 localVAD;
    Word16 Etot;                                               /* total energy                         */
    Word32 ener = 0;                                           /* residual energy from Levinson-Durbin */
    Word16 pitch[3];                                           /* open-loop pitch values               */
    Word16 voicing[3];                                         /* open-loop pitch gains                */
    Word16 A[NB_SUBFR*(M+1)];                                  /* A(z) unquantized for the 4 subframes */
    Word16 Aw[NB_SUBFR*(M+1)];                                 /* A(z) unquantized for the 4 subframes */
    Word16 vad_hover_flag, noisy_speech_HO, clean_speech_HO, NB_speech_HO;
    Word16 epsP_h[M+1];                                        /* LP prediction errors                 */
    Word16 epsP_l[M+1];                                        /* LP prediction errors                 */
    Word16 isp_new[M];                                         /* ISPs at the end of the frame         */
    Word16 isf_new[M];                                         /* ISFs at the end of the frame         */
    Word16 isp_tmp[M];
    Word16 Aq[NB_SUBFR*(M+1)];                                 /* A(z) quantized for the 4 subframes   */
    Word16 syn[L_FRAME];                                       /* synthesis vector                     */
    Word16 res[L_FRAME];                                       /* residual signal for FER protection   */
    Word16 exc2[L_FRAME];                                      /* enhanced excitation                  */
    Word16 pitch_buf[NB_SUBFR];                                /* floating pitch for each subframe     */
    Word16 dummy_buf[L_FRAME32k];                              /* dummy buffer - no usage              */
    Word16 snr_sum_he;
    Word16 allow_cn_step;
    Word16 tmps;
    Word16 harm_flag; /* Q0 */
    Word16 high_lpn_flag;
    Word16 localVAD_HE_SAD;
    Word16 vad_flag_dtx;
    Word16 coder_type;
    Word16 hf_gain_fx[NB_SUBFR];
    Word16 Q_new, Q_exp,Q_r[2];
    Word16 excitation_max_test, shift;
    Word32 Le_min_scaled;
    Word16 Q_sp_div;
    Word16 non_staX, Scale_fac[2];
    Word16 sp_floor;
    Word16 fft_buff[2*L_FFT];
    Word32 q_env[NUM_ENV_CNG];
    Word16 sid_bw = 0;
    Word16 exc3[L_FRAME];
    Word32 lp_bckr, hp_bckr, Ltmp;
    Word16 tmp, e_tmp;

    /*------------------------------------------------------------------*
     * Initialization
     *------------------------------------------------------------------*/

    st->L_frame_fx = L_FRAME;
    move16();
    st->gamma = GAMMA1;
    move16();
    st->core_fx = AMR_WB_CORE;
    move16();
    st->core_brate_fx = st->total_brate_fx;
    move16();
    st->input_bwidth_fx = st->last_input_bwidth_fx;
    move16();
    st->bwidth_fx = st->last_bwidth_fx;
    move16();
    coder_type = GENERIC;
    move16();
    input_frame = st->input_frame_fx;
    move16();          /* frame length of the input signal */
    st->extl_fx = -1;
    st->encoderPastSamples_enc = (L_FRAME*9)/16;
    st->encoderLookahead_enc = L_LOOK_12k8;
    st->bpf_off_fx = 0;
    move16();
    test();
    if( sub(st->last_core_fx,HQ_CORE) == 0 || sub(st->last_codec_mode,MODE2) == 0 )
    {
        st->bpf_off_fx = 1;
        move16();
    }
    st->igf = 0;
    move16();

    /* Updates in case of EVS primary mode -> AMR-WB IO mode switching */
    IF( sub(st->last_core_fx,AMR_WB_CORE) != 0 )
    {
        updt_IO_switch_enc_fx( st, input_frame);
    }

    /* Updates in case of HQ -> AMR-WB IO mode switching */
    Q_new = 0;
    move16();       /* prevent compiler warning only*/
    core_switching_pre_enc_fx(st,&(st->LPDmem),input_frame,NULL, NULL);

    set16_fx( hf_gain_fx, 0, NB_SUBFR );
    set16_fx( old_inp, 0, L_INP_12k8 );
    exc = old_exc + L_EXC_MEM;                                  /* pointer to excitation signal in the current frame */

    Copy( st->LPDmem.old_exc, old_exc, L_EXC_MEM );

    new_inp = old_inp + L_INP_MEM;                              /* pointer to new samples of the input signal */
    inp = new_inp - L_LOOK_12k8;                                     /* pointer to current frame of input signal */
    wsp = old_wsp + L_WSP_MEM;                                  /* pointer to current frame of weighted signal */

    Copy( st->old_inp_12k8_fx, old_inp, L_INP_MEM );
    Copy( st->old_wsp_fx, old_wsp, L_WSP_MEM );

    new_inp_16k = old_inp_16k + L_INP_MEM;                      /* pointer to new samples of the input signal in 16kHz core */
    inp_16k = new_inp_16k - L_LOOK_16k;                         /* pointer to the current frame of input signal in 16kHz core */
    Copy( st->old_inp_16k_fx, old_inp_16k, L_INP_MEM );

    /*----------------------------------------------------------------*
     * set input samples buffer
     *----------------------------------------------------------------*/

    /* get delay to synchronize ACELP and MDCT frame */
    delay = NS2SA_fx2(st->input_Fs_fx, DELAY_FIR_RESAMPL_NS);

    Copy( st->input - delay, st->old_input_signal_fx, input_frame+delay );

    /*----------------------------------------------------------------*
     * Buffering of input signal
     * HP filtering
     *----------------------------------------------------------------*/
    Copy( input_sp, st->input, n_samples );
    FOR( i = n_samples; i < input_frame; i++ )
    {
        st->input[i] = 0;
        move16();
    }
    hp20( st->input, 1, input_frame, st->mem_hp20_in_fx, st->input_Fs_fx );

    /*-----------------------------------------------------------------*
     * switching from ACELP@16k core to AMR-WB IO mode
     *-----------------------------------------------------------------*/
    st->rate_switching_reset=0;
    move16();
    test();
    test();
    IF( sub(st->last_core_fx,AMR_WB_CORE) != 0 && sub(st->last_L_frame_fx,L_FRAME16k) == 0 && sub(st->last_core_fx,HQ_CORE) != 0)
    {
        /* in case of switching, do not apply BPF */
        st->bpf_off_fx = 1;
        move16();
        st->rate_switching_reset=lsp_convert_poly_fx(st->lsp_old_fx, L_FRAME, 1);

        /* convert old quantized LSF vector */
        lsp2lsf_fx( st->lsp_old_fx, st->lsf_old_fx, M, INT_FS_FX );

        /* Reset LPC mem */
        Copy( GEWB_Ave_fx, st->mem_AR_fx, M );
        set16_fx( st->mem_MA_fx,0, M );

        /* update synthesis filter memories */
        synth_mem_updt2( L_FRAME, st->last_L_frame_fx, st->LPDmem.old_exc, st->LPDmem.mem_syn_r, st->mem_syn1_fx, st->LPDmem.mem_syn, ENC );
        Copy( st->LPDmem.old_exc, old_exc, L_EXC_MEM );
        Copy( st->mem_syn1_fx, st->LPDmem.mem_syn2, M );

        /* lsp -> isp */
        Copy( stable_ISP_fx, isp_tmp, M );
        lsp2isp_fx( st->lsp_old_fx, st->lsp_old_fx, isp_tmp, M );

        /* update buffer of old subframe pitch values */
        FOR( i=NB_SUBFR16k-NB_SUBFR; i<NB_SUBFR16k; i++ )
        {
            /*((float)FAC_16k/(float)FAC_12k8) * st_fx->old_pitch_buf[i]*/
            st->old_pitch_buf_fx[i-1] = mult_r(26214, st->old_pitch_buf_fx[i]);
            move16();
        }

        FOR( i=2*NB_SUBFR16k-NB_SUBFR; i<2*NB_SUBFR16k; i++ )
        {
            st->old_pitch_buf_fx[i-2] =  mult_r(26214, st->old_pitch_buf_fx[i]);
            move16();
        }
    }

    test();
    if(sub(st->last_bwidth_fx,NB)==0 && st->ini_frame_fx!=0)
    {
        st->rate_switching_reset=1;
        move16();
    }
    /*----------------------------------------------------------------*
     * Change the sampling frequency to 12.8 kHz
     *----------------------------------------------------------------*/


    modify_Fs_fx( st->input, input_frame, st->input_Fs_fx, new_inp, 12800, st->mem_decim_fx, 0 );

    /* update signal buffer */
    Copy( new_inp, st->buf_speech_enc+L_FRAME, L_FRAME );
    Scale_sig( st->buf_speech_enc+L_FRAME, L_FRAME, 1 );
    /*------------------------------------------------------------------*
     * Perform fixed preemphasis through 1 - g*z^-1
     *-----------------------------------------------------------------*/

    Preemph_scaled( new_inp, &Q_new, &st->mem_preemph_fx, st->Q_max, PREEMPH_FAC, 0, 1, L_Q_MEM, L_FRAME, st->last_coder_type_fx, 1 );

    Q_exp = sub(Q_new, st->Q_old);
    move16();
    st->Q_old = Q_new;
    move16();

    Le_min_scaled = Scale_mem_pre_proc( st->ini_frame_fx, Q_exp, &Q_new, old_inp, &(st->mem_wsp_fx), st->enrO_fx, st->bckr_fx, st->ave_enr_fx,
                                        st->ave_enr2_fx, st->fr_bands1_fx, st->fr_bands2_fx, st->Bin_E_old_fx );

    Q_exp = sub(Q_new, st->prev_Q_new);
    move16();
    Scale_mem_enc( Q_exp, old_inp_16k, old_exc, st->old_bwe_exc_fx, &(st->LPDmem.mem_w0), st->LPDmem.mem_syn, st->LPDmem.mem_syn2,
                   &st->mem_deemp_preQ_fx, st->last_exc_dct_in_fx, st->old_input_lp_fx );

    /*----------------------------------------------------------------*
     * Compute spectrum, find energy per critical frequency band
     * Track energy and signal dynamics
     * Detect NB spectrum in a 16kHz-sampled input
     *----------------------------------------------------------------*/

    analy_sp( inp, Q_new, fr_bands, lf_E, &Etot, st->min_band_fx, st->max_band_fx, Le_min_scaled, Scale_fac, st->Bin_E_fx,
              st->Bin_E_old_fx, PS, st->lgBin_E_fx, st->band_energies, fft_buff );

    noise_est_pre_fx( Etot, st->ini_frame_fx, &st->Etot_l_fx, &st->Etot_h_fx, &st->Etot_l_lp_fx, &st->Etot_last_fx,
                      &st->Etot_v_h2_fx, &st->sign_dyn_lp_fx, st->harm_cor_cnt_fx, &st->Etot_lp_fx );

    /*----------------------------------------------------------------*
     * VAD
     *----------------------------------------------------------------*/

    vad_flag = wb_vad_fx( st, fr_bands, &localVAD, &noisy_speech_HO, &clean_speech_HO, &NB_speech_HO,
                          &snr_sum_he, &localVAD_HE_SAD, &(st->flag_noisy_speech_snr), Q_new ) ;

    if ( vad_flag == 0 )
    {
        coder_type = INACTIVE;
        move16();
    }
    /* apply DTX hangover for CNG analysis */
    vad_flag_dtx = dtx_hangover_addition_fx( st, localVAD, vad_flag, sub(st->lp_speech_fx, st->lp_noise_fx), 0,  &vad_hover_flag );



    /*-----------------------------------------------------------------*
     * Select SID or FRAME_NO_DATA frame if DTX enabled
     *-----------------------------------------------------------------*/

    IF ( sub(st->last_core_fx,AMR_WB_CORE) != 0 )
    {
        st->fd_cng_reset_flag = 1;
        move16();
    }
    ELSE IF ( s_and((st->fd_cng_reset_flag > 0),(sub(st->fd_cng_reset_flag,10) < 0)) )
    {
        st->fd_cng_reset_flag = add(st->fd_cng_reset_flag,1);
    }
    ELSE
    {
        st->fd_cng_reset_flag = 0;
        move16();
    }

    dtx_fx( st, vad_flag_dtx, inp, Q_new );

    test();
    if( L_sub(st->core_brate_fx,FRAME_NO_DATA) == 0 && L_sub(st->last_core_fx,AMR_WB_CORE) != 0 )
    {
        /* force SID frame in case of AMR-WB IO/EVS primary mode switching */
        st->core_brate_fx = SID_1k75;
        move16();
    }

    /*----------------------------------------------------------------*
     * Noise energy down-ward update and total noise energy estimation
     * Long-term energies and relative frame energy updates
     * Correlation correction as a function of total noise level
     *----------------------------------------------------------------*/
    noise_est_down_fx( fr_bands, st->bckr_fx, tmpN, tmpE, st->min_band_fx, st->max_band_fx, &st->totalNoise_fx,
                       Etot, &st->Etot_last_fx, &st->Etot_v_h2_fx, Q_new, Le_min_scaled );

    high_lpn_flag = 0;
    move16(); /* Q0 flag  */
    long_enr_fx( st,  Etot, localVAD_HE_SAD, high_lpn_flag );
    relE = sub(Etot, st->lp_speech_fx); /* Q8 */

    IF( sub(st->bwidth_fx, NB) != 0 )
    {
        lp_bckr = Mean32( st->bckr_fx, 10 );
    }
    ELSE
    {
        lp_bckr = Mean32( st->bckr_fx+1, 9 );
    }
    hp_bckr = L_shr(L_add(st->bckr_fx[st->max_band_fx-1] , st->bckr_fx[st->max_band_fx]),1);
    if( hp_bckr == 0 ) /* Avoid division by zero. */
    {
        hp_bckr = L_deposit_l(1);
    }
    tmp = BASOP_Util_Divide3232_Scale( lp_bckr, hp_bckr, &e_tmp );
    Ltmp = L_shr_r( L_deposit_h( tmp ), sub( 15, e_tmp ) );
    st->bckr_tilt_lt = L_add( Mpy_32_16_r( st->bckr_tilt_lt, 29491 ), Mpy_32_16_r( Ltmp, 3277 ) );

    corr_shift = correlation_shift_fx( st->totalNoise_fx );

    /*----------------------------------------------------------------*
     * WB, SWB and FB bandwidth detector
     *----------------------------------------------------------------*/
    bw_detect_fx( st, st->input, localVAD, NULL, NULL );
    /* in AMR_WB IO, limit the maximum band-width to WB */
    if( sub(st->bwidth_fx,WB) > 0 )
    {
        st->bwidth_fx = WB;
        move16();
    }

    /*----------------------------------------------------------------*
     * Perform LP analysis
     * Compute weighted inp
     * Perform open-loop pitch analysis
     * Perform 1/4 pitch precision improvement
     *----------------------------------------------------------------*/

    IF ( vad_flag == 0 )
    {
        /* reset the OL pitch tracker memories during inactive frames */
        pitch_ol_init_fx( &st->old_thres_fx, &st->old_pitch, &st->delta_pit_fx, &st->old_corr_fx) ;
    }

    analy_lp_AMR_WB_fx( inp, &ener, A, epsP_h, epsP_l, isp_new, st->lsp_old1_fx,
                        isf_new, st->old_pitch_la, st->old_voicing_la, Q_new, Q_r );

    find_wsp( A, inp, wsp, &st->mem_wsp_fx, TILT_FAC_FX, L_FRAME, L_LOOK_12k8, L_SUBFR, Aw, GAMMA1, NB_SUBFR );
    Scale_wsp( wsp, &(st->old_wsp_max), &shift, &Q_exp, &(st->old_wsp_shift),
               st->old_wsp2_fx, st->mem_decim2_fx, st->old_wsp_fx, add(L_FRAME, L_LOOK_12k8));

    excitation_max_test = -32768;
    move16();
    FOR (i = 0; i < L_EXC_MEM; i++)
    {
        excitation_max_test = s_max(abs_s(old_exc[i]),excitation_max_test);
    }

    test();
    if( sub(excitation_max_test,8192)>0 && shift==0 )
    {
        shift = -1;
        move16();
    }
    pitch_ol_fx( pitch, voicing, &st->old_pitch, &st->old_corr_fx, corr_shift, &st->old_thres_fx, &st->delta_pit_fx, st->old_wsp2_fx, wsp, st->mem_decim2_fx, relE, 0, st->bwidth_fx, st->Opt_SC_VBR_fx );
    st->old_pitch_la = pitch[2];
    move16();
    st->old_voicing_la = voicing[2];
    move16();
    /* VAD parameters update */
    vad_param_updt_fx( st, pitch, voicing, corr_shift, vad_flag, A );

    /*------------------------------------------------------------------*
     * Update estimated noise energy and voicing cut-off frequency
     *-----------------------------------------------------------------*/
    noise_est_fx( st, tmpN, pitch, voicing, epsP_h, epsP_l, Etot, relE, corr_shift, tmpE, fr_bands, &cor_map_sum, &sp_div, &Q_sp_div, &non_staX, &harm_flag,
                  lf_E,  &st->harm_cor_cnt_fx,  st->Etot_l_lp_fx, st->Etot_v_h2_fx, &st->bg_cnt_fx, st->lgBin_E_fx, Q_new, Le_min_scaled, &sp_floor );

    /*----------------------------------------------------------------*
     * Change the sampling frequency to 16 kHz,
     *   input@16kHz needed for AMR-WB IO BWE @23.85kbps
     *----------------------------------------------------------------*/

    test();
    IF( L_sub(st->input_Fs_fx, 16000) == 0 )
    {
        /* no resampling needed, only delay adjustement to account for the FIR resampling delay */
        tmps = NS2SA_fx2(16000, DELAY_FIR_RESAMPL_NS);
        Copy_Scale_sig( &st->mem_decim16k_fx[tmps], new_inp_16k, tmps, -1 );      /* Input in Q0 -> Output in Q-1 to mimic the resampling filter */
        Copy_Scale_sig( st->input, new_inp_16k + tmps, sub(input_frame, tmps), -1 );  /* Input in Q0 -> Output in Q-1 to mimic the resampling filter */
        Copy( st->input + input_frame - shl(tmps,1), st->mem_decim16k_fx, shl(tmps,1) );  /* memory still in Q0 */
    }
    ELSE IF( L_sub(st->input_Fs_fx, 32000) == 0 || L_sub(st->input_Fs_fx, 48000) == 0 )
    {
        modify_Fs_fx( st->input, input_frame, st->input_Fs_fx, new_inp_16k, 16000, st->mem_decim16k_fx, 0 );
    }

    /*----------------------------------------------------------------*
     * Encoding of SID frames
     *----------------------------------------------------------------*/
    test();
    IF ( L_sub(st->core_brate_fx,SID_1k75) == 0 || L_sub(st->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        /* encode CNG parameters */
        CNG_enc_fx( st, L_FRAME, Aq, inp, ener, isp_new, isf_new , &allow_cn_step, st->burst_ho_cnt_fx, sub(Q_new,1),
                    q_env, &sid_bw, st->exc_mem2_fx );

        /* comfort noise generation */
        CNG_exc_fx( st->core_brate_fx, L_FRAME, &st->Enew_fx, &st->cng_seed_fx, exc, exc2, &st->lp_ener_fx,
                    st->last_core_brate_fx, &st->first_CNG_fx, &st->cng_ener_seed_fx, dummy_buf, allow_cn_step,
                    &st->last_allow_cn_step_fx, sub(st->prev_Q_new,1), sub(Q_new,1), st->num_ho_fx, q_env, st->lp_env_fx,
                    st->old_env_fx, st->exc_mem_fx, st->exc_mem1_fx, &sid_bw, &st->cng_ener_seed1_fx, exc3, st->Opt_AMR_WB_fx );

        if ( st->first_CNG_fx == 0 )
        {
            st->first_CNG_fx = 1;
            move16();
        }

        /* synthesis */
        syn_12k8_fx( L_FRAME, Aq, exc2, syn, st->LPDmem.mem_syn2, 1, Q_new, st->Q_syn  );

        /* reset the encoder */
        CNG_reset_enc_fx( st,  &(st->LPDmem), pitch_buf, dummy_buf );

        /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
        Copy( syn + L_FRAME - L_SYN_MEM, st->LPDmem.mem_syn_r, L_SYN_MEM );

        /* update st->mem_syn1 for ACELP core switching */
        Copy( st->LPDmem.mem_syn2, st->mem_syn1_fx, M );
    }

    /*----------------------------------------------------------------*
     * Encoding of all other frames
     *----------------------------------------------------------------*/
    ELSE
    {
        /*-----------------------------------------------------------------*
         * After inactive period, use the most up-to-date ISPs
         *-----------------------------------------------------------------*/
        test();
        IF( L_sub(st->last_core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st->last_core_brate_fx,SID_1k75) == 0 )
        {
            Copy( st->lspCNG_fx, st->lsp_old_fx, M );
            E_LPC_isp_isf_conversion( st->lspCNG_fx, st->lsf_old_fx, M);
            set16_fx( old_exc, 0, L_EXC_MEM );
        }

        /*-----------------------------------------------------------------*
         * ISF Quantization and interpolation
         *-----------------------------------------------------------------*/
        isf_enc_amr_wb_fx( st, isf_new, isp_new, Aq, st->clas_fx, &st->stab_fac_fx);

        /*---------------------------------------------------------------*
         * Calculation of LP residual (filtering through A[z] filter)
         *---------------------------------------------------------------*/

        calc_residu_fx( st, inp, res, Aq, 0 );
        st->burst_ho_cnt_fx = 0;
        move16();

        /*------------------------------------------------------------*
         * Encode excitation
         *------------------------------------------------------------*/

        encod_amr_wb_fx( st, &(st->LPDmem), inp, Aw, Aq, pitch, voicing, res, syn, exc, exc2, pitch_buf, hf_gain_fx, inp_16k, shift, Q_new );

        /* update st->mem_syn1 for ACELP core switching */
        Copy( st->LPDmem.mem_syn, st->mem_syn1_fx, M );

        /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
        Copy( syn + L_FRAME - L_SYN_MEM, st->LPDmem.mem_syn_r, L_SYN_MEM );

        /*--------------------------------------------------------------------------------------*
         * Write VAD information into the bitstream in AMR-WB IO mode
         *--------------------------------------------------------------------------------------*/

        push_indice_fx( st, IND_VAD_FLAG, vad_flag, 1 );
    }

    E_UTIL_deemph2( sub(Q_new,1), syn, PREEMPH_FAC, L_FRAME, &(st->LPDmem.syn[M]) );
    Copy( syn+L_FRAME-M-1, st->LPDmem.syn,M+1 );

    /*-----------------------------------------------------------------*
     * Updates
     *-----------------------------------------------------------------*/

    /* update old weighted speech buffer - for OL pitch analysis */
    Copy( &old_wsp[L_FRAME], st->old_wsp_fx, L_WSP_MEM );

    /* update old input signal buffer */
    Copy( &old_inp[L_FRAME], st->old_inp_12k8_fx, L_INP_MEM );

    /* update old input signal @16kHz buffer */
    Copy( &old_inp_16k[L_FRAME16k], st->old_inp_16k_fx, L_INP_MEM );

    /* update of old per-band energy spectrum */
    Copy32( fr_bands + NB_BANDS, st->enrO_fx, NB_BANDS );

    /* update the last bandwidth */
    st->last_input_bwidth_fx = st->input_bwidth_fx;
    st->last_bwidth_fx = st->bwidth_fx;

    /* update signal buffers */
    Copy( new_inp, st->buf_speech_enc_pe+L_FRAME, L_FRAME );
    Copy( wsp, st->buf_wspeech_enc+L_FRAME+L_SUBFR, L_FRAME + L_LOOK_12k8 );
    updt_enc_fx( st, L_FRAME, coder_type, old_exc, pitch_buf, 0, Aq, isf_new, isp_new, dummy_buf );

    core_encode_update( st );

    /* update main codec parameters */
    st->last_extl_fx           = -1;
    move16();
    st->last_core_fx           = st->core_fx;
    move16();
    st->last_L_frame_fx        = L_FRAME;
    move16();
    st->last_core_brate_fx     = st->core_brate_fx;
    move16();
    st->last_total_brate_fx    = st->total_brate_fx;
    move16();
    st->Etot_last_fx           = Etot;
    move16();
    st->last_coder_type_raw_fx = st->coder_type_raw_fx;
    move16();
    st->last_codec_mode        = st->codec_mode;
    move16();

    st->prev_Q_new = Q_new;

    /* Increase the counter of initialization frames */
    if( sub(st->ini_frame_fx,MAX_FRAME_COUNTER) < 0 )
    {
        st->ini_frame_fx = add(st->ini_frame_fx,1);
    }

    if( L_sub(st->core_brate_fx,SID_1k75) > 0 )
    {
        st->last_active_brate_fx = st->total_brate_fx;
        move32();
    }

    test();
    IF ( L_sub(st->core_brate_fx,SID_1k75) > 0 && st->first_CNG_fx )
    {
        if( sub(st->act_cnt_fx,BUF_DEC_RATE) >= 0 )
        {
            st->act_cnt_fx = 0;
            move16();
        }

        st->act_cnt_fx = add(st->act_cnt_fx,1);

        test();
        if( sub(st->act_cnt_fx,BUF_DEC_RATE) == 0 && st->ho_hist_size_fx > 0 )
        {
            st->ho_hist_size_fx = sub(st->ho_hist_size_fx,1);
        }

        st->act_cnt2_fx = add(st->act_cnt2_fx,1);
        if( sub(st->act_cnt2_fx, MIN_ACT_CNG_UPD) >= 0 )
        {
            st->act_cnt2_fx = MIN_ACT_CNG_UPD;
            move16();
        }
    }


    return;
}
