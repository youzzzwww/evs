/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef STAT_ENC_FX_H
#define STAT_ENC_FX_H

#include <stdio.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "stat_dec_fx.h" /* Compilation switches                   */


/*------------------------------------------------------------------------------------------*
 * Indice
 *------------------------------------------------------------------------------------------*/

typedef struct
{
    UWord16 value;                                        /* value of the quantized indice */
    Word16 nb_bits;                                      /* number of bits used for the quantization of the indice */
} Indice_fx;

typedef  struct
{
    Word32 r;
    Word32 i;
} complex_32;
typedef struct
{
    Word16 r;
    Word16 i;
} complex_16;
typedef struct
{
    Word16 s16Exp;
    Word32 s32Mantissa;
} T_VAD_EXP;

typedef struct
{
    Word16  bw_index;                                         /* index of band width */

    /* feature */
    Word16  sp_center[SP_CENTER_NUM];                        /* spectral center*/
    Word16  ltd_stable_rate[STABLE_NUM];                      /* time-domain stable rate*/
    Word16  sfm[SFM_NUM];                                     /* spectral flatness*/
    Word16  f_tonality_rate[TONA_NUM];                        /* tonality rate*/
    Word16  pre_spec_low_dif[PRE_SPEC_DIF_NUM];                /* low frequency spectral different*/
    Word32  frames_power_32[POWER_NUM];                       /* energy of several frames*/
    Word32  frame_sb_energy[BG_ENG_NUM];                      /* energy of sub-band divided non-uniformly*/
    Word32  t_bg_energy;                                      /* time background energy of several frames*/
    T_VAD_EXP t_bg_energy_sum;                                /* number of time background energy*/
    Word16  tbg_energy_count;                                 /* sum of time background energy of several frames*/
    Word16  bg_update_count;                                  /* time of background update*/
    Word32  frame_energy_smooth;                              /* smoothed energy of several frames*/

    /************************************************************************/
    /* history  parameters                                                  */
    /************************************************************************/
    Word32  smooth_spec_amp[SPEC_AMP_NUM];                    /* smoothed spectral amplitude*/
    Word32  pre_snr[PRE_SNR_NUM];                             /* previous time SNR*/
    Word32  sb_bg_energy[BG_ENG_NUM];                         /* sub-band background energy*/
    Word16  continuous_noise_num;                             /* time of continuous noise frames*/
    Word16  continuous_speech_num;                            /* time of continuous speech frames*/
    Word16  continuous_speech_num2;                           /* time 2 of continuous speech frames*/
    Word32  fg_energy_est_start;                              /* flag by that indicate whether if estimate energy*/
    Word16  speech_flag;                                      /* residual number of hangover 1 */
    Word32  lt_noise_sp_center_diff_counter;                  /* number of the member lt_noise_sp_center_diff_sum*/
    Word32  fg_energy;                                        /* foreground energy sum  */
    Word32  bg_energy;                                        /* background energy sum  */
    Word32  update_num;                                       /* time of update, initial value is 16*/
    Word32  update_sum;                                       /* time of update, initial value is 0*/
    Word32  update_hangover;                                  /* background update hangover*/
    Word32  lt_bg_highf_eng;                                  /* average  of long time high frequency energy*/
    Word32  lt_noise_sp_center_diff_sum;                      /* different sum of long time noise sp_center*/
    Word32  lt_snr_org;                                       /* original long time SNR*/
    Word32  l_speech_snr;                                     /* sum of snr's of active frames*/
    Word32  l_silence_snr;                                    /* sum of snr's of non active frames*/
    Word32  l_speech_snr_count;                               /* number of active frames*/
    Word32  l_silence_snr_count;                              /* number of non active frames*/
    Word32  lf_snr_smooth;                                    /* smoothed lf_snr*/
    Word16  speech_flag2;
    Word16  frameloop;                                        /* number of frame*/
    Word16  lt_noise_sp_center0;                              /* long time noise sp_center0*/
    Word16  lt_noise_sp_center3;                              /* long time noise sp_center3*/
    Word16  music_background_rate;                            /* music background rate*/
    Word16  tonality_rate3;                                   /* tonality rate*/
    Word16  bg_energy_count;                                  /* number of the background energy frame */
    Word16  fg_energy_count;                                  /* number of the foreground energy frame */
    Word16  Q_frames_power_32;                                /* the Scaling of frames_power_fix32*/
    Word16  scale_spec_low_dif;                               /* the Scaling of spec_low_dif*/
    Word16  sb_bg_energy_scale;                               /* the Scaling of sb_bg_energy*/
    Word16  frame_sb_energy_scale;                            /* the Scaling of frame_sb_energy*/
    Word16  scale_t_bg_energy;                                /* the Scaling of t_bg_energy*/
    Word16  frame_energy_smooth_scale;                        /* the Scaling of frame_energy_smooth*/
    Word16  bg_energy_scale;                                  /* the Scaling of bg_energy*/
    Word16  fg_energy_scale;                                  /* the Scaling of fg_energy*/
    Word16  updateNumWithSnr;                                 /* the number of the background update with SNR*/

} T_CldfbVadState;

/*ari.h*/
typedef struct
{
    Word32 low,high;
    Word16 vobf;
} TastatEnc;

/*---------------------------------------------------------------*
 * IGF                                                           *
 *---------------------------------------------------------------*/
/* IGFSCFEncoder.h */
typedef struct
{
    Word16            ptrBitIndex;
    Word16            bitCount;
    Word16            prev[64]; /* no more than 64 SCFs for the IGF energy envelope of one block, short or long */
    Word16            prevSave[64];
    Word16            scfCountLongBlock;
    Word16            t;
    Word16            tSave;
    Word16            context_saved;
    const Word16     *cf_se00;
    const Word16     *cf_se01;
    Word16            cf_off_se01;
    const Word16     *cf_se02;
    const Word16     *cf_off_se02;
    const Word16     *cf_se10;
    Word16            cf_off_se10;
    const Word16     *cf_se11;
    const Word16     *cf_off_se11;
    TastatEnc         acState;
} IGFSCFENC_INSTANCE, *IGFSCFENC_INSTANCE_HANDLE;

/* IGFEnc.h */
typedef struct igf_enc_private_data_struct
{
    IGF_INFO                    igfInfo;
    Word16                      igfScfQuantized[IGF_MAX_SFB];
    IGFSCFENC_INSTANCE          hIGFSCFArithEnc;

    /*** whitening ***/
    Word16                      igfCurrWhiteningLevel[IGF_MAX_TILES];
    Word16                      igfPrevWhiteningLevel[IGF_MAX_TILES];

    Word32                      prevSFM_FIR[IGF_MAX_TILES];           /* 15Q16  */
    Word16                      prevSFM_IIR[IGF_MAX_TILES];           /* 2Q13   */
    Word16                      wasTransient;                         /* 15Q0   */

    Word16                      igfBitstream[IGF_MAX_GRANULE_LEN];
    Word16                      igfBitstreamBits;

} IGF_ENC_PRIVATE_DATA, *IGF_ENC_PRIVATE_DATA_HANDLE;

typedef struct igf_enc_instance_struct
{
    IGF_ENC_PRIVATE_DATA  igfData;
    Word32                infoSamplingRate;
    Word16                infoStartFrequency;
    Word16                infoStopFrequency;
    Word16                infoStartLine;
    Word16                infoStopLine;
    Word16                infoTotalBitsWritten;
    Word16                infoTotalBitsPerFrameWritten;
    Word16                flatteningTrigger;
    Word32                spec_be_igf[N_MAX];           /* copy of MDCT spectrum */
    Word16                spec_be_igf_e;                /* exponent of copy of MDCT spectrum */
    Word16                tns_predictionGain;
} IGF_ENC_INSTANCE, *IGF_ENC_INSTANCE_HANDLE;



typedef struct
{
    Word16  nbits;             /* number of bits used by ACELP or TCX     */

    /* signal memory */
    Word16 syn[1+M];           /* Synthesis memory (non-pe)                              */
    /*Word16 wsyn;*/              /* Weighted synthesis memory                              */
    Word16 mem_syn[M];         /* ACELP synthesis memory (pe) before post-proc           */
    Word16 mem_syn2[M];        /* ACELP synthesis memory (pe) after post-proc            */
    Word16 mem_w0;            /* weighting filter memory */
    Word16 tilt_code;
    Word16 mem_syn_r[L_SYN_MEM];      /* ACELP synthesis memory for 1.25ms                      */
    Word16 mem_syn3[M];

    /* ACELP memory */
    Word16 old_exc[L_EXC_MEM];     /* ACELP exc memory (Aq)                   */

    /* TCX memory */
    Word16 Txnq[4*L_MDCT_OVLP_MAX];  /* Q target (overlap or ACELP+ZIR, use Aq) */
    Word16 *acelp_zir;
    Word16 tcx_target_bits_fac;

    Word32 gc_threshold; /* exponent = 15, 15Q16 */

} LPD_state;


typedef struct PLC_ENC_EVS
{

    Word16 nBits;                /* number of bits */
    Word16 enableGplc;

    Word16 Q_new;
    Word16 Q_exp;

    Word16 T0_4th;
    Word16 T0;
    Word16 old_exc_Qold[8];               /* ACELP exc memory (Aq)                         */

    Word16 lsfoldbfi0_14Q1[M];            /* Previous frame LSF                      */
    Word16 lsfoldbfi1_14Q1[M];            /* Past previous frame LSF   */
    Word16 lsfold_14Q1[M];                /* old lsf (frequency domain) */
    Word16 lspold_Q15[M];                 /* old lsp (immittance spectral pairs) */

    Word16 mem_MA_14Q1[M];
    Word16 mem_AR[M];
    Word16 lsf_adaptive_mean_14Q1[M];     /*  Mean isf for bfi cases                 */
    Word16 stab_fac_Q15;


    /* For TCX case */

    LPD_state *LPDmem;
    Word16 lsf_con[M];
    Word16 last_lsf_ref[M];
    Word16 last_lsf_con[M];
} PLC_ENC_EVS, *HANDLE_PLC_ENC_EVS;


/*****************************************/
/*          MODE2 STAT ENC             */
/*****************************************/

/*transient_detection.h*/
/** Delay buffer.
 * Used to buffer input samples and to define the subblock size of a transient detector.
 */
typedef struct
{
    /** Subblock size of a transient detector that uses this delay buffer. */
    Word16   nSubblockSize;
    /** Delay buffer */
    Word16 buffer[L_FRAME48k/NSUBBLOCKS];
    /** Size of the delay buffer in use. Maximum delay from all users of this buffer. */
    Word16   nDelay;
} DelayBuffer;

/** Subblock energies.
 * Holds subblock energies and recursively accumulated energies.
 * Also buffers the energies.
 */
typedef struct
{
    /** Delay buffer. */
    DelayBuffer * pDelayBuffer;
    /** Subblock energies with a delay buffering. */
    Word32 subblockNrg[NSUBBLOCKS+MAX_TD_DELAY];
    /** Recursively accumulated subblock energies with a delay buffering.
     At index i the value corresponds to the accumulated subblock energy up to i-1,
     including block i-1 and without block i. */
    Word32 accSubblockNrg[NSUBBLOCKS+MAX_TD_DELAY+1];
    /** subblockNrgChange[i] = max(subblockNrg[i]/subblockNrg[i-1], subblockNrg[i-1]/subblockNrg[i]) */
    Word16 subblockNrgChange[NSUBBLOCKS+MAX_TD_DELAY];
    /** Size of the delay buffer in use, as number of subblocks. Maximum delay from all users of this buffer. */
    Word16   nDelay;
    /* Delay of the input (modulo pDelayBuffer->nSubblockSize), nPartialDelay <= pDelayBuffer->nDelay. */
    Word16   nPartialDelay;

    /** Decay factor for the recursive accumulation */
    Word16 facAccSubblockNrg;

    /** High-pass filter states (delay line) */
    Word16 firState1;
    Word16 firState2;

} SubblockEnergies;

struct TransientDetector;

/** Attack detection function.
 * Definition of a function used for checking the presence of an attack, given subblock energies, accumulated subblock energies and a threshold.
 * @param pSubblockNrg Subblock energies.
 * @param pAccSubblockNrg Recursively accumulated subblock energies.
 At index i the value corresponds to the accumulated subblock energy up to i-1,
 including block i-1 and without block i.
 * @param nSubblocks Number of subblocks available (those with an index >= 0). Subblocks from 0 to NSUBBLOCKS-1 correspond to the current frame.
 * @param nPastSubblocks Number of past subblocks available (those with a negative index).
 * @param attackRatioThreshold Attack ratio threshold.
 * @param pbIsAttackPresent Pointer to an output variable that will be set to TRUE if an attack is found, otherwise set to FALSE.
 * @param pAttackIndex Pointer to an output variable that will hold an attack position.
 */
typedef void (* TCheckSubblocksForAttack)(Word32 const * pSubblockNrg, Word32 const * pAccSubblockNrg, Word16 nSubblocks, Word16 nPastSubblocks, Word16 attackRatioThreshold, Word16 * pbIsAttackPresent, Word16 * pAttackIndex);

/** Transient detector.
 */
typedef struct TransientDetector
{
    /** Subblock energies used in this transient detector. */
    SubblockEnergies * pSubblockEnergies;
    /* Delay of the transient detector in number of subblocks, nDelay <= pSubblockEnergies->nDelay. */
    Word16   nDelay;
    /** Number of subblocks to check for transients. */
    Word16   nSubblocksToCheck;
    /** Function for checking a presence of an attack. */
    TCheckSubblocksForAttack CheckSubblocksForAttack;
    /** Attack ratio threshold. */
    Word16 attackRatioThreshold;
    /** True when an attack was detected. */
    Word16   bIsAttackPresent;
    /** The index of an attack. */
    Word16   attackIndex;

} TransientDetector;


/** Transient detection.
 * Holds all transient detectors and buffers used by them.
 */
typedef struct TransientDetection
{
    /** Transient detector. */
    TransientDetector transientDetector;
    /** Delay buffer used by the transient detectors. */
    DelayBuffer delayBuffer;
    /** Subblock energies used by the transient detector. */
    SubblockEnergies subblockEnergies;
} TransientDetection;

/* Arrays and variables specific to encoder */
typedef struct
{
    HANDLE_FD_CNG_COM hFdCngCom;

    Word32  msPeriodog[NPART]; /* Periodogram */
    Word16  msPeriodog_exp;    /* Common exponent for fft and cldfb energies */
    Word16  msPeriodog_exp_fft;
    Word16  msPeriodog_exp_cldfb;
    Word32  msBminWin[NPART];
    Word32  msBminSubWin[NPART];
    Word16  msPsd[NPART];          /* Power Spectral Density estimate (i.e., smoothed periodogram) */
    Word32  msAlpha[NPART];        /* Optimal smoothing parameter */
    Word32  msMinBuf[MSNUMSUBFR*NPART];       /* Buffer of minima */
    Word32  msCurrentMinOut[NPART];
    Word32  msCurrentMin[NPART];
    Word32  msCurrentMinSubWindow[NPART];
    Word16  msLocalMinFlag[NPART];
    Word16  msNewMinFlag[NPART];
    Word16  msPsdFirstMoment[NPART];
    Word32  msPsdSecondMoment[NPART];
    Word16  msNoiseFloor[NPART];   /* Estimated noise floor */
    Word32  msNoiseEst[NPART];     /* Estimated noise level */
    Word16  msNoiseEst_exp;
    Word32  energy_ho[NPART];
    Word16  energy_ho_exp;
    Word32  msNoiseEst_old[NPART];
    Word16  msNoiseEst_old_exp;

    Word16  msPeriodogBuf[MSBUFLEN*NPART];
    Word16  msPeriodogBufPtr;

    Word16  stopFFTbinDec;
    Word16  startBandDec;
    Word16  stopBandDec;
    Word16  npartDec;
    Word16  midbandDec[NPART];
    Word16  nFFTpartDec;
    Word16  partDec[NPART];
    Word16  psizeDec[NPART];
    Word16  psizeDec_norm[NPART];
    Word16  psizeDec_norm_exp;
    Word16  psize_invDec[NPART];

    Word16  msLogPeriodog[NPART];
    Word16  msLogNoiseEst[NPART];
}
FD_CNG_ENC;
typedef FD_CNG_ENC *HANDLE_FD_CNG_ENC;



typedef struct Encoder_State_fx
{

    /*----------------------------------------------------------------------------------*
     * Common parameters
     *----------------------------------------------------------------------------------*/

    Word16 codec_mode;                          /* MODE1 or MODE2 */
    Word16 last_codec_mode;                     /* Previous Codec Mode*/
    Word16 mdct_sw_enable;                      /* MDCT switching enable flag */
    Word16 mdct_sw;                             /* MDCT switching indicator */
    Word16 prev_hi_ener;
    Word16 prev_hi_sparse;
    Word16 clas_sec_old_fx;                     /* MDCT classifier secondary decision memory */
    Word16 clas_final_old_fx;                   /* MDCT classifier final decision memory */
    Word32 last_gain1;
    Word32 last_gain2;
    Word16 nb_bits_tot_fx;                      /* total number of bits already written */
    Word16 next_bit_pos_fx;                     /* position of the next bit to be written in the bitstream */
    Indice_fx *ind_list_fx;                     /* list of indices */
    Word16 next_ind_fx;                         /* pointer to the next empty slot in the list of indices */
    Word16 last_ind_fx;                         /* last written indice */

    Word32 input_Fs_fx;                         /* input signal sampling frequency in Hz */
    Word32 total_brate_fx;                      /* total bitrate in kbps of the codec */
    Word32 last_total_brate_fx;                 /* total bitrate in kbps of the codec */
    Word16 core_fx;                             /* core (ACELP_CORE or HQ_CORE) */
    Word32 core_brate_fx;                        /* core bitrate */
    Word32 last_core_brate_fx;                    /* previous frame core bitrate */
    Word16 input_frame_fx;                      /* Frame lenght (function of input_Fs) */
    Word16 extl_fx;                             /* extension layer */
    Word16 last_extl_fx;                        /* previous extension layer */
    Word32 extl_brate_fx;                       /* extension layer bitrate */
    Word16 input_bwidth_fx;                     /* input signal bandwidth */
    Word16 last_input_bwidth_fx;                /* input signal bandwidth in the previous frame */
    Word16 bwidth_fx;                           /* encoded bandwidth NB, WB, SWB or FB */
    Word16 max_bwidth_fx;                       /* maximum encoded bandwidth */
    Word16 last_bwidth_fx;                      /* input signal bandwidth in the previous frame */
    Word16 L_frame_fx;                          /* ACELP core internal frame length */
    Word16 Opt_AMR_WB_fx;                       /* flag indicating AMR-WB IO mode */
    Word16 Opt_DTX_ON_fx;                       /* flag indicating DTX operation */
    Word16 cng_type_fx;                         /* flag indicating LP or CLDFB based SID/CNG */
    Word16 active_fr_cnt_fx;                    /* counter of active frames */
    Word16 Opt_SC_VBR_fx;                       /* flag indicating SC-VBR mode */
    Word16 last_Opt_SC_VBR_fx;                  /* flag indicating SC-VBR mode in the last frame */
    Word16 lp_cng_mode2;

    /*----------------------------------------------------------------------------------*
     * ACELP core parameters
     *----------------------------------------------------------------------------------*/

    Word16 clas_fx;                                 /* current frame clas */
    Word16 last_clas_fx;                            /* previous frame signal classification */
    Word32 Bin_E_fx[L_FFT];                         /* Q_new + Q_SCALE -2 per bin energy of two frames */
    Word32 Bin_E_old_fx[L_FFT/2];                   /* per bin energy of old 2nd frames */
    Word16 lsp_old1_fx[M];                          /* old unquantized LSP vector at the end of the frame */
    Word16 lsf_old1_fx[M];                          /* old LSF vector at the end of the frame */
    Word16 lsp_old_fx[M];                           /* old LSP vector at the end of the frame */
    Word16 lsf_old_fx[M];                           /* old LSF vector at the end of the frame */
    Word16 lsp_old16k_fx[M];                        /* old LSP vector at the end of the frame @16kHz */
    Word16 lspold_enc_fx[M];                        /* old LSP vector at the end of the frame @16kHz */
    Word16 pstreaklen_fx;                           /* LSF quantizer */
    Word16 streaklimit_fx;                          /* LSF quantizer */
    Word32 offset_scale1_fx[MAX_NO_MODES][MAX_NO_SCALES+1];     /* offsets for LSF LVQ structure 1st 8-dim subvector Q0*/
    Word32 offset_scale2_fx[MAX_NO_MODES][MAX_NO_SCALES+1];     /* offsets for LSF LVQ structure 2nd 8-dim subvector Q0*/
    Word32 offset_scale1_p_fx[MAX_NO_MODES_p][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure, pred. case, 1st 8-dim subvector Q0*/
    Word32 offset_scale2_p_fx[MAX_NO_MODES_p][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure, pred. case, 2nd 8-dim subvector Q0*/
    Word16 no_scales_fx[MAX_NO_MODES][2];                         /* LSF LVQ structure Q0*/
    Word16 no_scales_p_fx[MAX_NO_MODES_p][2];                     /* LSF LVQ structure Q0*/
    Word16 stab_fac_fx;                                        /* LSF stability factor */
    Word16 mem_decim_fx[2*L_FILT_MAX];                      /* decimation filter memory */
    Word16 mem_deemph_fx;                                   /* deemphasis filter memory */
    Word16 mem_preemph_fx;                                  /* preemphasis filter memory */
    Word32 mem_hp20_in_fx[5];                               /* HP filter memory for AMR-WB IO */
    Word16 old_inp_12k8_fx[L_INP_MEM];                      /* memory of input signal at 12.8kHz */
    Word16 old_wsp_fx[L_WSP_MEM];                           /* old weighted signal vector */
    /*Word16 old_exc_fx[L_EXC_MEM];*/                           /* old excitation vector */
    Word16 old_wsp2_fx[(L_WSP_MEM - L_INTERPOL)/OPL_DECIM]; /* old decimated weighted signal vector qwsp */
    Word32 fr_bands1_fx[NB_BANDS];                 /* Q_new + Q_SCALE spectrum per critical bands of the previous frame  */
    Word32 fr_bands2_fx[NB_BANDS];                 /* Q_new + Q_SCALE spectrum per critical bands 2 frames ago  */
    Word16 mem_wsp_fx;                                        /* weighted signal vector memory */
    Word16 mem_decim2_fx[3];                                /* weighted signal decimation filter memory qwsp */
    Word16 mem_syn1_fx[M];                                  /* synthesis filter memory (for core switching and FD BWE) */

    Word16 clip_var_fx[6];
    Word16 past_qua_en_fx[4];
    Word16 mem_AR_fx[M];                                    /* AR memory of LSF quantizer (past quantized LSFs without mean) */
    Word16 mem_MA_fx[M];                                    /* MA memory of LSF quantizer (past quantized residual) (used also in AMR-WB IO mode) */
    Word16 mCb1_fx;                                         /* LSF quantizer - counter of stationary frames after a transition frame */
    Word16 coder_type_raw_fx;
    Word16 last_coder_type_raw_fx;                          /* raw last_coder_type (coming from the sigal classification) */
    Word16 last_coder_type_fx;                              /*Q0 previous coding type */
    Word16 ini_frame_fx;                                    /* initialization frames counter */
    Word16 old_thres_fx;                                    /* normalized correlation weighting in open-loop pitch Q15 */
    Word16 old_corr_fx;                                     /* normalized correlation in previous frame (mean value) Q15 */
    Word16 old_pitch;                                       /* previous pitch for open-loop pitch search Q0 */
    Word16 delta_pit_fx;                                    /* open-loop pitch extrapolation correction Q0 */
    Word32 ee_old_fx;
    Word16 min_band_fx;                                     /* Q0 minimum critical band of useful bandwidth */
    Word16 max_band_fx;                                     /* Q0 maximum critical band of useful bandwidth */
    Word16 tc_cnt_fx;                                       /* TC frame counter */
    Word16 audio_frame_cnt_fx;                              /* Counter of relative presence of audio frames      */
    Word32 old_dE1_fx;                                      /* Maximum energy increase in previous frame */
    Word16 old_ind_deltaMax_fx;                             /* Index of the sub-subframe of maximum energy in previous frame */
    Word32 old_enr_ssf_fx[2*NB_SSF];                        /* Maxima of energies per sub-subframes of previous frame */
    Word16 spike_hyst_fx;                                   /* Hysteresis to prevent UC after sharp energy spike */
    Word16 music_hysteresis_fx;                             /* Counter of frames after AUDIO coding mode to prevent UC */
    Word16 last_harm_flag_acelp_fx;                         /* harmonicity flag for ACELP @32kbps rate */
    Word16 old_Aq_12_8_fx[M+1];                             /* Q12 old Aq[] for core switching */
    Word16 old_Es_pred_fx;                                  /* Q8 old Es_pred for core switching */
    Word16 high_stable_cor_fx;                              /* flag for stable high correlation */

    Word16 seed_tcx_fx ;                                    /* AC mode (GSC) - seed for noise fill*/
    Word16 cor_strong_limit_fx;                             /* AC mode (GSC) - Indicator about high spectral correlation per band */
    Word16 GSC_noisy_speech_fx;                             /* AC mode (GSC) - flag to indicate GSC on SWB noisy speech */
    Word16 mem_last_pit_band_fx;                            /* AC mode (GSC) - memory of the last band where pitch contribution was significant */
    Word16 mem_w0_tmp_fx;
    Word16 mem_syn_tmp_fx[M];
    Word16 var_cor_t_fx[VAR_COR_LEN];
    Word16 mid_dyn_fx;                                      /* AC mode (GSC) - signal dynamic                         */
    Word16 old_y_gain_fx[MBANDS_GN];                        /* AC mode (GSC) - AR mem for low rate gain quantization  */
    Word16 noise_lev_fx;                                    /* AC mode (GSC) - Q0 noise level                             */
    Word16 past_dyn_dec_fx;                                 /* AC mode (GSC) - Past noise level decision */
    Word32 Last_frame_ener_fx;                              /* AC mode (GSC) - Last frame energy  */
    Word16 pit_exc_hangover;                                /* AC mode (GSC) - Hangover for the time contribution switching*/

    Word16 last_exc_dct_in_fx[L_FRAME];                     /* AC mode (GSC) - previous exciation                        */
    Word16 last_bitallocation_band_fx[MBANDS_GN];           /* AC mode (GSC) - previous bit allocation of each band      */


    Word32 past_PS_fx[HIGHEST_FBIN];
    Word16 past_ps_diff_fx;
    Word16 past_epsP2_fx;
    Word16 inact_cnt_fx;
    Word16 wdrop_fx;
    Word16 wdlp_0_95_sp_fx;
    Word16 sp_mus_state_fx;
    Word16 past_dec_fx[HANG_LEN];
    Word16 past_dlp_fx[HANG_LEN];                           /* Speech/music classifier - buffer of past non-binary decisions */
    Word16 last_lsp_fx[M_LSP_SPMUS];
    Word16 last_cor_map_sum_fx;
    Word16 last_non_sta_fx;
    Word16 past_log_enr_fx[NB_BANDS_SPMUS];                 /* Speech/music classifier - last average per-band log energy used for non_staX */

    Word16 gsc_last_bfi_count_fx;                           /* Speech/music classifier - counter for last frame erasure */
    Word16 mold_corr_fx;
    Word16 lt_gpitch_fx;                                    /*Q15 */
    Word16 mean_avr_dyn_fx;                                 /* Q7 Speech/music classifier - long term average dynamic */
    Word16 last_sw_dyn_fx;                                  /* Q7 Speech/music classifier - last dynamic              */
    Word16 lt_dec_thres_fx;                                 /* Speech/music classifier - Long term speech/music thresold values */
    Word16 ener_RAT_fx;                                     /* Q15 Speech/music classifier - LF/to total energy ratio */
    Word16 lgBin_E_fx[L_FFT];                               /* Q8 per bin energy of two frames */

    /* speech/music classifier improvement parameters */
    Word16 old_Bin_E_fx[3*N_OLD_BIN_E];
    Word16 buf_flux_fx[BUF_LEN];
    Word16 buf_pkh_fx[BUF_LEN];
    Word16 buf_epsP_tilt_fx[BUF_LEN];
    Word16 buf_cor_map_sum_fx[BUF_LEN];
    Word16 buf_Ntonal_fx[BUF_LEN];
    Word16 buf_Ntonal2_fx[BUF_LEN];
    Word16 buf_Ntonal_lf_fx[BUF_LEN];
    Word16 buf_dlp_fx[10];
    Word16 onset_cnt_fx;
    Word16 buf_etot_fx[4];
    Word16 attack_hangover_fx;
    Word16 dec_mov_fx;
    Word16 dec_mov1_fx;
    Word16 mov_log_max_spl_fx;
    Word16 old_lt_diff_fx[2];
    Word16 UV_cnt1_fx;
    Word16 LT_UV_cnt1_fx;
    Word16 lpe_buf_fx[HANG_LEN_INIT];
    Word16 voicing_buf_fx[HANG_LEN_INIT];
    Word16 gsc_hangover_fx;
    Word16 sparse_buf_fx[HANG_LEN_INIT];
    Word16 hf_spar_buf_fx[HANG_LEN_INIT];
    Word16 LT_sparse_fx;
    Word16 gsc_cnt_fx;
    Word16 last_vad_spa_fx;

    Word16 lt_music_hangover_fx;
    Word16 tonality2_buf_fx[HANG_LEN_INIT];
    Word16 tonality3_buf_fx[HANG_LEN_INIT];
    Word16 LPCErr_buf_fx[HANG_LEN_INIT];
    Word16 lt_music_state_fx;
    Word16 lt_speech_state_fx;
    Word16 lt_speech_hangover_fx;

    Word16 Last_pulse_pos_fx;                               /* FEC - last position of the first glotal pulse in the frame */
    Word16 lsfoldbfi0_fx[M];                                /* FEC - LSF vector of the previous frame */
    Word16 lsfoldbfi1_fx[M];                                /* FEC - LSF vector of the past previous frame */
    Word16 lsf_adaptive_mean_fx[M];                         /* FEC - adaptive mean LSF vector for FEC */
    Word16 next_force_safety_net_fx;                        /* FEC - flag to force safety net in next frame */
    Word16 gsc_last_music_flag_fx;                          /* Speech/music classifier - last music flag */
    Word16 gsc_lt_diff_etot_fx[MAX_LT];                     /* Speech/music classifier - long-term total energy variation */
    Word16 gsc_thres_fx[4];                                 /* Speech/music classifier - classification threshold */
    Word16 gsc_nb_thr_1_fx;                                 /* Speech/music classifier - number of consecutives frames of level 1 */
    Word16 gsc_nb_thr_3_fx;                                 /* Speech/music classifier - number of consecutives frames of level 3 */
    Word16 gsc_mem_etot_fx;                                 /* Speech/music classifier - total energy memory  */

    Word16 old_S_fx[L_FFT/2];                               /* Q7 Tonal detector - prev. log-energy spectrum with subtracted floor */
    Word16 cor_map_fx[L_FFT/2];                             /* Q15 Tonal detector - LT correlation map */
    Word16 noise_char_fx;                                   /* Q11 Tonal detector - LT noise character */
    Word32 ave_enr2_fx[NB_BANDS];                           /* Q_new + Q_SCALE Tonal detector - LT average E per crit. band (for non_sta2) */
    Word16 act_pred_fx;                                     /* Q15 Tonal detector - prediction of speech activity from 0 to 1 (0-inactive, 1-active) */
    Word16 multi_harm_limit_fx;                             /* Q9 Tonal detector - adaptive threshold */
    Word32 enrO_fx[NB_BANDS];                               /*                 Noise estimator - previous energy per critical band */
    Word32 bckr_fx[NB_BANDS];                               /* Q_new + Q_SCALE Noise estimator - background noise estimation per critical band */
    Word32 ave_enr_fx[NB_BANDS];                            /* Q_new + Q_SCALE Noise estimator - long-term average energy per critical band */
    Word16 pitO_fx;                                         /* Q0 Noise estimator - previous open-loop pitch value */
    Word16 aEn_fx;                                          /* Q0 Noise estimator - noise estimator adaptation flag */
    Word16 totalNoise_fx;                                   /* Q8 Noise estimator - total noise energy */
    Word16 first_noise_updt_fx;                             /* Q0 Noise estimator - flag used to determine if the first noise update frame */
    Word16 harm_cor_cnt_fx;                                 /* Q0 Noise estimator - 1st memory counter of harm or correlation frame */
    Word16 bg_cnt_fx;                                       /* Q0 Noise estimator - pause length counter */
    Word16 prim_act_quick_fx;                               /* Noise estimator - primary activity quick */
    Word16 prim_act_slow_fx;                                /* Noise estimator - primary activity slow  */
    Word16 prim_act_fx;                                     /* Noise estimator - primary activity slow rise quick fall */
    Word16 prim_act_quick_he_fx;                            /* Noise estimator - primary activity quick */
    Word16 prim_act_slow_he_fx;                             /* Noise estimator - primary activity slow  */
    Word16 prim_act_he_fx;                                  /* Q15 Noise estimator - primary activity slow rise quick fall */
    Word16 Etot_l_fx;                                       /* Q8 Noise estimator - Track energy from below  */
    Word16 Etot_h_fx;                                       /* Q8 Noise estimator - Track energy from above  */
    Word16 Etot_l_lp_fx;                                    /* Q8 Noise estimator - Smoothed low energy      */
    Word16 Etot_last_fx;                                    /*Q8*/
    Word16 Etot_lp_fx;                             /* Q8 Noise estimator - Filtered input energy   */
    Word16 lt_tn_track_fx;                         /* Q15 */
    Word16 lt_tn_dist_fx;                          /* Q8*/
    Word16 lt_Ellp_dist_fx;                        /* Etot low lp same domain as  *Etot_l_lp, Q8 */
    Word16 lt_haco_ev_fx;                          /* Q15 */
    Word16 low_tn_track_cnt_fx;                    /* Q0 */
    Word16 epsP_0_2_lp_fx;                        /* Q12, all  epsP quotas , range  ]8.0 ..0]*/
    Word16 epsP_0_2_ad_lp_fx;
    Word16 epsP_2_16_lp_fx;
    Word16 epsP_2_16_lp2_fx;
    Word16 epsP_2_16_dlp_lp2_fx;                     /* Q12 */
    Word16  lt_aEn_zero_fx;                 /* Q15 */
    Word16 nb_active_frames_fx;
    Word16 hangover_cnt_fx;
    Word16 lp_speech_fx;
    Word16 Etot_v_h2_fx;

    Word16 sign_dyn_lp_fx;                                  /*Q8*/
    Word16 Opt_HE_SAD_ON_fx;
    Word16 nb_active_frames_he_fx;
    Word16 hangover_cnt_he_fx;
    Word16 nb_active_frames_HE_SAD_fx;
    Word16 hangover_cnt_HE_SAD_fx;

    /* should be L_var */
    Word32  L_vad_flag_reg_H_fx;
    Word32  L_vad_flag_reg_L_fx;
    Word32  L_vad_prim_reg_fx;
    Word16 vad_flag_cnt_50_fx;
    Word16 vad_prim_cnt_16_fx;

    Word16 hangover_cnt_dtx_fx;
    Word16 hangover_cnt_music_fx;


    Word16 bcg_flux_fx;
    Word16 soft_hangover_fx;
    Word16 voiced_burst_fx;
    Word16 bcg_flux_init_fx;
    Word16 voicing_old_fx;
    Word16 nb_active_frames_he1_fx;
    Word16 hangover_cnt_he1_fx;


    Word32 bckr_tilt_lt;

    Word16 var_SID_rate_flag_fx;                        /* CNG and DTX - flag for variable SID rate */
    Word32 lp_ener_fx;                                  /* CNG and DTX - low-pass filtered energy for CNG */
    Word16 cng_seed_fx;                                    /* CNG and DTX - seed for white noise random generator */
    Word16 lspCNG_fx[M];                                /* CNG and DTX - LP filtered lsps */
    Word16 first_CNG_fx;                                /* CNG and DTX - first CNG frame flag */
    Word16 lp_noise_fx;                                 /* CNG and DTX - LP filterend total noise estimation */
    Word16 cnt_SID_fx;                                  /* CNG and DTX - counter of SID update for the interop. mode or dtx, if enabled*/
    Word16 max_SID_fx;                                  /* CNG and DTX - max allowed number of CNG FRAME_NO_DATA frames */
    Word16 interval_SID_fx;                             /* CNG and DTX - interval of SID update, default 8 */
    Word16 old_enr_index_fx;                            /* CNG and DTX - index of last encoded CNG energy */
    Word32 Enew_fx;                                        /* CNG and DTX - CNG target residual energy */
    Word16 VarDTX_cnt_voiced_fx;                        /* CNG and DTX - counter for variable DTX activation (speech) */
    Word32 lt_ener_voiced_fx;                           /* CNG and DTX - long-term energy of signal (measured on voiced parts) */
    Word16 VarDTX_cnt_noise_fx;                         /* CNG and DTX - counter for variable DTX activation (noise) */
    Word32 lt_ener_noise_fx;                            /* CNG and DTX - long-term energy of background noise */
    Word32 lt_ener_last_SID_fx;                         /* CNG and DTX - long-term energy of last SID frame */
    Word16 cng_hist_size_fx;                            /* CNG and DTX - size of CNG history buffer for averaging, <0,DTX_HIST_SIZE> */
    Word16 cng_hist_ptr_fx;                                /* CNG and DTX - pointer for averaging buffers */
    Word16 cng_lsp_hist_fx[DTX_HIST_SIZE*M];            /* CNG and DTX - old LSP buffer for averaging */
    Word16 cng_ener_hist_fx[DTX_HIST_SIZE];             /* CNG and DTX - log energy buffer for averaging */
    Word16 cng_cnt_fx;                                  /* CNG and DTX - counter of CNG frames for averaging */
    Word16 cng_ener_seed_fx;                            /* CNG and DTX - seed for random generator for variation of excitation energy */
    Word16 cng_ener_seed1_fx;
    Word32 frame_ener_fx;
    Word16 lp_sp_enr_fx;                                /*Q8*/
    Word16 last_allow_cn_step_fx;
    Word16 ho_hist_size_fx;                             /* CNG and DTX - size of DTX hangover history buffer for averaging, <0,HO_HIST_SIZE> */
    Word16 ho_hist_ptr_fx;                              /* CNG and DTX - pointer for averaging buffers */
    Word16 ho_lsp_hist_fx[HO_HIST_SIZE*M];              /* CNG and DTX - old LSP buffer for averaging */
    Word32 ho_ener_hist_fx[HO_HIST_SIZE];               /* CNG and DTX - energy buffer for averaging */
    Word32 ho_env_hist_fx[HO_HIST_SIZE*NUM_ENV_CNG];
    Word16 act_cnt_fx;                                  /* CNG and DTX - counter of active frames */
    Word16 ho_circ_size_fx;                             /* CNG and DTX - size of DTX hangover history buffer for averaging, <0,HO_HIST_SIZE> */
    Word16 ho_circ_ptr_fx;                              /* CNG and DTX - pointer for averaging buffers */
    Word16 ho_lsp_circ_fx[HO_HIST_SIZE*M];              /* CNG and DTX - old LSP buffer for averaging */
    Word32 ho_ener_circ_fx[HO_HIST_SIZE];               /* CNG and DTX - energy buffer for averaging */
    Word32 ho_env_circ_fx[HO_HIST_SIZE*NUM_ENV_CNG];
    Word16 burst_ho_cnt_fx;                             /* CNG and DTX - counter of hangover frames at end of active burst */
    Word16 cng_buf_cnt;                                 /* CNG and DTX - Counter of buffered CNG parameters */
    Word16 cng_exc2_buf[HO_HIST_SIZE*L_FFT];            /* CNG and DTX - exc2 buffer for storing */
    Word16 cng_Qexc_buf[HO_HIST_SIZE];                  /* CNG and DTX - Q_exc buffer for storing */
    Word32 cng_brate_buf[HO_HIST_SIZE];                     /* CNG and DTX - buffer for storing last_active_brate */


    Word16 CNG_mode_fx;                                 /* CNG and DTX - mode for DTX configuration */
    Word32 last_active_brate_fx;                        /* CNG and DTX - last active frame bitrate used for CNG_mode control */
    Word16 ho_16k_lsp_fx[HO_HIST_SIZE];                 /* CNG and DTX - 16k LSPs flags */
    Word16 last_CNG_L_frame_fx;                         /* CNG and DTX - last CNG frame length */
    Word16 act_cnt2_fx;                                 /* CNG and DTX - counter of active frames for CNG_mode switching */
    Word16 ho_lsp_circ2_fx[HO_HIST_SIZE*M];             /* CNG and DTX - second buffer of LSPs */
    Word16 num_ho_fx;                                   /* CNG and DTX - number of selected hangover frames */
    Word16 hangover_terminate_flag_fx;                  /* CNG and DTX - flag indicating whether to early terminate DTX hangover */
    Word32 old_env_fx[NUM_ENV_CNG];
    Word32 lp_env_fx[NUM_ENV_CNG];
    Word32 cng_res_env_fx[NUM_ENV_CNG*HO_HIST_SIZE];
    Word16 exc_mem_fx[24];
    Word16 exc_mem1_fx[30];
    Word16 exc_mem2_fx[30];

    struct dispMem_fx dm_fx;                            /* Noise enhancer - phase dispersion algorithm memory */

    Word16  uv_count_fx;                        /*Q0*/  /* Stationary noise UV modification - unvoiced counter */
    Word16 act_count_fx;                        /*Q0*/  /* Stationary noise UV modification - activation counter */
    Word32 ge_sm_fx;                                    /* Stationary noise UV modification - smoothed excitation gain */
    Word16 lspold_s_fx[M];                      /*Q15*/ /* Stationary noise UV modification - old LSP vector */
    Word16 noimix_seed_fx;                      /*Q0*/  /* Stationary noise UV modification - mixture seed */
    Word16 min_alpha_fx;                        /*Q15*/ /* Stationary noise UV modification - minimum alpha */
    Word16 exc_pe_fx;                                   /* Stationary noise UV modification - memory of the preemphasis filter */

    Word16 last_L_frame_fx;                             /* ACELP@16kHz - last L_frame value */
    Word16 mem_decim16k_fx[2*L_FILT_MAX];               /* ACELP@16kHz - decimation filter memory @16kHz */
    Word16 mem_preemph16k_fx;                           /* ACELP@16kHz - preemphasis filter memory @16kHz */
    Word16 old_inp_16k_fx[L_INP_MEM];                   /* ACELP@16kHz - memory of input signal @16 kHz */
    Word16 mem_deemp_preQ_fx;                           /* ACELP@16kHz - prequantizer deemhasis memory */
    Word16 mem_preemp_preQ_fx;                          /* ACELP@16kHz - prequantizer preemhasis memory */
    Word16 last_nq_preQ_fx;                             /* ACELP@16kHz - AVQ subquantizer number of the last sub-band of the last subframe  */
    Word16 use_acelp_preq;                              /* ACELP@16kHz - flag of prequantizer usage */

    Word16 bpf_off_fx;
    Word16 old_pitch_buf_fx[2*NB_SUBFR16k];             /*Q6 Bass post-filter - buffer of old subframe pitch values */

    Word16 lps_fx;
    Word16 lpm_fx;

    /* stable short pitch detection */
    Word16 voicing0_sm_fx;
    Word16 voicing_sm_fx;
    Word16 LF_EnergyRatio_sm_fx;
    Word16 predecision_flag_fx;
    Word32 diff_sm_fx;
    Word32 energy_sm_fx;

    Word16 last_ener_fx;                                    /* AC mode (GSC) - previous energy                           */

    /*----------------------------------------------------------------------------------*
     * HF WB BWE for AMR-WB IO mode at 23.85 kbps
     *----------------------------------------------------------------------------------*/
    /*Word16 prev_HFcg;*/                                    /* HF BWE - previous HF correction gain in 23.85 mode */
    Word16 gain_alpha_fx;
    Word16 mem_hf2_enc_fx[2*L_FILT16k];
    Word16 mem_hp400_enc_fx[6];
    Word16 mem_hf_enc_fx[2*L_FILT16k];
    Word16 mem_syn_hf_enc_fx[M16k];
    Word16 seed2_enc_fx;

    /*----------------------------------------------------------------------------------*
     * CLDFB analysis
     *----------------------------------------------------------------------------------*/
    HANDLE_CLDFB_FILTER_BANK cldfbAna_Fx;

    HANDLE_CLDFB_FILTER_BANK cldfbSyn_Fx;

    /*----------------------------------------------------------------------------------*
     * FD CNG handle
     *----------------------------------------------------------------------------------*/
    HANDLE_FD_CNG_ENC hFdCngEnc_fx;
    Word16 fd_cng_reset_flag;
    Word16 last_totalNoise_fx;
    Word16 totalNoise_increase_hist_fx[TOTALNOISE_HIST_SIZE];
    Word16 totalNoise_increase_len_fx;
    /*----------------------------------------------------------------------------------*
     * SC-VBR parameters
     *----------------------------------------------------------------------------------*/

    Word16 vadsnr_fx;                            /*Q7*/
    Word16 vadnoise_fx;

    /* NELP variables */
    Word16 shape1_filt_mem_fx[20];
    Word16 shape2_filt_mem_fx[20];
    Word16 shape3_filt_mem_fx[20];
    Word16 txlpf1_filt1_mem_fx[20];
    Word16 txlpf1_filt2_mem_fx[20];
    Word16 txhpf1_filt1_mem_fx[20];
    Word16 txhpf1_filt2_mem_fx[20];
    Word16 bp1_filt_mem_wb_fx[8];
    Word32 bp1_filt_mem_nb_fx[14];
    Word16 nelp_lp_fit_mem[NELP_LP_ORDER*2];
    Word16 nelp_enc_seed_fx;
    Word16 nelp_gain_mem_fx;
    Word16 last_nelp_mode_fx;
    Word16 nelp_mode_fx;
    Word16 qprevIn_fx;
    Word16 qprevGain_fx;

    /* PPP variables */
    Word16 pppcountE_fx;
    Word16 bump_up_fx;                            /*Q0*/
    Word16 last_ppp_mode_fx;                    /*Q0*/
    Word16 ppp_mode_fx;
    Word16 prev_ppp_gain_pit_fx;                /*Q14*/
    Word16 prev_tilt_code_fx;

    /* voiced encoder variables */
    Word16   firstTime_voicedenc_fx;            /*Q0*/

    /* DTFS variables */
    Word16 dtfs_enc_a_fx[MAXLAG_WI];            /*Q0*/
    Word16 dtfs_enc_b_fx[MAXLAG_WI];            /*Q0*/
    Word16 dtfs_enc_lag_fx;
    Word16 dtfs_enc_nH_fx;
    Word16 dtfs_enc_nH_4kHz_fx;
    Word16 dtfs_enc_upper_cut_off_freq_of_interest_fx;
    Word16 dtfs_enc_upper_cut_off_freq_fx;
    Word16 dtfs_enc_Q;

    Word32 prev_cw_en_fx;                        /*Q_prev_cw_en_fx*/
    Word16 ph_offset_E_fx;                        /*Q15 normalized y 2*Pi   */
    Word16 lastLgainE_fx;                       /*Q11 Previous gain value for the low band */
    Word16 lastHgainE_fx;                       /*Q11 Previous gain value for the high band */
    Word16 lasterbE_fx[NUM_ERB_WB];             /*Q13 Previous Amplitude spectrum (ERB) */
    Word16 Q_prev_cw_en_fx;

    Word16 mode_QQF_fx;
    Word16 rate_control_fx;
    Word16 SNR_THLD_fx;
    Word16 Q_to_F_fx;
    Word16 pattern_m_fx;
    Word16 patterncount_fx;
    Word16 Last_Resort_fx;
    Word16 numactive_fx;                    /* keep the count of the frames inside current 600 frame bloack.*/
    Word32 sum_of_rates_fx;                    /* sum of the rates of past 600 active frames*/
    Word32 global_avr_rate_fx;                /* global rate upto current time. recorded a (rate in kbps) *6000*/
    Word16 frame_cnt_ratewin_fx;            /* keep count of how many ratewin (600) windows */

    Word16 set_ppp_generic_fx;
    Word16 avoid_HQ_VBR_NB;

    /*----------------------------------------------------------------------------------*
     * HQ core parameters
     *----------------------------------------------------------------------------------*/
    Word16 input_buff[L_FRAME48k+L_FRAME48k+NS2SA(48000, DELAY_FIR_RESAMPL_NS)];
    Word16 * input;
    Word16 * old_input_signal_fx;
    Word16 Q_old_wtda;
    Word16 old_hpfilt_in_fx;
    Word16 old_hpfilt_out_fx;
    Word32 EnergyLT_fx;
    Word32 Energy_Old_fx;
    Word16 TransientHangOver_fx;
    Word16 old_out_fx[L_FRAME48k];
    Word16 Q_old_out;
    Word16 last_core_fx;
    Word16 hq_generic_speech_class_fx;
    Word16 mode_count_fx;                                 /* HQ_HARMONIC mode count                     */
    Word16 mode_count1_fx;                                /* HQ_NORMAL mode count                       */
    Word16 Nb_ACELP_frames_fx;
    Word16 prev_Npeaks_fx;                     /* number of peaks in previous frame */
    Word16 prev_peaks_fx[HVQ_MAX_PEAKS];       /* indices of the peaks in previous frame */
    Word16 hvq_hangover_fx;
    Word32 manE_peak_mem;
    Word16 expE_peak_mem;
    Word16 prev_hqswb_clas_fx;
    Word16 prev_SWB_peak_pos_fx[NI_USE_PREV_SPT_SBNUM];


    Word16 lt_old_mode[3];
    Word16 lt_voicing;
    Word16 lt_corr;
    Word32 lt_tonality;
    Word16 lt_corr_pitch[3];
    Word16 lt_hangover;
    Word16 lowrate_pitchGain;

    Word16 prev_frm_index_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB];
    Word16 prev_frm_hfe2_fx;
    Word16 prev_stab_hfe2_fx;
    Word16 prev_ni_ratio_fx;               /* 15 */
    Word16 prev_En_sb_fx[NB_SWB_SUBBANDS]; /* QsEn(4) */


    Word16 last_bitalloc_max_band_fx[BANDS_MAX];
    Word32 last_ni_gain_fx[BANDS_MAX];
    Word16 last_env_fx[BANDS_MAX];
    Word16 last_max_pos_pulse_fx;

    /* PVQ range coder state */
    UWord32 rc_low_fx;
    UWord32 rc_range_fx;
    Word16 rc_cache_fx;
    Word16 rc_carry_fx;
    Word16 rc_carry_count_fx;
    Word16 rc_num_bits_fx;
    Word16 rc_tot_bits_fx;
    Word16 rc_offset_fx;
    /*----------------------------------------------------------------------------------*
     * TBE parameters
     *----------------------------------------------------------------------------------*/

    Word16 old_speech_shb_fx[L_LOOK_16k + L_SUBFR16k];          /* Buffer memories */
    Word16 old_speech_wb_fx[(L_LOOK_12k8 + L_SUBFR) * 5/16];    /* Buffer memories */
    Word16 old_input_fhb_fx[NS2SA(48000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) - L_FRAME48k/2];
    Word16 state_ana_filt_shb_fx[ (2*ALLPASSSECTIONS_STEEP+1) ];
    Word16 cldfbHBLT;
    /* states for the filters used in generating SHB excitation from WB excitation*/
    Word32 mem_csfilt_fx[2];

    /* states for the filters used in generating SHB signal from SHB excitation*/
    Word16 state_syn_shbexc_fx[L_SHB_LAHEAD];
    Word16 state_lpc_syn_fx[LPC_SHB_ORDER];
    Word16 old_bwe_exc_fx[PIT16k_MAX * 2];                    /*Q_exc*/
    Word16 bwe_seed_fx[2];
    Word32 bwe_non_lin_prev_scale_fx;                        /*Q30*/
    Word16 old_bwe_exc_extended_fx[NL_BUFF_OFFSET];
    Word16 syn_overlap_fx[L_SHB_LAHEAD];
    Word16 decim_state1_fx[(2*ALLPASSSECTIONS_STEEP+1)];
    Word16 decim_state2_fx[(2*ALLPASSSECTIONS_STEEP+1)];
    Word16 mem_genSHBexc_filt_down_wb2_fx[(2*ALLPASSSECTIONS_STEEP+1)];
    Word16 mem_genSHBexc_filt_down_wb3_fx[(2*ALLPASSSECTIONS_STEEP+1)];
    Word16 mem_genSHBexc_filt_down_shb_fx[(2*ALLPASSSECTIONS_STEEP+1)];
    Word32 elliptic_bpf_2_48k_mem_fx[4][4];
    Word32 prev_fb_energy_fx;

    Word32 prev_gainFr_SHB_fx;
    Word16 lsp_shb_slow_interpl_fx[LPC_SHB_ORDER];
    Word16 lsp_shb_fast_interpl_fx[LPC_SHB_ORDER];
    Word16 shb_inv_filt_mem_fx[LPC_SHB_ORDER];
    Word16 lsp_shb_spacing_fx[3];
    Word16 prev_swb_GainShape_fx;
    Word16 prev_frGainAtten_fx;

    Word16 spectral_tilt_reset_fx;
    Word16 consec_inactive_fx;
    Word16 ra_deltasum_fx;
    Word16 trigger_SID_fx;
    Word16 running_avg_fx; /*Q15 */
    Word32 L_snr_sum_vad_fx; /*Q4*/
    Word16 prev_wb_GainShape;
    Word16 swb_lsp_prev_interp_fx[LPC_SHB_ORDER];
    Word16 fb_state_lpc_syn_fx[LPC_SHB_ORDER];
    Word16 fb_tbe_demph_fx;
    Word16 tilt_mem_fx;

    Word16 prev_coder_type_fx;
    Word16 prev_lsf_diff_fx[LPC_SHB_ORDER];
    Word16 prev_tilt_para_fx;
    Word16 cur_sub_Aq_fx[M+1];

    /* quantized data */
    Word16 lsf_idx_fx[NUM_Q_LSF];
    Word16 m_idx_fx;
    Word16 grid_idx_fx;
    Word16 idxSubGains_fx;
    Word16 idxFrameGain_fx;
    Word16 idx_shb_fr_gain_fx;
    Word16 idx_res_gs_fx[NB_SUBFR16k];
    Word16 idx_mixFac_fx;

    Word16 lsf_WB_fx;
    Word16 gFrame_WB_fx;

    Word16 idxGain_fx;
    Word16 dec_2_over_3_mem_fx[12];
    Word16 dec_2_over_3_mem_lp_fx[6];
    /*----------------------------------------------------------------------------------*
     * SWB BWE parameters
     *----------------------------------------------------------------------------------*/

    Word16 new_input_hp_fx[NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS)];
    Word16 old_input_fx[NS2SA(48000, DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS)];
    Word16 old_input_wb_fx[NS2SA(16000, DELAY_FD_BWE_ENC_NS)];
    Word16 old_input_lp_fx[NS2SA(16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_NS)];
    Word16 old_syn_12k8_16k_fx[NS2SA(16000, DELAY_FD_BWE_ENC_NS)];
    Word16 old_fdbwe_speech_fx[L_FRAME48k];
    Word16 mem_deemph_old_syn_fx;
    Word16 prev_mode_fx;
    Word16 L_old_wtda_swb_fx[L_FRAME48k];
    Word16 prev_Q_input_lp;
    Word16 prev_L_swb_norm1_fx;
    Word32 prev_global_gain_fx;
    Word16 modeCount_fx;
    Word32 EnergyLF_fx;
    Word32 prev_energy_fbe_fb_fx;
    Word16 tbe_demph_fx;
    Word16 tbe_premph_fx;
    Word16 mem_stp_swb_fx[LPC_SHB_ORDER];
    Word16 *ptr_mem_stp_swb_fx;
    Word16 gain_prec_swb_fx;
    Word16 mem_zero_swb_fx[LPC_SHB_ORDER];

    /*----------------------------------------------------------------------------------*
     * HR SWB BWE parameters
     *----------------------------------------------------------------------------------*/

    Word16 L_old_wtda_hr_fx[L_FRAME48k];

    /*----------------------------------------------------------------------------------*
     * WB, SWB and FB bandwidth detector
     *----------------------------------------------------------------------------------*/

    Word16 lt_mean_NB_fx;
    Word16 lt_mean_WB_fx;
    Word16 lt_mean_SWB_fx;
    Word16 count_WB_fx;
    Word16 count_SWB_fx;
    Word16 count_FB_fx;

    /*----------------------------------------------------------------------------------*
     * SWB DTX/CNG parameters
     *----------------------------------------------------------------------------------*/

    Word16 last_vad_fx;
    Word16 last_wb_cng_ener_fx;
    Word16 last_shb_cng_ener_fx;
    Word16 mov_wb_cng_ener_fx;
    Word16 mov_shb_cng_ener_fx;
    Word16 shb_cng_ini_cnt_fx;
    Word16 last_SID_bwidth_fx;
    Word16 shb_NO_DATA_cnt_fx;

    /*----------------------------------------------------------------------------------*
     * Channel-aware mode
     *----------------------------------------------------------------------------------*/


    Word16 rf_mode;
    Word16 rf_mode_last;
    Word16 Opt_RF_ON;
    Word16 rf_frame_type;

    Word16 rf_target_bits_write;
    Word16 rf_fec_offset;
    Word16 rf_fec_indicator;

    Word16 rf_targetbits_buff[MAX_RF_FEC_OFFSET];
    Word16 rf_indx_frametype[MAX_RF_FEC_OFFSET];

    Word16 rf_mem_w0;
    Word16 rf_clip_var[6];
    Word16 rf_tilt_code;
    Word16 rf_mem_syn2[M];
    struct dispMem_fx rf_dm_fx;
    Word32 rf_gc_threshold;

    Word16 rf_target_bits;
    Word16 rf_tilt_buf[NB_SUBFR16k];

    Word16 rf_indx_lsf[MAX_RF_FEC_OFFSET][3];
    Word16 rf_indx_pitch[MAX_RF_FEC_OFFSET][NB_SUBFR16k];
    Word16 rf_indx_fcb[MAX_RF_FEC_OFFSET][NB_SUBFR16k];
    Word16 rf_indx_gain[MAX_RF_FEC_OFFSET][NB_SUBFR16k];
    Word16 rf_indx_EsPred[MAX_RF_FEC_OFFSET];
    Word16 rf_indx_ltfMode[MAX_RF_FEC_OFFSET][NB_SUBFR16k];

    Word16 rf_indx_nelp_fid[MAX_RF_FEC_OFFSET];
    Word16 rf_indx_nelp_iG1[MAX_RF_FEC_OFFSET];
    Word16 rf_indx_nelp_iG2[MAX_RF_FEC_OFFSET][2];

    Word16 rf_indx_tbeGainFr[MAX_RF_FEC_OFFSET];

    Word16 rf_tcxltp_pitch_int_past;
    Word16 rf_last_tns_active;
    Word16 rf_second_last_tns_active;
    Word16 rf_second_last_core;
    Word16 rf_clas[MAX_RF_FEC_OFFSET];
    Word16 rf_gain_tcx[MAX_RF_FEC_OFFSET];
    Word16 rf_tcxltp_param[MAX_RF_FEC_OFFSET];

    Word16 rf_bwe_gainFr_ind;



    /*----------------------------------------------------------------------------------*
     * Fixed point only variables
     *----------------------------------------------------------------------------------*/

    Word16 prev_Q_bwe_exc;
    Word16 prev_Q_bwe_syn;
    Word16 Q_stat_noise_ge;
    Word16 Q_stat_noise;
    Word16 Q_syn2;
    Word16 Q_syn;
    Word16 Q_max[L_Q_MEM];
    Word16 Q_max_16k[L_Q_MEM];
    Word16 Q_old;
    Word16 prev_Q_old;
    Word16 old_wsp_max;                   /* Last weigthed speech maximal value     */
    Word16 old_wsp_shift;                 /* Last wsp scaling                       */
    Word16 prev_Q_new;
    Word16 prev_Q_shb;

    /*----------------------------------------------------------------------------------*
     * HR SWB BWE parameters
     *----------------------------------------------------------------------------------*/

    Word16 old_wtda_hr_fx_exp;
    Word16 EnergyLT_fx_exp;

    Word16 prev_lsp_shb_fx[LPC_SHB_ORDER];
    Word16 prev_lsp_wb_fx[LPC_SHB_ORDER_WB];
    Word16 prev_lpc_wb_fx[LPC_SHB_ORDER_WB];
    Word16 last_last_ppp_mode_fx;
    Word16 prev_lsp_wb_temp_fx[LPC_SHB_ORDER_WB];

    Word16 frame_size_index;      /* 0-FRAME_SIZE_NB-1: index determining the frame size */
    Word16 bits_frame_nominal;    /* avg bits per frame on active frame */
    Word16 bits_frame;            /* total bits per frame */
    Word16 bits_frame_core;       /* bits per frame for the core */
    Word8 narrowBand;

    /*ACELP config*/
    ACELP_config acelp_cfg;       /* configuration set for each frame */

    ACELP_config acelp_cfg_rf; /* configuration for RF frame */

    Word16 mode_index;            /* Mode Index for LPD core */

    /*TCX config*/
    TCX_config tcx_cfg;
    Word16 L_frameTCX;

    /* cod_main.c */
    Word16 mem_preemph_enc;              /* speech preemph filter memory (at encoder-sampling-rate) */

    Word16 *speech_TCX;
    Word16 *new_speech_TCX;

    Word16 *speech_enc;
    Word16 *speech_enc_pe;
    Word16 *new_speech_enc;
    Word16 *new_speech_enc_pe;
    Word16 buf_speech_enc[L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k];
    Word16 buf_speech_enc_pe[L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k];
    Word16 *wspeech_enc;
    Word16 buf_wspeech_enc[L_FRAME16k+L_SUBFR+L_FRAME16k+L_NEXT_MAX_16k]; /*normally there is a lookahead for 12k8 and 16k but L_FRAME_MAX=L_FRAME_16K+L_NEXT_16k*/
    Word16 *synth;
    Word16 buf_synth[OLD_SYNTH_SIZE_ENC+L_FRAME_MAX];

    /* Core Signal Analysis Outputs */
    Word16 noiseTiltFactor; /* compensation for LPC tilt in noise filling  */
    Word16 noiseLevelMemory;  /* counter of consecutive low TCX noise levels */
    STnsData tnsData[2];
    Word8 fUseTns[2];

    Word8 enableTcxLpc; /* global toggle for the TCX LPC quantizer */
    Word16 envWeighted;  /* are is{p,f}_old_q[] weighted or not? */

    Word8 acelpEnabled; /* Flag indicating if ACELP can be used */
    Word8 tcx10Enabled; /* Flag indicating if TCX 10 can be used */
    Word8 tcx20Enabled; /* Flag indicating if TCX 20 can be used */

    Word16 tcxMode; /* Chosen TCX mode for this frame */

    LPD_state LPDmem;

    Word16 mem_wsp_enc;                   /* wsp vector memory */

    Word16  nb_bits_header_ace;          /* number of bits for the header */
    Word16  nb_bits_header_tcx;          /* number of bits for the header */

    /*Added by fcs : restrict the possible in EVS: 0 base 10 = d.c.b.a base 2*/
    /* a = 0/1 : ACELP on/off*/
    /* b = 0/1 : TCX20 on/off*/
    /* c = 0/1 : TCX40 on/off*/
    /* d = 0/1 : TCX80 on/off*/
    Word8 restrictedMode;

    /* Framing */
    Word16 nb_subfr;

    Word16 preemph_fac;   /*Preemphasis factor*/

    Word16 gamma;
    Word16 inv_gamma;

    TransientDetection transientDetection;
    Word16 transient_info[3];

    Word16 acelpFramesCount; /* Acelp frame counter. Counts upto 50 only !!! */

    Word16 prevTempFlatness_fx; /*  exponent is AVG_FLAT_E */

    Word32 prevEnergyHF_fx;
    Word32 currEnergyHF_fx;
    Word16 currEnergyHF_e_fx; /* exponent of currEnergyHF and prevEnergyHF */
    Word32 energyCoreLookahead_Fx;
    Word16 sf_energyCoreLookahead_Fx;

    /* lsf quantizer*/
    Word16 parcorr[2];
    Word16 parcorr_mid[2];

    Word16  lpcQuantization;
    Word16  numlpc;
    Word16 encoderLookahead_enc;
    Word16 encoderPastSamples_enc;
    Word16 encoderLookahead_FB;

    /* pitch_ol for adaptive lag window */
    Word16 old_pitch_la;               /* past open loop pitch lag from look-ahead  */
    Word16 old_voicing_la;             /* past open loop pitch gain from look-ahead */

    /* for DTX operation */
    Word16  vad_2nd_stage_fx;

    Word32 band_energies[2*NB_BANDS];     /* energy in critical bands without minimum noise floor MODE2_E_MIN */
    Word16 band_energies_exp;             /* exponent for energy in critical bands without minimum noise floor MODE2_E_MIN */

    Word8 tcxonly;

    Word16 Q_max_enc[2];

    Word16 finalVAD;
    Word8 flag_noisy_speech_snr;              /*encoder detector for noisy speech*/

    Word16 fscale;
    Word32 sr_core;
    Word32 last_sr_core;
    Word8  acelp_autocorr;                   /* Optimize acelp in 0 covariance or 1 correlation domain */

    Word16 pit_min;
    Word16 pit_fr1;
    Word16 pit_fr1b;
    Word16 pit_fr2;
    Word16 pit_max;
    Word16 pit_res_max; /* goes from 1 upto 6 (see core_enc_init.c: init_acelp()) */

    /* for FAC */
    Word16 L_frame_past;
    Word16 memQuantZeros[L_FRAME_PLUS];

    /*Adaptive BPF*/
    Word16 bpf_gain_param;
    Word16 bpf_T[NB_SUBFR16k];
    Word16 bpf_gainT[NB_SUBFR16k];

    struct MEM_BPF
    {
        Word16 noise_buf[2*L_FILT16k];
        Word16 error_buf[L_FILT16k];
        Word32 lp_error;
        Word32 lp_error_ener;
        Word16 noise_shift_old;
    } mem_bpf;

    /* TCX-LTP */
    Word8 tcxltp;
    Word16 tcxltp_pitch_int;
    Word16 tcxltp_pitch_fr;
    Word16 tcxltp_gain;
    Word16 tcxltp_pitch_int_past;
    Word16 tcxltp_pitch_fr_past;
    Word16 tcxltp_gain_past;
    Word16 tcxltp_norm_corr_past;
    Word16 buf_speech_ltp[L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k];
    Word16 *speech_ltp;
    Word16 *new_speech_ltp;
    Word16 tcxltp_bits;
    Word16 tcxltp_param[LTPSIZE];

    Word16 measuredBwRatio; /* measured bw; used for TCX noise-filling. 1Q14 */
    Word16 nmStartLine; /* Starting line for the noise measurement. Q0 */

    Word8 glr;
    Word16 glr_idx[2];
    Word32 gain_code[NB_SUBFR16k];
    Word32 mean_gc[2];
    Word16 prev_lsf4_mean;
    Word16 last_stab_fac;
    Word8 glr_reset;

    /*for rate switching*/
    Word16 rate_switching_reset; /*Rate switching flag requiring a reset of memories at least partially */
    Word16 rate_switching_reset_16kHz;

    Word16 enablePlcWaveadjust;
    Word16 Tonal_SideInfo;
    IGF_ENC_INSTANCE hIGFEnc;
    Word16 igf;

    Word16 seed_acelp;

    Word16 tcx_lpc_shaped_ari;

    PLC_ENC_EVS plcExt;

    Word16 tec_tfa;
    TEMPORAL_ENVELOPE_CODING_ENCODER_FX tecEnc;
    Word16 tec_flag;
    Word16 tfa_flag;
    Word32 tfa_enr[N_TEC_TFA_SUBFR];

    Word16 nTimeSlots;                     /* for CLDFB */

    T_CldfbVadState vad_st;

    Word16 vbr_generic_ho_fx;
    Word16 last_7k2_coder_type_fx;

    Word16 sharpFlag;

    Word16 Local_VAD;

} Encoder_State_fx;

/* Structure for storing correlations between ACELP codebook components and target */
typedef struct
{
    Word16 xx;   /* energy of target x */
    Word16 y1y1; /* energy of adaptive cbk contribution y1 */
    Word16 y2y2; /* energy of fixed cbk contribution y2 */
    Word16 xy1;  /* correlation of x and y1 */
    Word16 xy2;  /* correlation of x and y2 */
    Word16 y1y2; /* correlation of y1 and y2 */

    Word16 xx_e;   /* energy of target x */
    Word16 y1y1_e; /* energy of adaptive cbk contribution y1 */
    Word16 y2y2_e; /* energy of fixed cbk contribution y2 */
    Word16 xy1_e;  /* correlation of x and y1 */
    Word16 xy2_e;  /* correlation of x and y2 */
    Word16 y1y2_e; /* correlation of y1 and y2 */
} ACELP_CbkCorr;

#endif
