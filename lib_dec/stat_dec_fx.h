/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef STAT_DEC_FX_H
#define STAT_DEC_FX_H


#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"
#include "stat_com.h"


/*------------------------------------------------------------------------------------------*
 * Indice
 *------------------------------------------------------------------------------------------*/


/* decoder mode enums */
typedef enum _DEC_MODE
{
    DEC_NO_FRAM_LOSS         = 0x0,
    DEC_CONCEALMENT_EXT      = 0x1
} DEC_MODE;

typedef enum
{
    FRAMEMODE_NORMAL        = 0x0,   /**< frame available            */
    FRAMEMODE_MISSING       = 0x1    /**< frame missing => conceal   */
                              ,FRAMEMODE_FUTURE        = 0x2
} frameMode_fx;


struct dispMem_fx
{
    Word16 prev_state;            /*Q0 */
    Word32 prev_gain_code;        /*Q16 */
    Word16 prev_gain_pit[6];      /*Q14 */
};


/*ari.h*/
typedef struct
{
    Word32 low,high,vobf;
} TastatDec;

/*---------------------------------------------------------------*
 * IGF                                                           *
 *---------------------------------------------------------------*/
/* IGFSCFDecoder.h */
typedef struct
{
    Word16            bitsRead;  /* after a call bitsRead contains the number of bits consumed by the decoder */
    Word16            prev[64];  /* no more than 64 SCFs for the IGF energy envelope of one block, short or long */
    Word16            scfCountLongBlock;
    Word16            t;
    const Word16     *cf_se00;
    const Word16     *cf_se01;
    Word16            cf_off_se01;
    const Word16     *cf_se02;
    const Word16     *cf_off_se02;
    const Word16     *cf_se10;
    Word16            cf_off_se10;
    const Word16     *cf_se11;
    const Word16     *cf_off_se11;
    TastatDec         acState;
} IGFSCFDEC_INSTANCE, *IGFSCFDEC_INSTANCE_HANDLE;

/* IGFDec.h */
typedef struct igfdec_private_data_struct
{

    IGF_INFO                  igfInfo;
    /* envelope reconstruction: */
    Word32                    igf_sN[IGF_MAX_SFB];                                                /* Q31 | only with short blocks as static needed                    */
    Word16                    igf_sN_e[IGF_MAX_SFB];                                              /*     | exponent for igf_sN                                        */
    Word32                    igf_pN[IGF_MAX_SFB];                                                /* Q31 | only with short blocks as static needed                    */
    Word16                    igf_pN_e[IGF_MAX_SFB];                                              /*     | exponent for igf_sN                                        */
    Word16                    igf_curr[IGF_MAX_SFB];                                              /* Q0  | igf_curr = [0, 91], current igf energies from bitstream    */
    Word16                    igf_prev[IGF_MAX_SFB];                                              /* Q0  | igf_prev = [0, 91], needed for concealment or indepflag==0 */
    Word16                    igf_curr_subframe[IGF_MAX_SUBFRAMES][IGF_TRANS_FAK][IGF_MAX_SFB];   /*     | current igf energies per subframe                          */
    Word16                    igf_prev_subframe[IGF_MAX_SUBFRAMES][IGF_MAX_SFB];                  /*     | needed for concealment or indepflag==0                     */
    Word16                    igf_flatteningTrigger_subframe[IGF_MAX_SUBFRAMES];

    /* spectral whitening: */
    Word32                    pSpecFlat[IGF_MAX_GRANULE_LEN];                                     /* Q31 | MDCT spectrum before LPC shaping                           */
    Word16                    pSpecFlat_exp;                                                      /*     | exponent of pSpecFlat                                      */
    Word16                    currWhiteningLevel[IGF_MAX_TILES];                                  /* Q0  | currWhiteningLevel = [0, 2], whitening lvl from bitstream  */
    Word16                    prevWhiteningLevel[IGF_MAX_TILES];                                  /* Q0  | prevWhiteningLevel = [0, 2], needed for concealment        */
    Word16                    currWhiteningLevel_subframe[IGF_MAX_SUBFRAMES][IGF_MAX_TILES];
    Word16                    prevWhiteningLevel_subframe[IGF_MAX_SUBFRAMES][IGF_MAX_TILES];      /*     | needed for concealment                                     */

    Word32                    totalNoiseNrg;
    Word16                    n_noise_bands;
    Word16                    headroom_TCX_noise_white;
    Word16                    headroom_TCX_noise;

    Word32                    totalNoiseNrg_off;
    Word16                    n_noise_bands_off;

    /* IGF SCF decoding: */
    IGFSCFDEC_INSTANCE        hArithSCFdec;

    /* concealment: */
    Word16                    frameLossCounter;

} IGFDEC_PRIVATE_DATA,*IGF_DEC_PRIVATE_DATA_HANDLE;

typedef struct igfdec_instance_struct
{
    Word16          isIGFActive;
    Word16          infoIGFAllZero;
    Word16          infoIGFStopLine;
    Word16          infoIGFStartLine;
    Word16          infoIGFStopFreq;
    Word16          infoIGFStartFreq;
    Word16          infoTCXNoise[IGF_MAX_GRANULE_LEN];
    Word16          flag_sparse[N_MAX];
    Word32          virtualSpec[N_MAX];                       /* Q31 | buffer for temp flattening */
    Word16          virtualSpec_e;                            /*     | exponent of virtualSpec */
    Word16          flatteningTrigger;
    IGFDEC_PRIVATE_DATA igfData;
} IGFDEC_INSTANCE, *IGF_DEC_INSTANCE_HANDLE;

typedef struct
{
    Word16 FrameSize;
    Word16 Pitch_fx;
    Word8 T_bfi_fx;
    Word16 Transient[MAX_POST_LEN];
    Word16 TCX_Tonality[DEC_STATE_LEN];
    Word16 outx_new_n1_fx;
    Word16 nsapp_gain_fx;
    Word16 nsapp_gain_n_fx;
    Word32 data_reci2_fx[L_FRAME_MAX];
    Word16 data_reci2_scale;
    Word16 data_noise[2*L_FRAME_MAX];
    Word32 ener_mean_fx;
    Word32 ener_fx;
    Word16 zp_fx;
    Word16 recovery_gain;
    Word16 step_concealgain_fx;
    Word16 concealment_method;
    Word16 subframe_fx;
    Word32 nbLostCmpt;
    Word16 seed;
} T_PLCInfo;

/*---------------------------------------------------------------*
 * Structures for Tonal MDCT PLC                                 *
 *---------------------------------------------------------------*/
typedef enum
{
    TONALMDCTCONCEAL_OK = 0,

    __error_codes_start = -100,

    TONALMDCTCONCEAL_NSAMPLES_LARGER_THAN_MAXBLOCKSIZE,
    TONALMDCTCONCEAL_INVALIDPOINTER,
    TONALMDCTCONCEAL_UNEXPECTED_ERROR,

    __error_codes_end
} TONALMDCTCONCEAL_ERROR;

typedef struct
{
    Word16 nSamples;
    Word16 nSamplesCore;
    Word16 * spectralData;
    Word16 spectralData_exp;
    Word16 * scaleFactors;
    Word16 * scaleFactors_exp;
    Word16 scaleFactors_max_e;
    Word16 gain_tcx_exp;
    Word8 blockIsValid;
    Word16 blockIsConcealed;
    Word8 tonalConcealmentActive;
} blockData;

typedef struct
{
    Word16 numIndexes;
    Word16 indexOfTonalPeak[MAX_NUMBER_OF_IDX];
    Word16 lowerIndex[MAX_NUMBER_OF_IDX];
    Word16 upperIndex[MAX_NUMBER_OF_IDX];
    Word16 phaseDiff[MAX_NUMBER_OF_IDX]; /* This one can be stored with 16 bits in range 0..2*PI */
    Word16 phase_currentFramePredicted[MAX_NUMBER_OF_IDX*GROUP_LENGTH]; /* This one can be stored with 16 bits in range  [-pi;pi] 2Q13, but the code has to be adapted to use moduo(2*PI) after adding */
} TonalComponentsInfo;

typedef void (*ApplyScaleFactorsPointer)(Word16 const x[], Word16 lg, Word16 lg_total, Word16 const scaleFactors[], Word16 const scaleFactors_exp[], Word16 gains_max_exp, Word32 y[]);

struct tonalmdctconceal
{
    TCX_config * tcx_cfg;

    ApplyScaleFactorsPointer pApplyScaleFactors;
    Word16 * pMDSTData;
    Word16 nSamples;
    Word16 nSamplesCore;
    Word16 nNonZeroSamples;
    Word16 nScaleFactors;

    Word32 lastPitchLag;

    blockData lastBlockData;
    blockData secondLastBlockData;

    Word16 scaleFactorsBuffers[2][FDNS_NPTS]; /* Contains also global gain. */
    Word16 scaleFactorsBuffers_exp[2][FDNS_NPTS];
    Word16 spectralDataBuffers[2][L_FRAME_MAX]; /* 16 bits is enough, because it is stored before applying scale factors. Take care that power spectrum is also stored here. */
    Word16 timeDataBuffer[2*L_FRAME_MAX]; /* 16 bits is enough for TD signal */

    Word16 * lastPcmOut;
    Word16 * secondLastPcmOut;
    Word16 * secondLastPowerSpectrum;
    Word16   secondLastPowerSpectrum_exp;

    Word16 nFramesLost;

    TonalComponentsInfo * pTCI;
};

typedef struct tonalmdctconceal* TonalMDCTConcealPtr;

/*****************************************/
/*              STAT DEC                 */
/*****************************************/

/*fd_cng_dec.h*/
/* Arrays and variables specific to decoder */
typedef struct
{
    HANDLE_FD_CNG_COM hFdCngCom;

    Word16  olapBufferAna[320];
    Word16  olapBufferSynth2[FFTLEN];

    Word32  msPeriodog[NPART_SHAPING];                /* Periodogram */
    Word16  msPeriodog_exp;                           /* Common exponent for fft and cldfb energies */
    Word16  msPeriodog_exp_fft;
    Word16  msPeriodog_exp_cldfb;
    Word32  msBminWin[NPART_SHAPING];
    Word32  msBminSubWin[NPART_SHAPING];
    Word16  msPsd[NPART_SHAPING];                     /* Power Spectral Density estimate (i.e., smoothed periodogram) */
    Word16  msPsd_exp_fft;
    Word32  msAlpha[NPART_SHAPING];                   /* Optimal smoothing parameter */


    Word32  msMinBuf[MSNUMSUBFR*NPART_SHAPING];       /* Buffer of minima */
    Word32  msCurrentMinOut[NPART_SHAPING];
    Word32  msCurrentMin[NPART_SHAPING];
    Word32  msCurrentMinSubWindow[NPART_SHAPING];

    Word16  msLocalMinFlag[NPART_SHAPING];
    Word16  msNewMinFlag[NPART_SHAPING];

    Word16  msPsdFirstMoment[NPART_SHAPING];
    Word32  msPsdSecondMoment[NPART_SHAPING];
    Word16  msNoiseFloor[NPART_SHAPING];               /* Estimated noise floor */
    Word32  msNoiseEst[NPART_SHAPING];                 /* Estimated noise level */
    Word16  msNoiseEst_exp;

    Word16  npart_shaping;                             /* Number of partitions */
    Word16  nFFTpart_shaping;                          /* Number of hybrid spectral partitions */
    Word16  part_shaping[NPART_SHAPING];               /* Partition upper boundaries (band indices starting from 0) */
    Word16  midband_shaping[NPART_SHAPING];            /* Central band of each partition */
    Word16  psize_shaping[NPART_SHAPING];              /* Partition sizes */
    Word16  psize_shaping_norm[NPART_SHAPING];         /* Partition sizes, fractional variable */
    Word16  psize_shaping_norm_exp;                    /* Partition sizes exponent for fractional variable */
    Word16  psize_inv_shaping[NPART_SHAPING];          /* Inverse of partition sizes */
    Word32  bandNoiseShape[FFTLEN2];                   /* CNG spectral shape computed at the decoder */
    Word16  bandNoiseShape_exp;                        /* exponent of bandNoiseShape */
    Word32  partNoiseShape[NPART];                     /* CNG spectral shape computed at the decoder */
    Word16  partNoiseShape_exp;                        /* exponent of partNoiseShape */

    Word16   flag_dtx_mode;

    Word32   lp_speech;                                /* format: Q9.23 */
    Word32   lp_noise;                                 /* format: Q9.23 */

    Word16  msPeriodogBuf[MSBUFLEN*NPART_SHAPING];
    Word16  msPeriodogBufPtr;

    Word16  msLogPeriodog[NPART_SHAPING];
    Word16  msLogNoiseEst[NPART_SHAPING];
}
FD_CNG_DEC;
typedef FD_CNG_DEC *HANDLE_FD_CNG_DEC;


typedef struct Decoder_State_fx
{

    /*----------------------------------------------------------------------------------*
     * Common parameters
     *----------------------------------------------------------------------------------*/
    Word16 codec_mode;                                  /* MODE1 or MODE2 */
    Word16 mdct_sw_enable;                              /* MDCT switching enable flag */
    Word16 mdct_sw;                                     /* MDCT switching indicator */
    Word16 last_codec_mode;                             /* last used codec mode*/
    UWord16 *bit_stream_fx;
    Word16 next_bit_pos_fx;                             /* position of the next bit to be read from the bitstream */

    Word32 output_Fs_fx;                                /* output sampling rate Q0*/
    Word16 output_frame_fx;                             /* Output frame length Q0*/
    Word32 total_brate_fx;                              /* total bitrate in kbps of the codec Q0*/
    Word32 last_total_brate_fx;                         /* last total bitrate in kbps of the codec Q0*/
    Word16 core_fx;                                     /* core (ACELP_CORE or HQ_CORE) */
    Word32 core_brate_fx;                               /* core bitrate */
    Word32 last_core_brate_fx;                          /* previous frame core bitrate Q0*/
    Word16 extl_fx;                                     /* extension layer Q0*/
    Word16 last_extl_fx;                                /* previous extension layer Q0*/
    Word32 extl_brate_fx;                               /* extension layer bitrate */
    Word16 L_frame_fx;                                  /* ACELP core internal frame length */
    Word16 bwidth_fx;                                   /* encoded signal bandwidth */
    Word16 Opt_AMR_WB_fx;                               /* flag indicating AMR-WB IO mode Q0*/
    Word16 Opt_VOIP_fx;                                 /* flag indicating VOIP mode with JBM */
    Word16 ini_frame_fx;                                /* initialization frames counter */

    /*----------------------------------------------------------------------------------*
     * ACELP core parameters
     *----------------------------------------------------------------------------------*/

    Word16 old_exc_fx[L_EXC_MEM_DEC];                   /* old excitation Q_exc*/
    Word16 old_excFB_fx[L_FRAME48k];                    /* old excitation FB */
    Word16 lsp_old_fx[M];                               /* old LSP vector at the end of the frame Q15*/
    Word16 lsf_old_fx[M];                               /* old LSF vector at the end of the frame Q2.56*/
    Word32 offset_scale1_fx[MAX_NO_MODES][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure 1st 8-dim subvector Q0*/
    Word32 offset_scale2_fx[MAX_NO_MODES][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure 2nd 8-dim subvector Q0*/
    Word32 offset_scale1_p_fx[MAX_NO_MODES_p][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure, pred. case, 1st 8-dim subvector Q0*/
    Word32 offset_scale2_p_fx[MAX_NO_MODES_p][MAX_NO_SCALES+1]; /* offsets for LSF LVQ structure, pred. case, 2nd 8-dim subvector Q0*/
    Word16 no_scales_fx[MAX_NO_MODES][2];                       /* LSF LVQ structure Q0*/
    Word16 no_scales_p_fx[MAX_NO_MODES_p][2];                   /* LSF LVQ structure Q0*/
    Word16 tilt_code_fx;                                /* tilt of code Q15*/
    Word16 mem_syn2_fx[M];                              /* synthesis filter memory Q_syn*/
    Word16 mem_syn1_fx[M];                              /* synthesis filter memory (for core switching and FD BWE) */
    Word16 mem_syn3_fx[M];
    Word16 mem_deemph_fx;                               /* deemphasis filter memory Q_syn*/
    Word32 L_mem_hp_out_fx[5];                          /* hp filter memory for synthesis */
    Word16 mem_MA_fx[M];                                /* MA memory of LSF quantizer (past quantized residual)(Qx2.56)*/
    Word16 mem_AR_fx[M];                                /* AR memory of LSF quantizer (past quantized LSFs without mean)(Qx2.56) */
    Word16 stab_fac_fx;                                 /* LSF stability factor Q15*/
    Word16 stab_fac_smooth_fx;                          /* low-pass filtered stability factor Q15*/
    Word16 last_coder_type_fx;                          /* previous coder type Q0*/
    Word16 agc_mem_fx[2];                               /* memory of AGC for saturation control Q0*/
    Word16 past_qua_en_fx[GAIN_PRED_ORDER];             /* gain quantization memory (used also in AMR-WB IO mode) */
    Word16 mid_lsf_int_fx;
    Word16 safety_net_fx;

    Word16 seed_tcx_fx;                                 /* AC mode (GSC) - seed for noise fill Q0*/
    Word16 GSC_noisy_speech_fx;                         /* AC mode (GSC) - flag to indicate GSC osn SWB noisy speech */
    Word16 Last_GSC_noisy_speech_flag_fx;               /* AC mode (GSC) - mem of the past flag to indicate GSC osn SWB noisy speech */
    Word16 cor_strong_limit_fx;                         /* AC mode (GSC) - Indicator about high spectral correlation per band */
    Word16 old_y_gain_fx[MBANDS_GN];                    /* AC mode (GSC) - AR mem for low rate gain quantization  */
    Word16 noise_lev_fx;                                /* AC mode (GSC) - noise level Q0*/
    Word16 lt_ener_per_band_fx[MBANDS_GN];              /* Q12 */
    Word32 Last_frame_ener_fx;                          /* AC mode (GSC) - last frame energy  */
    Word16 Last_GSC_spectrum_fx[L_FRAME];               /* AC mode (GSC) - Last good GSC spectrum */
    Word16 Last_GSC_pit_band_idx_fx;                    /* AC mode (GSC) - Last pitch band index Q0*/
    Word16 last_exc_dct_in_fx[L_FRAME];                 /* AC mode (GSC) - previous excitation                       */
    Word16 last_ener_fx;                                /* AC mode (GSC) - previous energy                           */
    Word16 last_bitallocation_band_fx[MBANDS_GN];       /* AC mode (GSC) - previous bit allocation of each band      */

    Word32 gc_threshold_fx;                             /* Noise enhancer - threshold for gain_code Q16*/
    struct dispMem_fx dm_fx;                            /* Noise enhancer - phase dispersion algorithm memory */

    Word16 prev_r_fx;                                   /* HF BWE - previous sub-frame gain                */
    Word16 fmerit_w_sm_fx;                              /* HF BWE - fmerit parameter memory */
    Word16 frame_count_fx;                              /* HF BWE - frame count             */
    Word16 ne_min_fx;                                   /* HF BWE - minimum Noise gate - short-term energy */
    Word16 fmerit_m_sm_fx;                              /* HF BWE - memory of fmerit_m param               */
    Word16 voice_fac_amr_wb_hf;                         /* HF BWE - voice factor                           */
    Word16 unvoicing_fx;                                /* HF BWE - unvoiced parameter                     */
    Word16 unvoicing_sm_fx;                             /* HF BWE - smoothed unvoiced parameter            */
    Word16 unvoicing_flag_fx;                           /* HF BWE - unvoiced flag                          */
    Word16 voicing_flag_fx;                             /* HF BWE - voiced flag                            */
    Word16 start_band_old_fx;                           /* HF BWE - previous start point for copying frequency band */
    Word32 OptCrit_old_fx;                              /* HF BWE - previous criterion value for deciding the start point */

    Word16 seed2_fx;                                     /* HF (6-7kHz) BWE - seed for random signal generator Q0*/
    Word16 mem_hp400_fx[6];                              /* HF (6-7kHz) BWE - hp400 filter memory */
    Word16 mem_hf_fx[2*L_FILT16k];                       /* HF (6-7kHz) BWE - band-pass filter memory Q(-2-memExp1)*/
    Word16 mem_syn_hf_fx[M16k];                          /* HF (6-7kHz) BWE - synthesis filter memory Q0*/
    Word16 delay_syn_hf_fx[NS2SA(16000,DELAY_CLDFB_NS)]; /* HF (6-7kHz) BWE - To synchronise BWE content with postfiltered synthesis Q0*/
    Word16 mem_hp_interp_fx[2*L_FILT16k];                /* HF (6-7 kHz) BWE - interp. memory */

    Word16 unv_cnt_fx;                                   /* Stationary noise UV modification - unvoiced frame counter Q0*/
    Word16 uv_count_fx;                                  /* Stationary noise UV modification - unvoiced counter Q0*/
    Word16 act_count_fx;                                 /* Stationary noise UV modification - activation counter Q0*/
    Word32 ge_sm_fx;                                     /* Stationary noise UV modification - smoothed excitation gain Q(GE_SHIFT)*/
    Word16 lspold_s_fx[M];                               /* Stationary noise UV modification - old LSP vector Q15*/
    Word16 noimix_seed_fx;                               /* Stationary noise UV modification - mixture seed Q0*/
    Word16 min_alpha_fx;                                 /* Stationary noise UV modification - minimum alpha Q15*/
    Word16 Q_stat_noise;                                 /* Q of  Exc_pe                         */
    Word16 exc_pe_fx;                                    /* Stationary noise UV modification - scale (Q_stat_noise) */
    Word16 Q_stat_noise_ge;                              /* Q of  ge_sm_fx                        */

    Word16 bfi_fx;                                       /* FEC - bad frame indicator */
    Word16 prev_bfi_fx;                                  /* FEC - previous bad frame indicator Q0*/
    Word16 seed_fx;                                      /* FEC - seed for random generator for excitation Q0*/
    Word16 lp_ener_FER_fx;                               /* FEC - long-term active-signal average energy Q8*/
    Word16 last_good_fx;                                 /* FEC - clas of last good received Q0*/
    Word16 lp_gainc_fx;                                  /* FEC - low-pass filtered code gain Q3*/
    Word16 lp_gainp_fx;                                  /* FEC - low-pass filtered pitch gain Q14 */
    Word32 lp_ener_fx;                                   /* FEC - low-pass filtered energy Q6*/
    Word32 enr_old_fx;                                   /* FEC - energy of the concealed frame Q0*/
    Word16 bfi_pitch_fx;                                 /* FEC - pitch for FEC */
    Word16 bfi_pitch_frame_fx;                           /*FEC - frame length when pitch for FEC is saved Q0*/
    Word32 old_pitch_buf_fx[2*NB_SUBFR16k+2];            /* FEC - buffer of old subframe pitch values 15Q16 */
    Word16 upd_cnt_fx;                                   /* FEC - counter of frames since last update Q0*/
    Word16 old_enr_LP;                                   /* FEC - LP filter gain Q5*/
    Word16 lsfoldbfi0_fx[M];                             /* FEC - LSF vector of the previous frame (Qx2.56)*/
    Word16 lsfoldbfi1_fx[M];                             /* FEC - LSF vector of the past previous frame (Qx2.56) */
    Word16 lsf_adaptive_mean_fx[M];                      /* FEC - adaptive mean LSF vector for FEC (Qx2.56)*/
    Word16 decision_hyst_fx;                             /* FEC - hysteresis of the music/speech decision Q0*/
    Word16 old_exc2_fx[L_EXC_MEM];                       /* FEC - old excitation2 used in fast recovery */
    Word16 old_syn2_fx[L_EXC_MEM];                       /* FEC - old syn speech used in fast recovery */
    Word16 relax_prev_lsf_interp_fx;
    Word16 mem_syn_clas_estim_fx[L_SYN_MEM_CLAS_ESTIM];  /* FEC - memory of the synthesis signal for frame class estimation */
    Word16 tilt_swb_fec_fx;                              /* FEC - SWB TBE TILT */

    Word16 cng_seed_fx;                    /*CNG and DTX - seed for white noise random generator*/
    Word16 lspCNG_fx[M];                   /* CNG and DTX - LP filtered ISPs Q15*/
    Word16 first_CNG_fx;                                    /* CNG and DTX - first CNG frame flag Q0*/
    Word32 Enew_fx;                        /* CNG and DTX - decoded residual energy Q6*/
    Word16 old_enr_index_fx;                                /* CNG and DTX - index of last encoded CNG energy Q0*/
    Word16 cng_ener_seed_fx;                /*CNG and DTX - seed for random generator for variation of excitation energyQ0*/
    Word16 cng_ener_seed1_fx;
    Word16 last_allow_cn_step_fx;              /*Q0*/
    Word16 ho_hist_size_fx;                                 /* CNG and DTX - size of DTX hangover history buffer for averaging, <0,HO_HIST_SIZE> */
    Word16 ho_hist_ptr_fx;                                  /* CNG and DTX - pointer for averaging buffers */
    Word16 ho_lsp_hist_fx[HO_HIST_SIZE*M];                  /* CNG and DTX - old LSP buffer for averaging */
    Word32 ho_ener_hist_fx[HO_HIST_SIZE];                   /* CNG and DTX - energy buffer for averaging */ /*Q6 */
    Word32 ho_env_hist_fx[HO_HIST_SIZE*NUM_ENV_CNG];
    Word16 act_cnt_fx;                                      /* CNG and DTX - counter of active frames */
    Word16 ho_circ_size_fx;                                 /* CNG and DTX - size of DTX hangover history buffer for averaging, <0,HO_HIST_SIZE> */
    Word16 ho_circ_ptr_fx;                                  /* CNG and DTX - pointer for averaging buffers */
    Word16 ho_lsp_circ_fx[HO_HIST_SIZE*M];                  /* CNG and DTX - old LSP buffer for averaging */
    Word32 ho_ener_circ_fx[HO_HIST_SIZE];                   /* CNG and DTX - energy buffer for averaging */ /* Q6 */
    Word32 ho_env_circ_fx[HO_HIST_SIZE*NUM_ENV_CNG];
    Word16 num_ho_fx;                                       /* DTX/CNG - number of selected hangover frames */
    Word16 ho_16k_lsp_fx[HO_HIST_SIZE];                     /* DTX/CNG - 16k LSPs flags */
    Word16 CNG_mode_fx;                                     /* DTX/CNG - mode for DTX configuration */
    Word32 last_active_brate_fx;                            /* DTX/CNG - last active frame bitrate used for CNG_mode control */
    Word16 last_CNG_L_frame_fx;                             /* DTX/CNG - last CNG frame length */
    Word16 act_cnt2_fx;                                     /* DTX/CNG - counter of active frames for CNG_mode switching */
    Word16 cng_type_fx;                                     /* DTX/CNG - flag indicating LP or CLDFB based SID/CNG */
    Word16 last_cng_type_fx;                                /* DTX/CNG - flag indicating last frame LP or CLDFB based SID/CNG */
    Word32 old_env_fx[20];
    Word32 lp_env_fx[20];
    Word16 exc_mem_fx[24];
    Word16 exc_mem1_fx[30];

    Word16 bpf_off_fx;                                      /* Bass post-filter - do not use BPF when this flag is set to 1 Q0*/
    Word16 pst_old_syn_fx[NBPSF_PIT_MAX];                   /* Bass post-filter - old synthesis buffer 1 Q_syn2-1*/
    Word16 pst_mem_deemp_err_fx;                            /* Bass post-filter - filter memory of noise LP filter for R3-R5 signal Q_syn2-1*/
    Word16 pst_lp_ener_fx;                                  /* Bass post-filter - long-term energy of R3-R5 synth Q8*/
    Word16 Track_on_hist_fx[L_TRACK_HIST];                  /* Bass post-filter - History of half frame usage */
    Word16 vibrato_hist_fx[L_TRACK_HIST];                   /* Bass post-filter - History of frames declared as vibrato */
    Word16 psf_att_fx;                                      /* Bass post-filter - post filter attenuation factor */
    Word16 mem_mean_pit_fx[L_TRACK_HIST];                   /* Bass post-filter - average pitch memory */

    Word16 Ng_ener_ST_fx;                                   /* Noise gate - short-term energy Q8*/

    Word16 last_L_frame_fx;                                 /* ACELP@16kHz - last value of st->L_frame */
    Word16 mem_preemp_preQ_fx;                              /* ACELP@16kHz - prequantizer preemhasis memory */
    Word16 last_nq_preQ_fx;                                 /* ACELP@16kHz - AVQ subquantizer number of the last sub-band of the last subframe  */
    Word16 use_acelp_preq;                                  /* ACELP@16kHz - flag of prequantizer usage */

    /* Improvement of unvoiced and audio signals in AMR-WB IO mode */
    Word16 UV_cnt_fx;                                       /* number of consecutives frames classified as UV */
    Word16 LT_UV_cnt_fx;                                    /* long-term consecutives frames classified as UV */
    Word16 Last_ener_fx;                                    /* last_energy frame                              */
    Word16 lt_diff_etot_fx[MAX_LT];                         /* stability estimation - long-term total energy variation */
    Word16 old_Aq_fx[68];                                   /* old LPC filter coefficient                     */
    Word16 lt_voice_fac_fx;                                 /* average voice factor over 4 sub-frames         */

    /*----------------------------------------------------------------------------------*
     * SC-VBR
     *----------------------------------------------------------------------------------*/

    Word16 last_ppp_mode_dec_fx;                /*Q0*/
    Word16 ppp_mode_dec_fx;                     /*Q0*/
    Word16 last_nelp_mode_dec_fx;
    Word16 nelp_mode_dec_fx;                    /* Q0 */
    Word16 firstTime_voiceddec_fx;              /*Q0*/

    /* DTFS variables */
    Word16 dtfs_dec_a_fx[MAXLAG_WI];            /*Variable Q format in dtfs_dec_Q*/
    Word16 dtfs_dec_b_fx[MAXLAG_WI];            /*Variable Q format in dtfs_dec_Q*/
    Word16 dtfs_dec_lag_fx;
    Word16 dtfs_dec_nH_fx;
    Word16 dtfs_dec_nH_4kHz_fx;                 /*Q0*/
    Word16 dtfs_dec_upper_cut_off_freq_of_interest_fx;  /*Q0*/
    Word16 dtfs_dec_upper_cut_off_freq_fx;      /*Q0*/
    Word16 ph_offset_D_fx;                      /* normalized by 2Pi Q15*/
    Word16 lastLgainD_fx;                       /* previous gain value for the low band Q11*/
    Word16 lastHgainD_fx;                       /* previous gain value for the high band Q11 */
    Word16 lasterbD_fx[NUM_ERB_WB];             /* previous amplitude spectrum (ERB) Q13*/
    Word16 dtfs_dec_Q;                          /*Q0*/

    /* NELP decoder variables */
    Word32 bp1_filt_mem_nb_dec_fx[14];          /* qfm currently Q0*/
    Word16 bp1_filt_mem_wb_dec_fx[8];           /* qfm currently Q0*/
    Word16 shape1_filt_mem_dec_fx[20];          /* qfm currently Q0*/
    Word16 shape2_filt_mem_dec_fx[20];          /* qfm currently Q0*/
    Word16 shape3_filt_mem_dec_fx[20];          /* qfm currently Q0*/
    Word16 nelp_dec_seed_fx;                    /* Q0*/
    Word16 gainp_ppp_fx;                        /*Q14*/
    Word16 FadeScale_fx;                        /*Q15*/
    Word16 prev_gain_pit_dec_fx;                /*Q14*/
    Word16 prev_tilt_code_dec_fx;               /*Q15*/

    /*----------------------------------------------------------------------------------*
     * channel-aware mode
     *----------------------------------------------------------------------------------*/

    Word16 tilt_code_dec_fx[NB_SUBFR16k];
    Word32 gain_code_fx[NB_SUBFR16k];

    Word16 rf_frame_type;
    Word16 use_partial_copy;
    Word16 prev_use_partial_copy;
    Word16 rf_flag;
    Word16 rf_flag_last;

    Word16 rf_fec_offset;
    Word16 next_coder_type;
    Word16 prev_rf_frame_type;
    Word16 rf_target_bits;

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
    /*----------------------------------------------------------------------------------*
     * HR SWB BWE parameters
     *----------------------------------------------------------------------------------*/

    Word16 old_out_hr_fx[L_FRAME48k];
    Word16 old_out_hr_exp_fx;
    Word16 bwe_highrate_seed_fx;
    Word16 t_audio_prev_fx[2*END_FREQ_BWE_FULL_FB/50 - NUM_NONTRANS_START_FREQ_COEF];
    Word16 t_audio_prev_fx_exp[NUM_TIME_SWITCHING_BLOCKS];
    Word16 old_is_transient_hr_bwe_fx;
    Word32 L_mem_EnergyLT_fx;
    Word16 mem_EnergyLT_fx_exp;

    /*----------------------------------------------------------------------------------*
     * HQ core parameters
     *----------------------------------------------------------------------------------*/

    Word16 synth_history_fx[L_FRAME48k+OLD_SYNTH_SIZE_DEC+NS2SA(48000, PH_ECU_LOOKAHEAD_NS)];            /* unified synthesis memory */
    Word16 *old_synthFB_fx;
    Word16 old_out_fx[L_FRAME48k];                          /* HQ core - previous synthesis for OLA */

    Word16 old_out_LB_fx[L_FRAME48k];                          /* HQ core - previous synthesis for OLA for Low Band */
    Word16 Q_old_wtda_LB;
    Word16 Q_old_wtda;
    Word16 Q_old_postdec;                                   /*scaling of the output of core_switching_post_dec_fx() */
    Word16 Qprev_synth_buffer_fx;
    Word32 oldIMDCTout_fx[L_FRAME8k];
    Word16 prev_oldauOut_fx[L_FRAME8k];
    Word16 old_auOut_2fr_fx[L_FRAME8k*2];
    Word16 old_out_pha_fx[2][N_LEAD_NB];              /* FEC for HQ Core, 0-phase matching old_out, 1-overlapping original old_out and  phase matching old_out*/
    Word16 diff_energy_fx;
    Word32 old_coeffs_fx[L_FRAME8k];                      /* HQ core - old coefficients (for FEC) */
    Word16 stat_mode_out_fx;
    Word16 stat_mode_old_fx;
    Word16 phase_mat_flag_fx;
    Word16 phase_mat_next_fx;
    Word16 old_Min_ind_fx;
    Word16 old_is_transient_fx[3];                          /* HQ core - previous transient flag (for FEC)  */
    Word16 old_bfi_cnt_fx;                                  /* HQ core - # of bfi until previous frame(for FEC)  */
    Word16 prev_old_bfi_fx;
    Word32 ynrm_values_fx[MAX_SB_NB][MAX_PGF];
    Word32 r_p_values_fx[MAX_SB_NB][MAX_ROW];
    /*Word16 old_hqswb_clas;*/   /* only used in inactive code, where it might probably be replaced by old_hqswb_clas_fx */
    Word16 Norm_gain_fx[NB_SFM];
    Word16 HQ_FEC_seed_fx;
    Word16 energy_MA_Curr_fx[2];
    Word16 last_core_fx;                  /*Q0*/

    Word16 last_hq_core_type_fx;                  /*Q0*/
    Word16 last_L_frame_ori_fx;
    Word16 previoussynth_fx[L_FRAME48k];
    Word16 old_synth_sw_fx[L_FRAME48k/2];
    Word16 delay_buf_out_fx[HQ_DELTA_MAX*HQ_DELAY_COMP];      /*Q0*/
    Word16 mem_norm_fx[NB_SFM];                             /* Q0 */
    Word16 mem_env_delta_fx;                                /* Q11 */
    Word16 no_att_hangover_fx;                              /* Q0 */
    Word32 energy_lt_fx;                                    /* Q13 */
    Word16 hq_generic_seed_fx;
    Word32 manE_peak_mem;
    Word16 expE_peak_mem;
    Word16 prev_noise_level_fx[2];                          /* Q15 */
    Word16  prev_hqswb_clas_fx;
    Word16 prev_R_fx[SFM_N];                                /* the table of bit allocation of last frame  */
    Word32 prev_coeff_out_fx[L_FRAME32k];                   /* Q12 */ /* the coefficients of last frame             */
    Word16 prev_SWB_peak_pos_fx[NI_USE_PREV_SPT_SBNUM];
    Word16 old_Aq_12_8_fx[M+1];                             /* Q12 old Aq[] for core switching */
    Word16 old_Es_pred_fx;                                  /* old Es_pred for core switching */


    Word16 HqVoicing_fx;
    Word16 fer_samples_fx[L_FRAME48k];
    Word32 prev_normq_fx[SFM_N_WB];                         /* Q14 */ /* previous norms                             */
    Word32 prev_env_fx[SFM_N_WB];                           /* previous noise envelopes                   */

    Word32 last_ni_gain_fx[BANDS_MAX];
    Word16 last_env_fx[BANDS_MAX];
    Word16 last_max_pos_pulse_fx;

    /* pre-echo reduction */
    Word16 memfilt_lb_fx; /* Q0 */
    Word32 mean_prev_hb_fx; /* Q0 */
    Word16 smoothmem_fx; /* Q15 */
    Word32 mean_prev_fx; /* Q0 */
    Word32 mean_prev_nc_fx; /* Q0 */
    Word16 wmold_hb_fx; /* Q15 */
    Word16 prevflag_fx; /* Q0 */
    Word16 pastpre_fx; /* Q0 */

    Word16 prev_frm_hfe2_fx;
    Word16 prev_stab_hfe2_fx;
    Word16 prev_ni_ratio_fx;               /* 15 */
    Word16 prev_En_sb_fx[NB_SWB_SUBBANDS]; /* QsEn(4) */

    /* PVQ range coder state */
    UWord32 rc_low_fx;
    UWord32 rc_range_fx;
    UWord32 rc_help_fx;
    Word16 rc_num_bits_fx;
    Word16 rc_offset_fx;
    Word16 rc_end_fx;

    Word16 prev_env_Q[SFM_N_WB];

    /*----------------------------------------------------------------------------------*
     * TBE parameters
     *----------------------------------------------------------------------------------*/

    /* states for the filters used in generating SHB excitation from WB excitation */
    Word16 state_lpc_syn_fx[LPC_SHB_ORDER];
    Word32 mem_csfilt_fx [2];

    /* states for the filters used in generating SHB signal from SHB excitation*/
    Word16 state_syn_shbexc_fx[L_SHB_LAHEAD];
    Word16 syn_overlap_fx[L_SHB_LAHEAD];                    /* overlap buffer used to Adjust SHB Frame Gain*/

    /* previous frame parameters for frame error concealment */
    Word16 lsp_prevfrm_fx[ LPC_SHB_ORDER];
    Word32 GainFrame_prevfrm_fx;
    Word16 GainAttn_fx;

    Word16 old_bwe_exc_fx[PIT16k_MAX * 2];          /*Q_exc*/
    Word16 bwe_seed_fx[2];                  /*Q0*/
    Word32 bwe_non_lin_prev_scale_fx;
    Word16 old_bwe_exc_extended_fx[NL_BUFF_OFFSET];
    Word16 last_voice_factor_fx;              /* Q6*/

    Word32 genSHBsynth_Hilbert_Mem_fx[HILBERT_MEM_SIZE];
    Word16 mem_genSHBexc_filt_down_shb_fx[2*ALLPASSSECTIONS_STEEP+1];
    Word16 mem_genSHBexc_filt_down_wb2_fx[2*ALLPASSSECTIONS_STEEP+1];
    Word16 mem_genSHBexc_filt_down_wb3_fx[2*ALLPASSSECTIONS_STEEP+1];
    Word16 genSHBsynth_state_lsyn_filt_shb_local_fx[ 2 * L_FILT16k ];
    Word16 state_lsyn_filt_shb_fx[ 2 * L_FILT16k ];
    Word16 state_lsyn_filt_dwn_shb_fx[ 2 * L_FILT16k ];
    Word16 state_32and48k_WB_upsample_fx[2 * L_FILT32k];    /* !!! this memory in FLP is called mem_resamp_HB */
    Word16 hb_prev_synth_buffer_fx[NS2SA(48000, DELAY_BWE_TOTAL_NS)];
    Word16 old_bwe_delay_fx;                /*Q0*/

    Word16 syn_dm_phase_fx;
    Word32 fbbwe_hpf_mem_fx[4][4];
    Word32 prev_wb_bwe_frame_pow_fx;
    Word32 prev_swb_bwe_frame_pow_fx;
    Word32 prev_ener_fx;
    Word16 prev_ener_fx_Q;
    Word16 prev_GainShape_fx;
    Word16 fb_state_lpc_syn_fx[LPC_SHB_ORDER];
    Word16 fb_tbe_demph_fx;
    Word16 prev_fbbwe_ratio_fx;

    /* WB/SWB bandwidth switching */
    Word16 tilt_wb_fx;
    Word16 tilt_swb_fx;
    Word16 prev_ener_shb_fx;
    Word32 enerLH_fx;
    Word32 prev_enerLH_fx;
    Word32 enerLL_fx;
    Word32 prev_enerLL_fx;

    Word16 prev_fractive_fx;
    Word16 bws_cnt_fx;
    Word16 bws_cnt1_fx;
    Word16 attenu_fx;
    Word16 last_inner_frame_fx;                             /* (HQ_CORE) DCT length */
    Word16 last_hq_tilt_fx;
    Word16 last_bwidth_fx;
    Word16 prev_weight1_fx;
    Word16 t_audio_q_fx[L_FRAME16k];
    Word16 tbe_demph_fx;
    Word16 tbe_premph_fx;
    Word16 mem_stp_swb_fx[LPC_SHB_ORDER];
    Word16 *ptr_mem_stp_swb_fx;
    Word16 gain_prec_swb_fx;
    Word16 mem_zero_swb_fx[LPC_SHB_ORDER];

    Word16 swb_lsp_prev_interp_fx[LPC_SHB_ORDER];
    Word32 prev1_shb_ener_sf_fx, prev2_shb_ener_sf_fx, prev3_shb_ener_sf_fx;
    Word16 prev_res_shb_gshape_fx, prev_mixFactors_fx;
    Word16 tilt_mem_fx;                                     /* Formant factor adaptation tilt smoothing memory */
    Word16 prev_lsf_diff_fx[LPC_SHB_ORDER];
    Word16 prev_tilt_para_fx;
    Word16 cur_sub_Aq_fx[M+1];

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

    Word16 old_core_synth_fx[L_FRAME16k];
    Word16 old_tbe_synth_fx[L_FRAME48k];
    Word16 int_3_over_2_tbemem_dec_fx[INTERP_3_2_MEM_LEN];
    Word16 interpol_3_2_cng_dec_fx[INTERP_3_2_MEM_LEN];
    Word16 mem_resamp_HB_fx[2*L_FILT16k];
    Word16 mem_resamp_HB_32k_fx[2*L_FILT32k];

    /*----------------------------------------------------------------------------------*
     * SWB BWE parameters
     *----------------------------------------------------------------------------------*/
    Word16 old_wtda_wb_fx_exp;
    Word16 L_old_wtda_swb_fx[L_FRAME48k];
    Word16 old_wtda_swb_fx_exp;
    Word16 mem_imdct_exp_fx;
    Word16 old_syn_12k8_16k_fx[NS2SA(16000, DELAY_FD_BWE_ENC_NS)]; /*Q_syn2-1*/

    Word16 mem_deemph_old_syn_fx;
    Word16 prev_mode_fx;
    Word16 prev_SWB_fenv_fx[SWB_FENV];
    Word16 prev_Energy_fx;
    Word32 prev_Energy_wb_fx;
    Word16 prev_L_swb_norm_fx;
    Word16 Seed_fx;
    Word16 memExp1;
    Word16 prev_frica_flag_fx;
    Word16 mem_imdct_fx[L_FRAME48k];
    Word16 prev_td_energy_fx;
    Word16 prev_weight_fx;
    Word16 prev_coder_type_fx;
    Word16 prev_flag_fx;
    Word16 last_wb_bwe_ener_fx;
    Word16 prev_frame_pow_exp;
    Word16 prev_Qx;
    Word16 prev_Q_bwe_exc;
    Word16 prev_Q_synth;
    Word16 prev_fb_ener_adjust_fx;

    /*----------------------------------------------------------------------------------*
     * SWB DTX/CNG parameters
     *----------------------------------------------------------------------------------*/

    Word16 shb_cng_ener_fx;
    Word16 wb_cng_ener_fx;
    Word16 last_wb_cng_ener_fx;
    Word16 last_shb_cng_ener_fx;
    Word16 swb_cng_seed_fx;
    Word16 lsp_shb_prev_prev_fx[LPC_SHB_ORDER];
    Word16 lsp_shb_prev_fx[LPC_SHB_ORDER];
    Word16 shb_dtx_count_fx;
    Word16 last_vad_fx;
    Word16 trans_cnt_fx;
    Word16 burst_cnt_fx;
    Word16 last_shb_ener_fx;

    /*----------------------------------------------------------------------------------*
     * HQ FEC
     *----------------------------------------------------------------------------------*/

    Word16 *prev_good_synth_fx;
    Word16 prev_sign_switch_fx[HQ_FEC_SIGN_SFM];
    Word16 prev_sign_switch_2_fx[HQ_FEC_SIGN_SFM];

    /* HQ PHASE ECU internal state */
    Word16 time_offs_fx;
    Word16 X_sav_fx[PH_ECU_SPEC_SIZE];
    Word16 Q_X_sav;
    Word16 num_p_fx;
    Word16 plocs_fx[MAX_PLOCS];
    Word32 plocsi_fx[MAX_PLOCS];
    Word16 env_stab_fx;
    Word16 mem_norm_hqfec_fx[SFM_N_ENV_STAB];
    Word16 mem_env_delta_hqfec_fx;
    Word16 env_stab_plc_fx;
    Word16 env_stab_state_p_fx[NUM_ENV_STAB_PLC_STATES];
    Word16 envstabplc_hocnt_fx;
    Word16 mag_chg_1st_fx[Lgw_max];          /* i/o: per band magnitude modifier for transients*/
    Word16 Xavg_fx[Lgw_max];                 /* Frequency group average gain to fade to   */
    Word16 beta_mute_fx;                     /* Factor for long-term mute            */
    Word16 last_fec_fx;
    Word16 ph_ecu_HqVoicing_fx;
    Word16 oldHqVoicing_fx;
    Word16 gapsynth_fx[L_FRAME48k];
    Word16 oldgapsynth_fx[L_FRAME48k];
    Word16 ph_ecu_active_fx;      /* Set if Phase ECU was used in last bad frame */
    Word16 ni_seed_forfec;

    /*----------------------------------------------------------------------------------*
     * LD music post-filter
     *----------------------------------------------------------------------------------*/
    Word16 LDm_mem_etot_fx;                               /* LD music post-filter - total energy memory  */
    Word16 LDm_last_music_flag_fx;                        /* LD music post-filter - last music flag */
    Word16 LDm_last_bfi_count_fx;                         /* LD music post-filter - counter for last frame erasure */
    Word16 LDm_nb_thr_1_fx;                               /* LD music post-filter - number of consecutives frames of level 1 */
    Word16 LDm_nb_thr_3_fx;
    Word16 dct_post_old_exc_fx[DCT_L_POST-OFFSET2];
    Word16 LDm_thres_fx[4];                               /* LD music post-filter - Classification threshold */
    Word16 LDm_lt_diff_etot_fx[MAX_LT];                   /* LD music post-filter - long-term total energy variation */
    Word16 LDm_enh_lp_gbin_fx[VOIC_BINS_HR];              /* LD music post-filter - smoothed suppression gain, per bin FFT */
    Word32 LDm_enh_lf_EO_fx[VOIC_BINS_HR];                /* LD music post-filter - old per bin E for previous half frame */
    Word16 LDm_enh_min_ns_gain_fx;                        /* LD music post-filter - minimum suppression gain */
    Word32 LDm_enro_fx[MBANDS_GN_LD];                     /* LD music post-filter - previous energy pre critical band */
    Word32 LDm_bckr_noise_fx[MBANDS_GN_LD];               /* LD music post-filter - background noise estimation per critical band */
    Word16 filt_lfE_fx[DCT_L_POST];
    Word16 last_nonfull_music_fx;
    Word16 Old_ener_Q;                                    /* Old energy scaling factor */

    /*----------------------------------------------------------------------------------*
     * Fixed point only
     *----------------------------------------------------------------------------------*/
    Word16 Q_exc;
    Word16 prev_Q_exc;
    Word16 Q_subfr[L_Q_MEM];

    Word16 prev_Q_bwe_syn;
    Word16 prev_Q_bwe_syn2;

    Word16 Q_syn2;
    Word16 Q_syn;
    Word16 prev_Q_syn;
    Word16 prev_hb_synth_fx_exp;

    Word16 prev_synth_buffer_fx[NS2SA(48000,DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS)];

    Word16 prev_lpc_wb_fx[LPC_SHB_ORDER_WB];
    Word16 GainShape_Delay[NUM_SHB_SUBFR/2];
    Word16 vbr_hw_BWE_disable_dec_fx;
    Word16 last_vbr_hw_BWE_disable_dec_fx;

    HANDLE_CLDFB_FILTER_BANK cldfbAna_fx;             /* main analysis filter bank handle */
    HANDLE_CLDFB_FILTER_BANK cldfbBPF_fx;             /* BPF analysis filter bank handle */
    HANDLE_CLDFB_FILTER_BANK cldfbSyn_fx;             /* main synthesis  filter bank handle */

    /*Frequency-domain-based CNG*/
    HANDLE_FD_CNG_DEC            hFdCngDec_fx;

    /*ACELP config*/
    ACELP_config acelp_cfg;       /*configuration set for each frame*/

    ACELP_config acelp_cfg_rf; /* configuration for RF frame */

    /*TCX config*/
    TCX_config tcx_cfg;
    Word16 L_frameTCX;

    /* evs decoder */
    Word16 m_decodeMode;
    Word16 m_frame_type;                       /*ZERO_FRAME/SID_FRAME/ACTIVE_FRAME*/
    Word16 m_old_frame_type;                   /*ZERO_FRAME/SID_FRAME/ACTIVE_FRAME*/


    /*dec_prm.c*/
    Word16 bits_frame;               /* bit per frame overall included */
    Word16 bits_frame_core;          /* bit per frame for the core */
    Word8 narrowBand;
    Word16 bits_common;              /* read bits from header and LPC*/

    Word8 last_is_cng;

    Word16 old_syn_Overl[L_FRAME32k/2];

    Word16 syn_Overl_TDAC[L_FRAME32k/2];
    Word16 syn_Overl_TDACFB[L_FRAME_MAX/2];

    Word16 syn_Overl[L_FRAME32k/2];
    Word16 syn_OverlFB[L_FRAME_MAX/2];
    Word16 old_synth[OLD_SYNTH_SIZE_DEC];       /* synthesis memory                 */
    Word16 old_synth_len;
    Word16 old_synth_lenFB;
    Word16 syn[M+1];

    /* bass_pf.c */
    Word16 bpf_gain_param;    /*int*/             /* bass post-filter gain factor parameter (0->noBpf)*/

    Word16 L_frame_past;
    Word16 L_frameTCX_past;

    Word16 lsfold_uw[M];                  /* old lsf (unweighted) */
    Word16 lspold_uw[M];                  /* old lsp (unweighted) */
    Word16 seed_tcx_plc;                  /* seed memory (for random function in TCX PLC) */
    Word16 past_gpit;                     /* past gain of pitch (for frame recovery) */
    Word32 past_gcode;                    /* past energy (!) of code  (for frame recovery) */ /*15Q16*/
    Word16 lsf_cng[M];                    /* xSF coefficients used for CNG generation (long term) */
    Word16 lspold_cng[M];                 /* xSP coefficients used for CNG generation (long term) */
    Word8 plcBackgroundNoiseUpdated;       /* flag: Is background noise estimate updated? */
    Word16 lsp_q_cng[M];                  /* xSP coefficients used for CNG generation (short term interpolated) */
    Word16 old_lsp_q_cng[M];              /* xSP coefficients used for CNG generation (short term interpolated) */
    Word16 lsf_q_cng[M];                  /* xSF coefficients used for CNG generation (short term interpolated) */
    Word16 old_lsf_q_cng[M];              /* xSF: old quantized lsfs for background noise */
    Word16 Aq_cng[(NB_SUBFR16k+1)*(M+1)]; /* LPC coefficients derived from CNG estimate  */
    Word16 mem_syn_unv_back[M];           /* filter memory for unvoiced synth */
    Word16 last_gain_syn_deemph;              /*Q15*/
    Word16 last_gain_syn_deemph_e;
    Word16 last_concealed_gain_syn_deemph;    /*Q15*/
    Word16 last_concealed_gain_syn_deemph_e;

    Word8 enableTcxLpc; /* global toggle for the TCX LPC quantizer */
    Word8 envWeighted;  /* are is{p,f}_old[] weighted or not? */

    /* variables for framing */
    Word16 nb_subfr;

    Word16 fscale;
    Word16 fscale_old;
    Word32 sr_core; /*Q0*/

    Word16 pit_min; /*int*/
    Word16 pit_fr1;
    Word16 pit_fr1b;
    Word16 pit_fr2;
    Word16 pit_max; /*int Q0*/
    Word16 pit_res_max;
    Word16 pit_res_max_past;

    Word16 pit_max_TCX; /*int Q0*/
    Word16 pit_min_TCX; /*int*/

    /*Preemphasis factor*/
    Word16 preemph_fac; /*0Q15*/
    Word16 gamma;
    Word16 inv_gamma;

    /*for AMR-WB like 6.4 to 7 kHz upsampling and noise filling*/
    Word16 mem_Aq[(NB_SUBFR16k)*(M+1)];   /* Q12 */

    /* Error concealment */
    Word16 last_core_bfi;
    Word16 nbLostCmpt;                       /* compt for number of consecutive lost frame*/
    Word16 prev_nbLostCmpt;                  /* compt for number of consecutive lost frame at the previous frame*/
    Word16 mode_lvq;                         /* index for LSF mean vector */
    Word16 lsfoldbfi0[M];                    /* Previous frame lsf                      */
    Word16 lsfoldbfi1[M];                    /* Past previous frame lsf                 */
    Word16 lsf_adaptive_mean[M];             /*  Mean lsf for bfi cases                 */
    Word32 old_fpitch;                       /* last pitch of previous frame  */ /*15Q16*/
    Word32 old_fpitchFB;                     /* PLC - last pitch of previous FB frame (depends on output sr) */  /*15Q16*/
    Word16 clas_dec;
    Word16 mem_pitch_gain[2*NB_SUBFR16k+2];  /* Pitch gain memory     Q14                   */
    Word8  plc_use_future_lag;
    Word16 prev_widow_left_rect;
    Word32 Mode2_lp_gainc;                          /* 15Q16  low passed code gain used for concealment*/
    Word32 Mode2_lp_gainp;                          /* 15Q16 low passed pitch gain used for concealment*/
    Word16 conCngLevelBackgroundTrace;        /* Q15  long term gain estimate for background level, used for PLC fade out */
    Word16 conCngLevelBackgroundTrace_e;
    /* state variables for the minimum statistics used for PLC */
    Word16 conNoiseLevelMemory[PLC_MIN_STAT_BUFF_SIZE];/*Q15*/
    Word16 conNoiseLevelMemory_e[PLC_MIN_STAT_BUFF_SIZE];
    Word16 conNoiseLevelIndex; /*Q0*/
    Word16 conCurrLevelIndex;  /*Q0*/
    Word16 conLastFrameLevel;/*Q15*/
    Word16 conLastFrameLevel_e;

    Word16 old_gaintcx_bfi, old_gaintcx_bfi_e;
    Word16 cummulative_damping_tcx;
    Word16 cummulative_damping; /*Q15*/
    Word16 cngTDLevel;
    Word16 cngTDLevel_e;

    Word16 conceal_eof_gain; /*Q15*/
    Word16 damping; /* 1Q14 */
    Word16 gainHelper; /*can be >1*/
    Word16 gainHelper_e;
    Word16 stepCompensate;
    Word16 stepCompensate_e;

    Word16 classifier_last_good;          /* last correctly received frame classification , old enum SIGNAL_CLASSIFICATION*/
    Word16 classifier_Q_mem_syn;          /*scalingfactor of mem_syn_clas_estim_fx in MODE2 */

    /* LPC quantization */
    Word16 lpcQuantization;
    Word16 numlpc;

    /* Bandwidth */
    Word16 TcxBandwidth;

    /* For NB and formant post-filter */
    PFSTAT pfstat;
    Word16 psf_lp_noise_fx;

    /* For adaptive tilt_code */
    Word16 voice_fac;

    Word8 tcxonly;

    /*TCX resisual Q*/
    Word16 resQBits[NB_DIV];       /* number of bits read for the residual Quantization in TCX*/

    Word16 last_ctx_hm_enabled;

    /* TCX-LTP */
    Word8 tcxltp;
    Word16 tcxltp_gain;
    Word16 tcxltp_pitch_int;
    Word16 tcxltp_pitch_fr;

    Word16 tcxltp_mem_in[TCXLTP_MAX_DELAY];
    Word16 tcxltp_mem_out[L_FRAME48k];
    Word16 tcxltp_pitch_int_post_prev;
    Word16 tcxltp_pitch_fr_post_prev;
    Word16 tcxltp_gain_post_prev;
    Word16 tcxltp_filt_idx_prev;

    struct tonalmdctconceal tonalMDCTconceal;
    Word8 tonal_mdct_plc_active;
    Word8 last_tns_active;
    Word8 second_last_tns_active;
    Word16 second_last_core;
    Word32 tcxltp_second_last_pitch;
    Word32 tcxltp_third_last_pitch;
    Word16 tcxltp_last_gain_unmodified;

    Word16 FBTCXdelayBuf[111]; /* 2.3125ms at 48kHz -> 111 samples */

    /* parameters for switching */
    Word16 mem_syn_r[L_SYN_MEM];         /*LPC synthesis memory needed for rate switching*/
    Word16 rate_switching_reset;

    Word32 lp_error_ener;
    Word32 mem_error;
    Word16 bpf_noise_buf[L_FRAME_16k];
    Word16 *p_bpf_noise_buf;

    Word8   enableGplc;
    Word16  flagGuidedAcelp; /*int*/
    Word16  T0_4th;/*int*/
    Word16  guidedT0; /*int*/

    Word16 enablePlcWaveadjust;
    Word16 tonality_flag;
    T_PLCInfo plcInfo;

    Word8 VAD;
    Word8 flag_cna;

    Word32 lp_noise;

    Word16 seed_acelp;

    Word16 core_ext_mode; /*GC,VC,UC,TC: core extended mode used for PLC or Acelp-external modules.*/

    Word8 dec_glr;
    Word16 dec_glr_idx;

    Word16 tcx_hm_LtpPitchLag;
    Word16 tcx_lpc_shaped_ari;

    Word16 igf;
    IGFDEC_INSTANCE  hIGFDec;

    CLDFB_SCALE_FACTOR scaleFactor;

    Word16 tec_tfa;
    Word16 tec_flag;
    Word16 tfa_flag;
    TEMPORAL_ENVELOPE_CODING_DECODER_FX tecDec_fx;

    Word16 con_tcx;
    Word16 last_con_tcx;
    Word16 old_ppp_mode_fx;
    Word16 old_hb_synth_fx[L_FRAME48k];

    Word16 prev_Q_exc_fr;
    Word16 prev_Q_syn_fr;

    Word16 writeFECoffset;


} Decoder_State_fx;
#endif
