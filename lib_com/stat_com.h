/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef STAT_COM_H
#define STAT_COM_H

#include "options.h"
#include "basop_util.h"
#include "cnst_fx.h"

/*----------------------------------------------------------------------------------*
 * Declaration of structures
 *----------------------------------------------------------------------------------*/

/* Forward declaration of Decoder_State_fx */
struct Decoder_State_fx;

/*-----------------------------------------------------------*
 * EV-VBR NB postfilter static variables
 *-----------------------------------------------------------*/
typedef struct
{
    Word16 on;                            /* On/off flag           */
    Word16 reset;                         /* reset flag           */
    Word16 mem_pf_in[L_SUBFR];                  /* Input memory          */
    Word16 mem_stp[L_SUBFR];              /*  1/A(gamma1) memory   */
    Word16 mem_res2[DECMEM_RES2];         /* A(gamma2) residual    */
    Word16 mem_zero[M];                   /* null memory to compute i.r. of A(gamma2)/A(gamma1) */
    Word16 gain_prec;                     /* for gain adjustment   */
} PFSTAT;

typedef struct
{
    Word16 a_fx[MAXLAG_WI];
    Word16 b_fx[MAXLAG_WI];
    Word16 lag_fx;
    Word16 nH_fx;
    Word16 nH_4kHz_fx;
    Word16 upper_cut_off_freq_of_interest_fx;
    Word16 upper_cut_off_freq_fx;
    Word16 Fs_fx;
    Word16 Q;
} DTFS_STRUCTURE_FX;

typedef struct
{
    Word16 lead_sign_ind;
    UWord32 index, size;
    Word16 dim, k_val;
} PvqEntry_fx;

typedef struct
{
    UWord8 buf[MAX_SIZEBUF_PBITSTREAM];
    Word16 curPos;
    Word32 numByte;
    Word32 numbits;
} BITSTREAM_FX, *PBITSTREAM_FX;

typedef struct
{
    PBITSTREAM_FX  bsInst;

    Word32  low;
    Word32  high;

    Word32  value;
    Word16    bits_to_follow;

    Word32    num_bits;

} ARCODEC_FX, *PARCODEC_FX;

/*---------------------------------------------------------------*
 * Encoder/Decoder Static RAM                                    *
 *---------------------------------------------------------------*/
typedef struct
{
    /*Coding mode info*/
    Word16 mode_index;

    /*LPC info*/
    Word8 midLpc;             /*Flag for using or not mid LPC for the current frame*/
    Word8 midLpc_enable;      /*Flag enabling or not the mid LPC for the current mode_index*/

    /*ICB flags*/
    Word8 pre_emphasis;
    Word8 pitch_sharpening; /*Flag for triggering pitch sharpening*/
    Word8 phase_scrambling; /*Flag for triggering phase scrambling*/
    Word16 formant_enh; /*Flag for triggering formant enhancement: Q15 representing 0...1.0f */
    Word8 formant_tilt;

    Word8 voice_tilt; /*Flag for triggering new voice factor tilt*/

    Word16 formant_enh_num;
    Word16 formant_enh_den;

    Word16 bpf_mode;

    Word16 nrg_mode;
    Word16 nrg_bits;

    Word16 ltp_mode;
    Word16 ltp_bits;

    Word16 ltf_mode;
    Word16 ltf_bits;

    Word16 gains_mode[NB_SUBFR16k];

    Word16 fixed_cdk_index[NB_SUBFR16k];

} ACELP_config;

/*tns_base.h*/
/** TNS configuration.
  * Use InitTnsConfiguration to initialize it.
  */
typedef struct
{
    Word16 maxOrder;

    /** Maximum number of filters. */
    Word16 nMaxFilters;

    /** Parameters for each TNS filter */
    struct TnsParameters const * pTnsParameters;

    /** Lower borders for each filter.
      * Upper border for the first filter is nsbBorders-1.
      * Upper borders for other filters is the lower border of previous filter.
      */
    Word16 iFilterBorders[TNS_MAX_NUM_OF_FILTERS+1];

} STnsConfig;

/** TNS filter.
  * Parameters that define a TNS filter.
  */
typedef struct
{
    /** Number of subbands covered by the filter. */
    Word16 spectrumLength;
    /** Direction of the filter. */
    Word16 direction;
    /** Filter order. */
    Word16 order;
    /** Quantized filter coefficients. */
    Word16 coefIndex[TNS_MAX_FILTER_ORDER];
    /** Prediction gain. The ratio of a signal and TNS residual energy. */
    Word16 predictionGain; /* exponent = PRED_GAIN_E */
    /** Average squared filter coefficient. */
    Word16 avgSqrCoef; /* exponent = 0 */
} STnsFilter;

/** TNS data.
  * TNS data describing all active filters.
  */
typedef struct
{
    /** Number of active filters. */
    Word16 nFilters;
    /** Active filters. */
    STnsFilter filter[TNS_MAX_NUM_OF_FILTERS];
} STnsData;

typedef enum
{
    TNS_NO_ERROR = 0,
    TNS_FATAL_ERROR
} TNS_ERROR;

typedef struct
{
    /* TCX mdct window */
    const PWord16 *tcx_mdct_window;
    const PWord16 *tcx_mdct_window_half;
    const PWord16 *tcx_mdct_window_minimum;
    const PWord16 *tcx_mdct_window_trans;
    Word16 tcx_aldo_window_1[L_FRAME32k*9/32];
    PWord16 tcx_aldo_window_1_trunc[L_FRAME32k*7/32];
    PWord16 tcx_aldo_window_2[L_FRAME32k*7/32];
    Word16 last_aldo;

    Word16 tcx5Size;  /* Size of the TCX5 spectrum. Always 5ms. */

    Word16 tcx_mdct_window_length;
    Word16 tcx_mdct_window_half_length; /*length of the "half" overlap window*/
    Word16 tcx_mdct_window_min_length; /* length of the "minimum" overlap */
    Word16 tcx_mdct_window_trans_length; /* length of the ACELP->TCX overlap */

    Word16 tcx_mdct_window_delay; /*length of window delay*/

    Word16 tcx_offset;
    Word16 tcx_mdct_window_length_old; /*for keeping old value for sample rate switching */

    /* TCX mdct window */
    const PWord16 *tcx_mdct_windowFB;
    const PWord16 *tcx_mdct_window_halfFB;
    const PWord16 *tcx_mdct_window_minimumFB;
    const PWord16 *tcx_mdct_window_transFB;
    Word16 tcx_aldo_window_1_FB[L_FRAME_MAX*9/32];
    PWord16 tcx_aldo_window_1_FB_trunc[L_FRAME_MAX*7/32];
    PWord16 tcx_aldo_window_2_FB[L_FRAME_MAX*7/32];

    Word16 tcx5SizeFB;  /* Size of the TCX5 spectrum. Always 5ms. */

    Word16 tcx_mdct_window_lengthFB;
    Word16 tcx_mdct_window_half_lengthFB; /*length of the "half" overlap window*/
    Word16 tcx_mdct_window_min_lengthFB; /* length of the "minimum" overlap */
    Word16 tcx_mdct_window_trans_lengthFB; /* length of the ACELP->TCX overlap */

    Word16 tcx_mdct_window_delayFB; /*length of window delay*/
    Word16 tcx_offsetFB;

    Word16 tcx_coded_lines;  /* max number of coded lines, depending on bandwidth mode */

    Word16 tcx_curr_overlap_mode; /* window overlap of current frame (0: full, 2: none, or 3: half) */
    Word16 tcx_last_overlap_mode; /* previous window overlap, i.e. left-side overlap of current frame */

    /* FAC window */
    Word16 lfacNext;
    Word16 lfacNextFB;

    /* TNS */
    Word8 fIsTNSAllowed;
    STnsConfig tnsConfig[2][2];
    STnsConfig const * pCurrentTnsConfig;

    /*Quantization*/
    Word16 sq_rounding; /*set the sq deadzone (no deadzone=0.5f)*/
    Word8 tcxRateLoopOpt;

    /*Bandwidth*/
    Word16 preemph_fac;     /*preemphasis factor*/
    Word16 bandwidth;

    /* Context HM - Residual Quantization*/
    Word8 ctx_hm;  /*Flag for enabling Context HM*/
    Word8 resq;      /*Flag for enabling Residual Quantization*/
    Word16 coder_type; /*INACTIVE, UNVOICED,VOICED,GENERIC*/

    Word16 na_scale;

    Word32 SFM2;
} TCX_config;

/* prot.h */
typedef struct
{
    Word16 bits;      /* bits per subframe */
    Word16 nbiter;        /* number of iterations */
    Word16 alp;           /* initial energy of all fixed pulses, exponent = ALP_E */
    Word8 nb_pulse;      /* number of pulses */
    Word8 fixedpulses;   /* number of pulses whose position is determined from correlation and not by iteration */
    Word8 nbpos[20];     /* number of positions tried in the pair-wise search */
    Word8 codetrackpos;  /* ordering of tracks -mode */
} PulseConfig;

/* fd_cng_common.h */
/* CLDFB-based CNG setup */
typedef struct
{
    Word16 fftlen;                  /* FFT length */
    Word16 stopFFTbin;              /* Number of FFT bins to be actually processed */
    Word16 numPartitions;           /* Number of partitions */
    const Word16* sidPartitions;          /* Upper boundaries for grouping the (sub)bands into partitions when transmitting SID frames (define as NULL pointer to avoid grouping) */

    Word16 numShapingPartitions;    /* Number of partitions */
    const Word16* shapingPartitions;      /* Upper boundaries for grouping the (sub)bands into partitions for shaping at the decoder (define as NULL pointer to avoid grouping) */
} FD_CNG_SETUP;

/* Scale setup */
typedef struct
{
    Word32 bitrateFrom;
    Word32 bitrateTo;

    Word16 scale;

    Word8 bwmode;

} SCALE_SETUP;

/* Arrays and variables common to encoder and decoder */
typedef struct
{
    FD_CNG_SETUP FdCngSetup;

    Word16  numSlots;       /* Number of time slots in CLDFB matrix */
    Word16  regularStopBand;/* Number of CLDFB bands to be considered */

    Word16  numCoreBands;   /* Number of core bands to be decomposed into FFT subbands */
    Word16  stopBand;       /* Total number of (sub)bands to be considered */
    Word16  startBand;      /* First (sub)band to be considered */
    Word16  stopFFTbin;     /* Total number of FFT subbands */
    Word16  frameSize;      /* Frame size in samples */
    Word16  fftlen;         /* FFT length used for the decomposition */
    Word16  fftlenShift;
    Word16  fftlenFac;

    Word16  timeDomainBuffer[L_FRAME16k];

    Word32  fftBuffer[FFTLEN];
    Word16 *olapBufferAna;            /* points to FD_CNG_DEC->olapBufferAna[320] in case of decoder */
    Word16  olapBufferSynth[FFTLEN];
    Word16 *olapBufferSynth2;         /* points to FD_CNG_DEC->olapBufferSynth2[FFTLEN] in case of decoder */
    const PWord16  * olapWinAna;
    const PWord16  * olapWinSyn;

    Word16  msM_win;
    Word16  msM_subwin;
    Word16  msFrCnt_init_counter;          /* Frame counter at initialization */
    Word16  init_old;

    Word16  msFrCnt_init_thresh;

    Word16  msFrCnt;        /* Frame counter */
    Word32  msAlphaCor[2];  /* Correction factor (smoothed) */
    Word16  msSlope[2];
    Word32  msQeqInvAv[2];
    Word16  msQeqInvAv_exp[2];
    Word16  msMinBufferPtr;

    Word32  msPsdSum[2];
    Word32  msPeriodogSum[2];
    Word16  msPeriodogSum_exp[2];

    Word16  offsetflag;

    Word32  periodog[FFTCLDFBLEN];     /* Periodogram */
    Word16  periodog_exp;
    Word16  exp_cldfb_periodog;

    Word32  cngNoiseLevel[FFTCLDFBLEN];  /* Noise level applied for the CNG in each (sub)band */
    Word16  cngNoiseLevelExp;
    Word16  seed;           /* Seed memory (for random function) */

    Word16  npart;          /* Number of partitions */
    Word16  midband[NPART];        /* Central band of each partition */
    Word16  nFFTpart;       /* Number of hybrid spectral partitions */
    Word16  part[NPART];           /* Partition upper boundaries (band indices starting from 0) */
    Word16  psize[NPART];          /* Partition sizes */
    Word16  psize_norm[NPART];     /* Partition sizes, fractional variable */
    Word16  psize_norm_exp;        /* Partition sizes exponent for fractional variable */
    Word16  psize_inv[NPART];      /* Inverse of partition sizes */
    Word16  FFTscalingFactor;      /* Squared ratio between core signal analysis FFT and noise estimator FFT */
    Word16  scalingFactor;
    Word16  invScalingFactor;
    Word16  nCLDFBpart;              /* Number of CLDFB spectral partitions */
    Word16  CLDFBpart[NPARTCLDFB];     /* CLDFB Partition upper boundaries (band indices starting from 0 above the core coder bands) */
    Word16  CLDFBpsize_inv[NPARTCLDFB];/* Inverse of CLDFB partition sizes */

    Word16  inactive_frame_counter;
    Word16  sid_frame_counter;
    Word16  active_frame_counter;

    Word32  sidNoiseEst[NPART];    /* Transmitted noise level */
    Word16  sidNoiseEstExp;

    Word16   frame_type_previous;

    Word16   A_cng[M+1];
    Word16   exc_cng[L_FRAME16k];

    Word32   CngBitrate;
    Word16   CngBandwidth;

    Word16   flag_noisy_speech;
    Word16   likelihood_noisy_speech;
}
FD_CNG_COM;
typedef FD_CNG_COM *HANDLE_FD_CNG_COM;

/*parameter_bitmaping.h*/
/** Function that gets specific value from p.
  * @param p Pointer to a variable that can also be structure or array.
  * @param index Index of a variable when p is an array, otherwise 0.
  * @param pValue Pointer to the value.
  * @return Substructure associated with this value or NULL if there is none.
  */
typedef void const * (* TGetParamValue)(void const * p, Word16 index, Word16 * pValue);

/** Function that puts specific value to p.
  * @param p Pointer to a variable that can also be structure or array.
  * @param index Index of a variable when p is an array, otherwise 0.
  * @param value The value.
  * @return Substructure associated with this value or NULL if there is none.
  */
typedef void * (* TSetParamValue)(void * p, Word16 index, Word16 value);

/** Function that return required number of bits for a value when it is coded.
  * @param value The value.
  * @param index Index of a variable when it is an element of an array, otherwise 0.
  * @return Number of bits required to code the value.
  */
typedef Word16 (* TGetNumberOfBits)(Word16 value, Word16 index);

/** Function that encodes a value.
  * @param value The value.
  * @param index Index of a variable when it is an element of an array, otherwise 0.
  * @return Coded value.
  */
typedef Word16 (* TEncodeValue)(Word16 value, Word16 index);

/** Function that decodes a value.
  * @param st Decoder state.
  * @param index Index of a variable when it is an element of an array, otherwise 0.
  * @param pValue A pointer where the decoded value should be stored.
  * @return Number of bits read from the bitstream.
  */
typedef Word16 (* TDecodeValue)(struct Decoder_State_fx *st, Word16 index, Word16 * pValue);

/** Structure that defines mapping between a parameter and a bistream. */
typedef struct ParamBitMap
{
    /** Number of bits in a bitstream required for the parameter.
      * If nBits is equal to 0 then GetNumberOfBits is used.
      */
    Word16 nBits;
    /** Function to get the number of bits required for a value of this parameter.
      * If nBits != 0 it is not used and can be set to NULL.
      * If fZeroAllowed == 0 then GetNumberOfBits must take care of this.
      */
    TGetNumberOfBits GetNumberOfBits;
    /** If fZeroAllowed is 0 then the value can be zero.
      * If the value can't be zero then value-1 is stored in a bitstream.
      * If EncodeValue is not equal to NULL, then the encode/decode function
      * must take care of this flag - there is no additional processing in parameter bitmapping.
      * If EncodeValue is equal to NULL, then the encode/decode function takes care of this.
      */
    Word8 fZeroAllowed;
    /** Function to get the value of this parameter.
      * The function returns a pointer to be used in functions in pSubParamBitMap.
      * If the function returns NULL then the same pointer as for the current
      * parameter is to be used.
      * The function should not do any additional processing if fZeroAllowed == 0,
      * but just set the value as it is.
      */
    TGetParamValue GetParamValue;
    /** Function to set the value of this parameter.
      * The function returns a pointer to be used in functions in pSubParamBitMap.
      * If the function returns NULL then the same pointer as for the current
      * parameter is to be used.
      * The function should not do any additional processing if fZeroAllowed == 0,
      * but just set the value as it is.
      */
    TSetParamValue SetParamValue;

    /** Function to encode a value of this parameter.
      * When it is equal to NULL, fixed-width coding is used.
      * If fZeroAllowed == 0 then EncodeValue must take care of this.
      */
    TEncodeValue EncodeValue;

    /** Function to decode a value of this parameter.
      * When it is equal to NULL, fixed-width coding is used.
      * If fZeroAllowed == 0 then DecodeValue must take care of this.
      */
    TDecodeValue DecodeValue;

    /** Pointer to the map for substructure.
      * The number of structures is determined by this parameter's value.
      * NULL means that there is no substructure.
      */
    struct ParamsBitMap const * pSubParamBitMap;
} ParamBitMap;

/** Structure that defines mapping between parameters and a bistream. */
typedef struct ParamsBitMap
{
    /** Number of parameters in params. */
    Word16 nParams;
    /** Definition of the mapping for each parameter. */
    ParamBitMap params[NPARAMS_MAX];
} ParamsBitMap;

/*tns_tables.h*/
struct TnsParameters
{
    /* Parameters for each TNS filter */
    Word16 startLineFrequency;   /* Starting lower frequency of the TNS filter [20..16000] */
    Word16 nSubdivisions;        /* Number of spectrum subdivisions in which the filter operates [1..8) */
    Word16 minPredictionGain;  /* Minimum prediction gain required to turn on the TNS filter. Exponent = PRED_GAIN_E */
    Word16 minAvgSqrCoef;      /* Minimum average square of coefficients required to turn on the TNS filter. Exponent = 0 */
};

/**********************************************/
/* Helper structures for hufmann table coding */
/**********************************************/

typedef struct
{
    Word8 value;
    Word16 code;
    Word8 nBits;
} Coding;


/* Scale TCX setup */
typedef struct
{
    Word16 bwmode;

    Word32 bitrateFrom;
    Word32 bitrateTo;

    Word16 scale;

} SCALE_TCX_SETUP;


/* glob_con.h */
typedef struct
{
    Word16 frame_bits;           /*Bits per frame*/
    Word16 frame_net_bits;       /*Net Bits per frame*/
    Word8  transmission_bits;    /*max=1*/
    Word8  transmission_mode[2]; /*SID,VBR/CBR*/
    Word8  bandwidth_bits;       /*max=2*/
    Word8  bandwidth_min;        /*first valid bandwidth (NB,WB,SWB,FB)*/
    Word8  bandwidth_max;        /*last valid bandwidth (NB,WB,SWB,FB)*/
    Word8  reserved_bits;        /*max=1*/
} FrameSizeParams;

typedef struct
{
    Word16 lb_scale;            /*!< Scale of low band area                   */
    Word16 lb_scale16;          /*!< Scale of low band area                   */
    Word16 ov_lb_scale;         /*!< Scale of adjusted overlap low band area  */
    Word16 hb_scale;            /*!< Scale of high band area                  */
    Word16 ov_hb_scale;         /*!< Scale of adjusted overlap high band area */
} CLDFB_SCALE_FACTOR;

struct CLDFB_FILTER_BANK
{
    const Word16 *p_filter;     /*!< Pointer to filter coefficients */

    Word16 *FilterStates;       /*!< Pointer to buffer of filter states */
    Word16 FilterStates_e[CLDFB_NO_COL_MAX+9];       /*!< Filter states time slot exponents */
    Word16 FilterStates_eg;     /*!< Filter states current exponent */

    Word16 p_filter_length;     /*!< Size of prototype filter. */
    const Word16 *rRotVctr;     /*!< Modulation tables. */
    const Word16 *iRotVctr;
    Word16 filterScale;         /*!< filter scale */

    Word16 synGain;             /*!< gain for synthesis filterbank */

    Word16 no_channels;         /*!< Total number of channels (subbands) */
    Word16 no_col;              /*!< Number of time slots       */
    Word16 lsb;                 /*!< Top of low subbands */
    Word16 usb;                 /*!< Top of high subbands */

    Word16 zeros;               /*!< number of zeros in filter coefficients */

    Word16 anaScalefactor;      /*!< Scale factor of analysis cldfb */
    Word16 synScalefactor;      /*!< Scale factor of synthesis cldfb */
    Word16 outScalefactor;      /*!< Scale factor of output data (syn only) */

    Word16 synFilterHeadroom;   /*!< Headroom for states in synthesis cldfb filterbank */

    UWord16 flags;              /*!< flags */

    Word16 bandsToZero;         /*!< additional bands which are zeroed in inverse modulation */

    Word16 filtermode;

    Word16 type;
    Word16 *memory;
    Word16 memory_length;

    Word16 scale;
};

typedef struct CLDFB_FILTER_BANK *HANDLE_CLDFB_FILTER_BANK;

typedef struct
{
    Word16 pGainTemp_m[CLDFB_NO_COL_MAX];
    Word16 pGainTemp_e[CLDFB_NO_COL_MAX];
    Word16 loTempEnv[CLDFB_NO_COL_MAX];
    Word16 loBuffer[CLDFB_NO_COL_MAX + MAX_TEC_SMOOTHING_DEG];

    Word16 cldfbExp;
    Word16 lastCldfbExp;
} TEMPORAL_ENVELOPE_CODING_DECODER_FX;
typedef TEMPORAL_ENVELOPE_CODING_DECODER_FX* HANDLE_TEC_DEC_FX;

typedef struct
{
    Word16 loBuffer[CLDFB_NO_COL_MAX + MAX_TEC_SMOOTHING_DEG + DELAY_TEMP_ENV_BUFF_TEC];
    Word16 loTempEnv[CLDFB_NO_COL_MAX];
    Word16 loTempEnv_ns[CLDFB_NO_COL_MAX];
    Word16 hiTempEnv[CLDFB_NO_COL_MAX + DELAY_TEMP_ENV_BUFF_TEC + EXT_DELAY_HI_TEMP_ENV];
    Word16 tranFlag;
    Word16 corrFlag;
} TEMPORAL_ENVELOPE_CODING_ENCODER_FX;
typedef TEMPORAL_ENVELOPE_CODING_ENCODER_FX* HANDLE_TEC_ENC_FX;

typedef enum
{
    FRAME_0 = 0,
    FRAME_2 =  40,
    FRAME_2_4 =  48,
    FRAME_4 = 80,
    FRAME_5_6 = 112,
    FRAME_7_2 = 144,
    FRAME_8 = 160,
    FRAME_9_6 = 192,
    FRAME_13_2 = 264,
    FRAME_16_4 = 328,
    FRAME_24_4 = 488,
    FRAME_32 = 640,
    FRAME_48 = 960,
    FRAME_64 = 1280,
    FRAME_96 = 1920,
    FRAME_128 = 2560
} FRAME_SIZE;

/*---------------------------------------------------------------*
 * IGF                                                           *
 *---------------------------------------------------------------*/
typedef struct igf_grid_struct
{
    Word16                swb_offset[IGF_MAX_SFB];
    Word16                swb_offset_len;
    Word16                startFrequency;
    Word16                stopFrequency;
    Word16                startLine;
    Word16                stopLine;
    Word16                startSfb;                               /* 15Q0, startSfb = [0, 11], IGF start sfb */
    Word16                stopSfb;                                /* 15Q0, stopSfb  = [0, 22], IGF stop  sfb */
    Word16                sfbWrap[IGF_MAX_TILES+1];
    Word16                sbWrap[IGF_MAX_TILES];
    Word16                nTiles;                                 /* 15Q0, nTiles = [1, 4], number of tiles */
    Word16                minSrcSubband;
    Word16                minSrcFrequency;
    Word16                tile[IGF_MAX_TILES];
    Word16                infoIsRefined;
    Word16                infoGranuleLen;                         /* 15Q0, infoGranuleLen = [0, 1200], core coder granule length */
    Word16                infoTransFac;                           /* 1Q14 */
    Word16                whiteningThreshold[2][IGF_MAX_TILES];   /* 2Q13 */
    Word16                gFactor;                                /* 1Q14 */
    Word16                fFactor;                                /* 1Q14 */
    Word16                lFactor;                                /* 1Q14 */
} IGF_GRID, *H_IGF_GRID;

typedef struct IGF_INFO_struct
{
    Word16                nfSeed;
    Word32                sampleRate;
    Word16                frameLength;
    Word16                maxHopsize;
    IGF_GRID              grid[IGF_NOF_GRIDS];
    Word16                bitRateIndex;
} IGF_INFO, *H_IGF_INFO;


typedef struct
{
    Word16                *indexBuffer;
    Word16                *peakIndices, *holeIndices;
    Word16                numPeakIndices, numHoleIndices;
} CONTEXT_HM_CONFIG;

/* Returns: index of next coefficient */
typedef Word16 (*get_next_coeff_function)(
    Word16 ii[2],             /* i/o: coefficient indexes       */
    Word16 *pp,               /* o  : peak(1)/hole(0) indicator */
    Word16 *idx,              /* o  : index in unmapped domain  */
    CONTEXT_HM_CONFIG *hm_cfg /* i  : HM configuration          */
);

/*CLDFB-VAD*/


#endif
