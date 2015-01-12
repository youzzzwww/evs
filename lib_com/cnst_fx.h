/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef CNST_FX_H
#define CNST_FX_H

#include "options.h"     /* Compilation switches                   */


#define MODE1                                1
#define MODE2                                2

/*----------------------------------------------------------------------------------*
 *  General constants
 *----------------------------------------------------------------------------------*/

#define MIN_ETOT_FX                            5120
#define LT_UV_THR_FX                         (100*64) /* in Q6 */
#define INV_MAX_LT_FX                         (Word16)((1.0f/MAX_LT)*32768)
#define LT_UV_THRSP_FX                       70

#define EVS_PI     3.14159265358979323846264338327950288f

#define PI_Q29        1686629713  /* PI in 2Q29 format */
#define PI_Q16        205887    /* PI in 15Q16 format*/

#define LG10              24660    /*  10*log10(2)  in Q13 */

#define CL_HYST_BR_FAC    3      /*(3.0f / 65536.0f)*/


#define EVS_LW_SIGN (Word32)0x80000000       /* sign bit */
#define EVS_LW_MIN (Word32)0x80000000
#define EVS_LW_MAX (Word32)0x7fffffff

#define EVS_SW_SIGN (Word16)0x8000          /* sign bit for Word16 type */
#define EVS_SW_MIN (Word16)0x8000           /* smallest Ram */
#define EVS_SW_MAX (Word16)0x7fff           /* largest Ram */


/***************** FROM cnst.h ******************/

#define RANDOM_INITSEED                       21845     /* Seed for random generators */
#define MAX_FRAME_COUNTER                     200
#define BITS_PER_SHORT                        16
#define BITS_PER_BYTE                         8
#define MAX_BITS_PER_FRAME                    2560
#define FEC_SEED                              12558
#define SYNC_GOOD_FRAME                       (UWord16) 0x6B21         /* synchronization word of a "good" frame */
#define SYNC_BAD_FRAME                        (UWord16) 0x6B20         /* synchronization word of a "bad" frame */
#define G192_BIN0                             (UWord16) 0x007F         /* binary "0" according to ITU-T G.192 */
#define G192_BIN1                             (UWord16) 0x0081         /* binary "1" according to ITU-T G.192 */

#define ENC                                   0         /* Index for "encoder" */
#define DEC                                   1         /* Index for "decoder" */

#define NB                                    0         /* Indicator of 4 kHz bandwidth */
#define WB                                    1         /* Indicator of 8 kHz bandwidth */
#define SWB                                   2         /* Indicator of 14 kHz bandwidth */
#define FB                                    3         /* Indicator of 20 kHz bandwidth */

/* Conversion of bandwidth string into numerical constant */
#define CONV_BWIDTH(bw)                       ( !strcmp(bw, "NB") ? NB : !strcmp(bw, "WB") ? WB : !strcmp(bw, "SWB") ? SWB : !strcmp(bw, "FB") ? FB : -1)

#define L_FRAME48k                            960       /* Frame size in samples at 48kHz */
#define L_FRAME32k                            640       /* Frame size in samples at 32kHz */
#define L_FRAME16k                            320       /* Frame size in samples at 16kHz */
#define L_FRAME8k                             160       /* Frame size in samples at 8kHz */

/* Conversion of ns to samples for a given sampling frequency */
#define NS2SA(fs,x)                           ((short)((((long)(fs)/100L) * ((x)/100L)) / 100000L))

extern const Word16 Idx2Freq_Tbl[];
#define Q6 64

#define chk_fs(fs)
/* 'x' is in Q6, 'Freq_Tbl'/1000 in Q9 */
/* only works for 'fs' = [8000,12800,16000,25600,32000,48000] (unpredictable otherwise) */
#define NS2SA_fx2(fs,x)                       (chk_fs(fs) mult((&Idx2Freq_Tbl[-2])[L_and(L_shr(fs,8),7)], (Word16)((x)/1000000.0f*Q6)))

/*----------------------------------------------------------------------------------*
 * Layers
 *----------------------------------------------------------------------------------*/

#define ACELP_CORE                            0         /* ACELP core layer                                         */
#define TCX_20_CORE                           1         /* TCX 20ms core layer                                      */
#define TCX_10_CORE                           2         /* TCX 10ms core layer                                      */
#define HQ_CORE                               3         /* HQ core layer                                            */
#define AMR_WB_CORE                           4         /* AMR-WB IO core                                           */

#define WB_TBE                                5         /* WB TBE layer (16/32/48kHz signals)                       */
#define WB_BWE                                6         /* WB BWE layer optimized for music (16/32/48kHz signals)   */

#define SWB_CNG                               7         /* SWB CNG layer (32/48kHz signals)                         */
#define SWB_TBE                               8         /* SWB TBE layer optimized for speech (32/48kHz signals)    */
#define SWB_BWE                               9         /* SWB BWE layer optimized for music (32/48kHz signals)     */
#define SWB_BWE_HIGHRATE                      10        /* SWB BWE layer optimized for highrate speech (32/48kHz)   */

#define FB_TBE                                11        /* FB TBE layer (48kHz signals)                             */
#define FB_BWE                                12        /* FB BWE layer optimized for music (48kHz)                 */
#define FB_BWE_HIGHRATE                       13        /* FB BWE layer optimized for highrate speech (48kHz)       */

#define IGF_BWE                               14        /* IGF layer for music (16.4 and 24.4kbps), 32kHz signals */

#define LP_CNG                                0         /* LP-based CNG in DTX operation */
#define FD_CNG                                1         /* FD-based CNG in DTX operation */

/*----------------------------------------------------------------------------------*
 * Bitrates
 *----------------------------------------------------------------------------------*/

#define FRAME_NO_DATA                         0         /* Frame with no data */
#define SID_1k75                              1750      /* SID at 1.75 kbps               (used only in AMR-WB IO mode   */
#define SID_2k40                              2400      /* SID at 2.40 kbps                                              */
#define PPP_NELP_2k80                         2800      /* PPP and NELP at 2.80 kbps      (used only for SC-VBR)         */

#define ACELP_3k60                            3600      /* ACELP core layer at 3.60 kbps (used only for channel-aware mode) */
#define ACELP_3k70                            3700      /* ACELP core layer at 3.60 kbps (used only for channel-aware mode) */
#define ACELP_5k90                            5900      /* ACELP core layer at average bitrate of 5.90 kbps (used only in SC-VBR mode)      */
#define ACELP_6k60                            6600      /* ACELP core layer at 6.60  kbps (used only in AMR-WB IO mode)  */
#define ACELP_7k20                            7200      /* ACELP core layer at 7.20  kbps                                */
#define ACELP_8k00                            8000      /* ACELP core layer at 8     kbps                                */
#define ACELP_8k85                            8850      /* ACELP core layer at 8.85  kbps (used only in AMR-WB IO mode)  */
#define ACELP_9k25                            9250      /* ACELP core layer at 9.25  kbps (used for WB BWE)              */
#define ACELP_9k60                            9600      /* ACELP core layer at 9.60  kbps                                */
#define ACELP_11k60                           11600     /* ACELP core layer at 11.60 kbps (used for SWB TBE)             */
#define ACELP_12k15                           12150     /* ACELP core layer at 12.20 kbps                                */
#define ACELP_12k65                           12650     /* ACELP core layer at 12.65 kbps (used only in AMR-WB IO mode)  */
#define ACELP_12k85                           12850     /* ACELP core layer at 12.85 kbps (used for WB BWE)              */
#define ACELP_13k20                           13200     /* ACELP core layer at 13.20 kbps                                */
#define ACELP_14k25                           14250     /* ACELP core layer at 14.25 kbps (used only in AMR-WB IO mode)  */
#define ACELP_14k80                           14800     /* ACELP core layer at 14.80 kbps (used for SWB TBE )            */
#define ACELP_15k85                           15850     /* ACELP core layer at 15.85 kbps (used only in AMR-WB IO mode)  */
#define ACELP_16k40                           16400     /* ACELP core layer at 16.40 kbps                                */
#define ACELP_18k25                           18250     /* ACELP core layer at 18.25 kbps (used only in AMR-WB IO mode)  */
#define ACELP_19k85                           19850     /* ACELP core layer at 19.85 kbps (used only in AMR-WB IO mode)  */
#define ACELP_22k60                           22600     /* ACELP core layer at 22.60 kbps (used for FB + SWB TBE)        */
#define ACELP_22k80                           22800     /* ACELP core layer at 22.80 kbps (used for SWB TBE)             */
#define ACELP_23k05                           23050     /* ACELP core layer at 23.05 kbps (used only in AMR-WB IO mode)  */
#define ACELP_23k85                           23850     /* ACELP core layer at 23.85 kbps (used only in AMR-WB IO mode)  */
#define ACELP_24k40                           24400     /* ACELP core layer at 24.40 kbps                                */
#define ACELP_29k00                           29000     /* ACELP core layer at 29.00 kbps (used for FB + SWB TBE)        */
#define ACELP_29k20                           29200     /* ACELP core layer at 29.20 kbps (used for SWB TBE)             */
#define ACELP_30k20                           30200     /* ACELP core layer at 30.20 kbps (used for FB + SWB BWE)        */
#define ACELP_30k40                           30400     /* ACELP core layer at 30.40 kbps (used for SWB BWE)             */
#define ACELP_32k                             32000     /* ACELP core layer at 32    kbps                                */
#define ACELP_48k                             48000     /* ACELP core layer at 48    kbps                                */
#define ACELP_64k                             64000     /* ACELP core layer at 64    kbps                                */

#define HQ_16k40                              16400     /* HQ core at 16.4 kbps   */
#define HQ_13k20                              13200     /* HQ core at 13.2 kbps */
#define HQ_24k40                              24400     /* HQ core at 24.4 kbps */
#define HQ_32k                                32000     /* HQ core at 32 kbps */
#define HQ_48k                                48000     /* HQ core at 48 kbps */
#define HQ_64k                                64000     /* HQ core at 64 kbps */
#define HQ_96k                                96000     /* HQ core at 96 kbps */
#define HQ_128k                               128000    /* HQ core at 128 kbps */

#define WB_TBE_0k35                           350       /* WB TBE layer (used only at 9.6 kbps on top of ACELP@12k8 core for 16kHz signals) */
#define WB_BWE_0k35                           350       /* WB BWE layer (used only on top of ACELP@12k8 core for 16kHz signals) */
#define WB_TBE_1k05                           1050      /* WB TBE layer (used only on top of ACELP@12k8 core for 16kHz signals) */
#define SWB_TBE_1k6                           1600      /* SWB TBE layer */
#define SWB_BWE_1k6                           1600      /* SWB BWE layer */
#define WB_TBE_0k65                           650       /* WB TBE layer (used only in channel-aware mode at 16.4 kbps on top of ACELP@12k8 core for 16kHz signals) */
#define SWB_TBE_1k2                           1200      /* SWB TBE layer (used only in channel-aware mode) */
#define FB_TBE_1k8                            1800      /* SWB+FB TBE layer (used only for 48kHz signals) */
#define FB_BWE_1k8                            1800      /* SWB+FB BWE layer (used only for 48kHz signals) */

#define SWB_TBE_2k8                           2800      /* SWB TBE layer @32kbps */
#define FB_TBE_3k0                            3000      /* SWB+FB TBE layer @32kbps (used only for 48kHz signals) */

#define SWB_BWE_16k                           16000     /* SWB BWE layer for highrate SWB speech */

/* Combine parameters into a single index (used to retrieve number of bits from bit allocation tables) */
#define LSF_BIT_ALLOC_IDX_fx(brate,ctype)     (L_mac0(L_mult0(6, BRATE2IDX_fx(brate)), (ctype), 1))

/* Combine coder_type, bandwidth, formant sharpening flag, and channel-aware flag into one indice */
#define SIG2IND_fx(ctype, bw, sf, ca_rf)      L_mac0(L_mac0(L_mac0(L_mult0(ctype, 1), bw, 1<<3), sf, 1<<6), ca_rf, 1<<7)

#define MAX_ACELP_SIG 194

/*----------------------------------------------------------------------------------*
 * Bitstream indices
 *----------------------------------------------------------------------------------*/

enum
{
    IND_CORE,
    IND_PPP_NELP_MODE,
    IND_SID_TYPE,
    IND_ACELP_16KHZ,
    IND_ACELP_SIGNALLING,
    IND_MDCT_CORE,
    IND_BWE_FLAG,
    IND_HQ_SWITCHING_FLG,
    IND_LAST_L_FRAME,
    IND_VAD_FLAG,
    IND_HQ_BWIDTH,
    IND_TC_SUBFR,

    IND_LSF_PREDICTOR_SELECT_BIT          = IND_TC_SUBFR + 4,
    IND_LSF,
    IND_MID_FRAME_LSF_INDEX                = IND_LSF + 17,

    IND_ISF_0_0,
    IND_ISF_0_1,
    IND_ISF_0_2,
    IND_ISF_0_3,
    IND_ISF_0_4,
    IND_ISF_1_0,
    IND_ISF_1_1,
    IND_ISF_1_2,
    IND_ISF_1_3,
    IND_ISF_1_4,

    IND_GSC_ATTACK,
    IND_GSC_SWB_SPEECH,
    IND_NOISE_LEVEL,
    IND_HF_NOISE,
    IND_PIT_CONTR_IDX,
    IND_FEC_CLAS,
    IND_FEC_ENR,
    IND_FEC_POS,
    IND_ES_PRED,
    IND_HARM_FLAG_ACELP,
    /* ------------- Loop for alg. codebook indices at 24.4 kbps (special case) -------------- */
    TAG_ALG_CDBK_4T64_24KBIT_START,
    IND_ALG_CDBK_4T64_1_24KBIT             = TAG_ALG_CDBK_4T64_24KBIT_START,
    IND_ALG_CDBK_4T64_2_24KBIT             = TAG_ALG_CDBK_4T64_24KBIT_START,
    TAG_ALG_CDBK_4T64_24KBIT_END           = TAG_ALG_CDBK_4T64_24KBIT_START + 40,
    /* ------------------------------------------------ */

    /* ------------- ACELP subframe loop -------------- */
    TAG_ACELP_SUBFR_LOOP_START,
    IND_PITCH                              = TAG_ACELP_SUBFR_LOOP_START,
    IND_LP_FILT_SELECT                     = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_1T64                      = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_2T32                      = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_4T64                      = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_4T64_1                    = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_4T64_2                    = TAG_ACELP_SUBFR_LOOP_START,
    IND_ALG_CDBK_4T64_1BIT                 = TAG_ACELP_SUBFR_LOOP_START,
    IND_GAUS_CDBK_INDEX                    = TAG_ACELP_SUBFR_LOOP_START,
    IND_TILT_FACTOR                        = TAG_ACELP_SUBFR_LOOP_START,
    IND_GAIN                               = TAG_ACELP_SUBFR_LOOP_START,
    IND_GAIN_CODE                          = TAG_ACELP_SUBFR_LOOP_START,
    IND_TC_IMP_SHAPE                       = TAG_ACELP_SUBFR_LOOP_START,
    IND_TC_IMP_POS                         = TAG_ACELP_SUBFR_LOOP_START,
    IND_TC_IMP_SIGN                        = TAG_ACELP_SUBFR_LOOP_START,
    IND_TC_IMP_GAIN                        = TAG_ACELP_SUBFR_LOOP_START,
    IND_GAIN_PIT                           = TAG_ACELP_SUBFR_LOOP_START,
    IND_PIT_IDX                            = TAG_ACELP_SUBFR_LOOP_START,
    IND_AVQ_GAIN                           = TAG_ACELP_SUBFR_LOOP_START,
    IND_I                                  = TAG_ACELP_SUBFR_LOOP_START,
    IND_KV                                 = TAG_ACELP_SUBFR_LOOP_START,
    IND_NQ                                 = TAG_ACELP_SUBFR_LOOP_START,
    IND_HF_GAIN_MODIFICATION               = TAG_ACELP_SUBFR_LOOP_START,
    TAG_ACELP_SUBFR_LOOP_END               = TAG_ACELP_SUBFR_LOOP_START + 300,
    /* ------------------------------------------------ */

    IND_MEAN_GAIN2,
    IND_Y_GAIN_TMP                         = IND_MEAN_GAIN2 + 32,
    IND_Y_GAIN_HF                          = IND_Y_GAIN_TMP + 32,
    IND_HQ_VOICING_FLAG,
    IND_HQ_SWB_CLAS,
    IND_NF_IDX,
    IND_LC_MODE,
    IND_YNRM,
    IND_HQ_SWB_EXC_SP_CLAS                 = IND_YNRM + 44,
    IND_HQ_SWB_EXC_CLAS                    = IND_HQ_SWB_EXC_SP_CLAS,
    IND_SWB_FENV_HQ                        = IND_HQ_SWB_EXC_CLAS,
    IND_FB_FENV_HQ                         = IND_SWB_FENV_HQ + 5,
    IND_DELTA_ENV_HQ                       = IND_FB_FENV_HQ + 5,
    IND_HVQ_BWE_NL,
    IND_NUM_PEAKS                          = IND_HVQ_BWE_NL + 2,
    IND_POS_IDX,
    IND_FLAGN                              = IND_POS_IDX + 280,
    IND_PG_IDX,
    IND_HVQ_PEAKS                          = IND_PG_IDX + 27,
    IND_HVQ_NF_GAIN                        = IND_HVQ_PEAKS + 54,
    IND_HQ2_SWB_CLAS                       = IND_HVQ_NF_GAIN + 2,
    IND_HQ2_DENG_MODE,
    IND_HQ2_DENG_8SMODE,
    IND_HQ2_DENG_8SMODE_N0,
    IND_HQ2_DENG_8SMODE_N1,
    IND_HQ2_DENG_8SPOS,
    IND_HQ2_DENG_8SDEPTH,
    IND_HQ2_DENG_HMODE,
    IND_HQ2_DIFF_ENERGY,
    IND_HQ2_P2A_FLAGS                      = IND_HQ2_DIFF_ENERGY + 100,
    IND_HQ2_LAST_BA_MAX_BAND               = IND_HQ2_P2A_FLAGS + 60,

    IND_FPC_INDICE                         = IND_HQ2_LAST_BA_MAX_BAND + 2,

    IND_RC_START                           = IND_FPC_INDICE + 160,
    IND_RC_END                             = IND_RC_START + 320,
    IND_HVQ_PVQ_GAIN                       = IND_RC_END,
    IND_NOISINESS                          = IND_HVQ_PVQ_GAIN + 8,
    IND_ENERGY,
    IND_CNG_HO,
    IND_SID_BW,
    IND_CNG_ENV1,
    IND_WB_FENV,
    IND_WB_CLASS,
    IND_IG1,
    IND_IG2A,
    IND_IG2B,
    IND_NELP_FID,
    IND_DELTALAG,
    IND_POWER,
    IND_AMP0,
    IND_AMP1,
    IND_GLOBAL_ALIGNMENT,
    IND_PVQ_FINE_GAIN,
    IND_UV_FLAG,
    IND_SHB_SUBGAIN                        = IND_PVQ_FINE_GAIN + 44,
    IND_SHB_FRAMEGAIN,
    IND_SHB_ENER_SF,
    IND_SHB_RES_GS1,
    IND_SHB_RES_GS2,
    IND_SHB_RES_GS3,
    IND_SHB_RES_GS4,
    IND_SHB_RES_GS5,
    IND_SHB_VF,
    IND_SHB_LSF,
    IND_SHB_MIRROR                         = IND_SHB_LSF + 5,
    IND_SHB_GRID,
    IND_SWB_CLASS,
    IND_SWB_TENV,
    IND_SWB_FENV                           = IND_SWB_TENV + 4,
    IND_SHB_CNG_GAIN                       = IND_SWB_FENV + 4,
    IND_DITHERING,
    IND_FB_SLOPE,

    IND_HQ2_SPT_SHORTEN,
    IND_HQ2_SUBBAND_FPC,
    IND_HQ2_SUBBAND_GAIN                   = IND_HQ2_SUBBAND_FPC + 100,
    IND_HQ2_DUMMY                          = IND_HQ2_SUBBAND_GAIN + 20,

    IND_LAGINDICES,
    IND_NOISEG,
    IND_AUDIO_GAIN,
    IND_AUDIO_DELAY,

    /* ------------- HR SWB BWE loop -------------- */
    TAG_HR_BWE_LOOP_START                  = IND_AUDIO_DELAY + 4,
    IND_HR_IS_TRANSIENT                    = TAG_HR_BWE_LOOP_START,
    IND_HR_GAIN                            = TAG_HR_BWE_LOOP_START,
    IND_HR_ENVELOPE                        = TAG_HR_BWE_LOOP_START,
    IND_HR_HF_GAIN                         = TAG_HR_BWE_LOOP_START,
    IND_I2                                 = TAG_HR_BWE_LOOP_START,
    IND_KV2                                = TAG_HR_BWE_LOOP_START,
    IND_NQ2                                = TAG_HR_BWE_LOOP_START,
    TAG_HR_BWE_LOOP_END                    = TAG_HR_BWE_LOOP_START + 200,
    /* ------------------------------------------------ */

    IND_CORE_SWITCHING_CELP_SUBFRAME,
    IND_CORE_SWITCHING_AUDIO_DELAY         = IND_CORE_SWITCHING_CELP_SUBFRAME + 20,
    IND_CORE_SWITCHING_AUDIO_GAIN,

    IND_UNUSED,

    IND_RF_FLAG                            = IND_UNUSED + 127,
    IND_RF_FRAME_TYPE,
    IND_RF_ISF_PREDICTOR_SELECT_BIT,
    IND_RF_ISF,
    IND_RF_MID_FRAME_LSF_INDEX             = IND_RF_ISF + 17,
    IND_RF_ES_PRED,
    /* ------------- RF ACELP subframe loop -------------- */
    TAG_RF_ACELP_SUBFR_LOOP_START,
    IND_RF_PITCH                           = TAG_RF_ACELP_SUBFR_LOOP_START,
    IND_RF_ALG_CDBK_1T64                   = TAG_RF_ACELP_SUBFR_LOOP_START,
    IND_RF_ALG_CDBK_2T32                   = TAG_RF_ACELP_SUBFR_LOOP_START,
    IND_RF_GAIN                            = TAG_RF_ACELP_SUBFR_LOOP_START,
    TAG_RF_ACELP_SUBFR_LOOP_END            = TAG_RF_ACELP_SUBFR_LOOP_START + 300,
    /*-----------------------------------------------------*/
    IND_RF_IG1,
    IND_RF_IG2A,
    IND_RF_IG2B,
    IND_RF_NELP_FID,
    IND_RF_BWE_GAINFRAME,
    IND_RF_BWE_LSF,
    IND_RF_BWE_GAINSHAPE,
    IND_RF_RSVD,
    IND_RF_RSVD_WB16K4,

    MAX_NUM_INDICES                        = IND_RF_RSVD_WB16K4 + 15
};


/*----------------------------------------------------------------------------------*
 * Delays
 *----------------------------------------------------------------------------------*/

#define FRAME_SIZE_NS                         20000000L

#define ACELP_LOOK_NS                         8750000L
#define DELAY_FIR_RESAMPL_NS                  937500L
#define DELAY_CLDFB_NS                        1250000L

#define DELAY_SWB_TBE_12k8_NS                 1250000L
#define DELAY_SWB_TBE_16k_NS                  1125000L
#define MAX_DELAY_TBE_NS                      1312500L
#define DELAY_BWE_TOTAL_NS                    2312500L
#define DELAY_FD_BWE_ENC_12k8_NS             (DELAY_BWE_TOTAL_NS - (MAX_DELAY_TBE_NS - DELAY_SWB_TBE_12k8_NS))
#define DELAY_FD_BWE_ENC_16k_NS              (DELAY_BWE_TOTAL_NS - (MAX_DELAY_TBE_NS - DELAY_SWB_TBE_16k_NS))
#define DELAY_FD_BWE_ENC_NS                   2250000L

#define L_LOOK_12k8                           NS2SA(INT_FS_FX, ACELP_LOOK_NS)     /* look-ahead length at 12.8kHz */
#define L_LOOK_16k                            NS2SA(INT_FS_16k_FX, ACELP_LOOK_NS)      /* look-ahead length at 16kHz   */

/* core switching constants @16kHz */
#define SWITCH_GAP_LENGTH_NS                  6250000L            /* lenght of ACELP->HQ switching gap in ms  */
#define HQ_DELAY_COMP                         NS2SA(8000, DELAY_CLDFB_NS)
#define HQ_DELTA_MAX                          6    /* maximum multiplication factor (==48kHz/8kHz) for core switching modules */

#define N_ZERO_MDCT_NS                        5625000L            /* Number of zeros in ms for MDCT */
#define NL_BUFF_OFFSET                        12

#define N_WS2N_FRAMES                         40        /* number of frames for attenuation during the band-width switching */
#define N_NS2W_FRAMES                         20        /* number of frames for attenuation during the band-width switching */

/*----------------------------------------------------------------------------------*
 * Coder types (only for ACELP core when not running in AMR-WB IO mode)
 *----------------------------------------------------------------------------------*/

#define INACTIVE                              0         /* inactive      */
#define UNVOICED                              1         /* unvoiced      */
#define VOICED                                2         /* purely voiced */
#define GENERIC                               3         /* generic       */
#define TRANSITION                            4         /* transition    */
#define AUDIO                                 5         /* audio (GSC)   */
#define LR_MDCT                               6         /* low-rate MDCT core */


/*--------------------------------------------------*
 * Partial copy frame types (only for ACELP core )
 *--------------------------------------------------*/

/* TCX partial copy frame types */
#define RF_NO_DATA                            0
#define RF_TCXFD                              1
#define RF_TCXTD1                             2
#define RF_TCXTD2                             3
/* ACELP partial copy frame types */
#define RF_ALLPRED              ACELP_MODE_MAX
#define RF_NOPRED           ACELP_MODE_MAX + 1
#define RF_GENPRED          ACELP_MODE_MAX + 2
#define RF_NELP             ACELP_MODE_MAX + 3


/*--------------------------------------------------------------*
 * MODE2 Frame length constants
 *---------------------------------------------------------------*/

#define L_FRAME_MAX           960         /* Max 20ms frame size @48kHz                   */
#define L_FRAME_PLUS         1200         /* Max frame size (long TCX frame)           */
#define L_NEXT_MAX            420         /* maximum encoder lookahead                    */
#define L_PAST_MAX            540         /* maximum encoder past samples                 */
#define L_LPC_ANA_WINDOW_MAX  800         /* = AMR-WB window scaled for FB (480 framing) */
#define L_MDCT_OVLP_MAX       480         /* = mdct overlap for FB (480 framing) */
#define N_TCX10_MAX           480         /* Max size of TCX10 MDCT size */
#define SYN_SFD_MAX             4         /* Max of next nearest integer of subframe corresponding to the decoder delay when bpf is active */
#define BITS_TEC                1         /* number of bits for TEC */
#define BITS_TFA                1         /* number of bits for TTF */
#define N_TEC_TFA_SUBFR        16         /* number of subframes of TEC/TFA */
#define L_TEC_TFA_SUBFR16k     (L_FRAME16k/N_TEC_TFA_SUBFR)       /* TEC/TFA subframe size @ 16kHz*/
#define MAX_TEC_SMOOTHING_DEG   6         /* max degree of smoothing for TEC */
#define N_MAX                1200         /* Max size of MDCT spectrum = 25ms @ 48kHz */

#define L_FRAME_12k8            256     /* Frame size at 12k8Hz: 20ms = 256 samples  */
#define L_FRAME_16k             320     /* Frame size at 16kHz:  20ms = 320 samples  */
#define L_NEXT_MAX_12k8         112     /* maximum encoder lookahead at 12k8Hz       */
#define L_PAST_MAX_12k8         144     /* maximum encoder past samples at 12k8Hz    */
#define L_DIV                   256     /* 20ms frame size (ACELP or short TCX frame) */
#define L_DIV_MAX               320
#define NB_DIV                  2       /* number of division (frame) per 20ms frame  */
#define NB_SUBFR                4       /* number of 5ms subframe per 20ms frame      */

#define L_NEXT_MAX_16k        140       /* maximum encoder lookahead at 16kHz        */
#define L_PAST_MAX_16k        180       /* maximum encoder past samples at 16kHz     */

#define L_NEXT_MAX_32k        280       /* maximum encoder lookahead at 32kHz        */
#define L_PAST_MAX_32k        360       /* maximum encoder past samples at 32kHz     */

#define MIDLSF_NBITS            5
#define ENDLSF_NBITS            31

/*----------------------------------------------------------------------------------*
 * ACELP core constants
 *----------------------------------------------------------------------------------*/

#define ACELP_MODE_MAX 4

#define RF_MODE_MAX     4

#define M                                     16        /* order of the LP filter @ 12.8kHz           */
#define L_FRAME                               256       /* frame size at 12.8kHz                      */
#define NB_SUBFR                              4         /* number of subframes per frame              */
#define L_SUBFR                               (L_FRAME/NB_SUBFR)            /* subframe size                              */
#define L_SUBFR_Q6                            ((L_FRAME/NB_SUBFR)*64)       /* subframe size                              */
#define L_SUBFR_Q16                           ((L_FRAME/NB_SUBFR)*65536)    /* subframe size                              */

#define L_INP_MEM                             (L_LOOK_16k + ((L_LP_16k - (NS2SA(INT_FS_16k, ACELP_LOOK_NS) + L_SUBFR16k/2)) - 3*L_SUBFR16k/2))            /* length of memory of input signal, given by the Look-Ahead + the past memory (max needed for the LP window at 16 kHz) */
#define L_INP_12k8                            (L_INP_MEM + L_FRAME)      /* length of input signal buffer @12.8kHz */
#define L_INP                                 (L_INP_MEM + L_FRAME32k)   /* length of input signal buffer */

#define L_EXC_MEM                             L_FRAME16k /*(PIT16k_MAX + L_INTERPOL)*/  /* length of memory of excitation signal */
#define L_EXC_MEM_12k8                        (PIT_MAX + L_INTERPOL)            /* length of memory of excitation signal @12.8kHz   */
#define L_EXC_MEM_DEC                         (3*L_FRAME16k/2)              /*Half-frame needed for MODE2 PLC in case of TCX->ACELP*/
#define L_EXC                                 (L_EXC_MEM + L_FRAME16k + 1)  /* length of excitation signal buffer */
#define L_EXC_DEC                             (L_EXC_MEM_DEC + L_FRAME16k + 1 + L_SUBFR)  /* length of decoder excitation signal buffer @16kHz*/
#define L_SYN_MEM                             NS2SA(48000,DELAY_CLDFB_NS)   /* synthesis memory length, 1.25ms @ 48kHz          */
#define L_SYN                                 (L_SYN_MEM + L_FRAME16k)      /* length of synthesis signal buffer */
#define L_WSP_MEM                             (PIT_MAX + L_INTERPOL)        /* length of memory for weighted input signal */
#define L_WSP                                 (L_WSP_MEM + L_FRAME + L_LOOK_12k8) /* length of weighted input signal buffer */

#define OLD_SYNTH_SIZE_DEC                    (2*L_FRAME_MAX)                   /* decoder past synthesis; needed for LTP, PLC and rate switching*/
#define OLD_SYNTH_SIZE_ENC                    L_FRAME_MAX+L_FRAME_MAX/4         /* encoder synth memory */
#define OLD_EXC_SIZE_DEC                      (3*L_FRAME_MAX/2+2*L_FIR_FER2) /*old excitation needed for decoder for PLC*/

#define TILT_CODE                             9830     /* ACELP code preemphasis factor ~=0.3f (0Q15) (=0.299987792968750) */

#define L_SUBFR16k                            (L_FRAME16k/NB_SUBFR)       /* subframe size at 16kHz                     */
#define L_HALFR16k                            (2*L_SUBFR16k)              /* half-frame size at 16kHz                   */

#define L_INTERPOL2                           16        /* Length of filter for interpolation         */
#define L_INTERPOL                            (L_INTERPOL2+1) /* Length of filter for interpolation         */
#define TILT_FAC                              0.68f    /* tilt factor (denominator)                  */
#define TILT_FAC_FX                           22282    /* tilt factor (denominator) fixed-point*/
#define M16k                                  20       /* order of the LP filter @ 16kHz             */
#define PIT_SHARP_fx                          27853    /* pitch sharpening factor */
#define F_PIT_SHARP                           0.85F    /* pitch sharpening factor                    */
#define PIT_UP_SAMP                           4
#define UP_SAMP                               4         /* upsampling factor for 1/4 interpolation filter */
#define PIT_L_INTERPOL2                       16
#define PIT_FIR_SIZE2                         (PIT_UP_SAMP*PIT_L_INTERPOL2+1)
#define PIT_UP_SAMP6                          6
#define PIT_L_INTERPOL6_2                     17
#define PIT_FIR_SIZE6_2                       (PIT_UP_SAMP6*PIT_L_INTERPOL6_2+1)
#define E_MIN_FX                              1       /* QSCALE (Q7)*/
#define STEP_DELTA_FX                         11
#define FORMANT_SHARPENING_NOISE_THRESHOLD_FX   5376     /* 21 (!8)lp_noise level above which formant sharpening is deactivated - at this level most of 20 dB SNR office noisy speech still uses sharpening */
#define LP_NOISE_THRESH                       FL2WORD32_SCALE(20.f, 8)
#define LFAC                                  160                          /* FAC maximum frame length */

#define L_FILT_UP8k                           24        /* Resampling - delay of filter for  8 kHz output signals (at 12.8 kHz sampling rate) */
#define RS_UP_FACT                            0x01
#define LEN_WIN_SSS                           120
#define L_FILT                                12        /* Delay of the low-pass filter in the BPF        */
#define L_FILT8k                              16        /* Resampling - delay of filter for  8 kHz input signals (at 8kHz sampling rate) */
#define L_FILT16k                             15        /* Resampling - delay of filter for 16 kHz input signals (at 16kHz sampling rate) */
#define L_FILT32k                             30        /* Resampling - delay of filter for 32 kHz input signals (at 32kHz sampling rate) */
#define L_FILT48k                             45        /* Resampling - delay of filter for 48 kHz input signals (at 48kHz sampling rate) */
#define L_FILT_UP16k                          12        /* Resampling - delay of filter for 16 kHz output signals (at 12.8 kHz sampling rate) */
#define L_FILT_UP32k                          12        /* Resampling - delay of filter for 32 kHz output signals (at 12.8 kHz sampling rate) */
#define L_FILT_UP48k                          12        /* Resampling - delay of filter for 48 kHz output signals (at 12.8 kHz sampling rate) */
#define L_FILT_MAX                            L_FILT48k /* Resampling - maximum length of all filters - for memories */
#define FAC_16k                               4         /* Resampling - factor for 16kHz */
#define FAC_12k8                              5         /* Resampling - factor for 12.8kHz */
#define RS_INV_FAC                            0x8000    /* Resampling - flag needed in rom_com and modif_fs to allow pre-scaled and non pre-scaled filters */

#define CLDFB_NO_CHANNELS_MAX                 60        /* CLDFB resampling - max number of CLDFB channels */
#define CLDFB_NO_COL_MAX                      16        /* CLDFB resampling - max number of CLDFB col. */
#define CLDFB_NO_COL_MAX_SWITCH               6         /* CLDFB resampling - max number of CLDFB col. for switching */
#define CLDFB_NO_COL_MAX_SWITCH_BFI           8         /* CLDFB resampling - max number of CLDFB col. for switching */

#define CLDFB_ANALYSIS                        0
#define CLDFB_SYNTHESIS                       1

#define L_FFT                                 256       /* Spectral analysis - length of the FFT */
#define LOG2_L_FFT                            8         /* Spectral analysis - log2 of L_FFT */

#define FFT_OFF2                              0         /* Spectral analysis - offset of the second analysis windows from the end of the look-ahead */
#define FFT_OFF1                              (FFT_OFF2 - 128) /* Spectral analysis - offset of the first analysis window from the end of the look-ahead */

#define BIN                                   (INT_FS_FX/L_FFT)/* Spectral analysis - Width of one frequency bin in Hz */
#define BIN4                                  800     /* BIN in Q4, i.e. BIN*2^4 */
#define NB_BANDS                              20        /* Spectral analysis - number of frequency bands */
#define VOIC_BINS                             74        /* Spectral analysis - max number of frequency bins considered as voiced (related to VOIC_BAND and L_FFT) */
#define VOIC_BAND                             17        /* Spectral analysis - number of critical bands considered as voiced (related to VOIC_BINS) */
#define VOIC_BINS_8k                          115       /* Spectral analysis - max number of frequency bins considered as voiced in NB (related to VOIC_BAND_8k and L_FFT) */
#define VOIC_BAND_8k                          17        /* Spectral analysis - number of critical bands considered as voiced in NB (related to VOIC_BINS_8k) */
#define N_SPEC                                (L_FFT/2)   /* number of spectral bins */

#define M_GAMMA_FX                            32440      /* Q15 - forgetting factor of active speech decision predictor */
#define M_GAMMA_M1FX                          21474836   /* Q31 - one minus forgetting factor of active speech decision predictor */
#define M_ALPHA_FX                            29491      /* Q15 - forgetting factor of LT correlation map */
#define ONE_MINUS_M_ALPHA                     3277       /* Q15 - one minus forgetting factor of LT correlation map */
#define THR_CORR_INIT_FX                      (56<<9 )   /* Q9 - starting threshold of multi-harm. correlation */
#define THR_NCHAR_WB_FX                       2048       /* Q11 threshold for noise character (WB) */
#define THR_NCHAR_NB_FX                       2048       /* Q11 threshold for noise character (NB) */

#define L_LP                                  320       /* LP analysis - LP window size */
#define L_LP_16k                              400       /* LP analysis @16kHz - LP window size for 16kHz */
#define L_LP_AMR_WB                           384       /* LP analysis - windows size (only for AMR-WB IO mode) */
#define GRID100_POINTS                        100       /* LP analysis - number of points to evaluate Chebyshev polynomials */
#define GRID80_POINTS                         80        /* LP analysis - number of points to evaluate Chebyshev polynomials used in the LP coefs. conversion */

#define GRID50_POINTS                         51        /* LP analysis - half-number of points to evaluate Chebyshev polynomials used in the LP coefs. conversion */
#define GRID40_POINTS                         41        /* LP analysis - half-number of points to evaluate Chebyshev polynomials used in the LP coefs. conversion */

#define PIT_MIN                               34        /* OL pitch analysis - Minimum pitch lag       */
#define PIT_MAX                               231       /* OL pitch analysis - Maximum pitch lag                          */
#define PIT_MIN_EXTEND                        20        /* OL pitch analysis - Minimum pitch lag of extended range     */
#define PIT_MAX_EXTEND                        231       /* OL pitch analysis - Maximum pitch lag of extended range     */
#define PIT_MIN_DOUBLEEXTEND                  17        /* OL pitch analysis - Minimum pitch lag of double-extended range     */
#define OPL_DECIM                             2         /* OL pitch analysis - decimation factor */
#define L_INTERPOL1                           4         /* OL pitch analysis - interval to compute normalized correlation */
#define FIR_SIZE1                             (UP_SAMP*L_INTERPOL1+1) /* OL pitch analysis - total length of the 1/4 interpolation filter */

#define PIT_MIN_SHORTER 29                              /* OL pitch analysis - minimum for wider pitch, MODE2 specific constant */

#define PIT_MIN_12k8                          29        /* Minimum pitch lag with resolution 1/4      */
#define PIT_FR2_12k8                          121       /* Minimum pitch lag with resolution 1/2      */
#define PIT_FR1_12k8                          154       /* Minimum pitch lag with resolution 1        */
#define PIT_MAX_12k8                          231       /* Maximum pitch lag                          */
#define PIT_FR1_8b_12k8                       82        /* Minimum pitch lag with resolution 1 for low bit-rate pitch delay codings*/
#define PIT_MIN_16k                           36
#define PIT_FR2_16k                           36
#define PIT_FR1_16k                           165
#define PIT_FR1_8b_16k                        165
#define PIT_MIN_25k6                          58
#define PIT_FR2_25k6                          58
#define PIT_FR1_25k6                          164
#define PIT_MAX_25k6                          463
#define PIT_FR1_8b_25k6                       164
#define PIT_MIN_32k                           72
#define PIT_FR2_32k                           72
#define PIT_FR1_32k                           75
#define PIT_MAX_32k                           577
#define PIT_FR1_8b_32k                        75
#define PIT_MAX_MAX                           PIT_MAX_32k
#define PIT_MAX_16k                           289

#define PIT_FR1_8b                            92        /* Pitch encoding - Minimum pitch lag with resolution 1        */
#define PIT_FR2_9b                            128       /* Pitch encoding - Minimum pitch lag with resolution 1/2      */
#define PIT_FR1_9b                            160       /* Pitch encoding - Minimum pitch lag with resolution 1        */
#define PIT_FR1_EXTEND_8b                     64        /* Pitch encoding - Minimum pitch lag with resolution 1 of extended range       */
#define PIT_FR2_EXTEND_9b                     116       /* Pitch encoding - Minimum pitch lag with resolution 1/2 of extended range     */
#define PIT_FR1_EXTEND_9b                     128       /* Pitch encoding - Minimum pitch lag with resolution 1 of extended range       */
#define PIT_FR1_DOUBLEEXTEND_8b               58        /* Pitch encoding - Minimum pitch lag with resolution 1 of double-extended range       */
#define PIT_FR2_DOUBLEEXTEND_9b               112       /* Pitch encoding - Minimum pitch lag with resolution 1/2 of double-extended range     */
#define PIT_FR1_DOUBLEEXTEND_9b               124       /* Pitch encoding - Minimum pitch lag with resolution 1 of double-extended range       */

#define LOW_PASS                              0         /* LP filtering - flag for low-pass filtering of the excitation */
#define FULL_BAND                             1         /* LP filtering - flag for no low-pass filtering of the excitation */
#define NORMAL_OPERATION                      2         /* LP filtering - flag for selecting the best of the two above */

#define NB_TRACK_FCB_2T                       2         /* Algebraic codebook - number of tracks in algebraic fixed codebook search with 2 tracks */
#define NB_POS_FCB_2T                         32        /* Algebraic codebook - number of positions in algebraic fixed codebook search with 2 tracks */
#define NB_TRACK_FCB_4T                       4         /* Algebraic codebook - number of tracks in algebraic fixed codebook search with 4 tracks */
#define NB_POS_FCB_4T                         16        /* Algebraic codebook - number of positions in algebraic fixed codebook search with 4 tracks */
#define NB_PULSE_MAX                          36
#define NPMAXPT                               ((NB_PULSE_MAX+NB_TRACK_FCB_4T-1)/NB_TRACK_FCB_4T)

#define GAIN_PRED_ORDER                       4         /* Gain quantization - prediction order for gain quantizer (only for AMR-WB IO mode) */
#define MEAN_ENER                             30        /* Gain quantization - average innovation energy */

#define DTX_HIST_SIZE                         8         /* CNG & DTX - number of last signal frames used for CNG averaging */
#define CNG_ISF_FACT_FX                       29491     /* Q15(0.9), CNG & DTX - CNG spectral envelope smoothing factor*/
#define STEP_AMR_WB_SID_FX                    10752     /* Q12 */
#define HO_HIST_SIZE                          8         /* CNG & DTX - maximal number of hangover frames used for averaging */

#define NUM_ENV_CNG                           20
#define BUF_L_NRG_FX                          22938     /* Q15(0.7), CNG & DTX - lower threshold offset for hangover updates */
#define ONE_OVER_BUF_H_NRG_FX                 31814     /* Q15(1/1.03), CNG & DTX - inverse of higher threshold offset for hangover updates */
#define HO_ATT_FAC_FX                         3277      /* Q15(0.1),  CNG & DTX - Hangover frame attenuation rate factor */

#define BUF_DEC_RATE                          25        /* CNG & DTX - buffer size decrease rate for active frames */
#define STEP_SID_FX                           21504     /* Q12 */
#define ISTEP_SID_FX                          6242      /* Inverse of CNG & DTX - CNG energy quantization step in Q15(1/5.25 in Q15)*/
#define ISTEP_AMR_WB_SID_FX                   12483     /* Q15(1/2.625) Inverse of CNG & DTX - CNG energy quantization step */

#define MIN_ACT_CNG_UPD                       20        /* DTX - Minimum number of consecutive active frames for CNG mode update */
#define FIXED_SID_RATE                        8         /* DTX SID rate */

#define TOTALNOISE_HIST_SIZE                  4

/************************************************************************/

/************************************************************************/
#define SUBFFT_QIN                            10
#define DATAFFT_Q                             5
#define SUBFFT_QOUT                           SUBFFT_QIN-DATAFFT_Q
#define SPECAMP_Q                             SUBFFT_QIN

#define  CLDFBVAD_NB_ID                       1
#define  CLDFBVAD_WB_ID                       2
#define  CLDFBVAD_SWB_ID                      3
#define  CLDFBVAD_FB_ID                       4
#define SP_CENTER_NUM                         4
#define STABLE_NUM                            6
#define SFM_NUM                               5
#define TONA_NUM                              3
#define PRE_SNR_NUM                           32
#define PRE_FLAG_NUM                          64
#define SPEC_AMP_NUM                          80
#define BG_ENG_NUM                            15
#define POWER_NUM                             56
#define PRE_SPEC_DIF_NUM                      56
#define CLDFBVAD_VAD_ON                       1
#define COM_VAD_ON                            2


#define MAX_CORR_SHIFT_FX 16384                         /* corrshift limit, 0.5 in Q15 */


#define START_NG                              5         /* Stationary noise UV modification */
#define FULL_NG                               10        /* Stationary noise UV modification */
#define ISP_SMOOTHING_QUANT_A1_FX             29491     /* 0.9f in Q15 */ /* Stationary noise UV modification */

#define KP559016994_FX                        18318   /* EDCT & EMDCT constants */
#define KP951056516_FX                        31164   /* EDCT & EMDCT constants */
#define KP587785252_FX                        19261   /* EDCT & EMDCT constants */
#define KP866025403_FX                        28378   /* EDCT & EMDCT constants */
#define KP250000000_FX                         8192   /* EDCT & EMDCT constants */

#define FEC_BITS_CLS                          2          /* FEC - number of bits for clas information */
#define FEC_BITS_ENR                          5          /* FEC - number of bits for energy information */
#define FEC_ENR_STEP                          (96.0f/(1<<FEC_BITS_ENR))
#define FEC_ENR_QLIMIT                        ((1<<FEC_BITS_ENR)-1)
#define FEC_BITS_POS                          8          /* FEC - number of bits for glottal pulse position */
#define L_SYN_MEM_CLAS_ESTIM                  (2*PIT16k_MAX - L_FRAME16k) /* FEC - memory of the synthesis signal for frame class estimation */
#define L_SYN_MEM_CLAS_ESTIM12k8              (2*PIT_MAX_12k8 - L_FRAME) /* FEC - memory of the synthesis signal for frame class estimation */
#define L_SYN_CLAS_ESTIM                      (L_SYN_MEM_CLAS_ESTIM + L_FRAME16k)   /* FEC - length of the synthesis signal buffer for frame class estimation */
#define L_SYN_CLAS_ESTIM12k8                  (L_SYN_MEM_CLAS_ESTIM12k8 + L_FRAME)   /* FEC - length of the synthesis signal buffer for frame class estimation */


#define L_PAST 80
#define L_CUMUL 20
#define FRAME_COUNT                           100        /* threshold of frame counter */

#define AUDIO_COUNTER_INI                     200        /* Counter initialization      */
#define AUDIO_COUNTER_STEP                    10         /* Counter increase on each audio frame      */
#define AUDIO_COUNTER_MAX                     1000        /* Counter saturation      */

#define BWD_TOTAL_WIDTH                       320        /* BWD width */
#define BWD_COUNT_MAX                         100        /* maximum value of BWD counter              */

#define PREEMPH_FAC                           22282      /* preemphasis factor at 12.8kHz (0.68f in 0Q15)              */
#define PREEMPH_FAC_16k                       23593      /* preemphasis factor at 16kHz (0.72f in 0Q15)                */
#define PREEMPH_FAC_SWB                       29491      /* preemphasis factor for super wide band (0.9f in 0Q15)      */
#define GAMMA1                                30147      /* weighting factor (numerator) default:0.92 (0Q15format)     */
#define GAMMA1_INV                            17809      /* weighting factor (numerator) default:0.92 (1Q14format)     */
#define GAMMA16k                              30802      /* weighting factor (numerator) default:0.94 (0Q15format)     */
#define GAMMA16k_INV                          17430      /* weighting factor (numerator) default:0.94 (1Q14format)     */

#define FORMANT_SHARPENING_G1                 24576      /* Formant sharpening numerator weighting at 12.8kHz (0.75f)   */
#define FORMANT_SHARPENING_G2                 29491      /* Formant sharpening denominator weighting at 12.8kHz (0.9f)  */
#define FORMANT_SHARPENING_G1_16k             26214      /* Formant sharpening numerator weighting at 16kHz (0.8f)      */
#define FORMANT_SHARPENING_G2_16k             30147      /* Formant sharpening denominator weighting at 16kHz (0.92f)   */

#define LD_FSCALE_DENOM   9
#define FSCALE_DENOM      (1 << LD_FSCALE_DENOM)
#define FSCALE_DENOM_HALF FSCALE_DENOM/2
#define FSCALE_E          2

/*----------------------------------------------------------------------------------*
 * ACELP@16kHz core constants
 *----------------------------------------------------------------------------------*/

#define NB_SUBFR16k                           5         /* number of subframes per frame @16kHz       */
#define INT_FS_16k                            16000     /* CELP core internal sampling frequency @16kHz         */

#define PIT16k_MIN                            42        /* Minimum pitch lag @16kHz                         */
#define PIT16k_FR2_9b                         118       /* Minimum pitch lag with resolution 1/2 @16kHz     */
#define PIT16k_FR1_9b                         154       /* Minimum pitch lag with resolution 1 @16kHz       */
#define PIT16k_MAX                            289       /* Maximum pitch lag @16kHz                         */
#define PIT16k_FR2_TC0_2SUBFR                 83        /* Minimum pitch lag with resolution 1/2 @16kHz for TC02, 2nd subframe */
#define PIT16k_MIN_EXTEND                     21        /* Minimum pitch lag of extended range @16kHz       */
#define PIT16k_MAX_EXTEND                     289       /* Maximum pitch lag of extended range @16kHz       */
#define PIT16k_FR2_EXTEND_9b                  88        /* Minimum 9 bit pitch lag with resolution 1/2 of extended range @16kHz     */
#define PIT16k_FR1_EXTEND_9b                  130       /* Minimum 9 bit pitch lag with resolution 1 of extended range @16kHz       */
#define PIT16k_FR2_EXTEND_10b                 264       /* Minimum 10 bit pitch lag with resolution 1/2 of extended range @16kHz    */

#define WIDTH_BAND                            8         /* sub-band width in AVQ coding                             */
#define G_AVQ_MIN_FX                          6554 /* Q13 */
#define G_AVQ_MIN_DIV10_FX                    655  /* Q13 */
#define G_AVQ_MAX_FX                          6144 /* Q6 */
#define FAC_PRE_AVQ_FX                        9830      /* preemhasis factor in ACELP pre-quantizer                            (0.3 in Q15)  */


#define G_AVQ_MIN_INACT_Q12                   2867      /* lower limit for gain Q in higher-rate ACELP contribution, inactive segments (0.7 in Q12) */
#define G_AVQ_MIN_INACT_48k_Q12               1434      /* lower limit for gain Q in higher-rate ACELP contribution, inactive segments, 48 kbit/s (0.35 in Q12) */
#define G_AVQ_MIN_INACT_64k_Q12               1024      /* lower limit for gain Q in higher-rate ACELP contribution, inactive segments, 64 kbit/s (0.25 in Q12 )*/
#define G_AVQ_DELTA_INACT_Q12                 221       /* (4.1 - 0.7) / ((1 << G_AVQ_BITS) - 1) in Q12 */
#define G_AVQ_DELTA_INACT_48k_Q12             159       /* (2.8 - 0.35) / ((1 << G_AVQ_BITS) - 1) */
#define G_AVQ_DELTA_INACT_64k_Q12             81        /*(1.5 - 0.25) / ((1 << G_AVQ_BITS) - 1) */
#define G_AVQ_BITS                            6         /* number of bits to quantize the AVQ gain in higher-rate ACELP contribtuion   */

#define G_PITCH_MIN_Q14                       0
#define G_PITCH_MAX_Q13                       9994/*1.22 */

#define G_CODE_MIN_FX                         164 /* Q13 */
#define G_CODE_MAX_FX                         320 /* Q6 */


#define G_PITCH_MIN_TC192_Q14                 1638
#define G_PITCH_MAX_MINUS_MIN_TC192_Q13       6963/*(G_PITCH_MAX_TC192 - G_PITCH_MIN_TC192) */
#define G_CODE_MIN_TC192_Q15                  19661
#define G_CODE_MIN_TC192_FX                   4915 /* Q13 */
#define G_CODE_MAX_TC192_Q0                   41

/*--------------------------------------------------------------*
 *  ACELP constants
 *---------------------------------------------------------------*/

#define MODE_MAX          15

#define NB_PULSES_MAX      15
#define ACELP_GAINS_CONST 0.8f      /* ACELP - adaptive codebook gain constraint  */


/*--------------------------------------------------------------*
 * TCX constants
 *---------------------------------------------------------------*/

#define NOISE_FILL_RANGES                     1   /* CURRENTLY BROKEN for NOISE_FILL_RANGES > 1! For dividing spectrum into multiple ranges, define NOISE_FILL_RANGES > 1. For each region one noise filling level will be transmited */
#define NBITS_NOISE_FILL_LEVEL                3   /* Number of bits used for coding noise filling level for each range */
#define MIN_NOISE_FILLING_HOLE                8
#define HOLE_SIZE_FROM_LTP(gain)   (add(4, extract_h(L_shr(L_mult0(gain, 0x6666), 10)))) /* 0x6666 -> 2.0*(4.0/0.625) (4Q11) */
#define FDNS_NPTS                             64
#define AVG_TCX20_LSF_BITS                    40
#define AVG_TCX10_LSF_BITS                    59
#define LTPSIZE                               3
#define TCXLTP_DELAY_NS                       250000
#define TCXLTP_MAX_DELAY                      NS2SA(48000,TCXLTP_DELAY_NS)
#define TCXLTP_LTP_ORDER                      24
#define TCX_RES_Q_BITS_GAIN                   3

#define LPC_SHAPED_ARI_MAX_RATE               ACELP_9k60

#define N_MAX_ARI                             800

#define SPEC_EXP_DEC                          20    /* initial decoder spectrum exponent */

/*----------------------------------------------------------------------------------*
 * TNS constants
 *----------------------------------------------------------------------------------*/

#define R1_48 690
#define R2_48 420
#define R1_16 230
#define R2_16 140
#define R1_25 368
#define R2_25 224

#define  TNS_MAX_NUM_OF_FILTERS   2                                                     /* TNS maximum number of filters                                    */
#define  TNS_MAX_FILTER_ORDER     8                                                     /* TNS maximum filter order                                         */
#define  ITF_MAX_FILTER_ORDER     16                                                    /* ITF maximum filter order                                         */
#define  NPRM_TNS                 (2+TNS_MAX_NUM_OF_FILTERS*(3+TNS_MAX_FILTER_ORDER))   /* TNS total number of quantized parameters                         */
#define  NPRM_RESQ                100                                                   /* Maximum number of parameter for residual Q in TCX                */
#define  NPRM_CTX_HM              3                                                     /* Number of Parameters for Context HM : flag+index                 */
#define  NPRM_DIV                 (2+NPRM_TNS+N_MAX/2+NPRM_RESQ+NPRM_CTX_HM)            /* Total number of quantized parameter in 1 division                */
#define  DEC_NPRM_DIV             NPRM_DIV                                              /* Total number of quantized parameter in 1 division (decoder side) */
#define  NPRM_LPC_NEW             50                                                    /* LPC total number of quantized parameters                         */

#define BITBUFSIZE (128000/50)

#define TNS_COEF_RES                          4     /* Bit resolution of the coefficients. */
#define INDEX_SHIFT                           (1 << (TNS_COEF_RES-1)) /* For shifting the index so that index 0 points to 0. */
#define FILTER_DOWNWARDS                      0
#define FILTER_UPWARDS                        1
#define DEFAULT_FILTER_DIRECTION              FILTER_UPWARDS

/*----------------------------------------------------------------------------------*
 * LSF quantization constants
 *----------------------------------------------------------------------------------*/

#define GENERIC_MA_LIMIT                      9600

#define SPC_FX                                770 /* q15 */
#define SPC_PLUS_FX                           771


#define ORDER                                 16
#define LSF_GAP_FX                            128       /*50.0f x 2.56*/

#define MODE1_LSF_GAP_FX                     179      /* MODE1_LSF_GAP*2.56 */ /* Minimum LSF separation for end-frame ISFs  */
#define LSF_GAP_FX                            128      /*50.0f in 14Q1*1.28   */
#define FREQ_MAX                              16384    /*6400Hz in 14Q1*1.28  */
#define FREQ_DIV                              800      /*400.0f in 14Q1    */

#define LSF_BITS_CNG                          29

#define ISF_GAP_FX                            128       /* Minimum ISF separation for end-frame ISFs (only in AMR-WB IO mode)  */
#define LSF_GAP_MID_FX                        205       /* 80.0 * 2.56 */ /* Minimum LSF separation for mid-frame LSFs  */
#define PREFERSFNET_FX                     1638      /* 0.05 in Q16*/
#define SFNETLOWLIMIT_WB                   3670016   /* 2.56x2.56*Q4 LSF quantizer - new sampling rate dependent thresholds used in LSF codebook decision logic, WB case */
#define SFNETLOWLIMIT_NB                   3984589   /* 2.56x2.56*Q4 LSF quantizer - new sampling rate dependent thresholds used in LSF codebook decision logic, NB case */
#define LSFMBEST                              2         /* number of survivors from one stage to another */
#define STREAKLEN                             3         /* Allow this many predictive frames, before starting limiting, For voiced +3 frames allowed */
#define STREAKMULT_FX                         26214     /* Exponential limiting multiplier */

#define LSFMBEST_MAX                          16

#define TCXLPC_NUMSTAGES                      3
#define TCXLPC_NUMBITS                        13
#define TCXLPC_IND_NUMSTAGES                  1
#define TCXLPC_IND_NUMBITS                    2
#define TCXLPC_LSF_GAP                        204 /* 80 in 14Q1*1.28 */
#define kMaxC                                 8

#define MAX_VQ_STAGES                         4         /* Maximum number of LSF VQ (3 trained and 1 LVQ) stages allowed */
#define MAX_VQ_STAGES_USED                    9          /*this is the maximum number of stages currently used and changing this will affect the memory allocated
MAX_VQ_STAGES is also used as offset for addressing some arrays, so this should NOT be changed*/

#define LEN_INDICE                            15
#define LATTICE_DIM                           8
#define NO_LEADERS                            49
#define MAX_NO_BR_LVQ                         28
#define MAX_NO_SCALES                         3
#define MAX_NO_VALS                           4
#define WB_LIMIT_LSF                          6350
#define CNG_LVQ_MODES                         16
#define MAX_NO_MODES                          128
#define START_CNG                             112
#define MAX_NO_MODES_p                        145
#define NO_CODING_MODES                       6
#define LVQ_COD_MODES                         18
/* BC-TCQ */
#define N_STAGE_VQ                            8
#define N_DIM                                 2
#define NUM_SUBSET                            8
#define OP_LOOP_THR_HVO                       1550146    /* 80% : Open-loop Threshold 2.56*2.56/16 */
#define NUM_STATE                             16        /* BC-TCQ - Number of state of the Trellis */
#define N_STAGE                               16        /* BC-TCQ - Smaple number in a frame */

#define SIZE_BK1                              256
#define SIZE_BK2                              256
#define SIZE_BK21                             64
#define SIZE_BK22                             128
#define SIZE_BK23                             128
#define SIZE_BK24                             32
#define SIZE_BK25                             32
#define SIZE_BK21_36b                         128
#define SIZE_BK22_36b                         128
#define SIZE_BK23_36b                         64

#define NB_QUA_GAIN5B   32     /* Number of quantization level        */
#define NB_QUA_GAIN6B   64     /* Number of quantization level        */
#define NB_QUA_GAIN7B   128    /* Number of quantization level        */
#define NB_QUA_GAIN8B   256

/*----------------------------------------------------------------------------------*
 * Transient detection
 *----------------------------------------------------------------------------------*/

#define NSUBBLOCKS  8               /* Number of subblocks per frame, one transient per a sub-block can be found */
#define MAX_TD_DELAY 2*NSUBBLOCKS   /* Maximum allowed delay (in number of subblocks) of the transient detection, affects required memory */

#define NO_TCX                    0
#define TCX_20                    1
#define TCX_10                    2
#define TCX_5                     3

#define TRANSITION_OVERLAP      (-2)
#define RECTANGULAR_OVERLAP     (-1)
#define FULL_OVERLAP              0
#define NOT_SUPPORTED             1
#define MIN_OVERLAP               2
#define HALF_OVERLAP              3

/*----------------------------------------------------------------------------------*
 * FEC constants
 *----------------------------------------------------------------------------------*/

#define UNVOICED_CLAS                         0         /* Unvoiced, silence, noise, voiced offset   */
#define UNVOICED_TRANSITION                   1         /* Transition from unvoiced to voiced components - possible onset, but too small */
#define VOICED_TRANSITION                     2         /* Transition from voiced - still voiced, but with very weak voiced characteristics     */
#define VOICED_CLAS                           3         /* Voiced frame, previous frame was also voiced or ONSET                   */
#define ONSET                                 4         /* Voiced onset sufficiently well built to follow with a voiced concealment    */
#define SIN_ONSET                             5         /* Artificial harmonic+noise onset (used only in decoder)                       */
#define INACTIVE_CLAS                         6         /* Inactive frame (used only in decoder)                  */
#define AUDIO_CLAS                            7         /* Audio frame (used only in AMR-WB IO mode) */

#define BETA_FEC_FX                           24576     /* FEC - weighting factor for LSF estimation in FER */
#define STAB_FAC_LIMIT_FX                     8192      /* FEC - limit at which safety net is forced for next frame */

#define MODE1_L_FIR_FER                      5         /* FEC - impulse response length for low- and high-pass filters in FEC */
#define L_FIR_FER                             3         /* FEC - impulse response length for low- & high-pass filters in FER concealment*/
#define L_FIR_FER2                            11        /* FEC - new filter tuning: 11*/
#define MAX_UPD_CNT                           5         /* FEC - maximum number of frames since last pitch update */

/* attenuation strategy in case of FER */
#define _ALPHA_S_FX                           19661
#define _ALPHA_V_FX                           32767
#define _ALPHA_VT_FX                          13107
#define _ALPHA_UT_FX                          26214
#define _ALPHA_U_FX                           13107
#define _ALPHA_U_FX_X_2                       26214
#define _ALPHA_UU_FX                          32767

#define PLC_MIN_CNG_LEV                       FL2WORD16(0.01f)
#define PLC_MIN_STAT_BUFF_SIZE                50  /* buffer size for minimum statistics */
#define G_LPC_RECOVERY_BITS                   1

/* constants used for concealment recovery strategy: energy control */
#define AGC_UP                    32113 /*0.98f Q15*/
#define AGC_DOWN                  32440 /*0.99f Q15*/

/*----------------------------------------------------------------------------------*
 * Transition mode (TC) constants
 *----------------------------------------------------------------------------------*/
/* Conversion of tc_subfr to index */
#define TC_SUBFR2IDX_16KHZ_fx(x)              mac_r(1024L, (x), 512) /* -1 => 0, 0 => 0, 64 => 1, 128 => 2, 192 => 3, 256 => 4 */

#define TC_SUBFR2IDX_fx(x)                    add(s_min(3, s_max(0, sub((x), 1))), TC_SUBFR2IDX_16KHZ_fx(x))

#define L_IMPULSE                             17        /* TC - length of one prototype impulse                                                          */
#define L_IMPULSE2                            8         /* TC - half-length of one prototype impulse == floor(L_IMPULSE/2)                               */
#define NUM_IMPULSE                           8         /* TC - number of prototype impulses                                                             */
#define N_GAIN_CODE_TC                        8         /* TC - number of levels for gain_code quantization for subrames without glot. impulse(s) -      */
#define N_GAIN_TC                             8         /* TC - number of levels for gain_trans quantization                                             */
/* TC - attention: DO NOT CHANGE the following constants - needed for correct bit-allocations        */
#define TC_0_0                                1         /* TC - subframe ID for TC: first glottal impulse in the 1st subframe, second in the 1st subframe    */
#define TC_0_64                               2         /* TC - subframe ID for TC: first glottal impulse in the 1st subframe, second in the 2nd subframe    */
#define TC_0_128                              3         /* TC - subframe ID for TC: first glottal impulse in the 1st subframe, second in the 3rd subframe    */
#define TC_0_192                              4         /* TC - subframe ID for TC: first glottal impulse in the 1st subframe, second in the 4th subframe    */

/*----------------------------------------------------------------------------------*
 * AVQ constants
 *----------------------------------------------------------------------------------*/

#define NB_LDQ3                               9         /* RE8 constants */
#define NB_LDQ4                               27
#define NB_SPHERE                             32
#define NB_LEADER                             36

#define NSV_MAX                               34        /* maximal number of sub-vectors used by the AVQ */

/*----------------------------------------------------------------------------------*
 * Arithmetic coder
 *----------------------------------------------------------------------------------*/
#define A_THRES_SHIFT                         2
#define A_THRES                               (1<<A_THRES_SHIFT)
#define VAL_ESC                               16
#define NBITS_CONTEXT                         8
#define NBITS_RATEQ                           2

#define cbitsnew                              16
#define stat_bitsnew                          14

#define ari_q4new  (((long)1<<cbitsnew)-1)
#define ari_q1new  (ari_q4new/4+1)
#define ari_q2new  (2*ari_q1new)
#define ari_q3new  (3*ari_q1new)
#define ari_q3newM1  (3*ari_q1new-1)

/* ari_hm.h */
#define kLtpHmFractionalResolution  7
#define kLtpHmFlag                  (1 << 8)

/* Max. number of coefficients (do not change) */
#define MAX_LENGTH         L_FRAME_MAX

/*----------------------------------------------------------------------------------*
 * TCQ constants
 *----------------------------------------------------------------------------------*/

#define MAX_PULSES                            560
#define MAX_POS                               320
#define MAX_LEN                               MAX_PULSES
#define NUM_ENG_PACKED_WORDS                  20        /* Storage for variable rate quantizer bits */

#define NORMAL_HQ_CORE                        0         /* Signal use of Normal HQ core */
#define LOW_RATE_HQ_CORE                      1         /* Signal use of Low Rate MDCT core */
#define LOW_RATE_HQ_CORE_TRAN                 2         /* Signal use of Low Rate MDCT core Tran SWB */
#define NORM_MDCT_FACTOR                      L_FRAME8k /* Normalize Low Rate MDCT coefficients to this frame size */
#define BANDS_MAX                             (4*14)
#define MAX_GQLEVS                            32        /* Max fine gain levels */
#define BITS_DE_CMODE                         1
#define BITS_DE_HMODE                         1
#define BITS_DE_8SMODE                        1
#define MAXIMUM_ENERGY_LOWBRATE               255
#define MINIMUM_ENERGY_LOWBRATE               -256
#define BITS_ABS_ENG                          7
#define ABS_ENG_OFFSET                        64
#define BITS_MAX_DEPTH                        3
#define BITS_DE_8SMODE_N0                     1
#define BITS_DE_8SMODE_N1                     1
#define BITS_DE_8SPOS                         5
#define BITS_DE_FCOMP                         5
#define BITS_DE_LSB                           1
#define DE_OFFSET0                            46
#define DE_OFFSET1                            32
#define DE_LIMIT                              64
#define LRMDCT_BE_OFFSET                      15
#define LRMDCT_BE_LIMIT                       31

#define HQCORE_NB_MIN_RATE                    7200     /* NB LR MDCT coding down to this bit rate */
#define HQCORE_WB_MIN_RATE                    13200    /* WB LR MDCT coding down to this bit rate */
#define HQCORE_SWB_MIN_RATE                   13200    /* SWB LR MDCT coding down to this bit rate */

#define LRMDCT_CROSSOVER_POINT                16400    /* Use LR MDCT core at this rate and below */

#define FWARP1                                8673     /* Warping constant #1 used to convert 16 to 14 kHz bandwidth for LR MDCT core ( ~=14kHz / 16kHz) */
#define FWARP2                                0.8650f  /* Warping constant #2 used to convert 16 to 14 kHz bandwidth for LR MDCT core ( ~=14kHz / 16kHz) */

#define SCALE1(x)                             ((short)((x*FWARP1+5000)/10000))
#define SCALE2(x)                             ((short)(x*FWARP2+0.5f))

#define NOOFBANDSIZES                         4
#define NUMBEROFBITSFORCOMBINING              11
#define COMBINING_PRECISION                   18
#define MAX_DATA_RATE                         128000
#define MAX_FPC_WORDS                         ((MAX_DATA_RATE/50)/16 + BANDS_MAX)

#define HTH_NORM                              17
#define LTH_NORM                              13
#define OFFSET_NORM                            3

/*----------------------------------------------------------------------------------*
 * SWB TBE constants
 *----------------------------------------------------------------------------------*/

#define STEPSNUM                              4                    /* Number of steps in a2lsp routine for SHB LPC */
#define ALLPASSSECTIONS_STEEP                 3                    /* Size of all pass filters for interpolation and decimation by a factor of 2 */
#define INTERP_3_1_MEM_LEN                    13
#define INTERP_3_2_MEM_LEN                    15
#define L_SHB_LAHEAD                          20                   /* Size of lookahead for SHB */
#define NUM_SHB_SUBFR                         16
#define LPC_SHB_ORDER                         10
#define LPC_WHTN_ORDER                        4                    /* Order of whitening filter for SHB excitation */
#define SHB_OVERLAP_LEN                       (L_FRAME16k-L_SHB_LAHEAD)/(NUM_SHB_SUBFR-1)
#define QUANT_DIST_INIT                       (10000000000)        /* Quantiser search distance initialisation */
#define HIBND_ACB_L_FAC                       5/2                  /* SHB Interpolation Factor */
#define NUM_HILBERTS                          2
#define HILBERT_ORDER1                        5
#define HILBERT_ORDER2                        4
#define HILBERT_MEM_SIZE                      (HILBERT_ORDER1 + (2*HILBERT_ORDER2) + (2*HILBERT_ORDER2))

#define ACELP_LOOK_12k8                       NS2SA(INT_FS_FX, ACELP_LOOK_NS)  /* SHB Low delay look-ahead at 12.8kHz */
#define ACELP_LOOK_16k                        NS2SA(INT_FS_16k, ACELP_LOOK_NS) /* SHB Low delay look-ahead at 16kHz */

#define NUM_BITS_SHB_SubGain                  6
#define NUM_BITS_SHB_FrameGain                6

#define NUM_BITS_SHB_FrameGain_LBR_WB         4
#define RECIP_ROOT_EIGHT_FX                   11585         /* 1.0 / sqrt(8.0) - constant Gain Shape over TD BWE subframes */

#define LPC_SHB_ORDER_WB                      6
#define LPC_WHTN_ORDER_WB                     2                    /* Order of whitening filter for WB excitation */
#define NUM_BITS_WB_LSF                       8
#define LPC_SHB_ORDER_LBR_WB                  4
#define NUM_BITS_LBR_WB_LSF                   2

#define COMP_FIL_ORDER                        19
#define MAX_BIQ_N                             L_FRAME32k

#define NUM_SHB_SUBGAINS                      4                    /* Number of subframe gains */
#define NUM_BITS_SHB_SUBGAINS                 5                    /* Number of bits for subframe gains for SWB */
#define NUM_BITS_SHB_FRAMEGAIN                5                    /* Number of bits for framegain for SWB */
#define NUM_BITS_SHB_ENER_SF                  6
#define NUM_BITS_SHB_RES_GS                   3
#define NUM_BITS_SHB_VF                       3
#define NUM_BITS_SHB_SUBGAINS_RF              5                    /* Number of bits for subframe gains for SWB in RF */
#define SHB_GAIN_QLOW_FX                      -262144              /* Q18*/   /* SHB gain lowest scalar quantizer value */
#define SHB_GAIN_QLOW_FX_16                   -65536               /* SHB gain lowest scalar quantizer value */
#define SHB_GAIN_QDELTA_FX_15                 4915                 /* SHB gain scalar quantizer step size */
#define SHB_GAIN_QDELTA_FX_16                 9830
#define SHB_GAIN_QDELTA_FX                    19661                /* 0.15 in Q17*/
#define NUM_Q_LSF                             5                    /* Number of quantized LSFs */
#define MIRROR_POINT_BITS                     2                    /* Number of bits used to quantize mirror point */
#define MIRROR_POINT_Q_CB_SIZE                4                    /* Size of codebook used to quantize mirror point */
#define MAX_LSF_FX_2                          8192                 /* Maximum value of the LSFs */
#define MAX_LSF_FX                            16384                /* Maximum value of the LSFs */
#define MAX_LSF_FX_BY_2                       8192                 /* Maximum value of the LSFs */
#define NUM_MAP_LSF                           5                    /* Number of mapped LSFs */
#define NUM_LSF_GRIDS                         4                    /* Number of LSF grids */
#define NUM_LSF_GRID_BITS                     2                    /* Number of bits used for the LSF grids */

#define VF_0th_PARAM_FX                       11141     /*.34*/
#define VF_1st_PARAM_FX                       16384     /*.5*/
#define VF_2nd_PARAM_FX                       (VF_1st_PARAM_FX - VF_0th_PARAM_FX)

#define GAMMA0_FX                             21299                /* Mean value of gamma1/gamma2 for formant PF   */
#define GAMMA_SHARP_FX                        4915               /* Largest sharpening for gamma1/gamma2 (0.83/0.67)*/
#define SWB_NOISE_MIX_FAC_FX                  4915             /* 0.15f in Q15 */

#define SWB_TILT_LOW_FX                       4096          /* Q12 1.0f Lower threshold for PF tilt adaptation */
#define SWB_TILT_HIGH_FX                      8192          /* Q12 2.0f Higher threshold for PF tilt adaptation */
#define SWB_TILT_DELTA_FX                     32767         /* Q15 (1.0f/(SWB_TILT_HIGH-SWB_TILT_LOW)) Inclination between thresholds */

#define HALF_POINT_FX                         (16384)
#define GAMMA3_PLUS_FX                        6554          /* NB post-filter - tilt weighting factor when k1>0 */
#define GAMMA3_MINUS_FX                       29491         /* NB post-filter - tilt weighting factor when k1<0 */
#define GAMMA3_PLUS_WB_FX                     21299         /* WB post-filter  */
#define GAMMA3_MINUS_WB_FX                    27853         /* WB post-filter  */
#define AGC_FAC_WB_FX                         27853                /* WB post-filter - gain adjustment factor */
#define AGC_FAC1_WB_FX                        (Word16)(32768L-AGC_FAC_WB_FX)    /* WB post-filter - gain adjustment factor complement */

/* SWB TBE, FX only constants */
#define NOISE_QFAC      6
#define NOISE_QADJ      (15-NOISE_QFAC)

/*----------------------------------------------------------------------------------*
 * SWB BWE constants
 *----------------------------------------------------------------------------------*/

#define INV_L_SUBFR16k_FX                     410  /*Q15 */
#define SWB_L_SUBFR                           160
#define FB_L_SUBFR                            240
#define SWB_FENV                              14
#define WB_FENV                               2
#define FB_GAIN_QLOW_FX                       0
#define FB_GAIN_QDELTA_FX                     512  /*Q14 */

#define NUM_BITS_FB_FRAMEGAIN                 4                 /* Number of bits for framegain for FB */
#define FB_BAND_BEGIN                         620
#define FB_BAND_END                           800
#define FB_BAND_WIDTH                         180
#define N_CAND                                2
#define N_CB11                                32   /* 5bits */
#define N_CB1ST                               128  /* 7bits */
#define N_CB2ND                               64   /* 6bits */
#define N_CB3RD                               32   /* 5bits */
#define N_CB4TH                               64   /* 6bits */
#define DIM1ST                                3
#define DIM2ND                                4
#define DIM3RD                                3
#define DIM4TH                                4
#define DIM11                                 (DIM1ST+DIM2ND)
#define DIM12                                 (DIM3RD+DIM4TH)
#define N_CAND_TR                             3
#define N_CB_TR1                              128
#define N_CB_TR2                              64
#define DIM_TR1                               2
#define DIM_TR2                               2
#define SWB_FENV_TRANS                        4
#define SWB_TENV                              4
#define NUM_SHARP                             9
#define SHARP_WIDTH                           32

#define HARMONIC                              3
#define NORMAL                                2
#define TRANSIENT                             1
#define NOISE                                 0

/*----------------------------------------------------------------------------------*
 * HR SWB BWE constants
 *----------------------------------------------------------------------------------*/

#define NSV_OVERLAP                           2             /* number of sub-bands overlaping with lower-band (0-8kHz) */   /* note that NSV_MAX >= END_FREQ_BWE_FULL/(8*50) + NSV_OVERLAP ! */
#define N_BANDS_BWE_HR                        4             /* number of frequency bands in non-transient frame */
#define N_BANDS_TRANS_BWE_HR                  2             /* number of frequency bands in transient frame */
#define END_FREQ_BWE                          14400         /* maximum frequency coded by AVQ */
#define END_FREQ_BWE_FULL                     16000         /* maximum frequency coded by HR SWB BWE */
#define END_FREQ_BWE_FULL_FB                  20000         /* maximum frequency coded by HR FB BWE */

#define NBITS_GLOB_GAIN_BWE_HR                5             /* number of bits of the global gain quantizer */
#define MIN_GLOB_GAIN_BWE_HR                  3             /* minimum value of the global gain quantizer */
#define MAX_GLOB_GAIN_BWE_HR                  500           /* maximum value of the global gain quantizer */

#define NBITS_ENVELOPE_BWE_HR1                6             /* number of bits for envelope VQ - first two subbands in non-transient frame */
#define NBITS_ENVELOPE_BWE_HR2                5             /* number of bits for envelope VQ - second two subbands in non-transient frame */
#define NBITS_ENVELOPE_BWE_HR_TR              4             /* number of bits for envelope VQ - two subbands in transient frame */
#define NUM_ENVLOPE_CODE_HR1                  64            /* dimension of envelope VQ - first two subbands in non-transient frame */
#define NUM_ENVLOPE_CODE_HR2                  32            /* dimension of envelope VQ - second two subbands in non-transient frame */
#define NUM_ENVLOPE_CODE_HR_TR                16            /* dimension of envelope VQ - two subbands in transient frame */
#define NUM_ENVLOPE_CODE_HR_TR2               8             /* dimension of envelope VQ - two subbands in transient frame */

#define NUM_NONTRANS_START_FREQ_COEF          (L_FRAME32k/2 - NSV_OVERLAP*WIDTH_BAND)                   /* start frequency coefficient (==7.6kHz) in non-transient frame */
#define NUM_NONTRANS_END_FREQ_COEF            (L_FRAME32k*END_FREQ_BWE/END_FREQ_BWE_FULL)               /* end frequency coefficient (==14.4kHz) in non-transient frame */
#define NUM_TRANS_START_FREQ_COEF             (NUM_NONTRANS_START_FREQ_COEF/NUM_TIME_SWITCHING_BLOCKS)  /* start frequency coefficient (==7.6kHz) in transient frame */
#define NUM_TRANS_END_FREQ_COEF               (NUM_NONTRANS_END_FREQ_COEF/NUM_TIME_SWITCHING_BLOCKS)    /* end frequency coefficient (==14.4kHz) in transient frame */
#define NUM_TRANS_END_FREQ_COEF_EFF           140
#define WIDTH_NONTRANS_FREQ_COEF              ((NUM_NONTRANS_END_FREQ_COEF - NUM_NONTRANS_START_FREQ_COEF)/N_BANDS_BWE_HR)  /* number of coefficients per band in non-transient frame */
#define WIDTH_TRANS_FREQ_COEF                 ((NUM_TRANS_END_FREQ_COEF - NUM_TRANS_START_FREQ_COEF)/N_BANDS_TRANS_BWE_HR)  /* number of coefficients per band in transient frame */

#define NBITS_THRESH_BWE_HR                   400           /* BWE HR number of bits threshold */

#define NBITS_HF_GAIN_BWE_HR                  2             /* number of bits for HF (noncoded) energy estimation */
#define BWE_HR_TRANS_EN_LIMIT1_FX_Q16         6554
#define BWE_HR_TRANS_EN_LIMIT2_FX_Q16         19661
#define BWE_HR_TRANS_EN_LIMIT3_FX_Q16         32767
#define BWE_HR_NONTRANS_EN_LIMIT1_FX_Q15      16384 /* 0.5 */
#define BWE_HR_NONTRANS_EN_LIMIT2_FX_Q14      19661 /* 1.2 */
#define BWE_HR_NONTRANS_EN_LIMIT2_FX_Q15      16384
#define BWE_HR_NONTRANS_EN_LIMIT3_FX_Q15      26214 /* 0.8 */

/*----------------------------------------------------------------------------------*
 * FD CNG
 *----------------------------------------------------------------------------------*/
#define DELTA                                 1e-20f
#define DELTA_MANTISSA_W16                    0x5e73
#define DELTA_MANTISSA_W32                    0x5e728433
#define DELTA_EXPONENT                        (-66)

#define CLDFB_SCALING                         FL2WORD16_SCALE(1.5, 1) /* Q 2.14 */

#define FFTLEN                                640
#define FFTLEN2                               (FFTLEN/2)
#define CORECLDFBLEN                          20
#define TOTCLDFBLEN                           40
#define FFTCLDFBLEN                           (FFTLEN2+TOTCLDFBLEN-CORECLDFBLEN)
#define NPART                                 24
#define NPARTCLDFB                            10
#define NPART_SHAPING                         66

#define MSSUBFRLEN                            12
#define MSNUMSUBFR                            6
#define MSBUFLEN                              5

#define NOISE_HEADROOM                        5    /* headroom of noise in generate_masking_noise */

#define MSALPHACORALPHA                       FL2WORD16(0.7f)
#define MSALPHACORALPHA2                      FL2WORD16(0.3f)
#define MSALPHACORMAX                         FL2WORD16(0.3f)
#define MSALPHAMAX                            FL2WORD16(0.96f)
#define MSALPHAHATMIN                         FL2WORD32(0.05f)
#define MSQEQINVMAX                           FL2WORD16(1.f/5.f)
#define MSAV                                  FL2WORD16_SCALE(2.12f, 2)
#define MSAV_EXP                              2
#define MSBETAMAX                             FL2WORD32(0.8f)
#define MSBETAMAX_SQRT                        FL2WORD32(0.894427191) /* sqrt(MSBETAMAX) */
#define MSSNREXP                              FL2WORD16(-0.02f/0.064f)

#define NB_LAST_BAND_SCALE                    FL2WORD16(0.8f)
#define SWB_13k2_LAST_BAND_SCALE              FL2WORD16(0.8f)

#define M_MAX                                 32
#define NSTAGES_MAX                           9
#define MBEST_MAX                             8
#define N_GAIN_MIN                            4
#define N_GAIN_MAX                            17

#define numSlots_inv_EXP                      (-3)
#define PREEMPH_COMPENSATION_EXP              4

#define CLDFBscalingFactor_EXP    (-15)
#define CLDFBinvScalingFactor_EXP ( 16)

#define CNG_NORM_RECIPROCAL_RANGE_SHIFT  2
#define CNG_RAND_GAUSS_SHIFT             2

#define CNA_MAX_BRATE                         13200

/*----------------------------------------------------------------------------------*
 * Bass post-filter constants
 *----------------------------------------------------------------------------------*/

#define NBPSF_PIT_MAX                         (PIT16k_MAX+1)  /* maximum pitch value for bass post-filter */
#define PST_L_FILT12k8                        12
#define PST_L_FILT                            15
#define L_TRACK_HIST                          10

/*----------------------------------------------------------------------------------*
 * NB post-filter constants
 *----------------------------------------------------------------------------------*/

#define LONG_H_ST                             20        /* NB post-filter - impulse response length */
#define GAMMA1_PST12K_FX                      24576      /* denominator weighting factor 12K (0.75 in Q15) */
#define GAMMA2_PST12K_FX                      22938      /*  numerator  weighting factor 12K (0.7  in Q15) */
#define POST_G1_FX                            GAMMA1_PST12K_FX /* 12 kbps default */
#define POST_G2_FX                            GAMMA2_PST12K_FX /* 12 kbps default */
#define GAMMA1_PST12K_MIN_FX                  21299  /* 0.65 in Q15 */
#define GAMMA2_PST12K_MIN_FX                  18022  /* 0.55 in Q15 */
#define GAMMA1_PST12K_NOIS_FX                 4915  /* 0.15 in Q15 */
#define GAMMA2_PST12K_NOIS_FX                 3277  /* 0.10 in Q15 */
#define F_UP_PST                              8         /* NB post-filter - resolution for fractionnal delay */
#define LH2_S                                 4         /* NB post-filter - length of INT16 interp. subfilters */
#define LH2_L                                 16        /* NB post-filter - length of long interp. subfilters  */
#define LH_UP_S                               (LH2_S/2)
#define LH_UP_L                               (LH2_L/2)
#define LH2_L_P1                              (LH2_L + 1)
#define DECMEM_RES2                           (PIT16k_MAX + 2 + LH_UP_L)
#define SIZ_RES2                              (DECMEM_RES2 + L_SUBFR)
#define SIZ_Y_UP                              ((F_UP_PST-1) * (L_SUBFR+1))
#define SIZ_TAB_HUP_L                         ((F_UP_PST-1) * LH2_L)
#define SIZ_TAB_HUP_S                         ((F_UP_PST-1) * LH2_S)
#define BG1_FX                               -328  /* -0.01 in Q15 */
#define BG2_FX                               -1638  /* -0.05 in Q15 */
#define CG1_FX                                29491  /*  0.9  in Q15 */
#define CG2_FX                                47514L /*  1.45 in Q15 */
#define C_LP_NOISE_FX                         819  /* 0.1/4.0 in Q15 */
#define CK_LP_NOISE_FX                        6291456L /* 15.0 * 0.1/4 in Q8 * 65536 */
#define LP_NOISE_THR_FX                       6400  /* 25.0 in Q8 */

/*----------------------------------------------------------------------------------*
 * Stability estimation
 *----------------------------------------------------------------------------------*/

#define NB_BFI_THR                            2         /* threshold for counter of last bad frames */
#define MAX_LT                                40

#define TH_0_MIN_FX                           5120      /* 2.5f in Q11 */
#define TH_1_MIN_FX                           3840      /* 1.875f in Q11 */
#define TH_2_MIN_FX                           3200      /* 1.5625f in Q11 */
#define TH_3_MIN_FX                           2688      /* 1.3125f in Q11 */

/*----------------------------------------------------------------------------------*
 * Speech/music classifier constants
 *----------------------------------------------------------------------------------*/

#define N_FEATURES                            12       /* number of features */
#define N_MIXTURES                            6        /* number of mixtures */
#define M_LSP_SPMUS                           6        /* number of LSPs used in speech/music classifier */
#define NB_BANDS_SPMUS                        15
#define START_BAND_SPMUS                      2
#define N_OLD_BIN_E                           42

#define LOWEST_FBIN                           3        /* lowest frequency bin for feature vector preparation */
#define HIGHEST_FBIN                          70       /* highest frequency bin for feature vector preparation */
#define HANG_LEN_INIT                         8        /* number of frames for hang-over (causes delay of decision) */
#define HANG_LEN                              8
#define BUF_LEN                               60
#define L_OVR                                 8

#define N_FEATURES_2                          3       /* number of features */

/*----------------------------------------------------------------------------------*
 * LD music post-filter constants
 *----------------------------------------------------------------------------------*/

#define DCT_L_POST                            640
#define OFFSET2                               192

#define VOIC_BINS_HR                          640
#define BIN_16kdct                            (6400/DCT_L_POST)
#define NB_LIMIT_BAND                         16
#define MBANDS_GN_LD                          20       /* number of bands for gain coding in the postfilter */

/*----------------------------------------------------------------------------------*
 * AC mode (GSC) constants
 *----------------------------------------------------------------------------------*/

#define NOISE_LEVEL_SP0                       8
#define NOISE_LEVEL_SP1a                      9
#define NOISE_LEVEL_SP1                       10
#define NOISE_LEVEL_SP2a                      11
#define NOISE_LEVEL_SP2                       12
#define NOISE_LEVEL_SP3a                      13
#define NOISE_LEVEL_SP3                       14

#define MAX_DYNAMIC                           82
#define MIN_DYNAMIC                           50
#define DYNAMIC_RANGE                         (MAX_DYNAMIC-MIN_DYNAMIC)
#define MAX_GSC_NF_BITS                       3
#define GSC_NF_STEPS                          (1 << MAX_GSC_NF_BITS)

#define CRIT_NOIS_BAND                        23

#define SSF                                   32        /* Sub-subframe length for energy estimation in UC decision */
#define NB_SSF                                (L_FRAME / SSF) /* number of sub-subframes per frame */

#define NB_MAX_PULSES                         12

#define MBANDS_GN                             16       /* Number of band for gain coding in GSC */
#define BAND1k2                               3

#define MBANDS_LOC                            (MBANDS_GN-1)
#define SWNB_SUBFR                            1

#define VAR_COR_LEN                           10

#define MIN_RATE_4SBFR                        ACELP_16k40
#define CFREQ_BITRATE                         ACELP_11k60

#define LT_UV_THR                             100
#define LT_UV_THRMID                          70

#define PIT_EXC_L_SUBFR                       L_FRAME
#define LOCAL_CT                              VOICED

/*----------------------------------------------------------------------------------*
 * Core Switching constants
 *----------------------------------------------------------------------------------*/
#define SWITCH_MAX_GAP                        360  /*  6.25 + 1.25 of filter mem max */
#define MDCT_WINDOW_ASYM                      0 /* Asymmetric window flag : 0 LD-sine MDCT window/1 LD-asymmetric MDCT window*/

/*----------------------------------------------------------------------------------*
 * HQ core constants
 *----------------------------------------------------------------------------------*/

#define HQ_NORMAL                             0
#define HQ_TRANSIENT                          1
#define HQ_HARMONIC                           2
#define HQ_HVQ                                3
#define HQ_GEN_SWB                            4
#define HQ_GEN_FB                             5

#define PREECHO_SMOOTH_LEN                    20
#define INV_PREECHO_SMOOTH_LENP1              (1 / (PREECHO_SMOOTH_LEN + 1.0));

#define MAX16B                                32767
#define MIN16B                                (-32768)

#define ALDO_WINDOW 4
#define WINDECAY48 1230
#define WINDECAY48_256 656
#define WINDECAY16 410
#define N16_CORE_SW 90
#define N_ZERO_8 45

#define SWITCH_OVERLAP_8k   15
#define SWITCH_GAP_LENGTH_8k 50

#define MAX_SEGMENT_LENGTH                    480
#define NUM_TIME_SWITCHING_BLOCKS             4
#define NUM_MAP_BANDS                         20
#define NUM_MAP_BANDS_HQ_24k4                 17
#define NUM_MAP_BANDS_HQ_32k                  18
#define FREQ_LENGTH                           800

#define STOP_BAND                             800

#define SFM_G1                                16
#define SFM_G1G2                              24
#define SFM_N_NB                              18
#define SFM_N_WB                              26
#define SFM_N_STA_8k                          27
#define SFM_N_STA_10k                         30
#define SFM_N_ENV_STAB                        SFM_N_STA_8k  /* Number of bands for env_stab stability measure */
#define SFM_N_ENV_STAB_WB                     SFM_N_WB      /* Number of bands for env_stab stability measure used in HQPLC decision for WB signals */
#define SFM_N_HARMONIC                        39
#define SFM_N                                 36
#define N_INTL_GRP_16                         2             /* Number of interleaving band groups at 16kHz samplerate */
#define N_INTL_GRP_32                         2             /* Number of interleaving band groups at 32kHz samplerate */
#define N_INTL_GRP_48                         3             /* Number of interleaving band groups at 48kHz samplerate */
#define SFM_N_SWB                             39
#define SFM_N_HARM                            31
#define SFM_N_HARM_FB                         33
#define NB_SFM                                44
#define NB_SFM_MAX                            58
#define WID_G1                                8
#define WID_G2                                16
#define WID_G3                                24
#define WID_GX                                32
#define NUMC_N                                544

#define QBIT_MAX2                             9

#define FLAGN_BITS                            1
#define GAIN0_BITS                            5
#define GAINI_BITS                            5

#define FLAGS_BITS                            2
#define FLAGS_BITS_FB                         3
#define NORM0_BITS                            5
#define NORMI_BITS                            5
#define NUMNRMIBITS_SWB_STA_8k                5*(SFM_N_STA_8k-1)
#define NUMNRMIBITS_SWB_STA_10k               5*(SFM_N_STA_10k-1)
#define NUMNRMIBITS_SWB_HARMONIC              185
#define NUMNRMIBITS_SWB                       190
#define NUMNRMIBITS                           215
#define NUMNRMIBITS_WB                        125

#define NOHUFCODE                             0
#define HUFCODE                               1
#define HUFF_THR                              10
#define NOSUPERPOSITION                       40

#define MAXVALUEOFFIRSTGAIN_FX                20480 /*2.5f in Q13 */
#define MINVALUEOFFIRSTGAIN_FX               -20480
#define NOOFGAINBITS1                         6

#define AUDIODELAYBITS                        6
#define DELTAOFFIRSTGAIN_FX                   2601 /*Q15 */

#define MAX_P_ATT                             40   /* Maximum number of pulses for gain attenuation factor */
#define NB_G                                  4    /* Number of band groups */
#define MAX_GAIN_BITS                         5    /* Maximum number of gain bits */

#define ENV_ADJ_START                         6    /* Number of consecutive bands for which the attenuation is maximum */
#define ENV_ADJ_INCL                          5    /* Inclination for mapping between attenuation region width and attenuation limit */

#define L_STAB_TBL                            10    /* Number of elements in stability transition table */
#define LUMPED_ENV_SMOOTH_FAC_FX              ((Word16)10089) /* Q19 (no typo error), 0.1/sqrt(27) */
#define CMPLMNT_ENV_SMOOTH_FAC_FX             ((Word16)29491) /* Q15 0.9 */
#define M_STAB_TBL_FX                         ((Word16)21068) /* Q13, 2.571756 */
#define HALF_D_STAB_TBL_FX                    ((Word16)  422) /* Q13 0.1013138/2.0 */
#define D_STAB_TBL_FX                         ((Word16)  845) /* Q13 0.1013138 */
#define NUM_ENV_STAB_PLC_STATES                    2         /* Number of states of markov model */
#define INV_NUM_ENV_STAB_PLC_STATES                16384     /* Q15 */
#define INV_STAB_TRANS_FX                          16497     /* Q14. Equal to 1.0f/(1-2*stab_trans_fx[L_STAB_TBL-1]) */

#define ATT_LIM_HANGOVER                      150       /* Number of hangover frames for disabling stability dependent attenuation */

#define START_EXC                             60
#define L_HARMONIC_EXC                        202

#define HQ_GENERIC_OFFSET                     2
#define HQ_GENERIC_END_FREQ                   560

#define HQ_GENERIC_FOFFSET_24K4               80
#define HQ_GENERIC_FOFFSET_32K                144
#define HQ_GENERIC_SWB_NBITS                  31
#define HQ_GENERIC_SWB_NBITS2                 30
#define HQ_GENERIC_FB_NBITS                   5

#define HQ_GENERIC_ST_FREQ                    224
#define HQ_GENERIC_LOW0                       80
#define HQ_GENERIC_HIGH0                      240
#define HQ_GENERIC_HIGH1                      368
#define HQ_GENERIC_HIGH2                      496
#define HQ_GENERIC_LEN0                       128
#define HQ_GENERIC_NVQIDX                     6

#define HQ_GENERIC_EXC0                       0
#define HQ_GENERIC_EXC1                       1
#define HQ_GENERIC_SP_EXC                     2


#define DIM_FB                                3
#define HQ_FB_FENV                            SWB_FENV + DIM_FB
#define N_CB_FB                               32

#define HVQ_THRES_BIN_24k                     224
#define HVQ_THRES_SFM_24k                     22
#define HVQ_THRES_BIN_32k                     320
#define HVQ_THRES_SFM_32k                     25
#define HVQ_MIN_PEAKS                         2
#define HVQ_MAX_PEAKS_32k                     23
#define HVQ_MAX_PEAKS_24k                     17
#define HVQ_MAX_PEAKS_24k_CLAS                20    /* Limit for HVQ mode */
#define HVQ_MAX_PEAKS                         HVQ_MAX_PEAKS_32k + 1
#define HVQ_NUM_SFM_24k                       (SFM_N_HARMONIC - 1 - HVQ_THRES_SFM_24k)
#define HVQ_NUM_SFM_32k                       (SFM_N_HARMONIC - 1 - HVQ_THRES_SFM_32k)

#define HVQ_MAX_RATE                          32000



#define NUMNRMIBITS_SWB_HVQ_24k               35
#define NUMNRMIBITS_SWB_HVQ_32k               25

#define MAX_PVQ_BANDS                         8
#define HVQ_MAX_PVQ_WORDS                     ((HVQ_MAX_RATE/50)/16 + MAX_PVQ_BANDS)
#define HVQ_MAX_POS_WORDS                     40
#define HVQ_PVQ_COEFS                         24
#define HVQ_BAND_MIN_PULSES                   2
#define HVQ_BAND_MAX_BITS_24k                 80
#define HVQ_BAND_MAX_BITS_32k                 95
#define HVQ_BAND_MAX_PULSES_24k               42
#define HVQ_BAND_MAX_PULSES_32k               68
#define HVQ_NEW_BAND_BIT_THR                  30

#define HVQ_NF_GROUPS                         2
#define HVQ_NF_WEIGHT1_FX                     31385   /* Q15 0.9578  - HVQ Classifier - Noise floor estimate weight 1     */
#define HVQ_NF_WEIGHT1B                       1383    /* Q15, 1 - HVQ_NF_WEIGHT1_FX */
#define HVQ_NF_WEIGHT2_FX                     21207   /* Q15 0.6472  - HVQ Classifier - Noise floor estimate weight 2     */
#define HVQ_NF_WEIGHT2B                       11561   /* Q15 1 - HVQ_NF_WEIGHT2_FX */
#define HVQ_PE_WEIGHT1_FX                     13840   /* Q15 0.42237 - HVQ Classifier - Peak envelope estimate weight 1   */
#define HVQ_PE_WEIGHT1B                       18928   /* Q15, 1 - HVQ_PE_WEIGHT1_FX */
#define HVQ_PE_WEIGHT2_FX                     26308   /* Q15 0.80285 - HVQ Classifier - Peak envelope estimate weight 2   */
#define HVQ_PE_WEIGHT2B                       6460    /* Q15, 1 - HVQ_PE_WEIGHT2_FX */
#define HVQ_SHARP_THRES                       9             /* HVQ Classifier - Sharpness threshold */
#define HVQ_SHARP_THRES_FX                    576     /*9 in Q6 */

#define HVQ_PA_FAC_FX                         23170   /* Q15 0.7071 - HVQ Classifier peak allocation factor */
#define HVQ_PA_PEAKS_SHARP1                   9             /* HVQ Classifier - Maximum number of peaks for band with high sharpness */
#define HVQ_PA_PEAKS_SHARP2                   3             /* HVQ Classifier - Maximum number of peaks for band with medium sharpness */
#define HVQ_PA_PEAKS_SHARP3                   2             /* HVQ Classifier - Maximum number of peaks for band with low sharpness */
#define HVQ_PA_SHARP_THRES2_FX                1024    /* Q6  16.0   - HVQ Classifier - Sharpness threshold for band with medium sharpness */
#define HVQ_PA_SHARP_THRES3_FX                768     /* Q6  12.0   - HVQ Classifier - Sharpness threshold for band with low sharpness */

#define HVQ_BW                                32            /* HVQ Classifier subband bandwidth */
#define HVQ_NSUB_32k                          10
#define HVQ_NSUB_24k                          7             /* HVQ Classifier number of subbands */

#define HVQ_BWE_NOISE_BANDS                   2             /* Number of BWE noise bands */
#define HVQ_BWE_WEIGHT1_FX                    ((Word16)31130)  /* 0.95 in Q15 */
#define HVQ_BWE_WEIGHT2_FX                    ((Word16)6554)   /* 0.2 in Q15 */
#define HVQ_NFPE_FACTOR_CUBE_FX               ((Word16)16777)  /* 6.4^3 in Q6 */
#define HVQ_LB_NFPE_FACTOR_CUBE_FX            ((Word16)16777)  /* 3.2^3 in Q9 */

#define HVQ_VQ_DIM                            5             /* HVQ peak VQ dimension */
#define HVQ_PVQ_GAIN_BITS                     5             /* Number of bits to encode PVQ gains in HVQ */
#define HVQ_NUM_CLASS                         4             /* Number of codebook classes */
#define HVQ_CB_SIZE                           256

#define NUM_PG_HUFFLEN                        9             /* Number of Huffman codewords for peak gains */
#define MAX_PG_HUFFLEN                        12            /* Length of the longest codeword for peak gain Huffman coding */

#define HVQ_CP_HUFF_OFFSET                    3             /* HVQ Code Pos - Delta offset */
#define HVQ_CP_HUFF_MAX                       51            /* HVQ Code Pos - Maximum delta for huffman coding */
#define HVQ_CP_HUFF_MAX_CODE                  13            /* HVQ Code Pos - Size of largest code word */
#define HVQ_CP_HUFF_NUM_LEN                   11            /* HVQ Code Pos - Number of different huffman lengths */
#define HVQ_CP_L2_MAX                         64            /* HVQ Code Pos - Layer 2 maximum size */
#define HVQ_CP_L1_LEN                         5             /* HVQ Code Pos - Layer 1 block size */
#define HVQ_CP_MAP_LEN                        8             /* HVQ Code Pos - Mapping table size */
#define HVQ_CP_MAP_IDX_LEN                    3             /* HVQ Code Pos - Mapping index size */
#define HVQ_CP_DELTA                          0             /* HVQ Code Pos - Use Delta coding */
#define HVQ_CP_SPARSE                         1             /* HVQ Code Pos - Use Sparse coding */

#define MAX_SPLITS                            10            /* Maximum number of PVQ band splits */
#define THR_ADD_SPLIT                         7             /* Threshold for using additional split */
#define PVQ_MAX_BAND_SIZE                     64            /* Maxiumum supported band size for PVQ search */
#define MIN_BAND_SIZE                         1             /* Minimum supported band size for PVQ search */
#define RC_BITS_RESERVED                      1
#define MAX_PVQ_BITS_PER_COEFFICIENT          80            /* Maximum bits per coefficient allocated per PVQ band. Q3. */

/*  index_pvq constants  */
#define KMAX_FX                                512

#define KMAX_NON_DIRECT_FX                     96                /* max K  for non-direct indexing  recursion rows  is 1+KMAX_NON_DIRECT +1 */
#define ODD_DIV_SIZE_FX                        48                /* ind0=1/1  ind1 =1/3  ...  ind47=1/95 */

#define L2D_C1                              -2597       /* quadratic polyfit approximation in the region .5 to 1.0 */
#define L2D_C2                              7932        /*  log2(x) = L2D_C1*x^2+  L2D_C2*x  +  C */





/* TCQ */
#define TCQ_MAX_BAND_SIZE                   120      /* Maxiumum supported band size for TCQ+USQ search */
#define STATES                              8
#define MAX_AR_FREQ                         16383
#define AR_BITS                             16
#define STATES_LSB                          4
#define TCQ_LSB_SIZE                        24
#define TCQ_AMP                             10

#define AR_TOP                              ( ( 1 << AR_BITS ) - 1 )
#define AR_FIRST                            ( AR_TOP / 4 + 1 )
#define AR_HALF                             ( 2 * AR_FIRST )
#define AR_THIRD                            ( 3 * AR_FIRST )

#define MAX_SIZEBUF_PBITSTREAM              1024

/*----------------------------------------------------------------------------------*
 * SWB BWE for LR MDCT core
 *----------------------------------------------------------------------------------*/
/* Q value */
#define SWB_BWE_LR_Qs                         12    /* Q value of spectra (Word32) */
#define SWB_BWE_LR_Qbe                        14    /* Q value of band_energy (Word32) */
#define SWB_BWE_LR_QRk                        16    /* Q value of Rk, Rcalc (Word32) */
#define SWB_BWE_LR_QsEn                        4    /* Q value of smoothed sqrt band energy */

#define G1_RANGE                              4
#define G1G2_RANGE                           15
#define GRP_SB                                4         /*Maximum subband groups*/
#define THR1                                  4         /* Bit allocation threshold value */
#define THR2                                  5         /* Bit allocation threshold value */
#define THR3                                  6         /* Bit allocation threshold value */

#define NB_SWB_SUBBANDS                       4         /* maximum number of subbands in normal2 subband coding */

#define SWB_SB_LEN0_12KBPS                    55/* length of subband number X in lowest bit rate operation */
#define SWB_SB_LEN1_12KBPS                    68
#define SWB_SB_LEN2_12KBPS                    84
#define SWB_SB_LEN3_12KBPS                    105

#define SWB_HIGHBAND_12KBPS                   (SWB_SB_LEN0_12KBPS+SWB_SB_LEN1_12KBPS+SWB_SB_LEN2_12KBPS+SWB_SB_LEN3_12KBPS)
#define SWB_LOWBAND_12KBPS                    (HQ_GENERIC_END_FREQ - SWB_HIGHBAND_12KBPS)
#define SWB_HIGHBAND_MAX                      SWB_HIGHBAND_12KBPS
#define SWB_LOWBAND_MAX                       SWB_LOWBAND_12KBPS

#define SWB_SB_OFF0_12KBPS                    0         /* subband offsets are based on the subband lengths */
#define SWB_SB_OFF1_12KBPS                    (SWB_SB_OFF0_12KBPS + SWB_SB_LEN0_12KBPS)
#define SWB_SB_OFF2_12KBPS                    (SWB_SB_OFF1_12KBPS + SWB_SB_LEN1_12KBPS)
#define SWB_SB_OFF3_12KBPS                    (SWB_SB_OFF2_12KBPS + SWB_SB_LEN2_12KBPS)
#define SWB_SB_OFF4_12KBPS                    (SWB_SB_OFF3_12KBPS + SWB_SB_LEN3_12KBPS)

/* 16.4 kbps */
#define SWB_SB_LEN0_16KBPS                    59/* length of subband number X in lowest bit rate operation */
#define SWB_SB_LEN1_16KBPS                    74
#define SWB_SB_LEN2_16KBPS                    92
#define SWB_SB_LEN3_16KBPS                    115

#define SWB_HIGHBAND_16KBPS                   (SWB_SB_LEN0_16KBPS+SWB_SB_LEN1_16KBPS+SWB_SB_LEN2_16KBPS+SWB_SB_LEN3_16KBPS)
#define SWB_LOWBAND_16KBPS                    (HQ_GENERIC_END_FREQ - SWB_HIGHBAND_16KBPS)

#define SWB_SB_OFF0_16KBPS                    0         /* subband offsets are based on the subband lengths */
#define SWB_SB_OFF1_16KBPS                    (SWB_SB_OFF0_16KBPS + SWB_SB_LEN0_16KBPS)
#define SWB_SB_OFF2_16KBPS                    (SWB_SB_OFF1_16KBPS + SWB_SB_LEN1_16KBPS)
#define SWB_SB_OFF3_16KBPS                    (SWB_SB_OFF2_16KBPS + SWB_SB_LEN2_16KBPS)
#define SWB_SB_OFF4_16KBPS                    (SWB_SB_OFF3_16KBPS + SWB_SB_LEN3_16KBPS)

/* SpectrumSmoothing */
#define L_SB                                  12       /* subband length for SpectrumSmoothing */

/* SpectrumSmoothing for NSS */
#define L_SB_NSS                              8
#define L_SB_NSS_HALF                         (L_SB_NSS/2)
#define NUM_SUBBAND_SMOOTH_MAX                (L_FRAME32k/L_SB_NSS+1)
#define MA_LEN                                7

/* Harmonic mode */
#define NB_SWB_SUBBANDS_HAR_SEARCH_SB         2        /* search number of subbands in harmonic subband coding */
#define NB_SWB_SUBBANDS_HAR                   4        /* maximum number of subbands in harmonic subband coding */


#define N_NBIGGEST_PULSEARCH                  18
#define N_NBIGGEST_SEARCH_LRG_B               32


/* 13.2 kbps */
#define SWB_SB_BW_LEN0_12KBPS_HAR        56  /* Group 1 length for BWE */
#define SWB_SB_BW_LEN1_12KBPS_HAR       100  /* Group 2 Length for BWE */
#define SWB_SB_BW_LEN2_12KBPS_HAR        SWB_SB_BW_LEN1_12KBPS_HAR
#define SWB_SB_BW_LEN3_12KBPS_HAR        SWB_SB_BW_LEN0_12KBPS_HAR

/* 16.4 kbps */
#define SWB_SB_BW_LEN0_16KBPS_HAR        60  /* Group 1 length for BWE */
#define SWB_SB_BW_LEN1_16KBPS_HAR       110  /* Group 2 Length for BWE */
#define SWB_SB_BW_LEN2_16KBPS_HAR        SWB_SB_BW_LEN1_16KBPS_HAR
#define SWB_SB_BW_LEN3_16KBPS_HAR        SWB_SB_BW_LEN0_16KBPS_HAR

#define SWB_SB_OFF0_SUB5_12KBPS_HAR           0     /* subband offsets are based on the subband lengths */
#define SWB_SB_OFF1_SUB5_12KBPS_HAR           (SWB_SB_OFF0_SUB5_12KBPS_HAR + SWB_SB_BW_LEN0_12KBPS_HAR)
#define SWB_SB_OFF2_SUB5_12KBPS_HAR           (SWB_SB_OFF1_SUB5_12KBPS_HAR + SWB_SB_BW_LEN1_12KBPS_HAR)
#define SWB_SB_OFF3_SUB5_12KBPS_HAR           (SWB_SB_OFF2_SUB5_12KBPS_HAR + SWB_SB_BW_LEN2_12KBPS_HAR)

#define SWB_SB_OFF0_SUB5_16KBPS_HAR           0     /* subband offsets are based on the subband lengths */
#define SWB_SB_OFF1_SUB5_16KBPS_HAR           (SWB_SB_OFF0_SUB5_16KBPS_HAR + SWB_SB_BW_LEN0_16KBPS_HAR)
#define SWB_SB_OFF2_SUB5_16KBPS_HAR           (SWB_SB_OFF1_SUB5_16KBPS_HAR + SWB_SB_BW_LEN1_16KBPS_HAR)
#define SWB_SB_OFF3_SUB5_16KBPS_HAR           (SWB_SB_OFF2_SUB5_16KBPS_HAR + SWB_SB_BW_LEN2_16KBPS_HAR)

#define LR_BLK_LEN                            16
#define LR_HLF_PK_BLK_LEN                     8
#define LR_LOWBAND_DIF_PK_LEN                 10
#define SWB_HAR_RAN1                          80
#define SWB_HAR_RAN2                          140
#define SWB_HAR_RAN3                          200
#define NI_USE_PREV_SPT_SBNUM                 10
#define SPT_SHORTEN_SBNUM                     4

/*----------------------------------------------------------------------------------*
 * FEC for HQ core
 *----------------------------------------------------------------------------------*/

#define MAX_PGF                               7
#define MAX_ROW                               2
#define MAX_SB                                9
#define MAX_SB_BWE                            3

#define MINIMUM_RATE_TO_ENCODE_VOICING_FLAG   45000
#define START_HIGH_FREQ                       2000
#define FRAMECTTOSTART_MDCT                   3

/*----------------------------------------------------------------------------------*
 * Channel-aware mode (FEC)
 *----------------------------------------------------------------------------------*/

#define LSF_BITS_RF_PREDONLY                 26   /* RF bit allocation */
#define LSF_BITS_RF_NOPRED                   15
#define LSF_MID_BITS_RF_PREDONLY              4   /* mid_LSF_bits_tbl[LSF_BIT_ALLOC_IDX(ACELP_3k60, GENERIC)] */
#define LSF_MID_BITS_RF_UNVOICED              4   /* mid_LSF_bits_tbl[LSF_BIT_ALLOC_IDX(ACELP_3k60, UNVOICED)] */
#define GAIN_BITS_RF_PREDONLY                 3
#define GAIN_BITS_RF_NOPRED                   3

#define GAIN_PIT_CBSIZE                       8
#define GAIN_CODE_CF_CBSIZE2                  4
#define GAIN_CODE_CF_CBSIZE3                  8

#define WSNR_THRLD1                           0   /* thresholds from more aggressive setting in FEC.c */
#define WSNR_THRLD2                           2

#define FRAME_ERASURE_RATE                    0   /* 0 = LOW, 1 = HIGH */

#define FEC_OFFSET                            3
#define MAX_RF_FEC_OFFSET                     10

#define RF_LSF_BITS_STAGE1                    8
#define RF_LSF_BITS_STAGE2                    8

/*----------------------------------------------------------------------------------*
 * HQ FEC
 *----------------------------------------------------------------------------------*/

#define MAXDELAY_FEC                          224

#define PH_ECU_ALDO_OLP2_NS                   (ACELP_LOOK_NS/2)                     /* half the aldo window overlap */
#define PH_ECU_LOOKAHEAD_NS                   (11*ACELP_LOOK_NS/(7*2))              /* Number of nanoseconds look-ahead ahead from the end of the past synthesized frame */
#define POST_HQ_DELAY_NS                      DELAY_BWE_TOTAL_NS                    /* delay of post processing after core HQ coding */
#define MAX_PLOCS                             Lprot48k/4+1                          /* maximum number of spectral peaks to be searched */
#define Lprot32k                              1024                                  /* HQ phase ECU prototype frame length */
#define Lprot32k_2                            Lprot32k/2
#define Quot_Lpr_Ltr                          4
#define Lgw_max                               9                                     /* maximum number frequency group widths */
#define BETA_MUTE_FAC_INI                     16384                                 /* Q15, initial noise attenuation factor */
#define Ltrana32k                             (Lprot32k/Quot_Lpr_Ltr)               /* transient analysis frame length */
#define Ltrana16k                             Ltrana32k/2
#define Ltrana8k                              Ltrana32k/4
#define pfind_sens                            0.9f                                  /* peakfinder sensitivity - tuning parameter */
#define Lprot_hamm_len2_48k                   NS2SA(48000,6000000L)
#define Lprot_hamm_len2_32k                   NS2SA(32000,6000000L)
#define Lprot_hamm_len2_16k                   NS2SA(16000,6000000L)
#define Lprot_hamm_len2_8k                    NS2SA(8000,6000000L)
#define Lprot48k                              Lprot32k * 3/2                        /* HQ phase ECU prototype frame length */
#define Lprot48k_2                            Lprot48k/2
#define Ltrana48k                             (Lprot48k/Quot_Lpr_Ltr)               /* transient analysis frame length */
#define PH_ECU_SPEC_SIZE                      Lprot48k
#define T_SIN_PI_2                            PH_ECU_SPEC_SIZE/4
#define RANDOM_START                          1
#define HQ_FEC_SIGN_SFM                       16
#define HQ_FEC_SIGN_THRES                     6
#define HQ_FEC_SIGN_THRES_TRANS               3
#define HQ_FEC_BAND_SIZE                      4
#define HQ_FEC_BAND_SIZE_SHIFT                2  /* 2^2 = HQ_FEC_BAND_SIZE */

/*--------------------------------------------------------------*
 * Tonal MDCT PLC constants
 *---------------------------------------------------------------*/
#define SPEC_EXP_DEC                          20

#define MAX_NUMBER_OF_IDX                     30
#define GROUP_LENGTH                          7
#define MAX_PEAKS_FROM_PITCH                  10
#define LAST_HARMONIC_POS_TO_CHECK            128 /* 128 because we check harmonics only up to 3.2 kHz */
#define FILTLEN                               15
#define ALLOWED_SIDE_LOBE_FLUCTUATION         FL2WORD16_SCALE(3.0f, 2) /*  4.8 dB */
#define ALLOWED_SIDE_LOBE_FLUCTUATION_EXP     2
#define LEVEL_ABOVE_ENVELOPE                  7.59f /*  8.8 dB */
#define UNREACHABLE_THRESHOLD                 FL2WORD16_SCALE(16.0f, 5) /*   12 dB Increase of LEVEL_ABOVE_ENVELOPE so that the threshold is not reached */
#define SMALL_THRESHOLD                       FL2WORD16_SCALE(1.10f, 5) /* 0.41 dB Increase of LEVEL_ABOVE_ENVELOPE for the peak detection at a definitive peak in the estimated spectrum */
#define BIG_THRESHOLD                         FL2WORD16_SCALE(1.5f, 5) /* 1.76 dB Increase of LEVEL_ABOVE_ENVELOPE for the peak detection at a probable peak in the estimated spectrum */

#define kSmallerLagsTargetBitsThreshold       150

#define kLtpHmGainThr                         0x3AE1 /* 0.46f */
#define kCtxHmOlRSThr                         0x5333 /* 2.6f (2Q13) */

#define kTcxHmSnrOffsetGc                     FL2WORD16_SCALE(0.03125f, 7)
#define kTcxHmSnrOffsetVc                     0

#define kTcxHmNumGainBits                     2         /* Number of bits for the gain index */
#define kTcxHmParabolaHalfWidth               4         /* Parabola half width */
#define kLtpHmGainThr                         0x3AE1 /* 0.46f */  /* Use the LTP pitch lag in the harmonic model? */

#define kSmallerLagsTargetBitsThreshold       150

#define TCX_LPC_OFFSET_SNR

#define kTcxSnrOffsetGc                       0x08 /* 0.03125f (7Q8) */
#define kTcxSnrOffsetVc                       0x20 /* 0.125f (7Q8) */

#define LOWRATE_TCXLPC_MAX_BR                 ACELP_9k60

/*--------------------------------------------------------------*
 * Waveform-adjustment MDCT PLC
 *---------------------------------------------------------------*/

#define DEC_STATE_LEN    10
#define MAX_POST_LEN     3

#define TCX_NONTONAL 0
#define TCX_TONAL 1

#define FIX_16(A) (Word16)(((A) >= 0) ? ((Word16)((A)*(SHRT_MAX)+0.5)) : ((Word16)((A)*(SHRT_MAX)-0.5)))

/*---------------------------------------------------------------*
 * IGF                                                           *
 *---------------------------------------------------------------*/
#define IGF_MAX_TILES               5
#define IGF_MAX_GRANULE_LEN         1200
#define IGF_TRANS_FAK               2
#define IGF_MAX_SFB                 23
#define IGF_NOF_GRIDS               3
#define IGF_MAX_SUBFRAMES           2

#define IGF_MODE_WB                 1
#define IGF_MODE_SWB                2
#define IGF_MODE_FB                 3


#define IGF_BITRATE_WB_9600         0
#define IGF_BITRATE_RF_WB_13200     1
#define IGF_BITRATE_SWB_9600        2
#define IGF_BITRATE_SWB_13200       3
#define IGF_BITRATE_RF_SWB_13200    4
#define IGF_BITRATE_SWB_16400       5
#define IGF_BITRATE_SWB_24400       6
#define IGF_BITRATE_SWB_32000       7
#define IGF_BITRATE_SWB_48000       8
#define IGF_BITRATE_SWB_64000       9
#define IGF_BITRATE_FB_16400        10
#define IGF_BITRATE_FB_24400        11
#define IGF_BITRATE_FB_32000        12
#define IGF_BITRATE_FB_48000        13
#define IGF_BITRATE_FB_64000        14
#define IGF_BITRATE_FB_96000        15
#define IGF_BITRATE_FB_128000       16
#define IGF_BITRATE_UNKNOWN         17


#define IGF_WHITENING_OFF           0
#define IGF_WHITENING_MID           1
#define IGF_WHITENING_STRONG        2

#define IGF_GRID_LB_NORM            0
#define IGF_GRID_LB_TRAN            1
#define IGF_GRID_LB_SHORT           2
#define IGF_GRID_UNUSED             3

/* constants for IGFSCFDecoder and IGFSCFEncoder */
#define IGF_CTX_OFFSET              3                                                           /* offset added to make the context values nonnegative, for array indexing */
#define IGF_CTX_COUNT               (2 * IGF_CTX_OFFSET + 1)                                    /* number of contexts for the AC statistical model */
#define IGF_MIN_ENC_SEPARATE       -12                                                          /* minimum residual value coded separately, without escape coding */
#define IGF_MAX_ENC_SEPARATE       +12                                                          /* maximum residual value coded separately, without escape coding */
#define IGF_SYMBOLS_IN_TABLE        (1 + (IGF_MAX_ENC_SEPARATE - IGF_MIN_ENC_SEPARATE + 1) + 1) /* alphabet size */

/*----------------------------------------------------------------------------------*
 * SC-VBR
 *----------------------------------------------------------------------------------*/

#define UVG1_CBSIZE                           32        /* NELP unvoiced gain-1 codebook size */
#define UVG2_CBSIZE                           64        /* NELP unvoiced gain-2 codebook size */

/* PPP constants */
#define MAXLAG_WI                             PIT16k_MAX/2  /* Maximum lag used in waveform interpolation */
#define NUM_ERB_WB                            24        /* Number of ERB bands in wideband */
#define NUM_ERB_NB                            22        /* Number of ERB bands in narrowband */

#define MAXLAG                                180

#define PPP_LAG_THRLD_Q6                      180*64
#define PPP_LAG_THRLD                         180
#define SNR_THLD_FX_Q8                        17152      /* Threshold is upscaled to Q8 to compared with st->vadsnr*/

#define WI_FX_phase_fx                        512

#define VBR_ADR_MAX_TARGET_x10_Q1             123 /* VBR_ADR_MAX_TARGET(6.15f)*10 in Q1 */

#define Q_SCALE                               7

#define CONST_1_16_Q14 19005      /* 1.16*16384 */
#define CONST_1_37_Q14 22446      /* 1.37*16384 */
#define SCALE_DOWN_ERAS_Q15 26214 /* 0.8*32768 */
#define SHAPE1_COEF_QF 15         /* shape1 num and den coeffcient Q format */
#define SHAPE2_COEF_QF 15         /* shape2 num and den coeffcient Q format */
#define SHAPE3_COEF_QF 15         /* shape3 num and den coeffcient Q format */
#define BP1_COEF_WB_QF 14         /* wb num and den coeffcient Q format */
#define BP1_COEF_NB_QF_ORDER7 13         /* nb num and den coeffcient Q format */

/*----------------------------------------------------------------------------------*
 * JBM
 *----------------------------------------------------------------------------------*/

#define MAX_JBM_SLOTS                         100 /* every primary copy and partial copy stored in JBM needs one slot */
#define MAX_AU_SIZE                           (128000/50/8) /* max frame size in bytes */

#define ACTIVE                                4
#define BACKGROUND                            1
#define LOST                                  14
#define SIZE_UNDERFLOW_ARRAY                  10000
#define TIME_WARP_SPACING                     4
#define FLAG_DECREASING_BUFFER_DEPTH          1
#define TIME_WARP_SPACING_EXPAND              1
#define PREALLOCATE_OUTPUT_SIZE               1000

#define TSM_LOST                              1
#define TSM_AUDIO                             2
#define TSM_ONSET                             3
#define TSM_UNVOICED                          4
#define TSM_VOICED                            5
#define TSM_TRANSITION                        6
#define TSM_GENERIC                           7
#define TSM_BACKGROUND                        8

/*----------------------------------------------------------------------------------*
 * Default decoder initialization
 *----------------------------------------------------------------------------------*/

#define DEC_SR_CORE_DEFAULT 12800
#define DEC_BR_DEFAULT      24400
#define DEC_BW_DEFAULT      SWB

/*------------------------------------------------------------------------------------*
FEC_clas_estim constants
*-------------------------------------------------------------------------------------*/
#define Q_MAX                                 12
#define LG10                                  24660       /*  10*log10(2)  in Q13                 */
#define L_Q_MEM                               5

#define GE_SHIFT                              6
#define LSF_1_OVER_256SQ                      5000
#define LSF_1_OVER_256SQSQ                    763

#define G_CODE_MIN_TC_Q15                     655   /*0.02 Q15*/
#define G_CODE_MAX_TC_Q0                      5

#define G_PITCH_MIN_Q14                       0         /* SQ of gains: pitch gain lower limit (0.0 in Q13)     */
#define G_PITCH_MAX_MINUS_MIN_Q13             9994      /* SQ of gains: pitch gain upper limit (1.22-0 in Q13)  */

/* higher ACELP constants */
#define G_AVQ_MIN_32kbps_Q15                  2621      /* lower limit for gain Q in higher-rate ACELP contribtuion @32kbps    (0.08 in Q15) */
#define G_AVQ_MIN_Q15                         26214     /* lower limit for gain Q in higher-rate ACELP contribtuion            (0.8 in Q15)  */
#define G_AVQ_MAX_Q0                          96        /* upper limit for gain Q in higher-rate ACELP contribtuion                          */

#define LG10_G_AVQ_MIN_32kbps_Q14           (-17972)    /* log10(0.08) lower limit for gain Q in higher-rate ACELP contribtuion @32kbps    (0.08 in Q15) */
#define LG10_G_AVQ_MIN_Q14                   (-1588)    /* log10(0.8)  lower limit for gain Q in higher-rate ACELP contribtuion            (0.8 in Q15)  */
#define LG10_G_AVQ_MAX_Q13                     16239    /* log10(96)   upper limit for gain Q in higher-rate ACELP contribtuion                          */
#define LG10_G_CODE_MIN_TC192_Q14            (-3635)    /* log10(0.6) */
#define LG10_G_CODE_MAX_TC192_Q13              13212    /* log10(41) */
#define LG10_G_CODE_MAX_Q13                     5726    /* log10(5) SQ of gains: code gain upper limit */
#define LG10_G_CODE_MIN_Q14                 (-27836)    /* log10(0.02) SQ of gains: code gain lower limit */
#define LG10_G_CODE_MIN_TC_Q14    LG10_G_CODE_MIN_Q14    /* log10(0.02) Q15*/
#define LG10_G_CODE_MAX_TC_Q13    LG10_G_CODE_MAX_Q13


/* AVQ (RE8) related consatnts */
#define QR                                    32768

#define Q_AVQ_OUT 6                         /* AVQ_output scaling currently Q9, but may change */
#define Q_AVQ_OUT_DEC 10


#define PIT_DECODE_2XL_SUBFR (2*L_SUBFR)
#define PIT_FR1_8b_MINUS_PIT_MIN_X2 ((PIT_FR1_8b-PIT_MIN)*2)
#define PIT_FR1_8b_MINUS_PIT_FR1_8b_MINUS_PIT_MIN_X2 (PIT_FR1_8b-PIT_FR1_8b_MINUS_PIT_MIN_X2)
#define PIT_FR2_9b_MINUS_PIT_MIN_X4 ((PIT_FR2_9b-PIT_MIN)*4)
#define PIT_FR1_EXT8b_MINUS_PIT_MIN_EXT_X2 ((PIT_FR1_EXTEND_8b-PIT_MIN_EXTEND)*2)
#define PIT_FR2_EXT9b_MINUS_PIT_MIN_EXT_X4 ((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4)
#define PIT_FR1_EXT9b_MINUS_PIT_FR2_EXT9b_X2 ((PIT_FR1_EXTEND_9b-PIT_FR2_EXTEND_9b)*2)
#define PIT_FR1_DEXT8b_MINUS_PIT_MIN_DEXT_X2 ((PIT_FR1_DOUBLEEXTEND_8b-PIT_MIN_DOUBLEEXTEND)*2 )
#define PIT_FR1_DEXT9b_MINUS_PIT_MIN_DEXT_X4 ((PIT_FR1_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4 )
#define PIT_FR1_DEXT9b_MINUS_PIT_FR2_DEXT9b_X2 ((PIT_FR1_DOUBLEEXTEND_9b-PIT_FR2_DOUBLEEXTEND_9b)*2)
#define PIT_FR2_DEXT9b_MINUS_PIT_MIN_DEXT_X4 ((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4 )

#define PIT_DECODE_1 ((PIT_FR2_9b-PIT_MIN)*4 + (PIT_FR1_9b-PIT_FR2_9b)*2)
#define PIT_DECODE_2 ((PIT_FR2_9b-PIT_MIN)*4)
#define PIT_DECODE_3 (PIT_FR1_9b - ((PIT_FR2_9b-PIT_MIN)*4) - ((PIT_FR1_9b-PIT_FR2_9b)*2))
#define PIT_DECODE_4 ((PIT16k_FR2_9b-PIT16k_MIN)*4)
#define PIT_DECODE_5 ((PIT16k_FR2_9b-PIT16k_MIN)*4 + (PIT16k_FR1_9b-PIT16k_FR2_9b)*2)
#define PIT_DECODE_6 (PIT16k_FR1_9b - ((PIT16k_FR2_9b-PIT16k_MIN)*4) - ((PIT16k_FR1_9b-PIT16k_FR2_9b)*2))

#define PIT_DECODE_7 (PIT_FR1_EXTEND_9b - (PIT_FR2_EXT9b_MINUS_PIT_MIN_EXT_X4) - (PIT_FR1_EXT9b_MINUS_PIT_FR2_EXT9b_X2))
#define PIT_DECODE_8 (PIT_FR1_DOUBLEEXTEND_8b - ((PIT_FR1_DOUBLEEXTEND_8b-PIT_MIN_DOUBLEEXTEND)*2))
#define PIT_DECODE_9 (( (PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4 + (PIT_FR1_DOUBLEEXTEND_9b-PIT_FR2_DOUBLEEXTEND_9b)*2))
#define PIT_DECODE_10 (PIT_FR1_DOUBLEEXTEND_9b - ((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4) - ((PIT_FR1_DOUBLEEXTEND_9b-PIT_FR2_DOUBLEEXTEND_9b)*2))

#define BIN4_FX                  800
#define TH_COR_FX                19661    /* Q15 Minimum correlation for per bin processing                        */
#define TH_D_FX                  800    /* 50 (Q4) Difference limit between nearest harmonic and a frequency bin */
#define TH_PIT_FX                128    /* (Fs / (2.0f * TH_D)) Maximum pitch for per bin processing             */

#define Q_SYN2_MAX           4 /* 1 -- > introduce by Sasken... */         /* Maximum scaling in synthesis domain */

#define GUESS_TBL_SZ                          256
#define INT_FS_FX                             12800  /* internal sampling frequency                */
#define INT_FS_16k_FX                         16000  /* CELP core internal sampling frequency @16kHz         */

#define WB_LIMIT_LSF_FX                       16256
#define Fs_2                                  16384     /* lsf max value (Use in reorder_fx.c) */
#define Fs_2_16k                              20480     /* lsf max value (Use in reorder_fx.c) */

#define LG10                                  24660       /*  10*log10(2)  in Q13                 */
#define LG10_s3_0                             16440       /* 10*log10(2)/1.55 = 1.00343331 in Q14              */

#define MU_MA_FX                              10923     /* original prediction factor for the AMR WB tables (Q15) */

#define E_MIN_FXQ15                           115     /* Q15*/

#define MAX_DYNAMIC_FX                         (82*128)
#define MIN_DYNAMIC_FX                         (50*128)
#define DYNAMIC_RANGE_FX                       (MAX_DYNAMIC_FX-MIN_DYNAMIC_FX)
#define MAX_GSC_NF_BITS                        3
#define GSC_NF_STEPS                           (1 << MAX_GSC_NF_BITS)
#define GSF_NF_DELTA_FX                        (DYNAMIC_RANGE_FX/GSC_NF_STEPS)

/*----------------------------------------------------------------------------------*
 * DTC/iDCT constants
 *----------------------------------------------------------------------------------*/

/* DCT related */
#define KP559016994_16FX                        1200479845   /* EDCT & EMDCT constants */
#define KP951056516_16FX                        2042378325   /* EDCT & EMDCT constants */
#define KP587785252_16FX                        1262259213   /* EDCT & EMDCT constants */

#define NUM_TIME_SWITCHING_BLOCKS_EXP 2



/*----------------------------------------------------------------------------------*
 * AVQ constants
 *----------------------------------------------------------------------------------*/

#define MIN_GLOB_GAIN_BWE_HR_FX 24576 /* Q13 */
#define MAX_GLOB_GAIN_BWE_HR_FX 32000 /* Q6 */

#define LG10_MIN_GLOB_GAIN_BWE_HR_Q14  7817
#define LG10_MAX_GLOB_GAIN_BWE_HR_Q13 22110


/*----------------------------------------------------------------------------------*
 * LD music post-filter constants
 *----------------------------------------------------------------------------------*/

#define BIN_16kdct_fx                         (6400/DCT_L_POST)
#define E_MIN_Q15                             14680    /* 0.0035 -> Q22 */

#define QSCALE                      7
#define MODE2_E_MIN                FL2WORD16_SCALE(0.03f, 15-QSCALE)       /* 0.03f in QSCALE */
#define MODE2_E_MIN_Q15            FL2WORD16(0.03f)                        /* 0.03f in Q15*/

/* long term pst parameters : */
#define L2_LH2_L           4  /* log2(LH2_L)                        */
#define MIN_GPLT_FX     21845 /* LT gain minimum 1/1.5 (Q15) */

#define AGC_FAC_FX      32358 /* gain adjustment factor  0.9875 (Q15) */
#define AGC_FAC1_FX  (Word16)(32768L - AGC_FAC_FX)

#define DOWNMIX_12k8_FX                          1850

/*----------------------------------------------------------------------------------*
 * HQ Constants
 *----------------------------------------------------------------------------------*/

#define INV2POWHALF                               23170   /* Q15, sqrt(2)/2 */
#define EPSILLON_FX                              (Word16) 1
#define MAX_SFM_LEN_FX                            96
#define MAX_P_ATT_FX                             (Word16) 40   /* Maximum number of pulses for gain attenuation factor */
#define MAX_PHG_FX                               (Word16) 10   /* Q0, Maximum number of pulses for which to apply pulse height dependent gain */
#define PHG_START_FX                             (Word16) 3277 /* Q15, 0.1f */
#define PHG_DELTA_FX                             (Word16) 2621 /* Q15, ((PHG_END-PHG_START)/MAX_PHG) */
#define ENV_ADJ_START_FX                         (Word16) 6    /* Q0, Number of consecutive bands for which the attenuation is maximum */
#define ENV_ADJ_INV_INCL_FX                      (Word16) 6554 /* Q15, 1/5.0f, Inverse inclination for mapping between attenuation region width and attenuation limit */

#define INV_HVQ_THRES_BIN_24k                      9362      /* 1/224 in Q21 */
#define INV_HVQ_THRES_BIN_32k                      6554      /* 1/320 in Q21 */
#define INV_BANDS_PER_SUBFRAMES                    14564     /* 1/9   in Q17 */
#define NUM_SUBFRAMES                              4
#define BANDS_PER_SUBFRAMES                        9
#define ENERGY_TH_NUM_SUBFRAMES                    1638400L    /* 400 in Q12 */
#define INV_SFM_N_ENV_STAB                         19418      /* 1/27 in Q19 */
#define ENERGY_TH_FX                               819200L    /* 100 in Q13 */
#define ENERGY_LT_BETA_FX                          30474      /* 0.93 in Q15 - Smoothing factor for long-term energy measure */
#define ENERGY_LT_BETA_1_FX                        2294       /* 0.07 i Q15 -  (1 - ENERGY_LT_BETA_FX) */
#define INV_DELTA_TH                               13107      /* 1/5 in Q16 - Inverse Delta energy threshold for transient detection for envelope stability */

#define ONE_OVER_HVQ_BAND_MAX_BITS_24k_FX        ((Word16)  410) /* Q15, 1/80 = 0.0125      */
#define ONE_OVER_HVQ_BAND_MAX_BITS_32k_FX        ((Word16)  345) /* Q15, 1/95 = 0.010526315 */

#define INV_PREECHO_SMOOTH_LENP1_FX             1560 /*(32768 / (PREECHO_SMOOTH_LEN + 1.0));*/

#define INV_HVQ_BW                                 1024    /* Q15 1/32 */

#define ACELP_48k_BITS                             960   /* Q0 - ACELP_48k / 50  */


/*--------------------------------------
 * FEC_HQ_phase_ecu
 *-------------------------------------*/
#define DELAY_HQ_CORE_FX              6
#define OFF_FRAMES_LIMIT              15     /* Hard limit the variable "time_offs" to 15 */

/*----------------------------------------------------------------------------------*
 * FEC for HQ core
 *----------------------------------------------------------------------------------*/


#define MAX_SB_NB                              3


#define NELP_LP_ORDER   8
#define NUM_NELP_GAINS 10

#define N_LEAD_NB                             70                                    /* (N_LEAD_MDCT*(L_FRAME8k/20)) */
#define N_ZERO_NB                             45                                    /* (N_ZERO_MDCT*(L_FRAME8k/20)) */
#define N_LEAD_O_NB                           90                                    /* (20.f-N_LEAD_MDCT)*(L_FRAME8k/20) */
#define N_ZERO_O_NB                           35                                    /* (10.f-N_ZERO_MDCT)*(L_FRAME8k/20) */
#define N_Z_L_NB                              115                                   /* (N_Z_L_MDCT*(float)(L/20)) = N_ZERO_NB + N_LEAD_NB*/
#define N_Z_L_O_NB                            205                                   /* (N_Z_L_O_MDCT*(float)(L/20)) = N_ZERO_NB + N_LEAD_NB + N_LEAD_O_NB */

#define ED_THRES_12P_fx                        66
#define ED_THRES_50P_fx                        326
#define ED_THRES_90P_fx                        1091

/*----------------------------------------------------------------------------------*
 * ISF quantizer constants
 *----------------------------------------------------------------------------------*/

/*qlpc_avq, lsf_msvq_ma*/
#define W_MODE0      0x1333    /*60.0f/FREQ_DIV in 0Q15*/
#define W_MODE1      0x14CD    /*65.0f/FREQ_DIV in 0Q15*/
#define W_MODE2      0x147B    /*64.0f/FREQ_DIV in 0Q15*/
#define W_MODE_ELSE  0x1429    /*60.0f/FREQ_DIV in 0Q15*/


enum TRACKPOS
{
    TRACKPOS_FIXED_FIRST        =0,         /* Fill tracks from left */
    TRACKPOS_FIXED_EVEN         =1,         /* Even tracks */
    TRACKPOS_FIXED_FIRST_TWO    =2,         /* One track of 32 */
    TRACKPOS_FIXED_TWO          =3,         /* Two tracks of 32 instead of four of 16 */
    TRACKPOS_FREE_ONE           =4,         /* One freely moving track with extra pulse */
    TRACKPOS_FREE_THREE         =5          /* Three freely moving tracks with single extra pulses */
};

#define ALP_E 2

/* Lag window strengths */
enum
{
    LAGW_WEAK,    /* weak lag window            */
    LAGW_MEDIUM,  /* medium strength lag window */
    LAGW_STRONG,  /* strong lag window          */

    NUM_LAGW_STRENGTHS
};

/* qmc_cng_common.h */
/* codec side */
#define SCALE_TABLE_SHIFT_FACTOR              (31-23)

/* parameter_bitmapping.h */
#define NPARAMS_MAX                           10

/*tns_tables.h*/

#define TRUE    1
#define FALSE   0

#define PRED_GAIN_E 8

/* Flags for CLDFB intialization */
/* Flag indicating that the states should be kept. */
#define CLDFB_FLAG_KEEP_STATES  8
/* Flag indicating 2.5 ms setup */
#define CLDFB_FLAG_2_5MS_SETUP 128


/* bits_alloc.h */
#define RATE_MODE_MAX 2              /*Number of rate mode*/
#define BANDWIDTH_MODE_MAX 2  /*Number of different bandwidth modes (NB/WB-FB)*/


typedef enum SIGNAL_CLASSIFER_MODE
{
    CLASSIFIER_ACELP,
    CLASSIFIER_TCX
} SIGNAL_CLASSIFIER_MODE;


#define NRG_CHANGE_E 8
#define AVG_FLAT_E 8

#define FRAME_SIZE_NB 16

/* RTP Payload Header Info */
#define ACTIVE_FRAME 0xFF
#define SID_FRAME    0xFA
#define ZERO_FRAME   0xF0


/*isf_enc and lsf_msvq_ma*/
#define MEL_IRL                                         23

/************************************************************************/

/************************************************************************/
#define COMVAD_CONTINUOUS_SP_NUM2_THR 40
#define UNKNOWN_NOISE 0
#define SILENCE       1

#define CNT0P1            1717986918  /* 0.1*2^34 */
#define CNT0P001          1099511627  /* 0.001*2^40 */
#define CNT0P0001         1759218604  /* 0.0001*2^44 */

#define CNT0P05           1717986918  /* 0.05*2^35 */

#define CNT1DIV28         1227133513  /* (1/28)*2^35 */
#define CNT1DIV14         1227133513  /* (1/14)*2^34 */
#define CNT1DIV8          1073741824  /* (1/8)*2^33 */

#define SP_CENTER_Q       10
#define ITD_STABLE_RATE_Q 15
#define SPEC_AMP_Q        14
#define SFM_Q             15
#define TONA_Q            14
#define lt_bg_highf_eng_Q 16

/************************************************************************/
/* TEC/TFA                                                              */
/************************************************************************/
#define DELAY_TEMP_ENV_BUFF_TEC   9
#define EXT_DELAY_HI_TEMP_ENV     2


/************************************************************************/
/* CLDFB                                                                */
/************************************************************************/

#define SCALE_MODULATION              ( 1 )

#define SCALE_GAIN_ANA_10             ( 4 )
#define SCALE_GAIN_ANA_16             ( 3 )
#define SCALE_GAIN_ANA_20             ( 3 )
#define SCALE_GAIN_ANA_32             ( 2 )
#define SCALE_GAIN_ANA_40             ( 2 )
#define SCALE_GAIN_ANA_60             ( 1 )

#define SCALE_GAIN_SYN                (-6 )

#define SCALE_CLDFB_ANA_10              ( SCALE_MODULATION + SCALE_GAIN_ANA_10  )
#define SCALE_CLDFB_ANA_16              ( SCALE_MODULATION + SCALE_GAIN_ANA_16  )
#define SCALE_CLDFB_ANA_20              ( SCALE_MODULATION + SCALE_GAIN_ANA_20  )
#define SCALE_CLDFB_ANA_32              ( SCALE_MODULATION + SCALE_GAIN_ANA_32  )
#define SCALE_CLDFB_ANA_40              ( SCALE_MODULATION + SCALE_GAIN_ANA_40  )
#define SCALE_CLDFB_ANA_60              ( SCALE_MODULATION + SCALE_GAIN_ANA_60  )

#define SCALE_CLDFB_SYN_10              ( SCALE_MODULATION + SCALE_GAIN_SYN )
#define SCALE_CLDFB_SYN_16              ( SCALE_MODULATION + SCALE_GAIN_SYN )
#define SCALE_CLDFB_SYN_20              ( SCALE_MODULATION + SCALE_GAIN_SYN )
#define SCALE_CLDFB_SYN_32              ( SCALE_MODULATION + SCALE_GAIN_SYN )
#define SCALE_CLDFB_SYN_40              ( SCALE_MODULATION + SCALE_GAIN_SYN )
#define SCALE_CLDFB_SYN_60              ( SCALE_MODULATION + SCALE_GAIN_SYN )

/************************************************************************/
/* FFT                                                                  */
/************************************************************************/

#define SCALEFACTORN2        ( 3)
#define SCALEFACTOR2         ( 2)
#define SCALEFACTOR3         ( 3)
#define SCALEFACTOR4         ( 3)
#define SCALEFACTOR5         ( 4)
#define SCALEFACTOR8         ( 4)
#define SCALEFACTOR10        ( 5)
#define SCALEFACTOR12        ( 5)
#define SCALEFACTOR15        ( 5)
#define SCALEFACTOR16        ( 5)
#define SCALEFACTOR20        ( 5)
#define SCALEFACTOR24        ( 6)
#define SCALEFACTOR30        ( 6)
#define SCALEFACTOR30_1      ( 5)
#define SCALEFACTOR30_2      ( 1)
#define SCALEFACTOR32        ( 6)
#define SCALEFACTOR32_1      ( 5)
#define SCALEFACTOR32_2      ( 1)
#define SCALEFACTOR40        ( 7)
#define SCALEFACTOR60        ( 7)
#define SCALEFACTOR64        ( 7)
#define SCALEFACTOR80        ( 8)
#define SCALEFACTOR100       (10)
#define SCALEFACTOR120       ( 8)
#define SCALEFACTOR128       ( 8)
#define SCALEFACTOR160       ( 8)
#define SCALEFACTOR192       (10)
#define SCALEFACTOR200       (10)
#define SCALEFACTOR240       ( 9)
#define SCALEFACTOR256       ( 9)
#define SCALEFACTOR320       (10)
#define SCALEFACTOR400       (10)
#define SCALEFACTOR480       (11)
#define SCALEFACTOR600       (10)

#define BASOP_CFFT_MAX_LENGTH 600

#endif /* CNST_FX_H */
