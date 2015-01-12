/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef PROT_COM_FX_H
#define PROT_COM_FX_H
#include <stdio.h>
#include "options.h"        /* Compilation switches                   */
#include "rom_com_fx.h"     /* Compilation switches                   */
#include "typedefs.h"
#include "stat_dec_fx.h"
#include "stat_enc_fx.h"
#include "stat_com.h"
#include "cnst_fx.h"
#include "basop_util.h"

/*----------------------------------------------------------------------------------*
 * Prototypes of global macros
 *----------------------------------------------------------------------------------*/
#ifndef _MSC_VER
#ifndef min
#define min(x,y)                            ((x)<(y)?(x):(y))
#endif

#ifndef max
#define max(x,y)                            ((x)>(y)?(x):(y))
#endif
#endif


void reset_preecho_dec_fx (
    Decoder_State_fx *st_fx              /* i/o: decoder state structure   */
);

void preecho_sb_fx(
    const Word32  brate,                /* i   Q0  : core bit-rate                                           */
    Word32 *wtda_audio_fx,              /* i   q_sig32  : imdct signal                                       */
    Word16 q_sig32,                     /* i   Q value for wtda_audio_fx                                     */
    Word16 *rec_sig_fx,                 /* i   q_sig16  : reconstructed signal, output of the imdct transform     */
    Word16 *imdct_mem_fx,               /* i   q_sig16  : memory of the imdct transform, used in the next frame   */
    Word16 q_sig16,                     /* i   Q value for rec_sig_fx and imdct_mem_fx */
    const Word16 framelength,           /* i   Q0  : frame length                                            */
    Word16 *memfilt_lb_fx,              /* i/o Q0  : memory                                                  */
    Word32 *mean_prev_hb_fx,            /* i/o Q0  : memory                                                  */
    Word16 *smoothmem_fx,               /* i/o Q15 : memory                                                  */
    Word32 *mean_prev_fx,               /* i/o Q0  : memory                                                  */
    Word32 *mean_prev_nc_fx,            /* i/o Q0  : memory                                                  */
    Word16 *wmold_hb_fx,                /* i/o Q15 : memory                                                  */
    Word16 *prevflag,                   /* i/o Q0  : flag                                                    */
    Word16 *pastpre,                    /* i/o Q0  : flag                                                    */
    /* Word32 *InMDCT_fx, */                 /* i   Q0  : input MDCT vector                                       */
    /* const Word16 IsTransient,*/           /* i   Q0  : transient flag                                          */
    const Word16 bwidth                 /* i   Q0  : bandwidth                                               */
);

void calc_normal_length_fx_32(
    const Word16 core,              /* i  : core                   : Q0  */
    const Word32 *sp,               /* i  : input signal           : Q12 */
    const Word16 mode,              /* i  : input mode             : Q0  */
    const Word16 extl,              /* i  : extension layer        : Q0  */
    Word16 *L_swb_norm,       /* o  : normalize length       : Q0  */
    Word16 *prev_L_swb_norm   /*i/o : last normalize length  : Q0  */
);

void calc_norm_envelop_fx_32(
    const Word32 SWB_signal_fx[],           /* i  : SWB spectrum           : Q12 */
    Word32 *envelope_fx,                    /* o  : normalized envelope    : Q16 */
    const Word16 L_swb_norm,                /* i  : length of envelope     : Q0  */
    const Word16 SWB_flength,               /* i  : Length of input/output : Q0  */
    const Word16 st_offset                  /* i  : offset                 : Q0  */
);

void push_indice_fx(
    Encoder_State_fx *st_fx,                  /* i/o: encoder state structure */
    Word16 id,                        /* i  : ID of the indice */
    UWord16 value,                    /* i  : value of the quantized indice */
    Word16 nb_bits                    /* i  : number of bits used to quantize the indice */
);

void push_next_indice_fx(
    Encoder_State_fx *st_fx,                  /* i/o: encoder state structure */
    UWord16 value,                  /* i  : value of the quantized indice */
    Word16 nb_bits                 /* i  : number of bits used to quantize the indice */
);

void push_next_bits_fx(
    Encoder_State_fx *st_fx,                  /* i/o: encoder state structure */
    Word16 bits[],                 /* i  : bit buffer to pack, sequence of single bits */
    Word16 nb_bits                 /* i  : number of bits to pack */
);
void get_NextCoderType_fx(
    UWord8 *bitsteam,              /* i : bitstream         */
    Word16 *next_coder_type        /* o : next coder type   */
);
void evs_enc_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure   */
    const Word16 input_sp[],            /* i  : input signal              */
    const Word16 n_samples              /* i  : number of input samples  */
);


void io_ini_enc_fx(
    const int   argc,                /* i  : command line arguments number             */
    char  *argv[],             /* i  : command line arguments                    */
    FILE  **f_input,           /* o  : input signal file                         */
    FILE  **f_stream,          /* o  : output bitstream file                     */
    FILE  **f_rate,            /* o  : bitrate switching profile (0 if N/A)      */
    FILE  **f_bwidth,          /* o  : bandwidth switching profile (0 if N/A)    */
    FILE  **f_rf,              /* o  : channel aware config profile (0 if N/A)   */
    Word16 *quietMode,         /* o  : limit printouts                           */
    Word16 *noDelayCmp,        /* o  : turn off delay compensation               */
    Encoder_State_fx *st       /* o  : state structure                           */
);

void read_next_rfparam_fx(
    Word16* rf_fec_offset,    /* o: rf offset                         */
    Word16* rf_fec_indicator, /* o: rf FEC indicator                  */
    FILE* f_rf                /* i: file pointer to read parameters   */
);

void read_next_brate_fx(
    Word32  *total_brate,             /* i/o: total bitrate                             */
    const Word32 last_total_brate,    /* i  : last total bitrate                        */
    FILE  *f_rate,                    /* i  : bitrate switching profile (0 if N/A)      */
    Word32   input_Fs,                /* i  : input sampling frequency                  */
    Word16 *Opt_AMR_WB,               /* i  : flag indicating AMR-WB IO mode            */
    Word16 *Opt_SC_VBR,               /* i/o: SC-VBR flag                               */
    Word16 *codec_mode                /* i/o: MODE1 or MODE2                            */
);


void read_next_bwidth_fx(
    Word16  *max_bwidth,            /* i/o: maximum encoded bandwidth                 */
    FILE    *f_bwidth,              /* i  : bandwidth switching profile (0 if N/A)    */
    Word32  *bwidth_profile_cnt,    /* i/o: counter of frames for bandwidth switching profile file */
    Word32   input_Fs               /* i  : input sampling frequency                  */
);

UWord16 get_next_indice_fx(                 /* o  : value of the indice */
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 nb_bits                    /* i  : number of bits that were used to quantize the indice */
);

UWord16 get_next_indice_1_fx(               /* o  : value of the indice */
    Decoder_State_fx *st_fx                   /* i/o: decoder state structure */
);

void get_next_indice_tmp_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 nb_bits                    /* i  : number of bits that were used to quantize the indice */
);


UWord16 get_indice_fx(                      /* o  : value of the indice */
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 pos,                       /* i  : absolute position in the bitstream */
    Word16 nb_bits                    /* i  : number of bits that were used to quantize the indice */
);

UWord16 get_indice_1_fx(                    /* o  : value of the indice */
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 pos                        /* i  : absolute position in the bitstream */
);

void reset_indices_enc_fx(
    Encoder_State_fx *st_fx                   /* i/o: encoder state structure */
);


void reset_indices_dec_fx(
    Decoder_State_fx *st_fx                   /* i/o: decoder state structure */
);

void write_indices_fx(
    Encoder_State_fx *st_fx,                  /* i/o: encoder state structure */
    FILE *file                       /* i  : output bitstream file                     */
);

Word16 read_indices_fx(                     /* o  : 1 = OK, 0 = something wrong            */
    Decoder_State_fx *st_fx,                    /* i/o: decoder state structure */
    FILE *file,                      /* i  : bitstream file                         */
    Word16 rew_flag                    /* i  : rewind flag (rewind file after reading) */
);

void evs_dec_previewFrame(
    UWord8 *bitstream,                      /* i  : bitstream pointer */
    Word16 bitstreamSize,                   /* i  : bitstream size    */
    Word16 *partialCopyFrameType,           /* o  : frame type of the partial copy */
    Word16 *partialCopyOffset               /* o  : offset of the partial copy relative to the primary copy */
);

void read_indices_from_djb_fx(
    Decoder_State_fx *st,                     /* i/o: decoder state structure */
    UWord8 *pt_stream,                 /* i  : bitstream file                         */
    Word16 nbits                       /* i  : number of bits                         */
    ,Word16 partialframe                /* i  : partial frame information     */
    ,Word16 next_coder_type             /* i  : next coder type information     */
);

void reset_rf_indices(
    Encoder_State_fx *st                /* i: state structure - contains partial RF indices     */
);

void getPartialCopyInfo(
    Decoder_State_fx *st,                     /* i  : decoder state structure       */
    Word16 *coder_type,
    Word16 *sharpFlag
);

void get_rfFlag(
    Decoder_State_fx *st,                     /* i  : decoder state structure       */
    Word16 *rf_flag,                  /* o  : check for the RF flag         */
    Word16 *nBits,
    Word16 *ind
);

void get_rfFrameType(
    Decoder_State_fx *st,                     /* i  : decoder state structure       */
    Word16 *rf_frame_type             /* o  : RF frame type                 */
);

void get_rf_fec_offset(
    Decoder_State_fx *st,                     /* i  : decoder state structure       */
    Word16 *rf_fec_offset             /* o  : RF fec offset                 */
);

void get_rfTargetBits(
    Word16 rf_frame_type,             /* i  : RF frame type                 */
    Word16 *rf_target_bits            /* o  : Number of RF target bits      */
);


Word16 BRATE2IDX_fx(Word32 brate);
Word16 BRATE2IDX16k_fx(Word32 brate);

Word32 BIT_ALLOC_IDX_fx(Word32 brate, Word16 ctype, Word16 sfrm, Word16 tc);
Word32 BIT_ALLOC_IDX_16KHZ_fx(Word32 brate, Word16 ctype, Word16 sfrm, Word16 tc);

void save_old_syn_fx(
    const Word16 L_frame,                   /* i  : frame length                */
    const Word16 syn[],                     /* i  : ACELP synthesis             */
    Word16 old_syn[],                 /* o  : old synthesis buffer        */
    Word16 old_syn_mem[],             /* i/o: old synthesis buffer memory */
    const Word16 preemph_fac,               /* i  : preemphasis factor          */
    Word16 *mem_deemph                /* i/o: deemphasis filter memory    */
);

void evs_dec_fx(
    Decoder_State_fx *st_fx,                /* i/o  : Decoder state structure        */
    Word16 output_sp[],                     /* o    : output synthesis signal */
    frameMode_fx frameMode                  /* i  : Decoder frame mode */
);

Word16 decodeVoip(
    Decoder_State_fx *st_fx,
    FILE *f_stream,
    FILE *f_synth,
    const char *jbmTraceFileName
    , const char *jbmFECoffsetFileName    /* : Output file  for Optimum FEC offset        */
);


void io_ini_dec_fx(
    const int argc,                /* i  : command line arguments number             */
    char *argv[],             /* i  : command line arguments                    */
    FILE **f_stream,          /* o  : input bitstream file                      */
    FILE **f_synth,           /* o  : output synthesis file                     */
    Word16 *quietMode,             /* o  : limited printouts                         */
    Word16 *noDelayCmp,            /* o  : turn off delay compensation               */
    Decoder_State_fx *st_fx,           /* o  : Decoder static variables structure        */
    char **jbmTraceFileName    /* o  : VOIP tracefilename                        */
    ,char **jbmFECoffsetFileName /* o  : Output file  for Optimum FEC offset       */
);

void deindex_lvq_cng_fx(
    Word16 *index,                  /* i  : index to be decoded, as an array of 3 short */
    Word16 *x_lvq,                  /* o  : decoded codevector  Q9*/
    Word16 idx_cv,                  /* i  : relative mode_lvq, wrt START_CNG */
    Word16 no_bits,                 /* i  : number of bits for lattice */
    Word32 * p_offset_scale1,
    Word32 * p_offset_scale2,
    Word16 * p_no_scales
);

void CNG_dec_fx(
    Decoder_State_fx *st_fx,              /* i/o: State structure                          */
    const Word16 L_frame,               /* i  : length of the frame                 Q0   */
    Word16 Aq[],                  /* o  : LP coefficients                     Q12  */
    const Word32  core_brate,           /* i  : core bitrate                        Q0   */
    Word16 *lsp_new,              /* i/o: current frame LSPs                  Q15  */
    Word16 *lsf_new,              /* i/o: current frame LSFs                  Qlog2(2.56) */
    Word16 *allow_cn_step         /* o  : allow CN step                       Q0   */
    ,Word16 *sid_bw                /* o  : 0-NB/WB, 1-SWB SID                       */
    ,Word32 *q_env
);

void swb_CNG_dec_fx(
    Decoder_State_fx *st_fx,               /* i/o: State structure                          */
    const Word16 *synth_fx,            /* i  : ACELP core synthesis at 32kHz            */
    Word16 *shb_synth_fx,        /* o  : high-band CNG synthesis                  */
    const Word16 sid_bw,               /* i  : 0-NB/WB, 1-SWB SID                       */
    const Word16 Qsyn                  /* i  : Q value of ACELP core synthesis          */
);

void CNG_reset_enc_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure             */
    LPD_state   *mem,              /* i/o: acelp memories                      */
    Word16 *pitch_buf,            /* o  : floating pitch for each subframe    */
    Word16 *voice_factors         /* o  : voicing factors                     */
);

void CNG_reset_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure            */
    Word16 *pitch_buf,            /* o  : floating pitch for each subframe   */
    Word16 *voice_factors         /* o  : voicing factors                    */
);

void CNG_exc_fx(
    const Word32  core_brate,           /* i  : core bitrate                             */
    const Word16 L_frame,               /* i  : length of the frame                      */
    Word32 *Enew,                 /* i/o: decoded SID energy                   Q6  */
    Word16 *seed,                 /* i/o: random generator seed                    */
    Word16 exc[],                 /* o  : current non-enhanced excitation   Q_new  */
    Word16 exc2[],                /* o  : current enhanced excitation       Q_new  */
    Word32 *lp_ener,              /* i/o: LP filtered E                            */
    const Word32 last_core_brate,       /* i  : previous frame core bitrate              */
    Word16 *first_CNG,            /* i/o: first CNG frame flag for energy init.    */
    Word16 *cng_ener_seed,        /* i/o: random generator seed for CNG energy     */
    Word16 bwe_exc[],             /* o  : excitation for SWB TBE                   */
    const Word16 allow_cn_step,         /* i  : allow CN step                            */
    Word16 *last_allow_cn_step,   /* i/o: last allow step                          */
    const Word16 OldQ_exc,              /* i  : Old excitation scaling                   */
    const Word16 Q_exc,                 /* i  : excitation scaling                       */
    const Word16 num_ho                 /* i  : number of selected hangover frames       */
    ,Word32 q_env[]
    ,Word32 *lp_env
    ,Word32 *old_env
    ,Word16 *exc_mem
    ,Word16 *exc_mem1
    ,Word16 *sid_bw
    ,Word16 *cng_ener_seed1
    ,Word16 exc3[]
    ,Word16 Opt_AMR_WB
);

void CNG_enc_fx(
    Encoder_State_fx *st_fx,            /* i/o: State structure                                     */
    const Word16 L_frame,               /* i  : length of the frame                           Q0    */
    Word16 Aq[],                        /* o  : LP coefficients                               Q12   */
    const Word16 *speech,               /* i  : pointer to current frame input speech buffer  Q_new */
    Word32 L_enr,                       /* i  : residual energy from Levinson-Durbin          Q6    */
    Word16 *lsp_new,                    /* i/o: current frame ISPs                            Q15   */
    Word16 *lsf_new,                    /* i/o: current frame ISFs                            Qlog2(2.56) */
    Word16 *allow_cn_step,              /* o  : allow CN step                                 Q0    */
    Word16 burst_ho_cnt,                /* i  : hangover frames at end of speech burst        Q0    */
    Word16 Q_new                        /* i  : Q value of speech                                   */
    ,Word32 *q_env
    ,Word16 *sid_bw
    ,Word16 *exc_mem2
);

void swb_CNG_enc_fx(
    Encoder_State_fx *st_fx,                /* i/o: State structure                                 */
    const Word16 *shb_speech_fx,        /* i  : SHB target signal (6-14kHz) at 16kHz            */
    const Word16 *syn_12k8_16k_fx       /* i  : ACELP core synthesis at 12.8kHz or 16kHz        */
);

void cng_params_upd_fx(
    const Word16 lsp_new[],             /* i  : LSP parameters                                     Q15   */
    const Word16 exc2[],                /* i  : current enhanced excitation                        Q_exc */
    const Word16 L_frame,               /* i  : frame length                                       Q0    */
    Word16 *ho_circ_ptr,          /* i/o: pointer for CNG averaging buffers                  Q0    */
    Word32 ho_ener_circ[],        /* o  : energy buffer for CNG averaging                    Q6    */
    Word16 *ho_circ_size,         /* i/o: size of DTX hangover history buffer for averaging  Q0    */
    Word16 ho_lsp_circ[],         /* o  : old LSP buffer for CNG averaging                   Q15   */
    const Word16 Q_exc,                 /* i  : Q value of excitation                                    */
    const Word16 enc_dec_flag,          /* i  : Flag indicating encoder or decoder (ENC,DEC)             */
    Word32 ho_env_circ[],         /* i/o: Envelope buffer                                          */
    Word16 *cng_buf_cnt,          /* i/o: Counter of postponed FFT-processing instances            */
    Word16 cng_exc2_buf[],        /* i/o: Excitation buffer                                  Q_exc */
    Word16 cng_Qexc_buf[],        /* i/o: Q_exc buffer                                       Q0    */
    Word32 cng_brate_buf[],       /* i/o: last_active_brate buffer                           Q0    */
    const Word32 last_active_brate      /* i  : Last active bit rate                               Q0    */
);

void cng_params_postupd_fx(
    const Word16 ho_circ_ptr,           /* i  : pointer for CNG averaging buffers                  Q0    */
    Word16 *cng_buf_cnt,          /* i/o: counter for CNG store buffers                      Q0    */
    const Word16 *const cng_exc2_buf,   /* i  : Excitation buffer                                  Q_exc */
    const Word16 *const cng_Qexc_buf,   /* i  : Q_exc buffer                                       Q0    */
    const Word32 *const cng_brate_buf,  /* i  : bit rate buffer                                    Q0    */
    Word32 ho_env_circ[]          /* i/o: Envelope buffer                                          */
);
Word32 get_delay_fx(                    /* o  : delay value in ms                         */
    const Word16 what_delay,            /* i  : what delay? (ENC or DEC)                  */
    const Word32 io_fs                  /* i  : input/output sampling frequency           */
);


void signalling_enc_fx(
    Encoder_State_fx *st_fx,                /* i  : encoder state structure   */
    const Word16 coder_type,            /* i  : coder type                */
    const Word16 sharpFlag              /* i  : formant sharpening flag   */
);

Word16 signalling_mode1_tcx20_enc(
    Encoder_State_fx *st,               /* i  : encoder state structure                  */
    Word16 push
);

void signalling_enc_rf(
    Encoder_State_fx *st                /* i  : encoder state structure                   */
);

void decision_matrix_core_dec(
    Decoder_State_fx *st                  /* i/o: decoder state structure                   */
);

void decision_matrix_enc_fx(
    Encoder_State_fx *st_fx,                /* i  : encoder state structure                   */
    const Word16 sp_aud_decision1,      /* i  : 1st stage speech/music classification     */
    const Word16 sp_aud_decision2,      /* i  : 2nd stage speech/music classification     */
    const Word16 coder_type,            /* i  : coder type                                */
    const Word16 vad_flag,
    Word16 *hq_core_type          /* o  : HQ core_fx type                           */
);

void decision_matrix_dec_fx(
    Decoder_State_fx *st,                 /* i/o: decoder state structure                   */
    Word16 *coder_type,           /* o  : coder type                                */
    Word16 *sharpFlag,            /* o  : formant sharpening flag                   */
    Word16 *hq_core_type,         /* o  : HQ core type                              */
    Word16 *core_switching_flag   /* o  : ACELP->HQ switching frame flag            */
);

void init_gp_clip_fx(
    Word16 mem[]                        /* o  : memory of gain of pitch clipping algorithm */
);

void gp_clip_test_isf_fx(
    const Word16 isf[],                 /* i  : isf values (in frequency domain)           */
    Word16 mem[],                 /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 Opt_AMR_WB             /* i  : flag indicating AMR-WB IO mode             */
);

void pitch_ol_init_fx(
    Word16 *old_thres,              /* o  : threshold for reinforcement of past pitch influence */
    Word16 *old_pitch,              /* o  : pitch  of the 2nd half-frame of previous frame      */
    Word16 *delta_pit,              /* o  : pitch evolution extrapolation                       */
    Word16 *old_corr                /* o  : correlation                                         */
);

void noise_est_down_fx(
    const Word32 fr_bands[],            /* i  : per band input energy (contains 2 vectors) */
    Word32 bckr[],                /* i/o: per band background noise energy estimate  */
    Word32 tmpN[],                /* o  : temporary noise update                     */
    Word32 enr[],                 /* o  : averaged energy over both subframes        */
    const Word16 min_band,              /* i  : minimum critical band                      */
    const Word16 max_band,              /* i  : maximum critical band                      */
    Word16 *totalNoise,           /* o  : noise estimate over all critical bands     */
    Word16 Etot,                  /* i  : Energy of current frame                    */
    Word16 *Etot_last,            /* i/o: Energy of last frame            Q8         */
    Word16 *Etot_v_h2,            /* i/o: Energy variations of noise frames  Q8      */
    Word16 Q_new,
    const Word32 e_min                  /* i  : minimum energy scaled    Q_new + QSCALE    */
);

void wb_vad_init_fx(
    Word16 *nb_active_frames,           /* o  : nb of consecutive active speech frames */
    Word16 *hangover_cnt,
    Word16 *lp_speech,                  /* o  : long-term active speech level          */
    Word16 *nb_active_frames_he,   /* o  : nb of consecutive active speech frames */
    Word16 *hangover_cnt_he,
    Word16 *bcg_flux,              /* o  : background noise fluctuation                           */
    Word16 *soft_hangover,         /* o  : soft hangover counter                                  */
    Word16 *voiced_burst,          /* o  : consecutive voiced speech counter                      */
    Word16 *bcg_flux_init,         /* o  : initialization period for noise fluctuation estimation */
    Word16 *nb_active_frames_he1,  /* o  : nb of consecutive active speech frames 1               */
    Word16 *hangover_cnt_he1,
    Word32 *L_vad_flag_reg_H,
    Word32 *L_vad_flag_reg_L,
    Word32 *L_vad_prim_reg,
    Word16 *vad_flag_cnt_50,
    Word16 *vad_prim_cnt_16,
    Word16 *hangover_cnt_dtx,
    Word16 *hangover_cnt_music
);

Word16 wb_vad_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                     */
    const Word32 fr_bands[],            /* i  : per band input energy (contains 2 vectors)    Q_new+QSCALE*/
    Word16 *localVAD,
    Word16 *noisy_speech_HO,      /* o  : SC-VBR noisy speech HO flag                 */
    Word16 *clean_speech_HO,      /* o  : SC-VBR clean speech HO flag                 */
    Word16 *NB_speech_HO,         /* o  : SC-VBR NB speech HO flag                    */
    Word16 *snr_sum_he,           /* o  : Output snr_sum as weighted spectral measure */
    Word16 *localVAD_HE_SAD,      /* o  : HE_SAD decision without hangovers           */
    Word8 *flag_noisy_speech_snr, /* o  :   */
    const Word16 Q_new                  /* i  : scaling factor          Q0                  */
);


void dtx_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                  */
    const Word16 vad,                 /* i  : vad flag                                 */
    const Word16 speech[],            /* i  : Pointer to the speech frame              */
    Word16 Q_speech             /* i  : Q factor for speech                     */

);

void dtx_hangover_control_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    const Word16 lsp_new_fx[M]          /* i  : current frame LSPs                       */
);

Word16 dtx_hangover_addition_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure */
    const Word16  localVAD,             /* i   Q0   */
    const Word16  vad_flag,             /* i   Q0   */
    const Word16  lp_snr,               /* i   Q8  */
    const Word16   cldfb_subtraction,   /* i   Q0   */
    Word16 *vad_hover_flag_ptr
);

void noise_est_init_fx(
    Word16 *totalNoise,             /* o  : noise estimate over all critical bands     */
    Word16 *first_noise_updt,       /* o  : noise update initialization flag           */
    Word32 bckr[],                  /* o  : per band background noise energy estimate  */
    Word32 enrO[],                  /* o  : per band old input energy                  */
    Word32 ave_enr[],               /* o  : per band long-term average energies        */
    Word16 *pitO,                   /* o  : open-loop pitch values from preceed. frame */
    Word16 *aEn,                    /* o  : noise adaptation hangover counter          */
    Word16 *st_harm_cor_cnt,      /* i/o: 1st harm correlation timer                */
    Word16 *bg_cnt,               /* i/o: pause burst length counter                */
    Word16 *lt_tn_track,          /*  Q15 */
    Word16 *lt_tn_dist,           /*  Q8 */
    Word16 *lt_Ellp_dist,         /* Etot low lp same domain as  *Etot_l_lp, Q8 */
    Word16 *lt_haco_ev,           /*  Q15 */
    Word16 *low_tn_track_cnt      /* Q0  */
);


void noise_est_pre_fx(
    const Word16 Etot,           /* i  : Energy of current frame  */
    const Word16 ini_frame_fx,   /* i  : Frame number (init)      */
    Word16 *Etot_l,        /* i/o: Track energy from below  */ /*Q8 */
    Word16 *Etot_h,        /* i/o: Track energy from above  */ /*Q8 */
    Word16 *Etot_l_lp,     /* i/o: Smoothed low energy      */ /*Q8 */
    Word16 *Etot_last,     /* i/o: Energy of last frame     */ /*Q8 */
    Word16 *Etot_v_h2,     /* i/o: Energy variations        */ /*Q8*/
    Word16 *sign_dyn_lp,   /* i/o: Smoother signal dynamics */ /*Q8*/
    Word16 harm_cor_cnt,   /* i :  correlation counter */
    Word16 *Etot_lp        /* i/o: Smoothed  energy */
);

void Preemph_scaled(
    Word16 new_speech[],     /* i  : Speech to scale already on 14 bits*/
    Word16 *Q_new,           /* o  : Scaling factor                  */
    Word16 *mem_preemph,     /* i/o: Preemph memory                  */
    Word16 *Q_max,           /* i/o: Q_new limitation                */
    const Word16 Preemph_factor,   /* i  : Preemphasis factor         Q15  */
    const Word16 bits,             /* i  : Bit to remove from the output to (15-bits)*/
    const Word16 bit1,             /* i  : Limit the output scaling to ((15-bits)-bit1) bits  */
    const Word16 L_Q_mem,          /* i  : Number of old scaling to take into account  */
    const Word16 Lframe,           /* i  : Frame length                    */
    const Word16 last_coder_type,  /* i  : coder_type                      */
    const Word16 Search_scaling    /* i  : enable the search of a proper scaling factor*/
);
Word32 Scale_mem_pre_proc(                 /* o  : Min energy scaled           */
    Word16 ini_frame_fx,      /* i  : Frame number                */
    Word16 Q_exp,             /* i  : Diff scaling factor         */
    Word16 *Q_new,            /* i/o: Absolute scaling factor     */
    Word16 *old_speech,       /* i/o: Speech memory               */
    Word16 *mem_wsp,          /* i/o: wsp vector memory           */
    Word32 *enrO,             /* i/o: Enr mem                     */
    Word32 *bckr,             /* i/o: Back ground_fx ener mem     */
    Word32 *ave_enr,          /* i/o: Ave_enr mem                 */
    Word32 *ave_enr2,          /* i/o: Ave_enr2 mem                 */
    Word32 *st_fr_bands1,     /* i/o: spectrum per critical bands of the previous frame  */
    Word32 *st_fr_bands2,     /* i/o: spectrum per critical bands 2 frames ago           */
    Word32 *st_Bin_E_old
);
void Scale_mem_enc(
    Word16 Q_exp,             /* i  : Diff scaling factor         */
    Word16 *old_speech16k,    /* i/o: Speech memory               */
    Word16 *old_exc,          /* i/o: excitation memory           */
    Word16 *old_bwe_exc,      /* i/o: BWE excitation memory       */
    Word16 *mem_w0,           /* i/o: target vector memory        */
    Word16 *mem_syn,          /* i/o: synthesis memory            */
    Word16 *mem_syn2,         /* i/o: synthesis memory            */
    Word16 *mem_deemp_preQ_fx, /*i/o: deemphasis memory for the high rate celp codec */
    Word16 *last_exc_dct_in,
    Word16 *old_input_lp
);

void SetModeIndex(
    Encoder_State_fx *st,
    Word32 total_brate,
    Word16 bwidth,
    const Word16 shift
);

void init_encoder_fx(
    Encoder_State_fx *st_fx       /* i/o: Encoder static variables structure            */
);

void destroy_encoder_fx( Encoder_State_fx *st_fx );

void analysisCldfbEncoder_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure                    */
    const Word16 *timeIn,
    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word16 realBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word16 imagBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word32 enerBuffSum[CLDFB_NO_CHANNELS_MAX],
    Word16 *enerBuffSum_exp,
    CLDFB_SCALE_FACTOR * scale
);

void cldfb_save_memory (HANDLE_CLDFB_FILTER_BANK hs);     /* i/o: cldfb handle */
void cldfb_restore_memory (HANDLE_CLDFB_FILTER_BANK hs);  /* i/o: cldfb handle */
void cldfb_reset_memory (HANDLE_CLDFB_FILTER_BANK hs);      /* i/o: cldfb handle */

Word16 gp_clip_fx(
    const Word16 *voicing,            /* i  : normalized correlations (from OL pitch)    */
    const Word16 i_subfr,             /* i  : subframe index                             */
    const Word16 coder_type,          /* i  : type of coder                              */
    const Word16 xn[],                /* i  : target vector                              */
    Word16 mem[],               /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 Q_new                /* i  : scaling factor                             */
);

void init_decoder_fx(
    Decoder_State_fx *st_fx           /* o:   Decoder static variables structure */
);

void destroy_decoder(
    Decoder_State_fx *st_fx           /* o:   Decoder static variables structure */
);

Word16 Enr_1_Az_fx(                 /* o  : impulse response energy      Q5  */
    const Word16 Aq[],              /* i  : LP filter coefs              Qx based on the fact that Aq[0] == 1.0 */
    const Word16 len                /* i  : impulse response length      Q0  */
);
void FEC_scale_syn_fx(
    const Word16 L_frame,          /* i  : length of the frame                     */
    Word16 *update_flg,      /* o: flag indicating re-synthesis after scaling*/
    Word16 clas,             /* i/o: frame classification                    */
    const Word16 last_good,        /* i:   last good frame classification          */
    Word16 *synth,           /* i/o: synthesized speech at Fs = 12k8 Hz      */
    const Word16 *pitch,           /* i:   pitch values for each subframe          */
    Word32 L_enr_old,        /* i:   energy at the end of previous frame     */
    Word32 L_enr_q,          /* i:   transmitted energy for current frame    */
    const Word16 coder_type,       /* i:   coder type                              */
    const Word16 prev_bfi,         /* i:   previous frame BFI                      */
    const Word32 last_core_brate, /* i:   previous frame core bitrate             */
    Word16 *exc,             /* i/o: excitation signal without enhancement   */
    Word16 *exc2,            /* i/o: excitation signal with enhancement      */
    Word16 Aq[],             /* i/o: LP filter coefs (can be modified if BR) */
    Word16 *old_enr_LP,      /* i/o: LP filter E of last good voiced frame   */
    const Word16 *mem_tmp,         /* i:   temp. initial synthesis filter states   */
    Word16 *mem_syn,         /* o:   initial synthesis filter states         */
    Word16 Q_exc,
    Word16 Q_syn
);

void improv_amr_wb_gs_fx(
    const Word16 clas,                       /* i  : signal frame class                  */
    const Word16 coder_type,                 /* i  : coder type                          */
    const Word32 core_brate,                 /* i  : bitrate allocated to the core       */
    Word16 *seed_tcx,                  /* i/o: Seed used for noise generation      */
    Word16 *old_Aq_fx,                 /* i/o: old LPC filter coefficient          */
    Word16 *mem_syn2_fx,               /* i/o: synthesis memory                    */
    const Word16 lt_voice_fac_fx,            /* i/o: long term voice factor       Q15    */
    const Word16 locattack,                  /* i  : Flag for a detected attack          */
    Word16 *Aq_fx,                     /* i/o: Decoded LP filter coefficient       */
    Word16 *exc2_fx,                   /* i/o: Decoded complete excitation         */
    const Word16 Q_exc2,                     /* i  : Exponent of Exc2                    */
    Word16 *mem_tmp_fx,                /* i/o: synthesis temporary memory          */
    Word16 *syn_fx,                    /*   o: Decoded synthesis to be updated     */
    const Word16 Q_syn,                      /* i  : Synthesis scaling             Q0    */
    const Word16 *pitch_buf_fx,              /* i  : Decoded pitch buffer                */
    const Word16 Last_ener_fx,               /* i  : Last energy (Q8) */
    const Word16 last_coder_type_fx          /* i  : Last coder_type */
);

void FEC_pitch_estim_fx(
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode             */
    const Word16 L_frame,                    /* i  : length of the frame                        */
    const Word16 clas,                       /* i  : current frame classification               */
    const Word16 last_good,                  /* i  : last good clas information                 */
    const Word16 pitch_buf[],                /* i  : Floating pitch   for each subframe         */
    const Word32 old_pitch_buf[],            /* i  : buffer of old subframe pitch values 15Q16  */
    Word16 *bfi_pitch,                 /* i/o: update of the estimated pitch for FEC      */
    Word16 *bfi_pitch_frame,           /* o  : frame length when pitch was updated        */
    Word16 *upd_cnt,                   /* i/o: update counter                             */
    const Word16 coder_type                  /* i  : coder_type                                 */
);

Word16 div_s_ss(                            /* o: result of division (Word16 Q0) */
    const Word16 n,                         /* i: numerator   (Word16 Q0         */
    const Word16 d                          /* i: denominator (Word16 Q0)        */
);

Word16 detect_transient_fx(
    const Word16 *in_fx, /*Q_new */
    /*Encoder_State *st, */
    const Word16 L,
    const Word16 coder_type,                /* i  : coder type               */
    Word16 Q_new,
    Encoder_State_fx *st_fx
);

Word16 detect_transient_fx32(               /* o  : transient flag           */
    const Word32* in_fx,                    /* i  : input signal             */
    Encoder_State_fx *st_fx,                /* i/o: Encoder state structure  */
    const Word16 L,                         /* i  : length (32 or 48kHz)     */
    const Word16 coder_type                 /* i  : coder type               */
);

void ar_encoder_start_fx( PARCODEC_FX arInst, PBITSTREAM_FX bsInst );
void ar_encoder_done_fx( PARCODEC_FX arInst );
void ar_decoder_start_fx( PARCODEC_FX arInst, PBITSTREAM_FX bsInst );
void ar_decoder_done_fx( PARCODEC_FX arInst );

void srt_vec_ind_fx (const Word32 *linear, Word32 *srt, Word16 *I, Word16 length);
Word32 ar_div(Word32 num, Word32 denum);

Word16 GetScale_fx( Word16 blen, Word32 bits_fx, Word32 *surplus_fx);
Word32 encode_position_ari_fx(PARCODEC_FX parenc, Word16* quants, Word16 size, Word32* est_bits_frame_fx);
Word32 encode_magnitude_usq_fx(ARCODEC_FX* parenc, Word16* magn, Word16 size, Word16 npulses, Word16 nzpos, Word32* est_frame_bits_fx);
Word32 encode_magnitude_tcq_fx(ARCODEC_FX* parenc, Word16* magn, Word16 size, Word16 npulses, Word16 nzpos, Word32* savedstates, Word32* est_frame_bits_fx);
Word32 GetISCScale_fx( Word32 *quants_fx, Word16 size, Word32 bits_fx, Word16 *magn_fx, Word32 *qscale_fx, Word32 *surplus_fx, Word16 *pulses, Word32* savedstates, Word32 noTCQ, Word16 *nzpout
                       , Word16 *bcount, Word32 *abuffer, Word16 *mbuffer, Word32 *sbuffer );
Word32 encode_signs_fx(ARCODEC_FX* parenc, Word16* magn, Word16 size, Word16 npos, Word32* est_frame_bits_fx);

void decode_position_ari_fx(PARCODEC_FX pardec, Word16 size, Word16 npulses, Word16* nz, Word16* position);
void decode_magnitude_usq_fx(ARCODEC_FX* pardec, Word16 size, Word16 npulses, Word16 nzpos, Word16* positions, Word16* out);
void decode_mangitude_tcq_fx(ARCODEC_FX* pardec, Word16 size, Word16 npulses, Word16 nzpos, Word16* positions, Word16* out, Word32* surplus_fx);
void decode_signs_fx(ARCODEC_FX* pardec, Word16 size, Word16* out);

void tcq_core_LR_enc_fx(
    Encoder_State_fx* st_fx,
    Word16   inp_vector_fx[],
    const Word32 coefs_norm_fx[],
    Word32 coefs_quant_fx[],
    const Word16 bit_budget,  /* number of bits */
    const Word16 nb_sfm,
    const Word16 *sfm_start,
    const Word16 *sfm_end,
    const Word16 *sfmsize,
    Word32 *R_fx,
    Word16 *npulses,
    Word16 *k_sort,
    const Word16 *p2a_flags,
    const Word16 p2a_bands,
    const Word16 *last_bitalloc,
    const Word16 input_frame,
    const Word16 adjustFlag,
    const Word16 is_transient
);

void tcq_core_LR_dec_fx(
    Decoder_State_fx *st_fx,
    Word16 *inp_vector_fx,
    const Word16 bit_budget,
    const Word16 bands,
    const Word16 *band_start,
    const Word16 *band_width,
    Word32 *Rk_fx,
    Word16 *npulses,
    Word16 *k_sort,
    const Word16 *p2a_flags,
    const Word16 p2a_bands,
    const Word16 *last_bitalloc,
    const Word16 input_frame,
    const Word16 adjustFlag,
    const Word16 *is_transient
);

void TCQLSB_fx(
    Word16 bcount,
    Word32 *abuffer_fx,
    Word16 *mbuffer_fx,
    Word32 *sbuffer_fx,
    Word16 *dpath
);

void TCQLSBdec_fx(
    Word16 *dpath,
    Word16 *mbuffer,
    Word16 bcount
);

void RestoreTCQ_fx(
    Word16 * magn,
    Word16 size,
    Word16 *bcount,
    Word16 *mbuffer
);

void RestoreTCQdec_fx(
    Word16 * magn,
    Word16 size,
    Word16 *bcount,
    Word16 *mbuffer
);

void InitLSBTCQ_fx(
    Word16 *bcount
);

void SaveTCQdata_fx(
    PARCODEC_FX arInst,
    Word16 *dpath,
    Word16 bcount
);

void LoadTCQdata_fx(
    PARCODEC_FX arInst,
    Word16 *dpath,
    Word16 bcount
);

void bit_allocation_second_fx(
    Word32 *Rk,
    Word32 *Rk_sort,
    Word16  BANDS,
    const Word16 *band_width,
    Word16 *k_sort,
    Word16 *k_num,
    const Word16 *p2a_flags,
    const Word16  p2a_bands,
    const Word16 *last_bitalloc,
    const Word16  input_frame
);

void window_ola_fx(
    Word32 *ImdctOut,
    Word16 *auOut,
    Word16 *Q_sig,
    Word16 *OldauOut,
    Word16 *Q_old,
    const Word16 L,
    const Word16 right_mode,
    const Word16 left_mode,
    const Word16 old_bfi,
    const Word16 oldHqVoicing,
    Word16 *oldgapsynth
);

void tcx_get_windows_mode1(
    Word16 left_mode,                   /* i: overlap mode of left window half          */
    Word16 right_mode,                  /* i: overlap mode of right window half         */
    Word16  *left_win,                  /* o: left overlap window                       */
    Word16  *right_win,                 /* o: right overlap window                      */
    Word16  *left_win_int,              /* o: left overlap window                       */
    Word16  *right_win_int,             /* o: right overlap window                      */
    Word16 const L
);

void sinq_fx(
    const Word16 tmp,                           /* i  : sinus factor cos(tmp*i+phi)  Q15*/
    const Word16 phi,                           /* i  : sinus phase cos(tmp*i+phi)  Q15*/
    const Word16 N,                             /* i  : size of output */
    Word16 x[]                                  /* o  : output vector  Q15*/
);

Word16 FEC_phase_matching_fx(
    Decoder_State_fx *st_fx,                    /* i  : Decoder State                           */
    Word32 *ImdctOut_fx,                        /* i  : input                                   */
    Word16 *auOut_fx,                           /* o  : output audio                            */
    Word16 *OldauOut_fx,
    Word16 OldauOut_pha_fx[2][N_LEAD_NB]
);

void FEC_phase_matching_nextgood_fx(
    const Word32 *ImdctOut_fx,                  /* i  : input                           */
    Word16 *auOut_fx,                     /* o  : output audio                    */
    Word16 *OldauOut_fx,                  /* i/o: audio from previous frame       */
    Word16 OldauOut_pha_fx[2][N_LEAD_NB],
    Word16 mean_en_high_fx /* Q5 */
);

void FEC_phase_matching_burst_fx(
    const Word32 *ImdctOut_fx,                  /* i  : input                           */
    Word16 *auOut_fx,                     /* o  : output audio                    */
    Word16 *OldauOut_fx,                  /* i/o: audio from previous frame       */
    Word16 OldauOut_pha_fx[2][N_LEAD_NB],
    Word16 *prev_oldauOut_fx              /* i : OldauOut from previous frame     */
);

void Repetition_smoothing_nextgood_fx(
    const Word32 *ImdctOut_fx,                  /* i  : input                           */
    Word16 *auOut_fx,                     /* o  : output audio                    */
    Word32 *OldImdctOut_fx,               /* i  : input                           */
    Word16 *OldauOut_fx,                  /* i/o: audio from previous frame       */
    Word16 cur_data_use_flag,             /* i  : current imdct data use flag     */
    Word16 overlap_time
);

Word16 Repetition_smoothing_fx(
    const Word32 *ImdctOut_fx,                  /* i  : input                                   */
    Word16 *auOut_fx,                     /* o  : output audio                            */
    Word32 *OldImdctOut_fx,               /* i/o: audio from previous frame               */
    Word16 *OldauOut_fx,                  /* i/o: audio from previous frame               */
    const Word16 L,                             /* i  : length                                  */
    Word16 *prev_oldauOut_fx,             /* i/o: IMDCT output from previous old frame    */
    Word16 overlap_time                   /* i : overlapping time                         */
);

void time_domain_FEC_HQ_fx(
    Decoder_State_fx *st_fx,                    /* i  : Decoder State                           */
    Word32 *wtda_audio_fx,                /* i  : input                                   */
    Word16 *out_fx,                       /* o : output audio                             */
    Word16 mean_en_high_fx,               /* i  : transient flag                          */
    const Word16 output_frame,
    Word16 *Q_synth
);

void common_overlapping_fx(
    Word16 *auOut_fx,                     /* i : Input                                   */
    Word16 *ImdctOutWin_fx,               /* o : Output                                  */
    Word16 *OldauOut_fx,                  /* i : Window                                  */
    Word16 end1,                          /* i : Decay                                   */
    Word16 offset1,
    Word16 start2,
    Word16 end2,
    Word16 offset_i2,
    Word16 offset2
);

void Smoothing_vector_scaledown_NB_fx(
    const Word16 OldauOutnoWin_fx[],            /* i  : Input vector 1                                  */
    const Word16 ImdctOutWin_fx[],              /* i  : Input vector 2                                  */
    const Word16 SmoothingWin_fx[],             /* i  : Smoothing window                                */
    Word16 auOut_fx[],                    /* o  : Output vector that contains vector 1 .* vector 2*/
    const Word16 ol_size                        /* i  : Overlap size                                    */
);

void Scaledown_fx(
    Word16 x[],                           /* i  : Input vector                                    */
    Word16 y[],                           /* o  : Output vector that contains vector 1 .* vector 2*/
    Word16 scale_v,
    const Word16 N                              /* i  : Overlap size                                    */
);

void Smoothing_vector_NB_fx(
    const Word16 OldauOutnoWin_fx[],            /* i  : Input vector 1                                   */
    const Word16 ImdctOutWin_fx[],              /* i  : Input vector 2                                   */
    const Word16 SmoothingWin_fx[],             /* i  : Smoothing window                                 */
    Word16 auOut_fx[],                    /* o  : Output vector that contains vector 1 .* vector 2 */
    const Word16 ol_size                        /* i  : Overlap size                                     */
);

void Windowing_1st_NB_fx(
    Word16 *ImdctOutWin_fx,               /* o : Output                                  */
    const Word32 *ImdctOut_fx,                  /* i : Input                                   */
    const Word16 *win_fx,                       /* i : Window                                  */
    const Word16 *smoothingWin_fx,              /* i : Smoothing Window                        */
    Word16 smoothing_flag                 /* i : 1=Smoothing window, 0=Original window   */
);

void Windowing_2nd_NB_fx(
    Word16 *ImdctOutWin_fx,               /* o : Output                   */
    const Word32 *ImdctOut_fx,                  /* i : Input                    */
    const Word16 *win_fx                        /* i : Window                   */
);

void Next_good_after_burst_erasures_fx(
    const Word32 *ImdctOut_fx,
    Word16 *auOut_fx,
    Word16 *OldauOut_fx,
    const Word16 ol_size
);

void subband_gain_bits_fx(
    const Word16 *Rk,                       /* i  : bit allocation per band Q3 */
    const Word16 N,                         /* i  : number of bands         */
    Word16 *bits,                     /* o  : gain bits per band      */
    const Word16 *sfmsize                   /* i  : Size of bands           */
);

Word16 assign_gain_bits_fx(                 /* o  : Number of assigned gain bits          */
    const Word16 core,                      /* i  : HQ core                               */
    const Word16 BANDS,                     /* i  : Number of bands                       */
    const Word16 *band_width,               /* i  : Sub band bandwidth                    */
    Word16 *Rk,                       /* i/o: Bit allocation/Adjusted bit alloc. Q3 */
    Word16 *gain_bits_array,          /* o  : Assigned gain bits                    */
    Word16 *Rcalc                     /* o  : Bit budget for shape quantizer     Q3 */
);

void apply_gain_fx(
    const Word16 *ord,                      /* i  : Indices for energy order                       */
    const Word16 *band_start,               /* i  : Sub band start indices                         */
    const Word16 *band_end,                 /* i  : Sub band end indices                           */
    const Word16 num_sfm,                   /* i  : Number of bands                                */
    const Word16 *gains,                    /* i  : Band gain vector                       Q12     */
    Word16 *xq                        /* i/o: Float synthesis / Gain adjusted synth  Q15/Q12 */
);

void fine_gain_pred_fx(
    const Word16 *sfm_start,                /* i  : Sub band start indices          */
    const Word16 *sfm_end,                  /* i  : Sub band end indices            */
    const Word16 *sfm_size,                 /* i  : Sub band bandwidths             */
    const Word16 *i_sort,                   /* i  : Energy sorting indices          */
    const Word16 *K,                        /* i  : Number of pulses per band       */
    const Word16 *maxpulse,                 /* i  : Maximum pulse per band          */
    const Word16 *R,                        /* i  : Bits per sub band           Q3  */
    const Word16 num_sfm,                   /* i  : Number of sub bands             */
    Word16 *xq,                       /* i/o: Quantized vector /quantized vector with finegain adj Q15*/
    Word16 *y,                        /* i/o: Quantized vector (int)          */
    Word16 *fg_pred,                  /* o  : Predicted fine gains        Q12 */
    const Word16 core                       /* i  : Core                            */
);

void fine_gain_quant_fx(
    Encoder_State_fx *st_fx,
    const Word16 *ord,                       /* i  : Indices for energy order                     */
    const Word16 num_sfm,                    /* i  : Number of bands                              */
    const Word16 *gain_bits,                 /* i  : Gain adjustment bits per sub band            */
    Word16 *fg_pred,                   /* i/o: Predicted gains / Corrected gains        Q12 */
    const Word16 *gopt                       /* i  : Optimal gains                            Q12 */
);

void fine_gain_dec_fx
(
    Decoder_State_fx *st,
    const Word16 *ord,                       /* i  : Indices for energy order             */
    const Word16 num_sfm,                    /* i  : Number of bands                      */
    const Word16 *gain_bits,                 /* i  : Gain adjustment bits per sub band    */
    Word16 *fg_pred                    /* i/o: Predicted gains / Corrected gains    */
);

void get_max_pulses_fx(
    const Word16 *band_start,               /* i  : Sub band start indices    */
    const Word16 *band_end,                 /* i  : Sub band end indices      */
    const Word16 *k_sort,                   /* i  : Indices for sorting by energy */
    const Word16 *npulses,                  /* i  : Pulses per sub band       */
    const Word16  BANDS,                    /* i  : Number of bands           */
    Word16 *inp_vector,               /* i/o: Encoded shape vectors (int)*/
    Word16 *maxpulse                  /* o  : Maximum pulse height per band */
);

Word16 pvq_core_enc_fx(
    Encoder_State_fx *st_fx,
    Word16 coefs_norm[],
    Word16 coefs_quant[],
    Word16 *Q_coefs,
    Word16 bits_tot,                  /* total number of bits */
    Word16 nb_sfm,
    const Word16 *sfm_start,
    const Word16 *sfm_end,
    const Word16 *sfmsize,
    Word16 *R,    /* Q3 */
    Word16 *Rs,
    Word16 *npulses,
    Word16 *maxpulse,
    const Word16 core
);

void pvq_encode_frame_fx(
    Encoder_State_fx *st_fx,
    const Word16 *coefs_norm,       /* i  : normalized coefficients to encode */
    Word16 Q_coefs,
    Word16 *coefs_quant,      /* o  : quantized coefficients */
    Word16 *gopt,             /* o  : optimal shape gains */
    Word16 *npulses,          /* o  : number of pulses per band */
    Word16 *pulse_vector,     /* o  : non-normalized pulse shapes */
    const Word16 *sfm_start,        /* i  : indices of first coefficients in the bands */
    const Word16 *sfm_end,          /* i  : indices of last coefficients in the bands */
    const Word16 *sfmsize,          /* i  : band sizes */
    const Word16 nb_sfm,            /* i  : total number of bands */
    const Word16 *R,                /* i  : bitallocation per band */
    const Word16 pvq_bits,          /* i  : number of bits avaiable */
    const Word16 core               /* i  : core */
);

void encode_energies_fx(
    Encoder_State_fx *st_fx,
    const Word16 *coefs,
    const Word16 Q_coefs,
    Word16 Np,
    Word16 *dim_part,
    Word32 *E_part,      /* 32-bit Q15 */
    Word16 *bits_part,
    Word16 *g_part,      /* Q15 */
    Word16 bits,
    Word16 *bits_left,
    Word32 enr,          /* 32-bit Q15 */
    Word16 dim,
    const Word16 strict_bits
);

Word16 pvq_core_dec_fx(
    Decoder_State_fx *st_fx,
    const Word16 *sfm_start,
    const Word16 *sfm_end,
    const Word16 *sfmsize,
    Word16 coefs_quant[],               /* o  : output   MDCT     */
    Word16 *Q_coefs,
    Word16 bits_tot,
    Word16 nb_sfm,
    Word16 *R,
    Word16 *Rs,
    Word16 *npulses,
    Word16 *maxpulse,
    const Word16 core
);

void pvq_decode_frame_fx(
    Decoder_State_fx *st_fx,
    Word16 *coefs_quant,      /* o  : quantized coefficients */
    Word16 *npulses,          /* o  : number of pulses per band */
    Word16 *pulse_vector,     /* o  : non-normalized pulse shapes */
    const Word16 *sfm_start,        /* i  : indices of first coefficients in the bands */
    const Word16 *sfm_end,          /* i  : indices of last coefficients in the bands */
    const Word16 *sfmsize,          /* i  : band sizes */
    const Word16 nb_sfm,            /* i  : total number of bands */
    const Word16 *R,                /* i  : bitallocation per band */
    const Word16 pvq_bits,          /* i  : number of bits avaiable */
    const Word16 core               /* i  : core */
);

void decode_energies_fx(
    Decoder_State_fx *st_fx,
    Word16 Np,
    Word16 *dim_part,
    Word16 *bits_part,
    Word16 *g_part,     /* Q15 */
    Word16 bits,
    Word16 *bits_left,
    Word16 dim,
    const Word16 strict_bits
);

Word16 atan2_fx(
    const Word32,
    const Word32
);

void srt_vec_ind16_fx (
    const Word16 *linear,       /* linear input */
    Word16 *srt,          /* sorted output*/
    Word16 *I,            /* index for sorted output  */
    Word16 length
);


PvqEntry_fx mpvq_encode_vec_fx(     /* o : leading_sign_index, index, size, k_val        */
    const Word16* vec_in,      /* i : signed pulse train        */
    Word16        dim_in,      /* i : dimension                 */
    Word16        k_val_local  /* i : nb unit pulses            */
);

PvqEntry_fx get_size_mpvq_calc_offset_fx( /* o : size, dim, k_val   */
    Word16  dim_in,            /* i : dimension                */
    Word16 k_val_in,           /* i : nb unit pulses           */
    UWord32* h_mem             /* o : offsets                  */
);

void mpvq_decode_vec_fx(            /* o :  void                        */
    const PvqEntry_fx* entry,      /* i :  sign_ind, index, dim, k_val */
    UWord32* h_mem,                /* i :  A/U offsets                 */
    Word16* vec_out                /* o :  pulse train                 */
);

void rc_enc_init_fx(
    Encoder_State_fx *st_fx,        /* i/o: Encoder state       */
    Word16 tot_bits                 /* i  : Total bit budget    */
);

void rc_encode_fx(
    Encoder_State_fx *st_fx,        /* i/o: Encoder state                       */
    UWord32 cum_freq,               /* i  : Cumulative frequency up to symbol   */
    UWord32 sym_freq,               /* i  : Symbol probability                  */
    UWord32 tot                     /* i  : Total cumulative frequency          */
);

void rc_enc_uniform_fx(
    Encoder_State_fx *st_fx,        /* i/o: Encoder state       */
    UWord32 value,                  /* i  : Value to encode     */
    UWord32 tot                     /* i  : Maximum value       */
);

void rc_enc_bits_fx(
    Encoder_State_fx *st_fx,        /* i/o: Encoder state       */
    UWord32 value,                  /* i  : Value to encode     */
    Word16 bits                     /* i  : Number of bits used */
);

void rc_enc_finish_fx(
    Encoder_State_fx *st_fx         /* i/o: Encoder state       */
);

Word16 rc_get_bits2_fx(             /* o: Number of bits needed         */
    const Word16 N,                 /* i: Number of bits currently used */
    const UWord32 range             /* i: Range of range coder          */
);

Word16 rc_get_bits_f2_fx(           /* o: Number of bits needed in Q3   */
    const Word16 N,                 /* i: Number of bits currently used */
    const UWord32 range             /* i: Range of range coder          */
);

UWord32 rc_decode_fx(               /* o  : Decoded cumulative frequency    */
    Decoder_State_fx *st_fx,        /* i/o: Decoder State                   */
    UWord32 tot                     /* i  : Total cumulative frequency      */
);

void rc_dec_update_fx(
    Decoder_State_fx *st_fx,        /* i/o: Decoder State           */
    UWord32 cum_freq,               /* i  : Cumulative frequency    */
    UWord32 sym_freq                /* i  : Symbol frequency        */
);

UWord32 rc_dec_uniform_fx(          /* i  : Decoded value   */
    Decoder_State_fx *st_fx,        /* i/o: Decoder State   */
    UWord32 tot                     /* i  : Maximum value   */
);

void rc_dec_init_fx(
    Decoder_State_fx *st_fx,        /* i/o: Decoder State       */
    Word16 tot_bits                 /* i  : Total bit budget    */
);

Word32 rc_dec_bits_fx(              /* i  : Decoded value   */
    Decoder_State_fx *st_fx,        /* i/o: Decoder State   */
    Word16 bits                     /* i  : Number of bits  */
);

void rc_dec_finish_fx(
    Decoder_State_fx *st_fx
);

UWord32 UL_inverse(
    const UWord32 UL_val,
    Word16 *exp
);

UWord32 UL_div(
    const UWord32 UL_num,
    const UWord32 UL_den
);

void hq_core_enc_fx(
    Encoder_State_fx *st_fx,
    const Word16 *audio,          /* i  : input audio signal             Q0  */
    const Word16 input_frame_orig,/* i  : frame length                        */
    const Word16 hq_core_type,    /* i  : HQ core type                        */
    const Word16 Voicing_flag
);

void normalizecoefs_fx(
    Word32 *coefs,                     /* i  : Input vector (Q12)                  */
    const Word16 *ynrm,                      /* i  : quantization indices for norms      */
    const Word16 num_bands,                  /* i  : Number of bands                     */
    const Word16 *band_start,                /* i  : Start of bands                      */
    const Word16 *band_end,                  /* i  : End of bands                        */
    Word16 *coefs_norm                 /* o  : Normalized output vector (Q12)      */
);

void calc_norm_fx(
    const Word32 *L_x,                       /* i  : Input vector.                   Qx  */
    const Word16 qx,                         /* i  : Q value of input                    */
    Word16 *norm,                      /* o  : Quantization indices for norms  Q0  */
    Word16 *normlg,                    /* o  : Quantized norms in log2         Q0  */
    const Word16 start_band,                 /* i  : Indice of band to start coding  Q0  */
    const Word16 num_bands,                  /* i  : Number of bands                 Q0  */
    const Word16 *band_len,                  /* i  : Length of bands                 Q0  */
    const Word16 *band_start                 /* i  : Start of bands                  Q0  */
);

void diff_envelope_coding_fx(
    const Word16 is_transient,       /* i  : transient indicator               Q0  */
    const Word16 num_env_bands,      /* i  : number of envelope bands to code  Q0  */
    const Word16 start_norm,         /* i  : start of envelope coding          Q0  */
    Word16 *ynrm,              /* i/o: quantization indices for norms    Q0  */
    Word16 *normqlg2,          /* i/o: quantized norms                   Q0  */
    Word16 *difidx             /* o  : differential code                 Q0  */
);

void reordernorm_fx(
    const Word16 *ynrm,             /* i  : quantization indices for norms     Q0 */
    const Word16 *normqlg2,         /* i  : quantized norms                    Q0 */
    Word16 *idxbuf,           /* o  : reordered quantization indices     Q0 */
    Word16 *normbuf,          /* o  : reordered quantized norms          Q0 */
    const Word16 nb_sfm             /* i  : number of bands                    Q0 */
);

void hq_hr_enc_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure fx          */
    Word32 *t_audio,           /* i/o: transform-domain coefficients  Q12? */
    const Word16 length,             /* i  : length of spectrum             Q0   */
    Word16 *num_bits,          /* i  : number of available bits       Q0   */
    const Word16 is_transient        /* i  : transient flag                 Q0   */
);

Word16 hq_classifier_enc_fx(        /* o  : Consumed bits                   Q0  */
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure             */
    const Word16 length,            /* i  : Frame length                    Q0  */
    const Word32 *coefs,            /* i  : Spectral coefficients           Q12 */
    const Word16 is_transient,      /* i  : Transient flag                  Q0  */
    Word16 *Npeaks,           /* o  : Number of identified peaks      Q0  */
    Word16 *peaks,            /* o  : Peak indices                    Q0  */
    Word32 *pe_gains,         /* o  : Peak gains                      Q12 */
    Word32 *nf_gains,         /* o  : Noise-fill gains                Q12 */
    Word16 *hqswb_clas        /* o  : HQ class                        Q0  */
);

void hvq_classifier_fx(
    const Word32 *input,             /* i  : input signal                Q12 */
    Word16 *prev_Npeaks,       /* i/o: Peak number memory          Q0  */
    Word16 *prev_peaks,        /* i/o: Peak indices memory         Q0  */
    Word16 *hqswb_clas,        /* i/o: HQ class                    Q0  */
    Word16 *Npeaks,            /* o  : Number of peaks             Q0  */
    Word16 *peaks,             /* o  : Peak indices                Q0  */
    const Word32 L_core_brate,       /* i  : Core bit-rate               Q0  */
    const Word16 last_core,          /* i  : Last core used              Q0  */
    Word32 *L_nf_gains,        /* o  : Noisefloor gains            Q12 */
    Word16 *hvq_hangover,      /* i/o: Mode-switch hangover        Q0  */
    Word32 *L_pe_gains         /* o  : peak gains                  Q12 */
);

void limit_band_noise_level_calc_fx(
    const Word16 *wnorm,             /* i  : reordered norm of sub-vectors        */
    Word16 *limit,             /* o  : highest band of bit allocation       */
    const Word32 core_brate,         /* i  : bit rate                             */
    Word16 *noise_level        /* o  : noise level Q15                      */
);

Word16 BitAllocWB_fx(         /* o  : t                                           Q0*/
    Word16 *y,                /* i  : norm of sub-vectors                         Q0*/
    Word16 B,                 /* i  : number of available bits                    Q0*/
    Word16 N,                 /* i  : number of sub-vectors                       Q0*/
    Word16 *R,                /* o  : bit-allocation indicator                    Q0*/
    Word16 *Rsubband_fx       /* o  : sub-band bit-allocation vector              Q3*/
);

void hq_wb_nf_bwe_fx(
    const Word16 *coeff_fx,          /* i  : coded/noisefilled normalized spectrum */
    const Word16 is_transient,
    const Word16 prev_bfi,           /* i  : previous bad frame indicator    */
    const Word32 *L_normq_v,
    const Word16 num_sfm,            /* i  : Number of subbands              */
    const Word16 *sfm_start,         /* i  : Subband start coefficient       */
    const Word16 *sfm_end,           /* i  : Subband end coefficient         */
    const Word16 *sfmsize,           /* i  : Subband band width              */
    const Word16 last_sfm,           /* i  : last coded subband              */
    const Word16 *R,                 /* i  : bit allocation                  */
    const Word16 prev_is_transient,  /* i  : previous transient flag         */
    Word32 *prev_normq_fx,     /* i/o: previous norms                  */
    Word32 *prev_env_fx,       /* i/o: previous noise envelopes        */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input */
    Word32 *prev_coeff_out_fx, /* i/o: decoded spectrum in previous frame */
    Word16 *prev_R,            /* i/o: bit allocation info. in previous frame */
    Word32 *L_coeff_out,       /* o  : coded/noisefilled spectrum      */
    Word16 *prev_env_Q
);

void harm_bwe_fine_fx(
    const Word16 *R,                 /* i  : bit allocation                              */
    const Word16 last_sfm,           /* i  : last coded subband                          */
    const Word16 high_sfm,           /* i  : higher transition band to BWE               */
    const Word16 num_sfm,            /* i  : total number of bands                       */
    const Word16 *norm,              /* i  : quantization indices for norms              */
    const Word16 *sfm_start,         /* i  : Subband start coefficient                   */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                     */
    Word16 *prev_L_swb_norm,   /* i/o: last normalize length                       */
    Word16 *coeff,             /* i/o: coded/noisefilled normalized spectrum       */
    Word32 *coeff_out,         /* o  : coded/noisefilled spectrum                  */
    Word16 *coeff_fine         /* o  : BWE fine structure                          */
);

void harm_bwe_fx(
    const Word16 *coeff_fine,        /* i  : fine structure for BWE                  */
    const Word16 *coeff,             /* i  : coded/noisefilled normalized spectrum   */
    const Word16 num_sfm,            /* i  : Number of subbands                      */
    const Word16 *sfm_start,         /* i  : Subband start coefficient               */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                 */
    const Word16 last_sfm,           /* i  : last coded subband                      */
    const Word16 *R,                 /* i  : bit allocation                          */
    const Word16 prev_hq_mode,       /* i  : previous hq mode                        */
    Word16 *norm,              /* i/o: quantization indices for norms          */
    Word16 *noise_level,       /* i/o: noise levels for harmonic modes         */
    Word16 *prev_noise_level,  /* i/o: noise factor in previous frame          */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input    */
    Word32 *coeff_out          /* o  : coded/noisefilled spectrum              */
);

void hq_generic_encoding_fx(
    const Word32 *coefs_fx,             /* i  : MDCT coefficients of weighted original      */
    Word16 *hq_generic_fenv_fx,   /* i/o: energy of SWB envelope                      */
    const Word16 hq_generic_offset,     /* i  : frequency offset for extracting energy      */
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                     */
    Word16 *hq_generic_exc_clas   /* o  : bwe excitation class                        */
);

void hq_generic_fine_fx(
    Word16 *coeff,                  /* i  : coded/noisefilled normalized spectrum   */
    const Word16 last_sfm,          /* i  : Last coded band                         */
    const Word16 *sfm_start,        /* i  : Subband start coefficient               */
    const Word16 *sfm_end,          /* i  : Subband end coefficient                 */
    Word16 *bwe_seed,               /* i/o: random seed for generating BWE input    */
    Word16 *coeff_out1              /* o  : HQ Generic input                        */
);

void hq_bwe_fx(
    const Word16 HQ_mode,           /* i  : HQ mode                                     */
    Word32 *coeff_out1,             /* i/o: BWE input & temporary buffer                */
    const Word16 *hq_generic_fenv,  /* i  : SWB frequency envelopes                     */
    Word32 *coeff_out,              /* o  : SWB signal in MDCT domain                   */
    const Word16 hq_generic_offset, /* i  : frequency offset for representing hq generic bwe*/
    Word16 *prev_L_swb_norm,        /*i/o : last normalize length                       */
    const Word16 hq_generic_exc_clas,/* i  : bwe excitation class                        */
    const Word16 *sfm_end,          /* i  : End of bands                                */
    const Word16 num_sfm,
    const Word16 num_env_bands,
    const Word16 *R
);

void hq_generic_decoding_fx(
    const Word16 HQ_mode,               /* i  : HQ mode                                     */
    Word32 *coeff_out1_fx,        /* i/o: BWE input & temporary buffer                */
    const Word16 *hq_generic_fenv_fx,   /* i  : SWB frequency envelopes                     */
    Word32 *coeff_out_fx,         /* o  : SWB signal in MDCT domain                   */
    const Word16 hq_generic_offset,     /* i  : frequency offset for representing hq generic*/
    Word16 *prev_L_swb_norm,      /* i/o: last normalize length                       */
    const Word16 hq_generic_exc_clas    /* i  : bwe excitation class                        */
);


void map_hq_generic_fenv_norm_fx(
    const Word16 hqswb_clas,
    const Word16 *hq_generic_fenv,               /* Q1, frequency-domain BWE envelope */
    Word16 *ynrm,
    Word16 *normqlg2,
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 hq_generic_offset
);

Word16 get_nor_delta_hf_fx(
    Decoder_State_fx *st,
    Word16 *ynrm,
    Word16 *Rsubband,                       /* Q3 */
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 core_sfm
);

Word16 calc_nor_delta_hf_fx(
    Encoder_State_fx *st,
    const Word32 *t_audio,
    Word16 *ynrm,
    Word16 *Rsubband,
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 *sfmsize,
    const Word16 *sfm_start,
    const Word16 core_sfm
);

void HQ_FEC_processing_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure                          */
    Word32 *t_audio_q_fx,             /* o  : MDCT coeffs. (for synthesis)              Q12    */
    Word16 is_transient,              /* i  : Old flag for transient                           */
    Word32 ynrm_values_fx[][MAX_PGF], /* i  : Old average Norm values for each group of bands Q12 */
    Word32 r_p_values_fx[][MAX_ROW],  /* i  : Computed y-intercept and slope by Regression    Q5 */
    Word16 num_Sb,                    /* i  : Number of sub-band group                         */
    Word16 nb_sfm,                    /* i  : Number of sub-band                               */
    Word16 *Num_bands_p,              /* i  : Number of coeffs. for each sub-band              */
    Word16 output_frame,              /* i  : Frame size                                       */
    const Word16 *sfm_start,                /* i  : Start of bands                                   */
    const Word16 *sfm_end,                  /* i  : End of bands                                     */
    Word16 output_frame_org           /* i  : Original Frame size                              */
);

void HQ_FEC_Mem_update_fx(
    Decoder_State_fx *st_fx,    /* i/o: decoder state structure            */
    Word32 *t_audio_q_fx,           /*Q12*/
    Word32 *normq_fx,               /*Q14*/
    Word16 *ynrm,
    Word16 *Num_bands_p,
    Word16 is_transient,
    Word16 hqswb_clas,
    Word16 c_switching_flag,
    Word16 nb_sfm,
    Word16 num_Sb,
    Word16 *mean_en_high_fx,      /*Q5*/
    Word16 hq_core_type,              /* i : normal or low-rate MDCT(HQ) core */
    Word16 output_frame
);

Word16 BitAllocF_fx (
    Word16 *y,                  /* i  : norm of sub-vectors                      :Q0  */
    Word32  bit_rate,           /* i  : bitrate                                  :Q0  */
    Word16 B,                   /* i  : number of available bits                 :Q0  */
    Word16 N,                   /* i  : number of sub-vectors                    :Q0  */
    Word16 *R,                  /* o  : bit-allocation indicator                 :Q0  */
    Word16 *Rsubband_fx,        /* o  : sub-band bit-allocation vector           :Q3  */
    const Word16 hqswb_clas,    /* i  : hq swb class                                  */
    const Word16 num_env_bands  /* i  : Number sub bands to be encoded for HQ_SWB_BWE  */
);

void hq_bit_allocation_fx(
    const Word32 core_brate,     /* i  : Core bit-rate                      */
    const Word16 length,         /* i  : Frame length                       */
    const Word16 hqswb_clas,     /* i  : HQ class                           */
    Word16 *num_bits,      /* i/o: Remaining bit budget               */
    const Word16 *normqlg2,      /* i  : Quantized norms                    */
    const Word16 nb_sfm,         /* i  : Number sub bands to be encoded     */
    const Word16 *sfmsize,       /* i  : Sub band bandwidths                */
    Word16 *noise_level,   /* o  : HVQ noise level                    */
    Word16 *R,             /* o  : Bit allocation per sub band        */
    Word16 *Rsubband,      /* o  : Fractional bit allocation          */
    Word16 *sum,           /* o  : Sum of allocated shape bits        */
    Word16 *core_sfm,      /* o  : Last coded band in core            */
    const Word16 num_env_bands   /* i  : Number sub bands to be encoded for HQ_SWB_BWE  */
);

void hvq_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure          */
    const Word16  num_bits,        /* i : Number of available bits          */
    const Word32  core_brate,      /* i : Core bit-rate                     */
    const Word16 *ynrm,            /* i  : Envelope coefficients            */
    Word16 *R,               /* i/o: Bit allocation/updated bit allocation */
    Word16 *noise_level,     /* o : Noise level in Q15                */
    Word16 *peak_idx,        /* o : Peak position vector              */
    Word16 *Npeaks,          /* o : Total number of peaks             */
    Word32 *coefsq_norm,     /* o : Output vector in Q12              */
    const Word16  core
);

void peak_vq_dec_fx(
    Decoder_State_fx *st_fx,         /* i/o: decoder state structure       */
    Word32 *coefs_out,        /* o  : Output coefficient vector Q12 */
    const Word16 brate,             /* i  : Core bitrate                  */
    const Word16 num_bits,          /* i  : Number of bits for HVQ        */
    const Word16 *ynrm,         /* i  : Envelope coefficients         */
    Word16 *R,            /* i/o: Bit allocation/updated bit allocation */
    Word16 *vq_peak_idx,      /* o  : Peak position vector          */
    Word16 *Npeaks,           /* o  : Number of peaks               */
    const Word16 core
);

void hq_pred_hb_bws_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure                 */
    const Word16 *ynrm,                  /* i  : norm quantization index vector          */
    const Word16 length,                 /* i  : frame length                            */
    const Word16 hqswb_clas,             /* i  : HQ SWB class                            */
    const Word16 *SWB_fenv               /* i  : SWB frequency envelopes             Q1  */
);
void hq_core_dec_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure fx         */
    Word16 synth[],                 /* o  : output synthesis                   */
    Word16 *Q_synth,                /* o  : Q value of synth                   */
    const Word16 output_frame,            /* i  : output frame length                */
    const Word16 hq_core_type,            /* i  : HQ core type                       */
    const Word16 core_switching_flag      /* i  : ACELP->HQ switching frame flag     */
);

void hq_hr_dec_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure fx                     */
    Word32 *L_coefsq,              /* o  : transform-domain coefficients              Q12 */
    const Word16 length,                 /* i  : frame length                               Q0  */
    Word16 num_bits,               /* i  : number of available bits                   Q0  */
    Word16 *ynrm,                  /* o  : norm quantization index vector             Q0  */
    Word16 *is_transient,          /* o  : transient flag                             Q0  */
    Word16 *hqswb_clas,            /* o  : HQ SWB class                               Q0  */
    Word16 *SWB_fenv               /* o  : SWB frequency envelopes                    Q1  */
);

void dequantize_norms_fx(
    const Word16 start_norm,         /* i  : First SDE encoded norm            */
    const Word16 num_sfm,            /* i  : Number of norms                   */
    const Word16 is_transient,       /* i  : Transient flag                    */
    Word16 *ynrm,                    /* o  : Decoded norm indices              */
    Word16 *normqlg2                 /* o  : Log2 of decoded norms             */
);

void huff_dec_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure                         */
    const Word16 N,                  /* i  : Number of codewords to decode                   */
    const Word16 buffer_len,         /* i  : Number of bits to read                          */
    const Word16 num_lengths,        /* i  : Number of different huffman codeword lengths    */
    const Word16 *thres,             /* i  : Threshold of first codeword of each length      */
    const Word16 *offset,            /* i  : Offset for first codeword                       */
    const Word16 *huff_tab,          /* i  : Huffman table order by codeword lengths         */
    Word16 *index              /* o  : Decoded index                                   */
);

Word16 hq_classifier_dec_fx(         /* o  : Consumed bits                   Q0 */
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure            */
    const Word32  core_brate,        /* i  : Core bit rate                   Q0 */
    const Word16 length,             /* i  : Frame length                    Q0 */
    Word16 *is_transient,      /* o  : Transient flag                  Q0 */
    Word16 *hqswb_clas         /* o  : HQ class                        Q0 */
);

void hq_configure_bfi_fx(
    Word16 *nb_sfm,             /* o  : Number of sub bands              Q0 */
    Word16 *num_Sb,             /* o  : Number of FEC sub bands ?        Q0 */
    Word16 *num_bands_p,        /* o  : FEC sub bands                    Q0 */
    Word16 const **sfmsize,     /* o  : Subband bandwidths                */
    Word16 const **sfm_start,   /* o  : Subband start coefficients        */
    Word16 const **sfm_end      /* o  : Subband end coefficients          */
);

void hq_swb_harmonic_calc_norm_envelop_fx(
    Word32 *L_SWB_signal,         /* i  : input signal                Q=12*/
    Word32 *L_envelope,           /* o  : output envelope             Q=12*/
    Word16 L_swb_norm,            /* i  : length of normaliztion          */
    Word16 SWB_flength            /* i  : length of input signal          */
);

Word16 build_nf_codebook_fx(      /* o  : Number of coefficients in nf codebook */
    const Word16 flag_32K_env_ho, /* i  : Envelope attenuation hangover flag */
    const Word16 *coeff,          /* i  : Coded spectral coefficients   */
    const Word16 *sfm_start,      /* i  : Subband start indices         */
    const Word16 *sfmsize,        /* i  : Subband widths                */
    const Word16 *sfm_end,        /* i  : Subband end indices           */
    const Word16 last_sfm,        /* i  : Last coded  band              */
    const Word16 *R,              /* i  : Per-band bit allocation       */
    Word16 *CodeBook,       /* o  : Noise-fill codebook           */
    Word16 *CodeBook_mod    /* o  : Densified noise-fill codebook */
);

Word16 find_last_band_fx(        /* o  : index of last band              */
    const Word16 *bitalloc,      /* i  : bit allocation                  */
    const Word16 nb_sfm         /* i  : number of possibly coded bands  */
);

void apply_noisefill_HQ_fx(
    const Word16 *R,             /* i  : bit allocation                  */
    const Word16 length,         /* i  : input frame length              */
    const Word16 flag_32K_env_ho,/* i  : envelope stability hangover flag*/
    const Word32 L_core_brate,   /* i  : core bit rate                   */
    const Word16 last_sfm,       /* i  : last coded subband              */
    const Word16 *CodeBook,      /* i  : Noise-fill codebook             */
    const Word16 *CodeBook_mod,  /* i  : Densified noise-fill codebook   */
    const Word16 cb_size,        /* i  : Codebook length                 */
    const Word16 *sfm_start,     /* i  : Subband start coefficient       */
    const Word16 *sfm_end,       /* i  : Subband end coefficient         */
    const Word16 *sfmsize,       /* i  : Subband band width              */
    Word16 *coeff          /* i/o: coded/noisefilled spectrum      */
);

void hvq_bwe_fine_fx(
    const Word16 last_sfm,           /* i  : last coded subband                          */
    const Word16 num_sfm,            /* i  : total number of bands                       */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                     */
    const Word16 *peak_idx,          /* i  : Peak index                                  */
    const Word16 Npeaks,             /* i  : Number of peaks                             */
    Word16 *peak_pos,          /* o  : Peak positions                              */
    Word16 *prev_L_swb_norm,   /* i/o: last normalize length                       */
    Word32 *L_coeff,           /* i/o: coded/noisefilled normalized spectrum       */
    Word16 *bwe_peaks,         /* o  : Positions of peaks in BWE                   */
    Word16 *coeff_fine         /* o  : HVQ BWE fine structure                      */
);

void hq_fold_bwe_fx(
    const Word16 last_sfm,           /* i  : last coded subband                     Q0 */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                Q0 */
    const Word16 num_sfm,            /* i  : Number of subbands                     Q0 */
    Word16 *coeff                    /* i/o: coded/noisefilled normalized spectrum  Q12 */
);

void apply_nf_gain_fx(
    const Word16 nf_idx,             /* i  : noise fill gain index                   Q0 */
    const Word16 last_sfm,           /* i  : last coded subband                      Q0 */
    const Word16 *R,                 /* i  : bit allocation                          Q0 */
    const Word16 *sfm_start,         /* i  : Subband start coefficient               Q0 */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                 Q0 */
    Word16 *coeff              /* i/o: coded/noisefilled normalized spectrum   Q12 */
);

void logqnorm_fx(
    const Word32 *x,                /* i : coefficient vector                         Qx  */
    const Word16 qx,                /* i : Q value of input                               */
    Word16 *k,                /* o : index                                      Q0  */
    const Word16 L,                 /* i : codebook length                            Q0  */
    const Word16 N,                 /* i : sub-vector size                            Q0  */
    const Word16 hvq_flag           /* i : HVQ flag                                   Q0  */
);

void logqnorm_2_fx(
    const Word32 *env_fl,            /* o  : index */
    const Word16 L,                  /* i  : codebook length */
    const Word16 n_env_band,         /* i  : sub-vector size */
    const Word16 nb_sfm,             /* i  : sub-vector size */
    Word16 *ynrm,
    Word16 *normqlg2,
    const Word32 *thren              /* i  : quantization thresholds */
);

void hvq_bwe_fx(
    const Word32 *coeff,
    const Word16 *coeff_fine,        /* i  : coded/noisefilled normalized spectrum  Qin */
    const Word16 *sfm_start,         /* i  : Subband start coefficient              Q0  */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                Q0  */
    const Word16 *sfm_len,           /* i  : Subband length                         Q0  */
    const Word16 last_sfm,           /* i  : last coded subband                     Q0  */
    const Word16 prev_hq_mode,       /* i  : previous hq mode                       Q0  */
    const Word16 *bwe_peaks,         /* i  : HVQ bwe peaks                          Q0  */
    const Word16 bin_th,             /* i  : HVQ transition bin                     Q0  */
    const Word16 num_sfm,            /* i  : Number of bands                        Q0  */
    const Word32 core_brate,         /* i  : Core bit-rate                          Q0  */
    const Word16 *R,                 /* i  : Bit allocation                             */
    Word16 *norm,              /* i/o: quantization indices for norms         Q0  */
    Word16 *noise_level,       /* i/o: noise levels for harmonic modes        Q15 */
    Word16 *prev_noise_level,  /* i/o: noise factor in previous frame         Q15 */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input   Q0  */
    Word32 *L_coeff_out,       /* o  : coded/noisefilled spectrum             Qout*/
    const Word16 qin,
    const Word16 qout
);

void hvq_concat_bands_fx
(
    const Word16 pvq_bands,          /* i  : Number of bands in concatenated PVQ target  */
    const Word16 *sel_bnds,          /* i  : Array of selected high bands                */
    const Word16 n_sel_bnds,         /* i  : Number of selected high bands               */
    Word16 *hvq_band_start,    /* i  : Band start indices                          */
    Word16 *hvq_band_width,    /* i  : Band widths                                 */
    Word16 *hvq_band_end       /* i  : Band end indices                            */
);

void noise_mix_fx(
    const Word16 *coeff_fine,        /* i  : normalized fine structure spectrum     Qin */
    const Word32 L_E,                /* i  : normalization factor                   Q17 */
    const Word32 L_normq,            /* i  : quantized norm                         Q14 */
    Word16 *seed,              /* i/o: random seed                            Q0  */
    const Word16 istart,             /* i  : start coefficient                      Q0  */
    const Word16 iend,               /* i  : end coefficient                        Q0  */
    const Word16 noise_level,        /* i  : noise_level                            Q0  */
    Word32 *L_coeff_out,       /* o  : noisemixed spectrum                    Qout */
    const Word16 qin,
    const Word16 qout
);

Word16 find_coding_range_fx(        /* o  : Number of bits used               Q0 */
    Encoder_State_fx *st_fx,         /* i/o: decoder state structure */
    const Word16 length,            /* i  : Frame length                      Q0 */
    const Word32 core_brate,        /* i  : bit rate                          Q0 */
    const Word16 num_env_bands,     /* i  : Number of envelope bands          Q0 */
    const Word16 *normqlg2,         /* i  : Quantized norms                   Q0 */
    Word16 *nb_sfm,           /* o  : Number of coded sub bands         Q0 */
    const Word16 hqswb_clas         /* i  : HQ core class                     Q0 */
);

Word16 noise_adjust_fx(             /* o  : index of noise attenuation     Q0  */
    const Word16 *coeffs_norm,      /* i  : normalized coefficients        Qx  */
    const Word16 qx,                /* i  : Q value of coeffs_norm             */
    const Word16 *bitalloc,         /* i  : bit allocation                 Q0  */
    const Word16 *sfm_start,        /* i  : band start                     Q0  */
    const Word16 *sfm_end,          /* i  : band end                       Q0  */
    const Word16 core_sfm           /* i  : index of the end band for core Q0  */
);

Word16 peak_vq_enc_fx(
    Encoder_State_fx *st_fx,         /* i/o: encoder state structure */
    const Word32 *coefs,            /* i  : Input coefficient vector Q14 */
    Word32 *coefs_out,        /* o  : Quantized output vector Q14 */
    const Word32 brate,             /* i  : Core bitrate */
    const Word16 num_bits,          /* i  : Number of bits for HVQ */
    const Word16 vq_peaks,          /* i  : Number of identified peaks */
    const Word16 *ynrm,             /* i  : Envelope coefficients          */
    Word16 *R,                /* i/o: Bit allocation/updated bit allocation */
    Word16 *vq_peak_idx,      /* i  : Peak index vector */
    Word32 *nf_gains          /* i  : Estimated noise floor gains Q14 */
);

Word16 hvq_enc_fx(                  /* o  : Consumed bits */
    Encoder_State_fx *st_fx,         /* i/o: encoder state structure */
    const Word32 brate,             /* i  : Total bit rate */
    const Word16 hvq_bits,          /* i  : HVQ bit budget */
    const Word16 Npeaks,            /* i  : Number of peaks */
    const Word16 *ynrm,             /* i  : Envelope coefficients          */
    Word16 *R,                /* i/o: Bit allocation/updated bit allocation */
    Word16 *peaks,            /* i  : Peak pos. / Encoded peak pos. */
    Word32 *nf_gains,         /* i/o: Noise fill gains / Quant. nf gains */
    Word16 *noise_level,      /* o  : Quantized noise level */
    const Word32 *pe_gains,         /* i  : Peak gains */
    const Word32 *coefs,            /* i  : spectrum coefficients in Q14 */
    Word32 *coefs_out         /* o  : encoded spectrum coefficients in Q14 */
);

void decode_coding_range_fx(
    Decoder_State_fx *st_fx,      /* i/o: decoder state structure */
    const Word16 length,         /* i  : Frame length                    Q0 */
    Word16 *num_bits,      /* i/o: Number of bits                  Q0 */
    Word16 *nb_sfm,        /* o  : Number of coded sub bands       Q0 */
    const Word16 hqswb_clas      /* i  : HQ BWE class                    Q0 */
);

void enforce_zero_for_min_envelope_fx(
    const Word16 hqswb_clas,     /* i  : HQ coding mode                     Q0  */
    const Word16 *ynrm,          /* i  : Envelope indices                   Q0  */
    Word32 *L_coefsq,      /* i/o: Quantized spectrum/zeroed spectrum Q12 */
    const Word16 nb_sfm,         /* i  : Number of coded sub bands          Q0  */
    const Word16 *sfm_start,     /* i  : Sub band start indices             Q0  */
    const Word16 *sfm_end        /* i  : Sub band end indices               Q0  */
);

void apply_envelope_fx(
    const Word16 *coeff,             /* i  : Coded/noisefilled normalized spectrum      Q12 */
    const Word16 *norm,              /* i  : Envelope                                */
    const Word16 *norm_adj,          /* i  : Envelope adjustment                        Q15 */
    const Word16 num_sfm,            /* i  : Total number of bands                   */
    const Word16 last_sfm,           /* i  : Last coded band                         */
    const Word16 HQ_mode,            /* i  : HQ mode                                 */
    const Word16 length,             /* i  : Frame length                            */
    const Word16 *sfm_start,         /* i  : Sub band start indices                  */
    const Word16 *sfm_end,           /* i  : Sub band end indices                    */
    Word32 *normq_v,           /* o  : Envelope with adjustment                   Q14 */
    Word32 *coeff_out,         /* o  : coded/noisefilled spectrum              */
    const Word16 *coeff1,            /* i  : coded/noisefilled spectrum                 Q12 */
    Word32 *coeff_out1         /* o  : coded/noisefilled spectrum                 Q12 */
);

void bitalloc_fx (
    Word16 *y,                /* i  : reordered norm of sub-vectors              Q0 */
    Word16 *idx,              /* i  : reordered sub-vector indices               Q0 */
    Word16 sum,               /* i  : number of available bits                   Q0 */
    Word16 N,                 /* i  : number of norms                            Q0 */
    Word16 K,                 /* i  : maximum number of bits per dimension       Q0 */
    Word16 *r,                /* o  : bit-allacation vector                      Q0 */
    const Word16 *sfmsize,          /* i  : band length                                Q0 */
    const Word16 hqswb_clas         /* i  : signal classification flag                 Q0 */
);

void bitallocsum_fx(
    Word16 *R,            /* i  : bit-allocation vector                         Q0 */
    const Word16 nb_sfm,        /* i  : number of sub-vectors                         Q0 */
    Word16 *sum,          /* o  : total number of bits allocated                Q0 */
    Word16 *Rsubband,     /* o  : rate per subband                              Q3 */
    const Word16 v,             /* i  : bit rate                                      Q0 */
    const Word16 length,        /* i  : length of spectrum (32 or 48 kHz samplerate)  Q0 */
    const Word16 *sfmsize       /* i  : band length                                   Q0 */
);

void env_adj_fx
(
    const Word16 *pulses,           /* i  : number of pulses per band           Q0  */
    const Word16 length,            /* i  : length of spectrum                  Q0  */
    const Word16 last_sfm,          /* i  : index of the last band              Q0  */
    Word16 *adj,              /* o  : adjustment factors for the envelope Q15 */
    const Word16 env_stab,          /* i  : envelope stability                  Q15 */
    const Word16 *sfmsize           /* i  : subband sizes                       Q0  */
);

Word16 env_stability_fx(            /* in Q15 */
    const Word16 *ynrm,             /*i:   Norm vector for current frame */
    const Word16 nb_sfm,            /*i:   Number of sub-bands    */
    Word16 *mem_norm,         /*i/o: Norm vector memory from past frame */
    Word16 *mem_env_delta     /*i/o: Envelope stability memory for smoothing in Q11 */
);

Word16 env_stab_smo_fx(             /* Q0 */
    Word16 env_stab,               /*i  : env_stab value                        Q15 */
    Word16 *env_stab_state_p,      /*i/o: env_stab state probabilities          Q15 */
    Word16 *ho_cnt                 /*i/o: hangover counter for speech state      */
);

void env_stab_transient_detect_fx(
    const Word16 is_transient,         /* i:   Transient flag                                           */
    const Word16 length,               /* i  : Length of spectrum (32 or 48 kHz)                        */
    const Word16 norm[],               /* i  : quantization indices for norms                           */
    Word16 *no_att_hangover,     /* i/o: Frame counter for attenuation hangover            (Q0)   */
    Word32 *L_energy_lt,         /* i/o: Long-term energy measure for transient detection  (Q13)  */
    const Word16 HQ_mode,              /* i  : HQ coding mode                                           */
    const Word16 bin_th,               /* i  : HVQ cross-over frequency bin                             */
    const Word32 *L_coeff,             /* i  : Coded spectral coefficients                              */
    const Word16 Qcoeff                /* i  : Q of coded spectral coefficients                         */
);

void hq_configure_fx(
    const Word16 length,              /* i  : Frame length                      Q0 */
    const Word16 hqswb_clas,          /* i  : HQ SWB class                      Q0 */
    const Word32 core_brate,          /* i  : Codec bitrate                     Q0 */
    Word16 *num_sfm,            /* o  : Total number of subbands          Q0 */
    Word16 *nb_sfm,             /* o  : Total number of coded bands       Q0 */
    Word16 *start_norm,         /* o  : First norm to be SDE encoded      Q0 */
    Word16 *num_env_bands,      /* o  : Number coded envelope bands       Q0 */
    Word16 *numnrmibits,        /* o  : Number of bits in fall-back norm encoding  Q0 */
    Word16 *hq_generic_offset,  /* o  : Freq offset for HQ GENERIC        Q0 */
    Word16 const **sfmsize,     /* o  : Subband bandwidths                Q0 */
    Word16 const **sfm_start,   /* o  : Subband start coefficients        Q0 */
    Word16 const **sfm_end      /* o  : Subband end coefficients          Q0 */
);

Word16 hvq_pvq_bitalloc_fx(
    Word16 num_bits,     /* i/o: Number of available bits (including gain bits) */
    const Word32 brate,        /* i  : bitrate                     */
    const Word16 bwidth_fx,    /* i  : Encoded bandwidth           */
    const Word16 *ynrm,        /* i  : Envelope coefficients       */
    const Word32 manE_peak,    /* i  : Peak energy mantissa        */
    const Word16 expE_peak,    /* i  : Peak energy exponent        */
    Word16 *Rk,          /* o  : bit allocation for concatenated vector */
    Word16 *R,           /* i/o: Global bit allocation       */
    Word16 *sel_bands,   /* o  : Selected bands for encoding */
    Word16 *n_sel_bands  /* o  : No. of selected bands for encoding */
);

void disf_ns_28b_fx(
    Word16 *indice,
    Word16 *isf_q
);

void isf_enc_amr_wb_fx(
    Encoder_State_fx *st,        /* i/o: state structure                             */
    Word16 *isf_new,     /* o  : quantized ISF vector                        */
    Word16 *isp_new,     /* i/o: ISP vector to quantize/quantized            */
    Word16 *Aq,          /* o  : quantized A(z) for 4 subframes              */
    Word16 clas,         /* i  : signal class                                */
    Word16 *stab_fac     /* o  : ISF stability factor                        */
);

void pvq_encode_fx(
    Encoder_State_fx *st_fx,
    const Word16 *x,            /* i:   vector to quantize            Q15-3=>Q12    */
    Word16 *y,                  /* o:   raw pulses  (non-scaled short)   Q0 */
    Word16  *xq,                /* o:   quantized vector               Q15     */
    Word32 *L_xq,               /* o:   quantized vector               Q31     */
    const Word16 pulses,        /* i:   number of allocated pulses       */
    const Word16 dim,           /* i:   Length of vector                 */
    const Word16 neg_gain       /* i:  - Gain       use - negative gain in  Q15    0 ..1            */
);

void pvq_decode_fx(
    Decoder_State_fx *st_fx,
    Word16 *xq,           /* o:   decoded vector (Q15)    */
    Word16 *y,            /* o:   decoded vector (non-scaled int)  */
    const Word16 k_val,         /* i:   number of allocated pulses       */
    const Word16 dim,           /* i:   Length of vector                 */
    const Word16 neg_gain       /* i:   Gain    (negated to fit 1.0 in Q15 as -1.0)      */
);

void de_interleave_spectrum_fx(
    Word32 *coefs,        /* i/o: input and output coefficients   Q?  */
    const Word16 length         /* i  : length of spectrum              Q0  */
);

void interleave_spectrum_fx(
    Word32 *coefs,        /* i/o: input and output coefficients   Q?  */
    const Word16 length         /* i  : length of spectrum              Q0  */
);

void recovernorm_fx(
    Word16 *idxbuf,             /* i  : reordered quantization indices  */
    Word16 *ynrm,               /* o  : recovered quantization indices  */
    Word16 *normqlg2,           /* o  : recovered quantized norms       */
    Word16 nb_sfm               /* i  : number of SFMs                  */
);

void map_quant_weight_fx(
    const Word16 normqlg2[],    /* i  : quantized norms                 */
    Word16 wnorm[],       /* o  : weighted norm                   */
    const Word16 is_transient,  /* i  : transient flag                  */
    const Word16 nb_sfm         /* i  : number of norms                 */
);

void fill_spectrum_fx(
    Word16 *coeff,               /* i/o: normalized MLT spectrum / nf spectrum                Q12 */
    Word32 *L_coeff_out,         /* i/o: Noisefilled MLT spectrum                             Q12 */
    const Word16 *R,                   /* i  : number of pulses per band                            Q0  */
    const Word16 is_transient,         /* i  : transient flag                                       Q0  */
    Word16 norm[],               /* i  : quantization indices for norms                       Q0  */
    const Word16 *hq_generic_fenv,     /* i  : HQ GENERIC envelope                                  Q1  */
    const Word16 hq_generic_offset,    /* i  : HQ GENERIC offset                                    Q0  */
    const Word16 nf_idx,               /* i  : noise fill index                                     Q0  */
    const Word16 length,               /* i  : Length of spectrum (32 or 48 kHz)                    Q0  */
    const Word16 env_stab,             /* i  : Envelope stability measure [0..1]                    Q15 */
    Word16 *no_att_hangover,     /* i/o: Frame counter for attenuation hangover               Q0  */
    Word32 *L_energy_lt,         /* i/o: Long-term energy measure for transient detection     Q13 */
    Word16 *bwe_seed,            /* i/o: random seed for generating BWE input                 Q0  */
    const Word16 hq_generic_exc_clas,  /* i  : BWE excitation class                                 Q0  */
    const Word16 core_sfm,             /* i  : index of the end band for core                       Q0  */
    const Word16 HQ_mode,              /* i  : HQ mode                                              Q0  */
    Word16 noise_level[],        /* i  : noise levels for harmonic modes                      Q15 */
    const Word32 L_core_brate,         /* i  : target bit-rate                                      Q0  */
    Word16 prev_noise_level[],   /* i/o: noise factor in previous frame                       Q15 */
    Word16 *prev_R,              /* i/o: bit allocation info. in previous frame               Q0  */
    Word32 *prev_coeff_out,      /* i/o: decoded spectrum in previous frame                   Q12 */
    const Word16 *peak_idx,            /* i  : peak indices for hvq                                 Q0  */
    const Word16 Npeaks,               /* i  : number of peaks in hvq                               Q0  */
    const Word16 *npulses,             /* i  : number of pulses per band                            Q0  */
    const Word16 prev_is_transient,    /* i  : previous transient flag                              Q0  */
    Word32 *prev_normq,          /* i/o: previous norms                                       Q14 */
    Word32 *prev_env,            /* i/o: previous noise envelopes                             Q(prev_env_Q) */
    const Word16 prev_bfi,             /* i  : previous bad frame indicator                         Q0  */
    const Word16 *sfmsize,             /* i  : Length of bands                                      Q0  */
    const Word16 *sfm_start,           /* i  : Start of bands                                       Q0  */
    const Word16 *sfm_end,             /* i  : End of bands                                         Q0  */
    Word16 *prev_L_swb_norm,     /* i/o: HVQ/Harmonic mode normalization length               Q0  */
    const Word16 prev_hq_mode,         /* i  : Previous HQ mode                                     Q0  */
    const Word16 num_sfm,              /* i  : Total number of bands                                Q0  */
    Word16 *prev_env_Q,
    const Word16 num_env_bands
);

Word16 FEC_pos_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 L_frame,           /* i  : length of the frame                     */
    const Word16 coder_type,        /* i  : coder type                              */
    const Word16 last_good,         /* i  : last good classfication                 */
    Word16 *last_pulse_pos,   /* o  : last glotal pulse position in the lost ACB */
    Word16 *clas,             /* o  : decoded classification                  */
    Word32 *enr_q,            /* o  : decoded energy     in Q0                 */
    const Word32  core_brate        /* i  : decoded bitrate                         */
);

void pit16k_Q_dec_fx(
    const Word16  pitch_index,  /* i  : pitch index                             */
    const Word16  nBits,        /* i  : # of Q bits                             */
    const Word16  limit_flag,   /* i  : restrained(0) or extended(1) limits     */
    Word16  *T0,          /* o  : integer pitch lag                       */
    Word16  *T0_frac,     /* o  : pitch fraction                          */
    Word16  *T0_min,      /* i/o: delta search min                        */
    Word16  *T0_max       /* i/o: delta search max                        */
);

Word16 frame_ener_fx(
    const Word16 L_frame,       /* i  : length of the frame                            */
    const Word16 clas,      /* i  : frame classification                           */
    const Word16 *synth,    /* i  : synthesized speech at Fs = 12k8 Hz       Q_new */
    const Word16 pitch,     /* i  : pitch period                             Q0    */
    Word32 *enr_q,    /* o  : pitch-synchronous or half_frame energy   Q0    */
    const Word16 offset,    /* i  : speech pointer offset (0 or L_FRAME)           */
    const Word16 Q_new,     /* i  : Scaling factor                                 */
    Word16 shift,     /* i  : Shift need to obtain 12 bits vectors           */
    const Word16 enc        /* i  : Encoder/decoder                                */
);

Word16 frame_energy_fx(       /* o  : Frame erergy in                               Q8 */
    Word16 L_frame,
    const Word16 *pitch,       /* i  : pitch values for each subframe                Q6 */
    const Word16 *speech,      /* i  : pointer to speech signal for E computation  Q_syn*/
    const Word16 lp_speech,    /* i  : long term active speech energy average      Q8   */
    Word16 *frame_ener,  /* o  : pitch-synchronous energy at frame end       Q8   */
    const Word16 Q_syn         /* i  : Synthesis scaling                                */
);

void int_lsp_fx(
    const Word16 L_frame,     /* i  : length of the frame               */
    const Word16 lsp_old[],   /* i  : LSPs from past frame              */
    const Word16 lsp_new[],   /* i  : LSPs from present frame           */
    Word16 *Aq,         /* o  : LP coefficients in both subframes */
    const Word16 m,           /* i  : order of LP filter                */
    const Word16 clas,        /* i  : signal frame class                */
    const Word16 *int_coeffs, /* i  : interpolation coefficients        */
    const Word16 Opt_AMR_WB   /* i  : flag indicating AMR-WB IO mode    */
);

void reorder_isf_fx(
    Word16 *isf,          /* i/o: ISFs in the frequency domain (0..0.5)   */
    const Word16 min_dist,      /* i  : minimum required distance               */
    const Word16 n,             /* i  : LPC order                               */
    const Word16 fs             /* i  : sampling frequency                      */
);

void reorder_lsf_fx(
    Word16 *lsf,          /* i/o: LSFs in the frequency domain (0..0.5)   Q(x2.56)*/
    const Word16 min_dist,      /* i  : minimum required distance               x2.56*/
    const Word16 n,             /* i  : LPC order                               */
    const Word32 fs               /* i  : sampling frequency                      */
);

void FEC_synchro_exc_fx(
    const Word16 L_frame,           /* i  : length of the frame                               */
    Word16 *exc,                /* i/o: exc vector to modify                              */
    const Word16 desire_puls_pos,   /* i  : Pulse position send by the encoder                */
    const Word16 true_puls_pos,     /* i  : Present pulse location                            */
    const Word16 Old_pitch          /* i  : Pitch use to create temporary adaptive codebook   */
);

Word16 findpulse_fx(                /* o  : pulse position                */
    const Word16 L_frame,             /* i  : length of the frame   */
    const Word16 res[],             /* i  : Residual signal     <12 bits  */
    const Word16 T0,                /* i  : Pitch estimation    Q0        */
    const Word16 enc,               /* i  : enc = 1 -> encoder side; enc = 0 -> decoder side */
    Word16 *sign              /* i/o: sign of the maximum */
);

void FEC_exc_estim_fx(
    Decoder_State_fx *st_fx,          /* i/o: Decoder static memory                        */
    const Word16 L_frame,           /* i  : length of the frame                          */
    Word16 *exc,              /* o  : pointer to excitation buffer (with past)     */
    Word16 *exc2,             /* o  : total excitation (for synthesis)             */
    Word16 dct_exc_tmp[],     /* o  : GSC excitation in DCT domain                 */
    Word16 *pitch_buf,        /* o  : Floating pitch   for each subframe           */
    Word16 *voice_factors,    /* o  : voicing factors                              */
    Word16 *tmp_tc,           /* o  : FEC pitch  Q6                                */
    Word16 *bwe_exc,          /* o  : excitation for SWB TBE                       */
    Word16 *lsf_new,          /* i  : ISFs at the end of the frame                 */
    Word16 *Q_exc,
    Word16 *tmp_noise          /* o  : long-term noise energy  Q0                   */
);

void FEC_lsf_estim_fx(
    Decoder_State_fx *st,               /* i/o: Decoder static memory                       */
    const Word16 L_frame,             /* i  : length of the frame                         */
    Word16 *Aq,                 /* o  : calculated A(z) for 4 subframes             */
    Word16 *lsf,                /* o  : estimated LSF vector                        */
    Word16 *lsp                 /* o  : estimated LSP vector                        */
);

void FEC_lsf_estim_enc_fx(
    Encoder_State_fx *st_fx,          /* i  : Encoder static memory                       */
    const Word16 L_frame,           /* i  : length of the frame                         */
    Word16 *lsf               /* o  : estimated LSF vector                        */
);

void FEC_clas_estim_fx(
    Decoder_State_fx *st_fx,    /* i/o: decoder state handle                    */
    const Word16 Opt_AMR_WB,          /* i  : flag indicating AMR-WB IO mode          */                /*A*/
    const Word16 L_frame,             /* i  : length of the frame                     */
    Word16 *clas,               /* i/o: frame classification                    */
    const Word16 coder_type,          /* i  : coder type                              */
    const Word16 *pitch,              /* i  : pitch values for each subframe    (Q6)  */
    Word16 *last_good,          /* i  : type of the last correctly received fr. */
    Word16 *syn,                /* i  : synthesis buffer                        */
    Word16 *lp_speech,          /* i/o: long term active speech energy average Q8 */
    Word16 *decision_hyst,      /* i/o: hysteresis of the music/speech decision */               /*A*/
    Word16 *UV_cnt,             /* i/o: number of consecutives frames classified as UV */         /*A*/
    Word16 *LT_UV_cnt,          /* i/o: long term consecutives frames classified as UV */         /*A*/
    Word16 *Last_ener,          /* i/o: last_energy frame                              */         /*A*/
    Word16 *locattack,          /* i/o: detection of attack (mainly to localized speech burst) */ /*A*/
    Word16 *lt_diff_etot,       /* i/o: long-term total energy variation               */         /*A*/
    Word16 *amr_io_class,       /* i/o: classification for AMR-WB IO mode       */                /*A*/
    const Word32  bitrate,            /* i  : Decoded bitrate                         */              /*A*/
    Word16 *Q_syn,              /* i  : Synthesis scaling                       */
    Word16 *class_para,         /* o  : classification para. fmerit1            */               /*A*/
    Word16 *mem_syn_clas_estim, /* i/o: memory of the synthesis signal for frame class estimation */
    Word16 *Q_mem_syn,          /*i/o : exponent for memory of synthesis signal for frame class estimation */ /*B*/
    Word16 pit_max,             /* i  : maximum pitch value, Q16 */                                           /*B*/
    Word16 LTP_Gain,            /* i  : LTP gain is 0..0.6 or negative  Q15*/                                 /*B*/
    Word16 mode,                /* i  : signal classifier mode                                              *//*B*/
    Word16 bfi,                 /* i  : bad frame indicator                                                 *//*B*/
    Word16 synthStart,          /* i  : starting point for synthesis buffer                                 *//*B*/
    Word16 synthLen             /* i  : synthesis buffer length , relevant for rescaling                    *//*B*/
);

void non_linearity_fx(
    const Word16 input[],               /* i  : input signal    Q_inp          */
    Word32 output[],              /* o  : output signal   2*Q_inp        */
    const Word16 length,                /* i  : input length                   */
    Word32 *prev_scale,           /* i/o: memory          Q30            */
    Word16 Q_inp,
    Word16 coder_type,            /* i  : Coder Type                     */
    Word16 *voice_factors,        /* i  : Voice Factors                  */
    const Word16 L_frame                /* i  : ACELP frame length             */
);

void stat_noise_uv_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: Decoder static memory                   */
    const Word16 coder_type,            /* i  : coding type                             */
    Word16 *lsp_new,              /* i  : end-frame LSP vector                    */
    Word16 *lsp_mid,              /* i  : mid-frame LSP vector                    */
    Word16 *Aq,                   /* o  : A(z) quantized for the 4 subframes      */
    Word16 *exc2                  /* i/o: excitation buffer                       */
);

void stat_noise_uv_mod_fx(
    const Word16 coder_type,       /* i  : Coder type                                 */
    Word16 noisiness,        /* i  : noisiness parameter                        */
    const Word16 *lsp_old,         /* i  : old LSP vector at 4th sfr                  */
    const Word16 *lsp_new,         /* i  : LSP vector at 4th sfr                      */
    const Word16 *lsp_mid,         /* i  : LSP vector at 2nd sfr                      */
    Word16 *Aq,              /* o  : A(z)   quantized for the 4 subframes       */
    Word16 *exc2,            /* i/o: excitation buffer                          */
    Word16 Q_exc,            /* i  : Q of exc2 excitation buffer [11..-1] expected */
    const Word16 bfi  ,            /* i  : Bad frame indicator                        */
    Word32 *ge_sm,           /* i/o: smoothed excitation gain                   */
    Word16 *uv_count,        /* i/o: unvoiced counter                           */
    Word16 *act_count,       /* i/o: activation counter                         */
    Word16 lspold_s[],       /* i/o: old LSP                                    */
    Word16 *noimix_seed,     /* i/o: mixture seed                               */
    Word16 *st_min_alpha,    /* i/o: minimum alpha                              */
    Word16 *exc_pe,          /* i/o: scale Q_stat_noise                         */
    const Word32 bitrate,          /* i  : core bitrate                               */
    const Word16 bwidth_fx,        /* i  : input bandwidth                            */
    Word16 *Q_stat_noise,    /* i/o: noise scaling                              */
    Word16 *Q_stat_noise_ge  /* i/o: noise scaling                              */
);

/* Just call preemph_copy_fx(), it does the same thing and that saves PROM */
#define preemph_fx(signal,mu,L,mem) preemph_copy_fx((signal),(signal),(mu),(L),(mem))

void preemph_copy_fx(
    const Word16 x[],   /* i  : input signal             Qx  */
    Word16 y[],   /* o  : output signal            Qx  */
    const Word16 mu,    /* i  : preemphasis coefficient  Q15 */
    const Word16 lg,    /* i  : vector size              Q0  */
    Word16 *mem   /* i/o: memory (x[-1])           Qx  */
);

Word16 vquant_fx(                   /* o: index of the winning codevector       */
    Word16 x[],               /* i: vector to quantize        Q13         */
    const Word16 x_mean[],          /* i: vector mean to subtract (0 if none)Q13*/
    Word16 xq[],              /* o: quantized vector                  Q13 */
    const Word16 cb[],              /* i: codebook                          Q13 */
    const Word16 dim,               /* i: dimension of codebook vectors         */
    const Word16 cbsize             /* i: codebook size                         */
);

Word16 w_vquant_fx(
    Word16 x[],         /* i: vector to quantize in Q10 */
    Word16 Qx,
    const Word16 weights[],   /* i: error weights in Q0 */
    Word16 xq[],        /* o: quantized vector in Q15 */
    const Word16 cb[],        /* i: codebook in Q15 */
    const Word16 cbsize,      /* i: codebook size */
    const Word16 rev_vect     /* i: reverse codebook vectors */
);

void Vr_subt(
    const Word16 x1[],  /* i  : Input vector 1                                   */
    const Word16 x2[],  /* i  : Input vector 2                                   */
    Word16 y[],   /* o  : Output vector that contains vector 1 - vector 2  */
    Word16 N      /* i  : Vector lenght                                    */
);

Word16 gsc_gainQ_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure            */
    const Word16 y_gain4[],         /* i  : Energy per band              Q13   */
    Word16 y_gainQ[],         /* o  : quantized energy per band    Q13   */
    const Word32 core_brate,        /* i  : Core rate                          */
    Word16 *old_y_gain,       /* i/o: AR mem for low rate ener Q   Q13   */
    const Word16 coder_type,        /* i  : coding type                        */
    const Word16 bwidth             /* i  : input signal bandwidth             */
);

Word16 emaximum_fx(             /* o  : return index with max energy value in vector  Q0 */
    const Word16 Qvec,          /* i  : Q of input vector                         Q0 */
    const Word16 *vec,          /* i  : input vector                              Qx */
    const Word16 lvec,          /* i  : length of input vector                    Q0 */
    Word32 *ener_max      /* o  : maximum energy value                      Q0 */
);

void set16_fx(
    Word16 y[],  /* i/o: Vector to set                       */
    const Word16 a,    /* i  : Value to set the vector to          */
    const Word16 N     /* i  : Lenght of the vector                */
);

void set32_fx(
    Word32 y[],  /* i/o: Vector to set                       */
    const Word32 a,    /* i  : Value to set the vector to          */
    const Word16 N     /* i  : Lenght of the vector                */
);

void Copy(
    const Word16 x[],  /* i  : input vector  */
    Word16 y[],  /* o  : output vector */
    const Word16 L     /* i  : vector length */
);

void Copy32(
    const Word32 x[],  /* i  : input vector  */
    Word32 y[],  /* o  : output vector */
    const Word16 L     /* i  : vector length */
);

void Scale_sig32(
    Word32 x[],  /* i/o: signal to scale                 Qx        */
    const Word16 lg,   /* i  : size of x[]                     Q0        */
    const Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void Copy_Scale_sig(
    const Word16 x[],   /* i  : signal to scale input           Qx        */
    Word16 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void Copy_Scale_sig_16_32(
    const Word16 x[],   /* i  : signal to scale input           Qx        */
    Word32 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void Copy_Scale_sig_32_16(
    const Word32 x[],   /* i  : signal to scale input           Qx        */
    Word16 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void init_lvq_fx(
    Word32 offset_scale1[][MAX_NO_SCALES+1],
    Word32 offset_scale2[][MAX_NO_SCALES+1],
    Word32 offset_scale1_p[][MAX_NO_SCALES+1],
    Word32 offset_scale2_p[][MAX_NO_SCALES+1],
    Word16 no_scales[][2],
    Word16 no_scales_p[][2]
);

void hf_synth_reset_fx(
    Word16 *seed2,             /* i/o: random seed for HF noise gen    */
    Word16 mem_hf[],           /* o  : HF band-pass filter memory      */
    Word16 mem_syn_hf[],       /* o  : HF synthesis memory             */
    Word16 mem_hp400[],        /* o  : memory of hp 400 Hz filter      */
    Word16 mem_hp_interp[],    /* o  : interpol. memory                */
    Word16 delay_syn_hf[]      /* o  : HF synthesis memory             */
);

void hf_synth_init_fx(
    Word16 mem_hp400[],   /* o  : 400 Hz high pass filter memory initialization    */
    Word16 mem_hf[]       /* o  : band pass 6kHz to 7kHz FIR filter initialization */
);

Word16 Random(              /* o  : output random value */
    Word16 *seed            /* i/o: random seed         */
);

Word16 own_random2_fx(      /* o  : output random value */
    Word16 seed             /* i  : random seed         */
);

void Random_Fill(
    Word16 *seed,           /* i/o: random seed         */
    Word16 n,               /* i  : number of values    */
    Word16 *y,              /* o  : output values       */
    Word16 scaling          /* i  : scaling of values   */
);

void iDiv_and_mod_32(
    const Word32 Numer,       /* i  : 32 bits numerator   */
    const Word16 Denom,       /* i  : 16 bits denominator */
    Word32 * Int_quotient,    /* o  : integer result of the division (int)(num/den) */
    Word32 * Int_mod,         /* o  : modulo result of the division  num-((int)(num/den)*den)*/
    const Word16 rshift       /* i  : 0 if no right shift / 1 if the denom is right shifted by 1*/
);

void lsf2lsp_fx(
    const Word16 lsf[],     /* i  : lsf[m] normalized (range: 0.0<=val<=0.5)  x2.56 */
    Word16 lsp[],     /* o  : lsp[m] (range: -1<=val<1)                 Q15   */
    const Word16 m,         /* i  : LPC order                                 Q0    */
    Word32 int_fs
);

void bass_psfilter_init_fx(
    Word16 old_syn[],        /* o  : Old synthesis buffer 1        */
    Word16 *mem_deemph_err,  /* o  : Error deemphasis memory       */
    Word16 *lp_ener          /* o  : long_term error signal energy */
);

void decod_gen_voic_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder static memory                     */
    const Word16 L_frame_fx,          /* i  : length of the frame                       */
    const Word16 sharpFlag_fx,        /* i  : formant sharpening flag                   */
    const Word16 *Aq_fx,              /* i  : LP filter coefficient                     */
    const Word16 coder_type_fx,       /* i  : coding type                               */
    const Word16 Es_pred_fx,          /* i  : predicted scaled innov. energy            */
    const Word16 do_WI_fx,            /* i  : do interpolation after a FER              */
    Word16 *pitch_buf_fx,           /* o  : floating pitch values for each subframe   */
    Word16 *voice_factors_fx,       /* o  : voicing factors                           */
    Word16 *exc_fx,                 /* i/o: adapt. excitation exc                     */
    Word16 *exc2_fx,                /* i/o: adapt. excitation/total exc               */
    Word16 *bwe_exc_fx,             /* o  : excitation for SWB TBE                    */
    Word16 *unbits,                 /* number of unused bits                          */
    Word16* gain_buf                /*Q14*/
);

Word16 tc_classif_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word16 L_frame                /* i  : length of the frame               */
);

void transition_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                 */
    const Word32  core_brate,         /* i  : core bitrate                            */
    const Word16 Opt_AMR_WB,          /* i  : flag indicating AMR-WB IO mode          */
    const Word16 L_frame,             /* i  : length of the frame                     */
    const Word16 i_subfr,             /* i  : subframe index                          */
    const Word16 coder_type,          /* i  : coder type                              */
    const Word16 tc_subfr,            /* i  : TC subframe index                       */
    Word16 *Jopt_flag,          /* i  : joint optimization flag                 */
    Word16 *exc,                /* o  : excitation signal                       */
    Word16 *T0,                 /* o  : close loop integer pitch                */
    Word16 *T0_frac,            /* o  : close loop fractional part of the pitch */
    Word16 *T0_min,             /* i/o: delta search min for sf 2 & 4           */
    Word16 *T0_max,             /* i/o: delta search max for sf 2 & 4           */
    Word16 **pt_pitch,          /* o  : floating pitch values                   */
    Word16 *position,           /* i/o: first glottal impulse position in frame */
    Word16 *bwe_exc,            /* o  : excitation for SWB TBE                  */
    Word16 *Q_exc               /*i/o : scaling of excitation                   */
);

void decod_tran_fx(
    Decoder_State_fx *st_fx,              /* i/o: decoder static memory                   */
    const Word16 L_frame_fx,            /* i  : length of the frame                     */
    const Word16 tc_subfr_fx,           /* i  : TC subframe index                       */
    const Word16 *Aq_fx,                /* i  : LP filter coefficient                   */
    const Word16 coder_type_fx,         /* i  : coding type                             */
    const Word16 Es_pred_fx,            /* i  : predicted scaled innov. energy          */
    Word16 *pitch_buf_fx,         /* o  : floating pitch values for each subframe */
    Word16 *voice_factors_fx,     /* o  : voicing factors                         */
    Word16 *exc_fx,               /* i/o: adapt. excitation exc                   */
    Word16 *exc2_fx,              /* i/o: adapt. excitation/total exc             */
    Word16 *bwe_exc_fx,           /* i/o: excitation for SWB TBE                  */
    Word16 *unbits,               /* i/o: number of unused bits                   */
    const Word16 sharpFlag,             /* i  : formant sharpening flag                 */
    Word16 *gain_buf              /*Q14*/
);

void interp_code_4over2_fx(
    const Word16 inp_code_fx[],         /* i  : input vector                Qx          */
    Word16 interp_code_fx[],      /* o  : output vector               Qx          */
    const Word16 inp_length             /* i  : length of input vector                  */
);

void pred_lt4_tc_fx(
    Word16 exc[],                   /* i/o: excitation buffer        */
    const Word16 T0,                      /* i  : integer pitch lag        */
    Word16 frac,                    /* i:   fraction of lag          */
    const Word16 *win,                    /* i  : interpolation window     */
    const Word16 imp_pos,                 /* i  : glottal impulse position */
    const Word16 i_subfr                  /* i  : subframe index           */
);

void gaus_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder static memory                                       */
    const Word32 core_brate,            /* i   */  /*1    : core bitrate                                    */
    const Word16 i_subfr,               /* i   */  /*2    : subframe index                                  */
    Word16 *code,                 /* o   */  /*3    : unvoiced excitation                        Q12  */
    Word32 *L_norm_gain_code,     /* o   */  /*4    : gain of normalized gaussian excitation     Q16  */
    Word16 *lp_gainp,             /* i/o */  /*5    : lp filtered pitch gain(FER)                Q14  */
    Word16 *lp_gainc,             /* i/o */  /*6    : lp filtered code gain (FER)                Q3   */
    Word16 *inv_gain_inov,        /* o   */  /*7    : unscaled innovation gain                   Q12  */
    Word16 *tilt_code,            /* o   */  /*8    : synthesis excitation spectrum tilt         Q15  */
    Word16 *voice_fac,            /* o   */  /*9    : estimated voicing factor                   Q15  */
    Word16 *gain_pit,             /* o   */  /*10  : pitch gain                                  Q14  */
    Word16 *pt_pitch_1,           /* o   */  /*11  : floating pitch buffer                       Q6   */
    Word16 *exc,                  /* o   */  /*12  : excitation signal frame                          */
    Word32 *L_gain_code,          /* o   */  /*13  : gain of the gaussian excitation             Q16  */
    Word16 *exc2,                 /* o   */  /*14  : Scaled excitation signal frame                   */
    Word16  *bwe_exc_fx,
    Word16 *sQ_exc,               /* i/o */  /*15  : Excitation scaling factor (Decoder state)        */
    Word16 *sQsubfr               /* i/o */  /*16  : Past excitation scaling factors (Decoder State)  */
);

void decod_unvoiced_fx(
    Decoder_State_fx *st_fx,            /*   i/o: decoder static memory                             */
    const Word16 *Aq_fx,                  /*   Q12        i  : LP filter coefficient                  */
    const Word16 coder_type_fx,       /*   Q0        i  : coding type                             */
    Word16 *tmp_noise_fx,       /*   Q5        o  : long term temporary noise energy        */
    Word16 *pitch_buf_fx,       /*   Q6        o  : floating pitch values for each subframe */
    Word16 *voice_factors_fx,   /*   Q15        o  : voicing factors                        */
    Word16 *exc_fx,             /*   Q_X      o  : adapt. excitation exc                    */
    Word16 *exc2_fx,            /*   Q_X        o  : adapt. excitation/total exc            */
    Word16 *bwe_exc_fx,         /*   Q_X        i/o: excitation for SWB TBE                 */
    Word16 *gain_buf
);

Word32 gain_dec_gaus_fx(            /* o  : quantized codebook gain                Q16  */
    Word16 index,             /* i  : quantization index                          */
    const Word16 bits,              /* i  : number of bits to quantize                  */
    const Word16 lowBound,          /* i  : lower bound of quantizer (dB)               */
    const Word16 topBound,          /* i  : upper bound of quantizer (dB)               */
    const Word16 inv_gain_inov,     /* o  : unscaled innovation gain                Q12 */
    Word32 *L_norm_gain_code  /* o  : gain of normalized gaussian excitation  Q16 */
);

void AVQ_demuxdec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    Word16  xriq[],           /* o:   decoded subvectors [0..8*Nsv-1] */
    Word16  *nb_bits,         /* i/o: number of allocated bits        */
    const Word16  Nsv,              /* i:   number of subvectors            */
    Word16 nq_out[]           /* i/o: AVQ nq index                    */
);

void re8_decode_base_index_fx(Word16 n, UWord16 I, Word16 *x);

void re8_k2y_fx(
    const Word16 *k,                /* i  : Voronoi index k[0..7]                    */
    const Word16 m,                 /* i  : Voronoi modulo (m = 2^r = 1<<r, where r is integer >=2) */
    Word16 *y                 /* o  : 8-dimensional point y[0..7] in RE8    */
);

void re8_vor_fx(
    const Word16 y[],               /* i  : point in RE8 (8-dimensional integer vector)                   */
    Word16 *n,                /* o  : codebook number n=0,2,3,4,... (scalar integer)                */
    Word16 k[],               /* o  : Voronoi index (integer vector of dimension 8) used only if n>4*/
    Word16 c[],               /* o  : codevector in Q0, Q2, Q3, or Q4 if n<=4, y=c                  */
    Word16 *ka                /* o  : identifier of absolute leader (to index c)                    */
);

void re8_cod_fx(
    Word16 x[],                     /* i  : point in RE8 (8-dimensional integer vector)                         */
    Word16 *n,                      /* i  : codebook number (*n is an integer defined in {0,2,3,4,..,n_max})    */
    UWord16 *I,                     /* o  : index of c (pointer to unsigned 16-bit word)                        */
    Word16 k[]                      /* o  : index of v (8-dimensional vector of binary indices) = Voronoi index */
);

void re8_PPV_fx(
    const Word32 x[],               /* i  : point in R^8Q15 */
    Word16 y[]                /* o  : point in RE8 (8-dimensional integer vector) */
);

void re8_dec_fx(
    Word16 n,                 /* i  : codebook number (*n is an integer defined in {0,2,3,4,..,n_max})    */
    const UWord16 I,                /* i  : index of c (pointer to unsigned 16-bit word)                        */
    const Word16 k[],               /* i  : index of v (8-dimensional vector of binary indices) = Voronoi index */
    Word16 y[]                /* o  : point in RE8 (8-dimensional integer vector)                         */
);

void gain_dec_SQ_fx(
    Decoder_State_fx *st_fx,         /* i/o: decoder state structure */
    const Word32 core_brate,       /* i  : core bitrate                             */
    const Word16 coder_type,       /* i  : coding type                              */
    const Word16 i_subfr,          /* i  : subframe number                          */
    const Word16 tc_subfr,         /* i  : TC subframe index                        */
    const Word16 *code,            /* i  : algebraic code excitation             Q12*/
    const Word16 Es_pred,          /* i  : predicted scaled innov. energy        Q8 */
    Word16 *gain_pit,        /* o  : Quantized pitch gain                  Q14*/
    Word32 *gain_code,       /* o  : Quantized codeebook gain              Q16*/
    Word16 *gain_inov,       /* o  : unscaled innovation gain              Q12*/
    Word32 *norm_gain_code   /* o  : norm. gain of the codebook excitation Q16*/
);

void gain_dec_amr_wb_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                     */
    const Word32 core_brate,        /* i  : core bitrate                                */
    Word16 *gain_pit,         /* o  : Quantized pitch gain                        */
    Word32 *gain_code,        /* o  : Quantized codeebook gain                    */
    Word16 *past_qua_en,      /* i/o: gain quantization memory (4 words)          */
    Word16 *gain_inov,        /* o  : unscaled innovation gain                    */
    const Word16 *code,             /* i  : algebraic code excitation                   */
    Word32 *norm_gain_code    /* o  : norm. gain of the codebook excitation       */
);

void isf_dec_amr_wb_fx(
    Decoder_State_fx *st,               /* i/o: State structure                             */
    Word16 *Aq,                 /* o  : quantized A(z) for 4 subframes              */
    Word16 *isf_new,            /* o  : de-quantized ISF vector                     */
    Word16 *isp_new             /* o  : de-quantized ISP vector                     */
);

void syn_output_fx(
    const Word16 codec_mode,        /* i  : MODE1 or MODE2                              */
    Word16 *synth,            /* i/o: float synthesis signal                      */
    const Word16 output_frame,      /* i  : output frame length                         */
    Word16 *synth_out,        /* o  : integer 16 bits synthesis signal            */
    const Word16 Q_syn2             /* i  : Synthesis scaling factor                    */
);

void amr_wb_dec_fx(
    Word16 output_sp[],                 /* o  : synthesis output                        */
    Decoder_State_fx *st_fx             /* o  : Decoder static variables structure      */
);

void decod_amr_wb_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder static memory                   */
    const Word16 *Aq_fx,                /* i  : LP filter coefficients                  */
    Word16 *pitch_buf_fx,         /* o  : floating pitch values for each subframe */
    Word16 *exc_fx,               /* i/o: adapt. excitation exc                   */
    Word16 *exc2_fx,              /* i/o: adapt. excitation/total exc             */
    Word16 hf_gain_fx[NB_SUBFR],  /* o  : decoded HF gain                         */
    Word16 *voice_factors_fx,     /* o  : voicing factors                         */
    Word16 *gain_buf              /*Q14*/
);

void disf_2s_46b_fx(
    Word16 *indice,           /* i  : quantized indices (use indice[0] = -1 in the decoder)   */
    Word16 *isf_q,            /* o  : quantized ISFs in the cosine domain                     */
    Word16 *mem_AR,           /* o  : quantizer memory for AR model                           */
    Word16 *mem_MA,           /* i/o: quantizer memory for MA model                           */
    const Word16 enc_dec            /* i  : encoder (0), decoder (1) G722.2 FER                     */
);

void disf_2s_36b_fx(
    Word16 *indice,           /* i  : quantized indices (use indice[0] = -1 in the decoder)   */
    Word16 *isf_q,            /* o  : quantized ISFs in the cosine domain                     */
    Word16 *mem_AR,           /* i/o: quantizer memory for AR model                           */
    Word16 *mem_MA,           /* i/o: quantizer memory for MA model                           */
    const Word16 enc_dec            /* i  : encoder (0), decoder (1) G722.2 FER                     */
);

void transf_cdbk_dec_fx(
    Decoder_State_fx *st_fx,       /* i/o: decoder state structure                         */
    const Word32 core_brate,     /* i  : core bitrate                                    */
    const Word16 coder_type,     /* i  : coding type                                     */
    const Word16 harm_flag_acelp,/* i  : harmonic flag for higher rates ACELP            */
    const Word16 i_subfr,        /* i  : subframe index                                  */
    const Word16 tc_subfr,       /* i  : TC subframe index                               */
    const Word16 Es_pred,        /* i  : predicited scaled innovation energy (Q8)        */
    const Word32 gain_code,      /* i  : innovative excitation gain (Q16)                */
    Word16 *mem_preemp,    /* i/o: dequantizer preemhasis memory                   */
    Word16 *gain_preQ,     /* o  : prequantizer excitation gain (Q2)               */
    Word32 *norm_gain_preQ,/* o  : normalized prequantizer excitation gain (Q16)   */
    Word16 code_preQ[],    /* o  : prequantizer excitation (Q8)                    */
    Word16 *unbits         /* o  : number of AVQ unused bits                       */
);

void gain_enc_SQ_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure      */
    const Word32 core_brate,        /* i  : core bitrate                                                    */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 tc_subfr,          /* i  : TC subframe index                                               */
    const Word16 *xn,               /* i  : target vector                                                  Q_xn */
    const Word16 *yy1,              /* i  : zero-memory filtered adaptive excitation                       Q_xn */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation             Q9 */
    const Word16 *code,             /* i  : algebraic excitation                                           Q9 */
    const Word16 Es_pred,           /* i  : predicted scaled innovation energy                             Q8 */
    Word16 *gain_pit,         /* o  : quantized pitch gain                                           Q14 */
    Word32 *gain_code,        /* o  : quantized codebook gain                                        Q16 */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                Q12 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                          Q16 */
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain,         /* i  : gain pitch clipping flag (1 = clipping)                         */
    const Word16  Q_xn              /* i  : xn and y1 scaling                                               */
);

void AVQ_cod_fx(             /* o:   comfort noise gain factor        */
    const Word16 xri[],      /* i:   vector to quantize               */
    Word16 xriq[],     /* o:   quantized normalized vector (assuming the bit budget is enough) */
    const Word16 NB_BITS,    /* i:   number of allocated bits         */
    const Word16 Nsv,        /* i:   number of subvectors (lg=Nsv*8)  */
    const Word16 Q_in        /* i:   Scaling input                    */
);

void AVQ_encmux_fx(
    Encoder_State_fx *st_fx,     /* i/o: encoder state structure                         */
    const Word16 extl,       /* i  : extension layer                                 */
    Word16 xriq[],     /* i/o: rounded subvectors [0..8*Nsv-1] followed
                                    by rounded bit allocations [8*Nsv..8*Nsv+Nsv-1]  */
    Word16 *nb_bits,   /* i/o: number of allocated bits                        */
    const Word16 Nsv,        /* i:   number of subvectors                            */
    Word16 nq_out[]    /* o  : AVQ nq index                                    */
);

Word32 Dot_product(         /* o  : Sum              */
    const Word16 x[],       /* i  : 12bits: x vector */
    const Word16 y[],       /* i  : 12bits: y vector */
    const Word16 lg         /* i  : vector length    */
);

Word16 usquant_fx(          /* o: index of the winning codeword   */
    const Word16 x,         /* i: scalar value to quantize        Qx*/
    Word16 *xq,       /* o: quantized value                 Qx*/
    const Word16 qlow,      /* i: lowest codebook entry (index 0) Qx*/
    const Word16 delta,     /* i: quantization step               Qx-1*/
    const Word16 cbsize     /* i: codebook size                   */
);

Word16 gain_quant_fx(       /* o:   quantization index            */
    Word32 *gain,     /* i/o: quantized gain                */
    Word16 *gain16,
    const Word16 c_min,     /* i:  log10 of lower limit in Q14         */
    const Word16 c_max,     /* i:  log10 of upper limit in Q13          */
    const Word16 bits,      /* i:   number of bits to quantize    */
    Word16 *expg
);

void gain_enc_mless_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure */
    const Word32  core_brate,        /* i  : core bitrate                                                    */
    const Word16 L_frame,           /* i  : length of the frame                                             */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 tc_subfr,          /* i  : TC subframe index                                               */
    const Word16 *xn,               /* i  : target vector                                                   */
    const Word16 *y1,               /* i  : zero-memory filtered adaptive excitation                        */
    const Word16  Q_xn,             /* i  : xn and y1 scaling                                               */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation              */
    const Word16 *code,             /* i  : algebraic excitation                                            */
    const Word16 Es_pred,           /* i  : predicted scaled innovation energy                              */
    Word16 *gain_pit,         /* o  : quantized pitch gain                                            */
    Word32 *gain_code,        /* o  : quantized codebook gain                                         */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                           */
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain          /* i  : gain pitch clipping flag (1 = clipping)                         */
);

Word16 gain_enc_gaus_fx(                /* o  : Return index of quantization      */
    Word32 *gain,                 /* i/o: Code gain to quantize             */
    const Word16 bits,                  /* i  : number of bits to quantize        */
    const Word16 lowBound,              /* i  : lower bound of quantizer (dB) Q8  */
    const Word16 stepSize,              /* i  : Step size choice              Q14 */
    const Word16 inv_stepSize           /* i  : Step size choice              Q15 */
);

void gain_enc_tc_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure */
    const Word32  core_brate,           /* i  : core bitrate                                       */
    const Word16 L_frame,               /* i  : length of the frame                                */
    const Word16 i_subfr,               /* i  : subframe index                                     */
    const Word16 tc_subfr,              /* i  : TC subframe index                                  */
    const Word16 xn_fx[],               /* i  : target vector                                      */
    const Word16 y2_fx[],               /* i  : zero-memory filtered algebraic codebook excitation */
    const Word16 code_fx[],             /* i  : algebraic excitation                               */
    const Word16 Es_pred_fx,            /* i  : predicted scaled innovation energy                 */
    Word16 *gain_pit_fx,          /* o  : Pitch gain / Quantized pitch gain                  */
    Word32 *gain_code_fx,         /* o  : quantized codebook gain                            */
    Word16 *gain_inov_fx,         /* o  : innovation gain                                    */
    Word32 *norm_gain_code_fx,    /* o  : norm. gain of the codebook excitation              */
    const Word16  Q_xn                  /* i  : xn and y1 scaling                               Q0 */
);

Word16 gaus_encode_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                       */
    const Word16 i_subfr,               /* i  : subframe index                                */
    const Word16 *h1,                   /* i  : weighted filter input response                */
    const Word16 *xn,                   /* i  : target vector                                 */
    Word16 *exc,                  /* o  : pointer to excitation signal frame            */
    Word16 *mem_w0,               /* o  : weighting filter denominator memory           */
    Word16 *clip_gain,            /* o  : memory of gain of pitch clipping algorithm    */
    Word16 *tilt_code,            /* o  : synthesis excitation spectrum tilt            */
    Word16 *code,                 /* o  : algebraic excitation                     Q9   */
    Word32 *gain_code,            /* o  : Code gain.                               Q16  */
    Word16 *y2,                   /* o  : zero-memory filtered adaptive excitation Q9   */
    Word16 *gain_inov,            /* o  : innovation gain                          Q12  */
    Word16 *voice_fac,            /* o  : voicing factor                           Q15  */
    Word16 *gain_pit,             /* o  : adaptive excitation gain                 Q14  */
    const Word16 Q_new,                 /* i  : scaling factor                                */
    const Word16 shift,                 /* i  : scaling factor                                */
    Word32 *norm_gain_code,       /* o  : normalized innovative cb. gain           Q16  */
    const Word32  core_brate            /* i  : core bitrate                                  */
);

void encod_unvoiced_fx(
    Encoder_State_fx *st_fx,            /* i/o: state structure                                 */
    LPD_state   *mem,            /* i/o: acelp memories                                  */
    const Word16 *speech_fx,            /* i  : Input speech                                    */
    const Word16 Aw_fx[],             /* i  : weighted A(z) unquantized for subframes         */
    const Word16 *Aq_fx,              /* i  : 12k8 Lp coefficient                             */
    const Word16 vad_flag_fx,
    const Word16 *res_fx,             /* i  : residual signal                                 */
    Word16 *syn_fx,             /* o  : core synthesis                                  */
    Word16 *tmp_noise_fx,       /* o  : long-term noise energy                          */
    Word16 *exc_fx,             /* i/o: current non-enhanced excitation                 */
    Word16 *pitch_buf_fx,       /* o  : floating pitch values for each subframe         */
    Word16 *voice_factors_fx,   /* o  : voicing factors                                 */
    Word16 *bwe_exc_fx,         /* i/o: excitation for SWB TBE                          */
    const Word16 Q_new,
    const Word16 shift
);

void transf_cdbk_enc_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure                         */
    const Word32  core_brate,           /* i  : core bitrate                                    */
    const Word16 extl,                  /* i  : extension layer                                 */
    const Word16 coder_type,            /* i  : coding type                                     */
    const Word16 harm_flag_acelp,       /* i  : harmonic flag for higher rates ACELP            */
    const Word16 i_subfr,               /* i  : subframe index                                  */
    const Word16 tc_subfr,              /* i  : TC subframe index                               */
    Word16 cn[],                  /* i/o: target vector in residual domain                */
    Word16 exc[],                 /* i/o: pointer to excitation signal frame              */
    const Word16 *p_Aq,                 /* i  : 12k8 Lp coefficient                             */
    const Word16 Ap[],                  /* i  : weighted LP filter coefficients                 */
    const Word16 h1[],                  /* i  : weighted filter input response                  */
    Word16 xn[],                  /* i/o: target vector                                   */
    Word16 xn2[],                 /* i/o: target vector for innovation search             */
    Word16 y1[],                  /* i/o: zero-memory filtered adaptive excitation        */
    const Word16 y2[],                  /* i  : zero-memory filtered innovative excitation      */
    const Word16 Es_pred,               /* i  : predicited scaled innovation energy             */
    Word16 *gain_pit,             /* i/o: adaptive excitation gain                        */
    const Word32 gain_code,             /* i  : innovative excitation gain                      */
    Word16 g_corr[],              /* o  : ACELP correlation values                        */
    const Word16 clip_gain,             /* i  : adaptive gain clipping flag                     */
    Word16 *mem_deemp,            /* i/o: prequantizer deemhasis memory                   */
    Word16 *mem_preemp,           /* i/o: prequantizer preemhasis memory                  */
    Word16 *gain_preQ,            /* o  : prequantizer excitation gain                    */
    Word16 code_preQ[],           /* o  : prequantizer excitation                         */
    Word16 *unbits,               /* o  : number of AVQ unused bits                       */
    const Word16 Q_new,                 /* i  : Current frame scaling                           */
    const Word16 shift                  /* i  : shifting applied to y1, xn,...                  */
);

Word16 encod_tran_fx(
    Encoder_State_fx *st_fx,              /* i/o: state structure                                 */
    LPD_state   *mem,              /* i/o: acelp memories                                  */
    const Word16 L_frame_fx,            /* i  : length of the frame                             */
    const Word16 speech_fx[],           /* i  : input speech                                    */
    const Word16 Aw_fx[],               /* i  : weighted A(z) unquantized for subframes         */
    const Word16 Aq_fx[],               /* i  : 12k8 Lp coefficient                             */
    const Word16 coder_type,            /* i  : coding type                                     */
    const Word16 Es_pred_fx,            /* i  : predicted scaled innov. energy                  */
    const Word16 T_op_fx[],             /* i  : open loop pitch                                 */
    const Word16 voicing_fx[],          /* i  : voicing                                         */
    const Word16 *res_fx,               /* i  : residual signal                                 */
    Word16 *syn_fx,               /* i/o: core synthesis                                  */
    Word16 *exc_fx,               /* i/o: current non-enhanced excitation                 */
    Word16 *exc2_fx,              /* i/o: current enhanced excitation                     */
    Word16 *pitch_buf_fx,         /* i/o: floating pitch values for each subframe         */
    Word16 *voice_factors,        /* o  : voicing factors                                 */
    Word16 *bwe_exc_fx,           /* i/o: excitation for SWB TBE                          */
    const Word16 gsc_attack_flag,       /* i  : Flag to indicate when an audio attack is deal with TM */
    Word16 *unbits,               /* i/o: number of unused bits                           */
    Word16 sharpFlag,             /* o  : formant sharpening flag                         */
    const Word16 shift,                 /* i  : Scaling to get 12 bits                          */
    const Word16 Q_new                  /* i  : Input scaling                                   */
);

Word16 est_tilt_fx(                         /* o  : tilt of the code              Q15   */
    const Word16 *exc,                      /* i  : adaptive excitation vector      Qx  */
    const Word16 gain_pit,                  /* i  : adaptive gain                   Q14 */
    const Word16 *code,                     /* i  : algebraic exctitation vector    Q9  */
    const Word32 gain_code,                 /* i  : algebraic code gain             Q16 */
    Word16 *voice_fac,                /* o  : voicing factor                  Q15 */
    const Word16 Q_exc                      /* i  : Scaling factor of excitation    Q0  */
);

Word16 Est_tilt2(                           /* o  : tilt of the code                    */
    const Word16 *exc,                      /* i  : adaptive excitation vector      Qx  */
    const Word16 gain_pit,                  /* i  : adaptive gain                   Q14 */
    const Word16 *code,                     /* i  : algebraic exctitation vector    Q9  */
    const Word32 gain_code,                 /* i  : algebraic code gain             Q16 */
    Word16 *voice_fac,                /* o  : voicing factor                  Q15 */
    const Word16 Q_exc                      /* i  : Scaling factor of excitation    Q0  */
);

void transition_enc_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure */
    const Word32  core_brate,       /* i  : core bitrate                                */
    const Word16 L_frame,           /* i  : length of the frame                         */
    const Word16 coder_type,        /* i  : coding type                                 */
    const Word16 i_subfr,           /* i  : subframe index                              */
    Word16 *tc_subfr,         /* i/o: TC subframe index                           */
    Word16 *Jopt_flag,        /* i  : joint optimization flag                     */
    Word16 *position,         /* i/o: maximum of residual signal index            */
    const Word16 voicing_fx[],      /* i  : normalized correlations (from OL pitch)     Q15*/
    const Word16 T_op_fx[],         /* i  : open loop pitch estimates in current frame  Q0*/
    Word16 *T0,               /* i/o: close loop integer pitch                    Q0*/
    Word16 *T0_frac,          /* i/o: close loop fractional part of the pitch     Q0*/
    Word16 *T0_min,           /* i/o: lower limit for close-loop search           Q0*/
    Word16 *T0_max,           /* i/o: higher limit for close-loop search          Q0*/
    Word16 *exc_fx,           /* i/o: pointer to excitation signal frame          Q_new*/
    Word16 *y1_fx,            /* o  : zero-memory filtered adaptive excitation    Q_new-1+shift*/
    const Word16 *res_fx,           /* i  : pointer to the LP residual signal frame     Q_new*/
    const Word16 *h1_fx,            /* i  : weighted filter input response              Q(14+shift)*/
    const Word16 *xn_fx,            /* i  : target vector                               Q_new-1+shift*/
    Word16 *xn2_fx,           /* o  : target vector for innovation search         Q_new-1+shift*/
    Word16 *gp_cl_fx,         /* i/o: memory of gain of pitch clipping algorithm  */
    Word16 *gain_pit_fx,      /* o  : adaptive excitation gain                    Q14*/
    Word16 *g_corr_fx,        /* o  : ACELP correlation values                    */
    Word16 *clip_gain,        /* i/o: adaptive gain clipping flag                 */
    Word16 **pt_pitch_fx,     /* o  : floating pitch values                       */
    Word16 *bwe_exc_fx,       /* o  : excitation for SWB TBE                      Q_new*/
    Word16 Q_new,             /* i  : Current scaling */
    Word16 shift              /* i  : downscaling needs for 12 bits convolutions */
);

Word16 abs_pit_enc_fx(             /* o  : pitch index                                              */
    const Word16 fr_steps,        /* i  : fractional resolution steps (2 or 4) for shortest pitches*/
    const Word16 limit_flag,      /* i  : restrained(0) or extended(1) limits                      */
    const Word16 T0,              /* i  : integer pitch lag                                        */
    const Word16 T0_frac          /* i  : pitch fraction                                           */
);

Word16 delta_pit_enc_fx(           /* o  : pitch index                         */
    const Word16 fr_steps,        /* i  : fractional resolution steps (2 or 4)*/
    const Word16 T0,              /* i  : integer pitch lag                   */
    const Word16 T0_frac,         /* i  : pitch fraction                      */
    const Word16 T0_min           /* i  : delta search min                    */
);

void set_impulse_fx(
    const Word16 xn_fx[],           /* i  :  target signal                                 */
    const Word16 h_orig_fx[],       /* i  :  impulse response of weighted synthesis filter */
    Word16 exc_fx[],          /* o  :  adaptive codebook excitation                  */
    Word16 yy1_fx[],          /* o  :  filtered adaptive codebook excitation         */
    Word16 *imp_shape,        /* o  :  adaptive codebook index                       */
    Word16 *imp_pos,          /* o  :  position of the glotal impulse center index   */
    Word32 *gain_trans_fx,    /* o  :  transition gain                         Q7    */
    Word16 Q_new              /* i  :  Current scaling                               */
);

void pit16k_Q_enc_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure                 */
    const Word16 nBits,             /* i  : # of Q bits                             */
    const Word16 limit_flag,        /* i  : restrained(0) or extended(1) Q limits   */
    const Word16 T0,                /* i  : integer pitch lag                       */
    const Word16 T0_frac,           /* i  : pitch fraction                          */
    Word16 *T0_min,           /* i/o: delta search min                        */
    Word16 *T0_max            /* o  : delta search max                        */
);

void gain_enc_amr_wb_fx(
    Encoder_State_fx *st,             /* i/o: encoder state structure                                         */
    const Word16 *xn,               /* i  : target vector                                                   */
    const Word16 Q_xn,              /* i  : xn and yy1 format                                    Q0         */
    const Word16 *yy1,              /* i  : zero-memory filtered adaptive excitation                        */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation              */
    const Word16 *code,             /* i  : algebraic excitation                                            */
    const Word32 core_brate,        /* i  : core bitrate                                                    */
    Word16 *gain_pit,         /* i/o: pitch gain / Quantized pitch gain                               */
    Word32 *gain_code,        /* o  : quantized codebook gain                                         */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                           */
    Word16 *g_coeff,          /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain,         /* i  : gain pitch clipping flag (1 = clipping)                         */
    Word16 *past_qua_en       /* i/o: gain quantization memory (4 words)                              */
);

void gain_enc_lbr_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure                                         */
    const Word32  core_brate,       /* i  : core bitrate                                                    */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 *xn,               /* i  : target vector                                               Q_xn*/
    const Word16 *y1,               /* i  : zero-memory filtered adaptive excitation                    Q_xn*/
    const Word16 Q_xn,              /* i  : xn and y1 format                                                */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation            Q9*/
    const Word16 *code,             /* i  : algebraic excitation                                          Q9*/
    Word16 *gain_pit,         /* o  : quantized pitch gain                                         Q14*/
    Word32 *gain_code,        /* o  : quantized codebook gain                                      Q16*/
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)              Q12*/
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                        Q16*/
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> mant/exp*/
    Word32 gc_mem[],          /* i/o: gain_code from previous subframes                               */
    Word16 gp_mem[],          /* i/o: gain_pitch from previous subframes                              */
    const Word16 clip_gain          /* i  : gain pitch clipping flag (1 = clipping)                         */
);

void gp_clip_test_gain_pit_fx(
    const Word16 gain_pit,        /* i  : gain of quantized pitch                  Q14 */
    Word16 mem[]            /* i/o: memory of gain of pitch clipping algorithm   */
);

void gp_clip_test_lsf_fx(
    const Word16 lsf[],         /* i  : lsf values (in frequency domain)           */
    Word16 mem[],         /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 Opt_AMR_WB     /* i  : flag indicating AMR-WB IO mode             */
);

Word16 inov_encode_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word32  core_brate,      /* i  : core bitrate                                    */
    const Word16 Opt_AMR_WB,       /* i  : flag indicating AMR-WB IO mode                  */
    const Word16 L_frame,          /* i  : length of the frame                             */
    const Word16 last_L_frame,     /* i  : length of the last frame                        */
    const Word16 coder_type,       /* i  : coding type                                     */
    const Word16 bwidth,           /* i  : input signal bandwidth                          */
    const Word16 sharpFlag,        /* i  : formant sharpening flag                         */
    const Word16 i_subfr,          /* i  : subframe index                                  */
    const Word16 tc_subfr,         /* i  : TC subframe index                               */
    const Word16 *p_Aq,            /* i  : LP filter coefficients                          Q12*/
    const Word16 gain_pit,         /* i  : adaptive excitation gain                        Q14*/
    Word16 *cn,              /* i/o: target vector in residual domain                Q_new*/
    const Word16 *exc,             /* i  : pointer to excitation signal frame              Q_new*/
    Word16 *h2,              /* i/o: weighted filter input response                  Q12*/
    const Word16 tilt_code,        /* i  : tilt of the excitation of previous subframe     Q15*/
    const Word16 pt_pitch,         /* i  : pointer to current subframe fractional pitch    Q6*/
    const Word16 *xn2,             /* i  : target vector for innovation search             Q_new-1+shift*/
    Word16 *code,            /* o  : algebraic excitation                            Q9*/
    Word16 *y2,              /* o  : zero-memory filtered algebraic excitation       Q9*/
    Word16 *unbits,          /* o  : number of unused bits for  PI                   */
    Word16 shift
);

Word32 sum2_fx(                     /* o  : sum of all squared vector elements    Q(2x+1)*/
    const Word16 *vec,                /* i  : input vector                          Qx*/
    const Word16 lvec                 /* i  : length of input vector                */
);

Word16 usdequant_fx(                /* Qx*/
    const Word16 idx,                 /* i: quantizer index Q0*/
    const Word16 qlow,                /* i: lowest codebook entry (index 0) Qx*/
    const Word16 delta                /* i: quantization step Qy*/
);

Word16 gain_dequant_fx(             /* o: decoded gain */
    Word16 index,                   /* i: quantization index */
    const Word16 min,               /* i: value of lower limit */
    const Word16 max,               /* i: value of upper limit */
    const Word16 bits,              /* i: number of bits to dequantize */
    Word16 *expg
);

void Es_pred_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    Word16 *Es_pred,          /* o  : predicited scaled innovation energy       Q8*/
    const Word16 coder_type,        /* i  : coder type                                  */
    const  Word32 core_brate        /* i  : core bitrate                               */
);

void gain_dec_tc_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word32  core_brate_fx,    /* i  : core bitrate                        */
    const Word16 *code_fx,          /* i  : algebraic code excitation           */
    const Word16 L_frame_fx,        /* i  : length of the frame                 */
    const Word16 i_subfr_fx,        /* i  : subframe number                     */
    const Word16 tc_subfr_fx,       /* i  :  TC subframe index                  */
    const Word16 Es_pred_fx,        /* i  :  predicted scaled innov. energy     */
    Word16 *gain_pit_fx,      /* o  : pitch gain                          */
    Word32 *gain_code_fx,     /* o  : Quantized codeebook gain            */
    Word16 *gain_inov_fx,     /* o  : unscaled innovation gain            */
    Word32 *norm_gain_code_fx /* o  : norm. gain of the codebook excit.   */
);

void gain_dec_mless_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                   */
    const Word32  core_brate_fx,      /* i  : core bitrate                              */
    const Word16 L_frame_fx,          /* i  : length of the frame                       */
    const Word16 coder_type_fx,       /* i  : coding type                               */
    const Word16 i_subfr_fx,          /* i  : subframe number                           */
    const Word16 tc_subfr_fx,         /* i  : TC subframe index                         */
    const Word16 *code_fx,            /* i  : algebraic code excitation                 */
    const Word16 Es_pred_fx,          /* i  : predicted scaled innov. energy            */
    Word16 *gain_pit_fx,        /* o  : Quantized pitch gain                   Q14*/
    Word32 *gain_code_fx,       /* o  : Quantized codeebook gain               Q16*/
    Word16 *gain_inov_fx,       /* o  : unscaled innovation gain               Q12*/
    Word32 *norm_gain_code_fx   /* o  : norm. gain of the codebook excitation  Q16*/
);

void pre_echo_att_fx(
    Word32 *Last_frame_ener_fx,     /* i/o: Energy of the last frame         2*Q_new+1*/
    Word16 *exc_fx,                 /* i/o: Excitation of the current frame  Q_new*/
    const Word16 gsc_attack_flag_fx,      /* i  : flag signalling attack encoded by AC mode (GSC) */
    const Word16 Q_new
    ,const Word16 last_coder_type_fx       /* i  : Last coding mode */
);

void gain_dec_lbr_fx(
    Decoder_State_fx *st_fx,              /* i/o: decoder state structure                                         */
    const Word32  core_brate,           /* i  : core bitrate                                                    */
    const Word16 coder_type,            /* i  : coding type                                                     */
    const Word16 i_subfr,               /* i  : subframe index                                                  */
    const Word16 *code_fx,              /* i  : algebraic excitation                                         Q12*/
    Word16 *gain_pit_fx,          /* o  : quantized pitch gain                                         Q14*/
    Word32 *gain_code_fx,         /* o  : quantized codebook gain                                      Q16*/
    Word16 *gain_inov_fx,         /* o  : gain of the innovation (used for normalization)              Q12*/
    Word32 *norm_gain_code_fx,    /* o  : norm. gain of the codebook excitation                        Q16*/
    Word32 gc_mem[],              /* i/o: gain_code from previous subframes                               */
    Word16 gp_mem[]               /* i/o: gain_pitch from previous subframes                              */
);

void lp_gain_updt_fx(
    const Word16 i_subfr,                    /* i  :  subframe number      Q0    */
    const Word16 gain_pit,                   /* i  : Decoded gain pitch    Q14   */
    const Word32 norm_gain_code,             /* i  : Normalised gain code   Q16  */
    Word16 *lp_gainp,                  /* i/o: LP-filtered pitch gain(FEC) Q14 */
    Word16 *lp_gainc,                  /* i/o: LP-filtered code gain (FEC) Q3 */
    const Word16 L_frame                     /* i  : length of the frame         */
);

void pit_Q_dec_fx(
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode          */
    const Word16 pitch_index,  /* i  : pitch index                             */
    const Word16 nBits,        /* i  : # of Q bits                             */
    const Word16 delta,        /* i  : Half the CL searched interval           */
    const Word16 pit_flag,     /* i  : absolute(0) or delta(1) pitch Q         */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) Q limits   */
    Word16 *T0,          /* o  : integer pitch lag                       */
    Word16 *T0_frac,     /* o  : pitch fraction                          */
    Word16 *T0_min,      /* i/o: delta search min                        */
    Word16 *T0_max       /* i/o: delta search max                        */
);

void abs_pit_dec_fx(
    const Word16 fr_steps,    /* i:   fractional resolution steps (0, 2, 4)    */
    Word16 pitch_index, /* i:   pitch index                              */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) limits     */
    Word16 *T0,         /* o:   integer pitch lag                        */
    Word16 *T0_frac     /* o:   pitch fraction                           */
);

void limit_T0_fx(
    const Word16 L_frame,  /* i  : length of the frame                                  */
    const Word16 delta,    /* i  : Half the close-loop searched interval                */
    const Word16 pit_flag,     /* i  : selecting absolute(0) or delta(1) pitch quantization */
    const Word16 limit_flag,   /* i  : flag for Q limits (0=restrained, 1=extended)         */
    const Word16 T0,       /* i  : rough pitch estimate around which the search is done */
    const Word16 T0_frac,  /* i  : pitch estimate fractional part                       */
    Word16 *T0_min,  /* o  : lower pitch limit                                    */
    Word16 *T0_max   /* o  : higher pitch limit                                   */
);

void delta_pit_dec_fx(
    const Word16 fr_steps,                   /* i  : fractional resolution steps (0, 2, 4)   */
    const Word16 pitch_index,                /* i  : pitch index                             */
    Word16 *T0,                        /* o  : integer pitch lag                       */
    Word16 *T0_frac,                   /* o  : pitch fraction                          */
    const Word16 T0_min                      /* i  : delta search min                        */
);

Word16 pit_decode_fx(                        /* o  : floating pitch value                    */
    Decoder_State_fx *st_fx,                     /* i/o: decoder state structure                 */
    const Word32  core_brate,                /* i  : core bitrate                            */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode          */
    const Word16 L_frame,                    /* i  : length of the frame                     */
    Word16 i_subfr,                    /* i  : subframe index                          */
    const Word16 coder_type,                 /* i  : coding type                             */
    Word16 *limit_flag,                /* i/o: restrained(0) or extended(1) Q limits   */
    Word16 *T0,                        /* o  : close loop integer pitch                */
    Word16 *T0_frac,                   /* o  : close loop fractional part of the pitch */
    Word16 *T0_min,                    /* i/o: delta search min for sf 2 & 4           */
    Word16 *T0_max,                    /* i/o: delta search max for sf 2 & 4           */
    const Word16 L_subfr                     /* i  : subframe length                         */
);

void pred_lt4(
    const Word16 excI[],                     /* in: excitation buffer */
    Word16 excO[],                     /* out: excitation buffer */
    Word16 T0,                         /* input : integer pitch lag */
    Word16 frac,                       /* input : fraction of lag   */
    Word16 L_subfr,                    /* input : subframe size     */
    const Word16 *win,                       /* i  : interpolation window     */
    const Word16 nb_coef,                    /* i  : nb of filter coef        */
    const Word16 up_sample                   /* i  : up_sample        */
);

void lp_filt_exc_dec_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    const Word16 codec_type,                 /* i  : coder type                                      */
    const Word32  core_brate,                /* i  : core bitrate                                    */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode                  */
    const Word16 coder_type,                 /* i  : coding type                                     */
    const Word16 i_subfr,                    /* i  : subframe index                                  */
    const Word16 L_subfr,                    /* i  : subframe size                                   */
    const Word16 L_frame,                    /* i  : frame size                                      */
    Word16 lp_flag,                    /* i  : operation mode signalling                       */
    Word16 *exc
);

void inov_decode_fx(
    Decoder_State_fx *st_fx,                   /* i/o: decoder state structure                     */
    const Word32  core_brate,                /* i  : core bitrate                                */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode              */
    const Word16 L_frame,                    /* i  : length of the frame                         */
    const Word16 coder_type,                 /* i  : coding type                                 */
    const Word16 sharpFlag,                  /* i  : formant sharpening flag                     */
    const Word16 i_subfr,                    /* i  : subframe index                              */
    const Word16 tc_subfr,                   /* i  : TC subframe index                           */
    const Word16 *p_Aq,                      /* i  : LP filter coefficients Q12                  */
    const Word16 tilt_code,                  /* i  : tilt of the excitation of previous subframe Q15 */
    const Word16 pt_pitch,                   /* i  : pointer to current subframe fractional pitch Q6*/
    Word16 *code                       /* o  : algebraic excitation                        */
);

void dec_acelp_1t64_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 code[]                       /* o:   algebraic (fixed) codebook excitation Q12*/
);

void dec_acelp_2t32_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 code[]                   /* o:   algebraic (fixed) codebook excitation */
);

void dec_acelp_4t64_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    Word16 nbbits,                  /* i  : number of bits per codebook               */
    Word16 code[],                  /* o  : algebraic (fixed) codebook excitation  Q9*/
    const Word16 Opt_AMR_WB
);

void weight_a_lc_fx(
    const Word16 a[],         /* i:  LP filter coefficients           Q12 */
    Word16 ap[],        /* o:  weighted LP filter coefficients  Q12 */
    const Word16 *gammatbl,   /* i:  weighting factor                 Q15 */
    const Word16 m            /* i:  order of LP filter               Q0  */
);

void weight_a_fx(
    const Word16 a[],         /* i:  LP filter coefficients           Q12 */
    Word16 ap[],        /* o:  weighted LP filter coefficients  Q12 */
    const Word16 gamma,       /* i:  weighting factor                 Q15 */
    const Word16 m            /* i:  order of LP filter               Q0  */
);

void Residu3_lc_fx(
    const Word16 a[],   /* i  :   prediction coefficients                 Q12 */
    const Word16 m,     /* i  :   order of LP filter                      Q0  */
    const Word16 x[],   /* i  :   input signal (usually speech)           Qx  */
    Word16 y[],   /* o  :   output signal (usually residual)        Qx  */
    const Word16 lg,    /* i  :   vector size                             Q0  */
    const Word16 shift  /* i  :   0=residu2, 1=residu                         */
);

void Residu3_10_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-10..-1] are needed)    */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i :   vector size                            Q0  */
    const Word16 shift  /* i :   0=residu2, 1=residu                        */
);

void Residu3_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-M..-1] are needed)     */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i  :   vector size                           Q0  */
    const Word16 shift  /* i  :   0=residu2, 1=residu                       */
);

void pre_exc_fx(
    const Word16 Opt_AMR_WB,            /* i  : flag indicating AMR-WB IO mode                   */
    const Word16 *speech,               /* i  : input speech                                     Q_new-1*/
    const Word16 *p_Aq,                 /* i  : 12k8 Lp coefficient                              Q12*/
    const Word16 *p_A,                  /* i  : unquantized A(q) filter with bandwidth expansion Q12*/
    const Word16 coder_type,            /* i  : coding type                                      */
    const Word16 i_subfr,               /* i  : current sub frame indicator                      */
    Word16 *Ap,                   /* o  : weighted LP filter coefficients                  Q12*/
    const Word16 *res,                  /* i  : residual signal                                  Q_new*/
    Word16 *exc,                  /* o  : excitation signal                                Q_new*/
    Word16 *h1,                   /* o  : impulse response of weighted synthesis filter    Q(14+shift)*/
    Word16 *h2,                   /* o  : impulse response of weighted synthesis filter    Q(12+shift), for codebook search*/
    Word16 *xn,                   /* o  : close-loop Pitch search target vector            Q_new-1+shift*/
    Word16 *cn,                   /* o  : target vector in residual domain                 Q_new*/
    Word16 *mem_syn,              /* i/o: memory of the synthesis filter                   Q_new-1*/
    Word16 *mem_w0,               /* i/o: weighting filter denominator memory              Q_new-1*/
    const Word16 L_subfr,               /* i  : subframe length                                  */
    Word16 shift
);

void encod_audio_fx(
    Encoder_State_fx *st_fx,              /* i/o: State structure                                 */
    LPD_state   *mem,                /* i/o: acelp memories                                  */
    const Word16 speech[],            /* i  : input speech                              Q_new */
    const Word16 Aw_fx[],             /* i  : weighted A(z) unquantized for subframes         */
    const Word16 Aq_fx[],             /* i  : 12k8 Lp coefficient                             */
    const Word16 T_op[],              /* i  : open loop pitch                                 */
    const Word16 voicing[],           /* i  : voicing                                   Q15   */
    const Word16 *res,                /* i  : residual signal                           Q_new */
    Word16 *synth,              /* i/o: core synthesis                            Q-1   */
    Word16 *exc,                /* i/o: current non-enhanced excitation           Q_new */
    Word16 *pitch_buf,          /* i/o: floating pitch values for each subframe   Q6    */
    Word16 *voice_factors,      /* o  : voicing factors                           Q15   */
    Word16 *bwe_exc,            /* o  : excitation for SWB TBE                    Q0    */
    const Word16 gsc_attack_flag,     /* i  : Flag that point to an attack coded with AC mode (GSC)  */
    const Word16 coder_type,          /* i  : coding type                                     */
    Word16 *lsf_new,            /* i  : current frame ISF vector                        */
    Word16 *tmp_noise,          /* o  : noise energy                                    */
    Word16 Q_new,
    Word16 shift
);

Word16 pit_encode_fx(              /* o  : Fractional pitch for each subframe         */
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure                    */
    const Word32 core_brate,       /* i  : core bitrate                               */
    const Word16 Opt_AMR_WB,       /* i  : flag indicating AMR-WB IO mode             */
    const Word16 L_frame,          /* i  : length of the frame                        */
    const Word16 coder_type,       /* i  : coding type                                */
    Word16 *limit_flag,      /* i/o: restrained(0) or extended(1) Q limits      */
    const Word16 i_subfr,          /* i  : subframe index                             */
    Word16 *exc,             /* i/o: pointer to excitation signal frame         */
    const Word16 L_subfr,          /* i  : subframe length                            */
    const Word16 *T_op,            /* i  : open loop pitch estimates in current frame */
    Word16 *T0_min,          /* i/o: lower limit for close-loop search          */
    Word16 *T0_max,          /* i/o: higher limit for close-loop search         */
    Word16 *T0,              /* i/o: close loop integer pitch                   */
    Word16 *T0_frac,         /* i/o: close loop fractional part of the pitch    */
    const Word16 *h1,              /* i  : weighted filter input response             */
    const Word16 *xn               /* i  : target vector                              */
);

Word32 dotp_fx(                     /* o  : dot product of x[] and y[]    */
    const Word16  x[],            /* i  : vector x[]                    */
    const Word16  y[],            /* i  : vector y[]                    */
    const Word16  n,              /* i  : vector length                 */
    Word16 * exp                   /* (o)    : exponent of result (0..+30)       */
);

Word32 syn_kern_16(
    Word32 L_tmp,
    const Word16 a[],
    const Word16 y[]
);

void syn_filt_s_lc_fx(
    const Word16 shift,             /* i  : scaling to apply        Q0   */
    const Word16 a[],               /* i  : LP filter coefficients  Q12  */
    const Word16 x[],               /* i  : input signal            Qx   */
    Word16 y[],               /* o  : output signal           Qx-s */
    const Word16 lg                 /* i  : size of filtering       Q0   */
);

void acelp_2t32_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                       */
    const Word16 dn[],              /* i  : corr. between target and h[].                 */
    const Word16 h[],               /* i  : impulse response of weighted synthesis filter */
    Word16 code[],            /* o  : algebraic (fixed) codebook excitation         */
    Word16 y[]                /* o  : filtered fixed codebook excitation            */
);

Word16 acelp_4t64_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    Word16 dn[],              /* i  : corr. between target and h[].                  */
    const Word16 cn[],              /* i  : residual after long term prediction            Q_new*/
    const Word16 H[],               /* i  : impulse response of weighted synthesis filter  Q12*/
    Word16 R[],               /* i  : autocorrelation values                         */
    const Word16 acelpautoc,        /* i  : autocorrealtion flag                           */
    Word16 code[],            /* o  : algebraic (fixed) codebook excitation          Q9*/
    Word16 y[],               /* o  : filtered fixed codebook excitation             Q9*/
    Word16 nbbits,            /* i  : number of bits per codebook                    */
    const Word16 cmpl_flag,         /* i  : coomplexity reduction flag                     */
    const Word16 Opt_AMR_WB         /* i  : flag indicating AMR-WB IO mode                 */
);

void acelp_1t64_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure                       */
    const Word16 dn[],             /* i  : corr. between target and h[].                 */
    const Word16 h[],              /* i  : impulse response of weighted synthesis filter */
    Word16 code[],           /* o  : algebraic (fixed) codebook excitation         */
    Word16 y[]               /* o  : filtered fixed codebook excitation            */
);

void cb_shape_fx(
    const Word16 preemphFlag,          /* i  : flag for pre-emphasis                           */
    const Word16 pitchFlag,            /* i  : flag for pitch sharpening                       */
    const Word16 scramblingFlag,       /* i  : flag for phase scrambling                       */
    const Word16 sharpFlag,            /* i  : flag for formant sharpening                     */
    const Word16 formantTiltFlag,      /* i  : flag for formant tilt                           */
    const Word16 g1,                   /* i  : formant sharpening numerator weighting          */
    const Word16 g2,                   /* i  : formant sharpening denominator weighting        */
    const Word16 *p_Aq,                /* i  : LP filter coefficients                          */
    Word16 *code,                /* i/o: signal to shape                                 */
    const Word16 tilt_code,            /* i  : tilt of code                                    */
    const Word16 pt_pitch              /* i  : pointer to current subframe fractional pitch    */
);

void corr_xh_fx(
    const Word16 x[],  /* i  : target signal                                   */
    Word16 dn[], /* o  : correlation between x[] and h[]                 */
    const Word16 h[]   /* i  : impulse response (of weighted synthesis filter) */
);

void corr_xh_jopt_fx(
    const Word16 h[],   /* i  : impulse response of weighted synthesis filter  Q12 */
    const Word16 x[],   /* i  : target vector                                  Q0  */
    Word16 dn[],  /* o  : correlation between target and h[]             Q12 */
    Word16 gain,  /* i  : Unquantized ACB gain                           Q14 */
    Word16 clip,  /* i  : clip                                           Q0  */
    Word16 shift  /* i  : scaling of x                                   Q0  */
);

void enhancer_fx(
    const Word32 core_brate,    /* i  : decoder bitrate                         */
    const Word16 Opt_AMR_WB,    /* i  : flag indicating AMR-WB IO mode          */
    const Word16 coder_type,    /* i  : coder type                              */
    const Word16 i_subfr,       /* i  : subframe number                         */
    const Word16 L_frame,       /* i  : frame size                              */
    const Word16 voice_fac,     /* i  : subframe voicing estimation         Q15 */
    const Word16 stab_fac,      /* i  : LP filter stablility measure        Q15 */
    Word32 norm_gain_code,/* i  : normalised innovative cb. gain      Q16 */
    const Word16 gain_inov,     /* i  : gain of the unscaled innovation     Q12 */
    Word32 *gc_threshold, /* i/o: gain code threshold                 Q16 */
    Word16 *code,         /* i/o: innovation                          Q12 */
    Word16 *exc2,         /* i/o: adapt. excitation/total exc.       Q_exc*/
    const Word16 gain_pit,      /* i  : quantized pitch gain                Q14 */
    struct dispMem_fx *dm_fx,   /* i/o: phase dispersion algorithm memory       */
    const Word16 Q_exc          /* i  : Q of the excitation                     */
);

Word16 Rescale_exc(
    Word16 dct_post_old_exc_fx[],   /* i/o: Music post processing memory */
    Word16 exc[],                   /* i/o: excitation to rescale           Q_exc */
    Word16 bwe_exc[],
    Word16 *last_exc_dct_in,
    Word16 lg,                      /* i  : frame size                            */
    Word16 lg32,
    Word32 L_gain_code,             /* i  : decoded codebook gain           Q16   */
    Word16 *sQ_exc,                 /* i/o: Excitation scaling factor             */
    Word16 *sQsubfr,                /* i/o: Past excitation scaling factors       */
    Word16 exc2[],                  /* o  : local excitation vector               */
    Word16 i_subfr,                 /* i  : subframe number                       */
    const Word16 coder_type
);

void Prep_music_postP_fx(
    Word16 exc_buffer_in[],  /* i/o: excitation buffer   Q_exc*/
    Word16 dct_buffer_out[], /* o  : DCT output buffer   (qdct)*/
    Word16 filt_lfE[],       /* i/o: long term spectrum energy Q15 */
    const Word16 last_core,        /* i  : last core  */
    const Word16 *pitch_buf,       /* i  : current frame pitch information Q6*/
    Word16 *LDm_enh_lp_gbin, /* o  : smoothed suppression gain, per bin FFT Q15*/
    const Word16 Q_exc,            /* i  : excitation scaling         */
    Word16 *qdct             /* o  : Scaling factor of dct coefficient */
);

void Post_music_postP_fx(
    Word16 dct_buffer_in[],  /* i/o: excitation buffer */
    Word16 *exc2,            /* i/o: Current excitation to be overwriten */
    const Word16 *mem_tmp,         /* i  : previous frame synthesis memory     */
    Word16 *st_mem_syn2,     /* i/o: current frame synthesis memory      */
    const Word16 *Aq,              /* i  : LPC filter coefficients             */
    Word16 *syn,             /* i/o: 12k8 synthesis                      */
    Word16 *Q_exc,           /* i  : excitation scaling                  */
    Word16 *prev_Q_syn,      /* i  : previsous frame synthesis scaling   */
    Word16 *Q_syn,           /* i  : Current frame synthesis scaling     */
    Word16 *mem_syn_clas_estim_fx,  /* i  : old 12k8 synthesis used for frame classification*/
    const Word16 IsIO,             /* i: Flag to indicate IO mode */
    Word16 *mem_deemph,         /* i/o: speech deemph filter memory                 */
    Word16 *st_pst_old_syn_fx,        /* i/o:  psfiler                                     */
    Word16 *st_pst_mem_deemp_err_fx,  /* i/o:  psfiler                                     */
    Word16 *mem_agc,
    PFSTAT *pf_stat,            /* i/o:  All memories related to NB post filter      */
    const Word16 *tmp_buffer       /* tmp_buffer in Q-1 */
    ,Word16 *mem_tmp2         /* Temporary memory used with scale_syn */
);

void unscale_AGC(
    const Word16 x[],
    const Word16 Qx,
    Word16 y[],
    Word16 mem[],
    const Word16 n
);

void LD_music_post_filter_fx(
    const Word16 dtc_in[],         /* i   : input synthesis                       Qdct   */
    Word16 dtc_out[],        /* o   : output synthesis                      Qdct   */
    const Word32 core_brate,       /* i   : core bitrate                             Q0  */
    Word16 bfi,              /* i   : Bad frame indicator                      Q0  */
    Word16 *last_music_flag, /* i/o : Previous music detection ouptut          Q0  */
    Word16 *last_bfi_cnt,    /* i/o : number of frame since last bfi           Q0  */
    Word16 *thresh,          /* i/o : Detection thresold                       Q0  */
    Word16 *nb_thr_1,        /* i/o : Number of consecutives frames of level 1 Q0  */
    Word16 *nb_thr_3,        /* i/o : Number of consecutives frames of level 3 Q0  */
    Word16 *lt_diff_etot,    /* i/o : Long term total energy variation         Q8  */
    Word16 *mem_etot,        /* i/o : Total energy memory                      Q8  */
    const Word16 min_ns_gain,      /* i   : minimum gain for inter-harm noise red.   Q15 */
    Word32 bckr[],           /* i/o : per band bckgnd. noise energy estimate     */
    Word32 enro[],           /* i/o : per band old input energy                  */
    Word32 lf_EO[],          /* i/o : old per bin E for previous half frame    2*Qdct+10  */
    Word16 lp_gbin[],        /* i/o : smoothed suppression gain, per FFT bin   Q15  */
    Word16 *filt_lfE,        /* i   : post filter weighting coefficient        Q15  */
    Word16 *last_nonfull_music, /* i : Number of frames sinces la "speech like" frame Q0*/
    Word16 *Old_ener_Q,      /* i/o : Old energy scaling factor           */
    const Word16 coder_type,       /* i   : Coder type : -1 in case of IO            Q0  */
    const Word16 Last_coder_type,  /* i   : input scaling                            Q0  */
    const Word16 Qdct              /* i   : input scaling                            Q0  */
);

void Rescale_mem(
    const Word16 Q_exc,          /* i   : current excitation scaling (>=0)          */
    Word16 *prev_Q_syn,    /* i/o  : scaling factor of previous frame          */
    Word16 *Q_syn,         /* i/o  : scaling factor of frame                   */
    Word16 *mem_syn2,      /* i/o  : modified synthesis memory                 */
    Word16 *mem_syn_clas_estim_fx, /* i/o  : old 12k8 core memory for classification */
    const Word16 MaxScaling,     /* i: Minimal difference between excitation scaling and synthesis scaling */
    Word16 *mem_deemph,    /* i/o: speech deemph filter memory                 */
    Word16 *pst_old_syn,   /* i/o:  psfiler                                     */
    Word16 *pst_mem_deemp_err, /* i/o:  psfiler                                     */
    Word16 *mem_agc,
    PFSTAT *pf_stat,       /* i/o:  All memories related to NB post filter      */
    const Word16 Vad_flag,
    const Word16 *tmp_buffer     /* tmp_buffer in Q-1 */
);

void Scale_sig(
    Word16 x[],                   /* i/o: signal to scale                 Qx        */
    const Word16 lg,                    /* i  : size of x[]                     Q0        */
    const Word16 exp0                   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void isf2lsf_fx(
    const Word16 *isf,                 /* i  : ISF vector                        */
    Word16 *lsf,                 /* o  : LSF vector                        */
    Word16 *stable_lsp           /* i/o: stable LSP filter coefficients    */
);

void isp2lsp_fx(
    const Word16 *isp,                 /* i  : LSP vector                        */
    Word16 *lsp,                 /* o  : ISP filter coefficients           */
    Word16 *stable_lsp,          /* i/o: stable LSP filter coefficients    */
    const Word16 m                     /* i  : order of LP analysis              */
);

void lsf2isf_fx(
    const Word16 *lsf,              /* i  : LSF vector                        */
    Word16 *isf,              /* o  : ISF vector                        */
    Word16 *stable_isp,       /* i/o: stable ISP filter coefficients    */
    const Word16 m                  /* i  : order of LP analysis              */
);

void lsp2isp_fx(
    const Word16 *lsp,              /* i  : LSP vector                        */
    Word16 *isp,              /* o  : ISP filter coefficients           */
    Word16 *stable_isp,       /* i/o: ISP filter coefficients           */
    const Word16 m                  /* i  : order of LP analysis              */
);

void updt_IO_switch_dec_fx(
    const Word16 output_frame,       /* i  : output frame length                */
    Decoder_State_fx *st_fx          /* o  : Decoder static variables structure */
);

void prep_tbe_exc_fx(
    const Word16 L_frame_fx,        /* i : length of the frame */
    const Word16 i_subfr_fx,        /* i : subframe index */
    const Word16 gain_pit_fx,       /* i : Pitch gain Q14*/
    const Word32 gain_code_fx,      /* i : algebraic codebook gain 16+Q_exc*/
    const Word16 code_fx[],         /* i : algebraic excitation Q9*/
    const Word16 voice_fac_fx,      /* i : voicing factor Q15*/
    Word16 *voice_factors_fx, /* o : TBE voicing factor Q15*/
    Word16 bwe_exc_fx[],      /* i/o: excitation for TBE Q_exc*/
    const Word16 gain_preQ_fx,      /* i : prequantizer excitation gain */
    const Word16 code_preQ_fx[],    /* i : prequantizer excitation */
    const Word16 Q_exc,             /* i : Excitation, bwe_exc Q-factor */
    Word16 T0,                /* i : integer pitch variables Q0 */
    Word16 T0_frac,           /* i : Fractional pitch variables Q0*/
    const Word16 coder_type,        /* i : coding type */
    Word32 core_brate         /* i :core bitrate */
);

void interp_code_5over2_fx(
    const Word16 inp_code[],          /* i  : input vector                */
    Word16 interp_code[],       /* o  : output vector               */
    const Word16 inp_length           /* i  : length of input vector      */
);

void lsp2lsf_fx(
    const Word16 lsp[],             /* i  : lsp[m] (range: -1<=val<1)             Q15*/
    Word16 lsf[],             /* o  : lsf[m] normalized (range: 0.0<=val<=0.5)    Q(x2.56)*/
    const Word16 m,                 /* i  : LPC order                    Q0*/
    Word32 int_fs
);

void lsf_enc_fx(
    Encoder_State_fx *st_fx,        /* i/o: state structure                             */
    const Word16 L_frame,           /* i  : length of the frame                         */
    const Word16 coder_type,        /* i  : coding type                                 */
    Word16 *lsf_new,          /* o  : quantized LSF vector                        */
    Word16 *lsp_new,          /* i/o: LSP vector to quantize/quantized            */
    Word16 *lsp_mid,          /* i/o  : mid-frame LSP vector                      */
    Word16 *Aq,               /* o  : quantized A(z) for 4 subframes              */
    Word16 *stab_fac,         /* o  : LSF stability factor                        */
    const Word16 Nb_ACELP_frames,
    const Word16 Q_new
);

void lsf_mid_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word16 int_fs,            /* i  : internal (ACELP) sampling frequency  */
    Word16 qlsp0[],           /* i  : quantized LSPs from frame beginning  Q15*/
    Word16 qlsp1[],           /* i  : quantized LSPs from frame endSQ15*/
    Word16 coder_type,        /* i  : Coder type */
    Word16 qlsp[],            /* o  : quantized LSPs    Q15*/
    const Word32 core_brate,        /* i  : core bitrate */
    Word16 ppp_mode,
    Word16 nelp_mode,
    Word16 prev_bfi,
    Word16 *mid_lsf_int,
    Word16 safety_net
);

Word16 lsf_stab_fx(             /* o  : LP filter stability   Q15*/
    const Word16 *lsf,          /* i  : LSF vector            Q(x2.56)*/
    const Word16 *lsfold,       /* i  : old LSF vector        Q(x2.56)*/
    const Word16 Opt_AMR_WB,    /* i  : flag indicating AMR-WB IO mode */
    const Word16 L_frame        /* i  : frame length */
);

void lsf_allocate_fx(
    const Word16 nBits,       /* i  : Number of bits to use for quantization     */
    const Word16 framemode,   /* i  : ISF quantizer mode                         */
    const Word16 framemode_p, /* i  : ISF quantizer mode predmode (mode_lvq_p)   */
    Word16 *stages0,    /* o  : Number of stages for safety-net quantizer  */
    Word16 *stages1,    /* o  : Number of stages for predictive quantizer  */
    Word16 levels0[],   /* o  : Number of vectors for each stage for SFNET */
    Word16 levels1[],   /* o  : Number of vectors for each stage for pred  */
    Word16 bits0[],     /* o  : Number of bits for each stage safety net   */
    Word16 bits1[]      /* o  : Number of bits for each stage pred         */
);

void vq_dec_lvq_fx (
    Word16 sf_flag,        /* i  : safety net flag                           */
    Word16 x[],            /* o  : Decoded vector                    Q(x2.56)*/
    Word16 indices[],      /* i  : Indices                                   */
    Word16 stages,         /* i  : Number of stages                          */
    Word16 N,              /* i  : Vector dimension                          */
    Word16 mode,           /* (i): mode_lvq, or mode_lvq_p           */
    Word16 no_bits,        /* (i): no. bits for lattice             */
    Word32 *p_offset_scale1,
    Word32 *p_offset_scale2,
    Word32 *p_offset_scale1_p,
    Word32 *p_offset_scale2_p,
    Word16 *p_no_scales,
    Word16 *p_no_scales_p
);

Word16 qlsf_ARSN_tcvq_Dec_16k_fx (
    Word16 *y_fx,                           /* o  : Quantized LSF vector    */
    Word16 *indice,                         /* i  : Indices                 */
    const Word16 nBits                      /* i  : number of bits          */
);

void tcvq_Dec_fx(Word16 *ind,
                 /*float *d_out, */
                 Word16 *d_out_fx,
                 Word16 safety_net
                );

Word32 qlsf_ARSN_tcvq_Enc_16k_fx(
    const Word16 *x_fx,             /* i  : Vector to be encoded  x2.65  */
    Word16 *y_fx,             /* o  : Quantized LSF vector  x2.65  */
    Word16 *indice,           /* o  : Indices                 */
    const Word16 *w_fx,             /* i  : LSF Weights           Q10  */
    const Word16 nBits,             /* i  : number of bits          */
    Word16 safety_net         /* i : safety_net flag */
);

void Vr_add(
    const Word16 *in1,              /* i  : Input vector 1                                   */
    const Word16 *in2,              /* i  : Input vector 2                                   */
    Word16 *out,              /* o  : Output vector that contains vector 1 + vector 2  */
    Word16 Len                /* i  : Vector lenght                                    */
);

void Vr_subt(
    const Word16 *in1,              /* i  : Input vector 1                                   */
    const Word16 *in2,              /* i  : Input vector 2                                   */
    Word16 *out,              /* o  : Output vector that contains vector 1 + vector 2  */
    Word16 Len                /* i  : Vector lenght                                    */
);

void index_lvq_fx (
    Word16 *quant,                  /* i  : codevector to be indexed (2 8-dim subvectors)  Q13*/
    Word16   *idx_lead,             /* i  : leader class index for each subvector */
    Word16   *idx_scale,            /* i  : scale index for each subvector */
    Word16   mode,                  /* i  : integer signalling the quantizer structure for the current bitrate */
    Word16 *index,                  /* o  : encoded index (represented on 3 short each with 15 bits ) */
    Word32 * p_offset_scale1,
    Word32 * p_offset_scale2,
    Word16 * p_no_scales
);

void multiply32_32_64_fx(
    Word32 x,
    Word32 y,
    Word32 *res
);

void permute_fx(
    Word16 *pTmp1,         /* i/o: vector whose components are to be permuted */
    const Word16 *perm     /* i  : permutation info (indexes that should be interchanged), max two perms */
);

Word32 mslvq_fx (
    Word16 *pTmp,                            /* i  : M-dimensional input vector */
    Word16 *quant,                           /* o  : quantized vector */
    Word16 *cv_out,                          /* o  : corresponding 8-dim lattice codevectors (without the scaling) */
    Word16 *idx_lead,                        /* o  : leader index for each 8-dim subvector  */
    Word16 *idx_scale,                       /* o  : scale index for each subvector */
    Word16 *w,                               /* i  : weights for LSF quantization */
    Word16 mode,                             /* i  : number indicating the coding type (V/UV/G...)*/
    Word16 mode_glb,                         /* i  : LVQ coding mode */
    Word16  pred_flag,                        /* i  : prediction flag (0: safety net, 1 - predictive )*/
    Word16 no_scales[][2]
);

Word32 mslvq_cng_fx (
    Word16 idx_cv,              /* i  : index of cv from previous stage                   */
    Word16 *pTmp,               /* i  : 16 dimensional input vector                  x2.56*/
    Word16 *quant,              /* o  : quantized vector                      x2.56*/
    Word16 *cv_out,             /* o  : corresponding 8-dim lattice codevectors (without the scaling) Q13*/
    Word16   *idx_lead,         /* o  : leader index for each 8-dim subvector               */
    Word16   *idx_scale,        /* o  : scale index for each subvector                   */
    const Word16 *w,            /* i  : weights for LSF quantization                   Q10*/
    Word16 * no_scales
);

void sort_fx(
    Word16 *r,  /* i/o: Vector to be sorted in place */
    Word16 lo,  /* i  : Low limit of sorting range   */
    Word16 up   /* I  : High limit of sorting range  */
);

void lsf_dec_fx(
    Decoder_State_fx *st_fx,             /* i/o: State structure                             */
    const Word16 tc_subfr,          /* i  : TC subframe index                           */
    const Word16 L_frame,           /* i  : length of the frame                         */
    const Word16 coder_type,        /* i  : coding type                                 */
    const Word16 bwidth,            /* i  : input signal bandwidth                      */
    Word16 *Aq,               /* o  : quantized A(z) for 4 subframes              */
    Word16 *lsf_new,          /* o  : de-quantized LSF vector                     */
    Word16 *lsp_new,          /* o  : de-quantized LSP vector                     */
    Word16 *lsp_mid           /* o  : de-quantized mid-frame LSP vector           */
);

void lsf_end_enc_fx(
    Encoder_State_fx *st,
    const Word16 *lsf,
    Word16 *qlsf,
    Word16 *mem_AR,
    Word16 * mem_MA,
    const Word16 nBits,
    const Word16 coder_type_org,
    const Word16 bwidth,
    Word32 *Bin_Ener,
    Word16 Q_ener,
    const Word32 int_fs,
    Word32 core_brate,
    Word16 *streaklimit,
    Word16 *pstreaklen,
    Word16 force_sf,
    Word16 RF_flag,
    Word16 mode2_flag,
    Word16 * lpc_param,
    Word16 * no_stages,
    Word16 * bits_param_lpc,
    Word16 coder_type_raw
);

void lsf_end_dec_fx(
    Decoder_State_fx *st,
    Word16 mode2_flag,
    const Word16 coder_type_org,
    const Word16 bwidth,
    const Word16 nBits,
    Word16 *qlsf,
    Word16 *mem_AR,
    Word16 *mem_MA,
    const Word32 int_fs,
    Word32 core_brate,
    Word32 *p_offset_scale1,
    Word32 *p_offset_scale2,
    Word32 *p_offset_scale1_p,
    Word32 *p_offset_scale2_p,
    Word16 *p_no_scales,
    Word16 *p_no_scales_p,
    Word16 *safety_net,
    Word16 *lpc_param,
    Word16 * nb_indices
);

Word16 find_pred_mode(
    const Word16 coder_type,
    const Word16 bwidth,
    const Word32 int_fs,
    Word16 * p_mode_lvq,
    Word16 * p_mode_lvq_p,
    Word32 core_brate
);

Word16 xsf_to_xsp(
    Word16 lsf
);

Word16 xsp_to_xsf(
    Word16 lsp
);

void int_lsp4_fx(
    const Word16 L_frame,    /* i  : length of the frame                   */
    const Word16 lsp_old[],  /* i  : LSPs from past frame               Q15*/
    const Word16 lsp_mid[],  /* i  : LSPs from mid-frame          Q15*/
    const Word16 lsp_new[],  /* i  : LSPs from present frame            Q15*/
    Word16 *Aq,        /* o  : LP coefficients in both subframes  Q12*/
    const Word16 m,          /* i  : order of LP filter                    */
    const Word16 clas,       /* i  : signal frame class                    */
    Word16 relax_prev_lsf_interp /* i  : relax prev frame lsf interp after erasure */
);

Word16 modify_Fs_fx(            /* o  : length of output    Q0  */
    const Word16 sigIn_fx[],    /* i  : signal to decimate  Q0  */
    Word16 lg,            /* i  : length of input     Q0  */
    const Word32 fin,           /* i  : frequency of input  Q0  */
    Word16 sigOut_fx[],   /* o  : decimated signal    Q0  */
    const Word32 fout,          /* i  : frequency of output Q0  */
    Word16 mem_fx[],      /* i/o: filter memory       Q0  */
    const Word16 nblp           /* i  : flag indicating if NB low-pass is applied */
);

void addBassPostFilterFx(
    const Word16            *harm_timeIn_Fx,
    Word32                 **rAnalysis_Fx,
    Word32                 **iAnalysis_Fx,
    HANDLE_CLDFB_FILTER_BANK   cldfbBank_bpf_Fx,
    Word32                  *workBuffer,
    const Word16 timeIn_e,
    const Word16             nTimeSlots,
    const Word16             nTimeSlotsTotal,
    const Word16             nBandsTotal,
    CLDFB_SCALE_FACTOR        *cldfb_scale
);

void bass_psfilter_fx(
    const Word16 Opt_AMR_WB,            /* i  : AMR-WB IO flag                   */
    Word16 synth_in_fx[],      /* i  : input synthesis (at 16kHz)        */
    const Word16 L_frame,        /* i  : length of the last frame          */
    Word16 pitch_buf_fx[],      /* i  : pitch for every subfr [0,1,2,3]   */
    Word16 old_syn_fx[],      /* i/o: NBPSF_PIT_MAX                     */
    Word16 *mem_deemph_err,    /* o  : Error deemphasis memory           */
    Word16 *lp_ener,        /* o  : long_term error signal energy     */
    const Word16 bpf_off,        /* i  : do not use BPF when set to 1      */
    Word16 v_stab_fx,        /* i  : stability factor                  */
    Word16 *v_stab_smooth_fx,    /* i/o: smoothed stability factor         */
    Word16 *mem_mean_pit,      /* i/o: average pitch memory              */
    Word16 *Track_on_hist,    /* i/o: History of half frame usage       */
    Word16 *vibrato_hist,      /* i/o: History of frames declared as vibrato*/
    Word16 *psf_att,        /* i/o: Post filter attenuation factor    */
    const Word16 coder_type,      /* i  : coder_type                        */
    Word16 Q_syn,
    Word16 bpf_noise_buf[]        /* o  : BPF error signal (at int_fs)      */
);

void speech_music_classif_fx(
    Encoder_State_fx *st,           /* i/o: state structure                                 */
    Word16 *sp_aud_decision0,
    Word16 *sp_aud_decision1, /* o  : 1st stage speech/music                          */
    Word16 *sp_aud_decision2, /* o  : 2nd stage speech/music                          */
    const Word16 *new_inp,          /* i  : new input signal                                */
    const Word16 *inp,              /* i  : input signal to locate attach position          */
    const Word16 vad_flag,
    const Word16 localVAD,
    const Word16 localVAD_HE_SAD,   /* i  : HE-SAD flag without hangover                    */
    const Word16 pitch[3],          /* i  : open-loop pitch estimate in three subframes     */
    const Word16 voicing[3],        /* i  : voicing estimate in three subframes         Q15 */
    const Word16 lsp_new[M],        /* i  : LSPs in current frame                       Q15 */
    const Word16 cor_map_sum,       /* i  : correlation map sum (from multi-harmonic anal.)Q8*/
    const Word32 epsP[M+1],         /* i  : LP prediciton error                         Q_esp*/
    const Word32 PS[],              /* i  : energy spectrum                     Q_new+QSCALE*/
    const Word16 Etot,              /* i  : total frame energy                          Q8  */
    const Word16 old_cor,           /* i  : max correlation from previous frame         Q15 */
    Word16 *coder_type,       /* i/o: coding type                                     */
    Word16 *attack_flag,      /* o  : flag to indicate if attack is to be treated by TC or GSC */
    Word16 non_sta,           /* i  : unbound non-stationarity for sp/mus classifier */
    Word16 relE,              /* i  : relative frame energy */
    Word16 Q_esp,             /* i  : scaling of esP */
    Word16 Q_inp              /* i  : scaling of input */
    ,Word16 *high_lpn_flag_ptr    /* o  :    noise log prob flag for NOISE_EST        */
    ,Word16 flag_spitch        /* i: flag to indicate very short stable pitch                  */
);

Word32 sub_lsp2lsf_fx(
    const Word16 lsp_i     /* i  : lsp[m] (range: -1<=val<1)                       Q15*/
);
Word16 lsp_convert_poly_fx(
    Word16  w[],           /* i/o: LSP or ISP parameters          */
    const Word16 L_frame,        /* i  : flag for up or down conversion */
    const Word16 Opt_AMRWB       /* i  : flag for the AMR-WB IO mode    */
);

void StableHighPitchDetect_fx(
    Word16 *flag_spitch,       /* o  : flag to indicate very short stable pitch */
    Word16 pitch[],            /* i/o: OL pitch buffer                         */
    const Word16 voicing[],          /* i  : OL pitch gains                          */
    const Word16 wsp[],              /* i  : weighted speech                         */
    const Word16 localVAD,
    Word16 *voicing_sm,        /* i/o: smoothed open-loop pitch gains          */
    Word16 *voicing0_sm,       /* i/o: smoothed high pitch gains               */
    Word16 *LF_EnergyRatio_sm, /* i/o: smoothed [0, 300Hz] relative peak energy*/
    Word16 *predecision_flag,  /* i/o: predecision flag                        */
    Word32 *diff_sm,           /* i/o: smoothed pitch frequency difference     */
    Word32 *energy_sm  ,        /* i/o: smoothed energy around pitch frequency  */
    Word16 Q_new,
    Word16 EspecdB[]
);

void Es_pred_enc_fx(
    Word16 *Es_pred,      /* o  : predicited scaled innovation energy Q8    */
    Word16 *indice,      /* o  : indice of quantization    */
    const Word16 L_frame,      /* i  : length of the frame                       */
    const Word16 *res,         /* i  : residual signal                           */
    const Word16 *voicing,     /* i  : normalized correlation in three 1/2frames */
    const Word16 nb_bits,      /* i  : allocated number of bits                  */
    const Word16 no_ltp,       /* i  : no_ltp flag                               */
    Word16 Q_new         /* i  : Scaling in speech                   Q0    */
);

void calc_residu_fx(
    Encoder_State_fx *st,          /* i/o: state structure                           */
    const Word16 *speech,      /* i  : weighted speech signal                    */
    Word16 *res,         /* o  : residual signal                           */
    const Word16 *p_Aq,        /* i  : quantized LP filter coefficients          */
    const Word16 vad_hover_flag
);
void updt_IO_switch_enc_fx(
    Encoder_State_fx *st,            /* i/o: state structure             */
    const Word16 input_frame       /* i  : input frame length          */
);
/*-------------------------------------------------------------------*
  * pre_proc()
  *
  * Pre-processing (spectral analysis, LP analysis, VAD, OL pitch calculation, coder mode selection, ...)
  *--------------------------------------------------------------------*/

void pre_proc_fx(
    Encoder_State_fx *st,                      /* i/o: encoder state structure                  */
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
    Word16 epsP_h[M+1],                /* o  : LP prediction errors                     */
    Word16 epsP_l[M+1],                /* o  : LP prediction errors                     */
    Word32 epsP[M+1],                /* o  : LP prediction errors                     */
    Word16 lsp_new[M],               /* o  : LSPs at the end of the frame             */
    Word16 lsp_mid[M],               /* o  : LSPs in the middle of the frame          */
    Word16 *coder_type,              /* o  : coder type                               */
    Word16 *sharpFlag,               /* o  : formant sharpening flag                  */
    Word16 *vad_hover_flag,
    Word16 *attack_flag,         /* o  : flag signalling attack encoded by AC mode (GSC)    */
    Word16 *new_inp_resamp16k,       /* o  : new input signal @16kHz, non pre-emphasised, used by the WB TBE/BWE */
    Word16 *Voicing_flag,            /* o  : voicing flag for HQ FEC                  */

    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], /* i/o : cldfb real buffer */
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], /* i/o : cldfb imag buffer */
    CLDFB_SCALE_FACTOR *cldfbScale,     /* o : cldfb scale */
    Word16 *old_exc,            /* i/o: static excitation memory                       */
    Word16 *hq_core_type,          /* o  : HQ core type                             */
    Word16 *Q_new,
    Word16 *shift,
    Word16 *Q_r
);

Word16 mdct_classifier_fx(                /* o: MDCT A/B decision                         */
    const Word16 *Y,                  /* i: re[0], re[1], ..., re[n/2], im[n/2 - 1], im[n/2 - 2], ..., im[1] */
    Encoder_State_fx *st_fx,          /* i/o: Encoder state variable                  */
    Word16 vadflag
    ,Word32 *cldfbBuf_Ener
);

void floating_point_add(
    Word32 *mx, /* io: mantissa of the addend Q31 */
    Word16 *ex, /* io: exponent of the addend Q0  */
    const Word32 my,  /* i:  mantissa of the adder Q31  */
    const Word16 ey   /* i:  exponent of the adder Q0   */
);

void MDCT_selector(
    Encoder_State_fx *st       /* i/o: Encoder State */
    , Word16 sp_floor            /* i  : Noise floor estimate Q7 */
    , Word16 Etot                /* i  : Total energy         Q8 */
    , Word16 cor_map_sum         /* i  : harmonicity factor   Q8 */
    , const Word16 voicing[]     /* i  : voicing factors      Q15*/
    , const Word32 enerBuffer[]  /* i  : CLDFB buffers             */
    , Word16 enerBuffer_exp      /* i  : exponent of enerBuffer  */
    , Word16 vadflag
);

void MDCT_selector_reset(
    Encoder_State_fx *st);            /* i/o: Encoder State */

Word16 minimum_fx(         /* o  : index of the minimum value in the input vector */
    const Word16 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word16 *min_fx   /* o  : minimum value in the input vector              */
);
Word16 minimum_32_fx(         /* o  : index of the minimum value in the input vector */
    const Word32 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word32 *min_fx   /* o  : minimum value in the input vector              */
);
Word16 maximum_32_fx(         /* o  : index of the maximum value in the input vector */
    const Word32 *vec,  /* i  : input vector                                   */
    const Word16 lvec,  /* i  : length of input vector                         */
    Word32 *max   /* o  : maximum value in the input vector              */
);
Word16 maximum_fx(         /* o  : index of the maximum value in the input vector */
    const Word16 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word16 *max_fx   /* o  : maximum value in the input vector              */
);
Word16 Exp32Array(
    const Word16  n,     /* (i): Array size   */
    const Word32  *sx    /* (i): Data array   */
);
Word16 Exp16Array(
    const Word16  n,     /* (i): Array size   */
    const Word16  *sx    /* (i): Data array   */
);

Word32 sum16_32_fx(     /* o  : sum of all vector elements            Qx*/
    const Word16 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
);
Word32 dot_product_mat_fx(    /* o  : the dot product x'*A*x        */
    const Word16  *x,      /* i  : vector x                     Q15 */
    const Word32  *A,      /* i  : matrix A                     Q0*/
    const Word16  m      /* i  : vector & matrix size          */

);

Word16 std_fx(              /* o: standard deviation                    */
    const Word16 *x,          /* i: input vector                          */
    const Word16 len          /* i: length of the input vector            */
);

Word16 stab_est_fx(
    Word16 etot,            /* i   : Total energy of the current frame   */
    Word16 *lt_diff_etot,   /* i/o : Long term total energy variation    */
    Word16 *mem_etot,       /* i/o : Total energy memory                 */
    Word16 *last_bfi_cnt,   /* i/o : number of frame since last bfi      */
    Word16 bfi,             /* i   : Bad frame indicator                 */
    Word16 *nb_thr_3,       /* i/o : Number of consecutives frames of level 3 */
    Word16 *nb_thr_1,       /* i/o : Number of consecutives frames of level 1 */
    Word16 *thresh,         /* i/o : Detection thresold                 */
    Word16 *last_music_flag,/* i/o : Previous music detection ouptut    */
    Word16 vad_flag
);
Word16 var_fx(                /* o: variance of vector                    Qx*/
    const Word16 *x,        /* i: input vector                          Qx*/
    const Word16 Qx,
    const Word16 len          /* i: length of inputvector                 */
);
void conv_fx(
    const Word16 x[],   /* i  : input vector                              Q_new*/
    const Word16 h[],   /* i  : impulse response (or second input vector) Q(15)*/
    Word16 y[],   /* o  : output vetor (result of convolution)      12 bits*/
    const Word16 L      /* i  : vector size                               */
);
void norm_corr_fx(
    const Word16 exc[],        /* i  : excitation buffer                          Q_new*/
    const Word16 xn[],         /* i  : target signal                              Q_new-1+shift*/
    const Word16 h[],          /* i  : weighted synthesis filter impulse response Q(14+shift)*/
    const Word16 t_min,        /* i  : minimum value of searched range            */
    const Word16 t_max,        /* i  : maximum value of searched range            */
    Word16 ncorr[],       /* o  : normalized correlation                     Q15 */
    const Word16 L_subfr       /* i  : subframe size                              */
);
void pit_Q_enc_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode          */
    const Word16 nBits,        /* i  : # of Q bits                             */
    const Word16 delta,        /* i  : Half the CL searched interval           */
    const Word16 pit_flag,     /* i  : absolute(0) or delta(1) pitch Q         */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) Q limits   */
    const Word16 T0,           /* i  : integer pitch lag                       */
    const Word16 T0_frac,      /* i  : pitch fraction                          */
    Word16 *T0_min,      /* i/o: delta search min                        */
    Word16 *T0_max       /* o  : delta search max                        */
);
/*void pit16k_Q_enc_fx( */
/*Encoder_State_fx *st_fx,        */ /* i/o: encoder state structure */
/*    const Word16 nBits,         */ /* i  : # of Q bits                             */
/*    const Word16 i_subfr,       */ /* i  : subframe index                          */
/*    const Word16 limit_flag,    */ /* i  : restrained(0) or extended(1) Q limits   */
/*    const Word16 T0,            */ /* i  : integer pitch lag                       */
/*    const Word16 T0_frac,       */ /* i  : pitch fraction                          */
/*          Word16 *T0_min,       */ /* i/o: delta search min                        */
/*          Word16 *T0_max        */ /* o  : delta search max                        */
/*); */

Word16 pitch_fr4_fx(            /* o  : chosen integer pitch lag                 */
    const Word16 exc[],      /* i  : excitation buffer                          Q_new*/
    const Word16 xn[],       /* i  : target signal                              Q_new-1+shift*/
    const Word16 h[],        /* i  : weighted synthesis filter impulse response Q(14+shift)*/
    const Word16 t0_min,     /* i  : minimum value in the searched range.       Q0*/
    const Word16 t0_max,     /* i  : maximum value in the searched range.       Q0*/
    Word16 *pit_frac,  /* o  : chosen fraction (0, 1, 2 or 3)             */
    const Word16 i_subfr,    /* i  : flag to first subframe                     */
    const Word16 limit_flag, /* i  : flag for limits (0=restrained, 1=extended) */
    const Word16 t0_fr2,     /* i  : minimum value for resolution 1/2           */
    const Word16 t0_fr1,     /* i  : minimum value for resolution 1             */
    const Word16 L_frame,    /* i  : length of the frame                        */
    const Word16 L_subfr     /* i  : size of subframe                           */
);

Word32 var_fx_32(                /* o: variance of vector                    Qx+16*/
    const Word16 *x,        /* i: input vector                          Qx*/
    const Word16 Qx,
    const Word16 len          /* i: length of inputvector                 */
);

Word16 sum16_fx(           /* o  : sum of all vector elements            Qx*/
    const Word16 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
);

Word32 sum32_fx(           /* o  : sum of all vector elements            Qx*/
    const Word32 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
);

Word32 Mean32(              /* o  : mean of the elements of the vector */
    const Word32 in[],      /* i  : input vector                       */
    const Word16 L          /* i  : length of input vector             */
);

void Deemph2(
    Word16 x[],   /* i/o: input signal overwritten by the output   Qx/Qx-1 */
    const Word16 mu,    /* i  : deemphasis factor                        Q15     */
    const Word16 L,     /* i  : vector size                              Q0      */
    Word16 *mem   /* i/o: memory (y[-1])                           Qx-1    */
);

void deemph_fx(
    Word16 *signal,   /* i/o: signal        Qx  */
    const Word16 mu,        /* i  : deemphasis factor   Q15 */
    const Word16 L,         /* i  : vector size         Q0  */
    Word16 *mem       /* i/o: memory (y[-1])      Qx  */
);

void deindex_lvq_fx(
    Word16 *index,                  /* i  : index to be decoded, as an array of 3 short        */
    Word16 *x_lvq,                  /* o  : decoded codevector                             Q(x2.56)*/
    Word16 mode,                    /* i  : LVQ  coding mode (select scales & no_lead ), or idx_cv */
    Word16 sf_flag,                 /* i  : safety net flag                      */
    Word16 no_bits,                 /* i  : number of bits for lattice                */
    Word32 *p_offset_scale1,     /* i  : offset for first subvector                */
    Word32 *p_offset_scale2,       /* i  : offset for the second subvector              */
    Word16 *p_no_scales
);
void Syn_filt_s(
    const Word16 shift,     /* i  : scaling to apply                          Q0   */
    const Word16 a[],       /* i  : LP filter coefficients                    Q12  */
    const Word16 m,         /* i  : order of LP filter                        Q0   */
    const Word16 x[],       /* i  : input signal                              Qx   */
    Word16 y[],       /* o  : output signal                             Qx-s */
    const Word16 lg,        /* i  : size of filtering                         Q0   */
    Word16 mem[],     /* i/o: memory associated with this filtering.    Qx-s */
    const Word16 update     /* i  : 0=no update, 1=update of memory.          Q0   */
);
void hp400_12k8_fx(
    Word16 signal[],  /* i/o: input signal / output is divided by 16 */
    const Word16 lg,        /* i  : lenght of signal                       */
    Word16 mem[]      /* i/o: filter memory [6]                      */
);
Word16 dot_prod_satcontr(const Word16 *x, const Word16 *y, Word16 qx, Word16 qy, Word16 *qo, Word16 len);
void syn_12k8_fx(
    Word16 L_frame,
    const Word16 *Aq,       /* i  : LP filter coefficients                       Q12   */
    const Word16 *exc,     /* i  : input signal                                 Q_exc */
    Word16 *synth,    /* o  : output signal                                Q_syn */
    Word16 *mem,      /* i/o: initial filter states                        Q_syn */
    const Word16 update_m,  /* i  : update memory flag: 0 --> no memory update   Q0    */
    /*                          1 --> update of memory         */
    const Word16 Q_exc,     /* i  : Excitation scaling                           Q0    */
    const Word16 Q_syn      /* i  : Synthesis scaling                            Q0    */
);
void updt_dec_fx(
    Decoder_State_fx *st_fx,             /* i/o: state structure                          */
    const Word16 L_frame,           /* i  : length of the frame                       */
    const Word16 coder_type,        /* i  : coding type                              */
    const Word16 *old_exc_fx,          /* i  : buffer of excitation                     */
    const Word16 *pitch_buf_fx,        /* i  : floating pitch values for each subframe  */
    const Word16 Es_pred,           /* i  : predicited scaled innovation energy      */
    const Word16 *Aq,               /* i  : A(z) quantized for all subframes         */
    const Word16 *lsf_new_fx,          /* i  : current frame LSF vector                 */
    const Word16 *lsp_new_fx,          /* i  : current frame LSP vector                 */
    const Word16 voice_factors[],   /* i  : voicing factors                          */
    const Word16 *old_bwe_exc_fx       /* i  : buffer of excitation                     */
    ,   const Word16 *gain_buf              /*Q14*/
);
Word32 Interpol_lc_fx(        /* o  : interpolated value             Qx+16 */
    const Word16 *x,       /* i  : input vector                     Q0  */
    const Word16 *win,     /* i  : interpolation window             Q14 */
    const Word16 frac,     /* i  : fraction (0..up_samp)            Q0  */
    const Word16 up_samp,  /* i  : upsampling factor                Q0  */
    const Word16 nb_coef   /* i  : number of coefficients           Q0  */
);

Word16 corr_xy1_fx(              /* o  : pitch gain  (0..GAIN_PIT_MAX)         */
    const Word16 xn_1[],      /* i  : target signal                         */
    const Word16 y1_1[],     /* i  : filtered adaptive codebook excitation */
    Word16 g_corr[],    /* o  : correlations <y1,y1>  and -2<xn,y1>   */
    const Word16 L_subfr,     /* i  : vector length                          */
    const Word16 norm_flag        /* i  : flag for constraining pitch contribution */
);

void updt_tar_fx(
    const Word16 *x,    /* i  : old target (for pitch search)     */
    Word16 *x2,   /* o  : new target (for codebook search)  */
    const Word16 *y,    /* i  : filtered adaptive codebook vector */
    const Word16 gain,  /* i  : adaptive codebook gain            */
    const Word16 L      /* i  : subframe size                     */
);
void updt_tar_HR_fx(
    const Word16 *x,    /* i  : old target (for pitch search)     */
    Word16 *x2,   /* o  : new target (for codebook search)  */
    const Word16 *y,    /* i  : filtered adaptive codebook vector */
    const Word16 gain,  /* i  : adaptive codebook gain            */
    const Word16 Qx,    /* i  : Scaling factor to adapt output to input */
    const Word16 L      /* i  : subframe size                     */
);

Word16 lp_filt_exc_enc_fx(
    const Word16 codec_mode,                 /* i  : MODE1 or MODE2                                  Q0 */
    const Word32 core_brate,                 /* i  : core bitrate                                    Q0 */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode                  Q0 */
    const Word16 coder_type,                 /* i  : coding type                                     Q0 */
    const Word16 i_subfr,                    /* i  : subframe index                                  Q0 */
    Word16 *exc,                       /* i/o: pointer to excitation signal frame              Q_new */
    const Word16 *h1,                        /* i  : weighted filter input response                  Q(14+shift) */
    const Word16 *xn,                        /* i  : target vector                                   Q_new-1+shift */
    Word16 *y1,                        /* o  : zero-memory filtered adaptive excitation        Q_new-1+shift */
    Word16 *xn2,                       /* o  : target vector for innovation search             Q_new-1+shift */
    const Word16 L_subfr,                    /* i  : length of vectors for gain quantization         Q0 */
    const Word16 L_frame,                    /* i  : frame size                                      Q0 */
    Word16 *g_corr,                    /* o  : ACELP correlation values                        mant/exp */
    const Word16 clip_gain,                  /* i  : adaptive gain clipping flag                     Q0 */
    Word16 *gain_pit,                  /* o  : adaptive excitation gain                        Q14 */
    Word16 *lp_flag                    /* i/o: mode selection                                  Q0 */
);

Word16 Interpol_4(         /* o  : interpolated value  */
    Word16 * x,      /* i  : input vector        */
    Word16 frac      /* i  : fraction (-4..+3)   */
);

void r_fft_fx_lc(
    const Word16 *phs_tbl,    /* i  : Table of phase            */
    const Word16 SIZE,        /* i  : Size of the FFT           */
    const Word16 SIZE2,       /* i  : Size / 2                  */
    const Word16 NUM_STAGE,   /* i  : Number of stage           */
    const Word16 *in_ptr,     /* i  : coeffients in the order re[0], re[1], ... re[n/2], im[1], im[2], ..., im[n/2-1] */
    Word16 *out_ptr,    /* o  : coeffients in the order re[0], re[1], ... re[n/2], im[1], im[2], ..., im[n/2-1] */
    const Word16 isign        /* i  : 1=fft, otherwize it's ifft                                                      */
);

void hf_synth_fx(
    const Word32  core_brate,       /* i  : core bitrate                   */
    const Word16 output_frame,      /* i  : output frame length            */
    const Word16 *Aq,               /* i  : quantized Az                   */
    const Word16 *exc,              /* i  : excitation at 12.8 kHz         */
    Word16 *synth,            /* i  : 12.8kHz synthesis signal       */
    Word16 *synth16k,         /* o  : 16kHz synthesis signal         */
    Word16 *seed2,            /* i/o: random seed for HF noise gen   */
    Word16 *mem_hp400,        /* i/o: memory of hp 400 Hz filter     */
    Word16 *mem_syn_hf,       /* i/o: HF synthesis memory            */
    Word16 *mem_hf,           /* i/o: HF band-pass filter memory     */
    const Word16 Q_exc,             /* i  : excitation scaling             */
    const Word16 Q_syn2,            /* i  : synthesis scaling              */
    Word16 *delay_syn_hf,     /*i/o: HF synthesis memory             */
    Word16 *memExp1,          /* o  : HF excitation exponent         */
    Word16 *mem_hp_interp,    /* i/o: interpol. memory               */
    const Word16 extl,            /* i  : flag indicating BWE            */
    const Word16 CNG_mode         /* i  : CNG_mode                       */
);
void acelp_core_dec_fx(
    Decoder_State_fx *st_fx,             /* i/o: decoder state structure             */
    Word16 synth_out[],                  /* o  : synthesis                           */
    Word32 bwe_exc_extended[],           /* i/o: bandwidth extended excitation       */
    Word16 *voice_factors,               /* o  : voicing factors                     */
    Word16 old_syn_12k8_16k[],           /* o  : intermediate ACELP synthesis at 12.8kHz or 16kHz to be used by SWB BWE */
    Word16 coder_type_fx,                /* i  : coder type                          */
    Word16 sharpFlag,
    Word16 pitch_buf_fx[NB_SUBFR16k],    /* o  : floating pitch for each subframe    */
    Word16 *unbits                       /* o  : number of unused bits               */
    ,Word16 *sid_bw                       /* o  : 0-NB/WB, 1-SWB SID                  */
);
void Inac_swtch_ematch_fx(
    Word16 exc2[],               /* i/o: CELP/GSC excitation buffer       Q_exc*/
    Word16 dct_exc_tmp[],        /* i  : GSC excitation in DCT domain          */
    Word16 lt_ener_per_band[],   /* i/o: Long term energy per band        Q12  */
    const Word16 coder_type,           /* i  : Coding mode                           */
    const Word16 L_frame,              /* i  : Frame lenght                          */
    const Word32 core_brate,           /* i  : Core bit rate                         */
    const Word16 Q_exc                 /* i  : input and output format of exc2       */
);
void stat_noise_uv_enc_fx(
    Encoder_State_fx *st_fx,          /* i/o: state structure                      */
    const Word16 coder_type,        /* i  : coding type                          */
    const Word32 *LepsP,            /* i  : LP prediction errors                 */
    Word16 *isp_new,          /* i  : immittance spectral pairs at 4th sfr */
    Word16 *isp_mid,          /* i  : immittance spectral pairs at 2nd sfr */
    Word16 *Aq,               /* i  : A(z) quantized for the 4 subframes   */
    Word16 *exc2,             /* i/o: excitation buffer                    */
    Word16 Q_new
);

void updt_enc_fx(
    Encoder_State_fx *st,           /* i/o: state structure                          */
    const Word16 L_frame,           /* i  : length of the frame                      */
    const Word16 coder_type,        /* i  : speech coder type                        */
    const Word16 *old_exc,          /* i  : buffer of excitation                     */
    const Word16 *pitch_buf,        /* i  : floating pitch for each subframe         */
    const Word16 Es_pred,           /* i  : predicited scaled innovation energy      */
    const Word16 *Aq,               /* i  : A(z) quantized for all subframes         */
    const Word16 *lsf_new,          /* i  : current frame LSF vector                 */
    const Word16 *lsp_new,          /* i  : current frame LSP vector                 */
    const Word16 *old_bwe_exc       /* i  : buffer of excitation                     */
);
void FEC_encode_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word16 *synth,            /* i  : pointer to synthesized speech for E computation */
    const Word16 coder_type,        /* i  : type of coder                                   */
    Word16 clas,              /* i  : signal clas for current frame                   */
    const Word16 *fpit,             /* i  : close loop fractional pitch buffer              */
    const Word16 *res,              /* i  : LP residual signal frame                        */
    Word16 *last_pulse_pos,   /* i/o: Position of the last pulse                      */
    const Word16 L_frame,           /* i  : Frame length                                    */
    const Word32 total_brate,       /* i  : total codec bitrate                             */
    const Word32  core_brate,        /* i  : total codec bitrate                             */
    const Word16 Q_new,             /* i  : input scaling                                   */
    const Word16 shift              /* i  : scaling to get 12bits                           */
);
void Ener_per_band_comp_fx(
    const Word16 exc_diff[],    /* i  : target signal                     Q_exc_diff     */
    Word16 y_gain4[],     /* o  : Energy per band to quantize     Q12            */
    const Word16 Q_exc_diff,
    const Word16 Mband,         /* i  : Max band                          */
    const Word16 Eflag
);
void acelp_core_enc_fx(
    Encoder_State_fx *st_fx,                     /* i/o: encoder state structure             */
    LPD_state   *mem,                    /* i/o: acelp memories             */
    const Word16 inp_fx[],                    /* i  : input signal of the current frame   */
    const Word16 vad_flag_fx,
    const Word32 ener_fx,                     /* i  : residual energy from Levinson-Durbin*/
    const Word16 pitch[3],                     /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing_fx[3],               /* i  : Open-loop pitch gains               */
    Word16 A_fx[NB_SUBFR16k*(M+1)],     /* i  : A(z) unquantized for the 4 subframes*/
    Word16 Aw_fx[NB_SUBFR16k*(M+1)],    /* i  : weighted A(z) unquant. for subframes*/
    const Word16 epsP_h_fx[M+1],                /* i  : LP prediction errors                */
    const Word16 epsP_l_fx[M+1],                /* i  : LP prediction errors                */
    Word16 lsp_new_fx[M],               /* i  : LSPs at the end of the frame        */
    Word16 lsp_mid_fx[M],               /* i  : LSPs in the middle of the frame     */
    Word16 coder_type_fx,               /* i  : coding type                         */
    const Word16 sharpFlag_fx,                /* i  : formant sharpening flag             */
    Word16 vad_hover_flag_fx,
    const Word16 gsc_attack_flag_fx,          /* i  : flag signalling attack encoded by AC mode (GSC) */
    Word32 bwe_exc_extended_fx[],       /* i/o: bandwidth extended excitation       */
    Word16 *voice_factors_fx,           /* o  : voicing factors                     */
    Word16 old_syn_12k8_16k_fx[],       /* o  : intermediate ACELP synthesis at 12.8kHz or 16kHz to be used by SWB BWE */
    Word16 pitch_buf_fx[NB_SUBFR16k],   /* o  : floating pitch for each subframe    */
    Word16 *unbits_fx,                   /* o  : number of unused bits               */
    const Word16 Q_new,
    const Word16 shift
);

#define EDCT_FACTOR_SCALE 2
void edct_fx(
    const Word32 *x,                         /* i  : input signal         Qq                  */
    Word32 *y,                         /* o  : output transform     Qq                  */
    Word16 length,                     /* i  : length                                   */
    Word16 *q                          /* i  : Q value of input signal                  */
);

void edst_fx(
    const Word32 *x,                         /* i  : input signal         Qq                  */
    Word32 *y,                         /* o  : output transform     Qq                  */
    Word16 length,                     /* i  : length                                   */
    Word16 *q                          /* i  : Q value of input signal                  */
);

void DoRTFTn_fx(
    Word32 *x,    /* i/o : real part of input and output data       */
    Word32 *y,     /* i/o : imaginary part of input and output data  */
    const Word16 n /* i : size of the FFT up to 1024 */
);

void edct2_fx(
    Word16 n,
    Word16 isgn,
    Word16 *in,
    Word32 *a,
    Word16 *q,
    const Word16 *ip,
    const Word16 *w
);
void DoRTFT160_16fx(
    Word16 x[],     /* i/o : real part of input and output data       */
    Word16 y[]      /* i/o : imaginary part of input and output data  */
);
void DoRTFT320_16fx(
    Word16 *x,   /* i/o : real part of input and output data       */
    Word16 *y    /* i/o : imaginary part of input and output data  */
);
void DoRTFT128_16fx(
    Word16 *x,    /* i/o : real part of input and output data       Q(Qx+Q_edct)*/
    Word16 *y     /* i/o : imaginary part of input and output data  Q(Qx+Q_edct)*/
);
void edct_16fx(
    const Word16 *x,         /* i  : input signal        Qx      */
    Word16 *y,         /* o  : output transform    Qx    */
    Word16 length,      /* i  : length            */
    Word16 bh      /* bit-headroom */
);
void highband_exc_dct_in_fx(
    const Word32 core_brate,                /* i  : core bitrate                            */
    const Word16 *mfreq_bindiv_loc,         /* i  : bin per bands tables                    */
    Word16 last_bin,                  /* i  : last bin of bit allocation              */
    Word16 Diff_len,                  /* i  : number of bin before cut-off frequency  */
    Word16 noise_lev,                 /* i  : pulses dynamic                          */
    Word16 pit_band_idx,              /* i  : bin position of the cut-off frequency   */
    Word16 *exc_diffQ,                /* i  : frequency coefficients of per band      */
    Word16 *seed_tcx,                 /* i  : Seed for noise                          */
    Word16 *Ener_per_bd_iQ,           /* i  : Quantized energy of targeted vector     */
    Word16 nb_subfr,                  /* i  : Number of subframe considered           */
    Word16 *exc_dct_in,               /* o  : dct of residual signal                  */
    Word16 last_coder_type,           /* i  : coding type of last frame               */
    Word16 *bitallocation_band,       /* i  : bit allocation flag of each band        */
    Word16 *lsf_new,                  /* i  : LSFs at the end of the frame            */
    Word16 *last_exc_dct_in,          /* i  : dct of residual signal of last frame    */
    Word16 *last_ener,                /* i  : frequency energy  of last frame         */
    Word16 *last_bitallocation_band,  /* i  : bit allocation flag of each band  of last frame   */
    Word16 *bitallocation_exc,        /* i  : flag of decoded coefficients            */
    Word16 bfi,                       /* i  : bad frame indicator                     */
    const Word16 coder_type,                /* i  : coder type                              */
    Word16 bwidth,
    Word16 *exc_wo_nf ,                /* o  : temporal excitation (in f domain) without noisefill   */
    Word16 Qexc_diffQ,
    Word16 Q_exc,
    const Word16 GSC_noisy_speech
);


void long_enr_fx(
    Encoder_State_fx *st_fx,      /* i/o: state structure                         */
    const Word16 Etot,        /* i  : total channel E (see lib_enc\analy_sp.c) */
    const Word16 localVAD_HE_SAD,    /* i  : HE-SAD flag without hangover             */
    Word16 high_lpn_flag
);

Word16 correlation_shift_fx(  /* o  : noise dependent voicing correction     Q15 */
    Word16 totalNoise_fx      /* i/o: noise estimate over all critical bands  Q8 */
);

void analy_lp_fx(
    const Word16 speech[],    /* i  : pointer to the speech frame                      */
    const Word16 L_frame,     /* i  : length of the frame                              */
    const Word16 L_look,      /* i  : look-ahead                                       */
    Word32 *ener,       /* o  : residual energy from Levinson-Durbin             */
    Word16 A[],         /* o  : A(z) filter coefficients                         */
    Word16 epsP_h[],    /* o  : LP analysis residual energies for each iteration */
    Word16 epsP_l[],    /* o  : LP analysis residual energies for each iteration */
    Word16 lsp_new[],   /* o  : current frame LSPs                               */
    Word16 lsp_mid[],   /* o  : current mid-frame LSPs                           */
    Word16 lsp_old[],   /* i/o: previous frame unquantized LSPs                  */
    const Word16 Top[2],      /* i  : open loop pitch lag                              */
    const Word16 Tnc[2],      /* i  : open loop pitch gain                             */
    const Word32 Core_sr,     /* i  : Internal core sampling rate                      */
    Word16 Q_new,
    Word16 *Q_r
);

void analy_lp_AMR_WB_fx(
    const Word16 speech[],    /* i  : pointer to the speech frame                      */
    Word32 *ener,       /* o  : residual energy from Levinson-Durbin             */
    Word16 A[],         /* o  : A(z) filter coefficients                         */
    Word16 epsP_h[],    /* o  : LP analysis residual energies for each iteration */
    Word16 epsP_l[],    /* o  : LP analysis residual energies for each iteration */
    Word16 isp_new[],   /* o  : current frame ISPs                               */
    Word16 isp_old[],   /* i/o: previous frame unquantized ISPs                  */
    Word16 isf_new[],   /* o  : current frame ISPs                               */
    Word16 Top,         /* i  : open loop pitch lag                              */
    Word16 Tnc,         /* i  : open loop pitch gain                             */
    Word16 Q_new,
    Word16 *Q_r
);

void a2rc_fx(const Word16 *a, Word16 *refl,Word16 lpcorder);

Word32  invert_dp(Word40 Linput, Word16 n, Word16 *Qout, Word16 wb_mode_bit);

void vad_param_updt_fx(
    Encoder_State_fx *st_fx,      /* i/o: state structure                         */
    Word16 pitch[3],        /* i  : open loop pitch lag for each half-frame                    Q0*/
    Word16 voicing[3],      /* i  : maximum normalized correlation for each half-frame         Q15*/
    Word16 corr_shift,      /* i  : correlation shift                                          Q15*/
    Word16 vad_flag,        /* i  : vad flag                                                   Q0*/
    const Word16 Az[]       /* i:  a coeffs */
);

void pitch_ol2_fx(
    const Word16 pit_min,        /* i  : minimum pitch value (20 or 29)                   */
    const Word16 pitch_ol,     /* i  : pitch to be improved                             */
    Word16 *pitch_fr_fx,   /* o  : adjusted 1/4 fractional pitch                    */ /*Q7 */
    Word16 *voicing_fr_fx, /* o  : adjusted 1/4 fractional voicing                  */ /*Q15 */
    const Word16 pos,       /* i  : position in frame where to calculate the improv. */
    const Word16 *wsp_fx,        /* i  : weighted speech for current frame and look-ahead */ /*Q_new-1+shift*/
    const Word16 delta           /* i  : delta for pitch search (2 or 7)                  */
);

void pitch_ol_fx(
    Word16 pitch[3],        /* o  : open loop pitch lag for each half-frame in range [29,231]      Q0  */
    Word16 voicing[3],      /* o  : maximum normalized correlation for each half-frame in [0,1.0[  Q15 */
    Word16 *old_pitch,      /* i/o: pitch of the 2nd half-frame of previous frame (i.e. pitch[1])  Q0  */
    Word16 *old_corr,       /* i/o: correlation of old_pitch (i.e. voicing[1] or corr_mean)        Q15 */
    Word16 corr_shift,      /* i  : normalized correlation correction                              Q15 */
    Word16 *old_thres,      /* i/o: maximum correlation weighting with respect to past frame pitch Q15 */
    Word16 *delta_pit,      /* i/o: old pitch extrapolation correction in range [-14,+14]          Q0  */
    Word16 *st_old_wsp2,    /* i/o: weighted speech memory                                         qwsp */
    const Word16 *wsp,            /* i  : weighted speech for current frame and look-ahead               qwsp */
    Word16 mem_decim2[3],   /* i/o: wsp decimation filter memory                                   qwsp */
    const Word16 relE,            /* i  : relative frame energy                                          Q8  */
    const Word16 last_class,      /* i  : frame classification of last frame                                 */
    const Word16 bwidth,          /* i  : bandwidth                                                          */
    const Word16 Opt_SC_VBR       /* i  : SC-VBR flag                                                        */
);

void Scale_wsp(
    Word16 *wsp,                /* i  : Weigthed speech                      */
    Word16 *old_wsp_max,        /* i  : Last weigthed speech maximal valu    */
    Word16 *shift,              /* i/o: Scaling of current frame             */
    Word16 *Q_exp,              /* i/o: Differential scaling factor          */
    Word16 *old_wsp_shift,      /* i/o: Last wsp scaling                     */
    Word16 *old_wsp,            /* i/o: Old weighted speech buffer           */
    Word16 *mem_decim2,         /* i/o: Decimation buffer                    */
    Word16 *old_wsp12k8,        /* i/o: wsp memory @ 12.8 kHz used in pitol2 */
    const  Word16 Len_p_look           /* i  : L_frame + look ahead                 */
);

Word16 multi_harm_fx(               /* o  : frame multi-harmonicity (1-harmonic, 0-not)         */
    const Word16 Bin_E[],           /* i  : log-energy spectrum of the current frame      Q7    */
    Word16 old_S[],           /* i/o: prev. log-energy spectrum w. subtracted floor Q7    */
    Word16 cor_map_LT[],      /* i/o: LT correlation map                            Q15   */
    Word16 *multi_harm_limit, /* i/o: multi harminic threshold                      Q9    */
    const Word32  total_brate,      /* i  : total bitrate                                 Q0    */
    const Word16 bwidth,            /* i  : input signal bandwidth                        Q0    */
    Word16 *cor_strong_limit, /* i/o: HF correlation indicator                      Q0    */
    Word16 *st_mean_avr_dyn,  /* i/o: long term average dynamic                     Q7    */
    Word16 *st_last_sw_dyn,   /* i/o: last dynamic                                  Q7    */
    Word16 *cor_map_sum       /*                                                    Q8    */
    , Word16 *sp_floor          /* o  : noise floor estimate                          Q7    */
);

Word16 find_uv_fx(                  /* o  : coding type                                             */
    Encoder_State_fx *st_fx,    /* i/o: encoder state structure                                 */
    const Word16 *T_op_fr,      /* i  : pointer to adjusted fractional pitch (4 val.)       Q6  */
    const Word16 *voicing_fr,   /* i  : refined correlation for each subframes              Q15 */
    const Word16 *voicing,      /* i  : correlation for 3 half-frames                       Q15 */
    const Word16 *speech,       /* i  : pointer to speech signal for E computation         Q_new*/
    const Word16 localVAD,      /* i  : vad without hangover                                    */
    const Word32 *ee,           /* i  : lf/hf Energy ratio for present frame                Q6  */
    const Word16 corr_shift,    /* i  : normalized correlation correction in noise          Q15 */
    const Word16 relE,          /* i  : relative frame energy                               Q8  */
    const Word16 Etot,          /* i  : total energy                                        Q8  */
    const Word32 hp_E[],        /* i  : energy in HF                            Q_new + Q_SCALE */
    const Word16 Q_new,
    Word16 *flag_spitch,  /* i/o: flag to indicate very short stable pitch and high correlation */
    const Word16 voicing_sm     /* i  : smoothed open-loop pitch gains                          */
);

Word16 signal_clas_fx(        /* o  : classification for current frames                 */
    Encoder_State_fx *st,         /* i/o: encoder state structure                           */
    Word16 *coder_type, /* i/o: coder type                                        */
    const Word16 voicing[3],  /* i  : normalized correlation for 3 half-frames          */
    const Word16 *speech,     /* i  : pointer to speech signal for E computation        */
    const Word16 localVAD,    /* i  : vad without hangover                              */
    const Word16 pit[3],      /* i  : open loop pitch values for 3 half-frames          */
    const Word32 *ee,         /* i  : lf/hf E ration for 2 half-frames                  */
    const Word16 relE,        /* i  : frame relative E to the long term average         */
    const Word16 L_look ,     /* i  : look-ahead                                        */
    Word16 *uc_clas     /* o  : temporary classification used in music/speech class*/
);

void find_tilt_fx(
    const Word32 fr_bands[],  /* i  : energy in frequency bands            Q_new + Q_SCALE*/
    const Word32 bckr[],      /* i  : per band background noise energy estimate    Q_new + Q_SCALE*/
    Word32 ee[2],       /* o  : lf/hf E ration for present frame               Q6*/
    const Word16 pitch[3],    /* i  : open loop pitch values for 3 half-frames           Q0*/
    const Word16 voicing[3],  /* i  : normalized correlation for 3 half-frames          Q15*/
    const Word32 *lf_E,       /* i  : per bin energy  for low frequencies         Q_new + Q_SCALE - 2*/
    const Word16 corr_shift,  /* i  : normalized correlation correction              Q15*/
    const Word16 bwidth,      /* i  : input signal bandwidth                                         */
    const Word16 max_band,    /* i  : maximum critical band                                          */
    Word32 hp_E[],      /* o  : energy in HF                                    Q_new + Q_SCALE*/
    const Word16 codec_mode,  /* i  : MODE1 or MODE2                                */
    const Word16 Q_new,       /* i  : scaling factor                           */
    Word32 *bckr_tilt_lt
    ,Word16 Opt_vbr_mode
);
void synth_mem_updt2(
    const Word16 L_frame,        /* i  : frame length                            */
    const Word16 last_L_frame,   /* i  : frame length                            */
    Word16 old_exc[],      /* i/o: excitation buffer                       */
    Word16 mem_syn_r[],    /* i/o: synthesis filter memory                 */
    Word16 mem_syn2[],     /* o  : synthesis filter memory for find_target */
    Word16 mem_syn[],      /* o  : synthesis filter memory for find_target */
    const Word16 dec             /* i  : flag for decoder indication             */
);
void swb_hr_noise_fill_fx(
    const Word16 is_transient,         /* i  : transient flag          */
    const Word16 spect_start,          /* i  : spectrum start point    */
    const Word16 spect_end,            /* i  : spectrum end point      */
    const Word16 tilt_wb,              /* i  : tilt of wideband signal */
    const Word16 pitch,                /* i  : pitch value             */
    const Word16 nq[],                 /* i  : AVQ nq index            */
    Word16 Nsv,                  /* i  : number of subband       */
    Word16 *bwe_highrate_seed,   /* i/o: seed of random noise    */
    Word16 *t_audio,             /* i/o: mdct spectrum           */
    Word16 Q_audio
);
Word16 swb_bwe_dec_hr_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure     */
    const Word16 *syn_12k8_16k_fx, /* i  : ACELP core synthesis @16kHz */
    const Word16 old_syn_exp,      /* i  : Exponent of core synthesis  */
    Word16 *hb_synth_fx,     /* o  : SHB synthesis               */
    const Word16 output_frame,     /* i  : frame length                */
    const Word16 unbits,           /* i  : number of core unused bits  */
    const Word16 pitch_buf[]       /* i  : pitch buffer                */
);
Word32 calc_tilt_bwe_fx(            /* o  : Tilt in Q24       */
    const Word16 *sp0,        /* i  : input signal      */
    const Word16 exp,         /* i  : Exp of inp signal */
    const Word16 N            /* i  : signal length     */
);
void Inverse_Transform(
    const Word32 *in_mdct,          /* i  : input MDCT vector               */
    Word16 *Q,              /* i/o: Q value of input                */
    Word32 *out,            /* o  : output vector                   */
    const Word16 is_transient,      /* i  : transient flag                  */
    const Word16 L,                 /* i  : output frame length             */
    const Word16 L_inner            /* i  : length of the transform         */
);
void iedct_short_fx(
    const Word32 *in,               /* i  : input vector        */
    Word16 *Q,                /* i/o: Q value of input    */
    Word32 *out,              /* o  : output vector       */
    const Word16 segment_length     /* i  : length              */
);

void swb_bwe_enc_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    Word16 *old_input_12k8_fx,    /* i  : input signal @12.8kHz for SWB BWE       */
    Word16 *old_input_16k_fx,     /* i  : input signal @16kHz for SWB BWE         */
    const Word16 *old_syn_12k8_16k_fx,  /* i  : ACELP core synthesis at 12.8kHz or 16kHz */
    const Word16 *new_swb_speech_fx,    /* i  : original input signal at 32kHz           */
    Word16 *shb_speech_fx,        /* i  : SHB target signal (6-14kHz) at 16kHz     */
    const Word16 coder_type,            /* i  : coding type                              */
    Word16 Q_shb_speech,
    Word16 Q_slb_speech
);
void wb_bwe_enc_fx(
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure                  */
    const Word16 *new_wb_speech_fx,  /* i  : original input signal at 16kHz           */
    Word16 coder_type          /* i  : coding type                              */
);

Word16 WB_BWE_encoding_fx(      /* o  : classification of wb signal            */
    const Word16 coder_type,         /* i  : coder type                             */
    const Word16 *yos_fx,            /* i  : MDCT coefficients of weighted original */
    Word16 *WB_fenv_fx,        /* i/o: energy of WB envelope                  */
    Encoder_State_fx *st_fx,             /* i/o: Encoder structure                      */
    Word16 Q_synth,
    Word16 Q_synth_lf
);


Word16 wb_bwe_dec_fx(
    Word16 *synth_fx,            /* i/o: ACELP core synthesis/final synthesis    */
    Word16 *hb_synth_fx,         /* o  : SHB synthesis/final synthesis           */
    const Word16 output_frame,         /* i  : frame length                            */
    Word16 coder_type,           /* i  : coding type                             */
    Word16 *voice_factors_fx,    /* i  : voicing factors                         */
    const Word16 pitch_buf_fx[],       /* i  : pitch buffer                            */
    Decoder_State_fx *st_fx                /* i/o: decoder state structure                 */
    ,Word16 * Qpost
);

Word16 WB_BWE_gain_deq_fx(
    Decoder_State_fx *st_fx ,    /* i/o: decoder state structure                 */
    Word16 *WB_fenv
);
Word16 swb_bwe_gain_deq_fx(     /* o  : BWE class                           */
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure                 */
    const Word16 core,                /* i  : core                                */
    Word16 *SWB_tenv,           /* o  : Q0, time-domain BWE envelope        */
    Word16 *SWB_fenv,           /* o  : Q1, frequency-domain BWE envelope   */
    const Word16 hr_flag,             /* i  : high rate flag                      */
    const Word16 hqswb_clas           /* i  : HQ BWE class                        */
);

Word16 swb_bwe_dec_fx(
    Decoder_State_fx *st_fx,                 /* i/o: decoder state structure                 */
    Word16 *synth_fx,              /* i  : ACELP core synthesis/final synthesis    */
    Word16 *hb_synth,              /* o  : SHB synthesis/final synthesis           */
    const Word16 output_frame            /* i  : frame length                            */
    ,Word16 * Qpost
);

Word16 WB_BWE_gain_pred_fx(
    Word16 *WB_fenv,                 /* o  : WB frequency envelopes               */
    const Word16 *core_dec_freq,           /* i  : Frequency domain core decoded signal */
    const Word16 coder_type,               /* i  : coding type                          */
    Word16 prev_coder_type,          /* i  : coding type of last frame            */
    Word16 prev_WB_fenv,             /* i  : envelope for last frame              */
    Word16 *voice_factors,           /* i  : voicing factors        Q15           */
    const Word16 pitch_buf[],              /* i  : pitch buffer           Q6            */
    Word32 last_core_brate,          /* i  : previous frame core bitrate          */
    Word16 last_wb_bwe_ener ,        /* i  : previous frame wb bwe signal energy */
    Word16 Q_syn
);
void calc_normal_length_fx(
    const Word16 core,             /* i  : core                   */
    const Word16 *sp,              /* i  : input signal           */
    const Word16 mode,             /* i  : input mode             */
    const Word16 extl,             /* i  : extension layer        */
    Word16 *L_swb_norm,      /* o  : normalize length       */
    Word16 *prev_L_swb_norm, /*i/o : last normalize length  */
    Word16 Q_syn
);

void calc_norm_envelop_fx(
    const Word16 SWB_signal[],    /* i  : SWB spectrum            Q_syn*/
    Word32 *envelope,       /* o  : normalized envelope     Q_syn*/
    const Word16 L_swb_norm,      /* i  : length of envelope     Q0    */
    const Word16 SWB_flength,     /* i  : Length of input/output       */
    const Word16 st_offset        /* i  : offset                       */
);

void WB_BWE_decoding_fx(
    const Word16 *core_dec_freq,    /* i  : Frequency domain core decoded signal */
    Word16 *WB_fenv,          /* i  : WB frequency envelopes               */
    Word32 *WB_signal32,      /* o  : WB signal in MDCT domain             */
    const Word16 WB_flength,        /* i  : Length of input/output               */
    const Word16 mode,              /* i  : classification for WB signal         */
    const Word16 last_extl,         /* i  : extl. layer for last frame           */
    Word32 *prev_Energy,      /* i/o: energy for last frame                */
    Word16 *prev_WB_fenv,     /* i/o: envelope for last frame              */
    Word16 *prev_L_wb_norm,   /* i/o: length for last frame wb norm        */
    const Word16 extl,              /* i  : extension layer                      */
    const Word16 coder_type,        /* i  : coding type                          */
    const Word32 total_brate,       /* i  : core layer bitrate                   */
    Word16 *Seed,             /* i/o: random generator seed                */
    Word16 *prev_flag,        /* i/o: attenu flag of last frame            */
    Word16 prev_coder_type ,  /* i  : coding type of last frame            */
    Word16 Q_syn,
    Word16 *Q_syn_hb          /*o   : Q value of WB_signal_32              */
);

void SWB_BWE_decoding_fx(
    const Word16 *core_dec_freq,    /* i  : Frequency domain core decoded signal  */
    Word16 *SWB_fenv,         /* i/o: SWB frequency envelopes               */
    Word32 *SWB_signal,       /* o  : SWB signal in MDCT domain             */
    const Word16 SWB_flength,       /* i  : Length of input/output                */
    const Word16 mode,              /* i  : classification for SWB signal         */
    Word16 *frica_flag,       /* o  : fricative signal flag                 */
    Word16 *prev_Energy,      /* i/o: energy for last frame                 */
    Word16 *prev_SWB_fenv,    /* i/o: envelope for last frame               */
    Word16 *prev_L_swb_norm,  /* i/o: length for last frame wb norm         */
    const Word16 tilt_nb,           /* i  : tilt of synthesis wb signal           */
    Word16 *Seed,             /* i/o: random generator seed                 */
    const Word16 st_offset,         /* i  : offset value due to different core    */
    Word16 *prev_weight,      /* i/o: excitation weight value of last frame */
    const Word16 extl ,             /* i  : extension layer                       */
    Word16 Q_syn
);
void time_envelop_shaping_fx(
    Word16 werr[],             /* i/o: SHB synthesis           Q_synth*/
    Word32 SWB_tenv[],         /* i/o: frequency envelope          Q15*/
    const Word16 L ,                 /* i  : frame length                   */
    Word16 *Q_synth
);
void time_reduce_pre_echo_fx(
    const Word16 *synth,             /* i  : ACELP core synthesis    Q_syn*/
    Word16 *error,             /* i/o: SHB BWE synthesis          Q0*/
    Word16 prev_td_energy,     /* o  : last td energy             Q0*/
    const Word16 L,                  /* i  : subframe length              */
    Word16 Q_syn,
    Word16 Q_synth
);
/*Word16 detect_transient_fx(  */
/*    const Word16 *in_fx, *//*Q_new */
/*   Encoder_State *st, */
/*    const Word16 L, */
/*    const Word16 coder_type ,           */ /* i  : coder type               */
/*          Word16 Q_new, */
/*Encoder_State_fx *st_fx */
/*); */


void preset_hq2_swb_fx
(
    const Word16 hqswb_clas_fx,     /* i   : HQ2 class information               */
    const Word16 band_end_fx[],     /* i   : band end of each SB                 */
    Word16 *har_bands_fx,     /* i/o : Number of LF harmonic bands         */
    Word16 p2a_bands_fx,      /* i   : flag for peakness                   */
    const Word16 length_fx,         /* i   : processed band length               */
    const Word16 bands_fx,          /* i   : Total number of Subbands in a frame */
    Word16 *lowlength_fx,     /* o   : lowband length                      */
    Word16 *highlength_fx,    /* o   : highband length                     */
    Word32 L_m[]              /* o   : MDCT                                */
);
void post_hq2_swb_fx
(
    const Word32 L_m[],             /* i   : input_signal                        */
    const Word16 lowlength_fx,      /* i   : lowband length                      */
    const Word16 highlength_fx,     /* i   : highband length                     */
    const Word16 hqswb_clas_fx,     /* i   : HQ2 class information               */
    const Word16 har_bands_fx,      /* i   : Number of LF harmonic bands         */
    const Word16 bands_fx,          /* i   : Total number of Subbands in a frame */
    const Word16 p2a_flags_fx[],    /* i   : HF tonal indicator                  */
    const Word16 band_start_fx[],   /* i   : band start of each SB               */
    const Word16 band_end_fx[],     /* i   : band end of each SB                 */
    Word32 L_y2[],            /* o   : output signal                       */
    Word16 npulses_fx[]       /* i/o : Number of coded spectrum            */
);

void swb_bwe_enc_lr_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure                      */
    const Word32 L_m_core[],           /* i  : lowband synthesis                            */
    Word16 QsL,
    const Word32 L_m_orig[],           /* i/o: scaled orig signal (MDCT)                    */
    Word32 L_m[],                /* o  : highband synthesis with lowband zeroed       */
    const Word32 L_total_brate,        /* i  : total bitrate for selecting subband pattern  */
    Word16 BANDS_fx,             /* i  : Total number of Subbands in a frame          */
    Word16 *band_start_fx,       /* i  : band start of each SB                        */
    Word16 *band_end_fx,         /* i  : band end of each SB                          */
    Word32 *L_band_energy,       /* i  : band_energy of each SB                       */
    Word16 Qbe,                  /* i  : Q value of band energy                       */
    Word16 *p2a_flags_fx,        /* i  : HF tonal indicator                           */
    const Word16 hqswb_clas_fx,        /* i  : HQ_NORMAL2 or HQ_HARMONIC mode               */
    Word16 lowlength_fx,         /* i  : lowband length                               */
    Word16 highlength_fx,        /* i  : highband length                              */
    Word16 *prev_frm_index_fx,   /* i/o: previous frame lag index for harmonic mode   */
    const Word16 har_bands_fx,         /* i  : Number of LF harmonic bands                  */
    Word16 *prev_frm_hfe2,       /* i/o: */
    Word16 *prev_stab_hfe2,      /* i/o: */
    const Word16 band_width_fx[],      /* i  : band_width information                       */
    const Word32 L_y2_ni[],            /* i  : band_width information                       */
    Word16 *ni_seed_fx           /* i/o: random seed for search buffer NI             */
);

void swb_bwe_dec_lr_fx(
    Decoder_State_fx *st_fx,                 /* i/o: decoder state structure                     */
    const Word32 L_m_core[],             /* i  : lowband synthesis                           */
    const Word16 QsL,                    /* i  : Q value of m_core                           */
    Word32 L_m[],                  /* o  : highband synthesis with lowband zeroed      */
    const Word32 L_total_brate,          /* i  : total bitrate for selecting subband pattern */
    Word16 BANDS_fx,               /* i  : Number subbands/Frame                       */
    Word16 *band_start_fx,         /* i  : Band Start of each SB                       */
    Word16 *band_end_fx,           /* i  : Band end of each SB                         */
    Word32 *L_band_energy,         /* i  : Band energy of each SB : Qbe                */
    Word16 Qbe,                    /* i  : Q value of band energy                      */
    Word16 *p2a_flags_fx,          /* i  : HF tonal Indicator                          */
    const Word16 hqswb_clas_fx,          /* i  : class information                           */
    Word16 lowlength_fx,           /* i  : Lowband Length                              */
    Word16 highlength_fx,          /* i  : Highband Length                             */
    const Word16 har_bands_fx,           /* i  : Number of LF harmonic bands                 */
    Word16 *prev_frm_hfe2_fx,      /* i/o: */
    Word16 *prev_stab_hfe2_fx,     /* i/o: */
    Word16 band_width_fx[],        /* i  : subband bandwidth */
    const  Word32 L_y2_ni[],              /* i/o: Sparse filled corecoder */
    Word16 *ni_seed_fx             /* i/o: random seed */
);

void GetPredictedSignal_fx(
    const Word16 *predBuf_fx,          /* i:  Q8     */
    Word32 *L_outBuf,            /* o:  Q9     */
    const Word16 lag_fx,               /* i:  Q0     */
    const Word16 fLen_fx,              /* i:  Q0     */
    const Word16 lagGains_fx,          /* i:  Qgain  */
    const Word16 Qgain                 /* i:  Q0     */
);

void Get20Log10Spec_fx(
    const Word32 *L_inBuf,     /* i  : input         Q_inBuf   */
    Word16 *outBuf_fx,   /* o  : output        Q7        */
    const Word16 fLen,         /* i  : loop length             */
    const Word16 Q_inBuf       /* i  : Qvalue of L_inBuf       */
);

void convert_lagIndices_pls2smp_fx(
    Word16 lagIndices_in_fx[],
    Word16 nBands_search_fx,
    Word16 lagIndices_out_fx[],
    const Word16 sspectra_fx[],
    const Word16 sbWidth_fx[],
    const Word16 fLenLow_fx
);

Word16 get_usebit_npswb_fx(
    Word16 hqswb_clas_fx
);

void SpectrumSmoothing_nss_fx(
    const Word32 *L_inBuf,                   /* i  : lowband MDCT              */
    Word16 *outBuf_fx,                 /* o  : output                    */
    Word16 *Qss,                       /* o  : Q value for output vector */
    const Word16 fLen                         /* i  : length                    */
);

void SpectrumSmoothing_fx(
    const Word32 *L_inBuf,                   /* i  : Qs  Low band MDCT             */
    Word16 *outBuf_fx,                 /* o  : Qss output                    */
    Word16 *Qss,                       /* o  : Q0  Q value of output vector  */
    const Word16 fLen,                       /* i  : Q0  length                    */
    const Word16 th_cut_fx                   /* i  : Qss threshold of cut          */
);

void GetSynthesizedSpecThinOut_fx(
    const Word16 *predBuf_fx,        /* i  : Qss: prediction buffer (i.e., lowband)  */
    const Word16 Qss,                /* i  :      Q value of input vector            */
    Word32 *L_outBuf,          /* o  : QsL: synthesized spectrum               */
    Word16 QsL,                /* o  :      Q value of synthesized spectrum    */
    const Word16 nBands_fx,          /* i  : Q0: number of subbands calculated       */
    const Word16 *sbWidth_fx,        /* i  : Q0: subband lengths                     */
    const Word16 *lagIndices_fx,     /* i  : Q0: lowband index for each subband      */
    const Word16 *lagGains_fx,       /* i  : Qgain: lagGain for each subband         */
    const Word16 *QlagGains_fx,      /* i  : Q0: Q value of lagGains_fx              */
    const Word16 predBufLen_fx       /* i  : Q0: lowband length                      */
);

Word32 hq2_bit_alloc_fx (
    const Word32 L_band_energy[],           /* i  : band energy of each subband                 */
    const Word16 bands,                     /* i  : total number of subbands in a frame         */
    Word32 L_Rk[],                    /* i/o: Bit allocation/Adjusted bit alloc.          */
    Word16 *bit_budget_fx,            /* i/o: bit bugdet                                  */
    Word16 *p2a_flags,                /* i  : HF tonal indicator                          */
    const Word16 weight_fx,                 /* i  : weight                                      */
    const Word16 band_width[],              /* i  : Sub band bandwidth                          */
    const Word16 num_bits,                  /* i  : available bits                              */
    const Word16 hqswb_clas,                /* i  : HQ2 class information                       */
    const Word16 bwidth,                    /* i  : input bandwidth                             */
    const Word16 is_transient               /* i  : indicator HQ_TRANSIENT or not               */
);

void hq2_noise_inject_fx(
    Word32 L_y2[],
    const Word16 band_start[],
    const Word16 band_end[],
    const Word16 band_width[],
    Word32 Ep_fx[],
    Word32 Rk_fx[],
    const Word16 npulses[],
    Word16 ni_seed,
    const Word16 bands,
    const Word16 ni_start_band,
    const Word16 bw_low,
    const Word16 bw_high,
    const Word32 enerL_fx,
    const Word32 enerH_fx,
    Word32 last_ni_gain_fx[],
    Word16 last_env_fx[],
    Word16 *last_max_pos_pulse,
    Word16 *p2a_flags,
    Word16 p2a_bands,
    const Word16 hqswb_clas,
    const Word16 bwidth,
    const Word32 bwe_br
);

void reordvct_fx(
    Word16 *y,         /* i/o: vector to rearrange    */
    const Word16 N,          /* i  : dimensions             */
    Word16 *idx        /* o  : reordered vector index */
);

void Bits2indvsb_fx (
    const Word32 *L_be,                 /* i  : Q14 Band Energy of sub-band                       */
    const Word16 start_band,            /* i  : Q0  start band indices                            */
    const Word16 end_band,              /* i  : Q0  end band indices                              */
    const Word16 Bits,                  /* i  : Q0  Total number of bits allocated to a group     */
    const Word32 L_Bits_needed,         /* i  : Q0  smallest bit number for allocation in group   */
    Word32 *L_Rsubband,           /* o  : Q18 bit allocation of sub-band                    */
    Word16 *p2aflags_fx           /* i/o: Q0  peaky/noise subband flag                      */
);

void hq2_bit_alloc_har_fx (
    const Word32 *L_y,                  /* i  : Q14 band energy of sub-vectors               */
    Word16 B_fx,                  /* i  : Q0  number of available bits                 */
    const Word16 N_fx,                  /* i  : Q0  number of sub-vectors                    */
    Word32 *L_Rsubband,           /* o  : Q18 sub-band bit-allocation vector           */
    Word16 p2a_bands_fx,          /* i  : Q0  highfreq bands                               */
    const Word32 L_core_brate,          /* i  : Q0  core bit rate                            */
    Word16 p2a_flags_fx[],        /* i/o: Q0  p2a_flags                                */
    const Word16 band_width_fx[]        /* i  : Q0  table of band_width                      */
);

void hq2_core_configure_fx (
    const Word16 frame_length,
    const Word16 num_bits,
    const Word16 is_transient,
    Word16 *bands,
    Word16 *length,
    Word16 band_width[],
    Word16 band_start[],
    Word16 band_end[],
    Word32 *L_qint,
    Word16 *eref,
    Word16 *bit_alloc_weight,
    Word16 *gqlevs,
    Word16 *Ngq,
    Word16 *p2a_bands,
    Word16 *p2a_th,
    Word16 *pd_thresh,
    Word16 *ld_slope,
    Word16 *ni_coef,
    Word32 L_bwe_br
);

void hq_lr_enc_fx(
    Encoder_State_fx *st_fx,           /* i/o:     : encoder state structure         */
    Word32 L_t_audio[],      /* i/o: Q12 : transform-domain coefs.         */
    const Word16 inner_frame_fx,   /* i  : Q0  : inner frame length              */
    Word16 *num_bits_fx,     /* i/o: Q0  : number of available bits        */
    const Word16 is_transient_fx   /* i  : Q0  : transient flag                  */
);

void hq_lr_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o:     : decoder state structure         */
    Word32 L_yout[],          /* o  : Q12 : transform-domain output coefs.  */
    const Word16 inner_frame,       /* i  : Q0  : inner frame length              */
    Word16 num_bits,          /* i  : Q0  : number of available bits        */
    Word16 *is_transient_fx   /* o  : Q0  : transient flag                  */
);

void reverse_transient_frame_energies_fx(
    Word32 L_band_energy[],       /* o  : Q14 : band energies       */
    const Word16 bands                  /* i  : Q0  : number of bands     */
);

Word16 har_est_fx(
    Word32 L_spectra[],                   /* i  : coded spectrum                 */
    Word16 N,                             /* i  : length of the desired spectrum */
    Word16 *har_freq_est1,                /* i/o: Estimation harmonics 1         */
    Word16 *har_freq_est2,                /* o  : Estimation harmonics 2         */
    Word16 *flag_dis,                     /* i/o: flag for BWE reconstruction    */
    Word16 *prev_frm_hfe2,                /* i/o: Estimated harmonic update      */
    const Word16 subband_search_offset[], /* i  : Subband Search range           */
    const Word16 sbWidth[],               /* i  : Subband Search range           */
    Word16 *prev_stab_hfe2                /* i/o: Estimated harmonic position    */
);

typedef struct GainItemStr_fx
{
    Word16 nmrValue_fx;
    Word16 gainIndex_fx;
} GainItem_fx;

void genhf_noise_fx(
    Word16 noise_flr_fx[],             /* i  : Qss smoothed non tonal                           */ /* sspectra_diff_fx:Qss */
    Word16 Qss,                        /* i  : Q0  Q value                                      */
    Word32 L_xSynth_har[],             /* o  : QsL hf non tonal components                      */ /* xSynth_har:QsL */
    Word16 QsL,                        /* i  : Q0  Q value                                      */
    Word16 *predBuf_fx,                /* i  : Qss smoothed tonal compone                       */ /* sspectra:Qss */
    Word16 bands,                      /* i  : Q0  total number of subbands in a frame          */
    Word16 harmonic_band,              /* i  : Q0  Number of LF harmonic frames                 */
    Word16 har_freq_est2,              /* i  : Q0  harmonic signal parameter                    */
    Word16 pos_max_hfe2,               /* i  : Q0  last pulse in core coder                     */
    Word16 *pul_res,                   /* o  : Q0  pulse resolution                             */
    GainItem_fx pk_sf_fx[],            /* o  :     representative region                        */
    const Word16 fLenLow,                    /* i  : Q0  low frequency length                         */
    const Word16 fLenHigh,                   /* i  : Q0  high frequency length                        */
    const Word16 sbWidth[],                  /* i  : Q0  bandwidth for high bands                     */
    const Word16 lagIndices[],               /* i  : Q0  correlation indices for most representative  */
    const Word16 subband_offsets[],          /* i  : Q0  band offsets for HF reconstruction           */
    const Word16 subband_search_offset[]     /* i  : Q0  most representative regions offsets in LF    */
);

void spt_shorten_domain_pre_fx(
    const Word16 band_start[],           /* i:   Starting position of sub band             */
    const Word16 band_end[],             /* i:   End position of sub band                  */
    const Word16 prev_SWB_peak_pos[],    /* i:   Spectral peak                             */
    const Word16 BANDS,                  /* i:   total number of bands                     */
    const Word32 L_bwe_br,               /* i:   bitrate information                       */
    Word16       new_band_start[],       /* o:   Starting position of new shorten sub band */
    Word16       new_band_end[],         /* o:   End position of new shorten sub band      */
    Word16       new_band_width[]        /* o:   new sub band bandwidth                    */
);

void spt_shorten_domain_band_save_fx(
    const Word16 bands,                  /* i:   total subband                */
    const Word16 band_start[],           /* i:   starting position of subband */
    const Word16 band_end[],             /* i:   end position of subband      */
    const Word16 band_width[],           /* i:   band width of subband        */
    Word16 org_band_start[],       /* o:   starting position of subband */
    Word16 org_band_end[],         /* o:   end position of subband      */
    Word16 org_band_width[]        /* o:   band width of subband        */
);

void spt_shorten_domain_band_restore_fx(
    const Word16 bands,                  /* i:   total subband                */
    Word16 band_start[],           /* i/o: starting position of subband */
    Word16 band_end[],             /* i/o: end position of subband      */
    Word16 band_width[],           /* i/o: band width of subband        */
    const Word16 org_band_start[],       /* o:   starting position of subband */
    const Word16 org_band_end[],         /* o:   end position of subband      */
    const Word16 org_band_width[]        /* o:   band width of subband        */
);

void spt_swb_peakpos_tmp_save_fx(
    const Word32 L_y2[],                   /* i:   coded spectral information   */
    const Word16 bands,                  /* i:   total number of bands        */
    const Word16 band_start[],           /* i:   starting position of subband */
    const Word16 band_end[],             /* i:   end position of subband      */
    Word16 prev_SWB_peak_pos_tmp[] /* o:   spectral peaks               */
);

Word16 div_s_ss(                        /* o: result of division (Word16 Q0) */
    const Word16 n,                      /* i: numerator   (Word16 Q0         */
    const Word16 d                       /* i: denominator (Word16 Q0)        */
);

void hf_parinitiz_fx(
    const Word32 L_total_brate,
    const Word16 hqswb_clas_fx,
    Word16 lowlength_fx,
    Word16 highlength_fx,
    Word16 wBands_fx[],
    const Word16 **subband_search_offset_fx,
    const Word16 **subband_offsets_fx,
    Word16 *nBands_fx,
    Word16 *nBands_search_fx,
    Word16 *swb_lowband_fx,
    Word16 *swb_highband_fx
);

void GetlagGains_fx(
    const Word16 *predBuf_fx,      /* i: Qss  Low freq. Smoothed Spectrum */
    const Word16 Qss,              /* i: Q0   Q value of predBuf          */
    const Word32 *L_band_energy,   /* i: Qbe  Band Energy                 */
    const Word16 Qbe,              /* i: Q0   Q value of band energy      */
    const Word16 nBands,           /* i: Q0   number of SWB subbands      */
    const Word16 *sbWidth,         /* i: Q0   width of SWB subbands       */
    const Word16 *lagIndices,      /* i: Q0   lagIndices                  */
    const Word16 predBufLen,       /* i: Q0   length of predBuf           */
    Word16 *lagGains_fx,     /* o: QlagGains lagGains               */
    Word16 *QlagGains        /* o: Q0   Q value of lagGains         */
);

void noise_extr_corcod_fx(
    Word32 L_spectra[],               /* i  : QsL core coder                                  */
    const Word32 L_spectra_ni[],            /* i  : QsL core coder with sparse filling              */
    Word16 sspectra_fx[],             /* o  : Qss Smoothed tonal information from core coder  */
    Word16 sspectra_diff_fx[],        /* o  : Qss non tonal infomration for gap filling       */
    Word16 sspectra_ni_fx[],          /* o  : Qss smoothed core coder                         */
    const Word16 fLenLow_fx,                /* i  : Q0  low frequency bands width                   */
    Word16 prev_hqswb_clas_fx,        /* i  : Q0  classification information                  */
    Word16 *prev_ni_ratio_fx,         /* i  : Q15 noise paraemeter                            */
    Word16 *Qss                       /* o  : Q0  Q value for sspectra_*_fx                   */
);

void ton_ene_est_fx(
    Word32 L_xSynth_har[],         /* i  : QsL  buffer with non tonal compoents   */
    Word16 QsL,                    /* i  : Q0   Q value for xSynth_har            */
    Word32 L_be_tonal[],           /* o  : QbeL tonal energy of the missing bands */
    Word16 *QbeL,                  /* o  : Q0   Q value for be_tonal              */
    Word32 L_band_energy[],        /* i  : Qbe  subband energies                  */
    Word16 Qbe,                    /* i  : Q0   Q value for band_energy           */
    const Word16 band_start[],           /* i  : Q0   subband start indices             */
    const Word16 band_end[],             /* i  : Q0   subband end indices               */
    const Word16 band_width[],           /* i  : Q0   subband widths                    */
    const Word16 fLenLow,                /* i  : Q0   low frequency width               */
    const Word16 fLenHigh,               /* i  : Q0   High frequency width              */
    Word16 bands,                  /* i  : Q0   total subbands                    */
    Word16 har_bands,              /* i  : Q0   total number of harmonics bands   */
    Word16 ni_lvl_fx,              /* i  : Q11  noise enve for the hf bands       */
    GainItem_fx pk_sf_fx[],          /* i  :                                        */
    Word16 Qss,                    /* i  : Q0   Q value for GainItem_fx->nmrValue */
    Word16 *pul_res                /* i  : Q0   tonal resolution                  */
);

void Gettonl_scalfact_fx
(
    Word32 *L_outBuf,                /* i/o: QsL synthesized spectrum                        */
    Word16 QsL,                      /* i  : Q0  Q value for outBuf                          */
    const Word32 *L_codbuf,                /* i  : QsL core coder                                  */
    const Word16 fLenLow,                  /* i  : Q0  lowband length                              */
    const Word16 fLenHigh,                 /* i  : Q0  highband length                             */
    const Word16 harmonic_band,            /* i  : Q0  total number of Low frequency bands         */
    const Word16 bands,                    /* i  : Q0  total number of subbands in a frame         */
    Word32 *L_band_energy,           /* i  : Qbe band energy of each subband                 */
    Word16 Qbe,                      /* i  : Q0  Q value for band_energy                     */
    const Word16 *band_start,              /* i  : Q0  subband start indices                       */
    const Word16 *band_end,                /* i  : Q0  subband end indices                         */
    const Word16 p2aflags[],               /* i  : Q0  missing bands in the core coder             */
    Word32 L_be_tonal[],             /* i  : QbeL tonal energy                               */
    Word16 QbeL,                     /* i  : Q0  Q value for be_tonal                        */
    GainItem_fx *pk_sf_fx,                /* i  :     toanl information for Sparse filling        */
    Word16 Qss,                      /* i  : Q0  Q value for pk_sf.nmrValue                  */
    Word16 *pul_res_pk               /* i  : Q0  pulse resolution information                */
);

void updat_prev_frm_fx(
    Word32 L_y2[],                   /* i/o: core coder buffer                 */
    Word32 L_t_audio[],              /*   o: core coder buffer                 */
    Word32 L_bwe_br,                 /*   i: core bitrate                      */
    Word16 length,                   /*   i: frame length coded bw             */
    const Word16 inner_frame,              /*   i: input frame length                */
    Word16 bands,                    /*   i: sub band resolution               */
    Word16 bwidth,                   /*   i: NB/WB/SWB indicator               */
    const Word16 is_transient,             /*   i: signal class information          */
    Word16 hqswb_clas,               /*   i: signal class information          */
    Word16 *prev_hqswb_clas,         /*   o: update signal class information   */
    Word16 prev_SWB_peak_pos[],      /*   o: update core coder last coded peaks*/
    Word16 prev_SWB_peak_pos_tmp[],  /*   o: update core coder last coded peaks*/
    Word16 prev_npulses[],           /*   o: update pulse resolution           */
    Word16 prev_npulses_tmp[],       /*   i: update pulse resolution           */
    Word16 *prev_frm_hfe2,           /*   o: update harmonics                  */
    Word16 *prev_stab_hfe2,          /*   o: update harmonics                  */
    Word16 bws_cnt                   /*   i: band width detector               */
);

void get_sigma_fx_har(
    const Word32 L_x_abs[],     /* i: Qi     absolute input    */
    const Word16 Qi,            /* i: Q0     Q value of x_abs  */
    const Word16 avg_fx,        /* i: Qavg   average of x_abs  */
    const Word16 Qavg,          /* i: Q0     Q value of avg    */
    const Word16 length_fx,     /* i: Q0     length            */
    Word16 *sigma_fx,     /* o: Qsigma sigma             */
    Word16 *Qsigma        /* o: Q0     Q value of sigma  */
);

void FindNBiggest2_simple_fx_har(
    const Word32 *L_inBuf,    /* i  : input buffer (searched)                     */
    const Word16 Qabs_in,     /* i  : Q value of input buffer                     */
    GainItem_fx *pk_sf_fx,   /* o  : N biggest components found                  */
    const Word16 nIdx_fx,     /* i  : search length                               */
    Word16 *n_fx,       /* i  : number of components searched (N biggest)   */
    Word16 n_nbiggestsearch
);

Word16 spectrumsmooth_noiseton_fx(   /* o  : Qss  ss_min                                      */
    Word32 L_spectra[],        /* i  : Qs   core coder                                  */
    /*Word16 Qs,*/               /* i  : Q0   Q value for spectra, spectra_ni             */
    const Word32 L_spectra_ni[],     /* i  : Qs   core coder with sparse filling              */
    Word16 sspectra_fx[],      /* o  : Qss  Smoothed tonal information from core coder  */
    Word16 sspectra_diff_fx[], /* o  : Qss  non tonal infomration for gap filling       */
    Word16 sspectra_ni_fx[],   /* o  : Qss  smoothed core coder                         */
    Word16 *Qss,               /* o  : Q0   Q value for sspectra*                       */
    const Word16 fLenLow_fx,         /* i  : Q0   low frequency boundaries                    */
    Word16 *ni_seed_fx         /* io : Q0   random seed                                 */
);

void noiseinj_hf_fx(
    Word32 L_xSynth_har[],          /* i/o : Qs   gap filled information            */
    Word16 Qs,                      /* i   : Q0   Q value for xSynth_har            */
    Word32 L_th_g[],                /* i   : Qs   level adjustment information      */
    Word32 L_band_energy[],         /* i   : Qbe  subband energies                  */
    Word16 Qbe,                     /* i   : Q0   Q value for band_energy           */
    Word16 *prev_En_sb_fx,          /* i/o : QsEn smoothed sqrt band Energies       */
    const Word16 p2a_flags_fx[],          /* i   : Q0   Missing bands in the core coder   */
    const Word16 BANDS_fx,                /* i   : Q0   total bands                       */
    const Word16 band_start_fx[],         /* i   : Q0   band start indices                */
    const Word16 band_end_fx[],           /* i   : Q0   band end indices                  */
    const Word16 fLenLow_fx,              /* i   : Q0   low frequency bandwidth           */
    const Word16 fLenHigh_fx              /* i   : Q0   SWB frequency bandwidth           */
);

void sqrt_32n_16_fx(
    Word32 L_in,              /* i  : input vector (Word32)       */
    Word16 Qin,               /* i  : Q value for L_in            */
    Word16 *out_fx,           /* o  : sqrt input vector (Word16)  */
    Word16 *Qout              /* o  : Q value for out_fx          */
);

void norm_vec_32_16_scale_fx(
    Word32 *L_vec_in,     /* i  : input vector                                        */
    Word16 Qin,           /* i  : Q value for input vector                            */
    Word16 length_fx,     /* i  :vector size                                          */
    Word16 *vec_out_fx,   /* o  : output vectror                                      */
    Word16 *Qout,         /* o  : Q value for output vectro                           */
    Word16 exp_safe       /* i  : suppress left shift: for prevend overflow on sum    */
);

void return_bits_normal2_fx(
    Word16 *bit_budget_fx,         /* i/o : bit budget                          */
    const Word16 p2a_flags_fx[],         /* i   : HF tonal indicator                  */
    const Word16 bands_fx,               /* i   : Total number of Subbands in a frame */
    const Word16 bits_lagIndices_fx[]    /* i   : bits for lagIndices                 */
);

Word16 peak_avrg_ratio_fx(
    const Word32 total_brate,
    const Word32 *input_hi_fx,               /* i  : input signal           */
    const Word16 length,                     /* i  : number of coefficients */
    Word16 *mode_count,                /* i/o: HQ_HARMONIC mode count */
    Word16 *mode_count1,               /* i/o: HQ_NORMAL mode count   */
    Word16 Q_coeff
);

void diffcod_fx(
    const Word16 N,         /* (i)   number of sub-vectors      */
    Word16 *y,        /* (i/o) indices of quantized norms */
    Word16 *difidx    /* (o)   differential code          */
);

void diffcod_lrmdct_fx(
    const Word16 N,                  /* i  : number of sub-vectors       */
    const Word16 be_ref,             /* i  : band energy reference */
    Word16 *y,                 /* i/o: indices of quantized norms */
    Word16 *difidx,            /* o  : differential code */
    const Word16 is_transient        /* i  : transient flag  */
);

Word16 encode_envelope_indices_fx(   /* o  : Number of bits if flag_pack=0,0 if flag_pack=1  Q0  */
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure   */
    const Word16 num_sfm,            /* i  : Number of subbands                              Q0  */
    const Word16 numnrmibits,        /* i  : Bitrate of fall-back coding mode                Q0  */
    Word16 *difidx,            /* i/o: Diff indices/encoded diff indices               Q0  */
    Word16 *LCmode,            /* o  : Coding mode if flag_pack=0, i : if flag_pack=1  Q0  */
    const Word16 flag_pack,          /* i  : indicator of packing or estimating bits         Q0  */
    const Word16 flag_HQ2            /* i  : indicator of HQ2 core                           Q0  */
    ,
    const Word16 is_transient        /* i  : indicator of HQ_TRANSIENT                       Q0  */
);
Word16 decode_envelope_indices_fx( /* o  : Number of bits                    */
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure */
    const Word16 start_norm,       /* i  : starting band index               */
    const Word16 num_sfm,          /* i  : Number of subbands                */
    const Word16 numnrmibits,      /* i  : Bitrate of fall-back coding mode  */
    Word16 *difidx,          /* o  : Diff indices/encoded diff indices */
    const Word16 flag_HQ2          /* i  : indicator of HQ2 core             */
    ,
    const Word16 is_transient      /* i  : indicator of HQ_TRANSIENT         */
);

void hdecnrm_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    const Word16 numNorms, /* (i)    number of norms */
    Word16 *index);        /* (o)    indices of quantized norms */

Word16 decode_huff_context_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 *hufftab,
    Word16 *rbits
);

void hdecnrm_context_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 N,
    Word16 *index,
    Word16 *n_length
);

void hdecnrm_resize_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 N,                 /* (i)  number of SFMs */
    Word16 *index             /* (o)  norm quantization index vector */
);

void hdecnrm_tran_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure   */
    const Word16 N,               /* i  : number of norms           */
    Word16 *index           /* o  : indices of quantized norms */
);

void mdct_spectrum_denorm_fx(
    const Word16 inp_vector[],       /* i   : Q0  :                                       */
    Word32 L_y2[],             /* i/o : Qs  : decoded spectrum                      */
    const Word16 band_start[],       /* i   : Q0  : table of start freq for every subband */
    const Word16 band_end[],         /* i   : Q0  : table of end freq for every subband   */
    const Word16 band_width[],       /* i   : Q0  : table of bandwidth for every subband  */
    const Word32 L_band_energy[],    /* i   : Qbe : band energy                           */
    const Word16 npulses[],          /* i   : Q0  : number of coded spectrum              */
    const Word16 bands,              /* i   : Q0  : numbers of subbands                   */
    const Word16 ld_slope_fx,        /* i   : Q15 :                                       */
    const Word16 pd_thresh_fx        /* i   : Q15 :                                       */
);

/* y(n)(Qx) = alpha(Q15) * x(Qx)  +  (1.0f-alpha)* y(n-1) (Qx) */
Word16 noise_est_AR1_Qx(                /* o: Qx   y(n)  */
    Word16 x,     /* i : Qx  x(n)  */
    Word16 y,     /* i : Qx  y(n-1) */
    Word16 alpha /*i : Q15 scaling of driving x(n)  */
);
void  noise_est_fx(
    Encoder_State_fx *st_fx,        /* i/o: state structure                                                      */
    const Word32 tmpN[],          /* i  : temporary noise update                            Q_new + QSCALE    */
    const Word16 *pit,          /* i  : open-loop pitch values for each half-frame           Q0             */
    const Word16 cor[],          /* i  : normalized correlation for all half-frames           Q15          */
    const Word16 epsP_h[],        /* i  : msb prediction error energies                        Q_r-1         */
    const Word16 epsP_l[],        /* i  : msb prediction error energies                        Q_r-1         */
    const Word16 Etot,          /* i  : total channel E (see find_enr_fx.c)                  Q8            */
    const  Word16 relE,                   /* i  : (VA_CHECK addition) relative frame energy            Q8?            */
    const Word16 corr_shift,        /* i  : normalized correlation correction                    Q15           */
    const Word32 enr[],          /* i  : averaged energy over both subframes               Q_new + Q_SCALE  */
    Word32 fr_bands[],        /* i  : spectrum per critical bands of the current frame  Q_new + Q_SCALE   */
    Word16 *cor_map_sum,      /* o  :                               Q8        */
    Word16 *sp_div,          /* o  :                            Q_sp_div      */
    Word16 *Q_sp_div,        /* o  :  Q factor for sp_div                                                */
    Word16 *non_staX,               /* o  : non-stationarity for sp/mus classifier                            */
    Word16 *loc_harm ,              /* o  :   multi-harmonicity flag for UV classifier                          */
    const Word32 *lf_E,                   /* i  : per bin energy  for low frequencies             Q_new + Q_SCALE -2  */
    Word16 *st_harm_cor_cnt,    /* i/o  : 1st harm correlation timer                         Q0           */
    const Word16 Etot_l_lp,        /* i    : Smoothed low energy                                Q8         */
    const Word16 Etot_v_h2,        /* i    : Energy variations                                  Q8           */
    Word16 *bg_cnt,          /* i    : Background burst length timer                      Q0           */
    Word16 EspecdB[],               /* i/o: log E spectrum (with f=0) of the current frame       Q7             */
    Word16 Q_new ,                  /* i  : Scaling of current frame                                            */

    const Word32 Le_min_scaled            /*i  : Minimum energy value     */
    , Word16 *sp_floor                /* o  : noise floor estimate                                 Q7             */
);


void autocorr_fx(
    const Word16 x[],       /* i  : Input signal                    */
    const Word16 m,         /* i  : LPC order                   Q0  */
    Word16 r_h[],     /* o  : Autocorrelations  (msb)     Q15 */
    Word16 r_l[],     /* o  : Autocorrelations  (lsb)         */
    Word16 *Q_r,      /* o  : normailsation shift of r    Q0  */
    const Word16 len,       /* i  : Frame lenght                    */
    const Word16* wind,     /* i  : Window used                     */
    Word16 rev_flag,
    const Word16 sym_flag   /* i  : symmetric window flag           */
);
void freq_dnw_scaling_fx(
    const Word16 cor_strong_limit, /* i  : HF correlation                */
    const Word16 coder_type,       /* i  : coder type                    */
    const Word16 noise_lev,        /* i  : Noise level                   */
    const Word32  core_brate,       /* i  : Core bitrate                  */
    Word16 fy_norm[],        /* i/o: Frequency quantized parameter */
    Word16 Qx        /* Q format of fy_norm*/
);
void Comp_and_apply_gain_fx(
    Word16 exc_diffQ[],       /* i/o: Quantized excitation                  */
    Word16 Ener_per_bd_iQ[],  /* i  : Target ener per band              Q13 */
    Word16 Ener_per_bd_yQ[],  /* i/o  : Ener per band for norm vector     i->Q13/o->Q13 */
    Word16 Mbands_gn,         /* i  : number of bands                  */
    const Word16 ReUseGain  ,        /* i  : Reuse the gain in Ener_per_bd_yQ  */
    Word16 Qexc_diff,
    Word16 Q_exc
);
void dec_pit_exc_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder static memory                     */
    const Word16 *Aq_fx,                  /* i  : LP filter coefficient                     */
    const Word16 coder_type_fx,           /* i  : coding type                               */
    const Word16 Es_pred_fx,              /* i  : predicted scaled innov. energy            */
    Word16 *pitch_buf_fx,           /* o  : floating pitch values for each subframe   */
    Word16 *code_fx,                /* o  : innovation                                */
    Word16 *exc_fx,                 /* i/o: adapt. excitation exc                     */
    Word16 *bwe_exc_fx,             /* o  : excitation for SWB TBE                    */
    const Word16 nb_subfr_fx              /* i  : Number of subframe considered             */
    ,     Word16 *gain_buf                  /*Q14*/
);
Word16 mean_fx(            /* o  : mean of vector                         */
    const Word16 *vec_fx,  /* i  : input vector                           */
    const Word16 lvec_fx   /* i  : length of input vector                 */
);
void gsc_dec_fx(
    Decoder_State_fx *st_fx,              /* i/o: State structure                     */
    Word16 exc_dct_in[],     /* i/o: dct of pitch-only excitation / total excitation */
    const Word16 pit_band_idx,  /* i  : bin position of the cut-off frequency   */
    const Word16 Diff_len,     /* i  : Lenght of the difference signal (before pure spectral)*/
    const Word16 bits_used,      /* i  : Number of bit used before frequency Q                 */
    const Word16 nb_subfr,    /* i  : Number of subframe considered             */
    const Word16 coder_type,     /* i  : coding type                                           */
    Word16 *last_bin,        /* i  : last bin of bit allocation                            */
    Word16 *lsf_new,         /* i  : ISFs at the end of the frame                          */
    Word16 *exc_wo_nf,        /* o  : excitation (in f domain) without noisefill   */
    Word16 Q_exc
);
void fft_rel_fx(
    Word16 x[],  /* i/o: input/output vector    */
    const Word16 n,    /* i  : vector length          */
    const Word16 m     /* i  : log2 of vector length  */
);
void ifft_rel_fx(
    Word16 io[],  /* i/o: input/output vector   */
    const Word16 n,     /* i  : vector length         */
    const Word16 m      /* i  : log2 of vector length */
);

Word16 gsc_gaindec_fx(                  /* o  : average frequency gain    */
    Decoder_State_fx *st_fx,              /* i/o: decoder state structure   */
    Word16 y_gainQ_fx[],        /* o  : quantized gain per band   */
    const Word32 core_brate_fx,       /* i  : core used                 */
    Word16 old_y_gain_fx[],     /* i/o: AR gain quantizer for low rate */
    const Word16 coder_type_fx,       /* i  : coding type               */
    const Word16 bwidth_fx            /* i  : input signal bandwidth    */
);
void Copy_Scale_sig(
    const Word16 x[],   /* i  : signal to scale input           Qx        */
    Word16 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);

void bands_and_bit_alloc_fx(
    const Word16 cor_strong_limit, /* i  : HF correlation                                        */
    const Word16 noise_lev,        /* i  : dwn scaling factor                                    */
    const Word32 core_brate,       /* i  : core bit rate                                         */
    const Word16 Diff_len,         /* i  : Lenght of the difference signal (before pure spectral)*/
    const Word16 bits_used,        /* i  : Number of bit used before frequency Q                 */
    Word16 *bit,             /* i/o: Number of bit allowed for frequency quantization      */
    const Word16 *Ener_per_bd_iQ,  /* i/o: Quantized energy vector                               */
    Word16 *max_ener_band,   /* o  : Sorted order                                          */
    Word16 *bits_per_bands,  /* i/o: Number of bit allowed per allowed subband        Q3   */
    Word16 *nb_subbands,     /* o  : Number of subband allowed                             */
    const Word16 *exc_diff,        /* i  : Difference signal to quantize (encoder side only)     */
    Word16 *concat_in,       /* o  : Concatened PVQ's input vector (encoder side only)     */
    Word16 *pvq_len,         /* o  : Number of bin covered with the PVQ                    */
    const Word16 coder_type,       /* i  : coding type                                           */
    const Word16 bwidth,           /* i  : input signal bandwidth                                */
    const Word16 GSC_noisy_speech
);


void decod_audio_fx(
    Decoder_State_fx *st_fx,             /* i/o: decoder static memory                     */
    Word16 dct_epit[],           /* o  : GSC excitation in DCT domain              */
    const Word16 *Aq,                  /* i  : LP filter coefficient                     */
    const Word16 coder_type,           /* i  : coding type                               */
    Word16 *pitch_buf,           /* o  : floating pitch values for each subframe   */
    Word16 *voice_factors,       /* o  : voicing factors                           */
    Word16 *exc,                 /* i/o: adapt. excitation exc                     */
    Word16 *exc2,                /* i/o: adapt. excitation/total exc               */
    Word16 *bwe_exc,             /* o  : excitation for SWB TBE                    */
    Word16 *lsf_new              /* i  : ISFs at the end of the frame              */
    , Word16 *gain_buf                /*Q14*/
);
Word16 Pit_exc_contribution_len_fx(   /* o  : bin where pitch contribution is significant */
    Encoder_State_fx *st_fx,              /* i/o: state structure                                  */
    const Word16 *dct_res,         /* i  : DCT of residual                 */
    Word16 *dct_pitex,       /* i/o: DCT of pitch contribution       */
    Word16 *pitch_buf,       /* i/o: Pitch per subframe              */
    const Word16 nb_subfr,         /* i  : Number of subframe considered   */
    Word16 *hangover,         /* i  : hangover for the time contribution switching */
    const Word16 coder_type,        /* i  : coding type */
    Word16 Qnew
);
void enc_pit_exc_fx(
    Encoder_State_fx *st_fx,                   /* i/o: State structure                                  */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 *speech,               /* i  : Input speech                                     */
    const Word16 Aw[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq[],                 /* i  : 12k8 Lp coefficient                               */
    const Word16 Es_pred,               /* i  : predicted scaled innov. energy                   */
    const Word16 *T_op,                 /* i  : open loop pitch                                  */
    const Word16 *voicing,              /* i  : voicing                                          */
    const Word16 *res,                  /* i  : residual signal                                  */
    Word16 *synth,                /* i/o: core synthesis                                   */
    Word16 *exc,                  /* i/o: current non-enhanced excitation                  */
    Word16 *T0,                   /* i/o: close loop integer pitch                         */
    Word16 *T0_frac,              /* i/o: close-loop pitch period - fractional part        */
    Word16 *pitch_buf,            /* i/o: Fractionnal per subframe pitch                   */
    const Word16 nb_subfr,              /* i  : Number of subframe considered                    */
    Word16 *gpit,                 /* o  : pitch mean gpit                                  */
    Word16 *saved_bit_pos,         /* o  : saved position in the bitstream before pitch contribution */
    Word16 Q_new,
    Word16 shift
);
void encod_amr_wb_fx(
    Encoder_State_fx *st,               /* i/o: state structure                                  */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 speech[],          /* i  : input speech                                     */
    const Word16 Aw[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq[],                 /* i  : 12k8 Lp coefficient                               */
    const Word16 pitch[3],          /* i  : open-loop pitch values for quantiz.              */
    const Word16 voicing[],         /* i  : voicing                                          */
    const Word16 *res,              /* i  : residual signal                                  */
    Word16 *syn,              /* i/o: core synthesis                                   */
    Word16 *exc,              /* i/o: current non-enhanced excitation                  */
    Word16 *exc2,             /* i/o: current enhanced excitation                      */
    Word16 *pitch_buf,        /* i/o: floating pitch values for each subframe          */
    Word16 hf_gain_fx[NB_SUBFR], /* o  : decoded HF gain                                  */
    const Word16 *speech16k_fx,
    Word16 shift,
    Word16 Q_new
);
void amr_wb_enc_fx(
    Encoder_State_fx *st,                      /* i/o: encoder state structure             */
    const Word16 input_sp[],               /* i  : input signal                        */
    const Word16 n_samples                 /* i  : number of input samples             */
);
void encod_gen_voic_fx(
    Encoder_State_fx *st_fx,                   /* i/o: state structure                                  */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 L_frame_fx,               /* i  : length of the frame                              */
    const Word16 sharpFlag_fx,            /* o  : formant sharpening flag                          */
    const Word16 speech_fx[],              /* i  : input speech                                     */
    const Word16 Aq_fx[],                  /* i  : 12k8 Lp coefficient                              */
    const Word16 A_fx[],                   /* i  : unquantized A(z) filter with bandwidth expansion */
    const Word16 coder_type_fx,            /* i  : coding type                                      */
    const Word16 Es_pred_fx,               /* i  : predicted scaled innov. energy                   */
    const Word16 T_op_fx[],                /* i  : open loop pitch                                  */
    const Word16 voicing_fx[],             /* i  : voicing                                          */
    const Word16 *res_fx,                  /* i  : residual signal                                  */
    Word16 *syn_fx,                  /* i/o: core synthesis                                   */
    Word16 *exc_fx,                  /* i/o: current non-enhanced excitation                  */
    Word16 *exc2_fx,                 /* i/o: current enhanced excitation                      */
    Word16 *pitch_buf_fx,            /* i/o: floating pitch values for each subframe          */
    Word16 *voice_factors_fx,        /* o  : voicing factors                                  */
    Word16 *bwe_exc_fx,              /* o  : excitation for SWB TBE                           */
    Word16 *unbits_fx,                   /* i/o: number of unused bits              */
    Word16 shift,
    Word16 Q_new
);
void bw_detect_fx(
    Encoder_State_fx *st,                     /* i/o: Encoder State       */
    const Word16 signal_in[],             /* i  : input signal        */
    const Word16 localVAD,
    Word32 *enerBuffer,                   /* i  : CLDFB Energy          */
    Word16 *cldfbBuf_Ener_Exp               /* i  : CLDFB Energy Exponent */
);
void hf_synth_amr_wb_init_fx(
    Word16 *prev_r,                    /* o  : previous sub-frame gain                  */
    Word16 *fmerit_w_sm,               /* o  : 1 sample memory fmerit_w param*/
    Word16 *frame_count,               /* o  : frame counter initialization                     */
    Word16 *ne_min,                    /* o  : minimum Noise gate - short-term energy initialization*/
    Word16 *fmerit_m_sm,               /* o  : 1 sample memory fmerit_m param                   */
    Word16 *voice_fac,                 /* o  : voice factor initialization                      */
    Word16 *unvoicing,                 /* o  : unvoiced parameter                               */
    Word16 *unvoicing_sm,              /* o  : smoothed unvoiced parameter                      */
    Word16 *unvoicing_flag,            /* o  : unvoiced flag                                    */
    Word16 *voicing_flag,              /* o  : voiced flag                                      */
    Word16 *start_band_old,            /* o  : previous start point for copying frequency band  */
    Word32 *OptCrit_old                /* o  : previous criterion value for deciding the start point */
);

void hf_synth_amr_wb_reset_fx(
    Word16 *seed2,                     /* i/o: random seed for HF noise gen                      */
    Word16 mem_syn_hf[],               /* o  : HF synthesis memory                               */
    Word16 mem_hp_interp[],            /* o  : interpol. memory                                  */
    Word16 *prev_r,                    /* o  : previous sub-frame gain                  */
    Word16 *fmerit_w_sm,               /* o  : 1 sample memory fmerit_w param                    */
    Word16 delay_syn_hf[],             /* o  : HF synthesis memory                               */
    Word16 *frame_count,               /* o  : frame counter memory                              */
    Word16 *ne_min,                    /* o  : minimum Noise gate - short-term energy memory     */
    Word16 *fmerit_m_sm,               /* o  : 1 sample memory fmerit_m param                    */
    Word16 *voice_fac,                 /* o  : voice factor memory                               */
    Word16 *unvoicing,                 /* o  : unvoiced parameter                               */
    Word16 *unvoicing_sm,              /* o  : smoothed unvoiced parameter                      */
    Word16 *unvoicing_flag,            /* o  : unvoiced flag                                    */
    Word16 *voicing_flag,              /* o  : voiced flag                                      */
    Word16 *start_band_old,            /* o  : previous start point for copying frequency band  */
    Word32 *OptCrit_old                /* o  : previous criterion value for deciding the start point */
);

void hf_synth_amr_wb_fx(
    const Word32  core_brate,               /* i  : core bitrate                                      */
    const Word16 output_frame,              /* i  : output frame length                               */
    const Word16 *Aq,                       /* i  : quantized Az : Q12                                */
    const Word16 *exc,                      /* i  : excitation at 12.8 kHz  : Qexc                    */
    Word16 *synth,                    /* i/o: synthesis signal at 12.8k : Qsyn                  */
    Word16 *mem_syn_hf,               /* i/o: HF synthesis memory : Qout                        */
    Word16 *delay_syn_hf,             /* i/o: HF synthesis memory  : Qout                       */
    Word16 *prev_r,                   /* i/o  : previous sub-frame gain  : Q10                  */
    Word16 *fmerit_w_sm,              /* i/o: smoothed fmerit_w    : Q14                        */
    Word16 *amr_io_class,             /* i  : signal class (determined by FEC algorithm)        */
    Word16 *mem_hp_interp,            /* i/o: interpol. memory : Qout                           */
    Word16 *synth_out,                /* i/o: output signal at output Fs  : Qout                */
    Word16 fmerit,                    /* i  : classify parameter from FEC : Q14                 */
    const Word16 *hf_gain,                  /* i  : decoded HF gain                                   */
    const Word16 *voice_factors,            /* i  : voicing factors  : Q15                            */
    const Word16 pitch_buf[],               /* i  : pitch buffer  : Q5                                */
    const Word16 ng_ener_ST,                /* i  : Noise gate - short-term energy : Q8               */
    const Word16 *lsf_new,                  /* i  : ISF vector  : Q2                                  */
    Word16 *frame_count,              /* i/o: frame counter                                     */
    Word16 *ne_min,                   /* i/o: minimum Noise gate  : Q8                          */
    Word16 *fmerit_m_sm,              /* i/o: smoothed fmerit_m   : Q14                         */
    Word16 *voice_facor_sm,           /* o  : voice factor memory : Q15                         */
    Word16 *unvoicing,                /* o  : unvoiced parameter  : Q15                         */
    Word16 *unvoicing_sm,             /* o  : smoothed unvoiced parameter  : Q15                */
    Word16 *unvoicing_flag,           /* o  : unvoiced flag                                     */
    Word16 *voicing_flag,             /* o  : voiced flag                                       */
    Word16 *start_band_old,           /* o  : previous start point for copying frequency band   */
    Word32 *OptCrit_old,              /* o  : previous criterion value for deciding the start point */
    const Word16 Q_exc,                      /* i  : exc scaling                                       */
    const Word16 Q_out                      /* i  : synth_out scaling                                 */
);

void hf_cod_init_fx(
    Word16 *mem_hp400_enc,             /* o: memory of hp 400 Hz filter   */
    Word16 *mem_hf1_enc,               /* o: HF band-pass filter memory   */
    Word16 *mem_syn_hf_enc,            /* o: HF synthesis memory          */
    Word16 *mem_hf2_enc,               /* o: HF band-pass filter memory   */
    Word16 *gain_alpha_fx                 /* o: smoothing gain for transitions between active and inactive frames */
);

void hf_cod_fx(
    const Word32 core_brate_fx,                     /* i  : core bitrate                 */
    const Word16 *speech16k_fx,                     /* i  : original speech at 16 kHz    */
    const Word16 Aq_fx[],                           /* i  : quantized Aq                 */
    const Word16 exc_fx[],                          /* i  : excitation at 12.8 kHz       */
    Word16 synth_fx[],                        /* i  : 12.8kHz synthesis signal     */
    Word16 *seed2_enc_fx,                     /* i/o: random seed for HF noise gen */
    Word16 *mem_hp400_enc_fx,                 /* i/o: memory of hp 400 Hz filter   */
    Word16 *mem_syn_hf_enc_fx,                /* i/o: HF synthesis memory          */
    Word16 *mem_hf1_enc_fx,                   /* i/o: HF band-pass filter memory   */
    Word16 *mem_hf2_enc_fx,                   /* i/o: HF band-pass filter memory   */
    const Word16 dtxHangoverCount_fx,
    Word16 *gain_alpha_fx,                    /* i/o: smoothing gain for transitions between active and inactive frames */
    Word16 *hf_gain_fx,                        /* o  :  HF gain to be transmitted to decoder */
    Word16 Q_exc,
    Word16 Q_syn
);

void re8_compute_base_index_fx(
    const Word16 *x,        /* i  : Elemen of Q2, Q3 or Q4                          */
    const Word16 ka,        /* i  : Identifier of the absolute leader related to x  */
    UWord16 *I         /* o  : index                                           */
);

void InitSWBencBuffer_fx(
    Encoder_State_fx* st_fx /* i/o: SHB encoder structure */
);

void ResetSHBbuffer_Enc_fx(
    Encoder_State_fx* st_fx /* i/o: SHB encoder structure */
);
void wb_tbe_extras_reset_synth_fx( Word16 state_lsyn_filt_shb[], Word16 state_lsyn_filt_dwn_shb[],
                                   Word16 state_32and48k_WB_upsample[]
                                   ,Word16 state_resamp_HB[]
                                 );

void pz_filter_sp_fx (
    const Word16 b [],
    const Word16 a [],
    Word16 x [],
    Word16 y [],
    Word16 buf [],
    Word16 PNR,
    Word16 PDR,
    Word16 N,
    Word16 Qa
);

void Decimate_allpass_steep_fx(
    const Word16 *in_fx,
    Word16 state_fx[],          /* array of size: 2*ALLPASSSECTIONS_STEEP+1 */
    Word16 N,                   /* number of input samples */
    Word16 out_fx[]             /* array of size N/2 */
);

Word32 root_a_fx(
    Word32 a,
    Word16 Q_a,
    Word16 *exp_out
);

Word32 root_a_over_b_fx(
    Word32 a,
    Word16 Q_a,
    Word32 b,
    Word16 Q_b,
    Word16 *exp_out
);


void fir_fx(
    const Word16 x[],                        /* i  : input vector                              */
    const Word16 h[],                        /* i  : impulse response of the FIR filter        */
    Word16 y[],                        /* o  : output vector (result of filtering)       */
    Word16 mem[],                      /* i/o: memory of the input signal (M samples)    */
    const Word16 L,                          /* i  : input vector size                         */
    const Word16 K,                          /* i  : order of the FIR filter (M+1 coefs.)      */
    const Word16 upd,                        /* i  : 1 = update the memory, 0 = not            */
    Word16 shift                       /* i  : difference between Q15 and scaling of h[] */
);


void Interpolate_allpass_steep_fx(
    const Word16 *in_fx,              /* i  : input signal                        */
    Word16 state_fx[],                /* array of size: 2*ALLPASSSECTIONS_STEEP+1 */
    Word16  N,                        /* number of input samples                  */
    Word16 out_fx[]                   /* o  : output signal, size 2*N             */
);

void interpolate_3_over_2_allpass_fx(
    const Word16 *input_fx,
    const Word16 len,
    Word16 *out_fx,
    Word16 *mem_fx,
    const Word16 *filt_coeff_fx
);
void interpolate_3_over_1_allpass_fx(
    const Word16 *input_fx,
    const Word16 len,
    Word16 *out_fx,
    Word16 *mem_fx,
    const Word16 *filt_coeff_fx
);
void decimate_2_over_3_allpass_fx(
    const Word16 *input_fx,
    const Word16 len,
    Word16 *out_fx,
    Word16 *mem_fx,
    const Word16 *filt_coeff_fx,
    const Word16 *lp_num_fx,
    const Word16 *lp_den_fx,
    Word16 *lp_mem_fx
);

void retro_interp4_5_fx(
    const Word16 *syn_fx,
    Word16 *pst_old_syn_fx
);
void retro_interp5_4_fx(
    Word16 *pst_old_syn_fx
);

void lsp2lpc_fx(
    Word16 *a,
    Word16 *freq,
    Word16 *prev_a,
    Word16 order
);

void PostShortTerm_fx(
    Word16 *sig_in,             /* i  : input signal (pointer to current subframe */
    Word16 *lpccoeff,           /* i  : LPC coefficients for current subframe */
    Word16 *sig_out,            /* o  : postfiltered output */
    Word16 *mem_stp,            /* i/o: postfilter memory*/
    Word16 *ptr_mem_stp,        /* i/o: pointer to postfilter memory*/
    Word16 *ptr_gain_prec,      /* i/o: for gain adjustment*/
    Word16 *mem_zero,           /* i/o: null memory to compute h_st*/
    Word16 formant_fac_fx       /* i  : Strength of post-filter*/
);

void GenShapedWBExcitation_fx(
    Word16 *excSHB,                    /* o   : synthesized shaped shb exctiation      */
    const Word16 *lpc_shb,                   /* i   : lpc coefficients                       */
    Word16 *exc4kWhtnd,                /* o   : whitened synthesized shb excitation    */
    Word32 *mem_csfilt,                /* i/o : memory                                 */
    Word16 *mem_genSHBexc_filt_down1,  /* i/o : memory                                 */
    Word16 *mem_genSHBexc_filt_down2,  /* i/o : memory                                 */
    Word16 *mem_genSHBexc_filt_down3,  /* i/o : memory                                 */
    Word16 *state_lpc_syn,             /* i/o : memory                                 */
    const Word16 coder_type,                 /* i   : coding type                            */
    const Word16 *bwe_exc_extended,          /* i   : bandwidth extended exciatation         */
    const Word16 Q_bwe_exc,
    Word16 bwe_seed[],                 /* i/o : random number generator seed           */
    const Word16 voice_factors[],             /* i   : voicing factor                         */
    const Word16 signal_type
    , const Word16 igf_flag
);

void GenWBSynth_fx(
    const Word16 *input_synspeech,           /* i  : input synthesized speech                */
    Word16 *shb_syn_speech_16k,        /* o  : output highband compnent                */
    Word16 *state_lsyn_filt_shb1,      /* i/o: memory                                  */
    Word16 *state_lsyn_filt_shb2       /* i/o: memory                                  */
);

void GenSHBSynth_fx(
    const Word16 *shb_target_speech,         /* i  : input synthesized speech                */
    Word16 *shb_syn_speech_32k,        /* o  : output highband component               */
    Word32 Hilbert_Mem[],              /* i/o: memory                                  */
    Word16 genSHBsynth_allpass_mem[],  /* i/o: memory                                  */
    const Word16 L_frame,                    /* i  : ACELP Frame length                      */
    Word16 *syn_dm_phase
);

Word16 swb_formant_fac_fx(  /* o : Formant filter strength [0,1] */
    const Word16   lpc_shb2, /* Q12 i : 2nd HB LPC coefficient */
    Word16*         tilt_mem /* i/o: Tilt smoothing memory */
);

void compute_poly_product_fx(Word16 *coef, Word32 *p,Word16 order);

void ScaleShapedSHB_fx(
    const Word16 length,                     /* i  : SHB overlap length                      */
    Word16 *synSHB,                    /* i/o: synthesized shb signal                  */
    Word16 *overlap,                   /* i/o: buffer for overlap-add                  */
    const Word16 *subgain,                   /* i  : subframe gain                           */
    const Word32 frame_gain,                 /* i  : frame gain                              */
    const Word16 *win,                       /* i  : window                                  */
    const Word16 *subwin,                     /* i  : subframes window                        */
    Word16 *Q_bwe_exc
    ,Word16 *Qx
    ,Word16 n_mem3
    ,Word16 prev_Q_bwe_syn2
);

void ScaleShapedWB_fx(
    const   Word16 length,          /* i    : SHB overlap length                 */
    Word16* synSHB,         /* i/o  : synthesized shb signal             */
    Word16* overlap,        /* i/o  : buffer for overlap-add             */
    const   Word16* subgain,        /* i    : subframe gain                      */
    const   Word32 frame_gain,      /* i    : frame gain                         */
    const   Word16* win,            /* i    : window                             */
    const   Word16* subwin,         /* i    : subframes window                   */
    const   Word16 Q_bwe_exc
    ,Word16 L_frame          /* i    : Frame length - determines whether 12.8 or 16kHz core in-use */
    ,Word16 dynQ             /* i    : indicate whether output is dynamic Q, or Q0                 */
    ,Word16* Qx              /* o    : newly computed Q factor for  synSHB                         */
    ,Word16 prev_Qx          /* i    : prev_Qx for memory scaling                                  */
    ,Word32* Hilbert_Mem     /* i    : Hilbert memory used for computing Qx                        */
);

void GenShapedSHBExcitation_fx(
    Word16 *excSHB,                                 /* o : synthesized shaped shb excitation Q_bwe_exc*/
    const Word16 *lpc_shb,                                /* i : lpc coefficients Q12*/
    Word16 *White_exc16k_FB,                        /* o : white excitation for the Fullband extension Q_bwe_exc */
    Word32 *mem_csfilt,                             /* i/o: memory */
    Word16 *mem_genSHBexc_filt_down_shb,            /* i/o: memory */
    Word16 *state_lpc_syn,                          /* i/o: memory */
    const  Word16 coder_type,                             /* i : coding type */
    const Word16 *bwe_exc_extended,                       /* i : bandwidth extended excitation */
    Word16 bwe_seed[],                              /* i/o: random number generator seed */
    Word16 voice_factors[],                         /* i : voicing factor*/
    const Word16 FB_flag,                                 /* i : FB flag indicator */
    Word16 *tbe_demph,                              /* i/o: de-emphasis memory */
    Word16 *tbe_premph,                             /* i/o: pre-emphasis memory */
    Word16 *lpc_shb_sf,                             /* i:   LP coefficients  */
    const Word32 shb_ener_sf_32,
    Word16 *shb_res_gshape,
    Word16 *shb_res,
    Word16 *vf_ind,
    const Word16 formant_fac,                             /* i   : Formant sharpening factor [0..1] */
    Word16 fb_state_lpc_syn[],                      /* i/o: memory */
    Word16 *fb_tbe_demph,                           /* i/o: fb de-emphasis memory */
    Word16 *Q_bwe_exc,
    Word16 *Q_bwe_exc_fb,
    const Word16 Q_shb
    ,Word16 n_mem2                 /* i :  n_mem2 scale factor to adjust 24.4/32kbps memories */
    ,Word16 prev_Q_bwe_syn         /* i :  st_fx->prev_Q_bwe_syn */
    ,Word32 bitrate
);


void space_lsfs_fx (
    Word16 *lsfs,                      /* i/o: Line spectral frequencies   */
    const Word16 order                       /* i  : order of LP analysis        */
);


void lsp_weights_fx(
    Word16 lsp_nq_fx[],
    Word16 w[],
    Word16 Order,
    Word16 *Qout
);


void lsp_convolve_fx(Word32 x, Word32 *p1, Word32 *p2, Word16 len);
Word32 poscos_fx(Word16 w);

void flip_spectrum_and_decimby4_fx(
    const Word16 input[],                    /* i  : input spectrum                          */
    Word16 output[],                   /* o  : output  spectrum                        */
    const Word16 length,                     /* i  : vector length                           */
    Word16 mem1[],                     /* i/o  : memory                                */
    Word16 mem2[],                     /* i/o  : memory                                */
    const Word16 ramp_flag                   /* i  : flag to trigger slow ramp-up of output following change of core */
);


Word16 lpc2lsp_fx(
    Word32 *a,
    Word16 *freq,
    Word16 *old_freq,
    Word16 order
);

Word16 usquant_fx(          /* o: index of the winning codeword   */
    const Word16 x,      /* i: scalar value to quantize        Qx*/
    Word16 *xq,    /* o: quantized value                 Qx*/
    const Word16 qlow,   /* i: lowest codebook entry (index 0) Qx*/
    const Word16 delta,  /* i: quantization step               Qx-1*/
    const Word16 cbsize  /* i: codebook size                   */
);

void wb_tbe_enc_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure             */
    const Word16 coder_type,           /* i  : coding type                         */
    const Word16 *hb_speech,           /* i  : HB target signal (6-8kHz) at 16kHz  */
    const Word32 *bwe_exc_extended,    /* i  : bandwidth extended exciatation      */
    const Word16 Q_new,                /* i  : input HB speech Q factor */
    const Word16 voice_factors[],      /* i  : voicing factors                     */
    const Word16 pitch_buf[],          /* i  : pitch for each subframe             */
    const Word16 voicing[]             /* i  : OL maximum normalized correlation   */
);

void wb_pre_proc_fx(
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure */
    const Word16 *new_inp_resamp16k,   /* i  : original input signal */
    Word16 *hb_speech            /* o  : HB target signal (6-8kHz) at 16kHz  */
);


void pz_filter_dp_fx (
    const Word16 b [],
    const Word16 a [],
    Word16 x [],
    Word16 y [],
    Word32 buf [],
    Word16 PNR,
    Word16 PDR,
    Word16 N,
    Word16 Qa
);

void flip_and_downmix_generic_fx(
    Word16 input[],                    /* i  : input spectrum                          */
    Word16 output[],                   /* o  : output  spectrum                        */
    const Word16 length,                     /* i  : length of spectra                       */
    Word32 mem1_ext[HILBERT_ORDER1],    /* i/o: Hilbert filter memory     */
    Word32 mem2_ext[2*HILBERT_ORDER2],  /* i/o: memory                  */
    Word32 mem3_ext[2*HILBERT_ORDER2],  /* i/o: memory                  */
    Word16 *phase_state                /* i/o: Phase state in case frequency isn't multiple of 50 Hz */
);


void wb_tbe_dec_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure                */
    const Word16 coder_type,                 /* i  : coding type */
    Word32 *bwe_exc_extended,          /* i  : bandwidth extended exciatation          */
    const Word16 Q_exc,
    const Word16 voice_factors[],            /* i  : voicing factors                         */
    Word16 *synth,                     /* i/o: ACELP core synthesis/final synthesis   */
    Word16 *Q_synth
);

void swb_tbe_enc_fx(
    Encoder_State_fx *st_fx,                        /* i/o: encoder state structure                 */
    const Word16 coder_type_fx,                 /* i  : coding type                             */
    Word16 *new_speech_fx,                /* i  : original input signal                   */
    Word32 *bwe_exc_extended_fx,          /* i  : bandwidth extended excitation           */
    const Word16 voice_factors_fx[],            /* i  : voicing factors                         */
    Word16 *White_exc16k_fx,              /* o  : shaped white excitation for the FB TBE  */
    Word16 *Q_white_exc,                  /* o  : generated white noise for FB Q factor   */
    Word16 Q_bwe_exc,                     /* i  : bandwidth extended excitation Q factor  */
    Word16 Q_shb,                         /* i  : SHB target Q factor */
    Word16 *voicing_fx,                   /* i  : OL maximum normalized correlation       */
    const Word16 pitch_buf[]                    /* i  : pitch for each subframe                 */
);

void swb_pre_proc_fx(
    Encoder_State_fx *st_fx,                     /* i/o: encoder state structure                 */
    const Word16 *input_fx,                  /* i  : original input signal                   */
    Word16 *new_swb_speech_fx,         /* o  : original input signal at 32kHz          */
    Word16 *shb_speech_fx,             /* o  : SHB target signal (6-14kHz) at 16kHz    */
    Word16 *Q_shb_spch                 /* o  : shb target signal Q factor              */
    , Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX]  /* i : real buffer        */
    , Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX]  /* i : imag buffer        */
    ,const CLDFB_SCALE_FACTOR *cldfbScale                                 /* i : scale data of real and imag CLDFB buffers */
);

void wb_tbe_extras_reset_fx(
    Word16  mem_genSHBexc_filt_down_wb2[],
    Word16  mem_genSHBexc_filt_down_wb3[]
);

void swb_tbe_reset_synth_fx(
    Word32 genSHBsynth_Hilbert_Mem[],
    Word16 genSHBsynth_state_lsyn_filt_shb_local_fx[]
);

void swb_tbe_reset_fx(
    Word32 mem_csfilt[],
    Word16 mem_genSHBexc_filt_down_shb[],
    Word16 state_lpc_syn[],
    Word16 syn_overlap[],
    Word16 state_syn_shbexc[],
    Word16 *tbe_demph,
    Word16 *tbe_premph,
    Word16 mem_stp_swb[],
    Word16 *gain_prec_swb
);

void InitSWBdecBuffer_fx(
    Decoder_State_fx *swb_dnc_fx                    /* i/o: SHB decoder structure                   */
);

void ResetSHBbuffer_Dec_fx(
    Decoder_State_fx *st_fx                       /* i/o: decoder state structure     */
);

Word16 squant_fx(                   /* o: index of the winning codeword   */
    const    Word16 x,               /* i: scalar value to quantize        */
    Word16* xq,             /* o: quantized value                 */
    const   Word16 cb[],            /* i: codebook                        */
    const   Word16 cbsize           /* i: codebook size                   */
);

void swb_tbe_dec_fx(
    Decoder_State_fx* st_fx,                /* i/o: decoder state structure */
    const   Word16 coder_type,                      /* i  : coding type */
    Word32* bwe_exc_extended,                       /* i  : bandwidth extended exciatation  2*Q_exc*/
    Word16  Q_exc,
    const   Word16 voice_factors[],                 /* i  : voicing factors                 */
    const   Word16 old_syn_12k8_16k[],              /* i  : low band synthesis*/
    Word16* White_exc16k,                           /* o  : shaped white excitation for the FB TBE */
    Word16* Q_white_exc,
    Word16* synth,                                  /* o  : SHB synthesis/final synthesis */
    Word16* Q_synth,
    Word16* pitch_buf
);


void tbe_write_bitstream_fx(
    Encoder_State_fx *st_fx                       /* i/o: encoder state structure                 */
);

void tbe_read_bitstream_fx(
    Decoder_State_fx *st_fx                       /* i/o: encoder state structure                 */
);

Word16 get_tbe_bits_fx(                          /* o  : TBE bit consumption per frame           */
    Word32 bitrate,                     /* i  : overall bitrate                         */
    Word16 bandwidth                    /* i  : bandwidht mode                          */
    ,Word16 rf_mode
);

void GenTransition_fx(
    const Word16 *input,                     /* i  : gain shape overlap buffer              */
    const Word16 *old_hb_synth,              /* i  : synthesized HB from previous frame     */
    Word16 length,                     /* i  : targeted length of transition signal   */
    Word16 *output,                    /* o  : synthesized transitions signal         */
    Word32 Hilbert_Mem[],              /* i/o: memory                                 */
    Word16 state_lsyn_filt_shb_local[],/* i/o: memory                                 */
    Word16 *syn_dm_phase,
    Word32   output_Fs,
    Word16 *up_mem,
    Word16   rf_flag
    , Word32 bitrate
);

void TBEreset_enc_fx(
    Encoder_State_fx *st_fx,          /* i/o: encoder state structure                 */
    Word16 bandwidth                  /* i  : bandwidth mode                          */
);

void TBEreset_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                 */
    Word16 bandwidth                  /* i  : bandwidth mode                          */
);

void fb_tbe_enc_fx(
    Encoder_State_fx *st,                /* i/o: encoder state structure                 */
    const Word16 new_input[],        /* i  : input speech at 48 kHz sample rate      */
    const Word16 fb_exc[],           /* i  : FB excitation from the SWB part         */
    Word16 Q_fb_exc
);

void fb_tbe_dec_fx(
    Decoder_State_fx *st,                /* i/o: encoder state structure                 */
    const Word16 fb_exc[],           /* i  : FB excitation from the SWB part         */
    Word16 Q_fb_exc,
    Word16 *hb_synth,           /* o  : high-band synthesis                     */
    Word16  hb_synth_exp
);


void elliptic_bpf_48k_generic_fx(
    const Word16 input_fx[],                /* i  : input signal                                */
    Word16 Q_input_fx,
    Word16 output_fx[],               /* o  : output signal                               */
    Word32 memory_fx[][4],            /* i/o: 4 arrays of 4 for memory                    */
    const Word16 full_band_bpf[][5]         /* i  : filter coefficients b0,b1,b2,a0,a1,a2  Q13  */
);

void synthesise_fb_high_band_fx(
    const Word16 excitation_in[],    /* i  : full band excitation                                */
    Word16 Q_fb_exc,
    Word16 output[],           /* o  : high band speech - 14.0 to 20 kHz                   */
    const Word32 fb_exc_energy,		 /* i  : full band excitation energy                         */
    const Word16 ratio,				 /* i  : energy ratio		                                */
    const Word16 L_frame,			 /* i  : ACELP frame length                                  */
    const Word16 bfi,                /* i  : fec flag			                                */
    Word16 *prev_fbbwe_ratio,  /* o  : previous frame energy for FEC                       */
    Word32 bpf_memory[][4],     /* i/o: memory for elliptic bpf 48k                         */
    Word16 Qout
);

void lsf_syn_mem_restore_fx(
    Encoder_State_fx *st_fx,                    /* o: state structure                                        */
    LPD_state* LPDmem,                /* o: LPD_state vewctor                                      */
    Word16 btilt_code_fx,             /* i:                                                       */
    Word32 gc_threshold_fx,           /* i:                                                       */
    Word16 *clip_var_bck_fx,          /* i:                                                       */
    Word16 next_force_sf_bck_fx,      /* i:                                                       */
    Word16 *lsp_new,                   /* o: LSP vector to quantize                                 */
    Word16 *lsf_new,                   /* o: quantized LSF vector                                   */
    Word16 *lsp_mid,                   /* o: mid-frame LSP vector                                   */
    Word16 clip_var,                   /* i: pitch clipping state var                               */
    Word16 *mem_AR,                    /* i: quantizer memory for AR model                          */
    Word16 *mem_MA,                    /* i: quantizer memory for MA model                          */
    Word16 *lsp_new_bck,               /* i: LSP vector to quantize- backup                         */
    Word16 *lsf_new_bck,               /* i: quantized LSF vector - backup                          */
    Word16 *lsp_mid_bck,               /* i: mid-frame LSP vector - backup                          */
    Word16 mCb1,                       /* i: counter for stationary frame after a transition frame  */
    Word32 *Bin_E,                     /* i: FFT Bin energy 128 *2 sets                             */
    Word32 *Bin_E_old,                 /* i: FFT Bin energy 128 sets                                */
    Word16 *mem_syn_bck,               /* i: synthesis filter memory                                */
    Word16 mem_w0_bck,                 /* i: memory of the weighting filter                         */
    Word16 streaklimit,                /* i:LSF quantizer                                           */
    Word16 pstreaklen                  /* i:LSF quantizer                                           */
);

void encod_nelp_fx(
    Encoder_State_fx *st_fx,                /* i/o: state structure */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 *speech_fx,                /* i  : input speech */
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq_fx[],                 /* i  : 12k8 Lp coefficient                               */
    Word16 *res_fx,                         /* o  : residual signal */
    Word16 *synth_fx,                       /* o  : core synthesis */
    Word16 *tmp_noise_fx,                   /* o  : long-term noise energy */
    Word16 *exc_fx,                         /* i/o: current non-enhanced excitation */
    Word16 *exc2_fx,                        /* i/o: current enhanced excitation */
    Word16 *pitch_buf_fx,                   /* o  : floating pitch values for each subframe */
    Word16* voice_factors_fx,               /* o  : voicing factors */
    Word16* bwe_exc_fx,                      /* o  : excitation for SWB TBE */
    Word16 Q_new,
    Word16 shift
);

void nelp_encoder_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state                      */
    Word16 *in_fx,                          /* i  : residual signal                    */
    Word16 *exc_fx,                         /* o  : NELP quantized excitation signal   */
    Word16 *qIn1
    ,Word16 reduce_gains
);

Word16 dequantize_uvg_fx(
    Word16 iG1,                         /* i: gain 1 index      */
    Word16 *iG2,                        /* i: gain 2 index      */
    Word16 *G,                          /* o: quantized gain    */
    Word16 bandwidth                    /* i: bandwidth     */
    ,Word16 do_scale
);

void quantize_uvg_fx(
    Word16 *G,
    Word16 *iG1,
    Word16 *iG2,
    Word16 *quantG,
    Word16 bandwidth
);


void generate_nelp_excitation_fx(
    Word16 *seed,
    Word16 *Gains,
    Word16 *output,
    Word16 gain_fac
);


Word16 find_rem(
    Word16 n,
    Word16 m,
    Word16 *r
);

Word32 find_remd(
    Word32 n,
    Word32 m,
    Word32 *r
);

void set_ppp_mode_fx(
    Encoder_State_fx *st_fx,            /* i/o: state structure */
    Word16 *coder_type       /* i : coding type      */
    ,const Word16 noisy_speech_HO,      /* i  : SC-VBR noisy speech HO flag */
    const Word16 clean_speech_HO,      /* i  : SC-VBR clean speech HO flag */
    const Word16 NB_speech_HO,         /* i  : SC-VBR NB speech HO flag */
    const Word16 localVAD,
    const Word16 localVAD_he,          /* i  : HE-SAD flag without hangover */
    Word16 *vad_flag
    ,Word16 T_op_fx[]             /* i : open loop pitch lag */
    ,Word16 sp_aud_decision1      /* i  : Speech Audio Decision */
);

void update_average_rate_fx(
    Encoder_State_fx *st_fx                         /* i/o: encoder state structure                */
);


void encod_ppp_fx(
    Encoder_State_fx * st_fx,        /* i/o: state structure */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 speech_fx[],         /* i : input speech Q_new*/
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq_fx[],                 /* i  : 12k8 Lp coefficient                               */
    Word16 *coder_type,        /* i/o : coding type */
    Word16 sharpFlag_fx,            /* i : formant sharpening flag */
    const Word16 T_op_fx[],           /* i : open loop pitch */
    const Word16 voicing_fx[],        /* i : voicing Q14*/
    Word16 *res_fx,             /* i/o: residual signal Q_new*/
    Word16 *synth_fx,           /* i/o: core synthesis Q-1*/
    Word16 *exc_fx,             /* i/o: current non-enhanced excitation Q_new*/
    Word16 *exc2_fx,            /*   o: current enhanced excitation Q0*/
    Word16 *pitch_buf_fx,       /*   o: floating pitch values for each subframe Q6*/
    Word16 *voice_factors,          /* o  : voicing factors */
    Word16 *bwe_exc,                /* o  : excitation for SWB TBE */
    Word16 Q_new,
    Word16 shift
);

void ppp_voiced_encoder_fx(
    Encoder_State_fx *st_fx,           /* i/o: state structure */
    Word16 *in_fx,          /* i : residual signal */
    Word16 *out_fx,          /* o : Quantized residual signal */
    Word16 delay_fx,          /* i : open loop pitch */
    Word16 *lpc1_fx,          /* i : prev frame de-emphasized LPC */
    Word16* lpc2_fx,          /* i : current frame de-emphasized LPC */
    Word16 *exc_fx,          /* i: previous frame quantized excitation */
    Word16 *pitch_fx,          /* o: floating pitch values for each subframe */
    Word16 vadsnr_fx,          /* i: current frame SNR Q7*/
    Word16 Qres
);

Word16 rint_new_fx(
    Word32 x                          /* i/o: Round to the nearest integer with mid point exception */
);

void decod_nelp_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder static memory */
    const Word16 coder_type,        /* i : coding type */
    Word16 *tmp_noise_fx,      /* o : long term temporary noise energy */
    Word16 *pitch_buf_fx,      /* o : floating pitch values for each subframe*/
    Word16 *exc_fx,          /* o : adapt. excitation exc */
    Word16 *exc2_fx,        /* o : adapt. excitation/total exc */
    Word16 *voice_factors,          /* o : Voice factor */
    Word16 *bwe_exc,
    Word16 *Q_exc,
    Word16 bfi            /* i : frame error rate */
    , Word16 *gain_buf        /*Q14*/
);

void nelp_decoder_fx(
    Decoder_State_fx *st,          /* i/o: decoder static memory */
    Word16 *exc_nelp,        /* o  : adapt. excitation/total exc */
    Word16 *exc,          /* o  : adapt. excitation exc */
    Word16 *Q_exc,
    Word16 bfi,            /* i  : frame error rate */
    const Word16 coder_type        /* i  : coding type */
    ,     Word16 *gain_buf        /*Q14*/
);

void core_switching_pre_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure     */
    const Word16 output_frame    /* i  : frame length                */
);

void fb_tbe_reset_synth_fx(
    Word32 fbbwe_hpf_mem_fx[][4],
    Word16 *prev_fbbwe_ratio_fx
);

void core_switching_post_dec_fx(
    Decoder_State_fx *st_fx,                    /* i/o: decoder state structure     */
    Word16 *synth,                 /* i/o: output synthesis Qsynth         */
    const Word16 output_frame,           /* i  : frame length                */
    const Word16 core_switching_flag,    /* i  : ACELP->HQ switching flag    */
    const Word16 coder_type,             /* i  : ACELP coder type            */
    Word16 *Qsynth
);
void acelp_core_switch_dec_bfi_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure  */
    Word16 synth_out[],          /* o  : synthesis Q_syn           */
    const Word16 coder_type            /* i  : coder type               */
);
void acelp_core_switch_dec_fx(
    Decoder_State_fx *st_fx,                /* decoder state structure          */
    Word16 *synth_subfr_out,   /* o synthesized ACELP subframe     Q_syn*/
    Word16 *tmp_synth_bwe,     /* o synthesized ACELP subframe BWE Q_syn*/
    const Word16 output_frame,       /* i  : input frame legth           */
    const Word16 core_switching_flag, /* i  : core switching flag         */
    Word16 *mem_synth,                  /* o  : synthesis to overlap        */
    Word16 *Q_syn
);

void core_switching_hq_prepare_dec_fx(
    Decoder_State_fx *st_fx,                /* i/o: encoder state structure */
    Word16 *num_bits,          /* i/o: bit budget update       */
    const Word16 output_frame        /* i  : output frame length     */
);
void fb_tbe_reset_enc_fx(

    Word32 elliptic_bpf_2_48k_mem_fx[][4],
    Word32 *prev_fb_energy_fx
);

void fb_bwe_reset_enc_fx(
    Word32 elliptic_bpf_2_48k_mem_fx[][4],
    Word32 *prev_energy_fbe_fb_fx
);


void core_switching_OLA_fx(
    Word16 *mem_over_hp,       /* i  : upsampling filter memory           Q0 */
    const Word16 last_L_frame,       /* i  : last L_frame lengthture               */
    const Word32   output_Fs,          /* i  : output sampling rate                  */
    Word16 *synth,             /* i/o: synthesized signal from HQ core    Q0 */
    Word16 *synth_subfr_out,   /* i  : synthesized signal from ACELP core Q0 */
    Word16 *synth_subfr_bwe,   /* i  : synthesized BWE from ACELP core    Q0 */
    const Word16 output_frame        /* i  : output frame length                   */
    ,Word16* Qsynth,
    Word16* Qsubfr
);

void acelp_core_switch_enc_fx(
    Encoder_State_fx *st_fx,                    /* i/o: encoder state structure             */
    LPD_state    *mem,
    const Word16 inp12k8[],              /* i  : input signal @12.8 kHz  Q0            */
    const Word16 inp16k[],               /* i  : input signal @16 kHz    Q0            */
    const Word16 T_op_orig[2],           /* i  : open-loop pitch values for quantiz. Q0*/
    const Word16 voicing[3],             /* i  : Open-loop pitch gains   Q15            */
    const Word16 A[NB_SUBFR16k*(M+1)],   /* i  : A(z) unquantized for the 4 subframes Q12*/
    Word16 *synth_subfr_bwe,        /* o  : CELP transition subframe  BWE       */
    Word16 shift,
    Word16 Q_new
);

void core_switching_hq_prepare_enc_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure */
    Word16 *num_bits,          /* i/o: bit budget update       */
    const Word16 input_frame,         /* i  : frame length            */
    Word32 *wtda_audio,        /* shall be q_audio + 15, audio allready scalled in wtda function  */
    const Word16 *audio
);

void core_switching_pre_enc_fx(
    Encoder_State_fx *st_fx,         /* i/o: encoder state structure           */
    LPD_state *mem,              /* i/o: encoder state structure           */
    const Word16 input_frame,        /* i  : frame length                      */
    const Word16 *old_inp_12k8,      /* i  : old input signal @12.8kHz         */
    const Word16 *old_inp_16k        /* i  : old input signal @16kHz           */
);

void core_switching_post_enc_fx(      /*done */
    Encoder_State_fx *st_fx,         /* i/o: encoder state structure             */
    const Word16 inp12k8[],      /* i  : input signal @12.8 kHz              */
    const Word16 inp16k[],       /* i  : input signal @16 kHz                */
    const Word16 T_op[2],        /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],     /* i  : Open-loop pitch gains               */
    const Word16 A[],            /* i  : unquant. LP filter coefs. (Q12)     */
    Word16 Qshift,
    Word16 Q_new,
    const Word16 Qsp,            /* i/o  : Q from acelp synthsis */
    Word16 *Qmus            /* i/o  : Q from mdct synthsis / Q of output synthesis  */
);

Word16 modify_Fs_intcub3m_sup_fx(       /* o  : length of output    */
    const Word16 sigIn[],          /* i  : signal to decimate with memory of 2 samples (indexes -2 & -1) */
    const Word16 lg,               /* i  : length of input (suppose that lg is such that lg_out is integer, ex multiple of 5 in case of 16kHz to 12.8 kHz) */
    const Word32   fin,              /* i  : frequency of input  */
    Word16 sigOut[],         /* o  : decimated signal    */
    const Word32   fout,             /* i  : frequency of output */
    Word16 *delayout         /* o  : delay of output */
);
void add_vec_fx(
    const Word16 x1[],   /* i  : Input vector 1                       */
    const Word16 Qx1,    /* i  : SCaling of iput 1                    */
    const Word16 x2[],   /* i  : Input vector 2                       */
    const Word16 Qx2,    /* i  : SCaling of iput 1                    */
    Word16 y[],    /* o  : Output vector that contains vector 1 + vector 2  */
    const Word16 Qy,     /* i  : SCaling of output 1                    */
    const Word16 N       /* i  : Vector lenght                                    */
);

/******************************* wrappers related functions **************/
void convert_fl_2_fx(const float *var_fl, Word16 *var_fx, Word16 count, Word16 Q_format);
void convert_fl_2_fx_32(const float *var_fl, Word32 *var_fx, Word16 count, Word16 Q_format);
void convert_fl_2_fx_32_MIN(const float *var_fl, Word32 *var_fx, Word16 count, Word16 Q_format, Word32 MIN_VAL);
void convert_sh_2_fx(const short *var_fl, Word16 *var_fx, Word16 count, Word16 Q_format);
void convert_dl_2_fx_32(double *var_fl, Word32 *var_fx, Word16 count, Word16 Q_format);
void convert_fx_2_fl(float *var_fl, const Word16 *var_fx, Word16 count, Word16 Q_format);
void convert_fx_2_sh(short *var_fl, const Word16 *var_fx, Word16 count, Word16 Q_format);
void convert_fx_2_dl_32(double *var_fl, const Word32 *var_fx, Word16 count, Word16 Q_format);
void convert_fx_2_fl_32(float *var_fl, const Word32 *var_fx, Word16 count, Word16 Q_format);
void convert_fx_2_fl_32_floor(float *var_fl, const Word32 *var_fx, Word16 count, Word16 Q_format);
/* void pre_proc_acelp_core_dec(Decoder_State *st,Decoder_State_fx *st_fx); */
/* void post_proc_acelp_core_dec(Decoder_State *st,Decoder_State_fx *st_fx); */

void convert_fl_2_fx_q(const float *x, Word16 *x_fx, Word16 len, Word16 *q);
/******************************* END OF wrappers related functions **************/

/******************************* END of temporary Hybride function related functions **************/

void Copy_Scale_sig32_16(
    const Word32 *src, /* i  : signal to scale                 Qx        */
    Word16 *dst, /* o  : scaled signal                   Qx        */
    Word16 len,  /* i  : size of x[]                     Q0        */
    Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx ?exp  */
);
Word32 Add_flt32_flt32(/* o: Result                             */
    Word32 a,          /* i: 1st Value                          */
    Word16 exp_a,      /* i: Exponent of 1st Value (Q of Value) */
    Word32 b,          /* i: 2nd Value                          */
    Word16 exp_b,      /* i: Exponent of 2nd Value (Q of Value) */
    Word16 *exp_out    /* o: Exponent of Result                 */
);
Word32 Mul_flt32_Q15(  /*  o: Result (Normalized)            */
    Word32 value,      /*  i: Pseudo_float Value             */
    Word16 *exp_v,     /*i/o: Exponent of Value (Q of Value) */
    Word16 frac        /*  i: Q15 value                      */
);
Word32 Div_flt32_flt32(/* o: Result (Normalized)                */
    Word32 a,          /* i: 1st Value                          */
    Word16 exp_a,      /* i: Exponent of 1st Value (Q of Value) */
    Word32 b,          /* i: 2nd Value                          */
    Word16 exp_b,      /* i: Exponent of 2nd Value (Q of Value) */
    Word16 *exp_out    /* o: Exponent of Result                 */
);
Word32 Calc_Energy_Autoscaled(/* o: Result (Energy)                  */
    const Word16 *signal,           /* i: Signal                           */
    Word16 signal_exp,        /* i: Exponent of Signal (Q of Signal) */
    Word16 len,               /* i: Frame Length                     */
    Word16 *energy_exp        /* o: Exponent of Energy (Q of Energy) */
);
Word16 Find_Max_Norm16(const Word16 *src, Word16 len);
Word16 Find_Max_Norm32(const Word32 *src, Word16 len);
Word32 Sqrt_Ratio32(Word32 L_val1, Word16 exp1, Word32 L_val2, Word16 exp2, Word16 *exp);
Word16 Invert16( /* result in Q'15 + 'exp' */
    Word16 val,
    Word16 *exp);

void swb_bwe_enc_hr_fx(
    Encoder_State_fx *st_fx,                 /* i/o: encoder state structure                */
    Word16 *new_input_fx,              /* i  : input signal                           */
    Word16 new_input_fx_exp,         /* i  : Exponent of input signal               */
    const Word16 input_frame,                /* i  : frame length                           */
    const Word16 coder_type,                 /* i  : coding type                            */
    const Word16 unbits                      /* i  : number of core unused bits             */
);

void wtda_fx(
    Word16 *new_audio,             /* i  : input audio   Q0 */
    Word16 *Q,                     /* i/o  : Q of Output Audio (use 15 for 'qout' for backward compatibility - NON SWB ENC HR FX MODULES) */
    Word32 *wtda_audio,            /* o  : windowed audio  Qout */
    Word16 *old_wtda,              /* i/o: windowed audio from previous frame Qout */
    Word16 *Qold_wtda,
    Word16 left_mode,
    Word16 right_mode,             /* window overlap of current frame (0: full, 2: none, or 3: half) */
    const Word16 L
);

void direct_transform_fx(
    const Word32 in32_fx[],
    Word32 out32_fx[],
    const Word16 is_transient,
    const Word16 L,
    Word16 *Q
);


/*-------------------------------------------------------------------*
* QC_VBR_FX functions
*
* ACELP core encoder
*--------------------------------------------------------------------*/

void r_fft_4_fx(Word16 * farray_ptr_fx, Word16 size, Word16 stage, Word16 isign);

DTFS_STRUCTURE_FX* DTFS_new_fx(
    void
);

void DTFS_copy_fx(
    DTFS_STRUCTURE_FX *Xout_fx,  /* o: DTFS structure  */
    DTFS_STRUCTURE_FX Xinp_fx    /* i: DTFS structure  */
);

DTFS_STRUCTURE_FX DTFS_sub_fx(
    DTFS_STRUCTURE_FX X1,  /* i: DTFS input 1 */
    DTFS_STRUCTURE_FX X2   /* i: DTFS input 2 */
);                      /* o: X1 - X2 */


void DTFS_to_fs_fx(
    const Word16 *x,          /* i : time domain signal               */
    Word16   N,              /* i : Length of input vector           */
    DTFS_STRUCTURE_FX *X_fx,      /* o : DTFS structure with a, b, lag    */
    Word16 Fs,              /* i : sampling rate                    */
    Word16 FR_flag,                     /* i: FR flag                       */
    Word16 *S_fx,
    Word16 *C_fx
);

void DTFS_fast_fs_inv_fx( DTFS_STRUCTURE_FX *This,Word16 *out_fx, Word16 N_fx, Word16 LOG2N);

void DTFS_car2pol_fx(
    DTFS_STRUCTURE_FX *X_fx       /* i/o : DTFS structure a, b, lag  */
);
void copy_phase_fx( DTFS_STRUCTURE_FX *X1_fx, DTFS_STRUCTURE_FX X2_fx, DTFS_STRUCTURE_FX *retX_fx);
Word32 getSpEngyFromResAmp_fx( DTFS_STRUCTURE_FX *X_fx,Word16 lband, Word16 hband,
                               const Word16 *curr_lpc, Word16 *sin_tab,
                               Word16 *cos_tab);

Word32 DTFS_setEngyHarm_fx(
    Word16 f1_fx,            /* i  : lower band freq of input to control energy   */
    Word16 f2_fx,            /* i  : upper band freq of input to control energy   */
    Word16 g1_fx,            /* i  : lower band freq of output to control energy  */
    Word16 g2_fx,            /* i  : upper band freq of output to control energy  */
    Word32 en2_fx,          /* i  : Target Energy to set the DTFS to             */
    Word16 Qen2_fx,          /* i  : Input Q format for en2             */
    Word16 *Qa_fx,          /* i  : Output Q format for x->a           */
    DTFS_STRUCTURE_FX *X_fx      /* i/o: DTFS to adjust the energy of                 */
);

void rshiftHarmBand_fx( DTFS_STRUCTURE_FX *X_fx,Word16 lband_fx, Word16 hband_fx, Word16 shift_fx);
void quant_target_fx( DTFS_STRUCTURE_FX *X_fx,const Word16 *curr_lpc, Word16 *w, Word16 *target,
                      Word16 *sin_tab, Word16 *cos_tab);

void GetSinCosTab_fx(Word16 L, Word16 *sinTab, Word16 *cosTab);

void DTFS_to_erb_fx(
    const DTFS_STRUCTURE_FX X_fx,      /* i : DTFS input       */
    Word16 *out_fx          /* o : ERB output       */
);

void DTFS_zeroPadd_fx(Word16 N_fx,DTFS_STRUCTURE_FX *X_fx);

Word32 DTFS_getEngy_fx( DTFS_STRUCTURE_FX *X_fx);

Word32 DTFS_getEngy_P2A_fx( DTFS_STRUCTURE_FX *X_fx);


Word32 DTFS_getEngy_band_fx(
    DTFS_STRUCTURE_FX X_fx,
    Word16 lband,
    Word16 hband
);

Word32 DTFS_getEngy_band_wb_fx(
    DTFS_STRUCTURE_FX X_fx,
    Word16 lband,
    Word16 hband
);

Word32 DTFS_freq_corr_fx(
    DTFS_STRUCTURE_FX X1_DTFS_fx,        /* i : X1 DTFS           */
    DTFS_STRUCTURE_FX X2_DTFS_fx,        /* i : X2 DTFS           */
    Word16 lband,            /* i : low cutoff        */
    Word16 hband,            /* i : high cutoff       */
    Word16 *Qout            /* o : Correlation Q format  */
);                                    /* o : Correlation       */

Word32 DTFS_setEngy_fx( DTFS_STRUCTURE_FX *X_DTFS_FX,Word32 en2_fx);

void DTFS_adjustLag_fx(
    DTFS_STRUCTURE_FX *X_DTFS_FX,          /* i/o : DTFS to adjust lag for */
    Word16 N_fx            /* i : Target lag               */
);

void DTFS_poleFilter_fx( DTFS_STRUCTURE_FX *X_fx,Word16 *LPC, Word16 N, Word16 *S_fx, Word16 *C_fx);
void DTFS_poleFilter_fx_9( DTFS_STRUCTURE_FX *X_fx, Word16 *pf_temp1, Word16 *pf_temp2, Word16 *pf_temp, Word16 *pf_n2_temp1);
void poleFilter_setup_fx(const Word16 *LPC, Word16 N, DTFS_STRUCTURE_FX X_fx, Word16 *S_fx, Word16 *C_fx, Word16 *pf_temp1, Word16 *pf_temp2, Word16 *pf_temp, Word16 *pf_n2_temp1);

void DTFS_zeroFilter_fx(
    DTFS_STRUCTURE_FX *X_fx,
    Word16 *LPC,
    Word16 N,
    Word16 *S_fx,
    Word16 *C_fx
);

Word16 DTFS_alignment_full_fx(
    DTFS_STRUCTURE_FX X1_DTFS_fx,      /* i : reference DTFS               */
    DTFS_STRUCTURE_FX X2_DTFS_fx,      /* i : DTFS to shift                */
    Word16 ph_offset_fx,               /* i : resolution                   */
    Word16 *S_fx,
    Word16 *C_fx
    , Word16 FR_flag
);

Word16 DTFS_alignment_extract_td_fx(Word16 *x1, Word16 *x2, Word16 lag);

Word16 DTFS_alignment_weight_fx(
    DTFS_STRUCTURE_FX *X_fx,
    DTFS_STRUCTURE_FX X2,
    Word16 Eshift,
    const Word16 *LPC1,
    const Word16 *LPC2,
    Word16 *S_fx,
    Word16 *C_fx,
    Word16 *pf_temp1,
    Word16 *pf_temp2,
    Word16 *pf_temp,
    Word16 *pf_n2
);


Word16 DTFS_alignment_fine_new_fx( DTFS_STRUCTURE_FX X1_fx, DTFS_STRUCTURE_FX X2_fx, Word16 *S_fx, Word16 *C_fx);


void DTFS_phaseShift_fx( DTFS_STRUCTURE_FX *X_fx,Word16 ph, Word16 Lag, Word16 *S_fx, Word16 *C_fx);
void Q2phaseShift_fx( DTFS_STRUCTURE_FX *X_fx,Word16 ph, Word16 Lag, Word16 *S_fx, Word16 *C_fx);


void DTFS_erb_inv_fx(
    Word16 *in_fx,          /* i : ERB inpt                      */
    Word16   *slot_fx,        /* i : ERB slots filled based on lag */
    Word16 *mfreq_fx,        /* i : erb frequence edges           */
    DTFS_STRUCTURE_FX *X_fx,        /* o : DTFS after erb-inv       */
    Word16 num_erb_fx        /* i : Number of ERB bands       */
);

void erb_add_fx(
    Word16 *curr_erb_fx,      /* i/o:  current ERB    */
    Word16   l_fx,          /* i  :  current lag    */
    const Word16 *prev_erb_fx,    /* i  :  previous ERB   */
    Word16   pl_fx,          /* i  :  previous lag   */
    const Word16   *index_fx,    /* i  :  ERB index      */
    Word16 num_erb_fx        /* i  :  number of ERBs */
);

void LPCPowSpect_fx(Word16 *freq, Word16 Nf, Word16 *LPC, Word16 Np,
                    Word16 *out);

void erb_slot_fx(
    Word16   lag_fx,        /* i : input lag          */
    Word16   *out_fx,        /* o : ERB slots          */
    Word16 *mfreq_fx,        /* i : ERB frequencies    */
    Word16 num_erb_fx        /* i : number of ERBs     */
);

Word16 DTFS_quant_cw_fx(
    DTFS_STRUCTURE_FX *X_fx,       /* i/o: DTFS unquant inp, quant out      */
    Word16  pl,            /* i  : Previous lag                     */
    const Word16 *curr_lpc_fx,        /* i  : LPC                              */
    Word16 *POWER_IDX,        /* o  : Power index                      */
    Word16 *AMP_IDX,             /* o  : Amplitude index                  */
    Word16 *lastLgainE_fx,        /* i/o: last frame lowband gain          */
    Word16 *lastHgainE_fx,        /* i/o: last frame highband gain         */
    Word16 *lasterbE_fx,           /* i/o: last frame ERB vector            */
    Word16 *sin_tab,
    Word16 *cos_tab
);


void DTFS_dequant_cw_fx(
    Word16   pl_fx,          /* i  : Previous lag                 */
    Word16   POWER_IDX_fx,      /* i  : POWER index                  */
    const Word16   *AMP_IDX_fx,     /* i  : Amp Shape index              */
    Word16 *lastLgainD_fx,      /* i/o: low band last gain           */
    Word16 *lastHgainD_fx,      /* i/o: high band last gain          */
    Word16 *lasterbD_fx,      /* i/o: last frame ERB vector        */
    DTFS_STRUCTURE_FX *X_fx,        /* o  : DTFS structure dequantized   */
    Word16 num_erb_fx
);

void DTFS_transform_fx(
    DTFS_STRUCTURE_FX X_fx,      /* i : Starting DTFS to use in WI       */
    DTFS_STRUCTURE_FX X2_fx,    /* i : Ending DTFS to use in WI         */
    const Word32 *phase_fx,      /* i : Phase contour             */
    Word16 *out_fx,          /* o : Output time domain waveform       */
    Word16   N,            /* i : Number of samples to generate     */
    Word16 FR_flag                  /* i : Flag to indicate called in FR context */
);

void DTFS_peaktoaverage_fx(
    DTFS_STRUCTURE_FX X_fx,          /* i : DTFS                           */
    Word32 *pos_fx,          /* o : positive peak to ave           */
    Word16 *Qpos,          /* o : positive peak to ave Q format  */
    Word32 *neg_fx,          /* o : negative peak to ave        */
    Word16 *Qneg          /* o : negative peak to ave Q format  */
);



Word16 ppp_extract_pitch_period_fx(
    const Word16 *in,                 /* i : input residual     */
    Word16 *out,                /* o : output residual    */
    Word16   l,                 /* i : lag                */
    Word16 *out_of_bound,        /* o : out of bound flag  */
    Word16 Qres

);

void Interpol_delay_fx(
    Word16 *out_fx,
    Word16 last_fx,
    Word16 current_fx,
    Word16 SubNum,
    const Word16* frac_fx
);


Word16 ppp_quarter_encoder_fx(
    DTFS_STRUCTURE_FX *CURRCW_Q_FX,      /* o  : Quantized (amp/phase) DTFS        */
    DTFS_STRUCTURE_FX *TARGETCW_FX,      /* o  : DTFS with quant phase but unquant Amp  */
    Word16   prevCW_lag,    /* i  : previous lag              */
    DTFS_STRUCTURE_FX CURRCW_NQ_FX,    /* i  : Unquantized DTFS            */
    const Word16 *curr_lpc_fx,    /* i  : LPCS                  */
    Word16 *lastLgainE_fx,    /* i/o: last low band gain            */
    Word16 *lastHgainE_fx,    /* i/o: last high band gain            */
    Word16 *lasterbE_fx,    /* i/o: last ERB vector              */
    DTFS_STRUCTURE_FX PREV_CW_E_FX,       /* i  : past DTFS                */
    Word16 *S_fx,                /* i  : sin table, Q15                         */
    Word16 *C_fx,                /* i  : cos table, Q15                         */
    Encoder_State_fx *st_fx
);


void WIsyn_fx(
    DTFS_STRUCTURE_FX PREVCW_FX,
    DTFS_STRUCTURE_FX *CURR_CW_DTFS_FX,
    const Word16 *curr_lpc_fx,
    Word16 *ph_offset_fx,
    Word16 *out_fx,
    Word16 N,
    Word16 FR_flag,                       /* i  : called for post-smoothing in FR         */
    Word16 *S_fx,
    Word16* C_fx,
    Word16 *pf_temp1,
    Word16 *pf_temp2,
    Word16 *pf_temp,
    Word16 *pf_n2
);

void deemph_lpc_fx(
    Word16 *p_Aq_curr_fx,           /* i : LP coefficients current frame                 */
    Word16 *p_Aq_old_fx,            /* i : LP coefficients previous frame                */
    Word16 *LPC_de_curr_fx,         /* o : De-emphasized LP coefficients current frame   */
    Word16 *LPC_de_old_fx,           /* o : De-emphasized LP coefficients previous frame  */
    Word16 deemph_old

);

Word16 erb_diff_search_fx(Word16 *prev_erb, const Word16 *curr_erb, Word16 *dif_erb,
                          Word16 *pow_spec, const Word16 *cb_fx,
                          Word16 cb_size, Word16 cb_dim, Word16 offset
                         );


void Acelp_dec_total_exc(
    Word16 *exc_fx,       /* i/o: adapt. excitation exc         */
    Word16 *exc2_fx,      /* i/o: adapt. excitation/total exc   */
    const Word16 gain_code16,   /* i  : Gain code Q0                  */
    const Word16 gain_pit_fx,   /* i  ; Pitch gain in Q14             */
    const Word16 i_subfr,       /* i  ; subfr                         */
    const Word16 *code_fx       /* i  : code in Q12                   */
);
void tbe_celp_exc(
    const Word16 L_frame_fx,          /* i  : Frame lenght */
    const Word16 i_subfr_fx,          /* i  : sub frame */
    const Word16 T0_fx,               /* i  : Integer pitch */
    const Word16 T0_frac_fx,          /* i  : Fractional part of the pitch */
    Word16 *error_fx,           /* i/o: Error */
    Word16 *bwe_exc_fx          /* i/o: bandwitdh extension signal */
);
Word16 tbe_celp_exc_offset(
    const Word16 T0_fx,               /* i  : Integer pitch */
    const Word16 T0_frac_fx,          /* i  : Fractional part of the pitch */
    const Word16 L_frame              /* i  : frame lenght */
);
void lsf_syn_mem_backup_fx(
    Encoder_State_fx *st_fx,           /* i: state structure                                       */
    LPD_state* LPDmem,                /* i: LPD state memory structure                            */
    Word16 *btilt_code,                /* i: tilt code                                             */
    Word32 *bgc_threshold,             /* i:                                                       */
    Word16 *clip_var_bck,              /* o:                                                       */
    Word16 *next_force_sf_bck,         /* o:                                                       */
    Word16 *lsp_new,                   /* i: LSP vector to quantize                                */
    Word16 *lsf_new,                   /* i: quantized LSF vector                                  */
    Word16 *lsp_mid,                   /* i: mid-frame LSP vector                                  */
    Word16 *clip_var,                  /* o: pitch clipping state var                              */
    Word16 *mem_AR,                    /* o: quantizer memory for AR model                         */
    Word16 *mem_MA,                    /* o: quantizer memory for AR model                         */
    Word16 *lsp_new_bck,               /* o: LSP vector to quantize- backup                        */
    Word16 *lsf_new_bck,               /* o: quantized LSF vector - backup                         */
    Word16 *lsp_mid_bck,               /* o: mid-frame LSP vector - backup                         */
    Word16 *mCb1,                      /* o: counter for stationary frame after a transition frame */
    Word32 *Bin_E,                     /* o: FFT Bin energy 128 *2 sets                            */
    Word32 *Bin_E_old,                 /* o: FFT Bin energy 128 sets                               */
    Word16 *mem_syn_bck,               /* o: synthesis filter memory                               */
    Word16 *mem_w0_bck,                /* o: memory of the weighting filter                        */
    Word16 *streaklimit,
    Word16 *pstreaklen
);

void decod_ppp_fx(
    Decoder_State_fx *st_fx,          /* i/o: state structure                */
    const Word16 Aq_fx[],          /* i  : 12k8 Lp coefficient              */
    Word16 *pitch_buf_fx,        /* i/o: fixed pitch values for each subframe    */
    Word16 *exc_fx,            /* i/o: current non-enhanced excitation        */
    Word16 *exc2_fx,          /* i/o: current enhanced excitation          */
    Word16 bfi              /* i  : bad frame indicator              */
    ,Word16 *gain_buf         /*Q14*/
    ,Word16  *voice_factors,               /* o  : voicing factors */
    Word16  *bwe_exc_fx                      /* o  : excitation for SWB TBE */
);

void ppp_quarter_decoder_fx(
    DTFS_STRUCTURE_FX *CURRCW_Q_DTFS_FX,    /* i/o: Current CW DTFS                        */
    Word16   prevCW_lag_fx,        /* i  : Previous lag                        */
    Word16 *lastLgainD_fx,      /* i/o: Last gain lowband Q11                    */
    Word16 *lastHgainD_fx,      /* i/o: Last gain highwband Q11                    */
    Word16 *lasterbD_fx,      /* i/o: Last ERB vector Q13                      */
    Word16 bfi,          /* i  : FER flag                          */
    Word16 *S_fx,                  /* i  : sine table, Q15                                           */
    Word16 *C_fx,                  /* i  : cosine table, Q15                                         */
    DTFS_STRUCTURE_FX PREV_CW_D_FX          /* i  : Previous DTFS                                             */
    ,Decoder_State_fx *st
);


void ppp_voiced_decoder_fx(
    Decoder_State_fx *st_fx,      /* i/o: state structure */
    Word16 *out_fx,        /* o : residual signal */
    const Word16 *lpc2_fx,    /* i : current frame LPC */
    Word16 *exc_fx,        /* i : previous frame excitation */
    Word16 *pitch,        /* o : floating pitch values for each subframe */
    Word16 bfi          /* i : Frame error rate */
);


void bandwidth_switching_detect_fx(
    Decoder_State_fx *st_fx                /* i/o: encoder state structure */
);

void bw_switching_pre_proc_fx(
    const Word16 *old_syn_12k8_16k_fx,     /* i  : ACELP core synthesis at 12.8kHz or 16kHz */
    Decoder_State_fx *st_fx                 /* i/o: decoder state structure     */
);

void updt_bw_switching_fx(
    Decoder_State_fx *st_fx,             /* i/o: decoder state structure                  */
    const Word16 *synth,                /* i  : float synthesis signal                   */
    const Word16 *inner_frame_tbl       /* i  : HQ inner_frame signallisation table      */
    ,const Word16 Qpost
);
void updt_dec_common_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure     */
    Word16 hq_core_type_fx,           /* i  : HQ core type                */
    const Word16 *synth               /* i  : decoded synthesis           */
);

void sort_32_fx(
    Word32 *r,    /* i/o: Vector to be sorted in place */
    const Word16 lo,    /* i  : Low limit of sorting range   */
    const Word16 up     /* I  : High limit of sorting range  */
);
void FEC_SinOnset_fx (
    Word16 *exc,        /* i/o : exc vector to modify                                           */
    Word16 puls_pos,    /* i   : last pulse position desired                                    */
    const Word16 T0,          /* i   : Pitch information of the 1 subfr                               */
    Word32 enr_q,       /* i   : energy provide by the encoder                                  */
    Word16 *Aq,         /* i   : A(z) filter   Q12                                              */
    const Word16 L_frame      /* i   : frame length                                                   */
    ,const Word16 Qold
);
Word16 FEC_enhACB_fx(
    const Word16 L_frame,                   /* i   : frame length                                                              */
    Word16 *exc_io,                   /* i/o : adaptive codebook memory                                                  */
    const Word16 new_pit,                   /* i   : decoded first frame pitch                                                 */
    const Word16 puls_pos,                  /* i   : decoder position of the last glottal pulses decoded in the previous frame */
    const Word16 bfi_pitch                  /* i   : Q6 pitch used for concealment                                             */
);


void fft3_fx(const Word16 [], Word16 [], const Word16);
void ifft3_fx(const Word16 [], Word16 [], const Word16);
void hq_ecu_fx(
    const Word16 *prevsynth,             /* i   : buffer of previously synthesized signal     */
    Word32 *ecu_rec,               /* o   : reconstructed frame in tda domain           */
    Word16 *time_offs,
    Word16 *X_sav,
    Word16 *Q_spec,               /* i/o  : Q value of stored spectrum                */
    Word16 *num_p,
    Word16 *plocs,
    Word32 *plocsi,                /* o   : Interpolated positions of the identified peaks (Q16) */
    const Word16 env_stab,
    Word16 *last_fec,
    const Word16 ph_ecu_HqVoicing,
    Word16 *ph_ecu_active,         /* i  : Phase ECU active flag                     */
    Word16 *gapsynth,
    const Word16 prev_bfi,               /* i   : indicating burst frame error                */
    const Word16 old_is_transient[2],    /* i   : flags indicating previous transient frames  */
    Word16 *mag_chg_1st,          /* i/o: per band magnitude modifier for transients*/
    Word16 *Xavg,                 /* i/o: Frequency group average gain to fade to   */
    Word16 *beta_mute,            /* o   : Factor for long-term mute                */
    const Word16 output_frame,           /* i   : frame length                                */
    Decoder_State_fx *st_fx                 /* i/o: decoder state structure                   */
);

void hq_timedomain_conc_fx(
    Word32 *ecu_rec,                    /* o  : reconstructed frame in tda domain         */
    Word16 *gapsynth,                   /* o  : Gap synthesis                             */
    const Word16 output_frame,                /* i  : frame length                              */
    const Word16 *prevsynth,                  /* i  : buffer of previously synthesized signal   */
    Decoder_State_fx *st_fx             /* i/o: decoder state structure                   */
);

Word16 ratio(const Word32, const Word32, Word16 *);
Word16 own_cos_fx( const Word16 x);
Word16 log2_div_fx(            /* o : Log2 of division, Q11 */
    const Word16 input_s,      /* i : Numerator         Q15 */
    const Word16 input_c       /* i : Denominator       Q15 */
);
Word16 bits2pulses_fx(
    const Word16 N,
    const Word16 bits,
    const Word16 strict_bits
);
Word16 pulses2bits_fx(
    const Word16 N,
    const Word16 P
);
Word16 get_angle_res_fx(
    const Word16 dim,
    const Word16 bits
);
Word16 get_pulse_fx(
    const Word16 q
);

/* IGFEnc.c */
void IGFEncApplyMono(const IGF_ENC_INSTANCE_HANDLE                   hInstance,          /**< in:     | instance handle of IGF Encoder                         */
                     const Word16                                    igfGridIdx,         /**< in: Q0  | IGF grid index                                         */
                     Encoder_State_fx                               *st,                 /**< in:     | Encoder state                                          */
                     Word32                                         *pMDCTSpectrum,      /**< in: Q31 | MDCT spectrum                                          */
                     Word16                                          MDCTSpectrum_e,     /**< in:     | exponent of MDCT spectrum                              */
                     Word32                                         *pPowerSpectrum,     /**< in: Q31 | MDCT^2 + MDST^2 spectrum, or estimate                  */
                     Word16                                          PowerSpectrum_e,    /**< in:     | exponent of pPowerSpectrum                             */
                     Word16                                          isTCX20,            /**< in: Q0  | flag indicating if the input is TCX20 or TCX10/2xTCX5  */
                     Word16                                          isTNSActive,        /**< in: Q0  | flag indicating if the TNS is active                   */
                     Word16                                          last_core_acelp     /**< in: Q0  | indictaor if last frame was acelp coded                */
                    );

void IGFEncConcatenateBitstream(const IGF_ENC_INSTANCE_HANDLE        hInstance,          /**< in:     | instance handle of IGF Encoder                 */
                                Word16                               bsBits,             /**< in: Q0  | number of IGF bits written to list of indices  */
                                Word16                              *next_ind,           /**< in/out: | pointer to actual bit indice                   */
                                Word16                              *nb_bits,            /**< in/out: | total number of bits already written           */
                                Indice_fx                           *ind_list_fx         /**< in:     | pointer to list of indices                     */
                               );

void IGFEncResetTCX10BitCounter(const IGF_ENC_INSTANCE_HANDLE        hInstance           /**< in:     | instance handle of IGF Encoder */
                               );

void IGFEncSetMode(const IGF_ENC_INSTANCE_HANDLE                     hInstance,          /**< in:     | instance handle of IGF Encoder */
                   const Word32                                      bitRate,            /**< in: Q0  | encoder bitrate                */
                   const Word16                                      mode                /**< in: Q0  | encoder bandwidth mode         */
                   , const Word16                                      rf_mode             /**< in:     | flag to signal the RF mode */

                  );

Word16 IGFEncWriteBitstream(                                                             /**< out:    | number of bits written per frame                                             */
    const IGF_ENC_INSTANCE_HANDLE            hInstance,          /**< in:     | instance handle of IGF Encoder                                               */
    Encoder_State_fx                        *st,                 /**< in:     | encoder state                                                                */
    Word16                                  *pBitOffset,         /**< in:     | ptr to bitOffset counter                                                     */
    const Word16                             igfGridIdx,         /**< in: Q0  | igf grid index see declaration of IGF_GRID_IDX for details                   */
    const Word16                             isIndepFlag         /**< in: Q0  | if 1 frame is independent, 0 = frame is coded with data from previous frame  */
);

Word16 IGFEncWriteConcatenatedBitstream(                                                 /**< out: Q0 | total number of bits written   */
    const IGF_ENC_INSTANCE_HANDLE hInstance,         /**< in:     | instance handle of IGF Encoder */
    void                         *st                 /**< in:     | encoder state                  */
);

/* IGFDec.c */
void IGFDecApplyMono(const IGF_DEC_INSTANCE_HANDLE                   hInstance,          /**< in:     | instance handle of IGF Decoder                       */
                     Word32                                         *spectrum,           /**< in/out: | MDCT spectrum                                        */
                     Word16                                         *spectrum_e,         /**< in/out: | exponent of spectrum                                 */
                     const Word16                                    igfGridIdx,         /**< in:     | in case of CELP->TCX switching, use 1.25 framelength */
                     Word16                                          bfi                 /**< in:     | frame loss == 1, frame good == 0                     */
                    );

void IGFDecCopyLPCFlatSpectrum(const IGF_DEC_INSTANCE_HANDLE         hInstance,          /**< in:     | instance handle of IGF Decoder     */
                               const Word32                         *pSpectrumFlat,      /**< in: Q31 | LPC flattend spectrum from TCX dec */
                               const Word16                          pSpectrumFlat_exp,  /**< in:     | exponent of pSpectrumFlat          */
                               const Word16                          igfGridIdx          /**< in: Q0  | IGF grid index                     */
                              );

void IGFDecReadData(const IGF_DEC_INSTANCE_HANDLE                    hInstance,          /**< in:     | instance handle of IGF Deccoder                      */
                    Decoder_State_fx                                *st,                 /**< in:     | decoder state                                        */
                    const Word16                                     igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength */
                    const Word16                                     isIndepFrame        /**< in: Q0  | if 1: arith dec force reset, if 0: no reset          */
                   );

void IGFDecReadLevel(  const IGF_DEC_INSTANCE_HANDLE                 hInstance,          /**< in:     | instance handle of IGF Deccoder                                  */
                       Decoder_State_fx                             *st,                 /**< in:     | decoder state                                                    */
                       const Word16                                  igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength             */
                       const Word16                                  isIndepFrame        /**< in: Q0  | if 1: arith dec force reset, if 0: no reset                      */
                    );

void IGFDecRestoreTCX10SubFrameData(const IGF_DEC_INSTANCE_HANDLE    hInstance,          /**< in:     | instance handle of IGF Decoder */
                                    const Word16                     subFrameIdx         /**< in: Q0  | index of subframe              */
                                   );

void IGFDecSetMode(const IGF_DEC_INSTANCE_HANDLE                     hInstance,          /**< in:     | instance handle of IGF Decoder */
                   const Word32                                      bitRate,            /**< in: Q0  | bitrate                        */
                   const Word16                                      mode,               /**< in: Q0  | bandwidth mode                 */
                   const Word16                                      defaultStartLine,   /**< in: Q0  | default start subband index    */
                   const Word16                                      defaultStopLine     /**< in: Q0  | default stop subband index     */
                   , const Word16                                      rf_mode             /**< in:     | flag to signal the RF mode */
                  );

void IGFDecStoreTCX10SubFrameData(const IGF_DEC_INSTANCE_HANDLE      hInstance,          /**< in:     | instance handle of IGF Decoder */
                                  const Word16                       subFrameIdx         /**< in: Q0  | index of subframe              */
                                 );

void IGFDecUpdateInfo(const IGF_DEC_INSTANCE_HANDLE                  hInstance,          /**< in:     | instance handle of IGF Decoder */
                      const Word16                                   igfGridIdx          /**< in:     | IGF grid index                 */
                     );

/* iisBaseLib.c */
void IGFCommonFuncsCalcSfbEnergyPowerSpec(const Word16               startSfb,           /**< in: Q0  | start sfb index                                          */
        const Word16               stopSfb,            /**< in: Q0  | stop  sfb index                                          */
        const Word16              *swb_offset,         /**< in: Q0  | IGF swb offset table                                     */
        Word32                    *pPowerSpectrum,     /**< in: Q31 | power spectrum                                           */
        Word16                    *pPowerSpectrum_exp, /**< in:     | Exponent of PowerSpectrum                                */
        Word32                    *sfbEnergy,          /**< out:Q31 | SFB energies , will be initialized inside this function  */
        Word16                    *sfbEnergy_exp       /**< out:    | Exponent of PowerSpectrum                                */
                                         );

Word16 IGFCommonFuncsIGFConfiguration(                                                   /**< out:    | error value: 0 -> error, 1 -> ok   */
    Word32                         bitRate,            /**< in: Q0  | bitrate in bs e.g. 9600 for 9.6kbs */
    Word16                         mode,               /**< in: Q0  | bandwidth mode                     */
    H_IGF_INFO                     hIGFInfo            /**< out:    | IGF info handle                    */
    ,Word16                         rf_mode             /**< in: flag to signal the RF mode */
);

Word16 IGFCommonFuncsIGFGetCFTables(                                                     /**< out:    | error value: 0 -> error, 1 -> ok     */
    Word32                           bitRate,            /**< in: Q0  | bitrate in bs e.g. 9600 for 9.6kbs   */
    Word16                           mode,               /**< in: Q0  | bandwidth mode                       */
    Word16                           rf_mode,            /**< in:     | flag to signal the RF mode */
    const Word16                   **cf_se00,            /**< out:    | CF table for t == 0 and f == 0       */
    const Word16                   **cf_se01,            /**< out:    | CF table for t == 0 and f == 1       */
    Word16                          *cf_off_se01,        /**< out:    | offset for CF table above            */
    const Word16                   **cf_se02,            /**< out:    | CF tables for t == 0 and f >= 2      */
    const Word16                   **cf_off_se02,        /**< out:    | offsets for CF tables above          */
    const Word16                   **cf_se10,            /**< out:    | CF table for t == 1 and f == 0       */
    Word16                          *cf_off_se10,        /**< out:    | offset for CF table above            */
    const Word16                   **cf_se11,            /**< out:    | CF tables for t == 1 and f >= 1      */
    const Word16                   **cf_off_se11         /**< out:    | offsets for CF tables above          */
);

void IGFCommonFuncsMDCTSquareSpec(const Word16                       sqrtBgn,            /**< in: Q0  | start MDCT subband index       */
                                  const Word16                       sqrtEnd,            /**< in: Q0  | stop  MDCT subband index       */
                                  const Word32                      *mdctSpec,           /**< in: Q31 | MDCT spectrum to square        */
                                  const Word16                       mdctSpec_e,         /**< in:     | exponent of mdctSpectrum       */
                                  Word32                            *mdctSquareSpec,     /**< out:Q31 | MDCT square spectrum           */
                                  Word16                            *mdctSquareSpec_e,   /**< out:    | exponent of mdctSquareSpec     */
                                  Word16                             indexOffset         /**< in: Q0  | index offset                   */
                                 );

void IGFCommonFuncsWriteSerialBit(void                              *st,                 /**< in:     | encoder/decoder state structure  */
                                  Word16                            *pBitOffset,         /**< out: Q0 | bit offset                       */
                                  Word16                             bit                 /**< in: Q0  | value of bit                     */
                                 );

/* IGFSCFEncoder.c */
void IGFSCFEncoderOpen(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData,         /* i/o: handle to public data */
    Word16                                  scfCountLongBlock,   /* i: number of SCFs for a long block */
    Word32                                  bitRate,             /* i: bitrate in bps */
    Word16                                  mode                 /* i: operating mode */
    , Word16                                  rf_mode             /**< in: flag to signal the RF mode */
);

void IGFSCFEncoderReset(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
);

Word16 IGFSCFEncoderEncode(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData,      /* i/o: handle to public data */
    Encoder_State_fx                       *st,               /* i/o: pointer to encoder state */
    Word16                                  bitCount,         /* i: offset to the first bit in bitbuffer which should be written by the raw AC functions */
    Word16                                 *sfe,              /* i: pointer to an array which contains the quantized SCF energies to be encoded */
    Word16                                  indepFlag,        /* i: 1 if the block is an independent block, 0 otherwise */
    Word16                                  doRealEncoding    /* i: whether the real encoding is needed, otherwise only the number of bits is used */
);

void IGFSCFEncoderSaveContextState(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
);

void IGFSCFEncoderRestoreContextState(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
);

/* IGFSCFDecoder.c */
void IGFSCFDecoderOpen(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData,         /* i/o: handle to public data */
    Word16                                  scfCountLongBlock,   /* i: number of SCFs for a long block */
    Word32                                  bitRate,             /* i: bitrate in bps */
    Word16                                  mode                 /* i: operating mode */
    ,Word16                                  rf_mode
);

void IGFSCFDecoderReset(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
);

void IGFSCFDecoderDecode(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData,      /* i/o: handle to public data */
    Decoder_State_fx                       *st,               /* i/o: pointer to decoder state */
    Word16                                 *sfe,              /* o: pointer to an array which will contain the decoded quantized SCFs */
    Word16                                  indepFlag         /* i: 1 if the block is an independent block, 0 otherwise */
);

Word32 SFM_Cal(Word32 magn[], Word16 n);


void Unified_weighting_fx(
    Word32 Bin_Ener_128_fx[], /* i  : FFT Bin energy 128 bins in two sets    Q_ener */
    Word16 Q_ener,
    const Word16 lsf_fx[],          /* i  : LSF vector                             x2.56 */
    Word16 w_fx[],            /* o  : LP weighting filter (numerator)         Q8 */
    const Word16 narrowBand,     /* i  : flag for Narrowband                     */
    const Word16 unvoiced,       /* i  : flag for Unvoiced frame                 */
    const Word32 sr_core,        /* i  : sampling rate of core-coder             */
    const Word16   order           /* i  : LP order                                */
);

void lsf_dec_bfi(
    Word16*lsf,               /*!< o  : 14Q1*1.28     quantized ISFs                            */
    const Word16*lsfold,              /*!< i  : 14Q1*1.28     past quantized ISF                        */
    Word16*lsf_adaptive_mean,   /*!< i  : 14Q1*1.28     ISF adaptive mean, updated when BFI==0    */
    const Word16 lsfBase[],   /* i  : base for differential lsf coding        */
    Word16*mem_MA,              /*!< i/o: 14Q1*1.28     quantizer memory for MA model             */
    Word16*mem_AR,              /*!< i/o: 14Q1*1.28     quantizer memory for MA model             */
    Word16 stab_fac,            /*!< i  :               ISF stability factor (shifted right by 1) */
    const Word16 last_coder_type,            /*!< i  :               coding type in last good received fr.     */
    Word16 L_frame,
    const Word16 last_good,            /*!< i  :               last good received frame                  */
    const Word16 nbLostCmpt,              /*!< i  :               counter of consecutive bad frames         */
    Word8  plcBackgroundNoiseUpdated,
    Word16 *lsf_q_cng,        /* o  : quantized ISFs for background noise     (14Q1*1.28) */
    Word16 *lsf_cng,
    Word16 *old_lsf_q_cng,   /* o  : old quantized ISFs for background noise */
    const Word16 Last_GSC_pit_band_idx,
    const Word16 Opt_AMR_WB,                 /* i  : IO flag                                */
    const Word16 tcxonly
);


void lpc_quantization(
    Encoder_State_fx * st,
    const Word16 core,
    const Word16 lpcQuantization,
    const Word16 lsfold_q[],
    const Word16 lsp[],
    const Word16 lspmid[],
    Word16 lsp_q[],
    Word16 lsf_q[], /* 14Q1*1.28 */
    Word16 lspmid_q[],
    Word16 lspq_ind[],
    Word16 clip_var[],
    Word16 mem_MA[],
    Word16 mem_AR[],
    const Word8  narrowBand,
    const Word16 coder_type,
    const Word8  acelp_midLpc,
    Word16 param_lpc[],
    Word16 nbits_lpc[],
    Word16 * bits_param_lpc,
    Word16 *no_param_lpc,
    Word16 *seed_acelp,
    Word32 * Bin_Ener_fx,
    Word32 * Bin_Ener_old_fx,
    const Word16 Q_ener
);



/* enc_util.c */
void Scale_sig(
    Word16 x[],  /* i/o: signal to scale                 Qx        */
    const Word16 lg,   /* i  : size of x[]                     Q0        */
    const Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx xx exp  */
);
void E_UTIL_f_preemph(Word16 *signal, const Word16 mu, const Word16 lg, Word16 *mem);
void E_UTIL_f_preemph2(Word16 shift, Word16 *signal, const Word16 mu, const Word16 lg, Word16 *mem);
Word16 E_UTIL_f_preemph3(Word16 *signal, const Word16 mu, const Word16 lg, Word16 *mem, Word16 bits);
void deemph_fx(Word16 * signal, const Word16 mu, const Word16 L, Word16 * mem);
void E_UTIL_deemph2(Word16 shift, Word16 *x, const Word16 mu, const Word16 L, Word16 *mem);
void E_UTIL_synthesis(const Word16 shift, const Word16 a[], const Word16 x[], Word16 y[],
                      const Word16 lg, Word16 mem[], const Word16 update, const Word16 m);
void E_UTIL_f_convolve(const Word16 x[], const Word16 h[], Word16 y[], const Word16 size);

Word16 E_ACELP_hh_corr(Word16 *x, Word16 *y, Word16 L_subfr, Word16 bits);
Word16 E_ACELP_toeplitz_mul(const Word16 R[], const Word16 c[], Word16 d[], const Word16 L_subfr, const Word16 highrate);

void E_ACELP_weighted_code(
    const Word16 code[], /* i: code             */
    const Word16 H[],    /* i: impulse response */
    Word16 Q,            /* i: Q format of H    */
    Word16 y[]           /* o: weighted code    */
);

void E_ACELP_conv(
    const Word16 xn2[], /* i */
    const Word16 h2[],  /* i */
    Word16 cn2[]        /* o */
);

void E_ACELP_build_code(
    Word16 nb_pulse,       /* i */
    const Word16 codvec[], /* i */
    const Word16 sign[],   /* i */
    Word16 code[],         /* o */
    Word16 ind[]           /* o */
);

void E_ACELP_setup_pulse_search_pos(
    const PulseConfig *config, /* i: pulse configuration    */
    Word16 k,                  /* i: interation number      */
    UWord8 ipos[]              /* o: pulse search positions */
);

void find_wsp(
    const Word16 Az[],
    const Word16 speech[],
    Word16 wsp[],
    Word16 *mem_wsp,
    const Word16 preemph_fac,
    const Word16 L_frame,
    const Word16 lookahead,
    const Word16 L_subfr,
    Word16 Aw[],                /* o  : weighted A(z) filter coefficients     */
    const Word16 gamma,         /* i  : weighting factor                      */
    const Word16 nb_subfr       /* i  : number of subframes                   */
);

void init_sig_buffers( Encoder_State_fx *st, const Word16 L_frame_old, const Word16 L_subfr );

void E_UTIL_cb_shape(
    const Word16 preemphFlag,          /* i  : flag for pre-emphasis                           */
    const Word16 pitchFlag,            /* i  : flag for pitch sharpening                       */
    const Word16 scramblingFlag,       /* i  : flag for phase scrambling                       */
    const Word16 formant_enh,          /* i  : use formant enhancement                     Q15 */
    const Word16 formant_tilt,         /* i  : use tilt of formant enhancement              Q0 */
    const Word16 g1,                   /* i  : formant sharpening numerator weighting          */
    const Word16 g2,                   /* i  : formant sharpening denominator weighting        */
    const Word16 *Aq,                  /* i  : quantized LPC coefficients                 3Q12 */
    Word16 *code,                /* i/o:                                            <14b */
    const Word16 tilt_code,            /* i  : tilt factor                                 Q15 */
    const Word16 pitch                 /* i  : pointer to current subframe fractional pitch    */
);

void E_UTIL_voice_factor( Word16 *exc,        /* i  : pointer to the excitation frame   Q_new */
                          Word16 i_subfr,       /* i  : subframe index                          */
                          Word16 *code,         /* i  : innovative codebook                  Q9 */
                          Word16 gain_pit,      /* i  : adaptive codebook gain             1Q14 */
                          Word32 gain_code,     /* i  : innovative cb. gain               15Q16 */
                          Word16 *voice_fac,    /* o  : subframe voicing estimation         Q15 */
                          Word16 *tilt_code,    /* o  : tilt factor                         Q15 */
                          Word16 L_subfr,       /* i  : subframe length                         */
                          Word16 flag_tilt,     /* i  : Flag for triggering new voice factor tilt*/
                          Word16 Q_new,         /* i  : excitation buffer format                 */
                          Word16 shift          /* i  : scaling to get 12bit                     */
                        );

Word16 E_UTIL_enhancer(
    Word16 voice_fac,           /* i  : subframe voicing estimation         Q15 */
    Word16 stab_fac,            /* i  : LP filter stability measure         Q15 */
    Word32 gain_code,           /* i  : innovative cb. gain               15Q16 */
    Word16 gain_inov,           /* i  : gain of the unscaled innovation     Q12 */
    Word32 *gc_threshold,       /* i/o: gain code threshold               15Q16 */
    Word16 *code,               /* i/o: innovation(in: Q9)             code_exp */
    Word16 *exc2,               /* i/o: adapt. excitation/total exc.        Q15 */
    Word16 gain_pit,            /* i  : Quantized pitch gain               1Q14 */
    Word32 *prev_gain_code,     /* i/o: previous codebook gain            15Q16 */
    Word16 prev_gain_pit[],     /* i/o: previous pitch gain, size=6        1Q14 */
    Word16 *prev_state,         /* i/o: Phase dispersion algorithm memory    Q0 */
    Word16 coder_type,          /* i  : coder type                              */
    Word16 cdk_index,           /* i  :                                         */
    Word16 L_subfr,             /* i  :                                         */
    Word16 L_frame,             /* i  : frame size                              */
    Word16 Q_new
);

Word32 E_LPC_schur(Word32 r[], Word16 reflCoeff[], Word32 epsP[], const Word16 m);
Word16 E_LPC_lev_dur_stab(const Word16 Rh[], const Word16 Rl[], Word16 A[],
                          Word32 epsP[], const Word16 order, Word16 *parcorr, Word16 k_max
                         );
Word16 E_LPC_lev_dur(const Word16 Rh[], const Word16 Rl[], Word16 A[],
                     Word32 epsP[], const Word16 order, Word16 *parcorr
                    );

void E_LPC_a_weight_inv(const Word16 *a, Word16 *ap, const Word16 gamma, const Word16 m);
void E_LPC_a_add_tilt(const Word16 *a, Word16 *ap, Word16 gamma, Word16 m);
void E_LPC_a_lsf_isf_conversion(Word16 *lpcCoeffs,   Word16 *lsf, const Word16 *old_lsf, Word16 lpcOrder, Word8  lpcRep);
void E_LPC_a_lsp_conversion(
    const Word16 *a,       /* input : LP filter coefficients                     */
    Word16 *lsp,       /* output: Line spectral pairs (in the cosine domain) */
    const Word16 *old_lsp, /* input : LSP vector from past frame                 */
    const Word16 m     /* input : LPC order                                  */
);
void E_LPC_lsp_lsf_conversion(
    const Word16 lsp[],    /* input: lsp[m] (range: -1<=val<1)                  */
    Word16 lsf[],       /* output : lsf[m] normalized (range: 0<=val<=6400)  */
    const Word16 m     /* input : LPC order                                 */
);
void E_LPC_lsf_lsp_conversion(
    const Word16 lsf[],    /* input : lsf[m] normalized (range: 0<=val<=6400)  */
    Word16 lsp[],       /* output: lsp[m] (range: -1<=val<1)                */
    const Word16 m     /* input : LPC order                                */
);

void E_LPC_f_lsp_a_conversion(const Word16 *lsp, Word16 *a, const Word16 m);
void E_LPC_a_isp_conversion(
    const Word16 *a,    /* input : LP filter coefficients                     */
    Word16 *isp,      /* output: Line spectral pairs (in the cosine domain) */
    const Word16 *old_isp,/* input : LSP vector from past frame                 */
    const Word16 m    /* input : LPC order                                  */
);
void E_LPC_isp_isf_conversion(const Word16 isp[], Word16 isf[], const Word16 m);
void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m);
void E_LPC_f_isp_a_conversion(const Word16 *isp, Word16 *a, const Word16 m);
void E_LPC_int_lpc_tcx(const Word16 lsp_old[],    /* input : LSPs from past frame (1Q14)             */
                       const Word16 lsp_new[],    /* input : LSPs from present frame (1Q14)          */
                       Word16 a[]       /* output: interpolated LP coefficients (4Q11)     */
                      );

/* enc_gain.c */

Word16 E_GAIN_closed_loop_search(Word16 exc[],
                                 Word16 xn[], Word16 h[],
                                 Word16 t0_min, Word16 t0_min_frac, Word16 t0_max, Word16 t0_max_frac, Word16 t0_min_max_res, Word16 *pit_frac, Word16 *pit_res, Word16 pit_res_max,
                                 Word16 i_subfr, Word16 pit_min, Word16 pit_fr2, Word16 pit_fr1, Word16 L_subfr);
void E_GAIN_norm_corr(Word16 exc[], Word16 xn[], Word16 h[],
                      Word16 t_min, Word16 t_max, Word16 corr_norm[], Word16 L_subfr);

/* enc_acelp.c */
void    E_ACELP_xy2_corr(Word16 xn[], Word16 y1[], Word16 y2[], ACELP_CbkCorr *g_corr, Word16 L_subfr, Word16 exp_xn);
Word16  E_ACELP_xy1_corr(Word16 xn[], Word16 y1[], ACELP_CbkCorr *g_corr, Word16 norm_flag, Word16 L_subfr, Word16 exp_xn);
void E_ACELP_codebook_target_update(Word16 *x, Word16 *x2, Word16 *y, Word16 gain, Word16 L_subfr);
void E_ACELP_h_vec_corr1(Word16 h[], Word16 vec[], UWord8 track, Word16 sign[], Word16 (*rrixix)[16], Word16 cor[], Word16 dn2_pos[], Word16 nb_pulse);
void E_ACELP_h_vec_corr2(Word16 h[], Word16 vec[], UWord8 track, Word16 sign[], Word16 (*rrixix)[16], Word16 cor[]);
void    E_ACELP_4tsearch(Word16 dn[], const Word16 cn[], const Word16 H[], Word16 code[], const PulseConfig *config, Word16 ind[], Word16 y[]);
void    E_ACELP_4tsearchx(Word16 dn[], const Word16 cn[], Word16 Rw[], Word16 code[], const PulseConfig *config, Word16 ind[]);
void    E_ACELP_pulsesign(const Word16 cn[], Word16 dn[], Word16 dn2[], Word16 sign[], Word16 vec[], const Word16 alp, Word16 const sign_val, const Word16 L_subfr);

Word16  E_ACELP_indexing(const Word16 code[], const PulseConfig *config, Word16 num_tracks, Word16 prm[]);
void    E_ACELP_findcandidates(Word16 dn2[], Word16 dn2_pos[], Word16 pos_max[]);
void    E_ACELP_vec_neg(Word16 h[], Word16 h_inv[], Word16 L_subfr);
void    E_ACELP_corrmatrix(Word16 h[], Word16 sign[], Word16 vec[], Word16 rrixix[4][16], Word16 rrixiy[4][256]);
void    E_ACELP_codearithp(const Word16 v[], UWord32 *n, UWord32 *ps, Word16 *p);

void find_targets_fx(
    const Word16 *speech,     /* i  : pointer to the speech frame                      Q_new-1*/
    const Word16 *mem_syn,    /* i  : memory of the synthesis filter                   Q_new-1*/
    const Word16 i_subfr,     /* i  : subframe index                                   */
    Word16 *mem_w0,     /* i/o: weighting filter denominator memory              Q_new-1*/
    const Word16 *p_Aq,       /* i  : interpolated quantized A(z) filter               Q12*/
    const Word16 *res,        /* i  : residual signal                                  Q_new*/
    const Word16 L_subfr,     /* i  : length of vectors for gain quantization          */
    const Word16 *Ap,         /* i  : unquantized A(z) filter with bandwidth expansion Q12*/
    Word16 tilt_fac,    /* i  : tilt factor                                  Q15 */
    Word16 *xn,         /* o  : Close-loop Pitch search target vector            Q_new-1*/
    Word16 *cn          /* o  : target vector in residual domain                 Q_new*/
    ,Word16 *h1                                            /* Q14 ?*/
);

void E_ACELP_adaptive_codebook(
    Word16 *exc,          /* i  : pointer to the excitation frame                  */
    Word16 T0,            /* i  : integer pitch lag                                */
    Word16 T0_frac,       /* i  : fraction of lag                                  */
    Word16 T0_res,        /* i  : pitch resolution                                 */
    Word16 T0_res_max,    /* i  : maximum pitch resolution                         */
    Word16 mode,          /* i  : filtering mode (0: no, 1: yes, 2: adaptive)      */
    Word16 i_subfr,       /* i  : subframe index                                   */
    Word16 L_subfr,       /* i  : subframe length                                  */
    Word16 L_frame,       /* i  : subframe length                                  */
    Word16 *h1,         /* i  : impulse response of weighted synthesis filter    */
    Word16 clip_gain,
    Word16 *xn,         /* i  : Close-loop Pitch search target vector            */
    Word16 *y1,         /* o  : zero-memory filtered adaptive excitation         */
    ACELP_CbkCorr *g_corr,     /* o  : ACELP correlation values                 */
    Word16 **pt_indice,   /* i/o: quantization indices pointer                     */
    Word16 *pitch_gain, /* o  : adaptive codebook gain                           */
    Word16 exp_xn
    ,Word16 rf_mode
    ,Word16 use_prev_sf_pit_gain
    ,Word16* lp_select
);

void E_ACELP_innovative_codebook(
    Word16 *exc,            /* i  : pointer to the excitation frame                  */
    Word16 T0,              /* i  : integer pitch lag                                */
    Word16 T0_frac,         /* i  : fraction of lag                                  */
    Word16 T0_res,          /* i  : pitch resolution                                 */
    Word16 gain_pit,        /* i  : adaptive codebook gain                           */
    Word16 tilt_code,       /* i  : tilt factor                                      */
    Word16 mode,            /* i  : innovative codebook mode                         */
    Word16 formant_enh,     /* i  : use formant enhancement                      Q15 */
    Word16 formant_tilt,    /* i  : use tilt of formant enhancement                  */
    const Word16 formant_enh_num, /* i  : formant sharpening numerator weighting           */
    const Word16 foramnt_enh_den, /* i  : formant sharpening denominator weighting         */
    Word16 pitch_sharpening,/* i  : use pitch sharpening                             */
    Word16 pre_emphasis,
    Word16 phase_scrambling,
    Word16 i_subfr,         /* i  : subframe index                                   */
    const Word16 *Aq,       /* i  : quantized LPC coefficients                       */
    Word16 *h1,             /* i  : impulse response of weighted synthesis filter    */
    Word16 *xn,             /* i  : Close-loop Pitch search target vector            */
    Word16 *cn,             /* i  : Innovative codebook search target vector         */
    Word16 *y1,             /* i  : zero-memory filtered adaptive excitation         */
    Word16 *y2,             /* o  : zero-memory filtered algebraic excitation        */
    Word8 acelpautoc,       /* i  : autocorrelation mode enabled                     */
    Word16 **pt_indice,     /* i/o: quantization indices pointer                     */
    Word16 *code,           /* o  : innovative codebook Q9                           */
    Word16 shift            /* i  : Scaling to get 12 bits                           */
);

Word16 E_ACELP_code43bit(const Word16 code[], UWord32 *ps, Word16 *p, UWord16 idxs[]);
void fcb_pulse_track_joint(UWord16 *idxs, Word16 wordcnt, UWord32 *index_n, Word16 *pulse_num, Word16 track_num);

void encode_acelp_gains(
    Word16 *code, /* Q9 */
    Word16 gains_mode,
    Word16 mean_ener_code,
    Word16 clip_gain,
    ACELP_CbkCorr *g_corr,
    Word16 *gain_pit,  /* Q14 */
    Word32 *gain_code, /* 15Q16 */
    Word16 **pt_indice,
    Word32 *past_gcode, /* 15Q16 */
    Word16 *gain_inov,  /* Q12 */
    Word16 L_subfr,
    Word16 *code2,
    Word32 *gain_code2,
    Word8 noisy_speech_flag /* (i) : noisy speech flag                                            */
);

extern const long int pulsestable[6][7];
extern const PulseConfig PulseConfTable[];
extern const PulseConfig PulseConfTable_NB[];
extern const Word16 ACELP_CDK_BITS[];

void D_ACELP_indexing(
    Word16 code[],
    PulseConfig config,
    Word16 num_tracks,
    Word16 index[]
);
void D_ACELP_decode_43bit(UWord16 idxs[], Word16 code[], Word16 *pulsestrack);
void fcb_pulse_track_joint_decode(UWord16 *idxs, Word16 wordcnt, UWord32 *index_n, Word16 *pulse_num, Word16 track_num);

/* lag window lag_wind.c */
void lag_wind(
    Word16 r_h[],           /* in/out: autocorrelations                                       */
    Word16 r_l[],           /* in/out: autocorrelations                                       */
    Word16 m,               /* input : order of LP filter                                     */
    Word32 sr,              /* input : sampling rate                                          */
    Word16 strength         /* input : LAGW_WEAK, LAGW_MEDIUM, or LAGW_STRONG                 */
);

void adapt_lag_wind(
    Word16 r_h[],           /* in/out: autocorrelations                                       */
    Word16 r_l[],           /* in/out: autocorrelations                                       */
    Word16 m,               /* input : order of LP filter                                     */
    const Word16 Top,       /* input : open loop pitch lag                                    */
    const Word16 Tnc,       /* input : open loop pitch gain                                   */
    Word32 sr               /* input : sampling rate                                          */
);

void hp20(Word16 signal[],     /* i/o: signal to filter                   any */
          const Word16 stride,       /* i  : stride to be applied accessing signal  */
          const Word16 lg,           /* i  : length of signal (integer)          Q0 */
          Word32 mem[5],       /* i/o: static filter memory with this layout: */
          /*      mem[0]: y[-2] (32-bit)                 */
          /*      mem[1]; y[-1] (32-bit)                 */
          /*      mem[2]: x[-2] << 16                    */
          /*      mem[3]: x[-1] << 16                    */
          /* Note: mem[0..3] need to be scaled per frame */
          /*      mem[5]: states scale                   */
          const Word32 sFreq);       /* i  : input sampling rate                 Q0 */

/* pit_fr4.c */
void Mode2_pred_lt4(Word16 exc[],      /* in/out: excitation buffer */
                    Word16 T0,           /* input : integer pitch lag */
                    Word16 frac,         /* input : fraction of lag */
                    Word16 T0_res,       /* input : pitch lag resolution */
                    Word16 T0_res_max,   /* input : maximum resolution */
                    Word16 L_subfr);     /* input : subframe size */


#define mvr2r_Word32(x,y,n)    Copy32(x,y,n)
#define mvr2r_Word16(x,y,n)    Copy(x,y,n)
#define vr_intset(val, vector, count) { Word16 i; FOR(i=0; i<(count); i++) { vector[i]=val; } }

float mean(                    /* o  : the mean of the elements of the vector */
    const float *vec,  /* i  : input vector                           */
    const short lvec   /* i  : length of input vector                 */
);

/* window.c */
void ham_cos_window(Word16 *fh, const Word16 n1, const Word16 n2);


/*---------------------------------------------------------------------*
 *              main routines                                          *
 *---------------------------------------------------------------------*/

void init_coder_ace_plus( Encoder_State_fx* st,const Word16 shift);
void core_coder_reconfig( Encoder_State_fx *st);
void core_coder_mode_switch( Encoder_State_fx *st, const Word16 bandwidth_in, const Word32 bitrate, const Word16 shift);

void enc_acelp_tcx_main(
    const Word16 new_samples[],          /* i  : new samples                         */
    Encoder_State_fx *st,                /* i/o: encoder state structure             */
    const Word16 coder_type,             /* i  : coding type                         */
    const Word16 pitch[3],               /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],             /* i  : open-loop pitch gains               */
    Word16 Aw[NB_SUBFR16k*(M+1)],        /* i  : weighted A(z) unquant. for subframes*/
    const Word16 lsp_new[M],             /* i  : LSPs at the end of the frame        */
    const Word16 lsp_mid[M],             /* i  : LSPs at the middle of the frame     */
    HANDLE_FD_CNG_ENC hFdCngEnc,         /* i/o: FD CNG handle                      */
    Word32 bwe_exc_extended[],           /* i/o: bandwidth extended excitation       */
    Word16 *voice_factors,               /* o  : voicing factors                     */
    Word16 pitch_buf[],                  /* o  : floating pitch for each subframe    */
    Word16 vad_hover_flag,
    Word16 *Q_new,
    Word16 *shift
);

/*-----------------------------------------------------------------*
 *   Pitch prediction for frame erasure                            *
 *-----------------------------------------------------------------*/
void pitch_pred_linear_fit(
    const Word16 /*short*/ bfi_cnt,           /* i:   bfi counter                                   */ /*Q0 */
    const Word16 /*short*/ last_good,         /* i:   last classification type                      */ /*Q0 */
    Word32 /*float*/ *old_pitch_buf,          /* i:   pitch lag buffer                              */ /*Q16*/
    Word32 /*float*/ *old_fpitch,             /* i:                                                 */ /*Q16*/
    Word32 /*float*/ *T0_out,                 /* o:   estimated close loop pitch                    */ /*Q16*/
    Word16 /*  int*/ pit_min,                 /* i:   Minimum pitch lag                             */ /*Q0 */
    Word16 /*  int*/ pit_max,                 /* i:   Maximum pitch lag                             */ /*Q0 */
    Word16 /*float*/ *mem_pitch_gain,         /* i:   pitch gain [0] is the most recent subfr gain  */ /*Q0*/
    Word16 /*  int*/ limitation,
    Word8  /*short*/ plc_use_future_lag,      /* i:   */                                               /*Q0 */
    Word16 /*short*/ *extrapolationFailed,    /* o: flag if extrap decides not to change the pitch  *//*Q0 */
    Word16 nb_subfr                           /* i:   number of ACELP subframes                     */
);

void get_subframe_pitch(
    Word16 nSubframes,   /* i:   number of subframes                              */   /*  Q0 */
    Word32 pitchStart,   /* i:   starting pitch lag (in subframe -1)              */   /*15Q16*/
    Word32 pitchEnd,     /* i:   ending pitch lag (in subframe nSubframes-1)      */   /*15Q16*/
    Word32 *pitchBuf     /* o:   interpolated pitch lag per subframe              */   /*15Q16*/
);

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
);

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
);

void core_encode_twodiv(
    const Word16 new_samples[],
    Encoder_State_fx *st,
    const Word16 coder_type,             /* i  : coding type                         */
    const Word16 pitch[3],                /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],             /* i  : open-loop pitch gains               */
    Word16 Aw[NB_SUBFR16k*(M+1)],  /* i  : weighted A(z) unquant. for subframes*/
    Word16 *Q_new,
    Word16 *shift
);

void core_encode_update( Encoder_State_fx *st);

void core_encode_update_cng( Encoder_State_fx *st,
                             Word16 *timeDomainBuffer,
                             Word16 *A,
                             Word16 *Aw,
                             Word16 Q_new,
                             Word16 shift
                           );

/* core_sig_ana.c */
void core_signal_analysis_post(const Word16 *new_samples, Encoder_State_fx *st);

/* ext_sig_ana.c */
void core_signal_analysis_enc(const Word16 *new_samples,  /*<!i: 0Q15*/
                              Word16 isp[][M],      /*<!o: 1Q14*/
                              Word16 isf[][M],      /*<!o: 3Q12*/
                              Word16 ispmid[][M],  /*<!i: 1Q14*/
                              Word16 isfmid[][M],  /*<!o: 3Q12*/
                              Encoder_State_fx *st);
void core_signal_analysis_high_bitrate( const Word16 *new_samples,          /*<!i: 0Q15*/
                                        const Word16 T_op[3],        /* i  : open-loop pitch values for quantiz. */
                                        const Word16 voicing[3],     /* i  : open-loop pitch gains               */
                                        const Word16 pitch[2],
                                        Word16 Aw[NB_SUBFR16k*(M+1)],/* i  : weighted A(z) unquant. for subframes*/
                                        Word16 lsp_new[],
                                        Word16 lsp_mid[],
                                        Encoder_State_fx *st,
                                        Word16 pTnsSize[],
                                        Word16 pTnsBits[],
                                        Word16 param_core[],
                                        Word16 *ltpBits,
                                        Word16 L_frame,
                                        Word16 L_frameTCX,
                                        Word32 **spectrum,
                                        Word16 *spectrum_e,
                                        Word16 *Q_new,
                                        Word16 *shift
                                      );

void weight_a_subfr_fx(
    const Word16 nb_subfr,   /* i  : number of subframes                 */
    const Word16 *A,         /* i  : LP filter coefficients          Q12 */
    Word16 *Aw,        /* o  : weighted LP filter coefficients Q12 */
    const Word16 gamma,      /* i  : weighting factor                    */
    const Word16 m           /* i  : order of LP filter                  */
);


/*---------------------------------------------------------------------*
 *              ACELP and TCX routines                                 *
 *---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*
 *             misc                                                    *
 *---------------------------------------------------------------------*/

Word32 get_gain(     /*<! output: codebook gain (adaptive or fixed)  Q16 */
    Word16 x[],        /*<! input : target signal                        */
    Word16 y[],        /*<! input : filtered codebook excitation         */
    Word16 n           /*<! input : segment length                       */
);

Word32 get_gain2(     /*<! output: codebook gain (adaptive or fixed)   Q16 */
    Word16 x[],        /*<! input : target signal                        */
    Word16 y[],        /*<! input : filtered codebook excitation         */
    Word16 n           /*<! input : segment length                       */
);

/*---------------------------------------------------------------------*
 *                          TCX.H                                      *
 *---------------------------------------------------------------------*
 *             Prototypes of signal processing routines                *
 *---------------------------------------------------------------------*/

/* q_gain2p.c */
Word16 gain_enc(
    const Word16 *code,            /*<! i    : algebraic excitation                                            */
    Word16 lcode,                  /*<! (i)  : Subframe size                                                   */
    Word16 *gain_pit,              /*<! o    : quantized pitch gain                                            */
    Word32 *gain_code,             /*<! o    : quantized codebook gain                                         */
    ACELP_CbkCorr *g_coeff,        /*<! i/o  : correlations <y1,y1>, -2<xn,y 1>,<y2,y2>, -2<xn,y2> and 2<y1,y2>*/
    Word16 mean_ener,              /*<! (i)  : mean_ener defined in open-loop Q8                               */
    const Word16 clip_gain,        /*<! i    : gain pitch clipping flag (1 = clipping)                         */
    Word32 *past_gcode,            /*<! (i/o): past gain of code                                               */
    Word16 *gain_inov,             /*<! (o)  : Q12 innovation gain                                             */
    const Word16 coder_type,       /*<! (i)  : coder type                                                      */
    const Word16 func_type         /*<! (i) : encode algorithm. 0: gain_enc_mless, 1:gain_enc_2                */
);

Word16 gain_enc_uv(
    const Word16 *code,            /*<! i    : algebraic excitation                                            */
    const Word16 *code2,           /*<! i    : gaussian excitation                                             */
    Word16 lcode,                  /*<! (i)  : Subframe size                                                   */
    Word16 *gain_pit,              /*<! o    : quantized pitch gain                                            */
    Word32 *gain_code,             /*<! o    : quantized codebook gain                                         */
    Word32 *gain_code2,            /*<! o    : quantized codebook gain                                         */
    Word8 noisy_speech_flag,       /* (i)    : noisy speech flag                                               */
    ACELP_CbkCorr *g_coeff,        /*<! i/o  : correlations <y1,y1>, -2<xn,y 1>,<y2,y2>, -2<xn,y2> and 2<y1,y2>*/
    Word16 mean_ener,              /*<! (i)  : mean_ener defined in open-loop Q8                               */
    Word32 *past_gcode,            /*<! (i/o): past gain of code                                               */
    Word16 *gain_inov,             /*<! (o)  : Q12 innovation gain                                             */
    const Word16 func_type         /*<! (i) : encode algorithm. 2:gain_enc_uv, 3:gain_enc_gacelp_uv            */
);

/* gp_clip.c*/

Word32 calc_gain_inov(                      /* returns innovation gain              Q16 */
    const Word16 *code,   /* i  : algebraic excitation            Q9  */
    Word16 lcode,         /* i  : Subframe size                   Q0  */
    Word32 *dotp,         /* o  : intermediate result           Q31-e */
    Word16 *dotp_e        /* o  : intermediate result exponent    Q0  */
);


Word16 Mode2_gp_clip(
    const Word16 *voicing,    /* i  : normalized correlations (from OL pitch)    */
    const Word16 i_subfr,     /* i  : subframe index                             */
    const Word16 coder_type,  /* i  : type of coder                              */
    const Word16 xn[],        /* i  : target vector                              */
    Word16 mem[],       /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 L_subfr,
    const Word16 Q_xn         /* i  : xn data format */
);

void gp_clip_test_isf(
    const Word16 isf[],     /* i  : isf values (in frequency domain)            */
    Word16 mem[],     /* i/o: memory of gain of pitch clipping algorithm  */
    const Word16 m      /* i  : dimension of isf              */
);


/* d_gain2p.c */
void decode_acelp_gains(
    Word16 *code,
    Word16 gains_mode,
    Word16 mean_ener_code,
    Word16 *gain_pit,
    Word32 *gain_code,
    Word16 **pt_indice,
    Word16 *past_gpit,
    Word32 *past_gcode,
    Word16 *gain_inov,
    Word16 L_subfr,
    Word16 *code2,
    Word32 *gain_code2
);

void d_gain_pred(
    Word16  nrg_mode,           /* i  : NRG mode                                  */
    Word16  *Es_pred,           /* o  : predicited scaled innovation energy       */
    Word16  **pt_indice         /* i/o: pointer to the buffer of indices          */
);

void Mode2_pit_encode(
    const Word16 coder_type, /* i  : coding model                               */
    const Word16 i_subfr,      /* i  : subframe index                             */
    Word16 **pt_indice,  /* i/o: quantization indices pointer               */
    Word16 *exc,         /* i/o: pointer to excitation signal frame         */
    const Word16 *T_op,        /* i  : open loop pitch estimates in current frame */
    Word16 *T0_min,      /* i/o: lower limit for close-loop search          */
    Word16 *T0_min_frac, /* i/o: lower limit for close-loop search          */
    Word16 *T0_max,      /* i/o: higher limit for close-loop search         */
    Word16 *T0_max_frac, /* i/o: higher limit for close-loop search         */
    Word16 *T0,          /* i/o: close loop integer pitch                   */
    Word16 *T0_frac,     /* i/o: close loop fractional part of the pitch    */
    Word16 *T0_res,      /* i/o: close loop pitch resolution                */
    Word16 *h1,          /* i  : weighted filter impulse response           */
    Word16 *xn,          /* i  : target vector                              */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr1b,
    Word16 pit_fr2,
    Word16 pit_max,
    Word16 pit_res_max
);

void limit_T0_voiced(
    const Word16 nbits,
    const Word16 res,
    const Word16 T0,            /* i  : rough pitch estimate around which the search is done */
    const Word16 T0_frac,       /* i  : pitch estimate fractional part                       */
    const Word16 T0_res,        /* i  : pitch resolution                                     */
    Word16 *T0_min,       /* o  : lower pitch limit                                    */
    Word16 *T0_min_frac,  /* o  : lower pitch limit                                    */
    Word16 *T0_max,       /* o  : higher pitch limit                                   */
    Word16 *T0_max_frac,  /* o  : higher pitch limit                                   */
    const Word16 pit_min,       /* i  : Minimum pitch lag                                    */
    const Word16 pit_max        /* i  : Maximum pitch lag                                    */
);

void Mode2_abs_pit_enc(
    Word16 T0,          /* i  : integer pitch lag              */
    Word16 T0_frac,     /* i  : pitch fraction                 */
    Word16 **pt_indice, /* i/o: pointer to Vector of Q indexes */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr2,
    Word16 pit_res_max
);

void Mode2_delta_pit_enc(
    Word16 T0,          /* i  : integer pitch lag              */
    Word16 T0_frac,     /* i  : pitch fraction                 */
    Word16 T0_res,      /* i  : pitch resolution               */
    Word16 T0_min,      /* i/o: delta search min               */
    Word16 T0_min_frac, /* i/o: delta search min               */
    Word16 **pt_indice  /* i/o: pointer to Vector of Q indexes */
);

Word32 Mode2_pit_decode(             /* o:   floating pitch value                      */
    const Word16 coder_type,   /* i:   coding model                              */
    Word16 i_subfr,      /* i:   subframe index                            */
    Word16 L_subfr,
    Word16 **pt_indice,  /* i/o: quantization indices pointer              */
    Word16 *T0,          /* i/o:   close loop integer pitch                */
    Word16 *T0_frac,     /* o:   close loop fractional part of the pitch   */
    Word16 *T0_res,      /* i/o: pitch resolution                          */
    Word16 *T0_min,      /* i/o: lower limit for close-loop search         */
    Word16 *T0_min_frac, /* i/o: lower limit for close-loop search         */
    Word16 *T0_max,      /* i/o: higher limit for close-loop search        */
    Word16 *T0_max_frac, /* i/o: higher limit for close-loop search        */
    Word16  pit_min,
    Word16  pit_fr1,
    Word16  pit_fr1b,
    Word16  pit_fr2,
    Word16  pit_max,
    Word16  pit_res_max
);

void Mode2_abs_pit_dec(
    Word16 *T0,         /* o:   integer pitch lag              */
    Word16 *T0_frac,    /* o:   pitch fraction                 */
    Word16 *T0_res,     /* o:   pitch resolution               */
    Word16 **pt_indice, /* i/o: pointer to Vector of Q indexes */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr2,
    Word16 pit_res_max
);

void Mode2_delta_pit_dec(
    Word16       *T0,          /* o:   integer pitch lag              */
    Word16       *T0_frac,     /* o:   pitch fraction                 */
    Word16       T0_res,       /* i:   pitch resolution               */
    Word16       *T0_min,      /* i: delta search min                 */
    Word16       *T0_min_frac, /* i: delta search min                 */
    Word16       **pt_indice   /* i/o: pointer to Vector of Q indexes */
);

void pitchDoubling_det(
    Word16 *wspeech,
    Word16 *pitch_ol,
    Word16 *T_op_fr,
    Word16 *voicing_fr
);

void nb_post_filt(
    const Word16 L_frame,       /* i  : frame length                            */
    PFSTAT *Pfstat,       /* i/o: Post filter related memories            */
    Word16 *psf_lp_noise, /* i  : Long term noise                   Q8    */
    const Word16 tmp_noise,     /* i  : noise energy                      Q0    */
    Word16 *Synth,        /* i  : 12k8 synthesis                    Qsyn  */
    const Word16 *Aq,           /* i  : LP filter coefficient             Q12   */
    const Word16 *Pitch_buf,    /* i  : Fractionnal subframe pitch buffer Q6    */
    const Word16 coder_type,    /* i  : coder_type                              */
    const Word16 disable_hpf    /* i  : flag to diabled HPF                     */
);

void Init_post_filter(PFSTAT * pfstat); /* (i)   : core decoder parameters */

void formant_post_filt(
    PFSTAT *pfstat,       /* i/o: Post filter related memories      */
    Word16 *synth_in,        /* i  : 12k8 synthesis                    */
    Word16 *Aq,           /* i  : LP filter coefficient             */
    Word16 *synth_out,  /* i/o: input signal                      */
    Word16 L_frame,
    Word32 lp_noise,  /* (i) : background noise energy (15Q16) */
    Word32 rate,         /* (i) : bit-rate */
    const Word16 off_flag        /* i  : off flag                        */
);

void scale_st(
    const Word16 * sig_in,    /* i  : postfilter i signal             */
    Word16 * sig_out,   /* i/o: postfilter o signal             */
    Word16 * gain_prec, /* i/o: last value of gain for subframe */
    Word16 L_subfr
);

void Calc_rc0_h(
    Word16 * h,   /* i  : impulse response of composed filter */
    Word16 * rc0  /* o  : 1st parcor */
);

void Filt_mu(
    Word16 * sig_in,        /* i  : signal (beginning at sample -1)     */
    Word16 * sig_out,       /* o  : signal with tilt                    */
    Word16 parcor0,         /* i  : parcor0 (mu = parcor0 * gamma3)     */
    Word16 L_subfr          /* i  : the length of subframe              */
);

/* fft_rel.c */

#define SIZE_256         256
#define NUM_STAGE_256    7
#define SIZE2_256        (SIZE_256/2)


/*-----------------------------------------------------------------------*
 * phase_dispersion:
 *
 * post-processing to enhance noise at low bit rate.
 *-----------------------------------------------------------------------*/

void phase_dispersion(
    const Word32 gain_code,  /* i  : gain of code  15Q16       */
    const Word16 gain_pit,   /* i  : gain of pitch   Q14       */
    Word16 code[],           /* i/o: code vector               */
    Word16 *code_e,          /* i/o: exponent of code          */
    const Word16 mode,       /* i  : level, 0=hi, 1=lo, 2=off  */
    Word32 *prev_gain_code,  /* i/o: static memory 15Q16       */
    Word16 prev_gain_pit[],  /* i/o: static memory Q14, size=6 */
    Word16 *prev_state,      /* i/o: static memory          Q0 */
    Word16 L_subfr           /* i  : subframe length [40,64]   */
);

void mdct_window_sine(const PWord16 **window, const Word16 n);

void mdct_window_aldo(
    Word16 *window1,
    PWord16 *window1_trunc,
    PWord16 *window2,
    Word16 n
);

void AVQ_cod_lpc(
    Word16 *nvec,  /* input:  vector to quantize (normalized) */
    Word16 *nvecq, /* output: quantized vector                */
    Word16 *indx,  /* output: index[] (4 bits per words)      */
    Word16 Nsv     /* input:  number of subvectors (lg=Nsv*8) */
);

void AVQ_dec_lpc(
    Word16 *indx,    /* input:  index[] (4 bits per words)      */
    Word16 *nvecq,   /* output: vector quantized                */
    Word16 Nsv);     /* input:  number of subvectors (lg=Nsv*8) */

void vlpc_1st_dec(
    Word16 index,      /* input:  codebook index                  */
    Word16 *lsfq);   /* i/o:    i:prediction   o:quantized lsf  */

void GetMDCT(
    TCX_config *tcx_cfg, /* input: configuration of TCX         */
    Word16 speech[],     /* input: speech[-LFAC..L_frame+LFAC]   */
    Word16 L_frame_glob,   /* input: frame length                  */
    LPD_state *LPDmem,  /* i/o: memories                        */
    Word32 spectrum[],   /* output: MDCT spectrum                */
    Word16 *spectrum_e,  /* output: MDCT spectrum exponent       */
    Word32 powerSpec[],  /* output: power spectrum (MDCT + MDST) */
    Word16 *powerSpec_e, /* output: power spectrum exponent      */
    Word16 startRatio    /* 8Q7 */
);

void HBAutocorrelation(
    TCX_config *tcx_cfg,        /* input: configuration of TCX          */
    Word16 left_overlap_mode,   /* input: overlap mode of left window half   */
    Word16 right_overlap_mode,  /* input: overlap mode of right window half  */
    Word16 speech[],            /* input: speech[-LFAC..L_frame+LFAC]   */
    Word16 L_frame,             /* input: frame length                  */
    Word32 *r,                  /* output: autocorrelations vector */
    Word16 m                    /* input : order of LP filter      */
);

void TNSAnalysis(
    TCX_config *tcx_cfg,    /* input: configuration of TCX */
    Word16 L_frame,         /* input: frame length */
    Word16 L_spec,
    Word16 tcxMode,         /* input: TCX mode for the frame/subframe - TCX20 | TCX10 | TCX 5 (meaning 2 x TCX 5) */
    Word8 isAfterACELP,     /* input: Flag indicating if the last frame was ACELP. For the second TCX subframe it should be 0  */
    Word32 spectrum[],      /* input: MDCT spectrum */
    STnsData * pTnsData,    /* output: Tns data */
    Word8 * pfUseTns,       /* output: Flag indicating if TNS is used */
    Word16 *predictionGain
);

void ShapeSpectrum(
    TCX_config *tcx_cfg,/*input: configuration of TCX*/
    Word16 A[],         /* input: quantized coefficients NxAz_q[M+1] */
    Word16 gainlpc[],   /* output: MDCT gains for the previous frame */
    Word16 gainlpc_e[], /* output: MDCT gains exponents */
    Word16 L_frame_glob,/* input: frame length             */
    Word16 L_spec,
    Word32 spectrum[],  /* i/o: MDCT spectrum */
    Word8 pfUseTns,     /* output: Flag indicating if TNS is used */
    Encoder_State_fx *st
);

void QuantizeSpectrum(
    TCX_config *tcx_cfg,    /*input: configuration of TCX*/
    Word16 A[],             /* input: quantized coefficients NxAz_q[M+1] */
    Word16 Aqind[],         /* input: frame-independent quantized coefficients (M+1) */
    Word16 gainlpc[],       /* input: MDCT gains of the previous frame */
    Word16 gainlpc_e[],     /* input: MDCT gains exponents */
    Word16 synth[],
    Word16 L_frame_glob,    /* input: frame length             */
    Word16 L_frameTCX_glob,
    Word16 L_spec,
    Word16 nb_bits,         /*input: bit budget*/
    Word8 tcxonly,          /*input: only TCX flag*/
    Word32 spectrum[],      /* i/o: MDCT spectrum, input is shaped MDCT spectrum */
    Word16 *spectrum_e,     /* i/o: MDCT spectrum exponent */
    STnsData * pTnsData,    /* input: Tns data */
    Word8 fUseTns,          /* input: Flag indicating if TNS is used */
    Word16 tnsSize,         /* input: number of tns parameters put into prm */
    LPD_state *LPDmem,      /*i/o: memories*/
    Word16 prm[],           /* output: tcx parameters          */
    Word16 frame_cnt,       /* input: frame counter in the super_frame */
    Encoder_State_fx *st,
    CONTEXT_HM_CONFIG *hm_cfg
);

/* Returns: index of next coefficient */
Word16 get_next_coeff_mapped(
    Word16 ii[2],             /* i/o: coefficient indexes       */
    Word16 *pp,               /* o  : peak(1)/hole(0) indicator */
    Word16 *idx,              /* o  : index in unmapped domain  */
    CONTEXT_HM_CONFIG *hm_cfg /* i  : HM configuration          */
);

/* Returns: index of next coefficient */
Word16 get_next_coeff_unmapped(
    Word16 ii[2],             /* i/o: coefficient indexes       */
    Word16 *pp,               /* o  : peak(1)/hole(0) indicator */
    Word16 *idx,              /* o  : index in unmapped domain  */
    CONTEXT_HM_CONFIG *hm_cfg /* i  : HM configuration          */
);

Word16 update_mixed_context(Word16 ctx, Word16 a);

Word16 ACcontextMapping_encode2_no_mem_s17_LC(
    Encoder_State_fx *st,
    Word16 *x,
    Word16 nt,
    Word16 lastnz,
    Word16 nbbits,
    Word16 resQMaxBits,
    CONTEXT_HM_CONFIG *hm_cfg);

Word16 ACcontextMapping_decode2_no_mem_s17_LC(
    Decoder_State_fx *st,/* i/o: decoder state */
    Word16 *x,          /* o: decoded spectrum */
    Word16 nt,         /* i: size of spectrum */
    Word16 nbbits,      /* i: bit budget */
    Word16 resQMaxBits, /* i: residual coding maximum bits*/
    CONTEXT_HM_CONFIG *hm_cfg /* i: context-based harmonic model configuration */
);

Word16 ACcontextMapping_encode2_estimate_no_mem_s17_LC(
    const Word16 *x,
    Word16 nt,
    Word16 *lastnz,
    Word16 *nEncoded,
    Word16 target,
    Word16 *stop,
    CONTEXT_HM_CONFIG *hm_cfg
);


/* tcx_utils.h */

Word16 getInvFrameLen(Word16 L_frame); /* returns 1/L_frame in Q21 format */

void WindowSignal(
    TCX_config const *tcx_cfg,                /* input: configuration of TCX              */
    Word16 offset,                            /* input: left folding point offset relative to the input signal pointer */
    Word16 left_overlap_mode,                 /* input: overlap mode of left window half  */
    Word16 right_overlap_mode,                /* input: overlap mode of right window half */
    Word16 * left_overlap_length,             /* output: TCX window left overlap length   */
    Word16 * right_overlap_length,            /* output: TCX window right overlap length  */
    Word16 const in[],                        /* input: input signal                      */
    Word16 * L_frame,                         /* input/output: frame length               */
    Word16 out[]                              /* output: output windowed signal           */
    ,Word8 fullband                           /* input: fullband flag                     */
);

void tcx_windowing_synthesis_current_frame(
    Word16 *signal,             /* i/o: signal vector                            */
    const PWord16 *window,      /* i: TCX window vector                          */
    const PWord16 *window_half, /* i: TCX window vector for half-overlap window  */
    const PWord16 *window_min,  /* i: TCX minimum overlap window                 */
    Word16 window_length,       /* i: TCX window length                          */
    Word16 window_half_length,  /* i: TCX half window length                     */
    Word16 window_min_length,   /* i: TCX minimum overlap length                 */
    Word16 left_rect,           /* i: left part is rectangular                   */
    Word16 left_mode,           /* i: overlap mode of left window half           */
    Word16 *acelp_zir,          /* i: acelp ZIR                                  */
    Word16 *old_syn,            /* i: old synthesis                              */
    Word16 *syn_overl,          /* i: overlap synthesis                          */
    Word16 *A_zir,
    const PWord16 *window_trans,
    Word16 acelp_zir_len,
    Word16 acelp_mem_len,
    Word16 bfi,
    Word16 last_core_bfi,       /* i :  last core                                */
    Word8 last_is_cng
    ,Word16 fullbandScale
);

void tcx_windowing_synthesis_past_frame(
    Word16 *signal,             /* i/o: signal vector                            */
    const PWord16 *window,      /* i: TCX window vector                          */
    const PWord16 *window_half, /* i: TCX window vector for half-overlap window  */
    const PWord16 *window_min,  /* i: TCX minimum overlap window                 */
    Word16 window_length,       /* i: TCX window length                          */
    Word16 window_half_length,  /* i: TCX half window length                     */
    Word16 window_min_length,   /* i: TCX minimum overlap length                 */
    Word16 right_mode           /* i: overlap mode (left_mode of current frame)  */
);

/* compute noise-measure flags for spectrum filling and quantization (0: tonal, 1: noise-like) */
void ComputeSpectrumNoiseMeasure(const Word32 *powerSpec,
                                 Word16 L_frame,
                                 Word16 startLine,
                                 Word8 resetMemory,
                                 Word16 *noiseFlags,
                                 Word16 lowpassLine);

void detectLowpassFac(const Word32 *powerSpec, Word16 powerSpec_e, Word16 L_frame, Word8 rectWin, Word16 *pLpFac, Word16 lowpassLine);
void ProcessIGF(
    IGF_ENC_INSTANCE_HANDLE  const hInstance,          /**< in: instance handle of IGF Encoder */
    Encoder_State_fx              *st,                 /**< in: Encoder state */
    Word32                         pMDCTSpectrum[],    /**< in: MDCT spectrum */
    Word16                        *pMDCTSpectrum_e,
    Word32                         pPowerSpectrum[],   /**< in: MDCT^2 + MDST^2 spectrum, or estimate */
    Word16                        *pPowerSpectrum_e,
    Word16                         isTCX20,            /**< in: flag indicating if the input is TCX20 or TCX10/2xTCX5 */
    Word16                         isTNSActive,        /**< in: flag indicating if the TNS is active */
    Word16                         isTransition,       /**< in: flag indicating if the input is the transition from from ACELP to TCX20/TCX10 */
    Word16                         frameno             /**< in: flag indicating index of current subframe */
);
void AnalyzePowerSpectrum(
    Encoder_State_fx *st,              /* i/o: encoder states                                  */
    Word16 L_frame,                   /* input: frame length                                  */
    Word16 L_frameTCX,                /* input: full band frame length                        */
    Word16 left_overlap,              /* input: left overlap length                           */
    Word16 right_overlap,             /* input: right overlap length                          */
    Word32 const mdctSpectrum[],      /* input: MDCT spectrum                                 */
    Word16 mdctSpectrum_e,
    Word16 const signal[],            /* input: windowed signal corresponding to mdctSpectrum */
    Word32 powerSpec[],               /* output: Power spectrum. Can point to signal          */
    Word16 *powerSpec_e
);

void lpc2mdct(Word16 *lpcCoeffs, Word16 lpcOrder,
              Word16 *mdct_gains, Word16 *mdct_gains_exp,
              Word16 *mdct_inv_gains, Word16 *mdct_inv_gains_exp);

void mdct_shaping(Word32 x[], Word16 lg, Word16 const gains[], Word16 const gains_exp[]);

void mdct_shaping_16(Word16 const x[], Word16 lg, Word16 lg_total, Word16 const gains[], Word16 const gains_exp[], Word16 gains_max_exp, Word32 y[]);

void mdct_noiseShaping_interp(Word32 x[], Word16 lg, Word16 gains[], Word16 gains_exp[]);

void AdaptLowFreqEmph(Word32 x[],
                      Word16 x_e,
                      Word16 xq[],
                      Word16 invGain,
                      Word16 invGain_e,
                      Word16 tcx_lpc_shaped_ari,
                      Word16 lpcGains[], Word16 lpcGains_e[],
                      const Word16 lg
                     );

void PsychAdaptLowFreqEmph(Word32 x[],
                           const Word16 lpcGains[], const Word16 lpcGains_e[]
                          );
void PsychAdaptLowFreqDeemph(Word32 x[],
                             const Word16 lpcGains[], const Word16 lpcGains_e[],
                             Word16 lf_deemph_factors[]
                            );

void AdaptLowFreqDeemph(Word32 x[], Word16 x_e,
                        Word16 tcx_lpc_shaped_ari,
                        Word16 lpcGains[], Word16 lpcGains_e[],
                        const Word16 lg,
                        Word16 lf_deemph_factors[]
                       );

Word16 SQ_gain(     /* output: SQ gain                   */
    Word32 x[],       /* input:  vector to quantize        */
    Word16 x_e,       /* input:  exponent                  */
    Word16 nbitsSQ,   /* input:  number of bits targeted   */
    Word16 lg,        /* input:  vector size (2048 max)    */
    Word16 *gain_e);  /* output: SQ gain exponent          */

void tcx_scalar_quantization(
    Word32 *x,                   /* i: input coefficients            */
    Word16 x_e,                  /* i: exponent                      */
    Word16 *xq,                  /* o: quantized coefficients        */
    Word16 L_frame,              /* i: frame length                  */
    Word16 gain,                 /* i: quantization gain             */
    Word16 gain_e,               /* i: quantization gain exponent    */
    Word16 offset,               /* i: rounding offset (deadzone)    */
    Word16 const *memQuantZeros, /* i: coefficients to be set to 0   */
    const Word16 alfe_flag
);

Word16 tcx_scalar_quantization_rateloop(
    Word32 *x,                    /* i  : input coefficients            */
    Word16 x_e,                   /* i  : exponent                      */
    Word16 *xq,                   /* o  : quantized coefficients        */
    Word16 L_frame,               /* i  : frame length                  */
    Word16 *gain,                 /* i/o: quantization gain             */
    Word16 *gain_e,               /* i/o: gain exponent                 */
    Word16 offset,                /* i  : rounding offset (deadzone)    */
    Word16 const *memQuantZeros,  /* i  : coefficients to be set to 0   */
    Word16 *lastnz_out,           /* i/o: last nonzero coeff index      */
    Word16 target,                /* i  : target number of bits         */
    Word16 *nEncoded,             /* o  : number of encoded coeff       */
    Word16 *stop,                 /* i/o: stop param                    */
    Word16 sqBits_in_noStop,      /* i  : number of sqBits as determined in prev. quant. stage, w/o using stop mechanism (ie might exceed target bits) */
    Word16 sqBits_in,             /* i  : number of sqBits as determined in prev. quant. stage, using stop mechanism (ie always <= target bits) */
    Word16 tcxRateLoopOpt,        /* i  : turns on/off rateloop optimization */
    const Word8 tcxonly,
    CONTEXT_HM_CONFIG *hm_cfg     /* i  : configuration of the context-based harmonic model */
);

/** Quantize gain.
 * Quantize gain in range [0..127],
 * @param n Length of the spectrum.
 * @param pGain I/O gain to be quantized.
 On return set to the dequantized value.
 * @param pQuantizedGain The quantized gain will be stored in pQuantizedGain.
 */
void QuantizeGain(Word16 n, Word16 *pGain, Word16 *pGain_e, Word16 *pQuantizedGain);

void tcx_noise_factor(
    Word32 *x_orig,         /* i: unquantized mdct coefficients             */
    Word16 x_orig_e,        /* i: exponent                                  */
    Word32 *sqQ,            /* i: quantized mdct coefficients               */
    Word16 iFirstLine,      /* i: first coefficient to be considered        */
    Word16 lowpassLine,     /* i: last nonzero coefficients after low-pass  */
    Word16 nTransWidth,     /* i: minimum size of hole to be checked        */
    Word16 L_frame,         /* i: frame length                              */
    Word16 gain_tcx,        /* i: tcx gain                                  */
    Word16 gain_tcx_e,      /* i: gain exponent                             */
    Word16 tiltCompFactor,  /* i: LPC tilt compensation factor              */
    Word16 *fac_ns,         /* o: noise factor                              */
    Word16 *quantized_fac_ns/* o: quantized noise factor                    */
);

void tcx_noise_filling(
    Word32 *Q,
    Word16 Q_e,
    Word16 seed,
    Word16 iFirstLine,
    Word16 lowpassLine,
    Word16 nTransWidth,
    Word16 L_frame,
    Word16 tiltCompFactor,
    Word16 fac_ns,
    Word16 *infoTCXNoise
);

void tcx_encoder_memory_update(
    Word16 *wsig,          /* i: targert weighted signal  */
    Word16 *xn_buf,        /* i/o: mdct output buffer/TD weigthed synthesis  */
    Word16 L_frame_glob,   /* i: global frame length                         */
    const Word16 *Ai,      /* i: Unquantized (interpolated) LPC coefficients */
    const Word16 *A,       /* i: Quantized LPC coefficients                  */
    Word16 preemph,        /* i: preemphasis factor*/
    LPD_state *LPDmem,     /* i/o: coder memory state                        */
    Encoder_State_fx *st,
    Word16 *synthout,    /* o: synthesis signal */
    Word16 Q_new,
    Word16 shift
);

void tcx_decoder_memory_update(
    Word16 *xn_buf,         /* i: mdct output buffer                          */
    Word16 *synthout,       /* i/o: synth                                     */
    Word16 L_frame_glob,    /* i: global frame length                         */
    Word16 *A,              /* i: Quantized LPC coefficients                  */
    Decoder_State_fx *st,
    Word16 *syn,            /* o: st->syn                                     */
    Word8 fb                /* i: fullband flag                               */
);

Word16 tcx_get_segSnr(Word16 *wsig, Word16 wsig_e,
                      Word32 *X, Word16 X_e,
                      Word32 *Xq, Word16 Xq_e,
                      Word16 *memQuantZeros,
                      const Word16 snrPenalty, /* 1Q14 */
                      Word16 gainlpc_reweight[], Word16 gainlpc_reweight_e[],
                      Word16 fdns_npts,
                      Word8 narrowBand,
                      Word16 gain_tcx, Word16 gain_tcx_e,
                      Word16 L_frame,
                      Word16 L_frame_glob,
                      Word16 L_subfr
                     );

/* Returns: number of bits used (including "bits")  Q0 */
Word16 tcx_ari_res_Q_spec(
    const Word32 x_orig[],  /* i: original spectrum                   Q31-e */
    Word16 x_orig_e,        /* i: original spectrum exponent          Q0 */
    const Word16 signs[],   /* i: signs (x_orig[.]<0)                 Q0 */
    Word32 x_Q[],           /* i/o: quantized spectrum                Q31-e */
    Word16 x_Q_e,           /* i: quantized spectrum exponent         Q0 */
    Word16 L_frame,         /* i: number of lines                     Q0 */
    Word16 gain,            /* i: TCX gain                            Q15-e */
    Word16 gain_e,          /* i: TCX gain exponent                   Q0 */
    Word16 prm[],           /* o: bit-stream                          Q0 */
    Word16 target_bits,     /* i: number of bits available            Q0 */
    Word16 bits,            /* i: number of bits used so far          Q0 */
    Word16 deadzone,        /* i: quantizer deadzone                  Q15 */
    const Word16 x_fac[]    /* i: spectrum post-quantization factors  Q14 */
);

/* Returns: number of bits used (including "bits") */
Word16 tcx_ari_res_invQ_spec(
    Word32 x_Q[],           /* i/o: quantized spectrum                Q31-e */
    Word16 x_Q_e,           /* i: quantized spectrum exponent         Q0 */
    Word16 L_frame,         /* i: number of lines                     Q0 */
    const Word16 prm[],     /* i: bit-stream                          Q0 */
    Word16 target_bits,     /* i: number of bits available            Q0 */
    Word16 bits,            /* i: number of bits used so far          Q0 */
    Word16 deadzone,        /* i: quantizer deadzone                  Q15 */
    const Word16 x_fac[]    /* i: spectrum post-quantization factors  Q14 */
);

Word16 tcx_res_Q_gain(
    Word16 sqGain,
    Word16 sqGain_e,
    Word16 *gain_tcx,
    Word16 *gain_tcx_e,
    Word16 *prm,
    Word16 sqTargetBits
);

Word16 tcx_res_Q_spec(
    Word32 *x_orig,
    Word16 x_orig_e,
    Word32 *x_Q,
    Word16 x_Q_e,
    Word16 L_frame,
    Word16 sqGain,
    Word16 sqGain_e,
    Word16 *prm,
    Word16 sqTargetBits,
    Word16 bits,
    Word16 sq_round,
    const Word16 lf_deemph_factors[]
);

Word16 tcx_res_invQ_gain(
    Word16 *gain_tcx,
    Word16 *gain_tcx_e,
    Word16 *prm,
    Word16 resQBits
);

Word16 tcx_res_invQ_spec(
    Word32 *x,
    Word16 x_e,
    Word16 L_frame,
    Word16 *prm,
    Word16 resQBits,
    Word16 bits,
    Word16 sq_round,
    const Word16 lf_deemph_factors[]
);

void InitTnsConfigs(
    Word32 nSampleRate,
    Word16 L_frame,
    STnsConfig tnsConfig[2][2]
    ,Word16 igfStopFreq
    ,Word32 bitrate
);

void SetTnsConfig(TCX_config * tcx_cfg, Word8 isTCX20, Word8 isAfterACELP);


/* ari.c */
Word32 L_multi31x16_X2(Word16 xh, Word16 xl, Word16 y);
Word32 mul_sbc_14bits(Word32 r, Word16 c);

void ari_copy_states(
    TastatEnc *source,
    TastatEnc *dest
);

void ari_start_encoding_14bits(
    TastatEnc *s
);

Word16 ari_done_encoding_14bits(
    Word16 *ptr,
    Word16  bp,
    TastatEnc *s
);

Word16 ari_encode_14bits_ext(
    Word16  *ptr,
    Word16   bp,
    TastatEnc *s,
    Word32   symbol,
    UWord16 const *cum_freq
);

void ari_start_decoding_14bits(
    Decoder_State_fx *st,
    TastatDec *s
);

Word16 ari_decode_14bits_s17_ext(
    Decoder_State_fx *st,
    TastatDec *s,
    UWord16 const *cum_freq
);

Word16 ari_decode_14bits_s27_ext(
    Decoder_State_fx *st,
    TastatDec *s,
    UWord16 const *cum_freq
);

Word16 ari_decode_14bits_bit_ext(
    Decoder_State_fx *st,
    TastatDec *s
);


Word16 ari_encode_overflow(TastatEnc *s);
Word16 ari_encode_14bits_range(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s, Word16 cum_freq_low, Word16 cum_freq_high);
Word16 ari_encode_14bits_sign(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s, Word16 sign);
Word16 ari_done_cbr_encoding_14bits(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s);
Word16 ari_decode_overflow(TastatDec *s);
Word16 ari_decode_14bits_pow(Word16 *ptr, Word16 bp, Word16 bits, Word16 *res, TastatDec *s, Word16 base);
Word16 ari_decode_14bits_sign(Word16 *ptr, Word16 bp, Word16 bits, Word16 *res, TastatDec *s);
Word16 ari_start_decoding_14bits_prm(const Word16 *ptr, Word16 bp, TastatDec *s);

/*arith_code.c*/
Word32 expfp(  /* o: Q31 */
    Word16 x,           /* i: mantissa  Q-e */
    Word16 x_e);        /* i: exponent  Q0  */

void powfp_odd2(Word16 base,     /* Q15 */
                Word16 exp,      /* Q0  */
                Word16 *pout1,   /* Q15 */
                Word16 *pout2);   /* Q15 */

void tcx_arith_scale_envelope(
    Word16 L_spec_core,         /* i: number of lines to scale    Q0 */
    Word16 L_frame,             /* i: number of lines             Q0 */
    Word32 env[],               /* i: unscaled envelope           Q16 */
    Word16 target_bits,         /* i: number of available bits    Q0 */
    Word16 low_complexity,      /* i: low-complexity flag         Q0 */
    Word16 s_env[],             /* o: scaled envelope             Q15-e */
    Word16 *s_env_e             /* o: scaled envelope exponent    Q0 */
);

void tcx_arith_render_envelope(
    const Word16 A_ind[],         /* i: LPC coefficients of signal envelope        */
    Word16 L_frame,               /* i: number of spectral lines                   */
    Word16 L_spec,
    Word16 preemph_fac,         /* i: pre-emphasis factor                        */
    Word16 gamma_w,             /* i: A_ind -> weighted envelope factor          */
    Word16 gamma_uw,            /* i: A_ind -> non-weighted envelope factor      */
    Word32 env[]                /* o: shaped signal envelope                     */
);

void tcx_get_gain(Word32 *x,        /* i: spectrum 1 */
                  Word16 x_e,       /* i: spectrum 1 exponent */
                  Word32 *y,        /* i: spectrum 2 */
                  Word16 y_e,       /* i: spectrum 2 exponent */
                  Word16 n,         /* i: length */
                  Word16 *gain,     /* o: gain */
                  Word16 *gain_e,   /* o: gain exponent */
                  Word32 *en_y,     /* o: energy of y (optional) */
                  Word16 *en_y_e    /* o: energy of y exponent (optional) */
                 );

void init_TCX_config(TCX_config *tcx_cfg,
                     Word16 L_frame,
                     Word16 fscale
                     ,Word16 L_frameTCX
                     ,Word16 fscaleFB
                    );

void tcx_arith_encode_envelope(
    Word32 spectrum[],                      /* i/o: MDCT coefficients           Q31-e */
    Word16 *spectrum_e,                     /* i/o: MDCT exponent               Q0 */
    Word16 signs[],                         /* o: signs (spectrum[.]<0)         Q0 */
    Word16 L_frame,                         /* i: frame or MDCT length          Q0 */
    Word16 L_frame_orig,                    /* i: length w/o BW limitation      Q0 */
    Encoder_State_fx *st,                   /* i/o: coder state                 */
    const Word16 A_ind[],                   /* i: quantised LPC coefficients    Q12 */
    Word16 target_bits,                     /* i: number of available bits      Q0 */
    Word16 prm[],                           /* o: bitstream parameters          Q0 */
    Word8 use_hm,                           /* i: use HM in current frame?      */
    Word16 prm_hm[],                        /* o: HM parameter area             Q0 */
    Word16 tcxltp_pitch,                    /* i: TCX LTP pitch in FD, -1 if n/a  Q0*/
    Word16 *arith_bits,                     /* o: bits used for ari. coding     Q0 */
    Word16 *signaling_bits,                 /* o: bits used for signaling       Q0 */
    Word16 *nf_seed                         /* o: noise filling seed            Q0 */
    ,Word16 low_complexity                  /* i: low-complexity flag           Q0 */
);
void tcx_arith_decode_envelope(
    Word32 q_spectrum[],                    /* o: quantised MDCT coefficients     Q31-e */
    Word16 *q_spectrum_e,                   /* o: MDCT exponent                   Q0 */
    Word16 L_frame,                            /* i: frame or MDCT length          */
    Word16 L_frame_orig,                       /* i: length w/o BW limitation      */
    Decoder_State_fx *st,
    const Word16 A_ind[],                    /* i: quantised LPC coefficients    */
    Word16 target_bits,                        /* i: number of available bits      */
    Word16 prm[],                              /* i: bitstream parameters          */
    Word8 use_hm,                             /* i: use HM in current frame?      */
    Word16 prm_hm[],                           /* i: HM parameter area             */
    Word16 tcxltp_pitch,                     /* i: TCX LTP pitch in FD, -1 if n/a*/
    Word16 *arith_bits,                        /* o: bits used for ari. coding     */
    Word16 *signaling_bits,                    /* o: bits used for signaling       */
    Word16 *nf_seed                         /* o: noise filling seed              Q0 */
    ,Word16 low_complexity                  /* i: low-complexity flag           Q0 */

);

#    define GET_ADJ2(T,L,F) (((L) << (F)) - (T))

void tcx_hm_render(
    Word32 lag,           /* i: pitch lag                         Q0  */
    Word16 fract_res,     /* i: fractional resolution of the lag  Q0  */
    Word16 p[]            /* o: harmonic model                    Q13 */
);

void tcx_hm_modify_envelope(
    Word16 gain,          /* i:   HM gain                           Q11 */
    Word32 lag,           /* i:   pitch lag                         Q0  */
    Word16 fract_res,     /* i:   fractional resolution of the lag  Q0  */
    Word16 p[],           /* i:   harmonic model                    Q13 */
    Word32 env[],         /* i/o: envelope                          Q16 */
    Word16 L_frame        /* i:   number of spectral lines          Q0  */
);

void UnmapIndex(
    Word16 PeriodicityIndex,
    Word16  Bandwidth,
    Word16 LtpPitchLag,
    Word8  SmallerLags,
    Word16 *FractionalResolution,
    Word32 *Lag);

/* Returns: PeriodicityIndex */
Word16 SearchPeriodicityIndex(
    const Word32 Mdct[],                /* (I) Coefficients, Mdct[0..NumCoeffs-1]                      */
    const Word32 UnfilteredMdct[],      /* (I) Unfiltered coefficients, UnfilteredMdct[0..NumCoeffs-1] */
    Word16 NumCoeffs,                   /* (I) Number of coefficients                                  */
    Word16 TargetBits,                  /* (I) Target bit budget (excl. Done flag)                     */
    Word16 LtpPitchLag,
    Word16 LtpGain,                     /* (I) LTP gain                                                */
    Word16 *RelativeScore               /* (O) Energy concentration factor                             */
);

void ConfigureContextHm(
    Word16 NumCoeffs,                   /* (I) Number of coefficients                         */
    Word16 TargetBits,                  /* (I) Target bit budget (excl. Done flag)            */
    Word16 PeriodicityIndex,            /* (I) Pitch related index                            */
    Word16 LtpPitchLag,                 /* (I) TCX-LTP pitch in F.D.                          */
    CONTEXT_HM_CONFIG *hm_cfg           /* (O) Context-based harmonic model configuration     */
);

Word16 EncodeIndex(
    Word16 Bandwidth,   /* 0: NB, 1: (S)WB */
    Word16 PeriodicityIndex,
    Encoder_State_fx *st);

Word16 CountIndexBits(
    Word16 Bandwidth,                  /* 0: NB, 1: (S)WB */
    Word16 PeriodicityIndex);

Word16
DecodeIndex(
    Decoder_State_fx *st,
    Word16 Bandwidth,
    Word16 *PeriodicityIndex);


void tcx_hm_analyse(
    const Word32 abs_spectrum[], /* i:   absolute spectrum            Q31-e */
    Word16 *spectrum_e,          /* i:   absolute spectrum exponent   Q0    */
    Word16 L_frame,              /* i:   number of spectral lines     Q0    */
    Word32 env[],                /* i/o: envelope shape               Q16   */
    Word16 targetBits,           /* i:   target bit budget            Q0    */
    Word16 coder_type,            /* i:  coder type                   Q0    */
    Word16 prm_hm[],             /* o:   HM parameters                Q0    */
    Word16 LtpPitchLag,          /* i:   LTP pitch lag or -1 if none  Q0    */
    Word16 LtpGain,              /* i:   LTP gain                     Q15   */
    Word16 *hm_bits              /* o:   bit consumption              Q0    */
);

void tcx_hm_decode(
    Word16 L_frame,              /* i:   number of spectral lines     Q0    */
    Word32 env[],                /* i/o: envelope shape               Q16   */
    Word16 targetBits,           /* i:   target bit budget            Q0    */
    Word16 coder_type,      /* i: coder_type                    Q0  */
    Word16 prm_hm[],             /* i:   HM parameters                Q0    */
    Word16 LtpPitchLag,          /* i:   LTP pitch lag or -1 if none  Q0    */
    Word16 *hm_bits              /* o:   bit consumption              Q0    */
);

void coder_tcx(
    Word16 n,
    TCX_config *tcx_cfg,    /*input: configuration of TCX*/
    Word16 A[],             /* input: quantized coefficients NxAz_q[M+1] */
    Word16 Aqind[],         /* input: frame-independent quantized coefficients (M+1) */
    Word16 synth[],
    Word16 L_frame_glob,    /* input: frame length             */
    Word16 L_frameTCX_glob,
    Word16 L_spec,
    Word16 nb_bits,         /*input: bit budget*/
    Word8 tcxonly,          /*input: only TCX flag*/
    Word32 spectrum[],      /* i/o: MDCT spectrum */
    Word16 *spectrum_e,     /* i/o: MDCT spectrum exponent               */
    LPD_state *LPDmem,      /*i/o: memories*/
    Word16 prm[],           /* output: tcx parameters          */
    Encoder_State_fx *st,
    CONTEXT_HM_CONFIG *hm_cfg
);

void coder_tcx_post(Encoder_State_fx *st,
                    LPD_state *LPDmem,
                    TCX_config *tcx_cfg,
                    Word16 *synth,
                    const Word16 *A,
                    const Word16 *Ai,
                    Word16 *wsig,
                    Word16 Q_new,
                    Word16 shift
                   );

void decoder_tcx(
    TCX_config *tcx_cfg, /* input: configuration of TCX         */
    Word16 prm[],        /* input:  parameters                  */
    Word16 A[],          /* input:  coefficients NxAz[M+1]      */
    Word16 Aind[],       /* input: frame-independent coefficients Az[M+1] */
    Word16 L_frame_glob, /* input:  frame length                */
    Word16 L_frameTCX,
    Word16 L_spec,
    Word16 synth[],      /* in/out: synth[-M-LFAC..lg]          */
    Word16 synthFB[],
    Decoder_State_fx*st,
    Word16 coder_type,   /* input : coder type                  */
    Word16 bfi,          /* input:  Bad frame indicator         */
    Word16 frame_cnt,    /* input: frame counter in the super_frame */
    Word16 stab_fac,     /* input: stability of isf             */
    Word16 past_core_mode
);

void decoder_tcx_post(Decoder_State_fx *st_fx,
                      Word16 *synth,
                      Word16 *synthFB,
                      Word16 *A,
                      Word16 bfi
                     );

/* cod_ace.c */
Word16 coder_acelp(      /* output SEGSNR for CL decision   */
    ACELP_config *acelp_cfg,     /*input/output: configuration of the ACELP coding*/
    const Word16 coder_type,     /* input: coding type              */
    const Word16 A[],            /* input: coefficients 4xAz[M+1]   */
    const Word16 Aq[],           /* input: coefficients 4xAz_q[M+1] */
    Word16 speech[],             /* input: speech[-M..lg]           */
    Word16 synth[],
    LPD_state *LPDmem,
    const Word16 voicing[],      /* input: open-loop LTP gain       */
    const Word16 T_op[],         /* input: open-loop LTP lag        */
    Word16 *prm,                 /* output: acelp parameters        */
    Word16 stab_fac,
    Encoder_State_fx *st,
    HANDLE_PLC_ENC_EVS hPlc_Ext,
    Word16 target_bits,          /* i/o : coder memory state        */
    Word16 Q_new,
    Word16 shift,
    Word16 *pitch_buf,           /* output : pitch values for each subfr.*/
    Word16 *voice_factors,       /* output : voicing factors             */
    Word16 *bwe_exc              /* output : excitation for SWB TBE      */
);


void decoder_acelp(
    Decoder_State_fx *st,
    Word16 coder_type,          /* input: coder type               */
    Word16 prm[],               /* input: parameters               */
    Word16 A[],                 /* input: coefficients NxAz[M+1]   */
    ACELP_config acelp_cfg,     /* input: ACELP config             */
    Word16 synth[],             /* i/o:   synth[-2*LFAC..L_DIV]              Q0   */
    Word16 *pT,                 /* out:   pitch for all subframe              Q0  */
    Word16 *pgainT,             /* out:   pitch gain for all subfr          1Q14  */
    Word16 stab_fac,            /* input: stability of isf         */
    Word16 *pitch_buffer,       /* out:   pitch values for each subfr.*/
    Word16 *voice_factors,      /* out:   voicing factors             */
    Word16 *bwe_exc             /* out:   excitation for SWB TBE      */
);

void enc_prm(
    const Word16 coder_type,    /* (i) : coding type                      */
    Word16 param[],             /* (i) : parameters                       */
    Word16 param_lpc[],         /* (i) : LPC parameters                   */
    Encoder_State_fx *st,       /* io: quantization Analysis values    */
    Word16 L_Frame,
    CONTEXT_HM_CONFIG hm_cfg[],
    Word16 * bits_param_lpc,
    Word16 no_param_lpc
);

void enc_prm_rf(
    Encoder_State_fx *st,
    const Word16 rf_frame_type,
    const Word16 fec_offset
);

void dec_prm(
    Word16 *core,               /* (0) : current frame mode */
    Word16 *last_core,          /* (0) : last frame mode */
    Word16 *coder_type,
    Word16 param[],             /* (o) : decoded parameters               */
    Word16 param_lpc[],         /* (o) : LPC parameters                   */
    Word16 *total_nbbits,       /* i/o : number of bits / decoded bits    */
    Decoder_State_fx *st,
    Word16 L_frame,
    Word16 *bitsRead
);

void dec_prm_core(
    Decoder_State_fx *st
);

void analy_sp(                  /* o:  fft0 bin energy                        Q0                 */
    Word16 *speech,             /* i:  speech buffer                          Q_new - preemph_bits */
    const Word16 Q_new,         /* i:  current scaling exp                    Q0                 */
    Word32 *fr_bands,           /* o:  energy in critical frequency bands     Q_new + QSCALE    */
    Word32 *lf_E,               /* o:  per bin E for first...                 Q_new + QSCALE - 2*/
    Word16 *Etot,               /* o:  total input energy                     Q8                 */
    const Word16 min_band,      /* i:  minimum critical band                  Q0                 */
    const Word16 max_band,      /* i:  maximum critical band                  Q0                 */
    const Word32 e_min_scaled,  /* i:  minimum energy scaled                  Q_new + QSCALE    */
    Word16 Scale_fac[2],        /* o:  FFT scales factors (2 values by frame) Q0                 */
    Word32 *Bin_E,              /* o  : per bin log energy spectrum                      */
    Word32 *Bin_E_old,          /* o  : per bin log energy spectrum for mid-frame        */
    Word32 *PS,                 /* o  : Per bin energy spectrum                          */
    Word16 *EspecdB,            /* o  : log E spectrum (with f=0) of the current frame Q7       */
    Word32 *band_energies,      /* o  : energy in critical frequency bands without minimum noise floor MODE2_E_MIN */
    Word16 *fft_buff            /* o  : FFT coefficients                                         */
);

void E_UTIL_copy_scale_sig(
    const Word16 x[],          /* i  : signal to scale input           Qx        */
    Word16 y[],                /* o  : scaled signal output            Qx        */
    const Word16 lg,           /* i  : size of x[]                     Q0        */
    const Word16 exp0          /* i  : exponent: x = round(x << exp)   Qx xx exp  */
);

void scale_sig32(
    Word32 x[],                 /* i/o: signal to scale                 Qx        */
    const Word16 lg,            /* i  : size of x[]                     Q0        */
    const Word16 exp0           /* i  : exponent: x = round(x << exp)   Qx xx exp  */
);


Word32 scale_mem(               /* o  : Min energy scaled           */
    Word16 ini_frames,          /* i  : Frame number                */
    Word16 Q_exp,               /* i  : Diff scaling factor         */
    Word16 *Q_new,              /* i/o: Absolute scaling factor     */
    Word16 *old_speech,         /* i/o: Speech memory               */
    Word16 *old_speech_ns,      /* i/o: Speech denoised memory      */
    Word16 *mem_wsp,            /* i/o: wsp vector memory           */
    Word32 *enrO,               /* i/o: Enr mem                     */
    Word32 *bckr,               /* i/o: Back ground_fx ener mem     */
    Word32 *lf_EO,              /* i/o: Energy per low freq bin mem */
    Word32 *ave_enr,            /* i/o: Ave_enr mem                 */
    Word32 *st_fr_bands1,       /* i/o: spectrum per critical bands of the previous frame  */
    Word32 *st_fr_bands2,       /* i/o: spectrum per critical bands 2 frames ago           */
    Word32 *ave_enr2            /* i/o: LT average E per crit. band (for non_sta2)  Q_new + QSCALE  */
);

Word16 rescale_exc(
    Word16 exc[],               /* i/o: excitation to rescale           Q_exc */
    Word16 lg,                  /* i  : frame size                            */
    Word32 L_gain_code,         /* i  : decoded codebook gain           Q16   */
    Word16 *sQ_exc,             /* i/o: Excitation scaling factor             */
    Word16 *sQsubfr,            /* i/o: Past excitation scaling factors       */
    Word16 exc2[],              /* o  : local excitation vector               */
    Word16 i_subfr              /* i  : subframe number                       */
);

Word16 rescale_mem(
    const Word16 *Q_exc,        /* i    : current excitation scaling (>=0)          */
    Word16 *prev_Q_syn,         /* i/o  : scaling factor of previous frame          */
    Word16 *Q_syn,              /* i/o  : scaling factor of frame                   */
    Word16 *mem_syn2,           /* i/o  : modified synthesis memory                 */
    Word16 *syn,                /* i/o  : synthesis  to rescale           Q_syn     */
    Word16  mem_len,            /* i    : lenght of modified synthesis memory       */
    Word16  i_subfr             /* i  : subframe number                       */
);

void gauss_L2(
    const Word16 h[],           /* i  : weighted LP filter impulse response   Q14+s */
    Word16 code2[],             /* o  : gaussian excitation                     Q9  */
    const Word16 y2[],          /* i  : zero-memory filtered code. excitation   Q9  */
    Word16 y11[],               /* o  : zero-memory filtered gauss. excitation  Q9  */
    Word32 *gain,               /* o  : excitation gain                             */
    ACELP_CbkCorr *g_corr,      /*i/o : correlation structure for gain coding       */
    const Word16 gain_pit,      /* i  : unquantized gain of code                    */
    const Word16 tilt_code,     /* i  : tilt of code                            Q15 */
    const Word16 *Aq,           /* i  : quantized LPCs                          Q12 */
    const Word16 formant_enh,   /* i  : formant enhancement factor              Q15 */
    Word16 *seed_acelp,         /*i/o : random seed                             Q0  */
    const Word16 shift
);

void gaus_L2_dec(
    Word16 *code,               /* o  : decoded gaussian codevector     Q9  */
    Word16 tilt_code,           /* i  : tilt of code                    Q15 */
    const Word16 *A,            /* i  : quantized LPCs                  Q12 */
    Word16 formant_enh,         /* i  : formant enhancement factor      Q15 */
    Word16 *seed_acelp          /*i/o : random seed                     Q0  */
);

void tcx_ltp_get_lpc(
    Word16 *x,
    Word16 L,
    Word16 *A,
    Word16 order
);

void predict_signal(
    const Word16 excI[],        /* i  : input excitation buffer  */
    Word16 excO[],              /* o  : output excitation buffer */
    const Word16 T0,            /* i  : integer pitch lag        */
    Word16 frac,                /* i  : fraction of lag          */
    const Word16 frac_max,      /* i  : max fraction             */
    const Word16 L_subfr        /* i  : subframe size            */
);

void tcx_ltp_pitch_search(
    Word16 pitch_ol,
    Word16 *pitch_int,
    Word16 *pitch_fr,
    Word16 *index,
    Word16 *norm_corr,
    const Word16 len,
    Word16 *wsp,
    Word16 pitmin,
    Word16 pitfr1,
    Word16 pitfr2,
    Word16 pitmax,
    Word16 pitres
);

void tcx_ltp_encode( Word8 tcxltp_on,
                     Word8 tcxOnly,
                     Word16 tcxMode,
                     Word16 L_frame,
                     Word16 L_subfr,
                     Word16 *speech,
                     Word16 *speech_ltp,
                     Word16 *wsp,
                     Word16 Top,
                     Word16 *ltp_param,
                     Word16 *ltp_bits,
                     Word16 *pitch_int,
                     Word16 *pitch_fr,
                     Word16 *gain,
                     Word16 *pitch_int_past,
                     Word16 *pitch_fr_past,
                     Word16 *gain_past,
                     Word16 *norm_corr_past,
                     Word16 last_core,
                     Word16 pitmin,
                     Word16 pitfr1,
                     Word16 pitfr2,
                     Word16 pitmax,
                     Word16 pitres,
                     struct TransientDetection const * pTransientDetection,
                     Word8 SideInfoOnly,
                     Word16 *A,
                     Word16 lpcorder
                   );

void tcx_ltp_post( Word8 tcxltp_on,
                   Word16 core,
                   Word16 L_frame,
                   Word16 L_frame_core,
                   Word16 delay,
                   Word16 *sig,
                   Word16 *tcx_buf,
                   Word16 tcx_buf_len,
                   Word16 bfi,
                   Word16 pitch_int,
                   Word16 pitch_fr,
                   Word16 gain,
                   Word16 *pitch_int_past,
                   Word16 *pitch_fr_past,
                   Word16 *gain_past,
                   Word16 *filtIdx_past,
                   Word16 pitres,
                   Word16 *pitres_past,
                   Word16 damping,
                   Word16 SideInfoOnly,
                   Word16 *mem_in,
                   Word16 *mem_out,
                   Word32 bitrate
                 );

void tcx_ltp_decode_params( Word16 *ltp_param,
                            Word16 *pitch_int,
                            Word16 *pitch_fr,
                            Word16 *gain,
                            Word16 pitmin,
                            Word16 pitfr1,
                            Word16 pitfr2,
                            Word16 pitmax,
                            Word16 pitres
                          );

/*isf_msvq_ma.c, lsf_msvq_ma.c*/
void msvq_enc
(
    const Word16 *const*cb,      /* i  : Codebook (indexed cb[*stages][levels][p])         (0Q15)     */
    const Word16 dims[],         /* i  : Dimension of each codebook stage (NULL: full dim.)                     */
    const Word16 offs[],         /* i  : Starting dimension of each codebook stage (NULL: 0)                    */
    const Word16 u[],       /* i  : Vector to be encoded (prediction and mean removed)(14Q1*1.28)   */
    const Word16 *levels,   /* i  : Number of levels in each stage                               */
    const Word16 maxC,      /* i  : Tree search size (number of candidates kept from             */
    /*      one stage to the next == M-best)                             */
    const Word16 stages,    /* i  : Number of stages                                             */
    const Word16 w[],       /* i  : Weights                                           Q8        */
    const Word16 N,         /* i  : Vector dimension                                             */
    const Word16 maxN,      /* i  : Codebook dimension                                           */
    Word16 Idx[]      /* o  : Indices                                                      */
);
void msvq_dec
(
    const Word16 *const*cb,      /* i  : Codebook (indexed cb[*stages][levels][p])     (0Q15) */
    const Word16 dims[],         /* i  : Dimension of each codebook stage (NULL: full dim.)   */
    const Word16 offs[],         /* i  : Starting dimension of each codebook stage (NULL: 0)  */
    const Word16 stages,    /* i  : Number of stages                                     */
    const Word16 N,         /* i  : Vector dimension                                     */
    const Word16 maxN,      /* i  : Codebook dimension                                   */
    const Word16 Idx[],     /* i  : Indices                                              */
    Word16 *uq        /* o  : quantized vector                              (3Q12) */
);


/*-------------------------------------------------------------------------
* Recovery tools for resynchronization
*------------------------------------------------------------------------*/
/*er_sync_exc.c*/
/** Resynchronize glotal pulse positions of the signal in src_exc and store it in dst_exc.
 * src_exc holds on call the harmonic part of the signal with the constant pitch, constructed by repeating the last pitch cycle of length pitchStart.
 * dst_exc holds on return the harmonic part of the signal with the pitch changing from pitchStart to pitchEnd.
 * src_exc and dst_exc can overlap, but src_exc < dst_exc must be fullfiled.
 * @param src_exc Input excitation buffer.
 * @param dst_exc Output excitation buffer.
 * @param nFrameLength Length of the frame, that is the length of the valid data in the excitation buffer on return.
 * @param nSubframes Number of subframes in the excitation buffer. nFrameLength must be divisible by nSubframes.
 * @param pitchStart Pitch at the end of the last frame.
 * @param pitchEnd Pitch at the end of the current frame.
 */
void PulseResynchronization(
    Word16 /*float*/    const   *   const   src_exc,        /*i   Q15*/
    Word16 /*float*/            *   const   dst_exc,        /*o   Q15*/
    Word16 /*int*/      const               nFrameLength,   /*i   Q0 */
    Word16 /*int*/      const               nSubframes,     /*i   Q0 */
    Word32 /*float*/    const               pitchStart,     /*i   Q16*/
    Word32 /*float*/    const               pitchEnd        /*i   Q16*/
);

/* er_util.c*/
/* PLC: [TCX: TD PLC]
 * PLC: FindSubframePitchInTCX: function to find the pitch per half a frame for the last TCX */
/*VERSIONINFO: This port is up to date with trunk rev. 30479*/
/*VERSIONINFO: This port is up to date with trunk rev. 7611*/

void con_acelp(
    const   Word16 A[],                   /*<! i     Q12: coefficients NxAz[M+1]                                  */
    const   Word16 /*int*/coder_type,     /*<! i     Q0 : ACELP coder type                                        */
    Word16 synth[],               /*<! i/o   Qx :   synthesis buffer,                 Headroom: 3 bits?   */
    Word16 /*int*/ *pT,           /*<! o     Q0 :   pitch for all subframe   [4]pit_min..pit_max          */
    Word16 *pgainT,               /*<! o     Q14:   pitch gain for all subfr [4] 0...1.3                  */
    const   Word16 stab_fac,              /*<! i     Q15: stability of isf                                        */
    Decoder_State_fx *st,
    const   Word16 *Qf_exc,               /*<! i       :  Q format for excitation buffer                          */
    Word16 *Qf_mem_syn            /*<! i/o     :  Q format for st->mem_syn            >rescaling done     */
    , Word16 *pitch_buffer
    , Word16 *voice_factors
    , Word16 *bwe_exc
);

void con_tcx(
    Word16 /*int*/   coder_type,            /* input: ACELP coder type          *//*Q0 */
    Word16 /*float*/ synth[],               /* i/o:   synth[]                   *//*Q0 */
    Word16 /*float*/ stab_fac,              /* input: stability of isf          *//*Q15*/
    Decoder_State_fx *st
);


/* isf_msvq_ma.c */
void isf_msvq_ma_enc(const Word16 *isf,        /* i: isf coefficients               (3Q12) */
                     Word16 *isfq,       /* o: quantized isf                         */
                     Word16 *isfq_ind,
                     Word16 *indices,    /* o: quantizer indices                     */
                     Word16 *nb_indices, /* o: number of bits for the indices        */
                     Word16 *mem_MA,     /* i/o: moving average filter buffer (3Q12) */
                     Word16 *parcorr,
                     const Word16 m,           /* i: lpc order                             */
                     const Word16 acelp_mode,  /* i: acelp mode                            */
                     const Word8  narrow_band  /* i: narrow-band flag                      */
                    );
void isf_msvq_ma_dec( Word16 *isfq,
                      Word16 *isfq_ind,
                      Word16 *indices,
                      Word16 *nb_indices,
                      Word16 *mem_MA,
                      Word16 m,
                      Word16 acelp_mode,
                      Word8  narrow_band       /* i: narrow-band flag                      */
                    );
Word16 isf_msvq_ma_encprm( Encoder_State_fx *st, Word16 *param_lpc, Word16 core, Word16 acelp_mode, Word16 acelp_midLpc, Word16 m );
Word16 isf_msvq_ma_decprm( Decoder_State_fx *st, Word16 *param_lpc, Word16 core, Word16 acelp_mode, Word16 acelp_midLpc, Word16 m );

/* Returns: non-zero on conversion error, 0 otherwise */
Word16 E_LPC_lsp_unweight(
    /* const */ Word16 xsp_w[], /* (I): weighted xSP             */
    Word16 xsp_uw[],            /* (O): unweighted xSP           */
    Word16 xsf_uw[],            /* (O): unweighted xSF           */
    Word16 inv_gamma,           /* (I): inverse weighting factor */
    Word16 lpcorder               /* (I): prediction order         */
);

Word16 lsf_ind_is_active(
    const Word16 lsf_q_ind[],
    const Word16 means[],
    Word16 bandwidth,
    Word16 cdk);

void lsf_update_memory(
    Word16 narrowband,         /* i  : narrowband flag                             */
    const Word16 qisf[],       /* i  : quantized xSF coefficients                  */
    Word16 old_mem_MA[],       /* i  : MA memory                                   */
    Word16 mem_MA[],           /* o  : updated MA memory                           */
    Word16 lpcorder              /* i  : LPC order                                   */
);

/* Returns: number of indices */
Word16 Q_lsf_tcxlpc(
    /* const */ Word16 xsf[],     /* (I) original xSF      */
    Word16 xsf_q[],               /* (O) quantized xSF     */
    Word16 xsf_q_ind[],           /* (O) quantized xSF (w/o MA prediction) */
    Word16 indices[],             /* (O) VQ indices        */
    Word16 lpcorder,              /* (I) LPC order         */
    Word16 narrowband,            /* (I) narrowband flag   */
    Word16 cdk,                   /* (I) codebook selector */
    Word16 mem_MA[],              /* (I) MA memory         */
    Word16 coder_type,
    Word32 * Bin_Ener,
    const Word16 Q_ener
);

/* Returns: number of indices */
Word16 D_lsf_tcxlpc(
    const Word16 indices[],       /* (I) VQ indices        */
    Word16 xsf_q[],               /* (O) quantized xSF     */
    Word16 xsf_q_ind[],           /* (O) quantized xSF (w/o MA prediction) */
    Word16 narrowband,            /* (I) narrowband flag   */
    Word16 cdk,                   /* (I) codebook selector */
    Word16 mem_MA[]               /* (I) MA memory         */
);

/* Returns: number of bits written */
Word16 enc_lsf_tcxlpc(
    Word16 **indices,             /* (I) Ptr to VQ indices */
    Encoder_State_fx *st          /* (I/O) Encoder state   */
);

/* Returns: number of bits read */
Word16 dec_lsf_tcxlpc(
    Decoder_State_fx *st,         /* (I/O) Decoder state   */
    Word16 **indices,             /* (O) Ptr to VQ indices */
    Word16 narrowband,            /* (I) narrowband flag   */
    Word16 cdk                    /* (I) codebook selector */
);

/* Returns: codebook index */
Word16 tcxlpc_get_cdk(
    Word16 coder_type             /* (I) VOICED indicator  */
);

/* lsf_msvq_ma.c */
void lsf_msvq_ma_enc(
    const Word16 *lsf,      /* i: lsf coefficients         (14Q1*1.28)*/
    Word16 *lsfq,        /* o: quantized lsf                  */
    Word16 *lsfq_ind,
    Word32 * Bin_Ener_128_fx,
    const Word16 Q_ener,
    Word16 *indices,    /* o: quantizer indices                */
    Word16 *nb_indices,  /* o: number of bits for the indices        */
    Word16 *mem_MA,    /* i/o: moving average filter buffer (3Q12*1.28)*/
    const Word16 m,          /* i: lpc order                    */
    const Word16 acelp_mode   /* i: acelp mode                  */
);

void lsf_msvq_ma_dec( Word16 *lsfq,
                      Word16 *lsfq_ind,
                      Word16 *indices,
                      Word16 *nb_indices,
                      Word16 *mem_MA,
                      Word16 m,
                      Word16 acelp_mode
                    );

Word16 lsf_msvq_ma_encprm( Encoder_State_fx * st,
                           Word16 *param_lpc,
                           Word16 core,
                           Word16 acelp_mode,
                           Word16 acelp_midLpc,
                           Word16 * bits_param_lpc,
                           Word16 no_indices
                         );
Word16 lsf_msvq_ma_decprm( Decoder_State_fx *st, Word16 *param_lpc, Word16 core, Word16 acelp_mode, Word16 acelp_midLpc
                           ,Word16 narrowBand
                           ,Word32 sr_core
                         );

Word16 lsf_bctcvq_encprm(
    Encoder_State_fx *st,
    Word16 *param_lpc,
    Word16 * bits_param_lpc,
    Word16 no_indices
);

Word16 lsf_bctcvq_decprm(
    Decoder_State_fx * st,
    Word16 *param_lpc
);
void lpc_unquantize(
    Decoder_State_fx * st,
    Word16 *lsfold,
    Word16 *lspold,
    Word16 *lsf,
    Word16 *lsp,
    const Word16 m,
    const Word16 lpcQuantization,
    Word16 *param_lpc,
    const Word16 numlpc,
    const Word16 core,
    Word16 *mem_MA,
    Word16 *mem_AR,
    Word16 *lspmid,
    Word16 *lsfmid,
    Word16 coder_type,
    Word16 acelp_midLpc,
    Word8  narrow_band,
    Word16 *seed_acelp,
    Word32 sr_core,
    Word16 *mid_lsf_int,
    Word16 prev_bfi,
    Word16 *safety_net
);

void midlsf_enc(const Word16 qlsf0[],  /* i: quantized lsf coefficients (3Q12)  */
                const Word16 qlsf1[],  /* i: quantized lsf coefficients (3Q12)  */
                const Word16 lsf[],    /* i: lsf coefficients           (3Q12)  */
                Word16 *idx,    /* o: codebook index          */
                const Word16 lpcorder   /* i: order of the lpc          */
                , Word32 * Bin_Ener_128_fx
                ,const Word16 Q_ener
                ,Word8 narrowBand
                ,Word32 sr_core
                ,Word16 coder_type
               );


void midlsf_dec(
    const Word16 qlsf0[],  /* i: quantized lsf coefficients (3Q12) */
    const Word16 qlsf1[],  /* i: quantized lsf coefficients (3Q12) */
    Word16 idx,        /* i: codebook index          */
    Word16 qlsf[],      /* o: decoded lsf coefficients   (3Q12) */
    Word16 coder_type,
    Word16 *mid_lsf_int,
    Word16 prev_bfi,
    Word16 safety_net);


Word16 const * PlcGetLsfBase (Word16   const lpcQuantization,
                              Word16   const narrowBand,
                              Word32   const sr_core);

/* tcx_mdct.h */
#define MDCT_A_E 1

#define TCX_IMDCT_SCALE 15
#define TCX_IMDCT_HEADROOM 1

void TCX_MDCT(const Word16 *x, Word32 *y, Word16 *y_e, Word16 l, Word16 m, Word16 r);

void TCX_MDST(const Word16 *x, Word32 *y, Word16 *y_e, Word16 l, Word16 m, Word16 r);

void TCX_MDCT_Inverse(Word32 *x, Word16 x_e, Word16 *y, Word16 l, Word16 m, Word16 r);


/* post_dec.h */
void post_decoder(
    Decoder_State_fx *st,
    Word16 coder_type,
    Word16 synth_buf[],
    Word16 pit_gain[],
    Word16 pitch[],
    Word16 signal_out[],
    Word16 * bpf_noise_buf
);

Word16 bass_pf_enc(
    Word16 *orig,               /* (i) : 12.8kHz original signal                      Q0 */
    Word16 *syn,                /* (i) : 12.8kHz synthesis to postfilter              Q0 */
    Word16 *T_sf,               /* (i) : Pitch period for all subframes (T_sf[16])    Q0 */
    Word16 *gainT_sf,           /* (i) : Pitch gain for all subframes (gainT_sf[16])  Q14 */
    Word16 l_frame,             /* (i) : frame length (should be multiple of l_subfr) Q0 */
    Word16 l_subfr,             /* (i) : sub-frame length (60/64)                     Q0 */
    Word16 *gain_factor_param,  /* (o) : quantized gain factor                        Q0 */
    Word16 mode,                /* (i) : coding mode of adapt bpf                        */
    struct MEM_BPF *mem_bpf     /* i/o : memory state                                    */
);

/* fd_cng_common.h */
/* Create an instance of type FD_CNG */
void createFdCngCom(HANDLE_FD_CNG_COM* hFdCngCom);

void initFdCngCom(HANDLE_FD_CNG_COM hFdCngCom, Word16 scale);

/* Delete the instance of type FD_CNG */
void deleteFdCngCom(HANDLE_FD_CNG_COM * hFdCngCom);

void resetFdCngEnc( Encoder_State_fx * st);

/* Initialize the spectral partitioning */
void initPartitions( const Word16* part_in,
                     Word16  npart_in,
                     Word16  startBand,
                     Word16  stopBand,
                     Word16* part_out,
                     Word16* npart_out,
                     Word16* midband,
                     Word16* psize,
                     Word16* psize_norm,
                     Word16* psize_norm_exp,
                     Word16* psize_inv,
                     Word16  stopBandFR);

/* Noise estimation using Minimum Statistics (MS) */
void compress_range(Word32 *in,
                    Word16  in_exp,
                    Word16 *out,
                    Word16  len
                   );

void expand_range(Word16 *in,
                  Word32 *out,
                  Word16 *out_exp,
                  Word16  len
                 );

void minimum_statistics(Word16  len,                    /* i  : Total number of partitions (CLDFB or FFT)                   */
                        Word16  lenFFT,                 /* i  : Number of FFT partitions                                  */
                        Word16 *psize,                  /* i  : Partition sizes, fractional                               */
                        Word16 *msPeriodog,             /* i  : Periodogram (energies)                                    */
                        Word16 *msNoiseFloor,           /* i/o: Noise floors (energies)                                   */
                        Word16 *msNoiseEst,             /* i/o: Noise estimates (energies)                                */
                        Word32 *msAlpha,                /* i/o: Forgetting factors                                        */
                        Word16 *msPsd,                  /* i/o: Power Spectral Density (smoothed periodogram => energies) */
                        Word16 *msPsdFirstMoment,       /* i/o: PSD statistics of 1st order (energy means)                */
                        Word32 *msPsdSecondMoment,      /* i/o: PSD statistics of 2nd order (energy variances)            */
                        Word32 *msMinBuf,               /* i/o: Buffer of minima (energies)                               */
                        Word32 *msBminWin,              /* o  : Bias correction factors                                   */
                        Word32 *msBminSubWin,           /* o  : Bias correction factors                                   */
                        Word32 *msCurrentMin,           /* i/o: Local minima (energies)                                   */
                        Word32 *msCurrentMinOut,        /* i/o: Local minima (energies)                                   */
                        Word32 *msCurrentMinSubWindow,  /* i/o: Local minima (energies)                                   */
                        Word16 *msLocalMinFlag,         /* i  : Binary flag                                               */
                        Word16 *msNewMinFlag,           /* i  : Binary flag                                               */
                        Word16 *msPeriodogBuf,          /* i/o: Buffer of periodograms (energies)                         */
                        Word16 *msPeriodogBufPtr,       /* i/o: Counter                                                   */
                        HANDLE_FD_CNG_COM st            /* i/o: FD_CNG structure containing buffers and variables         */
                       );

/* Generate the comfort noise based on the target noise level */
void generate_comfort_noise_enc (Encoder_State_fx *stcod,
                                 Word16 Q_new,
                                 Word16 gen_exc
                                );
void generate_comfort_noise_dec (Word32 **bufferReal,         /* o   : matrix to real part of input bands */
                                 Word32 **bufferImag,         /* o   : matrix to imaginary part of input bands */
                                 Word16  *bufferScale,        /* o   : pointer to scalefactor for real and imaginary part of input bands */
                                 Decoder_State_fx *stdec,
                                 Word16 *Q_new,
                                 Word16 gen_exc
                                );
void
generate_comfort_noise_dec_hf (Word32 **bufferReal,         /* o   : matrix to real part of input bands */
                               Word32 **bufferImag,         /* o   : matrix to imaginary part of input bands */
                               Word16  *bufferScale,        /* o   : pointer to scalefactor for real and imaginary part of input bands */
                               Decoder_State_fx *stdec
                              );

/* Generate the comfort noise based on the target noise level */
void generate_masking_noise (Word16 *timeDomainBuffer,        /* i/o: time-domain signal */
                             Word16 Q,
                             HANDLE_FD_CNG_COM st            /* i/o: FD_CNG structure containing all buffers and variables */
                             ,Word16 length
                             ,Word16 core
                            );

void generate_masking_noise_mdct (Word32 *mdctBuffer,       /* i/o: time-domain signal */
                                  Word16 *mdctBuffer_e,     /* i/o: exponent time-domain signal */
                                  HANDLE_FD_CNG_COM st      /* i/o: FD_CNG structure containing all buffers and variables */
                                  ,Word16 L_frame
                                 );

/* Apply bitrate-dependant scale */
void apply_scale(Word32 *scale,
                 Word16 bwmode,
                 Word32 bitrate
                );

/* Compute the power for each partition */
void bandcombinepow(Word32* bandpow,                           /* i  : Power for each band */
                    Word16  exp_bandpow,                       /* i  : exponent of bandpow */
                    Word16  nband,                             /* i  : Number of bands */
                    Word16* part,                              /* i  : Partition upper boundaries (band indices starting from 0) */
                    Word16  npart,                             /* i  : Number of partitions */
                    Word16* psize_inv,                         /* i  : Inverse partition sizes */
                    Word32* partpow,                           /* o  : Power for each partition */
                    Word16* exp_partpow
                   );

/* Scale partitions (with smoothing) */
void scalebands (Word32 *partpow,           /* i  : Power for each partition */
                 Word16 *part,              /* i  : Partition upper boundaries (band indices starting from 0) */
                 Word16  npart,             /* i  : Number of partitions */
                 Word16 *midband,           /* i  : Central band of each partition */
                 Word16  nFFTpart,          /* i  : Number of FFT partitions */
                 Word16  nband,             /* i  : Number of bands */
                 Word32 *bandpow,           /* o  : Power for each band */
                 Word16  flag_fft_en
                );

/* Get central band for each partition */
void getmidbands(Word16* part,              /* i  : Partition upper boundaries (band indices starting from 0) */
                 Word16  npart,             /* i  : Number of partitions */
                 Word16* midband,           /* o  : Central band of each partition */
                 Word16* psize,             /* o  : Partition sizes */
                 Word16* psize_norm,        /* o  : Partition sizes, fractional values */
                 Word16* psize_norm_exp,    /* o  : Exponent for fractional partition sizes */
                 Word16* psize_inv          /* o  : Inverse of partition sizes */
                );

/* STFT analysis filterbank */
void AnalysisSTFT (const Word16 *timeDomainInput, /* i  : pointer to time signal */
                   Word16 Q,
                   Word32 *fftBuffer,             /* o  : FFT bins */
                   Word16 *fftBuffer_exp,         /* i  : exponent of FFT bins */
                   HANDLE_FD_CNG_COM st          /* i/o: FD_CNG structure containing all buffers and variables */
                  );

/* STFT synthesis filterbank */
void SynthesisSTFT (Word32 *fftBuffer,           /* i    : pointer to FFT bins */
                    Word16  fftBufferExp,        /* i    : exponent of FFT bins */
                    Word16 *timeDomainOutput,    /* o    : pointer to time domain signal */
                    Word16 *olapBuffer,          /* i/o  : pointer to overlap buffer */
                    const PWord16 *olapWin,      /* i    : pointer to overlap window */
                    Word16 tcx_transition,
                    HANDLE_FD_CNG_COM st,       /* i/o  : pointer to FD_CNG structure containing all buffers and variables */
                    Word16 gen_exc,
                    Word16 *Q_new
                   );

/* Compute some calues used in the bias correction of the minimum statistics algorithm */
void mhvals(Word16 d, Word16 * m/*, float * h*/);

/* Random generator with Gaussian distribution with mean 0 and std 1 */
Word32 rand_gauss (Word16 *seed);

void lpc_from_spectrum (Word32 *powspec,
                        Word16  powspec_exp,
                        Word16  start,
                        Word16  stop,
                        Word16  fftlen,
                        Word16 *A,
                        Word16  lpcorder,
                        Word16  preemph_fac);

void msvq_decoder(const Word16 *const cb[], /* i  : Codebook (indexed cb[*stages][levels][p]) */
                  Word16 stages,     /* i  : Number of stages                          */
                  Word16 N,          /* i  : Vector dimension                          */
                  Word16 maxN,       /* i  : Codebook vector dimension                 */
                  Word16 Idx[],      /* i  : Indices                                   */
                  Word16 *uq         /* o  : quantized vector                          */
                 );

/*fd_cng_dec.h*/
/* Create an instance of type FD_CNG */
void createFdCngDec(HANDLE_FD_CNG_DEC* hFdCngDec);

Word16 initFdCngDec(HANDLE_FD_CNG_DEC hFdCngDec, Word16 scale);
extern void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m);
extern void E_LPC_f_isp_a_conversion(const Word16 *isp, Word16 *a, const Word16 m);

/* Delete the instance of type FD_CNG */
void deleteFdCngDec(HANDLE_FD_CNG_DEC * hFdCngDec);

/* Configure CLDFB-CNG */
void configureFdCngDec(HANDLE_FD_CNG_DEC hs,      /* i/o: Contains the variables related to the CLDFB-based CNG process */
                       Word16 bandwidth,
                       Word32 bitrate,
                       Word16 cng_type);


/* Apply the CLDFB-based CNG */
Word16 ApplyFdCng (Word16 * timeDomainInput,       /* i  : pointer to time domain input */
                   Word16 Q,
                   Word32 ** cldfbBufferReal,        /* i/o: real part of the CLDFB buffer */
                   Word32 ** cldfbBufferImag,        /* i/o: imaginary part of the CLDFB buffer */
                   Word16  * cldfbBufferScale,       /* o  : pointer to the scalefactor for real and imaginary part of the CLDFB buffer */
                   HANDLE_FD_CNG_DEC st,          /* i/o: pointer to FD_CNG structure containing all buffers and variables */
                   Word16 m_frame_type,            /* i  : type of frame at the decoder side */
                   Decoder_State_fx *stdec,
                   const Word16 concealWholeFrame  /* i  : binary flag indicating frame loss */
                   ,Word16 is_music
                  );

/* Perform noise estimation */
void perform_noise_estimation_dec(const Word16 *timeDomainInput, /* i:   pointer to time domain input */
                                  Word16 Q,
                                  HANDLE_FD_CNG_DEC st          /* i/o: FD_CNG structure containing all buffers and variables */
                                 );

/* Decode the CLDFB-CNG bitstream */
void FdCng_decodeSID(HANDLE_FD_CNG_COM st,         /* i/o: FD_CNG structure containing all buffers and variables */
                     Decoder_State_fx *corest);     /* i/o: decoder state structure */

void FdCng_exc(
    HANDLE_FD_CNG_COM hs,
    Word16 *CNG_mode,
    Word16 L_frame,
    Word16 *lsp_old,
    Word16 first_CNG,
    Word16 *lsp_CNG,
    Word16 *Aq,                    /* o:   LPC coeffs */
    Word16 *lsp_new,               /* o:   lsp  */
    Word16 *lsf_new,               /* o:   lsf  */
    Word16 *exc,                   /* o:   LP excitation   */
    Word16 *exc2,                  /* o:   LP excitation   */
    Word16 *bwe_exc                /* o:   LP excitation for BWE */
);

void noisy_speech_detection(
    const Word16 vad,
    const Word16 *ftimeInPtr,               /* i  : input time-domain frame                  */
    const Word16 frameSize,                 /* i  : frame size                               */
    const Word16 Q,
    const Word32 *msNoiseEst,               /* i  : noise estimate over all critical bands */
    const Word16 msNoiseEst_exp,            /* i  : exponent for noise estimate over all critical bands */
    const Word16 *psize_norm,
    const Word16 psize_norm_exp,
    const Word16 nFFTpart,                  /* i  : Number of partitions taken into account  */
    Word32 *lp_noise,                       /* i/o: pointer to long term total Noise energy average */
    Word32 *lp_speech,                      /* i/o: pointer to long term active speech energy average */
    Word16 *flag_noisy_speech
);

/* Create an instance of type FD_CNG */
void createFdCngEnc(HANDLE_FD_CNG_ENC* hFdCngEnc);

void initFdCngEnc(HANDLE_FD_CNG_ENC hsEnc, Word32 input_Fs, Word16 scale);

/* Delete the instance of type FD_CNG */
void deleteFdCngEnc(HANDLE_FD_CNG_ENC * hFdCngEnc);

/* Configure CLDFB-CNG */
void configureFdCngEnc(HANDLE_FD_CNG_ENC hs,   /* i/o: Contains the variables related to the CLDFB-based CNG process */
                       Word16 bandwidth,        /* i:   bandwidth */
                       Word32 bitrate);

/* Perform noise estimation */
void perform_noise_estimation_enc(
    Word32 *band_energies,                /* i: energy in critical bands without minimum noise floor MODE2_E_MIN */
    Word16 exp_band_energies,
    Word32 *enerBuffer,
    Word16 enerBuffer_exp,
    HANDLE_FD_CNG_ENC st);               /* i/o: FD_CNG structure containing all buffers and variables */

/* Adjust the noise estimator at the beginning of each CNG phase (encoder-side) */
Word16
AdjustFirstSID(Word16  npart,
               Word32 *msPeriodog,
               Word16  msPeriodog_exp,
               Word32 *energy_ho,
               Word16 *energy_ho_exp,
               Word32 *msNoiseEst,
               Word16 *msNoiseEst_exp,
               Word32 *msNoiseEst_old,
               Word16 *msNoiseEst_old_exp,
               Word16 *active_frame_counter,
               Encoder_State_fx *stcod
              );

/* Generate a bitstream out of the partition levels */
void FdCng_encodeSID (HANDLE_FD_CNG_ENC st,     /* i/o: FD_CNG structure containing all buffers and variables */
                      Encoder_State_fx *corest,
                      Word16  preemph_fac        /* i  : preemphase factor */
                     );

/*parameter_bitmaping.h*/
/** Writes parameters from paramsBitMap into a stream.
 * @param paramsBitMap Definition of parameters and mappings to a bitstream.
 * @param nParam Size of paramsBitMap.
 * @param pParameter A parameter to be used in get and set functions from paramsBitMap.
 * @param pStream Pointer to a stream where parameters should be stored.
 * @param pnSize Set to the number of elements written to the stream.
 * @param pnBits Set to the number of required bits.
 */
void GetParameters(ParamsBitMap const * paramsBitMap, Word16 nParams, void const * pParameter, Word16 ** pStream, Word16 * pnSize, Word16 * pnBits);

/** Reads parameters from a stream into paramsBitMap.
 * @param paramsBitMap Definition of parameters and mappings to a bitstream.
 * @param nParam Size of paramsBitMap.
 * @param pParameter A parameter to be used in get and set functions from paramsBitMap.
 * @param pStream Pointer to a stream from which parameters are read.
 * @param pnSize Set to the number of elements read from the stream.
 */
void SetParameters(ParamsBitMap const * paramsBitMap, Word16 nParams, void * pParameter, Word16 const ** pStream, Word16 * pnSize);

/** Writes parameters from a stream into a bitstream.
 * @param paramsBitMap Definition of parameters and mappings to a bitstream.
 * @param nParam Size of paramsBitMap.
 * @param pStream Pointer to a stream from which parameters are read.
 * @param pnSize Set to the number of elements read from the stream.
 * @param st Encoder state.
 * @param pnBits Set to the number of bits written.
 */
void WriteToBitstream(ParamsBitMap const * paramsBitMap, Word16 nParams, Word16 const ** pStream, Word16 * pnSize, Encoder_State_fx *st, Word16 * pnBits);

/** Reads parameters from a bitstream into a stream.
 * @param paramsBitMap Definition of parameters and mappings to a bitstream.
 * @param nArrayLength Size of paramsBitMap.
 * @param st Decoder state.
 * @param pStream Pointer to a stream where parameters should be stored.
 * @param pnSize Set to the number of elements written to the stream.
 */
void ReadFromBitstream(ParamsBitMap const * paramsBitMap, Word16 nArrayLength, Decoder_State_fx *st, Word16 ** pStream, Word16 * pnSize);

void const * GetTnsFilterOrder(void const * p, Word16 index, Word16 * pValue);
void * SetTnsFilterOrder(void * p, Word16 index, Word16 value);
void const * GetNumOfTnsFilters(void const * p, Word16 index, Word16 * pValue);
void * SetNumOfTnsFilters(void * p, Word16 index, Word16 value);
void const * GetTnsEnabled(void const * p, Word16 index, Word16 * pValue);
void * SetTnsEnabled(void * p, Word16 index, Word16 value);
void const * GetTnsEnabledSingleFilter(void const * p, Word16 index, Word16 * pValue);
void * SetTnsEnabledSingleFilter(void * p, Word16 index, Word16 value);
void const * GetTnsFilterCoeff(void const * p, Word16 index, Word16 * pValue);
void * SetTnsFilterCoeff(void * p, Word16 index, Word16 value);


Word16 GetSWBTCX10TnsFilterCoeffBits(Word16 value, Word16 index);
Word16 EncodeSWBTCX10TnsFilterCoeff(Word16 value, Word16 index);
Word16 DecodeSWBTCX10TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue);
Word16 GetSWBTCX20TnsFilterCoeffBits(Word16 value, Word16 index);
Word16 EncodeSWBTCX20TnsFilterCoeff(Word16 value, Word16 index);
Word16 DecodeSWBTCX20TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue);

Word16 GetWBTCX10TnsFilterCoeffBits(Word16 value, Word16 index);
Word16 EncodeWBTCX10TnsFilterCoeff(Word16 value, Word16 index);
Word16 DecodeWBTCX10TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue);
Word16 GetWBTCX20TnsFilterCoeffBits(Word16 value, Word16 index);
Word16 EncodeWBTCX20TnsFilterCoeff(Word16 value, Word16 index);
Word16 DecodeWBTCX20TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue);

Word16 GetTnsFilterOrderBitsSWBTCX10(Word16 value, Word16 index);
Word16 EncodeTnsFilterOrderSWBTCX10(Word16 value, Word16 index);
Word16 DecodeTnsFilterOrderSWBTCX10(Decoder_State_fx *st, Word16 index, Word16 * pValue);
Word16 GetTnsFilterOrderBitsSWBTCX20(Word16 value, Word16 index);
Word16 EncodeTnsFilterOrderSWBTCX20(Word16 value, Word16 index);
Word16 DecodeTnsFilterOrderSWBTCX20(Decoder_State_fx *st, Word16 index, Word16 * pValue);
Word16 GetTnsFilterOrderBits(Word16 value, Word16 index);
Word16 EncodeTnsFilterOrder(Word16 value, Word16 index);
Word16 DecodeTnsFilterOrder(Decoder_State_fx *st, Word16 index, Word16 * pValue);


/*tns_base.h*/
/** Reset TNS data.
 * Resets TNS data to the initial state.
 * @param pTnsData pointer to a TNS data to be reset.
 */
void ResetTnsData(STnsData * pTnsData);

/** Clear TNS filter data.
 * Resets TNS filter order and all coefficients to 0.
 * @param pTnsFilter pointer to a TNS filter to be cleared.
 */
void ClearTnsFilterCoefficients(STnsFilter * pTnsFilter);

/** Init TNS configuration.
 * Fills STnsConfig structure with sensible content.
 * @param nSampleRate Sampling rate of the input.
 * @param nFrameLength Frame length.
 * @param pTnsConfig TNS configuration to be initialized.
 * @return 0 on success, otherwise 1.
 */
Word16 InitTnsConfiguration(Word32 nSampleRate,
                            Word16 frameLength,
                            STnsConfig * pTnsConfig
                            ,Word16 igfStopFreq
                            ,Word32 bitrate
                           );

/** Detect TNS parameters.
 * Detects if TNS should be used and fills TNS data in pTnsData.
 * @param pTnsConfig TNS configuration.
 * @param pSpectrum Spectrum lines.
 TNS is tested on the spectrum defined by pSpectrum.
 * @param sbCnt Number of active subbands.
 * @param pTnsData TNS data is filled with sensible information.
 * @return Returns 1 if Tns should be used, otherwise 0.
 */
Word16 DetectTnsFilt(STnsConfig const * pTnsConfig,
                     Word32 const pSpectrum[],
                     STnsData * pTnsData,
                     Word16 *predictionGain);

/** Modify spectrum using TNS filter.
 * Modifies spectrum unsing TNS filter defined by pTnsData.
 * If fIsAnalyses is true considers spectrum to be
 * an input of encoder and returns residum.
 * If fIsAnalyses is false considers spectrum to be
 * a residum from decoder and returns output spectrum.
 * @param pTnsConfig TNS configuration.
 * @param pTnsData TNS data describing filters.
 * @param spectrum Input/output spectrum.
 * @param fIsAnalysis Defines if TNS filter is applied
 *   in encoder (TRUE) or in decoder (FALSE).
 * @return 0 on success, otherwise 1.
 */
Word16 ApplyTnsFilter(STnsConfig const * pTnsConfig,
                      STnsData const * pTnsData,
                      Word32 spectrum[],
                      Word8 fIsAnalysis);

Word16 ITF_Detect_fx(Word32 const pSpectrum[],
                     Word16 startLine,
                     Word16 stopLine,
                     Word16 maxOrder,
                     Word16* A,
                     Word16* Q_A,
                     Word16* predictionGain,
                     Word16* curr_order,
                     Word16 Q);

TNS_ERROR ITF_Apply_fx(Word32 spectrum[],
                       Word16 startLine, Word16 stopLine, const Word16* A,
                       Word16 Q_A,
                       Word16 curr_order);

/** Write TNS data into a stream of integers.
 * Writes TNS data into a stream of integers.
 * @param pTnsConfig TNS configuration.
 * @param pTnsData TNS data to be written to a bitstream.
 * @param stream Output stream of integers.
 * @param pnSize Set to the number of elements written to the stream.
 * @param pnBits Set to the number of required.
 * @return 0 on success, otherwise 1.
 */
Word16 EncodeTnsData(STnsConfig const * pTnsConfig,
                     STnsData const * pTnsData,
                     Word16 * stream,
                     Word16 * pnSize,
                     Word16 * pnBits);

/** Read TNS data from a stream of integers.
 * Reads TNS data from a stream of integers.
 * @param pTnsConfig TNS configuration.
 * @param stream Input stream of integers.
 * @param pnSize Set to the number of elements read from stream.
 * @param pTnsData TNS data to be read from the bitstream.
 * @return 1 if TNS is active, otherwise 0.
 */
Word16 DecodeTnsData(STnsConfig const * pTnsConfig,
                     Word16 const * stream,
                     Word16 * pnSize,
                     STnsData * pTnsData);

/** Write TNS data into a bitstream.
 * Writes TNS data into a bitstream.
 * @param pTnsConfig TNS configuration.
 * @param stream Contains TNS data written with EncodeTnsData.
 * @param pnSize Set to the number of used elements from stream.
 * @param st Encoder state.
 * @param pnBits Set to the number of bits written.
 * @return 0 on success, otherwise 1.
 */
Word16 WriteTnsData(STnsConfig const * pTnsConfig,
                    Word16 const * stream,
                    Word16 * pnSize,
                    Encoder_State_fx *st,
                    Word16 * pnBits);

/** Read TNS data from a bitstream.
 * Reads TNS data from a bitstream using bin2int.
 * @param pTnsConfig TNS configuration.
 * @param st Decoder state.
 * @param pnBits Set to the number of bits written.
 * @param stream Contains TNS data that can be decoded with DecodeTnsData.
 * @param pnSize Set to the number of used elements from stream.
 * @return 0 on success, otherwise 1.
 */
Word16 ReadTnsData(STnsConfig const * pTnsConfig,
                   Decoder_State_fx *st,
                   Word16 * pnBits,
                   Word16 * stream,
                   Word16 * pnSize);

Word16
CLDFB_getNumChannels(Word32 sampleRate);

void
cldfbAnalysisFiltering( HANDLE_CLDFB_FILTER_BANK anaCldfb,        /*!< Handle of Cldfb Analysis Bank   */
                        Word32 **cldfbReal,                     /*!< Pointer to real subband slots */
                        Word32 **cldfbImag,                     /*!< Pointer to imag subband slots */
                        CLDFB_SCALE_FACTOR *scaleFactor,        /*!< Scale factors of CLDFB data     */
                        const Word16 *timeIn,                 /*!< Time signal */
                        const Word16 timeIn_e,                /*!< Time signal */
                        const Word16 nTimeSlots,              /*!< Time slots to be processed */
                        Word32 *pWorkBuffer                   /*!< pointer to temporal working buffer */
                      );

void
cldfbSynthesisFiltering(HANDLE_CLDFB_FILTER_BANK synCldfb,       /*!< Handle of Cldfb Synthesis Bank  */
                        Word32  **CldfbBufferReal,             /*!< Pointer to 32 bit real subband slots */
                        Word32  **CldfbBufferImag,             /*!< Pointer to 32 bit imag subband slots */
                        const CLDFB_SCALE_FACTOR *scaleFactor, /*!< Scale factors of CLDFB data     */
                        Word16    *timeOut,                  /*!< Time signal */
                        const Word16 timeOut_e,              /*!< Target exponent for output signal */
                        const Word16 nTimeSlots,            /*!< number of time slots to be processed */
                        Word32   *pWorkBuffer                /*!< pointer to temporal working buffer */
                       );

Word16
cldfbInitAnalysisFilterBank(HANDLE_CLDFB_FILTER_BANK h_Cldfb,    /*!< CLDFB Handle */
                            Word16 *pFilterStates,           /*!< Pointer to filter state buffer */
                            Word16  frameSize,               /*!< FrameSize */
                            Word16  lsb,                     /*!< Number of lower bands */
                            Word16  usb,                     /*!< Number of upper bands */
                            Word16  no_channels,             /*!< Number of critically sampled bands */
                            UWord16 flags                    /*!< Flags */
                           );

void
cldfbAnalysisFilteringSlot(HANDLE_CLDFB_FILTER_BANK anaCldfb,    /*!< Handle of Cldfb Synthesis Bank  */
                           Word32 *cldfbReal,                  /*!< Low and High band, real */
                           Word32 *cldfbImag,                  /*!< Low and High band, imag */
                           Word32 *pWorkBuffer                 /*!< pointer to temporal working buffer */
                          );

Word16
cldfbInitSynthesisFilterBank(HANDLE_CLDFB_FILTER_BANK h_Cldfb,   /*!< CLDFB Handle */
                             Word16 *pFilterStates,          /*!< Pointer to filter state buffer */
                             Word16 noCols,                  /*!< Number of time slots  */
                             Word16 lsb,                     /*!< Number of lower bands */
                             Word16 usb,                     /*!< Number of upper bands */
                             Word16 no_channels,             /*!< Number of critically sampled bands */
                             UWord16 flags                   /*!< Flags */
                            );

void cldfbSynthesisFilteringSlot(HANDLE_CLDFB_FILTER_BANK  synCldfb,
                                 const Word32 *realSlot,
                                 const Word32 *imagSlot,
                                 const Word16  scaleFactorLowBand,
                                 const Word16  scaleFactorHighBand,
                                 Word16       *timeOut,
                                 const Word16  stride,
                                 Word32       *pWorkBuffer
                                );


void configureCldfb ( HANDLE_CLDFB_FILTER_BANK h_cldfb,        /*!< CLDFB Handle */
                      const Word16 no_channels,                /*!< Number of critically sampled bands */
                      const Word16 frameSize                   /*!< FrameSize */
                    );

void openCldfb ( HANDLE_CLDFB_FILTER_BANK *h_cldfb,    /*!< CLDFB Handle */
                 const Word16 type,                                   /*!< analysis or synthesis */
                 const Word16 maxCldfbBands,                          /*!< number of cldfb bands */
                 const Word16 frameSize                               /*!< FrameSize */
               );

void resampleCldfb (
    HANDLE_CLDFB_FILTER_BANK hs,                         /*!< CLDFB Handle */
    const Word16 newCldfbBands,                          /*!< number of cldfb bands */
    const Word16 frameSize,                              /*!< FrameSize */
    const Word8 firstFrame
);

void
deleteCldfb (HANDLE_CLDFB_FILTER_BANK * h_cldfb);        /*!< CLDFB Handle */

Word16
CreateCldfbAnalysisFilterBank (HANDLE_CLDFB_FILTER_BANK  *hAnalysisCldfb,
                               const Word16             maxCldfbBands,
                               const Word16             frameSize,
                               UWord16                  flags
                              );

Word16
DestroyCldfbAnalysisFilterBank (HANDLE_CLDFB_FILTER_BANK *hAnalysisCldfb
                               );

Word16
CreateCldfbSynthesisFilterBank (HANDLE_CLDFB_FILTER_BANK  *hSynthesisCldfb,
                                const Word16             maxCldfbBands,
                                const Word16             frameSize,
                                UWord16                  flags
                               );

Word16
DestroyCldfbSynthesisFilterBank (HANDLE_CLDFB_FILTER_BANK *hSynthesisCldfb
                                );

Word16
ResampleCldfbAnalysisFilterBank (HANDLE_CLDFB_FILTER_BANK hs,
                                 const Word16           newCldfbBands,
                                 Word16                 newFrameSize,
                                 UWord16                flags,
                                 Word8                  firstFrame
                                );



Word16
AnalysisPostSpectrumScaling_Fx (HANDLE_CLDFB_FILTER_BANK cldfbBank,
                                Word32 **rSubband32,
                                Word32 **iSubband32,
                                Word16 **rSubband16,
                                Word16 **iSubband16,
                                Word16  *cldfbScale
                               );

void
GetEnergyFromCplxCldfbData_Fx(Word32 **energyValues,
                              Word32 *energyLookahead,
                              Word16 *sf_energyLookahead,
                              const Word16 numLookahead,
                              Word16 **realValues,
                              Word16 **imagValues,
                              Word16   sf_Values,
                              Word16   numberBands,
                              Word16   numberCols,
                              Word32  *energyHF,
                              Word16  *energyHF_Exp
                             );

void
GetEnergyCldfb( Word32 *energyLookahead,    /*!< o: Q31 |   pointer to the result in the core look-ahead slot */
                Word16 *sf_energyLookahead, /*!< o:         pointer to the scalefactor of the result in the core look-ahead slot  - apply as negative exponent*/
                const Word16 numLookahead,  /*!< i: Q0      the number of look-ahead time-slots */
                Word16 **realValues,        /*!< i: Q15 |   the real part of the CLDFB subsamples */
                Word16 **imagValues,        /*!< i: Q15 |   the imaginary part of the CLDFB subsamples */
                Word16   sf_Values,         /*!< i:         scalefactor of the CLDFB subcamples - apply as a negated Exponent */
                Word16   numberBands,       /*!< i: Q0  |   number of CLDFB bands */
                Word16   numberCols,        /*!< i: Q0  |   number of CLDFB subsamples */
                Word32  *energyHF,          /*!< o: Q31 |   pointer to HF energy */
                Word16  *energyHF_Exp,      /*!< o:         pointer to exponent of HF energy */
                Word32 *energyValuesSum,    /*!< o: Q31 |   pointer to sum array of energy values, not initialized*/
                Word16 *energyValuesSum_Exp /*!< o:         pointer to exponents of energyValuesSum, not initialized */
                , HANDLE_TEC_ENC_FX hTecEnc
              );

/*bits_alloc.h*/
Word16 BITS_ALLOC_get_rate_mode(
    const Word16 frame_size_index      /*i: frame_size_index*/
);

void BITS_ALLOC_init_config_acelp(
    const Word32 bit_rate,
    const Word8  narrowBand,
    const Word16  nb_subfr,
    ACELP_config *pConfigAcelp       /*o:  configuration structure of ACELP*/
);

Word16 BITS_ALLOC_config_acelp(
    const Word16 bits_frame,            /*i: remaining bit budget for the frame  */
    const Word16  coder_type,        /*i: coder type      */
    ACELP_config *pConfigAcelp,      /*i/o:  configuration structure of ACELP*/
    const Word16 narrowband,
    const Word16 nb_subfr
);

void BITS_ALLOC_ACELP_config_rf(const Word16 coder_type,
                                Word16 *tilt_code,
                                Word16 *rf_frame_type,
                                Word16 *rf_target_bits,
                                Word16 nb_subfr
                                , Word16 rf_fec_indicator
                                , Word16 *pitch_buf
                               );
void BITS_ALLOC_TCX_config_rf(
    Word16 *rf_frame_type,
    Word16 *rf_target_bits,
    Word16 PLC_Mode,
    Word16 coder_type,
    Word16 last_core,
    Word16 TD_mode
);

Word16 BITS_ALLOC_adjust_acelp_fixed_cdk(
    const Word16 bits_frame,    /*i: bit budget      */
    Word16 *fixed_cdk_index,    /*o: codebook index    */
    const Word16 nb_subfr      /*i: number of subframes*/
);



Word16 BITS_ALLOC_adjust_acelp_fixed_16KHZ_cdk(
    const Word16 bits_frame,      /*i: bit budget      */
    Word16 *fixed_cdk_index,        /*o: codebook index    */
    const Word16 nb_subfr        /*i: number of subframes*/
);

/*transient_detection.h*/
/** Init transient detection.
 * Fills TransientDetection structure with sensible content.
 * @param nFrameLength Frame length.
 * @param nTCXDelay Delay for the TCX Long/Short transient detector.
 *        Don't include the delay of the MDCT overlap.
 * @param pTransientDetection Structure to be initialized. It contains all transient detectors to be used.
 */
void InitTransientDetection(Word16 nFrameLength,
                            Word16 nTCXDelay,
                            struct TransientDetection * pTransientDetection);

/** Runs transient detection.
 * Runs all transient detectors defined in pTransientDetection
 * and calculates mean zero crossing.
 * @param input New input samples.
 * @param nSamplesAvailable Number of new input samples available.
 * @param pTransientDetection Structure that contains transient detectors to be run.
 */
void RunTransientDetection(Word16 const * input, Word16 nSamplesAvailable, struct TransientDetection * pTransientDetection);


/** Get the average temporal flatness measure using subblock energies aligned with the TCX.
 * @param pTransientDetection Structure that contains transient detectors to be run.
 * @param nCurrentSubblocks Number of the subblocks from the current frame to use for the calculation.
 A lookeahead can also be use if it exists.
 * @param nPrevSubblocks Number of subblocks from the previous frames to use for the calculation.
 * @return average temporal flatness measure with exponent AVG_FLAT_E
 */
Word16 GetTCXAvgTemporalFlatnessMeasure(struct TransientDetection const * pTransientDetection, Word16 nCurrentSubblocks, Word16 nPrevSubblocks);


/** Get the maximum energy change using subblock energies aligned with the TCX.
 * @param pTransientDetection Structure that contains transient detectors to be run.
 * @param nCurrentSubblocks Number of the subblocks from the current frame to use for the calculation.
 A lookeahead can also be use if it exists.
 * @param nPrevSubblocks Number of subblocks from the previous frames to use for the calculation.
 * @param maximum energy change with exponent NRG_CHANGE_E
 */
Word16 GetTCXMaxenergyChange(struct TransientDetection const * pTransientDetection,
                             const Word8 isTCX10,
                             const Word16 nCurrentSubblocks, const Word16 nPrevSubblocks);

/** Set TCX window length and overlap configuration
 * @param prevEnergyHF previous HF energy. Exponent must be the same as for currEnergyHF.
 * @param currEnergyHF current HF energy. Exponent must be the same as for prevEnergyHF.
 */
void SetTCXModeInfo(Encoder_State_fx *st,
                    struct TransientDetection const * pTransientDetection,
                    Word16 * tcxModeOverlap);

/* Tonal Concealment */

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Init( TonalMDCTConcealPtr self,
        Word16 nSamples,
        Word16 nSamplesCore,
        Word16 nScaleFactors,
        TCX_config * tcx_cfg,
        ApplyScaleFactorsPointer pApplyScaleFactors
                                            );


/* Must be called only when a good frame is recieved - concealment is inactive */
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_SaveFreqSignal( TonalMDCTConcealPtr self,
        Word32 const *mdctSpectrum,
        Word16 const mdctSpectrum_exp,
        Word16 nNewSamples,
        Word16 nNewSamplesCore,
        Word16 const *scaleFactors,
        Word16 const *scaleFactors_exp,
        Word16 const gain_tcx_exp
                                                      );

/* The call to TonalMDCTConceal_UpdateState() should be called after TonalMDCTConceal_Apply. */
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_UpdateState(TonalMDCTConcealPtr self,
        Word16 nNewSamples,
        Word32 pitchLag,
        Word16 badBlock,
        Word8 tonalConcealmentActive
                                                   );

/* The call to TonalMDCTConceal_SaveTimeSignal() should be at the
 * place where the TD signal corresponds to the FD signal stored with TonalMDCTConceal_SaveFreqSignal. */
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_SaveTimeSignal( TonalMDCTConcealPtr self,
        Word16             *timeSignal,
        Word16              numSamples
                                                      );

/* Calculates MDST, power spectrum and performs peak detection.
 * Uses the TD signal in pastTimeSignal; if pastTimeSignal is NULL, uses the
 * TD signal stored using TonalMDCTConceal_SaveTimeSignal.  If the
 * second last frame was also lost, it is expected that pastTimeSignal
 * could hold a signal somewhat different from the one stored in
 * TonalMDCTConceal_SaveTimeSignal (e.g. including fade-out).*/
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Detect( TonalMDCTConcealPtr const self,        /*IN */
        Word32 const pitchLag,                 /*IN */
        Word16 const * const pastTimeSignal,   /*IN */
        Word16 * const umIndices               /*OUT*/
                                              );

/* Conceals the lost frame using the FD signal previously stored using
 * TonalMDCTConceal_SaveFreqSignal.  Stores the concealed harmonic part of
 * the signal in mdctSpectrum, the rest of the spectrum is unchanged. */
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Apply( TonalMDCTConcealPtr self,       /*IN */
        Word32 * mdctSpectrum,          /*OUT*/
        Word16 * mdctSpectrum_exp       /*OUT*/
                                             );

/* Conceals the lost frame using the FD signal previously stored using
 * TonalMDCTConceal_SaveFreqSignal.  Stores the concealed noise part of
 * the signal in mdctSpectrum, the rest of the spectrum is unchanged. */
TONALMDCTCONCEAL_ERROR TonalMDCTConceal_InsertNoise( TonalMDCTConcealPtr self,      /*IN */
        Word32* mdctSpectrum,          /*OUT*/
        Word16* mdctSpectrum_exp,      /*OUT*/
        Word8   tonalConcealmentActive,
        Word16* pSeed,                 /*IN/OUT*/
        Word16  tiltCompFactor,
        Word16  crossfadeGain,
        Word16  crossOverFreq);

/* Detect tonal components in the lastMDCTSpectrum, use
 * secondLastPowerSpectrum for the precise location of the peaks and
 * store them in indexOfTonalPeak.  Updates lowerIndex, upperIndex,
 * pNumIndexes accordingly. */
void DetectTonalComponents(Word16 indexOfTonalPeak[],                      /* OUT */
                           Word16 lowerIndex[],                            /* OUT */
                           Word16 upperIndex[],                            /* OUT */
                           Word16 * pNumIndexes,                           /* OUT */
                           Word32 lastPitchLag, Word32 currentPitchLag,    /* IN */
                           Word16 const lastMDCTSpectrum[],                /* IN */
                           Word16 lastMDCTSpectrum_exp,                    /* IN */
                           ApplyScaleFactorsPointer pApplyScaleFactors,    /* IN */
                           Word16 const scaleFactors[],                    /* IN */
                           Word16 const scaleFactors_exp[],                /* IN */
                           Word16 const scaleFactors_max_e,                /* IN */
                           Word32 const secondLastPowerSpectrum[],         /* IN */
                           Word16 nSamples                                 /* IN */
                           ,Word16 nSamplesCore
                           ,Word16 floorPowerSpectrum                      /* IN */
                          );

/* When called, the tonal components are already stored in
 * indexOfTonalPeak.  Detect tonal components in the lastMDCTSpectrum,
 * use secondLastPowerSpectrum for the precise location of the peaks and
 * then keep in indexOfTonalPeak only the tonal components that are
 * again detected Updates indexOfTonalPeak, lowerIndex, upperIndex,
 * phaseDiff, phases, pNumIndexes accordingly. */
void RefineTonalComponents(Word16 indexOfTonalPeak[],                     /* OUT */
                           Word16 lowerIndex[],                           /* OUT */
                           Word16 upperIndex[],                           /* OUT */
                           Word16 phaseDiff[],                            /* OUT */
                           Word16 phases[],                               /* OUT */
                           Word16 * pNumIndexes,                          /* OUT */
                           Word32 lastPitchLag,                           /* IN  */
                           Word32 currentPitchLag,                        /* IN  */
                           Word16 const lastMDCTSpectrum[],               /* IN  */
                           Word16 const lastMDCTSpectrum_exp,             /* IN  */
                           ApplyScaleFactorsPointer pApplyScaleFactorsPointer,  /* IN  */
                           Word16 const scaleFactors[],                   /* IN  */
                           Word16 const scaleFactors_exp[],               /* IN  */
                           Word16 const scaleFactors_max_e,               /* IN  */
                           Word32 const secondLastPowerSpectrum[],        /* IN  */
                           Word16 nSamples                                /* IN  */
                           ,Word16 nSamplesCore
                           ,Word16 floorPowerSpectrum                      /* IN */
                          );

void set_state(Word16 *state, Word16 num, Word16 N);
void concealment_init_x(Word16 N, void *_plcInfo);
void concealment_update_x(Word16 bfi, Word16 curr_mode, Word16 harmonic, Word32 *invkoef, Word16 *invkoef_scale, void *_plcInfo);

Word16 waveform_adj_fix( Word16 *overlapbuf,
                         Word16 *outdata2,
                         Word16 *outx_new,
                         Word16 *data_noise,
                         Word16 *outx_new_n1,
                         Word16 *nsapp_gain,
                         Word16 *nsapp_gain_n,
                         Word16 Framesize,
                         Word8  pre_bfi,
                         Word16 voicing,
                         Word16 curr_mode,
                         Word16 subframe,
                         Word16 pitch);

void waveform_adj2_fix(  Word16 *overlapbuf,
                         Word16 *outx_new,
                         Word16 *data_noise,
                         Word16 *outx_new_n1,
                         Word16 *nsapp_gain,
                         Word16 *nsapp_gain_n,
                         Word16 *recovery_gain,
                         Word16 step_concealgain,
                         Word16 pitch,
                         Word16 Framesize,
                         Word16 curr_mode,
                         Word16 subframe,
                         Word16 bfi_cnt,
                         Word16  bfi
                      );

Word16 ffr_getSfWord16(Word16 *vector, /*!< Pointer to input vector */
                       Word16 len);           /*!< Length of input vector */

void concealment_decode_fix(Word16 curr_mode, Word32 *invkoef, Word16 *invkoef_scale,void *_plcInfo);
void concealment_signal_tuning_fx(Word16 bfi, Word16 curr_mode, Word16 *outx_new_fx, void *_plcInfo, Word16 nbLostCmpt, Word16 pre_bfi, Word16 *OverlapBuf_fx, Word16 past_core_mode, Word16 *outdata2_fx, Decoder_State_fx *st);
Word16 Spl_GetScalingSquare_x(Word16 *in_vector, Word16 in_vector_length, Word16 times);
Word32 Spl_Energy_x(Word16* vector, Word16 vector_length, Word16* scale_factor);
void Log10OfEnergy_x(Word16 *s, Word32 *enerlogval, Word16 len);
void concealment_update2_x(Word16 *outx_new, void *_plcInfo, Word16 FrameSize);
Word16 Sqrt_x_fast(Word32 value);

Word32 dot_w32_accuracy_x(Word16 *s1, Word16 *s2, Word16 nbits, Word16 N);

Word16 int_div_s_x(Word16 a, Word16 b);

Word16  GetW32Norm_x(Word32 *s, Word16 N);

Word16 harmo_x(Word32 *X, Word16  Framesize, Word16 pitch);

Word16 get_low_freq_eng_rate_x(Word32 *mdct_data, Word16 curr_mode, Word16 N);

void LpFilter2_x(Word16 *x, Word16 *y, Word16 N);

void sig_tilt_x(Word16 *s, Word16 FrameSize, Word32 *enr1, Word32 *enr2) ;

void get_maxConv_and_pitch_x(Word16 *s_LP, Word16 s, Word16 e, Word16 N,
                             Word32 *maxConv, Word16 *maxConv_bits, Word16 *pitch);

Word16 get_voicing_x(Word16 *s_LP, Word16 pitch, Word32 covMax,Word16 maxConv_bits,  Word16 Framesize);

void pitch_modify_x(Word16 *s_LP, Word16 *voicing, Word16 *pitch, Word16 FrameSize);

Word16 Is_Periodic_x(Word32 *mdct_data, Word16 cov_max, Word16  zp, Word32 ener,
                     Word32 ener_mean, Word16  pitch, Word16  Framesize);

Word16 get_conv_relation_x(Word16 *s_LP, Word16 shift, Word16 N);

Word16  pitch_search_fx(Word16 *s,
                        Word16 *outx_new,
                        Word16   Framesize,
                        Word16 *voicing,
                        Word16 zp,
                        Word32 ener,
                        Word32 ener_mean,
                        Word32 *mdct_data,
                        Word16 curr_mode);



/* 14Q1*1.28 */
#define LSF_GAP_VAL(x) (Word16)((x)*2.0f*1.28f)

/*longarith.h*/

/**
 * \brief          inplace long shift right: a[] = a[] >> bits
 *                 Logical shift right of UWord32 vector a[] by 'bits' positions.
 */

void longshr (UWord32 a[], Word16 bits, Word16 len);


/*qlpc_avq.h*/
void lsf_weight_2st(const Word16 *lsfq, /* input: quantized lsf coefficients (3Q12) */
                    Word16 *w,          /* output: weighting function (0Q15)        */
                    const Word16 mode   /* input: operational mode                  */
                   );



void qlpc_avq(
    const Word16 *lsf,      /* (i) Input LSF vectors              */
    const Word16 *lsfmid,   /* (i) Input LSF vectors              */
    Word16 *lsf_q,          /* (o) Quantized LFS vectors          */
    Word16 *lsfmid_q,       /* (o) Quantized LFS vectors          */
    Word16 *index,          /* (o) Quantization indices           */
    Word16 *nb_indices,     /* (o) Number of quantization indices */
    Word16 *nbbits,         /* (o) Number of quantization bits    */
    const Word16 core,      /* (i) TCX10 or TCX20                 */
    Word32 sr_core
);

Word16 encode_lpc_avq( Encoder_State_fx *st, Word16 numlpc, Word16 *param_lpc, Word16 mode );

Word16 dlpc_avq(
    Word16 *index,            /* (i) Quantization indices  */
    Word16 *LSF_Q,            /* (o) Quantized LSF vectors */
    Word16 numlpc,            /* (i) Number of sets of lpc */
    Word32 sr_core
);

Word16 decode_lpc_avq( Decoder_State_fx *st, Word16 numlpc, Word16 *param_lpc );

Word16 vlpc_1st_cod(const Word16 *lsf,  /* input:  vector to quantize              */
                    Word16 *lsfq         /* i/o:    i:prediction   o:quantized lsf  */
                    ,Word16 *wout              /* o: lsf weights */
                    ,Word16 rf_mode
                   );

Word16 vlpc_2st_cod(                  /* output: number of allocated bits             */
    const Word16 *lsf,  /* input:  normalized vector to quantize (3Q12) */
    Word16 *lsfq,  /* i/o:    i:1st stage   o:1st+2nd stage (3Q12) */
    Word16 *indx, /* output: index[] (4 bits per words)           */
    Word16  mode, /* input:  0=abs, >0=rel                        */
    Word32 sr_core
);

void vlpc_2st_dec(
    Word16 *lsfq,    /* i/o:    i:1st stage   o:1st+2nd stage   */
    Word16 *indx,    /* input:  index[] (4 bits per words)      */
    Word16 mode,     /* input:  0=abs, >0=rel                   */
    Word32 sr_core);

void open_decoder_LPD(
    Decoder_State_fx *st,
    Word32 bitrate,
    Word16 bandwidth
);


void update_decoder_LPD_cng(
    Decoder_State_fx *st,
    Word16 coder_type,
    Word16 *timeDomainBuffer,
    Word16 *A,
    Word16 *bpf_noise_buf
);

void reconfig_decoder_LPD( Decoder_State_fx *st, Word16 bits_frame, Word16 bandwidth_mode, Word32 bitrate, Word16 L_frame_old);

void mode_switch_decoder_LPD( Decoder_State_fx *st, Word16 bandwidth_in, Word32 bitrate, Word16 frame_size_index
                            );

void decoder_LPD(
    Word16 signal_out[],   /* output: signal with LPD delay (7 subfrs) */
    Word16 signal_outFB[],
    Word16 *total_nbbits,  /* i/o:    number of bits / decoded bits    */
    Decoder_State_fx *st , /* i/o:    decoder memory state pointer     */
    Word16 *bpf_noise_buf,
    Word16 bfi,
    Word16 *bitsRead,
    Word16 *coder_type,
    Word16 param[],
    Word16 *pitch_buf,
    Word16 *voice_factors,
    Word16 *ptr_bwe_exc
);

Word16 dec_acelp_tcx_frame(
    Decoder_State_fx *st,
    Word16 *coder_type,
    Word16 *concealWholeFrame,
    Word16 *pcmBuf,
    Word16 * bpf_noise_buf,
    Word16 * pcmbufFB,
    Word32 bwe_exc_extended[],
    Word16 *voice_factors,
    Word16 pitch_buf[]
);


Word16 tcxGetNoiseFillingTilt(Word16 A[],
                              Word16 lpcorder,
                              Word16 L_frame,
                              Word16 mode,
                              Word16 *noiseTiltFactor
                             );


void tcxFormantEnhancement(
    Word16 xn_buf[],
    Word16 gainlpc[], Word16 gainlpc_e[],
    Word32 spectrum[], Word16 *spectrum_e,
    Word16 L_frame
    ,Word16 L_frameTCX
);


void tcxInvertWindowGrouping(TCX_config *tcx_cfg,
                             Word32 xn_buf[],
                             Word32 spectrum[],
                             Word16 L_frame,
                             Word8 fUseTns,
                             Word16 last_core,
                             Word16 index,
                             Word16 frame_cnt,
                             Word16 bfi);

/*lerp.h*/

void lerp(Word16 *f, Word16 *f_out, Word16 bufferNewSize, Word16 bufferOldSize);

void attenuateNbSpectrum(Word16 L_frame, Word32 *spectrum);

Word8 getTcxonly(const Word32 bitrate);
Word8 getCtxHm(const Word32 bitrate, const Word16 rf_flag);
Word8 getResq(const Word32 bitrate);

Word8 getTnsAllowed(const Word32 bitrate
                    ,const Word16 igf
                   );

Word8 getRestrictedMode(const Word32 bitrate, const Word16 Opt_AMR_WB);

Word16 sr2fscale(const Word32 sr);

Word32 getCoreSamplerateMode2(const Word32 bitrate, const Word16 bandwidth, const Word16 rf_flag);
Word16 getTcxBandwidth(const Word16 bandwidth);

Word8 getIgfPresent(
    const Word32 bitrate,
    const Word16 bandwidth
    , const Word16 rf_mode
);

Word8 getCnaPresent(
    const Word32 bitrate,
    const Word16 bandwidth
);

Word8 getTcxLtp(
    const Word32 bitrate,
    const Word32 sr_core,
    const Word16 Opt_AMR_WB
);

Word16 initPitchLagParameters(
    const Word32 sr_core,
    Word16 *pit_min,
    Word16 *pit_fr1,
    Word16 *pit_fr1b,
    Word16 *pit_fr2,
    Word16 *pit_max
);

Word16 getNumTcxCodedLines(const Word16 bwidth);

Word16 getTcxLpcShapedAri(
    const Word32 total_brate,
    const Word16 bwidth
    ,const Word16 rf_mode
);


Word16 vad_init(T_CldfbVadState *vad_state);

void subband_FFT(
    Word32 ** Sr,     /*(i) real part of the CLDFB*/
    Word32 ** Si,     /*(i) imag part of the CLDFB*/
    Word32 *spec_amp, /*(o) spectral amplitude*/
    Word32 Offset,    /*(i) offset of the CLDFB*/
    Word16 *fftoQ     /*(o) the Scaling */
);

void UpdateState( T_CldfbVadState *vad_state,
                  Word16 vad_flag,
                  Word32 frame_energy,       /*(i)  current frame energy*/
                  Word16 sacle_sbpower,      /*(i)  the Scaling of current frame energy*/
                  Word32 update_flag,        /*(i)  current frame update flag*/
                  Word16 music_backgound_f,  /*(i)  backgound music flag*/
                  Word32 HB_Power,           /*(i)  current frame high frequency energy*/
                  Word16 HB_Power_Q          /*(i)  the Scaling of current frame high frequency energy*/
                );

void calc_snr_flux( Word32  tsnr_fix,                /*(i) time-domain SNR*/
                    Word32 *pre_snr,             /*(io)time-domain SNR storage*/
                    Word32 *snr_flux                 /*(o) average tsnr*/
                  );

Word16 update_decision(
    T_CldfbVadState *st,
    Word32 frame_energy,     /*(i) current frame energy*/
    Word32 HB_Power,         /*(i) current frame high frequency energy*/
    Word16 frameloop,        /*(i) amount of  frames*/
    Word16 bw,               /*(i) band width index*/
    Word16 frame_energy_Q,   /*(i) the Scaling of current frame energy*/
    Word16 HB_Power_Q,  /*(i) the Scaling of current frame high frequency energy*/
    Word32 snr,              /*(i) frequency domain SNR */
    Word32 tsnr,             /*(i) time domain SNR */
    Word16 vad_flag,
    Word16 music_backgound_f /*(i) background music flag*/
);

void frame_spec_dif_cor_rate( T_CldfbVadState *st,             /*(io) vad state*/
                              Word32 *spec_amp,              /*(i) spectral amplitude*/
                              Word16 sacle,                  /*(i) the scaling of spec_amp*/
                              Word16 f_tonality_rate[3]  /*(o) tonality rate*/
                            );

void ltd_stable( T_CldfbVadState *st,       /*(io) vad state*/
                 Word16 *ltd_stable_rate, /*(o) time-domain stable rate*/
                 Word32 frame_energy,     /*(i) current frame energy*/
                 Word16 frameloop,        /*(i) amount of  frames*/
                 Word16 Q_frames_power    /*(i) the Scaling of  frames_power*/
               );

void snr_calc( T_CldfbVadState *st,      /*(io) vad state*/
               Word16  sacle_sbpower,  /*(i) the Scaling of sbpower*/
               Word32  *snr,           /*(o) frequency domain SNR */
               Word32  *tsnr,          /*(o) time domain SNR */
               Word32  frame_energy,   /*(i) current frame energy */
               Word32  bandwith        /*(i) band width*/
             );

void bg_music_decision( T_CldfbVadState *st,
                        Word16  *music_backgound_f,  /*(i)  background music flag*/
                        Word32  frame_energy,        /*(i)  current frame energy 1*/
                        Word16  frame_energy_Q       /*(i)  the Scaling of current frame energy*/
                      );

void background_update( T_CldfbVadState *st,
                        Word16 scale,           /*(i) the Scaling of frame energy*/
                        Word32 frame_energy,     /*(i) current frame energy*/
                        Word32 update_flag,      /*(i) update flag*/
                        Word16 music_backgound_f /*(i) background music flag*/
                      );

void spec_center( Word32* sb_power,  /*(i) energy of sub-band divided uniformly*/
                  Word16* sp_center, /*(o) spectral center*/
                  Word32  bandwith,  /*(i) band width*/
                  Word16  Q_sb_p     /*(i) the Scaling of sb_power*/
                );

void spec_flatness( Word32 *spec_amp,          /*(i) spectral amplitude*/
                    Word32 smooth_spec_amp[],  /*(i) smoothed spectral amplitude*/
                    Word16 sSFM[5]             /*(o) spectral flatness rate*/
                  );

void calc_lt_snr( T_CldfbVadState *st,  /*(io) vad state*/
                  Word32 *lt_snr_org,          /*(o) original long time SNR*/
                  Word32 *lt_snr,              /*(o) long time SNR calculated by fg_energy and bg_energy*/
                  Word32  fg_energy,           /*(i) foreground energy sum  */
                  Word16  fg_energy_count,     /*(i) amount of the foreground energy frame */
                  Word32  bg_energy,           /*(i) background energy sum  */
                  Word16  bg_energy_count,     /*(i) amount of the background energy frame */
                  Word16  bw_index,            /*(i) band width index*/
                  Word16  lt_noise_sp_center0  /*(i) long time noise spectral center by 0*/
                );

Word16 vad_decision( T_CldfbVadState *st,
                     Word32 l_snr,             /*(i) long time frequency domain*/
                     Word32 lt_snr_org,        /*(i) original long time SNR*/
                     Word32 lt_snr,            /*(i) long time SNR calculated by fg_energy and bg_energy*/
                     Word32 snr_flux,          /*(i) average tsnr of several frames*/
                     Word32 snr,               /*(i) frequency domain SNR */
                     Word32 tsnr,              /*(i) time domain SNR */
                     Word32 frame_energy,      /*(i) current frame energy */
                     Word16 music_backgound_f, /*(i) background music flag*/
                     Word16 frame_energy_Q     /*(i) the scaling of current frame energy*/
                   );

Word16 comvad_decision( T_CldfbVadState *st,
                        Word32 l_snr,              /*(i) long time frequency domain*/
                        Word32 lt_snr_org,         /*(i) original long time SNR*/
                        Word32 lt_snr,             /*(i) long time SNR calculated by fg_energy and bg_energy*/
                        Word32 snr_flux,           /*(i) average tsnr of several frames*/
                        Word32 snr,                /*(i) frequency domain SNR */
                        Word32 tsnr,               /*(i) time domain SNR */
                        Word32 frame_energy,       /*(i) current frame energy */
                        Word16 music_backgound_f,  /*(i) background music flag*/
                        Word16 frame_energy_Q,     /*(i) the Scaling of current frame energy*/
                        Word16 *cldfb_addition,    /*(o) adjust the harmonized hangover */
                        Word16 vada_flag
                      );

void calc_lf_snr(
    Word32 *lf_snr_smooth,         /*(o) smoothed lf_snr*/
    Word32 *lf_snr,                /*(o) long time frequency domain SNR calculated by l_speech_snr and l_silence_snr*/
    Word32 l_speech_snr,           /*(i) sum of active frames snr */
    Word32 l_speech_snr_count,     /*(i) amount of the active frame  */
    Word32 l_silence_snr,          /*(i) sum of the nonactive frames snr*/
    Word32 l_silence_snr_count,    /*(i) amount of the nonactive frame */
    Word16 fg_energy_count,        /*(i) amount of the foreground energy frame */
    Word16 bg_energy_count,        /*(i) amount of the background energy frame */
    Word16 bw_index                /*(i) band width index*/
);

Word32 construct_snr_thresh( Word16 sp_center[],              /*(i) spectral center*/
                             Word32 snr_flux,                 /*(i) snr flux*/
                             Word32 lt_snr,                   /*(i) long time time domain snr*/
                             Word32 l_snr,                    /*(i) long time frequency domain snr*/
                             Word32 continuous_speech_num,    /*(i) amount of continuous speech frames*/
                             Word16 continuous_noise_num,     /*(i) amount of continuous noise frames*/
                             Word32 fg_energy_est_start,      /*(i) whether if estimated energy*/
                             Word16 bw_index                  /*(i) band width index*/
                           );

Word16 vad_proc(T_CldfbVadState *vad_st,
                Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],    /* i: real values */
                Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],    /* i: imag values */
                Word16 riBuffer_exp,                                           /* i: exponent of real & imag Buffer */

                Word16 *cldfb_addition,                        /*o: adjust the harmonized hangover */
                Word32 enerBuffer[CLDFB_NO_CHANNELS_MAX],    /* i: energy vector per band */
                Word16 enerBuffer_exp,                       /* i: exponent of energy vector */
                Word16 bandwidth,                            /* 1: NB; 2:WB;3:SWB;4:FB*/
                Word16 vada_flag
               );

void est_energy(Word32 enerBuffer[CLDFB_NO_CHANNELS_MAX],          /* i: energy vector per band */
                Word16 enerBuffer_exp,                           /* i: exponent of energy vector */
                Word32 *frame_sb_energy,       /*(o) energy of sub-band divided non-uniformly*/
                Word32 *frame_energy2_p,       /*(o) frame energy 2*/
                Word32 *HB_Power_p,             /*(o) high frequency energy*/
                Word32 *frame_energy_p,        /*(o) frame energy 1*/
                Word16 *sb_power_Q,             /*(o) the scaling of sb_power*/
                Word16 *frame_energy2_Q,       /*(o) the scaling of frame_energy*/
                Word16 *HB_Power_Q,            /*(o) the scaling of HB_Power*/
                Word16 *frame_energy_Q,        /*(o) the Scaling of frame_energy*/
                Word16 *frame_sb_energy_scale, /*(o) the Scaling of frame_sb_energy[]*/
                Word32 bandwidth               /*(i) band width*/
               );

/**
 * \brief    Function performs a complex 16-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR16 bits.
 *
 *           WOPS FLC version:                                 196 cycles
 *           WOPS with 32x16 bit multiplications (scale on ):  288 cycles
 *           WOPS with 32x16 bit multiplications (scale off):  256 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
void fft16(Word32 *re, Word32 *im, Word16 s, Word16 bScale);



void dlpc_bfi(
    const Word16 L_frame,
    Word16 *lsf_q,            /* o  : quantized LSFs                         */
    const Word16 *lsfold,           /* i  : past quantized LSF                     */
    const Word16 last_good,         /* i  : last good received frame               */
    const Word16 nbLostCmpt,        /* i  : counter of consecutive bad frames      */
    Word16 mem_MA[],          /* i/o: quantizer memory for MA model          */
    Word16 mem_AR[],          /* i/o: quantizer memory for AR model          */
    Word16 *stab_fac,         /* i  : LSF stability factor                   */
    Word16 *lsf_adaptive_mean,/* i  : LSF adaptive mean, updated when BFI==0 */
    Word16   numlpc,            /* i  : Number of division per superframe      */
    Word16 lsf_cng[],
    Word8   plcBackgroundNoiseUpdated,
    Word16 *lsf_q_cng,        /* o  : quantized LSFs                      */
    Word16 *old_lsf_q_cng,    /* o  : old quantized LSFs for background noise */
    const Word16* lsfBase,    /* i  : base for differential LSF coding        */
    Word16 tcxonly
);

void isf_dec_bfi(
    const Word16 last_type,            /*!< i  :               coding type in last good received fr.     */
    Word16*isf_q,               /*!< o  : 14Q1*1.28     quantized ISFs                            */
    const Word16*isfold,              /*!< i  : 14Q1*1.28     past quantized ISF                        */
    const Word16 last_good,            /*!< i  :               last good received frame                  */
    const Word16 bfi_cnt,              /*!< i  :               counter of consecutive bad frames         */
    Word16*mem_MA,              /*!< i/o: 14Q1*1.28     quantizer memory for MA model             */
    Word16 stab_fac,            /*!< i  :               ISF stability factor (shifted right by 1) */
    Word16*isf_adaptive_mean,   /*!< i  : 14Q1*1.28     ISF adaptive mean, updated when BFI==0    */
    const Word16 coder_type,           /*!< i  :               Coders frame type                         */
    Word16 lpcorder,               /*!< i: :               lpc order                               */
    Word16 *isf_cng,
    Word8  plcBackgroundNoiseUpdated,
    Word16 *isf_q_cng,        /* o  : quantized ISFs for background noise     */
    Word16 *old_isf_q_cng,   /* o  : old quantized ISFs for background noise */
    const Word16 xsfBase[]    /* i  : base for differential XSF coding        */
);

void tfaCalcEnv_fx(const Word16* shb_speech, Word32* enr);
Word16 tfaEnc_TBE_fx(Word32* enr,
                     Word16 last_core,
                     Word16* voicing,   /* Q15 */
                     Word16* pitch_buf, /* Q6 */
                     Word16 Q_enr
                    );

void tecEnc_TBE_fx(Word16* corrFlag, const Word16* voicing, Word16 coder_type);

void set_TEC_TFA_code_fx(const Word16 corrFlag, Word16* tec_flag, Word16* tfa_flag);

Word16 procTecTfa_TBE_Fx(Word16 *hb_synth_Fx,
                         Word16 hb_synth_fx_exp,
                         Word16 *gain_m,
                         Word16 *gain_e,
                         Word16 flat_flag,
                         Word16 last_core
                         , Word16 l_subfr
                         , Word16 code
                        );


void
calcGainTemp_TBE_Fx(
    Word32** pCldfbRealSrc_Fx,
    Word32** pCldfbImagSrc_Fx,
    Word16 cldfb_exp,
    Word16* loBuffer_Fx,
    Word16 startPos,          /*!<  Start position of the current envelope. */
    Word16 stopPos,           /*!<  Stop position of the current envelope. */
    Word16 lowSubband,   /* lowSubband */
    Word16* pGainTemp_m,
    Word16* pGainTemp_e
    , Word16 code
);

void resetTecEnc_Fx(
    HANDLE_TEC_ENC_FX hTecEnc,
    Word16 flag

);

void
resetTecDec_Fx(
    HANDLE_TEC_DEC_FX hTecDec
);

void calcHiEnvLoBuff_Fix(
    Word16 noCols,
    Word16* pFreqBandTable,        /*!<  freqbandTable. */
    Word16 nSfb,                   /*!<  Number of scalefactors. */
    Word32** pCldfbPow_Fix			 /*float** pCldfbPow*/,
    Word16* loBuffer_Fix           /*float* loBuffer    Q8*/,
    Word16* hiTempEnvOrig_Fix      /*float* hiTempEnvOrig*/,
    Word16  pCldfbPow_FixScale
);

void calcLoEnvCheckCorrHiLo_Fix(
    Word16 noCols,
    Word16* pFreqBandTable,        /*!<  freqbandTable. */
    Word16* loBuffer_Fix /*float* loBuffer*/,
    Word16* loTempEnv_Fix /*float* loTempEnv*/,
    Word16* loTempEnv_ns_Fix /*  float* loTempEnv_ns*/,
    Word16* hiTempEnvOrig_Fix /*float* hiTempEnvOrig*/,
    Word16*  corrFlag /*int* corrFlag*/
);



/**************************** Moved from er_util.h: ********************************/

/*----------------------------------------------------------------------*
 * PLC: [ACELP: Fade-out]
 * PLC: getLevelSynDeemph: derive on subframe basis the level of LPC
 *      synthesis and deeemphasis based on the given input
 *----------------------------------------------------------------------*/

Word16 getLevelSynDeemph( /*10Q5*/
    Word16        h1Init[],     /* i: input value or vector to be processed */ /* Q15 */
    Word16  const A[],          /* i: LPC coefficients                      */ /* Qx  */
    Word16  const lpcorder,     /* i: LPC order                             */ /* Q0  */
    Word16  const lenLpcExc,    /* i: length of the LPC excitation buffer   */ /* Q0  */
    Word16  const preemph_fac,  /* i: preemphasis factor                    */ /* Q15 */
    Word16  const numLoops,     /* i: number of loops                       */ /* Q0  */
    Word16       *Exp           /* o: exponent of return value Q15          */
);


/*-----------------------------------------------------------------*
 * PLC: [ACELP: general]
 * PLC: high pass filtering
 *-----------------------------------------------------------------*/
void highPassFiltering(
    const   Word16 last_good,    /* i:   short  last classification type                            */
    const   Word16 L_buffer,     /* i:   int    buffer length                                       */
    Word16 exc2[],               /* i/o: Qx     unvoiced excitation before the high pass filtering  */
    const   Word16 hp_filt[],    /* i:   Q15    high pass filter coefficients                       */
    const   Word16 l_fir_fer);   /* i:        high pass filter length                               */


/*----------------------------------------------------------------------------------*
 * PLC: [Common: mode decision]
 * PLC: Decide which Concealment to use. Update pitch lags if needed
 *----------------------------------------------------------------------------------*/
Word16 GetPLCModeDecision( Decoder_State_fx *st
                         );

/* PLC: [Common: Fade-out]
 * PLC: and for PLC fade out */

void minimumStatistics(Word16*       noiseLevelMemory,      /* Q15, internal state */
                       Word16*       noiseLevelIndex,       /* Q0, internal state */
                       Word16*       currLevelIndex,        /* Q0, internal state (circular buffer) */
                       Word16*       noiseEstimate,         /* Q15, previous estimate of background noise */
                       Word16*       lastFrameLevel,        /* Q15, level of the last frame */
                       Word16        currentFrameLevel,     /* Q15, level of the current frame */
                       Word16*       noiseLevelMemory_e,    /* scaling factor for noiseLevelMemory  */
                       Word16  const noiseEstimate_e,       /* exponent of noiseEstimate */
                       Word16*       new_noiseEstimate_e,   /* new exponent of noise Estimate*/
                       Word16* const lastFrameLevel_e,      /* exponent of lastFrameLevel    */
                       Word16        currentFrameLevel_e);  /* exponent of currentFrameLevel */

/*-----------------------------------------------------------------------------------*
 *  PLC: [ACELP: innovative codebook]
 *  PLC: genPlcFiltBWAdapt : deriuve filter coefficients for filtering
 the first pitch cycle, bitrate dependent
 *-----------------------------------------------------------------------------------*/

void genPlcFiltBWAdap(
    Word32 const  sr_core,        /*<! i  : Q0  core sampling rate*/
    Word16       *lpFiltAdapt,    /*<! o  : Q15 filter coefficients for filtering codebooks in case of flc*/
    Word16 const  type,           /*<! i  : Q0  type of filter, 0= lowpass, 1= highpass*/
    Word16 const  alpha           /*<! i  : Q15 fade out factor [0 1) used decrease filter tilt*/
);


/**************************** Moved from er_scale_syn.h: ********************************/

Word16 Damping_fact(                    /* o : damping factor                                       *//*Q14*/
    const Word16 coder_type,        /* i : coding type in last good received frame              */
    const Word16 nbLostCmpt,        /* i : counter of consecutive bfi frames                    */
    const Word16 last_good,         /* i : last good frame class                                */
    const Word16 stab_fac,          /* i : ISF stability factor                                 *//*Q15*/
    Word32 *lp_gainp,         /*i/o: damped pitch gain                                    *//*Q16 Word32!*/
    const Word16 core               /* i : current coding mode                                  */
);

/********************* Moved from vad_basop.h: ********************************/

Word16 vadmin( Word16 a,
               Word16 b
             );

Word32 vad_Sqrt_l( Word32 i_s32Val,
                   Word16 *io_s16Q
                 );

Word32 fft_vad_Sqrt_l( Word32 i_s32Val,
                       Word16 i_s16Q,
                       Word16 *o_s16Q
                     );

T_VAD_EXP VAD_AddExp( T_VAD_EXP i_tExp1,
                      T_VAD_EXP i_tExp2
                    );

Word16 VAD_L_CMP( Word32 s32Mantissa1,
                  Word16 i_tExp1,
                  Word32 s32Mantissa2,
                  Word16 i_tExp2
                );

Word32 VAD_L_ADD( Word32 s32Mantissa1,
                  Word16 i_tExp1,
                  Word32 s32Mantissa2,
                  Word16 i_tExp2,
                  Word16 *s16Exp
                );

Word32 VAD_L_div( Word32 L_var1,
                  Word32 L_var2,
                  Word16 Q_L_var1,
                  Word16 Q_L_var2,
                  Word16 *Q_OUT
                );

Word32 VAD_Log2( Word32 i_s32Val,
                 Word16 i_s16Q
               );

Word16 ffr_getSfWord32( Word32 *vector,
                        Word16 len
                      );

Word32 VAD_Pow( Word32 i_s32Base,
                Word32 i_s32Exp,
                Word16 i_s16BaseQ,
                Word16 i_s16ExpQ,
                Word16 *o_pOuQ
              );

Word32 VAD_Pow2( Word32 i_s32X,
                 Word16 i_s16Q,
                 Word16 *o_pOuQ
               );

Word16 FixSqrt( Word32 i_s32Val,
                Word16 *io_s16Q
              );

void cfftf( Word16* scale,
            complex_32 *c,
            complex_32 *ch,
            const complex_16 *wa
          );


void getLookAheadResSig( Word16 *speechLookAhead, Word16 *A_3Q12, Word16 *res, Word16 L_frame, Word16 numSubFrame );
void updateLSFForConcealment( HANDLE_PLC_ENC_EVS decState, Word16 *isf_14Q1, Word16 m );
void getConcealedLP( HANDLE_PLC_ENC_EVS memDecState, Word16 *AqCon, const Word16 xsfBase[], Word16 last_good, Word16 L_frame );
void getConcealedLSF( HANDLE_PLC_ENC_EVS memDecState, const Word16 xsfBase[], Word16 last_good, Word16 L_frame );

void RecLpcSpecPowDiffuseLc( Word16 *ispq, Word16 *isp_old, Word16 *isfq, Decoder_State_fx *st);
void modify_lsf(
    Word16 *lsf,
    const Word16 n,
    const Word32 sr_core
);


void coderLookAheadInnovation(
    Word16 A_3Q12[],                /* input: coefficients NxAz[M+1]    */
    Word16 *pT,                     /* out:   pitch for all subframe    */
    HANDLE_PLC_ENC_EVS st,       /* i/o:   coder memory state        */
    Word16 *speechLookAhead_Qx,     /* i:   input speech in Q(st->Qold) */
    Word16 *old_exc,                /* i:   input excitation in Q(st->Qold) */
    Word16 L_frame                  /* i:   input frame length           */
);

void encoderSideLossSimulation(
    Encoder_State_fx *st,
    HANDLE_PLC_ENC_EVS hPlc_Ext,
    Word16 *isf_q,                  /* Q1*1.28 */
    Word16 stab_fac,                /* Q15 */
    Word8 calcOnlyISF,
    const Word16 L_frame
);

void enc_prm_side_Info( HANDLE_PLC_ENC_EVS hPlc_Ext, Encoder_State_fx *st );

void GplcTcxEncSetup(Encoder_State_fx *st,
                     HANDLE_PLC_ENC_EVS hPlc_Ext,
                     Word16 Q_new);
Word16 encSideSpecPowDiffuseDetector(
    Word16 *isf_ref,
    Word16 *isf_con,
    Word32 sr_core,
    Word16 *prev_isf4_mean,
    Word8 sw
);

void updateSpecPowDiffuseIdx( Encoder_State_fx *st);


void BASOP_cfft(Word32 *re, Word32 *im, Word16 sizeOfFft, Word16 s, Word16 *scale, Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH]);
void BASOP_cfft16(Word16 *re, Word16 *im, Word16 sizeOfFft, Word16 s, Word16 *scale);
void BASOP_rfft(Word32 *x, Word16 sizeOfFft, Word16 *scale, Word16 isign);
void BASOP_rfft16(Word16 *x, Word16 sizeOfFft, Word16 *scale, Word16 isign);

void open_PLC_ENC_EVS(
    HANDLE_PLC_ENC_EVS hPlcExt,
    Word32 sampleRate
);

void gPLC_encInfo (
    HANDLE_PLC_ENC_EVS self,
    Word32 modeBitrate,
    Word16 modeBandwidth,
    Word16 old_clas,
    Word16 acelp_ext_mode
);

void v_sort(Word16 *r, const Word16 lo, const Word16 up);

void coder_acelp_rf(
    ACELP_config *acelp_cfg_rf,   /*input/output: configuration of the ACELP coding*/
    const Word16 coder_type,      /* input: coding type              */
    const Word16 A[],             /* input: coefficients 4xAz[M+1]   */
    const Word16 Aq[],            /* input: coefficients 4xAz_q[M+1] */
    Word16 speech[],              /* input: speech[-M..lg]           */
    const Word16 voicing[],       /* input: open-loop LTP gain       */
    const Word16 T_op[],          /* input: open-loop LTP lag        */
    Word16 stab_fac,
    Encoder_State_fx *st,
    Word16 target_bits,           /* i/o : coder memory state        */
    const Word16 rf_frame_type,   /* i  : rf_frame_type               */
    Word16 *exc_rf,         /* i/o: pointer to RF excitation    */
    Word16 *syn_rf,         /* i/o: pointer to RF synthesis     */
    Word16 Q_new,
    Word16 shift
);


#endif /*PROT_COM_FX_H */
