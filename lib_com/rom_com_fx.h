/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef ROM_COM_FX_H
#define ROM_COM_FX_H

#include <stdio.h>
#include "stat_com.h"
#include "stat_enc_fx.h"   /* Encoder static structure               */
#include "stat_dec_fx.h"   /* Decoder static structure               */


typedef struct
{
    Word32 fin_fx;       /* input frequency                   Q0	*/
    Word32 fout_fx;      /* output frequency                  Q0	*/
    Word16 fac_num_fx;   /* numerator of resampling factor    Q0	*/
    Word16 fac_den_fx;   /* denominator of resampling factor  Q0	*/
    Word16 lg_out;   /* denominator of resampling factor  Q15	*/
    const Word16 *filter_fx;   /* resampling filter coefficients    Q14	*/
    Word16 filt_len_fx;  /* number of filter coeff.			Q0	*/
    UNS_Word16 flags_fx;     /* flags from config. table          Q0	*/
} Resampling_cfg_fx;

/*-----------------------------------------------------------------*
 * Tables with bit-allocations
 *-----------------------------------------------------------------*/

extern const Word16 LSF_bits_tbl[];                      /* Bit allocation table for end-frame ISF quantizer */
extern const Word16 mid_LSF_bits_tbl[];                  /* Bit allocation table for mid-frame ISF quantizer */
extern const Word16 Es_pred_bits_tbl[];                  /* Bit allocation table for scaled innovation energy prediction */
extern const Word16 gain_bits_tbl[];                     /* Bit allocation table for gain quantizer */
extern const Word16 ACB_bits_tbl[];                      /* Bit allocation table for adaptive codebook (pitch) */
extern const Word16 FCB_bits_tbl[];                      /* Bit allocation table for algebraic (fixed) codebook (innovation) */
extern const Word16 reserved_bits_tbl[];                 /* Bit allocation table for reserved bits */

extern const Word16 ACB_bits_16kHz_tbl[];                /* Bit allocation table for adaptive codebook (pitch) @16kHz */
extern const Word16 FCB_bits_16kHz_tbl[];                /* Bit allocation table for algebraic (fixed) codebook (innovation) @16kHz */
extern const Word16 gain_bits_16kHz_tbl[];               /* Bit allocation table for gain quantizer @16kHz */
extern const Word16 AVQ_bits_16kHz_tbl[];                /* Bit allocation table for AVQ bits @16kHz ACELP, active segments */
extern const Word32 pow2[];                              /* Table with power of 2 values */

extern const Word16 crit_bins[];                         /* (used only in AMR-WB IO mode) */
extern const Word16 mfreq_bindiv_loc[];
extern const Word16 mfreq_bindiv_LD[];
extern const Word16 mfreq_loc_LD[];                      /* LD music post-filter */
extern const Word16 crit_bins_corr_fx[];
/*-----------------------------------------------------------------*
 * Table of bitrates
 *-----------------------------------------------------------------*/

extern const Word32 brate_tbl[];
extern const Word32 acelp_sig_tbl[MAX_ACELP_SIG];


extern const Word16 Nb[NB_SFM];
extern const Word16 LNb[ NB_SFM];
extern const Word16 CSN[NB_SFM];
/*-----------------------------------------------------------------*
 * BC-TCQ frame-end quantization tables
 *-----------------------------------------------------------------*/

extern const Word16 wind_sss_fx[LEN_WIN_SSS];  /*window for modify_sf ana*/
extern const Word16 filter5_39s320_120_fx[];                     /* LP FIR filter for 8kHz signal resampling */
extern const Word16 stable_LSF_fx_16k[];
extern const Word16 UVWB_Ave_fx[];
extern const Word16 UVNB_Ave_fx[];
extern const Word16 SVWB_Ave_fx[];
extern const Word16 SVNB_Ave_fx[];
extern const Word16 IANB_Ave_fx[];
extern const Word16 IAWB_Ave_fx[];
extern const Word16 IAWB2_Ave_fx[];
extern const Word16 GENB_Ave_fx[];
extern const Word16 GEWB_Ave_fx[];
extern const Word16 GEWB2_Ave_fx[];
extern const Word16 TRWB_Ave_fx[];
extern const Word16 TRWB2_Ave_fx[];
extern const Word16 means_wb_cleanspeech_lsf16k0[];
extern const Word16 means_swb_cleanspeech_lsf25k6[];
extern const Word16 means_swb_cleanspeech_lsf32k0[];

extern const Word16 stable_ISP_fx[];
extern const Word16 stable_ISF_fx[];
extern const Word16 stable_LSP_fx[];
extern const Word16 lsp_shb_prev_tbl_fx[];
extern const Word16 sin_table256_fx[];
extern const Word16 b_hp400_fx[];    /* Q12 (/4) */
extern const Word16 a_hp400_fx[];
extern const Word16 fir_6k_7k_fx[];
extern const Word16 no_lead_fx[][MAX_NO_SCALES*2];
extern const Word16 no_lead_p_fx[][MAX_NO_SCALES*2];
extern const Word16 change_pl_order_fx[];
extern const Word16 cng_sort_fx[];
extern const Word8 Ind_Guess[256];
extern const Word16 slope[128];
extern const Word16 table[129];
extern const Word16 Es_pred_qua_5b_fx[32];
extern const Word16 Es_pred_qua_4b_fx[16];
extern const Word16 Es_pred_qua_3b_fx[8];
extern const Word16 Es_pred_qua_4b_no_ltp_fx[16];
extern const Word16 tbl_gain_code_tc_quant_mean[N_GAIN_CODE_TC-1];
extern const Word16 tbl_gain_code_tc_fx[N_GAIN_CODE_TC];
extern const Word16 gain_qua_mless_7b_fx[];
extern const Word16 gain_qua_mless_6b_fx[];
extern const Word16 gain_qua_mless_5b_fx[];
extern const Word16 gp_gamma_1sfr_8b_fx[];
extern const Word16 gp_gamma_1sfr_7b_fx[];
extern const Word16 gp_gamma_1sfr_6b_fx[];
extern const Word16 gp_gamma_2sfr_7b_fx[];
extern const Word16 gp_gamma_2sfr_6b_fx[];
extern const Word16 gp_gamma_3sfr_6b_fx[];
extern const Word16 gp_gamma_4sfr_6b_fx[];

extern const Word16 pred_gain_fx[GAIN_PRED_ORDER];
extern const Word16 t_qua_gain6b_fx[64*2];
extern const Word16 t_qua_gain7b_fx[128*2];

extern const Word16 b_1sfr_fx[];
extern const Word16 b_2sfr_fx[];
extern const Word16 b_3sfr_fx[];
extern const Word16 b_4sfr_fx[];

extern const Word16 inter4_2_fx[];

extern const Word16 Gamma_19661_Tbl_fx[];
extern const Word16 Gamma_29491_Tbl[];
extern const Word16 pwf_fx[17];

extern const Word32 inv[1000];

extern const Word16 * const ModeMeans_fx[];
extern const Word16 * const Predictors_fx[];

extern const Word16 tbl_mid_voi_wb_1b_fx[];
extern const Word16 tbl_mid_voi_wb_2b_fx[];
extern const Word16 tbl_mid_voi_wb_3b_fx[];
extern const Word16 tbl_mid_voi_wb_4b_fx[];
extern const Word16 tbl_mid_voi_wb_5b_fx[];
extern const Word16 tbl_mid_voi_wb_6b_fx[];

extern const Word16 tbl_mid_unv_wb_2b_fx[];
extern const Word16 tbl_mid_unv_wb_3b_fx[];
extern const Word16 tbl_mid_unv_wb_4b_fx[];
extern const Word16 tbl_mid_unv_wb_5b_fx[];
extern const Word16 tbl_mid_unv_wb_6b_fx[];

extern const Word16 tbl_mid_gen_wb_2b_fx[];
extern const Word16 tbl_mid_gen_wb_3b_fx[];
extern const Word16 tbl_mid_gen_wb_4b_fx[];
extern const Word16 tbl_mid_gen_wb_5b_fx[];
extern const Word16 tbl_mid_gen_wb_6b_fx[];

extern const Word16 CBsizes_fx[];
extern const Word16 CBbits_fx[];
extern const Word16 CBbits_p_fx[];
extern const Word16 BitsVQ_fx[];
extern const Word16 BitsVQ_p_fx[];


extern const Word16 Predictor0_fx[];
extern const Word16 Predictor1_fx[];
extern const Word16 Predictor2_fx[];
extern const Word16 Predictor3_fx[];
extern const Word16 Predictor4_fx[];
extern const Word16 Predictor5_fx[];
extern const Word16 Predictor6_fx[];
extern const Word16 Predictor7_fx[];
extern const Word16 Predictor8_fx[];

extern const Word16 interpol_frac_mid_16k_fx[NB_SUBFR16k*3];
extern const Word16 interpol_frac_mid_fx[NB_SUBFR*3];

extern const Word16 * const Quantizers_p_fx[];
extern const Word16 * const Quantizers_fx[];
extern const Word16 CNG_SN1_fx[];
extern const Word16 CB_fx[];
extern const Word16 CB_p_fx[];

extern const Word16 sigma_p_fx[][16];
extern const Word16 sigma_fx[][16];

extern const Word16 pi0_fx[];
extern const Word32 table_no_cv_fx[];
extern const Word16 pl_par_fx[];	       /* 1 if even number of signs */
extern const Word16 C_fx[LATTICE_DIM+1][LATTICE_DIM+1];
extern const Word16 no_vals_ind_fx[NO_LEADERS][MAX_NO_VALS];
extern const Word16 no_vals_fx[NO_LEADERS];

extern const Word16 vals_fx[NO_LEADERS][MAX_NO_VALS];
extern const Word16 scales_fx[][MAX_NO_SCALES*2];
extern const Word16 scales_p_fx[][MAX_NO_SCALES*2];
extern const Word16 predmode_tab[][6];
extern const Word16 min_lat_bits_SN_fx[];
extern const Word16 min_lat_bits_pred_fx[];
extern const Word16 offset_in_lvq_mode_SN_fx[][21];

extern const Word16 offset_in_lvq_mode_pred_fx[][32];

extern const Word16 offset_lvq_modes_SN_fx[];
extern const Word16 offset_lvq_modes_pred_fx[];
extern const Word16 inv_sigma_p_fx[][16];
extern const Word16 inv_sigma_fx[][16];
extern const Word16 perm_fx[][4];
extern const Word16 pl_fx[];

/*----------------------------------------------------------------------------------*
 * ISF quantization (AMR-WB IO mode)
 *----------------------------------------------------------------------------------*/
extern const Word16 Indirect_dico1[SIZE_BK1];

extern const Word16 mean_isf_amr_wb_fx[M];                  /* Mean ISF vector (only in AMR-WB IO mode) */
extern const Word16 mean_isf_noise_amr_wb_fx[];             /* Mean ISF vector for SID frame (only in AMR-WB IO mode) */

extern const Word16 dico1_isf_fx[];                         /* ISF codebook - common 1st stage, 1st split (only in AMR-WB IO mode) */
extern const Word16 dico2_isf_fx[];                         /* ISF codebook - common 1st stage, 2nd split (only in AMR-WB IO mode) */

extern const Word16 dico21_isf_46b_fx[];                    /* ISF codebook - 46b, 2nd stage, 1st split (only in AMR-WB IO mode) */
extern const Word16 dico22_isf_46b_fx[];                    /* ISF codebook - 46b, 2nd stage, 2st split (only in AMR-WB IO mode) */
extern const Word16 dico23_isf_46b_fx[];                    /* ISF codebook - 46b, 2nd stage, 3rd split (only in AMR-WB IO mode) */
extern const Word16 dico24_isf_46b_fx[];                    /* ISF codebook - 46b, 2nd stage, 4th split (only in AMR-WB IO mode) */
extern const Word16 dico25_isf_46b_fx[];                    /* ISF codebook - 46b, 2nd stage, 5th split (only in AMR-WB IO mode) */

extern const Word16 dico21_isf_36b_fx[];                    /* ISF codebook - 36b, 2nd stage, 1st split (only in AMR-WB IO mode) */
extern const Word16 dico22_isf_36b_fx[];                    /* ISF codebook - 36b, 2nd stage, 2nd split (only in AMR-WB IO mode) */
extern const Word16 dico23_isf_36b_fx[];                    /* ISF codebook - 36b, 2nd stage, 3rd split (only in AMR-WB IO mode) */

extern const Word16 dico1_ns_28b_fx[];                      /* ISF codebook for SID frames - 28b, 1st split */
extern const Word16 dico2_ns_28b_fx[];                      /* ISF codebook for SID frames - 28b, 2nd spilt */
extern const Word16 dico3_ns_28b_fx[];                      /* ISF codebook for SID frames - 28b, 3rd spilt */
extern const Word16 dico4_ns_28b_fx[];                      /* ISF codebook for SID frames - 28b, 4th spilt */
extern const Word16 dico5_ns_28b_fx[];                      /* ISF codebook for SID frames - 28b, 5th spilt */

/*-----------------------------------------------------------------*
 * BC-TCVQ frame-end quantization tables (BC-TCVQ is used in Voiced mode for 16kHz isf)
 *-----------------------------------------------------------------*/
extern const Word16 NTRANS[4][NUM_STATE];
extern const Word16 NTRANS2[4][NUM_STATE];

extern const Word16 BC_TCVQ_BIT_ALLOC_40B[];
extern const Word16 FixBranch[4][4][N_STAGE_VQ - 4];

extern const Word16 AR_IntraCoeff_fx[N_STAGE_VQ-1][2][2];
extern const Word16 SN_IntraCoeff_fx[N_STAGE_VQ-1][2][2];

extern const Word16 scale_ARSN_fx[];
extern const Word16 scale_inv_ARSN_fx[];

extern const Word16 AR_TCVQ_CB_SUB1_fx[2][128][2];
extern const Word16 AR_TCVQ_CB_SUB2_fx[2][64][2];
extern const Word16 AR_TCVQ_CB_SUB3_fx[4][32][2];

extern const Word16 SN_TCVQ_CB_SUB1_fx[2][128][2];
extern const Word16 SN_TCVQ_CB_SUB2_fx[2][64][2];
extern const Word16 SN_TCVQ_CB_SUB3_fx[4][32][2];

extern const Word16 AR_SVQ_CB1_fx[32][8];
extern const Word16 AR_SVQ_CB2_fx[16][8];


extern const Word16 interpol_frac_mid_relaxprev_12k8_fx[NB_SUBFR*3];
extern const Word16 interpol_frac_mid_FEC_fx[NB_SUBFR*3];
extern const Word16 interpol_frac_mid_relaxprev_pred_12k8_fx[NB_SUBFR*3];
extern const Word16 interpol_frac_mid_relaxprev_16k_fx[NB_SUBFR16k*3];
extern const Word16 interpol_frac_mid_16k_FEC_fx[NB_SUBFR16k*3];
extern const Word16 interpol_frac_mid_relaxprev_pred_16k_fx[NB_SUBFR16k*3];
extern const Resampling_cfg_fx resampling_cfg_tbl_fx[];

extern const Word16 filter_LP15_180H_fx[];  /* Sincfilt.m: N=180*2+1, Fmin=0, Fmax=1/15, hann( N )' */
extern const Word16 filter_LP3_90H_fx[];    /* Sincfilt.m: N=90*2+1, Fmin=0, Fmax=1/3, hann( N )' */
extern const Word16 filter_LP12_180H_13b_fx[];  /* Sincfilt.m: N=180*2+1, Fmin=0, Fmax=1/12, hann( N )' */
extern const Word16 filter_LP15_180H_13b_fx[];  /* Sincfilt.m: N=180*2+1, Fmin=0, Fmax=1/15, hann( N )' */
extern const Word16 filter_LP15_360H_13b_fx[];  /* Sincfilt.m: N=360*2+1, Fmin=0, Fmax=1/15, hann( N )' */
extern const Word16 filter_LP24_180H_13b_fx[];
extern const Word16 filter_LP24_180H_fx[];

extern const Word16 inter4_1_fx[];

extern const Word16 Low_H[L_SUBFR];
extern const Word16 Mid_H[L_SUBFR];
extern const Word16 phs_tbl_dec[];
extern const Word16 FFT_reorder_64[];
extern const Word16 FFT_reorder_256[];

extern const Word16 tbl_gain_trans_tc_fx[];
extern const Word16 Glottal_cdbk_fx[];

extern const Word16 gaus_dico_fx[];


extern const Word16 edct_table_80_fx[];                     /* EDCT */
extern const Word16 edct_table_120_fx[];                    /* EDCT */
extern const Word16 edct_table_100_fx[];                    /* EDCT */
extern const Word16 edct_table_320_fx[];                    /* EDCT */
extern const Word16 edct_table_480_fx[];                    /* EDCT */
extern const Word16 edct_table_600_fx[];                    /* EDCT */
extern const Word16 edct_table_128_fx[];                    /* EDCT */
extern const Word16 edct_table_160_fx[];                    /* EDCT */
extern const Word16 edct_table_40_fx[];                     /* EDCT */
extern const Word16 edct_table_20_fx[];                     /* EDCT */
extern const Word16 edct_table_10_fx[];
extern const Word16 edct_table_16_fx[];
extern const Word16 edct_table_32_fx[];
extern const Word16 edct_table_60_fx[];
extern const Word16 edct_table_64_fx[];
extern const Word16 edct_table_192_fx[];
extern const Word16 edct_table_200_fx[];
extern const Word16 edct_table_240_fx[];
extern const Word16 edct_table_256_fx[];
extern const Word16 edct_table_400_fx[];

/*------------------------------------------------------------------------------*
 * FFT transform
 *------------------------------------------------------------------------------*/
extern const Word16 FFT_REORDER_128[];
extern const Word16 FFT_REORDER_512[];
extern const Word16 FFT_REORDER_1024[];
extern const Word16 FFT_W64[];
extern const Word16 FFT_W128[];
extern const Word16 FFT_W256[];
extern const Word16 FFT_W512[];
extern const Word16 sincos_t_rad3_fx[];
extern const Word16 sincos_t_ext_fx[];

extern const Word16 Ip_fft16_fx[6];
extern const Word16 w_fft16_fx[8];

extern const Word16 Ip_fft8_fx[6];
extern const Word16 w_fft8_fx[4];

extern const Word16 Ip_fft32_fx[6];
extern const Word16 w_fft32_fx[16];

extern const Word16 Ip_fft64_fx[6];
extern const Word16 w_fft64_fx[32];

extern const Word16 Ip_fft4_fx[6];
extern const Word16 w_fft4_fx[2];

extern const Word16 Ip_fft128_fx[10];
extern const Word16 w_fft128_fx[64];

extern const Word16 ip_edct2_64_fx[6];
extern const Word16 w_edct2_64_fx[80];
extern const Word32 w_fft64_16fx[32];
extern const Word16 Odx_fft64_16fx[64];
extern const Word16 Ip_fft64_16fx[6];
extern const Word16 Ip_fft32_16fx[6];
extern const Word32 w_fft128_16fx[64];
extern const Word16 Ip_fft128_16fx[10];
extern const Word16 Idx_dortft320_16fx[320];
extern const Word16 Ip_fft32_16fx[6];
extern const Word32 w_fft32_16fx[16];
extern const Word16 edct_table_160_16fx[160];
extern const Word16 edct_table_128_16fx[128];
extern const Word16 edct_table_320_16fx[320];
extern const Word16 mean_m_fx[];

extern const Word16 YG_dicMR_4_fx[];
extern const Word16 YG_dicMR_2_fx[];
extern const Word16 YG_dicMR_3_fx[];
extern const Word16 YG_dicMR_1_fx[];
extern const Word16 YG_mean16_fx[];
extern const Word16 YG_dic1_2_fx[];
extern const Word16 YG_dic1_1_fx[];
extern const Word16 YG_mean14_fx[];
extern const Word16 YGain_dic3_LR_fx[];/*Q13 */
extern const Word16 YGain_dic2_LR_fx[];/*Q13 */

extern const Word16 YGain_mean_LR_fx[];
extern const Word16 Gain_mean_dicHR_fx[];
extern const Word16 Gain_mean_dic_fx[];
extern const Word16 mean_gain_dic_fx[];
extern const Word16 Gain_mean_fx[];
extern const Word16 Gain_dic3_NBHR_fx[];
extern const Word16 Gain_dic2_NBHR_fx[];
extern const Word16 Gain_dic2_NB_fx[];
extern const Word16 Gain_dic3_NB_fx[];
extern const Word16 Mean_dic_NB_fx[];
extern const Word16 Gain_mean_dicNB_fx[];
extern const Word16 Gain_meanNB_fx[];
/*extern const Word16 dic_gp_13_fx[]; */
extern const Word16 dic_gp_fx[];
extern const Word16 mean_gp_fx[];
extern const Word16 Gain_dic1_NB_fx[];
extern const Word16 YGain_dic1_LR_fx[];

extern const Word16 mfreq_loc_Q2fx[];
extern const Word16 mfreq_loc_div_25[];
extern const Word16 mfreq_bindiv_loc_fx[];
extern const Word16 sm_table_fx[];
extern const Word16 GSC_freq_bits[];

extern const Word16 gsc_sfm_start[];
extern const Word16 gsc_sfm_end[];
extern const Word16 gsc_sfm_size[];

extern const Word16 sincos_t_fx[161];
extern const Word32 GSC_freq_bits_fx[];

extern const Word16 *scalar_tbls_fx[4];
extern const Word16 scalar_4bit_fx[16];
extern const Word16 scalar_3bit_fx[8];
extern const Word16 scalar_2bit_fx[4];
extern const Word16 scalar_1bit_fx[2];
extern const Word16 Mean_isf_wb[];
extern const Word16 interpol_isp_amr_wb_fx[];
extern const Word16 interpol_frac_16k_fx[NB_SUBFR16k];
extern const Word16 interpol_frac_fx[NB_SUBFR];
extern const Word16 CNG_burst_att_fx[6][8];
extern const Word32 pow2_fx[];
extern const Word32 CNG_details_codebook_fx[64][NUM_ENV_CNG];

extern const Word16 deem_tab_fx[];                          /* HF BWE - de-emphasis coefficients       */
extern const Word16 filt_hp_fx[];                           /* HF BWE - HP filter coefficients in freq. domain */
extern const Word16 HP_gain_fx[];                           /* HF BWE - quantization table for 23.85 */
extern const Word16 fir_6k_8k_fx[];                         /* HF BWE - band-pass filter coefficients */
extern const Word16 exp_tab_q_fx[];
extern const Word16 exp_tab_p_fx[];
extern const Word16 swb_bwe_trans_subband_width_fx[];
extern const Word16 sqrt_swb_bwe_trans_subband_width_fx[];
extern const Word16 sqrt_swb_bwe_subband_fx_L1[];
extern const Word16 sqrt_swb_bwe_subband_fx_L2[];
extern const Word16 smooth_factor_fx[];
extern const Word16 swb_bwe_sm_subband_fx[];

extern const Word16 swb_bwe_trans_subband_fx[];
extern const Word16 swb_bwe_subband_fx[];
extern const Word16 F_2_5_fx[64];

extern const Word16 swb_inv_bwe_subband_width_fx[];
extern const Word16 w_NOR_fx[];
extern const Word16 sin_switch_8[15];
extern const Word16 sin_switch_16[30];
extern const Word16 sin_switch_32[60];
extern const Word16 sin_switch_48[90];
/* sinus table for overlapadd */
/*extern const Word16 sin_switch_96_16_fx[156]; */
/*extern const Word16 sin_switch_96_12_fx[108]; */
extern const Word16 one_on_win_48k_fx[210]; /*Q14 */
extern const Word16 one_on_win_8k_16k_48k_fx[70]; /*Q14 */
extern const Word16 TRWB2_Ave_fx[16];
extern const Word16 TRWB_Ave_fx[16];
/*------------------------------------------------------------------------------*
 * ACEPL/HQ core switching tables
 *------------------------------------------------------------------------------*/
extern const Word16 hp12800_32000_fx[];
extern const Word16 hp16000_32000_fx[];
extern const Word16 hp12800_48000_fx[];
extern const Word16 hp16000_48000_fx[];
extern const Word16 hp12800_16000_fx[];
extern const Word16 inner_frame_tbl_fx[];


extern const Word16 EnvCdbk11_fx [];
extern const Word16 EnvCdbk1st_fx [];
extern const Word16 EnvCdbk2nd_fx [];
extern const Word16 EnvCdbk3rd_fx [];
extern const Word16 EnvCdbk4th_fx [];
extern const Word16 Env_TR_Cdbk1_fx [];
extern const Word16 Env_TR_Cdbk2_fx [];
extern const Word16 EnvCdbkFB_fx [];
extern const Word16 Mean_env_fx[];
extern const Word16 Mean_env_tr_fx[];
extern const Word16 Mean_env_fb_fx[];

extern const Word32 PI_select_table_fx[23][8];              /* selection table for Pulse indexing          */
extern const Word32 PI_offset_fx[8][8];                     /* offset table for Pulse indexing             */
extern const Word16 PI_factor_fx[];                         /* PI factor table for Pulse indexing          */

extern const short Num_bands_NB[];


extern const Word16 hvq_cb_search_overlap24k[17];
extern const Word16 hvq_cb_search_overlap32k[21];

extern const Word16 hvq_pg_huff_offset[NUM_PG_HUFFLEN];
extern const Word16 hvq_pg_huff_thres[NUM_PG_HUFFLEN];
extern const Word16 hvq_pg_huff_tab[32];

extern const Word16 hvq_cp_huff_len[52];
extern const Word16 hvq_cp_huff_val[52];
extern const Word16 hvq_cp_layer1_map5[HVQ_CP_MAP_LEN];

extern const Word16 hvq_cp_huff_thres[HVQ_CP_HUFF_NUM_LEN];
extern const Word16 hvq_cp_huff_offset[HVQ_CP_HUFF_NUM_LEN];
extern const Word16 hvq_cp_huff_tab[52];

extern const Word16 pgain_huffnorm[32];
extern const Word16 pgain_huffsizn[32];

extern const Word16 huffnorm_tran[32];
extern const Word16 huffsizn_tran[32];

extern const Word16 resize_huffnorm[32];
extern const Word16 resize_huffsizn[32];

extern const Word16 huffsizn[32];

extern const Word16 lsf_q_num_bits[];                    /* Size of each element of the above, in bits */
extern const Word16 lsf_q_num_bits_rf[];                 /* Size of each element of the above, in bits */

/*------------------------------------------------------------------------------*
 * AVQ - RE8 tables
 *------------------------------------------------------------------------------*/

extern const Word16 select_table22_fx[][9];
extern const Word16 vals_a_fx[][4];                         /* value of leader element */
extern const Word16 vals_q_fx[][4];                         /* code parameter for every leader */
extern const UWord16 Is_fx[];                               /* codebook start address for every leader */
extern const Word16 A3_fx[];                                /* A3 - Number of the absolute leaders in codebook Q3 */
extern const Word16 A4_fx[];                                /* A4 - Number of the absolute leaders in codebook Q4 */
extern const UWord16 I3_fx[];                               /* I3 - Cardinality offsets for absolute leaders in Q3 */
extern const UWord16 I4_fx[];                               /* I4 - Cardinality offset for absolute leaders in Q4 */

extern const Word16 Da_nq_fx[];
extern const Word16 Da_pos_fx[], Da_nb_fx[];
extern const Word16 Da_id_fx[];

extern const Word16 Assym_window_W16fx[];
extern const Word16 assym_window_16k_fx[];
extern const Word16 Hamcos_Window[];

extern const Word16 overlap_coefs_fx_0_9[NSV_OVERLAP*WIDTH_BAND]; /* in Q15 */

extern const Word16 swb_hr_env_code1_fx[]; /* HR SWB BWE - envelope Q table - first two subabnds in non-transient frames */
extern const Word16 swb_hr_env_code2_fx[]; /* HR SWB BWE - envelope Q table - second two subabnds in non-transient frames*/
extern const Word16 swb_hr_env_code3_fx[]; /* HR SWB BWE - envelope Q table - two subands in transient frames */

extern const Word16 short_window_48kHz_fx[L_FRAME48k/2/2];
extern const Word16 short_window_32kHz_fx[L_FRAME32k/2/2];
extern const Word16 short_window_16kHz_fx[L_FRAME16k/2/2];
extern const Word16 short_window_8kHz_fx[L_FRAME8k/2/2];


extern const Word16 Hilbert_coeffs_fx[4*NUM_HILBERTS][HILBERT_ORDER1+1];
extern const Word16 win_flatten_4k_fx[];                 /* Window for calculating whitening filter for WB excitation */

extern const Word32 no_lead[][MAX_NO_SCALES*2];
extern const Word32 no_lead_p[][MAX_NO_SCALES*2];

extern const Word16 min_lat_bits_SN_fx[];
extern const Word16 min_lat_bits_pred_fx[];

extern const Word16 fb_bwe_sm_subband[];
extern const Word16 fb_bwe_subband[];

extern const Word16 sfm_width[20];
extern const Word16 a_table_fx[20];

/* HQ inner_frame signallisation table */
extern const Word16 inner_frame_tbl[];


/*------------------------------------------------------------------------------*
 * SWB TBE tables
 *------------------------------------------------------------------------------*/

extern const Word16 skip_bands_SWB_TBE[];                /* bands for SWB TBE quantisation */
extern const Word16 skip_bands_WB_TBE[];                 /* bands for WB TBE quantisation  */

extern const Word16 swb_lsp_prev_interp_init[];

extern const Word16 allpass_poles_3_ov_2[];                /* All pass 3 over 2 filter coefficients */
extern const Word16 decimate_3_ov_2_lowpass_num[];
extern const Word16 decimate_3_ov_2_lowpass_den[];

/* Band structure */
extern const Word16 band_len[];
extern const Word16 band_start[];
extern const Word16 band_end[];
extern const Word16 band_len_wb[];
extern const Word16 band_start_wb[];
extern const Word16 band_end_wb[];
extern const Word16 band_len_harm[];
extern const Word16 band_start_harm[];
extern const Word16 band_end_harm[];
extern const Word16 dicnlg2[40];

extern const Word16 intl_bw_16[N_INTL_GRP_16];
extern const Word16 intl_bw_32[N_INTL_GRP_32];
extern const Word16 intl_bw_48[N_INTL_GRP_48];
extern const Word16 intl_cnt_16[N_INTL_GRP_16];
extern const Word16 intl_cnt_32[N_INTL_GRP_32];
extern const Word16 intl_cnt_48[N_INTL_GRP_48];
extern const Word16 norm_order_48[NB_SFM];
extern const Word16 norm_order_32[SFM_N_SWB];
extern const Word16 norm_order_16[SFM_N_WB];

extern const Word16 gaus_dico_swb_fx[];                     /* Gaussian codebook for SWB TBE */
extern const Word16 win_flatten_fx[];                       /* Window for calculating whitening filter for SHB excitation */
extern const Word16 AP1_STEEP_FX[];                         /* All pass filter coeffs for interpolation and decimation by a factor of 2 */
extern const Word16 AP2_STEEP_FX[];                         /* All pass filter coeffs for interpolation and decimation by a factor of 2 */
extern const Word16 cos_fb_exc_fx[32];
extern const Word16 window_wb_fx[];
extern const Word16 subwin_wb_fx[];                         /* Short overlap add window for SHB excitation used in anal and synth  */
extern const Word16 window_shb_fx[];                        /* Overlap add window for SHB excitation used in anal and synth */
extern const Word16 window_shb_32k_fx[];
extern const Word16 subwin_shb_fx[];                        /* Short overlap add window for SHB excitation used in anal and synth  */
extern const Word16 * const lsf_q_cb_fx[];                  /* Codebook array for each LSF */
extern const Word16 lsf_q_cb_size_fx[];                     /* Size of each element of the above */
extern const Word16 * const lsf_q_cb_rf_fx[];               /* Codebook array for each LSF */
extern const Word16 grid_smoothing_fx[];                    /* LSF mirroring smoothing table */
extern const Word16 mirror_point_q_cb_fx[];                 /* LSF mirroring point codebook */
extern const Word16 lbr_wb_bwe_lsfvq_cbook_2bit_fx[];
extern const Word16 swb_tbe_lsfvq_cbook_8b[];
extern const Word32 SHBCB_FrameGain16_fx[];
extern const Word16 SHBCB_SubGain5bit_fx[];                 /* 5 bit Quantizer table for SHB gain shapes */
extern const Word16 SHBCB_SubGain5bit_12_fx[];              /* 5 bit Quantizer table for SHB gain shapes */
extern const Word16 HBCB_SubGain5bit_fx[];                  /* 5-bit TD WB BWE temporal shaping codebook */
extern const Word32 SHBCB_FrameGain64_fx[];                 /* 6 bit Quantizer table for SHB overall gain */
extern const Word32 SHBCB_FrameGain16_fx[];
extern const Word16 full_band_bpf_1_fx[][5];
extern const Word16 full_band_bpf_2_fx[][5];
extern const Word16 full_band_bpf_3_fx[][5];
extern const Word16 lsf_q_cb_4b_fx[];                       /* 4 bit differential scalar quantizer table for TD SWB BWE LSFs 1 and 2*/
extern const Word16 lsf_q_cb_3b_fx[];                       /* 3 bit differential scalar quantizer table for TD SWB BWE LSFs 3, 4 and 5*/

extern const Word16 cos_coef_new[4];
extern const Word16 pwAlpha[10];

extern const Word16 interpol_frac_shb[NB_SUBFR*2];

extern const Word16 wb_bwe_lsfvq_cbook_8bit_fx[];
extern const Word16 lsf_grid_fx[4][5];                      /* LSF mirroring adjustment grid */
extern const Word16 ola_win_shb_switch_fold_fx[];
extern const Word16 win_lpc_hb_wb_fx[];
extern const Word16 wac_h[];
extern const Word16 wac_l[];
extern const Word16 wac_swb_h[LPC_SHB_ORDER];
extern const Word16 wac_swb_l[LPC_SHB_ORDER];
extern const Word16 lpc_weights_fx[];
extern const Word16 win_lpc_shb_fx[];                       /* Window for calculating SHB LPC coeffs */
extern const Word16 cos_table[512];
extern const Word16 cos_diff_table[512];
extern const Word16 Ip_fft256_fx[10];
extern const Word16 w_fft256_fx[128];
extern const Word16 Ip_fft512_fx[18];
extern const Word16 w_fft512_fx[256];
extern const Word16 sinq_16k[];
extern const Word16 sinq_32k[];
extern const Word16 sinq_48k[];
extern const Word16 Asr_LP32_fx[41];
extern const Word16 Asr_LP16_fx[21];
extern const Word16 Asr_LP48_fx[61];
extern const Word16 window_8_16_32kHz_fx[];
extern const Word16 window_48kHz_fx[];
extern const Word16 window_256kHz[];
extern const Word16 half_overlap_25[];
extern const Word16 half_overlap_48[];
extern const Word16 half_overlap_int[];
extern const Word16 small_overlap_25[];
extern const Word16 small_overlap_48[];
extern const Word16 small_overlap_int[];

/* SC-VBR: NELP filter coefficients */

extern const Word16 txlpf1_num_coef_fx[11];
extern const Word16 txlpf1_den_coef_fx[11];
extern const Word16 txhpf1_num_coef_fx[11];
extern const Word16 txhpf1_den_coef_fx[11];


extern const Word16 shape1_num_coef_fx[11];
extern const Word16 shape1_den_coef_fx[11];
extern const Word16 shape2_num_coef_fx[11];
extern const Word16 shape2_den_coef_fx[11];
extern const Word16 shape3_num_coef_fx[11];
extern const Word16 shape3_den_coef_fx[11];
extern const Word16 bp1_num_coef_wb_fx[5];
extern const Word16 bp1_den_coef_wb_fx[5];
extern const Word16 bp1_num_coef_nb_fx[13];
extern const Word16 bp1_den_coef_nb_fx[13];

extern const Word16 bp1_num_coef_nb_fx_order7[8];
extern const Word16 bp1_den_coef_nb_fx_order7[8];

extern const Word16 UVG1CB_WB_FX[UVG1_CBSIZE][2];
extern const Word16 UVG2CB1_WB_FX[UVG2_CBSIZE][5];
extern const Word16 UVG2CB2_WB_FX[UVG2_CBSIZE][5];

extern const Word16 UVG1CB_NB_FX[UVG1_CBSIZE][2];
extern const Word16 UVG2CB1_NB_FX[UVG2_CBSIZE][5];
extern const Word16 UVG2CB2_NB_FX[UVG2_CBSIZE][5];

extern const Word16 frac_4sf_fx[NB_SUBFR +2];

extern const Word16 erb_WB_fx[NUM_ERB_WB + 1];
extern const Word16 erb_NB_fx[NUM_ERB_NB + 1];

extern const Word16 AmpCB1_WB_fx[640];
extern const Word16 pwf78_fx[17];

extern const Word16 AmpCB2_WB_fx[64*(NUM_ERB_WB-13)];

extern const Word16 AmpCB1_NB_fx[640];

extern const Word16 AmpCB2_NB_fx[64*(NUM_ERB_NB-13)];

extern const Word16 PowerCB_WB_fx[128];
extern const Word16 PowerCB_NB_fx[128];
extern const Word16 sinc_fx[8][12];
extern const Word16 hvq_cb_search_overlap24k[17];
extern const Word16 hvq_cb_search_overlap32k[21];

extern const Word32 inverse_table[];

extern const Word16 post_dct_wind_fx[];
extern const Word16 mfreq_loc_LD_fx[];
extern const Word16 inv_mfreq_bindiv_LD_fx[];
extern const Word16 inv_mfreq_bindiv_LD_M1_fx[];
/*extern const Word16 inv_mfreq_M1_SC_FACT_fx[]; */
extern const Word16 sc_qnoise_fx[];
extern const Word16 MAX_SNR_SNR1_tab_FX[MBANDS_GN_LD];
extern const Word16 INV_MAX_SNR_tab_FX[MBANDS_GN_LD];

extern const Word16 W_DTX_HO_FX[HO_HIST_SIZE];
extern const Word16 HO_ATT_FX[5];


/*----------------------------------------------------------------------------------*
 * SWB HR BWE
 *----------------------------------------------------------------------------------*/

extern const Word16 wscw16q15_fx[];
extern const Word16 wscw16q15_8_fx[];
extern const Word16 wscw16q15_16_fx[];
extern const Word16 wscw16q15_32_fx[];

extern const Word16 crit_bands_loc_fx[];
extern const Word16 lag_h[];
extern const Word16 lag_l[];

extern const Word16 hvq_class_c_fx[];
extern const Word32 dicn_fx[40];
extern const Word16 sqac_headroom_fx[146];
extern const Word16 inv_N_fx[32];
extern const Word16 hvq_bwe_fac_fx[16];
extern const Word16 att_step_fx[4];
extern const Word16 gain_att_fx[40];
extern const Word16 stab_trans_fx[];
extern const Word16 env_stab_tp_fx[2][2];
extern const Word16 subf_norm_groups_fx[4][11];
extern const Word32 dicn_pg_fx[];
extern const Word16 hvq_peak_cb_fx[];
extern const Word16 rat_fx[SFM_N_WB];
extern const Word16 fb_inv_bwe_subband_width_fx[];
extern const Word16 fb_smooth_factor_fx[];
extern const Word16 hvq_thr_adj_fx[];
extern const Word16 hvq_index_mapping_fx[4];
extern const Word16 hq_nominal_scaling_inv[];
extern const Word16 hq_nominal_scaling[];
extern const Word16 band_len_idx[];
extern const Word16 band_len_ener_shift[];
extern const Word16 fine_gain_pred_sqrt_bw[];
extern const Word16* finegain_fx[5];
extern const Word32 SQRT_DIM_fx[];
extern const Word16 gain_cb_size[];

/* PVQ tables */
extern const UWord32 exactdivodd_fx[ODD_DIV_SIZE_FX];


extern const Word32 thren_fx[39];

typedef struct
{
    Word16 bands;
    Word16 bw;
    const Word16 *band_width;
    Word32 L_qint;
    Word16 eref;
    Word16 bit_alloc_weight;
    Word16 gqlevs;
    Word16 Ngq;
    Word16 p2a_bands;
    Word16 p2a_th;
    Word16 pd_thresh;
    Word16 ld_slope;
    Word16 ni_coef;
} Xcore_Config_fx;

/* LR-MDCT configuration tables */
extern const Xcore_Config_fx xcore_config_8kHz_007200bps_long_fx;
extern const Xcore_Config_fx xcore_config_8kHz_008000bps_long_fx;
extern const Xcore_Config_fx xcore_config_8kHz_009600bps_long_fx;
extern const Xcore_Config_fx xcore_config_8kHz_013200bps_long_fx;
extern const Xcore_Config_fx xcore_config_8kHz_016400bps_long_fx;

extern const Xcore_Config_fx xcore_config_8kHz_007200bps_short_fx;
extern const Xcore_Config_fx xcore_config_8kHz_008000bps_short_fx;
extern const Xcore_Config_fx xcore_config_8kHz_009600bps_short_fx;
extern const Xcore_Config_fx xcore_config_8kHz_013200bps_short_fx;
extern const Xcore_Config_fx xcore_config_8kHz_016400bps_short_fx;

extern const Xcore_Config_fx xcore_config_16kHz_013200bps_long_fx;
extern const Xcore_Config_fx xcore_config_16kHz_016400bps_long_fx;

extern const Xcore_Config_fx xcore_config_16kHz_013200bps_short_fx;
extern const Xcore_Config_fx xcore_config_16kHz_016400bps_short_fx;

extern const Xcore_Config_fx xcore_config_32kHz_013200bps_long_fx;
extern const Xcore_Config_fx xcore_config_32kHz_016400bps_long_fx;

extern const Xcore_Config_fx xcore_config_32kHz_013200bps_short_fx;
extern const Xcore_Config_fx xcore_config_32kHz_016400bps_short_fx;

extern const Word16 Odx_fft32_5[32];
extern const Word16 Idx_dortft160[160];


extern const Word16 nextstate[STATES][2];

extern const Word16 fine_gain_bits[];

/*----------------------------------------------------------------------------------*
 * SWB BWE for LR MDCT core
 *----------------------------------------------------------------------------------*/
/* HQ_NORMAL mode */
extern const Word16 subband_offsets_12KBPS[NB_SWB_SUBBANDS];
extern const Word16 subband_offsets_16KBPS[NB_SWB_SUBBANDS];
extern const Word16 bits_lagIndices_fx[NB_SWB_SUBBANDS];
extern const Word16 subband_search_offsets_fx[NB_SWB_SUBBANDS];
extern const Word16 bw_SPT_tbl[2][SPT_SHORTEN_SBNUM];

/* HQ_HARMONIC mode */
extern const Word16 bits_lagIndices_mode0_Har_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB];
extern const Word16 subband_offsets_sub5_13p2kbps_Har_fx[NB_SWB_SUBBANDS_HAR];
extern const Word16 subband_search_offsets_13p2kbps_Har_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB];
extern const Word16 subband_offsets_sub5_16p4kbps_Har_fx[NB_SWB_SUBBANDS_HAR];
extern const Word16 subband_search_offsets_16p4kbps_Har_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB] ;
extern const Word16 gain_table_fx[NB_SWB_SUBBANDS];


extern const Word16 inv_tbl_fx[];
extern const Word32 table_logcum_fx[563];
extern const Word32 pow_getbitsfrompulses_fx[16];
extern const Word16 DDP_fx[4];
extern const Word16 step_tcq_fx[8][STATES];
extern const Word16 denc_fx[8][STATES]; /* enc trellis */
extern const Word16 ddec_fx[8][STATES]; /* dec trellis */
extern const Word16 step_LSB_fx[STATES_LSB][2];
extern const Word16 denc_LSB_fx[STATES_LSB][2];
extern const Word16 dqnt_LSB_fx[STATES_LSB][4];

extern const Word16 dstep_LSB_fx[4][2];
extern const Word16 ddec_LSB_fx[4][2];

extern const Word16 SmoothingWin_NB875_fx[];
extern const Word16 SmoothingWin_NB2_fx[];

extern const Word16 window_48kHz_fx16[];
extern const Word16 ENR_ATT_fx[5];


extern const Word16 cu15_fx[28][3];
extern const Word16 cu4_fx[6][3];
extern const Word16 ct2_fx[7][14];

extern const unsigned char* const hBitsN[65];

extern const Word16 One_div_fx[];


/*---------------------------------------------------------------------*
 * TABLE ROM, defined in lib_com\rom_com.c
 *---------------------------------------------------------------------*/
extern const Word16 inter4_2tcx2[4][4];
extern const Word16 inter6_2tcx2[6][4];
typedef struct TCX_LTP_FILTER
{
    const Word16 *filt;
    Word16 length;
} TCX_LTP_FILTER;
extern const TCX_LTP_FILTER tcxLtpFilters[12];

extern const Word16 Grid[];
extern const Word16 low_H[64];
extern const Word16 low_H16k[80];
extern const Word16 mid_H[64];
extern const Word16 mid_H16k[80];

extern const Word16 E_ROM_inter4_1[PIT_UP_SAMP * L_INTERPOL1 + 1];
extern const Word16 E_ROM_inter6_1[PIT_UP_SAMP6 * L_INTERPOL1 + 1];
extern Word16 E_ROM_qua_gain5b_const[NB_QUA_GAIN5B * 2];
extern Word16 E_ROM_qua_gain6b_const[NB_QUA_GAIN6B * 2] ;
extern Word16 E_ROM_qua_gain7b_const[NB_QUA_GAIN7B * 2] ;
extern Word16 E_ROM_qua_gain8b_const[NB_QUA_GAIN8B * 2] ;
extern Word16 Es_pred_qua[8];
extern Word16 Es_pred_qua_2[16];


extern const SCALE_TCX_SETUP scaleTcxTable[13];

extern const FrameSizeParams FrameSizeConfig[FRAME_SIZE_NB];
extern const Word16 amrwb_frame_sizes[9];

/*fd_cng_commom.h*/
extern Word16 d_array[18];
extern Word16 m_array[18];
#define msQeqInvAv_thres_EXP      (-4)
extern Word16 msQeqInvAv_thresh[3];
extern Word16 msNoiseSlopeMax[4];
#define msNoiseSlopeMax_EXP        3

extern SCALE_SETUP scaleTable[20];
extern SCALE_SETUP scaleTable_cn_only[18];
extern const Word16 scaleTable_cn_only_amrwbio[3][2];

extern Word16 sidparts_encoder_noise_est[24];


extern const FD_CNG_SETUP FdCngSetup_nb;
extern const FD_CNG_SETUP FdCngSetup_wb1;
extern const FD_CNG_SETUP FdCngSetup_wb2;
extern const FD_CNG_SETUP FdCngSetup_wb3;
extern const FD_CNG_SETUP FdCngSetup_swb1;
extern const FD_CNG_SETUP FdCngSetup_swb2;

extern Word16 maxN_37bits;
extern Word16 maxC_37bits;
extern Word16 stages_37bits;
extern Word16 levels_37bits[6];
extern Word16 bits_37bits[6];
extern Word16 const * const cdk_37bits[];

/* tcx_utils.c */
extern const Word16 gain_corr_fac[]; /*pow(10,2^(-n-2)/28)*/
extern const Word16 gain_corr_inv_fac[];/*pow(10,-2^(-n-2)/28)*/

/* ari_stat.h */
extern const Word8   ari_lookup_s17_LC[4096];
extern const UWord16 ari_pk_s17_LC_ext[64][18];
extern const Word16 Tab_esc_nb[4];

/* ari_hm.c */
extern const Word16 NumRatioBits[2][17];
extern const Word16 Ratios_WB_2[32];
extern const Word16 Ratios_WB_3[32];
extern const Word16 Ratios_WB_4[32];
extern const Word16 Ratios_WB_5[32];
extern const Word16 Ratios_WB_6[32];
extern const Word16 Ratios_WB_7[32];
extern const Word16 Ratios_WB_8[16];
extern const Word16 Ratios_WB_9[16];
extern const Word16 Ratios_WB_10[16];
extern const Word16 Ratios_WB_11[16];
extern const Word16 Ratios_WB_12[16];
extern const Word16 Ratios_WB_13[16];
extern const Word16 Ratios_WB_14[16];
extern const Word16 Ratios_WB_15[16];
extern const Word16 Ratios_WB_16[4];
extern const Word16 Ratios_WB_17[4];
extern const Word16 Ratios_WB_18[4];
extern const Word16 Ratios_NB_2[32];
extern const Word16 Ratios_NB_3[16];
extern const Word16 Ratios_NB_4[16];
extern const Word16 Ratios_NB_5[16];
extern const Word16 Ratios_NB_6[16];
extern const Word16 Ratios_NB_7[16];
extern const Word16 Ratios_NB_8[16];
extern const Word16 Ratios_NB_9[8];
extern const Word16 Ratios_NB_10[8];
extern const Word16 Ratios_NB_11[8];
extern const Word16 Ratios_NB_12[8];
extern const Word16 Ratios_NB_13[4];
extern const Word16 Ratios_NB_14[4];
extern const Word16 Ratios_NB_15[4];
extern const Word16 Ratios_NB_16[4];
extern const Word16 Ratios_NB_17[4];
extern const Word16 Ratios_NB_18[4];
extern const Word16 *const Ratios[2][17];

extern const Word16 qGains[2][1 << kTcxHmNumGainBits];

/*tns_tables.h*/

extern struct TnsParameters const tnsParametersIGF32kHz_LowBR[1];

extern struct TnsParameters const tnsParameters32kHz[2];

extern struct TnsParameters const tnsParameters32kHz_grouped[2];

extern struct TnsParameters const tnsParameters16kHz[1];

extern struct TnsParameters const tnsParameters16kHz_grouped[2];
extern struct TnsParameters const tnsParameters48kHz_grouped[2];

extern const Word16 tnsAcfWindow[TNS_MAX_FILTER_ORDER];

/**********************************************************************/
/*  Definition of the mapping between TNS parameters and a bitstream  */
/**********************************************************************/

extern ParamsBitMap const tnsEnabledSWBTCX20BitMap[1];
extern ParamsBitMap const tnsEnabledSWBTCX10BitMap[1];
extern ParamsBitMap const tnsEnabledWBTCX20BitMap[1];
extern ParamsBitMap const tnsEnabledWBTCX10BitMap[1];

/**********************************************/
/* Helper structures for hufmann table coding */
/**********************************************/

extern Coding codesTnsAllCoeffs[];

extern Coding codesTnsCoeff5[];
extern Coding codesTnsCoeff6[];

extern Coding codesTnsCoeff0WBTCX20[];
extern Coding codesTnsCoeff0WBTCX10[];
extern Coding codesTnsCoeff1WBTCX20[];
extern Coding codesTnsCoeff1WBTCX10[];
extern Coding codesTnsCoeff2WBTCX20[];
extern Coding codesTnsCoeff2WBTCX10[];
extern Coding codesTnsCoeff3WB[];

extern Coding codesTnsCoeff456[];
extern Coding codesTnsCoeff7[];

extern Word16 const nTnsCoeffCodes;

extern Coding * codesTnsCoeffSWBTCX20[];
extern Coding * codesTnsCoeffSWBTCX10[];
extern Coding * codesTnsCoeffWBTCX20[];
extern Coding * codesTnsCoeffWBTCX10[];
extern Word16 const nTnsCoeffTables;

extern Coding codesTnsOrderTCX20[];
extern Coding codesTnsOrderTCX10[];
extern Coding codesTnsOrder[];
extern Word16 const nTnsOrderCodes;


/*******************************/
/* Quantized TNS coefficients. */
/*******************************/

/* 4 bit resolution TNS coefficients. */
extern const Word16 tnsCoeff4[16];

/*bits_alloc.h*/

/*bits_alloc_tables.h*/
/*RATE MODE CONFIGURATION TABLE*/
extern const Word16 RATE_MODE_MAPPING[FRAME_SIZE_NB];


/*ACELP INNOVATIVE CDBK*/
#define ACELP_FIXED_CDK_NB 48
#define ACELP_FIXED_CDK_BITS(n) ACELP_CDK_BITS[n]

/* Number of states for any combination pulses in any combination of vector length */
extern const Word32 pulsestostates[17][9];



/*ACELP NRG ELEMENT*/
extern const Word16 ACELP_NRG_MODE[RATE_MODE_MAX][BANDWIDTH_MODE_MAX][ACELP_MODE_MAX+RF_MODE_MAX];
extern const Word16 ACELP_NRG_BITS[3];

/*ACELP LTP ELEMENT*/
extern const Word16 ACELP_LTP_MODE[RATE_MODE_MAX][BANDWIDTH_MODE_MAX][ACELP_MODE_MAX+RF_MODE_MAX];
extern const Word16 ACELP_LTP_BITS_SFR[8+RF_MODE_MAX][5];

/*ACELP LTF ELEMENT*/
extern const Word16 ACELP_LTF_MODE[RATE_MODE_MAX][BANDWIDTH_MODE_MAX][ACELP_MODE_MAX+RF_MODE_MAX];
extern const Word16 ACELP_LTF_BITS[4];

/*ACELP GAINS ELEMENT*/
extern const Word16 ACELP_GAINS_MODE[RATE_MODE_MAX][BANDWIDTH_MODE_MAX][ACELP_MODE_MAX+RF_MODE_MAX];
extern const Word16 ACELP_GAINS_BITS[11];

/*ACELP BPF ADAPT ELEMENT*/
extern const Word16 ACELP_BPF_MODE[RATE_MODE_MAX][BANDWIDTH_MODE_MAX][ACELP_MODE_MAX+RF_MODE_MAX];
extern const Word16 ACELP_BPF_BITS[4];

extern const Word8 gTcxLpcLsf[3][2];
extern const Word16 *lsf_means[2];
typedef Word16 lsp_unw_triplet[3];
extern const lsp_unw_triplet p16_gamma0_92to1[16];
extern const lsp_unw_triplet p16_gamma0_94to1[16];

extern const Word16 *const lsf_codebook[2][2][TCXLPC_NUMSTAGES];
extern const Word16 lsf_numbits[TCXLPC_NUMSTAGES];
extern const Word16 lsf_dims[TCXLPC_NUMSTAGES];
extern const Word16 lsf_offs[TCXLPC_NUMSTAGES];
extern const Word16 lsf_q_diff_cb_8b_rf[];
extern const Word16 lsf_cdk_nb_gc_stg1[];
extern const Word16 lsf_cdk_nb_gc_stg2[];
extern const Word16 lsf_cdk_nb_gc_stg3[];
extern const Word16 lsf_ind_cdk_nb_gc_stg4[];
extern const Word16 lsf_cdk_nb_vc_stg1[];
extern const Word16 lsf_cdk_nb_vc_stg2[];
extern const Word16 lsf_cdk_nb_vc_stg3[];
extern const Word16 lsf_ind_cdk_nb_vc_stg4[];
extern const Word16 lsf_cdk_wb_gc_stg1[];
extern const Word16 lsf_cdk_wb_gc_stg2[];
extern const Word16 lsf_cdk_wb_gc_stg3[];
extern const Word16 lsf_ind_cdk_wb_gc_stg4[];
extern const Word16 lsf_cdk_wb_vc_stg1[];
extern const Word16 lsf_cdk_wb_vc_stg2[];
extern const Word16 lsf_cdk_wb_vc_stg3[];
extern const Word16 lsf_ind_cdk_wb_vc_stg4[];

extern const Word16 *const lsf_ind_codebook[2][2][TCXLPC_IND_NUMSTAGES];
extern const Word16 lsf_ind_numbits[TCXLPC_IND_NUMSTAGES];
extern const Word16 lsf_ind_dims[TCXLPC_IND_NUMSTAGES];
extern const Word16 lsf_ind_offs[TCXLPC_IND_NUMSTAGES];
extern const Word16 min_distance_thr[2][2];

extern const Word16 mean_isf_wb [16];

extern const Word16 means_nb_31bits_ma_lsf[16];
extern const Word16 means_wb_31bits_ma_lsf[16];

extern const Word16 maxC_wbhb_31bits_ma_lsf;
extern const Word16 stages_wbhb_31bits_ma_lsf;
extern const Word16 levels_wbhb_31bits_ma_lsf[5];
extern const Word16 bits_wbhb_31bits_ma_lsf[5];

extern const Word16 lsf_fit_model[5][16];


extern const Word16 cdk_wbhb_31bits_ma_lsf_10_p12[1536];

extern const Word16 cdk_wbhb_31bits_ma_lsf_20_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_30_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_40_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_50_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_11_p12[1536];

extern const Word16 cdk_wbhb_31bits_ma_lsf_21_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_31_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_41_p12[768];

extern const Word16 cdk_wbhb_31bits_ma_lsf_51_p12[768];

extern const Word16 * const cdk_wbhb_31bits_ma_lsf0[];
extern const Word16 * const cdk_wbhb_31bits_ma_lsf1[];


extern const Word16 CLDFB80_10[100];
extern const Word16 CLDFB80_16[160];
extern const Word16 CLDFB80_20[200];
extern const Word16 CLDFB80_32[320];
extern const Word16 CLDFB80_40[400];
extern const Word16 CLDFB80_60[600];

extern const Word16 rRotVectr_10[];
extern const Word16 iRotVectr_10[];
extern const Word16 rRotVectr_16[];
extern const Word16 iRotVectr_16[];
extern const Word16 rRotVectr_20[];
extern const Word16 iRotVectr_20[];
extern const Word16 rRotVectr_32[];
extern const Word16 iRotVectr_32[];
extern const Word16 rRotVectr_40[];
extern const Word16 iRotVectr_40[];
extern const Word16 rRotVectr_60[];
extern const Word16 iRotVectr_60[];

extern const Word16 cldfb_anaScale[];
extern const Word16 cldfb_synScale[];
extern const Word16 cldfb_synGain[];
extern const Word16 *cldfb_protoFilter_2_5ms[];
extern const Word16 cldfb_scale_2_5ms[7];


extern const Word16 lag_window_8k[2][16];
extern const Word16 lag_window_12k8[NUM_LAGW_STRENGTHS][2][16];
extern const Word16 lag_window_16k[NUM_LAGW_STRENGTHS][2][16];
extern const Word16 lag_window_25k6[NUM_LAGW_STRENGTHS][2][16];
extern const Word16 lag_window_32k[NUM_LAGW_STRENGTHS][2][16];
extern const Word16 lag_window_48k[2][16];

/**********************************************************************/ /**
igf settings structure for each bitrate mode
**************************************************************************/
typedef struct igf_mode_type
{
    Word32     sampleRate;
    Word16     frameLength;
    Word32     bitRate;
    Word16     igfMinFq;
    Word16     transFac;
    Word16     maxHopsize;
} IGF_MODE,*H_IGF_MODE;



extern IGF_MODE igfMode[17];
extern const Word16 swb_offset_LB_new[17][IGF_MAX_SFB];
extern const Word16 igf_whitening_TH[17][2][IGF_MAX_TILES];

extern const Word16 cf_off_se01_tab[10];

extern const Word16 cf_off_se10_tab;

extern const Word16 cf_off_se02_tab[10][IGF_CTX_COUNT];

extern const Word16 cf_off_se11_tab[IGF_CTX_COUNT][IGF_CTX_COUNT];
extern const Word16 cf_se00_tab[IGF_SYMBOLS_IN_TABLE + 1];

extern const Word16 cf_se01_tab[10][IGF_SYMBOLS_IN_TABLE + 1];
extern const Word16 cf_se02_tab[10][IGF_CTX_COUNT][IGF_SYMBOLS_IN_TABLE + 1];

extern const Word16 cf_se10_tab[IGF_SYMBOLS_IN_TABLE + 1];
extern const Word16 cf_se11_tab[IGF_CTX_COUNT][IGF_CTX_COUNT][IGF_SYMBOLS_IN_TABLE + 1];

extern const Word32 bwMode2fs[4];

extern const Word16 kLog2TableFrac_x[256];
extern const Word16 kExp2TableFrac_x[256];
extern Word16 sqrt_table_pitch_search[256+1];

extern const Word16 grid50_fx[(GRID50_POINTS-1)/2 - 1];
extern const Word16 grid40_fx[(GRID40_POINTS-1)/2 - 1];
/******************** moved over from table_decl.h ************************/

/* For pitch predictor */
#define INTERP_EXP 0
extern const Word16 pitch_inter4_1[UP_SAMP * L_INTERPOL1 + 1]; /*1Q14*/
extern const Word16 pitch_inter4_2[PIT_FIR_SIZE2];             /*1Q14*/
extern const Word16 pitch_inter6_2[PIT_FIR_SIZE6_2];           /*1Q14*/

/* For bass postfilter */
extern const Word16 filt_lp[1 + L_FILT];
extern const Word16 filt_lp_16kHz[1+L_FILT16k];

extern const Word16 L_frame_inv[8];

extern const Word16 TecLowBandTable[];
extern const Word16 TecSC_Fx[];

extern const Word16 uniform_model_fx[];

/* fft.c */
extern const Word16 RotVector_24[2*(24-3)];
extern const Word16 RotVector_32[2*20];
extern const Word16 RotVector_192[2*(192-16)];
extern const Word16 RotVector_256[2*(256-32)];
extern const Word16 RotVector_320[2*(320-20)];
extern const Word16 RotVector_400[2*(400-20)];
extern const Word16 RotVector_480[2*(480-30)];
extern const Word16 RotVector_600[2*(600-30)];


extern Word16 tab_ari_qnew[4][4];


extern  const Word16 num_nelp_lp_fx[NELP_LP_ORDER+1];
extern  const Word16 den_nelp_lp_fx[NELP_LP_ORDER+1];



#endif /*ROM_COM_H */

