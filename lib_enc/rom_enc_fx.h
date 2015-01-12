/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef ROM_ENC_FX_H
#define ROM_ENC_FX_H

#include <stdio.h>
#include "options.h"
#include "cnst_fx.h"
#include "stat_enc_fx.h"


/*----------------------------------------------------------------------------------*
 * General tables
 *----------------------------------------------------------------------------------*/

extern const Word16 sqrt_han_window[];          /* LP analysis: half of the square root hanning window */
extern const Word16 crit_bands[];               /* LP analysis: critical bands */

extern const Word16 hangover_hd_tbl_fx[3];
extern const Word16 hangover_sf_tbl_fx[6];

extern const Word16 H_fir[];
extern const Word16 len_12k8_fx[];
extern const Word16 len1_12k8_fx[];
extern const Word16 sublen_12k8_fx[];
extern const Word16 sublen1_12k8_fx[];
extern const Word16 pit_max_12k8_fx[];
extern const Word16 sec_length_12k8_fx[];
extern const Word16 sec_length1_12k8_fx[];
extern const Word16 w_fx[HANG_LEN][HANG_LEN];
extern const Word16 SF_mult_fx[N_FEATURES];
extern const Word16 SF_8k_mult_fx[N_FEATURES];
extern const Word32 SF_add_fx[N_FEATURES];
extern const Word32 SF_8k_add_fx[N_FEATURES];

extern const Word32 lvm_noise_fx[];
extern const Word32 invV_noise_fx[N_MIXTURES*N_FEATURES*N_FEATURES];
extern const Word16 m_noise_fx[N_MIXTURES*N_FEATURES];
extern const Word32 lvm_music_fx[N_MIXTURES];
extern const Word32 invV_music_fx[N_MIXTURES*N_FEATURES*N_FEATURES];
extern const Word16 m_music_fx[N_MIXTURES*N_FEATURES];
extern const Word32 invV_speech_fx[N_MIXTURES*N_FEATURES*N_FEATURES];
extern const Word32 lvm_speech_fx[N_MIXTURES];
extern const Word16 m_speech_fx[N_MIXTURES*N_FEATURES];
extern const Word16 inv_delta_tab[7];

extern const Word16 lsf_unified_fit_model_nb[4][16];
extern const Word16 lsf_unified_fit_model_wb[4][16];
extern const Word16 lsf_unified_fit_model_wbhb[4][16];

extern const Word32 Freq_Weight_Com_fx[160];
extern const Word32 Freq_Weight_UV_fx[160]; /*Q31*/
extern const Word16 hann_window_320_fx[];
extern const Word16 W_HIST_FX[];
extern const Word16 W_HIST_S_FX[];


/*----------------------------------------------------------------------------------*
 * Huffman coding
 *----------------------------------------------------------------------------------*/

extern const Word16 huffsizn_e_fx[32];
extern const Word16 huffsizn_n_fx[32];

extern const Word16 huffnorm_e_fx[32];
extern const Word16 huffnorm_n_fx[32];
extern const Word16 hessize_fx[8];
extern const Word16 hescode_fx[8];

extern const Word16 resize_huffnorm_fx[32];
extern const Word16 huffnorm_fx[32];
extern const Word16 pgain_huffnorm_fx[32];

extern const Word16 Weight[86];
extern const Word16 kLowPeriodicityThr[2];  /* ari_hm_enc.c */

extern UWord8 E_ROM_tipos[];    /* ACELP indexing */

extern const Word16 lsf_numlevels[TCXLPC_NUMSTAGES];
extern const Word16 lsf_ind_numlevels[TCXLPC_IND_NUMSTAGES];

/*----------------------------------------------------------------------------------*
 * Starting line for the noise measurement in TCX.
 *----------------------------------------------------------------------------------*/
extern const Word16 startLineWB[11];
extern const Word16 startLineSWB[9];

/*----------------------------------------------------------------------------------*
* CLDFB-VAD
*----------------------------------------------------------------------------------*/
extern const Word16 BAND_NUM_TAB[5];
extern const Word16 BAND_SCALE_AJ[5];
extern const Word32 LS_MIN_SILENCE_SNR[4] ;
extern const Word32 LT_MIN_SILENCE_SNR[4] ;
extern const Word16 Nregion_index_NB[9];
extern const Word16 Nregion_index_WB[13];
extern const Word16 Nregion_index_SWB[16] ;
extern const Word16 Nregion_index_FB[16] ;
extern const Word16 Nregion_preoff[12];
extern const Word16 SNR_SUB_BAND_NUM[4];
extern const Word16 BAND_MUL[4] ;
extern const Word16 *REGION_INDEX[4] ;
extern const Word32 MAX_LF_SNR_TAB[5];
extern const Word16 band_num[4];
extern const Word32 COMVAD_INIT_SNR_DELTA[5];
extern const Word16 i_t_1[10];
extern const Word16 i_t_2[23];
extern const complex_16 wnk_table_16[16];
extern const complex_16 M_in_fix16[16];
extern const Word16 M_Wr_fix16[16];
extern const Word16 M_Wi_fix16[16];

/* bw_detect_fx.c */
extern const Word16 bwd_start_bin_fx[4];
extern const Word16 bwd_end_bin_fx[4];

#endif /* ROM_ENC_FX_H */

