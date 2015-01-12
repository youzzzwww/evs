/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#ifndef ROM_DEC_FX_H
#define ROM_DEC_FX_H

#include <stdio.h>
#include "options.h"     /* Compilation switches                   */
#include "stat_enc_fx.h"   /* Encoder static structure               */
#include "stat_dec_fx.h"   /* Decoder static structure               */

/*----------------------------------------------------------------------------------*
 * General tables
 *----------------------------------------------------------------------------------*/
extern const Word16 gw_fx[Lgw_max];
extern const Word16 gw_len_inv_fx[Lgw_max-1];
extern const Word16 gwlpr_fx[Lgw_max];

extern const Word16 hestable_fx[15];

extern const Word16 h_high_fx[5];
extern const Word16 inv_sqi[15];
extern const Word16 sqi[15];


/*------------------------------------------------------------------------------*
 * AVQ - RE8 tables
 *------------------------------------------------------------------------------*/

extern const Word16 mult_avq_tab_fx[];
extern const Word16 shift_avq_tab_fx[];

/*----------------------------------------------------------------------------------*
 * HR SWB BWE parameters
 *----------------------------------------------------------------------------------*/

extern const Word16 swb_hr_inv_frm_len[4]; /* in Q19 */
extern const Word16 inv_tbl_2n_minus1[];

/*---------------------------------------------------------------------*
 * TABLE ROM, defined in lib_dec_fx\rom_dec_fx.c
 *---------------------------------------------------------------------*/

extern const Word16 hntable_fx[55];
extern const Word16 hetable_fx[57];

extern const Word16 H_low[5];

extern const Word16 lsf_tab_fx[LPC_SHB_ORDER];

/*---------------------------------------------------------------------*
 * NB post-filter tables
 *---------------------------------------------------------------------*/

extern const Word16 Tab_hup_s[];
extern const Word16 Tab_hup_l[];


/*-----------------------------
 * FEC_HQ_phase_ecu
 *------------------------------*/
extern const Word16 FFT_W256[];
extern const Word16 w_hamm_sana48k_2_fx[];
extern const Word16 w_hamm48k_2_fx[];
extern const Word16 w_hamm32k_2_fx[];
extern const Word16 w_hamm16k_2_fx[];
extern const Word16 w_hamm8k_2_fx[];
extern const Word16 POW_ATT_TABLE0[];
extern const Word16 POW_ATT_TABLE1[];
extern const Word16 GR_POW_HEADROOM[];


/*Table 256 / (L_frame*2) , needed in sig_classifier.c and er_dec_acelp.c*/
extern const Word16 T_256DIV_L_Frame[];
extern const Word16 T_DIV_L_Frame[];        /*0Q15 * 2^-7 */

/* er_dec_tcx.c */
extern const Word16 h_high3_32[L_FIR_FER2];
extern const Word16 h_high3_25_6[L_FIR_FER2];
extern const Word16 h_high3_16[L_FIR_FER2];

extern const Word16 pow2tab[15];

/* CLDFB BPF */
extern const Word16 bpf_weights_16_Fx[16];

#endif /* ROM_DEC_FX.H */
