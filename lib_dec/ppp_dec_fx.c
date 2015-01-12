/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"


/*===================================================================*/
/* FUNCTION      : void ppp_quarter_decoder_fx ()					 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :	   			                                     */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*  _ Word16 bfi_fx - Q0  bad frame indicator				         */
/*  _ const Word16 *curr_lpc_fx - Q12		current frame LPC        */
/*  _ Word16 *exc_fx		- Q0		previous frame excitation    */
/*  _ Word16 prevCW_lag_fx  - Q0        Previous lag				 */
/*   _ (struct DTFS_fx) PREV_CW_D_FX :  prototype in polar domain	 */
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ Decoder_State_fx *st_fx:										 */
/*		_ Word16 *pitch	- Q6 floating pitch values for each subframe */
/*      _ Word16 *out_fx - Q0				     residual signal	 */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ Decoder_State_fx *st_fx:										 */
/*				  _	lsp_old_fx	-               Q15					 */
/*				  _	st_fx->dtfs_dec_xxxx							 */
/*                _  gainp_ppp                  Q14                  */
/*                _ lastLgainD_fx - Q11                              */
/*                _ lastHgainD_fx - Q11                              */
/*                _ lasterbD_fx   - Q13                              */
/*   _ (struct DTFS_fx) CURRCW_Q_DTFS_FX :  prototype in polar domain*/
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/* 	 _ Word16 *pitch_buf_fx	- Q6 fixed pitch values for each subframe */
/*	 _ Word16 *exc_fx       - Q0 previous frame excitation			 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*					 _ None											 */
/*-------------------------------------------------------------------*/
/* CALLED FROM : RX													 */
/*===================================================================*/

void ppp_quarter_decoder_fx(
    DTFS_STRUCTURE_FX *CURRCW_Q_DTFS_FX,    /* i/o: Current CW DTFS											  */
    Word16   prevCW_lag_fx,        /* i  : Previous lag											  */
    Word16 *lastLgainD_fx,			/* i/o: Last gain lowband Q11									  */
    Word16 *lastHgainD_fx,			/* i/o: Last gain highwband Q11									  */
    Word16 *lasterbD_fx,			/* i/o: Last ERB vector Q13										  */
    Word16 bfi,					/* i  : FER flag												  */
    Word16 *S_fx,                  /* i  : sine table, Q15                                           */
    Word16 *C_fx,                  /* i  : cosine table, Q15                                         */
    DTFS_STRUCTURE_FX PREV_CW_D_FX,         /* i  : Previous DTFS											  */
    Decoder_State_fx *st_fx
)
{
    DTFS_STRUCTURE_FX *PREVDTFS_FX = DTFS_new_fx();
    Word16 AMP_IDX_fx[2];
    Word16 temp_pl_fx =  prevCW_lag_fx, temp_l_fx = CURRCW_Q_DTFS_FX->lag_fx;
    Word16 temp_fx;
    Word16 l_fx = CURRCW_Q_DTFS_FX->lag_fx;
    Word16 POWER_IDX_fx;
    Word16 Erot_fx = 0;
    Word16 num_erb_fx = 24;
    Word32 temp32d_fx,temp32n_fx;
    Word32 L_tmp, L_tmp1;
    Word16 tmp, exp;


    IF ( sub(CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx,4000 ) == 0)
    {
        num_erb_fx = 22;
        move16();
    }
    ELSE IF ( sub(CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx,6400) == 0 )
    {
        num_erb_fx = 24;
        move16();
    }

    DTFS_copy_fx(PREVDTFS_FX, PREV_CW_D_FX);
    IF (bfi == 0)
    {
        POWER_IDX_fx =(Word16) get_next_indice_fx( st_fx, 6);
        move16();
        AMP_IDX_fx[0] =(Word16) get_next_indice_fx( st_fx, 6);
        move16();
        AMP_IDX_fx[1] =(Word16) get_next_indice_fx( st_fx, 6);
        move16();

        /* Amplitude Dequantization */
        /*This normalization and de-normalization is done to avoid division by 12800. And this logic is used only in
        dequant_cw. So upper cut-off frequencies need to be multiplied by a factor2.56.
        This logic of normalisation is not employed in adjustlag, hence denormalisation is necessury.*/
        /*As the upper cut of freqencies are normalized to 12800, we have to multiply upper cut off freq by
        	2.56(1/12800 in Q15) */
        temp32n_fx = L_mult(CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx,10486);/* Q0+Q27 = Q28 */
        CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx = (Word16)L_shr(temp32n_fx,13);/*Q15 */
        temp32n_fx = L_mult(CURRCW_Q_DTFS_FX->upper_cut_off_freq_of_interest_fx,10486);/* Q0+Q27 = Q28 */
        CURRCW_Q_DTFS_FX->upper_cut_off_freq_of_interest_fx = (Word16)L_shr(temp32n_fx,13);/*Q15 */

        DTFS_dequant_cw_fx(prevCW_lag_fx,POWER_IDX_fx,AMP_IDX_fx,lastLgainD_fx,lastHgainD_fx,lasterbD_fx,CURRCW_Q_DTFS_FX,num_erb_fx);
        /*De-normalize cut off frequencies */

        temp32n_fx = L_shl((Word32)CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx,13);/*Q28 */
        CURRCW_Q_DTFS_FX->upper_cut_off_freq_fx = (Word16)find_remd(temp32n_fx, 20971,&temp32d_fx);
        temp32n_fx = L_shl((Word32)CURRCW_Q_DTFS_FX->upper_cut_off_freq_of_interest_fx,13);/*Q28 */
        CURRCW_Q_DTFS_FX->upper_cut_off_freq_of_interest_fx = (Word16)find_remd(temp32n_fx, 20971,&temp32d_fx);
    }

    /* Copying phase spectrum over */
    DTFS_adjustLag_fx(PREVDTFS_FX,l_fx);

    temp_fx = sub(L_FRAME,temp_l_fx); /*Q0 */

    exp = norm_s(temp_pl_fx);
    tmp = div_s(shl(1,sub(14,exp)),temp_pl_fx); /*Q(29-exp) */
    L_tmp = L_mult(temp_fx,tmp); /*Q(31-exp); +1 due to /2 */
    L_tmp = L_shl(L_tmp,sub(exp,15)); /*Q16 */

    exp = norm_s(temp_l_fx);
    tmp = div_s(shl(1,sub(14,exp)),temp_l_fx); /*Q(29-exp) */
    L_tmp1 = L_mult(temp_fx,tmp); /*Q(31-exp); +1 due to /2 */
    L_tmp1 = L_shl(L_tmp1,sub(exp,15)); /*Q16 */

    L_tmp = L_add(L_tmp,L_tmp1); /*Q16 */

    tmp = lshr(extract_l(L_tmp),1); /*Q15 */
    L_tmp = L_mult(temp_l_fx,tmp); /*Q16 */
    temp_fx = rint_new_fx(L_tmp);
    Erot_fx = sub(temp_l_fx,temp_fx); /*Q0 */

    Q2phaseShift_fx(PREVDTFS_FX,shl(Erot_fx,2),CURRCW_Q_DTFS_FX->lag_fx,S_fx,C_fx);
    IF ( sub(bfi,1) == 0 )
    {
        DTFS_car2pol_fx(CURRCW_Q_DTFS_FX);
    }
    /*Phase copying is done through copy_phase instead of car2pol and pol2car */
    copy_phase_fx(PREVDTFS_FX,*CURRCW_Q_DTFS_FX,CURRCW_Q_DTFS_FX);

    {
        temp_fx = (Word16) get_next_indice_fx( st_fx, 3 );

        temp_fx = sub(temp_fx,3);
        temp_fx = shl(temp_fx,2);/*Q2 */
        Q2phaseShift_fx(CURRCW_Q_DTFS_FX,temp_fx,CURRCW_Q_DTFS_FX->lag_fx,S_fx,C_fx);
    }

    free(PREVDTFS_FX);

    return;
}

