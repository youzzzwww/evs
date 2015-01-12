/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches */
#include "prot_fx.h"       /* Function prototypes */
#include "stl.h"

/*===================================================================*/
/* FUNCTION      : void decod_ppp_fx ()								 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :	decode highly voiced frames	 using PPP			 */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*  _ const Word16 Aq_fx[] - Q12 12k8 Lp coefficient				 */
/*  _ Word16 bfi_fx - Q0  bad frame indicator				         */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ Decoder_State_fx *st_fx:										 */
/*                _ lastLgainD_fx - Q11                              */
/*                _ lastHgainD_fx - Q11                              */
/*                _ lasterbD_fx   - Q13                              */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ Decoder_State_fx *st_fx:										 */
/*				  _	lsp_old_fx	-               Q15					 */
/*				  _	st_fx->dtfs_dec_xxxx							 */
/*				  _	a nd b in   st_fx->dtfs_dec_Q					 */
/*				  	rest all in                 Q0					 */
/*                _  gainp_ppp                  Q14                  */
/*                _ FadeScale_fx  -             Q15                  */
/*                _ tilt_code_fx  -             Q15                  */
/*				  _ prev_tilt_code_dec_fx -     Q15		     		 */
/*                _ prev_gain_pit_dec_fx  -     Q14                  */
/*                _ dm_fx.prev_state -          Q0                   */
/*                _ dm_fx.prev_gain_code -      Q16                  */
/*                _ .dm_fx.prev_gain_pit  -     Q14                  */
/*                _ prev_gain_pit_dec -			Q14                  */
/* 	 _ Word16 *pitch_buf_fx	- Q6 fixed pitch values for each subframe */
/*	 _ Word16 *exc_fx       - Q_exc current non-enhanced excitation	 */
/* 	 _ Word16 *exc2_fx		- Q_exc current enhanced excitation 	 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*					 _ None											 */
/*-------------------------------------------------------------------*/
/* CALLED FROM : RX													 */
/*===================================================================*/

void decod_ppp_fx(
    Decoder_State_fx *st_fx,					/* i/o: state structure */
    const Word16 Aq_fx[],						/* i  : 12k8 Lp coefficient */
    Word16 *pitch_buf_fx,				/* i/o: fixed pitch values for each subframe */
    Word16 *exc_fx,						/* i/o: current non-enhanced excitation */
    Word16 *exc2_fx,					/* i/o: current enhanced excitation */
    Word16 bfi							/* i  : bad frame indicator */
    , Word16 *gain_buf
    ,Word16  *voice_factors,               /* o  : voicing factors */
    Word16  *bwe_exc_fx                      /* o  : excitation for SWB TBE */
)
{
    Word16 k;

    Word16 LPC_de_curr_fx[M+1], p_Aq_curr_fx[M], p_Aq_old_fx[M+1];
    Word16 excQ_ppp_fx[L_FRAME], pitch_fx[NB_SUBFR], LPC_de_old_fx[M+1];

    /* call voiced decoder at this point */
    FOR( k=0; k<M; k++)
    {
        p_Aq_curr_fx[k] = Aq_fx[k+(3*(M+1))+1];
        move16();/*Q12 */
    }


    deemph_lpc_fx(p_Aq_curr_fx, p_Aq_old_fx, LPC_de_curr_fx, LPC_de_old_fx ,0 );/*LPC in Q12 */

    ppp_voiced_decoder_fx(st_fx, excQ_ppp_fx, LPC_de_curr_fx, exc_fx, pitch_fx, bfi);

    st_fx->tilt_code_fx = st_fx->tilt_code_dec_fx[3];
    move16();

    Copy(excQ_ppp_fx, exc_fx, L_FRAME);
    Copy(exc_fx, exc2_fx, L_FRAME);

    st_fx->dm_fx.prev_state = 2;
    move16();

    IF ( bfi == 0 )
    {
        st_fx->dm_fx.prev_gain_pit[0] = st_fx->prev_gain_pit_dec_fx;
        move16();/*Q14 */
    }
    ELSE
    {
        st_fx->prev_gain_pit_dec_fx = st_fx->gainp_ppp_fx;
        move16();/*Q14 */
        st_fx->dm_fx.prev_gain_pit[0] = st_fx->prev_gain_pit_dec_fx;
        move16();/*Q14 */
    }

    FOR(k=3; k<7; k++)
    {
        st_fx->dm_fx.prev_gain_pit[k-2] = st_fx->dm_fx.prev_gain_pit[k-3];
        move16();/*Q14 */
    }

    Copy(pitch_fx, pitch_buf_fx, NB_SUBFR);/*Q6 */

    interp_code_5over2_fx( exc2_fx, bwe_exc_fx, L_FRAME );
    set16_fx( voice_factors, 0, NB_SUBFR16k );



    set16_fx(gain_buf,0,NB_SUBFR16k);
    return;
}
