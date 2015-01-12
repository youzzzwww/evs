/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches */
#include "cnst_fx.h"       /* Common constants */
#include "prot_fx.h"       /* Function prototypes */
#include "stl.h"

/*======================================================================*/
/* FUNCTION      :  decod_nelp_fx()                                      */
/*-----------------------------------------------------------------------*/
/* PURPOSE       :  Decode unvoiced NELP                                 */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS  :                                             */
/*    _ (Struct)   st_fx        : decoder static memory                  */
/*    _ (Word16)   coder_type   : coding type                            */
/*    _ (Word16[]) tmp_noise_fx : long term temporary noise energy       */
/*    _ (Word16[]) pitch_buf_fx : floating pitch values for each
                                  subframe(Q6)                           */
/*    _ (Word16[]) exc_fx       : adapt. excitation exc (Q_exc)             */
/*    _ (Word16[]) exc2_fx      : adapt. excitation/total exc (Q_exc)       */
/*    _ (Word16)   bfi          : frame error rate                       */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/*    _ (Word16[]) exc_fx       : adapt. excitation exc (Q_exc)          */
/*    _ (Word16[]) exc2_fx      : adapt. excitation/total exc (Q_exc)     */
/*-----------------------------------------------------------------------*/

/*    _ (Word16[]) tmp_noise_fx     : long term temporary noise energy
                                      (Q0)                               */
/*    _ (Word16[]) pitch_buf_fx     : floating pitch values for each
                                      subframe (Q6)		                 */
/*    _ (Word16[]) st_fx->dispMem   : Noise enhancer - phase dispersion
                                      algorithm memory (Q14)		     */
/*    _ (Word16)   st_fx->tilt_code : tilt of code (Q15)				 */
/*    _ (Word16)   st_fx->prev_gain_pit_dec 							 */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/*     _ None                                                            */
/*=======================================================================*/

void decod_nelp_fx(
    Decoder_State_fx *st_fx,
    const Word16 coder_type,
    Word16 *tmp_noise_fx,
    Word16 *pitch_buf_fx,
    Word16 *exc_fx,
    Word16 *exc2_fx,
    Word16 *voice_factors,
    Word16 *bwe_exc_fx,
    Word16 *Q_exc, Word16 bfi
    , Word16 *gain_buf
)
{
    Word16 i;
    Word16 exc_nelp_fx[L_FRAME];

    *tmp_noise_fx = 0;
    move16();

    nelp_decoder_fx(st_fx, exc_nelp_fx, exc_fx, Q_exc, bfi, coder_type
                    , gain_buf

                   );

    Copy(exc_nelp_fx, exc_fx, L_FRAME);
    Copy(exc_nelp_fx, exc2_fx, L_FRAME);

    st_fx->tilt_code_fx = 0;
    move16();/* purely unvoiced */       /* Q15 */
    set16_fx(st_fx->tilt_code_dec_fx, 0, NB_SUBFR16k);

    st_fx->dm_fx.prev_state = 0;
    move16();/* Q0 */
    st_fx->prev_gain_pit_dec_fx = 0;
    move16();                 /* Q14 */
    st_fx->dm_fx.prev_gain_pit[0]= st_fx->prev_gain_pit_dec_fx;
    move16(); /* Q16 */

    FOR(i=1; i<5; i++)
    {
        st_fx->dm_fx.prev_gain_pit[i] = st_fx->dm_fx.prev_gain_pit[i-1];
        move16();    /* Q14 */
    }

    set16_fx(pitch_buf_fx, L_SUBFR_Q6, NB_SUBFR);  /* L_SUBFR = 64, Q6 */
    set16_fx(voice_factors, 0, NB_SUBFR16k);
    interp_code_5over2_fx( exc2_fx, bwe_exc_fx, L_FRAME );
    return;
}
