/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * decod_unvoiced()
 *
 * Decode unvoiced (UC) frames
 *-------------------------------------------------------------------*/

void decod_unvoiced_fx(
    Decoder_State_fx *st_fx,              /* 					i/o: decoder static memory                  */
    const Word16 *Aq_fx,                  /* 	Q12				i  : LP filter coefficient                  */
    const Word16 coder_type_fx,         /* 	Q0				i  : coding type                            */
    Word16 *tmp_noise_fx,          /* 	Q0				o  : long term temporary noise energy       */
    Word16 *pitch_buf_fx,          /* 	Q6				o  : floating pitch values for each subframe*/
    Word16 *voice_factors_fx,      /* 	Q15				o  : voicing factors                        */
    Word16 *exc_fx,                /* 	Q_X			o  : adapt. excitation exc                  */
    Word16 *exc2_fx,               /* 	Q_X				o  : adapt. excitation/total exc            */
    Word16 *bwe_exc_fx             /* 	Q_X				i/o: excitation for SWB TBE					*/
    ,Word16 *gain_buf
)
{
    Word16 gain_pit_fx = 0;          /* Quantized pitch gain                 */
    Word32 gain_code_fx;             /* Quantized algebraic codeebook gain   */
    Word16 gain_inov_fx;             /* inovation gain                       */
    Word32 norm_gain_code_fx;        /* normalized algebraic codeebook gain  */
    Word16 voice_fac_fx;             /* Voicing factor                       */
    Word16 code_fx[L_SUBFR];         /* algebraic codevector                 */
    Word16 i_subfr_fx;
    const Word16 *p_Aq_fx;
    Word16 *pt_pitch_fx;

    test();
    IF ( sub(st_fx->last_ppp_mode_dec_fx,1) == 0 || sub(st_fx->last_nelp_mode_dec_fx,1) == 0 )
    {
        /* SC_VBR - reset the decoder, to avoid memory not updated issue for this unrealistic case */
        CNG_reset_dec_fx( st_fx, pitch_buf_fx, voice_factors_fx );
    }

    p_Aq_fx = Aq_fx;
    move16();   /*Q12*/               /* pointer to interpolated LPC parameters */
    pt_pitch_fx = pitch_buf_fx;
    move16();               /* pointer to the pitch buffer  */

    FOR( i_subfr_fx=0; i_subfr_fx<L_FRAME; i_subfr_fx+=L_SUBFR )
    {
        /*----------------------------------------------------------------*
         * Unvoiced subframe processing
         *----------------------------------------------------------------*/

        gaus_dec_fx( st_fx, st_fx->core_brate_fx, i_subfr_fx, code_fx, &norm_gain_code_fx,
                     &st_fx->lp_gainp_fx, &st_fx->lp_gainc_fx, &gain_inov_fx, &st_fx->tilt_code_fx,
                     &voice_fac_fx, &gain_pit_fx, pt_pitch_fx, exc_fx, &gain_code_fx, exc2_fx, bwe_exc_fx,&(st_fx->Q_exc),st_fx->Q_subfr );

        *tmp_noise_fx = extract_h(norm_gain_code_fx);		/*Q16*/

        /*----------------------------------------------------------------*
         * Excitation enhancements (update of total excitation signal)
         *----------------------------------------------------------------*/

        enhancer_fx( st_fx->core_brate_fx, 0, coder_type_fx, i_subfr_fx, L_FRAME, voice_fac_fx, st_fx->stab_fac_fx,
                     norm_gain_code_fx, gain_inov_fx, &st_fx->gc_threshold_fx, code_fx, exc2_fx, gain_pit_fx, &(st_fx->dm_fx), st_fx->Q_exc );

        voice_factors_fx[i_subfr_fx/L_SUBFR] =  0;
        move16();

        interp_code_5over2_fx( &exc_fx[i_subfr_fx], &bwe_exc_fx[i_subfr_fx * HIBND_ACB_L_FAC], L_SUBFR );

        p_Aq_fx += (M+1);
        move16();
        pt_pitch_fx++;
        st_fx->tilt_code_dec_fx[i_subfr_fx/L_SUBFR] = st_fx->tilt_code_fx;
        move16();
    }

    /* SC-VBR */
    st_fx->prev_gain_pit_dec_fx = gain_pit_fx;
    move16();

    set16_fx( gain_buf, 0, NB_SUBFR );
    set32_fx( st_fx->gain_code_fx, 0L, NB_SUBFR16k );

    return;
}
