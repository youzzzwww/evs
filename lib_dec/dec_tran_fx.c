/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*======================================================================*/
/* FUNCTION : decod_tran_fx()										    */
/*----------------------------------------------------------------------*/
/* PURPOSE : Decode transition (TC) frames                      		*/
/*																		*/
/*----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS :												*/
/* _ (Struct)	st_fx			: decoder static memory					*/
/* _ (Word16) L_frame_fx		: length of the frame		Q0			*/
/* _ (Word16[]) Aq_fx			: LP filter coefficient		Q12			*/
/* _ (Word16) coder_type_fx		: coding type				Q12			*/
/* _ (Word16) Es_pred_fx		: predicted scaled innov. energy Q8		*/
/* _ (Word16[]) pitch_buf_fx	: floating pitch values for each subframe Q6*/
/* _ (Word16[])	voice_factors_fx: frame error rate				Q15		*/
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q_exc)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q_exc)    */
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/

void decod_tran_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder static memory                  */
    const Word16 L_frame_fx,             /* i  : length of the frame                    */
    const Word16 tc_subfr_fx,            /* i  : TC subframe index                      */
    const Word16 *Aq_fx,                 /* i  : LP filter coefficient                  */
    const Word16 coder_type_fx,          /* i  : coding type                            */
    const Word16 Es_pred_fx,             /* i  : predicted scaled innov. energy         */
    Word16 *pitch_buf_fx,          /* o  : floating pitch values for each subframe*/
    Word16 *voice_factors_fx,      /* o  : voicing factors                        */
    Word16 *exc_fx,                /* i/o: adapt. excitation exc                  */
    Word16 *exc2_fx,               /* i/o: adapt. excitation/total exc            */
    Word16 *bwe_exc_fx,            /* i/o: excitation for SWB TBE                 */
    Word16 *unbits,                /* i/o: number of unused bits                  */
    const Word16 sharpFlag,              /* i  : formant sharpening flag                */
    Word16 *gain_buf
)
{
    Word16 T0, T0_frac, T0_min, T0_max; /* integer pitch variables                     */
    Word32 gain_code_fx=0;              /* Quantized algebraic codeebook gain          */
    Word32 norm_gain_code_fx=0;         /* normalized algebraic codeebook gain         */
    Word16 gain_pit_fx = 0;             /* Quantized pitch gain                        */
    Word16 voice_fac_fx;                /* Voicing factor                              */
    Word16 gain_inov_fx=0;              /* inovation gain                              */
    Word16 code_fx[L_SUBFR];            /* algebraic codevector                        */
    const Word16 *p_Aq_fx;              /* pointer to lp filter coefficient            */
    Word16 *pt_pitch_fx;                /* pointer to floating pitch                   */
    Word16 i_subfr, i;                  /* tmp variables                               */
    Word16 position;                    /* TC related flag                             */
    Word16 gain_preQ_fx = 0;            /* Gain of prequantizer excitation             */
    Word16 code_preQ_fx[L_SUBFR];       /* Prequantizer excitation                     */
    Word16 Jopt_flag;                   /* flag indicating zero adaptive contribtuion  */
    Word32 norm_gain_preQ_fx;
    Word16 gain_code16;
    Word32 L_tmp;
    Word16 tmp16, tmp1_fx, tmp_fx;
    gain_preQ_fx = 0;
    move16();
    set16_fx( code_preQ_fx, 0, L_SUBFR );
    /*----------------------------------------------------------------*
     * ACELP subframe loop
     *----------------------------------------------------------------*/

    p_Aq_fx = Aq_fx;
    move16();
    pt_pitch_fx = pitch_buf_fx;
    move16();
    Jopt_flag = 0;
    move16();
    norm_gain_preQ_fx = 0;
    move16();

    FOR( i_subfr = 0; i_subfr < L_frame_fx; i_subfr += L_SUBFR )
    {
        /*------------------------------------------------------------*
         * TC : subframe determination &
         * adaptive/glottal part of excitation construction
         *------------------------------------------------------------*/

        test();
        IF( i_subfr == 0 && sub(st_fx->Q_exc,2) > 0 )
        {
            tmp16 = sub(2, st_fx->Q_exc);
            Scale_sig(exc_fx-L_EXC_MEM, L_EXC_MEM, tmp16);
            Scale_sig(bwe_exc_fx-PIT16k_MAX*2, PIT16k_MAX*2, tmp16);
            Scale_sig(st_fx->last_exc_dct_in_fx, L_FRAME, tmp16);
            st_fx->Q_exc = add(st_fx->Q_exc,tmp16);
        }

        transition_dec_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, i_subfr, coder_type_fx, tc_subfr_fx, &Jopt_flag, exc_fx,
                           &T0, &T0_frac, &T0_min, &T0_max, &pt_pitch_fx, &position, bwe_exc_fx, &st_fx->Q_exc );

        /*-----------------------------------------------------------------*
         * Transform domain contribution decoding - active frames
         *-----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            gain_code_fx = 0;
            move16();
            transf_cdbk_dec_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, 0, i_subfr, tc_subfr_fx,
                                Es_pred_fx, gain_code_fx, &st_fx->mem_preemp_preQ_fx, &gain_preQ_fx, &norm_gain_preQ_fx, code_preQ_fx, unbits);
        }

        /*-----------------------------------------------------------------*
         * ACELP codebook search + pitch sharpening
         *-----------------------------------------------------------------*/

        inov_decode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, coder_type_fx,
                        sharpFlag, i_subfr, tc_subfr_fx, p_Aq_fx, st_fx->tilt_code_fx, *pt_pitch_fx, code_fx );

        /*-----------------------------------------------------------------*
         * De-quantize the gains
         * Update tilt of code: 0.0 (unvoiced) to 0.5 (voiced)
         *-----------------------------------------------------------------*/

        IF( Jopt_flag == 0 )
        {
            /* 2/3-bit decoding */
            gain_dec_tc_fx( st_fx, st_fx->core_brate_fx, code_fx, L_frame_fx, i_subfr, tc_subfr_fx,  Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );
        }
        ELSE
        {
            /* 5-bit decoding */
            IF( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 )
            {
                gain_dec_SQ_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, i_subfr, tc_subfr_fx, code_fx, Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );
            }
            ELSE
            {
                gain_dec_mless_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, coder_type_fx, i_subfr, tc_subfr_fx , code_fx,
                Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );

            }
        }

        /* update LP filtered gains for the case of frame erasures */
        lp_gain_updt_fx( i_subfr, gain_pit_fx, L_add(norm_gain_code_fx,norm_gain_preQ_fx), &st_fx->lp_gainp_fx, &st_fx->lp_gainc_fx, L_frame_fx );

        st_fx->tilt_code_fx = est_tilt_fx( exc_fx+i_subfr, gain_pit_fx, code_fx, gain_code_fx,&voice_fac_fx,st_fx->Q_exc);

        /*----------------------------------------------------------------------*
         * Find the total excitation
         *----------------------------------------------------------------------*/

        IF ( sub(L_frame_fx,L_FRAME) == 0 ) /* Rescaling for 12.8k core */
        {
            Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr], &bwe_exc_fx[i_subfr * HIBND_ACB_L_FAC], st_fx->last_exc_dct_in_fx,
                         L_SUBFR, L_SUBFR * HIBND_ACB_L_FAC, gain_code_fx, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, i_subfr, coder_type_fx );
        }
        ELSE    /* Rescaling for 16k core */
        {
            Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr], &bwe_exc_fx[i_subfr * 2], st_fx->last_exc_dct_in_fx,
            L_SUBFR, L_SUBFR * 2, gain_code_fx, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, i_subfr, coder_type_fx );
        }

        gain_code16 = round_fx(L_shl(gain_code_fx,st_fx->Q_exc)); /*Q_exc*/
        Acelp_dec_total_exc( exc_fx, exc2_fx, gain_code16, gain_pit_fx, i_subfr, code_fx );

        /*-----------------------------------------------------------------*
         * Add the ACELP pre-quantizer contribution
         *-----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            tmp1_fx = add(15-Q_AVQ_OUT_DEC-2,st_fx->Q_exc);
            FOR( i = 0; i < L_SUBFR; i++ )
            {
                L_tmp = L_mult(gain_preQ_fx, code_preQ_fx[i]);    /* Q2 + Q10 -> Q13*/
                L_tmp = L_shl(L_tmp,tmp1_fx);                     /* Q16 + Q_exc      */
                tmp_fx = round_fx(L_tmp);

                exc2_fx[i+i_subfr] = add(exc2_fx[i+i_subfr],tmp_fx);
                move16();
                exc_fx[i+i_subfr] = add(exc_fx[i+i_subfr],tmp_fx);
                move16();
            }
        }

        /*-----------------------------------------------------------------*
         * Prepare TBE excitation
         *-----------------------------------------------------------------*/

        prep_tbe_exc_fx( L_frame_fx, i_subfr, gain_pit_fx, gain_code_fx, code_fx, voice_fac_fx,
                         &voice_factors_fx[i_subfr/L_SUBFR], bwe_exc_fx, gain_preQ_fx, code_preQ_fx,
                         st_fx->Q_exc, T0, T0_frac, coder_type_fx, st_fx->core_brate_fx );

        /*----------------------------------------------------------------*
         * Excitation enhancements (update of total excitation signal)
         *----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 )
        {
            Copy( exc_fx+i_subfr, exc2_fx+i_subfr, L_SUBFR );
        }
        ELSE
        {
            enhancer_fx( st_fx->core_brate_fx, 0, coder_type_fx, i_subfr, L_frame_fx, voice_fac_fx, st_fx->stab_fac_fx,
            norm_gain_code_fx, gain_inov_fx, &st_fx->gc_threshold_fx, code_fx, exc2_fx, gain_pit_fx, &(st_fx->dm_fx), st_fx->Q_exc );
        }

        p_Aq_fx += (M+1);
        move16();
        pt_pitch_fx++;
        st_fx->tilt_code_dec_fx[i_subfr/L_SUBFR] = st_fx->tilt_code_fx;
        move16();
        st_fx->gain_code_fx[i_subfr/L_SUBFR] = gain_code_fx;
        move32();
        gain_buf[i_subfr/L_SUBFR] = gain_pit_fx;
        move16();
    }

    /* SC-VBR */
    st_fx->prev_gain_pit_dec_fx = gain_pit_fx;
    move16(); /*Q14*/

    return;
}
