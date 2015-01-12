/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"


/*---------------------------------------------------------------------*
 * decod_amr_wb()
 *
 * Decode excitation signal in AMR-WB IO mode
 *---------------------------------------------------------------------*/

void decod_amr_wb_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder static memory                     */
    const Word16 *Aq_fx,                  /* i  : LP filter coefficients                    */
    Word16 *pitch_buf_fx,           /* o  : floating pitch values for each subframe   */
    Word16 *exc_fx,                 /* i/o: adapt. excitation exc                     */
    Word16 *exc2_fx,                /* i/o: adapt. excitation/total exc               */
    Word16 hf_gain_fx[NB_SUBFR],    /* o  : decoded HF gain                           */
    Word16 *voice_factors_fx,       /* o  : voicing factors                           */
    Word16 *gain_buf                /* o  :                                       Q14 */
)
{
    Word16  T0, T0_frac, T0_min, T0_max;  /* integer pitch variables                               */
    Word16 gain_pit_fx, gain_code16;      /* pitch gain                                            */
    Word32 L_gain_code_fx;                /* gain/normalized gain of the algebraic excitation      */
    Word32 L_norm_gain_code_fx;           /* normalized gain of the algebraic excitation           */
    Word16 gain_inov_fx;                  /* Innovation gain                                       */
    Word16 voice_fac_fx;                  /* voicing factor                                        */
    Word16 code_fx[L_SUBFR];              /* algebraic codevector                                  */
    const Word16 *p_Aq_fx;                /* Pointer to frame LP coefficient                       */
    Word16 *pt_pitch_fx;                  /* pointer to floating pitch                             */
    Word16 i_subfr;                       /* tmp variables                                         */
    Word16 pitch_limit_flag;
    Word32 L_Voice_fac_ave, L_tmp;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    p_Aq_fx = Aq_fx;                      /* pointer to interpolated LPC parameters */
    pt_pitch_fx = pitch_buf_fx;           /* pointer to the pitch buffer */
    L_Voice_fac_ave = L_deposit_l(0);
    pitch_limit_flag = 0;
    move16();  /* always restrained pitch Q range in IO mode */

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/

    FOR( i_subfr = 0; i_subfr < L_FRAME; i_subfr += L_SUBFR )
    {
        /*----------------------------------------------------------------------*
         * Decode pitch lag
         *----------------------------------------------------------------------*/

        *pt_pitch_fx = pit_decode_fx( st_fx, st_fx->core_brate_fx, 1, L_FRAME, i_subfr, -1, &pitch_limit_flag, &T0, &T0_frac, &T0_min, &T0_max, L_SUBFR );

        /*--------------------------------------------------------------*
         * Find the adaptive codebook vector
         *--------------------------------------------------------------*/

        pred_lt4(&exc_fx[i_subfr],&exc_fx[i_subfr], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

        /*--------------------------------------------------------------*
         * LP filtering of the adaptive excitation
         *--------------------------------------------------------------*/

        lp_filt_exc_dec_fx( st_fx, MODE1, st_fx->core_brate_fx, 1, -1, i_subfr, L_SUBFR, L_FRAME, 0, exc_fx );

        /*--------------------------------------------------------------*
         * Innovation decoding
         *--------------------------------------------------------------*/

        inov_decode_fx( st_fx, st_fx->core_brate_fx, 1, L_FRAME, -1, 0, i_subfr, -1, p_Aq_fx, st_fx->tilt_code_fx, *pt_pitch_fx, code_fx);

        /*--------------------------------------------------------------*
         * Gain decoding
         * Estimate spectrum tilt and voicing
         *--------------------------------------------------------------*/

        gain_dec_amr_wb_fx( st_fx, st_fx->core_brate_fx, &gain_pit_fx, &L_gain_code_fx, st_fx->past_qua_en_fx, &gain_inov_fx, code_fx, &L_norm_gain_code_fx );

        /* update LP filtered gains for the case of frame erasures */
        lp_gain_updt_fx( i_subfr, gain_pit_fx, L_norm_gain_code_fx, &st_fx->lp_gainp_fx, &st_fx->lp_gainc_fx, L_FRAME );

        st_fx->tilt_code_fx = est_tilt_fx( exc_fx+i_subfr, gain_pit_fx, code_fx, L_gain_code_fx, &voice_fac_fx, st_fx->Q_exc );

        Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr], NULL, st_fx->last_exc_dct_in_fx, L_SUBFR, 0,
                     L_gain_code_fx, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, i_subfr, -1 );

        gain_code16 = round_fx(L_shl(L_gain_code_fx,st_fx->Q_exc)); /*Q_exc*/

        /*----------------------------------------------------------------------*
         * Find the total excitation
         *----------------------------------------------------------------------*/

        Acelp_dec_total_exc( exc_fx, exc2_fx, gain_code16, gain_pit_fx, i_subfr, code_fx );

        /*----------------------------------------------------------------*
         * Excitation enhancements
         *----------------------------------------------------------------*/

        enhancer_fx( st_fx->core_brate_fx, 1, -1, i_subfr, L_FRAME, voice_fac_fx, st_fx->stab_fac_fx,
                     L_norm_gain_code_fx, gain_inov_fx, &st_fx->gc_threshold_fx, code_fx, exc2_fx, gain_pit_fx, &(st_fx->dm_fx),st_fx->Q_exc );

        /*-----------------------------------------------------------------*
         * HF gain modification factors at 23.85 kbps
         *-----------------------------------------------------------------*/

        IF ( L_sub(st_fx->core_brate_fx,ACELP_23k85) == 0 )
        {
            hf_gain_fx[shr(i_subfr,6)] = (Word16)get_next_indice_fx(st_fx, 4);
        }

        /*voice_fac = VF_0th_PARAM + VF_1st_PARAM * voice_fac + VF_2nd_PARAM * voice_fac * voice_fac;*/
        L_tmp = L_mult(VF_2nd_PARAM_FX, mult_r(voice_fac_fx, voice_fac_fx));
        L_tmp = L_mac(L_tmp, VF_1st_PARAM_FX, voice_fac_fx);
        voice_fac_fx = mac_r(L_tmp, VF_0th_PARAM_FX, 32767);

        /*voice_factors[i_subfr/L_SUBFR] = min( max(0.0f, voice_fac), 1.0f);*/
        voice_factors_fx[i_subfr/L_SUBFR] = s_min(s_max(0, voice_fac_fx),32767);
        move16();
        p_Aq_fx += (M+1);
        pt_pitch_fx++;

        L_Voice_fac_ave = L_mac(L_Voice_fac_ave, 8192, voice_fac_fx);
        gain_buf[i_subfr/L_SUBFR] = gain_pit_fx;
    }

    st_fx->lt_voice_fac_fx = round_fx(L_Voice_fac_ave);

    return;
}
