/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*==========================================================================*/
/* FUNCTION : void  dec_pit_exc_fx()       	    							*/
/*--------------------------------------------------------------------------*/
/* PURPOSE  :  Decode pitch only contribution                          	    */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word16*) Aq_fx  		   : LP filter coefficient		Q12			    */
/* _ (Word16) coder_type_fx    : coding type				Q0			    */
/* _ (Word16) nb_subfr_fx      :Number of subframe considered               */
/* _ (Word16) Es_pred_fx       :predicted scaled innov. energy              */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ (Word16*) pitch_buf_fx	: floating pitch values for each subframe Q6    */
/* _ (Word16*)	code_fx     : innovation                                    */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/*  Decoder_State_fx *st_fx     : decoder state structure                   */
/* _ (Word16*) exc_fx			: adapt. excitation exc         		    */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _ None																    */
/*==========================================================================*/
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
    , Word16 *gain_buf              /*Q14*/
)
{
    Word16 T0_fx, T0_frac_fx, T0_min_fx, T0_max_fx;/* integer pitch variables    */
    Word16 gain_pit_fx = 0;        /* pitch gain               Q14                            */
    Word32 gain_code_fx;           /* gain/normalized gain of the algebraic excitation Q16      */
    Word32 norm_gain_code_fx;      /* normalized gain of the algebraic excitation        Q16   */
    Word16 gain_inov_fx;           /* Innovation gain  Q12                                      */
    Word16 voice_fac_fx;           /* voicing factor  Q15                                      */
    Word16 L_subfr_fx,pit_idx_fx;
    const Word16 *p_Aq_fx;         /* Pointer to frame LP coefficient Q12                      */
    Word16 *pt_pitch_fx;           /* pointer to floating pitch     Q6                        */
    Word16 i_subfr_fx, i;          /* tmp variables                                         */
    Word16 Local_BR_fx, Pitch_BR_fx, Pitch_CT_fx;
    Word16 pitch_limit_flag;
    Word16 exc2_bidon[L_SUBFR];
    Word16 *pt_gain;            /* Pointer to floating gain values for each subframe     */

    Word16 gain_code16,gain_pitx2;
    Word32 L_tmp;
    Word16 nbits;

    IF( sub(st_fx->GSC_noisy_speech_fx,1) == 0 )
    {
        Local_BR_fx = ACELP_7k20;
        move16();
        Pitch_CT_fx = GENERIC;
        move16();
        Pitch_BR_fx = ACELP_7k20;
        move16();
    }
    ELSE
    {
        Local_BR_fx = ACELP_7k20;
        move16();
        Pitch_CT_fx = AUDIO;
        move16();
        Pitch_BR_fx = extract_l(st_fx->core_brate_fx);
    }
    gain_code_fx = 0;
    move16();
    pitch_limit_flag = 1;
    move16();/* always extended pitch Q range */

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/
    L_subfr_fx = mult_r(L_FRAME,div_s(1,nb_subfr_fx));
    p_Aq_fx = Aq_fx;              /* pointer to interpolated LPC parameters */
    pt_pitch_fx = pitch_buf_fx; /* pointer to the pitch buffer */
    pt_gain = gain_buf;         /* pointer to the gain buffer  */
    FOR ( i_subfr_fx = 0; i_subfr_fx < L_FRAME; i_subfr_fx += L_subfr_fx )
    {
        /*----------------------------------------------------------------------*
         *  Decode pitch lag
        *----------------------------------------------------------------------*/

        *pt_pitch_fx = pit_decode_fx( st_fx, Pitch_BR_fx, 0, L_FRAME, i_subfr_fx, Pitch_CT_fx, &pitch_limit_flag, &T0_fx, &T0_frac_fx, &T0_min_fx, &T0_max_fx, L_subfr_fx );
        move16();

        /*--------------------------------------------------------------*
         * Find the adaptive codebook vector.
         *--------------------------------------------------------------*/

        pred_lt4( &exc_fx[i_subfr_fx], &exc_fx[i_subfr_fx], T0_fx, T0_frac_fx, L_subfr_fx+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );

        /*--------------------------------------------------------------*
         * Innovation decoding
         *--------------------------------------------------------------*/

        IF( sub(st_fx->GSC_noisy_speech_fx,1) == 0)
        {
            inov_decode_fx( st_fx, Local_BR_fx, 0, L_FRAME, LOCAL_CT, 1, i_subfr_fx, -1, p_Aq_fx, st_fx->tilt_code_fx, *pt_pitch_fx, code_fx );
            /*--------------------------------------------------------------*
             * Gain decoding
             * Estimate spectrum tilt and voicing
             *--------------------------------------------------------------*/

            gain_dec_mless_fx( st_fx, Local_BR_fx, L_FRAME, LOCAL_CT, i_subfr_fx, -1, code_fx, Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );

            st_fx->tilt_code_fx = est_tilt_fx( exc_fx+i_subfr_fx, gain_pit_fx, code_fx, gain_code_fx, &voice_fac_fx,0 );
        }
        ELSE
        {
            nbits = 4;
            move16();

            set16_fx(code_fx, 0, L_SUBFR);
            gain_code_fx = L_deposit_l(0);
            st_fx->tilt_code_fx = 0;
            move16();
            pit_idx_fx = (Word16)get_next_indice_fx( st_fx, nbits );
            move16();

            gain_pit_fx = add(9590,dic_gp_fx[pit_idx_fx]);
            move16();     /*Q14  0.5853 in Q14 9590*/
            gain_code_fx = L_shl(gain_pit_fx, 10);  /* Use gain pitch as an indicator to help finding the best scaling value. gain_code_fx used a temp var*/
        }

        /*----------------------------------------------------------------------*
         * Find the total excitation
         *----------------------------------------------------------------------*/

        Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr_fx], &bwe_exc_fx[i_subfr_fx * HIBND_ACB_L_FAC], st_fx->last_exc_dct_in_fx,
                     L_subfr_fx, L_subfr_fx * HIBND_ACB_L_FAC, gain_code_fx, &(st_fx->Q_exc), st_fx->Q_subfr, NULL, i_subfr_fx, coder_type_fx);

        gain_code16 = round_fx(L_shl(gain_code_fx,st_fx->Q_exc)); /*Q_exc*/

        IF( sub(st_fx->GSC_noisy_speech_fx,1) == 0)
        {
            Acelp_dec_total_exc( exc_fx, exc2_bidon-i_subfr_fx, gain_code16, gain_pit_fx, i_subfr_fx, code_fx );
        }
        ELSE
        {
            IF (norm_s(s_or(gain_pit_fx, 1)) == 0)
            {
                FOR (i = 0; i < L_subfr_fx; i++)
                {
                    L_tmp = L_shl(L_mult(gain_pit_fx, exc_fx[i+i_subfr_fx]), 1); /*Q16+Q_exc*/
                    exc_fx[i+i_subfr_fx] = round_fx(L_tmp); /*Q_exc*/
                }
            }
            ELSE
            {
                gain_pitx2 = shl(gain_pit_fx, 1); /*Q15*/

                FOR (i = 0; i < L_subfr_fx; i++)
                {
                    L_tmp = L_mult(gain_pitx2, exc_fx[i+i_subfr_fx]); /*Q16+Q_exc*/
                    exc_fx[i+i_subfr_fx] = round_fx(L_tmp);  /*Q_exc*/
                }
            }
        }
        IF( sub(L_subfr_fx,128) == 0)         /*2*L_SUBFR*/
        {
            p_Aq_fx += 2*(M+1);
            move16();
            pt_pitch_fx++;
            *pt_pitch_fx = *(pt_pitch_fx-1);
            move16();
            pt_pitch_fx++;
            *pt_gain = gain_pit_fx;
            move16();
            pt_gain++;
            *pt_gain = *(pt_gain-1);
            move16();
            pt_gain++;
            IF( i_subfr_fx == 0 )
            {
                /* update gains for FEC - equivalent to lp_gain_updt() */
                st_fx->lp_gainp_fx = extract_h(L_mult(9830,gain_pit_fx));     /*Q14 (3/10 in Q15 9830)*/
                st_fx->lp_gainc_fx = 0;
                move16();
            }
            ELSE
            {
                /* update gains for FEC - equivalent to lp_gain_updt() */
                st_fx->lp_gainp_fx = extract_h(L_mult(22938,gain_pit_fx));   /*Q14 (7/10 in Q15 22938)*/
                st_fx->lp_gainc_fx = 0;
                move16();
            }
        }
        ELSE IF( sub(L_subfr_fx,256) == 0)                   /*4*L_SUBFR*/
        {
            pt_pitch_fx++;
            *pt_pitch_fx = *(pt_pitch_fx-1);
            move16();
            pt_pitch_fx++;
            *pt_pitch_fx = *(pt_pitch_fx-1);
            move16();
            pt_pitch_fx++;
            *pt_pitch_fx = *(pt_pitch_fx-1);
            move16();
            pt_pitch_fx++;
            *pt_gain = gain_pit_fx;
            move16();
            pt_gain++;
            *pt_gain = *(pt_gain-1);
            move16();
            pt_gain++;
            *pt_gain = *(pt_gain-1);
            move16();
            pt_gain++;
            *pt_gain = *(pt_gain-1);
            move16();
            pt_gain++;
            p_Aq_fx += 4*(M+1);
            move16();

            /* update gains for FEC - equivalent to lp_gain_updt() */
            st_fx->lp_gainp_fx = gain_pit_fx;
            move16();
            st_fx->lp_gainc_fx = 0;
            move16();
        }
        ELSE
        {
            p_Aq_fx += (M+1);
            move16();
            pt_pitch_fx++;
            move16();
            *pt_gain = gain_pit_fx;
            move16();
            pt_gain++;

            lp_gain_updt_fx( i_subfr_fx, gain_pit_fx, 0, &st_fx->lp_gainp_fx, &st_fx->lp_gainc_fx, L_FRAME );
        }
    }

    return;
}
