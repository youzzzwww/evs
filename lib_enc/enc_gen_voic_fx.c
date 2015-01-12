/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_com_fx.h"     /* Static table prototypes                */

#include "stl.h"


/*======================================================================*/
/* FUNCTION : encod_gen_voic_fx()										*/
/*----------------------------------------------------------------------*/
/* PURPOSE : encode generic (GC), voiced (VC) and AMR-WB IO frames		*/
/*																		*/
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    */
/* _ (Struct)	st_fx			: encoder static memory					*/
/* _ (Word16) L_frame_fx		: length of the frame		    Q0		*/

/* _ (Word16[]) speech_fx			: input speech		        Q0		*/
/* _ (Word16[]) Aq_fx			: LP filter coefficient		    Q12		*/
/* _ (Word16[]) A_fx			: unquantized A(z) filter               */
/*                                with bandwidth expansion 		Q12	    */
/* _ (Word16) coder_type_fx		: coding type							*/
/* _ (Word16) Es_pred_fx		: predicted scaled innov. energy Q8		*/
/* _ (Word16[]) T_op_fx			: open loop pitch Q0					*/
/* _ (Word16[]) voicing_fx	    : floating pitch values for each subframe Q15*/
/* _ (Word16[]) res_fx			: residual signal				Q_new	*/
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q0)		 */
/* _ (Word16[]) pitch_buf_fx	: floating pitch values for each subframe Q6*/
/* _ (Word16) shift		: 	shift										 */
/* _ (Word16) Q_new		: 			                                     */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q0)		 */
/* _ (Word16[]) syn_fx			:core synthesis					Q_new	*/
/* _ (Word16[])	voice_factors_fx: voicing factors 				Q15		 */
/* _ (Word16[]) bwe_exc_fx		: excitation for SWB TBE        Q0		 */
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/

void encod_gen_voic_fx(
    Encoder_State_fx *st_fx,                 /* i/o: state structure                                  */
    LPD_state   *mem,                  /* i/o: acelp memories                                   */
    const Word16 L_frame_fx,               /* i  : length of the frame                              */
    const Word16 sharpFlag_fx,             /* o  : formant sharpening flag                          */
    const Word16 speech_fx[],              /* i  : input speech                                     */
    const Word16 Aw_fx[],                  /* i  : weighted A(z) unquantized for subframes          */
    const Word16 Aq_fx[],                  /* i  : 12k8 Lp coefficient                              */
    const Word16 coder_type_fx,            /* i  : coding type                                      */
    const Word16 Es_pred_fx,               /* i  : predicted scaled innov. energy                   */
    const Word16 T_op_fx[],                /* i  : open loop pitch                                  */
    const Word16 voicing_fx[],             /* i  : voicing                                          */
    const Word16 *res_fx,                  /* i  : residual signal                                  */
    Word16 *syn_fx,                  /* i/o: core synthesis                                   */
    Word16 *exc_fx,                  /* i/o: current non-enhanced excitation                  */
    Word16 *exc2_fx,                 /* i/o: current enhanced excitation                      */
    Word16 *pitch_buf_fx,            /* i/o: floating pitch values for each subframe          */
    Word16 *voice_factors_fx,        /* o  : voicing factors                                  */
    Word16 *bwe_exc_fx,              /* o  : excitation for SWB TBE                           */
    Word16 *unbits_fx,               /* i/o: number of unused bits							*/
    Word16 shift,
    Word16 Q_new
)
{
    Word16 xn_fx[L_SUBFR];                  /* Target vector for pitch search    */
    Word16 xn2_fx[L_SUBFR];                 /* Target vector for codebook search */
    Word16 cn_fx[L_SUBFR];                  /* Target vector in residual domain  */
    Word16 h1_fx[L_SUBFR+(M+1)];            /* Impulse response vector           */
    Word16 h2_fx[L_SUBFR+(M+1)];            /* Impulse response vector           */
    Word16 code_fx[L_SUBFR];                /* Fixed codebook excitation         */
    Word16 y1_fx[L_SUBFR]= {0};             /* Filtered adaptive excitation      */
    Word16 y2_fx[L_SUBFR];                  /* Filtered algebraic excitation     */
    Word16 gain_pit_fx = 0;                 /* Pitch gain                        */
    Word16 voice_fac_fx;                    /* Voicing factor                    */
    Word32 gain_code_fx = 0;                /* Gain of code                      */
    Word16 gain_inov_fx=0;                  /* inovation gain                    */
    Word32 gc_mem[NB_SUBFR-1];              /* gain_code from previous subframes */
    Word16 gp_mem[NB_SUBFR-1];              /* gain_pitch from previous subframes*/
    Word16 i, i_subfr_fx;                   /* tmp variables                     */
    Word16 T0_fx=0, T0_frac_fx=0;           /* close loop integer pitch and fractional part */
    Word16 T0_min_fx, T0_max_fx;            /* pitch variables                   */
    Word16 *pt_pitch_fx;                    /* pointer to floating pitch buffer  */
    Word16 g_corr_fx[10];                   /* ACELP correl, values + gain pitch */
    Word16 clip_gain_fx;                    /* LSF clip gain                     */
    const Word16 *p_Aw_fx, *p_Aq_fx;        /* pointer to LP filter coeff. vector*/
    Word16 error_fx = 0;
    Word16 gain_preQ_fx = 0;                /* Gain of prequantizer excitation   */
    Word16 code_preQ_fx[L_SUBFR];           /* Prequantizer excitation           */
    Word16 unbits_PI_fx = 0;                /* number of unused bits for  PI     */
    Word32 norm_gain_code_fx=0;
    Word16 pitch_limit_flag;
    Word16 Gain_pitX2,gcode16;
    Word32 Ltmp;
    Word32 Ltmp1;
    Word32 Lgcode;
    Word16 tmp1_fx;
    Word16 shift_wsp;
    Word16 harm_flag_acelp;
    Word16 lp_select, lp_flag;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    gain_pit_fx = 0;
    move16();
    gain_code_fx = L_deposit_l(0);
    gain_preQ_fx = 0;
    move16();
    unbits_PI_fx = 0;
    move16();
    error_fx = 0;
    move16();

    IF( sub(L_frame_fx,L_FRAME) == 0)
    {
        T0_max_fx = PIT_MAX;
        move16();
        T0_min_fx = PIT_MIN;
        move16();
    }
    ELSE /* L_frame == L_FRAME16k */
    {
        T0_max_fx = PIT16k_MAX;
        move16();
        T0_min_fx = PIT16k_MIN;
        move16();
    }

    p_Aw_fx = Aw_fx;
    p_Aq_fx = Aq_fx;
    pt_pitch_fx = pitch_buf_fx;
    gain_preQ_fx = 0;
    move16();
    set16_fx( code_preQ_fx, 0, L_SUBFR );

    shift_wsp = add(Q_new,shift);

    /* set and write harmonicity flag */
    harm_flag_acelp = 0;
    move16();
    test();
    test();
    IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && L_sub(st_fx->core_brate_fx,ACELP_32k) <= 0 && sub(coder_type_fx,GENERIC) == 0 )
    {
        if( sub(st_fx->last_harm_flag_acelp_fx,2) > 0 )
        {
            harm_flag_acelp = 1;
            move16();
        }

        push_indice_fx( st_fx, IND_HARM_FLAG_ACELP, harm_flag_acelp, 1 );
    }

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/

    FOR( i_subfr_fx=0; i_subfr_fx<L_frame_fx; i_subfr_fx+=L_SUBFR )
    {

        /*----------------------------------------------------------------*
         * Find the the excitation search target "xn" and innovation
         *   target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/

        Copy( &res_fx[i_subfr_fx], &exc_fx[i_subfr_fx], L_SUBFR );

        find_targets_fx( speech_fx, mem->mem_syn, i_subfr_fx, &mem->mem_w0, p_Aq_fx,
                         res_fx, L_SUBFR, p_Aw_fx, st_fx->preemph_fac, xn_fx, cn_fx, h1_fx );

        Copy_Scale_sig( h1_fx, h2_fx, L_SUBFR, -2 );
        Scale_sig( h1_fx, L_SUBFR, add(1, shift) ); /* set h1[] in Q14 with scaling for convolution */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig( xn_fx, L_SUBFR, shift );

        *pt_pitch_fx = pit_encode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, coder_type_fx, &pitch_limit_flag, i_subfr_fx, exc_fx,
                                      L_SUBFR, T_op_fx, &T0_min_fx, &T0_max_fx, &T0_fx, &T0_frac_fx, h1_fx, xn_fx );

        tbe_celp_exc(L_frame_fx,i_subfr_fx,T0_fx, T0_frac_fx, &error_fx, bwe_exc_fx);

        /*-----------------------------------------------------------------*
         * Find adaptive exitation
         *-----------------------------------------------------------------*/

        pred_lt4(&exc_fx[i_subfr_fx], &exc_fx[i_subfr_fx], T0_fx, T0_frac_fx, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

        /*-----------------------------------------------------------------*
         * Gain clipping test to avoid unstable synthesis on frame erasure
         *-----------------------------------------------------------------*/

        clip_gain_fx = gp_clip_fx(voicing_fx,i_subfr_fx,coder_type_fx,xn_fx,st_fx->clip_var_fx,sub(shift_wsp, 1));

        if( sub(coder_type_fx,INACTIVE) == 0 )
        {
            /* in case of AVQ inactive, limit the gain to 0.65 */
            clip_gain_fx = 2;
            move16();
        }

        /*-----------------------------------------------------------------*
         * LP filtering of the adaptive excitation, codebook target computation
         *-----------------------------------------------------------------*/

        lp_select = lp_filt_exc_enc_fx( MODE1, st_fx->core_brate_fx, 0, coder_type_fx, i_subfr_fx, exc_fx, h1_fx,
                                        xn_fx, y1_fx, xn2_fx, L_SUBFR, L_frame_fx, g_corr_fx, clip_gain_fx, &gain_pit_fx, &lp_flag );

        IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
        {
            push_indice_fx( st_fx, IND_LP_FILT_SELECT, lp_select, 1 );
        }

        /*st_fx->lowrate_pitchGain = 0.9f * st_fx->lowrate_pitchGain + 0.1f * gain_pit_fx;*/
        st_fx->lowrate_pitchGain = round_fx(L_mac(L_mult(29491, st_fx->lowrate_pitchGain), 6554, gain_pit_fx)); /*Q14*Q16(0.1) + Q15 -> Q15*/

        /*-----------------------------------------------------------------*
         * Transform domain contribution encoding - active frames
         *-----------------------------------------------------------------*/

        test();
        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && sub(coder_type_fx,INACTIVE) != 0 )
        {
            transf_cdbk_enc_fx( st_fx, st_fx->core_brate_fx, st_fx->extl_fx, coder_type_fx, harm_flag_acelp, i_subfr_fx, -1, cn_fx, exc_fx,
                                p_Aq_fx, p_Aw_fx, h1_fx, xn_fx, xn2_fx, y1_fx, y2_fx, Es_pred_fx, &gain_pit_fx, gain_code_fx, g_corr_fx, clip_gain_fx,
                                &(st_fx->mem_deemp_preQ_fx), &(st_fx->mem_preemp_preQ_fx), &gain_preQ_fx, code_preQ_fx, unbits_fx,  Q_new, shift);
        }

        /*-----------------------------------------------------------------*
         * Innovation encoding
         *-----------------------------------------------------------------*/

        inov_encode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, st_fx->last_L_frame_fx,
                        coder_type_fx, st_fx->bwidth_fx, sharpFlag_fx, i_subfr_fx, -1, p_Aq_fx,
                        gain_pit_fx, cn_fx, exc_fx, h2_fx, mem->tilt_code, *pt_pitch_fx, xn2_fx, code_fx, y2_fx, &unbits_PI_fx, shift );

        /*-----------------------------------------------------------------*
         * Gain encoding
         *-----------------------------------------------------------------*/

        IF ( L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0)
        {
            gain_enc_lbr_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, i_subfr_fx, xn_fx, y1_fx, shift_wsp, y2_fx, code_fx,
                             &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx, g_corr_fx, gc_mem, gp_mem, clip_gain_fx );
        }
        ELSE IF ( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 )
        {
            gain_enc_SQ_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, i_subfr_fx, -1, xn_fx, y1_fx, y2_fx, code_fx, Es_pred_fx,
                            &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx, g_corr_fx, clip_gain_fx, shift_wsp );
        }
        ELSE
        {
            gain_enc_mless_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, coder_type_fx, i_subfr_fx, -1, xn_fx, y1_fx, shift_wsp, y2_fx, code_fx, Es_pred_fx,
            &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx, g_corr_fx, clip_gain_fx );
        }

        if ( sub(st_fx->last_ppp_mode_fx,1) == 0 )
        {
            /* SC-VBR - all other st->clip_var values will be updated even in a PPP frame */
            st_fx->clip_var_fx[1] = gain_pit_fx;
            move16();
        }
        gp_clip_test_gain_pit_fx( gain_pit_fx, st_fx->clip_var_fx );

        Lgcode = L_shl(gain_code_fx, Q_new);      /* scaled gain_code with Qnew -> Q16*/
        gcode16 = round_fx(Lgcode);

        mem->tilt_code = Est_tilt2(&exc_fx[i_subfr_fx], gain_pit_fx, code_fx, Lgcode, &voice_fac_fx, shift);

        /*-----------------------------------------------------------------*
         * Transform domain contribution encoding - inactive frames
         *-----------------------------------------------------------------*/

        test();
        IF ( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && sub(coder_type_fx,INACTIVE) == 0 )
        {
            transf_cdbk_enc_fx( st_fx, st_fx->core_brate_fx, st_fx->extl_fx, coder_type_fx, 0, i_subfr_fx, -1, cn_fx, exc_fx,
                                p_Aq_fx, p_Aw_fx, h1_fx, xn_fx, xn2_fx, y1_fx, y2_fx, Es_pred_fx, &gain_pit_fx, gain_code_fx, g_corr_fx, clip_gain_fx,
                                &(st_fx->mem_deemp_preQ_fx), &(st_fx->mem_preemp_preQ_fx), &gain_preQ_fx, code_preQ_fx, unbits_fx,  Q_new, shift);
        }

        IF (norm_s(gain_pit_fx) == 0)
        {
            FOR (i = 0; i < L_SUBFR; i++)
            {
                exc2_fx[i+i_subfr_fx] = round_fx(L_shl(L_mult(gain_pit_fx,exc_fx[i+i_subfr_fx]),1));
            }
        }
        ELSE
        {
            Gain_pitX2 = shl(gain_pit_fx, 1);
            FOR (i = 0; i < L_SUBFR; i++)
            {
                exc2_fx[i+i_subfr_fx] = mult_r(Gain_pitX2,exc_fx[i+i_subfr_fx]);
            }
        }

        /*-----------------------------------------------------------------*
          * Update memory of the weighting filter
          *-----------------------------------------------------------------*/

        /* st_fx->mem_w0 = xn[L_SUBFR-1] - (gain_pit*y1[L_SUBFR-1]) - (gain_code*y2[L_SUBFR-1]); */
        Ltmp = L_mult(gcode16, y2_fx[L_SUBFR - 1]);
        Ltmp = L_shl(Ltmp, add(5, shift));
        Ltmp = L_negate(Ltmp);
        Ltmp = L_mac(Ltmp, xn_fx[L_SUBFR - 1], 16384);
        Ltmp = L_msu(Ltmp, y1_fx[L_SUBFR - 1], gain_pit_fx);
        Ltmp = L_shl(Ltmp, sub(1, shift));
        mem->mem_w0 = round_fx(Ltmp);     /*Q_new-1        */

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            tmp1_fx = add(16-(2+Q_AVQ_OUT_DEC+1),Q_new);

            FOR( i = 0; i < L_SUBFR; i++ )
            {
                /* Contribution from AVQ layer */
                Ltmp1 = L_mult(gain_preQ_fx, code_preQ_fx[i]);    /* Q2 + Q6 -> Q9*/
                Ltmp1 = L_shl(Ltmp1,tmp1_fx);                     /* Q16 + Q_exc */

                /* Compute exc2 */
                Ltmp = L_shl(L_mult(gain_pit_fx,exc_fx[i+i_subfr_fx]),1);
                exc2_fx[i+i_subfr_fx] = round_fx(L_add(Ltmp, Ltmp1));

                /* code in Q9, gain_pit in Q14 */
                Ltmp = L_mult(gcode16, code_fx[i]);
                Ltmp = L_shl(Ltmp, 5);
                Ltmp = L_mac(Ltmp, exc_fx[i + i_subfr_fx], gain_pit_fx);
                Ltmp = L_shl(Ltmp, 1); /* saturation can occur here */

                exc_fx[i+i_subfr_fx] = round_fx(L_add(Ltmp, Ltmp1));
            }

        }
        ELSE
        {
            /*-----------------------------------------------------------------*
             * Construct adaptive part of the excitation
             * Save the non-enhanced excitation for FEC_exc
             *-----------------------------------------------------------------*/
            FOR (i = 0; i < L_SUBFR; i++)
            {
                /* code in Q9, gain_pit in Q14 */
                Ltmp = L_mult(gcode16, code_fx[i]);
                Ltmp = L_shl(Ltmp, 5);
                Ltmp = L_mac(Ltmp, exc_fx[i + i_subfr_fx], gain_pit_fx);
                Ltmp = L_shl(Ltmp, 1); /* saturation can occur here */
                exc_fx[i + i_subfr_fx] = round_fx(Ltmp);
            }
        }
        /*-----------------------------------------------------------------*
         * Prepare TBE excitation
         *-----------------------------------------------------------------*/

        prep_tbe_exc_fx( L_frame_fx, i_subfr_fx, gain_pit_fx, gain_code_fx, code_fx, voice_fac_fx,
                         &voice_factors_fx[i_subfr_fx/L_SUBFR], bwe_exc_fx, gain_preQ_fx, code_preQ_fx, Q_new,
                         T0_fx, T0_frac_fx, coder_type_fx, st_fx->core_brate_fx );

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[].
         * Update A(z) filters
         *-----------------------------------------------------------------*/

        Syn_filt_s( 1, p_Aq_fx, M, &exc_fx[i_subfr_fx], &syn_fx[i_subfr_fx], L_SUBFR, mem->mem_syn, 1 );

        p_Aw_fx += (M+1);
        p_Aq_fx += (M+1);
        pt_pitch_fx++;
    }

    /* write reserved bits */
    WHILE( unbits_PI_fx > 0 )
    {
        i = s_min(unbits_PI_fx, 16);
        push_indice_fx( st_fx, IND_UNUSED, 0, i );
        unbits_PI_fx -= i;
    }
    /* SC-VBR */
    st_fx->prev_ppp_gain_pit_fx = gain_pit_fx;
    move16();
    st_fx->prev_tilt_code_fx = mem->tilt_code;
    move16();

    return;

}
