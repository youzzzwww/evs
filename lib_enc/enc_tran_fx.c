/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Static table prototypes                */


#include "stl.h"

/*=================================================================================*/
/* FUNCTION      :  void encod_tran_fx ()									       */
/*---------------------------------------------------------------------------------*/
/* PURPOSE       :															       */
/*---------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														       */
/*   (Word16) L_frame_fx	  : length of the frame                           Q0   */
/*	 (Word16[]) speech_fx	  : input speech 								  Q0   */
/*	 (Word16[]) Aq_fx		  : 12k8 Lp coefficient							  Q12  */
/*	 (Word16[]) A_fx		  : unquantized A(z) filter with bandwidth expansion Q12  */
/*   (Word16) coder_type	  : coding type									   Q0  */
/*   (Word16) Es_pred_fx	  : predicted scaled innov. energy			       Q8  */
/*	 (Word16[]) T_op_fx		  : open loop pitch	                               Q0  */
/*	 (Word16[]) voicing_fx	  : voicing			                               Q15 */
/*	 (Word16*) res_fx		  : residual signal	                               Q_new*/
/*	 (Word16) gsc_attack_flag : Flag to indicate when an audio attack is deal with TM*/
/*	 (Word16) shift			  : shift factor							           */
/*	 (Word16[]) Q_new		  : input scaling				                       */
/*---------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														       */
/*	 (Word16*) voice_factors  : voicing factors							Q15	       */
/*---------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													       */
/*   Encoder_State_fx *st_fx  :Encoder state structure						       */
/*	 (Word16*) syn_fx		  :core synthesis							Qnew	   */
/*	 (Word16*) exc_fx		  :current non-enhanced excitation			Q0	       */
/*	 (Word16*) exc2_fx		  :current enhanced excitation				Q0         */
/*	 (Word16*) pitch_buf_fx	  :floating pitch values for each subframe  Q6         */
/*   (Word16*) bwe_exc_fx	  :excitation for SWB TBE                   Q0         */
/*---------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														       */
/*					 _ None												           */
/*---------------------------------------------------------------------------------*/

Word16 encod_tran_fx(
    Encoder_State_fx *st_fx,                /* i/o: state structure                                   */
    LPD_state   *mem,                /* i/o: acelp memories                                    */
    const Word16 L_frame_fx,              /* i  : length of the frame                               */
    const Word16 speech_fx[],             /* i  : input speech                                      */
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq_fx[],                 /* i  : 12k8 Lp coefficient                               */
    const Word16 coder_type,              /* i  : coding type                                       */
    const Word16 Es_pred_fx,              /* i  : predicted scaled innov. energy                    */
    const Word16 T_op_fx[],               /* i  : open loop pitch                                   */
    const Word16 voicing_fx[],            /* i  : voicing                                           */
    const Word16 *res_fx,                 /* i  : residual signal                                   */
    Word16 *syn_fx,                 /* i/o: core synthesis                                    */
    Word16 *exc_fx,                 /* i/o: current non-enhanced excitation                   */
    Word16 *exc2_fx,                /* i/o: current enhanced excitation                       */
    Word16 *pitch_buf_fx,           /* i/o: floating pitch values for each subframe           */
    Word16 *voice_factors,          /* o  : voicing factors                                   */
    Word16 *bwe_exc_fx,             /* i/o: excitation for SWB TBE                            */
    const Word16 gsc_attack_flag,         /* i  : Flag to indicate when an audio attack is deal with TM */
    Word16 *unbits,                 /* i/o: number of unused bits                             */
    Word16 sharpFlag,               /* o  : formant sharpening flag                           */
    const Word16 shift,					  /* i  : Scaling to get 12 bits                            */
    const Word16 Q_new					  /* i  : Input scaling                                     */
)
{
    Word16 xn[L_SUBFR];                      /* Target vector for pitch search               */
    Word16 xn2[L_SUBFR];                     /* Target vector for codebook search            */
    Word16 cn[L_SUBFR];                      /* Target vector in residual domain             */
    Word16 h1[L_SUBFR+(M+1)];                /* Impulse response vector                      */
    Word16 h2_fx[L_SUBFR+(M+1)];             /* Impulse response vector                      */
    Word16 code[L_SUBFR];                    /* Fixed codebook excitation                    */
    Word16 y1[L_SUBFR];                      /* Filtered adaptive excitation                 */
    Word16 y2[L_SUBFR];                      /* Filtered algebraic excitation                */
    Word16 gain_pit = 0,Gain_pitX2,gcode16;  /* Pitch gain                                   */
    Word16 voice_fac;                        /* Voicing factor                               */
    Word32 gain_code = 0;                    /* Gain of code                                 */
    Word32 Lgcode;
    Word16 gain_inov=0;                      /* inovation gain                               */
    Word16 i, i_subfr, tc_subfr,tmp1_fx,tmp_fx;             /* tmp variables                                */
    Word16 position, T0_min, T0_max;         /* pitch and TC variables                       */
    Word16 T0, T0_frac;                      /* close loop integer pitch and fractional part */
    Word16 *pt_pitch;                        /* pointer to floating pitch buffer             */
    Word16 g_corr[10];                       /* ACELP correlation values  and gain pitch     */
    Word16 clip_gain;                        /* LSF clip gain                                */
    const Word16 *p_Aw, *p_Aq;               /* pointer to LP filter coefficient vector      */
    Word16 gain_preQ = 0;                    /* Gain of prequantizer excitation              */
    Word16 code_preQ[L_SUBFR];               /* Prequantizer excitation                      */
    Word16 Jopt_flag;                        /* joint optimization flag                      */
    Word16 unbits_PI = 0;                    /* saved bits for PI                            */
    Word32 norm_gain_code=0;

    Word16 shift_wsp;
    Word32 L_tmp;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/
    gain_pit = 0;
    move16();
    gain_code = L_deposit_l(0);
    gain_preQ = 0;
    move16();
    unbits_PI = 0;
    move16();
    IF( sub(L_frame_fx,L_FRAME) == 0 )
    {
        T0_max = PIT_MAX;
        move16();
        T0_min = PIT_MIN;
        move16();
    }
    ELSE /* L_frame == L_FRAME16k */
    {
        T0_max = PIT16k_MAX;
        move16();
        T0_min = PIT16k_MIN;
        move16();
    }

    /**unbits = 0;move16();*/
    Jopt_flag = 0;
    move16();
    tc_subfr = -1;
    move16();
    if( gsc_attack_flag )
    {
        tc_subfr = 3*L_SUBFR;
        move16();
    }

    p_Aw = Aw_fx;
    p_Aq = Aq_fx;
    pt_pitch = pitch_buf_fx;
    gain_preQ = 0;
    move16();
    set16_fx( code_preQ, 0, L_SUBFR );
    shift_wsp = add(Q_new,shift);

    /*----------------------------------------------------------------*
     * ACELP subframe loop
     *----------------------------------------------------------------*/

    FOR ( i_subfr=0; i_subfr<L_frame_fx; i_subfr+=L_SUBFR )
    {
        /*----------------------------------------------------------------*
         * Find the the excitation search target "xn" and innovation
         *   target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/

        Copy( &res_fx[i_subfr], &exc_fx[i_subfr], L_SUBFR );

        find_targets_fx( speech_fx, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq,
                         res_fx, L_SUBFR, p_Aw, st_fx->preemph_fac, xn, cn, h1 );

        Copy_Scale_sig(h1, h2_fx, L_SUBFR, -2);
        Scale_sig(h1, L_SUBFR, add(1, shift)); /* set h1[] in Q14 with scaling for convolution */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig(xn, L_SUBFR, shift);

        /*-----------------------------------------------------------------*
         * TC: subframe determination &
         * adaptive/glottal part of excitation construction
         *-----------------------------------------------------------------*/

        transition_enc_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, coder_type, i_subfr, &tc_subfr, &Jopt_flag, &position, voicing_fx, T_op_fx, &T0,
                           &T0_frac, &T0_min, &T0_max, exc_fx, y1, res_fx, h1, xn, xn2, st_fx->clip_var_fx, &gain_pit, g_corr, &clip_gain, &pt_pitch, bwe_exc_fx ,Q_new,shift);

        /*-----------------------------------------------------------------*
         * Transform domain contribution encoding - active frames
         *-----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            transf_cdbk_enc_fx( st_fx, st_fx->core_brate_fx, st_fx->extl_fx, coder_type, 0, i_subfr, tc_subfr, cn, exc_fx,
                                p_Aq, Aw_fx, h1, xn, xn2, y1, y2, Es_pred_fx, &gain_pit, gain_code, g_corr, clip_gain,
                                &(st_fx->mem_deemp_preQ_fx), &(st_fx->mem_preemp_preQ_fx), &gain_preQ, code_preQ, unbits,  Q_new, shift);
        }

        /*-----------------------------------------------------------------*
         * ACELP codebook search + pitch sharpening
         *-----------------------------------------------------------------*/

        inov_encode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, st_fx->last_L_frame_fx, coder_type, st_fx->bwidth_fx, sharpFlag,
                        i_subfr, tc_subfr, p_Aq, gain_pit, cn, exc_fx, h2_fx, mem->tilt_code, *pt_pitch, xn2, code, y2, &unbits_PI, shift );

        test();
        test();
        test();
        if( (sub(st_fx->L_frame_fx,L_FRAME16k) == 0) && (tc_subfr == 0) && (sub(i_subfr,L_SUBFR) == 0) && (sub(T0,2*L_SUBFR) == 0) )
        {
            Jopt_flag = 1;
            move16();
        }
        /*-----------------------------------------------------------------*
         * Quantize the gains
         * Test quantized gain of pitch for pitch clipping algorithm
         * Update tilt of code: 0.0 (unvoiced) to 0.5 (voiced)
         *-----------------------------------------------------------------*/
        IF( Jopt_flag == 0 )
        {
            /* SQ gain_code */
            gain_enc_tc_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, i_subfr, tc_subfr, xn, y2, code, Es_pred_fx,
                            &gain_pit, &gain_code, &gain_inov, &norm_gain_code, shift_wsp );
        }
        ELSE
        {
            IF ( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 )
            {
                /* SQ gain_pit and gain_code */
                gain_enc_SQ_fx( st_fx, st_fx->core_brate_fx, coder_type, i_subfr, tc_subfr, xn, y1, y2, code, Es_pred_fx,
                &gain_pit, &gain_code, &gain_inov, &norm_gain_code, g_corr, clip_gain, shift_wsp );
            }
            ELSE
            {
                /* VQ gain_pit and gain_code */
                gain_enc_mless_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, coder_type, i_subfr, tc_subfr, xn, y1, shift_wsp, y2, code, Es_pred_fx,
                &gain_pit, &gain_code, &gain_inov, &norm_gain_code, g_corr, clip_gain );
            }
        }
        gp_clip_test_gain_pit_fx( gain_pit, st_fx->clip_var_fx );

        Lgcode = L_shl(gain_code, Q_new);      /* scaled gain_code with Qnew -> Q16*/
        gcode16 = round_fx(Lgcode);
        mem->tilt_code = est_tilt_fx(&exc_fx[i_subfr], gain_pit, code, Lgcode, &voice_fac, shift);

        /*-----------------------------------------------------------------*
        * Update memory of the weighting filter
        *-----------------------------------------------------------------*/

        /*st->mem_w0 = xn[L_SUBFR-1] - (gain_pit*y1[L_SUBFR-1]) - (gain_code*y2[L_SUBFR-1]);*/
        L_tmp = L_mult(gcode16, y2[L_SUBFR - 1]);
        L_tmp = L_shl(L_tmp, add(5, shift));
        L_tmp = L_negate(L_tmp);
        L_tmp = L_mac(L_tmp, xn[L_SUBFR - 1], 16384);
        L_tmp = L_msu(L_tmp, y1[L_SUBFR - 1], gain_pit);
        L_tmp = L_shl(L_tmp, sub(1, shift));
        mem->mem_w0 = round_fx(L_tmp); /*Q_new-1*/

        /*-----------------------------------------------------------------*
         * Construct adaptive part of the excitation
         * Save the non-enhanced excitation for FEC_exc
         *-----------------------------------------------------------------*/

        /* Here, all these conditions have one purpose: to use   */
        /* the most efficient loop (the one with the least ops)  */
        /* This is done by upscaling gain_pit_fx and/or gain_code16 */
        /* when they don't use all 16 bits of precision          */

        /* exc Q_exc, gpit Q14, code Q12, gcode Q0 */
        IF (norm_s(gain_pit) == 0)
        {
            FOR (i = 0; i < L_SUBFR; i++)
            {
                exc2_fx[i+i_subfr] = round_fx(L_shl(L_mult(gain_pit,exc_fx[i+i_subfr]),1));
            }
        }
        ELSE
        {
            Gain_pitX2 = shl(gain_pit, 1);
            FOR (i = 0; i < L_SUBFR; i++)
            {
                exc2_fx[i+i_subfr] = mult_r(Gain_pitX2,exc_fx[i+i_subfr]);
            }
        }

        /*-----------------------------------------------------------------*
         * Construct adaptive part of the excitation
         * Save the non-enhanced excitation for FEC_exc
         *-----------------------------------------------------------------*/
        FOR (i = 0; i < L_SUBFR; i++)
        {
            /* code in Q9, gain_pit in Q14 */
            L_tmp = L_mult(gcode16, code[i]);
            L_tmp = L_shl(L_tmp, 5);
            L_tmp = L_mac(L_tmp, exc_fx[i + i_subfr], gain_pit);
            L_tmp = L_shl(L_tmp, 1); /* saturation can occur here */
            exc_fx[i + i_subfr] = round_fx(L_tmp);
        }

        /*-----------------------------------------------------------------*
         * Add the ACELP pre-quantizer contribution
         *-----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            tmp1_fx = add(16-(2+Q_AVQ_OUT_DEC+1),Q_new);
            FOR( i = 0; i < L_SUBFR; i++ )
            {
                L_tmp = L_mult(gain_preQ, code_preQ[i]);    /* Q2 + Q10 -> Q13*/
                L_tmp = L_shl(L_tmp,tmp1_fx);               /* Q16 + Q_exc    */
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

        prep_tbe_exc_fx( L_frame_fx, i_subfr, gain_pit, gain_code, code, voice_fac, &voice_factors[i_subfr/L_SUBFR],
                         bwe_exc_fx, gain_preQ, code_preQ, Q_new, T0, T0_frac, coder_type, st_fx->core_brate_fx );

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[].
         * Update A(z) filters
         *-----------------------------------------------------------------*/

        Syn_filt_s( 1, p_Aq, M, &exc_fx[i_subfr], &syn_fx[i_subfr], L_SUBFR, mem->mem_syn, 1 );

        p_Aw += (M+1);
        p_Aq += (M+1);
        pt_pitch++;
    }

    /* write reserved bits */
    WHILE( unbits_PI > 0 )
    {
        i = s_min(unbits_PI, 16);
        push_indice_fx( st_fx, IND_UNUSED, 0, i );
        unbits_PI -= i;
    }

    /* write TC configuration */
    IF( sub(L_frame_fx,L_FRAME) == 0 )
    {
        IF( sub(tc_subfr,TC_0_0) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }
        ELSE IF( sub(tc_subfr,TC_0_64) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }
        ELSE IF( sub(tc_subfr,TC_0_128) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
        }
        ELSE IF( sub(tc_subfr,TC_0_192) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }
        ELSE IF( sub(tc_subfr,L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }
        ELSE IF( sub(tc_subfr,2*L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }
        ELSE IF( sub(tc_subfr,3*L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
        }

    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        IF( tc_subfr == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 2 );
        }
        ELSE IF( sub(tc_subfr,L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 2 );
        }
        ELSE IF( sub(tc_subfr,2*L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 2, 2 );
        }
        ELSE IF( sub(tc_subfr,3*L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 3, 2 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 0, 1 );
        }
        ELSE IF( sub(tc_subfr,4*L_SUBFR) == 0 )
        {
            push_indice_fx( st_fx, IND_TC_SUBFR, 3, 2 );
            push_indice_fx( st_fx, IND_TC_SUBFR, 1, 1 );
        }

    }

    /* SC-VBR */
    st_fx->prev_ppp_gain_pit_fx = gain_pit;
    move16();
    st_fx->prev_tilt_code_fx = mem->tilt_code;
    move16();

    return tc_subfr;
}
