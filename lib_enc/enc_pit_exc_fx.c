/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */


#include "stl.h"

/*======================================================================*/
/* FUNCTION : enc_pit_exc_fx()										    */
/*----------------------------------------------------------------------*/
/* PURPOSE : Encode pitch only contribution								*/
/*																		*/
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    */
/* _ (Struct)	st_fx			: encoder static memory					*/
/* _ (Word16[]) speech_fx			: input speech		        Qnew-1	*/
/* _ (Word16[]) Aq_fx			: LP filter coefficient		    Q12		*/
/* _ (Word16[]) A_fx			: unquantized A(z) filter               */
/*                                with bandwidth expansion 		Q12	    */
/* _ (Word16) coder_type_fx		: coding type							*/
/* _ (Word16) Es_pred_fx		: predicted scaled innov. energy Q8		*/
/* _ (Word16[]) T_op_fx			: open loop pitch Q0					*/
/* _ (Word16[]) voicing_fx	    : floating pitch values for each subframe Q15*/
/* _ (Word16[]) res_fx			: residual signal				Q_new	 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Qnew)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Qnew)	 */
/* _ (Word16[]) pitch_buf_fx	: floating pitch values for each subframe Q6*/
/* _ (Word16[]) *wH1,           : Weighted impulses response mask        */
/* _ (Word16) shift		        : 	shift								 */
/* _ (Word16) Q_new		        :	                                     */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Qnew)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Qnew)	 */
/* _ (Word16[]) syn_fx			:core synthesis					   	     */
/* _ (Word16[])	voice_factors_fx: voicing factors 				Q15		 */
/* _ (Word16[]) bwe_exc_fx		: excitation for SWB TBE        Q0		 */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/
void enc_pit_exc_fx(
    Encoder_State_fx *st_fx,              /* i/o: State structure                                  */
    LPD_state   *mem,                     /* i/o: acelp memories                                   */
    const Word16 *speech,               /* i  : Input speech                                     */
    const Word16 Aw[],                  /* i  : weighted A(z) unquantized for subframes          */
    const Word16 Aq[],                  /* i  : 12k8 Lp coefficient                              */
    const Word16 Es_pred,               /* i  : predicted scaled innov. energy                   */
    const Word16 *T_op,                 /* i  : open loop pitch                                  */
    const Word16 *voicing,              /* i  : voicing                                          */
    const Word16 *res,                  /* i  : residual signal                                  */
    Word16 *synth,                /* i/o: core synthesis                                   */
    Word16 *exc,                  /* i/o: current non-enhanced excitation                  */
    Word16 *T0,                   /* i/o: close loop integer pitch                         */
    Word16 *T0_frac,              /* i/o: close-loop pitch period - fractional part        */
    Word16 *pitch_buf,            /* i/o: Fractionnal per subframe pitch                   */
    const Word16 nb_subfr,              /* i  : Number of subframe considered                    */
    Word16 *gpit,                 /* o  : pitch mean gpit                                  */
    Word16 *saved_bit_pos,        /* o  : saved position in the bitstream before pitch contribution */
    Word16 Q_new,
    Word16 shift
)
{
    Word16 xn[PIT_EXC_L_SUBFR];          /* Target vector for pitch search    */
    Word16 xn2[PIT_EXC_L_SUBFR];         /* Target vector for codebook search */
    Word16 h1[PIT_EXC_L_SUBFR+(M+1)];    /* Impulse response vector           */
    Word16 y1[PIT_EXC_L_SUBFR];          /* Filtered adaptive excitation      */
    Word16 code[L_SUBFR];                /* Fixed codebook excitation         */
    Word16 y2[L_SUBFR];                  /* Filtered algebraic excitation     */
    Word16 voice_fac;                    /* Voicing factor                    */
    Word32 gain_code;                    /* Gain of code                      */
    Word16 gain_inov;                    /* inovation gain                    */
    Word16 gain_pit;                     /* Pitch gain                        */
    Word16 pit_idx, i_subfr;             /* tmp variables                     */
    Word16 T0_min, T0_max;               /* pitch variables                   */
    Word16 g_corr[10];                   /* ACELP correlation values + gain pitch */
    Word16 clip_gain, i;                 /* LSF clip gain and LP flag         */
    const Word16 *p_Aw, *p_Aq;           /* pointer to LP filter coefficient vector */
    Word16 cn1[L_SUBFR], *cn;            /* (Used only when L_subfr == L_SUBFR) Target vector in residual domain  */
    Word16 *pt_pitch;                    /* pointer to floating pitch         */
    Word16 L_subfr;
    Word16 cum_gpit, gpit_tmp;
    Word16 Local_BR, Pitch_BR, Pitch_CT;
    Word16 unbits_PI = 0;                /* saved bits for PI                 */
    Word32 norm_gain_code;
    Word16 pitch_limit_flag;
    Word16 h2[PIT_EXC_L_SUBFR+(M+1)];    /* Impulse response vector           */
    Word32 Ltmp;
    Word32 Lgcode;
    Word16 gcode16;
    Word16 shift_wsp;
    Word16 lp_select, lp_flag;
    /*------------------------------------------------------------------*
    * Initialization
    *------------------------------------------------------------------*/

    pitch_limit_flag = 1;
    move16(); /* always extended pitch Q range */

    IF( st_fx->GSC_noisy_speech_fx )
    {
        Local_BR = ACELP_7k20;
        move16();
        Pitch_CT = GENERIC;
        move16();
        Pitch_BR = ACELP_7k20;
        move16();
    }
    ELSE
    {

        Local_BR = ACELP_7k20;
        move16();
        Pitch_CT = AUDIO;
        move16();
        Pitch_BR = extract_l(st_fx->core_brate_fx);
    }
    gain_code = 0;
    move16();
    T0_max = PIT_MAX;
    move16();
    T0_min = PIT_MIN;
    move16();

    cum_gpit = 0;
    move16();

    L_subfr = mult_r(L_FRAME,div_s(1,nb_subfr));

    *saved_bit_pos = st_fx->next_bit_pos_fx;
    move16();

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/
    cn = NULL;
    if(sub(L_subfr, L_SUBFR)==0)
    {
        cn = cn1;
        move16();
    }
    p_Aw = Aw;

    p_Aq = Aq;
    pt_pitch = pitch_buf;       /* pointer to the pitch buffer */
    shift_wsp = add(Q_new,shift);
    FOR ( i_subfr = 0; i_subfr < L_FRAME; i_subfr += L_subfr )
    {

        /*----------------------------------------------------------------*
         * Bandwidth expansion of A(z) filter coefficients
         * Find the the excitation search target "xn" and innovation
         *   target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/
        Copy( &res[i_subfr], &exc[i_subfr], L_subfr );
        /* condition on target (compared to float) has been put outside the loop */
        find_targets_fx( speech, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq,
                         res, L_subfr, p_Aw, st_fx->preemph_fac, xn, cn,h1);
        Copy_Scale_sig(h1, h2, L_subfr, -2);
        Scale_sig(h1, L_subfr, add(1, shift)); /* set h1[] in Q14 with scaling for convolution */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig(xn, L_subfr, shift);

        /*----------------------------------------------------------------*
         * Close-loop pitch search and quantization
         * Adaptive exc. construction
         *----------------------------------------------------------------*/
        *pt_pitch = pit_encode_fx( st_fx, Pitch_BR, 0, L_FRAME, Pitch_CT, &pitch_limit_flag, i_subfr, exc,
                                   L_subfr, T_op, &T0_min, &T0_max, T0, T0_frac, h1, xn );
        /*-----------------------------------------------------------------*
         * Find adaptive exitation
         *-----------------------------------------------------------------*/

        pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_subfr+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
        /*-----------------------------------------------------------------*
         * Gain clipping test to avoid unstable synthesis on frame erasure
         * or in case of floating point encoder & fixed p. decoder
         *-----------------------------------------------------------------*/

        clip_gain = gp_clip_fx(voicing, i_subfr, AUDIO, xn, st_fx->clip_var_fx,sub(shift_wsp, 1));

        /*-----------------------------------------------------------------*
         * Codebook target computation
         * (No LP filtering of the adaptive excitation)
         *-----------------------------------------------------------------*/

        lp_select = lp_filt_exc_enc_fx( MODE1, st_fx->core_brate_fx, 0, AUDIO, i_subfr, exc, h1,
                                        xn, y1, xn2, L_subfr, L_FRAME, g_corr, clip_gain, &gain_pit, &lp_flag );

        IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
        {
            push_indice_fx( st_fx, IND_LP_FILT_SELECT, lp_select, 1 );
        }

        /*st_fx->lowrate_pitchGain = 0.9f * st_fx->lowrate_pitchGain + 0.1f * gain_pit;*/
        st_fx->lowrate_pitchGain = round_fx(L_mac(L_mult(29491, st_fx->lowrate_pitchGain), 6554, gain_pit)); /*Q14*Q16(0.1) + Q15 -> Q15*/


        gpit_tmp =  gain_pit;
        move16(); /*Q14*/
        test();
        IF( st_fx->GSC_noisy_speech_fx == 0 || sub(L_subfr,L_SUBFR ) != 0 )
        {
            pit_idx = vquant_fx( &gain_pit, mean_gp_fx, &gain_pit, dic_gp_fx, 1, 16 );
            push_indice_fx( st_fx, IND_PIT_IDX, pit_idx, 4 );
        }
        ELSE
        {
            /*-----------------------------------------------------------------*
             * Innovation & gain encoding
             *-----------------------------------------------------------------*/
            /* h2 in Q12 for codebook search */
            /* h1 has been scaled with 1 + shift so we need to remove 2 and (1+shift) = -3 - shift*/
            Copy_Scale_sig( h1, h2, L_subfr, sub(-2-1,shift) );

            inov_encode_fx( st_fx, Local_BR, 0, L_FRAME, st_fx->last_L_frame_fx, LOCAL_CT, WB, 1, i_subfr, -1, p_Aq,
            gain_pit, cn, exc, h2, mem->tilt_code, *pt_pitch, xn2, code, y2, &unbits_PI,shift
                          );
            /*-----------------------------------------------------------------*
             * Gain encoding
             *-----------------------------------------------------------------*/
            gain_enc_mless_fx( st_fx, Local_BR, L_FRAME, LOCAL_CT, i_subfr, -1, xn, y1, shift_wsp, y2, code, Es_pred,
            &gain_pit, &gain_code, &gain_inov, &norm_gain_code, g_corr, clip_gain );
        }

        gp_clip_test_gain_pit_fx( gain_pit, st_fx->clip_var_fx );

        Lgcode = L_shl(gain_code, Q_new);      /* scaled gain_code with Qnew -> Q16*/
        gcode16 = round_fx(Lgcode);
        IF( st_fx->GSC_noisy_speech_fx)
        {
            mem->tilt_code = Est_tilt2(&exc[i_subfr], gain_pit, code, Lgcode, &voice_fac, shift);
            move16();
        }
        ELSE
        {
            mem->tilt_code  = 0;
            move16();
        }
        /*-----------------------------------------------------------------*
         * Update memory of the weighting filter
         *-----------------------------------------------------------------*/
        IF( st_fx->GSC_noisy_speech_fx)
        {
            Ltmp = L_mult(gcode16, y2[L_subfr - 1]);
            Ltmp = L_shl(Ltmp, add(5, shift));
            Ltmp = L_negate(Ltmp);
            Ltmp = L_mac(Ltmp, xn[L_subfr - 1], 16384);
            Ltmp = L_msu(Ltmp, y1[L_subfr - 1], gain_pit);
            Ltmp = L_shl(Ltmp, sub(1, shift));
            mem->mem_w0 = round_fx(Ltmp);      /*Q_new-1         */
        }
        ELSE
        {
            Ltmp = L_mult(xn[L_subfr - 1], 16384);
            Ltmp = L_msu(Ltmp, y1[L_subfr - 1], gain_pit);
            Ltmp = L_shl(Ltmp, sub(1, shift));
            mem->mem_w0 = round_fx(Ltmp);     /*Q_new-1         */
        }

        /*-----------------------------------------------------------------*
         * Construct adaptive part of the excitation
         * Save the non-enhanced excitation for FEC_exc
         *-----------------------------------------------------------------*/
        IF( st_fx->GSC_noisy_speech_fx)
        {
            FOR ( i = 0; i < L_subfr; i++ )
            {
                /* code in Q9, gain_pit in Q14 */
                Ltmp = L_mult(gcode16, code[i]);
                Ltmp = L_shl(Ltmp, 5);
                Ltmp = L_mac(Ltmp, exc[i + i_subfr], gain_pit);
                Ltmp = L_shl(Ltmp, 1); /* saturation can occur here */
                exc[i + i_subfr] = round_fx(Ltmp);
            }
        }
        ELSE
        {
            FOR ( i = 0; i < L_subfr; i++ )
            {

                Ltmp = L_mult(exc[i + i_subfr], gain_pit);
                Ltmp = L_shl(Ltmp, 1); /* saturation can occur here */
                exc[i + i_subfr] = round_fx(Ltmp);
            }
        }

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[].
         * Update A(z) filters
         *-----------------------------------------------------------------*/

        Syn_filt_s( 1, p_Aq, M, &exc[i_subfr], &synth[i_subfr], L_subfr, st_fx->mem_syn_tmp_fx, 1 );
        IF( sub(L_subfr,2*L_SUBFR) == 0 )
        {
            IF( i_subfr == 0 )
            {
                cum_gpit = mult_r(gpit_tmp,16384);
            }
            ELSE
            {
                cum_gpit = add(cum_gpit,mult_r(gpit_tmp,16384));
            }
            p_Aw += 2*(M+1);
            move16();
            p_Aq += 2*(M+1);
            move16();
            pt_pitch++;
            *pt_pitch = *(pt_pitch-1);
            move16();
            pt_pitch++;
        }
        ELSE IF(sub(L_subfr,4*L_SUBFR) == 0 )
        {
            cum_gpit = gpit_tmp;
            move16();

            pt_pitch++;
            *pt_pitch = *(pt_pitch-1);
            move16();
            pt_pitch++;
            *pt_pitch = *(pt_pitch-1);
            move16();
            pt_pitch++;
            *pt_pitch = *(pt_pitch-1);
            pt_pitch++;
            p_Aw += 4*(M+1);
            p_Aq += 4*(M+1);
        }
        ELSE
        {
            IF( i_subfr == 0 )
            {

                cum_gpit = mult_r(gpit_tmp,8192);
            }
            ELSE
            {
                cum_gpit = add(cum_gpit,mult_r(gpit_tmp,8192));
            }

            pt_pitch++;
            p_Aw += (M+1);
            p_Aq += (M+1);
        }
    }

    cum_gpit = shl(cum_gpit,1); /*Q15*/
    *gpit = round_fx(L_mac(L_mult(3277 , *gpit), 29491, cum_gpit)); /*Q15*/


}
