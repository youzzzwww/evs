/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"        /* required by wmc_tool */

/*-------------------------------------------------------------------*
 * encod_amr_wb()
 *
 * Encode excitation signal in AMR-WB IO mode
 *-------------------------------------------------------------------*/
void encod_amr_wb_fx(
    Encoder_State_fx *st,             /* i/o: state structure                         */
    LPD_state   *mem,             /* i/o: acelp memories                          */
    const Word16 speech[],          /* i  : input speech                            */
    const Word16 Aw[],              /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq[],              /* i  : 12k8 Lp coefficient                     */
    const Word16 pitch[3],          /* i  : open-loop pitch values for quantiz.     */
    const Word16 voicing[],         /* i  : voicing                                 */
    const Word16 *res,              /* i  : residual signal                         */
    Word16 *syn,              /* i/o: core synthesis                          */
    Word16 *exc,              /* i/o: current non-enhanced excitation         */
    Word16 *exc2,             /* i/o: current enhanced excitation             */
    Word16 *pitch_buf,        /* i/o: floating pitch values for each subframe */
    Word16 hf_gain_fx[NB_SUBFR], /* o  : decoded HF gain                      */
    const Word16 *speech16k_fx,
    Word16 shift,
    Word16 Q_new
)
{
    Word16 xn[L_SUBFR];                  /* Target vector for pitch search    */
    Word16 xn2[L_SUBFR];                 /* Target vector for codebook search */
    Word16 cn[L_SUBFR];                  /* Target vector in residual domain  */
    Word16 h1[L_SUBFR+(M+1)];            /* Impulse response vector           */
    Word16 h2[L_SUBFR+(M+1)];            /* Impulse response vector           */
    Word16 code[L_SUBFR];                /* Fixed codebook excitation         */
    Word16 y1[L_SUBFR];                  /* Filtered adaptive excitation      */
    Word16 y2[L_SUBFR];                  /* Filtered algebraic excitation     */
    Word16 gain_pit ;                    /* Pitch gain                        */
    Word16 voice_fac;                    /* Voicing factor                    */
    Word32 gain_code;                    /* Gain of code                      */
    Word16 gain_inov;                    /* inovation gain                    */
    Word16 i, i_subfr;                   /* tmp variables                     */
    Word16 T0, T0_frac;                  /* close loop integer pitch and fractional part */
    Word16 T0_min, T0_max;               /* pitch variables                   */
    Word16 *pt_pitch;                    /* pointer to floating pitch buffer  */
    Word16 g_corr[10];                   /* ACELP correl, values + gain pitch */
    Word16 clip_gain;                    /* LSF clip gain                     */
    const Word16 *p_Aw, *p_Aq;           /* pointer to LP filter coeff. vector*/
    Word16 dummy_buf[20];                /* dummy buffer - no usage           */
    Word32 norm_gain_code;
    Word16 pitch_limit_flag;
    Word16 shift_wsp;
    Word32 L_tmp;
    Word16 gcode16;
    Word32 Ltmp;
    Word32 Lgcode;
    Word16 T_op[3];                      /* pitch period for quantization     */
    Word16 lp_select, lp_flag;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    set16_fx( y2, 0, L_SUBFR );
    set16_fx( code, 0, L_SUBFR );

    pitch_limit_flag = 0;
    move16(); /* always restrained pitch Q range in IO mode */
    T0_max = PIT_MAX;
    move16();
    T0_min = PIT_MIN;
    move16();

    p_Aw = Aw;
    p_Aq = Aq;
    pt_pitch = pitch_buf;
    shift_wsp = add(Q_new,shift);

    Copy( pitch, T_op, 2 );
    if (sub(T_op[0],PIT_MIN) <= 0)
    {
        T_op[0] = shl(T_op[0],1);
        move16();
    }

    if (sub(T_op[1],PIT_MIN) <= 0)
    {
        /*T_op[1] *= 2;*/
        T_op[1] = shl(T_op[1],1);
        move16();
    }

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/
    FOR ( i_subfr = 0; i_subfr < L_FRAME; i_subfr += L_SUBFR )
    {
        /*----------------------------------------------------------------*
         * Bandwidth expansion of A(z) filter coefficients
         * Find the the excitation search target "xn" and innovation
         *   target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/
        Copy( &res[i_subfr], &exc[i_subfr], L_SUBFR );

        find_targets_fx( speech, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq,
                         res, L_SUBFR, p_Aw, TILT_FAC_FX, xn, cn, h1 );

        Copy_Scale_sig(h1, h2, L_SUBFR, -2);
        Scale_sig(h1, L_SUBFR, add(1, shift)); /* set h1[] in Q14 with scaling for convolution */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig(xn, L_SUBFR, shift);

        /*----------------------------------------------------------------*
         * Close-loop pitch search and quantization
         * Adaptive exc. construction
         *----------------------------------------------------------------*/

        *pt_pitch = pit_encode_fx(st, st->core_brate_fx, 1, L_FRAME, -1, &pitch_limit_flag, i_subfr, exc,
                                  L_SUBFR, T_op, &T0_min, &T0_max, &T0, &T0_frac, h1, xn );

        /*-----------------------------------------------------------------*
         * Find adaptive exitation
         *-----------------------------------------------------------------*/

        pred_lt4(&exc[i_subfr], &exc[i_subfr], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

        /*-----------------------------------------------------------------*
         * Gain clipping test to avoid unstable synthesis on frame erasure
         *   or in case of floating point encoder & fixed p. decoder
         *-----------------------------------------------------------------*/

        clip_gain = gp_clip_fx( voicing, i_subfr, 0, xn, st->clip_var_fx, sub(shift_wsp,1));

        /*-----------------------------------------------------------------*
         * LP filtering of the adaptive excitation, codebook target computation
         *-----------------------------------------------------------------*/

        lp_select = lp_filt_exc_enc_fx( MODE1, st->core_brate_fx, 1, -1, i_subfr, exc, h1,
                                        xn, y1, xn2, L_SUBFR, L_FRAME, g_corr, clip_gain, &gain_pit, &lp_flag );

        IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
        {
            push_indice_fx( st, IND_LP_FILT_SELECT, lp_select, 1 );
        }

        /*-----------------------------------------------------------------*
         * Innovation encoding
         *-----------------------------------------------------------------*/
        inov_encode_fx( st, st->core_brate_fx, 1, L_FRAME, st->last_L_frame_fx, -1, -1, 0, i_subfr, -1, p_Aq, gain_pit, cn,
                        exc, h2, mem->tilt_code, *pt_pitch, xn2, code, y2, dummy_buf, shift );
        /*-----------------------------------------------------------------*
         * Gain encoding
         * Pitch gain clipping test
         * Estimate spectrum tilt and voicing
         *-----------------------------------------------------------------*/
        gain_enc_amr_wb_fx( st, xn, shift_wsp, y1, y2, code, st->core_brate_fx, &gain_pit, &gain_code,
                            &gain_inov, &norm_gain_code, g_corr, clip_gain, st->past_qua_en_fx );

        gp_clip_test_gain_pit_fx( gain_pit, st->clip_var_fx );
        Lgcode = L_shl(gain_code, Q_new);      /* scaled gain_code with Qnew -> Q16*/
        gcode16 = round_fx(Lgcode);

        mem->tilt_code = Est_tilt2( exc+i_subfr, gain_pit, code, Lgcode, &voice_fac, shift );

        FOR (i = 0; i < L_SUBFR; i++)
        {
            exc2[i+i_subfr] = round_fx(L_shl(L_mult(gain_pit,exc[i+i_subfr]),1));
        }

        /*-----------------------------------------------------------------*
         * Update memory of the weighting filter
         *-----------------------------------------------------------------*/

        /*st->mem_w0 = xn[L_SUBFR-1] - gain_pit * y1[L_SUBFR-1] - gain_code * y2[L_SUBFR-1];*/
        Ltmp = L_mult(gcode16, y2[L_SUBFR - 1]);
        Ltmp = L_shl(Ltmp, add(5, shift));
        Ltmp = L_negate(Ltmp);
        Ltmp = L_mac(Ltmp, xn[L_SUBFR - 1], 16384);
        Ltmp = L_msu(Ltmp, y1[L_SUBFR - 1], gain_pit);
        Ltmp = L_shl(Ltmp, sub(1, shift));
        mem->mem_w0 = round_fx(Ltmp);     /*Q_new-1        */

        /*-----------------------------------------------------------------*
         * Find the total excitation
         *-----------------------------------------------------------------*/
        FOR (i = 0; i < L_SUBFR; i++)
        {
            L_tmp = L_mult(gcode16, code[i]);
            L_tmp = L_shl(L_tmp, 5);
            L_tmp = L_mac(L_tmp, exc[i + i_subfr], gain_pit);
            L_tmp = L_shl(L_tmp, 1); /* saturation can occur here */
            exc[i + i_subfr] = round_fx(L_tmp);
        }

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[]
         * Update A(z) filters
         *-----------------------------------------------------------------*/
        Syn_filt_s(1, p_Aq, M, &exc[i_subfr], &syn[i_subfr], L_SUBFR, mem->mem_syn, 1 );

        /*-----------------------------------------------------------------*
         * HF gain modification factors at 23.85 kbps
         *-----------------------------------------------------------------*/

        IF ( L_sub(st->core_brate_fx,ACELP_23k85) == 0 )
        {
            hf_cod_fx( st->core_brate_fx, &speech16k_fx[i_subfr * L_SUBFR16k/L_SUBFR], Aq, &exc[i_subfr], &syn[i_subfr],
                       &st->seed2_enc_fx, st->mem_hp400_enc_fx, st->mem_syn_hf_enc_fx,
                       st->mem_hf_enc_fx, st->mem_hf2_enc_fx, st->hangover_cnt_fx, &st->gain_alpha_fx, &hf_gain_fx[i_subfr/L_SUBFR],
                       add(Q_new,1),
                       st->Q_syn);

            push_indice_fx(st, IND_HF_GAIN_MODIFICATION, hf_gain_fx[i_subfr/L_SUBFR], 4 );
        }

        p_Aw += (M+1);
        p_Aq += (M+1);
        pt_pitch++;
    }

    return;
}
