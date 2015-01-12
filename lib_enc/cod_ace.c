/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "stl.h"

#include "rom_basop_util.h"

/*-------------------------------------------------------------------*
 * coder_acelp()
 *
 * Encode ACELP frame
 *-------------------------------------------------------------------*/

Word16 coder_acelp(             /* output SEGSNR for CL decision   */
    ACELP_config *acelp_cfg,      /*input/output: configuration of the ACELP coding*/
    const Word16 coder_type,      /* input: coding type              */
    const Word16 A[],             /* input: coefficients 4xAz[M+1]   */
    const Word16 Aq[],            /* input: coefficients 4xAz_q[M+1] */
    Word16 speech[],              /* input: speech[-M..lg]           */
    Word16 synth[],
    LPD_state *LPDmem,
    const Word16 voicing[],       /* input: open-loop LTP gain       */
    const Word16 T_op[],          /* input: open-loop LTP lag        */
    Word16 *prm,                  /* output: acelp parameters        */
    Word16 stab_fac,
    Encoder_State_fx *st,
    HANDLE_PLC_ENC_EVS hPlc_Ext,
    Word16 target_bits,           /* i/o : coder memory state        */
    Word16 Q_new,
    Word16 shift
    ,Word16 *pitch_buf            /* output : pitch values for each subfr.*/
    ,Word16 *voice_factors        /* output : voicing factors             */
    ,Word16 *bwe_exc              /* output : excitation for SWB TBE      */
)
{
    Word16 i, j, i_subfr, j_subfr;
    Word16 T0, T0_min, T0_min_frac, T0_max, T0_max_frac, T0_res;
    Word16 T0_frac;
    Word16 tmp, tmp2, Es_pred;
    Word16 gain_pit, voice_fac;
    Word32 gain_code, Ltmp, Ltmp2;
    ACELP_CbkCorr g_corr;
    const Word16 *p_A, *p_Aq;
    Word16 h1[L_SUBFR];   /* weighted impulse response of LP */
    Word16 code[L_SUBFR];
    Word16 xn_exp;
    Word16 Q_xn;
    Word16 Q_new_p5;
    Word16 cn[L_SUBFR];
    Word16 xn[L_SUBFR];
    Word16 y1[L_SUBFR];        /* Filtered adaptive excitation       */
    Word16 y2[L_SUBFR];        /* Filtered adaptive excitation       */
    Word16 res_save;
    Word16 exc_buf[L_EXC_MEM+L_DIV_MAX+1], *exc;
    Word16 exc2[L_SUBFR];
    Word16 *syn,syn_buf[M+L_DIV_MAX+L_DIV_MAX/2];  /*128 for the memory, L_DIV for the current synth and 128 for the ZIR for next TCX*/
    Word16 syn2[L_DIV_MAX];
    Word16 gain_inov;
    Word32 past_gcode;
    Word16 L_frame;
    Word16 clip_gain;
    Word32 gain_code2;
    Word16 code2[L_SUBFR];
    Word16 y22[L_SUBFR]; /* Filtered adaptive excitation */
    Word32 gain_code_vect[2];
    Word16 error = 0;
    Word16 gain_preQ = 0;                /* Gain of prequantizer excitation   */
    Word16 code_preQ[L_SUBFR];           /* Prequantizer excitation           */

    Word16 dummy = 0;
    set16_fx(code_preQ, 0, L_SUBFR);


    T0 = 0;           /* to avoid compilation warnings */
    T0_frac = 0;      /* to avoid compilation warnings */
    T0_res = 0;       /* to avoid compilation warnings */
    gain_pit = 0;     /* to avoid compilation warnings */


    /* Configure ACELP */
    LPDmem->nbits = BITS_ALLOC_config_acelp( target_bits, coder_type, &(st->acelp_cfg), st->narrowBand, st->nb_subfr );

    /* Init Framing parameters */
    move16();
    move16();
    move16();
    L_frame = st->L_frame_fx;


    /*------------------------------------------------------------------------*
     * Previous frame is TCX (for non-EVS modes)(deactivated permanently)     *
     *------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------*
     * Initialize buffers                                                     *
     *------------------------------------------------------------------------*/

    /* Rescale ACELP memories, which were not scaled yet*/
    xn_exp = sub(sub(15+1, Q_new),shift);
    Q_xn = add(sub(Q_new,1),shift);
    Q_new_p5 = add(Q_new, 5);

    /* Reset phase dispersion */
    IF (st->last_core_fx > ACELP_CORE)
    {
        move16();
        move16();
        move16();
        st->dm_fx.prev_gain_code = 0;
        set16_fx(st->dm_fx.prev_gain_pit, 0, 6);
        st->dm_fx.prev_state = 0;
    }

    /* set excitation memory*/
    move16();
    move16();
    exc = exc_buf+L_EXC_MEM;
    Copy(LPDmem->old_exc, exc_buf, L_EXC_MEM);
    *(exc+st->L_frame_fx) = 0;

    /* Init syn buffer */
    move16();
    syn = syn_buf + M;
    Copy(LPDmem->mem_syn, syn_buf, M);


    /* calculate residual */
    move16();
    p_Aq = Aq;
    FOR (i_subfr=0; i_subfr<L_frame; i_subfr+=L_SUBFR)
    {

        Residu3_fx( p_Aq, &speech[i_subfr], &exc[i_subfr], L_SUBFR, 1 );
        p_Aq += (M+1);

    }
    /*------------------------------------------------------------------------*
     * Find and quantize mean_ener_code for gain quantizer                    *
     *------------------------------------------------------------------------*/

    IF (acelp_cfg->nrg_mode>0)
    {

        Es_pred_enc_fx(&Es_pred, prm, L_frame, exc, voicing, acelp_cfg->nrg_bits, acelp_cfg->nrg_mode>1, Q_new
                      );
        prm++;
    }
    ELSE
    {

        Es_pred=0;
    }

    IF (sub(st->L_frame_fx,L_FRAME) == 0)
    {
        Copy(Aq+2*(M+1), st->cur_sub_Aq_fx, (M+1));
    }
    ELSE
    {
        Copy(Aq+3*(M+1), st->cur_sub_Aq_fx, (M+1));
    }


    /*------------------------------------------------------------------------*
     *          Loop for every subframe in the analysis frame                 *
     *------------------------------------------------------------------------*
     *  To find the pitch and innovation parameters. The subframe size is     *
     *  L_SUBFR and the loop is repeated L_FRAME_PLUS/L_SUBFR   *
     *  times.                                                                *
     *     - compute impulse response of weighted synthesis filter (h1[])     *
     *     - compute the target signal for pitch search                       *
     *     - find the closed-loop pitch parameters                            *
     *     - encode the pitch delay                                           *
     *     - update the impulse response h1[] by including fixed-gain pitch   *
     *     - find target vector for codebook search                           *
     *     - correlation between target vector and impulse response           *
     *     - codebook search                                                  *
     *     - encode codebook address                                          *
     *     - VQ of pitch and codebook gains                                   *
     *     - find synthesis speech                                            *
     *     - update states of weighting filter                                *
     *------------------------------------------------------------------------*/
    move16();
    move16();
    p_A = A;
    p_Aq = Aq;

    move16();
    res_save = exc[0];

    j_subfr = 0;
    move16();
    FOR (i_subfr=0; i_subfr<L_frame; i_subfr+=L_SUBFR)
    {

        /* Restore exc[i_subfr] and save next exc[L_SUBFR+i_subfr] */
        move16();
        move16();
        exc[i_subfr] = res_save;
        res_save = exc[L_SUBFR+i_subfr];

        /*--------------------------------------------------------------------------*
         * Find target for pitch search (xn[]), target for innovation search (cn[]) *
         * and impulse response of the weighted synthesis filter (h1[]).            *
         *--------------------------------------------------------------------------*/
        find_targets_fx(
            speech,
            &syn[i_subfr-M],
            i_subfr,
            &LPDmem->mem_w0,
            p_Aq,
            exc,
            L_SUBFR,
            p_A,
            st->preemph_fac,
            xn,
            cn
            ,h1
        );

        /*---------------------------------------------------------------*
         * Compute impulse response, h1[], of weighted synthesis filter  *
         *---------------------------------------------------------------*/
        Scale_sig(h1, L_SUBFR, add(1,shift)); /* Q13+1-shift */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig(xn, L_SUBFR, shift);

        /*-----------------------------------------------------------------*
         * Gain clipping test to avoid unstable synthesis on frame erasure
         * or in case of floating point encoder & fixed p. decoder
         *-----------------------------------------------------------------*/

        clip_gain = Mode2_gp_clip( voicing, i_subfr, coder_type, xn, st->clip_var_fx, L_SUBFR, Q_xn );

        /*-----------------------------------------------------------------*
         * - find unity gain pitch excitation (adaptive codebook entry)    *
         *   with fractional interpolation.                                *
         * - find filtered pitch exc. y1[]=exc[] convolved with h1[])      *
         * - compute pitch gain1                                           *
         *-----------------------------------------------------------------*/

        IF ( acelp_cfg->ltp_bits!=0 )
        {
            /* Adaptive Codebook (GC and VC) */

            Mode2_pit_encode( acelp_cfg->ltp_mode,
                              i_subfr,
                              &prm,
                              &exc[i_subfr],
                              T_op,
                              &T0_min,
                              &T0_min_frac,
                              &T0_max,
                              &T0_max_frac,
                              &T0,
                              &T0_frac,
                              &T0_res,
                              h1,
                              xn,
                              st->pit_min,
                              st->pit_fr1,
                              st->pit_fr1b,
                              st->pit_fr2,
                              st->pit_max,
                              st->pit_res_max);

            E_ACELP_adaptive_codebook( exc,
                                       T0,
                                       T0_frac,
                                       T0_res,
                                       st->pit_res_max,
                                       acelp_cfg->ltf_mode,
                                       i_subfr,
                                       L_SUBFR,
                                       L_frame,
                                       h1,
                                       clip_gain,
                                       xn,
                                       y1,
                                       &g_corr,
                                       &prm,
                                       &gain_pit,
                                       xn_exp
                                       ,0
                                       ,0
                                       ,&dummy
                                     );



        }
        ELSE IF ( acelp_cfg->ltp_bits==0 )
        {
            /* No adaptive codebook (UC) */
            gain_pit=0;
            g_corr.xy1=0;
            g_corr.xy1_e=0;
            g_corr.y1y1=0;
            g_corr.y1y1_e=0;
            set16_fx(y1,0,L_SUBFR);
            set16_fx(exc+i_subfr,0,L_SUBFR);
            T0 = L_SUBFR;
            T0_frac = 0;
            T0_res = 1;
        }

        IF( st->igf != 0 )
        {
            tbe_celp_exc(L_frame, i_subfr, T0, T0_frac, &error, bwe_exc);
        }

        pitch_buf[i_subfr/L_SUBFR] = shl(add(shl(T0,2),T0_frac), 4);

        /*----------------------------------------------------------------------*
         *                 Encode the algebraic innovation                      *
         *----------------------------------------------------------------------*/

        E_ACELP_innovative_codebook(  exc,
                                      T0,
                                      T0_frac,
                                      T0_res,
                                      gain_pit,
                                      LPDmem->tilt_code,
                                      acelp_cfg->fixed_cdk_index[j_subfr],
                                      acelp_cfg->formant_enh,
                                      acelp_cfg->formant_tilt,
                                      acelp_cfg->formant_enh_num,
                                      acelp_cfg->formant_enh_den,
                                      acelp_cfg->pitch_sharpening,
                                      acelp_cfg->pre_emphasis,
                                      acelp_cfg->phase_scrambling,
                                      i_subfr,
                                      p_Aq,
                                      h1,
                                      xn,
                                      cn,
                                      y1,
                                      y2,
                                      st->acelp_autocorr,
                                      &prm,
                                      code,
                                      shift);

        E_ACELP_xy2_corr(xn, y1, y2, &g_corr, L_SUBFR, Q_xn);

        g_corr.y2y2_e = sub(g_corr.y2y2_e, 18);  /* -18 (y2*y2: Q9*Q9) */
        g_corr.xy2_e  = sub(g_corr.xy2_e,  add(Q_xn,9));  /* -(Q_xn+9) (xn: Q_xn y2: Q9) */
        g_corr.y1y2_e = sub(g_corr.y1y2_e, add(Q_xn,9));  /* -(Q_xn+9) (y1: Q_xn y2: Q9) */
        g_corr.xx_e   = sub(g_corr.xx_e,   add(Q_xn,Q_xn));  /* -(Q_xn+Q_xn) (xn: Q_xn) */


        /*----------------------------------------------------------------------*
         *                 Add Gaussian excitation                              *
         *----------------------------------------------------------------------*/

        IF (sub(acelp_cfg->gains_mode[j_subfr], 7) == 0)
        {

            gauss_L2(h1,
                     code2,
                     y2,
                     y22,
                     &gain_code2,
                     &g_corr,
                     gain_pit,
                     LPDmem->tilt_code,
                     p_Aq,
                     acelp_cfg->formant_enh_num,
                     &(st->seed_acelp),
                     shift
                    );
        }
        ELSE
        {
            gain_code2 = L_deposit_l(0);
            set16_fx(code2, 0, L_SUBFR);
            set16_fx(y22, 0, L_SUBFR);
        }

        /*----------------------------------------------------------*
         *  - Compute the fixed codebook gain                       *
         *  - quantize fixed codebook gain                          *
         *----------------------------------------------------------*/

        encode_acelp_gains( code,
                            acelp_cfg->gains_mode[j_subfr],
                            Es_pred,
                            clip_gain,
                            &g_corr,
                            &gain_pit,
                            &gain_code,
                            &prm,
                            &past_gcode,
                            &gain_inov,
                            L_SUBFR,
                            code2,
                            &gain_code2,
                            st->flag_noisy_speech_snr
                          );

        gp_clip_test_gain_pit_fx( gain_pit, st->clip_var_fx );

        gain_code_vect[0] = gain_code;
        move32();
        gain_code_vect[1] = gain_code;
        move32();

        /*----------------------------------------------------------*
         * - voice factor (for pitch enhancement)                   *
         *----------------------------------------------------------*/
        E_UTIL_voice_factor( exc,
                             i_subfr,
                             code,
                             gain_pit,
                             gain_code,
                             &voice_fac,
                             &(LPDmem->tilt_code),
                             L_SUBFR,
                             acelp_cfg->voice_tilt,
                             Q_new,
                             shift
                           );

        st->rf_tilt_buf[i_subfr/L_SUBFR] = LPDmem->tilt_code;

        /*-----------------------------------------------------------------*
         * Update memory of the weighting filter
         *-----------------------------------------------------------------*/
        /* st_fx->mem_w0 = xn[L_SUBFR-1] - (gain_pit*y1[L_SUBFR-1]) - (gain_code*y2[L_SUBFR-1]); */
        Ltmp = Mpy_32_16_1(gain_code, y2[L_SUBFR-1]);
        Ltmp = L_shl(Ltmp, add(5,Q_xn));
        Ltmp = L_mac(Ltmp, y1[L_SUBFR-1], gain_pit);
        /* Add Gaussian contribution*/
        Ltmp2 = Mpy_32_16_1(gain_code2, y22[L_SUBFR-1]);
        Ltmp2 = L_shl(Ltmp2, add(5,Q_xn));
        Ltmp = L_add(Ltmp, Ltmp2);
        LPDmem->mem_w0 =sub(xn[L_SUBFR-1], round_fx(L_shl(Ltmp, 1)));
        move16();
        BASOP_SATURATE_WARNING_OFF;
        LPDmem->mem_w0 =shr(LPDmem->mem_w0, shift); /*Qnew-1*/
        BASOP_SATURATE_WARNING_ON;



        /*-------------------------------------------------------*
         * - Find the total excitation.                          *
         *-------------------------------------------------------*/

        tmp2 = shr(L_SUBFR, 1);
        FOR (j = 0; j < 2; j++)
        {
            FOR (i = sub(tmp2, shr(L_SUBFR, 1)); i < tmp2; i++)
            {
                /* code in Q9, gain_pit in Q14; exc Q_new */
                Ltmp = Mpy_32_16_1(gain_code2, code2[i]);
                Ltmp = L_shl(Ltmp, Q_new_p5);
                Ltmp = L_mac(Ltmp, gain_pit, exc[i+i_subfr]);
                BASOP_SATURATE_WARNING_OFF
                exc2[i] = round_fx(L_shl(Ltmp, 1));
                BASOP_SATURATE_WARNING_ON

                Ltmp2 = Mpy_32_16_1(gain_code_vect[j], code[i]);
                Ltmp2 = L_shl(Ltmp2, Q_new_p5);
                Ltmp = L_add(Ltmp, Ltmp2);
                BASOP_SATURATE_WARNING_OFF
                Ltmp = L_shl(Ltmp, 1);       /* saturation can occur here */
                BASOP_SATURATE_WARNING_ON
                exc[i + i_subfr] = round_fx(Ltmp);
            }
            tmp2 = L_SUBFR;
            move16();
        }

        /*-----------------------------------------------------------------*
        * Prepare TBE excitation
        *-----------------------------------------------------------------*/

        IF( st->igf != 0 )
        {
            prep_tbe_exc_fx( L_frame,
                             i_subfr,
                             gain_pit,
                             gain_code,
                             code,
                             voice_fac,
                             &voice_factors[i_subfr/L_SUBFR],
                             bwe_exc,
                             gain_preQ,
                             code_preQ,
                             Q_new,
                             T0,
                             T0_frac,
                             coder_type,
                             st->core_brate_fx );
        }

        /*---------------------------------------------------------*
         * Enhance the excitation                                  *
         *---------------------------------------------------------*/

        E_UTIL_enhancer(  voice_fac,
                          stab_fac,
                          past_gcode,
                          gain_inov,
                          &LPDmem->gc_threshold,
                          code,
                          exc2,
                          gain_pit,
                          &st->dm_fx.prev_gain_code,
                          st->dm_fx.prev_gain_pit,
                          &st->dm_fx.prev_state,
                          coder_type,
                          acelp_cfg->fixed_cdk_index[j_subfr],
                          L_SUBFR,
                          L_frame,
                          Q_new
                       );

        /*----------------------------------------------------------*
         * - compute the synthesis speech                           *
         *----------------------------------------------------------*/

        E_UTIL_synthesis(1, p_Aq, exc2, &syn2[i_subfr], L_SUBFR, LPDmem->mem_syn2, 1, M);

        /*Save data for BPF*/

        move16();
        move16();
        /* st->bpf_T[j_subfr] = (int)((float)T0+(float)T0_frac/(float)T0_res+0.5f); */
        st->bpf_T[j_subfr] = add(T0, shr(div_s(T0_frac, T0_res), 14));
        st->bpf_gainT[j_subfr] = gain_pit;

        E_UTIL_synthesis(1, p_Aq, &exc[i_subfr], &syn[i_subfr], L_SUBFR, &syn[i_subfr-M], 0, M);

        /*----------------------------------------------------------*
         * Update                                                   *
         *----------------------------------------------------------*/
        move16();
        move16();
        p_A += (M+1);
        p_Aq += (M+1);

        IF( hPlc_Ext != NULL )
        {
            hPlc_Ext->T0_4th = T0;
            move16();
        }

        move32();
        st->gain_code[j_subfr] = gain_code;

        j_subfr = add(j_subfr, 1);
    } /* end of subframe loop */

    p_A  -= (M+1);
    p_Aq -= (M+1);


    /*----------------------------------------------------------*
     * Update LPD memory                                        *
     *----------------------------------------------------------*/
    Copy (exc+L_frame-L_EXC_MEM, LPDmem->old_exc, L_EXC_MEM);
    Copy(syn+L_frame-M, LPDmem->mem_syn, M);
    Copy(syn+L_frame-L_SYN_MEM, LPDmem->mem_syn_r, L_SYN_MEM);

    IF( hPlc_Ext != NULL )
    {
        hPlc_Ext->Q_exp = sub( Q_new, hPlc_Ext->Q_new );
        move16();
        hPlc_Ext->Q_new = Q_new;
        move16();
        Copy( exc+L_frame-L_EXC_MEM-8, hPlc_Ext->old_exc_Qold, 8);
    }

    /*----------------------------------------------------------*
     * ZIR at the end of the ACELP frame (for TCX)              *
     *----------------------------------------------------------*/
    Copy(syn2, syn, L_frame);
    move16();
    tmp = LPDmem->syn[M];
    E_UTIL_deemph2(sub(Q_new,1), syn, st->preemph_fac, L_frame, &tmp);

    bufferCopyFx(syn+L_frame-(L_frame/2), LPDmem->Txnq, shr(L_frame,1),0 /*Qf_syn*/, -1 /*Qf_Txnq*/, 0 /*Q_syn*/ , 0 /*Q_Txnq*/);
    Copy(syn+L_frame-M-1, LPDmem->syn, 1+M); /*Q0*/
    Copy(syn, synth, L_frame);

    assert(T0_res <= 6);

    /*Update MODE1*/
    Copy(p_Aq, st->old_Aq_12_8_fx, M+1 );
    st->old_Es_pred_fx = Es_pred;

    return 0;
}
