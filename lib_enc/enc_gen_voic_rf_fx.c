/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "rom_basop_util.h"
#include "basop_mpy.h"



void reset_rf_indices(
    Encoder_State_fx *st         /* i: state structure - contains partial RF indices */
)
{
    st->rf_frame_type = 0; /* since this function is called every frame this will happen even for a SID frame, hence treating it as GSC frame, i.e no RF encoding */

    {
        Word16 i, j;
        st->rf_mem_w0 = 0;
        set16_fx(st->rf_clip_var, 0 ,6);
        st->rf_tilt_code = 0;
        set16_fx(st->rf_mem_syn2, 0, M);
        st->rf_dm_fx.prev_state = 0;
        st->rf_dm_fx.prev_gain_code = 0;
        FOR(i=0; i<6; i++)
        {
            st->rf_dm_fx.prev_gain_pit[i] = 0;
        }

        st->rf_gc_threshold = 0;
        set16_fx(st->rf_tilt_buf, 0, NB_SUBFR16k);

        st->rf_target_bits = 0;
        st->rf_target_bits_write = 0;
        st->rf_tcxltp_pitch_int_past = st->L_frame_fx;
        st->rf_last_tns_active = 0;
        st->rf_second_last_tns_active = 0;
        st->rf_second_last_core= 0;

        FOR( i = 0; i < MAX_RF_FEC_OFFSET; i++)
        {
            st->rf_indx_frametype[i] = RF_NO_DATA;
            st->rf_targetbits_buff[i] = 6;
            st->rf_indx_lsf[i][0] = 0;
            st->rf_indx_lsf[i][1] = 0;
            st->rf_indx_lsf[i][2] = 0;
            st->rf_indx_EsPred[i] = 0;
            st->rf_indx_nelp_fid[i] = 0;
            st->rf_indx_nelp_iG1[i] = 0;
            st->rf_indx_nelp_iG2[i][0] = 0;
            st->rf_indx_nelp_iG2[i][1] = 0;

            FOR( j = 0; j < NB_SUBFR16k; j++)
            {
                st->rf_indx_ltfMode[i][j] = 0;
                st->rf_indx_pitch[i][j] = 0;
                st->rf_indx_fcb[i][j] = 0;
                st->rf_indx_gain[i][j] = 0;
            }

            st->rf_clas[i] = UNVOICED_CLAS;
            st->rf_gain_tcx[i] = 0;
            st->rf_tcxltp_param[i] = 0;
        }
    }

    return;
}


/*-------------------------------------------------------------------*
 * coder_acelp_rf()
 *
 * Encode excitation signal (partial redundancy)
 *-------------------------------------------------------------------*/
void coder_acelp_rf(
    ACELP_config *acelp_cfg_rf,   /*input/output: configuration of the ACELP coding*/
    const Word16 coder_type,      /* input: coding type              */
    const Word16 A[],             /* input: coefficients 4xAz[M+1]   */
    const Word16 Aq[],            /* input: coefficients 4xAz_q[M+1] */
    Word16 speech[],        /* input: speech[-M..lg]           */
    const Word16 voicing[],       /* input: open-loop LTP gain       */
    const Word16 T_op[],          /* input: open-loop LTP lag        */
    Word16 stab_fac,
    Encoder_State_fx *st,
    Word16 target_bits,     /* i/o : coder memory state         */
    const Word16 rf_frame_type,   /* i  : rf_frame_type               */
    Word16 *exc_rf,         /* i/o: pointer to RF excitation    */
    Word16 *syn_rf,         /* i/o: pointer to RF synthesis     */
    Word16 Q_new,
    Word16 shift
)
{
    Word16 i, j, i_subfr, j_subfr;
    Word16 T0, T0_min, T0_min_frac, T0_max, T0_max_frac, T0_res;
    Word16 T0_frac;
    Word16 tmp2;
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
    Word16 exc_nelp[L_FRAME];
    Word16 exc2[L_SUBFR];
    Word16 syn2[L_DIV_MAX];
    Word16 gain_inov;
    Word32 past_gcode;
    Word16 L_frame;
    Word16 clip_gain;
    Word32 gain_code2;
    Word16 code2[L_SUBFR];
    Word16 y22[L_SUBFR]; /* Filtered adaptive excitation */
    Word32 gain_code_vect[2];
    Word16 *prm_rf;
    Word16 Es_pred_rf;
    Word16 nSubfr;
    Word16 prev_gain_pit;
    Word16 rf_coder_type;
    Word16 lp_select;

    /* to avoid compilation warnings */
    past_gcode = 0;
    gain_inov = 0;
    T0 = 0;
    T0_frac = 0;
    T0_res = 0;
    gain_pit = 0;
    gain_code = 0;
    voice_fac = 0;
    prev_gain_pit=0;
    Es_pred_rf = 0;
    set16_fx(code, 0, L_SUBFR);

    /*-----------------------------------------------------------------------*
    * Configure ACELP partial copy                                           *
    *------------------------------------------------------------------------*/
    tmp2 = BITS_ALLOC_config_acelp( target_bits, rf_frame_type, acelp_cfg_rf, 0, st->nb_subfr );

    /* Init Framing parameters */
    L_frame = st->L_frame_fx;

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
        st->rf_dm_fx.prev_gain_code = 0;
        set16_fx(st->rf_dm_fx.prev_gain_pit, 0, 6);
        st->rf_dm_fx.prev_state = 0;
    }

    /* calculate residual */
    p_Aq = Aq;
    FOR (i_subfr=0; i_subfr<L_frame; i_subfr+=L_SUBFR)
    {

        Residu3_fx( p_Aq, &speech[i_subfr], &exc_rf[i_subfr], L_SUBFR, 1 );
        p_Aq += (M+1);

    }
    /*------------------------------------------------------------------------*
    * Find and quantize mean_ener_code for gain quantizer                    *
    *------------------------------------------------------------------------*/

    Es_pred_rf = 0;
    IF (acelp_cfg_rf->nrg_mode>0)
    {

        Es_pred_enc_fx(&Es_pred_rf, &st->rf_indx_EsPred[0], L_frame, exc_rf, voicing,
                       acelp_cfg_rf->nrg_bits, acelp_cfg_rf->nrg_mode>1, Q_new);
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
    p_A = A;
    p_Aq = Aq;

    res_save = exc_rf[0];
    nSubfr = 0;
    j_subfr = 0;

    FOR (i_subfr=0; i_subfr<L_frame; i_subfr+=L_SUBFR)
    {

        /* Restore exc[i_subfr] and save next exc[L_SUBFR+i_subfr] */
        move16();
        move16();
        exc_rf[i_subfr] = res_save;
        res_save = exc_rf[L_SUBFR+i_subfr];

        /*--------------------------------------------------------------------------*
        * Find target for pitch search (xn[]), target for innovation search (cn[]) *
        * and impulse response of the weighted synthesis filter (h1[]).            *
        *--------------------------------------------------------------------------*/
        find_targets_fx(
            speech,
            &syn_rf[i_subfr-M],
            i_subfr,
            &(st->rf_mem_w0),
            p_Aq,
            exc_rf,
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
        /* full frame nelp partial copy encoding */
        IF( sub(rf_frame_type,RF_NELP) == 0 )
        {
            IF( i_subfr == 0 )
            {
                nelp_encoder_fx( st, exc_rf, exc_nelp, &Q_new
                                 ,0
                               );
            }
            Copy( &exc_nelp[i_subfr], exc2, L_SUBFR );
            Copy( &exc_nelp[i_subfr], exc_rf, L_SUBFR );

        }
        ELSE
        {
            clip_gain = Mode2_gp_clip( voicing, i_subfr, coder_type, xn, st->rf_clip_var, L_SUBFR, Q_xn );

            /*-----------------------------------------------------------------*
            * - find unity gain pitch excitation (adaptive codebook entry)    *
            *   with fractional interpolation.                                *
            * - find filtered pitch exc. y1[]=exc[] convolved with h1[])      *
            * - compute pitch gain1                                           *
            *-----------------------------------------------------------------*/
            if( acelp_cfg_rf->gains_mode[i_subfr/L_SUBFR] == 0 )
            {
                gain_pit = prev_gain_pit;
                move16();
            }

            IF ( acelp_cfg_rf->ltp_bits!=0 )
            {
                prm_rf = &st->rf_indx_pitch[0][nSubfr];

                /* Adaptive Codebook (GC and VC) */
                Mode2_pit_encode( acelp_cfg_rf->ltp_mode,
                i_subfr,
                &prm_rf,
                &exc_rf[i_subfr],
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

                /* find ACB excitation */
                rf_coder_type = (acelp_cfg_rf->gains_mode[i_subfr/L_SUBFR]>0)?(acelp_cfg_rf->gains_mode[i_subfr/L_SUBFR]):(100);

                E_ACELP_adaptive_codebook( exc_rf,
                T0,
                T0_frac,
                T0_res,
                st->pit_res_max,
                acelp_cfg_rf->ltf_mode,
                i_subfr,
                L_SUBFR,
                L_frame,
                h1,
                clip_gain,
                xn,
                y1,
                &g_corr,
                &prm_rf,
                &gain_pit,
                xn_exp
                ,st->rf_mode
                ,rf_coder_type
                ,&lp_select
                                         );



                if( acelp_cfg_rf->ltf_mode == NORMAL_OPERATION )
                {
                    st->rf_indx_ltfMode[0][nSubfr] = lp_select;
                }
            }
            ELSE IF ( acelp_cfg_rf->ltp_bits==0 )
            {
                /* No adaptive codebook (UC) */
                gain_pit=0;
                g_corr.xy1=0;
                g_corr.xy1_e=0;
                g_corr.y1y1=0x4000; /* set to 0x4000 instead of 0 to avoid assert failue in gain_enc : assert(coeff0 >= 0x4000) */
                g_corr.y1y1_e=0;
                set16_fx(y1,0,L_SUBFR);
                set16_fx(exc_rf+i_subfr,0,L_SUBFR);
                T0 = L_SUBFR;
                T0_frac = 0;
                T0_res = 1;
            }


            /*----------------------------------------------------------------------*
            *                 Encode the algebraic innovation                      *
            *----------------------------------------------------------------------*/
            IF( acelp_cfg_rf->fixed_cdk_index[i_subfr/L_SUBFR] >= 0 )
            {
                prm_rf = &st->rf_indx_fcb[0][nSubfr];

                E_ACELP_innovative_codebook(  exc_rf,
                                              T0,
                                              T0_frac,
                                              T0_res,
                                              gain_pit,
                                              st->rf_tilt_code,
                                              acelp_cfg_rf->fixed_cdk_index[j_subfr],
                                              acelp_cfg_rf->formant_enh,
                                              acelp_cfg_rf->formant_tilt,
                                              acelp_cfg_rf->formant_enh_num,
                                              acelp_cfg_rf->formant_enh_den,
                                              acelp_cfg_rf->pitch_sharpening,
                                              acelp_cfg_rf->pre_emphasis,
                                              acelp_cfg_rf->phase_scrambling,
                                              i_subfr,
                                              p_Aq,
                                              h1,
                                              xn,
                                              cn,
                                              y1,
                                              y2,
                                              st->acelp_autocorr,
                                              &prm_rf,
                                              code,
                                              shift);
            }
            ELSE
            {
                set16_fx(code, 0, L_SUBFR);
                set16_fx(y2, 0, L_SUBFR);
            }

            E_ACELP_xy2_corr(xn, y1, y2, &g_corr, L_SUBFR, Q_xn);

            g_corr.y2y2_e = sub(g_corr.y2y2_e, 18);  /* -18 (y2*y2: Q9*Q9) */
            g_corr.xy2_e  = sub(g_corr.xy2_e,  add(Q_xn,9));  /* -(Q_xn+9) (xn: Q_xn y2: Q9) */
            g_corr.y1y2_e = sub(g_corr.y1y2_e, add(Q_xn,9));  /* -(Q_xn+9) (y1: Q_xn y2: Q9) */
            g_corr.xx_e   = sub(g_corr.xx_e,   add(Q_xn,Q_xn));  /* -(Q_xn+Q_xn) (xn: Q_xn) */


            /*----------------------------------------------------------------------*
             *                 Add Gaussian excitation                              *
             *----------------------------------------------------------------------*/
            gain_code2 = L_deposit_l(0);
            set16_fx(code2, 0, L_SUBFR);
            set16_fx(y22, 0, L_SUBFR);


            /*----------------------------------------------------------*
            *  - Compute the fixed codebook gain                       *
            *  - quantize fixed codebook gain                          *
            *----------------------------------------------------------*/
            IF( acelp_cfg_rf->gains_mode[i_subfr/L_SUBFR] != 0 )
            {
                prm_rf = &st->rf_indx_gain[0][nSubfr];

                encode_acelp_gains( code,
                acelp_cfg_rf->gains_mode[j_subfr],
                Es_pred_rf,
                clip_gain,
                &g_corr,
                &gain_pit,
                &gain_code,
                &prm_rf,
                &past_gcode,
                &gain_inov,
                L_SUBFR,
                code2,
                &gain_code2,
                st->flag_noisy_speech_snr
                                  );
            }
            gp_clip_test_gain_pit_fx( gain_pit, st->rf_clip_var );

            gain_code_vect[0] = gain_code;
            move32();
            gain_code_vect[1] = gain_code;
            move32();

            /*----------------------------------------------------------*
            * - voice factor (for pitch enhancement)                   *
            *----------------------------------------------------------*/
            E_UTIL_voice_factor( exc_rf,
            i_subfr,
            code,
            gain_pit,
            gain_code,
            &voice_fac,
            &(st->rf_tilt_code),
            L_SUBFR,
            acelp_cfg_rf->voice_tilt,
            Q_new,
            shift
                               );


            /*-----------------------------------------------------------------*
             * Update memory of the weighting filter
             *-----------------------------------------------------------------*/
            /* st_fx->_rf_mem_w0 = xn[L_SUBFR-1] - (gain_pit*y1[L_SUBFR-1]) - (gain_code*y2[L_SUBFR-1]); */
            Ltmp = Mpy_32_16_1(gain_code, y2[L_SUBFR-1]);
            Ltmp = L_shl(Ltmp, add(5,Q_xn));
            Ltmp = L_mac(Ltmp, y1[L_SUBFR-1], gain_pit);
            /* Add Gaussian contribution*/
            Ltmp2 = Mpy_32_16_1(gain_code2, y22[L_SUBFR-1]);
            Ltmp2 = L_shl(Ltmp2, add(5,Q_xn));
            Ltmp = L_add(Ltmp, Ltmp2);
            st->rf_mem_w0 =sub(xn[L_SUBFR-1], round_fx(L_shl(Ltmp, 1)));
            move16();
            BASOP_SATURATE_WARNING_OFF;
            st->rf_mem_w0 =shr(st->rf_mem_w0, shift); /*Qnew-1*/
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
                    Ltmp = L_mac(Ltmp, gain_pit, exc_rf[i+i_subfr]);
                    BASOP_SATURATE_WARNING_OFF
                    exc2[i] = round_fx(L_shl(Ltmp, 1));
                    BASOP_SATURATE_WARNING_ON

                    Ltmp2 = Mpy_32_16_1(gain_code_vect[j], code[i]);
                    Ltmp2 = L_shl(Ltmp2, Q_new_p5);
                    Ltmp = L_add(Ltmp, Ltmp2);
                    BASOP_SATURATE_WARNING_OFF
                    Ltmp = L_shl(Ltmp, 1);       /* saturation can occur here */
                    BASOP_SATURATE_WARNING_ON
                    exc_rf[i + i_subfr] = round_fx(Ltmp);
                }
                tmp2 = L_SUBFR;
            }



            /*---------------------------------------------------------*
            * Enhance the excitation                                  *
            *---------------------------------------------------------*/
            E_UTIL_enhancer(  voice_fac,
                              stab_fac,
                              past_gcode,
                              gain_inov,
                              &st->rf_gc_threshold,
                              code,
                              exc2,
                              gain_pit,
                              &st->rf_dm_fx.prev_gain_code,
                              st->rf_dm_fx.prev_gain_pit,
                              &st->rf_dm_fx.prev_state,
                              coder_type,
                              acelp_cfg_rf->fixed_cdk_index[j_subfr],
                              L_SUBFR,
                              L_frame,
                              Q_new
                           );
        }

        /*----------------------------------------------------------*
        * - compute the synthesis speech                           *
        *----------------------------------------------------------*/

        E_UTIL_synthesis(1, p_Aq, exc2, &syn2[i_subfr], L_SUBFR, st->rf_mem_syn2, 1, M);

        E_UTIL_synthesis(1, p_Aq, &exc_rf[i_subfr], &syn_rf[i_subfr], L_SUBFR, &syn_rf[i_subfr-M], 0, M);

        /*----------------------------------------------------------*
        * Update                                                   *
        *----------------------------------------------------------*/
        p_A += (M+1);
        p_Aq += (M+1);
        nSubfr++;

        st->gain_code[j_subfr] = gain_code;
        j_subfr = add(j_subfr, 1);

        /* copy current gain for next subframe use, in case there is no explicit encoding */
        prev_gain_pit = gain_pit;

    } /* end of subframe loop */


    return;
}


