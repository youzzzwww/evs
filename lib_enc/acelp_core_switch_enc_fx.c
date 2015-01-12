/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"                /* required for wmc_tool */

/*---------------------------------------------------------------------*
   * Local functions
   *---------------------------------------------------------------------*/

static void encod_gen_voic_core_switch_fx( Encoder_State_fx *st_fx,
        LPD_state *mem,
        const Word16 L_frame_fx, const Word16 inp_fx[],
        const Word16 Aq_fx[], const Word16 A_fx[], const Word16 coder_type_fx, const Word16 T_op_fx[],
        const Word16 voicing_fx[], Word16 *exc_fx, const Word32 core_bitrate_fx,  Word16 shift, Word16 Q_new );

void bwe_switch_enc_fx( Encoder_State_fx *st_fx, const Word16 *new_speech, Word16 *synth_subfr_bwe );

static Word16 dotprod_satcont(const Word16 *x, const Word16 *y, Word16 qx, Word16 qy, Word16 *qo, Word16 len, Word16 delta);

/*-------------------------------------------------------------------*
 * acelp_core_switch_enc_fx()
 *
 * ACELP core encoder in the ACELP->HQ switching frame
 *--------------------------------------------------------------------*/

void acelp_core_switch_enc_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure                */
    LPD_state    *mem,
    const Word16 inp12k8[],              /* i  : input signal @12.8 kHz  Q0             */
    const Word16 inp16k[],               /* i  : input signal @16 kHz    Q0             */
    const Word16 T_op_orig[2],           /* i  : open-loop pitch values for quantiz. Q0 */
    const Word16 voicing[3],             /* i  : Open-loop pitch gains   Q15            */
    const Word16 A[NB_SUBFR16k*(M+1)],   /* i  : A(z) unquantized for the 4 subframes Q12*/
    Word16 *synth_subfr_bwe,       /* o  : CELP transition subframe  BWE          */
    Word16 shift,
    Word16 Q_new
)
{
    Word16 i, T_op[2],tmp;
    Word16 old_exc[L_EXC], *exc;         /* excitation signal buffer      Qexc           */
    const Word16 *inp;
    Word32  cbrate;
    Word16 Aq[2*(M+1)];

    /* initializations */
    exc = &old_exc[L_EXC_MEM];
    move16();          /* pointer to excitation signal in the current frame */
    Copy( mem->old_exc, old_exc, L_EXC_MEM );      /*now old_exc has the same scaling as st_fx->old_exc; need to change later? */

    Copy( st_fx->old_Aq_12_8_fx, Aq, M+1 );
    Copy( st_fx->old_Aq_12_8_fx, Aq + (M+1), M+1 );

    T_op[0] = T_op_orig[0];
    move16();
    T_op[1] = T_op_orig[1];
    move16();

    /*----------------------------------------------------------------*
     * set switching frame bit-rate
     *----------------------------------------------------------------*/

    IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )   /* ACELP@12k8 core */
    {
        inp = inp12k8;

        IF( L_sub(st_fx->core_brate_fx, ACELP_24k40 )  > 0 )
        {
            cbrate = L_add(ACELP_24k40, 0);
        }
        ELSE
        {
            cbrate = L_add(st_fx->core_brate_fx, 0);
        }
    }
    ELSE /* ACELP@16k core */
    {
        inp = inp16k;

        IF( L_sub(st_fx->core_brate_fx, ACELP_8k00) <= 0 )
        {
            cbrate = L_add(ACELP_8k00, 0);
        }
        ELSE IF (L_sub(st_fx->core_brate_fx, ACELP_14k80) <= 0 )
        {
            cbrate = L_add(ACELP_14k80, 0);
        }
        ELSE
        {
            cbrate = L_min( st_fx->core_brate_fx, ACELP_22k60 );
        }
    }

    IF( sub(st_fx->last_L_frame_fx, st_fx->L_frame_fx) != 0 )
    {
        IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
        {
            T_op[0] = shr(add(round_fx(L_shl(L_mult(20480, T_op[0]), 1)), 1), 1);
            move16();
            T_op[1] = shr(add(round_fx(L_shl(L_mult(20480, T_op[1]), 1)), 1), 1);
            move16();
        }
        ELSE
        {
            T_op[0] = shr(add(round_fx(L_shl(L_mult(26214, T_op[0]), 2)), 1), 1);
            move16();
            T_op[1] = shr(add(round_fx(L_shl(L_mult(26214, T_op[1]), 2)), 1), 1);
            move16();
        }
    }

    /*----------------------------------------------------------------*
     * Excitation encoding
     *----------------------------------------------------------------*/

    encod_gen_voic_core_switch_fx( st_fx, mem, st_fx->last_L_frame_fx, inp, Aq, A, GENERIC, T_op, voicing, exc, cbrate, shift, Q_new );

    /*----------------------------------------------------------------*
     * bit-stream: modify the layer of sub frame CELP
     *----------------------------------------------------------------*/

    FOR( i=0; i<20; i++ )
    {
        st_fx->ind_list_fx[IND_CORE_SWITCHING_CELP_SUBFRAME+i].value=st_fx->ind_list_fx[TAG_ACELP_SUBFR_LOOP_START+i].value;
        move16();
        st_fx->ind_list_fx[IND_CORE_SWITCHING_CELP_SUBFRAME+i].nb_bits=st_fx->ind_list_fx[TAG_ACELP_SUBFR_LOOP_START+i].nb_bits;
        move16();
        st_fx->ind_list_fx[TAG_ACELP_SUBFR_LOOP_START+i].nb_bits=-1;
        move16();
    }

    /*----------------------------------------------------------------*
     * BWE encoding
     *----------------------------------------------------------------*/

    test();
    test();
    IF( ( sub(st_fx->last_L_frame_fx, L_FRAME16k) == 0 && sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME16k) == 0 ) || sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME8k) == 0 )
    {
        set16_fx( synth_subfr_bwe, 0, SWITCH_MAX_GAP );
    }
    ELSE
    {

        tmp = 0;
        bwe_switch_enc_fx( st_fx, (const Word16 *)st_fx->old_input_signal_fx, synth_subfr_bwe );
        Scale_sig(synth_subfr_bwe,SWITCH_MAX_GAP,negate(tmp));
    }

    return;
}

/*-------------------------------------------------------------------*
   * encod_gen_voic_core_switch()
   *
   * Encode excitation signal in ACELP->HQ switching frame
   *-------------------------------------------------------------------*/

static void encod_gen_voic_core_switch_fx(
    Encoder_State_fx *st_fx,          /* i/o: state structure                  */
    LPD_state *mem,
    const Word16 L_frame,           /* i  : length of the frame              */
    const Word16 inp[],             /* i  : input signal                     */
    const Word16 Aq[],              /* i  : LP coefficients                  */
    const Word16 A[],               /* i  : unquantized A(z) filter          */
    const Word16 coder_type,        /* i  : coding type                      */
    const Word16 T_op[],            /* i  : open loop pitch                  */
    const Word16 voicing[],         /* i  : voicing                          */
    Word16 *exc,              /* i/o: current non-enhanced excitation  */
    const Word32 core_bitrate,      /* i  : switching frame bit-rate         */
    Word16 shift,
    Word16 Q_new
)
{
    Word16 res[L_SUBFR];            /* residual signal  Qexc                 */
    Word16 Ap[M+1];                 /* A(z) with spectral expansion  Q12     */
    Word16 xn[L_SUBFR];             /* Target vector for pitch search        */
    Word16 xn2[L_SUBFR];            /* Target vector for codebook search     */
    Word16 cn[L_SUBFR];             /* Target vector in residual domain      */
    Word16 h1[L_SUBFR+(M+1)];       /* Impulse response vector               */

    Word16 code[L_SUBFR];           /* Fixed codebook excitation  Q9         */
    Word16 y1[L_SUBFR];             /* Filtered adaptive excitation          */
    Word16 y2[L_SUBFR];             /* Filtered algebraic excitation         */
    Word16 gain_pit ;               /* Pitch gain    Q15                     */
    Word16 voice_fac;               /* Voicing factor  Q15                   */
    Word32 gain_code;               /* Gain of code    Q16                   */
    Word16 gain_inov;               /* inovation gain                        */
    Word16 i,gcode16;               /* tmp variables                         */
    Word16 T0, T0_frac;             /* close loop integer pitch and fractional part */
    Word16 T0_min, T0_max;          /* pitch variables                       */

    Word16 pitch,tmp16;             /* floating pitch value                  */
    Word16 g_corr[6];               /* ACELP correl, values + gain pitch     */
    Word16 clip_gain;               /* ISF clip gain                         */

    Word16 unbits;                  /* number of unused bits for  PI         */
    Word32 norm_gain_code;
    Word16 pitch_limit_flag;
    Word32 L_tmp, Lgcode;
    Word16 shift_wsp;
    Word16 h2[L_SUBFR+(M+1)];
    Word16 lp_select, lp_flag;

    /*------------------------------------------------------------------*
    * Initializations
    *------------------------------------------------------------------*/

    shift_wsp = add(Q_new,shift);

    unbits = 0;
    move16();

    IF( sub(L_frame, L_FRAME) == 0 )
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

    /*------------------------------------------------------------------*
    * Calculation of LP residual (filtering through A[z] filter)
    *------------------------------------------------------------------*/

    tmp16=st_fx->L_frame_fx;
    move16();
    st_fx->L_frame_fx=L_SUBFR;
    move16();
    calc_residu_fx(st_fx,inp,res,Aq,0);
    st_fx->L_frame_fx=tmp16;
    move16();

    /*------------------------------------------------------------------*
    * ACELP subframe loop
    *------------------------------------------------------------------*/


    Copy( res, exc, L_SUBFR );

    IF( sub(L_frame,L_FRAME16k)==0 )
    {
        weight_a_fx( A, Ap, GAMMA16k, M ); /* Bandwidth expansion of A(z) filter coefficients */
        find_targets_fx(inp, mem->mem_syn, 0, &mem->mem_w0, Aq, res, L_SUBFR, Ap, TILT_FAC_FX, xn, cn ,h1);
    }
    ELSE
    {
        weight_a_fx( A, Ap, GAMMA1, M );   /* Bandwidth expansion of A(z) filter coefficients */
        find_targets_fx(inp, mem->mem_syn,0,&mem->mem_w0, Aq, res, L_SUBFR, Ap, TILT_FAC_FX,  xn, cn ,h1);
    }

    /*Scale_sig(h1, L_SUBFR, shift);  *//*Q14-shift */
    Copy_Scale_sig(h1, h2, L_SUBFR, -2);
    Scale_sig(h1, L_SUBFR, add(1, shift)); /* set h1[] in Q14 with scaling for convolution */

    /* scaling of xn[] to limit dynamic at 12 bits */
    Scale_sig(xn, L_SUBFR, shift);

    /*----------------------------------------------------------------*
    * Close-loop pitch search and quantization
    * Adaptive exc. construction
    *----------------------------------------------------------------*/

    pitch = pit_encode_fx( st_fx, core_bitrate, 0, L_frame, coder_type, &pitch_limit_flag,
                           0, exc, L_SUBFR, T_op, &T0_min, &T0_max, &T0, &T0_frac, h1, xn );

    /*-----------------------------------------------------------------*
    * Find adaptive exitation
    *-----------------------------------------------------------------*/

    pred_lt4( &exc[0], &exc[0], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );

    /*-----------------------------------------------------------------*
    * Gain clipping test to avoid unstable synthesis on frame erasure
    *   or in case of floating point encoder & fixed p. decoder
    *-----------------------------------------------------------------*/

    clip_gain = gp_clip_fx( voicing, 0, coder_type, xn, st_fx->clip_var_fx, sub(shift_wsp, 1) );

    /*-----------------------------------------------------------------*
    * LP filtering of the adaptive excitation, codebook target computation
    *-----------------------------------------------------------------*/

    lp_select = lp_filt_exc_enc_fx( MODE1, core_bitrate, 0, coder_type, 0, exc, h1, xn, y1, xn2, L_SUBFR, L_frame, g_corr, clip_gain, &gain_pit, &lp_flag );

    IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
    {
        push_indice_fx( st_fx, IND_LP_FILT_SELECT, lp_select, 1 );
    }

    /*-----------------------------------------------------------------*
    * Innovation encoding
    *-----------------------------------------------------------------*/

    inov_encode_fx( st_fx, core_bitrate, 0, L_frame,st_fx->last_L_frame_fx, coder_type, st_fx->bwidth_fx, 0, 0, -1, Aq, gain_pit, cn, exc,
                    h2, mem->tilt_code, pitch, xn2, code, y2, &unbits,shift);

    /*-----------------------------------------------------------------*
    * Gain encoding
    *-----------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        gain_enc_mless_fx( st_fx,core_bitrate, L_frame, TRANSITION, 0, -1, xn, y1, shift_wsp, y2, code, st_fx->old_Es_pred_fx,
                           &gain_pit, &gain_code, &gain_inov, &norm_gain_code, g_corr, clip_gain );
    }
    ELSE
    {
        gain_enc_mless_fx( st_fx,core_bitrate, L_frame, coder_type, 0, -1, xn, y1, shift_wsp, y2, code, st_fx->old_Es_pred_fx,
        &gain_pit, &gain_code, &gain_inov, &norm_gain_code, g_corr, clip_gain );
    }

    gp_clip_test_gain_pit_fx( gain_pit, st_fx->clip_var_fx );

    Lgcode = L_shl(gain_code, Q_new);       /* scaled gain_code with Qnew -> Q16*/
    gcode16 = round_fx(Lgcode);

    mem->tilt_code = Est_tilt2( exc+0, gain_pit, code, gain_code, &voice_fac,shift );

    /*-----------------------------------------------------------------*
    * Construct adaptive part of the excitation
    *-----------------------------------------------------------------*/

    FOR( i = 0; i < L_SUBFR; i++ )
    {
        /* code in Q9, gain_pit in Q14 */
        L_tmp = L_mult(gcode16, code[i]);
        L_tmp = L_shl(L_tmp, 5);
        L_tmp = L_mac(L_tmp, exc[i ], gain_pit);
        L_tmp = L_shl(L_tmp, 1); /* saturation can occur here */
        exc[i ] = round_fx(L_tmp);
    }

    /* write reserved bits */
    IF( unbits )
    {
        push_indice_fx(st_fx, IND_UNUSED, 0, unbits );
    }

    /*-----------------------------------------------------------------*
     * long term prediction on the 2nd sub frame
     *-----------------------------------------------------------------*/

    pred_lt4(&exc[L_SUBFR], &exc[L_SUBFR], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

    FOR( i = L_SUBFR; i < 2*L_SUBFR; i++ )
    {
        exc[i] = round_fx(L_shl(L_mult(exc[i], gain_pit), 1));
    }

    return;
}


/*-------------------------------------------------------------------*
    * bwe_switch_enc()
    *
    * Encode BWE in ACELP->HQ switching frame
    *-------------------------------------------------------------------*/

void bwe_switch_enc_fx(
    Encoder_State_fx *st_fx,
    const Word16 *new_speech_fx,        /* i  : original input signal  Q0 */
    Word16 *synth_subfr_bwe_fx    /* o  : BWE synthesis           */
)
{

    Word16 k, Loverlapp_fx, d1m_fx, maxd1_fx, ind1_fx, gapsize_fx;
    Word16 delta_fx, fdelay_fx;
    const Word16 *hp_filter_fx;
    const Word16 *fpointers_tab[6] = {hp12800_16000_fx, hp12800_32000_fx, hp12800_48000_fx, hp16000_48000_fx, hp16000_32000_fx, hp16000_48000_fx};
    Word16 tmp, shift;
    const Word16 *ptmp;
    Word16 tmp_mem_fx[2*L_FILT48k], tmp_mem2_fx[2*L_FILT48k], hb_synth_tmp_fx[480];
    Word16 Fs_kHz;
    Word16 q_tmp1, q_tmp2, Qmc, Qsq;
    Word32 L_tmp1, L_tmp2, L_tmp3, min_sq_cross_fx;
    Word16 accA_fx, accB_fx, min_corr_fx, E1_fx, E2_fx, gain_fx;

    Word16 n, L;
    L = NS2SA_fx2(st_fx->input_Fs_fx,FRAME_SIZE_NS);

    /* set multiplication factor according to the sampling rate */
    tmp = extract_l(L_shr(st_fx->input_Fs_fx,14));
    delta_fx = add(tmp,1);
    Fs_kHz = shl(delta_fx,4);
    tmp = add(tmp,i_mult2(3,(sub(st_fx->last_L_frame_fx,L_FRAME)!=0)));
    ptmp = fpointers_tab[tmp];
    move16();

    hp_filter_fx = ptmp;
    fdelay_fx=i_mult2(16,delta_fx);
    IF(sub(st_fx->last_L_frame_fx,L_FRAME)==0)
    {
        fdelay_fx=i_mult2(20,delta_fx);
    }

    n = i_mult2(N16_CORE_SW,delta_fx);

    set16_fx( tmp_mem_fx, 0, 2*L_FILT48k);
    set16_fx( tmp_mem2_fx, 0, 2*L_FILT48k);

    Loverlapp_fx = i_mult2(delta_fx,SWITCH_OVERLAP_8k*2);
    gapsize_fx = i_mult2(delta_fx,NS2SA(16000,SWITCH_GAP_LENGTH_NS));

    shift = sub(add(add(shr(L,1),n),Loverlapp_fx),gapsize_fx) ;
    Copy( new_speech_fx+shift,synth_subfr_bwe_fx,add(gapsize_fx,fdelay_fx) );
    Copy( new_speech_fx+sub(shift,fdelay_fx),tmp_mem_fx,fdelay_fx );

    tmp = add(gapsize_fx, fdelay_fx);
    fir_fx( synth_subfr_bwe_fx, hp_filter_fx, synth_subfr_bwe_fx, tmp_mem_fx, tmp, fdelay_fx, 1, 0 );
    Copy(synth_subfr_bwe_fx+shr(fdelay_fx,1),synth_subfr_bwe_fx,sub(gapsize_fx,shr(fdelay_fx,1)) );

    tmp = i_mult2(Fs_kHz,10);
    fir_fx( new_speech_fx, hp_filter_fx, hb_synth_tmp_fx, tmp_mem2_fx, tmp, fdelay_fx, 1, 0 );

    min_sq_cross_fx = L_negate(1);
    Qsq = 0;
    move16();
    min_corr_fx = 0;
    move16();
    Qmc = 0;
    move16();
    d1m_fx = 0;
    move16();

    maxd1_fx = sub(tmp,add(gapsize_fx,fdelay_fx));

    IF (sub(delta_fx, 2) == 0)
    {
        maxd1_fx = shr(maxd1_fx,1);
    }
    ELSE IF (sub(delta_fx, 3) == 0 )
    {
        maxd1_fx = extract_h(L_mult(maxd1_fx, 10923));
    }

    /* find delay */
    ptmp = &hb_synth_tmp_fx[fdelay_fx];
    move16();
    FOR( k = 0; k < maxd1_fx; k++ )
    {
        accA_fx = dotprod_satcont(ptmp, ptmp, 0, 0, &q_tmp1, gapsize_fx, delta_fx);
        accB_fx = dotprod_satcont(ptmp, synth_subfr_bwe_fx, 0, 0, &q_tmp2, gapsize_fx, delta_fx);
        ptmp += delta_fx;
        L_tmp1 = L_mult0(accB_fx, accB_fx); /*2*q_tmp2; */
        L_tmp2 = Mult_32_16(L_tmp1, min_corr_fx); /*2*q_tmp2+Qmc-15 */
        L_tmp3 = Mult_32_16(min_sq_cross_fx, accA_fx); /*Qsq+q_tmp1-15 */
        shift = s_min(add(shl(q_tmp2,1),Qmc),add(q_tmp1,Qsq));
        L_tmp2 = L_shr(L_tmp2, sub(add(shl(q_tmp2,1),Qmc),shift));
        L_tmp3 = L_shr(L_tmp3, sub(add(q_tmp1,Qsq),shift));

        IF (L_sub(L_tmp2,L_tmp3)>=0)
        {
            d1m_fx = k;
            move16();
            min_corr_fx = accA_fx;
            move16();
            Qmc = q_tmp1;
            move16();
            min_sq_cross_fx = L_add(L_tmp1, 0);
            Qsq = shl(q_tmp2,1);
            move16();
        }
    }

    push_indice_fx(st_fx, IND_CORE_SWITCHING_AUDIO_DELAY, d1m_fx, AUDIODELAYBITS );

    tmp = add(i_mult2(d1m_fx,delta_fx),fdelay_fx);
    ptmp = &hb_synth_tmp_fx[tmp];
    move16();
    E1_fx = dotprod_satcont(synth_subfr_bwe_fx, synth_subfr_bwe_fx, 0, 0, &q_tmp1, gapsize_fx, 1);
    E2_fx = dotprod_satcont(ptmp, ptmp, 0, 0, &q_tmp2, gapsize_fx, 1);

    IF (!E1_fx)
    {
        E1_fx = shl(1,14);
        q_tmp1 = 14;
        move16();
    }
    IF (!E2_fx)
    {
        E2_fx = shl(1,14);
        q_tmp2 = 14;
        move16();
    }

    tmp = div_s(shl(1, 14), E1_fx); /*Q(29-q_tmp1) */
    L_tmp1 = L_mult(tmp, E2_fx); /*30-q_tmp1+q_tmp2 */
    q_tmp2 = sub(q_tmp1, q_tmp2); /*30-q_tmp2 */
    L_tmp1 = L_shl(L_tmp1, sub(q_tmp2, 24));
    gain_fx = round_fx(Isqrt(L_tmp1));  /*Q12  */

    ind1_fx = usquant_fx( gain_fx, &gain_fx, shr(MINVALUEOFFIRSTGAIN_FX,1), shr(DELTAOFFIRSTGAIN_FX,4), (1 << NOOFGAINBITS1) );
    push_indice_fx( st_fx,IND_CORE_SWITCHING_AUDIO_GAIN,  ind1_fx, NOOFGAINBITS1 );

    return;
}

static Word16 dotprod_satcont(const Word16 *x, const Word16 *y, Word16 qx, Word16 qy, Word16 *qo, Word16 len, Word16 delta)
{
    Word16 tmp_tabx[L_FRAME48k], tmp_taby[L_FRAME48k];
    Word16 shift, q, ener, i;
    Word32 L_tmp;

    Copy( x, tmp_tabx, len );
    Copy( y, tmp_taby, len );
    shift = 0;
    move16();
    DO
    {
        L_tmp = L_deposit_l(0);
        Overflow = 0;
        move16();
        FOR ( i = 0; i < len; i += delta )
        {
            L_tmp = L_mac0(L_tmp, tmp_tabx[i], tmp_taby[i]); /*Q(qx+qy-shift) */
        }

        IF(Overflow != 0)
        {
            FOR( i = 0; i < len; i += delta )
            {
                tmp_tabx[i] = shr(tmp_tabx[i], 2);
                move16();
                tmp_taby[i] = shr(tmp_taby[i], 2);
                move16();
            }
            shift = add(shift, 4);
        }
    }
    WHILE(Overflow != 0);

    q = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, q); /*Q(qx+qy-shift+q) */
    ener = extract_h(L_tmp); /*Q(qx+qy-shift+q-16) */
    q = add(q, add(qx, qy));
    *qo = sub(q, add(shift, 16));

    return ener;
}
