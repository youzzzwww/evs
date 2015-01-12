/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*VERSIONINFO: File up to date with trunk rev. 39929*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_dec_fx.h"
#include "options.h"
#include "stl.h"

extern const Word16 T_DIV_L_Frame[];/*0Q15 * 2^-7 */

/*Table 2^7 * 1/L_frame */

#define L_SYN_BUF M+L_DIV_MAX+L_DIV_MAX
#define L_EXC_BUF OLD_EXC_SIZE_DEC_16k+L_DIV_MAX+L_SUBFR+1+L_DIV_MAX/2

/*LOCAL FUNCTIONS*/
static void memsynPrecission(Word16 nbLostCmpt,Word16* mem_syn, Word16* exc, Word16 len, Word16*s_16);



/***************************************************************
 *  \brief Main concealment function for ACELP
 *
 *
****************************************************************/

void con_acelp(
    const   Word16 A[],                   /*<! i     Q12: coefficients NxAz[M+1]                                  */
    const   Word16 coder_type,            /*<! i     Q0 : ACELP coder type                                        */
    Word16 synth[],               /*<! i/o   Qx :   synthesis buffer,                 Headroom: 3 bits?   */
    Word16 *pT,                   /*<! o     Q0 :   pitch for all subframe   [4]pit_min..pit_max          */
    Word16 *pgainT,               /*<! o     Q14:   pitch gain for all subfr [4] 0...1.3                  */
    const   Word16 stab_fac,              /*<! i     Q15: stability of isf                                        */
    Decoder_State_fx *st,
    const   Word16 *Qf_exc,               /*<! i       :  Q format for excitation buffer                          */
    Word16 *Qf_mem_syn,           /*<! i/o     :  Q format for st->mem_syn            >rescaling done     */
    Word16 *pitch_buffer,
    Word16 *voice_factors,
    Word16 *bwe_exc
)
{

    Word16 i_subfr, i, T0; /*Q0*/
    Word16 tmp, tmp2, tmp_deemph;
    Word16 mem_syn[M], mem_syn2[M], *syn;
    Word16 *noise_buf;
    Word16 *exc, *harmonic_exc_buf, buf[L_EXC_MEM_DEC+M+L_FRAME_16k+L_FRAME_16k/2];  /*Qf_exc*/
    const Word16 *p_A;
    Word32 pitch_buf[NB_SUBFR16k] /*15Q16*/;
    Word16 alpha; /*Q14*/
    Word16 step, gain, gainCNG,ftmp;
    Word16 *pt_exc;
    Word16 gain_inov;
    Word16 gainCNG_e ; /*scaling Factor (exponent) of gainCNG*/
    Word16 Qf_syn;    /*Q format and exponent of new synthesis*/
    Word16 *pt1_exc;
    Word32 tmp_tc; /*15Q16*/
    Word16 extrapolationFailed;
    Word32 predPitchLag; /*15Q16*/
    Word32 pc; /*float 15Q16*/
    Word16 fUseExtrapolatedPitch;/*int*/
    Word16 tmpSeed, Tc; /*Q0*/
    Word16 s_32;
    Word16 synthScaling;
    PWord16  const *w = st->tcx_cfg.tcx_mdct_window;
    Word16 W1, W2;
    Word16 nSubframes;
    Word16 s_16, s_gain_inov, s2;
    Word16 tmp_16, gain_16;
    Word32 tmp_32, gain_32, step_32;
    Word16 l_fir_fer;
    Word16 lpFiltAdapt[3];
    Word16 hp_filt[3];
    Word16 Qf_syn_new;
    Word16 exc_unv[L_FRAME_16k+L_FRAME_16k/2];
    Word16 syn_unv[L_FRAME_16k+L_FRAME_16k/2];
    Word16 mem_syn_unv[M];
    Word16 gain_lpc[NB_SUBFR16k];
    Word16 gain_lpc_e[NB_SUBFR16k];
    Word16 mem[M];
    Word16 h1[L_FRAME_16k/4+1];
    Word16 gainSynthDeemph;
    Word16 gainSynthDeemph_e;
    Word16 l;
    Word16 g, g_e;
    Word16 n;
    const Word16 scale_h1 = 5;

    /*Inits*/
    move16();
    l_fir_fer = L_FIR_FER;

    fUseExtrapolatedPitch = 0;
    move16();
    extrapolationFailed = 1;
    move16();

    move16();
    move16();
    move16();
    alpha = 0;
    /*st->Mode2_lp_gainc    = L_shl(st->Mode2_lp_gainc,7);*/ /*rudiment, could be changed in the whole file but should work also*/
    pc=L_deposit_l(0);

    /*------------------------------------------------------------------------*
     * Initialize buffers                                                     *
     *------------------------------------------------------------------------*/

    /* set ACELP synthesis memory */
    Copy( st->mem_syn2_fx, mem_syn, M);

    /* set excitation memory*/
    harmonic_exc_buf = buf+M;
    exc = harmonic_exc_buf+L_EXC_MEM_DEC;
    Copy( st->old_exc_fx, harmonic_exc_buf, L_EXC_MEM_DEC);
    exc[st->L_frame_fx] = 0;
    move16();

    /*------------------------------------------------------------------------*
     * PLC: [ACELP:Extrapolate Pitch Lag]
     *------------------------------------------------------------------------*/

    if (sub(st->flagGuidedAcelp, 1) == 0)
    {
        T0 = st->guidedT0;
        move16();
    }

    pitch_pred_linear_fit(st->nbLostCmpt, st->last_good_fx,
                          st->old_pitch_buf_fx,
                          &st->old_fpitch,
                          &predPitchLag, st->pit_min, st->pit_max, st->mem_pitch_gain, 0,
                          st->plc_use_future_lag, &extrapolationFailed, st->nb_subfr);
    T0 = round_fx(predPitchLag);

    IF (extrapolationFailed != 0)
    {
        /*------------------------------------------------------------------------*
         * - Construct adaptive codebook from side information                    *
         *------------------------------------------------------------------------*/

        IF (st->flagGuidedAcelp == 0)
        {
            nSubframes = 0;
            move16();
        }
        ELSE
        {
            nSubframes = 2;
            move16();
            /* Construct adaptive codebook with T0, T0_frac, T0_res, gain_pit for 2 sub-frames */
            l = shl(L_SUBFR,1);
            FOR (i = 0; i < l; i++)
            {
                exc[i] = exc[sub(i,st->guidedT0)];
                move16();
            }
        }
    }
    ELSE
    {
        nSubframes = 0;
        move16();
    }

    tmp_tc = st->old_fpitch;
    move16();        /* take the previous frame last pitch*/
    if( nSubframes > 0 )
    {
        tmp_tc = L_deposit_h(st->guidedT0);    /* take the transmit pitch*/
    }

    /* PLC: [ACELP: Fade-out]
     * PLC: calculate damping factor */
    alpha = Damping_fact(coder_type, st->nbLostCmpt, st->last_good_fx, stab_fac, &(st->Mode2_lp_gainp), 0); /*Q14*/
    st->cummulative_damping = shl(mult(st->cummulative_damping,alpha),1);/*shl(Q15*Q14,1)=shl(Q14,1) = Q15*/
    if (sub(st->nbLostCmpt,1)==0)
    {
        st->cummulative_damping = FL2WORD16(1.f); /*Q15*/
    }

    /*-----------------------------------------------------------------*
     * PLC: [ACELP: adaptive codebook]
     * PLC: Construct the harmonic part of excitation
     *-----------------------------------------------------------------*/

    IF( sub(st->last_good_fx , UNVOICED_TRANSITION ) >= 0)
    {

        /*---------------------------------------------------------------*
         *  Last pitch cycle of the previous frame is repeatedly copied. *
         *---------------------------------------------------------------*/

        Tc = round_fx(tmp_tc);
        BASOP_SATURATE_WARNING_OFF /*if this ever saturates, it doesn't matter*/
        tmp = sub(shl(abs_s(sub(T0,Tc)),6) , mult(FL2WORD16_SCALE(0.15f,-2),shl(Tc,4)) /*Q6*/);
        BASOP_SATURATE_WARNING_ON
        test();
        test();
        test();
        if ((T0 > 0) && (sub(T0 , Tc) != 0 )
                && ( tmp < 0 )
                && extrapolationFailed == 0 )
        {
            fUseExtrapolatedPitch = 1;
            move16();
        }

        pt_exc = exc;
        if (st->enableGplc != 0)
        {
            pt_exc = &exc[nSubframes*L_SUBFR];
        }
        pt1_exc = pt_exc - Tc;

        IF (fUseExtrapolatedPitch != 0)
        {
            /* Required because later pt1_exc[1] used in filtering points to exc[0]. To make it safe also for GPL pt_exc is used instead of exc */
            pt_exc[0] = 0;
            move16();
            pt_exc = harmonic_exc_buf;
            assert(pt_exc < pt1_exc-1);
        }

        /* if 1st erased frame after a good ACELP frame and not NB */
        IF (sub(st->nbLostCmpt, 1) == 0)
        {
            /* pitch cycle is first low-pass filtered */

            /*get filter coefficients*/
            genPlcFiltBWAdap(
                st->sr_core, /*W32 Q0*/
                &lpFiltAdapt[0] /*Q15*/,
                0,
                st->cummulative_damping /*Q15*/
            );
            FOR( i=0 ; i< Tc; i++ )
            {

                /* *pt_exc++ = ( lpFiltAdapt[0] * pt1_exc[-1] + lpFiltAdapt[1] * pt1_exc[0] + lpFiltAdapt[2] * pt1_exc[1]);*/
                tmp_32  =   L_mult(       lpFiltAdapt[0] , pt1_exc[-1])  ;
                tmp_32  =   L_mac(tmp_32, lpFiltAdapt[1] , pt1_exc[0])  ;
                tmp_16  =   mac_r(tmp_32, lpFiltAdapt[2]  , pt1_exc[1]);
                move16();
                *pt_exc =   tmp_16;
                pt_exc++;
                pt1_exc++;
            }
        }
        ELSE
        {
            /* copy the first pitch cycle without low-pass filtering */
            FOR( i=0 ; i< Tc; i++ )
            {
                move16();
                *pt_exc++ = *pt1_exc++;
            }
        }

        if (fUseExtrapolatedPitch != 0)
        {
            pt1_exc = harmonic_exc_buf;
        }

        l = add(st->L_frame_fx,sub(imult1616(L_SUBFR,sub(1,nSubframes)),Tc));
        FOR( i= 0; i < l; i++ )
        {
            move16();
            *pt_exc++ = *pt1_exc++;
        }

        /*-------------------------------------------------------*
         *  PLC: [ACELP: adaptive codebook]
         *  PLC: Resync pulse positions.
         *-------------------------------------------------------*/

        IF( nSubframes > 0 )
        {
            move16();
            move16();
            pitch_buf[0] = L_deposit_h(st->guidedT0);
            pitch_buf[1] = L_deposit_h(st->guidedT0);
        }

        IF (nSubframes>0)
        {
            move16();
            move16();
            pitch_buf[3] = pitch_buf[2] = pitch_buf[1]; /* do not resync on second half of frame */
            if (sub(st->nb_subfr, 5) == 0)
            {
                /* for guided acelp cases and nSubframes=2, set pitch_buf[4] to avoid memory_access issues in post_decoder() */
                pitch_buf[4] = pitch_buf[3];
                move16();
            }
        }
        ELSE
        {
            IF (fUseExtrapolatedPitch != 0)
            {

                get_subframe_pitch(st->nb_subfr, st->old_fpitch, predPitchLag, pitch_buf);
                PulseResynchronization(harmonic_exc_buf, exc, st->L_frame_fx, st->nb_subfr, st->old_fpitch, predPitchLag);
            }
            ELSE {
                set32_fx(pitch_buf, st->old_fpitch, st->nb_subfr);
            }
        }

        /*------------------------------------------------------------*
         *  PLC: [ACELP: adaptive codebook]
         *  PLC: Create the harmonic part needed for the overlap-add.
         *------------------------------------------------------------*/
        pt_exc = exc+st->L_frame_fx;
        pt1_exc = pt_exc - T0;
        if (T0 == 0)
        {
            pt1_exc = pt_exc - Tc;
        }
        l = shr(st->L_frame_fx, 1);
        FOR (i = 0; i < l; i++)
        {
            *pt_exc++ = *pt1_exc++;
            move16();
        }

        /*-------------------------------------------------------*
         *  PLC: [ACELP: adaptive codebook]
         *  PLC: update the floating point pitch for consecutive loss
         *-------------------------------------------------------*/

        IF (fUseExtrapolatedPitch != 0)
        {
            move32();
            st->old_fpitch = predPitchLag;
            if (sub(st->flagGuidedAcelp ,1) == 0)
            {
                st->old_fpitch = L_deposit_h(T0);
            }
        }

        /*-------------------------------------------------------*
         *  PLC: [ACELP: adaptive BPF]
         *  PLC: Accommodate the BPF
         *-------------------------------------------------------*/

        st->bpf_gain_param = 3 ; /*full BPF*/                                                      move16();

        /*-------------------------------------------------------*
         *  PLC: [ACELP: adaptive codebook]
         *  PLC: Calculate the initial gain and fade out step.
         *-------------------------------------------------------*/

        /* Compute pitch coherence (copied from decoder classifier)*/

        pc = L_abs(L_sub(L_add(pitch_buf[3], L_sub(pitch_buf[2], pitch_buf[1])), pitch_buf[0])); /*9Q6*/ /*> 15Q16*/

        /*                         mapping: floor(( 0.824[15Q15]-x[15Q0]*0.0733[0Q15] )*4)  */

        pc = Mpy_32_16_1(   L_shl(pc,1)/*precompensate Q14 from table*/,               /*15Q16*/
                            T_256DIV_L_Frame[L_shr(L_msu0(54000,shr(st->L_frame_fx,5),2402),15-2)]);

        test();
        test();/*test();*/
        IF (sub(st->last_good_fx , UNVOICED_TRANSITION) <=  0
            && (sub(coder_type   , GENERIC            ) == 0)
            &&  L_sub(pc         , 6*2*32768/*6(15Q16)*/ ) >  0 /*&& (stab_fac <= 0.5f)*/
           )
        {
            gain = 0;
            move16();
            st->Mode2_lp_gainp = L_deposit_l(0);
        }
        ELSE
        {
            gain = 0x4000 /*1 (1Q14)*/; /* start-of-the-frame gain */
            st->Mode2_lp_gainp = L_shl(L_deposit_l(alpha),15); /*1Q14->2Q29*/
        }

        tmp_16 = extract_l(L_shl(L_mac(-28000,st->L_frame_fx,95),1-15));
        tmp_16 = T_DIV_L_Frame[tmp_16];
        tmp_32 = L_mult0(tmp_16 , sub(gain , alpha));/* 0Q15 * 2^-7  * 1Q14  -> 2Q29 * 2^-7*/
        tmp_32 = L_shr(tmp_32,6); /*-> 1Q30*/
        step = round_fx(tmp_32); /*->1Q14*/

        /*FLC: step: 6.25e-5 .. 0.0045*/ /*-> s_step = -7*/
        /*lp_gainp : 0..0.2856..0.98*/

        /*-------------------------------------------------------*
         *  PLC: [ACELP: Fade-out]
         *  Apply fade out
         *-------------------------------------------------------*/

        tmp_16 = 0;
        move16();
        l = 0;
        move16();
        FOR (i_subfr = 0; i_subfr < st->nb_subfr; i_subfr++)
        {
            pgainT[i_subfr] = gain; /*Q14*/                                                           move16();
            i = l;
            move16();
            l = add(l, L_SUBFR);
            FOR (; i < l; i++)
            {
                move16();
                BASOP_SATURATE_WARNING_OFF
                exc[i] = mult_r(exc[i] , shl(gain,1)); /*overflow is first iteration because gain may be 1 after shift*/
                BASOP_SATURATE_WARNING_ON
                gain = sub(gain , step);
            }
        }

        l = add(st->L_frame_fx,shr(st->L_frame_fx,1));
        FOR (; i < l; i++ )
        {
            move16();
            BASOP_SATURATE_WARNING_OFF
            exc[i] = mult_r(exc[i] , shl(gain,1)); /*overflow is first iteration because gain may become 1 due to shift*/
            BASOP_SATURATE_WARNING_ON
            gain = sub(gain , step);
        }

        FOR (i = 0; i < st->nb_subfr; i ++)
        {
            pT[i] =  round_fx(pitch_buf[i]); /*Q0*/
            pitch_buffer[i] = round_fx(pitch_buf[i]);
        }

        /* update old exc without random part*/
        Copy(harmonic_exc_buf+st->L_frame_fx, st->old_exc_fx,  L_EXC_MEM_DEC);
    }
    ELSE
    {
        /* No harmonic part */
        assert( (int)(sizeof(buf)/sizeof(buf[0])) - M - L_EXC_MEM_DEC >= st->L_frame_fx + st->L_frame_fx/2);
        set16_fx(&exc[0], 0, add(st->L_frame_fx, shr(st->L_frame_fx,1)));

        FOR (i = 0; i < st->nb_subfr; i ++)
        {
            pitch_buf[i] = L_deposit_h(st->L_frame_fx); /*15Q16*/
            pgainT[i] = 0;
            move16();
            pT[i] = L_SUBFR;
            move16();
            pitch_buffer[i] = L_SUBFR;
            move16();
        }

        st->bpf_gain_param = 0; /*no BPF*/                                                          move16();
    }

    /*-----------------------------------------------------------------*
     * Construct the random part of excitation (5/2 st->L_frame_fx + 2L_FIR_FER - 2)
     *
     * This is done in Q0 and will be corrected to current Q format of excitation
     * when doing "non-causal ringing of the FIR filter"
     *
     * search for "Scale from randomized buffer to excitation buffer"
     *-----------------------------------------------------------------*/
    noise_buf = buf;
    move16();
    tmpSeed = st->seed_acelp;
    move16();
    l = add(st->L_frame_fx,sub(l_fir_fer,1));
    FOR (i = 0; i < l; i++)
    {
        tmpSeed = own_random2_fx(tmpSeed);
        noise_buf[i] = tmpSeed;   /*Q0*/                                                            move16();
    }

    st->seed_acelp = tmpSeed;
    move16();
    l = add(add(st->L_frame_fx,shr(st->L_frame_fx,1)),sub(l_fir_fer,1));
    FOR ( ; i < l; i++)
    {
        tmpSeed = own_random2_fx(tmpSeed);
        noise_buf[i] = tmpSeed;  /*Q0*/                                                             move16();
    }

    /*get filter coefficients*/
    genPlcFiltBWAdap(st->sr_core, /*W32 Q0*/
                     &hp_filt[0], /*Q15*/
                     1,
                     st->cummulative_damping); /*Q15*/

    /* PLC: [ACELP: Fade-out]
     * PLC: retrieve background level */

    tmp2 = shl(div_s(st->L_frame_fx,shl(L_SUBFR,3)),3-15);


    tmp = FL2WORD16(1.0f);
    gainSynthDeemph=getLevelSynDeemph(&(tmp),
                                      A,
                                      M,
                                      L_SUBFR,
                                      st->preemph_fac,
                                      tmp2,
                                      &gainSynthDeemph_e);



    /*gainCNG=st->cngTDLevel/gainSynthDeemph;*/
    BASOP_Util_Divide_MantExp(st->cngTDLevel, st->cngTDLevel_e, gainSynthDeemph, gainSynthDeemph_e,&gainCNG, &gainCNG_e);
    gainCNG_e=sub(gainCNG_e,15-5);/*Q15->Q5*/
    if(gainCNG==0)
    {
        gainCNG_e = 0;
        move16();
    }

    gain_32 = st->Mode2_lp_gainc; /*15Q16 *//* start-of-the-frame gain */                         move16();

    ftmp = round_fx(L_shl(gain_32,1));/*Q0*/

    BASOP_SATURATE_WARNING_OFF
    tmp_16 = sub(shl(gainCNG,sub(gainCNG_e,5/*Q5*/)),ftmp);
    /*in case of overflow:*/
    test();
    if ((sub(shl(ftmp,sub(gainCNG_e,1)),MAXVAL_WORD16) == 0) && (gainCNG == MAXVAL_WORD16))
    {
        move16();
        tmp_16 = 1;
    }
    BASOP_SATURATE_WARNING_ON

    IF (tmp_16 > 0 )
    {
        gainCNG = ftmp/*Q0*/;
        gainCNG_e = 5; /*-> Q5*/
        s_16 = norm_s(gainCNG);
        gainCNG = shl(gainCNG,s_16);
        gainCNG_e = sub(gainCNG_e,s_16);
    }

    /* end-of-the-frame gain */
    /* st->Mode2_lp_gainc = alpha * st->Mode2_lp_gainc + (1.0f - alpha) * gainCNG;*/
    tmp_32 = Mpy_32_16_1(st->Mode2_lp_gainc /*Q16*/,alpha/*Q14*/);/*Q31-16 = Q15*/
    s_32 = norm_l(tmp_32);
    tmp_32 = L_shl(tmp_32,s_32);
    tmp_16 = round_fx(tmp_32);
    s_16 = negate(s_32);
    s_16 = sub(s_16,-1-15); /*->Q15*/

    tmp2 = sub(16384/*1 in Q14*/,alpha); /*Q14*/
    tmp2 = mult(tmp2,gainCNG); /*Q14+Q5 +1 -16 = Q4*/
    s2 = norm_s(tmp2);
    tmp2 = shl(tmp2,s2);
    s2 = add(negate(s2),gainCNG_e);
    s2 = sub(s2,4-15);/*->Q15*/

    s_16 = BASOP_Util_Add_MantExp(tmp_16,s_16,tmp2,s2,&tmp_16);
    st->Mode2_lp_gainc = L_shl(L_deposit_l(tmp_16),add(s_16,1));
    test();
    IF( (sub(st->last_good_fx, UNVOICED_TRANSITION)==0 ) && (sub(coder_type,GENERIC)==0) )
    {
        st->Mode2_lp_gainc = L_deposit_h(gainCNG);/*Q21*/
        st->Mode2_lp_gainc = L_shr(st->Mode2_lp_gainc,sub(5,gainCNG_e)); /*15Q16, no scaling*/
    }

    highPassFiltering(st->last_good_fx, add(st->L_frame_fx,shr(l_fir_fer,1)), noise_buf, hp_filt, l_fir_fer);

    pt_exc = noise_buf + l_fir_fer/2;

    /*** Find energy normalization factor ***/
    /*gain_inov = 1.0f / (float)sqrt( dot_product( pt_exc, pt_exc, st->L_frame_fx ) / st->L_frame_fx );*//* normalize energy */ /*<--- FLC*/

    BASOP_SATURATE_WARNING_OFF /*norm_llQ31 at the end of Dot_productSq16HQ may throw an overflow, but result is okay*/
    tmp_32 = Dot_productSq16HQ(0,pt_exc,st->L_frame_fx,&s_32);
    BASOP_SATURATE_WARNING_ON
    s_32 = add(s_32, 31-1);
    /*scalingfactor is twice the headroom (at noise insertion onto the buffer), -1 (because of mult) +31 (Result is Q31) +s_32 (output scalingfactor of dot_product)*/

    tmp_16        = T_DIV_L_Frame[L_shl(L_mac(-28000,st->L_frame_fx,95),1-15)];
    tmp_32        = Mpy_32_16_1(tmp_32,tmp_16 ); /* Q31 * 2^s_32  * 0Q15 * 2^-7 */

    s_32          = sub(s_32,7);          /*tmp_32 is Q31 * 2^s_32 */

    /*assure doing Isqrt not for 0*/
    IF (tmp_32 != 0)
    {
        s_gain_inov = s_32;
        tmp_32      = ISqrt32(tmp_32, &s_gain_inov);
    }
    ELSE
    {
        s_gain_inov = 0;
        tmp_32 = 0;
    }


    gain_inov = round_fx(tmp_32);/*Inverse sqrt*/ /* Q15 * 2^s_gain_inov */

    /* PLC: [ACELP: Fade-out]
     * PLC: Linearly attenuate the gain through the frame */

    step_32 = L_sub(gain_32, st->Mode2_lp_gainc); /* 15Q16 */
    tmp_16 = extract_l(L_shl(L_mac(-28000,st->L_frame_fx,95),1-15));
    step_32 = Mpy_32_16_1(step_32, T_DIV_L_Frame[tmp_16]); /* 15Q16 * 2^-7 = 15Q16 * Q15 * 2^-7 */
    step_32 = L_shr(step_32, 7); /* 15Q16 */

    test();
    if ((sub(st->last_good_fx ,UNVOICED_CLAS)==0) && (sub(coder_type,UNVOICED)!= 0)) /* Attenuate somewhat on unstable unvoiced */
    {
        gain_inov = mult_r(gain_inov, FL2WORD16(0.8f)); /*Q15 * 2^s_gain_inov*/
    }

    IF ( sub(st->last_good_fx , UNVOICED_TRANSITION)>=0 )
    {
        Word16 tilt_code;

        /*tilt_code = (float)(0.10f*(1.0f + st->voice_fac));*/
        tilt_code = mac_r(FL2WORD32(0.1f), FL2WORD16(0.1f) , st->voice_fac);

        gain_inov = mult_r(gain_inov, sub(FL2WORD16(1.0f), tilt_code)); /* Q15 * 2^s_gain_inov */
    }

    pt_exc = noise_buf;

    /* non-causal ringing of the FIR filter   */

    /* gain_16 = gain_32 = gain_inov * gain */
    gain_32 = Mpy_32_16_1(gain_32, gain_inov);  /* 15Q16 * Q15 * 2^s_gain_inov */
    gain_32 = L_shl(gain_32, add(15, s_gain_inov)); /* Q31 */
    gain_16 = round_fx(gain_32); /* Q15 */

    /* step_32 = gain_inov * step */
    step_32 = Mpy_32_16_1(step_32, gain_inov);  /* 15Q16 * Q15 * 2^s_gain_inov */
    step_32 = L_shl(step_32, add(15, s_gain_inov)); /* Q31 */

    g_e = norm_s(round_fx(L_shl(Mpy_32_16_1(st->Mode2_lp_gainc, gain_inov), add(15, s_gain_inov)))); /* norm_s for gain*gain_inov at the end of the following loops */
    g_e = s_min(norm_s(gain_16), g_e);
    gain_16 = shl(gain_16, g_e);
    gain_32 = L_shl(gain_32, g_e);
    step_32 = L_shl(step_32, g_e);
    l = shr(l_fir_fer,1);
    FOR( i=0 ; i < l; i++ )
    {
        /* *pt_exc++ *= (gain_inov * gain); <=> *pt_exc++ *= gain_16; */ /*<-- FLC*/
        *pt_exc = mult_r(*pt_exc , gain_16); /* Q0 = Q0 * Q15 */               move16();
        pt_exc++;
    }

    /* gain -= step; gain is updated after the loop and inside the loop gain_16 = gain_inov * gain is modified using gain_inov * (gain-step) =  gain_inov * gain - gain_inov * step */ /*<-- FLC*/
    FOR( i=0 ; i < st->L_frame_fx; i++ )
    {
        /* *pt_exc++ *= (gain_inov * gain); <=> *pt_exc++ *= gain_16; */ /*<-- FLC*/
        *pt_exc = mult_r(*pt_exc , gain_16);  /* Q0 = Q0 * Q15 */              move16();
        pt_exc++;

        gain_32 = L_sub(gain_32, step_32);
        gain_16 = round_fx(gain_32);
    }

    l = add(shr(st->L_frame_fx,1),l_fir_fer/2);
    FOR( i=0 ; i < l; i++ )
    {
        /* *pt_exc++ *= (gain_inov * gain); <=> *pt_exc++ *= gain_16; */ /*<-- FLC*/
        *pt_exc = mult_r(*pt_exc , gain_16); /* Q0 = Q0 * Q15 */               move16();
        pt_exc++;
    }

    /*store st->past_gcode*/
    /* at this point gain is equal to st->Mode2_lp_gainc, so we don't need to calculate gain at all */
    st->past_gcode = st->Mode2_lp_gainc; /*15Q16 */                                                               move16();

    /*-----------------------------------------------------------------*
     * PLC: [ACELP: general]
     * PLC: Construct the total excitation
     *-----------------------------------------------------------------*/

    IF( st->last_good_fx < UNVOICED_TRANSITION )
    {
        bufferCopyFx(noise_buf+l_fir_fer/2, exc, add(st->L_frame_fx,shr(st->L_frame_fx,1)), 0, *Qf_exc, negate(g_e), 0); /*copy between different formats*/
        Copy(harmonic_exc_buf+st->L_frame_fx, st->old_exc_fx, L_EXC_MEM_DEC);
        Copy(exc,  exc_unv, add(st->L_frame_fx, shr(st->L_frame_fx,1)));   /* Update exc_unv */
    }
    ELSE
    {
        /* Update exc_unv */
        bufferCopyFx(noise_buf+l_fir_fer/2, exc_unv, add(st->L_frame_fx,shr(st->L_frame_fx,1)), 0, *Qf_exc, negate(g_e), 0); /*copy between different formats*/
    }

    /* Compute total excitation in noisebuffer to save memories */
    IF( sub( st->last_good_fx, UNVOICED_TRANSITION )  >= 0 )
    {
        Vr_add(exc, exc_unv, noise_buf, add(st->L_frame_fx, 1));
    }
    ELSE
    {
        noise_buf = exc_unv;
    }

    IF( sub( st->L_frame_fx, L_FRAME ) == 0 )
    {
        interp_code_5over2_fx(noise_buf, bwe_exc, st->L_frame_fx);
        set16_fx(voice_factors, st->last_voice_factor_fx, NB_SUBFR);
    }
    ELSE
    {
        interp_code_4over2_fx(noise_buf, bwe_exc, st->L_frame_fx);
        set16_fx(voice_factors, st->last_voice_factor_fx, NB_SUBFR16k);
    }

    /*----------------------------------------------------------*
     * - compute the synthesis speech                           *
     *----------------------------------------------------------*/

    /* Init syn buffer */
    syn = buf + M;
    Copy(st->mem_syn2_fx, buf, M );

    IF (sub(st->nbLostCmpt,1) == 0)
    {
        IF( st->last_good_fx < UNVOICED_TRANSITION )
        {
            Copy(st->mem_syn2_fx, mem_syn_unv, M );
        }
        ELSE
        {
            set16_fx( mem_syn_unv, 0, M );
        }
    }
    ELSE
    {
        Copy( st->mem_syn_unv_back, mem_syn_unv, M );
    }

    /* voiced synth */
    IF(st->last_good_fx >= UNVOICED_TRANSITION)
    {
        p_A = A;
        FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
        {
            tmp = 0;
            move16();
            set16_fx(h1, 0, L_SUBFR+1);
            set16_fx(mem, 0, M);
            h1[0] = FL2WORD16(1.0f/((float)(1 << scale_h1)));
            move16();
            E_UTIL_synthesis(0, p_A, h1, h1, L_SUBFR, mem, 0, M); /* impulse response of LPC     */
            deemph_fx(h1, st->preemph_fac, L_SUBFR, &tmp); /* impulse response of deemph  */

            /* impulse response level = gain introduced by synthesis+deemphasis */
            /* gain_lpc[i_subfr/L_SUBFR] = 1.f/(float)sqrt(dotp( h1, h1, L_SUBFR)); */
            tmp_32 = Dot_productSq16HQ(0, h1, L_SUBFR, &gain_lpc_e[i_subfr/L_SUBFR]);
            gain_lpc_e[i_subfr/L_SUBFR] = add(gain_lpc_e[i_subfr/L_SUBFR], 2*scale_h1);
            move16();
            gain_lpc[i_subfr/L_SUBFR] = round_fx(ISqrt32(tmp_32, &gain_lpc_e[i_subfr/L_SUBFR]));

            p_A += (M+1); /* Pointer move */
        }

        g = 0;
        move16();
        FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
        {

            g = mult_r(st->last_gain_syn_deemph, gain_lpc[i_subfr/L_SUBFR]);
            g_e = add(st->last_gain_syn_deemph_e, gain_lpc_e[i_subfr/L_SUBFR]);
            g = shl(g, g_e);
            FOR (i=0; i < L_SUBFR; i++)
            {
                /* exc[i_subfr + i] *= st->last_gain_syn_deemph*gain_lpc[j]; */
                exc[i_subfr + i] = mult_r(exc[i_subfr + i], g);
                move16();
            }
        }
        l = add(st->L_frame_fx, shr(st->L_frame_fx, 1));
        FOR (i = st->L_frame_fx; i < l; i++)
        {
            exc[i] = mult_r(exc[i], g);
            move16();
        }

        /*Rescale the synthesis memory*/
        Qf_syn_new = *Qf_mem_syn;
        move16();
        Qf_syn = *Qf_mem_syn;
        move16();
        rescale_mem(Qf_exc, &Qf_syn_new, &Qf_syn, mem_syn, NULL, M, st->L_frame_fx);
        synthScaling = sub(*Qf_exc,Qf_syn);

        p_A = A;

        /*in case of more than 5 consecutive concealed frames, improve precision of synthesis*/
        memsynPrecission(st->nbLostCmpt,mem_syn, exc, st->L_frame_fx, &s_16);
        FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
        {


            E_UTIL_synthesis(synthScaling, p_A, &exc[i_subfr], &syn[i_subfr], L_SUBFR, mem_syn, 1, M);
            p_A += (M+1);
        }
        Copy( mem_syn, mem_syn2, M );
        /* synthesize ola*/
        E_UTIL_synthesis(synthScaling, p_A-(M+1), &exc[i_subfr], &syn[i_subfr], (st->L_frame_fx/2), mem_syn2, 0, M);
    }

    test();
    IF(sub(st->nbLostCmpt,5)>0 && (s_16 > 0) )
    {
        /*scale back mem_syn, exc and synthesis*/
        Scale_sig(mem_syn,M,negate(s_16));
        Scale_sig(syn, add(shr(st->L_frame_fx,1),st->L_frame_fx) ,negate(s_16));
        /*Scale_sig(exc, add(shr(st->L_frame_fx,1),st->L_frame_fx) ,negate(s_16));*/
    }


    /* unvoiced synth */

    tmp = 0;
    move16();
    p_A = st->Aq_cng;

    FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
    {
        set16_fx(h1, 0, L_SUBFR+1);
        set16_fx(mem, 0, M);
        h1[0] = FL2WORD16(1.0f/((float)(1 << scale_h1)));
        move16();
        E_UTIL_synthesis(0, p_A, h1, h1, L_SUBFR, mem, 0, M); /* impulse response of LPC     */
        deemph_fx(h1, st->preemph_fac, L_SUBFR, &tmp); /* impulse response of deemph  */

        /* impulse response level = gain introduced by synthesis+deemphasis */
        /* gain_lpc[i_subfr/L_SUBFR] = 1.f/(float)sqrt(dotp( h1, h1, L_SUBFR)); */
        tmp_32 = Dot_productSq16HQ(0, h1, L_SUBFR, &gain_lpc_e[i_subfr/L_SUBFR]);
        gain_lpc_e[i_subfr/L_SUBFR] = add(gain_lpc_e[i_subfr/L_SUBFR], 2*scale_h1);
        move16();
        gain_lpc[i_subfr/L_SUBFR] = round_fx(ISqrt32(tmp_32, &gain_lpc_e[i_subfr/L_SUBFR]));

        p_A += (M+1); /* Pointer move */
    }

    g = 0;
    move16();
    FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
    {
        g = mult_r(st->last_gain_syn_deemph, gain_lpc[i_subfr/L_SUBFR]);
        g_e = add(st->last_gain_syn_deemph_e, gain_lpc_e[i_subfr/L_SUBFR]);
        g = shl(g, g_e);
        FOR (i=0; i < L_SUBFR; i++)
        {
            /* exc[i_subfr + i] *= st->last_gain_syn_deemph*gain_lpc[j]; */
            exc_unv[i_subfr + i] = mult_r(exc_unv[i_subfr + i], g);
            move16();
        }
    }
    l = add(st->L_frame_fx, shr(st->L_frame_fx, 1));
    FOR (i = st->L_frame_fx; i < l; i++)
    {
        exc_unv[i] = mult_r(exc_unv[i], g);
        move16();
    }

    /* Update Qf_syn */
    Qf_syn_new = *Qf_mem_syn;
    move16();
    Qf_syn = *Qf_mem_syn;
    move16();
    rescale_mem(Qf_exc, &Qf_syn_new, &Qf_syn, mem_syn_unv, NULL, M, st->L_frame_fx);
    synthScaling = sub(*Qf_exc,Qf_syn);
    *Qf_mem_syn = Qf_syn;

    p_A = st->Aq_cng;

    /*in case of more than 5 consecutive concealed frames, improve precision of synthesis*/
    memsynPrecission(st->nbLostCmpt,mem_syn_unv, exc_unv, st->L_frame_fx, &s_16);

    FOR (i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR)
    {
        E_UTIL_synthesis(synthScaling, p_A, &exc_unv[i_subfr], &syn_unv[i_subfr], L_SUBFR, mem_syn_unv, 1, M);
        p_A += (M+1);
    }
    Copy(mem_syn_unv,st->mem_syn_unv_back,M);

    IF(sub(st->last_good_fx,UNVOICED_TRANSITION) < 0)
    {
        Copy(mem_syn_unv,mem_syn,M);
        /* unvoiced for ola */
        E_UTIL_synthesis(synthScaling, p_A-(M+1), &exc_unv[i_subfr], &syn_unv[i_subfr], shr(st->L_frame_fx,1), mem_syn_unv, 0, M);
    }

    test();
    IF(sub(st->nbLostCmpt,5)>0 && (s_16 > 0) )
    {
        /*scale back mem_syn_unv, exc_unv and synthesis*/
        Scale_sig(mem_syn_unv,M,negate(s_16));
        IF(sub(st->last_good_fx,UNVOICED_TRANSITION) < 0)
        {
            Scale_sig(mem_syn,M,negate(s_16));
            Scale_sig(syn_unv, add(shr(st->L_frame_fx,1),st->L_frame_fx) ,negate(s_16));
        }
        ELSE
        {
            Scale_sig(syn_unv, st->L_frame_fx ,negate(s_16));
        }

        Scale_sig(st->mem_syn_unv_back,M,negate(s_16));

        /*Scale_sig(exc_unv, add(shr(st->L_frame_fx,1),st->L_frame_fx) ,negate(s_16));*/
    }

    /* add separate synthesis buffers */
    IF (sub(st->last_good_fx,UNVOICED_TRANSITION) >= 0)
    {
        FOR( i=0 ; i < st->L_frame_fx; i++ )
        {
            syn[i] = add(syn[i], syn_unv[i]);
            move16();
        }
    }
    ELSE
    {
        Copy(syn_unv,syn,add(st->L_frame_fx, shr(st->L_frame_fx,1)));
    }


    /* update buffer for the classification */
    {
        Word16 pit16[NB_SUBFR16k];
        Word16 k;
        FOR(k = 0 ; k < st->nb_subfr; k++)
        {
            pit16[k] = round_fx(L_shl(pitch_buf[k],6));/*Q6*/
        }

        FEC_clas_estim_fx(
            st,
            /*Opt_AMR_WB*/0, /*A*/
            st->L_frame_fx,
            &(st->clas_dec),
            coder_type,
            pit16,
            &st->classifier_last_good,
            syn,
            &st->lp_ener_FER_fx,
            /**decision_hyst*/NULL,     /* i/o: hysteresis of the music/speech decision                           */
            /**UV_cnt*/ NULL,           /* i/o: number of consecutives frames classified as UV                    */
            /**LT_UV_cnt*/ NULL,        /* i/o: long term consecutives frames classified as UV                    */
            /**Last_ener*/ NULL,        /* i/o: last_energy frame                                                 */
            /**locattack*/ NULL,        /* i/o: detection of attack (mainly to localized speech burst)            */
            /**lt_diff_etot*/NULL,      /* i/o: long-term total energy variation                                  */
            /**amr_io_class*/ NULL,     /* i/o: classification for AMR-WB IO mode                                 */
            /*bitrate*/ 0  ,            /* i  : Decoded bitrate                                                   */
            &Qf_syn,                    /* i  : Synthesis scaling                                                 */
            /**class_para*/ NULL,       /* o  : classification para. fmerit1                                      */
            st->mem_syn_clas_estim_fx,  /* i/o: memory of the synthesis signal for frame class estimation         */
            &st->classifier_Q_mem_syn, /*i/o : exponent for memory of synthesis signal for frame class estimation */
            st->pit_max,                /* i  : maximum pitch value, Q0                                           */
            FL2WORD16(-1.f),            /* i  : LTP Gain                                                          */
            0/*CLASSIFIER_ACELP*/,      /* i  : signal classifier mode                                            */
            1/*bfi*/,                   /* i  : bad frame indicator                                               */
            M,                          /* i  : starting point of synthesis buffer                                */
            add(st->L_frame_fx,
        shr(st->L_frame_fx,1))      /* i  : length of synthesis buffer, relevant for rescaling                */
        );
    }

    /* Update Pitch Lag memory */
    Copy32(&st->old_pitch_buf_fx[st->nb_subfr], st->old_pitch_buf_fx, st->nb_subfr);
    Copy32(pitch_buf, &st->old_pitch_buf_fx[st->nb_subfr], st->nb_subfr);


    /*updating enr_old parameters*/
    frame_ener_fx( st->L_frame_fx, st->last_good_fx, syn, round_fx(tmp_tc), &(st->enr_old_fx), 1, 0, 0, 0 );

    st->enr_old_fx = L_shl(st->enr_old_fx,shl(negate(Qf_syn),1));

    /* update ACELP synthesis memory */
    Copy(mem_syn, st->mem_syn2_fx , M);
    Copy(syn+st->L_frame_fx-L_SYN_MEM, st->mem_syn_r , L_SYN_MEM);

    /*Q_mem_syn_new = Q_mem_syn;*//*NOT "+synthScaling", cause mem_syn format is not changed*/
    /* Deemphasis and output synth */
    tmp_deemph = st->syn[M];

    E_UTIL_deemph2(*Qf_mem_syn, syn, st->preemph_fac, add(st->L_frame_fx,shr(st->L_frame_fx,1)), &tmp_deemph);
    tmp_deemph = shr(syn[st->L_frame_fx-1],1); /*Q0->Q-1*/

    Copy(syn, synth, st->L_frame_fx);

    bufferCopyFx(syn+st->L_frame_fx-st->L_frame_fx/2, st->old_syn_Overl, shr(st->L_frame_fx,1),0 /*Qf_syn*/, -1 /*Qf_old_xnq*/, 0 , 0 /*Q_old_xnq*/);

    /* save last half frame if next frame is TCX */
    bufferCopyFx(syn+st->L_frame_fx, st->syn_Overl_TDAC, shr(st->L_frame_fx,1),0 /*Qf_syn*/, -1 /*Qf_old_xnq*/, 0 , 0 /*Q_old_xnq*/ );
    Copy(syn+st->L_frame_fx-M-1, st->syn, add(1,M));

    /* update old_Aq */
    Copy(p_A-(M+1), st->old_Aq_12_8_fx, add(M,1));

    /* Update TCX-LTP */
    IF ( st->tcxltp != 0 )
    {
        st->tcxltp_last_gain_unmodified /*Q15*/      =  round_fx(L_shl(st->Mode2_lp_gainp /*2Q29*/,2-1));
        if( st->Mode2_lp_gainp == 0 )
        {
            st->tcxltp_last_gain_unmodified = 0;
            move16();
        }
    }

    Copy(syn+st->L_frame_fx, st->syn_Overl, shr(st->L_frame_fx,1));

    /* create aliasing and windowing */

    W1 = st->tcx_cfg.tcx_mdct_window_length;
    move16();
    W2 = shr(W1,1);

    st->tcx_cfg.tcx_curr_overlap_mode = FULL_OVERLAP;
    move16();

    n = extract_h(L_mult(st->L_frame_fx,FL2WORD16((float)N_ZERO_MDCT_NS/(float)FRAME_SIZE_NS)));

    bufferCopyFx(syn+st->L_frame_fx-n, st->old_out_LB_fx, sub(st->L_frame_fx, n), 0, 0, st->Q_old_wtda_LB, 0);
    FOR (i=0; i < W2; i++)
    {
        st->old_out_LB_fx[i+n] = round_fx(Mpy_32_16_1(L_mult(w[i].v.re,w[i].v.re),st->old_out_LB_fx[i+n]));
    }
    FOR ( ; i < W1; i++)
    {
        st->old_out_LB_fx[i+n] = round_fx(Mpy_32_16_1(L_mult(w[W2-1-(i-W2)].v.im,w[W2-1-(i-W2)].v.im),st->old_out_LB_fx[i+n]));
    }
    set16_fx(&st->old_out_LB_fx[W1+n], 0, n);

    st->Q_old_wtda = st->Q_old_wtda_LB;



    FOR (i=0; i<W2; i++)
    {
        move16();
        buf[i] = mult_r(st->syn_Overl_TDAC[i],w[i].v.re);

    }
    FOR(; i<W1; i++)
    {
        move16();
        buf[i] = mult_r(st->syn_Overl_TDAC[i],w[W1-1-i].v.im);
    }


    FOR (i=0; i<W2; i++)
    {
        move16();
        st->syn_Overl_TDAC[i] = add(buf[i],buf[W1-1-i]); /* A-D */
    }
    /*-2*/
    FOR (i=0; i<W2; i++)
    {
        move16();
        st->syn_Overl_TDAC[W2+i] = add(buf[W2+i],buf[W1-1-W2-i]);/* B-C */
    }


    FOR (i=0; i<W2; i++)
    {
        move16();
        st->syn_Overl_TDAC[i] = mult_r(st->syn_Overl_TDAC[i],w[i].v.re);
    }

    FOR(; i<W1; i++)
    {
        move16();
        st->syn_Overl_TDAC[i] = mult_r(st->syn_Overl_TDAC[i],w[W1-1-i].v.im);
    }

    /* update memory for full band */
    lerp(st->syn_Overl_TDAC, st->syn_Overl_TDACFB, shr(st->L_frameTCX, 1), shr(st->L_frame_fx, 1));
    lerp(st->syn_Overl, st->syn_OverlFB, shr(st->L_frameTCX, 1), shr(st->L_frame_fx, 1));
    lerp(st->old_out_LB_fx, st->old_out_fx, st->L_frameTCX, st->L_frame_fx);

    /* copy total excitation exc2 as 16kHz for acelp mode1 decoding */
    lerp(exc, st->old_exc2_fx, L_EXC_MEM, st->L_frame_fx);
    lerp(syn, st->old_syn2_fx, L_EXC_MEM, st->L_frame_fx);
    st->bfi_pitch_fx = shl(round_fx(pitch_buf[st->nb_subfr-1]),6);
    move16();
    st->bfi_pitch_frame_fx = st->L_frame_fx;
    move16();

    return;
}

static void memsynPrecission(Word16 nbLostCmpt,Word16* mem_syn, Word16* exc, Word16 len, Word16*s_16)
{
    IF(sub(nbLostCmpt,5)>0 )
    {
        Word16 sf_mem_syn, sf_exc,k, tmp_loop, max, tmp, i;
        tmp = 0;
        move16();
        *s_16 = 0;
        move16();
        max = 0;
        move16();

        /*check energy of mem_syn*/
        FOR(i=0; i<M; i++)
        {
            /*saturation doesn't matter*/
            BASOP_SATURATE_WARNING_OFF
            tmp = add(tmp,abs_s(mem_syn[i]));
            BASOP_SATURATE_WARNING_ON
        }
        /*if there is energy in scale_syn, then increase precision*/
        IF(abs_s(tmp) > 0 )
        {
            sf_mem_syn = getScaleFactor16(mem_syn,M);
            /*sf_exc = getScaleFactor16(exc, add(shr(len,1),len));*/ /*this returns 0 if signal is 0*/
            tmp_loop = add(shr(len,1),len);
            FOR(k=0; k<tmp_loop; k++)
            {
                max = s_max(max,abs_s(exc[k]));
            }
            sf_exc = norm_s(max);
            if (max==0)
            {
                sf_exc = 16;
                move16();
            }
            move16();
            *s_16 = s_max(sub(s_min(sf_exc,sf_mem_syn),5),0)/*5 bits of headroom, scaling not smaller than 0*/;
            Scale_sig(mem_syn,M,*s_16);
            Scale_sig(exc, add(shr(len,1),len) ,*s_16);
        }
    }

}
