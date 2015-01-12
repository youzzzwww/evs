/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"
#include "basop_util.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/
#define pitch_0_9  14746  /* 0.9 in Q14 */
#define pitch_0_6   9830  /* 0.6 in Q14 */
#define SIZE        64
#define SIZE2       32
#define NUM_STAGES   5

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/
static void phase_dispersion_fx(Word32 gain_code,Word16 gain_pit,Word16 code[],Word16 mode,struct dispMem_fx *dm_fx);
static void agc2_fx(const Word16 *sig_in,Word16 *sig_out,const Word16 l_trm);

/*======================================================================================*/
/* FUNCTION : enhancer_fx()																*/
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Enhancement of the excitation signal before synthesis						*/
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :																	*/
/* _ (Word32) core_brate : decoder bitrate												*/
/* _ (Word16) Opt_AMR_WB : flag indicating AMR-WB IO mode								*/
/* _ (Word16) coder_type : coder type													*/
/* _ (Word16) i_subfr : subframe number													*/
/* _ (Word16) voice_fac : subframe voicing estimation  (Q15)							*/
/* _ (Word16) stab_fac : LP filter stablility measure  (Q15)							*/
/* _ (Word32) norm_gain_code : normalised innovative cb. gain  (Q16)					*/
/* _ (Word16) gain_inov : gain of the unscaled innovation (Q12) 						*/
/* _ (Word16) gain_pit_fx : Pitch gain			(Q14)									*/
/* _ (Word16) Q_exc : Q of the excitation												*/
/* _ (Word16) Enc : Encoder  = 1; decoder = 0 											*/
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																	*/
/* _ (Word16*) voice_factors_fx : TBE voicing factor (Q15)								*/
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																*/
/* _ (Word32*) gc_threshold : gain code threshold  (Q16)								*/
/* _ (Word16*[]) code : innovation (Q12)												*/
/* _ (Word16*[]) exc2 : adapt. excitation/total exc (Q0)								*/
/* _ (struct dispMem_fx*) dm_fx : phase dispersion algorithm memory						*/
/*								 (a[0]->Q0,a[1]->Q16,a[2-7]->Q14)						*/
/*--------------------------------------------------------------------------------------*/

/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																	*/
/* _ None																				*/
/*======================================================================================*/
void enhancer_fx(
    const Word32 core_brate,   /* i  : decoder bitrate                          */
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode           */
    const Word16 coder_type,   /* i  : coder type                               */
    const Word16 i_subfr,      /* i  : subframe number                          */
    const Word16 L_frame,      /* i  : frame size                               */
    const Word16 voice_fac,    /* i  : subframe voicing estimation         Q15  */
    const Word16 stab_fac,     /* i  : LP filter stablility measure        Q15  */
    Word32 norm_gain_code,  /* i  : normalised innovative cb. gain   Q16  */
    const Word16 gain_inov,    /* i  : gain of the unscaled innovation     Q12  */
    Word32 *gc_threshold,/* i/o: gain code threshold                 Q16  */
    Word16 *code,        /* i/o: innovation                          Q12  */
    Word16 *exc2,        /* i/o: adapt. excitation/total exc.        Q_exc*/
    const Word16 gain_pit,     /* i  : quantized pitch gain                Q14  */
    struct dispMem_fx *dm_fx,  /* i/o: phase dispersion algorithm memory        */
    const Word16 Q_exc         /* i  : Q of the excitation                      */
)
{
    Word16 tmp, fac, *pt_exc2;
    Word16 i;
    Word32 L_tmp;
    Word16 gain_code_hi;
    Word16 pit_sharp, tmp16;
    Word16 excp[L_SUBFR], sc;

    pit_sharp = gain_pit;
    move16();       /* to remove gcc warning */
    pt_exc2 = exc2 + i_subfr;
    move16();

    /*------------------------------------------------------------*
     * Phase dispersion to enhance noise at low bit rate
     *------------------------------------------------------------*/

    i = 2;
    move16(); /* no dispersion */
    IF (Opt_AMR_WB)
    {
        IF ( L_sub(core_brate,ACELP_6k60) <= 0)
        {
            i = 0;
            move16();   /* high dispersion  */
        }
        ELSE if ( L_sub(core_brate,ACELP_8k85) <= 0)
        {
            i = 1;
            move16();    /* low dispersion  */
        }
    }
    ELSE IF( sub(coder_type,UNVOICED) != 0)

    {
        test();
        test();
        test();
        test();
        IF ( L_sub(core_brate,ACELP_7k20) <= 0 )
        {
            i = 0;
            move16(); /* high dispersion  */
        }
        ELSE if ( ( sub(coder_type,GENERIC) == 0 || sub(coder_type,TRANSITION) == 0 || sub(coder_type,AUDIO) == 0 || sub(coder_type,INACTIVE) == 0 ) && L_sub(core_brate,ACELP_9k60) <= 0 )
        {
            i = 1;
            move16();    /* low dispersion  */
        }
    }
    phase_dispersion_fx(norm_gain_code, gain_pit, code, i, dm_fx);

    /*------------------------------------------------------------
     * noise enhancer
     *
     * - Enhance excitation on noise. (modify gain of code)
     *   If signal is noisy and LPC filter is stable, move gain
     *   of code 1.5 dB toward gain of code threshold.
     *   This decreases by 3 dB noise energy variation.
     *-----------------------------------------------------------*/

    /* tmp = 0.5f * (1.0f - voice_fac) */
    tmp = msu_r(0x40000000, voice_fac, 16384); /*Q15 */ /* 1=unvoiced, 0=voiced */
    /* fac = stab_fac * tmp */
    fac = mult(stab_fac, tmp); /*Q15*/

    IF (L_sub(norm_gain_code, *gc_threshold) < 0)
    {
        L_tmp = Madd_32_16(norm_gain_code, norm_gain_code, 6226);/*Q16 */
        L_tmp = L_min(L_tmp, *gc_threshold);/*Q16 */
    }
    ELSE
    {
        L_tmp = Mult_32_16(norm_gain_code, 27536);/*Q16 */
        L_tmp = L_max(L_tmp, *gc_threshold); /*Q16 */
    }
    *gc_threshold = L_tmp;
    move32(); /*Q16 */

    /* gain_code = (fac * tmp) + (1.0 - fac) * gain_code ==> fac * (tmp - gain_code) + gain_code */
    L_tmp = L_sub(L_tmp, norm_gain_code); /*Q16 */
    norm_gain_code = Madd_32_16(norm_gain_code, L_tmp, fac);/*Q16 */

    /* gain_code *= gain_inov - Inverse the normalization */
    L_tmp = Mult_32_16(norm_gain_code, gain_inov); /*Q13*/ /* gain_inov in Q12 */

    sc = 6;
    move16();

    gain_code_hi = round_fx(L_shl(L_tmp, add(Q_exc, 3))); /* in Q_exc */

    /*------------------------------------------------------------*
     * pitch enhancer
     *
     * - Enhance excitation on voiced. (HP filtering of code)
     *   On voiced signal, filtering of code by a smooth fir HP
     *   filter to decrease energy of code at low frequency.
     *------------------------------------------------------------*/
    test();
    IF( !Opt_AMR_WB && sub(coder_type,UNVOICED) == 0 )
    {
        /* Copy(code, exc2, L_SUBFR) */
        FOR (i = 0; i < L_SUBFR; i++)
        {
            pt_exc2[i] = round_fx(L_shl(L_mult(gain_code_hi, code[i]), sc)); /*Q0 */ /* code in Q12 (Q9 for encoder) */
        }
    }
    ELSE
    {
        test();
        test();
        IF ( Opt_AMR_WB && ( L_sub(core_brate,ACELP_8k85) == 0|| L_sub(core_brate,ACELP_6k60) == 0 ) )
        {
            pit_sharp = shl(gain_pit, 1); /* saturation can occur here Q14 -> Q15 */

            /* saturation takes care of "if (pit_sharp > 1.0) { pit_sharp=1.0; }" */
            IF (sub(pit_sharp, 16384) > 0)
            {
                tmp16 = mult(pit_sharp, 8192);
                FOR (i = 0; i < L_SUBFR; i++)
                {
                    /* excp[i] = pt_exc2[i] * pit_sharp * 0.25 */
                    excp[i] = mult_r(pt_exc2[i], tmp16);
                    move16();
                }
            }
        }

        IF ( sub(L_frame, L_FRAME16k) == 0 )
        {
            /* tmp = 0.150 * (1.0 + voice_fac) */
            /* 0.30=voiced, 0=unvoiced */
            tmp = mac_r(0x10000000L, voice_fac, 4915);/*Q15 */
        }
        ELSE
        {
            /* tmp = 0.125 * (1.0 + voice_fac) */
            /* 0.25=voiced, 0=unvoiced */
            tmp = mac_r(0x10000000L, voice_fac, 4096);/*Q15 */
        }

        /*-----------------------------------------------------------------
         * Do a simple noncasual "sharpening": effectively an FIR
         * filter with coefs [-tmp 1.0 -tmp] where tmp=0...0.25.
         * This is applied to code and add_fxed to exc2
         *-----------------------------------------------------------------*/
        /* pt_exc2[0] += code[0] - tmp * code[1] */
        L_tmp = L_deposit_h(code[0]);                   /* if Enc :Q9 * Q15 -> Q25 */
        L_tmp = L_msu(L_tmp, code[1], tmp);             /* Q12 * Q15 -> Q28 */
        L_tmp = L_shl(L_mult(gain_code_hi, extract_h(L_tmp)), sc);
        pt_exc2[0] = msu_r(L_tmp, -32768, pt_exc2[0]);
        move16();/* in Q_exc */

        FOR (i = 1; i < L_SUBFR-1; i++)
        {
            /* pt_exc2[i] += code[i] - tmp * code[i-1] - tmp * code[i+1] */
            L_tmp = L_msu(-32768, code[i], -32768);
            L_tmp = L_msu(L_tmp, code[i + 1], tmp);
            tmp16 = msu_r(L_tmp, code[i - 1], tmp);
            L_tmp = L_shl(L_mult(gain_code_hi, tmp16), sc);
            pt_exc2[i] = msu_r(L_tmp, -32768, pt_exc2[i]);
            move16(); /* in Q_exc */
        }

        /* pt_exc2[L_SUBFR-1] += code[L_SUBFR-1] - tmp * code[L_SUBFR-2] */
        L_tmp = L_deposit_h(code[L_SUBFR - 1]);/*Q28 */
        L_tmp = L_msu(L_tmp, code[L_SUBFR - 2], tmp);/*Q28 */
        L_tmp = L_shl(L_mult(gain_code_hi, extract_h(L_tmp)), sc);
        pt_exc2[L_SUBFR - 1] = msu_r(L_tmp, -32768, pt_exc2[L_SUBFR - 1]);
        move16();/* in Q_exc */
        test();
        test();
        IF ( Opt_AMR_WB && ( L_sub(core_brate,ACELP_8k85) == 0 || L_sub(core_brate,ACELP_6k60) == 0 ) )
        {
            IF (sub(pit_sharp, 16384) > 0)
            {
                FOR (i = 0; i < L_SUBFR; i++)
                {
                    /* excp[i] += pt_exc2[i] */
                    excp[i] = add(excp[i], pt_exc2[i]);
                    move16();
                }
                agc2_fx(pt_exc2, excp, L_SUBFR);
                Copy(excp, pt_exc2, L_SUBFR);
            }
        }
    }
}

/*---------------------------------------------------------*
 * Enhancement of the excitation signal before synthesis
 *---------------------------------------------------------*/

Word16 E_UTIL_enhancer(
    Word16 voice_fac,           /* i  : subframe voicing estimation         Q15 */
    Word16 stab_fac,            /* i  : LP filter stability measure         Q15 */
    Word32 gain_code,           /* i  : innovative cb. gain               15Q16 */
    Word16 gain_inov,           /* i  : gain of the unscaled innovation     Q11 */
    Word32 *gc_threshold,       /* i/o: gain code threshold               15Q16 */
    Word16 *code,               /* i/o: innovation(in: Q9)             code_exp */
    Word16 *exc2,               /* i/o: adapt. excitation/total exc.            */
    Word16 gain_pit,            /* i  : Quantized pitch gain               1Q14 */
    Word32 *prev_gain_code,     /* i/o: previous codebook gain            15Q16 */
    Word16 prev_gain_pit[],     /* i/o: previous pitch gain, size=6        1Q14 */
    Word16 *prev_state,         /* i/o: Phase dispersion algorithm memory    Q0 */
    Word16 coder_type,          /* i  : coder type                              */
    Word16 cdk_index,           /* i  :                                         */
    Word16 L_subfr,             /* i  : length of subframe                      */
    Word16 L_frame,             /* i  : frame size                              */
    Word16 Q_new
)
{
    Word16 disp_mode, i;
    Word16 tmp, fac, gain;
    Word32 L_tmp;
    Word16 code_exp, exc2_exp;
    Word16 max_cdk_index_uv;

    move16();
    code_exp = 15-9;
    exc2_exp = 15-Q_new;
    gain_inov = shr(gain_inov,1);
    /*-----------------------------------------------------------------*
     * Phase dispersion to enhance noise at low bit rates
     *-----------------------------------------------------------------*/

    max_cdk_index_uv = 12;
    move16();
    if ( sub(L_frame, L_FRAME16k) == 0 )
    {
        max_cdk_index_uv = 16;
        move16();
    }
    disp_mode = 2;                 /* any=off */        move16();
    test();
    test();
    test();
    test();
    IF ( ( (sub(coder_type, VOICED) != 0) && (sub(cdk_index, 4) <= 0) ) || ( (sub(coder_type, UNVOICED) == 0) && (sub(cdk_index, max_cdk_index_uv) <= 0) ) )
    {
        disp_mode = 0; /* high */                       move16();
    }
    ELSE if ( (sub(coder_type, VOICED) != 0) && (sub(cdk_index, 9) <= 0) )
    {
        disp_mode = 1; /* low  */                       move16();
    }

    phase_dispersion(gain_code, gain_pit,code, &code_exp, disp_mode, prev_gain_code, prev_gain_pit, prev_state, L_subfr);

    /*------------------------------------------------------------*
     * noise enhancer                                             *
     * ~~~~~~~~~~~~~~                                             *
     * - Enhance excitation on noise. (modify gain of code)       *
     *   If signal is noisy and LPC filter is stable, move gain   *
     *   of code 1.5 dB toward gain of code threshold.            *
     *   This decrease by 3 dB noise energy variation.            *
     *------------------------------------------------------------*/
    fac = 0;
    move16();

    /* if gain_code is computed function of energy, noise enhancer is by-passed.*/
    BASOP_SATURATE_WARNING_OFF
    tmp = msu_r(FL2WORD32(0.5f), FL2WORD16(0.5f), voice_fac);             /* 1=unvoiced, 0=voiced */
    BASOP_SATURATE_WARNING_ON
    fac = mult_r(stab_fac, tmp);             /* fac in Q15 */

    L_tmp = L_add(0,gain_code);              /* L_tmp in 15Q16 */

    IF (L_sub(L_tmp,*gc_threshold) < 0)
    {
        L_tmp = L_shl(Mpy_32_32(L_tmp, FL2WORD32(1.19f/2.0f)),1);
        L_tmp = L_min(L_tmp, *gc_threshold);
    }
    ELSE
    {
        L_tmp = Mpy_32_32(L_tmp, FL2WORD32(1.0f/1.19f));
        L_tmp = L_max(L_tmp, *gc_threshold);
    }
    move32();
    *gc_threshold = L_tmp;               /* in 15Q16 */

    /* gain = ( (fac * L_tmp) + (gain_code - fac*gain_code) ) * gain_inov */
    /* exponent of L_tmp: 31-16 + 15-11 */
    L_tmp = Mpy_32_16_1(L_add(Mpy_32_16_1(L_tmp, fac), L_sub(gain_code, Mpy_32_16_1(gain_code, fac))), gain_inov);

    /* exponent gain: 31-16 + 15-11 - tmp */
    tmp = norm_l(L_tmp);

    /* exponent of code: 31-16 + 15-11 - tmp + code_exp */
    move16();
    code_exp = sub(add(31-16 + 15-11, code_exp), tmp);

    L_tmp = L_shl(L_tmp, tmp);
    gain = round_fx(L_tmp);

    FOR (i=0; i<L_subfr; i++)
    {
        code[i] = mult_r(code[i], gain);
        move16();
    }

    /*------------------------------------------------------------*
     * pitch enhancer                                             *
     * ~~~~~~~~~~~~~~                                             *
     * - Enhance excitation on voice. (HP filtering of code)      *
     *   On voiced signal, filtering of code by a smooth fir HP   *
     *   filter to decrease energy of code in low frequency.      *
     *------------------------------------------------------------*/

    /* exponent difference of code and exc2. +1 accounts for headroom required below. */
    gain = add(sub(code_exp, exc2_exp), 1);

    tmp = mac_r(FL2WORD32(0.125f), FL2WORD16(0.125f), voice_fac);         /* 0.25=voiced, 0=unvoiced */
    if ( sub(L_frame, L_FRAME16k) == 0 )
    {
        tmp = mac_r(FL2WORD32(0.150f), FL2WORD16(0.150f), voice_fac);     /* 0.30=voiced, 0=unvoiced */
    }

    /* exc2[0]         = exc2[0]         + code[0]         - tmp*code[1]; */
    L_tmp = L_mult(code[0], 16384);
    L_tmp = L_msu0(L_tmp,tmp,code[1]);
    if (gain)
    {
        L_tmp = L_shl(L_tmp,gain);
    }
    exc2[0] = msu_r(L_tmp,-32768, exc2[0]);
    move16();

    FOR (i=1; i<L_subfr-1; i++)
    {
        /* exc2[i]         = exc2[i]         + code[i]         - tmp*(code[i+1]+code[i-1]); */
        L_tmp = L_mult(code[i], 16384);
        L_tmp = L_msu0(L_tmp,tmp,code[i-1]);
        L_tmp = L_msu0(L_tmp,tmp,code[i+1]);
        if (gain)
        {
            L_tmp = L_shl(L_tmp,gain);
        }
        exc2[i] = msu_r(L_tmp,-32768, exc2[i]);
        move16();
    }
    /* exc2[L_subfr-1] = exc2[L_subfr-1] + code[L_subfr-1] - tmp*code[L_subfr-2]; */
    L_tmp = L_mult(code[i], 16384);
    L_tmp = L_msu0(L_tmp,tmp,code[i-1]);
    if (gain)
    {
        L_tmp = L_shl(L_tmp,gain);
    }

    exc2[i] = msu_r(L_tmp,-32768, exc2[i]);
    move16();

    return code_exp;
}


/*-----------------------------------------------------------------------*
 * Phase_dispersion:
 *
 * post-processing to enhance noise in low bit rate.
 *-----------------------------------------------------------------------*/
/*======================================================================================*/
/* FUNCTION : phase_dispersion_fx()														*/
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : post-processing to enhance noise in low bit rate.							*/
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :																	*/
/* _ (Word32) gain_code : gain of code  Q16												*/
/* _ (Word16) gain_pit : gain of pitch	Q14												*/
/* _ (Word16) mode : level, 0=hi, 1=lo, 2=off											*/
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																	*/
/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																*/
/* _ (Word16[]) code : code vector (Q12)												*/
/* _ (struct dispMem_fx*) dm_fx : static memory (size = 8)								*/
/*								 (a[0]->Q0,a[1]->Q16,a[2-7]->Q14)						*/
/*--------------------------------------------------------------------------------------*/

/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																	*/
/* _ None																				*/
/*======================================================================================*/
static void phase_dispersion_fx(
    Word32 gain_code,     /* i  : gain of code              Q16  */
    Word16 gain_pit,      /* i  : gain of pitch            Q14  */
    Word16 code[],        /* i/o: code vector                   */
    Word16 mode,          /* i  : level, 0=hi, 1=lo, 2=off      */
    struct dispMem_fx *dm_fx    /* i/o: static memory (size = 8)      */
)
{
    Word16 i, j, state;
    Word16 *prev_gain_pit, *prev_state;
    Word32 *prev_gain_code;
    Word16 *code2_real, *code2_imag;
    Word16 *code_real, *code_imag;
    const Word16 *h_real, *h_imag;

    Word16 code2[2 * L_SUBFR];

    prev_state = &(dm_fx->prev_state);
    prev_gain_code = &(dm_fx->prev_gain_code);
    prev_gain_pit =  dm_fx->prev_gain_pit;

    state = 2;
    move16();
    if (sub(gain_pit, pitch_0_9) < 0)
    {
        state = 1;
        move16();
    }

    if (sub(gain_pit, pitch_0_6) < 0)
    {
        state = 0;
        move16();
    }

    FOR (i = 5; i > 0; i--)
    {
        prev_gain_pit[i] = prev_gain_pit[i - 1];
        move16();
    }
    prev_gain_pit[0] = gain_pit;
    move16();

    IF (L_sub(L_sub(gain_code, *prev_gain_code), L_shl(*prev_gain_code, 1)) > 0)
    {
        state = s_min(add(state, 1), 2);
    }
    ELSE
    {
        j = 0;
        move16();

        FOR (i = 0; i < 6; i++)
        {
            j = sub(j, shr(sub(prev_gain_pit[i], pitch_0_6), 15));
        }

        if (sub(j, 2) > 0)
        {
            state = 0;
            move16();
        }

        if (sub(sub(state, *prev_state), 1) > 0)
        {
            state = sub(state, 1);
        }
    }

    *prev_gain_code = gain_code;
    move32();
    *prev_state = state;
    move16();

    /*-----------------------------------------------------------------*
     * circular convolution
     *-----------------------------------------------------------------*/

    state = add(state, mode);              /* level of dispersion */

    IF (sub(state, 2) < 0)
    {
        r_fft_fx_lc(phs_tbl_dec, SIZE, SIZE2, NUM_STAGES, code, code2, 1);

        h_real = Mid_H;
        move16();
        if (state == 0)
        {
            h_real = Low_H;
            move16();
        }

        /* FFT Coefs are in code2 */
        code2_real = code2;
        move16();
        code2_imag = code2 + L_SUBFR - 1;
        move16();

        code_real = code;
        move16();
        code_imag = code + L_SUBFR - 1;
        move16();

        h_imag = h_real + L_SUBFR - 1;
        move16();

        *code_real++ = mult(*code2_real++, *h_real++);
        move16();  /* DC */

        FOR (i=1; i<L_SUBFR/2; i++)
        {
            *code_real++ = msu_r(L_mult(*code2_real, *h_real), *code2_imag, *h_imag);
            move16();
            *code_imag-- = mac_r(L_mult(*code2_real, *h_imag), *code2_imag, *h_real);
            move16();

            code2_real++;
            h_imag--;
            h_real++;
            code2_imag--;
        }
        *code_real++ = mult(*code2_real++, *h_real++);
        move16();  /* DC */

        r_fft_fx_lc(phs_tbl_dec, SIZE, SIZE2, NUM_STAGES, code, code2, 0);

        FOR (i = 0; i < L_SUBFR; i++)
        {
            /* saturation can occur here */
            code[i] = shl(code2[i], 1);     /*Q12 */  move16();
        }
    }
}

/*======================================================================================*/
/* FUNCTION : agc2_fx()																	*/
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : AGC post-processing for lower G722.2 modes									*/
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :																	*/
/* _ (Word16*[]) sig_in : postfilter input signal		(Q0)							*/
/* _ (Word16) l_trm : subframe size														*/
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																	*/
/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																*/
/* _ (Word16*[]) sig_out : postfilter output signal		(Q0)							*/
/*--------------------------------------------------------------------------------------*/

/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																	*/
/* _ None																				*/
/*======================================================================================*/
static void agc2_fx(
    const Word16 *sig_in,   /* i  : postfilter input signal  */
    Word16 *sig_out,  /* i/o: postfilter output signal */
    const Word16 l_trm      /* i  : subframe size            */
)
{

    Word16 i, exp;
    Word16 gain_in, gain_out, g0;
    Word32 s;

    Word16 temp;

    /* calculate gain_out with exponent */
    temp = shr(sig_out[0], 2);
    s = L_mult0(temp, temp);
    FOR (i = 1; i < l_trm; i++)
    {
        temp = shr(sig_out[i], 2);
        s = L_mac0(s, temp, temp);
    }
    IF (s != 0)
    {
        exp = sub(norm_l(s), 1);
        gain_out = round_fx(L_shl(s, exp));

        /* calculate gain_in with exponent */
        temp = shr(sig_in[0], 2);
        s = L_mult0(temp, temp);
        FOR (i = 1; i < l_trm; i++)
        {
            temp = shr(sig_in[i], 2);
            s = L_mac0(s, temp, temp);
        }

        g0 = 0;
        move16();
        IF (s != 0)
        {
            i = norm_l(s);
            gain_in = round_fx(L_shl(s, i));
            exp = sub(exp, i);

            /*---------------------------------------------------*
             *  g0 = sqrt(gain_in / gain_out)
             *---------------------------------------------------*/
            s = L_mult0(128, div_s(gain_out, gain_in)); /* s = gain_out / gain_in */
            s = L_shr(s, exp);   /* add exponent */

            s = Isqrt(s);
            g0 = round_fx(L_shl(s, 9));
        }

        /* sig_out(n) = gain(n) sig_out(n) */
        FOR (i = 0; i < l_trm; i++)
        {
            sig_out[i] = round_fx(L_shl(L_mac(-8192, sig_out[i], g0), 2));
        }
    }
}

