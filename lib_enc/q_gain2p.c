/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*-------------------------------------------------------------------------*
 * procedure q_gain2_plus                                                  *
 * ~~~~~~~~~~~~~~~~~~~~~~                                                  *
 * Quantization of pitch and codebook gains.                               *
 * The following routines is Q_gains updated for AMR_WB_PLUS.              *
 * MA prediction is removed and MEAN_ENER is now quantized with 2 bits and *
 * transmitted once every ACELP frame to the gains decoder.                *
 * The pitch gain and the code gain are vector quantized and the           *
 * mean-squared weighted error criterion is used in the quantizer search.  *
 *-------------------------------------------------------------------------*/
#include <assert.h>
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"
#include "rom_com_fx.h"

#define RANGE         64

enum FUNC_GAIN_ENC
{
    FUNC_GAIN_ENC_MLESS = 0,   /* Memory-less gain coding */
    FUNC_GAIN_ENC_2 = 1,       /* AMR-WB gains quantizer (6bits/subfr (mode 4) or 7bits/subfr (mode 5)) */        /* !!! to be removed !!! */
    FUNC_GAIN_ENC_UV,          /* UV gains quantizer (5bits/subfr) */
    FUNC_GAIN_ENC_GACELP_UV    /* UV GACELP gain quantizer ((7=5-2bits/subfr) */
};



void encode_acelp_gains(
    Word16 *code,
    Word16 gains_mode,
    Word16 mean_ener_code,
    Word16 clip_gain,
    ACELP_CbkCorr *g_corr,
    Word16 *gain_pit,
    Word32 *gain_code,
    Word16 **pt_indice,
    Word32 *past_gcode,
    Word16 *gain_inov,
    Word16 L_subfr
    , Word16 *code2,
    Word32 *gain_code2
    ,Word8 noisy_speech_flag /* (i) : noisy speech flag                                            */
)
{
    Word16 index = 0, func_type = 0;

    BASOP_SATURATE_ERROR_ON;

    SWITCH(gains_mode)
    {
    case 1:
    case 2:
    case 3:
    case 10:
        /* Memory-less gain coding */
        gains_mode = sub(gains_mode, 1);
        func_type = FUNC_GAIN_ENC_MLESS;
        move16();
        BREAK;
    case 4:
    case 5:
        assert(0);
        BREAK;
    case 6:
        /* UV gains quantizer (6 bits/subfr) */
        gains_mode = sub(gains_mode, 6);
        func_type = FUNC_GAIN_ENC_UV;
        move16();
        BREAK;
    case 7:
        gains_mode = sub(gains_mode, 7);
        func_type = FUNC_GAIN_ENC_GACELP_UV;
        move16();
        BREAK;
    default:
        fprintf(stderr, "invalid gains coding for acelp!\n");
        assert(0);
        func_type = 0;
        move16(); /*To avoid compiler warning*/
        BREAK;
    }

    IF( func_type == FUNC_GAIN_ENC_MLESS )
    {
        index = gain_enc(code,
                         L_subfr, gain_pit, gain_code, g_corr, mean_ener_code,
                         clip_gain, past_gcode, gain_inov, gains_mode, func_type);
    }
    ELSE
    {
        index = gain_enc_uv(code,
        code2,
        L_subfr, gain_pit, gain_code,
        gain_code2,
        noisy_speech_flag,
        g_corr, mean_ener_code,
        past_gcode, gain_inov,
        func_type);
    }

    move16();
    **pt_indice = index;
    (*pt_indice)++;

    BASOP_SATURATE_ERROR_OFF;
}

/*---------------------------------------------------------------------*
 * procedure gain_enc_mless
 * Quantization of pitch and codebook gains.
 * - an initial predicted gain, gcode0, is first determined based on
 *   the predicted scaled innovation energy
 * - the correction  factor gamma = g_code / gcode0 is then vector quantized
 *   along with gain_pit
 * - the mean-squared weighted error criterion is used for the quantizer search
 *---------------------------------------------------------------------*/

Word16 gain_enc(              /* o   : quantization pitch index                                    <Q0> */
    const Word16 *code,      /* i   : algebraic excitation                                        <Q9> */
    Word16 lcode,            /* (i) : Subframe size in range: 40,64,80                            <Q0> */
    Word16 *gain_pit,        /* o   : quantized pitch gain                                       <Q16> */
    /* i/o : only func=1,coder_type=1 quantized pitch gain              <Q16> */
    Word32 *gain_code,       /* o   : quantized codebook gain                                    <Q16> */
    ACELP_CbkCorr *g_coeff,  /* i   : correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2>  */
    Word16 mean_ener,        /* (i) : only func=0: mean_ener defined in open-loop (3 bits)        <Q8> */
    const Word16 clip_gain,  /* i   : only func=0,1: gain pitch clipping flag (1 = clipping)      <Q0> */
    Word32 *past_gcode,      /* o   : past gain of code                                          <Q16> */
    Word16 *gain_inov,       /* (o) : Q12 innovation gain                                        <Q12> */
    const Word16 coder_type, /* (i) : only func=0,1: coder type                                   <Q0> */
    const Word16 func_type   /* (i) : algorithm: 0=gain_enc_mless, 1=gain_enc_2                   <Q0> */
)
{
    Word16 i, j, index, size, min_index, exp_L_tmp1;
    Word16 gcode0, gcode0_gi, exp_gcode0, exp_sum, exp_code, g_code_shl;

    Word16 g_code;
    Word16 coeff0, coeff1, coeff2, coeff3, coeff4, exp_coeff0, exp_coeff1, exp_coeff2, exp_coeff3, exp_coeff4;
    Word16 shr_coeff0, shr_coeff1, shr_coeff2, shr_coeff3, shr_coeff4;
    const Word16 *p;
    const Word16 *t_qua_gain;
    Word32 L_tmp, dist_min, L_tmp1;



    assert((func_type != FUNC_GAIN_ENC_UV) && (func_type != FUNC_GAIN_ENC_GACELP_UV));

    /* Debug test value (not instrumented) */
    gcode0 = -3000;
    move16();

    /*----------------------------------------------------------------*
     * - calculate the unscaled innovation energy
     * - calculate the predicted gain code
     *----------------------------------------------------------------*/

    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = calc_gain_inov(code, lcode, &L_tmp1, &exp_L_tmp1);
    move16();
    *gain_inov = round_fx(L_shl(L_tmp, 15-3));  /* gain_inov in Q12 */

    /*----------------------------------------------------------------*
     * calculate the predicted gain code
     *----------------------------------------------------------------*/
    IF (func_type == FUNC_GAIN_ENC_MLESS)
    {
        /*j = 10 * log10((dot_product(code, code, lcode) + 0.01) / lcode) */
        j = BASOP_Util_lin2dB(L_tmp1, exp_L_tmp1, 1); /* Q8 */

        /* predicted codebook gain */
        gcode0 = sub(mean_ener, j);            /* Q8 */

    }

    /*----------------------------------------------------------------*
     *  Compute coefficients needed for the quantization.
     *
     *  coeff[0] =    yy1 yy1
     *  coeff[1] = -2 xn yy1
     *  coeff[2] =    y2 y2
     *  coeff[3] = -2 xn y2
     *  coeff[4] =  2 yy1 y2
     *
     * Product <yy1 yy1> and <xn yy1> have been computed in Adpt_enr() and
     * are in vector g_coeff[].
     *----------------------------------------------------------------*/

    coeff0 = g_coeff->y1y1;
    move16();
    exp_coeff0 = g_coeff->y1y1_e;
    move16();
    coeff2 = g_coeff->y2y2;
    move16();
    exp_coeff2 = g_coeff->y2y2_e;
    move16();

    coeff1 = g_coeff->xy1;
    move16();
    exp_coeff1 = add(g_coeff->xy1_e, 1);
    coeff3 = g_coeff->xy2;
    move16();
    exp_coeff3 = add(g_coeff->xy2_e, 1);
    coeff4 = g_coeff->y1y2;
    move16();
    exp_coeff4 = add(g_coeff->y1y2_e, 1);

    /*---------------------------------------------------------------*
     * Decode codebook gain and the adaptive excitation low-pass
     * filtering factor (Finalize computation )
     *---------------------------------------------------------------*/


    /* gcode0 = pow(10, 0.05 * (Es_pred - Ei)) */
    /*----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)                    gcode in Q8
     *        = pow(2, 3.321928*gcode0/20)
     *        = pow(2, 0.166096*gcode0)
     *----------------------------------------------------------------*/

    /* Check if gcode0 was uninitialized. */
    assert(gcode0 != -3000);

    L_tmp = L_mult(gcode0, FL2WORD16(0.166096f));
    exp_gcode0 = add(1,extract_l(L_shr(L_tmp, 24)));
    L_tmp = L_lshl(L_tmp, 7);
    L_tmp = L_and(0x7FFFFFFF, L_tmp);

    L_tmp = Pow2(30,round_fx(L_tmp));
    gcode0 = round_fx(L_tmp);
    /* exponent of gcode0 = exp_gcode0 */

    /*-----------------------------------------------------------------*
     * gain quantization initializations
     * - find the initial quantization pitch index
     * - set the gains searching range
     *----------------------------------------------------------------*/

    /*----------------------------------------------------------------*
     * Find the best quantizer
     *
     * Before doing the computation we need to align exponents of coeff[]
     * to be sure to have the maximum precision.
     *
     * In the table the pitch gains are in Q14, the code gains are in Q11 and
     * are multiplied by gcode0 which have been multiplied by 2^exp_gcode0.
     * Also when we compute g_pitch*g_pitch, g_code*g_code and g_pitch*g_code
     * we divide by 2^15.
     * Considering all the scaling above we have:
     *
     *   exp_code = exp_gcode0 + 4
     *   if (func_type == gain_enc_2)
     *         gcode0 *= gain_inov (in Q12)  => exp_code += 3
     *
     *   g_pitch*g_pitch  = +1+1
     *   g_pitch          = +1
     *   g_code*g_code    = (2*exp_code)
     *   g_code           = exp_code
     *   g_pitch*g_code   = + 1 + exp_code
     *
     *   g_pitch*g_pitch * coeff[0]  ;exp_max0 = exp_coeff[0] + 2
     *   g_pitch         * coeff[1]  ;exp_max1 = exp_coeff[1] + 1
     *   g_code*g_code   * coeff[2]  ;exp_max2 = exp_coeff[2] + (2*exp_code)
     *   g_code          * coeff[3]  ;exp_max3 = exp_coeff[3] + exp_code
     *   g_pitch*g_code  * coeff[4]  ;exp_max4 = exp_coeff[4] + 1 + exp_code
     *----------------------------------------------------------------*/

    exp_code = add(exp_gcode0, 4);

    exp_coeff0 = add(exp_coeff0, 2);
    exp_coeff1 = add(exp_coeff1, 1);
    exp_coeff2 = add(exp_coeff2, shl(exp_code, 1));
    exp_coeff3 = add(exp_coeff3, exp_code);
    exp_coeff4 = add(exp_coeff4, add(1, exp_code));

    /* Find maximum exponent */
    exp_sum = s_max(exp_coeff1, exp_coeff0);
    exp_sum = s_max(exp_coeff2, exp_sum);
    exp_sum = s_max(exp_coeff3, exp_sum);
    exp_sum = s_max(exp_coeff4, exp_sum);
    exp_sum = add(exp_sum,2);

    /* Align exponents of summands in loop far below. */
    shr_coeff0 = sub(exp_sum, exp_coeff0);
    shr_coeff1 = sub(exp_sum, exp_coeff1);
    shr_coeff2 = sub(exp_sum, exp_coeff2);
    shr_coeff3 = sub(exp_sum, exp_coeff3);
    shr_coeff4 = sub(exp_sum, exp_coeff4);
    /* Codebook search */

    dist_min = L_deposit_h(MAX_16);

    min_index = 0;
    move16();

    {
        Word16 size_clip;


        IF( coder_type == 0)
        {

            t_qua_gain = E_ROM_qua_gain5b_const;
            size_clip=9;
            size=NB_QUA_GAIN5B;
        }
        ELSE IF(coder_type == 1)
        {

            t_qua_gain = E_ROM_qua_gain6b_const;
            size_clip=6;
            size = NB_QUA_GAIN6B;                            /* searching range of the gain quantizer */
        }
        ELSE IF(sub(coder_type, 9) == 0)
        {
            t_qua_gain = E_ROM_qua_gain8b_const;
            size_clip=33;
            size = NB_QUA_GAIN8B; /* searching range of the gain quantizer */
        }

        ELSE
        {

            t_qua_gain = E_ROM_qua_gain7b_const;
            size_clip=21;
            size = NB_QUA_GAIN7B;
        }

        if ( sub(clip_gain,1) == 0)
        {
            size = sub(size, size_clip);                     /* limit pitch gain  to 1.0 */
        }
        gcode0_gi = gcode0;
        move16();

    }
    move16();
    p = t_qua_gain;

    index = 0;
    move16();

    /* divide all coeff1,2,3,4 by coeff0 */
    /* in order to skip multiplication with coeff0 in loop */
    assert(coeff0 >= 0x4000);
    coeff0 = div_s(0x4000,coeff0);
    coeff1 = mult_r(coeff1,coeff0);
    coeff2 = mult_r(coeff2,coeff0);
    coeff3 = mult_r(coeff3,coeff0);
    coeff4 = mult_r(coeff4,coeff0);

    FOR (i = 0; i < size; i++)
    {
        /*
           Note: gcode0_gi: either gcode0 or gcode0*gain_inov
           g_pitch = *p++;
           g_code = gcode0_gi * *p++;

           dist = g_pitch*g_pitch * coeff.y1y1
           + g_pitch        * coeff.xy1 (negated)
           + g_code*g_code  * coeff.y2y2
           + g_code         * coeff.xy2 (negated)
           + g_pitch*g_code * coeff.y1y2;
        */

        /* Since g_code has a significant dynamic, we prefer to normalize this 16-bit value */
        g_code_shl = norm_s(p[2*i+1]);
        g_code = shl(p[2*i+1],g_code_shl);
        g_code = mult_r(g_code, gcode0_gi);
        BASOP_SATURATE_WARNING_OFF  /* needed to skip overflow warnings due to exceeding shift values */
        L_tmp = L_shr(Mpy_32_16_1(L_mult(g_code, g_code),coeff2),shr_coeff2);
        if (g_code_shl != 0)
            L_tmp =           L_shr(L_tmp,g_code_shl);
        L_tmp = L_sub(L_tmp,L_shr(L_mult(g_code,                   coeff3),shr_coeff3));
        L_tmp = L_add(L_tmp,L_shr(Mpy_32_16_1(L_mult(g_code, p[2*i+0]), coeff4),shr_coeff4));
        if (g_code_shl != 0)
            L_tmp = L_shr(L_tmp,g_code_shl);
        /* Here, we use L_mult0 to compensate the factor 0.5 applied to coeff[1..4] before */
        L_tmp = L_add(L_tmp,L_shr(L_mult0(p[2*i+0],p[2*i+0]),              shr_coeff0));
        L_tmp = L_sub(L_tmp,L_shr(L_mult(p[2*i+0],                 coeff1),shr_coeff1));
        L_tmp1= L_sub(L_tmp, dist_min);
        BASOP_SATURATE_WARNING_ON
        if (L_tmp1 < 0)
        {
            index = i;
            move16();
        }
        if (L_tmp1 < 0)
        {
            dist_min = L_min(L_tmp, dist_min);
        }
    }
    index = add(index, min_index);
    *gain_pit = t_qua_gain[2*index+0];
    move16();
    g_code = t_qua_gain[2*index+1];
    move16();

    L_tmp = L_mult(g_code, gcode0);            /* Q11*Q15 -> Q27 */
    exp_gcode0 = add(exp_gcode0,-11);
    L_tmp = L_shl(L_tmp, exp_gcode0);       /*   Q27 -> Q16 */


    *gain_code = L_tmp;
    move32();
    /* Q16/Q12 => Q5 */
    L_tmp = L_deposit_h(BASOP_Util_Divide3216_Scale(L_tmp,*gain_inov,&i));
    *past_gcode = L_shl(L_tmp,sub(i,15-12));


    return index;
}

Word16 gain_enc_uv(           /* o   : quantization pitch index                                    <Q0> */
    const Word16 *code,      /* i   : algebraic excitation                                        <Q9> */
    const Word16 *code2,     /* i   : gaussian excitation                                         <Q9> */
    Word16 lcode,            /* (i) : Subframe size in range: 40,64,80                            <Q0> */
    Word16 *gain_pit,        /* o   : quantized pitch gain                                       <Q16> */
    Word32 *gain_code,       /* o   : quantized codebook gain                                    <Q16> */
    Word32 *gain_code2,      /* o   : quantized codebook gain                                    <Q16> */
    Word8 noisy_speech_flag, /* (i) : noisy speech flag                                                */
    ACELP_CbkCorr *g_coeff,  /* i   : correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2>  */
    Word16 mean_ener,        /* (i) : only func=0: mean_ener defined in open-loop (3 bits)        <Q8> */
    Word32 *past_gcode,      /* o   : past gain of code                                          <Q16> */
    Word16 *gain_inov,       /* (o) : Q12 innovation gain                                        <Q12> */
    const Word16 func_type   /* (i) : algorithm: 2=gain_enc_uv, 3=gain_enc_gacelp_uv              <Q0> */
)
{
    Word16 i, index, exp_L_tmp1, tmp;
    Word16 exp_gcode;
    Word16 g_code;
    Word32 L_tmp, L_tmp1;
    Word8 gacelp_uv;
    Word32 pred_nrg_frame;
    Word16 exp_gcode2, g_code2, norm_code2;
    Word16 c, c_e, c_index2, c_index2_e, c_first, c_first_e;
    Word16 s, tmp1, s1;
    Word16 index2;
    const Word16 log2_scale=16;

    pred_nrg_frame = 0;     /* to suppress compilation warnings */
    g_code2 = 0;            /* to suppress compilation warnings */
    exp_gcode2 = 0;         /* to suppress compilation warnings */


    assert((func_type != FUNC_GAIN_ENC_MLESS) );

    /* Debug check value (not instrumented) */
    index2 = -3000;
    move16();

    gacelp_uv = 0;
    move16();
    if (sub(func_type, FUNC_GAIN_ENC_GACELP_UV) == 0)
    {
        gacelp_uv = 1;
        move16();
    }

    /*----------------------------------------------------------------*
     * - calculate the unscaled innovation energy
     * - calculate the predicted gain code
     *----------------------------------------------------------------*/

    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = calc_gain_inov(code, lcode, NULL, NULL);
    *gain_inov = round_fx(L_shl(L_tmp, 15-3));  /* gain_inov in Q12 */

    /*----------------------------------------------------------------*
     * calculate the predicted gain code
     *----------------------------------------------------------------*/
    IF (gacelp_uv != 0)
    {
        /* pred_nrg_frame = (float)pow(10.0,mean_ener/20.0); */
        L_tmp = L_mult(mean_ener, FL2WORD16(0.166096f * 2)); /* 6Q25 */
        pred_nrg_frame = BASOP_Util_InvLog2(L_sub(L_tmp, FL2WORD32_SCALE(15.f, 6))); /* 15Q16 */

        /* gcode = pred_nrg_frame * (*gain_inov); */
        L_tmp = Mpy_32_16_1(pred_nrg_frame, *gain_inov); /* 18Q13 */
        i = norm_l(L_tmp);
        g_code = round_fx(L_shl(L_tmp, i));
        exp_gcode = sub(18, i);

        /* norm_code2 = 1.0f / sqrt((dot_product(code2, code2, lcode) + 0.01f) / lcode); */
        L_tmp = calc_gain_inov(code2, lcode, NULL, NULL);
        norm_code2 = round_fx(L_shl(L_tmp, 15-3));  /* Q12 */

        /* g_code2 = pred_nrg_frame * norm_code2; */
        L_tmp = Mpy_32_16_1(pred_nrg_frame, norm_code2); /* 18Q13 */
        i = norm_l(L_tmp);
        g_code2 = round_fx(L_shl(L_tmp, i));
        exp_gcode2 = sub(18, i);
    }
    ELSE
    {
        g_code = *gain_inov;
        move16();
        exp_gcode = 3;
        move16();
    }

    tmp = BASOP_Util_Divide1616_Scale(g_coeff->xy2, mult_r(g_coeff->y2y2, g_code), &i);     /*Correlation based*/
    L_tmp = L_shl( L_deposit_h(tmp), add(i, sub(g_coeff->xy2_e, add(g_coeff->y2y2_e, add(exp_gcode, log2_scale)))) );
    /* exponent of L_tmp is 16, accounted below by adding log2(2^16) */

    index = 0;
    move16();

    IF (L_tmp > 0)
    {
        /*index = (int)(((20.f*log10(g_code)+30.f)/1.9f)+0.5f))); */
        /* Since ((20*log10(x)+30)/1.9)+0.5 = 63 (max index) implies x is between 2^15 and 2^16,
           L_tmp might saturate at 65535 and above. That is why log2_scale is 16. */
        tmp = BASOP_Util_lin2dB(L_tmp, 16, 0); /* Q8 */

        IF (gacelp_uv != 0)
        {
            L_tmp = L_mult(add(tmp, FL2WORD16_SCALE(20.0f, 7)), FL2WORD16(1.0f/1.25f));
        }
        ELSE
        {
            L_tmp = L_mult(add(tmp, FL2WORD16_SCALE(30.0f, 7)), FL2WORD16(1.0f/1.9f));
        }

        index = round_fx(L_shr(L_tmp, 8));
        index = s_max(0, s_min(63, index));
        if (gacelp_uv != 0) index = s_min(31, index);
    }

    /* *gain_code= (float) pow(10.f,(((index*1.9f)-30.f)/20.f)); */

    /*----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     *        = pow(2, 3.321928*gcode0/20)
     *        = pow(2, 0.166096*gcode0)
     *----------------------------------------------------------------*/
    IF (gacelp_uv != 0)
    {
        L_tmp = L_mac(FL2WORD32_SCALE(-0.166096*20.0f, 7-1),shl(index, 16-7), FL2WORD16(0.166096f*1.25f));
    }
    ELSE
    {
        L_tmp = L_mac(FL2WORD32_SCALE(-0.166096*30.0f, 7-1),shl(index, 16-7), FL2WORD16(0.166096f*1.9f));
    }
    i = add(1,extract_l(L_shr(L_tmp, 25)));
    L_tmp = L_lshl(L_tmp, 6);
    L_tmp = L_and(0x7FFFFFFF, L_tmp);

    L_tmp = Pow2(30,round_fx(L_tmp));
    L_tmp = L_shl(L_tmp, i-(31-16)); /* Q16 */

    IF (gacelp_uv != 0)
    {
        /* *past_gcode = L_tmp * pred_nrg_frame; */
        i = norm_l(L_tmp);
        L_tmp1 = L_shl(L_tmp, i);
        exp_L_tmp1 = sub(15, i);

        i = norm_l(pred_nrg_frame);
        L_tmp1 = Mpy_32_32(L_tmp1, L_shl(pred_nrg_frame, i));
        exp_L_tmp1 = add(exp_L_tmp1, sub(15, i));

        *past_gcode = L_shl(L_tmp1, sub(exp_L_tmp1, 15)); /* Q16 */                                   move32();
    }
    ELSE
    {
        *past_gcode = L_tmp;  /*unscaled gain*/                                                       move32();
    }


    *gain_code = L_shl(Mpy_32_16_1(*past_gcode, *gain_inov), 3);
    move32();

    *gain_pit = 0;
    move16();

    IF (gacelp_uv != 0)
    {
        /* c_first = 0.8f*g_coeff->xx - (*gain_code) * (*gain_code) * g_coeff->y2y2; */
        /* c_first = g_coeff->xx - (*gain_code) * (*gain_code) * g_coeff->y2y2; */
        tmp = g_coeff->xx;
        move16();
        if (noisy_speech_flag != 0)
        {
            tmp = mult_r(FL2WORD16(0.8f), tmp);
        }

        s1 = norm_l(*gain_code);
        tmp1 = round_fx(L_shl(*gain_code, s1));
        s1 = sub(15, s1);
        tmp1 = mult_r(mult_r(tmp1, tmp1), g_coeff->y2y2);

        c_first_e = BASOP_Util_Add_MantExp(tmp, g_coeff->xx_e,
                                           negate(tmp1), add(g_coeff->y2y2_e, shl(s1, 1)),
                                           &c_first);

        L_tmp = Mpy_32_16_1(*gain_code, BASOP_Util_Divide1616_Scale(g_code2, g_code, &s));
        L_tmp = L_shl(L_tmp, sub(sub(add(s, exp_gcode2), exp_gcode), 2)); /* Q16 */
        L_tmp1 = L_add(L_tmp, 0);

        s1 = norm_l(*gain_code);
        tmp1 = round_fx(L_shl(*gain_code, s1));
        s1 = sub(15, s1);

        c_index2 = 0x7FFF;
        move16();
        c_index2_e = 127;
        move16();
        FOR (i = 0; i < 4; i++)
        {
            /* c = c_first - L_tmp1 * (L_tmp1 * g_coeff->y1y1 + 2 * (*gain_code) * g_coeff->y1y2); */
            s = norm_l(L_tmp1);
            tmp = round_fx(L_shl(L_tmp1, s));
            s = sub(15, s);

            c_e = BASOP_Util_Add_MantExp(mult_r(tmp, g_coeff->y1y1), add(s, g_coeff->y1y1_e),
                                         mult_r(tmp1, g_coeff->y1y2), add(add(s1, g_coeff->y1y2_e), 1),
                                         &c);
            c = mult_r(c, tmp);
            c_e = add(c_e, s);
            c_e = BASOP_Util_Add_MantExp(c_first, c_first_e, negate(c), c_e, &c);

            tmp = 0;
            move16();
            if (sub(c_e, c_index2_e) < 0)
            {
                tmp = 1;
                move16();
            }
            test();
            if (sub(c_e, c_index2_e) == 0 && sub(abs_s(c), abs_s(c_index2)) < 0)
            {
                tmp = 1;
                move16();
            }

            IF (tmp != 0)
            {
                index2 = i;
                move16();
                c_index2 = c;
                move16();
                c_index2_e = c_e;
                move16();
                *gain_code2 = L_tmp1;
                move32();
            }

            L_tmp1 = L_add(L_tmp1, L_tmp);
        }

        /* check if value was uninitialized */
        assert(index2 != -3000);
        index = add(index, shl(index2, 5));
    }


    return index;
}

