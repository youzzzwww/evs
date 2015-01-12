/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define RANGE          64
#define NB_QUA_GAIN7B 128     /* Number of quantization levels */

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/
static Word16 Find_Opt_gainQ_fx(Word16 *coeff, Word16 *exp_coeff, Word16 *gain_pit, Word32 *gain_code,
                                Word16 gcode0,Word16 exp_gcode0,const Word16 *cdbk, const Word16 size );
/*==========================================================================*/
/* FUNCTION      :  Es_pred_enc_fx()                                        */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Calculation and quantization of average predicted innovation energy to be*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/*   _ Word16 L_frame,     i  : length of the frame       Q0                */
/*   _ Word16 *res,        i  : residual signal           Q_new           */
/*   _ Word16 *voicing,    i  : normalized correlation in three 1/2frames Q15*/
/*   _ Word16 coder_type,  i  : coder_type                Q0       			*/
/*   _ Word16 bwidth,      i  : input signal bandwidth    Q0       			*/
/*   _ Word32  core_brate, i  : core bitrate              Q0       			*/
/*   _ Word16 Q_new        i  : Scaling in speech         Q0     			*/
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*   _ Word16 *Es_pred,    o  : predicited scaled innovation energy    Q8   */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*   _ None.                                                                */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                               */
/*--------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                      */
/*==========================================================================*/
void Es_pred_enc_fx(
    Word16 *Es_pred,      /* o  : predicited scaled innovation energy Q8    */
    Word16 *indice,      /* o  : indice of quantization    */
    const Word16 L_frame,      /* i  : length of the frame                       */
    const Word16 *res,         /* i  : residual signal                           */
    const Word16 *voicing,     /* i  : normalized correlation in three 1/2frames */
    const Word16 nb_bits,      /* i  : allocated number of bits                  */
    const Word16 no_ltp,       /* i  : no_ltp flag                               */
    Word16 Q_new         /* i  : Scaling in speech                   Q0    */
)
{
    Word16 i, i_subfr,size, tmp16, tmp16_2, Q_res;
    Word16 weight;
    Word16 s0,s1, ener_dB, mean_ener_code16;
    const Word16 *qua_table;
    Word32 ener_fx, Lmean_ener_code, Ltmp;

    Lmean_ener_code = L_deposit_l(0);
    Q_res = sub(shl(Q_new, 1), 3);

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        weight = 8192;
        move16();/*0.25f in Q15*/
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        weight = 6554;
        move16();/*0.2f in Q15*/
    }

    /*----------------------------------------------------------*
     * calculate the average residual signal energy in four sframes
     *----------------------------------------------------------*/

    FOR (i_subfr = 0; i_subfr < L_frame; i_subfr += L_SUBFR)
    {
        /* calculate the energy of residual signal */
        tmp16 = mult_r(res[i_subfr+0], 8192); /* remove 2bits */
        ener_fx = L_mult(tmp16, tmp16);
        FOR (i=1; i<L_SUBFR; i++)
        {
            tmp16 = mult_r(res[i_subfr+i], 8192); /* remove 2bits */
            ener_fx = L_mac(ener_fx,tmp16, tmp16);
        }

        /* ener = 10 * (float)log10(ener / (float)L_SUBFR) */
        s1 = 0;
        move16();
        s0 = norm_l(ener_fx);

        IF (ener_fx != 0) /* Log2_norm_lc doesn't Support Input <= 0; deal with it here */
        {
            s1 = Log2_norm_lc(L_shl(ener_fx, s0));
            s0 = sub(30, s0);
        }
        s0 = sub(s0, add(Q_res,6));
        Ltmp = Mpy_32_16(s0, s1, LG10);
        ener_dB = extract_l(L_shr(Ltmp, 14-8)); /* Q8 Energy in log10 */
        test();
        if ((ener_dB < 0) && (no_ltp==0))
        {
            ener_dB = 0;
        }

        /* update the average energy of residual signal */
        Lmean_ener_code = L_mac(Lmean_ener_code,ener_dB, weight); /* Q24 */
    }

    IF(no_ltp==0)
    {
        /*----------------------------------------------------------*
        * subtract an estimate of adaptive codebook contribution
        *----------------------------------------------------------*/
        /*mean_ener_code -= 10.0f * (0.5f * voicing[0] + 0.5f * voicing[1]);*/
        Lmean_ener_code = L_msu(Lmean_ener_code, voicing[0], 1280);/*Q24*/
        Lmean_ener_code = L_msu(Lmean_ener_code, voicing[1], 1280);/*Q24*/
        mean_ener_code16 = extract_h(Lmean_ener_code);/*Q8*/

        /*----------------------------------------------------------*n
         * quantize the average predicted innovation energy
         *----------------------------------------------------------*/
        SWITCH ( nb_bits )
        {
        case 5:
            {
                qua_table =  Es_pred_qua_5b_fx;
                BREAK;
            }
        case 4:
            {
                qua_table =  Es_pred_qua_4b_fx;
                BREAK;
            }
        case 3:
            {
                qua_table =  Es_pred_qua_3b_fx;
                BREAK;
            }
        default:
            {
                qua_table =  Es_pred_qua_5b_fx;
                BREAK;
            }
        }
    }
    ELSE
    {
        mean_ener_code16 = extract_h(Lmean_ener_code);/*Q8*/

        qua_table =  Es_pred_qua_4b_no_ltp_fx;
    }
    size = extract_l(pow2_fx[nb_bits]); /*maximum number of bit is 6 */

    /* find the nearest neighbour (codevector) */
    *Es_pred = qua_table[0];
    move16();
    tmp16 = abs_s(sub(mean_ener_code16, qua_table[0]));
    *indice = 0;
    move16();

    FOR (i=1; i<size; i++)
    {
        tmp16_2 = abs_s(sub(mean_ener_code16, qua_table[i]));
        IF (sub(tmp16_2, tmp16) < 0)
        {
            tmp16 = tmp16_2;
            move16();
            *indice = i;
            move16();
            *Es_pred = qua_table[i];
            move16();
        }
    }


    return;
}
/*---------------------------------------------------------------------*
  * gain_enc_mless()
  *
  * Quantization of pitch and codebook gains without prediction (memory-less)
  * - an initial predicted gain, gcode0, is first determined based on
  *   the predicted average innovation energy
  * - a correction factor gamma = g_code / gcode0 is then vector quantized along with gain_pit
  * - the mean-squared weighted error criterion is used for codebook search
  *---------------------------------------------------------------------*/
void gain_enc_mless_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word32  core_brate,        /* i  : core bitrate                                                    */
    const Word16 L_frame,           /* i  : length of the frame                                             */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 tc_subfr,          /* i  : TC subframe index                                               */
    const Word16 *xn,               /* i  : target vector                                                   */
    const Word16 *y1,               /* i  : zero-memory filtered adaptive excitation                        */
    const Word16  Q_xn,             /* i  : xn and y1 scaling                                               */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation              */
    const Word16 *code,             /* i  : algebraic excitation                                            */
    const Word16 Es_pred,           /* i  : predicted scaled innovation energy                              */
    Word16 *gain_pit,         /* o  : quantized pitch gain                                            */
    Word32 *gain_code,        /* o  : quantized codebook gain                                         */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                           */
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain          /* i  : gain pitch clipping flag (1 = clipping)                         */

)
{

    Word16 index, size, nBits, nBits2;
    Word16 gcode0, Ei, gain_code16;
    const Word16 *qua_table;
    Word16 coeff[5], exp_coeff[5];
    Word16 exp, exp_code, exp_inov, exp_gcode0, frac, tmp;
    Word32 L_tmp, L_tmp1, L_tmp2;
    Word16 tmp1, expg;
    Word16 exp1, exp2;
    Word16 exp_num, exp_den, exp_div, frac_den;
    Word32 L_frac_num, L_frac_den, L_div;

    /*-----------------------------------------------------------------*
     * calculate the rest of the correlation coefficients
     * c2 = <y2,y2>, c3 = -2<xn,y2>, c4 = 2<y1,y2>
     *-----------------------------------------------------------------*/

    coeff[0] = g_corr[0];
    move16();
    exp_coeff[0] = g_corr[1];
    move16();
    coeff[1] = negate(g_corr[2]);
    move16();  /* coeff[1] = -2 xn yy1 */
    exp_coeff[1] = add(g_corr[3], 1);
    move16();

    /* Compute scalar product <y2[],y2[]> */
    coeff[2] = extract_h(Dot_product12(y2, y2, L_SUBFR, &exp));
    exp_coeff[2] = add(sub(exp, 18), shl(Q_xn, 1));
    move16(); /* -18 (y2 Q9) */

    /* Compute scalar product -2*<xn[],y2[]> */
    coeff[3] = extract_h(L_negate(Dot_product12(xn, y2, L_SUBFR, &exp)));
    exp_coeff[3] = add(sub(exp, 9 - 1), Q_xn);
    move16(); /* -9 (y2 Q9), +1 (2 xn y2) */

    /* Compute scalar product 2*<y1[],y2[]> */
    coeff[4] = extract_h(Dot_product12(y1, y2, L_SUBFR, &exp));
    exp_coeff[4] = add(sub(exp, 9 - 1), Q_xn);
    move16(); /* -9 (y2 Q9), +1 (2 y1 y2) */

    /*-----------------------------------------------------------------*
     * calculate the unscaled innovation energy
     * calculate the predicted gain code
     *-----------------------------------------------------------------*/

    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = Dot_product12(code, code, L_SUBFR, &exp_code);
    exp_inov = sub(exp_code, 18+6);
    exp_code = sub(exp_code, 30);

    /*Ei = 10 * log10((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */

    /*----------------------------------------------------------------*
     * calculate the predicted gain code
     *----------------------------------------------------------------*/
    tmp = norm_l(L_tmp);
    frac = Log2_norm_lc(L_shl(L_tmp, tmp));
    tmp = add(30-18-6-1, sub(exp_code, tmp));  /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    L_tmp1 = Mpy_32_16(tmp, frac, 12330); /* Q13 */
    Ei = round_fx(L_shl(L_tmp1, 11));     /* Q8 */

    /* predicted codebook gain */
    gcode0 = sub(Es_pred, Ei);            /* Q8 */

    /*---------------------------------------------------------------*
     * Decode codebook gain and the adaptive excitation low-pass
     * filtering factor (Finalize computation )
     *---------------------------------------------------------------*/
    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = Isqrt_lc(L_tmp, &exp_inov);
    *gain_inov = extract_h(L_shl(L_tmp, sub(exp_inov, 3)));  /* gain_inov in Q12 */

    /* gcode0 = pow(10, 0.05 * (Es_pred - Ei)) */
    /*----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     *        = pow(2, 3.321928*gcode0/20)
     *        = pow(2, 0.166096*gcode0)
     *----------------------------------------------------------------*/

    L_tmp = L_mult(gcode0, 21771);            /* *0.166096 in Q17 -> Q26    */
    L_tmp = L_shr(L_tmp, 10);                 /* From Q26 to Q16            */
    frac = L_Extract_lc(L_tmp, &exp_gcode0);  /* Extract exponent of gcode0 */

    gcode0 = extract_l(Pow2(14, frac));    /* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767   */
    exp_gcode0 = sub(exp_gcode0, 14);

    /*-----------------------------------------------------------------*
     * select the codebook, size and number of bits
     * set the gains searching range
     *-----------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr)) ];
        move16();
    }
    test();
    test();
    test();
    test();
    test();
    IF( (sub(tc_subfr,3*L_SUBFR) == 0 && sub(i_subfr,3*L_SUBFR) == 0 && sub(L_frame,L_FRAME) == 0) ||
        (sub(tc_subfr,4*L_SUBFR) == 0 && sub(i_subfr,4*L_SUBFR) == 0 && sub(L_frame,L_FRAME16k) == 0) )
    {
        /*    *gain_pit =  (g_corr[2]*tmp2) - (0.5f*g_corr[4]*tmp3);
                        =  ((-0.5f*g_corr[1]*g_corr[2]) - (-0.25*g_corr[3]*g_corr[4]))/tmp1;
                        =  ((0.25*g_corr[3]*g_corr[4]) - (0.5*g_corr[1]*g_corr[2]))/tmp1; */

        /*     *gain_code = (g_corr[0]*tmp3) - (0.5f*g_corr[4]*tmp2);
                          = ((-0.5*g_corr[3]*g_corr[0]) - (-0.25*g_corr[1]*g_corr[4]))/tmp1;
                          = ((0.25*g_corr[1]*g_corr[4]) - (0.5*g_corr[0]*g_corr[3]))/tmp1; */

        L_tmp1 = L_mult(coeff[0],coeff[2]); /*Q31*/
        exp1 = add(exp_coeff[0], exp_coeff[2]);

        L_tmp2 = L_shr(L_mult(coeff[4],coeff[4]),2); /*Q31*/
        exp2 = add(exp_coeff[4], exp_coeff[4]);

        IF(sub(exp1,exp2)>0)
        {
            L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
            exp_den = exp1;
            move16();
        }
        ELSE
        {
            L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
            exp_den = exp2;
            move16();
        }
        L_frac_den = L_sub(L_tmp1,L_tmp2); /*Q31*/

        frac_den = extract_h(L_frac_den);
        frac_den  = s_max(frac_den,1);
        L_frac_den  = L_max(L_frac_den,1);
        exp = norm_l(L_frac_den);
        tmp = div_s(shl(1,sub(14,exp)),frac_den); /*Q(14-exp)*/

        L_tmp1 = L_shr(L_mult(coeff[3],coeff[4]),2); /*Q31*/
        exp1 = add(exp_coeff[3], exp_coeff[4]);

        L_tmp2 = L_shr(L_mult(coeff[1],coeff[2]),1); /*Q31*/
        exp2 = add(exp_coeff[1], exp_coeff[2]);

        IF(sub(exp1,exp2)>0)
        {
            L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
            exp_num = exp1;
            move16();
        }
        ELSE
        {
            L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
            exp_num = exp2;
            move16();
        }
        L_frac_num = L_sub(L_tmp1,L_tmp2); /*Q31*/

        L_div = Mult_32_16(L_frac_num,tmp); /*Q(30-exp)*/
        exp_div = sub(exp_num,exp_den);

        *gain_pit = round_fx(L_shl(L_div,add(exp,exp_div))); /*Q14*/

        L_tmp1 = L_shr(L_mult(coeff[1],coeff[4]),2); /*Q31*/
        exp1 = add(exp_coeff[1], exp_coeff[4]);

        L_tmp2 = L_shr(L_mult(coeff[0],coeff[3]),1); /*Q31*/
        exp2 = add(exp_coeff[0], exp_coeff[3]);

        IF(sub(exp1,exp2)>0)
        {
            L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
            exp_num = exp1;
        }
        ELSE
        {
            L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
            exp_num = exp2;
        }
        L_frac_num = L_sub(L_tmp1,L_tmp2); /*Q31*/

        L_div = Mult_32_16(L_frac_num,tmp); /*Q(30-exp)*/
        exp_div = sub(exp_num,exp_den);

        *gain_code = L_shl(L_div,sub(add(exp,exp_div),14));
        move32();/*Q16*/

        /* set number of bits for two SQs */
        nBits2 = shr(add(nBits,1),1);
        nBits = shr(nBits,1);

        /* gain_pit Q */

        tmp1 = mult_r(G_PITCH_MAX_MINUS_MIN_TC192_Q13,div_s(1,sub(shl(1, nBits), 1)));   /*Q13*/   /* set quantization step */
        index = usquant_fx( *gain_pit, gain_pit, G_PITCH_MIN_TC192_Q14, tmp1, shl(1, nBits) );
        move16();
        push_indice_fx( st_fx, IND_GAIN_PIT, index, nBits );

        /* gain_code Q */
        /**gain_code /= gcode0;*/
        IF(gcode0 != 0)
        {
            tmp = div_s(16384,gcode0); /*Q15*/
            L_tmp = Mult_32_16(*gain_code,tmp); /*Q16*/
            *gain_code = L_shr(L_tmp,add(14,exp_gcode0)); /*Q16*/
        }

        index = gain_quant_fx( gain_code, &gain_code16, LG10_G_CODE_MIN_TC192_Q14, LG10_G_CODE_MAX_TC192_Q13, nBits2, &expg );
        push_indice_fx( st_fx, IND_GAIN_CODE, index, nBits2 );
        L_tmp = L_mult(gain_code16,gcode0); /*Q0*Q0 -> Q1*/
        *gain_code = L_shl(L_tmp,add(add(expg,exp_gcode0),15)); /*Q16*/
    }
    ELSE
    {
        size = extract_l(pow2[nBits]);

        SWITCH ( nBits )
        {
        case 7:
            {
                qua_table = gain_qua_mless_7b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,30);
                BREAK;
            }
        case 6:
            {
                qua_table = gain_qua_mless_6b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,14);
                BREAK;
            }
        case 5:
            {
                qua_table = gain_qua_mless_5b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,6);
                BREAK;
            }
        default:
            {
                qua_table = gain_qua_mless_6b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,14);
                BREAK;
            }
        }

        /* in case of AVQ inactive, limit the gain_pit to 0.65 */
        test();
        IF( sub(clip_gain,2) == 0 && sub(nBits,6) == 0 )
        {
            size = sub(size,36);
            nBits = sub(nBits,1);
        }

        /*-----------------------------------------------------------------*
         * search for the best quantizer
         *-----------------------------------------------------------------*/
        index = Find_Opt_gainQ_fx(coeff, exp_coeff, gain_pit, gain_code, gcode0, exp_gcode0, qua_table, size);
        push_indice_fx( st_fx, IND_GAIN, index, nBits );
    }

    /* *norm_gain_code = *gain_code / *gain_inov; */
    exp = sub(norm_s(*gain_inov),1);
    exp = s_max(exp, 0);

    tmp = div_s(shr(8192,exp),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code, tmp),sub(1,exp));
    move32();

    return;
}


/*---------------------------------------------------------------------*
* gain_enc_SQ()
*
* Scalar Quantization of pitch and codebook gains without prediction
* - an initial predicted gain, gcode0, is first determined based on
*   the predicted scaled innovation energy
* - a correction factor gamma = g_code / gcode0 is then vector quantized
*   along with gain_pit
* - the mean-squared weighted error criterion is used for codebook search
*---------------------------------------------------------------------*/

void gain_enc_SQ_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure      */
    const Word32 core_brate,        /* i  : core bitrate                                                    */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 tc_subfr,          /* i  : TC subframe index                                               */
    const Word16 *xn,               /* i  : target vector                                                  Q_xn */
    const Word16 *yy1,              /* i  : zero-memory filtered adaptive excitation                       Q_xn */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation             Q9 */
    const Word16 *code,             /* i  : algebraic excitation                                           Q9 */
    const Word16 Es_pred,           /* i  : predicted scaled innovation energy                             Q8 */
    Word16 *gain_pit,         /* o  : quantized pitch gain                                           Q14 */
    Word32 *gain_code,        /* o  : quantized codebook gain                                        Q16 */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                Q12 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                          Q16 */
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, <xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain,         /* i  : gain pitch clipping flag (1 = clipping)                         */
    const Word16  Q_xn              /* i  : xn and y1 scaling                                               */
)
{
    Word16 index, nBits_pitch, nBits_code;
    Word16 gcode0, Ei, gain_code16;
    Word16 coeff[5], exp_coeff[5];
    Word16 exp, exp_code, exp_inov, exp_gcode0, frac, tmp;

    Word32 L_tmp, L_tmp1, L_tmp2;
    Word16 tmp1, expg;
    Word16 exp1, exp2;
    Word16 exp_num, exp_den, exp_div, frac_den;
    Word32 L_frac_num, L_frac_den, L_div;

    /*-----------------------------------------------------------------*
     * calculate the rest of the correlation coefficients
     * c2 = <y2,y2>, c3 = -2<xn,y2>, c4 = 2<y1,y2>
     *-----------------------------------------------------------------*/
    /*g_corr[1] *= -0.5;*/
    /*g_corr[2] = dotp( y2, y2, L_SUBFR )  + 0.01f;*/
    /*g_corr[3] = dotp( xn, y2, L_SUBFR )  - 0.02f;*/
    /*g_corr[4] = dotp( yy1, y2, L_SUBFR ) + 0.02f;*/

    coeff[0] = g_corr[0];
    move16();
    exp_coeff[0] = g_corr[1];
    move16();
    coeff[1] = g_corr[2];
    move16();  /* coeff[1] = xn yy1 */
    exp_coeff[1] = g_corr[3];
    move16();

    /* Compute scalar product <y2[],y2[]> */
    coeff[2] = extract_h(Dot_product12(y2, y2, L_SUBFR, &exp));
    exp_coeff[2] = add(sub(exp, 18), shl(Q_xn, 1));
    move16(); /* -18 (y2 Q9) */

    /* Compute scalar product <xn[],y2[]> */
    coeff[3] = extract_h(Dot_product12(xn, y2, L_SUBFR, &exp));
    exp_coeff[3] = add(sub(exp, 9 ), Q_xn);
    move16(); /* -9 (y2 Q9), (xn y2) */

    /* Compute scalar product <y1[],y2[]> */
    coeff[4] = extract_h(Dot_product12(yy1, y2, L_SUBFR, &exp));
    exp_coeff[4] = add(sub(exp, 9), Q_xn);
    move16(); /* -9 (y2 Q9), (y1 y2) */

    /*-----------------------------------------------------------------*
     * calculate the unscaled innovation energy
     * calculate the predicted gain code
     * calculate optimal gains
     *-----------------------------------------------------------------*/
    /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;*/
    /**gain_inov = 1.0f / (float)sqrt( Ecode );*/

    L_tmp = Dot_product12(code, code, L_SUBFR, &exp_code);
    exp_inov = sub(exp_code, 18+6);
    exp_code = sub(exp_code, 30);

    /*Ei = 10 * log10((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    /*----------------------------------------------------------------*
     * calculate the predicted gain code
     *----------------------------------------------------------------*/
    tmp = norm_l(L_tmp);
    frac = Log2_norm_lc(L_shl(L_tmp, tmp));
    tmp = add(30-18-6-1, sub(exp_code, tmp));  /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    L_tmp1 = Mpy_32_16(tmp, frac, 12330); /* Q13 */
    Ei = round_fx(L_shl(L_tmp1, 11));     /* Q8 */

    /* predicted codebook gain */
    gcode0 = sub(Es_pred, Ei);            /* Q8 */

    /*---------------------------------------------------------------*
     * Decode codebook gain and the adaptive excitation low-pass
     * filtering factor (Finalize computation )
     *---------------------------------------------------------------*/
    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = Isqrt_lc(L_tmp, &exp_inov);
    *gain_inov = extract_h(L_shl(L_tmp, sub(exp_inov, 3)));  /* gain_inov in Q12 */

    /* gcode0 = pow(10, 0.05 * (Es_pred - Ei)) */
    /*----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     *        = pow(2, 3.321928*gcode0/20)
     *        = pow(2, 0.166096*gcode0)
     *----------------------------------------------------------------*/

    L_tmp = L_mult(gcode0, 21771);            /* *0.166096 in Q17 -> Q26    */
    L_tmp = L_shr(L_tmp, 10);                 /* From Q26 to Q16            */
    frac = L_Extract_lc(L_tmp, &exp_gcode0);  /* Extract exponent of gcode0 */

    gcode0 = extract_l(Pow2(14, frac));    /* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767   */
    exp_gcode0 = sub(exp_gcode0, 14);


    /*tmp1 = (g_corr[0]*g_corr[2]) - (g_corr[4]*g_corr[4]);
    tmp2 = g_corr[1]/tmp1;
    tmp1 = g_corr[3]/tmp1;

    *gain_pit =  (g_corr[2]*tmp2) - (g_corr[4]*tmp1);
    *gain_code = (g_corr[0]*tmp1) - (g_corr[4]*tmp2);*/

    /*    *gain_pit =  (g_corr[2]*tmp2) - (g_corr[4]*tmp3);
                  =  ((g_corr[1]*g_corr[2]) - (g_corr[3]*g_corr[4]))/tmp1;*/

    /*     *gain_code = (g_corr[0]*tmp3) - (g_corr[4]*tmp2);
                   = ((g_corr[3]*g_corr[0]) - (g_corr[1]*g_corr[4]))/tmp1;*/

    L_tmp1 = L_mult(coeff[0],coeff[2]); /*Q31*/
    exp1 = add(exp_coeff[0], exp_coeff[2]);

    L_tmp2 = L_mult(coeff[4],coeff[4]); /*Q31*/
    exp2 = add(exp_coeff[4], exp_coeff[4]);

    IF(sub(exp1,exp2)>0)
    {
        L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
        exp_den = exp1;
        move16();
    }
    ELSE
    {
        L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
        exp_den = exp2;
        move16();
    }
    L_frac_den = L_sub(L_tmp1,L_tmp2); /*Q31*/

    frac_den = extract_h(L_frac_den);
    frac_den  = s_max(frac_den,1);
    L_frac_den  = L_max(L_frac_den,1);
    exp = norm_l(L_frac_den);
    tmp = div_s(shl(1,sub(14,exp)),frac_den); /*Q(14-exp)*/



    L_tmp1 = L_mult(coeff[3],coeff[4]); /*Q31*/
    exp1 = add(exp_coeff[3], exp_coeff[4]);

    L_tmp2 = L_mult(coeff[1],coeff[2]); /*Q31*/
    exp2 = add(exp_coeff[1], exp_coeff[2]);

    IF(sub(exp1,exp2)>0)
    {
        L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
        exp_num = exp1;
        move16();
    }
    ELSE
    {
        L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
        exp_num = exp2;
        move16();
    }
    L_frac_num = L_sub(L_tmp2, L_tmp1); /*Q31*/

    L_div = Mult_32_16(L_frac_num,tmp); /*Q(30-exp)*/
    exp_div = sub(exp_num,exp_den);

    *gain_pit = round_fx(L_shl(L_div,add(exp,exp_div))); /*Q14*/

    L_tmp1 = L_mult(coeff[1],coeff[4]); /*Q31*/
    exp1 = add(exp_coeff[1], exp_coeff[4]);

    L_tmp2 = L_mult(coeff[0],coeff[3]); /*Q31*/
    exp2 = add(exp_coeff[0], exp_coeff[3]);

    IF(sub(exp1,exp2)>0)
    {
        L_tmp2 = L_shr(L_tmp2,sub(exp1,exp2)); /*Q31*/
        exp_num = exp1;
    }
    ELSE
    {
        L_tmp1 = L_shr(L_tmp1,sub(exp2,exp1)); /*Q31*/
        exp_num = exp2;
    }
    L_frac_num = L_sub(L_tmp2, L_tmp1); /*Q31*/

    L_div = Mult_32_16(L_frac_num,tmp); /*Q(30-exp)*/
    exp_div = sub(exp_num,exp_den);

    *gain_code = L_shl(L_div,sub(add(exp,exp_div),14));
    move32();/*Q16*/

    /*-----------------------------------------------------------------*
     * limit the pitch gain searching range (if indicated by clip_gain)
     *-----------------------------------------------------------------*/

    test();
    test();
    IF( sub(clip_gain,1) == 0 && sub(*gain_pit, 15565)  > 0)
    {
        *gain_pit = 15565;
        move16();
    }
    ELSE IF( sub(clip_gain,2) == 0 && sub(*gain_pit,10650) > 0 )
    {
        *gain_pit = 10650;
        move16();
    }

    /*-----------------------------------------------------------------*
     * search for the best quantized values
     *-----------------------------------------------------------------*/

    nBits_pitch = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr)) ];

    /* set number of bits for two SQs */
    nBits_code = shr(add(nBits_pitch,1),1);
    nBits_pitch = shr(nBits_pitch,1);

    /* gain_pit Q */
    /*tmp1 = (G_PITCH_MAX - G_PITCH_MIN) / ((1 << nBits_pitch) - 1);*/      /* set quantization step */
    tmp1 = mult_r(G_PITCH_MAX_Q13,div_s(1,sub(shl(1, nBits_pitch), 1)));   /*Q13*/   /* set quantization step */

    index = usquant_fx( *gain_pit, gain_pit, G_PITCH_MIN_Q14, tmp1, shl(1, nBits_pitch) );
    move16();
    push_indice_fx( st_fx, IND_GAIN_PIT, index, nBits_pitch );

    /* gain_code Q */
    /* *gain_code /= gcode0; */
    IF(gcode0 != 0)
    {
        tmp = div_s(16384,gcode0); /*Q15*/
        L_tmp = Mult_32_16(*gain_code,tmp); /*Q16*/
        *gain_code = L_shr(L_tmp,add(14,exp_gcode0)); /*Q16*/
    }

    index = gain_quant_fx( gain_code, &gain_code16, LG10_G_CODE_MIN_Q14, LG10_G_CODE_MAX_Q13, nBits_code, &expg );
    push_indice_fx( st_fx, IND_GAIN_CODE, index, nBits_code );
    L_tmp = L_mult(gain_code16,gcode0); /*Q0*Q0 -> Q1*/
    *gain_code = L_shl(L_tmp,add(add(expg,exp_gcode0),15));
    move32();     /*Q16*/

    /* *norm_gain_code = *gain_code / *gain_inov; */
    exp = sub(norm_s(*gain_inov),1);
    exp = s_max(exp, 0);

    tmp = div_s(shr(8192,exp),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code, tmp),sub(1,exp));
    move32();

    return;
}

/*-------------------------------------------------------------------*
 * gain_enc_gaus()
 *
 * Quantization of gain for Gaussian codebook
 *-------------------------------------------------------------------*/
Word16 gain_enc_gaus_fx(          /* o  : Return index of quantization      */
    Word32 *gain,        /* i/o: Code gain to quantize             */
    const Word16 bits,         /* i  : number of bits to quantize        */
    const Word16 lowBound,     /* i  : lower bound of quantizer (dB) Q8  */
    const Word16 stepSize,     /* i  : Step size choice              Q14 */
    const Word16 inv_stepSize  /* i  : Step size choice              Q15 */
)
{
    Word16 index, exp_gain, frac_gain, wtmp;
    Word16 enr_q, wenr;
    Word32 Ltmp, enr;

    /*enr = 20.0 * log10(*gain + 0.001)     codebook gain in dB  */
    exp_gain = norm_l(*gain);
    frac_gain = Log2_norm_lc(L_shl(*gain, exp_gain));
    exp_gain = sub(30-16, exp_gain);

    enr = Mpy_32_16(exp_gain, frac_gain, LG10); /* Output in Q13 */
    wenr = extract_h(L_shl(enr, 8+3));

    /*----------------------------------------------------------------*
     * Quantize linearly the log E
     *----------------------------------------------------------------*/

    wtmp = sub(wenr, lowBound);                  /* Q8 */

    index = extract_l(L_shr(L_mac(8388608, wtmp, inv_stepSize),16+8));

    /* index [0 (1<<bits)-1] */
    index = s_min(index, sub(shl(1,bits),1));
    index = s_max(index, 0);

    Ltmp = L_mac(L_shl(lowBound,7), index, stepSize);
    enr_q = round_fx(L_shl(Ltmp,16-7));               /* enr_q Q8 */

    /* gain = (float)pow( 10.0f, enr/20.0f )   quantized codebook gain */
    enr = L_mult(enr_q, 21772);              /* 0.166096 in Q17 -> Q26 */
    enr = L_shr(enr, 10);                    /*Q26->Q16*/
    frac_gain = L_Extract_lc(enr, &exp_gain);

    Ltmp = Pow2(14, frac_gain);              /* Put 14 as exponent */
    exp_gain= sub(exp_gain, 14);             /* Retreive exponent of wtmp */
    *gain = L_shl(Ltmp ,add(16,exp_gain));
    move32(); /*Q16*/

    return index;
}
/*-----------------------------------------------------------------*
  * gain_enc_tc()
  *
  * Search and quantization of gain_code for subframes (in the
  * beginning of frame) without pulses in TC - 3b coding.
  * In this case:
  * - gain_pit = 0
  * - gain_code - scalar quantization (no prediciton history used)
  *-----------------------------------------------------------------*/
void gain_enc_tc_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    const Word32  core_brate,        /* i  : core bitrate                                       */
    const Word16 L_frame,           /* i  : length of the frame                                */
    const Word16 i_subfr,           /* i  : subframe index                                     */
    const Word16 tc_subfr,          /* i  : TC subframe index                                  */
    const Word16 xn_fx[],              /* i  : target vector                                      */
    const Word16 y2_fx[],              /* i  : zero-memory filtered algebraic codebook excitation */
    const Word16 code_fx[],            /* i  : algebraic excitation                               */
    const Word16 Es_pred_fx,           /* i  : predicted scaled innovation energy                 */
    Word16 *gain_pit_fx,         /* o  : Pitch gain / Quantized pitch gain                  */
    Word32 *gain_code_fx,        /* o  : quantized codebook gain                            */
    Word16 *gain_inov_fx,        /* o  : innovation gain                                    */
    Word32 *norm_gain_code_fx,    /* o  : norm. gain of the codebook excitation                           */
    const Word16  Q_xn       /* i  : xn and y1 scaling                                   Q0 */
)
{
    Word16 i, index=0, nBits, num, den, exp_num, exp_den;
    Word16 Ei_fx, g_code_fx, gcode0_fx;
    Word16 expg, expg2, e_tmp, f_tmp, exp_gcode0, tmp_fx, frac,tmp16;
    Word32 L_tmp, L_tmp1;
    Word16 wgain_code=0, gain_code16;
    *gain_pit_fx = 0;
    move16();
    /*----------------------------------------------------------------*
     * get number of bits for gain quantization
     *----------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
        move16();
    }

    /*----------------------------------------------------------------*
     * find the code pitch (for current subframe)
     *----------------------------------------------------------------*/

    /**gain_code = dotp( xn, y2, L_SUBFR )/( dotp( y2, y2, L_SUBFR ) + 0.01f );*/
    /* Compute scalar product <y2[],y2[]> */
    L_tmp = Dot_product(y2_fx, y2_fx, L_SUBFR);       /* -18 (y2 Q9) */
    exp_den = norm_l(L_tmp);
    den = extract_h(L_shl(L_tmp, exp_den));
    exp_den = sub(add(exp_den, 18), shl(Q_xn, 1));

    /* Compute scalar product <xn[],y2[]> */
    L_tmp1 = Dot_product(xn_fx, y2_fx, L_SUBFR);      /* -9 (y2 Q9)  */
    exp_num = sub(norm_l(L_tmp1),1);
    num = extract_h(L_shl(L_tmp1, exp_num));
    exp_num = sub(add(exp_num, 8 ), Q_xn);

    tmp16 = s_or(shr(num, 16), 1); /* extract sign if num < 0 tmp16 = -1 else tmp16 = 1 */
    num = abs_s(num);

    /*----------------------------------------------------------------*
     * compute gain = xy/yy
     *----------------------------------------------------------------*/
    g_code_fx = div_s(num, den);

    i = sub(exp_num, exp_den); /* Gain_trans in Q7 */
    g_code_fx = i_mult2(g_code_fx, tmp16); /* apply sign */
    *gain_code_fx = L_shr(L_deposit_l(g_code_fx),i);
    move32();

    /*----------------------------------------------------------------*
     * calculate the predicted gain code
     * decode codebook gain
     *----------------------------------------------------------------*/

    *gain_pit_fx = 0;
    move16();

    /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;
    *gain_inov = 1.0f / (float)sqrt( Ecode );*/

    L_tmp = Dot_product12(code_fx, code_fx, L_SUBFR, &expg);
    expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    expg2 = expg;
    move16();
    L_tmp1 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
    L_tmp = Isqrt_lc(L_tmp, &expg);

    *gain_inov_fx = extract_h(L_shl(L_tmp, sub(expg, 3)));
    move16();/* gain_inov in Q12 */

    /*Ei = 10 * (float)log10( Ecode );*/
    e_tmp = norm_l(L_tmp1);
    f_tmp = Log2_norm_lc(L_shl(L_tmp1, e_tmp));
    e_tmp = sub(expg2,add(1,e_tmp));
    L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/
    Ei_fx = round_fx(L_shl(L_tmp1, 11)); /* Q8 */
    /*gcode0 = (float) pow(10, 0.05 * (Es_pred - Ei));*/
    gcode0_fx = sub(Es_pred_fx, Ei_fx); /* Q8 */
    /*-----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     * = pow(2, 3.321928*gcode0/20)
     * = pow(2, 0.166096*gcode0)
     *-----------------------------------------------------------------*/
    L_tmp = L_mult(gcode0_fx, 21771); /* *0.166096 in Q17 -> Q26 */
    L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */
    gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
    exp_gcode0 = sub(exp_gcode0, 14);
    IF( sub(nBits,3) > 0 )
    {
        /*g_code = *gain_code / gcode0;*/
        IF(gcode0_fx != 0)
        {
            tmp16 = div_s(16384,gcode0_fx); /*Q15*/
            L_tmp = Mult_32_16(*gain_code_fx,tmp16); /*Q16*/
            *gain_code_fx = L_shr(L_tmp,add(14,exp_gcode0)); /*Q16*/
        }
        ELSE
        {
            *gain_code_fx = MAX_32;
            move32();
        }

        /*index = gain_quant( &g_code, G_CODE_MIN, G_CODE_MAX, nBits );*/
        index = gain_quant_fx( gain_code_fx, &gain_code16, LG10_G_CODE_MIN_TC_Q14, LG10_G_CODE_MAX_TC_Q13, nBits, &expg );

        /**gain_code = g_code * gcode0;*/
        L_tmp = L_mult(gain_code16,gcode0_fx); /*Q0*Q0 -> Q1*/
        *gain_code_fx = L_shl(L_tmp,add(add(expg,exp_gcode0),15)); /*Q16*/     move32();

        push_indice_fx( st_fx, IND_GAIN_CODE, index, nBits );
    }
    ELSE
    {
        index = N_GAIN_CODE_TC-1;
        move16();
        FOR( i=0; i < N_GAIN_CODE_TC-1; i++ )
        {
            L_tmp = L_mult(tbl_gain_code_tc_quant_mean[i], gcode0_fx); /* Q13*Q0 -> Q14 */
            L_tmp = L_shl(L_tmp, add(exp_gcode0, 2));           /*    Q14 -> Q16 */

            IF( L_sub(*gain_code_fx, L_tmp) < 0 )
            {
                index = i;
                move16();
                BREAK;
            }
        }
        /*----------------------------------------------------------------*
         * 3-bit -> 2-bit encoding
         *----------------------------------------------------------------*/
        IF( sub(nBits,2) == 0 )
        {
            /* 2-bit -> 3-bit decoding */
            index = shr(index ,1);
            wgain_code = tbl_gain_code_tc_fx[shl(index,1)];
            move16();
            /**gain_code *= gcode0;*/
            L_tmp = L_mult(wgain_code, gcode0_fx);         /* Q13*Q0 -> Q14 */
            *gain_code_fx= L_shl(L_tmp, add(exp_gcode0, 2));
            move32(); /* Q14 -> Q16 */
            push_indice_fx( st_fx, IND_GAIN_CODE, index, nBits );
        }
        ELSE /* nBits == 3 */
        {
            wgain_code = tbl_gain_code_tc_fx[index];
            move16();
            /**gain_code *= gcode0;*/
            L_tmp = L_mult(wgain_code, gcode0_fx);         /* Q13*Q0 -> Q14 */
            *gain_code_fx= L_shl(L_tmp, add(exp_gcode0, 2));
            move32(); /* Q14 -> Q16 */
            push_indice_fx( st_fx, IND_GAIN_CODE, index, nBits );
        }
    }

    /*-----------------------------------------------------------------*
     * decode normalized codebook gain
     *-----------------------------------------------------------------*/
    /**norm_gain_code = *gain_code / *gain_inov;*/
    expg = sub(norm_s(*gain_inov_fx),1);
    expg = s_max(expg, 0);

    tmp_fx = div_s(shr(8192,expg),*gain_inov_fx);
    *norm_gain_code_fx = L_shr(Mult_32_16(*gain_code_fx, tmp_fx),sub(1,expg));
    move32();
    return;
}
/*-----------------------------------------------------------------*
  * Find_Opt_gainQ_fx()
  *
  * Find the best quantizer
  *-----------------------------------------------------------------*/
static Word16 Find_Opt_gainQ_fx(
    Word16 *coeff,
    Word16 *exp_coeff,
    Word16 *gain_pit,
    Word32 *gain_code,
    Word16 gcode0,
    Word16 exp_gcode0,
    const Word16 *cdbk,               /* i  : Codebook used */
    const Word16 size                 /* i  : size of Codebook used */
)
{
    Word16 index, i, j;
    const  Word16 *p;
    Word16 g_pitch, g2_pitch, g_code, g_pit_cod, g2_code, g2_code_lo;
    Word32 dist_min;
    Word16 coeff_lo[5];
    Word16 exp_max[5];
    Word16 exp_code, e_max;
    Word32 L_tmp, L_tmp1;


    /*----------------------------------------------------------------*
     * Find the best quantizer
     * ~~~~~~~~~~~~~~~~~~~~~~~
     * Before doing the computation we need to align exponents of coeff[]
     * to be sure to have the maximum precision.
     *
     * In the table the pitch gains are in Q14, the code gains are in Q9 and
     * are multiply by gcode0 which have been multiply by 2^exp_gcode0.
     * Also when we compute g_pitch*g_pitch, g_code*g_code and g_pitch*g_code
     * we divide by 2^15.
     * Considering all the scaling above we have:
     *
     *   exp_code = exp_gcode0-9+15 = exp_gcode0+6
     *
     *   g_pitch*g_pitch  = -14-14+15
     *   g_pitch          = -14
     *   g_code*g_code    = (2*exp_code)+15
     *   g_code           = exp_code
     *   g_pitch*g_code   = -14 + exp_code +15
     *
     *   g_pitch*g_pitch * coeff[0]  ;exp_max0 = exp_coeff[0] - 13
     *   g_pitch         * coeff[1]  ;exp_max1 = exp_coeff[1] - 14
     *   g_code*g_code   * coeff[2]  ;exp_max2 = exp_coeff[2] +15+(2*exp_code)
     *   g_code          * coeff[3]  ;exp_max3 = exp_coeff[3] + exp_code
     *   g_pitch*g_code  * coeff[4]  ;exp_max4 = exp_coeff[4] + 1 + exp_code
     *----------------------------------------------------------------*/

    exp_code = add(exp_gcode0, 6);

    exp_max[0] = sub(exp_coeff[0], 13);
    move16();
    exp_max[1] = sub(exp_coeff[1], 14);
    move16();
    exp_max[2] = add(exp_coeff[2], add(15, shl(exp_code, 1)));
    move16();
    exp_max[3] = add(exp_coeff[3], exp_code);
    move16();
    exp_max[4] = add(exp_coeff[4], add(1, exp_code));
    move16();

    /* Find maximum exponant */
    e_max = exp_max[0];
    move16();
    FOR (i = 1; i < 5; i++)
    {
        e_max = s_max(exp_max[i], e_max);
    }

    /* align coeff[] and save in special 32 bit double precision */
    FOR (i = 0; i < 5; i++)
    {
        j = add(sub(e_max, exp_max[i]), 2); /* /4 to avoid overflow */
        L_tmp = L_deposit_h(coeff[i]);
        L_tmp = L_shr(L_tmp, j);
        L_Extract(L_tmp, &coeff[i], &coeff_lo[i]);
        coeff_lo[i] = shr(coeff_lo[i], 3);/* lo >> 3 */       move16();
    }

    /* searching of codebook */
    p = cdbk;
    move16();
    dist_min = L_deposit_h(MAX_16);
    index = 0;
    move16();
    FOR (i = 0; i < size; i++)
    {
        g_pitch = *p++;
        move16();
        g_code = *p++;
        move16();

        g_code = mult_r(g_code, gcode0);
        g2_pitch = mult_r(g_pitch, g_pitch);
        g_pit_cod = mult_r(g_code, g_pitch);
        L_tmp = L_mult(g_code, g_code);
        g2_code_lo = L_Extract_lc(L_tmp, &g2_code);

        L_tmp = L_mult(coeff[2], g2_code_lo);
        L_tmp = L_shr(L_tmp, 3);
        L_tmp = L_mac(L_tmp, coeff_lo[0], g2_pitch);
        L_tmp = L_mac(L_tmp, coeff_lo[1], g_pitch);
        L_tmp = L_mac(L_tmp, coeff_lo[2], g2_code);
        L_tmp = L_mac(L_tmp, coeff_lo[3], g_code);
        L_tmp = L_mac(L_tmp, coeff_lo[4], g_pit_cod);
        L_tmp = L_shr(L_tmp, 12);
        L_tmp = L_mac(L_tmp, coeff[0], g2_pitch);
        L_tmp = L_mac(L_tmp, coeff[1], g_pitch);
        L_tmp = L_mac(L_tmp, coeff[2], g2_code);
        L_tmp = L_mac(L_tmp, coeff[3], g_code);
        L_tmp = L_mac(L_tmp, coeff[4], g_pit_cod);

        L_tmp1 = L_sub(L_tmp, dist_min);
        if (L_tmp1 < 0)
        {
            index = i;
            move16();
        }
        dist_min = L_min(L_tmp, dist_min);
    }

    p = &cdbk[add(index, index)];
    move16();

    *gain_pit  = *p++; /* selected pitch gain in Q14 */ move16();
    g_code = *p++;    /* selected  code gain in Q9 */ move16();

    L_tmp = L_mult(g_code, gcode0);            /* Q9*Q0 -> Q10 */
    L_tmp = L_shl(L_tmp, add(exp_gcode0, 6));  /* Q10 -> Q16 */

    *gain_code = L_tmp; /* gain of code in Q16 */       move16();
    return index;
}
/*---------------------------------------------------------------------*
  * gain_enc_lbr()
  *
  * Quantization of pitch and codebook gains without prediction (memory-less)
  * in ACELP at 6.6 and 7.5 kbps
  * - the gain codebooks and gain estimation constants are different in each subframe
  * - the estimated gain, gcode0, is first determined based on
  *   classification and/or previous quantized gains (from previous subframes in the current frame)
  * - a correction factor gamma = g_code / gcode0 is then vector quantized
  *   along with gain_pit
  * - the mean-squared error criterion is used for codebook search
  *---------------------------------------------------------------------*/

void gain_enc_lbr_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word32  core_brate,       /* i  : core bitrate                                                    */
    const Word16 coder_type,        /* i  : coding type                                                     */
    const Word16 i_subfr,           /* i  : subframe index                                                  */
    const Word16 *xn,               /* i  : target vector                                                   Q_xn*/
    const Word16 *y1,               /* i  : zero-memory filtered adaptive excitation                        Q_xn*/
    const Word16 Q_xn,              /* i  : xn and y1 format                                                  */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation              Q9*/
    const Word16 *code,             /* i  : algebraic excitation                                            Q9*/
    Word16 *gain_pit,         /* o  : quantized pitch gain                                            Q14*/
    Word32 *gain_code,        /* o  : quantized codebook gain                                         Q16*/
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                 Q12*/
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                           Q16*/
    Word16 *g_corr,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> mant/exp*/
    Word32 gc_mem[],          /* i/o: gain_code from previous subframes                                 */
    Word16 gp_mem[],          /* i/o: gain_pitch from previous subframes                                */
    const Word16 clip_gain          /* i  : gain pitch clipping flag (1 = clipping)                         */
)
{

    Word16 index = 0, size, nBits, n_pred, ctype;
    const Word16 *b, *cdbk = 0;
    Word16 gcode0, aux[10];
    Word16 coeff[5], exp_coeff[5];
    Word16 exp, exp_code, exp_inov, exp_gcode0, frac, tmp;
    Word32 L_tmp, L_tmp1, L_inov;

    /*-----------------------------------------------------------------*
     * calculate the rest of the correlation coefficients
     * c2 = <y2,y2>, c3 = -2<xn,y2>, c4 = 2<y1,y2>, c5* = <xn,xn>
     * c5* - not necessary to calculate
     *-----------------------------------------------------------------*/

    coeff[0] = g_corr[0];
    move16();
    exp_coeff[0] = g_corr[1];
    move16();
    coeff[1] = negate(g_corr[2]);
    move16();  /* coeff[1] = -2 xn yy1 */
    exp_coeff[1] = add(g_corr[3], 1);
    move16();

    /* Compute scalar product <y2[],y2[]> */

    coeff[2] = extract_h(Dot_product12(y2, y2, L_SUBFR, &exp));
    exp_coeff[2] = add(sub(exp, 18), shl(Q_xn, 1));  /* -18 (y2 Q9) */  move16();

    /* Compute scalar product -2*<xn[],y2[]> */

    coeff[3] = extract_h(L_negate(Dot_product12(xn, y2, L_SUBFR, &exp)));
    exp_coeff[3] = add(sub(exp, 9 - 1), Q_xn);  /* -9 (y2 Q9), +1 (2 xn y2) */  move16();

    /* Compute scalar product 2*<y1[],y2[]> */

    coeff[4] = extract_h(Dot_product12(y1, y2, L_SUBFR, &exp));
    exp_coeff[4] = add(sub(exp, 9 - 1), Q_xn);  /* -9 (y2 Q9), +1 (2 yy1 y2) */ move16();

    /*g_corr[2] += 0.01F; g_corr[3] -= 0.02F; g_corr[4] += 0.02F;*/

    /*Ecode = ( dotp( code, code, L_SUBFR ) + 0.01f ) / L_SUBFR;
    *gain_inov = 1.0f / (float)sqrt(Ecode);*/
    L_tmp = Dot_product12(code, code, L_SUBFR, &exp_code);
    L_inov = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
    /* exp_code: -18 (code in Q9), -6 (/L_SUBFR), -31 (L_tmp Q31->Q0) */
    /* output gain_inov*/
    exp_inov = sub(exp_code, 18 + 6);
    L_inov = Isqrt_lc(L_inov, &exp_inov);
    *gain_inov = extract_h(L_shl(L_inov, sub(exp_inov, 3))); /* gain_inov in Q12 */


    /*-----------------------------------------------------------------*
     * select the codebook, size and number of bits
     * set the gains searching range
     *-----------------------------------------------------------------*/

    nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, 0)];
    move16();
    size = extract_l(pow2[nBits]);

    ctype = shl(sub(coder_type, 1), 1);

    /*-----------------------------------------------------------------*
     * calculate prediction of gcode
     * search for the best codeword
     *-----------------------------------------------------------------*/
    IF (i_subfr == 0)
    {
        b = b_1sfr_fx;
        move16();
        n_pred = 2;
        move16();

        SWITCH ( nBits )
        {
        case 8:
            {
                cdbk = gp_gamma_1sfr_8b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,60);
                move16();
                BREAK;
            }
        case 7:
            {
                cdbk = gp_gamma_1sfr_7b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,27);
                move16();
                BREAK;
            }
        case 6:
            {
                cdbk = gp_gamma_1sfr_6b_fx;
                move16();
                if ( sub(clip_gain,1) == 0 ) size = sub(size,10);
                move16();
                BREAK;
            }
        }

        /* calculate predicted gain */
        aux[0] = 4096;
        move16();
        aux[1] = shl(ctype,12);

        /*     gcode0 = (float)pow(10, dotp(b, aux, n_pred) - 0.5f * (float)log10(Ecode));
               gcode0 = (float)pow(10, dotp(b, aux, n_pred) - 0.05f * 10 * (float)log10(Ecode));
               gcode0 = (float)pow(10, 0.05(20 * dotp(b, aux, n_pred) - 10 * (float)log10(Ecode))); */

        exp_code = sub(exp_code, 18 + 6 + 1);
        exp = norm_l(L_tmp);
        frac = Log2_norm_lc(L_shl(L_tmp, exp));
        exp = sub(exp_code,exp);
        L_tmp1 = Mpy_32_16(exp, frac, 24660); /* Q14 */ /* 10*log10(2) in Q13*/

        L_tmp = Dot_product(b, aux, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp,320);/*Q14, 20 in Q4*/
        L_tmp = L_sub(L_tmp,L_tmp1);/*Q14*/

        gcode0 = round_fx(L_shl(L_tmp, 10)); /* Q8 */

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, gcode0/20)
         *        = pow(2, 3.321928*gcode0/20)
         *        = pow(2, 0.166096*gcode0)
         *-----------------------------------------------------------------*/

        L_tmp = L_mult(gcode0, 21771); /* *0.166096 in Q17 -> Q26 */
        L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);
        index= Find_Opt_gainQ_fx(coeff, exp_coeff, gain_pit, gain_code, gcode0, exp_gcode0, cdbk, size);

        gc_mem[0] = *gain_code;
        move16(); /*Q16*/
        gp_mem[0] = *gain_pit;
        move16();/*Q14*/
    }
    ELSE IF (sub(i_subfr,L_SUBFR) == 0)
    {
        b = b_2sfr_fx;
        move16();
        n_pred = 4;
        move16();

        switch ( nBits )
        {
        case 7:
        {
            cdbk = gp_gamma_2sfr_7b_fx;
            move16();
            if ( sub(clip_gain,1) == 0 ) size = sub(size,30);
            move16();
            BREAK;
        }
        case 6:
        {
            cdbk = gp_gamma_2sfr_6b_fx;
            move16();
            if ( sub(clip_gain,1) == 0 ) size = sub(size,12);
            move16();
            BREAK;
        }
        }

        /* calculate predicted gain */
        aux[0] = 4096;
        move16();
        aux[1] = shl(ctype,12);
        move16();

        /*aux[2] = (float)log10(gc_mem[0]);
                 = log2(gc_mem[0])*log10(2);*/
        exp = norm_l(gc_mem[0]);
        frac = Log2_norm_lc(L_shl(gc_mem[0], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_1sfr_fx)=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864);
        move16(); /* Q16 */
        aux[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux[3] = shr(gp_mem[0],2);
        move16();/*Q12*/

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b, aux, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        index= Find_Opt_gainQ_fx(coeff, exp_coeff, gain_pit, gain_code, gcode0, exp_gcode0, cdbk, size);
        gc_mem[1] = *gain_code;
        move32();
        gp_mem[1] = *gain_pit;
        move16();
    }
    ELSE IF (sub(i_subfr,2*L_SUBFR) == 0)
    {
        b = b_3sfr_fx;
        move16();
        n_pred = 6;
        move16();
        cdbk = gp_gamma_3sfr_6b_fx;
        move16();
        if ( sub(clip_gain,1) == 0 )
        {
            size = sub(size,11);
        }

        /* calculate predicted gain */
        aux[0] = 4096;
        move16();
        aux[1] = shl(ctype,12);
        move16();

        /*aux[2] = (float)log10(gc_mem[0]);
                 = log2(gc_mem[0])*log10(2);*/
        exp = norm_l(gc_mem[0]);
        frac = Log2_norm_lc(L_shl(gc_mem[0], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_mem[0])=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864); /* Q16 */
        aux[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        /*aux[3] = (float)log10(gc_mem[1]);
                 =  log2(gc_mem[1])*log10(2);*/
        exp = norm_l(gc_mem[1]);
        frac = Log2_norm_lc(L_shl(gc_mem[1], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_mem[1])=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864); /* Q16 */
        aux[3] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux[4] = shr(gp_mem[0],2);
        move16();/*Q12*/
        aux[5] = shr(gp_mem[1],2);
        move16();/*Q12*/

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b, aux, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /*----------------------------------------------------------------*
         * Find the best quantizer
         * ~~~~~~~~~~~~~~~~~~~~~~~
         * Before doing the computation we need to align exponents of coeff[]
         * to be sure to have the maximum precision.
         *
         * In the table the pitch gains are in Q14, the code gains are in Q9 and
         * are multiply by gcode0 which have been multiply by 2^exp_gcode0.
         * Also when we compute g_pitch*g_pitch, g_code*g_code and g_pitch*g_code
         * we divide by 2^15.
         * Considering all the scaling above we have:
         *
         *   exp_code = exp_gcode0-9+15 = exp_gcode0+6
         *
         *   g_pitch*g_pitch  = -14-14+15
         *   g_pitch          = -14
         *   g_code*g_code    = (2*exp_code)+15
         *   g_code           = exp_code
         *   g_pitch*g_code   = -14 + exp_code +15
         *
         *   g_pitch*g_pitch * coeff[0]  ;exp_max0 = exp_coeff[0] - 13
         *   g_pitch         * coeff[1]  ;exp_max1 = exp_coeff[1] - 14
         *   g_code*g_code   * coeff[2]  ;exp_max2 = exp_coeff[2] +15+(2*exp_code)
         *   g_code          * coeff[3]  ;exp_max3 = exp_coeff[3] + exp_code
         *   g_pitch*g_code  * coeff[4]  ;exp_max4 = exp_coeff[4] + 1 + exp_code
         *----------------------------------------------------------------*/

        index= Find_Opt_gainQ_fx(coeff, exp_coeff, gain_pit, gain_code, gcode0, exp_gcode0, cdbk, size);

        gc_mem[2] = *gain_code;
        move32();
        gp_mem[2] = *gain_pit;
        move16();
    }
    ELSE IF (sub(i_subfr,3*L_SUBFR) == 0)
    {
        b = b_4sfr_fx;
        move16();
        n_pred = 8;
        move16();

        cdbk = gp_gamma_4sfr_6b_fx;
        move16();
        if ( sub(clip_gain,1) == 0 )
        {
            size = sub(size,11);
            move16();
        }

        /* calculate predicted gain */
        aux[0] = 4096;
        move16();
        aux[1] = shl(ctype,12);
        move16();

        /*aux[2] = (float)log10(gc_mem[0]);
                 = log2(gc_mem[0])*log10(2);*/
        exp = norm_l(gc_mem[0]);
        frac = Log2_norm_lc(L_shl(gc_mem[0], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_mem[0])=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864); /* Q16 */
        aux[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        /*aux[3] = (float)log10(gc_mem[1]);
                 =  log2(gc_mem[1])*log10(2);*/
        exp = norm_l(gc_mem[1]);
        frac = Log2_norm_lc(L_shl(gc_mem[1], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_mem[1])=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864); /* Q16 */
        aux[3] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */


        /*aux[4] = (float)log10(gc_mem[2]);
                 =  log2(gc_mem[2])*log10(2);*/
        exp = norm_l(gc_mem[2]);
        frac = Log2_norm_lc(L_shl(gc_mem[2], exp));
        exp = sub(sub(30,exp),16); /*Q_format(gc_mem[2])=16*/
        L_tmp1 = Mpy_32_16(exp, frac, 9864); /* Q16 */
        aux[4] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux[5] = shr(gp_mem[0],2);
        move16();/*Q12*/
        aux[6] = shr(gp_mem[1],2);
        move16();/*Q12*/
        aux[7] = shr(gp_mem[2],2);
        move16();/*Q12*/
        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b, aux, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        index = Find_Opt_gainQ_fx(coeff, exp_coeff, gain_pit, gain_code, gcode0, exp_gcode0, cdbk, size);
    }

    /* *norm_gain_code = *gain_code / *gain_inov; */
    exp = sub(norm_s(*gain_inov),1);
    exp = s_max(exp, 0);

    tmp = div_s(shr(8192,exp),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code, tmp),sub(1,exp));
    move32();
    {
        push_indice_fx( st_fx, IND_GAIN, index, nBits );
    }
    return;
}

/*-------------------------------------------------------------------*
 * gain_enc_amr_wb()
 *
 * Quantization of pitch and codebook gains (used also in AMR-WB IO mode)
 * MA prediction is performed on the innovation energy (in dB with mean removed).
 * An initial predicted gain, gcode0, is first determined and the correction
 * factor     alpha = g_code / gcode0   is quantized.
 * The pitch gain and the correction factor are vector quantized and the
 * mean-squared weighted error criterion is used in the quantizer search.
 *-------------------------------------------------------------------*/

#define nb_qua_gain7b  128  /* Number of quantization levels */
#define MEAN_ENER    30
#define RANGE        64

void gain_enc_amr_wb_fx(
    Encoder_State_fx *st,               /* i/o: encoder state structure      */
    const Word16 *xn,               /* i  : target vector                                                   */
    const Word16 Q_xn,              /* i  : xn and yy1 format                                    Q0 */
    const Word16 *yy1,               /* i  : zero-memory filtered adaptive excitation                        */
    const Word16 *y2,               /* i  : zero-memory filtered algebraic codebook excitation              */
    const Word16 *code,             /* i  : algebraic excitation                                            */
    const Word32 core_brate,        /* i  : core bitrate                                                    */
    Word16 *gain_pit,         /* i/o: pitch gain / Quantized pitch gain                               */
    Word32 *gain_code,        /* o  : quantized codebook gain                                         */
    Word16 *gain_inov,        /* o  : gain of the innovation (used for normalization)                 */
    Word32 *norm_gain_code,   /* o  : norm. gain of the codebook excitation                           */
    Word16 *g_coeff,           /* i/o: correlations <y1,y1>, -2<xn,y1>,<y2,y2>, -2<xn,y2> and 2<y1,y2> */
    const Word16 clip_gain,         /* i  : gain pitch clipping flag (1 = clipping)                         */
    Word16 *past_qua_en       /* i/o: gain quantization memory (4 words)                              */
)
{

    Word16 i, j, index, min_ind, size;
    Word16 exp, frac, gcode0, exp_gcode0, e_max, exp_code, exp_inov, qua_ener;
    Word16 g_pitch, g2_pitch, g_code, g_pit_cod, g2_code, g2_code_lo;
    Word16 coeff[5], coeff_lo[5], exp_coeff[5];
    Word16 exp_max[5], tmp, nBits;
    Word32 L_tmp, dist_min, L_inov, L_tmp1;
    const Word16 *t_qua_gain, *p;

    /*----------------------------------------------------------------*
     * Find the initial quantization pitch index
     * Set gains search range
     *----------------------------------------------------------------*/
    IF (core_brate>= ACELP_12k65)
    {
        t_qua_gain = t_qua_gain7b_fx;
        move16();
        /* pt at 1/4th of table */
        p = t_qua_gain7b_fx + RANGE;
        move16();

        j = nb_qua_gain7b - RANGE;
        move16();

        IF (sub(clip_gain, 1) == 0)
        {
            j = sub(j, 27);     /* limit gain pitch to 1.0 */
        }
        min_ind = 0;
        move16();
        g_pitch = *gain_pit;
        move16();

        FOR (i = 0; i < j; i++)
        {
            if (sub(g_pitch, *p) > 0)
            {
                min_ind = add(min_ind, 1);
            }
            p += 2;
        }
        size = RANGE;
        move16();
        nBits = 7;
    }
    ELSE
    {
        t_qua_gain = t_qua_gain6b_fx;
        min_ind = 0;
        move16();
        size = RANGE;
        move16();
        if (sub(clip_gain, 1) == 0)
        {
            size = sub(size, 16); /* limit gain pitch to 1.0 */
        }
        nBits = 6;
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
    coeff[0] = g_coeff[0];
    move16();
    exp_coeff[0] = g_coeff[1];
    move16();
    coeff[1] = negate(g_coeff[2]);
    move16();  /* coeff[1] = -2 xn yy1 */
    exp_coeff[1] = add(g_coeff[3], 1);
    move16();

    /* Compute scalar product <y2[],y2[]> */
    coeff[2] = extract_h(Dot_product12(y2, y2, L_SUBFR, &exp));
    exp_coeff[2] = add(sub(exp, 18), shl(Q_xn, 1));  /* -18 (y2 Q9) */

    /* Compute scalar product -2*<xn[],y2[]> */
    coeff[3] = extract_h(L_negate(Dot_product12(xn, y2, L_SUBFR, &exp)));
    exp_coeff[3] = add(sub(exp, 9 - 1), Q_xn);  /* -9 (y2 Q9), +1 (2 xn y2) */

    /* Compute scalar product 2*<yy1[],y2[]> */
    coeff[4] = extract_h(Dot_product12(yy1, y2, L_SUBFR, &exp));
    exp_coeff[4] = add(sub(exp, 9 - 1), Q_xn);  /* -9 (y2 Q9), +1 (2 yy1 y2) */

    /*----------------------------------------------------------------*
     *  Find energy of code and compute:
     *
     *    L_tmp = MEAN_ENER - 10log10(energy of code/ L_subfr)
     *          = MEAN_ENER - 3.0103*log2(energy of code/ L_subfr)
     *----------------------------------------------------------------*/
    L_tmp = Dot_product12(code, code, L_SUBFR, &exp_code);
    L_inov = L_add(L_tmp, 0);
    /* exp_code: -18 (code in Q9), -6 (/L_subfr), -31 (L_tmp Q31->Q0) */
    /* output gain_inov*/
    exp_inov = sub(exp_code, 18 + 6);
    L_inov = Isqrt_lc(L_inov, &exp_inov);
    *gain_inov = extract_h(L_shl(L_inov, sub(exp_inov, 3)));    /* gain_inov in Q12 */

    exp_code = sub(exp_code, 18 + 6 + 31);
    frac = Log2_lc(L_tmp, &exp);
    exp = add(exp, exp_code);
    L_tmp = Mpy_32_16(exp, frac, -24660);  /* x -3.0103(Q13) -> Q14 */

    L_tmp = L_mac(L_tmp, MEAN_ENER, 8192); /* + MEAN_ENER in Q14 */

    /*----------------------------------------------------------------*
     * predicted codebook gain
     *----------------------------------------------------------------*/
    L_tmp = L_shl(L_tmp, 10);                           /* From Q14 to Q24 */
    L_tmp = L_mac0(L_tmp, pred_gain_fx[0], past_qua_en[0]);  /* Q14*Q10 -> Q24 */
    L_tmp = L_mac0(L_tmp, pred_gain_fx[1], past_qua_en[1]);  /* Q14*Q10 -> Q24 */
    L_tmp = L_mac0(L_tmp, pred_gain_fx[2], past_qua_en[2]);  /* Q14*Q10 -> Q24 */
    L_tmp = L_mac0(L_tmp, pred_gain_fx[3], past_qua_en[3]);  /* Q14*Q10 -> Q24 */

    gcode0 = extract_h(L_tmp);                          /* From Q24 to Q8  */

    /*----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     *        = pow(2, 3.321928*gcode0/20)
     *        = pow(2, 0.166096*gcode0)
     *----------------------------------------------------------------*/
    L_tmp = L_mult(gcode0, 5443);          /* *0.166096 in Q15 -> Q24    */
    L_tmp = L_shr(L_tmp, 8);               /* From Q24 to Q16            */
    L_Extract(L_tmp, &exp_gcode0, &frac);  /* Extract exponent of gcode0 */

    gcode0 = extract_l(Pow2(14, frac));    /* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767   */
    exp_gcode0 = sub(exp_gcode0, 14);

    /*----------------------------------------------------------------*
     * Find the best quantizer
     * ~~~~~~~~~~~~~~~~~~~~~~~
     * Before doing the computation we need to aling exponents of coeff[]
     * to be sure to have the maximum precision.
     *
     * In the table the pitch gains are in Q14, the code gains are in Q11 and
     * are multiply by gcode0 which have been multiply by 2^exp_gcode0.
     * Also when we compute g_pitch*g_pitch, g_code*g_code and g_pitch*g_code
     * we divide by 2^15.
     * Considering all the scaling above we have:
     *
     *   exp_code = exp_gcode0-11+15 = exp_gcode0+4
     *
     *   g_pitch*g_pitch  = -14-14+15
     *   g_pitch          = -14
     *   g_code*g_code    = (2*exp_code)+15
     *   g_code           = exp_code
     *   g_pitch*g_code   = -14 + exp_code +15
     *
     *   g_pitch*g_pitch * coeff[0]  ;exp_max0 = exp_coeff[0] - 13
     *   g_pitch         * coeff[1]  ;exp_max1 = exp_coeff[1] - 14
     *   g_code*g_code   * coeff[2]  ;exp_max2 = exp_coeff[2] +15+(2*exp_code)
     *   g_code          * coeff[3]  ;exp_max3 = exp_coeff[3] + exp_code
     *   g_pitch*g_code  * coeff[4]  ;exp_max4 = exp_coeff[4] + 1 + exp_code
     *----------------------------------------------------------------*/

    exp_code = add(exp_gcode0, 4);

    exp_max[0] = sub(exp_coeff[0], 13);
    move16();
    exp_max[1] = sub(exp_coeff[1], 14);
    move16();
    exp_max[2] = add(exp_coeff[2],
                     add(15, shl(exp_code, 1)));
    move16();
    exp_max[3] = add(exp_coeff[3], exp_code);
    move16();
    exp_max[4] = add(exp_coeff[4],
                     add(1, exp_code));
    move16();

    /* Find maximum exponant */
    e_max = exp_max[0];
    move16();
    FOR (i = 1; i < 5; i++)
    {
        e_max = s_max(exp_max[i], e_max);
    }

    /* align coeff[] and save in special 32 bit double precision */
    FOR (i = 0; i < 5; i++)
    {
        j = add(sub(e_max, exp_max[i]), 2); /* /4 to avoid overflow */
        L_tmp = L_deposit_h(coeff[i]);
        L_tmp = L_shr(L_tmp, j);
        L_Extract(L_tmp, &coeff[i], &coeff_lo[i]);
        coeff_lo[i] = shr(coeff_lo[i], 3);/* lo >> 3 */move16();
    }

    /* Codebook search */
    dist_min = L_add(MAX_32, 0);
    p = &t_qua_gain[shl(min_ind, 1)];
    move16();

    index = 0;
    move16();
    FOR (i = 0; i < size; i++)
    {
        g_pitch = *p++;
        move16();
        g_code = *p++;
        move16();

        g_code = mult_r(g_code, gcode0);
        g2_pitch = mult_r(g_pitch, g_pitch);
        g_pit_cod = mult_r(g_code, g_pitch);
        L_tmp = L_mult(g_code, g_code);
        L_Extract(L_tmp, &g2_code, &g2_code_lo);

        L_tmp = L_mult(coeff[2], g2_code_lo);
        L_tmp = L_shr(L_tmp, 3);
        L_tmp = L_mac(L_tmp, coeff_lo[0], g2_pitch);
        L_tmp = L_mac(L_tmp, coeff_lo[1], g_pitch);
        L_tmp = L_mac(L_tmp, coeff_lo[2], g2_code);
        L_tmp = L_mac(L_tmp, coeff_lo[3], g_code);
        L_tmp = L_mac(L_tmp, coeff_lo[4], g_pit_cod);
        L_tmp = L_shr(L_tmp, 12);
        L_tmp = L_mac(L_tmp, coeff[0], g2_pitch);
        L_tmp = L_mac(L_tmp, coeff[1], g_pitch);
        L_tmp = L_mac(L_tmp, coeff[2], g2_code);
        L_tmp = L_mac(L_tmp, coeff[3], g_code);
        L_tmp = L_mac(L_tmp, coeff[4], g_pit_cod);

        L_tmp1 = L_sub(L_tmp, dist_min);
        /* splitting the if cost half the complexity of using IF macro */
        if (L_tmp1 < 0)
        {
            dist_min = L_add(L_tmp, 0);
        }
        if (L_tmp1 < 0)
        {
            index = i;
            move16();
        }

    }
    /* Read the quantized gains */
    index = add(index, min_ind);

    p = &t_qua_gain[add(index, index)];
    move16();
    *gain_pit = *p++; /* selected pitch gain in Q14 */ move16();
    g_code = *p++;    /* selected  code gain in Q11 */ move16();

    L_tmp = L_mult(g_code, gcode0);            /* Q11*Q0 -> Q12 */
    L_tmp = L_shl(L_tmp, add(exp_gcode0, 4));  /* Q12 -> Q16 */

    *gain_code = L_tmp; /* gain of code in Q16 */       move16();

    /*---------------------------------------------------*
     * qua_ener = 20*log10(g_code)
     *          = 6.0206*log2(g_code)
     *          = 6.0206*(log2(g_codeQ11) - 11)
     *---------------------------------------------------*/
    L_tmp = L_deposit_l(g_code);
    frac = Log2_lc(L_tmp, &exp);
    exp = sub(exp, 11);
    L_tmp = Mpy_32_16(exp, frac, 24660);   /* x 6.0206 in Q12 */

    qua_ener = extract_l(L_shr(L_tmp, 3));  /* result in Q10 */

    /*----------------------------------------------------------------*
     * update table of past quantized energies
     *----------------------------------------------------------------*/

    past_qua_en[3] = past_qua_en[2];
    move16();
    past_qua_en[2] = past_qua_en[1];
    move16();
    past_qua_en[1] = past_qua_en[0];
    move16();
    past_qua_en[0] = qua_ener;
    move16();


    exp = sub(norm_s(*gain_inov),1);
    exp = s_max(exp, 0);

    tmp = div_s(shr(8192,exp),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code, tmp),sub(1,exp));
    move32();

    push_indice_fx( st, IND_GAIN, index, nBits );

    return;
}
