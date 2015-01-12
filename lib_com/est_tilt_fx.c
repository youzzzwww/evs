/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"
#include "basop_util.h"


/*======================================================================*/
/* FUNCTION : est_tilt_fx()												*/
/*-----------------------------------------------------------------------*/
/* PURPOSE :  Estimate spectral tilt based on the relative E of adaptive */
/* and innovative excitations                                            */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :													 */
/* _ (Word16 *) exc :  adaptive excitation vector      Q0                */
/* _ (Word16) gain_pit : adaptive gain                 Q14               */
/* _ (Word16 *) code : algebraic exctitation vector    Q12               */
/* _ (Word32) gain_code :  algebraic code gain         Q16               */
/* _ (Word16) Q_exc : Scaling factor of excitation     Q0                */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/* _ (Word16 *) voice_fac :   voicing factor          Q15                */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ (Word16) tolt_code :  tilt of the code           Q15                */
/*=======================================================================*/
Word16 est_tilt_fx(              /* o  : tilt of the code              Q15      */
    const Word16 *exc,        /* i  : adaptive excitation vector      Qx  */
    const Word16 gain_pit,    /* i  : adaptive gain                   Q14 */
    const Word16 *code,       /* i  : algebraic exctitation vector    Q9  */
    const Word32 gain_code,   /* i  : algebraic code gain             Q16 */
    Word16 *voice_fac,  /* o  : voicing factor                  Q15 */
    const Word16 Q_exc        /* i  : Scaling factor of excitation    Q0  */
)
{
    Word16 i, tmp, exp, ener1, exp1, ener2, exp2;
    Word32 L_tmp;
    Word16 tilt_code;

    ener1 = extract_h(Dot_product12(exc, exc, L_SUBFR, &exp1));
    exp1 = sub(exp1, add(Q_exc, Q_exc));
    L_tmp = L_mult(gain_pit, gain_pit); /* energy of pitch excitation */
    exp = norm_l(L_tmp);
    tmp = extract_h(L_shl(L_tmp, exp));
    ener1 = mult(ener1, tmp);
    exp1 = sub(sub(exp1, exp), 10);     /* 10 -> gain_pit Q14 to Q9   */

    ener2 = extract_h(Dot_product12(code, code, L_SUBFR, &exp2));

    exp = norm_l(gain_code);
    tmp = extract_h(L_shl(gain_code, exp));
    tmp = mult(tmp, tmp);               /* energy of innovative code excitation */
    ener2 = mult(ener2, tmp);
    exp2 = sub(exp2, add(exp, exp));

    i = sub(exp1, exp2);
    BASOP_SATURATE_WARNING_OFF
    ener1 = shr(ener1, sub(1, s_min(i, 0)));
    ener2 = shr(ener2, add(s_max(0, i), 1));
    BASOP_SATURATE_WARNING_ON
    tmp = sub(ener1, ener2);
    ener1 = add(add(ener1, ener2), 1);

    /* find voice factor (1=voiced, -1=unvoiced) */
    exp = div_s(abs_s(tmp), ener1);
    if (tmp < 0)
    {
        exp = negate(exp);
    }
    *voice_fac = exp;
    move16();

    /* tilt of code for next subframe: 0.5=voiced, 0=unvoiced */

    /* tilt_code = (float)(0.25*(1.0 + *voice_fac)) */
    tilt_code = mac_r(8192L*65536-0x8000, *voice_fac, 8192); /*Q15 */

    return tilt_code;
}
/*-------------------------------------------------------------------*
 * Est_tilt2:
 *
 * Estimate spectral tilt based on the relative E of adaptive
 * and innovative excitations
 *-------------------------------------------------------------------*/
Word16 Est_tilt2(      /* o  : tilt of the code                    */
    const Word16 *exc,        /* i  : adaptive excitation vector      Qx  */
    const Word16 gain_pit,    /* i  : adaptive gain                   Q14 */
    const Word16 *code,       /* i  : algebraic exctitation vector    Q9  */
    const Word32 gain_code,   /* i  : algebraic code gain             Q16 */
    Word16 *voice_fac,  /* o  : voicing factor                  Q15 */
    const Word16 Q_exc        /* i  : Scaling factor of excitation    Q0  */
)
{
    Word16 i, tmp, exp, ener1, exp1, ener2, exp2;
    Word32 L_tmp;
    Word16 tilt_code;

    /* Scale exc to avoid overflow */
    ener1 = extract_h(Energy_scale(exc, L_SUBFR, Q_exc, &exp1));

    exp1 = sub(exp1, add(Q_exc, Q_exc));
    L_tmp = L_mult(gain_pit, gain_pit); /* energy of pitch excitation */
    exp = norm_l(L_tmp);
    tmp = extract_h(L_shl(L_tmp, exp));
    ener1 = mult(ener1, tmp);
    exp1 = sub(sub(exp1, exp), 10);     /* 10 -> gain_pit Q14 to Q9   */

    ener2 = extract_h(Dot_product12(code, code, L_SUBFR, &exp2));

    exp = norm_l(gain_code);
    tmp = extract_h(L_shl(gain_code, exp));
    tmp = mult(tmp, tmp);               /* energy of innovative code excitation */
    ener2 = mult(ener2, tmp);
    exp2 = sub(exp2, add(exp, exp));

    i = sub(exp1, exp2);
    ener1 = shr(ener1, sub(1, s_min(i, 0)));
    ener2 = shr(ener2, add(s_max(0, i), 1));

    tmp = sub(ener1, ener2);
    ener1 = add(add(ener1, ener2), 1);

    /* find voice factor (1=voiced, -1=unvoiced) */
    exp = div_s(abs_s(tmp), ener1);
    if (tmp < 0)
    {
        exp = negate(exp);
    }
    *voice_fac = exp;
    move16();

    /* tilt of code for next subframe: 0.5=voiced, 0=unvoiced */

    /* tilt_code = (float)(0.25*(1.0 + *voice_fac)) */
    tilt_code = mac_r(8192L*65536-0x8000, *voice_fac, 8192);

    return tilt_code;
}

/*---------------------------------------------------------*
 * Find voice factor and tilt code                         *
 *---------------------------------------------------------*/
void E_UTIL_voice_factor( Word16 *exc,          /* i  : pointer to the excitation frame   Q_new */
                          Word16 i_subfr,       /* i  : subframe index                          */
                          Word16 *code,         /* i  : innovative codebook                  Q9 */
                          Word16 gain_pit,      /* i  : adaptive codebook gain             1Q14 */
                          Word32 gain_code,     /* i  : innovative cb. gain               15Q16 */
                          Word16 *voice_fac,    /* o  : subframe voicing estimation         Q15 */
                          Word16 *tilt_code,    /* o  : tilt factor                         Q15 */
                          Word16 L_subfr,       /* i  : subframe length                         */
                          Word16 flag_tilt,     /* i  : Flag for triggering new voice factor tilt*/
                          Word16 Q_new,         /* i  : excitation buffer format                 */
                          Word16 shift          /* i  : scaling to get 12bit                     */
                        )
{
    Word16 i, e, e2, stmp, exp_ener, fac;
    Word32 ener, tmp, num;

    BASOP_SATURATE_ERROR_ON;

    IF(shift != 0)
    {
        fac = shl(0x4000,add(1,shift));
        /* energy of pitch excitation */
        stmp = mult_r(exc[0+i_subfr], fac); /* remove fac bits */
        ener = L_mac0(0L,stmp, stmp);
        FOR (i=1; i<L_subfr; i++)
        {
            stmp = mult_r(exc[i+i_subfr], fac); /* remove fac bits */
            ener = L_mac0(ener, stmp, stmp);
        }
    }
    ELSE
    {
        ener = L_mult0(exc[0+i_subfr], exc[0+i_subfr]);
        FOR (i=1; i<L_subfr; i++)
        {
            ener = L_mac0(ener, exc[i+i_subfr], exc[i+i_subfr]); /* Q_new -> exponent = (15-Q_new)*2+1  */
        }
    }

    /* exponent of ener: (2*(15-Q_new+shift)+1+2-exp_ener-2*e2) */
    exp_ener = norm_l(ener);
    if(ener == 0)
    {
        exp_ener = 31;
        move16();
    }
    ener = L_shl(ener,exp_ener);
    e2 = norm_s(gain_pit);
    gain_pit = shl(gain_pit,e2);
    ener = Mpy_32_16_1(ener, mult_r(gain_pit, gain_pit));


    /* energy of innovative code excitation */
    tmp = L_deposit_l(1);

    FOR (i=0; i<L_subfr; i++)
    {
        tmp = L_mac0(tmp, code[i], code[i]); /* 6Q9 -> 13Q18 */
    }
    /* exponent of tmp: 2*(15-9)+1+2*(15-e))  */
    e = norm_l(gain_code);
    gain_code = L_shl(gain_code, e);
    tmp = Mpy_32_32(tmp, Mpy_32_32(gain_code,gain_code));

    /* find voice factor (1=voiced, -1=unvoiced) */
    /*i = (2*(15-Q_new+shift)+1+2-exp_ener-2*e2) - (2*(15-9)+1 + 2*(15-e));*/
    i = sub(sub(sub(sub(sub(33,add(shift,shift)),add(Q_new,Q_new)),exp_ener),add(e2,e2)),sub(43,add(e,e)));
    IF(i >= 0)
    {
        ener = L_shr(ener,1);
        tmp = L_shr(tmp, add(1,i));
    }
    ELSE
    {
        tmp = L_shr(tmp,1);
        BASOP_SATURATE_WARNING_OFF
        ener = L_shr(ener, sub(1,i));
        BASOP_SATURATE_WARNING_ON
    }

    *voice_fac = 0;
    move16();
    num = L_sub(ener, tmp);
    IF(num != 0)
    {
        BASOP_SATURATE_WARNING_OFF /* Allow saturating the voice factor because if has a limited range by definition. */
        *voice_fac = divide3232(num, L_add(ener, tmp));
        move16();
        BASOP_SATURATE_WARNING_ON
    }

    /* find tilt of code for next subframe */
    IF (flag_tilt==0)
    {
        /*Between 0 (=unvoiced) and 0.5 (=voiced)*/
        move16();
        *tilt_code = add(FL2WORD16(0.25f), mult_r(FL2WORD16(0.25f), *voice_fac));
    }
    ELSE IF (flag_tilt==1)
    {
        /*Between 0.25 (=unvoiced) and 0.5 (=voiced)*/
        move16();
        *tilt_code = add(mult_r(FL2WORD16(0.125f), *voice_fac), FL2WORD16(0.125f+0.25f));
    }
    ELSE
    {
        /*Between 0.28 (=unvoiced) and 0.56 (=voiced)*/
        move16();
        *tilt_code = add(mult_r(FL2WORD16(0.14f), *voice_fac), FL2WORD16(0.14f+0.28f));
    }
    BASOP_SATURATE_ERROR_OFF;
}
