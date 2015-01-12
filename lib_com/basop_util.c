/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "basop_util.h"
#include "rom_basop_util.h"
#include "basop_mpy.h"
#include "cnst_fx.h"
#include "control.h"
#include "options.h"

#include <stdio.h>
#include <assert.h>

#include "stl.h"

#define DOT12_SUBDIV_LD 2  /* log2(number of dot product sub divisions) */

#define HP20_COEF_SCALE  2
#define INV_BANDS10 3277 /* 1/10 in Q15 */
#define INV_BANDS9  3641 /* 1/9  in Q15 */

extern const Word32 SqrtTable[32];
extern const Word16 SqrtDiffTable[32];

extern const Word32 ISqrtTable[32];
extern const Word16 ISqrtDiffTable[32];

extern const Word32 InvTable[32];
extern const Word16 InvDiffTable[32];


Word32 BASOP_Util_Log2(Word32 x)
{
    Word32  exp;
    Word16  exp_e;
    Word16  nIn;
    Word16  accuSqr;
    Word32  accuRes;



    assert(x >= 0);

    if (x == 0)
    {

        return ((Word32)MIN_32);
    }

    /* normalize input, calculate integer part */
    exp_e = norm_l(x);
    x = L_shl(x,exp_e);
    exp = L_deposit_l(exp_e);

    /* calculate (1-normalized_input) */
    nIn = extract_h(L_sub(MAX_32,x));

    /* approximate ln() for fractional part (nIn *c0 + nIn^2*c1 + nIn^3*c2 + ... + nIn^8 *c7) */

    /* iteration 1, no need for accumulation */
    accuRes = L_mult(nIn,ldCoeff[0]);             /* nIn^i * coeff[0] */
    accuSqr = mult(nIn,nIn);                      /* nIn^2, nIn^3 .... */

    /* iteration 2 */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[1]);  /* nIn^i * coeff[1] */
    accuSqr = mult(accuSqr,nIn);                  /* nIn^2, nIn^3 .... */

    /* iteration 3 */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[2]);  /* nIn^i * coeff[2] */
    accuSqr = mult(accuSqr,nIn);                  /* nIn^2, nIn^3 .... */

    /* iteration 4 */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[3]);  /* nIn^i * coeff[3] */
    accuSqr = mult(accuSqr,nIn);                  /* nIn^2, nIn^3 .... */

    /* iteration 5 */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[4]);  /* nIn^i * coeff[4] */
    accuSqr = mult(accuSqr,nIn);                  /* nIn^2, nIn^3 .... */

    /* iteration 6 */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[5]);  /* nIn^i * coeff[5] */
    accuSqr = mult(accuSqr,nIn);                  /* nIn^2, nIn^3 .... */

    /* iteration 7, no need to calculate accuSqr any more */
    accuRes = L_mac(accuRes,accuSqr,ldCoeff[6]);  /* nIn^i * coeff[6] */

    /* ld(fractional part) = ln(fractional part)/ln(2), 1/ln(2) = (1 + 0.44269504) */
    accuRes = L_mac0(L_shr(accuRes,1),extract_h(accuRes),14506);

    accuRes = L_shr(accuRes,LD_DATA_SCALE-1);     /* fractional part/LD_DATA_SCALE */
    exp = L_shl(exp,(31-LD_DATA_SCALE));          /* integer part/LD_DATA_SCALE */
    accuRes = L_sub(accuRes,exp);                 /* result = integer part + fractional part */


    return (accuRes);
}

Word32 BASOP_Util_InvLog2(Word32 x)
{
    Word16  frac;
    Word16  exp;
    Word32  retVal;
    UWord32 index3;
    UWord32 index2;
    UWord32 index1;
    UWord32 lookup3f;
    UWord32 lookup12;
    UWord32 lookup;



    if ( x < FL2WORD32(-31.0/64.0) )
    {

        return 0;
    }
    test();
    if ( (L_sub(x,FL2WORD32(31.0/64.0)) >= 0) || (x == 0) )
    {

        return 0x7FFFFFFF;
    }

    frac   = extract_l(L_and(x,0x3FF));

    index3 = L_and(L_shr(x,10),0x1F);
    index2 = L_and(L_shr(x,15),0x1F);
    index1 = L_and(L_shr(x,20),0x1F);

    exp = extract_l(L_shr(x,25));
    if ( x > 0 )
    {
        exp = sub(31,exp);
    }
    if ( x < 0 )
    {
        exp = negate(exp);
    }

    lookup3f = L_add(exp2x_tab_long[index3],L_shr(Mpy_32_16_1(0x0016302F,frac),1));
    lookup12 = Mpy_32_32(exp2_tab_long[index1],exp2w_tab_long[index2]);
    lookup   = Mpy_32_32(lookup12, lookup3f);

    retVal = L_shr(lookup,sub(exp,3));


    return retVal;
}


Word16 BASOP_Util_Add_MantExp                    /*!< Exponent of result        */
(Word16   a_m,       /*!< Mantissa of 1st operand a */
 Word16   a_e,       /*!< Exponent of 1st operand a */
 Word16   b_m,       /*!< Mantissa of 2nd operand b */
 Word16   b_e,       /*!< Exponent of 2nd operand b */
 Word16  *ptrSum_m)  /*!< Mantissa of result */
{
    Word32 L_lm, L_hm;
    Word16 shift;



    /* Compare exponents: the difference is limited to +/- 15
       The Word16 mantissa of the operand with higher exponent is moved into the low
     part of a Word32 and shifted left by the exponent difference. Then, the
     unshifted mantissa of the operand with the lower exponent is added to the lower
     16 bits. The addition result is normalized and the upper Word16 of the result represents
     the mantissa to return. The returned exponent takes into account all shift operations
     including the final 16-bit extraction.
     Note: The resulting mantissa may be inaccurate in the case, where the mantissa of the operand
           with higher exponent is not really left-aligned, while the mantissa of the operand with
         lower exponent is so. If in such a case, the difference in exponents is more than 15,
         an inaccuracy is introduced.
         Example:
         A: a_e = 20, a_m = 0x0001
         B: b_e =  0, b_m = 0x4000
             correct:      A+B=1*2^20+1*2^14=0x0010.0000+0x0000.4000=0x0010.4000=0x4100*2^6
         previously:   A+B=1*2^20+1*2^14=0x0001+0x0000=0x0001*2^20
         this version: A+B=1*2^20+1*2^14=0x0000.8000+0x0000.4000=0x6000*2^6
    */

    shift = sub(a_e, b_e);
    if (shift >= 0)
        shift = s_min( 15,shift);

    if (shift < 0)
        shift = s_max(-15,shift);
    a_e = s_max(a_e,b_e);
    L_hm = L_deposit_l(a_m);       /* mantissa belonging to higher exponent */
    L_lm = L_deposit_l(a_m);       /* mantissa belonging to lower exponent */
    if (shift >= 0)
        L_lm = L_deposit_l(b_m);
    if (shift < 0)
        L_hm = L_deposit_l(b_m);

    if (shift > 0)
        shift = negate(shift);

    L_hm = L_shr(L_hm,shift);    /* shift left due to negative shift parameter */
    a_e = add(a_e, shift);
    L_hm = L_add(L_hm, L_lm);
    shift = norm_l(L_hm);
    L_hm = L_shl(L_hm,shift);
    *ptrSum_m = extract_h(L_hm);
    move16();

    a_e = sub(a_e,shift);
    if (L_hm)
        a_e = add(a_e,16);
    return (a_e);
}


void BASOP_Util_Divide_MantExp (Word16   a_m,          /*!< Mantissa of dividend a */
                                Word16   a_e,          /*!< Exponent of dividend a */
                                Word16   b_m,          /*!< Mantissa of divisor b */
                                Word16   b_e,          /*!< Exponent of divisor b */
                                Word16  *ptrResult_m,  /*!< Mantissa of quotient a/b */
                                Word16  *ptrResult_e   /*!< Exponent of quotient a/b */
                               )
{
    Word16 index, frac;
    Word16 preShift, postShift;
    Word16 m;
    Word32 m32;



    assert(b_m != 0);

    /* normalize b */
    preShift = norm_s(b_m);
    m = shl(b_m, preShift);

    /* make b positive */
    BASOP_SATURATE_WARNING_OFF;
    if (m < 0) m = negate(m);
    BASOP_SATURATE_WARNING_ON;

    /* get table index (upper 6 bits minus 16) */
    /* index = (m >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), m, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = shl(s_and(m, 0x1FF), 1); /* Q10 */

    /* interpolate 1/b */
    m = msu_r(InvTable[index], InvDiffTable[index], frac);

    /* restore sign */
    if (b_m < 0) m = negate(m);

    /* multiply with a */
    m32 = L_mult(a_m, m);

    /* normalize result */
    postShift = norm_l(m32);
    m = round_fx(L_shl(m32, postShift));

    /* exponent */
    *ptrResult_e = sub(add(add(a_e, sub(1, b_e)), preShift), postShift);
    move16();

    *ptrResult_m = m;
    move16();

}


/* local function for Sqrt16 and Sqrt16norm */
static Word16 Sqrt16_common(Word16 m,
                            Word16 e)
{
    Word16 index, frac;

    assert((m >= 0x4000) || (m == 0));

    /* get table index (upper 6 bits minus 32) */
    /* index = (m >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), m, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = s_and(m, 0x1FF); /* Q9 */

    /* interpolate */
    if (m != 0)
    {
        BASOP_SATURATE_WARNING_OFF;
        m = mac_r(SqrtTable[index], SqrtDiffTable[index], frac);
        BASOP_SATURATE_WARNING_ON;
    }

    /* handle odd exponents */
    if (s_and(e, 1) != 0) m = mult_r(m, 0x5a82);

    return m;
}

/* local function for Sqrt32 and Sqrt32norm */
static Word32 Sqrt32_common(Word32 m,
                            Word16 e)
{
    Word16 m16, index, frac;

    assert((m >= 0x40000000) || (m == 0));

    m16 = round_fx(m);

    /* get table index (upper 6 bits minus 32) */
    /* index = (m16 >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), m16, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = s_and(m16, 0x1FF); /* Q9 */

    /* interpolate */
    if (m != 0)
    {
        BASOP_SATURATE_WARNING_OFF;
        m = L_mac(SqrtTable[index], SqrtDiffTable[index], frac);
        BASOP_SATURATE_WARNING_ON;
    }

    /* handle odd exponents */
    if (s_and(e, 1) != 0) m = Mpy_32_16_1(m, 0x5a82);

    return m;
}

/* local function for ISqrt16 and ISqrt16norm */
static Word16 ISqrt16_common(Word16 m,
                             Word16 e)
{
    Word16 index, frac;

    assert(m >= 0x4000);

    /* get table index (upper 6 bits minus 32) */
    /* index = (m >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), m, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = s_and(m, 0x1FF); /* Q9 */

    /* interpolate */
    m = msu_r(ISqrtTable[index], ISqrtDiffTable[index], frac);

    /* handle even exponents */
    if (s_and(e, 1) == 0) m = mult_r(m, 0x5a82);

    return m;
}

/* local function for ISqrt32 and ISqrt32norm */
static Word32 ISqrt32_common(Word32 m,
                             Word16 e)
{
    Word16 m16, index, frac;

    assert(m >= 0x40000000);

    m16 = round_fx(m);

    /* get table index (upper 6 bits minus 32) */
    /* index = (m16 >> 25) - 32; */
    index = mac_r(-32768 - (32 << 16), m16, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = s_and(m16, 0x1FF); /* Q9 */

    /* interpolate */
    m = L_msu(ISqrtTable[index], ISqrtDiffTable[index], frac);

    /* handle even exponents */
    if (s_and(e, 1) == 0) m = Mpy_32_16_1(m, 0x5a82);

    return m;
}


Word16 Sqrt16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{
    Word16 preShift, e;

    assert(mantissa >= 0);

    /* normalize */
    preShift = norm_s(mantissa);

    e = sub(*exponent, preShift);
    mantissa = shl(mantissa, preShift);

    /* calc mantissa */
    mantissa = Sqrt16_common(mantissa, e);

    /* e = (e + 1) >> 1 */
    *exponent = mult_r(e, 1 << 14);
    move16();

    return mantissa;
}

Word16 Sqrt16norm(              /*!< output mantissa */
    Word16 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{

    assert((mantissa >= 0x4000) || (mantissa == 0));

    /* calc mantissa */
    mantissa = Sqrt16_common(mantissa, *exponent);

    /* e = (e + 1) >> 1 */
    *exponent = mult_r(*exponent, 1 << 14);
    move16();

    return mantissa;
}


Word16 ISqrt16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{
    Word16 preShift, e;

    assert(mantissa > 0);

    /* normalize */
    preShift = norm_s(mantissa);

    e = sub(*exponent, preShift);
    mantissa = shl(mantissa, preShift);

    /* calc mantissa */
    mantissa = ISqrt16_common(mantissa, e);

    /* e = (2 - e) >> 1 */
    *exponent = msu_r(1L << 15, e, 1 << 14);
    move16();

    return mantissa;
}


Word32 Sqrt32(                  /*!< output mantissa */
    Word32 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{
    Word16 preShift, e;

    assert(mantissa >= 0);

    /* normalize */
    preShift = norm_l(mantissa);

    e = sub(*exponent, preShift);
    mantissa = L_shl(mantissa, preShift);

    /* calc mantissa */
    mantissa = Sqrt32_common(mantissa, e);

    /* e = (e + 1) >> 1 */
    *exponent = mult_r(e, 1 << 14);
    move16();

    return mantissa;
}

Word32 Sqrt32norm(              /*!< output mantissa */
    Word32 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{

    assert((mantissa >= 0x40000000) || (mantissa == 0));

    /* calc mantissa */
    mantissa = Sqrt32_common(mantissa, *exponent);

    /* e = (e + 1) >> 1 */
    *exponent = mult_r(*exponent, 1 << 14);
    move16();

    return mantissa;
}


Word32 ISqrt32(                  /*!< output mantissa */
    Word32 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{
    Word16 preShift, e;

    assert(mantissa > 0);

    /* normalize */
    preShift = norm_l(mantissa);

    e = sub(*exponent, preShift);
    mantissa = L_shl(mantissa, preShift);

    /* calc mantissa */
    mantissa = ISqrt32_common(mantissa, e);

    /* e = (2 - e) >> 1 */
    *exponent = msu_r(1L << 15, e, 1 << 14);
    move16();

    return mantissa;
}

Word32 ISqrt32norm(              /*!< output mantissa */
    Word32 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{

    assert(mantissa >= 0x40000000);

    /* calc mantissa */
    mantissa = ISqrt32_common(mantissa, *exponent);

    /* e = (2 - e) >> 1 */
    *exponent = msu_r(1L << 15, *exponent, 1 << 14);
    move16();

    return mantissa;
}


Word16 Inv16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
)
{
    Word16 index, frac;
    Word16 preShift;
    Word16 m, e;



    assert(mantissa != 0);

    /* absolute */
    BASOP_SATURATE_WARNING_OFF;
    m = abs_s(mantissa);
    BASOP_SATURATE_WARNING_ON;

    /* normalize */
    preShift = norm_s(m);

    e = sub(*exponent, preShift);
    m = shl(m, preShift);

    /* get table index (upper 6 bits minus 32) */
    /* index = (m >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), m, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = shl(s_and(m, 0x1FF), 1); /* Q10 */

    /* interpolate */
    m = msu_r(InvTable[index], InvDiffTable[index], frac);

    /* restore sign */
    if (mantissa < 0) m = negate(m);

    /* e = 1 - e */
    *exponent = sub(1, e);
    move16();

    return m;
}


void BASOP_Util_Sqrt_InvSqrt_MantExp (Word16 mantissa,      /*!< mantissa */
                                      Word16 exponent,      /*!< expoinent */
                                      Word16 *sqrt_mant,    /*!< Pointer to sqrt mantissa */
                                      Word16 *sqrt_exp,     /*!< Pointer to sqrt exponent */
                                      Word16 *isqrt_mant,   /*!< Pointer to 1/sqrt mantissa */
                                      Word16 *isqrt_exp     /*!< Pointer to 1/sqrt exponent */
                                     )
{
    Word16 index, frac;
    Word16 preShift;
    Word16 m, mi, e_odd;



    assert(mantissa > 0);

    /* normalize */
    preShift = norm_s(mantissa);

    exponent = sub(exponent, preShift);
    mantissa = shl(mantissa, preShift);

    /* get table index (upper 6 bits minus 32) */
    /* index = (m >> 9) - 32; */
    index = mac_r(-32768 - (32 << 16), mantissa, 1 << 6);

    /* get fractional part for interpolation (lower 9 bits) */
    frac = s_and(mantissa, 0x1FF); /* Q9 */

    /* interpolate */
    BASOP_SATURATE_WARNING_OFF;
    m = mac_r(SqrtTable[index], SqrtDiffTable[index], frac);
    mi = msu_r(ISqrtTable[index], ISqrtDiffTable[index], frac);
    BASOP_SATURATE_WARNING_ON;

    /* handle even/odd exponents */
    e_odd = s_and(exponent, 1);
    if (e_odd != 0) m = mult_r(m, 0x5a82);
    if (e_odd == 0) mi = mult_r(mi, 0x5a82);

    /* e = (e + 1) >> 1 */
    *sqrt_exp = mult_r(exponent, 1 << 14);
    move16();

    /* e = (2 - e) >> 1 */
    *isqrt_exp = msu_r(1L << 15, exponent, 1 << 14);
    move16();

    /* Write result */
    *sqrt_mant = m;
    move16();
    *isqrt_mant = mi;
    move16();

}

/********************************************************************/
/*!
  \brief   Calculates the scalefactor needed to normalize input array

    The scalefactor needed to normalize the Word16 input array is returned <br>
    If the input array contains only '0', a scalefactor 0 is returned <br>
    Scaling factor is determined wrt a normalized target x: 16384 <= x <= 32767 for positive x <br>
    and   -32768 <= x <= -16384 for negative x
*/

Word16 getScaleFactor16(                 /* o: measured headroom in range [0..15], 0 if all x[i] == 0 */
    const Word16 *x,      /* i: array containing 16-bit data */
    const Word16 len_x)   /* i: length of the array to scan  */
{
    Word16 i, i_min, i_max;
    Word16 x_min, x_max;



    x_max = 0;
    move16();
    x_min = 0;
    move16();
    FOR (i = 0; i < len_x; i++)
    {
        if (x[i] >= 0)
            x_max = s_max(x_max,x[i]);
        if (x[i] < 0)
            x_min = s_min(x_min,x[i]);
    }

    i_max = 0x10;
    move16();
    i_min = 0x10;
    move16();

    if (x_max != 0)
        i_max = norm_s(x_max);

    if (x_min != 0)
        i_min = norm_s(x_min);

    i = s_and(s_min(i_max, i_min),0xF);


    return i;
}


/********************************************************************/
/*!
  \brief   Calculates the scalefactor needed to normalize input array

    The scalefactor needed to normalize the Word32 input array is returned <br>
    If the input array contains only '0', a scalefactor 0 is returned <br>
    Scaling factor is determined wrt a normalized target x: 1073741824 <= x <= 2147483647 for positive x <br>
    and   -2147483648 <= x <= -1073741824 for negative x
*/

Word16 getScaleFactor32(               /* o: measured headroom in range [0..31], 0 if all x[i] == 0 */
    const Word32 *x,      /* i: array containing 32-bit data */
    const Word16 len_x)   /* i: length of the array to scan  */
{
    Word16 i, i_min, i_max;
    Word32 x_min, x_max;



    x_max = L_add(0, 0);
    x_min = L_add(0, 0);
    FOR (i = 0; i < len_x; i++)
    {
        if (x[i] >= 0)
            x_max = L_max(x_max,x[i]);
        if (x[i] < 0)
            x_min = L_min(x_min,x[i]);
    }

    i_max = 0x20;
    move16();
    i_min = 0x20;
    move16();

    if (x_max != 0)
        i_max = norm_l(x_max);

    if (x_min != 0)
        i_min = norm_l(x_min);

    i = s_and(s_min(i_max, i_min),0x1F);


    return i;
}

Word16 normalize16(Word16 mantissa, Word16 *pexponent)
{
    Word16 tmp;

    tmp = norm_s(mantissa);
    mantissa = shl(mantissa, tmp);
    move16();
    *pexponent = sub(*pexponent, tmp);


    return mantissa;
}
Word16 divide3216(Word32 x, Word16 y)
{
    Word16 z;


    z = 0;
    move16();
    if (0 == y)
    {
        return 0x7fff;
    }

    IF (x != 0)
    {
        Word16 den, sign;
        Word32 num;
        num = L_abs(x);
        den = abs_s(y);

        sign = s_and(s_xor(extract_h(x),y),-32768 /* 0x8000 */);

        z = div_l(num,den);
        if (0 != sign)
        {
            z = negate(z);
        }
    }


    return z;
}

Word16 divide1616(Word16 x, Word16 y)
{
    Word16 z, num, den, sign;


    num = abs_s(x);
    den = abs_s(y);

    sign = s_and(s_xor(x,y),-32768 /* 0x8000 */);

    move16();
    z = 0x7fff;
    if ( sub(num, den) < 0)
        z = div_s(num,den);

    if (0 != sign)
    {
        z = negate(z);
    }


    return z;
}

Word16 divide3232(Word32 L_num, Word32 L_denom)
{
    Word16 z;
    Word32 sign;


    sign = L_and(L_xor(L_num,L_denom),0x80000000);

    L_num   = L_abs(L_num);
    L_denom = L_abs(L_denom);

    /* limit the range of denominator to Word16 */
    z = s_min(norm_l(L_num),norm_l(L_denom));
    L_num   = L_shl(L_num,z);
    L_denom = L_shl(L_denom,z);

    /* round_fx instead of extract_h improves spectral distortion in E_UTIL_lev_dur (schur version). */
    z = div_l(L_num,round_fx(L_denom));
    if (0 != sign)
    {
        z = negate(z);
    }


    return z;
}

Word16 BASOP_Util_Divide3232_uu_1616_Scale(Word32 x, Word32 y, Word16 *s)
{
    Word16 z;
    Word16 sx;
    Word16 sy;
    Word16 x16;
    Word16 y16;



    assert(x >= 0);
    assert(y >  0);

    if ( x == 0 )
    {
        *s = 0;
        move16();


        return (0);
    }

    sx = norm_l(x);
    sy = norm_l(y);

    x16 = extract_h(L_shl(x,sx));
    y16 = extract_h(L_shl(y,sy));

    if(sub(x16,y16) > 0)
    {
        sx = sub(sx,1);
    }

    if(sub(y16,x16) < 0)
    {
        x16 = mult_r(x16,0x4000);
    }


    z = div_s(x16,y16);
    move16();
    *s = sub(sy,sx);


    return (z);
}

Word16 BASOP_Util_Divide3232_Scale(Word32 x, Word32 y, Word16 *s)
{
    Word16 z;
    Word16 sy;



    sy = norm_l(y);
    if (sy > 0)
    {
        sy = sub(sy,1);
    }
    y  = L_shl(y,sy);

    z = BASOP_Util_Divide3216_Scale(x, extract_h(y), s);
    move16();
    *s = add(*s,sy);


    return (z);
}

Word16 BASOP_Util_Divide3216_Scale(   /* o: result of division x/y, not normalized  */
    Word32 x,                         /* i: numerator, signed                       */
    Word16 y,                         /* i: denominator, signed                     */
    Word16 *s)                        /* o: scaling, 0, if x==0                     */
{
    Word16 z;
    Word16 sx;
    Word16 sy;
    Word16 sign;



    /*assert (x > (Word32)0);
    assert (y >= (Word16)0);*/

    /* check, if numerator equals zero, return zero then */
    IF ( x == (Word32)0 )
    {
        move16();
        *s = 0;


        return ((Word16)0);
    }

    sign = s_xor(extract_h(x),y);  /* just to exor the sign bits */
    BASOP_SATURATE_WARNING_OFF
    x = L_abs(x);
    y = abs_s(y);
    BASOP_SATURATE_WARNING_ON
    sx = sub(norm_l(x),1);
    x  = L_shl(x,sx);
    sy = norm_s(y);
    y  = shl(y,sy);
    *s = sub(sy,sx);
    move16();

    z = div_s(round_fx(x),y);

    if ( sign < 0 )                /* if sign bits differ, negate the result */
    {
        z = negate(z);
    }


    return z;
}

Word16 BASOP_Util_Divide1616_Scale(Word16 x, Word16 y, Word16 *s)
{
    Word16 z;
    Word16 sx;
    Word16 sy;
    Word16 sign;



    /* assert (x >= (Word16)0); */
    assert (y != (Word16)0);

    sign = 0;
    move16();

    IF ( x < 0 )
    {
        x = negate(x);
        sign = s_xor(sign,1);
    }

    IF ( y < 0 )
    {
        y = negate(y);
        sign= s_xor(sign,1);
    }

    IF ( x == (Word16)0 )
    {
        move16();
        *s = 0;


        return ((Word16)0);
    }

    sx = norm_s(x);
    x  = shl(x,sx);
    x  = shr(x,1);
    move16();
    *s = sub(1,sx);

    sy = norm_s(y);
    y  = shl(y,sy);
    move16();
    *s = add(*s,sy);

    z = div_s(x,y);

    if ( sign != 0 )
    {
        z = negate(z);
    }


    return z;
}

void copyWord8(const Word8 *src, Word8 *dst, const Word32 n)
{
    Word32 i;


    FOR (i=0; i<n; i++)
    {
        dst[i] = src[i];
        move16();
    }

}



void set_zero_Word8(Word8 X[], Word32 n)
{
    Word32 i;


    FOR (i=0; i<n; i++)
    {
        X[i] = 0;
        move16();
    }

}


Word32 L_mult0_3216(Word32 x, Word16 y)
{
    UWord16 mpy_low16;
    Word32  mpy_high32;


    Mpy_32_16_ss(x, y, &mpy_high32, &mpy_low16);

    mpy_high32 = L_add(L_shl(mpy_high32,15),L_lshr(L_deposit_h(mpy_low16),17) );


    return mpy_high32;
}

Word16 BASOP_util_norm_l_dim2_cplx (const Word32 * const *re, /*!< Real part of 32 Bit input */
                                    const Word32 * const *im, /*!< Imag part if 32 Bit input */
                                    Word16 startBand, /*!< start band of cplx data   */
                                    Word16 stopBand,  /*!< stop band of cplx data    */
                                    Word16 startSlot, /*!< start slot of cplx data   */
                                    Word16 stopSlot   /*!< stop slot of cplx data    */
                                   )
{
    Word16  col;
    Word16  band;
    Word16  maxShift;
    Word32  maxVal;



    maxVal = L_deposit_l(1);

    FOR (col=startSlot; col < stopSlot; col++)
    {
        FOR (band=startBand; band < stopBand; band++)
        {
            maxVal = L_max(maxVal,L_abs(re[col][band]));
            maxVal = L_max(maxVal,L_abs(im[col][band]));
        }
    }
    maxShift = norm_l(maxVal);


    return (maxShift);
}

Word16 BASOP_util_norm_s_bands2shift (Word16 x)
{
    Word16 shift;

    shift = sub(WORD16_BITS-1,norm_s(negate(x)));

    return (shift);
}


#define SINETAB SineTable512
#define LD 9

/*
 * Calculates coarse lookup values for sine/cosine and residual angle.
 * \param x angle in radians with exponent = 2 or as radix 2 with exponent = 0.
 * \param scale shall always be 2
 * \param sine pointer to where the sine lookup value is stored into
 * \param cosine pointer to where the cosine lookup value is stored into
 * \param flag_radix2 flag indicating radix 2 angle if non-zero.
 */
static Word16 fixp_sin_cos_residual_16(Word16 x, const Word16 scale, Word16 *sine, Word16 *cosine, Word8 flag_radix2)
{
    Word16 residual;
    Word16 s;
    Word16 ssign;
    Word16 csign;
    Word16 tmp, cl, sl;
    const Word16 shift = 15-LD-1-scale;

    sl = 0;         /* to avoid compilation warnings */
    cl = 0;         /* to avoid compilation warnings */


    if (flag_radix2 == 0)
    {
        x = mult_r(x, FL2WORD16(1.0/EVS_PI));
    }
    s = shr(x, shift);

    residual = s_and(x, (1<<shift)-1);
    /* We assume "2+scale" is a constant */
    residual = shl(residual,2+scale);
    residual = mult_r(residual,FL2WORD16(EVS_PI/4.0));

    /* Sine sign symmetry */
    ssign = s_and(s, (1<<LD)<<1);

    /* Cosine sign symmetry */
    csign = s_and(add(s, (1<<LD)), (1<<LD)<<1);

    /* Modulo PI */
    s = s_and(s, (2<<LD)-1);

    /* PI/2 symmetry */
    s = s_min(s, sub(2<<LD, s));

    {
        tmp = s_min(sub(1<<LD, s), s);
        s = sub(tmp,s);

        if ( ! s)
        {
            move16();
            sl = SINETAB[tmp].v.im;
        }
        if (! s)
        {
            move16();
            cl = SINETAB[tmp].v.re;
        }
        if (s)
        {
            move16();
            sl = SINETAB[tmp].v.re;
        }
        if (s)
        {
            move16();
            cl = SINETAB[tmp].v.im;
        }

        if (ssign)
        {
            sl = negate(sl);
        }
        if (csign)
        {
            cl = negate(cl);
        }

        move16();
        move16();
        *sine   = sl;
        *cosine = cl;
    }

    return residual;
}

Word16 getCosWord16(Word16 theta)
{
    Word16 result, residual, sine, cosine;

    residual = fixp_sin_cos_residual_16(theta, 2, &sine, &cosine, 0);
    /* This negation prevents the subsequent addition from overflow */
    /* The negation cannot overflow, sine is in range [0x0..0x7FFF] */
    sine = negate(sine);
    result = mac_r(L_mult0(sine, residual), cosine, 16384);


    return result;
}

Word16 getCosWord16R2(Word16 theta)
{
    Word16 result, residual, sine, cosine;

    residual = fixp_sin_cos_residual_16(theta, 1, &sine, &cosine, 1);
    /* This negation prevents the subsequent addition from overflow */
    /* The negation cannot overflow, sine is in range [0x0..0x7FFF] */
    BASOP_SATURATE_WARNING_OFF
    sine = negate(sine);
    result = msu_r(L_mult(sine, residual), cosine, -32768);
    BASOP_SATURATE_WARNING_ON


    return result;
}

/*
 * Calculate Integer Square Root of 'val'. This is the equivalent of (int)sqrt(val);
 * The return value will be truncated to the lowest integer (throwing away the fractionnal part.
 *
 * There are many ways to do this. The approach here is to use a simple function to get a
 * 1st estimate of (int)sqrt(val) and then correct this estimate if it is too low or too high.
 *
 * Using Word16, the range of 'val' is limited to roughly 2^30.
 *
 * Complexity: Worst=31Clks, Best=27Clks
 */
Word16 getSqrtWord32(Word32 val)
{
    Word32 L_temp, L_temp2;
    Word16 temp, temp2;
    Word16 exp, exp2;

    /* Calc Approximation */
    exp2 = norm_l(val);
    L_temp2 = L_shl(val, exp2);
    exp = sub(31-32, exp2);
    L_temp = Isqrt_lc(L_temp2, &exp); /* 12 clks */

    temp = round_fx(L_temp);
    L_temp = Mult_32_16(L_temp2, temp); /* 2 clks */

    L_temp = L_shl(L_temp, sub(exp, exp2));

    /* The Approximation Error Range is -1..+7, so Too Low by 1 or Up to Too High by 7 */
    temp = round_fx(L_temp);

    /* Too High? */
    if (L_msu0(val, temp, temp) < 0)
    {
        /* Reduce by 2 */
        temp = sub(temp, 2);
    }
    /* Too High? */
    if (L_msu0(val, temp, temp) < 0)
    {
        /* Reduce by 2 */
        temp = sub(temp, 2);
    }
    /* Too High? */
    if (L_msu0(val, temp, temp) < 0)
    {
        /* Reduce by 2 */
        temp = sub(temp, 2);
    }
    /* Too High? */
    if (L_msu0(val, temp, temp) < 0)
    {
        /* Reduce by 1 */
        temp = sub(temp, 1);
    }

    /* Try +1 */
    temp2 = add(temp, 1);
    /* It fits? */
    if (L_msu0(val, temp2, temp2) >= 0)
    {
        /* Yes */
        temp = temp2;
        move16();
    }
    return temp;
}
Word16 findIndexOfMinWord32(Word32 *x, const Word16 len)
{
    Word16 i, indx;


    indx = 0;
    move16();
    FOR (i = 1; i < len; i++)
    {
        if (L_sub(x[i],x[indx]) < 0)
        {
            indx = i;
            move16();
        }
    }


    return indx;
}


Word16 imult1616(Word16 x, Word16 y)
{
    assert((int)x * (int)y < 32768 && (int)x * (int)y >= -32768);
    return extract_l(L_mult0(x, y));
}

Word32 imult3216(Word32 x, Word16 y)
{
    Word32 mh;
    UWord16 ml;

    Mpy_32_16_ss(x, y, &mh, &ml);

    mh = L_shl(mh, 15);
    ml = lshr(ml, 1);

    return L_or(mh, L_deposit_l(ml));
}

Word16 idiv1616U(Word16 x, Word16 y)
{
    Word16 s;


    /* make y > x */
    s = add(sub(norm_s(y), norm_s(x)), 1);
    s = s_max(s, 0);

    BASOP_SATURATE_WARNING_OFF
    y = shl(y, s);
    BASOP_SATURATE_WARNING_ON

    /* divide and shift */
    y = shr(div_s(x, y), sub(15, s));


    return y;
}

Word16 idiv1616(Word16 x, Word16 y)
{
    Word16 s, num, den, sign;


    num = abs_s(x);
    den = abs_s(y);

    sign = s_and(s_xor(x,y),-32768 /* 0x8000 */);

    /* make num > den */
    s = add(sub(norm_s(den), norm_s(num)), 1);
    s = s_max(s, 0);

    den = shl(den, s);

    /* divide and shift */
    y = shr(div_s(num, den), sub(15, s));

    if (0 != sign)
    {
        y = negate(y);
    }


    return y;
}

Word32 norm_llQ31(        /* o : normalized result              Q31 */
    Word32 L_c,          /* i : upper bits of accu             Q-1 */
    Word32 L_sum,        /* i : lower bits of accu, unsigned   Q31 */
    Word16 * exp         /* o : exponent of result in [-32,31]  Q0 */
)
{
    Word16 i;
    Word32 L_tmp;

    /* Move MSBit of L_sum into L_c */
    Carry = 0;
    L_tmp = L_add_c(L_sum, L_sum);    /* L_tmp = L_sum << 1         */
    L_c = L_add_c(L_c,L_c);
    L_add(0,0);
    test();
    IF ((L_c != (Word32) 0L) && (L_c != (Word32) 0xFFFFFFFFL))
    {
        i = norm_l(L_c);
        L_c = L_shl(L_c,i);
        i = sub(31,i);                  /* positive exponent  */
        L_sum = L_lshr(L_tmp, 1);       /* L_sum with MSBit=0 */
        L_sum = L_lshr(L_sum, i);
        L_sum = L_add(L_c,L_sum);
    }
    ELSE
    {
        i = -32;
        move16();      /* default exponent, if total sum=0 */
        IF (L_sum)
        {
            i = norm_l(L_sum);
            L_sum = L_shl(L_sum,i);
            i = negate(i);                  /* negative or zero exponent */
        }
    }
    *exp = i;
    move16();
    return L_sum;
}


Word32 Dot_product16HQ(   /* o : normalized result              Q31 */
    const Word32 L_off,  /* i : initial sum value               Qn */
    const Word16 x[],    /* i : x vector                        Qn */
    const Word16 y[],    /* i : y vector                        Qn */
    const Word16 lg,     /* i : vector length, range [0..7FFF]  Q0 */
    Word16 * exp         /* o : exponent of result in [-32,31]  Q0 */
)
{
    Word16 i;
    Word32 L_sum, L_c, L_test;
    /* Clear carry flag and init sum */
    Carry = 0;
    L_c = L_add(0,0);
    L_sum = L_macNs(L_off,0,0);
    if (L_sum > 0)
        L_c = L_macNs(L_c,0,0);
    if (L_sum < 0)
        L_c = L_msuNs(L_c,0,0);

    FOR (i=0; i < lg; i++)
    {
        BASOP_SATURATE_WARNING_OFF /*in case of both multiplicands being -32768, overflow occurs - not severe*/
        L_test = L_mult(x[i], y[i]);
        BASOP_SATURATE_WARNING_ON
        Carry = 0;
        L_sum = L_macNs(L_sum, x[i], y[i]);
        Overflow = 0;  /* to avoid useless warning in L_macNs/L_msuNs calling L_mult */
        if (L_test >= 0)
            L_c = L_macNs(L_c,0,0);
        if (L_test < 0)
            L_c = L_msuNs(L_c,0,0);
    }
    L_sum = norm_llQ31(L_c,L_sum,exp);
    return L_sum;
}

Word32 Norm32Norm(const Word32 *x, const Word16 headroom, const Word16 length, Word16 *result_e)
{
    Word32 L_tmp, L_tmp2;
    Word16 i, shift, tmp;

    move16();
    shift = headroom;

    L_tmp = L_deposit_l(0);

    FOR (i=0; i<length; i++)
    {
        L_tmp2 = L_sub(L_tmp, 0x40000000);
        if (L_tmp2 >= 0) shift = sub(shift,1);
        if (L_tmp2 >= 0) L_tmp = L_shr(L_tmp, 2);

        tmp = round_fx(L_shl(x[i], shift));
        L_tmp = L_mac0(L_tmp, tmp, tmp); /* exponent = (1-shift*2) , Q(30+shift*2) */
    }

    move16();
    *result_e = sub(1, shl(shift,1));

    return L_tmp;
}

Word32 Dot_productSq16HQ( /* o : normalized result              Q31 */
    const Word32 L_off,  /* i : initial sum value               Qn */
    const Word16 x[],    /* i : x vector                        Qn */
    const Word16 lg,     /* i : vector length, range [0..7FFF]  Q0 */
    Word16 * exp         /* o : exponent of result in [-32,31]  Q0 */
)
{
    Word16 i;
    Word32 L_sum, L_c;
    /* Clear carry flag and init sum */
    Carry = 0;
    L_c = L_add(0,0);
    L_sum = L_macNs(L_off,0,0);
    if (L_sum > 0)
        L_c = L_macNs(L_c,0,0);
    if (L_sum < 0)
        L_c = L_msuNs(L_c,0,0);

    FOR (i=0; i < lg; i++)
    {
        Carry = 0;
        BASOP_SATURATE_WARNING_OFF /*multiplication of -32768 * -32768 throws an overflow, but is not critical*/
        L_sum = L_macNs(L_sum, x[i], x[i]);
        BASOP_SATURATE_WARNING_ON
        Overflow = 0;  /* to avoid useless warning in L_macNs calling L_mult */
        L_c = L_macNs(L_c,0,0);
    }
    L_sum = norm_llQ31(L_c,L_sum,exp);
    return L_sum;
}

Word32 dotp_s_fx(const Word16 *x, const Word16 *y, const Word16 n, Word16 s)
{
    Word16 i;
    Word16 n2;
    Word32 L_tmp;
    Word32 L_sum;


    L_sum = L_add(0,0);

    n2 = shr(n,1);

    s = sub(s,1);

    FOR (i=0; i < n2; i++)
    {
        L_tmp = L_mult0(x[2*i], y[2*i]);
        L_tmp = L_mac0(L_tmp, x[2*i+1], y[2*i+1]);
        L_sum = L_add(L_sum, L_shr(L_tmp, s));
    }

    IF ( s_and(n,1) )
    {
        L_tmp = L_mult0(x[n-1], y[n-1]);
        L_sum = L_add(L_sum, L_shr(L_tmp, s));
    }


    return L_sum;
}

Word32 Sum32_scale(       /* o  : the sum of the elements of the vector */
    const Word32 *vec,    /* i  : input vector                          */
    const Word16 lvec,    /* i  : length of input vector                */
    const Word16 scale
)
{
    Word16 i;
    Word32 Ltmp;

    assert ( lvec > 1);

    Ltmp = L_shr(vec[0], scale);
    FOR (i = 1; i < lvec; i++)
    {
        Ltmp = L_add(Ltmp, L_shr(vec[i], scale));
    }
    return Ltmp;
}


Word32 BASOP_util_Pow2(
    const Word32 exp_m, const Word16 exp_e,
    Word16 *result_e
)
{
    static const Word16 pow2Coeff[8] =
    {
        FL2WORD16(0.693147180559945309417232121458177),    /* ln(2)^1 /1! */
        FL2WORD16(0.240226506959100712333551263163332),    /* ln(2)^2 /2! */
        FL2WORD16(0.0555041086648215799531422637686218),   /* ln(2)^3 /3! */
        FL2WORD16(0.00961812910762847716197907157365887),  /* ln(2)^4 /4! */
        FL2WORD16(0.00133335581464284434234122219879962),  /* ln(2)^5 /5! */
        FL2WORD16(1.54035303933816099544370973327423e-4),  /* ln(2)^6 /6! */
        FL2WORD16(1.52527338040598402800254390120096e-5),  /* ln(2)^7 /7! */
        FL2WORD16(1.32154867901443094884037582282884e-6)   /* ln(2)^8 /8! */
    };

    Word32 frac_part, tmp_frac, result_m;
    Word16 int_part;

    int_part = 0;         /* to avoid compilation warnings */
    frac_part = 0;        /* to avoid compilation warnings */

    IF (exp_e > 0)
    {
        /* "+ 1" compensates L_shr(,1) of the polynomial evaluation at the loop end. */

        int_part = add(1,extract_l(L_shr(exp_m, sub(31, exp_e))));
        frac_part = L_lshl(exp_m, exp_e);
        frac_part = L_and(0x7FFFFFFF, frac_part);
    }
    if (exp_e <= 0)
        frac_part = L_shl(exp_m, exp_e);
    if (exp_e <= 0)
    {
        int_part = 1;
        move16();
    }

    /* Best accuracy is around 0, so try to get there with the fractional part. */
    IF( (tmp_frac = L_sub(frac_part,FL2WORD32(0.5))) >= 0)
    {
        int_part = add(int_part, 1);
        frac_part = L_sub(tmp_frac,FL2WORD32(0.5));
    }
    ELSE IF( (tmp_frac = L_add(frac_part,FL2WORD32(0.5))) < 0)
    {
        int_part = sub(int_part, 1);
        frac_part = L_add(tmp_frac,FL2WORD32(0.5));
    }

    /* Evaluate taylor polynomial which approximates 2^x */
    {
        Word32 p;
        Word16 i;


        /* First taylor series coefficient a_0 = 1.0, scaled by 0.5 due to L_shr(,1). */
        result_m = L_add(FL2WORD32(1.0/2.0),L_shr(Mpy_32_16_1(frac_part, pow2Coeff[0]), 1));
        p = Mpy_32_32(frac_part, frac_part);
        FOR (i = 1; i < 7; i++)
        {
            /* next taylor series term: a_i * x^i, x=0 */
            result_m = L_add(result_m, L_shr(Mpy_32_16_1(p, pow2Coeff[i]), 1));
            p = Mpy_32_32(p, frac_part);
        }
        result_m = L_add(result_m, L_shr(Mpy_32_16_1(p, pow2Coeff[i]), 1));
    }
    *result_e = int_part;
    move16();
    return result_m;
}
Word16 findIndexOfMaxWord32(Word32 *x, const Word16 len)
{
    Word16 i, indx;


    indx = 0;
    move16();
    FOR (i = 1; i < len; i++)
    {
        if (L_sub(x[i],x[indx]) > 0)
        {
            indx = i;
            move16();
        }
    }


    return indx;
}


Word16 getNormReciprocalWord16(Word16 x)
{
    assert(x <= (Word16)(sizeof(BASOP_util_normReciprocal)/sizeof(BASOP_util_normReciprocal[0])));
    return extract_h(BASOP_util_normReciprocal[x]);
}
Word16 getNormReciprocalWord16Scale(Word16 x, Word16 s)
{
    assert(x <= (Word16)(sizeof(BASOP_util_normReciprocal)/sizeof(BASOP_util_normReciprocal[0])));
    return round_fx(L_shl(BASOP_util_normReciprocal[x],s));
}

Word32 BASOP_Util_fPow(
    Word32 base_m, Word16 base_e,
    Word32 exp_m, Word16 exp_e,
    Word16 *result_e
)
{

    Word16 ans_lg2_e, base_lg2_e;
    Word32 base_lg2_m, ans_lg2_m, result_m;
    Word16 shift;


    test();
    IF ((base_m == 0) && (exp_m != 0))
    {
        *result_e = 0;
        move16();
        return 0;
    }
    /* Calc log2 of base */
    shift      = norm_l(base_m);
    base_m     = L_shl(base_m, shift);
    base_e     = sub(base_e, shift);
    base_lg2_m = BASOP_Util_Log2(base_m);

    /* shift: max left shift such that neither base_e or base_lg2_m saturate. */
    shift = sub(s_min(norm_s(base_e), WORD16_BITS-1-LD_DATA_SCALE), 1);
    /* Compensate shift into exponent of result. */
    base_lg2_e = sub(WORD16_BITS-1, shift);
    base_lg2_m = L_add(L_shr(base_lg2_m, sub(WORD16_BITS-1-LD_DATA_SCALE, shift)), L_deposit_h(shl(base_e, shift)));

    /* Prepare exp */
    shift = norm_l(exp_m);
    exp_m = L_shl(exp_m, shift);
    exp_e = sub(exp_e, shift);

    /* Calc base pow exp */
    ans_lg2_m = Mpy_32_32(base_lg2_m, exp_m);
    ans_lg2_e = add(exp_e, base_lg2_e);

    /* Calc antilog */
    result_m = BASOP_util_Pow2(ans_lg2_m, ans_lg2_e, result_e);

    return result_m;
}


Word32 dotWord32_16(const Word32 * X, const Word16 * Y, Word16 n)
{
    Word32 acc;
    Word16 i;



    acc=L_deposit_l(0);

    FOR (i=0; i<n; i++)
    {
        /* ERROR! */
        acc = L_mac(acc, extract_h(X[i]), Y[i]);
    }


    return acc;
}


/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Dot_product12_offs()                                    |
 |                                                                           |
 |       Compute scalar product of <x[],y[]> using accumulator.              |
 |       The parameter 'L_off' is added to the accumulation result.          |
 |       The result is normalized (in Q31) with exponent (0..30).            |
 |   Notes:                                                                  |
 |       o  data in x[],y[] must provide enough headroom for accumulation    |
 |       o  L_off must correspond in format with product of x,y              |
 |          Example: 0.01f for Q9 x Q9: 0x0000147B in Q19                    |
 |                   means: L_off = FL2WORD32_SCALE(0.01,31-19)             |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |       dot_product = L_off + sum(x[i]*y[i])     i=0..N-1                   |
 |___________________________________________________________________________|
*/
Word32 Dot_product12_offs(                 /* (o) Q31: normalized result (1 < val <= -1) */
    const Word16 x[],                     /* (i) 12bits: x vector                       */
    const Word16 y[],                     /* (i) 12bits: y vector                       */
    const Word16 lg,                      /* (i)    : vector length in range [1..256]   */
    Word16 * exp,                         /* (o)    : exponent of result (0..+30)       */
    Word32 L_off                            /* (i) initial summation offset / 2           */
)
{
    Word16 i, sft;
    Word32 L_sum;


    L_sum = L_mac0(L_off, x[0], y[0]);
    FOR (i = 1; i < lg; i++)
    {
        L_sum = L_mac0(L_sum, x[i], y[i]);
    }
    /* Normalize acc in Q31 */

    sft = norm_l(L_sum);
    if (exp != NULL)
    {
        L_sum = L_shl(L_sum, sft);
    }

    /* exponent = 0..30, when L_sum != 0 */
    if (L_sum != 0)
    {
        sft = sub(31,sft);
    }

    if (exp != NULL)
    {
        *exp = sft;
        move16();
    }

    return L_sum;
}

Word32 Dot_product15_offs(                 /* (o) Q31: normalized result (1 < val <= -1) */
    const Word16 x[],                     /* (i) 15bits: x vector                       */
    const Word16 y[],                     /* (i) 15bits: y vector                       */
    const Word16 lg,                      /* (i)    : vector length in range [1..256]   */
    Word16 *exp,                          /* (o)    : exponent of result (0..+30)       */
    Word32 L_off                          /* (i) initial summation offset               */
)
{
    Word16 i, sft, fac, ld;
    Word32 L_sum;

    ld = sub(14,norm_s(lg));
    fac = shr(-32768,ld);
    L_sum = L_shr(L_off,ld);

    FOR (i = 0; i < lg; i++)
    {
        L_sum = L_add(L_sum, Mpy_32_16_1(L_msu(0, y[i],fac),x[i]));
    }

    /* Avoid returning 0 */
    if (L_sum == 0)
    {
        L_sum = L_add(L_sum, 1);
    }

    /* Normalize acc in Q31 */
    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);

    /* exponent = 0..30, when L_sum != 0 */
    if (L_sum != 0)
    {
        sft = add(ld, sub(30,sft));
    }

    *exp = sft;
    move16();

    return L_sum;
}


/*

     precondition:  headroom in Y is sufficient for n accumulations
*/
Word32 BASOP_Util_Add_Mant32Exp                  /*!< o: normalized result mantissa */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e,       /*!< i: Exponent of 2nd operand b  */
 Word16  *ptr_e)     /*!< o: exponent of result         */
{
    Word32 L_tmp;
    Word16 shift;



    /* Compare exponents: the difference is limited to +/- 30
       The Word32 mantissa of the operand with lower exponent is shifted right by the exponent difference.
       Then, the unshifted mantissa of the operand with the higher exponent is added. The addition result
       is normalized and the result represents the mantissa to return. The returned exponent takes into
       account all shift operations.
    */

    if (!a_m)
        a_e = add(b_e,0);

    if (!b_m)
        b_e = add(a_e,0);

    shift = sub(a_e, b_e);
    shift = s_max(-31,shift);
    shift = s_min(31, shift);
    if (shift < 0)
    {
        /* exponent of b is greater than exponent of a, shr a_m */
        a_m = L_shl(a_m,shift);
    }
    if (shift > 0)
    {
        /* exponent of a is greater than exponent of b */
        b_m = L_shr(b_m,shift);
    }
    a_e = add(s_max(a_e,b_e),1);
    L_tmp = L_add(L_shr(a_m,1),L_shr(b_m,1));
    shift = norm_l(L_tmp);
    if (shift)
        L_tmp = L_shl(L_tmp,shift);
    if (L_tmp == 0)
        a_e = add(0,0);
    if (L_tmp != 0)
        a_e = sub(a_e,shift);
    *ptr_e = a_e;

    return (L_tmp);
}


Word16 BASOP_Util_Cmp_Mant32Exp                  /*!< o: flag: result of comparison */
/*      0, if a == b               */
/*      1, if a > b                */
/*     -1, if a < b                */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e)       /*!< i: Exponent of 2nd operand b  */

{
    Word32 diff_m;
    Word16 diff_e, shift, result;



    /*
       This function compares two input parameters, both represented by a 32-bit mantissa and a 16-bit exponent.
       If both values are identical, 0 is returned.
       If a is greater b, 1 is returned.
       If a is less than b, -1 is returned.
    */

    /* Check, if both mantissa and exponents are identical, when normalized: return 0 */
    shift = norm_l(a_m);
    if (shift)
        a_m = L_shl(a_m, shift);
    if (shift)
        a_e = sub(a_e, shift);

    shift = norm_l(b_m);
    if (shift)
        b_m = L_shl(b_m, shift);
    if (shift)
        b_e = sub(b_e, shift);

    /* align exponent, if any mantissa is zero */
    if (!a_m)
        a_e = add(b_e,0);
    if (!b_m)
        b_e = add(a_e,0);

    BASOP_SATURATE_WARNING_OFF
    diff_m = L_sub(a_m,b_m);
    BASOP_SATURATE_WARNING_ON
    diff_e = sub(a_e,b_e);

    test();
    IF(diff_m == 0 && diff_e == 0)
    {
        return 0;
    }

    /* Check sign, exponent and mantissa to identify, whether a is greater b or not */
    result = sub(0,1);

    IF (a_m  >= 0)
    {
        /* a is positive */
        if (b_m < 0)
        {
            result = add(1,0);
        }

        test();
        test();
        test();
        if ((b_m >= 0) && ((diff_e > 0) || (diff_e == 0 && diff_m > 0)))
        {
            result = add(1,0);
        }
    }
    ELSE
    {
        /* a is negative */
        test();
        test();
        test();
        if ((b_m < 0) && ((diff_e < 0) || (diff_e == 0 && diff_m > 0)))
        {
            result = add(1,0);
        }
    }
    return result;
}

/*

     headroom is introduced into acc
*/



void bufferCopyFx(
    Word16* src,       /*<! Qx  pointer to input buffer                */
    Word16* dest,      /*<! Qx  pointer to output buffer               */
    Word16 length,     /*<! Q0  length of buffer to copy               */
    Word16 Qf_src,     /*<! Q0  Q format (frac-bits) of source buffer  */
    Word16 Qf_dest,    /*<! Q0  Q format (frac-bits )of dest buffer    */
    Word16 Q_src,      /*<! Q0  exponent of source buffer              */
    Word16 Q_dest      /*<! Q0  exponent of destination buffer         */
)
{
    Word16 tmp_16,i;

    /*Copy( st->old_exc, exc_buf, st->old_exc_len);*/
    tmp_16 = sub(sub(Qf_src,Qf_dest),sub(Q_src,Q_dest));
    IF (tmp_16>0) /*if value will be shifted right, do a multiplication with rounding ->preserves more accuracy*/
    {
        tmp_16 = shl(1,sub(15,tmp_16));
        FOR (i = 0 ; i < length ; i++)
        {
            *(dest+i) = mult_r(*(src+i),tmp_16);
            move16();
        }
    }
    ELSE IF (tmp_16 <0)/*leftshift - no accuracy preservation needed*/
    {
        FOR (i = 0 ; i < length ; i++)
        {
            *(dest+i) = shr(*(src+i),tmp_16);
            move16();
        }
    }
    ELSE /*no shift, simply copy*/
    {
        FOR (i = 0 ; i < length ; i++)
        {
            *(dest+i) = *(src+i);
            move16();
        }
    }
}

Word32 dotWord32_16_Mant32Exp(const Word32 *bufX32,/* i: 32-bit buffer with unknown headroom */
                              Word16 bufX32_exp,   /* i: exponent of buffer bufX32           */
                              const Word16 *bufY16,/* i: 16-bit buffer quite right-aligned   */
                              Word16 bufY16_exp,   /* i: exponent of buffer bufY16           */
                              Word16 len,          /* i: buffer len to process               */
                              Word16 *exp)         /* o: result exponent                     */
{
    Word32 L_sum;
    Word16 shift, shift1, i;



    shift = getScaleFactor32(bufX32, len);         /* current available headroom */
    shift = sub(shift, sub(14,norm_s(len)));        /* reduced required headroom  */
    L_sum = L_add(0,0);                            /* Clear accu                 */
    FOR(i=0; i < len; i++)
    {
        L_sum = L_mac0(L_sum, round_fx(L_shl(bufX32[i], shift)), bufY16[i]);
    }
    shift1 = norm_l(L_sum);
    L_sum = L_shl(L_sum, shift1);                  /* return value */

    shift = sub(add(bufX32_exp, bufY16_exp), add(shift, shift1));
    shift = add(shift, 1); /* compensate for factor of 2 introduced by L_mac0 */
    /* In case of NULL result, we want to have a 0 exponent too */
    if (L_sum == 0)
        shift = 0;
    *exp = shift;
    move16();


    return L_sum;

}

Word16 BASOP_Util_lin2dB(Word32 x, Word16 x_e, Word16 fEnergy)
{
    assert(x >= 0);

    /* log2 */
    x = L_shr(BASOP_Util_Log2(x), 25-16); /* Q16 */

    /* add exponent */
    x = L_msu(x, x_e, -32768 /* 0x8000 */);

    /* convert log2 to 20*log10 */
    x = Mpy_32_16_1(x, FL2WORD16_SCALE(6.0206f, 3)); /* Q13 */

    /* if energy divide by 2 (->10*log10) */
    if (fEnergy != 0) x = L_shr(x, 1);

    /* return dB as 7Q8 */
    return round_fx(L_shl(x, 8-13+16)); /* Q8 */
}

/* --- fixp_atan() ----    */
#define Q_ATANINP   (25)    /* Input in q25, Output in q14 */
#define Q_ATANOUT   (14)
#define ATI_SF              ((32-1)-Q_ATANINP)  /* 6 */
#define ATO_SF              ((16-1)-Q_ATANOUT)  /* 1   ] -pi/2 .. pi/2 [ */
/* --- fixp_atan2() ---    */
#define Q_ATAN2OUT  (13)
#define AT2O_SF             ((16-1)-Q_ATAN2OUT) /* 2   ] -pi   .. pi   ] */


Word16 BASOP_util_atan2(              /* o: atan2(y,x)    [-pi,pi]        Q13   */
    Word32 y,     /* i:                                     */
    Word32 x,     /* i:                                     */
    Word16 e      /* i: exponent difference (exp_y - exp_x) */
)
{
    Word16 q;
    Word32 at;
    Word16 ret = FL2WORD16(-1.0f);
    Word16 sf,sfo,stf;
    Word32 L_sign;

    if(L_or(y,x) == 0)
    {
        return 0;
    }

    IF(x == 0)
    {
        ret = FL2WORD16_SCALE(+EVS_PI/2,AT2O_SF);
        move16();
        if ( y <  0 )
        {
            ret = negate(ret);
        }

        return ret;
    }

    /* --- division */
    L_sign = L_and(L_xor(x,y), 0x80000000 );

    q = FL2WORD16(1.0f);  /* y/x = neg/zero = -Inf */
    sf = 0;
    BASOP_SATURATE_WARNING_OFF
    q =  BASOP_Util_Divide3232_uu_1616_Scale(L_abs(y),L_abs(x), &sf);
    BASOP_SATURATE_WARNING_ON

    BASOP_SATURATE_WARNING_OFF
    if(L_sign < 0)
        q = negate(q);
    BASOP_SATURATE_WARNING_ON

    sfo = add(sf,e);

    /* --- atan() */
    IF  ( sub(sfo,ATI_SF) > 0 )
    {
        /* --- could not calc fixp_atan() here bec of input data out of range
             ==> therefore give back boundary values */

        sfo = s_min(sfo, MAXSFTAB);

        /*q = FL2WORD16( 0.0f );                              move16();*/

        if(  q > 0 )
        {
            move16();
            q = +f_atan_expand_range[sfo-ATI_SF-1];
        }
        if(  q < 0 )
        {
            move16();
            q = -f_atan_expand_range[sfo-ATI_SF-1];
        }
    }
    ELSE
    {
        /* --- calc of fixp_atan() is possible; input data within range
             ==> set q on fixed scale level as desired from fixp_atan() */
        stf = sub(sfo, ATI_SF);

        at = L_deposit_h(q);
        if (stf < 0)  at = L_shl(at,stf);

        q = BASOP_util_atan(at);  /* ATO_SF*/
    }


    /* --- atan2() */

    ret = shr(q,(AT2O_SF - ATO_SF)); /* now AT2O_SF for atan2 */
    IF (  x < 0 )
    {
        if (  y >= 0 )
        {
            ret = add(ret, FL2WORD16_SCALE(EVS_PI, AT2O_SF));
        }
        if(y < 0)
        {
            ret = sub(ret, FL2WORD16_SCALE( EVS_PI,AT2O_SF));
        }
    }

    return ret;
}

/* SNR of fixp_atan() = 56 dB*/
#define ONEBY3P56  0x26800000 /* 1.0/3.56 in q31*/
#define P281       0x00026000 /* 0.281 in q19*/
#define ONEP571    0x6487 /* 1.571 in q14*/

Word16 BASOP_util_atan(                 /* o:  atan(x)           [-pi/2;pi/2]   1Q14  */
    Word32 x         /* i:  input data        (-64;64)       6Q25  */
)
{
    Word16 sign, result, exp;
    Word16 res_e;
    Word16 tmp, xx;



    sign = 0;
    move16();
    if (x < 0)
    {
        sign = 1;
        move16();
    }
    x = L_abs(x);

    /* calc of arctan */
    IF(L_sub(x, FL2WORD32(0.045f/64.0f)) < 0 )
    {
        result = round_fx(L_shl(x,5)); /*Q14*/
        /*BASOP_util_atan_16(0.0444059968): max error 0.0000567511, mean 0.000017, abs mean 0.000017*/
    }
    ELSE
    IF(L_sub(x,( L_shl(1,Q_ATANINP)-FL2WORD32(0.00395))) < 0 )
    {
        xx =round_fx(L_shl(x,6));
        tmp = mult_r(xx, xx);            /* q15 * q15 - (16-1) = q15*/
        tmp = mult_r(tmp, 0x1340);      /* 15 * (ONEBY3P56) q14 - (16-1) = q14*/
        tmp = add(tmp, 0x4000); /*L_shl(1,14) = 524288*/                /* q14 + q14 = q14 */
        res_e=Q_ATANOUT-15+14-16+1;
        move16();
        if(sub(xx,tmp) > 0)
        {
            res_e = add(res_e,1);
        }
        if(sub(xx,tmp) > 0)
        {
            xx = shr(xx,1);
        }
        result = div_s(xx, tmp);
        result = msu_r(0, result, shl(-32768,res_e));
        /*BASOP_util_atan_16(0.7471138239): max error 0.0020029545, mean 0.000715, abs mean 0.000715*/
    }
    ELSE IF( L_sub(x,FL2WORD32(1.28/64.0)) < 0 )
    {
        Word16 delta_fix;
        Word32 PI_BY_4 = FL2WORD32(3.1415926/4.0)/2; /* pi/4 in q30 */

        delta_fix = round_fx(L_shl(L_sub(x,FL2WORD32(1.0/64.0)), 5)); /* q30 */
        result = round_fx(L_sub(L_add(PI_BY_4, L_msu(0,delta_fix,-16384)),(L_mult0(delta_fix, delta_fix))));
        /* BASOP_Util_fPow(0.7472000122): max error 0.0020237688, mean 0.000026, abs mean 0.000520 */
    }
    ELSE
    {
        exp = sub(norm_l(x),1);
        xx = round_fx(L_shl(x,exp));
        /* q25+exp * q25+exp - (16-1) = q19+2*exp*/
        tmp = mac_r(L_shl(P281,shl(exp,1)),xx, xx);                 /* q19+2*exp + q19+2*exp = q19+2*exp*/
        res_e = norm_s(tmp);
        result = div_s(xx, shl(tmp,res_e));
        result = shl(result, add(add(Q_ATANOUT-Q_ATANINP/*-exp*/+19/*+2*exp*/-16+1, res_e ),exp));
        result = sub(ONEP571,result);          /* q14 + q14 = q14*/
        /*BASOP_Util_fPow(1.2799999714): max error 0.0020168927, mean 0.000066, abs mean 0.000072*/
    }

    if (sign)
    {
        result = negate(result);
    }

    return(result);
}

/* compare two positive normalized 16 bit mantissa/exponent values */
/* return value: positive if first value greater, negative if second value greater, zero if equal */
Word16 compMantExp16Unorm(Word16 m1, Word16 e1, Word16 m2, Word16 e2)
{
    Word16 tmp;

    assert((m1 >= 0x4000) && (m2 >= 0x4000)); /* comparisons below work only for normalized mantissas */

    tmp = sub(e1, e2);
    if (tmp == 0) tmp = sub(m1, m2);

    return tmp;
}

