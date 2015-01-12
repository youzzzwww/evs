/*___________________________________________________________________________
 |                                                                           |
 |  This file contains mathematic operations in fixed point.                 |
 |                                                                           |
 |  Isqrt()              : inverse square root (16 bits precision).          |
 |  Pow2()               : 2^x  (16 bits precision).                         |
 |  Log2()               : log2 (16 bits precision).                         |
 |  Dot_product()        : scalar product of <x[],y[]>                       |
 |                                                                           |
 |  In this file, the values use theses representations:                     |
 |                                                                           |
 |  Word32 L_32     : standard signed 32 bits format                         |
 |  Word16 hi, lo   : L_32 = hi<<16 + lo<<1  (DPF - Double Precision Format) |
 |  Word32 frac, Word16 exp : L_32 = frac << exp-31  (normalised format)     |
 |  Word16 int, frac        : L_32 = int.frac        (fractional format)     |
 |___________________________________________________________________________|
*/

#include "stl.h"
#include "math_op.h"
#include "rom_basic_math.h"

#include <stdlib.h>
#include <stdio.h>

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Isqrt                                                   |
 |                                                                           |
 |       Compute 1/sqrt(L_x).                                                |
 |       if L_x is negative or zero, result is 1 (7fffffff).                 |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   1- Normalization of L_x.                                                |
 |   2- call Isqrt_lc(L_x, exponant)                                         |
 |   3- L_y = L_x << exponant                                                |
 |___________________________________________________________________________|
*/
Word32 Isqrt(                              /* (o) Q31 : output value (range: 0<=val<1)         */
     Word32 L_x                            /* (i) Q0  : input value  (range: 0<=val<=7fffffff) */
)
{
    Word16 exp;
    Word32 L_y;

    exp = norm_l(L_x);
    L_x = L_shl(L_x, exp);                 /* L_x is normalized */
    exp = sub(31, exp);

    L_x = Isqrt_lc(L_x, &exp);

    L_y = L_shl(L_x, exp);                 /* denormalization   */

    return (L_y);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Isqrt_lc                                                |
 |                                                                           |
 |       Compute 1/sqrt(value).                                              |
 |       if value is negative or zero, result is 1 (frac=7fffffff, exp=0).   |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   The function 1/sqrt(value) is approximated by a table and linear        |
 |   interpolation.                                                          |
 |                                                                           |
 |   1- If exponant is odd then shift fraction right once.                   |
 |   2- exponant = -((exponant-1)>>1)                                        |
 |   3- i = bit25-b30 of fraction, 16 <= i <= 63 ->because of normalization. |
 |   4- a = bit10-b24                                                        |
 |   5- i -=16                                                               |
 |   6- fraction = table[i]<<16 - (table[i] - table[i+1]) * a * 2            |
 |___________________________________________________________________________|
*/
Word32 Isqrt_lc(
     Word32 frac,  /* (i)   Q31: normalized value (1.0 < frac <= 0.5) */
     Word16 * exp  /* (i/o)    : exponent (value = frac x 2^exponent) */
)
{
    Word16 i, a;
    Word32 L_tmp;

    IF (frac <= (Word32) 0)
    {
        *exp = 0;                          move16();
        return 0x7fffffff; /*0x7fffffff*/
    }

    /* If exponant odd -> shift right by 10 (otherwise 9) */
    L_tmp = L_shr(frac, shift_Isqrt_lc[s_and(*exp, 1)]);

    /* 1) -16384 to shift left and change sign                 */
    /* 2) 32768 to Add 1 to Exponent like it was divided by 2  */
    /* 3) We let the mac_r add another 0.5 because it imitates */
    /*    the behavior of shr on negative number that should   */
    /*    not be rounded towards negative infinity.            */
    /* It replaces:                                            */
    /*    *exp = negate(shr(sub(*exp, 1), 1));   move16();     */
    *exp = mac_r(32768, *exp, -16384);     move16();

    a = extract_l(L_tmp);                           /* Extract b10-b24 */
    a = lshr(a, 1);

    i = mac_r(L_tmp, -16*2-1, 16384);               /* Extract b25-b31 minus 16 */
    
    L_tmp = L_msu(L_table_isqrt[i], table_isqrt_diff[i], a);/* table[i] << 16 - diff*a*2 */

    return L_tmp;
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Pow2()                                                  |
 |                                                                           |
 |     L_x = pow(2.0, exponant.fraction)         (exponant = interger part)  |
 |         = pow(2.0, 0.fraction) << exponant                                |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   The function Pow2(L_x) is approximated by a table and linear            |
 |   interpolation.                                                          |
 |                                                                           |
 |   1- i = bit10-b15 of fraction,   0 <= i <= 31                            |
 |   2- a = bit0-b9   of fraction                                            |
 |   3- L_x = table[i]<<16 - (table[i] - table[i+1]) * a * 2                 |
 |   4- L_x = L_x >> (30-exponant)     (with rounding)                       |
 |___________________________________________________________________________|
*/

Word32 Pow2(                              /* (o) Q0  : result       (range: 0<=val<=0x7fffffff) */
    Word16 exponant,                      /* (i) Q0  : Integer part.      (range: 0<=val<=30)   */
    Word16 fraction                       /* (i) Q15 : Fractionnal part.  (range: 0.0<=val<1.0) */
)
{
    Word16 exp, i, a;
    Word32 L_x;



    i = mac_r(-32768, fraction, 32);         /* Extract b10-b16 of fraction */
    a = s_and(fraction, 0x3ff);              /* Extract  b0-b9  of fraction */

    L_x = L_deposit_h(table_pow2[i]);           /* table[i] << 16   */
    L_x = L_mac(L_x, table_pow2_diff_x32[i], a);/* L_x -= diff*a*2  */

    exp = sub(30, exponant);

    L_x = L_shr_r(L_x, exp);


    return L_x;
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Dot_product12()                                         |
 |                                                                           |
 |       Compute scalar product of <x[],y[]> using accumulator.              |
 |                                                                           |
 |       The result is normalized (in Q31) with exponent (0..30).            |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |       dot_product = sum(x[i]*y[i])     i=0..N-1                           |
 |___________________________________________________________________________|
*/

Word32 Dot_product12(                      /* (o) Q31: normalized result (1 < val <= -1) */
     const Word16 x[],                     /* (i) 12bits: x vector                       */
     const Word16 y[],                     /* (i) 12bits: y vector                       */
     const Word16 lg,                      /* (i)    : vector length                     */
     Word16 * exp                          /* (o)    : exponent of result (0..+30)       */
)
{
    Word16 i, sft;
    Word32 L_sum;

    L_sum = L_mac(1, x[0], y[0]);
    FOR (i = 1; i < lg; i++)
        L_sum = L_mac(L_sum, x[i], y[i]);

    /* Normalize acc in Q31 */

    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);

    *exp = sub(30, sft);                   move16();  /* exponent = 0..30 */

    return L_sum;
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Energy_scale()                                          |
 |                                                                           |
 |       Compute energy of signal (scaling the input if specified)           |
 |                                                                           |
 |       The result is normalized (in Q31) with exponent (0..30).            |
 |___________________________________________________________________________|
*/

Word32 Energy_scale(                       /* (o) : Q31: normalized result (1 < val <= -1)  */
     const Word16 x[],                     /* (i) : input vector x                          */
     const Word16 lg,                      /* (i) : vector length                           */
     Word16 expi,                          /* (i) : exponent of input                       */
     Word16 *exp                           /* (o) : exponent of result (0..+30)             */
)
{
    Word16 i, sft, tmp;
    Word32 L_sum;


    L_sum = 0;   /* just to avoid superflous compiler warning about uninitialized use of L_sum */

    IF (expi == 0)
    {
        L_sum = L_mac(1, x[0], x[0]);
        FOR (i = 1; i < lg; i++)
        {
            L_sum = L_mac(L_sum, x[i], x[i]);
        }
    }
    IF (expi < 0)
    {
        sft = lshl(-32768 /* 0x8000 */, expi);
        tmp = mult_r(x[0], sft);
        L_sum = L_mac(1, tmp, tmp);
        FOR (i = 1; i < lg; i++)
        {
            tmp = mult_r(x[i], sft);
            L_sum = L_mac(L_sum, tmp, tmp);
        }
    }
    IF (expi > 0)
    {
        tmp = shl(x[0], expi);
        L_sum = L_mac(1, tmp, tmp);
        FOR (i = 1; i < lg; i++)
        {
            tmp = shl(x[i], expi);
            L_sum = L_mac(L_sum, tmp, tmp);
        }
    }

    /* Normalize acc in Q31 */

    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);

    *exp = sub(30, sft);                                                            move16();  /* exponent = 0..30 */


    return L_sum;
}

Word32 Sqrt_l(     /* o : output value,                          Q31 */
    Word32 L_x,    /* i : input value,                           Q31 */
    Word16 *exp    /* o : right shift to be applied to result,   Q1  */
)
{
    /*
        y = sqrt(x)

        x = f * 2^-e,   0.5 <= f < 1   (normalization)

        y = sqrt(f) * 2^(-e/2)  

        a) e = 2k   --> y = sqrt(f)   * 2^-k  (k = e div 2,
                                               0.707 <= sqrt(f) < 1)
        b) e = 2k+1 --> y = sqrt(f/2) * 2^-k  (k = e div 2,
                                               0.5 <= sqrt(f/2) < 0.707)
     */
    
    Word16 e, i, a, tmp;
    Word32 L_y;

    if (L_x <= 0)
    {
        *exp = 0;        move16 ();
        return L_deposit_l(0);
    }

    e = s_and(norm_l(L_x), 0x7FFE);  /* get next lower EVEN norm. exp  */
    L_x = L_shl(L_x, e);             /* L_x is normalized to [0.25..1) */
    *exp = e;            move16 ();  /* return 2*exponent (or Q1)      */

    L_x = L_shr(L_x, 9);
    a = extract_l(L_x);              /* Extract b10-b24                */
    a = lshr(a, 1);     

    i = mac_r(L_x, -16*2-1, 16384);                  /* Extract b25-b31 minus 16 */

    L_y = L_deposit_h(sqrt_table[i]);                /* table[i] << 16                 */
    tmp = sub(sqrt_table[i], sqrt_table[i + 1]);     /* table[i] - table[i+1])         */
    L_y = L_msu(L_y, tmp, a);                        /* L_y -= tmp*a*2                 */
       
    /* L_y = L_shr (L_y, *exp); */                   /* denormalization done by caller */

    return (L_y);
}

/*---------------------------------------------------------------------------*
 * L_Frac_sqrtQ31
 *
 * Calculate square root from fractional values (Q31 -> Q31)
 * Uses 32 bit internal representation for precision
 *---------------------------------------------------------------------------*/
Word32 L_Frac_sqrtQ31(    /* o  : Square root if input */
    const Word32 x        /* i  : Input                */
)
{
    Word32 log2_work;
    Word16 log2_int, log2_frac;

    test();
    if (x > 0)
    {
        log2_int = norm_l(x);
        log2_frac = Log2_norm_lc(L_shl(x, log2_int));

        log2_work = L_msu((31+30)*65536L/2, 16384, log2_int);
        log2_work = L_mac0(log2_work, log2_frac, 1);

        log2_frac = L_Extract_lc(log2_work, &log2_int);

        return Pow2(log2_int, log2_frac);
    }
    return 0;
}

/*----------------------------------------------------------------------------------*
 * Frac_sqrt
 *
 * Calculate square root from fractional values (Q15 -> Q15)
 *----------------------------------------------------------------------------------*/
Word16 Frac_sqrt(         /* o  : Square root if input */
    const Word16 x        /* i  : Input                */
)
{
    return round_fx(L_Frac_sqrtQ31(L_deposit_h(x)));
}


/*----------------------------------------------------------------------------------*
 * i_mult2
 *
 * Faster Integer Multiplication
 *----------------------------------------------------------------------------------*/

Word16 i_mult2 (Word16 a, Word16 b)
{
    return extract_l(L_mult0(a, b));
}



