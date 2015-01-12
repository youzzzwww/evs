/*--------------------------------------------------------------------------*
 *                         MATH_OP.H	                                    *
 *--------------------------------------------------------------------------*
 *       Mathematical operations                                            *
 *--------------------------------------------------------------------------*/
#include "oper_32b.h"
#include "log2.h"

Word32 Isqrt(           /* (o) Q31 : output value (range: 0<=val<1)         */
     Word32 L_x         /* (i) Q0  : input value  (range: 0<=val<=7fffffff) */
);
Word32 Isqrt_lc(
     Word32 frac,       /* (i/o) Q31: normalized value (1.0 < frac <= 0.5) */
     Word16 * exp       /* (i/o)    : exponent (value = frac x 2^exponent) */
);

Word32 Pow2(            /* (o) Q0  : result       (range: 0<=val<=0x7fffffff) */
     Word16 exponant,   /* (i) Q0  : Integer part.      (range: 0<=val<=30)   */
     Word16 fraction    /* (i) Q15 : Fractionnal part.  (range: 0.0<=val<1.0) */
);
Word32 Dot_product12(   /* (o) Q31: normalized result (1 < val <= -1) */
     const Word16 x[],  /* (i) 12bits: x vector                       */
     const Word16 y[],  /* (i) 12bits: y vector                       */
     const Word16 lg,   /* (i)    : vector length                     */
     Word16 * exp       /* (o)    : exponent of result (0..+30)       */
);

Word32 Energy_scale(    /* (o) Q31: normalized result (1 < val <= -1) */
     const Word16 x[],  /* (i) 12bits: x vector                       */
     const Word16 lg,   /* (i)  : vector length                       */
     Word16 expi,       /* (i)  : exponent of input                   */
     Word16 *exp        /* (o)  : exponent of result (0..+30)         */
);

Word32 Sqrt_l(Word32 L_x, Word16 *exp);

Word32 L_Frac_sqrtQ31(  /* o  : Square root if input */
    const Word32 x      /* i  : Input                */
);

Word16 Frac_sqrt(       /* o  : Square root if input */
    const Word16 x      /* i  : Input                */
);

Word16 i_mult2 (Word16 a, Word16 b);

#include "math_32.h"

