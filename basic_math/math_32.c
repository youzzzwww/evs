/*#include "options.h" */
#include "stl.h"
#ifdef ALLOW_40bits
#include "enh40.h"
#endif
#include "oper_32b.h"

/* 32x16 multiply: */
Word32 Mult_32_16(Word32 a, Word16 b)
{
	Word32 result; 
#ifdef ALLOW_40bits /* if activated; need 40 bits basic-op files */
    UWord16 lo;
    /* use Mpy_32_16_ss(): */
    Mpy_32_16_ss(a, b, &result, &lo);  
#else 
    Word16 lo, hi;
  /* do things by hand: */
    lo = L_Extract_lc(a, &hi);
    result = Mpy_32_16(hi, lo, b);
#endif
    return result;
}
/* 32x32 multiply: */
Word32 Mult_32_32(Word32 a, Word32 b)
{
	Word32 result; 
#ifdef ALLOW_40bits /* if activated; need 40 bits basic-op files */
    UWord32 lo;
    /* use Mpy_32_32_ss(): */
    Mpy_32_32_ss(a, b, &result, &lo);
#else 
    Word16 hi, lo, b_hi, b_lo;
  /* do things by hand: */
    lo = L_Extract_lc(a, &hi);
    b_lo = L_Extract_lc(b, &b_hi);
    result = Mpy_32(hi, lo, b_hi, b_lo);
#endif
	return result;
}

/* 32x16 multiply-accumulate: */
Word32 Madd_32_16(Word32 L_num, Word32 a, Word16 b)
{
    Word32 result; 
#ifdef ALLOW_40bits /* if activated; need 40 bits basic-op files */
    UWord16 lo;
    /* use Mpy_32_16_ss(): */
    Mpy_32_16_ss(a, b, &result, &lo);
    result = L_add(L_num, result);
#else 
    Word16 lo, hi;
  /* do things by hand: */
    lo = L_Extract_lc(a, &hi);
    result = Mac_32_16(L_num, hi, lo, b);
#endif
    return result;
}

/* 32x16 multiply-substract: */
Word32 Msub_32_16(Word32 L_num, Word32 a, Word16 b)
{
	Word32 result; 
#ifdef ALLOW_40bits /* if activated; need 40 bits basic-op files */
    UWord16 lo;
    /* use Mpy_32_16_ss(): */
    Mpy_32_16_ss(a, b, &result, &lo);
    result = L_sub(L_num, result);
#else 
    Word16 lo, hi;
  /* do things by hand: */
    lo = L_Extract_lc(a, &hi);
    result = Msu_32_16(L_num, hi, lo, b);
#endif
    return result;
}

