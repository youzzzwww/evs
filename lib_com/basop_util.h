/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef __BASOP_UTIL_H__
#define __BASOP_UTIL_H__

#include "stl.h"
#include "typedef.h"
#include "basop32.h"
#include "basop_mpy.h"


#define _LONG                long
#define _SHORT               short
#ifdef _WIN32
#define _INT64               __int64
#else
#define _INT64               long long
#endif

#define WORD32_BITS         32
#define MAXVAL_WORD32       ((signed)0x7FFFFFFF)
#define MINVAL_WORD32       ((signed)0x80000000)
#define WORD32_FIX_SCALE    ((_INT64)(1)<<(WORD32_BITS-1))

#define WORD16_BITS         16
#define MAXVAL_WORD16       (((signed)0x7FFFFFFF)>>16)
#define MINVAL_WORD16       (((signed)0x80000000)>>16)
#define WORD16_FIX_SCALE    ((_INT64)(1)<<(WORD16_BITS-1))

/*!
  \def  Macro converts a float < 1 to Word32 fixed point with saturation and rounding
*/
#define FL2WORD32(val)                                                                                                     \
(Word32)( ( (val) >= 0) ?                                                                                                               \
((( (double)(val) * (WORD32_FIX_SCALE) + 0.5 ) >= (double)(MAXVAL_WORD32) ) ? (_LONG)(MAXVAL_WORD32) : (_LONG)( (double)(val) * (double)(WORD32_FIX_SCALE) + 0.5)) : \
((( (double)(val) * (WORD32_FIX_SCALE) - 0.5) <=  (double)(MINVAL_WORD32) ) ? (_LONG)(MINVAL_WORD32) : (_LONG)( (double)(val) * (double)(WORD32_FIX_SCALE) - 0.5)) )

/*!
  \def   Macro converts a float < 1 to Word16 fixed point with saturation and rounding
*/
#define FL2WORD16(val)                                                                                                     \
(Word16)( ( (val) >= 0) ?                                                                                                               \
((( (double)(val) * (WORD16_FIX_SCALE) + 0.5 ) >= (double)(MAXVAL_WORD16) ) ? (_LONG)(MAXVAL_WORD16) : (_LONG)( (double)(val) * (double)(WORD16_FIX_SCALE) + 0.5)) : \
((( (double)(val) * (WORD16_FIX_SCALE) - 0.5) <=  (double)(MINVAL_WORD16) ) ? (_LONG)(MINVAL_WORD16) : (_LONG)( (double)(val) * (double)(WORD16_FIX_SCALE) - 0.5)) )

/*!
  \def   Macro converts a Word32 fixed point to Word16 fixed point <1 with saturation
*/
#define WORD322WORD16(val)                                                                           \
 ( ( ((((val) >> (WORD32_BITS-WORD16_BITS-1)) + 1) > (((_LONG)1<<WORD16_BITS)-1)) && ((_LONG)(val) > 0) ) ? \
 (Word16)(_SHORT)(((_LONG)1<<(WORD16_BITS-1))-1):(Word16)(_SHORT)((((val) >> (WORD32_BITS-WORD16_BITS-1)) + 1) >> 1) )

/*!
  \def   Macro converts a Word32 fixed point < 1 to float shifts result left by scale
*/
#define WORD322FL_SCALE(x,scale) ( ((float)((_LONG)(x))) / (((_INT64)1<<(WORD32_BITS-1 - (scale)))) )

/*!
  \def   Macro converts a float < 1 to Word32 fixed point with saturation and rounding, shifts result right by scale
*/
/* Note: Both x and scale must be constants at compile time, scale must be in range -31..31 */
#define FL2WORD32_SCALE(x,scale) FL2WORD32((double)(x) *(((_INT64)1<<(WORD32_BITS-1 - (scale)))) / ((_INT64)1<<(WORD32_BITS-1)))

/*!
  \def   Macro converts a Word16 fixed point < 1 to float shifts result left by scale
*/
#define WORD162FL_SCALE(x,scale) ( ((float)((_LONG)(x))) / (((_INT64)1<<(WORD16_BITS-1 - (scale)))) )

/*!
  \def   Macro converts a float < 1 to Word16 fixed point with saturation and rounding, shifts result right by scale
*/
/* Note: At compile time, x must be a float constant and scale must be an integer constant in range -15..15 */
#define FL2WORD16_SCALE(x,scale) FL2WORD16((float)(x) *(((_INT64)1<<(WORD16_BITS-1 - (scale)))) / ((_INT64)1<<(WORD16_BITS-1)))


/* Word16 Packed Type */
typedef struct
{
    struct
    {
        Word16 re;
        Word16 im;
    } v;
} PWord16;

#define cast16 move16


#define LD_DATA_SCALE     (6)
#define LD_DATA_SHIFT_I5  (7)

#define modDiv2(x)        sub(x,shl(shr(x,1),1))
#define modDiv8(x)        L_sub(x,L_shl(L_shr(x,3),3))

static __inline Word16 limitScale16( Word16 s)
{
    /* It is assumed, that s is calculated just before, therefore we can switch upon sign */
    if (s >= 0)
        s = s_min(s,WORD16_BITS-1);
    if (s < 0)
        s = s_max(s,1-WORD16_BITS);
    return (s);
}

static __inline Word16 limitScale32( Word16 s)
{
    /* It is assumed, that s is calculated just before, therefore we can switch upon sign */
    if (s >= 0)
        s = s_min(s, WORD32_BITS-1);
    if (s < 0)
        s = s_max(s, 1-WORD32_BITS);
    return (s);
}

/*!**********************************************************************
   \brief   Add two values given by mantissa and exponent.

   Mantissas are in 16-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

************************************************************************/
Word16 BASOP_Util_Add_MantExp                    /*!< Exponent of result        */
(Word16   a_m,       /*!< Mantissa of 1st operand a */
 Word16   a_e,       /*!< Exponent of 1st operand a */
 Word16   b_m,       /*!< Mantissa of 2nd operand b */
 Word16   b_e,       /*!< Exponent of 2nd operand b */
 Word16  *ptrSum_m); /*!< Mantissa of result */

/************************************************************************/
/*!
   \brief   Divide two values given by mantissa and exponent.

   Mantissas are in 16-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

   For performance reasons, the division is based on a table lookup
   which limits accuracy.
*/
void BASOP_Util_Divide_MantExp (Word16   a_m,          /*!< Mantissa of dividend a */
                                Word16   a_e,          /*!< Exponent of dividend a */
                                Word16   b_m,          /*!< Mantissa of divisor b */
                                Word16   b_e,          /*!< Exponent of divisor b */
                                Word16  *ptrResult_m,  /*!< Mantissa of quotient a/b */
                                Word16  *ptrResult_e   /*!< Exponent of quotient a/b */
                               );

/* deprecated, use Sqrt32! */
Word32 CodecB_Sqrt_l(     /* o : output value,                          Q31 */
    Word32 L_x,    /* i : input value,                           Q31 */
    Word16 *exp    /* o : right shift to be applied to result,   Q1  */
);


/************************************************************************/
/*!
  \brief   Calculate the squareroot of a number given by mantissa and exponent

  Mantissa is in 16/32-bit-fractional format with values between 0 and 1. <br>
  For *norm versions mantissa has to be between 0.5 and 1. <br>
  The base for the exponent is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>
  The exponent is addressed via pointers and will be overwritten with the result.
*/
Word16 Sqrt16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

Word16 Sqrt16norm(              /*!< output mantissa */
    Word16 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

Word32 Sqrt32(                  /*!< output mantissa */
    Word32 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

Word32 Sqrt32norm(              /*!< output mantissa */
    Word32 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

/* deprecated, use Sqrt16! */
void BASOP_Util_Sqrt_MantExp (Word16  *mantissa,       /*!< Pointer to mantissa */
                              Word16  *exponent        /*!< Pointer to exponent */
                             );

/* deprecated, use Sqrt16norm! */
void BASOP_Util_Sqrt_MantExpNorm (Word16  *mantissa,   /*!< Pointer to normalized mantissa */
                                  Word16  *exponent    /*!< Pointer to exponent */
                                 );

/****************************************************************************/
/*!
  \brief   Calculate the inverse of the squareroot of a number given by mantissa and exponent

  Mantissa is in 16/32-bit-fractional format with values between 0 and 1. <br>
  For *norm versions mantissa has to be between 0.5 and 1. <br>
  The base for the exponent is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>
  The exponent is addressed via pointers and will be overwritten with the result.
*/
Word16 ISqrt16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

Word32 ISqrt32(                  /*!< output mantissa */
    Word32 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

Word32 ISqrt32norm(              /*!< output mantissa */
    Word32 mantissa,  /*!< normalized input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

/* deprecated, use ISqrt16! */
void BASOP_Util_InvSqrt_MantExp (Word16  *mantissa,    /*!< Pointer to mantissa */
                                 Word16  *exponent     /*!< Pointer to exponent */
                                );

/*****************************************************************************/
/*!
  \brief   Calculate the inverse of a number given by mantissa and exponent

  Mantissa is in 16-bit-fractional format with values between 0 and 1. <br>
  The base for the exponent is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>
  The operand is addressed via pointers and will be overwritten with the result.

  The function uses a table lookup and a newton iteration.
*/
Word16 Inv16(                  /*!< output mantissa */
    Word16 mantissa,  /*!< input mantissa */
    Word16 *exponent  /*!< pointer to exponent */
);

/* deprecated, use Inv16! */
void BASOP_Util_Inv_MantExp (Word16 *mantissa,
                             Word16 *exponent
                            );

/******************************************************************************/
/*!
  \brief   Calculate the squareroot and inverse of squareroot of a number given by mantissa and exponent

  Mantissa is in 16-bit-fractional format with values between 0 and 1. <br>
  The base for the exponent is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>
*/
void BASOP_Util_Sqrt_InvSqrt_MantExp (Word16 mantissa,      /*!< mantissa */
                                      Word16 exponent,      /*!< expoinent */
                                      Word16 *sqrt_mant,    /*!< Pointer to sqrt mantissa */
                                      Word16 *sqrt_exp,     /*!< Pointer to sqrt exponent */
                                      Word16 *isqrt_mant,   /*!< Pointer to 1/sqrt mantissa */
                                      Word16 *isqrt_exp     /*!< Pointer to 1/sqrt exponent */
                                     );

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
    const Word16 len_x);  /* i: length of the array to scan  */

/********************************************************************/
/*!
  \brief   Calculates the scalefactor needed to normalize input array

    The scalefactor needed to normalize the Word32 input array is returned <br>
    If the input array contains only '0', a scalefactor 0 is returned <br>
    Scaling factor is determined wrt a normalized target x: 1073741824 <= x <= 2147483647 for positive x <br>
    and   -2147483648 <= x <= -1073741824 for negative x
*/

Word16 getScaleFactor32(                 /* o: measured headroom in range [0..31], 0 if all x[i] == 0 */
    const Word32 *x,      /* i: array containing 32-bit data */
    const Word16 len_x);  /* i: length of the array to scan  */

/**
 * \brief normalize mantissa and update the exponent accordingly.
 * \param mantissa the mantissa to be normalized
 * \param pexponent pointer to the exponent.
 * \return the normalized mantissa.
 */
Word16 normalize16(Word16 mantissa, Word16 *pexponent);

/****************************************************************************/
/*!
  \brief   Returns the maximum scalefactor out of an array of Word16 scalefactors

  can be generally used as a replacement for s_max(v1,v2) to find the maximum
  value in an array of Word16 values

  \return maximum value in array of Word16 values
*/
Word16 BASOP_Util_GetMaxScaleFactor16(  Word16 *exp,    /*!< Pointer to scalefactor array       */
                                        Word16 len);    /*!< Length of array to be processed    */

/****************************************************************************/
/*!
  \brief   Does fractional integer division of Word32 arg1 by Word16 arg2

  both input arguments may be positive or negative <br>
  the result is truncated to Word16

  \return fractional integer Word16 result of arg1/arg2
*/
Word16 divide3216(  Word32 x,   /*!< Numerator*/
                    Word16 y);  /*!< Denominator*/


/****************************************************************************/
/*!
  \brief   Does fractional integer division of Word16 arg1 by Word16 arg2

  both input arguments may be positive or negative <br>
  the result is truncated to Word16

  \return fractional integer Word16 result of arg1/arg2
*/
Word16 divide1616(  Word16 x,   /*!< Numerator*/
                    Word16 y);  /*!< Denominator*/


/****************************************************************************/
/*!
  \brief   Does fractional integer division of Word32 arg1 by Word32 arg2

   this function makes both the numerator and the denominator positive integers,
   and scales up both values to avoid losing the accuracy of the outcome
   too much

   WARNING: it should be arg1 < arg2 because of the maximum degree of scaling for the mantissa!

  \return fractional Word16 integer z = arg1(32bits)/arg2(32bits)
*/
Word16 divide3232(  Word32 x,   /*!< Numerator*/
                    Word32 y);  /*!< Denominator*/


/****************************************************************************/
/*!
  \brief   Does fractional integer division of UWord32 arg1 by UWord32 arg2

   This function ensures both the numerator  and the denominator are positive integers,
   and scales up both values to avoid losing the accuracy of the outcome
   too much.<br>

   CAUTION: Arg 3 is a Word16 pointer which will point to the scalefactor difference
   s_diff = sub(s2,s1), where s1 and s2 are the scalefactors of the arguments, which
   were shifted in order to e.g. preserve accuracy.
   I.e. the result has to be scaled due to shifting it
   s_diff to the right to obtain the real result of the division.

  \return fractional Word16 integer z = arg1(32bits)/arg2(32bits)
*/
Word16 BASOP_Util_Divide3232_uu_1616_Scale( Word32 x,   /*!< i  : Numerator*/
        Word32 y,   /*!< i  : Denominator*/
        Word16 *s); /*!< o  : Additional scalefactor difference*/


/****************************************************************************/
/*!
  \brief   Does fractional integer division of Word32 arg1 by Word32 arg2

   This function scales up both values to avoid losing the accuracy of the outcome
   too much.<br>

   CAUTION: Arg 3 is a Word16 pointer which will point to the scalefactor difference
   s_diff = sub(s2,s1), where s1 and s2 are the scalefactors of the arguments, which
   were shifted in order to e.g. preserve accuracy.
   I.e. the result has to be scaled due to shifting it
   s_diff to the right to obtain the real result of the division.

  \return fractional Word16 integer z = arg1(32bits)/arg2(32bits)
*/
Word16 BASOP_Util_Divide3232_Scale( Word32 x,   /*!< i  : Numerator*/
                                    Word32 y,   /*!< i  : Denominator*/
                                    Word16 *s); /*!< o  : Additional scalefactor difference*/


/****************************************************************************/
/*!
  \brief   Does fractional integer division of Word32 arg1 by Word16 arg2


  \return fractional Word16 integer z = arg1(32bits)/arg2(16bits) , z not normalized
*/
Word16 BASOP_Util_Divide3216_Scale(     Word32 x,   /*!< i  : Numerator  */
                                        Word16 y,   /*!< i  : Denominator*/
                                        Word16 *s); /*!< o  : Additional scalefactor difference*/


/****************************************************************************/
/*!
  \brief   Does fractional division of Word16 arg1 by Word16 arg2


  \return fractional Q15 Word16 z = arg1(Q15)/arg2(Q15)  with scaling s
*/
Word16 BASOP_Util_Divide1616_Scale( Word16 x,   /*!< i  : Numerator*/
                                    Word16 y,   /*!< i  : Denominator*/
                                    Word16 *s); /*!< o  : Additional scalefactor difference*/

/************************************************************************/
/*!
  \brief 	Binary logarithm with 7 iterations

  \param   x

  \return log2(x)/64
 */
/************************************************************************/
Word32 BASOP_Util_Log2(Word32 x);


/************************************************************************/
/*!
  \brief 	Binary power

  Date: 06-JULY-2012 Arthur Tritthart, IIS Fraunhofer Erlangen

  Version with 3 table lookup and 1 linear interpolations

  Algorithm: compute power of 2, argument x is in Q7.25 format
             result = 2^(x/64)
             We split exponent (x/64) into 5 components:
             integer part:      represented by b31..b25  (exp)
             fractional part 1: represented by b24..b20  (lookup1)
             fractional part 2: represented by b19..b15  (lookup2)
             fractional part 3: represented by b14..b10  (lookup3)
             fractional part 4: represented by b09..b00  (frac)
             => result = (lookup1*lookup2*(lookup3+C1*frac)<<3)>>exp

  Due to the fact, that all lookup values contain a factor 0.5
  the result has to be shifted by 3 to the right also.
  Table exp2_tab_long contains the log2 for 0 to 1.0 in steps
  of 1/32, table exp2w_tab_long the log2 for 0 to 1/32 in steps
  of 1/1024, table exp2x_tab_long the log2 for 0 to 1/1024 in
  steps of 1/32768. Since the 2-logarithm of very very small
  negative value is rather linear, we can use interpolation.

  Limitations:

  For x <= 0, the result is fractional positive
  For x > 0, the result is integer in range 1...7FFF.FFFF
  For x < -31/64, we have to clear the result
  For x = 0, the result is ~1.0 (0x7FFF.FFFF)
  For x >= 31/64, the result is 0x7FFF.FFFF

  \param  x

  \return pow(2,(x/64))
 */
/************************************************************************/
Word32 BASOP_Util_InvLog2(Word32 x);

Word16 BASOP_util_norm_s_bands2shift (Word16 x);

/************************************************************************/
/*!
  \brief 	Calculate the headroom of the complex data in a 1 dimensional array

  \return number of headroom bits
 */
Word16 BASOP_util_norm_l_dim1_cplx (const Word32  *re, const Word32  *im, Word16 startBand, Word16 stopBand);

/***********************************************************************/
/*!
  \brief 	Calculate the headroom of the real data in a 2 dimensional array

  \return number of headroom bits
 */
Word16 BASOP_util_norm_l_dim2_real (const Word32 **x,       /*!< matrix of 32 Bit input  */
                                    Word16 startBand, /*!< start band of cplx data */
                                    Word16 stopBand,  /*!< stop band of cplx data  */
                                    Word16 startSlot, /*!< start slot of cplx data */
                                    Word16 stopSlot   /*!< stop slot of cplx data  */
                                   );

/***********************************************************************/
/*!
  \brief 	Calculate the headroom of the complex data in a 2 dimensional array

  \return number of headroom bits
 */
Word16 BASOP_util_norm_l_dim2_cplx (const Word32 * const *re,      /*!< Real part of 32 Bit input */
                                    const Word32 * const *im,      /*!< Imag part if 32 Bit input */
                                    Word16 startBand, /*!< start band of cplx data   */
                                    Word16 stopBand,  /*!< stop band of cplx data    */
                                    Word16 startSlot, /*!< start slot of cplx data   */
                                    Word16 stopSlot   /*!< stop slot of cplx data    */
                                   );

/************************************************************************/
/*!
    \brief Multiplies complex array with constant factor and does shl

    Multiplies a 4-element complex array with a constant factor and
    shifts the result left. The complex array consists of 2 arrays,
    one depicting the real part, the other the imag. part, respective

    \return output array of energies

*/
void   BASOP_util_mult_shift_dim1_4_cplx (Word32 *re,       /*!< Real part of 32 Bit input */
        Word32 *im,       /*!< Imag part if 32 Bit input */
        Word16  a,        /*!< constant factor           */
        Word16  s         /*!< shift factor              */
                                         );

/************************************************************************/
/*!
    \brief Multiplies one column of a complex array with 2 constant factors  and does shl

    Multiplies row i2 of a 16-column 2dim complex array's imaginary part with a certain factor A
    and does the same with the real part and another factor B.
    The result is shifted and saved into the same array, column i.
    The complex input array consists of 2 arrays,
    one depicting the real part, the other the imag. part, respective

    \return output array of energies

*/
void   BASOP_util_mult_shift_dim1_16_cplx (Word32 **re,     /*!< i/o: Real part of 32 Bit input */
        Word32 **im,       /*!< i/o: Imag part if 32 Bit input */
        Word16   i,        /*!< i  : band index i              */
        Word16   i2,       /*!< i  : band index i2             */
        Word16   a,        /*!< i  : constant factor A         */
        Word16   b,        /*!< i  : constant factor B         */
        Word16   s         /*!< i  : shift factor              */
                                          );

/****************************************************************************/
/*!
  \brief   Does a data copy of Word8 *arg1 to Word8 *arg2 with Word16 arg3 number of moves
*/
void copyWord8(     const Word8 *src,   /*!< i  : Source address             */
                    Word8 *dst,         /*!< i  : Destination address        */
                    const Word32 n);    /*!< i  : Number of elements to copy */


/****************************************************************************/
/*!
  \brief   Does a data move of Word16 *arg1 to Word16 *arg2 with Word16 arg3 number of moves
*/
void moveWord16(    const Word16 *src,      /*!< i  : Source address             */
                    Word16 *dst,            /*!< i  : Destination address        */
                    Word16 n);              /*!< i  : Number of elements to copy */

/****************************************************************************/
/*!
  \brief Shifts Word16 arg1 left by Word16 arg2 digits

  Shifts left with saturation and sign extension in case of negative scale factor
*/
Word16 scaleValue16(    const Word16 value,     /*!< i  : value */
                        Word16 scalefactor);    /*!< i  : scalefactor */

/****************************************************************************/
/*!
  \brief Shifts Word32 arg1 left by Word16 arg2 digits

  Shifts left with saturation and sign extension in case of negative scale factor
*/
Word32 scaleValue32(    const Word32 value,     /*!< i  : value          */
                        Word16 scalefactor);    /*!< i  : scalefactor    */


/****************************************************************************/
/*!
  \brief   Sets Word8 array arg1[] to zero for a length of Word16 arg2 elements
*/
void set_zero_Word8(    Word8 X[],      /*!< i  : Address of array                   */
                        Word32 n);      /*!< i  : Number of elements to set to zero  */


void vr_add_w32(const Word32 * X, const Word32 * Y, Word32 * Z, Word16 n);


/****************************************************************************/
/*!
  \brief Shifts Word32 array arg2[] elements left by Word16 arg1 digits

  The result is saved in Word32 arg3[]. <br>
  Processing is done for Word16 arg4 elements

*/
void smulWord32(    const Word16 nLeftshifts,   /*!< i  : scalefactor                           */
                    const Word32 *X,            /*!< i  : Address of input array                */
                    Word32 *Z,                  /*!< i/o: Adress of array holding results       */
                    const Word16 n);            /*!< i  : Number of elements to be processed    */


/****************************************************************************/
/*!
  \brief Shifts Word32 array arg2[] elements left by Word16 arg1 digits
  Result is a Word 16 Array

  The rounded results are saved in Word16 arg3[]. <br>
  Processing is done for Word16 arg4 elements

*/
void smulWord32_r(  const Word16 nLeftshifts,   /*!< i  : scalefactor                           */
                    const Word32 *X,            /*!< i  : Address of input array                */
                    Word16 *Z,                  /*!< i/o: Address of output array               */
                    const Word16 n);            /*!< i  : Number of elements to be processed    */


/****************************************************************************/
/*!
  \brief Shifts Word16 array arg2[] elements left by Word16 arg1 digits

  The result is saved in Word16 arg3[]. <br>
  Processing is done for Word16 arg4 elements

*/
void smulWord16(    const Word16 nLeftshifts,   /*!< i  : scalefactor                           */
                    const Word16 *X,            /*!< i  : Address of input array                */
                    Word16 *Z,                  /*!< i/o: Address of output array               */
                    const Word16 n);            /*!< i  : Number of elements to be processed    */

/****************************************************************************/
/*!
  \brief    Does a multiplication of Word32 * Word16 input values

  \return   z32 = x32 * y16
*/
Word32 L_mult0_3216(    Word32 x,   /*!<   : Multiplier     */
                        Word16 y);  /*!<   : Multiplicand   */

/* Calculate sin/cos. Angle in 2Q13 format, result has exponent = 1 */
Word16 getCosWord16(Word16 theta);
Word32 getCosWord32(Word32 theta);
/**
 * \brief calculate cosine of angle. Tuned for ISF domain.
 * \param theta Angle normalized to radix 2, theta = (angle in radians)*2.0/pi
 * \return result with exponent 0.
 */
Word16 getCosWord16R2(Word16 theta);

/****************************************************************************/
/*!
  \brief    square root abacus algorithm

  \return integer sqrt(x)
 */
Word16 getSqrtWord32(Word32 x);

/****************************************************************************/
/*!
  \brief    finds index of min Word16 in array

  \return   index of min Word16
 */
Word16 findIndexOfMinWord16(Word16 *x, const Word16 len);

/****************************************************************************/
/*!
  \brief    finds index of min Word32 in array

  \return   index of min Word32
 */
Word16 findIndexOfMinWord32(Word32 *x, const Word16 len);

/****************************************************************************/
/*!
  \brief    finds index of max Word16 in array

  \return   index of max Word16
 */
Word16 findIndexOfMaxWord16(Word16 *x, const Word16 len);

/****************************************************************************/
/*!
  \brief    finds index of max Word32 in array

  \return   index of max Word32
 */
Word16 findIndexOfMaxWord32(Word32 *x, const Word16 len);

/****************************************************************************/
/*!
  \brief    16x16->16 integer multiplication without overflow control

  \return 16x16->16 integer
 */
Word16 imult1616(Word16 x, Word16 y);

/****************************************************************************/
/*!
  \brief    32x16->32 integer multiplication with overflow control

  \return 32x16->32 integer
 */
Word32 imult3216(Word32 x, Word16 y);

/****************************************************************************/
/*!
  \brief    16/16->16 unsigned integer division

  x and y have to be positive, x has to be < 16384

  \return 16/16->16 integer
 */

Word16 idiv1616U(Word16 x, Word16 y);

/****************************************************************************/
/*!
  \brief    16/16->16 signed integer division

  x and y have to be positive, x has to be < 16384

  \return 16/16->16 integer
 */

Word16 idiv1616(Word16 x, Word16 y);


/*-------------------------------------------------------------------*
 * \brief: CodecB_Dot_product: please use Dot_product12_offs instead if possible
 *
 * Compute scalar product of <x[],y[]> using accumulator.
 * Performs no normalization, as opposed to Dot_product12()
 *-------------------------------------------------------------------*/
Word32 CodecB_Dot_product(     /*<! o  : Sum              */
    const Word16 x[],   /*<! i  : 12bits: x vector */
    const Word16 y[],   /*<! i  : 12bits: y vector */
    const Word16 lg     /*<! i  : vector length    */
);

/*------------------------------------------------------------------*
 * Dot_product16HQ:
 *
 * \brief Compute scalar product of <x[],y[]> using 64-bit accumulator.
 *
 * Performs normalization of the result, returns the exponent
 * Note: In contrast to dotWord32, no headroom is required for data
 *       in x[] and y[], means, they may have any format Qn
 *------------------------------------------------------------------*/
Word32 Dot_product16HQ(   /*<! o : normalized result              Q31 */
    const Word32 L_off,  /*<! i : initial sum value               Qn */
    const Word16 x[],    /*<! i : x vector                        Qn */
    const Word16 y[],    /*<! i : y vector                        Qn */
    const Word16 lg,     /*<! i : vector length, range [0..7FFF]  Q0 */
    Word16 * exp         /*<! o : exponent of result in [-32,31]  Q0 */
);


/*------------------------------------------------------------------*
 * norm_llQ31:
 *
 * \brief Compute normalized Q31 Values out of overflowed Q31 value
 *
 * Performs the calculation of a normalized Q31 Value with its
 * scalingfactor, taking into account the overflowed Q31 input value
 * and the number of Carrys, collected.
 *------------------------------------------------------------------*/
Word32 norm_llQ31(        /* o : normalized result              Q31 */
    Word32 L_c,          /* i : upper bits of accu             Q-1 */
    Word32 L_sum,        /* i : lower bits of accu, unsigned   Q31 */
    Word16 * exp         /* o : exponent of result in [-32,31]  Q0 */
);


/**
 * \brief Compute dot product of 2 32 bit vectors
 * \param x input vector 1, Q31
 * \param y input vector 2, Q31
 * \param headroom amount of headroom bits of both input vectors
 * \param length the length of the input vectors
 * \param result_e pointer to where the exponent of the result will be stored into
 * \return the dot product of x and y, Q31
 */
Word32 Dot_product32Norm(const Word32 *x, const Word32 *y, const Word16 headroom, const Word16 length, Word16 *result_e);

/**
 * \brief Compute dot product of 1 32 bit vectors with itself
 * \param x input vector 1
 * \param headroom amount of headroom bits the input vector
 * \param length the length of the input vector
 * \param result_e pointer to where the exponent of the result will be stored into
 * \return the dot product of x and x.
 */
Word32 Norm32Norm(const Word32 *x, const Word16 headroom, const Word16 length, Word16 *result_e);


/*------------------------------------------------------------------*
 * Dot_productSq16HQ:
 *
 * \brief Compute scalar product of <x[],x[]> using 64-bit accumulator.
 *
 * Performs normalization of the result, returns the exponent
 * Note: In contrast to dotWord32, no headroom is required for data
 *       in x[], means, they may have any format Qn
 *------------------------------------------------------------------*/
Word32 Dot_productSq16HQ( /*<! o : normalized result              Q31 */
    const Word32 L_off,  /*<! i : initial sum value               Qn */
    const Word16 x[],    /*<! i : x vector                        Qn */
    const Word16 lg,     /*<! i : vector length, range [0..7FFF]  Q0 */
    Word16 * exp         /*<! o : exponent of result in [-32,31]  Q0 */
);

/*------------------------------------------------------------------*
 * dotp_s_fx:
 *
 * \brief Compute scalar product of <x[],y[]> using 64-bit accumulator.
 *
 * Performs no normalization of the result
 *------------------------------------------------------------------*/
Word32 dotp_s_fx(       /*<! o  : dot product of vector x and y 16Q15 */
    const Word16 *x,   /*<! i  : vector x                       6Q9  */
    const Word16 *y,   /*<! i  : vector y                       6Q9  */
    const Word16  n,   /*<! i  : vector length                   Q0  */
    Word16  s    /*<! i  : headroom                        Q0  */
);

/*-------------------------------------------------------------------*
 * Sum32:
 *
 * \brief Return the sum of one 32 bits vector
 *-------------------------------------------------------------------*/
Word32 Sum32(             /*<! o  : the sum of the elements of the vector */
    const Word32 *vec,    /*<! i  : input vector                          */
    const Word16 lvec     /*<! i  : length of input vector                */
);

/*-------------------------------------------------------------------*
 * Sum32_scale:
 *
 * \brief Return the sum of one 32 bits vector scaled by a factor
 *-------------------------------------------------------------------*/
Word32 Sum32_scale(             /* o  : the sum of the elements of the vector */
    const Word32 *vec,    /* i  : input vector                          */
    const Word16 lvec,    /* i  : length of input vector                */
    const Word16 scale    /* i  : right shift in Q0 to be applied to each summand */
);

/*-------------------------------------------------------------------*
 * Emaximum2:
 *
 * \brief Find index of a maximum energy in a vector
 *-------------------------------------------------------------------*/
Word16 Emaximum2(               /*<! o  : return index with energy value in vector  Q0 */
    const Word16 Qvec,          /*<! i  : Q of input vector                         Q0 */
    const Word16 *vec,          /*<! i  : input vector                              Qx */
    const Word16 lvec,          /*<! i  : length of input vector                    Q0 */
    Word32 *ener_max      /*<! o  : maximum energy value                      Q0 */
);

/*-------------------------------------------------------------------*
 * Emaximum2_Qx:
 *
 * \brief Find maximum energy in a vector
 *
 *  Calculates energy in Qx format. Format has to be handled by the caller
 *-------------------------------------------------------------------*/
void Emaximum2_Qx(
    const Word16 *vec,          /* i  : input vector                              Qx */
    const Word16 lvec,          /* i  : length of input vector                    Q0 */
    Word16 *ener_max,     /* o  : maximum energy value                      Qx */
    Word16 *exp_enrq      /* o  :  scalingfactor of ener_max                   */
);

/**
 * \brief return 2 ^ (exp * 2^exp_e)
 * \param exp_m mantissa of the exponent to 2.0f
 * \param exp_e exponent of the exponent to 2.0f
 * \param result_e pointer to a INT where the exponent of the result will be stored into
 * \return mantissa of the result
 */
Word32 BASOP_util_Pow2(
    const Word32 exp_m, const Word16 exp_e,
    Word16 *result_e
);


/* deprecated, use ISqrt32norm! */
Word32 Isqrt_lc(
    Word32 frac,  /*!< (i)   Q31: normalized value (1.0 < frac <= 0.5) */
    Word16 * exp  /*!< (i/o)    : exponent (value = frac x 2^exponent) */
);

/**
 * \brief return 1/x
 * \param x index of lookup table
 * \return Word16 value of 1/x
 */
Word16 getNormReciprocalWord16(Word16 x);

/**
 * \brief return (1/x) << s
 * \param x index of lookup table
 * \param s shift factor
 * \return Word16 value of (1/x) << s
 */
Word16 getNormReciprocalWord16Scale(Word16 x, Word16 s);

/*************************************************************************
 *
 *   FUNCTION:   BASOP_Util_fPow()
 */
/**
 * \brief BASOP_Util_fPow
 *
 *   PURPOSE:   Computes pow(base_m, base_e, exp_m, exp_e), where base_m and base_e
 *              specify the base, and exp_m and exp_e specify the exponent.
 *              The result is returned in a mantissa and exponent representation.
 *
 *   DESCRIPTION:
 *        The function BASOP_Util_fPow(L_x) calculates the power function by
 *        calculating 2 ^ (log2(base)*exp)
 *
 * \param base_m mantissa of base
 * \param base_e exponent of base
 * \param exp_m mantissa of exponent
 * \param exp_e exponent of exponent
 * \param result_e pointer to exponent of result
 * \return Word32 mantissa of result
 *
 *************************************************************************/

Word32 BASOP_Util_fPow(               /* (o) : mantissa of result                                              */
    Word32 base_m, Word16 base_e, /* (i) : input value for base (mantissa and exponent)                    */
    Word32 exp_m, Word16 exp_e,   /* (i) : input value for exponent (mantissa and exponent)                */
    Word16 *result_e              /* (o) : output pointer to exponent of result                            */
);

/****************************************************************************/
/*!
  \brief   Accumulates multiplications

  Does the Multiplication of a Word32 and a Word16 arrays with elements arg1[i] * arg2[i] after trunkating
  the elements to 16 bit and accumulates all results
  over a length of arg3

  \return Word32 result of accumulated multiplications over Word32 array arg1 and Word16 array arg2
*/
Word32 dotWord32_16(const Word32 * X,   /*!< i  : Address of Array X         */
                    const Word16 * Y,   /*!< i  : Address of Array Y         */
                    Word16 n);          /*!< i  : Number of elements to mac  */

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
 |                   means: L_off = FL2WORD32_SCALE(0.01f,31-19)             |
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
    Word32 L_off                          /* (i) initial summation offset /2              */
);

Word32 Dot_product15_offs(                 /* (o) Q31: normalized result (1 < val <= -1) */
    const Word16 x[],                     /* (i) 15bits: x vector                       */
    const Word16 y[],                     /* (i) 15bits: y vector                       */
    const Word16 lg,                      /* (i)    : vector length in range [1..256]   */
    Word16 *exp,                          /* (o)    : exponent of result (0..+30)       */
    Word32 L_off                          /* (i) initial summation offset               */
);

/****************************************************************************/
/*!
  \brief   Accumulates multiplications

	Accumulates the elementwise multiplications of Word32 Array X with Word16 Array Y
	pointed to by arg1 and arg2. Length of to be multiplied arrays is arg3

  \return Word32 result of accumulated multiplications over Word32 array arg1 and Word16 array arg2
*/
Word32 dotWord32_16_scale(const Word32 * X, const Word16 * Y, Word16 n);

/*!**********************************************************************
   \brief   Add two values given by mantissa and exponent.

   Mantissas are in 32-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

************************************************************************/
Word32 BASOP_Util_Add_Mant32Exp                  /*!< o: normalized result mantissa */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e,       /*!< i: Exponent of 2nd operand b  */
 Word16  *ptr_e);    /*!< o: exponent of result         */

/*!**********************************************************************
   \brief   Returns maximum of two normalized values given by mantissa and exponent.

   Mantissas are in 32-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

************************************************************************/
Word32 BASOP_Util_Max_Mant32Exp                  /*!< o: result mantissa */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e,       /*!< i: Exponent of 2nd operand b  */
 Word16  *ptr_e);    /*!< o: exponent of result         */

/*!**********************************************************************
   \brief   Returns minimum of two normalized values given by mantissa and exponent.

   Mantissas are in 32-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

************************************************************************/
Word32 BASOP_Util_Min_Mant32Exp                  /*!< o: normalized result mantissa */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e,       /*!< i: Exponent of 2nd operand b  */
 Word16  *ptr_e);     /*!< o: exponent of result        */

/*!**********************************************************************
   \brief   Returns the comparison result of two normalized values given by mantissa and exponent.
            return value: -1: a < b, 0: a == b, 1; a > b

   Mantissas are in 32-bit-fractional format with values between 0 and 1. <br>
   The base for exponents is 2.  Example:  \f$  a = a\_m * 2^{a\_e}  \f$<br>

************************************************************************/
Word16 BASOP_Util_Cmp_Mant32Exp                  /*!< o: flag: result of comparison */
(Word32   a_m,       /*!< i: Mantissa of 1st operand a  */
 Word16   a_e,       /*!< i: Exponent of 1st operand a  */
 Word32   b_m,       /*!< i: Mantissa of 2nd operand b  */
 Word16   b_e);       /*!< i: Exponent of 2nd operand b  */

/****************************************************************************/
/*!
  \brief   Accumulates multiplications

	Accumulates the elementwise multiplications of Word32 Array X with Word16 Array Y
	pointed to by arg1 and arg2 with specified headroom. Length of to be multiplied arrays is arg3,
    headroom with has to be taken into account is specified in arg4

  \return Word32 result of accumulated multiplications over Word32 array arg1 and Word16 array arg2 and Word16 pointer
          to exponent correction factor which needs to be added to the exponent of the result vector
*/
Word32 dotWord32_16_guards(const Word32 * X, const Word16 * Y, Word16 n, Word16 hr, Word16 * shift);

/********************************************************************
 * bufferCopyFx
 *
 * \brief copies buffer while preserving Format of destination buffer
*********************************************************************
*/
void bufferCopyFx(
    Word16* src,       /*<! Qx  pointer to input buffer                */
    Word16* dest,      /*<! Qx  pointer to output buffer               */
    Word16 length,     /*<! Q0  length of buffer to copy               */
    Word16 Qf_src,     /*<! Q0  Q format (frac-bits) of source buffer  */
    Word16 Qf_dest,    /*<! Q0  Q format (frac-bits )of dest buffer    */
    Word16 Q_src,      /*<! Q0  exponent of source buffer              */
    Word16 Q_dest      /*<! Q0  exponent of destination buffer         */
);

/****************************************************************************/
/*!
  \brief   Accumulates multiplications

	Accumulates the elementwise multiplications of Word32 Array bufX32 with Word16 Array bufY16
	pointed to by arg1 to arg4 including the corresponding exponents. Length of to be multiplied arrays is arg5,

  \return Word32 result of accumulated multiplications over Word32 array arg1 and Word16 array arg3 and Word16 pointer
          to exponent of the result
*/
Word32 dotWord32_16_Mant32Exp(const Word32 *bufX32,/* i: 32-bit buffer with unknown headroom */
                              Word16 bufX32_exp,   /* i: exponent of buffer bufX32           */
                              const Word16 *bufY16,/* i: 16-bit buffer quite right-aligned   */
                              Word16 bufY16_exp,   /* i: exponent of buffer bufY16           */
                              Word16 len,          /* i: buffer len to process               */
                              Word16 *exp);         /* o: result exponent                     */

/*!**********************************************************************
   \brief   Converts linear factor or energy to Decibel
            return value: fEnergy=0: 20 * log10(x * 2^{x\_e}),
                          fEnergy=1: 10 * log10(x * 2^{x\_e})

   Mantissa x is in 32-bit-fractional format with values between 0 and 1. <br>
   The base for exponent x_e is 2. <br>

************************************************************************/
Word16 BASOP_Util_lin2dB(                 /*!< o: dB value (7Q8) */
    Word32 x,        /*!< i: mantissa */
    Word16 x_e,      /*!< i: exponent */
    Word16 fEnergy); /*!< i: flag indicating if x is energy */

/*!**********************************************************************
   \brief   Calculates atan(x).
************************************************************************/
Word16 BASOP_util_atan(                 /*!< o:  atan(x)           [-pi/2;pi/2]   1Q14  */
    Word32 x         /*!< i:  input data        (-64;64)       6Q25  */
);

/*!**********************************************************************
   \brief   Calculates atan2(y,x).
************************************************************************/
Word16 BASOP_util_atan2(              /*!< o: atan2(y,x)    [-pi,pi]        Q13   */
    Word32 y,     /*!< i:                                     */
    Word32 x,     /*!< i:                                     */
    Word16 e      /*!< i: exponent difference (exp_y - exp_x) */
);

/*!**********************************************************************
   \brief   addVec16 returns vecZ = vecX+vecY

************************************************************************/

void addVec16(
    Word16 X[],   /*!< i  : input vector1*/
    Word16 Y[],   /*!< i  : input vector2*/
    Word16 Z[],   /*!< o  : output vector*/
    Word16 len    /*!< i  : length */
);

/*!**********************************************************************
   \brief   norm_llQ31 returns Word32 with scalingfactor, with 2 32bit accus as input

************************************************************************/
Word32 norm_llQ31(        /* o : normalized result              Q31 */
    Word32 L_c,          /* i : upper bits of accu             Q-1 */
    Word32 L_sum,        /* i : lower bits of accu, unsigned   Q31 */
    Word16 * exp         /* o : exponent of result in [-32,31]  Q0 */
);

/* compare two positive normalized 16 bit mantissa/exponent values */
/* return value: positive if first value greater, negative if second value greater, zero if equal */
Word16 compMantExp16Unorm(Word16 m1, Word16 e1, Word16 m2, Word16 e2);



#endif /* __BASOP_UTIL_H__ */
