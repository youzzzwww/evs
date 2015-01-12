/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "prot_fx.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "basop_util.h"
#include "stl.h"
#include "options.h" /* Needed for Stack Counting Mechanism Macros (when Instrumented) */

Word32 vad_Sqrt_l(     /* o : output value,                          Q31 */
    Word32 i_s32Val,
    Word16 *io_s16Q
)
{
    Word16 exp;
    Word32 result;

    exp = sub(31,*io_s16Q);
    result = Sqrt32(i_s32Val, &exp);
    *io_s16Q = sub(31,exp);
    move16();

    return (result);
}

Word32 fft_vad_Sqrt_l(     /* o : output value,                          Q31 */
    Word32 i_s32Val,
    Word16 i_s16Q,
    Word16 *o_s16Q
)
{
    Word16 exp;
    Word32 result;

    exp = sub(31, i_s16Q);
    result = Sqrt32(i_s32Val, &exp);
    *o_s16Q = sub(31, exp);
    move16();

    return (result);
}

Word32 VAD_L_div(Word32 L_var1, Word32 L_var2,Word16 Q_L_var1,Word16 Q_L_var2,Word16 *Q_OUT )
{
    Word32 result;

    result = L_deposit_h(BASOP_Util_Divide3232_Scale(L_var1, L_var2, Q_OUT));
    move16();
    *Q_OUT = add(sub(sub(31, *Q_OUT), Q_L_var2), Q_L_var1);
    return result;
}

Word32 VAD_Log2(Word32 i_s32Val, Word16 i_s16Q)
{
    Word32 result;

    /* log10(x) = log2(x) * 1.0/log2(10), exponent LD_DATA_SCALE - 1 */
    result = BASOP_Util_Log2(i_s32Val);

    result = L_add(result, L_shl(L_deposit_l(sub(31, i_s16Q)), 31-LD_DATA_SCALE));

    return result;
}

T_VAD_EXP VAD_AddExp(T_VAD_EXP i_tExp1, T_VAD_EXP i_tExp2)
{
    Word16 s16Shift;
    T_VAD_EXP tRtnVal;

    if(i_tExp1.s32Mantissa == 0)
    {
        return i_tExp2;
    }

    if(i_tExp2.s32Mantissa == 0)
    {
        return i_tExp1;
    }
    s16Shift = sub(s_min(i_tExp1.s16Exp,i_tExp2.s16Exp),1);
    tRtnVal.s32Mantissa = L_add(L_shr(i_tExp2.s32Mantissa, sub(i_tExp2.s16Exp,s16Shift)), L_shr(i_tExp1.s32Mantissa, sub(i_tExp1.s16Exp,s16Shift)));
    tRtnVal.s16Exp = s16Shift;
    move16();

    s16Shift =  norm_l(tRtnVal.s32Mantissa);
    tRtnVal.s32Mantissa = L_shl(tRtnVal.s32Mantissa, s16Shift);
    tRtnVal.s16Exp = add(tRtnVal.s16Exp,s16Shift);

    return tRtnVal;

}
Word32 VAD_L_ADD(Word32 s32Mantissa1,Word16 i_tExp1, Word32 s32Mantissa2, Word16 i_tExp2,Word16 *s16Exp)
{
    Word32 result;

    result = BASOP_Util_Add_Mant32Exp(s32Mantissa1, sub(31, i_tExp1), s32Mantissa2, sub(31, i_tExp2), s16Exp);

    move16();
    *s16Exp = sub(31, *s16Exp);

    return result;
}

Word16 VAD_L_CMP(Word32 s32Mantissa1,Word16 i_tExp1, Word32 s32Mantissa2, Word16 i_tExp2)
{
    Word16 ret;

    ret = BASOP_Util_Cmp_Mant32Exp(s32Mantissa1, sub(31, i_tExp1), s32Mantissa2, sub(31, i_tExp2));

    return ret;
}

Word16 FixSqrt(Word32 i_s32Val, Word16 *io_s16Q)
{
    Word16 result, exp;

    exp = sub(31, *io_s16Q);
    result = round_fx(Sqrt32(i_s32Val, &exp));
    move16();
    *io_s16Q = sub(15, exp);

    return result;
}

Word32 VAD_Pow(Word32 i_s32Base, Word32 i_s32Exp,
               Word16 i_s16BaseQ, Word16 i_s16ExpQ, Word16 *o_pOuQ)
{
    Word32 result;
    result = BASOP_Util_fPow(i_s32Base, sub(31, i_s16BaseQ), i_s32Exp, sub(31, i_s16ExpQ), o_pOuQ);
    move16();
    *o_pOuQ = sub(31, *o_pOuQ);
    return result;
}

Word32 VAD_Pow2(Word32 i_s32X,  Word16 i_s16Q, Word16 *o_pOuQ)
{
    Word32 result;
    result = BASOP_util_Pow2(i_s32X, sub(31, i_s16Q), o_pOuQ);
    move16();
    *o_pOuQ = sub(31, *o_pOuQ);

    return result;
}
