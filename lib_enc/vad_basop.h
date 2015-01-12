/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef __VAD_BASOP_H__
#define __VAD_BASOP_H__

#include "typedef.h"
#include "basop32.h"
#include "stat_enc_fx.h"

Word16 vadmin( Word16 a,
               Word16 b
             );

Word32 vad_Sqrt_l( Word32 i_s32Val,
                   Word16 *io_s16Q
                 );

Word32 fft_vad_Sqrt_l( Word32 i_s32Val,
                       Word16 i_s16Q,
                       Word16 *o_s16Q
                     );

T_VAD_EXP VAD_AddExp( T_VAD_EXP i_tExp1,
                      T_VAD_EXP i_tExp2
                    );

Word16 VAD_L_CMP( Word32 s32Mantissa1,
                  Word16 i_tExp1,
                  Word32 s32Mantissa2,
                  Word16 i_tExp2
                );

Word32 VAD_L_ADD( Word32 s32Mantissa1,
                  Word16 i_tExp1,
                  Word32 s32Mantissa2,
                  Word16 i_tExp2,
                  Word16 *s16Exp
                );

Word32 VAD_L_div( Word32 L_var1,
                  Word32 L_var2,
                  Word16 Q_L_var1,
                  Word16 Q_L_var2,
                  Word16 *Q_OUT
                );

Word32 VAD_Log2( Word32 i_s32Val,
                 Word16 i_s16Q
               );

Word16 ffr_getSfWord32( Word32 *vector,
                        Word16 len
                      );

Word32 VAD_Pow( Word32 i_s32Base,
                Word32 i_s32Exp,
                Word16 i_s16BaseQ,
                Word16 i_s16ExpQ,
                Word16 *o_pOuQ
              );

Word32 VAD_Pow2( Word32 i_s32X,
                 Word16 i_s16Q,
                 Word16 *o_pOuQ
               );

Word16 FixSqrt( Word32 i_s32Val,
                Word16 *io_s16Q
              );

void cfftf(	Word16* scale,
            complex_32 *c,
            complex_32 *ch,
            const complex_16 *wa
          );


#endif
