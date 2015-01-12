/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "basop_mpy.h"
#include "stl.h"
#include "options.h" /* Needed for Stack Counting Mechanism Macros (when Instrumented) */

Word32 Mpy_32_16_1(Word32 x, Word16 y)
{
    Word32 mh;
    UWord16 ml;

    Mpy_32_16_ss(x, y, &mh, &ml);

    return (mh);
}

Word32 Mpy_32_16_r(Word32 x, Word16 y)
{
    Word32 mh;
    UWord16 ml;

    Mpy_32_16_ss(x, y, &mh, &ml);

    if(s_and(ml, -32768 /* 0x8000 */))
    {
        mh = L_add(mh, 1);
    }

    return (mh);
}

Word32 Mpy_32_32(Word32 x, Word32 y)
{
    Word32 mh;
    UWord32 ml;

    Mpy_32_32_ss(x, y, &mh, &ml);

    return (mh);
}

void cplxMpy_32_16(Word32 *c_Re,
                   Word32 *c_Im,
                   const Word32 a_Re,
                   const Word32 a_Im,
                   const Word16 b_Re,
                   const Word16 b_Im)
{
    *c_Re = L_sub(Mpy_32_16_1(a_Re,b_Re),Mpy_32_16_1(a_Im,b_Im));
    move32();
    *c_Im = L_add(Mpy_32_16_1(a_Re,b_Im),Mpy_32_16_1(a_Im,b_Re));
    move32();
}
