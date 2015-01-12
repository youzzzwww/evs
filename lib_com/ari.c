/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include "assert.h"
#include "prot_fx.h"
#include "basop_mpy.h"
#include "cnst_fx.h"
#include "stl.h"

/**
 * \brief  31x16 Bit multiply (x*y)
 *
 * \param[i] xh  high part, bit [30..15]
 * \param[i] xl  low part, 15 LSBits
 * \param[i] y
 *
 * \return x*y
 */
Word32 L_multi31x16_X2(Word16 xh, Word16 xl, Word16 y)
{
    Word32 z;

    z = L_shl(L_mult0(xh,y),15);
    z = L_mac0(z,xl,y);

    return z;
}

/*---------------------------------------------------------------
  Ari 14 bits common routines
  -------------------------------------------------------------*/

/**
 * \brief  Integer Multiply
 *
 * \param[i] r
 * \param[i] c
 *
 * \return r*c
 */
Word32 mul_sbc_14bits(Word32 r, Word16 c)
{
    Word32 ret;


    /*
      temp = (((int32) r)*((int32) c))>>stat_bitsnew;
     */
    assert(stat_bitsnew == 14);
    ret = Mpy_32_16_1(L_shl(r,15-stat_bitsnew), c);

    /*assert( (((int) r)*((int) c))>>stat_bitsnew == ret);*/

    return (ret);
}
