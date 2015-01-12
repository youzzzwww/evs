/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"
#include "rom_com_fx.h"


Word32 calc_gain_inov(                      /* returns innovation gain              Q16 */
    const Word16 *code,   /* i  : algebraic excitation            Q9  */
    Word16 lcode,         /* i  : Subframe size                   Q0  */
    Word32 *dotp,         /* o  : intermediate result           Q31-e */
    Word16 *dotp_e        /* o  : intermediate result exponent    Q0  */
)
{
    Word32 L_tmp;
    Word16 exp_L_tmp, i;

    /* L_tmp = dot_product(code, code, lcode) + 0.01 */
    L_tmp = Dot_product12_offs(code, code, lcode, &exp_L_tmp, FL2WORD32_SCALE(0.01f/2.0f, 31-19));
    exp_L_tmp = sub(exp_L_tmp, 18);

    /* gain_inov = 1.0f / sqrt((dot_product(code, code, lcode) + 0.01) / lcode) */
    /* Note: lcode is in range: 32,40,64,80 */
    assert((lcode == 32) || (lcode == 40) || (lcode == 64) || (lcode == 80));
    if (s_and(lcode, sub(lcode, 1)) != 0)
    {
        L_tmp = Mpy_32_32(L_tmp, FL2WORD32(64.0/80.0));
    }
    exp_L_tmp = sub(exp_L_tmp, sub(14, norm_s(lcode)));

    i = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, i);
    exp_L_tmp = sub(exp_L_tmp, i);

    if (dotp != NULL)
    {
        *dotp = L_tmp;
        move32();
    }
    if (dotp_e != NULL)
    {
        *dotp_e = exp_L_tmp;
        move16();
    }

    L_tmp = ISqrt32norm(L_tmp, &exp_L_tmp);

    return L_shl(L_tmp, sub(exp_L_tmp, 15)); /* 15Q16 */
}

