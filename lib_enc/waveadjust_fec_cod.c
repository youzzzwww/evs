/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "stat_com.h"
#include "stl.h"    /* for wmc_tool */


Word32 SFM_Cal(Word32 magn[], Word16 n)
{
    /* Counted Dymamic RAM: 16 words */
    Word32 logCurFlatness;
    Word32 magn_abs, frac, logMagn, sumLogMagn, sumMagn;
    Word16 i, norm_value, logSumMagn, logn;
    sumLogMagn = L_deposit_l(0);
    sumMagn = L_deposit_l(0);
    FOR(i = 0; i < n; i++)
    {
        /* log2(magn(i)) */
        magn_abs = L_abs(magn[i]);
        norm_value = norm_l(L_max(magn_abs, 1));
        /* next two codes lost precision. */
        frac = L_and(L_shr(L_shl(magn_abs, norm_value), 22), 0xFF);
        logMagn = L_deposit_l(add(shl(sub(30, norm_value), 8), kLog2TableFrac_x[frac]));; /* Q8 */
        /* sum(log2(magn(i))) */
        sumLogMagn =L_add(sumLogMagn, logMagn); /* Q8 */

        sumMagn =L_add(sumMagn, magn_abs);
    }

    IF(L_sub(sumMagn,MAX_32)==0)
    {
        sumMagn = L_deposit_l(0);
        FOR(i = 0; i < n; i++)
        {
            magn_abs = L_shr(L_abs(magn[i]), 8);
            sumMagn = L_add(sumMagn, magn_abs);
        }
        /* log2(sumMagn) */
        norm_value = norm_l(sumMagn);
        frac = L_and(L_shr(L_shl(sumMagn, norm_value), 22), 0xFF);
        logSumMagn = add(shl(sub(38,norm_value), 8), kLog2TableFrac_x[frac]); /* Q8 */
    }
    ELSE
    {
        /* log2(sumMagn) */
        norm_value = norm_l(sumMagn);
        frac = L_and(L_shr(L_shl(sumMagn, norm_value), 22), 0xFF);
        logSumMagn = add(shl(sub(30, norm_value), 8), kLog2TableFrac_x[frac]); /* Q8 */
    }

    /* log2(n) */
    norm_value = norm_l(n);
    frac = L_and(L_shr(L_shl(n, norm_value), 22), 0xFF);
    logn = add(shl(sub(30, norm_value), 8), kLog2TableFrac_x[frac]); /* Q8 */


    logMagn=L_sub(L_mult0(n, sub(logSumMagn, logn)), sumLogMagn);
    logMagn = (Word32)L_max(0, logMagn);

    logCurFlatness = L_deposit_l(div_l(L_shl(logMagn, 1), n));
    frac = L_and(logCurFlatness, 0xFF);
    norm_value = (Word16)L_shr(logCurFlatness, 8);
    move16();

    logCurFlatness = L_sub(kExp2TableFrac_x[frac], 33);
    logCurFlatness = L_shl(logCurFlatness, sub(16, norm_value)); /* Q31 */


    return logCurFlatness;
}

