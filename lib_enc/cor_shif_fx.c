/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * correlation_shift_fx
 *
 * Find normalized correlation correction dependent on estimated noise
 * Note: this variable is basically active only if noise suppression
 * is desactivated. Otherwise, for default NS = 14 dB and up to 10dB SNR
 * it can be assumed about 0
 *-------------------------------------------------------------------*/

Word16 correlation_shift_fx(  /* o  : noise dependent voicing correction     Q15 */
    Word16 totalNoise_fx      /* i/o: noise estimate over all critical bands  Q8 */
)
{
    Word16  corr_shift_fx, e_Noise, f_Noise, wtmp;
    Word32 Ltmp;

    corr_shift_fx = 0;
    move16();

    IF (sub(totalNoise_fx, 7215) > 0) /* to make corr_shift > 0.0 */
    {
        /*------------------------------------------------------------*
         * useful values range from 0 to 1 (can saturate at 1.0) Q31 value
         * corr_shift = 2.4492E-4 * exp(0.1596 * totalNoise) - 0.022
         * Result in Q14
         *------------------------------------------------------------*/
        Ltmp = L_mult(totalNoise_fx, 7545);        /* Q24 */
        Ltmp = L_shr(Ltmp, 8);                    /* Q24 -> Q16 */
        f_Noise = L_Extract_lc(Ltmp, &e_Noise);
        wtmp = extract_l(Pow2(14, f_Noise));      /* Put 14 as exponent */

        e_Noise = sub(e_Noise, 14);               /* Retreive exponent of wtmp */
        Ltmp = Mpy_32_16(8, 837, wtmp);           /* 2.4492e-4(Q31) * exp(0.1596*totalNoise) */
        Ltmp = L_shl(Ltmp,add(e_Noise, 15));      /* Result in Q31 */
        corr_shift_fx = round_fx(L_sub(Ltmp, 47244640));  /* Represents corr_shift in Q15 */
    }
    corr_shift_fx = s_min(corr_shift_fx,16384); /* limit to 0.5 */

    return corr_shift_fx;
}
