/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* tables definition                    */
#include "stl.h"


Word32 Interpol_lc_fx(        /* o  : interpolated value             Qx+16 */
    const Word16 *x,       /* i  : input vector                     Q0  */
    const Word16 *win,     /* i  : interpolation window             Q14 */
    const Word16 frac,     /* i  : fraction (0..up_samp)            Q0  */
    const Word16 up_samp,  /* i  : upsampling factor                Q0  */
    const Word16 nb_coef   /* i  : number of coefficients           Q0  */
)
{
    Word16 i;
    const Word16 *c1, *c2, *x2;
    Word32 L_sum;

    x2 = &x[1];
    c1 = &win[frac];
    c2 = &win[sub(up_samp,frac)];
    L_sum = L_mult0(*x--, *c1);
    L_sum = L_mac0(L_sum, *x2++, *c2);
    FOR (i=1; i<nb_coef; i++)
    {
        c2 += up_samp;        /* move16() not needed, since the coefficient can be rearrange in bit exact way */
        c1 += up_samp;
        /* Using L_mac0 limits the risk of saturation during the loop, saturation may occures after the loop */
        if (*c1)
        {
            L_sum = L_mac0(L_sum, *x, *c1);
        }
        --x;
        if (*c2)
        {
            L_sum = L_mac0(L_sum, *x2, *c2);
        }
        ++x2;
    }
    L_sum = L_shl(L_sum,1);

    return L_sum;
}

/*--------------------------------------------------------------------------*
 * Interpol_4()
 *
 * For interpolating the normalized correlation with 1/4 resolution.
 *--------------------------------------------------------------------------*/
Word16 Interpol_4(         /* o  : interpolated value  */
    Word16 * x,      /* i  : input vector        */
    Word16 frac      /* i  : fraction (-4..+3)   */
)
{
    Word16 i;
    Word32 L_sum;

    x = x - L_INTERPOL1 + 1;

    L_sum = L_mult(x[0], (inter4_1_fx+UP_SAMP-1)[-frac]);
    FOR (i = 1;  i < 2 * L_INTERPOL1; i++)
    {
        /*
         * Here, additions with UP_SAMP are not counted
         * because, the window could easily be modified
         * so that the values needed are contiguous.
         */
        frac -= UP_SAMP;
        L_sum = L_mac(L_sum, x[i], (inter4_1_fx+UP_SAMP-1)[-frac]);
    }
    BASOP_SATURATE_WARNING_OFF
    /* Here, saturation might occur by intention */
    L_sum = L_shl(L_sum,1);
    BASOP_SATURATE_WARNING_ON
    return round_fx(L_sum);
}
