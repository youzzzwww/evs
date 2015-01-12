/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "stl.h"


#define NB_TRACK  4
#define STEP      NB_TRACK

/*-------------------------------------------------------------------*
 * corr_xh_fx:
 *
 * Compute the correlation between the target signal and the impulse
 * response of the weighted synthesis filter.
 *
 *   y[i]=sum(j=i,l-1) x[j]*h[j-i], i=0,l-1
 *-------------------------------------------------------------------*/
void corr_xh_fx(
    const Word16 x[],  /* i  : target signal                                   */
    Word16 dn[], /* o  : correlation between x[] and h[]                 */
    const Word16 h[]   /* i  : impulse response (of weighted synthesis filter) */
)
{
    Word16 i, j, k;
    Word32 L_tmp, y32[L_SUBFR], L_maxloc, L_tot;

    /* first keep the result on 32 bits and find absolute maximum */
    L_tot = L_deposit_l(1);

    FOR (k = 0; k < NB_TRACK; k++)
    {
        L_maxloc = L_deposit_l(0);
        FOR (i = k; i < L_SUBFR; i += STEP)
        {
            L_tmp = L_mac(1L, x[i], h[0]); /* 1 -> to avoid null dn[] */
            FOR (j = i; j < L_SUBFR-1; j++)
            {
                L_tmp = L_mac(L_tmp, x[j+1], h[j+1 - i]);
            }

            y32[i] = L_tmp;
            move32();
            L_tmp = L_abs(L_tmp);
            L_maxloc = L_max(L_tmp, L_maxloc);
        }
        /* tot += 3*max / 8 */
        L_maxloc = L_shr(L_maxloc, 2);
        L_tot = L_add(L_tot, L_maxloc);           /* +max/4 */
        L_tot = L_add(L_tot, L_shr(L_maxloc, 1)); /* +max/8 */
    }

    /* Find the number of right shifts to do on y32[] so that    */
    /* 6.0 x sumation of max of dn[] in each track not saturate. */

    j = sub(norm_l(L_tot), 4); /* 4 -> 16 x tot */

    FOR (i = 0; i < L_SUBFR; i++)
    {
        dn[i] = round_fx(L_shl(y32[i], j));
    }
}
