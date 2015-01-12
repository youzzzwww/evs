/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "options.h"
#include "basop_util.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/* Returns: index of next coefficient */
Word16 get_next_coeff_mapped(
    Word16 ii[2],             /* i/o: coefficient indexes       */
    Word16 *pp,               /* o  : peak(1)/hole(0) indicator */
    Word16 *idx,              /* o  : index in unmapped domain  */
    CONTEXT_HM_CONFIG *hm_cfg /* i  : HM configuration          */
)
{
    Word16 p;

    p = s_and(sub(ii[1], hm_cfg->numPeakIndices), sub(hm_cfg->indexBuffer[ii[1]], hm_cfg->indexBuffer[ii[0]]));
    if (p > 0)
    {
        p = 0;
        move16();
    }
    if (p < 0)
    {
        p = 1;
        move16();
    }
    *pp = p;
    move16();
    *idx = ii[p];
    move16();
    ii[p] = add(ii[p], 1);
    move16();
    return hm_cfg->indexBuffer[*idx];
}

/* Returns: index of next coefficient */
Word16 get_next_coeff_unmapped(
    Word16 ii[2],             /* i/o: coefficient indexes       */
    Word16 *pp,               /* o  : peak(1)/hole(0) indicator */
    Word16 *idx,              /* o  : index in unmapped domain  */
    CONTEXT_HM_CONFIG *hm_cfg /* i  : HM configuration          */
)
{
    (void)pp;
    (void)hm_cfg;

    *idx = ii[0];
    move16();
    ii[0] = add(ii[0], 1);
    move16();
    return *idx;
}

Word16 update_mixed_context(Word16 ctx, Word16 a)
{
    Word32 t32;
    Word16 t=0; /* initialize just to avoid compiler warning */

    t32 = L_mac0(1-13, s_and(a, ~1), add(shr(a, 2), 1));
    if (t32 <= 0)
    {
        t = extract_l(t32);
    }
    a = shr(a, 3);
    if (t32 > 0)
    {
        t = s_min(a, 2);
    }
    return add(shl(s_and(ctx, 0xf), 4), add(t, 13));
}

