/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "control.h"
#include "basop_util.h"


void lsf_weight_2st(
    const Word16 *lsfq,	/* input: quantized lsf coefficients (14Q1*1.28)        */
    Word16 *w,    /* output: weighting function (0Q15*1.28)               */
    const Word16 mode   /* input: operational mode                              */
)
{
    Word16 i;
    Word16 d[M+1], weight;

    /* compute lsf distance */
    d[0] = lsfq[0];
    move16();                       /*14Q1*1.28*/
    FOR (i=1; i<M; i++)
    {
        d[i] = sub(lsfq[i],lsfq[i-1]);
        move16();                       /*14Q1*1.28*/
    }
    d[M] = sub(FREQ_MAX,lsfq[M-1]);
    move16();                       /*14Q1*1.28*/

    /* weighting function */
    weight = W_MODE_ELSE;
    move16();                       /* rel2 */
    IF (mode == 0)
    {
        weight = W_MODE0;
        move16();                       /* abs */
    }
    ELSE IF (sub(mode,1) == 0)
    {
        weight = W_MODE1;
        move16();                       /* mid */
    }
    ELSE if (sub(mode,2) == 0)
    {
        weight = W_MODE2;
        move16();                       /* rel1 */
    }

    FOR (i=0; i<M; i++)
    {
        /* assert(d[i]>0); */

        /*w[i] = (weight * sqrt(d[i]*d[i+1])));*/

        w[i] = mult_r(weight,getSqrtWord32(L_shl(L_mult0(d[i],d[i+1]),6)));
        move16();
    }


    return;
}
