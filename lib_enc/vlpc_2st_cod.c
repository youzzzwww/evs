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


Word16 vlpc_2st_cod(	/* output: number of allocated bits				    */
    const Word16 *lsf,	/* input:  normalized vector to quantize (14Q1*1.28)*/
    Word16 *lsfq,	/* i/o:    i:1st stage   o:1st+2nd stage (14Q1*1.28)*/
    Word16 *indx,	/* output: index[] (4 bits per words)               */
    Word16  mode,	/* input:  0=abs, >0=rel                            */
    Word32 sr_core
)
{
    Word16  i, nbits;
    Word16  w[M], x[M];
    Word16  nq, xq[M];
    Word32  L_tmp;
    Word16  gap;



    /* 0 bit with true weighting: save 0.5 bit */
    lsf_weight_2st(lsf, w, 1);											/*w:0Q15*1.28*/

    FOR (i=1; i<M; i++)
    {
        x[i] = divide1616(shr(sub(lsf[i],lsfq[i]),2),w[i]);					/*5Q10*/	move16();
    }
    i = shr(sub(lsf[0],lsfq[0]),2);				                         	/*5Q10*/
    IF (w[0] != 0)
    {
        i = shl(mult_r(i,20972),1);                             			/*5Q10*1.28*/
        i = divide1616(i,w[0]);                                 			/*5Q10*/
    }
    x[0] = i;					                                          	/*5Q10*/	move16();

    L_tmp = L_mult(x[0],x[0]);                                      /*10Q21*/
    BASOP_SATURATE_WARNING_OFF /* Allow saturate because we only need to know if the result is smaller than 8.0f */
    FOR (i=1; i<M; i++)
    {
        L_tmp = L_mac(L_tmp,x[i],x[i]);					/*10Q21*/
    }
    BASOP_SATURATE_WARNING_ON

    IF (L_sub(L_tmp, FL2WORD32_SCALE(8.0f, 31-21)) < 0)	/*tmp40 < 8.0f */
    {
        indx[0] = 0;
        move16();
        indx[1] = 0;
        move16();

        /* the following assignments are excluded from the complexity count
           due to temporaray change of the type of the return value
           (nbits -> wops) */
        nbits = 6;						/* 2*(2+1) */	 		                        move16();
        test();
        IF ( (mode == 0) || (sub(mode,3) == 0) )
        {
            nbits = 10;					/* 2*(2+3) */					                move16();
        }
        ELSE IF (sub(mode,1) == 0)
        {
            nbits = 2;      			/* 2*1 */			                         	move16();
        }

        return nbits;
    }

    /* weighting from the 1st stage */
    lsf_weight_2st(lsfq, w, mode);

    /* find lsf and scale the residual */
    FOR (i=0; i<M; i++)
    {
        /* limit to range [-32767+1024+2048;32767-1024-2048] to guarantee enough headroom in quantizer */
        x[i] = s_max(s_min(divide1616(shr(sub(lsf[i],lsfq[i]),2),w[i]), 32767 - 1024 - 2048), -32767 + 1024 + 2048);	/*5Q10*/ move16();
    }

    /* quantize */
    AVQ_cod_lpc(x, xq, indx, 2);

    /* quantized lsf */
    FOR (i=0; i<M; i++)
    {
        /*lsfq[i] += (w[i]*(float)xq[i]);*/
        lsfq[i] = add(lsfq[i],shl(mult_r(w[i],xq[i]),2));				/*14Q1*1.28*/ 	move16();
    }

    /* total number of bits using entropic code to index the quantizer number */
    nbits = 0;
    move16();
    FOR (i=0; i<2; i++)
    {
        nq = indx[i];
        move16();

        test();
        IF ((mode == 0) || (sub(mode,3) == 0))				  /* abs, rel2 */
        {
            /*nbits += (2+(nq*4));*/
            nbits = add(nbits,add(2,shl(nq,2)));			/* 2 bits to specify Q2,Q3,Q4,ext; nbits += (2+(nq*4)); */
            IF (sub(nq,6) > 0)
            {
                /*nbits += nq-3;*/
                nbits = add(nbits,sub(nq,3));				/* unary code (Q7=1110, ...) */
            }
            ELSE IF (sub(nq,4) > 0)
            {
                /*nbits += nq-4;*/
                nbits = add(nbits,sub(nq,4));				/* Q5=0, Q6=10 */
            }
            ELSE IF (nq == 0)
            {
                /*nbits += 3;*/
                nbits = add(nbits,3);						/* Q0=110 */
            }
        }
        ELSE IF (sub(mode,1) == 0)							/* mid */
        {
            /*nbits += nq*5;*/
            nbits = add(nbits,extract_l(L_mult0(nq,5)));	/* unary code (Q0=0, Q1=10, ...) */
            if (nq == 0)
            {
                /*nbits += 1;*/
                nbits = add(nbits,1);
            }
        }
        ELSE
        {
            /*nbits += (2+(nq*4));*/                    	/* rel1 */
            nbits = add(nbits,add(2,shl(nq,2)));	       	/* 2 bits to specify Q2,Q3,Q4,ext */

            /*nbits += 1;*/
            nbits = add(nbits,1);		                 	/* Q0 = 0 */

            if (sub(nq,4) > 0)
            {
                /*nbits += nq-3-1;*/
                nbits = add(nbits,sub(nq,4));		        /* unary code (Q5=10, Q6=110, ...) */
            }
        }
    }/*FOR (i=0; i<2; i++)*/

    /* reorder */
    sort_fx(lsfq, 0, M-1);
    IF ( L_sub(sr_core,16000) == 0 )
    {
        gap = 102;
    }
    ELSE IF ( L_sub(sr_core,25600) == 0 )
    {
        gap = 64;
    }
    ELSE IF ( L_sub(sr_core,32000) == 0 )
    {
        gap = 51;
    }
    ELSE
    {
        gap = 34;
    }
    reorder_lsf_fx(lsfq, gap, M, INT_FS_FX);


    return nbits;
}
