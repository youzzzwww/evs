/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"

/*----------------------------------------------------------------------------------*
 * procedure updt_tar:
 *
 * Update the target vector for codebook search.
 *----------------------------------------------------------------------------------*/
void updt_tar_fx(
    const Word16 *x,    /* i  : old target (for pitch search)     */
    Word16 *x2,   /* o  : new target (for codebook search)  */
    const Word16 *y,    /* i  : filtered adaptive codebook vector */
    const Word16 gain,  /* i  : adaptive codebook gain            */
    const Word16 L      /* i  : subframe size                     */
)
{
    Word16 i;
    Word32 L_tmp;


    FOR (i = 0; i < L; i++)
    {
        /*x2[i] = x[i] - gain*y[i];*/
        L_tmp = L_mult(x[i], 16384);
        L_tmp = L_msu(L_tmp, y[i], gain);
        x2[i] = extract_h(L_shl(L_tmp, 1));
    }
}
/*----------------------------------------------------------------------------------*
 * procedure updt_tar:
 *
 * Update the target vector for codebook search.
 *----------------------------------------------------------------------------------*/
void updt_tar_HR_fx(
    const Word16 *x,          /* i  : old target (for pitch search)           */
    Word16 *x2,         /* o  : new target (for codebook search)        */
    const Word16 *y,          /* i  : filtered adaptive codebook vector       */
    const Word16 gain,        /* i  : adaptive codebook gain  Q2              */
    const Word16 Qx,          /* i  : Scaling factor to adapt output to input */
    const Word16 L            /* i  : subframe size                           */
)
{
    Word16 i;
    Word32 L_tmp, L_tmp1;


    FOR (i = 0; i < L; i++)
    {
        /*x2[i] = x[i] - gain*y[i];*/
        L_tmp = L_mult(x[i], 32767);
        L_tmp1 = L_shl(L_mult(y[i], gain), Qx);
        L_tmp = L_sub(L_tmp, L_tmp1);
        x2[i] = extract_h(L_tmp);
    }
}
