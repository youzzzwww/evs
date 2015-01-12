/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*===================================================================*/
/* FUNCTION      :  Interpol_delay_fx ()                             */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Interpolate pitch lag for a subframe             */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) last_fx:  previous frame delay, Q0					 */
/*   _ (Word16) current_fx: current frame delay, Q0                  */
/*   _ (Word16)    SubNum : subframe number                          */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                                                                   */
/*   _ (Word16 []) out_fx  : 3 Intepolated delays, Q4            */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/
/* NOTE: this function uses a 5 entry table frac_fx (Q4 unsigned)    */
/*===================================================================*/

void Interpol_delay_fx(Word16 *out_fx, Word16 last_fx, Word16 current_fx,
                       Word16 SubNum, const Word16* frac_fx)
{
    Word16 i,temp;
    Word32 L_add1,L_add2;

    FOR (i=0; i<3; i++)
    {
        temp= sub(16,frac_fx[SubNum+i]);/* Q4 */
        L_add1 = L_shr(L_mult(last_fx,temp),1);/* Q4 */
        L_add2 = L_shr(L_mult(current_fx,frac_fx[SubNum+i]),1);/* Q4 */
        out_fx[i] = (Word16)L_add(L_add1,L_add2);
        move16();/* Q4 */

    }
    return;
}

/*-------------------------------------------------------------------*
 * deemph_lpc()
 *
 * De-emphasis of LP coefficients
 * convolve LPC with [1 -PREEMPH_FAC] to de-emphasise LPC
 *--------------------------------------------------------------------*/

void deemph_lpc_fx(
    Word16 *p_Aq_curr_fx,           /* i : LP coefficients current frame                       */
    Word16 *p_Aq_old_fx,            /* i : LP coefficients previous frame                      */
    Word16 *LPC_de_curr_fx,         /* o : De-emphasized LP coefficients current frame  in Q12 */
    Word16 *LPC_de_old_fx,           /* o : De-emphasized LP coefficients previous frame in Q12 */
    Word16 deemph_old

)
{
    Word16 k,temp;
    Word16 b_fx[M+2];/* Q12 */
    Word16 a_fx[2] = {-22282, 32767};/* Q15 {-PREEMPH_FAC,1.0} */

    b_fx[0] = 4096;
    move16();/* 1 in Q12 */
    FOR(k = 0; k < M; k++)
    {
        b_fx[k+1] = p_Aq_curr_fx[k];
        move16();/* Q12 */
    }
    b_fx[M+1] = 0;
    move16();

    FOR(k = 0; k <= M; k++)
    {
        /* LPC_de_curr[k] = a[0]*b[k] + a[1]*b[k+1]; */
        temp = mult(a_fx[0],b_fx[k]);/* Q12 */
        LPC_de_curr_fx[k] = add(temp,b_fx[k+1]);
        move16();/* Q12 */
    }

    IF ( sub( deemph_old, 1) == 0)
    {

        /* ignoring the 1st value which is 1.0 in this case */
        b_fx[0] = 4096;
        move16();/* 1 in Q12 */
        FOR(k = 0; k < M; k++)
        {
            b_fx[k+1] = p_Aq_old_fx[k+1];
            move16();
        }
        b_fx[M+1] = 0;
        move16();

        FOR(k = 0; k <= M; k++)
        {
            /* LPC_de_old[k] = a[0]*b[k] + a[1]*b[k+1]; */
            temp = mult(a_fx[0],b_fx[k]);/* Q12 */
            LPC_de_old_fx[k] = add(temp,b_fx[k+1]);
            move16();/* Q12 */
        }
    }


    return; /*  both outputs LPC_de_curr_fx and LPC_de_old_fx are in Q12 */
}


