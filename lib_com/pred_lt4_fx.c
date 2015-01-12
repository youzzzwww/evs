/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "rom_com_fx.h"   /* Static table prototypes                */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Function  pred_lt4:                                               *
 *           ~~~~~~~~~                                               *
 *-------------------------------------------------------------------*
 * Compute the result of long term prediction with fractional       *
 * interpolation of resolution 1/4.                                  *
 *                                                                   *
 * On return exc[0..L_subfr-1] contains the interpolated signal      *
 *   (adaptive codebook excitation)                                  *
 *-------------------------------------------------------------------*/

void pred_lt4(
    const Word16 excI[],        /* in : excitation buffer       */
    Word16 excO[],        /* out: excitation buffer       */
    Word16 T0,            /* input : integer pitch lag    */
    Word16 frac,          /* input : fraction of lag      */
    Word16 L_subfr,       /* input : subframe size        */
    const Word16 *win,          /* i  : interpolation window    */
    const Word16 nb_coef,       /* i  : nb of filter coef       */
    const Word16 up_sample      /* i  : up_sample               */

)
{
    Word16   i, j;
    Word32   s;
    const Word16 *x0, *x1, *x2, *c1, *c2;
    x0 = &excI[-T0];


    frac = negate(frac);

    IF ( frac < 0 )
    {
        frac = add(frac,up_sample);
        x0--;
    }

    FOR (j=0; j<L_subfr; j++)
    {
        x1 = x0++;
        x2 = x1+1;
        c1 = (&win[frac]);
        c2 = (&win[up_sample-frac]);

        s = L_deposit_l(0);
        FOR(i=0; i<nb_coef; i++)
        {
            /*s += (*x1--) * (*c1) + (*x2++) * (*c2);*/
            s = L_mac0(s, (*x1--),(*c1));
            s = L_mac0(s, (*x2++),(*c2));

            c1+=up_sample;
            c2+=up_sample;
        }
#if (INTERP_EXP != -1)
        s = L_shl(s,INTERP_EXP+1);
#endif

        excO[j] = round_fx(s);
    }
    return;
}


/*======================================================================*/
/* FUNCTION : pred_lt4_tc_fx() */
/*-----------------------------------------------------------------------*/
/* PURPOSE :   * adapt. search of the second impulse in the same subframe (when appears) */
/* On return, exc[0..L_subfr-1] contains the interpolated signal         */
/*   (adaptive codebook excitation)                                      */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                                                    */
/* _ (Word16 []) exc  : excitation buffer             Q0                 */
/* _ (Word16) L_subfr : subframe size                 Q0                 */
/* _ (Word16 ) T0 : integer pitch lag                 Q0                 */
/* _ (Word16 ) frac : fraction of lag                 Q0                 */
/* _ (Word16 ) imp_pos : glottal impulse position     Q0                 */
/* _ (Word16 *) win : Interpolation window used       Q14                */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/* _ (Word16 []) exc  : output excitation buffer      Q0                 */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/* NONE																	 */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* NONE                                                                  */
/*=======================================================================*/
void pred_lt4_tc_fx(
    Word16 exc[],   /* i/o: excitation buffer        */
    const Word16 T0,      /* i  : integer pitch lag        */
    Word16 frac,    /* i:   fraction of lag          */
    const Word16 *win,    /* i  : interpolation window     */
    const Word16 imp_pos, /* i  : glottal impulse position */
    const Word16 i_subfr  /* i  : subframe index           */
)
{
    Word16 i, j,k,l;
    const Word16 *x0;
    Word16 excO[L_SUBFR+1];
    Word32 L_sum;
    Word16 excI[2*L_SUBFR];
    Copy( exc + sub(i_subfr, L_SUBFR), excI, shl(L_SUBFR,1) );

    test();
    IF (sub(add(T0, sub(imp_pos, L_IMPULSE2)), L_SUBFR) < 0 && sub(T0, L_SUBFR) < 0)
    {
        set16_fx(&excI[sub(L_SUBFR,T0)], 0, T0);
        set16_fx(excO, 0, L_SUBFR+1 );
        x0 = excI + sub(L_SUBFR, L_INTERPOL2-1);

        IF (frac > 0)
        {
            frac = sub(frac,UP_SAMP);
            x0--;
        }

        l = add(UP_SAMP-1, frac);
        FOR (j = T0; j < L_SUBFR+1; j++)
        {
            k = l;
            move16();
            L_sum = L_mult(x0[0], win[k]);
            FOR (i = 1; i < 2 * L_INTERPOL2; i++)
            {
                /*
                 * Here, additions with UP_SAMP are not counted
                 ki* because, the window could easily be modified
                 * so that the values needed are contiguous.
                 */
                k += UP_SAMP;
                L_sum = L_mac(L_sum, x0[i], win[k]);    /*Q1 */
            }
            L_sum = L_shl(L_sum, 1);  /*Q0h */

            excO[j] = round_fx(L_sum);

            x0++;
        }
        FOR (i = T0; i < L_SUBFR; i++)
        {
            exc[i+i_subfr] = add(exc[i+i_subfr], mult_r(PIT_SHARP_fx, excO[i]));
            move16();
        }
    }

    return;

}
