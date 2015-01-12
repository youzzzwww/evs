/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"

/*========================================================================*/
/* FUNCTION : deemph_fx()												  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Deemphasis: filtering through 1/(1-mu z^-1)				  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) mu  : deemphasis factor   Q15								  */
/* _ (Word16) L	  : vector size         								  */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _ (Word16*) signal	  : signal              Q_syn2-1			      */
/* _ (Word16*) mem	  : memory (y[-1])    	    Q_syn2-1				  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void deemph_fx(
    Word16 *signal,   /* i/o: signal				Qx  */
    const Word16 mu,        /* i  : deemphasis factor   Q15 */
    const Word16 L,         /* i  : vector size         Q0  */
    Word16 *mem       /* i/o: memory (y[-1])      Qx  */
)
{
    Word16 i;
    Word32 L_tmp;

    L_tmp = L_deposit_h(signal[0]);
    L_tmp = L_mac(L_tmp, *mem, mu);
    signal[0] = round_fx(L_tmp);

    FOR (i = 1; i < L; i++)
    {
        L_tmp = L_deposit_h(signal[i]);
        L_tmp = L_mac(L_tmp, signal[i - 1], mu);
        signal[i] = round_fx(L_tmp);
    }

    *mem = signal[L - 1];
    move16();
}

/*-------------------------------------------------------------------*
 * Deeemph2 :
 *
 * Deemphasis: filtering through 1/(1-mu z^-1)
 * Output divided by 2
 *-------------------------------------------------------------------*/
void Deemph2(
    Word16 x[],   /* i/o: input signal overwritten by the output   Qx/Qx-1 */
    const Word16 mu,    /* i  : deemphasis factor                        Q15     */
    const Word16 L,     /* i  : vector size                              Q0      */
    Word16 *mem   /* i/o: memory (y[-1])                           Qx-1    */
)
{
    Word16 i;
    Word32 L_tmp;

    /* saturation can occur in L_mac() */

    L_tmp = L_mult(x[0], 16384);
    x[0] = mac_r(L_tmp, *mem, mu);
    move16();

    FOR (i = 1; i < L; i++)
    {
        L_tmp = L_mult(x[i], 16384);
        x[i] = mac_r(L_tmp, x[i - 1], mu);
        move16();
    }

    *mem = x[L - 1];
    move16();
}


/*
 * E_UTIL_deemph2
 *
 * Parameters:
 *    shift          I: scale output
 *    x              I/O: signal				  Qx/Qx-shift
 *    mu             I: deemphasis factor	  Qx
 *    L              I: vector size
 *    mem            I/O: memory (signal[-1])   Qx
 *
 * Function:
 *    Filtering through 1/(1-mu z^-1)
 *    Signal is divided by 2.
 *
 * Returns:
 *    void
 */
void E_UTIL_deemph2(Word16 shift, Word16 *x, const Word16 mu, const Word16 L, Word16 *mem)
{
    Word16 i;
    Word32 L_tmp;

    /* signal[0] = signal[0] + mu * (*mem); */
    L_tmp = L_deposit_h(*mem);
    IF(shift >= 0)
    {
        shift = shr(-32768, shift);
        FOR (i = 0; i < L; i++)
        {
            L_tmp = L_msu(Mpy_32_16_1(L_tmp, mu), x[i],shift);
            x[i] = round_fx(L_tmp);
        }

    }
    ELSE
    {
        FOR (i = 0; i < L; i++)
        {
            L_tmp = L_msu(Mpy_32_16_1(L_tmp, mu), shr(x[i],shift),-32768);
            x[i] = round_fx(L_tmp);
        }
    }

    *mem = x[L - 1];
    move16();

    return;
}
