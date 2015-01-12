/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"

/*----------------------------------------------------------------------------------*
 *  findpulse()
 *
 *  Find first pitch pulse in a frame
 *----------------------------------------------------------------------------------*/
Word16 findpulse_fx(      /* o  : pulse position                */
    const Word16 L_frame, /* i  : length of the frame   */
    const Word16 res[],   /* i  : Residual signal     <12 bits  */
    const Word16 T0,      /* i  : Pitch estimation    Q0        */
    const Word16 enc,     /* i  : enc = 1 -> encoder side; enc = 0 -> decoder side */
    Word16 *sign    /* i/o: sign of the maximum */
)
{
    const Word16 *ptr;
    Word16 maxval;
    Word16 i, maxi;
    Word32 Ltmp;
    Word16 resf[L_FRAME16k]; /* Low pass filtered residual */

    IF (enc != DEC)
    {
        /*------------------------------------------------------------------------*
         * 1. Very simple LP filter
         *------------------------------------------------------------------------*/

        /* resf[0] = 0.50f * res[0] + 0.25f * res[1] */
        Ltmp = L_mult(res[0], 16384);
        resf[0] = mac_r(Ltmp, res[1], 8192);
        move16();
        FOR (i=1; i<L_frame-1; i++)
        {
            /* resf[i] = 0.25f * res[i-1] + 0.5f * res[i] + 0.25f * res[i+1] */
            Ltmp    = L_mult(8192, res[i-1]);
            Ltmp    = L_mac(Ltmp, 16384, res[i]);
            resf[i] = mac_r(Ltmp, 8192, res[i+1]);
            move16();
        }

        /* resf[L_frame-1] = 0.25f * res[L_frame-2] + 0.50f * res[L_frame-1] */
        Ltmp = L_mult(res[L_frame-2], 8192);
        resf[L_frame-1] = mac_r(Ltmp, 16384, res[L_frame-1]);
        move16();

        /*------------------------------------------------------------------------*
         * 2. Find "biggest" pitch pulse
         *------------------------------------------------------------------------*/

        ptr    = resf + L_frame - 1;
        move16();
        maxi   = 0;
        move16();

        FOR (i = 1; i < T0; i++)
        {
            Ltmp = L_mult0(ptr[-maxi], ptr[-maxi]);
            if (L_msu0(Ltmp, ptr[-i], ptr[-i]) < 0)
            {
                maxi = i;
                move16();
            }
        }
        /*
        *sign = 1;                 move16();
        test();
        if (ptr[-maxi] >= 0)
        {
            *sign = 0;             move16();
        }*/
        *sign = negate(shr(ptr[-maxi], 15));
        move16();
    }
    ELSE
    {
        /*-----------------------------------------------------------------*
         * 2. Find "biggest" pulse in the last pitch section according to the sign
         *-----------------------------------------------------------------*/

        maxval = 0;
        move16();
        maxi = 0;
        move16();

        IF (*sign == 0)
        {
            FOR (i = 0; i < T0; i++)
            {
                if (sub(res[i], maxval) >= 0)
                {
                    maxi   = add(i, 1);
                }
                maxval = s_max(res[i], maxval);
            }
        }
        ELSE
        {
            FOR (i = 0; i < T0; i++)
            {
                if (sub(res[i], maxval) <= 0)
                {
                    maxi   = add(i, 1);
                }
                maxval = s_min(res[i], maxval);
            }
        }
    }

    return maxi;
}
