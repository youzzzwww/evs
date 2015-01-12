/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "options.h"     /* EV-VBR compilation switches            */
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"      /* Weighted mops computation related code */

/*-----------------------------------------------------------------------*
 * phase_dispersion:
 *
 * post-processing to enhance noise at low bit rate.
 *-----------------------------------------------------------------------*/

void phase_dispersion(
    const Word32 gain_code,  /* i  : gain of code  15Q16       */
    const Word16 gain_pit,   /* i  : gain of pitch   Q14       */
    Word16 code[],           /* i/o: code vector               */
    Word16 *code_exp,        /* i/o: exponent of code          */
    const Word16 mode,       /* i  : level, 0=hi, 1=lo, 2=off  */
    Word32 *prev_gain_code,  /* i/o: static memory 15Q16       */
    Word16 prev_gain_pit[],  /* i/o: static memory Q14, size=6 */
    Word16 *prev_state,      /* i/o: static memory          Q0 */
    Word16 L_subfr           /* i  : subframe length [40,64,80]*/
)
{
    Word16 i, j, state, scale2;
    Word32 x32[2*L_SUBFR];
    Word16 *code_real, *code_imag;
    const Word16 *h_real, *h_imag;



    move16();
    state = 2;

    if ( sub(gain_pit,FL2WORD16_SCALE(0.9f, 15-14)) < 0)
    {
        move16();
        state = 1;
    }
    if ( sub(gain_pit, FL2WORD16_SCALE(0.6f, 15-14)) < 0 )
    {
        move16();
        state = 0;
    }

    FOR (i=5; i>0; i--)
    {
        move16();
        prev_gain_pit[i] = prev_gain_pit[i-1];
    }
    move16();
    prev_gain_pit[0] = gain_pit;


    IF ( L_sub(gain_code, L_add(*prev_gain_code, L_shl(*prev_gain_code,1))) > 0 )
    {
        if (sub(state,2) < 0)
        {
            state = add(state, 1);
        }
    }
    ELSE
    {
        j=0;
        FOR (i=0; i<6; i++)
        {

            if ( L_sub(prev_gain_pit[i], FL2WORD16_SCALE(0.6f, 15-14)) < 0 )
            {
                j = add(j,1);
            }
        }

        if (sub(j,2) > 0)
        {
            move16();
            state = 0;
        }

        if ( sub(sub(state, *prev_state),1) > 0 )
        {
            state = sub(state,1);
        }
    }

    move32();
    move16();
    *prev_gain_code = gain_code;
    *prev_state = state;

    /*-----------------------------------------------------------------*
     * circular convolution
     *-----------------------------------------------------------------*/

    state = add(state,  mode);                        /* level of dispersion */
    j = *code_exp;
    move16();
    IF( sub(state,2) < 0 )
    {
        FOR(i=0; i<L_subfr; i++)
        {
            x32[i] = L_deposit_h(code[i]);
        }

        BASOP_rfft(x32, L_subfr, &j, -1);

        /* Normalize output data. */
        scale2 = getScaleFactor32(x32, L_subfr);
        FOR (i=0; i<L_subfr/2-1; i++)
        {
            code[i]           = round_fx(L_shl(x32[2*i+0], scale2));
            code[L_subfr-1-i] = round_fx(L_shl(x32[2*i+3], scale2));
        }

        code[L_subfr/2-1] = round_fx(L_shl(x32[L_subfr-2], scale2));
        code[L_subfr/2]   = round_fx(L_shl(x32[1], scale2));

        j = sub(j, scale2);

        h_real = low_H16k;
        move16();
        if( sub(L_subfr, 64) <= 0)
        {
            h_real = low_H;
            move16();
        }
        IF ( sub(state, 1) == 0)
        {
            h_real = mid_H16k;
            move16();
            if( sub(L_subfr, 64) <= 0)
            {
                h_real = mid_H;
                move16();
            }
        }
        h_imag = h_real + L_subfr - 1;
        move16();

        code_real  = &code[0];
        code_imag  = &code[L_subfr-1];

        x32[0] = L_mult(*code_real++, *h_real++);
        move32();
        FOR (i=1; i<L_subfr/2; i++)
        {
            x32[2*i]   = L_msu(L_mult(*code_real,   *h_real)  ,*code_imag,   *h_imag);
            move32();
            x32[2*i+1] = L_mac(L_mult(*code_real++, *h_imag--),*code_imag--, *h_real++);
            move32();
        }
        x32[1] = L_mult(*code_real++, *h_real++);
        move32();

        /* low_H and mid_H are in Q14 format, thus account that here. */
        j = add(j,1);

        BASOP_rfft(x32, L_subfr, &j, 1);
        scale2 = getScaleFactor32(x32, L_subfr);
        FOR (i=0; i<L_subfr; i++)
        {
            code[i] = round_fx(L_shl(x32[i], scale2));
        }
        j = sub(j, scale2);
    }

    /* Store exponent of code */
    move16();
    *code_exp = j;

}

