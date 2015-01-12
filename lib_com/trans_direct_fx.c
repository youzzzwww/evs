/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "cnst_fx.h"    /* Common FX constants                    */
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Function prototypes                    */



void direct_transform_fx(
    const Word32 in32_fx[],
    Word32 out32_fx[],
    const Word16 is_transient,
    const Word16 L,
    Word16 *Q
)
{

    Word16 i, k;
    Word16 seg;
    Word16 segment_length, segment_length2, segment_length4;

    const Word16 *wh_fx;
    const Word16 *wl_fx;
    const Word32 *sh_fx;
    const Word32 *sl_fx2;
    Word32 *sl_fx;
    Word32 *iseg_fx;
    Word32 *oseg_fx;
    Word32 dctin32_fx[L_FRAME48k];
    Word32 in32_r16_fx[L_FRAME48k];

    const Word16 *win_fx;
    Word16 shift, Qmin = 31;
    Word32 L_tmp;
    Word16 Qs[NUM_TIME_SWITCHING_BLOCKS];


    segment_length = shr(L, 1);
    segment_length2 = shr(segment_length, 1);
    segment_length4 = shr(segment_length2, 1);

    IF (is_transient)
    {
        IF (sub(L, L_FRAME48k) == 0 )
        {
            win_fx = wscw16q15_fx;
        }
        ELSE IF (sub(L, L_FRAME32k) == 0)
        {
            win_fx = wscw16q15_32_fx;
        }
        ELSE IF (sub(L, L_FRAME8k) == 0)
        {
            win_fx = wscw16q15_8_fx;
        }
        ELSE
        {
            win_fx = wscw16q15_16_fx;
        }

        sh_fx = &in32_fx[L-1];
        add(0,0);
        sl_fx = &in32_r16_fx[L-1];
        add(0,0);
        FOR (i = 0; i < segment_length; i++)
        {
            in32_r16_fx[i] = (*sh_fx--);
            move32();
            (*sl_fx--) = in32_fx[i];
            move32();
        }

        iseg_fx = &in32_r16_fx[-segment_length4];
        add(0,0);
        oseg_fx = out32_fx;

        wh_fx = &win_fx[segment_length4];
        add(0,0);
        wl_fx = wh_fx - 1;

        shift = extract_l(L_mult0(3, segment_length4));
        sh_fx = &iseg_fx[shift];
        add(0,0);
        sl_fx2 = sh_fx - 1;


        FOR (i = 0; i < segment_length4; i++)
        {
            L_tmp = L_negate(Mult_32_16( (*sh_fx++), (*wh_fx++) )); /*Q+15-15=Q */
            dctin32_fx[i] = Madd_32_16(L_tmp, *sl_fx2--, *wl_fx--  );
            move32();/*Q  */
        }

        sl_fx2 = &iseg_fx[segment_length2 - 1];
        add(0,0);

        FOR (i = segment_length4; i < segment_length2; i++)
        {
            dctin32_fx[i] = L_negate(*sl_fx2--);
            move32();
        }

        Qs[0] = *Q;
        move16();
        edct_fx(dctin32_fx, oseg_fx, segment_length2, &Qs[0]);
        Qmin = s_min(Qs[0], Qmin);

        iseg_fx += segment_length2;
        add(0,0);
        oseg_fx += segment_length2;
        add(0,0);

        FOR (seg = 1 ; seg <  NUM_TIME_SWITCHING_BLOCKS-1; seg++)
        {
            wh_fx = &win_fx[segment_length4];
            add(0,0);
            wl_fx = wh_fx - 1;
            sh_fx = &iseg_fx[shift];
            add(0,0);
            sl_fx2 = sh_fx - 1;
            FOR (i = 0; i < segment_length4; i++)
            {
                L_tmp = L_negate(Mult_32_16( (*sh_fx++), (*wh_fx++) )); /*Q+15-15=Q */
                dctin32_fx[i] = Madd_32_16(L_tmp, *sl_fx2--, *wl_fx--  );
                move32(); /*Q  */
            }

            sh_fx = iseg_fx;
            sl_fx2 = &iseg_fx[segment_length2 - 1];
            add(0,0);
            wh_fx = &win_fx[segment_length2 - 1];
            add(0,0);
            wl_fx = win_fx ;

            FOR (i = segment_length4; i < segment_length2; i++)
            {
                L_tmp = Mult_32_16( (*sh_fx++), (*wh_fx--) ); /*Q+15-15=Q */
                dctin32_fx[i] = Madd_32_16(L_tmp, *sl_fx2--, *wl_fx++  );
                move32(); /*Q  */
            }

            Qs[seg] = *Q;
            move16();
            edct_fx(dctin32_fx, oseg_fx, segment_length2, &Qs[seg]);
            Qmin = s_min(Qs[seg], Qmin);

            iseg_fx += segment_length2;
            add(0,0);
            oseg_fx += segment_length2;
            add(0,0);
        }

        sh_fx = &iseg_fx[shift - 1];
        add(0,0);
        FOR (i = 0; i < segment_length4; i++)
        {
            dctin32_fx[i] = L_negate(*sh_fx--);
            move32();
        }


        sh_fx = iseg_fx;
        sl_fx2 = &iseg_fx[segment_length2 - 1];
        add(0,0);
        wh_fx = &win_fx[segment_length2 - 1];
        add(0,0);
        wl_fx = win_fx;

        FOR (i = segment_length4; i < segment_length2; i++)
        {
            L_tmp = Mult_32_16( (*sl_fx2--), (*wl_fx++) ); /*Q+15-15=Q */
            dctin32_fx[i] = Madd_32_16(L_tmp, *sh_fx++, *wh_fx--  );
            move32(); /*Q  */
        }
        Qs[NUM_TIME_SWITCHING_BLOCKS-1] = *Q;
        move16();
        edct_fx(dctin32_fx, oseg_fx, segment_length2, &Qs[NUM_TIME_SWITCHING_BLOCKS-1]);
        Qmin = s_min(Qs[NUM_TIME_SWITCHING_BLOCKS-1], Qmin);

        *Q = Qmin;
        move16();
        oseg_fx = out32_fx;
        FOR ( k=0; k<NUM_TIME_SWITCHING_BLOCKS; k++ )
        {
            shift = sub(Qs[k], *Q);
            FOR (i = 0; i<segment_length2; i++)
            {
                *oseg_fx = L_shr((*oseg_fx), shift);
                move32();
                oseg_fx++;
            }
        }
    }
    ELSE
    {
        edct_fx(in32_fx, out32_fx, L, Q);
    }

    return;
}
