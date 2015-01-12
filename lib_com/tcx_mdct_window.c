/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "options.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"
#include "prot_fx.h"
#include "stl.h"

void mdct_window_sine(const PWord16 **window, const Word16 n)
{
    *window = getSineWindowTable(n);
}


extern const Word16 window_48kHz_fx[];
extern const Word16 window_256kHz[];
extern const Word16 window_8_16_32kHz_fx[];

void mdct_window_aldo(
    Word16 *window1,
    PWord16 *window1_trunc,
    PWord16 *window2,
    Word16 n
)
{
    Word16 i, n0, n1, n2, d, tmp;
    const Word16 *p1, *p2;

    /* set table pointers and decimation factor */
    SWITCH (n)
    {
    case 320/2:
        p1 = window_48kHz_fx + 2;
        p2 = window_48kHz_fx + 1110 - 3;
        d = 6;
        BREAK;
    case 512/2:
        p1 = window_256kHz;
        p2 = window_256kHz + 592 - 1;
        d = 2;
        BREAK;
    case 640/2:
        p1 = window_48kHz_fx + 1;
        p2 = window_48kHz_fx + 1110 - 2;
        d = 3;
        BREAK;
    case 1024/2:
        p1 = window_256kHz;
        p2 = window_256kHz + 592 - 1;
        d = 1;
        BREAK;
    case 1280/2:
        p1 = window_48kHz_fx + 1;
        p2 = window_48kHz_fx + 1110 - 2;
        d = 3;
        BREAK;
    case 1920/2:
        p1 = window_48kHz_fx;
        p2 = window_48kHz_fx + 1110 - 1;
        d = 1;
        BREAK;
    default:
        assert(0);
        return;
    }

    /* set lengths */
    n0 = shr(imult1616(n, 9), 5);
    n1 = shr(imult1616(n, 23), 5); /* left slope length */
    n2 = shr(imult1616(n, 14), 5); /* right slope length */

    /* first part (long slope) */
    IF (sub(n, 1280/2) != 0)
    {
        FOR (i = 0; i < n0; i++)
        {
            *window1 = *p1;
            move16();
            window1++;
            p1 += d;
        }

        tmp = shr(n, 1);
        FOR ( ; i < tmp; i++)
        {
            window1_trunc->v.im = *p1;
            move16();
            window1_trunc++;
            p1 += d;
        }

        test();
        if (sub(n, 512/2) == 0 || sub(n, 320/2) == 0) p1++;

        FOR ( ; i < n1; i++)
        {
            window1_trunc--;
            window1_trunc->v.re = *p1;
            move16();
            p1 += d;
        }
    }
    ELSE
    {
        const Word16 *pi = window_8_16_32kHz_fx;

        FOR (i = 0; i < n0; i+=2)
        {
            *window1 = *p1;
            move16();
            window1++;
            p1 += d;

            *window1 = *pi;
            move16();
            window1++;
            pi++;
        }

        tmp = shr(n, 1);
        FOR ( ; i < tmp; i+=2)
        {
            window1_trunc->v.im = *p1;
            move16();
            window1_trunc++;
            p1 += d;

            window1_trunc->v.im = *pi;
            move16();
            window1_trunc++;
            pi++;
        }

        FOR ( ; i < n1; i+=2)
        {
            window1_trunc--;
            window1_trunc->v.re = *pi;
            move16();
            pi++;

            window1_trunc--;
            window1_trunc->v.re = *p1;
            move16();
            p1 += d;
        }
    }

    /* second part (short slope) */
    IF (sub(n, 1280/2) != 0)
    {
        tmp = shr(n2, 1);
        FOR (i = 0; i < tmp; i++)
        {
            window2->v.im = *p2;
            move16();
            window2++;
            p2 -= d;
        }

        test();
        if (sub(n, 512/2) == 0 || sub(n, 320/2) == 0) p2--;

        FOR ( ; i < n2; i++)
        {
            window2--;
            window2->v.re = *p2;
            move16();
            p2 -= d;
        }
    }
    ELSE
    {
        const Word16 *pi = window_8_16_32kHz_fx + 370 - 1;

        tmp = shr(n2, 1);
        FOR (i = 0; i < tmp; i+=2)
        {
            window2->v.im = *p2;
            move16();
            window2++;
            p2 -= d;

            window2->v.im = *pi;
            move16();
            window2++;
            pi--;
        }

        FOR ( ; i < n2; i+=2)
        {
            window2--;
            window2->v.re = *pi;
            move16();
            pi--;

            window2--;
            window2->v.re = *p2;
            move16();
            p2 -= d;
        }
    }
}

