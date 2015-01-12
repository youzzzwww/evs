/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function Prototypes                    */
#include "stl.h"

/*--------------------------------------------------------------------------*/
/*  Function  diffcod                                                       */
/*  ~~~~~~~~~~~~~~~~~                                                       */
/*                                                                          */
/*  Differential coding for indices of quantized norms                      */
/*--------------------------------------------------------------------------*/
/*  Word16   N          (i)   number of sub-vectors                         */
/*  Word16   *y         (i/o) indices of quantized norms                    */
/*  Word16   *difidx    (o)   differential code                             */
/*--------------------------------------------------------------------------*/

void diffcod_fx(
    const Word16 N,         /* (i)   number of sub-vectors      */
    Word16 *y,        /* (i/o) indices of quantized norms */
    Word16 *difidx    /* (o)   differential code          */
)
{
    Word16 i, k, r;

    FOR (i=N-1; i>0; i--)
    {
        r = sub(i, 1);
        k = sub(y[i], y[r]);
        if ( sub(k, -15) < 0 )
        {
            y[r] = add(y[i], 15);
            move16();
        }
    }

    FOR (i=1; i<N; i++)
    {
        r = sub(i, 1);
        k = sub(y[i], y[r]);
        IF ( sub(k, 16) > 0 )
        {
            k = 16;
            move16();
            y[i] = add(y[r], 16);
            move16();
        }
        difidx[r] = add(k, 15);
        move16();
    }

    return;
}




/*--------------------------------------------------------------------------
 * diffcod_lrmdct()
 *
 * Differential coding for indices of quantized norms
 *--------------------------------------------------------------------------*/

void diffcod_lrmdct_fx(
    const Word16 N,                  /* i  : number of sub-vectors       */
    const Word16 be_ref,             /* i  : band energy reference */
    Word16 *y,                 /* i/o: indices of quantized norms */
    Word16 *difidx,            /* o  : differential code */
    const Word16 is_transient        /* i  : transient flag  */
)
{
    Word16 i, m, r;
    Word16 k;
    Word16 thr_l,thr_h;

    IF(is_transient)
    {
        thr_l = -15;
        move16();
        thr_h = 16;
        move16();
    }
    ELSE
    {
        thr_l = -32;
        move16();
        thr_h = 31;
        move16();
    }

    difidx[0] = sub(y[0], be_ref);
    move16();
    IF( sub(difidx[0], thr_h) > 0)
    {
        difidx[0] = thr_h;
        move16();
        y[0] = add(be_ref, thr_h);
        move16();
    }

    IF( sub(difidx[0],thr_l) < 0 )
    {
        difidx[0] = thr_l;
        move16();
        y[0] = add(be_ref, thr_l);
        move16();
    }

    m = sub(N, 1);
    FOR (i=m; i>0; i--)
    {
        r = sub(i, 1);
        k = sub(y[i], y[r]);
        move16();
        if ( sub(k, thr_l) < 0 )
        {
            y[r] = sub(y[i], thr_l);
            move16();
        }
    }

    FOR (i=1; i<N; i++)
    {
        r = sub(i, 1);
        k = sub(y[i], y[r]);
        IF ( sub(k, thr_h) > 0 )
        {
            k = thr_h;
            move16();
            y[i] = add(y[r], thr_h);
            move16();
        }
        difidx[i] = k;
        move16();
    }

    return;
}

