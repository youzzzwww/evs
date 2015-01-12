/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "basop_util.h"
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

static Word32 syn_kern_2(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = L_msu(L_tmp,  y[-1],  a[1]);
    return  L_msu(L_tmp,  y[-2],  a[2]);
}

static Word32 syn_kern_4(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_2(L_tmp, a, y);
    return  syn_kern_2(L_tmp, a+2, y-2);
}

static Word32 syn_kern_6(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_4(L_tmp, a, y);
    return  syn_kern_2(L_tmp, a+4, y-4);
}

static Word32 syn_kern_8(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_4(L_tmp, a, y);
    return  syn_kern_4(L_tmp, a+4, y-4);
}

static Word32 syn_kern_10(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_8(L_tmp, a, y);
    return  syn_kern_2(L_tmp, a+8, y-8);
}

Word32 syn_kern_16(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_8(L_tmp, a, y);
    return  syn_kern_8(L_tmp, a+8, y-8);
}

static Word32 syn_kern_24(Word32 L_tmp, const Word16 a[], const Word16 y[])
{
    L_tmp = syn_kern_16(L_tmp, a, y);
    return  syn_kern_8(L_tmp, a+16, y-16);
}

/*------------------------------------------------------------------*
 * Syn_filt_s_lc:
 *
 * perform the synthesis filtering 1/A(z).
 * Optimized Version when No Memory, Past is Set to 0
 *------------------------------------------------------------------*/
void syn_filt_s_lc_fx(
    const Word16 shift,     /* i  : scaling to apply        Q0   */
    const Word16 a[],       /* i  : LP filter coefficients  Q12  */
    const Word16 x[],       /* i  : input signal            Qx   */
    Word16 y[],       /* o  : output signal           Qx-s */
    const Word16 lg         /* i  : size of filtering       Q0   */
)
{
    Word16 i, j;
    Word32 L_tmp;
    Word16 a0;
    Word16 q;


    q = add( norm_s(a[0]), 1 );
    a0 = shr(a[0], shift); /* input / 2^shift */

    /*-----------------------------------------------------------------------*
     * Do the filtering
     *-----------------------------------------------------------------------*/
    FOR (i = 0; i < M; i++)
    {
        L_tmp = L_mult(*x++, a0);
        /* Stop at i to Avoid Mults with Zeros */
        FOR (j = 1; j <= i; j++)
        {
            L_tmp = L_msu(L_tmp, y[-j], a[j]);
        }

        L_tmp = L_shl(L_tmp, q);
        *y++ = round_fx(L_tmp);
    }

    FOR (; i < lg; i++)
    {
        L_tmp = syn_kern_16(L_mult(*x++, a0), a, y);
        L_tmp = L_shl(L_tmp, q);
        *y++ = round_fx(L_tmp);
    }
}

/*------------------------------------------------------------------*
 * Syn_filt_s:
 *
 * perform the synthesis filtering 1/A(z).
 *------------------------------------------------------------------*/
void Syn_filt_s(
    const Word16 shift,     /* i  : scaling to apply                          Q0   */
    const Word16 a[],       /* i  : LP filter coefficients                    Q12  */
    const Word16 m,         /* i  : order of LP filter                        Q0   */
    const Word16 x[],       /* i  : input signal                              Qx   */
    Word16 y[],       /* o  : output signal                             Qx-s */
    const Word16 lg,        /* i  : size of filtering                         Q0   */
    Word16 mem[],     /* i/o: memory associated with this filtering.    Qx-s */
    const Word16 update     /* i  : 0=no update, 1=update of memory.          Q0   */
)
{
    E_UTIL_synthesis(shift, a, x, y, lg, mem, update, m);
}


/*
 * E_UTIL_synthesis
 *
 * Parameters:
 *   shift       i  : scaling to apply for a[0]                 Q0
 *   a[]         i  : LP filter coefficients                    Qx
 *   x[]         i  : input signal                              Qx
 *   y[]         o  : output signal                             Qx-s
 *   lg          i  : size of filtering                         Q0
 *   mem[]       i/o: memory associated with this filtering.    Qx-s
 *   update      i  : 0=no update, 1=update of memory.          Q0
 *   m           i  : order of LP filter                        Q0
 *
 * Function:
 *    Perform the synthesis filtering 1/A(z).
 *    Memory size is always M.
 *
 * Returns:
 *    void
 */
void E_UTIL_synthesis(const Word16 shift, const Word16 a[], const Word16 x[], Word16 y[],
                      const Word16 lg, Word16 mem[], const Word16 update, const Word16 m
                     )
{
    Word16 i, j, a0;
    Word32 L_tmp;
    Word16 q;
    Word32 (*syn_kern)(Word32 L_tmp, const Word16 a[], const Word16 y[]) = NULL;

    if (sub(m, 6) == 0)
    {
        syn_kern = syn_kern_6;
    }
    if (sub(m, 10) == 0)
    {
        syn_kern = syn_kern_10;
    }
    if (sub(m, 16) == 0)
    {
        syn_kern = syn_kern_16;
    }
    if (sub(m, 24) == 0)
    {
        syn_kern = syn_kern_24;
    }
    assert(syn_kern != NULL);

    q = add( norm_s(a[0]), 1 );

    /*-----------------------------------------------------------------------*
     * Set Memory Pointer at End for Backward Access
     *-----------------------------------------------------------------------*/
    mem += m;                           /*move16();*/

    a0 = shr(a[0], shift); /* input / 2^shift */

    /*-----------------------------------------------------------------------*
     * Do the filtering
     *-----------------------------------------------------------------------*/
    /* Filtering Only from Input + Memory */
    L_tmp = syn_kern(L_mult(a0, *x++), a, mem);
    L_tmp = L_shl(L_tmp, q);
    *y++ = round_fx(L_tmp);

    /* Filtering from Input + Mix of Memory & Output Signal Past */
    FOR (i = 1; i < m; i++)
    {
        L_tmp = L_mult(a0, *x++);
        /* Process Output Signal Past */
        FOR (j = 1; j <= i; j++)
        {
            L_tmp = L_msu(L_tmp, a[j], y[-j]);
        }
        /* Process Memory */
        FOR (; j <= m; j++)
        {
            L_tmp = L_msu(L_tmp, a[j], mem[i-j]);
        }
        L_tmp = L_shl(L_tmp, q);
        *y++ = round_fx(L_tmp);
    }

    /* Filtering from Input + Output Signal Past */
    FOR (; i < lg; i++)
    {
        L_tmp = syn_kern(L_mult(a0, *x++), a, y);
        L_tmp = L_shl(L_tmp, q);
        *y++ = round_fx(L_tmp);
    }

    /*-----------------------------------------------------------------------*
     * Update memory if required
     *-----------------------------------------------------------------------*/
    IF (update != 0)
    {
        FOR (i = 0; i < m; i++)
        {
            *--mem = *--y;
            move16();
        }
    }

    return;
}



/*-------------------------------------------------------------------*
 * synth_mem_updt2()
 *
 * Update of synthesis filter memories in case of ACELP@12k8 <-> ACELP@16k switching
 *--------------------------------------------------------------------*/

void synth_mem_updt2(
    const Word16 L_frame,        /* i  : frame length                            */
    const Word16 last_L_frame,   /* i  : frame length                            */
    Word16 old_exc[],      /* i/o: excitation buffer                       */
    Word16 mem_syn_r[],    /* i/o: synthesis filter memory                 */
    Word16 mem_syn2[],     /* o  : synthesis filter memory for find_target */
    Word16 mem_syn[],      /* o  : synthesis filter memory for find_target */
    const Word16 dec             /* i  : flag for decoder indication             */
)
{
    Word16 mem_syn_r_size_old, mem_syn_r_size_new;

    /* Residual and update old_exc */
    IF( sub(dec, DEC) == 0 )
    {
        lerp( old_exc+L_EXC_MEM_DEC-(last_L_frame+last_L_frame/2), old_exc+L_EXC_MEM_DEC-(L_frame+L_frame/2), L_frame+L_frame/2, last_L_frame+last_L_frame/2 );
    }
    ELSE
    {
        lerp( old_exc+L_EXC_MEM-last_L_frame, old_exc+L_EXC_MEM-L_frame, L_frame, last_L_frame );
    }

    /*Resamp memory*/
    /*Size of LPC syn memory*/
    /* 1.25/20.0 = 1.0/16.0 -> shift 4 to the right. */
    mem_syn_r_size_old = shr(last_L_frame, 4);
    mem_syn_r_size_new = shr(L_frame, 4);

    lerp( mem_syn_r + L_SYN_MEM - mem_syn_r_size_old, mem_syn_r + L_SYN_MEM - mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );

    Copy( mem_syn_r+L_SYN_MEM-M, mem_syn2, M );

    IF( mem_syn != NULL )
    {
        Copy( mem_syn2, mem_syn, M );
    }

}


