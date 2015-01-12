/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"
#include "stl.h"
#include "basop_mpy.h"
#include "basop_util.h"


/*-----------------------------------------------------------------*
 * Local constants
 *-----------------------------------------------------------------*/

#define MAX_LEN_LP   960     /* maximum length in samples of the LP analysis window */

/*---------------------------------------------------------------------*
 * autocorr()
 *
 * Compute autocorrelations of input signal
 *---------------------------------------------------------------------*/
void autocorr_fx(
    const Word16 x[],       /* i  : Input signal                    */
    const Word16 m,         /* i  : LPC order                   Q0  */
    Word16 r_h[],     /* o  : Autocorrelations  (msb)     Q15 */
    Word16 r_l[],     /* o  : Autocorrelations  (lsb)         */
    Word16 *Q_r,      /* o  : normalisation shift of r    Q0  */
    const Word16 len,       /* i  : Frame lenght                    */
    const Word16* wind,     /* i  : Window used                     */
    Word16 rev_flag,
    const Word16 sym_flag   /* i  : symmetric window flag           */
)
{
    Word16 i, j, norm, shift, y[MAX_LEN_LP];
    Word16 fact;
    Word32 L_sum, L_tmp;

    IF(sub(rev_flag,1) == 0)
    {
        /* Windowing of signal */
        FOR (i = 0; i < len; i++)
        {
            y[i] = mult_r(x[i], wind[len-i-1]);
            move16();
        }
    }
    ELSE IF( sub(sym_flag,1) == 0 )
    {
        /* symmetric window of even length */
        FOR( i=0; i<len/2; i++ )
        {
            y[i] = mult_r(x[i], wind[i]);
            move16();
        }
        FOR( ; i<len; i++ )
        {
            y[i] = mult_r(x[i], wind[len-i-1]);
            move16();
        }
    }
    ELSE  /* assymetric window */
    {
        FOR (i = 0; i < len; i++)
        {
            y[i] = mult_r(x[i], wind[i]);
            move16();
        }
    }


    /* calculate energy of signal */
    L_sum = L_deposit_h(16); /* sqrt(256), avoid overflow after rounding */
    FOR (i=0; i<len; i+=2)
    {
        L_tmp = L_mult0(y[i], y[i]);
        L_tmp = L_and(L_tmp, ~(128-1));
        L_tmp = L_mac0(L_tmp, y[i+1], y[i+1]);
        L_tmp = L_shr(L_tmp, 7);
        L_sum = L_add(L_sum, L_tmp);
    }

    /* scale signal to avoid overflow in autocorrelation */
    norm = norm_l(L_sum);
    shift = sub(4, shr(norm, 1));

    IF (shift > 0)
    {
        fact = lshr(-32768, shift);
        FOR (i = 0; i < len; i++)
        {
            y[i] = mult_r(y[i], fact);
            move16();
        }
    }
    ELSE
    {
        shift = 0;
        move16();
    }

    /* Compute and normalize r[0] */
    L_sum = L_mac(1, y[0], y[0]);
    FOR (i = 1; i < len; i++)
    {
        L_sum = L_mac(L_sum, y[i], y[i]);
    }
    norm = norm_l(L_sum);
    L_sum = L_shl(L_sum, norm);
    L_Extract(L_sum, &r_h[0], &r_l[0]);        /* Put in DPF format (see oper_32b) */

    /* Compute r[1] to r[m] */
    FOR (i = 1; i <= m; i++)
    {
        L_sum = L_mult(y[0],y[i]);
        FOR (j = 1; j < len - i; j++)
        {
            L_sum = L_mac(L_sum, y[j], y[j + i]);
        }

        L_sum = L_shl(L_sum, norm);
        L_Extract(L_sum, &r_h[i], &r_l[i]);    /* Put in DPF format (see oper_32b) */
    }

    *Q_r = sub(norm, shl(shift, 1));
    move16();
}

/*****************************************************************************
 *                                                                           *
 *   Function Name : Div_32_opt                                              *
 *                                                                           *
 *   Purpose :                                                               *
 *             Fractional integer division of two 32 bit numbers.            *
 *             L_num / L_denom.                                              *
 *             L_num and L_denom must be positive and L_num < L_denom.       *
 *             L_denom = denom_hi<<16 + denom_lo<<1                          *
 *             denom_hi is a normalize number.                               *
 *                                                                           *
 *   Inputs :                                                                *
 *                                                                           *
 *    L_num                                                                  *
 *             32 bit long signed integer (Word32) whose value falls in the  *
 *             range : 0x0000 0000 < L_num < L_denom                         *
 *                                                                           *
 *    L_denom = denom_hi<<16 + denom_lo<<1      (DPF)                        *
 *                                                                           *
 *       denom_hi                                                            *
 *             16 bit positive normalized integer whose value falls in the   *
 *             range : 0x4000 < hi < 0x7fff                                  *
 *       denom_lo                                                            *
 *             16 bit positive integer whose value falls in the              *
 *             range : 0 < lo < 0x7fff                                       *
 *                                                                           *
 *   Return Value :                                                          *
 *                                                                           *
 *    L_div                                                                  *
 *             32 bit long signed integer (Word32) whose value falls in the  *
 *             range : 0x0000 0000 <= L_div <= 0x7fff ffff.                  *
 *                                                                           *
 *  Algorithm:                                                               *
 *                                                                           *
 *  - find = 1/L_denom.                                                      *
 *      First approximation: approx = 1 / denom_hi                           *
 *      1/L_denom = approx * (2.0 - L_denom * approx )                       *
 *                                                                           *
 *  -  result = L_num * (1/L_denom)                                          *
 *****************************************************************************
*/
static
Word32 Div_32_opt (Word32 L_num, Word16 denom_hi, Word16 denom_lo)
{
    Word16 approx /*, hi, lo, n_hi , n_lo*/;
    Word32 L_32;

    /* First approximation: 1 / L_denom = 1/denom_hi */

    approx = div_s ((Word16) 0x3fff, denom_hi);

    /* 1/L_denom = approx * (2.0 - L_denom * approx) */

    L_32 = Msu_32_16 ((Word32) 0x7fffffffL, denom_hi, denom_lo, approx);

    L_32 = Mpy_32_16_1(L_32, approx);

    /* L_num * (1/L_denom) */

    L_32 = Mpy_32_32(L_num, L_32);

    L_32 = L_shl (L_32, 2);

    return (L_32);
}

/*
 * E_LPC_lev_dur
 *
 * Parameters:
 *    Rh          I:   Rh[m+1] Vector of autocorrelations (msb)
 *    Rl          I:   Rl[m+1] Vector of autocorrelations (lsb)
 *    A           O:   A[m]    LPC coefficients  (m = 16)        Qx (A[0] is always 1, so the format can be deduced in the caller with norm_s(A[0]))
 *    epsP        O:   error vector, msb
 *    order       I:   LPC order                                 Q0
 *
 * Function:
 *    Levinson-Durbin algorithm to compute
 *    the LPC parameters from the autocorrelations of speech.
 *
 * Returns:
 *    void
 */
Word16 E_LPC_lev_dur(const Word16 Rh[], const Word16 Rl[], Word16 A[],
                     Word32 epsP[], const Word16 order
                     ,Word16 *mem
                    )
{
    return(E_LPC_lev_dur_stab(Rh, Rl, A, epsP, order, mem, 32750)); /* 0.99945 in Q15 */
}

Word16 E_LPC_lev_dur_stab(const Word16 Rh[], const Word16 Rl[], Word16 A[],
                          Word32 epsP[], const Word16 order,
                          Word16 *mem, Word16 k_max
                         )
{
    Word16 i, j, k;
    Word16 hi, lo;
    Word16 Kh, Kl;                 /* reflection coefficient; hi and lo           */
    Word16 alp_h, alp_l, alp_exp;  /* Prediction gain; hi lo and exponent         */
    Word32 t0, t1, t2;             /* temporary variables                         */
    Word16 flag;
    Word16 Ah[TCXLTP_LTP_ORDER + 1], Al[TCXLTP_LTP_ORDER + 1];   /* LPC coef. in double prec. */


    BASOP_SATURATE_WARNING_OFF
    if(epsP!=NULL)
    {
        epsP[0] = L_Comp(Rh[0], Rl[0]);
        move32();
    }

    flag=0;
    move16();

    /* K = A[1] = -R[1] / R[0] */
    t1 = L_Comp(Rh[1], Rl[1]);             /* R[1] in Q31      */
    t2 = L_abs(t1);                        /* abs R[1]         */
    t0 = L_deposit_l(0);
    IF (Rh[0] != 0)
    {
        t0 = Div_32_opt(t2, Rh[0], Rl[0]);     /* R[1]/R[0] in Q31 */
        /* Cause a difference in MODE1 due to different implementation of div32*/
    }
    if (t1 > 0)
    {
        t0 = L_negate(t0);                 /* -R[1]/R[0]       */
    }
    Kl = L_Extract_lc(t0, &Kh);            /* K in DPF         */
    t0 = L_shr(t0, 4);                     /* A[1] in Q27      */
    L_Extract(t0, &Ah[1], &Al[1]);         /* A[1] in DPF      */

    /* Alpha = R[0] * (1-K**2) */
    t0 = Sqr_32(Kh, Kl);                   /* K*K      in Q31 */
    t0 = L_abs(t0);                        /* Some case <0 !! */
    t0 = L_sub((Word32) 0x7fffffffL, t0);  /* 1 - K*K  in Q31 */
    lo = L_Extract_lc(t0, &hi);            /* DPF format      */
    t0 = Mpy_32(Rh[0], Rl[0], hi, lo);     /* Alpha in Q31    */
    if(epsP!=NULL)
    {
        epsP[1] = t0;
        move32();
    }

    /* Normalize Alpha */
    alp_exp = norm_l(t0);
    t0 = L_shl(t0, alp_exp);
    alp_l = L_Extract_lc(t0, &alp_h);
    /* DPF format */

    /*--------------------------------------*
     * ITERATIONS  I=2 to m
     *--------------------------------------*/

    FOR (i = 2; i <= order; i++)
    {
        /* t0 = SUM(R[j]*A[i-j], j=1, i-1) + R[i] */
        t0 = L_deposit_l(0);
        FOR (j = 1; j < i; j++)
        {
            t0 = Mac_32(t0, Rh[j], Rl[j], Ah[i - j], Al[i - j]);
        }

        t0 = L_shl(t0, 4);                 /* result in Q27 -> convert to Q31 */
        /* No overflow possible            */

        /* Compose and add R[i] in Q3 */
        t0 = L_mac(t0, Rl[i], 1);
        t0 = L_msu(t0, Rh[i], -32768);

        /* K = -t0 / Alpha */
        t1 = L_abs(t0);
        t2 = L_deposit_l(0);
        IF (alp_h != 0)
        {
            t2 = Div_32_opt(t1, alp_h, alp_l); /* abs(t0)/Alpha                   */
            /* Cause a difference in MODE1 due to different implementation of div32*/
        }

        if (t0 > 0)
        {
            t2 = L_negate(t2);             /* K =-t0/Alpha                    */
        }
        t2 = L_shl(t2, alp_exp);           /* denormalize; compare to Alpha   */
        test();
        if ((mem!=NULL) && ((sub(abs_s(extract_h(t2)), k_max) > 0)))
        {
            flag=1;
            move16();/* Test for unstable filter. If unstable keep old A(z) */
        }
        Kl = L_Extract_lc(t2, &Kh);        /* K in DPF                        */

        /*------------------------------------------*
         *  Compute new LPC coeff. -> An[i]
         *  An[j]= A[j] + K*A[i-j]     , j=1 to i-1
         *  An[i]= K
         *------------------------------------------*/

        k = mult_r(i, 16384);
        FOR (j = 1; j < k; j++)
        {
            /* Do two Iterations Together to Allow Direct Update of Ah & Al */
            t0 = Mac_32(L_Comp(Ah[j], Al[j]), Kh, Kl, Ah[i-j], Al[i-j]);
            t1 = Mac_32(L_Comp(Ah[i-j], Al[i-j]), Kh, Kl, Ah[j], Al[j]);
            L_Extract(t0, &Ah[j], &Al[j]);
            L_Extract(t1, &Ah[i-j], &Al[i-j]);
        }
        IF (s_and(i, 1) == 0)
        {
            t0 = Mac_32(L_Comp(Ah[j], Al[j]), Kh, Kl, Ah[i-j], Al[i-j]);
            L_Extract(t0, &Ah[j], &Al[j]);
        }
        t2 = L_shr(t2, 4);                    /* t2 = K in Q31 ->convert to Q27 */
        L_Extract(t2, &Ah[i], &Al[i]);        /* An[i] in Q27                   */

        /* Alpha = Alpha * (1-K**2) */
        t1 = L_mult(Kh, Kh);                  /* K*K      in Q31 */
        t0 = L_mac(t1, mult(Kh, Kl), 2);
        t0 = L_abs(t0);                       /* Some case <0 !! */
        t0 = L_sub((Word32) 0x7fffffffL, t0); /* 1 - K*K  in Q31 */
        lo = L_Extract_lc(t0, &hi);           /* DPF format      */
        t0 = Mpy_32(alp_h, alp_l, hi, lo);    /* Alpha in Q31    */


        /* store denormalized alpha in epsP */
        t1 = L_shr(t0, alp_exp);
        if(epsP!=NULL)
        {
            epsP[i] = t1;
            move32();
        }

        /* Normalize Alpha */
        j = norm_l(t0);
        t0 = L_shl(t0, j);
        alp_l = L_Extract_lc(t0, &alp_h);/* DPF format */
        alp_exp = add(alp_exp, j);            /* Add normalization to alp_exp */
    }

    /* Adaptive scaling */
    t1 = L_deposit_l(0);
    FOR (i = 1; i <= order; i++)
    {
        t0 = L_Comp(Ah[i], Al[i]);
        t1 = L_max( t1, L_abs(t0) );
    }
    k  = s_min( norm_l( t1 ), 3 );
    A[0] = shl( 2048, k );
    move16();
    FOR (i = 1; i <= order; i++)
    {
        t0 = L_Comp(Ah[i], Al[i]);
        A[i] = round_fx(L_shl(t0, k));
    }

    BASOP_SATURATE_WARNING_ON
    IF (mem != NULL)
    {
        /* Enforce stable LPC filter  - parcorr[0] and parcorr[1] are not LPC coeffiecients */
        IF(flag)
        {
            Copy(mem, A, order + 1);
        }
        ELSE /* If stable LPC filter, store into memories */
        {
            Copy(A, mem, order + 1);
        }
    }


    return(flag);
}

/*
 * E_LPC_a_add_tilt
 *
 * Parameters:
 *    a              I: LP filter coefficients (m+1 coeffs)
 *    ap             O: modified LP filter coefficients (m+2 coeffs)
 *    gamma          I: tilt factor
 *    m              I: order of LP filter
 *
 * Function:
 *    Modified LP filter by adding 1st order pre-premphasis, Ap(z)=A(z).(1-gamma.z^(-1))
 *
 * Returns:
 *    void
 */
void E_LPC_a_add_tilt(const Word16 *a, Word16 *ap, Word16 gamma, Word16 m)
{
    Word16 i;
    Word32 Amax, Atmp[M+2];
    Word16 shift;




    Amax = L_mult( 16384, a[0] );
    FOR (i = 1; i <= m; i++)
    {
        Atmp[i] = L_sub( L_mult(16384, a[i]), L_mult0(gamma, a[i-1]) );
        move32();
        Amax = L_max( Amax, L_abs( Atmp[i] ) );
    }
    Atmp[m+1] = L_negate( L_mult0(gamma, a[m]) );
    move32();
    Amax = L_max( Amax, L_abs( Atmp[m+1] ) );
    shift = norm_l( Amax );
    ap[0] = shl( a[0], sub(shift,1) );
    move16();
    FOR (i = 1; i <= m; i++)
    {
        ap[i] = round_fx( L_shl( Atmp[i], shift ) );
    }
    ap[m+1] = round_fx( L_shl( Atmp[m+1], shift ) );

}

void E_LPC_int_lpc_tcx(const Word16 lsp_old[],    /* input : LSPs from past frame              Q15   */
                       const Word16 lsp_new[],    /* input : LSPs from present frame           Q15   */
                       Word16 a[]       /* output: interpolated LP coefficients      Q12   */
                      )
{
    Word16 i, lsp[M];



    FOR (i = 0; i < M; i++)
    {
        /*lsp[i] = lsp_old[i]*0.125f + lsp_new[i]*0.875f;*/
        lsp[i] = round_fx(L_mac(L_mult(lsp_old[i],4096),lsp_new[i],28672));
    }
    E_LPC_f_lsp_a_conversion(lsp, a, M);


    return;
}

static void lsp_reorder(
    Word16 *lsp,               /* (I/O): LSP vector (acos() domain)  Q13*1.2732 */
    Word16 min_dist,           /* (I): minimum required distance     Q13*1.2732 */
    Word16 lpcorder            /* (I): LPC order                     Q0         */
)
{
    Word16 i;
    Word16 lsp_min, lsp_max;


    /* Verify the LSF ordering and minimum GAP */
    lsp_min = min_dist;
    move16();
    FOR (i=0; i<lpcorder; ++i)
    {
        lsp[i] = s_max(lsp[i], lsp_min);
        move16();
        lsp_min = add(lsp[i], min_dist);
    }

    /* Reverify the LSF ordering and minimum GAP in the reverse order (security) */
    lsp_max = sub(32767, min_dist);

    /* If danger of unstable filter in case of resonance in HF */
    lpcorder = sub(lpcorder, 1);
    IF (sub(lsp[lpcorder], lsp_max) > 0)
    {
        /* Reverify the minimum LSF gap in the reverse sense */
        FOR (i = lpcorder; i>=0; --i)
        {
            lsp[i] = s_min(lsp[i], lsp_max);
            move16();
            lsp_max = sub(lsp[i], min_dist);
        }
    }

}

/* Approximate unweighting */
Word16 E_LPC_lsp_unweight(
    /* const */ Word16 lsp_w[], /* (I): weighted xSP             Q15 */
    Word16 lsp_uw[],            /* (O): unweighted xSP           Q15 */
    Word16 lsf_uw[],            /* (O): unweighted LSF       Q1*1.28 */
    Word16 inv_gamma,           /* (I): inverse weighting factor Q14 */
    Word16 lpcorder             /* (I): prediction order         Q0  */
)
{
    Word16 lsp_w_orig[M], lsp_w_diff[M], mean, step; /* Q13*1.2732 */
    const lsp_unw_triplet *unw_coeffs = NULL;
    Word16 i;

    step = 0;       /* to avoid compilation warnings */


    assert(lpcorder == 16);

    /* Table selection */
    IF (sub(inv_gamma, GAMMA16k_INV) == 0)
    {
        unw_coeffs = p16_gamma0_94to1;
        move16();
    }
    ELSE IF (sub(inv_gamma, GAMMA1_INV) == 0)
    {
        unw_coeffs = p16_gamma0_92to1;
        move16();
    }
    ELSE
    {
        assert(0);
    }

    /* step = M_PI/(float)(lpcorder+1); */
    step = 1927;
    move16();
    mean = 0;
    move16();

    /* Apply acos() and get mean removed version */
    FOR (i=0; i<lpcorder; ++i)
    {
        mean = add(mean, step);
        lsp_w_orig[i] = shl(xsp_to_xsf(lsp_w[i]), 1);
        move16();
        lsp_w_diff[i] = sub(lsp_w_orig[i], mean);
        move16();
    }

    /* Approximate unweighting by 3-tap FIR */
    lsp_uw[0] = add(lsp_w_orig[0], round_fx(L_shl(L_mac0(L_mult0(unw_coeffs[0][1], lsp_w_diff[0]), unw_coeffs[0][2], lsp_w_diff[1]), 2)));
    FOR (i=1; i<lpcorder-1; ++i)
    {
        lsp_uw[i] = add(lsp_w_orig[i], round_fx(L_shl(L_mac0(L_mac0(L_mult0(unw_coeffs[i][0], lsp_w_diff[i-1]), unw_coeffs[i][1], lsp_w_diff[i]), unw_coeffs[i][2], lsp_w_diff[i+1]), 2)));
        move16();
    }
    lsp_uw[i] = add(lsp_w_orig[i], round_fx(L_shl(L_mac0(L_mult0(unw_coeffs[i][0], lsp_w_diff[i-1]), unw_coeffs[i][1], lsp_w_diff[i]), 2)));
    move16();

    /* Reorder */
    lsp_reorder(lsp_uw, 256, lpcorder);

    /* Convert to LSF, apply cos() */
    FOR (i=0; i<lpcorder; ++i)
    {
        lsf_uw[i] = shr_r(lsp_uw[i], 1);
        move16();
        lsp_uw[i] = xsf_to_xsp(lsf_uw[i]);
        move16();
    }

    return 0;
}


/*
 * E_LPC_schur
 *
 * Parameters:
 *    R         I:   Rh[M+1] Vector of autocorrelations (msb)
 *    reflCoeff O:   rc[M]   Reflection coefficients.          Q15
 *    epsP      O:   error vector
 *
 * Function:
 *    Schur algorithm to compute
 *    the LPC parameters from the autocorrelations of speech.
 *
 * Returns:
 *    void
 */
Word32 E_LPC_schur(Word32 r[], Word16 reflCoeff[], Word32 epsP[], const Word16 m)
{
    Word16   i, j, temp16, mM1, mMi, s;
    Word32   g0[M], *g1, tmp32;
    const    Word32 min_epsP = 1; /* > 0.01f*2^27/2^30 */
    Word32   tmp_epsP;



    mM1 = sub(m,1);

    s = getScaleFactor32(r,add(m,1));
    IF (s!=0)
    {
        scale_sig32(r,add(m,1),s); /* scale in-place */
    }

    g1 = r;
    Copy32(r+1,g0,m);

    /* compute g0[0]/g1[0], where g0[0] < g1[0] */
    temp16 = negate(divide3232(g0[0], g1[0]));
    reflCoeff[0] = temp16;
    move16();
    move32();
    epsP[0] = r[0];


    FOR (i=0; i<mM1; i++)
    {
        /* g1[0] = g0[0]*temp16 + g1[0]; */
        tmp32 = Mpy_32_16_1(g0[0], temp16);
        g1[0] = L_add(g1[0],tmp32);
        move32();

        mMi = sub(m,i);
        FOR (j=1; j<mMi; j++)
        {
            /* g0[j-1] = g0[j]        + g1[j]*temp16;
            g1[j]   = g0[j]*temp16 + g1[j];       */
            g0[j-1] = L_add( g0[j], Mpy_32_16_1(g1[j], temp16));
            move32();
            g1[j] =   L_add( g1[j], Mpy_32_16_1(g0[j], temp16));
            move32();
        }
        temp16 = negate(divide3232( g0[0], g1[0]));
        reflCoeff[i+1] = temp16;
        move16();

        /* Prediction errors */
        tmp_epsP = L_shr(g1[0],s);
        if (tmp_epsP <= 0 )
        {
            tmp_epsP = L_add(0,min_epsP);
        }
        epsP[i+1] = tmp_epsP;
        move32();
    }

    /* epsP[i+1] = g0[0]*temp16 + g1[0]; */
    tmp_epsP = L_add(g1[0], Mpy_32_16_1( g0[0], temp16 ));
    tmp_epsP = L_shr(tmp_epsP,s);
    if (tmp_epsP <= 0)
    {
        tmp_epsP = L_add(min_epsP, 0);
    }
    epsP[i+1] = tmp_epsP;
    move32();

    /* prediction gain = divide3232(L_shr(epsP[0], PRED_GAIN_E), g1[0]); */



    return g1[0];
}


extern const PWord16 *w_a[7];
extern const PWord16 w19N[127];
extern const PWord16 w18N[127];
extern void BASOP_getTables(const PWord16 **ptwiddle, const PWord16 **sin_twiddle, Word16 *psin_step, Word16 length);
static
void spec2isf(
    Word16/*double*/      spec_r[],   /* input spectrum real part (only left half + one zero)*/
    Word16/*double*/      spec_i[],   /* input spectrum imag part (only left half+right halt with zeros)*/
    Word16/*short*/         speclen,    /* length of spectrum (only left half)*/
    Word16 /*double*/      lsf[],      /* locations of LSFs (buffer must be sufficiently long) */ /*15Q16*/
    const Word16 /*double*/ old_lsf[]   /* locations of LSFs (buffer must be sufficiently long) */ /*15Q16*/
)
{

    /*spec_r[] needs a 0 in the end!*/
    Word16 s;
    Word16 tmp, i;
    Word16 specix,lsfix;

    move16();
    move16();
    specix = lsfix = 0;
    move16();
    s = spec_r[specix++];

    WHILE (sub(specix , speclen) <0 && sub(lsfix,15) <= 0)
    {

        /*check for next zero crossing*/
        /*for (; s*spec_r[specix] >= 0; specix++);*/
        WHILE (mult(s,spec_r[specix]) >= 0) specix = add(specix, 1);

        tmp = divide1616(spec_r[specix-1], sub(spec_r[specix-1],spec_r[specix] )  );
        move16();
        /*lsf[lsfix] = L_add(L_deposit_h(sub(specix,1)) ,  L_shl(L_deposit_l(tmp),1));*/ /*Q16*/
        lsf[lsfix] = add(shl(sub(specix,1),7) ,  shr((tmp),8)); /*7Q8*/

        lsfix++;

        /*check for the next zero crossing*/
        /*for (; s*spec_i[specix] >= 0; specix++);*/

        WHILE (mult(s,spec_i[specix]) >= 0) specix = add(specix, 1);

        tmp = divide1616(spec_i[specix-1], sub(spec_i[specix-1],spec_i[specix]) );
        move16();
        /*lsf[lsfix] = L_add(L_deposit_h(sub(specix,1)) ,  L_shl(L_deposit_l(tmp),1));*/ /*Q16*/
        lsf[lsfix] = add(shl(sub(specix,1),7) ,  shr((tmp),8)); /*7Q8*/

        lsfix++;

        spec_r[speclen] = s;
        move16();
        spec_i[speclen] = s;
        move16();

        s = negate(s);
    }

    IF (sub(lsfix,16) < 0)
    {
        FOR(i=0; i<16; i++)
        {
            lsf[i] = old_lsf[i];
            move16();
        }
    }

    return;
}

void E_LPC_a_lsf_isf_conversion(Word16 *lpcCoeffs,   Word16 *lsf, const Word16 *old_lsf, Word16 lpcOrder, Word8  lpcRep)
{
    Word32 RealFFT[128];
    Word32 ImagFFT[128];
    Word16 RealOut[130];
    Word16 ImagOut[130];
    Word32 *ptrReal;
    Word32 *ptrImag;
    Word16 n, i, j, step, scale;
    const PWord16 *ptwiddle, *pwn17, *pwn17i;
    PWord16 *pwn15, *pwn15i, tmpw15;
    Word16 N = 256;
    Word16 s[4];
    Word32 L_tmp, L_tmp1, L_tmp3;
    Word16 lpc[19];


    /* half length FFT */
    scale = add(norm_s(lpcCoeffs[0]),1)+5;
    move16();

    /*s = [sum(a) ((-1).^(1:length(a)))*a];*/
    L_tmp = L_deposit_h(0);
    FOR(j=0; j<=lpcOrder; j++)
    {
        L_tmp = L_mac(L_tmp, lpcCoeffs[j], 0x800);
    }
    /*s[1] = round_fx(L_tmp);    move16();*/

    L_tmp1 = L_deposit_h(0);
    FOR(j=0; j<lpcOrder/2; j++)
    {
        L_tmp1 = L_msu(L_tmp1, lpcCoeffs[2*j], 0x800);
        L_tmp1 = L_mac(L_tmp1, lpcCoeffs[2*j+1], 0x800);
    }
    L_tmp1 = L_msu(L_tmp1, lpcCoeffs[2*j], 0x800);
    /*s[2] = round_fx(L_tmp1);    move16();*/


    L_tmp3 = L_add(L_tmp1,L_tmp);
    IF(L_tmp3 != 0)
    {
        s[1] = BASOP_Util_Divide3232_Scale(L_sub(L_tmp1,L_tmp),L_tmp3, &step);
        move16();
        /*s[1] = BASOP_Util_Divide1616_Scale(sub(s[2],s[1]),add(s[2],s[1]), &step);   move16();*/
        BASOP_SATURATE_WARNING_OFF
        s[0] = negate(shr(-32768,step+1));
        move16();
        s[2] = negate(shr(-32768,step+1));
        move16();
        BASOP_SATURATE_WARNING_ON
    }
    ELSE
    {
        s[1] = 16384/4;
        move16();
        s[0] = 0;
        move16();
        s[2] = 0;
        move16();
    }
    lpc[0] = mult_r(lpcCoeffs[0], s[0]);
    move16();
    L_tmp = L_mult(s[0], lpcCoeffs[1]);
    lpc[1] = mac_r(L_tmp, lpcCoeffs[1-1], s[1]);
    move16();

    FOR (n = 2; n < 17; n++)
    {
        L_tmp = L_mult(s[0], lpcCoeffs[n]);
        L_tmp = L_mac(L_tmp, lpcCoeffs[n - 1], s[1]);
        lpc[n] = mac_r(L_tmp, lpcCoeffs[n - 2], s[2]);
        move16();
    }
    lpc[18] = mult_r(lpcCoeffs[16], s[0]);
    move16();
    L_tmp = L_mult(s[0], lpcCoeffs[15]);
    lpc[17] = mac_r(L_tmp, lpcCoeffs[16], s[1]);
    move16();

    scale = getScaleFactor16(lpc, 19)- 1 - SCALEFACTOR16;

    ptrReal = RealFFT;
    ptrImag = ImagFFT;

    FOR(j=0; j<9; j++)
    {
        ptrReal[j*8] = L_shl(L_deposit_h(lpc[2*j]),scale);
        ptrImag[j*8] = L_shl(L_deposit_h(lpc[2*j+1]),scale);
    }
    ptrReal[j*8] = L_shl(L_deposit_h(lpc[2*j]),scale);
    ptrImag[j*8] = 0;
    move16();
    j++;

    FOR(; j<16; j++)
    {
        ptrReal[j*8] = L_deposit_h(0);
        ptrImag[j*8] = L_deposit_h(0);
    }

    fft16(ptrReal, ptrImag, 8, 0);

    ptrReal++;
    ptrImag++;

    FOR(i=1; i<8; i++)
    {
        ptwiddle = w_a[i-1];

        ptrReal[0] = L_shl(L_deposit_h(lpc[0]),scale);
        ptrImag[0] = L_shl(L_deposit_h(lpc[1]),scale);

        FOR(j=1; j<9; j++)
        {
            ptrReal[j*8] = L_shl(L_msu(L_mult(lpc[2*j],ptwiddle->v.re), lpc[2*j+1],ptwiddle->v.im),scale);
            move32();
            ptrImag[j*8] = L_shl(L_mac(L_mult(lpc[2*j+1],ptwiddle->v.re), lpc[2*j],ptwiddle->v.im),scale);
            move32();
            ptwiddle++;
        }

        ptrReal[j*8] = L_shl(L_mac(0,lpc[2*j],ptwiddle->v.re),scale);
        move32();
        ptrImag[j*8] = L_shl(L_mac(0, lpc[2*j],ptwiddle->v.im),scale);
        move32();
        ptwiddle++;
        j++;
        FOR(; j<16; j++)
        {
            ptrReal[j*8] = L_deposit_h(0);
            ptrImag[j*8] = L_deposit_h(0);
            ptwiddle++;
        }

        fft16(ptrReal, ptrImag, 8, 0);

        ptrReal++;
        ptrImag++;

    }

    /* pre-twiddle */
    BASOP_getTables(NULL, &ptwiddle, &step, 128);
    IF (lpcRep != 0)
    {
        pwn17i = &w19N[126];
        pwn17 = w19N;
    }
    ELSE
    {
        pwn17i = &w18N[126];
        pwn17 = w18N;
    }

    pwn15 = &tmpw15;
    pwn15i = &tmpw15;

    RealOut[0] = round_fx(2*L_add(RealFFT[0], ImagFFT[0]));
    ImagOut[0]  = 0;
    move16();

    RealOut[128] = 0;
    move16();
    ImagOut[128] = round_fx(L_sub(L_add(RealFFT[0],RealFFT[0]), L_add(ImagFFT[0], ImagFFT[0])));

    ptwiddle += 8;
    FOR(i=1; i<=N/2/4; i++)
    {
        Word16 ReAr = extract_h(L_add(RealFFT[i],RealFFT[N/2-i]));
        Word16 ReBr = extract_h(L_sub(RealFFT[N/2-i], RealFFT[i]));
        Word16 ImAr = extract_h(L_sub(ImagFFT[i], ImagFFT[N/2-i]));
        Word16 ImBr = extract_h(L_add(ImagFFT[i], ImagFFT[N/2-i]));
        BASOP_SATURATE_WARNING_OFF
        tmpw15.v.re = mac_r(L_mult(ptwiddle->v.re, pwn17->v.re), ptwiddle->v.im, pwn17->v.im);
        tmpw15.v.im = msu_r(L_mult(ptwiddle->v.re, pwn17->v.im),  ptwiddle->v.im, pwn17->v.re);
        BASOP_SATURATE_WARNING_ON
        RealOut[i] = mac_r(L_msu(L_msu(L_mult(ReAr, pwn17->v.re),ImAr, pwn17->v.im), ReBr, pwn15->v.im), ImBr, pwn15->v.re);
        move16();
        ImagOut[i] = mac_r(L_mac(L_mac(L_mult(ReAr, pwn17->v.im), ImAr, pwn17->v.re), ReBr, pwn15->v.re), ImBr, pwn15->v.im);
        move16();
        BASOP_SATURATE_WARNING_OFF
        tmpw15.v.re = msu_r(L_mult(ptwiddle->v.im, pwn17i->v.im), ptwiddle->v.re, pwn17i->v.re);
        tmpw15.v.im = mac_r(L_mult(ptwiddle->v.re, pwn17i->v.im),  ptwiddle->v.im, pwn17i->v.re);
        BASOP_SATURATE_WARNING_ON
        RealOut[N/2-i] = msu_r(L_mac(L_mac(L_mult(ReAr, pwn17i->v.re), ImAr, pwn17i->v.im), ImBr, pwn15i->v.re),  ReBr, pwn15i->v.im);
        move16();
        ImagOut[N/2-i] = msu_r(L_msu(L_msu(L_mult(ReAr, pwn17i->v.im), ImAr, pwn17i->v.re), ReBr, pwn15i->v.re), ImBr, pwn15i->v.im);
        move16();

        ptwiddle += 8;
        pwn17++;
        pwn17i--;
    }

    ptwiddle -= 16;
    /*change real with imaginary for ptwiddle*/
    FOR(; i<N/2/2; i++)
    {
        Word16 ReAr = extract_h(L_add(RealFFT[i],RealFFT[N/2-i]));
        Word16 ReBr = extract_h(L_sub(RealFFT[N/2-i], RealFFT[i]));
        Word16 ImAr = extract_h(L_sub(ImagFFT[i], ImagFFT[N/2-i]));
        Word16 ImBr = extract_h(L_add(ImagFFT[i], ImagFFT[N/2-i]));
        BASOP_SATURATE_WARNING_OFF
        tmpw15.v.re = mac_r(L_mult(ptwiddle->v.im, pwn17->v.re), ptwiddle->v.re, pwn17->v.im);
        tmpw15.v.im = msu_r(L_mult(ptwiddle->v.im, pwn17->v.im),  ptwiddle->v.re, pwn17->v.re);
        BASOP_SATURATE_WARNING_ON
        RealOut[i] = mac_r(L_msu(L_msu(L_mult(ReAr, pwn17->v.re),ImAr, pwn17->v.im), ReBr, pwn15->v.im), ImBr, pwn15->v.re);
        move16();
        ImagOut[i] = mac_r(L_mac(L_mac(L_mult(ReAr, pwn17->v.im), ImAr, pwn17->v.re), ReBr, pwn15->v.re), ImBr, pwn15->v.im);
        move16();
        BASOP_SATURATE_WARNING_OFF
        tmpw15.v.re = msu_r(L_mult(ptwiddle->v.re, pwn17i->v.im), ptwiddle->v.im, pwn17i->v.re);
        tmpw15.v.im = mac_r(L_mult(ptwiddle->v.im, pwn17i->v.im),  ptwiddle->v.re, pwn17i->v.re);
        BASOP_SATURATE_WARNING_ON
        RealOut[N/2-i] = msu_r(L_mac(L_mac(L_mult(ReAr, pwn17i->v.re), ImAr, pwn17i->v.im), ImBr, pwn15i->v.re), ReBr, pwn15i->v.im);
        move16();
        ImagOut[N/2-i] = msu_r(L_msu(L_msu(L_mult(ReAr, pwn17i->v.im), ImAr, pwn17i->v.re), ReBr, pwn15i->v.re), ImBr, pwn15i->v.im);
        move16();

        ptwiddle -= 8;
        pwn17++;
        pwn17i--;
    }
    ptwiddle += 0;
    {
        Word16 ReAr = extract_h(L_add(RealFFT[i],RealFFT[N/2-i]));
        Word16 ReBr = extract_h(L_sub(RealFFT[N/2-i], RealFFT[i]));
        Word16 ImAr = extract_h(L_sub(ImagFFT[i], ImagFFT[N/2-i]));
        Word16 ImBr = extract_h((L_negate(L_add(ImagFFT[i], ImagFFT[N/2-i]))));
        BASOP_SATURATE_WARNING_OFF
        tmpw15.v.re = mac_r(L_mult(ptwiddle->v.im, pwn17->v.re), ptwiddle->v.re, pwn17->v.im);
        tmpw15.v.im = msu_r(L_mult(ptwiddle->v.im, pwn17->v.im), ptwiddle->v.re, pwn17->v.re);
        BASOP_SATURATE_WARNING_ON
        RealOut[i] = msu_r(L_msu(L_msu(L_mult(ReAr, pwn17->v.re), ImAr, pwn17->v.im), ReBr, pwn15->v.im), ImBr, pwn15->v.re);
        move16();
        ImagOut[i] = msu_r(L_mac(L_mac(L_mult(ReAr, pwn17->v.im), ImAr, pwn17->v.re), ReBr, pwn15->v.re), ImBr, pwn15->v.im);
        move16();
    }

    spec2isf(RealOut, ImagOut, 128, lsf, old_lsf);
    IF (lpcRep == 0)
    {
        lsf[lpcOrder - 1] = shl(lpcCoeffs[lpcOrder], add(norm_s(lpcCoeffs[0]),1));
        move16(); /* From Qx to Q15 with saturation */
        lsf[lpcOrder - 1] = xsp_to_xsf(lsf[lpcOrder - 1]);
        move16();
        lsf[lpcOrder - 1] = shr(lsf[lpcOrder - 1], 1);
        move16();
    }

}

