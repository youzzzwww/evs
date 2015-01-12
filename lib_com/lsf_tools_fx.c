/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h" /* Static table prototypes                */



#define NC_MAX 8
#define GUESS_TBL_SZ 256

#define depack_4_values(cbp, val0, val1, val2, val3) \
  val0 =  shr((cbp)[0], 4); \
  val1 =  shr((cbp)[1], 4); \
  val2 =  shr((cbp)[2], 4); \
  val3 =  add(add(shr(lshl((cbp)[2],12),4),lshr(lshl((cbp)[1],12),8)),s_and((cbp)[0],0xF));

/*-------------------------------------------------------------------*
    * Local constants
    *-------------------------------------------------------------------*/

#define NC        ( M/2)
#define NC16k     ( M16k/2)

#define SPC       0.0234952f
#define SPC_plus  SPC * 1.001f
#define ALPHA_SQ  ( ( 0.5f / PI2) * ( 0.5f / PI2))

/*-------------------------------------------------------------------*
    * Local functions
    *-------------------------------------------------------------------*/
Word16 root_search_fx( Word16 low, Word16 high, Word32 *v_low, Word32 *coef,Word16 order);
Word32 calc_weight( Word16 delta1,Word16 delta2,Word16 *n1 );
Word32 polynomial_eval_fx( Word16 f, Word32 *coef,Word16 order);
void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m);
void E_LPC_lsp_lsf_conversion(const Word16 lsp[],Word16 lsf[],const Word16 m);
Word16 E_LPC_f_lsp_pol_get(const Word16 lsp[], Word32 f[],  const Word16 n, const Word16 past_Ovf, const Word16 isMODE1);

static Word16 chebyshev(Word16 x, Word32 *f, Word16 n, Word16 shift)
{

    Word16 cheb;
    Word32 t0, b1, b2;

    cheb = norm_s(x);
    if (cheb)
    {
        x = shl(x,1);
    }
    t0 = Mpy_32_16_1(*f++, x);      /* t0 = x*b2                  */
    if (!cheb)
        t0 = L_shl(t0, 1);          /* t0 = 2*x*b2                */
    b1 = L_add(t0, *f++);         /* b1 = 2*x*b2 + f[1]         */

    /* i = 2 */
    t0 = Mpy_32_16_1(b1, x);        /* t0 = x*b1                  */
    if (!cheb)
        t0 = L_shl(t0, 1);          /* t0 = 2*x*b1                */
    b2 = L_add(t0, *f++);         /* b0 = 2*x*b1 - b2 + f[i]    */

    /* i = 3 */
    t0 = Mpy_32_16_1(b2, x);        /* t0 = x*b1                  */
    if (!cheb)
        t0 = L_shl(t0, 1);          /* t0 = 2*x*b1                */
    t0 = L_sub(t0, b1);           /* t0 = 2*x*b1 - b2           */
    b1 = L_add(t0, *f++);         /* b0 = 2*x*b1 - b2 + f[i]    */

    /* i = 4 */
    t0 = Mpy_32_16_1(b1, x);        /* t0 = x*b1                  */
    if (!cheb)
        t0 = L_shl(t0, 1);          /* t0 = 2*x*b1                */
    t0 = L_sub(t0, b2);           /* t0 = 2*x*b1 - b2           */

    /* If the LP order is greater than 10 */
    IF(sub(n, 5) > 0)
    {
        b2 = L_add(t0, *f++);         /* b0 = 2*x*b1 - b2 + f[i]    */
        /* i = 5 */
        t0 = Mpy_32_16_1(b2, x);        /* t0 = x*b1                  */
        if (!cheb)
            t0 = L_shl(t0, 1);          /* t0 = 2*x*b1                */
        t0 = L_sub(t0, b1);           /* t0 = 2*x*b1 - b2           */
        b1 = L_add(t0, *f++);         /* b0 = 2*x*b1 - b2 + f[i]    */

        /* i = 6 */
        t0 = Mpy_32_16_1(b1, x);        /* t0 = x*b1                  */
        if (!cheb)
            t0 = L_shl(t0, 1);          /* t0 = 2*x*b1                */
        t0 = L_sub(t0, b2);           /* t0 = 2*x*b1 - b2           */
    }
    /* IF (sub(n,8) == 0) */
    IF (n == 8)
    {
        b2 = L_add(t0, *f++);         /* b0 = 2*x*b1 - b2 + f[i]    */
        /* i = 7 */
        t0 = Mpy_32_16_1(b2, x);      /* t0 = x*b1                  */
        if (!cheb)
            t0 = L_shl(t0, 1);        /* t0 = 2*x*b1                */
        t0 = L_sub(t0, b1);         /* t0 = 2*x*b1 - b2           */
        /*b1 = L_add(b2,0);*/
    }
    ELSE
    b2 = L_add(b1,0);

    t0/*b2*/ = L_add(t0, *f++);   /* b0 = 2*x*b1 - b2 + f[i]    */

    t0 = Mpy_32_16_1(t0/*b2*/, x);/* t0 = x*b1                  */
    if (cheb)
        t0 = L_shr(t0,1);
    t0 = L_sub(t0, /*b1*/b2);     /* t0 = x*b1 - b2             */
    t0 = L_add(t0, *f++);         /* t0 = x*b1 - b2 + 0.5*f[n]  */


    BASOP_SATURATE_WARNING_OFF
    t0 = L_shl(t0, shift);            /* Qx to Q30 with saturation */
    cheb = round_fx(t0);              /* Result in Q14              */
    cheb = s_max(-32767,cheb);        /* to avoid saturation        */
    BASOP_SATURATE_WARNING_ON
    return (cheb);
}

void E_LPC_a_isp_conversion(const Word16 a[], Word16 isp[], const Word16 old_isp[], const Word16 m)
{
    Word16 i, nf, ip, order, nc;
    Word16 xlow, ylow, xhigh, yhigh;
    Word16 x, y, tmp, exp;
    Word32 f[2][NC_MAX+1];
    Word32 t0, t1;
    Word16 scale=1024;


    /*-------------------------------------------------------------*
     * find the sum and diff polynomials F1(z) and F2(z)
     *      F1(z) = [A(z) + z^M A(z^-1)]
     *      F2(z) = [A(z) - z^M A(z^-1)]/(1-z^-2)
     *
     * for (i=0; i<NC; i++)
     * {
     *   f1[i] = a[i] + a[m-i];
     *   f2[i] = a[i] - a[m-i];
     * }
     * f1[NC] = 2.0*a[NC];
     *
     * for (i=2; i<NC; i++)            Divide by (1-z^-2)
     *   f2[i] += f2[i-2];
     *-------------------------------------------------------------*/
    nc = shr(m, 1);

    scale = shl( 256, norm_s(a[0]) );

    FOR (i = 0; i < nc; i++)
    {
        t0 = L_mult(a[i],scale);                                 /*Q23*/
        t1 = L_mult(a[m-i],scale);                               /*Q23*/
        f[0][i] = L_add(t0,t1);
        move32();   /*Q23*/
        f[1][i] = L_sub(t0,t1);
        move32();   /*Q23*/
    }
    f[0][nc] = L_mult(a[nc],scale);
    move32(); /*Q23-1*/

    FOR (i = 2; i < nc; i++)
    {
        f[1][i] = L_add(f[1][i], f[1][i - 2]);
        move32();
    }
    /* f[1][nc] = L_shr(f[1][nc],1);                 move32(); */
    f[1][nc-1] = L_shr(f[1][nc-1],1);
    move32();

    f[0][2] = L_sub(f[0][2], f[0][0]);
    move32();
    f[1][2] = L_sub(f[1][2], f[1][0]);
    move32();


    /*----------------------------------------------------------------*
     * Find the ISPs (roots of F1(z) and F2(z) ) using the
     * Chebyshev polynomial evaluation.
     * The roots of F1(z) and F2(z) are alternatively searched.
     * We start by finding the first root of F1(z) then we switch_fx
     * to F2(z) then back to F1(z) and so on until all roots are found.
     *
     *  - Evaluate Chebyshev pol. at grid points and check for sign change.
     *  - If sign change track the root by subdividing the interval
     *    2 times and ckecking sign change.
     *----------------------------------------------------------------*/

    nf = 0;
    move16();  /* number of found frequencies */
    ip = 0;
    move16();  /* indicator for f1 or f2      */

    order = nc;
    move16();

    xlow = Grid[0];
    move16();
    ylow = chebyshev(xlow, f[ip], order, 7);

    FOR (i = 1; i <= GRID100_POINTS; i++)
    {
        xhigh = xlow;
        move16();
        yhigh = ylow;
        move16();
        xlow = Grid[i];
        move16();
        ylow = chebyshev(xlow, f[ip], order, 7);

        IF (L_mult(ylow, yhigh) <= 0)
        {
            t0 = L_mult(xhigh, 0x4000);
            /* divide 2 times the interval */
            x = mac_r(t0, xlow, 0x4000); /* xmid = (xlow + xhigh)/2 */
            y = chebyshev(x, f[ip], order, 7);

            IF (L_mult(ylow, y) <= 0)
            {
                yhigh = y;
                move16();
                xhigh = x;
                move16();
                y = ylow;
                move16();
                x = xlow;
                move16();
                /* 'xhigh' has changed, update 't0' */
                t0 = L_mult(xhigh, 0x4000);
            }
            xlow = mac_r(t0, x, 0x4000);  /* xlow = (x + xhigh)/2 */
            ylow = chebyshev(xlow, f[ip], order, 7);

            IF (L_mult(y, ylow) <= 0)
            {
                yhigh = ylow;
                move16();
                xhigh = xlow;
                move16();
                ylow = y;
                move16();
                xlow = x;
                move16();
            }

            /*--------------------------------------------------------*
             * Linear interpolation
             * xint = xlow - ylow*(xhigh-xlow)/(yhigh-ylow)
             *--------------------------------------------------------*/

            y = sub(yhigh, ylow);

            IF (y != 0)
            {
                x = sub(xhigh, xlow);

                tmp = abs_s(y);
                exp = norm_s(tmp);
                if (exp)
                    tmp = shl(tmp, exp);
                tmp = div_s((Word16) 16383, tmp);
                t0 = L_mult(x, tmp);
                t0 = L_shr(t0, sub(20, exp));
                tmp = extract_l(t0);         /* y = (xhigh-xlow)/(yhigh-ylow) in Q11 */

                /* Restore Sign */
                if (y < 0)
                    tmp = negate(tmp);

                t0 = L_mult(ylow, tmp);           /* result in Q26 */
                t0 = L_shr(t0, 11);             /* result in Q15 */
                xlow = sub(xlow, extract_l(t0));/* xint = xlow - ylow*y */
            }

            isp[nf++] = xlow;
            move16();

            IF (sub(nf,(m-1)) >= 0)
            {
                BREAK;
            }

            ip = s_xor(ip, 1);
            order = sub(nc, ip);
            ylow = chebyshev(xlow, f[ip], order, 7);
        }
    }

    /*----------------------------------------------------------------*
     * Check if m-1 roots found, if not use the ISPs from previous frame
     *----------------------------------------------------------------*/

    isp[m - 1] = shl(a[m], add(norm_s(a[0]),1));
    move16(); /* From Qx to Q15 with saturation */

    IF (s_min(sub(nf, m - 1), sub(8192, abs_s(a[m]))) < 0)
    {
        FOR (i = 0; i < m; i++)
        {
            isp[i] = old_isp[i];
            move16();
        }
    }


}

/*
* E_LPC_f_isp_a_conversion
*
* Parameters:
*    isp            I: Immittance spectral pairs      Q15
*    a              O: Predictor coefficients (order = m)  Q12
*    m              I: order of LP filter
*
* Function:
*    Convert ISPs to predictor coefficients a[]
*
* Returns:
*    void
*/
void E_LPC_f_isp_a_conversion(const Word16 *isp, Word16 *a, const Word16 m)
{
    Word16 i, j;
    Word32 f1[NC_MAX+1], f2[NC_MAX+1]; /* Q23 */
    Word16 nc, q;
    Word32 t0, tmax, t0p, t0n;



    /*-----------------------------------------------------*
    *  Find the polynomials F1(z) and F2(z)               *
    *-----------------------------------------------------*/

    nc = shr(m, 1);

    E_LPC_f_lsp_pol_get(&isp[0], f1, nc, 0, 0);
    E_LPC_f_lsp_pol_get(&isp[1], f2, sub(nc, 1), 0, 0);
    /*-----------------------------------------------------*
     *  Multiply F2(z) by (1 - z^-2)                       *
     *-----------------------------------------------------*/

    FOR (i = sub(nc, 1); i > 1; i--)
    {
        /* f2[i] -= f2[i-2]; */
        f2[i] = L_sub(f2[i], f2[i - 2]);
        move32();
    }

    /*----------------------------------------------------------*
     *  Scale F1(z) by (1+isp[m-1])  and  F2(z) by (1-isp[m-1]) *
     *----------------------------------------------------------*/

    FOR (i = 0; i < nc; i++)
    {
        /* f1[i] *= (1.0 + isp[m-1]); */
        f1[i] = Madd_32_16(f1[i], f1[i], isp[m - 1]);
        move32();

        /* f2[i] *= (1.0 - isp[m-1]); */
        f2[i] = Msub_32_16(f2[i], f2[i], isp[m - 1]);
        move32();
    }

    /*-----------------------------------------------------*
     *  A(z) = (F1(z)+F2(z))/2                             *
     *  F1(z) is symmetric and F2(z) is antisymmetric      *
     *-----------------------------------------------------*/

    /* Maximum LPC */
    tmax = L_deposit_l(1);
    FOR (i = 1; i < nc; i++)
    {
        t0 = L_add(f1[i], f2[i]);
        tmax = L_max( tmax, L_abs(t0) );
        t0 = L_sub(f1[i], f2[i]);
        tmax = L_max( tmax, L_abs(t0) );
    }
    q = s_min( norm_l(tmax), 6 );

    DO
    {

        /* a[0] = 1.0 */
        a[0] = shl(256,q);
        move16();
        j = sub(m, 1);
        FOR (i = 1;  i < nc; i++)
        {
            /* a[i] = 0.5*(f1[i] + f2[i]) */
            t0 = L_add(f1[i], f2[i]);          /* f1[i] + f2[i]             */
            t0 = L_shl(t0, q);
            a[i] = round_fx(t0);             /* from Q23 to Q12 and * 0.5 */

            /* a[j] = 0.5*(f1[i] - f2[i]) */
            t0 = L_sub(f1[i], f2[i]);          /* f1[i] - f2[i]             */
            t0 = L_shl(t0, q);
            a[j] = round_fx(t0);             /* from Q23 to Q12 and * 0.5 */

            j = sub(j,1);
        }

        /* a[NC] = 0.5*f1[NC]*(1.0 + isp[m-1]) */
        t0 = Madd_32_16(f1[nc], f1[nc], isp[m - 1]);

        BASOP_SATURATE_WARNING_OFF /*overflow handling in loop expression*/
        t0 = L_shl(t0, q);
        t0n = L_sub(t0 , 0x7FFFFFFF); /*check for positive overflow*/
        t0p = L_sub(t0,  0x80000000); /*check for negative overflow*/
        BASOP_SATURATE_WARNING_ON

        q = sub(q,1); /*decrease q in case of overflow*/
    } WHILE(t0n == 0 || t0p == 0); /*in case of overflow, recalculate coefficients*/

    a[nc] = round_fx(t0);                /* from Q23 to Q12 and * 0.5 */

    /* a[m] = isp[m-1] */
    t0 = L_mult(a[0], isp[m - 1]);       /* from Q15 to Q12          */
    a[m] = round_fx(t0);


    return;
}

/*===================================================================*/
/* FUNCTION : lpc2lsp_fx () */
/*-------------------------------------------------------------------*/
/* PURPOSE : Convert LPC coefficients to LSP coefficients */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* */
/* _ (Word32 []) a : LPC coefficients, Q27 */
/* _ (Word16 []) old_freq: Previous frame LSP coefficients, Q15 */
/* _ (Word16 []) order: LPC order */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word16 []) freq: LSP coefficients, Q15 */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ (Word16) flag: 1 means all 10 LSPs are found, 0 otherwise */
/*===================================================================*/
Word16 lpc2lsp_fx(
    Word32* a,
    Word16* freq,
    Word16* old_freq,
    Word16 order
)
{
    Word16 i;
    Word16 rt, low, high, prev_rt, rc;
    Word32 p[11], q[11]; /*  Q26 */
    Word32 Ltemp, v_low;
    Word32 Lacc;
    Word16 tfreq[21];

    /* First construct the P,Q polynomial */
    /* p[0] = q[0] = 1 */
    /* p[i] = -lpcCoeff[i] - lpcCoeff[11-i] - p[i-1] ( 1<=i<=5)*/
    /* q[i] = -lpcCoeff[i] + lpcCoeff[11-i] + q[i-1] ( 1<=i<=5)*/
    Ltemp = L_deposit_h( 0x400 ); /*  Ltemp is 1.0 in Q26 */

    p[0] = Ltemp;
    move32();
    q[0] = Ltemp;
    move32();

    FOR ( i = 1; i < ( order / 2 ) + 1; i++ )
    {
        Lacc = a[order - i];
        move32();/* Q27 */
        Lacc = L_sub( Lacc, a[i - 1] ); /*  Lacc=-lpcCoeff[i-1] + lpcCoeff[order-i]//Q27 */
        q[i] = L_add( L_shr( Lacc, 1 ), q[i - 1] );
        move32();/* Q26 */

        Lacc = L_add( Lacc, L_shl( a[i - 1], 1 ) );/*  Lacc=lpcCoeff[i-1] + lpcCoeff[order-i]//Q27 */

        p[i] = L_sub( L_negate( L_shr( Lacc, 1 ) ), p[i - 1] );
        move32();/* Q26 */
    }

    /* Search roots of the P and Q polynomials */

    v_low = polynomial_eval_fx( 0, p, order ); /* Q25 */
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    low = 0;
    high = 8;
    prev_rt = 0;
    rc = 0; /*  root counter */
    FOR ( i = 0; i < 32; i++ )
    {
        rt = root_search_fx( low, high, &v_low, p, order );
        low = high;
        move16();
        high = add( high, 8 );

        IF( sub( rt, prev_rt ) >= 0 )
        {
            tfreq[rc] = rt;
            move16();
            rc = add( rc, 2 );
        }
        prev_rt = add( rt, 6 );
    } /*  End for P roots */

    tfreq[rc] = 0x3f80;
    move16(); /*  Set a high enough value as fake root for Q search */

    IF ( sub( rc, order ) < 0 )
    {
        /*  lost P root */
        /* copy from previous LSP and return */
        FOR ( i = 0; i < order; i++ )
        {
            move16();
            freq[i] = old_freq[i];
        }
        return ( 0 );
    }
    ELSE
    {
        /*  Search for Q roots between P roots */
        v_low = L_deposit_h( 0x800 ); /* Init a positive value for v_low */
        rc = 1;
        move16();
        FOR ( i = 0; i < order / 2; i++ )
        {
            low = shr( tfreq[rc - 1], 6 );
            high = add( shr( tfreq[rc + 1], 6 ), 1 );
            rt = root_search_fx( low, high, &v_low, q, order );

            IF ( rt < 0 )
            {
                /* No Q root in this interval */
                /* copy from previous LSP and return */
                FOR ( i = 0; i < order; i++ )
                {
                    move16();
                    freq[i] = old_freq[i];
                }
                return ( 0 );
            }
            ELSE
            {
                move16();
                tfreq[rc] = rt;
                rc = add( rc, 2 );
            } /*  end else, find Q root */
        } /*  end for */
    } /* end else */

    FOR ( i = 0; i < order; i++ )
    {
        freq[i] = tfreq[i];
        move16();
    }

    return ( 1 );
}

/*===================================================================*/
/* FUNCTION      :  lsp2lpc_fx ()                                    */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Convert LSP coefficients to LPC coefficients     */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*                                                                   */
/*   _ (Word16 []) freq: LSP coefficients, Q15                       */
/*   _ (Word16 []) prev_a : previous frame LPC coefficients, Q12     */
/*   _ (Word16 []) order : LPC order                                 */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) a : LPC coefficients, Q12                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _  None                                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _  None                                                         */
/*===================================================================*/
void lsp2lpc_fx(
    Word16 *a,
    Word16 *freq,
    Word16 *prev_a,
    Word16 order
)
{
    Word16 i;
    Word32 pq[ LPC_SHB_ORDER ] ;
    Word32 p[ LPC_SHB_ORDER ], q[ LPC_SHB_ORDER ];
    Word32 Ltemp;
    Word32 Lacc;
    Word16 tmp_pci[M+1];
    Word16 giOverflow;


    FOR ( i = 0; i < order; i++ )
    {
        tmp_pci[i]=prev_a[i];
        move16();
    }

    compute_poly_product_fx( freq, pq, order );
    /*This change is to get funtionality if L_sub_sat*/
    giOverflow = 0 ;
    move16(); /* clear overflow flag */
    Overflow = 0;
    move16();
    FOR ( i=0; i < order/2; i++ )
    {
        Ltemp = L_add( pq[ i ], pq[ i + 1 ] );
        giOverflow = ( Word16 )Overflow;
        move16();
        IF ( sub(giOverflow,1) == 0  )
        {
            BREAK;
        }

        p[ i ] = Ltemp ;
        move32();
    }

    IF ( sub(giOverflow,1) == 0  )
    {
        FOR (i=0; i < order; i++ )
        {
            a[i] = mult_r( prev_a[i], pwAlpha[i] );
            move16();
        }
        return;
    }

    compute_poly_product_fx( freq+1, pq, order );


    giOverflow = 0;
    move16();
    Overflow = 0;
    move16();

    FOR ( i=0; i < order/2; i++ )
    {
        Ltemp = L_sub( pq[ i+1 ], pq[i] );
        giOverflow = ( Word16 ) Overflow;
        move16();
        IF ( sub(giOverflow,1) == 0  )
        {
            BREAK;
        }

        q[i] = Ltemp;
        move32();
    }

    IF ( sub(giOverflow,1) == 0 )
    {
        FOR (i = 0; i < order; i++ )
        {
            a[i] = mult_r( prev_a[i], pwAlpha[i] );
            move16();
        }
    }
    ELSE
    {
        FOR (i = 0; i < order/2; i++ )
        {
            Overflow = 0;
            move16();
            Lacc = L_add( p[i], q[i] );  /* p[i], q[i] in Q24 */
            if ( Overflow )
            {
                giOverflow = 1;
                move16();
            }

            Lacc = L_negate( Lacc  );    /* Lacc=-(p[i]-q[i])/2 in Q25 */
            Overflow = 0;
            move16();
            Lacc = L_add( L_shl( Lacc, 3 ), 0x08000 );  /* rounding */
            if ( Overflow )
            {
                giOverflow = 1;
                move16();
            }

            a[i] = extract_h( Lacc ); /* a[i] in Q12 */

            IF ( sub(giOverflow,1) == 0  )
            {
                BREAK;
            }

        }
        FOR ( i=0; i < order/2; i++ )
        {
            Overflow = 0;
            move16();
            Lacc = L_sub( q[i], p[i] );  /* p[i], q[i] in Q24 */
            if( Overflow )
            {
                giOverflow = 1;
                move16();
            }
            Overflow = 0;
            move16();
            Lacc = L_add( L_shl( Lacc, 3 ), 0x08000);  /* rounding */
            if ( Overflow )
            {
                giOverflow = 1;
                move16();
            }

            a[ order-1-i ] = extract_h( Lacc );


            IF ( sub(giOverflow,1) == 0  )
            {
                BREAK;
            }

        }
    }


    IF ( sub(giOverflow,1) == 0  )
    {
        FOR ( i = 0; i < order; i++ )
        {
            a[i] = mult_r( tmp_pci[i], pwAlpha[i] );
            move16();
        }
    }

}

/*
 * E_LPC_f_lsp_pol_get
 *
 * Parameters:
 *    lsp/isp        I: Line spectral pairs (cosine domaine)      Q15
 *    f              O: the coefficients of F1 or F2        Q23
 *    n              I: no of coefficients (m/2)
 *                      == NC for F1(z); == NC-1 for F2(z)
 *    fact           I: scaling factor
 *
 *-----------------------------------------------------------*
 * procedure E_LPC_f_lsp_pol_get:                            *
 *           ~~~~~~~~~~~                                     *
 *   Find the polynomial F1(z) or F2(z) from the LSPs.       *
 * This is performed by expanding the product polynomials:   *
 *                                                           *
 * F1(z) =   product   ( 1 - 2 LSF_i z^-1 + z^-2 )           *
 *         i=0,2,4,6,8                                       *
 * F2(z) =   product   ( 1 - 2 LSF_i z^-1 + z^-2 )           *
 *         i=1,3,5,7,9                                       *
 *                                                           *
 * where LSP_i are the LSPs in the cosine domain.            *
 *                                                           *
 *-----------------------------------------------------------*
 *   R.A.Salami    October 1990                              *
 *-----------------------------------------------------------*
 */
Word16 E_LPC_f_lsp_pol_get(const Word16 lsp[], Word32 f[],  const Word16 n, const Word16 past_Ovf, const Word16 isMODE1)
{
    /* All computation in Q23 */
    const Word16 *plsp;
    Word16 i, j;
    Word16 b;
    Word32 b32;
    Word16 Ovf = 0;
    Word16 Q_out;
    Word16 m2;


    Q_out = 31-23;
    move16();
    Ovf = past_Ovf;
    move16();

    test();
    if(past_Ovf && isMODE1) /* Currently this feature is implemented only in MODE1 */
    {
        /* In some NB cases, overflow where detectected
            in f1 or f2 polynomial computation when it
            happen we reduce the precision of the computing
            to limit the risk of saturation*/
        Q_out = add(Q_out, past_Ovf);
    }
    Overflow = 0;
    move16();
    plsp = lsp;
    f[0] = L_shl(1, sub(31, Q_out));
    move32();
    /*b = -2.0f * *plsp;*/
    b = *plsp;
    move16();
    m2 = shl(-2, sub(15, Q_out));
    f[1] = L_mult(b, m2);
    move32();

    FOR (i = 2; i <= n; i++)
    {
        plsp += 2;
        /*b = 2.0f * *plsp;*/
        move16();
        b = *plsp;
        b32 = L_mult(b, m2);

        /*f[i] = -b*f[i-1] + 2.0f*f[i-2];*/
        move32();
        f[i] = L_shl(L_sub(f[i-2], Mpy_32_16_1(f[i-1], b)),1);

        FOR (j = i-1; j > 1; j--)
        {
            /*f[j] += b*f[j-1] + f[j-2];*/
            move32();
            f[j] = L_add(f[j], L_sub(f[j-2], L_shl(Mpy_32_16_1(f[j-1], b),1)));
        }
        move32();
        f[1] = L_add(f[1], b32);
    }


    test();
    IF (Overflow>0 && isMODE1)
    {
        /* If an overflow is detected, redo the computation with 1 bit less */
        Ovf = add(Ovf,1);
        Ovf = E_LPC_f_lsp_pol_get(lsp, f, n ,Ovf, isMODE1);
    }
    return Ovf;
}

void E_LPC_a_lsp_conversion(
    const Word16 *a,       /* input : LP filter coefficients (Qx)                      */
    Word16 *lsp,     /* output: Line spectral pairs (in the cosine domain)(0Q15) */
    const Word16 *old_lsp, /* input : LSP vector from past frame (0Q15)                */
    const Word16 m         /* input : length of the LP filter coefficients             */
)
{
    Word16 i, nf, ip, nc;
    Word16 xlow, ylow, xhigh, yhigh;
    Word16 x, y, tmp, exp;
    Word32 f[2][NC_MAX+1];
    Word32 t0, t1;
    Word32 sum, diff;
    Word16 scale;



    nc = shr(m, 1);

    scale = shl( 128, norm_s(a[0]) );

    /*-------------------------------------------------------------*
     * find the sum and diff polynomials F1(z) and F2(z)           *
     *      F1(z) = [A(z) + z^11 A(z^-1)]/(1+z^-1)                 *
     *      F2(z) = [A(z) - z^11 A(z^-1)]/(1-z^-1)                 *
     *-------------------------------------------------------------*/

    f[0][0] = L_mult(a[0],scale);    /*1.0f in Q23*/          move32();
    f[1][0] = L_mult(a[0],scale);    /*1.0f in Q23*/          move32();
    FOR (i = 1; i < nc; i++)
    {
        t0   = L_mult(a[i],scale);                       /*Q23*/
        sum  = L_mac(t0, a[m+1-i],scale);
        diff = L_msu(t0, a[m+1-i],scale);
        f[0][i] = L_sub(sum,f[0][i-1]);
        move32();   /*Q23*/
        f[1][i] = L_add(diff,f[1][i-1]);
        move32();   /*Q23*/
    }
    t1   = L_mult0(a[i],scale);                            /*Q23-1*/
    sum  = L_mac0(t1, a[m+1-i],scale);
    diff = L_msu0(t1, a[m+1-i],scale);
    f[0][nc] = L_sub(sum, L_shr(f[0][i-1],1));
    move32();   /*Q23-1*/
    f[1][nc] = L_add(diff,L_shr(f[1][i-1],1));
    move32();   /*Q23-1*/

    /* Precalculate difference to index 0 for index 2 */
    f[0][2] = L_sub(f[0][2], f[0][0]);
    move32();
    f[1][2] = L_sub(f[1][2], f[1][0]);
    move32();

    /*---------------------------------------------------------------------*
    * Find the LSPs (roots of F1(z) and F2(z) ) using the                 *
    * Chebyshev polynomial evaluation.                                    *
    * The roots of F1(z) and F2(z) are alternatively searched.            *
    * We start by finding the first root of F1(z) then we switch          *
    * to F2(z) then back to F1(z) and so on until all roots are found.    *
    *                                                                     *
    *  - Evaluate Chebyshev pol. at grid points and check for sign change.*
    *  - If sign change track the root by subdividing the interval        *
    *    4 times and ckecking sign change.                                *
    *---------------------------------------------------------------------*/
    nf = 0;
    move16();  /* number of found frequencies */
    ip = 0;
    move16();  /* indicator for f1 or f2      */

    xlow = Grid[0];
    move16();
    ylow = chebyshev(xlow, f[ip], nc, 8);

    FOR (i = 1; i <= GRID100_POINTS; i++)
    {
        xhigh = xlow;
        move16();
        yhigh = ylow;
        move16();
        xlow = Grid[i];
        move16();
        ylow = chebyshev(xlow, f[ip], nc, 8);

        IF (L_mult(ylow, yhigh) <= 0)
        {
            t0 = L_mult(xhigh, 0x4000);
            /* divide 2 times the interval */
            x = mac_r(t0, xlow, 0x4000); /* xmid = (xlow + xhigh)/2 */
            y = chebyshev(x, f[ip], nc, 8);

            IF (L_mult(ylow, y) <= 0)
            {
                yhigh = y;
                move16();
                xhigh = x;
                move16();
                y = ylow;
                move16();
                x = xlow;
                move16();
                /* 'xhigh' has changed, update 't0' */
                t0 = L_mult(xhigh, 0x4000);
            }
            xlow = mac_r(t0, x, 0x4000); /* xmid = (xlow + xhigh)/2 */
            ylow = chebyshev(xlow, f[ip], nc, 8);

            IF (L_mult(y, ylow) <= 0)
            {
                yhigh = ylow;
                move16();
                xhigh = xlow;
                move16();
                ylow = y;
                move16();
                xlow = x;
                move16();
            }

            /*--------------------------------------------------------*
             * Linear interpolation
             * xint = xlow - ylow*(xhigh-xlow)/(yhigh-ylow)
             *--------------------------------------------------------*/
            y = msu_r(L_mult(yhigh,0x4000), ylow,0x4000);

            IF (y != 0)
            {
                x = sub(xhigh, xlow);
                BASOP_SATURATE_WARNING_OFF
                tmp = abs_s(y);
                BASOP_SATURATE_WARNING_ON
                exp = norm_s(tmp);
                if (exp)
                    tmp = shl(tmp, exp);
                tmp = div_s((Word16) 16383/2, tmp);
                t0 = L_mult(x, tmp);
                t0 = L_shr(t0, sub(20, exp));
                tmp = extract_l(t0);         /* y = (xhigh-xlow)/(yhigh-ylow) in Q11 */

                /* Restore Sign */
                if (y < 0)
                    tmp = negate(tmp);

                t0 = L_mult(ylow, tmp);         /* result in Q26 */
                t0 = L_shr(t0, 11);             /* result in Q15 */
                xlow = sub(xlow, extract_l(t0));/* xint = xlow - ylow*y */
            }
            lsp[nf++] = xlow;
            move16();

            IF (sub(nf,m) >= 0)
            {
                BREAK;
            }

            ip = s_xor(ip, 1);
            ylow = chebyshev(xlow, f[ip], nc, 8);
        }
    }

    /* Check if m roots found */
    /* if not use the LSPs from previous frame */

    IF (sub(nf,m) < 0)
    {
        FOR(i=0; i<m; i++)
        {
            lsp[i] = old_lsp[i];
            move16();
        }
    }


    return;
}

/*
 * E_LPC_f_lsp_a_conversion
 *
 * Parameters:
 *    lsp            I: Line spectral pairs          Q15
 *    a              O: Predictor coefficients (order = m)  Qx (The Q factor of the output to be deduced from a(0))
 *    m              I: order of LP filter
 *
 * Function:
 *    Convert ISPs to predictor coefficients a[]
 *
 * Returns:
 *    void
 */
void E_LPC_f_lsp_a_conversion(const Word16 *lsp, Word16 *a,  const Word16 m)
{
    Word16 i, j, k;
    Word32 f1[NC_MAX+1], f2[NC_MAX+1];
    Word16 nc;
    Word32 t0;
    Word16 Ovf, Ovf2;


    /*-----------------------------------------------------*
     *  Find the polynomials F1(z) and F2(z)               *
     *-----------------------------------------------------*/

    nc = shr(m, 1);

    assert(m == 16 || m == 10);

    Ovf = 0;
    move16();
    Ovf = E_LPC_f_lsp_pol_get(&lsp[0], f1, nc, Ovf, 1);
    Ovf2 = E_LPC_f_lsp_pol_get(&lsp[1], f2, nc, Ovf, 1);
    IF(sub(Ovf2,Ovf) !=0)
    {
        /* to ensure similar scaling for f1 and f2 in case
          an overflow would be detected only in f2,
          but this case never happen on my dtb */
        E_LPC_f_lsp_pol_get(&lsp[0], f1, nc, s_max(Ovf2,Ovf), 1);
    }
    /*-----------------------------------------------------*
     *  Multiply F1(z) by (1+z^-1) and F2(z) by (1-z^-1)   *
     *-----------------------------------------------------*/
    /*modification*/
    k = sub(nc,1);
    FOR (i = 0; i <= k; i++)
    {
        f1[nc-i] = L_add(f1[nc-i],f1[nc-i-1]);
        move32();
        f2[nc-i] = L_sub(f2[nc-i],f2[nc-i-1]);
        move32();
    }

    /*-----------------------------------------------------*
     *  A(z) = (F1(z)+F2(z))/2                             *
     *  F1(z) is symmetric and F2(z) is antisymmetric      *
     *-----------------------------------------------------*/

    t0 = L_deposit_l(0);
    FOR (i = 1; i <= nc; i++)
    {
        t0 = L_max( t0, L_abs(L_add(f1[i], f2[i])) );
        t0 = L_max( t0, L_abs(L_sub(f1[i], f2[i])) );
    }
    k = s_min( norm_l(t0), 6 );
    a[0] = shl( 256, k );
    move16();
    test();
    IF( Ovf || Ovf2)
    {
        a[0] = shl( 256, sub(k,Ovf) );
        move16();
    }
    j = m;
    FOR (i = 1;  i <= nc; i++)
    {
        /* a[i] = 0.5*(f1[i] + f2[i]) */
        t0 = L_add(f1[i],f2[i]);
        t0 = L_shl(t0, k);
        a[i] = round_fx(t0);             /* from Q23 to Qx and * 0.5 */

        /* a[j] = 0.5*(f1[i] - f2[i]) */
        t0 = L_sub(f1[i],f2[i]);
        t0 = L_shl(t0, k);
        a[j] = round_fx(t0);             /* from Q23 to Qx and * 0.5 */
        j--;
    }

    return;
}

/*---------------------------------------------------------------------------
   * reorder_lsf()
   *
   * To make sure that the LSFs are properly ordered and to keep a certain
   * minimum distance between consecutive LSFs.
   *--------------------------------------------------------------------------*/
void reorder_lsf_fx(
    Word16 *lsf,      /* i/o: LSFs in the frequency domain (0..0.5)   Q(x2.56)*/
    const Word16 min_dist,  /* i  : minimum required distance               x2.56*/
    const Word16 n,         /* i  : LPC order                               */
    const Word32 fs         /* i  : sampling frequency                      */
)
{
    Word16 i, lsf_min, n_m_1;
    Word16 lsf_max;

    lsf_min = min_dist;
    move16();

    /*-----------------------------------------------------------------------*
     * Verify the LSF ordering and minimum GAP
     *-----------------------------------------------------------------------*/

    FOR (i = 0; i < n; i++)
    {
        if (sub(lsf[i], lsf_min) < 0)
        {
            lsf[i] = lsf_min;
            move16();
        }
        lsf_min = add(lsf[i], min_dist);
    }

    /*-----------------------------------------------------------------------*
     * Reverify the LSF ordering and minimum GAP in the reverse order (security)
     *-----------------------------------------------------------------------*/
    lsf_max = round_fx(L_sub(L_shr(L_mult0(extract_l(L_shr(fs,1)), 1311),9-16), L_deposit_h(min_dist))); /* Q0 + Q9 , 1311 is 2.56 in Q9 */
    n_m_1 = sub(n,1);
    IF (sub(lsf[n_m_1], lsf_max) > 0)    /* If danger of unstable filter in case of resonance in HF */
    {
        FOR (i = n_m_1; i >= 0; i--) /* Reverify the minimum LSF gap in the reverse direction */
        {
            if (sub(lsf[i], lsf_max) > 0)
            {
                lsf[i] = lsf_max;
                move16();
            }
            lsf_max = sub(lsf[i], min_dist);
        }
    }
}

void space_lsfs_fx (
    Word16* lsfs,           /* i/o: Line spectral frequencies */
    const Word16 order      /* i : order of LP analysis */
)
{
    Word16 delta; /*  Q1.15 */
    Word16 i, flag=1;

    WHILE ( flag == 1 )
    {
        flag = 0;
        move16();
        FOR ( i = 0; i <= order; i++ )
        {
            delta = ( Word16 )( i == 0 ? lsfs[0] : ( i == order ?
                                sub( HALF_POINT_FX, lsfs[i - 1] ) : sub( lsfs[i],
                                        lsfs[i - 1] ) ) );
            move16();
            test();
            IF ( sub( delta, SPC_FX ) < 0 )
            {
                flag = 1;
                move16();
                delta = sub( delta, SPC_PLUS_FX );

                IF ( i == order )
                {
                    lsfs[i - 1] = add( lsfs[i - 1], delta );
                    move16();
                }
                ELSE
                {
                    IF ( i == 0 )
                    {
                        lsfs[i] = sub( lsfs[i], delta );
                        move16();
                    }
                    ELSE
                    {
                        delta = mult_r( delta, HALF_POINT_FX );
                        lsfs[i - 1] = add( lsfs[i - 1], delta );
                        move16();
                        lsfs[i] = sub( lsfs[i], delta );
                        move16();
                    }
                }
            }
        }
    }

    return;
}

/*=================================================================== */
/* FUNCTION : lsp_weights_fx () */
/*------------------------------------------------------------------- */
/* PURPOSE : This function computes the weights for the               */
/* given unquantized lsp vector                                       */
/*------------------------------------------------------------------- */
/* INPUT ARGUMENTS :
_ (Word16 []) lsp_nq_fx: input unquantized lsp vector */
/* _(Word16 Order) FilterOrder */
/*------------------------------------------------------------------- */
/* OUTPUT ARGUMENTS : */
/* _ (Word16 []) w: weight vector Q(9-n_max) */

/*------------------------------------------------------------------- */
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None. */
/*------------------------------------------------------------------- */
/* RETURN ARGUMENTS : */
/* _ None. */
/*=================================================================== */

void lsp_weights_fx(
    Word16 lsp_nq_fx[],
    Word16 w[],
    Word16 Order,
    Word16* Qout
)
{
    Word16 lpcOrder =Order;

    Word16 i, n1, tmp_loop;
    Word16 norm[20];
    Word32 Lsum1[20];
    Word16 delta1, delta2, temp;
    Word16 n_max =-32768;
    move16();

    temp = 0;
    move16();

    tmp_loop = sub( lpcOrder, 1 );
    FOR ( i = 0; i < tmp_loop; i++ )
    {
        delta1 = sub( lsp_nq_fx[i], temp );
        delta2 = sub( lsp_nq_fx[i + 1], lsp_nq_fx[i] );
        Lsum1[i] = calc_weight( delta1, delta2, &n1 );
        move32();/* Q( 26-n1) */
        norm[i] = n1;
        move16();

        if (sub(norm[i],n_max) > 0 )
        {
            n_max = norm[i];
            move16();
        }
        temp = lsp_nq_fx[i];
        move16();
    }
    delta1 = sub( lsp_nq_fx[i], temp );
    delta2 = sub( 16384, lsp_nq_fx[i] );

    Lsum1[i] = calc_weight( delta1, delta2, &n1 );
    move32(); /* Q( 26-n1) */
    norm[i] = n1;
    move16();

    if ( sub(norm[i], n_max) > 0 )
    {
        n_max = norm[i];
        move16();
    }
    FOR ( i = 0; i < lpcOrder; i++ )
    {
        w[i] = round_fx( L_shl( Lsum1[i], sub( norm[i], n_max + 1 ) ) ); /* Q( 9-n_max) */
    }

    IF ( lpcOrder != LPC_SHB_ORDER_WB )
    {
        w[3] = round_fx( L_shl( L_mult( w[3], 18022 ), 1 ) ); /* Q( 9-n_max) */
        w[4] = round_fx( L_shl( L_mult( w[4], 18022 ), 1 ) ); /* Q( 9-n_max) */
    }

    *Qout = 9 - n_max;
    move16();
}

/*
 * E_LPC_isf_isp_conversion
 *
 * Parameters:
 *    isf            I: isf[m] normalized (range: 0 <= val <= 0.5)  14Q1*1.28
 *    isp            O: isp[m] (range: -1 <= val < 1)        Q15
 *    m              I: LPC order
 *
 * Function:
 *    Transformation isf to isp
 *
 *    ISF are immitance spectral pair in frequency domain (0 to 6400).
 *    ISP are immitance spectral pair in cosine domain (-1 to 1).
 *
 * Returns:
 *    void
 */
void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m)
{
    Word16 i;

    assert( m==16 || m==10 );


    FOR (i = 1; i < m; i++)
    {
        *isp++ = xsf_to_xsp(*isf++);
        move16();
    }
    *isp = xsf_to_xsp(shl(*isf, 1));
    move16();


    return;
}

/*
 * E_LPC_isp_isf_conversion
 *
 * Parameters:
 *    isp            I: isp[m] (range: -1 <= val < 1)               Q15
 *    isf            O: isf[m] normalized (range: 0 <= val <= 6400) x1.28
 *    m              I: LPC order
 *
 * Function:
 *    Transformation isp to isf
 *
 *    ISP are immitance spectral pair in cosine domain (-1 to 1).
 *    ISF are immitance spectral pair in frequency domain (0 to 6400).
 *
 * Returns:
 *    energy of prediction error
 */
void E_LPC_isp_isf_conversion(const Word16 isp[], Word16 isf[], const Word16 m)
{
    Word16 i;

    assert( m==16 || m==10 );



    FOR (i = 0; i < m; i++)
    {
        isf[i] = xsp_to_xsf(isp[i]);
        move16();
    }

    isf[m - 1] = shr(isf[m - 1], 1);
    move16();


    return;
}


Word16 xsf_to_xsp(Word16 lsf)
{
    /* lsp = cos(lsf * 3.1415/6400); */
    return getCosWord16R2(lsf);
}

Word16 xsp_to_xsf(Word16 lsp)
{
    Word16 ind, tmp;
    Word32 L_tmp;


    /*------------------------------------------------------*
     * find value in table that is just greater than lsp
     *------------------------------------------------------*/

    /* Retrieve Index Guess */
    /* Based on lsp */
    ind = mac_r(GUESS_TBL_SZ/2*65536-0x8000, lsp, GUESS_TBL_SZ/2);
    ind = Ind_Guess[ind];
    move16();

    /* Correct Index so that */
    /*  table[ind] > isp[i]  */
    tmp = sub(lsp, table[ind]);
    /*
       69%: (Final Index - Index Guess) is <= 1
       28%: (Final Index - Index Guess) is 2
        2%: (Final Index - Index Guess) is >= 3
       <1%: ...
    */
    IF (tmp > 0) /* possible range 0 to -5 (-1-2-2) */
    {
        ind = sub(ind, 1);
        tmp = sub(lsp, table[ind]);

        IF (tmp > 0)
        {
            ind = sub(ind, 2);
            tmp = sub(lsp, table[ind]);
            if (tmp > 0)
            {
                ind = sub(ind, 2);
            }
            tmp = sub(lsp, table[ind+1]);
            if (tmp <= 0)
            {
                ind = add(ind, 1);
            }
            tmp = sub(lsp, table[ind]);
        }
    }

    /* acos(lsp)= ind*128 + (lsp-table[ind]) * slope[ind] / 2048 */
    L_tmp = L_mac(1L<<11, tmp, slope[ind]);
    L_tmp = L_shr(L_tmp, 12);  /* (lsp-table[ind]) * slope[ind]) >> 11 */
    L_tmp = L_mac0(L_tmp, ind, 128);


    return extract_l(L_tmp);
}

/*-------------------------------------------------------------------*
 * a2rc()
 *
 * Convert from LPC to reflection coeff
 *-------------------------------------------------------------------*/

void a2rc_fx( const Word16* a,        /* i:  can be any Q     */
              Word16* refl,      /* o:  Q15              */
              Word16 lpcorder
            )

{
    Word16 f_fx[M];
    Word16 km_fx;
    Word32 L_tmp1, L_tmp2;
    Word16 tmp;
    Word16 denom_mant, denom_exp, exp;
    Word16 tmp_denom_exp;
    Word32 new_mant;
    Word16 temp;
    Word16 m, j, n;
    Word16 q, q_a, q_a2, One_Qx;
    Word32 One_Qx2;
    q = add( norm_s(a[-1]), 1 );
    q_a = sub(15, q);
    q_a2 = add(shl(q_a,1),1);
    /* copy into internal vars so they can be changed */

    FOR ( m = 0; m < lpcorder; m++ )
    {
        /* f_fx[m] = p_fx[m]; */
        f_fx[m] = negate(a[m]);
        move16();
    }
    One_Qx = shl(1,q_a);
    One_Qx2 = L_shl(1, q_a2);
    FOR ( m = lpcorder - 1; m >= 0; m-- )
    {
        km_fx = f_fx[m];
        move16();

        test();
        IF ( sub( km_fx, negate(One_Qx)) <= 0 || sub( km_fx, One_Qx) >= 0 )
        {
            FOR ( j = 0; j < lpcorder; j++ )
            {
                refl[j] = 0;
                move16();
            }

            return;
        }

        refl[m] = negate( km_fx );
        move16();
        L_tmp1 = L_add(0,One_Qx2); /* 1 in 2xq_a+1 */
        L_tmp1 = L_msu( L_tmp1, km_fx, km_fx ); /* 1-km*km in Q25 */
        denom_exp = norm_l( L_tmp1 );

        /* new_mant = invert_dp(L_tmp1,4, &tmp_denom_exp,1); sum in Q61-Q25-n=Q36-n */
        exp = norm_l( L_tmp1 );
        tmp_denom_exp = exp;
        tmp = extract_h( L_shl( L_tmp1, exp ) );
        exp = sub( sub( 30, exp ), q_a2 );
        IF ( tmp )
        {
            tmp = div_s( 16384, tmp ); /* 15+exp */
        }
        ELSE
        {
            tmp = 0;
        }
        new_mant = L_deposit_h( tmp );
        temp = round_fx( L_shl( new_mant, 0 ) ); /* in Q14 */
        denom_mant = temp;
        move16();
        denom_exp = abs_s( sub( tmp_denom_exp, 5 ) );
        L_tmp1 = L_mult( km_fx, denom_mant ); /* km*denom. Q12*Q14 = Q27 */
        L_tmp1 = L_shl( L_tmp1, q ); /* change to Q31. simulation showed no overflow */
        tmp = round_fx( L_tmp1 ); /* extract in Q15 */

        FOR ( j = 0; j < m / 2; j++ )
        {
            n = sub( sub( m, ( Word16 )1 ), j );
            L_tmp1 = L_mult( denom_mant, f_fx[j] ); /* denom*f[j]. Q15*Q12 = Q28 (floating with exp) */
            L_tmp1 = L_mac( L_tmp1, tmp, f_fx[n] ); /* denom*f[j]+km*denom*f[n] in Q28 (floating with exp) */
            L_tmp2 = L_mult( denom_mant,f_fx[n] ); /* denom*f[n]. Q15*Q12 = Q28 (floating with exp) */
            L_tmp2 = L_mac( L_tmp2, tmp,f_fx[j] ); /* denom*f[n]+km*denom*f[j] in Q28 (floating with exp) */
            L_tmp1 = L_shl( L_tmp1, denom_exp ); /* bringing to true Q28 */
            L_tmp2 = L_shl( L_tmp2, denom_exp ); /* bringing to true Q28 */
            f_fx[j] = round_fx( L_tmp1 ); /* extracting in q_a */
            f_fx[n] = round_fx( L_tmp2 ); /* extracting in q_a */
        }

        IF ( m & 1 )
        {
            L_tmp1 = L_mult( denom_mant, f_fx[ j] ); /* denom*f[j]. Q15*Q12 = Q28 (floating with exp) */
            L_tmp1 = L_mac( L_tmp1, tmp,f_fx[j] ); /* denom*f[j]+km*denom*f[j] in Q28 (floating with exp) */
            L_tmp1 = L_shl( L_tmp1, denom_exp ); /* bringing to true Q28 */
            f_fx[j] = round_fx( L_tmp1 ); /* extracting in q_a */
        }
    }

    FOR ( j = 0; j < lpcorder; j++ )
    {
        refl[j] = shl( refl[j], q );
        move16();
    }


    return;
}

void vq_dec_lvq_fx (
    Word16 sf_flag,        /* i  : safety net flag                           */
    Word16 x[],            /* o  : Decoded vector                    Q(x2.56)*/
    Word16 indices[],      /* i  : Indices                                   */
    Word16 stages,         /* i  : Number of stages                          */
    Word16 N,              /* i  : Vector dimension                          */
    Word16 mode,           /* (i): mode_lvq, or mode_lvq_p                     */
    Word16 no_bits,        /* (i): no. bits for lattice                         */
    Word32 *p_offset_scale1,
    Word32 *p_offset_scale2,
    Word32 *p_offset_scale1_p,
    Word32 *p_offset_scale2_p,
    Word16 *p_no_scales,
    Word16 *p_no_scales_p
)
{
    Word16  x_lvq[M];
    Word16 i, stagesm1;
    Word16 pt_fx;

    /* clear vector */
    set16_fx(x, 0, N);

    /*-----------------------------------------------*
     * add contribution of each stage
     *-----------------------------------------------*/
    stagesm1 = sub(stages,1);
    IF (sub(sf_flag,1) == 0)
    {
        FOR(i=0; i<stagesm1; i++)
        {
            pt_fx = i_mult2(indices[i], N);
            Vr_add(x, &Quantizers_fx[CB_fx[mode]+i][pt_fx], x, N);
        }

        deindex_lvq_fx(&indices[stagesm1], x_lvq, mode, sf_flag, no_bits, p_offset_scale1, p_offset_scale2, p_no_scales);
    }
    ELSE
    {
        FOR(i=0; i<stagesm1; i++)
        {
            pt_fx = i_mult2(indices[i], N);
            Vr_add(x, &Quantizers_p_fx[CB_p_fx[mode]+i][pt_fx], x, N);
        }

        deindex_lvq_fx(&indices[stagesm1], x_lvq, mode,  sf_flag, no_bits, p_offset_scale1_p, p_offset_scale2_p, p_no_scales_p);
    }

    Vr_add(x, x_lvq, x, N);
}

void lsf_allocate_fx(
    const Word16 nBits,       /* i  : Number of bits to use for quantization     */
    const Word16 framemode,   /* i  : ISF quantizer mode                         */
    const Word16 framemode_p, /* i  : ISF quantizer mode predmode (mode_lvq_p)   */
    Word16 *stages0,    /* o  : Number of stages for safety-net quantizer  */
    Word16 *stages1,    /* o  : Number of stages for predictive quantizer  */
    Word16 levels0[],   /* o  : Number of vectors for each stage for SFNET */
    Word16 levels1[],   /* o  : Number of vectors for each stage for pred  */
    Word16 bits0[],     /* o  : Number of bits for each stage safety net   */
    Word16 bits1[]      /* o  : Number of bits for each stage pred         */
)
{
    Word16 i;
    Word16 cumleft;
    Word16 bits_lvq, n_stages, nbits0;

    /* VOICED@16kHz */
    IF(sub(framemode, 14) == 0)
    {
        return;
    }
    cumleft = nBits;
    move16();

    /*---------------------------------------------------*
     * Calculate bit allocation for safety-net quantizer
     *---------------------------------------------------*/

    cumleft = BitsVQ_fx[framemode];
    move16();
    bits_lvq = sub(nBits,cumleft);
    nbits0 = CBbits_fx[framemode];
    move16();
    IF (sub(nbits0, -1) >0)
    {
        IF (nbits0>0)
        {
            n_stages = 2;
            move16();
            levels0[0] = CBsizes_fx[nbits0];
            move16();
            bits0[0] = nbits0;
            move16();
            bits0[1] = sub(cumleft,nbits0);

            IF ( bits0[1] == 0 )
            {
                n_stages = sub(n_stages,1);
            }
            ELSE
            {
                levels0[1] = CBsizes_fx[sub(cumleft,nbits0)];
                move16();
            }
        }
        ELSE /* no bits for VQ stage */
        {
            n_stages = 0;
            move16();
        }

        *stages0 = n_stages;
        move16();
        IF (bits_lvq > 0)
        {
            bits0[n_stages] = bits_lvq;
            move16();
            levels0[n_stages] = bits_lvq;
            move16();/* this is number of bits, not levels */
            *stages0 = add(n_stages,1);
            move16();
        }
    }
    ELSE
    {
        *stages0 = 0;
        move16();
    }

    /*---------------------------------------------------*
     * Calculate bit allocation for predictive quantizer
     *---------------------------------------------------*/
    IF ( sub(framemode_p, -1)>0 )
    {
        cumleft = BitsVQ_p_fx[framemode_p];
        move16();
        bits_lvq = sub(nBits, cumleft);
        nbits0 = CBbits_p_fx[framemode_p];
        move16();

        IF (sub(nbits0,-1) > 0)
        {
            IF ( nbits0 > 0 )
            {
                IF ( sub(framemode_p, 7) == 0 )
                {
                    /* for UNVOICED_WB only */
                    n_stages = 3;
                    move16();
                    FOR( i=0; i<n_stages; i++ )
                    {
                        levels1[i] = CBsizes_fx[nbits0];
                        move16();
                        bits1[i] = nbits0;
                        move16();
                    }
                    bits1[n_stages] = bits_lvq;
                    move16();
                    levels1[n_stages] = bits_lvq;
                    move16();
                    *stages1 = add(n_stages, 1);
                }
                ELSE
                {
                    n_stages = 1;
                    move16();
                    levels1[0] = CBsizes_fx[nbits0];
                    move16();
                    bits1[0] = nbits0;
                    move16();
                    nbits0 = sub(cumleft, nbits0);
                    IF (nbits0>0)
                    {
                        levels1[1] = CBsizes_fx[nbits0];
                        move16();
                        bits1[1] = nbits0;
                        move16();
                        n_stages = 2;
                        move16();
                    }

                    levels1[n_stages] = bits_lvq;
                    move16();/* this is number of bits, not levels */
                    bits1[n_stages] = bits_lvq;
                    move16();
                    *stages1 = add(n_stages ,1);
                }
            }
            ELSE
            {
                *stages1 = 1;
                move16();
                bits1[0] = bits_lvq;
                move16();
                levels1[0] = bits_lvq;
                move16();
            }
        }
        ELSE
        {
        }
    }

    return;
}

Word16 find_pred_mode(
    const Word16 coder_type,   /* i: coding type                                          */
    const Word16 bwidth,       /* i: bandwidth index                                      */
    const Word32 int_fs,       /* i: sampling frequency                                   */
    Word16 * p_mode_lvq,       /* o: index of LSF codebooks in safety net mode            */
    Word16 * p_mode_lvq_p,     /* o: index of LSF codebooks in predictive mode (AR or MA) */
    Word32 core_brate)         /* i: core bit rate                                        */
{
    Word16 idx, predmode;

    /* bwidth = 0(NB), 1 (WB), 2(WB2); line index in predmode_tab[][] */
    idx = bwidth;
    move16();
    if (sub(idx, 1) >0)
    {
        idx = 1;
    }
    IF (L_sub(int_fs, INT_FS_16k) == 0)
    {
        /* WB2 is actually used if sampling frequency is 16kHz */
        idx = 2;
        move16();
    }
    ELSE
    {
        test();
        test();
        if ((L_sub(core_brate, GENERIC_MA_LIMIT) >= 0)&&(sub(coder_type, GENERIC) == 0)&&(sub(idx,1) == 0))
        {
            idx = 3;
            move16();
        }
    }
    predmode = predmode_tab[idx][coder_type];
    move16();
    IF (sub(idx, 2) <= 0)
    {
        *p_mode_lvq = add(i_mult2(NO_CODING_MODES, idx), coder_type);
        IF (predmode>0)
        {
            *p_mode_lvq_p = *p_mode_lvq;
            move16();
        }
        ELSE
        {
            *p_mode_lvq_p = -1;
            move16();
        }
    }
    ELSE  /* WB 12.8 with MA pred in GENERIC*/
    {
        *p_mode_lvq = add(NO_CODING_MODES, coder_type);
        IF (sub(coder_type, GENERIC) == 0)
        {
            *p_mode_lvq_p = 18;
            move16();
        }
        ELSE
        {
            IF (predmode>0)
            {
                *p_mode_lvq_p = *p_mode_lvq;
            }
            ELSE
            {
                *p_mode_lvq_p = -1;
            }
        }
    }



    return predmode;
}

/*---------------------------------------------------------------------------*
 * reorder_isf
 *
 * To make sure that the  isfs are properly ordered and to keep a certain
 * minimum distance between consecutive isfs.
 *--------------------------------------------------------------------------*/
void reorder_isf_fx(
    Word16 *isf,      /* i/o: ISFs in the frequency domain (0..0.5)   */
    const Word16 min_dist,  /* i  : minimum required distance               */
    const Word16 n,         /* i  : LPC order                               */
    const Word16 fs         /* i  : sampling frequency                      */
)
{
    Word16 i, isf_min;
    Word16 isf_max;

    isf_min = min_dist;
    move16();

    /*-----------------------------------------------------------------------*
     * Verify the ISF ordering and minimum GAP
     *-----------------------------------------------------------------------*/

    FOR (i = 0; i < n - 1; i++)
    {
        if (sub(isf[i], isf_min) < 0)
        {
            isf[i] = isf_min;
            move16();
        }
        isf_min = add(isf[i], min_dist);
    }

    /*-----------------------------------------------------------------------*
     * Reverify the ISF ordering and minimum GAP in the reverse order (security)
     *-----------------------------------------------------------------------*/

    /*isf_max = sub(shr(fs,1), min_dist);*/
    isf_max = sub(fs, min_dist);      /* Fs already divide per 2 */

    IF (sub(isf[n-2], isf_max) > 0)    /* If danger of unstable filter in case of resonance in HF */
    {
        FOR (i = sub(n, 2); i >= 0; i--) /* Reverify the minimum ISF gap in the reverse direction */
        {
            if (sub(isf[i], isf_max) > 0)
            {
                isf[i] = isf_max;
                move16();
            }
            isf_max = sub(isf[i], min_dist);
        }
    }
}

/*========================================================================*/
/* FUNCTION : lsf_stab_fx()                                                  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Check LSF stability (distance between old LSFs and              */
/*             current LSFs)                                                  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                      */
/* _ (Word16) Opt_AMR_WB  : flag indicating AMR-WB IO mode                  */
/* _ (Word16*) lsf          : LSPs from past frame                 Q(x2.56) */
/* _ (Word16*) lsfold      : LSPs from past frame                 Q(x2.56) */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                      */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                      */
/* _ (Word16) stab_fac_fx        : LP filter stability   Q15                  */
/*========================================================================*/
Word16 lsf_stab_fx(            /* o  : LP filter stability   Q15*/
    const Word16 *lsf,      /* i  : LSF vector            Q(x2.56)*/
    const Word16 *lsfold,    /* i  : old LSF vector        Q(x2.56)*/
    const Word16 Opt_AMR_WB,    /* i  : flag indicating AMR-WB IO mode */
    const Word16 L_frame       /* i  : frame length */
)
{
    Word16 i, m;
    Word32 L_tmp;
    Word16 tmp, e;

    /*-------------------------------------------------------------------*
     * Check stability on lsf: distance between old lsf and current lsf
     *-------------------------------------------------------------------*/
    IF ( Opt_AMR_WB )
    {
        m = M-1;
        move16();
        tmp = sub(lsf[0], lsfold[0]);
        L_tmp = L_mult(tmp, tmp);   /* Q1 */
        FOR (i = 1; i < m; i++)
        {
            tmp = sub(lsf[i], lsfold[i]);
            L_tmp = L_mac(L_tmp, tmp, tmp);   /* Q1 */
        }
    }
    ELSE
    {
        m = M;
        move16();
        L_tmp = 0;
        move32();
        FOR (i = 0; i < m; i++)
        {
            tmp = sub(lsf[i], lsfold[i]);
            L_tmp = L_mac(L_tmp, tmp, tmp);   /* Q1 */
        }
    }

    e = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, e); /*Q(1+e)*/

    IF( L_frame == L_FRAME16k)
    {
        /*stab_fac = (float)(1.25f - (tmp/625000.0f));*/
        L_tmp = Mpy_32_16_1(L_tmp, 16777);     /* 30-eQ(1+e)*-21Q36 = 30-21-eQ31-9+e */
    }
    ELSE
    {
        /* stab_fac = (float)(1.25f - (tmp1/400000.0f*2.56=1024000)) */
        L_tmp = Mpy_32_16_1(L_tmp, 26214);     /* 30-eQ(1+e)*-21Q36 = 30-21-eQ31-9+e */
    }

    e = sub(30-21-1,e);
    tmp = round_fx(L_shl(L_tmp, e)); /*Q14*/

    tmp = sub(20480, tmp);                /* 1.25 - tmp in Q14          */
    tmp = shl(tmp, 1);                    /* Q14 -> Q15 with saturation */

    tmp = s_max(tmp, 0);

    return tmp;
}
/*-------------------------------------------------------------------*
  * lsp2isp()
  *
  * Convert LSPs to ISPs via predictor coefficients A[]
  *-------------------------------------------------------------------*/

void lsp2isp_fx(
    const Word16 *lsp,           /* i  : LSP vector                        */
    Word16 *isp,           /* o  : ISP filter coefficients           */
    Word16 *stable_isp,    /* i/o: ISP filter coefficients           */
    const Word16 m               /* i  : order of LP analysis              */
)
{
    Word16 a[M+1];

    /* LSP --> A */
    /*lsp2a_stab( lsp, a, m );*/
    E_LPC_f_lsp_a_conversion(lsp, a, m );

    /* A --> ISP */
    /*a2isp( a, isp, stable_isp, grid );*/
    E_LPC_a_isp_conversion( a, isp, stable_isp, m);

    /* Update to latest stable ISP */
    Copy( isp, stable_isp, M );
}
/*-------------------------------------------------------------------*
  * isp2lsp()
  *
  * Convert ISPs to LSPs via predictor coefficients A[]
  *-------------------------------------------------------------------*/

void isp2lsp_fx(
    const Word16 *isp,           /* i  : LSP vector                        */
    Word16 *lsp,           /* o  : ISP filter coefficients           */
    Word16 *stable_lsp,    /* i/o: stable LSP filter coefficients    */
    const Word16 m               /* i  : order of LP analysis              */
)
{
    Word16 a[M+1];

    /* ISP --> A */
    /*isp2a( isp, a, m );*/
    E_LPC_f_isp_a_conversion(isp, a, m );
    /* A --> LSP */
    /*a2lsp_stab( a, lsp, stable_lsp, grid );*/
    E_LPC_a_lsp_conversion(a, lsp, stable_lsp, m );
    /* Update to latest stable LSP */
    Copy( lsp, stable_lsp, M );
}

/*-------------------------------------------------------------------*
   * lsf2isf()
   *
   * Convert LSPs to ISPs
   *-------------------------------------------------------------------*/

void lsf2isf_fx(
    const Word16 *lsf,           /* i  : LSF vector                        */
    Word16 *isf,           /* o  : ISF vector                        */
    Word16 *stable_isp,    /* i/o: stable ISP filter coefficients    */
    const Word16 m               /* i  : order of LP analysis              */
)
{
    Word16 tmp_lsp[M];
    Word16 tmp_isp[M];

    /* LSF --> LSP */
    /*lsf2lsp( lsf, tmp_lsp, m, int_fs );*/
    E_LPC_lsf_lsp_conversion(lsf, tmp_lsp, m);

    /* LSP --> ISP */
    lsp2isp_fx( tmp_lsp, tmp_isp, stable_isp, m );

    /* ISP --> ISF */
    /*isp2isf( tmp_isp, isf, m, int_fs );*/
    E_LPC_isp_isf_conversion(tmp_isp, isf, m);

    return;
}
/*-------------------------------------------------------------------*
   * isf2lsf()
   *
   * Convert ISFs to LSFs
   *-------------------------------------------------------------------*/

void isf2lsf_fx(
    const Word16 *isf,           /* i  : ISF vector                        */
    Word16 *lsf,           /* o  : LSF vector                        */
    Word16 *stable_lsp     /* i/o: stable LSP filter coefficients    */
)
{
    Word16 tmp_isp[M];
    Word16 tmp_lsp[M];

    /* ISF --> ISP */
    /*isf2isp( isf, tmp_isp, m, int_fs );*/
    E_LPC_isf_isp_conversion(isf, tmp_isp, M);
    /* ISP --> LSP */
    isp2lsp_fx( tmp_isp, tmp_lsp, stable_lsp, M);

    /* LSP --> LSF */
    /*lsp2lsf( tmp_lsp, lsf, m, int_fs );*/
    E_LPC_lsp_lsf_conversion(tmp_lsp, lsf, M);
    return;
}
/*==========================================================================*/
/* FUNCTION      : void lsp2lsf_fx ()                                        */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :                                                            */
/*  * Transformation of LSPs to LSFs                                        */
/*  * LSP are line spectral pair in cosine domain (-1 to 1).                */
/*  * LSF are line spectral frequencies (0 to fs/2).                        */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                        */
/* Word16 lsp[]      i  : lsp[m] (range: -1<=val<1)                       Q15  */
/* Word16 m          i  : LPC order                                        Q0  */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/* Word16 lsf[]      o  : lsf[m] normalized (range: 0.0<=val<=0.5)  Q(x2.56)*/
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                    */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                        */
/*                     _ None                                                    */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                                                             */
/*==========================================================================*/
void lsp2lsf_fx(
    const Word16 lsp[],     /* i  : lsp[m] (range: -1<=val<1)                       Q15*/
    Word16 lsf[],     /* o  : lsf[m] normalized (range: 0.0<=val<=0.5)    Q(x2.56)*/
    const Word16 m          /* i  : LPC order                                        Q0*/
    ,Word32 int_fs
)
{
    Word16 i;
    Word32 L_tmp;

    FOR (i = 0; i < m; i++)
    {
        /*------------------------------------------------------*
         * find value in table that is just greater than lsp[i]
         *------------------------------------------------------*/

        /* Retrieve Index Guess */
        /* Based on lsp[i] */
        L_tmp = sub_lsp2lsf_fx(lsp[i]);
        IF(L_sub(int_fs, INT_FS_16k_FX) ==0)
        {
            L_tmp = L_shr(L_mult0(extract_l(L_tmp),5),2);
        }
        lsf[i] = extract_l(L_tmp);
    }
}
/*===========================================================================*/
/* FUNCTION : lsf2lsp_fx()                                                     */
/*---------------------------------------------------------------------------*/
/* PURPOSE :  Transformation of LSFs to LSPs                                 */
/* LSP are line spectral pairs in cosine domain (-1 to 1).                   */
/* LSF are line spectral frequencies (0 to fs/2).                             */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                         */
/* _ (Word16[]) lsf        : lsf[m] normalized (range: 0.0<=val<=0.5) Q(x2.56)     */
/* _ (Word16) m            : LPC order                                             */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                         */
/* _ (Word16*) lsp        : lsp[m] (range: -1<=val<1)                   Q15         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                         */
/* _ None                                                                     */
/*===========================================================================*/
void lsf2lsp_fx(
    const Word16 lsf[],     /* i  : lsf[m] normalized (range: 0.0<=val<=0.5)  x2.56 */
    Word16 lsp[],     /* o  : lsp[m] (range: -1<=val<1)                 Q15   */
    const Word16 m          /* i  : LPC order                                 Q0    */
    , Word32 int_fs
)
{
    Word16 i, ind, offset;
    Word32 L_tmp;
    Word16 diff;
    Word16 indmdiff;
    Word16 lsf_tmp;


    /* convert LSFs to the cosine domain */

    FOR (i = 0; i < m; i++)
    {
        IF(L_sub(int_fs, INT_FS_16k_FX) == 0)
        {
            L_tmp = L_mult(*lsf,26214); /*  26214 = 0.8 in Q15 */
            lsf_tmp = extract_h(L_tmp);
            ind = shr(lsf_tmp, 7);             /* ind    = b7-b15 of lsf[i] */
            offset = s_and(lsf_tmp, 0x7f);   /* offset = b0-b6  of lsf[i] */
            lsf++;
        }
        ELSE
        {
            ind = shr(*lsf, 7);             /* ind    = b7-b15 of lsf[i] */
            offset = s_and(*lsf++, 0x7f);   /* offset = b0-b6  of lsf[i] */
        }
        offset = shl(offset, 8);
        /* lsp[i] = table[ind] + ((table[ind+1]-table[ind]) * offset) / 128 */
        IF(sub(ind,128)>=0)
        {
            diff = sub(ind,128);
            indmdiff = sub(ind,diff);
            L_tmp = L_mult(sub(table[sub(indmdiff, 1)], table[indmdiff]), offset);
        }
        ELSE
        {
            L_tmp = L_mult(sub(table[add(ind, 1)], table[ind]), offset);
        }
        *lsp++ = extract_h(L_msu(L_tmp, table[ind], -32768));
        move16();
    }
}

void tcvq_Dec_fx(Word16 *ind, /*float *d_out, */Word16 *d_out_fx, Word16 safety_net)
{
    Word16 i;
    Word16 index[9];
    Word16 stage, state, branch[N_STAGE], codeword[N_STAGE];
    Word16 fins, iwd;

    Word16 pred_fx[N_DIM];
    Word16 D_fx[N_STAGE_VQ][N_DIM];
    const Word16 (*TCVQ_CB_SUB1_fx)[128][2], (*TCVQ_CB_SUB2_fx)[64][2], (*TCVQ_CB_SUB3_fx)[32][2];
    const Word16 (*IntraCoeff_fx)[2][2];

    /* mvs2s(ind, index, 9); */
    Copy(ind, index, 9);

    IF (safety_net)
    {
        TCVQ_CB_SUB1_fx = SN_TCVQ_CB_SUB1_fx;
        TCVQ_CB_SUB2_fx = SN_TCVQ_CB_SUB2_fx;
        TCVQ_CB_SUB3_fx = SN_TCVQ_CB_SUB3_fx;
        IntraCoeff_fx   = SN_IntraCoeff_fx;
    }
    ELSE
    {
        TCVQ_CB_SUB1_fx = AR_TCVQ_CB_SUB1_fx;
        TCVQ_CB_SUB2_fx = AR_TCVQ_CB_SUB2_fx;
        TCVQ_CB_SUB3_fx = AR_TCVQ_CB_SUB3_fx;
        IntraCoeff_fx   = AR_IntraCoeff_fx;
    }

    /* Decode Final state */
    fins = s_and(index[0], 15);
    move16();

    /* Decode Branch info */
    branch[0] = shr(index[1], 4);
    move16();
    branch[1] = shr(index[2], 4);
    move16();
    FOR (stage = 2; stage < N_STAGE_VQ-4; stage++)
    {
        branch[stage] = shr(index[stage+1], 3);
        move16();
    }

    branch[4] = s_and(fins, 0x1);
    move16();
    branch[5] = s_and(shr(fins, 1), 0x1);
    move16();
    branch[6] = s_and(shr(fins, 2), 0x1);
    move16();
    branch[7] = s_and(shr(fins, 3), 0x1);
    move16();

    /* Decode Codeword info */
    FOR (stage = 0; stage < 2; stage++)
    {
        codeword[stage] = shl(s_and(index[stage+1], 15), 3);
        move16();
    }

    FOR (stage = 2; stage < N_STAGE_VQ-4; stage++)
    {
        codeword[stage] = shl(s_and(index[stage+1], 7), 3);
        move16();
    }

    FOR (stage = N_STAGE_VQ-4; stage < N_STAGE_VQ; stage++)
    {
        codeword[stage] = shl(s_and(index[stage+1], 3), 3);
        move16();
    }

    state = shl(shr(fins, 2), 2);

    /* stage #1 */
    iwd    = add(NTRANS2[branch[0]+2][state], codeword[0]);
    /* D[0][0] = TCVQ_CB_SUB1[0][iwd][0]; */
    /* D[0][1] = TCVQ_CB_SUB1[0][iwd][1]; */
    D_fx[0][0] = TCVQ_CB_SUB1_fx[0][iwd][0];
    move16();
    D_fx[0][1] = TCVQ_CB_SUB1_fx[0][iwd][1];
    move16();
    state  = NTRANS2[branch[0]][state];
    move16();

    /* stage #2 */
    /* pred[0] = IntraCoeff[0][0][0] * D[0][0] + IntraCoeff[0][0][1]*D[0][1]; */
    /* pred[1]  = IntraCoeff[0][1][0] * D[0][0] + IntraCoeff[0][1][1]*D[0][1]; */
    pred_fx[0] = add(mult_r(IntraCoeff_fx[0][0][0], D_fx[0][0]), mult_r(IntraCoeff_fx[0][0][1], D_fx[0][1]));
    move16();
    pred_fx[1] = add(mult_r(IntraCoeff_fx[0][1][0], D_fx[0][0]), mult_r(IntraCoeff_fx[0][1][1], D_fx[0][1]));
    move16();

    iwd    = add(NTRANS2[branch[1]+2][state], codeword[1]);
    /* D[1][0] = TCVQ_CB_SUB1[1][iwd][0] + pred[0]; */
    /* D[1][1] = TCVQ_CB_SUB1[1][iwd][1] + pred[1]; */
    D_fx[1][0] = add(TCVQ_CB_SUB1_fx[1][iwd][0], pred_fx[0]);
    move16();
    D_fx[1][1] = add(TCVQ_CB_SUB1_fx[1][iwd][1], pred_fx[1]);
    move16();
    state  = NTRANS2[branch[1]][state];
    move16();

    /* stage #3 - #4 */
    FOR (stage = 2; stage < N_STAGE_VQ-4; stage++)
    {
        /* pred[0]     = IntraCoeff[stage-1][0][0] * D[stage-1][0] + IntraCoeff[stage-1][0][1]*D[stage-1][1]; */
        /* pred[1]    = IntraCoeff[stage-1][1][0] * D[stage-1][0] + IntraCoeff[stage-1][1][1]*D[stage-1][1]; */

        pred_fx[0]  = add(mult_r(IntraCoeff_fx[stage-1][0][0], D_fx[stage-1][0]), mult_r(IntraCoeff_fx[stage-1][0][1], D_fx[stage-1][1]));
        move16();
        pred_fx[1]  = add(mult_r(IntraCoeff_fx[stage-1][1][0], D_fx[stage-1][0]), mult_r(IntraCoeff_fx[stage-1][1][1], D_fx[stage-1][1]));
        move16();

        iwd      = add(NTRANS2[branch[stage]+2][state], codeword[stage]);
        /* D[stage][0] = TCVQ_CB_SUB2[stage-2][iwd][0] + pred[0]; */
        /* D[stage][1] = TCVQ_CB_SUB2[stage-2][iwd][1] + pred[1]; */
        D_fx[stage][0] = add(TCVQ_CB_SUB2_fx[stage-2][iwd][0], pred_fx[0]);
        move16();
        D_fx[stage][1] = add(TCVQ_CB_SUB2_fx[stage-2][iwd][1], pred_fx[1]);
        move16();
        state    = NTRANS2[branch[stage]][state];
        move16();
    }

    /* stage #5 - #8 */
    FOR (stage = N_STAGE_VQ-4; stage < N_STAGE_VQ; stage++)
    {
        /* pred[0]    = IntraCoeff[stage-1][0][0] * D[stage-1][0] + IntraCoeff[stage-1][0][1]*D[stage-1][1]; */
        /* pred[1]    = IntraCoeff[stage-1][1][0] * D[stage-1][0] + IntraCoeff[stage-1][1][1]*D[stage-1][1]; */

        pred_fx[0]  = add(mult_r(IntraCoeff_fx[stage-1][0][0], D_fx[stage-1][0]), mult_r(IntraCoeff_fx[stage-1][0][1], D_fx[stage-1][1]));
        move16();
        pred_fx[1]  = add(mult_r(IntraCoeff_fx[stage-1][1][0], D_fx[stage-1][0]), mult_r(IntraCoeff_fx[stage-1][1][1], D_fx[stage-1][1]));
        move16();

        iwd  = add(NTRANS2[branch[stage]+2][state], codeword[stage]);
        /* D[stage][0] = TCVQ_CB_SUB3[stage-4][iwd][0]; */
        /* D[stage][1] = TCVQ_CB_SUB3[stage-4][iwd][1]; */
        D_fx[stage][0] = add(TCVQ_CB_SUB3_fx[stage-4][iwd][0], pred_fx[0]);
        move16();
        D_fx[stage][1] = add(TCVQ_CB_SUB3_fx[stage-4][iwd][1], pred_fx[1]);
        move16();
        state    = NTRANS2[branch[stage]][state];
        move16();
    }

    FOR (stage = 0; stage < N_STAGE_VQ; stage++)
    {
        FOR (i = 0; i < N_DIM; i++)
        {
            /* d_out[(N_DIM*stage) + i] = D[stage][i]; */
            d_out_fx[(N_DIM*stage) + i] = D_fx[stage][i];
            move16();
        }
    }
    return;
}

Word16 qlsf_ARSN_tcvq_Dec_16k_fx (
    Word16 *y_fx,       /* o  : Quantized LSF vector    */
    Word16 *indice,      /* i  : Indices                 */
    const Word16 nBits         /* i  : number of bits          */
)
{
    Word16 i;
    Word16 safety_net;

    Word16 error_svq_q_fx[M];

    /* Select Mode */
    safety_net = indice[0];
    move16();


    IF (sub(safety_net, 1) == 0)
    {
        tcvq_Dec_fx(&indice[1], /*y, */y_fx, safety_net);

        IF (sub(nBits, 30) > 0)
        {
            FOR (i = 0; i < 8; i++)
            {
                error_svq_q_fx[i] = AR_SVQ_CB1_fx[indice[10]][i];
                move16();
                error_svq_q_fx[i+8] = AR_SVQ_CB2_fx[indice[11]][i];
                move16();
            }

            FOR (i = 0; i < M; i++)
            {
                y_fx[i] = add(y_fx[i], error_svq_q_fx[i]);
                move16();
            }
        }
    }
    ELSE
    {
        tcvq_Dec_fx(&indice[1], /*y, */y_fx, safety_net);

        IF (sub(nBits, 30) > 0)
        {
            FOR (i = 0; i < 8; i++)
            {
                error_svq_q_fx[i] = AR_SVQ_CB1_fx[indice[10]][i];
                move16();
                error_svq_q_fx[i+8] = AR_SVQ_CB2_fx[indice[11]][i];
                move16();
            }

            FOR (i = 0; i < M; i++)
            {
                y_fx[i] = add(y_fx[i], error_svq_q_fx[i]);
                move16();
            }
        }
    }

    return safety_net;
}

/*======================================================================*/
/* FUNCTION : lsf_syn_mem_backup_fx                                 */
/*----------------------------------------------------------------------*/
/* PURPOSE : back-up synthesis filter memory and LSF qunatizer memories */
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                            */
/* _ (Word16*) lsp_new           : LSP vector to quantize                */
/* _ (Word16*) lsf_new           : quantized LSF vector                  */
/* _ (Word16*) lsp_mid           :  mid-frame LSP vector                 */
/* _ (Encoder_State_fx) st_fx :  Encoder state Structure                  */
/*-----------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                           */
/* _None                                                                 */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                           */
/* _ (Word16) clip_var     : pitch clipping state var                    */
/* _ (Word16*) mem_AR      : quantizer memory for AR model               */
/* _ (Word16*) mem_MA      : quantizer memory for MA model               */
/* _ (Word16*) lsp_new_bck : LSP vector to quantize- backup              */
/* _ (Word16*) lsf_new_bck : quantized LSF vector - backup               */
/* _ (Word16*) lsp_mid_bck : mid-frame LSP vector - backup               */
/* _ (Word16)     mCb1     :counter for stationary frame after a transition frame */
/* _ (Word32*) Bin_E       : FFT Bin energy 128 *2 sets                  */
/* _ (Word32*) Bin_E_old   : FFT Bin energy 128 *2 sets                  */
/* _ (Word16*) mem_syn_bck : synthesis filter memory                     */
/* _ (Word16)  mem_w0_bck  : memory of the weighting filter              */
/* _ (Word16) streaklimit  : LSF quantizer                               */
/* _ (Word16) pstreaklen   : LSF quantizer                               */
/*-----------------------------------------------------------------------*/

/* _ None                                                                */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                           */
/* _ None                                 */
/*=======================================================================*/

void lsf_syn_mem_backup_fx(
    Encoder_State_fx *st_fx,                  /* i: state structure                                       */
    LPD_state* LPDmem,                /* i: LPD state memory structure                            */
    Word16 *btilt_code_fx,            /* i: tilt code                                             */
    Word32 *gc_threshold_fx,          /* i:                                                       */
    Word16 *clip_var_bck_fx,          /* o:                                                       */
    Word16 *next_force_sf_bck_fx,     /* o:                                                       */

    Word16 *lsp_new,                   /* i: LSP vector to quantize                                */
    Word16 *lsf_new,                   /* i: quantized LSF vector                                  */
    Word16 *lsp_mid,                   /* i: mid-frame LSP vector                                  */
    Word16 *clip_var,                  /* o: pitch clipping state var                              */
    Word16 *mem_AR,                    /* o: quantizer memory for AR model                         */
    Word16 *mem_MA,                    /* o: quantizer memory for AR model                         */
    Word16 *lsp_new_bck,               /* o: LSP vector to quantize- backup                        */
    Word16 *lsf_new_bck,               /* o: quantized LSF vector - backup                         */
    Word16 *lsp_mid_bck,               /* o: mid-frame LSP vector - backup                         */
    Word16 *mCb1,                      /* o: counter for stationary frame after a transition frame */
    Word32 *Bin_E,                     /* o: FFT Bin energy 128 *2 sets                            */
    Word32 *Bin_E_old,                 /* o: FFT Bin energy 128 sets                               */
    Word16 *mem_syn_bck,               /* o: synthesis filter memory                               */
    Word16 *mem_w0_bck,                /* o: memory of the weighting filter                        */
    Word16 *streaklimit,
    Word16 *pstreaklen
)
{
    Word16 i;

    *clip_var = st_fx->clip_var_fx[0];
    move16();

    FOR(i=0; i<M; i++)
    {
        mem_AR[i] = st_fx->mem_AR_fx[i];
        move16();
        mem_MA[i] = st_fx->mem_MA_fx[i];
        move16();
        lsp_new_bck[i] = lsp_new[i];
        move16();
        lsf_new_bck[i] = lsf_new[i];
        move16();
        lsp_mid_bck[i] = lsp_mid[i];
        move16();
    }

    *mCb1 = st_fx->mCb1_fx;
    move16();
    *streaklimit = st_fx->streaklimit_fx;
    move16();
    *pstreaklen = st_fx->pstreaklen_fx;
    move16();

    FOR(i=0; i<L_FFT; i++)
    {
        Bin_E[i]=st_fx->Bin_E_fx[i];
        move32();
    }

    FOR(i=0; i<(L_FFT/2); i++)
    {
        Bin_E_old[i]=st_fx->Bin_E_old_fx[i];
        move32();
    }

    /* back-up memories */
    FOR(i=0; i<M; i++)
    {
        mem_syn_bck[i] = st_fx->LPDmem.mem_syn[i];
        move16();
    }

    *mem_w0_bck = st_fx->LPDmem.mem_w0;
    move16();


    *btilt_code_fx = LPDmem->tilt_code;
    *gc_threshold_fx = LPDmem->gc_threshold;
    Copy( st_fx->clip_var_fx, clip_var_bck_fx, 6 );
    *next_force_sf_bck_fx = st_fx->next_force_safety_net_fx;


    return;
}

void lsf_update_memory(
    Word16 narrowband,         /* i  : narrowband flag                             */
    const Word16 qlsf[],       /* i  : quantized lsf coefficients                  */
    Word16 old_mem_MA[],       /* i  : MA memory                                   */
    Word16 mem_MA[],           /* o  : updated MA memory                           */
    Word16 lpcorder            /* i  : LPC order                                   */
)
{
    Word16 i;

    FOR (i=0; i<lpcorder; ++i)
    {
        move16();
        mem_MA[i] = sub(sub(qlsf[i], lsf_means[narrowband][i]), mult_r(MU_MA_FX, old_mem_MA[i]));
    }

}

/*======================================================================*/
/* FUNCTION : lsf_syn_mem_restore_fx                             */
/*----------------------------------------------------------------------*/
/* PURPOSE :  restore synthesis filter memory and LSF quantizer memories*/
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                             */
/* _ (Word16) clip_var     : pitch clipping state var                  */
/* _ (Word16*) mem_AR      : quantizer memory for AR model             */
/* _ (Word16*) mem_MA      : quantizer memory for MA model             */
/* _ (Word16*) lsp_new_bck : LSP vector to quantize- backup            */
/* _ (Word16*) lsf_new_bck : quantized LSF vector - backup             */
/* _ (Word16*) lsp_mid_bck : mid-frame LSP vector - backup             */
/* _ (Word16)     mCb1     :counter for stationary frame after a transition frame */
/* _ (Word32*) Bin_E       : FFT Bin energy 128 *2 sets                  */
/* _ (Word32*) Bin_E_old   : FFT Bin energy 128 *2 sets                  */
/* _ (Word16*) mem_syn_bck : synthesis filter memory                     */
/* _ (Word16)  mem_w0_bck  : memory of the weighting filter              */
/* _ (Word16) streaklimit  : LSF quantizer                               */
/* _ (Word16) pstreaklen   : LSF quantizer                               */
/*-----------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                           */
/* _None                                                                 */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                           */
/* _ (Word16*) lsp_new           : LSP vector to quantize                */
/* _ (Word16*) lsf_new           : quantized LSF vector                  */
/* _ (Word16*) lsp_mid           :  mid-frame LSP vector                 */
/* _ (Encoder_State_fx) st_fx :  Encoder state Structure                  */
/*-----------------------------------------------------------------------*/

/* _ None                                                                */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                           */
/* _ None                                 */
/*=======================================================================*/
void lsf_syn_mem_restore_fx(
    Encoder_State_fx *st_fx,                        /* o: state structure                                        */
    LPD_state* LPDmem,                /* o: LPD_state vewctor                                      */
    Word16 btilt_code_fx,                 /* i:                                                       */
    Word32 gc_threshold_fx,           /* i:                                                       */
    Word16 *clip_var_bck_fx,              /* i:                                                       */
    Word16 next_force_sf_bck_fx,          /* i:                                                       */

    Word16 *lsp_new,                   /* o: LSP vector to quantize                                 */
    Word16 *lsf_new,                   /* o: quantized LSF vector                                   */
    Word16 *lsp_mid,                   /* o: mid-frame LSP vector                                   */
    Word16 clip_var,                   /* i: pitch clipping state var                               */
    Word16 *mem_AR,                    /* i: quantizer memory for AR model                          */
    Word16 *mem_MA,                    /* i: quantizer memory for MA model                          */
    Word16 *lsp_new_bck,               /* i: LSP vector to quantize- backup                         */
    Word16 *lsf_new_bck,               /* i: quantized LSF vector - backup                          */
    Word16 *lsp_mid_bck,               /* i: mid-frame LSP vector - backup                          */
    Word16 mCb1,                       /* i: counter for stationary frame after a transition frame  */
    Word32 *Bin_E,                     /* i: FFT Bin energy 128 *2 sets                             */
    Word32 *Bin_E_old,                 /* i: FFT Bin energy 128 sets                                */
    Word16 *mem_syn_bck,               /* i: synthesis filter memory                                */
    Word16 mem_w0_bck,                 /* i: memory of the weighting filter                         */
    Word16 streaklimit,                /* i:LSF quantizer                                           */
    Word16 pstreaklen                  /* i:LSF quantizer                                           */
)
{
    Word16 i;

    /* restore lsf memories */
    st_fx->clip_var_fx[0] = clip_var;
    move16();

    FOR(i=0; i<M; i++)
    {
        st_fx->mem_AR_fx[i] = mem_AR[i];
        move16();
        st_fx->mem_MA_fx[i] = mem_MA[i];
        move16();
        lsp_new[i] = lsp_new_bck[i];
        move16();
        lsf_new[i] = lsf_new_bck[i];
        move16();
        lsp_mid[i] = lsp_mid_bck[i];
        move16();
    }

    st_fx->mCb1_fx = mCb1;
    move16();
    st_fx->streaklimit_fx = streaklimit;
    move16();
    st_fx->pstreaklen_fx = pstreaklen;
    move16();

    FOR(i=0; i<L_FFT; i++)
    {
        st_fx->Bin_E_fx[i] = Bin_E[i];
        move16();
    }

    FOR(i=0; i<(L_FFT/2); i++)
    {
        st_fx->Bin_E_old_fx[i]=Bin_E_old[i];
        move32();
    }

    /* restoring memories */
    st_fx->LPDmem.mem_w0 = mem_w0_bck;
    move16();

    FOR(i=0; i<M; i++)
    {
        st_fx->LPDmem.mem_syn[i] = mem_syn_bck[i];
        move16();
    }

    move16();
    move32();
    move16();
    LPDmem->tilt_code = btilt_code_fx;
    LPDmem->gc_threshold = gc_threshold_fx;
    Copy( clip_var_bck_fx, st_fx->clip_var_fx, 6 );
    st_fx->next_force_safety_net_fx = next_force_sf_bck_fx;

    return;
}

/* Returns: codebook index */
Word16 tcxlpc_get_cdk(
    Word16 acelp_ext_mode         /* (I) GC/VC indicator   */
)
{
    Word16 cdk;

    move16();
    cdk = 0;
    if ( sub(acelp_ext_mode, VOICED) == 0 )
    {
        cdk = 1;
        move16();
    }

    return cdk;
}

void msvq_dec
(
    const Word16 *const*cb,  /* i  : Codebook (indexed cb[*stages][levels][p]) (14Q1*1.28)*/
    const Word16 dims[],     /* i  : Dimension of each codebook stage (NULL: full dim.)   */
    const Word16 offs[],     /* i  : Starting dimension of each codebook stage (NULL: 0)  */
    const Word16 stages,/* i  : Number of stages                                     */
    const Word16 N,     /* i  : Vector dimension                                     */
    const Word16 maxN,  /* i  : Codebook dimension                                   */
    const Word16 Idx[], /* i  : Indices                                              */
    Word16 *uq          /* o  : quantized vector                          (14Q1*1.28)*/
)
{
    Word16 i,j,offset;
    Word16 N34;
    Word16 n, maxn, start;



    set16_fx(uq, 0, N);

    FOR ( i=0; i<stages; i++ )
    {
        n = N;
        move16();
        maxn = maxN;
        move16();
        if (dims)
        {
            n = dims[i];
            move16();
        }
        if (dims)
        {
            maxn = n;
            move16();
        }

        assert((maxn % 4) == 0);
        N34 = mult(maxn, FL2WORD16(0.75));

        start = 0;
        move16();
        if (offs)
        {
            start = offs[i];
            move16();
        }

        /*vr_add( uq+start, cb[i]+Idx[i]*maxn, uq+start, n );, where uq = a zero vector*/
        offset = i_mult2(Idx[i],N34);

        FOR (j=0; j<n; j+=4)
        {
            Word16 val0, val1, val2, val3;

            depack_4_values(cb[i]+offset+3*(j>>2), val0, val1, val2, val3)

            uq[start+j+0] = add(uq[start+j+0], val0);
            move16();  /*14Q1*1.28*/
            uq[start+j+1] = add(uq[start+j+1], val1);
            move16();  /*14Q1*1.28*/
            uq[start+j+2] = add(uq[start+j+2], val2);
            move16();  /*14Q1*1.28*/
            uq[start+j+3] = add(uq[start+j+3], val3);
            move16();  /*14Q1*1.28*/
        }
    }


    return;
}

/*
 * E_LPC_lsp_lsf_conversion
 *
 * Parameters:
 *    lsp            I: lsp[m] (range: -1 <= val < 1)               Q15
 *    lsf            O: lsf[m] normalized (range: 0 <= val <= 6400)
 *    m              I: LPC order
 *
 * Function:
 *    Transformation lsp to lsf
 *
 *    LSP are line spectral pair in cosine domain (-1 to 1).        (0Q15)
 *    LSF are line spectral pair in frequency domain (0 to 6400).
 *
 * Returns:
 *    energy of prediction error
 */
void E_LPC_lsp_lsf_conversion(const Word16 lsp[], Word16 lsf[], const Word16 m)
{
    Word16 i;

    assert( m==16 || m==10 );



    FOR (i = 0; i < m; i++)
    {
        lsf[i] = xsp_to_xsf(lsp[i]);
        move16();
    }


    return;
}

/*
 * E_LPC_lsf_lsp_conversion
 *
 * Parameters:
 *    lsf            I: lsf[m] normalized (range: 0 <= val <= 0.5)  x2.56
 *    lsp            O: lsp[m] (range: -1 <= val < 1)        Q15
 *    m              I: LPC order
 *
 * Function:
 *    Transformation lsf to lsp
 *
 *    LSF are line spectral pair in frequency domain (0 to 6400).
 *    LSP are line spectral pair in cosine domain (-1 to 1).
 *
 * Returns:
 *    void
 */
void E_LPC_lsf_lsp_conversion(const Word16 lsf[], Word16 lsp[], const Word16 m)
{
    Word16 i;

    assert( m==16 || m==10 );

    /* convert ISFs to the cosine domain */
    FOR (i = 0; i < m; i++)
    {
        *lsp++ = xsf_to_xsp(*lsf++);
        move16();
    }


    return;
}

/*==========================================================================*/
/* FUNCTION      : void sub_lsp2lsf_fx ()                                        */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :                                                            */
/*  * Transformation of LSPs to LSFs                                        */
/*  * LSP are line spectral pair in cosine domain (-1 to 1).                */
/*  * LSF are line spectral frequencies (0 to fs/2).                        */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                        */
/* Word16 lsp_i      i  : lsp[m] (range: -1<=val<1)                       Q15  */
/* Word16 m          i  : LPC order                                        Q0  */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/* Word32 lsf[]      o  : lsf[m] normalized (range: 0.0<=val<=0.5)  Q(x2.56)<<16 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                    */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                        */
/*                     _ None                                                    */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                                                             */
/*==========================================================================*/
Word32 sub_lsp2lsf_fx(
    const Word16 lsp_i     /* i  : lsp[m] (range: -1<=val<1)                       Q15*/
)
{
    Word16 ind, tmp;
    Word32 L_tmp;

    /*------------------------------------------------------*
     * find value in table that is just greater than lsp[i]
     *------------------------------------------------------*/

    /* Retrieve Index Guess */
    /* Based on lsp[i] */
    ind = mac_r(GUESS_TBL_SZ/2*65536-0x8000, lsp_i, GUESS_TBL_SZ/2);
    ind = Ind_Guess[ind];
    move16();

    /* Correct Index so that */
    /*  table[ind] > lsp[i]  */
    tmp = sub(lsp_i, table[ind]);
    /*
       69%: (Final Index - Index Guess) is <= 1
       28%: (Final Index - Index Guess) is 2
        2%: (Final Index - Index Guess) is >= 3
       <1%: ...
    */
    IF (tmp > 0) /* possible range 0 to -5 (-1-2-2) */
    {
        ind = sub(ind, 1);
        tmp = sub(lsp_i, table[ind]);
        IF  (tmp > 0)
        {
            ind = sub(ind, 2);
            tmp = sub(lsp_i, table[ind]);
            if (tmp > 0)
            {
                ind = sub(ind, 2);
            }
            tmp = sub(lsp_i, table[ind+1]);
            if (tmp <= 0)
            {
                ind = add(ind, 1);
            }
            tmp = sub(lsp_i, table[ind]);
        }
    }

    /* acos(lsp[i])= ind*128 + (lsp[i]-table[ind]) * slope[ind] / 2048 */
    L_tmp = L_mac(1L<<11, tmp, slope[ind]);
    L_tmp = L_shr(L_tmp, 12);  /* (lsp[i]-table[ind]) * slope[ind]) >> 11 */
    L_tmp = L_mac0(L_tmp, ind, 128);

    return L_tmp;
}

/*===================================================================*/
/* FUNCTION : compute_poly_product_fx () */
/*-------------------------------------------------------------------*/
/* PURPOSE : Compute polynomial product for P(z) for LSP */
/* to LPC conversion */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16[]) coef : LSP coefficients, Q15 */
/* _ (Word16 []) order: LPC order */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word32[]) p : output sequence, Q24 */
/* 1st entry is always 1.0, Q24 */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*===================================================================*/
/* NOTE: */
/* P(z)=(1+z^-1)*PI(j=1to(order/2)) (1 - 2*z^-1*cos(2*pi*w(2j-1)) + z^-2) */
/* Q(z)=(1-z^-1)*PI(j=1to(order/2)) (1 - 2*z^-1*cos(2*pi*w(2j)) + z^-2) */
/*===================================================================*/
void compute_poly_product_fx(
    Word16* coef,   /* i : LSP coefficients, Q15 */
    Word32* p,      /* o : output sequence, Q24 */
    Word16 order    /* i : order */
)
{
    Word16 i, len;
    Word32 lspcos[LPC_SHB_ORDER];
    Word32 p2[LPC_SHB_ORDER]; /* intermediate product, Q24*/
    Word32* p_in, *p_out, *p_temp;

    FOR ( i = 0; i < order / 2; i++ )
    {
        lspcos[i] = poscos_fx( coef[i * 2] );
        move32(); /* lspcos =-cos(lsp) in Q30*/
    }

    /* Set up first polynomial for convolution (1 -2cos(w5) 1) */
    /* First element of output is 1, all in Q24 */
    p[0] = L_deposit_h( 0x100 ); /* 1.0 in Q24 */
    p[2] = L_deposit_h( 0x100 ); /* 1.0 in Q24 */
    p[1] = L_shr( lspcos[order / 2 - 1], 5 ); /* p2[1]=-2cos(w5), Q24 */
    p2[0] = L_deposit_h( 0x100 ); /* 1.0 in Q24 */

    len = 1;
    move16();

    len = 1;
    p_in = p + 1;  /* 2nd entry of input sequence */
    p_out = p2 + 1;  /* 2nd entry of output sequence */
    FOR ( i = 0; i < ( order / 2 - 1 ); i++ )
    {
        lsp_convolve_fx( L_shr( lspcos[i], 5 ), p_in, p_out, len );

        p_temp = p_in;
        p_in = p_out;
        p_out = p_temp; /* swap input/output buffer */
        len = add( len, 1 );
    }

    /* if((order/2 - 1)%2 != 0) */
    IF ( s_and( ( order / 2 - 1 ), 1 ) != 0 )
    {
        FOR ( i = 1; i <= order / 2; i++ )
        {
            p[i] = p_in[i - 1];
            move32();
        }
    }

}

/*===================================================================*/
/* FUNCTION : lsp_convolve_fx ( ) */
/*-------------------------------------------------------------------*/
/* PURPOSE : Convolution of LSP polynomial for LSP to LPC */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ ( Word32[]) p1 : 2nd entry of input sequence, Q24 */
/* 1st entry is 1.0, Q24 */
/* _ ( Word32) x : -2cos( w) in Q24 */
/* _ ( Word16) len: length of output-2 */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ ( Word32[]) p2 : 2nd entry of output sequence, Q24 */
/* 1st entry is 1.0, Q24 */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*===================================================================*/
/* NOTE: Convolves ( 1 S 1) where S is the -2cos( w) value) with the */
/* long sequence where the input long sequence on consecutive calls */
/* is ( 1 X1 1), ( 1 X1 X2 X1), ( 1 X1 X2 X3 X2), ( 1 X1 X2 X3 X4 X3). */
/* ( Since the sequence is symmetric, we only need to output one */
/* entry beyond the center point for the next iteration). */
/* The 1st entry of the convolution is 1, the 2nd is X1+S1, and */
/* the rest have the form X( i-1) + X( i)*S + X( i+1). */
/* Final output sequence is ( 1 X1 X2 X3 X4 X5 X4) */
/*===================================================================*/
void lsp_convolve_fx(
    Word32 x,
    Word32* p1,
    Word32* p2,
    Word16 len
)
{
    Word16 i, d1h, d1l, d2h, d2l;
    Word32 Ltemp;
    Word32 Lacc;

    d1h = extract_h( x ); /*  d1h in Q8 */
    d1l = extract_l( x ); /*  d1l in Q24 */

    Ltemp = L_add( x, p1[0] ); /* first output is p1[0]+x, Q24 */

    p2[0] = Ltemp;
    move32();
    Ltemp = L_deposit_h( 0x100 ); /*  Ltemp=1.0, Q24 */
    FOR ( i = 0; i < len; i++ )
    {
        Ltemp = L_add( Ltemp, p1[i + 1] ); /*  Ltemp2=p1[i-1]+p1[i+1], Q24 */
        d2h = extract_h( p1[i] );
        d2l = extract_l( p1[i] );

        /*  Lacc=L_mult_su( d1h,d2l); */
        Lacc = L_mult0( d1h, ( UWord16 )d2l );
        Lacc = L_mac0( Lacc, d2h, d1l );
        Lacc = L_add( L_shr( Lacc, 16 ), L_shr( L_mult( d1h, d2h ), 1 ) );
        /*  Lacc=p1[i]* x, Q16 */
        Lacc = L_add( L_shl( Lacc, 8 ), Ltemp ); /* Lacc=p1[i-1]+p1[i+1]+p1[i]*x, Q24 */

        p2[i + 1] = Lacc;
        move32();

        Ltemp = L_add(0,p1[i]);
    } /*  end for */

    p2[i + 1] = p2[i - 1];
    move32();
}



/*===================================================================*/
/* FUNCTION : poscos_fx ( ) */
/*-------------------------------------------------------------------*/
/* PURPOSE : Compute cosine by approximation */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* */
/* _ ( Word16) w: angle in radians/pi, Q14 ( 0<=w<=16384) */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ ( Word32) : -cos( w) in Q30 */
/*===================================================================*/
/* NOTE: This function uses 4 coefficients in the approx formula */
/* Approx Formula: cos( w) = z + c1*z^3 + c2*z^5 + c3*z^7 + c4*z^9 */
/* where z = pi/2 - w, c1=-.1666665668, c2=.8333025139E-2, */
/* c3=-.198074182E-3, and c4=.2601903036E-5 */
/*===================================================================*/


Word32 poscos_fx( Word16 w )
{
    Word16 a, z, z2, z3, z5, z7, z9;
    Word32 Ltemp;
    Word32 Lacc;

    IF ( w == 0 )
    {
        return ( 0xc0000004 ); /*  -1.0 in Q30 */
    }
    ELSE
    {
        z = 0x2000;
        move16(); /*  z=0.5 in Q14 */
        z = shl( sub( z, w ), 2 ); /*  z = 0.5-w in Q16 */

        a = 0x6488;
        move16(); /* a=pi in Q13 */
        z = mult_r( z, a ); /*  z=pi*( 0.5-w), Q14 */
        z2 = mult_r( z, z ); /*  z2=z^2, Q13 */
        z3 = round_fx( L_shl( L_mult0( z, z2 ), 0 ) ); /*  z3=z^3, Q11 */
        z5 = mult_r( z2, z3 ); /*  z5=z^5, Q9 */
        z7 = round_fx( L_shl( L_mult0( z2, z5 ), 0 ) ); /*  z7=z^7, Q6 */
        z9 = round_fx( L_shl( L_mult0( z2, z7 ), 0 ) ); /* z9=z^9, Q3 */

        Lacc = L_mult( z9, cos_coef_new[0] ); /* Lacc=c4*z^9, Q31, c4 in Q28 */
        Lacc = L_mac0( Lacc, z7, cos_coef_new[1] ); /*  c3 in Q25 */
        Ltemp = L_shl( L_mult( z3, cos_coef_new[3] ), 1 ); /*  Ltemp=c1*z^3, Q31, c1 in Q17 */
        Lacc = L_add( L_shr( Lacc, 1 ), Ltemp ); /*  Lacc=c1*z^3+c3*z^7+c4*^z9, Q30 */
        Lacc = L_add( Lacc, L_deposit_h( z ) );
        Ltemp = L_mult( z5, cos_coef_new[2] ); /*  Ltemp=-c2*z^5, Q29, cos_coef[2]=-c2 in Q19 */
        Lacc = L_sub( Lacc, L_shl( Ltemp, 1 ) ); /*  Lacc=cos( w) in Q30 */

        return L_negate( Lacc ); /*  return -cos( w), Q30 */
    } /*  end else */
}
/*===================================================================*/
/* FUNCTION : root_search_fx ( ) */
/*-------------------------------------------------------------------*/
/* PURPOSE : Search root of the P or Q polynomial in a given */
/* interval using binary search */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ ( Word16) low : Low index of the interval, Q9 ( 0-511) */
/* _ ( Word16) high : High index of the interval, Q9 ( 0-511) */
/* _ ( Word32 []) coef: polynomial coefficients, Q26 */
/* _ ( Word16) order : LPC order */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ ( Word32) v_low: Polynomial value at low index, Q25 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ ( Word16) root in Q15, or 0xffff if no root is found */
/*===================================================================*/

Word16 root_search_fx( Word16 low,
                       Word16 high,
                       Word32* v_low,
                       Word32* coef,
                       Word16 order
                     )
{
    Word16 f;
    Word32 v_high, vh;
    Word32 Ltemp, L_tmp1, L_tmp, Ltmp;
    Word16 exp1, tmp;

    v_high = polynomial_eval_fx( high, coef, order ); /*  v_high has the value at high index */

    IF ( v_high != 0 )
    {
        /*  No exact root found */
        test();
        IF ( ( v_high ^ ( *v_low ) ) >= 0 )
        {
            /*  No root in this interval ( low high] */
            *v_low = v_high;
            move32();
            return ( -1 );
        }
        ELSE
        {
            /*  there is a root in this interval */
            /* Do binary search for root */
            vh = v_high;
            move16();

            WHILE ( sub( sub( high, low ), 2 ) >= 0 )
            {
                /*  high-low>=2 */
                f = shr( add( high, low ), 1 ); /*  f=( high+low)/2 */
                Ltemp = polynomial_eval_fx( f, coef, order );
                IF ( Ltemp == 0 )
                {
                    *v_low = v_high;
                    move32();/*  Set low value for next root search call */
                    return ( shl( f, 6 ) );
                }
                ELSE
                {
                    test();
                    IF ( ( Ltemp ^ ( *v_low ) ) < 0 )
                    {
                        /*  root between f & low */
                        high = f;
                        move16();
                        vh = L_add(0,Ltemp);
                    }
                    ELSE
                    {
                        *v_low = Ltemp;
                        move32();
                        low = f;
                        move16();
                    }
                }
            } /*  end while */

            /* do interpolation for root in between high and low */
            /*  Lacc=divide_dp( *v_low, L_sub( *v_low,vh),2,1); // Lacc in Q31 */
            L_tmp = L_sub( *v_low, vh );

            if ( L_sub( *v_low, vh ) < 0 )
            {
                L_tmp = L_negate( L_tmp );
            }

            exp1 = norm_l( L_tmp );
            L_tmp1 = L_shl( L_tmp, exp1 );
            tmp = extract_h( L_tmp1 );
            exp1 = sub(30 - 25, exp1);
            tmp = div_s( 16384, tmp );              /* 15+exp1 */
            Ltmp = Mult_32_16( *v_low, tmp );       /* 15+exp1+25-15 */
            Ltemp = L_shl( Ltmp, ( 6 - exp1 ) );    /* Q31 */
            if ( L_sub( *v_low, vh ) < 0 )
            {
                Ltemp = L_negate( Ltemp );
            }
            Ltemp = L_shr( Ltemp, 9 );              /*  Ltemp =quotient*1.0 in Q31 */

            *v_low = v_high;
            move16();
            return ( add( round_fx( Ltemp ), shl( low, 6 ) ) );
        } /*  end else ( root in interval) */
    } /*  end else ( exact root at high) */

    /*  find the exact root */
    *v_low = v_high;
    move32(); /*  Set low value for next root search call */
    return ( shl( high, 6 ) ); /*  return exact root at high in Q15 */
}

/*===================================================================*/
/* FUNCTION : calc_weight( ) */
/*-------------------------------------------------------------------*/
/* PURPOSE : This function computes the weight given delta1
and delta2 */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16) delta1: (Q15) */
/* _ (Word16) delta2: (Q15) */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word16 *) n1: o/p weight is Q(31-n1) */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None. */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ (Word32) Lsum1: computed weight Q(31-n1) */
/* (alpha/(sqrt(delta1*delta2)) */
/*===================================================================*/

Word32 calc_weight(
    Word16 delta1,
    Word16 delta2,
    Word16* n1
)
{
    Word16 n;
    Word32 L_tmp;
    Word16 alpha=0x4F94;
    move16(); /*  ( 0.5*250/( 2*pi)) in Q10*/

    L_tmp = L_mult0( delta1, delta2 ); /* Q30 */
    n = norm_l( L_tmp );
    L_tmp = L_shl( L_tmp, n );
    n = sub(1, n);
    L_tmp = Isqrt_lc( L_tmp, &n ); /* Q( 31-n)*/

    L_tmp = Mult_32_16( L_tmp, alpha ); /* Q( 26-n) */

    *n1 = n;
    move16();

    return L_tmp;
}

/*===================================================================*/
/* FUNCTION : polynomial_eval_fx ( ) */
/*-------------------------------------------------------------------*/
/* PURPOSE : Evaluate P or Q polynomial at given frequency */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16) f : frequency used as index for cosine table lookup */
/* Q9 */
/* _ (Word32 []) coef: polynomial coefficients, Q26 */
/* _ (Word16 []) order: LPC order */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ (Word32) polynomial value at given frequency f, Q25 */
/*===================================================================*/
/* Note: This function uses 512 entry cosine table cos_table[], Q15 */
/*===================================================================*/
/* if (K = order/2) */
/* P(w)=cos(Kw)+p[1]*cos((K-1)w)+p[2]*cos((K-2)w)+...+p[(K-1)]*cos(w)+p[K]/2 */
/* Q(w)=cos(Kw)+q[1]*cos((K-1)w)+q[2]*cos((K-2)w)+...+q[(K-1)]*cos(w)+q[K]/2 */
/*===================================================================*/

/* 40-32 bit conversion */
Word32 polynomial_eval_fx( Word16 f,
                           Word32* coef,
                           Word16 order
                         )
{
    Word16 i, idx;
    Word16 dh, dl, coslut;
    Word32 Ltemp, L_tmp1, L_tmp;
    Word32 Lacc;
    idx = f;
    move16();
    dh = extract_h( coef[order / 2] ); /* Q10 */
    dl = extract_l( coef[order / 2] ); /* Q16 */

    coslut = 0x4000;
    move16(); /*  coslut=0.5 in Q15 */
    Ltemp = L_mult0( coslut, dh ); /*  Ltemp=MSW, coef[5]/2 in Q25 */

    Lacc = L_mult0( coslut, dl ); /*  Lacc=LSW, coef[5]/2 in Q41 //Q31 */
    Lacc = L_shr( Lacc, 1 ); /* Q30 */
    FOR ( i = ( order / 2 ) - 1; i > 0; i-- )
    {
        coslut = cos_table[idx];
        move16(); /*  coslut=cos( ( ( order/2)-i)f), Q15 */
        dh = extract_h( coef[i] );/* Q10 */
        dl = extract_l( coef[i] );/* Q16 */

        Ltemp = L_add( Ltemp, L_mult0( dh, coslut ) ); /* Q25 */
        IF ( dl < 0 )
        {
            L_tmp1 = L_shl( L_add( 65536, dl ), 14 );
            L_tmp = Mult_32_16( L_tmp1, coslut );
            Lacc = L_add( Lacc, L_tmp );
        }
        ELSE
        {
            Lacc = L_add( Lacc, L_shr( L_mult0( coslut, dl ), 1 ) );/* Q30 */
        }

        idx += f;
        if ( sub(idx, 512) >= 0 ) idx = sub(idx, 512); /*  modulo of 512 */
    }

    coslut = cos_table[idx];
    move16(); /*  coslut=cos( 5f), Q15 */
    Ltemp = L_add( Ltemp, L_mult0( 0x400, coslut ) ); /*  coef[0]=1.0, Q25 */

    return ( L_add( Ltemp, L_shr( Lacc, 15 ) ) ); /* Q25 */
}

/*----------------------------------------------------------------------------------*
*  v_sort:
*
*  This is very fast with almost ordered vectors. The first 15 ISF values
*  are almost always sorted.
*----------------------------------------------------------------------------------
*
* Source (G718):
*  v_sort_fx (isf_nokia_fx.c)
*
*/
void v_sort(
    Word16 *r,    /* i/o: Vector to be sorted in place */
    const Word16 lo,    /* i  : Low limit of sorting range   */
    const Word16 up     /* I  : High limit of sorting range  */
)
{
    Word16 i, j;
    Word16 tempr;


    FOR (i=sub(up, 1); i>=lo; i--)
    {
        tempr = r[i];
        move16();
        FOR (j=i; j<up; j++)
        {
            IF (sub(tempr, r[j+1]) <= 0)
            {
                BREAK;
            }
            r[j] = r[j+1];
            move16();
        }
        r[j] = tempr;
        move16();
    }


    return;
}
