/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"

#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

/* The conversion modes. */
#define DOWNCONV   0
#define UPCONV     1
#define NC         (M/2)

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/
static void powerspect_fx(
    const Word16 x[],        /* i: Q15 Grid points x[0:m-1]             */
    Word16 N,                /* i: Number of grid points                */
    Word32  R[],             /* i: Q20 Coefficients of R(x) in R[0:NC]  */
    Word32  S[],             /* i: Q20 Coefficients of S(x) in S[0:NC]  */
    Word32  G[],             /* o: Q15 Power spectrum G[0:N]            */
    Word16 mode              /* i: Flag for up or down conversion       */
);

static void spectautocorr_fx(
    const Word16 x[],        /* i: Grid points x[0:m-1]             */
    const Word16 N,          /* i: Number of grid points            */
    const Word32 G[],        /* i: Power spectrum G[0:N-1]          */
    Word16 rh[],             /* o: Autocorrelation r[0:M]           */
    Word16 rl[]              /* o: Autocorrelation r[0:M]           */
);

static Word32 b_inv_sq(
    const Word32 in32,      /* i  : Input not normalized to inverse */
    const Word16 exp_in     /* I  : input current exponent          */
);

static Word32 inv_pow(
    const Word32 re,
    const Word32 se,
    const Word16 x

);

static void zeros2poly_fx( Word16 x[], Word32 R[], Word32 S[] );

static void polydecomp_fx(
    Word16 A[],            /* i: Q12 linear prediction coefficients */
    Word32 P[],            /* o: Q22 coefficients of R(x)           */
    Word32 Q[]             /* o: Q22 coefficients of S(x)           */
);

static void cheb2poly_fx(
    Word32 L_P[]       /* i/o Q22: The coefficients of C(x) and P(x) */
);

/*---------------------------------------------------------------------*
 *  lsp_convert_poly()
 *
 *  Converts the LP filter estimated at 16.0 kHz sampling rate down
 *  12.8 kHz frequency scale or alternatively from 12.8 kHz up to
 *  16.0 kHz.  The former is called down conversation and latter up
 *  conversion.  The resulting LP filter is characterized with its
 *  line spectrum pairs.  The original Lp filter can be either in
 *  its immittance, used for the AMR-WB IO mode, or line spectrum
 *  pair representation.
 *
 *  The conversion is based the autocorrelation computed from the
 *  power spectrum of the LP filter that is truncated or extrapolated
 *  to the desired frequency scale.
 *---------------------------------------------------------------------*/

Word16 lsp_convert_poly_fx(
    Word16  w[],           /* i/o: LSP or ISP parameters          */
    const Word16 L_frame,        /* i  : flag for up or down conversion */
    const Word16 Opt_AMRWB       /* i  : flag for the AMR-WB IO mode    */
)
{
    const Word16 N50 = GRID50_POINTS;
    const Word16 N40 = GRID40_POINTS;
    Word16 flag;

    Word32 G[GRID50_POINTS];
    Word16 i;
    Word16 A[M+1];
    Word32 R[NC+1], S[NC+1];
    Word32 epsP[M+1];
    Word16 rh[M+1], rl[M+1];
    Word16 oldA[M+3];

    /*---------------------------------------------------------------------*
     * Because AMR-WB IO mode uses immittance spectrum frequency representation
     * instead of line spectrum frequency representation, the input
     * parameters do not give the zeros of the polynomials R(x) and S(x).
     * Hence R(x) and S(x) are formed via the polynomial A(z) of the linear
     * prediction filter.
     *---------------------------------------------------------------------*/

    IF( Opt_AMRWB )
    {
        E_LPC_f_isp_a_conversion( w, oldA, M );

        polydecomp_fx( oldA, R, S );
    }

    /*---------------------------------------------------------------------*
     * Form the polynomials R(x) and S(x) from their zeros that are the
     * line spectrum pairs of A(z).  The polynomial coefficients can be
     * scaled for convenience, because scaling will not affect the
     * resulting LP coefficients.  Scaling by 128 gives the correct offset
     * to the power spectrum for n = 16.
     *---------------------------------------------------------------------*/

    ELSE
    {
        E_LPC_f_lsp_a_conversion( w, oldA, M );

        zeros2poly_fx(w, R, S);
    }

    /*---------------------------------------------------------------------*
     * Conversion from 16.0 kHz down to 12.8 kHz.  The power spectrum
     * needs to be computed only up to 6.4 kHz, because the upper band
     * is omitted.
     *---------------------------------------------------------------------*/

    IF (sub(L_frame,L_FRAME) == 0)
    {
        powerspect_fx(grid50_fx, N50, R, S, G, DOWNCONV);
        spectautocorr_fx(grid40_fx, N40, G, rh, rl);
    }
    /*---------------------------------------------------------------------*
     * Conversion from 12.8 kHz up to 16.0 kHz.
     * Compute the power spectrum of the LP filter, extrapolate the
     * power spectrum from 6.4 kHz to 8.0 kHz, and compute auto-
     * correlation on this power spectrum.
     *---------------------------------------------------------------------*/

    ELSE
    {
        powerspect_fx(grid40_fx, N40, R, S, G, UPCONV);

        FOR (i = N40; i < N50; i++)
        {
            G[i] = G[N40-1];
            move32();
        }

        spectautocorr_fx(grid50_fx, N50, G, rh, rl);
    }


    /*---------------------------------------------------------------------*
     * Compute the linear prediction coefficients from the autocorrelation
     * and convert to line spectrum pairs.
     *---------------------------------------------------------------------*/
    flag=E_LPC_lev_dur(rh, rl, A, epsP, M, oldA);
    E_LPC_a_lsp_conversion(A, w, stable_LSP_fx, M );


    return(flag);
}

/*---------------------------------------------------------------------*
 *  powerspect()
 *
 *  Computes the power spectrum G(w) = 1/|A(w)|^2 at N points on
 *  the real axis x = cos w by utilizing the line spectrum frequency
 *  decomposition
 *
 *     A(z) = (P(z) + Q(z))/2,
 *
 *  where assuming A(z) of an even degree n,
 *
 *     P(z) = [A(z) + z^(n+1) A(1/z)]/(1/z + 1),
 *     Q(z) = [A(z) - z^(n+1) A(1/z)]/(1/z - 1).
 *
 *  The zeros of these polynomials give the line spectrum frequencies
 *  of A(z).  It can be shown that for an even n,
 *
 *     |A(x)|^2 = 2 (1 + x) R(x)^2 + 2 (1 - x) S(x)^2,
 *
 *  where x = cos w, and R(x) and S(x) are the direct polynomials
 *  resulting from the Chebyshev series representation of P(z)
 *  and Q(z).
 *
 *  This routine assumes the grid X = 1, x[0], x[1], .., x[m-1],
 *  -, ..., -x[1], -x[0], -1 such that x[i] = cos((i+1)*pi/N) for
 *  evaluating the power spectrum.  Only m = (N-1)/2 - 1 grid points
 *  need to be stored, because cos(0) and cos(pi/2) are trivial,
 *  and the points above pi/2 are obtained readily using the symmetry
 *  of cosine.
 *
 *  The power spectrum can be scaled as a*G[], where a is chosen
 *  for convenience. This is because the scaling has no impact on
 *  the LP coefficients to be determined based on the power spectrum.
 *---------------------------------------------------------------------*/

void powerspect_fx(
    const Word16 x[],        /* i: Q15 Grid points x[0:m-1]             */
    Word16 N,                /* i: Number of grid points                */
    Word32  R[],             /* i: Q20 Coefficients of R(x) in R[0:NC]  */
    Word32  S[],             /* i: Q20 Coefficients of S(x) in S[0:NC]  */
    Word32  G[],             /* o: Q15 Power spectrum G[0:N]            */
    Word16 mode              /* i: Flag for up or down conversion       */
)
{
    Word32 s0, se, so, r0, re, ro;
    Word16 i, j;
    Word16 iuni, imid;
    Word32 L_tmp;
    Word16 x2;
    Word32 mh;
    UWord16 ml;

    /*---------------------------------------------------------------------*
     * Down conversion yields iuni unique grid points that do not have
     * symmetric counterparts above x = cos(pi/2) = 0.
     * Set the mid point of the frequency grid.
     *---------------------------------------------------------------------*/

    IF (mode == DOWNCONV)
    {
        iuni = (GRID50_POINTS - 1)/5 - 1;
        move16();
        imid = (GRID50_POINTS - 1)/2;
        move16();
    }

    /*---------------------------------------------------------------------*
    * Power spectrum x = cos(pi) = -1 that is not needed in down
    * conversion. Set the mid point of the frequency grid.
    *---------------------------------------------------------------------*/

    ELSE
    {
        iuni = 0;
        move16();
        imid = (GRID40_POINTS - 1)/2;
        move16();

        G[N-1] = S[0];
        move32();

        FOR (j = 1; j <= NC; j++)
        {
            G[N-1] = L_sub(S[j], G[N-1]);
            move32();
        }
        G[N-1] = b_inv_sq(G[N-1], 19);
        move32();
    }

    /*---------------------------------------------------------------------*
     * Power spectrum x = cos(0) = 1.
     *---------------------------------------------------------------------*/

    G[0] = R[0];
    move32();
    FOR (j = 1; j <= NC; j++)
    {
        G[0] = L_add(R[j], G[0]);
        move32();
    }

    G[0] = b_inv_sq(max(G[0],1), 19);
    move32();

    /*---------------------------------------------------------------------*
     * Power spectrum at x = cos(pi/2) = 0.
     *---------------------------------------------------------------------*/
    G[imid] = inv_pow(R[NC], S[NC], 0);
    move32();

    /*---------------------------------------------------------------------*
     * Power spectrum at unique points that do not have symmetric
     * counterparts at x > cos(pi/2) = 0.
     *---------------------------------------------------------------------*/

    FOR (i = 1; i <= iuni; i++)
    {
        Mpy_32_16_ss(R[0], x[i-1], &mh, &ml);
        r0 = L_add(R[1], mh);

        Mpy_32_16_ss(S[0], x[i-1], &mh, &ml);
        s0 = L_add(S[1], mh);


        FOR (j = 2; j <= NC; j++)
        {
            Mpy_32_16_ss(r0, x[i-1], &mh, &ml);
            r0 = L_add(R[j], mh);

            Mpy_32_16_ss(s0, x[i-1], &mh, &ml);
            s0 = L_add(S[j], mh);
        }

        G[i] = inv_pow(r0, s0, x[i-1]);
        move32();
    }

    /*---------------------------------------------------------------------*
     * Power spectrum at points other than x = -1, 0, and 1 and unique
     * points is computed using the anti-symmetry of the grid relative
     * to the midpoint x = 0 in order to reduce looping.
     *---------------------------------------------------------------------*/

    FOR ( ; i < imid; i++)
    {
        x2 = mult_r(x[i-1], x[i-1]);

        Mpy_32_16_ss(R[0], x2, &mh, &ml);
        re = L_add(R[2], mh);
        Mpy_32_16_ss(R[1], x2, &mh, &ml);
        ro = L_add(R[3], mh);

        Mpy_32_16_ss(S[0], x2, &mh, &ml);
        se = L_add(S[2], mh);
        Mpy_32_16_ss(S[1], x2, &mh, &ml);
        so = L_add(S[3], mh);

        FOR (j = 4; j < NC; j+=2)
        {
            Mpy_32_16_ss(re, x2, &mh, &ml);
            re = L_add(R[j], mh);
            Mpy_32_16_ss(ro, x2, &mh, &ml);
            ro = L_add(R[j+1], mh);
            Mpy_32_16_ss(se, x2, &mh, &ml);
            se = L_add(S[j], mh);
            Mpy_32_16_ss(so, x2, &mh, &ml);
            so = L_add(S[j+1], mh);
        }

        Mpy_32_16_ss(re, x2, &mh, &ml);
        L_tmp = L_add(R[j], mh);
        Mpy_32_16_ss(ro, x[i-1], &mh, &ml);
        re = L_add(L_tmp, mh);
        ro = L_sub(L_tmp, mh);

        Mpy_32_16_ss(se, x2, &mh, &ml);
        L_tmp = L_add(S[j], mh);
        Mpy_32_16_ss(so, x[i-1], &mh, &ml);
        se = L_add(L_tmp, mh);
        so = L_sub(L_tmp, mh);

        G[i]     = inv_pow(re, se, x[i-1]);
        move32();
        G[N-i-1] = inv_pow(so, ro, x[i-1]);
        move32();
    }

    return;
}

static Word32 b_inv_sq(
    const Word32 in32,      /* i  : Input not normalized to inverse */
    const Word16 exp_in     /* i  : input current exponent          */
)
{
    Word16 m_den, exp_den;
    Word16 div_out;
    Word32 Ltmp;

    exp_den = norm_l(in32);
    m_den = extract_h(L_shl(in32, exp_den));
    exp_den = add(sub(30,exp_den),sub(16,exp_in));

    m_den = mult_r(m_den, m_den);
    exp_den = shl(exp_den,1);

    div_out = div_s(8192,m_den);
    Ltmp = L_shl(div_out, add(sub(30-13, exp_den),15));  /*Q15*/

    return Ltmp;
}

static Word32 inv_pow(
    const Word32 re,
    const Word32 se,
    const Word16 x
)
{
    Word16 exp1, exp2;
    Word16 tmp;
    Word32 L_tmp;
    Word32 mh;
    UWord16 ml;
    Word32 r0, s0;

    IF(re==0)
    {
        exp1 = 30;
        move16();
        r0 = L_deposit_l(0);
    }
    ELSE
    {
        exp1 = norm_l(re);
        tmp = extract_h(L_shl(re, exp1));
        L_tmp = L_shr(L_mult(tmp, tmp), 1);
        Mpy_32_16_ss(L_tmp, x, &mh, &ml);
        r0 = L_add(L_tmp, mh);
    }

    IF(se==0)
    {
        exp2 = 30;
        move16();
        s0 = L_deposit_l(0);
    }
    ELSE
    {
        exp2 = norm_l(se);
        tmp = extract_h(L_shl(se, exp2));
        L_tmp = L_shr(L_mult(tmp, tmp), 1);
        Mpy_32_16_ss(L_tmp, x, &mh, &ml);
        s0 = L_sub(L_tmp, mh);
    }

    IF(exp1 > exp2)
    {
        exp1 = shl(sub(exp1, exp2), 1);
        r0 = L_shr(r0, exp1);

        exp2 = add(add(exp2, exp2), 8);
    }
    ELSE
    {
        exp2 = shl(sub(exp2, exp1), 1);
        s0 = L_shr(s0, exp2);

        exp2 = add(add(exp1, exp1), 8);
    }

    r0 = L_add(r0, s0);
    exp1 = norm_l(r0);
    L_tmp = L_shl(r0, exp1);
    tmp = extract_h(L_tmp);
    IF(tmp==0)
    {
        return MAX_32;
    }
    tmp = div_s((Word16)((1<<14)-1), tmp);
    exp1 = add(exp1, exp2);
    L_tmp = L_shr(tmp, sub(31, exp1)); /* result in Q15 */

    return(L_tmp);
}


/*---------------------------------------------------------------------*
 *  spectautocorr()
 *
 *  Computes the autocorrelation r[j] for j = 0, 1, ..., M from
 *  the power spectrum P(w) by using rectangle rule to approximate
 *  the integral
 *
 *             1     pi
 *     r[j] = ---    I  P(w) cos(j*w) dw.
 *            2*pi -pi
 *
 *  It is sufficient to evaluate the integrand only from w = 0 to
 *  w = pi due to the symmetry P(-w) = P(w).  We can further
 *  employ the relation
 *
 *      cos(j*(pi - w)) = (-1)^j cos(j*w)
 *
 *  to use symmetries relative to w = pi/2.
 *
 *  When applying the rectangle rule, it is useful to separate w = 0,
 *  w = pi/2, and w = pi.  By using a frequency grid of N points, we
 *  can express the rectangle rule as
 *
 *     r[j] = G[0] + 2*a*G[(N-1)/2] + b*G[N-1]
 *
 *                      M
 *                 + 2 sum (G[i] - G[N-i-1]) cos(j*x[i])
 *                     i=1
 *
 *  where G[i] is the power spectrum at the grid point cos(i*pi/N)
 *  and M = (N-1)/2 - 1 is the number of the grid points in the
 *  interval(0, pi/2).
 *
 *  The coefficients
 *
 *     b = (-1)^j
 *     a = (1 + (-1)^(j+1))(-1)^floor(j/2)
 *
 *  follow from the properties of cosine.  The computation further
 *  uses the recursion
 *
 *     cos(j*w) = 2*cos(w)*cos((j-1)*w) - cos((j-2)*w)
 *
 *  Note that the autocorrelation can be scaled for convenience,
 *  because this scaling has no impact on the LP coefficients to be
 *  calculated from the autocorrelation. The expression of r[j] thus
 *  omits the division by N.
 *
 *  See the powerspect function on the definition of the grid.
 *
 *  References
 *  J. Makhoul, "Spectral linear prediction: properties and
 *  applications," IEEE Trans. on Acoustics, Speech and Signal
 *  Processing, Vol. 23, No. 3, pp.283-296, June 1975
 *---------------------------------------------------------------------*/

static void spectautocorr_fx(
    const Word16 x[],        /* i: Grid points x[0:m-1]             */
    const Word16 N,          /* i: Number of grid points            */
    const Word32 G[],        /* i: Power spectrum G[0:N-1]          */
    Word16 rh[],             /* o: Autocorrelation r[0:M]           */
    Word16 rl[]              /* o: Autocorrelation r[0:M]           */
)
{
    Word16 c[M+1];          /* c[j] = cos(j*w) */
    Word32 gp, gn;
    Word16 i, j;
    Word16 imid;
    Word32 mh;
    UWord16 ml;
    Word32 r[M+1];
    Word16 exp0;

    /*---------------------------------------------------------------------*
     * The mid point of the cosine table x of m entries assuming an odd m.
     * Only the entries x[0] = cos(pi/m), x[1] = cos(2*pi/m), ...,
     * x[imid-1] = cos((imid-1)*pi/m) need to be stored due to trivial
     * cos(0), cos(pi/2), cos(pi), and symmetry relative to pi/2.
     * Here m = 51.
     *---------------------------------------------------------------------*/

    imid = (N - 1)/2;
    move16();

    /*---------------------------------------------------------------------*
     * Autocorrelation r[j] at zero lag j = 0 for the upper half of the
     * unit circle, but excluding the points x = cos(0) and x = cos(pi).
     *---------------------------------------------------------------------*/

    r[0] = G[1];
    move32();
    FOR (i = 2; i < N-1; i++)
    {
        r[0] = L_add(r[0], G[i]);
        move32();
    }

    /*---------------------------------------------------------------------*
     * Initialize the autocorrelation r[j] at lags greater than zero
     * by adding the midpoint x = cos(pi/2) = 0.
     *---------------------------------------------------------------------*/

    r[1] = L_deposit_l(0);
    r[2] = -G[imid];
    move32();

    FOR (i = 3; i < M; i+=2)
    {
        r[i]   =  L_deposit_l(0);
        r[i+1] = -r[i-1];
        move32();
    }

    /*---------------------------------------------------------------------*
     * Autocorrelation r[j] at lags j = 1, 2, ..., M.  The computation
     * employes the relation cos(j*(pi - w)) = (-1)^j cos(j*w) and
     * cos(j*w) = 2*cos(w)*cos((j-1)*w) - cos((j-2)*w) for obtaining
     * the cosine c[j] = cos(j*w).
     *---------------------------------------------------------------------*/

    c[0] = (Word16)32767;
    move16();     /* 1.0 in Q15 */
    FOR (i = 1; i < imid; i++)
    {
        gp = L_add(G[i],  G[N-i-1]);
        gn = L_sub(G[i],  G[N-i-1]);

        /*r[1] = L_mac(r[1], x[i-1], gn);*/
        Mpy_32_16_ss(gn, x[i-1], &mh, &ml);
        r[1] = L_add(r[1], mh);
        move32();
        c[1] = x[i-1];
        move16();

        FOR (j = 2; j < M; j+=2)
        {
            c[j] = mult_r(c[j-1], x[i-1]);
            move16();
            c[j] = add(c[j], sub(c[j], c[j-2]));
            move16();

            /*r[j] = L_mac(r[j], c[j], gp);*/
            Mpy_32_16_ss(gp, c[j], &mh, &ml);
            r[j] = L_add(r[j], mh);
            move32();

            c[j+1] = mult_r(c[j], x[i-1]);
            move16();
            c[j+1] = add(c[j+1], sub(c[j+1], c[j-1]));
            move16();

            /*r[j+1] = L_mac(r[j+1], c[j+1], gn);*/
            Mpy_32_16_ss(gn, c[j+1], &mh, &ml);
            r[j+1] = L_add(r[j+1], mh);
            move32();
        }
        c[j] = mult_r(c[j-1], x[i-1]);
        move16();
        c[j] = add(c[j], sub(c[j], c[j-2]));
        move16();

        Mpy_32_16_ss(gp, c[j], &mh, &ml);
        r[j] = L_add(r[j], mh);
        move32();
    }

    /*---------------------------------------------------------------------*
     * Add the endpoints x = cos(0) = 1 and x = cos(pi) = -1 as
     * well as the lower half of the unit circle.
     *---------------------------------------------------------------------*/
    gp = L_shr(L_add(G[0], G[N-1]), 1);
    gn = L_shr(L_sub(G[0], G[N-1]), 1);

    r[0]= L_add(r[0], gp);
    move32();
    exp0 = norm_l(r[0]);
    L_Extract(L_shl(r[0], exp0), &rh[0], &rl[0]);

    FOR (j = 1; j < M; j+=2)
    {
        L_Extract(L_shl(L_add(r[j], gn), exp0), &rh[j], &rl[j]);
        L_Extract(L_shl(L_add(r[j+1], gp), exp0), &rh[j+1], &rl[j+1]);
    }

    return;
}

/*---------------------------------------------------------------------*
 *  zeros2poly()
 *
 *  Computes the coefficients of the polynomials
 *
 *      R(x) =   prod  (x - x[i]),
 *             i = 0,2,4,...
 *
 *      S(x) =   prod  (x - x[i]),
 *             i = 1,3,5,...
 *
 *  when their zeros x[i] are given for i = 0, 1, ..., n-1. The
 *  routine assumes n = 1 or even n greater than or equal to 4.
 *
 *  The polynomial coefficients are returned in R[0:n/2-1] and
 *  S[0:n/2-1]. The leading coefficients are in R[0] and S[0].
 *---------------------------------------------------------------------*/
static void zeros2poly_fx(
    Word16 x[],           /* i: Q15  Zeros of R(x) and S(x)      */
    Word32 R[],           /* o: Q22  Coefficients of R(x)        */
    Word32 S[]            /* o: Q22  Coefficients of S(x)        */
)
{
    Word16  xr, xs;
    Word16  i, j;
    Word32  mh;
    UWord16  ml;

    R[0] = (1<<27)-1;
    move32();
    S[0] = (1<<27)-1;
    move32();
    R[1] = L_msu(0, x[0], 1<<11);
    move32();
    S[1] = L_msu(0, x[1], 1<<11);
    move32();

    FOR (i = 2; i <= NC; i++)
    {
        xr = negate(x[2*i-2]);
        xs = negate(x[2*i-1]);

        Mpy_32_16_ss(R[i-1], xr, &R[i], &ml);
        Mpy_32_16_ss(S[i-1], xs, &S[i], &ml);

        FOR (j = i-1; j > 0; j--)
        {
            Mpy_32_16_ss(R[j-1], xr, &mh, &ml);
            R[j] = L_add(R[j], mh);
            Mpy_32_16_ss(S[j-1], xs, &mh, &ml);
            S[j] = L_add(S[j], mh);
        }
    }

    return;
}

/*---------------------------------------------------------------------*
 *  polydecomp()
 *
 *  Computes the coefficients of the symmetric and antisymmetric
 *  polynomials P(z) and Q(z) that define the line spectrum pair
 *  decomposition of a given polynomial A(z) of order n. For even n,
 *
 *      P(z) = [A(z) + z^(n+1) A(1/z)]/(1/z + 1),
 *      Q(z) = [A(z) - z^(n+1) A(1/z)]/(1/z - 1),
 *
 *  These polynomials are then expressed in their direct form,
 *  respectively, R(x) and S(x), on the real axis x = cos w using
 *  explicit Chebyshev polynomials of the first kind.
 *
 *  The coefficients of the polynomials R(x) and S(x) are returned
 *  in R[0:n/2] and S[0:n/2] for the given linear prediction
 *  coefficients A[0:n/2]. Note that R(x) and S(x) are formed in
 *  place such that P(z) is stored in the same array than R(x),
 *  and Q(z) is stored in the same array than S(x).
 *
 *  The routines assumes n = 16.
 *---------------------------------------------------------------------*/

static void polydecomp_fx(
    Word16 A[],            /* i: Q12 linear prediction coefficients */
    Word32 R[],            /* o: Q20 coefficients of R(x)           */
    Word32 S[]             /* o: Q20 coefficients of S(x)           */
)
{
    Word16 scale;
    Word16 i;
    Word32 Ltmp1, Ltmp2;

    scale = shl((1<<5), norm_s(A[0]));

    R[0] = (1<<20)-1;
    move32(); /* Q20 */
    S[0] = (1<<20)-1;
    move32();

    FOR(i=0; i<NC; i++)
    {
        Ltmp1 = L_mult(A[i+1], scale);

        Ltmp2 = L_msu(Ltmp1, A[M-i], scale);
        Ltmp1 = L_mac(Ltmp1, A[M-i], scale);

        R[i+1] = L_sub(Ltmp1, R[i]);
        move32();
        S[i+1] = L_add(Ltmp2, S[i]);
        move32();
    }

    cheb2poly_fx(R);
    cheb2poly_fx(S);

    return;
}

/*---------------------------------------------------------------------*
 *  cheb2poly_fx()
 *
 *  Computes the coefficients of the explicit Chebyshev polynomial
 *  P(x) = P[0]*x^n + P[1]*x^(n-1) + ... + P[n] given the coefficients
 *  of the series
 *
 *     C(x) = C[0]*T_n(x) + C[1]*T_n-1(x) + ... + C[n]*T_0(x)
 *
 *  where T_n(x) is the nth Chebyshev polynomial of the first kind.
 *  This implementation assumes C[0] = 1. Only value n = 8 is
 *  supported.
 *
 *  The conversion from C(x) to P(x) is done in place such that the
 *  coefficients of C(x) are given in P[0:8] and those of P(x) are
 *  returned in the same array.
 *---------------------------------------------------------------------*/

static void cheb2poly_fx(
    Word32 L_P[]       /* i/o Q20: The coefficients of C(x) and P(x) */
)
{
    Word32 L_C[NC+1], L_tmp;
    Word16 i;

    FOR(i=1; i<=NC; i++)
    {
        L_C[i] = L_P[i];
        move32();
    }

    L_P[0] = (1<<27)-1;
    move32();

    L_P[1] = L_shl(L_C[1], 6);
    move32();  /*  64.0*C[1] */
    L_P[8] = L_shl(L_C[1], 3);
    move32();  /*   8.0*C[1] */

    L_P[5] = L_sub(L_P[1], L_P[8]);
    move32();  /*  56.0*C[1] */

    L_P[2] = L_shl(L_C[3], 2);
    move32();
    L_tmp = L_add(L_C[3], L_P[2]);                             /*   5.0*C[3] */
    L_P[7] = L_sub(L_tmp, L_sub(L_P[8], L_C[1]));
    move32();  /*  -7.0*C[1] */

    L_P[8] = L_shl(L_C[3], 4);
    move32();  /*  16.0*C[3] */
    L_P[3] = L_sub(L_P[8], L_shl(L_P[5], 1));
    move32();  /*-112.0*C[1] */

    L_P[5] = L_sub(L_P[5], L_add(L_P[8], L_P[2]));
    move32();  /* -20.0*C[3] */

    L_P[2] = L_shl(L_C[5], 2);
    move32();
    L_P[5] = L_add(L_P[5], L_P[2]);
    move32();  /*   4.0*C[5] */

    L_tmp = L_sub(L_P[7], L_sub(L_P[2], L_C[5]));              /*  -3.0*C[5] */
    L_P[7] = L_add(L_tmp, L_C[7]);
    move32();  /*       C[7] */

    L_P[6] = L_shl(L_C[2], 4);
    move32();
    L_tmp = L_sub((160<<20), L_P[6]);
    L_P[4] = L_sub(L_tmp, L_shl(L_C[2], 5));
    move32();  /* -48.0*C[2] */

    L_tmp = L_add(L_P[6], L_shl(L_C[2], 1));                   /*  18.0*C[2] */
    L_P[6] = L_sub(L_tmp, (32<<20));
    move32();

    L_P[8] = L_shl(L_C[4], 3);
    move32();
    L_P[4] = L_add(L_P[4], L_P[8]);
    move32();  /*   8.0*C[4] */

    L_tmp = L_sub(L_P[6], L_P[8]);                             /*  -8.0*C[4] */
    L_P[6] = L_add(L_tmp, L_shl(L_C[6], 1));
    move32();  /*   2.0*C[6] */

    L_tmp = L_shl(L_C[2], 5);                                  /*  32.0*C[2] */
    L_P[2] = L_sub(L_tmp, (256<<20));
    move32();

    L_tmp = L_add((1<<21)+1, L_C[8]);
    L_tmp = L_shr(L_tmp, 1);                        /* 1+0.5*C[8] */
    L_tmp = L_sub(L_tmp, L_C[2]);
    L_tmp = L_add(L_tmp, L_C[4]);
    L_P[8] = L_sub(L_tmp, L_C[6]);
    move32();

    return;
}
