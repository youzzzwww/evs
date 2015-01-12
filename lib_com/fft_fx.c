/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*-----------------------------------------------------------------*
* Local functions
*-----------------------------------------------------------------*/

#define FFT3_ONE_THIRD   21845  /* 1/3 in Q16 */

static void fft5_shift4_16fx( Word16   n1, Word16 *zRe, Word16 *zIm, const Word16 *Idx );
static void fft64_16fx( Word16 *x, Word16 *y, const Word16 *Idx );
static void fft32_5_16fx( Word16 *x, Word16 *y, const Word16 *Idx );
static void cftmdl_16fx(Word16 n, Word16 l, Word16 *a, const Word32 *w);
static void cftfsub_16fx( Word16 n, Word16 *a, const Word32 *w );
static void cft1st_16fx(Word16 n, Word16 *a, const Word32 *w);
static void cftmdl_16fx(Word16 n, Word16 l, Word16 *a, const Word32 *w);
static void fft5_shift4_16fx( Word16   n1, Word16 *zRe, Word16 *zIm, const Word16 *Idx );
static void bitrv2_SR_16fx( Word16 n, const Word16 *ip, Word16 *a );
static void fft64_16fx( Word16 *x, Word16 *y, const Word16 *Idx );
static void fft5_32_16fx( Word16 *zRe, Word16 *zIm, const Word16 *Idx );
static void cdftForw_16fx( Word16 n, Word16 *a, const Word16 *ip, const Word32 *w );

#include "math_32.h"

/*-----------------------------------------------------------------*
* Local functions
*-----------------------------------------------------------------*/
static void cdftForw_fx( Word16 n, Word32 *a, const Word16 *ip, const Word16 *w );
static void bitrv2_SR_fx( Word16 n, const Word16 *ip, Word32 *a );
static void cftfsub_fx( Word16 n, Word32 *a, const Word16 *w );
static void cft1st_fx( Word16 n, Word32 *a, const Word16 *w );
static void cftmdl_fx( Word16 n, Word16 l, Word32 *a, const Word16 *w );


void DoRTFTn_fx(
    Word32 *x,    /* i/o : real part of input and output data       */
    Word32 *y,     /* i/o : imaginary part of input and output data  */
    const Word16 n /* i : size of the FFT up to 1024 */
)
{

    Word16 i;
    Word32 z[2048], *pt;

    pt = z;
    FOR ( i=0; i<n; i++ )
    {
        *pt++ = x[i];
        move16();
        *pt++ = y[i];
        move16();
    }

    IF (sub(n, 4) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft4_fx,w_fft4_fx);
    }
    ELSE IF (sub(n, 8) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft8_fx,w_fft8_fx);
    }
    ELSE IF (sub(n, 16) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft16_fx,w_fft16_fx);
    }
    ELSE IF (sub(n, 32) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft32_fx,w_fft32_fx);
    }
    ELSE IF (sub(n, 64) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft64_fx,w_fft64_fx);
    }
    ELSE IF (sub(n, 128) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft128_fx,w_fft128_fx);
    }
    ELSE IF (sub(n, 256) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft256_fx,w_fft256_fx);
    }
    ELSE IF (sub(n, 512) == 0)
    {
        cdftForw_fx(2*n,z,Ip_fft512_fx,w_fft512_fx);
    }

    x[0]=z[0];
    move16();
    y[0]=z[1];
    move16();
    pt = &z[2];
    FOR( i=n-1; i>=1 ; i--)
    {
        x[i]=*pt++;
        move16();
        y[i]=*pt++;
        move16();
    }

    return;
}

/*-----------------------------------------------------------------*
 * cdftForw_fx()
 * Main fuction of Complex Discrete Fourier Transform
 *-----------------------------------------------------------------*/
static void cdftForw_fx(
    Word16 n,    /* i    : data length of real and imag				 */
    Word32 *a,   /* i/o  : input/output data                     Q(q)*/
    const Word16 *ip,  /* i    : work area for bit reversal				 */
    const Word16 *w    /* i    : cos/sin table						  Q14*/
)
{
    /* bit reversal */
    bitrv2_SR_fx(n, ip + 2, a);

    /* Do FFT */
    cftfsub_fx(n, a, w);
}

/*-----------------------------------------------------------------*
 * bitrv2_SR_fx()
 * Bit reversal
 *-----------------------------------------------------------------*/
static void bitrv2_SR_fx(
    Word16 n,     /* i    : data length of real and imag			  */
    const Word16 *ip,   /* i/o  : work area for bit reversal				  */
    Word32 *a     /* i/o  : input/output data                     Q(q)*/
)
{
    Word16 j, j1, k, k1, m, m2;
    Word16 l;
    Word32 xr, xi, yr, yi;

    l = n;
    move16();
    m = 1;
    move16();

    WHILE (shl(m, 3) < l)
    {
        l = shr(l, 1);
        m = shl(m, 1);
    }

    m2 = shl(m, 1);
    IF (shl(m, 3) == l)
    {
        FOR (k = 0; k < m; k++)
        {
            FOR (j = 0; j < k; j++)
            {
                j1 = add(shl(j, 1), ip[k]);
                k1 = add(shl(k, 1), ip[j]);
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
                j1 = add(j1, m2);
                k1 = add(k1, shl(m2, 1));
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
                j1 = add(j1, m2);
                k1 = sub(k1, m2);
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
                j1 = add(j1, m2);
                k1 = add(k1, shl(m2, 1));
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
            }

            j1 = add(add(shl(k, 1), m2), ip[k]);
            k1 = add(j1, m2);
            xr = L_add(0,a[j1]);
            xi = L_add(0,a[j1 + 1]);
            yr = L_add(0,a[k1]);
            yi = L_add(0,a[k1 + 1]);
            a[j1] = yr;
            move32();
            a[j1 + 1] = yi;
            move32();
            a[k1] = xr;
            move32();
            a[k1 + 1] = xi;
            move32();
        }
    }
    ELSE
    {
        FOR (k = 1; k < m; k++)
        {
            FOR (j = 0; j < k; j++)
            {
                j1 = add(shl(j, 1), ip[k]);
                k1 = add(shl(k, 1), ip[j]);
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
                j1 = add(j1, m2);
                k1 = add(k1, m2);
                xr = L_add(0,a[j1]);
                xi = L_add(0,a[j1 + 1]);
                yr = L_add(0,a[k1]);
                yi = L_add(0,a[k1 + 1]);
                a[j1] = yr;
                move32();
                a[j1 + 1] = yi;
                move32();
                a[k1] = xr;
                move32();
                a[k1 + 1] = xi;
                move32();
            }
        }
    }

    return;
}

/*-----------------------------------------------------------------*
 * cftfsub_fx()
 * Complex Discrete Fourier Transform
 *-----------------------------------------------------------------*/
static void cftfsub_fx(
    Word16 n,     /* i    : data length of real and imag			  */
    Word32 *a,    /* i/o  : input/output data                     Q(q)*/
    const Word16 *w     /* i    : cos/sin table                          Q14*/
)
{
    Word16 j, j1, j2, j3, l;
    Word32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

    l = 2;
    move16();

    IF (n > 8)
    {
        cft1st_fx(n, a, w);
        l = 8;
        move16();
        WHILE ((shl(l, 2) < n))
        {
            cftmdl_fx(n, l, a, w);
            l = shl(l, 2);
        }
    }
    IF (shl(l, 2) == n)
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j, l);
            j2 = add(j1, l);
            j3 = add(j2, l);
            x0r = L_add(a[j], a[j1]);
            x0i = L_add(a[j + 1], a[j1 + 1]);
            x1r = L_sub(a[j], a[j1]);
            x1i = L_sub(a[j + 1], a[j1 + 1]);
            x2r = L_add(a[j2], a[j3]);
            x2i = L_add(a[j2 + 1], a[j3 + 1]);
            x3r = L_sub(a[j2], a[j3]);
            x3i = L_sub(a[j2 + 1], a[j3 + 1]);
            a[j] = L_add(x0r, x2r);
            move32();
            a[j2] = L_sub(x0r, x2r);
            move32();
            a[j + 1] = L_add(x0i, x2i);
            move32();
            a[j2 + 1] = L_sub(x0i, x2i);
            move32();
            a[j1] = L_sub(x1r, x3i);
            move32();
            a[j1 + 1] = L_add(x1i, x3r);
            move32();
            a[j3] = L_add(x1r, x3i);
            move32();
            a[j3 + 1] = L_sub(x1i, x3r);
            move32();
        }
    }
    ELSE
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j, l);
            x0r = L_sub(a[j], a[j1]);
            x0i = L_sub(a[j + 1], a[j1 + 1]);
            a[j] = L_add(a[j], a[j1]);
            move32();
            a[j + 1] = L_add(a[j + 1], a[j1 + 1]);
            move32();
            a[j1] = x0r;
            move32();
            move32();
            a[j1 + 1] = x0i;
            move32();
            move32();
        }
    }

    return;
}

/*-----------------------------------------------------------------*
 * cft1st_fx()
 * Subfunction of Complex Discrete Fourier Transform
 *-----------------------------------------------------------------*/
static void cft1st_fx(
    Word16 n,     /* i    : data length of real and imag              */
    Word32 *a,    /* i/o  : input/output data                     Q(q)*/
    const  Word16 *w     /* i    : cos/sin table                          Q14*/
)
{
    Word16 j, k1, k2;
    Word16 wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
    Word32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

    x0r = L_add(a[0], a[2]);
    x0i = L_add(a[1], a[3]);
    x1r = L_sub(a[0], a[2]);
    x1i = L_sub(a[1], a[3]);
    x2r = L_add(a[4], a[6]);
    x2i = L_add(a[5], a[7]);
    x3r = L_sub(a[4], a[6]);
    x3i = L_sub(a[5], a[7]);
    a[0] = L_add(x0r, x2r);
    move32();
    a[1] = L_add(x0i, x2i);
    move32();
    a[4] = L_sub(x0r, x2r);
    move32();
    a[5] = L_sub(x0i, x2i);
    move32();
    a[2] = L_sub(x1r, x3i);
    move32();
    a[3] = L_add(x1i, x3r);
    move32();
    a[6] = L_add(x1r, x3i);
    move32();
    a[7] = L_sub(x1i, x3r);
    move32();

    wk1r = w[2];
    move16();
    x0r = L_add(a[8], a[10]);
    x0i = L_add(a[9], a[11]);
    x1r = L_sub(a[8], a[10]);
    x1i = L_sub(a[9], a[11]);
    x2r = L_add(a[12], a[14]);
    x2i = L_add(a[13], a[15]);
    x3r = L_sub(a[12], a[14]);
    x3i = L_sub(a[13], a[15]);
    a[8] = L_add(x0r, x2r);
    move32();
    a[9] = L_add(x0i, x2i);
    move32();
    a[12] = L_sub(x2i, x0i);
    move32();
    a[13] = L_sub(x0r, x2r);
    move32();

    x0r = L_sub(x1r, x3i);
    x0i = L_add(x1i ,x3r);
    a[10] = Mult_32_16(L_shl(L_sub(x0r, x0i), 1), wk1r);
    move32();
    a[11] = Mult_32_16(L_shl(L_add(x0r, x0i), 1), wk1r);
    move32();
    x0r = L_add(x3i, x1r);
    x0i = L_sub(x3r,x1i);
    a[14] = Mult_32_16(L_shl(L_sub(x0i, x0r), 1), wk1r);
    move32();
    a[15] = Mult_32_16(L_shl(L_add(x0i, x0r), 1), wk1r);
    move32();

    k1 = 0;
    move16();
    FOR (j = 16; j < n; j += 16)
    {
        k1 = add(k1, 2);
        k2 = shl(k1, 1);
        wk2r = w[k1];
        move16();
        wk2i = w[k1 + 1];
        move16();
        wk1r = w[k2];
        move16();
        wk1i = w[k2 + 1];
        move16();
        wk3r = extract_l(L_sub(L_deposit_l(wk1r), L_shr(L_mult(wk2i, wk1i), 14)));
        wk3i = extract_l(L_msu0(L_shr(L_mult(wk2i, wk1r), 14), wk1i, 1));
        x0r = L_add(a[j], a[j + 2]);
        x0i = L_add(a[j + 1], a[j + 3]);
        x1r = L_sub(a[j], a[j + 2]);
        x1i = L_sub(a[j + 1], a[j + 3]);
        x2r = L_add(a[j + 4], a[j + 6]);
        x2i = L_add(a[j + 5], a[j + 7]);
        x3r = L_sub(a[j + 4], a[j + 6]);
        x3i = L_sub(a[j + 5], a[j + 7]);
        a[j] = L_add(x0r, x2r);
        move32();
        a[j + 1] = L_add(x0i, x2i);
        move32();
        x0r = L_sub(x0r, x2r);
        x0i = L_sub(x0i, x2i);
        a[j + 4] = L_sub(Mult_32_16(L_shl(x0r, 1), wk2r), Mult_32_16(L_shl(x0i, 1), wk2i));
        move32();
        a[j + 5] = L_add(Mult_32_16(L_shl(x0i, 1), wk2r), Mult_32_16(L_shl(x0r, 1), wk2i));
        move32();
        x0r = L_sub(x1r, x3i);
        x0i = L_add(x1i, x3r);
        a[j + 2] = L_sub(Mult_32_16(L_shl(x0r, 1), wk1r), Mult_32_16(L_shl(x0i, 1), wk1i));
        move32();
        a[j + 3] = L_add(Mult_32_16(L_shl(x0i, 1), wk1r), Mult_32_16(L_shl(x0r, 1), wk1i));
        move32();
        x0r = L_add(x1r, x3i);
        x0i = L_sub(x1i, x3r);
        a[j + 6] = L_sub(Mult_32_16(L_shl(x0r, 1), wk3r), Mult_32_16(L_shl(x0i, 1), wk3i));
        move32();
        a[j + 7] = L_add(Mult_32_16(L_shl(x0i, 1), wk3r), Mult_32_16(L_shl(x0r, 1), wk3i));
        move32();

        wk1r = w[k2 + 2];
        move16();
        wk1i = w[k2 + 3];
        move16();
        wk3r = extract_l(L_sub(L_deposit_l(wk1r), L_shr(L_mult(wk2r, wk1i), 14)));
        wk3i = extract_l(L_msu0(L_shr(L_mult(wk2r, wk1r), 14), wk1i, 1));
        x0r = L_add(a[j + 8], a[j + 10]);
        x0i = L_add(a[j + 9], a[j + 11]);
        x1r = L_sub(a[j + 8], a[j + 10]);
        x1i = L_sub(a[j + 9], a[j + 11]);
        x2r = L_add(a[j + 12], a[j + 14]);
        x2i = L_add(a[j + 13], a[j + 15]);
        x3r = L_sub(a[j + 12], a[j + 14]);
        x3i = L_sub(a[j + 13], a[j + 15]);
        a[j + 8] = L_add(x0r, x2r);
        move32();
        a[j + 9] = L_add(x0i, x2i);
        move32();
        x0r = L_sub(x0r, x2r);
        x0i = L_sub(x0i, x2i);
        a[j + 12] = L_negate(L_add(Mult_32_16(L_shl(x0r, 1), wk2i), Mult_32_16(L_shl(x0i, 1), wk2r)));
        move32();
        a[j + 13] = L_sub(Mult_32_16(L_shl(x0r, 1), wk2r), Mult_32_16(L_shl(x0i, 1), wk2i));
        move32();
        x0r = L_sub(x1r, x3i);
        x0i = L_add(x1i, x3r);
        a[j + 10] = L_sub(Mult_32_16(L_shl(x0r, 1), wk1r), Mult_32_16(L_shl(x0i, 1), wk1i));
        move32();
        a[j + 11] = L_add(Mult_32_16(L_shl(x0i, 1), wk1r), Mult_32_16(L_shl(x0r, 1), wk1i));
        move32();
        x0r =L_add(x1r, x3i);
        x0i =L_sub(x1i, x3r);
        a[j + 14] = L_sub(Mult_32_16(L_shl(x0r, 1), wk3r), Mult_32_16(L_shl(x0i, 1), wk3i));
        move32();
        a[j + 15] = L_add(Mult_32_16(L_shl(x0i, 1), wk3r), Mult_32_16(L_shl(x0r, 1), wk3i));
        move32();
    }

    return;
}

/*-----------------------------------------------------------------*
 * cftmdl_fx()
 * Subfunction of Complex Discrete Fourier Transform
 *-----------------------------------------------------------------*/
static void cftmdl_fx(
    Word16 n,     /* i    : data length of real and imag			   */
    Word16 l,     /* i    : initial shift for processing			   */
    Word32 *a,    /* i/o  : input/output data              Q(Qx+Q_edct)*/
    const  Word16 *w     /* i    : cos/sin table							Q30*/
)
{
    Word16 j, j1, j2, j3, k, k1, k2, m, m2;
    Word16 wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
    Word32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
    Word16 tmp;

    m = shl(l, 2);
    FOR (j = 0; j < l; j += 2)
    {
        j1 = add(j, l);
        j2 = add(j1, l);
        j3 = add(j2, l);
        x0r = L_add(a[j], a[j1]);
        x0i = L_add(a[j + 1], a[j1 + 1]);
        x1r = L_sub(a[j], a[j1]);
        x1i = L_sub(a[j + 1], a[j1 + 1]);
        x2r = L_add(a[j2], a[j3]);
        x2i = L_add(a[j2 + 1], a[j3 + 1]);
        x3r = L_sub(a[j2], a[j3]);
        x3i = L_sub(a[j2 + 1], a[j3 + 1]);
        a[j] = L_add(x0r, x2r);
        move32();
        a[j + 1] = L_add(x0i, x2i);
        move32();
        a[j2] = L_sub(x0r, x2r);
        move32();
        a[j2 + 1] =L_sub(x0i, x2i);
        move32();
        a[j1] = L_sub(x1r, x3i);
        move32();
        a[j1 + 1] = L_add(x1i, x3r);
        move32();
        a[j3] = L_add(x1r, x3i);
        move32();
        a[j3 + 1] = L_sub(x1i, x3r);
        move32();
    }

    wk1r = w[2];
    move16();
    tmp = add(l, m);
    FOR (j = m; j < tmp; j += 2)
    {
        j1 = add(j, l);
        j2 = add(j1, l);
        j3 = add(j2, l);
        x0r = L_add(a[j], a[j1]);
        x0i = L_add(a[j + 1], a[j1 + 1]);
        x1r = L_sub(a[j], a[j1]);
        x1i = L_sub(a[j + 1], a[j1 + 1]);
        x2r = L_add(a[j2], a[j3]);
        x2i = L_add(a[j2 + 1], a[j3 + 1]);
        x3r = L_sub(a[j2], a[j3]);
        x3i = L_sub(a[j2 + 1], a[j3 + 1]);
        a[j] = L_add(x0r, x2r);
        move32();
        a[j + 1] = L_add(x0i, x2i);
        move32();
        a[j2] = L_sub(x2i, x0i);
        move32();
        a[j2 + 1] = L_sub(x0r, x2r);
        move32();
        x0r = L_sub(x1r, x3i);
        x0i = L_add(x1i, x3r);
        a[j1] = Mult_32_16(L_shl(L_sub(x0r, x0i), 1), wk1r);
        move32();
        a[j1 + 1] = Mult_32_16(L_shl(L_add(x0r, x0i), 1), wk1r);
        move32();
        x0r = L_add(x3i, x1r);
        x0i = L_sub(x3r, x1i);
        a[j3] = Mult_32_16(L_shl(L_sub(x0i, x0r), 1), wk1r);
        move32();
        a[j3 + 1] = Mult_32_16(L_shl(L_add(x0r, x0i), 1), wk1r);
        move32();
    }

    k1 = 0;
    move16();
    m2 = shl(m, 1);
    FOR (k = m2; k < n; k += m2)
    {
        k1 = add(k1, 2);
        k2 = shl(k1, 1);
        wk2r = w[k1];
        move16();
        wk2i = w[k1 + 1];
        move16();
        wk1r = w[k2];
        move16();
        wk1i = w[k2 + 1];
        move16();
        wk3r = extract_l(L_sub(L_deposit_l(wk1r), L_shr(L_mult(wk2i, wk1i), 14)));
        wk3i = extract_l(L_msu0(L_shr(L_mult(wk2i, wk1r), 14), wk1i, 1));

        tmp = add(l, k) ;
        FOR (j = k; j < tmp; j += 2)
        {
            j1 = add(j, l);
            j2 = add(j1, l);
            j3 = add(j2, l);
            x0r = L_add(a[j], a[j1]);
            x0i = L_add(a[j + 1], a[j1 + 1]);
            x1r = L_sub(a[j], a[j1]);
            x1i = L_sub(a[j + 1], a[j1 + 1]);
            x2r = L_add(a[j2], a[j3]);
            x2i = L_add(a[j2 + 1], a[j3 + 1]);
            x3r = L_sub(a[j2], a[j3]);
            x3i = L_sub(a[j2 + 1], a[j3 + 1]);
            a[j] = L_add(x0r, x2r);
            move32();
            a[j + 1] = L_add(x0i, x2i);
            move32();
            x0r = L_sub(x0r, x2r);
            x0i = L_sub(x0i, x2i);
            a[j2] = L_sub(Mult_32_16(L_shl(x0r, 1), wk2r), Mult_32_16(L_shl(x0i, 1), wk2i));
            move32();
            a[j2 + 1] = L_add(Mult_32_16(L_shl(x0i, 1), wk2r), Mult_32_16(L_shl(x0r, 1), wk2i));
            move32();
            x0r = L_sub(x1r, x3i);
            x0i = L_add(x1i, x3r);
            a[j1] = L_sub(Mult_32_16(L_shl(x0r, 1), wk1r), Mult_32_16(L_shl(x0i, 1), wk1i));
            move32();
            a[j1 + 1] = L_add(Mult_32_16(L_shl(x0i, 1), wk1r), Mult_32_16(L_shl(x0r, 1), wk1i));
            move32();
            x0r = L_add(x1r, x3i);
            x0i = L_sub(x1i, x3r);
            a[j3] = L_sub(Mult_32_16(L_shl(x0r, 1), wk3r), Mult_32_16(L_shl(x0i, 1), wk3i));
            move32();
            a[j3 + 1] = L_add(Mult_32_16(L_shl(x0i, 1), wk3r), Mult_32_16(L_shl(x0r, 1), wk3i));
            move32();
        }

        wk1r = w[k2 + 2];
        move16();
        wk1i = w[k2 + 3];
        move16();
        wk3r = extract_l(L_sub(L_deposit_l(wk1r), L_shr(L_mult(wk2r, wk1i), 14)));
        wk3i = extract_l(L_msu0(L_shr(L_mult(wk2r, wk1r), 14), wk1i, 1));

        tmp = add(l, add(k, m));
        FOR (j = add(k, m); j < tmp; j += 2)
        {
            j1 = add(j, l);
            j2 = add(j1, l);
            j3 = add(j2, l);
            x0r = L_add(a[j], a[j1]);
            x0i = L_add(a[j + 1], a[j1 + 1]);
            x1r = L_sub(a[j], a[j1]);
            x1i = L_sub(a[j + 1], a[j1 + 1]);
            x2r = L_add(a[j2], a[j3]);
            x2i = L_add(a[j2 + 1], a[j3 + 1]);
            x3r = L_sub(a[j2], a[j3]);
            x3i = L_sub(a[j2 + 1], a[j3 + 1]);
            a[j] = L_add(x0r, x2r);
            move32();
            a[j + 1] = L_add(x0i, x2i);
            move32();
            x0r= L_sub(x0r, x2r);
            x0i=L_sub(x0i, x2i);
            a[j2] = L_negate(L_add(Mult_32_16(L_shl(x0r, 1), wk2i), Mult_32_16(L_shl(x0i, 1), wk2r)));
            move32();
            a[j2 + 1] = L_sub(Mult_32_16(L_shl(x0r, 1), wk2r), Mult_32_16(L_shl(x0i, 1), wk2i));
            move32();
            x0r = L_sub(x1r, x3i);
            x0i = L_add(x1i, x3r);
            a[j1] = L_sub(Mult_32_16(L_shl(x0r, 1), wk1r), Mult_32_16(L_shl(x0i, 1), wk1i));
            move32();
            a[j1 + 1] = L_add(Mult_32_16(L_shl(x0i, 1), wk1r), Mult_32_16(L_shl(x0r, 1), wk1i));
            move32();
            x0r = L_add(x1r, x3i);
            x0i = L_sub(x1i, x3r);
            a[j3] = L_sub(Mult_32_16(L_shl(x0r, 1), wk3r), Mult_32_16(L_shl(x0i, 1), wk3i));
            move32();
            a[j3 + 1] = L_add(Mult_32_16(L_shl(x0i, 1), wk3r), Mult_32_16(L_shl(x0r, 1), wk3i));
            move32();
        }
    }

    return;
}


static void cftbsub_fx(
    Word16 n,
    Word32 *a,
    const Word16 *w     /* i    : cos/sin table                 */
)
{
    Word16 j, j1, j2, j3, l;
    Word32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

    l = 2;
    move16();
    IF (n > 8)
    {
        cft1st_fx(n, a, w);
        l = 8;
        move16();

        WHILE (sub(shl(l, 2), n) < 0)
        {
            cftmdl_fx(n, l, a, w);
            l = shl(l, 2);
        }
    }

    IF (sub(shl(l, 2), n) == 0)
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j, l);
            j2 = add(j1, l);
            j3 = add(j2, l);
            x0r = L_add(a[j], a[j1]);
            x0i = L_negate(L_add(a[j + 1], a[j1 + 1]));
            x1r = L_sub(a[j], a[j1]);
            x1i = L_sub(a[j1 + 1], a[j + 1]);
            x2r = L_add(a[j2], a[j3]);
            x2i = L_add(a[j2 + 1], a[j3 + 1]);
            x3r = L_sub(a[j2], a[j3]);
            x3i = L_sub(a[j2 + 1], a[j3 + 1]);
            a[j] = L_add(x0r, x2r);
            move32();
            a[j + 1] = L_sub(x0i, x2i);
            move32();
            a[j2] = L_sub(x0r, x2r);
            move32();
            a[j2 + 1] = L_add(x0i, x2i);
            move32();
            a[j1] = L_sub(x1r, x3i);
            move32();
            a[j1 + 1] = L_sub(x1i, x3r);
            move32();
            a[j3] = L_add(x1r, x3i);
            move32();
            a[j3 + 1] = L_add(x1i, x3r);
            move32();
        }
    }
    ELSE
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j, l);
            x0r = L_sub(a[j], a[j1]);
            x0i = L_sub(a[j1 + 1], a[j + 1]);
            a[j] = L_add(a[j], a[j1]);
            move32();
            a[j + 1] = L_negate(L_add(a[j + 1], a[j1 + 1]));
            move32();
            a[j1] = x0r;
            move32();
            move32();
            a[j1 + 1] = x0i;
            move32();
            move32();
        }
    }
}

static void rftfsub_fx(
    Word16 n,
    Word32 *a,
    Word16 nc,
    const Word16 *c
)
{
    Word16 j, k, kk, ks, m, tmp;
    Word32 xr, xi, yr, yi;
    Word16 wkr, wki;

    m = shr(n, 1);
    /*ks = 2 * nc / m; */
    tmp = shl(nc, 1);
    ks = 0;
    move16();
    WHILE (sub(tmp, m) >= 0)
    {
        ks = add(ks, 1);
        tmp = sub(tmp, m);
    }
    kk = 0;
    move16();
    FOR (j = 2; j < m; j += 2)
    {
        k = sub(n, j);
        kk = add(kk, ks);
        wkr = sub(8192, c[nc - kk]);
        wki = c[kk];
        move16();
        xr = L_sub(a[j], a[k]);
        xi = L_add(a[j + 1], a[k + 1]);
        yr = L_sub(Mult_32_16(L_shl(xr, 1), wkr), Mult_32_16(L_shl(xi, 1), wki));
        yi = L_add(Mult_32_16(L_shl(xi, 1), wkr), Mult_32_16(L_shl(xr, 1), wki));
        a[j] = L_sub(a[j], yr);
        move32();
        a[j + 1] = L_sub(a[j + 1], yi);
        move32();
        a[k] = L_add(a[k], yr);
        move32();
        a[k + 1] = L_sub(a[k + 1], yi);
        move32();
    }
}


static void rftbsub_fx(
    Word16 n,
    Word32 *a,
    Word16 nc,
    const Word16 *c
)
{
    Word16 j, k, kk, ks, m, tmp;
    Word32 xr, xi, yr, yi;
    Word16 wkr, wki;

    a[1] = L_negate(a[1]);
    m = shr(n, 1);
    /*ks = 2 * nc / m; */
    tmp = shl(nc, 1);
    ks = 0;
    move16();
    WHILE (sub(tmp, m) >= 0)
    {
        ks = add(ks, 1);
        tmp = sub(tmp, m);
    }
    kk = 0;
    move16();
    FOR (j = 2; j < m; j += 2)
    {
        k = sub(n, j);
        kk = add(kk, ks);
        wkr = sub(8192, c[nc - kk]);
        wki = c[kk];
        move16();
        xr = L_sub(a[j], a[k]);
        xi = L_add(a[j + 1], a[k + 1]);
        yr = L_add(Mult_32_16(L_shl(xr, 1), wkr), Mult_32_16(L_shl(xi, 1), wki));
        yi = L_sub(Mult_32_16(L_shl(xi, 1), wkr), Mult_32_16(L_shl(xr, 1), wki));
        a[j] = L_sub(a[j], yr);
        move32();
        a[j + 1] = L_sub(yi, a[j + 1]);
        move32();
        a[k] = L_add(a[k], yr);
        move32();
        a[k + 1] = L_sub(yi, a[k + 1]);
        move32();
    }
    a[m + 1] = L_negate(a[m + 1]);
    move32();
}


static void dctsub_fx(
    Word16 n,
    Word32 *a,
    Word16 nc,
    const Word16 *c
)
{
    Word16 j, k, kk, ks, m, tmp;
    Word16 wkr, wki;
    Word32 xr;

    m = shr(n, 1);
    /*ks = nc / n; */
    tmp = nc;
    move16();
    ks = 0;
    move16();
    WHILE (sub(tmp, n) >= 0)
    {
        ks = add(ks, 1);
        tmp = sub(tmp, n);
    }
    kk = 0;
    move16();
    FOR (j = 1; j < m; j++)
    {
        k = sub(n, j);
        kk = add(kk, ks);
        wkr = sub(c[kk], c[nc - kk]);
        wki = add(c[kk], c[nc - kk]);
        xr = L_sub(Mult_32_16(L_shl(a[j], 1), wki), Mult_32_16(L_shl(a[k], 1), wkr));
        a[j] = L_add(Mult_32_16(L_shl(a[j], 1), wkr), Mult_32_16(L_shl(a[k], 1), wki));
        move32();
        a[k] = xr;
        move32();
    }
    a[m] = Mult_32_16(L_shl(a[m], 1), c[0]);
}

/*-----------------------------------------------------------------*
 * edct2_fx()
 *
 * Transformation of the signal to DCT domain
 * OR Inverse EDCT-II for short frames
 *-----------------------------------------------------------------*/

void edct2_fx(
    Word16 n,
    Word16 isgn,
    Word16 *in,
    Word32 *a,
    Word16 *q,
    const Word16 *ip,
    const Word16 *w
)
{
    Word16 j, nw, nc;
    Word32 xr;

    *q = Exp16Array(n, in);
    *q = add(*q, 6);
    FOR (j = 0; j < n; j++)
    {
        a[j] = L_shl((Word32) in[j], *q);
        move32();
    }

    nw = ip[0];
    move16();
    if (sub(n, shl(nw, 2)) > 0)
    {
        nw = shr(n, 2);
    }

    nc = ip[1];
    move16();
    if (n > nc)
    {
        nc = n;
        move16();
    }

    IF (isgn < 0)
    {
        xr = a[n - 1];
        move32();
        FOR (j = n - 2; j >= 2; j -= 2)
        {
            a[j + 1] = L_sub(a[j], a[j - 1]);
            move32();
            a[j] = L_add(a[j], a[j - 1]);
            move32();
        }
        a[1] = L_sub(a[0], xr);
        move32();
        a[0] = L_add(a[0], xr);
        move32();

        IF (n > 4)
        {
            rftbsub_fx(n, a, nc, w + nw);
            bitrv2_SR_fx(n, ip + 2, a);
            cftbsub_fx(n, a, w);
        }
        ELSE IF (n == 4)
        {
            cftfsub_fx(n, a, w);
        }
    }

    IF (isgn >= 0)
    {
        a[0] = L_shr(a[0], 1);
        move32();
    }

    dctsub_fx(n, a, nc, w + nw);

    IF (isgn >= 0)
    {
        IF (n > 4)
        {
            bitrv2_SR_fx(n, ip + 2, a);
            cftfsub_fx(n, a, w);
            rftfsub_fx(n, a, nc, w + nw);
        }
        ELSE IF (n == 4)
        {
            cftfsub_fx(n, a, w);
        }
        xr = L_sub(a[0], a[1]);
        a[0] = L_add(a[0], a[1]);
        move32();
        FOR (j = 2; j < n; j += 2)
        {
            a[j - 1] = L_sub(a[j], a[j + 1]);
            move32();
            a[j] = L_add(a[j], a[j + 1]);
            move32();
        }
        a[n - 1] = xr;
        move32();

        FOR (j = 0; j < n; j ++)
        {
            a[j] = L_shr(a[j], 5);
            move32();
        }
    }
}


/*-----------------------------------------------------------------*
* fft5_shift4()
* 5-point FFT with 4-point circular shift
*-----------------------------------------------------------------*/

static void fft5_shift4_16fx(
    Word16   n1,      /* i   : length of data                           */
    Word16 *zRe,    /* i/o : real part of input and output data       */
    Word16 *zIm,    /* i/o : imaginary part of input and output data  */
    const Word16   *Idx     /* i   : pointer of the address table             */
)
{
    Word16 T1, To, T8, Tt, T9, Ts, Te, Tp, Th, Tn,T2, T3, T4, T5, T6, T7;
    Word16 i0,i1,i2,i3,i4;
    Word32 L_tmp;

    i0 = Idx[0];
    move16();
    i1 = Idx[n1];
    move16();
    i2 = Idx[n1*2];
    move16();
    i3 = Idx[n1*3];
    move16();
    i4 = Idx[n1*4];
    move16();

    T1 = zRe[i0];
    move16();
    To = zIm[i0];
    move16();

    T2 = zRe[i1];
    move16();
    T3 = zRe[i4];
    move16();
    T4 = add(T2,T3);
    T5 = zRe[i2];
    move16();
    T6 = zRe[i3];
    move16();
    T7 = add(T5,T6);
    T8 = add(T4,T7);
    Tt = sub(T5,T6);
    /*    T9 = KP559016994 * (T4 - T7); */
    L_tmp = Mult_32_16(KP559016994_16FX,sub(T4,T7));
    T9 = round_fx(L_tmp);
    Ts = sub(T2,T3);

    T2 = zIm[i1];
    move16();
    T3 = zIm[i4];
    move16();
    T4 = add(T2,T3);
    T5 = zIm[i2];
    move16();
    T6 = zIm[i3];
    move16();
    T7 = add(T5,T6);
    Te = sub(T2,T3);
    Tp = add(T4,T7);
    Th = sub(T5,T6);
    /*       Tn = KP559016994 * (T4 - T7); */
    L_tmp = Mult_32_16(KP559016994_16FX,sub(T4,T7));
    Tn = round_fx(L_tmp);

    zRe[i0] = add(T1,T8);
    move16();
    zIm[i0] = add(To,Tp);
    move16();

    /*        T2 = KP951056516*Te + KP587785252*Th; */
    L_tmp = Mult_32_16(KP951056516_16FX,Te);
    L_tmp = Madd_32_16(L_tmp,KP587785252_16FX,Th);
    T2 = round_fx(L_tmp);

    /*T3 = KP951056516*Th - KP587785252*Te; */
    L_tmp = Mult_32_16(KP951056516_16FX,Th);
    L_tmp = Msub_32_16(L_tmp,KP587785252_16FX,Te);
    T3 = round_fx(L_tmp);

    T6 = sub(T1,shr(T8,2));
    T4 = add(T9,T6);
    T5 = sub(T6,T9);
    zRe[i1] = sub(T4,T2);
    move16();
    zRe[i2] = add(T5,T3);
    move16();
    zRe[i4] = add(T4,T2);
    move16();
    zRe[i3] = sub(T5,T3);
    move16();

    /*    T2 = KP951056516 * Ts + KP587785252 * Tt; */
    L_tmp = Mult_32_16(KP951056516_16FX,Ts);
    L_tmp = Madd_32_16(L_tmp,KP587785252_16FX,Tt);
    T2 = round_fx(L_tmp);

    /*                T3 = KP951056516 * Tt - KP587785252 * Ts; */
    L_tmp = Mult_32_16(KP951056516_16FX,Tt);
    L_tmp = Msub_32_16(L_tmp,KP587785252_16FX,Ts);
    T3 = round_fx(L_tmp);

    T6 = sub(To,shr(Tp,2));
    T4 = add(Tn,T6);
    T5 = sub(T6,Tn);
    zIm[i4] = sub(T4,T2);
    move16();
    zIm[i2] = sub(T5,T3);
    move16();
    zIm[i1] = add(T2,T4);
    move16();
    zIm[i3] = add(T3,T5);
    move16();

    return;
}

/*-----------------------------------------------------------------*
* fft5_32()
* 5-point FFT called for 32 times
*-----------------------------------------------------------------*/
static void fft5_32_16fx(
    Word16 *zRe,    /* i/o : real part of input and output data       */
    Word16 *zIm,    /* i/o : imaginary part of input and output data  */
    const Word16   *Idx     /* i   : pointer of the address table             */
)
{
    Word16 T1, To, T8, Tt, T9, Ts, Te, Tp, Th, Tn,T2, T3, T4, T5, T6, T7;
    Word16 i0,i1,i2,i3,i4;
    Word32 L_tmp;

    i0 = Idx[0];
    move16();
    i1 = Idx[32];
    move16();
    i2 = Idx[64];
    move16();
    i3 = Idx[96];
    move16();
    i4 = Idx[128];
    move16();

    T1 = zRe[i0];
    move16();
    To = zIm[i0];
    move16();

    T2 = zRe[i1];
    move16();
    T3 = zRe[i4];
    move16();
    T4 = add(T2, T3);
    T5 = zRe[i2];
    move16();
    T6 = zRe[i3];
    move16();
    T7 = add(T5,T6);
    T8 = add(T4,T7);
    Tt = sub(T5,T6);
    /* T9 = KP559016994 * (T4 - T7); */
    L_tmp = Mult_32_16(KP559016994_16FX,sub(T4,T7));
    T9 = round_fx(L_tmp);
    Ts = sub(T2,T3);

    T2 = zIm[i1];
    move16();
    T3 = zIm[i4];
    move16();
    T4 = add(T2,T3);
    T5 = zIm[i2];
    move16();
    T6 = zIm[i3];
    move16();
    T7 = add(T5,T6);
    Te = sub(T2,T3);
    Tp = add(T4,T7);
    Th = sub(T5,T6);
    L_tmp = Mult_32_16(KP559016994_16FX,sub(T4,T7));
    Tn = round_fx(L_tmp);



    zRe[i0] = add(T1,T8);
    move16();
    zIm[i0] = add(To,Tp);
    move32();



    /*T2 = KP951056516*Te + KP587785252*Th; */
    L_tmp = Mult_32_16(KP951056516_16FX,Te);
    L_tmp = Madd_32_16(L_tmp,KP587785252_16FX,Th);
    T2 = round_fx(L_tmp);

    /*T3 = KP951056516*Th - KP587785252*Te; */
    L_tmp = Mult_32_16(KP951056516_16FX,Th);
    L_tmp = Msub_32_16(L_tmp,KP587785252_16FX,Te);
    T3 = round_fx(L_tmp);



    T6 = sub(T1,shr(T8,2));
    T4 = add(T9,T6);
    T5 = sub(T6,T9);
    zRe[i3] = sub(T4,T2);
    move32();
    zRe[i1] = add(T5,T3);
    move32();
    zRe[i2] = add(T4,T2);
    move32();
    zRe[i4] = sub(T5,T3);
    move32();

    /*    T2 = KP951056516 * Ts + KP587785252 * Tt; */
    L_tmp = Mult_32_16(KP951056516_16FX,Ts);
    L_tmp = Madd_32_16(L_tmp,KP587785252_16FX,Tt);
    T2 = round_fx(L_tmp);

    /*                T3 = KP951056516 * Tt - KP587785252 * Ts; */
    L_tmp = Mult_32_16(KP951056516_16FX,Tt);
    L_tmp = Msub_32_16(L_tmp,KP587785252_16FX,Ts);
    T3 = round_fx(L_tmp);

    T6 = sub(To,shr(Tp,2));
    T4 = add(Tn,T6);
    T5 = sub(T6,Tn);
    zIm[i2] = sub(T4,T2);
    move16();
    zIm[i1] = sub(T5,T3);
    move16();
    zIm[i3] = add(T2,T4);
    move16();
    zIm[i4] = add(T3,T5);
    move16();


    return;
}

/*-----------------------------------------------------------------*
* fft64()
* 64-point FFT
*-----------------------------------------------------------------*/
static void fft64_16fx(
    Word16 *x,      /* i/o : real part of input and output data       */
    Word16 *y,      /* i/o : imaginary part of input and output data  */
    const Word16 *Idx     /* i   : pointer of the address table             */
)
{
    Word16 i,id,jd;
    Word16 z[128];
    move16();/*penalty for 1 ptr init */
    FOR ( i=0; i<64; i++ )
    {
        id = Idx[i];
        move16();
        z[2*i]   = x[id];
        move16();
        z[2*i+1] = y[id];
        move16();
    }

    cdftForw_16fx(128,z,Ip_fft64_16fx,w_fft64_16fx);

    move16();/*penalty for 1 ptr init */
    FOR( i=0; i<64 ; i++)
    {
        jd = Odx_fft64_16fx[i];
        move16();
        id = Idx[jd];
        move16();
        x[id]=z[2*i];
        move16();
        y[id]=z[2*i+1];
        move16();
    }

    return;
}


/*-----------------------------------------------------------------*
* fft32_5()
* 32-point FFT called for 5 times
*-----------------------------------------------------------------*/
static void fft32_5_16fx(
    Word16 *x,      /* i/o : real part of input and output data       */
    Word16 *y,      /* i/o : imaginary part of input and output data  */
    const Word16 *Idx     /* i   : pointer of the address table             */
)
{
    Word16 i,id,jd;
    Word16 z[64];

    move16();/*penalty for 1 ptr init */
    FOR( i=0; i<32; i++ )
    {
        id = Idx[i];
        move16();
        z[2*i]   = x[id];
        move16();
        z[2*i+1] = y[id];
        move16();
    }

    cdftForw_16fx(64,z,Ip_fft32_16fx,w_fft32_16fx);

    move16();/*penalty for 1 ptr init */
    FOR( i=0; i<32; i++ )
    {
        jd = Odx_fft32_5[i];
        move16();
        id = Idx[jd];
        move16();
        x[id]=z[2*i];
        move16();
        y[id]=z[2*i+1];
        move16();
    }

    return;
}



/*-----------------------------------------------------------------*
* DoRTFT160()
* a low complexity 2-dimensional DFT of 160 points
*-----------------------------------------------------------------*/
void DoRTFT160_16fx(
    Word16 x[],     /* i/o : real part of input and output data       */
    Word16 y[]      /* i/o : imaginary part of input and output data  */
)
{
    Word16 j;

    /* Applying 32-point FFT for 5 times based on the address table Idx_dortft160 */
    FOR(j=0; j<5; j++)
    {
        fft32_5_16fx(x,y,Idx_dortft160+shl(j,5)/*32*j*/);
    }

    /* Applying 5-point FFT for 32 times based on the address table Idx_dortft160 */
    FOR(j=0; j<32; j++)
    {
        fft5_32_16fx(x,y,Idx_dortft160+j);
    }

    return;
}

/*-----------------------------------------------------------------*
* DoRTFT320()
* a low complexity 2-dimensional DFT of 320 points
*-----------------------------------------------------------------*/
void DoRTFT320_16fx(
    Word16 *x,   /* i/o : real part of input and output data       */
    Word16 *y    /* i/o : imaginary part of input and output data  */
)
{
    Word16 j;

    /* Applying 64-point FFT for 5 times based on the address table Idx_dortft160 */
    FOR(j=0; j<5; j++)
    {
        fft64_16fx(x,y,Idx_dortft320_16fx+shl(j,6)/*64*j*/);
    }

    /* Applying 5-point FFT for 64 times based on the address table Idx_dortft160 */
    FOR(j=0; j<64; j++)
    {
        fft5_shift4_16fx(64,x,y,Idx_dortft320_16fx+j);
    }

    return;
}

/*-----------------------------------------------------------------*
* DoRTFT128()
* FFT with 128 points
*-----------------------------------------------------------------*/
void DoRTFT128_16fx(
    Word16 *x,    /* i/o : real part of input and output data       Q(Qx+Q_edct)*/
    Word16 *y     /* i/o : imaginary part of input and output data  Q(Qx+Q_edct)*/
)
{

    Word16 i;
    Word16 z[256];

    move16();/*penalty for 1 ptr init */
    FOR ( i=0; i<128; i++ )
    {
        z[2*i]   = x[i];
        move16();
        z[2*i+1] = y[i];
        move16();
    }

    cdftForw_16fx(256,z,Ip_fft128_16fx,w_fft128_16fx);

    x[0]=z[0];
    move16();
    y[0]=z[1];
    move16();
    move16();/*penalty for 1 ptr init */
    move16();/*penalty for 1 ptr init */
    FOR( i=1; i<128 ; i++)
    {
        x[128-i]=z[2*i];
        move16();
        y[128-i]=z[2*i+1];
        move16();
    }

    return;
}
/*-----------------------------------------------------------------*
* cdftForw()
* Main fuction of Complex Discrete Fourier Transform
*-----------------------------------------------------------------*/
static void cdftForw_16fx(
    Word16 n,    /* i    : data length of real and imag				 */
    Word16 *a,   /* i/o  : input/output data             Q(Qx+Q_edct)*/
    const Word16 *ip,  /* i    : work area for bit reversal				 */
    const Word32 *w    /* i    : cos/sin table						  Q30*/
)
{
    /* bit reversal */
    bitrv2_SR_16fx(n, ip + 2, a);

    /* Do FFT */
    cftfsub_16fx(n, a, w);
}

/*-----------------------------------------------------------------*
* bitrv2_SR()
* Bit reversal
*-----------------------------------------------------------------*/
static void bitrv2_SR_16fx(
    Word16 n,     /* i    : data length of real and imag			  */
    const Word16 *ip,   /* i/o  : work area for bit reversal				  */
    Word16 *a     /* i/o  : input/output data             Q(Qx+Q_edct)*/
)
{
    Word16 j, j1, k, k1, m, m2;
    Word16 l;
    Word16 xr, xi, yr, yi;

    l = n;
    move16();
    m = 1;
    move16();

    WHILE (sub(shl(m,3),l) < 0)
    {
        l = shr(l,1);
        m = shl(m,1);
    }

    m2 = shl(m,1);
    IF (sub(shl(m, 3),l) == 0)
    {
        FOR (k = 0; k < m; k++)
        {
            FOR (j = 0; j < k; j++)
            {
                j1 = add(shl(j,1),ip[k]);
                k1 = add(shl(k,1),ip[j]);
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
                j1 = add(j1,m2);
                k1 = add(k1,shl(m2,1));
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
                j1 = add(j1,m2);
                k1 = sub(k1,m2);
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
                j1 = add(j1,m2);
                k1 = add(k1,shl(m2,1));
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
            }

            j1 = add(add(shl(k,1),m2),ip[k]);
            k1 = add(j1,m2);
            xr = a[j1];
            move16();
            xi = a[j1 + 1];
            move16();
            yr = a[k1];
            move16();
            yi = a[k1 + 1];
            move16();
            a[j1] = yr;
            move16();
            a[j1 + 1] = yi;
            move16();
            a[k1] = xr;
            move16();
            a[k1 + 1] = xi;
            move16();
        }
    }
    ELSE
    {
        FOR (k = 1; k < m; k++)
        {
            FOR (j = 0; j < k; j++)
            {
                j1 = add(shl(j,1),ip[k]);
                k1 = add(shl(k,1),ip[j]);
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
                j1 = add(j1,m2);
                k1 = add(k1,m2);
                xr = a[j1];
                move16();
                xi = a[j1 + 1];
                move16();
                yr = a[k1];
                move16();
                yi = a[k1 + 1];
                move16();
                a[j1] = yr;
                move16();
                a[j1 + 1] = yi;
                move16();
                a[k1] = xr;
                move16();
                a[k1 + 1] = xi;
                move16();
            }
        }
    }

    return;
}

/*-----------------------------------------------------------------*
* cftfsub()
* Complex Discrete Fourier Transform
*-----------------------------------------------------------------*/
static void cftfsub_16fx(
    Word16 n,     /* i    : data length of real and imag			  */
    Word16 *a,    /* i/o  : input/output data             Q(Qx+Q_edct)*/
    const Word32 *w     /* i    : cos/sin table                          Q30*/
)
{
    Word16 j, j1, j2, j3, l;
    Word16 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

    l = 2;
    move16();
    IF (sub(n,8) > 0)
    {
        cft1st_16fx(n, a, w);
        l = 8;
        move16();
        WHILE (sub(shl(l, 2),n) < 0)
        {
            cftmdl_16fx(n, l, a, w);
            l = shl(l,2);
        }
    }

    IF (sub(shl(l,2),n) == 0)
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j,l);
            j2 = add(j1,l);
            j3 = add(j2,l);
            x0r = add(a[j],a[j1]);
            x0i = add(a[j + 1],a[j1 + 1]);
            x1r = sub(a[j],a[j1]);
            x1i = sub(a[j + 1],a[j1 + 1]);
            x2r = add(a[j2],a[j3]);
            x2i = add(a[j2 + 1],a[j3 + 1]);
            x3r = sub(a[j2],a[j3]);
            x3i = sub(a[j2 + 1],a[j3 + 1]);
            a[j] = add(x0r,x2r);
            move16();
            a[j + 1] = add(x0i,x2i);
            move16();
            a[j2] = sub(x0r,x2r);
            move16();
            a[j2 + 1] = sub(x0i,x2i);
            move16();
            a[j1] = sub(x1r,x3i);
            move16();
            a[j1 + 1] = add(x1i,x3r);
            move16();
            a[j3] = add(x1r,x3i);
            move16();
            a[j3 + 1] = sub(x1i,x3r);
            move16();
        }
    }
    ELSE
    {
        FOR (j = 0; j < l; j += 2)
        {
            j1 = add(j,l);
            x0r = sub(a[j],a[j1]);
            x0i = sub(a[j + 1],a[j1 + 1]);
            a[j] = add(a[j],a[j1]);
            move16();
            a[j + 1] = add(a[j + 1],a[j1 + 1]);
            move16();
            a[j1] = x0r;
            move16();
            a[j1 + 1] = x0i;
            move16();
        }
    }
    return;
}

/*-----------------------------------------------------------------*
* cft1st()
* Subfunction of Complex Discrete Fourier Transform
*-----------------------------------------------------------------*/
static void cft1st_16fx(
    Word16 n,     /* i    : data length of real and imag              */
    Word16 *a,    /* i/o  : input/output data             Q(Qx+Q_edct)*/
    const  Word32 *w     /* i    : cos/sin table                          Q30*/
)
{
    Word16 j, k1, k2;
    Word32 wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
    Word16 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
    Word16 tmp;
    Word32 L_tmp;

    x0r = add(a[0],a[2]);
    x0i = add(a[1],a[3]);
    x1r = sub(a[0],a[2]);
    x1i = sub(a[1],a[3]);
    x2r = add(a[4],a[6]);
    x2i = add(a[5],a[7]);
    x3r = sub(a[4],a[6]);
    x3i = sub(a[5],a[7]);
    a[0] = add(x0r,x2r);
    move16();
    a[1] = add(x0i,x2i);
    move16();
    a[4] = sub(x0r,x2r);
    move16();
    a[5] = sub(x0i,x2i);
    move16();
    a[2] = sub(x1r,x3i);
    move16();
    a[3] = add(x1i,x3r);
    move16();
    a[6] = add(x1r,x3i);
    move16();
    a[7] = sub(x1i,x3r);
    move16();

    wk1r = w[2];
    move32();

    x0r = add(a[8],a[10]);
    x0i = add(a[9],a[11]);
    x1r = sub(a[8],a[10]);
    x1i = sub(a[9],a[11]);
    x2r = add(a[12],a[14]);
    x2i = add(a[13],a[15]);
    x3r = sub(a[12],a[14]);
    x3i = sub(a[13],a[15]);
    a[8] = add(x0r,x2r);
    move16();
    a[9] = add(x0i,x2i);
    move16();
    a[12] = sub(x2i,x0i);
    move16();
    a[13] = sub(x0r,x2r);
    move16();

    x0r = sub(x1r,x3i);
    x0i = add(x1i,x3r);
    tmp = sub(x0r,x0i);
    L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */

    a[10] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

    tmp = add(x0r,x0i);
    L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */
    a[11] = round_fx(L_shl(L_tmp,1)); /* Q(Qx+Q_edct) */

    x0r = add(x3i,x1r);
    x0i = sub(x3r,x1i);
    tmp = sub(x0i,x0r);
    L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */
    a[14] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

    tmp = add(x0i,x0r);
    L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */
    a[15] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

    k1 = 0;
    move16();

    FOR (j = 16; j < n; j += 16)
    {
        k1 = add(k1,2);
        k2 = shl(k1,1);

        wk2r = L_add(0,w[k1]);
        wk2i = L_add(0,w[k1 + 1]);
        wk1r = L_add(0,w[k2]);
        wk1i = L_add(0,w[k2 + 1]);

        L_tmp = L_shl(Mult_32_32(wk2i,wk1i),1);/*Q29 */
        wk3r = L_sub(wk1r,L_shl(L_tmp,1));/*Q30 */

        L_tmp = L_shl(Mult_32_32(wk2i,wk1r),1);/*Q29 */
        wk3i = L_sub(L_shl(L_tmp,1),wk1i);/*Q30 */

        x0r = add(a[j],a[j + 2]);
        x0i = add(a[j + 1],a[j + 3]);
        x1r = sub(a[j],a[j + 2]);
        x1i = sub(a[j + 1],a[j + 3]);
        x2r = add(a[j + 4],a[j + 6]);
        x2i = add(a[j + 5],a[j + 7]);
        x3r = sub(a[j + 4],a[j + 6]);
        x3i = sub(a[j + 5],a[j + 7]);
        a[j] = add(x0r,x2r);
        move16();
        a[j + 1] = add(x0i,x2i);
        move16();

        x0r = sub(x0r,x2r);
        x0i = sub(x0i,x2i);
        L_tmp = Mult_32_16(wk2r,x0r);/*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk2i,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 4] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        L_tmp = Mult_32_16(wk2r,x0i);/*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk2i,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 5] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        x0r = sub(x1r,x3i);
        x0i = add(x1i,x3r);
        L_tmp = Mult_32_16(wk1r,x0r);/*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk1i,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 2] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        L_tmp = Mult_32_16(wk1r,x0i);/*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk1i,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 3] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        x0r = add(x1r,x3i);
        x0i = sub(x1i,x3r);
        L_tmp = Mult_32_16(wk3r,x0r); /*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk3i,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 6] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        L_tmp = Mult_32_16(wk3r,x0i); /*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk3i,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 7] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        wk1r = L_add(0,w[k2 + 2]);
        wk1i = L_add(0,w[k2 + 3]);
        L_tmp = L_shl(Mult_32_32(wk2r,wk1i),1);/*Q29 */
        wk3r = L_sub(wk1r,L_shl(L_tmp,1)); /*Q30  */

        L_tmp = L_shl(Mult_32_32(wk2r,wk1r),1);/*Q29 */
        wk3i = L_sub(L_shl(L_tmp,1),wk1i); /*Q30 */

        x0r = add(a[j + 8],a[j + 10]);
        x0i = add(a[j + 9],a[j + 11]);
        x1r = sub(a[j + 8],a[j + 10]);
        x1i = sub(a[j + 9],a[j + 11]);
        x2r = add(a[j + 12],a[j + 14]);
        x2i = add(a[j + 13],a[j + 15]);
        x3r = sub(a[j + 12],a[j + 14]);
        x3i = sub(a[j + 13],a[j + 15]);
        a[j + 8] = add(x0r,x2r);
        move16();
        a[j + 9] = add(x0i,x2i);
        move16();

        x0r = sub(x0r,x2r);
        x0i = sub(x0i,x2i);
        tmp = negate(x0r);
        L_tmp = Mult_32_16(wk2i,tmp);/*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk2r,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 12] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        tmp = negate(x0i);
        L_tmp = Mult_32_16(wk2i,tmp);/*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk2r,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 13] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        x0r = sub(x1r,x3i);
        x0i = add(x1i,x3r);
        L_tmp = Mult_32_16(wk1r,x0r);/*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk1i,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 10] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        L_tmp = Mult_32_16(wk1r,x0i);/*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk1i,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 11] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        x0r = add(x1r,x3i);
        x0i = sub(x1i,x3r);

        L_tmp = Mult_32_16(wk3r,x0r); /*Q(15+Qx+Q_edct) */
        L_tmp = Msub_32_16(L_tmp,wk3i,x0i); /*Q(15+Qx+Q_edct) */
        a[j + 14] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        L_tmp = Mult_32_16(wk3r,x0i); /*Q(15+Qx+Q_edct) */
        L_tmp = Madd_32_16(L_tmp,wk3i,x0r); /*Q(15+Qx+Q_edct) */
        a[j + 15] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */
    }

    return;
}

/*-----------------------------------------------------------------*
* cftmdl()
* Subfunction of Complex Discrete Fourier Transform
*-----------------------------------------------------------------*/
static void cftmdl_16fx(
    Word16 n,     /* i    : data length of real and imag			   */
    Word16 l,     /* i    : initial shift for processing			   */
    Word16 *a,    /* i/o  : input/output data              Q(Qx+Q_edct)*/
    const  Word32 *w     /* i    : cos/sin table							Q30*/
)
{
    Word16 j, j1, j2, j3, k, k1, k2, m, m2;
    Word32 wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
    Word16 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
    Word16 tmp, tmp2;
    Word32 L_tmp;
    Word32 L_x0r, L_x0i;

    m = shl(l, 2);
    move16();
    FOR (j = 0; j < l; j += 2)
    {
        j1 = add(j,l);
        j2 = add(j1,l);
        j3 = add(j2,l);
        x0r = add(a[j],a[j1]);
        x0i = add(a[j + 1],a[j1 + 1]);
        x1r = sub(a[j],a[j1]);
        x1i = sub(a[j + 1],a[j1 + 1]);
        x2r = add(a[j2],a[j3]);
        x2i = add(a[j2 + 1],a[j3 + 1]);
        x3r = sub(a[j2],a[j3]);
        x3i = sub(a[j2 + 1],a[j3 + 1]);
        a[j] = add(x0r,x2r);
        move16();
        a[j + 1] = add(x0i,x2i);
        move16();
        a[j2] = sub(x0r,x2r);
        move16();
        a[j2 + 1] = sub(x0i,x2i);
        move16();
        a[j1] = sub(x1r,x3i);
        move16();
        a[j1 + 1] = add(x1i,x3r);
        move16();
        a[j3] = add(x1r,x3i);
        move16();
        a[j3 + 1] = sub(x1i,x3r);
        move16();
    }

    wk1r = w[2];
    move32();
    tmp2 = add(l,m);
    FOR (j = m; j < tmp2; j += 2)
    {
        j1 = add(j,l);
        j2 = add(j1,l);
        j3 = add(j2,l);
        x0r = add(a[j],a[j1]);
        x0i = add(a[j + 1],a[j1 + 1]);
        x1r = sub(a[j],a[j1]);
        x1i = sub(a[j + 1],a[j1 + 1]);
        x2r = add(a[j2],a[j3]);
        x2i = add(a[j2 + 1],a[j3 + 1]);
        x3r = sub(a[j2],a[j3]);
        x3i = sub(a[j2 + 1],a[j3 + 1]);
        a[j] = add(x0r,x2r);
        move16();
        a[j + 1] = add(x0i,x2i);
        move16();
        a[j2] = sub(x2i,x0i);
        move16();
        a[j2 + 1] = sub(x0r,x2r);
        move16();

        x0r = sub(x1r,x3i);
        x0i = add(x1i,x3r);
        tmp = sub(x0r,x0i);
        L_tmp = Mult_32_16(wk1r,tmp);/*Q(15+Qx+Q_edct) */
        a[j1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        tmp = add(x0r,x0i);
        L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */
        a[j1 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        x0r = add(x3i,x1r);
        x0i = sub(x3r,x1i);
        tmp = sub(x0i,x0r);
        L_tmp = Mult_32_16(wk1r,tmp);/*Q(15+Qx+Q_edct) */
        a[j3] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

        tmp = add(x0i,x0r);
        L_tmp = Mult_32_16(wk1r,tmp); /*Q(15+Qx+Q_edct) */
        a[j3 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */
    }

    k1 = 0;
    move16();
    m2 = shl(m,1);
    FOR (k = m2; k < n; k += m2)
    {
        k1 = add(k1,2);
        k2 = shl(k1,1);
        wk2r = L_add(0,w[k1]);
        wk2i = L_add(0,w[k1 + 1]);
        wk1r = L_add(0,w[k2]);
        wk1i = L_add(0,w[k2 + 1]);
        L_tmp = L_shl(Mult_32_32(wk2i,wk1i),1);/*Q29 */
        wk3r = L_sub(wk1r,L_shl(L_tmp,1));/*Q30 */

        L_tmp = L_shl(Mult_32_32(wk2i,wk1r),1);/*Q29 */
        wk3i = L_sub(L_shl(L_tmp,1),wk1i);/*Q30 */

        tmp2 = add(l,k);
        FOR (j = k; j < tmp2; j += 2)
        {
            j1 = add(j,l);
            j2 = add(j1,l);
            j3 = add(j2,l);
            x0r = add(a[j],a[j1]);
            x0i = add(a[j + 1],a[j1 + 1]);
            x1r = sub(a[j],a[j1]);
            x1i = sub(a[j + 1],a[j1 + 1]);
            x2r = add(a[j2],a[j3]);
            x2i = add(a[j2 + 1],a[j3 + 1]);
            x3r = sub(a[j2],a[j3]);
            x3i = sub(a[j2 + 1],a[j3 + 1]);
            a[j] = add(x0r,x2r);
            move16();
            a[j + 1] = add(x0i,x2i);
            move16();

            x0r = sub(x0r,x2r);
            x0i = sub(x0i,x2i);

            L_tmp = Mult_32_16(wk2r,x0r); /*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk2i,x0i); /*Q(15+Qx+Q_edct) */
            a[j2] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_tmp = Mult_32_16(wk2r,x0i); /*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk2i,x0r); /*Q(15+Qx+Q_edct) */
            a[j2 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            x0r = sub(x1r,x3i);
            x0i = add(x1i,x3r);

            L_tmp = Mult_32_16(wk1r,x0r); /*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk1i,x0i); /*Q(15+Qx+Q_edct) */
            a[j1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_tmp = Mult_32_16(wk1r,x0i); /*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk1i,x0r); /*Q(15+Qx+Q_edct) */
            a[j1 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_x0r = L_add((Word32) x1r, (Word32) x3i);
            L_x0i = L_sub((Word32) x1i, (Word32) x3r);
            x0r = extract_l(L_x0r);
            x0i = extract_l(L_x0i);
            L_tmp = Mult_32_16(wk3r,x0r); /*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk3i,x0i); /*Q(15+Qx+Q_edct) */
            a[j3] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_tmp = Mult_32_16(wk3r,x0i); /*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk3i,x0r); /*Q(15+Qx+Q_edct) */
            a[j3 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */
        }

        wk1r = w[k2 + 2];
        move32();
        wk1i = w[k2 + 3];
        move32();
        L_tmp = L_shl(Mult_32_32(wk2r,wk1i),1);/*Q29 */
        wk3r = L_sub(wk1r,L_shl(L_tmp,1)); /*Q30  */

        L_tmp = L_shl(Mult_32_32(wk2r,wk1r),1);/*Q29 */
        wk3i = L_sub(L_shl(L_tmp,1),wk1i); /*Q30 */

        tmp2 = add(l,add(k,m));
        FOR (j = add(k,m); j < tmp2; j += 2)
        {
            j1 = add(j,l);
            j2 = add(j1,l);
            j3 = add(j2,l);
            x0r = add(a[j],a[j1]);
            x0i = add(a[j + 1],a[j1 + 1]);
            x1r = sub(a[j],a[j1]);
            x1i = sub(a[j + 1],a[j1 + 1]);
            x2r = add(a[j2],a[j3]);
            x2i = add(a[j2 + 1],a[j3 + 1]);
            x3r = sub(a[j2],a[j3]);
            x3i = sub(a[j2 + 1],a[j3 + 1]);
            a[j] = add(x0r,x2r);
            move16();
            a[j + 1] = add(x0i,x2i);
            move16();

            x0r = sub(x0r,x2r);
            x0i = sub(x0i,x2i);

            tmp = negate(x0r);
            L_tmp = Mult_32_16(wk2i,tmp);/*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk2r,x0i); /*Q(15+Qx+Q_edct) */
            a[j2] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            tmp = negate(x0i);
            L_tmp = Mult_32_16(wk2i,tmp);/*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk2r,x0r); /*Q(15+Qx+Q_edct) */
            a[j2 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            x0r = sub(x1r,x3i);
            x0i = add(x1i,x3r);

            L_tmp = Mult_32_16(wk1r,x0r);/*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk1i,x0i); /*Q(15+Qx+Q_edct) */
            a[j1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_tmp = Mult_32_16(wk1r,x0i);/*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk1i,x0r); /*Q(15+Qx+Q_edct) */
            a[j1 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            x0r = add(x1r,x3i);
            x0i = sub(x1i,x3r);

            L_tmp = Mult_32_16(wk3r,x0r); /*Q(15+Qx+Q_edct) */
            L_tmp = Msub_32_16(L_tmp,wk3i,x0i); /*Q(15+Qx+Q_edct) */
            a[j3] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */

            L_tmp = Mult_32_16(wk3r,x0i); /*Q(15+Qx+Q_edct) */
            L_tmp = Madd_32_16(L_tmp,wk3i,x0r); /*Q(15+Qx+Q_edct) */
            a[j3 + 1] = round_fx(L_shl(L_tmp,1)); /*Q(Qx+Q_edct) */
        }
    }

    return;
}

void fft3_fx(const Word16 X[], Word16 Y[], const Word16 n)
{
    Word16  Z[PH_ECU_SPEC_SIZE];
    Word16 *Z0, *Z1, *Z2;
    Word16 *z0, *z1, *z2;
    const Word16 *x;
    const Word16 *t_sin = sincos_t_rad3_fx;
    Word16  m, mMinus1, step;
    Word16  i, l;
    Word16  c1_ind, s1_ind, c2_ind, s2_ind;
    Word16  c1_step, s1_step, c2_step, s2_step;
    Word16 *RY, *IY, *RZ0, *IZ0, *RZ1, *IZ1, *RZ2, *IZ2;
    Word32  acc;
    Word16  mBy2, orderMinus1;
    const Word16 *pPhaseTbl;

    /* Determine the order of the transform, the length of decimated  */
    /* transforms m, and the step for the sine and cosine tables.     */
    SWITCH(n)
    {
    case 1536:
        orderMinus1 = 9-1;
        move16();
        m = 512;
        move16();
        step = 1;
        move16();
        pPhaseTbl = FFT_W256;
        BREAK;
    case 384:
        orderMinus1 = 7-1;
        move16();
        m = 128;
        move16();
        step = 4;
        move16();
        pPhaseTbl = FFT_W64;
        BREAK;
    default:
        orderMinus1 = 7-1;
        move16();
        m = 128;
        move16();
        step = 4;
        move16();
        pPhaseTbl = FFT_W64;
        BREAK;
    }

    /* Compose decimated sequences X[3i], X[3i+1],X[3i+2] */
    /* compute their FFT of length m.                                 */
    Z0 = &Z[0];
    z0 = &Z0[0];
    Z1 = &Z0[m];
    z1 = &Z1[0];   /* Z1 = &Z[ m];     */
    Z2 = &Z1[m];
    z2 = &Z2[0];   /* Z2 = &Z[2m];     */
    x  =  &X[0];
    FOR (i = 0; i < m; i++)
    {
        *z0++ = *x++;            /* Z0[i] = X[3i];   */                         move16();
        *z1++ = *x++;            /* Z1[i] = X[3i+1]; */                         move16();
        *z2++ = *x++;            /* Z2[i] = X[3i+2]; */                         move16();
    }
    mBy2 = shr(m,1);
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, Z0, Z0, 1);
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, Z1, Z1, 1);
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, Z2, Z2, 1);

    /* Butterflies of order 3. */
    /* pointer initialization */
    mMinus1 = sub(m,1);
    RY = &Y[0];
    IY = &Y[n];
    IY--;  /* Decrement the address counter.*/
    RZ0 = &Z0[0];
    IZ0 = &Z0[mMinus1];
    RZ1 = &Z1[0];
    IZ1 = &Z1[mMinus1];
    RZ2 = &Z2[0];
    IZ2 = &Z2[mMinus1];

    c1_step = negate(step);
    s1_step = step;
    move16();
    c2_step = shl(c1_step,1);
    s2_step = shl(s1_step,1);
    c1_ind = add(T_SIN_PI_2, c1_step);
    s1_ind = s1_step;
    move16();
    c2_ind = add(T_SIN_PI_2, c2_step);
    s2_ind = s2_step;
    move16();

    /* special case: i = 0 */
    acc = L_mult(*RZ0++, 0x4000);
    acc = L_mac(acc, *RZ1++, 0x4000);
    *RY++ = mac_r(acc, *RZ2++, 0x4000);
    move16();

    /* first 3/12-- from 1 to (3*m/8)-1 */
    l = sub(shr(n, 3),1); /* (3*m/8) - 1 = (n/8) - 1 */
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0++, 15); /* Align with the following non-fractional mode so as to gain 1 more bit headroom. */
        acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_mac0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_mac0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc); /* bit growth = 1 (compensated by non-fractional mode MAC). */

        acc = L_shl(*IZ0--, 15);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_mac0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2++, t_sin[s2_ind]);
        acc = L_mac0(acc, *IZ2--, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = add(c1_ind, c1_step);
        s1_ind = add(s1_ind, s1_step);
        c2_ind = add(c2_ind, c2_step);
        s2_ind = add(s2_ind, s2_step);
    }

    /* next 1/12-- from (3*m/8) to (4*m/8)-1 */
    l = shr(m,3);  /* (4*m/8) - (3*m/8) = m/8 */
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0++, 15);
        acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_mac0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc);

        acc = L_shl(*IZ0--, 15);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_mac0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2++, t_sin[s2_ind]);
        acc = L_msu0(acc, *IZ2--, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = add(c1_ind, c1_step);
        s1_ind = add(s1_ind, s1_step);
        c2_ind = sub(c2_ind, c2_step);
        s2_ind = sub(s2_ind, s2_step);
    }

    /* special case: i = m/2 i.e. 1/3 */
    acc = L_shl(*RZ0--, 15);
    acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);
    acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
    *RY++ = round_fx(acc);

    acc = 0;
    acc = L_msu0(acc, *RZ1--, t_sin[s1_ind]);
    acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
    *IY-- = round_fx(acc);
    IZ0++;
    IZ1++;
    IZ2++;

    c1_ind = add(c1_ind, c1_step);
    s1_ind = add(s1_ind, s1_step);
    c2_ind = sub(c2_ind, c2_step);
    s2_ind = sub(s2_ind, s2_step);

    /* next  2/12-- from ((m/2)+1) to (6*m/8)-1 */
    l = sub(shr(m,2), 1);  /* (6*m/8) - ((m/2)+1) = m/4 - 1 */
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0--, 15);
        acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_msu0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc);

        acc = L_mult0(*IZ0++, -32768);
        acc = L_msu0(acc, *RZ1--, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ1++, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = add(c1_ind, c1_step);
        s1_ind = add(s1_ind, s1_step);
        c2_ind = sub(c2_ind, c2_step);
        s2_ind = sub(s2_ind, s2_step);
    }

    /*--------------------------half--------------------------// */
    /* next 2/12-- from (6*m/8) to (8*m/8) - 1 */
    l = shr(m,2);
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0--, 15);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_msu0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_mac0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc);

        acc = L_mult0(*IZ0++, -32768);
        acc = L_msu0(acc, *RZ1--, t_sin[s1_ind]);
        acc = L_mac0(acc, *IZ1++, t_sin[c1_ind]);
        acc = L_mac0(acc, *RZ2--, t_sin[s2_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = sub(c1_ind, c1_step);
        s1_ind = sub(s1_ind, s1_step);
        c2_ind = add(c2_ind, c2_step);
        s2_ind = add(s2_ind, s2_step);
    }

    /* special case: i = m, i.e 2/3 */
    acc = L_shl(*RZ0++, 15);
    acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
    acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
    *RY++ = round_fx(acc);

    acc = L_deposit_l(0);
    acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
    acc = L_mac0(acc, *RZ2++, t_sin[s2_ind]);
    *IY-- = round_fx(acc);
    IZ0--; /* Just decrement the address counter */
    IZ1--;
    IZ2--;

    c1_ind = sub(c1_ind, c1_step);
    s1_ind = sub(s1_ind, s1_step);
    c2_ind = add(c2_ind, c2_step);
    s2_ind = add(s2_ind, s2_step);

    /* next 1/12-- from (m + 1) to (9*m/8) - 1 */
    l = sub(shr(m, 3), 1);   /* (9*m/8) - (m +1) = m/8 - 1 */
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0++, 15);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc);

        acc = L_shl(*IZ0--, 15);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *RZ2++, t_sin[s2_ind]);
        acc = L_msu0(acc, *IZ2--, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = sub(c1_ind, c1_step);
        s1_ind = sub(s1_ind, s1_step);
        c2_ind = add(c2_ind, c2_step);
        s2_ind = add(s2_ind, s2_step);
    }

    /* last 3/12-- from (9*m/8) to (12*m/8) - 1 */
    l = shr(n,3);    /* (12*m/8) - (9*m/8) = 3*m/8 = n/8 */
    FOR (i = 0; i < l; i++)
    {
        acc = L_shl(*RZ0++, 15);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);  /* Non-fractional mode gains 1 more bit headroom. */
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_mac0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY++ = round_fx(acc);

        acc = L_shl(*IZ0--, 15);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *RZ2++, t_sin[s2_ind]);
        acc = L_mac0(acc, *IZ2--, t_sin[c2_ind]);
        *IY-- = round_fx(acc);

        c1_ind = sub(c1_ind, c1_step);
        s1_ind = sub(s1_ind, s1_step);
        c2_ind = sub(c2_ind, c2_step);
        s2_ind = sub(s2_ind, s2_step);
    }

    /* special case: i = 3*m/2 */
    acc = L_shl(*RZ0, 15);
    acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
    acc = L_mac0(acc, *RZ2, t_sin[c2_ind]);
    *RY = round_fx(acc);

    return;
}


void ifft3_fx(const Word16 Z[], Word16 X[], const Word16 n)
{
    Word16   Y[PH_ECU_SPEC_SIZE];
    const Word16 *t_sin = sincos_t_rad3_fx;
    Word16   m, mMinus1, step, step2;
    Word16   i, l;
    Word16   c0_ind, s0_ind, c1_ind, s1_ind, c2_ind, s2_ind;
    const Word16 *RZ0,  *IZ0,  *RZ1,  *IZ1,  *RZ2,  *IZ2;
    const Word16 *RZ00, *IZ00, *RZ10, *IZ10, *RZ20, *IZ20;
    Word16 *RY0, *IY0, *RY1, *IY1, *RY2, *IY2, *y0, *y1, *y2, *pX;
    Word32  acc;
    Word16  mBy2, orderMinus1, nMinusMBy2;
    const Word16 *pPhaseTbl;

    /* Determine the order of the transform, the length of decimated  */
    /* transforms m, and the step for the sine and cosine tables.     */
    SWITCH(n)
    {
    case 1536:
        orderMinus1 = 9-1;
        move16();
        m = 512;
        move16();
        step = 1;
        move16();
        pPhaseTbl = FFT_W256;
        BREAK;
    case 384:
        orderMinus1 = 7-1;
        move16();
        m = 128;
        move16();
        step = 4;
        move16();
        pPhaseTbl = FFT_W64;
        BREAK;
    default:
        orderMinus1 = 7-1;
        move16();
        m = 128;
        move16();
        step = 4;
        move16();
        pPhaseTbl = FFT_W64;
        BREAK;
    }

    nMinusMBy2 = shr(sub(n, m),1);
    mMinus1 = sub(m,1);
    /* pointer initialization */
    RY0 = &Y[0];
    IY0 = &Y[m];
    RY1 = &RY0[m];
    IY1 = &RY1[mMinus1];
    RY2 = &RY1[m];
    IY2 = &RY2[mMinus1];

    RZ00 = &Z[0]; /* The zero positions of the pointers */
    RZ10 = &RZ00[m];
    RZ20 = &RZ00[nMinusMBy2];
    IZ00 = &Z[n];
    IZ10 = &IZ00[-m];
    IZ20 = &IZ00[-nMinusMBy2];

    RZ0 = RZ00;  /* Reset the pointers to zero positions.  */
    RZ1 = RZ10;
    RZ2 = RZ20;
    IZ0 = IZ00;
    IZ1 = IZ10;
    IZ2 = IZ20;

    /* Inverse butterflies of order 3. */

    /* Construction of Y0 */
    acc = L_mult(*RZ0++, 0x4000);
    acc = L_mac(acc, *RZ1++, 0x4000);
    *RY0++ = mac_r(acc, *RZ2--, 0x4000);
    move16();
    IZ0--;
    IZ1--;
    IZ2++;
    IY0--;

    l = sub(shr(m, 1), 1);
    FOR (i = 0; i < l; i++)
    {
        acc = L_mult(*RZ0++, 0x4000);
        acc = L_mac(acc, *RZ1++, 0x4000);
        *RY0++ = mac_r(acc, *RZ2--, 0x4000);
        move16();

        acc = L_mult(*IZ0--, 0x4000);
        acc = L_mac(acc, *IZ1--, 0x4000);
        *IY0-- = msu_r(acc, *IZ2++, 0x4000);
        move16();
    }

    /* m/2 */
    acc = L_mult(*RZ0, 0x4000);
    acc = L_mac(acc, *RZ1, 0x4000);
    *RY0++ = mac_r(acc, *RZ2, 0x4000);
    move16();


    /* Construction of Y1 */
    c0_ind=T_SIN_PI_2;
    s0_ind=0;
    c1_ind=T_SIN_PI_2*1/3;
    s1_ind=T_SIN_PI_2*2/3;
    c2_ind=T_SIN_PI_2*1/3;
    s2_ind=T_SIN_PI_2*2/3;

    RZ0 = RZ00;  /* Reset pointers to zero positions. */
    RZ1 = RZ10;
    RZ2 = RZ20;
    IZ0 = IZ00;
    IZ1 = IZ10;
    IZ2 = IZ20;
    acc =     L_mult0(*RZ0++, t_sin[c0_ind]);
    acc = L_msu0(acc, *RZ1++, t_sin[c1_ind]);
    acc = L_msu0(acc, *RZ2--, t_sin[c2_ind]);
    IZ0--;
    acc = L_msu0(acc, *IZ1--, t_sin[s1_ind]);
    acc = L_msu0(acc, *IZ2++, t_sin[s2_ind]);
    *RY1++ = round_fx(acc);

    c0_ind=sub(c0_ind,step);
    s0_ind=add(s0_ind,step);
    c1_ind=add(c1_ind,step);
    s1_ind=sub(s1_ind,step);
    c2_ind=sub(c2_ind,step);
    s2_ind=add(s2_ind,step);

    /* From 1 to (m/4) - 1. */
    l = sub(shr(m,2),1);
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ0, t_sin[c0_ind]);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_msu0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY1++ = round_fx(acc);

        acc =     L_mult0(*IZ0--, t_sin[c0_ind]);
        acc = L_msu0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_mac0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
        *IY1-- = round_fx(acc);

        c0_ind=sub(c0_ind,step);
        s0_ind=add(s0_ind,step);
        c1_ind=add(c1_ind,step);
        s1_ind=sub(s1_ind,step);
        c2_ind=sub(c2_ind,step);
        s2_ind=add(s2_ind,step);
    }

    /* From m/4 to m/2 -1. */
    l = shr(m, 2);  /* m/2 - m/4 = m/4 */
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ0, t_sin[c0_ind]);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
        acc = L_mac0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_msu0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY1++ = round_fx(acc);

        acc =     L_mult0(*IZ0--, t_sin[c0_ind]);
        acc = L_msu0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_msu0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_mac0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
        *IY1-- = round_fx(acc);

        c0_ind=sub(c0_ind,step);
        s0_ind=add(s0_ind,step);
        c1_ind=add(c1_ind,step);
        s1_ind=sub(s1_ind,step);
        c2_ind=add(c2_ind,step);
        s2_ind=sub(s2_ind,step);
    }

    /* m/2 */
    acc =     L_mult0(*RZ0, t_sin[c0_ind]);
    acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
    acc = L_mac0(acc, *RZ2, t_sin[c2_ind]);
    acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
    acc = L_msu0(acc, *IZ1, t_sin[s1_ind]);
    acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
    *RY1++ = round_fx(acc);

    /* Construction of Y2 */
    c0_ind=T_SIN_PI_2;
    s0_ind=0;
    c1_ind=T_SIN_PI_2*1/3;
    s1_ind=T_SIN_PI_2*2/3;
    c2_ind=T_SIN_PI_2*1/3;
    s2_ind=T_SIN_PI_2*2/3;
    step2 = shl(step,1);

    RZ0 = RZ00;  /* Reset pointers to zero positions. */
    RZ1 = RZ10;
    RZ2 = RZ20;
    IZ0 = IZ00;
    IZ1 = IZ10;
    IZ2 = IZ20;
    acc =     L_mult0(*RZ0++, t_sin[c0_ind]);
    acc = L_msu0(acc, *RZ1++, t_sin[c1_ind]);
    acc = L_msu0(acc, *RZ2--, t_sin[c2_ind]);
    IZ0--;
    acc = L_mac0(acc, *IZ1--, t_sin[s1_ind]);
    acc = L_mac0(acc, *IZ2++, t_sin[s2_ind]);
    *RY2++ = round_fx(acc);

    c0_ind=sub(c0_ind,step2);
    s0_ind=add(s0_ind,step2);
    c1_ind=sub(c1_ind,step2);
    s1_ind=add(s1_ind,step2);
    c2_ind=add(c2_ind,step2);
    s2_ind=sub(s2_ind,step2);

    /* From 1 to (m/8) - 1. */
    l = sub(shr(m, 3),1);  /* m/8 - 1. */
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ0, t_sin[c0_ind]);
        acc = L_msu0(acc, *RZ1, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_mac0(acc, *IZ2, t_sin[s2_ind]);
        *RY2++ = round_fx(acc);

        acc =     L_mult0(*IZ0--, t_sin[c0_ind]);
        acc = L_msu0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_mac0(acc, *RZ2--, t_sin[s2_ind]);
        *IY2-- = round_fx(acc);

        c0_ind=sub(c0_ind,step2);
        s0_ind=add(s0_ind,step2);
        c1_ind=sub(c1_ind,step2);
        s1_ind=add(s1_ind,step2);
        c2_ind=add(c2_ind,step2);
        s2_ind=sub(s2_ind,step2);
    }

    /* From (m/8) to (m/4) - 1. */
    l = shr(m, 3);  /* m/4 - m/8 = m/8 */
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ0, t_sin[c0_ind]);
        acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_mac0(acc, *IZ2, t_sin[s2_ind]);
        *RY2++ = round_fx(acc);

        acc =     L_mult0(*IZ0--, t_sin[c0_ind]);
        acc = L_mac0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_mac0(acc, *RZ2--, t_sin[s2_ind]);
        *IY2-- = round_fx(acc);

        c0_ind=sub(c0_ind,step2);
        s0_ind=add(s0_ind,step2);
        c1_ind=add(c1_ind,step2);
        s1_ind=sub(s1_ind,step2);
        c2_ind=add(c2_ind,step2);
        s2_ind=sub(s2_ind,step2);
    }

    /* From m/4 to 3*m/8 - 1. */
    l = shr(m, 3);  /* 3*m/8 - m/4 = m/8 */
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ0, t_sin[c0_ind]);
        acc = L_mac0(acc, *RZ1, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY2++ = round_fx(acc);

        acc =     L_mult0(*IZ0--, t_sin[c0_ind]);
        acc = L_mac0(acc, *IZ1--, t_sin[c1_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
        *IY2-- = round_fx(acc);

        c0_ind=sub(c0_ind,step2);
        s0_ind=add(s0_ind,step2);
        c1_ind=add(c1_ind,step2);
        s1_ind=sub(s1_ind,step2);
        c2_ind=sub(c2_ind,step2);
        s2_ind=add(s2_ind,step2);
    }

    /* From 3*m/8 to m/2 - 1*/
    l = shr(m, 3);   /* m/2 - 3*m/8 = m/8 */
    FOR (i = 0; i < l; i++)
    {
        acc =     L_mult0(*RZ1, t_sin[c1_ind]);
        acc = L_msu0(acc, *RZ0, t_sin[c0_ind]);
        acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
        acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
        acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
        acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
        *RY2++ = round_fx(acc);

        acc =     L_mult0(*IZ1--, t_sin[c1_ind]);
        acc = L_msu0(acc, *IZ0--, t_sin[c0_ind]);
        acc = L_mac0(acc, *IZ2++, t_sin[c2_ind]);
        acc = L_mac0(acc, *RZ0++, t_sin[s0_ind]);
        acc = L_msu0(acc, *RZ1++, t_sin[s1_ind]);
        acc = L_msu0(acc, *RZ2--, t_sin[s2_ind]);
        *IY2-- = round_fx(acc);

        c0_ind=add(c0_ind,step2);
        s0_ind=sub(s0_ind,step2);
        c1_ind=add(c1_ind,step2);
        s1_ind=sub(s1_ind,step2);
        c2_ind=sub(c2_ind,step2);
        s2_ind=add(s2_ind,step2);
    }

    /* m/2 */
    acc =     L_mult0(*RZ1, t_sin[c1_ind]);
    acc = L_msu0(acc, *RZ0, t_sin[c0_ind]);
    acc = L_msu0(acc, *RZ2, t_sin[c2_ind]);
    acc = L_msu0(acc, *IZ0, t_sin[s0_ind]);
    acc = L_mac0(acc, *IZ1, t_sin[s1_ind]);
    acc = L_msu0(acc, *IZ2, t_sin[s2_ind]);
    *RY2++ = round_fx(acc);

    /* Compute the inverse FFT for all 3 blocks. */
    RY0 = &Y[0];    /* Rewind the pointers. */
    RY1 = &Y[m];
    RY2 = &RY1[m];
    mBy2 = shr(m,1);
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, RY0, RY0, 0);  /* inverse FFT */
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, RY1, RY1, 0);  /* inverse FFT */
    r_fft_fx_lc(pPhaseTbl, m, mBy2, orderMinus1, RY2, RY2, 0);  /* inverse FFT */

    y0 = RY0;
    y1 = RY1;
    y2 = RY2;

    /* Interlacing and scaling, scale = 1/3 */
    pX = X;
    FOR (i = 0; i < m; i++)
    {
        *pX++ = shl(mult_r(*y0++, FFT3_ONE_THIRD), 1);
        move16();
        *pX++ = shl(mult_r(*y1++, FFT3_ONE_THIRD), 1);
        move16();
        *pX++ = shl(mult_r(*y2++, FFT3_ONE_THIRD), 1);
        move16();
    }

    return;
}
