/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"     /* Function prototypes                    */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define INV_SQR2_FX  23170
#define N_MAX_SAS    256

/*---------------------------------------------------------------------*
  *  fft_rel_fx()
  *
  *  Computes the split-radix FFT in place for the real-valued
  *  signal x of length n.  The algorithm has been ported from
  *  the fortran code of [1].
  *
  *  The function  needs sine and cosine tables t_sin and t_cos,
  *  and the constant N_MAX_SAS.  The table  entries  are defined as
  *  sin(2*pi*i) and cos(2*pi*i) for i = 0, 1, ..., N_MAX_SAS-1. The
  *  implementation  assumes  that any entry  will not be needed
  *  outside the tables. Therefore, N_MAX_SAS and n must be properly
  *  set.  The function has been tested  with the values n = 16,
  *  32, 64, 128, 256, and N_MAX_SAS = 1280.
  *
  *  References
  *  [1] H.V. Sorensen,  D.L. Jones, M.T. Heideman, C.S. Burrus,
  *      "Real-valued fast  Fourier transform  algorithm,"  IEEE
  *      Trans. on Signal Processing,  Vol.35, No.6, pp 849-863,
  *      1987.
  *
  *  OUTPUT
  *      x[0:n-1]  Transform coeffients in the order re[0], re[1],
  *                ..., re[n/2], im[n/2-1], ..., im[1].
  *---------------------------------------------------------------------*/
/*MERGE fft_rel_fx and fft_rel_fx_lc */
void fft_rel_fx(
    Word16 x[],  /* i/o: input/output vector    */
    const Word16 n,    /* i  : vector length          */
    const Word16 m     /* i  : log2 of vector length  */
)
{
    Word16  i, j, k, n1, n2, n4;
    Word16  step;
    Word16  xt, t1, t2;
    Word16 *x0, *x1, *x2;
    const Word16 *s, *c;
    Word16 *xi2, *xi3, *xi4, *xi1;


    /*-----------------------------------------------------------------*
     * Digit reverse counter
     *-----------------------------------------------------------------*/

    j = 0;
    move16();
    x0 = &x[0];
    move16();
    FOR (i = 0; i < n-1; i++)
    {
        IF (sub(i,j) < 0)
        {
            xt   = x[j];
            move16();
            x[j] = *x0;
            move16();
            *x0  = xt;
            move16();
        }
        x0++;
        k = shr(n,1);
        WHILE (sub(k,j) <= 0)
        {
            j = sub(j,k);
            k  = shr(k,1);
        }
        j = add(j,k);
    }

    /*-----------------------------------------------------------------*
     * Length two butterflies
     *-----------------------------------------------------------------*/

    x0 = &x[0];
    move16();
    x1 = &x[1];
    move16();
    FOR (i = 0; i < n/2; i++)
    {
        xt  = *x0;
        move16();
        *x0 = add(xt,*x1);
        move16();
        *x1 = sub(xt,*x1);
        move16();
        x0++;
        x0++;
        x1++;
        x1++;
    }

    /*-----------------------------------------------------------------*
     * Other butterflies
     *
     * The implementation described in [1] has been changed by using
     * table lookup for evaluating sine and cosine functions.  The
     * variable ind and its increment step are needed to access table
     * entries.  Note that this implementation assumes n4 to be so
     * small that ind will never exceed the table.  Thus the input
     * argument n and the constant N_MAX_SAS must be set properly.
     *-----------------------------------------------------------------*/

    n2 = 1;
    move16();
    /* step = N_MAX_SAS/4; */
    FOR (k = 2; k <= m; k++)
    {
        n4 = n2;
        move16();
        n2 = shl(n4,1);
        n1 = shl(n2,1);

        step = N_MAX_SAS/n1;

        x0 = x;
        x1 = x + n2;
        x2 = x + add(n2, n4);
        FOR (i = 0; i < n; i += n1)
        {
            xt = *x0;
            move16();        /* xt = x[i];   */
            *x0 =  add(xt,*x1);
            move16();        /* x[i] = xt + x[i+n2];    */
            *x1  = sub(xt,*x1);
            move16();        /* x[i+n2] = xt - x[i+n2];      */
            *x2 = negate(*x2);
            move16();        /* x[i+n2+n4] = -x[i+n2+n4];     */


            s = sincos_t_fx + step;
            c = s + 64;
            xi1 = x + add(i, 1);
            xi3 = xi1 + n2;
            xi2 = xi3 - 2;
            xi4 = xi1 + sub(n1, 2);

            FOR (j = 1; j < n4; j++)
            {
                t1  =  add(mult_r(*xi3,*c),mult_r(*xi4,*s));    /* t1 = *xi3**(pt_c+ind) + *xi4**(pt_s+ind);   */
                t2 = sub(mult_r(*xi3,*s),mult_r(*xi4,*c));      /* t2 = *xi3**(pt_s+ind) - *xi4**(pt_c+ind);     */
                *xi4 = sub(*xi2,t2);
                move16();
                *xi3 = negate(add(*xi2,t2));
                move16();
                *xi2 =  sub(*xi1,t1);
                move16();
                *xi1 =  add(*xi1,t1);
                move16();

                xi4--;
                xi2--;
                xi3++;
                xi1++;
                c += step;
                s += step;    /* autoincrement by ar0 */
            }

            x0 += n1;
            x1 += n1;
            x2 += n1;
        }
        /* step = shr(step, 1); */
    }

    return;
}

void ifft_rel_fx(
    Word16 io[],  /* i/o: input/output vector   */
    const Word16 n,     /* i  : vector length         */
    const Word16 m      /* i  : log2 of vector length */
)
{
    Word16 i, j, k;
    Word16 step;
    Word16 n2, n4, n8, i0;
    Word16 is, id;
    Word16 *x,*xi0, *xi1, *xi2, *xi3, *xi4, *xup1, *xdn6, *xup3, *xdn8;
    Word16 xt;
    Word16 r1;
    Word16 t1, t2, t3, t4, t5;
    const Word16 *s, *c, *s3, *c3;

    Word16 cc1, cc3, ss1, ss3;
    Word16 tmp;


    /*-----------------------------------------------------------------*
     * ifft
     *-----------------------------------------------------------------*/

    x     = &io[-1];
    move16();
    n2 = shl(n,1);
    FOR (k=1; k<m; k++)
    {
        is = 0;
        move16();
        id = n2;
        move16();
        n2 = n2 >> 1;
        move16();
        n4 = n2 >> 2;
        move16();
        n8 = n4 >> 1;
        move16();
        WHILE (is < n-1)
        {
            xi1 = x + is + 1;
            move16();
            xi2 = xi1 + n4;
            move16();
            xi3 = xi2 + n4;
            move16();
            xi4 = xi3 + n4;
            move16();

            FOR (i=is; i<n; i+= id)
            {
                t1 = sub(*xi1,*xi3);
                *xi1 = add(*xi1,*xi3);
                move16();
                *xi2 = shl(*xi2,1);
                move16();
                *xi3 = sub(t1,shl(*xi4,1));
                move16();
                *xi4 = add(t1,shl(*xi4,1));
                move16();

                IF (sub(n4,1) != 0)
                {
                    t1 = mult_r(sub(*(xi2+n8),*(xi1+n8)),INV_SQR2_FX);
                    t2 = mult_r(add(*(xi4+n8),*(xi3+n8)),INV_SQR2_FX);

                    *(xi1+n8) = add(*(xi1+n8),*(xi2+n8));
                    move16();
                    *(xi2+n8) = sub(*(xi4+n8),*(xi3+n8));
                    move16();
                    *(xi3+n8) = negate(shl(add(t2,t1),1));
                    move16();
                    *(xi4+n8) = shl(sub(t1,t2),1);
                    move16();
                }
                xi1 += id;
                move16();
                xi2 += id;
                move16();
                xi3 += id;
                move16();
                xi4 += id;
                move16();
            }
            is = sub(shl(id,1),n2);
            id  = shl(id,2);
        }
        /*Can be acheived with a shr */
        step  = N_MAX_SAS/n2;
        move16();

        s = sincos_t_fx + step;
        move16();
        c = s + 64;
        move16();
        s3 = sincos_t_fx + i_mult2(step,3);
        move16();
        c3 = s3 + 64;
        move16();
        FOR (j=2; j<=n8; j++)
        {
            cc1 = *c ;
            move16();
            ss1 = *s;
            move16();
            cc3 = *c3;
            move16();
            ss3 = *s3;
            move16();

            is  = 0;
            move16();
            id  = shl(n2,1);

            c += step;
            move16();
            s += step;
            move16();

            c3 += 3*step;
            move16();
            s3 += 3*step;
            move16();
            WHILE (sub(is,sub(n,1)) < 0)
            {
                xup1 = x + j + is;
                move16();
                xup3 = xup1 + shl(n4,1);
                move16();
                xdn6 = xup3 - shl(j,1) +2;
                move16();

                xdn8 = xdn6 + shl(n4,1);
                move16();

                FOR (i=is; i<n; i+=id)
                {
                    t1 = sub(*xup1,*xdn6);
                    *xup1 = add(*xup1,*xdn6);
                    move16();
                    xup1 += n4;
                    move16();
                    xdn6 -= n4;
                    move16();

                    t2 = sub(*xdn6,*xup1);
                    *xdn6 = add(*xup1,*xdn6);
                    move16();

                    xdn6 += n4;
                    move16();
                    t3 = add(*xdn8,*xup3);
                    *xdn6  = sub(*xdn8,*xup3);
                    move16();

                    xup3 += n4;
                    move16();
                    xdn8 -= n4;
                    move16();

                    t4 =  add(*xup3,*xdn8);
                    *xup1= sub(*xup3,*xdn8);
                    move16();

                    t5 = sub(t1,t4);
                    t1 = add(t1,t4);
                    t4 = sub(t2,t3);
                    t2 = add(t2,t3);
                    *xup3 = sub(mult_r(t1,cc3),mult_r(t2,ss3));
                    move16();
                    xup3 -= n4;
                    move16();
                    *xup3 = add(mult_r(t5,cc1),mult_r(t4,ss1));
                    move16();
                    *xdn8 = sub(mult_r(t5,ss1),mult_r(t4,cc1));
                    move16();

                    xdn8 += n4;
                    move16();
                    *xdn8 = add(mult_r(t2,cc3),mult_r(t1,ss3));
                    move16();

                    xup1 -= n4;
                    move16();
                    xup1 += id;
                    move16();
                    xup3 += id;
                    move16();
                    xdn6 += id;
                    move16();
                    xdn8 += id;
                    move16();
                }
                is = sub(shl(id,1),n2);
                id = shl(id,2);
            }
        }
    }

    /*-----------------------------------------------------------------*
     * Length two butterflies
     *-----------------------------------------------------------------*/

    is = 1;
    move16();
    id = 4;
    move16();
    WHILE (is < n)
    {
        xi0 = x + is ;
        move16();
        xi1 = xi0 + 1;
        move16();

        FOR (i0=is; i0<=n; i0+=id)
        {
            r1 = *xi0;
            move16();
            *xi0= add(r1,*xi1);
            move16();
            *xi1 = sub(r1,*xi1);
            move16();
            xi0 += id;
            move16();
            xi1 += id;
            move16();
        }
        is = sub(shl(id,1),1);
        id = shl(id,2);
    }

    /*-----------------------------------------------------------------*
     * Digit reverse counter
     *-----------------------------------------------------------------*/

    j = 1;
    move16();
    FOR (i=1; i<n; i++)
    {
        IF (sub(i,j) < 0)
        {
            xt = x[j];
            move16();
            x[j] = x[i];
            move16();
            x[i] = xt;
            move16();
        }
        k = shr(n, 1);
        WHILE (sub(k,j) < 0)
        {
            j = sub(j,k);
            k =shr(k, 1);
        }
        j = add(j,k);
    }

    /*-----------------------------------------------------------------*
     * Normalization
     *-----------------------------------------------------------------*/

    tmp = div_s(1,n); /*Q15 */
    FOR (i=1; i<=n; i++)
    {
        x[i] = mult_r(x[i],tmp);
        move16();
    }

    return;
}
