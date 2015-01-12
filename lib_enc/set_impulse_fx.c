/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-----------------------------------------------------------------*
 * Local constant
 *-----------------------------------------------------------------*/
#define INPOL 4 /* +- range in samples for impulse position searching */

/*-----------------------------------------------------------------*
 * Local function prototype
 *-----------------------------------------------------------------*/
static void convolve_tc_fx(const Word16 g[],const Word16 h[],Word16 y[],const Word16 L_1,const Word16 L_2);
static void correlate_tc_fx(const Word16 *x, Word16 *y, const Word16 *h, const Word16 start, const Word16 L_1,const Word16 L_2);
static void convolve_tc2_fx(const Word16 g[],const Word16 h[], Word16 y[], const Word16 pos_max);

/*---------------------------------------------------------------------------------------*
 * Function  set_impulse() for TC                                                        *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                                        *
 * Builds glottal codebook contribution based on glotal impulses positions finding.      *
 *                                                                                       *
 * Returns a position of the glotal impulse center and                                   *
 * a number of the glotal impulse shape.                                                 *
 *                                                                                       *
 *               |----|              |----|                                   xn         *
 *     imp_pos-> ||   |  imp_shape-> | g1 |                                   |          *
 *               | |  |              | g2 |  exc    |---|    y1  ----         |          *
 *               |  | |--------------|    |---------| h |-------|gain|-------(-)---> xn2 *
 *               |   ||              | gn |         |---|        ----                    *
 *               |----|              |----|                                              *
 *              codebook           excitation       h_orig       gain                    *
 *                                                                                       *
 *                                                                                       *
 *                                 nominator      dd    <xn,y1>*<xn,y1>                  *
 * Searching criterion: maximize ------------- = ---- = -----------------                *
 *                                denominator     rr        <y1,y1>                      *
 *                                                                                       *
 * Notice: gain = gain_trans * gain_pit (computed in trans_enc() function)               *
 *                                                                                       *
 *---------------------------------------------------------------------------------------*/
void set_impulse_fx(
    const Word16 xn_fx[],        /* i  :  target signal                                 */
    const Word16 h_orig_fx[],    /* i  :  impulse response of weighted synthesis filter */
    Word16 exc_fx[],       /* o  :  adaptive codebook excitation                  */
    Word16 yy1_fx[],       /* o  :  filtered adaptive codebook excitation         */
    Word16 *imp_shape,     /* o  :  adaptive codebook index                       */
    Word16 *imp_pos,       /* o  :  position of the glotal impulse center index   */
    Word32 *gain_trans_fx, /* o  :  transition gain                         Q7    */
    Word16 Q_new           /* i  :  Current scaling                               */
)
{
    Word16 i, j, m;
    Word16 start1, start2, end1;
    Word32 rr_fx[L_SUBFR];        /*  criterion:  nominator coeficients  */
    Word16 dd_fx[L_SUBFR],tmp16;  /*  criterion: denominator coeficients */
    Word16 gh_fx[L_SUBFR], num, den, exp_num, exp_den;  /*  convolution of 'g' and 'h' filters */
    Word16 krit_fx, krit_max_fx, gain16;
    Word32 Lrr, Ldd, Ltmp,Ltmp1;
    const Word16 *pt_Glt;
    krit_max_fx = -32768;
    move16();

    /* main loop */
    /*  impulse  */
    FOR (m = 0; m < NUM_IMPULSE; m++)
    {
        /* set searching ranges */
        IF (sub(*imp_pos, L_SUBFR-INPOL) < 0)
        {
            end1 = add(*imp_pos, INPOL);
        }
        ELSE
        {
            end1 = L_SUBFR;
            move16();
        }
        IF (sub(*imp_pos, INPOL) > 0)
        {
            start1 = sub(*imp_pos, INPOL);
        }
        ELSE
        {
            start1 = 0;
            move16();
        }
        IF (sub(start1, L_IMPULSE2)>0)
        {
            start2 = start1;
            move16();
        }
        ELSE
        {
            start2 = L_IMPULSE2;
            move16();
        }

        /*-----------------------------------------------------------*
         *   nominator & DEnominator, gh=convolve(g,h)
         *-----------------------------------------------------------*/
        IF (sub(start1, L_IMPULSE2) <0 )
        {
            Lrr = L_deposit_l(0);
            Ldd = L_deposit_l(0);
            convolve_tc_fx(&Glottal_cdbk_fx[m*L_IMPULSE+L_IMPULSE2-start1],
                           &h_orig_fx[0], gh_fx, add(L_IMPULSE-L_IMPULSE2, start1), L_SUBFR);

            /* nominator & DEnominator row <0> */
            FOR (i=0; i < L_SUBFR; i++)
            {
                Lrr = L_mac(Lrr, gh_fx[i], gh_fx[i]);
                Ldd = L_mac(Ldd, gh_fx[i], xn_fx[i]);
            }
            rr_fx[start1] = Lrr;
            move32();
            dd_fx[start1] = round_fx(Ldd);
            rr_fx[start1] = L_max(rr_fx[start1], 1);

            FOR (i=add(start1, 1); i<L_IMPULSE2; i++)
            {
                Lrr = L_deposit_l(0);
                Ldd = L_deposit_l(0);
                /* DEnominator rows <1,L_IMPULSE2-1> */
                FOR (j = L_SUBFR-1; j > 0; j--)
                {
                    /* gh_fx[j] = gh_fx[j-1] + glottal_cdbk[m*L_IMPULSE+L_IMPULSE2-i]*h_orig_fx[j] */
                    gh_fx[j] = mac_r(L_deposit_h(gh_fx[j-1]),
                                     Glottal_cdbk_fx[m*L_IMPULSE+L_IMPULSE2-i], h_orig_fx[j]);
                    move16();
                    Lrr = L_mac(Lrr, gh_fx[j], gh_fx[j]);
                    Ldd = L_mac(Ldd, gh_fx[j], xn_fx[j]);
                }

                gh_fx[0]  = mult_r(Glottal_cdbk_fx[m*L_IMPULSE+L_IMPULSE2-i],h_orig_fx[0]);
                move16();
                Lrr = L_mac(Lrr, gh_fx[0], gh_fx[0]);
                Ldd = L_mac(Ldd, gh_fx[0], xn_fx[0]);
                dd_fx[i] = round_fx(Ldd);
                rr_fx[i] = L_max(Lrr, 1);
                move32();
                /* move rr and dd into rr[i] and dd[i] */
            }

            /* complete convolution(excitation,h_orig) */
            FOR (j=L_SUBFR-1; j > 0; j--)
            {
                gh_fx[j]  = mac_r(L_deposit_h(gh_fx[j-1]),
                                  Glottal_cdbk_fx[m*L_IMPULSE], h_orig_fx[j]);
            }
        }
        ELSE
        {
            convolve_tc_fx(&Glottal_cdbk_fx[m*L_IMPULSE], h_orig_fx, gh_fx, L_IMPULSE, L_SUBFR);
        }

        IF (sub(end1, start2) >= 0)
        {
            /* DEnominator row <L_SUBFR-1> */
            Lrr = L_mult(gh_fx[0], gh_fx[0]);
            FOR (j=1; j <= L_IMPULSE2; j++)
            {
                /*rr[L_SUBFR-1] += gh[j]*gh[j];*/
                Lrr = L_mac(Lrr, gh_fx[j], gh_fx[j]);
            }
            rr_fx[L_SUBFR-1] = Lrr;
            move32();
            /* DEnominator rows <L_IMPULSE2,L_SUBFR-2> */
            FOR (i = L_SUBFR-2; i >= start2; i--)
            {
                /*rr[i] = rr[i+1] + gh[L_SUBFR+L_IMPULSE2-1-i]*gh[L_SUBFR+L_IMPULSE2-1-i];*/
                rr_fx[i] = L_mac(rr_fx[i+1], gh_fx[L_SUBFR+L_IMPULSE2-1-i],
                                 gh_fx[L_SUBFR+L_IMPULSE2-1-i]);
                move32();
            }
            /* nominator rows <L_IMPULSE2,L_SUBFR-1> */
            correlate_tc_fx(xn_fx, &dd_fx[L_IMPULSE2], gh_fx, sub(start2, L_IMPULSE2),
                            L_SUBFR, sub(end1, L_IMPULSE2));
        }
        /*------------------------------------------------------------*
         *    maxim. criterion
         *------------------------------------------------------------*/
        FOR (i = start1; i < end1; i++)
        {
            /* krit = (float)(dd[i]*dd[i])/rr[i] */
            exp_num = sub(norm_s(dd_fx[i]), 1);
            num = shl(dd_fx[i], exp_num);
            num = mult_r(num, num);

            exp_den = norm_l(rr_fx[i]);
            den  = extract_h(L_shl(rr_fx[i], exp_den));

            num = div_s(num,den);
            krit_fx = shr(num, sub(sub(shl(exp_num, 1), exp_den), 2)); /* Q18 */

            IF (sub(krit_fx, krit_max_fx) > 0)
            {
                krit_max_fx = krit_fx;
                move16();
                *imp_pos = i;
                move16();
                *imp_shape = m;
                move16();
            }
        }
    }

    /*---------------------------------------------------------------*
     *    Build the excitation using found codeword
     *---------------------------------------------------------------*/

    set16_fx(exc_fx, 0, L_SUBFR);
    set16_fx(yy1_fx, 0, L_SUBFR);
    tmp16 = sub(extract_l(L_mac0(L_IMPULSE2, *imp_shape, L_IMPULSE)), *imp_pos);
    pt_Glt = &Glottal_cdbk_fx[tmp16];
    move16();
    j = add(*imp_pos, L_IMPULSE2);
    FOR (i = sub(*imp_pos, L_IMPULSE2); i <= j; i++)
    {
        test();
        if (i >= 0 && sub(i, L_SUBFR) < 0)
        {
            exc_fx[i] = pt_Glt[i];
            move16();/*Q13*/
        }
    }

    /*---------------------------------------------------------------*
     *    Form filtered excitation, find gain_trans
     *---------------------------------------------------------------*/
    convolve_tc2_fx(exc_fx, h_orig_fx, yy1_fx, *imp_pos);

    /* Find the ACELP correlations and the pitch gain (for current subframe) */
    /**gain_trans = dot_product( xn, yy1, L_SUBFR )/(dot_product( yy1, yy1, L_SUBFR ) + 0.01f);*/
    /* Compute scalar product <y1[],y1[]> */
    Ltmp = Dot_product(yy1_fx, yy1_fx, L_SUBFR);
    exp_den = norm_l(Ltmp);
    den = extract_h(L_shl(Ltmp, exp_den));

    /* Compute scalar product <xn[],y1[]> */
    Ltmp1 = Dot_product(xn_fx, yy1_fx, L_SUBFR);
    exp_num = sub(norm_l(Ltmp1), 1);
    num = extract_h(L_shl(Ltmp1, exp_num));
    tmp16 = s_or(shr(num, 16), 1);        /* extract sign if num < 0 tmp16 = -1 else tmp16 = 1 */
    num = abs_s(num);

    /* compute gain = xy/yy */
    gain16= div_s(num, den);

    i = add(exp_num, sub(Q_new, 1+1+3));
    i = sub(i, exp_den); /* Gain_trans in Q7 */
    gain16 = i_mult2(gain16, tmp16);/* apply sign */
    *gain_trans_fx = L_shr(L_deposit_l(gain16), i);
    move32();

}
/*-------------------------------------------------------------------*
 *    convolve_tc_fx:
 *
 *   convolution for different vectors' lengths
 *-------------------------------------------------------------------*/
static void convolve_tc_fx(
    const Word16 g[],       /* i  :  input vector                                Qx      */
    const Word16 h[],       /* i  :  impulse response (or second input vector)   Q15     */
    Word16 y[],       /* o  :  output vetor (result of convolution)        12 bits */
    const Word16 L_1,       /* i  :  vector h size                                       */
    const Word16 L_2        /* i  :  vector g size                                       */
)
{
    Word16 i, n, len;
    Word32 L_sum;

    FOR (n = 0; n < L_2; n++)
    {
        len = s_min(add(n,1), L_1);
        L_sum = L_mult(g[0], h[n]);
        FOR (i = 1; i < len; i++)
        {
            L_sum = L_mac(L_sum, g[i], h[n - i]);
        }

        y[n] = round_fx(L_sum); /* Q12*/
    }
}
/*-------------------------------------------------------------------*
 * convolve_tc2_fx:
 *
 *   convolution for one vector with only L_IMPULSE nonzero coefficients
 *-------------------------------------------------------------------*/
static void convolve_tc2_fx(
    const Word16 g[],     /* i  : input vector                                Qx      */
    const Word16 h[],     /* i  : impulse response (or second input vector)   Q15     */
    Word16 y[],     /* o  : output vetor (result of convolution)        12 bits */
    const Word16 pos_max  /* o  : artificial impulse position                         */
)
{
    Word32 temp;
    Word16 i, n;
    Word16 i_start, i_end, i_end2;

    i_start = sub(pos_max,L_IMPULSE2);
    i_start  = s_max(i_start, 0);

    i_end = add(pos_max, L_IMPULSE);
    i_end  = s_min(i_end, L_SUBFR);

    FOR (n = i_start; n < L_SUBFR; n++)
    {
        temp = L_mult(g[0], h[n]);
        i_end2 = s_min(add(n, 1), i_end);

        FOR (i = 1; i < i_end2; i++)
        {
            temp = L_mac(temp, g[i], h[n-i]);
        }
        y[n] = round_fx(temp);
    }
}
/*-------------------------------------------------------------------*
 * correlate_tc:
 *
 *   correlation for different vectors' lengths
 *-------------------------------------------------------------------*/
static void correlate_tc_fx(
    const Word16 *x,     /* i: target signal                                   */
    Word16 *y,     /* o: correlation between x[] and h[]  -Q3               */
    const Word16 *h,     /* i: impulse response (of weighted synthesis filter) */
    const Word16 start,  /* i: index of iterest                                */
    const Word16 L_1,    /* i: vector size                                     */
    const Word16 L_2     /* i: index of interest                               */
)
{
    Word16 i, j;
    Word32 s;

    FOR (i = start; i < L_2; i++)
    {
        s = L_deposit_l(0);
        FOR (j = i; j < L_1; j++)
        {
            s = L_mac(s, x[j],h[j-i]);
        }
        y[i] = round_fx(s);
    }
}
