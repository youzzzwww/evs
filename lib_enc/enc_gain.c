/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <memory.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include "stl.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "basop_util.h"


/*
 * E_GAIN_norm_corr
 *
 * Parameters:
 *    exc            I: excitation buffer (Q_new)
 *    xn             I: target signal (Q_new+shift-1)
 *    h              I: weighted synthesis filter impulse response (Q14+shift)
 *    t0_min         I: minimum value in the searched range
 *    t0_max         I: maximum value in the searched range
 *    corr_norm      O: normalized correlation (Q15+(Q_new+shift-1)+scale)
 *
 * Function:
 *    Find the normalized correlation between the target vector and the
 *    filtered past excitation (correlation between target and filtered
 *    excitation divided by the square root of energy of filtered excitation)
 *    Size of subframe = L_SUBFR.
 *
 * Returns:
 *    void
 */
void E_GAIN_norm_corr(Word16 exc[], Word16 xn[], Word16 h[],
                      Word16 t_min, Word16 t_max, Word16 corr_norm[], Word16 L_subfr)
{
    Word16 excf[L_SUBFR];  /* filtered past excitation (Q_new+shift-1) */
    Word16 ps, norm, exp_alp, exp_ps, scale, L_subfr2;
    Word16 t, j, k;
    Word32 L_tmp, L_tmp2;


    k = negate(t_min);
    L_subfr2 = shr(L_subfr, 1);

    /* compute the filtered excitation for the first delay t_min */
    E_UTIL_f_convolve(&exc[k], h, excf,L_subfr);

    /* Compute rounded down 1/sqrt(energy of xn[]) */
    Dot_product12_offs(xn, xn, L_subfr, &scale, 1);

    scale = add(scale, 2+1);          /* energy of xn[] x 2 + rounded up     */
    scale = negate(shr(scale, 1));    /* (1<<scale) < 1/sqrt(energy rounded) */

    /* loop for every possible period */
    FOR (t = t_min; t < t_max; t++)
    {
        /* Compute correlation between xn[] and excf[] */

        /*for (j = 0; j < L_subfr; j++)
        {
           ps  += xn[j] * excf[j];                                          MAC(1);
           alp += excf[j] * excf[j];                                        MAC(1);
        }*/
        L_tmp = Dot_product12_offs(xn, excf, L_subfr, &exp_ps, 1);
        ps = extract_h(L_tmp); /* Scaling of ps = 15 + 2x(Q_new+shift-1) - exp_ps */


        /* Compute 1/sqrt(energy of excf[]) */
        L_tmp = Dot_product12_offs(excf, excf, L_subfr2, NULL, 1);
        L_tmp2 = Dot_product12_offs(excf+L_subfr2, excf+L_subfr2, L_subfr2, NULL, 1);
        exp_alp = sub(s_min(norm_l(L_tmp), norm_l(L_tmp2)), 1);
        L_tmp = L_add(L_shl(L_tmp, exp_alp), L_shl(L_tmp2, exp_alp));
        exp_alp = sub(31+1, exp_alp);

        /*norm = (Float32)(1.0F / sqrt(alp));                                 SQRT(1);*/
        L_tmp = ISqrt32(L_tmp, &exp_alp);
        norm = extract_h(L_tmp); /* Scaling of norm = 15 - (Q_new+shift-1) - exp_alp */

        /* Normalize correlation = correlation * (1/sqrt(energy)) */
        /*corr_norm[t-t_min] = ps * norm;                                     MULT(1); STORE(1);*/
        L_tmp = L_mult(ps, norm);
        L_tmp = L_shl(L_tmp, add(add(exp_ps, exp_alp), scale));

        corr_norm[t-t_min] = round_fx(L_tmp); /* Scaling of corr_norm = 15 + (Q_new+shift-1) + scale */

        /* update the filtered excitation excf[] for the next iteration */
        k = sub(k,1);

        FOR(j = L_subfr - 1; j > 0; j--)
        {
            /*excf[j] = excf[j - 1] + exc[k] * h[j];                           MAC(1); STORE(1);*/
            /* saturation can occur in add() */
            excf[j] = add(mult_r(exc[k], h[j]), excf[j - 1]);
            move16();

        }
        excf[0] = mult_r(exc[k],h[0]);
        move16();
    }
    /* Last reduced iteration for t=t_max */
    L_tmp = Dot_product12_offs(xn, excf, L_subfr, &exp_ps, 1);
    ps = extract_h(L_tmp);


    /* Compute 1/sqrt(energy of excf[]) */
    L_tmp = Dot_product12_offs(excf, excf, L_subfr2, NULL, 1);
    L_tmp2 = Dot_product12_offs(excf+L_subfr2, excf+L_subfr2, L_subfr2, NULL, 1);
    exp_alp = sub(s_min(norm_l(L_tmp), norm_l(L_tmp2)), 1);
    L_tmp = L_add(L_shl(L_tmp, exp_alp), L_shl(L_tmp2, exp_alp));
    exp_alp = sub(31+1, exp_alp);


    /*norm = (Float32)(1.0F / sqrt(alp));                                 SQRT(1);*/
    L_tmp = ISqrt32(L_tmp, &exp_alp);
    norm = extract_h(L_tmp);

    /* Normalize correlation = correlation * (1/sqrt(energy)) */
    /*corr_norm[t-t_min] = ps * norm;                                     MULT(1); STORE(1);*/
    L_tmp = L_mult(ps, norm);
    L_tmp = L_shl(L_tmp, add(add(exp_ps, exp_alp), scale));
    corr_norm[t-t_min] = round_fx(L_tmp);

    return;
}


/*
 * E_GAIN_norm_corr_interpolate
 *
 * Parameters:
 *    x           I: input vector
 *    frac        I: fraction (-4..+3)
 *
 * Function:
 *    Interpolating the normalized correlation
 *
 * Returns:
 *    interpolated value
 */
static Word16 E_GAIN_norm_corr_interpolate(Word16 *x, Word16 frac)
{
    Word16 *x1, *x2, i;
    const Word16 *c1, *c2;
    Word32 s;

    IF (frac < 0)
    {
        frac = add(frac,4);
        x--;
    }

    x1 = &x[0];
    x2 = &x[1];
    c1 = &E_ROM_inter4_1[frac];
    c2 = &E_ROM_inter4_1[4 - frac];

    /*s = x1[0] * c1[0] + x2[0] * c2[0];
    s += x1[-1] * c1[4] + x2[1] * c2[4];
    s += x1[-2] * c1[8] + x2[2] * c2[8];
    s += x1[-3] * c1[12] + x2[3] * c2[12];*/

    s = L_deposit_l(0);
    FOR (i = 0; i < 4; i++)
    {
        s = L_mac(s,*x1--,*c1);
        s = L_mac(s,*x2++,*c2);
        c1 += 4;
        c2 += 4;
    }

    return round_fx(L_shl(s, 1));
}

static Word16 E_GAIN_norm_corr_interpolate6(Word16 *x, Word16 frac)
{
    Word16 *x1, *x2, i;
    const Word16 *c1, *c2;
    Word32 s;

    IF (frac < 0)
    {
        frac = add(frac,6);
        x--;
    }

    x1 = &x[0];
    x2 = &x[1];
    c1 = &E_ROM_inter6_1[frac];
    c2 = &E_ROM_inter6_1[6 - frac];

    /*s = x1[0] * c1[0] + x2[0] * c2[0];
    s += x1[-1] * c1[6] + x2[1] * c2[6];
    s += x1[-2] * c1[12] + x2[2] * c2[12];
    s += x1[-3] * c1[18] + x2[3] * c2[18];*/
    s = L_deposit_l(0);
    FOR (i = 0; i < 4; i++)
    {
        s = L_mac(s,*x1--,*c1);
        s = L_mac(s,*x2++,*c2);
        c1 += 6;
        c2 += 6;
    }

    return round_fx(L_shl(s, 1));
}

/*
 * E_GAIN_closed_loop_search
 *
 * Parameters:
 *    exc            I: excitation buffer
 *    xn             I: target signal
 *    h              I: weighted synthesis filter impulse response
 *    dn             I: residual domain target signal
 *    t0_min         I: minimum value in the searched range
 *    t0_max         I: maximum value in the searched range
 *    pit_frac       O: chosen fraction
 *    i_subfr        I: flag to first subframe
 *    t0_fr2         I: minimum value for resolution 1/2
 *    t0_fr1         I: minimum value for resolution 1
 *
 * Function:
 *    Find the closed loop pitch period with 1/4 subsample resolution.
 *
 * Returns:
 *    chosen integer pitch lag
 */
Word16 E_GAIN_closed_loop_search(Word16 exc[],
                                 Word16 xn[], Word16 h[],
                                 Word16 t0_min, Word16 t0_min_frac, Word16 t0_max, Word16 t0_max_frac, Word16 t0_min_max_res, Word16 *pit_frac, Word16 *pit_res, Word16 pit_res_max,
                                 Word16 i_subfr, Word16 pit_min, Word16 pit_fr2, Word16 pit_fr1, Word16 L_subfr)
{
    Word16 corr_v[32 + 2 * L_INTERPOL1 + 1];
    Word16 cor_max, max, temp;
    Word16 *corr;
    Word16 i, fraction, frac1, frac2, step;
    Word16 t0, t_min, t_max;

    set16_fx(corr_v, 0, 32 + 2 * L_INTERPOL1 + 1);
    /* Find interval to compute normalized correlation */
    if (t0_min_frac>0)
    {
        t0_min = add(t0_min,1);
    }
    t_min = sub(t0_min,L_INTERPOL1);
    t_max = add(t0_max,L_INTERPOL1);

    /* allocate memory to normalized correlation vector */
    corr = &corr_v[negate(t_min)];      /* corr[t_min..t_max] */

    /* Compute normalized correlation between target and filtered excitation */
    E_GAIN_norm_corr(exc, xn, h, t_min, t_max, corr_v, L_subfr);

    /*  find integer pitch */
    max = corr[t0_min];
    move16();
    t0  = t0_min;
    move16();

    FOR(i = t0_min + 1; i <= t0_max; i++)
    {
        BASOP_SATURATE_WARNING_OFF;
        if( sub(corr[i],max) >= 0)
        {
            t0 = i;
            move16();
        }
        max = s_max(max, corr[i]);
        BASOP_SATURATE_WARNING_ON;
    }



    /* If first subframe and t0 >= pit_fr1, do not search fractionnal pitch */
    test();
    IF((i_subfr == 0) && sub(t0,pit_fr1) >= 0)
    {
        *pit_frac = 0;
        move16();
        *pit_res = 1;
        move16();
        return(t0);
    }


    /*
     * Search fractionnal pitch
     * Test the fractions around t0 and choose the one which maximizes
     * the interpolated normalized correlation.
     */

    IF ( sub(t0_min_max_res,shr(pit_res_max,1))  == 0)
    {
        t0_min_frac = shl(t0_min_frac,1);
        t0_max_frac = shl(t0_max_frac,1);
    }

    step = 1;
    frac1 = sub(1,pit_res_max);
    frac2 = sub(pit_res_max,1);

    test();
    test();
    IF (((i_subfr == 0) && sub(t0,pit_fr2) >= 0) || sub(pit_fr2,pit_min) <= 0)
    {
        step = 2;
        frac1 = sub(2,pit_res_max);
        frac2 = sub(pit_res_max,2);
    }
    test();
    IF ( (sub(t0,t0_min) == 0) && (t0_min_frac==0) )
    {
        frac1 = t0_min_frac;
        move16();
    }
    ELSE
    {
        test();
        IF ( (sub(t0,t0_min) == 0) && (sub(add(frac1,pit_res_max),t0_min_frac)<0) )
        {
            frac1 = sub(t0_min_frac,pit_res_max);
        }
    }

    if (sub(t0,t0_max) == 0)
    {
        frac2 = t0_max_frac;
        move16();
    }
    assert(frac1<=0 && frac2>=0 && frac2>frac1);

    IF (sub(pit_res_max,6) == 0)
    {
        cor_max = E_GAIN_norm_corr_interpolate6(&corr[t0], frac1);
        fraction = frac1;

        FOR (i = (frac1 + step); i <= frac2; i += step)
        {
            temp = E_GAIN_norm_corr_interpolate6(&corr[t0], i);
            IF (sub(temp,cor_max) > 0)
            {
                cor_max = temp;
                move16();
                fraction = i;
                move16();
            }

        }
    }
    ELSE
    {
        cor_max = E_GAIN_norm_corr_interpolate(&corr[t0], frac1);
        fraction = frac1;

        FOR (i = (frac1 + step); i <= frac2; i += step)
        {
            temp = E_GAIN_norm_corr_interpolate(&corr[t0], i);
            IF (sub(temp,cor_max) > 0)
            {
                cor_max = temp;
                move16();
                fraction = i;
                move16();
            }

        }
    }

    /* limit the fraction value */
    IF (fraction < 0)
    {
        fraction = add(fraction,pit_res_max);
        t0 = sub(t0,1);
    }
    test();
    test();
    IF (((i_subfr == 0) && sub(t0,pit_fr2) >= 0) || sub(pit_fr2,pit_min) <= 0)
    {
        *pit_res = shr(pit_res_max,1);
        move16();
        *pit_frac = shr(fraction,1);
        move16();
    }
    ELSE
    {
        *pit_res = pit_res_max;
        move16();
        *pit_frac = fraction;
        move16();
    }
    return (t0);
}

