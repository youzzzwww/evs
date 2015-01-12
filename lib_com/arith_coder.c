/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"

/* Fixed point implementation of exp(negate()) */
Word32 expfp(  /* o: Q31 */
    Word16 x,           /* i: mantissa  Q-e */
    Word16 x_e)         /* i: exponent  Q0  */
{
    Word16 xi, xf, tmp;
    Word16 b0, b1, b2, b3;
    Word32 y, L_tmp;


    assert(x > 0);

    L_tmp = L_shl(L_deposit_h(x), x_e);

    /* split into integer and fractional parts */
    xi = round_fx(L_tmp);
    xf = extract_l(L_tmp);

    BASOP_SATURATE_WARNING_OFF;
    xf = negate(xf);
    BASOP_SATURATE_WARNING_ON;

    /* Fractional part */
    /* y = 65536
            +         xf
            +       ((xf*xf) / (2*65536))
            +   ((((((xf*xf) / (2*65536))*xf) / 65536)*65536/3) / 65536)
            + ((((((((xf*xf) / (2*65536))*xf) / 65536)*65536/3) / 65536)*xf) / (4*65536)); */
    y = L_mac0(65536, xf, 1);
    tmp = shr(mult(xf, xf), 2);
    y = L_mac0(y, tmp, 1);
    tmp = shr(mult(shr(mult(tmp, xf), 1), 65536/3), 1);
    y = L_mac0(y, tmp, 1);
    tmp = shr(mult(tmp, xf), 3);
    y = L_mac0(y, tmp, 1);

    /* Integer part */
    b0 = s_and(xi, 1);
    b1 = s_and(xi, 2);
    b2 = s_and(xi, 4);
    b3 = s_and(xi, 8);

    if (b0 != 0) y = Mpy_32_16_1(y, 24109);   /* exp(-1) in -1Q16 */
    if (b1 != 0) y = Mpy_32_16_1(y, 17739);   /* exp(-2) in -2Q17 */
    if (b2 != 0) y = Mpy_32_16_1(y, 19205);   /* exp(-4) in -5Q20 */
    if (b3 != 0) y = Mpy_32_16_1(y, 22513);   /* exp(-8) in -11Q26 */

    /* scaling: -1*b0 - 2*b1 -5*b2 -11*b3 */
    y = L_shr(y, add(add(xi, shr(xi, 2)), shr(b3, 3)));

    /* zero for xi >= 16 */
    if (shr(xi, 4) > 0)
    {
        y = L_deposit_l(0);
    }


    return L_shl(y, 15);
}

/* Fixed point implementation of pow(), where base is fixed point (16/16) and exponent a small *odd* integer
 *
 * Returns: *pout1 = ( (base/65536)^(2*exp - 1) ) * 65536
 *          *pout2 = ( (base/65536)^(2*exp + 1) ) * 65536
 *
 * NOTE: This function must be in sync with ari_decode_14bits_pow() */
void powfp_odd2(Word16 base,     /* Q15 */
                Word16 exp,      /* Q0  */
                Word16 *pout1,   /* Q15 */
                Word16 *pout2)   /* Q15 */
{
    /* this version is in sync with ari_enc_14bits_pow()
     * that is, we have to start multiplication from the largest power-of-two, in order to
     * get the rounding errors to appear at the same places */
    Word16 pows[12];    /* powers of two exponents*/
    Word16 exp2;
    Word16 out, out2;
    Word16 k, h, maxk;

    assert(exp >= 0);

    out = base;
    move16();
    out2 = 0x7FFF;
    move16();
    IF (exp != 0)
    {
        exp2 = sub(exp, 1);
        maxk = sub(15, norm_s(exp));
        assert(maxk < 12);

        pows[0] = base;
        move16();
        FOR (k = 0; k < maxk; k++)
        {
            pows[k+1] = mult_r(pows[k], pows[k]);
            move16();
        }
        k = sub(k, 1);
        h = shl(1, k);      /* highest bit of exp2 */
        out2 = base;
        move16();
        out = mult_r(out, pows[k+1]);   /* we already know that "exp" has the highest bit set to one since we calculated .. */
        /* .. the effective length of "exp" earlier on, thus we omit the branch for out2 */
        if (s_and(exp2, h) != 0)
        {
            out2 = mult_r(out2, pows[k+1]);
        }

        h = shr(h, 1);
        FOR (k = sub(k, 1); k >= 0; k--)
        {
            if (s_and(exp, h) != 0)
            {
                out = mult_r(out, pows[k+1]);
            }

            if (s_and(exp2, h) != 0)
            {
                out2 = mult_r(out2, pows[k+1]);
            }

            h = shr(h, 1);
        }
    }

    *pout1 = out2;
    move16();
    *pout2 = out;
    move16();

}

/*------------------------------------------------------------------------
 * Function: tcx_arith_scale_envelope
 *
 * For optimal performance of the arithmetic coder, the envelope shape must
 * be scaled such that the expected bit-consumption of a signal that
 * follows the scaled shape coincides with the target bitrate.
 * This function calculates a first-guess scaling and then uses the bi-section
 * search to find the optimal scaling.
 *
 * We assume that lines follow the Laplacian distribution, whereby the expected
 * bit-consumption would be log2(2*e*s[k]), where s[k] is the envelope value
 * for the line in question. However, this theoretical formula assumes that
 * all lines are encoded with magnitude+sign. Since the sign is unnecessary
 * for 0-values, that estimate of bit-consumption is biased when s[k] is small.
 * Analytical solution of the expectation for small s[k] is difficult, whereby
 * we use the approximation log2(2*e*s[k] + 0.15 + 0.035 / s[k]) which is accurate
 * on the range 0.08 to 1.0.
 *
 * NOTE: This function must be bit-exact on all platforms such that encoder
 * and decoder remain synchronized.
 *-------------------------------------------------------------------------*/
void tcx_arith_scale_envelope(
    Word16 L_spec_core,         /* i: number of lines to scale    Q0 */
    Word16 L_frame,             /* i: number of lines             Q0 */
    Word32 env[],               /* i: unscaled envelope           Q16 */
    Word16 target_bits,         /* i: number of available bits    Q0 */
    Word16 low_complexity,      /* i: low-complexity flag         Q0 */
    Word16 s_env[],             /* o: scaled envelope             Q15-e */
    Word16 *s_env_e             /* o: scaled envelope exponent    Q0 */
)
{
    Word32 ienv[N_MAX_ARI];
    Word16 scale, iscale, iscale_e, a_e, b, b_e;
    Word16 lob, hib, adjust;
    Word16 k, iter, max_iter, lob_bits, hib_bits;
    Word16 statesi, bits;
    Word32 mean, a, s, L_tmp;
    Word16 mean_e, tmp, tmp2;



    lob_bits = 0;
    move16();
    hib_bits = 0;
    move16();

    /* Boosting to account for expected spectrum truncation (kMax) */
    /* target_bits = (int)(target_bits * (1.2f - 0.00045f * target_bits + 0.00000025f * target_bits * target_bits)); */
    L_tmp = L_shr(Mpy_32_16_1(L_mult0(target_bits, target_bits), 17180), 6); /* Q15; 17180 -> 0.00000025f (Q36) */
    L_tmp = L_sub(L_tmp, L_shr(L_mult0(target_bits, 30199), 11)); /* Q15; 30199 -> 0.00045f (Q26) */
    L_tmp = L_add(L_tmp, 39322); /* Q15; 39322 -> 1.2f (Q15) */
    L_tmp = Mpy_32_16_1(L_tmp, target_bits); /* Q0 */
    assert(L_tmp < 32768);
    target_bits = extract_l(L_tmp);

    /* Calculate inverse envelope and find initial scale guess based on mean */
    mean = L_deposit_l(0);
    FOR (k = 0; k < L_frame; k++)
    {
        /* ienv[k] = 1.0f / env[k];
        mean += ienv[k]; */

        tmp = norm_l(env[k]);
        tmp2 = sub(15, tmp);
        tmp = Inv16(round_fx(L_shl(env[k], tmp)), &tmp2);
        ienv[k] = L_shl(L_deposit_h(tmp), sub(tmp2, 15)); /* Q16 */                 move32();
        mean = L_add(mean, ienv[k]);
    }
    tmp = norm_s(L_frame);
    tmp = shl(div_s(8192, shl(L_frame, tmp)), sub(tmp, 7));
    mean = L_shr(Mpy_32_16_1(mean, tmp), 6); /* Q16 */

    /* Rate dependent compensation to get closer to the target on average */
    /* mean = (float)pow(mean, (float)L_frame / (float)target_bits * 0.357f); */
    tmp = BASOP_Util_Divide1616_Scale(L_frame, target_bits, &tmp2);
    tmp = mult_r(tmp, FL2WORD16(0.357f));
    mean = BASOP_Util_fPow(mean, 15, L_deposit_h(tmp), tmp2, &mean_e);

    /* Find first-guess scaling coefficient "scale" such that if "mean" is the
     * mean of the envelope, then the mean bit-consumption is approximately
     *
     * log2(2*e*mean*scale + 0.15 + 0.035/(mean*scale)) * L_frame = target_bits
     */
    /* a = 2*2.71828183f*mean*mean; */
    tmp = round_fx(mean);
    a = L_mult(mult_r(tmp, FL2WORD16_SCALE(2.71828183f, 2)), tmp);
    a_e = add(shl(mean_e, 1), 3);

    /* b = (0.15f - (float)pow(2.0f, target_bits/(float)L_frame)) * mean; */
    tmp = BASOP_Util_Divide1616_Scale(target_bits, L_frame, &tmp2);
    tmp = round_fx(BASOP_util_Pow2(L_deposit_h(tmp), tmp2, &tmp2));
    b_e = BASOP_Util_Add_MantExp(FL2WORD16(0.15f), 0, negate(tmp), tmp2, &b);
    b = mult_r(b, round_fx(mean));
    b_e = add(b_e, mean_e);

    /* scale = (-b + (float)sqrt(b*b - 4.0f*a*0.035f)) / (2.0f * a); */
    tmp = round_fx(BASOP_Util_Add_Mant32Exp(L_mult(b, b), shl(b_e, 1), Mpy_32_16_1(a, FL2WORD16(-4.0f*0.035f)), a_e, &tmp2));
    tmp = Sqrt16(tmp, &tmp2);
    tmp2 = BASOP_Util_Add_MantExp(negate(b), b_e, tmp, tmp2, &scale);
    scale = BASOP_Util_Divide1616_Scale(scale, round_fx(a), &tmp);
    scale = shl(scale, sub(sub(add(tmp, tmp2), a_e), 1)); /* Q15 */

    /* iscale = 1.0f / scale; */
    iscale_e = 0;
    move16();
    iscale = Inv16(scale, &iscale_e);

    lob = 0;
    move16();
    hib = 0;
    move16();

    max_iter = 2;
    move16();
    if(low_complexity)
    {
        max_iter = 1;
        move16();
    }

    FOR (iter = 0; iter < max_iter; iter++)
    {
        statesi = 0x7FFF;
        move16();
        bits = 0;
        move16();

        FOR (k = 0; k < L_frame; k++)
        {
            s = Mpy_32_16_1(ienv[k], scale); /* Q16 */

            IF (L_sub(s, FL2WORD32_SCALE(0.08f, 15)) <= 0)
            {
                /* If s = 0.08, the expected bit-consumption is log2(1.0224). Below 0.08, the bit-consumption
                   estimate function becomes inaccurate, so use log2(1.0224) for all values below 0.08. */
                /* round(state * 1.0224 * 32768) */
                statesi = mult_r(statesi, FL2WORD16_SCALE(1.0224, 1));
                tmp = norm_s(statesi);
                statesi = shl(statesi, tmp);
                bits = add(bits, sub(1, tmp));
            }
            ELSE IF (L_sub(s, FL2WORD32_SCALE(255.0, 15)) <= 0)
            {
                /* a = 5.436564f * s + 0.15f + 0.035f * env[k] * iscale; */
                L_tmp = L_shl(Mpy_32_16_1(s, FL2WORD16_SCALE(5.436564f, 3)), 3);
                L_tmp = L_add(L_tmp, FL2WORD32_SCALE(0.15f, 15));
                L_tmp = L_add(L_tmp, L_shl(Mpy_32_16_1(env[k], mult_r(FL2WORD16(0.035f), iscale)), iscale_e));

                tmp = norm_l(L_tmp);
                statesi = mult_r(statesi, round_fx(L_shl(L_tmp, tmp)));
                bits = add(bits, sub(15, tmp));

                tmp = norm_s(statesi);
                statesi = shl(statesi, tmp);
                bits = sub(bits, tmp);
            }
            ELSE
            {
                /* for large envelope values, s > 255, bit consumption is approx log2(2*e*s)
                 * further, we use round(log2(x)) = floor(log2(x)+0.5) = floor(log2(x*sqrt(2))) */
                /* a = 5.436564f * s; */
                L_tmp = Mpy_32_16_1(s, FL2WORD16_SCALE(5.436564f * 1.4142f, 3)); /* Q13 */
                bits = add(bits, sub(17, norm_l(L_tmp)));
            }
        }

        IF (sub(bits, target_bits) <= 0)   /* Bits leftover => scale is too small */
        {
            lob      = scale;
            move16();
            lob_bits = bits;
            move16();

            IF (hib > 0)   /* Bisection search */
            {
                adjust = div_s(sub(hib_bits, target_bits), sub(hib_bits, lob_bits));
                scale = add(mult_r(sub(lob, hib), adjust), hib);
            }
            ELSE   /* Initial scale adaptation */
            {
                /* adjust = 1.05f * target_bits / (float)bits;
                   scale *= adjust; */
                adjust = mult_r(FL2WORD16_SCALE(1.05f, 1), target_bits);
                adjust = BASOP_Util_Divide1616_Scale(adjust, bits, &tmp);
                scale = shl(mult_r(scale, adjust), add(1, tmp));
            }
        }
        ELSE   /* Ran out of bits => scale is too large */
        {
            hib      = scale;
            move16();
            hib_bits = bits;
            move16();

            IF (lob > 0)   /* Bisection search */
            {
                adjust = div_s(sub(hib_bits, target_bits), sub(hib_bits, lob_bits));
                scale = add(mult_r(sub(lob, hib), adjust), hib);
            }
            ELSE { /* Initial scale adaptation */
                adjust = div_s(mult_r(FL2WORD16(0.95f), target_bits), bits);
                scale = mult_r(scale, adjust);
            }
        }
        iscale_e = 0;
        move16();
        iscale = Inv16(scale, &iscale_e);
    }
    L_frame = L_spec_core;
    move16();

    tmp = getScaleFactor32(env, L_frame);
    *s_env_e = sub(add(15, iscale_e), tmp);
    move16();
    BASOP_SATURATE_WARNING_OFF;
    a = L_shl(1265000, sub(15, *s_env_e));
    BASOP_SATURATE_WARNING_ON;

    FOR (k = 0; k < L_frame; k++)
    {
        L_tmp = Mpy_32_16_1(L_shl(env[k], tmp), iscale);
        L_tmp = L_min(L_tmp, a);
        s_env[k] = round_fx(L_tmp);
    }

}

/*------------------------------------------------------------------------
 * Function: tcx_arith_render_envelope
 *
 * Calculate the envelope of the spectrum based on the LPC shape. The
 * envelope is used in a perceptual domain, whereby the LPC shape has to
 * be multiplied by the perceptual model.
 * Operations that are performed on the spectrum, which change the magnitude
 * expectation of lines, such as low-frequency emphasis, are included in the
 * envelope shape.
 * NOTE: This function must be bit-exact on all platforms such that encoder
 * and decoder remain synchronized.
 *-------------------------------------------------------------------------*/
void tcx_arith_render_envelope(
    const Word16 A_ind[],       /* i: LPC coefficients of signal envelope        */
    Word16 L_frame,             /* i: number of spectral lines                   */
    Word16 L_spec,
    Word16 preemph_fac,         /* i: pre-emphasis factor                        */
    Word16 gamma_w,             /* i: A_ind -> weighted envelope factor          */
    Word16 gamma_uw,            /* i: A_ind -> non-weighted envelope factor      */
    Word32 env[]                /* o: shaped signal envelope                     */
)
{
    Word16 k;
    Word16 tmpA[M+2];
    Word16 signal_env[FDNS_NPTS], signal_env_e[FDNS_NPTS];
    Word16 gainlpc[FDNS_NPTS], gainlpc_e[FDNS_NPTS];



    /* Compute perceptual LPC envelope, transform it into freq.-domain gains */
    weight_a_fx( A_ind, tmpA, gamma_w, M );
    lpc2mdct( tmpA, M, NULL, NULL, gainlpc, gainlpc_e );

    /* Add pre-emphasis tilt to LPC envelope, transform LPC into MDCT gains */
    E_LPC_a_weight_inv(A_ind, signal_env, gamma_uw, M);
    E_LPC_a_add_tilt(signal_env, tmpA, preemph_fac, M);
    lpc2mdct(tmpA, M+1, signal_env, signal_env_e, NULL, NULL);

    /* Compute weighted signal envelope in perceptual domain */
    FOR (k = 0; k < FDNS_NPTS; k++)
    {
        signal_env[k] = mult_r(signal_env[k], gainlpc[k]);
        move16();
        signal_env_e[k] = add(signal_env_e[k], gainlpc_e[k]);
        move16();
    }

    /* Adaptive low frequency emphasis */
    set32_fx(env, 0x10000, L_frame);

    AdaptLowFreqDeemph(env, 15,
                       1,
                       gainlpc, gainlpc_e,
                       L_frame, NULL);

    /* Scale from FDNS_NPTS to L_frame and multiply LFE gains */
    test();
    IF (sub(L_frame, 384) <= 0 && s_and(L_frame, 0x3F) == 0)   /* FDNS_NPTS = 64 */
    {
        mdct_noiseShaping_interp(env, L_frame, signal_env, signal_env_e);
    }
    ELSE
    {
        mdct_shaping(env, L_frame, signal_env, signal_env_e);
    }
    FOR (k=L_frame; k<L_spec; ++k)
    {
        env[k] = env[k-1];
        move32();
    }

}

