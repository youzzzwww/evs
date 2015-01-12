/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "stl.h"

/* Returns: estimated SQ scale    Q15-e */
static Word16 tcx_arith_estimate_scale(
    const Word32 abs_spectrum[],    /* i: absolute MDCT coefficients  Q31-e */
    Word16 abs_spectrum_e,          /* i: MDCT exponent               Q0 */
    Word16 L_frame,                 /* i: number of spectral lines    Q0 */
    const Word16 envelope[],        /* i: scaled envelope             Q15-e */
    Word16 envelope_e,              /* i: scaled envelope exponent    Q0 */
    Word16 *scale_e                 /* o: scale exponent              Q0 */
)
{
    Word16 scale, tmp, k, s, s1;
    Word32 L_tmp, accu;



    /* compute normalised standard deviation and determine approximate scale */
    accu = L_deposit_l(0);
    s = 30;
    move16();

    FOR (k = 0; k < L_frame; k++)
    {
        /* tmp = abs_spectrum[k] * envelope[k];
           scale += tmp * tmp; */

        /* normalize, multiply, square */
        s1 = 30;
        move16();
        if (abs_spectrum[k] != 0)
        {
            s1 = norm_l(abs_spectrum[k]);
        }

        tmp = mult_r(round_fx(L_shl(abs_spectrum[k], s1)), envelope[k]);
        L_tmp = L_mult0(tmp, tmp);
        tmp = sub(shl(s1, 1), 1);

        /* adjust accu scaling */
        s1 = s;
        move16();
        if (L_and(accu, 0x40000000) != 0) s = sub(s, 1);
        s = s_min(s, tmp);

        s1 = sub(s1, s);
        if (s1 != 0) accu = L_shr(accu, s1);

        /* scale and accumulate */
        BASOP_SATURATE_WARNING_OFF;
        accu = L_add(accu, L_shr(L_tmp, sub(tmp, s)));
        BASOP_SATURATE_WARNING_ON;
    }
    s = sub(shl(add(abs_spectrum_e, envelope_e), 1), s);
    if (accu == 0) accu = L_deposit_l(1);

    /* scale = (float)sqrt((L_frame * 65536.0f*65536.0f*4.0f) / scale); */
    scale = BASOP_Util_Divide3216_Scale(accu, shl(L_frame, 2), &tmp);
    s = sub(add(s, tmp), 15);
    scale = ISqrt16(scale, &s);
    *scale_e = s;


    return scale;
}

#define kMaxNumHeapElems 10

typedef struct HeapElem
{
    Word32 mScore; /* Sort key                    */
    Word16 mIndex; /* Original index              */
} HeapElem;

typedef struct Heap
{
    HeapElem mElem[2*kMaxNumHeapElems+1];
    Word16 mSize;
} Heap;

static void MinHeapify_i(Heap *H, Word16 i)
{
    Word16 left, right, largest;
    HeapElem T;



    left = add(shl(i, 1), 1);
    right = add(left, 1);
    largest = i;
    move16();

    if (L_sub(H->mElem[left].mScore, H->mElem[largest].mScore) < 0)
    {
        largest = left;
        move16();
    }

    if (L_sub(H->mElem[right].mScore, H->mElem[largest].mScore) < 0)
    {
        largest = right;
        move16();
    }

    WHILE (sub(largest, i) != 0)
    {
        T.mIndex = H->mElem[i].mIndex;
        move16();
        T.mScore = L_add(H->mElem[i].mScore, 0);

        H->mElem[i].mIndex = H->mElem[largest].mIndex;
        move16();
        H->mElem[i].mScore = H->mElem[largest].mScore;
        move32();

        H->mElem[largest].mIndex = T.mIndex;
        move16();
        H->mElem[largest].mScore = T.mScore;
        move32();

        i = largest;
        move16();

        left = add(shl(i, 1), 1);
        right = add(left, 1);

        if (L_sub(H->mElem[left].mScore, H->mElem[largest].mScore) < 0)
        {
            largest = left;
            move16();
        }

        if (L_sub(H->mElem[right].mScore, H->mElem[largest].mScore) < 0)
        {
            largest = right;
            move16();
        }
    }

}

static Word16 tcx_arith_find_max_scale( /*  Q15-e */
    const Word32 abs_spectrum[],    /* i: absolute MDCT coefficients    Q31-e */
    Word16 abs_spectrum_e,          /* i: MDCT exponent                 Q0 */
    Word16 L_frame,                 /* i: number of spectral lines      Q0 */
    const Word16 envelope[],        /* i: scaled envelope               Q15-e */
    Word16 envelope_e,              /* i: scaled envelope exponent      Q0 */
    const Word16 exps[],            /* i: expfp(-(int)envelope[]/2)     Q15 */
    Word16 deadzone,                /* i: deadzone (0.5f = no deadzone) Q15 */
    Word16 *scale_e                 /* o: scale exponent                Q0 */
)
{
    Word16 i, k, q, scale, tmp, s;
    Word32 p, L_tmp;
    Heap heap = {{{0,0}},0}; /* silence a compiler warning */
    Word16 tmpi1, tmpi2;
    const Word32 limit = FL2WORD32_SCALE(-9.70406052784f, 6); /* = ln(1/16384): log of smallest allowed probability */



    /* Find the top most offending lines according to probability estimates */
    heap.mSize = kMaxNumHeapElems;
    move16();

    FOR (i = 0; i < heap.mSize; i++)
    {
        heap.mElem[i].mIndex = 0;
        move16();
        heap.mElem[i].mScore = L_deposit_l(0);
    }

    tmp = add(shl(heap.mSize, 1), 1);
    FOR (; i < tmp; i++)
    {
        heap.mElem[i].mScore = L_deposit_h(0x7FFF);
    }

    FOR (k = 0; k < L_frame; k++)
    {
        p = Mpy_32_16_1(abs_spectrum[k], envelope[k]);

        IF (L_sub(p, heap.mElem[0].mScore) > 0)
        {
            heap.mElem[0].mScore = p;
            move32();
            heap.mElem[0].mIndex = k;
            move16();
            MinHeapify_i(&heap, 0);
        }
    }

    /* Make sure the scale is limited so that the offending lines don't cause probability underflow. */
    /* Also limit scale to avoiding saturation of the gain quantizer */
    /* scale = 1.0f/(float)sqrt(L_frame*0.5f); */
    tmp = 15-1;
    move16();
    scale = ISqrt16(L_frame, &tmp);
    *scale_e = tmp;
    move16();

    FOR (i = 0; i < heap.mSize; i++)
    {
        k = heap.mElem[i].mIndex;
        move16();

        /* Get approximate maximum allowed magnitude */
        /* q = (int)ceil(((limit - log(1.0f - (exps[k]/65536.0) * (exps[k]/65536.0))) / (-(int)envelope[k]/2/65536.0) - 1) / 2.0f); */
        L_tmp = L_sub(0x7FFFFFFF, L_mult(exps[k], exps[k]));
        L_tmp = Mpy_32_16_1(BASOP_Util_Log2(L_tmp), 22713); /* Q25; 22713 -> 1/log2(e) */
        L_tmp = L_sub(limit, L_tmp);
        tmp = negate(BASOP_Util_Divide3216_Scale(L_tmp, envelope[k], &s));
        s = sub(add(s, 6), sub(envelope_e, 1));
        L_tmp = L_shl(L_deposit_h(tmp), sub(s, 15+1)); /* Q16 */
        L_tmp = L_sub(L_tmp, 0x8000);
        q = extract_h(L_add(L_tmp, 0xFFFF)); /* ceil */

        /* Refinement: get the exact q */
        powfp_odd2(exps[k], q, &tmpi1, &tmpi2);

        IF (sub(sub(tmpi1, tmpi2), 2) >= 0)   /* q may be too low */
        {
            powfp_odd2(exps[k], add(q, 1), &tmpi1, &tmpi2);

            WHILE (sub(sub(tmpi1, tmpi2), 2) >= 0)
            {
                q = add(q, 1);
                powfp_odd2(exps[k], add(q, 1), &tmpi1, &tmpi2);
            }
        }
        ELSE   /* q is too high */
        {
            q = sub(q, 1);
            powfp_odd2(exps[k], q, &tmpi1, &tmpi2);

            WHILE (sub(sub(tmpi1, tmpi2), 2) < 0)
            {
                q = sub(q, 1);
                powfp_odd2(exps[k], q, &tmpi1, &tmpi2);
            }
        }

        /* Find the largest scale so that the quantized magnitude is at most q */
        /* p = (q+0.99f-deadzone)/(abs_spectrum[k] + 0.000001f); */
        L_tmp = L_add(L_deposit_h(q), L_mult(sub(FL2WORD16(0.99f), deadzone), 1)); /* Q16 */
        tmp = BASOP_Util_Divide3232_Scale(L_tmp, L_add(abs_spectrum[k], 1), &s);
        s = sub(add(s, 15), abs_spectrum_e);

        k = norm_s(tmp);
        tmp = shl(tmp, k);
        s = sub(s, k);

        /* assert((int)(abs_spectrum[k] * p + deadzone) <= q); */

        /* scale = min(scale, p); */
        IF (compMantExp16Unorm(tmp, s, scale, *scale_e) < 0)
        {
            scale = tmp;
            move16();
            *scale_e = s;
            move16();
        }
    }


    return scale;
}

/* Returns: index of highest freq. nonzero line (-1 if all zeros) */
static Word16 tcx_arith_find_kMax(
    const Word32 abs_spectrum[],    /* i: absolute MDCT coefficients    Q31-e */
    Word16 abs_spectrum_e,          /* i: MDCT exponent                 Q0 */
    Word16 L_frame,                 /* i: number of spectral lines      Q0 */
    Word16 scale,                   /* i: scalar quantizer scale        Q15-e */
    Word16 scale_e,                 /* i: scale exponent                Q0 */
    Word16 deadzone,                /* i: deadzone (0.5f = no deadzone) Q15 */
    const Word16 deadzone_flags[]   /* i: line-wise deadzone control    */
)
{
    Word16 kMax;
    Word32 tmp[2];


    move32();
    move32();
    tmp[0] = L_shr(L_mac(0x7FFFFFFF, deadzone, (Word16)0x8000), abs_spectrum_e); /* 1.0f - deadzone scaled to MDCT exponent */
    tmp[1] = L_shr(0x7FFFFFFF, abs_spectrum_e); /* 1.0f scaled to MDCT exponent */

    FOR (kMax = sub(L_frame, 1); kMax >= 0; kMax--)
    {
        IF (L_sub(L_shl(Mpy_32_16_1(abs_spectrum[kMax], scale), scale_e), tmp[deadzone_flags[kMax]]) >= 0)
        {
            BREAK;
        }
    }


    return kMax;
}

#define LOG2_E  FL2WORD16_SCALE(1.44269504089f, 1)

/* Returns: best scale  Q15-e */
static Word16 tcx_arith_rateloop(
    const Word32 abs_spectrum[],   /* i: absolute MDCT coefficients     Q31-e */
    Word16 abs_spectrum_e,         /* i: MDCT exponent                  Q0 */
    Word16 L_frame,                /* i: number of spectral lines       Q0 */
    const Word16 envelope[],       /* i: scaled envelope                Q15-e */
    Word16 envelope_e,             /* i: scaled envelope exponent       Q0 */
    const Word16 exps[],           /* i: expfp(-(int)envelope[]/2)      Q15 */
    Word16 target_bits,            /* i: target bit budget              Q0 */
    Word16 deadzone,               /* i: deadzone (0.5f = no deadzone)  Q15 */
    const Word16 deadzone_flags[], /* i: line-wise deadzone control     Q0 */
    Word16 *target_bits_fac,       /* i/o: scale estimator compensation Q14 */
    Word16 *scale_e                /* o: scale exponent                 Q0 */
)
{
    Word16 k, kMax, q;
    Word16 s, adjust;
    Word16 fixed_bits[2][N_MAX_ARI];
    Word32 max_complexity;
    Word16 iter;          /* rate loop iteration counter               */
    Word16 scale;         /* SQ scale factor to try next               */
    Word16 scale_best;    /* best SQ scale factor                      */
    Word16 scale_max;     /* maximum allowable scale factor            */
    Word16 lob;           /* lower bound of SQ scale factor            */
    Word16 hib;           /* upper bound of SQ scale factor            */
    Word16 flag;          /* 1:bit surplus, -1:bit deficit, 0:unknown  */
    Word32 complexity;    /* cumulative rate loop complexity           */
    Word32 bits;          /* number of bits (approximate)           Q9 */
    Word32 L_tmp;
    Word16 tmp, tmp3;
    Word32 tmp2;



    scale = tcx_arith_estimate_scale(abs_spectrum, abs_spectrum_e, L_frame, envelope, envelope_e, &tmp);
    scale = mult_r(scale, *target_bits_fac);
    tmp = add(tmp, 1);

    scale_max = tcx_arith_find_max_scale(abs_spectrum, abs_spectrum_e, L_frame, envelope, envelope_e, exps, deadzone, scale_e);

    BASOP_SATURATE_WARNING_OFF;
    scale = shl(scale, sub(tmp, *scale_e));
    BASOP_SATURATE_WARNING_ON;
    scale = s_min(scale, scale_max);

    scale_best     = scale;
    move16();
    lob            = 0;
    move16();
    hib            = 0;
    move16();
    flag           = 0;
    move16();
    complexity     = L_deposit_l(0);
    bits           = L_deposit_l(0);
    iter           = 0;
    move16();

    max_complexity = L_mult0(96, L_frame);

    /* Precalculate fixed bit costs */
    FOR (k = 0; k < L_frame; k++)
    {
        /* fixed_bits[0][k] = -log2f(1 - exps[k] / 65536.0f); */
        L_tmp = L_mac(0x7FFFFFFF, exps[k], (Word16)0x8000); /* Q31 */
        L_tmp = L_negate(BASOP_Util_Log2(L_tmp)); /* Q25 */
        fixed_bits[0][k] = round_fx(L_tmp); /* Q9 */

        /* fixed_bits[1][k] = 1 - s*0.5f*LOG2_E - log2f(1 - (exps[k]/65536.0f) * (exps[k]/65536.0f)); */
        L_tmp = L_msu(0x7FFFFFFF, exps[k], exps[k]); /* Q31 */
        L_tmp = BASOP_Util_Log2(L_tmp); /* Q25 */
        L_tmp = L_sub(1<<25, L_tmp);
        L_tmp = L_sub(L_tmp, L_shl(L_mult0(mult_r(envelope[k], LOG2_E), 1<<10), envelope_e));
        fixed_bits[1][k] = round_fx(L_tmp); /* Q9 */
    }

    tmp2 = L_msu0(L_sub(max_complexity, 48), L_frame, 11);
    WHILE (L_sub(complexity, tmp2) < 0)
    {
        kMax = tcx_arith_find_kMax(
                   abs_spectrum,
                   abs_spectrum_e,
                   L_frame,
                   scale, *scale_e,
                   deadzone,
                   deadzone_flags
               );
        complexity = L_mac0(L_mac0(L_add(complexity, 16+2), sub(L_frame, kMax), 5), kMax, 2);

        bits = /*estimator_undershoot * kMax +*/ L_deposit_l(1<<9); /* Q9 */

        L_tmp = L_mult(deadzone, 1); /* Q16 */
        tmp = add(sub(abs_spectrum_e, 15), *scale_e);
        tmp3 = add(2+9, envelope_e);
        FOR (k = 0; k <= kMax; k++)
        {
            q = extract_h(L_add(L_shl(Mpy_32_16_1(abs_spectrum[k], scale), tmp), L_tmp));
            bits = L_mac0(bits, fixed_bits[s_min(1,q)][k], 1); /* Q9 */
            bits = L_mac0(bits, round_fx(L_shl(L_mult0(mult_r(envelope[k], LOG2_E), q), tmp3)), 1);
        }
        complexity = L_mac0(L_add(complexity, 32), 6, kMax);

        IF (iter == 0)   /* First rate loop iteration */
        {
            IF (sub(scale, scale_max) < 0)   /* Only update in non-degenerate case */
            {
                /* Update estimator temporal compensation factor */
                tmp = BASOP_Util_Divide3232_Scale(L_mult0(target_bits, 1<<9), bits, &s);
                BASOP_SATURATE_WARNING_OFF;
                tmp = shl(mult_r(*target_bits_fac, tmp), s);
                BASOP_SATURATE_WARNING_ON;
                tmp = s_min(tmp, FL2WORD16_SCALE(1.25f, 1));
                tmp = s_max(tmp, FL2WORD16_SCALE(0.75f, 1));
                *target_bits_fac = tmp;
                move16();
            }
        }

        IF (L_sub(bits, L_mult0(target_bits, 1<<9)) <= 0)   /* Bits leftover => scale is too small */
        {
            test();
            IF (flag <= 0 || sub(scale, scale_best) >= 0)
            {
                scale_best = scale;
                move16();
                flag       = 1;
                move16();
            }

            lob = scale;
            move16();

            IF (hib > 0)   /* Bisection search */
            {
                scale = add(shr(lob, 1), shr(hib, 1));
            }
            ELSE   /* Initial scale adaptation */
            {
                /* adjust = 1.25f * target_bits / (float)bits; */
                tmp = BASOP_Util_Divide3232_Scale(L_mult0(target_bits, 0x280), bits, &s);
                BASOP_SATURATE_WARNING_OFF; /* adjust limited to <= 2.0, scale to <= scale_max */
                adjust = shl(tmp, sub(s, 1)); /* Q14 */
                scale = shl(mult_r(scale, adjust), 1);
                BASOP_SATURATE_WARNING_ON;
                scale = s_min(scale, scale_max);
            }
        }
        ELSE   /* Ran out of bits => scale is too large */
        {
            hib = scale;
            move16();

            IF (lob > 0)   /* Bisection search */
            {
                scale = add(shr(lob, 1), shr(hib, 1));
            }
            ELSE { /* Initial scale adaptation */
                /* adjust = 0.8f * target_bits / (float)bits; */
                tmp = BASOP_Util_Divide3232_Scale(L_mult0(target_bits, 0x19A), bits, &s);
                adjust = shl(tmp, s); /* Q15 */
                adjust = s_max(adjust, FL2WORD16(0.5f));
                scale = mult_r(scale, adjust);
            }

            IF (flag <= 0)
            {
                scale_best = scale;
                move16();
                flag = 0;
                move16();
            }
        }
        iter = add(iter, 1);
    }


    return scale_best;
}

/* Returns: number of bits consumed */
static Word16 tcx_arith_encode(
    Word16 q_abs_spectrum[],        /* i/o: scalar quantized absolute spectrum      Q0 */
    const Word16 signs[],           /* i: signs                                        */
    Word16 kMax,                    /* i: number of nonzero spectral lines to code  Q0 */
    Word16 L_frame,                 /* i: nominal number of spectral lines          Q0 */
    const Word16 exps[],            /* i: expfp(-(int)envelope[]/2)                 Q15 */
    Word16 target_bits,             /* i: target bit budget                         Q0 */
    Word16 prm[]                    /* o: bit-stream                                Q0 */
)
{
    TastatEnc as, as_lastgood;
    Word16 bp, bp_lastgood;
    Word16 k;
    Word16 kEncoded;
    Word16 tmpi1, tmpi2;



    /* Final coding */
    ari_start_encoding_14bits(&as);
    ari_copy_states(&as, &as_lastgood);
    bp = 0;
    move16();
    bp_lastgood = 0;
    move16();
    kEncoded = kMax;
    move16();

    FOR (k = 0; k <= kMax; k++)
    {
        IF (q_abs_spectrum[k] == 0)
        {
            assert(exps[k] >= 2);
            bp = ari_encode_14bits_range(prm, bp, target_bits, &as, shr(exps[k], 1), 16384);
        }
        ELSE   /* q_abs_spectrum[k] != 0 */
        {
            powfp_odd2(exps[k], q_abs_spectrum[k], &tmpi1, &tmpi2);

            WHILE (sub(tmpi1, add(tmpi2, 2)) < 0)
            {
                q_abs_spectrum[k] = sub(q_abs_spectrum[k], 1);
                move16();
                powfp_odd2(exps[k], q_abs_spectrum[k], &tmpi1, &tmpi2);
            }

            bp = ari_encode_14bits_range(prm, bp, target_bits, &as, shr(tmpi2, 1), shr(tmpi1, 1));
            bp = ari_encode_14bits_sign(prm, bp, target_bits, &as, signs[k]);
        }
        /* Check bit budget status */
        IF (ari_encode_overflow(&as))   /* no bits left */
        {
            /* printf("\noverflow at %d\n\n", k); */

            IF (sub(q_abs_spectrum[k], 1) > 0)   /* Lower magnitude is still > 0 */
            {
                /* Restore state */
                ari_copy_states(&as_lastgood, &as);
                bp = bp_lastgood;
                move16();

                /* Quantize to lower magnitude */
                q_abs_spectrum[k] = sub(q_abs_spectrum[k], 1);
                move16();

                /* Retry encoding */
                powfp_odd2(exps[k], q_abs_spectrum[k], &tmpi1, &tmpi2);

                bp = ari_encode_14bits_range(prm, bp, target_bits, &as, shr(tmpi2, 1), shr(tmpi1, 1));
                bp = ari_encode_14bits_sign(prm, bp, target_bits, &as, signs[k]);

                IF (!ari_encode_overflow(&as))   /* Success */
                {
                    ari_copy_states(&as, &as_lastgood);
                    bp_lastgood = bp;
                    move16();
                    kEncoded = k;
                    move16();

                    set16_fx(q_abs_spectrum+k+1, 0, sub(kMax, k));
                    BREAK;
                }
            }
            ari_copy_states(&as_lastgood, &as);
            bp = bp_lastgood;
            move16();
            kEncoded = sub(k, 1);

            set16_fx(q_abs_spectrum+k, 0, sub(kMax, kEncoded));
            BREAK;
        }
        ELSE
        {
            ari_copy_states(&as, &as_lastgood);
            bp_lastgood = bp;
            move16();
        }
    }

    /* Send zeros until L_frame */
    tmpi1 = add(kEncoded, 1);
    kEncoded = sub(L_frame, 1);
    FOR (k = tmpi1; k < L_frame; k++)
    {
        assert(exps[k] >= 1);

        bp = ari_encode_14bits_range(prm, bp, target_bits, &as, shr(exps[k],1), 16384);
        /* Check bit budget status */
        IF (ari_encode_overflow(&as))   /* no bits left */
        {
            ari_copy_states(&as_lastgood, &as);
            bp = bp_lastgood;
            move16();
            kEncoded = sub(k, 1);
            BREAK;
        }
        ELSE
        {
            ari_copy_states(&as, &as_lastgood);
            bp_lastgood = bp;
            move16();
        }
    }

    IF (sub(kEncoded, sub(L_frame, 1)) == 0)   /* RESQ bits possibly available */
    {
        /* Limit target bits to actually needed bits */
        target_bits = add(add(bp, 16), extract_l(as.vobf));
    }
    return ari_done_cbr_encoding_14bits(prm, bp, target_bits, &as);
}

void tcx_arith_encode_envelope(
    Word32 spectrum[],                      /* i/o: MDCT coefficients           Q31-e */
    Word16 *spectrum_e,                     /* i/o: MDCT exponent               Q0 */
    Word16 signs[],                         /* o: signs (spectrum[.]<0)         Q0 */
    Word16 L_frame,                         /* i: frame or MDCT length          Q0 */
    Word16 L_spec,                          /* i: frame or MDCT length          Q0 */
    Encoder_State_fx *st,                   /* i/o: coder state                 */
    const Word16 A_ind[],                   /* i: quantised LPC coefficients    Q12 */
    Word16 target_bits,                     /* i: number of available bits      Q0 */
    Word16 prm[],                           /* o: bitstream parameters          Q0 */
    Word8 use_hm,                           /* i: use HM in current frame?      */
    Word16 prm_hm[],                        /* o: HM parameter area             Q0 */
    Word16 tcxltp_pitch,                    /* i: TCX LTP pitch in FD, -1 if n/a  Q0*/
    Word16 *arith_bits,                     /* o: bits used for ari. coding     Q0 */
    Word16 *signaling_bits,                 /* o: bits used for signaling       Q0 */
    Word16 *nf_seed                         /* o: noise filling seed            Q0 */
    ,Word16 low_complexity                  /* i: low-complexity flag           Q0 */
)
{
    Word32 env[N_MAX_ARI];             /* unscaled envelope (Q16) */
    Word16 *envelope; /* scaled envelope (Q15-e) */
    Word16 envelope_e;
    Word16 exponents[N_MAX_ARI]; /* Q15 */
    Word16 L_spec_core;
    Word16 *q_spectrum;
    TCX_config *tcx_cfg;
    Word16 scale, scale_e;
    Word16 k, kMax;
    Word16 deadzone;
    const Word16 *deadzone_flags;
    Word16 gamma_w, gamma_uw;
    Word16 hm_bits;
    Word32 L_tmp, L_tmp2;
    Word16 tmp;


    assert(L_spec <= N_MAX_ARI);


    tcx_cfg        = &st->tcx_cfg;
    deadzone       = tcx_cfg->sq_rounding;
    move16();
    deadzone_flags = st->memQuantZeros;
    *signaling_bits = 0;
    move16();

    assert(st->enableTcxLpc);
    gamma_w  = FL2WORD16(1.0f);
    move16();
    gamma_uw = st->inv_gamma;
    move16();

    tcx_arith_render_envelope(
        A_ind,
        L_frame,
        L_spec,
        tcx_cfg->preemph_fac,
        gamma_w,
        gamma_uw,
        env
    );

    FOR (k = 0; k < L_spec; k++)
    {
        signs[k] = extract_l(L_lshr(spectrum[k], 31));
        if (spectrum[k] < 0)
        {
            spectrum[k] = L_abs(spectrum[k]);
            move32();
        }
    }

    IF (use_hm != 0)
    {
        tcx_hm_analyse(
            spectrum,
            spectrum_e,
            L_spec,
            env,
            target_bits,
            tcx_cfg->coder_type,
            prm_hm,
            tcxltp_pitch,
            st->tcxltp_gain,
            &hm_bits
        );

        target_bits = sub(target_bits, hm_bits);
        *signaling_bits = add(*signaling_bits, hm_bits);
        move16();
    }
    ELSE
    {
        prm_hm[0] = 0;  /* just to be sure */                                       move16();
        hm_bits   = 0;
        move16();
    }

    L_spec_core = L_spec;
    move16();
    if (st->igf)
    {
        L_spec_core = s_min(L_spec_core, st->hIGFEnc.infoStartLine);
    }
    envelope = (Word16*)env;

    tcx_arith_scale_envelope(
        L_spec,
        L_spec_core,
        env,
        target_bits,
        low_complexity,
        envelope,
        &envelope_e
    );

    tmp = sub(envelope_e, 1+15);
    FOR (k = 0; k < L_spec; k++)
    {
        exponents[k] = round_fx(expfp(envelope[k], tmp));
    }

    scale = tcx_arith_rateloop(
                spectrum,
                *spectrum_e,
                L_spec,
                envelope,
                envelope_e,
                exponents,
                target_bits,
                deadzone,
                deadzone_flags,
                &st->LPDmem.tcx_target_bits_fac,
                &scale_e
            );

    /* Final quantization */
    kMax = tcx_arith_find_kMax(
               spectrum,
               *spectrum_e,
               L_spec,
               scale, scale_e,
               deadzone,
               deadzone_flags
           );

    q_spectrum = (Word16*)env; /* Reuse buffer */

    L_tmp = L_mult(deadzone, 1); /* Q16 */
    tmp = add(sub(*spectrum_e, 15), scale_e);
    FOR (k = 0; k <= kMax; k++)
    {
        /* quantise using dead-zone */
        q_spectrum[k] = extract_h(L_add(L_shl(Mpy_32_16_1(spectrum[k], scale), tmp), L_tmp));
    }

    /* Final encoding */
    *arith_bits = tcx_arith_encode(
                      q_spectrum,
                      signs,
                      kMax,
                      L_spec,
                      exponents,
                      target_bits,
                      prm
                  );

    /* Multiply back the signs */
    L_tmp2 = L_deposit_l(0);
    FOR (k = 0; k <= kMax; k++)
    {
        L_tmp2 = L_macNs(L_tmp2, q_spectrum[k], k);

        if (signs[k] != 0) L_tmp = L_mult(q_spectrum[k], -1 << (30 - SPEC_EXP_DEC));
        if (signs[k] == 0) L_tmp = L_mult(q_spectrum[k], 1 << (30 - SPEC_EXP_DEC));
        spectrum[k] = L_tmp;
        move32();
    }
    *spectrum_e = SPEC_EXP_DEC;
    move16();
    set32_fx(spectrum+k, 0, sub(s_max(L_frame, L_spec), k));

    /* noise filling seed */
    *nf_seed = extract_l(L_tmp2);

}

