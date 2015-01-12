/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "stl.h"
#include "options.h"
#include "prot_fx.h"
#include "rom_basop_util.h"
#include "basop_util.h"
#include "cnst_fx.h"

#define inv_int InvIntTable
extern const Word16 int_sqr[17];

static Word16 quantize(Word32 x, Word16 invGain, Word16 shift, Word32 offset)
{
    Word16 tmp16;
    Word32 tmp32;

    tmp32 = Mpy_32_16_1(L_abs(x), invGain);                   /* multiply */
    tmp32 = L_shl(tmp32, shift);                            /* convert to 15Q16 */
    tmp32 = L_add(tmp32, offset);                           /* add offset */
    tmp16 = extract_h(tmp32);                               /* truncate */
    if (x < 0) tmp16 = negate(tmp16);                       /* restore sign */

    return tmp16;
}

/* compute noise-measure flags for spectrum filling and quantization (0: tonal, 1: noise-like) */
void ComputeSpectrumNoiseMeasure(const Word32 *powerSpec,
                                 Word16 L_frame,
                                 Word16 startLine,
                                 Word8 resetMemory,
                                 Word16 *noiseFlags,
                                 Word16 lowpassLine)
{
    Word16 i, lastTone;
    Word32 s, c;
    Word16 tmp16;
    Word32 tmp1, tmp2 = 0; /* initialization only to avoid compiler warning, not counted */


    IF (resetMemory != 0)
    {
        FOR (i = 0; i < lowpassLine; i++)
        {
            noiseFlags[i] = 0;
            move16();
        }
    }

    FOR (i = lowpassLine; i < L_frame; i++)
    {
        noiseFlags[i] = 1;
        move16();
    }

    test();
    IF (powerSpec != NULL && sub(add(startLine, 6), L_frame) < 0)
    {
        lastTone = 0;
        move16();

        /* noise-measure flags for spectrum filling and quantization (0: tonal, 1: noise-like) */
        i = sub(startLine, 1);

        s = L_shr(powerSpec[i-7], 4);
        s = L_add(s, L_shr(powerSpec[i-6], 4));
        s = L_add(s, L_shr(powerSpec[i-5], 4));
        s = L_add(s, L_shr(powerSpec[i-4], 4));
        s = L_add(s, L_shr(powerSpec[i-3], 4));
        s = L_add(s, L_shr(powerSpec[i-2], 4));
        s = L_add(s, L_shr(powerSpec[i-1], 4));
        s = L_add(s, L_shr(powerSpec[i  ], 4));
        s = L_add(s, L_shr(powerSpec[i+1], 4));
        s = L_add(s, L_shr(powerSpec[i+2], 4));
        s = L_add(s, L_shr(powerSpec[i+3], 4));
        s = L_add(s, L_shr(powerSpec[i+4], 4));
        s = L_add(s, L_shr(powerSpec[i+5], 4));
        s = L_add(s, L_shr(powerSpec[i+6], 4));
        s = L_add(s, L_shr(powerSpec[i+7], 4));

        tmp16 = sub(lowpassLine, 7);
        FOR (i = add(i, 1); i < tmp16; i++)
        {
            c = L_shr(powerSpec[i-1], 4);
            c = L_add(c, L_shr(powerSpec[i], 4));
            c = L_add(c, L_shr(powerSpec[i+1], 4));

            s = L_sub(s, L_shr(powerSpec[i-8], 4));
            s = L_add(s, L_shr(powerSpec[i+7], 4));

            tmp1 = L_shr(c, 2);
            if (noiseFlags[i] == 0) c = L_shl(c, 1);
            if (noiseFlags[i] == 0) tmp2 = L_sub(c, tmp1); /* 1.75 * c */
            if (noiseFlags[i] != 0) tmp2 = L_add(c, tmp1); /* 1.25 * c */

            tmp2 = L_sub(s, tmp2);
            if (tmp2 >= 0)
            {
                noiseFlags[i] = 1;
                move16();
            }
            if (tmp2 < 0)
            {
                noiseFlags[i] = 0;
                move16();
            }
            if (tmp2 < 0)
            {
                lastTone = i;
                move16();
            }
        }

        /* lower L_frame*startRatio lines are tonal (0), upper 7 lines are processed separately */
        tmp16 = sub(lowpassLine, 1);
        FOR (; i < tmp16; i++)
        {
            c = L_shr(powerSpec[i-1], 4);
            c = L_add(c, L_shr(powerSpec[i], 4));
            c = L_add(c, L_shr(powerSpec[i+1], 4));

            tmp1 = L_shr(c, 2);
            if (noiseFlags[i] == 0) c = L_shl(c, 1);
            if (noiseFlags[i] == 0) tmp2 = L_sub(c, tmp1); /* 1.75 * c */
            if (noiseFlags[i] != 0) tmp2 = L_add(c, tmp1); /* 1.25 * c */

            /* running sum can't be updated any more, just use the latest one */
            tmp2 = L_sub(s, tmp2);
            if (tmp2 >= 0)
            {
                noiseFlags[i] = 1;
                move16();
            }
            if (tmp2 < 0)
            {
                noiseFlags[i] = 0;
                move16();
                /* lastTone = i; */
            }
        }
        noiseFlags[i] = 1;   /* uppermost line is defined as noise-like (1) */  move16();

        if (lastTone > 0)    /* spread uppermost tonal line one line upward */
        {
            noiseFlags[lastTone+1] = 0;
            move16();
        }
    }

}

void detectLowpassFac(const Word32 *powerSpec, Word16 powerSpec_e, Word16 L_frame, Word8 rectWin, Word16 *pLpFac, Word16 lowpassLine)
{
    Word16 i, tmp;
    Word32 threshold;


    threshold = FL2WORD32_SCALE(0.1f * 2*NORM_MDCT_FACTOR, 28); /* Q3 */
    BASOP_SATURATE_WARNING_OFF /* Allow saturation, because threshold is being compared to powerSpec[i] below. */
    threshold = L_shl(threshold, sub(28, powerSpec_e));

    if (rectWin != 0)
    {
        /* compensate for bad side-lobe attenuation with asymmetric windows */
        threshold = L_shl(threshold, 1);
    }
    BASOP_SATURATE_WARNING_ON

    tmp = shr(lowpassLine, 1);
    FOR (i = sub(lowpassLine, 1); i >= tmp; i--)
    {
        IF (L_sub(powerSpec[i], threshold) > 0)
        {
            BREAK;
        }
    }

    tmp = getInvFrameLen(L_frame);

    tmp = mult_r(FL2WORD16(0.7f), round_fx(L_shl(L_mult0(add(i, 1), tmp), 9)));
    *pLpFac = add(tmp, mult_r(FL2WORD16(0.3f), *pLpFac));
    move16();

}

/*-----------------------------------------------------------*
 * Compute noise-measure flags for spectrum filling          *
 * and quantization (0: tonal, 1: noise-like).               *
 * Detect low pass if present.                               *
 *-----------------------------------------------------------*/
void AnalyzePowerSpectrum(
    Encoder_State_fx *st,              /* i/o: encoder states                                  */
    Word16 L_frame,                   /* input: frame length                                  */
    Word16 L_frameTCX,                /* input: full band frame length                        */
    Word16 left_overlap,              /* input: left overlap length                           */
    Word16 right_overlap,             /* input: right overlap length                          */
    Word32 const mdctSpectrum[],      /* input: MDCT spectrum                                 */
    Word16 mdctSpectrum_e,
    Word16 const signal[],            /* input: windowed signal corresponding to mdctSpectrum */
    Word32 powerSpec[],               /* output: Power spectrum. Can point to signal          */
    Word16 *powerSpec_e
)
{
    Word16 i, iStart, iEnd, lowpassLine;
    Word16 tmp, s1, s2;
    Word32 tmp32;
    Word8 tmp8;

    lowpassLine = L_frameTCX;
    move16();

    *powerSpec_e = 16;
    move16();
    TCX_MDST(signal,
             powerSpec,
             powerSpec_e,
             left_overlap,
             sub(L_frameTCX, shr(add(left_overlap, right_overlap), 1)),
             right_overlap);

    iStart = 0;
    move16();
    iEnd = L_frameTCX;
    move16();

    IF (st->narrowBand != 0)
    {
        attenuateNbSpectrum(L_frameTCX, powerSpec);
    }

    /* get shift to common exponent */
    s1 = 0;
    move16();
    s2 = 0;
    move16();
    tmp = sub(mdctSpectrum_e, *powerSpec_e);
    if (tmp > 0)
    {
        s2 = negate(tmp);
    }
    if (tmp < 0)
    {
        s1 = tmp;
        move16();
    }

    /* get headroom */
    tmp = sub(getScaleFactor32(mdctSpectrum, L_frameTCX), s1);
    tmp = s_min(tmp, sub(getScaleFactor32(powerSpec, L_frameTCX), s2));
    s1 = add(s1, tmp);
    s2 = add(s2, tmp);

    /* power spectrum: MDCT^2 + MDST^2 */
    FOR (i = iStart; i < iEnd; i++)
    {
        tmp = round_fx(L_shl(mdctSpectrum[i], s1));
        tmp32 = L_mult0(tmp, tmp);

        tmp = round_fx(L_shl(powerSpec[i], s2));
        tmp32 = L_mac0(tmp32, tmp, tmp);

        powerSpec[i] = tmp32;
        move32();
    }

    *powerSpec_e = add(shl(sub(mdctSpectrum_e, s1), 1), 1);
    move16();

    tmp8 = 0;
    move16();
    test();
    if ( L_msu0(L_mult0(st->L_frame_fx, extract_l(st->last_sr_core)), st->L_frame_past, extract_l(st->sr_core)) != 0
            || sub(st->last_core_fx, TCX_20_CORE ) != 0 )
    {
        tmp8 = 1;
        move16();
    }

    ComputeSpectrumNoiseMeasure(powerSpec,
                                L_frameTCX,
                                divide3216(L_mult(st->nmStartLine, L_frame), st->L_frame_fx),
                                tmp8,
                                st->memQuantZeros,
                                lowpassLine);

    IF( L_sub(st->total_brate_fx, ACELP_24k40) <= 0 )
    {
        lowpassLine = shl(mult(st->tcx_cfg.bandwidth, L_frame), 1);

        test();
        detectLowpassFac(powerSpec, *powerSpec_e,
                         L_frame,
                         sub(st->last_core_fx, ACELP_CORE) == 0,
                         &st->measuredBwRatio,
                         lowpassLine);
    }
    ELSE
    {
        st->measuredBwRatio = 0x4000;
        move16();
    }
}

void AdaptLowFreqEmph(Word32 x[],
                      Word16 x_e,
                      Word16 xq[],
                      Word16 invGain,
                      Word16 invGain_e,
                      Word16 tcx_lpc_shaped_ari,
                      Word16 lpcGains[], Word16 lpcGains_e[],
                      const Word16 lg
                     )
{
    Word16 i, i_max, i_max_old, lg_4, tmp16, s;
    Word32 tmp32;


    IF (tcx_lpc_shaped_ari == 0)
    {
        lg_4 = shr(lg, 2);

        /* 1. find first magnitude maximum in lower quarter of spectrum */
        invGain_e = add(invGain_e, 1);
        i_max = -1;
        move16();

        FOR (i = 0; i < lg_4; i++)
        {
            tmp32 = Mpy_32_16_1(L_abs(x[i]), invGain);            /* multiply */
            tmp32 = L_shl(tmp32, sub(add(x_e, invGain_e), 15)); /* convert to 15Q16 */

            test();
            IF ((sub(abs_s(xq[i]), 2) >= 0) && (tmp32 >= 0x3A000))   /* 0x3A000 -> 3.625 (15Q16) */
            {

                /* Debug initialization to catch illegal cases of xq[i] */
                tmp16 = 0;

                if (xq[i] > 0)
                {
                    tmp16 = 2;
                    move16();
                }
                if (xq[i] < 0)
                {
                    tmp16 = -2;
                    move16();
                }

                assert(tmp16 != 0);

                xq[i] = add(xq[i], tmp16);
                move16();

                i_max = i;
                move16();
                BREAK;
            }
        }

        s = sub(add(x_e, invGain_e), 15);

        /* 2. compress value range of all xq up to i_max: add two steps */
        FOR (i = 0; i < i_max; i++)
        {
            xq[i] = quantize(x[i], invGain, s, 0x6000);
            move16();
        }

        /* 3. find first mag. maximum below i_max which is half as high */
        i_max_old = i_max;
        move16();

        IF (i_max_old >= 0)
        {
            invGain_e = add(invGain_e, 1);
            i_max = -1;  /* reset first maximum, update inverse gain */             move16();

            FOR (i = 0; i < lg_4; i++)
            {
                tmp32 = Mpy_32_16_1(L_abs(x[i]), invGain);            /* multiply */
                tmp32 = L_shl(tmp32, sub(add(x_e, invGain_e), 15)); /* convert to 15Q16 */

                test();
                IF ((sub(abs_s(xq[i]), 2) >= 0) && (tmp32 >= 0x3A000))   /* 0x3A000 -> 3.625 (15Q16) */
                {

                    /* Debug initialization to catch illegal cases of xq[i] */
                    tmp16 = 0;

                    if (xq[i] > 0)
                    {
                        tmp16 = 2;
                        move16();
                    }
                    if (xq[i] < 0)
                    {
                        tmp16 = -2;
                        move16();
                    }

                    assert(tmp16 != 0);

                    xq[i] = add(xq[i], tmp16);
                    move16();

                    i_max = i;
                    move16();
                    BREAK;
                }
            }
        }

        s = sub(add(x_e, invGain_e), 15);

        /* 4. re-compress and quantize all xq up to half-height i_max+1 */
        FOR (i = 0; i < i_max; i++)
        {
            xq[i] = quantize(x[i], invGain, s, 0x6000);
            move16();
        }

        /* 5. always compress 2 lines; lines could be at index 0 and 1! */
        IF (i_max_old >= 0)
        {
            invGain_e = sub(invGain_e, 1);  /* reset inverse gain */
            if (sub(i_max, i_max_old) < 0)
            {
                i_max = i_max_old;
                move16();
            }
        }

        i = add(i_max, 1);

        tmp32 = Mpy_32_16_1(L_abs(x[i]), invGain);            /* multiply */
        tmp32 = L_shl(tmp32, sub(add(x_e, invGain_e), 15)); /* convert to 15Q16 */
        IF (L_sub(tmp32, 0x3A000) >= 0)
        {

            /* Debug initialization to catch illegal cases of xq[i] */
            tmp16 = 0;

            if (xq[i] > 0)
            {
                tmp16 = 2;
                move16();
            }
            if (xq[i] < 0)
            {
                tmp16 = -2;
                move16();
            }

            assert(tmp16 != 0);

            xq[i] = add(xq[i], tmp16);
            move16();
        }
        ELSE
        {
            xq[i] = quantize(x[i], invGain, sub(add(x_e, invGain_e), 15), 0x6000);
            move16();
        }

        i = add(i, 1);

        tmp32 = Mpy_32_16_1(L_abs(x[i]), invGain);            /* multiply */
        tmp32 = L_shl(tmp32, sub(add(x_e, invGain_e), 15)); /* convert to 15Q16 */
        IF (L_sub(tmp32, 0x3A000) >= 0)
        {

            /* Debug initialization to catch illegal cases of xq[i] */
            tmp16 = 0;

            if (xq[i] > 0)
            {
                tmp16 = 2;
                move16();
            }
            if (xq[i] < 0)
            {
                tmp16 = -2;
                move16();
            }

            assert(tmp16 != 0);

            xq[i] = add(xq[i], tmp16);
            move16();
        }
        ELSE
        {
            xq[i] = quantize(x[i], invGain, sub(add(x_e, invGain_e), 15), 0x6000);
            move16();
        }


    }
    ELSE   /*if(!tcx_lpc_shaped_ari)*/
    {
        PsychAdaptLowFreqEmph(x, lpcGains, lpcGains_e);
    }/*if(!tcx_lpc_shaped_ari)*/

}

void PsychAdaptLowFreqEmph(Word32 x[],
                           const Word16 lpcGains[], const Word16 lpcGains_e[]
                          )
{
    Word16 i;
    Word16 max, max_e, fac, min, min_e, tmp, tmp_e;
    Word32 L_tmp;



    assert(lpcGains[0] >= 0x4000);

    max = lpcGains[0];
    move16();
    max_e = lpcGains_e[0];
    move16();
    min = lpcGains[0];
    move16();
    min_e = lpcGains_e[0];
    move16();

    /* find minimum (min) and maximum (max) of LPC gains in low frequencies */
    FOR (i = 1; i < 9; i++)
    {
        IF (compMantExp16Unorm(lpcGains[i], lpcGains_e[i], min, min_e) < 0)
        {
            min = lpcGains[i];
            move16();
            min_e = lpcGains_e[i];
            move16();
        }

        IF (compMantExp16Unorm(lpcGains[i], lpcGains_e[i], max, max_e) > 0)
        {
            max = lpcGains[i];
            move16();
            max_e = lpcGains_e[i];
            move16();
        }
    }

    min_e = add(min_e, 5); /* min *= 32.0f; */

    test();
    IF ((compMantExp16Unorm(max, max_e, min, min_e) < 0) && (max > 0))
    {
        /* fac = tmp = (float)pow(min / max, 0.0078125f); */
        tmp_e = max_e;
        move16();
        tmp = Inv16(max, &tmp_e);
        L_tmp = L_shl(L_mult(tmp, min), sub(add(tmp_e, min_e), 6)); /* Q25 */
        L_tmp = L_add(BASOP_Util_Log2(L_tmp), 6<<25); /* Q25 */
        L_tmp = L_shr(L_tmp, 7); /* 0.0078125f = 1.f/(1<<7) */
        L_tmp = BASOP_Util_InvLog2(L_sub(L_tmp, 1<<25)); /* Q30 */
        tmp = round_fx(L_tmp); /* Q14 */
        fac = shr(tmp, 1); /* Q13 */

        /* gradual boosting of lowest 32 bins; DC is boosted by (min/max)^1/4 */
        FOR (i = 31; i >= 0; i--)
        {
            x[i] = L_shl(Mpy_32_16_1(x[i], fac), 2);
            move32();
            fac  = shl(mult_r(fac, tmp), 1);
        }
    }

}

Word16 SQ_gain(     /* output: SQ gain                   */
    Word32 x[],       /* input:  vector to quantize        */
    Word16 x_e,       /* input:  exponent                  */
    Word16 nbitsSQ,   /* input:  number of bits targeted   */
    Word16 lg,        /* input:  vector size (2048 max)    */
    Word16 *gain_e)   /* output: SQ gain exponent          */
{
    Word16 i, iter, lg_4, s, tmp16;
    Word32 ener, tmp32;
    Word32 target, fac, offset;
    Word32 en[N_MAX/4];


    lg_4 = shr(lg, 2);

    /* energy of quadruples with 9dB offset */
    FOR (i=0; i<lg_4; i++)
    {
        /* normalization */
        s = 15;
        move16();

        tmp16 = norm_l(x[0]);
        if (x[0] != 0) s = s_min(s, tmp16);

        tmp16 = norm_l(x[1]);
        if (x[1] != 0) s = s_min(s, tmp16);

        tmp16 = norm_l(x[2]);
        if (x[2] != 0) s = s_min(s, tmp16);

        tmp16 = norm_l(x[3]);
        if (x[3] != 0) s = s_min(s, tmp16);

        s = sub(s, 2);  /* 2 bits headroom */

        /* calc quadruple energy */
        ener = L_deposit_l(1);

        tmp16 = extract_h(L_shl(x[0], s));
        ener = L_mac(ener, tmp16, tmp16);

        tmp16 = extract_h(L_shl(x[1], s));
        ener = L_mac(ener, tmp16, tmp16);

        tmp16 = extract_h(L_shl(x[2], s));
        ener = L_mac(ener, tmp16, tmp16);

        tmp16 = extract_h(L_shl(x[3], s));
        ener = L_mac(ener, tmp16, tmp16);

        s = shl(sub(x_e, s), 1);

        /* log */
        tmp32 = L_add(BASOP_Util_Log2(ener), L_shl(L_deposit_l(s), 25)); /* log2, 6Q25 */
        en[i] = L_shr(tmp32, 9); /* 15Q16 */                                    move32();
        x += 4;
    }

    /* SQ scale: 4 bits / 6 dB per quadruple */
    target = L_mult(0x3FC8, sub(nbitsSQ, shr(lg, 4))); /* 0x3FC8 -> 0.15*log2(10) */
    fac = L_add(0x2A854B, 0); /* -> 12.8f*log2(10); */
    offset = L_add(fac, 0);

    /* find offset (0 to 128 dB with step of 0.125dB) */
    FOR (iter=0; iter<10; iter++)
    {
        fac = L_shr(fac, 1);
        offset = L_sub(offset, fac);
        ener = L_deposit_l(0);

        FOR (i=0; i<lg_4; i++)
        {
            tmp32 = L_sub(en[i], offset);

            /* avoid SV with 1 bin of amp < 0.5f */
            if (L_sub(tmp32, 0xFF20) > 0)    /* 0xFF20 -> 0.3*log2(10); */
            {
                ener = L_add(ener, tmp32);
            }

            /* if ener is above target -> break and increase offset */
            IF (L_sub(ener, target) > 0)
            {
                offset = L_add(offset, fac);
                BREAK;
            }
        }
    }

    offset = L_add(L_shr(offset, 1), 0x17EB0); /* 0x17EB0 -> 0.45*log2(10) */

    *gain_e = add(extract_h(offset), 1);
    move16();
    offset = L_sub(L_and(offset, 0xFFFF), 0x10000);
    tmp16 = extract_h(BASOP_Util_InvLog2(L_shl(offset, 9)));

    /* return gain */

    return tmp16;
}

void tcx_scalar_quantization(
    Word32 *x,                   /* i: input coefficients            */
    Word16 x_e,                  /* i: exponent                      */
    Word16 *xq,                  /* o: quantized coefficients        */
    Word16 L_frame,              /* i: frame length                  */
    Word16 gain,                 /* i: quantization gain             */
    Word16 gain_e,               /* i: quantization gain exponent    */
    Word16 offset,               /* i: rounding offset (deadzone)    */
    Word16 const *memQuantZeros, /* i: coefficients to be set to 0   */
    const Word16 alfe_flag
)
{
    Word16 i, tmp16, s;
    Word32 tmp32, offs32;


    /* common exponent for x and gain for comparison */
    tmp16 = sub(gain_e, x_e);
    tmp32 = L_shl(L_deposit_h(gain), s_max(-31, s_min(tmp16, 0)));
    tmp16 = negate(s_max(tmp16, 0));

    i = sub(L_frame, 1);
    WHILE ((memQuantZeros[i] != 0) && (L_sub(L_abs(L_shl(x[i], tmp16)), tmp32) < 0))
    {
        test();
        xq[i] = 0;
        move16();
        i = sub(i, 1);
    }

    /* invert gain */
    gain = Inv16(gain, &gain_e);

    s = sub(add(x_e, gain_e), 15);
    /* substract 0x8000 to affect the mac_r in the following loop
       so it acts like extract_h. the 0x4000 will be multiplied by 2
       by the mac_r to get to 0x8000 and disable the round. */
    offset = sub(offset, 0x4000);

    FOR (; i >= 0; i--)
    {
        offs32 = Mpy_32_16_1(L_abs(x[i]), gain);         /* multiply */
        offs32 = L_shl(offs32, s);                            /* convert to 15Q16 */
        tmp16 = mac_r(offs32, offset, 1);                     /* add offset and truncate */
        if (x[i] < 0) tmp16 = negate(tmp16);                  /* restore sign */

        xq[i] = tmp16;
        move16();
    }

    IF (alfe_flag == 0)
    {
        AdaptLowFreqEmph(x, x_e, xq, gain, gain_e,
                         0, NULL, NULL,
                         L_frame
                        );
    }

}

Word16 tcx_scalar_quantization_rateloop(
    Word32 *x,                  /* i  : input coefficients            */
    Word16 x_e,                 /* i  : exponent                      */
    Word16 *xq,                 /* o  : quantized coefficients        */
    Word16 L_frame,             /* i  : frame length                  */
    Word16 *gain,               /* i/o: quantization gain             */
    Word16 *gain_e,             /* i/o: gain exponent                 */
    Word16 offset,              /* i  : rounding offset (deadzone)    */
    Word16 const*memQuantZeros, /* i  : coefficients to be set to 0   */
    Word16 *lastnz_out,         /* i/o: last nonzero coeff index      */
    Word16 target,              /* i  : target number of bits         */
    Word16 *nEncoded,           /* o  : number of encoded coeff       */
    Word16 *stop,               /* i/o: stop param                    */
    Word16 sqBits_in_noStop,    /* i  : number of sqBits as determined in prev. quant. stage, w/o using stop mechanism (ie might exceed target bits) */
    Word16 sqBits_in,           /* i  : number of sqBits as determined in prev. quant. stage, using stop mechanism (ie always <= target bits) */
    Word16 tcxRateLoopOpt,      /* i  : turns on/off rateloop optimization */
    const Word8 tcxonly,
    CONTEXT_HM_CONFIG *hm_cfg   /* i  : configuration of the context-based harmonic model */
)
{
    const Word16 iter_max = 4;
    Word16 sqBits;
    Word16 stopFlag;
    Word8 ubfound,lbfound;
    Word16 ub, ub_e, lb, lb_e;
    Word16 shift, shiftInv;
    Word16 iter;
    Word16 sqGain, sqGain_e;
    Word16 w_lb, w_ub;
    const Word16 kDampen = 10;
    Word16 old_stopFlag;
    Word16 old_nEncoded;
    Word16 old_sqBits;
    Word16 mod_adjust0, mod_adjust1;
    Word16 inv_target, inv_target_e;
    const Word16 kMargin = 0x7AE1; /* 0.96 */
    const Word16 kMarginInv = 0x42AB; /* 1/0.96 (1Q14) */
    Word16 tmp, fac1, fac2;
    Word32 tmp32;
    Word16 lastnz;



    /* Init */
    sqGain = *gain;
    move16();
    sqGain_e = *gain_e;
    move16();
    stopFlag = *stop;
    move16();
    ubfound = 0;
    move16();
    lbfound = 0;
    move16();
    shift = 0x41DE; /* 10^(1/80), 1Q14 */                                       move16();
    shiftInv = 0x78D7; /* 10^(-1/40) */                                         move16();
    lb = lb_e = 0;
    move16();
    ub = ub_e = 0;
    move16();
    w_lb = 0;
    move16();
    w_ub = 0;
    move16();
    lastnz = *lastnz_out;
    move16();
    old_stopFlag   = stopFlag;
    move16();
    old_nEncoded   = *nEncoded;
    move16();
    old_sqBits     = sqBits_in_noStop;
    move16();

    sqBits      = sqBits_in;
    move16();

    mod_adjust0 = extract_l(L_shr(L_max(0x10000, L_sub(0x24CCD, L_mult(0x0052, target))), 3));  /* 2Q13 */
    mod_adjust1 = div_s(0x2000, mod_adjust0);   /* 0Q15 */

    inv_target_e = 15;
    move16();
    inv_target = Inv16(target, &inv_target_e);

    fac1 = shl(mult(mult(kMarginInv, mod_adjust0), inv_target), 1); /* 2Q13 */
    fac2 = mult(mult(kMargin, mod_adjust1), inv_target);

    /* Loop */
    FOR ( iter=0 ; iter<iter_max ; iter++ )
    {
        IF (sub(tcxRateLoopOpt, 2) == 0)
        {
            /* Ajust sqGain */
            IF ( stopFlag != 0 )
            {
                lbfound = 1;
                move16();
                lb = sqGain;
                move16();
                lb_e = sqGain_e;
                move16();
                w_lb = add(sub(stopFlag, target), kDampen);

                IF (ubfound != 0)
                {
                    /* common exponent for addition */
                    sqGain_e = s_max(lb_e, ub_e);

                    /* multiply and add */
                    tmp32 = L_shr(L_mult(lb, w_ub), sub(sqGain_e, lb_e));
                    tmp32 = L_add(tmp32, L_shr(L_mult(ub, w_lb), sub(sqGain_e, ub_e)));

                    /* convert to normalized 16 bit */
                    tmp = norm_l(tmp32);
                    sqGain = round_fx(L_shl(tmp32, tmp));
                    sqGain_e = sub(sqGain_e, tmp);

                    /* divide */
                    sqGain = BASOP_Util_Divide1616_Scale(sqGain, add(w_ub, w_lb), &tmp);
                    sqGain_e = add(sqGain_e, tmp);
                }
                ELSE
                {
                    tmp = round_fx(L_shl(L_mult(stopFlag, fac1), add(inv_target_e, 15)));
                    sqGain = mult(sqGain, sub(tmp, sub(mod_adjust0, 0x2000)));
                    sqGain = normalize16(sqGain, &sqGain_e);
                    sqGain_e = add(sqGain_e, 2);
                }
            }
            ELSE
            {
                ubfound = 1;
                move16();
                ub = sqGain;
                move16();
                ub_e = sqGain_e;
                move16();
                w_ub = add(sub(target, sqBits), kDampen);

                IF (lbfound != 0)
                {
                    /* common exponent for addition */
                    sqGain_e = s_max(lb_e, ub_e);

                    /* multiply and add */
                    tmp32 = L_shr(L_mult(lb, w_ub), sub(sqGain_e, lb_e));
                    tmp32 = L_add(tmp32, L_shr(L_mult(ub, w_lb), sub(sqGain_e, ub_e)));

                    /* convert to normalized 16 bit */
                    tmp = norm_l(tmp32);
                    sqGain = round_fx(L_shl(tmp32, tmp));
                    sqGain_e = sub(sqGain_e, tmp);

                    /* divide */
                    sqGain = BASOP_Util_Divide1616_Scale(sqGain, add(w_ub, w_lb), &tmp);
                    sqGain_e = add(sqGain_e, tmp);
                }
                ELSE {
                    tmp = round_fx(L_shl(L_mult(sqBits, fac2), add(inv_target_e, 15)));
                    sqGain = mult(sqGain, sub(tmp, add(mod_adjust1, (Word16)0x8000)));
                    sqGain = normalize16(sqGain, &sqGain_e);
                }
            }
        }
        ELSE   /* tcxRateLoopOpt != 2 */
        {

            /* Ajust sqGain */
            IF ( stopFlag != 0)
            {
                lbfound = 1;
                move16();
                lb = sqGain;
                move16();
                lb_e = sqGain_e;
                move16();

                IF (ubfound != 0)
                {
                    sqGain = mult(lb, ub);
                    sqGain_e = add(lb_e, ub_e);
                    sqGain = Sqrt16(sqGain, &sqGain_e);
                }
                ELSE
                {
                    shift = shl(mult(shift, shift), 1);
                    shiftInv = mult(shiftInv, shiftInv);

                    sqGain = mult(sqGain, shift);
                    sqGain = normalize16(sqGain, &sqGain_e);
                    sqGain_e = add(sqGain_e, 1);
                }
            }
            ELSE {
                ubfound = 1;
                move16();
                ub = sqGain;
                move16();
                ub_e = sqGain_e;
                move16();

                IF (lbfound != 0)
                {
                    sqGain = mult(lb, ub);
                    sqGain_e = add(lb_e, ub_e);
                    sqGain = Sqrt16(sqGain, &sqGain_e);
                }
                ELSE {
                    sqGain = mult(sqGain, shiftInv);
                    sqGain = normalize16(sqGain, &sqGain_e);

                    shift = shl(mult(shift, shift), 1);
                    shiftInv = mult(shiftInv, shiftInv);
                }
            }

        }

        /* Quantize spectrum */
        tcx_scalar_quantization( x, x_e, xq, L_frame, sqGain, sqGain_e, offset, memQuantZeros, tcxonly );

        /* Estimate bitrate */
        stopFlag = 1;
        move16();
        if (tcxRateLoopOpt > 0)
        {
            stopFlag = 0;
            move16();
        }

        sqBits = ACcontextMapping_encode2_estimate_no_mem_s17_LC( xq, L_frame,
                 &lastnz,
                 nEncoded, target, &stopFlag,
                 hm_cfg
                                                                );

        IF ( tcxRateLoopOpt > 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            IF ( ((sub(*nEncoded, old_nEncoded) >= 0) && (sub(stopFlag, old_stopFlag) >= 0)) ||
                 ((sub(*nEncoded, old_nEncoded) > 0) && ((stopFlag == 0) && (old_stopFlag > 0))) ||
                 ((stopFlag == 0) && (old_stopFlag == 0)) )
            {
                *gain = sqGain;
                move16();
                *gain_e = sqGain_e;
                move16();
                old_nEncoded = *nEncoded;
                move16();
                old_stopFlag = stopFlag;
                move16();
                old_sqBits = sqBits;
                move16();
                *lastnz_out = lastnz;
                move16();
            }
        }
    } /* for ( iter=0 ; iter<iter_max ; iter++ ) */

    IF ( tcxRateLoopOpt > 0 )
    {
        /* Quantize spectrum */
        tcx_scalar_quantization( x, x_e, xq, L_frame, *gain, *gain_e, offset, memQuantZeros, tcxonly );

        /* Output */
        *nEncoded = old_nEncoded;
        move16();
        sqBits = old_sqBits;
        move16();
        *stop  = old_stopFlag;
        move16();
    }
    ELSE
    {
        /* Output */
        *gain = sqGain;
        move16();
        *gain_e = sqGain_e;
        move16();
        *stop = stopFlag;
        move16();
        *lastnz_out = lastnz;
        move16();
    }


    return sqBits;
}

void QuantizeGain(Word16 n, Word16 *pGain, Word16 *pGain_e, Word16 *pQuantizedGain)
{
    Word16 ener, ener_e, enerInv, enerInv_e, gain, gain_e;
    Word16 quantizedGain;
    Word32 tmp32;


    ener = mult_r(shl(n, 5), FL2WORD16(128.f/NORM_MDCT_FACTOR));
    ener_e = 15-5-7;
    move16();
    BASOP_Util_Sqrt_InvSqrt_MantExp(ener, ener_e, &ener, &ener_e, &enerInv, &enerInv_e);

    gain = mult(*pGain, ener);
    gain_e = *pGain_e + ener_e;

    assert(gain > 0);

    /* quantize gain with step of 0.714 dB */
    quantizedGain = add(round_fx(BASOP_Util_Log2(L_deposit_h(gain))), shl(gain_e, 9)); /* 6Q9 */
    quantizedGain = mult(quantizedGain, 0x436E); /* 10Q5;  0x436E -> 28/log2(10) (4Q11) */
    quantizedGain = shr(add(quantizedGain, 0x10), 5); /* round */

    if (quantizedGain < 0)
    {
        quantizedGain = 0;
        move16();
    }

    if (quantizedGain > 127)
    {
        quantizedGain = 127;
        move16();
    }

    *pQuantizedGain = quantizedGain;
    move16();

    tmp32 = L_shl(L_mult0(quantizedGain, 0x797D), 7); /* 6Q25; 0x797D -> log2(10)/28 (Q18) */
    gain_e = add(extract_l(L_shr(tmp32, 25)), 1); /* get exponent */
    gain = round_fx(BASOP_Util_InvLog2(L_or(tmp32, 0xFE000000)));

    *pGain = mult(gain, enerInv);
    move16();
    *pGain_e = add(gain_e, enerInv_e);
    move16();

}

void tcx_noise_factor(
    Word32 *x_orig,         /* i: unquantized mdct coefficients             */
    Word16 x_orig_e,        /* i: exponent                                  */
    Word32 *sqQ,            /* i: quantized mdct coefficients               */
    Word16 iFirstLine,      /* i: first coefficient to be considered        */
    Word16 lowpassLine,     /* i: last nonzero coefficients after low-pass  */
    Word16 nTransWidth,     /* i: minimum size of hole to be checked        */
    Word16 L_frame,         /* i: frame length                              */
    Word16 gain_tcx,        /* i: tcx gain                                  */
    Word16 gain_tcx_e,      /* i: gain exponent                             */
    Word16 tiltCompFactor,  /* i: LPC tilt compensation factor              */
    Word16 *fac_ns,         /* o: noise factor                              */
    Word16 *quantized_fac_ns/* o: quantized noise factor                    */
)
{
    Word16 i, k, maxK, segmentOffset;
    Word32 sqErrorNrg, n;
    Word16 inv_gain2, inv_gain2_e, tilt_factor, nTransWidth_1;
    Word32 accu1, accu2, tmp32;
    Word16 tmp1, tmp2, s;
    Word16 c1, c2;
    Word16 att;  /* noise level attenuation factor for transient windows */
    Word32 xMax;


    assert(nTransWidth <= 16);

    c1 = sub(shl(nTransWidth, 1), 4);
    c2 = mult(FL2WORD16(0.28125f), inv_int[nTransWidth]);
    nTransWidth_1 = sub(nTransWidth, 1);

    /*Adjust noise filling level*/
    sqErrorNrg = L_deposit_l(0);
    n = L_deposit_l(0);

    /* get inverse frame length */
    tmp1 = getInvFrameLen(L_frame);

    /* tilt_factor = 1.0f /(float)pow(max(0.375f, tiltCompFactor), 1.0f/(float)L_frame); */
    tmp32 = BASOP_Util_Log2(L_deposit_h(s_max(0x3000, tiltCompFactor))); /* 6Q25 */
    tmp32 = L_shr(Mpy_32_16_1(tmp32, negate(tmp1)), 6);
    tilt_factor = round_fx(BASOP_Util_InvLog2(L_sub(tmp32, 0x2000000))); /* 1Q14 */

    /* inv_gain2 = 1.0f / ((float)(nTransWidth * nTransWidth) * gain_tcx); */
    tmp32 = L_mult(imult1616(nTransWidth, nTransWidth), gain_tcx); /* 15Q16 */
    tmp1 = norm_l(tmp32);
    inv_gain2 = round_fx(L_shl(tmp32, tmp1));
    inv_gain2_e = add(sub(15, tmp1), gain_tcx_e);
    inv_gain2 = Inv16(inv_gain2, &inv_gain2_e);
    inv_gain2 = shr(inv_gain2, 2); /* 2 bits headroom */
    inv_gain2_e = add(inv_gain2_e, 2);

    /* find last nonzero line below iFirstLine, use it as start offset */
    tmp1 = shr(iFirstLine, 1);
    FOR (i = iFirstLine; i > tmp1; i--)
    {
        IF (sqQ[i] != 0)
        {
            BREAK;
        }
    }
    /* inv_gain2 *= (float)pow(tilt_factor, (float)i); */
    FOR (k = 0; k < i; k++)
    {
        inv_gain2 = shl(mult(inv_gain2, tilt_factor), 1);
    }

    /* initialize left (k) and right (maxK) non-zero neighbor pointers */
    k = 0;
    move16();
    FOR (maxK = 1; maxK < nTransWidth; maxK++)
    {
        IF (sqQ[i+maxK] != 0)
        {
            BREAK;
        }
    }
    i = add(i, 1);
    segmentOffset = i;
    move16();

    IF (sub(nTransWidth, 3) <= 0)
    {
        accu1 = L_deposit_l(0);
        accu2 = L_deposit_l(0);
        xMax = L_deposit_l(0);

        FOR (k = s_and(i, (Word16)0xFFFE); k < lowpassLine; k++)
        {
            xMax = L_max(xMax, L_abs(x_orig[k]));
        }
        s = sub(norm_l(xMax), 4);

        FOR (k = s_and(i, (Word16)0xFFFE); k < lowpassLine; k += 2)
        {
            /* even-index bins, left sub-win */
            tmp1 = round_fx(L_shl(x_orig[k], s));
            accu1 = L_mac0(accu1, tmp1, tmp1);

            /* odd-index bins, right sub-win */
            tmp1 = round_fx(L_shl(x_orig[k+1], s));
            accu2 = L_mac0(accu2, tmp1, tmp1);
        }
        k = 0;
        move16();

        if (accu1 == 0) accu1 = L_deposit_l(1);
        if (accu2 == 0) accu2 = L_deposit_l(1);

        att = BASOP_Util_Divide3232_Scale( L_shl(L_min(accu1, accu2), 1), L_add(accu1, accu2), &s );
        att = Sqrt16(att, &s);
        BASOP_SATURATE_WARNING_OFF; /* att is always <= 1.0 */
        att = shl(att, s);
        BASOP_SATURATE_WARNING_ON;
    }
    ELSE
    {
        att = 0x7FFF;
        move16();
    }

    accu1 = L_deposit_l(0);

    tmp1 = sub(lowpassLine, nTransWidth);
    FOR (; i <= tmp1; i++)
    {
        inv_gain2 = shl(mult(inv_gain2, tilt_factor), 1);

        IF (sub(maxK, 1) == 0)    /* current line is not zero, so reset pointers */
        {
            k = sub(i, segmentOffset);

            IF (k > 0)     /* add segment sum to sum of segment magnitudes */
            {
                IF (sub(nTransWidth, 3) <= 0)
                {
                    tmp2 = sub(k, c1);
                    if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
                    if (tmp2 > 0) n = L_mac(n, nTransWidth_1, (Word16)0x8000);
                    if (tmp2 <= 0) n = L_mac(n, int_sqr[k], c2);
                }
                ELSE
                {
                    tmp2 = sub(k, 12);
                    if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
                    if (tmp2 > 0) n = L_sub(n, 0x70000);
                    if (tmp2 <= 0) n = L_mac(n, int_sqr[k], FL2WORD16(0.03515625f));
                }
                sqErrorNrg = L_add(sqErrorNrg, accu1);
                accu1 = L_deposit_l(0);      /* segment ended here, so reset segment sum */
                k = 0;
                move16();
            }

            FOR (; maxK < nTransWidth; maxK++)
            {
                IF (sqQ[i+maxK] != 0)
                {
                    BREAK;
                }
            }
            segmentOffset = add(i, 1); /* new segment might start at next line */
        }
        ELSE   /* current line is zero, so update pointers & segment sum */
        {
            if (sub(k, nTransWidth) < 0)
            {
                k = add(k, 1);
            }

            tmp2 = sub(maxK, nTransWidth);
            if (tmp2 < 0)
            {
                maxK = sub(maxK, 1);
            }

            test();
            if ((tmp2 >= 0) && (sqQ[i+sub(nTransWidth, 1)] != 0))
            {
                maxK = sub(nTransWidth, 1);
            }

            /* update segment sum: magnitudes scaled by smoothing function */
            /*accu1 += (float)fabs(x_orig[i]) * inv_gain2 * (float)(k * maxK);*/
            tmp2 = mult(inv_gain2, shl(imult1616(k, maxK), 8));
            accu1 = L_add(accu1, L_abs(Mpy_32_16_1(x_orig[i], tmp2)));
        }
    }

    FOR (; i < lowpassLine; i++)
    {
        inv_gain2 = shl(mult(inv_gain2, tilt_factor), 1);

        IF (sub(maxK, 1) == 0)    /* current line is not zero, so reset pointers */
        {
            k = sub(i, segmentOffset);

            IF (k > 0)     /* add segment sum to sum of segment magnitudes */
            {
                IF (sub(nTransWidth, 3) <= 0)
                {
                    tmp2 = sub(k, c1);
                    if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
                    if (tmp2 > 0) n = L_mac(n, nTransWidth_1, (Word16)0x8000);
                    if (tmp2 <= 0) n = L_mac(n, int_sqr[k], c2);
                }
                ELSE
                {
                    tmp2 = sub(k, 12);
                    if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
                    if (tmp2 > 0) n = L_sub(n, 0x70000);
                    if (tmp2 <= 0) n = L_mac(n, int_sqr[k], FL2WORD16(0.03515625f));
                }
                sqErrorNrg = L_add(sqErrorNrg, accu1);
            }
            segmentOffset = add(i, 1); /* no new segments since maxK remains 1 */
        }
        ELSE    /* current line is zero, so update pointers & energy sum */
        {
            if (sub(k, nTransWidth) < 0)
            {
                k = add(k, 1);
            }
            if (sub(maxK, nTransWidth) < 0)
            {
                maxK = sub(maxK, 1);
            }

            /* update segment sum: magnitudes scaled by smoothing function */
            /*accu1 += (float)fabs(x_orig[i]) * inv_gain2 * (float)(k * maxK);*/
            tmp2 = mult(inv_gain2, shl(imult1616(k, maxK), 8));
            accu1 = L_add(accu1, L_abs(Mpy_32_16_1(x_orig[i], tmp2)));
        }
    }

    k = sub(i, segmentOffset);
    IF (k > 0)    /* add last segment sum to sum of segment magnitudes */
    {
        IF (sub(nTransWidth, 3) <= 0)
        {
            tmp2 = sub(k, c1);
            if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
            if (tmp2 > 0) n = L_mac(n, nTransWidth_1, (Word16)0x8000);
            if (tmp2 <= 0) n = L_mac(n, int_sqr[k], c2);
        }
        ELSE
        {
            tmp2 = sub(k, 12);
            if (tmp2 > 0) n = L_msu(n, k, (Word16)0x8000);
            if (tmp2 > 0) n = L_sub(n, 0x70000);
            if (tmp2 <= 0) n = L_mac(n, int_sqr[k], FL2WORD16(0.03515625f));
        }
        sqErrorNrg = L_add(sqErrorNrg, accu1);
    }

    /* noise level factor: average of segment magnitudes of noise bins */
    IF (n > 0)
    {
        tmp1 = BASOP_Util_Divide3232_Scale(Mpy_32_16_1(sqErrorNrg, att), n, &s);
        s = add(add(add(s, x_orig_e), inv_gain2_e), 7 - 15);
        BASOP_SATURATE_WARNING_OFF;
        tmp1 = shl(tmp1, s);
        BASOP_SATURATE_WARNING_ON;
    }
    ELSE
    {
        tmp1 = 0;
        move16();
    }

    /* quantize, dequantize noise level factor (range 0.09375 - 0.65625) */
    tmp2 = round_fx(L_shr(L_mult(tmp1, FL2WORD16_SCALE(1.34375f, 1)), 14-NBITS_NOISE_FILL_LEVEL));

    if (sub(tmp2, (1<<NBITS_NOISE_FILL_LEVEL)-1) > 0)
    {
        tmp2 = (1<<NBITS_NOISE_FILL_LEVEL)-1;
        move16();
    }

    *quantized_fac_ns = tmp2;
    move16();

    *fac_ns = extract_l(L_mult0(*quantized_fac_ns, shr(FL2WORD16(0.75f), NBITS_NOISE_FILL_LEVEL)));
}


void tcx_encoder_memory_update(
    Word16 *wsig,            /* i : target weighted signal */
    Word16 *xn_buf,          /* i/o: mdct output buffer/time domain weigthed synthesis        */
    Word16 L_frame_glob,     /* i: global frame length                         */
    const Word16 *Ai,        /* i: Unquantized (interpolated) LPC coefficients */
    const Word16 *A,         /* i: Quantized LPC coefficients                  */
    Word16 preemph,          /* i: preemphasis factor                          */
    LPD_state *LPDmem,       /* i/o: coder memory state                        */
    Encoder_State_fx *st,
    Word16 *synthout,
    Word16 Q_new,
    Word16 shift
)
{
    Word16 tmp;
    Word16 buf[1+M+LFAC+L_FRAME_PLUS];
    Word16 *synth;


    /* Output synth */
    Copy(xn_buf, synthout, L_frame_glob);


    /* Update synth */
    synth = buf + M+1;
    Copy(LPDmem->syn, buf, M+1);

    Copy(xn_buf, synth, L_frame_glob);
    Copy(synth + sub(L_frame_glob, M+1), LPDmem->syn, M+1);

    IF (st->tcxonly == 0)
    {
        /* Update weighted synthesis */
        Residu3_fx(Ai+(st->nb_subfr-1)*(M+1), synth + sub(L_frame_glob, 1), &tmp, 1, Q_new+shift-1);
        LPDmem->mem_w0 =sub(wsig[sub(L_frame_glob, 1)], tmp);
        move16();
        LPDmem->mem_w0 =shr(LPDmem->mem_w0, shift); /*Qnew-1*/
    }


    /* Emphasis of synth -> synth_pe */
    tmp = synth[-(M+1)];
    move16();
    E_UTIL_f_preemph2(Q_new-1, synth - M, preemph, add(M, L_frame_glob), &tmp);

    Copy(synth + sub(L_frame_glob, M), LPDmem->mem_syn, M);
    Copy(synth + sub(L_frame_glob, M), LPDmem->mem_syn2, M);
    Copy(synth + sub(L_frame_glob, L_SYN_MEM), LPDmem->mem_syn_r, L_SYN_MEM);

    test();
    IF (st->tcxonly == 0 || sub(L_frame_glob,L_FRAME16k)<=0)
    {
        /* Update excitation */
        IF (sub(L_frame_glob, L_EXC_MEM) < 0)
        {
            Copy( LPDmem->old_exc + L_frame_glob, LPDmem->old_exc, sub(L_EXC_MEM, L_frame_glob) );
            Residu3_fx(A, synth, LPDmem->old_exc + sub(L_EXC_MEM, L_frame_glob), L_frame_glob, 1);
        }
        ELSE
        {
            Residu3_fx(A, synth + sub(L_frame_glob, L_EXC_MEM), LPDmem->old_exc, L_EXC_MEM, 1);
        }

    }

}


/*---------------------------------------------------------------
 * Residual Quantization
 *--------------------------------------------------------------*/

/* Returns: number of bits used (including "bits")  Q0 */
Word16 tcx_ari_res_Q_spec(
    const Word32 x_orig[],  /* i: original spectrum                   Q31-e */
    Word16 x_orig_e,        /* i: original spectrum exponent          Q0 */
    const Word16 signs[],   /* i: signs (x_orig[.]<0)                 Q0 */
    Word32 x_Q[],           /* i/o: quantized spectrum                Q31-e */
    Word16 x_Q_e,           /* i: quantized spectrum exponent         Q0 */
    Word16 L_frame,         /* i: number of lines                     Q0 */
    Word16 gain,            /* i: TCX gain                            Q15-e */
    Word16 gain_e,          /* i: TCX gain exponent                   Q0 */
    Word16 prm[],           /* o: bit-stream                          Q0 */
    Word16 target_bits,     /* i: number of bits available            Q0 */
    Word16 bits,            /* i: number of bits used so far          Q0 */
    Word16 deadzone,        /* i: quantizer deadzone                  Q15 */
    const Word16 x_fac[]    /* i: spectrum post-quantization factors  Q14 */
)
{
    Word16 i, j, num_zeros;
    Word16 zeros[L_FRAME_PLUS];
    Word16 fac_p, sign;
    Word32 thres, x_Q_m, x_Q_p;
    Word32 L_tmp, L_tmp2;
    Word16 s, s2;


    /* Limit the number of residual bits */
    target_bits = s_min(target_bits, NPRM_RESQ);


    /* Requantize the spectrum line-by-line */
    /* fac_m = deadzone * 0.5f;
       fac_p = 0.5f - fac_m; */
    num_zeros = 0;
    move16();

    s = sub(add(gain_e, x_Q_e), x_orig_e);
    FOR (i=0; i < L_frame; i++)
    {
        IF (sub(bits, target_bits) >= 0)   /* no bits left */
        {
            BREAK;
        }

        IF (x_Q[i] != 0)
        {
            sign = x_fac[i];
            move16();
            if (signs[i] != 0) sign = negate(sign);

            /* x_Q_m = x_Q[i] - sign*fac_m;
               x_Q_p = x_Q[i] + sign*fac_p; */

            L_tmp = L_mult(sign, deadzone); /* sign*deadzone/2 in Q31 */
            x_Q_m = L_sub(x_Q[i], L_shr(L_tmp, x_Q_e));

            L_tmp = L_mac(L_tmp, sign, (Word16)0x8000); /* sign*(deadzone-1)/2 in Q31 */
            x_Q_p = L_sub(x_Q[i], L_shr(L_tmp, x_Q_e));

            /* if (fabs(x_orig[i] - gain * x_Q_m) < fabs(x_orig[i] - gain * x_Q_p)) */
            L_tmp  = L_abs(L_sub(x_orig[i], L_shl(Mpy_32_16_1(x_Q_m, gain), s)));
            L_tmp2 = L_abs(L_sub(x_orig[i], L_shl(Mpy_32_16_1(x_Q_p, gain), s)));

            IF (L_sub(L_tmp, L_tmp2) < 0)   /* Decrease magnitude */
            {
                x_Q[i] = x_Q_m;
                move32();
                prm[bits] = 0;
                move16();
            }
            ELSE   /* Increase magnitude */
            {
                x_Q[i] = x_Q_p;
                move32();
                prm[bits] = 1;
                move16();
            }
            bits = add(bits, 1);
        }
        ELSE
        {
            zeros[num_zeros] = i;
            move16();
            num_zeros = add(num_zeros, 1);
        }
    }

    /* Requantize zeroed-lines of the spectrum */
    fac_p = msu_r(FL2WORD32(2*0.33f), deadzone, FL2WORD16(2*0.33f)); /* Q16 */
    target_bits = sub(target_bits, 1); /* reserve 1 bit for the check below */

    s = sub(gain_e, x_orig_e);
    s2 = sub(x_Q_e, 1);
    FOR (j = 0; j < num_zeros; j++)
    {
        IF (sub(bits, target_bits) >= 0)   /* 1 or 0 bits left */
        {
            BREAK;
        }

        i = zeros[j];
        move16();

        thres = L_mult(fac_p, x_fac[i]); /* Q31 */

        IF (L_sub(L_abs(x_orig[i]), L_shl(Mpy_32_16_1(thres, gain), s)) > 0)
        {
            prm[bits] = 1;
            move16();
            bits = add(bits, 1);

            prm[bits] = sub(1, signs[i]);
            move16();
            bits = add(bits, 1);

            L_tmp = L_shr(thres, s2);
            if (signs[i]) L_tmp = L_negate(L_tmp);
            x_Q[i] = L_tmp;
            move32();
        }
        ELSE
        {
            prm[bits] = 0;
            move16();
            bits = add(bits, 1);
        }
    }


    return bits;
}

#define kMaxEstimatorOvershoot  5
#define kMaxEstimatorUndershoot 0

Word16 tcx_res_Q_gain(
    Word16 sqGain,
    Word16 sqGain_e,
    Word16 *gain_tcx,
    Word16 *gain_tcx_e,
    Word16 *prm,
    Word16 sqTargetBits
)
{
    Word16 bits;
    Word16 gain_reQ, gain_reQ_e;

    /*Refine the gain quantization : Normal greedy gain coding */

    gain_reQ = *gain_tcx;
    move16();
    gain_reQ_e = *gain_tcx_e;
    move16();

    /* make sure we have a bit of headroom */
    IF (sub(gain_reQ, 0x7000) > 0)
    {
        gain_reQ = shr(gain_reQ, 1);
        gain_reQ_e = add(gain_reQ_e, 1);
    }

    /* bring sqGain to same exponent */
    sqGain = shr(sqGain, sub(gain_reQ_e, sqGain_e));

    FOR (bits=0; bits < TCX_RES_Q_BITS_GAIN; bits++)
    {
        IF (sub(sqGain, gain_reQ) < 0)
        {
            prm[bits] = 0;
            move16();
            gain_reQ = mult_r(gain_reQ, gain_corr_inv_fac[bits]);
        }
        ELSE
        {
            prm[bits] = 1;
            move16();
            gain_reQ = shl(mult_r(gain_reQ, gain_corr_fac[bits]), 1);
        }

        IF (sub(bits, sqTargetBits) < 0)
        {
            *gain_tcx = gain_reQ;
            move16();
            *gain_tcx_e = gain_reQ_e;
            move16();
        }
    }


    return bits;
}

Word16 tcx_res_Q_spec(
    Word32 *x_orig,
    Word16 x_orig_e,
    Word32 *x_Q,
    Word16 x_Q_e,
    Word16 L_frame,
    Word16 sqGain,
    Word16 sqGain_e,
    Word16 *prm,
    Word16 sqTargetBits,
    Word16 bits,
    Word16 sq_round,
    const Word16 lf_deemph_factors[] /* 1Q14 */
)
{
    Word16 i;
    Word16 fac_m, fac_p;
    Word32 tmp1, tmp2;
    Word16 s, s2, lf_deemph_factor;
    Word16 c;
    Word32 thres;


    /* Limit the number of residual bits */
    sqTargetBits = s_min(sqTargetBits, NPRM_RESQ);

    /* Requantize the spectrum line-by-line */
    fac_m = shr(sq_round, 1);
    fac_p = sub(0x4000, fac_m);

    /* exponent difference of x_orig and x_Q * sqGain */
    s = sub(x_orig_e, add(x_Q_e, sqGain_e));

    lf_deemph_factor = 0x4000;
    move16();
    s2 = sub(x_Q_e, 1);

    FOR (i = 0; i < L_frame; i++)
    {
        IF (sub(bits, sub(sqTargetBits, kMaxEstimatorUndershoot)) >= 0)
        {
            fac_m = 0;
            move16();
            fac_p = 0;
            move16();

            IF (sub(bits, s_min(NPRM_RESQ, add(sqTargetBits, kMaxEstimatorOvershoot))) >= 0)
            {
                BREAK;
            }
        }

        test();
        test();
        IF ((x_Q[i] != 0) && ((lf_deemph_factors == NULL) || (sub(lf_deemph_factors[i], 0x2000) > 0)))
        {
            tmp1 = L_add(x_orig[i], 0);
            tmp2 = Mpy_32_16_1(x_Q[i], sqGain);
            if (s > 0) tmp2 = L_shr(tmp2, s);
            if (s < 0) tmp1 = L_shl(tmp1, s);

            if (lf_deemph_factors != NULL)
            {
                lf_deemph_factor = lf_deemph_factors[i];
                move16();
            }

            IF (L_sub(tmp1, tmp2) < 0)
            {
                prm[bits] = 0;
                move16();
                bits = add(bits, 1);

                if (x_Q[i] > 0) tmp1 = L_mult(fac_m, lf_deemph_factor);
                if (x_Q[i] < 0) tmp1 = L_mult(fac_p, lf_deemph_factor);
                x_Q[i] = L_sub(x_Q[i], L_shr(tmp1, s2));
                move32();
            }
            ELSE
            {
                prm[bits] = 1;
                move16();
                bits = add(bits, 1);

                if (x_Q[i] > 0) tmp1 = L_mult(fac_p, lf_deemph_factor);
                if (x_Q[i] < 0) tmp1 = L_mult(fac_m, lf_deemph_factor);
                x_Q[i] = L_add(x_Q[i], L_shr(tmp1, s2));
                move32();
            }
        }
    }

    /*Quantize zeroed-line of the spectrum*/
    c = sub(FL2WORD16(0.66f), mult_r(sq_round, FL2WORD16(0.66f)));

    FOR (i = 0; i < L_frame; i++)
    {
        IF (sub(bits, sub(sqTargetBits, 2)) >= 0)
        {
            BREAK;
        }

        test();
        test();
        IF ((x_Q[i] == 0) && ((lf_deemph_factors == NULL) || (sub(lf_deemph_factors[i], 0x2000) > 0)))
        {
            if (lf_deemph_factors != NULL)
            {
                lf_deemph_factor = lf_deemph_factors[i];
                move16();
            }

            thres = L_mult(c, lf_deemph_factor);
            tmp1 = L_shl(Mpy_32_16_1(thres, sqGain), sub(sqGain_e, x_orig_e));

            IF (L_sub(x_orig[i], tmp1) > 0)
            {
                prm[bits] = 1;
                move16();
                bits = add(bits, 1);

                prm[bits] = 1;
                move16();
                bits = add(bits, 1);

                x_Q[i] = L_shl(thres, sub(1, x_Q_e));
                move32();
            }
            ELSE IF (L_add(x_orig[i], tmp1) < 0)
            {
                prm[bits] = 1;
                move16();
                bits = add(bits, 1);

                prm[bits] = 0;
                move16();
                bits = add(bits, 1);

                x_Q[i] = L_shl(L_negate(thres), sub(1, x_Q_e));
                move32();
            }
            ELSE
            {
                prm[bits] = 0;
                move16();
                bits = add(bits, 1);
            }
        }
    }

    /*Be sure that every possible bits are initialized*/
    FOR (i = bits; i < NPRM_RESQ; i++)
    {
        prm[i] = 0;
        move16();
    }


    return bits;
}
void ProcessIGF(
    IGF_ENC_INSTANCE_HANDLE  const hInstance,          /**< in: instance handle of IGF Encoder */
    Encoder_State_fx              *st,                 /**< in: Encoder state */
    Word32                         pMDCTSpectrum[],    /**< in: MDCT spectrum */
    Word16                        *pMDCTSpectrum_e,
    Word32                         pPowerSpectrum[],   /**< in: MDCT^2 + MDST^2 spectrum, or estimate */
    Word16                        *pPowerSpectrum_e,
    Word16                         isTCX20,            /**< in: flag indicating if the input is TCX20 or TCX10/2xTCX5 */
    Word16                         isTNSActive,        /**< in: flag indicating if the TNS is active */
    Word16                         isTransition,       /**< in: flag indicating if the input is the transition from from ACELP to TCX20/TCX10 */
    Word16                         frameno             /**< in: flag indicating index of current subframe */
)
{
    Word16 igfGridIdx;
    Word16 isIndepFlag;
    Word16 bsBits;
    Word16 bsStart;


    isIndepFlag = 1;
    move16();
    test();
    IF (isTransition && isTCX20)
    {
        igfGridIdx = IGF_GRID_LB_TRAN;
        move16();
    }
    ELSE
    {
        IF (isTCX20)
        {
            igfGridIdx = IGF_GRID_LB_NORM;
            move16();
        }
        ELSE
        {
            /* It is short block */
            igfGridIdx = IGF_GRID_LB_SHORT;
            move16();
            if (sub(frameno, 1) == 0)
            {
                isIndepFlag = 0;
                move16();
            }
        }
    }


    IGFEncApplyMono(hInstance,                   /**< in: instance handle of IGF Encoder */
                    igfGridIdx,                  /**< in: IGF grid index */
                    st,                          /**< in: Encoder state */
                    pMDCTSpectrum,               /**< in: MDCT spectrum */
                    *pMDCTSpectrum_e,
                    pPowerSpectrum,              /**< in: MDCT^2 + MDST^2 spectrum, or estimate */
                    *pPowerSpectrum_e,
                    isTCX20,                     /**< in: flag indicating if the input is TCX20 or TCX10/2xTCX5 */
                    isTNSActive,                 /**< in: flag indicating if the TNS is active */
                    (st->last_core_fx == ACELP_CORE)
                   );
    {
        const Word32 tns_predictionGain = (&st->hIGFEnc)->tns_predictionGain;
        const Word16 startLine = (&st->hIGFEnc)->infoStartLine;
        const Word16 endLine = (&st->hIGFEnc)->infoStopLine;
        const Word16 maxOrder = 8;
        const Word32 *spec_before = (&st->hIGFEnc)->spec_be_igf;
        Word32 x_itf[N_MAX] = {0};
        Word16 j;
        Word16 curr_order = 0;
        Word16 A[ITF_MAX_FILTER_ORDER+1];
        Word16 Q_A;
        Word16 predictionGain = 0;
        Word16 *flatteningTrigger = &(&st->hIGFEnc)->flatteningTrigger;

        FOR (j = startLine; j < endLine; j++)
        {
            x_itf[j] = spec_before[j];
            move32();
        }

        ITF_Detect_fx( x_itf, startLine, endLine, maxOrder, A, &Q_A, &predictionGain, &curr_order, shl((&st->hIGFEnc)->spec_be_igf_e, 1) );

        *flatteningTrigger = 0;
        test();
        IF (L_sub(tns_predictionGain, FL2WORD32_SCALE(1.15, PRED_GAIN_E)) < 0 &&
            sub(predictionGain, FL2WORD16_SCALE(1.15, PRED_GAIN_E)) < 0)
        {
            *flatteningTrigger = 1;
        }
    }

    bsStart = st->next_ind_fx;
    move16();
    hInstance->infoTotalBitsPerFrameWritten = 0;
    move16();
    IF (isTCX20)
    {
        IGFEncWriteBitstream(hInstance,
                             NULL,
                             &hInstance->infoTotalBitsPerFrameWritten,
                             igfGridIdx,
                             isIndepFlag);
    }
    ELSE
    {
        IGFEncWriteBitstream(hInstance,
        st,
        &hInstance->infoTotalBitsPerFrameWritten,
        igfGridIdx,
        isIndepFlag);
    }

    bsBits = sub(st->next_ind_fx, bsStart);
    IF (!isTCX20)
    {
        IGFEncConcatenateBitstream(hInstance, bsBits, &st->next_ind_fx, &st->nb_bits_tot_fx, st->ind_list_fx);
    }

}

void attenuateNbSpectrum(Word16 L_frame, Word32 *spectrum)
{
    Word16 i, length, att;

    length = idiv1616U(L_frame, 20);

    att = FL2WORD16(0.66f);
    move16();
    if (sub(length, 8) == 0)
    {
        att = FL2WORD16(0.6f);
        move16();
    }

    spectrum += sub(L_frame, length);
    FOR (i=0; i < length; i++)
    {
        spectrum[i] = Mpy_32_16_1(spectrum[i], att);
        move32();
        att = mult_r(att, att);
    }
}
