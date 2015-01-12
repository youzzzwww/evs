/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_similarityestimation.c Algorithms for correlation and similarity estimation. */

/* system headers */
#include <assert.h>
#include <stdlib.h>
/* flc header */
#include "stl.h"
#include "basop_util.h"
/* local headers */
#include "jbm_pcmdsp_similarityestimation.h"
#include "options.h"

/* Returns the number of right shifts to be applied to the signal before correlation functions. */
Word16 getSignalScaleForCorrelation(Word32 sampleRate)
{
    Word16 ret;

    IF( L_sub(sampleRate, 16000) < 0 )
    {
        ret = 2;
        move16();
    }
    ELSE IF( L_sub(sampleRate, 32000) >= 0 )
    {
        ret = 4;
        move16();
    }
    ELSE
    {
        ret = 3;
        move16();
    }

    return ret;
}

/* Copies the right shifted signal to another buffer. */
void scaleSignal16(const Word16 *src, Word16 *dst, Word16 n, Word16 rightShift)
{
    Word16 i;

    FOR(i = 0; i < n; i++)
    {
        dst[i] = shr_r(src[i], rightShift);
        move16();
    }

}


/* Calculates cross correlation coefficient for template segment. */
Word32 cross_correlation_self(const Word16 * signal,
                              Word16 x, Word16 y, Word16 corr_len)
{
    Word32 sum;
    Word16 i;

    sum = L_deposit_l(0);
    FOR(i = 0; i < corr_len; i++)
    {
        sum = L_mac0(sum, signal[x + i], signal[y + i]);
    }

    return sum;
}

/* Calculates cross correlation coefficient for template segment. */
Word32 cross_correlation_subsampled_self(const Word16 * signal,
        Word16 x, Word16 y, Word16 corr_len, Word16 subsampling)
{
    Word32 sum;
    Word16 i;

    sum = L_deposit_l(0);
    FOR(i = 0; i < corr_len; i += subsampling)
    {
        sum = L_mac0(sum, signal[x + i], signal[y + i]);
    }

    return sum;
}

/* Calculates normalized cross correlation coefficient for template segment. */
Word16 normalized_cross_correlation_self(const Word16 * signal,
        Word16 x, Word16 y, Word16 corr_len,
        Word16 subsampling, Word32 * energy)
{
    const Word16 *signalX, *signalY;
    Word32 sumXY, sumXX, sumYY, product;
    Word16 sqrtXY, cc;
    Word16 i, normX, normY, normXY, normCC;

    signalX = &signal[x];
    signalY = &signal[y];
    sumXY = L_deposit_l(0);
    sumXX = L_deposit_l(0);
    sumYY = L_deposit_l(0);

    FOR(i = 0; i < corr_len; i += subsampling)
    {
        sumXY = L_mac0(sumXY, signalX[i], signalY[i]);
        sumXX = L_mac0(sumXX, signalX[i], signalX[i]);
        sumYY = L_mac0(sumYY, signalY[i], signalY[i]);
    }

    normX = norm_l(sumXX);
    sumXX = L_shl(sumXX, normX);
    normY = norm_l(sumYY);
    sumYY = L_shl(sumYY, normY);
    product = L_mult0(extract_h(sumXX), extract_h(sumYY));
    normXY = add(normX, normY);
    normXY = sub(normXY, 32);

    /* change norm to factor of 2 */
    IF( s_and(normXY, 0x1) != 0 )
    {
        product = L_shr(product, 1);
        normXY = sub(normXY, 1);
    }
    sqrtXY = getSqrtWord32(product);
    normXY = shr(normXY, 1);

    IF(sqrtXY != 0)
    {
        normCC = 0;
        move16();
        cc = BASOP_Util_Divide3216_Scale(sumXY, sqrtXY, &normCC);
        normCC = add(normCC, 16);
        /* scale to Q15 with saturation */
        BASOP_SATURATE_WARNING_OFF
        cc = shl_r(cc, add(normXY, normCC));
        BASOP_SATURATE_WARNING_ON
        *energy = L_shr_r(L_deposit_l(sqrtXY), normXY);
    }
    ELSE   /* conceal silent frames */
    {
        cc = 0;
        move16();
        *energy = L_deposit_l(1);
    }

    return cc; /* Q15 */
}

/* Splits the signal into segments and checks if all of them have very low energy. */
Word8 isSilence(const Word16 * signal, Word16 len, Word16 segments)
{
    Word16 i, j, samplesPerSegment;
    Word32 energy, maxEnergy;
    Word8 ret;

    assert(len > 0);
    assert(segments > 0);

    /* Every segment is checked using the following formula:
     *   10 * log10(sum_i(signal[i]*signal[i]))) > -65
     * For simplification/complexity, this is replaced by:
     *   20 * log10(sum_i(abs(signal[i]))) > -65
     */

    ret = 1;
    move16();
    energy = L_deposit_l(0);
    samplesPerSegment = idiv1616U(len, segments);
    /* calculate maxEnergy with factor 2 to reduce rounding error */
    maxEnergy = L_mult0(samplesPerSegment, 37); /* 37 = 2 * exp10(-65.0 / 20) * 32768 */
    maxEnergy = L_shr(maxEnergy, 1);
    j = samplesPerSegment;
    move16();
    /* check all but last segment */
    FOR(i = 0; i < len; i++)
    {
        /* division by 32768 is done later */
        energy = L_add(energy, L_abs(L_deposit_l(signal[i])));
        IF( sub(i, j) == 0 )
        {
            /* check energy of current segment */
            /*     20 * log10(energy / 32768 / samplesPerSegment) > -65
             * =>  energy > samplesPerSegment * 10 ^ (-65 / 20) * 32768 */
            IF( L_sub(energy, maxEnergy) > 0 )
            {
                ret = 0;
                move16();
                BREAK;
            }
            energy = L_deposit_l(0);
            j = add(j, samplesPerSegment);
        }
    }
    /* check last segment */
    if( L_sub(energy, maxEnergy) > 0 )
    {
        ret = 0;
        move16();
    }
    return ret;
}

