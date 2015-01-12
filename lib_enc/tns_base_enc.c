/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/



#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include <memory.h>
#include <assert.h>
#include "rom_com_fx.h"

#define HLM_MIN_NRG (32768.0f * 2*NORM_MDCT_FACTOR / (640*640))

/** Get TNS filter parameters from autocorrelation.
  *
  * @param rxx Autocorrelation function/coefficients.
  * @param maxOrder Maximum filter order/number of coefficients.
  * @param pTnsFilter Pointer to the output filter.
  */
static void GetFilterParameters(Word32 rxx[], Word16 maxOrder, STnsFilter * pTnsFilter);

/** Quantization for reflection coefficients.
  *
  * @param parCoeff input reflection coefficients.
  * @param index output quantized values.
  * @param order number of coefficients/values.
  */
static void Parcor2Index(Word16 const parCoeff[], Word16 index[], Word16 order);

/** Linear prediction analysis/synthesis filter definition.
  * @param order filter order.
  * @param parCoeff filter (PARCOR) coefficients.
  * @param state state of the filter. Must be at least of 'order' size.
  * @param x the current input value.
  * @return the output of the filter.
  */
typedef Word32 (* TLinearPredictionFilter)(Word16 order, Word16 const parCoeff[], Word32 * state, Word32 x);


/********************************/
/*      Interface functions     */
/********************************/

#define MAX_SUBDIVISIONS 3

Word16 DetectTnsFilt(STnsConfig const * pTnsConfig,
                     Word32 const pSpectrum[],
                     STnsData * pTnsData,
                     Word16 *predictionGain)
{
    Word16 facs[TNS_MAX_NUM_OF_FILTERS][MAX_SUBDIVISIONS];
    Word16 facs_e[TNS_MAX_NUM_OF_FILTERS][MAX_SUBDIVISIONS]; /* exponents of facs[][] */
    Word16 shifts[TNS_MAX_NUM_OF_FILTERS][MAX_SUBDIVISIONS];
    Word16 iFilter = 0;

    ResetTnsData(pTnsData);

    IF (pTnsConfig->maxOrder <= 0)
    {
        return 0;
    }

    /* Calculate norms for each spectrum part */
    FOR (iFilter = 0; iFilter < pTnsConfig->nMaxFilters; iFilter++)
    {
        Word16 idx0;
        Word16 idx1;
        Word16 nSubdivisions;
        Word16 iSubdivisions;

        move16();
        move16();
        move16();
        idx0 = pTnsConfig->iFilterBorders[iFilter+1];
        idx1 = pTnsConfig->iFilterBorders[iFilter];
        nSubdivisions = pTnsConfig->pTnsParameters[iFilter].nSubdivisions;

        assert(pTnsConfig->pTnsParameters[iFilter].nSubdivisions <= MAX_SUBDIVISIONS);

        FOR (iSubdivisions = 0; iSubdivisions < nSubdivisions; iSubdivisions++)
        {
            Word16 iStartLine;
            Word16 iEndLine;
            Word16 tmp, headroom, shift;
            Word32 L_tmp, tmp32;

            /* iStartLine = idx0 + (idx1-idx0)*iSubdivisions/nSubdivisions;
               iEndLine = idx0 + (idx1-idx0)*(iSubdivisions+1)/nSubdivisions; */
            assert((nSubdivisions == 1) || (nSubdivisions == 3));

            tmp = sub(idx1, idx0);
            iStartLine = imult1616(tmp, iSubdivisions);
            iEndLine = add(iStartLine, tmp);

            if (sub(nSubdivisions, 3) == 0) iStartLine = mult(iStartLine, 0x2AAB);
            iStartLine = add(iStartLine, idx0);

            if (sub(nSubdivisions, 3) == 0) iEndLine = mult(iEndLine, 0x2AAB);
            iEndLine = add(iEndLine, idx0);

            /*norms[iFilter][iSubdivisions] = norm2FLOAT(pSpectrum+iStartLine, iEndLine-iStartLine);*/
            headroom = getScaleFactor32(&pSpectrum[iStartLine], sub(iEndLine, iStartLine));

            /* Calculate norm of spectrum band */
            L_tmp = Norm32Norm(pSpectrum+iStartLine, headroom, sub(iEndLine, iStartLine), &shift);

            /* Check threshold HLM_MIN_NRG */
            BASOP_SATURATE_WARNING_OFF;
            tmp32 = L_sub(L_shl(L_tmp, sub(shift, 24-31*2)), FL2WORD32_SCALE(HLM_MIN_NRG, 24));
            BASOP_SATURATE_WARNING_ON;

            /* get pre-shift for autocorrelation */
            tmp = sub(shift, norm_l(L_tmp)); /* exponent for normalized L_tmp */
            tmp = shr(sub(1, tmp), 1); /* pre-shift to apply before autocorrelation */
            shifts[iFilter][iSubdivisions] = s_min(tmp, headroom);
            move16();

            /* calc normalization factor */
            facs[iFilter][iSubdivisions] = 0;
            move16();
            facs_e[iFilter][iSubdivisions] = 0;
            move16();

            if (tmp32 > 0)
            {
                facs[iFilter][iSubdivisions] = 0x7FFF;
                move16(); /* normalization not needed for one subdivision */
            }

            test();
            IF ((tmp32 > 0) && (sub(nSubdivisions, 1) > 0))
            {
                move16();
                facs_e[iFilter][iSubdivisions] = shl(sub(tmp, shifts[iFilter][iSubdivisions]), 1);

                tmp = sub(1, shl(tmp, 1)); /* exponent of autocorrelation */
                L_tmp = L_shl(L_tmp, sub(shift, tmp)); /* shift L_tmp to that exponent */

                /* calc factor (with 2 bits headroom for sum of 3 subdivisions) */
                move16();
                facs[iFilter][iSubdivisions] = div_s(0x2000, round_fx(L_tmp)); /* L_tmp is >= 0x2000000 */
            }

        }

    }
    /* Calculate normalized autocorrelation for spectrum subdivision and get TNS filter parameters based on it */
    FOR (iFilter = 0; iFilter < pTnsConfig->nMaxFilters; iFilter++)
    {
#define RXX_E (3)
        Word32 rxx[TNS_MAX_FILTER_ORDER+1];
        Word16 idx0;
        Word16 idx1;
        Word16 spectrumLength;
        STnsFilter * pFilter;
        Word16 nSubdivisions;
        Word16 iSubdivisions;
        Word16 tmpbuf[325];

        set32_fx(rxx, 0, TNS_MAX_FILTER_ORDER+1);

        move16();
        move16();
        move16();
        idx0 = pTnsConfig->iFilterBorders[iFilter+1];
        idx1 = pTnsConfig->iFilterBorders[iFilter];
        spectrumLength = sub(idx1, idx0);
        pFilter = pTnsData->filter + iFilter;
        nSubdivisions = pTnsConfig->pTnsParameters[iFilter].nSubdivisions;

        FOR (iSubdivisions = 0; iSubdivisions < nSubdivisions; iSubdivisions++)
        {
            Word16 iStartLine, n, i;
            Word16 iEndLine;
            const Word16 * pWindow;
            Word16 lag, shift;
            Word32 L_tmp;

            IF ( facs[iFilter][iSubdivisions] == 0 )
            {
                BREAK;
            }


            /* iStartLine = idx0 + (idx1-idx0)*iSubdivisions/nSubdivisions;
               iEndLine = idx0 + (idx1-idx0)*(iSubdivisions+1)/nSubdivisions; */
            assert((nSubdivisions == 1) || (nSubdivisions == 3));

            iStartLine = imult1616(spectrumLength, iSubdivisions);
            iEndLine = add(iStartLine, spectrumLength);

            if (sub(nSubdivisions, 3) == 0) iStartLine = mult(iStartLine, 0x2AAB);
            iStartLine = add(iStartLine, idx0);

            if (sub(nSubdivisions, 3) == 0) iEndLine = mult(iEndLine, 0x2AAB);
            iEndLine = add(iEndLine, idx0);


            move16();
            shift = shifts[iFilter][iSubdivisions];

            move16();
            pWindow = tnsAcfWindow;

            n = sub(iEndLine, iStartLine);
            assert(n < (Word16)(sizeof(tmpbuf)/sizeof(Word16)));
            FOR (i = 0; i < n; i++)
            {
                tmpbuf[i] = round_fx(L_shl(pSpectrum[iStartLine+i], shift));
            }

            FOR (lag = 0; lag <= pTnsConfig->maxOrder; lag++)
            {
                n = sub(sub(iEndLine,lag), iStartLine);

                L_tmp = L_deposit_l(0);
                FOR (i = 0; i < n; i++)
                {
                    L_tmp = L_mac0(L_tmp, tmpbuf[i], tmpbuf[i+lag]);
                }

                if (lag != 0) L_tmp = Mpy_32_16_1(L_tmp, *pWindow++);

                L_tmp = Mpy_32_16_1(L_tmp, facs[iFilter][iSubdivisions]);
                L_tmp = L_shl(L_tmp, facs_e[iFilter][iSubdivisions]);

                rxx[lag] = L_add(rxx[lag], L_tmp);
                move32();
            }

        }

        IF ( sub(iSubdivisions,nSubdivisions) == 0 ) /* meaning there is no subdivision with low energy */
        {
            pFilter->spectrumLength = spectrumLength;
            move16();
            /* Limit the maximum order to spectrum length/4 */
            GetFilterParameters(rxx, s_min (pTnsConfig->maxOrder, shr(pFilter->spectrumLength,2)), pFilter);
        }
    }

    if (predictionGain)
    {
        assert(pTnsConfig->nMaxFilters == 1);
        move16();
        *predictionGain = pTnsData->filter->predictionGain;
    }

    /* We check the filter's decisions in the opposite direction */
    FOR (iFilter = sub(pTnsConfig->nMaxFilters,1); iFilter >= 0; iFilter--)
    {
        STnsFilter * pFilter;
        struct TnsParameters const * pTnsParameters;

        move16();
        move16();
        pFilter = pTnsData->filter + iFilter;
        pTnsParameters = pTnsConfig->pTnsParameters + iFilter;

        IF ( s_or(sub(pFilter->predictionGain,pTnsParameters->minPredictionGain) > 0,
                  sub(pFilter->avgSqrCoef,pTnsParameters->minAvgSqrCoef) > 0 ) )
        {
            move16();
            pTnsData->nFilters = add(pTnsData->nFilters,1);
        }
        ELSE IF (pTnsData->nFilters > 0) /* If a previous filter is turned on */
        {
            /* Since TNS filter of order 0 is not allowed we haved to signal in the stream filter of order 1 with the 0th coefficient equal to 0 */
            ClearTnsFilterCoefficients(pFilter);
            move16();
            move16();
            pFilter->order = 1;
            pTnsData->nFilters = add(pTnsData->nFilters,1);
        }
        ELSE
        {
            ClearTnsFilterCoefficients(pFilter);
        }
    }


    test();
    return (pTnsData->nFilters > 0);
}

Word16 EncodeTnsData(STnsConfig const * pTnsConfig, STnsData const * pTnsData, Word16 * stream, Word16 * pnSize, Word16 * pnBits)
{

    move16();
    move16();
    *pnSize = 0;
    *pnBits = 0;

    IF ( sub(pTnsConfig->nMaxFilters, 1) > 0 )
    {

        IF ( sub(pTnsConfig->iFilterBorders[0],512) < 0 )
        {
            GetParameters(tnsEnabledSWBTCX10BitMap, 1, pTnsData, &stream, pnSize, pnBits);
        }
        ELSE
        {
            GetParameters(tnsEnabledSWBTCX20BitMap, 1, pTnsData, &stream, pnSize, pnBits);
        }
    }
    ELSE
    {

        IF ( sub(pTnsConfig->iFilterBorders[0],240) < 0 )
        {
            GetParameters(tnsEnabledWBTCX10BitMap, 1, pTnsData, &stream, pnSize, pnBits);
        }
        ELSE
        {
            GetParameters(tnsEnabledWBTCX20BitMap, 1, pTnsData, &stream, pnSize, pnBits);
        }
    }

    return TNS_NO_ERROR;
}

Word16 WriteTnsData(STnsConfig const * pTnsConfig, Word16 const * stream, Word16 * pnSize, Encoder_State_fx *st, Word16 * pnBits)
{

    IF ( sub(pTnsConfig->nMaxFilters,1) > 0 )
    {

        IF ( sub(pTnsConfig->iFilterBorders[0],512) < 0 )
        {
            WriteToBitstream(tnsEnabledSWBTCX10BitMap, 1, &stream, pnSize, st, pnBits);
        }
        ELSE
        {
            WriteToBitstream(tnsEnabledSWBTCX20BitMap, 1, &stream, pnSize, st, pnBits);
        }
    }
    ELSE
    {

        IF ( sub(pTnsConfig->iFilterBorders[0],240) < 0 )
        {
            WriteToBitstream(tnsEnabledWBTCX10BitMap, 1, &stream, pnSize, st, pnBits);
        }
        ELSE
        {
            WriteToBitstream(tnsEnabledWBTCX20BitMap, 1, &stream, pnSize, st, pnBits);
        }
    }


    return TNS_NO_ERROR;
}

/*********************************************************************************************/
/*  Definitions of functions used in the mapping between TNS parameters and a bitstream.     */
/*********************************************************************************************/

/* Helper functions for Hufmann table coding */

/********************************/
/*      Private functions       */
/********************************/

static void GetFilterParameters(Word32 rxx[], Word16 maxOrder, STnsFilter * pTnsFilter)
{
    Word16 i;
    Word16 parCoeff[TNS_MAX_FILTER_ORDER];
    Word32 epsP[TNS_MAX_FILTER_ORDER+1], L_tmp;
#if TNS_COEF_RES == 5
    Word16 const * values = tnsCoeff5;
#elif TNS_COEF_RES == 4
    Word16 const * values = tnsCoeff4;
#elif TNS_COEF_RES == 3
    Word16 const * values = tnsCoeff3;
#endif
    Word16 * indexes = pTnsFilter->coefIndex;


    /* compute TNS filter in lattice (ParCor) form with LeRoux-Gueguen algorithm */
    L_tmp = E_LPC_schur(rxx, parCoeff, epsP, maxOrder);
    BASOP_SATURATE_WARNING_OFF /* Allow saturation, this value is compared against a threshold. */
    pTnsFilter->predictionGain = divide3232(L_shr(epsP[0], PRED_GAIN_E), L_tmp);
    BASOP_SATURATE_WARNING_ON
    /* non-linear quantization of TNS lattice coefficients with given resolution */
    Parcor2Index(parCoeff, indexes, maxOrder);

    /* reduce filter order by truncating trailing zeros */
    i = sub(maxOrder,1);

    WHILE ((i >= 0) && (indexes[i] == 0))
    {
        i = sub(i, 1);
    }

    move16();
    pTnsFilter->order = add(i, 1);

    /* compute avg(coef*coef) */
    L_tmp = L_deposit_l(0);

    FOR (i = pTnsFilter->order-1; i >= 0; i--)
    {
        Word16 value ;

        move16();
        value = shr(values[indexes[i]+INDEX_SHIFT], 1);

        move16();
        L_tmp = L_mac0(L_tmp, value, value);

    }
    move16();
    pTnsFilter->avgSqrCoef = round_fx(L_tmp);

    /* assert(maxOrder == 8);
     pTnsFilter->avgSqrCoef = shr(pTnsFilter->avgSqrCoef, 3); */

}

static void Parcor2Index(const Word16 parCoeff[] /*Q15*/, Word16 index[], Word16 order)
{
    Word16 nValues;
    Word16 const * values;
    Word16 i;
    Word16 iIndex;
    Word16 x;


    move16();
    move16();
    nValues = 1 << TNS_COEF_RES;
#if TNS_COEF_RES == 5
    values = tnsCoeff5;
#elif TNS_COEF_RES == 4
    values = tnsCoeff4;
#elif TNS_COEF_RES == 3
    values = tnsCoeff3;
#endif

    FOR (i = 0; i < order; i++)
    {
        move16();
        move16();
        iIndex = 1;
        x = parCoeff[i];

        /* parCoeff is in the range of -1.0 ... 1.0 by definition */
        /* assert((x >= FL2WORD16(-1.0f)) && (x <= FL2WORD16(1.0f))); */

        WHILE ((iIndex < nValues) && (x > add(shr(values[iIndex-1], 1), shr(values[iIndex], 1))) )
        {
            iIndex = add(iIndex,1);

        }
        index[i] = sub(iIndex, 1 + INDEX_SHIFT);
    }

}

