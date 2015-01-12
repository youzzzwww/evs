/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "stl.h"
#include "stat_com.h"
#include "stl.h"
#include <memory.h>
#include <assert.h>
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "basop_util.h"

#define HLM_MIN_NRG 32768.0f

/** Linear prediction analysis/synthesis filter definition.
  * @param order filter order.
  * @param parCoeff filter (PARCOR) coefficients.
  * @param state state of the filter. Must be at least of 'order' size.
  * @param x the current input value.
  * @return the output of the filter.
  */
typedef Word32 (* TLinearPredictionFilter)(Word16 order, Word16 const parCoeff[], Word32 * state, Word32 x);

/** Inverse quantization for reflection coefficients.
  *
  * @param index input quantized values.
  * @param parCoeff output reflection coefficients.
  * @param order number of coefficients/values.
  */
static void Index2Parcor(Word16 const index[], Word16 parCoeff[], Word16 order);

/** Linear prediction analysis filter.
  * See TLinearPredictionFilter for details.
  */
static Word32 FIRLattice(Word16 order, const Word16 *parCoeff, Word32 *state, Word32 x);

/** Linear prediction synthesis filter.
  * See TLinearPredictionFilter for details.
  */
static Word32 IIRLattice(Word16 order, const Word16 *parCoeff, Word32 *state, Word32 x);

/** TNS analysis/synthesis filter.
  * @param spectrum input spectrum values.
  * @param numOfLines number of lines in the spectrum.
  * @param parCoeff filter (PARCOR) coefficients.
  * @param order filter order.
  * @param direction direction in which to filter.
  * @param filter function that implements filtering.
    By this function it is defined whether analysis or synthesis is performed.
  * @param output filtered output spectrum values.
    Inplace operation is supported, so it can be equal to spectrum.
  */
static void TnsFilter(Word32 const spectrum[], Word16 numOfLines,
                      Word16 const parCoeff[], Word16 order, Word16 direction,
                      TLinearPredictionFilter filter, Word32 * state,
                      Word32 output[]);

static void ITF_TnsFilter_fx( Word32 const spectrum[],
                              const Word16 numOfLines,
                              const Word16 A[], /* Q14 */
                              const Word16 Q_A,
                              const Word16 order,
                              Word32 output[]);

static void ITF_GetFilterParameters_fx(Word32 rxx[],
                                       Word16 maxOrder,
                                       Word16* A, /* Q14 */
                                       Word16* Q_A,
                                       Word16* predictionGain);

/********************************/
/*      Interface functions     */
/********************************/

#define MAX_SUBDIVISIONS 3

Word16 InitTnsConfiguration(
    Word32 nSampleRate,
    Word16 frameLength,
    STnsConfig * pTnsConfig,
    Word16 igfStopFreq,
    Word32 bitrate
)
{
    Word16 iFilter = 0;
    Word16 * startLineFilter;
    Word32 L_tmp;
    Word16 s1;
    Word16 s2;


    move16();
    startLineFilter = &pTnsConfig->iFilterBorders[1];

    /* Sanity checks */
    assert((nSampleRate > 0) && (frameLength > 0) && (pTnsConfig != NULL));
    if ((nSampleRate <= 0) || (frameLength <= 0) || (pTnsConfig == NULL))
    {
        return TNS_FATAL_ERROR;
    }


    /* Initialize TNS filter flag and maximum order */
    move16();
    pTnsConfig->maxOrder    = TNS_MAX_FILTER_ORDER;

    IF (L_sub(bitrate, ACELP_32k) <= 0)
    {
        move16();
        move16();
        pTnsConfig->nMaxFilters = sizeof(tnsParametersIGF32kHz_LowBR)/sizeof(tnsParametersIGF32kHz_LowBR[0]);
        pTnsConfig->pTnsParameters = tnsParametersIGF32kHz_LowBR;
    }
    ELSE
    {
        test();
        IF (L_sub(nSampleRate,32000) > 0 && L_sub(nSampleRate, L_mult0(100, frameLength)) == 0)
        {
            move16();
            pTnsConfig->nMaxFilters = sizeof(tnsParameters48kHz_grouped)/sizeof(tnsParameters48kHz_grouped[0]);
            move16();
            pTnsConfig->pTnsParameters = tnsParameters48kHz_grouped;
        }
        ELSE
        IF ( L_sub(nSampleRate,16000) > 0 )
        {
            move16();
            pTnsConfig->nMaxFilters = sizeof(tnsParameters32kHz)/sizeof(tnsParameters32kHz[0]);

            move16();
            pTnsConfig->pTnsParameters = tnsParameters32kHz;

            if ( L_sub(nSampleRate, L_mult0(100, frameLength)) == 0 )    /* sub-frame length is <= 10 ms */
            {
                move16();
                pTnsConfig->pTnsParameters = tnsParameters32kHz_grouped;
            }
        }
        ELSE
        {
            IF ( L_sub(nSampleRate, L_mult0(100, frameLength)) == 0 )    /* sub-frame length is <= 10 ms */
            {
                move16();
                move16();
                pTnsConfig->nMaxFilters = sizeof(tnsParameters16kHz_grouped)/sizeof(tnsParameters16kHz_grouped[0]);
                pTnsConfig->pTnsParameters = tnsParameters16kHz_grouped;
            }
            ELSE
            {
                move16();
                move16();
                pTnsConfig->nMaxFilters = sizeof(tnsParameters16kHz)/sizeof(tnsParameters16kHz[0]);
                pTnsConfig->pTnsParameters = tnsParameters16kHz;
            }
        }
    }

    assert(pTnsConfig->nMaxFilters <= TNS_MAX_NUM_OF_FILTERS);

    /* Set starting MDCT line for each filter based on the starting frequencies from the TNS table */

    FOR (iFilter = 0; iFilter < pTnsConfig->nMaxFilters; iFilter++)
    {
        assert(pTnsConfig->pTnsParameters[iFilter].startLineFrequency < 0.5f*nSampleRate);
        assert(nSampleRate <= 96000);
        move16();
        startLineFilter[iFilter] = divide3232(L_mult0(frameLength, pTnsConfig->pTnsParameters[iFilter].startLineFrequency), L_shl(nSampleRate,14));
    }

    IF (igfStopFreq > 0)
    {
        L_tmp = L_mult(frameLength, igfStopFreq);
        s1    = sub(norm_l(L_tmp), 1);
        s2    = norm_l(nSampleRate);

        move16();
        pTnsConfig->iFilterBorders[0] = shr(  div_l(  L_shl(L_tmp, s1), extract_h(L_shl(nSampleRate, s2)) )  , sub(WORD16_BITS-1, sub(s2, s1)));

    }
    ELSE
    {
        move16();
        pTnsConfig->iFilterBorders[0] = frameLength;
    }

    return TNS_NO_ERROR;
}


Word16 ApplyTnsFilter(
    STnsConfig const * pTnsConfig,
    STnsData const * pTnsData,
    Word32 spectrum[],
    Word8 fIsAnalysis)
{
    Word16 result;
    TLinearPredictionFilter filter;
    Word32 state[TNS_MAX_FILTER_ORDER];
    Word16 iFilter;
    Word16 stopLine, startLine;
    Word16 const * pBorders;


    move16();
    filter = IIRLattice;
    if (fIsAnalysis)
    {
        move16();
        filter = FIRLattice;
    }
    set32_fx(state, 0, TNS_MAX_FILTER_ORDER);
    move16();
    pBorders = pTnsConfig->iFilterBorders;

    FOR (iFilter = pTnsConfig->nMaxFilters-1; iFilter >= 0; iFilter--)
    {
        Word16 parCoeff[TNS_MAX_FILTER_ORDER];
        STnsFilter const * pFilter;


        move16();
        move16();
        move16();
        pFilter = &pTnsData->filter[iFilter];
        stopLine = pBorders[iFilter];
        startLine = pBorders[iFilter+1];

        Index2Parcor(pFilter->coefIndex, parCoeff, pFilter->order);

        TnsFilter(&spectrum[startLine], stopLine-startLine,
                  parCoeff, pFilter->order, pFilter->direction, filter, state,
                  &spectrum[startLine]);

    }

    move16();
    result = TNS_NO_ERROR;
    if (pTnsData->nFilters < 0)
    {
        move16();
        result = TNS_FATAL_ERROR;
    }

    return result;
}

TNS_ERROR ITF_Apply_fx(Word32 spectrum[],
                       Word16 startLine,
                       Word16 stopLine,
                       const Word16* A,
                       Word16 Q_A,
                       Word16 order
                      )
{

    ITF_TnsFilter_fx(&spectrum[startLine],
                     (Word16)(stopLine-startLine),
                     A,
                     Q_A,
                     (Word16)order,
                     &spectrum[startLine]);

    return TNS_NO_ERROR;
}

Word16 ITF_Detect_fx(Word32 const pSpectrum[],
                     Word16 startLine,
                     Word16 stopLine,
                     Word16 maxOrder,
                     Word16* A,
                     Word16* Q_A,
                     Word16* predictionGain,
                     Word16* curr_order,
                     Word16 Q   )
{
    Word16 const idx0 = startLine;
    Word16 const idx1 = stopLine;
    Word16 spectrumLength;
    Word16 const nSubdivisions = MAX_SUBDIVISIONS;
    Word16 iSubdivisions;
    Word16 iStartLine;
    Word16 iEndLine;
    Word16 facs[MAX_SUBDIVISIONS];
    Word16 facs_e[MAX_SUBDIVISIONS]; /* exponents of facs[][] */
    Word16 shifts[MAX_SUBDIVISIONS];
    Word16 tmp, headroom, shift;
    Word32 rxx[ITF_MAX_FILTER_ORDER+1];
    Word16 lag;
    Word32 L_tmp, tmp32;
    Word16 tmpbuf[325];
    Word16 n, i;

    move16();
    move16();
    move16();

    if (maxOrder <= 0)
    {
        return 0;
    }

    /* Calculate norms for each spectrum part */
    FOR (iSubdivisions = 0; iSubdivisions < nSubdivisions; iSubdivisions++)
    {
        assert((nSubdivisions == 1) || (nSubdivisions == 3));

        tmp = sub(idx1, idx0);
        iStartLine = imult1616(tmp, iSubdivisions);
        iEndLine = add(iStartLine, tmp);

        if (sub(nSubdivisions, 3) == 0) iStartLine = mult(iStartLine, 0x2AAB);
        iStartLine = add(iStartLine, idx0);

        if (sub(nSubdivisions, 3) == 0) iEndLine = mult(iEndLine, 0x2AAB);
        iEndLine = add(iEndLine, idx0);

        headroom = getScaleFactor32(&pSpectrum[iStartLine], sub(iEndLine, iStartLine));

        /* Calculate norm of spectrum band */
        L_tmp = Norm32Norm(pSpectrum+iStartLine, headroom, sub(iEndLine, iStartLine), &shift);

        /* Check threshold HLM_MIN_NRG */
        BASOP_SATURATE_WARNING_OFF;
        tmp32 = L_sub(L_shl(L_tmp, sub(shift, 24-Q)), FL2WORD32_SCALE(HLM_MIN_NRG, 24));
        BASOP_SATURATE_WARNING_ON;

        /* get pre-shift for autocorrelation */
        tmp = sub(shift, norm_l(L_tmp)); /* exponent for normalized L_tmp */
        tmp = shr(sub(1, tmp), 1); /* pre-shift to apply before autocorrelation */
        shifts[iSubdivisions] = s_min(tmp, headroom);
        move16();

        /* calc normalization factor */
        facs[iSubdivisions] = 0;
        move16();
        facs_e[iSubdivisions] = 0;
        move16();

        if (tmp32 > 0)
        {
            facs[iSubdivisions] = 0x7FFF;
            move16(); /* normalization not needed for one subdivision */
        }

        test();
        IF ((tmp32 > 0) && (sub(nSubdivisions, 1) > 0))
        {
            move16();
            facs_e[iSubdivisions] = shl(sub(tmp, shifts[iSubdivisions]), 1);

            tmp = sub(1, shl(tmp, 1)); /* exponent of autocorrelation */
            L_tmp = L_shl(L_tmp, sub(shift, tmp)); /* shift L_tmp to that exponent */

            /* calc factor (with 2 bits headroom for sum of 3 subdivisions) */
            facs[iSubdivisions] = div_s(0x2000, round_fx(L_tmp)); /* L_tmp is >= 0x2000000 */ move16();
        }

    }

    /* Calculate normalized autocorrelation for spectrum subdivision and get filter parameters based on it */
#define RXX_E (3)
    {
        set32_fx(rxx, 0, ITF_MAX_FILTER_ORDER+1);

        spectrumLength = sub(idx1, idx0);

        FOR (iSubdivisions = 0; iSubdivisions < nSubdivisions; iSubdivisions++)
        {
            IF ( facs[iSubdivisions] == 0 )
            {
                BREAK;
            }


            assert((nSubdivisions == 1) || (nSubdivisions == 3));

            iStartLine = imult1616(spectrumLength, iSubdivisions);
            iEndLine = add(iStartLine, spectrumLength);

            if (sub(nSubdivisions, 3) == 0) iStartLine = mult(iStartLine, 0x2AAB);
            iStartLine = add(iStartLine, idx0);

            if (sub(nSubdivisions, 3) == 0) iEndLine = mult(iEndLine, 0x2AAB);
            iEndLine = add(iEndLine, idx0);


            move16();
            shift = shifts[iSubdivisions];

            n = sub(iEndLine, iStartLine);
            assert(n < (Word16)(sizeof(tmpbuf)/sizeof(Word16)));
            FOR (i = 0; i < n; i++)
            {
                tmpbuf[i] = round_fx(L_shl(pSpectrum[iStartLine+i], shift));
            }

            FOR (lag = 0; lag <= maxOrder; lag++)
            {
                n = sub(sub(iEndLine,lag), iStartLine);

                L_tmp = L_deposit_l(0);
                FOR (i = 0; i < n; i++)
                {
                    L_tmp = L_mac0(L_tmp, tmpbuf[i], tmpbuf[i+lag]);
                }

                L_tmp = Mpy_32_16_1(L_tmp, facs[iSubdivisions]);
                L_tmp = L_shl(L_tmp, facs_e[iSubdivisions]);

                rxx[lag] = L_add(rxx[lag], L_tmp);
                move32();
            }

        }

        IF ( sub(iSubdivisions,nSubdivisions) == 0 ) /* meaning there is no subdivision with low energy */
        {
            /* Limit the maximum order to spectrum length/4 */
            ITF_GetFilterParameters_fx(rxx, s_min (maxOrder, shr(spectrumLength,2)), A, Q_A, predictionGain);

            *curr_order = maxOrder;
        }

    }

    return 1;
}


/* Helper functions for Hufmann table coding */


/** Get number of bits from a Huffman table.
  * The table must be sorted by values.
  */
static Word16 GetBitsFromTable(Word16 value, Coding codes[], Word16 nSize)
{
    (void)nSize;
    assert((value >= 0) && (value < nSize) && (nSize >= 0) && (nSize <= 256));

    move16();
    cast16();
    return (Word16)codes[value].nBits;
}

/** Get the code for a value from a Huffman table.
  * The table must be sorted by values.
  */
static Word16 EncodeUsingTable(Word16 value, Coding codes[], Word16 nSize)
{
    (void)nSize;
    assert((value >= 0) && (value < nSize) && (nSize >= 0) && (nSize <= 256));

    move16();
    return codes[value].code;
}


/** Decode a value from a bitstream using a Huffman table. */
static Word16 DecodeUsingTable(Decoder_State_fx *st, Word16 * pValue, Coding codes[], Word16 nSize)
{
    Word16 code = 0;
    Word16 nBits = 0;
    Word16 valueIndex;

    assert((nSize >= 0) && (nSize <= 256));


    move16();
    valueIndex = nSize;


    WHILE (valueIndex == nSize)
    {
        code = add(shl(code,1), get_next_indice_fx(st, 1));
        nBits = add(nBits,1);
        assert((nBits <= nSize) && (nBits <= 16));

        FOR (valueIndex = 0; valueIndex < nSize; valueIndex++)
        {

            IF ( s_and(sub(codes[valueIndex].nBits,nBits) == 0, sub(codes[valueIndex].code,code) == 0) )
            {
                BREAK;
            }
        }

    }
    assert( (valueIndex < nSize) );

    cast16();
    move16();
    *pValue  = (Word16)codes[valueIndex].value;

    return codes[valueIndex].nBits;
}


/* TNS filter coefficients */

void const * GetTnsFilterCoeff(void const * p, Word16 index, Word16 * pValue)
{
    *pValue = ((Word16 const *)p)[index] + INDEX_SHIFT;
    return NULL;
}

void * SetTnsFilterCoeff(void * p, Word16 index, Word16 value)
{
    ((Word16 *)p)[index] = sub(value, INDEX_SHIFT);
    return NULL;
}


Word16 GetSWBTCX20TnsFilterCoeffBits(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return GetBitsFromTable(value, codesTnsCoeffSWBTCX20[index], nTnsCoeffCodes);
}

Word16 EncodeSWBTCX20TnsFilterCoeff(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return EncodeUsingTable(value, codesTnsCoeffSWBTCX20[index], nTnsCoeffCodes);
}

Word16 DecodeSWBTCX20TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return DecodeUsingTable(st, pValue, codesTnsCoeffSWBTCX20[index], nTnsCoeffCodes);
}

Word16 GetSWBTCX10TnsFilterCoeffBits(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return GetBitsFromTable(value, codesTnsCoeffSWBTCX10[index], nTnsCoeffCodes);
}

Word16 EncodeSWBTCX10TnsFilterCoeff(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return EncodeUsingTable(value, codesTnsCoeffSWBTCX10[index], nTnsCoeffCodes);
}

Word16 DecodeSWBTCX10TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return DecodeUsingTable(st, pValue, codesTnsCoeffSWBTCX10[index], nTnsCoeffCodes);
}

Word16 GetWBTCX20TnsFilterCoeffBits(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return GetBitsFromTable(value, codesTnsCoeffWBTCX20[index], nTnsCoeffCodes);
}

Word16 EncodeWBTCX20TnsFilterCoeff(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return EncodeUsingTable(value, codesTnsCoeffWBTCX20[index], nTnsCoeffCodes);
}

Word16 DecodeWBTCX20TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return DecodeUsingTable(st, pValue, codesTnsCoeffWBTCX20[index], nTnsCoeffCodes);
}

Word16 GetWBTCX10TnsFilterCoeffBits(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return GetBitsFromTable(value, codesTnsCoeffWBTCX10[index], nTnsCoeffCodes);
}

Word16 EncodeWBTCX10TnsFilterCoeff(Word16 value, Word16 index)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return EncodeUsingTable(value, codesTnsCoeffWBTCX10[index], nTnsCoeffCodes);
}

Word16 DecodeWBTCX10TnsFilterCoeff(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    assert((index >= 0) && (index < nTnsCoeffTables));

    return DecodeUsingTable(st, pValue, codesTnsCoeffWBTCX10[index], nTnsCoeffCodes);
}


/* TNS filter order */

void const * GetTnsFilterOrder(void const * p, Word16 index, Word16 * pValue)
{
    move16();
    *pValue = ((STnsFilter const *)p)[index].order;

    move16();
    return ((STnsFilter const *)p)[index].coefIndex;
}

void * SetTnsFilterOrder(void * p, Word16 index, Word16 value)
{
    move16();
    ((STnsFilter *)p)[index].order = value;

    move16();
    return ((STnsFilter *)p)[index].coefIndex;
}

Word16 GetTnsFilterOrderBitsSWBTCX20(Word16 value, Word16 index)
{
    (void)index;

    return GetBitsFromTable(value-1, codesTnsOrderTCX20, nTnsOrderCodes);
}

Word16 EncodeTnsFilterOrderSWBTCX20(Word16 value, Word16 index)
{
    (void)index;

    return EncodeUsingTable(value-1, codesTnsOrderTCX20, nTnsOrderCodes);
}

Word16 DecodeTnsFilterOrderSWBTCX20(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    (void)index;

    return DecodeUsingTable(st, pValue, codesTnsOrderTCX20, nTnsOrderCodes);
}

Word16 GetTnsFilterOrderBitsSWBTCX10(Word16 value, Word16 index)
{
    (void)index;

    return GetBitsFromTable(value-1, codesTnsOrderTCX10, nTnsOrderCodes);
}

Word16 EncodeTnsFilterOrderSWBTCX10(Word16 value, Word16 index)
{
    (void)index;

    return EncodeUsingTable(value-1, codesTnsOrderTCX10, nTnsOrderCodes);
}

Word16 DecodeTnsFilterOrderSWBTCX10(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    (void)index;

    return DecodeUsingTable(st, pValue, codesTnsOrderTCX10, nTnsOrderCodes);
}

Word16 GetTnsFilterOrderBits(Word16 value, Word16 index)
{
    (void)index;

    return GetBitsFromTable(value-1, codesTnsOrder, nTnsOrderCodes);
}

Word16 EncodeTnsFilterOrder(Word16 value, Word16 index)
{
    (void)index;

    return EncodeUsingTable(value-1, codesTnsOrder, nTnsOrderCodes);
}

Word16 DecodeTnsFilterOrder(Decoder_State_fx *st, Word16 index, Word16 * pValue)
{
    (void)index;

    return DecodeUsingTable(st, pValue, codesTnsOrder, nTnsOrderCodes);
}

/* Number of TNS filters */

void const * GetNumOfTnsFilters(void const * p, Word16 index, Word16 * pValue)
{
    *pValue = ((STnsData const *)p)[index].nFilters;

    return ((STnsData const *)p)[index].filter;
}

void * SetNumOfTnsFilters(void * p, Word16 index, Word16 value)
{
    ((STnsData *)p)[index].nFilters = value;

    return ((STnsData *)p)[index].filter;
}

/* TNS enabled/disabled flag */

void const * GetTnsEnabled(void const * p, Word16 index, Word16 * pValue)
{
    *pValue = ((STnsData const *)p)[index].nFilters > 0 ? 1 : 0;
    return NULL;
}

void * SetTnsEnabled(void * p, Word16 index, Word16 value)
{
    (void)p,(void)index,(void)value;
    return NULL;
}

void const * GetTnsEnabledSingleFilter(void const * p, Word16 index, Word16 * pValue)
{
    *pValue = ((STnsData const *)p)[index].nFilters > 0 ? 1 : 0;
    return ((STnsData const *)p)[index].filter;
}

void * SetTnsEnabledSingleFilter(void * p, Word16 index, Word16 value)
{
    ((STnsData *)p)[index].nFilters = value;
    return ((STnsData *)p)[index].filter;
}

/********************************/
/*      Private functions       */
/********************************/

void ResetTnsData(STnsData * pTnsData)
{
    Word16 iFilter;


    pTnsData->nFilters = 0;

    FOR (iFilter = 0; iFilter < (Word16) (sizeof(pTnsData->filter)/sizeof(pTnsData->filter[0])); iFilter++)
    {
        STnsFilter * const pTnsFilter = &pTnsData->filter[iFilter];
        pTnsFilter->spectrumLength = 0;
        pTnsFilter->direction = DEFAULT_FILTER_DIRECTION;
        pTnsFilter->predictionGain  = FL2WORD16_SCALE(1.0f, PRED_GAIN_E);
        pTnsFilter->avgSqrCoef = 0;
        ClearTnsFilterCoefficients(pTnsFilter);
    }

}

void ClearTnsFilterCoefficients(STnsFilter * pTnsFilter)
{
    move16();
    pTnsFilter->order = 0;
    assert(TNS_MAX_FILTER_ORDER == 8);
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    pTnsFilter->coefIndex[0] = 0;
    pTnsFilter->coefIndex[1] = 0;
    pTnsFilter->coefIndex[2] = 0;
    pTnsFilter->coefIndex[3] = 0;
    pTnsFilter->coefIndex[4] = 0;
    pTnsFilter->coefIndex[5] = 0;
    pTnsFilter->coefIndex[6] = 0;
    pTnsFilter->coefIndex[7] = 0;
}

static void Index2Parcor(const Word16 index[], Word16 parCoeff[], Word16 order)
{
    Word16 const * values;
    Word16 i;

    move16();
    values = tnsCoeff4;

    FOR (i = 0; i < order; i++)
    {
        move16();
        parCoeff[i] = values[add(index[i], INDEX_SHIFT)];
    }

}



static Word32 FIRLattice(Word16 order, const Word16 *parCoeff /*Q15*/, Word32 *state, Word32 x /* Q0 */)
{
    Word16 i;
    Word32 tmpSave, tmp;


    tmpSave = L_add(x,0);
    FOR (i = 0; i < order-1; i++)
    {
        tmp = L_add(state[i], Mpy_32_16_1(x, parCoeff[i]));
        x = L_add(x, Mpy_32_16_1(state[i], parCoeff[i])); /* exponent: 31+0 */
        state[i] = tmpSave;
        move32();
        tmpSave = L_add(tmp,0);
    }

    /* last stage: only need half operations */
    x = L_add(x, Mpy_32_16_1(state[order-1], parCoeff[order-1]));
    state[order-1] = tmpSave;
    move32();

    return x;
}

static Word32 IIRLattice(Word16 order, const Word16 *parCoeff, Word32 *state, Word32 x)
{
    Word16 i;


    /* first stage: no need to calculate state[order-1] */
    x = L_sub(x, Mpy_32_16_1(state[order-1], parCoeff[order-1]));

    FOR (i = order-2; i >= 0; i--)
    {
        x = L_sub(x, Mpy_32_16_1(state[i], parCoeff[i]));
        state[i+1] = L_add(state[i], Mpy_32_16_1(x, parCoeff[i]));
        move32();
    }

    move32();
    state[0] = x;
    return x;
}

static void TnsFilter(Word32 const spectrum[], Word16 numOfLines,
                      Word16 const parCoeff[], Word16 order, Word16 direction,
                      TLinearPredictionFilter filter, Word32 * state,
                      Word32 output[])
{
    Word16 j;


    assert((order >= 0) && (order <= TNS_MAX_FILTER_ORDER));
    assert((numOfLines > 0) || ((numOfLines == 0) && (order == 0)));
    assert((direction == FILTER_DOWNWARDS) || (direction == FILTER_UPWARDS));


    IF (order == 0)
    {

        test();
        IF ( s_and((spectrum != output), numOfLines > 0) )
        {
            Copy32(spectrum, output, numOfLines);
        }
    }
    ELSE
    {
        IF ( sub(direction,FILTER_DOWNWARDS) == 0 )
        {

            FOR (j = sub(numOfLines,1); j >= 0; j--)
            {
                move32();
                output[j] = filter(order, parCoeff, state, spectrum[j]);
            }
        }
        ELSE /* direction == FILTER_UPWARDS */
        {

            FOR (j = 0; j < numOfLines; j++)
            {
                move32();
                output[j] = filter(order, parCoeff, state, spectrum[j]);
            }
        }
    }
}

static void ITF_TnsFilter_fx( Word32 const spectrum[],
                              const Word16 numOfLines,
                              const Word16 A[], /* ifdef FIX_ITF_OVERFLOW Q_A else Q14 */
                              const Word16 Q_A,
                              const Word16 order,
                              Word32 output[])
{
    Word16 i, j;
    Word32 buf[ITF_MAX_FILTER_ORDER + N_MAX];
    Word32* p;
    Word16 shift;
    assert((order >= 0) && (order <= ITF_MAX_FILTER_ORDER));
    assert((numOfLines > 0) || ((numOfLines == 0) && (order == 0)));

    IF (order == 0)
    {

        test();
        IF ( s_and((spectrum != output), numOfLines > 0) )
        {
            Copy32(spectrum, output, numOfLines);
        }
    }
    ELSE
    {
        shift = sub(15, Q_A);

        p = buf + ITF_MAX_FILTER_ORDER;
        set32_fx(buf, 0, ITF_MAX_FILTER_ORDER);
        Copy32(spectrum, p, numOfLines);
        FOR (j = 0; j < numOfLines; j++)
        {
            Word32 L_tmp;

            L_tmp = L_add(p[0], 0);
            FOR (i = 1; i < order; i++)
            {
                L_tmp = L_add(L_tmp, L_shl(Mpy_32_16_1(p[-i],  A[i]), shift));
            }
            output[j] = L_tmp;
            move32();
            ++p;
        }
    }

}

static void ITF_GetFilterParameters_fx(Word32 rxx[],
                                       Word16 maxOrder,
                                       Word16* A, /* ifdef FIX_ITF_OVERFLOW Q_A else Q14 */
                                       Word16* Q_A,
                                       Word16* predictionGain)
{
    Word16 i, j, i_2, tmp;
    Word16 parCoeff[ITF_MAX_FILTER_ORDER];
    Word32 epsP[ITF_MAX_FILTER_ORDER+1], L_tmp;

    /* compute filter in ParCor form with LeRoux-Gueguen algorithm */
    L_tmp = E_LPC_schur(rxx, parCoeff, epsP, maxOrder);
    BASOP_SATURATE_WARNING_OFF /* Allow saturation, this value is compared against a threshold. */
    *predictionGain = divide3232(L_shr(epsP[0], PRED_GAIN_E), L_tmp);
    BASOP_SATURATE_WARNING_ON

    {
        Word32 A32[ITF_MAX_FILTER_ORDER];
        Word16 tmp1_l, tmp1_h, tmp2_l, tmp2_h;

        /* Convert ParCor / reflection coefficients to LPC */
        A32[0] = FL2WORD32_SCALE(1.0, 31-11-16);
        move16(); /* Q11+16 */
        A32[1] = L_shr(L_deposit_h(parCoeff[0]), 4); /* Q11+16 */

        FOR (i=1; i<maxOrder; i++)
        {
            L_tmp =  L_shr(L_deposit_h(parCoeff[i]), 3); /* Q11+16 */

            i_2 = shr(i, 1);
            FOR(j=0; j<i_2; j++)
            {
                tmp1_l = L_Extract_lc(A32[i-1-j+1], &tmp1_h);
                tmp2_l = L_Extract_lc(A32[j+1],     &tmp2_h);
                A32[j+1]     = Mac_32(A32[j+1],     parCoeff[i], 0, tmp1_h, tmp1_l); /*+= parCoeff[i] * a[i-1-j+1];*/ move32();
                A32[i-1-j+1] = Mac_32(A32[i-1-j+1], parCoeff[i], 0, tmp2_h, tmp2_l); /*+= parCoeff[i] * tmp;*/ move32();
            }
            if (i & 1)
            {
                tmp2_l = L_Extract_lc(A32[j+1], &tmp2_h);
                A32[j+1]     = Mac_32(A32[j+1], parCoeff[i], 0, tmp2_h, tmp2_l);/*+= parCoeff[i] * a[j+1];*/ move32();
            }

            A32[i+1] = L_shr(L_deposit_h(parCoeff[i]), 4); /* Q11+16 */ move32();

        }

        tmp = 3;
        move16();/* assume Q11 -> Q14 */
        FOR (i=0; i<maxOrder; i++)
        {
            tmp = s_min(tmp, norm_l(A32[i]));
        }
        FOR (i=0; i<maxOrder; i++)
        {
            A[i] = round_fx(L_shl(A32[i], tmp)); /* Q11+tmp */
        }
        *Q_A = add(11, tmp);
    }
}

