/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#define _USE_MATH_DEFINES

#include <assert.h>
#include "stl.h"

#include "options.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "rom_basop_util.h"
#include "basop_util.h"


/***********************************************************************************/
/* forward declaration for local functions, see implementation at end of this file */
/***********************************************************************************/
static void calcPseudoSpec(Word32 * mdctSpec, Word16 mdctSpec_exp, Word16 nSamples, Word16 floorPowerSpectrum, Word32 * powerSpec, Word16 * powerSpec_exp);
static void getEnvelope(Word16 nSamples, Word32 const * powerSpec, Word16 F0, Word32 * envelope, Word32 * smoothedSpectrum);
static void GetF0(Word16 const nSamples,
                  Word16 const nSamplesCore,
                  Word32 const * const powerSpectrum, Word32 const pitchLag, Word16 * const pOrigF0, Word16 * const pF0);
static void findStrongestHarmonics(Word16 nSamples, Word32 const * powerSpectrum, Word16 F0, Word16 nTotalHarmonics, Word16 * pHarmonicIndexes, Word16 * pnHarmonics);
static void CorrectF0(Word16 const * pHarmonicIndexes, Word16 const nHarmonics, Word16 * pF0);
static void findCandidates(Word16 nSamples,
                           Word32 * MDCTSpectrum,                /* i: MDCT spectrum                        */
                           Word16 MDCTSpectrum_exp, Word16 * thresholdModificationNew
                           ,Word16 floorPowerSpectrum             /* i: lower limit for power spectrum bins  */
                          );
static void modifyThreshold(Word16 i, Word16 F0, Word16 threshold, Word16 * thresholdModification);
static void modifyThresholds(Word16 F0, Word16 origF0, Word16 * thresholdModification);
static void RefineThresholdsUsingPitch(Word16 nSamples,
                                       Word16 nSamplesCore,
                                       Word32 const powerSpectrum[],
                                       Word32 lastPitchLag,
                                       Word32 currentPitchLag,
                                       Word16 * pF0,
                                       Word16 * thresholdModification);

static void findTonalComponents(Word16 * indexOfTonalPeak, Word16 * lowerIndex, Word16 * upperIndex, Word16 *numIndexes, Word16 nSamples, const Word32 * powerSpectrum, Word16 F0, Word16 * thresholdModification);

/*******************************************************/
/*-------------- public functions -------------------- */
/*******************************************************/

/* Detect tonal components in the lastMDCTSpectrum, use
 * secondLastPowerSpectrum for the precise location of the peaks and
 * store them in indexOfTonalPeak.  Updates lowerIndex, upperIndex,
 * pNumIndexes accordingly. */
void  DetectTonalComponents(Word16 indexOfTonalPeak[],
                            Word16 lowerIndex[],
                            Word16 upperIndex[],
                            Word16 * pNumIndexes,
                            Word32 lastPitchLag, Word32 currentPitchLag,
                            Word16 const lastMDCTSpectrum[],
                            Word16 lastMDCTSpectrum_exp,
                            ApplyScaleFactorsPointer pApplyScaleFactors,
                            Word16 const scaleFactors[],
                            Word16 const scaleFactors_exp[],
                            Word16 const scaleFactors_max_e,
                            Word32 const secondLastPowerSpectrum[],
                            Word16 nSamples
                            ,Word16 nSamplesCore
                            ,Word16 floorPowerSpectrum        /* i: lower limit for power spectrum bins  */
                           )
{
    Word16 F0;
    Word16 thresholdModification[L_FRAME_MAX];
    Word32 pScaledMdctSpectrum[L_FRAME_MAX];


    pApplyScaleFactors(lastMDCTSpectrum, nSamplesCore, nSamples, scaleFactors, scaleFactors_exp, scaleFactors_max_e, pScaledMdctSpectrum);
    lastMDCTSpectrum_exp = add(lastMDCTSpectrum_exp, scaleFactors_max_e);

    /* Find peak candidates in the last frame. */
    findCandidates(nSamples, pScaledMdctSpectrum, lastMDCTSpectrum_exp, thresholdModification
                   , floorPowerSpectrum
                  );

    /* Refine peak candidates using the pitch information */
    RefineThresholdsUsingPitch(nSamples, nSamplesCore, secondLastPowerSpectrum, lastPitchLag, currentPitchLag, &F0, thresholdModification);

    /* Find peaks in the second last frame */
    findTonalComponents(indexOfTonalPeak, lowerIndex, upperIndex,  pNumIndexes, nSamples, secondLastPowerSpectrum, F0, thresholdModification);
}

/* When called, the tonal components are already stored in
 * indexOfTonalPeak.  Detect tonal components in the lastMDCTSpectrum,
 * use secondLastPowerSpectrum for the precise location of the peaks and
 * then keep in indexOfTonalPeak only the tonal components that are
 * again detected Updates indexOfTonalPeak, lowerIndex, upperIndex,
 * phaseDiff, phases, pNumIndexes accordingly. */
void RefineTonalComponents(Word16 indexOfTonalPeak[],
                           Word16 lowerIndex[],
                           Word16 upperIndex[],
                           Word16 phaseDiff[],
                           Word16 phases[], Word16 * pNumIndexes,
                           Word32 lastPitchLag, Word32 currentPitchLag,
                           Word16 const lastMDCTSpectrum[],
                           Word16 const lastMDCTSpectrum_exp,
                           ApplyScaleFactorsPointer pApplyScaleFactorsPointer,
                           Word16 const scaleFactors[],
                           Word16 const scaleFactors_exp[],
                           Word16 const scaleFactors_max_e,
                           Word32 const secondLastPowerSpectrum[],
                           Word16 nSamples
                           ,Word16 nSamplesCore
                           ,Word16 floorPowerSpectrum        /* i: lower limit for power spectrum bins  */
                          )
{
    Word16 newIndexOfTonalPeak[MAX_NUMBER_OF_IDX];
    Word16 newLowerIndex[MAX_NUMBER_OF_IDX];
    Word16 newUpperIndex[MAX_NUMBER_OF_IDX];
    Word16 newNumIndexes, nPreservedPeaks;
    Word16 iNew, iOld, j;
    Word16 * pOldPhase, * pNewPhase;


    DetectTonalComponents(newIndexOfTonalPeak, newLowerIndex, newUpperIndex, &newNumIndexes, lastPitchLag, currentPitchLag,
                          lastMDCTSpectrum, lastMDCTSpectrum_exp, pApplyScaleFactorsPointer, scaleFactors, scaleFactors_exp, scaleFactors_max_e, secondLastPowerSpectrum, nSamples
                          ,nSamplesCore
                          ,floorPowerSpectrum
                         );

    nPreservedPeaks = 0;
    move16();
    iNew = 0;
    move16();
    pOldPhase = phases;
    pNewPhase = phases;

    FOR (iOld = 0; iOld < *pNumIndexes; iOld++)
    {
        /* We don't want that the old peak index is at the border of the new peak region, that is why >= newUpperIndex and > newLowerIndex */
        test();
        WHILE (sub(iNew,newNumIndexes) < 0 && sub(indexOfTonalPeak[iOld],newUpperIndex[iNew]) >= 0)
        {
            iNew = add(iNew,1);
        }

        test();
        IF (sub(iNew,newNumIndexes) < 0 && sub(indexOfTonalPeak[iOld],newLowerIndex[iNew]) > 0)
        {
            newIndexOfTonalPeak[nPreservedPeaks] = indexOfTonalPeak[iOld];
            move16();
            newLowerIndex[nPreservedPeaks] = lowerIndex[iOld];
            move16();
            newUpperIndex[nPreservedPeaks] = upperIndex[iOld];
            move16();
            phaseDiff[nPreservedPeaks] = phaseDiff[iOld];
            move16();

            FOR (j = lowerIndex[iOld]; j <= upperIndex[iOld]; j++)
            {
                *pNewPhase++ = *pOldPhase++;
                move16();
            }
            nPreservedPeaks = add(nPreservedPeaks,1);
        }
        ELSE
        {
            pOldPhase += sub(upperIndex[iOld],add(lowerIndex[iOld],1));
        }
    }

    FOR (iNew = 0; iNew < nPreservedPeaks; iNew++)
    {
        indexOfTonalPeak[iNew] = newIndexOfTonalPeak[iNew];
        move16();
        lowerIndex[iNew] = newLowerIndex[iNew];
        move16();
        upperIndex[iNew] = newUpperIndex[iNew];
        move16();
    }
    *pNumIndexes = nPreservedPeaks;
    move16();

}

/*****************************************************
---------------- private functions -------------------
******************************************************/
static void calcPseudoSpec(Word32 * mdctSpec,                /* i: MDCT spectrum                        */
                           Word16 mdctSpec_exp,              /* i: exponent of MDCT spectrum            */
                           Word16 nSamples,                  /* i: frame size                           */
                           Word16 floorPowerSpectrum,        /* i: lower limit for power spectrum bins  */
                           Word32 * powerSpec,               /* o: estimated power spectrum             */
                           Word16 * powerSpec_exp)           /* o: exponent of estimated power spectrum */
{
    Word16 k;
    Word32 x, L_tmp, L_tmp_floor;
    Word16 tmp_loop;



    *powerSpec_exp = add(add(mdctSpec_exp, mdctSpec_exp), 5);
    move16();

    k = sub(31, *powerSpec_exp);
    /* If the signal is bellow floor, special care is needed for *powerSpec_exp */
    IF (sub(add(16-3, norm_s(floorPowerSpectrum)), k) < 0) /*extra 3 bits of headroom for MA filter in getEnvelope*/
    {
        k = add(16-3, norm_s(floorPowerSpectrum)); /*extra 3 bits of headroom for MA filter in getEnvelope*/
        L_tmp_floor = L_shl(L_deposit_l(floorPowerSpectrum), k);
        set32_fx(powerSpec, L_tmp_floor, nSamples);
        *powerSpec_exp = sub(31, k);
    }
    ELSE
    {
        L_tmp_floor = L_shl(L_deposit_l(floorPowerSpectrum), k);

        tmp_loop = sub(nSamples, 2);
        FOR (k = 1; k <= tmp_loop; k++)
        {
            x = L_sub(L_shr(mdctSpec[k+1], 1),L_shr(mdctSpec[k-1], 1)); /* An MDST estimate */

            x = L_shr(Mpy_32_32(x, x), 3);

            L_tmp = Mpy_32_32(mdctSpec[k], mdctSpec[k]);
            L_tmp = L_shr(L_tmp, 5);

            powerSpec[k] = L_max(L_tmp_floor, L_add(L_tmp, x));
            move32();
        }
    }

    powerSpec[0] = L_shr(powerSpec[1], 1);
    move32();
    powerSpec[nSamples-1] = L_shr(powerSpec[nSamples-2], 1);
    move32();

}

#define LEVEL_EXP 3/*+4*/
static void getEnvelope(Word16 nSamples,                       /*i:               Q0 */
                        Word32 const * powerSpec,              /*i: powerSpec_exp    */
                        Word16 F0,                             /*i:              5Q10*/
                        Word32 * envelope,                     /*o: powerSpec_exp + LEVEL_EXP*/
                        Word32 * smoothedSpectrum              /*o: powerSpec_exp + LEVEL_EXP*/
                       )
{
    Word16 nFilterLength, nHalfFilterLength, nSecondHalfFilterLength, n1, n2;
    Word16 level, inv_len;
    Word16 i;
    Word32 sum, tmp;

    IF (F0 == 0)
    {
        nFilterLength = 15;
        move16();
    }
    ELSE IF (F0 <= FL2WORD16_SCALE(10.0f,5))
    {
        nFilterLength = 11;
        move16();
    }
    ELSE IF (F0 >= FL2WORD16_SCALE(22.0f,5))
    {
        nFilterLength = 23;
        move16();
    }
    ELSE
    {
        nFilterLength = s_or(1, shr(F0,10)); /*1+2*(int)(F0/2); F0->Q10*/           move16();
    }

    nHalfFilterLength = shr(nFilterLength,1);

    n1 = add(nHalfFilterLength,1);

    nSecondHalfFilterLength = sub(nFilterLength,nHalfFilterLength);

    n2 = sub(nSecondHalfFilterLength,1);

    assert((nFilterLength >= 7) && (nFilterLength <= 23) && (nFilterLength %2 == 1));


    sum = L_deposit_l(0);
    level = FL2WORD16_SCALE(LEVEL_ABOVE_ENVELOPE,LEVEL_EXP); /*Q12*/

    FOR (i = 0; i < n2; i++)
    {
        sum = L_add(sum, powerSpec[i]);
    }
    /* No need for PTR_INIT for powerSpec[i+n2] as we continue from the previous loop */
    FOR (i = 0; i < n1; i++)
    {
        sum = L_add(sum, powerSpec[i+n2]);
        tmp = Mpy_32_16_1(sum/*Q31,powerSpec_exp*/,level/*Q12*/); /*Q28,powerSpec_exp*/
        envelope[i]/*Q28,powerSpec_exp*/ = Mpy_32_16_1(tmp/*Q28,powerSpec_exp*/, InvIntTable[i+nSecondHalfFilterLength]/*Q15*/);
        move32();
    }

    inv_len = mult_r(level, InvIntTable[nFilterLength]);
    FOR (i = n1; i < nSamples-n2; i++)
    {
        sum = L_add(sum, L_sub(powerSpec[i+n2],powerSpec[i-n1]));
        envelope[i] = Mpy_32_16_1(sum, inv_len);
        move32();
    }

    FOR (i = nSamples-n2; i < nSamples; i++)
    {
        sum = L_sub(sum, powerSpec[i-n1]);
        tmp = Mpy_32_16_1(sum,level);
        envelope[i] = Mpy_32_16_1(tmp, InvIntTable[nSamples-(i-nHalfFilterLength)]);
        move32();
    }

    FOR (i = 1; i < nSamples-1; i++)
    {
        smoothedSpectrum[i] = L_add(L_add(Mpy_32_16_1(powerSpec[i-1],FL2WORD16_SCALE(0.75f,LEVEL_EXP)),L_shr(powerSpec[i],LEVEL_EXP)), Mpy_32_16_1(powerSpec[i+1],FL2WORD16_SCALE(0.75f,LEVEL_EXP)));
    }

    move32();
    move32();
    smoothedSpectrum[0] = L_add(Mpy_32_16_1(powerSpec[1],FL2WORD16_SCALE(0.75f,LEVEL_EXP)),L_shr(powerSpec[0],LEVEL_EXP));
    smoothedSpectrum[nSamples-1] = L_add(Mpy_32_16_1(powerSpec[nSamples-2],FL2WORD16_SCALE(0.75f,LEVEL_EXP)),L_shr(powerSpec[nSamples-1],LEVEL_EXP));


}

static void GetF0(Word16 /*int*/    const           nSamples,       /*i   -  Q0 */
                  Word16 /*int*/    const           nSamplesCore,   /*i   -  Q0 */
                  Word32 /*float*/  const * const   powerSpectrum,  /*i   -  Qx */ /*is justed handed over and given back*/
                  Word32 /*float*/  const           pitchLag,       /*i   -  Q16*/
                  Word16 /*float*/        * const   pOrigF0,        /*o   -  Q10*/
                  Word16 /*float*/        * const   pF0)            /*o   -  Q10*/
{
    Word16 /*float*/ tmpPitchLag;
    Word16 /*int*/ rgiStrongHarmonics[MAX_PEAKS_FROM_PITCH];
    Word16 /*int*/ nTotalHarmonics, nStrongHarmonics;
    Word16 tmp;


    assert(LAST_HARMONIC_POS_TO_CHECK <= nSamplesCore);

    /* Use only F0 >= 100 Hz */
    test();
    IF ((pitchLag > 0) && (sub(round_fx(pitchLag) , shr(nSamplesCore,1)) <= 0))
    {

        tmpPitchLag /*"halfPitchLag" in FLC - read as Q5 for comparison to halfpitchlag */
            = round_fx(L_shl(pitchLag,4)); /*no division by 2, will be done in following division -
                                      furthermore, do a leftshift before rounding, to preserve more accuracy -
                                      will be accommodated also in following division*/

        /**pF0 = nSamplesCore/tmpPitchLag;*/
        BASOP_Util_Divide_MantExp(nSamplesCore,0,tmpPitchLag,-(1/*division by 2*/+4/*accommodate accuracy-prevention-leftshift*/),pF0,&tmp); /*pF0 is Q15*/
        move16();
        *pF0 = shr(*pF0,sub(5,tmp)); /*Q10 without scalingfactor*/
        move16();
        *pOrigF0 = *pF0; /*Q10*/
        move16();
        tmp = 2*LAST_HARMONIC_POS_TO_CHECK;
        if (sub(nSamples , 2*LAST_HARMONIC_POS_TO_CHECK) < 0 )
        {
            move16();
            tmp = nSamples;
        }
        BASOP_Util_Divide_MantExp(tmp,15,*pF0,5,&nTotalHarmonics,&tmp);
        nTotalHarmonics =shl(nTotalHarmonics,sub(tmp,15));


        /* Get in rgiStrongHarmonics all i for which i*F0 are the strongest harmonics */
        findStrongestHarmonics(nSamples, powerSpectrum, *pF0, nTotalHarmonics, rgiStrongHarmonics, &nStrongHarmonics);

        CorrectF0(rgiStrongHarmonics, nStrongHarmonics, pF0);
    }
    ELSE
    {
        move16();
        move16();
        *pF0 = 0;
        *pOrigF0 = 0;
    }

}


static void findStrongestHarmonics(Word16 nSamples,
                                   Word32 const * powerSpectrum,
                                   Word16 F0/*5Q10*/,
                                   Word16 nTotalHarmonics,
                                   Word16 * pHarmonicIndexes,
                                   Word16 * pnHarmonics)
{
    Word32 peaks[MAX_PEAKS_FROM_PITCH], smallestPeak;
    Word16 nPeaksToCheck, nPeaks, iSmallestPeak;
    Word16 i, l, k;
    (void)nSamples;


    nPeaks = 0;
    move16();

    iSmallestPeak = 0;
    move16();
    smallestPeak = 0x7fffffff;
    move32();

    nPeaksToCheck = s_min(nTotalHarmonics, MAX_PEAKS_FROM_PITCH+1);

    FOR (i = 1; i < nPeaksToCheck; i++)
    {
        Word32 newPeak;

        k = extract_h(L_shl(L_mult(i,F0),5));        /*k = (int)(i*F0);*/
        assert(k > 0 && k < 2*LAST_HARMONIC_POS_TO_CHECK && k < nSamples);

        newPeak = L_add(powerSpectrum[k], 0);

        peaks[nPeaks] = newPeak;
        move32();
        pHarmonicIndexes[nPeaks] = i;
        move16();

        IF (L_sub(newPeak,smallestPeak)  <= 0)
        {
            iSmallestPeak = nPeaks;
            move16();
            smallestPeak = L_add(newPeak, 0);
        }

        nPeaks = add(nPeaks,1);
    }

    FOR (; i < nTotalHarmonics; i++)
    {
        Word32 newPeak;

        k = extract_h(L_shl(L_mult(i,F0),5));
        assert(k > 0 && k < 2*LAST_HARMONIC_POS_TO_CHECK && k < nSamples);

        newPeak = L_add(powerSpectrum[k], 0);

        IF (L_sub(newPeak,smallestPeak) > 0)
        {
            peaks[iSmallestPeak] = newPeak;
            move32();
            pHarmonicIndexes[iSmallestPeak] = i;
            move16();
            smallestPeak = L_add(newPeak, 0);

            FOR (l = 0; l < MAX_PEAKS_FROM_PITCH; l++)
            {
                IF (peaks[l] <= smallestPeak)
                {
                    iSmallestPeak = l;
                    move16();
                    smallestPeak = L_add(peaks[l], 0);
                }
            }
        }
    }

    sort_fx(pHarmonicIndexes, 0, sub(nPeaks,1));

    *pnHarmonics = nPeaks;
    move16();

}

/* Use new F0, for which harmonics are most common in pHarmonicIndexes */
static void CorrectF0(Word16  /*int*/ const * pHarmonicIndexes, /*I    - Q0  */
                      Word16  /*int*/ const   nHarmonics,       /*I    - Q0  */
                      Word16  /*float*/ *     pF0)              /*I/O  - Q10 range: {0}, [4..18) */
{
    Word16 /*int*/   i;
    Word16 /*float*/ F0;
    Word16 /*int*/   diff[MAX_PEAKS_FROM_PITCH-1], sortedDiff[MAX_PEAKS_FROM_PITCH-1];
    Word16 /*int*/   iMostCommonDiff, nMostCommonDiff, nSameDiff, iMult;

    Word16 tmp;


    F0 = *pF0;

    test();
    IF (F0 > 0 && nHarmonics != 0)
    {
        tmp = sub(nHarmonics, 1);
        FOR (i = 0; i < tmp; i++)
        {
            diff[i] = sub(pHarmonicIndexes[i+1], pHarmonicIndexes[i]);
            move16();
            sortedDiff[i] = diff[i];
            move16();
        }
        sort_fx(sortedDiff, 0,sub(nHarmonics, 1+1));
        iMostCommonDiff = sortedDiff[0];
        move16();
        nSameDiff = 1;
        move16();
        i = 1;
        move16();
        IF (sub(imult1616(sortedDiff[0],pHarmonicIndexes[0]),1) == 0)
        {
            /* Find how many distances between peaks have length 1 */
            FOR (; i < tmp; i++)
            {
                if (sub(sortedDiff[i],1) == 0)
                {
                    nSameDiff=add(nSameDiff,1);
                }
            }
        }
        nMostCommonDiff = nSameDiff;
        move16();

        /* If there are at least 3 distances between peaks with length 1 and if the 1st harmonic is in pHarmonicIndexes then keep the original F0 */
        /* Otherwise find the most common distance between peaks */
        IF (sub(nSameDiff,3) < 0)
        {
            /* Find the most common difference */
            FOR (i = nSameDiff; i < tmp; i++)
            {
                IF (sub(sortedDiff[i], sortedDiff[i-1]) == 0 )
                {
                    nSameDiff=add(nSameDiff,1);
                }
                ELSE
                {
                    IF (sub(nSameDiff, nMostCommonDiff) > 0)
                    {
                        nMostCommonDiff = nSameDiff;
                        move16();
                        iMostCommonDiff = sortedDiff[i-1];
                        move16();
                    }
                    ELSE {
                        test();
                        IF (sub(nSameDiff, nMostCommonDiff)==0 && (abs_s(sub(iMostCommonDiff,pHarmonicIndexes[0])) > abs_s(sub(sortedDiff[i-1],pHarmonicIndexes[0]))))
                        {
                            nMostCommonDiff = nSameDiff;
                            move16();
                            iMostCommonDiff = sortedDiff[i-1];
                            move16();
                        }
                    }
                    nSameDiff = 1;
                    move16();
                }
            }
            IF (sub(nSameDiff,nMostCommonDiff) > 0)
            {
                nMostCommonDiff = nSameDiff;
                move16();
                iMostCommonDiff = sortedDiff[nHarmonics-2];
                move16();
            }
        }

        /* If there are enough peaks at the same distance */
        IF (sub(nMostCommonDiff, MAX_PEAKS_FROM_PITCH/2) >= 0)
        {
            iMult = 1;
            move16();
            FOR (i = 0; i < tmp; i++)
            {
                IF (sub(diff[i], iMostCommonDiff) == 0)
                {
                    iMult = pHarmonicIndexes[i];
                    move16();
                    BREAK;
                }
                test();
                test();
                IF (sub(sub(nHarmonics,2),i) > 0 && (sub(diff[i], diff[i+1]) == 0) && (sub(add(diff[i],diff[i+1]), iMostCommonDiff) == 0))
                {
                    iMult = pHarmonicIndexes[i];
                    move16();
                    BREAK;
                }
            }

            /* If the real F0 is much higher than the original F0 from the pitch */

            IF (sub(iMult, 3)  <= 0)
            {
                /* Use iMostCommonDiff, because the lowest pHarmonicIndexes[i] (which is equal to iMult) may not correspond to the new F0, but to it's multiple */
                F0 = round_fx(L_shl(L_mult(iMostCommonDiff /*Q0*/,F0 /*Q10*/),15));
            }
            ELSE
            {
                F0 = 0;
            }
        }
        /* Otherwise if there are at least 3 distances between peaks with length 1 and if the 1st harmonic is in pHarmonicIndexes then keep the original F0 */
        /* Otherwise don't use F0 */
        ELSE
        {
            test();
            if ((sub(iMostCommonDiff,1) > 0) || (sub(nMostCommonDiff,3) < 0))
            {
                /* Not enough peaks at the same distance => don't use the pitch. */
                F0 = 0;
                move16();
            }
        }
        *pF0 = F0;
        move16();
    }

}

static void modifyThreshold(Word16  /*int*/   i,                      /*I   - Q0 */
                            Word16  /*float*/ F0,                     /*I   - Q10*/
                            Word16  /*float*/ threshold,              /*I   - Q10*/
                            Word16* /*float*/ thresholdModification)  /*I/O - Q10*/
{
    Word32 harmonic;
    Word16 fractional /*Q15*/;
    Word16 k /*Q0*/;
    Word16 twoTimesFract /*Q10*/;



    harmonic = L_mult(shl(i,5),F0); /*Q0 * Q10 = 15Q16*/
    k = extract_h(harmonic); /*Q0*/
    fractional = lshr(extract_l(harmonic),1); /* Fractional part of the i*F0 */ /*Q15*/
    twoTimesFract = mult(2048/*2 in Q10*/,fractional/*Q15*/); /*Q10*/  /* threshold if the center of the peek is between k-1 and k, threshold+2 if the center of the peek is between k and k+1 */

    move16();
    thresholdModification[k] = threshold;
    move16();
    thresholdModification[k-1] = add(threshold/*Q10*/,                       twoTimesFract/*Q10*/); /*Q10*/
    move16();
    thresholdModification[k+1] = add(threshold/*Q10*/, sub(2048/*2 in Q10*/, twoTimesFract/*Q10*/)/*Q10*/); /*Q10*/

}

static void modifyThresholds(Word16  /*float*/ F0,                        /*I   - Q10*/
                             Word16  /*float*/ origF0,                    /*I   - Q10*/
                             Word16* /*float*/ thresholdModification)     /*I/O - Q10*/
{
    Word16 /*int*/ i, /*int*/ nHarmonics;
    Word16 tmp, tmpM, tmpE;




    IF (origF0 > 0)
    {
        IF (F0 == 0)
        {
            nHarmonics /*Q0*/ = s_min(MAX_PEAKS_FROM_PITCH /*Q0*/, shl((div_s(LAST_HARMONIC_POS_TO_CHECK /*Q0*/, origF0/*Q10*/) /*Q15*2^10*/),-5 /*Q0*2^-5*/) /*Q0*/);

            FOR (i = 1; i <= nHarmonics; i++)
            {
                modifyThreshold(i, origF0, FL2WORD16_SCALE(0.7f,5) /*0.7f in Q10*/, thresholdModification);
            }
        }
        IF (F0 > 0)
        {
            nHarmonics /*Q0*/ = s_min(MAX_PEAKS_FROM_PITCH /*Q0*/, shl((div_s(LAST_HARMONIC_POS_TO_CHECK /*Q0*/,     F0/*Q10*/) /*Q15*2^10*/),-5 /*Q0*2^-5*/) /*Q0*/);

            /*(int)(F0/origF0+0.5f)*/
            BASOP_Util_Divide_MantExp(F0,0,origF0,0,&tmpM,&tmpE);
            tmp=round_fx(L_shl(L_deposit_l(tmpM),add(tmpE,1)));

            FOR (i = tmp; i > 0; i--)
            {
                modifyThreshold(i, origF0, FL2WORD16_SCALE(0.35f,5), thresholdModification);
            }
            FOR (i = 1; i <= nHarmonics; i++)
            {
                modifyThreshold(i,     F0, FL2WORD16_SCALE(0.35f,5), thresholdModification);
            }
        }
    }

}

static void findCandidates(Word16 nSamples,                      /* i: frame size                           */
                           Word32 * MDCTSpectrum,                /* i: MDCT spectrum                        */
                           Word16 MDCTSpectrum_exp,              /* i: exponent of MDCT spectrum            */
                           Word16 * thresholdModificationNew     /* o: threshold modification Q10           */
                           ,Word16 floorPowerSpectrum             /* i: lower limit for power spectrum bins  */
                          )
{
    Word32 powerSpectrum[L_FRAME_MAX];
    Word16 powerSpectrum_exp;
    Word32 envelope[L_FRAME_MAX];
    Word32 smoothedSpectrum[L_FRAME_MAX];
    Word16 upperIdx, lowerIdx;
    Word16 k, j;
    Word32 biggerNeighbor;
    Word16 tmp_loop1, tmp_loop2, tmp_loop3;



    calcPseudoSpec(MDCTSpectrum, MDCTSpectrum_exp, nSamples, floorPowerSpectrum, powerSpectrum, &powerSpectrum_exp);

    getEnvelope(nSamples, powerSpectrum, 0, envelope, smoothedSpectrum);

    set16_fx(thresholdModificationNew, UNREACHABLE_THRESHOLD, nSamples);

    k = GROUP_LENGTH/2;
    move16();
    tmp_loop1 = sub(nSamples, (GROUP_LENGTH-GROUP_LENGTH/2));
    tmp_loop2 = sub(nSamples,1);
    WHILE ( sub(k, tmp_loop1) <=  0)
    {
        IF (L_sub(smoothedSpectrum[k],envelope[k]) > 0)
        {
            /* The check that bin at k is bigger than bins at k-1 and k+1 is needed to avoid deadlocks when the thresholds are low. */
            /* It removes some true peaks, especially if non weighted sum is used for the smoothed spectrum. */
            biggerNeighbor = L_max(powerSpectrum[k-1], powerSpectrum[k+1]);

            IF (L_sub(powerSpectrum[k], biggerNeighbor) >=  0)
            {
                /* Find the right foot */
                upperIdx = add(k, 1);
                WHILE ( sub(upperIdx,tmp_loop2)  < 0 )
                {

                    IF (L_sub(powerSpectrum[upperIdx],powerSpectrum[upperIdx+1]) < 0)
                    {
                        /* Side lobes may increase for certain amount */
                        IF (L_sub(   L_shl(Mpy_32_16_1(powerSpectrum[upperIdx], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP),    powerSpectrum[upperIdx+1] ) < 0 )
                        {
                            BREAK;
                        }
                        /* Check for further decrease after a side lobe increase */
                        FOR (j = add(upperIdx,1); j < tmp_loop2; j++)
                        {
                            IF (L_sub(  powerSpectrum[j],     L_shl(Mpy_32_16_1(powerSpectrum[j+1], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP)  ) < 0)
                            {
                                BREAK;
                            }
                        }
                        /* Side lobe increase must be 2 times smaller than the decrease to the foot */
                        /* Eq. to 2.0f*powerSpectrum[lowerIdx-1]/powerSpectrum[lowerIdx] > powerSpectrum[lowerIdx]/powerSpectrum[j] */
                        IF ( L_sub(  Mpy_32_32(L_shl(powerSpectrum[upperIdx+1],1),powerSpectrum[j]),     Mpy_32_32(powerSpectrum[upperIdx], powerSpectrum[upperIdx])  ) > 0 )
                        {
                            BREAK;
                        }
                        upperIdx = sub(j,1);
                    }
                    upperIdx = add(upperIdx, 1);
                }
                /* left foot */
                lowerIdx = sub(k,1);
                WHILE ( lowerIdx > 0 )
                {

                    IF (L_sub(powerSpectrum[lowerIdx], powerSpectrum[lowerIdx-1]) < 0)
                    {
                        /* Side lobes may increase for certain amount */
                        IF (L_sub(  L_shl(Mpy_32_16_1(powerSpectrum[lowerIdx], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP),   powerSpectrum[lowerIdx-1]) < 0 )
                        {
                            BREAK;
                        }
                        /* Check for further decrease after a side lobe increase */
                        FOR (j = sub(lowerIdx,1); j > 0; j--)
                        {
                            IF (L_sub (powerSpectrum[j],    L_shl(Mpy_32_16_1(powerSpectrum[j-1], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP) )  <  0)
                            {
                                BREAK;
                            }
                        }
                        /* Side lobe increase must be 2 times smaller than the decrease to the foot */
                        /* Eq. to 2.0f*powerSpectrum[lowerIdx-1]/powerSpectrum[lowerIdx] > powerSpectrum[lowerIdx]/powerSpectrum[j] */
                        IF (L_sub (   Mpy_32_32(L_shl(powerSpectrum[lowerIdx-1],1), powerSpectrum[j]),  Mpy_32_32(powerSpectrum[lowerIdx], powerSpectrum[lowerIdx]))  > 0 )
                        {
                            BREAK;
                        }
                        lowerIdx = add(j,1);
                    }
                    lowerIdx = sub(lowerIdx, 1);
                }

                /* Check if there is a bigger peak up to the next peak foot */
                tmp_loop3 = s_min(upperIdx, tmp_loop1);
                FOR (j = s_max(GROUP_LENGTH/2, lowerIdx); j <= tmp_loop3; j++)
                {
                    if (L_sub(powerSpectrum[j], powerSpectrum[k]) > 0)
                    {
                        k = j;
                        move16();
                    }
                }

                /* Modify thresholds for the following frame */
                tmp_loop3 = add(k,2);
                FOR (j = sub(k,1); j < tmp_loop3; j++)
                {
                    thresholdModificationNew[j] = BIG_THRESHOLD;
                    move16();

                    if (L_sub(smoothedSpectrum[j], envelope[j]) > 0)
                    {
                        thresholdModificationNew[j] = SMALL_THRESHOLD;
                        move16();
                    }

                }
                /* Jump to the next foot of the peak. */
                k = upperIdx;
                move16();
            }
        }
        k = add(k, 1);
    }

}

static void RefineThresholdsUsingPitch(Word16 nSamples,
                                       Word16 nSamplesCore,
                                       Word32 const powerSpectrum[],
                                       Word32 lastPitchLag,
                                       Word32 currentPitchLag,
                                       Word16 * pF0,
                                       Word16 * thresholdModification)
{
    Word16 pitchIsStable;
    Word16 origF0;
    Word32 L_tmp;

    /*pitchIsStable = (fabs(lastPitchLag-currentPitchLag) < 0.25f);*/
    pitchIsStable = 0;
    move16();
    L_tmp = L_abs(L_sub(lastPitchLag, currentPitchLag));
    if (L_sub(L_tmp, FL2WORD32_SCALE(0.25f, 15)) < 0)
    {
        pitchIsStable = 1;
        move16();
    }

    IF (pitchIsStable)
    {
        GetF0(nSamples,
              nSamplesCore,
              powerSpectrum, lastPitchLag, &origF0, pF0);

        modifyThresholds(*pF0, origF0, thresholdModification);
    }
    ELSE
    {
        *pF0 = 0;
        move16();
    }

}

static void findTonalComponents(Word16 * indexOfTonalPeak,       /* OUT */
                                Word16 * lowerIndex,             /* OUT */
                                Word16 * upperIndex,             /* OUT */
                                Word16 *numIndexes,              /* OUT */
                                Word16 nSamples,                 /* IN */
                                const Word32 * powerSpectrum,    /* IN */
                                Word16 F0,                       /* IN */
                                Word16 * thresholdModification)  /* IN */
{
    Word32 envelope[L_FRAME_MAX];
    Word32 smoothedSpectrum[L_FRAME_MAX];
    Word16 nrOfFIS;
    Word16 upperIdx, lowerIdx, lowerBound;
    Word16 k, j, m;
    Word32 biggerNeighbor;
    Word16 tmp_loop1, tmp_loop2, tmp_loop3;

    getEnvelope(nSamples, powerSpectrum, F0, envelope, smoothedSpectrum);


    nrOfFIS = 0;
    move16();
    lowerBound = 0;
    move16();

    k = GROUP_LENGTH/2;
    move16();
    tmp_loop1 = sub(nSamples, (GROUP_LENGTH-GROUP_LENGTH/2));
    tmp_loop2 = sub(nSamples,1);
    WHILE ( sub(k, tmp_loop1) <=  0)
    {
        /* There is 3 bits headroom in envelope and max of thresholdModification is 16384, so shifting left for 4 would produce overflow only when the result is anyhow close to 1 */
        IF (L_sub(L_shr(smoothedSpectrum[k], 1), L_shl(Mpy_32_16_1(envelope[k]/*Q28,powerSpec_exp*/, thresholdModification[k]/*Q10*/), 4)) > 0)
        {
            /* The check that bin at k is bigger than bins at k-1 and k+1 is needed to avoid deadlocks when the thresholds are low. */
            /* It removes some true peaks, especially if non weighted sum is used for the smoothed spectrum. */
            biggerNeighbor = L_max(powerSpectrum[k-1], powerSpectrum[k+1]);

            IF (L_sub(powerSpectrum[k], biggerNeighbor) >= 0 )
            {
                /* Find the right foot */
                upperIdx = add(k, 1);
                WHILE (sub(upperIdx, tmp_loop2) < 0)
                {
                    IF (L_sub(powerSpectrum[upperIdx], powerSpectrum[upperIdx+1]) < 0)
                    {
                        /* Side lobes may increase for certain amount */
                        IF (L_sub(  L_shl(Mpy_32_16_1(powerSpectrum[upperIdx], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP),  powerSpectrum[upperIdx+1]) < 0)
                        {
                            BREAK;
                        }
                        /* Check for further decrease after a side lobe increase */
                        FOR (j = add(upperIdx, 1); j < tmp_loop2; j++)
                        {
                            IF (L_sub( powerSpectrum[j], L_shl(Mpy_32_16_1(powerSpectrum[j+1], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP))  <  0)
                            {
                                BREAK;
                            }
                        }
                        /* Side lobe increase must be 2 times smaller than the decrease to the foot */
                        /* Eq. to 2.0f*powerSpectrum[lowerIdx-1]/powerSpectrum[lowerIdx] > powerSpectrum[lowerIdx]/powerSpectrum[j] */
                        IF (L_sub(  Mpy_32_32(L_shl(powerSpectrum[upperIdx+1], 1), powerSpectrum[j]),    Mpy_32_32(powerSpectrum[upperIdx], powerSpectrum[upperIdx])) > 0)
                        {
                            BREAK;
                        }
                        upperIdx = sub(j, 1);
                    }
                    upperIdx = add(upperIdx, 1);
                }
                /* left foot */
                lowerIdx = sub(k, 1);
                WHILE (sub(lowerIdx, lowerBound) > 0)
                {
                    IF (L_sub(powerSpectrum[lowerIdx], powerSpectrum[lowerIdx-1]) < 0)
                    {
                        /* Side lobes may increase for certain amount */
                        IF ( L_sub(L_shl(Mpy_32_16_1(powerSpectrum[lowerIdx], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP), powerSpectrum[lowerIdx-1]) < 0)
                        {
                            BREAK;
                        }
                        /* Check for further decrease after a side lobe increase */
                        FOR (j = sub(lowerIdx, 1); j > 0; j--)
                        {
                            IF (L_sub(powerSpectrum[j], L_shl(Mpy_32_16_1(powerSpectrum[j-1], ALLOWED_SIDE_LOBE_FLUCTUATION), ALLOWED_SIDE_LOBE_FLUCTUATION_EXP)) < 0)
                            {
                                BREAK;
                            }
                        }
                        /* Side lobe increase must be 2 times smaller than the decrease to the foot */
                        /* Eq. to 2.0f*powerSpectrum[lowerIdx-1]/powerSpectrum[lowerIdx] > powerSpectrum[lowerIdx]/powerSpectrum[j] */
                        IF ( L_sub( Mpy_32_32(L_shl(powerSpectrum[lowerIdx-1], 1), powerSpectrum[j]), Mpy_32_32(powerSpectrum[lowerIdx], powerSpectrum[lowerIdx])) >  0)
                        {
                            BREAK;
                        }
                        lowerIdx = add(j, 1);
                    }
                    lowerIdx = sub(lowerIdx, 1);
                }

                lowerBound = upperIdx;
                move16();

                /* Check if there is a bigger peak up to the next peak foot */
                tmp_loop3 = s_min(upperIdx, tmp_loop1);
                FOR (j = s_max(GROUP_LENGTH/2, lowerIdx); j <= tmp_loop3; j++)
                {
                    if (L_sub(powerSpectrum[j],powerSpectrum[k]) > 0)
                    {

                        k = j;
                        move16();
                    }
                }

                assert((nrOfFIS == 0) || (indexOfTonalPeak[nrOfFIS-1] < k));

                lowerIndex[nrOfFIS] = sub(k, GROUP_LENGTH/2);
                move16();

                upperIndex[nrOfFIS] = add(k,(GROUP_LENGTH-GROUP_LENGTH/2-1));
                move16();

                test();
                IF ((nrOfFIS > 0) && (sub(lowerIndex[nrOfFIS], upperIndex[nrOfFIS-1]) <=  0))
                {
                    m = shr(add(k, indexOfTonalPeak[nrOfFIS-1]), 1);
                    upperIndex[nrOfFIS-1] = m;
                    move16();
                    lowerIndex[nrOfFIS] = add(m, 1);
                    move16();
                }

                indexOfTonalPeak[nrOfFIS++] = k;
                move16();

                IF (sub(nrOfFIS, MAX_NUMBER_OF_IDX) == 0 )
                {
                    BREAK;
                }
                /* Jump to the next foot of the peak. */
                k = upperIdx;
                move16();
            }
        }
        k = add(k, 1);
    }

    *numIndexes = nrOfFIS;
    move16();

}

