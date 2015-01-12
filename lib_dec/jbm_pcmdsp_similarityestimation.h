/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_similarityestimation.h Algorithms for correlation and similarity estimation. */

#ifndef PCMDSP_SIMILARITYESTIMATION_H
#define PCMDSP_SIMILARITYESTIMATION_H PCMDSP_SIMILARITYESTIMATION_H

/* local headers */
#include "stl.h"

/*! Returns the number of right shifts to be applied to the signal before correlation functions. */
Word16 getSignalScaleForCorrelation(Word32 sampleRate);

/*! Copies the right shifted signal to another buffer. */
void scaleSignal16(const Word16 *src, Word16 *dst, Word16 n, Word16 rightShift);

/*
********************************************************************************
*
*     Function        : cross_correlation_self
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : (float) cross correlation coefficient
*     Information     : Calculate cross correlation coefficient for template
*                       segment.
*                       The returned value is signal-energy dependant.
*
*                       Used formula:
*
*                       corr_len-1
*                       ----
*                       \
*                       /    (j+x)*(j+y)
*                       ----
*                       j=0
*
********************************************************************************
*/
Word32 cross_correlation_self(const Word16 * signal,
                              Word16 x,
                              Word16 y,
                              Word16 corr_len);

/*
********************************************************************************
*
*     Function        : cross_correlation_subsampled_self
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : (float) cross correlation coefficient
*     Information     : Calculate cross correlation coefficient for template
*                       segment.
*                       The returned value is signal-energy dependant.
*
*                       Used formula:
*
*                       corr_len-1
*                       ----
*                       \
*                       /    (j+x)*(j+y)
*                       ----
*                       j=0
*
********************************************************************************
*/
Word32 cross_correlation_subsampled_self(const Word16 * signal,
        Word16 x,
        Word16 y,
        Word16 corr_len,
        Word16 subsampling);

/*
********************************************************************************
*
*     Function        : normalized_cross_correlation_self
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : (float) normalized cross correlation coefficient
*     Information     : Calculate normalized cross correlation coefficient
*                       for template segment.
*                       The returned value is signal-energy independant.
*                       This means, no matter how loud your signal is, equal
*                       signals will return 1.0, cross-phased signals -1.0.
*
*                       This function fills parameter energy with the common
*                       energy of signal x and signal y. This might be useful
*                       for silence detection.
*
*                       Used formula:
*
*                       corr_len-1
*                       ----
*                       \         (j+x)*(j+y)
*                        \    __________________
*                        /      --------------
*                       /     -/ (j+x)���+(j+y)���
*                       ----
*                       j=0
*
********************************************************************************
*/
Word16 normalized_cross_correlation_self(const Word16 * signal,
        Word16 x,
        Word16 y,
        Word16 corr_len,
        Word16 subsampling,
        Word32 * energy);

/* Splits the signal into segments and checks if all of them have very low energy. */
Word8 isSilence(const Word16 * signal, Word16 len, Word16 segments);

#endif /* PCMDSP_SIMILARITYESTIMATION_H */
