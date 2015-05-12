/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_window.h Window functions. */

#ifndef PCMDSP_WINDOW_H
#define PCMDSP_WINDOW_H PCMDSP_WINDOW_H

/* instrumentation headers */
#include "basop_util.h"

/*! Tables contain a Hann window (cos-shaped) of length 960 or 640.
 *  Roughly:
 *
 *                       1    __
 *                           /  \
 *                       0 _/    \_
 *                         <------>
 *                            n
 */
extern Word16 pcmdsp_window_hann_960[960];
extern Word16 pcmdsp_window_hann_640[640];
extern Word16 pcmdsp_window_hann_1280[1280];

/** Overlap/Add of two signal with a given window. */
/** @param[in] fadeOut signal to fade out
 *  @param[in] fadeIn signal to fade in
 *  @param[in] out buffer to store the output signal
 *  @param[in] n number of samples
 *  @param[in] nChannels number of channels
 *  @param[in] fadeOutWin window for fade out
 *  @param[in] fadeInWin window for fade in */
void overlapAdd(const Word16 *fadeOut, const Word16 *fadeIn, Word16 *out,
                Word16 n, Word16 nChannels, const Word16 *fadeOutWin, const Word16 *fadeInWin, Word16 hannIncrementor);

#endif /* PCMDSP_WINDOW_H */
