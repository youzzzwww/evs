/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_window.c Window functions. */

/* instrumentation headers */
/* local headers */
#include "jbm_pcmdsp_window.h"
#include "options.h"
#include "stl.h"


/* Overlap/Add of two signal with a given window. */
void overlapAdd(const Word16 *fadeOut, const Word16 *fadeIn, Word16 *out,
                Word16 n, Word16 nChannels, const Word16 *fadeOutWin, const Word16 *fadeInWin, Word16 hannIncrementor )
{
    Word32 fdOutVal, fdInVal;
    Word16 i, j, hannIter, combinedVal;


    FOR(j = 0; j < nChannels; j++)
    {
        /* reset Hann window iterator to beginning (both channels use same window) */
        hannIter = 0;
        move16();
        FOR( i = j; i < n; i += nChannels )
        {
            fdOutVal = L_mult( fadeOut[i], fadeOutWin[hannIter] );
            fdInVal  = L_mult( fadeIn[i],  fadeInWin[hannIter] );
            /* round to 16bit value and saturate (L_add already applies saturation) */
            combinedVal = round_fx( L_add( fdOutVal, fdInVal ) );

            out[i] = combinedVal;
            move16();
            /* advance the Hann window iterator by incrementor (dependent on sample rate). */
            hannIter = add( hannIter, hannIncrementor );
        }
    }
}

