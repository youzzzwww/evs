/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef __dct_H
#define __dct_H

#include "stl.h"


/**
 * \brief Calculate DCT type IV of given length. The DCT IV is
 *        calculated by a complex FFT, with some pre and post twiddeling.
 *        A factor of sqrt(2/N) is NOT applied.
 * \param pDat pointer to input/output data (in place processing).
 * \param pDat_e pointer to the exponent of pDat
 * \param size size of pDat.
 */
void dct_IV(Word32 *pDat, Word16*pDat_e, Word16 size);


/**
 * \brief Calculate DST type IV of given length. The DST IV is
 *        calculated by a complex FFT, with some pre and post twiddeling.
 *        A factor of sqrt(2/N) is NOT applied.
 * \param pDat pointer to input/output data (in place processing).
 * \param pDat_e pointer to the exponent of pDat
 * \param size size of pDat.
 */
void dst_IV(Word32 *pDat, Word16*pDat_e, Word16 size);

#endif
