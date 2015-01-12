/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stl.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "prot_fx.h"

void UnmapIndex(
    Word16 PeriodicityIndex,
    Word16  Bandwidth,
    Word16 LtpPitchLag,
    Word8  SmallerLags,
    Word16 *FractionalResolution,
    Word32 *Lag)
{
    Word16 LtpPitchIndex, Multiplier;
    Word16 Lag16;

    test();
    IF ((LtpPitchLag > 0) && (s_and(PeriodicityIndex, kLtpHmFlag) != 0))
    {
        LtpPitchIndex = shr(PeriodicityIndex, 9);
        Multiplier = s_and(PeriodicityIndex, 0xff);

        assert(0 <= LtpPitchIndex && LtpPitchIndex <= 16);
        assert(1 <= Multiplier && Multiplier <= (1 << NumRatioBits[Bandwidth][LtpPitchIndex]));

        *FractionalResolution = kLtpHmFractionalResolution;
        move16();
        *Lag = L_shr(L_mult0(LtpPitchLag, Ratios[Bandwidth][LtpPitchIndex][Multiplier-1]), 8);
        move32();
    }
    ELSE
    {
        IF (sub(PeriodicityIndex, 16) < 0)
        {
            *FractionalResolution = 3;
            move16();
            Lag16 = add(PeriodicityIndex, GET_ADJ2(0, 6, 3));
        }
        ELSE IF (sub(PeriodicityIndex, 80) < 0)
        {
            *FractionalResolution = 4;
            move16();
            Lag16 = add(PeriodicityIndex, GET_ADJ2(16, 8, 4));
        }
        ELSE IF (sub(PeriodicityIndex, 208) < 0)
        {
            *FractionalResolution = 3;
            move16();
            Lag16 = add(PeriodicityIndex, GET_ADJ2(80, 12, 3));
        }
        ELSE {
            test();
            IF (sub(PeriodicityIndex, 224) < 0 || SmallerLags != 0)
            {
                *FractionalResolution = 1;
                move16();
                Lag16 = add(PeriodicityIndex, GET_ADJ2(208, 28, 1));
            }
            ELSE {
                *FractionalResolution = 0;
                move16();
                Lag16 = add(PeriodicityIndex, GET_ADJ2(224, 188, 0));
            }
        }
        *Lag = L_deposit_l(Lag16);
    }
}


void ConfigureContextHm(
    Word16 NumCoeffs,                   /* (I) Number of coefficients                         */
    Word16 TargetBits,                  /* (I) Target bit budget (excl. Done flag)            */
    Word16 PeriodicityIndex,            /* (I) Pitch related index                            */
    Word16 LtpPitchLag,                 /* (I) TCX-LTP pitch in F.D.                          */
    CONTEXT_HM_CONFIG *hm_cfg           /* (O) Context-based harmonic model configuration     */
)
{
    Word8   Bandwidth, SmallerLags;
    Word32  i, Limit, Lag;
    Word16  j, Index, FractionalResolution;
    Word16  *tmp;


    Bandwidth = 0;
    move16();
    if (sub(NumCoeffs, 256) >= 0)
    {
        Bandwidth = 1;
        move16();
    }

    SmallerLags = 0;
    move16();
    test();
    if ((sub(TargetBits, kSmallerLagsTargetBitsThreshold) <= 0) || (Bandwidth == 0))
    {
        SmallerLags = 1;
        move16();
    }

    UnmapIndex(PeriodicityIndex,
               Bandwidth,
               LtpPitchLag,
               SmallerLags,
               &FractionalResolution, &Lag);

    /* Set up and fill peakIndices */
    hm_cfg->peakIndices = hm_cfg->indexBuffer;
    tmp = hm_cfg->peakIndices;
    Limit = L_shl(L_deposit_l(sub(NumCoeffs, 1)), FractionalResolution);
    IF (L_sub(Lag, Limit) < 0)
    {
        FOR (i=Lag; i<Limit; i+=Lag)
        {
            Index = extract_l(L_shr(i, FractionalResolution));
            *tmp++ = sub(Index, 1);
            move16();
            *tmp++ = Index;
            move16();
            *tmp++ = add(Index, 1);
            move16();
        }
    }
    hm_cfg->numPeakIndices = (Word16)(tmp - hm_cfg->indexBuffer);

    /* Set up and fill holeIndices */
    hm_cfg->holeIndices = hm_cfg->indexBuffer + hm_cfg->numPeakIndices;
    tmp = hm_cfg->holeIndices;
    Index = 0;
    move16();
    IF (hm_cfg->numPeakIndices > 0)
    {
        FOR (j=0; j<hm_cfg->numPeakIndices; j+=3)
        {
            FOR (; Index<hm_cfg->peakIndices[j]; ++Index)
            {
                *tmp++ = Index;
                move16();
            }
            Index = add(Index, 3); /* Skip the peak */
        }
    }
    IF (sub(Index, NumCoeffs) < 0)
    {
        FOR (; Index<NumCoeffs; ++Index)
        {
            *tmp++ = Index;
            move16();
        }
    }
    hm_cfg->numHoleIndices = (Word16)(tmp - hm_cfg->holeIndices);
    *tmp++ = NumCoeffs;
    move16(); /* Add extremal element signaling the end of the buffer */

}

Word16 CountIndexBits(
    Word16 Bandwidth,
    Word16 PeriodicityIndex)
{
    Word16 result;
    Word16 PeriodicityIndexS;

    result = 8;
    move16();
    PeriodicityIndexS = shr(PeriodicityIndex, 9);

    if (s_and(PeriodicityIndex, kLtpHmFlag) != 0)
    {
        result = NumRatioBits[Bandwidth][PeriodicityIndexS];
        move16();
    }

    return result;
}



void tcx_hm_render(
    Word32 lag,           /* i: pitch lag                         Q0  */
    Word16 fract_res,     /* i: fractional resolution of the lag  Q0  */
    Word16 p[]            /* o: harmonic model                    Q13 */
)
{
    Word16 k, tmp, height;
    Word16 PeakDeviation;
    Word32 f0, tmp32;

    /* Set up overall shape */
    f0 = L_shl(lag, sub(15, fract_res)); /* Q31 */

    tmp32 = Mpy_32_16_1(f0, -26474);
    tmp32 = L_shr_r(BASOP_Util_InvLog2(L_shl(tmp32, 7)), 2);
    tmp32 = L_sub(603979776L, tmp32);
    tmp32 = L_add(L_add(tmp32, tmp32), Mpy_32_16_1(tmp32, 26214));
    height = round_fx(tmp32); /* Q13 */

    tmp32 = Mpy_32_16_1(f0, -18910);
    tmp32 = L_shr_r(BASOP_Util_InvLog2(L_shl(tmp32, 7)), 2);
    tmp32 = L_sub(1395864371L, tmp32);
    PeakDeviation = round_fx(tmp32); /* Q14 */

    tmp = div_s(13915, PeakDeviation);
    tmp = mult_r(tmp, tmp); /* Q15 */

    /* Render the prototype peak */
    p[kTcxHmParabolaHalfWidth] = height;
    move16();

    FOR (k=1; k<=kTcxHmParabolaHalfWidth; ++k)
    {
        p[kTcxHmParabolaHalfWidth+k] = round_fx(Mpy_32_16_1(BASOP_Util_InvLog2(L_shl(L_mult0(i_mult2(negate(k),k), tmp),10)), height));
    }
    /* Mirror */
    FOR (k=-kTcxHmParabolaHalfWidth; k<0; ++k)
    {
        p[kTcxHmParabolaHalfWidth+k] = p[kTcxHmParabolaHalfWidth-k];
        move16();
    }

}

void tcx_hm_modify_envelope(
    Word16 gain,          /* i:   HM gain                           Q11 */
    Word32 lag,           /* i:   pitch lag                         Q0  */
    Word16 fract_res,     /* i:   fractional resolution of the lag  Q0  */
    Word16 p[],           /* i:   harmonic model                    Q13 */
    Word32 env[],         /* i/o: envelope                          Q16 */
    Word16 L_frame        /* i:   number of spectral lines          Q0  */
)
{
    Word16 k, h, x, l1,l2, L_frame_m1, L_frame_for_loop;
    Word16 inv_shape[2*kTcxHmParabolaHalfWidth+1];

    IF ( gain == 0 )
    {
        return;
    }

    FOR (k=0; k<2*kTcxHmParabolaHalfWidth+1; ++k)
    {
        /* Q24 = Q11 * Q13; 512 = 1.0 in Q24 format */
        inv_shape[k] = div_s(512, add(512, mult_r(gain, p[k])));
        move16();
    }

    h = 1;
    move16();
    k = extract_l(L_shr(lag,fract_res));

    L_frame_m1 = sub(L_frame,1);
    L_frame_for_loop = add(L_frame,kTcxHmParabolaHalfWidth - 1);

    WHILE ( sub(k,L_frame_for_loop) <= 0 )
    {
        l1 = s_max(0, sub(k,kTcxHmParabolaHalfWidth));
        l2 = s_min(add(k,kTcxHmParabolaHalfWidth), L_frame_m1);
        FOR (x=l1; x<=l2; ++x)
        {
            env[x] = Mpy_32_16_1(env[x], inv_shape[x-k+kTcxHmParabolaHalfWidth]);
            move32();
        }

        h = add(h,1);
        k = extract_l(L_shr(imult3216(lag,h),fract_res));
    }

}


