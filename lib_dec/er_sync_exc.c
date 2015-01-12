/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*This BASOP port is up to date with trunk rev. 36554*/

#include "cnst_fx.h"
#include "stl.h"
#include "prot_fx.h"
#include <stdlib.h>
#include <limits.h>
#include "basop32.h"
#include "basop_util.h"

/*-------------------------------------------------------------------------
*
* Perform resynchronisation of the last glottal pulse in voiced lost frame
*
*------------------------------------------------------------------------*/


/** Get the location of the minimum energy in the given signal.
  * @returns Index of the position of the minimum energy, that is the position i where filter(x[i-filterLength/2],...,x[i+(filterLength-filterLength/2)-1]) is at maximum.
  */
static Word16 GetMinimumPosition(
    Word16 const * x,   /* Input signal.                                                            Qx*/
    Word16 length,      /* The length of the input signal.                                          Q0*/
    Word16 filterLength /* the length of the filter length used for the energy calculation.         Q0*/
)
{
    Word16 iMinEnergyPos, center, i;
    Word16 cnt, tmp_e,  tmp16;
    Word32 energy, energy_old, tmptest;




    filterLength = s_min(filterLength, length);
    center = shr(filterLength,1);
    iMinEnergyPos = center;
    move16();

    IF (filterLength > 0)
    {
        center = add(center,1);
        energy = L_deposit_l(0);
        energy_old = L_deposit_l(0);

        cnt = sub(length,filterLength);

        tmp_e = 0;
        move16();
        FOR (i = 0; i < cnt; i++)
        {
            tmp16 = shr(x[i],tmp_e);
            energy = L_msu(energy_old,tmp16,tmp16);
            tmp16 = shr(x[i+filterLength],tmp_e);
            BASOP_SATURATE_WARNING_OFF /*Saturation will be handled*/
            energy = L_mac(energy,tmp16,tmp16);
            BASOP_SATURATE_WARNING_ON

            /*if (energy == MAXVAL_WORD32)*/
            BASOP_SATURATE_WARNING_OFF /*saturates if energy < 0*/
            tmptest = L_sub(energy,MAXVAL_WORD32);
            BASOP_SATURATE_WARNING_ON
            IF (tmptest == 0)
            {
                tmp_e = add(tmp_e,1);
                energy = L_shr(energy_old,2);
                tmp16 = shr(x[i],tmp_e);
                energy = L_msu(energy,tmp16,tmp16);
                tmp16 = shr(x[i+filterLength],tmp_e);
                energy = L_mac(energy,tmp16,tmp16);
            }

            IF (energy < 0)
            {
                iMinEnergyPos = add(i,center);
                tmp_e = 0;
                move16();
                energy = 0;
                move16();
            }
            energy_old = L_add(energy, 0);
        }

    }

    return iMinEnergyPos;
}

/*!
  * \brief FindMaxPeak - Get the location of the maximum peak in the given signal.
  *
  * \returns Index of the position of the maximum peak, that is the position i where abs(x[i]) has it's maximum.
  */

static Word16 FindMaxPeak
(
    Word16 /*float*/ const * x,     /*<!i   : Input signal.*/
    Word16 /*int*/ length           /*<!i   : The length of the input signal.*/
)
{
    Word16 iMax, i;



    iMax = 0;
    move16();
    FOR (i = 1; i < length; i++)
    {
        if (sub(abs_s(x[i]) , abs_s(x[iMax]))>0)
        {
            move16();
            iMax = i;
        }
    }

    return iMax;
}

static void AddSamples(
    Word16 const * const    old_exc,            /*Qx*/
    Word16       * const    new_exc,            /*Qx*/
    Word16 const            L_frame,            /*Q0*/
    Word16 const            n_samples_to_add,   /*Q0*/
    Word16 const            min_pos[],          /*Q0*/
    Word16 const            points_by_pos[],    /*Q0*/
    Word16 const            nb_min              /*Q0*/
)
{
    Word16 * pt_dest;
    Word16 const * pt_src;
    Word16 last_min_pos, i, j;
    Word16 ftmp;



    pt_dest = new_exc;
    move16();
    pt_src = old_exc;
    move16();
    last_min_pos = 0;
    move16();

    FOR (i = 0; i < nb_min; i++)
    {
        /* Copy section */
        FOR (j = min_pos[i] - last_min_pos; j > 0; j--)
        {
            *pt_dest++ = *pt_src++;
            move16();
        }
        /* Add some samples */
        ftmp = negate(mult(*pt_src,FL2WORD16(.05f)));

        FOR (j = 0; j < points_by_pos[i];  j++)
        {
            *pt_dest++ = ftmp;
            move16();
            ftmp = negate(ftmp);
        }
        /* Prepare for the next loop iteration */
        last_min_pos = min_pos[i];
        move16();
    }
    /* Copy remaining length */
    FOR (j = sub(L_frame , add(n_samples_to_add , last_min_pos)); j > 0; j--)
    {
        *pt_dest++ = *pt_src++;
        move16();
    }

}

static void RemoveSamples(
    Word16 /*float*/const   * const old_exc,            /*i   : Qx */
    Word16 /*float*/        * const new_exc,            /*o   : Qx */
    Word16 /*int*/ const            L_frame,            /*i   : Q0 */
    Word16 /*int*/ const            n_samples_to_add,   /*i   : Q0 */
    Word16 /*int*/ const            min_pos[],          /*i   : Q0 */
    Word16 /*int*/ const            points_by_pos[],    /*i   : Q0 */
    Word16 /*int*/ const            nb_min              /*i   : Q0*/
)
{
    Word16 /*float*/ * pt_dest;
    Word16 /*float*/ const * pt_src;
    Word16 /*int*/ last_min_pos, i, j;



    pt_dest = new_exc+L_frame;
    last_min_pos = sub(L_frame,n_samples_to_add);

    FOR(i = sub(nb_min,1); i >= 0; i--)
    {
        /* Compute len to copy */
        /* Copy section, removing some samples */
        pt_src = old_exc+last_min_pos;

        FOR (j = sub(last_min_pos , add(min_pos[i],points_by_pos[i])); j > 0; j--)
        {
            *--pt_dest = *--pt_src;
            move16();
        }
        /* Prepare for the next loop iteration */
        last_min_pos = min_pos[i];
        move16();
    }
    /* Copy remaining length */
    pt_src = old_exc+last_min_pos;

    FOR (j = last_min_pos; j > 0; j--)
    {
        *--pt_dest = *--pt_src;
        move16();
    }

}

/** Resynchronize glotal pulse positions of the signal in src_exc and store it in dst_exc.
  * src_exc holds on call the harmonic part of the signal with the constant pitch, constructed by repeating the last pitch cycle of length pitchStart.
  * dst_exc holds on return the harmonic part of the signal with the pitch changing from pitchStart to pitchEnd.
  * src_exc and dst_exc can overlap, but src_exc < dst_exc must be fullfiled.
  */
/*This BASOP port is up to date with trunk rev 8779(svnext2)*/
void PulseResynchronization(
    Word16 /*float*/    const   *   const   src_exc,        /*<!i  : Input excitation buffer                                                                        Q15*/
    Word16 /*float*/            *   const   dst_exc,        /*<!o  : Output excitation buffer                                                                       Q15*/
    Word16 /*int*/      const               nFrameLength,   /*<!i  : Length of the frame, that is the length of the valid data in the excitation buffer on return.  Q0 */
    Word16 /*int*/      const               nSubframes,     /*<!i  : Number of subframes in the excitation buffer. nFrameLength must be divisible by nSubframes     Q0 */
    Word32 /*float*/    const               pitchStart,     /*<!i  : Pitch at the end of the last frame                                                             Q16*/
    Word32 /*float*/    const               pitchEnd        /*<!i  : Pitch at the end of the current frame                                                          Q16*/
)
{
    Word16 /*int*/    T0, i, k;
    Word16 /*float*/  perCycleDeltaDelta, cycleDelta, freqStart, fractionalLeft;
    Word16 /*int*/    roundedPitchStart, nSamplesDelta, nSamplesDeltaRemain, iMinPos1, iMinPos[NB_PULSES_MAX+1], iDeltaSamples[NB_PULSES_MAX+1], maxDeltaSamples, roundedCycleDelta;
    Word16 tmp16, tmp16_a, freqStart_e/*exponent of freqStart*/, tmp_e, samplesDelta_e, perCycleDeltaDelta_e,cycleDelta_e , tmp2_e, tmp3_e;
    Word32 /* pitchDelta, */ tmp32, tmp32_a, tmp32_b, samplesDelta, absPitchDiff, cycleDelta32;



    test();
    test();
    test();
    test();
    test();
    test();
    IF ((L_sub(L_deposit_h(nFrameLength),pitchStart) < 0)
        || (pitchStart <= 0) ||  (pitchEnd <= 0)
        || (sub(nSubframes, 1) < 0)
        || (sub(nSubframes, 5) > 0)
        || (L_sub(Mpy_32_16_1(pitchEnd,add(nSubframes,1)),Mpy_32_16_1(pitchStart,sub(nSubframes,1))) <= 0)
        || (src_exc-dst_exc >= 0))
    {
        /* This is error handling and recovery that should never occur. */
        test();
        IF (src_exc != dst_exc && sub(nFrameLength, 1200) <= 0)
        {
            Copy(src_exc, dst_exc, nFrameLength);
        }
        return;
    }

    roundedPitchStart = round_fx(pitchStart);     /*Q0*/

    /* freqStart = 1.0f/roundedPitchStart; */
    freqStart_e = 15;
    move16();
    freqStart = Inv16(roundedPitchStart, &freqStart_e); /*Q15,freqStart_e*/

    /* Calculate number of samples to be removed (if negative) or added (if positive) */
    /*samplesDelta = 0.5f*pitchDelta*nFrameLength*(nSubframes+1)*freqStart;*/
    /* pitchDelta*freqStart = ((pitchEnd - pitchStart)/roundedPitchStart)/nSubframes  */
    tmp16 = shl(roundedPitchStart, 2); /*Q0*/
    if (sub(nSubframes, 5) == 0)
    {
        tmp16 = add(tmp16, roundedPitchStart);/*Q0*/ /*tmp16=roundedPitchStart*nSubframes*/
    }
    tmp_e = norm_s(tmp16);
    tmp16 = shl(tmp16, tmp_e);/*Q0,-tmp_e*/ /*tmp16=roundedPitchStart*nSubframes*/
    tmp_e = sub(15, tmp_e);
    tmp16 = Inv16(tmp16, &tmp_e); /*Q15,tmp_e*/ /*tmp16=1.0/(roundedPitchStart*nSubframes)*/
    tmp32 = L_sub(pitchEnd,pitchStart);
    tmp2_e = norm_l(tmp32);
    tmp32 = L_shl(tmp32, tmp2_e);/*Q16,-tmp2_e*/
    tmp32 = Mpy_32_16_1(tmp32, tmp16);/*Q16,tmp_e-tmp2_e*/ /*tmp32=pitchDelta*freqStart*/
    tmp_e = sub(tmp_e, tmp2_e); /* sum up all the scalings for tmp32 */

    tmp16 = imult1616(nFrameLength,add(nSubframes,1));/*Q0*//*tmp16=nFrameLength*(nSubframes+1)*/
    tmp2_e = norm_s(tmp16);
    tmp16 = shl(tmp16,tmp2_e);

    tmp32 = Mpy_32_16_1(tmp32,tmp16);/*Q1 scaling (tmp_e-tmp2_e-1), -1 because of 0.5f*/ /*tmp32=0.5f*pitchDelta*nFrameLength*(nSubframes+1)*freqStart*/
    tmp_e = sub(sub(tmp_e,tmp2_e),1); /* sum up all the scalings for tmp32 */
    tmp_e = add(tmp_e,31-1); /* tmp32 is now regarded as Q31 with scaling tmp_e */

    /*samplesDelta -= nFrameLength*(1.0f-pitchStart*freqStart);*/
    tmp2_e = norm_l(pitchStart);
    tmp32_a = L_shl(pitchStart,tmp2_e);
    tmp32_a = Mpy_32_16_1(tmp32_a/*Q16,-tmp2_e*/,freqStart/*Q15,freqStart_e*/);/*Q16, scaling (freqStart_e-tmp2_e)*/ /*tmp32_a=pitchStart*freqStart*/
    tmp16 = norm_l(tmp32_a);
    tmp32_a = L_shl(tmp32_a,tmp16);
    tmp2_e = sub(sub(freqStart_e,tmp16),tmp2_e); /* sum up all scalings for tmp32_a */
    tmp2_e = add(tmp2_e,31-16); /* tmp32_a is now regarded as Q31 with scaling tmp2_e */

    tmp3_e = tmp2_e;
    tmp32_a = L_negate(tmp32_a);
    tmp32_a = L_add(L_shl(1, sub(31, tmp3_e)), tmp32_a); /*Q31,tmp3_e*//*tmp32_a= 1.0f-pitchStart*freqStart*/
    tmp2_e = norm_s(nFrameLength);
    tmp16_a = shl(nFrameLength,tmp2_e);
    tmp32_a = Mpy_32_16_1(tmp32_a/*Q31,tmp3_e*/,tmp16_a/*Q0,-tmp2_e*/);/*Q16,tmp3_e-tmp2_e*/ /*tmp32_a= nFrameLength*(1.0f-pitchStart*freqStart)*/
    tmp2_e = add(sub(tmp3_e, tmp2_e), 15);
    samplesDelta = BASOP_Util_Add_Mant32Exp(tmp32, tmp_e, L_negate(tmp32_a), tmp2_e, &samplesDelta_e); /*Q31,samplesDelta_e*/

    /* To have enough samples in the buffer of length nFrameLength*(nSubframes+1)/nSubframes, pitchEnd/pitchEnd must be bigger than (nSubframes-1)/(nSubframes+1)=1-2/(nSubframes+1) */
    /* Thus nSubframes must be bigger than 1 */
    nSamplesDelta = round_fx(L_shl(samplesDelta,sub(samplesDelta_e,31-16))); /*Q0*/
    nSamplesDeltaRemain = abs_s(nSamplesDelta);
    /* Find the location of the glottal pulse */
    T0 = FindMaxPeak(src_exc, roundedPitchStart); /*Q0*/
    /* Get the index of the last pulse in the resynchronized frame */
    /*k = (int)ceil((nFrameLength-nSamplesDelta-T0)*freqStart - 1);*/
    tmp32 = BASOP_Util_Add_Mant32Exp(L_mult(sub(nFrameLength,add(nSamplesDelta,T0)),freqStart)/*Q16*/,add(freqStart_e,31-16),FL2WORD32(-1.f),0,&tmp_e);
    tmp32 = L_shl(tmp32,sub(tmp_e,31-16))/*Q16*/;
    tmp32 = L_add(tmp32,FL2WORD32_SCALE(1.f,31-16));
    k=extract_h(tmp32);
    test();
    IF ((k >= 0) && sub(add(k,1) , NB_PULSES_MAX)<=0)
    {
        absPitchDiff = L_abs(L_sub(L_deposit_h(roundedPitchStart),pitchEnd));/*Q16*/

        /* Calculate the delta of the samples to be added/removed between consecutive cycles */
        /*perCycleDeltaDelta = (absPitchDiff*(nFrameLength-samplesDelta) - (float)fabs(samplesDelta)*roundedPitchStart)
                             / ((k+1)*(T0+0.5f*k*roundedPitchStart));*/
        tmp32 = L_sub(L_deposit_h(nFrameLength),L_shl(samplesDelta,sub(samplesDelta_e,31-16)));/*Q16*/
        tmp_e = 15;  /*tmp32 = Q31,tmp_e*/                                                          move16();
        tmp2_e = norm_l(tmp32);
        tmp32 = L_shl(tmp32,tmp2_e);
        tmp_e = sub(tmp_e,tmp2_e); /*tmp32 = Q31,tmp_e*/
        tmp2_e = norm_l(absPitchDiff);
        tmp32_b = L_shl(absPitchDiff,tmp2_e);
        tmp_e = sub(tmp_e,tmp2_e);
        tmp32 = Mpy_32_16_1(tmp32_b,round_fx(tmp32));/*Q16,tmp_e*/ /*tmp32 = absPitchDiff*(nFrameLength-samplesDelta)*/
        tmp32_a = Mpy_32_16_1(L_abs(samplesDelta)/*Q31,samplesDelta_e*/,roundedPitchStart/*Q0*/);/*Q16,samplesDelta_e*/ /*tmp32_a=fabs(samplesDelta)*roundedPitchStart*/
        tmp32 = BASOP_Util_Add_Mant32Exp(tmp32, add(tmp_e,31-16), L_negate(tmp32_a), add(samplesDelta_e,31-16),&tmp_e);/*Q31,tmp_e*/ /*tmp32=absPitchDiff*(nFrameLength-samplesDelta)-fabs(samplesDelta)*roundedPitchStart*/
        tmp16 = imult1616(add(k,1),add(shl(T0,1),imult1616(k,roundedPitchStart)));/*Q0,-1*/ /*tmp16=(k+1)*(T0+0.5f*k*roundedPitchStart)*/
        perCycleDeltaDelta = BASOP_Util_Divide3216_Scale(tmp32/*Q31,tmp_e*/,tmp16/*Q0,-1*/,&perCycleDeltaDelta_e); /*Q15,perCycleDeltaDelta_e*/
        perCycleDeltaDelta_e = add(perCycleDeltaDelta_e,sub(tmp_e,-1+15));
        tmp_e = norm_s(perCycleDeltaDelta);
        perCycleDeltaDelta_e = sub(perCycleDeltaDelta_e,tmp_e);
        perCycleDeltaDelta = shl(perCycleDeltaDelta,tmp_e);/*Q15,perCycleDeltaDelta_e*/


        /* Calculate the integer number of samples to be added/removed in each pitch cycle */
        /*cycleDelta = max(0, (absPitchDiff-(k+1)*perCycleDeltaDelta)*T0*freqStart); */
        tmp_e = norm_s(k+1);
        tmp32 = L_mult(perCycleDeltaDelta/*Q15,perCycleDeltaDelta_e*/,shl(add(k,1),tmp_e)/*Q0, tmp_e*/)/*Q0+16, perCycleDeltaDelta_e-tmp_e*/;
        tmp32 = BASOP_Util_Add_Mant32Exp(absPitchDiff/*Q16*/,31-16,L_negate(tmp32),add(sub(perCycleDeltaDelta_e,tmp_e),31-16),&tmp_e);/*Q31,tmp_e*/
        tmp32 = Mpy_32_16_1(tmp32,T0/*Q0*/);/*Q16,tmp_e*/
        tmp32 = Mpy_32_16_1(tmp32/*Q16,tmp_e*/,freqStart/*Q15,freqStart_e*/)/*Q16, tmp_e+(freqStart_e)*/;
        tmp32 = L_max(0,tmp32);
        cycleDelta_e = add(tmp_e,freqStart_e);
        tmp32_a = L_shl(tmp32,cycleDelta_e);
        roundedCycleDelta = extract_h(L_abs(tmp32_a));
        if (tmp32<0)
        {
            roundedCycleDelta = negate(roundedCycleDelta);
        }
        fractionalLeft = lshr(extract_l(tmp32_a),1);/*Q15*/
        tmp_e = sub(15,norm_l(tmp32));
        cycleDelta_e = add(cycleDelta_e,tmp_e);
        tmp32 = L_shr(tmp32,sub(tmp_e,15));/*Q31 frac, cycleDelta_e*/
        cycleDelta = round_fx(tmp32);/*Q15, cycleDelta_e*/
        if (cycleDelta == 0)
        {
            move16();
            cycleDelta_e = 0;
        }

        /*roundedCycleDelta = (int)(cycleDelta); */ /*done above*/
        move16();
        iDeltaSamples[0] = roundedCycleDelta;/*Q0*/
        /*fractionalLeft = cycleDelta-roundedCycleDelta;*/ /*done above*/
        nSamplesDeltaRemain = sub(nSamplesDeltaRemain,roundedCycleDelta);/*Q0*/

        tmp_e = (s_max(2,k));
        tmp_e = norm_s(tmp_e);/*maximum norming factor for following loop*/


        FOR (i = 1; i <= k; i++)
        {
            /*cycleDelta = (absPitchDiff-(k+1-i)*perCycleDeltaDelta) + fractionalLeft; */
            tmp32 = L_mult(perCycleDeltaDelta/*Q15,perCycleDeltaDelta_e*/,shl(sub(add(k,1),i),tmp_e)/*Q0, tmp_e*/)/*Q0+16, perCycleDeltaDelta_e-tmp_e*/; /*calcultion of base for first iteration*/
            tmp32 = L_shl(tmp32,sub(perCycleDeltaDelta_e,tmp_e));/*Q16*/
            tmp32_a = L_sub(absPitchDiff,tmp32);
            tmp32_b = L_lshl(L_deposit_l(fractionalLeft/*Q15*/),1)/*Q16*/;
            cycleDelta32 = L_add(tmp32_a,tmp32_b);/*Q16*/
            cycleDelta32 = L_max(0, cycleDelta32);

            /* Make sure that the number of samples increases */
            IF (L_sub(L_deposit_h(roundedCycleDelta), cycleDelta32) > 0)
            {
                iDeltaSamples[i] = roundedCycleDelta;
                move16();
                roundedCycleDelta = extract_h(cycleDelta32); /* cycleDelta32 should never be < 0 here */
                iDeltaSamples[i-1] = roundedCycleDelta;
                move16();

            }
            ELSE
            {
                roundedCycleDelta = extract_h(cycleDelta32); /* cycleDelta32 should never be < 0 here */
                iDeltaSamples[i] = roundedCycleDelta;
                move16();
            }
            /*fractionalLeft = cycleDelta-roundedCycleDelta = cycleDelta-(int)cycleDelta;*/
            fractionalLeft = lshr(extract_l(cycleDelta32),1); /*Q15*/ /* cycleDelta32 should never be < 0 here */
            nSamplesDeltaRemain = sub(nSamplesDeltaRemain,roundedCycleDelta);
        }
        iDeltaSamples[k+1] = s_max(0, nSamplesDeltaRemain);
        move16();
        maxDeltaSamples = s_max(iDeltaSamples[k], iDeltaSamples[k+1]);/*Q0*/

        /* Find the location of the minimum energy between the first two pulses */

        /*iMinPos1 = T0+GetMinimumPosition(src_exc+T0, min(roundedPitchStart, (nSubframes+1)*nFrameLength/nSubframes-T0), maxDeltaSamples);*/
        BASOP_Util_Divide_MantExp(add(nSubframes,1),15,nSubframes,15,&tmp16,&tmp_e);
        tmp32 = L_mult(nFrameLength/*Q0*/,tmp16/*Q15,tmp_e*/);/*Q16,tmp_e*/
        tmp16 = round_fx(L_shl(tmp32,tmp_e));
        tmp16 = sub(tmp16,T0);
        tmp16 = s_min(roundedPitchStart,tmp16);

        iMinPos1 = GetMinimumPosition(
                       src_exc+T0,         /*Qx*/
                       tmp16,              /*Q0*/
                       maxDeltaSamples     /*Q0*/
                   );
        iMinPos1 = add(iMinPos1,T0);


        IF (nSamplesDelta < 0)
        {
            /* Find the location of the minimum energy before the first pulse */

            IF (sub(iMinPos1 , add(roundedPitchStart , shr(iDeltaSamples[0],1))) > 0 )
            {
                iMinPos[0] = sub(iMinPos1 , sub(roundedPitchStart , shr(iDeltaSamples[0],1)));
                move16();
            }
            ELSE
            {
                move16();
                iMinPos[0] = sub(GetMinimumPosition(src_exc, T0, iDeltaSamples[0]) , shr(iDeltaSamples[0],1));
            }

            /* Find the location of the minimum energy between the pulses */
            FOR (i = 1; i <= k; i++)
            {
                move16();
                iMinPos[i] = add(iMinPos1 , sub(imult1616(sub(i,1),roundedPitchStart) , shr(iDeltaSamples[i],1)));
            }
            /* Find the location of the minimum energy after the last pulse */

            IF (sub(add(iMinPos1 , add(imult1616(k,roundedPitchStart) , sub(iDeltaSamples[k+1] , shr(iDeltaSamples[k+1],1))))  , sub(nFrameLength,nSamplesDelta) ) < 0)
            {
                move16();
                iMinPos[k+1] = add(iMinPos1 , sub(imult1616(k,roundedPitchStart) , shr(iDeltaSamples[k+1],1)));
            }
            ELSE
            {
                /*iMinPos[k+1] = T0+k*roundedPitchStart
                               + GetMinimumPosition(src_exc+T0+k*roundedPitchStart, nFrameLength-nSamplesDelta-(T0+k*roundedPitchStart), iDeltaSamples[k+1])
                               - iDeltaSamples[k+1]/2;                                                 */
                tmp16 = GetMinimumPosition(src_exc+T0+k*roundedPitchStart, sub(nFrameLength,add(nSamplesDelta,add(T0,imult1616(k,roundedPitchStart)))), iDeltaSamples[k+1]);
                tmp16 = add(add(T0,imult1616(k,roundedPitchStart)),tmp16);
                tmp16 = sub(tmp16,shr(iDeltaSamples[k+1],1));
                iMinPos[k+1] = tmp16;
                move16();
            }

            IF (sub(add(iMinPos[k+1],iDeltaSamples[k+1]) , sub(nFrameLength,nSamplesDelta)) > 0 )
            {
                iDeltaSamples[k] += add(iMinPos[k+1] , sub(iDeltaSamples[k+1] , sub(nFrameLength,nSamplesDelta)));
                iDeltaSamples[k+1] = sub(nFrameLength , add(nSamplesDelta , iMinPos[k+1]));
            }

            /* Remove samples at the given positions */
            RemoveSamples(src_exc, dst_exc, nFrameLength, nSamplesDelta, iMinPos, iDeltaSamples, k+2);
        }
        ELSE
        {
            /* Find the location of the minimum energy before the first pulse */
            IF (sub(iMinPos1 , roundedPitchStart) > 0 )
            {
                iMinPos[0] = sub(iMinPos1 , roundedPitchStart);
                move16();
            }
            ELSE
            {
                iMinPos[0] = GetMinimumPosition(src_exc, T0, iDeltaSamples[0]);
                move16();
            }
            /* Find the location of the minimum energy between the pulses */

            FOR (i = 1; i <= k; i++)
            {
                iMinPos[i] = iMinPos1;
                move16();
                iMinPos1 = add(iMinPos1,roundedPitchStart);
            }

            /* Find the location of the minimum energy after the last pulse */
            IF (sub(iMinPos1 , sub(nFrameLength,nSamplesDelta)) < 0)
            {
                iMinPos[k+1] = iMinPos1;
                move16();
            }
            ELSE
            {

                tmp16 = GetMinimumPosition(src_exc+T0+k*roundedPitchStart, sub(nFrameLength,add(nSamplesDelta,add(T0,imult1616(k,roundedPitchStart)))), iDeltaSamples[k+1]);
                tmp16 = add(add(tmp16,T0),imult1616(k,roundedPitchStart));
                iMinPos[k+1] = tmp16;
                move16();
            }

            IF (sub(add(iMinPos[k+1],iDeltaSamples[k+1]) , sub(nFrameLength,nSamplesDelta)) > 0 )
            {
                move16();
                move16();
                iDeltaSamples[k] = add(iDeltaSamples[k] , add(iMinPos[k+1] , sub(iDeltaSamples[k+1] , sub(nFrameLength,nSamplesDelta))));
                iDeltaSamples[k+1] = sub(sub(nFrameLength, nSamplesDelta),iMinPos[k+1]);
            }
            /* Add samples at the given positions */
            AddSamples(src_exc, dst_exc, nFrameLength, nSamplesDelta, iMinPos, iDeltaSamples, k+2);
        }
    }

}

