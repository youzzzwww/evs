/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#define _USE_MATH_DEFINES

#include <assert.h>
#include "stl.h"
#include "basop_util.h"
#include "options.h"
#include "typedef.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stat_com.h"


/************************************************************************************/
/* forward declarations for local functions, see implementation at end of this file */
/************************************************************************************/

static void CalcMDST(void * p,
                     Word16 const * timeSignal,
                     Word32 * mdstOutput,
                     Word16 *mdstOutput_e);

static void CalcPowerSpec(Word32 * mdctSpec,                /* i: MDCT spectrum                        */
                          Word16 mdctSpec_exp,              /* i: exponent of MDCT spectrum            */
                          Word32 * mdstSpec,                /* i: MDST spectrum                        */
                          Word16 mdstSpec_exp,              /* i: exponent of MDST spectrum               */
                          Word16 nSamples,                  /* i: frame size                           */
                          Word16 floorPowerSpectrum,        /* i: lower limit for power spectrum bins  */
                          Word32 * powerSpec,               /* o: power spectrum                       */
                          Word16 * powerSpec_exp);

static void CalcPowerSpecAndDetectTonalComponents(TonalMDCTConcealPtr const self,
        Word32 secondLastMDST[],
        Word16 secondLastMDST_exp,
        Word32 const pitchLag);
static void FindPhases(                                /* o: current phase   [-pi;pi]        2Q13 */
    TonalMDCTConcealPtr const self, /* i: pointer to internal structure        */
    Word32 secondLastMDCT[],        /* i: MDST spectrum data                   */
    Word32 secondLastMDST[],        /* i: MDCT spectrum data                   */
    Word16 diff_exp);               /* i: exp_MDST - exp_MDCT                  */

static void FindPhaseDifferences(                                   /* o: Phase difference [-pi;pi]        2Q13*/
    TonalMDCTConcealPtr const self,    /* i: Pointer to internal structure        */
    Word32 powerSpectrum[]);           /* i: Power spectrum data                  */


/*******************************************************/
/*-------------- public functions -------------------- */
/*******************************************************/

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Init( TonalMDCTConcealPtr self,
        Word16 nSamples,
        Word16 nSamplesCore,
        Word16 nScaleFactors,
        TCX_config * tcx_cfg,
        ApplyScaleFactorsPointer pApplyScaleFactors
                                            )
{
    test();
    IF (sub(nSamples,L_FRAME_MAX) > 0 || sub(nScaleFactors,FDNS_NPTS) > 0)
    {
        assert(nSamples <= L_FRAME_MAX);
        assert(nScaleFactors <= FDNS_NPTS);
        return TONALMDCTCONCEAL_NSAMPLES_LARGER_THAN_MAXBLOCKSIZE;
    }
    assert((self->nScaleFactors == nScaleFactors) || (self->nSamples != nSamples)); /* If nSamples doesn't change then also nScaleFactors must stay the same */

    self->tcx_cfg = tcx_cfg;

    self->lastBlockData.spectralData       = self->spectralDataBuffers[0];
    move16();
    self->secondLastBlockData.spectralData = self->spectralDataBuffers[1];
    move16();
    self->secondLastPowerSpectrum = self->secondLastBlockData.spectralData;
    move16();

    self->lastBlockData.scaleFactors       = self->scaleFactorsBuffers[0];
    move16();
    self->secondLastBlockData.scaleFactors = self->scaleFactorsBuffers[1];
    move16();
    self->lastBlockData.scaleFactors_exp       = self->scaleFactorsBuffers_exp[0];
    move16();
    self->secondLastBlockData.scaleFactors_exp = self->scaleFactorsBuffers_exp[1];
    move16();

    self->lastBlockData.blockIsValid       = 0;
    move16();
    self->secondLastBlockData.blockIsValid = 0;
    move16();
    self->nSamples = 0;
    move16();
    self->nScaleFactors = 0;
    move16();

    /* If the second last frame was lost, we reuse saved TonalComponentsInfo and don't update pcm buffers */
    assert(sizeof(*self->pTCI) <= (self->lastPcmOut-self->timeDataBuffer)*sizeof(self->timeDataBuffer[0]));

    self->pTCI = (TonalComponentsInfo *)self->timeDataBuffer;
    move16();


    self->pApplyScaleFactors = pApplyScaleFactors;
    move16();
    self->lastPitchLag = L_deposit_l(0);

    IF (sub(self->nSamples,nSamples) != 0)
    {
        self->secondLastBlockData.blockIsValid = 0;
        move16();
        self->lastBlockData.blockIsValid = 0;
        move16();
    }

    self->nSamples = nSamples;
    move16();
    self->nSamplesCore = nSamplesCore;
    move16();

    self->nScaleFactors = nScaleFactors;
    move16();

    /* Offset the pointer to the end of buffer, so that pTCI is not destroyed when
       new time samples are stored in lastPcmOut */  move16();
    move16();
    self->secondLastPcmOut        = &self->timeDataBuffer[sub(2*L_FRAME_MAX,shl(s_min(L_FRAME_MAX, nSamples),1))];
    self->lastPcmOut              = &self->timeDataBuffer[sub(2*L_FRAME_MAX,s_min(L_FRAME_MAX, nSamples))];


    return TONALMDCTCONCEAL_OK;
}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_SaveFreqSignal( TonalMDCTConcealPtr self,
        Word32 const *mdctSpectrum,
        Word16 const mdctSpectrum_exp,
        Word16 nNewSamples,
        Word16 nNewSamplesCore,
        Word16 const *scaleFactors,
        Word16 const *scaleFactors_exp,
        Word16 const gain_tcx_exp
                                                      )
{
    Word16 * temp;
    Word16 nOldSamples, tmp_exp, s, i, max_exp;


    assert(nNewSamples > 0 && nNewSamples <= 2*L_FRAME_MAX);

    /* Avoid overwriting self->secondLastPowerSpectrum stored in spectralData,
       because it is needed if the second last and the current frame are lost
       and concealed using the Tonal MDCT PLC */ test();
    IF (!self->lastBlockData.tonalConcealmentActive || sub(self->lastBlockData.nSamples,nNewSamples) != 0)
    {
        IF (sub(nNewSamples,L_FRAME_MAX) <= 0)
        {
            /* Shift the buffers */
            temp = self->secondLastBlockData.spectralData; /* Save the pointer */        move16();
            self->secondLastBlockData.spectralData = self->lastBlockData.spectralData;
            move16();
            self->lastBlockData.spectralData = temp;
            move16();

            tmp_exp = self->secondLastBlockData.spectralData_exp; /* Save the pointer */        move16();
            self->secondLastBlockData.spectralData_exp = self->lastBlockData.spectralData_exp;
            move16();
            self->lastBlockData.spectralData_exp = tmp_exp;
            move16();

            tmp_exp = self->secondLastBlockData.gain_tcx_exp; /* Save the pointer */            move16();
            self->secondLastBlockData.gain_tcx_exp = self->lastBlockData.gain_tcx_exp;
            move16();
            self->lastBlockData.gain_tcx_exp = tmp_exp;
            move16();

            tmp_exp = self->secondLastBlockData.scaleFactors_max_e; /* Save the pointer */           move16();
            self->secondLastBlockData.scaleFactors_max_e = self->lastBlockData.scaleFactors_max_e;
            move16();
            self->lastBlockData.scaleFactors_max_e = tmp_exp;
            move16();

            temp = self->secondLastBlockData.scaleFactors;
            move16();
            self->secondLastBlockData.scaleFactors = self->lastBlockData.scaleFactors;
            move16();
            self->lastBlockData.scaleFactors = temp;
            move16();

            temp = self->secondLastBlockData.scaleFactors_exp;
            move16();
            self->secondLastBlockData.scaleFactors_exp = self->lastBlockData.scaleFactors_exp;
            move16();
            self->lastBlockData.scaleFactors_exp = temp;
            move16();
        }
        ELSE
        {
            self->lastBlockData.spectralData           = self->spectralDataBuffers[0];
            move16();
            self->secondLastBlockData.spectralData     = self->spectralDataBuffers[1];
            move16();
            self->lastBlockData.scaleFactors           = self->scaleFactorsBuffers[0];
            move16();
            self->secondLastBlockData.scaleFactors     = self->scaleFactorsBuffers[1];
            move16();
            self->lastBlockData.scaleFactors_exp       = self->scaleFactorsBuffers_exp[0];
            move16();
            self->secondLastBlockData.scaleFactors_exp = self->scaleFactorsBuffers_exp[1];
            move16();
        }

        nOldSamples = self->lastBlockData.nSamples;
        move16();
        self->lastBlockData.nSamples = nNewSamples;
        move16();
        self->secondLastBlockData.nSamples = nOldSamples;
        move16();

        nOldSamples = self->lastBlockData.nSamplesCore;
        move16();
        self->lastBlockData.nSamplesCore = nNewSamplesCore;
        move16();
        self->secondLastBlockData.nSamplesCore = nOldSamples;
        move16();
    }

    test();
    IF ((nNewSamples > 0) && (sub(nNewSamples,2*L_FRAME_MAX) <= 0))
    {
        /* Store new data */
        s = getScaleFactor32(mdctSpectrum, nNewSamples);

        /*Copy(scaleFactors_exp, self->lastBlockData.scaleFactors_exp, self->nScaleFactors);*/
        max_exp = 0;
        FOR (i = 0; i < self->nScaleFactors; i++)
        {
            self->lastBlockData.scaleFactors_exp[i] = scaleFactors_exp[i];
            move16();
            max_exp = s_max(max_exp, scaleFactors_exp[i]);
        }

        /*s = sub(s, max_exp);*/
        self->lastBlockData.scaleFactors_max_e = max_exp;

        FOR (i = 0; i < nNewSamples; i++)
        {
            self->lastBlockData.spectralData[i] = extract_h(L_shl(mdctSpectrum[i], s));
            move16();
        }
        self->lastBlockData.spectralData_exp = sub(mdctSpectrum_exp,s);
        move16();
        self->lastBlockData.gain_tcx_exp = gain_tcx_exp;

        Copy(scaleFactors, self->lastBlockData.scaleFactors, self->nScaleFactors);
    }
    return TONALMDCTCONCEAL_OK;
}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_UpdateState(TonalMDCTConcealPtr self,
        Word16 nNewSamples,
        Word32 pitchLag,
        Word16 badBlock,
        Word8 tonalConcealmentActive
                                                   )
{
    Word8 newBlockIsValid;

    assert(!(!badBlock && tonalConcealmentActive));

    IF (badBlock)
    {
        newBlockIsValid = self->lastBlockData.blockIsValid;
        move16();
    }
    ELSE
    {
        newBlockIsValid = 0;
        move16();
        test();
        if((sub(nNewSamples,2*L_FRAME_MAX) <= 0) && (nNewSamples > 0))
        {
            newBlockIsValid = 1;
            move16();
        }
    }

    /* Shift old state */ move16();
    move16();
    move16();
    self->secondLastBlockData.blockIsConcealed = self->lastBlockData.blockIsConcealed;
    self->secondLastBlockData.blockIsValid = self->lastBlockData.blockIsValid;
    self->secondLastBlockData.tonalConcealmentActive = self->lastBlockData.tonalConcealmentActive;

    /* Store new state */ move16();
    move16();
    move16();
    self->lastBlockData.blockIsConcealed = badBlock;
    self->lastBlockData.blockIsValid = newBlockIsValid;
    self->lastBlockData.tonalConcealmentActive = tonalConcealmentActive;

    self->lastPitchLag = pitchLag;
    move32();

    return TONALMDCTCONCEAL_OK;
}
static void FindPhases(                                /* o: currenc phase   [-pi;pi]        2Q13 */
    TonalMDCTConcealPtr const self, /* i: pointer to internal structure        */
    Word32 secondLastMDCT[],        /* i: MDST spectrum data                   */
    Word32 secondLastMDST[],        /* i: MDCT spectrum data                   */
    Word16 diff_exp)                /* i: exp_MDST - exp_MDCT                  */
{
    Word16 i;
    Word16 l;
    Word16 *pCurrentPhase;



    pCurrentPhase = self->pTCI->phase_currentFramePredicted;
    /* for each index/index group */
    FOR( i = 0; i < self->pTCI->numIndexes; i++)
    {
        FOR (l = self->pTCI->lowerIndex[i]; l <= self->pTCI->upperIndex[i]; l++)
        {
            /* in contrast to the float code, the parameter secondLastMDST[l]
               needs not to be negated - due to a different implementation of
               the MDST */
            *pCurrentPhase++ = BASOP_util_atan2(secondLastMDST[l], secondLastMDCT[l], diff_exp);
            move16();
        }
    }

}

#define BANDWIDTH 7.0f
#define G         FL2WORD32(1.0/(2*1.36))
#define MAXRATIO  FL2WORD16_SCALE(44.8f,6) /* Maximum ratio |ODFT[k-1]|/|ODFT[k+1]| is 16.5 dB, that is maximum ratio (for fractional = 0) is (cos(PI/bandwidth)/cos(3PI/bandwidth))^1.36 */
#define MM        1934815907               /* FL2WORD32(cos(EVS_PI/BANDWIDTH));  */
#define SS        29166                    /* FL2WORD16(cos((3*EVS_PI)/BANDWIDTH)*4);  Q17*/
#define N         931758243                /* FL2WORD32(sin(EVS_PI/BANDWIDTH)); */
#define J         31946                    /* FL2WORD16(sin((3*EVS_PI)/BANDWIDTH)); */

static void FindPhaseDifferences(                                   /* o: Phase difference [-pi;pi]         2Q13*/
    TonalMDCTConcealPtr const self,    /* i: Pointer to internal structure        */
    Word32 powerSpectrum[])            /* i: Power spectrum data                  */
{
    Word16 i, k;
    Word16 * phaseDiff;
    Word16 fractional, sf, sfn, sfd;
    Word16 divi, s, j;
    Word32 a, Q, L_tmp, m, n;

    s = SS;
    move16();
    j = J;
    move16();

    phaseDiff = self->pTCI->phaseDiff;

    FOR (i = 0; i < self->pTCI->numIndexes; i++)
    {
        m = MM;
        move16();
        n = N;
        move16();

        k = self->pTCI->indexOfTonalPeak[i];
        move16();

        IF (L_sub(Mpy_32_16_1(powerSpectrum[k-1],FL2WORD16_SCALE(1.0f,6)),Mpy_32_16_1(powerSpectrum[k+1], MAXRATIO))  >= 0)
        {
            phaseDiff[i] = 0; /*(float)tan(0.0f*EVS_PI/bandwidth);*/                      move16();
            if(s_and(k,1) != 0)
                phaseDiff[i] = FL2WORD16_SCALE(-EVS_PI,2);  /*2Q13*/
        }
        ELSE
        {
            IF (L_sub(Mpy_32_16_1(powerSpectrum[k+1],FL2WORD16_SCALE(1.0f,6)),Mpy_32_16_1(powerSpectrum[k-1], MAXRATIO))  >= 0)
            {
                phaseDiff[i] = FL2WORD16_SCALE(EVS_PI,2); /*(float)tan(2.0f*PI/bandwidth);*/ move16();
                if(s_and(k,1) != 0)
                    phaseDiff[i] = FL2WORD16_SCALE(0,2);  /*2Q13*/
            }
            ELSE {
                /*Q = (float)pow(odft_left/odft_right, G);
                  a = (m - Q * s) / (n + Q * j);
                  phaseDiff[i] = (float)atan(a) * (bandwidth/2.0f);*/
                /*max divi=44.8 & sf=6*/
                divi = BASOP_Util_Divide3232_uu_1616_Scale(powerSpectrum[k-1],powerSpectrum[k+1], &sf);
                Q = BASOP_Util_fPow(L_deposit_h(divi), sf, G, 0, &sf);
                L_tmp = Mpy_32_16_1(Q,s);
                sfn = sub(sf, 2);

                if(sfn > 0)
                    m = L_shr(m, sfn);

                IF(sfn < 0)
                {
                    L_tmp = L_shl(L_tmp, sfn);
                    sfn = 0;
                }

                a = L_sub(m, L_tmp);  /*sf*/

                L_tmp = Mpy_32_16_1(Q,j);
                IF(sf >= 0)
                {
                    L_tmp = L_shr(L_tmp, 1);
                    sfd = add(sf,1);
                    n = L_shr(n,sfd);
                }
                ELSE{
                    sfd = 0;
                    L_tmp = L_shl(L_tmp, sf);
                }

                L_tmp = L_add(n,L_tmp);
                fractional = BASOP_util_atan2(a, L_tmp, sub(sfn,sfd));  /*2Q13*/
                L_tmp = L_mult(fractional, FL2WORD16_SCALE(BANDWIDTH/2.0f,2));    /*2Q13*2Q13=4Q27*/     move16();

                /* Add PI for odd bins */
                if(s_and(k,1) != 0)
                    L_tmp = L_sub(L_tmp, FL2WORD32_SCALE(EVS_PI,4));  /*4Q27*/

                phaseDiff[i] = round_fx(L_shl(L_tmp,2));      /*2Q13*/
            }
        }
    }
}

static void CalcPowerSpecAndDetectTonalComponents(TonalMDCTConcealPtr const self,
        Word32 secondLastMDST[],
        Word16 secondLastMDST_exp,
        Word32 const pitchLag)
{
    Word16 nSamples;
    Word16 i;
    Word16 floorPowerSpectrum; /* Minimum significant value of a spectral line in the power spectrum */
    Word32 secondLastMDCT[L_FRAME_MAX];
    Word32 powerSpectrum[L_FRAME_MAX];
    Word16 invScaleFactors[FDNS_NPTS];
    Word16 invScaleFactors_exp[FDNS_NPTS];
    Word16 powerSpectrum_exp, tmp_exp, old_exp, secondLastMDCT_exp;


    nSamples = self->nNonZeroSamples;
    move16();


    self->pApplyScaleFactors(self->secondLastBlockData.spectralData, self->secondLastBlockData.nSamplesCore, nSamples,
                             self->secondLastBlockData.scaleFactors, self->secondLastBlockData.scaleFactors_exp, self->secondLastBlockData.scaleFactors_max_e, secondLastMDCT);

    /* It is taken into account that the MDCT is not normalized. */
    floorPowerSpectrum/*Q0*/ = extract_l(Mpy_32_16_1(L_mult0(self->nSamples,self->nSamples),82));      /*1/400 = 82 Q15*/
    powerSpectrum_exp = 0;
    move16();
    secondLastMDCT_exp = add(add(self->secondLastBlockData.spectralData_exp,self->secondLastBlockData.gain_tcx_exp),self->secondLastBlockData.scaleFactors_max_e);

    CalcPowerSpec(secondLastMDCT,
                  secondLastMDCT_exp,
                  secondLastMDST,
                  secondLastMDST_exp,
                  nSamples,
                  floorPowerSpectrum,
                  powerSpectrum,
                  &powerSpectrum_exp);

    /* This setting to minimal level is required because the power spectrum is used in the threshold adaptation using the pitch up to self->nSamples. */
    set32_fx(powerSpectrum+nSamples, floorPowerSpectrum, sub(self->nSamples, nSamples));

    DetectTonalComponents(self->pTCI->indexOfTonalPeak,
                          self->pTCI->lowerIndex,
                          self->pTCI->upperIndex,
                          &self->pTCI->numIndexes,
                          self->lastPitchLag,
                          pitchLag,
                          self->lastBlockData.spectralData,
                          add(self->lastBlockData.spectralData_exp,self->lastBlockData.gain_tcx_exp),
                          self->pApplyScaleFactors,
                          self->lastBlockData.scaleFactors,
                          self->lastBlockData.scaleFactors_exp,
                          self->lastBlockData.scaleFactors_max_e,
                          powerSpectrum,
                          nSamples
                          ,self->nSamplesCore
                          ,floorPowerSpectrum
                         );

    FindPhases(self, secondLastMDCT, secondLastMDST, sub(secondLastMDST_exp,secondLastMDCT_exp));

    FindPhaseDifferences(self, powerSpectrum);

    IF (self->pTCI->numIndexes > 0)
    {

        self->secondLastPowerSpectrum = self->secondLastBlockData.spectralData;

        /*sqrtFLOAT(powerSpectrum, powerSpectrum, nSamples);*/
        old_exp = powerSpectrum_exp;
        powerSpectrum_exp = mult_r(sub(powerSpectrum_exp,2), 1 << 14);  /*remove 2 bits of headroom from CalcPowerSpec*/
        FOR (i = 0; i < nSamples; i++)
        {
            tmp_exp = old_exp;
            powerSpectrum[i] = Sqrt32(powerSpectrum[i], &tmp_exp);
            powerSpectrum[i] = L_shr(powerSpectrum[i], sub(powerSpectrum_exp, tmp_exp));
            move32();
        }

        FOR (i = 0; i < self->nScaleFactors; i++)
        {
            move16();
            move16();
            invScaleFactors_exp[i] = self->secondLastBlockData.scaleFactors_exp[i];
            invScaleFactors[i] = Inv16(self->secondLastBlockData.scaleFactors[i], &invScaleFactors_exp[i]);
        }

        mdct_shaping(powerSpectrum, self->nSamplesCore, invScaleFactors, invScaleFactors_exp);
        FOR (i = self->nSamplesCore; i < nSamples; i++)
        {
            powerSpectrum[i] = L_shl(Mpy_32_16_1(powerSpectrum[i], invScaleFactors[FDNS_NPTS-1]), invScaleFactors_exp[FDNS_NPTS-1]);
            move32();
        }

        /* 16 bits are now enough for storing the power spectrum */
        FOR (i = 0; i < nSamples; i++)
        {
            self->secondLastPowerSpectrum[i] = round_fx(powerSpectrum[i]);
        }

        powerSpectrum_exp = sub(powerSpectrum_exp, self->secondLastBlockData.gain_tcx_exp);
        self->secondLastPowerSpectrum_exp = powerSpectrum_exp;
        move16();
    }
}


static void CalcMDST(void * p,
                     Word16 const * timeSignal,
                     Word32 * mdstOutput,
                     Word16 *mdstOutput_e)
{
    Word16 windowedTimeSignal[L_FRAME_PLUS+2*L_MDCT_OVLP_MAX];
    struct tonalmdctconceal const * pMDSTInfo = (struct tonalmdctconceal const *)p;
    Word16 left_overlap, right_overlap, L_frame;


    L_frame = pMDSTInfo->nSamples;
    move16();

    WindowSignal(pMDSTInfo->tcx_cfg,
                 pMDSTInfo->tcx_cfg->tcx_offsetFB,
                 FULL_OVERLAP,
                 FULL_OVERLAP,
                 &left_overlap,
                 &right_overlap,
                 timeSignal,
                 &L_frame,
                 windowedTimeSignal,
                 1);

    TCX_MDST(windowedTimeSignal,
             mdstOutput,
             mdstOutput_e,
             left_overlap,
             sub(L_frame, shr(add(left_overlap, right_overlap), 1)),
             right_overlap);

}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Detect( TonalMDCTConcealPtr const self,
        Word32 const pitchLag,
        Word16 const * const pastTimeSignal,
        Word16 * const numIndices)
{
    Word32 secondLastMDST[L_FRAME_MAX];
    Word32 * powerSpectrum = secondLastMDST;
    Word16 i, powerSpectrum_exp, secondLastMDST_exp, s;
    Word16 nSamples;


    nSamples = self->nSamples;
    move16();
    secondLastMDST_exp = 16;  /*time signal Q-1*/
    test();
    test();
    test();
    test();
    test();
    IF (self->lastBlockData.blockIsValid && self->secondLastBlockData.blockIsValid
        && (sub(self->lastBlockData.nSamples,nSamples) == 0) && (sub(self->secondLastBlockData.nSamples,nSamples) == 0)
        && (!self->secondLastBlockData.blockIsConcealed || self->secondLastBlockData.tonalConcealmentActive || (pitchLag != 0)) /* Safety if the second last frame was concealed and tonal concealment was inactive */
       )
    {

        IF (self->lastBlockData.blockIsConcealed == 0)
        {
            IF (self->secondLastBlockData.tonalConcealmentActive == 0)
            {
                IF (pastTimeSignal == NULL)
                {
                    CalcMDST(self, self->secondLastPcmOut, secondLastMDST, &secondLastMDST_exp);
                }
                ELSE
                {
                    CalcMDST(self, pastTimeSignal, secondLastMDST, &secondLastMDST_exp);
                }

                self->nNonZeroSamples = 0;
                FOR (i = 0; i < self->nSamples; i++)
                {
                    if (self->secondLastBlockData.spectralData[i] != 0)
                    {
                        self->nNonZeroSamples = i;
                        move16();
                    }
                }

                /* 23 is the maximum length of the MA filter in getEnvelope */
                self->nNonZeroSamples = s_min(self->nSamples, add(self->nNonZeroSamples, 23));
                move16();
                nSamples = self->nNonZeroSamples;
                move16();

                s = getScaleFactor32(secondLastMDST, nSamples);

                FOR (i = 0; i < nSamples; i++)
                {
                    secondLastMDST[i] = L_shl(secondLastMDST[i], s);
                    move32();
                }
                secondLastMDST_exp = sub(secondLastMDST_exp, s);
                move16();
                CalcPowerSpecAndDetectTonalComponents(self, secondLastMDST, secondLastMDST_exp, pitchLag);
            }
            ELSE
            {
                /* If the second last frame was also lost, it is expected that pastTimeSignal could hold a bit different signal (e.g. including fade-out) from the one stored in TonalMDCTConceal_SaveTimeSignal. */
                /* That is why we reuse the already stored information about the concealed spectrum in the second last frame */
                nSamples = self->nNonZeroSamples;
                move16();
                self->pApplyScaleFactors(self->secondLastPowerSpectrum, self->nSamplesCore, nSamples,
                self->secondLastBlockData.scaleFactors, self->secondLastBlockData.scaleFactors_exp, self->secondLastBlockData.scaleFactors_max_e, powerSpectrum);

                powerSpectrum_exp = getScaleFactor32(powerSpectrum, nSamples);
                powerSpectrum_exp = sub(powerSpectrum_exp, 3); /*extra 3 bits of headroom for MA filter in getEnvelope*/

                /* multFLOAT(powerSpectrum, powerSpectrum, powerSpectrum, nSamples); */
                FOR(i = 0; i < nSamples; i++)
                {
                    Word32 const t = L_shl(powerSpectrum[i], powerSpectrum_exp);
                    powerSpectrum[i] = Mpy_32_32(t, t);
                    move32();
                }

                RefineTonalComponents(self->pTCI->indexOfTonalPeak,
                self->pTCI->lowerIndex,
                self->pTCI->upperIndex,
                self->pTCI->phaseDiff,
                self->pTCI->phase_currentFramePredicted,
                &self->pTCI->numIndexes,
                self->lastPitchLag,
                pitchLag,
                self->lastBlockData.spectralData,
                add(self->lastBlockData.spectralData_exp,self->lastBlockData.gain_tcx_exp),
                self->pApplyScaleFactors,
                self->lastBlockData.scaleFactors,
                self->lastBlockData.scaleFactors_exp,
                self->lastBlockData.scaleFactors_max_e,
                powerSpectrum,
                nSamples
                ,self->nSamplesCore
                ,extract_l(Mpy_32_16_1(L_mult0(self->nSamples,self->nSamples),82)) /* floorPowerSpectrum */
                                     );
            }
        }
    }
    ELSE
    {
        self->pTCI->numIndexes = 0;
        move16();
    }

    *numIndices = self->pTCI->numIndexes;
    move16();


    return TONALMDCTCONCEAL_OK;
}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_InsertNoise( TonalMDCTConcealPtr self,      /*IN */
        Word32* mdctSpectrum,          /*OUT*/
        Word16* mdctSpectrum_exp,      /*OUT*/
        Word8   tonalConcealmentActive,
        Word16* pSeed,                 /*IN/OUT*/
        Word16  tiltCompFactor,
        Word16  crossfadeGain,
        Word16  crossOverFreq)
{
    Word16 i, ld, fac;
    Word16 rnd, exp, exp_last, exp_noise, inv_samples, inv_exp;
    Word16 g, tiltFactor, tilt, tmp;
    Word32 nrgNoiseInLastFrame, nrgWhiteNoise, L_tmp, L_tmp2;



    g = sub(FL2WORD16(1.0f),crossfadeGain);

    rnd = 1977;
    move16();
    if (self->lastBlockData.blockIsConcealed)
    {
        rnd = *pSeed;
        move16();
    }

    IF (self->lastBlockData.blockIsValid == 0)
    {
        set32_fx(mdctSpectrum, 0, self->nSamples);
        *mdctSpectrum_exp = SPEC_EXP_DEC;
    }
    ELSE
    {
        L_tmp = FL2WORD32(0.375f);
        inv_exp = 15;
        move16();
        inv_samples = Inv16(self->lastBlockData.nSamples, &inv_exp);
        tiltFactor = round_fx(BASOP_Util_fPow(L_max(L_tmp, L_deposit_h(tiltCompFactor)), 0, L_deposit_h(inv_samples),inv_exp, &exp));
        BASOP_SATURATE_WARNING_OFF /*next op may result in 32768*/
        tiltFactor = shl(tiltFactor, exp);
        BASOP_SATURATE_WARNING_ON

        tilt = FL2WORD16(1.0f);
        move16();

        nrgNoiseInLastFrame = L_deposit_h(0);
        nrgWhiteNoise = L_deposit_h(0);
        exp_last = exp_noise = 0;
        move16();
        move16();
        IF (!tonalConcealmentActive)
        {
            ld = sub(14,norm_s(self->lastBlockData.nSamples));
            fac = shr(-32768,ld);

            FOR (i = 0; i < crossOverFreq; i++)
            {
                Word16 x = self->lastBlockData.spectralData[i];
                Word32 y;
                rnd = extract_l(L_mac0(13849, rnd, 31821));
                y = L_mult(tilt,rnd);

                nrgNoiseInLastFrame = L_add(nrgNoiseInLastFrame, Mpy_32_16_1(L_msu(0, x,fac),x));
                x = round_fx(y);
                nrgWhiteNoise = L_add(nrgWhiteNoise, Mpy_32_16_1(L_msu(0, x,fac),x));

                mdctSpectrum[i] = y; /*  15Q16 */               move32();

                tilt = mult_r(tilt,tiltFactor);
            }

            IF (nrgNoiseInLastFrame == 0)
            {
                set32_fx(mdctSpectrum, 0, crossOverFreq);
                *mdctSpectrum_exp = SPEC_EXP_DEC;
            }
            ELSE
            {
                exp_last = add(ld,shl(self->lastBlockData.spectralData_exp,1));
                exp_noise = add(ld,30);

                IF (nrgWhiteNoise > 0)
                {
                    ld = norm_l(nrgNoiseInLastFrame);
                    nrgNoiseInLastFrame = L_shl(nrgNoiseInLastFrame,ld);
                    exp_last = sub(exp_last,ld);
                    ld = norm_l(nrgWhiteNoise);
                    nrgWhiteNoise = L_shl(nrgWhiteNoise,ld);
                    exp_noise = sub(exp_noise,ld);

                    exp = sub(exp_last, exp_noise);

                    IF(nrgNoiseInLastFrame > nrgWhiteNoise)
                    {
                        nrgNoiseInLastFrame = L_shr(nrgNoiseInLastFrame,1);
                        exp = add(exp,1);
                    }
                    tmp = div_l(nrgNoiseInLastFrame,round_fx(nrgWhiteNoise));
                    tmp = Sqrt16(tmp, &exp);
                    g = mult_r(g,tmp);

                    L_tmp = L_deposit_h(0);
                    ld = sub(self->lastBlockData.spectralData_exp, 15);
                    exp = sub(ld, exp);

                    IF(exp > 0)
                    {
                        g = shr(g,exp);
                        *mdctSpectrum_exp = self->lastBlockData.spectralData_exp;
                    }
                    ELSE
                    {
                        crossfadeGain = shl(crossfadeGain,exp);
                        *mdctSpectrum_exp = sub(self->lastBlockData.spectralData_exp,exp);
                    }
                    /*make a headroom for mdct_shaping*/
                    exp = sub(*mdctSpectrum_exp, SPEC_EXP_DEC);
                    /* assert(exp < 0);*/
                    IF(exp < 0)
                    {
                        *mdctSpectrum_exp = SPEC_EXP_DEC;
                    }
                    ELSE
                    {
                        exp = 0;
                    }
                }

                FOR (i = 0; i < crossOverFreq; i++)
                {
                    Word16 const x = self->lastBlockData.spectralData[i];
                    Word32 const y = mdctSpectrum[i];

                    if(g > 0)
                    {
                        L_tmp = Mpy_32_16_1(y,g);
                    }

                    L_tmp2 = L_msu(L_tmp,crossfadeGain,x);
                    if(y > 0)
                    {
                        L_tmp2 = L_mac(L_tmp,crossfadeGain,x);
                    }
                    mdctSpectrum[i] = L_shl(L_tmp2, exp);
                    move32();
                }
            }
            exp = sub(self->lastBlockData.spectralData_exp, sub(*mdctSpectrum_exp,16));
            FOR (i = crossOverFreq; i < self->lastBlockData.nSamples; i++)
            {
                mdctSpectrum[i] = L_shl(L_deposit_l(self->lastBlockData.spectralData[i]), exp);
                move32();
            }
        }
        ELSE
        {
            Word16 l;
            assert(self->pTCI->numIndexes > 0);

            FOR (l = self->pTCI->lowerIndex[0]; l <= self->pTCI->upperIndex[0]; l++)
            {
                mdctSpectrum[l] = L_deposit_l(0);
            }

            ld = sub(14,norm_s(self->lastBlockData.nSamples));
            fac = shr(-32768,ld);
            FOR (l = 0; l < self->pTCI->lowerIndex[0]; l++)
            {
                Word16 x = self->lastBlockData.spectralData[l];
                Word32 y;
                rnd = extract_l(L_mac0(13849, rnd, 31821));
                y = L_mult(tilt,rnd);

                nrgNoiseInLastFrame = L_add(nrgNoiseInLastFrame, Mpy_32_16_1(L_msu(0, x,fac),x));
                x = round_fx(y);
                nrgWhiteNoise = L_add(nrgWhiteNoise, Mpy_32_16_1(L_msu(0, x,fac),x));

                mdctSpectrum[l] = y; /*  15Q16 L_deposit_l(y);*/        move32();

                tilt = mult_r(tilt,tiltFactor);
            }

            FOR (i = 1; i < self->pTCI->numIndexes; i++)
            {
                /*tilt *= (float)pow(tiltFactor, self->pTCI->upperIndex[i-1]-self->pTCI->lowerIndex[i-1]+1);*/
                tmp= round_fx(BASOP_Util_fPow(L_deposit_h(tiltFactor), 0, L_deposit_h(self->pTCI->upperIndex[i-1]-self->pTCI->lowerIndex[i-1]+1),15, &exp));
                tmp = shl(tmp, exp);
                tilt = mult_r(tilt,tmp);

                FOR (l = self->pTCI->lowerIndex[i]; l <= self->pTCI->upperIndex[i]; l++)
                {
                    mdctSpectrum[l] = L_deposit_l(0);
                }

                FOR (l = self->pTCI->upperIndex[i-1]+1; l < self->pTCI->lowerIndex[i]; l++)
                {
                    Word16 x = self->lastBlockData.spectralData[l];
                    Word32 y;
                    rnd = extract_l(L_mac0(13849, rnd, 31821));
                    y = L_mult(tilt,rnd);

                    nrgNoiseInLastFrame = L_add(nrgNoiseInLastFrame, Mpy_32_16_1(L_msu(0, x,fac),x));
                    x = round_fx(y);
                    nrgWhiteNoise = L_add(nrgWhiteNoise, Mpy_32_16_1(L_msu(0, x,fac),x));

                    mdctSpectrum[l] = y; /*  15Q16 L_deposit_l(y);*/    move32();

                    tilt = mult_r(tilt,tiltFactor);
                }
            }

            tmp = round_fx(BASOP_Util_fPow(L_deposit_h(tiltFactor), 0, L_deposit_h(self->pTCI->upperIndex[self->pTCI->numIndexes-1]-self->pTCI->lowerIndex[self->pTCI->numIndexes-1]+1),15, &exp));
            BASOP_SATURATE_WARNING_OFF /*next op may result in 32768*/
            tmp = shl(tmp, exp);
            BASOP_SATURATE_WARNING_ON
            tilt = mult_r(tilt,tmp);

            FOR (l = add(self->pTCI->upperIndex[self->pTCI->numIndexes-1], 1); l < crossOverFreq; l++)
            {
                Word16 x = self->lastBlockData.spectralData[l];
                Word32 y;
                rnd = extract_l(L_mac0(13849, rnd, 31821));
                y = L_mult(tilt,rnd);

                nrgNoiseInLastFrame = L_add(nrgNoiseInLastFrame, Mpy_32_16_1(L_msu(0, x,fac),x));
                x = round_fx(y);
                nrgWhiteNoise = L_add(nrgWhiteNoise, Mpy_32_16_1(L_msu(0, x,fac),x));

                mdctSpectrum[l] = y; /*  15Q16 L_deposit_l(y);*/   move32();

                tilt = mult_r(tilt,tiltFactor);
            }

            IF (nrgNoiseInLastFrame == 0)
            {
                set32_fx(mdctSpectrum, 0, crossOverFreq);
                *mdctSpectrum_exp = SPEC_EXP_DEC;
            }
            ELSE
            {
                exp_last = add(ld,shl(self->lastBlockData.spectralData_exp,1));
                exp_noise = add(ld,shl(15,1));

                ld = norm_l(nrgNoiseInLastFrame);
                nrgNoiseInLastFrame = L_shl(nrgNoiseInLastFrame,ld);
                exp_last = sub(exp_last,ld);
                ld = norm_l(nrgWhiteNoise);
                nrgWhiteNoise = L_shl(nrgWhiteNoise,ld);
                exp_noise = sub(exp_noise,ld);

                exp = sub(exp_last, exp_noise);

                IF(nrgNoiseInLastFrame > nrgWhiteNoise)
                {
                    nrgNoiseInLastFrame = L_shr(nrgNoiseInLastFrame,1);
                    exp = add(exp,1);
                }
                tmp = div_l(nrgNoiseInLastFrame,round_fx(nrgWhiteNoise));
                tmp = Sqrt16(tmp, &exp);
                g = mult_r(g,tmp);

                L_tmp = L_deposit_h(0);
                ld = sub(self->lastBlockData.spectralData_exp, 15);
                exp = sub(ld, exp);
                IF(exp > 0)
                {
                    g = shr(g,exp);
                    *mdctSpectrum_exp = self->lastBlockData.spectralData_exp;
                }
                ELSE {
                    crossfadeGain = shl(crossfadeGain,exp);
                    *mdctSpectrum_exp = sub(self->lastBlockData.spectralData_exp,exp);
                }
                /*make a headroom for mdct_shaping*/
                exp = sub(*mdctSpectrum_exp, SPEC_EXP_DEC);


                IF(exp < 0)
                {
                    *mdctSpectrum_exp = SPEC_EXP_DEC;
                }
                ELSE
                {
                    exp = 0;
                }

                FOR (l = 0; l < self->pTCI->lowerIndex[0]; l++)
                {
                    Word16 const x = self->lastBlockData.spectralData[l];
                    Word32 const y = mdctSpectrum[l];

                    if(g > 0)
                    {
                        L_tmp = Mpy_32_16_1(y,g);
                    }

                    L_tmp2 = L_msu(L_tmp,crossfadeGain,x);
                    if(y > 0)
                    {
                        L_tmp2 = L_mac(L_tmp,crossfadeGain,x);
                    }
                    mdctSpectrum[l] = L_shl(L_tmp2, exp);
                    move32();
                }

                FOR (i = 1; i < self->pTCI->numIndexes; i++)
                {
                    FOR (l = self->pTCI->upperIndex[i-1]+1; l < self->pTCI->lowerIndex[i]; l++)
                    {
                        Word16 const x = self->lastBlockData.spectralData[l];
                        Word32 const y = mdctSpectrum[l];

                        if(g > 0)
                        {
                            L_tmp = Mpy_32_16_1(y,g);
                        }

                        L_tmp2 = L_msu(L_tmp,crossfadeGain,x);
                        if(y > 0)
                        {
                            L_tmp2 = L_mac(L_tmp,crossfadeGain,x);
                        }
                        mdctSpectrum[l] = L_shl(L_tmp2, exp);
                        move32();
                    }
                }

                /* initialize bins of tonal components with zero: basically not
                   necessary, but currently the whole spectrum is rescaled in
                   mdct_noiseShaping() and then there would be a processing of
                   uninitialized values */
                FOR (i = 0; i < self->pTCI->numIndexes; i++)
                {
                    FOR (l = self->pTCI->lowerIndex[i]; l <= self->pTCI->upperIndex[i]; l++)
                    {
                        mdctSpectrum[l] = L_deposit_l(0);
                    }
                }

                FOR (l = add(self->pTCI->upperIndex[self->pTCI->numIndexes-1], 1); l < crossOverFreq; l++)
                {
                    Word16 const x = self->lastBlockData.spectralData[l];
                    Word32 const y = mdctSpectrum[l];

                    if(g > 0)
                    {
                        L_tmp = Mpy_32_16_1(y,g);
                    }

                    L_tmp2 = L_msu(L_tmp,crossfadeGain,x);
                    if(y > 0)
                    {
                        L_tmp2 = L_mac(L_tmp,crossfadeGain,x);
                    }
                    mdctSpectrum[l] = L_shl(L_tmp2, exp);
                    move32();
                }
            }
            exp = sub(self->lastBlockData.spectralData_exp, sub(*mdctSpectrum_exp,16));
            FOR (l = crossOverFreq; l < self->lastBlockData.nSamples; l++)
            {
                mdctSpectrum[l] = L_shl(L_deposit_l(self->lastBlockData.spectralData[l]), exp);
                move32();
            }
        }
    }

    *pSeed = rnd;
    move16();

    return TONALMDCTCONCEAL_OK;
}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_Apply(TonalMDCTConcealPtr self,     /*IN */
        Word32 *mdctSpectrum,         /*IN/OUT*/
        Word16 *mdctSpectrum_exp      /*IN */
                                             )
{
    Word16 i, l, exp;
    Word16 * phaseDiff, * pCurrentPhase;
    Word32 phaseToAdd, currentPhase;
    Word32 powerSpectrum[L_FRAME_MAX];
    Word16 nSamples;



    IF (self->lastBlockData.blockIsValid & self->secondLastBlockData.blockIsValid)
    {
        assert(self->pTCI->numIndexes > 0);

        nSamples = self->nNonZeroSamples;
        move16();
        assert(self->pTCI->upperIndex[self->pTCI->numIndexes-1] < nSamples);


        self->pApplyScaleFactors(self->secondLastPowerSpectrum, self->nSamplesCore, nSamples,
                                 self->secondLastBlockData.scaleFactors, self->secondLastBlockData.scaleFactors_exp, self->secondLastBlockData.scaleFactors_max_e, powerSpectrum);

        phaseDiff = self->pTCI->phaseDiff; /* if multiple frame loss occurs use the phase from the last frame and continue rotating */
        pCurrentPhase = self->pTCI->phase_currentFramePredicted;

        exp = sub(*mdctSpectrum_exp, add(add(self->secondLastPowerSpectrum_exp, add(self->secondLastBlockData.gain_tcx_exp,1)),self->secondLastBlockData.scaleFactors_max_e));

        IF (!self->lastBlockData.blockIsConcealed)
        {
            if (self->secondLastBlockData.tonalConcealmentActive != 0)
            {
                self->nFramesLost = add( self->nFramesLost,1);
            }
            if (self->secondLastBlockData.tonalConcealmentActive == 0)
            {
                self->nFramesLost = 2;
                move16();
            }
        }
        pCurrentPhase = self->pTCI->phase_currentFramePredicted;
        /* for each index group */
        FOR (i = 0; i < self->pTCI->numIndexes; i++)
        {
            /*phaseToAdd = self->nFramesLost*phaseDiff[i];     */
            phaseToAdd = L_mult0(self->nFramesLost,phaseDiff[i]);      /*2Q13 * Q0=Q13*/
            /* Move phaseToAdd to range -PI..PI */

            WHILE (L_sub(phaseToAdd, FL2WORD32_SCALE(EVS_PI,31-13)) > 0)
            {
                phaseToAdd = L_sub(phaseToAdd, FL2WORD32_SCALE(2*EVS_PI,31-13));
            }
            WHILE (L_sub(phaseToAdd, FL2WORD32_SCALE(-EVS_PI,31-13))  < 0)
            {
                phaseToAdd = L_add(phaseToAdd, FL2WORD32_SCALE(2*EVS_PI,31-13));
            }

            FOR (l = self->pTCI->lowerIndex[i]; l <= self->pTCI->upperIndex[i]; l++)
            {
                /* *pCurrentPhase and phaseToAdd are in range -PI..PI */
                currentPhase = L_mac0(phaseToAdd, (*pCurrentPhase++), 1);                 /*2Q13+2Q13=3Q13*/

                if (L_sub(currentPhase, FL2WORD32_SCALE(EVS_PI,31-13)) > 0)
                {
                    currentPhase = L_sub(currentPhase, FL2WORD32_SCALE(2*EVS_PI,31-13));
                }
                if (L_sub(currentPhase, FL2WORD32_SCALE(-EVS_PI,31-13))  < 0)
                {
                    currentPhase = L_add(currentPhase, FL2WORD32_SCALE(2*EVS_PI,31-13));
                }
                /* getCosWord16 returns 1Q14*/
                mdctSpectrum[l] = Mpy_32_16_1(powerSpectrum[l],getCosWord16(extract_l(currentPhase)));
                move32();
                mdctSpectrum[l] = L_shr(mdctSpectrum[l], exp);
            }
        }
    }

    self->nFramesLost = add(self->nFramesLost,1);
    move16();

    return TONALMDCTCONCEAL_OK;
}

TONALMDCTCONCEAL_ERROR TonalMDCTConceal_SaveTimeSignal( TonalMDCTConcealPtr self,
        Word16*             timeSignal,
        Word16              nNewSamples
                                                      )
{
    IF (sub(nNewSamples,self->nSamples) == 0)
    {
        assert(nNewSamples <= L_FRAME_MAX);
        IF (!self->secondLastBlockData.tonalConcealmentActive)
        {
            Copy(self->lastPcmOut, self->secondLastPcmOut, self->nSamples);
        }
        Copy(timeSignal, self->lastPcmOut, self->nSamples);
    }

    return TONALMDCTCONCEAL_OK;
}
static void CalcPowerSpec(Word32 * mdctSpec,                /* i: MDCT spectrum Q31,mdctSpec_exp          */
                          Word16 mdctSpec_exp,              /* i: exponent of MDCT spectrum               */
                          Word32 * mdstSpec,                /* i: MDST spectrum Q31,mdstSpec_exp          */
                          Word16 mdstSpec_exp,              /* i: exponent of MDST spectrum               */
                          Word16 nSamples,                  /* i: frame size                              */
                          Word16 floorPowerSpectrum,        /* i: lower limit for power spectrum bins Q0  */
                          Word32 * powerSpec,               /* o: power spectrum                          */
                          Word16 * powerSpec_exp)           /* o: exponent of power spectrum              */
{
    Word16 k, s1, s2, tmp;
    Word32 x, L_tmp, L_tmp_floor;


    k = s_max(mdctSpec_exp, mdstSpec_exp);
    *powerSpec_exp = add(add(k, k), 3); /*extra 3 bits of headroom for MA filter in getEnvelope*/ move16();
    s1 = sub(*powerSpec_exp, add(mdctSpec_exp, mdctSpec_exp));
    s2 = sub(*powerSpec_exp, add(mdstSpec_exp, mdstSpec_exp));

    k = sub(31, *powerSpec_exp);
    /* If the signal is bellow floor, special care is needed for *powerSpec_exp */
    IF (sub(add(16-3, norm_s(floorPowerSpectrum)), k) < 0) /*extra 3 bits of headroom for MA filter in getEnvelope*/
    {
        k = sub(k, add(16-3, norm_s(floorPowerSpectrum))); /*extra 3 bits of headroom for MA filter in getEnvelope*/
        *powerSpec_exp = add(*powerSpec_exp, k);
        s1 = add(s1, k);
        s2 = add(s2, k);
        k = add(16-3, norm_s(floorPowerSpectrum));
    }
    L_tmp_floor = L_shl(L_deposit_l(floorPowerSpectrum), k);

    tmp = sub(nSamples, 2);
    FOR (k = 1; k <= tmp; k++)
    {
        x = Mpy_32_32(mdctSpec[k], mdctSpec[k]); /*Q31,2*mdctSpec_exp*/

        L_tmp = Mpy_32_32(mdstSpec[k], mdstSpec[k]); /*Q31,2*mdstSpec_exp*/
        x = L_add(L_shr(x,s1), L_shr(L_tmp,s2)); /*Q31,*powerSpec_exp*/

        powerSpec[k] = L_max(L_tmp_floor, x);
        move32();
    }

    powerSpec[0] = L_shr(powerSpec[1], 1);
    move32();
    powerSpec[nSamples-1] = L_shr(powerSpec[nSamples-2], 1);
    move32();

}
