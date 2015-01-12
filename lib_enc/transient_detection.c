/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* Exponent of attack threshold. Picked according to current threshold values. */
#define ATTACKTHRESHOLD_E      4
/* Exponent of subblock energies and accumulated subblock energies.
   The current value of 2 does not prevent saturations to happen in all cases. */
#define SUBBLOCK_NRG_E         4
/* Exponent of the subblock energy change.
   This value is coupled to the transient detector API. */
#define SUBBLOCK_NRG_CHANGE_E  NRG_CHANGE_E

#define MIN_BLOCK_ENERGY ((Word32)1)


/************************************************/
/*                                              */
/*        Internal functions prototypes         */
/*                                              */
/************************************************/

static void InitDelayBuffer(Word16 nFrameLength, Word16 nDelay, DelayBuffer * pDelayBuffer);
static void InitSubblockEnergies(Word16 nFrameLength, Word16 nDelay, DelayBuffer * pDelayBuffer, SubblockEnergies * pSubblockEnergies);
static void InitTransientDetector(SubblockEnergies * pSubblockEnergies, Word16 nDelay, Word16 nSubblocksToCheck,
                                  TCheckSubblocksForAttack pCheckSubblocksForAttack,
                                  Word16 attackRatioThreshold, TransientDetector * pTransientDetector);
static void UpdateDelayBuffer(Word16 const * input, Word16 nSamplesAvailable, DelayBuffer * pDelayBuffer);
static void HighPassFilter(Word16 const * input, Word16 length, Word16 * pFirState1, Word16 * pFirState2, Word16 * output);
static void UpdateSubblockEnergies(Word16 const * input, Word16 nSamplesAvailable, SubblockEnergies * pSubblockEnergies);
static void CalculateSubblockEnergies(Word16 const * input, Word16 nSamplesAvailable, SubblockEnergies * pSubblockEnergies);
static void RunTransientDetector(TransientDetector * pTransientDetector);

/************************************************/
/*                                              */
/*  Functions that define transient detectors   */
/*                                              */
/************************************************/

/** TCX decision.
  * Check IF there is an attack in a subblock. Version FOR TCX Long/Short decision.
  * See TCheckSubblocksForAttack FOR definition of parameters.
  * It is assumed that the delay of MDCT overlap was not taken into account, so that the last subblock corresponds to the newest input subblock.
  */
static void GetAttackForTCXDecision(Word32 const * pSubblockNrg, Word32 const * pAccSubblockNrg, Word16 nSubblocks, Word16 nPastSubblocks, Word16 attackRatioThreshold, Word16 * pbIsAttackPresent, Word16 * pAttackIndex)
{
    Word16 i;
    Word16 bIsAttackPresent, attackIndex;
    Word16 attackRatioThreshold_1_5;

    (void)nPastSubblocks;
    (void)nSubblocks;
    assert(nSubblocks >= NSUBBLOCKS);
    assert(nPastSubblocks >= 2);

    /* attackRatioThreshold_1_5 = attackRatioThreshold * 1.5, exponent is ATTACKTHRESHOLD_E+1 */
    attackRatioThreshold_1_5 = add(shr(attackRatioThreshold,2),shr(attackRatioThreshold,1));

    move16();
    move16();
    bIsAttackPresent = FALSE;
    attackIndex = 0;
    /* Search for the last attack in the subblocks */
    if ( s_or(L_sub(L_shr(pSubblockNrg[-1],ATTACKTHRESHOLD_E), Mpy_32_16_1(pAccSubblockNrg[-1], attackRatioThreshold)) > 0,
              L_sub(L_shr(pSubblockNrg[-2],ATTACKTHRESHOLD_E), Mpy_32_16_1(pAccSubblockNrg[-2], attackRatioThreshold)) > 0) )
    {
        move16();
        bIsAttackPresent = TRUE;
    }

    FOR (i = 0; i < NSUBBLOCKS; i++)
    {
        IF ( L_sub(L_shr(pSubblockNrg[i],ATTACKTHRESHOLD_E), Mpy_32_16_1(pAccSubblockNrg[i], attackRatioThreshold)) > 0 )
        {
            if (i < 6)
            {
                move16();
                bIsAttackPresent = TRUE;
            }

            if ( s_and(sub(attackIndex,2) != 0, sub(attackIndex,6) != 0) )
            {
                move16();
                attackIndex = i;
            }
        }
        ELSE     /* no attack, but set index anyway in case of strong energy increase */
        {
            IF ( s_and(( L_sub(L_shr(pSubblockNrg[i],1+ATTACKTHRESHOLD_E), Mpy_32_16_1(pSubblockNrg[sub(i,1)], attackRatioThreshold_1_5)) > 0 ),
            ( L_sub(L_shr(pSubblockNrg[i],1+ATTACKTHRESHOLD_E), Mpy_32_16_1(pSubblockNrg[sub(i,2)], attackRatioThreshold_1_5)) > 0 )) )
            {

                if ( s_and(sub(attackIndex,2) != 0, sub(attackIndex,6) != 0) )
                {
                    move16();
                    attackIndex = i;
                }
            }
        }
    }
    /* avoid post-echos on click sounds (very short transients) due to TNS aliasing */
    if ( sub(attackIndex,4) == 0 )
    {
        move16();
        attackIndex = 7;
    }
    if ( sub(attackIndex,5) == 0 )
    {
        move16();
        attackIndex = 6;
    }

    move16();
    move16();
    *pAttackIndex = attackIndex;
    *pbIsAttackPresent = bIsAttackPresent;

}


/** Initialize TCX transient detector.
  * See InitTransientDetector for definition of parameters.
  */
static void InitTCXTransientDetector(Word16 nDelay, SubblockEnergies * pSubblockEnergies, TransientDetector * pTransientDetector)
{
    InitTransientDetector(pSubblockEnergies, nDelay, NSUBBLOCKS, GetAttackForTCXDecision, FL2WORD16(8.5f/(1<<ATTACKTHRESHOLD_E)), pTransientDetector);
}


/************************************************/
/*                                              */
/*              Interface functions             */
/*                                              */
/************************************************/

void InitTransientDetection(Word16 nFrameLength,
                            Word16 nTCXDelay,
                            TransientDetection * pTransientDetection)
{
    /* Init the delay buffer. */
    InitDelayBuffer(nFrameLength, nTCXDelay, &pTransientDetection->delayBuffer);
    /* Init a subblock energies buffer used for the TCX Short/Long decision. */
    InitSubblockEnergies(nFrameLength, nTCXDelay, &pTransientDetection->delayBuffer, &pTransientDetection->subblockEnergies);
    /* Init the TCX Short/Long transient detector. */
    InitTCXTransientDetector(nTCXDelay, &pTransientDetection->subblockEnergies, &pTransientDetection->transientDetector);
    /* We need two past subblocks for the TCX TD and NSUBBLOCKS+1 for the temporal flatness measure FOR the TCX LTP. */
    pTransientDetection->transientDetector.pSubblockEnergies->nDelay =
        add(pTransientDetection->transientDetector.pSubblockEnergies->nDelay, NSUBBLOCKS+1);
}

/**
 * \brief Calculate average of temporal energy change.
 * \return average temporal energy change with exponent = 8
 */
Word16 GetTCXAvgTemporalFlatnessMeasure(struct TransientDetection const * pTransientDetection, Word16 nCurrentSubblocks, Word16 nPrevSubblocks)
{
    Word16 i;
    TransientDetector const * pTransientDetector;
    SubblockEnergies const * pSubblockEnergies;
    Word16 nDelay;
    Word16 nRelativeDelay;
    Word16 const * pSubblockNrgChange;
    Word32 sumTempFlatness;
    Word16 nTotBlocks;


    /* Initialization */
    pTransientDetector = &pTransientDetection->transientDetector;
    pSubblockEnergies = pTransientDetector->pSubblockEnergies;
    move16();
    nDelay = pTransientDetector->nDelay;
    nRelativeDelay = sub(pSubblockEnergies->nDelay, nDelay);
    pSubblockNrgChange = NULL;
    nTotBlocks = add(nCurrentSubblocks,nPrevSubblocks);

    assert(nTotBlocks > 0);

    sumTempFlatness = L_deposit_l(0);

    assert((nPrevSubblocks <= nRelativeDelay) && (nCurrentSubblocks <= NSUBBLOCKS+nDelay));

    move16();
    pSubblockNrgChange = &pSubblockEnergies->subblockNrgChange[sub(nRelativeDelay,nPrevSubblocks)];

    FOR (i = 0; i < nTotBlocks; i++)
    {
        sumTempFlatness = L_add(sumTempFlatness, L_deposit_l(pSubblockNrgChange[i]));
    }

    /* exponent = AVG_FLAT_E */
    i = div_l(L_shl(sumTempFlatness,16-15+SUBBLOCK_NRG_CHANGE_E-AVG_FLAT_E), nTotBlocks);

    return i;
}

Word16 GetTCXMaxenergyChange(struct TransientDetection const * pTransientDetection,
                             const Word8 isTCX10,
                             const Word16 nCurrentSubblocks, const Word16 nPrevSubblocks)
{
    Word16 i;
    TransientDetector const * pTransientDetector;
    SubblockEnergies const * pSubblockEnergies;
    Word16 nDelay;
    Word16 nRelativeDelay;
    Word16 const * pSubblockNrgChange;
    Word16 maxEnergyChange;
    Word16 nTotBlocks;


    pTransientDetector = &pTransientDetection->transientDetector;
    pSubblockEnergies = pTransientDetector->pSubblockEnergies;
    move16();
    nDelay = pTransientDetector->nDelay;
    nRelativeDelay = sub(pSubblockEnergies->nDelay, nDelay);
    pSubblockNrgChange = NULL;
    nTotBlocks = nCurrentSubblocks+nPrevSubblocks;

    assert(nTotBlocks > 0);
    maxEnergyChange = FL2WORD16_SCALE(0.0f, SUBBLOCK_NRG_CHANGE_E);

    assert((nPrevSubblocks <= nRelativeDelay) && (nCurrentSubblocks <= NSUBBLOCKS+nDelay));
    pSubblockNrgChange = &pSubblockEnergies->subblockNrgChange[nRelativeDelay-nPrevSubblocks];

    IF ( s_or(pTransientDetector->bIsAttackPresent, isTCX10) )    /* frame is TCX-10 */
    {
        Word32 const * pSubblockNrg = &pSubblockEnergies->subblockNrg[sub(nRelativeDelay,nPrevSubblocks)];
        Word32 nrgMin, nrgMax;
        Word16 idxMax = 0;

        move16();

        nrgMax = L_add(pSubblockNrg[0], 0);

        /* find subblock with maximum energy */
        FOR (i = 1; i < nTotBlocks; i++)
        {
            if ( L_sub(nrgMax, pSubblockNrg[i]) < 0 )
            {
                idxMax = i;
                move16();
            }
            nrgMax = L_max(nrgMax, pSubblockNrg[i]);
        }

        nrgMin = L_add(nrgMax, 0);

        /* find minimum energy after maximum */
        FOR (i = idxMax + 1; i < nTotBlocks; i++)
        {
            nrgMin = L_min(nrgMin, pSubblockNrg[i]);
        }
        /* lower maxEnergyChange if energy doesn't decrease much after energy peak */
        /* if (nrgMin > 0.375f * nrgMax) */
        if ( 0 > L_sub(Mpy_32_16_1(nrgMax, FL2WORD16(0.375f)), nrgMin) )
        {
            nTotBlocks = sub(idxMax, 3);
        }
    }

    FOR (i = 0; i < nTotBlocks; i++)
    {
        maxEnergyChange = s_max(maxEnergyChange, pSubblockNrgChange[i]);
    }

    move16();
    i = maxEnergyChange;

    return i;
}

void RunTransientDetection(Word16 const * input, Word16 nSamplesAvailable, TransientDetection * pTransientDetection)
{
    Word16 filteredInput[L_FRAME48k];
    SubblockEnergies * pSubblockEnergies = &pTransientDetection->subblockEnergies;
    TransientDetector * pTransientDetector = &pTransientDetection->transientDetector;

    assert((input != NULL) && (pTransientDetection != NULL) && (pSubblockEnergies != NULL) && (pTransientDetector != NULL));

    HighPassFilter(input, nSamplesAvailable, &pSubblockEnergies->firState1, &pSubblockEnergies->firState2, filteredInput);

    /* Update subblock energies. */
    UpdateSubblockEnergies(filteredInput, nSamplesAvailable, pSubblockEnergies);

    /* Run transient detectors. */
    RunTransientDetector(pTransientDetector);

    /* Update the delay buffer. */
    UpdateDelayBuffer(filteredInput, nSamplesAvailable, &pTransientDetection->delayBuffer);
}

void SetTCXModeInfo(Encoder_State_fx *st,
                    TransientDetection const * pTransientDetection,
                    Word16 * tcxModeOverlap)
{
    assert(pTransientDetection != NULL);

    IF( sub(st->codec_mode,MODE2) == 0 )
    {

        /* determine window sequence (1 long or 2 short windows) */

        test();
        IF ( st->tcx10Enabled != 0 && st->tcx20Enabled != 0 )
        {
            /* window switching based on transient detector output */
            test();
            test();
            test();
            IF ( ((pTransientDetection->transientDetector.bIsAttackPresent != 0)
                  ||  (L_sub(Mpy_32_16_1(st->currEnergyHF_fx, FL2WORD16(1.0f/39.0f)), st->prevEnergyHF_fx) > 0))
                 && ((sub(st->last_core_fx, ACELP_CORE) != 0) && (sub(st->last_core_fx, AMR_WB_CORE) != 0)) )
            {
                move16();
                st->tcxMode = TCX_10;
            }
            ELSE
            {
                move16();
                st->tcxMode = TCX_20;
            }
        }
        ELSE
        {
            /* window selection (non-adaptive) based on flags only */
            IF (st->tcx10Enabled)
            {
                move16();
                st->tcxMode = TCX_10;
            }
            ELSE IF (st->tcx20Enabled)
            {
                move16();
                st->tcxMode = TCX_20;
            }
            ELSE {
                move16();
                st->tcxMode = NO_TCX;
            }
        }
        test();
        test();
        IF (st->last_core_fx == ACELP_CORE || st->last_core_fx == AMR_WB_CORE)
        {
            move16();
            st->tcx_cfg.tcx_last_overlap_mode = TRANSITION_OVERLAP;
        }
        ELSE IF ( (sub(st->tcxMode, TCX_10) == 0) && (sub(st->tcx_cfg.tcx_curr_overlap_mode, ALDO_WINDOW) == 0))
        {
            move16();
            st->tcx_cfg.tcx_last_overlap_mode = FULL_OVERLAP;
        }
        ELSE
        {
            move16();
            st->tcx_cfg.tcx_last_overlap_mode = st->tcx_cfg.tcx_curr_overlap_mode;
        }

        /* determine window overlaps (0 full, 2 none, or 3 half) */

        IF ( sub(st->tcxMode, TCX_10) == 0 )
        {
            IF( pTransientDetection->transientDetector.attackIndex < 0)
            {
                move16();
                *tcxModeOverlap = HALF_OVERLAP;
            }
            ELSE
            {
                move16();
                *tcxModeOverlap = s_and(pTransientDetection->transientDetector.attackIndex, 3);
                if ( sub(*tcxModeOverlap,1) == 0 )
                {
                    move16();
                    *tcxModeOverlap = FULL_OVERLAP;
                }
            }
        }
        ELSE IF ( sub(st->tcxMode, TCX_20) == 0 )
        {
            IF (sub(pTransientDetection->transientDetector.attackIndex, 7) == 0)
            {
                move16();
                *tcxModeOverlap = HALF_OVERLAP;
            }
            ELSE IF (sub(pTransientDetection->transientDetector.attackIndex, 6) == 0)
            {
                move16();
                *tcxModeOverlap = MIN_OVERLAP;
            }
            ELSE
            {
                move16();
                *tcxModeOverlap = ALDO_WINDOW;
            }
        }
        ELSE                      /* NO_TCX */
        {
            move16();
            *tcxModeOverlap = TRANSITION_OVERLAP;
        }
        test();
        if ((sub(st->tcx_cfg.tcx_last_overlap_mode, TRANSITION_OVERLAP) == 0) && (sub(*tcxModeOverlap, ALDO_WINDOW) == 0))
        {
            move16();
            *tcxModeOverlap = FULL_OVERLAP;
        }

        /* Sanity check */
        assert(*tcxModeOverlap !=1);

    }

}

/************************************************/
/*                                              */
/*              Internal functions              */
/*                                              */
/************************************************/

static void InitDelayBuffer(Word16 nFrameLength, Word16 nDelay, DelayBuffer * pDelayBuffer)
{
    Word16 const nMaxBuffSize = sizeof(pDelayBuffer->buffer)/sizeof(pDelayBuffer->buffer[0]);


    move16();
    move16();
    assert((nFrameLength > NSUBBLOCKS) && (nFrameLength % NSUBBLOCKS == 0) && (nDelay >= 0) && (pDelayBuffer != NULL));
    pDelayBuffer->nSubblockSize = nFrameLength/NSUBBLOCKS;
    assert(pDelayBuffer->nSubblockSize <= nMaxBuffSize);
    set16_fx(pDelayBuffer->buffer, 0, nMaxBuffSize);
    pDelayBuffer->nDelay = nDelay % pDelayBuffer->nSubblockSize;
    assert(pDelayBuffer->nDelay <= nMaxBuffSize);

}

static void InitSubblockEnergies(Word16 nFrameLength, Word16 nDelay, DelayBuffer * pDelayBuffer, SubblockEnergies * pSubblockEnergies)
{
    Word16 const nMaxBuffSize = sizeof(pSubblockEnergies->subblockNrg)/sizeof(pSubblockEnergies->subblockNrg[0]);
    (void)nFrameLength;


    assert((pDelayBuffer != NULL) && (pSubblockEnergies != NULL) && (pDelayBuffer->nSubblockSize * NSUBBLOCKS == nFrameLength) && (pDelayBuffer->nSubblockSize > 0));

    set32_fx(pSubblockEnergies->subblockNrg,    MIN_BLOCK_ENERGY, nMaxBuffSize);
    set32_fx(pSubblockEnergies->accSubblockNrg, MIN_BLOCK_ENERGY, nMaxBuffSize+1);
    set16_fx(pSubblockEnergies->subblockNrgChange, 0x7fff, nMaxBuffSize);
    pSubblockEnergies->nDelay = nDelay / pDelayBuffer->nSubblockSize;
    assert(pSubblockEnergies->nDelay < nMaxBuffSize);
    pSubblockEnergies->nPartialDelay = nDelay % pDelayBuffer->nSubblockSize;
    pSubblockEnergies->facAccSubblockNrg = FL2WORD16(0.8125f); /* Energy accumulation factor */
    pSubblockEnergies->firState1 = 0;
    pSubblockEnergies->firState2 = 0;

    pSubblockEnergies->pDelayBuffer = pDelayBuffer;
    pDelayBuffer->nDelay = s_max(pDelayBuffer->nDelay, pSubblockEnergies->nPartialDelay);

}

/** Init transient detector.
  * Fills TransientDetector structure with sensible content and enable it.
  * @param pSubblockEnergies Subblock energies used in this transient detector.
  * @param nDelay Delay FOR this transient detector.
  * @param nSubblocksToCheck Number of subblocks to check in this transient detector.
  * @param pCheckSubblockForAttack Attack detection function FOR this transient detector.
  * @param pSetAttackPosition Function FOR finalizing this transient detector.
  * @param attackRatioThreshold Attack ratio threshold with exponent ATTACKTHRESHOLD_E.
  * @param pTransientDetector Structure to be initialized.
  */
static void InitTransientDetector(SubblockEnergies * pSubblockEnergies, Word16 nDelay, Word16 nSubblocksToCheck,
                                  TCheckSubblocksForAttack pCheckSubblocksForAttack,
                                  Word16 attackRatioThreshold, TransientDetector * pTransientDetector)
{
    Word16 nMaxBuffSize;

    move16();
    nMaxBuffSize = sizeof(pSubblockEnergies->subblockNrg)/sizeof(pSubblockEnergies->subblockNrg[0]);

    assert((pSubblockEnergies != NULL) && (pSubblockEnergies->pDelayBuffer != NULL) && (pTransientDetector != NULL) && (pSubblockEnergies->pDelayBuffer->nSubblockSize != 0));
    pTransientDetector->pSubblockEnergies = pSubblockEnergies;
    pTransientDetector->nDelay = (nDelay - pSubblockEnergies->nPartialDelay) / pSubblockEnergies->pDelayBuffer->nSubblockSize;
    assert(nDelay == pTransientDetector->nDelay * pSubblockEnergies->pDelayBuffer->nSubblockSize + pSubblockEnergies->nPartialDelay);
    assert(pTransientDetector->nDelay < nMaxBuffSize);
    pSubblockEnergies->nDelay = s_max(pSubblockEnergies->nDelay, pTransientDetector->nDelay);
    assert(nSubblocksToCheck <= NSUBBLOCKS + pTransientDetector->nDelay);
    pTransientDetector->nSubblocksToCheck = nSubblocksToCheck;
    pTransientDetector->CheckSubblocksForAttack = pCheckSubblocksForAttack;
    pTransientDetector->attackRatioThreshold = attackRatioThreshold;
    pTransientDetector->bIsAttackPresent = FALSE;
    pTransientDetector->attackIndex = -1;
}

/* This function should be inlined and WMOPS instrumentation takes that into account, meaning that all references are considered as local variables */
static Word32 InlineFilter(Word16 inValue, Word16 firState1, Word16 firState2)
{
    /*  0.375f * inValue - 0.5f * firState1 + 0.125f * firState2 */

    return L_msu(L_mac(L_mult(firState2, FL2WORD16(0.125f)), inValue, FL2WORD16(0.375f)), firState1, FL2WORD16(0.5f));
}

static void HighPassFilter(Word16 const * input, Word16 length, Word16 * pFirState1, Word16 * pFirState2, Word16 * output)
{
    Word16 i;

    output[0] = round_fx(InlineFilter(input[0], *pFirState1, *pFirState2));
    output[1] = round_fx(InlineFilter(input[1], input[0], *pFirState1));

    FOR (i = 2; i < length; i++)
    {
        output[i] = round_fx(InlineFilter(input[i], input[i-1], input[i-2]));
    }

    /* update filter states: shift time samples through delay line */
    move16();
    move16();
    *pFirState2 = input[length-2];
    *pFirState1 = input[length-1];

}

static void RunTransientDetector(TransientDetector * pTransientDetector)
{
    Word16 const attackRatioThreshold = pTransientDetector->attackRatioThreshold;
    SubblockEnergies const * pSubblockEnergies = pTransientDetector->pSubblockEnergies;
    Word16 const nDelay = pTransientDetector->nDelay;
    Word16 const nRelativeDelay = pSubblockEnergies->nDelay - nDelay;
    Word32 const * pSubblockNrg = &pSubblockEnergies->subblockNrg[nRelativeDelay];
    Word32 const * pAccSubblockNrg = &pSubblockEnergies->accSubblockNrg[nRelativeDelay];

    assert((pTransientDetector->CheckSubblocksForAttack != NULL));

    pTransientDetector->CheckSubblocksForAttack(pSubblockNrg, pAccSubblockNrg,
            NSUBBLOCKS+nDelay, nRelativeDelay,
            attackRatioThreshold,
            &pTransientDetector->bIsAttackPresent, &pTransientDetector->attackIndex);

}

static void UpdateDelayBuffer(Word16 const * input, Word16 nSamplesAvailable, DelayBuffer * pDelayBuffer)
{
    Word16 i;
    Word16 nDelay;


    move16();
    nDelay = pDelayBuffer->nDelay;

    assert((nDelay >= 0) && (nDelay <= (int)sizeof(pDelayBuffer->buffer)/(int)sizeof(pDelayBuffer->buffer[0])));
    assert(nSamplesAvailable <= NSUBBLOCKS*pDelayBuffer->nSubblockSize);
    /* If this is not the last frame */
    IF (nSamplesAvailable == NSUBBLOCKS*pDelayBuffer->nSubblockSize)
    {
        /* Store the newest samples into the delay buffer */
        FOR (i = 0; i < nDelay; i++)
        {
            move16();
            pDelayBuffer->buffer[i] = input[i+nSamplesAvailable-nDelay];
        }
    }

}

static void UpdateSubblockEnergies(Word16 const * input, Word16 nSamplesAvailable, SubblockEnergies * pSubblockEnergies)
{
    Word16 i;


    assert((pSubblockEnergies->nDelay >= 0) && (pSubblockEnergies->nDelay+NSUBBLOCKS <= (int)sizeof(pSubblockEnergies->subblockNrg)/(int)sizeof(pSubblockEnergies->subblockNrg[0])));
    assert(pSubblockEnergies->nPartialDelay <= pSubblockEnergies->pDelayBuffer->nDelay);
    /* At least one block delay is required when subblock energy change is required */
    assert(pSubblockEnergies->nDelay >= 1);

    /* Shift old subblock energies */
    FOR (i = 0; i < pSubblockEnergies->nDelay; i++)
    {
        move32();
        move32();
        move16();
        pSubblockEnergies->subblockNrg[i] = pSubblockEnergies->subblockNrg[i+NSUBBLOCKS];
        pSubblockEnergies->accSubblockNrg[i] = pSubblockEnergies->accSubblockNrg[i+NSUBBLOCKS];
        pSubblockEnergies->subblockNrgChange[i] = pSubblockEnergies->subblockNrgChange[i+NSUBBLOCKS];
    }

    /* Compute filtered subblock energies for the new samples */
    CalculateSubblockEnergies(input, nSamplesAvailable, pSubblockEnergies);

}

/* This function should be inlined and WMOPS instrumentation takes that into account, meaning that all references are considered as local variables */
static void UpdatedAndStoreAccWindowNrg(Word32 newWindowNrgF, Word32 * pAccSubblockNrg, Word16 facAccSubblockNrg, Word32 * pOutAccWindowNrgF)
{
    /* Store the accumulated energy */
    move32();
    *pOutAccWindowNrgF = *pAccSubblockNrg;
    /* Update the accumulated energy: maximum of the current and the accumulated energy */
    *pAccSubblockNrg = Mpy_32_16_1(*pAccSubblockNrg, facAccSubblockNrg);

    if ( L_sub(newWindowNrgF, *pAccSubblockNrg) > 0 )
    {
        move32();
        *pAccSubblockNrg = newWindowNrgF;
    }
}

static void CalculateSubblockEnergies(Word16 const * input, Word16 nSamplesAvailable, SubblockEnergies * pSubblockEnergies)
{
    DelayBuffer * pDelayBuffer;
    Word16 nSubblockSize;
    Word16 nDelay;
    Word16 nPartialDelay;
    Word16 * delayBuffer;
    Word16 facAccSubblockNrg;
    Word32 * pSubblockNrg;
    Word32 * pAccSubblockNrg;
    Word16 * pSubblockNrgChange;
    Word32 * pAccSubblockTmp;
    Word16 nWindows;
    Word16 w, k, k2, tmp;
    Word16 firState1, firState2;
    Word32 w0, w1;
    Word32 accu;

    move16();
    pDelayBuffer = pSubblockEnergies->pDelayBuffer;
    facAccSubblockNrg = pSubblockEnergies->facAccSubblockNrg;

    move16();
    move16();
    move16();
    nSubblockSize = pDelayBuffer->nSubblockSize;
    nDelay = pSubblockEnergies->nDelay;
    nPartialDelay = pSubblockEnergies->nPartialDelay;

    delayBuffer = &pDelayBuffer->buffer[sub(pDelayBuffer->nDelay, nPartialDelay)];
    pSubblockNrg = &pSubblockEnergies->subblockNrg[nDelay];
    pAccSubblockNrg = &pSubblockEnergies->accSubblockNrg[nDelay];
    pSubblockNrgChange = &pSubblockEnergies->subblockNrgChange[nDelay];

    move16();
    move16();
    /* nWindows = (nSamplesAvailable + nPartialDelay) / nSubblockSize */
    nWindows = shr(div_s( add(nSamplesAvailable,nPartialDelay), shl(nSubblockSize, 7)),8);
    firState1 = pSubblockEnergies->firState1;
    firState2 = pSubblockEnergies->firState2;
    pAccSubblockTmp = &pAccSubblockNrg[nWindows];

    IF (nWindows > 0)
    {
        /* Process left over samples from the previous frame. */
        accu = L_add(MIN_BLOCK_ENERGY, 0);
        assert((SUBBLOCK_NRG_E & 1) == 0);
        FOR (k = 0; k < nPartialDelay; k++)
        {
            tmp = shr(delayBuffer[k], SUBBLOCK_NRG_E/2);
            accu = L_mac0(accu, tmp, tmp);
        }

        /* Process new samples in the 0. subblock. */
        w = sub(nSubblockSize,nPartialDelay);
        assert((SUBBLOCK_NRG_E & 1) == 0);
        FOR (k = 0; k < w; k++)
        {
            tmp = shr(input[k], SUBBLOCK_NRG_E/2);
            accu = L_mac0(accu, tmp, tmp);
        }

        move32();
        pSubblockNrg[0] = accu;
        /* Set accumulated subblock energy at this point. */
        UpdatedAndStoreAccWindowNrg(pSubblockNrg[0], pAccSubblockTmp, facAccSubblockNrg, &pAccSubblockNrg[0]);

        FOR (w = 1; w < nWindows; w++)
        {
            accu = L_add(MIN_BLOCK_ENERGY, 0);
            /* Process new samples in the w. subblock. */
            k2 = add(k, nSubblockSize);
            assert((SUBBLOCK_NRG_E & 1) == 0);
            FOR (; k < k2; k++)
            {
                tmp = shr(input[k], SUBBLOCK_NRG_E/2);
                accu = L_mac0(accu, tmp, tmp);
            }
            move32();
            pSubblockNrg[w] = accu;
            /* Set accumulated subblock energy at this point. */
            UpdatedAndStoreAccWindowNrg(pSubblockNrg[w], pAccSubblockTmp, facAccSubblockNrg, &pAccSubblockNrg[w]);
        }

        /* Calculate energy change for each block. */
        FOR (w = 0; w < nWindows; w++)
        {
            w0 = L_add(pSubblockNrg[w], 0);
            w1 = L_add(pSubblockNrg[sub(w,1)], 0);

            IF ( L_sub(w0, w1) >  0 )
            {
                k2 = BASOP_Util_Divide3232_uu_1616_Scale(w0, w1, &k);
            }
            ELSE
            {
                k2 = BASOP_Util_Divide3232_uu_1616_Scale(w1, w0, &k);
            }
            move16();
            pSubblockNrgChange[w] = MAX_16;
            IF ( sub(k,SUBBLOCK_NRG_CHANGE_E) < 0 )
            {
                move16();
                pSubblockNrgChange[w] = shr(k2, sub(SUBBLOCK_NRG_CHANGE_E,k));
            }
        }
    }

    move16();
    move16();
    pSubblockEnergies->firState1 = firState1;
    pSubblockEnergies->firState2 = firState2;
}


