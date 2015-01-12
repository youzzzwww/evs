/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "stat_enc_fx.h"
#include "basop_util.h"

/**********************************************************************/ /*
write single bit to stream
**************************************************************************/
static void IGF_write_bit(Encoder_State_fx                             *st,                 /**< in:     | encoder state structure  */
                          Word16                                       *bitCount,           /**< in/out: | bit counter              */
                          Word16                                        bit                 /**< in:     | value of bit             */
                         )
{
    IGFCommonFuncsWriteSerialBit(st, bitCount, bit);

}

/**********************************************************************/ /*
write bits to stream
**************************************************************************/
static void IGF_write_bits(Encoder_State_fx                            *st,                 /**< in:     | encoder state structure  */
                           Word16                                      *bitCount,           /**< in/out: | bit counter              */
                           Word16                                       value,              /**< in:     | value to be written      */
                           Word16                                       bits                /**< in: Q0  | number of bits           */
                          )
{
    Word16 tmp;

    WHILE (bits)
    {
        bits = sub(bits, 1);
        tmp  = s_and(value, shl(1, bits));
        IF (tmp == 0)
        {
            IGF_write_bit(st, bitCount, 0);
        }
        ELSE
        {
            IGF_write_bit(st, bitCount, 1);
        }
    }

    return;
}

/**********************************************************************/ /*
envelope estimation
**************************************************************************/
static void IGF_CalculateEnvelope(const IGF_ENC_INSTANCE_HANDLE         hInstance,          /**< in:     | instance handle of IGF Encoder                     */
                                  Word32                               *pMDCTSpectrum,      /**< in: Q31 | MDCT spectrum                                      */
                                  Word16                                MDCTSpectrum_e,     /**< in:     | exponent of MDCT spectrum                          */
                                  Word32                               *pPowerSpectrum,     /**< in: Q31 | MDCT^2 + MDST^2 spectrum, or estimate              */
                                  Word16                                PowerSpectrum_e,    /**< in:     | exponent of MDCT^2 + MDST^2 spectrum, or estimate  */
                                  const Word16                          igfGridIdx          /**< in: Q0  | IGF grid index                                     */
                                 )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16   *swb_offset;
    Word16    sfb;                              /* this is the actual scalefactor band */
    Word16    width;                            /* this is width in subbands of the actual scalefactor band */
    Word16    tile_idx;
    Word16    strt_cpy;
    Word16    gain;                             /* the gain which has to be applied to the source tile to get the destination energy */
    Word16    gain_exp;
    Word16    tb;
    Word16    zeroNrg;                          /* Q0 | flag indicating if the signal contains almost no energy */
    Word32    sfbEnergyR[IGF_MAX_SFB];
    Word16    sfbEnergyR_exp[IGF_MAX_SFB];
    Word32    sfbEnergyC[IGF_MAX_SFB];          /* the energy of the destination region of the tile */
    Word16    sfbEnergyC_exp[IGF_MAX_SFB];
    Word32    sfbEnergyTileR[IGF_MAX_SFB];
    Word16    sfbEnergyTileR_exp[IGF_MAX_SFB];
    Word32    sfbEnergyTileC[IGF_MAX_SFB];      /* the energy of the destination region of the tile */
    Word16    sfbEnergyTileC_exp[IGF_MAX_SFB];
    Word32    LFMDCTSpectrum[N_MAX];
    Word16    LFMDCTSpectrum_exp;
    Word32    LFPowerSpectrum[N_MAX];
    Word16    tmp;
    Word16    tmp_exp;
    Word32    L_tmp;
    Word16    shift;


    /* initialize variables */
    Copy32(pMDCTSpectrum, hInstance->spec_be_igf, hInstance->infoStopLine);
    hPrivateData             = &hInstance->igfData;
    hGrid                    = &hPrivateData->igfInfo.grid[igfGridIdx];
    swb_offset               = hGrid->swb_offset;
    move16();
    hInstance->spec_be_igf_e = MDCTSpectrum_e;
    move16();
    zeroNrg                  = 0;
    move16();

    IF (pPowerSpectrum != NULL)
    {
        FOR (tile_idx = 0; tile_idx < hGrid->nTiles; tile_idx++)
        {
            strt_cpy = hGrid->sbWrap[tile_idx];
            move16();
            FOR (sfb = hGrid->sfbWrap[tile_idx]; sfb < hGrid->sfbWrap[tile_idx + 1]; sfb++)
            {
                FOR (tb = swb_offset[ sfb ]; tb < swb_offset[ sfb+1 ]; tb++)
                {
                    LFMDCTSpectrum[tb]  = pMDCTSpectrum[strt_cpy];
                    move32();
                    LFPowerSpectrum[tb] = pPowerSpectrum[strt_cpy];
                    move32();
                    strt_cpy            = add(strt_cpy, 1);
                }
            }
        }
        IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->startSfb,
                                             hGrid->stopSfb,
                                             hGrid->swb_offset,
                                             pPowerSpectrum,
                                             &PowerSpectrum_e,
                                             sfbEnergyC,
                                             sfbEnergyC_exp);
        IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->startSfb,
                                             hGrid->stopSfb,
                                             hGrid->swb_offset,
                                             LFPowerSpectrum,
                                             &PowerSpectrum_e,
                                             sfbEnergyTileC,
                                             sfbEnergyTileC_exp);
        IGFCommonFuncsMDCTSquareSpec(hGrid->startLine,
                                     hGrid->stopLine,
                                     LFMDCTSpectrum,
                                     MDCTSpectrum_e,
                                     LFMDCTSpectrum,
                                     &LFMDCTSpectrum_exp,
                                     0);
        IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->startSfb,
                                             hGrid->stopSfb,
                                             hGrid->swb_offset,
                                             LFMDCTSpectrum,
                                             &LFMDCTSpectrum_exp,
                                             sfbEnergyTileR,
                                             sfbEnergyTileR_exp);
    }
    ELSE
    {
        IGFCommonFuncsMDCTSquareSpec(hGrid->startLine,
        hGrid->stopLine,
        pMDCTSpectrum,
        MDCTSpectrum_e,
        LFMDCTSpectrum,
        &LFMDCTSpectrum_exp,
        0);
        IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->startSfb,
        hGrid->stopSfb,
        hGrid->swb_offset,
        LFMDCTSpectrum,
        &LFMDCTSpectrum_exp,
        sfbEnergyR,
        sfbEnergyR_exp);
    }

    FOR (tile_idx = 0; tile_idx < hGrid->nTiles; tile_idx++)
    {
        FOR(sfb = hGrid->sfbWrap[tile_idx]; sfb < hGrid->sfbWrap[tile_idx + 1]; sfb++)
        {
            width    = sub(swb_offset[sfb + 1], swb_offset[sfb]);
            L_tmp    = 0;
            move16();
            gain_exp = 0;
            move16();

            IF (pPowerSpectrum)
            {
                IF (sfbEnergyTileR[sfb] == 0)
                {
                    sfbEnergyTileR[sfb]      = 0x00010000;
                    move32();
                    sfbEnergyTileR_exp[sfb]  = 0;
                    move16();
                    zeroNrg                  = 1;
                    move16();
                }
                IF (sfbEnergyTileC[sfb] == 0)
                {
                    sfbEnergyTileC[sfb]      = 0x00010000;
                    move32();
                    sfbEnergyTileC_exp[sfb]  = 0;
                    move16();
                    zeroNrg                  = 1;
                    move16();
                }
                IF (sfbEnergyC[sfb] == 0)
                {
                    sfbEnergyC[sfb]      = 0x00010000;
                    move32();
                    sfbEnergyC_exp[sfb]  = 0;
                    move16();
                    zeroNrg              = 1;
                    move16();
                }

                BASOP_Util_Divide_MantExp(round_fx(sfbEnergyTileR[sfb]),
                                          sfbEnergyTileR_exp[sfb],
                                          width,
                                          15,
                                          &gain,
                                          &gain_exp);
                BASOP_Util_Divide_MantExp(round_fx(sfbEnergyC[sfb]),
                                          sfbEnergyC_exp[sfb],
                                          round_fx(sfbEnergyTileC[sfb]),
                                          sfbEnergyTileC_exp[sfb],
                                          &tmp,
                                          &tmp_exp);
                L_tmp    = L_mult(gain, tmp);
                gain_exp = add(gain_exp, tmp_exp);
            }
            ELSE
            {
                IF(sfbEnergyR[sfb] == 0)
                {
                    sfbEnergyR[sfb]      = 0x00010000;
                    move32();
                    sfbEnergyR_exp[sfb]  = 0;
                    move16();
                    zeroNrg              = 1;
                    move16();
                }
                BASOP_Util_Divide_MantExp(round_fx(sfbEnergyR[sfb]),
                sfbEnergyR_exp[sfb],
                width,
                15,
                &gain,
                &gain_exp);
                L_tmp = L_deposit_h(gain);
            }

            /* gain = 0.5f + (float)((2.885390081777927f * log(gain) + 16.f)); */
            L_tmp    = BASOP_Util_Log2(L_tmp);
            L_tmp    = L_add(L_tmp, L_deposit_h(shl(gain_exp, 15-6)));
            shift    = norm_l(L_tmp);
            gain     = round_fx(L_shl(L_tmp, shift));
            gain_exp = sub(7, shift);
            gain_exp = BASOP_Util_Add_MantExp(gain, gain_exp, FL2WORD16_SCALE(16, 4), 4, &gain);
            gain_exp = BASOP_Util_Add_MantExp(gain, gain_exp, 0x4000, 0, &gain);
            gain     = shr(gain, min(sub(15, gain_exp), 15));

            if (gain > 91)
            {
                gain = s_min(gain, 91);  /* 13+15+63, see arithocde encode residual */
            }
            if (gain < 0)
            {
                gain = s_max(gain, 0);
            }

            /* set gain to zero if the signal contains too less energy */
            if (zeroNrg != 0)
            {
                gain = 0;
                move16();
            }

            hPrivateData->igfScfQuantized[sfb] = gain;
            move16();
        }
    }

    return;
}

/**********************************************************************/ /*
writes IGF SCF values
**************************************************************************/
static void IGF_WriteEnvelope(                                                            /**< out: Q0 | number of bits writen                                                        */
    const IGF_ENC_INSTANCE_HANDLE           hInstance,          /**< in:     | instance handle of IGF Encoder                                               */
    Encoder_State_fx                       *st,                 /**< in:     | encoder state                                                                */
    Word16                                 *pBitOffset,         /**< in:     | ptr to bitOffset counter                                                     */
    const Word16                            igfGridIdx,         /**< in: Q0  | igf grid index see declaration of IGF_GRID_IDX for details                   */
    const Word16                            isIndepFlag,        /**< in: Q0  | if 1 frame is independent, 0 = frame is coded with data from previous frame  */
    Word16                                 *igfAllZero          /**< in: Q0  | returns 1 if all IGF scfs are zero, else 0                                   */
)
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 sfb;

    *igfAllZero   = 1;
    move16();
    hPrivateData  = &hInstance->igfData;
    hGrid         = &hPrivateData->igfInfo.grid[igfGridIdx];

    FOR (sfb = hGrid->startSfb; sfb < hGrid->stopSfb; sfb++)
    {
        IF (hPrivateData->igfScfQuantized[sfb] != 0)
        {
            *igfAllZero = 0;
            move16();
            BREAK;
        }
    }

    IF (*igfAllZero != 0)
    {
        IGF_write_bit(st, pBitOffset, 1);
        IF (NULL == st)
        {
            IGFSCFEncoderSaveContextState(&hPrivateData->hIGFSCFArithEnc);
        }
        IGFSCFEncoderReset(&hPrivateData->hIGFSCFArithEnc);
        IF (NULL == st)
        {
            IGFSCFEncoderRestoreContextState(&hPrivateData->hIGFSCFArithEnc);
        }
    }
    ELSE
    {
        IGF_write_bit(st, pBitOffset, 0);
        IF (NULL == st)
        {
            IGFSCFEncoderSaveContextState(&hPrivateData->hIGFSCFArithEnc);
        }

        *pBitOffset = IGFSCFEncoderEncode(&hPrivateData->hIGFSCFArithEnc,
        st,
        *pBitOffset,
        &hPrivateData->igfScfQuantized[hGrid->startSfb],
        isIndepFlag,
        (NULL != st));
        move16();

        IF (NULL == st)
        {
            IGFSCFEncoderRestoreContextState(&hPrivateData->hIGFSCFArithEnc);
        }
    }

}

/**********************************************************************/ /*
identifies significant spectral content
**************************************************************************/
static void IGF_ErodeSpectrum(Word16                             *highPassEner_exp,         /**< out:    | exponent of highPassEner       */
                              const IGF_ENC_INSTANCE_HANDLE       hInstance,                /**< in:     | instance handle of IGF Encoder */
                              Word32                             *pSpectrum,                /**< in/out: | MDCT spectrum                  */
                              Word32                             *pPowerSpectrum,           /**< in/out: | power spectrum                 */
                              Word16                              pPowerSpectrum_exp,       /**< in:     | exponent of power spectrum     */
                              const Word16                        igfGridIdx                /**< in: Q0  | IGF grid index                 */
                             )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 i;
    Word16 igfBgn;
    Word16 igfEnd;
    Word32 highPassEner;          /* Q31 */
    Word32 lastLine;
    Word32 nextLine;
    Word32 L_c;
    Word32 highPassEner_Ovfl;
    Word16 s;
    Word16 tmploop;
    Word16 *swb_offset;
    Word16 sfb;
    Word16 startSfb;
    Word16 stopSfb;
    Word16 line;
    Word16 flag;
    Word16 *igfScaleF;
    Word16 tmp;
    Word32 L_tmp;


    hPrivateData      = &hInstance->igfData;
    hGrid             = &hPrivateData->igfInfo.grid[igfGridIdx];
    igfBgn            = hGrid->startLine;
    move16();
    igfEnd            = hGrid->stopLine;
    move16();
    swb_offset        = hGrid->swb_offset;
    move16();
    startSfb          = hGrid->startSfb;
    move16();
    stopSfb           = hGrid->stopSfb;
    move16();
    igfScaleF         = hPrivateData->igfScfQuantized;
    move16();
    *highPassEner_exp = 0;
    move16();
    highPassEner      = L_add(0, 0);

    IF (NULL == pPowerSpectrum)
    {
        FOR (i = igfBgn; i< hGrid->infoGranuleLen; i++)
        {
            pSpectrum[i] = L_deposit_l(0);
        }
        return;
    }

    IF (igfBgn > 0)
    {
        L_c = L_add(0, 0);
        FOR (i = 0; i < igfBgn; i++)
        {
            Carry        = 0;
            highPassEner = L_add_c(highPassEner, Mpy_32_16_1(pPowerSpectrum[i], shl(i,4)/*Q4*/)/*Q20, pPowerSpectrum_exp*/);
            Overflow     = 0;
            L_c          = L_macNs(L_c, 0, 0);
        }

        highPassEner      = norm_llQ31(L_c, highPassEner, highPassEner_exp);                                                    /*Q20, highPassEner_exp*/
        *highPassEner_exp = add(*highPassEner_exp,pPowerSpectrum_exp);
        igfBgn            = shl(igfBgn, 1);
        highPassEner      = L_deposit_l(BASOP_Util_Divide3216_Scale(highPassEner/*Q20, highPassEner_exp*/, igfBgn /*Q0*/,&s));  /*Q15, highPassEner_exp+11-16+s*/
        *highPassEner_exp = add(add(*highPassEner_exp,s),12 - 16 + (31 - 15));                                                  /*Q15->Q31,highPassEner_exp*/
        lastLine          = pSpectrum[i - 1];
        move32();
        nextLine          = L_add(0, 0);

        /* May overflow - just for threshold comparison                                                   */
        /* negate because the negated may be 1 larger in abs,                                             */
        /* so whenever compared to the negation of a maximum possible pPowerspectrum, it is still larger  */
        BASOP_SATURATE_WARNING_OFF
        highPassEner_Ovfl = L_shl(L_negate(highPassEner), sub(*highPassEner_exp, pPowerSpectrum_exp));
        L_tmp             = L_add(pPowerSpectrum[i - 1], highPassEner_Ovfl);
        BASOP_SATURATE_WARNING_ON

        if (L_tmp >= 0)
        {
            nextLine = L_add(pSpectrum[i], 0);
        }
        tmploop = sub(igfEnd,1);
        FOR (/*i*/; i < tmploop; i++)
        {
            /* May overflow - just for threshold comparison */
            BASOP_SATURATE_WARNING_OFF
            L_tmp = L_add(pPowerSpectrum[i], highPassEner_Ovfl);
            BASOP_SATURATE_WARNING_ON

            IF (L_tmp < 0)
            {
                lastLine       = L_add(pSpectrum[i], 0);
                pSpectrum[i]   = nextLine;
                move32();
                nextLine       = L_add(0, 0);
            }
            ELSE
            {
                pSpectrum[i-1] = lastLine;
                move32();
                lastLine       = L_add(pSpectrum[i], 0);
                nextLine       = L_add(pSpectrum[i+1], 0);
            }
        }

        /* May overflow - just for threshold comparison */
        BASOP_SATURATE_WARNING_OFF
        L_tmp = L_add(pPowerSpectrum[i], highPassEner_Ovfl);
        BASOP_SATURATE_WARNING_ON
        IF (L_tmp < 0)
        {
            pSpectrum[i] = L_deposit_l(0);
        }
    }

    /* delete spectrum above igfEnd: */
    FOR (i = igfEnd; i < hGrid->infoGranuleLen; i++)
    {
        pSpectrum[i]      = L_deposit_l(0);
        pPowerSpectrum[i] = L_deposit_l(0);
    }

    FOR (sfb = startSfb; sfb < stopSfb; sfb++)
    {
        flag = 0;
        move16();
        FOR (line = swb_offset[sfb]; line < swb_offset[sfb+1]; line++)
        {
            if (pSpectrum[line] != 0)
            {
                flag = 1;
                move16();
            }
        }
        tmp = igfScaleF[sfb];
        move16();
        if(flag)
        {
            tmp = sub(igfScaleF[sfb], 1);
        }
        if (igfScaleF[sfb])
        {
            igfScaleF[sfb] = tmp;
            move16();
        }
    }

}

/**********************************************************************/ /*
crest factor calculation
**************************************************************************/
static Word16 IGF_getCrest(                                                                 /**< out: Q15| crest factor                 */
    Word16                                      *crest_exp,          /**< out:    | exponent of crest factor     */
    const Word32                                *powerSpectrum,      /**< in: Q31 | power spectrum               */
    const Word16                                 powerSpectrum_exp,  /**< in:     | exponent of power spectrum   */
    const Word16                                 start,              /**< in: Q0  | start subband index          */
    const Word16                                 stop                /**< in: Q0  | stop subband index           */
)
{
    Word16 i;
    Word16 x;
    Word16 s;
    Word32 x_eff32;
    Word16 x_max;
    Word16 crest;
    Word16 tmp;
    Word32 tmp32;

    x_eff32    = L_add(0, 0);
    x_max      = 0;
    move16();
    crest      = FL2WORD16(.5f);
    move16();
    *crest_exp = 1;
    move16();

    FOR (i = start; i < stop; i++)
    {
        /*x      = max(0, (int)(log(powerSpectrum[i]) * INV_LOG_2));*/

        /*see IGF_getSFM for more comment */
        x = sub(sub(powerSpectrum_exp, norm_l(powerSpectrum[i])), 1);       /*Q0*/
        if (powerSpectrum[i] == 0)                                          /*special case: energy is zero*/
        {
            x = 0;
            move16();
        }
        x       = s_max(0, x);
        x_eff32 = L_mac0(x_eff32, x ,x);                                    /*Q0*/
        x_max   = s_max(x_max, x);                                          /*Q0*/
    }

    /*x_eff /= (stop - start);*/
    x_eff32 = BASOP_Util_Divide3216_Scale(x_eff32, sub(stop,start), &s);    /*Q-1, s*/
    s       = add(s, 32);                                                   /*make x_eff Q31*/

    /*trunkate to int*/
    x_eff32 = L_shr(x_eff32, sub(31, s));
    x_eff32 = L_shl(x_eff32, sub(31, s));

    test();
    IF (x_eff32 > 0 && x_max > 0)
    {
        /*crest = max(1.f, (float)x_max/sqrt(x_eff));*/
        tmp32      = ISqrt32(x_eff32, &s);                                  /*Q31, s*/
        tmp32      = Mpy_32_16_1(tmp32/*Q31, s*/, x_max/*Q0*/);        /*Q16, s*/
        i          = norm_l(tmp32);
        tmp32      = L_shl(tmp32, i);                                       /*Q31, s-i+15*/
        crest      = extract_h(tmp32);
        *crest_exp = add(sub(s, i), 15);

        /* limit crest factor to a lower bound of 1, may overflow */
        BASOP_SATURATE_WARNING_OFF
        tmp = shl(-1, sub(15, *crest_exp));                                 /* build negative threshold */
        tmp = add(crest, tmp);
        BASOP_SATURATE_WARNING_ON
        if (tmp < 0)
        {
            crest      = 1;
            move16();
        }
        if (tmp < 0)
        {
            *crest_exp = 15;
            move16();
        }
    }

    return crest;
}

/*************************************************************************
calculates spectral flatness measurment
**************************************************************************/
static Word16 IGF_getSFM(                                                                   /**< out: Q15| SFM value              */
    Word16                                        *SFM_exp,            /**< out:    | exponent of SFM Factor */
    const Word32                                  *energy,             /**< in:  Q31| energies               */
    const Word16                                  *energy_exp,         /**< in:     | exponent of energies   */
    const Word16                                   start,              /**< in:  Q0 | start subband index    */
    const Word16                                   stop                /**< in:  Q0 | stop subband index     */
)
{
    Word16 n,i, s;
    Word32 num;
    Word32 denom;
    Word16 denom_exp;
    Word16 invDenom_exp, numf_exp;
    Word16 numf;
    Word32 SFM32;
    Word32 L_c;
    Word16 invDenom, SFM;

    L_c   = L_add(0, 0);
    num   = L_add(0, 0);
    denom = L_shr(2147483 /*0,001 in Q31 - float is "1", here*/,s_min(*energy_exp, 31));
    denom = L_max(denom, 1);
    *SFM_exp = 0;
    move16();
    SFM   = FL2WORD16_SCALE(1.0f, 0);
    move16();

    FOR (i = start; i < stop; i++)
    {
        /*ln(x * 2^-Qx * 2^xExp) = ln(x) - Qx + xExp*/

        /* n       = sub(sub(31,norm_l(tmp32)),1);  */                                                 /*<- ld    */
        /* n       = sub(n,31);                     */                                                 /*<- -Qx   */
        /* n       = add(n,*energy_exp);            */                                                 /*<- +xExp */

        n = sub(sub(*energy_exp, norm_l(energy[i])), 1);                                          /*<-- short form*/

        if (energy[i] == 0)                                                                       /*special case: energy is zero*/
        {
            n = 0;
            move16();
        }

        n        = s_max(0, n);
        num      = L_add(num, L_deposit_l(n));                                                    /*Q0*/

        Carry    = 0;
        denom    = L_add_c(energy[i], denom);
        Overflow = 0;

        L_c       = L_macNs(L_c, 0, 0);
    }

    denom     = norm_llQ31(L_c, denom, &denom_exp);                                               /*Q31*/
    denom_exp = add(denom_exp, *energy_exp);

    /* calculate SFM only if signal is present */
    IF (denom != 0)
    {
        /*numf   = (float)num / (float)(stop - start);*/
        numf     =  BASOP_Util_Divide3216_Scale(num,                                              /*Q0*/
                                                sub(stop,start),                                  /*Q0*/
                                                &s);                                               /*Q-1 s*/
        numf_exp = add(s,16);                                                                     /*-> numf Q15 numf_exp*/
        /*denom /= (float)(stop - start);*/
        /*return ((float)pow(2.0, numf + 0.5f) / denom);*/

        /*SFM= ((float)pow(2.0, numf + 0.5f) * invDenom);*/
        invDenom     = BASOP_Util_Divide3232_uu_1616_Scale(L_deposit_l(sub(stop, start))          /*Q0*/,
                       denom                                  /*Q31, denom_exp*/,
                       &s);                                    /*Q-16, s-denom_exp*/
        invDenom_exp = add(sub(s, denom_exp), 31);                                                /*invDenom: Q15, invDenom_exp*/

        /*add .5f to numf*/
        SFM32     = L_add(L_shl(L_deposit_l(numf), numf_exp) /*16Q15*/,FL2WORD32_SCALE(.5f, 16)); /*16Q15*/
        s         = norm_l(SFM32);
        SFM32     = L_shl(SFM32, s);
        s         = sub(16, s);                                                                   /*SFM32(numf) is Q31 now*/

        /*do the pow2 and the mult*/
        SFM32     = BASOP_util_Pow2(SFM32, s, &s);
        SFM32     = Mpy_32_16_1(SFM32, invDenom);
        *SFM_exp  = add(s, invDenom_exp);

        /*Transform to Q15*/
        s         = norm_l(SFM32);
        SFM       = round_fx(L_shl(SFM32, s));
        *SFM_exp  = sub(*SFM_exp, s);

        /**SFM_exp = s_min(*SFM_exp, 0);*/
        IF (*SFM_exp > 0)
        {
            *SFM_exp = 0;
            move16();
            SFM      = FL2WORD16_SCALE(1.0f, 0);
            move16();
        }
    }

    return SFM /*Q15*/;
}

/**********************************************************************/ /*
calculates the IGF whitening levels by SFM and crest
**************************************************************************/
static void IGF_Whitening(const IGF_ENC_INSTANCE_HANDLE                 hInstance,          /**< in:     | instance handle of IGF Encoder               */
                          Word32                                       *powerSpectrum,      /**< in: Q31 | MDCT/MDST power spectrum                     */
                          const Word16                                  powerSpectrum_exp,  /**< in:     | exponent of powerspectrum                    */
                          const Word16                                  igfGridIdx,         /**< in: Q0  | IGF grid index                               */
                          Word16                                        isTransient,        /**< in: Q0  | boolean, indicating if transient is detected */
                          Word16                                        last_core_acelp     /**< in: Q0  | indictaor if last frame was acelp coded      */
                         )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 p;                             /*Q0*/
    Word16 tmp;
    Word16 SFM;
    Word16 crest;
    Word16 SFM_exp;
    Word16 crest_exp;
    Word16 s;
    Word32 tmp32;
    Word32 SFM32;

    hPrivateData = &hInstance->igfData;
    hGrid        = &hPrivateData->igfInfo.grid[igfGridIdx];

    IF (igfGridIdx != IGF_GRID_LB_NORM)
    {
        FOR (p = 0; p < hGrid->nTiles; p++)
        {
            /* reset filter */
            hPrivateData->prevSFM_FIR[p] = L_deposit_l(0);
            hPrivateData->prevSFM_IIR[p] = 0;
            move16();

            /* preset values: */
            hPrivateData->igfCurrWhiteningLevel[p] = IGF_WHITENING_OFF;
            move16();
        }
    }
    FOR (p = 0; p < IGF_MAX_TILES; p++)
    {
        /* update prev data: */
        hPrivateData->igfPrevWhiteningLevel[p] = hPrivateData->igfCurrWhiteningLevel[p];
        move16();
        /* preset values: */
        hPrivateData->igfCurrWhiteningLevel[p] = IGF_WHITENING_OFF;
        move16();
    }

    IF (!s_or(isTransient, hPrivateData->wasTransient))
    {
        IF (powerSpectrum)
        {
            Word16 nT = hGrid->nTiles;
            SWITCH (hPrivateData->igfInfo.bitRateIndex)
            {
            case IGF_BITRATE_WB_9600:
            case IGF_BITRATE_SWB_9600:
            case IGF_BITRATE_SWB_16400:
            case IGF_BITRATE_SWB_24400:
            case IGF_BITRATE_SWB_32000:
            case IGF_BITRATE_FB_16400:
            case IGF_BITRATE_FB_24400:
            case IGF_BITRATE_FB_32000:
                nT = sub(nT, 1);
                BREAK;
            default:
                BREAK;
            }
            FOR (p = 0; p < nT; p++)
            {
                /*tmp  = IGF_getSFM(powerSpectrum, hGrid->tile[p], hGrid->tile[p+1]) / IGF_getCrest(powerSpectrum, hGrid->tile[p], hGrid->tile[p+1]);*/
                SFM   = IGF_getSFM(&SFM_exp, powerSpectrum, &powerSpectrum_exp, hGrid->tile[p], hGrid->tile[p + 1]);
                crest = IGF_getCrest(&crest_exp, powerSpectrum, powerSpectrum_exp, hGrid->tile[p], hGrid->tile[p + 1]);

                tmp   = BASOP_Util_Divide1616_Scale(SFM, crest, &s);  /*   Q15 */
                s     = add(s, sub(SFM_exp, crest_exp));
                tmp32 = L_shl(L_deposit_l(tmp)/*16Q15, s*/,add(s,1)); /* 15Q16 */

                test();
                IF (last_core_acelp || hPrivateData->wasTransient)
                {
                    hPrivateData->prevSFM_FIR[p] = tmp32;         /* 15Q16 */                 move32();
                    hPrivateData->prevSFM_IIR[p] = shr(tmp, 2);   /*  2Q13 */                 move16();
                }

                /*SFM  = tmp + hPrivateData->prevSFM_FIR[p] + 0.5f * hPrivateData->prevSFM_IIR[p];*/
                SFM32 = L_add(tmp32,hPrivateData->prevSFM_FIR[p]);
                SFM32 = L_mac0(SFM32,hPrivateData->prevSFM_IIR[p]/*Q13*/,FL2WORD16_SCALE(.5f,15-(16-13)));/*15Q16*/

                BASOP_SATURATE_WARNING_OFF
                /*SFM  = min(2.7f, SFM);*/
                /*Overflow possible in shift, intended*/
                SFM = s_min(FL2WORD16_SCALE(2.7f,2),extract_h(L_shr(SFM32,16-29)/*->Q29*/)/*->Q13*/ );
                BASOP_SATURATE_WARNING_ON

                hPrivateData->prevSFM_FIR[p] = tmp32; /*15Q16*/                               move32();
                hPrivateData->prevSFM_IIR[p] = SFM;
                move16();

                IF (sub(SFM , hGrid->whiteningThreshold[1][p]) > 0)
                {
                    hPrivateData->igfCurrWhiteningLevel[p] = IGF_WHITENING_STRONG;
                    move16();
                }
                ELSE IF (sub(SFM , hGrid->whiteningThreshold[0][p]) > 0)
                {
                    hPrivateData->igfCurrWhiteningLevel[p] = IGF_WHITENING_MID;
                    move16();
                }
            }
            SWITCH (hPrivateData->igfInfo.bitRateIndex)
            {
            case IGF_BITRATE_WB_9600:
            case IGF_BITRATE_RF_WB_13200:
            case IGF_BITRATE_RF_SWB_13200:
            case IGF_BITRATE_SWB_9600:
            case IGF_BITRATE_SWB_16400:
            case IGF_BITRATE_SWB_24400:
            case IGF_BITRATE_SWB_32000:
            case IGF_BITRATE_FB_16400:
            case IGF_BITRATE_FB_24400:
            case IGF_BITRATE_FB_32000:
                move16();
                hPrivateData->igfCurrWhiteningLevel[hGrid->nTiles - 1] = hPrivateData->igfCurrWhiteningLevel[hGrid->nTiles - 2];
                BREAK;
            default:
                BREAK;
            }
        }
        ELSE
        {
            FOR (p = 0; p < hGrid->nTiles; p++)
            {
                hPrivateData->igfCurrWhiteningLevel[p] = IGF_WHITENING_MID;
                move16();
            }
        }
    }
    ELSE
    {
        /* reset filter */
        FOR (p = 0; p < IGF_MAX_TILES; p++)
        {
            hPrivateData->prevSFM_FIR[p] = L_deposit_l(0);
            hPrivateData->prevSFM_IIR[p] = 0;
            move16();
        }
    }
    hPrivateData->wasTransient = isTransient;
    move16();

}

/**********************************************************************/ /*
write whitening levels into bitstream
**************************************************************************/
static void IGF_WriteWhiteningTile(                                                       /**< out: Q0 | number of bits written     */
    Encoder_State_fx                  *st,                 /**< in:     | encoder state handle       */
    Word16                            *pBitOffset,         /**< in:     | ptr to bitOffset counter   */
    Word16                             whiteningLevel      /**< in: Q0  | whitening levels to write  */
)
{
    IF (L_sub(whiteningLevel, IGF_WHITENING_MID) == 0)
    {
        IGF_write_bits(st, pBitOffset, 0, 1);
    }
    ELSE
    {
        IGF_write_bits(st, pBitOffset, 1, 1);
        IF (L_sub(whiteningLevel , IGF_WHITENING_OFF) == 0)
        {
            IGF_write_bits(st, pBitOffset, 0, 1);
        }
        ELSE
        {
            IGF_write_bits(st, pBitOffset, 1, 1);
        }
    }
}

/**********************************************************************/ /*
writes the whitening levels
**************************************************************************/
static void IGF_WriteWhiteningLevels(                                                     /**< out: Q0 | total number of bits written                                                 */
    const IGF_ENC_INSTANCE_HANDLE    hInstance,          /**< in:     | instance handle of IGF encoder                                               */
    Encoder_State_fx                *st,                 /**< in:     | encoder state                                                                */
    Word16                          *pBitOffset,         /**< in:     | ptr to bitOffset counter                                                     */
    const Word16                     igfGridIdx,         /**< in: Q0  | igf grid index see declaration of IGF_GRID_IDX for details                   */
    const Word16                     isIndepFlag         /**< in: Q0  | if 1 frame is independent, 0 = frame is coded with data from previous frame  */
)
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 p;
    Word16 nTiles;
    Word16 isSame;
    Word32 tmp32;


    isSame        = 1;
    move16();
    hPrivateData  = &hInstance->igfData;
    hGrid         = &hPrivateData->igfInfo.grid[igfGridIdx];
    nTiles        = hGrid->nTiles;
    move16();

    IF (isIndepFlag)
    {
        isSame = 0;
        move16();
    }
    ELSE
    {
        p     = 0;
        move16();
        tmp32 = L_add(0, 0);

        WHILE ((sub(p, nTiles) < 0) && (tmp32 == 0))
        {
            test();
            tmp32 = L_sub(hPrivateData->igfCurrWhiteningLevel[p] , hPrivateData->igfPrevWhiteningLevel[p]);
            if (tmp32 != 0)
            {
                isSame = 0;
                move16();
            }
            p++;
        }
    }
    IF (isSame)
    {
        IGF_write_bits(st, pBitOffset, 1, 1);
    }
    ELSE
    {
        IF (!isIndepFlag)
        {
            IGF_write_bits(st, pBitOffset, 0, 1);
        }
        IGF_WriteWhiteningTile(st, pBitOffset, hPrivateData->igfCurrWhiteningLevel[0]);
        p      = 1;
        move16();
        tmp32  = L_add(0, 0);
        if (sub(p, nTiles) < 0)
        {
            isSame = 1;
            move16();
        }

        WHILE ((sub(p, nTiles) < 0) && (tmp32 == 0))
        {
            test();
            tmp32 = L_sub(hPrivateData->igfCurrWhiteningLevel[p] , hPrivateData->igfCurrWhiteningLevel[p - 1]);
            if (tmp32 != 0)
            {
                isSame = 0;
                move16();
            }
            p++;
        }

        IF (!isSame)
        {
            IGF_write_bits(st, pBitOffset, 1, 1);
            FOR (p = 1; p < nTiles; p++)
            {
                IGF_WriteWhiteningTile(st, pBitOffset, hPrivateData->igfCurrWhiteningLevel[p]);
            }
        }
        ELSE
        {
            IGF_write_bits(st, pBitOffset, 0, 1);
        }
    }

}

/**********************************************************************/ /*
write flattening trigger
**************************************************************************/
static void IGF_WriteFlatteningTrigger(                                                   /**< out:    | number of bits written         */
    const IGF_ENC_INSTANCE_HANDLE  hInstance,          /**< in:     | instance handle of IGF Encoder */
    Encoder_State_fx              *st,                 /**< in:     | encoder state                  */
    Word16                        *pBitOffset          /**< in:     | ptr to bitOffset counter       */
)
{
    Word16 flatteningTrigger;


    flatteningTrigger = hInstance->flatteningTrigger;
    move16();

    IGF_write_bits(st, pBitOffset, flatteningTrigger, 1);

}

/**********************************************************************/ /*
updates the start/stop frequency of IGF according to igfGridIdx
**************************************************************************/
static void IGF_UpdateInfo(const IGF_ENC_INSTANCE_HANDLE                hInstance,          /**< in:     | instance handle of IGF Encoder */
                           const Word16                                 igfGridIdx          /**< in: Q0  | IGF grid index                 */
                          )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;


    hPrivateData                  = &hInstance->igfData;
    hGrid                         = &hPrivateData->igfInfo.grid[igfGridIdx];
    hInstance->infoStartFrequency = hGrid->startFrequency;
    move16();
    hInstance->infoStopFrequency  = hGrid->stopFrequency;
    move16();
    hInstance->infoStartLine      = hGrid->startLine;
    move16();
    hInstance->infoStopLine       = hGrid->stopLine;
    move16();

    return;
}

/**********************************************************************/ /*
IGF bitsream writer
**************************************************************************/
Word16 IGFEncWriteBitstream(                                                             /**< out:    | number of bits written per frame                                             */
    const IGF_ENC_INSTANCE_HANDLE            hInstance,          /**< in:     | instance handle of IGF Encoder                                               */
    Encoder_State_fx                        *st,                 /**< in:     | encoder state                                                                */
    Word16                                  *pBitOffset,         /**< in:     | ptr to bitOffset counter                                                     */
    const Word16                             igfGridIdx,         /**< in: Q0  | igf grid index see declaration of IGF_GRID_IDX for details                   */
    const Word16                             isIndepFlag         /**< in: Q0  | if 1 frame is independent, 0 = frame is coded with data from previous frame  */
)
{
    Word16 igfAllZero;
    Word16 startBitCount;


    startBitCount                           = *pBitOffset;
    move16();
    hInstance->infoTotalBitsPerFrameWritten = 0;
    move16();

    if (isIndepFlag)
    {
        hInstance->infoTotalBitsWritten = 0;
        move16();
    }

    IGF_WriteEnvelope(hInstance,                        /* i: instance handle of IGF Encoder                                              */
                      st,                               /* i: encoder state                                                               */
                      pBitOffset,                       /* i: ptr to bitOffset counter                                                    */
                      igfGridIdx,                       /* i: igf grid index see definition of IGF_GRID_IDX for details                   */
                      isIndepFlag,                      /* i: if 1 frame is independent, 0 = frame is coded with data from previous frame */
                      &igfAllZero);                      /* o: *igfAllZero                                                                 */

    IGF_WriteWhiteningLevels(hInstance,                 /* i: instance handle of IGF Encoder                                              */
                             st,                        /* i: encoder state                                                               */
                             pBitOffset,                /* i: ptr to bitOffset counter                                                    */
                             igfGridIdx,                /* i: igf grid index see definition of IGF_GRID_IDX for details                   */
                             isIndepFlag);              /* i: if 1 frame is independent, 0 = frame is coded with data from previous frame */

    IGF_WriteFlatteningTrigger(hInstance,               /* i: instance handle of IGF Encoder                                              */
                               st,                      /* i: encoder state                                                               */
                               pBitOffset);             /* i: ptr to bitOffset counter                                                    */

    hInstance->infoTotalBitsPerFrameWritten = sub(*pBitOffset, startBitCount);
    hInstance->infoTotalBitsWritten         = add(hInstance->infoTotalBitsWritten, hInstance->infoTotalBitsPerFrameWritten);

    return hInstance->infoTotalBitsPerFrameWritten;
}

/**********************************************************************/ /*
sets the IGF mode according to given bitrate
**************************************************************************/
void IGFEncSetMode(const IGF_ENC_INSTANCE_HANDLE                     hInstance,             /**< in:     | instance handle of IGF Encoder */
                   const Word32                                      bitRate,               /**< in: Q0  | encoder bitrate                */
                   const Word16                                      mode                   /**< in: Q0  | encoder bandwidth mode         */
                   ,const Word16                                      rf_mode                /**< in: Q0  | flag to signal the RF mode     */
                  )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;

    hPrivateData = &hInstance->igfData;
    hPrivateData->igfBitstreamBits = 0;
    move16();
    set16_fx(hPrivateData->igfScfQuantized, 0, IGF_MAX_SFB);
    set16_fx(hPrivateData->igfCurrWhiteningLevel, 0, IGF_MAX_TILES);
    set16_fx(hPrivateData->igfPrevWhiteningLevel, 0, IGF_MAX_TILES);
    set16_fx(hPrivateData->igfBitstream, 0, IGF_MAX_GRANULE_LEN);
    hPrivateData->wasTransient     = 0;
    move16();
    set32_fx(hPrivateData->prevSFM_FIR, 0, IGF_MAX_TILES);
    set16_fx(hPrivateData->prevSFM_IIR, 0, IGF_MAX_TILES);

    IF (IGFCommonFuncsIGFConfiguration(bitRate, mode, &hPrivateData->igfInfo,rf_mode) != 0)
    {
        IGFSCFEncoderOpen(&hPrivateData->hIGFSCFArithEnc,
                          sub(hPrivateData->igfInfo.grid[0].stopSfb, hPrivateData->igfInfo.grid[0].startSfb),
                          bitRate,
                          mode
                          ,rf_mode

                         );

        hInstance->infoSamplingRate   = hPrivateData->igfInfo.sampleRate;
        move32();
        hInstance->infoStartFrequency = hPrivateData->igfInfo.grid[0].startFrequency;
        move16();
        hInstance->infoStopFrequency  = hPrivateData->igfInfo.grid[0].stopFrequency;
        move16();
        hInstance->infoStartLine      = hPrivateData->igfInfo.grid[0].startLine;
        move16();
        hInstance->infoStopLine       = hPrivateData->igfInfo.grid[0].stopLine;
        move16();
    }
    ELSE
    {
        /* IGF configuration failed -> error! */
        hInstance->infoSamplingRate   =  0;
        move32();
        hInstance->infoStartFrequency = -1;
        move16();
        hInstance->infoStopFrequency  = -1;
        move16();
        hInstance->infoStartLine      = -1;
        move16();
        hInstance->infoStopLine       = -1;
        move16();
        fprintf(stderr,"IGFEncSetMode: initialization error!\n");
    }

    /* reset remaining variables */
    hInstance->infoTotalBitsWritten         = 0;
    move16();
    hInstance->infoTotalBitsPerFrameWritten = 0;
    move16();
    hInstance->flatteningTrigger            = 0;
    move16();
    hInstance->spec_be_igf_e                = 0;
    move16();
    hInstance->tns_predictionGain           = 0;
    move16();
    set32_fx(hInstance->spec_be_igf, 0, N_MAX);

    return;
}

/**********************************************************************/ /*
IGF bitsream concatenation for TCX10 modes
**************************************************************************/
void IGFEncConcatenateBitstream(const IGF_ENC_INSTANCE_HANDLE        hInstance,          /**< in:     | instance handle of IGF Encoder                 */
                                Word16                               bsBits,             /**< in: Q0  | number of IGF bits written to list of indices  */
                                Word16                              *next_ind,           /**< in/out: | pointer to actual bit indice                   */
                                Word16                              *nb_bits,            /**< in/out: | total number of bits already written           */
                                Indice_fx                           *ind_list_fx         /**< in:     | pointer to list of indices                     */
                               )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    Word16 i;
    Word16 *pBitstream;


    hPrivateData = &hInstance->igfData;
    pBitstream   = &hPrivateData->igfBitstream[0];
    *next_ind    = sub(*next_ind, bsBits);

    FOR (i = 0; i < bsBits; i++)
    {
        pBitstream[hPrivateData->igfBitstreamBits+i] = ind_list_fx[*next_ind+i].value;
        move16();
    }
    *nb_bits                       = sub(*nb_bits, bsBits);
    hPrivateData->igfBitstreamBits = add(hPrivateData->igfBitstreamBits, bsBits);

    return;
}

/**********************************************************************/ /*
IGF reset bitsream bit counter for TCX10 modes
**************************************************************************/
void IGFEncResetTCX10BitCounter(const IGF_ENC_INSTANCE_HANDLE        hInstance           /**< in:     | instance handle of IGF Encoder */
                               )
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;

    hPrivateData                    = &hInstance->igfData;
    hPrivateData->igfBitstreamBits  = 0;
    move16();
    hInstance->infoTotalBitsWritten = 0;
    move16();

    return;
}

/**********************************************************************/ /*
IGF write concatenated bitsream for TCX10 modes
**************************************************************************/
Word16 IGFEncWriteConcatenatedBitstream(                                                 /**< out: Q0 | total number of bits written   */
    const IGF_ENC_INSTANCE_HANDLE hInstance,         /**< in:     | instance handle of IGF Encoder */
    void                         *st                 /**< in:     | encoder state                  */
)
{
    IGF_ENC_PRIVATE_DATA_HANDLE hPrivateData;
    Word16 i;
    Word16 *pBitstream;


    hPrivateData = &hInstance->igfData;
    pBitstream   = &hPrivateData->igfBitstream[0];

    i = 0;
    move16();
    WHILE (sub(i, hPrivateData->igfBitstreamBits) < 0)
    {
        push_next_indice_fx(st, pBitstream[i], 1 );
        i = add(i, 1);
    }

    return hInstance->infoTotalBitsWritten;
}

/**********************************************************************/ /*
apply the IGF encoder, main encoder interface
**************************************************************************/
void IGFEncApplyMono(const IGF_ENC_INSTANCE_HANDLE                   hInstance,          /**< in:     | instance handle of IGF Encoder                         */
                     const Word16                                    igfGridIdx,         /**< in: Q0  | IGF grid index                                         */
                     Encoder_State_fx                               *st,                 /**< in:     | Encoder state                                          */
                     Word32                                         *pMDCTSpectrum,      /**< in: Q31 | MDCT spectrum                                          */
                     Word16                                          MDCTSpectrum_e,     /**< in:     | exponent of MDCT spectrum                              */
                     Word32                                         *pPowerSpectrum,     /**< in: Q31 | MDCT^2 + MDST^2 spectrum, or estimate                  */
                     Word16                                          PowerSpectrum_e,    /**< in:     | exponent of pPowerSpectrum                             */
                     Word16                                          isTCX20,            /**< in: Q0  | flag indicating if the input is TCX20 or TCX10/2xTCX5  */
                     Word16                                          isTNSActive,        /**< in: Q0  | flag indicating if the TNS is active                   */
                     Word16                                          last_core_acelp     /**< in: Q0  | indictaor if last frame was acelp coded                */
                    )
{
    Word32 *pPowerSpectrumParameter;                         /* If it is NULL it informs a function that specific handling is needed */
    Word32 *pPowerSpectrumParameterWhitening;                /* If it is NULL it informs a function that specific handling is needed */
    Word16 highPassEner_exp;                                 /*exponent of highpass energy - maybe not needed*/


    pPowerSpectrumParameter = NULL;
    test();
    if ((isTNSActive == 0) && (isTCX20 != 0))
    {
        pPowerSpectrumParameter = pPowerSpectrum;
    }
    pPowerSpectrumParameterWhitening = NULL;
    if (isTCX20 != 0)
    {
        pPowerSpectrumParameterWhitening = pPowerSpectrum;
    }

    IGF_UpdateInfo(hInstance,                               /* i: instance handle of IGF Encoder            */
                   igfGridIdx);                             /* i: IGF grid index                            */

    IGF_CalculateEnvelope(hInstance,                        /* i: instance handle of IGF Encoder            */
                          pMDCTSpectrum,                    /* i: MDCT spectrum                             */
                          MDCTSpectrum_e,                   /* i: exponent of MDCT spectrum                 */
                          pPowerSpectrumParameter,          /* i: MDCT^2 + MDST^2 spectrum, or estimate     */
                          PowerSpectrum_e,                  /* i: exponent of pPowerSpectrum                */
                          igfGridIdx);                      /* i: IGF grid index                            */

    IGF_Whitening(hInstance,                                /* i: instance handle of IGF Encoder            */
                  pPowerSpectrumParameterWhitening,         /* i: MDCT^2 + MDST^2 spectrum, or estimate     */
                  PowerSpectrum_e,                          /* i: exponent of powerSpectrum                 */
                  igfGridIdx,                               /* i: IGF grid index                            */
                  (st->transientDetection.transientDetector.bIsAttackPresent == 1),
                  last_core_acelp);                         /* i: last frame was acelp indicator            */

    pPowerSpectrumParameter = NULL;
    if (isTCX20 != 0)
    {
        pPowerSpectrumParameter = pPowerSpectrum;
    }

    IGF_ErodeSpectrum(                                      /* o: highpass energy                           */
        &highPassEner_exp,                    /* o: exponent of highPassEner                  */
        hInstance,                            /* i: instance handle of IGF Encoder            */
        pMDCTSpectrum,                        /* i: MDCT spectrum                             */
        pPowerSpectrumParameter,              /* i: MDCT^2 + MDST^2 spectrum, or estimate     */
        PowerSpectrum_e,                      /* i: exponent of pPowerSpectrum                */
        igfGridIdx);                          /* i: IGF grid index                            */

}

