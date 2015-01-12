/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "basop_util.h"

/**********************************************************************/ /*
returns an int val, multiplied with transFac
**************************************************************************/
static Word16 IGF_ApplyTransFac(                                                            /**< out: Q0 | multiplication factor                                                        */
    const Word16               val,                /**< in: Q15 | input value for multiplication, Q15                                          */
    const Word16               transFac            /**< in: Q14 | multiplicator for variable val, Q14: 1.25f=0x5000, 1.0f=0x4000, 0.5f=0x2000  */
)
{
    Word16 ret;

    if(sub(transFac, 0x4000) == 0)
    {
        return val;
    }

    ret = shl(val, 1);
    ret = mac_r(0x00000000, ret, transFac);
    ret = add(ret, s_and(ret, 1));


    return ret;
}

/**********************************************************************/ /*
maps a given bitrate to the IGF_BITRATE index
**************************************************************************/
static Word16 IGF_MapBitRateToIndex(                                                        /**< out: Q0 | return bit rate index  */
    Word32 bitRate,                                         /**< in:     | bitrate                */
    Word16 mode                                             /**< in:     | bandwidth mode         */
    , Word16 rf_mode                                        /**< in:     | flag to signal the RF mode */
)
{
    Word16 bitRateIndex;


    bitRateIndex = IGF_BITRATE_UNKNOWN;
    move16();

    switch (mode)
    {
    case IGF_MODE_WB:
        switch (bitRate)
        {
        case 13200:
            if (sub(rf_mode,1) == 0)
            {
                bitRateIndex = IGF_BITRATE_RF_WB_13200;
            }
            break;
        case 9600:
            bitRateIndex = IGF_BITRATE_WB_9600;
            break;
        default:
            break;
        }
        break;
    case IGF_MODE_SWB:
        switch (bitRate)
        {
        case  9600:
            bitRateIndex = IGF_BITRATE_SWB_9600;
            break;
        case 13200:
            bitRateIndex = IGF_BITRATE_SWB_13200;
            if (sub(rf_mode,1) == 0)
            {
                bitRateIndex = IGF_BITRATE_RF_SWB_13200;
            }
            break;
        case 16400:
            bitRateIndex = IGF_BITRATE_SWB_16400;
            break;
        case 24400:
            bitRateIndex = IGF_BITRATE_SWB_24400;
            break;
        case 32000:
            bitRateIndex = IGF_BITRATE_SWB_32000;
            break;
        case 48000:
            bitRateIndex = IGF_BITRATE_SWB_48000;
            break;
        case 64000:
            bitRateIndex = IGF_BITRATE_SWB_64000;
            break;
        default:
            break;
        }
        break;
    case IGF_MODE_FB:
        switch (bitRate)
        {
        case 16400:
            bitRateIndex = IGF_BITRATE_FB_16400;
            break;
        case 24400:
            bitRateIndex = IGF_BITRATE_FB_24400;
            break;
        case 32000:
            bitRateIndex = IGF_BITRATE_FB_32000;
            break;
        case 48000:
            bitRateIndex = IGF_BITRATE_FB_48000;
            break;
        case 64000:
            bitRateIndex = IGF_BITRATE_FB_64000;
            break;
        case 96000:
            bitRateIndex = IGF_BITRATE_FB_96000;
            break;
        case 128000:
            bitRateIndex = IGF_BITRATE_FB_128000;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return bitRateIndex;
}

/**********************************************************************/ /*
IGF grid setup
**************************************************************************/
static void IGF_gridSetUp(H_IGF_GRID                                    hGrid,              /**< out:    | IGF grid handle                                                    */
                          Word16                                        bitRateIndex,       /**< in: Q0  | IGF bitrate index                                                  */
                          Word32                                        sampleRate,         /**< in:     | sample rate                                                        */
                          Word16                                        frameLength,        /**< in:     | frame length                                                       */
                          Word16                                        transFac,           /**< in:     | transFac                                                           */
                          Word16                                        igfMinFq            /**< in:     | IGF minimum frequency indicating lower start frequency for copy up */
                         )
{
    Word16 t;
    Word16 sfb;
    const Word16 *swb_offset;
    Word16 swb_offset_len;
    Word16 bandwidth;
    Word16 wrp_sfb;
    Word16 tmp1;
    Word16 tmp2;
    Word32 L_tmp1;
    Word32 L_tmp2;

    swb_offset     = NULL;
    move16();
    swb_offset_len = 0;
    move16();

    SWITCH (bitRateIndex)
    {
    case IGF_BITRATE_WB_9600:
    case IGF_BITRATE_SWB_9600:
    case IGF_BITRATE_RF_WB_13200:
    case IGF_BITRATE_RF_SWB_13200:
    case IGF_BITRATE_SWB_13200:
    case IGF_BITRATE_SWB_16400:
    case IGF_BITRATE_SWB_24400:
    case IGF_BITRATE_SWB_32000:
    case IGF_BITRATE_SWB_48000:
    case IGF_BITRATE_SWB_64000:
        swb_offset     = &swb_offset_LB_new[bitRateIndex][1];
        swb_offset_len = swb_offset_LB_new[bitRateIndex][0];
        move16();
        Copy(&igf_whitening_TH[bitRateIndex][0][0], &hGrid->whiteningThreshold[0][0], IGF_MAX_TILES * 2);
        BREAK;
    case IGF_BITRATE_FB_16400:
    case IGF_BITRATE_FB_24400:
    case IGF_BITRATE_FB_32000:
        swb_offset     = &swb_offset_LB_new[bitRateIndex][1];
        swb_offset_len = swb_offset_LB_new[bitRateIndex][0];
        move16();
        Copy(&igf_whitening_TH[bitRateIndex][0][0], &hGrid->whiteningThreshold[0][0], IGF_MAX_TILES * 2);
        BREAK;
    case IGF_BITRATE_FB_48000:
    case IGF_BITRATE_FB_64000:
    case IGF_BITRATE_FB_96000:
    case IGF_BITRATE_FB_128000:
        swb_offset     = &swb_offset_LB_new[bitRateIndex][1];
        swb_offset_len = swb_offset_LB_new[bitRateIndex][0];
        move16();
        Copy(&igf_whitening_TH[bitRateIndex][0][0], &hGrid->whiteningThreshold[0][0], IGF_MAX_TILES * 2);
        BREAK;
    case IGF_BITRATE_UNKNOWN:
    default:
        BREAK;
    }

    FOR(sfb = 0; sfb < swb_offset_len; sfb++)
    {
        hGrid->swb_offset[sfb] = IGF_ApplyTransFac(swb_offset[sfb], transFac);
        move16();
    }

    hGrid->infoIsRefined     = 0;
    move16();
    frameLength              = IGF_ApplyTransFac(frameLength, transFac);
    tmp2                     = norm_s(frameLength);
    bandwidth                = shl(frameLength,tmp2);
    hGrid->swb_offset_len    = extract_l(L_shr(sampleRate, 2));
    tmp1                     = sub(norm_s(hGrid->swb_offset_len), 1);
    hGrid->swb_offset_len    = shl(hGrid->swb_offset_len, tmp1);
    bandwidth                = div_s(hGrid->swb_offset_len, bandwidth);
    tmp2                     = sub(add(tmp2, 1), tmp1);
    bandwidth                = shr(bandwidth, sub(15, tmp2));


    hGrid->swb_offset_len    = swb_offset_len;
    move16();
    hGrid->startSfb          = 0;
    move16();
    hGrid->stopSfb           = sub(hGrid->swb_offset_len, 1);
    hGrid->startLine         = hGrid->swb_offset[ hGrid->startSfb ];
    move16();
    hGrid->stopLine          = hGrid->swb_offset[ hGrid->stopSfb ];
    move16();
    hGrid->startFrequency    = imult1616(bandwidth, hGrid->startLine);
    hGrid->stopFrequency     = imult1616(bandwidth, hGrid->stopLine);

    L_tmp1                   = L_mult0(igfMinFq, frameLength);
    tmp1                     = sub(norm_l(L_tmp1), 1);
    L_tmp1                   = L_shl(L_tmp1, tmp1);

    tmp2                     = norm_l(sampleRate);
    L_tmp2                   = L_shl(sampleRate, tmp2);
    tmp1                     = add(WORD16_BITS-1, sub(tmp1, add(tmp2, 1))); /* takes into account sampleRate >> 1 */

    hGrid->minSrcSubband     = div_s(extract_h(L_tmp1), extract_h(L_tmp2));
    hGrid->minSrcSubband     = shr(hGrid->minSrcSubband, tmp1);


    hGrid->minSrcSubband     = add(hGrid->minSrcSubband, s_and(hGrid->minSrcSubband, 1));
    hGrid->minSrcFrequency   = imult1616(bandwidth, hGrid->minSrcSubband);
    hGrid->infoGranuleLen    = frameLength;
    move16();
    hGrid->infoTransFac      = transFac;
    move16();

    hGrid->sfbWrap[0]        = 0;
    move16();
    hGrid->tile[0]           = hGrid->startLine;
    move16();


    /*************************************************************************/
    SWITCH (bitRateIndex)
    {
        /* SWB 13200 */
    case IGF_BITRATE_WB_9600:
        hGrid->nTiles           = 2;
        move16();
        wrp_sfb                 = 2;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[1]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();
        BREAK;

    case IGF_BITRATE_RF_WB_13200:
        hGrid->nTiles           = 2;
        wrp_sfb                 = 2;

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];

        /*2nd*/
        hGrid->sfbWrap[1+1]     = hGrid->stopSfb;
        hGrid->sbWrap[1]        = hGrid->minSrcSubband;
        hGrid->tile[1+1]        = hGrid->swb_offset[hGrid->stopSfb];

        BREAK;
    case IGF_BITRATE_SWB_9600:
        hGrid->nTiles           = 3;
        wrp_sfb                 = 1;

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];

        /*2nd*/
        wrp_sfb                 = 2;
        hGrid->sfbWrap[1+1]     = wrp_sfb;
        hGrid->sbWrap[1]        = hGrid->minSrcSubband + IGF_ApplyTransFac(32, transFac);
        hGrid->tile[1+1]        = hGrid->swb_offset[wrp_sfb];

        /*3rd*/
        hGrid->sfbWrap[2+1]     = hGrid->stopSfb;
        hGrid->sbWrap[2]        = hGrid->minSrcSubband + IGF_ApplyTransFac(46, transFac);
        hGrid->tile[2+1]        = hGrid->swb_offset[hGrid->stopSfb];

        BREAK;
    case IGF_BITRATE_RF_SWB_13200:
        hGrid->nTiles           = 3;
        wrp_sfb                 = 1;

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];

        /*2nd*/
        wrp_sfb                 = 2;
        hGrid->sfbWrap[1+1]     = wrp_sfb;
        hGrid->sbWrap[1]        = hGrid->minSrcSubband + IGF_ApplyTransFac(32, transFac);
        hGrid->tile[1+1]        = hGrid->swb_offset[wrp_sfb];

        /*3rd*/
        hGrid->sfbWrap[2+1]     = hGrid->stopSfb;
        hGrid->sbWrap[2]        = hGrid->minSrcSubband + IGF_ApplyTransFac(46, transFac);
        hGrid->tile[2+1]        = hGrid->swb_offset[hGrid->stopSfb];

        BREAK;

    case IGF_BITRATE_SWB_13200:
        hGrid->nTiles           = 2;
        move16();
        wrp_sfb                 = 4;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[1]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(32, transFac));
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();
        BREAK;

    case IGF_BITRATE_SWB_16400:
        hGrid->nTiles           = 3;
        move16();
        wrp_sfb                 = 4;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = 6;
        move16();
        hGrid->sbWrap[1]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(48, transFac));
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[6];
        move16();

        /*3nd*/
        hGrid->sfbWrap[2+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[2]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(64, transFac));
        move16();
        hGrid->tile[2+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();
        BREAK;

    case IGF_BITRATE_SWB_24400:
    case IGF_BITRATE_SWB_32000:
        hGrid->nTiles           = 3;
        move16();
        wrp_sfb                 = 4;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = 7;
        move16();
        hGrid->sbWrap[1]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(32, transFac));
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[7];
        move16();

        /*3nd*/
        hGrid->sfbWrap[2+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[2]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(64, transFac));
        move16();
        hGrid->tile[2+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();
        BREAK;
    case IGF_BITRATE_SWB_48000:
    case IGF_BITRATE_SWB_64000:
        hGrid->nTiles           = 1;
        move16();
        wrp_sfb                 = hGrid->stopSfb;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[0]        = sub(shl(hGrid->startLine, 1), hGrid->stopLine);
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();

        BREAK;
    case IGF_BITRATE_FB_16400:
        hGrid->nTiles           = 3;
        move16();
        wrp_sfb                 = 4;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();
        wrp_sfb                 = 7;
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[1]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*3nd*/
        hGrid->sfbWrap[2+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[2]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[2+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();

        BREAK;

    case IGF_BITRATE_FB_24400:
    case IGF_BITRATE_FB_32000:
        hGrid->nTiles           = 4;
        move16();
        wrp_sfb                 = 4;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[0]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[wrp_sfb];
        move16();
        wrp_sfb                 = 6;
        move16();

        /*2nd*/
        hGrid->sfbWrap[1+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[1]        = add(hGrid->minSrcSubband, IGF_ApplyTransFac(32, transFac));
        move16();
        hGrid->tile[1+1]        = hGrid->swb_offset[wrp_sfb];
        move16();
        wrp_sfb                 = 9;
        move16();

        /*3nd*/
        hGrid->sfbWrap[2+1]     = wrp_sfb;
        move16();
        hGrid->sbWrap[2]        = hGrid->minSrcSubband;
        move16();
        hGrid->tile[2+1]        = hGrid->swb_offset[wrp_sfb];
        move16();

        /*4nd*/
        hGrid->sfbWrap[3+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[3]        = add(hGrid->minSrcSubband, sub(hGrid->swb_offset[9], hGrid->swb_offset[8]));
        move16();
        hGrid->tile[3+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();
        BREAK;
    case IGF_BITRATE_FB_48000:
    case IGF_BITRATE_FB_64000:
    case IGF_BITRATE_FB_96000:
    case IGF_BITRATE_FB_128000:
        hGrid->nTiles           = 1;
        move16();

        /*1st*/
        hGrid->sfbWrap[0+1]     = hGrid->stopSfb;
        move16();
        hGrid->sbWrap[0]        = sub(shl(hGrid->startLine, 1), hGrid->stopLine);
        move16();
        hGrid->tile[0+1]        = hGrid->swb_offset[hGrid->stopSfb];
        move16();

        BREAK;
    default:
        BREAK;
    }/*switch*/

    /*************************************************************************/
    /*************************************************************************/


    /* adapt level envelope: */
    SWITCH (bitRateIndex)
    {
    case IGF_BITRATE_RF_WB_13200:
    case IGF_BITRATE_WB_9600:
        hGrid->gFactor = FL2WORD16_SCALE(0.80f, 1);
        move16();
        hGrid->fFactor = FL2WORD16_SCALE(0.70f, 1);
        move16();
        hGrid->lFactor = FL2WORD16_SCALE(0.60f, 1);
        move16();
        BREAK;
    case IGF_BITRATE_SWB_13200:
    case IGF_BITRATE_FB_16400:
    case IGF_BITRATE_SWB_16400:
        hGrid->gFactor = FL2WORD16_SCALE(0.93f, 1);
        move16();
        hGrid->fFactor = FL2WORD16_SCALE(0.20f, 1);
        move16();
        hGrid->lFactor = FL2WORD16_SCALE(0.85f, 1);
        move16();
        BREAK;
    case IGF_BITRATE_FB_24400:
    case IGF_BITRATE_SWB_24400:
    case IGF_BITRATE_FB_32000:
    case IGF_BITRATE_SWB_32000:
        hGrid->gFactor = FL2WORD16_SCALE(0.965f, 1);
        move16();
        hGrid->fFactor = FL2WORD16_SCALE(0.20f,  1);
        move16();
        hGrid->lFactor = FL2WORD16_SCALE(0.85f,  1);
        move16();
        BREAK;
    case IGF_BITRATE_FB_48000:
    case IGF_BITRATE_SWB_48000:
    case IGF_BITRATE_FB_64000:
    case IGF_BITRATE_SWB_64000:
        hGrid->gFactor = FL2WORD16_SCALE(1.00f, 1);
        move16();
        hGrid->fFactor = FL2WORD16_SCALE(0.20f, 1);
        move16();
        hGrid->lFactor = FL2WORD16_SCALE(1.00f, 1);
        move16();
        BREAK;
    case IGF_BITRATE_SWB_9600:
    case IGF_BITRATE_RF_SWB_13200:
    default:
        hGrid->gFactor = FL2WORD16_SCALE(1.00f, 1);
        move16();
        hGrid->fFactor = FL2WORD16_SCALE(0.00f, 1);
        move16();
        hGrid->lFactor = FL2WORD16_SCALE(1.00f, 1);
        move16();
    }

    FOR (t = add(hGrid->nTiles, 1); t < IGF_MAX_TILES; t++)
    {
        hGrid->tile[t]       = 0;
        move16();
        hGrid->sbWrap[t - 1] = 0;
        move16();
        hGrid->sfbWrap[t]    = 0;
        move16();
    }

}

/**********************************************************************/ /*
calculates energy per sfb via power spectrum
**************************************************************************/
void IGFCommonFuncsCalcSfbEnergyPowerSpec(const Word16               startSfb,            /**< in: Q0  | start sfb index                                          */
        const Word16               stopSfb,             /**< in: Q0  | stop  sfb index                                          */
        const Word16              *swb_offset,          /**< in: Q0  | IGF swb offset table                                     */
        Word32                    *pPowerSpectrum,      /**< in: Q31 | power spectrum                                           */
        Word16                    *pPowerSpectrum_exp,  /**< in:     | Exponent of PowerSpectrum                                */
        Word32                    *sfbEnergy,           /**< out:Q31 | SFB energies , will be initialized inside this function  */
        Word16                    *sfbEnergy_exp        /**< out:    | Exponent of PowerSpectrum                                */
                                         )
{
    Word16/*Q0*/   sfb;
    Word16/*Q0*/   line;
    Word32 L_c;


    FOR (sfb = startSfb; sfb < stopSfb; sfb++)
    {
        sfbEnergy[sfb] = L_deposit_l(0);
    }
    IF (NULL == pPowerSpectrum)
    {
        return;
    }

    FOR (sfb = startSfb; sfb < stopSfb; sfb++)
    {
        L_c = L_deposit_l(0);
        FOR (line = swb_offset[sfb]; line < swb_offset[sfb+1]; line++)
        {
            Carry = 0;
            sfbEnergy[sfb] = L_add_c(sfbEnergy[sfb], pPowerSpectrum[line]);
            move32();
            Overflow = 0;
            L_c = L_macNs(L_c,0,0);
        }
        sfbEnergy[sfb] = norm_llQ31(L_c,sfbEnergy[sfb],&(sfbEnergy_exp[sfb]));
        move32();
        sfbEnergy_exp[sfb] = add(sfbEnergy_exp[sfb],*pPowerSpectrum_exp);
        move16();
    }
}

/**********************************************************************/ /*
calculate the MDCT square spectrum in the IGF range
**************************************************************************/
void IGFCommonFuncsMDCTSquareSpec(const Word16                       sqrtBgn,            /**< in: Q0  | start MDCT subband index       */
                                  const Word16                       sqrtEnd,            /**< in: Q0  | stop  MDCT subband index       */
                                  const Word32                      *mdctSpec,           /**< in: Q31 | MDCT spectrum to square        */
                                  const Word16                       mdctSpec_e,         /**< in:     | exponent of mdctSpectrum       */
                                  Word32                            *mdctSquareSpec,     /**< out:Q31 | MDCT square spectrum           */
                                  Word16                            *mdctSquareSpec_e,   /**< out:    | exponent of mdctSquareSpec     */
                                  Word16                             indexOffset         /**< in: Q0  | index offset                   */
                                 )
{
    Word16 i;
    Word16 j;
    Word16 s1;
    Word16 tmp;


    /* get headroom, only in IGF range */
    s1 = getScaleFactor32(mdctSpec + sqrtBgn, sub(sqrtEnd, sqrtBgn));

    /* set new exponent */
    *mdctSquareSpec_e = add(shl(sub(mdctSpec_e, s1), 1), 1);
    move16();

    /* MDCT square spectrum: MDCT^2 */
    j = add(sqrtBgn, indexOffset);                                    /* handle indexOffset with care, otherwise memory overruns may occur! */


    FOR (i = sqrtBgn; i < sqrtEnd; i++)
    {
        tmp                 = round_fx(L_shl(mdctSpec[i], s1));
        mdctSquareSpec[j++] = L_mult0(tmp, tmp);
        move32();
    }


}

/**********************************************************************/ /*
write bits to stream
**************************************************************************/
void IGFCommonFuncsWriteSerialBit(void                              *st,                 /**< in:     | encoder/decoder state structure  */
                                  Word16                            *pBitOffset,         /**< out: Q0 | bit offset                       */
                                  Word16                             bit                 /**< in: Q0  | value of bit                     */
                                 )
{

    IF (st)
    {
        push_next_indice_fx(st, bit, 1);
    }
    *pBitOffset = add(*pBitOffset, 1);
    move16();

    return;
}

/**********************************************************************/ /*
changes the IGF configuration
**************************************************************************/
Word16 IGFCommonFuncsIGFConfiguration(                                                   /**< out:    | error value: 0 -> error, 1 -> ok   */
    Word32                         bitRate,            /**< in: Q0  | bitrate in bs e.g. 9600 for 9.6kbs */
    Word16                         mode,               /**< in: Q0  | bandwidth mode                     */
    H_IGF_INFO                     hIGFInfo            /**< out:    | IGF info handle                    */
    ,Word16                         rf_mode             /**< in: flag to signal the RF mode */
)
{
    H_IGF_GRID hGrid;
    Word16 retValue;
    Word32 sampleRate;
    Word16 frameLength;
    Word16 igfMinFq;
    Word16 maxHopsize;

    retValue = 0;     /* bitrate index is unknown -> error! */                                               move16();

    /* interface call for reading in settings */
    hIGFInfo->bitRateIndex = IGF_MapBitRateToIndex(bitRate, mode
                             ,rf_mode
                                                  );

    IF (sub(hIGFInfo->bitRateIndex, IGF_BITRATE_UNKNOWN) != 0)
    {
        retValue = 1; /* no error */                                                                         move16();

        /* mapping to local values */
        sampleRate  = igfMode[hIGFInfo->bitRateIndex].sampleRate;
        move32();
        frameLength = igfMode[hIGFInfo->bitRateIndex].frameLength;
        move16();
        igfMinFq    = igfMode[hIGFInfo->bitRateIndex].igfMinFq;
        move16();
        maxHopsize  = igfMode[hIGFInfo->bitRateIndex].maxHopsize;
        move16();

        /* basic information */
        hIGFInfo->sampleRate  = sampleRate;
        move32();
        hIGFInfo->frameLength = frameLength;
        move16();
        hIGFInfo->maxHopsize  = maxHopsize;
        move16();
        hIGFInfo->nfSeed      = 0;
        move16();

        /* set up regular IGF grid for TCX 20  (transfac = 1.f) */
        hGrid                 = &hIGFInfo->grid[IGF_GRID_LB_NORM];
        IGF_gridSetUp(hGrid,
                      hIGFInfo->bitRateIndex,
                      sampleRate,
                      frameLength,
                      FL2WORD16_SCALE(1, 1),
                      igfMinFq);

        /* set up IGF grid for CELP->TCX 20 transitions (transfac = 1.25) */
        hGrid                 = &hIGFInfo->grid[IGF_GRID_LB_TRAN];
        IGF_gridSetUp(hGrid,
                      hIGFInfo->bitRateIndex,
                      sampleRate,
                      frameLength,
                      FL2WORD16_SCALE(1.25, 1),
                      igfMinFq);
        /* set up IGF grid for TCX 10 (transfac = 0.5) */
        hGrid                 = &hIGFInfo->grid[IGF_GRID_LB_SHORT];
        IGF_gridSetUp(hGrid,
                      hIGFInfo->bitRateIndex,
                      sampleRate,
                      frameLength,
                      FL2WORD16_SCALE(0.50f, 1),
                      igfMinFq);
    }

    return retValue;
}

/**********************************************************************/ /*
selects cumulative frequency tables and offsets for the IGF SCF arithmetic coder
**************************************************************************/
Word16 IGFCommonFuncsIGFGetCFTables(                                                     /**< out:    | error value: 0 -> error, 1 -> ok     */
    Word32                           bitRate,            /**< in: Q0  | bitrate in bs e.g. 9600 for 9.6kbs   */
    Word16                           mode,               /**< in: Q0  | bandwidth mode                       */
    Word16                           rf_mode,            /**< in:     | flag to signal the RF mode           */
    const Word16                   **cf_se00,            /**< out:    | CF table for t == 0 and f == 0       */
    const Word16                   **cf_se01,            /**< out:    | CF table for t == 0 and f == 1       */
    Word16                          *cf_off_se01,        /**< out:    | offset for CF table above            */
    const Word16                   **cf_se02,            /**< out:    | CF tables for t == 0 and f >= 2      */
    const Word16                   **cf_off_se02,        /**< out:    | offsets for CF tables above          */
    const Word16                   **cf_se10,            /**< out:    | CF table for t == 1 and f == 0       */
    Word16                          *cf_off_se10,        /**< out:    | offset for CF table above            */
    const Word16                   **cf_se11,            /**< out:    | CF tables for t == 1 and f >= 1      */
    const Word16                   **cf_off_se11         /**< out:    | offsets for CF tables above          */
)
{
    Word16 retValue;
    Word16 bitRateIndex;


    retValue     = 0;     /* bitrate index is unknown -> error! */                                          move16();
    bitRateIndex = IGF_MapBitRateToIndex(bitRate, mode
                                         ,rf_mode
                                        );


    IF (sub(bitRateIndex, IGF_BITRATE_UNKNOWN) != 0)
    {
        retValue = 1; /* no error */                                                                        move16();
        SWITCH(bitRateIndex)
        {
        case IGF_BITRATE_WB_9600:
        case IGF_BITRATE_RF_WB_13200:
        case IGF_BITRATE_SWB_9600:
        case IGF_BITRATE_SWB_13200:
        case IGF_BITRATE_RF_SWB_13200:
        case IGF_BITRATE_SWB_16400:
        case IGF_BITRATE_SWB_24400:
        case IGF_BITRATE_SWB_32000:
        case IGF_BITRATE_SWB_48000:
        case IGF_BITRATE_SWB_64000:
            *cf_se00      = cf_se00_tab;
            *cf_se01      = cf_se01_tab[bitRateIndex];
            *cf_off_se01  = cf_off_se01_tab[bitRateIndex];
            *cf_se02      = &cf_se02_tab[bitRateIndex][0][0];
            move16();
            *cf_off_se02  = &cf_off_se02_tab[bitRateIndex][0];
            move16();
            *cf_se10      = &cf_se10_tab[0];
            move16();
            *cf_off_se10  = cf_off_se10_tab;
            *cf_se11      = &cf_se11_tab[0][0][0];
            move16();
            *cf_off_se11  = &cf_off_se11_tab[0][0];
            move16();
            BREAK;
        case IGF_BITRATE_FB_16400:
        case IGF_BITRATE_FB_24400:
        case IGF_BITRATE_FB_32000:
            bitRateIndex  = add(sub(bitRateIndex, IGF_BITRATE_FB_16400), IGF_BITRATE_SWB_16400);
            *cf_se00      = cf_se00_tab;
            *cf_se01      = cf_se01_tab[bitRateIndex];
            *cf_off_se01  = cf_off_se01_tab[bitRateIndex];
            *cf_se02      = &cf_se02_tab[bitRateIndex][0][0];
            move16();
            *cf_off_se02  = &cf_off_se02_tab[bitRateIndex][0];
            move16();
            *cf_se10      = &cf_se10_tab[0];
            move16();
            *cf_off_se10  = cf_off_se10_tab;
            *cf_se11      = &cf_se11_tab[0][0][0];
            move16();
            *cf_off_se11  = &cf_off_se11_tab[0][0];
            move16();
            BREAK;
        case IGF_BITRATE_FB_48000:
        case IGF_BITRATE_FB_64000:
            bitRateIndex  = add(sub(bitRateIndex, IGF_BITRATE_FB_48000), IGF_BITRATE_SWB_48000);
            *cf_se00      = cf_se00_tab;
            *cf_se01      = cf_se01_tab[bitRateIndex];
            *cf_off_se01  = cf_off_se01_tab[bitRateIndex];
            *cf_se02      = &cf_se02_tab[bitRateIndex][0][0];
            move16();
            *cf_off_se02  = &cf_off_se02_tab[bitRateIndex][0];
            move16();
            *cf_se10      = &cf_se10_tab[0];
            move16();
            *cf_off_se10  = cf_off_se10_tab;
            *cf_se11      = &cf_se11_tab[0][0][0];
            move16();
            *cf_off_se11  = &cf_off_se11_tab[0][0];
            move16();
            BREAK;
        case IGF_BITRATE_FB_96000:
        case IGF_BITRATE_FB_128000:
            bitRateIndex  = IGF_BITRATE_SWB_48000;
            move16();
            *cf_se00      = cf_se00_tab;
            *cf_se01      = cf_se01_tab[bitRateIndex];
            *cf_off_se01  = cf_off_se01_tab[bitRateIndex];
            *cf_se02      = &cf_se02_tab[bitRateIndex][0][0];
            move16();
            *cf_off_se02  = &cf_off_se02_tab[bitRateIndex][0];
            move16();
            *cf_se10      = &cf_se10_tab[0];
            move16();
            *cf_off_se10  = cf_off_se10_tab;
            *cf_se11      = &cf_se11_tab[0][0][0];
            move16();
            *cf_off_se11  = &cf_off_se11_tab[0][0];
            move16();
            BREAK;
        case IGF_BITRATE_UNKNOWN:
        default:
            BREAK;
        }
    }
    return retValue;
}

