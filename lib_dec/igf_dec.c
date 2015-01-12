/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <memory.h>
#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "stat_dec_fx.h"
#include "basop_util.h"

/**********************************************************************/ /*
get scalefactor of an Word32 array with condition
**************************************************************************/
static Word16 IGF_getScaleFactor32Cond(                                                     /**< out: Q0 | measured headroom in range [0..31], 0 if all x[i] == 0  */
    const Word16                    *cond,               /**< in: Q0  | array conating the condition                            */
    const Word32                    *x,                  /**< in: Q31 | array containing 32-bit data                            */
    const Word16                     len_x               /**< in: Q0  | length of the array to scan                             */
)
{
    Word16 i;
    Word16 i_min;
    Word16 i_max;
    Word32 x_min;
    Word32 x_max;
    Word32 tmp32;


    x_max = L_add(0, 0);
    x_min = L_add(0, 0);

    FOR (i = 0; i < len_x; i++)
    {
        tmp32 = L_add(x[i], 0);                     /*L_and(x[i], cond[i]);*/

        if (cond[i] == 0)
        {
            tmp32 = L_deposit_h(0);
        }


        if (tmp32>= 0)
        {
            x_max = L_max(x_max, tmp32);
        }
        if (tmp32< 0)
        {
            x_min = L_min(x_min, tmp32);
        }
    }

    i_max = 0x20;
    move16();
    i_min = 0x20;
    move16();

    if (x_max != 0)
    {
        i_max = norm_l(x_max);
    }
    if (x_min != 0)
    {
        i_min = norm_l(x_min);
    }

    i = s_and(s_min(i_max, i_min), 0x1F);

    return i;
}

/**********************************************************************/ /*
measures TCX noise
**************************************************************************/
static Word16 IGF_replaceTCXNoise_1(                                                        /**< out: Q0 | number of noise bands      */
    const Word32                       *in,                 /**< in: Q31 | MDCT spectrum              */
    Word16                              s_l,                /**< in: Q0  | noise headroom             */
    const Word16                       *TCXNoise,           /**< in:     | tcx noise indicator vector */
    const Word16                        start,              /**< in: Q0  | start MDCT subband index   */
    const Word16                        stop,               /**< in: Q0  | stop MDCT subband index    */
    Word32                             *totalNoiseNrg       /**< out:    | measured noise energy      */
)
{
    Word16 sb;
    Word16 tmp16;
    Word16 noise;
    Word32 nE;


    tmp16 = 0;
    move16();
    noise = 0;
    move16();
    s_l   = sub(s_l, 5);
    nE    = L_add(0, 0);

    FOR (sb = start; sb < stop; sb++)
    {
        if (TCXNoise[sb])
        {
            tmp16 = extract_h(L_shl(in[sb], s_l));
        }
        if (TCXNoise[sb])
        {
            nE = L_mac(nE, tmp16, tmp16);
        }
        if (TCXNoise[sb])
        {
            noise = add(noise, 1);
        }
    }

    *totalNoiseNrg = nE;
    move32();

    return noise;
}

/**********************************************************************/ /*
replaces TCX noise
**************************************************************************/
static void IGF_replaceTCXNoise_2(Word32                               *in,                 /**< in/out: | MDCT spectrum                */
                                  const Word16                         *TCXNoise,           /**< in: Q0  | tcx noise indicator vector   */
                                  const Word16                          start,              /**< in: Q0  | start MDCT subband index     */
                                  const Word16                          stop,               /**< in: Q0  | stop MDCT subband index      */
                                  Word32                                totalNoiseNrg,      /**< in:     | measured noise energy        */
                                  const Word16                          s_l,                /**< in: Q0  | noise headroom               */
                                  Word16                               *nfSeed              /**< in:     | random generator noise seed  */
                                 )
{
    Word16 sb;
    Word16 g;
    Word16 val;
    Word32 rE;
    Word32 L_tmp;


    val = 0;
    move16();
    rE  = L_add(0, 0);

    FOR (sb = start; sb <  stop; sb++)
    {
        if (TCXNoise[sb])
        {
            val = Random(nfSeed);
        }
        if (TCXNoise[sb])
        {
            in[sb] = L_deposit_l(val);
        }
        if (TCXNoise[sb])
        {
            val = shr(val, 5);
        }
        if (TCXNoise[sb])
        {
            rE = L_mac(rE, val, val);
        }
    }

    totalNoiseNrg = L_shr(totalNoiseNrg, 1);


    /* make sure that rE is never 0 */
    if (rE == 0)
    {
        rE = L_add(totalNoiseNrg, 0);               /* save move32() -> use L_add(x, 0) = x; */
    }

    /* if totalNoiseNrg == 0, then rE must be at least 0x00010000, otherwise division by 0 will occur */
    if (totalNoiseNrg == 0)
    {
        rE = L_max(rE, 0x00010000);
    }

    /* make sure that rE is never smaller than totalNoiseNrg */
    L_tmp = L_sub(rE, totalNoiseNrg);
    if (L_tmp < 0)
    {
        rE = L_add(totalNoiseNrg, 0);               /* save move32() -> use L_add(x, 0) = x; */
    }


    g = getSqrtWord32(L_mult(divide3232(totalNoiseNrg, rE), FL2WORD16(1.0f / 4.0f)));
    g = shl(g, 1);

    FOR (sb = start; sb < stop; sb++)
    {
        if (TCXNoise[sb])
        {
            in[sb] = L_shr(L_mult(extract_l(in[sb]), g), s_l);
            move32();
        }
    }

}

/**********************************************************************/ /*
reads whitening levels
**************************************************************************/
static void IGF_decode_whitening_level(Decoder_State_fx                *st,                 /**< in:     | decoder state                    */
                                       IGF_DEC_PRIVATE_DATA_HANDLE      hPrivateData,       /**< in:     | instance handle of IGF Deccoder  */
                                       const Word16                     p                   /**< in: Q0  | tile index, p = [0, 3]           */
                                      )
{
    Word16 tmp;


    tmp = get_next_indice_fx(st, 1);

    IF (tmp == 0)
    {
        hPrivateData->currWhiteningLevel[p] = IGF_WHITENING_MID;
        move16();

        return;
    }

    tmp = get_next_indice_fx(st, 1);
    hPrivateData->currWhiteningLevel[p]     = IGF_WHITENING_STRONG;
    move16();

    if (tmp == 0)
    {
        hPrivateData->currWhiteningLevel[p] = IGF_WHITENING_OFF;
        move16();
    }

}

/**********************************************************************/ /*
reads flattening trigger
**************************************************************************/
static void IGF_decode_temp_flattening_trigger(Decoder_State_fx        *st,                 /**< in:     | decoder state                   */
        IGF_DEC_INSTANCE_HANDLE  hInstance           /**< in:     | instance handle of IGF Deccoder */
                                              )
{
    hInstance->flatteningTrigger = get_next_indice_fx(st, 1);
}

/**********************************************************************/ /*
set power spectrum values to zero, needed for energy calculation
**************************************************************************/
static void IGF_setLinesToZero(const Word16                             startLine,          /**< in: Q0  | start MDCT subband index       */
                               const Word16                             stopLine,           /**< in: Q0  | stop  MDCT subband index       */
                               const Word32                            *pSpectralData,      /**< in:     | original MDCT spectrum         */
                               Word32                                  *pPowerSpecIGF       /**< in/out: | prepared IGF energy spectrum   */
                              )
{
    Word16 i;


    /* set energy values in the IGF "power spectrum" to 0,
       if there is content in the original MDCT spectrum */
    FOR (i = startLine; i < stopLine; i++)
    {
        if (pSpectralData[i] != 0)
        {
            pPowerSpecIGF[i] = L_deposit_l(0);
        }
    }

}

/**********************************************************************/ /*
prepare IGF spectrum
**************************************************************************/
static void IGF_prep(IGF_DEC_PRIVATE_DATA_HANDLE                        hPrivateData,       /**< in:     | IGF private data handle                              */
                     const Word16                                       igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength */
                     const Word16                                      *TCXNoise,           /**< in: Q0  | TCX noise vector                                     */
                     Word32                                            *igf_spec,           /**< in: Q31 | prepared IGF spectrum                                */
                     Word16                                            *igf_spec_e,         /**< in:     | array exponents of igf_spec, one exponent per tile   */
                     const Word32                                      *src_spec,           /**< in:     | source spectrum                                      */
                     const Word16                                       src_spec_e,         /**< in:     | exponent of src_spec, whitening off                  */
                     const Word16                                       specMed_e           /**< in:     | exponent of medium flattening level                  */
                    )
{
    H_IGF_GRID hGrid;
    H_IGF_INFO hInfo;
    Word16 i;
    Word16 tb;
    Word16 sfb;
    Word16 nTiles;
    Word16 n_noise_bands;
    Word16 n_noise_bands_off;
    Word16 strt_cpy;
    Word16 startLine;
    Word16 minSrcSubband;
    Word16 tile_idx;
    Word32 totalNoiseNrg;
    Word32 totalNoiseNrg_off;
    const Word32 *sel_spec;


    /* initialize variables */
    hInfo              = &hPrivateData->igfInfo;
    hGrid              = &hPrivateData->igfInfo.grid[igfGridIdx];
    n_noise_bands      = hPrivateData->n_noise_bands;
    move16();
    n_noise_bands_off  = hPrivateData->n_noise_bands_off;
    move16();
    totalNoiseNrg      = hPrivateData->totalNoiseNrg;
    move32();
    totalNoiseNrg_off  = hPrivateData->totalNoiseNrg_off;
    move32();
    nTiles             = hGrid->nTiles;
    move16();
    startLine          = hGrid->startLine;
    move16();
    minSrcSubband      = hGrid->minSrcSubband;
    move16();
    tile_idx           = 0;
    move16();

    FOR (tile_idx = 0; tile_idx < nTiles; tile_idx++)
    {
        strt_cpy = hGrid->sbWrap[tile_idx];
        move16();

        /* strong whitening detected */
        IF (sub(IGF_WHITENING_STRONG, hPrivateData->currWhiteningLevel[tile_idx]) == 0)
        {
            /* fill igf_spec with random noise */
            tb = hGrid->swb_offset[hGrid->sfbWrap[tile_idx]];
            move16();
            FOR (i = strt_cpy; i < startLine; i++)
            {
                igf_spec[tb++] = L_deposit_l(Random(&hInfo->nfSeed));   /* 31Q0, fill LSBs */
            }

            /* set exponent of the current tile, random noise is 31Q0 */
            igf_spec_e[tile_idx] = 31;
            move16();
        }
        ELSE
        {
            /* medium whitening detected */
            IF (sub(IGF_WHITENING_MID, hPrivateData->currWhiteningLevel[tile_idx]) == 0)
            {
                IF (n_noise_bands != 0)
                {
                    IGF_replaceTCXNoise_2(igf_spec,
                    TCXNoise,

                    minSrcSubband,
                    startLine,
                    totalNoiseNrg,
                    hPrivateData->headroom_TCX_noise_white,
                    &hInfo->nfSeed);
                }

                /* selected source spectrum is igf_spec, igf_spec contains the whitened signal in the core region */
                sel_spec = igf_spec;
                move16();

                /* set exponent of the current tile */
                igf_spec_e[tile_idx] = specMed_e;
                move16();
            }
            /* off whitening detectded */
            ELSE
            {
                IF (n_noise_bands_off != 0)
                {
                    IGF_replaceTCXNoise_2(hPrivateData->pSpecFlat,
                    TCXNoise,
                    minSrcSubband,
                    startLine,
                    totalNoiseNrg_off,
                    hPrivateData->headroom_TCX_noise,
                    &hInfo->nfSeed);

                }
                /* selected source spectrum is pSpecFlat, pSpecFlat contains the signal before the LPC reshaping */
                sel_spec = src_spec;
                move16();

                /* set exponent of the current tile */
                igf_spec_e[tile_idx] = src_spec_e;
                move16();
            }
            /* generate the raw IGF spectrum out if the selected spectrum */
            FOR (sfb = hGrid->sfbWrap[tile_idx]; sfb < hGrid->sfbWrap[tile_idx + 1]; sfb++)
            {
                FOR (tb = hGrid->swb_offset[sfb]; tb < hGrid->swb_offset[sfb + 1]; tb++)
                {
                    igf_spec[tb] = sel_spec[strt_cpy];
                    move32();
                    strt_cpy     = add(strt_cpy, 1);
                }
            }
        }
    }

}

/**********************************************************************/ /*
calculates IGF energies
**************************************************************************/
static void IGF_calc(IGF_DEC_PRIVATE_DATA_HANDLE                        hPrivateData,       /**< in:     | IGF private data handle                              */
                     const Word16                                       igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength */
                     const Word32                                      *spectrum,           /**< in: Q31 | MDCT spectrum                                        */
                     const Word16                                       spectrum_e,         /**< in:     | exponent of pSpectralData                            */
                     Word32                                            *igf_spec,           /**< in: Q31 | prepared IGF spectrum                                */
                     Word16                                            *igf_spec_e          /**< in:     | array exponents of igf_spec, one exponent per tile   */
                    )
{
    H_IGF_GRID hGrid;
    Word16 i;
    Word32 *igf_pN;                                     /* Q31 | processed energy                                     */
    Word16 *igf_pN_e;                                   /*     | exponents of igf_pN, one for each entry of igf_pN    */
    Word32 *igf_sN;                                     /* Q31 | survived energy                                      */
    Word16 *igf_sN_e;                                   /*     | exponents of igf_sN, one for each entry of igf_sN    */
    Word32 squaredSpectra[IGF_MAX_GRANULE_LEN];         /* Q31 | MDCT^2 spectra                                       */
    Word16 squaredSpectra_e[IGF_MAX_TILES];             /*     | exponents of squaredSpectra, one exponent per tile!  */


    /* initialize variables */
    hGrid    = &hPrivateData->igfInfo.grid[igfGridIdx];
    igf_pN   = hPrivateData->igf_pN;
    igf_pN_e = hPrivateData->igf_pN_e;
    igf_sN   = hPrivateData->igf_sN;
    igf_sN_e = hPrivateData->igf_sN_e;

    set32_fx(squaredSpectra, 0, IGF_MAX_GRANULE_LEN);
    set16_fx(squaredSpectra_e, 0, IGF_MAX_TILES);

    /* square the original spectrum */
    IGFCommonFuncsMDCTSquareSpec(hGrid->startLine,
                                 hGrid->stopLine,
                                 spectrum,
                                 spectrum_e,
                                 squaredSpectra,
                                 squaredSpectra_e,
                                 0);

    /* calculate the energy per SFB of the survied subbands */
    IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->startSfb,
                                         hGrid->stopSfb,
                                         hGrid->swb_offset,
                                         squaredSpectra,
                                         squaredSpectra_e,
                                         igf_sN,
                                         igf_sN_e);

    /* loop over tiles, every tile has his own exponent! */
    FOR (i = 0; i < hGrid->nTiles; i++)
    {
        /* square the prepared IGF spectrum */
        IGFCommonFuncsMDCTSquareSpec(hGrid->tile[i],
                                     hGrid->tile[i + 1],
                                     igf_spec,
                                     igf_spec_e[i],
                                     squaredSpectra,
                                     &squaredSpectra_e[i],
                                     0);

        /* set all squared values to 0, if the core contains survied lines */
        IGF_setLinesToZero(hGrid->tile[i],
                           hGrid->tile[i + 1],
                           spectrum,
                           squaredSpectra);

        /* calculate the energy per SFB of the processed subbands */
        IGFCommonFuncsCalcSfbEnergyPowerSpec(hGrid->sfbWrap[i],
                                             hGrid->sfbWrap[i + 1],
                                             hGrid->swb_offset,
                                             squaredSpectra,
                                             &squaredSpectra_e[i],
                                             igf_pN,
                                             igf_pN_e);
    }

}

/**********************************************************************/ /*
apply IGF
**************************************************************************/
static void IGF_appl(IGF_DEC_PRIVATE_DATA_HANDLE                        hPrivateData,       /**< in:     | IGF private data handle                              */
                     const Word16                                       igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength */
                     Word32                                            *spectrum,           /**< in: Q31 | MDCT spectrum                                        */
                     Word16                                            *spectrum_e,         /**< in:     | exponent of pSpectralData                            */
                     const Word32                                      *igf_spec,           /**< in: Q31 | prepared IGF spectrum                                */
                     const Word16                                      *igf_spec_e,         /**< in:     | array exponents of igf_spec, one exponent per tile   */
                     Word32                                            *virtualSpec,        /**< out:Q31 | virtual IGF spectrum, used for temp flattening       */
                     Word16                                            *virtualSpec_e,      /**< out:    | exponent of virtualSpec                              */
                     Word16                                            *flag_sparse         /**< out: Q0 | temp flattening indicator                            */
                    )
{
    H_IGF_GRID hGrid;
    Word16 i;
    Word16 tb;
    Word16 sfb;
    Word16 shift;
    Word16 s;
    Word16 s_sfb;
    Word16 start_sfb;
    Word16 stop_sfb;
    Word16 sfb_p1;
    Word16 sfb_m1;
    Word16 hopsize;
    Word16 sum;
    Word16 tileIdx;
    Word16 width;           /* Q0   | width of the current sfb                        */
    Word16 width_e;         /*      | exponent of widthent sfb, initialized as 15!    */
    Word16 gFactor;         /* 1Q14 | general SCF adaption                            */
    Word16 fFactor;         /* 1Q14 | first SCF adaption                              */
    Word16 lFactor;         /* 1Q14 | last SCF adaption                               */
    Word16 w0;              /* Q15  | float value: 0.201f                             */
    Word16 w1;              /* Q15  | float value: 0.389f                             */
    Word16 w2;              /* Q15  | float value: 0.410f                             */
    Word16 comp_th;         /* Q0   | compression threshold                           */
    Word16 comp_ratio;      /* Q0   | compression ratio as shift, original value: 256 */
    Word16 comp_offset;     /* Q0   | comp_offset = comp_ratio * comp_th - comp_th;   */
    Word16 dE;              /* Q31  | energy below igfBgn                             */
    Word16 dE_e ;           /*      | exponent of dE                                  */
    Word16 gn;              /* Q0   | gain read from bitstream + processing           */
    Word16 gn_e;            /*      | exponent of gn                                  */
    Word16 maxGain_e;       /*      | maximal gain exponent over sfbs                 */
    Word16 tmp;
    Word16 tmp_e;
    Word16 tmp_loop;
    Word32 L_tmp;
    Word16 L_tmp_e;
    Word32 L_tmp2;
    Word32 sNlocal;
    Word16 sNlocal_e;
    Word32 dNlocal;
    Word16 dNlocal_e;
    Word32 E;
    Word16 E_e;
    Word32 *sN;
    Word16 *sN_e;
    Word32 *pN;
    Word16 *pN_e;
    Word16 gain[IGF_MAX_SFB];
    Word16 gain_e[IGF_MAX_SFB];
    Word16 dN[IGF_MAX_SFB + 1];
    Word16 dN_e[IGF_MAX_SFB + 1];
    Word16 dS[IGF_MAX_SFB];
    Word16 dS_e[IGF_MAX_SFB];
    Word32 energyTmp[24];
    Word32 L_c;
    Word16 Hr;


    /* initialize variables */
    w0          = 6586;
    move16();
    w1          = 12747;
    move16();
    w2          = 13435;
    move16();
    comp_th     = 1;
    move16();
    comp_ratio  = 8  ;
    move16();
    comp_offset = 255;
    move16();
    dE          = 0;
    move16();
    dE_e        = 0;
    move16();
    tmp         = 0;
    move16();
    s           = 0;
    move16();
    tmp_e       = 0;
    move16();
    gn          = 0;
    move16();
    gn_e        = 0;
    move16();
    maxGain_e   = 0;
    move16();
    L_tmp_e     = 0;
    move16();
    dNlocal_e   = 0;
    move16();
    L_tmp       = L_add(0, 0);
    dNlocal     = L_add(0, 0);

    set16_fx(gain,       0, IGF_MAX_SFB);
    set16_fx(gain_e,     0, IGF_MAX_SFB);
    set16_fx(dN,         0, add(IGF_MAX_SFB,1));
    set16_fx(dN_e,       0, add(IGF_MAX_SFB,1));
    set16_fx(dS,         0, IGF_MAX_SFB);
    set16_fx(dS_e,       0, IGF_MAX_SFB);
    set32_fx(energyTmp,  0, 24);

    /* more inits */
    hGrid       = &hPrivateData->igfInfo.grid[igfGridIdx];
    sN          = hPrivateData->igf_sN;
    sN_e        = hPrivateData->igf_sN_e;
    pN          = hPrivateData->igf_pN;
    pN_e        = hPrivateData->igf_pN_e;
    start_sfb   = hGrid->startSfb;
    move16();
    stop_sfb    = hGrid->stopSfb;
    move16();
    gFactor     = hGrid->gFactor;
    move16();
    fFactor     = hGrid->fFactor;
    move16();
    lFactor     = hGrid->lFactor;
    move16();

    /* reset virtual spec */
    set16_fx(flag_sparse, 0, N_MAX);
    set32_fx(virtualSpec, 0, N_MAX);
    *virtualSpec_e = *spectrum_e;
    move16();

    /* collect energy below hGrid->startLine: */
    tmp = sub(hGrid->startLine, 24);
    IGFCommonFuncsMDCTSquareSpec(tmp,
                                 hGrid->startLine,
                                 spectrum,
                                 *spectrum_e,
                                 energyTmp,
                                 &dE_e,
                                 negate(tmp));

    L_c = L_add(0, 0);
    FOR (tb = 0; tb < 24; tb++)
    {
        Carry    = 0;
        L_tmp    = L_add_c(L_tmp, energyTmp[tb]);
        Overflow = 0;
        L_c = L_macNs(L_c, 0, 0);
    }
    L_tmp = norm_llQ31(L_c, L_tmp, &shift);
    /* float: dE = (float)sqrt(dE / 24.f); basop: */
    shift = add(sub(shift, 4), dE_e);                                         /* x/24 = (x >> 4) * 1/1.5 */
    dE    = Sqrt16norm(extract_h(L_tmp), &shift);
    dE    = mult_r(dE, FL2WORD16(0.81649658092772603273242802490196f));       /* 0.81649658092772603273242802490196f = sqrt(1/1.5)) */
    dE_e  = shift;
    move16();

    /* select correct hopsize for envelope refinement */
    hopsize = 2;
    move16();
    if (sub(hPrivateData->currWhiteningLevel[0], IGF_WHITENING_OFF) == 0)
    {
        hopsize = 4;
        move16();
    }
    if (sub(hPrivateData->currWhiteningLevel[0], IGF_WHITENING_STRONG) == 0)
    {
        hopsize = 1;
        move16();
    }
    hopsize = s_min(hopsize, hPrivateData->igfInfo.maxHopsize);

    IF (hopsize > 1)
    {
        FOR (sfb = start_sfb; sfb < stop_sfb; sfb += hopsize)
        {
            tmp_loop = s_min(add(sfb, hopsize), stop_sfb);
            FOR (tb = add(sfb, 1); tb < tmp_loop; tb++)
            {
                sN[sfb] = BASOP_Util_Add_Mant32Exp(sN[sfb],
                                                   sN_e[sfb],
                                                   sN[tb],
                                                   sN_e[tb],
                                                   &sN_e[sfb]);
                move32();
                pN[sfb] = BASOP_Util_Add_Mant32Exp(pN[sfb],
                                                   pN_e[sfb],
                                                   pN[tb],
                                                   pN_e[tb],
                                                   &pN_e[sfb]);
                move32();
                sN[tb]  = L_deposit_l(0);
                pN[tb]  = L_deposit_l(0);
            }
        }
    }

    /* IGF_rescale_SCF */
    IF (hGrid->infoIsRefined != 0)
    {
        FOR (sfb = start_sfb; sfb < stop_sfb; sfb += 2)
        {
            /* calculate and normalize the width of the current sfb */
            width   = sub(hGrid->swb_offset[sfb + 2], hGrid->swb_offset[sfb]);
            shift   = norm_s(width);
            width   = shl(width, shift);
            width_e = sub(15, shift);                                         /* initial value of width_e is 15, -> width = 15Q0 */

            /* float: gn = 0.25f * igf_curr - 4.f; basop: */
            gn   = hPrivateData->igf_curr[sfb >> 1];
            move16();
            move16();
            gn_e = 13;                                                        /* set exponent of igf_curr to 13 = 15 - 2; -> igf_curr = igf_curr * 0.25, virtual division by 4 */
            gn   = sub(gn, 16);                                               /* 13Q2 | 4 = 16 * 2^(-15 + 13); ("4" has same exponent as igf_curr now) */

            /* float: tmp = pow(2.f, gn); basop: */
            L_tmp = BASOP_util_Pow2(L_deposit_h(gn), gn_e, &L_tmp_e);

            /* float: tmp = tmp * tmp; basop: */
            tmp     = round_fx(L_tmp);
            L_tmp   = L_mult(tmp, tmp);
            L_tmp_e = add(L_tmp_e, L_tmp_e);

            /* get sNlocal | float: sNlocal = sN[ sfb ] + sN[ sfb+ 1 ]; basop: */
            sNlocal = BASOP_Util_Add_Mant32Exp(sN[sfb],
                                               sN_e[sfb],
                                               sN[sfb + 1],
                                               sN_e[sfb + 1],
                                               &sNlocal_e);

            /* float: sNlocal /= width; basop: */
            shift     = sub(norm_l(sNlocal), 1);                              /* leave MSB empty, so in the division sNlocal is always smaller than width */
            sNlocal   = L_deposit_h(div_s(extract_h(L_shl(sNlocal, shift)), width));
            sNlocal_e = sub(sub(sNlocal_e, shift), width_e);

            /* float: tmp  = max(0.001 * sNlocal, tmp - sNlocal); basop: */
            L_tmp   = BASOP_Util_Add_Mant32Exp(L_tmp,
                                               L_tmp_e,
                                               L_negate(sNlocal),
                                               sNlocal_e,
                                               &L_tmp_e);                      /* float: tmp = tmp - sNlocal */

            /* max(0.001 * sNlocal, L_tmp) */
            /* Build a threshold and compare with L_tmp.
               Build negated threshold and compare with negated L_tmp to cover also fullscale L_tmp case */
            BASOP_SATURATE_WARNING_OFF
            L_tmp2 = L_shl(L_negate(Mpy_32_16_1(sNlocal, FL2WORD16(0.001f))), sub(sNlocal_e, L_tmp_e));
            L_tmp2 = L_sub(L_tmp2, L_negate(L_tmp));
            BASOP_SATURATE_WARNING_ON

            IF (L_tmp2 < 0)
            {
                L_tmp   = Mpy_32_16_1( sNlocal,FL2WORD16(0.001f));
                L_tmp_e = sNlocal_e;
                move16();
            }

            /* calc square root of L_tmp and store result in dN */
            L_tmp       = Sqrt32(L_tmp, &L_tmp_e);
            dN[sfb]     = round_fx(L_tmp);
            dN_e[sfb]   = L_tmp_e;
            move16();
            dN[sfb+1]   = dN[sfb];
            move16();
            dN_e[sfb+1] = dN_e[sfb];
            move16();
        }
    }
    ELSE
    {
        FOR (sfb = start_sfb; sfb < stop_sfb; sfb++)
        {
            /* calculate and normalize the width of the current sfb */
            width   = sub(hGrid->swb_offset[sfb + 1], hGrid->swb_offset[sfb]);
            shift   = norm_s(width);
            width   = shl(width, shift);
            width_e = sub(15, shift);                                         /* initial value of width_e is 15, -> width = 15Q0 */

            /* float: gn = 0.25f * igf_curr - 4.f; basop: */
            gn   = hPrivateData->igf_curr[sfb];
            move16();
            move16();
            gn_e = 13;                                                        /* set exponent of igf_curr to 13 = 15 - 2; -> igf_curr = igf_curr * 0.25, virtual division by 4 */
            gn   = sub(gn, 16);                                               /* 13Q2 | 4 = 16 * 2^(-15 + 13); ("4" has same exponent as igf_curr now) */

            /* float: tmp = pow(2.f, gn); basop: */
            L_tmp = BASOP_util_Pow2(L_deposit_h(gn), gn_e, &L_tmp_e);

            /* float: tmp = tmp * tmp; basop: */
            tmp     = round_fx(L_tmp);
            L_tmp   = L_mult(tmp, tmp);
            L_tmp_e = add(L_tmp_e, L_tmp_e);

            /* get sNlocal */
            sNlocal   = sN[sfb];
            move32();
            sNlocal_e = sN_e[sfb];
            move16();

            /* float: sNlocal /= width; basop: */
            shift     = sub(norm_l(sNlocal), 1);                    /* leave MSB empty, so in the division sNlocal is always smaller than width */
            sNlocal   = L_deposit_h(div_s(extract_h(L_shl(sNlocal, shift)), width));
            sNlocal_e = sub(sub(sNlocal_e, shift), width_e);

            /* float: tmp  = max(0.001 * sNlocal, tmp - sNlocal); basop: */
            L_tmp   = BASOP_Util_Add_Mant32Exp(L_tmp,
            L_tmp_e,
            L_negate(sNlocal),
            sNlocal_e,
            &L_tmp_e);                      /* float: tmp = tmp - sNlocal */

            /* max(0.001 * sNlocal, L_tmp) */
            /* Build a threshold and compare with L_tmp.
               Build negated threshold and compare with negated L_tmp to cover also fullscale L_tmp case */
            BASOP_SATURATE_WARNING_OFF
            L_tmp2 = L_shl(L_negate(Mpy_32_16_1(sNlocal, FL2WORD16(0.001f))), sub(sNlocal_e,L_tmp_e));
            L_tmp2 = L_sub(L_tmp2, L_negate(L_tmp));
            BASOP_SATURATE_WARNING_ON

            IF (L_tmp2 < 0 )
            {
                L_tmp   = Mpy_32_16_1(sNlocal,FL2WORD16(0.001f));
                L_tmp_e = sNlocal_e;
            }

            /* calc square root of L_tmp and store result in dN */
            L_tmp     = Sqrt32(L_tmp, &L_tmp_e);
            dN[sfb]   = round_fx(L_tmp);
            dN_e[sfb] = L_tmp_e;
            move16();
        }
    }

    dS[start_sfb]   = dN[start_sfb];
    move16();
    dS_e[start_sfb] = dN_e[start_sfb];
    move16();

    /* first value with adaption to core energy: */
    tmp_e = BASOP_Util_Add_MantExp(dE,
                                   dE_e,
                                   negate(dN[start_sfb]),
                                   dN_e[start_sfb],
                                   &tmp);                                      /* float: tmp = dE - dN[start_sfb] */
    IF (tmp < 0)
    {
        /* float: dS[start_sfb] = dN[start_sfb] + fFactor * (dE-dN[start_sfb]); basop: */
        L_tmp = L_mult(fFactor, tmp);
        L_tmp_e = add(tmp_e, 1);                                              /* 1Q14 | fFactor is 1Q14 */
        dS_e[start_sfb] = BASOP_Util_Add_MantExp(dN[start_sfb],
                          dN_e[start_sfb],
                          round_fx(L_tmp),
                          L_tmp_e,
                          &dS[start_sfb]);
        move16();
    }
    /* last value with less energy: */
    dS[stop_sfb - 1]   = mult_r(lFactor, dN[stop_sfb - 1]);
    move16();
    move16();
    dS_e[stop_sfb - 1] = add(dN_e[stop_sfb - 1], 1);                          /* 1Q14 | lFactor is 1Q14 */

    sfb_p1 = add(start_sfb, 1);
    sfb_m1 = sub(stop_sfb, 1);
    test();
    IF (hGrid->infoIsRefined != 0 && sub(hopsize, 1) == 0)
    {
        /* apply filter to absolute energy values: */
        FOR (sfb = sfb_p1; sfb < sfb_m1; sfb++)
        {
            /* float: dS[sfb] = w0 * dN[sfb-1] + w1 * dN[sfb+0] + w2 * dN[sfb+1]; basop: */
            L_tmp     = L_mult(w0, dN[sfb - 1]);
            dS[sfb]   = round_fx(L_tmp);
            move16();
            dS_e[sfb] = dN_e[sfb-1];                                          /* w0 is Q15, so no need to add an exponent */
            L_tmp     = L_mult(w1, dN[sfb]);
            dS_e[sfb] = BASOP_Util_Add_MantExp(dS[sfb],
                                               dS_e[sfb],
                                               round_fx(L_tmp),
                                               dN_e[sfb],                     /* w1 is Q15, so no need to add an exponent */
                                               &tmp);
            move16();
            dS[sfb]   = tmp;
            move16();
            L_tmp     = L_mult(w2, dN[sfb + 1]);
            dS_e[sfb] = BASOP_Util_Add_MantExp(dS[sfb],
                                               dS_e[sfb],
                                               round_fx(L_tmp),
                                               dN_e[sfb + 1],                 /* w2 is Q15, so no need to add an exponent */
                                               &tmp);
            move16();
            dS[sfb]   = tmp;
            move16();
        }
    }
    ELSE
    {
        FOR (sfb = sfb_p1; sfb < sfb_m1; sfb++)
        {
            dS[sfb]   = dN[sfb];
            move16();
            dS_e[sfb] = dN_e[sfb];
            move16();
        }
    }

    Hr      = 0;
    move16();
    tileIdx = -1;
    move16();
    FOR (sfb = start_sfb; sfb < stop_sfb; sfb += hopsize)
    {
        E   = L_add(0, 0);
        E_e = 0;
        move16();
        sum = 0;
        move16();

        FOR (tb = 0; tb < hopsize; tb++)
        {
            /* calculate of the current sfb width */
            width = sub(hGrid->swb_offset[s_min(add(add(sfb, tb), 1), stop_sfb)],   /* 15Q0 | width is Q0 */
                        hGrid->swb_offset[s_min(add(sfb, tb),         stop_sfb)]);

            tmp   = dS[s_min(add(sfb, tb), sub(stop_sfb, 1))];
            tmp_e = dS_e[s_min(add(sfb, tb), sub(stop_sfb, 1))];

            /* square tmp */
            L_tmp   = L_mult(tmp, tmp);
            L_tmp_e = add(tmp_e, tmp_e);

            /* mult L_tmp times width */
            L_tmp   = L_mult(round_fx(L_tmp), width);
            L_tmp_e = add(L_tmp_e, 15);                                       /* 15Q0 | width is Q0 */

            /* calculate resulting energy */
            E       = BASOP_Util_Add_Mant32Exp(E,
                                               E_e,
                                               L_tmp,
                                               L_tmp_e,
                                               &E_e);
            sum   = add(sum, width);                                          /* 15Q0 | sum shares its exponent with width */
        }

        /* normalize sum for the following division */
        shift     = norm_s(sum);
        sum       = shl(sum, shift);                                          /* exponent of sum: sub(15, shift) */

        /* divide E by sum */
        tmp       = div_s(shr(round_fx(E), 1), sum);                          /* shift E 1 bit to the right in order to make it smaller than sum */
        tmp_e     = sub(add(E_e, 1), sub(15, shift));                         /* 15Q0 | sum is 15Q0 */

        /* multiply the result by the hopsize */
        L_tmp     = L_mult(tmp, hopsize);
        L_tmp_e   = add(tmp_e, 15);                                           /* 15Q0 | hopsize is 15Q0 */

        /* take the square root and store the result in dS */
        L_tmp     = Sqrt32(L_tmp, &L_tmp_e);
        dS[sfb]   = round_fx(L_tmp);
        dS_e[sfb] = L_tmp_e;
        move16();

        /* calculate the new dN */
        dN[sfb] = mult_r(gFactor, dS[sfb]);
        move16();
        move16();
        dN_e[sfb] = add(dS_e[sfb], 1);                                        /* 1Q14 | gFactor is 1Q14 */

        /* calculate of the current sfb width */
        width = sub(hGrid->swb_offset[sfb + 1],                               /* 15Q0 | width is Q0 */
                    hGrid->swb_offset[sfb]);

        /* square dN */
        L_tmp     = L_mult(dN[sfb], dN[sfb]);
        L_tmp_e   = add(dN_e[sfb], dN_e[sfb]);

        /* mult L_tmp times width */
        shift     = norm_l(L_tmp);
        L_tmp     = L_shl(L_tmp, shift);
        L_tmp     = L_mult(round_fx(L_tmp), width);
        L_tmp_e   = sub(add(L_tmp_e, 15), shift);                             /* 15Q0 | width is Q0 */
        shift     = norm_l(L_tmp);

        /* store normalized result */
        dNlocal   = L_shl(L_tmp, shift);
        dNlocal_e = sub(L_tmp_e, shift);

        /* gain calculation */
        gain[sfb] = 0;
        move16();
        IF (pN[sfb] != 0)
        {
            tmp         = BASOP_Util_Divide3232_Scale(dNlocal, pN[sfb],&s);
            s           = sub(add(s, dNlocal_e), pN_e[sfb]);
            gain[sfb]   = Sqrt16(tmp, &s);
            move16();
            gain_e[sfb] = s;
            move16();

            /* apply gain compression, comp_th is 15Q0 */
            tmp = s_min(15, sub(15, gain_e[sfb]));                            /* 15Q0 | shift for the compressor constants which are in 15Q0 */
            BASOP_SATURATE_WARNING_OFF
            tmp = shl(comp_th, tmp);                                          /*      | tmp may saturate (tmp is the shifted compressor threshold):                         */
            /*      | if gain -> 1, no overflow will occur                                                */
            /*      |  because gain_e = [15, 0] otherwise: gain >= 2(gain_e >= 16), gain < 1 (gain_e <=0) */
            /*      | if gain -> 0, overflow of tmp may occurr, but it doesn't matter:                    */
            /*      |  then "if ((tmp = 0x7fff) > gain)" results the same as "if ((tmp = 1) > gain)"      */
            /*      |  because gain is smaller than 1 and therefore smaller than 0x7fff                   */
            /*      | if gain -> Inf, overflow may occur, but ut doesn't matter:                          */
            /*      |  then "if ((tmp = 0x0000) > gain)" results the same as "if ((tmp = 1) > gain)"      */
            /*      |  because gain is larger than 1 and therefore larger than 0x0000                     */
            BASOP_SATURATE_WARNING_ON
            IF (sub(gain[sfb], tmp) > 0)
            {
                gain_e[sfb] = BASOP_Util_Add_MantExp(gain[sfb],
                                                     gain_e[sfb],
                                                     comp_offset,
                                                     15,
                                                     &gain[sfb]);
                move16();
                gain_e[sfb] = sub(gain_e[sfb], comp_ratio);
                move16();
            }

            /* get the maximal exponent of the gain array, needed for exponent adjustment of the spectrum */
            maxGain_e = s_max(maxGain_e, gain_e[sfb]);
        }
        sfb_p1 = add(sfb, 1);
        sfb_m1 = s_min(add(sfb, hopsize), stop_sfb);
        FOR (s_sfb = sfb_p1; s_sfb < sfb_m1; s_sfb++)
        {
            gain[s_sfb]   = gain[sfb];
            move16();
            gain_e[s_sfb] = gain_e[sfb];
            move16();
        }

        /*--- check gains /spectrum exponents for possible overflows --- */
        /* get tile index */
        if (sub(hGrid->sfbWrap[tileIdx + 1], sfb) <= 0)
        {
            tileIdx = add(tileIdx, 1);
        }
        /*do a test multiplication with the highest possible value*/
        L_tmp             = Mpy_32_16_1(0xFFFF8000/*igf_spec occupies only the 16LSBs */, gain[sfb]);
        L_tmp_e           = add(igf_spec_e[tileIdx], gain_e[sfb]);
        /*check whether overflow would occur and calculate Headroom, needed*/
        shift             = sub(L_tmp_e, *spectrum_e);
        tmp               = sub(shift , sub(norm_l(L_tmp), TCX_IMDCT_HEADROOM));
        if (tmp > 0)
        {
            Hr = s_max(Hr, tmp);
        }

        /* disable rescaling if gain is smaler than 1     */
        /* gain < 1, if norm_s(gain[sfb]) >= gain_e[sfb]  */
        tmp = sub(norm_s(gain[sfb]), gain_e[sfb]);
        if (tmp >= 0)
        {
            Hr = 0;
            move16();
        }
    }

    /* Rescale spectrum if overflow may occur */
    tileIdx = -1;
    move16();
    IF (Hr > 0)
    {
        /* rescale virtual Spec, cheap and easy: reset scalingfactor */
        *virtualSpec_e = add(*virtualSpec_e, Hr);
        move16();

        /* rescale spectrum */
        FOR (i = 0; i < hGrid->stopLine; i++)
        {
            spectrum[i] = L_shr(spectrum[i], Hr);
            move16();
        }
        *spectrum_e = add(*spectrum_e, Hr);
        move16();
    }

    /* tiling */
    tileIdx = -1;
    move16();
    FOR (sfb = start_sfb; sfb < stop_sfb; sfb++)
    {
        /* get tile index */
        if (sub(hGrid->sfbWrap[tileIdx + 1], sfb) == 0)
        {
            tileIdx = add(tileIdx, 1);
        }

        IF (hPrivateData->frameLossCounter > 0)
        {
            /* normalize gain */
            tmp         = norm_s(gain[sfb]);
            gain[sfb]   = shl(gain[sfb], tmp);
            gain_e[sfb] = sub(gain_e[sfb], tmp);

            /* gain[sfb] = min(gain[sfb], 12.f); */
            BASOP_SATURATE_WARNING_OFF                                        /* threshold, may overflow */
            tmp = shl(gain[sfb], sub(gain_e[sfb], 15 - 5));                   /* 10Q5 | tmp is in 10Q5 */
            BASOP_SATURATE_WARNING_ON

            IF (tmp > 384)                                                    /* 10Q5 | 384 = 12 in 10Q5 */
            {
                gain[sfb]   = 384;
                move16();
                gain_e[sfb] = 10;
                move16();
            }

            IF (sub(hPrivateData->frameLossCounter, 5) < 0)
            {
                /* gain[sfb] -= gain[sfb] / 8 * hPrivateData->frameLossCounter; -> multiply with 0Q15 -> adaption of the exponent not needed */
                IF (sub(hPrivateData->frameLossCounter, 1) == 0)
                {
                    /* 0Q15 | >> 3 ^= * 0.125 = 1 / 8 */
                    gain[sfb] = sub(gain[sfb], shr_r(gain[sfb], 3));
                    move16();
                }
                ELSE IF (sub(hPrivateData->frameLossCounter, 2) == 0)
                {
                    /* 0Q15 | >> 2 ^= * 0.25 = 2 / 8 */
                    gain[sfb] = sub(gain[sfb], shr_r(gain[sfb], 2));
                    move16();
                }
                ELSE IF (sub(hPrivateData->frameLossCounter, 3) == 0)
                {
                    /* 0Q15 | * 12288 ^= * 0.3750 = 3 / 8 */
                    gain[sfb] = sub(gain[sfb], mult_r(gain[sfb], 12288));
                    move16();
                }
                ELSE
                {
                    /* 0Q15 | >> 1 ^= * 0.5 = 4 / 8 */
                    gain[sfb] = sub(gain[sfb], shr_r(gain[sfb], 1));
                    move16();
                }
            }
            ELSE
            {
                /* gain[sfb] /= 2; -> reduce exponent by 1 */
                gain_e[sfb] = sub(gain_e[sfb], 1);
                move16();
            }
        }

        FOR (tb = hGrid->swb_offset[sfb]; tb < hGrid->swb_offset[sfb + 1]; tb++)
        {
            /* multiply the prepared IGF spectrum with the gain */
            L_tmp2            = L_add(0, 0);                                  /* set L_tmp2 to default value */
            L_tmp             = Mpy_32_16_1(igf_spec[tb], gain[sfb]);
            L_tmp_e           = add(igf_spec_e[tileIdx], gain_e[sfb]);

            /* store the finalized IGF spectrum */
            IF (spectrum[tb] == 0)
            {
                shift           = sub(L_tmp_e, *spectrum_e);
                tmp             = norm_l(L_tmp) - shift - 32;
                if (tmp < 0)
                {
                    L_tmp2      = L_shl(L_tmp, shift);
                }
                spectrum[tb]    = L_tmp2;
                move32();
                flag_sparse[tb] = 1;
                move16();
            }
            ELSE
            {
                shift           = sub(L_tmp_e, *virtualSpec_e);
                tmp             = norm_l(L_tmp) - shift - 32;
                if (tmp < 0)
                {
                    L_tmp2      = L_shl(L_tmp, shift);
                }
                virtualSpec[tb] = L_tmp2;
                move32();
                flag_sparse[tb] = 2;
                move16();
            }
        }
    }

}

/**********************************************************************/ /*
spectral whitening
**************************************************************************/
static void IGF_getWhiteSpectralData(const Word32                      *in,                 /**< in: Q31 | MDCT spectrum              */
                                     Word16                             s_l,                /**< in: Q0  | getScaleFactor32() of in   */
                                     Word32                            *out,                /**< out: Q31| whitened spectrum          */
                                     const Word16                       start,              /**< in: Q0  | start MDCT subband index   */
                                     const Word16                       stop,               /**< in: Q0  | stop MDCT subband index    */
                                     const Word16                       level               /**< in: Q0  | whitening strength         */
                                    )
{
    Word16 j;
    Word32 ak;                                                  /* moving average */
    Word16 tmp_16;
    Word16 s;
    Word16 div;
    Word16 nrm_i;
    Word16 nrm_tab[] = {17, 18, 20, 22, 24, 27, 30, 34};        /* ??? */


    /* inits */
    div = 0;
    move16();
    s_l = sub(s_l, 2);
    ak  = L_add(0, 0);


    FOR (j = start - level; j < start + level; j++)
    {
        tmp_16 = extract_h(L_shl(in[j], s_l));
        ak     = L_mac(ak, tmp_16, tmp_16);
    }
    FOR (j = start; j < stop - level; j++)
    {
        tmp_16 = extract_h(L_shl(in[j + level], s_l));
        ak     = L_mac(ak, tmp_16, tmp_16);
        tmp_16 = sub(31 - 1 - 1, norm_l(ak));

        if (ak == 0)
        {
            tmp_16 = 0;
            move16();
        }

        tmp_16 = s_min(14, sub(15, shr(tmp_16, 1)));
        div    = shl(1, tmp_16);
        out[j] = L_mult(extract_h(L_shl(in[j], s_l)), div);
        move32();
        tmp_16 = extract_h(L_shl(in[j - level], s_l));
        ak     = L_msu(ak, tmp_16, tmp_16);
    }

    nrm_i = 0;
    move16();

    FOR (; j < stop; j++)
    {
        s      = norm_l(ak);
        tmp_16 = sub(31 - 1 - 1, norm_l(L_shl(L_mult(extract_h(L_shl(ak, sub(s, 4))), nrm_tab[nrm_i++]), sub(15, s))));
        tmp_16 = s_min(14, sub(15, shr(tmp_16, 1)));
        div    = shl(1, tmp_16);

        if (L_sub(ak, 16) < 0)
        {
            div = 1;
            move16();
        }

        out[j] = L_mult(extract_h(L_shl(in[j], s_l)), div);
        move32();
        tmp_16 = extract_h(L_shl(in[j - level], s_l));
        ak     = L_msu(ak, tmp_16, tmp_16);
    }

}

/**********************************************************************/ /*
refines the IGF grid
**************************************************************************/
static void IGF_RefineGrid(H_IGF_GRID                                   hGrid               /**< in/out: | IGF grid handle  */
                          )
{
    Word16 a[IGF_MAX_SFB+1];
    Word16 sfb;
    Word16 tmp;
    Word16 delta;


    set16_fx(a, 0, IGF_MAX_SFB+1);


    hGrid->infoIsRefined = 1;
    move16();
    FOR (sfb = 0; sfb < hGrid->swb_offset_len; sfb++)
    {
        tmp    = shl(sfb, 1);
        a[tmp] = hGrid->swb_offset[sfb];
        move16();
        tmp    = add(tmp, 1);
        delta  = sub(hGrid->swb_offset[sfb+1], hGrid->swb_offset[sfb]);
        delta  = mac_r(0x00195000, FL2WORD16_SCALE(0.45f, -1), shl(delta, 5));
        a[tmp] = add(hGrid->swb_offset[sfb], shr(delta, 6));
        move16();
        if (s_and(a[tmp], 1) != 0)
        {
            a[tmp] = sub(a[tmp], 1);
            move16();
        }
    }
    hGrid->stopSfb = shl(hGrid->stopSfb, 1);
    FOR (sfb = 0; sfb <= hGrid->stopSfb; sfb++)
    {
        hGrid->swb_offset[sfb] = a[sfb];
        move16();
    }

    FOR (sfb = 0; sfb <= hGrid->nTiles; sfb++)
    {
        hGrid->sfbWrap[sfb]    = shl(hGrid->sfbWrap[sfb], 1);
        move16();
    }

}

/**********************************************************************/ /*
reads whitening information from the bitstream
**************************************************************************/
void IGFDecReadData(const IGF_DEC_INSTANCE_HANDLE                    hInstance,          /**< in:     | instance handle of IGF Deccoder                      */
                    Decoder_State_fx                                *st,                 /**< in:     | decoder state                                        */
                    const Word16                                     igfGridIdx,         /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength */
                    const Word16                                     isIndepFrame        /**< in: Q0  | if 1: arith dec force reset, if 0: no reset          */
                   )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 p;
    Word16 nT;
    Word16 tmp;


    IF (hInstance != NULL)
    {
        hPrivateData = &hInstance->igfData;
        hGrid        = &hPrivateData->igfInfo.grid[igfGridIdx];
        nT           = hGrid->nTiles;
        move16();
        tmp          = 0;
        move16();

        /* set/reset all values to default = IGF_WHITENING_OFF */
        FOR (p = 0; p < IGF_MAX_TILES; p++)
        {
            hPrivateData->currWhiteningLevel[p] = IGF_WHITENING_OFF;
            move16();
        }

        IF (isIndepFrame == 0)
        {
            tmp = get_next_indice_fx(st, 1);
        }

        IF (tmp == 1)
        {
            FOR (p = 0; p < nT; p++)
            {
                hPrivateData->currWhiteningLevel[p] = hPrivateData->prevWhiteningLevel[p];
                move16();
            }
        }
        ELSE
        {
            IGF_decode_whitening_level(st, hPrivateData, 0);
            tmp = get_next_indice_fx(st, 1);
            IF (tmp == 1)
            {
                FOR (p = 1; p < nT; p++)
                {
                    IGF_decode_whitening_level(st, hPrivateData, p);
                }
            }
            ELSE
            {
                FOR (p = 1; p < nT; p++)
                {
                    hPrivateData->currWhiteningLevel[p] = hPrivateData->currWhiteningLevel[0];
                    move16();
                }
            }
        }

        /* save current level for concealment */
        FOR (p = 0; p < IGF_MAX_TILES; p++)
        {
            hPrivateData->prevWhiteningLevel[p] = hPrivateData->currWhiteningLevel[p];
            move16();
        }

        /* read flattening trigger from bitstream */
        IGF_decode_temp_flattening_trigger(st, hInstance);
    }

}

/**********************************************************************/ /*
read the IGF level information from the bitsream
**************************************************************************/
void IGFDecReadLevel(                                                                    /**< out: Q0 | return igfAllZero flag indicating if no envelope is transmitted  */
    const IGF_DEC_INSTANCE_HANDLE                 hInstance,            /**< in:     | instance handle of IGF Decoder                                   */
    Decoder_State_fx                             *st,                   /**< in:     | decoder state                                                    */
    const Word16                                  igfGridIdx,           /**< in: Q0  | in case of CELP->TCX switching, use 1.25 framelength             */
    const Word16                                  isIndepFrame          /**< in: Q0  | if 1: arith dec force reset, if 0: no reset                      */
)
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 m_igfSfbStart;
    Word16 IGFAllZero;

    IGFAllZero = 1;
    move16();

    IF (hInstance != NULL)
    {
        hPrivateData  = &hInstance->igfData;
        hGrid         = &hPrivateData->igfInfo.grid[igfGridIdx];
        m_igfSfbStart = hGrid->startSfb;
        move16();
        IGFAllZero    = get_next_indice_fx(st, 1);

        IF (IGFAllZero == 0)
        {
            Copy(hPrivateData->igf_curr, hPrivateData->igf_prev, hGrid->stopSfb);
            IGFSCFDecoderDecode(&hPrivateData->hArithSCFdec,
                                st,
                                &hPrivateData->igf_curr[m_igfSfbStart],                /* 0Q15, hPrivateData->igf_curr = [0, 91] */
                                isIndepFrame
                               );
        }
        ELSE
        {
            IGFSCFDecoderReset(&hPrivateData->hArithSCFdec);
            set16_fx(&hPrivateData->igf_curr[m_igfSfbStart], 0, sub(hGrid->stopSfb, m_igfSfbStart));
        }
    }

    hInstance->infoIGFAllZero = IGFAllZero;
    move16();
}

/**********************************************************************/ /*
apply the IGF decoder
**************************************************************************/
void IGFDecApplyMono(const IGF_DEC_INSTANCE_HANDLE                   hInstance,          /**< in:     | instance handle of IGF Decoder                       */
                     Word32                                         *spectrum,           /**< in/out: | MDCT spectrum                                        */
                     Word16                                         *spectrum_e,         /**< in/out: | exponent of spectrum                                 */
                     const Word16                                    igfGridIdx,         /**< in:     | in case of CELP->TCX switching, use 1.25 framelength */
                     Word16                                          bfi                 /**< in:     | frame loss == 1, frame good == 0                     */
                    )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;
    Word16 i;
    Word16 whiteningLevel;
    Word16 s_l;                               /*      | headroom of pSpecFlat                         */
    Word16 specMed_e;                         /*      | exponent of the medium whitened spectrum      */
    Word32 igf_spec[IGF_MAX_GRANULE_LEN];     /* Q31  | prepared IGF spectrum                         */
    Word16 igf_spec_e[IGF_MAX_TILES];         /*      | exponents of igf_spec, one exponent per tile  */


    hPrivateData = &hInstance->igfData;
    hGrid        = &hPrivateData->igfInfo.grid[igfGridIdx];

    /* initialize variables */
    whiteningLevel                          = 7;
    move16();
    specMed_e                               = 0;
    move16();
    hPrivateData->n_noise_bands             = 0;
    move16();
    hPrivateData->n_noise_bands_off         = 0;
    move16();
    hPrivateData->headroom_TCX_noise_white  = 0;
    move16();
    hPrivateData->headroom_TCX_noise        = 0;
    move16();
    hPrivateData->totalNoiseNrg             = L_add(0, 0);
    hPrivateData->totalNoiseNrg_off         = L_add(0, 0);

    set32_fx(igf_spec,   0, IGF_MAX_GRANULE_LEN);
    set16_fx(igf_spec_e, 0, IGF_MAX_TILES);

    /* concealment counter */
    IF (bfi != 0)
    {
        hPrivateData->frameLossCounter = add(hPrivateData->frameLossCounter, 1);
    }
    ELSE
    {
        hPrivateData->frameLossCounter = 0;
    }

    /* skip IGF processing if all IGF levels are zero */
    IF (hInstance->infoIGFAllZero == 0)
    {


        FOR (i = 0; i < hGrid->nTiles; i++)
        {
            IF (sub(hPrivateData->currWhiteningLevel[i], IGF_WHITENING_MID) == 0)
            {
                s_l = getScaleFactor32(hPrivateData->pSpecFlat + hGrid->minSrcSubband - whiteningLevel,
                                       add(sub(hGrid->startLine, hGrid->minSrcSubband), whiteningLevel));

                specMed_e = hPrivateData->pSpecFlat_exp;
                move16();
                IGF_getWhiteSpectralData(hPrivateData->pSpecFlat,
                                         s_l,
                                         igf_spec,
                                         hGrid->minSrcSubband,
                                         hGrid->startLine,
                                         whiteningLevel);

                /*14 seems to be precise enough*/
                hPrivateData->headroom_TCX_noise_white = IGF_getScaleFactor32Cond(hInstance->infoTCXNoise + hGrid->minSrcSubband,
                        igf_spec + hGrid->minSrcSubband,
                        sub(hGrid->startLine, hGrid->minSrcSubband));
                hPrivateData->n_noise_bands = IGF_replaceTCXNoise_1(igf_spec,
                                              hPrivateData->headroom_TCX_noise_white,
                                              hInstance->infoTCXNoise,
                                              hGrid->minSrcSubband,
                                              hGrid->startLine,
                                              &hPrivateData->totalNoiseNrg);

                BREAK;
            }
        }

        FOR (i = 0; i < hGrid->nTiles; i++)
        {
            IF (hPrivateData->currWhiteningLevel[ i ] == IGF_WHITENING_OFF)
            {
                hPrivateData->headroom_TCX_noise = IGF_getScaleFactor32Cond(hInstance->infoTCXNoise + hGrid->minSrcSubband,
                                                   hPrivateData->pSpecFlat + hGrid->minSrcSubband,
                                                   sub(hGrid->startLine, hGrid->minSrcSubband));

                hPrivateData->n_noise_bands_off = IGF_replaceTCXNoise_1(hPrivateData->pSpecFlat,
                                                  hPrivateData->headroom_TCX_noise,
                                                  hInstance->infoTCXNoise,
                                                  hGrid->minSrcSubband,
                                                  hGrid->startLine,
                                                  &hPrivateData->totalNoiseNrg_off);
                BREAK;
            }
        }

        /* apply IGF in three steps: */
        IGF_prep(hPrivateData,
                 igfGridIdx,
                 hInstance->infoTCXNoise,
                 igf_spec,
                 igf_spec_e,
                 hPrivateData->pSpecFlat,
                 hPrivateData->pSpecFlat_exp,
                 specMed_e);
        IGF_calc(hPrivateData,
                 igfGridIdx,
                 spectrum,
                 *spectrum_e,
                 igf_spec,
                 igf_spec_e);
        IGF_appl(hPrivateData,
                 igfGridIdx,
                 spectrum,
                 spectrum_e,
                 igf_spec,
                 igf_spec_e,
                 hInstance->virtualSpec,
                 &hInstance->virtualSpec_e,
                 hInstance->flag_sparse);

    }

    /* reset TCX noise indicator vector */
    set16_fx(hInstance->infoTCXNoise, 0, hGrid->infoGranuleLen);

}

/**********************************************************************/ /*
set mode is used to init the IGF dec with a new bitrate
**************************************************************************/
void IGFDecSetMode(const IGF_DEC_INSTANCE_HANDLE                     hInstance,          /**< in:     | instance handle of IGF Decoder */
                   const Word32                                      bitRate,            /**< in: Q0  | bitrate                        */
                   const Word16                                      mode,               /**< in: Q0  | bandwidth mode                 */
                   const Word16                                      defaultStartLine,   /**< in: Q0  | default start subband index    */
                   const Word16                                      defaultStopLine     /**< in: Q0  | default stop subband index     */
                   , const Word16                                      rf_mode             /**< in:     | flag to signal the RF mode */
                  )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;


    hPrivateData           = &hInstance->igfData;
    hInstance->isIGFActive = 0;
    move16();

    IF (IGFCommonFuncsIGFConfiguration(bitRate, mode, &hPrivateData->igfInfo, rf_mode) != 0)
    {
        IGFSCFDecoderOpen(&hPrivateData->hArithSCFdec,
                          sub(hPrivateData->igfInfo.grid[0].stopSfb, hPrivateData->igfInfo.grid[0].startSfb),
                          bitRate,
                          mode
                          ,rf_mode
                         );

        hInstance->infoIGFStopLine   = hPrivateData->igfInfo.grid[0].stopLine;
        move16();
        hInstance->infoIGFStartLine  = hPrivateData->igfInfo.grid[0].startLine;
        move16();
        hInstance->infoIGFStopFreq   = hPrivateData->igfInfo.grid[0].stopFrequency;
        move16();
        hInstance->infoIGFStartFreq  = hPrivateData->igfInfo.grid[0].startFrequency;
        move16();
        hInstance->infoIGFAllZero    = 0;
        move16();
        hInstance->isIGFActive       = 1;
        move16();

        test();
        IF ((sub(hPrivateData->igfInfo.bitRateIndex, IGF_BITRATE_SWB_64000) <= 0) || (sub(hPrivateData->igfInfo.bitRateIndex, IGF_BITRATE_FB_64000) <= 0))
        {
            IGF_RefineGrid(&hPrivateData->igfInfo.grid[IGF_GRID_LB_NORM]);
            IGF_RefineGrid(&hPrivateData->igfInfo.grid[IGF_GRID_LB_TRAN]);
            IGF_RefineGrid(&hPrivateData->igfInfo.grid[IGF_GRID_LB_SHORT]);
        }
    }
    ELSE
    {
        hInstance->infoIGFStopLine   = defaultStopLine;
        move16();
        hInstance->infoIGFStartLine  = defaultStartLine;
        move16();
        hInstance->infoIGFStopFreq   = -1;
        move16();
        hInstance->infoIGFStartFreq  = -1;
        move16();
        fprintf(stderr,"IGFDecSetMode: initialization error!\n");
    }

}

/**********************************************************************/ /*
updates the start/stop frequency of IGF according to igfGridIdx
**************************************************************************/
void IGFDecUpdateInfo(const IGF_DEC_INSTANCE_HANDLE                  hInstance,          /**< in:     | instance handle of IGF Decoder */
                      const Word16                                   igfGridIdx          /**< in:     | IGF grid index                 */
                     )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;


    hPrivateData = &hInstance->igfData;
    IF (hInstance->isIGFActive != 0)
    {
        hGrid                       = &hPrivateData->igfInfo.grid[igfGridIdx];
        hInstance->infoIGFStartFreq = hGrid->startFrequency;
        move16();
        hInstance->infoIGFStopFreq  = hGrid->stopFrequency;
        move16();
        hInstance->infoIGFStartLine = hGrid->startLine;
        move16();
        hInstance->infoIGFStopLine  = hGrid->stopLine;
        move16();
    }
}

/**********************************************************************/ /*
copy the LPC flat spectrum to IGF buffer
**************************************************************************/
void IGFDecCopyLPCFlatSpectrum(const IGF_DEC_INSTANCE_HANDLE         hInstance,          /**< in:     | instance handle of IGF Decoder     */
                               const Word32                         *pSpectrumFlat,      /**< in: Q31 | LPC flattend spectrum from TCX dec */
                               const Word16                          pSpectrumFlat_exp,  /**< in:     | exponent of pSpectrumFlat          */
                               const Word16                          igfGridIdx          /**< in: Q0  | IGF grid index                     */
                              )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;
    H_IGF_GRID hGrid;


    IF (hInstance)
    {
        hPrivateData = &hInstance->igfData;
        hGrid        = &hPrivateData->igfInfo.grid[igfGridIdx];

        /* pSpectrumFlat_exp has to be multiplied with 1024 = 2^10 go achive proper gain values */
        hPrivateData->pSpecFlat_exp = add(pSpectrumFlat_exp, 10);
        move16();
        Copy32(pSpectrumFlat, hPrivateData->pSpecFlat, hGrid->infoGranuleLen);
    }

}

/**********************************************************************/ /*
store the IGF bitstream information for TCX10 subframes
**************************************************************************/
void IGFDecStoreTCX10SubFrameData(const IGF_DEC_INSTANCE_HANDLE      hInstance,          /**< in:     | instance handle of IGF Decoder */
                                  const Word16                       subFrameIdx         /**< in: Q0  | index of subframe              */
                                 )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;


    hPrivateData = &hInstance->igfData;

    /* store igf energies for subframe*/
    Copy(hPrivateData->igf_curr, hPrivateData->igf_curr_subframe[subFrameIdx][0], IGF_MAX_SFB);
    Copy(hPrivateData->igf_prev, hPrivateData->igf_prev_subframe[subFrameIdx], IGF_MAX_SFB);

    /* store spectral whitening information for current subframe */
    Copy(hPrivateData->currWhiteningLevel, hPrivateData->currWhiteningLevel_subframe[subFrameIdx], IGF_MAX_TILES);
    Copy(hPrivateData->prevWhiteningLevel, hPrivateData->prevWhiteningLevel_subframe[subFrameIdx], IGF_MAX_TILES);
    /* store flattening trigger for current subframe */
    hPrivateData->igf_flatteningTrigger_subframe[subFrameIdx] = hInstance->flatteningTrigger;
    move16();

}

/**********************************************************************/ /*
restore the IGF bitstream information for TCX10 subframes
**************************************************************************/
void IGFDecRestoreTCX10SubFrameData(const IGF_DEC_INSTANCE_HANDLE    hInstance,          /**< in:     | instance handle of IGF Decoder */
                                    const Word16                     subFrameIdx         /**< in: Q0  | index of subframe              */
                                   )
{
    IGF_DEC_PRIVATE_DATA_HANDLE hPrivateData;


    hPrivateData = &hInstance->igfData;

    /* store igf energies for subframe*/
    Copy(hPrivateData->igf_curr_subframe[subFrameIdx][0], hPrivateData->igf_curr, IGF_MAX_SFB);
    Copy(hPrivateData->igf_prev_subframe[subFrameIdx], hPrivateData->igf_prev, IGF_MAX_SFB);

    /* store spectral whitening information for current subframe */
    Copy(hPrivateData->currWhiteningLevel_subframe[subFrameIdx], hPrivateData->currWhiteningLevel, IGF_MAX_TILES);
    Copy(hPrivateData->prevWhiteningLevel_subframe[subFrameIdx], hPrivateData->prevWhiteningLevel, IGF_MAX_TILES);
    /* restore flattening trigger for current subframe */
    hInstance->flatteningTrigger = hPrivateData->igf_flatteningTrigger_subframe[subFrameIdx];
    move16();

}
