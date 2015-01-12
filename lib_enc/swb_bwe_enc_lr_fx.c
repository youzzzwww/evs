/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"

#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "basop_mpy.h"

/*--------------------------------------------------------------------------*
 * GetSubbandCorrIndex2_har()
 *
 * Finds the index of best correlation between highband (*inBuf) and lowband (*predBuf) for current subband of length (fLen)
 *--------------------------------------------------------------------------*/
static
Word16 GetSubbandCorrIndex2_har_fx(     /* o  : best correlation index                            */
    const Word32 *L_inBuf,              /* i  : target buffer (i.e., highband signal) : spectra   */
    const Word16 fLen,                  /* i  : window length                                     */
    const Word16 *predBuf_fx,           /* i  : prediction buffer (i.e., lowband) : sspectra      */
    const Word16 predBufLen,            /* i  : sspectra buffer size                              */
    const Word16 maxLag_fx,             /* i  : search length                                     */
    const GainItem_fx *G_item_fx,         /* i  :                                                   */
    const Word16 nZero_fx,              /* i  : number of nonzero components used in              */
    Word16 *prev_frame_bstindx_fx /* i  : previous frame best Indices                       */
)
{
    Word16 i, j;
    Word16 bestIdx_fx;
    Word32 L_lagCorr_sq;
    Word32 L_lagEnergy;

    Word32 L_energy;
    Word32 L_corr;
    Word32 L_corr_sq;

    Word16 corr_sq_hi_fx;
    Word32 L_corr_sq_tmp;
    Word32 L_lagCorr_sq_tmp;
    Word16 corr_sq_fx;
    Word16 lagCorr_sq_fx;
    Word32 L_energy_tmp;
    Word32 L_lagEnergy_tmp;
    Word16 energy_fx;
    Word16 lagEnergy_fx;

    Word16 N1_fx, N2_fx;
    Word16 exp_safe_e;

    Word16 exp_corr;
    Word16 exp_energy;

    Word32 L_buf;

    Word16 ibuf_fx[L_FRAME32k];
    Word16 pbuf_fx[L_FRAME32k];
    Word16 *ptr_pbuf;
    Word16 exp_shift;

    Word16 exp_norm;

    Word32 L_tmp;

    exp_safe_e = 4;

    L_tmp = L_deposit_l(0);
    FOR(i=0; i<fLen; i++)
    {
        L_tmp = L_or(L_tmp, L_abs(L_inBuf[i]));
    }
    exp_norm = norm_l(L_tmp);
    exp_shift = sub(exp_norm, exp_safe_e);

    FOR(i=0; i<fLen; i++)
    {
        ibuf_fx[i] = extract_h( L_shl(L_inBuf[i], exp_shift) ); /* Qi+exp_shift-16 */
    }

    FOR(i=0; i<predBufLen; i++)
    {
        pbuf_fx[i] = shr(predBuf_fx[i], exp_safe_e);
        move16(); /* Qss-exp_safe_e */
    }

    bestIdx_fx = 0;
    move16();
    L_lagCorr_sq = L_deposit_l(0);
    L_lagEnergy = 0x7FFFFFFF;
    move32();

    N1_fx = s_max(0x0, sub(*prev_frame_bstindx_fx, shr(maxLag_fx, 1)));
    IF( *prev_frame_bstindx_fx < 0 )
    {
        N2_fx = sub(maxLag_fx, 1);
    }
    ELSE
    {
        N2_fx = s_min(sub(maxLag_fx, 1), add(*prev_frame_bstindx_fx, shr(maxLag_fx,1)));
    }
    predBuf_fx += N1_fx;
    ptr_pbuf = pbuf_fx + N1_fx;


    /* find the best lag */
    FOR( i = N1_fx; i <= N2_fx; i++ )
    {
        L_corr = L_deposit_l(0);

        /* get the energy, remove the old and update with the new energy index */

        L_energy = L_deposit_l(0);
        FOR(j = 0; j < fLen; j++)
        {
            /*energy += *predBuf * *predBuf; */
            L_energy = L_mac(L_energy, *ptr_pbuf, *ptr_pbuf);                        /* Q*2-1; */
            ptr_pbuf++;
        }

        ptr_pbuf -= fLen;

        /* get cross-correlation */
        IF( L_energy != 0x0L )
        {
            L_corr = L_deposit_l(0);
            FOR(j = 0; j < nZero_fx; j++)
            {
                /*corr += inBuf[G_item[j].gainIndex]* predBuf[G_item[j].gainIndex]; */
                L_corr = L_mac(L_corr, ibuf_fx[G_item_fx[j].gainIndex_fx], ptr_pbuf[G_item_fx[j].gainIndex_fx]);                    /* Q*2-1 */
            }

            /*corr_sq = corr*corr; */
            exp_norm = norm_l(L_corr);
            exp_norm = sub(exp_norm, 1); /* added for Overflow  0x8000 * 0x8000  -> Overflow */
            L_corr_sq = L_shl(L_corr, exp_norm);
            corr_sq_hi_fx = extract_h(L_corr_sq);
            L_corr_sq = L_mult(corr_sq_hi_fx, corr_sq_hi_fx); /* (((Qhi:Qsh+exp_norm_hi-16)+Qss+1)+exp_norm-16)*2+1 */
            L_corr_sq = L_shr(L_corr_sq, shl(exp_norm, 1)); /* (QCorr-16)*2+1 */

            IF(Overflow != 0)
            {
                L_corr_sq = 0x0L;
                move16();
                Overflow = 0;
                move16();
            }

            /* normalize for L_lagCorr_sq and L_corr_sq */
            L_buf = L_or(L_lagCorr_sq, L_corr_sq);
            exp_corr = norm_l(L_buf);
            L_corr_sq_tmp = L_shl(L_corr_sq, exp_corr);
            L_lagCorr_sq_tmp = L_shl(L_lagCorr_sq, exp_corr);
            corr_sq_fx = extract_h(L_corr_sq_tmp);
            lagCorr_sq_fx = extract_h(L_lagCorr_sq_tmp);

            /* normalize for L_lagEnergy and L_energy */
            L_buf = L_or(L_lagEnergy, L_energy);
            exp_energy = norm_l(L_buf);
            L_energy_tmp = L_shl(L_energy, exp_energy);
            L_lagEnergy_tmp = L_shl(L_lagEnergy, exp_energy);
            energy_fx = extract_h(L_energy_tmp);
            lagEnergy_fx = extract_h(L_lagEnergy_tmp);

            /*if( (double)lagCorr_sq*(double)energy < (double)corr_sq*(double)lagEnergy ) */
            IF (L_msu(L_mult(lagCorr_sq_fx, energy_fx), corr_sq_fx, lagEnergy_fx) < 0)
            {
                bestIdx_fx    = i;
                move16();
                L_lagCorr_sq  = L_add(L_corr_sq, 0);
                L_lagEnergy   = L_add(L_energy, 0);
            }

        }
        predBuf_fx++;
        ptr_pbuf++;
    }

    test();
    IF( L_lagCorr_sq == 0x0 && *prev_frame_bstindx_fx < 0 )
    {
        bestIdx_fx = 0x0;
        move16();
    }
    ELSE
    {
        if( L_lagCorr_sq == 0x0 )
        {
            bestIdx_fx = *prev_frame_bstindx_fx;
            move16();
        }
    }

    *prev_frame_bstindx_fx = bestIdx_fx;
    move16();

    return bestIdx_fx;
}

/*--------------------------------------------------------------------------*
 * getswbindices_har()
 *
 * Finds the pulse index of best correlation between highband (*yos) and lowband (*y2) for two groups of length sbLen
 *--------------------------------------------------------------------------*/

static void getswbindices_har_fx(
    const  Word32 *L_yos,                       /* i  : original input spectrum   */
    Word16 exp_refBuf,                   /* i  :                           */
    Word16 *y2_fx,                       /* i  : decoded spectrum          */
    const Word16 nBands_search_fx,             /* i  : number of bands           */
    Word16 *lagIndices_fx,               /* o  : pulse index               */
    Word16 *prev_frame_bstindx_fx,       /* i/o: prev frame index          */
    const Word16 swb_lowband_fx,               /* i  : length of the LF spectrum */
    const Word16 *subband_offsets_fx,          /* i  :                           */
    const Word16 *sbWidth_fx,                  /* i  :                           */
    const Word16 *subband_search_offset_fx     /* i  :                           */
)
{
    Word16      i, j, k, sb, tmp;
    Word16      *ptr_predBuf;
    GainItem_fx Nbiggest_fx[(NB_SWB_SUBBANDS_HAR_SEARCH_SB)*N_NBIGGEST_PULSEARCH];
    Word16      n_nbiggestsearch_fx[NB_SWB_SUBBANDS_HAR];

    Word16      search_offset_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB];
    Word16      nlags_fx[NB_SWB_SUBBANDS_HAR_SEARCH_SB];

    Word16      low_freqsgnl_fx[L_FRAME32k]; /* Qy2 (sspectra) */

    ptr_predBuf = y2_fx;

    /* Get the number of HF groups for performing Similarity search */
    FOR( sb = 0; sb < nBands_search_fx; sb++ )
    {
        /*nlags[sb] = (short)pow(2, bits_lagIndices_mode0_Har[sb]); */
        nlags_fx[sb] = shl(1, bits_lagIndices_mode0_Har_fx[sb]);
    }

    j = 0;
    move16();
    FOR( sb=0; sb<nBands_search_fx; sb++ )
    {
        /* Find NBiggest samples in HF Groups */
        FindNBiggest2_simple_fx_har( L_yos + add(swb_lowband_fx, subband_offsets_fx[sb]), exp_refBuf, Nbiggest_fx+j,
                                     sbWidth_fx[sb], &n_nbiggestsearch_fx[sb], N_NBIGGEST_PULSEARCH );

        search_offset_fx[sb] = subband_search_offset_fx[sb];
        j = add(j, N_NBIGGEST_PULSEARCH);
    }

    /* Similarity Search for the HF spectrum */
    FOR( sb=0; sb<nBands_search_fx; sb++ )
    {
        IF( sb == 0 )
        {
            /* copy SSmoothed LF Spectrum */
            ptr_predBuf = y2_fx + sub(search_offset_fx[sb], shr(nlags_fx[sb], 1));
            tmp = add(sbWidth_fx[sb], nlags_fx[sb]);
            FOR(i=0; i<tmp; i++)
            {
                low_freqsgnl_fx[i] = *ptr_predBuf++;
                move16();
            }
        }
        ELSE
        {
            /* copy SSmoothed LF Spectrum */
            k = 0;
            move16();
            tmp = sub(search_offset_fx[sb], add(sbWidth_fx[sb], shr(nlags_fx[sb], 1)));
            FOR( j = add(search_offset_fx[sb], shr(nlags_fx[sb], 1)); j > tmp; j-- )
            {
                low_freqsgnl_fx[k] = y2_fx[j];
                k = add(k, 1);
            }
        }
        /* correlation b/w HF spectrum Group1 of length sbLen and decoded LF spectrum */
        lagIndices_fx[sb] = (Word16)GetSubbandCorrIndex2_har_fx( L_yos + add(swb_lowband_fx, subband_offsets_fx[sb]),
                            sbWidth_fx[sb],
                            low_freqsgnl_fx,
                            add(sbWidth_fx[sb], nlags_fx[sb]),
                            nlags_fx[sb], Nbiggest_fx+(sb*N_NBIGGEST_PULSEARCH),
                            n_nbiggestsearch_fx[sb], &prev_frame_bstindx_fx[sb]
                                                               );
    }

    return;
}

static Word16 GetSubbandCorrIndex2_pulsestep_fx(
    const Word32 *L_inBuf,        /* i: original input vector (highband)                 */
    const Word16 *predBuf_fx,     /* i: spectrum smoothing vector                        */
    const Word16 *predBufMa_fx,   /* i: moving averaged spectrum smoothing vector  Q=Qss */
    const Word16 fLen,            /* i: subband length of highband                       */
    const Word16 maxLag,          /* i: number of search pulse number                    */
    const GainItem_fx *gi_fx,     /* i: lag gain structure                               */
    const Word16 nZero,           /* i:                                                  */
    const Word16 ssearch_buflim,  /* i: length of search buffer                          */
    const Word16 *predBuf_ni_fx   /* i: spectrum including noise smoothing vector        */
)
{
    Word16 i, j;
    Word16 absPos_fx;
    Word16 bestIdx_fx;
    Word32 L_lagCorr_sq;
    Word32 L_lagEnergy;

    const Word16 *ptr_predBuf;
    const Word16 *ptr_predBuf_ni;

    Word32 L_energy;
    Word32 L_corr;
    Word16 corr_fx;

    Word32 L_corr_sq;


    Word16 hiBuf_fx[L_FRAME32k];
    Word16 exp_norm_hi;

    Word16 exp_norm;
    Word32 L_corr_sq_tmp;
    Word32 L_lagCorr_sq_tmp;
    Word16 corr_sq_fx;
    Word16 lagCorr_sq_fx;
    Word32 L_energy_tmp;
    Word32 L_lagEnergy_tmp;
    Word16 energy_fx;
    Word16 lagEnergy_fx;

    Word16 ib_flag_fx;

    Word32 L_buf;
    Word32 L_buf2;
    Word32 L_temp;

    Word16 ssBuf_fx[L_FRAME32k];
    Word16 ssBuf_ni_fx[L_FRAME32k];
    Word16 *ptr_ssBuf_ni_fx;
    Word16 exp_norm_ss;

    set16_fx(hiBuf_fx, 0x0, L_FRAME32k);
    set16_fx(ssBuf_fx, 0x0, L_FRAME32k);

    ib_flag_fx = 0;
    move16();

    absPos_fx = 0;
    move16();
    bestIdx_fx = -1;
    move16();
    L_lagCorr_sq = L_deposit_l(0);
    L_lagEnergy = 0x7FFFFFFFL;
    move32();

    ptr_predBuf = predBuf_fx;
    ptr_predBuf_ni = predBuf_ni_fx;

    /* This part must be computed on parent function. */
    exp_norm_ss = 2;
    move16();

    j = add(ssearch_buflim,fLen);
    FOR (i=0; i<j; i++)
    {
        ssBuf_fx[i] = shr(predBuf_fx[i], exp_norm_ss);
        move16(); /* Qss+exp_norm_ss */
        ssBuf_ni_fx[i] = shr(predBuf_ni_fx[i], exp_norm_ss);
        move16(); /* Qss+exp_norm_ss */
    }

    L_temp = L_deposit_l(0);
    FOR (i = 0 ; i < fLen; i++)
    {
        L_temp = L_or(L_temp, L_abs(L_inBuf[i]));
    }
    exp_norm_hi = norm_l(L_temp);
    exp_norm_hi = sub(exp_norm_hi, 3); /* max 109 < 2^7 , sspectrum is sparse, it is't need 4 */

    FOR (i = 0 ; i < fLen; i++)
    {
        hiBuf_fx[i] = extract_h(L_shl(L_inBuf[i], exp_norm_hi));
        move16(); /* Qsh+exp_norm_hi-16 */
    }

    /* Get the initial energy for zero lag */
    WHILE( *ptr_predBuf == 0 && sub(absPos_fx, ssearch_buflim) < 0 )
    {
        ptr_predBuf++;
        ptr_predBuf_ni++;
        absPos_fx = add(absPos_fx, 1);
    }

    IF( sub(absPos_fx, ssearch_buflim) == 0 )
    {
        ptr_predBuf--;
        ptr_predBuf_ni--;
        absPos_fx = sub(absPos_fx, 1);
    }

    ptr_ssBuf_ni_fx = ssBuf_ni_fx + absPos_fx;
    L_energy = L_deposit_l(0);
    FOR( i = 0; i < fLen; i++)
    {
        L_energy = L_mac(L_energy, *ptr_ssBuf_ni_fx, *ptr_ssBuf_ni_fx); /* (Qss-exp_norm_ss)*2+1 */
        ptr_ssBuf_ni_fx++;
        ptr_predBuf_ni++;
    }

    ptr_ssBuf_ni_fx -= fLen;
    ptr_predBuf_ni -= fLen;

    L_lagEnergy = L_add(L_energy, 0);

    /* Find the best lag */
    FOR( i = 0; i < maxLag; i++ )
    {
        L_corr = L_deposit_l(0);

        /* Get the energy, remove the old and update with the new energy index */
        L_energy = L_deposit_l(0);
        FOR(j = 0; j < fLen; j++)
        {
            L_energy = L_mac(L_energy, *ptr_ssBuf_ni_fx,  *ptr_ssBuf_ni_fx);
            ptr_ssBuf_ni_fx++;
            ptr_predBuf_ni++;
        }
        ptr_ssBuf_ni_fx -= fLen;
        ptr_predBuf_ni -= fLen;

        /* Get cross-correlation */
        IF( L_energy != 0x0L )
        {
            L_corr = L_deposit_l(0);
            FOR(j = 0; j < nZero; j++)
            {
                /*corr += inBuf[G_item[j].gainIndex]* predBufMa[G_item[j].gainIndex]; */
                L_corr = L_mac(L_corr, hiBuf_fx[gi_fx[j].gainIndex_fx], predBufMa_fx[gi_fx[j].gainIndex_fx]); /* Qsh+Qss+1 */
            }

            /*corr_sq = corr*corr; */
            exp_norm = norm_l(L_corr);
            exp_norm = sub(exp_norm, 1); /* added for Overflow  0x8000 * 0x8000  -> Overflow */
            L_corr_sq = L_shl(L_corr, exp_norm);
            corr_fx = extract_h(L_corr_sq);
            L_corr_sq = L_mult(corr_fx, corr_fx); /* (((Qhi:Qsh+exp_norm_hi-16)+Qss+1)+exp_norm-16)*2+1 */
            L_corr_sq = L_shr(L_corr_sq, shl(exp_norm, 1)); /* (QCorr-16)*2+1 */

            /*if( (lagCorr_sq == 0.0f && corr_sq == 0.0f) || (double)lagCorr_sq*(double)energy < (double)corr_sq*(double)lagEnergy ) */
            /*{ */
            /*    bestIdx = i; */
            /*    bestAbsPos = absPos; */
            /*    lagCorr_sq = corr_sq; */
            /*    lagCorr = corr; */
            /*    lagEnergy = energy; */
            /*} */
            /* normalize for L_lagCorr_sq and L_corr_sq */
            L_buf = L_or(L_lagCorr_sq, L_corr_sq);
            exp_norm = norm_l(L_buf); /* overflow allowed */
            L_corr_sq_tmp = L_shl(L_corr_sq, exp_norm);
            L_lagCorr_sq_tmp = L_shl(L_lagCorr_sq, exp_norm);
            corr_sq_fx = extract_h(L_corr_sq_tmp);
            lagCorr_sq_fx = extract_h(L_lagCorr_sq_tmp);

            /* normalize for L_lagEnergy and L_energy */
            L_buf = L_or(L_lagEnergy, L_energy);
            exp_norm = norm_l(L_buf); /* overflow allowed */
            L_energy_tmp = L_shl(L_energy, exp_norm);
            L_lagEnergy_tmp = L_shl(L_lagEnergy, exp_norm);
            energy_fx = extract_h(L_energy_tmp);
            lagEnergy_fx = extract_h(L_lagEnergy_tmp);

            L_buf = L_or(L_lagCorr_sq, L_corr_sq);
            L_buf2 = L_msu(L_mult(lagCorr_sq_fx, energy_fx), corr_sq_fx, lagEnergy_fx);
            test();
            IF( L_buf == 0 ||  L_buf2 < 0 )
            {
                bestIdx_fx = i;
                move16();
                L_lagCorr_sq = L_add(L_corr_sq, 0);
                L_lagEnergy = L_add(L_energy, 0);
            }
        }
        ptr_predBuf++;
        ptr_ssBuf_ni_fx++;
        ptr_predBuf_ni++;
        absPos_fx++;

        WHILE( *ptr_predBuf == 0 && sub(absPos_fx, ssearch_buflim) < 0 )
        {
            ptr_predBuf++;
            ptr_ssBuf_ni_fx++;
            ptr_predBuf_ni++;
            absPos_fx = add(absPos_fx, 1);
        }

        IF( sub(absPos_fx, ssearch_buflim) >= 0 )
        {
            if( sub(bestIdx_fx, -1) == 0 )
            {
                ib_flag_fx = 1;
                move16();
            }

            BREAK;
        }
    }

    IF( sub(ib_flag_fx, 1) == 0 )
    {
        bestIdx_fx = 0;
        move16();
    }

    return bestIdx_fx;
}

static void GetSWBIndices_fx(
    const Word16 *predBuf_fx,            /* i  : low-frequency band                  */
    /*const Word16 Qss,*/                  /* i  : Q value of predBuf_fx               */
    const Word32 *L_targetBuf,           /* i  : SWB MDCT coeff.                     */
    const Word16 Qsh,                    /* i  : Q value of L_targetBuf              */
    const Word16 nBands_search,          /* i  : number of search subbands           */
    const Word16 *sbWidth,               /* i  : subband lengths                     */
    Word16 *lagIndices,            /* o  : selected lags for subband coding    */
    const Word16 predBufLen,             /* i  : low-frequency band length           */
    GainItem_fx *gi_fx,                 /* o  : most representative region          */
    const Word16 *subband_offsets,       /* o  : N biggest components                */
    Word16 *predBuf_ni_fx          /* i  : low-frequency band filled noise     */
)
{
    Word16      j;
    Word16      sb, tmp;
    Word16      sbLen;
    Word16      n_nbiggestsearch_fx[NB_SWB_SUBBANDS];
    Word16      ssearch_buflim_fx;
    Word16      search_offset_fx[NB_SWB_SUBBANDS];
    Word16      nlags_fx[NB_SWB_SUBBANDS];

    Word16      exp_refBuf;
    Word16      sspectra_ma_fx[L_FRAME32k];


    Word32      L_temp;


    /* Initializations */
    exp_refBuf = Qsh;

    j = 0;
    move16();
    FOR (sb = 0; sb < nBands_search; sb++)
    {
        FindNBiggest2_simple_fx_har( L_targetBuf + subband_offsets[sb], exp_refBuf, gi_fx+j, sbWidth[sb], &n_nbiggestsearch_fx[sb], N_NBIGGEST_PULSEARCH );

        j = add(j, N_NBIGGEST_PULSEARCH);
        move16();
    }

    /* Selection of most representative subband (full search) */
    FOR ( sb = 0; sb < nBands_search; sb++ )
    {
        nlags_fx[sb] = shl(1, bits_lagIndices_fx[sb]);
        move16();
    }

    FOR ( sb = 0; sb < nBands_search; sb++ )
    {
        search_offset_fx[sb] = subband_search_offsets_fx[sb];
        move16();
    }

    sspectra_ma_fx[0] = add(shr(predBuf_ni_fx[0],1), shr(predBuf_ni_fx[1],1));
    tmp = sub(predBufLen, 1);
    FOR( sb=1; sb < tmp; sb++ )
    {
        /*sspectra_ma[sb] = (predBuf[sb-1] + predBuf[sb] + predBuf[sb+1])/3.0f; */
        L_temp = L_mult(predBuf_ni_fx[sb], 10922); /* 10922 = 0.33333 (Q15) */
        L_temp = L_add(L_temp, L_mult(predBuf_ni_fx[sb-1], 10922));
        L_temp = L_add(L_temp, L_mult(predBuf_ni_fx[sb+1], 10922));  /* Qss+15+1 */
        sspectra_ma_fx[sb] = round_fx(L_temp);
    }
    sspectra_ma_fx[sb] = add(shr(predBuf_ni_fx[sb-1],1), shr(predBuf_ni_fx[sb],1));

    /* Partial search for rest of subbands except the last which is fixed */
    FOR ( sb=0; sb<nBands_search; sb++)
    {
        sbLen = sbWidth[sb];
        ssearch_buflim_fx = sub(predBufLen, add(sbLen, search_offset_fx[sb]));
        lagIndices[sb] = GetSubbandCorrIndex2_pulsestep_fx(
                             L_targetBuf + subband_offsets[sb],
                             predBuf_fx + search_offset_fx[sb],
                             sspectra_ma_fx + search_offset_fx[sb],
                             sbLen, nlags_fx[sb], gi_fx+(sb*N_NBIGGEST_PULSEARCH),
                             n_nbiggestsearch_fx[sb],
                             ssearch_buflim_fx, predBuf_ni_fx + search_offset_fx[sb]);
    }

}

static void gethar_noisegn_fx(
    Encoder_State_fx *st_fx,
    Word32 L_spectra[],                 /* i   : Qs   input MDCT                          */
    Word16 QsL,                         /* i   : Q0   Q value for L_spectra, L_xSynth_har */
    Word16 noise_flr_fx[],              /* i   : Qss  noise floor                         */
    Word16 Qss,                         /* i   : Q0   Q value for noise_flr_fx, sspectra  */
    Word32 L_xSynth_har[],              /* o   : Qs   output SWB MDCT                     */
    const Word16 sbWidth[],                   /* i   : Q0   band width for SWB                  */
    const Word16 lagIndices[],                /* i   : Q0   lag Indices                         */
    const Word16 bands,                       /* i   : Q0   all band number                     */
    const Word16 har_bands,                   /* i   : Q0   harmonic band number                */
    const Word16 fLenLow,                     /* i   : Q0   low frequency band width            */
    const Word16 fLenHigh,                    /* i   : Q0   SWB band width                      */
    const Word16 subband_offsets[],           /* i   : Q0   offset                              */
    const Word16 subband_search_offset[],     /* i   : Q0   offset                              */
    const Word16 band_start[],                /* i   : Q0   band start array                    */
    const Word16 band_end[],                  /* i   : Q0   band end array                      */
    const Word16 band_width[],                /* i   : Q0   band width                          */
    Word32 L_band_energy[],             /* i   : Qbe  band energy (Log scale)             */
    Word16 Qbe,                         /* i   : Q0   Q value for L_band_energy           */
    Word32 L_be_tonal[],                /* o   : QbeL tonal energy                        */
    Word16 *QbeL,                       /* o   : Q0   Q value for L_be_tonal              */
    Word16 *sspectra_fx,                /* i   : Qss  smoothed spectrum                   */
    Word16 har_freq_est2,               /* i   : Q0   for harmonic structure              */
    Word16 pos_max_hfe2,                /* i   : Q0   for harmonic structure              */
    Word16 *pul_res_fx,                 /* o   : Q0                                       */
    GainItem_fx pk_sf_fx[]                   /* o   :                                          */
)
{
    Word16 i;

    Word32 L_xSynth_har_sft[L_FRAME32k];
    Word32 L_hfspec_sft[L_FRAME32k];

    Word32 L_hfspec[L_FRAME32k];

    GainItem_fx get_pk_fx[N_NBIGGEST_SEARCH_LRG_B];
    Word16 n_nbiggestsearch_fx, imin_fx, gqlevs_fx;

    Word32 L_g1, L_g2;
    Word16 exp_safe;
    Word32 L_temp;

    Word16 exp_normn, exp_normd;
    Word16 temp_fx;
    Word16 div_fx;

    Word16 exp_norm_g1, exp_norm_g2;
    Word16 sqrt_fx, Qsqrt;
    Word16 g_fx;
    Word16 exp, frac;

    Word16 dmin_fx, d_fx;

    Word16 temp_lo, temp_hi;
    Word16 Qg;

    /*Generate HF noise*/
    genhf_noise_fx(noise_flr_fx, Qss, L_xSynth_har, QsL, sspectra_fx ,bands, har_bands, har_freq_est2, pos_max_hfe2,pul_res_fx,pk_sf_fx,fLenLow,
                   fLenHigh, sbWidth, lagIndices, subband_offsets, subband_search_offset);

    Copy32(&L_spectra[fLenLow], L_hfspec, fLenHigh);
    FindNBiggest2_simple_fx_har(L_hfspec, QsL, get_pk_fx, fLenHigh, &n_nbiggestsearch_fx, N_NBIGGEST_SEARCH_LRG_B);
    FOR(i=0; i<n_nbiggestsearch_fx; i++)
    {
        L_hfspec[get_pk_fx[i].gainIndex_fx] = 0x0;
        move16();
    }

    L_temp = 0x0;
    FOR(i=0; i<fLenHigh; i++)
    {
        L_temp = L_or(L_temp, L_abs(L_hfspec[i]));
    }
    exp_norm_g1 = norm_l(L_temp);
    FOR(i=0; i<fLenHigh; i++)
    {
        L_hfspec_sft[i] = L_shl(L_hfspec[i], exp_norm_g1);
    }

    L_temp = 0x0;
    FOR(i=0; i<fLenHigh; i++)
    {
        L_temp = L_or(L_temp, L_abs(L_xSynth_har[i]));
    }
    exp_norm_g2 = norm_l(L_temp);
    FOR(i=0; i<fLenHigh; i++)
    {
        L_xSynth_har_sft[i] = L_shl(L_xSynth_har[i], exp_norm_g2);
    }

    exp_safe = 4;
    move16();
    L_g1 = L_deposit_l(0);
    L_g2 = L_deposit_l(0);
    FOR(i=0; i<fLenHigh; i++)
    {
        temp_fx = round_fx(L_shr(L_hfspec_sft[i], exp_safe));
        L_g1 = L_mac(L_g1, temp_fx, temp_fx); /* 4: safe shift */
        temp_fx = round_fx(L_shr(L_xSynth_har_sft[i], exp_safe));
        L_g2 = L_mac(L_g2, temp_fx, temp_fx); /* 4: safe shift */
    }

    /*g = (float) log10(sqrt(g1/g2));*/
    /* Div Part */
    exp_normn = norm_l(L_g1);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_l(L_g2);
    temp_fx = extract_h(L_shl(L_g2, exp_normd));
    exp_normd = sub(exp_normd, 16);

    div_fx = div_l(L_shl(L_g1, exp_normn), temp_fx);
    /* SQRT Part */
    sqrt_32n_16_fx(L_deposit_h(div_fx), add(add(sub(exp_normn, exp_normd), shl(sub(exp_norm_g1, exp_norm_g2), 1)), 15), &sqrt_fx, &Qsqrt); /* (exp_normn-exp_normd+(exp_norm_g1-exp_norm_g2)*2 -1)+16 */

    /* Log10 Part */
    g_fx = 0x0;
    move16();
    IF ( sqrt_fx > 0x0 )
    {
        L_temp = L_deposit_l(sqrt_fx);

        exp = norm_l(L_temp);
        frac = Log2_norm_lc(L_shl(L_temp, exp));
        exp = sub(30, exp);
        exp = sub(exp, Qsqrt);
        L_temp = L_Comp(exp, frac);

        L_temp = Mpy_32_16_1(L_temp, 19728);   /* log(2)/log(10)=.30102999566398119521 = 19728.3(Q16)    Q(0+16+1)=Q17 */
        L_temp = L_shl(L_temp, 13);      /* Q17+13=30    30-16=14 */
        g_fx = round_fx(L_temp);
    }

    gqlevs_fx = 4;
    move16();
    dmin_fx = 32767;
    move16();
    imin_fx = 0;
    move16();

    FOR (i = 0; i < gqlevs_fx; i++)
    {
        d_fx= abs_s (g_fx - gain_table_fx[i]);
        IF (sub(d_fx, dmin_fx) < 0)
        {
            dmin_fx = d_fx;
            move16();
            imin_fx = i;
            move16();
        }
    }

    push_indice_fx( st_fx,IND_NOISEG, imin_fx, 2);

    /*g=(float) pow (10.0f,gain_table[imin]);*/
    L_temp = L_mult(gain_table_fx[imin_fx], 27213); /* Q14+Q13+1=Q28  log(10)/log(2)=3.3219 27213.23(Q13) */
    L_temp = L_shr(L_temp, 12);   /* Q28-Q12 -> Q16 */
    temp_lo = L_Extract_lc(L_temp, &temp_hi);
    Qg = sub(14, temp_hi);
    g_fx = extract_l(Pow2(14, temp_lo));
    g_fx = shl(g_fx, sub(11, Qg));

    ton_ene_est_fx(
        L_xSynth_har, QsL, L_be_tonal, QbeL, L_band_energy, Qbe,
        band_start, band_end, band_width, fLenLow, fLenHigh, bands, har_bands, g_fx,
        pk_sf_fx, Qss, pul_res_fx
    );

    return;
}

static void EncodeSWBSubbands_fx(
    Encoder_State_fx *st_fx,                   /* i/o: encoder state structure                     */
    Word32 *L_spectra,               /* i/o: MDCT domain spectrum                        */
    Word16 QsL,                      /* i  : Q value for L_spectra                       */
    const Word16 fLenLow_fx,               /* i  : lowband length                              */
    const Word16 fLenHigh_fx,              /* i  : highband length                             */
    const Word16 nBands_fx,                /* i  : number of subbands                          */
    const Word16 nBands_search_fx,         /* i  : number of subbands to be searched for BWE   */
    const Word16 *sbWidth_fx,              /* i  : subband lengths                             */
    const Word16 *subband_offsets_fx,      /* i  : Subband offset for BWE                      */
    Word16 *lagIndices_fx,           /* o  : lowband index for each subband              */
    const Word16 BANDS_fx,                 /* i  : noise estimate from WB part                 */
    const Word16 *band_start_fx,           /* i  : Number subbands/Frame                       */
    const Word16 *band_end_fx,             /* i  : Band Start of each SB                       */
    Word32 *L_band_energy,           /* i  : Band end of each SB, :Qbe                   */
    Word16 Qbe,                      /* i  : Q value of band energy                      */
    const Word16 *p2a_flags_fx,            /* i  : BAnd energy of each SB                      */
    const Word16 hqswb_clas_fx,            /* i  : lowband synthesis                           */
    Word16 *prev_frm_index_fx,       /* i  : clas information                            */
    const Word16 har_bands_fx,             /* i/o: Index of the previous Frame                 */
    const Word16 *subband_search_offset_fx,/* i  : Number of harmonic LF bands                 */
    Word16 *prev_frm_hfe2_fx,        /* i/o:                                             */
    Word16 *prev_stab_hfe2_fx,       /* i/o:                                             */
    const Word16 band_width_fx[],          /* i  : band width                                  */
    const Word32 L_spectra_ni[],           /* i  : Qs noise injected spectra                   */
    Word16 *ni_seed_fx               /* i/o: random seed                                 */
)
{
    Word16 i, k;
    Word16 sspectra_fx[L_FRAME32k];
    Word16 sspectra_ni_fx[L_FRAME32k];
    Word16 sspectra_diff_fx[L_FRAME32k];
    Word16 Qss;  /* Q value of Smoothed Spectrum low-subband */
    Word32 L_be_tonal[SWB_HAR_RAN1]; /* Q */
    Word16 ss_min_fx; /* Qss */
    Word32 L_th_g[NB_SWB_SUBBANDS];
    Word16 QbeL;
    GainItem_fx pk_sf_fx[(NB_SWB_SUBBANDS)*8];
    Word16 pul_res_fx[NB_SWB_SUBBANDS];

    GainItem_fx Nbiggest_fx[NB_SWB_SUBBANDS*N_NBIGGEST_PULSEARCH];

    Word32 L_xSynth_har[L_FRAME32k]; /* Qs */

    Word16 lagGains_fx[NB_SWB_SUBBANDS];
    Word16 QlagGains[NB_SWB_SUBBANDS];
    Word16 har_freq_est1,har_freq_est2;
    Word16 flag_dis;
    Word16 pos_max_hfe2;

    har_freq_est1 = 0;
    move16();
    har_freq_est2 = 0;
    move16();
    flag_dis = 1;
    move16();
    pos_max_hfe2 = 0;
    move16();

    set16_fx(sspectra_fx, 0, fLenLow_fx);
    set16_fx(sspectra_ni_fx, 0, fLenLow_fx);
    set32_fx(L_xSynth_har, 0, L_FRAME32k);
    set16_fx(pul_res_fx,0,NB_SWB_SUBBANDS);


    IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        pos_max_hfe2 = har_est_fx( L_spectra, fLenLow_fx, &har_freq_est1, &har_freq_est2, &flag_dis, prev_frm_hfe2_fx, subband_search_offset_fx, sbWidth_fx, prev_stab_hfe2_fx );
        noise_extr_corcod_fx(L_spectra, L_spectra_ni, sspectra_fx, sspectra_diff_fx, sspectra_ni_fx, fLenLow_fx , st_fx->prev_hqswb_clas_fx ,&st_fx->prev_ni_ratio_fx, &Qss);
        /* Find best indices for each group */
        getswbindices_har_fx(
            L_spectra,
            QsL, sspectra_ni_fx,
            nBands_search_fx, lagIndices_fx, prev_frm_index_fx, fLenLow_fx, subband_offsets_fx, sbWidth_fx , subband_search_offset_fx);

        /* Write the indices into the bitstream */
        FOR (k = 0; k < nBands_search_fx; k++)
        {
            push_indice_fx( st_fx, IND_LAGINDICES, lagIndices_fx[k], bits_lagIndices_mode0_Har_fx[k]);
        }

        IF ( flag_dis == 0 )
        {
            test();
            if ( sub(har_freq_est2, SWB_HAR_RAN1) != 0 || sub(har_freq_est2, *prev_frm_hfe2_fx) != 0 )
            {
                har_freq_est2 = add(har_freq_est2, lagIndices_fx[0]);
                move16();
            }
        }

        gethar_noisegn_fx(
            st_fx,
            L_spectra, QsL, sspectra_diff_fx, Qss, L_xSynth_har,
            sbWidth_fx, lagIndices_fx, BANDS_fx, har_bands_fx, fLenLow_fx, fLenHigh_fx,
            subband_offsets_fx, subband_search_offset_fx, band_start_fx, band_end_fx, band_width_fx,
            L_band_energy, Qbe, L_be_tonal, &QbeL, sspectra_fx,
            har_freq_est2, pos_max_hfe2, pul_res_fx, pk_sf_fx);


        Gettonl_scalfact_fx(
            L_xSynth_har, QsL, L_spectra_ni, fLenLow_fx, fLenHigh_fx, har_bands_fx, BANDS_fx, L_band_energy, Qbe, band_start_fx, band_end_fx,
            p2a_flags_fx, L_be_tonal, QbeL, pk_sf_fx ,Qss, pul_res_fx);

        IF( flag_dis == 0 )
        {
            *prev_frm_hfe2_fx = 0;
            move16();
        }
        ELSE
        {
            *prev_frm_hfe2_fx = har_freq_est2;
            move16();
        }

        FOR( k = BANDS_fx - NB_SWB_SUBBANDS; k < BANDS_fx; k++ )
        {
            FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
            {
                L_spectra[i] = L_xSynth_har[i-fLenLow_fx];
                move32(); /* QsL */
            }
        }
    }
    ELSE
    {
        ss_min_fx = spectrumsmooth_noiseton_fx(
            L_spectra, /*QsL,*/ L_spectra_ni, sspectra_fx, sspectra_diff_fx, sspectra_ni_fx, &Qss, fLenLow_fx, ni_seed_fx);

        /* Get lag indices */
        GetSWBIndices_fx( sspectra_fx, /*Qss,*/
        L_spectra + fLenLow_fx, QsL,
        nBands_search_fx, sbWidth_fx, lagIndices_fx, fLenLow_fx,
        Nbiggest_fx, subband_offsets_fx, sspectra_fx
                        );

        /* Bitstream operations */
        FOR(k=0; k<nBands_fx; k++)
        {
            IF ( sub(p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+k], 1) == 0 )
            {
                lagIndices_fx[k] = 0;
                move16();
                lagGains_fx[k] = 0;
                move16();
                QlagGains[k] = 15;
                move16();
            }
            ELSE
            {
                push_indice_fx( st_fx, IND_LAGINDICES, lagIndices_fx[k], bits_lagIndices_fx[k]);
            }
        }

        convert_lagIndices_pls2smp_fx( lagIndices_fx, nBands_search_fx, lagIndices_fx,
                                       sspectra_fx, sbWidth_fx, fLenLow_fx );

        GetlagGains_fx( sspectra_ni_fx, Qss, &L_band_energy[BANDS_fx-NB_SWB_SUBBANDS], Qbe, nBands_fx, sbWidth_fx, lagIndices_fx, fLenLow_fx, lagGains_fx, QlagGains );
        FOR(k=0; k<NB_SWB_SUBBANDS; k++)
        {
            lagGains_fx[k] = mult_r(lagGains_fx[k], 29491);  /* lagGains[k]*0.9f; */
        }

        FOR(k=0; k<NB_SWB_SUBBANDS; k++)
        {
            L_th_g[k] = L_deposit_l(0);
            IF(p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+k] == 0)
            {
                L_th_g[k] = L_shl(L_mult(lagGains_fx[k], ss_min_fx), sub(QsL, add(add(QlagGains[k], Qss), 1))); /* QlagGain+Qss -> QsL */
            }
        }

        GetSynthesizedSpecThinOut_fx(sspectra_ni_fx, Qss, L_xSynth_har, QsL, nBands_fx, sbWidth_fx, lagIndices_fx, lagGains_fx, QlagGains, fLenLow_fx);

        /*Level adjustment for the missing bands*/
        noiseinj_hf_fx(
            L_xSynth_har, QsL, L_th_g, L_band_energy, Qbe,
            st_fx->prev_En_sb_fx,
            p2a_flags_fx, BANDS_fx, band_start_fx, band_end_fx, fLenLow_fx, fLenHigh_fx);


        FOR( k = BANDS_fx - NB_SWB_SUBBANDS; k < BANDS_fx; k++ )
        {
            IF( p2a_flags_fx[k] == 0 )
            {
                FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
                {
                    L_spectra[i] = L_xSynth_har[i-fLenLow_fx];
                    move32(); /* Qob */
                }
            }
            ELSE
            {
                FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
                {
                    L_spectra[i] = L_spectra_ni[i];
                    move32();
                }
            }
        }
    }

    return;
}

void swb_bwe_enc_lr_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure                      */
    const Word32 L_m_core[],           /* i  : lowband synthesis                            */
    Word16 QsL,                  /* i  : Q value                                      */
    const Word32 L_m_orig[],           /* i/o: scaled orig signal (MDCT)                    */
    Word32 L_m[],                /* o  : highband synthesis with lowband zeroed       */
    const Word32 L_total_brate,        /* i  : total bitrate for selecting subband pattern  */
    Word16 BANDS_fx,             /* i  : Total number of Subbands in a frame          */
    Word16 *band_start_fx,       /* i  : band start of each SB                        */
    Word16 *band_end_fx,         /* i  : band end of each SB                          */
    Word32 *L_band_energy,       /* i  : band_energy of each SB                       */
    Word16 Qbe,                  /* i  : Q value of band energy                       */
    Word16 *p2a_flags_fx,        /* i  : HF tonal indicator                           */
    const Word16 hqswb_clas_fx,        /* i  : HQ_NORMAL2 or HQ_HARMONIC mode               */
    Word16 lowlength_fx,         /* i  : lowband length                               */
    Word16 highlength_fx,        /* i  : highband length                              */
    Word16 *prev_frm_index_fx,   /* i/o: previous frame lag index for harmonic mode   */
    const Word16 har_bands_fx,         /* i  : Number of LF harmonic bands                  */
    Word16 *prev_frm_hfe2,       /* i/o:                                              */
    Word16 *prev_stab_hfe2,      /* i/o:                                              */
    const Word16 band_width_fx[],      /* i  : band_width information                       */
    const Word32 L_y2_ni[],            /* i  : band_width information                       */
    Word16 *ni_seed_fx           /* i/o: random seed for search buffer NI             */
)
{
    Word16 k;
    Word16 nBands_fx;
    Word16 nBands_search_fx;
    Word16 wBands_fx[NB_SWB_SUBBANDS];
    Word16 lagIndices_fx[NB_SWB_SUBBANDS];
    Word16 swb_lowband_fx, swb_highband_fx, allband_fx;

    const Word16 *subband_offsets_fx;
    const Word16 *subband_search_offset_fx;

    Word32 *p_L_m;

    subband_search_offset_fx = subband_search_offsets_13p2kbps_Har_fx;
    subband_offsets_fx = subband_offsets_sub5_13p2kbps_Har_fx;

    hf_parinitiz_fx(L_total_brate,hqswb_clas_fx,lowlength_fx,highlength_fx,wBands_fx,&subband_search_offset_fx,&subband_offsets_fx,&nBands_fx,&nBands_search_fx,&swb_lowband_fx,&swb_highband_fx);
    allband_fx = add(swb_lowband_fx, swb_highband_fx);
    move16();

    /* Prepare m[], low part from WB core, high part from 32k input */
    Copy32( L_m_core,                  L_m,                 swb_lowband_fx  );
    Copy32(&L_m_orig[swb_lowband_fx], &L_m[swb_lowband_fx], swb_highband_fx );

    EncodeSWBSubbands_fx(
        st_fx,
        L_m, QsL,
        swb_lowband_fx, swb_highband_fx, nBands_fx, nBands_search_fx, wBands_fx, subband_offsets_fx,
        lagIndices_fx,
        BANDS_fx, band_start_fx, band_end_fx,
        L_band_energy, Qbe,
        p2a_flags_fx,
        hqswb_clas_fx, prev_frm_index_fx, har_bands_fx, subband_search_offset_fx,
        prev_frm_hfe2, prev_stab_hfe2,
        band_width_fx, L_y2_ni, ni_seed_fx
    );

    p_L_m = &L_m[sub(allband_fx, 1)];
    *p_L_m = Mult_32_16(*p_L_m,  2028);
    move32();
    p_L_m--; /* 0.0625 =  2028 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m,  4096);
    move32();
    p_L_m--; /* 0.125  =  4096 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m,  8192);
    move32();
    p_L_m--; /* 0.25   =  8192 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m, 16384);
    move32();
    p_L_m--; /* 0.5    = 16384 (Q15) */

    /* set low frequencies to zero */
    FOR ( k = 0; k < swb_lowband_fx; k++ )
    {
        L_m[k] = L_deposit_l(0);
    }

    return;
}
