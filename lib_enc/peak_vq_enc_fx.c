/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"
#include "rom_enc_fx.h"
#include "rom_com_fx.h"
#include "stl.h"            /* required for wmc_tool */

/*--------------------------------------------------------------------------
 * Local functions
 *--------------------------------------------------------------------------*/

static void quant_peaks_fx(Encoder_State_fx *, const Word32*, Word32*, const Word32*,
                           Word16*, const Word16, const Word32, const Word16);
static Word16 hvq_code_pos_fx(Encoder_State_fx *st_fx, const Word16 *inp, const Word16 length, const Word16 num_peaks);
static Word16 sparse_code_pos_fx(const Word16 *inp, const Word16 length, Word16 *result);

/*--------------------------------------------------------------------------
 * peak_vq_enc_fx()
 *
 * Vector Quantization of MDCT peaks
 *--------------------------------------------------------------------------*/

Word16 peak_vq_enc_fx(
    Encoder_State_fx *st_fx,         /* i/o: encoder state structure         */
    const Word32 *coefs,            /* i  : Input coefficient vector Q12    */
    Word32 *coefs_out,        /* o  : Quantized output vector Q12     */
    const Word32 brate,             /* i  : Core bitrate                    */
    const Word16 num_bits,          /* i  : Number of bits for HVQ          */
    const Word16 vq_peaks,          /* i  : Number of identified peaks      */
    const Word16 *ynrm,             /* i  : Envelope coefficients           */
    Word16 *R,                /* i/o: Bit allocation/updated bit allocation */
    Word16 *vq_peak_idx,      /* i  : Peak index vector               */
    Word32 *nf_gains          /* i  : Estimated noise floor gains Q12 */
)
{
    Word16 pos_bits;
    Word32 normq;
    Word32 pgain_q[HVQ_MAX_PEAKS_32k];
    Word32 peak_gains[HVQ_MAX_PEAKS_32k];
    Word16 coefs_pvq[HVQ_PVQ_COEFS*MAX_PVQ_BANDS], *pCoefsPvq; /* Q12 */
    Word32 pvq_vector[HVQ_PVQ_COEFS*MAX_PVQ_BANDS], *pPvqVector;
    Word32 *pPvqVectorBandStart;
    Word16 pvq_vector_norm[HVQ_PVQ_COEFS*MAX_PVQ_BANDS];
    Word16 fg_pred[NB_SFM_MAX];
    Word16 i, j, k, m, r, pvq_bands, num_overlap_bins;
    Word16 hcode_l, FlagN, low_peak_bin, vq_cb_idx, max_peaks, bin_th, bin_th2;
    Word16 bits;
    Word16 nf_seed;
    Word16 pgain_cb_idx[HVQ_MAX_PEAKS], pgain_difidx[HVQ_MAX_PEAKS]; /* Q0 */
    Word16 pvq_norm[MAX_PVQ_BANDS];
    Word16 pvq_bits, bit_budget;
    Word16 pos_vec[HVQ_THRES_BIN_32k];
    Word16 npulses[MAX_PVQ_BANDS];
    Word16 pvq_inp_vector[HVQ_PVQ_COEFS*MAX_PVQ_BANDS];
    Word16 k_sort[MAX_PVQ_BANDS];
    Word16 Rk[MAX_PVQ_BANDS];
    Word16 gopt[NB_SFM];
    Word16 tmp1, exp1;
    Word16 Q_coefs;


    Word16 indx, vqPeaksMinus1, tmp16, whiteNoise;
    Word16 *pPgainDifIdx, *pPgainCbIdx, *pVqPeakIdx, *pPosVec;
    Word32 *pPeakGains, *pCoefsOut;
    const Word32 *pCoefs;
    Word32 acc;
    UWord16 dontCare16;
    Word32 manE_peak, manPeakGains, manPkEnrg;  /* Due to very wide dynamic range, use floating point format, i.e., (man, exp) */
    Word16 expE_peak, expPeakGains, expPkEnrg;
    Word16 *pSelBnds;
    Word16 sel_bnds[HVQ_NUM_SFM_24k];
    Word16 hvq_band_end[MAX_PVQ_BANDS];
    Word16 hvq_band_start[MAX_PVQ_BANDS];
    Word16 hvq_band_width[MAX_PVQ_BANDS];
    Word16 n_sel_bnds;
    UWord32 lsb;
    bits = 0;
    move16();
    nf_seed = RANDOM_INITSEED;
    move16();


    set16_fx(coefs_pvq, 0, HVQ_PVQ_COEFS*MAX_PVQ_BANDS );
    set32_fx(pvq_vector, 0, HVQ_PVQ_COEFS*MAX_PVQ_BANDS);
    set16_fx(pvq_vector_norm, 0, HVQ_PVQ_COEFS*MAX_PVQ_BANDS );
    set16_fx(npulses, 0, MAX_PVQ_BANDS );

    /* Set bit-rate dependent variables */
    IF (L_sub(brate, HQ_24k40) == 0)
    {
        max_peaks = HVQ_MAX_PEAKS_24k;
        move16();
        bin_th = HVQ_THRES_BIN_24k;
        move16();
        bin_th2 = HVQ_THRES_BIN_24k/HVQ_NF_GROUPS;
        move16();
    }
    ELSE
    {
        max_peaks = HVQ_MAX_PEAKS_32k;
        move16();
        bin_th = HVQ_THRES_BIN_32k;
        move16();
        bin_th2 = HVQ_THRES_BIN_32k/HVQ_NF_GROUPS;
        move16();
    }

    FOR (i = 0; i < bin_th; i++)
    {
        pos_vec[i] = 0;
        move16();
    }

    /* Quantize noise floor gains */
    FOR (i = 0; i < HVQ_NF_GROUPS; i++)
    {
        logqnorm_fx(&nf_gains[i], 12, &indx, 32, 1, 0);

        /* De-quantization */
        acc = L_add(dicn_fx[indx], 0);         /* Q14 */
        nf_gains[i] = L_shr(acc, 1 + 2);  /* nf_gains in Q12. dicn_fx is in Q14. Need extra shift +2. */
        push_indice_fx(st_fx, IND_HVQ_NF_GAIN , (int)indx, 5);
        bits = add(bits, 5);
    }

    /* Signal number of peaks */
    i = sub(max_peaks, vq_peaks);
    push_indice_fx(st_fx, IND_NUM_PEAKS, (int)i, 5);
    bits = add(bits, 5);

    /* Identify position of first peak and arrange peak gains by position */
    pVqPeakIdx = &vq_peak_idx[0];
    low_peak_bin = bin_th;
    FOR (i = 0; i < vq_peaks; i++)
    {
        indx = *pVqPeakIdx++;
        move16();
        IF (sub(indx, low_peak_bin) < 0)
        {
            low_peak_bin = indx;
            move16();
        }
        /* Store the sign information. */
        IF ( coefs[indx] < 0)
        {
            pos_vec[indx] = -1;    /* Negative. */                              move16();
        }
        ELSE
        {
            pos_vec[indx] = 1;     /* Positive */                               move16();
        }
    }

    pPeakGains = &peak_gains[0];
    pVqPeakIdx = &vq_peak_idx[0];
    pPosVec = &pos_vec[0];
    pCoefs = &coefs[0];
    FOR (i = 0; i < bin_th; i++)
    {
        acc = *pCoefs++;
        IF(*pPosVec++ != 0)
        {
            *pPeakGains++ = L_abs(acc);  /* in Q12 */                           move32();
            *pVqPeakIdx++ = i;
            move16();
        }
    }

    /* Scale down peak gains */
    /* Divided by 4 is equivalent to consider peak_gains to be in Q14 from Q12.
     * No physical bit shift is actually required.
     */

    /* Quantize peak gains */
    pPeakGains = &peak_gains[0];
    pPgainCbIdx = &pgain_cb_idx[0];
    logqnorm_fx(pPeakGains++, 14, pPgainCbIdx++, 32, 1, 1);
    vqPeaksMinus1 = sub(vq_peaks, 1);
    FOR (i = 0; i < vqPeaksMinus1; i++)
    {
        logqnorm_fx(pPeakGains++, 14, pPgainCbIdx++, 45, 1, 1);
    }

    /* Code quantized peak gain indices
     * and also scale up peak gains. */
    diffcod_fx(vq_peaks, pgain_cb_idx, &pgain_difidx[1]);
    /* Accumulate peak energy. */
    manE_peak = L_deposit_l(0);
    expE_peak = 32;
    move16();
    FOR (i = 0; i < vq_peaks; i++)
    {
        indx = pgain_cb_idx[i];
        move16();
        /* Scale up peak gains */
        pgain_q[i] = L_shl(dicn_pg_fx[indx], 2);  /* pgain_q in Q12 */          move32();
        /* Use floating point operation to deal with wide dynamic range.l
         * 32-bit mantissa is used here. It should be even more accurate than
         * the floating-point reference code with 24-bit mantissa! */
        expPeakGains = norm_l(pgain_q[i]);
        manPeakGains = L_shl(pgain_q[i], expPeakGains);
        Mpy_32_32_ss(manPeakGains, manPeakGains, &manPkEnrg, &lsb);  /* peak_gains square */
        expPkEnrg = shl(expPeakGains, 1); /* Multiply by 2 due to squaring. */
        /* True floating value = manPkEng x 2^(32 - 1 - expPkEnrg - 2*12).
         * In this context, the 32-bit manPkEng is in Q0.
         * 32 is due to Mpy_32_32() only providing the 32 MSBs of the 64 bits product.
         * -1 is due fractional mode Multiply. 2*12 is due to square of Q12. */
        floating_point_add(&manE_peak, &expE_peak, manPkEnrg, expPkEnrg);
    }
    pgain_difidx[0] = pgain_cb_idx[0];
    move16();
    Mpy_32_16_ss(manE_peak, 9830, &manE_peak, &dontCare16); /* Multiply by 0.3 */
    Mpy_32_16_ss(st_fx->manE_peak_mem, 22938, &st_fx->manE_peak_mem, &dontCare16); /* Multiply by 0.7 */
    floating_point_add(&manE_peak, &expE_peak, st_fx->manE_peak_mem, st_fx->expE_peak_mem);
    st_fx->manE_peak_mem = manE_peak;
    move32();
    st_fx->expE_peak_mem = expE_peak;
    move16();

    /* Huffman coding */
    hcode_l = 0;
    pPgainDifIdx = &pgain_difidx[1];
    FOR (i = 0; i < vqPeaksMinus1; i++)
    {
        indx = *pPgainDifIdx++;
        move16();
        hcode_l = add(hcode_l, pgain_huffsizn[indx]);
    }

    FlagN = HUFCODE;
    move16();

    tmp16 = extract_l(L_mult0(GAINI_BITS, vqPeaksMinus1));
    IF ( sub(hcode_l, tmp16) >= 0)
    {
        hcode_l = tmp16;
        move16();
        FlagN = NOHUFCODE;
        move16();
    }

    push_indice_fx(st_fx, IND_FLAGN, (int)FlagN, 1);
    push_indice_fx(st_fx, IND_PG_IDX, (int)pgain_difidx[0], GAIN0_BITS);

    IF (FlagN)
    {
        pPgainDifIdx = &pgain_difidx[1];
        FOR (i = 0; i < vqPeaksMinus1; i++)
        {
            j = *pPgainDifIdx++;
            move16();
            m = pgain_huffnorm_fx[j];
            move16();
            r = pgain_huffsizn[j];
            move16();

            push_indice_fx(st_fx, IND_PG_IDX, (int)m, r );
        }
    }
    ELSE
    {
        pPgainDifIdx = &pgain_difidx[1];
        FOR (i = 0; i < vqPeaksMinus1; i++)
        {
            push_indice_fx(st_fx, IND_PG_IDX, (int)(*pPgainDifIdx++), GAINI_BITS );
        }
    }

    /* Number of bits used for peak gain quantization */
    bits = add(bits, add(FLAGN_BITS + GAIN0_BITS, hcode_l));

    /* Add sign for peak shape normalization */
    FOR (i = 0; i < vq_peaks; i++)
    {
        indx = vq_peak_idx[i];
        move16();
        peak_gains[i] = pgain_q[i];   /* Q12 */                                 move32();
        IF (pos_vec[indx] < 0)
        {
            peak_gains[i] = L_negate(peak_gains[i]); /* Q12 */
        }
    }

    /* Quantize peak shapes */
    FOR (i = 0; i < vqPeaksMinus1; i++)
    {
        num_overlap_bins = sub(5, sub(vq_peak_idx[i+1], vq_peak_idx[i]));
        indx = sub(vq_peak_idx[i], 2);
        quant_peaks_fx(st_fx, &coefs[indx], &coefs_out[indx], &peak_gains[i], &vq_cb_idx, num_overlap_bins, brate, vq_peaks);
        push_indice_fx(st_fx, IND_HVQ_PEAKS, (UWord16)vq_cb_idx, 8 );
        bits = add(bits, 9);
    }

    indx = sub(vq_peak_idx[i], 2);
    quant_peaks_fx(st_fx, &coefs[indx], &coefs_out[indx], &peak_gains[i], &vq_cb_idx, 0, brate, vq_peaks);
    push_indice_fx(st_fx, IND_HVQ_PEAKS, (UWord16)vq_cb_idx, 8 );
    bits = add(bits, 9);

    /* Quantize peak positions and sign with HVQ */
    pos_bits = hvq_code_pos_fx( st_fx, pos_vec, bin_th, vq_peaks );

    bits = add(bits, pos_bits);
    bit_budget = sub(num_bits, bits);

    /* Calculate number of PVQ bands to code and assign bits */
    pvq_bands = hvq_pvq_bitalloc_fx(bit_budget, brate, st_fx->bwidth_fx, ynrm, manE_peak, expE_peak, Rk, R, sel_bnds,
                                    &n_sel_bnds);

    /* Get band limits for concatenated PVQ target */
    hvq_concat_bands_fx(pvq_bands, sel_bnds, n_sel_bnds, hvq_band_start,
                        hvq_band_width, hvq_band_end);

    /* Quantize PVQ bands */
    pCoefsOut = coefs_out;
    pCoefs = coefs;
    pPvqVector = pvq_vector;
    pSelBnds = sel_bnds;
    m = bin_th;
    move16();
    FOR (k = 0; k < pvq_bands; k++)
    {
        IF (sub(k, sub(pvq_bands, n_sel_bnds)) >= 0)
        {
            i = band_start_harm[*pSelBnds++];
            move16();
            pCoefs = coefs + i;
            pCoefsOut = coefs_out + i;
        }
        k_sort[k] = k;
        j = 0;
        move16();
        pPvqVectorBandStart = pPvqVector;
        WHILE (sub(j, hvq_band_width[k]) < 0)
        {
            IF (*pCoefsOut++ == 0)
            {
                *pPvqVector++ = *pCoefs; /* Q12 */                              move32();
                j = add(j, 1);
            }
            pCoefs++;
        }
        logqnorm_fx( pPvqVectorBandStart, 12, &pvq_norm[k], 40, hvq_band_width[k], 0 );

    }

    /* Normalize coefficients */
    normalizecoefs_fx( pvq_vector, pvq_norm, pvq_bands, hvq_band_start, hvq_band_end, pvq_vector_norm );
    Q_coefs = 12;
    move16();

    bit_budget = sub(bit_budget, i_mult2(HVQ_PVQ_GAIN_BITS, pvq_bands));

    pvq_bits = bit_budget;
    move16();
    set16_fx( npulses, 0, MAX_PVQ_BANDS );

    pvq_encode_frame_fx(st_fx, pvq_vector_norm, Q_coefs, coefs_pvq, gopt, npulses, pvq_inp_vector, hvq_band_start, hvq_band_end, hvq_band_width, pvq_bands,
                        Rk, pvq_bits, HQ_CORE );

    FOR (i = 0; i < pvq_bands; i++)
    {
        k_sort[i] = i;
        move16();
    }


    fine_gain_pred_fx( hvq_band_start, hvq_band_end, hvq_band_width, k_sort, npulses, NULL, NULL, pvq_bands, coefs_pvq,
                       pvq_inp_vector, fg_pred, HQ_CORE );

    pCoefsOut = &coefs_out[0];
    pSelBnds = &sel_bnds[0];
    pCoefsPvq = &coefs_pvq[0];
    FOR (k = 0; k < pvq_bands; k++)
    {
        indx = pvq_norm[k];
        tmp1 = ratio(gopt[k], fg_pred[k], &exp1);
        tmp1 = shr(tmp1, sub(1, exp1));  /* Q13 */
        Mpy_32_16_ss(dicn_fx[indx], tmp1, &normq, &dontCare16); /* dicn_fx in Q14, sorted_pvq_gain_pred_err_fx in Q13. */

        logqnorm_fx(&normq, 12, &pvq_norm[k], 40, 1, 0); /* normq in Q(14+(16+13)+1-32)=Q12 */
        pvq_norm[k] = sub(pvq_norm[k], 8);
        IF (pvq_norm[k] < 0)
        {
            pvq_norm[k] = 0;
            move16();
        }

        push_indice_fx(st_fx, IND_HVQ_PVQ_GAIN, pvq_norm[k], HVQ_PVQ_GAIN_BITS);
        pvq_bits = add(pvq_bits, HVQ_PVQ_GAIN_BITS);

        pvq_norm[k] = add(pvq_norm[k], 8);

        indx = pvq_norm[k];
        move16();
        normq = L_add(dicn_fx[indx], 0);  /* in Q14 */
        j = 0;
        move16();
        IF (sub(k, sub(pvq_bands, n_sel_bnds)) >= 0)
        {
            i = band_start_harm[*pSelBnds++];
            move16();
            pCoefsOut = coefs_out + i;
        }
        WHILE (sub(j, hvq_band_width[k]) < 0)
        {
            IF (L_sub(*pCoefsOut, 0) == 0)
            {
                acc = L_mult(*pCoefsPvq++, fg_pred[k]); /* in Q(15 + 1 + 12 = 28) */
                tmp16 = extract_h(acc);  /* in Q(28 - 16 = 12) */
                Mpy_32_16_ss(normq, tmp16, &acc, &dontCare16);  /* acc(Q11), normq(Q14), tmp16(Q12) */
                *pCoefsOut = L_shl(acc, 12 - 11); /* Q12 */
                j = add(j, 1);
            }
            pCoefsOut++;
        }
    }
    bits = add(bits, pvq_bits);

    /* Noise fill unqantized coeffs with one gain per group */
    pCoefsOut = &coefs_out[-1];
    FOR (i = 0; i < HVQ_NF_GROUPS; i++)
    {
        FOR (j = 0; j < bin_th2; j++)
        {
            IF (*(++pCoefsOut) == 0)
            {
                whiteNoise = Random(&nf_seed);  /* Q15 */
                Mpy_32_16_ss(nf_gains[i], whiteNoise, pCoefsOut, &dontCare16); /* nf_gains in Q12. *pCoefsOut in Q12 */
            }
        }
    }

    return bits;
}

/*--------------------------------------------------------------------------
 * quant_peaks_fx()
 *
 * Applies VQ on input vector
 *--------------------------------------------------------------------------*/

static void quant_peaks_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    const Word32 *vect_in,        /* i  : Target vector in Q12    */
    Word32 *vect_out,       /* i/o: Quantized vector in Q12 */
    const Word32 *peak_gain,      /* i  : Peak gain vector in Q12 */
    Word16 *vq_idx,         /* o  : Codebook index          */
    const Word16 overlap,         /* i  : Overlap indicator       */
    const Word32 brate,           /* i  : Core bitrate            */
    const Word16 Npeaks           /* i  : Number of peaks         */
)
{
    Word16 x[4]; /* Qx */
    Word16 xq[4]; /* Q15 */
    Word16 weights[4];  /* Q0 */
    Word16 *pWeights;
    Word16 i, cb_class, search_overlap, indx, cbSize;
    Word16 expPeakGain, manPeakGain, expIn, manIn;
    Word32 vectIn, absPeakGain1, absPeakGain;
    UWord16 dontCare;
    Word16 Qx_vec[4];
    Word16 Qx = 15;

    set16_fx(weights,1,4);

    /* Divide vect_in[] by peak_gain to yield x[]. */
    expPeakGain = norm_l(*peak_gain);  /* exponent */
    manPeakGain = extract_h(L_shl(*peak_gain, expPeakGain)); /* mantissa */
    manPeakGain = abs_s(manPeakGain); /* Prepare for div_s() only accepting +ve. */
    FOR (i = 0; i < 4; i++)
    {
        indx = hvq_index_mapping_fx[i];
        move16();
        vectIn = L_add(vect_in[indx], 0);
        expIn = norm_l(vectIn);                  /* exponent */
        expIn = sub(expIn, 1);
        expIn = s_min(expIn, expPeakGain);       /* highest Q is Q15 */
        manIn = extract_h(L_shl(vectIn, expIn)); /* mantissa */
        manIn = abs_s(manIn);  /* Prepare for div_s() only accepting +ve. */

        x[i] = div_s(manIn, manPeakGain); /* in Q(15+expIn-expPeakGain) */

        Qx_vec[i] = add(15, sub(expIn, expPeakGain));
        Qx = s_min(Qx, Qx_vec[i]);

        /* Restore the sign destroyed by abs operations. */
        if (L_xor(vectIn, *peak_gain) < 0) /* Check the sign bits (MSB). */
        {
            x[i] = negate(x[i]);
        }
    }
    FOR (i = 0; i < 4; i++)
    {
        IF (sub(Qx_vec[i], Qx) != 0)
        {
            x[i] = shr(x[i], sub(Qx_vec[i], Qx));   /* Qx */
        }
    }
    absPeakGain  = L_abs(peak_gain[0]);
    IF (vect_out[0] != 0)
    {
        absPeakGain1 = L_abs(peak_gain[-1]);
        IF (L_sub(absPeakGain1, absPeakGain) > 0)
        {
            weights[0] = 0;
            move16();
            if (vect_out[1] != 0)
            {
                weights[1] = 0;
                move16();
            }
        }
    }
    IF (overlap > 0)
    {
        absPeakGain1 = L_abs(peak_gain[1]);
        IF (L_sub(absPeakGain1, absPeakGain) > 0)
        {
            indx = sub(4, overlap);
            pWeights = &weights[indx];
            FOR (i = 0; i < overlap; i++)
            {
                *pWeights++ = 0;
                move16();
            }
        }
    }

    /* Classify */
#if HVQ_VQ_DIM != 5
#error w_vquant_fx() is hard-wired to dim = 4 = (HVQ_VQ_DIM - 1).
#endif
    cb_class = w_vquant_fx(x, Qx, weights, 0, hvq_class_c_fx, HVQ_NUM_CLASS, 0);
    IF (brate == HQ_24k40)
    {
        indx = sub(HVQ_MAX_PEAKS_24k, Npeaks);
        search_overlap = hvq_cb_search_overlap24k[indx];
        move16();
    }
    ELSE
    {
        indx = sub(HVQ_MAX_PEAKS_32k, Npeaks);
        search_overlap = hvq_cb_search_overlap32k[indx];
        move16();
    }

    /* Quantize */
    cbSize = add(HVQ_CB_SIZE/2, search_overlap);
    IF ( cb_class == 0 )
    {
        *vq_idx = w_vquant_fx(x, Qx, weights, xq, hvq_peak_cb_fx, cbSize, 0);
        push_indice_fx(st_fx, IND_HVQ_PEAKS, 0, 1 );
    }
    ELSE IF( sub(cb_class, 1) == 0 )
    {
        indx = sub(HVQ_CB_SIZE*2, shl(search_overlap,2));
        *vq_idx = w_vquant_fx(x, Qx, weights, xq, &hvq_peak_cb_fx[indx], cbSize, 0);
        *vq_idx = add(*vq_idx, sub(HVQ_CB_SIZE/2, search_overlap));
        push_indice_fx(st_fx, IND_HVQ_PEAKS, 0, 1 );
    }
    ELSE IF( sub(cb_class, 2) == 0 )
    {
        indx = sub(HVQ_CB_SIZE*2, shl(search_overlap,2));
        *vq_idx = w_vquant_fx(x, Qx, weights, xq, &hvq_peak_cb_fx[indx], cbSize, 1);
        *vq_idx = add(*vq_idx, sub(HVQ_CB_SIZE/2, search_overlap));
        push_indice_fx(st_fx, IND_HVQ_PEAKS, 1, 1 );
    }
    ELSE
    {
        *vq_idx = w_vquant_fx(x, Qx, weights, xq, hvq_peak_cb_fx, cbSize, 1);
        push_indice_fx(st_fx, IND_HVQ_PEAKS, 1, 1 );
    }

    FOR (i = 0; i < 4; i++)
    {
        indx = hvq_index_mapping_fx[i];
        move16();
        IF (weights[i] != 0)
        {
            Mpy_32_16_ss(*peak_gain, xq[i], &vect_out[indx], &dontCare); /* peak_gains in Q12, xq in Q15 -> Q12. */
            move32();
        }
    }
    vect_out[2] = *peak_gain;
    move32();    /* vect_out in Q12 */

    return;
}

/*--------------------------------------------------------------------------
 * code_pos()
 *
 * Code pulse positions
 *--------------------------------------------------------------------------*/

static Word16 sparse_code_pos_fx(
    const Word16 *inp,
    const Word16 length,
    Word16 *result
)
{
    Word16 layer2[HVQ_CP_L2_MAX];
    Word16 layer_length;
    Word16 i,j,tmp;
    Word16 val, idx;
    Word16 bits = 0;
    Word16 mask;

    set16_fx(layer2, 0, HVQ_CP_L2_MAX);

    /*layer_length = (short)((float)length/HVQ_CP_L1_LEN + 0.5); */
    layer_length = round_fx(L_mult0(length, 13107));    /* 0+16-16, 13107 is 1/5 in Q16  */

    FOR (j = 0; j < layer_length; j++)
    {
        tmp = s_min(i_mult2(add(j,1),HVQ_CP_L1_LEN), length);
        FOR (i = i_mult2(j, HVQ_CP_L1_LEN); i < tmp; i++)
        {
            IF (inp[i] != 0)
            {
                layer2[j] = 1;
                move16();
                BREAK;
            }
        }
    }

    FOR (i = 0; i < layer_length; i++)
    {
        result[i] = layer2[i];
        move16();
    }
    bits = add(bits, layer_length);

    FOR (j = 0; j < layer_length; j++)
    {
        IF (layer2[j] != 0)
        {
            val = 0;
            move16();
            tmp = s_min(i_mult2(add(j,1),HVQ_CP_L1_LEN), length);
            FOR (i = i_mult2(j, HVQ_CP_L1_LEN); i < tmp; i++)
            {
                val = shl(val, 1);
                val = s_or(val, inp[i]);
            }

            FOR (idx = 0; idx < HVQ_CP_MAP_LEN; idx++)
            {
                IF (sub(hvq_cp_layer1_map5[idx], val) == 0)
                {
                    BREAK;
                }
            }

            mask = shl(1, HVQ_CP_MAP_IDX_LEN - 1);
            FOR (i = 0; i < HVQ_CP_MAP_IDX_LEN; i++)
            {
                result[bits++] = shr(s_and(idx, mask), sub(HVQ_CP_MAP_IDX_LEN - 1, i));
                mask >>= 1;
            }
        }
    }

    return bits;
}

/*--------------------------------------------------------------------------
 * hvq_code_pos()
 *
 * Code pulse positions
 *--------------------------------------------------------------------------*/

static Word16 hvq_code_pos_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure      */
    const Word16 *inp,
    const Word16 length,
    const Word16 num_peaks
)
{
    Word16 sparse_result[4*HVQ_THRES_BIN_32k/HVQ_CP_L1_LEN];
    Word16 delta[HVQ_MAX_PEAKS_32k];
    Word16 peak_idx[HVQ_MAX_PEAKS_32k];
    Word16 inp_abs[HVQ_THRES_BIN_32k];
    Word16 inp_sign[HVQ_MAX_PEAKS_32k];

    Word16 i, j;
    Word16 bits;
    Word16 delta_max;
    Word16 delta_bits, sparse_bits;
    Word16 tmp;

    bits = 0;
    move16();

    /* Extract sorted peak index vector and sign vector */
    j = 0;
    move16();
    FOR(i = 0; i < length; i++)
    {
        inp_abs[i] = abs_s(inp[i]);
        IF (inp[i] != 0)
        {
            peak_idx[j] = i;
            move16();
            inp_sign[j] = inp[i];
            move16();
            j = add(j, 1);
        }
    }

    /* Calculate delta */
    delta[0] = add(peak_idx[0], HVQ_CP_HUFF_OFFSET);
    move16();
    delta_max = delta[0];
    move16();
    FOR (i = 1; i < num_peaks; i++)
    {
        delta[i] = sub(sub(peak_idx[i], peak_idx[i-1]), HVQ_CP_HUFF_OFFSET);
        if (sub(delta_max, delta[i]) < 0)
        {
            delta_max = delta[i];
            move16();
        }
    }

    /* Calculate bits needed for huffman coding of deltas */
    delta_bits = -1;
    move16();
    IF (sub(delta_max, HVQ_CP_HUFF_MAX) <= 0)
    {
        delta_bits = 0;
        move16();
        FOR (i = 0; i < num_peaks; i++)
        {
            delta_bits = add(delta_bits, hvq_cp_huff_len[delta[i]]);
        }
    }

    /* Calculate bits neeed for sparse coding */
    sparse_bits = sparse_code_pos_fx(inp_abs, length, sparse_result);

    /* Decide which coding mode to use */
    test();
    IF (sub(delta_bits, sparse_bits) > 0 || delta_bits < 0)
    {
        push_indice_fx(st_fx, IND_POS_IDX, HVQ_CP_SPARSE, 1);

        FOR (i = 0; i < sparse_bits; i++)
        {
            push_indice_fx(st_fx, IND_POS_IDX, sparse_result[i], 1);
        }
        bits = add(add(bits, sparse_bits), 1);
    }
    ELSE
    {
        push_indice_fx(st_fx, IND_POS_IDX, HVQ_CP_DELTA, 1);

        FOR (i = 0; i < num_peaks; i++)
        {
            j = delta[i];
            move16();
            push_indice_fx(st_fx, IND_POS_IDX, hvq_cp_huff_val[j], hvq_cp_huff_len[j]);
        }
        bits = add(add(bits, delta_bits), 1);
    }

    /* Send sign */
    FOR (i = 0; i < num_peaks; i++)
    {
        tmp = 1;
        move16();
        if (inp_sign[i] < 0)
        {
            tmp = 0;
            move16();
        }
        push_indice_fx(st_fx, IND_POS_IDX, tmp, 1);
    }
    bits = add(bits, num_peaks);

    return bits;
}

