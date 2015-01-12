/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"        /* required for wmc_tool */


#define PK_VQ_NOISE_DELTA  ((Word16)3277)  /* 0.1 in Q15 */

/* Local functions */
static void dequant_peaks_fx( Decoder_State_fx *st_fx, Word32 *vect_out, const Word32 *peak_gain);
static Word16 hvq_dec_pos_fx(Decoder_State_fx *st_fx, Word16 *pos_vec, const Word16 length, const Word16 num_peaks );
static Word16 sparse_dec_pos_fx(Decoder_State_fx *st_fx, Word16 *out, const Word16 length );

/*--------------------------------------------------------------------------
 * hvq_dec_fx()
 *
 * HVQ decoder
 *--------------------------------------------------------------------------*/

void hvq_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure */
    const Word16  num_bits,        /* i : Number of available bits */
    const Word32  core_brate,      /* i : Core bit-rate */
    const Word16 *ynrm,            /* i : Envelope coefficients          */
    Word16 *R,               /* i/o: Bit allocation/updated bit allocation */
    Word16 *noise_level,     /* o : Noise level in Q15 */
    Word16 *peak_idx,        /* o : Peak position vector */
    Word16 *Npeaks,          /* o : Total number of peaks */
    Word32 *coefsq_norm,     /* o : Output vector in Q12 */
    const Word16  core
)
{
    Word16 i;
    Word16 bits;
    Word16 noise_level_idx;

    bits = num_bits;

    FOR( i = 0; i < HVQ_BWE_NOISE_BANDS; i++ )
    {
        noise_level_idx = get_next_indice_fx( st_fx, 2 ); /* 2-bits => max noise_level-idx = 3 */
        noise_level[i] = i_mult(noise_level_idx, PK_VQ_NOISE_DELTA);
        move16();/* max noise_level=3*0.1 => Q15 is good enough */

        bits = sub(bits, 2);
    }

    peak_vq_dec_fx( st_fx, coefsq_norm, (Word16)core_brate, bits, ynrm, R, peak_idx,
                    Npeaks, core );
}

/*--------------------------------------------------------------------------
 * peak_vq_dec()
 *
 * Vector de-quantization of MDCT peaks
 *--------------------------------------------------------------------------*/

void peak_vq_dec_fx(
    Decoder_State_fx *st_fx,         /* i/o: decoder state structure */
    Word32 *coefs_out,        /* o  : Output coefficient vector Q12 */
    const Word16 brate,             /* i  : Core bitrate                  */
    const Word16 num_bits,          /* i  : Number of bits for HVQ        */
    const Word16 *ynrm,             /* i  : Envelope coefficients         */
    Word16 *R,                /* i/o: Bit allocation/updated bit allocation */
    Word16 *vq_peak_idx,      /* o  : Peak position vector          */
    Word16 *Npeaks,           /* o  : Number of peaks               */
    const Word16 core
)
{
    Word16 vq_peaks, i, j, k, FlagN, hcode_l, diff;
    Word16 bin_th, bin_th2, max_peaks, pvq_bands;
    Word16 nf_gains_idx[HVQ_NF_GROUPS], pgain_difidx[HVQ_MAX_PEAKS_32k], pvq_norm[MAX_PVQ_BANDS];
    Word16 gain_bits_array[MAX_PVQ_BANDS];
    Word16 pos_bits;
    Word32 nf_gains_fx[HVQ_NF_GROUPS], peak_gains_fx[HVQ_MAX_PEAKS_32k];
    Word16 pvq_vector[HVQ_PVQ_COEFS*MAX_PVQ_BANDS];
    Word16 res_vec[HVQ_THRES_BIN_32k];
    Word16 k_sort[HVQ_MAX_PVQ_WORDS];
    Word16 pvq_inp_vector[HVQ_PVQ_COEFS*HVQ_MAX_PVQ_WORDS], pvq_maxpulse[HVQ_MAX_PVQ_WORDS];
    Word16 npulses[MAX_PVQ_BANDS];
    Word16 pvq_bits, Rk[MAX_PVQ_BANDS];
    Word16 fg_pred[NB_SFM_MAX];

    Word32 *pCoefsOut;
    Word16 whiteNoise;
    UWord16 dontCare;
    Word32 acc;
    Word16 *pPvqVector;
    Word32 manE_peak, manPeakGains, manPkEnrg;  /* Due to very wide dynamic range, use floating point format, i.e., (man, exp) */
    Word16 expE_peak, expPeakGains, expPkEnrg;
    Word16 *pSelBnds;
    Word16 sel_bnds[HVQ_NUM_SFM_24k];
    Word16 hvq_band_end[MAX_PVQ_BANDS];
    Word16 hvq_band_start[MAX_PVQ_BANDS];
    Word16 hvq_band_width[MAX_PVQ_BANDS];
    Word16 n_sel_bnds;
    Word32 normq;
    UWord32 lsb;

    Word16 nf_seed = RANDOM_INITSEED;
    move16();

    set16_fx( gain_bits_array, 0, MAX_PVQ_BANDS );
    set16_fx( pvq_vector, 0, HVQ_PVQ_COEFS*MAX_PVQ_BANDS );
    set16_fx( npulses, 0, MAX_PVQ_BANDS );
    set16_fx( pvq_inp_vector, 0, HVQ_PVQ_COEFS*HVQ_MAX_PVQ_WORDS );

    /* Set bitrate dependent variables */
    IF (sub(brate, HQ_24k40) == 0)
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

    /* Get number of peaks */
    vq_peaks = get_next_indice_fx( st_fx, 5 );
    vq_peaks = sub(max_peaks, vq_peaks);
    *Npeaks = vq_peaks;
    move16();
    diff = 5;
    move16();

    /* De-quantize peak positions */
    FOR (i = 0; i < bin_th; i++)
    {
        res_vec[i] = 0;
        move16();
    }

    /* Unpack PVQ codewords */
    pos_bits = hvq_dec_pos_fx(st_fx, res_vec, bin_th, vq_peaks);
    diff = add(diff, pos_bits);

    j = 0;
    move16();
    FOR (i = 0; i < bin_th; i++)
    {
        IF ( res_vec[i] != 0)
        {
            vq_peak_idx[j++] = i;
            move16();
        }
    }

    /* Huffman or differential coding */
    FlagN = (Word16) get_next_indice_fx( st_fx, 1 );

    /* De-quantize peak gains */
    pgain_difidx[0] = get_next_indice_fx( st_fx, GAIN0_BITS );
    peak_gains_fx[0] = dicn_pg_fx[pgain_difidx[0]]; /* Q12 */               move32();
    if (res_vec[vq_peak_idx[0]] < 0)
    {
        peak_gains_fx[0] = L_negate(peak_gains_fx[0]);
        move16();
    }

    hcode_l = 0;
    move16();
    IF (FlagN)
    {
        huff_dec_fx( st_fx, vq_peaks-1, MAX_PG_HUFFLEN, NUM_PG_HUFFLEN, hvq_pg_huff_thres, hvq_pg_huff_offset, hvq_pg_huff_tab, &pgain_difidx[1] );

        FOR (i = 1; i < vq_peaks; i++)
        {
            hcode_l = add(hcode_l, pgain_huffsizn[pgain_difidx[i]]);
            move16();/* indirect addressing*/
        }
    }
    ELSE
    {
        FOR (i = 1; i < vq_peaks; i++)
        {
            pgain_difidx[i] = get_next_indice_fx(st_fx, GAINI_BITS );
            move16();
            hcode_l = add(hcode_l, GAINI_BITS);
        }
    }

    FOR (i = 1; i < vq_peaks; i++)
    {
        pgain_difidx[i] = add(pgain_difidx[i], sub(pgain_difidx[i - 1],15));
        move16();
        peak_gains_fx[i] = dicn_pg_fx[pgain_difidx[i]];
        move32();/* Q12  move16(); */
        if (res_vec[vq_peak_idx[i]] < 0)
        {
            peak_gains_fx[i] = L_negate(peak_gains_fx[i]);
            move32();
        }
    }

    /* Scale up peak gains and accumulate peak energy */
    manE_peak = L_deposit_l(0);
    expE_peak = 32;
    move16();
    FOR (i = 0; i < vq_peaks; i++)
    {
        peak_gains_fx[i] = L_shl(peak_gains_fx[i], 2);
        move32(); /* Q12 */
        /* Use floating point operation to deal with wide dynamic range.
         * 32-bit mantissa is used here. It should be even more accurate than
         * the floating-point reference code with 24-bit mantissa! */
        expPeakGains = norm_l(peak_gains_fx[i]);
        manPeakGains = L_shl(peak_gains_fx[i], expPeakGains);
        Mpy_32_32_ss(manPeakGains, manPeakGains, &manPkEnrg, &lsb);  /* peak_gains square */
        expPkEnrg = shl(expPeakGains, 1); /* Multiply by 2 due to squaring. */

        floating_point_add(&manE_peak, &expE_peak, manPkEnrg, expPkEnrg);
    }
    Mpy_32_16_ss(manE_peak, 9830, &manE_peak, &dontCare); /* Multiply by 0.3 */
    Mpy_32_16_ss(st_fx->manE_peak_mem, 22938, &st_fx->manE_peak_mem, &dontCare); /* Multiply by 0.7 */
    floating_point_add(&manE_peak, &expE_peak, st_fx->manE_peak_mem, st_fx->expE_peak_mem);
    st_fx->manE_peak_mem = manE_peak;
    st_fx->expE_peak_mem = expE_peak;
    /* Number of bits used for peak gain quantization */
    diff = add(diff, add(FLAGN_BITS + GAIN0_BITS, hcode_l));

    /* De-quantize peaks */
    FOR (i = 0; i < vq_peaks; i++)
    {
        dequant_peaks_fx( st_fx, &coefs_out[vq_peak_idx[i]-2], &peak_gains_fx[i]); /* coefs_out in Q12, peak_gains_fx in Q14 */
        diff = add(diff, 9);
    }

    FOR (i = 0; i < HVQ_NF_GROUPS; i++)
    {
        nf_gains_idx[i] = get_next_indice_fx( st_fx, 5 );
        move16();
        nf_gains_fx[i] = L_shr(dicn_fx[nf_gains_idx[i]],1);
        move32(); /* nf_gains in Q14 */
        diff = add(diff, 5);
    }
    pvq_bits = sub(num_bits, diff);

    /* Calculate number of PVQ bands to code and assign bits */
    pvq_bands = hvq_pvq_bitalloc_fx(pvq_bits, brate, st_fx->bwidth_fx, ynrm, manE_peak, expE_peak, Rk, R, sel_bnds,
                                    &n_sel_bnds);

    pvq_bits = sub(pvq_bits, i_mult2(HVQ_PVQ_GAIN_BITS, pvq_bands));
    /* Get band limits for concatenated PVQ target */
    hvq_concat_bands_fx(pvq_bands, sel_bnds, n_sel_bnds, hvq_band_start,
                        hvq_band_width, hvq_band_end);

    FOR (k = 0; k < pvq_bands; k++)
    {
        k_sort[k] = k;
        move16();
    }

    pvq_decode_frame_fx(st_fx, pvq_vector, npulses, pvq_inp_vector, hvq_band_start, hvq_band_end, hvq_band_width, pvq_bands, Rk, pvq_bits, core );


    fine_gain_pred_fx( hvq_band_start, hvq_band_end, hvq_band_width, k_sort, npulses, pvq_maxpulse, NULL,
                       pvq_bands, pvq_vector, pvq_inp_vector, fg_pred, core );

    fine_gain_dec_fx( st_fx, k_sort, pvq_bands, gain_bits_array, fg_pred);

    apply_gain_fx(k_sort, hvq_band_start, hvq_band_end, pvq_bands, fg_pred, pvq_vector);

    pPvqVector = pvq_vector;
    pCoefsOut = coefs_out;
    pSelBnds = sel_bnds;
    move16();
    FOR (k = 0; k < pvq_bands; k++)
    {
        pvq_norm[k] = get_next_indice_fx( st_fx, HVQ_PVQ_GAIN_BITS );
        pvq_norm[k] = add(pvq_norm[k], 8);
        move16();

        diff = add(diff, HVQ_PVQ_GAIN_BITS);

        j = 0;
        move16();
        IF (sub(k, sub(pvq_bands, n_sel_bnds)) >= 0)
        {
            i = band_start_harm[*pSelBnds++];
            move16();
            move16();
            pCoefsOut = coefs_out + i;
        }
        normq = L_add(dicn_fx[pvq_norm[k]], 0);
        WHILE (sub(j, hvq_band_width[k]) < 0)
        {
            IF (L_sub(*pCoefsOut, 0) == 0)
            {
                Mpy_32_16_ss(normq, *pPvqVector++, &acc, &dontCare);  /* acc(Q11), normq(Q14), pvq_vector(Q12) */
                *pCoefsOut = L_shl(acc, 12 - 11); /* Q12 */                 move32();
                j = add(j, 1);
            }
            pCoefsOut++;
        }
    }

    /* Noise fill unqantized coeffs with one gain per group */
    pCoefsOut = &coefs_out[-1];
    FOR (i = 0; i < HVQ_NF_GROUPS; i++)
    {
        FOR (j = 0; j < bin_th2; j++)
        {
            IF (*(++pCoefsOut) == 0)
            {
                whiteNoise = Random(&nf_seed);  /* Q15 */
                Mpy_32_16_ss(nf_gains_fx[i], whiteNoise, &acc, &dontCare); /* nf_gains_fx[] in Q14 */
                *pCoefsOut = L_shr(acc, 14-12);  /* Q12 */                  move32();
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------
 * dequant_peaks()
 *
 * Reads codebook vector and scales peak
 *--------------------------------------------------------------------------*/

static void dequant_peaks_fx(
    Decoder_State_fx *st_fx,      /* i/o: decoder state structure */
    Word32 *vect_out,      /* o  : Quantized vector in Q12 */
    const Word32 *peak_gain      /* i  : Peak gain in Q12        */
)
{
    Word16 xq[4];
    const Word16 *tmp;
    Word16 i, hvq_cb_rev;
    Word16 cb_idx, indx;
    Word32 absPeakGain1, absPeakGain;
    UWord16 dontCare;

    hvq_cb_rev = get_next_indice_fx( st_fx, 1 );
    cb_idx = get_next_indice_fx( st_fx, 8 );

    indx = shl(cb_idx,2);
    IF ( hvq_cb_rev )
    {
        indx = add(indx,3);
        tmp = &hvq_peak_cb_fx[indx];
        FOR (i = 0; i < 4; i++)
        {
            xq[i] = *tmp--; /* Q15 */                                     move16();
        }
    }
    ELSE
    {
        tmp = &hvq_peak_cb_fx[indx];
        FOR (i = 0; i < 4; i++)
        {
            xq[i] = *tmp++; /* Q15 */                                     move16();
        }
    }

    absPeakGain  = L_abs(peak_gain[0]);

    IF(vect_out[0] == 0)
    {
        Mpy_32_16_ss(*peak_gain, xq[0], &vect_out[0], &dontCare); /* vect_out in Q12 */
        Mpy_32_16_ss(*peak_gain, xq[1], &vect_out[1], &dontCare); /* Q12 */
    }
    ELSE
    {
        absPeakGain1 = L_abs(peak_gain[-1]);
        IF(L_sub(absPeakGain1, absPeakGain) <= 0)
        {
            Mpy_32_16_ss(*peak_gain, xq[0], &vect_out[0], &dontCare); /* vect_out in Q12 */
            Mpy_32_16_ss(*peak_gain, xq[1], &vect_out[1], &dontCare); /* Q12 */
        }
        ELSE
        {
            IF(vect_out[1] == 0 || (L_sub(absPeakGain1, absPeakGain) <= 0))
            {
                Mpy_32_16_ss(*peak_gain, xq[1], &vect_out[1], &dontCare);
            }
        }
    }
    vect_out[2] = *peak_gain; /* vect_out in Q12 */
    Mpy_32_16_ss(*peak_gain, xq[2], &vect_out[3], &dontCare);
    Mpy_32_16_ss(*peak_gain, xq[3], &vect_out[4], &dontCare);

    return;
}


/*--------------------------------------------------------------------------
 * hvq_dec_pos()
 *
 * HVQ decode peak positions
 *--------------------------------------------------------------------------*/

static Word16 hvq_dec_pos_fx(
    Decoder_State_fx *st_fx,                      /* i/o: decoder state structure   */
    Word16 *pos_vec,
    const Word16 length,
    const Word16 num_peaks
)
{
    Word16 peak_idx[HVQ_MAX_PEAKS_32k];
    Word16 delta[HVQ_MAX_PEAKS_32k];
    Word16 sign_vec[HVQ_MAX_PEAKS_32k];

    Word16 mode;
    Word16 num_bits, tmp;
    Word16 i, j;

    num_bits = 0;
    move16();
    set16_fx(pos_vec, 0, length);

    mode = get_next_indice_fx(st_fx, 1);
    num_bits = add(num_bits, 1);

    IF (mode == HVQ_CP_DELTA)
    {
        huff_dec_fx(st_fx, num_peaks, HVQ_CP_HUFF_MAX_CODE, HVQ_CP_HUFF_NUM_LEN, hvq_cp_huff_thres, hvq_cp_huff_offset, hvq_cp_huff_tab, delta);

        FOR (i = 0; i < num_peaks; i++)
        {
            num_bits = add(num_bits, hvq_cp_huff_len[delta[i]]);
        }

        peak_idx[0] = sub(delta[0], HVQ_CP_HUFF_OFFSET);
        FOR (i = 1; i < num_peaks; i++)
        {
            peak_idx[i] = add(add(delta[i], peak_idx[i-1]), HVQ_CP_HUFF_OFFSET);
            move16();
        }

        FOR (i = 0; i < num_peaks; i++)
        {
            pos_vec[peak_idx[i]] = 1;
            move16();
        }
    }
    ELSE
    {
        tmp = sparse_dec_pos_fx(st_fx, pos_vec, length);
        num_bits = add(num_bits, tmp);
    }

    FOR (i = 0; i < num_peaks; i++)
    {
        IF (get_next_indice_1_fx(st_fx) == 0)
        {
            sign_vec[i] = -1;
            move16();
        }
        ELSE
        {
            sign_vec[i] = 1;
            move16();
        }
    }
    num_bits = add(num_bits, num_peaks);

    j = 0;
    move16();
    FOR (i = 0; i < length; i++)
    {
        if (sub(pos_vec[i], 1) == 0)
        {
            pos_vec[i] = i_mult2(pos_vec[i], sign_vec[j++]);
            move16();
        }
    }

    return num_bits;
}

/*--------------------------------------------------------------------------
 * sparse_dec_pos()
 *
 * Sparse decode positions
 *--------------------------------------------------------------------------*/

static Word16 sparse_dec_pos_fx(
    Decoder_State_fx *st_fx,                      /* i/o: decoder state structure   */
    Word16 *out,
    const Word16 length
)
{
    Word16 layer2[HVQ_CP_L2_MAX];
    Word16 layer_length;
    Word16 i, j, tmp;
    Word16 bits;
    Word16 idx, val;

    set16_fx(layer2, 0, HVQ_CP_L2_MAX);
    set16_fx(out, 0, length);
    bits = 0;
    move16();

    /*layer_length = (short)((float)length/HVQ_CP_L1_LEN + 0.5); */
    layer_length = round_fx(L_mult0(length, 13107));    /* 0+16-16, 13107 is 1/5 in Q16   */

    FOR (i = 0; i < layer_length; i++)
    {
        layer2[i] = get_next_indice_1_fx(st_fx);
        move16();
    }
    bits = add(bits, layer_length);

    FOR (j = 0; j < layer_length; j++)
    {
        IF (sub(layer2[j], 1) == 0)
        {
            idx = get_next_indice_fx(st_fx, HVQ_CP_MAP_IDX_LEN);
            bits = add(bits, HVQ_CP_MAP_IDX_LEN);

            val = hvq_cp_layer1_map5[idx];
            move16();

            tmp = i_mult2(j, HVQ_CP_L1_LEN);
            FOR (i = sub(s_min(i_mult2(add(j,1), HVQ_CP_L1_LEN), length), 1); i >= tmp; i--)
            {
                out[i] = s_and(val, 1);
                move16();
                val = lshr(val, 1);
            }
        }
    }

    return bits;
}
