/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "rom_com_fx.h" /* Static table prototypes FIP version    */
#include "stl.h"        /* required for wmc_tool */
#include "prot_fx.h"

/*--------------------------------------------------------------------------*
  * Local functions
  *--------------------------------------------------------------------------*/

static void overlap_hq_bwe_fx( const Word32 *hq_swb_overlap_buf, Word32 *coeff_out, const Word16 n_swb_overlap_offset,
                               const Word16 n_swb_overlap, const Word16 *R, const Word16 num_env_bands, const Word16 num_sfm, const Word16 *sfm_end);

/*--------------------------------------------------------------------------*
* hq_swb_harmonic_calc_norm_envelop()
*
* Calculate normalization envelop
*--------------------------------------------------------------------------*/

void hq_swb_harmonic_calc_norm_envelop_fx(
    Word32 *L_SWB_signal,         /* i  : input signal                Q=12*/
    Word32 *L_envelope,           /* o  : output envelope             Q=12*/
    Word16 L_swb_norm,            /* i  : length of normaliztion          */
    Word16 SWB_flength            /* i  : length of input signal          */
)
{

    Word16 lookback;
    Word16 env_index;
    Word16 n_freq;
    Word16 n_lag_now;
    Word16 n_lag;
    Word16 i;
    Word32 L_tmp;

    lookback = shr(L_swb_norm,1);
    env_index = 0;
    move16();
    FOR (n_freq = 0; n_freq < lookback; n_freq++)
    {
        n_lag_now = add(lookback,n_freq);

        /* Apply MA filter */
        L_envelope[env_index] = EPSILLON_FX;
        move16();

        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_abs(L_SWB_signal[n_lag]);
            L_envelope[env_index] = L_add(L_envelope[env_index], L_tmp);
            move32();
        }
        env_index = add(env_index, 1);
    }

    n_lag_now = L_swb_norm;
    move16();

    FOR (n_freq = 0; n_freq < SWB_flength-L_swb_norm; n_freq++)
    {
        /* Apply MA filter */
        L_envelope[env_index] = EPSILLON_FX;
        move16();
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_abs(L_SWB_signal[add(n_freq,n_lag)]);
            L_envelope[env_index] = L_add(L_envelope[env_index], L_tmp);
            move32();
        }
        env_index = add(env_index, 1);
    }

    i = 0;
    move16();
    FOR (n_freq = SWB_flength-L_swb_norm; n_freq<SWB_flength-lookback; n_freq++)
    {
        n_lag_now = sub(L_swb_norm, i);

        /* Apply MA filter */
        L_envelope[env_index] = L_deposit_l(EPSILLON_FX);
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_abs(L_SWB_signal[add(n_freq, n_lag)]);
            L_envelope[env_index] = L_add(L_envelope[env_index], L_tmp);
            move32();
        }
        env_index = add(env_index, 1);
        i = add(i, 1);
    }

    return;
}


/*--------------------------------------------------------------------------*
* noise_level_calc_fx()
*
* Calculate noise level and limited band
*--------------------------------------------------------------------------*/

void limit_band_noise_level_calc_fx(
    const Word16 *wnorm,             /* i  : reordered norm of sub-vectors        */
    Word16 *limit,             /* o  : highest band of bit allocation       */
    const Word32 core_brate,         /* i  : bit rate                             */
    Word16 *noise_level        /* o  : noise level Q15                      */
)
{
    Word16 ener_limit, ener_sum;
    Word16 i;
    Word16 nb_sfm;
    Word32 fact;
    Word16 tmp;

    nb_sfm = *limit;
    move16();
    ener_limit = 0;
    move16();
    *noise_level = 0;
    move16();
    FOR (i = 0; i < 10; i++)
    {
        ener_limit = add(ener_limit, wnorm[i]);
        *noise_level = add(*noise_level, abs_s(sub(wnorm[i+1], wnorm[i])));
        move16();

    }
    ener_sum = ener_limit;
    move16();

    tmp = sub(nb_sfm, 1);
    FOR (i = 10; i < tmp; i++)
    {
        ener_sum = add(ener_sum, wnorm[i]);
        *noise_level = add(*noise_level, abs_s(sub(wnorm[i+1], wnorm[i])));
        move16();
    }
    ener_sum = add(ener_sum, wnorm[nb_sfm-1]);


    fact = L_add(2022929597, 0);
    if (L_sub(core_brate, HQ_24k40) == 0)
    {
        fact = L_add(1900523029, 1);
    }

    fact = Mult_32_16(fact, ener_sum);
    i = 9;
    move16();

    WHILE (L_sub(L_deposit_h(ener_limit), fact) < 0)
    {
        ener_limit = add(ener_limit, wnorm[++i]);
    }
    *limit = i;
    move16();

    /* calculate noise level for spectrum filling */
    if (*noise_level < 0)
    {
        *noise_level = 0;
        move16();
    }

    IF (sub(*noise_level, shr(ener_sum, 2)) >= 0)
    {
        *noise_level = 0;
        move16();
    }
    ELSE
    {
        IF(ener_sum !=0)
        {
            *noise_level = sub(8192, div_s(*noise_level, ener_sum));
        }
        ELSE
        {
            *noise_level = 8192;
        }
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * build_nf_codebook_fx()
 *
 * Build noise-fill codebook for HQ mode
 * NOTE: Q values preliminary
 *--------------------------------------------------------------------------*/

Word16 build_nf_codebook_fx(      /* o  : Number of coefficients in nf codebook Q=0*/
    const Word16 flag_32K_env_ho, /* i  : Envelope attenuation hangover flag Q=0*/
    const Word16 *coeff,          /* i  : Coded spectral coefficients   Q=12*/
    const Word16 *sfm_start,      /* i  : Subband start indices         Q=0*/
    const Word16 *sfmsize,        /* i  : Subband widths                Q=0*/
    const Word16 *sfm_end,        /* i  : Subband end indices           Q=0*/
    const Word16 last_sfm,        /* i  : Last coded  band              Q=0*/
    const Word16 *R,              /* i  : Per-band bit allocation       Q=0*/
    Word16 *CodeBook,       /* o  : Noise-fill codebook           Q=12*/
    Word16 *CodeBook_mod    /* o  : Densified noise-fill codebook Q=12*/
)
{
    Word16 sfm_base;
    Word16 sfm;
    Word16 E_cb_vec;
    Word16 i,j;
    Word16 cb_size;

    /* Build codebook */

    cb_size = 0;
    move16();

    FOR ( sfm = 0; sfm <= last_sfm; sfm++ )
    {
        IF (R[sfm] != 0)
        {
            IF (flag_32K_env_ho)
            {
                /* Build compressed (+/- 1) noise-fill codebook */
                sfm_base = sfm_start[sfm];
                move16();
                FOR (i = 0; i < sfmsize[sfm]/8; i++)
                {
                    E_cb_vec = 0;
                    move16();
                    FOR (j = sfm_base+i*8; j < sfm_base+(i+1)*8; j++)
                    {
                        IF (coeff[j] > 0.0f)
                        {
                            CodeBook_mod[cb_size] = 1<<12;
                            move16(); /* set to 1, Q value 12 */
                            E_cb_vec = add(E_cb_vec,1);
                        }
                        ELSE IF (coeff[j] < 0.0f)
                        {
                            CodeBook_mod[cb_size] = -1<<12;
                            move16(); /* set to -1, Q value 12 */
                            E_cb_vec = add(E_cb_vec,1);
                        }
                        ELSE
                        {
                            CodeBook_mod[cb_size] = 0;
                            move16();
                        }
                        cb_size = add(cb_size,1);
                    }

                    if (E_cb_vec < 2)
                    {
                        cb_size = sub(cb_size,8);
                    }
                }
            }
            ELSE
            {
                FOR (j = sfm_start[sfm]; j < sfm_end[sfm]; j++)
                {
                    CodeBook[cb_size] = coeff[j];
                    move16();
                    cb_size = add(cb_size,1);
                }
            }
        }
    }

    IF (flag_32K_env_ho)
    {
        FOR (j = 0; j < cb_size; j++)
        {
            IF (CodeBook_mod[j] != 0.0f)
            {
                /* Densify codebook */
                CodeBook[j] = -4096;
                move16();   /* -1 in Q12 */
                if (CodeBook_mod[j] > 0)
                {
                    CodeBook[j] = 4096;
                    move16();   /* 1 in Q12 */
                }

                IF (CodeBook_mod[cb_size-j-1] != 0)
                {
                    CodeBook[j] = shl(CodeBook[j], 1);
                    move16();  /* Mult by 2 */
                }
            }
            ELSE
            {
                CodeBook[j] = CodeBook_mod[cb_size-j-1];
                move16();
            }
        }
    }

    return cb_size;
}


/*--------------------------------------------------------------------------*
* find_last_band()
*
* Find the last band which has bits allocated
*--------------------------------------------------------------------------*/

Word16 find_last_band_fx(        /* o  : index of last band              */
    const Word16 *bitalloc,      /* i  : bit allocation                  */
    const Word16 nb_sfm          /* i  : number of possibly coded bands  */
)
{
    Word16 sfm, core_sfm;

    core_sfm = sub(nb_sfm,1);

    FOR (sfm = nb_sfm-1; sfm >= 0; sfm--)
    {
        IF ( bitalloc[sfm] != 0 )
        {
            core_sfm = sfm;
            move16();
            BREAK;
        }
    }

    return core_sfm;
}

/*--------------------------------------------------------------------------*
 * apply_noisefill_HQ()
 *
 * Inject noise in non-coded bands
 *--------------------------------------------------------------------------*/

void apply_noisefill_HQ_fx(
    const Word16 *R,             /* i  : bit allocation                     Q0  */
    const Word16 length,         /* i  : input frame length                 Q0  */
    const Word16 flag_32K_env_ho,/* i  : envelope stability hangover flag   Q0  */
    const Word32 L_core_brate,   /* i  : core bit rate                      Q0  */
    const Word16 last_sfm,       /* i  : last coded subband                 Q0  */
    const Word16 *CodeBook,      /* i  : Noise-fill codebook                Q12 */
    const Word16 *CodeBook_mod,  /* i  : Densified noise-fill codebook      Q12 */
    const Word16 cb_size,        /* i  : Codebook length                    Q0  */
    const Word16 *sfm_start,     /* i  : Subband start coefficient          Q0  */
    const Word16 *sfm_end,       /* i  : Subband end coefficient            Q0  */
    const Word16 *sfmsize,       /* i  : Subband band width                 Q0  */
    Word16 *coeff          /* i/o: coded/noisefilled spectrum         Q12 */
)
{
    Word16 sfm;
    Word16 cb_pos;
    Word16 E_corr;
    Word16 cb_buff[PVQ_MAX_BAND_SIZE];
    Word16 i, j;
    Word16 istart;
    UWord16 lsb;
    Word32 L_E_cb_vec;
    Word32 L_E_corr;

    test();
    test();
    IF ( (sub(length, L_FRAME32k) >= 0) || (L_sub(L_core_brate, HQ_32k) > 0) || (L_sub(L_core_brate, HQ_24k40) < 0) )
    {
        /* Read from codebook */
        cb_pos = 0;
        move16();

        FOR (sfm = 0; sfm <= last_sfm; sfm++)
        {
            IF (R[sfm] == 0)
            {
                IF (sub(flag_32K_env_ho, 1) == 0)
                {
                    L_E_cb_vec = L_deposit_l(0);
                    IF (sub(sfm, 20) < 0)
                    {
                        FOR (i = 0; i < sfmsize[sfm]; i++)
                        {
                            cb_buff[i] = CodeBook_mod[cb_pos++];
                            move16();
                            L_E_cb_vec = L_mac0(L_E_cb_vec, cb_buff[i], cb_buff[i]); /*Q24 (12+12) */

                            if (sub(cb_pos, cb_size) >= 0)
                            {
                                cb_pos = 0;
                                move16();
                            }
                        }
                    }
                    ELSE
                    {
                        FOR (i = 0; i < sfmsize[sfm]; i++)
                        {
                            cb_buff[i] = CodeBook[cb_pos++];
                            move16();
                            L_E_cb_vec = L_mac0(L_E_cb_vec, cb_buff[i], cb_buff[i]); /*Q24 (12+12) */

                            if (sub(cb_pos, cb_size) >= 0)
                            {
                                cb_pos = 0;
                                move16();
                            }
                        }
                    }

                    /*E_corr = E_cb_vec / ((float) sfmsize[sfm]); */
                    Mpy_32_16_ss(L_E_cb_vec, inv_tbl_fx[sfmsize[sfm]], &L_E_corr, &lsb);        /*Q24 (24+15+1-16) */
                    move16();

                    /*E_corr = 1.0f / (float)sqrt(E_corr); */
                    L_E_corr = Isqrt(L_E_corr);                                     /*Q19 (31-24/2) */
                    E_corr = extract_h(L_shl(L_E_corr, 10));                        /*Q13 (13-(19-16)) */

                    istart = sfm_start[sfm];
                    move16();
                    FOR(j = istart; j  < sfm_end[sfm]; j++)
                    {
                        /*coeff[j] = cb_buff[j - istart] * E_corr; */
                        coeff[j] = extract_h(L_shl(L_mult(cb_buff[j - istart], E_corr), 2));    /*Q12 (12+13+1+2-16) */
                    }
                }
                ELSE
                {
                    FOR (j = sfm_start[sfm]; j < sfm_end[sfm]; j++)
                    {
                        coeff[j] = CodeBook[cb_pos++];
                        move16();
                        if (sub(cb_pos, cb_size) >= 0)
                        {
                            cb_pos = 0;
                            move16();
                        }
                    }
                }
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * harm_bwe_fine_fx()
 *
 * Prepare harmonic BWE fine structure
 *--------------------------------------------------------------------------*/

void harm_bwe_fine_fx(
    const Word16 *R,                 /* i  : bit allocation                              */
    const Word16 last_sfm,           /* i  : last coded subband                          */
    const Word16 high_sfm,           /* i  : higher transition band to BWE               */
    const Word16 num_sfm,            /* i  : total number of bands                       */
    const Word16 *norm,              /* i  : quantization indices for norms              */
    const Word16 *sfm_start,         /* i  : Subband start coefficient                   */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                     */
    Word16 *prev_L_swb_norm,   /* i/o: last normalize length                       */
    Word16 *coeff,             /* i/o: coded/noisefilled normalized spectrum       */
    Word32 *coeff_out          /* o  : coded/noisefilled spectrum                  */
    , Word16 *coeff_fine         /* o  : BWE fine structure                          */
)
{
    Word16 sfm;
    Word16 i;
    Word32 normq;
    Word16 SWB_signal[L_HARMONIC_EXC];
    Word32 envelope[L_HARMONIC_EXC], L_signal[L_HARMONIC_EXC];
    Word16 enve_lo[L_HARMONIC_EXC], enve_hi[L_HARMONIC_EXC];
    Word16 *src, *dst, *end;
    Word16 norm_signal;

    Word16 norm_width = 64;
    move16();

    /* shape the spectrum */
    FOR (sfm = 0; sfm <= last_sfm; sfm++)
    {
        IF( R[sfm] != 0 )
        {
            normq = L_add(0,dicn_fx[norm[sfm]]);

            FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            {
                coeff_out[i] = L_shl(Mult_32_16(normq, coeff[i]),1); /*12 14+12+1+1-16  */
                move32();
            }
        }
        ELSE
        {
            FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            {
                coeff_out[i] = L_deposit_l(0);
            }
        }
    }

    /* excitation replication */
    Copy32(coeff_out, L_signal, L_HARMONIC_EXC);
    calc_normal_length_fx_32( HQ_CORE, coeff_out, HQ_HARMONIC, -1, &norm_width, prev_L_swb_norm );
    hq_swb_harmonic_calc_norm_envelop_fx( L_signal, envelope, norm_width, L_HARMONIC_EXC );

    /* Normalize with envelope */
    FOR (i = 0; i < L_HARMONIC_EXC; i++)
    {
        IF (L_signal[i] > 0)
        {
            norm_signal = norm_l(envelope[i]);
            enve_lo[i] = L_Extract_lc(L_shl(envelope[i], norm_signal), &enve_hi[i]);
            L_signal[i] = Div_32(L_signal[i], enve_hi[i], enve_lo[i]);
            SWB_signal[i] = round_fx(L_shl(L_signal[i], norm_signal));
            move16();
            move16();
            move32();

        }
        ELSE
        {
            norm_signal = norm_l(envelope[i]);
            enve_lo[i] = L_Extract_lc(L_shl(envelope[i], norm_signal), &enve_hi[i]);
            L_signal[i] = L_negate(Div_32(L_negate(L_signal[i]), enve_hi[i], enve_lo[i]));
            SWB_signal[i] = round_fx(L_shl(L_signal[i], norm_signal));
            move16();
            move16();
            move32();

        }
    }

    dst = coeff_fine + sfm_end[last_sfm];
    end = coeff_fine + sfm_end[num_sfm-1];

    IF (sub(sfm_end[last_sfm], sfm_end[high_sfm]) <= L_HARMONIC_EXC - START_EXC )
    {
        src = SWB_signal + START_EXC + sub(sfm_end[last_sfm], sfm_end[high_sfm]);
    }
    ELSE
    {
        src = SWB_signal + L_HARMONIC_EXC - 1;
    }

    WHILE (dst < end)
    {
        logic32();
        WHILE (dst < end && src < &SWB_signal[L_HARMONIC_EXC])
        {
            *dst++ = *src++;
            move16();
        }
        src --;

        logic32();
        WHILE (dst < end && src >= &SWB_signal[START_EXC])
        {
            *dst++ = *src--;
            move16();
        }
        src++;
    }

    return;
}

/*--------------------------------------------------------------------------*
 * hvq_bwe_fine()
 *
 * Prepare HVQ BWE fine structure
 *--------------------------------------------------------------------------*/

void hvq_bwe_fine_fx(
    const Word16 last_sfm,           /* i  : last coded subband                        Q0  */
    const Word16 num_sfm,            /* i  : total number of bands                     Q0  */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                   Q0  */
    const Word16 *peak_idx,          /* i  : Peak index                                Q0  */
    const Word16 Npeaks,             /* i  : Number of peaks                           Q0  */
    Word16 *peak_pos,          /* o  : Peak positions                            Q0  */
    Word16 *prev_L_swb_norm,   /* i/o: last normalize length                     Q0  */
    Word32 *L_coeff,           /* i  : coded/noisefilled normalized spectrum     Q12 */
    Word16 *bwe_peaks,         /* o  : Positions of peaks in BWE                 Q0  */
    Word16 *coeff_fine         /* o  : HVQ BWE fine structure                    Q15 */
)
{
    Word16 i, j;
    Word16 SWB_signal[L_HARMONIC_EXC];
    Word32 L_envelope[L_HARMONIC_EXC];
    Word16 *src, *dst, *end;
    Word16 *peak_dst, *peak_src;
    Word16 norm_width = 64;
    Word16 tmp;
    Word16 shift, shift2;
    Word32 L_tmp;
    UWord16 lsb;

    calc_normal_length_fx_32( HQ_CORE, L_coeff, HQ_HVQ, -1, &norm_width, prev_L_swb_norm );

    hq_swb_harmonic_calc_norm_envelop_fx(L_coeff, L_envelope, norm_width, L_HARMONIC_EXC);

    /* Normalize with envelope */
    FOR (i = 0; i < L_HARMONIC_EXC; i++)
    {
        /*SWB_signal[i] = SWB_signal[i] / envelope[i]; */

        shift = norm_l(L_envelope[i]);
        tmp = round_fx(L_shl(L_envelope[i], shift));                /* 12+s-16=Q(-4+s) */

        /* Avoid division by zero */
        if (tmp == 0)
        {
            tmp = 1<<14;
            move16();
        }

        tmp = div_s(1<<14, tmp);                                    /* 15+14-(-4+s)=Q(33-s) */
        Mpy_32_16_ss(L_coeff[i], tmp, &L_tmp, &lsb);                  /* 12+33-s+1-16=Q(30-s) */
        shift2 = add(shift, 1);
        tmp = round_fx(L_shl(L_tmp, shift2));                       /* 30-s+s+1-16=Q(15) */
        SWB_signal[i] = add(tmp, lshr(lsb, sub(32, shift2)));
        move16();    /* Q15 */
        /*SWB_signal[i] = round_fx(L_shl(L_tmp, add(shift, 1)));      // 30-s+s+1-16=Q(15) */

    }

    dst = coeff_fine;
    end = coeff_fine + sfm_end[num_sfm-1] - sfm_end[last_sfm];

    src = SWB_signal + START_EXC;
    peak_src = peak_pos + START_EXC;

    FOR (i = 0; i < Npeaks; i++)
    {
        if ( sub(peak_idx[i], L_HARMONIC_EXC) < 0 )
        {
            peak_pos[peak_idx[i]] = 1;
            move16();
        }
    }

    i = sub(L_HARMONIC_EXC, 1);
    WHILE ( i-- > 0 )
    {
        IF ( sub(peak_pos[i], 1) == 0 )
        {
            BREAK;
        }
    }

    if ( sub(i, 180) < 0 )
    {
        i = 180;
        move16();
    }

    FOR ( j = L_HARMONIC_EXC-1; j > i+1; j-- )
    {
        SWB_signal[j] = 0;
        move16();
    }

    peak_dst = bwe_peaks + sfm_end[last_sfm];
    WHILE ( dst < end )
    {
        test();
        WHILE ( dst < end && src < &SWB_signal[L_HARMONIC_EXC] )
        {
            *dst++ = *src++;
            move16();
            *peak_dst++ = *peak_src++;
            move16();
        }
        peak_src--;
        src --;

        test();
        WHILE ( dst < end && src  >= &SWB_signal[START_EXC] )
        {
            *dst++ = *src--;
            move16();
            *peak_dst++ = *peak_src--;
            move16();
        }
        peak_src++;
        src++;
    }

    return;
}

/*--------------------------------------------------------------------------*
 * hq_fold_bwe_fx()
 *
 * HQ mode folding BWE
 *--------------------------------------------------------------------------*/

void hq_fold_bwe_fx(
    const Word16 last_sfm,           /* i  : last coded subband                     Q0 */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                Q0 */
    const Word16 num_sfm,            /* i  : Number of subbands                     Q0 */
    Word16 *coeff                    /* i/o: coded/noisefilled normalized spectrum  Q12 */
)
{
    Word16 low_coeff;
    Word16 first_coeff;
    Word16 *src, *dst, *end;

    low_coeff = shr(sfm_end[last_sfm], 1);
    src       = coeff + sfm_end[last_sfm] - 1;

    first_coeff = sfm_end[last_sfm];
    dst = coeff + sfm_end[last_sfm];
    end = coeff + sfm_end[num_sfm-1];

    WHILE(dst < end)
    {
        WHILE(dst < end && src >= &coeff[low_coeff])
        {
            *dst++ = *src--;
            move16();
        }

        src++;

        WHILE(dst < end && src < &coeff[first_coeff])
        {
            *dst++ = *src++;
            move16();
        }
    }
    return;
}

/*--------------------------------------------------------------------------*
 * apply_nf_gain()
 *
 * Apply noise fill gain
 *--------------------------------------------------------------------------*/

void apply_nf_gain_fx(
    const Word16 nf_idx,             /* i  : noise fill gain index                   Q0 */
    const Word16 last_sfm,           /* i  : last coded subband                      Q0 */
    const Word16 *R,                 /* i  : bit allocation                          Q0 */
    const Word16 *sfm_start,         /* i  : Subband start coefficient               Q0 */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                 Q0 */
    Word16 *coeff              /* i/o: coded/noisefilled normalized spectrum   Q12 */
)
{
    Word16 sfm;
    Word16 j;

    FOR (sfm = 0; sfm <= last_sfm; sfm++)
    {
        IF (R[sfm] == 0)
        {
            FOR (j = sfm_start[sfm]; j < sfm_end[sfm]; j++)
            {
                /* Scale NoiseFill */
                coeff[j] = shr(coeff[j], nf_idx);
                move16();
            }
        }
    }

    return;
}


/*--------------------------------------------------------------------------*
 * harm_bwe_fx()
 *
 * HQ Harmonic BWE
 *--------------------------------------------------------------------------*/

void harm_bwe_fx(
    const Word16 *coeff_fine,        /* i  : fine structure for BWE                  */
    const Word16 *coeff,             /* i  : coded/noisefilled normalized spectrum   */
    const Word16 num_sfm,            /* i  : Number of subbands                      */
    const Word16 *sfm_start,         /* i  : Subband start coefficient               */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                 */
    const Word16 last_sfm,           /* i  : last coded subband                      */
    const Word16 *R,                 /* i  : bit allocation                          */
    const Word16 prev_hq_mode,       /* i  : previous hq mode                        */
    Word16 *norm,              /* i/o: quantization indices for norms          */
    Word16 *noise_level,       /* i/o: noise levels for harmonic modes         */
    Word16 *prev_noise_level,  /* i/o: noise factor in previous frame          */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input    */
    Word32 *coeff_out          /* o  : coded/noisefilled spectrum              */
)
{
    Word16 i, j;
    Word16 sfm, band_width;
    Word32 normq, L_tmp,L_tmp2;
    Word32 E_L;
    Word16 alfa = 16384;
    Word16 tmp, tmp1, exp1;
    Word16 beta;
    Word32 *src, *dst;

    move16();   /* alfa */

    FOR (sfm = 0; sfm <= last_sfm; sfm++)
    {
        IF (R[sfm] == 0)
        {
            normq = dicn_fx[norm[sfm]];  /*Q14  */                      move16();

            FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            {
                coeff_out[i] = L_shl(Mult_32_16(normq, coeff[i]),1);
                move32();  /*12 Q(14 +12+1-16 +1) */

            }
        }
    }
    noise_level[1] = noise_level[0];
    move16();

    /* shaping the BWE spectrum further by envelopes and noise factors */
    L_tmp = L_mult(29491, prev_noise_level[0]); /*  15 +1 +15 */
    noise_level[0] = round_fx(L_mac(L_tmp, 3277, noise_level[0]));      /*15  */

    L_tmp = L_mult(29491, prev_noise_level[1]);
    noise_level[1] = round_fx(L_mac(L_tmp, 3277, noise_level[1]));

    test();
    IF (prev_hq_mode == HQ_NORMAL || sub(prev_hq_mode, HQ_GEN_SWB) == 0)
    {
        IF (sub(noise_level[0], 8192) < 0)
        {
            noise_level[0] = shl(noise_level[0], 2);
            move16();
        }

        IF (sub(noise_level[1], 8192) < 0)
        {
            noise_level[1] = shl(noise_level[1], 2);
            move16();
        }
    }

    FOR (i = add(last_sfm, 1); i < num_sfm; i++)
    {
        E_L = L_add(0,1);
        FOR (j = sfm_start[i]; j < sfm_end[i]; j++)
        {
            L_tmp =L_mult0(coeff_fine[j], coeff_fine[j]);/*Q30  */
            E_L =L_add(E_L,L_shr(L_tmp,6));/*Q24 */
        }

        normq = L_add(0,dicn_fx[norm[i]]);

        alfa = noise_level[0];
        move16();
        if (sub(i, 27) > 0)
        {
            alfa = noise_level[1];
            move16();
        }

        band_width = sub(sfm_end[i], sfm_start[i]); /* */
        exp1 =norm_l(E_L);
        IF(exp1 ==0)
        {
            E_L = Mult_32_16(E_L, inv_tbl_fx[band_width]); /* Q24 (24+15+1-16) *//*24+15+1-16  */
            tmp = div_l(E_L,sub(32767,alfa));  /*Q24-15-1 =8 */
            tmp = max(1,tmp);
            L_tmp =L_deposit_h(tmp); /*24 */
            E_L = Isqrt(L_tmp); /* Q19 (31-24/2) */
        }
        ELSE
        {
            exp1 =sub(exp1,1);
            E_L = Mult_32_16(L_shl(E_L,exp1), inv_tbl_fx[band_width]); /* Q24+exp1 (24+exp1+15+1-16) */
            tmp = div_l(E_L,sub(32767,alfa));  /*Q24+exp1-15-1 =8+exp1 */
            tmp = max(1,tmp);
            L_tmp =L_shl(L_deposit_l(tmp),sub(16,exp1)); /*24  8+exp1+16-exp1 */
            E_L = Isqrt(L_tmp); /* Q19 (31-24/2) */
        }

        exp1 = norm_s(alfa);
        tmp1 = shl(alfa, exp1);
        exp1 = add(1, exp1);
        tmp1 = s_max(tmp1, 16384);
        tmp1 = div_s(16384, tmp1);
        L_tmp2 = L_deposit_h(tmp1);
        L_tmp2 = Isqrt_lc(L_tmp2, &exp1);
        beta = round_fx(L_shl(L_tmp2, exp1));
        beta =shr(beta,1); /*Q15 */


        FOR(sfm = sfm_start[i]; sfm < sfm_end[i]; sfm++)
        {
            L_tmp = Mult_32_16(E_L,coeff_fine[sfm]); /*Q19 19 + 15 +1-16   */
            L_tmp = L_shl(L_tmp,9);/*Q28 */
            tmp = Random(bwe_seed);  /*Q15 */
            L_tmp2 =L_shr(L_mult(beta,tmp),3);/* Q28 31-3  15+15 +1-3 */
            L_tmp =L_add(L_tmp,L_tmp2);/*Q28 */
            coeff_out[sfm] = L_shl(Mult_32_32(L_tmp, normq),1);/*Q12  28 +14 +1 -31 */  move32();
        }
    }

    prev_noise_level[0] = noise_level[0];
    move16();
    prev_noise_level[1] = noise_level[1];
    move16();

    src = &coeff_out[add(sfm_end[last_sfm], L_HARMONIC_EXC - START_EXC)];  /*Q12 */

    dst = src-1;
    FOR( i = 0; i < 16; i++ )
    {
        *src = Mult_32_16(*src, hvq_bwe_fac_fx[i]);   /* Q12 (12+15+1-16) */ move32();
        src++;
        *dst = Mult_32_16(*dst, hvq_bwe_fac_fx[i]);   /* Q12 (12+15+1-16) */ move32();
        dst--;
    }
    IF(sub(num_sfm, 33) == 0)
    {
        set32_fx(&coeff_out[800], 0, 160);
    }
    return;
}

/*--------------------------------------------------------------------------*
 * HVQ_bwe_fx()
 *
 * HQ HVQ BWE
 *--------------------------------------------------------------------------*/

void hvq_bwe_fx(
    const Word32 *L_coeff,           /* i  : coded/noisefilled normalized spectrum  Q12 */
    const Word16 *coeff_fine,        /* i  : coded/noisefilled normalized spectrum  Qin */
    const Word16 *sfm_start,         /* i  : Subband start coefficient              Q0  */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                Q0  */
    const Word16 *sfm_len,           /* i  : Subband length                         Q0  */
    const Word16 last_sfm,           /* i  : last coded subband                     Q0  */
    const Word16 prev_hq_mode,       /* i  : previous hq mode                       Q0  */
    const Word16 *bwe_peaks,         /* i  : HVQ bwe peaks                          Q0  */
    const Word16 bin_th,             /* i  : HVQ transition bin                     Q0  */
    const Word16 num_sfm,            /* i  : Number of bands                        Q0  */
    const Word32 core_brate,         /* i  : Core bit-rate                          Q0  */
    const Word16 *R,                 /* i  : Bit allocation                             */
    Word16 *norm,              /* i/o: quantization indices for norms         Q0  */
    Word16 *noise_level,       /* i/o: noise levels for harmonic modes        Q15 */
    Word16 *prev_noise_level,  /* i/o: noise factor in previous frame         Q15 */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input   Q0  */
    Word32 *L_coeff_out,       /* o  : coded/noisefilled spectrum             Qout*/
    const Word16 qin,
    const Word16 qout
)
{
    Word16 i, j;
    Word16 N;
    Word32 L_normq;
    Word32 L_E;
    Word32 L_tmp_norm = 0;
    Word16 bwe_noise_th = 0;
    Word16 peak_band, low, high, sel_norm;
    Word16 norm_ind;
    Word32 *L_src, *L_dst;
    Word16 istart, iend;
    Word16 offset = sfm_end[last_sfm];

    /* Fx specific variables */
    Word32 L_tmp0, L_tmp1;
    Word16 tmp, tmp2, band_size;
    Word16 last_norm_ind;
    Word16 shift, power_shift;
    Word16 coeff_s[L_FRAME48k];
    Word16 j_N;
    Word16 n_c;
    UWord16 lsb;

    move32();   /* L_tmp_norm */
    move16();   /* bwe_noise_th */

    bwe_noise_th = add(bin_th, shr(sub(sfm_end[sub(num_sfm,1)], bin_th),1));
    logqnorm_fx(&L_coeff_out[sfm_start[last_sfm]], qout, &norm[last_sfm], 40, sfm_len[last_sfm], 0);
    move16();

    /* shaping the BWE spectrum further by envelopes and noise factors */
    noise_level[0] = round_fx(L_mac(L_mult(29491,prev_noise_level[0]),3277,noise_level[0])); /* Q15 (15+15+1-16) */
    noise_level[1] = round_fx(L_mac(L_mult(29491,prev_noise_level[1]),3277,noise_level[1])); /* Q15 (15+15+1-16) */

    test();
    IF (prev_hq_mode == HQ_NORMAL || sub(prev_hq_mode, HQ_GEN_SWB) == 0)
    {
        IF( sub(noise_level[0], 8192 /* 0.25f */) < 0 )
        {
            noise_level[0] = shl(noise_level[0], 2);
            move16();
        }

        IF( sub(noise_level[1], 8192 /* 0.25f */) < 0 )
        {
            noise_level[1] = shl(noise_level[1], 2);
            move16();
        }
    }

    norm_ind = add(last_sfm, 1);
    IF ( L_sub(core_brate, HQ_24k40) == 0 )
    {
        peak_band = 0;
        move16();

        tmp = 1;
        move16();
        FOR (i = sfm_start[norm_ind]; i < sfm_end[norm_ind+1]; i++)
        {
            tmp2 = abs_s(coeff_fine[i-offset]);
            tmp = s_max(tmp, tmp2);
        }
        band_size = sub(sfm_end[norm_ind+1], sfm_start[norm_ind]);

        /* Headroom for square and accumulate */
        shift = sub(norm_s(tmp), sqac_headroom_fx[band_size]);
        L_E = L_deposit_l(1);
        FOR (i = sfm_start[norm_ind]; i < sfm_end[norm_ind+1]; i++)
        {
            if (bwe_peaks[i])
            {
                peak_band = 1;
                move16();
            }
            /* E_L += coeff_fine[i-offset] * coeff_fine[i-offset]; */
            coeff_s[i] = shl(coeff_fine[i-offset], shift);
            move16();/* Q15 + shift */
            L_E = L_mac0(L_E, coeff_s[i], coeff_s[i]); /* Q2*(qin + shift) */
        }
        power_shift = shl(shift, 1);

        L_E = L_shr(L_E,sub(power_shift,28-2*qin));  /* Q28 */

        /* E_L = (float)sqrt((sfm_end[norm_ind+1] - sfm_start[norm_ind])/E_L); */
        L_E = Mult_32_16(L_E, inv_tbl_fx[band_size]); /* Q28 (28+15+1-16) */
        /* To avoid overflow in Isqrt() */
        if( L_E == 0 )
        {
            L_E = L_deposit_l(1);
        }
        L_E = Isqrt(L_E); /* Q17 (31-28/2) */

        /* normq = 0.1f*dicn[norm[norm_ind-1]] + 0.8f*dicn[norm[norm_ind]] + 0.1f*dicn[norm[norm_ind+1]]; */
        /* tmp_norm = 0.1f*dicn[norm[norm_ind]] + 0.8f*dicn[norm[norm_ind+1]] + 0.1f*dicn[norm[norm_ind+2]]; */
        L_tmp0 = L_add(dicn_fx[norm[norm_ind]], 0);
        L_tmp1 = L_add(dicn_fx[norm[norm_ind+1]], 0);
        L_normq = Madd_32_16(Madd_32_16(Mult_32_16(L_tmp0, 26214 /* Q15, 0.8f */), dicn_fx[norm[norm_ind-1]], 3277 /* Q15, 0.1f */), L_tmp1, 3277 /* Q15, 0.1f */); /* Q14 (14+15+1-16) */ move16();
        L_tmp_norm = Madd_32_16(Madd_32_16(Mult_32_16(L_tmp1, 26214 /* Q15, 0.8f */), dicn_fx[norm[norm_ind+2]], 3277 /* Q15, 0.1f */), L_tmp0, 3277 /* Q15, 0.1f */); /* Q14 (14+15+1-16) */ move16();

        istart = sfm_start[norm_ind];
        move16();
        /* iend = istart + sfm_len[norm_ind]/2; */
        iend = 240;
        move16();

        noise_mix_fx( &coeff_fine[-offset], L_E, L_normq, bwe_seed, istart, iend, noise_level[0], L_coeff_out, qin, qout);

        j = 0;
        move16();
        /* N = sfm_len[norm_ind]/2+sfm_len[norm_ind+1]/2-1; */
        N = 31;
        move16();
        j_N = N;
        move16();

        istart = iend;
        move16();
        /* iend = sfm_start[norm_ind+1] + sfm_len[norm_ind+1]/2; */
        iend = 272;
        move16();

        /* special case that is not handled by noise_mix_fx() */
        n_c = sub(MAX_16,noise_level[0]); /* Q15 */
        FOR (i = istart; i < iend; i++)
        {
            /* coeff_out[i] = ((float)(N-j)/N*normq + (float)j/N*tmp_norm)*((1.0f - noise_level[i/bwe_noise_th])*coeff_fine[i-offset]*E_L + noise_level[i/bwe_noise_th]*own_random(bwe_seed)/32768.0f); */
            L_tmp1 = Madd_32_16(Mult_32_16(L_normq, inv_N_fx[j_N] ), L_tmp_norm, inv_N_fx[j]); /* Q14 (14+15+1-16) */
            j = add(j,1);
            j_N = sub(j_N,1);

            Mpy_32_16_ss(L_E, coeff_fine[i-offset], &L_tmp0, &lsb); /* Qin+2 (17+qin+1-16) */
            Mpy_32_16_ss(L_tmp0, n_c, &L_tmp0, &lsb); /* Qin+2-15 (qin+2+15+1-16) */

            IF(L_tmp0 != 0)
            {
                /* Normalize with 1 bit headroom for addition */
                tmp = 30-(qin+2); /* Assuming fixed Q values */
                tmp = s_min(norm_l(L_tmp0), tmp);
                tmp = sub(tmp,1);

                L_tmp0 = L_add(L_shl(L_tmp0,tmp), L_deposit_l(lshr(lsb, sub(16,tmp)))); /* Qin+2+tmp */

                L_tmp0 = L_add(L_tmp0, L_shr(L_mult0(noise_level[0], Random(bwe_seed)), sub(30-(qin+2), tmp))); /* Qin+2+tmp */
                tmp = round_fx(L_shl(L_tmp0,27-(qin+2)-tmp)); /* Q11 (Qin+2+tmp+27-qin-2-tmp-16) */

                Mpy_32_16_ss(L_tmp1, tmp, &L_tmp0, &lsb); /* Q10 (14+11+1-16) */
                L_coeff_out[i] = L_add(L_shl(L_tmp0,qout-10), L_deposit_l(lshr(lsb, 10+16-qout)));
                move32(); /* Qout (10+qout-10) */
            }
            ELSE
            {
                L_tmp0 = L_mult0(noise_level[0], Random(bwe_seed)); /* Q30 (15+15) */
                tmp = round_fx(L_tmp0); /* Q14 (30-16) */
                Mpy_32_16_ss(L_tmp1, tmp, &L_tmp0, &lsb); /* Q13 (14+14+1-16) */
                L_coeff_out[i] = L_shr(L_tmp0,13-qout);
                move32();/* Qout (13-(13-qout)) */
            }
        }

        istart = iend;
        move16();
        iend = sfm_end[norm_ind+1];
        move16();

        noise_mix_fx( &coeff_fine[-offset], L_E, L_tmp_norm, bwe_seed, istart, iend, noise_level[0], L_coeff_out, qin, qout);

        norm_ind = add(norm_ind, 2);
    }

    FOR ( ; norm_ind < num_sfm; norm_ind++)
    {
        IF ( R[norm_ind] == 0 )
        {
            peak_band = 0;
            move16();

            FOR (i = sfm_start[norm_ind]; i < sfm_end[norm_ind]; i++)
            {
                if (bwe_peaks[i])
                {
                    peak_band = 1;
                    move16();
                }
            }

            istart = sfm_start[norm_ind];
            move16();
            iend = sfm_end[norm_ind];
            move16();

            last_norm_ind = sub(num_sfm, 1);
            test();
            test();
            IF ( sub(peak_band, 1) == 0 && sub(norm_ind,add(last_sfm, 1)) > 0 && sub(norm_ind, last_norm_ind) < 0 )
            {
                istart = sub(istart, shr(sfm_len[norm_ind-1],1));
                iend = add(iend, shr(sfm_len[norm_ind+1],1));
            }

            tmp = 1;
            move16();
            FOR (i = istart; i < iend; i++)
            {
                tmp2 = abs_s(coeff_fine[i-offset]);
                tmp = s_max(tmp, tmp2);
            }
            band_size = sub(iend, istart);
            /* Headroom for square and accumulate */
            shift = sub(norm_s(tmp), sqac_headroom_fx[band_size]);

            L_E = L_add(0,1L);
            FOR (i = istart; i < iend; i++)
            {
                /* E_L += coeff_fine[i-offset] * coeff_fine[i-offset]; */
                coeff_s[i] = shl(coeff_fine[i-offset], shift);
                move16();/* Q15 + shift */
                L_E = L_mac0(L_E, coeff_s[i], coeff_s[i]); /* Q2*(15 + shift) */
            }
            power_shift = shl(shift, 1);

            L_E = L_shr(L_E,sub(power_shift,28-2*qin));  /* Q28 */

            /* E_L = (float)sqrt((iend - istart)/E_L); */
            L_E = Mult_32_16(L_E, inv_tbl_fx[band_size]); /* Q28 (28+15+1-16) */
            /* To avoid overflow in Isqrt() */
            if( L_E == 0 )
            {
                L_E = L_add(1L,0);
            }
            L_E = Isqrt(L_E); /* Q17 (31-28/2) */

            IF ( peak_band )
            {
                IF ( sub(add(norm_ind,2), num_sfm) > 0 )
                {
                    /* normq = 0.15f*dicn[norm[norm_ind-1]] + 0.85f*dicn[norm[norm_ind]]; */
                    L_normq = Madd_32_16(Mult_32_16(dicn_fx[norm[norm_ind]], 27853 /* Q15, 0.85f */), dicn_fx[norm[norm_ind-1]], 4915 /* Q15, 0.1f */); /* Q14 (14+15+1-16) */ move16();
                    move16();
                }
                ELSE
                {
                    /* normq = 0.1f*dicn[norm[norm_ind-1]] + 0.8f*dicn[norm[norm_ind]] + 0.1f*dicn[norm[norm_ind+1]]; */
                    L_normq = Madd_32_16(Madd_32_16(Mult_32_16(dicn_fx[norm[norm_ind]], 26214 /* Q15, 0.8f */), dicn_fx[norm[norm_ind+1]], 3277 /* Q15, 0.1f */), dicn_fx[norm[norm_ind-1]], 3277 /* Q15, 0.1f */); /* Q14 (14+15+1-16) */ move16();
                    move16();
                    move16();
                }
            }
            ELSE
            {
                low = norm_ind;
                move16();
                high = s_min(add(norm_ind,1), last_norm_ind);
                move16();
                sel_norm = norm[norm_ind-1];
                move16();
                FOR (j = low; j <= high; j++)
                {
                    if ( sub(norm[j], sel_norm) > 0 )
                    {
                        sel_norm = norm[j];
                        move16();
                    }
                }
                L_normq = dicn_fx[sel_norm];
                move16();
            }

            iend = s_min(sfm_end[norm_ind], bwe_noise_th);
            move16();
            IF( sub(iend, sfm_start[norm_ind]) > 0 )
            {
                noise_mix_fx( &coeff_fine[-offset], L_E, L_normq, bwe_seed, sfm_start[norm_ind], iend, noise_level[0], L_coeff_out, qin, qout);
            }
            ELSE
            {
                iend = sfm_end[norm_ind];
                move16();
                noise_mix_fx( &coeff_fine[-offset], L_E, L_normq, bwe_seed, sfm_start[norm_ind], iend, noise_level[1], L_coeff_out, qin, qout);
            }
            /* Noisemix up to threshold done */
            IF( sub(iend, bwe_noise_th) == 0 )
            {
                noise_mix_fx( &coeff_fine[-offset], L_E, L_normq, bwe_seed, iend, sfm_end[norm_ind], noise_level[1], L_coeff_out, qin, qout);
            }

        }
        ELSE /* R[norm_ind] > 0 */
        {
            FOR (i = sfm_start[norm_ind]; i < sfm_end[norm_ind]; i++)
            {
                L_coeff_out[i] = L_coeff[i]; /* Scaling already applied */      move32();
            }
        }
    }

    prev_noise_level[0] = noise_level[0];
    move16();
    prev_noise_level[1] = noise_level[1];
    move16();
    L_src = &L_coeff_out[add(sfm_end[last_sfm], L_HARMONIC_EXC - START_EXC)]; /* Address initialization */
    L_dst = L_src - 1 ;   /* Address computation */

    FOR( i = 0; i < 16; i++ )
    {
        *L_src = Mult_32_16(*L_src, hvq_bwe_fac_fx[i]);
        L_src++;  /* Qout (Qout+15+1-16) */ move32();
        *L_dst = Mult_32_16(*L_dst, hvq_bwe_fac_fx[i]);
        L_dst--;  /* Qout (Qout+15+1-16) */ move32();
    }

    return;
}

/*-------------------------------------------------------------------*
* hvq_concat_bands_fx()
*
* Compute the band limits for concatenated bands for PVQ target signal in HVQ
*--------------------------------------------------------------------------*/
void hvq_concat_bands_fx
(
    const Word16 pvq_bands,          /* i  : Number of bands in concatenated PVQ target  */
    const Word16 *sel_bnds,          /* i  : Array of selected high bands                */
    const Word16 n_sel_bnds,         /* i  : Number of selected high bands               */
    Word16 *hvq_band_start,    /* i  : Band start indices                          */
    Word16 *hvq_band_width,    /* i  : Band widths                                 */
    Word16 *hvq_band_end       /* i  : Band end indices                            */
)
{
    Word16 k, k_1;
    const Word16 *pSelBnds;

    pSelBnds = sel_bnds;
    FOR (k = 0; k < pvq_bands; k++)
    {

        IF( sub(k, sub(pvq_bands, n_sel_bnds)) >= 0)
        {
            k_1 = sub(k, 1);
            hvq_band_start[k] = hvq_band_end[k_1];
            move16();
            hvq_band_width[k] = band_len_harm[*pSelBnds++];
            move16();
            move16();
            hvq_band_end[k]   = add(hvq_band_end[k_1], hvq_band_width[k]);
            move16();
        }
        ELSE
        {
            hvq_band_start[k] = extract_l(L_mult0(k, HVQ_PVQ_COEFS));
            hvq_band_width[k] = HVQ_PVQ_COEFS;
            move16();
            hvq_band_end[k]   = add(hvq_band_start[k], HVQ_PVQ_COEFS);
            move16();
        }
    }

    return;
}
/*--------------------------------------------------------------------------*
 * noise_mix_fx()
 *--------------------------------------------------------------------------*/

void noise_mix_fx(
    const Word16 *coeff_fine,        /* i  : normalized fine structure spectrum     Qin */
    const Word32 L_E,                /* i  : normalization factor                   Q17 */
    const Word32 L_normq,            /* i  : quantized norm                         Q14 */
    Word16 *seed,              /* i/o: random seed                            Q0  */
    const Word16 istart,             /* i  : start coefficient                      Q0  */
    const Word16 iend,               /* i  : end coefficient                        Q0  */
    const Word16 noise_level,        /* i  : noise_level                            Q0  */
    Word32 *L_coeff_out,       /* o  : noise mixed spectrum                   Qout */
    const Word16 qin,
    const Word16 qout
)
{
    Word16 i, tmp, n_c;
    Word32 L_tmp0;
    UWord16 lsb;

    n_c = sub(MAX_16,noise_level); /* Q15 */
    FOR (i = istart; i < iend; i++)
    {
        /* L_coeff_out[i] = ((1.0f - noise_level)*coeff_fine[i]*L_E + noise_level*own_random(seed)/32768.0f)*normq; */
        Mpy_32_16_ss(L_E, coeff_fine[i], &L_tmp0, &lsb); /* Qin+qL_E-15 (qL_E+qin+1-16) */
        Mpy_32_16_ss(L_tmp0, n_c, &L_tmp0, &lsb);   /* Qin+qL_E-15 (qin+qL_E-15+15+1-16) */

        IF(L_tmp0 != 0)
        {
            /* Normalize with 1 bit headroom for addition */
            tmp = 30-(qin+2); /* Assuming fixed Q values */
            tmp = s_min(norm_l(L_tmp0), tmp);
            tmp = sub(tmp,1);

            L_tmp0 = L_add(L_shl(L_tmp0,tmp), L_deposit_l(lshr(lsb, sub(16,tmp)))); /* Qin+2+tmp */
            L_tmp0 = L_add(L_tmp0, L_shr(L_mult0(noise_level, Random(seed)), sub(30-(qin+2), tmp))); /* Qin+2+tmp */

            tmp = round_fx(L_shl(L_tmp0,27-(qin+2)-tmp)); /* Q11 (Qin+2+tmp+27-qin-2-tmp-16) */
            Mpy_32_16_ss(L_normq, tmp, &L_tmp0, &lsb); /* Q10 (14+11+1-16) */
            L_coeff_out[i] = L_add(L_shl(L_tmp0,qout-10), L_deposit_l(lshr(lsb, 10+16-qout)));
            move32();/* Qout (10+qout-10) */
        }
        ELSE
        {
            L_tmp0 = L_mult0(noise_level, Random(seed)); /* Q30 (15+15) */
            tmp = round_fx(L_tmp0); /* Q14 (30-16) */
            Mpy_32_16_ss(L_normq, tmp, &L_tmp0, &lsb); /* Q13 (14+14+1-16) */
            L_coeff_out[i] = L_shr(L_tmp0,13-qout);
            move32();/* Qout (13-(13-qout)) */
        }
    }
}


/*--------------------------------------------------------------------------*
 * hq_generic_fine_fx()
 *
 * Prepare HQ GENERIC HF fine structure
 *--------------------------------------------------------------------------*/
void hq_generic_fine_fx(
    Word16 *coeff,             /* i  : coded/noisefilled normalized spectrum   */
    const Word16 last_sfm,           /* i  : Last coded band                         */
    const Word16 *sfm_start,         /* i  : Subband start coefficient               */
    const Word16 *sfm_end,           /* i  : Subband end coefficient                 */
    Word16 *bwe_seed,          /* i/o: random seed for generating BWE input    */
    Word16 *coeff_out1         /* o  : HQ GENERIC input                        */
)
{
    Word16 sfm;
    Word16 i;

    FOR (sfm = 0; sfm <= last_sfm; sfm++)
    {
        FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
        {
            IF (coeff[i]==0)
            {
                coeff_out1[i] = -2048;
                move16();
                if (Random(bwe_seed) > 0)
                {
                    coeff_out1[i] = 2048;
                    move16();
                }
            }
            ELSE
            {
                coeff_out1[i] = coeff[i];
                move16();
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * overlap_hq_bwe_fx()
 *
 * Overlapping at the boundary between HQ core and BWE
 *--------------------------------------------------------------------------*/
static void overlap_hq_bwe_fx(
    const Word32 *hq_swb_overlap_buf,        /* i  : spectrum from HQ core   */
    Word32 *coeff_out,                 /* i/o: spectrum from BWE, overlapped output  */
    const Word16 n_swb_overlap_offset,       /* i  : starting offset of overlapping  */
    const Word16 n_swb_overlap,              /* i  : length of overlapping  */
    const Word16 *R,
    const Word16 num_env_bands,
    const Word16 num_sfm,
    const Word16 *sfm_end
)
{
    Word16 i;
    Word16 weighting;
    Word16 step;
    Word16 exp, tmp, n_band;

    IF ( R[sub(num_env_bands,1)] != 0)
    {
        Copy32( hq_swb_overlap_buf, &coeff_out[n_swb_overlap_offset], n_swb_overlap );
    }
    ELSE
    {
        exp = norm_s(n_swb_overlap);
        tmp = div_s(16384, shl(n_swb_overlap, exp));/*15 + 14 - exp */
        tmp = shr(tmp, sub(14, exp));/*15 */
        step = mult_r(tmp, 32767);/*15 */
        weighting = 32767;
        move16();
        FOR (i = 0; i < n_swb_overlap; i++)
        {
            coeff_out[add(n_swb_overlap_offset,i)] = L_add(coeff_out[add(n_swb_overlap_offset,i)],
            Mult_32_16(L_sub(hq_swb_overlap_buf[i], coeff_out[add(n_swb_overlap_offset,i)]), weighting));
            move32();
            weighting = sub(weighting, step);
        }
    }

    FOR (n_band = num_env_bands; n_band < num_sfm; n_band++)
    {
        IF (R[n_band] !=0 )
        {
            FOR(i=sfm_end[sub(n_band,1)]; i<sfm_end[n_band]; ++i)
            {
                coeff_out[i] = hq_swb_overlap_buf[sub(i,n_swb_overlap_offset)];
                move32();
            }
        }
    }
    return;
}

/*--------------------------------------------------------------------------*
 * map_hq_generic_fenv_norm()
 *
 * mapping high frequency envelope to high band norm
 *--------------------------------------------------------------------------*/
void map_hq_generic_fenv_norm_fx(
    const Word16 hqswb_clas,
    const Word16 *hq_generic_fenv,               /* Q1, frequency-domain BWE envelope */
    Word16 *ynrm,
    Word16 *normqlg2,
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 hq_generic_offset
)
{
    Word32 env_fl[17];/*Q10 */
    Word16 i;

    set32_fx(env_fl,0,17);

    IF ( sub(hq_generic_offset,144) == 0 )
    {
        env_fl[0] = L_shl(hq_generic_fenv[1],7);
        move32();
        env_fl[1] = L_add(L_mult0(hq_generic_fenv[2],85),L_mult0(hq_generic_fenv[3],43));
        move32();
        env_fl[2] = L_add(L_mult0(hq_generic_fenv[3],85),L_mult0(hq_generic_fenv[4],43));
        move32();
        env_fl[3] = L_add(L_mult0(hq_generic_fenv[4],43),L_mult0(hq_generic_fenv[5],85));
        move32();
        env_fl[4] = L_add(L_mult0(hq_generic_fenv[5],43),L_mult0(hq_generic_fenv[6],85));
        move32();
        env_fl[5] = L_shl(hq_generic_fenv[7],7);
        move32();
        env_fl[6] = L_add(L_mult0(hq_generic_fenv[8],96),L_mult0(hq_generic_fenv[9],32));
        move32();
        env_fl[7] = L_add(L_mult0(hq_generic_fenv[9],96),L_mult0(hq_generic_fenv[10],32));
        move32();
        env_fl[8] = L_add(L_mult0(hq_generic_fenv[10],32),L_mult0(hq_generic_fenv[11],96));
        move32();
    }
    ELSE
    {
        env_fl[0] =  L_add(L_mult0(hq_generic_fenv[0],43),L_mult0(hq_generic_fenv[1],85));
        move32();
        env_fl[1] =  L_add(L_mult0(hq_generic_fenv[1],43),L_mult0(hq_generic_fenv[2],85));
        move32();
        env_fl[2] =  L_shl(hq_generic_fenv[3],7);
        move32();
        env_fl[3] =  L_add(L_mult0(hq_generic_fenv[4],85),L_mult0(hq_generic_fenv[5],43));
        move32();
        env_fl[4] =  L_add(L_mult0(hq_generic_fenv[5],85),L_mult0(hq_generic_fenv[6],43));
        move32();
        env_fl[5] =  L_add(L_mult0(hq_generic_fenv[6],43),L_mult0(hq_generic_fenv[7],85));
        move32();
        env_fl[6] =  L_add(L_mult0(hq_generic_fenv[7],43),L_mult0(hq_generic_fenv[8],85));
        move32();
        env_fl[7] =  L_add(L_mult0(hq_generic_fenv[8],43),L_mult0(hq_generic_fenv[9],85));
        move32();
        env_fl[8] =  L_add(L_mult0(hq_generic_fenv[9],43),L_mult0(hq_generic_fenv[10],85));
        move32();
        env_fl[9] =  L_add(L_mult0(hq_generic_fenv[10],32),L_mult0(hq_generic_fenv[11],96));
        move32();
        env_fl[10] =  L_shl(hq_generic_fenv[12],7);
        move32();
        env_fl[11] =  L_shl(hq_generic_fenv[13],7);
        move32();
    }

    IF ( sub(hqswb_clas, HQ_GEN_FB) == 0 )
    {
        IF ( sub( hq_generic_offset, 144) == 0 )
        {
            env_fl[9] = L_shl(hq_generic_fenv[12],7);
            move32();
            env_fl[10] =  L_add(L_mult0(hq_generic_fenv[12],32),L_mult0(hq_generic_fenv[13],96));
            move32();
            env_fl[11] =  L_add(L_mult0(hq_generic_fenv[13],64),L_mult0(hq_generic_fenv[14],64));
            move32();
            env_fl[12] = L_shl(hq_generic_fenv[12],7);
            move32();
            env_fl[13] = env_fl[12];
            move32();
        }
        ELSE
        {
            env_fl[12] = L_shl(hq_generic_fenv[14],7);
            move32();
            env_fl[13] =  L_add(L_mult0(hq_generic_fenv[14],32),L_mult0(hq_generic_fenv[15],96));
            move32();
            env_fl[14] =  L_add(L_mult0(hq_generic_fenv[15],64),L_mult0(hq_generic_fenv[16],64));
            move32();
            env_fl[15] = L_shl(hq_generic_fenv[16],7);
            move32();
            env_fl[16] = env_fl[15];
            move32();
        }
    }

    logqnorm_2_fx( env_fl,40, num_env_bands, nb_sfm, ynrm+num_env_bands, normqlg2+num_env_bands, thren_fx);

    FOR( i=num_env_bands; i < nb_sfm; ++i )
    {
        normqlg2[i] = dicnlg2[min(add(ynrm[i],10),39)];
        move16();
    }
    return;
}

static void update_rsubband_fx(const Word16 nb_sfm,
                               Word16 *Rsubband,                        /* Q3 */
                               Word16 b_add_bits_denv
                              )
{
    Word16 i;

    /* updating bit allocation */
    WHILE (b_add_bits_denv>0)
    {
        i=sub(nb_sfm,1);
        WHILE(b_add_bits_denv>0 && i>=0)
        {
            IF ( sub(Rsubband[i], 24)>0)
            {
                Rsubband[i] = sub(Rsubband[i] , 8);
                move16();
                b_add_bits_denv = sub(b_add_bits_denv,1);
            }
            i = sub(i,1);
        }
    }

    return;
}

Word16 get_nor_delta_hf_fx(
    Decoder_State_fx *st,
    Word16 *ynrm,
    Word16 *Rsubband,                       /* Q3 */
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 core_sfm
)
{
    Word16 i;
    Word16 delta,bitsforDelta,add_bits_denv;

    add_bits_denv = 0;
    move16();
    IF (sub(core_sfm ,num_env_bands) >= 0)
    {
        bitsforDelta = (Word16)get_next_indice_fx(st,2);
        bitsforDelta = add(bitsforDelta, 2);
        add_bits_denv = add(add_bits_denv,2);

        FOR(i=num_env_bands; i<nb_sfm; ++i)
        {
            IF (Rsubband[i]!=0)
            {
                delta = (Word16)get_next_indice_fx(st,bitsforDelta);
                ynrm[i] = add(ynrm[i],sub(delta ,(shl(1,sub(bitsforDelta,1)))));
                move16();
                /*ynrm[i] += delta - (1<<(bitsforDelta-1));*/
                add_bits_denv = add(add_bits_denv,bitsforDelta);
            }
        }
        update_rsubband_fx(nb_sfm, Rsubband,add_bits_denv);
    }
    return add_bits_denv;
}

Word16 calc_nor_delta_hf_fx(
    Encoder_State_fx *st,
    const Word32 *t_audio,
    Word16 *ynrm,
    Word16 *Rsubband,
    const Word16 num_env_bands,
    const Word16 nb_sfm,
    const Word16 *sfmsize,
    const Word16 *sfm_start,
    const Word16 core_sfm
)
{
    Word16 i;
    Word16 ynrm_t[44],normqlg2_t[44];
    Word16 delta,max_delta,min_delta,bitsforDelta,add_bits_denv;
    Word16 temp_num;

    temp_num=0;
    move16();

    max_delta=-100;
    move16();
    calc_norm_fx( t_audio, 12, ynrm_t, normqlg2_t, 0, nb_sfm, sfmsize, sfm_start );
    add_bits_denv = 0;
    move16();
    FOR(i=num_env_bands; i<nb_sfm; ++i)
    {
        IF (Rsubband[i]!=0)
        {
            delta = sub(ynrm_t[i] , ynrm[i]);
            IF (delta > 0)
            {
                delta = add(delta,1);
            }
            ELSE
            {
                delta = negate(delta);
            }
            if (sub(delta,max_delta)>0)
            {
                max_delta = delta;
                move16();
            }
        }
    }
    IF (sub(core_sfm ,num_env_bands)>= 0)
    {
        IF (sub(max_delta, 16) < 0)
        {
            bitsforDelta = 2;
            move16();
            FOR( ; max_delta >= 2; max_delta >>= 1 )
            {
                bitsforDelta = add(bitsforDelta,1);
            }
        }
        ELSE
        {
            bitsforDelta = 5;
            move16();
        }
        max_delta = sub( shl(1, sub(bitsforDelta,1)), 1);
        min_delta = negate(add(max_delta,1));

        /* updating norm & storing delta norm */
        add_bits_denv = 2;
        move16();
        push_indice_fx( st, IND_DELTA_ENV_HQ, sub(bitsforDelta,2) , 2 );
        FOR(i=num_env_bands; i<nb_sfm; ++i)
        {
            IF (Rsubband[i]!=0)
            {
                delta = sub(ynrm_t[i] ,ynrm[i]);
                IF (sub(delta, max_delta) > 0)
                {
                    delta = max_delta;
                    move16();
                }
                ELSE if (sub(delta , min_delta) < 0)
                {
                    delta = min_delta;
                    move16();
                }
                push_indice_fx( st, IND_DELTA_ENV_HQ, delta - min_delta , bitsforDelta );
                ynrm[i] = add(ynrm[i], delta);
                move16();
                add_bits_denv = add(add_bits_denv, bitsforDelta);


                temp_num = add(temp_num,1);
            }
        }

        /* updating bit allocation */
        update_rsubband_fx(nb_sfm, Rsubband,add_bits_denv);
    }
    return add_bits_denv;
}


/*-------------------------------------------------------------------*
* hq_bwe_fx()
*
* HQ GENERIC
*--------------------------------------------------------------------------*/
void hq_bwe_fx(
    const Word16 HQ_mode,                        /* i  : HQ mode                                     */
    Word32 *coeff_out1,                    /* i/o: BWE input & temporary buffer                */
    const Word16 *hq_generic_fenv,               /* i  : SWB frequency envelopes                     */
    Word32 *coeff_out,                     /* o  : SWB signal in MDCT domain                   */
    const Word16 hq_generic_offset,              /* i  : frequency offset for representing hq swb bwe*/
    Word16 *prev_L_swb_norm,               /*i/o : last normalize length                       */
    const Word16 hq_generic_exc_clas,            /* i  : bwe excitation class                        */
    const Word16 *sfm_end,                       /* i  : End of bands                                */
    const Word16 num_sfm,
    const Word16 num_env_bands,
    const Word16 *R
)
{
    Word16 n_swb_overlap_offset, n_swb_overlap;
    Word32 hq_swb_overlap_buf_fx[L_FRAME32k];

    n_swb_overlap_offset = add(swb_bwe_subband_fx[0] , hq_generic_offset);
    n_swb_overlap = sub(sfm_end[sub(num_env_bands,1)] , n_swb_overlap_offset);


    Copy32(&coeff_out[n_swb_overlap_offset], hq_swb_overlap_buf_fx, sub(add(n_swb_overlap , sfm_end[sub(num_sfm,1)]) , sfm_end[sub(num_env_bands,1)] ));

    hq_generic_decoding_fx(HQ_mode, coeff_out1, hq_generic_fenv, coeff_out, hq_generic_offset, prev_L_swb_norm, hq_generic_exc_clas);

    overlap_hq_bwe_fx( hq_swb_overlap_buf_fx, coeff_out, n_swb_overlap_offset, n_swb_overlap, R, num_env_bands, num_sfm, sfm_end );

    return;
}

/*--------------------------------------------------------------------------*
 * hq_wb_nf_bwe()
 *
 * HQ WB noisefill and BWE
 *--------------------------------------------------------------------------*/

void hq_wb_nf_bwe_fx(
    const Word16 *coeff_fx,                 /* i  : coded/noisefilled normalized spectrum */
    const Word16 is_transient,
    const Word16 prev_bfi,                  /* i  : previous bad frame indicator    */
    const Word32 *L_normq_v,
    const Word16 num_sfm,                   /* i  : Number of subbands              */
    const Word16 *sfm_start,                /* i  : Subband start coefficient       */
    const Word16 *sfm_end,                  /* i  : Subband end coefficient         */
    const Word16 *sfmsize,                  /* i  : Subband band width              */
    const Word16 last_sfm,                  /* i  : last coded subband              */
    const Word16 *R,                        /* i  : bit allocation                  */
    const Word16 prev_is_transient,         /* i  : previous transient flag         */
    Word32 *prev_normq_fx,            /* i/o: previous norms                  */
    Word32 *prev_env_fx,              /* i/o: previous noise envelopes        */
    Word16 *bwe_seed,                 /* i/o: random seed for generating BWE input */
    Word32 *prev_coeff_out_fx,        /* i/o: decoded spectrum in previous frame */
    Word16 *prev_R,                   /* i/o: bit allocation info. in previous frame */
    Word32 *L_coeff_out,              /* o  : coded/noisefilled spectrum      */
    Word16 *prev_env_Q
)
{
    Word16 i;
    Word16 sfm;
    Word16 total_bit;
    Word16 num;
    Word32 env_fx,peak_fx,fabs_coeff_out_fx,min_coef_fx;
    Word32 L_tmp1,L_tmp2=0,L_tmp3,L_tmp4;
    Word16 tmp1,exp=0,exp1,exp2,exp3,harm_para_fx,sharp_fx,step_fx,alfa_fx = 4096;
    Word32 avrg_norm_fx,prev_avrg_norm_fx;
    Word16 bitalloc_var_fx;
    Word16 tmp;
    Word32 L_tmp;
    Word32 mean_fx;

    IF( is_transient == 0 )
    {
        IF( sub(prev_bfi, 1) == 0)
        {
            Copy32(L_normq_v,prev_normq_fx,SFM_N_WB);
        }

        /* the variance of bit allocation */
        total_bit = 0;
        bitalloc_var_fx = 0;
        FOR (sfm = 8; sfm <= last_sfm; sfm++)
        {
            tmp = abs_s(sub(R[sfm],R[sub(sfm,1)]));
            bitalloc_var_fx = add(bitalloc_var_fx,tmp);
            total_bit = add(total_bit,R[sfm]);
        }
        IF(sub(last_sfm,8) > 0)
        {
            exp = norm_s(total_bit);
            tmp = shl(total_bit,exp);/*Q(exp) */
            tmp = div_s(16384,tmp);/*Q(15+14-exp) */
            L_tmp = L_mult(tmp, bitalloc_var_fx); /*Q(29-exp+1) */
            bitalloc_var_fx = round_fx(L_shl(L_tmp,exp));/*Q14 */
        }
        ELSE
        {
            bitalloc_var_fx = 0;                              /*Q14 */
        }
        /* calculate the peak-average ratio of saturable subbands */
        num = 0;
        sharp_fx = 0;
        FOR(sfm = last_sfm; sfm >= 8; sfm--)
        {
            tmp = shl(sfmsize[sfm], 9);/*Q9 */
            tmp = mult( rat_fx[sfm],tmp );/*Q(14+9-15=8) */
            IF(sub(shl(R[sfm],8),tmp) >= 0)
            {
                peak_fx = 0;
                move16();
                mean_fx = 0;
                move16();
                FOR( i = sfm_start[sfm]; i < sfm_end[sfm]; i++ )
                {
                    fabs_coeff_out_fx = L_abs(L_coeff_out[i]);
                    mean_fx = L_add(mean_fx, fabs_coeff_out_fx);/*Q12 */
                    if(L_sub(fabs_coeff_out_fx, peak_fx) > 0)
                    {
                        peak_fx = L_add(fabs_coeff_out_fx,0);/*Q12 */
                    }
                }

                IF(mean_fx != 0)
                {
                    exp = norm_l(mean_fx);
                    mean_fx = L_shl(mean_fx, exp);/*Q(exp+12) */
                    tmp = round_fx(mean_fx);/*Q(exp-4) */
                    tmp = div_s(16384,tmp); /*Q(15+14-exp+4 = 33-exp) */
                    L_tmp = Mult_32_16(peak_fx, tmp);/*Q(12+33-exp-15 = 30-exp) */
                    tmp = shl(sfmsize[sfm], 9);/*Q9 */
                    L_tmp = Mult_32_16(L_tmp, tmp);/*Q(30-exp+9-15 = 24-exp) */
                    tmp = round_fx(L_shl(L_tmp,exp));/*Q8   */
                    sharp_fx = add(sharp_fx,tmp);
                }
                num = add(num,1);
            }
        }
        IF(num != 0)
        {
            num = add(num,num);
            exp = norm_s(sharp_fx);
            sharp_fx = shl(sharp_fx,exp);/*Q(8+exp) */
            tmp = div_s(16384,sharp_fx);/*Q(21-exp) */
            L_tmp = L_mult(num,tmp);/*Q(22-exp) */
            sharp_fx = round_fx(L_shl(L_tmp,add(8,exp)));/*Q14 */
        }
        ELSE
        {
            sharp_fx = 16384;
            move16();/*Q14 = 1 */
        }
        harm_para_fx = sharp_fx;
        move16();/*Q14 */

        IF(last_sfm == 0)
        {
            tmp = 0;
            move16();
        }
        ELSE
        {
            tmp = div_s(1,last_sfm);/*Q15 */
        }

        L_tmp = L_mult(5,sharp_fx);/*Q15 */
        L_tmp = Mult_32_16(L_tmp,tmp);/*Q15 */
        step_fx = round_fx(L_shl(L_tmp,16));/*Q15  */
        alfa_fx = 20480;
        move16();/*Q13 = 2.5 */
        /* fill noise for the insaturable subbands */
        FOR( sfm = 0; sfm < num_sfm; sfm++ )
        {
            env_fx   = L_deposit_l(0);
            L_tmp2 =L_deposit_l(0);
            exp = 0;
            move16();
            test();
            IF(R[sfm] != 0 && sub(R[sfm], shl(mult(24756,sfmsize[sfm]),1)) < 0)
            {
                /* calculate the energy of the undecoded coefficients */
                env_fx =L_deposit_l(0);
                exp1=norm_l(L_normq_v[sfm]);
                L_tmp4 = L_shl(L_normq_v[sfm],exp1); /*14+exp1 */
                L_tmp1 = Mult_32_32(L_tmp4,L_tmp4); /*2*exp1-3   14+exp1+14+exp1 -31  */
                L_tmp2 =L_deposit_l(0);
                peak_fx = L_deposit_l(0);
                min_coef_fx = L_add(0,0x7fffffff);

                FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                {
                    fabs_coeff_out_fx = abs(L_coeff_out[i]);
                    test();
                    if(L_sub(fabs_coeff_out_fx, min_coef_fx)<0 && L_coeff_out[i] != 0)
                    {
                        min_coef_fx = L_add(0,fabs_coeff_out_fx);
                    }
                    if(L_sub(fabs_coeff_out_fx,peak_fx) > 0)
                    {
                        peak_fx = L_add(0,fabs_coeff_out_fx);
                    }
                }

                exp2 =norm_l(peak_fx);
                FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                {
                    L_tmp4 = L_shl(L_coeff_out[i],exp2); /*12+exp2 */
                    L_tmp3 = L_shr(Mult_32_32(L_tmp4,L_tmp4),4);/*2*exp2-7-4    12+exp2+12+exp2-31-4   */
                    L_tmp2 = L_add(L_tmp2 ,L_tmp3); /*2*exp2-11 */
                }

                tmp1 =div_s(1,sfmsize[sfm]); /*15  */
                L_tmp4 = Mult_32_16(L_tmp2,tmp1);/*2*exp2-11   2*exp2-7+15+1-16              */

                exp =norm_l(L_tmp1);
                L_tmp1 = L_shl(L_tmp1,exp);/*exp + 2*exp1 - 3 */
                exp1 =sub(add(exp,shl(exp1,1)),3);

                exp =norm_l(L_tmp4);
                L_tmp4 = L_shl(L_tmp4,exp);/*exp + 2*exp1 - 3 */
                exp2 =sub(add(exp,shl(exp2,1)),11);
                exp =min(exp1,exp2);

                L_tmp1 = L_shl(L_tmp1,sub(exp,exp1));
                L_tmp4 = L_shl(L_tmp4,sub(exp,exp2));
                env_fx= L_sub(L_tmp1,L_tmp4); /*exp */
                exp1 = norm_l(env_fx);
                env_fx = L_shl(env_fx,exp1);/*exp + exp1 */
                exp =add(exp,exp1);

                IF (env_fx > 0 )
                {
                    IF(sfm == 0)
                    {
                        avrg_norm_fx = L_add(L_shr(L_normq_v[0],1),L_shr(L_normq_v[1],1));/*13 */
                        avrg_norm_fx = L_add(avrg_norm_fx,L_shr(L_normq_v[2],1));/*13                      */
                        prev_avrg_norm_fx = L_add(L_shr(prev_normq_fx[0],1),L_shr(prev_normq_fx[1],1));/*13 */
                        prev_avrg_norm_fx = L_add(prev_avrg_norm_fx,L_shr(prev_normq_fx[2],1));/*13 */
                    }
                    ELSE IF (sub(sfm,25) == 0)
                    {
                        avrg_norm_fx = L_add(L_shr(L_normq_v[23],1),L_shr(L_normq_v[24],1));/*13 */
                        avrg_norm_fx = L_add(avrg_norm_fx,L_shr(L_normq_v[25],1));/*13 */
                        prev_avrg_norm_fx = L_add(L_shr(prev_normq_fx[23],1),L_shr(prev_normq_fx[24],1));/*13 */
                        prev_avrg_norm_fx = L_add(prev_avrg_norm_fx,L_shr(prev_normq_fx[25],1));/*13 */
                    }
                    ELSE
                    {
                        avrg_norm_fx = L_add(L_shr(L_normq_v[sub(sfm,1)],1),L_shr(L_normq_v[sfm],1));/*13 */
                        avrg_norm_fx = L_add(avrg_norm_fx,L_shr(L_normq_v[sfm+1],1));/*13 */
                        prev_avrg_norm_fx = L_add(L_shr(prev_normq_fx[sub(sfm,1)],1),L_shr(prev_normq_fx[sfm],1));/*13 */
                        prev_avrg_norm_fx = L_add(prev_avrg_norm_fx,L_shr(prev_normq_fx[add(sfm,1)],1));/*13 */
                    }

                    test();
                    test();
                    IF((sub(bitalloc_var_fx,4915) > 0 || L_sub(L_normq_v[sfm], peak_fx)<0)

                       && peak_fx != 0
                      )/* */
                    {

                        exp =sub(31,exp);
                        env_fx = Isqrt_lc(env_fx,&exp);
                        L_tmp1 = Mult_32_32(env_fx,peak_fx);/*12-exp  31-exp+12-31  */
                        L_tmp2 = Mult_32_16(avrg_norm_fx,harm_para_fx);/*12 13 + 14 + 1 -16 */
                        exp1 = norm_l(L_tmp1);
                        L_tmp1 = L_shl(L_tmp1,exp1);/* 12-exp+exp1 */
                        exp = add(sub(12,exp),exp1);
                        L_tmp2 = Div_32(L_tmp2, extract_h(L_tmp1), extract_l(L_tmp1));
                        exp=sub(43,exp);
                    }
                    ELSE
                    {
                        L_tmp1 = Mult_32_16(L_normq_v[sfm],alfa_fx);/*12 13 + 14 + 1 -16 */
                        IF(L_sub(L_tmp1,peak_fx)<0)
                        {
                            exp=sub(31,exp);
                            env_fx = Isqrt_lc(env_fx,&exp);
                            exp1 = norm_l(env_fx);
                            env_fx = L_shl(env_fx,exp1);/* 31-exp+exp1 */
                            exp = add(sub(31,exp),exp1);
                            L_tmp1 = (Word32)sharp_fx;
                            L_tmp2 = Div_32(L_tmp1, extract_h(env_fx), extract_l(env_fx)); /* 45-exp 14 - exp + 31 //39-exp 8 - exp + 31 */
                            exp = sub(45,exp);
                            exp1 = norm_l(peak_fx);
                            L_tmp1 = L_shl(peak_fx,exp1); /*12 + exp1 */
                            L_tmp1 = Div_32(L_tmp2,extract_h(L_tmp1), extract_l(L_tmp1)); /*  exp - (12 + exp1) +31 */
                            L_tmp2 = Mult_32_32(L_tmp2,L_tmp1);/*2*exp+exp1-12    exp +exp - (12 + exp1) +31 - 31 */
                            exp = sub(add(shl(exp,1),exp1),12);
                        }
                        ELSE
                        {
                            exp = sub(31,exp);
                            env_fx = Isqrt_lc(env_fx,&exp);
                            exp1 = norm_l(env_fx);
                            env_fx =L_shl(env_fx,exp1);/* 31-exp+exp1 */
                            exp = add(sub(31,exp),exp1);
                            L_tmp1=(Word32)(sharp_fx);
                            L_tmp2 = Div_32(L_tmp1, extract_h(env_fx), extract_l(env_fx)); /* 45-exp 14 - exp + 31  //39-exp 8 - exp + 31  */
                            exp =sub(45,exp);
                        }

                        sharp_fx = add(sharp_fx,shr(step_fx,1));
                    }

                    IF(L_sub(L_tmp2,L_shl(min_coef_fx,sub(exp,13)))>0)/*exp  */
                    {
                        L_tmp2 = L_shr(min_coef_fx,1);
                        exp = 12;
                        move16();
                    }

                    IF(sub(prev_bfi,1) == 0)
                    {
                        prev_env_Q[sfm] = exp;
                        move16();
                        prev_env_fx[sfm] = L_tmp2;
                        move32();
                    }
                    /* smooth the noise magnitudes between inter-frame */
                    test();
                    test();
                    IF(L_sub(prev_avrg_norm_fx,L_shr(avrg_norm_fx,1))>0 && L_sub(prev_avrg_norm_fx,L_shl(avrg_norm_fx,1))<0 && prev_is_transient == 0)
                    {
                        exp1 =norm_l(prev_env_fx[sfm]);
                        L_tmp1 = L_shl(prev_env_fx[sfm],exp1);/* prev_env_Q[sfm] +exp1 */

                        exp2= norm_l(L_tmp2);
                        L_tmp3 = L_shl(L_tmp2,exp2);/* exp +exp2 */

                        exp3 =min(add(prev_env_Q[sfm],exp1),add(exp,exp2));

                        L_tmp1 = L_shl(L_tmp1,sub(sub(exp3,prev_env_Q[sfm]),exp1)); /*exp3 */
                        L_tmp3 = L_shl(L_tmp3,sub(sub(exp3,exp),exp2)); /*exp3 */
                        L_tmp2 = L_add(L_shr(L_tmp1,1),L_shr(L_tmp3,1));/*exp3 */
                        exp = exp3;
                        move16();
                    }
                    FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                    {
                        IF (coeff_fx[i] == 0)
                        {
                            tmp1 = Random(bwe_seed);/*Q15 */
                            L_tmp1= Mult_32_16(L_tmp2,tmp1);/*exp   exp+15+1 -16 */
                            L_tmp1 = L_shl(L_tmp1,sub(12,exp));
                            L_coeff_out[i] = L_tmp1;
                            move32();
                        }
                    }
                }
                ELSE
                {
                    exp =0;
                    move16();
                    L_tmp2 =L_deposit_l(0);
                }
            }
            ELSE IF(R[sfm] == 0)
            {
                /* fill random noise for 0 bit subbands */
                FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                {
                    IF(coeff_fx[i] == 0)
                    {
                        tmp1 =Random(bwe_seed);/*Q15 */
                        L_tmp1= Mult_32_16(L_normq_v[sfm],tmp1);/*14   14+15+1 -16 */
                        L_tmp1 = L_shr(L_tmp1,2);/* */
                        L_coeff_out[i] = L_tmp1;
                        move32();
                    }
                }
                L_tmp2 = L_add(0,L_normq_v[sfm]);
                exp = 14;
                move16();
            }

            test();
            test();
            test();
            test();
            IF(sub(sfm,sub(SFM_N_WB,1))==0 && prev_is_transient == 0 && L_sub(prev_normq_fx[sfm],L_shr(L_normq_v[sfm],1))>0
               && L_sub(prev_normq_fx[sfm],L_shl(L_normq_v[sfm],1)) < 0 && sub(bitalloc_var_fx,4915) <= 0)
            {
                FOR (i = add(sfm_start[sfm],12); i < sfm_end[sfm]; i++)
                {
                    test();
                    test();
                    test();
                    test();
                    IF(L_sub(abs(L_coeff_out[i]),L_shl(abs(prev_coeff_out_fx[i]),2)) >0
                       || L_sub(abs(L_coeff_out[i]),L_shr(abs(prev_coeff_out_fx[i]),2)) <0
                       || ((R[sfm] ==0 ||prev_R[sfm] == 0) && add(R[sfm],prev_R[sfm]) != 0))
                    {
                        IF(L_coeff_out[i] >0)
                        {
                            L_coeff_out[i] = L_add(L_shr(abs(L_coeff_out[i]),1),L_shr(abs(prev_coeff_out_fx[i]),1));
                            move32();
                        }
                        ELSE
                        {
                            L_coeff_out[i] = -L_add(L_shr(abs(L_coeff_out[i]),1),L_shr(abs(prev_coeff_out_fx[i]),1));
                            move32();
                        }
                    }
                }
            }
            prev_env_Q[sfm] = exp;
            move16();
            prev_env_fx[sfm] = L_tmp2;
            move32();
        }
    }
    ELSE
    {
        /* fill random noise for 0 bit subbands of transient frame */
        FOR(sfm = 0; sfm < num_sfm; sfm++)
        {
            IF( R[sfm] == 0 )
            {
                FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                {
                    tmp1 =Random(bwe_seed);/*Q15 */
                    L_tmp1= Mult_32_16(L_normq_v[sfm],tmp1);/*14   14+15+1 -16 */
                    L_tmp1 =L_shr(L_tmp1,2);/* */
                    L_coeff_out[i] = L_tmp1;
                    move32();
                }
            }
        }
        set16_fx( prev_env_Q, 0, SFM_N_WB );
        set32_fx( prev_env_fx, 0, SFM_N_WB );
    }

    Copy32(L_normq_v,prev_normq_fx,SFM_N_WB);
    Copy32(L_coeff_out,prev_coeff_out_fx, sfm_end[num_sfm-1]);
    Copy(R,prev_R,num_sfm);
    return;
}

/*--------------------------------------------------------------------------*
 * enforce_zero_for_min_envelope_fx()
 *
 * Detect minimum level of envelope and set corresponding bands to zero
 *--------------------------------------------------------------------------*/

void enforce_zero_for_min_envelope_fx(
    const Word16 hqswb_clas,     /* i  : HQ coding mode                     Q0  */
    const Word16 *ynrm,          /* i  : Envelope indices                   Q0  */
    Word32 *L_coefsq,      /* i/o: Quantized spectrum/zeroed spectrum Q12 */
    const Word16 nb_sfm,         /* i  : Number of coded sub bands          Q0  */
    const Word16 *sfm_start,     /* i  : Sub band start indices             Q0  */
    const Word16 *sfm_end        /* i  : Sub band end indices               Q0  */
)
{
    Word16 i, j;

    /* prevent non-zero output for all-zero input */
    IF( sub(hqswb_clas,HQ_HVQ) != 0 )
    {
        IF( sub(ynrm[0], 31) == 0 )
        {
            FOR( j = sfm_start[0]; j < sfm_end[0]; j++ )
            {
                L_coefsq[j] = L_deposit_l(0);
            }
        }

        FOR( i = 1; i < nb_sfm; i++ )
        {
            IF( sub(ynrm[i], 39) == 0 )
            {
                FOR( j = sfm_start[i]; j < sfm_end[i]; j++ )
                {
                    L_coefsq[j] = L_deposit_l(0);
                }
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * apply_envelope()
 *
 * Apply spectral envelope with envelope adjustments
 *--------------------------------------------------------------------------*/

void apply_envelope_fx(
    const Word16 *coeff,             /* i/o: Coded/noisefilled normalized spectrum  Q12 */
    const Word16 *norm,              /* i  : Envelope                               Q0  */
    const Word16 *norm_adj,          /* i  : Envelope adjustment                    Q15 */
    const Word16 num_sfm,            /* i  : Total number of bands                  Q0  */
    const Word16 last_sfm,           /* i  : Last coded band                        Q0  */
    const Word16 HQ_mode,            /* i  : HQ mode                                Q0  */
    const Word16 length,             /* i  : Frame length                           Q0  */
    const Word16 *sfm_start,         /* i  : Sub band start indices                 Q0  */
    const Word16 *sfm_end,           /* i  : Sub band end indices                   Q0  */
    Word32 *normq_v,           /* o  : Envelope with adjustment               Q14 */
    Word32 *coeff_out,         /* o  : coded/noisefilled spectrum             Q12 */
    const Word16 *coeff1,            /* i  : coded/noisefilled spectrum             Q12 */
    Word32 *coeff_out1         /* o  : coded/noisefilled spectrum             Q12 */
)
{
    Word16 i;
    Word16 sfm;
    UWord16 lsb;
    Word32 normq;
    Word32 L_tmp;
    Word16 len;

    len = num_sfm;
    move16();
    test();
    if( sub(HQ_mode, HQ_GEN_SWB) == 0 || sub(HQ_mode, HQ_GEN_FB) == 0 )
    {
        len = add(last_sfm, 1);
    }

    IF( sub(length, L_FRAME16k) == 0 )
    {
        FOR (sfm = 0; sfm < num_sfm; sfm++)
        {
            normq_v[sfm] = dicn_fx[norm[sfm]];
            move16();
            move32();
            /*normq = normq_v[sfm] * norm_adj[sfm];               */
            Mpy_32_16_ss(normq_v[sfm], norm_adj[sfm], &normq, &lsb);        /* Q14 (14+15+1-16) */

            FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            {
                /*coeff_out[i] = coeff[i]*normq; */
                Mpy_32_16_ss(normq, coeff[i], &L_tmp, &lsb);
                coeff_out[i] = L_add(L_shl(L_tmp, 1), lshr(lsb, 15));
                move32();   /* Q12 (14+12+1-16)+1 */
            }
        }
    }
    ELSE
    {
        FOR (sfm = 0; sfm < len; sfm++)
        {
            normq_v[sfm] = dicn_fx[norm[sfm]];
            move16();
            move32();
            /*normq_v[sfm] *= norm_adj[sfm];               */
            Mpy_32_16_ss(normq_v[sfm], norm_adj[sfm], &normq_v[sfm], &lsb);
            move32();  /* Q14 (14+15+1-16) */

            normq = L_add(0,normq_v[sfm]);
            FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            {
                /*coeff_out[i] = coeff[i]*normq; */
                Mpy_32_16_ss(normq, coeff[i], &L_tmp, &lsb);
                coeff_out[i] = L_add(L_shl(L_tmp, 1), lshr(lsb, 15));
                move32();   /* Q12 (14+12+1-16)+1 */
            }
        }

        test();
        IF ( sub(HQ_mode, HQ_GEN_SWB) == 0 || sub(HQ_mode, HQ_GEN_FB) == 0 )
        {
            FOR (sfm = 0; sfm <= last_sfm; sfm++)
            {
                normq = L_add(0,normq_v[sfm]);
                FOR (i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
                {
                    /*coeff_out1[i] = coeff_out1[i]*normq; */
                    Mpy_32_16_ss(normq, coeff1[i], &L_tmp, &lsb);
                    coeff_out1[i] = L_add(L_shl(L_tmp, 1), lshr(lsb, 15));
                    move32();   /* Q12 (14+12+1-16)+1 */
                }
            }
        }
    }


    return;
}


