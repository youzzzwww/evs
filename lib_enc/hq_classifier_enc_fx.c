/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "stl.h"            /* Debug prototypes                       */

/*-----------------------------------------------------------------*
 * Local constants
 *-----------------------------------------------------------------*/

#define SHARP_DIST_THRES            22.2f
#define SHARP_DIST_THRES_FX         1420   /* Q6, 22.2 */

/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/

void hvq_classifier_fx( const Word32 *input, Word16 *prev_Npeaks, Word16 *prev_peaks,
                        Word16 *hqswb_clas, Word16 *Npeaks, Word16 *peaks, const Word32 L_core_brate,
                        const Word16 last_core, Word32 *L_nf_gains, Word16 *hvq_hangover, Word32 *L_pe_gains );

/*--------------------------------------------------------------------------*
 * hq_classifier_enc_fx()
 *
 * HQ mode selector (decision_matrix)
 *--------------------------------------------------------------------------*/

Word16 hq_classifier_enc_fx(        /* o  : Consumed bits                   Q0  */
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure             */
    const Word16 length,            /* i  : Frame length                    Q0  */
    const Word32 *coefs,            /* i  : Spectral coefficients           Q12 */
    const Word16 is_transient,      /* i  : Transient flag                  Q0  */
    Word16 *Npeaks,           /* o  : Number of identified peaks      Q0  */
    Word16 *peaks,            /* o  : Peak indices                    Q0  */
    Word32 *pe_gains,         /* o  : Peak gains                      Q12 */
    Word32 *nf_gains,         /* o  : Noise-fill gains                Q12 */
    Word16 *hqswb_clas        /* o  : HQ class                        Q0  */
)
{
    Word16 bits;

    *hqswb_clas = HQ_NORMAL;
    IF( sub( is_transient, 1) == 0 )
    {
        *hqswb_clas = HQ_TRANSIENT;
        move16();
    }

    /* classification and limit bandwidth for bit allocation */
    test();
    test();
    test();
    IF( sub(length, L_FRAME32k) == 0 && sub(is_transient,1) != 0 && L_sub(st_fx->core_brate_fx, HQ_32k) <= 0 && sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) == 0 )
    {
        /* Detect HQ_HARMONIC mode */
        *hqswb_clas = peak_avrg_ratio_fx( st_fx->total_brate_fx, coefs, NUMC_N+96, &st_fx->mode_count_fx, &st_fx->mode_count1_fx, 12 );

        /* Detect harmonic VQ mode HQ_HVQ */
        hvq_classifier_fx( coefs, &st_fx->prev_Npeaks_fx, st_fx->prev_peaks_fx, hqswb_clas, Npeaks, peaks, st_fx->core_brate_fx, st_fx->last_core_fx,
                           nf_gains, &st_fx->hvq_hangover_fx, pe_gains );
    }

    test();
    test();
    test();
    IF ( sub(length, L_FRAME48k) == 0 && sub( is_transient, 1) != 0 && L_sub(st_fx->core_brate_fx, HQ_32k) <= 0  && sub( st_fx->bwidth_fx, st_fx->last_bwidth_fx) == 0 )
    {
        /* Detect HQ_HARMONIC mode */
        *hqswb_clas = peak_avrg_ratio_fx( st_fx->total_brate_fx, coefs, NUMC_N+96, &st_fx->mode_count_fx, &st_fx->mode_count1_fx, 12);
        /* Detect harmonic VQ mode HQ_HVQ */
        hvq_classifier_fx( coefs, &st_fx->prev_Npeaks_fx, st_fx->prev_peaks_fx, hqswb_clas, Npeaks, peaks,
                           st_fx->core_brate_fx, st_fx->last_core_fx, nf_gains, &st_fx->hvq_hangover_fx, pe_gains );
    }

    test();
    test();
    IF( sub(length, L_FRAME48k) == 0 && L_sub(st_fx->core_brate_fx, HQ_32k) <= 0 && sub(*hqswb_clas,HQ_NORMAL) == 0 )
    {
        *hqswb_clas = HQ_GEN_FB;
        move16();
    }

    test();
    IF( sub( length, L_FRAME32k) >= 0 && L_sub( st_fx->core_brate_fx, HQ_32k) <= 0 )
    {
        bits = 2;
        move16();
    }
    ELSE
    {
        bits = 1;
        move16();
    }

    test();
    IF ( sub( length, L_FRAME48k) == 0 && L_sub( st_fx->core_brate_fx, HQ_32k) <= 0 )
    {
        IF ( sub( *hqswb_clas, HQ_GEN_SWB) >= 0)
        {
            push_indice_fx( st_fx, IND_HQ_SWB_CLAS, *hqswb_clas - 5, bits );
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_HQ_SWB_CLAS, *hqswb_clas, bits );
        }
    }
    ELSE
    {
        push_indice_fx( st_fx, IND_HQ_SWB_CLAS, *hqswb_clas, bits );
    }

    test();
    test();
    IF ( sub( *hqswb_clas, HQ_NORMAL) == 0 && sub( length, L_FRAME32k) == 0 && L_sub( st_fx->core_brate_fx, HQ_32k ) <= 0 )
    {
        *hqswb_clas = HQ_GEN_SWB;
        move16();
    }

    return bits;
}

/*--------------------------------------------------------------------------*
 * peak_avrg_ratio()
 *
 * Classify the input signal and decide if it has a harmonic structure
 *--------------------------------------------------------------------------*/
Word16 peak_avrg_ratio_fx(
    const Word32 total_brate,
    const Word32 *input_hi_fx,               /* i  : input signal           */
    const Word16 length,                     /* i  : number of coefficients */
    Word16 *mode_count,                /* i/o: HQ_HARMONIC mode count */
    Word16 *mode_count1,               /* i/o: HQ_NORMAL mode count   */
    Word16 Q_coeff
)
{
    Word16 i, j, q, k, k1, hqswb_clas;
    Word32 mean_fx, peak_fx;
    Word32 input_abs_fx[L_FRAME32k];
    Word32 peak_th_fx;

    FOR ( i = 96; i < length; i++)
    {
        input_abs_fx[i] = L_abs(input_hi_fx[i]);
    }

    hqswb_clas = HQ_NORMAL;
    move16();
    peak_th_fx = L_shl(10L, sub(Q_coeff, 5)); /* 5 is safe shift */

    k = 0;
    move16();
    k1 = 0;
    move16();
    q = 96;     /* q used for indexing */

    FOR( i = 3; i < 17; i ++ )
    {
        peak_fx = L_deposit_l(0);
        mean_fx = L_deposit_l(1);

        /*for(j = 0; j < 32; j ++, q ++) */
        FOR(j = 0; j < 32; j ++)
        {
            input_abs_fx[q] =L_shr(input_abs_fx[q],5); /*Q_coeff-5 */
            mean_fx =L_add(mean_fx,input_abs_fx[q]); /*Q_coeff-5 */
            IF (L_sub(input_abs_fx[q] , peak_fx)>0)
            {
                peak_fx =input_abs_fx[q] ; /*Q_coeff-5 */
            }
            q ++;
        }

        IF(sub(i,8) < 0)
        {
            if(L_sub(peak_fx, Mult_32_16(mean_fx, 4608))>0) /* Q15 0.140625 */
            {
                k = add(k,1);
            }
        }
        ELSE
        {
            test();
            if(L_sub(peak_fx, Mult_32_16(mean_fx, 3686))>0  /*Q15 0.1125 */
            && L_sub(peak_fx, peak_th_fx) >0) /*Q27 10 */
            {
                k1 = add(k1,1);
            }
        }
    }

    test();
    IF( sub(add(k,k1),10) >= 0 && sub(k1,5) > 0 )
    {
        if( sub(*mode_count,8) < 0 )
        {
            *mode_count = add(*mode_count,1);
        }

        if( *mode_count1 > 0 )
        {
            *mode_count1 = sub(*mode_count1,1);
        }
    }
    ELSE
    {
        if( sub(*mode_count1,8) < 0 )
        {
            *mode_count1 = add(*mode_count1,1);
        }

        if( *mode_count > 0 )
        {
            *mode_count = sub(*mode_count,1);
        }
    }

    test();
    test();
    test();
    test();
    test();
    test();
    if ((sub(add(k, k1), 5) >= 0 && sub(k1, 2) > 0 && L_sub(total_brate, HQ_24k40) == 0)
            || (((sub(add(k, k1), 10) >= 0 && sub(k1, 5) > 0) || sub(*mode_count, 5) >= 0) && sub(*mode_count1, 5) < 0))
    {
        hqswb_clas = HQ_HARMONIC;
        move16();
    }

    return hqswb_clas;
}

/*--------------------------------------------------------------------------*
 * hvq_classifier()
 *
 * Classification of harmonic low band content for Harmonic VQ
 *--------------------------------------------------------------------------*/

void hvq_classifier_fx(
    const Word32 *input,             /* i  : input signal                Q12 */
    Word16 *prev_Npeaks,       /* i/o: Peak number memory          Q0  */
    Word16 *prev_peaks,        /* i/o: Peak indices memory         Q0  */
    Word16 *hqswb_clas,        /* i/o: HQ class                    Q0  */
    Word16 *Npeaks,            /* o  : Number of peaks             Q0  */
    Word16 *peaks,             /* o  : Peak indices                Q0  */
    const Word32 L_core_brate,       /* i  : Core bit-rate               Q0  */
    const Word16 last_core,          /* i  : Last core used              Q0  */
    Word32 *L_nf_gains,        /* o  : Noisefloor gains            Q12 */
    Word16 *hvq_hangover,      /* i/o: Mode-switch hangover        Q0  */
    Word32 *L_pe_gains         /* o  : peak gains                  Q12 */
)
{
    const Word16 *p_adj;
    UWord16 lsb;

    Word32 L_input_abs[L_FRAME32k];
    Word32 L_input_max;
    Word32 L_thr[L_FRAME16k];
    Word32 L_thr_tmp;
    Word32 L_m;
    Word32 L_tmp;
    Word32 L_d;
    Word32 L_peak;
    Word32 L_nf, L_pe;
    Word32 L_pe_mean[HVQ_NSUB_32k], L_nf_mean[HVQ_NSUB_32k];

    Word16 inv_nsub;
    Word16 sharp_dist;
    Word16 exp1, exp2;
    Word16 tmp;
    Word16 shift;
    Word16 idx;
    Word16 frac;
    Word16 inv_nf_mean;
    Word16 inv_gains_nsub;
    Word16 nf_mean_norm;
    Word16 num_sharp_bands, i, j, k, q, peak_th, nsub, pindx, N, offset;
    Word16 num_peak_cands, high, low;
    Word16 sharp[HVQ_NSUB_32k];
    Word16 peak_cand_idx[HVQ_THRES_BIN_32k], avail_peaks[HVQ_NSUB_32k];

    L_input_max = L_deposit_l(0);
    set32_fx(L_thr, 0, L_FRAME16k);

    IF ( L_sub(L_core_brate, HQ_24k40) == 0 )
    {
        nsub = HVQ_NSUB_24k;
        move16();
        inv_nsub = 4681;
        move16();   /* 1/7 in Q15 */
        inv_gains_nsub = 10923;
        move16();   /* 1/3 in Q15 */
    }
    ELSE
    {
        nsub = HVQ_NSUB_32k;
        move16();
        inv_nsub = 3277;
        move16();   /* 1/10 in Q15 */
        inv_gains_nsub = 6554;
        move16();   /* 1/5 in Q15 */
    }

    N = shl(nsub, 5);   /* Mult by 32 (HVQ_BW) */

    test();
    test();
    IF ( sub(*hqswb_clas, HQ_HARMONIC) == 0 && last_core != ACELP_CORE && sub(last_core, AMR_WB_CORE) != 0 )
    {
        FOR ( i = 0; i < N; i++ )
        {
            L_input_abs[i] = L_abs(input[i]);
            if (L_input_abs[i] > L_input_max)
            {
                L_input_max = L_input_abs[i];
                move16();
            }
        }

        exp1 = norm_l(L_input_max);

        *Npeaks = 0;
        move16();
        L_nf = 3276800;
        move32();   /* 800 in Q12 */
        L_pe = 3276800;
        move32();   /* 800 in Q12    */
        num_sharp_bands = 0;
        move16();
        k = 0;
        move16();
        q = 0;
        move16();
        sharp_dist = 0;
        move16();

        /* Find peak threshold */
        FOR ( i = 0; i < nsub; i++ )
        {
            L_peak = 0;
            L_nf_mean[i] = 0;
            L_pe_mean[i] = 0;
            FOR ( j = 0; j < HVQ_BW; j++ )
            {
                L_d = L_input_abs[q];
                IF ( L_sub(L_d, L_nf) > 0 )
                {
                    /*nf = HVQ_NF_WEIGHT1 * nf + (1 - HVQ_NF_WEIGHT1) * d; */
                    Mpy_32_16_ss(L_d, HVQ_NF_WEIGHT1B, &L_tmp, &lsb);     /* 12+15-15=12 */
                    Mpy_32_16_ss(L_nf, HVQ_NF_WEIGHT1_FX, &L_nf, &lsb);     /* 12+15-15=12 */
                    L_nf = L_add(L_nf, L_tmp);  /*Q12 */
                }
                ELSE
                {
                    /*nf = HVQ_NF_WEIGHT2 * nf + (1 - HVQ_NF_WEIGHT2) * d; */
                    Mpy_32_16_ss(L_d, HVQ_NF_WEIGHT2B, &L_tmp, &lsb);     /* 12+15-15=12 */
                    Mpy_32_16_ss(L_nf, HVQ_NF_WEIGHT2_FX, &L_nf, &lsb);     /* 12+15-15=12 */
                    L_nf = L_add(L_nf, L_tmp);  /*Q12 */
                }

                IF ( L_sub(L_d, L_pe) > 0 )
                {
                    /*pe = HVQ_PE_WEIGHT1 * pe + (1 - HVQ_PE_WEIGHT1) * d; */
                    Mpy_32_16_ss(L_d, HVQ_PE_WEIGHT1B, &L_tmp, &lsb);     /* 12+15-15=12 */
                    Mpy_32_16_ss(L_pe, HVQ_PE_WEIGHT1_FX, &L_pe, &lsb);     /* 12+15-15=12 */
                    L_pe = L_add(L_pe, L_tmp);  /*Q12 */
                }
                ELSE
                {
                    /*pe = HVQ_PE_WEIGHT2 * pe + (1 - HVQ_PE_WEIGHT2) * d; */
                    Mpy_32_16_ss(L_d, HVQ_PE_WEIGHT2B, &L_tmp, &lsb);     /* 12+15-15=12 */
                    Mpy_32_16_ss(L_pe, HVQ_PE_WEIGHT2_FX, &L_pe, &lsb);     /* 12+15-15=12 */
                    L_pe = L_add(L_pe, L_tmp);  /*Q12 */
                }

                L_nf_mean[i] = L_add(L_nf_mean[i], L_nf);
                L_pe_mean[i] = L_add(L_pe_mean[i], L_pe);

                IF ( L_sub(L_d, L_peak) > 0 )
                {
                    L_peak = L_add(L_d, 0);
                }

                q = add(q, 1);
            }
            L_nf_mean[i] = L_shr(L_nf_mean[i], 5);    /* Divide by 5 (HVQ_BW) */
            L_pe_mean[i] = L_shr(L_pe_mean[i], 5);    /* Divide by 5 (HVQ_BW) */

            /*thr_tmp = (float)pow( pe_mean[i]/nf_mean[i], HVQ_THR_POW ) * nf_mean[i]; */
            exp1 = norm_l(L_nf_mean[i]);
            nf_mean_norm = extract_h(L_shl(L_nf_mean[i], exp1));      /* 12+s-16=s-4 */
            IF ( nf_mean_norm == 0 )
            {
                inv_nf_mean = 0;
            }
            ELSE
            {
                inv_nf_mean = div_s(1<<14, nf_mean_norm);               /* 15+14-s+4=33-s */
            }
            Mpy_32_16_ss(L_pe_mean[i], inv_nf_mean, &L_tmp, &lsb);    /*12+33-s-15=30-s */

            exp2 = norm_l(L_tmp);
            tmp = Log2_norm_lc(L_shl(L_tmp, exp2));     /* Q15 */
            exp2 = exp1 - exp2;                         /* Q0 */
            L_tmp = Mpy_32_16(exp2, tmp, 32767);        /* 1 in Q15. Q16 */
            Mpy_32_16_ss(L_tmp, 28836, &L_tmp, &lsb);   /* 16+15-15=16 */
            frac = L_Extract_lc(L_tmp, &tmp);           /* Q15 and Q0 */
            L_tmp = Pow2(14, frac);                     /* Q14 */
            L_tmp = L_shl(L_tmp, tmp);                  /* Q14 */

            Mpy_32_16_ss(L_tmp, nf_mean_norm, &L_tmp, &lsb);            /*14+s-4-15=s-5 */
            shift = sub(17, exp1);                                      /* 16-(s-5)=17-s */
            L_thr_tmp = L_shl(L_tmp, shift);                            /* Q16 */
            L_thr_tmp = L_add(L_thr_tmp, lshr(lsb, sub(16, shift)));    /*Q16 */

            set32_fx(&L_thr[k], L_thr_tmp, HVQ_BW);
            k = add(k, HVQ_BW);

            /*sharp[i] = peak/nf_mean[i]; */
            Mpy_32_16_ss(L_peak, inv_nf_mean, &L_tmp, &lsb);      /* 12+33-s-15=30-s */
            shift = sub(exp1, 8);
            sharp[i] = extract_h(L_shl(L_tmp, shift));          /* 30-s+s-8-16 -> Q6 */

            /*sharp_dist += (sharp[i]-HVQ_SHARP_THRES); */
            sharp_dist = add(sharp_dist, sub(sharp[i], HVQ_SHARP_THRES_FX));

            if ( sub(sharp[i], HVQ_SHARP_THRES_FX) > 0 )
            {
                num_sharp_bands = add(num_sharp_bands, 1);
            }
        }

        /* Estimate noise floor gains */
        offset = s_and(nsub, 1);
        FOR ( i = 0; i < s_and(nsub, (Word16)0xFFFE); i++ )
        {
            /*(2*i+1)/nsub */
            idx = mult(add(shl(i, 1), 1), add(inv_nsub, 1));            /*0+15-15 = 0 */
            L_nf_gains[idx] = L_add(L_nf_gains[idx], L_nf_mean[i+offset]);
            L_pe_gains[idx] = L_add(L_pe_gains[idx], L_pe_mean[i+offset]);
        }

        FOR ( i = 0; i < HVQ_NF_GROUPS; i++ )
        {
            Mpy_32_16_ss(L_nf_gains[i], inv_gains_nsub, &L_nf_gains[i], &lsb);       /*12+15-15=12 */
            Mpy_32_16_ss(L_pe_gains[i], inv_gains_nsub, &L_pe_gains[i], &lsb);       /*12+15-15=12 */
        }

        /* Allocate available peaks */
        FOR ( i = 0; i < nsub; i++ )
        {
            avail_peaks[i] = HVQ_PA_PEAKS_SHARP1;
            move16();
            idx = mult(add(shl(i, 1), 1), add(inv_nsub, 1));            /*0+15-15 = 0 */
            Mpy_32_16_ss(L_nf_gains[idx], HVQ_PA_FAC_FX, &L_tmp, &lsb);   /* 12+15-15 -> Q12 */
            IF( L_sub(L_nf_mean[i], L_tmp) < 0 )
            {
                IF ( sub(sharp[i], HVQ_PA_SHARP_THRES3_FX) < 0 )
                {
                    avail_peaks[i] = HVQ_PA_PEAKS_SHARP3;
                    move16();
                }
                ELSE IF( sub(sharp[i], HVQ_PA_SHARP_THRES2_FX) < 0 )
                {
                    avail_peaks[i] = HVQ_PA_PEAKS_SHARP2;
                    move16();
                }
            }
        }



        /* Adjust threshold around previous peaks */
        FOR ( i = 0; i < *prev_Npeaks; i++ )
        {
            j = sub(prev_peaks[i], 2);
            k = add(prev_peaks[i], 2);
            p_adj = hvq_thr_adj_fx;

            FOR( q = j; q < k; q++ )
            {
                Mpy_32_16_ss(L_thr[q], *p_adj++, &L_thr[q], &lsb);  /* 12+15-15=12 */
                move32();
            }
        }

        num_peak_cands = 0;
        move16();

        /* Remove everything below threshold for peak search */
        L_input_abs[0] = L_deposit_l(0);
        L_input_abs[1] = L_deposit_l(0);
        L_input_abs[N-2] = L_deposit_l(0);
        L_input_abs[N-1] = L_deposit_l(0);
        FOR ( i = 0; i < N-2; i++ )
        {
            IF ( L_sub(L_input_abs[i], L_thr[i]) < 0 )
            {
                L_input_abs[i] = L_deposit_l(0);
            }
            ELSE
            {
                L_input_abs[num_peak_cands] = L_input_abs[i];
                move32();
                peak_cand_idx[num_peak_cands] = i;
                move16();
                num_peak_cands = add(num_peak_cands, 1);
            }
        }

        IF ( L_sub(L_core_brate, HQ_24k40) == 0 )
        {
            peak_th = HVQ_MAX_PEAKS_24k_CLAS;
            move16();
        }
        ELSE
        {
            peak_th = HVQ_MAX_PEAKS_32k;
            move16();
        }

        /* Find peaks */
        pindx = maximum_32_fx(L_input_abs, num_peak_cands, &L_m);
        i = 0;
        move16();

        WHILE ( L_m > 0 && sub(i, peak_th+1) < 0)
        {
            idx = mult(peak_cand_idx[pindx], INV_HVQ_BW);       /* 0+15-15=0 */
            IF ( avail_peaks[idx] > 0 )
            {
                peaks[i++] = peak_cand_idx[pindx];
                avail_peaks[idx]--;
            }

            j = sub(pindx, 2);
            k = add(pindx, 2);

            if ( j < 0 )
            {
                j = 0;
                move16();
            }

            tmp = sub(num_peak_cands, 1);
            if ( sub(k, tmp) > 0 )
            {
                k = tmp;
                move16();
            }

            low = sub(peak_cand_idx[pindx], 2);
            high = add(peak_cand_idx[pindx], 2);

            if ( low < 0 )
            {
                low = 0;
                move16();
            }

            tmp = sub(N, 1);
            if ( sub(high, tmp) > 0 )
            {
                high = tmp;
                move16();
            }

            FOR( q = j; q <= pindx; q++ )
            {
                IF( sub(peak_cand_idx[q], low) >= 0 )
                {
                    peak_cand_idx[q] = 0;
                    move16();
                    L_input_abs[q] = 0;
                    move16();
                }
            }

            FOR( q = pindx + 1; q <= k; q++ )
            {
                IF ( sub(peak_cand_idx[q], high) <= 0 )
                {
                    peak_cand_idx[q] = 0;
                    move16();
                    L_input_abs[q] = 0;
                    move16();
                }
            }

            pindx = maximum_32_fx(L_input_abs, num_peak_cands, &L_m);
        }

        *Npeaks = i;
        move16();
        IF ( sub(*Npeaks, HVQ_MIN_PEAKS) > 0 )
        {
            test();
            IF ( sub(num_sharp_bands, sub(nsub, 3)) > 0 && sub(*Npeaks, peak_th) <= 0 )
            {
                sharp_dist = mult(sharp_dist, inv_nsub);    /*x+15-15=x */
                test();
                IF ( sub(sharp_dist, SHARP_DIST_THRES_FX) <= 0 && *hvq_hangover < 0 )
                {
                    *hvq_hangover = add(*hvq_hangover, 1);
                }
                ELSE
                {
                    *hqswb_clas = HQ_HVQ;
                    move16();
                    *hvq_hangover = 2;
                    move16();
                }

                /* update memory */
                *prev_Npeaks = *Npeaks;
                move16();
                Copy( peaks, prev_peaks, *Npeaks );
            }
            ELSE
            {
                IF( *hvq_hangover > 0 )
                {
                    *hqswb_clas = HQ_HVQ;
                    move16();
                    *hvq_hangover = sub(*hvq_hangover, 1);
                    move16();
                }
                ELSE
                {
                    *hvq_hangover = -1;
                    move16();
                }
            }
        }
        ELSE
        {
            /* Zero peaks, likely silence input. */
            *hvq_hangover = -1;
            move16();
        }


        IF ( L_sub(L_core_brate, HQ_24k40) == 0 )
        {
            *Npeaks = s_min( HVQ_MAX_PEAKS_24k, *Npeaks );
            move16();
        }
        ELSE
        {
            *Npeaks = s_min( HVQ_MAX_PEAKS_32k, *Npeaks );
            move16();
        }
    }
    ELSE
    {
        *prev_Npeaks = 0;
        move16();
        *hvq_hangover = 0;
        move16();
    }


    return;
}


