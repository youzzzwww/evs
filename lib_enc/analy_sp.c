/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include "rom_enc_fx.h"
#include <assert.h>


static void find_enr( Word16 data[], Word32 band[], Word32 *ptE, Word32 *LEtot, const Word16 min_band, const Word16 max_band,
                      const Word16 Q_new2, const Word32 e_min, Word32 *Bin_E, Word16 BIN_FREQ_FX, Word32 *band_energies );

/*-------------------------------------------------------------------*
 * analy_sp()
 *
 * Spectral analysis of 12.8kHz input
 *-------------------------------------------------------------------*/

void analy_sp(
    Word16 *speech,       /* i  : speech buffer                          Q_new - preemph_bits */
    const Word16 Q_new,         /* i  : current scaling exp                    Q0                 */
    Word32 *fr_bands,     /* o  : energy in critical frequency bands     Q_new + QSCALE    */
    Word32 *lf_E,         /* o  : per bin E for first...                 Q_new + QSCALE - 2*/
    Word16 *Etot,         /* o  : total input energy                     Q8                 */
    const Word16 min_band,      /* i  : minimum critical band                  Q0                 */
    const Word16 max_band,      /* i  : maximum critical band                  Q0                 */
    const Word32 e_min_scaled,  /* i  : minimum energy scaled                  Q_new + QSCALE    */
    Word16 Scale_fac[2],  /* o  : FFT scales factors (2 values by frame) Q0                 */
    Word32 *Bin_E,        /* o  : per-bin energy spectrum                                  */
    Word32 *Bin_E_old,    /* o  : per-bin energy spectrum of the previous frame            */
    Word32 *PS,           /* o  : per-bin energy spectrum                                  */
    Word16 *EspecdB,      /* o  : per-bin log energy spectrum (with f=0) Q7                */
    Word32 *band_energies,/* o  : energy in critical frequency bands without minimum noise floor MODE2_E_MIN */
    Word16 *fft_buff      /* o  : FFT coefficients                                         */
)
{
    Word16 *pt;
    Word16 i_subfr, i, exp_etot, frac_etot, exp, exp_frac, exp2;
    Word32 *pt_bands;
    Word32 Ltmp, LEtot, L_tmp2;
    Word16 *pt_fft;
    Word16 Min_val, Max_val;
    Word16 Scale_fac2;
    Word16 fft_temp[L_FFT];


    /*-----------------------------------------------------------------*
     * Compute spectrum
     * find energy per critical frequency band and total energy in dB
     *-----------------------------------------------------------------*/

    pt_bands = fr_bands;
    pt_fft = fft_buff;
    LEtot = L_deposit_l(0);

    FOR (i_subfr=0; i_subfr <= 1; i_subfr++)
    {
        pt = speech + 3*(L_SUBFR/2) - L_FFT/2;
        if(i_subfr != 0)
        {
            pt = speech + 7*(L_SUBFR/2) - L_FFT/2;
        }

        /* Clear 1st value of 1st part, copy 1st value of 2nd part */
        fft_temp[0] = 0;
        move16();
        fft_temp[L_FFT/2] = pt[L_FFT/2];
        move16();
        Max_val = s_max(fft_temp[0],fft_temp[L_FFT/2]);
        Min_val = s_min(fft_temp[0],fft_temp[L_FFT/2]);

        FOR (i=1; i<L_FFT/2; i++)
        {
            /* 1st windowed part */
            fft_temp[i] = mult_r(pt[i], sqrt_han_window[i]);
            move16();
            if (fft_temp[i] > 0)
                Max_val = s_max(Max_val, fft_temp[i]);
            if (fft_temp[i] < 0)
                Min_val = s_min(Min_val, fft_temp[i]);

            /* 2nd windowed part  */
            fft_temp[L_FFT-i] = mult_r(pt[L_FFT-i], sqrt_han_window[i]);
            move16();
            if (fft_temp[L_FFT-i] > 0)
                Max_val = s_max(Max_val, fft_temp[L_FFT-i]);
            if (fft_temp[L_FFT-i] < 0)
                Min_val = s_min(Min_val, fft_temp[L_FFT-i]);
        }

        /* Combine -Min_val and Max_val into one */
        Max_val = s_max(negate(Min_val),Max_val);

        Scale_fac[i_subfr] = s_min(sub(norm_s(Max_val), 1), 6);
        move16();
        Scale_fac2 = shl(Scale_fac[i_subfr], 1);
        Scale_sig(fft_temp, L_FRAME_12k8, Scale_fac[i_subfr]);

        r_fft_fx_lc(FFT_W128, SIZE_256, SIZE2_256, NUM_STAGE_256, fft_temp, pt_fft, 1);

        /* find energy per critical band */
        find_enr( pt_fft, pt_bands, lf_E + i_subfr*VOIC_BINS, &LEtot, min_band, max_band,
                  add(Q_new,Scale_fac2), e_min_scaled, &Bin_E[i_subfr*L_FFT/2], BIN, band_energies + i_subfr*NB_BANDS);

        pt_bands += NB_BANDS;
        pt_fft += L_FFT;
    }

    /* Average total log energy over both half-frames */
    frac_etot = 0;
    move16();
    exp_etot  = norm_l(LEtot);
    IF (LEtot != 0) /* Log2_norm_lc doesn't Support Input <= 0; deal with it here */
    {
        frac_etot = Log2_norm_lc(L_shl(LEtot, exp_etot));
        exp_etot = sub(30, exp_etot);
    }
    exp_etot = sub(exp_etot, add(Q_new, 1+3)); /* remove scaling effect, 4 already removed */
    Ltmp = Mpy_32(exp_etot, frac_etot, LG10, 0);

    /*Q8 Averaged the total energy over both half-frames in log10  */
    *Etot = extract_l(L_shr(Ltmp, 14-8));

    Bin_E[L_FFT/2-1] = Bin_E[L_FFT/2-2];
    move32();
    Bin_E[L_FFT-1] = Bin_E[L_FFT-2];
    move32();

    /* Per-bin log-energy spectrum */
    exp2 = sub(31, add(Q_new, QSCALE-2));
    L_tmp2 = L_mac(-56783L, exp2, 28391);
    FOR (i = 0; i < L_FFT/2; i++)
    {
        Bin_E_old[i] = Bin_E[i];
        move32();

        /* tmp = (input[i] + input[i+Len]+0.001f)/2.0f */
        Ltmp = L_max(1, L_add(L_shr(Bin_E[i],1), L_shr(Bin_E[i+L_FFT/2],1)));
        if(PS != NULL)
        {
            PS[i] = Ltmp;
            move32();
        }

        /* 10.0*log((float)tmp)                                  */
        /* 10.0*Log(2)*Log2(L_tmp/2)                             */
        /* 6.93147*Log2(L_tmp/2)                                 */
        /* 0.86643*(Log2(L_tmp)-1)*8                             */
        /* 0.86643*Log2(L_tmp)*8 - 6.93147                       */
        /* We'll put it in Q8                                    */
        exp = norm_l(Ltmp);
        exp_frac = Log2_norm_lc(L_shl(Ltmp, exp));
        /* -56783L is to substract 0.86643 in Q16                */
        /* 28391 is 0.86643 in Q15                               */
        /* 1774 is (0.86643 in Q15) * 8 / 128 (/128 to go in Q7) */
        EspecdB[i] = mac_r(L_shl(L_msu(L_tmp2, exp, 28391), 3+7), exp_frac, 887);
        move16();
    }


    return;
}

/*------------------------------------------------------------------------*
 * find_enr()
 *
 * find input signal energy for each critical band and first 74 LF bins
 * The energy is normalized by the number of frequency bins in a channel
 *------------------------------------------------------------------------*/

static void find_enr(
    Word16 data[],        /* i  : fft result                                                  */
    Word32 band[],        /* o  : per band energy                           Q_new + QSCALE   */
    Word32 *ptE,          /* o  : per bin energy  for low frequencies       Q_new + QSCALE-2 */
    Word32 *LEtot,        /* o  : total energy                              Q_new + QSCALE   */
    const Word16 min_band,      /* i  : minimum critical band                     Q0                */
    const Word16 max_band,      /* i  : maximum critical band                     Q0                */
    const Word16 Q_new2,        /* i  : scaling factor                            Q0                */
    const Word32 e_min,         /* i  : minimum energy scaled                     Q_new + QSCALE   */
    Word32 *Bin_E,        /* o  : Per bin energy                            Q_new + QSCALE-2 */
    Word16 BIN_FREQ_FX,   /* i  : Number of frequency bins                                    */
    Word32 *band_energies /* o  : per band energy without MODE2_E_MIN        */
)
{
    Word16 i, cnt, shift_to_norm;
    Word16 freq, wtmp;
    Word16 *ptR, *ptI, diff_scaleP1, diff_scaleM2;
    Word16 exp_band;
    Word32 Ltmp, Ltmp1;
    Word16 voic_band;
    Word32 etot;
    Word16 exp_etot;
    Word32 *tmpptr;


    ptR = &data[1];        /* first real */
    ptI = &data[L_FFT-1];  /* first imaginary */

    /*-----------------------------------------------------------------------------------*
     * Scaling needed by band and ptE output
     * Wants all energies scaled by Q_new + QSCALE to maintain maximum
     * precision on bckr noise in clean speech
     * First shift left by Q_new + QSCALE than shift right by 2*Q_new-1
     * shift left (Q_new + QSCALE - (2*Q_new -1))
     * shift left (QSCALE - Q_new + 1) == shift left by (QSCALE+1) - Q_new
     *-----------------------------------------------------------------------------------*/

    diff_scaleP1 = sub(QSCALE+1 + 1, Q_new2);
    diff_scaleM2 = sub(QSCALE+1 - 2, Q_new2);

    voic_band = VOIC_BAND_8k;
    move16();
    assert (VOIC_BAND == VOIC_BAND_8k);

    etot = L_deposit_l(0);
    exp_etot = 0;
    move16();

    /*-----------------------------------------------------------------*
     * For low frequency bins, save per bin energy for the use
     * in NS and find_tilt()
     *-----------------------------------------------------------------*/

    freq = BIN_FREQ_FX;
    move16();
    FOR (i = 0; i < voic_band; i++)      /* up to maximum allowed voiced critical band  */
    {
        tmpptr = Bin_E;
        move16();
        Ltmp1 = L_deposit_l(0);

        FOR (; freq <= crit_bands[i]; freq += BIN_FREQ_FX)
        {
            /*ptE = *ptR * *ptR + *ptI * *ptI */      /* energy */
            Ltmp = L_mult(*ptI, *ptI);
            Ltmp = L_mac(Ltmp, *ptR, *ptR);

            /* *ptE *= 4.0 / (L_FFT*L_FFT) */
            /* normalization - corresponds to FFT normalization by 2/L_FFT */
            BASOP_SATURATE_WARNING_OFF; /* saturation seems to have no effect (tested by simulation) */
            *ptE = L_shl(Ltmp, diff_scaleM2);
            move32(); /* scaled by Q_new + QSCALE - 2 */
            BASOP_SATURATE_WARNING_ON;
            /*band[i] += *ptE++;*/
            *Bin_E  = *ptE;
            move32();
            Bin_E++;
            Ltmp1 = L_add(Ltmp1, Ltmp);

            ptE++;
            ptR++;
            ptI--;
        }

        exp_band = sub(norm_l(Ltmp1), 1);/* divide by 2 to ensure band < cnt */
        wtmp = round_fx(L_shl(Ltmp1, exp_band));

        /* band[i] /= cnt */             /* normalization per frequency bin */
        cnt = (Word16)(Bin_E - tmpptr);
        shift_to_norm = norm_s(cnt);
        wtmp = div_s(wtmp, shl(cnt, shift_to_norm));
        Ltmp1 = L_deposit_l(wtmp);

        exp_band = sub(exp_band, shift_to_norm);
        exp_band = sub(diff_scaleP1, exp_band);
        BASOP_SATURATE_WARNING_OFF; /* saturation seems to have no effect (tested by simulation) */
        band[i] = L_shl(Ltmp1, exp_band);
        move32();/* band scaled by Q_new + QSCALE */
        BASOP_SATURATE_WARNING_ON;

        test();
        IF (sub(i,min_band) >= 0 && sub(i,max_band) <= 0)
        {
            IF (L_sub(band[i],e_min) < 0)
            {
                Ltmp1 = L_shl(e_min, 0);
                exp_band = 0;
                move16();
            }

            wtmp = sub(exp_band, exp_etot);
            if (wtmp > 0)
            {
                etot = L_shr(etot, wtmp);
            }
            exp_etot = s_max(exp_etot, exp_band);
            etot = L_add(etot, L_shl(Ltmp1, sub(exp_band, exp_etot)));
        }

        band_energies[i] = band[i];
        move32();

        band[i] = L_max(band[i], e_min);
        move32();
    }

    IF (sub(BIN_FREQ_FX, 50) == 0)
    {
        /*-----------------------------------------------------------------*
         * Continue compute the E per critical band for high frequencies
         *-----------------------------------------------------------------*/

        FOR (i = voic_band; i < NB_BANDS; i++)
        {
            tmpptr = Bin_E;
            move16();
            Ltmp1 = L_deposit_l(0);

            FOR (; freq <= crit_bands[i]; freq += BIN_FREQ_FX)
            {
                /* *ptE = *ptR * *ptR + *ptI * *ptI */
                Ltmp = L_mult(*ptI, *ptI);
                Ltmp = L_mac(Ltmp, *ptR, *ptR);

                /* *ptE *= 4.0 / (L_FFT*L_FFT) */
                /* normalization - corresponds to FFT normalization by 2/L_FFT  */
                BASOP_SATURATE_WARNING_OFF; /* saturation seems to have no effect (tested by simulation) */
                *Bin_E = L_shl(Ltmp, diff_scaleM2);
                move32();   /* scaled by Q_new + QSCALE - 2 */
                BASOP_SATURATE_WARNING_ON;
                Bin_E++;
                Ltmp1  = L_add(Ltmp1, Ltmp);

                ptR++;
                ptI--;
            }

            exp_band = sub(norm_l(Ltmp1), 1);            /* divide by 2 to ensure band < cnt */
            wtmp = round_fx(L_shl(Ltmp1, exp_band));

            /* band[i] /= cnt */             /* normalization per frequency bin */
            cnt = (Word16)(Bin_E - tmpptr);
            shift_to_norm = norm_s(cnt);
            wtmp = div_s(wtmp, shl(cnt, shift_to_norm));
            Ltmp1 = L_deposit_l(wtmp);

            exp_band = sub(exp_band, shift_to_norm);
            exp_band = sub(diff_scaleP1, exp_band);
            BASOP_SATURATE_WARNING_OFF; /* saturation seems to have no effect (tested by simulation) */
            band[i] = L_shl(Ltmp1, exp_band);
            move32();/* band scaled by Q_new + QSCALE */
            BASOP_SATURATE_WARNING_ON;

            test();
            IF (sub(i,min_band) >= 0 && sub(i,max_band) <= 0)
            {
                IF (L_sub(band[i],e_min) < 0)
                {
                    Ltmp1 = L_shl(e_min, 0);
                    exp_band = 0;
                    move16();
                }

                wtmp = sub(exp_band, exp_etot);
                if (wtmp > 0)
                {
                    etot = L_shr(etot, wtmp);
                }
                exp_etot = s_max(exp_etot, exp_band);

                etot = L_add(etot, L_shl(Ltmp1, sub(exp_band, exp_etot)));
            }

            band_energies[i] = band[i];
            move32();

            band[i] = L_max(band[i], e_min);
            move32();
        }
    }

    /*-----------------------------------------------------------------*
     * Find the total energy over the input bandwidth
     *-----------------------------------------------------------------*/

    etot = L_add(*LEtot, L_shl(etot, sub(exp_etot, 4)));

    *LEtot = etot;
    move32();


    return;
}
