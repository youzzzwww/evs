/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "stl.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include <assert.h>


#  define MDCT_SW_SIG_LINE_THR       840   /* 2.85f*LOG_10 in Q7 */  /* Significant spectral line threshold above Etot (dB)                               */
#  define MDCT_SW_SIG_PEAK_THR       9216  /* 36.0f        in Q8 */  /* Significant peak threshold below Etot (dB)                                        */
#  define MDCT_SW_HI_SPARSE_THR      8192  /* 0.25f        in Q15*/  /* Max. ratio of significant spectral lines for the spectrum to be considered sparse */
#  define MDCT_SW_HI_ENER_LO_THR     1920  /* 7.5f         in Q8 */  /* Hi band low energy threshold (dB)                                                 */
#  define MDCT_SW_SPARSE_THR         6554  /* 0.25f*0.8f   in Q15*/
#  define MDCT_SW_1_VOICING_THR      29491 /* 0.9f         in Q15*/  /* Voicing threshold                                                                 */
#  define MDCT_SW_1_VOICING_THR2     23593 /* 0.9f*0.8f    in Q15*/
#  define MDCT_SW_1_HI_ENER_LO_THR   3200  /* 12.5f        in Q8 */  /* Hi band high energy threshold (dB)                                                */
#  define MDCT_SW_1_SIG_HI_LEVEL_THR 7168  /* 28.0f        in Q8 */  /* High signal level threshold above noise floor (dB)                                */
#  define MDCT_SW_1_SIG_LO_LEVEL_THR 5760  /* 22.5f        in Q8 */  /* Low signal level threshold above noise floor (dB)                                 */
#  define MDCT_SW_1_COR_THR          20480 /* 80.0f        in Q8 */  /* Threshold on cor_map_sum to indicate strongly tonal signal                        */
#  define MDCT_SW_1_COR_THR2         16384 /* 80.0f*0.8f   in Q8 */
#  define MDCT_SW_1_SPARSENESS_THR   21299 /* 0.65f        in Q15*/  /* Threshold on spectrum sparseness                                                  */
#  define MDCT_SW_1_SPARSENESS_THR2  17039 /* 0.65f*0.8f   in Q15*/

#  define MDCT_SW_2_VOICING_THR      19661 /* 0.6f         in Q15*/  /* Voicing threshold                                                                 */
#  define MDCT_SW_2_VOICING_THR2     15729 /* 0.6f*0.8f    in Q15*/
#  define MDCT_SW_2_HI_ENER_LO_THR   2432  /* 9.5f         in Q8 */  /* Hi band low energy threshold (dB)                                                 */
#  define MDCT_SW_2_SIG_HI_LEVEL_THR 4864  /* 19.0f        in Q8 */  /* High signal level threshold above noise floor (dB)                                */
#  define MDCT_SW_2_SIG_LO_LEVEL_THR 6016  /* 23.5f        in Q8 */  /* Low signal level threshold above noise floor (dB)                                 */
#  define MDCT_SW_2_COR_THR          16000 /* 62.5f        in Q8 */  /* Threshold on cor_map_sum to indicate strongly tonal signal                        */
#  define MDCT_SW_2_COR_THR2         12800 /* 62.5f*0.8f   in Q8 */
#  define MDCT_SW_2_SPARSENESS_THR   13107 /* 0.4f         in Q15*/  /* Threshold on spectrum sparseness                                                  */
#  define MDCT_SW_2_SPARSENESS_THR2  10486 /* 0.4f*0.8f    in Q15*/

static Word16 get_sparseness( /* Returns sparseness measure (Q15) */
    const Word16 Bin_E[], /* i  : per bin energy dB        Q7 */
    Word16 n,             /* i  : number of bins           Q0 */
    Word16 thr            /* i  : peak threshold           Q8 */
)
{
    Word16 num_max, i;

    thr = add(thr, mult(thr, 4958)); /* Convert to 10*log() domain from 10*log10() domain, and also to Q7 */

    thr = s_max(thr, 384); /* 3.0 in Q7 */ /* Set an absolute minimum for close to silent signals */

    num_max = 0;
    move16();

    FOR (i=1; i<n-1; ++i)
    {
        if (sub(Bin_E[i], s_max(s_max(Bin_E[i-1], Bin_E[i+1]), thr)) > 0)
        {
            num_max = add(num_max, 1);
        }
    }

    n = shr(sub(n, 2), 1);
    return div_s(sub(n, num_max), n);
}

static Word16 get_mean_ener( /* Returns mean energy in dB (Q8) */
    const Word32 enerBuffer[],  /* i  : CLDFB buffers             */
    Word16 enerBuffer_exp,      /* i  : exponent of enerBuffer  */
    Word16 n                    /* i  : number of bins          */
)
{
    Word32 L_tmp;
    Word16 i, shift, frac_nrg, exp_nrg;

    shift = sub(14, norm_s(n));
    if (sub(shl(1, shift), n) < 0) shift = add(shift, 1);

    L_tmp = L_deposit_l(0);
    FOR (i=0; i<n; ++i)
    {
        L_tmp = L_add(L_tmp, L_shr(enerBuffer[i], shift));
    }
    L_tmp = Mult_32_16(L_tmp, div_s(1, n));

    /* Log energy */
    exp_nrg  = norm_l(L_tmp);
    frac_nrg = Log2_norm_lc(L_shl(L_tmp, exp_nrg));
    exp_nrg  = sub(30, exp_nrg);
    exp_nrg  = sub(add(exp_nrg, shift), sub(31, enerBuffer_exp));
    L_tmp    = Mpy_32_16(exp_nrg, frac_nrg, 9864); /* log10(2) in Q15 */

    return round_fx(L_shl(L_tmp, 8));
}

void MDCT_selector(
    Encoder_State_fx *st,      /* i/o: Encoder State           */
    Word16 sp_floor,           /* i  : Noise floor estimate Q7 */
    Word16 Etot,               /* i  : Total energy         Q8 */
    Word16 cor_map_sum,        /* i  : harmonicity factor   Q8 */
    const Word16 voicing[],    /* i  : voicing factors      Q15*/
    const Word32 enerBuffer[], /* i  : CLDFB buffers           */
    Word16 enerBuffer_exp,     /* i  : exponent of enerBuffer  */
    Word16 vadflag
)
{
    test();
    IF (sub(st->mdct_sw_enable, MODE1) == 0 || sub(st->mdct_sw_enable, MODE2) == 0)
    {
        Word16 hi_ener, frame_voicing, sparseness;
        Word16 peak_count;
        Word16 prefer_tcx, prefer_hq_core, switching_point, hi_sparse, sparse;
        Word16 lob_cldfb, hib_cldfb, lob_fft, hib_fft;
        Word16 i, tmp;
        Word16 sig_lo_level_thr, sig_hi_level_thr, cor_thr, cor_thr2, voicing_thr, voicing_thr2, sparseness_thr, sparseness_thr2, hi_ener_lo_thr;
        Word16 last_core;

        sp_floor = shl(sp_floor, 1); /* convert to Q8 */

        IF (sub(st->bwidth_fx, NB) == 0)
        {
            lob_cldfb = 3200/400;
            move16();
            hib_cldfb = 4000/400;
            move16();
            lob_fft = (L_FFT/2)/2; /* 3.2 KHz */                move16();
            hib_fft = (40*(L_FFT/2))/64; /* 4.0 KHz */          move16();
        }
        ELSE IF (sub(st->bwidth_fx, WB) == 0)
        {
            lob_cldfb = 4800/400;
            move16();
            hib_cldfb = 8000/400;
            move16();
            lob_fft = 3*L_FFT/2/4; /* 4.8 KHz */                move16();
            hib_fft = L_FFT/2; /* 6.4 KHz (should be 8 KHz) */  move16();
        }
        ELSE
        {
            lob_cldfb = 6400/400;
            move16();
            hib_cldfb = 16000/400;
            move16();
            if (sub(st->bwidth_fx, FB) == 0)
            {
                hib_cldfb = 24000/400;
                move16();
            }
            lob_fft = L_FFT/2; /* 6.4 KHz */                    move16();
            hib_fft = L_FFT/2; /* 6.4 KHz (should be 8 KHz) */  move16();
        }

        /* st->last_core_fx is reset to TCX_20_CORE in init_acelp() => fix it here */
        last_core = st->last_core_fx;
        move16();
        test();
        if (sub(st->last_codec_mode, MODE1) == 0 && sub(last_core, TCX_20_CORE) == 0)
        {
            last_core = HQ_CORE;
            move16();
        }

        /* Voicing */
        frame_voicing = add(shr(voicing[0], 1), shr(voicing[1], 1));

        /* Spectral sparseness */
        sparseness = get_sparseness(st->lgBin_E_fx, lob_fft, sub(Etot, MDCT_SW_SIG_PEAK_THR));

        /* Hi band energy */
        hi_ener = get_mean_ener(&enerBuffer[lob_cldfb], enerBuffer_exp, sub(hib_cldfb, lob_cldfb));

        /* Hi band sparseness */
        IF (sub(st->bwidth_fx, SWB) >= 0)
        {
            /* For SWB, assume hi band sparseness based on 4.8 KHz-6.4 KHz band */
            lob_fft = 3*L_FFT/2/4; /* 4.8 KHz */                move16();
        }
        peak_count = 0;
        move16();
        tmp = add(MDCT_SW_SIG_LINE_THR, shr(Etot, 1)); /* Q7 */
        FOR (i=lob_fft; i<hib_fft; ++i)
        {
            if (sub(st->lgBin_E_fx[i], tmp) >= 0)
            {
                peak_count = add(peak_count, 1);
            }
        }

        hi_sparse = 0;
        move16();
        if (sub(peak_count, mult_r(sub(hib_fft, lob_fft), MDCT_SW_HI_SPARSE_THR)) <= 0)
        {
            hi_sparse = 1;
            move16();
        }

        sparse = 0;
        move16();
        if (sub(peak_count, mult_r(sub(hib_fft, lob_fft), MDCT_SW_SPARSE_THR)) <= 0)
        {
            sparse = 1;
            move16();
        }

        /* Hysteresis */
        test();
        test();
        if (st->prev_hi_sparse > 0 && sparse > 0 && sub(s_min(s_min(voicing[0], voicing[1]), voicing[2]), MDCT_SW_1_VOICING_THR) >= 0)
        {
            hi_sparse = 1;
            move16();
        }

        /* Allowed switching point? */
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        switching_point = (sub(last_core, HQ_CORE) != 0 && sub(last_core, TCX_20_CORE) != 0) || /* previous core was non-MDCT */
                          (sub(st->prev_hi_ener, MDCT_SW_HI_ENER_LO_THR) <= 0 || sub(hi_ener, MDCT_SW_HI_ENER_LO_THR) <= 0) || /* hi band is close to silent */
                          (sub(last_core, HQ_CORE) == 0 && (sub(st->mdct_sw_enable, MODE1) == 0 || (hi_sparse > 0 && st->prev_hi_sparse >= 0 && sub(st->prev_hi_sparse, 1) <= 0))) || /* HQ_CORE and hi band became sparse */
                          (sub(last_core, TCX_20_CORE) == 0 && (hi_sparse == 0 && st->prev_hi_sparse > 0)); /* TCX and hi band became dense */

        IF (sub(st->mdct_sw_enable, MODE1) == 0)
        {
            sig_lo_level_thr = MDCT_SW_1_SIG_LO_LEVEL_THR;
            move16();
            sig_hi_level_thr = MDCT_SW_1_SIG_HI_LEVEL_THR;
            move16();
            cor_thr          = MDCT_SW_1_COR_THR;
            move16();
            cor_thr2         = MDCT_SW_1_COR_THR2;
            move16();
            voicing_thr      = MDCT_SW_1_VOICING_THR;
            move16();
            voicing_thr2     = MDCT_SW_1_VOICING_THR2;
            move16();
            sparseness_thr   = MDCT_SW_1_SPARSENESS_THR;
            move16();
            sparseness_thr2  = MDCT_SW_1_SPARSENESS_THR2;
            move16();
            hi_ener_lo_thr   = MDCT_SW_1_HI_ENER_LO_THR;
            move16();
        }
        ELSE   /* st->mdct_sw_enable == MODE2 */
        {
            sig_lo_level_thr = MDCT_SW_2_SIG_LO_LEVEL_THR;
            move16();
            sig_hi_level_thr = MDCT_SW_2_SIG_HI_LEVEL_THR;
            move16();
            cor_thr          = MDCT_SW_2_COR_THR;
            move16();
            cor_thr2         = MDCT_SW_2_COR_THR2;
            move16();
            voicing_thr      = MDCT_SW_2_VOICING_THR;
            move16();
            voicing_thr2     = MDCT_SW_2_VOICING_THR2;
            move16();
            sparseness_thr   = MDCT_SW_2_SPARSENESS_THR;
            move16();
            sparseness_thr2  = MDCT_SW_2_SPARSENESS_THR2;
            move16();
            hi_ener_lo_thr   = MDCT_SW_2_HI_ENER_LO_THR;
            move16();
        }

        test();
        test();
        test();
        test();
        test();
        prefer_tcx = (sub(sub(Etot, sp_floor), sig_hi_level_thr) >= 0) && /* noise floor is low */
                     (sub(cor_map_sum, cor_thr) >= 0 || sub(frame_voicing, voicing_thr) >= 0 || sub(sparseness, sparseness_thr) >= 0) && /* strong tonal components */
                     (sub(hi_ener, hi_ener_lo_thr) <= 0 || hi_sparse > 0); /* high freqs have low energy or are sparse */

        test();
        test();
        test();
        test();
        test();
        test();
        prefer_hq_core = (sub(sub(Etot, sp_floor), sig_lo_level_thr) < 0) || /* noise floor is very high */
                         (sub(cor_map_sum, cor_thr2) < 0 && sub(frame_voicing, voicing_thr2) < 0 && sub(sparseness, sparseness_thr2) < 0) || /* too weak tonal components */
                         (sub(st->mdct_sw_enable, MODE1) == 0 && prefer_tcx == 0 && sub(st->transientDetection.transientDetector.bIsAttackPresent, 1) == 0);

        /* Prefer HQ_CORE on transients */
        test();
        IF ( sub(st->mdct_sw_enable, MODE2) == 0 && sub(st->transientDetection.transientDetector.bIsAttackPresent, 1) == 0 )
        {
            prefer_tcx = 0;
            move16();
            prefer_hq_core = 1;
            move16();
        }

        test();
        test();
        test();
        IF (switching_point && (prefer_tcx || prefer_hq_core))
        {
            IF (prefer_tcx)
            {
                st->core_fx = TCX_20_CORE;
                move16();
            }
            ELSE /* prefer_hq_core */
            {
                st->core_fx = HQ_CORE;
                move16();
            }
        }
        ELSE IF (sub(last_core, HQ_CORE) == 0 || sub(last_core, TCX_20_CORE) == 0)
        {
            st->core_fx = last_core;
            move16();
        }

        test();
        test();
        test();
        /* Prevent the usage of HQ_CORE on noisy-speech or inactive */
        IF (sub(st->mdct_sw_enable, MODE2) == 0 && sub(st->core_fx, HQ_CORE) == 0 && (sub(st->flag_noisy_speech_snr, 1) == 0 || vadflag==0))
        {
            st->core_fx = TCX_20_CORE;
            move16();
        }

        /* Update memories */
        st->prev_hi_sparse = add(st->prev_hi_sparse, hi_sparse);
        move16();
        if (hi_sparse <= 0)
        {
            st->prev_hi_sparse = hi_sparse;
            move16();
        }
        st->prev_hi_sparse = s_min(st->prev_hi_sparse, 2);
        st->prev_hi_ener = hi_ener;
        move16();
    }
}

void MDCT_selector_reset(
    Encoder_State_fx *st          /* i/o: Encoder State */
)
{
    st->prev_hi_ener = 0;
    move16();
    st->prev_hi_sparse = -1;
    move16();
}
