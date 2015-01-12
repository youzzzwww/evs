/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"            /* required for wmc_tool */

/*--------------------------------------------------------------------------*
 * fill_spectrum()
 *
 * Apply spectral filling by
 * - filling zero-bit bands below BWE region
 * - applying BWE above transition frequency
 *--------------------------------------------------------------------------*/

void fill_spectrum_fx(
    Word16 *coeff,               /* i/o: normalized MLT spectrum / nf spectrum                Q12 */
    Word32 *L_coeff_out,         /* i/o: Noisefilled MLT spectrum                             Q12 */
    const Word16 *R,                   /* i  : number of pulses per band                            Q0  */
    const Word16 is_transient,         /* i  : transient flag                                       Q0  */
    Word16 norm[],               /* i  : quantization indices for norms                       Q0  */
    const Word16 *hq_generic_fenv,     /* i  : HQ GENERIC envelope                                  Q1  */
    const Word16 hq_generic_offset,    /* i  : HQ GENERIC offset                                    Q0  */
    const Word16 nf_idx,               /* i  : noise fill index                                     Q0  */
    const Word16 length,               /* i  : Length of spectrum (32 or 48 kHz)                    Q0  */
    const Word16 env_stab,             /* i  : Envelope stability measure [0..1]                    Q15 */
    Word16 *no_att_hangover,     /* i/o: Frame counter for attenuation hangover               Q0  */
    Word32 *L_energy_lt,         /* i/o: Long-term energy measure for transient detection     Q13 */
    Word16 *bwe_seed,            /* i/o: random seed for generating BWE input                 Q0  */
    const Word16 hq_generic_exc_clas,  /* i  : BWE excitation class                                 Q0  */
    const Word16 core_sfm,             /* i  : index of the end band for core                       Q0  */
    const Word16 HQ_mode,              /* i  : HQ mode                                              Q0  */
    Word16 noise_level[],        /* i  : noise levels for harmonic modes                      Q15 */
    const Word32 L_core_brate,         /* i  : target bit-rate                                      Q0  */
    Word16 prev_noise_level[],   /* i/o: noise factor in previous frame                       Q15 */
    Word16 *prev_R,              /* i/o: bit allocation info. in previous frame               Q0  */
    Word32 *prev_coeff_out,      /* i/o: decoded spectrum in previous frame                   Q12 */
    const Word16 *peak_idx,            /* i  : peak indices for hvq                                 Q0  */
    const Word16 Npeaks,               /* i  : number of peaks in hvq                               Q0  */
    const Word16 *npulses,             /* i  : number of pulses per band                            Q0  */
    const Word16 prev_is_transient,    /* i  : previous transient flag                              Q0  */
    Word32 *prev_normq,          /* i/o: previous norms                                       Q14 */
    Word32 *prev_env,            /* i/o: previous noise envelopes                             Q(prev_env_Q) */
    const Word16 prev_bfi,             /* i  : previous bad frame indicator                         Q0  */
    const Word16 *sfmsize,             /* i  : Length of bands                                      Q0  */
    const Word16 *sfm_start,           /* i  : Start of bands                                       Q0  */
    const Word16 *sfm_end,             /* i  : End of bands                                         Q0  */
    Word16 *prev_L_swb_norm,     /* i/o: HVQ/Harmonic mode normalization length               Q0  */
    const Word16 prev_hq_mode,         /* i  : Previous HQ mode                                     Q0  */
    const Word16 num_sfm               /* i  : Total number of bands                                Q0  */
    ,Word16 *prev_env_Q
    ,const Word16 num_env_bands         /* i  : Number sub bands to be encoded for HQ_GEN            Q0  */
)
{
    Word16 CodeBook[FREQ_LENGTH];       /* Q12 */
    Word16 cb_size;
    Word16 last_sfm;
    Word16 CodeBook_mod[FREQ_LENGTH];   /*Q12 */
    Word16 norm_adj[NB_SFM];            /*Q15 */
    Word16 high_sfm;
    Word16 flag_32K_env_hangover;
    Word16 bin_th;
    Word16 peak_pos[L_HARMONIC_EXC];
    Word16 bwe_peaks[L_FRAME48k];
    Word32 L_normq_v[NB_SFM];           /*Q14 */
    Word16 coeff_fine[L_FRAME48k];      /*Q15 */
    Word32 L_coeff_out1[L_FRAME48k];    /*Q12 */

    set16_fx( peak_pos, 0, L_HARMONIC_EXC );
    set16_fx( bwe_peaks, 0, L_FRAME48k );
    set16_fx(norm_adj, 32767, num_sfm);    /* 1.0, Q15 */
    cb_size = 0;
    move16();
    bin_th = 0;
    move16();
    high_sfm = 23;
    move16();

    test();
    IF ( sub(HQ_mode, HQ_TRANSIENT) == 0 )
    {
        last_sfm = sub(num_sfm, 1);
    }
    ELSE IF ( sub(HQ_mode,HQ_GEN_SWB) == 0 || sub(HQ_mode,HQ_GEN_FB) == 0 )
    {
        last_sfm = s_max(core_sfm,sub(num_env_bands,1));
    }
    ELSE
    {
        last_sfm = core_sfm;
        move16();
    }

    IF ( sub(HQ_mode, HQ_HARMONIC) == 0 )
    {
        /*high_sfm = (core_brate == HQ_24k40) ? HVQ_THRES_SFM_24k-1 : HVQ_THRES_SFM_32k-3; */
        high_sfm = sub(HVQ_THRES_SFM_32k, 1);
        if (L_sub(L_core_brate, HQ_24k40) == 0)
        {
            high_sfm = sub(HVQ_THRES_SFM_24k, 1);
        }

        if( sub(last_sfm, high_sfm) < 0 )
        {
            last_sfm = high_sfm;
            move16();
        }
    }
    ELSE if ( sub(HQ_mode, HQ_HVQ) == 0 )
    {
        bin_th = sfm_end[last_sfm];
        move16();
    }

    /* Transient analysis for envelope stability measure */
    IF ( sub(length, L_FRAME32k) == 0 )
    {
        env_stab_transient_detect_fx( is_transient, length, norm, no_att_hangover, L_energy_lt, HQ_mode, bin_th, L_coeff_out, 12 );
    }

    test();
    test();
    test();
    test();
    IF (  sub(length, L_FRAME16k) == 0 ||
          ((sub(length, L_FRAME32k) == 0 && sub(HQ_mode, HQ_HARMONIC) != 0 && sub(HQ_mode, HQ_HVQ) != 0) && *no_att_hangover == 0) )
    {
        /* Norm adjustment function */
        env_adj_fx( npulses, length, last_sfm, norm_adj, env_stab, sfmsize );
    }

    /*flag_32K_env_hangover = ( length == L_FRAME32k && ( (env_stab < 0.5f && *no_att_hangover == 0) || HQ_mode == HQ_HVQ ) );   */
    flag_32K_env_hangover = 0;
    move16();
    test();
    test();
    test();
    if ( sub(length, L_FRAME32k) == 0 && ( (sub(env_stab, 16384) < 0 && *no_att_hangover == 0) || sub(HQ_mode, HQ_HVQ) == 0 ) )
    {
        flag_32K_env_hangover = 1;
        move16();
    }


    /*----------------------------------------------------------------*
     * Build noise-fill codebook
     *----------------------------------------------------------------*/

    IF ( sub(HQ_mode, HQ_HVQ) != 0 )
    {
        cb_size = build_nf_codebook_fx(flag_32K_env_hangover, coeff, sfm_start, sfmsize, sfm_end, last_sfm, R, CodeBook, CodeBook_mod);
    }
    /*----------------------------------------------------------------*
     * Prepare fine structure for Harmonic and HVQ
     *----------------------------------------------------------------*/

    IF ( sub(HQ_mode, HQ_HARMONIC) == 0 )
    {
        harm_bwe_fine_fx( R, last_sfm, high_sfm, num_sfm, norm, sfm_start, sfm_end, prev_L_swb_norm, coeff, L_coeff_out, coeff_fine );
    }
    ELSE IF ( sub(HQ_mode, HQ_HVQ) == 0 )
    {
        hvq_bwe_fine_fx( last_sfm, num_sfm, sfm_end, peak_idx, Npeaks, peak_pos, prev_L_swb_norm, L_coeff_out, bwe_peaks, coeff_fine );
    }

    /*----------------------------------------------------------------*
     * Apply noise-fill
     *----------------------------------------------------------------*/

    IF ( sub(HQ_mode, HQ_HVQ) != 0 )
    {
        apply_noisefill_HQ_fx( R, length, flag_32K_env_hangover, L_core_brate, last_sfm, CodeBook,
                               CodeBook_mod, cb_size, sfm_start, sfm_end, sfmsize, coeff );
    }

    /*----------------------------------------------------------------*
     * Normal mode BWE
     *----------------------------------------------------------------*/

    IF ( HQ_mode == HQ_NORMAL )
    {
        hq_fold_bwe_fx(last_sfm, sfm_end, num_sfm, coeff);
    }

    /*----------------------------------------------------------------*
     * Apply noise-fill adjustment
     *----------------------------------------------------------------*/

    test();
    test();
    test();
    IF( (sub(length, L_FRAME32k) >= 0 || L_sub(L_core_brate, HQ_32k) > 0 || L_sub(L_core_brate, HQ_24k40) < 0)
        && sub(HQ_mode, HQ_HVQ) != 0 )
    {
        apply_nf_gain_fx(nf_idx, last_sfm, R, sfm_start, sfm_end, coeff);
    }

    /*----------------------------------------------------------------*
     * Prepare fine strucutre for HQ GENERIC
     *----------------------------------------------------------------*/
    test();
    IF ( sub(HQ_mode, HQ_GEN_SWB) == 0 || sub(HQ_mode, HQ_GEN_FB) == 0 )
    {
        hq_generic_fine_fx( coeff, last_sfm, sfm_start, sfm_end, bwe_seed, coeff_fine );
    }

    /*----------------------------------------------------------------*
     * Apply envelope
     *----------------------------------------------------------------*/

    test();
    IF ( sub(HQ_mode, HQ_HARMONIC) != 0 && sub(HQ_mode, HQ_HVQ) != 0 )
    {
        apply_envelope_fx( coeff, norm, norm_adj, num_sfm, last_sfm, HQ_mode, length, sfm_start, sfm_end,
                           L_normq_v, L_coeff_out, coeff_fine, L_coeff_out1 );
    }

    /*----------------------------------------------------------------*
     * Harmonic BWE, HVQ BWE and HQ SWB BWE
     *----------------------------------------------------------------*/
    test();
    IF ( sub(HQ_mode,  HQ_HARMONIC) == 0 )
    {
        harm_bwe_fx( coeff_fine, coeff, num_sfm, sfm_start, sfm_end, last_sfm, R, prev_hq_mode, norm, noise_level, prev_noise_level, bwe_seed, L_coeff_out );
    }
    ELSE IF ( sub(HQ_mode, HQ_HVQ) == 0 )
    {
        hvq_bwe_fx( L_coeff_out, coeff_fine, sfm_start, sfm_end, sfmsize, last_sfm,
                    prev_hq_mode, bwe_peaks, bin_th, num_sfm, L_core_brate, R, norm,
                    noise_level, prev_noise_level, bwe_seed, L_coeff_out, 15, 12 );
    }
    ELSE IF ( sub(HQ_mode, HQ_GEN_SWB) == 0 || sub(HQ_mode, HQ_GEN_FB) == 0 )
    {
        hq_bwe_fx( HQ_mode, L_coeff_out1, hq_generic_fenv, L_coeff_out, hq_generic_offset, prev_L_swb_norm, hq_generic_exc_clas, sfm_end, num_sfm, num_env_bands, R );
    }

    /*----------------------------------------------------------------*
     * HQ WB BWE refinements
     *----------------------------------------------------------------*/
    test();
    IF ( sub(length, L_FRAME16k) == 0 && L_sub(L_core_brate, HQ_32k) == 0 )
    {
        hq_wb_nf_bwe_fx( coeff, is_transient, prev_bfi, L_normq_v, num_sfm, sfm_start, sfm_end, sfmsize, last_sfm, R,
                         prev_is_transient, prev_normq, prev_env, bwe_seed, prev_coeff_out, prev_R, L_coeff_out, prev_env_Q );
    }

    /*----------------------------------------------------------------*
     * Update memories
     *----------------------------------------------------------------*/

    test();
    IF ( sub(HQ_mode, HQ_HARMONIC) != 0 && sub(HQ_mode, HQ_HVQ) != 0 )
    {
        prev_noise_level[0] = 3277;
        move16();/* 0.1 in Q15 */
        prev_noise_level[1] = 3277;
        move16();/* 0.1 in Q15 */
    }
    test();
    IF ( !(sub(length, L_FRAME16k) == 0 && L_sub(L_core_brate, HQ_32k) == 0) )
    {
        set32_fx( prev_env, 0, SFM_N_WB );
        set32_fx( prev_normq, 0, SFM_N_WB );
    }

    test();
    IF ( sub(length, L_FRAME32k) == 0 && L_sub(L_core_brate, HQ_32k) <= 0 )
    {
        Copy(R, prev_R, NB_SFM);
        Copy32(L_coeff_out, prev_coeff_out, 544);
    }

    return;
}
