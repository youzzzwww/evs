/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "stl.h"

/*--------------------------------------------------------------------------*
 * hq_hr_enc_fx()
 *
 * HQ High rate encoding routine
 *--------------------------------------------------------------------------*/
void hq_hr_enc_fx(
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure fx          */
    Word32 *t_audio,           /* i/o: transform-domain coefficients  Q12  */
    const Word16 length,             /* i  : length of spectrum             Q0   */
    Word16 *num_bits,          /* i  : number of available bits       Q0   */
    const Word16 is_transient        /* i  : transient flag                 Q0   */
)
{
    Word16 nb_sfm;                                /* Q0   */
    Word16 sum, hcode_l;                          /* Q0   */
    Word16 difidx[NB_SFM];                        /* Q0   */
    Word16 normqlg2[NB_SFM], ynrm[NB_SFM];        /* Q0   */
    Word16 nf_idx;                                /* Q0   */
    Word16 bits;                                  /*      */
    Word16 LCmode;                                /* Q0   */
    Word16 shape_bits, num_sfm, numnrmibits;      /* Q0   */
    Word16 hqswb_clas;                            /* Q0   */
    Word16 num_env_bands;                         /* Q0   */
    Word16 Npeaks, start_norm;                    /* Q0   */
    Word16 difidx_org[NB_SFM];                    /* Q0   */
    Word16 R[NB_SFM];                             /* Q0   */
    Word16 peaks[HVQ_MAX_PEAKS];                  /* Q0   */
    const Word16 *sfmsize, *sfm_start, *sfm_end;  /* Q0   */
    Word16 npulses[NB_SFM], maxpulse[NB_SFM];     /* Q0   */
    Word16 Rsubband[NB_SFM];                      /* Q3   */
    Word32 t_audio_q[L_FRAME48k];                 /* Q12  */
    Word32 nf_gains[HVQ_NF_GROUPS];               /* Q12  */
    Word32 pe_gains[HVQ_NF_GROUPS];               /* Q12  */
    Word16 noise_level[HVQ_BWE_NOISE_BANDS];      /* Q15  */
    Word16 hq_generic_offset;                     /* Q0   */
    Word16 hq_generic_fenv[HQ_FB_FENV];           /* Q1   */
    Word16 hq_generic_exc_clas = 0;               /* Q0   */
    Word16 core_sfm;                              /* Q0   */
    Word16 har_freq_est1, har_freq_est2;
    Word16 flag_dis;
    const Word16 *subband_search_offset;
    Word16 wBands[2];

    Word16 t_audio_norm[L_FRAME48k];
    Word16 t_audio_q_norm[L_FRAME48k];
    Word16 Q_audio;
    Word16 i;
    Word16 b_delta_env;
    Word16 Q_shift;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    Npeaks = 0;
    Q_audio = 0;    /* to avoid compilation warnings */

    set16_fx( npulses, 0, NB_SFM );
    set16_fx( maxpulse, 0, NB_SFM );
    set16_fx( difidx_org, 0, NB_SFM );
    set32_fx( t_audio_q, 0, L_FRAME48k );
    set32_fx( nf_gains, 0, HVQ_NF_GROUPS );
    set32_fx( pe_gains, 0, HVQ_NF_GROUPS );
    flag_dis = 1;
    move16();
    har_freq_est1 = 0;
    move16();
    har_freq_est2 = 0;
    move16();

    /*------------------------------------------------------------------*
     * Classification
     *------------------------------------------------------------------*/

    bits = hq_classifier_enc_fx( st_fx, length, t_audio, is_transient, &Npeaks, peaks, pe_gains, nf_gains, &hqswb_clas );

    *num_bits = sub(*num_bits, bits);

    /*------------------------------------------------------------------*
     * set quantization parameters
     *------------------------------------------------------------------*/

    hq_configure_fx( length, hqswb_clas, st_fx->core_brate_fx, &num_sfm, &nb_sfm, &start_norm, &num_env_bands, &numnrmibits, &hq_generic_offset,
                     &sfmsize, &sfm_start, &sfm_end );

    /*------------------------------------------------------------------*
     * Transient frame handling
     *------------------------------------------------------------------*/

    /* Interleave MLT coefficients of 4 sub-vectors in case of transient */
    IF( sub( is_transient, 1 ) == 0 )
    {
        interleave_spectrum_fx( t_audio, length );
    }

    /*------------------------------------------------------------------*
     * Scalar quantization of norms
     * Encode norm indices
     *------------------------------------------------------------------*/

    /* calculate and quantize norms */
    calc_norm_fx( t_audio, 12, ynrm, normqlg2, start_norm, num_env_bands, sfmsize, sfm_start );

    /* create differential code of quantized norm indices */
    diff_envelope_coding_fx(is_transient, num_env_bands, start_norm, ynrm, normqlg2, difidx);

    /* Find coding mode and calculate bit rate */
    hcode_l = encode_envelope_indices_fx( st_fx, num_env_bands, numnrmibits, difidx, &LCmode, 0, NORMAL_HQ_CORE, is_transient );
    *num_bits = sub(*num_bits, add( hcode_l, NORM0_BITS + FLAGS_BITS ) );

    /* Encode norm indices */
    encode_envelope_indices_fx( st_fx, num_env_bands, numnrmibits, difidx, &LCmode, 1, NORMAL_HQ_CORE, is_transient );

    /*------------------------------------------------------------------*
    * HQ Generic HF encoding
    *------------------------------------------------------------------*/

    test();
    IF ( sub( hqswb_clas, HQ_GEN_SWB ) == 0 || sub( hqswb_clas, HQ_GEN_FB ) == 0 )
    {
        hq_generic_encoding_fx(t_audio, hq_generic_fenv, hq_generic_offset, st_fx, &hq_generic_exc_clas);
        IF (sub(hq_generic_exc_clas , HQ_GENERIC_SP_EXC) == 0)
        {
            *num_bits = add(*num_bits,1);        /* conditional 1 bit saving for representing FD3 BWE excitation class */
        }
        map_hq_generic_fenv_norm_fx( hqswb_clas, hq_generic_fenv, ynrm, normqlg2, num_env_bands, nb_sfm, hq_generic_offset );
    }

    /*------------------------------------------------------------------*
     * Bit allocation
     *------------------------------------------------------------------*/

    hq_bit_allocation_fx( st_fx->core_brate_fx, length, hqswb_clas, num_bits, normqlg2, nb_sfm, sfmsize, noise_level,
                          R, Rsubband, &sum, &core_sfm, num_env_bands );

    /*------------------------------------------------------------------*
     * Normalize coefficients with quantized norms
     *------------------------------------------------------------------*/
    IF( hqswb_clas != HQ_HVQ )
    {
        test();
        IF (hqswb_clas == HQ_GEN_SWB || hqswb_clas == HQ_GEN_FB)
        {
            b_delta_env = calc_nor_delta_hf_fx( st_fx, t_audio, ynrm, Rsubband, num_env_bands, nb_sfm, sfmsize, sfm_start, core_sfm );
            sum -= b_delta_env;
        }
        normalizecoefs_fx( t_audio, ynrm, nb_sfm, sfm_start, sfm_end, t_audio_norm );
        Q_audio = 12;
    }

    /*------------------------------------------------------------------*
     * Quantize/code spectral fine structure using PVQ or HVQ
     *------------------------------------------------------------------*/
    IF( sub( hqswb_clas, HQ_HVQ) == 0 )
    {
        sum = hvq_enc_fx( st_fx, st_fx->core_brate_fx, *num_bits, Npeaks, ynrm, R, peaks, nf_gains,
                          noise_level, pe_gains, t_audio, t_audio_q );
        *num_bits = sub(*num_bits, sum);
    }
    ELSE
    {
        shape_bits = pvq_core_enc_fx( st_fx, t_audio_norm, t_audio_q_norm, &Q_audio, sum, nb_sfm, sfm_start, sfm_end, sfmsize, Rsubband, R,
        npulses, maxpulse, HQ_CORE );
        *num_bits = add( *num_bits, sub( sum, shape_bits) );
    }

    test();
    IF ( sub(hqswb_clas, HQ_HVQ) == 0 || sub(hqswb_clas, HQ_HARMONIC) == 0 )
    {
        subband_search_offset = subband_search_offsets_13p2kbps_Har_fx;
        wBands[0] = SWB_SB_BW_LEN0_16KBPS_HAR;
        move16();
        wBands[1] = SWB_SB_BW_LEN1_16KBPS_HAR;
        move16();

        IF (sub(hqswb_clas, HQ_HARMONIC) == 0)
        {
            Q_shift = sub(SWB_BWE_LR_Qs, Q_audio);
            FOR (i = 0; i < 300; i++)
            {
                t_audio_q[i] = L_shl(L_deposit_l(t_audio_q_norm[i]), Q_shift); /* Q12 */
            }
        }

        har_est_fx( t_audio_q, 300, &har_freq_est1, &har_freq_est2, &flag_dis, &st_fx->prev_frm_hfe2_fx, subband_search_offset, wBands, &st_fx->prev_stab_hfe2_fx );

        st_fx->prev_frm_hfe2_fx = har_freq_est2;
        move16();
    }

    test();
    test();
    IF ( sub(hqswb_clas, HQ_HARMONIC) != 0 || sub(hqswb_clas, HQ_HVQ) != 0 || flag_dis == 0)
    {
        st_fx->prev_frm_hfe2_fx = 0; /*reset*/        move16();
        st_fx->prev_stab_hfe2_fx = 0; /*reset*/       move16();
    }

    nf_idx = 0;
    move16();
    test();
    test();
    test();
    IF ( sub(is_transient,1 ) != 0 && sub( hqswb_clas, HQ_HVQ )!= 0 && !(sub(length, L_FRAME16k) == 0 && L_sub( st_fx->core_brate_fx, HQ_32k) == 0) )
    {
        test();
        IF (sub(hqswb_clas, HQ_GEN_SWB) == 0 || sub(hqswb_clas, HQ_GEN_FB) == 0)
        {
            nf_idx = noise_adjust_fx( t_audio_norm, 12, R, sfm_start, sfm_end, max(core_sfm,sub(num_env_bands,1)));
            push_indice_fx( st_fx, IND_NF_IDX, nf_idx, 2 );
        }
        ELSE
        {
            nf_idx = noise_adjust_fx( t_audio_norm, 12, R, sfm_start, sfm_end, core_sfm );
            push_indice_fx( st_fx, IND_NF_IDX, nf_idx, 2 );
        }
    }
    /* updates */
    st_fx->prev_hqswb_clas_fx = hqswb_clas;
    move16();


    return;
}

