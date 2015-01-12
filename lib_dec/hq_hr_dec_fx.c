/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"     /* Function prototypes                   */
#include "rom_com_fx.h" /* Static table prototypes                */


void hq_pred_hb_bws_fx(
    Decoder_State_fx *st_fx,                 /* i/o: decoder state structure                 */
    const Word16 *ynrm,                  /* i  : norm quantization index vector          */
    const Word16 length,                 /* i  : frame length                            */
    const Word16 hqswb_clas,             /* i  : HQ SWB class                            */
    const Word16 *SWB_fenv               /* i  : SWB frequency envelopes             Q1  */
)
{
    Word16 i;
    Word32 L_tmp;
    Word16 tmp,exp;

    IF( sub(length,L_FRAME32k) >= 0)
    {
        /* calculate the switching parameters */
        test();
        test();
        IF( ( sub(hqswb_clas,HQ_GEN_SWB) != 0 && L_sub(st_fx->core_brate_fx,HQ_32k) <= 0 ) || L_sub(st_fx->core_brate_fx,HQ_32k) > 0 )
        {
            st_fx->prev_ener_shb_fx = 0;
            move16();
            L_tmp = L_deposit_l(0);
            FOR(i=25; i<SFM_N_HARM; i++)
            {
                L_tmp = L_add(L_tmp,dicn_fx[ynrm[i]]);/*Q14*/
            }
            L_tmp = L_min(8191, L_shr(L_tmp, 13));
            st_fx->prev_ener_shb_fx = extract_l(L_tmp);/*Q1*/
            st_fx->prev_ener_shb_fx = shl(mult(st_fx->prev_ener_shb_fx,2521),2);/*Q3*/
            IF(L_sub(dicn_fx[ynrm[sub(SFM_N_HARM,2)]],dicn_fx[ynrm[25]]) >= 0)
            {
                st_fx->last_hq_tilt_fx = 26214;/*Q15*/          move16();
            }
            ELSE
            {
                exp = norm_l(dicn_fx[ynrm[25]]);
                L_tmp = L_shl(dicn_fx[ynrm[25]],exp);/*Q(14+exp)*/
                tmp = extract_h(L_tmp);/*Q(14+exp-16=exp-2)*/
                tmp = div_s(16384,tmp);/*Q(15+14-exp+2 = 31-exp)*/
                L_tmp = Mult_32_16(dicn_fx[ynrm[sub(SFM_N_HARM,2)]],tmp);/*Q(14+31-exp-15 = 30-exp)*/
                tmp = extract_l(L_shr(L_tmp,sub(15,exp)));/*Q15*/
                st_fx->last_hq_tilt_fx = s_min(tmp,26214);/*Q15*/
            }
        }
        ELSE
        {
            st_fx->prev_ener_shb_fx = 0;
            move16();
            FOR(i=0; i<SWB_FENV-3; i++)
            {
                st_fx->prev_ener_shb_fx = add(st_fx->prev_ener_shb_fx,SWB_fenv[i]);/*Q1*/
            }
            st_fx->prev_ener_shb_fx = shl(mult(st_fx->prev_ener_shb_fx,2521),2);/*Q3*/
            IF(sub(SWB_fenv[11],SWB_fenv[1]) >= 0)
            {
                st_fx->last_hq_tilt_fx = 26214;/*Q15*/    move16();
            }
            ELSE
            {
                exp = norm_s(SWB_fenv[1]);
                tmp = shl(SWB_fenv[1],exp);/*Q(exp+1)*/
                tmp = div_s(16384,tmp);/*Q(15+14-exp-1 = 28-exp)*/
                tmp = mult(SWB_fenv[11],tmp);/*Q(1+28-exp-15 = 14-exp)*/
                tmp = shl(tmp,add(exp,1));/*Q15*/
                st_fx->last_hq_tilt_fx = s_min(tmp,26214);/*Q15*/
            }
        }
    }

    IF( sub(st_fx->last_inner_frame_fx,L_FRAME32k) >= 0 )
    {
        set16_fx(st_fx->prev_SWB_fenv_fx, st_fx->prev_ener_shb_fx, SWB_FENV);
    }

    return;

}
/*--------------------------------------------------------------------------*
 * hq_hr_dec_fx()
 *
 * HQ High rate decoding routine
 *--------------------------------------------------------------------------*/
void hq_hr_dec_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure fx                     */
    Word32 *t_audio_q,             /* o  : transform-domain coefficients              Q12 */
    const Word16 length,                 /* i  : frame length                               Q0  */
    Word16 num_bits,               /* i  : number of available bits                   Q0  */
    Word16 *ynrm,                  /* o  : norm quantization index vector             Q0  */
    Word16 *is_transient,          /* o  : transient flag                             Q0  */
    Word16 *hqswb_clas,            /* o  : HQ SWB class                               Q0  */
    Word16 *SWB_fenv               /* o  : SWB frequency envelopes                    Q1  */
)
{
    Word16 nb_sfm;
    Word16 sum, hcode_l;
    const Word16 *sfmsize, *sfm_start, *sfm_end;
    Word16 num_sfm, numnrmibits;
    Word16 nf_idx;
    Word16 normqlg2[NB_SFM], R[NB_SFM];
    Word16 pulses[NB_SFM], maxpulse[NB_SFM];
    Word16 env_stab;                            /*Q15*/
    Word16 Rsubband[NB_SFM];                    /*Q3*/
    Word16 start_norm, Npeaks = 0;
    Word16 noise_level[HVQ_BWE_NOISE_BANDS];    /*Q15*/
    Word16 peak_idx[HVQ_MAX_PEAKS_32k];
    Word16 hq_generic_offset;
    Word16 num_env_bands;
    Word16 hq_generic_exc_clas = 0;
    Word16 core_sfm;
    Word16 har_freq_est1, har_freq_est2;
    Word16 flag_dis;
    const Word16 *subband_search_offset;
    Word16 wBands[2];
    Word16 bits, i;
    Word16 t_audio_q_norm[L_FRAME48k];
    Word16 Q_audio;
    Word16 b_delta_env;
    Word16 tmp,n_band;
    Word16 Q_shift;

    Q_audio = 0;    /* to avoid compilation warnings */

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    set16_fx( pulses, 0, NB_SFM );
    set16_fx( maxpulse, 0, NB_SFM );
    flag_dis = 1;
    move16();
    har_freq_est1 = 0;
    move16();
    har_freq_est2 = 0;
    move16();

    /*------------------------------------------------------------------*
     * Decode classification
     *------------------------------------------------------------------*/

    bits = hq_classifier_dec_fx( st_fx, st_fx->core_brate_fx, length, is_transient, hqswb_clas);
    num_bits = sub(num_bits, bits);

    /*------------------------------------------------------------------*
     * set quantization parameters
     *------------------------------------------------------------------*/

    hq_configure_fx( length, *hqswb_clas, st_fx->core_brate_fx, &num_sfm, &nb_sfm, &start_norm,
                     &num_env_bands, &numnrmibits, &hq_generic_offset, &sfmsize, &sfm_start, &sfm_end );

    /*------------------------------------------------------------------*
     * Unpacking bit-stream
     *------------------------------------------------------------------*/

    nf_idx = 0;
    move16();
    test();
    test();
    test();
    IF( !*is_transient && sub(*hqswb_clas, HQ_HVQ) != 0 && !(sub(length, L_FRAME16k) == 0 && L_sub(st_fx->core_brate_fx, HQ_32k) == 0))
    {
        nf_idx = get_next_indice_fx( st_fx, 2 );
    }

    /*------------------------------------------------------------------*
     * Decode envelope
     *------------------------------------------------------------------*/

    hcode_l = decode_envelope_indices_fx( st_fx, start_norm, num_env_bands, numnrmibits, ynrm, NORMAL_HQ_CORE, *is_transient );
    num_bits = sub(num_bits, add(hcode_l, NORM0_BITS + FLAGS_BITS));

    dequantize_norms_fx( start_norm, num_env_bands, *is_transient, ynrm, normqlg2 );

    test();
    IF ( sub(*hqswb_clas,  HQ_GEN_SWB) == 0 || sub(*hqswb_clas, HQ_GEN_FB) == 0 )
    {
        hq_generic_exc_clas = swb_bwe_gain_deq_fx( st_fx, HQ_CORE, NULL, SWB_fenv, st_fx->core_brate_fx == HQ_32k, *hqswb_clas );
        if ( sub(hq_generic_exc_clas , HQ_GENERIC_SP_EXC) == 0)
        {
            num_bits = add(num_bits,1);        /* conditional 1 bit saving for representing HQ GENERIC excitation class */
        }
        map_hq_generic_fenv_norm_fx( *hqswb_clas, SWB_fenv, ynrm, normqlg2, num_env_bands, nb_sfm, hq_generic_offset );
    }

    env_stab = 0;
    move16();
    IF( sub(*hqswb_clas, HQ_HVQ) == 0 )
    {
        st_fx->mem_env_delta_fx = 0;
        move16();
    }
    ELSE IF( sub(length, L_FRAME32k) == 0 )
    {
        env_stab = env_stability_fx( ynrm, SFM_N_ENV_STAB, st_fx->mem_norm_fx, &st_fx->mem_env_delta_fx );
    }
    ELSE
    {
        st_fx->mem_norm_fx[0] = 31;
        move16();
        st_fx->mem_env_delta_fx = 0;
        move16();
    }

    IF ( sub(*hqswb_clas, HQ_HVQ) == 0 )
    {
        st_fx->env_stab_fx = 32767;
        move16();       /* 1 in Q15, stable by definition */
    }
    ELSE
    {
        IF ( sub(length, L_FRAME32k) == 0 )
        {
            st_fx->env_stab_fx = env_stab;
            move16();    /* calculated stability */
        }
        ELSE
        {
            st_fx->env_stab_fx = env_stability_fx( ynrm, SFM_N_ENV_STAB_WB, st_fx->mem_norm_hqfec_fx, &st_fx->mem_env_delta_hqfec_fx );
        }
    }
    st_fx->env_stab_plc_fx = env_stab_smo_fx(s_min(st_fx->env_stab_fx, sub(32767, stab_trans_fx[L_STAB_TBL-1])), st_fx->env_stab_state_p_fx, &st_fx->envstabplc_hocnt_fx);

    /*------------------------------------------------------------------*
     * Bit allocation
     *------------------------------------------------------------------*/

    hq_bit_allocation_fx( st_fx->core_brate_fx, length, *hqswb_clas, &num_bits, normqlg2, nb_sfm, sfmsize, noise_level,
                          R, Rsubband, &sum, &core_sfm, num_env_bands );

    test();
    test();
    IF( sub(*hqswb_clas, HQ_GEN_SWB) == 0 && st_fx->bws_cnt1_fx > 0 && L_sub(st_fx->core_brate_fx, HQ_24k40) == 0 )
    {
        tmp = i_mult(st_fx->bws_cnt1_fx, 1638);
        move16();
        IF( sub(st_fx->L_frame_fx, L_FRAME16k) == 0 )
        {
            FOR (n_band = 0; n_band < 4; n_band++)
            {
                SWB_fenv[n_band] = mult_r(SWB_fenv[n_band], tmp);
                move16();
            }
        }

        FOR (n_band = 4; n_band < SWB_FENV; n_band++)
        {
            SWB_fenv[n_band] = mult_r(SWB_fenv[n_band], tmp);
            move16();
        }
    }

    test();
    IF ( sub(*hqswb_clas , HQ_GEN_SWB) == 0 || sub(*hqswb_clas , HQ_GEN_FB) == 0 )
    {
        b_delta_env = get_nor_delta_hf_fx(st_fx, ynrm, Rsubband, num_env_bands, nb_sfm, core_sfm );
        sum = sub(sum,b_delta_env);
    }

    /*------------------------------------------------------------------*
     * Decode spectral fine structure using HVQ/PVQ
     *------------------------------------------------------------------*/

    IF( sub(*hqswb_clas, HQ_HVQ) == 0 )
    {
        hvq_dec_fx( st_fx, num_bits, st_fx->core_brate_fx, ynrm, R, noise_level, peak_idx, &Npeaks, t_audio_q, st_fx->core_fx );
    }
    ELSE
    {
        pvq_core_dec_fx(st_fx, sfm_start, sfm_end, sfmsize, t_audio_q_norm, &Q_audio, sum, nb_sfm, Rsubband, R, pulses, maxpulse, HQ_CORE );
    }

    test();
    IF ( sub(*hqswb_clas, HQ_HVQ) == 0 || sub(*hqswb_clas, HQ_HARMONIC) == 0 )
    {
        subband_search_offset = subband_search_offsets_13p2kbps_Har_fx;
        wBands[0] = SWB_SB_BW_LEN0_16KBPS_HAR;
        move16();
        wBands[1] = SWB_SB_BW_LEN1_16KBPS_HAR;
        move16();

        IF (sub(*hqswb_clas, HQ_HARMONIC) == 0)
        {
            Q_shift = sub(SWB_BWE_LR_Qs, Q_audio);
            FOR (i = 0; i < 300; i++)
            {
                t_audio_q[i] = L_shl(L_deposit_l(t_audio_q_norm[i]), Q_shift); /* Q12 */
            }
        }

        har_est_fx( t_audio_q, 300 ,&har_freq_est1, &har_freq_est2, &flag_dis, &st_fx->prev_frm_hfe2_fx, subband_search_offset, wBands, &st_fx->prev_stab_hfe2_fx );

        st_fx->prev_frm_hfe2_fx = har_freq_est2;
        move16();
    }

    test();
    test();
    IF ( sub(*hqswb_clas, HQ_HARMONIC) != 0 || sub(*hqswb_clas, HQ_HVQ) != 0 || flag_dis == 0)
    {
        st_fx->prev_frm_hfe2_fx = 0; /*reset*/        move16();
        st_fx->prev_stab_hfe2_fx = 0; /*reset*/       move16();
    }

    /*------------------------------------------------------------------*
     * Spectral filling
     *------------------------------------------------------------------*/

    fill_spectrum_fx( t_audio_q_norm, t_audio_q, R, *is_transient, ynrm, SWB_fenv, hq_generic_offset, nf_idx, length, env_stab,
                      &st_fx->no_att_hangover_fx, &st_fx->energy_lt_fx, &st_fx->hq_generic_seed_fx, hq_generic_exc_clas,
                      core_sfm, *hqswb_clas, noise_level, st_fx->core_brate_fx, st_fx->prev_noise_level_fx, st_fx->prev_R_fx, st_fx->prev_coeff_out_fx, peak_idx, Npeaks, pulses, st_fx->old_is_transient_fx[0],
                      st_fx->prev_normq_fx, st_fx->prev_env_fx, st_fx->prev_bfi_fx, sfmsize, sfm_start, sfm_end, &st_fx->prev_L_swb_norm_fx, st_fx->prev_hqswb_clas_fx, num_sfm,
                      st_fx->prev_env_Q, num_env_bands );

    enforce_zero_for_min_envelope_fx( *hqswb_clas, ynrm, t_audio_q, nb_sfm, sfm_start, sfm_end );


    IF( sub(*is_transient, 1) == 0 )
    {
        de_interleave_spectrum_fx( t_audio_q, length );
    }

    /*------------------------------------------------------------------*
     * WB/SWB bandwidth switching
     *------------------------------------------------------------------*/
    hq_pred_hb_bws_fx(st_fx, ynrm, length, *hqswb_clas, SWB_fenv );

    /* update */
    st_fx->prev_hqswb_clas_fx = *hqswb_clas;
    move16();

    return;
}
