/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "stl.h"
/*--------------------------------------------------------------------------
 *  hq_core_dec()
 *
 *  HQ core decoder
 *--------------------------------------------------------------------------*/

void hq_core_dec_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure fx         */
    Word16 synth[],                 /* o  : output synthesis                   */
    Word16 *Q_synth,                /* o  : Q value of synth                   */
    const Word16 output_frame,            /* i  : output frame length                */
    const Word16 hq_core_type,            /* i  : HQ core type                       */
    const Word16 core_switching_flag      /* i  : ACELP->HQ switching frame flag     */
)
{
    Word16 num_bits, is_transient, hqswb_clas, inner_frame;
    Word16 i, j, flag_uv, num_Sb, nb_sfm;
    Word16 ynrm[NB_SFM], num_bands_p[MAX_SB_NB];
    Word16 ener_match;                  /* Q13 */
    Word32 t_audio_q[L_FRAME48k];       /* Q12 */
    Word16 Q_audio;
    Word32 wtda_audio[2*L_FRAME48k];
    Word16 delay_comp;
    Word32 normq_fx[NB_SFM];
    Word16 mean_en_high_fx;
    Word16 SWB_fenv_fx[SWB_FENV+DIM_FB];
    const Word16 *sfmsize, *sfm_start, *sfm_end;
    Word16 mem_mdct_nowin[L_FRAME48k];

    Word16 tmp, tmp_loop;
    Word32 L_tmp;
    UWord16 lsb;

    /*--------------------------------------------------------------------------
     * Initializations
     *--------------------------------------------------------------------------*/

    set32_fx( t_audio_q, 0, L_FRAME48k );
    set16_fx( num_bands_p, 0, MAX_SB_NB );
    set16_fx( ynrm, 39, NB_SFM );              /* Initialize to the smallest value */
    mean_en_high_fx = 0;
    move16();
    Q_audio = 12;
    move16();
    sfm_start = sfm_end = NULL;
    num_Sb = nb_sfm = 0;

    st_fx->tcx_cfg.tcx_last_overlap_mode = st_fx->tcx_cfg.tcx_curr_overlap_mode;
    move16();
    if (sub(st_fx->tcx_cfg.tcx_curr_overlap_mode, FULL_OVERLAP) == 0)
    {
        st_fx->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;
        move16();
    }
    st_fx->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
    move16();

    /*--------------------------------------------------------------------------
     * Find the number of bits for transform-domain coding
     *--------------------------------------------------------------------------*/

    /* set the total bit-budget */
    /*num_bits = (short)(st->total_brate / 50); */
    Mpy_32_16_ss(st_fx->total_brate_fx, 5243, &L_tmp, &lsb);  /* 5243 is 1/50 in Q18. (0+18-15=3) */
    num_bits = extract_l(L_shr(L_tmp, 3)); /*Q0 */

    IF( !st_fx->bfi_fx )
    {
        IF ( sub(core_switching_flag, 1) == 0 )
        {
            core_switching_hq_prepare_dec_fx( st_fx, &num_bits, output_frame );

            /* During ACELP->HQ core switching, limit the HQ core bitrate to 48kbps */
            if ( sub(num_bits, HQ_48k / 50)  > 0 )
            {
                num_bits = (Word16)(HQ_48k / 50);
                move16();
            }
        }

        /* subtract signalling bits */
        num_bits = sub(num_bits, st_fx->next_bit_pos_fx);

        /* set FEC parameters */
        flag_uv = sub(1, st_fx->HqVoicing_fx);

        /* subtract the number of bits for pitch & gain at higher bitrates */
        test();
        IF ( !(core_switching_flag) && L_sub(st_fx->core_brate_fx, MINIMUM_RATE_TO_ENCODE_VOICING_FLAG) > 0 )
        {
            st_fx->HqVoicing_fx = get_next_indice_fx( st_fx, 1 );
            num_bits = sub(num_bits, 1);
        }
        ELSE
        {
            st_fx->HqVoicing_fx = 0;
            move16();
            if ( L_sub(st_fx->core_brate_fx, MINIMUM_RATE_TO_ENCODE_VOICING_FLAG) > 0 )
            {
                st_fx->HqVoicing_fx = 1;
                move16();
            }
        }
    }
    ELSE
    {
        flag_uv = 0;
        move16();
    }

    /* set inner frame (== coded bandwidth) length */
    inner_frame = inner_frame_tbl[st_fx->bwidth_fx];
    move16();

    IF ( st_fx->bfi_fx == 0)
    {
        st_fx->ph_ecu_HqVoicing_fx = 0;
        move16();
        if ( sub(output_frame, L_FRAME16k) >= 0 )
        {
            st_fx->ph_ecu_HqVoicing_fx = st_fx->HqVoicing_fx;
            move16();
        }
    }

    IF ( sub(output_frame, L_FRAME8k) == 0 )
    {
        hq_configure_bfi_fx( &nb_sfm, &num_Sb, num_bands_p, &sfmsize, &sfm_start, &sfm_end );
    }

    /*--------------------------------------------------------------------------
     * transform-domain decoding
     *--------------------------------------------------------------------------*/

    IF( sub(st_fx->bfi_fx, 1) == 0 )
    {
        is_transient = st_fx->old_is_transient_fx[0];
        move16();

        /* WB/SWB bandwidth switching */
        test();
        test();
        if ( sub(inner_frame, L_FRAME16k) == 0 && sub(st_fx->last_inner_frame_fx, L_FRAME32k) >= 0 && st_fx->bws_cnt_fx > 0)
        {
            inner_frame = st_fx->last_inner_frame_fx;
            move16();
        }

        IF ( sub(output_frame, L_FRAME16k) >= 0 ) /* Apply phase ecu for WB, SWB and FB */
        {
            /* ecu_rec sent to OLA, env_stab passed in ph_ecu_st */
            hq_ecu_fx(st_fx->prev_good_synth_fx, t_audio_q, &st_fx->time_offs_fx, st_fx->X_sav_fx, &st_fx->Q_X_sav, &st_fx->num_p_fx, st_fx->plocs_fx, st_fx->plocsi_fx, st_fx->env_stab_fx,
                      &st_fx->last_fec_fx, st_fx->ph_ecu_HqVoicing_fx, &st_fx->ph_ecu_active_fx, st_fx->gapsynth_fx, st_fx->prev_bfi_fx, st_fx->old_is_transient_fx, st_fx->mag_chg_1st_fx,
                      st_fx->Xavg_fx, &st_fx->beta_mute_fx, output_frame, st_fx );
        }
        ELSE
        {
            HQ_FEC_processing_fx( st_fx, t_audio_q, is_transient, st_fx->ynrm_values_fx, st_fx->r_p_values_fx, num_Sb, nb_sfm, num_bands_p,
            inner_frame, sfm_start, sfm_end, output_frame );
        }

        st_fx->old_is_transient_fx[2] = st_fx->old_is_transient_fx[1];
        move16();
        st_fx->old_is_transient_fx[1] = st_fx->old_is_transient_fx[0];
        move16();

        IF ( sub(output_frame, L_FRAME16k) >= 0 )
        {
            /* keep st->previoussynth updated as in FEC_HQ_pitch_analysis but no LP analysis */
            delay_comp = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS);

            Copy( st_fx->previoussynth_fx + delay_comp, st_fx->previoussynth_fx, sub(output_frame, delay_comp) );
            Copy( st_fx->delay_buf_out_fx, st_fx->previoussynth_fx + output_frame - delay_comp, delay_comp );

            flag_uv = 1;
            move16();   /*  disable costly pitch out synthesis in bfi frame  */
            st_fx->HqVoicing_fx = sub(1, flag_uv);                         /*  safety setting  for HQ->ACELP switch logic           */
            set16_fx( st_fx->fer_samples_fx, 0, L_FRAME48k );              /*  safety, create a known signal state for HQ->ACELP switch logic */
        }
    }
    ELSE
    {
        IF( sub(hq_core_type, LOW_RATE_HQ_CORE) == 0 )
        {
            IF( sub(st_fx->prev_bfi_fx, 1) == 0 )
            {
                set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
                set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
                st_fx->last_max_pos_pulse_fx = 0;
                move16();
            }

            /* HQ low rate decoder */
            hq_lr_dec_fx( st_fx, t_audio_q, inner_frame, num_bits, &is_transient );

            hqswb_clas = is_transient;
            move16();
            Q_audio = 12;
            move16();
        }
        ELSE
        {
            /* HQ high rate decoder */
            hq_hr_dec_fx(st_fx, t_audio_q, inner_frame, num_bits, ynrm, &is_transient, &hqswb_clas, SWB_fenv_fx);
            Q_audio = 12;
            move16();
        }

        /* WB/SWB bandwidth switching */
        if ( st_fx->bws_cnt_fx > 0 )
        {
            inner_frame = st_fx->last_inner_frame_fx;
            move16();
        }

        /* scaling (coefficients are in nominal level) */
        IF( sub(output_frame, NORM_MDCT_FACTOR) != 0 )
        {
            IF (sub(output_frame, L_FRAME32k) == 0)
            {
                Q_audio = sub(Q_audio, 1);      /* Multiply by 2 */
            }
            ELSE
            {
                tmp = mult_r(output_frame, 410/2);     /* 1/8000 in Q15 */
                ener_match = hq_nominal_scaling_inv[tmp];
                FOR( i=0; i < inner_frame; i++ )
                {
                    /*t_audio_q[i] *= ener_match;*/
                    Mpy_32_16_ss(t_audio_q[i], ener_match, &L_tmp, &lsb);   /*12+13-15=10 */
                    t_audio_q[i] = L_add(L_shl(L_tmp, 2), lshr(lsb, 14));
                    move16();   /* Q12 */
                }
            }
        }

        HQ_FEC_Mem_update_fx( st_fx, t_audio_q, normq_fx, ynrm, num_bands_p, is_transient, hqswb_clas,
                              core_switching_flag, nb_sfm, num_Sb, &mean_en_high_fx, hq_core_type, output_frame );
    }
    /*--------------------------------------------------------------------------
     * Attenuate HFs in case of band-width switching (from higher BW to lower BW)
     *--------------------------------------------------------------------------*/

    IF( st_fx->bws_cnt_fx > 0 )
    {
        tmp = sub(N_WS2N_FRAMES,st_fx->bws_cnt_fx);
        ener_match = div_s(tmp,N_WS2N_FRAMES);  /*Q15*/

        IF( is_transient )
        {
            FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                tmp_loop = mult(inner_frame,8192);
                FOR( j=mult(inner_frame_tbl_fx[st_fx->bwidth_fx],8192); j<tmp_loop; j++ )
                {
                    tmp = i_mult(i,inner_frame); /*Q0*/
                    tmp = mult(tmp,8192);  /*Q0*/
                    tmp = add(tmp,j);
                    t_audio_q[tmp] = Mult_32_16(t_audio_q[tmp],ener_match);
                    move32();/*Q12*/
                }
            }
        }
        ELSE
        {
            FOR( i=inner_frame_tbl_fx[st_fx->bwidth_fx]; i<inner_frame; i++ )
            {
                t_audio_q[i] = Mult_32_16(t_audio_q[i],ener_match);
                move32(); /*Q12*/
            }
        }
    }

    /* attenuate HFs in case of band-width switching */
    IF( st_fx->bws_cnt1_fx > 0 )
    {
        IF( sub(st_fx->bws_cnt1_fx,N_NS2W_FRAMES) == 0 )
        {
            ener_match = 32767;
            move16(); /*Q15*/
        }
        ELSE
        {
            ener_match = div_s(st_fx->bws_cnt1_fx,N_NS2W_FRAMES);   /*Q15*/
        }

        IF( is_transient )
        {
            FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                tmp_loop = mult(inner_frame,8192);
                FOR( j=mult(inner_frame_tbl_fx[sub(st_fx->bwidth_fx,1)],8192); j<tmp_loop; j++ )
                {
                    tmp = i_mult(i,inner_frame);    /*Q0*/
                    tmp = mult(tmp,8192);   /*Q0*/
                    tmp = add(tmp,j);
                    t_audio_q[tmp] = Mult_32_16(t_audio_q[tmp],ener_match);
                    move32(); /*Q12*/
                }
            }
        }
        ELSE
        {
            FOR( i=inner_frame_tbl_fx[sub(st_fx->bwidth_fx,1)]; i<inner_frame; i++ )
            {
                t_audio_q[i] = Mult_32_16(t_audio_q[i],ener_match);   /*Q12*/ move32();
            }
        }
    }

    /* WB/SWB bandwidth switching */
    IF(is_transient)
    {
        Copy_Scale_sig_32_16(&t_audio_q[240], st_fx->t_audio_q_fx, 80, -13);
    }
    ELSE
    {
        Copy_Scale_sig_32_16(t_audio_q, st_fx->t_audio_q_fx, L_FRAME16k, -13);
    }
    if(st_fx->bws_cnt_fx > 0)
    {
        inner_frame = st_fx->last_inner_frame_fx;
        move16();
    }

    /*--------------------------------------------------------------------------
     * Inverse transform
     * Overlap-add
     * Pre-echo reduction
     *--------------------------------------------------------------------------*/

    test();
    IF (sub(output_frame, L_FRAME8k) == 0 || st_fx->bfi_fx == 0)
    {
        Inverse_Transform( t_audio_q, &Q_audio, wtda_audio, is_transient, output_frame, inner_frame );
        *Q_synth = Q_audio;
        move16();
    }

    IF ( sub(output_frame, L_FRAME8k) == 0 )
    {
        test();
        IF( st_fx->bfi_fx == 0 && st_fx->prev_bfi_fx == 0)
        {
            Copy_Scale_sig(st_fx->old_out_fx+N_ZERO_NB, st_fx->prev_oldauOut_fx, output_frame-N_ZERO_NB, negate(st_fx->Q_old_wtda) );
        }
        ELSE IF( sub(st_fx->prev_bfi_fx, 1) == 0)
        {
            set16_fx( st_fx->prev_oldauOut_fx, 0, output_frame );
        }

        test();
        test();
        test();
        test();
        IF( (sub(st_fx->prev_bfi_fx, 1) == 0 || sub(st_fx->bfi_fx, 1) == 0) && st_fx->old_is_transient_fx[2] == 0 && sub(st_fx->last_core_fx, HQ_CORE) == 0 && sub(st_fx->last_codec_mode,MODE1)==0)
        {
            time_domain_FEC_HQ_fx( st_fx, wtda_audio, synth, mean_en_high_fx, output_frame, Q_synth );
        }
        ELSE
        {
            window_ola_fx( wtda_audio, synth, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
            st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, st_fx->prev_bfi_fx, st_fx->oldHqVoicing_fx , st_fx->oldgapsynth_fx );
            st_fx->phase_mat_next_fx = 0;
            move16();
        }

        test();
        test();
        IF ( (st_fx->bfi_fx == 0 && st_fx->prev_bfi_fx == 0) || !(sub(output_frame, L_FRAME16k) >= 0))
        {
            preecho_sb_fx( st_fx->core_brate_fx, wtda_audio, Q_audio, synth, mem_mdct_nowin, *Q_synth, output_frame, &st_fx->memfilt_lb_fx,
                           &st_fx->mean_prev_hb_fx, &st_fx->smoothmem_fx, &st_fx->mean_prev_fx, &st_fx->mean_prev_nc_fx, &st_fx->wmold_hb_fx, &st_fx->prevflag_fx, &st_fx->pastpre_fx,
                           /*t_audio_q, is_transient,*/ st_fx->bwidth_fx );
        }
    }
    ELSE
    {
        test();
        IF (sub(st_fx->bfi_fx, 1) == 0 && sub(output_frame, L_FRAME16k) >= 0 )
        {
            /* PHASE_ECU active */
            Q_audio = 15;
            move16();
            window_ola_fx( t_audio_q, synth, &Q_audio, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
            ALDO_WINDOW, ALDO_WINDOW, st_fx->prev_bfi_fx && !st_fx->ph_ecu_active_fx, st_fx->oldHqVoicing_fx, st_fx->oldgapsynth_fx );
            *Q_synth = Q_audio;
            move16();
        }
        ELSE
        {
            /* no BFI or baseline PLC active */
            window_ola_fx( wtda_audio, synth, Q_synth, st_fx->old_out_fx, &st_fx->Q_old_wtda, output_frame,
            st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, st_fx->prev_bfi_fx && !st_fx->ph_ecu_active_fx, st_fx->oldHqVoicing_fx, st_fx->oldgapsynth_fx);
        }

        test();
        test();
        IF ( (st_fx->bfi_fx == 0 && st_fx->prev_bfi_fx == 0) || !(sub(output_frame, L_FRAME16k) >= 0))
        {
            preecho_sb_fx( st_fx->core_brate_fx, wtda_audio, Q_audio, synth, mem_mdct_nowin, *Q_synth, output_frame, &st_fx->memfilt_lb_fx,
            &st_fx->mean_prev_hb_fx, &st_fx->smoothmem_fx, &st_fx->mean_prev_fx, &st_fx->mean_prev_nc_fx, &st_fx->wmold_hb_fx, &st_fx->prevflag_fx, &st_fx->pastpre_fx,
            /*t_audio_q, is_transient,*/ st_fx->bwidth_fx );
        }
    }

    test();
    test();
    test();
    test();
    test();
    test();
    IF (!st_fx->bfi_fx
        && st_fx->prev_bfi_fx
        && (sub(st_fx->last_codec_mode, MODE2) == 0)
        && (sub(st_fx->last_core_bfi, TCX_20_CORE) == 0 || sub(st_fx->last_core_bfi, TCX_10_CORE) == 0)
        && (st_fx->plcInfo.concealment_method == TCX_NONTONAL)
        && (L_sub(st_fx->plcInfo.nbLostCmpt, 4) < 0) )
    {
        waveform_adj2_fix(st_fx->tonalMDCTconceal.secondLastPcmOut,
                          synth,
                          st_fx->plcInfo.data_noise,
                          &st_fx->plcInfo.outx_new_n1_fx,
                          &st_fx->plcInfo.nsapp_gain_fx,
                          &st_fx->plcInfo.nsapp_gain_n_fx,
                          &st_fx->plcInfo.recovery_gain,
                          st_fx->plcInfo.step_concealgain_fx,
                          st_fx->plcInfo.Pitch_fx,
                          st_fx->plcInfo.FrameSize,
                          1,
                          0,
                          add(extract_l(st_fx->plcInfo.nbLostCmpt), 1),
                          st_fx->bfi_fx);
    }

    IF (sub(output_frame, L_FRAME16k) >= 0)
    {
        IF (sub(st_fx->ph_ecu_HqVoicing_fx, 1) == 0)
        {
            st_fx->oldHqVoicing_fx = 1;
            move16();
            Copy(st_fx->gapsynth_fx, st_fx->oldgapsynth_fx, L_FRAME48k);
        }
        ELSE
        {
            st_fx->oldHqVoicing_fx = 0;
            move16();
        }
    }
    ELSE
    {
        st_fx->oldHqVoicing_fx = 0;
        move16();
    }

    if( sub(st_fx->nbLostCmpt, FRAMECTTOSTART_MDCT) == 0 )
    {
        st_fx->HqVoicing_fx = 0;
        move16();
    }

    IF( sub(output_frame, L_FRAME8k) == 0)
    {
        Copy32(wtda_audio, st_fx->oldIMDCTout_fx, output_frame);
        Copy(&st_fx->old_auOut_2fr_fx[output_frame], st_fx->old_auOut_2fr_fx, output_frame);
        Copy_Scale_sig(synth, &st_fx->old_auOut_2fr_fx[output_frame], output_frame, negate(*Q_synth));
    }

    /* prepare synthesis output buffer (as recent as possible) for HQ FEC */

    {
        Word16 nbsubfr;
        /*nbsubfr = extract_l(L_mult0(st_fx->L_frame_fx,FL2WORD16(1/L_SUBFR)));*/
        nbsubfr = 4;
        if(sub(st_fx->L_frame_fx,320) == 0)
        {
            nbsubfr = 5;
            move16();
        }

        Copy32( &st_fx->old_pitch_buf_fx[nbsubfr], &st_fx->old_pitch_buf_fx[0], nbsubfr );
        set32_fx( &st_fx->old_pitch_buf_fx[nbsubfr],  (L_SUBFR<<16), nbsubfr );
        Copy( &st_fx->mem_pitch_gain[2], &st_fx->mem_pitch_gain[nbsubfr+2], nbsubfr );
        set16_fx( &st_fx->mem_pitch_gain[2], 0, nbsubfr );
    }

    return;
}
