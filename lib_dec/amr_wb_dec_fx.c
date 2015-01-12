/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "basop_mpy.h"
#include "basop_util.h"     /* Function prototypes                    */

/*-------------------------------------------------------------------*
 * amr_wb_dec()
 *
 * AMR-WB decoder
 *-------------------------------------------------------------------*/

void amr_wb_dec_fx(
    Word16 output_sp[],                                   /* o  : synthesis output                */
    Decoder_State_fx *st_fx                                  /* o  : Decoder static variables structure */
)
{
    Word16 i;
    Word16 vad_flag;
    Word16 coder_type;
    Word16 output_frame;                                  /* frame length at output sampling freq. */
    Word16 allow_cn_step;
    Word16 locattack, amr_io_class;
    Word16 tmps;
    Word16 synth_out_fx[L_FRAME48k];

    Word16 class_para_fx, hf_gain_fx[NB_SUBFR], voice_factors_fx[NB_SUBFR];
    Word16 delay_comp;
    Word16 last_core_ori;
    Word16 tmp_buffer_fx[L_FRAME48k];
    Word16 dct_buffer_fx[DCT_L_POST];
    Word16 frame_e_fx;
    Word16 exc_buffer_fx[DCT_L_POST];
    Word16 lsp_new_fx[M];                                 /* LSPs at the end of the frame          */
    Word16 lsf_new_fx[M];                                 /* LSFs at the end of the frame          */
    Word16 xsp_tmp[M];
    Word16 Aq_fx[NB_SUBFR*(M+1)];                         /* A(q) quantized for the 4 subframes    */
    Word16 exc2_fx[L_FRAME];                              /* total excitation buffer               */
    Word16 mem_tmp_fx[M];                                 /* temporary synthesis filter memory     */
    Word32 L_enr_q_fx;                                    /* E information for FER protection      */
    Word16 tmp_noise_fx;                                  /* Long term temporary noise energy      */
    Word16 FEC_pitch_fx;                                  /* FEC pitch                             */
    Word16 dummy_buf_fx[L_FRAME32k];                      /* dummy buffer - no usage               */
    Word16 old_exc_fx[L_EXC_DEC], *exc_fx;                /* excitation signal buffer              */
    Word16 syn_fx[L_FRAME];                               /* synthesis signal buffer               */
    Word32 L_tmp, L_tmp1;
    Word16 pitch_buf_fx[NB_SUBFR], Qdct, tmp_coder_type;  /* floating pitch for each subframe (Q6) */
    Word16 tmp16;
    Word16 sid_bw = 0;
    Word32 L_Ng_ener;
    Word16 exp2, ng_ener;

    Word16 bpf_error_signal[L_FRAME];
    CLDFB_SCALE_FACTOR scaleFactor;
    Word32 workBuffer[128*3];
    Word32 q_env[20];
    Word16 exc3[L_FRAME];
    Word16 gain_buf[NB_SUBFR16k];

    Word32 *realBuffer[CLDFB_NO_COL_MAX], *imagBuffer[CLDFB_NO_COL_MAX];
    Word32 realBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], imagBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
    Word16 syn_fx_tmp2[L_FRAME];

    Word16 pitch_buf_tmp[NB_SUBFR];
    Word16 update_flg;

    /*------------------------------------------------------------------*
     * Initialization
     *------------------------------------------------------------------*/

    FOR( i=0; i<CLDFB_NO_COL_MAX; i++ )
    {
        set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        realBuffer[i] = realBufferTmp[i];
        imagBuffer[i] = imagBufferTmp[i];
    }
    set16_fx( gain_buf, 0, NB_SUBFR16k );

    st_fx->use_partial_copy = 0;
    move16();
    st_fx->rf_flag = 0;
    move16();
    st_fx->rf_flag_last = 0;
    move16();
    st_fx->L_frame_fx = L_FRAME;
    move16();
    st_fx->core_fx = AMR_WB_CORE;
    move16();
    st_fx->core_brate_fx = st_fx->total_brate_fx;
    move16();
    st_fx->extl_fx = -1;
    move16();
    st_fx->bwidth_fx = WB;
    move16();
    coder_type = GENERIC;
    move16();
    output_frame = st_fx->output_frame_fx;
    move16();           /* frame length of the input signal */

    st_fx->bpf_off_fx = 0;
    move16();
    if( sub(st_fx->last_core_fx,HQ_CORE) == 0 )
    {
        st_fx->bpf_off_fx = 1;
        move16();
        st_fx->pfstat.on = 0;
        move16();
    }
    st_fx->igf = 0;
    move16();

    st_fx->sr_core     = i_mult(st_fx->L_frame_fx,50);
    st_fx->fscale_old  = st_fx->fscale;
    st_fx->fscale      = sr2fscale(st_fx->sr_core);

    /* Updates in case of EVS primary mode -> AMR-WB IO mode switching */
    IF( sub(st_fx->last_core_fx,AMR_WB_CORE) != 0 )
    {
        updt_IO_switch_dec_fx( output_frame, st_fx );
    }

    /* Updates in case of EVS -> AMR-WB IO switching */
    core_switching_pre_dec_fx( st_fx, output_frame );

    last_core_ori = st_fx->last_core_fx;
    move16();
    set16_fx( hf_gain_fx, 0, NB_SUBFR);

    amr_io_class = UNVOICED_CLAS;
    move16();
    L_enr_q_fx = L_deposit_l(0);
    tmp_noise_fx = 0;
    move16();

    Copy( st_fx->old_exc_fx, old_exc_fx, L_EXC_MEM_DEC );
    exc_fx = old_exc_fx + L_EXC_MEM_DEC;
    /* reset post-filter in case of switching */
    if( st_fx->pfstat.on == 0 )
    {
        st_fx->pfstat.reset = 1;
        move16();
    }
    IF( st_fx->bfi_fx > 0 )
    {
        st_fx->nbLostCmpt = add(st_fx->nbLostCmpt,1);
    }
    ELSE
    {
        st_fx->nbLostCmpt = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     * switching from ACELP@16k core to AMR-WB IO mode
     *-----------------------------------------------------------------*/
    st_fx->rate_switching_reset=0;
    move16();
    test();
    test();
    IF( sub(st_fx->last_core_fx,AMR_WB_CORE) != 0 && sub(st_fx->last_L_frame_fx,L_FRAME16k ) == 0 && sub(st_fx->last_core_fx,HQ_CORE) != 0)
    {
        /* in case of switching, do not apply BPF */
        st_fx->bpf_off_fx = 1;
        move16();

        if(st_fx->pfstat.on!=0)
        {
            Word16 mem_syn_r_size_old, mem_syn_r_size_new;

            mem_syn_r_size_old = shr(st_fx->last_L_frame_fx, 4);
            mem_syn_r_size_new = shr(st_fx->L_frame_fx, 4);
            lerp( st_fx->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_old, st_fx->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            lerp( st_fx->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_old, st_fx->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
        }

        st_fx->rate_switching_reset=lsp_convert_poly_fx( st_fx->lsp_old_fx, L_FRAME, 1 );
        /* convert old quantized LSF vector */
        lsp2lsf_fx( st_fx->lsp_old_fx, st_fx->lsf_old_fx, M, INT_FS_FX );

        /* FEC - update adaptive LSF mean vector */
        Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi1_fx, M );
        Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi0_fx, M );
        Copy( st_fx->lsf_old_fx, st_fx->lsf_adaptive_mean_fx, M );

        /* Reset LPC mem */
        IF( L_sub(st_fx->sr_core,16000) == 0 )
        {
            Copy( GEWB2_Ave_fx, st_fx->mem_AR_fx, M );
        }
        ELSE
        {
            Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
        }
        set16_fx( st_fx->mem_MA_fx,0, M );

        /* update synthesis filter memories */
        synth_mem_updt2( L_FRAME, st_fx->last_L_frame_fx, st_fx->old_exc_fx, st_fx->mem_syn_r, st_fx->mem_syn2_fx, NULL, DEC );
        Copy( st_fx->old_exc_fx, old_exc_fx, L_EXC_MEM_DEC );
        Copy_Scale_sig(st_fx->mem_syn2_fx, st_fx->mem_syn1_fx, M, sub(-1,st_fx->Q_syn)); /*Q-1*/


        /* LSP -> ISP */
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsp2isp_fx( st_fx->lsp_old_fx, st_fx->lsp_old_fx, xsp_tmp, M );

        /* update buffer of old subframe pitch values */
        FOR( i=NB_SUBFR16k-NB_SUBFR; i<NB_SUBFR16k; i++ )
        {
            /*((float)FAC_16k/(float)FAC_12k8) * st_fx->old_pitch_buf[i]*/
            st_fx->old_pitch_buf_fx[i-1] = Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 26214);
            move32();
        }

        FOR( i=2*NB_SUBFR16k-NB_SUBFR; i<2*NB_SUBFR16k; i++ )
        {
            st_fx->old_pitch_buf_fx[i-2] = Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 26214);
            move32();
        }

        IF( sub(st_fx->bfi_pitch_frame_fx,L_FRAME) != 0 )
        {
            st_fx->bfi_pitch_fx = mult_r(26214, st_fx->bfi_pitch_fx);
            st_fx->bfi_pitch_frame_fx = L_FRAME;
            move16();
        }
    }

    IF( sub(st_fx->last_core_fx,AMR_WB_CORE) != 0 )
    {
        /* reset the unvoiced/audio signal improvement memories */
        E_LPC_f_isp_a_conversion( st_fx->lsp_old_fx, st_fx->old_Aq_fx, M );
        Copy( st_fx->old_Aq_fx, st_fx->old_Aq_fx + (M+1), M+1 );
        Copy( st_fx->old_Aq_fx, st_fx->old_Aq_fx + 2*(M+1), M+1 );
        Copy( st_fx->old_Aq_fx, st_fx->old_Aq_fx + 3*(M+1), M+1 );
    }

    test();
    if(sub(st_fx->last_bwidth_fx,NB)==0 && st_fx->ini_frame_fx!=0)
    {
        st_fx->rate_switching_reset=1;
        move16();
    }

    /*----------------------------------------------------------------------*
     * GOOD frame
     *----------------------------------------------------------------------*/

    IF( !st_fx->bfi_fx )
    {
        /*----------------------------------------------------------------*
         * Processing of FRAME_NO_DATA frames
         * Decoding of SID frames
         *----------------------------------------------------------------*/

        test();
        IF ( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st_fx->core_brate_fx,SID_1k75) == 0 )
        {
            /* decode CNG parameters */
            CNG_dec_fx( st_fx, L_FRAME, Aq_fx, st_fx->core_brate_fx, lsp_new_fx, lsf_new_fx, &allow_cn_step, &sid_bw, q_env );

            /* comfort noise generation */
            CNG_exc_fx( st_fx->core_brate_fx, L_FRAME, &st_fx->Enew_fx, &st_fx->cng_seed_fx, exc_fx, exc2_fx, &st_fx->lp_ener_fx, st_fx->last_core_brate_fx,
                        &st_fx->first_CNG_fx, &st_fx->cng_ener_seed_fx, dummy_buf_fx, allow_cn_step, &st_fx->last_allow_cn_step_fx, st_fx->prev_Q_exc, st_fx->Q_exc , st_fx->num_ho_fx,
                        q_env, st_fx->lp_env_fx, st_fx->old_env_fx, st_fx->exc_mem_fx, st_fx->exc_mem1_fx, &sid_bw, &st_fx->cng_ener_seed1_fx, exc3, st_fx->Opt_AMR_WB_fx );

            set16_fx( voice_factors_fx, 32767, NB_SUBFR );
            class_para_fx = 0;
            move16();

            if( st_fx->first_CNG_fx == 0 )
            {
                st_fx->first_CNG_fx = 1;
                move16();
            }

            /* update past excitation signals for LD music post-filter */
            Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
            Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );

            /* synthesis at 12k8 Hz sampling rate */
            syn_12k8_fx( L_FRAME, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1, st_fx->Q_exc, st_fx->Q_syn );

            CNG_reset_dec_fx( st_fx, pitch_buf_fx, dummy_buf_fx );

            /* update st->mem_syn1 for ACELP core switching */
            Copy_Scale_sig(st_fx->mem_syn2_fx, st_fx->mem_syn1_fx, M, sub(-1,st_fx->Q_syn)); /*Q-1*/

            IF( st_fx->flag_cna )
            {
                Word16 pitch_temp[4];
                pitch_temp[2] = shl(L_FRAME, 6);
                move16();
                pitch_temp[3] = shl(L_FRAME, 6);
                move16();
                frame_energy_fx( L_FRAME, pitch_temp, syn_fx, 0, &frame_e_fx, st_fx->Q_syn );
                /*st->psf_lp_noise = 0.99f * st->psf_lp_noise + 0.01f * frame_e;                 */
                st_fx->psf_lp_noise_fx = round_fx(L_mac(L_mult(32440, st_fx->psf_lp_noise_fx), 328, frame_e_fx));  /*Q8*/
            }

            vad_flag = 0;
            move16();
        }

        /*----------------------------------------------------------------*
         * Decoding of all other frames
         *----------------------------------------------------------------*/

        ELSE
        {
            /*-----------------------------------------------------------------*
             * After CNG period, use the most up-to-date ISPs
             *-----------------------------------------------------------------*/

            test();
            IF ( L_sub(st_fx->last_core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st_fx->last_core_brate_fx,SID_1k75) == 0 )
            {
                Copy( st_fx->lspCNG_fx, st_fx->lsp_old_fx, M );
                lsp2lsf_fx( st_fx->lspCNG_fx, st_fx->lsf_old_fx, M, INT_FS_FX );
                set16_fx( old_exc_fx, 0, L_EXC_MEM_DEC );
            }

            /*------------------------------------------------------------*
             * Extracts VAD information from the bitstream in AMR-WB IO mode
             *------------------------------------------------------------*/

            vad_flag = (Word16)get_next_indice_fx( st_fx, 1 );

            coder_type = GENERIC;
            move16();
            if ( vad_flag == 0 )
            {
                coder_type = INACTIVE;
                move16();
            }

            /*-----------------------------------------------------------------*
             * ISF de-quantization and interpolation
             *-----------------------------------------------------------------*/

            isf_dec_amr_wb_fx( st_fx, Aq_fx, lsf_new_fx, lsp_new_fx );

            /*------------------------------------------------------------*
             * Decode excitation
             *------------------------------------------------------------*/

            decod_amr_wb_fx( st_fx, Aq_fx, pitch_buf_fx, exc_fx, exc2_fx, hf_gain_fx, voice_factors_fx, gain_buf );

            /* synthesis for ACELP core switching and SWB BWE */
            syn_12k8_fx( L_FRAME, Aq_fx, exc_fx, tmp_buffer_fx, st_fx->mem_syn1_fx, 1, st_fx->Q_exc, -1 );

            /*------------------------------------------------------------*
             * Update long-term energies for FEC
             * Update ISP vector for CNG
             *------------------------------------------------------------*/

            IF( sub(coder_type,INACTIVE) == 0 )
            {
                IF( sub(st_fx->unv_cnt_fx,20) > 0 )
                {
                    /*ftmp = st->lp_gainc * st->lp_gainc;*/
                    L_tmp1 =  L_mult0(st_fx->lp_gainc_fx, st_fx->lp_gainc_fx);   /* Q3*Q3 -> Q6*/
                    /*st->lp_ener = 0.7f * st->lp_ener + 0.3f * ftmp;*/
                    L_tmp = Mult_32_16(st_fx->lp_ener_fx, 22938);
                    st_fx->lp_ener_fx = L_add(L_tmp, Mult_32_16(L_tmp1, 9830));  /*Q6 + Q6*/
                    FOR( i=0; i<M; i++ )
                    {
                        L_tmp = L_mult(3277, lsp_new_fx[i]);
                        st_fx->lspCNG_fx[i] = round_fx(L_mac(L_tmp, 29491, st_fx->lspCNG_fx[i]));
                    }
                }
                ELSE
                {
                    st_fx->unv_cnt_fx = add(st_fx->unv_cnt_fx,1);
                }
            }
            ELSE
            {
                st_fx->unv_cnt_fx = 0;
                move16();
            }

            /*------------------------------------------------------------*
             * Save filter memory in case the synthesis is redone after scaling
             * Core synthesis at 12k8 Hz
             *------------------------------------------------------------*/

            Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx,4, &st_fx->mem_deemph_fx,
            st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, vad_flag, tmp_buffer_fx );
            Copy( st_fx->mem_syn2_fx, mem_tmp_fx, M );
            syn_12k8_fx( L_FRAME, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1, st_fx->Q_exc, st_fx->Q_syn );

            /*------------------------------------------------------------*
             * FEC - Estimate the classification information
             *------------------------------------------------------------*/

            FEC_clas_estim_fx( st_fx, 1, L_FRAME, &st_fx->clas_dec, coder_type, pitch_buf_fx, &st_fx->last_good_fx, syn_fx, &st_fx->lp_ener_FER_fx,
            &st_fx->decision_hyst_fx, &st_fx->UV_cnt_fx, &st_fx->LT_UV_cnt_fx, &st_fx->Last_ener_fx, &locattack, st_fx->lt_diff_etot_fx,
            &amr_io_class, st_fx->core_brate_fx, &st_fx->Q_syn, &class_para_fx, st_fx->mem_syn_clas_estim_fx, &st_fx->classifier_Q_mem_syn, 0, 0, 0, 0, 0, 0 );

            /* update past excitation signals for LD music post-filter */
            Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
            Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );
            Copy( st_fx->dct_post_old_exc_fx, exc_buffer_fx, DCT_L_POST-OFFSET2 );

            IF( sub(output_frame,L_FRAME8k) != 0 )
            {
                IF ( sub(coder_type,INACTIVE) == 0 )
                {
                    frame_energy_fx( L_FRAME, pitch_buf_fx, syn_fx, 0, &frame_e_fx, st_fx->Q_syn );
                    /*st->psf_lp_noise = 0.99f * st->psf_lp_noise + 0.01f * frame_e;                 */
                    st_fx->psf_lp_noise_fx = round_fx(L_mac(L_mult(32440, st_fx->psf_lp_noise_fx), 328, frame_e_fx));  /*Q8*/
                }
            }

            test();
            test();
            IF( sub(amr_io_class,UNVOICED_CLAS) != 0 && sub(coder_type,INACTIVE) != 0 && sub(st_fx->psf_lp_noise_fx,15<<8) < 0 )
            {
                tmp_coder_type = AUDIO;
                move16();
                test();
                if (sub(st_fx->last_coder_type_fx,INACTIVE) == 0 || sub(st_fx->last_coder_type_fx,UNVOICED) == 0)
                {
                    tmp_coder_type = INACTIVE;
                    move16();
                }
                /* Extrapolation of the last future part, windowing and high resolution DCT transform */
                Prep_music_postP_fx( exc_buffer_fx, dct_buffer_fx, st_fx->filt_lfE_fx, st_fx->last_core_fx, pitch_buf_fx, st_fx->LDm_enh_lp_gbin_fx, st_fx->Q_exc, &Qdct );

                /* LD music post-filter */
                LD_music_post_filter_fx( dct_buffer_fx, dct_buffer_fx, st_fx->core_brate_fx, st_fx->bfi_fx, &st_fx->LDm_last_music_flag_fx, &st_fx->LDm_last_bfi_count_fx,
                                         st_fx->LDm_thres_fx, &st_fx->LDm_nb_thr_1_fx, &st_fx->LDm_nb_thr_3_fx, st_fx->LDm_lt_diff_etot_fx,
                                         &st_fx->LDm_mem_etot_fx, st_fx->LDm_enh_min_ns_gain_fx, st_fx->LDm_bckr_noise_fx, st_fx->LDm_enro_fx,
                                         st_fx->LDm_enh_lf_EO_fx, st_fx->LDm_enh_lp_gbin_fx, st_fx->filt_lfE_fx, &st_fx->last_nonfull_music_fx,
                                         &st_fx->Old_ener_Q, -1, tmp_coder_type, Qdct );

                /* Inverse DCT transform, retrieval of the aligned excitation, re-synthesis */
                Post_music_postP_fx( dct_buffer_fx, exc2_fx, mem_tmp_fx, st_fx->mem_syn2_fx, Aq_fx, syn_fx, &st_fx->Q_exc, &st_fx->prev_Q_syn,
                                     &st_fx->Q_syn, st_fx->mem_syn_clas_estim_fx, 1, &st_fx->mem_deemph_fx, st_fx->pst_old_syn_fx,
                                     &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, NULL
                                     ,NULL
                                   );
            }
            ELSE
            {
                /*------------------------------------------------------------*
                 * Improvement for unvoiced and audio signals
                 *------------------------------------------------------------*/
                improv_amr_wb_gs_fx( amr_io_class, coder_type, st_fx->core_brate_fx, &st_fx->seed_tcx_fx, st_fx->old_Aq_fx, st_fx->mem_syn2_fx, st_fx->lt_voice_fac_fx,
                locattack, Aq_fx, exc2_fx, st_fx->Q_exc, mem_tmp_fx, syn_fx, st_fx->Q_syn, pitch_buf_fx, st_fx->last_ener_fx, st_fx->last_coder_type_fx );

                FOR( i = 0; i < DCT_L_POST; i++ )
                {
                    /*st->filt_lfE[i] = 0.3f + 0.7f * st->filt_lfE[i] ;*/
                    st_fx->filt_lfE_fx[i] = round_fx(L_mac(L_deposit_h(1229),  22938, st_fx->filt_lfE_fx[i]));
                }
            }

            /*------------------------------------------------------------*
             * FEC - Estimate pitch
             *------------------------------------------------------------*/

            FEC_pitch_estim_fx( 1, L_FRAME, st_fx->clas_dec, st_fx->last_good_fx, pitch_buf_fx, st_fx->old_pitch_buf_fx, &st_fx->bfi_pitch_fx,
            &st_fx->bfi_pitch_frame_fx, &st_fx->upd_cnt_fx, GENERIC );

            /*------------------------------------------------------------*
             * FEC - Smooth the speech energy evolution when recovering after a BAD frame
             * (smoothing is performed in the excitation domain and signal is resynthesized after)
             *------------------------------------------------------------*/

            FOR( i=0; i<NB_SUBFR; i++ )
            {
                pitch_buf_tmp[i] = mult_r(pitch_buf_fx[i], 512);
                move16();
            }

            FEC_scale_syn_fx( L_FRAME, &update_flg, st_fx->clas_dec, st_fx->last_good_fx, syn_fx, pitch_buf_tmp, st_fx->enr_old_fx, L_enr_q_fx, -1, st_fx->prev_bfi_fx, st_fx->last_core_brate_fx,
            exc_fx, exc2_fx, Aq_fx, &st_fx->old_enr_LP, mem_tmp_fx, st_fx->mem_syn2_fx, st_fx->Q_exc, st_fx->Q_syn );
        }

    } /* End of GOOD FRAME */

    /*----------------------------------------------------------------*
     * BAD frame
     *----------------------------------------------------------------*/
    ELSE
    {
        /* long burst frame erasures */
        test();
        if( sub(st_fx->nbLostCmpt,5) > 0 && sub(st_fx->clas_dec,VOICED_CLAS) >= 0 )
        {
            st_fx->last_good_fx = VOICED_TRANSITION;
            move16();
        }
        vad_flag = st_fx->last_vad_fx;
        move16();
        amr_io_class = st_fx->last_good_fx;
        move16();
        class_para_fx = 0;
        move16();

        /* LSF estimation and A(z) calculation */
        FEC_lsf_estim_fx( st_fx, L_FRAME, Aq_fx, lsf_new_fx, lsp_new_fx );

        /* calculation of excitation signal */
        FEC_exc_estim_fx( st_fx, L_FRAME, exc_fx, exc2_fx, tmp_buffer_fx, pitch_buf_fx, voice_factors_fx,
        &FEC_pitch_fx, dummy_buf_fx, lsf_new_fx, &st_fx->Q_exc, &tmp_noise_fx);

        /* synthesis for ACELP core switching and SWB BWE */
        syn_12k8_fx( L_FRAME, Aq_fx, exc_fx, tmp_buffer_fx, st_fx->mem_syn1_fx, 1, st_fx->Q_exc, -1 );

        /* update past excitation signals */
        Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
        Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );

        FOR( i = 0; i < DCT_L_POST; i++ )
        {
            /*st->filt_lfE[i] = 0.3f + 0.7f * st->filt_lfE[i];*/
            st_fx->filt_lfE_fx[i] = round_fx(L_mac(L_deposit_h(1229),  22938, st_fx->filt_lfE_fx[i]));
        }

        Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx, 4, &st_fx->mem_deemph_fx,
        st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, 1, tmp_buffer_fx );

        /* synthesis at 12k8 Hz sampling rate */
        syn_12k8_fx( L_FRAME, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1 , st_fx->Q_exc, st_fx->Q_syn );

        /* estimate the pitch-synchronous speech energy per sample to be used when normal operation recovers */
        frame_ener_fx( L_FRAME, st_fx->last_good_fx, syn_fx, shr(FEC_pitch_fx, 6), &st_fx->enr_old_fx, L_FRAME, st_fx->Q_syn, 3, 0 );
    }

    /*--------------------------------------------------------*
     * NB post-filter
     *--------------------------------------------------------*/
    test();
    IF( sub(output_frame,L_FRAME8k) == 0 || sub(st_fx->last_bwidth_fx,NB) == 0)
    {
        FOR( i=0; i<NB_SUBFR; i++ )
        {
            pitch_buf_tmp[i] = mult_r(pitch_buf_fx[i], 512);
            move16();
        }
        IF( sub(output_frame,L_FRAME8k) == 0 )
        {
            st_fx->pfstat.on = 1;
            move16();
            nb_post_filt( L_FRAME, &(st_fx->pfstat), &st_fx->psf_lp_noise_fx, tmp_noise_fx, syn_fx, Aq_fx, pitch_buf_tmp, coder_type, 0 );
        }
        ELSE
        {
            st_fx->pfstat.on = 0;
            move16();
            nb_post_filt( L_FRAME, &(st_fx->pfstat), &st_fx->psf_lp_noise_fx, tmp_noise_fx, syn_fx, Aq_fx, pitch_buf_tmp, AUDIO, 0 );
        }
    }

    /*------------------------------------------------------------------*
     * Perform fixed deemphasis through 1/(1 - g*z^-1)
     *-----------------------------------------------------------------*/

    /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
    Copy( syn_fx + L_FRAME - L_SYN_MEM, st_fx->mem_syn_r, L_SYN_MEM );

    /* update old synthesis for classification */
    Copy( syn_fx + L_FRAME - L_SYN_MEM_CLAS_ESTIM12k8, st_fx->mem_syn_clas_estim_fx, L_SYN_MEM_CLAS_ESTIM12k8 );

    deemph_fx( syn_fx, PREEMPH_FAC, L_FRAME, &(st_fx->mem_deemph_fx) );

    /* might be able to use syn_fx_tmp[0]... tbv */
    unscale_AGC( syn_fx, st_fx->Q_syn, syn_fx_tmp2, st_fx->agc_mem_fx, L_FRAME );
    Copy( syn_fx_tmp2, syn_fx, L_FRAME );

    /* TCX=Q-1, ACELP2 Q0 */
    Copy_Scale_sig( syn_fx + L_FRAME/2, st_fx->old_syn_Overl, L_FRAME/2, sub(-1,st_fx->Q_syn)); /*Q_syn*/
    Copy_Scale_sig( syn_fx + L_FRAME-M-1, st_fx->syn, M+1, sub(0,st_fx->Q_syn)); /*Q0*/

    /*------------------------------------------------------------------*
     * Formant post-filter
     *-----------------------------------------------------------------*/

    Copy( syn_fx, tmp_buffer_fx + L_SYN_MEM, L_FRAME );
    IF( sub(output_frame,L_FRAME8k) != 0  &&  sub(st_fx->last_bwidth_fx,NB) != 0)
    {
        st_fx->pfstat.on = 1;
        move16();
        test();
        formant_post_filt( &(st_fx->pfstat), tmp_buffer_fx + L_SYN_MEM, Aq_fx, syn_fx, L_FRAME, st_fx->lp_noise, st_fx->total_brate_fx, sub(amr_io_class,AUDIO_CLAS) == 0);
    }



    /*----------------------------------------------------------------*
     * Comfort Noise Addition
     *----------------------------------------------------------------*/

    test();
    IF( (sub(st_fx->psf_lp_noise_fx,15<<8) >= 0) || (coder_type == INACTIVE) )
    {
        /*VAD only for non inactive frame*/
        st_fx->VAD = 0;
        move16();
        test();
        if( st_fx->VAD && sub(coder_type,INACTIVE) != 0 )
        {
            st_fx->VAD = 1;
            move16();
        }

        ApplyFdCng( syn_fx, st_fx->Q_syn, NULL, NULL, 0, st_fx->hFdCngDec_fx, st_fx->m_frame_type, st_fx, 0
                    , 0
                  );

        st_fx->hFdCngDec_fx->hFdCngCom->frame_type_previous = st_fx->m_frame_type;

        /*Noisy speech detector*/
        noisy_speech_detection( st_fx->VAD, syn_fx, L_FRAME, st_fx->Q_syn, st_fx->hFdCngDec_fx->msNoiseEst, st_fx->hFdCngDec_fx->msNoiseEst_exp,
                                st_fx->hFdCngDec_fx->psize_shaping_norm, st_fx->hFdCngDec_fx->psize_shaping_norm_exp, st_fx->hFdCngDec_fx->nFFTpart_shaping,
                                &(st_fx->hFdCngDec_fx->lp_noise), &(st_fx->hFdCngDec_fx->lp_speech), &(st_fx->hFdCngDec_fx->hFdCngCom->flag_noisy_speech) );

        st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech = mult_r(st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.99));
        IF ( st_fx->hFdCngDec_fx->hFdCngCom->flag_noisy_speech != 0 )
        {
            st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech = add(st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.01));
            move16();
        }
        st_fx->lp_noise = st_fx->hFdCngDec_fx->lp_noise;
        move16();

        IF( st_fx->flag_cna )
        {
            generate_masking_noise( syn_fx, st_fx->Q_syn, st_fx->hFdCngDec_fx->hFdCngCom, st_fx->hFdCngDec_fx->hFdCngCom->frameSize, AMR_WB_CORE );
        }

        /*Copy(syn+L_FRAME-M-1, st_fx->syn, M+1);*/
    }

    /*----------------------------------------------------------------*
      * Change the sampling frequency to 8/16/32 kHz
      * Bass post-filter
      *----------------------------------------------------------------*/

    /* check if the CLDFB works on the right sample rate */
    IF( (st_fx->cldfbAna_fx->usb * st_fx->cldfbAna_fx->no_col) != L_FRAME )
    {
        /* resample to ACELP internal sampling rate */
        Word16 newCldfbBands = CLDFB_getNumChannels(INT_FS_FX);

        resampleCldfb( st_fx->cldfbAna_fx, newCldfbBands, L_FRAME, 0 );
        resampleCldfb( st_fx->cldfbBPF_fx, newCldfbBands, L_FRAME, 0 );

        if( st_fx->ini_frame_fx > 0 )
        {
            st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels,st_fx->cldfbAna_fx->no_channels);
        }
    }

    /* bass post-filter (when "bpf_error_signal" is defined, no additional delay is introduced) */
    bass_psfilter_fx( st_fx->Opt_AMR_WB_fx, syn_fx, L_FRAME, pitch_buf_fx, st_fx->pst_old_syn_fx,
                      &st_fx->pst_mem_deemp_err_fx, &st_fx->pst_lp_ener_fx, st_fx->bpf_off_fx, st_fx->stab_fac_fx, &st_fx->stab_fac_smooth_fx,
                      st_fx->mem_mean_pit_fx, st_fx->Track_on_hist_fx, st_fx->vibrato_hist_fx, &st_fx->psf_att_fx, GENERIC, st_fx->Q_syn, bpf_error_signal);

    cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &scaleFactor, syn_fx,
                            negate(st_fx->Q_syn), CLDFB_NO_COL_MAX, workBuffer );

    scaleFactor.hb_scale = scaleFactor.lb_scale;
    move16();

    /* CLDFB analysis and add the BPF error signal */
    move16();
    move16(); /* bpf_length */
    addBassPostFilterFx( bpf_error_signal, realBuffer, imagBuffer, st_fx->cldfbBPF_fx, workBuffer, negate(st_fx->Q_syn ),
                         (st_fx->bpf_off_fx == 0) ? CLDFB_NO_COL_MAX : 0, st_fx->cldfbAna_fx->no_col, st_fx->cldfbAna_fx->no_channels,
                         &scaleFactor );

    st_fx->Q_syn2 = st_fx->Q_syn;
    move16();

    if( sub(st_fx->cldfbSyn_fx->bandsToZero,sub(st_fx->cldfbSyn_fx->no_channels,st_fx->cldfbAna_fx->no_channels)) != 0 )
    {
        /* in case of BW switching, re-init to default */
        st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels, st_fx->cldfbAna_fx->no_channels);
    }

    /* CLDFB synthesis of the combined signal */
    cldfbSynthesisFiltering( st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &scaleFactor, synth_out_fx, negate(st_fx->Q_syn2), CLDFB_NO_COL_MAX, workBuffer );

    /* Bring CLDFB output to Q-1 */
    Scale_sig( synth_out_fx, output_frame, negate(st_fx->Q_syn2) );
    st_fx->Q_syn2 = 0;
    move16();

    /* save synthesis - needed in case of core switching */
    Copy( synth_out_fx, st_fx->previoussynth_fx, output_frame );
    st_fx->Q_syn2 = 0;
    move16();

    /*--------------------------------------------------------*
     * calculate the average frame energy
     *--------------------------------------------------------*/

    frame_ener_fx( L_FRAME, st_fx->clas_dec, syn_fx, mult_r(pitch_buf_fx[3],512), &L_Ng_ener, L_FRAME, -1, 3, 0 );

    /*--------------------------------------------------------*
     * optimized for NO_S@-26dBov with street noise @ SNR=25dB
     *--------------------------------------------------------*/

    /* ng_ener = 10.0f * (float)log10(ng_ener + 0.01f) - 90.3087f + 15; */
    L_Ng_ener = L_max(1, L_Ng_ener);
    tmp16 = norm_l(L_Ng_ener);
    exp2 = Log2_norm_lc(L_shl(L_Ng_ener, tmp16));
    tmp16 = sub(30, tmp16);
    ng_ener = mac_r(L_shl(L_mac(-1233858L, tmp16, 24660), 8+2), exp2, 771);
    /* st_fx->ng_ener_ST = 0.7f * st_fx->ng_ener_ST + 0.3f * ng_ener; */
    st_fx->Ng_ener_ST_fx = mac_r(L_mult(st_fx->Ng_ener_ST_fx, 22938), ng_ener, 9830);
    move16();

    /*-----------------------------------------------------------------*
     * Bandwidth extension 6kHz-8kHz
     *-----------------------------------------------------------------*/

    IF( sub(output_frame,L_FRAME16k) >= 0 )
    {
        /*three last arguments of the function are: scaling of exc2_fx, syn_fx and synth_out_fx;
          scaling of syn_fx is Q_syn2 (that's before modify_Fs), and scaling of synth_out_fx is Q_syn2-1 (after modify_Fs);
          thus the bugfix;
          plus: amr_io_class_fx changed to amr_io_class*/

        hf_synth_amr_wb_fx( st_fx->core_brate_fx, output_frame, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn_hf_fx,
                            st_fx->delay_syn_hf_fx, &st_fx->prev_r_fx, &st_fx->fmerit_w_sm_fx, &amr_io_class, st_fx->mem_hp_interp_fx, synth_out_fx,
                            class_para_fx, hf_gain_fx, voice_factors_fx, pitch_buf_fx, st_fx->Ng_ener_ST_fx, lsf_new_fx,
                            &st_fx->frame_count_fx, &st_fx->ne_min_fx, &st_fx->fmerit_m_sm_fx, &st_fx->voice_fac_amr_wb_hf, &st_fx->unvoicing_fx, &st_fx->unvoicing_sm_fx,
                            &st_fx->unvoicing_flag_fx, &st_fx->voicing_flag_fx, &st_fx->start_band_old_fx, &st_fx->OptCrit_old_fx, st_fx->Q_exc, st_fx->Q_syn2 );
    }

    /*----------------------------------------------------------------------*
     * Updates
     *----------------------------------------------------------------------*/

    updt_dec_fx( st_fx, L_FRAME, coder_type, old_exc_fx, pitch_buf_fx, 0, Aq_fx , lsf_new_fx, lsp_new_fx, voice_factors_fx, dummy_buf_fx, gain_buf );

    /* update main codec parameters */
    st_fx->last_core_fx = st_fx->core_fx;
    move16();
    st_fx->last_extl_fx = -1;
    move16();
    st_fx->last_codec_mode = st_fx->codec_mode;
    move16();
    st_fx->last_L_frame_fx = L_FRAME;
    move16();
    st_fx->last_core_brate_fx = st_fx->core_brate_fx;
    move16();
    st_fx->last_codec_mode = st_fx->codec_mode;
    move16();
    st_fx->last_bwidth_fx = WB;
    move16();
    st_fx->prev_Q_exc = st_fx->Q_exc;
    move16();
    if ( !st_fx->bfi_fx )
    {
        st_fx->last_good_fx = st_fx->clas_dec;
        move16();
    }
    st_fx->last_vad_fx = vad_flag;
    move16();

    /*----------------------------------------------------------------*
     * Overlap of ACELP synthesis with old MDCT memory
     *----------------------------------------------------------------*/

    if( st_fx->bfi_fx )
    {
        /* calculate another loss frame to fill gap in case of switching frame loss */
        acelp_core_switch_dec_bfi_fx( st_fx, st_fx->fer_samples_fx, coder_type );
    }

    delay_comp = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS);
    Scale_sig(st_fx->delay_buf_out_fx, delay_comp, sub(st_fx->Q_syn2,st_fx->Q_old_postdec));
    st_fx->Q_old_postdec=st_fx->Q_syn2;
    move16();
    IF( sub(last_core_ori,HQ_CORE) == 0 )
    {
        Word16 step, alpha,nz;

        Scale_sig(st_fx->old_out_fx, L_FRAME48k, sub(st_fx->Q_syn2,st_fx->Q_old_wtda));
        st_fx->Q_old_wtda=st_fx->Q_syn2;
        move16();

        Copy( st_fx->delay_buf_out_fx,synth_out_fx, delay_comp );   /* copy the HQ/ACELP delay synchroniation buffer at the beginning of ACELP frame */

        i = 15;
        move16();
        tmps = NS2SA_fx2(st_fx->output_Fs_fx, 6000000L);
        nz = NS2SA_fx2(st_fx->output_Fs_fx, N_ZERO_MDCT_NS);
        step = Inv16(tmps, &i);
        step = shl(step, i);
        alpha = 0;
        move16();

        test();
        IF( st_fx->prev_bfi_fx && st_fx->HqVoicing_fx )
        {
            Copy_Scale_sig( st_fx->fer_samples_fx, &st_fx->old_out_fx[nz], tmps,negate(st_fx->Q_syn2));
        }

        FOR (i = 0; i < tmps; i++)
        {
            synth_out_fx[i+delay_comp] = msu_r(L_mult(synth_out_fx[i+delay_comp], alpha), st_fx->old_out_fx[i+nz], add(alpha, -32768));
            move16();
            alpha = add(alpha, step);
        }
    }

    st_fx->prev_bfi_fx = st_fx->bfi_fx;

    if( L_sub(st_fx->core_brate_fx,SID_1k75) > 0 )
    {
        st_fx->last_active_brate_fx = st_fx->total_brate_fx;
        move32();
    }

    test();
    IF( L_sub(st_fx->core_brate_fx,SID_1k75) > 0 && st_fx->first_CNG_fx )
    {
        if( sub(st_fx->act_cnt_fx,BUF_DEC_RATE) >= 0 )
        {
            st_fx->act_cnt_fx = 0;
            move16();
        }

        st_fx->act_cnt_fx = add(st_fx->act_cnt_fx,1);

        test();
        if( sub(st_fx->act_cnt_fx,BUF_DEC_RATE) == 0 && st_fx->ho_hist_size_fx > 0 )
        {
            st_fx->ho_hist_size_fx = sub(st_fx->ho_hist_size_fx,1);
        }

        st_fx->act_cnt2_fx = add(st_fx->act_cnt2_fx,1);
        st_fx->act_cnt2_fx = s_min(st_fx->act_cnt2_fx,MIN_ACT_CNG_UPD);
    }

    /*----------------------------------------------------------------*
     * HP filtering
     * Final synthesis output
     *----------------------------------------------------------------*/

    /* Delay ACELP synthesis by DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS delay */
    IF ( sub(output_frame,L_FRAME16k) >= 0 )
    {
        tmps = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS);
        Scale_sig( st_fx->prev_synth_buffer_fx, tmps, sub(st_fx->Q_syn2, st_fx->Qprev_synth_buffer_fx) );

        Copy( synth_out_fx, tmp_buffer_fx, output_frame );
        Copy( st_fx->prev_synth_buffer_fx, synth_out_fx, tmps );
        Copy( tmp_buffer_fx, synth_out_fx + tmps, output_frame - tmps );
        Copy( tmp_buffer_fx + output_frame - tmps, st_fx->prev_synth_buffer_fx, tmps );
    }

    /* HP filter */
    Scale_sig32( st_fx->L_mem_hp_out_fx, 4, sub(st_fx->Q_syn2, st_fx->Qprev_synth_buffer_fx) );
    st_fx->Qprev_synth_buffer_fx = st_fx->Q_syn2;
    hp20( synth_out_fx, 1/*stride*/, output_frame, st_fx->L_mem_hp_out_fx, L_mult0(output_frame, 50) );

    /* save synthesis for core switching */
    Copy( synth_out_fx+NS2SA_fx2( st_fx->output_Fs_fx,ACELP_LOOK_NS+DELAY_BWE_TOTAL_NS), st_fx->old_synth_sw_fx, NS2SA_fx2(st_fx->output_Fs_fx,FRAME_SIZE_NS-ACELP_LOOK_NS-DELAY_BWE_TOTAL_NS) );

    {
        /* TCX-LTP Postfilter: used in AMR-WB IO to update memories and to avoid discontinuities when the past frame was TCX */
        Word16 delta = NS2SA_fx2( st_fx->output_Fs_fx, TCXLTP_DELAY_NS );
        Scale_sig(st_fx->tcxltp_mem_in, delta, sub(st_fx->Q_syn2, st_fx->Qprev_synth_buffer_fx));
        Scale_sig(st_fx->tcxltp_mem_out, output_frame, sub(st_fx->Q_syn2, st_fx->Qprev_synth_buffer_fx));
        tcx_ltp_post( st_fx->tcxltp, ACELP_CORE, output_frame, st_fx->L_frame_past, 0, synth_out_fx, NULL,
                      delta, 0, 0, 0, 0, &st_fx->tcxltp_pitch_int_post_prev,
                      &st_fx->tcxltp_pitch_fr_post_prev, &st_fx->tcxltp_gain_post_prev,
                      &st_fx->tcxltp_filt_idx_prev, st_fx->pit_res_max,
                      &st_fx->pit_res_max_past,
                      0, 0, st_fx->tcxltp_mem_in,
                      st_fx->tcxltp_mem_out, st_fx->total_brate_fx );
    }

    /* final output of synthesis signal */
    syn_output_fx( st_fx->codec_mode, synth_out_fx, output_frame, output_sp, st_fx->Q_syn2 );


    return;
}
