/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_enc_fx.h"     /* Encoder static table prototypes        */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * acelp_core_enc()
 *
 * ACELP core encoder
 *--------------------------------------------------------------------*/
void acelp_core_enc_fx(
    Encoder_State_fx *st_fx,                    /* i/o: encoder state structure             */
    LPD_state *mem,                        /* i/o: acelp memories                      */
    const Word16 inp_fx[],                    /* i  : input signal of the current frame   */
    const Word16 vad_flag_fx,
    const Word32 ener_fx,                     /* i  : residual energy from Levinson-Durbin*/
    const Word16 pitch[3],                    /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing_fx[3],               /* i  : Open-loop pitch gains               */
    Word16 A_fx[NB_SUBFR16k*(M+1)],     /* i  : A(z) unquantized for the 4 subframes*/
    Word16 Aw_fx[NB_SUBFR16k*(M+1)],    /* i  : weighted A(z) unquant. for subframes*/
    const Word16 epsP_h_fx[M+1],              /* i  : LP prediction errors                */
    const Word16 epsP_l_fx[M+1],              /* i  : LP prediction errors                */
    Word16 lsp_new_fx[M],               /* i  : LSPs at the end of the frame        */
    Word16 lsp_mid_fx[M],               /* i  : LSPs in the middle of the frame     */
    Word16 coder_type_fx,               /* i  : coding type                         */
    const Word16 sharpFlag_fx,                /* i  : formant sharpening flag             */
    Word16 vad_hover_flag_fx,
    const Word16 gsc_attack_flag_fx,          /* i  : flag signalling attack encoded by AC mode (GSC) */
    Word32 bwe_exc_extended_fx[],       /* i/o: bandwidth extended excitation       */
    Word16 *voice_factors_fx,           /* o  : voicing factors                     */
    Word16 old_syn_12k8_16k_fx[],       /* o  : intermediate ACELP synthesis at 12.8kHz or 16kHz to be used by SWB BWE */
    Word16 pitch_buf_fx[NB_SUBFR16k],   /* o  : floating pitch for each subframe    */
    Word16 *unbits_fx,                  /* o  : number of unused bits               */
    const Word16 Q_new,
    const Word16 shift
)
{
    Word16 nBits;                                                 /* reserved bits                        */
    Word16 i;
    Word16 old_exc_fx[L_EXC], *exc_fx;                            /* excitation signal buffer             */
    Word16 lsf_new_fx[M];                                         /* ISFs at the end of the frame         */
    Word16 Aq_fx[NB_SUBFR16k*(M+1)];                              /* A(z)   quantized for the 4 subframes */
    Word16 syn_fx[L_FRAME16k];                                    /* synthesis vector                     */
    Word16 res_fx[L_FRAME16k];                                    /* Residual signal for FER protection   */
    Word16 exc2_fx[L_FRAME16k];                                   /* enhanced excitation                  */
    Word16 Es_pred_fx;                                            /* predicited scaled innovation energy  */
    Word16 tmp_noise_fx;                                          /* NB post-filter long-term noise energy*/
    Word16 tc_subfr_fx;                                           /* TC sub-frame indication              */
    Word16 old_bwe_exc_fx[(PIT16k_MAX + (L_FRAME16k + 1) + L_SUBFR16k) * 2]; /* excitation buffer         */
    Word16 *bwe_exc_fx;                                           /* excitation for SWB TBE               */
    Word16 allow_cn_step_fx;
    Word16 int_fs_fx;
    Word32 L_epsP[2];
    /* SC-VBR - back-up memories for LSF quantizer and synthesis filter */
    Word16 mCb1_fx, pstreaklen_fx;
    Word16 mem_MA_fx[M], mem_AR_fx[M], lsp_new_bck_fx[M], lsf_new_bck_fx[M], lsp_mid_bck_fx[M], mem_syn_bck_fx[M];
    Word32 Bin_E_fx[L_FFT], Bin_E_old_fx[L_FFT/2];
    Word16 clip_var_fx, mem_w0_bck_fx, streaklimit_fx;
    Word16 T_op_fx[3];
    Word16 nb_bits;
    Word16 indice;
    Word16 tmp16;
    Word16 enr_index;
    Word16 enr, maxv, scale, att;
    Word16 hi, lo;
    Word16 *pt_res;
    Word32 L_tmp, L_ener;

    Word16 tilt_code_bck_fx;
    Word32 gc_threshold_bck_fx;
    Word16 clip_var_bck_fx[6];
    Word16 next_force_sf_bck_fx;
    Word32 q_env[NUM_ENV_CNG];
    Word16 sid_bw=-1;
    Word16 exc3_fx[L_FRAME16k];
    Word16 syn1_fx[L_FRAME16k];



    /*------------------------------------------------------------------*
     * Initialization
     *------------------------------------------------------------------*/

    Es_pred_fx = 0;
    move16();

    Copy( pitch, T_op_fx, 3 );

    /* convert pitch values to 16kHz domain */
    IF ( sub(st_fx->L_frame_fx,L_FRAME16k) == 0 )
    {
        /*T_op[0] = (short)(T_op[0] * 1.25f + 0.5f);*/
        T_op_fx[0] = round_fx(L_mac(L_shl(T_op_fx[0],16), T_op_fx[0], 8192));
        /*T_op[1] = (short)(T_op[1] * 1.25f + 0.5f);*/
        T_op_fx[1] = round_fx(L_mac(L_shl(T_op_fx[1],16), T_op_fx[1], 8192));
        T_op_fx[2] = T_op_fx[1];
        move16();
    }
    exc_fx = old_exc_fx + L_EXC_MEM;                                  /* pointer to excitation signal in the current frame */
    Copy( mem->old_exc, old_exc_fx, L_EXC_MEM );

    bwe_exc_fx = old_bwe_exc_fx + PIT16k_MAX * 2;                     /* pointer to BWE excitation signal in the current frame */
    Copy( st_fx->old_bwe_exc_fx, old_bwe_exc_fx, PIT16k_MAX * 2);

    st_fx->bpf_off_fx = 0;
    move16();
    test();
    IF( sub(st_fx->last_core_fx,HQ_CORE) == 0 || sub(st_fx->last_codec_mode,MODE2)==0)
    {
        /* in case of HQ->ACELP switching, do not apply BPF */
        st_fx->bpf_off_fx = 1;
        move16();
        st_fx->Last_frame_ener_fx = MAX_32;
        move32();
    }

    /* force safety-net LSFQ in the first frames after CNG segment */
    if( L_sub(st_fx->last_core_brate_fx,SID_2k40) <= 0 )
    {
        st_fx->Nb_ACELP_frames_fx = 0;
        move16();
    }
    st_fx->Nb_ACELP_frames_fx = add(st_fx->Nb_ACELP_frames_fx,1);

    int_fs_fx = INT_FS_16k_FX;
    move16();
    if( sub(st_fx->L_frame_fx, L_FRAME) == 0)
    {
        int_fs_fx = INT_FS_FX;
        move16();
    }

    tmp_noise_fx = 0;
    move16();
    tc_subfr_fx = 0;
    move16();

    /* SC-VBR temporary variables */
    mCb1_fx = 0;
    move16();
    pstreaklen_fx = 0;
    move16();
    clip_var_fx = 0;
    move16();
    mem_w0_bck_fx = 0;
    move16();
    streaklimit_fx = 0;
    move16();

    /* channel-aware mode */
    reset_rf_indices(st_fx);

    /*-----------------------------------------------------------------*
     * ACELP@12k8 / ACELP@16k switching
     *-----------------------------------------------------------------*/
    test();
    IF( sub(st_fx->last_L_frame_fx,st_fx->L_frame_fx) != 0 && sub(st_fx->last_core_fx,HQ_CORE) != 0 )
    {
        /* in case of switching, do not apply BPF */
        st_fx->bpf_off_fx = 1;
        move16();

        /* force safety-net LSFQ in the first frames after ACELP@12k8/ACELP@16k switching */
        st_fx->Nb_ACELP_frames_fx = 1;
        move16();

        /* convert old quantized LSP vector */
        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            IF(L_sub(st_fx->input_Fs_fx, 8000) > 0)    /* 16 kHz core never allowed in with NB sampling frequency */
            {
                st_fx->rate_switching_reset=lsp_convert_poly_fx( st_fx->lsp_old_fx, st_fx->L_frame_fx, 0 );

            }
        }
        ELSE
        {
            st_fx->rate_switching_reset=st_fx->rate_switching_reset_16kHz;
            move16();
            Copy( st_fx->lsp_old16k_fx, st_fx->lsp_old_fx, M );
        }

        /* convert old quantized LSF vector */
        lsp2lsf_fx( st_fx->lsp_old_fx, st_fx->lsf_old_fx, M, int_fs_fx );

        /* interpolation of unquantized ISPs */
        if(st_fx->rate_switching_reset)
        {
            /*extrapolation in case of unstable LSP*/
            int_lsp4_fx( st_fx->L_frame_fx, lsp_mid_fx, lsp_mid_fx, lsp_new_fx, A_fx, M, 0, 0 );
        }
        else
        {
            int_lsp4_fx( st_fx->L_frame_fx, st_fx->lsp_old_fx, lsp_mid_fx, lsp_new_fx, A_fx, M, 0, 0 );
        }

        /* Reset LPC mem */
        Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
        set16_fx( st_fx->mem_MA_fx,0, M );

        /* update synthesis filter memories */
        synth_mem_updt2( st_fx->L_frame_fx, st_fx->last_L_frame_fx, mem->old_exc, mem->mem_syn_r,mem->mem_syn2, mem->mem_syn, ENC );
        Copy( mem->old_exc, old_exc_fx, L_EXC_MEM );
        Copy( mem->mem_syn2,st_fx->mem_syn1_fx, M );
        Copy( mem->mem_syn2, mem->mem_syn3, M );

        /* update Aw[] coefficients */
        weight_a_subfr_fx( shr(st_fx->L_frame_fx,6), A_fx, Aw_fx, st_fx->gamma, M );

    }

    test();
    test();
    if(sub(st_fx->last_bwidth_fx,NB)==0 && sub(st_fx->bwidth_fx,NB)!=0 && st_fx->ini_frame_fx!=0)
    {
        st_fx->rate_switching_reset=1;
        move16();
    }
    /*----------------------------------------------------------------*
     * Encoding of CNG frames
     *----------------------------------------------------------------*/
    test();
    IF ( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 || L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
        {
            /* Run CNG post parameter update */
            cng_params_postupd_fx( st_fx->ho_circ_ptr_fx, &st_fx->cng_buf_cnt, st_fx->cng_exc2_buf, st_fx->cng_Qexc_buf,
                                   st_fx->cng_brate_buf, st_fx->ho_env_circ_fx);
            /* encode CNG parameters */
            CNG_enc_fx( st_fx, st_fx->L_frame_fx, Aq_fx, inp_fx, ener_fx, lsp_new_fx, lsf_new_fx, &allow_cn_step_fx, st_fx->burst_ho_cnt_fx, sub(Q_new,1), q_env, &sid_bw, st_fx->exc_mem2_fx );
            /* comfort noise generation */
            CNG_exc_fx( st_fx->core_brate_fx, st_fx->L_frame_fx, &st_fx->Enew_fx, &st_fx->cng_seed_fx, exc_fx, exc2_fx, &st_fx->lp_ener_fx, st_fx->last_core_brate_fx,
                        &st_fx->first_CNG_fx, &st_fx->cng_ener_seed_fx, bwe_exc_fx, allow_cn_step_fx, &st_fx->last_allow_cn_step_fx, sub(st_fx->prev_Q_new,1), sub(Q_new,1), st_fx->num_ho_fx,
                        q_env, st_fx->lp_env_fx, st_fx->old_env_fx, st_fx->exc_mem_fx, st_fx->exc_mem1_fx, &sid_bw, &st_fx->cng_ener_seed1_fx, exc3_fx, st_fx->Opt_AMR_WB_fx );
        }
        ELSE
        {
            IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
            {
                FdCng_encodeSID( st_fx->hFdCngEnc_fx, st_fx, st_fx->preemph_fac );
                st_fx->last_CNG_L_frame_fx = st_fx->L_frame_fx;
                move16();
            }

            generate_comfort_noise_enc( st_fx, Q_new, 1 );

            FdCng_exc( st_fx->hFdCngEnc_fx->hFdCngCom, &st_fx->CNG_mode_fx, st_fx->L_frame_fx, st_fx->lsp_old_fx,
            st_fx->first_CNG_fx, st_fx->lspCNG_fx, Aq_fx, lsp_new_fx, lsf_new_fx, exc_fx, exc2_fx, bwe_exc_fx );
            IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
            {
                maxv = 0;
                move16();
                FOR(i = 0; i < st_fx->L_frame_fx; i++)
                {
                    maxv = s_max(maxv, abs_s(exc_fx[i]));
                }
                scale = norm_s(maxv);
                pt_res = exc_fx;
                L_ener = L_deposit_l(1);
                IF( sub(st_fx->L_frame_fx, L_FRAME) == 0)
                {
                    FOR (i=0; i<128; i++)
                    {
                        tmp16 = shl(*pt_res,scale);
                        L_tmp = L_mult0(tmp16, tmp16);
                        pt_res++;
                        tmp16 = shl(*pt_res,scale);
                        L_tmp = L_mac0(L_tmp, tmp16, tmp16); /* 2*(Q_new+scale) */
                        pt_res++;
                        L_ener = L_add(L_ener, L_shr(L_tmp, 7)); /* 2*(Q_new+scale)+1, divide by L_frame done here */
                    }
                }
                ELSE /* L_FRAME16k */
                {
                    FOR (i=0; i<160; i++)
                    {
                        tmp16 = shl(*pt_res,scale);
                        L_tmp = L_mult0(tmp16, tmp16);
                        pt_res++;
                        tmp16 = shl(*pt_res,scale);
                        L_tmp = L_mac0(L_tmp, tmp16, tmp16); /* 2*(Q_new+scale) */
                        pt_res++;
                        L_ener = L_add(L_ener, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*(Q_new+scale)+15+1-16+1, divide by L_frame done here */
                    }
                }

                hi = norm_l(L_ener);
                lo = Log2_norm_lc(L_shl(L_ener, hi));
                hi = sub(30, add(hi, shl(add(Q_new, scale), 1)));   /* log2 exp in Q2*(Q_new+scale) */
                L_tmp = L_Comp(hi, lo); /* Q16 */
                enr = round_fx(L_shl(L_tmp, 8)); /* Q8 (16+8-16) */

                /* decrease the energy in case of WB input */
                IF( sub(st_fx->bwidth_fx, NB) != 0 )
                {
                    IF( sub(st_fx->bwidth_fx,WB) == 0 )
                    {
                        IF( st_fx->CNG_mode_fx >= 0 )
                        {
                            /* Bitrate adapted attenuation */
                            att = ENR_ATT_fx[st_fx->CNG_mode_fx];
                        }
                        ELSE
                        {
                            /* Use least attenuation for higher bitrates */
                            att = ENR_ATT_fx[4];
                        }
                    }
                    ELSE
                    {
                        att = 384;
                        move16();/*Q8*/
                    }
                    enr = sub(enr, att );
                }

                /* calculate the energy quantization index */
                enr_index = add(enr, 512 /* Q8(2.0) */);   /* enr + 2.0 */
                enr_index = extract_l(L_shr(L_mult0(enr_index, STEP_SID_FX), 12+8));   /* Q0 (8+12-(8+12)) */

                /* limit the energy quantization index */
                enr_index = s_min(enr_index, 127);
                enr_index = s_max(enr_index, 0);

                st_fx->old_enr_index_fx = enr_index;
                move16();
            }
        }

        /* synthesis at 12.8kHz sampling rate */
        syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc2_fx, syn_fx, mem->mem_syn2, 1, Q_new, st_fx->Q_syn );

        IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
        {
            syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc3_fx, syn1_fx, mem->mem_syn3, 1, Q_new, st_fx->Q_syn );
        }
        /* reset the encoder */
        CNG_reset_enc_fx( st_fx, mem, pitch_buf_fx, voice_factors_fx );

        IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
        {
            /* update st->mem_syn1 for ACELP core switching */
            Copy( mem->mem_syn3, st_fx->mem_syn1_fx, M );

            /* update ACELP core synthesis filter memory */
            Copy( mem->mem_syn3, mem->mem_syn, M );

            /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
            Copy( syn1_fx + st_fx->L_frame_fx - L_SYN_MEM, mem->mem_syn_r, L_SYN_MEM );

            /* save and delay synthesis to be used by SWB BWE */
            save_old_syn_fx( st_fx->L_frame_fx, syn1_fx, old_syn_12k8_16k_fx, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx );
        }
        ELSE
        {
            /* update st->mem_syn1 for ACELP core switching */
            Copy( mem->mem_syn2, st_fx->mem_syn1_fx, M );

            /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
            Copy( syn_fx + st_fx->L_frame_fx - L_SYN_MEM, mem->mem_syn_r, L_SYN_MEM );

            /* save and delay synthesis to be used by SWB BWE */
            save_old_syn_fx( st_fx->L_frame_fx, syn_fx, old_syn_12k8_16k_fx, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx );
        }
    }

    /*----------------------------------------------------------------*
     * Encoding of all other frames
     *----------------------------------------------------------------*/

    ELSE
    {
        /*-----------------------------------------------------------------*
         * After inactive period, use the most up-to-date ISPs
         *-----------------------------------------------------------------*/

        test();
        IF ( L_sub(st_fx->last_core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st_fx->last_core_brate_fx,SID_2k40) == 0 )
        {
            Copy( st_fx->lspCNG_fx, st_fx->lsp_old_fx, M );

            lsp2lsf_fx( st_fx->lspCNG_fx, st_fx->lsf_old_fx, M, int_fs_fx );
        }

        /*-----------------------------------------------------------------*
         * Reset higher ACELP pre-quantizer in case of switching
         *-----------------------------------------------------------------*/
        IF( !st_fx->use_acelp_preq )
        {
            st_fx->mem_deemp_preQ_fx  = 0;
            move16();
            st_fx->mem_preemp_preQ_fx = 0;
            move16();
            st_fx->last_nq_preQ_fx = 0;
            move16();
        }

        st_fx->use_acelp_preq = 0;
        move16();

        /*-----------------------------------------------------------------*
         * LSF Quantization
         * A[z] calculation
         *-----------------------------------------------------------------*/

        /* SC-VBR - back-up memories for LSF quantizer and synthesis filter */
        lsf_syn_mem_backup_fx( st_fx, &(st_fx->LPDmem), &tilt_code_bck_fx, &gc_threshold_bck_fx, clip_var_bck_fx, &next_force_sf_bck_fx,
        lsp_new_fx, lsf_new_fx, lsp_mid_fx, &clip_var_fx, mem_AR_fx, mem_MA_fx, lsp_new_bck_fx, lsf_new_bck_fx,
        lsp_mid_bck_fx, &mCb1_fx, Bin_E_fx, Bin_E_old_fx, mem_syn_bck_fx, &mem_w0_bck_fx, &streaklimit_fx, &pstreaklen_fx);

        lsf_enc_fx( st_fx,  st_fx->L_frame_fx, coder_type_fx, lsf_new_fx, lsp_new_fx, lsp_mid_fx, Aq_fx, &st_fx->stab_fac_fx,
        st_fx->Nb_ACELP_frames_fx, Q_new );

        /*---------------------------------------------------------------*
         * Calculation of LP residual (filtering through A[z] filter)
         *---------------------------------------------------------------*/

        calc_residu_fx( st_fx, inp_fx, res_fx, Aq_fx, vad_hover_flag_fx );

        /*---------------------------------------------------------------*
         * Calculation of prediction for scaled innovation energy
         * (for memory-less gain quantizer)
         *---------------------------------------------------------------*/
        test();
        test();
        test();
        test();
        test();
        test();
        IF( ( sub(coder_type_fx,UNVOICED) != 0 && sub(coder_type_fx,AUDIO) != 0 && sub(coder_type_fx,INACTIVE) != 0
        && !(L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0 && sub(coder_type_fx,TRANSITION) != 0) )
        || (sub(coder_type_fx,INACTIVE) == 0 && L_sub(st_fx->total_brate_fx,ACELP_32k) >= 0) )
        {
            nb_bits = Es_pred_bits_tbl[BIT_ALLOC_IDX_fx(st_fx->core_brate_fx, coder_type_fx, -1, -1)];
            move16();
            Es_pred_enc_fx( &Es_pred_fx, &indice, st_fx->L_frame_fx, res_fx, voicing_fx, nb_bits, 0, Q_new );
            push_indice_fx( st_fx, IND_ES_PRED, indice, nb_bits );
        }


        /*------------------------------------------------------------*
         * Encode excitation according to coding type
         *------------------------------------------------------------*/
        test();
        test();
        IF( st_fx->nelp_mode_fx )
        {
            /* SC-VBR - NELP frames */
            encod_nelp_fx( st_fx, mem, inp_fx, Aw_fx, Aq_fx, res_fx, syn_fx, &tmp_noise_fx, exc_fx, exc2_fx, pitch_buf_fx,
            voice_factors_fx, bwe_exc_fx, Q_new, shift);
        }
        ELSE IF( sub(coder_type_fx,UNVOICED) == 0 )
        {
            /* UNVOICED frames (Gauss. excitation) */
            encod_unvoiced_fx( st_fx, mem, inp_fx, Aw_fx, Aq_fx,  vad_flag_fx, res_fx, syn_fx,
            &tmp_noise_fx, exc_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx,Q_new,shift );
        }
        ELSE IF( sub(coder_type_fx,TRANSITION) == 0)
        {
            tc_subfr_fx = encod_tran_fx( st_fx, mem, st_fx->L_frame_fx, inp_fx, Aw_fx, Aq_fx, coder_type_fx, Es_pred_fx, T_op_fx, voicing_fx, res_fx, syn_fx,
            exc_fx, exc2_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx, gsc_attack_flag_fx, unbits_fx, sharpFlag_fx, shift, Q_new  );
            move16();
        }
        ELSE IF( st_fx->ppp_mode_fx )
        {
            encod_ppp_fx( st_fx, mem, inp_fx, Aw_fx, Aq_fx, &coder_type_fx, sharpFlag_fx, T_op_fx, voicing_fx,
            res_fx, syn_fx, exc_fx, exc2_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx, Q_new, shift);

            IF( st_fx->bump_up_fx )   /* PPP failed, bump up */
            {
                /* restore memories of LSF quantizer and synthesis filter */
                lsf_syn_mem_restore_fx( st_fx, &(st_fx->LPDmem), tilt_code_bck_fx, gc_threshold_bck_fx, clip_var_bck_fx, next_force_sf_bck_fx,
                lsp_new_fx, lsf_new_fx, lsp_mid_fx, clip_var_fx, mem_AR_fx, mem_MA_fx, lsp_new_bck_fx, lsf_new_bck_fx,
                lsp_mid_bck_fx, mCb1_fx, Bin_E_fx,Bin_E_old_fx,mem_syn_bck_fx, mem_w0_bck_fx, streaklimit_fx, pstreaklen_fx );

                /* redo LSF quantization */
                lsf_enc_fx( st_fx, st_fx->L_frame_fx, coder_type_fx, lsf_new_fx, lsp_new_fx, lsp_mid_fx, Aq_fx, &st_fx->stab_fac_fx, st_fx->Nb_ACELP_frames_fx, Q_new );

                /* recalculation of LP residual (filtering through A[z] filter) */
                calc_residu_fx( st_fx, inp_fx, res_fx, Aq_fx, 0 );

                /* VOICED frames in SC-VBR when bumped up*/
                encod_gen_voic_fx( st_fx, mem, st_fx->L_frame_fx, sharpFlag_fx, inp_fx, Aw_fx, Aq_fx, coder_type_fx, Es_pred_fx, T_op_fx, voicing_fx, res_fx, syn_fx,
                exc_fx, exc2_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx, unbits_fx, shift, Q_new );
            }
        }
        ELSE IF( sub(coder_type_fx,AUDIO) == 0 || ( sub(coder_type_fx,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_24k40) <= 0 ) )
        {
            /* AUDIO and INACTIVE frames (coded by GSC technology) */
            encod_audio_fx( st_fx, mem, inp_fx, Aw_fx, Aq_fx, T_op_fx, voicing_fx, res_fx, syn_fx, exc_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx,
                            gsc_attack_flag_fx, coder_type_fx, lsf_new_fx, &tmp_noise_fx, Q_new , shift);
        }
        ELSE
        {
            /* GENERIC, VOICED and INACTIVE frames (coded by AVQ technology) */
            encod_gen_voic_fx( st_fx, mem, st_fx->L_frame_fx, sharpFlag_fx, inp_fx, Aw_fx, Aq_fx, coder_type_fx, Es_pred_fx, T_op_fx, voicing_fx, res_fx, syn_fx,
            exc_fx, exc2_fx, pitch_buf_fx, voice_factors_fx, bwe_exc_fx, unbits_fx, shift, Q_new );
        }

        /* update st->mem_syn1 for ACELP core switching */
        Copy( mem->mem_syn, st_fx->mem_syn1_fx, M );

        /* update old synthesis buffer - needed for ACELP internal sampling rate switching */
        Copy( syn_fx + st_fx->L_frame_fx - L_SYN_MEM, mem->mem_syn_r, L_SYN_MEM );

        /* save and delay synthesis to be used by SWB BWE */
        save_old_syn_fx( st_fx->L_frame_fx, syn_fx, old_syn_12k8_16k_fx, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx );

        /*--------------------------------------------------------------------------------------*
         * Addition of complementary information in UC frame or VAD information in AMR-WB IO mode
         *--------------------------------------------------------------------------------------*/

        IF ( sub(st_fx->nelp_mode_fx,1) != 0 )
        {
            L_epsP[0] = L_Comp(epsP_h_fx[2],epsP_l_fx[2]);
            move32();
            L_epsP[1] = L_Comp(epsP_h_fx[M],epsP_l_fx[M]);
            move32();
            Copy(exc_fx, exc2_fx, st_fx->L_frame_fx);
            stat_noise_uv_enc_fx( st_fx, coder_type_fx, L_epsP, lsp_new_fx, lsp_mid_fx, Aq_fx, exc2_fx, Q_new );
        }

        /*-----------------------------------------------------------------*
         * Encode supplementary information for Frame Error Concealment
         *-----------------------------------------------------------------*/

        FEC_encode_fx( st_fx, syn_fx, coder_type_fx, st_fx->clas_fx, pitch_buf_fx, res_fx, &st_fx->Last_pulse_pos_fx,
        st_fx->L_frame_fx, st_fx->total_brate_fx, st_fx->core_brate_fx, Q_new, shift );

        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            Copy( Aq_fx+2*(M+1), st_fx->cur_sub_Aq_fx, (M+1) );
        }
        ELSE
        {
            Copy( Aq_fx+3*(M+1), st_fx->cur_sub_Aq_fx, (M+1) );
        }

    }   /* end of active inp coding */

    /*-----------------------------------------------------------------*
     * Write ACELP unused bits
     *-----------------------------------------------------------------*/

    test();
    test();
    IF ( L_sub(st_fx->core_brate_fx,SID_2k40) != 0 && L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st_fx->core_brate_fx,PPP_NELP_2k80) != 0 )
    {
        /* reserved bits */
        test();
        test();
        IF ( sub(coder_type_fx,AUDIO) == 0 || ( sub(coder_type_fx,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_24k40) <= 0 ) )
        {
            nBits = 0;
            move16();
        }
        ELSE IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            nBits = reserved_bits_tbl[BIT_ALLOC_IDX_fx(st_fx->core_brate_fx, coder_type_fx, -1, TC_SUBFR2IDX_fx(tc_subfr_fx))];
            move16();
        }
        ELSE
        {
            nBits = 0;
            move16();
        }
        WHILE( nBits > 0 )
        {
            i = s_min(nBits, 16);
            push_indice_fx( st_fx, IND_UNUSED, 0, i );
            nBits = sub(nBits,i);
        }
    }

    /*Update MODE2 core switching memory*/
    tmp16 = mem->syn[M];
    move16();
    E_UTIL_deemph2( sub(Q_new,1), syn_fx, st_fx->preemph_fac, st_fx->L_frame_fx, &tmp16 );
    Copy(syn_fx+st_fx->L_frame_fx-M-1, mem->syn, M+1);


    /*-----------------------------------------------------------------*
     * Apply non linearity in case of SWB TBE
     *-----------------------------------------------------------------*/
    test();
    IF (sub(st_fx->last_Opt_SC_VBR_fx,1)==0 && st_fx->Opt_SC_VBR_fx==0)
    {
        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
        set16_fx( st_fx->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );
    }

    IF( !st_fx->Opt_SC_VBR_fx )
    {
        /* Apply a non linearity to the SHB excitation */
        non_linearity_fx( bwe_exc_fx, bwe_exc_extended_fx, L_FRAME32k, &st_fx->bwe_non_lin_prev_scale_fx, Q_new, coder_type_fx, voice_factors_fx, st_fx->L_frame_fx);
    }
    test();
    if ( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 || L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
    }

    /*-----------------------------------------------------------------*
     * Updates
     *-----------------------------------------------------------------*/
    updt_enc_fx( st_fx, st_fx->L_frame_fx, coder_type_fx, old_exc_fx, pitch_buf_fx,
                 Es_pred_fx,Aq_fx, lsf_new_fx, lsp_new_fx, old_bwe_exc_fx );

    test();
    IF( (st_fx->Opt_DTX_ON_fx != 0 ) && (L_sub(st_fx->core_brate_fx,SID_2k40) > 0) )
    {
        /* update CNG parameters in active frames */
        cng_params_upd_fx( lsp_new_fx, exc_fx, st_fx->L_frame_fx, &st_fx->ho_circ_ptr_fx, st_fx->ho_ener_circ_fx,
                           &st_fx->ho_circ_size_fx, st_fx->ho_lsp_circ_fx, Q_new, ENC, NULL, &st_fx->cng_buf_cnt,
                           st_fx->cng_exc2_buf, st_fx->cng_Qexc_buf, st_fx->cng_brate_buf, st_fx->last_active_brate_fx );

        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            /* store LSPs@16k, potentially to be used in CNG@16k */
            Copy( st_fx->lsp_old16k_fx, &(st_fx->ho_lsp_circ2_fx[(st_fx->ho_circ_ptr_fx)*M]), M );
        }

        /* Set 16k LSP flag for CNG buffer */
        st_fx->ho_16k_lsp_fx[st_fx->ho_circ_ptr_fx] = 0;
        move16();
        if(sub(st_fx->L_frame_fx, L_FRAME) != 0)
        {
            st_fx->ho_16k_lsp_fx[st_fx->ho_circ_ptr_fx] = 1;
            move16();
        }

        /* efficient DTX hangover control */
        IF ( sub(st_fx->burst_ho_cnt_fx,1) > 0 )
        {
            dtx_hangover_control_fx( st_fx, lsp_new_fx );
        }
    }

    /* SC-VBR update of average data rate */
    IF ( sub(vad_flag_fx,1) == 0 )
    {
        update_average_rate_fx( st_fx );

    }

    return;
}
