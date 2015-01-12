/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"            /* Compilation switches                   */
#include "cnst_fx.h"            /* Common constants                       */
#include "rom_com_fx.h"         /* Static table prototypes                */
#include "prot_fx.h"            /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*==========================================================================*/
/* FUNCTION      : void acelp_core_dec_fx ()                                */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :  ACELP core decoder                                      */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/*  _ Word16 coder_type_fx        i  : coder type                           */

/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*   _ Word16 *voice_factors      o  : voicing factors      Q15             */
/*   _ Word16 old_syn_12k8_16k[]  o  : intermediate ACELP    Q_syn2-1       */
/*          synthesis at 12.8kHz or 16kHz to be used by SWB BWE             */
/*   _ Word16 synth_out[]         o  : synthesis        Q_syn2-1            */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*   _ Decoder_State_fx *st_fx:                                             */
/*   _ Word16 bwe_exc_extended[]       i/o: bandwidth extended excitation Q0*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*           _ None                                                         */
/*--------------------------------------------------------------------------*/
/* CALLED FROM : RX                                                         */
/*==========================================================================*/

void acelp_core_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure                    */
    Word16 synth_out[],                 /* o  : synthesis                                  */
    Word32 bwe_exc_extended[],          /* i/o: bandwidth extended excitation              */
    Word16 *voice_factors,              /* o  : voicing factors                            */
    Word16 old_syn_12k8_16k[],          /* o  : intermediate ACELP synthesis for SWB BWE   */
    Word16 coder_type_fx,               /* i  : coder type                                 */
    Word16 sharpFlag,
    Word16 pitch_buf_fx[NB_SUBFR16k],   /* o  : floating pitch for each subframe           */
    Word16 *unbits,                     /* o  : number of unused bits                      */
    Word16 *sid_bw                      /* o  : 0-NB/WB, 1-SWB SID                         */
)
{
    Word16 old_exc_fx[L_EXC_DEC] = {0}, *exc_fx;        /* excitation signal buffer (Q0)         */
    Word16 syn_fx_tmp[L_FRAME_16k+L_SUBFR] = {0}, *syn_fx = syn_fx_tmp+L_SUBFR; /* synthesis signal buffer               */
    Word16 temp_buf[L_FRAME16k + L_SYN_MEM];
    Word16 output_frame;                      /* frame length at output sampling freq. */
    Word16 mem_tmp_fx[M];                     /* temporary synthesis filter memory     */
    Word32 enr_q_fx;                          /* E information for FER protection      */
    Word16 tmp_noise_fx;                      /* Long term temporary noise energy      */
    Word16 i, int_fs;
    Word16 tc_subfr_fx;
    Word16 allow_cn_step_fx;
    Word16 temp_buf_fx[L_FRAME16k + L_SYN_MEM];

    Word16 Aq_fx[NB_SUBFR16k*(M+1)]= {0}; /*Q12*/
    Word16 Es_pred_fx;  /*Q8*/
    Word16 old_bwe_exc_fx[((PIT16k_MAX + (L_FRAME16k+1) + L_SUBFR16k) * 2)] = {0}; /* excitation buffer */
    Word16 old_exc2_fx[L_FRAME16k + L_EXC_MEM], *exc2_fx;      /* total excitation buffer               */
    Word16 *bwe_exc_fx;
    Word16 lsf_new_fx[M];                     /* LSFs at the end of the frame    */
    Word16 lsp_new_fx[M];                     /* LSPs at the end of the frame    */
    Word16 lsp_mid_fx[M];                     /* LSPs in the middle of the frame */
    Word16 FEC_pitch_fx; /*Q6*/
    Word16 last_pulse_pos;
    Word16 T0_tmp;
    Word16 do_WI_fx;
    Word16 dct_buffer_fx[DCT_L_POST];
    Word16 exc_buffer_fx[DCT_L_POST];
    Word16 dct_exc_tmp[L_FRAME16k];
    Word16 qdct;
    Word16  delta_mem_scale;
    Word16 bpf_error_signal[L_FRAME16k];
    CLDFB_SCALE_FACTOR scaleFactor;
    Word32 workBuffer[128*3];
    Word32 q_env[20];
    Word16 exc3_fx[L_FRAME16k];
    Word16 syn1_fx_tmp[L_FRAME16k+2], *syn1_fx;
    Word32 *realBuffer[CLDFB_NO_COL_MAX], *imagBuffer[CLDFB_NO_COL_MAX];
    Word16 gain_buf[NB_SUBFR16k]; /*Q14*/
    Word16 syn_fx_tmp2[L_FRAME_16k];
    Word16 pitch_buf_tmp[NB_SUBFR16k];
    Word16 k;
    Word16 update_flg;
    Word32 realBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
    Word32 imagBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];

    FOR( i=0; i<CLDFB_NO_COL_MAX; i++ )
    {
        set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        realBuffer[i] = realBufferTmp[i];
        move32();
        imagBuffer[i] = imagBufferTmp[i];
        move32();
    }

    /*----------------------------------------------------------------*
     * Initialization
     *----------------------------------------------------------------*/
    set16_fx(syn_fx_tmp, 0, L_SUBFR);
    syn_fx = syn_fx_tmp+L_SUBFR;
    syn1_fx_tmp[0] = 0;
    move16();
    syn1_fx_tmp[1] = 0;
    move16();
    syn1_fx = syn1_fx_tmp+2;
    /*output_frame = (Word16)(st_fx->output_Fs_fx / 50); move16();*/
    output_frame = st_fx->output_frame_fx;
    move16();
    st_fx->bpf_off_fx = 0;
    move16();
    if( sub(st_fx->last_core_fx,HQ_CORE) == 0 )
    {
        /* in case of HQ->ACELP switching, do not apply BPF */
        st_fx->bpf_off_fx = 1;
        move16();
        /* in case of core switching, reset post-filter memories */
        st_fx->pfstat.on = 0;
        move16();
        /* reset the GSC pre echo energy threshold in case of switching */
        st_fx->Last_frame_ener_fx = MAX_32;
        move32();
    }
    if(st_fx->prev_bfi_fx > 0)
    {
        /* reset the GSC pre echo energy threshold in case of FEC */
        st_fx->Last_frame_ener_fx = MAX_32;
        move32();
    }
    st_fx->clas_dec = st_fx->last_good_fx;
    move16();
    enr_q_fx = 0;
    move16();
    Es_pred_fx = 0;
    move16();
    tmp_noise_fx = 0;

    Copy( st_fx->old_exc_fx, old_exc_fx, L_EXC_MEM_DEC );
    exc_fx = old_exc_fx + L_EXC_MEM_DEC;
    move16();
    Copy( st_fx->old_exc2_fx, old_exc2_fx, L_EXC_MEM );
    exc2_fx = old_exc2_fx + L_EXC_MEM;
    Copy( st_fx->old_bwe_exc_fx, old_bwe_exc_fx, PIT16k_MAX * 2);
    bwe_exc_fx = old_bwe_exc_fx + PIT16k_MAX * 2;
    move16();
    last_pulse_pos = 0;
    move16();
    do_WI_fx = 0;
    move16();
    st_fx->GSC_noisy_speech_fx = 0;
    move16();
    st_fx->relax_prev_lsf_interp_fx = 0;
    move16();

    set16_fx( gain_buf, 0, NB_SUBFR16k );
    IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
    {
        st_fx->gamma = GAMMA1;
        move16();
        st_fx->preemph_fac = PREEMPH_FAC;
        move16();
        int_fs = INT_FS_FX;
        move16();
    }
    ELSE
    {
        st_fx->gamma = GAMMA16k;
        move16();
        st_fx->preemph_fac = PREEMPH_FAC_16k;
        move16();
        int_fs = INT_FS_16k;
        move16();
    }

    /* reset post-filter in case post-filtering was off in previous frame  */
    if( st_fx->pfstat.on == 0 )
    {
        st_fx->pfstat.reset = 1;
        move16();
    }



    /*----------------------------------------------------------------*
     * Updates in case of internal sampling rate switching
     *----------------------------------------------------------------*/
    test();
    IF( sub(st_fx->last_L_frame_fx,st_fx->L_frame_fx) != 0 && sub(st_fx->last_core_fx, HQ_CORE) != 0 )
    {
        if( st_fx->pfstat.on != 0 )
        {
            Word16 mem_syn_r_size_old, mem_syn_r_size_new;

            mem_syn_r_size_old = shr(st_fx->last_L_frame_fx, 4);
            mem_syn_r_size_new = shr(st_fx->L_frame_fx, 4);
            lerp( st_fx->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_old, st_fx->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            lerp( st_fx->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_old, st_fx->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
        }

        /* convert quantized LSP vector */
        st_fx->rate_switching_reset=lsp_convert_poly_fx( st_fx->lsp_old_fx, st_fx->L_frame_fx, 0);
        /* convert old quantized LSF vector */
        lsp2lsf_fx( st_fx->lsp_old_fx, st_fx->lsf_old_fx, M, int_fs );

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
        synth_mem_updt2( st_fx->L_frame_fx, st_fx->last_L_frame_fx, st_fx->old_exc_fx, st_fx->mem_syn_r, st_fx->mem_syn2_fx, NULL, DEC );
        Copy( st_fx->old_exc_fx, old_exc_fx, L_EXC_MEM_DEC );
        Copy_Scale_sig(st_fx->mem_syn2_fx, st_fx->mem_syn1_fx, M, sub(-1,st_fx->Q_syn)); /*Q-1*/

        Copy( st_fx->mem_syn2_fx, st_fx->mem_syn3_fx, M );

        /* update buffer of old subframe pitch values */
        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            FOR( i=NB_SUBFR16k-NB_SUBFR; i<NB_SUBFR16k; i++ )
            {
                /*((float)FAC_16k/(float)FAC_12k8) * st_fx->old_pitch_buf[i]*/
                st_fx->old_pitch_buf_fx[i-1] = Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 26214);
                move32();
            }

            FOR( i=2*NB_SUBFR16k-NB_SUBFR; i<2*NB_SUBFR16k; i++ )
            {
                st_fx->old_pitch_buf_fx[i-2] =  Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 26214);
                move32();
            }

            IF( sub(st_fx->bfi_pitch_frame_fx,L_FRAME) != 0 )
            {
                st_fx->bfi_pitch_fx = mult_r(26214, st_fx->bfi_pitch_fx);
                st_fx->bfi_pitch_frame_fx = L_FRAME;
                move16();
            }
        }
        ELSE
        {
            FOR( i=2*NB_SUBFR-1; i>=NB_SUBFR; i-- )
            {
                /*((float)FAC_12k8/(float)FAC_16k) * st_fx->old_pitch_buf[i];*/
                st_fx->old_pitch_buf_fx[i+2] = L_add(st_fx->old_pitch_buf_fx[i], Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 8192));
                move32();
            }
            st_fx->old_pitch_buf_fx[NB_SUBFR+1] = st_fx->old_pitch_buf_fx[NB_SUBFR+2];
            move32();

            FOR( i=NB_SUBFR-1; i>=0; i-- )
            {
                /*((float)FAC_12k8/(float)FAC_16k) * st_fx->old_pitch_buf[i];*/
                st_fx->old_pitch_buf_fx[i+1] = L_add(st_fx->old_pitch_buf_fx[i], Mpy_32_16_1(st_fx->old_pitch_buf_fx[i], 8192));
                move32();
            }
            st_fx->old_pitch_buf_fx[0] = st_fx->old_pitch_buf_fx[1];
            move32();

            IF( sub(st_fx->bfi_pitch_frame_fx,L_FRAME16k) != 0 )
            {
                /*st_fx->bfi_pitch *= ((float)FAC_12k8/(float)FAC_16k);*/
                st_fx->bfi_pitch_fx = add(st_fx->bfi_pitch_fx, mult_r(st_fx->bfi_pitch_fx,8192));
                st_fx->bfi_pitch_frame_fx = L_FRAME16k;
                move16();
            }
        }
    }

    test();
    test();
    if(sub(st_fx->last_bwidth_fx,NB)==0 && sub(st_fx->bwidth_fx,NB)!=0 && st_fx->ini_frame_fx!=0)
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
         * Decoding of TC subframe clasification
         *----------------------------------------------------------------*/

        tc_subfr_fx = -1;
        move16();
        if( sub(coder_type_fx,TRANSITION) == 0 )
        {
            tc_subfr_fx = tc_classif_fx( st_fx, st_fx->L_frame_fx );
            move16();
        }
        /*----------------------------------------------------------------*
         * Decoding of inactive CNG frames
         *----------------------------------------------------------------*/
        test();
        IF ( st_fx->core_brate_fx == FRAME_NO_DATA || L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
        {
            /* decode CNG parameters */
            IF ( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
            {

                CNG_dec_fx( st_fx, st_fx->L_frame_fx, Aq_fx, st_fx->core_brate_fx, lsp_new_fx, lsf_new_fx, &allow_cn_step_fx, sid_bw, q_env );

                /* comfort noise generation */
                CNG_exc_fx( st_fx->core_brate_fx, st_fx->L_frame_fx, &st_fx->Enew_fx, &st_fx->cng_seed_fx, exc_fx, exc2_fx, &st_fx->lp_ener_fx, st_fx->last_core_brate_fx,
                            &st_fx->first_CNG_fx, &(st_fx->cng_ener_seed_fx), bwe_exc_fx, allow_cn_step_fx, &st_fx->last_allow_cn_step_fx, st_fx->prev_Q_exc, st_fx->Q_exc, st_fx->num_ho_fx,
                            q_env, st_fx->lp_env_fx, st_fx->old_env_fx, st_fx->exc_mem_fx, st_fx->exc_mem1_fx, sid_bw, &st_fx->cng_ener_seed1_fx, exc3_fx ,st_fx->Opt_AMR_WB_fx );
            }
            ELSE
            {
                IF( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
                {
                    FdCng_decodeSID(st_fx->hFdCngDec_fx->hFdCngCom, st_fx);
                    *sid_bw=0;
                    move16();
                }

                generate_comfort_noise_dec( NULL, NULL, NULL, st_fx, &(st_fx->Q_exc), 2 );

                FdCng_exc(st_fx->hFdCngDec_fx->hFdCngCom, &st_fx->CNG_mode_fx, st_fx->L_frame_fx, st_fx->lsp_old_fx, st_fx->first_CNG_fx, st_fx->lspCNG_fx, Aq_fx, lsp_new_fx, lsf_new_fx, exc_fx, exc2_fx, bwe_exc_fx);
            }

            delta_mem_scale = 3;
            move16();
            test();
            if( L_sub(st_fx->lp_ener_fx,40) < 0 && sub(st_fx->cng_type_fx,LP_CNG) == 0 ) /* very low energy frames, less than 0.3125 */
            {
                delta_mem_scale = 0;
                move16();
            }
            i = st_fx->Q_exc;
            Rescale_exc( st_fx->dct_post_old_exc_fx, exc_fx, bwe_exc_fx, st_fx->last_exc_dct_in_fx, st_fx->L_frame_fx,
                         st_fx->L_frame_fx* HIBND_ACB_L_FAC, 0, &(st_fx->Q_exc), st_fx->Q_subfr, NULL, 0, INACTIVE);

            Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx, delta_mem_scale,
                         &st_fx->mem_deemph_fx, st_fx->pst_old_syn_fx,&st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1],&st_fx->pfstat, 0, NULL );
            Copy_Scale_sig(exc2_fx, exc2_fx, st_fx->L_frame_fx, sub(st_fx->Q_exc, i));

            /* update past excitation signals for LD music post-filter */
            Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
            Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );

            /* synthesis at 12.8kHz sampling rate */
            syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1, st_fx->Q_exc, st_fx->Q_syn );
            IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
            {
                syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc3_fx, syn1_fx, st_fx->mem_syn3_fx, 1, st_fx->Q_exc, st_fx->Q_syn );
            }

            /* reset the decoder */
            CNG_reset_dec_fx( st_fx, pitch_buf_fx, voice_factors );

            IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 )
            {
                /* update st_fx->mem_syn1 for ACELP core switching */
                Copy( st_fx->mem_syn3_fx, st_fx->mem_syn1_fx, M );
                /* update old synthesis for classification */
                Copy( syn1_fx + st_fx->L_frame_fx - L_SYN_MEM_CLAS_ESTIM, st_fx->mem_syn_clas_estim_fx, L_SYN_MEM_CLAS_ESTIM );

                /* save and delay synthesis to be used by SWB BWE */
                Copy_Scale_sig(syn1_fx, temp_buf_fx, st_fx->L_frame_fx, sub(-1,st_fx->Q_syn));
                save_old_syn_fx( st_fx->L_frame_fx, temp_buf_fx, old_syn_12k8_16k, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx);
            }
            ELSE
            {
                /* update st_fx->mem_syn1 for ACELP core switching */
                Copy( st_fx->mem_syn2_fx, st_fx->mem_syn1_fx, M );
                /* update old synthesis for classification */
                Copy( syn_fx + st_fx->L_frame_fx - L_SYN_MEM_CLAS_ESTIM, st_fx->mem_syn_clas_estim_fx, L_SYN_MEM_CLAS_ESTIM );

                /* save and delay synthesis to be used by SWB BWE */
                Copy_Scale_sig(syn_fx, temp_buf_fx, st_fx->L_frame_fx, sub(-1,st_fx->Q_syn));
                save_old_syn_fx( st_fx->L_frame_fx, temp_buf_fx, old_syn_12k8_16k, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx);
            }
        }

        /*----------------------------------------------------------------*
         * Decoding of all other frames
         *----------------------------------------------------------------*/

        ELSE
        {
            /*-----------------------------------------------------------------*
             * After CNG period, use the most up-to-date LSPs
             *-----------------------------------------------------------------*/

            test();
            IF ( st_fx->last_core_brate_fx == FRAME_NO_DATA || L_sub(st_fx->last_core_brate_fx,SID_2k40) == 0 )
            {
                Copy( st_fx->lspCNG_fx, st_fx->lsp_old_fx, M );

                lsp2lsf_fx( st_fx->lspCNG_fx, st_fx->lsf_old_fx, M, int_fs );
            }

            /*-----------------------------------------------------------------*
             * Reset higher ACELP pre-quantizer in case of switching
             *-----------------------------------------------------------------*/
            IF( !st_fx->use_acelp_preq )
            {
                st_fx->mem_preemp_preQ_fx = 0;
                move16();
                st_fx->last_nq_preQ_fx = 0;
                move16();
            }

            st_fx->use_acelp_preq = 0;
            move16();

            /*-----------------------------------------------------------------*
             * LSF de-quantization and interpolation
             *-----------------------------------------------------------------*/

            lsf_dec_fx( st_fx,
            tc_subfr_fx,
            st_fx->L_frame_fx, coder_type_fx, st_fx->bwidth_fx, Aq_fx, lsf_new_fx, lsp_new_fx, lsp_mid_fx );

            /*-----------------------------------------------------------------*
             * FEC - first good frame after lost frame(s) (possibility to correct the ACB)
             *-----------------------------------------------------------------*/

            IF( L_sub(st_fx->core_brate_fx,ACELP_11k60) >= 0 )
            {
                last_pulse_pos = 0;
                move16();

                /* decode the last glottal pulse position */
                T0_tmp = FEC_pos_dec_fx( st_fx, st_fx->L_frame_fx, coder_type_fx, st_fx->last_good_fx, &last_pulse_pos, &st_fx->clas_dec, &enr_q_fx, st_fx->core_brate_fx );
                move16();

                test();
                test();
                test();
                test();
                test();
                test();
                IF( sub(st_fx->clas_dec,SIN_ONSET) == 0 && last_pulse_pos != 0 && sub(st_fx->prev_bfi_fx,1) == 0 )
                {
                    FEC_SinOnset_fx( old_exc_fx+L_EXC_MEM_DEC-L_EXC_MEM, last_pulse_pos, T0_tmp, enr_q_fx, Aq_fx, st_fx->L_frame_fx
                    ,st_fx->Q_exc
                                   );
                }
                ELSE IF( (sub(coder_type_fx,GENERIC) == 0 || sub(coder_type_fx,VOICED) == 0) && last_pulse_pos != 0 && sub(st_fx->old_bfi_cnt_fx,1) == 0 && sub(output_frame,L_FRAME16k) == 0)
                {
                    do_WI_fx = FEC_enhACB_fx( st_fx->L_frame_fx, old_exc_fx+L_EXC_MEM_DEC-L_EXC_MEM , T0_tmp, last_pulse_pos, st_fx->bfi_pitch_fx );
                }
            }

            /*------------------------------------------------------------*
             * In case of first frame after an erasure and transition from voiced to unvoiced or inactive
             * redo the LPC interpolation
             *------------------------------------------------------------*/
            test();
            test();
            test();
            test();
            IF( st_fx->stab_fac_fx == 0 && st_fx->old_bfi_cnt_fx > 0 && sub(st_fx->clas_dec,VOICED_CLAS) != 0 && sub(st_fx->clas_dec,ONSET) != 0 && st_fx->relax_prev_lsf_interp_fx == 0 )
            {
                int_lsp4_fx(st_fx->L_frame_fx, st_fx->lsp_old_fx, lsp_mid_fx, lsp_new_fx, Aq_fx,  M, 0, 2 );
            }

            /*---------------------------------------------------------------*
             * Decoding of the scaled predicted innovation energy
             *---------------------------------------------------------------*/
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF( ( sub(coder_type_fx,UNVOICED) != 0   &&
                  sub(coder_type_fx,AUDIO) != 0     &&
                  sub(coder_type_fx,INACTIVE) != 0  &&
                  !(L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0 && sub(coder_type_fx,TRANSITION) != 0) ) ||
                (sub(coder_type_fx,INACTIVE) == 0 && L_sub(st_fx->total_brate_fx,ACELP_32k) >= 0)
              )
            {
                Es_pred_dec_fx( st_fx, &Es_pred_fx, coder_type_fx, st_fx->core_brate_fx );
            }

            /*------------------------------------------------------------*
            * Decode excitation according to coding type
            *------------------------------------------------------------*/

            test();
            test();
            IF( sub(st_fx->nelp_mode_dec_fx,1) == 0)
            {
                /* SC-VBR - NELP frames */
                Scale_sig(exc_fx-L_EXC_MEM, L_EXC_MEM, -st_fx->Q_exc);
                st_fx->Q_exc = 0;
                move16();

                decod_nelp_fx( st_fx, coder_type_fx, &tmp_noise_fx, pitch_buf_fx, exc_fx, exc2_fx,
                               voice_factors, bwe_exc_fx, &st_fx->Q_exc, st_fx->bfi_fx, gain_buf );

                Rescale_exc(st_fx->dct_post_old_exc_fx, exc_fx, NULL, st_fx->last_exc_dct_in_fx, L_FRAME, 0, (Word32)0, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, L_FRAME, coder_type_fx);
            }
            ELSE IF( sub(coder_type_fx,UNVOICED) == 0)
            {
                /* UNVOICED frames */
                decod_unvoiced_fx( st_fx, Aq_fx, coder_type_fx, &tmp_noise_fx, pitch_buf_fx, voice_factors, exc_fx, exc2_fx, bwe_exc_fx, gain_buf );
            }
            ELSE IF ( sub(st_fx->ppp_mode_dec_fx,1) == 0 )
            {
                Scale_sig(exc_fx-L_EXC_MEM, L_EXC_MEM, -st_fx->Q_exc);
                st_fx->Q_exc = 0;
                /* SC-VBR - PPP frames */
                decod_ppp_fx( st_fx, Aq_fx, pitch_buf_fx, exc_fx, exc2_fx, st_fx->bfi_fx, gain_buf, voice_factors, bwe_exc_fx );

                Rescale_exc( st_fx->dct_post_old_exc_fx, exc_fx, NULL, st_fx->last_exc_dct_in_fx, L_FRAME, 0, (Word32)0, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, L_FRAME, coder_type_fx );
            }
            ELSE IF( sub(coder_type_fx,TRANSITION ) == 0)
            {
                decod_tran_fx( st_fx, st_fx->L_frame_fx, tc_subfr_fx, Aq_fx, coder_type_fx, Es_pred_fx, pitch_buf_fx, voice_factors, exc_fx, exc2_fx, bwe_exc_fx, unbits, sharpFlag, gain_buf );
            }
            ELSE IF( sub(coder_type_fx,AUDIO) == 0|| ( sub(coder_type_fx,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_24k40) <= 0) )
            {
                decod_audio_fx( st_fx, dct_exc_tmp, Aq_fx, coder_type_fx, pitch_buf_fx, voice_factors, exc_fx, exc2_fx, bwe_exc_fx, lsf_new_fx, gain_buf );
                tmp_noise_fx = shr_r(st_fx->lp_gainc_fx,3);  /*Q0*/
            }
            ELSE
            {
                decod_gen_voic_fx( st_fx, st_fx->L_frame_fx, sharpFlag, Aq_fx, coder_type_fx, Es_pred_fx, do_WI_fx, pitch_buf_fx, voice_factors, exc_fx, exc2_fx, bwe_exc_fx, unbits, gain_buf );
            }
            /* synthesis for ACELP core switching and SWB BWE */
            syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc_fx, temp_buf_fx, st_fx->mem_syn1_fx, 1, st_fx->Q_exc, -1 );
            /* save and delay synthesis to be used by SWB BWE */
            save_old_syn_fx( st_fx->L_frame_fx, temp_buf_fx, old_syn_12k8_16k, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx );


            /*-----------------------------------------------------------------*
             * Apply energy matching when switching to inactive frames
             *-----------------------------------------------------------------*/

            Inac_swtch_ematch_fx( exc2_fx, dct_exc_tmp, st_fx->lt_ener_per_band_fx, coder_type_fx, st_fx->L_frame_fx, st_fx->core_brate_fx, st_fx->Q_exc );

            /*------------------------------------------------------------*
             * Decode information and modify the excitation signal of stationary unvoiced frames
             *------------------------------------------------------------*/
            IF ( sub(st_fx->nelp_mode_dec_fx,1) != 0 )
            {
                stat_noise_uv_dec_fx( st_fx, coder_type_fx, lsp_new_fx, lsp_mid_fx, Aq_fx, exc2_fx );
            }

            /*------------------------------------------------------------*
             * Save filter memory in case the synthesis is redone after scaling
             * Synthesis at 12k8 Hz sampling rate
             *------------------------------------------------------------*/

            /* update past excitation signals for LD music post-filter */
            Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
            Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );
            Copy( st_fx->dct_post_old_exc_fx, exc_buffer_fx, DCT_L_POST-OFFSET2 );

            test();
            IF( sub(coder_type_fx, AUDIO )== 0 && st_fx->GSC_noisy_speech_fx == 0 )
            {

                /* Extrapolation of the last future part, windowing and high resolution DCT transform */
                qdct = 0;
                Prep_music_postP_fx( exc_buffer_fx,  dct_buffer_fx, st_fx->filt_lfE_fx, st_fx->last_coder_type_fx, pitch_buf_fx,
                st_fx->LDm_enh_lp_gbin_fx, st_fx->Q_exc, &qdct );

                /* LD music post-filter */
                LD_music_post_filter_fx( dct_buffer_fx, dct_buffer_fx, st_fx->core_brate_fx, st_fx->bfi_fx, &st_fx->LDm_last_music_flag_fx, &st_fx->LDm_last_bfi_count_fx,
                st_fx->LDm_thres_fx, &st_fx->LDm_nb_thr_1_fx, &st_fx->LDm_nb_thr_3_fx, st_fx->LDm_lt_diff_etot_fx,
                &st_fx->LDm_mem_etot_fx, st_fx->LDm_enh_min_ns_gain_fx, st_fx->LDm_bckr_noise_fx, st_fx->LDm_enro_fx,
                st_fx->LDm_enh_lf_EO_fx, st_fx->LDm_enh_lp_gbin_fx, st_fx->filt_lfE_fx, &st_fx->last_nonfull_music_fx,
                &st_fx->Old_ener_Q, AUDIO, st_fx->last_coder_type_fx, qdct );

                /* Inverse DCT transform, retrieval of the aligned excitation, re-synthesis */
                Post_music_postP_fx( dct_buffer_fx, exc2_fx, st_fx->mem_syn2_fx, st_fx->mem_syn2_fx, Aq_fx, syn_fx, &st_fx->Q_exc, &st_fx->prev_Q_syn,
                &st_fx->Q_syn, st_fx->mem_syn_clas_estim_fx,0, &st_fx->mem_deemph_fx, st_fx->pst_old_syn_fx,
                &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, temp_buf_fx
                ,mem_tmp_fx
                                   );
            }
            ELSE
            {
                /* Core synthesis at 12.8kHz or 16kHz */
                Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx, 4,
                &st_fx->mem_deemph_fx, st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx,
                &st_fx->agc_mem_fx[1], &st_fx->pfstat, coder_type_fx==INACTIVE?0:1, temp_buf_fx );

                Copy( st_fx->mem_syn2_fx, mem_tmp_fx, M );

                syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1 , st_fx->Q_exc, st_fx->Q_syn);

                FOR( i = 0; i < DCT_L_POST; i++ )
                {
                    /*st_fx->filt_lfE_fx[i] = 0.3f + 0.7f * st_fx->filt_lfE_fx[i];*/
                    st_fx->filt_lfE_fx[i] = round_fx(L_mac((1228<<(16)), 22938, st_fx->filt_lfE_fx[i]));
                }
            }

            /*------------------------------------------------------------*
             * FEC - Estimate the classification information
             *------------------------------------------------------------*/

            FEC_clas_estim_fx( st_fx, st_fx->Opt_AMR_WB_fx, st_fx->L_frame_fx, &st_fx->clas_dec, coder_type_fx, pitch_buf_fx,
            &st_fx->last_good_fx, syn_fx, &st_fx->lp_ener_FER_fx, &st_fx->decision_hyst_fx,
            NULL, NULL, NULL, NULL, 0, NULL, st_fx->core_brate_fx, &st_fx->Q_syn, temp_buf_fx,
            st_fx->mem_syn_clas_estim_fx, &st_fx->classifier_Q_mem_syn, 0, 0, 0, 0, 0, 0 );

            /*------------------------------------------------------------*
             * FEC - Estimate pitch
             *------------------------------------------------------------*/

            FEC_pitch_estim_fx( st_fx->Opt_AMR_WB_fx, st_fx->L_frame_fx, st_fx->clas_dec, st_fx->last_good_fx, pitch_buf_fx, st_fx->old_pitch_buf_fx,
            &st_fx->bfi_pitch_fx, &st_fx->bfi_pitch_frame_fx, &st_fx->upd_cnt_fx, coder_type_fx );

            /*------------------------------------------------------------*
             * FEC - Smooth the speech energy evolution when recovering after a BAD frame
             * (smoothing is performed in the excitation domain and signal is resynthesized after)
             *------------------------------------------------------------*/

            k = 0;
            move16();
            FOR (i = 0; i < st_fx->L_frame_fx; i += L_SUBFR)
            {
                pitch_buf_tmp[k] = mult_r(pitch_buf_fx[k], 512);
                move16();
                k++;
            }

            FEC_scale_syn_fx( st_fx->L_frame_fx, &update_flg, st_fx->clas_dec, st_fx->last_good_fx, syn_fx, pitch_buf_tmp, st_fx->enr_old_fx, enr_q_fx, coder_type_fx, st_fx->prev_bfi_fx,
            st_fx->last_core_brate_fx, exc_fx, exc2_fx, Aq_fx, &st_fx->old_enr_LP, mem_tmp_fx, st_fx->mem_syn2_fx, st_fx->Q_exc, st_fx->Q_syn);
        }

    } /* End of GOOD FRAME */

    /*----------------------------------------------------------------*
     * BAD frame
     *----------------------------------------------------------------*/
    ELSE
    {
        /* SC-VBR */
        if ( sub(st_fx->last_nelp_mode_dec_fx,1) == 0 )
        {
            st_fx->nelp_mode_dec_fx = 1;
            move16();
        }

        /* long burst frame erasures */
        test();
        test();
        if( sub(st_fx->nbLostCmpt,5) > 0 && sub(st_fx->clas_dec,VOICED_CLAS) >= 0 && sub(st_fx->clas_dec,INACTIVE_CLAS) < 0 )
        {
            st_fx->last_good_fx = VOICED_TRANSITION;
            move16();
        }

        /* LSF estimation and A(z) calculation */
        FEC_lsf_estim_fx( st_fx, st_fx->L_frame_fx, Aq_fx, lsf_new_fx, lsp_new_fx );

        IF ( st_fx->nelp_mode_dec_fx == 1 )
        {
            /* SC-VBR */
            Scale_sig(exc_fx-L_EXC_MEM, L_EXC_MEM, -st_fx->Q_exc);
            st_fx->Q_exc = 0;
            move16();

            decod_nelp_fx( st_fx, coder_type_fx, &tmp_noise_fx, pitch_buf_fx, exc_fx, exc2_fx,
            voice_factors, bwe_exc_fx, &st_fx->Q_exc, st_fx->bfi_fx, gain_buf );

            FEC_pitch_fx = pitch_buf_fx[3];
            move16();

            Rescale_exc( st_fx->dct_post_old_exc_fx, exc_fx, NULL, st_fx->last_exc_dct_in_fx, L_FRAME, 0, (Word32)0, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, L_FRAME, coder_type_fx );
        }
        ELSE
        {
            /* calculation of excitation signal */

            FEC_exc_estim_fx( st_fx, st_fx->L_frame_fx, exc_fx, exc2_fx, dct_exc_tmp, pitch_buf_fx, voice_factors, &FEC_pitch_fx, bwe_exc_fx, lsf_new_fx, &st_fx->Q_exc, &tmp_noise_fx );

            Rescale_exc( NULL, exc_fx, bwe_exc_fx, st_fx->last_exc_dct_in_fx, st_fx->L_frame_fx, L_FRAME32k, (Word32)0,
            &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, st_fx->L_frame_fx, st_fx->last_coder_type_fx );

            tmp_noise_fx = shr_r(st_fx->lp_gainc_fx,3);  /*Q0*/

            /* SC-VBR */
            st_fx->prev_gain_pit_dec_fx = st_fx->lp_gainp_fx;
            move16(); /*Q14*/
        }

        /* synthesis for ACELP core switching and SWB BWE */
        syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc_fx, temp_buf_fx, st_fx->mem_syn1_fx, 1, st_fx->Q_exc, -1 );  /*old_syn_12k8_16k directly in q-1*/

        /* save and delay synthesis to be used by SWB BWE */
        save_old_syn_fx( st_fx->L_frame_fx, temp_buf_fx, old_syn_12k8_16k, st_fx->old_syn_12k8_16k_fx, st_fx->preemph_fac, &st_fx->mem_deemph_old_syn_fx );

        /* Apply energy matching when switching to inactive frames */
        Inac_swtch_ematch_fx( exc2_fx, dct_exc_tmp, st_fx->lt_ener_per_band_fx, coder_type_fx, st_fx->L_frame_fx, st_fx->core_brate_fx, st_fx->Q_exc );

        /* udate past excitation signals for LD music post-filter */
        Copy( st_fx->dct_post_old_exc_fx + L_FRAME, st_fx->dct_post_old_exc_fx, DCT_L_POST-L_FRAME-OFFSET2 );
        Copy( exc2_fx, st_fx->dct_post_old_exc_fx+DCT_L_POST-L_FRAME-OFFSET2, L_FRAME );

        /* synthesis at 12k8 Hz sampling rate */
        Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx, 4, &st_fx->mem_deemph_fx,
        st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, 1, temp_buf_fx );

        syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc2_fx, syn_fx, st_fx->mem_syn2_fx, 1, st_fx->Q_exc, st_fx->Q_syn);

        /* update buffer for classifier */
        Copy( exc2_fx + st_fx->L_frame_fx - L_EXC_MEM, st_fx->old_exc2_fx, L_EXC_MEM );
        Copy( syn_fx + st_fx->L_frame_fx - L_EXC_MEM, st_fx->old_syn2_fx, L_EXC_MEM );
        st_fx->prev_Q_exc_fr = st_fx->Q_exc;
        st_fx->prev_Q_syn_fr = st_fx->Q_syn;

        Copy( syn_fx + st_fx->L_frame_fx - L_SYN_MEM_CLAS_ESTIM, st_fx->mem_syn_clas_estim_fx, L_SYN_MEM_CLAS_ESTIM );

        /* estimate the pitch-synchronous speech energy per sample to be used when normal operation recovers */
        /* fer_energy( st_fx->L_frame_fx, st_fx->last_good_fx, syn_fx, FEC_pitch_fx, &st_fx->enr_old_fx, st_fx->L_frame_fx ); */
        frame_ener_fx( st_fx->L_frame_fx, st_fx->last_good_fx, syn_fx, shr(add(FEC_pitch_fx,32),6), &st_fx->enr_old_fx, st_fx->L_frame_fx, st_fx->Q_syn, 3, 0 );

        IF ( st_fx->nelp_mode_dec_fx !=1 )
        {
            /* modify the excitation signal of stationary unvoiced frames */
            stat_noise_uv_mod_fx( coder_type_fx, 0, st_fx->lsp_old_fx, lsp_new_fx, lsp_new_fx, Aq_fx, exc2_fx, st_fx->Q_exc, 1, &st_fx->ge_sm_fx,
            &st_fx->uv_count_fx, &st_fx->act_count_fx, st_fx->lspold_s_fx, &st_fx->noimix_seed_fx, &st_fx->min_alpha_fx,
            &st_fx->exc_pe_fx, st_fx->core_brate_fx,st_fx->bwidth_fx, &st_fx->Q_stat_noise, &st_fx->Q_stat_noise_ge );
        }

        /* SC-VBR */
        st_fx->FadeScale_fx = mult(st_fx->FadeScale_fx,24576);  /*24576 in Q15*/
    }

    IF (sub(st_fx->L_frame_fx,L_FRAME) == 0)
    {
        Copy( Aq_fx+2*(M+1), st_fx->cur_sub_Aq_fx, (M+1) );
    }
    ELSE
    {
        Copy( Aq_fx+3*(M+1), st_fx->cur_sub_Aq_fx, (M+1) );
    }

    /*--------------------------------------------------------*
     * Apply NB postfilter in case of 8kHz output
     *--------------------------------------------------------*/
    if( sub(st_fx->last_bwidth_fx,NB) == 0 )
    {
        k = 0;
        move16();
        FOR (i = 0; i < st_fx->L_frame_fx; i += L_SUBFR)
        {
            pitch_buf_tmp[k] = mult_r(pitch_buf_fx[k], 512);
            move16();
            k++;
        }

        IF(sub(st_fx->bwidth_fx,NB) == 0)
        {
            st_fx->pfstat.on = 1;
            move16();

            nb_post_filt( st_fx->L_frame_fx, &(st_fx->pfstat), &st_fx->psf_lp_noise_fx, tmp_noise_fx, syn_fx, Aq_fx, pitch_buf_tmp, coder_type_fx, 0 );
        }
        ELSE
        {
            st_fx->pfstat.on = 0;
            move16();

            nb_post_filt( st_fx->L_frame_fx, &(st_fx->pfstat), &st_fx->psf_lp_noise_fx, tmp_noise_fx, syn_fx, Aq_fx, pitch_buf_tmp, AUDIO, 0 );
        }
    }

    /*------------------------------------------------------------------*
     * Perform fixed deemphasis through 1/(1 - g*z^-1)
     *-----------------------------------------------------------------*/

    /* Update old synthesis buffer - needed for ACELP internal sampling rate switching */
    Copy( syn_fx + st_fx->L_frame_fx - L_SYN_MEM, st_fx->mem_syn_r, L_SYN_MEM );
    deemph_fx( syn_fx, st_fx->preemph_fac, st_fx->L_frame_fx, &(st_fx->mem_deemph_fx) );

    /* might be able to use syn_fx_tmp[0]... tbv */
    unscale_AGC( syn_fx, st_fx->Q_syn, syn_fx_tmp2, st_fx->agc_mem_fx, st_fx->L_frame_fx );
    Copy(syn_fx_tmp2, syn_fx, st_fx->L_frame_fx);

    /* Update CODEC-B memories*/
    Copy_Scale_sig( syn_fx + st_fx->L_frame_fx/2, st_fx->old_syn_Overl, st_fx->L_frame_fx/2, sub(-1,st_fx->Q_syn)); /*Q-1*/
    Copy_Scale_sig( syn_fx + st_fx->L_frame_fx-M-1, st_fx->syn, M+1, sub(0,st_fx->Q_syn)); /*Q0*/

    /*------------------------------------------------------------------*
     * Formant post-filter
     *-----------------------------------------------------------------*/

    test();
    test();
    IF( sub(st_fx->last_bwidth_fx,WB) >= 0 && L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && L_sub(st_fx->core_brate_fx,ACELP_32k) <= 0)
    {
        Copy( syn_fx, temp_buf + L_SYN_MEM, L_FRAME16k );
        st_fx->pfstat.on = 1;
        move16();
        formant_post_filt( &(st_fx->pfstat), temp_buf + L_SYN_MEM, Aq_fx, syn_fx, L_FRAME16k, st_fx->lp_noise, st_fx->total_brate_fx, 0);
    }
    ELSE IF( sub(st_fx->last_bwidth_fx,WB) >= 0 )
    {
        if( st_fx->pfstat.on )
        {
            Copy( st_fx->pfstat.mem_pf_in+L_SYN_MEM-M, syn_fx-M, M );
            Residu3_fx ( Aq_fx, syn_fx, temp_buf, L_SUBFR, 1 );
            E_UTIL_synthesis ( 1, Aq_fx, temp_buf, syn_fx, L_SUBFR, st_fx->pfstat.mem_stp+L_SYN_MEM-M, 0, M );
            Copy( syn_fx, temp_buf, L_SUBFR );
            scale_st ( temp_buf, syn_fx, &st_fx->pfstat.gain_prec, L_SUBFR );
        }
        st_fx->pfstat.on = 0;
        move16();
    }

    /*----------------------------------------------------------------*
     * Comfort noise addition
     *----------------------------------------------------------------*/

    test();
    test();
    IF( st_fx->flag_cna || (sub(st_fx->cng_type_fx,FD_CNG) == 0 && L_sub(st_fx->total_brate_fx,ACELP_32k) <= 0) )
    {
        /*VAD only for non inactive frame*/
        test();
        st_fx->VAD = st_fx->VAD && (coder_type_fx != INACTIVE);

        /*Noisy speech detector*/
        noisy_speech_detection( st_fx->VAD, syn_fx, st_fx->L_frame_fx, st_fx->Q_syn, st_fx->hFdCngDec_fx->msNoiseEst, st_fx->hFdCngDec_fx->msNoiseEst_exp,
                                st_fx->hFdCngDec_fx->psize_shaping_norm, st_fx->hFdCngDec_fx->psize_shaping_norm_exp, st_fx->hFdCngDec_fx->nFFTpart_shaping,
                                &(st_fx->hFdCngDec_fx->lp_noise), &(st_fx->hFdCngDec_fx->lp_speech), &(st_fx->hFdCngDec_fx->hFdCngCom->flag_noisy_speech) );

        st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech = mult_r(st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.99));
        IF ( st_fx->hFdCngDec_fx->hFdCngCom->flag_noisy_speech != 0 )
        {
            st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech = add(st_fx->hFdCngDec_fx->hFdCngCom->likelihood_noisy_speech, FL2WORD16(0.01));
            move16();
        }
        st_fx->lp_noise = st_fx->hFdCngDec_fx->lp_noise;
        move32();

        /*Noise estimate*/
        test();
        test();
        ApplyFdCng( syn_fx, st_fx->Q_syn, realBuffer, imagBuffer, 0, st_fx->hFdCngDec_fx, st_fx->m_frame_type, st_fx, 0
                    , (sub(coder_type_fx, AUDIO )== 0 && st_fx->GSC_noisy_speech_fx == 0)
                  );

        /* CNA: Generate additional comfort noise to mask potential coding artefacts */
        test();
        test();
        test();
        test();
        IF( st_fx->flag_cna && sub(coder_type_fx,AUDIO) != 0 )
        {
            generate_masking_noise( syn_fx, st_fx->Q_syn, st_fx->hFdCngDec_fx->hFdCngCom, st_fx->hFdCngDec_fx->hFdCngCom->frameSize, 0 );
        }
        ELSE IF( st_fx->flag_cna && coder_type_fx == AUDIO && st_fx->last_core_fx == ACELP_CORE && st_fx->last_coder_type_fx != AUDIO )
        {
            FOR (i=0; i < st_fx->hFdCngDec_fx->hFdCngCom->frameSize/2; i++)
            {
                syn_fx[i] = add( syn_fx[i], shr_r( mult_r( st_fx->hFdCngDec_fx->hFdCngCom->olapBufferSynth2[i+5*st_fx->hFdCngDec_fx->hFdCngCom->frameSize/4], st_fx->hFdCngDec_fx->hFdCngCom->fftlenFac ), -st_fx->Q_syn ) );
                move16();
            }
        }

        st_fx->hFdCngDec_fx->hFdCngCom->frame_type_previous = st_fx->m_frame_type;
        move16();
    }

    test();
    IF( st_fx->flag_cna == 0 || sub(coder_type_fx,AUDIO) == 0 )
    {
        set16_fx( st_fx->hFdCngDec_fx->hFdCngCom->olapBufferSynth2, 0, st_fx->hFdCngDec_fx->hFdCngCom->fftlen );
    }

    /*----------------------------------------------------------------*
    * Resample to the output sampling rate (8/16/32/48 kHz)
    * Bass post-filter
    *----------------------------------------------------------------*/

    /* check if the CLDFB works on the right sample rate */
    IF( (st_fx->cldfbAna_fx->usb * st_fx->cldfbAna_fx->no_col) != st_fx->L_frame_fx )
    {
        /* resample to ACELP internal sampling rate */
        Word16 newCldfbBands = CLDFB_getNumChannels((int)(st_fx->L_frame_fx * 50));
        resampleCldfb( st_fx->cldfbAna_fx, newCldfbBands, st_fx->L_frame_fx, 0 );
        resampleCldfb( st_fx->cldfbBPF_fx, newCldfbBands, st_fx->L_frame_fx, 0 );

        if( st_fx->ini_frame_fx > 0 )
        {
            st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels,st_fx->cldfbAna_fx->no_channels);
        }
    }

    IF( sub(st_fx->L_frame_fx,st_fx->last_L_frame_fx) != 0 )
    {
        IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            retro_interp5_4_fx( st_fx->pst_old_syn_fx );
        }
        ELSE IF( sub(st_fx->L_frame_fx,L_FRAME16k) == 0 )
        {
            retro_interp4_5_fx( syn_fx, st_fx->pst_old_syn_fx );
        }
    }

    /* bass post-filter (when "bpf_error_signal" is defined, no additional delay is introduced) */
    bass_psfilter_fx( st_fx->Opt_AMR_WB_fx, syn_fx, st_fx->L_frame_fx, pitch_buf_fx, st_fx->pst_old_syn_fx,
                      &st_fx->pst_mem_deemp_err_fx, &st_fx->pst_lp_ener_fx, st_fx->bpf_off_fx, st_fx->stab_fac_fx, &st_fx->stab_fac_smooth_fx,
                      st_fx->mem_mean_pit_fx, st_fx->Track_on_hist_fx, st_fx->vibrato_hist_fx, &st_fx->psf_att_fx, coder_type_fx, st_fx->Q_syn, bpf_error_signal );

    /* analysis of the synthesis at internal sampling rate */
    cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &scaleFactor, syn_fx, negate(st_fx->Q_syn), CLDFB_NO_COL_MAX, workBuffer);

    scaleFactor.hb_scale = scaleFactor.lb_scale;
    move16();

    /* analysis and add the BPF error signal */
    move16();
    move16(); /* bpf_length */
    addBassPostFilterFx( bpf_error_signal, realBuffer, imagBuffer, st_fx->cldfbBPF_fx, workBuffer, negate(st_fx->Q_syn),
                         (st_fx->bpf_off_fx == 0) ? CLDFB_NO_COL_MAX : 0, st_fx->cldfbAna_fx->no_col, st_fx->cldfbAna_fx->no_channels, &scaleFactor );

    /* set output mask for upsampling */
    IF( sub(st_fx->bwidth_fx,NB) == 0 )
    {
        /* set NB mask for upsampling */
        st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels,10);
    }
    ELSE if( sub(st_fx->cldfbSyn_fx->bandsToZero,sub(st_fx->cldfbSyn_fx->no_channels,st_fx->cldfbAna_fx->no_channels)) != 0 )
    {
        /* in case of BW switching, re-init to default */
        st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels, st_fx->cldfbAna_fx->no_channels);
    }

    /*WB/SWB-FD_CNG*/
    scaleFactor.hb_scale = scaleFactor.lb_scale;
    move16();

    test();
    test();
    test();
    IF( ( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st_fx->core_brate_fx,SID_2k40) == 0 ) && ( sub(st_fx->cng_type_fx,FD_CNG) == 0 ) && ( sub(st_fx->hFdCngDec_fx->hFdCngCom->numCoreBands,st_fx->cldfbSyn_fx->no_channels) < 0 ) )
    {
        generate_comfort_noise_dec_hf( realBuffer, imagBuffer, &scaleFactor.hb_scale, st_fx );

        st_fx->cldfbSyn_fx->bandsToZero = 0;
        move16();
        if( sub(st_fx->hFdCngDec_fx->hFdCngCom->regularStopBand, st_fx->cldfbSyn_fx->no_channels) < 0 )
        {
            st_fx->cldfbSyn_fx->bandsToZero = sub(st_fx->cldfbSyn_fx->no_channels, st_fx->hFdCngDec_fx->hFdCngCom->regularStopBand);
        }
        st_fx->cldfbSyn_fx->lsb = st_fx->cldfbAna_fx->no_channels;
        move16();
    }

    /* synthesis of the combined signal */
    st_fx->Q_syn2 = st_fx->Q_syn;
    move16();
    cldfbSynthesisFiltering(st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &scaleFactor, synth_out, negate(st_fx->Q_syn2), CLDFB_NO_COL_MAX, workBuffer);

    /* Bring CLDFB output to Q0 */
    Scale_sig(synth_out, output_frame, negate(st_fx->Q_syn2));
    st_fx->Q_syn2 = 0;
    move16();

    /* save synthesis - needed in case of core switching */
    Copy( synth_out, st_fx->previoussynth_fx, output_frame );

    /*-----------------------------------------------------------------*
     * Bandwidth extension 6kHz-7kHz (only for 16kHz input signals)
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( (sub(st_fx->L_frame_fx,L_FRAME) == 0 && sub(st_fx->bwidth_fx,NB) != 0 && sub(output_frame,L_FRAME16k) >= 0 &&
         ( sub(st_fx->extl_fx,-1) == 0 || sub(st_fx->extl_fx,SWB_CNG) == 0 || (sub(st_fx->extl_fx,WB_BWE) == 0 && st_fx->extl_brate_fx == 0 && sub(coder_type_fx,AUDIO) != 0)) ) )
    {
        hf_synth_fx( st_fx->core_brate_fx, output_frame, Aq_fx, exc2_fx, syn_fx, synth_out, &st_fx->seed2_fx,
                     st_fx->mem_hp400_fx, st_fx->mem_syn_hf_fx, st_fx->mem_hf_fx, st_fx->Q_exc,
                     st_fx->Q_syn2, st_fx->delay_syn_hf_fx, &st_fx->memExp1, st_fx->mem_hp_interp_fx, st_fx->extl_fx, st_fx->CNG_mode_fx );
    }
    ELSE
    {
        hf_synth_reset_fx( &st_fx->seed2_fx, st_fx->mem_hf_fx, st_fx->mem_syn_hf_fx, st_fx->mem_hp400_fx, st_fx->mem_hp_interp_fx, st_fx->delay_syn_hf_fx );
    }

    /*-----------------------------------------------------------------*
     * Populate parameters for SWB TBE
     *-----------------------------------------------------------------*/

    /* Apply a non linearity to the SHB excitation */
    test();
    test();
    test();
    test();
    IF( ( !st_fx->bfi_fx && ( st_fx->prev_bfi_fx )) || ((sub(st_fx->last_vbr_hw_BWE_disable_dec_fx,1) == 0) && (st_fx->vbr_hw_BWE_disable_dec_fx == 0)) )
    {
        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
        set16_fx( st_fx->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );
    }


    IF( !st_fx->ppp_mode_dec_fx )
    {
        non_linearity_fx( bwe_exc_fx, bwe_exc_extended, L_FRAME32k, &st_fx->bwe_non_lin_prev_scale_fx, st_fx->Q_exc ,
                          coder_type_fx, voice_factors, st_fx->L_frame_fx );
    }

    test();
    if( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
    {
        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
    }

    /*----------------------------------------------------------------------*
     * Updates
     *----------------------------------------------------------------------*/

    updt_dec_fx( st_fx, st_fx->L_frame_fx, coder_type_fx, old_exc_fx, pitch_buf_fx, Es_pred_fx, Aq_fx, lsf_new_fx, lsp_new_fx, voice_factors, old_bwe_exc_fx, gain_buf );

    test();
    IF( st_fx->first_CNG_fx != 0 && L_sub(st_fx->core_brate_fx,SID_2k40) > 0 )
    {
        /* update CNG parameters in active frames */
        cng_params_upd_fx( lsp_new_fx, exc_fx, st_fx->L_frame_fx, &st_fx->ho_circ_ptr_fx, st_fx->ho_ener_circ_fx, &st_fx->ho_circ_size_fx, st_fx->ho_lsp_circ_fx,
                           st_fx->Q_exc, DEC, st_fx->ho_env_circ_fx, NULL, NULL, NULL, NULL, st_fx->last_active_brate_fx );

        /* Set 16k LSP flag for CNG buffer */
        st_fx->ho_16k_lsp_fx[st_fx->ho_circ_ptr_fx] = 0;
        move16();
        if( sub(st_fx->L_frame_fx, L_FRAME) != 0 )
        {
            st_fx->ho_16k_lsp_fx[st_fx->ho_circ_ptr_fx] = 1;
            move16();
        }
    }

    return;
}
