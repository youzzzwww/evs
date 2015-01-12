/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"            /* Compilation switches                   */
#include "cnst_fx.h"            /* Common constants                       */
#include "rom_com_fx.h"         /* Static table prototypes                */
#include "prot_fx.h"            /* Function prototypes                    */
#include "stl.h"                /* required for wmc_tool */
#include "basop_mpy.h"          /*required for CodeB_Mpy_32_16()*/
#include "basop_util.h"         /* Function prototypes                    */


/*---------------------------------------------------------------------*
 * bandwidth_switching_detect_fx()
 *
 *
 *---------------------------------------------------------------------*/

void bandwidth_switching_detect_fx(
    Decoder_State_fx *st_fx                /* i/o: encoder state structure */
)
{
    /* update band-width switching counter */
    test();
    test();
    test();
    IF( sub(st_fx->bws_cnt1_fx, N_NS2W_FRAMES) >= 0 )
    {
        st_fx->bws_cnt1_fx = 0;
        move16();
    }
    ELSE IF( L_sub(st_fx->total_brate_fx, ACELP_9k60) > 0 && L_sub(st_fx->last_core_brate_fx, ACELP_9k60) < 0
             && sub(st_fx->bwidth_fx, SWB) == 0 && sub(st_fx->last_bwidth_fx, WB) == 0 )
    {
        st_fx->bws_cnt1_fx = add(st_fx->bws_cnt1_fx,1);
        move16();
    }
    ELSE IF( st_fx->bws_cnt1_fx > 0 )
    {
        IF( sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) < 0 )
        {
            st_fx->bws_cnt_fx = sub( shl(sub(N_NS2W_FRAMES, st_fx->bws_cnt1_fx), 1), 1 );
            move16();
        }
        ELSE
        {
            st_fx->bws_cnt_fx = 0;
            move16();
        }

        IF( sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) < 0 )
        {
            st_fx->bws_cnt1_fx = 0;
            move16();
        }
        ELSE
        {
            IF(sub(st_fx->bwidth_fx, SWB) == 0)
            {
                st_fx->bws_cnt1_fx = add(st_fx->bws_cnt1_fx,1);
                move16();
            }
            ELSE
            {
                st_fx->bws_cnt1_fx = 0;
                move16();
            }
        }
    }

    /* update band-width switching counter */
    test();
    test();
    test();
    IF( sub(st_fx->bws_cnt_fx, N_WS2N_FRAMES) >= 0 )
    {
        st_fx->bws_cnt_fx = 0;
        move16();
    }
    ELSE IF( L_sub(st_fx->total_brate_fx, ACELP_9k60) < 0 && L_sub(st_fx->last_core_brate_fx, ACELP_9k60) > 0
             && sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) < 0 && sub(st_fx->bwidth_fx, WB) == 0 )
    {
        st_fx->bws_cnt_fx = add(st_fx->bws_cnt_fx,1);
        move16();
    }
    ELSE IF( st_fx->bws_cnt_fx > 0 )
    {
        IF( sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) > 0 )
        {
            st_fx->bws_cnt1_fx = shr(sub(N_WS2N_FRAMES, st_fx->bws_cnt_fx), 1);
            move16();
        }
        ELSE
        {
            st_fx->bws_cnt1_fx = 0;
            move16();
        }

        IF( sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) > 0 )
        {
            st_fx->bws_cnt_fx = 0;
            move16();
        }
        ELSE
        {
            IF( sub(st_fx->bwidth_fx, WB) == 0 )
            {
                st_fx->bws_cnt_fx = add(st_fx->bws_cnt_fx,1);
                move16();
            }
            ELSE
            {
                st_fx->bws_cnt_fx = 0;
                move16();
            }
        }
    }

    return;
}

/*---------------------------------------------------------------------*
 * Calc_freq_ener()
 *
 *
 *---------------------------------------------------------------------*/

static Word32 Calc_freq_ener(Word32 L_tmp, const Word16 Q_syn2)
{
    Word32 enerLL_fx;
    Word16 exp, tmp;
    IF(L_tmp == 0)
    {
        enerLL_fx = L_deposit_l(0);
    }
    ELSE
    {
        exp = norm_l(L_tmp);
        tmp = extract_h(L_shl(L_tmp, exp));
        exp = sub(exp, sub(30,shl(Q_syn2,1)));

        tmp = div_s(16384, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp);  /*31-exp*/

        enerLL_fx = L_shr(L_tmp, sub(sub(31, exp), Q_syn2)); /*st_fx->Q_syn2-1*/
    }
    return enerLL_fx;
}

/*---------------------------------------------------------------------*
 * bw_switching_pre_proc_fx()
 *
 *
 *---------------------------------------------------------------------*/

void bw_switching_pre_proc_fx(
    const Word16 *old_syn_12k8_16k_fx,    /* i  : ACELP core synthesis at 12.8kHz or 16kHz */
    Decoder_State_fx *st_fx                      /* i/o: decoder state structure     */
)
{
    Word16 i;
    Word16 syn_dct_fx[L_FRAME];
    Word32 L_tmp;
    IF( sub(st_fx->core_fx, ACELP_CORE) == 0 )
    {
        /*----------------------------------------------------------------------*
         * Calculate tilt of the ACELP core synthesis
         *----------------------------------------------------------------------*/

        st_fx->tilt_wb_fx = round_fx(L_shl(calc_tilt_bwe_fx(old_syn_12k8_16k_fx, -1, st_fx->L_frame_fx), 3));

        /*-------------------------------------------------------------------------------*
         * Calculate frequency energy of 0~3.2kHz and 3.2~6.4kHz the ACELP core synthesis
         *-------------------------------------------------------------------------------*/

        edct_16fx(old_syn_12k8_16k_fx, syn_dct_fx, L_FRAME, 6);

        L_tmp = L_deposit_l(0);
        FOR ( i=0; i < L_FRAME/2; i++ )
        {
            L_tmp = L_mac0(L_tmp, syn_dct_fx[i], syn_dct_fx[i]);
        }
        L_tmp = L_shr(L_tmp, 7); /*2*(st_fx->Q_syn2-1)*/
        st_fx->enerLL_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));
        L_tmp = L_deposit_l(0);
        FOR (; i<L_FRAME; i++ )
        {
            L_tmp = L_mac0(L_tmp, syn_dct_fx[i], syn_dct_fx[i]);
        }
        L_tmp = L_shr(L_tmp, 7);    /*2*(st_fx->Q_syn2-1)*/
        st_fx->enerLH_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));
    }
    ELSE
    {
        IF( st_fx->old_is_transient_fx[0] )
        {
            L_tmp = L_deposit_l(0);
            FOR ( i=0; i<32; i++ )
            {
                L_tmp = L_mac0(L_tmp, st_fx->t_audio_q_fx[i], st_fx->t_audio_q_fx[i]);
            }
            L_tmp = L_shr(L_tmp, 5);      /*st_fx->Q_syn2-1*/
            st_fx->enerLL_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));
            L_tmp = L_deposit_l(0);
            FOR (; i<64; i++ )
            {
                L_tmp = L_mac0(L_tmp, st_fx->t_audio_q_fx[i], st_fx->t_audio_q_fx[i]);
            }
            L_tmp = L_shr(L_tmp, 5);     /*st_fx->Q_syn2-1*/
            st_fx->enerLH_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));
        }
        ELSE
        {
            L_tmp = L_deposit_l(0);
            FOR ( i=0; i < L_FRAME/2; i++ )
            {
                L_tmp = L_mac0(L_tmp, st_fx->t_audio_q_fx[i], st_fx->t_audio_q_fx[i]);
            }
            L_tmp = L_shr(L_tmp, 7);     /*st_fx->Q_syn2-1*/
            st_fx->enerLL_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));
            L_tmp = L_deposit_l(0);
            FOR (; i<L_FRAME; i++ )
            {
                L_tmp = L_mac0(L_tmp, st_fx->t_audio_q_fx[i], st_fx->t_audio_q_fx[i]);
            }
            L_tmp = L_shr(L_tmp, 7);     /*st_fx->Q_syn2-1*/
            st_fx->enerLH_fx = Calc_freq_ener(L_tmp, shl(st_fx->Q_syn2,1));

        }
    }


    test();
    test();
    test();
    test();
    test();
    IF( st_fx->last_bwidth_fx == 0 && sub(st_fx->extl_fx, SWB_CNG) <= 0 )
    {
        st_fx->prev_ener_shb_fx = 0;
        move16();
        set16_fx(st_fx->prev_SWB_fenv_fx, 0, SWB_FENV);
        st_fx->last_hq_tilt_fx = 3277;
        move16();
    }
    ELSE if(((sub(st_fx->core_fx, ACELP_CORE) == 0 && sub(st_fx->last_core_fx, HQ_CORE) == 0) || (sub(st_fx->core_fx, st_fx->last_core_fx) == 0 && sub(st_fx->extl_fx, st_fx->last_extl_fx) != 0)) && sub(st_fx->last_bwidth_fx, SWB) >= 0)
    {
        st_fx->attenu_fx = 3277;
        move16();
    }

    if(sub(st_fx->last_core_fx, ACELP_CORE) == 0)
    {
        st_fx->last_hq_tilt_fx = 3277;
        move16();
    }

    test();
    test();
    test();
    test();
    IF(sub(st_fx->last_core_fx, HQ_CORE) == 0
       || ( sub(st_fx->last_core_fx, ACELP_CORE) == 0
            && !(sub(st_fx->last_extl_fx, WB_TBE) == 0 || sub(st_fx->last_extl_fx, SWB_TBE) == 0 || sub(st_fx->last_extl_fx, FB_TBE) == 0) && L_sub(st_fx->core_brate_fx, ACELP_8k00) > 0))
    {
        st_fx->prev_fractive_fx = 0;
        move16();
    }
    return;
}


/*---------------------------------------------------------------------*
* core_switching_pre_dec_fx()
*
* Preprocessing/preparation for ACELP/HQ core switching
*---------------------------------------------------------------------*/
void core_switching_pre_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure     */
    const Word16 output_frame       /* i  : frame length                */
)
{
    Word16 oldLenClasBuff, newLenClasBuff;

    /* Codec switching */
    IF( sub(st_fx->last_codec_mode,MODE2)==0 )
    {
        st_fx->mem_deemph_fx = st_fx->syn[M];
        move16();
        Scale_sig(&(st_fx->mem_deemph_fx), 1, st_fx->Q_syn); /* Brings mem_deemph to Qsyn */

        Copy_Scale_sig(st_fx->mem_syn2_fx, st_fx->mem_syn1_fx, M, sub(-1,st_fx->Q_syn)); /*Q-1*/

        st_fx->bpf_off_fx = 1;
        move16();
        Scale_sig(st_fx->pfstat.mem_pf_in, L_SUBFR, st_fx->Q_syn);           /* Post_filter mem */
        Scale_sig(st_fx->pfstat.mem_res2, DECMEM_RES2, st_fx->Q_syn);         /* NB post_filter mem */
        Scale_sig(st_fx->pfstat.mem_stp, L_SUBFR, st_fx->Q_syn);             /* Post_filter mem */
        set16_fx( st_fx->pst_old_syn_fx, 0, NBPSF_PIT_MAX );                 /* BPF mem*/

        st_fx->last_con_tcx=st_fx->con_tcx;
        move16();
        st_fx->con_tcx=0;
        move16();

        /* reset upd_cnt */
        st_fx->upd_cnt_fx = MAX_UPD_CNT;
        move16();
        st_fx->igf = 0;
        move16();
        set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX*2 );

        IF( L_sub(st_fx->output_Fs_fx,16000) >= 0 )
        {
            hf_synth_reset_fx( &st_fx->seed2_fx, st_fx->mem_hf_fx, st_fx->mem_syn_hf_fx, st_fx->mem_hp400_fx, st_fx->mem_hp_interp_fx, st_fx->delay_syn_hf_fx );
        }
        set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );

        set32_fx( st_fx->prev_env_fx, 0, SFM_N_WB );
        set32_fx( st_fx->prev_normq_fx, 0, SFM_N_WB );

        set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
        set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
        st_fx->last_max_pos_pulse_fx = 0;
        move16();

        IF( L_sub(st_fx->output_Fs_fx,16000)>0 )
        {
            set32_fx( st_fx->prev_coeff_out_fx, 0, L_FRAME32k );
        }

        /* pre-echo */
        st_fx->pastpre_fx = 0;
        move16();
        /* reset the GSC pre echo energy threshold in case of switching */
        st_fx->Last_frame_ener_fx = MAX_32;
        move32();

        test();
        IF( sub(st_fx->last_core_fx,TCX_20_CORE)==0 || sub(st_fx->last_core_fx,TCX_10_CORE)==0 )
        {
            st_fx->last_core_fx = HQ_CORE;
            move16();
            Copy( st_fx->FBTCXdelayBuf, st_fx->prev_synth_buffer_fx, NS2SA(st_fx->output_Fs_fx, DELAY_BWE_TOTAL_NS - DELAY_CLDFB_NS));
        }

        IF(st_fx->prev_bfi_fx!=0)
        {
            Word16 delay_comp;

            /*switch off Hq Voicing as it was not uodated in MODE2*/
            st_fx->oldHqVoicing_fx=0;
            st_fx->HqVoicing_fx=0;
            move16();

            delay_comp = NS2SA_fx2(st_fx->output_Fs_fx, DELAY_CLDFB_NS);

            IF( !st_fx->last_con_tcx && st_fx->last_core_bfi == ACELP_CORE && sub(st_fx->core_fx,HQ_CORE)==0 )
            {

                Word16 i, no_col;
                Word32 *realBuffer[CLDFB_NO_COL_MAX], *imagBuffer[CLDFB_NO_COL_MAX];
                Word32 realBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX], imagBufferTmp[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
                CLDFB_SCALE_FACTOR scaleFactor;
                Word32 workBuffer[128*3];

                FOR( i=0; i<CLDFB_NO_COL_MAX; i++ )
                {
                    set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
                    set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
                    realBuffer[i] = realBufferTmp[i];
                    move32();
                    imagBuffer[i] = imagBufferTmp[i];
                    move32();
                }

                no_col = s_min(st_fx->cldfbAna_fx->no_col, idiv1616(sub(add(delay_comp, st_fx->cldfbAna_fx->no_channels),1) , st_fx->cldfbAna_fx->no_channels));

                /* QMF analysis of the synthesis at internal sampling rate */
                cldfb_save_memory( st_fx->cldfbAna_fx );
                cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &scaleFactor, st_fx->syn_Overl, 0, no_col, workBuffer);
                cldfb_restore_memory( st_fx->cldfbAna_fx );

                scaleFactor.hb_scale = scaleFactor.lb_scale;
                move16();

                /* QMF synthesis of the combined signal */
                cldfb_save_memory( st_fx->cldfbSyn_fx );
                cldfbSynthesisFiltering(st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &scaleFactor, st_fx->fer_samples_fx, 0, no_col, workBuffer);
                cldfb_restore_memory( st_fx->cldfbSyn_fx );
            }

            IF( !st_fx->last_con_tcx && st_fx->last_core_bfi == ACELP_CORE && sub(st_fx->core_fx,HQ_CORE)==0 )
            {
                lerp(st_fx->syn_Overl, st_fx->fer_samples_fx+delay_comp,shr(st_fx->output_frame_fx,1), shr(st_fx->last_L_frame_fx,1));
                /*Set to zero the remaining part*/
                set16_fx( st_fx->fer_samples_fx+delay_comp+shr(st_fx->output_frame_fx,1), 0, shr(st_fx->output_frame_fx,1)-delay_comp);
            }
        }

        st_fx->use_acelp_preq = 0;
        move16();

    }

    /*FEC*/
    IF( sub(st_fx->L_frame_fx,L_FRAME16k)<=0 )
    {
        test();
        IF( sub(st_fx->last_L_frame_fx,L_FRAME16k)<=0 && sub(st_fx->core_fx, HQ_CORE)!=0 )
        {
            IF( sub(st_fx->L_frame_fx,st_fx->last_L_frame_fx)!=0 )
            {
                IF (sub(st_fx->L_frame_fx,st_fx->last_L_frame_fx)>0)
                {
                    oldLenClasBuff = extract_l(L_shr(Mpy_32_16_1(L_mult0(st_fx->last_L_frame_fx,getInvFrameLen(st_fx->L_frame_fx)/*Q21*/)/*Q21*/,L_SYN_MEM_CLAS_ESTIM/*Q0*/)/*Q6*/,6)/*Q0*/);
                    newLenClasBuff = L_SYN_MEM_CLAS_ESTIM;
                    move16();
                }
                ELSE
                {
                    oldLenClasBuff = L_SYN_MEM_CLAS_ESTIM;
                    move16();
                    newLenClasBuff = extract_l(L_shr(Mpy_32_16_1(L_mult0(st_fx->L_frame_fx,getInvFrameLen(st_fx->last_L_frame_fx)/*Q21*/)/*Q21*/,L_SYN_MEM_CLAS_ESTIM/*Q0*/)/*Q6*/,6)/*Q0*/);
                }
                lerp( st_fx->mem_syn_clas_estim_fx, st_fx->mem_syn_clas_estim_fx, newLenClasBuff, oldLenClasBuff );
            }
        }
        ELSE
        {
            set16_fx( st_fx->mem_syn_clas_estim_fx, 0,L_SYN_MEM_CLAS_ESTIM );
        }
    }

    test();
    test();
    IF( (sub(st_fx->core_fx,ACELP_CORE)==0 || sub(st_fx->core_fx, AMR_WB_CORE)==0 ) && sub( st_fx->last_core_fx,HQ_CORE)==0 )
    {
        IF(sub(st_fx->L_frame_fx, L_FRAME16k)==0 )
        {
            Copy( TRWB2_Ave_fx, st_fx->lsf_old_fx, M );
            lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, INT_FS_16k );
        }
        ELSE
        {
            Copy( TRWB_Ave_fx, st_fx->lsf_old_fx, M ); /* init of LSP */
            lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, INT_FS_FX );
        }

        st_fx->mem_deemph_fx = 0;
        move16();
        set16_fx( st_fx->mem_syn2_fx, 0, M );
        set16_fx( st_fx->mem_syn1_fx, 0, M );
        st_fx->bwe_non_lin_prev_scale_fx = 0;
        move16();

        /* Reset ACELP parameters */
        set16_fx( st_fx->mem_MA_fx,0, M );
        IF( L_sub(st_fx->sr_core,16000) == 0 )
        {
            Copy( GEWB2_Ave_fx, st_fx->mem_AR_fx, M );
        }
        ELSE
        {
            Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
        }
        st_fx->tilt_code_fx = 0;
        move16();
        st_fx->gc_threshold_fx = 0;
        move16();
        st_fx->dm_fx.prev_gain_code = L_deposit_l(0);
        set16_fx(st_fx->dm_fx.prev_gain_pit, 0, 6);
        st_fx->dm_fx.prev_state = 0;
        move16();

        st_fx->last_coder_type_fx = GENERIC;
        move16();

        frame_ener_fx( output_frame, UNVOICED_CLAS, st_fx->previoussynth_fx, -1, &st_fx->enr_old_fx, 1, 0, 0, 0 );
        st_fx->lp_gainp_fx = 0;
        move16();

        /* the sqrt below needs to be changed to use basop Sqrt16 */
        /*st_fx->lp_gainc_fx = (float)sqrt( st_fx->lp_ener_fx ); */
        IF( st_fx->lp_ener_fx != 0 )
        {
            Word32  L_tmp;
            Word16 tmp, exp;
            exp = norm_l(st_fx->lp_ener_fx); /* In Q6 */
            tmp = extract_h(L_shl(st_fx->lp_ener_fx, exp));
            exp = sub(exp, 30-6);

            tmp = div_s(16384, tmp);
            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp);

            st_fx->lp_gainc_fx = round_fx(L_shl(L_tmp, sub(exp, 12))); /* In Q3 */
        }

        st_fx->last_ppp_mode_dec_fx = 0;
        move16();
        st_fx->last_nelp_mode_dec_fx = 0;
        move16();

        st_fx->last_voice_factor_fx = 0;
        move16();
        st_fx->Last_GSC_noisy_speech_flag_fx = 0;
        move16();

        Copy32( st_fx->old_pitch_buf_fx + st_fx->nb_subfr, st_fx->old_pitch_buf_fx, st_fx->nb_subfr);
        set32_fx( st_fx->old_pitch_buf_fx + st_fx->nb_subfr, (L_SUBFR<<16), st_fx->nb_subfr );

        /* reset CLDFB memories */
        cldfb_reset_memory( st_fx->cldfbAna_fx );
        cldfb_reset_memory( st_fx->cldfbBPF_fx );
        cldfb_reset_memory( st_fx->cldfbSyn_fx );

        /* reset TBE memories */
        test();
        IF (!st_fx->last_con_tcx)
        {
            set16_fx(st_fx->old_exc_fx,0, L_EXC_MEM_DEC );
        }
        ELSE IF (sub(st_fx->L_frame_fx,L_FRAME16k) < 0)
        {
            /* resample from 16kHz to 12.8kHZ */
            synth_mem_updt2( st_fx->L_frame_fx, L_FRAME16k, st_fx->old_exc_fx, st_fx->mem_syn_r, st_fx->mem_syn2_fx, NULL, DEC );
        }

        set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX*2 );

        IF( L_sub(st_fx->output_Fs_fx, 16000L)>=0 )
        {
            hf_synth_reset_fx( &st_fx->seed2_fx, st_fx->mem_hf_fx, st_fx->mem_syn_hf_fx, st_fx->mem_hp400_fx, st_fx->mem_hp_interp_fx, st_fx->delay_syn_hf_fx );
        }
        set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );
    }

    test();
    test();
    IF( sub(st_fx->core_fx,HQ_CORE)==0 && (sub(st_fx->last_core_fx,ACELP_CORE)==0 || st_fx->last_core_fx == AMR_WB_CORE) )
    {
        set32_fx( st_fx->prev_env_fx, 0, SFM_N_WB );
        set32_fx( st_fx->prev_normq_fx, 0, SFM_N_WB );

        set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
        set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
        st_fx->last_max_pos_pulse_fx = 0;

        set16_fx( st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM );
        st_fx->prev_frm_hfe2_fx = 0;
        st_fx->prev_stab_hfe2_fx = 0;

        IF( L_sub(st_fx->output_Fs_fx,16000) > 0 )
        {
            set32_fx( st_fx->prev_coeff_out_fx, 0, L_FRAME32k );
        }

        set16_fx( st_fx->old_out_fx, 0, output_frame );
    }

    /* handle switching cases where preecho_sb was not called in the last frame (memory not up to date) */
    st_fx->pastpre_fx--;
    IF( st_fx->pastpre_fx <= 0 )
    {
        reset_preecho_dec_fx( st_fx );
    }

    IF( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        st_fx->VAD = 0;
        move16();
        st_fx->m_frame_type = ZERO_FRAME;
        move16();
    }
    ELSE IF( L_sub(st_fx->core_brate_fx,SID_2k40) <= 0 )
    {
        st_fx->VAD = 0;
        move16();
        st_fx->m_frame_type = SID_FRAME;
        move16();
    }
    ELSE
    {
        st_fx->VAD = 1;
        move16();
        st_fx->m_frame_type = ACTIVE_FRAME;
        move16();
    }

    /*switch on CNA on active frames*/
    test();
    test();
    test();
    test();
    IF( sub(st_fx->core_fx,AMR_WB_CORE) != 0 && st_fx->VAD && L_sub(st_fx->total_brate_fx,CNA_MAX_BRATE) <= 0 )
    {
        st_fx->flag_cna = 1;
        move16();
    }
    ELSE IF( sub(st_fx->core_fx,AMR_WB_CORE) == 0 && st_fx->VAD && L_sub(st_fx->total_brate_fx,ACELP_8k85) <= 0 )
    {
        st_fx->flag_cna = 1;
        move16();
    }
    ELSE IF( st_fx->VAD )
    {
        st_fx->flag_cna = 0;
        move16();
    }

    if( sub(st_fx->core_fx,AMR_WB_CORE) == 0 )
    {
        st_fx->cng_type_fx = LP_CNG;
        move16();
    }

    test();
    test();
    test();
    test();
    IF( st_fx->hFdCngDec_fx && ((sub(st_fx->last_L_frame_fx,st_fx->L_frame_fx) != 0) ||
                                (sub(st_fx->hFdCngDec_fx->hFdCngCom->frameSize,st_fx->L_frame_fx)!=0) ||
                                st_fx->ini_frame_fx == 0 || sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) != 0))
    {

        IF( sub(st_fx->core_fx,AMR_WB_CORE) != 0 )
        {
            configureFdCngDec(st_fx->hFdCngDec_fx, st_fx->bwidth_fx, (st_fx->rf_flag == 1 && st_fx->total_brate_fx == 13200) ? 9600:st_fx->total_brate_fx, st_fx->L_frame_fx );
        }
        ELSE
        {
            configureFdCngDec(st_fx->hFdCngDec_fx, 1, ACELP_8k00, st_fx->L_frame_fx );

            if( st_fx->VAD )
            {
                st_fx->hFdCngDec_fx->hFdCngCom->CngBitrate = st_fx->total_brate_fx;
                move32();
            }
        }

        test();
        test();
        IF ( sub(st_fx->last_L_frame_fx,st_fx->L_frame_fx) != 0 && sub(st_fx->L_frame_fx,L_FRAME16k) <= 0 && sub(st_fx->last_L_frame_fx,L_FRAME16k) <= 0 )
        {
            lerp( st_fx->hFdCngDec_fx->hFdCngCom->olapBufferSynth2, st_fx->hFdCngDec_fx->hFdCngCom->olapBufferSynth2, st_fx->L_frame_fx*2, st_fx->last_L_frame_fx*2 );
        }
    }

    return;
}

/*---------------------------------------------------------------------*
 * core_switching_post_dec()
 *
 * Postprocessing for ACELP/HQ core switching
 *---------------------------------------------------------------------*/

void core_switching_post_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure     */
    Word16 *synth,                /* i/o: output synthesis Qsynth     */
    const Word16 output_frame,          /* i  : frame length                */
    const Word16 core_switching_flag,   /* i  : ACELP->HQ switching flag    */
    const Word16 coder_type,            /* i  : ACELP coder type            */
    Word16 *Qsynth                /* i/o: Scaling of ACELP exit (Q_syn2-1) or HQ exit (Qsynth); changes after this function */
)
{
    Word16 i, delay_comp, delta;
    Word16 tmpF, tmp, Fs_kHz, shift, *ptmp1, *ptmp2;
    Word32 L_tmp;
    Word16 synth_subfr_out[SWITCH_MAX_GAP], synth_subfr_bwe[SWITCH_MAX_GAP];
    Word16 mem_synth[NS2SA(16000, DELAY_CLDFB_NS)+2];
    Word16 Qtmp;
    Word16 Qsubfr;

    /* Rescale synthesis in Q0 to avoid multiple rescaling after */
    tmp = Find_Max_Norm16(synth,output_frame);
    Scale_sig(synth,output_frame,tmp);
    *Qsynth=add(*Qsynth,tmp);

    test();
    IF( sub(st_fx->core_fx, ACELP_CORE) == 0 && st_fx->bfi_fx )
    {
        acelp_core_switch_dec_bfi_fx( st_fx, st_fx->fer_samples_fx, coder_type );  /*the output at Q0*/
    }

    /* set multiplication factor according to the sampling rate */
    tmp = extract_l(L_shr(st_fx->output_Fs_fx,13));
    Fs_kHz = shl(add(tmp,1),3);

    delta = 1;
    move16();
    if ( sub(output_frame, L_FRAME16k) >= 0)
    {
        delta = shr(Fs_kHz, 3);
    }

    /* set delay compensation between HQ synthesis and ACELP synthesis */
    delay_comp = i_mult2(delta, HQ_DELAY_COMP);

    IF( sub(st_fx->core_fx, HQ_CORE) == 0 )
    {
        st_fx->use_acelp_preq = 0;
        move16();
        /* rescaling to the min exp of the 2 */
        /* Qtmp=s_min(*Qsynth,st_fx->Q_old_postdec);
         Scale_sig(synth, output_frame, sub(Qtmp,*Qsynth));
         Scale_sig(st_fx->delay_buf_out_fx, delay_comp, sub(Qtmp,st_fx->Q_old_postdec));*/

        st_fx->mem_deemph_old_syn_fx = 0;
        move16();

        test();
        test();
        test();
        IF ( core_switching_flag && sub(st_fx->last_L_frame_fx, st_fx->last_L_frame_ori_fx) == 0 && ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0 ))
        {
            acelp_core_switch_dec_fx( st_fx, synth_subfr_out, synth_subfr_bwe, output_frame, core_switching_flag, mem_synth, &Qsubfr );
        }
        test();
        test();
        IF( core_switching_flag && sub(st_fx->last_core_fx, HQ_CORE) == 0 && st_fx->prev_bfi_fx )
        {
            Copy( st_fx->delay_buf_out_fx, synth_subfr_out, delay_comp );
            Qsubfr=st_fx->Q_old_postdec;
        }

        /* delay HQ synthesis to synchronize with ACELP synthesis */
        /* rescaling to the min exp of the 2 */
        Qtmp=s_min(*Qsynth,st_fx->Q_old_postdec);
        Scale_sig(synth, output_frame, sub(Qtmp,*Qsynth));
        *Qsynth=Qtmp;
        move16();
        Scale_sig(st_fx->delay_buf_out_fx, delay_comp, sub(Qtmp,st_fx->Q_old_postdec));
        st_fx->Q_old_postdec=Qtmp;
        move16();

        Copy( synth, &synth[delay_comp], output_frame);
        Copy( st_fx->delay_buf_out_fx, synth, delay_comp );
        Copy( &synth[output_frame], st_fx->delay_buf_out_fx, delay_comp );

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
        IF ( core_switching_flag && sub(st_fx->last_L_frame_fx, st_fx->last_L_frame_ori_fx) == 0 && ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0 ))
        {
            /* mem_over_hp_fx : Qsubfr */
            core_switching_OLA_fx( mem_synth, st_fx->last_L_frame_fx, st_fx->output_Fs_fx, synth, synth_subfr_out, synth_subfr_bwe, output_frame, Qsynth,&Qsubfr );
        }
        ELSE IF ( core_switching_flag && sub(st_fx->last_core_fx, HQ_CORE) == 0 && st_fx->prev_bfi_fx ) /* HQ | ACELP | TRANSITION  with ACELP frame lost */
        {
            /* Overlapp between old->out (stocked in st_fx->fer_samples)and good HQ frame on L/2 */
            ptmp1 = &synth[delay_comp];
            shift = i_mult2(Fs_kHz, 10);
            tmp = i_mult2(delta,shr(N16_CORE_SW,1));

            Scale_sig(st_fx->fer_samples_fx, output_frame, *Qsynth);
            ptmp2 = &st_fx->fer_samples_fx[tmp];
            tmp = div_s(1, shift); /*Q15*/
            tmpF = 0;
            move16();

            FOR( i=0; i<shift; i++ )
            {
                L_tmp = L_mult((*ptmp1), tmpF); /*Qsynth + 16*/
                *ptmp1 = round_fx( L_mac( L_tmp, add(sub(24576, tmpF),8192), (*ptmp2))); /*Qsynth*/
                ptmp1++;
                ptmp2++;
                tmpF = add(tmpF, tmp);
            }
        }
        ELSE IF ( ( !core_switching_flag && sub(st_fx->core_fx, HQ_CORE) == 0 && (sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx,AMR_WB_CORE) == 0) )  /* ACELP | TRANSITION | HQ with TRANSITION lost */
                  || (core_switching_flag && st_fx->prev_bfi_fx && sub(st_fx->last_L_frame_fx, st_fx->last_L_frame_ori_fx) != 0) )
        {
            /* Overlapp between CELP estimation (BFI) and good HQ frame on L/2 */
            shift = i_mult2(Fs_kHz, 10);
            tmp = div_s(1, shift); /*Q15*/
            tmpF = 0;
            move16();
            ptmp1 = synth;
            Scale_sig(st_fx->fer_samples_fx, output_frame, *Qsynth);
            ptmp2 = st_fx->fer_samples_fx;
            FOR( i=0; i<shift; i++ )
            {
                L_tmp = L_mult((*ptmp1), tmpF); /*Qsynth + 16*/
                *ptmp1 = round_fx( L_mac( L_tmp, add(sub(24576, tmpF),8192), (*ptmp2))); /*Qsynth*/
                tmpF = add(tmpF, tmp);
                ptmp1++;
                ptmp2++;
            }
        }

        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
        set32_fx( st_fx->prev_env_fx, 0, SFM_N_WB );
        set32_fx( st_fx->prev_normq_fx, 0, SFM_N_WB );
        Copy_Scale_sig( synth, st_fx->previoussynth_fx, output_frame, negate(*Qsynth) ); /*scaling of st_fx->previoussynth_fx set at Q0*/

        /*Set post-filtering flag to zero*/
        st_fx->pfstat.on = 0;
        move16();
    }
    ELSE
    {
        IF ( sub(st_fx->last_core_fx, HQ_CORE) == 0 ) /*  MDCT to ACELP transition */
        {
            Qtmp = s_min(s_min(*Qsynth, st_fx->Q_old_postdec), st_fx->Q_old_wtda);

            Scale_sig(synth, output_frame, sub(Qtmp,*Qsynth));
            Scale_sig(st_fx->delay_buf_out_fx, delay_comp, sub(Qtmp,st_fx->Q_old_postdec));
            Scale_sig(st_fx->old_out_fx, L_FRAME48k, sub(Qtmp, st_fx->Q_old_wtda));
            *Qsynth = Qtmp;
            move16();
            st_fx->Q_old_postdec=Qtmp;
            move16();
            st_fx->Q_old_wtda=Qtmp;
            move16();

            Copy( st_fx->delay_buf_out_fx, synth, delay_comp );   /* copy the HQ/ACELP delay synchroniation buffer at the beginning of ACELP frame */

            tmp = i_mult2(delta, N_ZERO_8);
            shift = i_mult2(Fs_kHz, 3);
            test();
            IF( st_fx->prev_bfi_fx && st_fx->HqVoicing_fx )
            {
                Copy_Scale_sig( st_fx->fer_samples_fx, &st_fx->old_out_fx[tmp], shift, *Qsynth );
            }

            ptmp2 = &st_fx->old_out_fx[tmp];
            tmp = div_s(1, shift);
            ptmp1 = &synth[delay_comp];
            tmpF = 0;
            move16();
            FOR( i=0; i<shift; i++ )
            {
                *ptmp1=add( mult_r(tmpF, *ptmp1),mult_r(sub(32767,tmpF),*ptmp2++));
                ptmp1++;
                tmpF = add(tmpF, tmp);
            }
        }

        set16_fx( st_fx->delay_buf_out_fx, 0, HQ_DELTA_MAX*HQ_DELAY_COMP );
        st_fx->oldHqVoicing_fx = 0;
        move16();

        set16_fx(st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM);
        st_fx->prev_frm_hfe2_fx = 0;
        move16();
        st_fx->prev_stab_hfe2_fx = 0;
        move16();
    }

    /* reset SWB BWE buffers */
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
    test();
    IF( ( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->extl_fx, SWB_BWE) == 0 ) || ( sub(st_fx->last_extl_fx, FB_BWE) != 0 && sub(st_fx->extl_fx, FB_BWE) == 0 ) ||
        ((sub(st_fx->last_core_fx, HQ_CORE) == 0 || sub(st_fx->last_extl_fx, SWB_TBE) == 0) && st_fx->extl_fx < 0 && sub(st_fx->core_fx, HQ_CORE) != 0)
        || (sub(st_fx->last_core_fx,ACELP_CORE) == 0 && sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->last_coder_type_fx,INACTIVE) != 0 && sub(coder_type,INACTIVE) == 0 && st_fx->bws_cnt_fx > 0)
      )
    {
        set16_fx( st_fx->L_old_wtda_swb_fx, 0, output_frame );

        if( sub(st_fx->last_extl_fx, WB_BWE) != 0 )
        {
            st_fx->prev_mode_fx = NORMAL;
            move16();
        }

        st_fx->prev_Energy_fx = 0;
        move16();
        st_fx->prev_L_swb_norm_fx = 8;
        move16();
        st_fx->prev_frica_flag_fx = 0;
        move16();
        set16_fx( st_fx->mem_imdct_fx, 0, L_FRAME48k );
        st_fx->prev_td_energy_fx = 0;
        move16();
        st_fx->prev_weight_fx = 6554;
        move16(); /*0.2 in Q15*/
        st_fx->prev_fb_ener_adjust_fx = 0;
        move16();
    }

    /* reset WB BWE buffers */
    test();
    IF( sub(st_fx->last_extl_fx, WB_BWE) != 0 && sub(st_fx->extl_fx, WB_BWE) == 0 )
    {
        set16_fx(st_fx->L_old_wtda_swb_fx, 0, output_frame);

        test();
        if ( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->last_extl_fx, FB_BWE) != 0 )
        {
            st_fx->prev_mode_fx = NORMAL;
            move16();
        }

        st_fx->prev_Energy_fx = 0;
        move16();
        st_fx->prev_L_swb_norm_fx = 8;
        move16();
        set16_fx( st_fx->mem_imdct_fx, 0, L_FRAME48k );
        st_fx->prev_flag_fx = 0;
        move16();
    }

    /* reset SWB TBE buffers */
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( (( sub(st_fx->extl_fx, SWB_TBE) == 0 || sub(st_fx->extl_fx, FB_TBE) == 0 || sub(st_fx->extl_fx, SWB_CNG) == 0) &&
         ( sub(st_fx->L_frame_fx, st_fx->last_L_frame_fx) != 0 || ( sub(st_fx->last_extl_fx, SWB_TBE) != 0  && sub(st_fx->last_extl_fx, FB_TBE) != 0 ) || sub(st_fx->last_core_fx, HQ_CORE) == 0 )) ||
        ( sub(st_fx->bwidth_fx, st_fx->last_bwidth_fx) < 0 && sub(st_fx->last_extl_fx, SWB_TBE) != 0 ) || st_fx->old_ppp_mode_fx
        || ((sub(st_fx->prev_coder_type_fx, AUDIO) == 0 || sub(st_fx->prev_coder_type_fx, INACTIVE) == 0) && st_fx->bws_cnt_fx > 0) )
    {
        swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                          st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx), &(st_fx->tbe_premph_fx)
                          ,st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx) );

        swb_tbe_reset_synth_fx( st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx );

        IF( sub(output_frame, L_FRAME16k) == 0 )
        {
            /* reset in case that SWB TBE layer is transmitted, but the output is 16kHz sampled */
            set16_fx( st_fx->mem_resamp_HB_32k_fx, 0, 2*L_FILT32k );
        }
        set16_fx(st_fx->int_3_over_2_tbemem_dec_fx, 0, INTERP_3_2_MEM_LEN);
    }

    /* Interp_3_2 CNG buffers reset */
    test();
    test();
    test();
    IF(L_sub(st_fx->output_Fs_fx,48000)==0 && ( (L_sub(st_fx->last_core_brate_fx,SID_2k40)>0 ) && (L_sub(st_fx->core_brate_fx,FRAME_NO_DATA)==0 || L_sub(st_fx->core_brate_fx,SID_2k40)==0)) )
    {
        set16_fx( st_fx->interpol_3_2_cng_dec_fx, 0, INTERP_3_2_MEM_LEN );
    }

    /* reset FB TBE buffers */
    test();
    test();
    IF( sub(st_fx->extl_fx, FB_TBE) == 0 && ( sub(st_fx->last_extl_fx, FB_TBE) != 0 || sub(st_fx->L_frame_fx, st_fx->last_L_frame_fx) != 0 ) )
    {
        fb_tbe_reset_synth_fx( st_fx->fbbwe_hpf_mem_fx, &st_fx->prev_fbbwe_ratio_fx );
    }

    /* reset WB TBE buffers */
    test();
    IF( sub(st_fx->last_extl_fx, WB_TBE) != 0 && sub(st_fx->extl_fx, WB_TBE) == 0 )
    {
        wb_tbe_extras_reset_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx );
        wb_tbe_extras_reset_synth_fx( st_fx->state_lsyn_filt_shb_fx, st_fx->state_lsyn_filt_dwn_shb_fx, st_fx->state_32and48k_WB_upsample_fx
                                      ,st_fx->mem_resamp_HB_fx
                                    );

        set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD / 4 );
        set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );
        set32_fx( st_fx->mem_csfilt_fx, 0, 2 );
    }

    return;
}

/*---------------------------------------------------------------------*
* core_switching_hq_prepare_dec()
*
* Preprocessing in the first HQ frame after ACELP frame
* - modify bit allocation for HQ core by removing CELP subframe budget
*---------------------------------------------------------------------*/

void core_switching_hq_prepare_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: encoder state structure */
    Word16 *num_bits,               /* i/o: bit budget update       */
    const Word16 output_frame       /* i  : output frame length     */
)
{
    Word32 cbrate;

    test();
    IF( sub(st_fx->last_core_fx, HQ_CORE) == 0 && st_fx->prev_bfi_fx )
    {
        Copy_Scale_sig( st_fx->old_out_fx, st_fx->fer_samples_fx, output_frame, negate(st_fx->Q_old_wtda) ); /*Q0*/
    }

    /* set switching frame bit-rate */
    IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
    {
        cbrate = L_add(st_fx->core_brate_fx, 0);
        if( L_sub(st_fx->core_brate_fx, ACELP_24k40) > 0 )
        {
            cbrate = L_add(ACELP_24k40, 0);
        }

        /* subtract ACELP switching frame bits */
        if( L_sub(st_fx->core_brate_fx, ACELP_11k60) >= 0 )
        {
            (*num_bits) = sub((*num_bits), 1); /* LP_FLAG bit */
        }
        *num_bits = sub( (*num_bits), ACB_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, GENERIC, 0, 0)] );     /* pitch bits*/
        *num_bits = sub( (*num_bits), gain_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, TRANSITION, 0, 0)] ); /* gain bits */
        *num_bits = sub( (*num_bits), FCB_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, GENERIC, 0, 0)] );     /* FCB bits  */
    }
    ELSE  /* L_frame_fx == L_FRAME16k */
    {
        IF( L_sub(st_fx->core_brate_fx, ACELP_8k00) <= 0 )
        {
            cbrate = L_add(ACELP_8k00, 0);
        }
        ELSE IF( L_sub(st_fx->core_brate_fx, ACELP_14k80) <= 0 )
        {
            cbrate = L_add(ACELP_14k80, 0);
        }
        ELSE
        {
            cbrate = L_min(st_fx->core_brate_fx, ACELP_22k60);
        }

        /* subtract ACELP switching frame bits */
        if( L_sub(st_fx->core_brate_fx, ACELP_11k60) >= 0 )
        {
            (*num_bits) = sub((*num_bits), 1); /* LP_FLAG bit */
        }
        *num_bits = sub((*num_bits), ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(cbrate, GENERIC, 0, 0)]);     /* pitch bits*/
        *num_bits = sub((*num_bits), gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(cbrate, GENERIC, 0, 0)]);    /* gain bits */
        *num_bits = sub((*num_bits), FCB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(cbrate, GENERIC, 0, 0)]);     /* FCB bits  */
    }

    /* subtract BWE bits */
    test();
    test();
    IF( !( ( sub(inner_frame_tbl_fx[st_fx->bwidth_fx], L_FRAME16k) == 0 && sub(st_fx->last_L_frame_fx, L_FRAME16k) == 0 ) || sub(inner_frame_tbl_fx[st_fx->bwidth_fx], L_FRAME8k) == 0 ) )
    {
        *num_bits = sub((*num_bits), (NOOFGAINBITS1 + AUDIODELAYBITS));
    }

    /* reset state of old_out if switching */
    set16_fx( st_fx->old_out_fx, 0, output_frame );

    return;

}
