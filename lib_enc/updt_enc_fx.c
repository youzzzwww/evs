/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * updt_enc()
 *
 * Common updates (all frame types)
 *-------------------------------------------------------------------*/

void updt_enc_fx(
    Encoder_State_fx *st,             /* i/o: state structure                          */
    const Word16 L_frame,           /* i  : length of the frame                      */
    const Word16 coder_type,        /* i  : speech coder type                        */
    const Word16 *old_exc,          /* i  : buffer of excitation                     */
    const Word16 *pitch_buf,        /* i  : floating pitch for each subframe         */
    const Word16 Es_pred,           /* i  : predicited scaled innovation energy      */
    const Word16 *Aq,               /* i  : A(z) quantized for all subframes         */
    const Word16 *lsf_new,          /* i  : current frame LSF vector                 */
    const Word16 *lsp_new,          /* i  : current frame LSP vector                 */
    const Word16 *old_bwe_exc       /* i  : buffer of excitation                     */
)
{
    Word16 i, tmp;


    /* update old excitation buffer */
    Copy( &old_exc[L_frame], st->LPDmem.old_exc, L_EXC_MEM );
    IF( !st->Opt_AMR_WB_fx )
    {
        Copy( &old_bwe_exc[L_FRAME32k], st->old_bwe_exc_fx, PIT16k_MAX * 2 );
    }

    /* update old LSP and LSF vector */
    Copy( lsp_new, st->lsp_old_fx, M );
    Copy( lsf_new, st->lsf_old_fx, M );

    /* update last coder type */
    st->last_coder_type_fx = coder_type;
    move16();
    test();
    test();
    test();
    if( sub(coder_type,INACTIVE) == 0|| (sub(st->bpf_off_fx,1) == 0 && sub(coder_type,AUDIO) != 0 && sub(coder_type,TRANSITION) != 0) )
    {
        st->last_coder_type_fx = UNVOICED;
        move16();
    }

    /* this ensures that st->last_coder_type_fx is never set to INACTIVE in case of AVQ inactive because the FEC does not distinguish between GSC inactive and AVQ inactive */
    test();
    if ( sub(coder_type,INACTIVE) == 0 && L_sub(st->total_brate_fx,ACELP_24k40) > 0)
    {
        st->last_coder_type_fx = GENERIC;
        move16();
    }

    test();
    test();
    test();
    IF( st->Opt_AMR_WB_fx && sub(coder_type,INACTIVE) == 0 && st->core_brate_fx != SID_1k75 && st->core_brate_fx != FRAME_NO_DATA )
    {
        /* overwrite previous coding type to help FEC */
        st->last_coder_type_fx = UNVOICED;
        move16();
    }

    /* AC mode (GSC) - in speech we can consider that the last pitch band reached the max */
    test();
    IF ( sub(coder_type,AUDIO) != 0 && sub(coder_type,INACTIVE) != 0 )
    {
        st->mem_last_pit_band_fx = 10 + BAND1k2;
        move16();
        st->past_dyn_dec_fx = NOISE_LEVEL_SP0-1;
        move16();   /* tends to speech */
        st->noise_lev_fx = NOISE_LEVEL_SP0-1;
        move16();   /* tends to speech */
        set16_fx( st->old_y_gain_fx, 0, MBANDS_GN );
        move16();
        /*st->mid_dyn_fx = 40.0f * 0.5f + st->mid_dyn * 0.5f;*/
        st->mid_dyn_fx = add(20*128, mult_r(st->mid_dyn_fx, 16384));     /*Q7*/
    }

    /* convert old LSP vector from 12kHz domain to 16kHz domain (needed in case of ACELP@12k8 <-> ACELP@16kHz switching) */
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        Copy( st->lsp_old_fx, st->lsp_old16k_fx, M );
        st->rate_switching_reset_16kHz=lsp_convert_poly_fx( st->lsp_old16k_fx, L_FRAME16k, st->Opt_AMR_WB_fx );
    }

    /* update buffer of old subframe pitch values */
    IF( sub(st->last_L_frame_fx,L_frame) != 0 )
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            FOR( i=0; i<NB_SUBFR; i++ )
            {
                st->old_pitch_buf_fx[NB_SUBFR+i] = mult_r(26214, st->old_pitch_buf_fx[NB_SUBFR+i+1]);
                move16();
            }
        }
        ELSE
        {
            FOR( i=NB_SUBFR; i>0; i-- )
            {
                st->old_pitch_buf_fx[NB_SUBFR+i] = add(mult_r(8192, st->old_pitch_buf_fx[NB_SUBFR+i-1]),st->old_pitch_buf_fx[NB_SUBFR+i-1]);
                move16();
            }
            st->old_pitch_buf_fx[2*NB_SUBFR16k-1] = st->old_pitch_buf_fx[2*NB_SUBFR16k-2];
            move16();
        }
    }
    tmp = shr(L_frame,6);
    Copy( &st->old_pitch_buf_fx[tmp], st->old_pitch_buf_fx, tmp);
    Copy( pitch_buf, &st->old_pitch_buf_fx[tmp], tmp);

    /* SC-VBR */
    st->last_Opt_SC_VBR_fx = st->Opt_SC_VBR_fx;
    move16();
    st->last_last_ppp_mode_fx = st->last_ppp_mode_fx;
    move16();
    st->last_ppp_mode_fx = st->ppp_mode_fx;
    move16();
    st->last_nelp_mode_fx = st->nelp_mode_fx;
    move16();

    /* core switching updates */
    Copy( &Aq[(st->L_frame_fx/L_SUBFR-1)*(M+1)], st->old_Aq_12_8_fx, M+1 );
    st->old_Es_pred_fx = Es_pred;

    return;
}

/*-------------------------------------------------------------------*
  * updt_IO_switch()
  *
  * Common updates for AMR-WB IO mode and EVS primary mode switching
  *-------------------------------------------------------------------*/

void updt_IO_switch_enc_fx(
    Encoder_State_fx *st,          /* i/o: state structure             */
    const Word16 input_frame       /* i  : input frame length          */
)
{
    Word16 xsp_tmp[M];

    IF( sub(st->last_core_fx,AMR_WB_CORE) == 0 )  /* switching to EVS primary mode */
    {
        /* reset onset detection counter */
        st->tc_cnt_fx = -1;

        /* force safety-net LSFQ in the first frames after the switching */
        st->Nb_ACELP_frames_fx = 0;
        move16();

        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode LSF(LSP) */
        Copy( stable_LSP_fx, xsp_tmp, M );
        isf2lsf_fx( st->lsf_old_fx, st->lsf_old_fx, xsp_tmp);
        Copy( stable_LSP_fx, xsp_tmp, M );
        isp2lsp_fx( st->lsp_old_fx, st->lsp_old_fx, xsp_tmp, M);
        isp2lsp_fx( st->lsp_old1_fx, st->lsp_old1_fx, xsp_tmp, M);

        Copy( stable_LSP_fx, xsp_tmp, M );
        isp2lsp_fx( st->lspCNG_fx, st->lspCNG_fx, xsp_tmp, M);
        /*st->old_enr_index = min( (short)((float)st->old_enr_index / STEP_AMR_WB_SID * STEP_SID), 127 );*/
        IF( st->old_enr_index_fx >=0 )
        {
            /* old index is initialized to -1,  and may only be updated after it has been calculated properly at least once once */
            st->old_enr_index_fx = s_min(mult(shl(st->old_enr_index_fx,1), 32459), 127 ); /*32459 = 2/(STEP_SID/STEP_AMR_WB_SID)*/
        }
        /* Perform preemphasis of the old input signal @16kHz */
        st->mem_preemph16k_fx = 0;
        move16();
        preemph_fx( st->old_inp_16k_fx, PREEMPH_FAC_16k, L_INP_MEM, &(st->mem_preemph16k_fx) );
        Scale_sig(st->old_inp_16k_fx, L_INP_MEM, st->prev_Q_new);
        /* reset TD BWE buffers */
        set16_fx( st->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k  );
        set16_fx( st->old_speech_wb_fx, 0, (L_LOOK_12k8 + L_SUBFR) * 5/16 );
        set16_fx( st->old_bwe_exc_fx, 0, PIT16k_MAX * 2 );
        set16_fx( st->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );

        st->bwe_non_lin_prev_scale_fx = 0;
        move16();
        set16_fx( st->decim_state1_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
        set16_fx( st->decim_state2_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
        set16_fx( st->L_old_wtda_swb_fx, 0, L_FRAME16k );
        set16_fx( st->old_input_wb_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );

        wb_tbe_extras_reset_fx( st->mem_genSHBexc_filt_down_wb2_fx, st->mem_genSHBexc_filt_down_wb3_fx );

        IF(  sub(input_frame,L_FRAME32k) >= 0 )
        {
            swb_tbe_reset_fx( st->mem_csfilt_fx, st->mem_genSHBexc_filt_down_shb_fx, st->state_lpc_syn_fx,
                              st->syn_overlap_fx, st->state_syn_shbexc_fx, &(st->tbe_demph_fx),&(st->tbe_premph_fx), st->mem_stp_swb_fx, &(st->gain_prec_swb_fx) );

        }

        IF( sub(input_frame,L_FRAME48k) == 0 )
        {
            fb_tbe_reset_enc_fx( st->elliptic_bpf_2_48k_mem_fx, &st->prev_fb_energy_fx );

            fb_bwe_reset_enc_fx( st->elliptic_bpf_2_48k_mem_fx, &st->prev_energy_fbe_fb_fx );
        }

        /* reset FD BWE buffers */
        st->prev_mode_fx = NORMAL;
        move16();

        /* reset the unvoiced/audio signal improvement  memories */
        st->seed_tcx_fx = 15687;
        move16();

        st->use_acelp_preq = 0;
        move16();
    }
    ELSE            /* switching to AMR-WB IO mode */
    {
        set16_fx(st->mem_MA_fx, 0, M );

        /* AMR-WB IO mode uses ISF(ISP), but EVS primary mode LSF(LSP) */
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsf2isf_fx( st->lsf_old_fx, st->lsf_old_fx, xsp_tmp, M);
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsp2isp_fx( st->lsp_old_fx, st->lsp_old_fx, xsp_tmp, M);
        lsp2isp_fx( st->lsp_old1_fx, st->lsp_old1_fx, xsp_tmp, M);
        Copy( stable_ISP_fx, xsp_tmp, M );
        lsp2isp_fx( st->lspCNG_fx, st->lspCNG_fx, xsp_tmp, M);

        IF( st->old_enr_index_fx >= 0 )
        {
            /* old_enr__index is initialized to -1,  and may only be updated this way after it has been calculated properly at least once once */
            /*st->old_enr_index = min( (short)((float)st->old_enr_index / STEP_SID * STEP_AMR_WB_SID), 63 );*/
            st->old_enr_index_fx = s_max(s_min(mult(st->old_enr_index_fx, 16384), 63 ),0); /*32459 = 2/(STEP_SID/STEP_AMR_WB_SID)*/
        }
        /* gain quantization memory */
        set16_fx(st->past_qua_en_fx, (-14<<10), GAIN_PRED_ORDER );
    }

    /* Force SID in case of AMR-WB IO/EVS primary mode switching */
    st->cnt_SID_fx = 0;
    move16();

    /* CNG - reset */
    st->cng_cnt_fx = 0;
    move16();
    st->ho_hist_size_fx = 0;
    move16();
    st->burst_ho_cnt_fx = 0;
    move16();

    /* LP memories */
    Copy( UVWB_Ave_fx, st->mem_AR_fx, M );

    /* FEC - update adaptive LSF mean vector */
    Copy( st->lsf_old_fx, st->lsfoldbfi0_fx, M );
    Copy( st->lsf_old_fx, st->lsfoldbfi1_fx, M );
    Copy( st->lsf_old_fx, st->lsf_adaptive_mean_fx, M );

    return;
}
