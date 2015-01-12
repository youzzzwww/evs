/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"             /* Compilation switches                   */
#include "cnst_fx.h"             /* Common constants                       */
#include "rom_enc_fx.h"          /* Encoder static table prototypes        */
#include "rom_com_fx.h"          /* Static table prototypes                */
#include "prot_fx.h"             /* Function prototypes                    */
#include "stl.h"                 /* required for wmc_tool                  */


/*---------------------------------------------------------------------*
* core_switching_pre_enc()
*
* Preprocessing (preparing) for ACELP/HQ core switching
*---------------------------------------------------------------------*/
void core_switching_pre_enc_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure           */
    LPD_state *mem,             /* i/o: encoder state structure           */
    const Word16 input_frame,      /* i  : frame length                      */
    const Word16 *old_inp_12k8,    /* i  : old input signal @12.8kHz         */
    const Word16 *old_inp_16k      /* i  : old input signal @16kHz           */
)
{
    Word16 Sample_Delay_HP, Sample_Delay_LP;
    Word16 tmp16;
    Word16 tmp;

    /* Mode switching */
    IF( sub(st_fx->last_codec_mode,MODE2) == 0 )
    {
        st_fx->mem_deemph_fx = st_fx->LPDmem.syn[M];
        move16();

        Copy( mem->mem_syn2, st_fx->mem_syn1_fx, M );

        st_fx->igf = 0;
        move16();

        /* reset BWE memories */
        set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX*2 );
        /*st->last_extl = -1;*/
        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);

        set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );

        test();
        IF( sub(st_fx->last_core_fx,TCX_20_CORE)==0 || sub(st_fx->last_core_fx,TCX_10_CORE)==0 )
        {
            st_fx->last_core_fx = HQ_CORE;
            move16();

            set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
            set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
            st_fx->last_max_pos_pulse_fx = 0;
            move16();

            st_fx->mode_count_fx = 0;
            move16();
            st_fx->mode_count1_fx = 0;
            move16();

            set16_fx( st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM );
            st_fx->prev_frm_hfe2_fx = 0;
            move16();
            st_fx->prev_stab_hfe2_fx = 0;
            move16();

            /*ALDO overlap windowed past: also used in MODE2 but for other MDCT-LB*/
            set16_fx( st_fx->old_out_fx, 0, input_frame );
        }

        test();
        IF( (sub(st_fx->L_frame_fx,L_FRAME16k) == 0) && (sub(st_fx->last_L_frame_fx,L_FRAME) == 0))
        {
            Copy( st_fx->lsp_old_fx, st_fx->lsp_old16k_fx, M );

            lsp_convert_poly_fx( st_fx->lsp_old_fx, st_fx->L_frame_fx, 0 );
        }

        st_fx->use_acelp_preq = 0;
        move16();

    }

    test();
    IF( add(st_fx->last_core_fx, 1) == 0 && sub(st_fx->core_fx, HQ_CORE) == 0 )
    {
        /* very first frame is HQ_CORE */
        st_fx->last_core_fx = HQ_CORE;
        move16();
    }
    test();
    test();
    IF( sub(st_fx->core_fx, HQ_CORE) == 0 && ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0 ) ) /* HQ init */
    {
        set32_fx( st_fx->last_ni_gain_fx, 0, BANDS_MAX );
        set16_fx( st_fx->last_env_fx, 0, BANDS_MAX );
        st_fx->last_max_pos_pulse_fx = 0;
        move16();

        st_fx->mode_count_fx = 0;
        move16();
        st_fx->mode_count1_fx = 0;
        move16();

        set16_fx( st_fx->prev_SWB_peak_pos_fx, 0, NI_USE_PREV_SPT_SBNUM );
        st_fx->prev_frm_hfe2_fx = 0;
        move16();
        st_fx->prev_stab_hfe2_fx = 0;
        move16();

        set16_fx( st_fx->old_out_fx, 0, input_frame );
    }
    test();
    test();
    IF( ( sub(st_fx->core_fx, ACELP_CORE) == 0 || sub(st_fx->core_fx, AMR_WB_CORE) == 0 ) && sub(st_fx->last_core_fx, HQ_CORE) == 0 )
    {
        IF(sub(st_fx->L_frame_fx, L_FRAME16k)==0 )
        {
            Copy( TRWB2_Ave_fx, st_fx->lsf_old_fx, M ); /* init of LSP */
            lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, INT_FS_16k );
        }
        ELSE
        {
            Copy( TRWB_Ave_fx, st_fx->lsf_old_fx, M ); /* init of LSP */
            lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, INT_FS_FX );
        }

        st_fx->mem_deemph_fx = 0;
        move16();
        st_fx->LPDmem.syn[M] = 0;
        move16();
        set16_fx(mem->mem_syn2, 0, M );
        set16_fx( mem->mem_syn, 0, M );
        set16_fx( st_fx->mem_syn1_fx, 0, M );
        st_fx->Nb_ACELP_frames_fx = 0;
        move16();

        /* Reset ACELP parameters */
        set16_fx( st_fx->mem_MA_fx,0, M );
        Copy( GEWB_Ave_fx, st_fx->mem_AR_fx, M );
        mem->mem_w0 = 0;
        move16();
        mem->tilt_code = 0;
        move16();
        init_gp_clip_fx( st_fx->clip_var_fx );
        mem->gc_threshold = 0;
        move16();

        /* set16_fx( st_fx->dispMem, 0, 8 ); */
        set16_fx( st_fx->dm_fx.prev_gain_pit, 0, 6 );
        st_fx->dm_fx.prev_state = 0;
        move16();
        st_fx->dm_fx.prev_gain_code = L_deposit_l(0);

        st_fx->last_coder_type_fx = GENERIC;
        move16();

        st_fx->last_last_ppp_mode_fx = st_fx->last_ppp_mode_fx;
        move16();
        st_fx->last_ppp_mode_fx = st_fx->ppp_mode_fx;
        move16();
        st_fx->last_nelp_mode_fx = st_fx->nelp_mode_fx;
        move16();

        tmp16 = add(NB_SUBFR,1);
        move16();

        if( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            tmp16=NB_SUBFR;
            move16();
        }

        Copy( st_fx->old_pitch_buf_fx + tmp16, st_fx->old_pitch_buf_fx, tmp16 );
        set16_fx( st_fx->old_pitch_buf_fx + tmp16, L_SUBFR,tmp16);

        /* Reset old ACELP buffers */
        set16_fx( mem->old_exc, 0, L_EXC_MEM );
        set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX*2 );

        /* reset BWE memories */
        st_fx->bwe_non_lin_prev_scale_fx = 0;
        set16_fx( st_fx->old_syn_12k8_16k_fx, 0, NS2SA(16000, DELAY_FD_BWE_ENC_NS) );
    }

    test();
    test();
    IF( L_sub(st_fx->input_Fs_fx, 16000) >= 0 && sub(st_fx->last_extl_fx, WB_BWE) != 0 && sub(st_fx->extl_fx, WB_BWE) == 0 )
    {
        test();
        IF( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->last_extl_fx, FB_BWE) != 0 )
        {
            st_fx->prev_mode_fx = NORMAL;
            move16();
            st_fx->modeCount_fx = 0;
            move16();
        }

        st_fx->prev_L_swb_norm1_fx = 8;
        move16();
    }
    test();
    test();
    test();
    test();
    test();
    IF( ( L_sub(st_fx->input_Fs_fx, 32000) >= 0 && sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->extl_fx, SWB_BWE) == 0 ) ||
        ( L_sub(st_fx->input_Fs_fx, 48000) >= 0 && sub(st_fx->last_extl_fx, FB_BWE) != 0 && sub(st_fx->extl_fx, FB_BWE) == 0 ) )
    {
        /* we are switching to SWB BWE - reset SWB BWE buffers */

        IF( sub(st_fx->L_frame_fx, L_FRAME) == 0 )
        {
            Sample_Delay_HP = NS2SA( 16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS );
            Sample_Delay_LP = NS2SA( 12800, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS );

            Copy( old_inp_12k8 + sub(L_INP_MEM + L_FRAME, Sample_Delay_LP), st_fx->old_input_lp_fx, Sample_Delay_LP );
        }
        ELSE
        {
            Sample_Delay_HP = NS2SA( 16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_16k_NS + DELAY_FIR_RESAMPL_NS - DELAY_CLDFB_NS );
            Sample_Delay_LP = NS2SA( 16000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_16k_NS );
            Copy( old_inp_16k + sub(L_INP_MEM + L_FRAME, Sample_Delay_LP), st_fx->old_input_lp_fx, Sample_Delay_LP );
        }

        tmp = sub(ACELP_LOOK_16k + L_SUBFR16k, Sample_Delay_HP);
        Copy( &st_fx->old_speech_shb_fx[tmp], st_fx->new_input_hp_fx, Sample_Delay_HP );
        add(0,0);

        IF (sub(st_fx->last_extl_fx,WB_BWE) != 0)
        {
            st_fx->prev_mode_fx = NORMAL;
            move16();
            st_fx->modeCount_fx = 0;
            move16();
        }

        st_fx->prev_L_swb_norm1_fx = 8;
        move16();/*8.0 in Q0 */

    }

    return;
}



/*---------------------------------------------------------------------*
* core_switching_post_enc()
*
* Postprocessing for ACELP/HQ core switching
*---------------------------------------------------------------------*/
void core_switching_post_enc_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure             */
    const Word16 old_inp_12k8[],    /* i  : input signal @12.8 kHz  Qinp        */
    const Word16 old_inp_16k[],     /* i  : input signal @16 kHz    Qinp        */
    const Word16 pitch[3],          /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],        /* i  : Open-loop pitch gains               */
    const Word16 A[],               /* i  : unquant. LP filter coefs. (Q12)     */
    Word16 Qshift,
    Word16 Q_new,
    const Word16 Qsp,               /* i/o  : Q from acelp synthsis */
    Word16 *Qmus              /* i/o  : Q from mdct synthsis / Q of output synthesis  */
)
{
    Word16 T_op[3];
    Word16 synth_subfr_bwe[SWITCH_MAX_GAP];              /* synthesized bwe for core switching */

    Copy( pitch, T_op, 3 );

    IF( sub(st_fx->core_fx, HQ_CORE) == 0 )
    {
        st_fx->use_acelp_preq = 0;
        move16();

        test();
        IF( ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0) )  /* core switching ==> CELP subframe encoding */
        {
            acelp_core_switch_enc_fx( st_fx, &(st_fx->LPDmem),old_inp_12k8 + L_INP_MEM - NS2SA_fx2(INT_FS_FX, ACELP_LOOK_NS),
                                      old_inp_16k  + L_INP_MEM - NS2SA_fx2(INT_FS_16k, ACELP_LOOK_NS), T_op, voicing, A, synth_subfr_bwe, Qshift, Q_new );
        }

        st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
        st_fx->mem_deemph_old_syn_fx = 0;
        move16();

    }
    ELSE
    {
        *Qmus=Qsp; /* Write Qout */

        /* reset SWB TBE buffers */
        test();
        IF( sub(st_fx->extl_fx, WB_TBE) == 0 && sub(st_fx->last_extl_fx, WB_TBE) != 0 )
        {
            wb_tbe_extras_reset_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx );

            IF ( sub(st_fx->last_extl_fx, WB_BWE) != 0 )
            {
                set16_fx( st_fx->decim_state1_fx, 0, 2*ALLPASSSECTIONS_STEEP+1 );
                set16_fx( st_fx->decim_state2_fx, 0, 2*ALLPASSSECTIONS_STEEP+1 );
            }

            set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD/4 );
            set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );
            set32_fx( st_fx->mem_csfilt_fx, 0, 2 );
        }

        test();
        IF( sub(st_fx->extl_fx, SWB_TBE) == 0 || sub(st_fx->extl_fx, FB_TBE) == 0 )
        {
            test();
            test();
            IF( sub(st_fx->last_core_fx,HQ_CORE)==0 ||sub(st_fx->L_frame_fx, st_fx->last_L_frame_fx) != 0 )
            {
                set16_fx( st_fx->state_ana_filt_shb_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
                set16_fx( st_fx->old_speech_shb_fx, 0, ACELP_LOOK_16k + L_SUBFR16k );

                swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                                  st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx), &(st_fx->tbe_premph_fx),
                                  st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx));

                set16_fx(st_fx->dec_2_over_3_mem_fx,0, 12);
                set16_fx(st_fx->dec_2_over_3_mem_lp_fx,0, 6);

            }
            ELSE IF( sub(st_fx->last_extl_fx, SWB_TBE) != 0 && sub(st_fx->last_extl_fx, FB_TBE) != 0 )
            {
                set16_fx( st_fx->state_ana_filt_shb_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
                set16_fx( st_fx->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k );
                swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                                  st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx), &(st_fx->tbe_premph_fx),
                                  st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx));

                set16_fx(st_fx->dec_2_over_3_mem_fx,0, 12);
                set16_fx(st_fx->dec_2_over_3_mem_lp_fx,0, 6);
            }
        }
        test();
        test();
        IF( sub(st_fx->extl_fx, FB_TBE) == 0 && ( sub(st_fx->last_extl_fx, FB_TBE) != 0 || sub(st_fx->L_frame_fx, st_fx->last_L_frame_fx) != 0 ) )
        {
            fb_tbe_reset_enc_fx( st_fx->elliptic_bpf_2_48k_mem_fx, &st_fx->prev_fb_energy_fx );
        }
        test();
        test();
        test();
        IF( ( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->extl_fx, SWB_BWE) == 0 ) || ( sub(st_fx->last_extl_fx, FB_BWE) != 0 && sub(st_fx->extl_fx, FB_BWE) == 0 ) )
        {
            fb_bwe_reset_enc_fx( st_fx->elliptic_bpf_2_48k_mem_fx, &st_fx->prev_energy_fbe_fb_fx );
            st_fx->EnergyLF_fx = L_deposit_l(0);
        }

    }

    return;
}

/*---------------------------------------------------------------------*
* core_switching_hq_prepare_enc()
*
* Preprocessing in the first HQ frame after ACELP frame
* - modify bit allocation for HQcore removing CELP subframe budget
* - update st->old_wtda to modify windows at the encoder
*---------------------------------------------------------------------*/

void core_switching_hq_prepare_enc_fx(
    Encoder_State_fx *st_fx,          /* i/o: encoder state structure */
    Word16 *num_bits,         /* i/o: bit budget update       */
    const Word16 input_frame,       /* i  : frame length            */
    Word32 *wtda_audio,       /* shall be q_audio + 15, audio allready scalled in wtda function  */
    const Word16 *audio
)
{
    Word16 delta,  Loverlapp, i;
    Word16 tmp16;
    Word32 cbrate, *pt_32;
    const Word16 *pt_cos, *pt_16;
    Word16 n;

    SWITCH (input_frame)
    {
    case L_FRAME8k:
        delta = 1;
        move16();
        pt_cos= sin_switch_8;
        BREAK;
    case L_FRAME16k:
        delta = 2;
        move16();
        pt_cos= sin_switch_16;
        BREAK;
    case L_FRAME32k:
        delta = 4;
        move16();
        pt_cos= sin_switch_32;
        BREAK;
    default :
        delta = 6;
        move16();
        pt_cos= sin_switch_48;
        BREAK;
    }

    /* set switching frame bit-rate */
    IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
    {
        IF( L_sub(st_fx->core_brate_fx, ACELP_24k40) > 0 )
        {
            cbrate = L_add(ACELP_24k40, 0);
        }
        ELSE
        {
            cbrate = L_add(st_fx->core_brate_fx, 0);
        }

        /* subtract ACELP switching frame bits */
        IF( L_sub(st_fx->core_brate_fx, ACELP_11k60) >= 0 )
        {
            (*num_bits) = sub((*num_bits), 1); /* LP_FLAG bit */
        }
        *num_bits = sub( (*num_bits), ACB_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, GENERIC, 0, 0)] );      /* pitch bits*/
        *num_bits = sub( (*num_bits), gain_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, TRANSITION, 0, 0)] );  /* gain bits */
        *num_bits = sub( (*num_bits), FCB_bits_tbl[BIT_ALLOC_IDX_fx(cbrate, GENERIC, 0, 0)] );      /* FCB bits  */
    }
    ELSE  /* L_frame == L_FRAME16k */
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
        IF( L_sub(st_fx->core_brate_fx, ACELP_11k60) >= 0 )
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
    IF( !( ( sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME16k) == 0 && sub(st_fx->last_L_frame_fx, L_FRAME16k) == 0 ) || sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME8k) == 0 ) )
    {
        *num_bits = sub((*num_bits), (NOOFGAINBITS1 + AUDIODELAYBITS));
    }

    /* Transition window at the encoder */
    n=i_mult2(N_ZERO_8,delta);
    Loverlapp=i_mult2(SWITCH_OVERLAP_8k,delta);
    /*Overflow=0; */

    pt_32 = wtda_audio+shr(input_frame,1);
    pt_16 = audio+sub(n,1);
    tmp16 = sub(shr(input_frame,1),Loverlapp);

    FOR( i = 0; i < tmp16; i++ )
    {
        /* wtda_audio[i+input_frame/2] = - audio[n-i-1]; */
        *pt_32++=L_negate(L_shr(L_deposit_h(*pt_16--),1));   /*   Q +16 -1  */
    }

    pt_cos = pt_cos + Loverlapp - 1;
    FOR( i = tmp16; i < shr(input_frame,1); i++ )
    {
        /* *pt_32++ =  - audio[n-i-1] *(float)cos((i+1-input_frame/2+Loverlapp)*EVS_PI/(2*(Loverlapp+1)));   win=cos() */
        *pt_32++=L_negate(L_mult0(*pt_16--,*pt_cos--));
    }

    /* reset state of old_out if switching */
    set16_fx( st_fx->old_out_fx,0,input_frame);

    return;
}
