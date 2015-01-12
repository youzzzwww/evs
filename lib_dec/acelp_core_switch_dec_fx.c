/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool */

/*---------------------------------------------------------------------*
* Local functions
*---------------------------------------------------------------------*/

static void decod_gen_voic_core_switch_fx( Decoder_State_fx *st_fx, const Word16 L_frame, const Word16 sharpFlag, const Word16 *Aq, const Word16 coder_type,
        Word16 *exc, const Word32  core_brate, Word16 *Q_exc );

/*-------------------------------------------------------------------*
* acelp_core_switch_dec()
*
* ACELP core decoder in the first ACELP->HQ switching frame
*-------------------------------------------------------------------*/

void acelp_core_switch_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure             */
    Word16 *synth_subfr_out,            /* o  : synthesized ACELP subframe     Q_syn*/
    Word16 *tmp_synth_bwe,              /* o  : synthesized ACELP subframe BWE Q_syn*/
    const Word16 output_frame,          /* i  : input frame length                  */
    const Word16 core_switching_flag,   /* i  : core switching flag                 */
    Word16 *mem_synth,                  /* o  : synthesis to overlap                */
    Word16 *Q_syn
)
{
    Word16 i, delta,  L_frame_for_cs, decode_bwe, tmp;
    Word16 d1m, ind1, fdelay, gapsize;
    Word32 cbrate;
    Word16 synth_intFreq[2*L_SUBFR];
    CLDFB_SCALE_FACTOR scaleFactor;
    Word32 workBuffer[128*3];
    Word16 old_exc[L_EXC_DEC], *exc;
    Word16 tmp_mem2[2*L_FILT48k], gain;
    Word16 hb_synth_tmp[NS2SA(48000,10000000)];
    const Word16 *hp_filter;
    Word16 Aq[2*(M+1)];
    Word16 bpf_error_signal[2*L_SUBFR];
    Word16 *pt1, *pt2;
    Word16 syn_fx_tmp[L_FRAME_16k];
    Word32 *realBuffer[CLDFB_NO_COL_MAX_SWITCH], *imagBuffer[CLDFB_NO_COL_MAX_SWITCH];
    Word32 realBufferTmp[CLDFB_NO_COL_MAX_SWITCH][CLDFB_NO_CHANNELS_MAX], imagBufferTmp[CLDFB_NO_COL_MAX_SWITCH][CLDFB_NO_CHANNELS_MAX];

    FOR( i=0; i<CLDFB_NO_COL_MAX_SWITCH; i++ )
    {
        set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        realBuffer[i] = realBufferTmp[i];
        move32();
        imagBuffer[i] = imagBufferTmp[i];
        move32();
    }

    /* initializations */
    d1m = 0;
    move16();
    gain = 0;
    move16();

    Copy( st_fx->old_Aq_12_8_fx, Aq, M+1 );
    Copy( st_fx->old_Aq_12_8_fx, Aq + (M+1), M+1 );

    set16_fx( mem_synth, 0, NS2SA(16000, DELAY_CLDFB_NS)+2 );
    set16_fx( synth_subfr_out, 0, SWITCH_MAX_GAP ); /* avoid valgrind complaining about uninitialized memory in core_switching_OLA_fx() */

    /* set multiplication factor according to the sampling rate */
    delta = 1;
    if( sub(output_frame, L_FRAME16k) > 0 )
    {
        delta = shr(output_frame,8);
    }

    /*----------------------------------------------------------------*
    * set switching frame bit-rate
    *----------------------------------------------------------------*/

    test();
    test();
    test();
    IF( core_switching_flag && sub(st_fx->last_L_frame_fx, st_fx->last_L_frame_ori_fx)  == 0 && ( (sub(st_fx->last_core_fx, ACELP_CORE) == 0) || (sub(st_fx->last_core_fx, AMR_WB_CORE) == 0) ) )
    {
        exc = old_exc + L_EXC_MEM_DEC;
        Copy( st_fx->old_exc_fx, old_exc, L_EXC_MEM_DEC ); /*scaling of exc from previous frame*/

        IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
        {
            cbrate = L_add(st_fx->core_brate_fx, 0);
            if( L_sub(cbrate, ACELP_24k40) > 0 )
            {
                cbrate = L_add(ACELP_24k40, 0);
            }

            L_frame_for_cs = L_FRAME;
            move16();
        }
        ELSE
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
                cbrate = L_min(st_fx->core_brate_fx, ACELP_22k60 );
            }

            L_frame_for_cs = L_FRAME16k;
            move16();
        }

        /*----------------------------------------------------------------*
        * Excitation decoding
        *----------------------------------------------------------------*/

        decod_gen_voic_core_switch_fx( st_fx, L_frame_for_cs, 0, Aq, GENERIC, exc, cbrate, &st_fx->Q_exc );

        /*----------------------------------------------------------------*
        * synthesis, deemphasis, postprocessing and resampling
        *----------------------------------------------------------------*/

        /* Core synthesis at 12.8kHz or 16kHz */
        Rescale_mem( st_fx->Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx,
                     4, &st_fx->mem_deemph_fx, st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx,&st_fx->agc_mem_fx[1], &st_fx->pfstat ,1, NULL );

        syn_12k8_fx( 2*L_SUBFR, Aq, exc, synth_intFreq, st_fx->mem_syn2_fx, 1, st_fx->Q_exc, st_fx->Q_syn );

        IF(st_fx->pfstat.on && (sub(st_fx->last_bwidth_fx,NB) == 0))
        {
            Word16 tmp_noise, pitch_buf_tmp[2];
            tmp_noise = 0;
            FOR( i=0; i<2; i++ )
            {
                pitch_buf_tmp[i] = L_SUBFR;
                move16();
            }
            nb_post_filt( 2*L_SUBFR, &(st_fx->pfstat), &tmp_noise, 0, synth_intFreq, Aq, pitch_buf_tmp, AUDIO, 0 );
        }

        IF( sub(L_frame_for_cs,L_FRAME)==0 )
        {
            deemph_fx( synth_intFreq, PREEMPH_FAC, 2*L_SUBFR, &(st_fx->mem_deemph_fx) );
        }
        ELSE
        {
            deemph_fx( synth_intFreq, PREEMPH_FAC_16k, 2*L_SUBFR, &(st_fx->mem_deemph_fx) );
        }

        unscale_AGC( synth_intFreq, st_fx->Q_syn, syn_fx_tmp+M, st_fx->agc_mem_fx, 2*L_SUBFR );
        Copy( syn_fx_tmp+M, synth_intFreq, 2*L_SUBFR );

        test();
        IF( st_fx->pfstat.on && (sub(st_fx->last_bwidth_fx,NB) != 0) )
        {
            Copy( st_fx->pfstat.mem_pf_in+L_SYN_MEM-M, syn_fx_tmp, M );
            Residu3_fx ( Aq, syn_fx_tmp + M, exc, L_SUBFR, 1 );
            E_UTIL_synthesis ( 1, Aq, exc, syn_fx_tmp, L_SUBFR, st_fx->pfstat.mem_stp+L_SYN_MEM-M, 0, M );
            scale_st ( syn_fx_tmp, synth_intFreq, &st_fx->pfstat.gain_prec, L_SUBFR );
        }
        st_fx->pfstat.on = 0;
        move16();

        IF ( st_fx->flag_cna != 0 )
        {
            generate_masking_noise( synth_intFreq, st_fx->Q_syn, st_fx->hFdCngDec_fx->hFdCngCom, 2*L_SUBFR, 0);
        }

        /*----------------------------------------------------------------*
         * Resample to the output sampling rate (8/16/32/48 kHz)
         * Bass post-filter
         *----------------------------------------------------------------*/

        /* bass post-filter (when "bpf_error_signal" is defined, no additional delay is introduced) */
        bass_psfilter_fx( st_fx->Opt_AMR_WB_fx, synth_intFreq, 2*L_SUBFR, NULL, st_fx->pst_old_syn_fx,
                          &st_fx->pst_mem_deemp_err_fx, &st_fx->pst_lp_ener_fx, st_fx->bpf_off_fx, st_fx->stab_fac_fx, &st_fx->stab_fac_smooth_fx,
                          st_fx->mem_mean_pit_fx, st_fx->Track_on_hist_fx, st_fx->vibrato_hist_fx, &st_fx->psf_att_fx, GENERIC,
                          st_fx->Q_syn, bpf_error_signal );

        cldfb_save_memory( st_fx->cldfbAna_fx );
        cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &scaleFactor, synth_intFreq,
                                negate(st_fx->Q_syn), CLDFB_NO_COL_MAX_SWITCH, workBuffer);
        cldfb_restore_memory( st_fx->cldfbAna_fx );

        /* CLDFB synthesis of the combined signal */
        scaleFactor.hb_scale = scaleFactor.lb_scale;
        move16();

        /* CLDFB analysis and add the BPF error signal */
        cldfb_save_memory( st_fx->cldfbBPF_fx );
        move16();
        move16(); /* bpf_length */
        addBassPostFilterFx( bpf_error_signal, realBuffer, imagBuffer, st_fx->cldfbBPF_fx, workBuffer,
                             negate(st_fx->Q_syn), (st_fx->bpf_off_fx == 0) ? CLDFB_NO_COL_MAX_SWITCH : 0,
                             CLDFB_NO_COL_MAX_SWITCH, st_fx->cldfbAna_fx->no_channels, &scaleFactor );
        cldfb_restore_memory( st_fx->cldfbBPF_fx );

        /* CLDFB synthesis of the combined signal */
        scaleFactor.hb_scale = scaleFactor.lb_scale;

        cldfb_save_memory( st_fx->cldfbSyn_fx );
        cldfbSynthesisFiltering(st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &scaleFactor, synth_subfr_out, 0, CLDFB_NO_COL_MAX_SWITCH, workBuffer);
        cldfb_restore_memory( st_fx->cldfbSyn_fx );
        *Q_syn = 0;
        move16();

        Copy_Scale_sig( synth_intFreq+ NS2SA(L_frame_for_cs * 50, SWITCH_GAP_LENGTH_NS-DELAY_CLDFB_NS)-2, mem_synth, NS2SA(L_frame_for_cs * 50, DELAY_CLDFB_NS)+2, negate(st_fx->Q_syn) ); /* Copy mem with Q0 */

        /*----------------------------------------------------------------*
        * BWE decoding
        *----------------------------------------------------------------*/

        decode_bwe = 0;
        move16();
        test();
        test();
        IF( !(( sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME16k) == 0 && sub(st_fx->last_L_frame_fx, L_FRAME16k) == 0) || sub(inner_frame_tbl[st_fx->bwidth_fx], L_FRAME8k) == 0) )
        {
            /* Decoding of BWE */
            d1m = (Word16)get_next_indice_fx(st_fx, AUDIODELAYBITS);
            ind1 = (Word16)get_next_indice_fx(st_fx, NOOFGAINBITS1);
            gain = usdequant_fx( ind1, MINVALUEOFFIRSTGAIN_FX, shr(DELTAOFFIRSTGAIN_FX,3) ); /*Q13*/
            decode_bwe = 1;
            move16();
        }

        test();
        test();
        test();
        IF( decode_bwe && !(( sub(output_frame, L_FRAME16k) == 0 && sub(st_fx->last_L_frame_fx, L_FRAME16k) == 0) || sub(output_frame, L_FRAME8k) == 0) )
        {
            set16_fx( tmp_mem2, 0, 2*L_FILT48k );

            hp_filter = hp16000_48000_fx;
            fdelay = 48;
            move16();
            IF ( L_sub(st_fx->output_Fs_fx, 16000) == 0 )
            {
                IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
                {
                    hp_filter = hp12800_16000_fx;
                    fdelay = 20;
                    move16();
                }
            }
            ELSE IF( L_sub(st_fx->output_Fs_fx, 32000) == 0 )
            {
                IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
                {
                    hp_filter = hp12800_32000_fx;
                    fdelay = 40;
                    move16();
                }
                ELSE
                {
                    hp_filter = hp16000_32000_fx;
                    fdelay = 32;
                    move16();
                }
            }
            ELSE IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
            {
                hp_filter = hp12800_48000_fx;
                fdelay = 60;
                move16();
            }

            /*I assume st_fx->old_synth_sw_fx is Q0 */
            fir_fx( st_fx->old_synth_sw_fx, hp_filter, hb_synth_tmp, tmp_mem2, shr(output_frame,1) , fdelay, 1, 0 );

            set16_fx( tmp_synth_bwe, 0, SWITCH_MAX_GAP );

            gapsize = i_mult2(delta, (NS2SA(16000,SWITCH_GAP_LENGTH_NS)));

            pt1 = tmp_synth_bwe;
            tmp = add(i_mult(d1m,delta),fdelay);
            pt2 = &hb_synth_tmp[tmp];
            FOR( i=0; i<gapsize; i++ )
            {
                *pt1++ = round_fx(L_shl(L_mult((*pt2++), gain), 2));
            }
        }
        ELSE
        {
            set16_fx( tmp_synth_bwe, 0, SWITCH_MAX_GAP );
        }
    }

    return;
}


/*-------------------------------------------------------------------*
* acelp_core_switch_dec_bfi()
*
* ACELP core decoder in the first ACELP->HQ switching frame in case of BAD frame
*-------------------------------------------------------------------*/

void acelp_core_switch_dec_bfi_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure */
    Word16 synth_out[],             /* o  : synthesis Q_syn         */
    const Word16 coder_type         /* i  : coder type              */
)
{
    Word16 old_exc[L_EXC_DEC], *exc;                     /* excitation signal buffer              */
    Word16 syn[L_FRAME16k];                              /* synthesis signal buffer               */
    Word16 lsf_new[M];                                   /* LSFs at the end of the frame          */
    Word16 lsp_new[M];                                   /* LSPs at the end of the frame          */
    Word16 Aq[NB_SUBFR16k*(M+1)];                        /* A(q)   quantized for the 4 subframes  */
    Word16 old_exc2[L_FRAME16k + L_EXC_MEM], *exc2;      /* total excitation buffer               */
    Word16 tmp_noise;                                    /* Long term temporary noise energy      */
    Word16 FEC_pitch;                                    /* FEC pitch                             */
    Word16 old_bwe_exc[((PIT16k_MAX + (L_FRAME16k+1) + L_SUBFR16k) * 2)]; /* excitation buffer */
    Word16 *bwe_exc;                                     /* Excitation for SWB TBE */
    Word16 tmp_float[NBPSF_PIT_MAX];
    Word16 tmp_float2[NBPSF_PIT_MAX];
    Word16 tmp_float3;
    Word16 tmp_float4[L_TRACK_HIST];
    Word16 tmp_float5[L_TRACK_HIST];
    Word16 tmp_float6[L_TRACK_HIST];
    Word16 tmp_float7;
    Word32 tmp_float32;
    Word16 voice_factors[NB_SUBFR16k];
    Word16 pitch_buf[NB_SUBFR16k];
    Word16 Q_exc;
    Word32 *realBuffer[CLDFB_NO_COL_MAX_SWITCH_BFI], *imagBuffer[CLDFB_NO_COL_MAX_SWITCH_BFI];
    Word32 realBufferTmp[CLDFB_NO_COL_MAX_SWITCH_BFI][CLDFB_NO_CHANNELS_MAX], imagBufferTmp[CLDFB_NO_COL_MAX_SWITCH_BFI][CLDFB_NO_CHANNELS_MAX];
    Word16 i;
    CLDFB_SCALE_FACTOR scaleFactor;
    Word32 workBuffer[128*3];

    FOR( i=0; i<CLDFB_NO_COL_MAX_SWITCH_BFI; i++ )
    {
        set32_fx(realBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        set32_fx(imagBufferTmp[i], 0, CLDFB_NO_CHANNELS_MAX);
        realBuffer[i] = realBufferTmp[i];
        imagBuffer[i] = imagBufferTmp[i];
    }

    /*----------------------------------------------------------------*
    * Initialization
    *----------------------------------------------------------------*/
    Q_exc = st_fx->Q_exc;
    move16();
    st_fx->bpf_off_fx = 1;
    move16();
    st_fx->clas_dec = st_fx->last_good_fx;
    move16();
    tmp_noise = 0;
    move16();

    Copy( st_fx->old_exc_fx, old_exc, L_EXC_MEM_DEC );
    exc = old_exc + L_EXC_MEM_DEC;
    Copy( st_fx->old_exc2_fx, old_exc2, L_EXC_MEM );
    exc2 = old_exc2 + L_EXC_MEM;
    Copy( st_fx->old_bwe_exc_fx, old_bwe_exc, PIT16k_MAX * 2);
    bwe_exc = old_bwe_exc + PIT16k_MAX * 2;
    st_fx->GSC_noisy_speech_fx = 0;
    move16();
    st_fx->relax_prev_lsf_interp_fx = 0;
    move16();

    /* SC-VBR */
    if( sub(st_fx->last_nelp_mode_dec_fx, 1) == 0 )
    {
        st_fx->nelp_mode_dec_fx = 1;
        move16();
    }

    Copy(st_fx->mem_AR_fx,tmp_float,M);
    Copy(st_fx->mem_MA_fx,tmp_float2,M);

    /* LSF estimation and A(z) calculation */
    FEC_lsf_estim_fx( st_fx, st_fx->L_frame_fx, Aq, lsf_new, lsp_new ); /*Scaling of last three???*/

    Copy( tmp_float, st_fx->mem_AR_fx, M );
    Copy( tmp_float2, st_fx->mem_MA_fx, M );

    /*----------------------------------------------------------------*
    * Excitation decoding
    *----------------------------------------------------------------*/

    IF( sub(st_fx->nelp_mode_dec_fx, 1) == 0 )
    {
        Word16 gain_buf[NB_SUBFR16k];
        Scale_sig(exc-L_EXC_MEM, L_EXC_MEM, -st_fx->Q_exc);
        st_fx->Q_exc = 0;
        /* SC-VBR */
        decod_nelp_fx( st_fx, coder_type, &tmp_noise, pitch_buf, exc, exc2, voice_factors, bwe_exc, &Q_exc, st_fx->bfi_fx, gain_buf );
        FEC_pitch = pitch_buf[3];
        move16();
        Rescale_exc( st_fx->dct_post_old_exc_fx, exc, NULL, st_fx->last_exc_dct_in_fx, L_FRAME, 0, (Word32)0, &Q_exc, st_fx->Q_subfr, exc2, L_FRAME, coder_type);
        st_fx->Q_exc = Q_exc;
    }
    ELSE
    {
        tmp_float[0] = st_fx->bfi_pitch_fx;
        move16();
        tmp_float[1] = st_fx->bfi_pitch_frame_fx;
        move16();
        tmp_float[2] = st_fx->lp_gainp_fx;
        move16();
        tmp_float[3] = st_fx->lp_gainc_fx;
        move16();
        tmp_float[4] = st_fx->upd_cnt_fx;
        move16();
        tmp_float[5] = st_fx->seed_fx;
        move16();

        /* calculation of excitation signal */
        FEC_exc_estim_fx( st_fx, st_fx->L_frame_fx, exc, exc2, syn /* dummy buffer */, pitch_buf, voice_factors, &FEC_pitch, bwe_exc, lsf_new, &Q_exc, &tmp_noise );
        Rescale_exc( NULL, exc, bwe_exc, st_fx->last_exc_dct_in_fx, st_fx->L_frame_fx, L_FRAME32k, (Word32)0,
        &Q_exc, st_fx->Q_subfr, exc2, st_fx->L_frame_fx, st_fx->last_coder_type_fx);
        st_fx->seed_fx = tmp_float[5];
        move16();
        st_fx->bfi_pitch_fx = tmp_float[0];
        move16();
        st_fx->bfi_pitch_frame_fx = tmp_float[1];
        move16();
        st_fx->lp_gainp_fx = tmp_float[2];
        move16();
        st_fx->lp_gainc_fx = tmp_float[3];
        move16();
        st_fx->upd_cnt_fx = tmp_float[4];
        move16();
    }

    /*------------------------------------------------------------------*
    * Synthesis
    *-----------------------------------------------------------------*/

    Rescale_mem( Q_exc, &st_fx->prev_Q_syn, &st_fx->Q_syn, st_fx->mem_syn2_fx, st_fx->mem_syn_clas_estim_fx, 4, &st_fx->mem_deemph_fx,
                 st_fx->pst_old_syn_fx, &st_fx->pst_mem_deemp_err_fx, &st_fx->agc_mem_fx[1], &st_fx->pfstat, 1, NULL );
    Copy( st_fx->mem_syn2_fx,tmp_float,M);
    syn_12k8_fx( st_fx->L_frame_fx, Aq, exc2, syn, tmp_float, 1, Q_exc, st_fx->Q_syn );

    tmp_float32 = st_fx->enr_old_fx;
    frame_ener_fx( st_fx->L_frame_fx, st_fx->last_good_fx, syn, shr(add(FEC_pitch,32),6), &tmp_float32, st_fx->L_frame_fx, st_fx->Q_syn, 3, 0 );

    /*------------------------------------------------------------------*
    * Perform fixed deemphasis through 1/(1 - g*z^-1)
    *-----------------------------------------------------------------*/
    tmp_float[0] = st_fx->mem_deemph_fx;
    move16(); /*if in acelp_core_dec_fx deemph_fx is used*/
    /*tmp_float = shr(st_fx->mem_deemph_fx, sub(st_fx->Q_syn,1));      if in acelp_core_dec_fx Deemph2 is used*/

    IF(sub(st_fx->L_frame_fx,L_FRAME )==0)
    {
        deemph_fx( syn, PREEMPH_FAC, L_FRAME, &tmp_float[0] ); /*Q0*/
    }
    ELSE
    {
        deemph_fx( syn, PREEMPH_FAC_16k, L_FRAME16k, &tmp_float[0] ); /*Q0*/
    }

    /*----------------------------------------------------------------*
    * Bass post-filter
    *----------------------------------------------------------------*/

    st_fx->bpf_off_fx=1;
    move16();
    Copy( st_fx->pst_old_syn_fx,tmp_float,NBPSF_PIT_MAX);

    tmp_float3 = st_fx->stab_fac_smooth_fx;
    move16();
    Copy( st_fx->mem_mean_pit_fx,tmp_float4, L_TRACK_HIST);
    Copy( st_fx->Track_on_hist_fx,tmp_float5, L_TRACK_HIST);
    Copy( st_fx->vibrato_hist_fx,tmp_float6, L_TRACK_HIST);
    tmp_float7 = st_fx->psf_att_fx;
    move16();

    /* apply bass post-filter (introduces delay of DELAY_BPF) */
    bass_psfilter_fx( st_fx->Opt_AMR_WB_fx, syn, st_fx->L_frame_fx, pitch_buf, tmp_float,
                      &st_fx->pst_mem_deemp_err_fx, &st_fx->pst_lp_ener_fx, st_fx->bpf_off_fx, st_fx->stab_fac_fx, &tmp_float3,
                      tmp_float4, tmp_float5, tmp_float6, &tmp_float7, coder_type, st_fx->Q_syn, old_exc /* tmp buffer*/);

    /*----------------------------------------------------------------*
    * Resamping to the output sampling frequency
    *----------------------------------------------------------------*/
    /* CLDFB analysis of the synthesis at internal sampling rate */
    cldfb_save_memory( st_fx->cldfbAna_fx );
    cldfbAnalysisFiltering( st_fx->cldfbAna_fx, realBuffer, imagBuffer, &scaleFactor, syn,
                            negate(st_fx->Q_syn), CLDFB_NO_COL_MAX_SWITCH_BFI, workBuffer);
    cldfb_restore_memory( st_fx->cldfbAna_fx );

    scaleFactor.hb_scale = scaleFactor.lb_scale;

    /* CLDFB synthesis of the combined signal */
    cldfb_save_memory( st_fx->cldfbSyn_fx );
    cldfbSynthesisFiltering( st_fx->cldfbSyn_fx, realBuffer, imagBuffer, &scaleFactor, synth_out,
                             negate(st_fx->Q_syn), CLDFB_NO_COL_MAX_SWITCH_BFI, workBuffer );

    /* output to Q0 */
    Scale_sig(synth_out,L_FRAME48k, negate(st_fx->Q_syn));

    cldfb_restore_memory( st_fx->cldfbSyn_fx );

    return;
}


/*-------------------------------------------------------------------*
* decod_gen_voic_core_switch()
*
* Decode excitation signal in teh first ACELP->HQ switching frame
*-------------------------------------------------------------------*/

static void decod_gen_voic_core_switch_fx(
    Decoder_State_fx *st_fx,       /* i/o: decoder static memory       */
    const Word16 L_frame,          /* i  : length of the frame         */
    const Word16 sharpFlag,        /* i  : flag for formant sharpening */
    const Word16 *Aq,              /* i  : LP filter coefficient       */
    const Word16 coder_type,       /* i  : coding type                 */
    Word16 *exc,             /* i/o: adapt. excitation exc       */
    const Word32  core_brate,      /* i  : switching frame bit-rate    */
    Word16 *Q_exc
)
{
    Word16 T0, T0_frac, T0_min, T0_max;/* integer pitch variables                          */
    Word16 gain_pit,gain_code16;   /* pitch gain                                           */
    Word32 gain_code,L_tmp;        /* gain/normalized gain of the algebraic excitation     */
    Word32 norm_gain_code;         /* normalized gain of the algebraic excitation          */
    Word16 gain_inov;              /* Innovation gain                                      */
    Word16 voice_fac;              /* voicing factor                                       */
    Word16 code[L_SUBFR];          /* algebraic codevector                                 */
    Word16 pitch;                  /* pointer to floating pitch                            */
    Word16 i;                      /* tmp variables                                        */
    Word16 pitch_limit_flag;
    Word16 *pt1;

    /*----------------------------------------------------------------------*
    * initializations
    *----------------------------------------------------------------------*/

    IF( sub(L_frame, L_FRAME) == 0 )
    {
        T0_max = PIT_MAX;
        move16();
        T0_min = PIT_MIN;
        move16();
    }
    ELSE /* L_frame == L_FRAME16k */
    {
        T0_max = PIT16k_MAX;
        move16();
        T0_min = PIT16k_MIN;
        move16();
    }

    /*----------------------------------------------------------------------*
    * Decode pitch lag
    *----------------------------------------------------------------------*/

    pitch = pit_decode_fx( st_fx, core_brate, 0, L_frame, 0, coder_type, &pitch_limit_flag, &T0, &T0_frac, &T0_min, &T0_max, L_SUBFR ); /*Q6*/

    /*--------------------------------------------------------------*
    * Find the adaptive codebook vector.
    *--------------------------------------------------------------*/

    pred_lt4( &exc[0], &exc[0], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );

    /*--------------------------------------------------------------*
    * LP filtering of the adaptive excitation
    *--------------------------------------------------------------*/

    lp_filt_exc_dec_fx( st_fx, MODE1, core_brate,0, coder_type, 0, L_SUBFR, L_frame, 0, exc); /*Scaling of exc doesn't change*/

    /*--------------------------------------------------------------*
    * Innovation decoding
    *--------------------------------------------------------------*/

    inov_decode_fx( st_fx, core_brate, 0, L_frame, coder_type, sharpFlag, 0, -1, Aq, st_fx->tilt_code_fx, pitch, code); /*code in Q9*/

    /*--------------------------------------------------------------*
    * Gain decoding
    * Estimate spectrum tilt and voicing
    *--------------------------------------------------------------*/

    IF( sub(L_frame, L_FRAME) == 0 )
    {
        gain_dec_mless_fx( st_fx, core_brate, L_frame, TRANSITION, 0, -1, code, st_fx->old_Es_pred_fx, &gain_pit, &gain_code, &gain_inov, &norm_gain_code );
    }
    ELSE
    {
        gain_dec_mless_fx( st_fx, core_brate, L_frame, coder_type, 0, -1, code, st_fx->old_Es_pred_fx, &gain_pit, &gain_code, &gain_inov, &norm_gain_code );
    }

    /* _ (Word16*) gain_pit : quantized pitch gain (Q14)									*/
    /* _ (Word32*) gain_code : quantized codebook gain (Q16)								*/
    /* _ (Word16*) gain_inov : gain of the innovation (used for normalization) (Q12)		*/
    /* _ (Word32*) norm_gain_code : norm. gain of the codebook excitation (Q16)			*/
    st_fx->tilt_code_fx = est_tilt_fx( exc, gain_pit, code, gain_code, &voice_fac, *Q_exc ); /*Q15*/

    /*----------------------------------------------------------------------*
    * Find the total excitation
    *----------------------------------------------------------------------*/

    /* Rescaling for 12.8k core */
    IF ( sub(L_frame,L_FRAME) == 0 )
    {
        Rescale_exc( NULL, &exc[0], NULL, st_fx->last_exc_dct_in_fx, L_SUBFR, L_SUBFR * HIBND_ACB_L_FAC, gain_code, &(st_fx->Q_exc), st_fx->Q_subfr, NULL, 0, GENERIC );
    }
    /* Rescaling for 16k core */
    ELSE
    {
        Rescale_exc(NULL, &exc[0], NULL, st_fx->last_exc_dct_in_fx, L_SUBFR, L_SUBFR * 2, gain_code, &(st_fx->Q_exc), st_fx->Q_subfr, NULL, 0, GENERIC);
    }

    gain_code16 = round_fx(L_shl(gain_code,st_fx->Q_exc)); /*Q_exc*/

    FOR (i = 0; i < L_SUBFR; i++)
    {
        L_tmp = L_shl(L_mult(gain_pit, exc[i]), 1); /*Q16+Q_exc*/
        /*exc2_fx[i+i_subfr] = round_fx(L_tmp);*/ /*Q_exc*/
        L_tmp = L_add(L_tmp, L_shl(L_mult(gain_code16, code[i]), 6)); /*Q16+Q_exc*/
        exc[i] = round_fx(L_tmp); /*Q_exc*/
    }

    /*-----------------------------------------------------------------*
    * long term prediction on the 2nd sub frame
    *-----------------------------------------------------------------*/

    pred_lt4(&exc[L_SUBFR], &exc[L_SUBFR], T0, T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
    pt1 = exc+L_SUBFR;

    FOR( i = 0; i < L_SUBFR;  i++ )
    {
        (*pt1) = round_fx(L_shl(L_mult(*pt1, gain_pit),1)); /*Q_exc + Q14 +1 +1 -16*/
        pt1++;
    }

    return;
}
