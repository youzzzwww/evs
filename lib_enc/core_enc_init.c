/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "basop_mpy.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"
#include "count.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include <string.h>


/*-----------------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------------*/

static void init_tcx( Encoder_State_fx *st, Word16 L_frame_old );
static void init_core_sig_ana( Encoder_State_fx *st );
static void init_acelp( Encoder_State_fx *st, Word16 L_frame_old, const Word16 shift);
static void init_modes( Encoder_State_fx *st );

/*-----------------------------------------------------------------------*
 * init_coder_ace_plus()
 *
 * Initialization of state variables
 *-----------------------------------------------------------------------*/

void init_coder_ace_plus(Encoder_State_fx *st, const Word16 shift)
{
    Word16 L_frame_old; /*keep old frame size for switching */
    Word16 L_subfr;


    /* Bitrate */
    st->tcxonly = getTcxonly(st->total_brate_fx);
    move16();

    /* Core Sampling Rate */
    st->sr_core = getCoreSamplerateMode2(st->total_brate_fx, st->bwidth_fx, st->rf_mode );
    st->fscale  = sr2fscale(st->sr_core);
    move16();

    /* Narrowband? */
    st->narrowBand = 0;
    move16();
    if( sub(st->bwidth_fx, NB) == 0 )
    {
        st->narrowBand = 1;
        move16();
    }

    /* Core Framing */
    L_frame_old = st->last_L_frame_fx;
    move16();
    st->L_frame_fx = extract_l(Mult_32_16(st->sr_core , 0x0290));
    st->L_frame_past = -1;
    move16();

    st->L_frameTCX = extract_l(Mult_32_16(st->input_Fs_fx , 0x0290));

    st->nb_subfr = NB_SUBFR;
    move16();
    L_subfr  = shr(st->L_frame_fx, 2);
    test();
    IF ( sub(st->L_frame_fx, L_FRAME16k) == 0 && L_sub(st->total_brate_fx, 32000) <= 0 )
    {
        st->nb_subfr = NB_SUBFR16k;
        move16();
        L_subfr  = mult(st->L_frame_fx, 0x199A);  /*1/5 = 0x199A Q15*/                                              move16();
    }

    /* Core Lookahead */
    st->encoderLookahead_enc = NS2SA_fx2(st->sr_core, ACELP_LOOK_NS);
    move16();
    st->encoderLookahead_FB =  NS2SA_fx2(st->input_Fs_fx, ACELP_LOOK_NS);
    move16();

    IF ( st->ini_frame_fx == 0 )
    {
        st->acelpFramesCount = 0;
        move16();
        st->prevTempFlatness_fx = FL2WORD16_SCALE(1.0f, AVG_FLAT_E);
        move16();
    }

    /* Initialize TBE */
    st->prev_coder_type_fx = GENERIC;
    move16();
    set16_fx(st->prev_lsf_diff_fx, 16384, LPC_SHB_ORDER);
    st->prev_tilt_para_fx = 0;
    move16();
    set16_fx( st->cur_sub_Aq_fx, 0, M+1 );

    st->currEnergyHF_fx = 0;
    move16();
    st->currEnergyHF_e_fx = 0;
    move16();
    st->energyCoreLookahead_Fx = 0;
    move16();

    /* Initialize LPC analysis/quantization */
    st->lpcQuantization = 0;
    move16();
    test();
    if( L_sub(st->sr_core,16000) <= 0 && st->tcxonly == 0 )
    {
        st->lpcQuantization = 1;
        move16();
    }


    st->next_force_safety_net_fx = 0;
    move16();
    test();
    test();
    IF ( sub(st->last_L_frame_fx,st->L_frame_fx) != 0 || sub(st->last_core_fx,AMR_WB_CORE) == 0 || sub(st->last_core_fx,HQ_CORE) == 0 )
    {
        set16_fx( st->mem_MA_fx, 0, M );
        Copy(GEWB_Ave_fx, st->mem_AR_fx, M);
    }

    /* Initialize IGF */
    st->hIGFEnc.infoStopFrequency = -1;
    move16();
    IF( st->igf )
    {
        IGFEncSetMode(&st->hIGFEnc, st->total_brate_fx, st->bwidth_fx, st->rf_mode);
    }

    /* Initialize TCX */
    init_tcx( st, L_frame_old );

    /* Initialize Core Signal Analysis Module */
    init_core_sig_ana( st );

    /* Initialize Signal Buffers */
    init_sig_buffers( st, L_frame_old, L_subfr );

    /* Initialize ACELP */
    init_acelp( st, L_frame_old , shift);

    if(st->ini_frame_fx == 0)
    {
        st->tec_tfa = 0;
        move16();
    }
    IF(st->tec_tfa == 0)
    {
        resetTecEnc_Fx(&st->tecEnc, 0);
    }
    ELSE
    {
        resetTecEnc_Fx(&st->tecEnc, 1);
    }
    st->tec_tfa = 0;
    move16();
    test();
    test();
    if( sub(st->bwidth_fx, SWB) == 0 && (L_sub(st->total_brate_fx, ACELP_16k40) == 0 || L_sub(st->total_brate_fx, ACELP_24k40) == 0) )
    {
        st->tec_tfa = 1;
        move16();
    }

    st->tec_flag = 0;
    move16();
    st->tfa_flag = 0;
    move16();
    /* Initialize DTX */
    IF( st->ini_frame_fx == 0 )
    {
        /*VAD B process init*/
        vad_init(&st->vad_st);
    }

    st->glr = 0;
    move16();

    test();
    test();
    test();
    if( (L_sub(st->total_brate_fx, ACELP_9k60)==0)||( L_sub(st->total_brate_fx, ACELP_16k40)==0)||
            (L_sub(st->total_brate_fx, ACELP_24k40)==0)||(L_sub(st->total_brate_fx, ACELP_32k)==0))
    {
        st->glr = 1;
        move16();
    }

    st->glr_reset = 0;
    move16();

    /* Initialize ACELP/TCX Modes */
    init_modes( st );

    /* Init I/O */


    /* Adaptive BPF */
    set16_fx(st->mem_bpf.noise_buf, 0, 2*L_FILT16k);
    set16_fx(st->mem_bpf.error_buf, 0, L_FILT16k);
    set16_fx(st->bpf_gainT, 0, NB_SUBFR16k);

    set16_fx(st->bpf_T, PIT_MIN_12k8, NB_SUBFR16k);

    st->mem_bpf.noise_shift_old = 0;
    move16();

    IF ( st->ini_frame_fx == 0 )
    {
        st->Q_max_enc[0] = 15;
        move16();
        st->Q_max_enc[1] = 15;
        move16();
    }

    st->enablePlcWaveadjust = 0;
    move16();
    if (L_sub(st->total_brate_fx, 48000) >= 0)
    {
        st->enablePlcWaveadjust = 1;
        move16();
    }

    open_PLC_ENC_EVS( &st->plcExt, st->sr_core );

    st->glr_idx[0] = 0;
    move16();
    st->glr_idx[1] = 0;
    move16();
    move16();
    move16(); /* casts */
    st->mean_gc[0] = L_deposit_h(0);
    st->mean_gc[1] = L_deposit_h(0);
    st->prev_lsf4_mean = 0;
    move16();

    st->last_stab_fac = 0;
    move16();


    return;
}

static void init_tcx( Encoder_State_fx *st, Word16 L_frame_old )
{
    Word16 i;
    Word16 fscaleFB;


    fscaleFB = div_l(L_shl(st->input_Fs_fx, LD_FSCALE_DENOM+1), 12800);

    init_TCX_config(&st->tcx_cfg, st->L_frame_fx, st->fscale, st->L_frameTCX, fscaleFB );

    st->tcx_cfg.tcx_mdct_window_length_old = st->tcx_cfg.tcx_mdct_window_length;
    move16();

    /* TCX Offset */
    st->tcx_cfg.tcx_offset = shr(st->tcx_cfg.tcx_mdct_window_delay, 1);
    move16();
    st->tcx_cfg.tcx_offsetFB = shr(st->tcx_cfg.tcx_mdct_window_delayFB, 1);
    move16();

    /*<0 rectangular transition with optimized window size = L_frame+L_frame/4*/
    st->tcx_cfg.lfacNext = sub(st->tcx_cfg.tcx_offset, shr(st->L_frame_fx, 2));
    move16();
    st->tcx_cfg.lfacNextFB = sub(st->tcx_cfg.tcx_offsetFB, shr(st->L_frameTCX, 2));

    IF ( st->ini_frame_fx == 0 )
    {
        st->tcx_cfg.tcx_curr_overlap_mode = st->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;
        move16();
        move16();
    }

    /* Init TCX target bits correction factor */
    st->LPDmem.tcx_target_bits_fac = 0x4000;     /*1.0f in 1Q14*/                                                     move16();
    st->measuredBwRatio  = 0x4000;               /*1.0f in 1Q14*/                                                     move16();
    st->noiseTiltFactor  = 9216;                 /*0.5625f in 1Q14*/                                                  move16();
    st->noiseLevelMemory = 0;
    move16();
    /*SQ deadzone & memory quantization*/

    /*0.375f: deadzone of 1.25->rounding=1-1.25/2 (No deadzone=0.5)*/
    st->tcx_cfg.sq_rounding = FL2WORD16(0.375f);
    move16();

    FOR (i = 0; i < L_FRAME_PLUS; i++)
    {
        st->memQuantZeros[i] = 0;
        move16();
    }

    /* TCX rate loop */
    st->tcx_cfg.tcxRateLoopOpt = 0;
    move16();

    if ( st->tcxonly != 0 )
    {
        st->tcx_cfg.tcxRateLoopOpt = 2;
        move16();
    }

    /* TCX bandwidth */
    move16();
    st->tcx_cfg.bandwidth = getTcxBandwidth(st->bwidth_fx);


    /* set number of coded lines */
    st->tcx_cfg.tcx_coded_lines = getNumTcxCodedLines(st->bwidth_fx);

    /* TNS in TCX */
    move16();
    move16();
    st->tcx_cfg.fIsTNSAllowed = getTnsAllowed(st->total_brate_fx
                                ,st->igf
                                             );
    st->tcx_cfg.pCurrentTnsConfig = NULL;

    IF ( st->tcx_cfg.fIsTNSAllowed != 0 )
    {
        InitTnsConfigs(bwMode2fs[st->bwidth_fx], st->tcx_cfg.tcx_coded_lines, st->tcx_cfg.tnsConfig, st->hIGFEnc.infoStopFrequency, st->total_brate_fx);
    }

    /* TCX-LTP */
    st->tcxltp = getTcxLtp(st->total_brate_fx, st->sr_core, 0);
    move16();

    test();
    test();
    test();
    test();
    IF( st->ini_frame_fx == 0 )
    {

        st->tcxltp_pitch_int_past = st->L_frame_fx;
        move16();
        st->tcxltp_pitch_fr_past = 0;
        move16();
        st->tcxltp_gain_past = 0;
        move16();
        st->tcxltp_norm_corr_past = 0;
        move16();
    }
    ELSE IF ( sub(st->L_frame_fx,L_frame_old) != 0 && !((st->total_brate_fx==16400||st->total_brate_fx==24400)&&(st->total_brate_fx==st->last_total_brate_fx)&&(st->last_bwidth_fx==st->bwidth_fx)) )
    {
        Word16	pitres, pitres_old;
        Word16	pit, pit_old;

        pitres_old = 4;
        move16();
        if (sub(160,shr(L_frame_old,sub(7,norm_s(L_frame_old)))) == 0)		/*if ( L_frame_old%160==0 )*/
        {
            pitres_old = 6;
            move16();
        }

        /*pit_old = (float)st->tcxltp_pitch_int_past + (float)st->tcxltp_pitch_fr_past/(float)pitres_old;*/
        pit_old = add(st->tcxltp_pitch_int_past, mult_r(st->tcxltp_pitch_fr_past, div_s(1,pitres_old)));

        pitres = 4;
        move16();
        if (sub(160,shr(st->L_frame_fx,sub(7,norm_s(st->L_frame_fx)))) == 0)		/*if ( st->L_frame_fx%160==0 )*/
        {
            pitres = 6;
            move16();
        }

        /*pit = pit_old * (float)st->L_frame_fx/(float)L_frame_old;*/
        pit = shl(mult_r(pit_old, div_s(st->L_frame_fx, shl(L_frame_old, 2))), 2);
        /*    assert(pit <= st->L_frame_fx);*/

        st->tcxltp_pitch_int_past = pit;
        move16();
        move16();
        st->tcxltp_pitch_fr_past = i_mult2(sub(pit,st->tcxltp_pitch_int_past),pitres);
        move16();
    }

    /* Residual Coding*/
    st->tcx_cfg.resq = getResq(st->total_brate_fx);
    move16();

    test();
    if ( st->tcx_cfg.resq != 0 && st->tcxonly == 0)
    {
        st->tcx_cfg.tcxRateLoopOpt = 1;
        move16();
    }

    st->tcx_cfg.ctx_hm = getCtxHm( st->total_brate_fx, st->rf_mode );

    st->tcx_lpc_shaped_ari = getTcxLpcShapedAri(
                                 st->total_brate_fx,
                                 st->bwidth_fx
                                 ,st->rf_mode
                             );

}

void init_sig_buffers( Encoder_State_fx *st, const Word16 L_frame_old, const Word16 L_subfr )
{

    /* Encoder Past Samples at encoder-sampling-rate */
    st->encoderPastSamples_enc = shr(imult1616(st->L_frame_fx, 9), 4);

    /* Initialize Signal Buffers and Pointers at encoder-sampling-rate */
    IF ( st->ini_frame_fx == 0 )
    {
        set16_fx(st->buf_speech_enc, 0, L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k);
        set16_fx(st->buf_speech_enc_pe, 0, L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k);
        set16_fx(st->buf_speech_ltp, 0, L_PAST_MAX_32k+L_FRAME32k+L_NEXT_MAX_32k);
        set16_fx(st->buf_wspeech_enc, 0, L_FRAME16k+L_SUBFR+L_FRAME16k+L_NEXT_MAX_16k);
    }
    ELSE
    {
        test();
        test();
        test();
        test();
        test();
        IF ( sub(st->L_frame_fx,L_frame_old) != 0 && !((L_sub(st->total_brate_fx,ACELP_16k40)==0||L_sub(st->total_brate_fx,ACELP_24k40)==0)&&(L_sub(st->total_brate_fx,st->last_total_brate_fx)==0)&&(sub(st->last_bwidth_fx,st->bwidth_fx)==0)) )
        {
            lerp( st->new_speech_enc-L_frame_old, st->buf_speech_enc, st->L_frame_fx, L_frame_old );
            test();
            IF( sub(st->last_core_fx,TCX_20_CORE) != 0 && sub(st->last_core_fx,TCX_10_CORE) != 0 )   /* condition should be checked again */
            {
                Copy( st->buf_speech_enc, st->buf_speech_ltp, st->L_frame_fx );
            }

            Copy_Scale_sig( st->old_wsp_fx, st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-L_WSP_MEM,L_WSP_MEM, sub(st->prev_Q_new, st->prev_Q_old));

            /*Resamp buffers needed only for ACELP*/
            test();
            test();
            IF( sub(st->L_frame_fx,L_FRAME) == 0 && !st->tcxonly )
            {
                Copy_Scale_sig( st->old_inp_12k8_fx, st->buf_speech_enc_pe+st->L_frame_fx-L_INP_MEM,L_INP_MEM, sub(st->prev_Q_new, st->prev_Q_old));

            }
            ELSE IF( sub(st->L_frame_fx,L_FRAME16k) == 0 && !st->tcxonly )
            {
                lerp( st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-L_WSP_MEM, st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-310, 310, L_WSP_MEM );
                Copy( st->old_inp_16k_fx, st->buf_speech_enc_pe+st->L_frame_fx-L_INP_MEM,L_INP_MEM);

            }

            st->mem_preemph_enc = st->buf_speech_enc[st->encoderPastSamples_enc+st->encoderLookahead_enc-1];
            move16();
            st->mem_wsp_enc = st->buf_wspeech_enc[st->L_frame_fx+L_SUBFR-1];
            move16();
        }
        /*coming from TCXonly modes*/
        ELSE IF( !st->tcxonly && L_sub(st->last_total_brate_fx,ACELP_32k)>=0)
        {

            Copy_Scale_sig( st->old_wsp_fx, st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-L_WSP_MEM,L_WSP_MEM, sub(st->prev_Q_new, st->prev_Q_old));

            /*Resamp buffers needed only for ACELP*/
            IF( sub(st->L_frame_fx,L_FRAME16k) == 0)
            {
                lerp( st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-L_WSP_MEM, st->buf_wspeech_enc+st->L_frame_fx + L_SUBFR-310, 310, L_WSP_MEM );
            }
            st->LPDmem.mem_w0 = 0;
            move16();
            st->mem_wsp_enc = st->buf_wspeech_enc[st->L_frame_fx+L_SUBFR-1];
            move16();
        }
    }

    st->new_speech_enc      = st->buf_speech_enc      + st->encoderPastSamples_enc    + st->encoderLookahead_enc;
    st->new_speech_enc_pe   = st->buf_speech_enc_pe   + st->encoderPastSamples_enc    + st->encoderLookahead_enc;
    st->new_speech_ltp      = st->buf_speech_ltp      + st->encoderPastSamples_enc    + st->encoderLookahead_enc;
    st->new_speech_TCX      = st->input_buff + L_FRAME48k + NS2SA(48000, DELAY_FIR_RESAMPL_NS) - NS2SA(st->input_Fs_fx, DELAY_FIR_RESAMPL_NS);

    st->speech_enc          = st->buf_speech_enc      + st->encoderPastSamples_enc;
    st->speech_enc_pe       = st->buf_speech_enc_pe   + st->encoderPastSamples_enc;
    st->speech_ltp          = st->buf_speech_ltp      + st->encoderPastSamples_enc;
    st->speech_TCX          = st->new_speech_TCX      - st->encoderLookahead_FB;

    st->wspeech_enc         = st->buf_wspeech_enc     + st->L_frame_fx + L_subfr;

    test();
    test();
    IF( st->ini_frame_fx == 0 || sub(st->L_frame_fx,L_frame_old) != 0 || sub(st->last_codec_mode,MODE1) == 0 )
    {
        set16_fx(st->buf_synth, 0, OLD_SYNTH_SIZE_ENC+L_FRAME_MAX);
    }

    st->synth               = st->buf_synth           + st->L_frame_fx + L_subfr;


    return;
}

static void init_core_sig_ana( Encoder_State_fx *st )
{


    /* Pre-emphasis factor and memory */

    st->preemph_fac = PREEMPH_FAC_SWB; /*SWB*/                                                    move16();
    IF ( sub(st->fscale, (16000*FSCALE_DENOM)/12800) < 0 )
    {
        st->preemph_fac = PREEMPH_FAC; /*WB*/                                                       move16();
    }
    ELSE IF ( sub(st->fscale, (24000*FSCALE_DENOM)/12800) < 0 )
    {
        st->preemph_fac = PREEMPH_FAC_16k; /*WB*/                                                   move16();
    }

    st->tcx_cfg.preemph_fac=st->preemph_fac;
    move16();

    st->gamma = GAMMA1;
    move16();
    st->inv_gamma = GAMMA1_INV;
    move16();
    IF ( L_sub(st->sr_core, 16000) == 0 )
    {
        st->gamma = GAMMA16k;
        move16();
        st->inv_gamma = GAMMA16k_INV;
        move16();
    }


    st->min_band_fx = 1;
    move16();
    st->max_band_fx = 16;
    move16();

    IF ( st->narrowBand == 0)
    {
        st->min_band_fx = 0;
        move16();
        st->max_band_fx = 19;
        move16();
    }


    return;
}

static void init_acelp( Encoder_State_fx *st, Word16 L_frame_old , const Word16 shift)
{
    Word16 mem_syn_r_size_old;
    Word16 mem_syn_r_size_new;


    /* Init pitch lag */
    st->pit_res_max = initPitchLagParameters(st->sr_core, &st->pit_min, &st->pit_fr1, &st->pit_fr1b, &st->pit_fr2, &st->pit_max);


    /* Init LPDmem */
    IF( st->ini_frame_fx == 0 )
    {
        set16_fx( st->LPDmem.syn, 0, 1+M );

        set16_fx( st->LPDmem.Txnq, 0, 4*L_MDCT_OVLP_MAX);
        st->LPDmem.acelp_zir = st->LPDmem.Txnq + shr(st->L_frame_fx,1);
        set16_fx( st->LPDmem.mem_syn_r, 0, L_SYN_MEM );
    }
    ELSE /*Rate switching*/
    {
        IF( sub(st->last_core_fx,ACELP_CORE) == 0 )
        {
            lerp( st->LPDmem.Txnq,st->LPDmem.Txnq, st->L_frame_fx, L_frame_old );
        }
        ELSE
        {
            lerp( st->LPDmem.Txnq,st->LPDmem.Txnq, st->tcx_cfg.tcx_mdct_window_length, st->tcx_cfg.tcx_mdct_window_length_old );
        }
        st->LPDmem.acelp_zir = st->LPDmem.Txnq + shr(st->L_frame_fx,1);

        /* Rate switching */
        IF( sub(st->last_codec_mode,MODE1) == 0 )
        {
            Copy( st->mem_syn1_fx, st->LPDmem.mem_syn2, M );
            set16_fx( st->LPDmem.Txnq, 0, 4*L_MDCT_OVLP_MAX );
            set16_fx( st->LPDmem.syn, 0, M );
        }

        /*AMR-WBIO->MODE2*/
        IF( sub(st->last_core_fx,AMR_WB_CORE) == 0 )
        {
            st->next_force_safety_net_fx=1;
            move16();
            st->last_core_fx = ACELP_CORE;
            move16();
        }
        /*HQ-CORE->MODE2*/
        test();
        IF( sub(st->last_codec_mode,MODE1)==0 && sub(st->last_core_fx,HQ_CORE) == 0 )
        {
            /*Reset of ACELP memories*/
            st->next_force_safety_net_fx=1;
            move16();
            st->rate_switching_reset = 1;
            move16();
            st->LPDmem.tilt_code = TILT_CODE;
            move16();
            set16_fx( st->LPDmem.old_exc, 0, L_EXC_MEM );
            set16_fx( st->LPDmem.syn, 0, 1+M );
            st->LPDmem.mem_w0 = 0;
            move16();
            set16_fx( st->LPDmem.mem_syn, 0, M );
            set16_fx( st->LPDmem.mem_syn2, 0, M );

            /* unquantized LPC*/
            test();
            IF ( !((L_sub(st->total_brate_fx,ACELP_16k40)==0||L_sub(st->total_brate_fx,ACELP_24k40)==0)&&(L_sub(st->total_brate_fx,st->last_total_brate_fx)==0)&&(sub(st->last_bwidth_fx,st->bwidth_fx)==0)) )
            {
                Copy( st->lsp_old1_fx, st->lspold_enc_fx, M ); /*lsp old @12.8kHz*/
                IF( sub(st->L_frame_fx,L_FRAME16k) == 0 )
                {
                    lsp_convert_poly_fx( st->lspold_enc_fx, st->L_frame_fx, 0 );
                }
            }
            Copy( st->lspold_enc_fx, st->lsp_old_fx, M ); /*used unquantized values for mid-LSF Q*/
            if(st->tcxonly==0)
            {
                lsp2lsf_fx( st->lsp_old_fx, st->lsf_old_fx, M, st->sr_core );
            }
            else
            {
                E_LPC_lsp_lsf_conversion( st->lsp_old_fx, st->lsf_old_fx, M );
            }

            st->last_core_fx = TCX_20_CORE;
            move16();

            st->tcx_cfg.last_aldo=1;  /*It was previously ALDO*/
            st->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
            /*ALDO overlap windowed past: also used in MODE1 but for other MDCT-FB*/
            set16_fx( st->old_out_fx, 0, st->L_frame_fx );
        }
        ELSE
        {
            test();
            test();
            IF( (sub(st->L_frame_fx,L_frame_old) != 0) &&  (sub(st->L_frame_fx,L_FRAME16k) <= 0) && (sub(L_frame_old,L_FRAME16k) <= 0) )
            {
                /* convert quantized LSP vector */
                st->rate_switching_reset=lsp_convert_poly_fx( st->lsp_old_fx, st->L_frame_fx, 0 );
                if(st->tcxonly==0)
                {
                    lsp2lsf_fx( st->lsp_old_fx, st->lsf_old_fx, M, st->sr_core );
                }
                else
                {
                    E_LPC_lsp_lsf_conversion( st->lsp_old_fx, st->lsf_old_fx, M );
                }
                IF( sub(st->L_frame_fx,L_FRAME16k)==0 )
                {
                    Copy( st->lsp_old_fx, st->lspold_enc_fx, M );
                }
                ELSE
                {
                    Copy( st->lsp_old1_fx, st->lspold_enc_fx, M );
                }

                synth_mem_updt2( st->L_frame_fx, st->last_L_frame_fx, st->LPDmem.old_exc, st->LPDmem.mem_syn_r, st->LPDmem.mem_syn2, st->LPDmem.mem_syn, ENC );

                /*Mem of deemphasis stay unchanged : st->LPDmem.syn*/
                {
                    Word16 tmp, A[M+1], Ap[M+1],tmp_buf[M+1];
                    /* Update wsyn */
                    /* lsp2a_stab( st->lsp_old, A, M ); */
                    E_LPC_f_lsp_a_conversion(st->lsp_old_fx, A, M);
                    weight_a_fx( A, Ap, GAMMA1, M );
                    tmp=0;
                    move16();
                    tmp_buf[0]=0;
                    move16();
                    Copy( st->LPDmem.mem_syn2, tmp_buf+1, M );
                    deemph_fx( tmp_buf+1, st->preemph_fac, M, &tmp );
                    Residu3_fx( Ap, tmp_buf+M, &tmp, 1, 1 );
                    st->LPDmem.mem_w0 = sub(shr(st->wspeech_enc[-1],shift), tmp);
                }
            }
            ELSE IF((sub(st->L_frame_fx,L_frame_old) != 0))
            {
                /*Partial reset of ACELP memories*/
                st->next_force_safety_net_fx=1;
                move16();
                st->rate_switching_reset = 1;
                move16();

                /*reset partly some memories*/
                st->LPDmem.tilt_code = TILT_CODE;
                move16();
                set16_fx( st->LPDmem.old_exc, 0, L_EXC_MEM );
                move16();

                /*Resamp others memories*/
                /*Size of LPC syn memory*/
                /* 1.25/20.0 = 1.0/16.0 -> shift 4 to the right. */
                mem_syn_r_size_old = shr(L_frame_old, 4);
                mem_syn_r_size_new = shr(st->L_frame_fx, 4);

                lerp( st->LPDmem.mem_syn_r+L_SYN_MEM-mem_syn_r_size_old, st->LPDmem.mem_syn_r+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
                Copy( st->LPDmem.mem_syn_r+L_SYN_MEM-M, st->LPDmem.mem_syn, M);
                Copy( st->LPDmem.mem_syn, st->LPDmem.mem_syn2, M );

                /*Untouched memories : st->LPDmem.syn & st->LPDmem.mem_w0*/
                st->LPDmem.mem_w0 = 0;
                move16();

                /* unquantized LPC*/
                Copy( st->lsp_old1_fx, st->lspold_enc_fx, M ); /*lsp old @12.8kHz*/
                IF( sub(st->L_frame_fx,L_FRAME16k)==0 )
                {
                    lsp_convert_poly_fx( st->lspold_enc_fx, st->L_frame_fx, 0 );
                }
                Copy( st->lspold_enc_fx, st->lsp_old_fx, M ); /*used unquantized values for mid-LSF Q*/
                if(st->tcxonly==0)
                {
                    lsp2lsf_fx( st->lsp_old_fx, st->lsf_old_fx, M, st->sr_core );
                }
                else
                {
                    E_LPC_lsp_lsf_conversion( st->lsp_old_fx, st->lsf_old_fx, M );
                }
            }
        }
    }

    test();
    test();
    if(sub(st->last_bwidth_fx,NB)==0 && sub(st->bwidth_fx,NB)!=0 && st->ini_frame_fx!=0)
    {
        st->rate_switching_reset=1;
        move16();
    }

    /* Post-processing */
    st->dm_fx.prev_gain_code = L_deposit_l(0);
    set16_fx(st->dm_fx.prev_gain_pit, 0, 6);
    st->dm_fx.prev_state = 0;
    move16();
    st->LPDmem.gc_threshold = 0;
    move16();

    /* Pulse Search configuration */
    st->acelp_autocorr = 1;
    move16();

    /*Use for 12.8 kHz sampling rate and low bitrates, the conventional pulse search->better SNR*/
    if ((L_sub(st->total_brate_fx, ACELP_9k60) <= 0 || st->rf_mode != 0) && (L_sub(st->sr_core,12800) == 0))
    {
        st->acelp_autocorr = 0;
        move16();
    }


    /*BPF parameters for adjusting gain in function of background noise*/
    IF( sub(st->codec_mode,MODE2) == 0 )
    {
        st->mem_bpf.lp_error_ener = L_deposit_l(0);
        if( st->last_codec_mode == MODE1 )
        {
            st->mem_bpf.lp_error = L_deposit_l(0);
        }
    }


    return;
}

static void init_modes( Encoder_State_fx *st )
{
    Word8 n;
    Word32 tmp32;




    /* Restrict ACE/TCX20/TCX10 mode */
    move16();
    st->restrictedMode = getRestrictedMode(st->total_brate_fx, st->Opt_AMR_WB_fx);
    move16();
    st->acelpEnabled = 0;
    move16();
    st->tcx20Enabled = 0;
    move16();
    st->tcx10Enabled = 0;

    if (sub(s_and(st->restrictedMode,1),1) == 0)
    {
        st->acelpEnabled = 1;
        move16();
    }
    if (sub(s_and(st->restrictedMode,2),2) == 0)
    {
        st->tcx20Enabled = 1;
        move16();
    }
    if (sub(s_and(st->restrictedMode,4),4) == 0)
    {
        st->tcx10Enabled = 1;
        move16();
    }


    /* TCX mode (TCX20 TCX10_10 or NO_TCX) */
    st->tcxMode = NO_TCX;
    move16();

    /* Bits Budget */
    /*st->bits_frame_nominal = (int)( (float)st->L_frame_fx * (float)FSCALE_DENOM * (float)st->bitrate / ( (float)st->fscale * 12800.0f ) );*/
    /*st->bits_frame_nominal = (int)( (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->bitrate/100.0f + 0.49f );*/
    /*328 = 0.010009765625 in 0Q15*/
    /* st->bits_frame_nominal = extract_h(L_add(L_mult(div_l(L_mult(shl(st->L_frame_fx,2),st->bitrate),st->fscale),328),16056)); */

    /* st->bits_frame_nominal = (int)( (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->bitrate/100.0f + 0.49f ); */
    assert(FSCALE_DENOM == 512);
    assert(st->fscale == 2 * st->L_frame_fx); /* this assumption is true if operated in 20ms frames with FSCALE_DENOM == 512, which is the current default */
    tmp32 = L_shl(st->total_brate_fx, 1); /* (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->bitrate */
    st->bits_frame_nominal = extract_l(L_shr(Mpy_32_16_1(tmp32, 20972), 6)); /* 20972 = 0.01 * 64 * 32768 */
    assert(st->bits_frame_nominal == (int)( (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->total_brate_fx/100.0f + 0.49f ));

    IF (st->Opt_AMR_WB_fx)
    {
        st->bits_frame      = st->bits_frame_nominal;
        st->bits_frame_core = st->bits_frame_nominal;
    }
    ELSE
    {
        FOR (n=0; n<FRAME_SIZE_NB; n++)
        {
            IF (sub(FrameSizeConfig[n].frame_bits,st->bits_frame_nominal) == 0)
            {
                move16();
                move16();
                move16();
                st->frame_size_index = n;
                st->bits_frame = FrameSizeConfig[n].frame_bits;
                st->bits_frame_core = FrameSizeConfig[n].frame_net_bits;
                BREAK;
            }
        }
        if (n==FRAME_SIZE_NB)
        {
            assert(!"Bitrate not supported: not part of EVS");
        }
    }

    /* Reconfigure core */
    core_coder_reconfig( st );


    return;
}
