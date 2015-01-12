/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "options.h"
#include "stl.h"
#include "cnst_fx.h" /* for MIN_CNG_LEV */

/*-----------------------------------------------------------------------*
 * open_decoder_LPD()
 *
 * Initialization of state variables
 *-----------------------------------------------------------------------*/

void open_decoder_LPD( Decoder_State_fx *st, Word32 bit_rate, Word16 bandwidth_mode )
{
    Word16 i;
    Word16 mem_syn_r_size_new;
    Word16 mem_syn_r_size_old;
    Word16 fscaleFB;



    st->total_brate_fx = bit_rate;
    st->fscale_old     = st->fscale;
    st->sr_core     = getCoreSamplerateMode2(st->total_brate_fx, bandwidth_mode, st->rf_flag);
    st->fscale         = sr2fscale(st->sr_core);
    fscaleFB           = sr2fscale(st->output_Fs_fx);

    /* initializing variables for frame lengths etc. right in the beginning */
    st->L_frame_fx = extract_l(Mult_32_16(st->sr_core , 0x0290));
    st->L_frameTCX = extract_l(Mult_32_16(st->output_Fs_fx , 0x0290));

    IF (st->ini_frame_fx == 0)
    {
        st->last_L_frame_fx = st->L_frame_past = st->L_frame_fx;
        move16();
        move16();
        st->L_frameTCX_past = st->L_frameTCX;
        move16();
    }

    st->tcxonly = getTcxonly(st->total_brate_fx);
    move16();

    /* the TD TCX PLC in MODE1 still runs with 80ms subframes */
    st->nb_subfr = NB_SUBFR;
    move16();
    test();
    test();
    test();
    test();
    if ( (sub(st->L_frame_fx,L_FRAME16k) == 0 && L_sub(st->total_brate_fx,32000) <= 0 ) ||
            (st->tcxonly != 0 && (L_sub(st->sr_core,32000)==0 || L_sub(st->sr_core,16000)==0) )
       )
    {
        st->nb_subfr = NB_SUBFR16k;
        move16();
    }

    /* (float)st->L_frame/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->total_brate_fx */
    st->bits_frame = extract_l(L_shr(Mpy_32_16_1( L_shl(st->total_brate_fx, 1) , 20972), 6)); /* 20972 = 0.01 * 64 * 32768 */

    assert(FSCALE_DENOM == 512);
    assert(st->fscale == 2 * st->L_frame_fx); /* this assumption is true if operated in 20ms frames with FSCALE_DENOM == 512, which is the current default */
    assert(st->bits_frame == (int)( (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->total_brate_fx/100.0f + 0.49f ));

    st->TcxBandwidth   = getTcxBandwidth(bandwidth_mode);
    st->narrowBand  = 0;
    move16();
    if (sub(bandwidth_mode, NB) == 0)
    {
        st->narrowBand  = 1;
        move16();
    }

    st->pit_res_max     = initPitchLagParameters(st->sr_core, &st->pit_min, &st->pit_fr1, &st->pit_fr1b, &st->pit_fr2, &st->pit_max);
    IF ( st->ini_frame_fx == 0)
    {
        st->pit_res_max_past = st->pit_res_max;
    }
    i = mult_r(st->L_frameTCX, getInvFrameLen(st->L_frame_fx));
    st->pit_max_TCX = extract_l(L_shr(L_mult(st->pit_max, i), 7));
    st->pit_min_TCX = extract_l(L_shr(L_mult(st->pit_min, i), 7));

    /*Preemphasis param*/
    st->preemph_fac = PREEMPH_FAC_SWB; /*SWB*/                                    move16();
    IF ( sub(st->fscale, (16000*FSCALE_DENOM)/12800) < 0)
    {
        st->preemph_fac = PREEMPH_FAC; /*WB*/                                       move16();
    }
    ELSE if ( sub(st->fscale, (24000*FSCALE_DENOM)/12800) < 0)
    {
        st->preemph_fac = PREEMPH_FAC_16k; /*WB*/                                   move16();
    }

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

    /* LPC quantization */
    st-> lpcQuantization = 0;
    move16();
    test();
    if( st->tcxonly == 0 && L_sub(st->sr_core, 16000) <= 0)
    {
        st->lpcQuantization = 1;
    }

    st->numlpc = 2;
    move16();
    if ( st->tcxonly==0 )
    {
        st->numlpc = 1;
        move16();
    }

    /* Initialize TBE */
    st->prev_coder_type_fx = GENERIC;
    set16_fx(st->prev_lsf_diff_fx,16384,LPC_SHB_ORDER);
    st->prev_tilt_para_fx = 0;
    set16_fx( st->cur_sub_Aq_fx, 0, M+1 );

    /*TCX config*/
    st->tcx_cfg.preemph_fac = st->preemph_fac;
    move16();
    st->tcx_cfg.tcx_mdct_window_length_old = st->tcx_cfg.tcx_mdct_window_length;
    move16();

    init_TCX_config( &st->tcx_cfg, st->L_frame_fx, st->fscale, st->L_frameTCX, fscaleFB );

    IF (st->ini_frame_fx == 0)
    {
        st->tcx_cfg.tcx_last_overlap_mode = st->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
        move16();
        move16();
    }

    /* TCX Offset */
    st->tcx_cfg.tcx_offset   = shr(st->tcx_cfg.tcx_mdct_window_delay, 1);
    st->tcx_cfg.tcx_offsetFB = shr(st->tcx_cfg.tcx_mdct_window_delayFB, 1);

    /* Initialize FAC */
    st->tcx_cfg.lfacNext   =  sub(st->tcx_cfg.tcx_offset,shr(st->L_frame_fx,2));
    st->tcx_cfg.lfacNextFB =  sub(st->tcx_cfg.tcx_offsetFB,shr(st->L_frameTCX,2));

    /* set number of coded lines */
    st->tcx_cfg.tcx_coded_lines = getNumTcxCodedLines(st->bwidth_fx);

    /* TNS in TCX */
    st->tcx_cfg.pCurrentTnsConfig = NULL;
    st->tcx_cfg.fIsTNSAllowed = getTnsAllowed(st->total_brate_fx, st->igf);

    IF (st->tcx_cfg.fIsTNSAllowed != 0)
    {
        InitTnsConfigs(bwMode2fs[st->bwidth_fx], st->tcx_cfg.tcx_coded_lines, st->tcx_cfg.tnsConfig, st->hIGFDec.infoIGFStopFreq, st->total_brate_fx);
    }

    resetTecDec_Fx( &(st->tecDec_fx) );

    /* Initialize decoder delay */
    /*Constraint for adaptive BPF, otherwise parameter estimation and post-processing not time aligned*/
    if ( st->tcxonly==0 )
    {
        assert(0 == (st->tcx_cfg.lfacNext>0 ? st->tcx_cfg.lfacNext : 0) );
    }

    IF (st->tcxonly == 0)
    {
        /* Init signal classifier */
        set16_fx( st->mem_syn_clas_estim_fx, 0, L_SYN_MEM_CLAS_ESTIM );
        st->classifier_last_good   = UNVOICED_CLAS;
        move16();
        st->classifier_Q_mem_syn   = 0 ;
        move16();
    }

    st->flag_cna = 0;
    move16();

    /* Static vectors to zero */
    IF (st->ini_frame_fx == 0)
    {

        st->last_is_cng = 0;
        move16();

        st->rate_switching_reset = 0;
        move16();
        set16_fx(st->old_syn_Overl, 0, L_FRAME32k/2);

        set16_fx(st->syn_Overl_TDAC, 0, L_FRAME32k/2);
        set16_fx(st->syn_OverlFB, 0, L_FRAME_MAX/2);
        set16_fx(st->syn_Overl_TDACFB, 0, L_FRAME_MAX/2);

        set16_fx(st->syn_Overl, 0, L_FRAME_MAX/2);

        set16_fx(st->old_synth, 0, OLD_SYNTH_SIZE_DEC);


        set16_fx(st->syn, 0, M+1);

        set16_fx(st->mem_syn_r, 0, L_SYN_MEM);

        mem_syn_r_size_old = 0;         /* just to avoid MSVC warnings */
        mem_syn_r_size_new = 0;         /* just to avoid MSVC warnings */

        st->con_tcx = 0;
    }
    ELSE
    {
        /* Reset old_synth in case of core sampling rate switching and codec switching*/
        test();
        IF( (sub(st->L_frame_fx, st->last_L_frame_fx) != 0) || (sub(st->last_codec_mode,MODE1)==0) )
        {
            set16_fx(st->old_synth, 0, OLD_SYNTH_SIZE_DEC);
        }

        /*Size of LPC syn memory*/
        /* 1.25/20.0 = 1.0/16.0 -> shift 4 to the right. */
        mem_syn_r_size_old = shr(st->last_L_frame_fx, 4);
        mem_syn_r_size_new = shr(st->L_frame_fx, 4);

        /*Reset LPC mem*/
        test();
        test();
        IF( (sub(st->L_frame_fx,st->last_L_frame_fx)!=0) || (sub(st->last_core_fx,AMR_WB_CORE)==0)
        || sub(st->last_core_fx,HQ_CORE) == 0 )
        {
            /*LPC quant. mem*/
            set16_fx(st->mem_MA_fx, 0, M);
            IF( L_sub(st->sr_core,16000) == 0 )
            {
                Copy( GEWB2_Ave_fx, st->mem_AR_fx, M );
            }
            ELSE
            {
                Copy( GEWB_Ave_fx, st->mem_AR_fx, M );
            }
        }

        /*Codec switching*/
        IF( sub(st->last_codec_mode,MODE1)==0 )
        {
            st->con_tcx = 0;
            /**/
            Copy( st->lsp_old_fx, st->lspold_uw, M );
            Copy( st->lsf_old_fx, st->lsfold_uw, M );
            set16_fx( st->syn, 0, M );
        }
        IF( sub(st->last_core_fx,AMR_WB_CORE)==0 )
        {
            st->last_core_fx = ACELP_CORE;
            move16();
            st->last_core_bfi = ACELP_CORE;
            move16();
        }

        /*Codec switching from ACELP-A */
        test();
        IF( sub(st->last_codec_mode, MODE1)==0 && sub(st->last_core_fx, ACELP_CORE)==0 )
        {
            st->last_core_bfi = ACELP_CORE;
            move16();

            /*PLC*/
            IF(st->prev_bfi_fx!=0)
            {
                PWord16  const *w;
                Word16 W1,W2,nz,delay_comp;

                W1 = st->tcx_cfg.tcx_mdct_window_lengthFB;
                move16();
                W2 = shr(st->tcx_cfg.tcx_mdct_window_lengthFB,1);
                w = st->tcx_cfg.tcx_mdct_windowFB; /*pointer - no need to instrument*/

                nz = NS2SA_fx2(st->output_Fs_fx, N_ZERO_MDCT_NS);
                delay_comp = NS2SA_fx2(st->output_Fs_fx, DELAY_CLDFB_NS); /*CLDFB delay*/

                Copy(st->fer_samples_fx+delay_comp, st->syn_OverlFB, shr(st->L_frameTCX,1));

                lerp(st->fer_samples_fx+delay_comp,st->syn_Overl,shr(st->L_frame_fx,1),shr(st->L_frameTCX,1)); /*Q0: ACELP(bfi)->TCX(rect)*/

                /*old_out needed for MODE1 routine and syn_Overl_TDAC for MODE2 routine*/
                st->Q_old_wtda=-1;
                set16_fx(st->old_out_fx, 0, nz);
                Copy_Scale_sig(st->fer_samples_fx+delay_comp, st->old_out_fx+nz,W1, st->Q_old_wtda); /*Q-1*/

                FOR (i=0; i < W2; i++)
                {
                    st->old_out_fx[i+nz] = round_fx(Mpy_32_16_1(L_mult(w[i].v.re,w[i].v.re),st->old_out_fx[i+nz]));
                }
                FOR ( ; i < W1; i++)
                {
                    st->old_out_fx[i+nz] = round_fx(Mpy_32_16_1(L_mult(w[W2-1-(i-W2)].v.im,w[W2-1-(i-W2)].v.im),st->old_out_fx[i+nz]));
                }
                set16_fx(&st->old_out_fx[W1+nz], 0, nz);

                lerp(st->old_out_fx, st->old_out_LB_fx, st->L_frame_fx, st->L_frameTCX);
                Copy(st->old_out_fx+nz,st->syn_Overl_TDACFB,shr(st->L_frameTCX,1));

                nz = NS2SA_fx2(st->sr_core, N_ZERO_MDCT_NS);
                Copy(st->old_out_LB_fx+nz,st->syn_Overl_TDAC,shr(st->L_frame_fx,1));
                st->Q_old_wtda_LB = st->Q_old_wtda;
            }
        }

        /* Rate switching */
        test();
        test();
        test();
        IF( sub(st->last_codec_mode,MODE1)==0 && sub(st->last_core_fx,HQ_CORE)==0 )
        {
            /*Codec switching from MDCT */

            /*Reset of ACELP memories*/
            move16();
            move16();
            st->rate_switching_reset=1;
            st->tilt_code_fx = TILT_CODE;
            set16_fx(st->old_exc_fx, 0, L_EXC_MEM_DEC);
            set16_fx(st->syn, 0, 1+M);
            set16_fx(st->mem_syn2_fx, 0, M);

            /*OLA -> zero */
            set16_fx(st->old_syn_Overl, 0, L_FRAME32k/2);  /*HQ-CORE(bfi)->TCX don't need it*/
            set16_fx(st->syn_Overl_TDAC, 0, L_FRAME32k/2);  /*HQ-CORE(bfi)->TCX don't need it*/
            set16_fx(st->syn_Overl_TDACFB, 0, L_FRAME_MAX/2); /*HQ-CORE(bfi)->TCX don't need it*/
            set16_fx(st->syn_Overl, 0, L_FRAME_MAX/2); /*HQ-CORE(bfi)->TCX don't need it*/

            Copy_Scale_sig(st->old_out_fx+NS2SA(st->output_Fs_fx, N_ZERO_MDCT_NS), st->syn_OverlFB, st->tcx_cfg.tcx_mdct_window_lengthFB, negate(add(st->Q_old_wtda, TCX_IMDCT_HEADROOM)));

            move16();
            move16();
            st->tcx_cfg.last_aldo=1;  /*It was previously ALDO*/
            st->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
            /*OLA for MDCT-LB always reset in codec switching cases*/
            set16_fx( st->old_out_LB_fx, 0, st->L_frame_fx );
            move16();
            st->last_core_bfi = TCX_20_CORE;

            st->pfstat.on=0;
            move16();
            /* reset CLDFB memories */
            cldfb_reset_memory( st->cldfbAna_fx );
            cldfb_reset_memory( st->cldfbBPF_fx );
            cldfb_reset_memory( st->cldfbSyn_fx );
        }
        ELSE IF( (sub(st->L_frame_fx,st->last_L_frame_fx)!=0) && (sub(st->L_frame_fx,L_FRAME16k)<=0) && (sub(st->last_L_frame_fx,L_FRAME16k)<=0)) /* Rate switching between 12.8 and 16 kHz*/
        {
            /*Interpolation of ACELP memories*/

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
            E_LPC_f_lsp_a_conversion(st->lsp_old_fx, st->old_Aq_12_8_fx, M);


            Copy( st->lsp_old_fx, st->lspold_uw, M );
            Copy( st->lsf_old_fx, st->lsfold_uw, M );

            synth_mem_updt2( st->L_frame_fx, st->last_L_frame_fx, st->old_exc_fx, st->mem_syn_r, st->mem_syn2_fx, NULL, DEC );

            /*mem of deemphasis stayed unchanged.*/
        }
        ELSE IF( sub(st->L_frame_fx,st->last_L_frame_fx)!=0 ) /* Rate switching involving TCX only modes */
        {
            /*Partial reset of ACELP memories*/
            st->rate_switching_reset = 1;

            /*reset partly some memories*/
            st->tilt_code_fx = TILT_CODE;
            set16_fx( st->old_exc_fx, 0, L_EXC_MEM_DEC );
            set16_fx( st->old_Aq_12_8_fx, 0, M+1 );

            /*Resamp others memories*/
            /*Size of LPC syn memory*/
            lerp( st->mem_syn_r+L_SYN_MEM-mem_syn_r_size_old, st->mem_syn_r+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            Copy( st->mem_syn_r+L_SYN_MEM-M, st->mem_syn2_fx, M );

            /*Untouched memories : st->syn */
        }
    }

    test();
    test();
    if(sub(st->last_bwidth_fx,NB)==0 && sub(st->bwidth_fx,NB)!=0 && st->ini_frame_fx!=0)
    {
        st->rate_switching_reset=1;
        move16();
    }

    st->old_synth_len = shl(st->L_frame_fx, 1);
    st->old_synth_lenFB = shl(st->L_frameTCX, 1);

    /* Formant postfilter */
    IF ( st->ini_frame_fx==0 )
    {
        /*do nothing*/
    }
    ELSE IF( sub(st->last_codec_mode,MODE2)==0)
    {
        IF (st->tcxonly==0)
        {
            IF ( st->pfstat.on!=0 )
            {
                lerp( st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
                lerp( st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            }
            ELSE
            {
                set16_fx( st->pfstat.mem_stp, 0,L_SYN_MEM );
                set16_fx( st->pfstat.mem_pf_in, 0,L_SYN_MEM );
                st->pfstat.reset = 1;
                st->pfstat.gain_prec = 16384;
                move16();
            }
        }
        ELSE IF ( st->pfstat.on!=0 )
        {
            lerp( st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            lerp( st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
        }
    }
    ELSE
    {
        /*codec switching*/

        /*reset post-filter except for Narrowband*/
        IF ( L_sub(st->output_Fs_fx,8000)!=0 )
        {
            st->pfstat.reset = 1;
            if(st->pfstat.on!=0)
            {
                st->pfstat.reset = 0;
                Scale_sig(st->pfstat.mem_pf_in, L_SUBFR, negate(st->Q_syn));           /* WB post_filter mem */
                Scale_sig(st->pfstat.mem_stp, L_SUBFR, negate(st->Q_syn));             /* WB post_filter mem */
                lerp( st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_stp+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
                lerp( st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_old, st->pfstat.mem_pf_in+L_SYN_MEM-mem_syn_r_size_new, mem_syn_r_size_new, mem_syn_r_size_old );
            }
        }
        ELSE{
            Scale_sig(st->pfstat.mem_pf_in, L_SUBFR, negate(st->Q_syn));           /* NB post_filter mem */
            Scale_sig(st->pfstat.mem_res2, DECMEM_RES2, negate(st->Q_syn));         /* NB post_filter mem */
            Scale_sig(st->pfstat.mem_stp, L_SUBFR, negate(st->Q_syn));             /* NB post_filter mem */
        }
    }

    /* bass pf reset */
    st->bpf_gain_param = 0;
    move16();
    set16_fx( st->pst_old_syn_fx, 0, NBPSF_PIT_MAX );

    /* lsf and lsp initialization */
    IF ( st->ini_frame_fx == 0 )
    {
        Copy(st->lsp_old_fx, st->lspold_uw, M);
        Copy(st->lsf_old_fx, st->lsfold_uw, M);

        set16_fx(st->lsf_cng, 0, M);
    }

    st->seed_tcx_plc = 21845;
    move16();
    st->past_gpit = 0;
    move16();
    st->past_gcode = L_deposit_l(0);
    st->gc_threshold_fx = L_deposit_l(0);

    E_LPC_lsf_lsp_conversion(st->lsf_cng, st->lspold_cng, M);
    E_LPC_f_lsp_a_conversion(st->lspold_cng, st->Aq_cng, M);
    st->plcBackgroundNoiseUpdated = 0;
    move16();
    Copy(st->lsf_old_fx, st->lsf_q_cng, M);
    Copy(st->lsf_old_fx, st->old_lsf_q_cng, M);
    Copy(st->lsp_old_fx, st->lsp_q_cng, M);
    Copy(st->lsp_old_fx, st->old_lsp_q_cng, M);
    set16_fx(st->mem_syn_unv_back, 0, M);

    st->last_gain_syn_deemph = 32768/2;
    move16();
    st->last_gain_syn_deemph_e = 1;
    move16();
    test();
    IF( sub(st->last_codec_mode,MODE1) == 0 || st->ini_frame_fx == 0 )
    {
        /* this assumes that MODE1 fades out in the frequency domain -
        otherwise some data from MODE1 would be needed here */
        st->last_concealed_gain_syn_deemph = 32768/2;
        move16();
        st->last_concealed_gain_syn_deemph_e = 1;
        move16();
        st->conceal_eof_gain = 32767;
        move16();
    }

    /* Post processing */
    set16_fx(st->mem_Aq,0,NB_SUBFR16k*(M+1));
    st->lp_ener_FER_fx = 15360;
    move16();   /*60 in Q8*/
    IF (st->ini_frame_fx == 0)
    {
        st->prev_bfi_fx = 0;
        move16();
        st->last_core_bfi = -1;
        move16();
    }
    st->prev_old_bfi_fx = 0;
    move16();

    st->nbLostCmpt = 0;
    move16();
    /*st->noise_filling_index = 0;*/ /* not in BASOP */

    Copy(st->lsf_old_fx, st->lsf_adaptive_mean, M);
    Copy(st->lsf_old_fx, st->lsfoldbfi0, M);
    Copy(st->lsf_old_fx, st->lsfoldbfi1, M);
    st->last_good_fx = UNVOICED_CLAS; /* last good received frame for concealment */      move16();
    st->clas_dec  = UNVOICED_CLAS;
    move16();
    move16();
    move16();
    st->old_enr_LP = 0;   /* LP filter E of last good voiced frame  */
    st->enr_old_fx = L_deposit_l(0);   /* energy at the end of the previous frame */
    st->Mode2_lp_gainc = L_deposit_l(0);
    st->Mode2_lp_gainp = L_deposit_l(0);

    st->prev_widow_left_rect = 0;
    move16();

    st->conCngLevelBackgroundTrace  = PLC_MIN_CNG_LEV;   /*Q15*/                  move16();
    st->conNoiseLevelIndex          = PLC_MIN_STAT_BUFF_SIZE-1;
    move16();
    st->conCurrLevelIndex           = 0;
    move16();
    st->conLastFrameLevel           = PLC_MIN_CNG_LEV;  /*Q15*/                   move16();
    set16_fx(st->conNoiseLevelMemory, PLC_MIN_CNG_LEV, PLC_MIN_STAT_BUFF_SIZE); /*Q15*/
    set16_fx(st->conNoiseLevelMemory_e, 0, PLC_MIN_STAT_BUFF_SIZE);
    st->conLastFrameLevel_e           = 0;
    st->conCngLevelBackgroundTrace_e  = 0;

    st->cummulative_damping_tcx = FL2WORD16(1.0f);
    move16();
    st->cummulative_damping = FL2WORD16(1.0f);
    move16();

    FOR ( i=0; i<2*NB_SUBFR16k+2; i++ )
    {
        st->old_pitch_buf_fx[i] = L_deposit_h(st->pit_min);
    }

    FOR ( i=0; i<2*NB_SUBFR16k+2; i++ )
    {
        st->mem_pitch_gain[i] = FL2WORD16_SCALE(1.f,1);/*Q14*/
    }

    st->old_fpitch = L_deposit_h(st->L_frame_fx);
    st->old_fpitchFB = L_deposit_h(st->L_frameTCX);

    /* For phase dispersion */
    st->dm_fx.prev_gain_code = L_deposit_l(0);
    set16_fx(st->dm_fx.prev_gain_pit, 0, 6);
    st->dm_fx.prev_state = 0;
    move16();

    st->voice_fac = -1; /* purely unvoiced  */                                    move16();

    /* TCX-LTP */
    st->tcxltp = getTcxLtp(st->total_brate_fx, st->sr_core, 0);
    move16();

    test();
    IF ( st->ini_frame_fx == 0 || sub(st->last_codec_mode,MODE1)==0)
    {
        st->tcxltp_pitch_int = st->pit_max;
        move16();
        st->tcxltp_pitch_fr = 0;
        move16();
        st->tcxltp_last_gain_unmodified = 0;
        move16();
        IF ( st->ini_frame_fx == 0)
        {
            set16_fx( st->tcxltp_mem_in, 0, TCXLTP_MAX_DELAY );
            set16_fx( st->tcxltp_mem_out, 0, L_FRAME48k );
            st->tcxltp_pitch_int_post_prev = 0;
            move16();
            st->tcxltp_pitch_fr_post_prev = 0;
            move16();
            st->tcxltp_gain_post_prev = 0;
            move16();
            st->tcxltp_filt_idx_prev = -1;
            move16();
        }
    }

    st->lp_error_ener = L_deposit_l(0);
    st->mem_error = L_deposit_l(0);
    st->tcx_cfg.ctx_hm = getCtxHm (st->total_brate_fx, st->rf_flag);
    st->last_ctx_hm_enabled = 0;
    move16();

    st->tcx_cfg.resq = getResq(st->total_brate_fx);
    move16();

    st->tcx_cfg.sq_rounding = FL2WORD16(0.375f); /*deadzone of 1.25->rounding=1-1.25/2 (No deadzone=0.5)*/    move16();

    st->tcx_lpc_shaped_ari = getTcxLpcShapedAri(
                                 st->total_brate_fx,
                                 bandwidth_mode
                                 ,st->rf_flag
                             );

    st->envWeighted = 0;
    move16();

    st->p_bpf_noise_buf = NULL;
    if (st->tcxonly == 0)
    {
        st->p_bpf_noise_buf = st->bpf_noise_buf;
    }

    st->tec_tfa = 0;
    move16();
    test();
    test();
    if ( sub(bandwidth_mode, SWB) == 0 &&
            (L_sub(st->total_brate_fx, ACELP_16k40) == 0 || L_sub(st->total_brate_fx, ACELP_24k40) == 0) )
    {
        st->tec_tfa = 1;
        move16();
    }

    st->tec_flag = 0;
    move16();
    st->tfa_flag = 0;
    move16();

    st->enableGplc = 0;
    move16();

    st->flagGuidedAcelp = 0;
    move16();
    st->T0_4th = L_SUBFR;
    move16();
    st->guidedT0 = st->T0_4th;
    move16();
    test();
    test();
    test();
    IF ( (st->ini_frame_fx == 0) || (st->bfi_fx || !st->prev_bfi_fx) || sub(st->last_codec_mode, MODE1) == 0 )
    {
        st->tonality_flag = 0;
        move16();
        concealment_init_x(st->L_frameTCX, &st->plcInfo);
    }

    st->enablePlcWaveadjust = 0;
    move16();
    if(L_sub(st->total_brate_fx, 48000) >= 0 )
    {
        st->enablePlcWaveadjust = 1;
        move16();
    }

    /* PLC: [TCX: Tonal Concealment] */
    st->tonalMDCTconceal.nScaleFactors = 0;
    move16();
    st->tonalMDCTconceal.nSamples = 0;
    move16();
    st->tonalMDCTconceal.lastPcmOut = 0x0;
    move16();
    st->tonalMDCTconceal.lastBlockData.tonalConcealmentActive = 0;
    move16();
    st->tonalMDCTconceal.lastBlockData.nSamples = 0;
    move16();

    TonalMDCTConceal_Init(&st->tonalMDCTconceal,
                          s_max(st->L_frame_fx,st->L_frameTCX),
                          st->L_frame_fx,
                          FDNS_NPTS,
                          &(st->tcx_cfg),
                          mdct_shaping_16
                         );

    st->last_tns_active = 0;
    st->second_last_tns_active = 0;
    st->second_last_core = -1;
    st->tcxltp_second_last_pitch = st->old_fpitch;
    move16();
    st->tcxltp_third_last_pitch  = st->old_fpitch;
    move16();

    move16();
    st->dec_glr = 0;
    test();
    test();
    test();
    if( (L_sub(st->total_brate_fx, 9600)==0)||( L_sub(st->total_brate_fx, 16400)==0)||
            (L_sub(st->total_brate_fx, 24400)==0))
    {
        move16();
        st->dec_glr = 1;
    }
    move16();
    st->dec_glr_idx = 0;

    return;
}

