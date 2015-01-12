/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "basop_util.h"
#include "prot_fx.h"
#include "stl.h"
#include "options.h"
#include "rom_com_fx.h"


void mode_switch_decoder_LPD( Decoder_State_fx *st, Word16 bandwidth, Word32 bitrate, Word16 frame_size_index
                            )
{
    Word16 fscale, switchWB;
    Word32 sr_core;
    Word8 bSwitchFromAmrwbIO;
    Word16 frame_size;


    switchWB = 0;
    move16();
    bSwitchFromAmrwbIO = 0;
    move16();

    if (sub(st->last_core_fx,AMR_WB_CORE)==0)
    {
        bSwitchFromAmrwbIO = 1;
        move16();
    }

    sr_core = getCoreSamplerateMode2(bitrate, bandwidth, st->rf_flag);

    fscale = sr2fscale(sr_core);
    move16();

    /* set number of coded lines */
    st->tcx_cfg.tcx_coded_lines = getNumTcxCodedLines(bandwidth);

    test();
    test();
    IF (( (sub(bandwidth, WB) >= 0) && (sub(fscale, (FSCALE_DENOM*16000)/12800) == 0) && (sub(fscale, st->fscale) == 0) ))
    {
        test();
        test();
        test();
        if ( ((L_sub(bitrate, 32000) > 0) && (st->tcxonly == 0)) || ((L_sub(bitrate, 32000) <= 0) && (st->tcxonly != 0)) )
        {
            switchWB = 1;
            move16();
        }
    }

    test();
    if( sub(st->last_L_frame_fx,L_FRAME16k) >0  && L_sub(st->total_brate_fx,ACELP_32k) <=0  )
    {
        switchWB = 1;
        move16();
    }


    st->igf = getIgfPresent(bitrate, bandwidth, st->rf_flag);

    st->hIGFDec.infoIGFStopFreq = -1;
    move16();
    IF( st->igf )
    {
        /* switch IGF configuration */
        IGFDecSetMode( &st->hIGFDec, st->total_brate_fx, bandwidth, -1, -1, st->rf_flag);

    }

    test();
    test();
    test();
    IF (  sub(fscale, st->fscale) != 0 || switchWB != 0 || bSwitchFromAmrwbIO != 0 || sub(st->last_codec_mode,MODE1)==0 )
    {
        open_decoder_LPD( st, bitrate, bandwidth );
    }
    ELSE
    {
        assert(fscale > (FSCALE_DENOM/2));

        st->fscale_old  = st->fscale;
        move16();
        st->fscale      = fscale;
        move16();
        st->L_frame_fx  = extract_l(Mult_32_16(st->sr_core , 0x0290));
        st->L_frameTCX  = extract_l(Mult_32_16(st->output_Fs_fx , 0x0290));

        st->tcx_cfg.ctx_hm = getCtxHm(bitrate, st->rf_flag);
        st->tcx_cfg.resq   = getResq(bitrate);
        move16();

        st->tcx_lpc_shaped_ari = getTcxLpcShapedAri(
            bitrate,
            bandwidth
            ,st->rf_flag
        );

        st->narrowBand = 0;
        move16();
        if ( sub(bandwidth, NB) == 0 )
        {
            st->narrowBand = 1;
            move16();
        }
        st->TcxBandwidth = getTcxBandwidth(bandwidth);

        st->tcx_cfg.pCurrentTnsConfig = NULL;
        st->tcx_cfg.fIsTNSAllowed = getTnsAllowed(bitrate, st->igf );
        move16();

        IF (st->tcx_cfg.fIsTNSAllowed != 0)
        {
            InitTnsConfigs(bwMode2fs[bandwidth], st->tcx_cfg.tcx_coded_lines, st->tcx_cfg.tnsConfig, st->hIGFDec.infoIGFStopFreq, st->total_brate_fx);
        }
    }

    frame_size = FrameSizeConfig[frame_size_index].frame_net_bits;
    move16();

    reconfig_decoder_LPD( st, frame_size, bandwidth, bitrate, st->last_L_frame_fx );

    test();
    IF (st->envWeighted != 0 && st->enableTcxLpc == 0)
    {
        Copy(st->lspold_uw, st->lsp_old_fx, M);
        Copy(st->lsfold_uw, st->lsf_old_fx, M);
        st->envWeighted = 0;
        move16();
    }

    /* update PLC LSF memories */
    lsp2lsf_fx( st->lsp_old_fx, st->lsfoldbfi1, M, extract_l(st->sr_core) );
    mvr2r_Word16(st->lsfoldbfi1, st->lsfoldbfi0,M);
    mvr2r_Word16(st->lsfoldbfi1, st->lsf_adaptive_mean,M);


    IF( st->igf != 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF( (sub(st->bwidth_fx, WB)==0 && sub(st->last_extl_fx, WB_TBE)!=0) ||
            (sub(st->bwidth_fx, SWB)==0 && sub(st->last_extl_fx, SWB_TBE)!=0) ||
            (sub(st->bwidth_fx, FB)==0 && sub(st->last_extl_fx,FB_TBE)!=0) )
        {
            TBEreset_dec_fx(st, st->bwidth_fx);
        }
        ELSE
        {
            set16_fx( st->state_lpc_syn_fx, 0, LPC_SHB_ORDER );
        }
    }

    test();
    test();
    IF( (sub(bandwidth, SWB) == 0) &&
        (L_sub(st->total_brate_fx, ACELP_16k40) == 0 || L_sub(st->total_brate_fx, ACELP_24k40) == 0)
      )
    {
        IF (st->tec_tfa == 0)
        {
            set16_fx(st->tecDec_fx.loBuffer, 0, MAX_TEC_SMOOTHING_DEG);
        }
        st->tec_tfa = 1;
        move16();
    }
    ELSE
    {
        st->tec_tfa = 0;
        move16();
    }

    st->tec_flag = 0;
    move16();
    st->tfa_flag = 0;
    move16();

    /* needed in decoder to read the bitstream */
    st->enableGplc = 0;
    move16();
    test();
    test();
    test();
    test();
    test();
    if( (sub(bandwidth, WB ) == 0 && L_sub(bitrate, 24400) == 0) ||
            (sub(bandwidth, SWB) == 0 && L_sub(bitrate, 24400) == 0 )||
            (sub(bandwidth, FB) == 0 && L_sub(bitrate, 24400) == 0 ) )
    {
        st->enableGplc = 1;
        move16();
    }
    move16();
    st->dec_glr = 0;
    test();
    test();
    if( (L_sub(st->total_brate_fx, 9600)==0)||( L_sub(st->total_brate_fx, 16400)==0)||
            (L_sub(st->total_brate_fx, 24400)==0))
    {
        st->dec_glr = 1;
        move16();
    }
    move16();
    st->dec_glr_idx = 0;

    test();
    test();
    test();
    IF ( (st->ini_frame_fx == 0) || (st->bfi_fx || !st->prev_bfi_fx) || sub(st->last_codec_mode, MODE1) == 0)
    {
        st->tonality_flag = 0;
        move16();
        concealment_init_x(st->L_frameTCX, &st->plcInfo);
    }

    st->enablePlcWaveadjust = 0;
    move16();
    if (L_sub(bitrate,48000) >= 0)
    {
        st->enablePlcWaveadjust = 1;
        move16();
    }
}

