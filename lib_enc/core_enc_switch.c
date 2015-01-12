/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_mpy.h"
#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"

#include "stl.h"

void core_coder_mode_switch( Encoder_State_fx *st, const Word16 bandwidth_in,
                             const Word32 bitrate,
                             const Word16 shift)
{
    Word16 i, fscale, switchWB;
    Word16 bSwitchFromAmrwbIO;
    Word16 bandwidth;
    Word32 tmp32;
    Word32 sr_core;


    move16();
    move16();
    move16();
    move16();

    bandwidth = bandwidth_in;

    switchWB = 0;
    bSwitchFromAmrwbIO = 0;
    if ( sub(st->last_core_fx, AMR_WB_CORE) == 0 )
    {
        move16();
        bSwitchFromAmrwbIO = 1;
    }

    sr_core = getCoreSamplerateMode2(bitrate, bandwidth, st->rf_mode);
    move16();
    fscale  = sr2fscale(sr_core);
    move16();

    IF ( s_and(s_and(sub(bandwidth,WB)>=0,  sub(fscale, (FSCALE_DENOM*16000)/12800)==0), sub(fscale, st->fscale)==0))
    {
        IF ( s_or(s_and(L_sub(bitrate, 32000)>0, (st->tcxonly==0)), s_and(L_sub(bitrate, 32000)<=0, st->tcxonly!=0)) )
        {
            move16();
            switchWB = 1;
        }
    }
    if( sub(st->last_codec_mode,MODE1)==0 )
    {
        move16();
        switchWB = 1;  /*force init when coming from MODE1*/
    }
    test();
    if( L_sub(st->last_total_brate_fx,ACELP_32k)>0 && L_sub(st->total_brate_fx,ACELP_32k) <= 0)
    {
        move16();
        switchWB = 1;  /*force init when coming from MODE1*/
    }

    test();
    test();
    IF ( (sub(fscale, st->fscale)==0) && (bSwitchFromAmrwbIO==0) && (switchWB==0) )
    {
        st->total_brate_fx = bitrate;
        move32();
        st->sr_core = sr_core;
        move32();
        st->L_frame_fx = extract_l(Mult_32_16(st->sr_core , 0x0290));
        assert(st->L_frame_fx == st->sr_core / 50);
        st->tcxonly = getTcxonly(st->total_brate_fx);

        /* st->bits_frame_nominal = (int)( (float)st->L_frame_fx/(float)st->fscale ) * (float)FSCALE_DENOM/128.0f * (float)st->bitrate/100.0f + 0.49f ; */
        /* st->bits_frame_nominal = extract_l(L_shr(Mpy_32_16_1( L_shl(st->bitrate,8), mult_r(div_s(st->fscale, shl(st->L_frame_fx,4)), FL2WORD16(FSCALE_DENOM/12800.f))), 6)); */
        tmp32 = L_shl(st->total_brate_fx, 1); /* (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->bitrate */
        st->bits_frame_nominal = extract_l(L_shr(Mpy_32_16_1(tmp32, 20972), 6)); /* 20972 = 0.01 * 64 * 32768 */
        assert(st->bits_frame_nominal == (int)( (float)st->L_frame_fx/(float)st->fscale * (float)FSCALE_DENOM/128.0f * (float)st->total_brate_fx/100.0f + 0.49f ));

        st->igf = getIgfPresent(bitrate, bandwidth, st->rf_mode);

        /* switch IGF configuration */
        IF (st->igf)
        {
            IGFEncSetMode( &st->hIGFEnc, st->total_brate_fx, bandwidth, st->rf_mode );
        }

        st->tcx_cfg.tcx_coded_lines = getNumTcxCodedLines(bandwidth);
        move16();
        st->tcx_cfg.bandwidth = getTcxBandwidth(st->bwidth_fx);
        move16();
        move16();
        st->tcx_cfg.tcxRateLoopOpt = 0;
        if (st->tcxonly)
        {
            move16();
            st->tcx_cfg.tcxRateLoopOpt = 2;
        }

        st->tcx_cfg.ctx_hm = getCtxHm(st->total_brate_fx, st->rf_mode );
        move16();
        st->tcx_cfg.resq = getResq(st->total_brate_fx);
        move16();
        st->tcx_lpc_shaped_ari = getTcxLpcShapedAri(
                                     st->total_brate_fx,
                                     bandwidth
                                     ,st->rf_mode
                                 );

        test();
        if (st->tcx_cfg.resq != 0 && st->tcxonly == 0)
        {
            st->tcx_cfg.tcxRateLoopOpt = 1;
            move16();
        }
        st->tcx_cfg.fIsTNSAllowed = getTnsAllowed(st->total_brate_fx, st->igf);
        IF (st->tcx_cfg.fIsTNSAllowed != 0)
        {
            InitTnsConfigs(bwMode2fs[bandwidth], st->tcx_cfg.tcx_coded_lines, st->tcx_cfg.tnsConfig, st->hIGFEnc.infoStopFrequency, st->total_brate_fx);
        }
        move16();

        st->narrowBand = 0;
        move16();
        if(sub(st->bwidth_fx, NB) == 0)
        {
            st->narrowBand = 1;
            move16();
        }
        IF ( st->narrowBand )
        {
            st->min_band_fx = 1;
            move16();
            st->max_band_fx = 16;
            move16();
        }
        ELSE
        {
            st->min_band_fx = 0;
            move16();
            st->max_band_fx = 19;
            move16();
        }

        FOR (i=0; i<FRAME_SIZE_NB; i++)
        {

            IF ( sub(FrameSizeConfig[i].frame_bits, st->bits_frame_nominal) == 0 )
            {
                move16();
                move16();
                move16();
                st->frame_size_index = i;
                st->bits_frame = FrameSizeConfig[i].frame_bits;
                st->bits_frame_core = FrameSizeConfig[i].frame_net_bits;
                BREAK;
            }
        }

        st->restrictedMode = getRestrictedMode(st->total_brate_fx, 0);

        core_coder_reconfig( st );
    }
    ELSE
    {
        st->igf = getIgfPresent(bitrate, bandwidth, st->rf_mode);
        init_coder_ace_plus(st, shift);
    }

    IF( st->igf != 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF( (sub(st->bwidth_fx, WB) == 0 && sub(st->last_extl_fx, WB_TBE) != 0) ||
            (sub(st->bwidth_fx, SWB) == 0 && sub(st->last_extl_fx, SWB_TBE) != 0) ||
            (sub(st->bwidth_fx, FB) == 0 && sub(st->last_extl_fx, FB_TBE) != 0) )
        {
            /* reset TBE buffers as previous frame wasn't using TBE */
            TBEreset_enc_fx( st, st->bwidth_fx );
        }
        ELSE
        {
            set16_fx( st->state_lpc_syn_fx, 0, LPC_SHB_ORDER );
        }
    }

    IF ( s_and(st->envWeighted!=0, st->enableTcxLpc==0) )
    {
        /* Unweight the envelope */
        E_LPC_lsp_unweight(
            st->lsp_old_fx,
            st->lsp_old_fx,
            st->lsf_old_fx,
            st->inv_gamma,
            M
        );
        move16();
        st->envWeighted = 0;
    }

    st->enablePlcWaveadjust = 0;
    move16();
    if (L_sub(bitrate, 48000) >= 0)
    {
        st->enablePlcWaveadjust = 1;
        move16();
    }

    test();
    IF(L_sub(st->last_total_brate_fx, 32000) > 0
       || L_sub(st->last_codec_mode,MODE1)==0 )
    {
        move16();
        st->glr_reset = 1;
    }
}

