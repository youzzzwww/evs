/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "basop_util.h"
#include "prot_fx.h"
#include "options.h"
#include "stl.h"
#include "rom_com_fx.h"


void reconfig_decoder_LPD( Decoder_State_fx *st, Word16 bits_frame, Word16 bandwidth_mode, Word32 bitrate, Word16 L_frame_old)
{


    move16();
    st->bits_frame=bits_frame;

    IF (bandwidth_mode==0)
    {
        move16();
        st->narrowBand = 1;
    }
    ELSE if (bandwidth_mode>0)
    {
        move16();
        st->narrowBand = 0;
    }

    BITS_ALLOC_init_config_acelp(bitrate, st->narrowBand, st->nb_subfr, &(st->acelp_cfg));

    /*Configuration of partial copy*/
    st->acelp_cfg_rf.mode_index = 1;
    st->acelp_cfg_rf.midLpc = 0;
    st->acelp_cfg_rf.midLpc_enable = 0;
    st->acelp_cfg_rf.pre_emphasis = 0;
    st->acelp_cfg_rf.formant_enh = 1;
    st->acelp_cfg_rf.formant_tilt = 1;
    st->acelp_cfg_rf.voice_tilt = 1;
    st->acelp_cfg_rf.formant_enh_num = FORMANT_SHARPENING_G1;
    st->acelp_cfg_rf.formant_enh_den = FORMANT_SHARPENING_G2;


    st->flag_cna = getCnaPresent(bitrate, bandwidth_mode);
    move16();

    /* TCX-LTP */
    st->tcxltp = getTcxLtp(bitrate, st->sr_core, 0);
    move16();

    {
        Word16 i;

        /*Scale TCX for non-active frames to adjust loudness with ACELP*/
        st->tcx_cfg.na_scale=FL2WORD16(1.0f);

        test();
        IF ((sub(bandwidth_mode,SWB)<0) && !(st->tcxonly))
        {
            Word16 scaleTableSize = sizeof (scaleTcxTable) / sizeof (scaleTcxTable[0]);  /* is a constant */

            FOR (i = 0 ; i < scaleTableSize ; i++)
            {
                test();
                test();
                IF ( (sub (bandwidth_mode,scaleTcxTable[i].bwmode) == 0) &&
                     (L_sub (bitrate,scaleTcxTable[i].bitrateFrom) >= 0) &&
                     (L_sub (bitrate,scaleTcxTable[i].bitrateTo)  < 0 )
                   )
                {
                    if( st->rf_flag )
                    {
                        i--;
                    }
                    st->tcx_cfg.na_scale=scaleTcxTable[i].scale;
                    BREAK;
                }
            }
        }
    }

    /*if its not the first frame resample overlap buffer to new sampling rate */
    IF ( st->ini_frame_fx != 0 )
    {
        IF ( sub (st->fscale,st->fscale_old) != 0 )
        {
            Word16 newLen;
            Word16 oldLen;

            newLen = st->tcx_cfg.tcx_mdct_window_length;
            move16();
            oldLen = st->tcx_cfg.tcx_mdct_window_length_old;
            move16();

            test();
            test();
            IF( (st->prev_bfi_fx && sub(st->last_core_bfi,ACELP_CORE)==0) || sub(st->last_core_fx,ACELP_CORE)==0 )
            {
                newLen = shr(st->L_frame_fx,1);
                oldLen = shr(L_frame_old,1);
            }

            lerp( st->old_syn_Overl, st->old_syn_Overl, newLen, oldLen );

            test();
            test();
            IF( (st->prev_bfi_fx && sub(st->last_core_bfi,ACELP_CORE)==0) || sub(st->last_core_fx,ACELP_CORE)==0 )
            {
                test();
                IF( st->prev_bfi_fx && sub(st->last_core_bfi,ACELP_CORE)==0 )
                {
                    lerp( st->syn_Overl_TDAC, st->syn_Overl_TDAC, newLen, oldLen );
                    lerp( st->syn_Overl,      st->syn_Overl,      newLen, oldLen );
                }
            }
        }


        IF (sub(st->L_frame_fx,L_FRAME16k) <= 0)
        {
            IF (sub(st->last_L_frame_fx,L_FRAME16k) <= 0)
            {
                IF (sub(st->L_frame_fx,st->last_L_frame_fx) != 0)
                {
                    Word16 oldLenClasBuff;
                    Word16 newLenClasBuff;
                    IF (sub(st->L_frame_fx,st->last_L_frame_fx) > 0)
                    {
                        oldLenClasBuff = extract_l(L_shr(Mpy_32_16_1(L_mult0(st->last_L_frame_fx,getInvFrameLen(st->L_frame_fx)/*Q21*/)/*Q21*/,L_SYN_MEM_CLAS_ESTIM/*Q0*/)/*Q6*/,6)/*Q0*/);

                        newLenClasBuff = L_SYN_MEM_CLAS_ESTIM;
                        move16();

                    }
                    ELSE
                    {
                        oldLenClasBuff = L_SYN_MEM_CLAS_ESTIM;
                        newLenClasBuff = extract_l(L_shr(Mpy_32_16_1(L_mult0(st->L_frame_fx,getInvFrameLen(st->last_L_frame_fx)/*Q21*/)/*Q21*/,L_SYN_MEM_CLAS_ESTIM/*Q0*/)/*Q6*/,6)/*Q0*/);
                    }
                    lerp(st->mem_syn_clas_estim_fx, st->mem_syn_clas_estim_fx, newLenClasBuff, oldLenClasBuff);
                }
            }
            ELSE
            {
                set16_fx(st->mem_syn_clas_estim_fx, 0, L_SYN_MEM_CLAS_ESTIM);
            }
        }
    }
    test();
    test();
    st->enableTcxLpc = (st->numlpc == 1) && (st->lpcQuantization == 1) && (bitrate <= LOWRATE_TCXLPC_MAX_BR || st->rf_flag);

    if (st->ini_frame_fx == 0)
    {
        st->envWeighted = 0;
    }


    return;
}
