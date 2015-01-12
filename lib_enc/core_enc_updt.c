/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"

void core_encode_update(Encoder_State_fx *st
                       )
{
    Word16 n;



    /*--------------------------------------------------------------*
    * Update Buffers
    *---------------------------------------------------------------*/

    /* Update Input Signal Buffers */

    n = add(st->encoderPastSamples_enc, st->encoderLookahead_enc);

    Copy(st->buf_speech_enc   +st->L_frame_fx, st->buf_speech_enc,    n);
    Copy(st->buf_speech_enc_pe+st->L_frame_fx, st->buf_speech_enc_pe, n);


    IF(!st->tcxonly)
    {
        n = add(st->L_frame_fx, shr(st->L_frame_fx,2));
        Copy(st->buf_wspeech_enc+st->L_frame_fx, st->buf_wspeech_enc, n);
    }


    IF ( s_or( s_or(st->core_fx == ACELP_CORE, s_or( L_sub(st->core_brate_fx, SID_2k40) == 0, L_sub(st->core_brate_fx, FRAME_NO_DATA) == 0) ), L_sub(st->core_fx, AMR_WB_CORE) == 0 ) )
    {
        Copy(st->new_speech_enc, st->new_speech_ltp, st->L_frame_fx);
    }

    n = add(st->encoderPastSamples_enc, st->encoderLookahead_enc);
    Copy(st->buf_speech_ltp+st->L_frame_fx, st->buf_speech_ltp, n);
    Copy(st->buf_synth+st->L_frame_fx, st->buf_synth, add(st->L_frame_fx,L_SUBFR));

    /* Update previous mode */

    move16();
    st->last_core_fx = st->core_fx; /* not in float -> remove? */


    test();
    test();
    test();
    IF( ((L_sub(st->core_brate_fx,SID_2k40) <= 0) && sub(st->cng_type_fx, FD_CNG) == 0)
        || (st->tcxonly && sub(st->codec_mode,MODE2)==0)
      )
    {
        /* reset LP memories */
        set16_fx( st->mem_MA_fx,0, M );
        Copy( GEWB_Ave_fx, st->mem_AR_fx, M );
    }

}


void core_encode_update_cng( Encoder_State_fx *st,
                             Word16 *timeDomainBuffer,
                             Word16 *A,
                             Word16 *Aw,
                             Word16 Q_new,
                             Word16 shift
                           )
{
    Word16 i;
    Word16 lsp[M];
    Word16 lsf[M]; /* 14Q1 * 1.28 */
    Word16 *synth, synth_buf[M+1+L_FRAME_PLUS+L_FRAME_PLUS/2], wsyn[L_FRAME_PLUS];
    Word16 *p_A, tmp;
    Word16 enr_index;
    Word16 enr, tmpv, maxv, scale, att;
    Word16 hi, lo;
    Word16 *pt_res;
    Word32 L_tmp, L_ener;


    /* LPC -> LSP/lsp */
    /* LSP/lsp -> LSF/lsf */
    E_LPC_a_lsp_conversion( A, lsp, st->lsp_old_fx, M );
    IF(sub(st->L_frame_fx, L_FRAME16k)== 0)
    {
        lsp2lsf_fx( lsp, lsf, M, INT_FS_16k_FX );
    }
    ELSE
    {
        E_LPC_lsp_lsf_conversion( lsp, lsf, M );
    }

    /* Update synth memory */
    move16();
    synth = synth_buf + (1+M);
    Copy( st->LPDmem.syn, synth_buf, 1+M );
    Copy( timeDomainBuffer, synth, st->L_frame_fx );
    Copy( synth+st->L_frame_fx-(1+M), st->LPDmem.syn, 1+M );
    Copy( synth, st->synth, st->L_frame_fx );

    /* Update ZIR */
    set16_fx( synth+st->L_frame_fx, 0, st->L_frame_fx/2 );
    E_UTIL_synthesis(1, A, synth+st->L_frame_fx, synth+st->L_frame_fx, st->L_frame_fx/2, &synth[st->L_frame_fx-M], 0, M );
    Copy( synth+st->L_frame_fx-(st->L_frame_fx/2), st->LPDmem.Txnq, st->L_frame_fx );

    /* Update pe-synth memory */
    move16();
    tmp = synth[-(1+M)];
    E_UTIL_f_preemph2( Q_new-1, synth-M, st->preemph_fac, M+st->L_frame_fx, &tmp );
    Copy( synth+st->L_frame_fx-M, st->LPDmem.mem_syn, M );
    Copy( synth+st->L_frame_fx-M, st->LPDmem.mem_syn2, M );

    /* Update excitation memory */
    Copy( st->LPDmem.old_exc+st->L_frame_fx, st->LPDmem.old_exc, s_max(L_EXC_MEM-st->L_frame_fx, 0 ));
    Residu3_fx( A, synth, st->LPDmem.old_exc+s_max(L_EXC_MEM-st->L_frame_fx, 0), s_max(L_EXC_MEM-st->L_frame_fx, 0), 1 );

    /* Update LP_CNG memory */
    IF( L_sub(st->core_brate_fx, SID_2k40) == 0 )
    {
        pt_res = st->LPDmem.old_exc + L_EXC_MEM - st->L_frame_fx;
        maxv = 0;
        move16();
        FOR(i = 0; i < st->L_frame_fx; i++)
        {
            maxv = s_max(maxv, abs_s(pt_res[i]));
        }
        scale = norm_s(maxv);
        L_ener = L_deposit_l(1);
        IF( sub(st->L_frame_fx, L_FRAME) == 0)
        {
            FOR (i=0; i<128; i++)
            {
                tmpv = shl(*pt_res,scale);
                L_tmp = L_mult0(tmpv, tmpv);
                pt_res++;
                tmpv = shl(*pt_res,scale);
                L_tmp = L_mac0(L_tmp, tmpv, tmpv); /* 2*(Q_new+scale) */
                pt_res++;
                L_ener = L_add(L_ener, L_shr(L_tmp, 7)); /* 2*(Q_new+scale)+1, divide by L_frame done here */
            }
        }
        ELSE /* L_FRAME16k */
        {
            FOR (i=0; i<160; i++)
            {
                tmpv = shl(*pt_res,scale);
                L_tmp = L_mult0(tmpv, tmpv);
                pt_res++;
                tmpv = shl(*pt_res,scale);
                L_tmp = L_mac0(L_tmp, tmpv, tmpv); /* 2*(Q_new+scale) */
                pt_res++;
                L_ener = L_add(L_ener, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*(Q_new+scale)+15+1-16+1, divide by L_frame done here */
            }
        }

        hi = norm_l(L_ener);
        lo = Log2_norm_lc(L_shl(L_ener, hi));
        hi = sub(30, add(hi, shl(add(Q_new, scale), 1)));   /* log2 exp in Q2*(Q_new+scale) */
        L_tmp = L_Comp(hi, lo); /* Q16 */
        enr = round_fx(L_shl(L_tmp, 8)); /* Q8 (16+8-16) */

        /* decrease the energy in case of WB input */
        IF( sub(st->bwidth_fx, NB) != 0 )
        {
            IF( sub(st->bwidth_fx,WB) == 0 )
            {
                IF( st->CNG_mode_fx >= 0 )
                {
                    /* Bitrate adapted attenuation */
                    att = ENR_ATT_fx[st->CNG_mode_fx];
                }
                ELSE
                {
                    /* Use least attenuation for higher bitrates */
                    att = ENR_ATT_fx[4];
                }
            }
            ELSE
            {
                att = 384;
                move16();/*Q8*/
            }
            enr = sub(enr, att );
        }

        /* calculate the energy quantization index */
        enr_index = add(enr, 512 /* Q8(2.0) */);   /* enr + 2.0 */
        enr_index = extract_l(L_shr(L_mult0(enr_index, STEP_SID_FX), 12+8));   /* Q0 (8+12-(8+12)) */

        /* limit the energy quantization index */
        enr_index = s_min(enr_index, 127);
        enr_index = s_max(enr_index, 0);

        st->old_enr_index_fx = enr_index;
    }

    /* Update weighted synthesis memory */
    move16();
    p_A = Aw;
    FOR ( i = 0; i < st->L_frame_fx; i += L_SUBFR )
    {
        Residu3_fx( p_A, &synth[i], &wsyn[i], L_SUBFR, 0);
        p_A += (M+1);
    }
    move16();



    tmp = sub(st->wspeech_enc[-1],shl(st->LPDmem.mem_w0,shift));
    E_UTIL_deemph2( -shift, wsyn, st->preemph_fac, st->L_frame_fx, &tmp );
    st->LPDmem.mem_w0 = sub(st->wspeech_enc[st->L_frame_fx-1], tmp);
    st->LPDmem.mem_w0 =shr(st->LPDmem.mem_w0, shift);

    /* Update LPC-related memories */

    Copy( lsp, st->lsp_old_fx, M );
    Copy( lsf, st->lsf_old_fx, M );
    move16();
    st->envWeighted = 0;
    Copy( A, st->old_Aq_12_8_fx, M+1 );
    st->old_Es_pred_fx=0;


    /* Reset acelp memories */
    move16();
    move16();
    st->dm_fx.prev_gain_code = L_deposit_l(0);
    set16_fx(st->dm_fx.prev_gain_pit,0,6);
    st->dm_fx.prev_state = 0;
    st->LPDmem.tilt_code = TILT_CODE;
    st->LPDmem.gc_threshold = L_deposit_l(0);

    /* Update ace/tcx mode */
    move16();
    st->core_fx = ACELP_CORE;

    /* Reset TCX overlap */
    st->tcx_cfg.tcx_curr_overlap_mode = st->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;
    move16();

    IF( st->first_CNG_fx == 0 )
    {
        Copy( st->lsp_old_fx, st->lspCNG_fx, M );
    }

    return;
}

