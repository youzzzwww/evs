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

void core_encode_twodiv(
    const Word16 new_samples[],
    Encoder_State_fx *st,    /* i/o : coder memory state               */
    const Word16 coder_type,             /* i  : coding type                         */
    const Word16 pitch[3],                /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],             /* i  : open-loop pitch gains               */
    Word16 Aw[NB_SUBFR16k*(M+1)],  /* i  : weighted A(z) unquant. for subframes*/
    Word16 *Q_new,
    Word16 *shift
)
{
    Word16 n;
    Word16 lsp_new[M], lsp_mid[M];
    Word16 lsf_q[M], lsp_q[M], lspmid_q[M];
    Word16 A_q[M+1];
    Word16 param_lpc[NPRM_LPC_NEW];
    Word16 nbits_lpc[2];
    Word16 param_core[2*NPRM_DIV];
    Word16 target_bits;
    Word16 gainlpc[2][FDNS_NPTS];
    Word16 gainlpc_e[2][FDNS_NPTS];
    Word16 tnsSize[2]; /* number of tns parameters put into prm */
    Word16 tnsBits[2]; /* number of tns bits in the frame */
    Word16 ltpBits;
    Word16 bitsAvailable;
    Word32 spectrum_buf[N_MAX];
    Word32 *spectrum[2]; /* MDCT spectrum */
    Word16 spectrum_e[2];
    Word16 indexBuffer[2*((N_MAX/2)+1)];
    CONTEXT_HM_CONFIG hm_cfg[2];
    Word16 i, T_op[3];
    Word16  bits_param_lpc[10], no_param_lpc;


    spectrum[0] = spectrum_buf;
    spectrum[1] = spectrum_buf + N_TCX10_MAX;

    hm_cfg[0].indexBuffer = &indexBuffer[0];
    move16();
    hm_cfg[1].indexBuffer = &indexBuffer[N_MAX/2+1];
    move16();

    move16();
    move16();
    move16();
    move16();
    move16();
    tnsSize[0] = 0;
    tnsSize[1] = 0;
    tnsBits[0] = 0;
    tnsBits[1] = 0;
    ltpBits = 0;

    FOR( i = 0; i < 3; i++ )
    {
        move16();
        T_op[i] = pitch[i];

        /* check minimum pitch for quantization */
        IF ( sub(T_op[i], PIT_MIN_SHORTER) < 0 )
        {
            move16();
            T_op[i] = shl(T_op[i], 1);
        }

        /* convert pitch values to core sampling-rate */
        IF ( sub(st->L_frame_fx, L_FRAME) != 0 )
        {
            move16();
            /* T_op[i] = (short)(T_op[i] * (float)st->L_frame_fx/(float)L_FRAME + 0.5f); */
            T_op[i] = round_fx(L_shr(L_mult0(T_op[i], st->L_frame_fx), 8 - 16));
        }
    }

    /*--------------------------------------------------------------*
    * TCX20/TCX10 switching decision
    *---------------------------------------------------------------*/

    move16();
    st->core_fx = TCX_10_CORE;
    if ( sub(st->tcxMode,TCX_20) == 0 )
    {
        move16();
        st->core_fx = TCX_20_CORE;
    }
    assert(st->tcxMode == TCX_20 || st->tcxMode == TCX_10);

    /*--------------------------------------------------------------*
    * Core Signal Analysis: MDCT, TNS, LPC analysis
    *---------------------------------------------------------------*/

    core_signal_analysis_high_bitrate(  new_samples,
                                        T_op,
                                        voicing,
                                        pitch,
                                        Aw,
                                        lsp_new,
                                        lsp_mid,
                                        st,
                                        tnsSize,
                                        tnsBits,
                                        param_core,
                                        &ltpBits,
                                        st->L_frame_fx,
                                        st->L_frameTCX,
                                        spectrum,
                                        spectrum_e,
                                        Q_new,
                                        shift
                                     );

    /*--------------------------------------------------------------*
    * LPC Quantization
    *---------------------------------------------------------------*/
    lpc_quantization( st, st->core_fx, st->lpcQuantization, st->lsf_old_fx , lsp_new, lsp_mid, lsp_q, lsf_q,
                      lspmid_q, NULL, st->clip_var_fx, st->mem_MA_fx, st->mem_AR_fx,
                      st->narrowBand, coder_type, 0, param_lpc, nbits_lpc, bits_param_lpc, &no_param_lpc, &(st->seed_acelp),
                      st->Bin_E_fx, st->Bin_E_old_fx, add(*Q_new, Q_SCALE - 2) );

    /*--------------------------------------------------------------*
      * Rate switching
      *---------------------------------------------------------------*/
    IF( st->rate_switching_reset!=0 )
    {
        Copy( lsp_q, st->lsp_old_fx, M );
        Copy( lsf_q, st->lsf_old_fx, M );
    }



    /*--------------------------------------------------------------*
    * Run Two TCX10
    *---------------------------------------------------------------*/

    IF ( sub(st->core_fx,TCX_10_CORE) == 0 )
    {
        Word16 last_ace_mode;


        move16();
        last_ace_mode = st->last_core_fx;


        FOR (n = 0; n < 2; n++)
        {
            IF(n == 0)
            {
                E_LPC_f_lsp_a_conversion(lspmid_q, A_q, M);
            }
            ELSE
            {
                E_LPC_f_lsp_a_conversion(lsp_q, A_q, M);
            }

            /* Shape spectrum */
            ShapeSpectrum(&(st->tcx_cfg),
                          A_q,
                          gainlpc[n],
                          gainlpc_e[n],
                          shr(st->L_frame_fx, 1),
                          shr(st->tcx_cfg.tcx_coded_lines, 1),
                          spectrum[n],
                          st->fUseTns[n],
                          st
                         );

            st->last_core_fx = st->core_fx;
            move16();
        }
        st->last_core_fx = last_ace_mode;
        move16();


        /* Calculate target bits */
        bitsAvailable = sub(sub(sub(st->bits_frame_core, nbits_lpc[0]), nbits_lpc[1]), st->nb_bits_header_tcx);

        /* subtract bits for TCX overlap mode (1 bit: full, 2 bits: half or no overlap) */
        bitsAvailable = sub(bitsAvailable,1);
        test();
        if (sub(st->tcx_cfg.tcx_curr_overlap_mode, HALF_OVERLAP) == 0 || sub(st->tcx_cfg.tcx_curr_overlap_mode, MIN_OVERLAP) == 0)
        {
            bitsAvailable = sub(bitsAvailable,1);
        }

        bitsAvailable = sub(bitsAvailable, st->hIGFEnc.infoTotalBitsWritten);

        /* calculate noise-filling over whole spectrum for TCX10 frames */
        move16();
        st->measuredBwRatio = 0x4000;

        FOR (n = 0; n < 2; n++)
        {
            target_bits = sub(shr(sub(add(bitsAvailable, 1), n),1), tnsBits[n]);

            if (n == 0)
            {
                target_bits = sub(target_bits, ltpBits);
            }

            test();
            if(st->enablePlcWaveadjust && n)
            {
                target_bits  = sub(target_bits, 1);
            }


            /* Run TCX10 encoder */

            QuantizeSpectrum(
                &(st->tcx_cfg),
                A_q,
                NULL,
                gainlpc[n],
                gainlpc_e[n],
                st->synth+n*st->L_frame_fx/2,
                shr(st->L_frame_fx, 1),
                shr(st->L_frameTCX, 1),
                shr(st->tcx_cfg.tcx_coded_lines, 1),
                target_bits,
                st->tcxonly,
                spectrum[n],
                &spectrum_e[n],
                st->tnsData+n,
                st->fUseTns[n],
                tnsSize[n],
                &(st->LPDmem),
                param_core+n*NPRM_DIV,
                n,
                st,
                &hm_cfg[n]
            );

            /* Update tcx overlap mode */
            test();
            if ((n > 0) || (st->tcxonly==0))
            {
                move16();
                st->tcx_cfg.tcx_last_overlap_mode = st->tcx_cfg.tcx_curr_overlap_mode;
            }

        }

        coder_tcx_post( st, &(st->LPDmem), &(st->tcx_cfg), st->synth, A_q, Aw, st->wspeech_enc, *Q_new, *shift );
    }

    /*--------------------------------------------------------------*
    * Run One TCX20
    *---------------------------------------------------------------*/

    IF ( sub(st->core_fx,TCX_20_CORE) == 0 )
    {

        E_LPC_f_lsp_a_conversion(lsp_q, A_q, M);


        ShapeSpectrum(&(st->tcx_cfg),
                      A_q,
                      gainlpc[0],
                      gainlpc_e[0],
                      st->L_frame_fx,
                      st->tcx_cfg.tcx_coded_lines,
                      spectrum[0],
                      st->fUseTns[0],
                      st
                     );

        /* Calculate target bits */

        target_bits = sub(sub(sub(sub(st->bits_frame_core, tnsBits[0]), nbits_lpc[0]), st->nb_bits_header_tcx), ltpBits);
        /* subtract bits for TCX overlap mode (1 bit: full, 2 bits: half or no overlap) */
        target_bits = sub(target_bits,1);
        test();
        if (sub(st->tcx_cfg.tcx_curr_overlap_mode, HALF_OVERLAP) == 0 || sub(st->tcx_cfg.tcx_curr_overlap_mode, MIN_OVERLAP) == 0)
        {
            target_bits = sub(target_bits,1);
        }

        target_bits = sub(target_bits, st->hIGFEnc.infoTotalBitsPerFrameWritten);


        if(st->enablePlcWaveadjust)
        {
            target_bits = sub(target_bits, 1);
        }


        QuantizeSpectrum(
            &(st->tcx_cfg),
            A_q,
            NULL,
            gainlpc[0],
            gainlpc_e[0],
            st->synth,
            st->L_frame_fx,
            st->L_frameTCX,
            st->tcx_cfg.tcx_coded_lines,
            target_bits,
            st->tcxonly,
            spectrum[0],
            &spectrum_e[0],
            &st->tnsData[0],
            st->fUseTns[0],
            tnsSize[0],
            &(st->LPDmem),
            param_core,
            0,
            st,
            &hm_cfg[0]
        );

        coder_tcx_post( st, &(st->LPDmem), &(st->tcx_cfg), st->synth, A_q, Aw, st->wspeech_enc, *Q_new, *shift );

    }

    /* Update lsp/lsf memory */
    Copy(lsf_q, st->lsf_old_fx, M);
    Copy(lsp_q, st->lsp_old_fx, M);



    /*--------------------------------------------------------------*
    * Generate Bitstream
    *---------------------------------------------------------------*/

    enc_prm( coder_type, param_core, param_lpc, st,  st->L_frame_fx, hm_cfg, bits_param_lpc, no_param_lpc );

}

