/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "options.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "prot_fx.h"

#define SIG2IND(ctype, bw, sf, ca_rf)         ( ctype | (bw << 3) | (sf << 6) | (ca_rf << 7) )

static void enc_prm_hm(
    Word16 *prm_hm,
    Encoder_State_fx *st,
    Word16 L_frame
)
{
    Word8 flag;

    /* Disable HM for non-GC,VC modes */
    test();
    IF (sub(st->tcx_cfg.coder_type, VOICED) != 0 && sub(st->tcx_cfg.coder_type, GENERIC) != 0)
    {
        return;
    }

    /* Flag */
    push_next_indice_fx(st, prm_hm[0], 1);

    IF (prm_hm[0])
    {
        /* Periodicy index */
        flag = 0;
        move16();
        if ( sub(L_frame, 256) >= 0)
        {
            flag = 1;
            move16();
        }
        EncodeIndex(flag, prm_hm[1], st);

        IF (sub(st->tcx_cfg.coder_type, VOICED) == 0)
        {
            /* Gain index */
            push_next_indice_fx(st, prm_hm[2], kTcxHmNumGainBits);
        }
    }
}

/*-----------------------------------------------------------------*
 * Function  enc_prm_rf()                                          *
 * ~~~~~~~~~~~~~~~~~~~~~~                                          *
 *                                                                 *
 * encode RF parameters for ACELP and TCX partial copy             *
 *-----------------------------------------------------------------*/

void enc_prm_rf( Encoder_State_fx *st,
                 const Word16 rf_frame_type,
                 const Word16 fec_offset
               )
{
    Word16 sfr, nb_subfr, n, index;
    Word16 ltp_mode, ltf_mode, gains_mode;

    nb_subfr = st->nb_subfr;

    /* partial copy bitstream writing */
    test();
    IF ( sub(rf_frame_type,RF_TCXFD) >= 0 && sub(rf_frame_type,RF_TCXTD2) <= 0)
    {
        /* TCX frames partial copy write */

        /* LSF indices */
        IF( sub(rf_frame_type, RF_TCXFD) == 0 )
        {
            push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][0], lsf_numbits[0]);  /* VQ 1 */
            push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][1], lsf_numbits[1]);  /* VQ 2 */
            push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][2], lsf_numbits[2]);  /* VQ 3 */
        }

        /* classification */
        test();
        test();
        IF( sub(st->rf_clas[fec_offset], UNVOICED_CLAS) == 0 )
        {
            index = 0;
            move16();
        }
        ELSE IF( (sub(st->rf_clas[fec_offset], VOICED_TRANSITION) == 0) || (sub(st->rf_clas[fec_offset], UNVOICED_TRANSITION) == 0) )
        {
            index = 1;
            move16();
        }
        ELSE IF( sub(st->rf_clas[fec_offset], VOICED_CLAS) == 0 )
        {
            index = 2;
            move16();
        }
        ELSE
        {
            index = 3;
            move16();
        }
        push_next_indice_fx(st, index, 2);

        IF( sub(rf_frame_type, RF_TCXFD) == 0 )
        {
            /* TCX global gain  = 7 bits */
            push_next_indice_fx(st, st->rf_gain_tcx[fec_offset], 7);

            /*window info
            1 bit for long overlap
            2 if minimum or half overlap*/
        }
        ELSE
        {
            /*gains adapt
            gains inov*/

            /*LPC on full rate -> excitation */
            /* pitch and gain */
            /* LTP data */
            test();
            IF ( (sub(rf_frame_type, RF_TCXTD1) == 0 || sub(rf_frame_type, RF_TCXTD2) == 0) && st->tcxltp != 0 )
            {
                push_next_indice_fx(st, st->rf_tcxltp_param[fec_offset], 9);
            }
        }
    }
    ELSE IF( sub(rf_frame_type,7) == 0 )   /* NELP bitstream writing */
    {
        /* LSF indices */
        push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][0], 8);  /* VQ 1 */
        push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][1], 8);  /* VQ 2 */

        /* NELP gain indices */
        push_next_indice_fx( st, st->rf_indx_nelp_iG1[fec_offset], 5 );
        push_next_indice_fx( st, st->rf_indx_nelp_iG2[fec_offset][0], 6 );
        push_next_indice_fx( st, st->rf_indx_nelp_iG2[fec_offset][1], 6 );

        /* NELP filter selection index */
        push_next_indice_fx( st, st->rf_indx_nelp_fid[fec_offset], 2 );

        /* tbe gainFr */
        push_next_indice_fx( st, st->rf_indx_tbeGainFr[fec_offset], 5 );
    }
    ELSE IF ( sub(rf_frame_type,4) >= 0 ) /* rf_frame_type ALL_PRED: 4, NO_PRED: 5, GEN_PRED: 6 */
    {
        /* LSF indices */
        push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][0], 8);  /* VQ 1 */
        push_next_indice_fx(st, st->rf_indx_lsf[fec_offset][1], 8);  /* VQ 2 */

        /* ES pred */
        push_next_indice_fx(st, st->rf_indx_EsPred[fec_offset], 3);

        ltp_mode = ACELP_LTP_MODE[1][1][rf_frame_type];
        ltf_mode = ACELP_LTF_MODE[1][1][rf_frame_type];
        gains_mode = ACELP_GAINS_MODE[1][1][rf_frame_type];

        /* Subframe parameters */
        FOR( sfr = 0; sfr < nb_subfr; sfr++ )
        {
            /* Pitch lag (5, or 8 bits) */
            n = ACELP_LTP_BITS_SFR[ltp_mode][sfr];
            IF (n != 0)
            {
                push_next_indice_fx(st, st->rf_indx_pitch[fec_offset][sfr], n);
            }

            /* Adaptive codebook filtering (1 bit) */
            IF( sub(ltf_mode,2) == 0 )
            {
                push_next_indice_fx(st, st->rf_indx_ltfMode[fec_offset][sfr], 1);
            }

            /*Innovative codebook*/
            test();
            test();
            test();
            IF( (sub(rf_frame_type,RF_NOPRED) == 0) ||
                (sub(rf_frame_type,RF_GENPRED) == 0 &&
                 (sfr == 0 || sub(sfr,2) == 0)) )
            {
                push_next_indice_fx(st, st->rf_indx_fcb[fec_offset][sfr], 7);
            }

            /* Gains (5b, 6b or 7b / subfr) */
            test();
            IF( sfr == 0 || sub(sfr,2) == 0 )
            {
                n = ACELP_GAINS_BITS[gains_mode];
                push_next_indice_fx(st, st->rf_indx_gain[fec_offset][sfr], n);
            }
        }
        /* tbe gainFr */
        push_next_indice_fx( st, st->rf_indx_tbeGainFr[fec_offset], 2 );
    }

    /***************/
    /*IMPORTANT: The last three bits are always the rf_frame_type in the bitstream (for both acelp and tcx partial copy);
                 the rf_frame_type indicates the length of the partial copy payload at the decoder.
                 The 2 bits before the rf_frame_type contains the fec_offset */

    /***************/
    /* write FEC offset just before the rf_frame_type */
    test();
    test();
    IF(sub(fec_offset,2) == 0 )
    {
        push_next_indice_fx(st, 0, 2);
    }
    ELSE IF(sub(fec_offset,3) == 0 || sub(fec_offset,5) == 0 || sub(fec_offset,7) == 0)
    {
        push_next_indice_fx(st, (fec_offset - 1)/2, 2);
    }

    /* write RF frame type last in the bitstream */
    push_next_indice_fx(st, rf_frame_type, 3);

}




/*-----------------------------------------------------------------*
 * Funtion  enc_prm()                                              *
 * ~~~~~~~~~~~~~~~~~~~~~~                                          *
 *                                                                 *
 * encode parameters according to selected mode including          *
 * the FAC parameters when transition occurs.                      *
 *-----------------------------------------------------------------*/

void enc_prm(
    const Word16 coder_type, /* (i) : coding type                      */
    Word16 param[],          /* (i) : parameters                       */
    Word16 param_lpc[],      /* (i) : LPC parameters                   */
    Encoder_State_fx *st,    /* i/o : quantization Analysis values     */
    Word16 L_frame,
    CONTEXT_HM_CONFIG hm_cfg[]
    ,Word16 * bits_param_lpc,
    Word16 no_param_lpc
)
{
    Word16 j, k, n, sfr, core, last_core, *prm, tmp;
    Word16 nbits_start, total_nbbits;
    Word16 nbits_header;
    Word16 nbits_lpc;
    Word16 nbits_tcx;
    Word16 lg,nb_subfr;
    Word16 lgFB;
    Word16 nTnsParams;
    Word16 nTnsBits;
    Word16 ix, j_old, wordcnt, bitcnt;
    Word16 hm_size;
    Word16 numlpc;
    Word8 flag_ctx_hm;
    Word16 index;
    Word16 idx;
    Word16 start_idx;
    Word16 nBits;

    /*--------------------------------------------------------------------------------*
     * INIT
     *--------------------------------------------------------------------------------*/

    move16();
    move16();
    move16();
    move16();
    move16();
    move16();
    nbits_lpc=0;
    nbits_tcx = 0;

    flag_ctx_hm = 0;

    /* Useful parameters */
    move16();
    move16();
    move16();
    nb_subfr = st->nb_subfr;
    core = st->core_fx;
    last_core = st->last_core_fx;

    /* Initialize pointers */
    move16();
    prm = param;

    /* Init counters */
    move16();
    move16();
    j = 0;
    nbits_start = st->nb_bits_tot_fx;


    /*--------------------------------------------------------------------------------*
     * HEADER
     *--------------------------------------------------------------------------------*/

    IF (sub(st->mdct_sw, MODE1) == 0)
    {
        /* Adjust st->bits_frame_core not to subtract MODE2 bandwidth signaling */
        st->bits_frame_core = add(st->bits_frame_core, FrameSizeConfig[st->frame_size_index].bandwidth_bits);

        /* Write MODE1 core mode signaling */
        signalling_mode1_tcx20_enc(st, 1);
    }

    /* Modes (ACE_GC, ACE_UC, TCX20, TCX10...) */

    IF ( st->tcxonly )
    {
        push_next_indice_fx(st, core==TCX_10_CORE, 1);
        {
            index = 3;
            move16();
            test();
            IF( sub(st->clas_fx, UNVOICED_CLAS) == 0 )
            {
                index = 0;
                move16();
            }
            ELSE IF( (sub(st->clas_fx, VOICED_TRANSITION) == 0) || (sub(st->clas_fx, UNVOICED_TRANSITION) == 0) )
            {
                index = 1;
                move16();
            }
            ELSE IF( sub(st->clas_fx, VOICED_CLAS) == 0 )
            {
                index = 2;
                move16();
            }
            push_next_indice_fx(st, index, 2);
        }
    }
    ELSE
    {
        IF ( core==ACELP_CORE )
        {
            /* write the RF signalling information */
            IF( sub(st->rf_mode,1) == 0)
            {
                /* find the section in the ACELP signalling table corresponding to bitrate */
                idx = 0;
                WHILE ( L_sub(acelp_sig_tbl[idx],st->total_brate_fx) != 0  )  /* total bitrate is kept at 13.2kbps */
                {
                    idx = add(idx,1);
                }

                /* retrieve the number of bits for signalling */
                nBits = (Word16) acelp_sig_tbl[++idx];

                /* retrieve the signalling index */
                idx = add(idx,1);
                start_idx = idx;
                WHILE( L_sub(acelp_sig_tbl[idx], SIG2IND(coder_type, st->bwidth_fx, st->sharpFlag, st->rf_mode)) != 0 )
                {
                    idx = add(idx,1);
                }
                push_next_indice_fx( st, idx - start_idx, nBits);
                push_next_indice_fx( st, 0, 1); /* Indicate to the decoder that the core is ACELP*/
                nbits_start = 3;
            }
            ELSE
            {
                push_next_indice_fx(st, coder_type, 3);
            }
        }
        ELSE
        {
            IF (sub(st->mdct_sw, MODE1) == 0)
            {
                /* 2 bits instead of 3 as TCX is already signaled */
                push_next_indice_fx(st, st->tcx_cfg.coder_type, 2 );
            }
            ELSE
            {
                IF (sub(st->mdct_sw_enable, MODE2) == 0)
                {
                    push_next_indice_fx(st, 1, 1); /* TCX */
                    push_next_indice_fx(st, 0, 1); /* not HQ_CORE */
                    push_next_indice_fx(st, st->tcx_cfg.coder_type, 2);
                }
                ELSE
                {
                    /*write the RF signalling information*/
                    IF( sub(st->rf_mode,1) == 0)
                    {
                        /* find the section in the ACELP signalling table corresponding to bitrate */
                        idx = 0;
                        WHILE (L_sub(acelp_sig_tbl[idx],st->total_brate_fx) != 0)
                        {
                            idx = add(idx,1);
                        }

                        /* retrieve the number of bits for signalling */
                        nBits = (Word16) acelp_sig_tbl[++idx];

                        test();
                        test();
                        IF(sub(st->tcx_cfg.coder_type,VOICED) == 0 ||
                           sub(st->tcx_cfg.coder_type,GENERIC) == 0 ||
                           sub(st->tcx_cfg.coder_type,TRANSITION) == 0)
                        {
                            st->sharpFlag=1;
                        }
                        ELSE
                        {
                            st->sharpFlag=0;
                        }

                        /* retrieve the signalling index */
                        idx = add(idx,1);
                        start_idx = idx;
                        WHILE( L_sub(acelp_sig_tbl[idx], SIG2IND(st->tcx_cfg.coder_type, st->bwidth_fx, st->sharpFlag, st->rf_mode))!= 0 )
                        {
                            idx = add(idx,1);
                        }
                        push_next_indice_fx( st, idx - start_idx, nBits);
                        push_next_indice_fx( st, 1, 1); /* Indicate to the decoder that the core is TCX*/
                        nbits_start = 3;
                    }
                    ELSE
                    {
                        push_next_indice_fx(st, 4+st->tcx_cfg.coder_type, 3 );
                    }
                }
            }
        }
    }

    /* Encode previous mode for error concealment */
    test();
    IF (!(core==ACELP_CORE&&st->tcx_cfg.lfacNext<=0))
    {
        tmp = 0;
        move16();
        if( sub(last_core, ACELP_CORE) > 0 )
        {
            tmp = 1;
            move16();
        }
        push_next_indice_fx(st, tmp, 1);
    }

    /* write TCX overlap mode (1 bit: full, 2 bits: half or no overlap) */
    IF ( core!=ACELP_CORE )
    {
        Word16 overlap_code;
        assert(st->tcx_cfg.tcx_curr_overlap_mode != NOT_SUPPORTED && st->tcx_cfg.tcx_curr_overlap_mode <= ALDO_WINDOW && st->tcx_cfg.tcx_curr_overlap_mode >= FULL_OVERLAP);  /*1 is not allowed!*/
        IF (sub(st->tcx_cfg.tcx_curr_overlap_mode, MIN_OVERLAP) == 0)
        {
            nbits_tcx = 2;
            move16();
            overlap_code = 2;
            move16();
        }
        ELSE IF (sub(st->tcx_cfg.tcx_curr_overlap_mode, HALF_OVERLAP) == 0)
        {
            nbits_tcx = 2;
            move16();
            overlap_code = 3;
            move16();
        }
        ELSE
        {
            nbits_tcx = 1;
            move16();
            overlap_code = 0;
            move16();
        }
        push_next_indice_fx(st, overlap_code, nbits_tcx);
    }

    IF( st->plcExt.enableGplc )
    {
        /* encode side information. */
        enc_prm_side_Info( &st->plcExt, st );
    }

    IF( st->glr != 0)
    {
        test();
        test();
        test();
        test();
        if (core != ACELP_CORE || sub(coder_type,INACTIVE) == 0 || (st->last_core_fx == ACELP_CORE && sub(st->last_coder_type_raw_fx, INACTIVE) == 0) || st->glr_reset != 0)
        {
            st->glr_idx[0] = 0;
            move16();
        }

        IF( sub(core,ACELP_CORE) == 0 )
        {
            push_next_indice_fx(st, st->glr_idx[0], G_LPC_RECOVERY_BITS);
        }
    }

    st->glr_reset = 0;
    move16();

    nbits_header = sub(st->nb_bits_tot_fx, nbits_start);


    /*--------------------------------------------------------------------------------*
     * LPC PARAMETERS
     *--------------------------------------------------------------------------------*/

    IF ( s_and(st->enableTcxLpc!=0, core != ACELP_CORE) )
    {
        /* Encode the indices */
        nbits_lpc = enc_lsf_tcxlpc(&param_lpc, st);
    }
    ELSE
    {
        IF (st->lpcQuantization == 0)
        {
            /* LPC quantizer */
            numlpc = 2;
            move16();
            if(sub(core, TCX_20_CORE) == 0)
            {
                numlpc = 1;
                move16();
            }

            nbits_lpc = encode_lpc_avq(st, numlpc, param_lpc, core);
        }
        ELSE IF (sub(st->lpcQuantization, 1)==0)
        {
            test();
            test();
            IF(L_sub(st->sr_core, 16000)==0 && sub(coder_type, VOICED) == 0 && sub(core, ACELP_CORE) == 0)
            {
                nbits_lpc = lsf_bctcvq_encprm(st, param_lpc, bits_param_lpc, no_param_lpc);
            }
            ELSE
            {
                nbits_lpc = lsf_msvq_ma_encprm(st, param_lpc, core, coder_type, st->acelp_cfg.midLpc, bits_param_lpc, no_param_lpc );
            }
        }
        ELSE
        {
            assert(0 && "LPC quant not supported!");
        }
    }


    /*--------------------------------------------------------------------------------*
     * PRINT BIT ALLOCATION
     *--------------------------------------------------------------------------------*/



    /*--------------------------------------------------------------------------------*
     * ACELP
     *--------------------------------------------------------------------------------*/

    IF (core == ACELP_CORE)
    {
        /* Adaptive BPF (2 bits)*/
        n = ACELP_BPF_BITS[st->acelp_cfg.bpf_mode];

        IF(n!=0)
        {
            push_next_indice_fx(st, st->bpf_gain_param, n);
        }

        /* Mean energy (2 or 3 bits) */
        n = ACELP_NRG_BITS[st->acelp_cfg.nrg_mode];

        IF(n!=0)
        {
            push_next_indice_fx(st, prm[j++], n);
        }

        /*ToDo : write frame-wise parameters about PPC_VC*/

        /* Subframe parameters */

        FOR (sfr=0; sfr<nb_subfr; sfr++)
        {
            /* Pitch lag (4, 5, 6, 8 or 9 bits) */
            move16();
            n = ACELP_LTP_BITS_SFR[st->acelp_cfg.ltp_mode][sfr];

            IF (n!=0)
            {
                push_next_indice_fx(st, prm[j++], n);
            }

            /* Adaptive codebook filtering (1 bit) */

            IF(sub(st->acelp_cfg.ltf_mode,2)==0)
            {
                push_next_indice_fx(st, prm[j++], 1);
            }

            /*Innovative codebook*/
            {
                move16();
                j_old = j;

                if ((st->acelp_cfg.fixed_cdk_index[sfr] >= ACELP_FIXED_CDK_NB) || (st->acelp_cfg.fixed_cdk_index[sfr] < 0))
                {
                    fprintf(stderr,"ACELP bits allocation: wrong fixed cdk bit allocation\n");
                    assert(0);
                }


                wordcnt = shr(ACELP_FIXED_CDK_BITS(st->acelp_cfg.fixed_cdk_index[sfr]), 4);

                bitcnt = s_and(ACELP_FIXED_CDK_BITS(st->acelp_cfg.fixed_cdk_index[sfr]), 15);


                FOR (ix = 0; ix < wordcnt; ix++)
                {
                    push_next_indice_fx(st, prm[j++], 16);
                }

                IF (bitcnt)
                {
                    push_next_indice_fx(st, prm[j++], bitcnt);
                }

                j = add(j_old, 8);
            }

            /* Gains (5b, 6b or 7b / subfr) */
            n = ACELP_GAINS_BITS[st->acelp_cfg.gains_mode[sfr]];
            push_next_indice_fx(st, prm[j++], n);
        }/*end of for(sfr)*/
    }/*end of mode[0]==0*/


    /*--------------------------------------------------------------------------------*
     * TCX20
     *--------------------------------------------------------------------------------*/
    IF ( s_or(sub(core,TCX_20_CORE) == 0, sub(core,HQ_CORE) == 0) )
    {
        flag_ctx_hm = 0;
        move16();
        IF(st->enablePlcWaveadjust)
        {
            Word16 *index = NULL;
            index = &st->Tonal_SideInfo;
            push_next_indice_fx(st, *index, 1);
        }

        /* TCX Gain = 7 bits */
        push_next_indice_fx(st, prm[j++], 7);

        /* TCX Noise Filling = NBITS_NOISE_FILL_LEVEL bits */
        push_next_indice_fx(st, prm[j++], NBITS_NOISE_FILL_LEVEL);

        /* LTP data */
        test();
        IF ( st->tcxltp || L_sub(st->sr_core, 25600) > 0)
        {
            IF ( prm[j] )
            {
                push_next_indice_fx(st, 1       , 1);
                push_next_indice_fx(st, prm[j+1], 9);
                push_next_indice_fx(st, prm[j+2], 2);
            }
            ELSE
            {
                push_next_indice_fx(st, 0       , 1);
            }
        }
        j = add(j, 3);

        /* TCX spectral data */
        lg = L_frame;
        move16();
        lgFB = st->tcx_cfg.tcx_coded_lines;
        move16();

        IF ( last_core==ACELP_CORE )
        {
            /* ACE->TCX transition */
            lg = add(lg, st->tcx_cfg.tcx_offset);
            lgFB = add(lgFB, shr(lgFB, 2));
            if(st->tcx_cfg.lfacNext<0)
            {
                lg = sub(lg,st->tcx_cfg.lfacNext);
            }
        }

        /* TNS data */
        nTnsParams = 0;
        move16();
        nTnsBits = 0;
        move16();

        IF (st->tcx_cfg.fIsTNSAllowed)
        {
            WriteTnsData(st->tcx_cfg.pCurrentTnsConfig, prm+j, &nTnsParams, st, &nTnsBits);
            j = add(j, nTnsParams);
        }

        hm_size = shl(mult(st->tcx_cfg.bandwidth, lg), 1);
        test();
        IF ( st->tcx_lpc_shaped_ari && sub(last_core, ACELP_CORE) != 0 )
        {
            enc_prm_hm(&prm[j], st, hm_size);
        }

        /*Context HM flag*/
        test();
        IF ( st->tcx_cfg.ctx_hm && sub(last_core, ACELP_CORE) != 0 )
        {
            push_next_indice_fx(st, prm[j], 1);

            IF (prm[j])
            {
                EncodeIndex(sub(hm_size,256) >= 0, prm[j+1], st);
                flag_ctx_hm = 1;
                move16();
            }
        }
        j = add(j, NPRM_CTX_HM);

        IF (st->igf)
        {
            st->hIGFEnc.infoTotalBitsPerFrameWritten = 0;
            move16();
            IF (sub(st->last_core_fx, ACELP_CORE) == 0)
            {
                IGFEncWriteBitstream( &st->hIGFEnc, st, &st->hIGFEnc.infoTotalBitsPerFrameWritten,
                                      IGF_GRID_LB_TRAN, 1 );
            }
            ELSE
            {
                IGFEncWriteBitstream( &st->hIGFEnc, st, &st->hIGFEnc.infoTotalBitsPerFrameWritten,
                IGF_GRID_LB_NORM, 1 );
            }
        }
        total_nbbits = sub(st->nb_bits_tot_fx, nbits_start);
        if(sub(st->rf_mode,1)==0)
        {
            total_nbbits = add(total_nbbits,st->rf_target_bits_write);
        }
        nbits_tcx = sub(st->bits_frame_core, total_nbbits);

        IF (st->tcx_lpc_shaped_ari != 0)
        {
            push_next_bits_fx(st, &prm[++j], nbits_tcx);
            j = add(j, nbits_tcx);
        }
        ELSE
        {
            ACcontextMapping_encode2_no_mem_s17_LC(st, prm+j,
            lgFB,
            prm[j-1], /* lastnz */
            nbits_tcx, NPRM_RESQ * st->tcx_cfg.resq, flag_ctx_hm ? hm_cfg : NULL);
        }

    }


    /*--------------------------------------------------------------------------------*
     * TCX10
     *--------------------------------------------------------------------------------*/


    IF (sub(core,TCX_10_CORE) == 0)
    {
        Word16 nbits_igf = 0;
        move16();
        IF (st->igf)
        {
            nbits_igf = IGFEncWriteConcatenatedBitstream( &st->hIGFEnc, st );
        }
        FOR (k = 0; k < 2; k++)
        {
            flag_ctx_hm = 0;
            move16();

            move16();
            move16();
            prm = param + (k*NPRM_DIV);

            j = 0;

            move16();
            nbits_tcx = total_nbbits = sub(st->nb_bits_tot_fx, nbits_start);

            test();
            IF(st->enablePlcWaveadjust && k)
            {
                Word16* index = NULL;
                index = &st->Tonal_SideInfo;
                push_next_indice_fx(st, *index, 1);
            }

            /* TCX Gain = 7 bits */
            push_next_indice_fx(st, prm[j++], 7);

            /* TCX Noise Filling = NBITS_NOISE_FILL_LEVEL bits */
            push_next_indice_fx(st, prm[j++], NBITS_NOISE_FILL_LEVEL);

            /* LTP data */
            test();
            test();
            IF (  (k == 0) && (st->tcxltp!=0 || L_sub(st->sr_core, 25600) > 0 ) )/* PLC pitch info for HB */
            {
                IF ( prm[j] )
                {
                    push_next_indice_fx(st, 1       , 1);
                    push_next_indice_fx(st, prm[j+1], 9);
                    push_next_indice_fx(st, prm[j+2], 2);
                }
                ELSE
                {
                    push_next_indice_fx(st, 0       , 1);
                }
            }
            j = add(j, 3);

            /* TCX spectral data */
            lg = shr(L_frame, 1);
            lgFB = shr(st->tcx_cfg.tcx_coded_lines, 1);

            IF ( s_and(k==0, last_core==ACELP_CORE) )
            {
                /* ACE->TCX transition */
                lg = add(lg, st->tcx_cfg.tcx_offset);
                lgFB = add(lgFB, shr(lgFB, 1));
                if(st->tcx_cfg.lfacNext<0)
                {
                    lg = sub(lg,st->tcx_cfg.lfacNext);
                }
            }

            /* TNS data */
            move16();
            move16();
            nTnsParams = 0;
            nTnsBits = 0;

            IF (st->tcx_cfg.fIsTNSAllowed)
            {

                SetTnsConfig(&st->tcx_cfg, 0, (last_core == ACELP_CORE) && (k == 0));

                WriteTnsData(st->tcx_cfg.pCurrentTnsConfig, prm+j, &nTnsParams, st, &nTnsBits);
                j = add(j, nTnsParams);
            }

            hm_size = shl(mult(st->tcx_cfg.bandwidth, lgFB), 1);

            /*Context HM flag*/
            test();
            test();
            IF ( st->tcx_cfg.ctx_hm && !(last_core == ACELP_CORE && k == 0) )
            {
                push_next_indice_fx(st, prm[j], 1);

                IF (prm[j])
                {
                    EncodeIndex(sub(hm_size,256) >= 0, prm[j+1], st);
                    flag_ctx_hm = 1;
                    move16();
                }
            }
            j = add(j, NPRM_CTX_HM);

            total_nbbits = sub(st->nb_bits_tot_fx, nbits_start);

            nbits_tcx = sub(shr(sub(add(sub(sub(sub(st->bits_frame_core, nbits_header), nbits_lpc), nbits_igf), 1), k), 1), sub(total_nbbits, nbits_tcx));

            ACcontextMapping_encode2_no_mem_s17_LC(st, prm+j,
                                                   lgFB,
                                                   prm[j-1], /* lastnz */
                                                   nbits_tcx, NPRM_RESQ * st->tcx_cfg.resq, flag_ctx_hm ? &hm_cfg[k] : NULL);

        } /* k, window index */
    }


    /*--------------------------------------------------------------------------------*
     * END
     *--------------------------------------------------------------------------------*/


    total_nbbits = sub(st->nb_bits_tot_fx, nbits_start);


    /* Check if total encoded bits does not exceed CBR target bits (->this must never happen) */
    if (st->bits_frame_core&&(total_nbbits>st->bits_frame_core))
    {
        fprintf(stderr,"AllocatedBits: %d Used bits: %d \n", st->bits_frame_core,total_nbbits);
        assert(!"Core totalbits > CBR target bitrate");
    }

    return;
}

