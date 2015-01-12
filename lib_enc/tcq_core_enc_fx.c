/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"
#include "stl.h"

void tcq_core_LR_enc_fx(
    Encoder_State_fx* st_fx,
    Word16   inp_vector_fx[],		/* x5 */
    const Word32 coefs_norm_fx[],		/* Q12 */
    Word32 coefs_quant_fx[],		/* Q12 */
    const Word16 bit_budget,     /* number of bits */
    const Word16 BANDS,
    const Word16 *sfm_start,
    const Word16 *sfm_end,
    const Word16 *sfmsize,
    Word32 *R_fx,					/* Q16 */
    Word16 *npulses,
    Word16 *k_sort,
    const Word16 *p2a_flags,
    const Word16 p2a_bands,
    const Word16 *last_bitalloc,
    const Word16 input_frame,
    const Word16 adjustFlag,
    const Word16 is_transient
)
{
    Word16 i, j, k, size, nb_bytes;

    Word16 USQ_TCQ[NB_SFM];              /* TCQ is selected by default*/
    Word16 coefs_norm_dec_fx[L_FRAME32k]; /* New output buffer (TCQ+USQ)*/
    Word32 savedstates[TCQ_MAX_BAND_SIZE];
    ARCODEC_FX arenc_fx, *parenc_fx;
    BITSTREAM_FX bs_fx, *pbs_fx;
    Word16 k_num[2];
    Word32 bit_surplus_fx[2];

    Word16 flag_wbnb = 0;
    Word16 lsbtcq_bits = TCQ_AMP;
    Word16 tcq_arbits  = 2;
    Word16 nzb = 0;
    Word16 nzbands = 0;
    Word16 bcount = 0;
    Word32 bsub_fx = 0;
    Word32 abuffer_fx[MAX_PULSES];
    Word16 mbuffer_fx[MAX_PULSES];
    Word32 sbuffer_fx[MAX_PULSES];
    Word16 dpath[280];
    Word32 Rk_sort_fx[NB_SFM];
    Word32 step_scale_fx[NB_SFM];
    Word16 pulses_fx, nzp_fx;

    Word32 gain_fx, crosscorr_fx, selfcorr_fx;
    Word16 hi, lo, exp;
    Word32 surplus_fx, delta_fx, est_frame_bits_fx;

    set16_fx(dpath, 0, 280);
    set32_fx(abuffer_fx, 0, MAX_PULSES);
    set32_fx(sbuffer_fx, 0, MAX_PULSES);
    set16_fx(mbuffer_fx, 0, MAX_PULSES);
    /* initialization */
    set32_fx(Rk_sort_fx, 0, NB_SFM);
    set16_fx(USQ_TCQ, 0, NB_SFM);
    set16_fx(coefs_norm_dec_fx, 0, L_FRAME32k);


    InitLSBTCQ_fx(&bcount);

    test();
    test();
    IF( input_frame <= L_FRAME16k  && adjustFlag == 0 && is_transient == 0 )
    {
        flag_wbnb = 1;
        move16();
        lsbtcq_bits = 0;
        move16();
        tcq_arbits  = 0;
        move16();
    }

    /* TCQ Index initialize */
    parenc_fx = &arenc_fx;
    pbs_fx = &bs_fx;

    pbs_fx->curPos = 7;
    move16();
    pbs_fx->numbits = L_deposit_l(0);
    pbs_fx->numByte = L_deposit_l(0);
    FOR (i = 0; i < MAX_SIZEBUF_PBITSTREAM; i++)
    {
        pbs_fx->buf[i] = 0;
        move16();
    }
    ar_encoder_start_fx(parenc_fx, pbs_fx);

    /* Bits distribution analysis */
    FOR ( i = 0; i < BANDS; i++ )
    {
        IF ( L_sub(ar_div(R_fx[i], sfmsize[i]), 49152) >= 0 )
        {
            /* USQ used for high importance bands*/
            USQ_TCQ[i] = 1;
            move16();
        }
        ELSE
        {
            /* TCQ used for usual bands */
            USQ_TCQ[i] = 0;
            move16();
        }

        IF( R_fx[i] > 0 )
        {
            /* nzbands++; */
            nzbands = add(nzbands, 1);
        }
    }

    FOR ( j = 0; j < BANDS; j++ )
    {
        IF ( R_fx[j] > 0 )
        {
            nzb++;
        }
    }

    bsub_fx = L_shl(add(tcq_arbits, lsbtcq_bits), 16); /* Q16 */
    IF( bsub_fx > 0)
    {
        bsub_fx = L_add( bsub_fx,  2048);
    }
    FOR ( j = BANDS - 1; j >= 0; j-- )
    {
        IF( R_fx[j] > 0 )
        {
            R_fx[j] = L_sub(R_fx[j], ar_div(bsub_fx, nzb));

            IF( R_fx[j] < 0)
            {
                bsub_fx = L_sub(bsub_fx, L_add(ar_div(bsub_fx, nzb), R_fx[j]));
                R_fx[j] = L_deposit_l(0);
            }
            ELSE
            {
                bsub_fx = L_sub(bsub_fx, ar_div(bsub_fx, nzb));
            }
            /* nzb--; */
            nzb = sub(nzb, 1);
        }
    }

    /* Sort the bit allocation table (R) in ascending order, figure out number of pulses per band */
    srt_vec_ind_fx(R_fx, Rk_sort_fx, k_sort, BANDS);

    /* Quantize spectral band shapes using TCQ */
    /* Select ISC */
    set32_fx(coefs_quant_fx, 0, sfm_end[BANDS - 1]+1);
    Copy32( coefs_norm_fx, coefs_quant_fx,sfm_end[BANDS-1]+1);

    delta_fx = L_deposit_l(0);
    est_frame_bits_fx = L_deposit_l(0);

    test();
    test();
    IF ( sub(input_frame, L_FRAME16k) <= 0  && adjustFlag == 0 && is_transient == 0 )
    {
        surplus_fx = -131072;
        move32();
        bit_allocation_second_fx( R_fx, Rk_sort_fx, BANDS, sfmsize, k_sort, k_num, p2a_flags, p2a_bands, last_bitalloc, input_frame );

        /* Separate the position information from the input signal(coefs_norm) */
        /* Gather the NZ coefficients*/
        FOR ( k = 0; k < BANDS; k++) /* Loop through non-zero blocks  */
        {
            test();
            IF ( sub(k, k_num[0]) != 0 && sub(k, k_num[1]) != 0)
            {
                test();
                test();
                IF ( R_fx[k_sort[k]] > 0 && USQ_TCQ[k_sort[k]] == 0 ) /* Then have non-zero block AND WILL BE ENCODED BY TCQ */
                {
                    /* Encode Position Info, NZ Info, Signs */
                    size = sfmsize[k_sort[k]];
                    move16();

                    /* Determine scale step, ISC and TCQ quantizer */
                    GetISCScale_fx( &coefs_quant_fx[sfm_start[k_sort[k]]], size,
                                    L_add( R_fx[k_sort[k]], delta_fx),
                                    /* R_fx[k_sort[k]], */
                                    &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], &step_scale_fx[k_sort[k]], &surplus_fx, &pulses_fx, savedstates, 0, &nzp_fx
                                    , 0, 0, 0, 0
                                  );

                    npulses[ k_sort[k]] = pulses_fx;
                    move16();

                    encode_position_ari_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, &est_frame_bits_fx );
                    encode_magnitude_tcq_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, npulses[k_sort[k]], nzp_fx, savedstates, &est_frame_bits_fx );
                    encode_signs_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, nzp_fx, &est_frame_bits_fx );
                    nzbands--;
                }
                /* Have USQ coded band */
                ELSE IF( R_fx[k_sort[k]] > 0 && sub(USQ_TCQ[k_sort[k]], 1) == 0 )
                {
                    size = sfmsize[k_sort[k]];
                    move16();

                    GetISCScale_fx( &coefs_quant_fx[ sfm_start[ k_sort[k]]], size,
                                    L_add( R_fx[k_sort[k]], delta_fx),
                                    /* R_fx[k_sort[k]], */
                                    &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], &step_scale_fx[k_sort[k]], &surplus_fx, &pulses_fx, savedstates, 1, &nzp_fx
                                    , 0, 0, 0, 0
                                  );

                    npulses[ k_sort[k]] = pulses_fx;
                    move16();

                    encode_position_ari_fx( parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, &est_frame_bits_fx );
                    encode_magnitude_usq_fx( parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, npulses[k_sort[k]], nzp_fx, &est_frame_bits_fx );
                    encode_signs_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[ k]]], size, nzp_fx, &est_frame_bits_fx );
                    nzbands--;
                }
                ELSE /* Then have  zero block  */
                {
                    npulses[ k_sort[ k]] = 0;
                    move16();
                    size = sfmsize[k_sort[k]];
                    move16();
                }

                delta_fx = L_deposit_l(0);
                test();
                IF( R_fx[k_sort[k]] > 0 && surplus_fx < 0 )
                {
                    /* delta_fx   = L_deposit_h( div_l( surplus_fx, nzbands ) ); */
                    delta_fx = ar_div( surplus_fx, nzbands);
                    surplus_fx = L_sub(surplus_fx, delta_fx);
                }
            }
        }

        test();
        test();
        test();
        IF (( L_sub(surplus_fx,524288) > 0 && sub(input_frame,L_FRAME8k) == 0 ) || ( L_sub(surplus_fx,786432) > 0 && sub(input_frame,L_FRAME16k) == 0 ))
        {
            bit_surplus_fx[0] = Mult_32_16(surplus_fx,24576); /* Q16 */
            bit_surplus_fx[1] = Mult_32_16(surplus_fx,8192);  /* Q16 */
        }
        ELSE
        {
            bit_surplus_fx[0] = surplus_fx;
            move32();
            bit_surplus_fx[1] = L_deposit_l(0);
        }

        FOR ( k = 0; k < BANDS; k++ )
        {
            FOR ( j = 0; j < 2; j++ )
            {
                IF ( sub(k, k_num[j]) == 0 )
                {
                    R_fx[k_sort[k]] = L_add(R_fx[k_sort[k]],bit_surplus_fx[j]);

                    test();
                    test();
                    IF ( R_fx[k_sort[k]] > 0 && USQ_TCQ[k_sort[k]] == 0 ) /* Then have non-zero block AND WILL BE ENCODED BY TCQ */
                    {
                        /* Encode Position Info, NZ Info, Signs */
                        size = sfmsize[k_sort[k]];
                        move16();

                        /* Determine scale step, ISC and TCQ quantizer */
                        GetISCScale_fx( &coefs_quant_fx[sfm_start[k_sort[k]]], size, R_fx[k_sort[k]], &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], &step_scale_fx[k_sort[k]], &surplus_fx, &pulses_fx, savedstates, 0, &nzp_fx
                                        , 0, 0, 0, 0
                                      );

                        npulses[ k_sort[k]] = pulses_fx;
                        move16();

                        encode_position_ari_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, &est_frame_bits_fx );
                        encode_magnitude_tcq_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, npulses[k_sort[k]], nzp_fx, savedstates, &est_frame_bits_fx );
                        encode_signs_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, nzp_fx, &est_frame_bits_fx );
                    }
                    /* Have USQ coded band */
                    ELSE IF( R_fx[k_sort[k]] > 0 && sub(USQ_TCQ[k_sort[k]], 1) == 0 )
                    {
                        size = sfmsize[k_sort[k]];
                        move16();

                        GetISCScale_fx( &coefs_quant_fx[ sfm_start[ k_sort[k]]], size, R_fx[k_sort[k]], &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], &step_scale_fx[k_sort[k]], &surplus_fx, &pulses_fx, savedstates, 1, &nzp_fx
                                        , 0, 0, 0, 0
                                      );

                        npulses[ k_sort[k]] = pulses_fx;
                        move16();

                        encode_position_ari_fx( parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, &est_frame_bits_fx );
                        encode_magnitude_usq_fx( parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, npulses[k_sort[k]], nzp_fx, &est_frame_bits_fx );
                        encode_signs_fx( parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, nzp_fx, &est_frame_bits_fx );
                    }
                    ELSE /* Then have  zero block */
                    {
                        npulses[ k_sort[k]] = 0;
                        move16();
                        size = sfmsize[k_sort[k]];
                        move16();
                    }
                }
            }
        }
    }
    ELSE
    {
        surplus_fx = L_deposit_l(0);

        /* Separate the position information from the input signal(coefs_norm) */
        /* Gather the NZ coefficients*/
        FOR( k = 0; k < BANDS; k++) /* Loop through non-zero blocks  */
        {
            IF( R_fx[k_sort[k]] > 0 )
            {
                size = sfmsize[k_sort[k]];
                move16();
                GetISCScale_fx( &coefs_quant_fx[ sfm_start[ k_sort[k]]], size, R_fx[k_sort[k]] + delta_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], &step_scale_fx[k_sort[k]], &surplus_fx, &pulses_fx, savedstates, 1, &nzp_fx, &bcount, abuffer_fx, mbuffer_fx, sbuffer_fx);

                npulses[ k_sort[k]] = pulses_fx;
                move16();
                encode_position_ari_fx(parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, &est_frame_bits_fx);
                encode_magnitude_usq_fx(parenc_fx, &coefs_norm_dec_fx[sfm_start[k_sort[k]]], size, npulses[k_sort[k]], nzp_fx, &est_frame_bits_fx);
                encode_signs_fx(parenc_fx, &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, nzp_fx, &est_frame_bits_fx);

                /* nzbands--;  */
                nzbands = sub(nzbands, 1);
            }
            ELSE /* Then have zero block  */
            {
                npulses[ k_sort[k]] = 0;
                move16();
                size = sfmsize[k_sort[k]];
                move16();
            }

            /* Surplus distribution */
            /* if( surplus > 0.0f && nzbands > 0 ) */
            test();
            IF( surplus_fx > 0 && nzbands > 0 )
            {
                /* delta = surplus / nzbands;
                   surplus -= delta; */

                delta_fx = ar_div(surplus_fx, nzbands);
                surplus_fx = L_sub(surplus_fx, delta_fx);
            }
        }
    }

    TCQLSB_fx(bcount, /*abuffer, */abuffer_fx, /*mbuffer, */mbuffer_fx, /*sbuffer, */sbuffer_fx, dpath);

    /* Save TCQ path to bitstream */
    SaveTCQdata_fx(parenc_fx, dpath, lsbtcq_bits);

    /* Add tcq sequence to decoding buffer */
    InitLSBTCQ_fx(&bcount);

    ar_encoder_done_fx(parenc_fx);

    /* Loop through non-zero blocks   */
    FOR (i = 0; i < L_FRAME32k; i++)
    {
        coefs_norm_dec_fx[i] = extract_l(L_mult0(coefs_norm_dec_fx[i], 5));
    }
    IF( !flag_wbnb )
    {
        FOR ( k = 0; k < BANDS; k++)
        {
            IF( R_fx[k_sort[k]] > 0 )
            {
                size = sfmsize[k_sort[k]];
                move16();
                RestoreTCQ_fx( &coefs_norm_dec_fx[ sfm_start[ k_sort[k]]], size, &bcount, mbuffer_fx );
            }
        }
    }

    nb_bytes = shr(bit_budget, 3);
    j = sub(bit_budget, shl(nb_bytes, 3));
    FOR( i = 0; i < nb_bytes; i++ )
    {
        push_indice_fx(st_fx, IND_HQ2_SUBBAND_FPC, pbs_fx->buf[i], 8);
    }
    IF( j > 0 )
    {
        push_indice_fx(st_fx, IND_HQ2_SUBBAND_FPC, (pbs_fx->buf[nb_bytes] >> (8 - j)), j);
    }

    /* Clear decoding buffer */
    set32_fx(coefs_quant_fx, 0, sfm_end[BANDS-1]+1);

    /* New analysis of decoded frame */
    FOR ( i = 0; i < BANDS; i++ )
    {
        IF ( R_fx[ k_sort[i]] > 0 )
        {
            gain_fx = L_deposit_l(0);

            crosscorr_fx = L_deposit_l(0);
            selfcorr_fx = L_deposit_l(0);

            FOR ( j = 0; j < sfmsize[k_sort[i]]; j++ )
            {
                crosscorr_fx = L_add( crosscorr_fx, Mult_32_16( coefs_norm_fx[sfm_start[k_sort[ i]]+j], shl(coefs_norm_dec_fx[sfm_start[k_sort[ i]]+j], 2) ) );/*1 */
                selfcorr_fx = L_mac0(selfcorr_fx, coefs_norm_dec_fx[sfm_start[k_sort[ i]]+j], coefs_norm_dec_fx[sfm_start[k_sort[ i]]+j]);
            }

            exp =sub(norm_l(crosscorr_fx), 1);
            gain_fx = ar_div( L_shl(crosscorr_fx, exp), selfcorr_fx); /* 1 + exp */
            gain_fx = L_shl( gain_fx, sub( 16, 1 + exp) + 2);/* 0.2 * Q16 */
            lo = L_Extract_lc(gain_fx, &hi);
            /* Use optimal gain */
            FOR ( j = 0; j < sfmsize[k_sort[i]]; j++ )
            {
                inp_vector_fx[sfm_start[k_sort[i]]+j] = coefs_norm_dec_fx[sfm_start[k_sort[i]]+j];
                move16();
                coefs_quant_fx[sfm_start[k_sort[ i]]+j] = L_add(L_shl(L_mult0(hi, coefs_norm_dec_fx[sfm_start[k_sort[ i]]+j]), 12),
                        L_shr(L_mult0(lo, coefs_norm_dec_fx[sfm_start[k_sort[ i]]+j]), 3)); /* Q12 */
            }
        }
    }



    return;

}
