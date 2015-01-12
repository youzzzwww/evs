/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */

#include "prot_fx.h"
#include "stl.h"                /* required for wmc_tool */

void tcq_core_LR_dec_fx(
    Decoder_State_fx *st_fx,
    Word16	  *inp_vector_fx,		/* x5 */
    const Word16 bit_budget,
    const Word16 BANDS,
    const Word16 *band_start,
    const Word16 *band_width,
    Word32 *Rk_fx,					/* Q16 */
    Word16 *npulses,
    Word16 *k_sort,
    const Word16 *p2a_flags,
    const Word16 p2a_bands,
    const Word16 *last_bitalloc,
    const Word16 input_frame,
    const Word16 adjustFlag,
    const Word16 *is_transient
)
{
    Word16 i, j, k;
    Word32 Rk_sort_fx[NB_SFM];
    Word16 flag_wbnb = 0;
    Word16 USQ_TCQ[NB_SFM];              /* TCQ is selected by default*/
    Word16 nb_bytes, pulsesnum, nz;

    Word16 positions_fx[L_FRAME48k];
    Word32 surplus_fx, delta_fx;
    Word16 k_num[2];
    Word32 bit_surplus_fx[2];
    ARCODEC_FX ardec_fx, *pardec_fx;
    BITSTREAM_FX bs_fx, *pbs_fx;

    Word16 nzb = 0;
    Word16 nzbands = 0;
    Word16 lsbtcq_bits = TCQ_AMP;
    Word16 tcq_arbits  = 2;

    /* LSB TCQ variables*/
    Word16 dpath[280];
    Word16 bcount = 0;
    Word32 bsub_fx = 0;
    Word16 mbuffer_fx[560];

    if (Overflow == 1)
    {
        Overflow = 0; /* set overflow flag to zero before entering TCQ finctions without any message */
    }

    /* initialization */
    set16_fx(dpath, 0, 280);
    set16_fx( USQ_TCQ, 0, NB_SFM );
    set16_fx(positions_fx, 0, L_FRAME32k);
    set16_fx(mbuffer_fx, 0, 560);

    test();
    test();
    IF ( sub(input_frame, L_FRAME16k) <= 0  && adjustFlag == 0 && *is_transient == 0 )
    {
        flag_wbnb = 1;
        move16();
        lsbtcq_bits = 0;
        move16();
        tcq_arbits  = 0;
        move16();
    }

    pardec_fx = &ardec_fx;
    pbs_fx = &bs_fx;
    pbs_fx->curPos = 7;
    move16();
    pbs_fx->numbits = L_deposit_l(0);
    pbs_fx->numByte = L_deposit_l(0);

    /* Bits distribution analysis*/
    FOR ( i = 0; i < BANDS; i++ )
    {
        IF (L_sub(ar_div(Rk_fx[i], band_width[i]), 49152) >= 0)
        {
            /* USQ used for high importance bands*/
            USQ_TCQ[i] = 1;
            move16();
        }
        ELSE
        {
            /* TCQ used for usual bands*/
            USQ_TCQ[i] = 0;
            move16();
        }
        if ( Rk_fx[i] > 0 )
        {
            nzbands = add(nzbands, 1);
        }
    }

    FOR ( j = 0; j < BANDS; j++ )
    {
        if ( Rk_fx[j] > 0 )
        {
            nzb = add(nzb, 1);
        }
    }

    bsub_fx = L_shl(L_add(tcq_arbits, lsbtcq_bits), 16);
    if( bsub_fx > 0)
    {
        bsub_fx = L_add( bsub_fx,  2048);
    }
    FOR ( j = BANDS - 1; j >= 0; j-- )
    {
        IF ( Rk_fx[j] > 0 )
        {
            Rk_fx[j] = L_sub(Rk_fx[j], ar_div(bsub_fx, nzb));
            IF ( Rk_fx[j] < 0)
            {
                bsub_fx = L_sub(bsub_fx, L_add(ar_div(bsub_fx, nzb), Rk_fx[j]));
                Rk_fx[j] = L_deposit_l(0);
            }
            ELSE
            {
                bsub_fx = L_sub(bsub_fx, ar_div(bsub_fx, nzb));
            }
            nzb = sub(nzb, 1);
        }
    }

    srt_vec_ind_fx(Rk_fx, Rk_sort_fx, k_sort, BANDS);

    /*read the bits*/
    nb_bytes = shr(bit_budget, 3);
    k = sub(bit_budget, shl(nb_bytes, 3));
    FOR ( i = 0; i < nb_bytes; i++ )
    {
        pbs_fx->buf[i] = (UWord8)get_next_indice_fx(st_fx, 8);
        move16();
    }

    IF ( k > 0 )
    {
        pbs_fx->buf[nb_bytes] = (UWord8)get_next_indice_fx(st_fx, k);
        pbs_fx->buf[nb_bytes] <<= (8 - k);
        /* i++;
           nb_bytes++; */
        i = add(i, 1);
        nb_bytes = add(nb_bytes, 1);
    }
    /* set two more bytes, which are used to flush the arithmetic coder, to 0
       -> this avoids reading of uninitialized memory */
    nb_bytes = s_min(add(nb_bytes, 2), MAX_SIZEBUF_PBITSTREAM);
    FOR ( ; i < nb_bytes; i++ )
    {
        pbs_fx->buf[i] = 0;
        move16();
    }

    ar_decoder_start_fx(pardec_fx, pbs_fx);

    delta_fx = L_deposit_l(0);
    surplus_fx = L_deposit_l(0);

    test();
    test();
    IF( sub(input_frame, L_FRAME16k) <= 0  && adjustFlag == 0 && *is_transient == 0 )
    {
        surplus_fx = -131072;
        move32();/*16 */

        bit_allocation_second_fx(Rk_fx, Rk_sort_fx, BANDS, band_width, k_sort, k_num, p2a_flags, p2a_bands, last_bitalloc, input_frame);
        FOR( k = 0; k < BANDS; k++ )
        {
            test();
            IF( sub(k, k_num[0]) != 0 && sub(k, k_num[1]) != 0)
            {
                test();
                test();
                IF (Rk_fx[k_sort[k]] > 0 && USQ_TCQ[k_sort[k]] == 0)
                {
                    IF( L_add( Rk_fx[k_sort[k]], delta_fx) < 0 )
                    {
                        pulsesnum = 0;
                        move16();
                        FOR( i = 0; i < band_width[k_sort[k]]; i++ )
                        {
                            inp_vector_fx[band_start[k_sort[k]] + i] = 0;
                            move16();
                        }
                        IF( surplus_fx != 0 )
                        {
                            /* surplus_fx += (Rk[k_sort[k]] + delta);*/
                            surplus_fx = L_add( Rk_fx[k_sort[k]], surplus_fx);
                            surplus_fx = L_add( delta_fx, surplus_fx);
                        }
                    }
                    ELSE
                    {
                        /*get number of pulses */
                        pulsesnum = GetScale_fx( band_width[k_sort[k]],
                        L_add( Rk_fx[k_sort[k]], delta_fx),
                        &surplus_fx );

                        decode_position_ari_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, &nz, &positions_fx[band_start[k_sort[k]]] );
                        /*decode tcq magniitude and update the surplus bits.*/
                        decode_mangitude_tcq_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, nz, &positions_fx[band_start[k_sort[k]]], &inp_vector_fx[band_start[k_sort[k]]], &surplus_fx );
                        decode_signs_fx( pardec_fx, band_width[k_sort[k]], &inp_vector_fx[band_start[k_sort[k]]] );
                    }
                    nzbands--;
                    move16();
                }
                ELSE IF (Rk_fx[k_sort[k]] > 0 && sub(USQ_TCQ[k_sort[k]], 1) == 0)
                {
                    IF( L_add( Rk_fx[k_sort[k]], delta_fx) < 0 )
                    {
                        pulsesnum = 0;
                        move16();
                        FOR( i = 0; i < band_width[k_sort[k]]; i++ )
                        {
                            inp_vector_fx[band_start[k_sort[k]] + i] = 0;
                            move16();
                        }
                        IF( surplus_fx != 0 )
                        {
                            surplus_fx = L_add( Rk_fx[k_sort[k]], surplus_fx);
                            surplus_fx = L_add( delta_fx, surplus_fx);
                        }
                    }
                    ELSE
                    {
                        pulsesnum = GetScale_fx(band_width[k_sort[k]],
                        L_add( Rk_fx[k_sort[k]], delta_fx),
                        &surplus_fx);

                        decode_position_ari_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, &nz, &positions_fx[band_start[k_sort[k]]] );
                        /*decode usq magnitude and don't need to update surplus bits*/
                        decode_magnitude_usq_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, nz, &positions_fx[band_start[k_sort[k]]], &inp_vector_fx[band_start[k_sort[k]]] );
                        decode_signs_fx( pardec_fx, band_width[k_sort[k]], &inp_vector_fx[band_start[k_sort[k]]] );
                    }
                    nzbands--;
                    move16();
                }
                ELSE
                {
                    pulsesnum = 0;
                    move16();
                    FOR ( i = 0; i < band_width[k_sort[k]]; i++ )
                    {
                        inp_vector_fx[band_start[k_sort[k]] + i] = 0;
                        move16();
                    }
                }

                npulses[k_sort[k]] = pulsesnum;
                move16();

                delta_fx = L_deposit_l(0);
                test();
                IF( Rk_fx[k_sort[k]] > 0 && surplus_fx < 0 )
                {
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
            bit_surplus_fx[0] = Mult_32_16(surplus_fx,24576);/* Q16 */
            bit_surplus_fx[1] = Mult_32_16(surplus_fx,8192);/* Q16 */
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
                    Rk_fx[k_sort[k]] = L_add(Rk_fx[k_sort[k]],bit_surplus_fx[j]);
                    move32();

                    test();
                    test();
                    IF ( Rk_fx[k_sort[k]] > 0 && USQ_TCQ[k_sort[k]] == 0 )
                    {
                        /* get number of pulses */
                        pulsesnum = GetScale_fx( band_width[k_sort[k]], Rk_fx[k_sort[k]], &surplus_fx );

                        decode_position_ari_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, &nz, &positions_fx[band_start[k_sort[k]]] );
                        /* decode tcq magniitude and update the surplus bits. */
                        decode_mangitude_tcq_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, nz, &positions_fx[band_start[k_sort[k]]], &inp_vector_fx[band_start[k_sort[k]]], &surplus_fx );
                        decode_signs_fx( pardec_fx, band_width[k_sort[k]], &inp_vector_fx[band_start[k_sort[k]]] );
                    }
                    ELSE IF( Rk_fx[k_sort[k]] > 0 && USQ_TCQ[k_sort[k]] == 1 )
                    {
                        pulsesnum = GetScale_fx( band_width[k_sort[k]], Rk_fx[k_sort[k]], &surplus_fx );

                        decode_position_ari_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, &nz, &positions_fx[band_start[k_sort[k]]] );
                        /* decode usq magnitude and don't need to update surplus bits */
                        decode_magnitude_usq_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, nz, &positions_fx[band_start[k_sort[k]]], &inp_vector_fx[band_start[k_sort[k]]] );
                        decode_signs_fx( pardec_fx, band_width[k_sort[k]], &inp_vector_fx[band_start[k_sort[k]]] );
                    }
                    ELSE
                    {
                        pulsesnum = 0;
                        move16();
                        FOR ( i = 0; i < band_width[k_sort[k]]; i++ )
                        {
                            inp_vector_fx[band_start[k_sort[k]] + i] = 0;
                            move16();
                        }
                    }

                    npulses[k_sort[k]] = pulsesnum;
                    move16();
                }
            }
        }
    }
    ELSE
    {
        FOR ( k = 0; k < BANDS; k++ )
        {
            IF ( Rk_fx[k_sort[k]] > 0 )
            {
                pulsesnum = GetScale_fx(band_width[k_sort[k]], Rk_fx[k_sort[k]] + delta_fx, &surplus_fx);

                decode_position_ari_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, &nz, &positions_fx[band_start[k_sort[k]]] );

                /*decode usq magnitude and don't need to update surplus bits*/
                decode_magnitude_usq_fx( pardec_fx, band_width[k_sort[k]], pulsesnum, nz, &positions_fx[band_start[k_sort[k]]], &inp_vector_fx[band_start[k_sort[k]]] );
                decode_signs_fx( pardec_fx, band_width[k_sort[k]], &inp_vector_fx[band_start[k_sort[k]]] );

                nzbands = sub(nzbands, 1);
            }
            ELSE
            {
                pulsesnum = 0;
                move16();
                FOR ( i = 0; i < band_width[k_sort[k]]; i++ )
                {
                    inp_vector_fx[band_start[k_sort[k]] + i] = 0;
                    move16();
                }
            }

            npulses[k_sort[k]] = pulsesnum;
            move16();

            /* surplus distribution */
            test();
            IF ( surplus_fx > 0 && nzbands > 0 )
            {
                delta_fx = ar_div(surplus_fx, nzbands);
                surplus_fx = L_sub(surplus_fx, delta_fx);
            }
        }
    }

    /* Load TCQ path from bitstream */
    LoadTCQdata_fx(pardec_fx, dpath, lsbtcq_bits);

    TCQLSBdec_fx(dpath, mbuffer_fx, 2*lsbtcq_bits);

    ar_decoder_done_fx(pardec_fx);

    /* Restore TCQ */
    IF ( !flag_wbnb )
    {
        FOR ( k = 0; k < BANDS; k++)
        {
            IF ( Rk_fx[k_sort[k]] > 0 )
            {
                RestoreTCQdec_fx( &inp_vector_fx[ band_start[ k_sort[ k]]], band_width[k_sort[k]], &bcount, mbuffer_fx );
            }
        }
    }
    ELSE
    {
        FOR( k = 0; k < BANDS; k++)
        {
            test();
            test();
            IF( Rk_fx[k_sort[k]] > 0 && k != k_num[0] && k != k_num[1] )
            {
                RestoreTCQdec_fx( &inp_vector_fx[ band_start[ k_sort[ k]]], band_width[k_sort[k]], &bcount, mbuffer_fx );
            }
        }

        FOR ( k = 0; k < BANDS; k++)
        {
            test();
            test();
            IF ( Rk_fx[k_sort[k]] > 0 && (k == k_num[0] || k == k_num[1]) )
            {
                RestoreTCQdec_fx( &inp_vector_fx[ band_start[ k_sort[ k]]], band_width[k_sort[k]], &bcount, mbuffer_fx );
            }
        }
    }



    return;
}
