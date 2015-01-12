/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "stl.h"
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "options.h"
#include "basop_util.h"
#include "rom_com_fx.h"


/*-------------------------------------------------------------------*
 * push_indice_fx( )
 *
 * Push a new indice into the buffer
 *-------------------------------------------------------------------*/

void push_indice_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    Word16 id,           /* i  : ID of the indice */
    UWord16 value,        /* i  : value of the quantized indice */
    Word16 nb_bits       /* i  : number of bits used to quantize the indice */
)
{
    Word16 i;


    IF ( sub(st_fx->last_ind_fx, id) == 0 )
    {
        /* indice with the same name as the previous one */
        i = st_fx->next_ind_fx;
    }
    ELSE
    {
        /* new indice - find an empty slot in the list */
        i = id;
        move16();
        WHILE (sub(st_fx->ind_list_fx[i].nb_bits, -1) != 0)
        {
            i = add(i, 1);
        }
    }

    /* store the values in the list */
    st_fx->ind_list_fx[i].value = value;
    move16();
    st_fx->ind_list_fx[i].nb_bits = nb_bits;
    move16();

    /* updates */
    st_fx->next_ind_fx = add(i, 1);
    st_fx->last_ind_fx = id;
    move16();
    st_fx->nb_bits_tot_fx = add(st_fx->nb_bits_tot_fx, nb_bits);

    return;
}

/*-------------------------------------------------------------------*
 * push_next_indice_fx()            *
 * Push a new indice into the buffer at the next position
 *-------------------------------------------------------------------*/

void push_next_indice_fx(
    Encoder_State_fx *st_fx,      /* i/o: encoder state structure */
    UWord16 value,       /* i  : value of the quantized indice */
    Word16 nb_bits      /* i  : number of bits used to quantize the indice */
)
{

    /* store the values in the list */
    st_fx->ind_list_fx[st_fx->next_ind_fx].value   = value;
    move16();
    st_fx->ind_list_fx[st_fx->next_ind_fx].nb_bits = nb_bits;
    move16();
    st_fx->next_ind_fx = add(st_fx->next_ind_fx, 1);


    /* update the total number of bits already written */
    st_fx->nb_bits_tot_fx = add(st_fx->nb_bits_tot_fx, nb_bits);

    return;
}


/*-------------------------------------------------------------------*
 * push_next_bits_fx()
 * Push a bit buffer into the buffer at the next position
 *-------------------------------------------------------------------*/

void push_next_bits_fx(
    Encoder_State_fx *st_fx,    /* i/o: encoder state structure */
    Word16 bits[],      /* i  : bit buffer to pack, sequence of single bits */
    Word16 nb_bits      /* i  : number of bits to pack */
)
{
    UWord16 code;
    Word16 i, nb_bits_m15;
    Indice_fx *ptr;

    ptr = &st_fx->ind_list_fx[st_fx->next_ind_fx];
    nb_bits_m15 = sub(nb_bits, 15);
    i = 0;
    move16();
    IF (nb_bits_m15 > 0)
    {
        FOR (; i<nb_bits_m15; i += 16)
        {
            code = s_or(lshl(bits[i], 15), s_or(lshl(bits[i+1], 14), s_or(lshl(bits[i+2], 13), s_or(lshl(bits[i+3], 12),
                                                s_or(lshl(bits[i+4], 11), s_or(lshl(bits[i+5], 10), s_or(lshl(bits[i+6], 9), s_or(lshl(bits[i+7], 8),
                                                        s_or(lshl(bits[i+8], 7), s_or(lshl(bits[i+9], 6), s_or(lshl(bits[i+10], 5), s_or(lshl(bits[i+11], 4),
                                                                s_or(lshl(bits[i+12], 3), s_or(lshl(bits[i+13], 2), s_or(lshl(bits[i+14], 1), bits[i+15])))))))))))))));

            ptr->value   = code;
            move16();
            ptr->nb_bits = 16;
            move16();
            ++ptr;
        }
    }
    IF (sub(i, nb_bits) < 0)
    {
        FOR (; i<nb_bits; ++i)
        {
            ptr->value   = bits[i];
            move16();
            ptr->nb_bits = 1;
            move16();
            ++ptr;
        }
    }
    st_fx->next_ind_fx = (Word16)(ptr - st_fx->ind_list_fx);
    st_fx->nb_bits_tot_fx = add(st_fx->nb_bits_tot_fx, nb_bits);
}

/*-------------------------------------------------------------------*
 * get_next_indice_fx( )
 *
 * Get the next indice from the buffer
 *-------------------------------------------------------------------*/

UWord16 get_next_indice_fx(              /* o  : value of the indice */
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    Word16 nb_bits                       /* i  : number of bits that were used to quantize the indice */
)
{
    UWord16 value;
    Word16 i;

    assert(nb_bits <= 16);
    value = 0;
    move16();
    FOR (i = 0; i < nb_bits; i++)
    {
        value = lshl(value, 1);
        value = add(value, st_fx->bit_stream_fx[st_fx->next_bit_pos_fx+i]);
    }

    /* update the position in the bitstream */
    st_fx->next_bit_pos_fx = add(st_fx->next_bit_pos_fx, nb_bits);
    return value;
}

/*-------------------------------------------------------------------*
 * get_next_indice_1_fx( )
 *
 * Get the next 1-bit indice from the buffer
 *-------------------------------------------------------------------*/

UWord16 get_next_indice_1_fx(           /* o  : value of the indice */
    Decoder_State_fx *st_fx               /* i/o: decoder state structure */
)
{
    return st_fx->bit_stream_fx[st_fx->next_bit_pos_fx++];
}

/*-------------------------------------------------------------------*
 * get_next_indice_tmp()
 *
 * update the total number of bits and the position in the bitstream
 *-------------------------------------------------------------------*/

void get_next_indice_tmp_fx(
    Decoder_State_fx *st_fx,         /* o  : decoder state structure */
    Word16 nb_bits                   /* i  : number of bits that were used to quantize the indice */
)
{
    /* update the position in the bitstream */
    st_fx->next_bit_pos_fx = add(st_fx->next_bit_pos_fx, nb_bits);
}

/*-------------------------------------------------------------------*
 * get_indice_fx( )
 *
 * Get indice at specific position in the buffer
 *-------------------------------------------------------------------*/

UWord16 get_indice_fx(               /* o  : value of the indice */
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure */
    Word16 pos,             /* i  : absolute position in the bitstream (update after the read) */
    Word16 nb_bits          /* i  : number of bits that were used to quantize the indice */
)
{
    UWord16 value;
    Word16 i;

    assert(nb_bits <= 16);
    value = 0;
    move16();
    FOR (i = 0; i < nb_bits; i++)
    {
        value = lshl(value, 1);
        value = add(value, st_fx->bit_stream_fx[pos+i]);
    }

    return value;
}

/*-------------------------------------------------------------------*
 * get_indice_1_fx( )
 *
 * Get a 1-bit indice at specific position in the buffer
 *-------------------------------------------------------------------*/

UWord16 get_indice_1_fx(             /* o  : value of the indice */
    Decoder_State_fx *st_fx,         /* i/o: decoder state structure */
    Word16 pos              /* i  : absolute position in the bitstream (update after the read) */
)
{
    return st_fx->bit_stream_fx[pos];
}

/*-------------------------------------------------------------------*
 * reset_indices_enc_fx()
 *
 * Reset the buffer of indices
 *-------------------------------------------------------------------*/

void reset_indices_enc_fx(
    Encoder_State_fx *st_fx
)
{
    Word16 i;

    st_fx->nb_bits_tot_fx = 0;
    move16();
    st_fx->next_ind_fx = 0;
    move16();
    st_fx->last_ind_fx = -1;
    move16();

    FOR (i=0; i<MAX_NUM_INDICES; i++)
    {
        st_fx->ind_list_fx[i].nb_bits = -1;
        move16();
    }

    return;
}

/*-------------------------------------------------------------------*
  * reset_indices_dec_fx()
  *
  * Reset the buffer of decoder indices
  *-------------------------------------------------------------------*/

void reset_indices_dec_fx(
    Decoder_State_fx *st_fx
)
{
    st_fx->next_bit_pos_fx = 0;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * write_indices_fx()
 *
 * Write the buffer of indices to a file
 *-------------------------------------------------------------------*/

void write_indices_fx(
    Encoder_State_fx *st_fx,      /* i/o: encoder state structure */
    FILE *file        /* i  : output bitstream file   */
)
{
    Word16 i, k;
    Word16 stream[2+MAX_BITS_PER_FRAME], *pt_stream;
    Word32  mask;
    /*-----------------------------------------------------------------*
     * Encode Sync Header and Frame Length
     *-----------------------------------------------------------------*/

    pt_stream = stream;
    for (i=0; i<(2 + MAX_BITS_PER_FRAME); ++i)
    {
        stream[i] = 0;
    }
    *pt_stream++ = (Word16)SYNC_GOOD_FRAME;
    *pt_stream++ = st_fx->nb_bits_tot_fx;

    /*----------------------------------------------------------------*
     * Bitstream packing (conversion of individual indices into a serial stream)
     * Writing the serial stream into file
     *----------------------------------------------------------------*/

    for (i=0; i<MAX_NUM_INDICES; i++)
    {
        if (st_fx->ind_list_fx[i].nb_bits != -1)
        {
            /* mask from MSB to LSB */
            mask = 1 << (st_fx->ind_list_fx[i].nb_bits - 1);

            /* write bit by bit */
            for (k=0; k < st_fx->ind_list_fx[i].nb_bits; k++)
            {
                if ( st_fx->ind_list_fx[i].value & mask )
                {
                    *pt_stream++ = G192_BIN1;
                }
                else
                {
                    *pt_stream++ = G192_BIN0;
                }

                mask >>= 1;
            }
        }
    }

    /* Clearing of indices */
    FOR (i=0; i<MAX_NUM_INDICES; i++)
    {
        st_fx->ind_list_fx[i].nb_bits = -1;
        move16();
    }

    /* write the serial stream into file */
    fwrite( stream, sizeof(unsigned short), 2+stream[1], file );

    /* reset index pointers */
    st_fx->nb_bits_tot_fx = 0;
    st_fx->next_ind_fx = 0;
    st_fx->last_ind_fx = -1;

    return;
}


static void decoder_selectCodec(
    Decoder_State_fx *st,            /* i/o: decoder state structure                */
    const Word32 total_brate,    /* i  : total bitrate                          */
    const Word16 bit0
)
{
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    /* check if we are in AMR-WB IO mode */
    IF( L_sub(total_brate, SID_1k75) == 0 ||
        L_sub(total_brate, ACELP_6k60) == 0  || L_sub(total_brate, ACELP_8k85) == 0  || L_sub(total_brate, ACELP_12k65) == 0 ||
        L_sub(total_brate, ACELP_14k25) == 0 || L_sub(total_brate, ACELP_15k85) == 0 || L_sub(total_brate, ACELP_18k25) == 0 ||
        L_sub(total_brate, ACELP_19k85) == 0 || L_sub(total_brate, ACELP_23k05) == 0 || L_sub(total_brate, ACELP_23k85) == 0 )
    {
        st->Opt_AMR_WB_fx = 1;
        move16();
    }
    ELSE IF ( L_sub(total_brate, FRAME_NO_DATA) != 0 )
    {
        st->Opt_AMR_WB_fx = 0;
        move16();
    }

    /* select MODE1 or MODE2 */
    IF (st->Opt_AMR_WB_fx)
    {
        st->codec_mode = MODE1;
        move16();
    }
    ELSE
    {
        SWITCH ( total_brate )
        {
        case 0:
            st->codec_mode = st->last_codec_mode;
            move16();
            BREAK;
        case 2400:
            st->codec_mode = st->last_codec_mode;
            move16();
            BREAK;
        case 2800:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 3600:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 3700:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 5900:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 7200:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 8000:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 9600:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        case 13200:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 16400:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        case 24400:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        case 32000:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 48000:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        case 64000:
            st->codec_mode = MODE1;
            move16();
            BREAK;
        case 96000:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        case 128000:
            st->codec_mode = MODE2;
            move16();
            BREAK;
        }
    }

    IF ( st->ini_frame_fx == 0 )
    {
        if(sub(st->codec_mode,-1) == 0 )
        {
            st->codec_mode = MODE1;
            move16();
        }
        st->last_codec_mode = st->codec_mode;
        move16();
    }

    /* set SID/CNG type */
    IF ( L_sub(total_brate,SID_2k40) == 0 )
    {
        IF ( bit0 == G192_BIN0 )
        {
            st->cng_type_fx = LP_CNG;
            move16();

            /* force MODE1 when selecting LP_CNG */
            st->codec_mode = MODE1;
            move16();
        }
        ELSE
        {
            st->cng_type_fx = FD_CNG;
            test();
            if ( sub(st->last_codec_mode, MODE2) == 0 && L_sub(st->last_total_brate_fx,13200) == 0 )
            {
                st->codec_mode = MODE1;
                move16();
            }
        }
        st->last_cng_type_fx = st->cng_type_fx;     /* CNG type switching at the first correctly received SID frame */
    }

    test();
    test();
    IF ( total_brate == FRAME_NO_DATA && sub(st->prev_bfi_fx,1) == 0 && st->Opt_AMR_WB_fx == 0 )
    {
        /* prevent CNG type change if the first SID frame after active segment is missing */
        test();
        test();
        test();
        IF ( (sub(st->codec_mode,MODE1) == 0 && sub(st->last_cng_type_fx,LP_CNG) == 0) == 0 ||
             (sub(st->codec_mode,MODE2) == 0 && sub(st->last_cng_type_fx,FD_CNG) == 0) == 0 )
        {
            IF( sub(st->last_cng_type_fx,-1) != 0 )
            {
                st->cng_type_fx = st->last_cng_type_fx;
                move16();
            }
            ELSE
            {
                st->cng_type_fx = FD_CNG;
                move16();
                if( sub(st->codec_mode,MODE1) == 0 )
                {
                    st->cng_type_fx = LP_CNG;
                    move16();
                }
            }

            if( sub(st->cng_type_fx,LP_CNG) == 0 )
            {
                st->codec_mode = MODE1;
                move16();
            }
        }
    }

    return;
}


void dec_prm_core(Decoder_State_fx *st)
{
    Word16 n, frame_size_index, num_bits;
    UWord16 lsb;
    Word32 L_tmp;

    frame_size_index = -1;
    move16();
    st->core_fx = -1;
    move16();

    IF (L_sub(st->total_brate_fx, FRAME_NO_DATA) == 0)
    {
        st->m_frame_type = ZERO_FRAME;
        move16();
    }
    ELSE IF (L_sub(st->total_brate_fx, SID_2k40) == 0)
    {
        st->m_frame_type = SID_FRAME;
        move16();
    }
    ELSE
    {
        st->m_frame_type = ACTIVE_FRAME;
        move16();
        Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
        num_bits = extract_l(L_shr(L_tmp, 3)); /* Q0 */
        assert(num_bits == st->total_brate_fx/50);
        FOR (n=0; n<FRAME_SIZE_NB; ++n)
        {
            IF (sub(FrameSizeConfig[n].frame_bits, num_bits) == 0)
            {
                frame_size_index = n;
                move16();
                BREAK;
            }
        }

        /* Get bandwidth mode */
        st->bwidth_fx = get_next_indice_fx(st, FrameSizeConfig[frame_size_index].bandwidth_bits);

        st->bwidth_fx = add(st->bwidth_fx, FrameSizeConfig[frame_size_index].bandwidth_min);

        /* Skip reserved bit */
        get_next_indice_tmp_fx(st, FrameSizeConfig[frame_size_index].reserved_bits);

        IF (get_next_indice_1_fx(st) != 0) /* TCX */
        {
            st->core_fx = TCX_20_CORE;
            move16();
            if (get_next_indice_1_fx(st) != 0)
            {
                st->core_fx = HQ_CORE;
                move16();
            }
        }
        ELSE /* ACELP */
        {
            st->core_fx = ACELP_CORE;
            move16();
        }
    }
}

/*-----------------------------------------------------------------*
 * decision_matrix_core_dec()
 *
 * Read core mode signalling bits from the bitstream
 * Set st->core, and st->bwidth if signalled together with the core.
 *-----------------------------------------------------------------*/

void decision_matrix_core_dec(
    Decoder_State_fx *st                 /* i/o: decoder state structure                   */
)
{
    Word16 start_idx;
    Word32 ind;
    Word16 nBits;

    assert(st->bfi_fx != 1);

    st->core_fx = -1;
    move16();
    st->bwidth_fx = -1;
    move16();

    test();
    IF ( L_sub(st->total_brate_fx, FRAME_NO_DATA) == 0 || L_sub(st->total_brate_fx, SID_2k40) == 0 )
    {
        st->core_fx = ACELP_CORE;
        move16();
    }
    /* SC-VBR */
    ELSE IF ( st->total_brate_fx == PPP_NELP_2k80 )
    {
        st->core_fx = ACELP_CORE;
        move16();
        return;
    }

    /*---------------------------------------------------------------------*
     * ACELP/HQ core selection
     *---------------------------------------------------------------------*/

    test();
    IF ( L_sub(st->total_brate_fx, ACELP_24k40) < 0 )
    {
        st->core_fx = ACELP_CORE;
        move16();
    }
    ELSE IF ( L_sub(st->total_brate_fx, ACELP_24k40) >= 0 && L_sub(st->total_brate_fx, ACELP_64k) <= 0 )
    {
        /* read the ACELP/HQ core selection bit */
        st->core_fx = imult1616(get_next_indice_fx( st, 1 ), HQ_CORE);
    }
    ELSE
    {
        st->core_fx = HQ_CORE;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Read ACELP signalling bits from the bitstream
     *-----------------------------------------------------------------*/

    IF ( sub(st->core_fx, ACELP_CORE) == 0 )
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        start_idx = 0;
        move16();
        WHILE ( L_sub(acelp_sig_tbl[start_idx], st->total_brate_fx) != 0 )
        {
            start_idx = add(start_idx, 1);
        }

        /* skip the bitrate */
        start_idx = add(start_idx, 1);

        /* retrieve the number of bits */
        nBits = extract_l(acelp_sig_tbl[start_idx]);
        start_idx = add(start_idx, 1);

        /* retrieve the signalling indice */
        ind = acelp_sig_tbl[add(start_idx, get_next_indice_fx( st, nBits ))];
        st->bwidth_fx = extract_l(L_and(L_shr(ind, 3), 0x7));

        /* convert signalling indice into signalling information */
        if ( L_sub(L_and(ind, 0x7), LR_MDCT) == 0 )
        {
            st->core_fx = HQ_CORE;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Read HQ signalling bits from the bitstream
     * Set HQ core type
     *-----------------------------------------------------------------*/

    IF ( sub(st->core_fx, HQ_CORE) == 0 )
    {
        /* read the HQ/TCX core switching flag */
        if ( get_next_indice_fx( st, 1 ) != 0 )
        {
            st->core_fx = TCX_20_CORE;
            move16();
        }

        /* For TCX: read/set band-width (needed for different I/O sampling rate support) */
        test();
        IF( sub(st->core_fx, TCX_20_CORE) == 0 && L_sub(st->total_brate_fx, ACELP_16k40) > 0 )
        {
            ind = get_next_indice_fx( st, 2 );

            IF( ind == 0 )
            {
                st->bwidth_fx = NB;
                move16();
            }
            ELSE IF( L_sub(ind, 1) == 0 )
            {
                st->bwidth_fx = WB;
                move16();
            }
            ELSE IF( L_sub(ind, 2) == 0 )
            {
                st->bwidth_fx = SWB;
                move16();
            }
            ELSE
            {
                st->bwidth_fx = FB;
                move16();
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * mdct_switching_dec()
 *
 * Set up MDCT core switching if indicated in the bit stream
 *-------------------------------------------------------------------*/

static void mdct_switching_dec(
    Decoder_State_fx *st                 /* i/o: decoder state structure                */
)
{
    IF (st->Opt_AMR_WB_fx != 0)
    {
        return;
    }

    test();
    test();
    IF (L_sub(st->total_brate_fx, ACELP_13k20) == 0 || L_sub(st->total_brate_fx, ACELP_32k) == 0)
    {
        st->mdct_sw_enable = MODE1;
        move16();
    }
    ELSE IF (L_sub(ACELP_16k40, st->total_brate_fx) <= 0 && L_sub(st->total_brate_fx, ACELP_24k40) <= 0)
    {
        st->mdct_sw_enable = MODE2;
        move16();
    }

    test();
    test();
    IF ( sub(st->codec_mode, MODE1) == 0 && sub(st->mdct_sw_enable, MODE1) == 0 )
    {
        /* Read ahead core mode signaling */
        Word16 next_bit_pos_save;
        Word16 core_save;
        Word16 bwidth_save;

        next_bit_pos_save = st->next_bit_pos_fx;
        move16();
        core_save = st->core_fx;
        move16();
        bwidth_save = st->bwidth_fx;
        move16();

        decision_matrix_core_dec(st); /* sets st->core */

        IF (sub(st->core_fx, TCX_20_CORE) == 0)
        {
            /* Trigger TCX */
            st->codec_mode = MODE2;
            move16();
            st->mdct_sw = MODE1;
            move16();
        }
        ELSE
        {
            /* Rewind bitstream */
            st->next_bit_pos_fx = next_bit_pos_save;
            move16();
            IF (st->bfi_fx != 0)
            {
                st->core_fx   = core_save;
                move16();
                st->bwidth_fx = bwidth_save;
                move16();
            }
        }
    }
    ELSE IF (sub(st->codec_mode, MODE2) == 0 && sub(st->mdct_sw_enable, MODE2) == 0)
    {
        /* Read ahead core mode signaling */
        Word16 next_bit_pos_save;
        Word16 core_save;
        Word16 bwidth_save;

        next_bit_pos_save = st->next_bit_pos_fx;
        move16();
        core_save = st->core_fx;
        move16();
        bwidth_save = st->bwidth_fx;
        move16();

        dec_prm_core(st); /* sets st->core */

        IF (sub(st->core_fx, HQ_CORE) == 0)
        {
            /* Trigger HQ_CORE */
            st->codec_mode = MODE1;
            move16();
            st->mdct_sw = MODE2;
            move16();
        }
        ELSE
        {
            /* Rewind bitstream */
            st->next_bit_pos_fx = next_bit_pos_save;
            move16();
            IF (st->bfi_fx != 0)
            {
                st->core_fx   = core_save;
                move16();
                st->bwidth_fx = bwidth_save;
                move16();
            }
        }
    }
}

/*-------------------------------------------------------------------*
 * BRATE2IDX_fx()
 *
 * Convert Bitrate to Index Value
 *-------------------------------------------------------------------*/

Word16 BRATE2IDX_fx(Word32 brate)
{
    Word32 L_temp;
    Word32 L_idx;
#define START 11

    extern const Word16 bit_rates_div50[];

    /* This is a Fast Bit Rate Value to Index Value Binary Search */
    L_temp = L_msu0(brate, bit_rates_div50[START], 50);
    L_temp = L_min(6, L_max(-6, L_temp));
    L_idx = L_add(L_temp, START);
    L_temp = L_msu0(brate, bit_rates_div50[L_idx], 50);
    L_temp = L_min(3, L_max(-3, L_temp));
    L_idx = L_add(L_temp, L_idx);
    L_temp = L_msu0(brate, bit_rates_div50[L_idx], 50);
    L_temp = L_min(1, L_max(-2, L_temp));
    L_idx = L_add(L_temp, L_idx);
    L_temp = L_msu0(brate, bit_rates_div50[L_idx], 50);
    if (L_temp != 0) L_idx = L_add(L_idx, 1);
    return (Word16)L_idx;
}


/*-------------------------------------------------------------------*
 * BRATE2IDX16k_fx()
 *
 * Convert Bitrate to Index Value
 *-------------------------------------------------------------------*/

Word16 BRATE2IDX16k_fx(Word32 brate)
{
    Word32 L_temp, L_idx;
#define START_16K 6

    extern const Word16 bit_rates_16k_div50[];

    if(L_sub(brate,ACELP_16k40)==0)
    {
        brate=ACELP_14k80;
    }

    /* This is a Fast Bit Rate Value to Index Value Binary Search */
    L_temp = L_msu0(brate, bit_rates_16k_div50[START_16K], 50);
    L_temp = L_min(3, L_max(-3, L_temp));
    L_idx = L_add(L_temp, START_16K);
    L_temp = L_msu0(brate, bit_rates_16k_div50[L_idx], 50);
    L_temp = L_min(2, L_max(-2, L_temp));
    L_idx = L_add(L_temp, L_idx);
    L_temp = L_msu0(brate, bit_rates_16k_div50[L_idx], 50);
    L_temp = L_min(1, L_max(-1, L_temp));
    L_idx = L_add(L_temp, L_idx);

    return (Word16)L_idx;
}

/*-------------------------------------------------------------------*
 * BIT_ALLOC_IDX_fx()
 *-------------------------------------------------------------------*/

Word32 BIT_ALLOC_IDX_fx(Word32 brate, Word16 ctype, Word16  sfrm, Word16 tc)
{
    Word32 L_temp;
    Word16 temp;
    if (ctype == INACTIVE) /* no sub(ctype, INACTIVE) because it is '0' */
        ctype = GENERIC;
    move16();
    L_temp = L_mac0(-1l*256, 1*256, ctype);

    temp = BRATE2IDX_fx(brate);
    L_temp = L_mac0(L_temp, 4*256, temp);
    if (tc >= 0)
        L_temp = L_mac0(L_temp, (10-4)*256, temp);
    /* So either 'temp' x 4 when 'tc < 0', 'temp' x 10 otherwise */

    L_temp = L_mac0(L_temp, 1*256, s_max(0, tc));

    L_temp = L_mac0(L_temp, s_max(0, sfrm), 1);
    if (sfrm < 0)
        L_temp = L_shr(L_temp, 2);
    L_temp = L_shr(L_temp, 6);

    return L_temp;
}

/*-------------------------------------------------------------------*
 * BIT_ALLOC_IDX_16KHZ_fx()
 *-------------------------------------------------------------------*/

Word32 BIT_ALLOC_IDX_16KHZ_fx(Word32 brate, Word16 ctype, Word16 sfrm, Word16 tc)
{
    Word32 L_temp;
    Word16 temp;
    /* 'ctype' =
       TRANSITION => 2
       GENERIC    => 1
       ALL Other  => 0
       */
    L_temp = L_and(shr(0x0240l, shl(ctype, 1)), 3);

    temp = BRATE2IDX16k_fx(brate);
    L_temp = L_mac0(L_temp, 3, temp);
    if (tc >= 0)
        L_temp = L_mac0(L_temp, (7-3), temp);
    /* So either 'temp' x 3 when 'tc < 0', 'temp' x 7 otherwise */

    L_temp = L_mac0(L_temp, 1, s_max(0, tc));

    IF (sfrm >= 0)
    {
        /* Mult by 5 */
        L_temp = L_add(L_temp, L_shl(L_temp, 2));
        L_temp = L_mac0(L_temp, shr(sfrm, 6), 1);
    }

    return L_temp;
}

/*-------------------------------------------------------------------*
 * read_indices_fx()
 *
 * Read indices from an ITU-T G.192 bitstream to the buffer
 * Simulate packet losses by inserting frame erasures
 *-------------------------------------------------------------------*/

Word16 read_indices_fx(                /* o  : 1 = reading OK, 0 = problem            */
    Decoder_State_fx *st,                /* i/o: decoder state structure                */
    FILE *file,              /* i  : bitstream file                         */
    Word16 rew_flag            /* i  : rewind flag (rewind file after reading)*/
)
{
    Word16 k;
    UWord16 utmp, stream[2+MAX_BITS_PER_FRAME], *pt_stream, *bit_stream_ptr;
    Word16 num_bits;
    Word32 total_brate;
    st->bfi_fx = 0;
    st->mdct_sw_enable = 0;
    st->mdct_sw = 0;
    reset_indices_dec_fx( st );

    /* read the Sync Header field from the bitstream */
    /* in case rew_flag is set, read until first good frame is encountered */
    do
    {
        /* read the Sync header */
        if ( fread( &utmp, sizeof(unsigned short), 1, file ) != 1 )
        {
            if( ferror( file ) )
            {
                /* error during reading */
                fprintf(stderr, "\nError reading the bitstream !");
                exit(-1);
            }
            else
            {
                /* end of file reached */
                return 0;
            }
        }

        /* set the BFI indicator according the value of Sync Header */
        if ( sub(utmp, SYNC_BAD_FRAME) == 0 )
        {
            st->bfi_fx = 1;
        }


        else
        {
            st->bfi_fx = 0;
        }

        /* read the Frame Length field from the bitstream */
        if ( fread( &num_bits, sizeof(unsigned short), 1, file ) != 1 )
        {
            if( ferror( file ) )
            {
                /* error during reading */
                fprintf(stderr, "\nError reading the bitstream !");
                exit(-1);
            }
            else
            {
                /* end of file reached */
                return 0;
            }
        }
        /* convert the frame length to total bitrate */
        total_brate = (long)(num_bits* 50);

        /* read ITU-T G.192 serial stream of indices from file to the local buffer */
        pt_stream = stream;
        fread( pt_stream, sizeof(unsigned short), num_bits, file );

    }
    while ( rew_flag && (st->bfi_fx || L_sub(total_brate,2800) < 0) );

    /* get total bit-rate */
    if ( st->bfi_fx == 0 )
    {
        /* select MODE1 or MODE2 */
        decoder_selectCodec( st, total_brate, *pt_stream );
    }


    /* in case rew_flag is set, rewind the file and return */
    /* (used in io_enc() to print out info about technologies and to initialize the codec) */
    if ( rew_flag )
    {
        rewind( file );
        st->total_brate_fx = total_brate;
        move16();
        return 1;
    }

    /* GOOD frame */
    if ( st->bfi_fx == 0  )
    {
        /* GOOD frame - convert ITU-T G.192 words to short values */
        bit_stream_ptr = st->bit_stream_fx;

        for( k = 0; k< num_bits; ++k)
        {
            *bit_stream_ptr++ = (*pt_stream++ == G192_BIN1);
        }
        /*add two zero bytes for arithmetic coder flush*/
        for(k=0; k< 2*8; ++k)
        {
            *bit_stream_ptr++ = 0;
        }
        /*a change of the total bitrate should not be
        known to the decoder, if the received frame was lost*/
        st->total_brate_fx = total_brate ;

        mdct_switching_dec(st);
    }
    return 1;
}
/*-------------------------------------------------------------------*
* isPartialCopyPresent()
*
* Check if the frame includes a partial copy for channel aware processing.
*-------------------------------------------------------------------*/

void getPartialCopyInfo(
    Decoder_State_fx *st,              /* i/o: decoder state structure       */
    Word16 *coder_type,
    Word16 *sharpFlag
)
{
    Word16 nBits;
    Word16 ind;
    /* check the rf flag in the packet */
    get_rfFlag( st, &(st->rf_flag), &nBits , &ind);

    /* get rf frame type info */
    get_rfFrameType( st, &(st->rf_frame_type) );

    /* Get the FEC offset info */
    get_rf_fec_offset( st, &(st->rf_fec_offset) );

    /* reset number of target bits in case of rate switching */
    st->rf_target_bits = 0;

    /* Get the number of bits used for RF*/
    IF( sub(st->rf_flag,1) == 0 )
    {
        *coder_type = s_and(ind,0x7);
        st->bwidth_fx = s_and(shr(ind,3), 0x7);
        *sharpFlag = s_and(shr(ind,6), 0x1);
        st->codec_mode = MODE2;
        move16();
        get_rfTargetBits( st->rf_frame_type, &(st->rf_target_bits) );
        IF( sub(st->bfi_fx,FRAMEMODE_FUTURE) == 0 )
        {
            st->use_partial_copy = 1;
            /* now set the frame mode to normal mode */
            test();
            IF(sub(st->rf_frame_type,RF_TCXFD) >= 0 && sub(st->rf_frame_type, RF_TCXTD2) <= 0)
            {
                st->bfi_fx = 1;
                st->core_fx = 1;
            }
            ELSE
            {
                st->bfi_fx = FRAMEMODE_NORMAL;
                st->core_fx = 0;
            }
        }
        get_next_indice_tmp_fx(st, nBits);
    }
}

/*-------------------------------------------------------------------*
* get_rfFlag()
*
* Check if rf flag is present in the bitstream
*-------------------------------------------------------------------*/

void get_rfFlag(
    Decoder_State_fx *st,                      /* i: decoder state structure       */
    Word16 *rf_flag,                 /* o  : check for the RF flag    */
    Word16 *nBits,
    Word16 *ind
)
{
    Word16 start_idx, nBits_tmp;
    Word16 ind_tmp;

    /* Init */
    *rf_flag = 0;

    /* check for rf_flag in the packet and extract the rf_frame_type and rf_fec_offset */
    test();
    test();
    IF( L_sub(st->total_brate_fx,ACELP_13k20) == 0 && (sub(st->bfi_fx,FRAMEMODE_NORMAL) == 0 || sub(st->bfi_fx, FRAMEMODE_FUTURE) == 0) )
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        start_idx = 0;
        WHILE ( L_sub(acelp_sig_tbl[start_idx], st->total_brate_fx) != 0 )
        {
            start_idx++;
            assert((start_idx < MAX_ACELP_SIG) && "ERROR: start_idx larger than acelp_sig_tbl[].\n");
        }

        /* skip the bitrate */
        start_idx = add(start_idx,1);

        /* retrieve the number of bits */
        nBits_tmp = (Word16) acelp_sig_tbl[start_idx++];

        /* retrieve the signalling indice */
        ind_tmp = (Word16) acelp_sig_tbl[start_idx + get_indice_fx( st, 0, nBits_tmp )];

        /* convert signalling indice into RF flag. */
        *rf_flag = s_and(shr(ind_tmp, 7), 0x1);

        if( ind )
        {
            *ind = ind_tmp;
        }

        if( nBits )
        {
            *nBits = nBits_tmp;
        }
    }
}

/*-------------------------------------------------------------------*
* get_rfFrameType()
*
* Extract the rf frame type
*-------------------------------------------------------------------*/

void get_rfFrameType(
    Decoder_State_fx *st,                      /* i  : decoder state structure       */
    Word16 *rf_frame_type            /* o  : RF frame type                 */
)
{
    Word16 num_bits = 0;

    IF( sub(st->rf_flag, 1)== 0)
    {
        /*num_bits = st->total_brate_fx/50;*/
        if( L_sub(st->total_brate_fx, ACELP_13k20) == 0 )
        {
            num_bits = 264;
            move16();     /* @13.2kbps */
        }
        else
        {
            UWord16 lsb;
            Word32 L_tmp;
            Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
            num_bits = extract_l(L_shr(L_tmp, 3));                /* Q0 */
        }

        /* the last three bits in a packet is the RF frame type */
        *rf_frame_type = get_indice_fx( st, num_bits - 3, 3 );
    }
    ELSE
    {
        *rf_frame_type = 0;
    }
}

/*-------------------------------------------------------------------*
* get_rf_fec_offset()
*
* Extract the FEC offset
*-------------------------------------------------------------------*/

void get_rf_fec_offset(
    Decoder_State_fx *st,                      /* i  : decoder state structure       */
    Word16 *rf_fec_offset            /* o  : RF fec offset                 */
)
{
    Word16 num_bits, tmp;

    IF( sub(st->rf_flag,1)== 0)
    {
        /*num_bits = st->total_brate_fx/50;*/
        if( L_sub(st->total_brate_fx, ACELP_13k20) == 0 )
        {
            num_bits = 264;
            move16();     /* @13.2kbps */
        }
        else
        {
            UWord16 lsb;
            Word32 L_tmp;
            Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
            num_bits = extract_l(L_shr(L_tmp, 3));                /* Q0 */
        }

        /* the two bits before the rf frame type contain the fec offset  */
        tmp = get_indice_fx( st, num_bits - 5, 2 );

        if( tmp == 0 )
        {
            *rf_fec_offset = 2;
            move16();
        }
        else
        {
            *rf_fec_offset = add(shl(tmp, 1), 1);
        }
    }
    ELSE
    {
        *rf_fec_offset = 0;
        move16();
    }
}

/*-------------------------------------------------------------------*
* get_rf_fec_offset()
*
* Extract the FEC offset
*-------------------------------------------------------------------*/

void get_rfTargetBits(
    Word16 rf_frame_type,           /* i  : RF frame type                 */
    Word16 *rf_target_bits            /* o  : Number of RF target bits      */
)
{

    /* Number of RF bits for different RF coder types */

    SWITCH (rf_frame_type)
    {
    case RF_NO_DATA:
        *rf_target_bits = 5;
        BREAK;
    case RF_TCXFD:
        *rf_target_bits = 27;
        BREAK;
    case RF_TCXTD1:
        *rf_target_bits = 16;
        BREAK;
    case RF_TCXTD2:
        *rf_target_bits = 16;
        BREAK;
    case RF_ALLPRED:
        /* Es_pred bits 3 bits, LTF: 1, pitch: 8,5,5,5, FCB: 0, gain: 7,0,7,0, Diff GFr: 4*/
        *rf_target_bits = 63;
        BREAK;
    case RF_NOPRED:
        /* Es_pred bits 3 bits, LTF: 0, pitch: 0, FCB: 7,7,7,7, gain: 6,0,6,0, Diff GFr: 2*/
        *rf_target_bits = 66;
        BREAK;
    case RF_GENPRED:
        /* Es_pred bits 3 bits, LTF: 1, pitch: 8,0,8,0, FCB: 6,7,5,5, gain: 5,0,5,0, Diff GFr: 0*/
        *rf_target_bits = 70;
        BREAK;
    case RF_NELP:
        /* gain: 19, Diff GFr: 5 */
        *rf_target_bits =  45;
        BREAK;
    }
}


/*-------------------------------------------------------------------*
 * get_NextCoderType_fx()
 *
 * Extract the coder type of next frame
 *-------------------------------------------------------------------*/

void get_NextCoderType_fx(
    UWord8 *bitsteam,                           /* i : bitstream         */
    Word16 *next_coder_type                    /* o : next coder type   */
)
{
    Word16 k;
    Word16 start_idx;
    Word16 nBits_tmp;
    Word8 bit_stream[ACELP_13k20/50];
    UWord16 tmp;


    FOR( k = 0; k < ACELP_13k20/50; k++ )
    {
        bit_stream[k] = (bitsteam[k / 8] >> (7 - (k % 8))) & 0x1;
    }
    start_idx = 0;
    WHILE ( L_sub(acelp_sig_tbl[start_idx], ACELP_13k20) != 0 )
    {
        start_idx = add(start_idx,1);
        assert((start_idx < MAX_ACELP_SIG) && "ERROR: start_idx larger than acelp_sig_tbl[].\n");
    }

    /* skip the bitrate */
    start_idx = add(start_idx,1);

    tmp = 0;
    move16();
    nBits_tmp = (Word16) acelp_sig_tbl[start_idx++];
    FOR (k = 0; k < nBits_tmp; k++)
    {
        tmp = lshl(tmp, 1);
        tmp = add(tmp, bit_stream[k]);
    }
    /* retrieve the signalling indice */
    *next_coder_type = s_and((Word16)acelp_sig_tbl[start_idx + tmp],0x7);
}

/*-------------------------------------------------------------------*
 * read_indices_from_djb_fx()
 *
 * Read indices from the de-jitter buffer payload (works also for AMR-WB IO mode)
 *-------------------------------------------------------------------*/

void read_indices_from_djb_fx(              /* o  : 1 = reading OK, 0 = problem   */
    Decoder_State_fx *st,                      /* i/o: decoder state structure       */
    UWord8 *pt_stream,               /* i  : bitstream file                */
    Word16 num_bits                  /* i  : input frame length in bits    */
    ,Word16 partialframe              /* i  : partial frame information     */
    ,Word16 next_coder_type           /* i  : next coder type information     */
)
{
    Word16 k;
    UWord16 *bit_stream_ptr;
    Word32 total_brate;
    Word16 bit0;

    bit0 = 0;
    /* There is no FRAME_NO_DATA or BAD frame indicator in RTP, frames are just missing.
     * In case of comfort noise handle missing frame as FRAME_NO_DATA, otherwise use PLC. */
    IF(num_bits != 0)
    {
        st->bfi_fx = 0;
        bit0 = ((*pt_stream & 0x80) != 0) ? G192_BIN1 : G192_BIN0;
    }
    ELSE IF(L_sub(st->total_brate_fx,SID_1k75) == 0 ||
            L_sub(st->total_brate_fx,SID_2k40) == 0  ||
            L_sub(st->total_brate_fx,FRAME_NO_DATA) == 0 )
    {
        st->bfi_fx = 0;
    }
    ELSE
    {
        st->bfi_fx = 1;
    }
    IF( sub(partialframe,1) == 0 || sub(st->prev_use_partial_copy,1) == 0 )
    {
        st->next_coder_type = next_coder_type;
    }
    ELSE
    {
        st->next_coder_type = INACTIVE;
    }

    st->mdct_sw_enable = 0;
    st->mdct_sw = 0;
    reset_indices_dec_fx( st );
    total_brate = num_bits * 50;

    IF(sub(partialframe,1) == 0)
    {
        st->bfi_fx = 2;
    }

    IF ( st->bfi_fx != 1 )
    {
        /* select MODE1 or MODE2 */
        decoder_selectCodec( st, total_brate, bit0 );

        /* convert bitstream from compact bytes to short values and store it in decoder state */
        bit_stream_ptr = st->bit_stream_fx;
        FOR( k = 0; k < num_bits; k++ )
        {
            *bit_stream_ptr++ = (pt_stream[k / 8] >> (7 - (k % 8))) & 0x1;
        }
        /* add two zero bytes for arithmetic coder flush */
        FOR( k=0; k < 8*2; ++k )
        {
            *bit_stream_ptr++ = 0;
        }

        /* a change of the total bitrate should not be
           known to the decoder, if the received frame was
           lost */
        st->total_brate_fx = total_brate;

        mdct_switching_dec(st);
    }
}




/*-------------------------------------------------------------------*
 * get_indice_preview()
 *
 * Indices preview to parse for the presence of partial copy
 *-------------------------------------------------------------------*/
static UWord16 get_indice_preview(
    UWord8 *bitstream,
    Word16 bitstreamSize,
    Word16 pos,
    Word16 nb_bits
)
{
    UWord16 value;
    Word16 i;
    UWord16 bitstreamShort[MAX_BITS_PER_FRAME+16];
    UWord16 *bitstreamShortPtr;

    /* convert bitstream from compact bytes to short values */
    bitstreamShortPtr = bitstreamShort;
    FOR( i = 0; i < bitstreamSize; i++ )
    {
        *bitstreamShortPtr++ = (bitstream[i / 8] >> (7 - (i % 8))) & 0x1;
    }

    assert(nb_bits <= 16);
    value = 0;
    FOR (i = 0; i < nb_bits; i++)
    {
        value = shl(value,1);
        value = add(value,bitstreamShort[pos+i]);
    }
    return value;
}

/*-------------------------------------------------------------------*
 * evs_dec_previewFrame()
 *
 * Signalling index preview
 *-------------------------------------------------------------------*/
void evs_dec_previewFrame(
    UWord8 *bitstream,
    Word16 bitstreamSize,
    Word16 *partialCopyFrameType,
    Word16 *partialCopyOffset
)
{
    Word32 total_brate;
    Word16 start_idx, nBits;
    Word32 ind;
    Word16 rf_flag;

    rf_flag = 0;
    *partialCopyFrameType = 0;
    *partialCopyOffset = 0;
    total_brate = bitstreamSize * 50;

    IF( L_sub(total_brate,ACELP_13k20) == 0 )
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        start_idx = 0;
        WHILE ( L_sub(acelp_sig_tbl[start_idx], total_brate) != 0 )
        {
            start_idx = add(start_idx,1);
            assert((start_idx < MAX_ACELP_SIG) && "ERROR: start_idx larger than acelp_sig_tbl[].\n");
        }

        /* skip the bitrate */
        start_idx = add(start_idx,1);
        /* retrieve the number of bits */
        nBits = (Word16) acelp_sig_tbl[start_idx++];

        /* retrieve the signalling indice */
        ind = acelp_sig_tbl[start_idx + get_indice_preview( bitstream, bitstreamSize, 0, nBits )];

        /* convert signalling indice into RF flag. */
        rf_flag = s_and(extract_l(L_shr(ind, 7)), 0x1);
        assert(rf_flag == ((ind >> 7) & 0x1));
        IF(rf_flag != 0)
        {
            /* read the fec offset at which the partial copy is received */
            ind = get_indice_preview( bitstream, bitstreamSize, (bitstreamSize-5), 2 );
            IF(ind== 0) *partialCopyOffset = 2;
            ELSE IF(L_sub(ind,1)==0) *partialCopyOffset = 3;
            ELSE IF(L_sub(ind,2)==0) *partialCopyOffset = 5;
            ELSE IF(L_sub(ind,3)==0) *partialCopyOffset = 7;

            /* the last three bits in a packet is the RF frame type */
            *partialCopyFrameType = get_indice_preview( bitstream, bitstreamSize, bitstreamSize - 3, 3 );
        }
    }
}


