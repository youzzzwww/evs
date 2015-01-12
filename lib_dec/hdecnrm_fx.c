/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"
#include "rom_dec_fx.h"
#include "stl.h"        /* required for wmc_tool */

/*--------------------------------------------------------------------------*/
/*  Function  hdecnrm_fx                                                    */
/*  ~~~~~~~~~~~~~~~~~~~~                                                    */
/*                                                                          */
/*  Huffman decoding for indices of quantized norms                         */
/*--------------------------------------------------------------------------*/
void hdecnrm_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 numNorms, /* (i)    number of norms */
    Word16 *index)         /* (o)    indices of quantized norms */
{
    Word16 i, j, k, n, m;
    Word16 temp;
    Word16 *pidx;


    pidx  = index;

    m = sub(numNorms, 1);
    FOR (i=0; i<m; i++)
    {
        j = (Word16)0;
        move16();
        k = (Word16)0;
        move16();

        if (get_next_indice_1_fx( st_fx ) != 0)
        {
            j = (Word16)1;
            move16();
        }
        if (get_next_indice_1_fx( st_fx ) != 0)
        {
            k = (Word16)1;
            move16();
        }
        n = add(shl(j, 1), k);
        j = shl(j, 2);
        temp = sub(add(16, n), j);
        IF ( get_next_indice_1_fx( st_fx ) != 0 )
        {
            temp = add(add(12, n), j);
            IF ( get_next_indice_1_fx( st_fx ) != 0 )
            {
                j = (Word16)0;
                move16();
                if ( get_next_indice_1_fx( st_fx ) != 0 )
                {
                    j = (Word16)1;
                    move16();
                }
                temp = add(8, n);
                if (j!=0)
                {
                    temp = add(temp, 12);
                }
                IF ( get_next_indice_1_fx( st_fx ) != 0 )
                {
                    temp = n;
                    move16();

                    if ( get_next_indice_1_fx( st_fx ) != 0 )
                    {
                        temp = add(4, n);
                    }
                    if (j!=0)
                    {
                        temp = add(temp, 24);
                    }
                }
            }
        }
        *pidx++ = temp;
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*/
/*  Function  decode_huff_context                                           */
/*  ~~~~~~~~~~~~~~~~~                                                       */
/*                                                                          */
/*  Context based Huffman decoding for indices of quantized norms           */
/*--------------------------------------------------------------------------*/
/*  const Word16 *hufftab,   (i)    Huffman table                           */
/*  Word16       *rbits      (i/o)  the number of read bits                 */
/*--------------------------------------------------------------------------*/

Word16 decode_huff_context_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 *hufftab,
    Word16 *rbits
)
{
    Word16 tmp_l,tmp_h;
    Word32 hufftab_idx;

    hufftab_idx = L_deposit_l(0);
    WHILE( hufftab[hufftab_idx] > 0)
    {
        tmp_h = shr(hufftab[hufftab_idx],4);
        tmp_l = sub(hufftab[hufftab_idx],shl(tmp_h,4));
        *rbits = add(*rbits,tmp_l);
        hufftab_idx = L_add(hufftab_idx, L_add(L_deposit_l(tmp_h), get_next_indice_fx( st_fx, tmp_l ) ));
    }
    return negate(hufftab[hufftab_idx]);
}


/*--------------------------------------------------------------------------*/
/*  hdecnrm_context_fx()                                                    */
/*  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                          */
/*                                                                          */
/*  Huffman decoding for indices of quantized norms                         */
/*--------------------------------------------------------------------------*/
/*  Word16       N           (i)    number of norms                         */
/*  Word16       *index      (o)    indices of quantized norms              */
/*  Word16       *n_length   (o)    decoded stream length                   */
/*--------------------------------------------------------------------------*/

void hdecnrm_context_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 N,
    Word16 *index,
    Word16 *n_length
)
{
    Word16 i, prevj, tmp;

    prevj = add(index[0], OFFSET_NORM);
    FOR( i=1; i < N; i++)
    {
        IF( sub(prevj, HTH_NORM) >0 )
        {
            /* above */
            tmp = decode_huff_context_fx( st_fx, hntable_fx, n_length);
            index[i] = sub(31 , tmp);
            move16();
        }
        ELSE
        {
            IF( sub(prevj, LTH_NORM) <0 )
            {
                /* less */
                index[i] = decode_huff_context_fx(st_fx, hntable_fx, n_length);
                move16();
            }
            ELSE
            {
                /* equal */
                index[i] = decode_huff_context_fx(st_fx, hetable_fx, n_length);
                move16();
            }
        }
        prevj = index[i];
        move16();
    }
    return;
}

void hdecnrm_resize_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word16 N,                 /* (i)  number of SFMs */
    Word16 *index             /* (o)  norm quantization index vector */
)
{
    Word16 i, j, k, m;
    Word16 temp;
    Word16 *pidx;

    pidx  = index;

    m = sub(N, 1);
    move16();
    FOR (i=0; i<m; i++)
    {
        j = 0;
        move16();
        k = 0;
        move16();

        FOR( j = 0; j < 11; j++)
        {
            IF ( get_next_indice_1_fx( st_fx ) != 0 )
            {
                k = add(k, 1);
                move16();
            }
            ELSE
            {
                BREAK;
            }
        }

        IF( sub(k, 11) == 0)
        {
            temp = 25;
            move16();
        }
        ELSE IF ( sub(k, 10) == 0)
        {
            temp = 5;
            move16();
        }
        ELSE IF ( sub(k, 9) == 0)
        {
            temp = 6;
            move16();
        }
        ELSE
        {
            IF ( get_next_indice_1_fx( st_fx ) != 0 )
            {
                temp = add(16, k);
                move16();
            }
            ELSE
            {
                temp = sub(15, k);
                move16();
            }

        }

        *pidx++ = temp;
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------
 *  huff_dec()
 *
 *  Huffman decoding
 *--------------------------------------------------------------------------*/

void huff_dec_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure                         */
    const Word16 N,                  /* i  : Number of codewords to decode                   */
    const Word16 buffer_len,         /* i  : Number of bits to read                          */
    const Word16 num_lengths,        /* i  : Number of different huffman codeword lengths    */
    const Word16 *thres,             /* i  : Threshold of first codeword of each length      */
    const Word16 *offset,            /* i  : Offset for first codeword                       */
    const Word16 *huff_tab,          /* i  : Huffman table order by codeword lengths         */
    Word16 *index              /* o  : Decoded index                                   */
)
{
    Word16 i, j, k;
    UWord16 val;
    Word16 last_bits;

    last_bits = buffer_len;
    move16();

    val = 0;
    move16();
    j = 0;
    move16();
    FOR (i = 0; i < N; i++)
    {
        last_bits = sub(buffer_len, j);
        val = lshl(val, last_bits);
        val = s_and(val, sub(lshl(1, buffer_len), 1));
        val = s_or(val, get_next_indice_fx( st_fx, last_bits ));

        /* Find codeword length */
        j = sub(num_lengths, 1);
        WHILE ( sub(val, thres[j]) < 0 )
        {
            j = sub(j, 1);
        }
        k = lshr(sub(val, thres[j]), j);
        *index++ = huff_tab[ add(offset[j], k) ];
        move16();
    }

    /* Put back unused bits */
    st_fx->next_bit_pos_fx = sub(st_fx->next_bit_pos_fx, j);

    return;
}

/*--------------------------------------------------------------------------
 * hdecnrm_trans()
 *
 * Huffman decoding for indices of quantized norms
 *--------------------------------------------------------------------------*/

void hdecnrm_tran_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure   */
    const Word16 N,               /* i  : number of norms           */
    Word16 *index           /* o  : indices of quantized norms */
)
{
    Word16 i, j, k, n, m;
    Word16 temp;
    Word16 *pidx;
    Word16 l;

    pidx  = index;
    move16();

    m = sub(N, 1);
    FOR (i=0; i<m; i++)
    {
        j = 0;
        move16();
        k = 0;
        move16();
        if ( get_next_indice_1_fx(st_fx) != 0 )
        {
            j = 1;
            move16();
        }

        if ( get_next_indice_1_fx(st_fx) != 0 )
        {
            k = 1;
            move16();
        }

        /*n = k * 2 + j; */
        n= add(shl(k, 1), j);
        /*l = k * 4; */
        l = shl(k, 2);
        test();
        test();
        test();
        test();
        test();
        IF((j==0 && k==0) || (sub(j, 1)==0 && k==0) || (sub(j,1)==0 && sub(k,1)==0))
        {
            temp = sub(add(15, l), n);
        }
        ELSE
        {
            IF ( get_next_indice_1_fx(st_fx) != 0 )
            {
                temp = sub(add(15, n), l);
            }
            ELSE
            {
                temp = sub(add(15, l), n);
                IF ( get_next_indice_1_fx(st_fx) != 0 )
                {
                    FOR(k=0; k<3; )
                    {
                        IF( get_next_indice_1_fx(st_fx) != 0)
                        {
                            k = add(k,1);
                        }
                        ELSE
                        {
                            BREAK;
                        }
                    }

                    test();
                    IF( k==0 || sub(k, 3) == 0 )
                    {
                        temp = sub(temp, 5);
                        if( sub(k, 3) == 0 )
                        {
                            temp = sub(temp, 1);
                        }
                    }
                    ELSE IF( sub(k,1)==0 )
                    {
                        temp = add(temp, 1);
                    }
                    ELSE
                    {
                        temp = add(temp, 2);
                        IF ( get_next_indice_1_fx(st_fx) != 0 )
                        {
                            temp = add(temp, 1);
                            if ( get_next_indice_1_fx(st_fx) != 0 )
                            {
                                temp = add(temp, 1);
                            }
                        }
                    }
                }
            }
        }

        *pidx++ = temp;
        move16();
    }

    return;
}


