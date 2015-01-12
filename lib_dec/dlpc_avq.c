/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


/* Header files */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "stl.h"
#include "prot_fx.h"
/* Constants */

#define M 16   /* length of LPC */

#define BFI_FAC FL2WORD16(0.9f)


/***********************************************/
/* Variable bit-rate multiple LPC un-quantizer */
/***********************************************/

Word16 dlpc_avq(
    Word16 *index,       /* (i)   Quantization indices                       */
    Word16 *LSF_Q,     /* (o)   Quantized LSF vectors                      */
    Word16 numlpc,       /* (i) Number of sets of lpc */
    Word32 sr_core
)
{
    Word16 i, nbi, last;
    Word16 *p_index, q_type;


    /* Last LPC index */

    move16();
    last = 0;
    if ( sub(numlpc,1)!=0 )
    {
        move16();
        last = M;
    }

    move16();
    p_index = index;

    /* Decode last LPC */

    FOR (i=0; i<M; i++)
    {
        move16();
        LSF_Q[last+i] = 0;
    }
    vlpc_1st_dec(p_index[0], &LSF_Q[last]);
    p_index++;
    vlpc_2st_dec(&LSF_Q[last], &p_index[0], 0, sr_core);
    nbi = add(2, add(p_index[0], p_index[1]));
    p_index += nbi;

    /* Decode intermediate LPC (512 framing) */

    IF ( sub(numlpc,2)==0 )
    {
        move16();
        q_type = p_index[0];
        p_index++;

        IF (q_type == 0)
        {

            FOR (i=0; i<M; i++)
            {
                move16();
                LSF_Q[i] = 0;
            }
            vlpc_1st_dec(p_index[0], &LSF_Q[0]);
            p_index++;
            vlpc_2st_dec(&LSF_Q[0], &p_index[0], 0, sr_core);
        }
        ELSE IF ( sub(q_type,1) == 0 )
        {

            FOR (i=0; i<M; i++)
            {
                move16();
                LSF_Q[i] = LSF_Q[M+i];
            }
            vlpc_2st_dec(&LSF_Q[0], &p_index[0], 3, sr_core);
        }
        nbi = add(2, add(p_index[0], p_index[1]));
        p_index += nbi;
    }

    return (Word16)(p_index-index);
}

static Word16 unary_decode(Decoder_State_fx *st, Word16 *ind)
{

    Word16 start_bit_pos;


    move16();
    start_bit_pos = st->next_bit_pos_fx;

    /* Index bits */

    move16();
    *ind = 0;

    WHILE (get_next_indice_1_fx(st) != 0)
    {
        move16();
        *ind = add(*ind,1);
    }

    if (*ind != 0)
    {
        move16();
        *ind = add(*ind, 1);
    }

    return sub(st->next_bit_pos_fx, start_bit_pos);

}

static Word16 pack4bits(Word16 nbits, Decoder_State_fx *st, Word16 *prm)
{
    Word16 i;


    move16();
    i=0;

    FOR ( ; nbits > 4; nbits -= 4 )
    {
        move16();
        prm[i] = get_next_indice_fx(st, 4);
        i = add(i,1);
    }

    prm[i] = get_next_indice_fx(st, nbits);
    move16();
    i = add(i,1);

    return(i);
}

Word16 decode_lpc_avq( Decoder_State_fx *st, Word16 numlpc, Word16 *param_lpc )
{
    Word16 k,j;
    Word16 nb, qn1, qn2, avqBits, q_type;
    Word16 start_bit_pos;


    move16();
    move16();
    j = 0;
    start_bit_pos = st->next_bit_pos_fx;


    FOR (k=0; k<numlpc; k++)
    {
        /* Decode quantizer type */

        IF (k==0)
        {
            move16();
            move16();
            q_type = 0;
            nb = 0;
        }
        ELSE
        {
            move16();
            nb = 1;
            q_type = get_next_indice_fx(st, nb);
            move16();
            param_lpc[j] = q_type;
            j = add(j,1);
        }

        /* Decode quantization indices */

        IF (q_type==0)
        {
            /* Absolute quantizer with 1st stage stochastic codebook */
            move16();
            param_lpc[j] = get_next_indice_fx(st, 8);
            j = add(j,1);
        }

        /* 2 bits to specify Q2,Q3,Q4,ext */
        qn1 = add(2, get_next_indice_fx(st, 2));
        qn2 = add(2, get_next_indice_fx(st, 2));

        /* Unary code */
        /* Q5 = 0, Q6=10, Q0=110, Q7=1110, ... */

        IF ( sub(qn1,4) > 0 )
        {
            nb = unary_decode(st, &qn1);

            if ( sub(nb,1) == 0 )
            {
                qn1 = add(qn1, 5);
            }
            if (sub(nb,2) == 0)
            {
                qn1 = add(qn1, 4);
            }
            if ( sub(nb,3) == 0 )
            {
                move16();
                qn1 = 0;
            }
            if ( sub(nb,3) > 0 )
            {
                qn1 = add(qn1, 3);
            }
        }

        IF ( sub(qn2,4) > 0 )
        {
            nb = unary_decode(st, &qn2);

            if ( sub(nb,1) == 0 )
            {
                qn2 = add(qn2, 5);
            }
            if (sub(nb,2) == 0)
            {
                qn2 = add(qn2, 4);
            }
            if ( sub(nb,3) == 0 )
            {
                move16();
                qn2 = 0;
            }
            if ( sub(nb,3) > 0 )
            {
                qn2 = add(qn2, 3);
            }
        }
        move16();
        param_lpc[j] = qn1;
        j = add(j, 1);
        move16();
        param_lpc[j] = qn2;
        j = add(j, 1);

        /* Decode Split-by-2 algebraic VQ */
        avqBits = shl(qn1,2);
        pack4bits(avqBits, st, &param_lpc[j]);
        j = add(j, qn1);

        avqBits = shl(qn2,2);
        pack4bits(avqBits, st, &param_lpc[j]);
        j = add(j, qn2);
    }

    return sub(st->next_bit_pos_fx, start_bit_pos);
}
