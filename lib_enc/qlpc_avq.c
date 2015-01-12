/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "stl.h"
#include "control.h"
#include "prot_fx.h"


/* Prototypes */

void qlpc_avq(
    const Word16 *lsf,        /* (i) Input LSF vectors             (14Q1*1.28)    */
    const Word16 *lsfmid,     /* (i) Input LSF vectors             (14Q1*1.28)    */
    Word16 *lsf_q,      /* (o) Quantized LFS vectors         (14Q1*1.28)    */
    Word16 *lsfmid_q,   /* (o) Quantized LFS vectors         (14Q1*1.28)    */
    Word16 *index,      /* (o) Quantization indices                         */
    Word16 *nb_indices, /* (o) Number of quantization indices               */
    Word16 *nbbits,     /* (o) Number of quantization bits                  */
    const	Word16 core,        /* (i) TCX10 or TCX20                               */
    Word32 sr_core
)
{
    Word16 i;
    Word16 lsfmid_q0[M];
    Word16 *tmp_index, indxt[256], nit, nbits, nbt;
    Word16 dummy[M];


    /* Init */
    tmp_index = &index[0];
    *nb_indices = 0;
    move16();

    tmp_index[0] = vlpc_1st_cod(lsf, lsf_q, dummy, 0);

    nbt = vlpc_2st_cod(lsf, lsf_q, &tmp_index[1], 0, sr_core);

    /*nit = 1 + 2 + index[1] + index[2]; nit < NPRM_LPC_NEW(=50) */
    nit = add(add(3,index[1]),index[2]);
    assert(nit < NPRM_LPC_NEW);

    /*tmp_index += nit;*/
    tmp_index = tmp_index + nit;
    /**nb_indices += nit;*/
    *nb_indices = add(*nb_indices,nit);
    move16();
    /*nbbits[0] = 8 + nbt;*/
    nbbits[0] = add(8,nbt);
    move16();

    *tmp_index = 0;
    move16();

    IF (sub(core, TCX_20_CORE) == 0)
    {

        return;
    }

    /* Quantize intermediate LPC (512 framing) */
    tmp_index++;
    /**nb_indices +=1;*/
    *nb_indices = add(*nb_indices,1);
    move16();

    /* LPC2: Abs? */
    tmp_index[0] = vlpc_1st_cod(lsfmid, lsfmid_q, dummy, 0);

    nbits = vlpc_2st_cod(lsfmid, lsfmid_q, &tmp_index[1], 0, sr_core);
    /*nbt = 8 + nbits;*/
    nbt = add(8,nbits);
    /*nit = 1 + 2 + tmp_index[1] + tmp_index[2];*/
    nit = add(add(3,tmp_index[1]),tmp_index[2]);

    /* LPC2: RelR? */
    FOR (i=0; i<M; i++)
    {
        lsfmid_q0[i] = lsf_q[i];
        move16();
    }
    nbits = vlpc_2st_cod(lsfmid, lsfmid_q0, indxt, 3, sr_core);

    IF (sub(nbits,nbt) < 0)
    {
        nbt = nbits;
        move16();
        /*nit = 2 + indxt[0] + indxt[1];*/
        nit = add(add(2,indxt[0]),indxt[1]);
        tmp_index[-1] = 1;
        move16();

        FOR (i=0; i<M; i++)
        {
            lsfmid_q[i] = lsfmid_q0[i];
            move16();
        }

        FOR (i=0; i<nit; i++)
        {
            tmp_index[i] = indxt[i];
            move16();
        }
    }

    tmp_index += nit;
    /**nb_indices += nit;*/
    *nb_indices = add(*nb_indices,nit);
    move16();
    /*nbbits[1] = 1 + nbt;*/
    nbbits[1] = add(1,nbt);
    move16();


    return;
}

static Word16 unary_code(Word16 ind, Encoder_State_fx *st)
{
    Word16 nb_bits;

    move16();
    nb_bits = 1;

    /* Index bits */
    ind = sub(ind, 1);

    FOR ( ; ind > 0; ind--)
    {
        push_next_indice_fx(st, 1, 1);
        nb_bits = add(nb_bits, 1);
    }

    /* Stop bit */
    push_next_indice_fx(st, 0, 1);

    return(nb_bits);
}

static Word16 unpack4bits(Word16 nbits, const Word16 *prm, Encoder_State_fx *st)
{
    Word16 i;

    IF (nbits == 0)
    {
        push_next_indice_fx(st, 0, 0);
        i = 1;
        move16();
    }
    ELSE
    {
        move16();
        i=0;

        FOR ( ; nbits > 4; nbits -= 4)
        {
            push_next_indice_fx(st, prm[i], 4);
            i = add(i, 1);
        }
        push_next_indice_fx(st, prm[i], nbits);
        i = add(i, 1);
    }

    return(i);
}
Word16 encode_lpc_avq( Encoder_State_fx *st, Word16 numlpc, Word16 *param_lpc, Word16 mode )
{
    Word16 k,j;
    Word16 q_type, nb_ind;
    Word16 i,qn1,qn2,nb,avqBits,st1;
    Word16 nb_bits;

    move16();
    move16();
    move16();
    st1=0;
    j = 0;
    nb_bits = 0;

    FOR (k=0; k<numlpc; k++)
    {
        /* Retrieve quantizer type */


        move16();
        q_type = 0;
        IF (k!=0)
        {
            move16();
            q_type = param_lpc[j];
            j = add(j,1);
        }

        /* Determine number of AVQ indices */
        move16();
        nb_ind = 0;

        if (q_type==0)
        {
            move16();
            st1 = param_lpc[j++];
        }
        move16();
        move16();
        qn1 = param_lpc[j++];
        qn2 = param_lpc[j++];
        nb_ind = add(qn1, qn2);

        IF ( s_or(k==0, s_and(k==1, mode!=1)) )
        {
            /* Encode quantizer type */


            move16();
            nb = 0;
            IF (k!=0)
            {
                nb = 1;
                push_next_indice_fx(st, q_type, nb);
            }
            nb_bits = add(nb_bits, nb);

            /* Encode quantizer data */

            IF (q_type==0)
            {
                /* Absolute quantizer with 1st stage stochastic codebook */
                push_next_indice_fx(st, st1, 8);
                nb_bits = add(nb_bits, 8);
            }

            /* 2 bits to specify Q2,Q3,Q4,ext */
            nb_bits = add(nb_bits, 4);
            i = sub(qn1, 2);

            if ( s_or(i<0, sub(i,3)>0) )
            {
                move16();
                i = 3;
            }
            push_next_indice_fx(st, i, 2);

            i = sub(qn2, 2);

            if ( s_or(i<0, sub(i,3)>0) )
            {
                move16();
                i = 3;
            }
            push_next_indice_fx(st, i, 2);

            /* Unary code for abs and rel LPC0/LPC2 */
            /* Q5 = 0, Q6=10, Q0=110, Q7=1110, ... */
            move16();
            nb = qn1;

            IF ( sub(nb,6) > 0)
            {
                nb = sub(nb, 3);
            }
            ELSE IF ( sub(nb,4) > 0)
            {
                nb = sub(nb, 4);
            }
            ELSE IF (nb == 0)
            {
                move16();
                nb = 3;
            }
            ELSE
            {
                move16();
                nb = 0;
            }

            IF (nb > 0)
            {
                unary_code(nb, st);
            }
            nb_bits = add(nb_bits, nb);

            move16();
            nb = qn2;

            IF ( sub(nb,6) > 0)
            {
                nb = sub(nb, 3);
            }
            ELSE IF ( sub(nb,4) > 0)
            {
                nb = sub(nb, 4);
            }
            ELSE IF (nb == 0)
            {
                move16();
                nb = 3;
            }
            ELSE
            {
                move16();
                nb = 0;
            }

            IF (nb > 0)
            {
                unary_code(nb, st);
            }
            nb_bits = add(nb_bits, nb);

            avqBits = shl(qn1,2);
            unpack4bits(avqBits, &param_lpc[j], st);
            j = add(j, qn1);
            nb_bits = add(nb_bits, avqBits);

            avqBits = shl(qn2, 2);
            unpack4bits(avqBits, &param_lpc[j], st);
            j = add(j, qn2);
            nb_bits = add(nb_bits, avqBits);
        }
        ELSE
        {
            j = add(j, nb_ind);
        }
    }

    return(nb_bits);
}

