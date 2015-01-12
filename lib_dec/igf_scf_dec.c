/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "stat_dec_fx.h"
#include "basop_util.h"


/**********************************************************************/ /**
initialization of an instance of this module
**************************************************************************/
void IGFSCFDecoderOpen(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData,         /* i/o: handle to public data */
    Word16                                  scfCountLongBlock,   /* i: number of SCFs for a long block */
    Word32                                  bitRate,             /* i: bitrate in bps */
    Word16                                  mode                 /* i: operating mode */
    ,Word16                                  rf_mode
)
{


    hPublicData->scfCountLongBlock = scfCountLongBlock;
    move16();
    hPublicData->t = 0;
    move16(); /* protect against the invalid request of starting decoding with a dependent block */

    IGFCommonFuncsIGFGetCFTables(
        bitRate,
        mode,
        rf_mode,
        &hPublicData->cf_se00,
        &hPublicData->cf_se01,
        &hPublicData->cf_off_se01,
        &hPublicData->cf_se02,
        &hPublicData->cf_off_se02,
        &hPublicData->cf_se10,
        &hPublicData->cf_off_se10,
        &hPublicData->cf_se11,
        &hPublicData->cf_off_se11
    );

}


static Word16 quant_ctx_fx(
    Word16 ctx  /* i: the context value to be quantized */
)
{
    /*
      ctx ... -5 -4 -3 -2 -1 0 1 2 3 4 5 ...
    Q(ctx)... -3 -3 -3 -2 -1 0 1 2 3 3 3 ...
    */
    Word16 result;


    result = s_min(abs_s(ctx), IGF_CTX_OFFSET); /* limit the absolute value to IGF_CTX_OFFSET */
    if (ctx < 0)   /* add the sign back, if needed */
    {
        result = negate(result);
    }

    return result;
}

static Word16 arith_decode_bits_fx(
    IGFSCFDEC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Decoder_State_fx *st,                    /* i/o: pointer to bitstream decoder state */
    Word16 nBits                             /* i: number of bits to decode */
)
{
    Word16 i;
    Word16 x;
    Word16 bit;


    x = 0;
    move16();
    FOR (i = 0; i < nBits; ++i)   /* nBits > 0 */
    {
        x = lshl(x, 1);
        /* decode one bit using the new raw AC function */
        bit = ari_decode_14bits_bit_ext(st, &hPrivateData->acState);
        if (bit != 0)
        {
            x = s_or(x, 1);
        }
    }

    return x;
}

static Word16 arith_decode_residual_fx(
    IGFSCFDEC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Decoder_State_fx *st,                    /* i/o: pointer to decoder state */
    const Word16 *cumulativeFrequencyTable,  /* i: cumulative frequency table to be used */
    Word16 tableOffset                       /* i: offset used to align the table */
)
{
    Word16 val;
    Word16 x = 0; /* to avoid a compiler warning (potentially uninitialized local variable used) */
    Word16 extra;


    /* decode one of the IGF_SYMBOLS_IN_TABLE == 27 alphabet symbols using the new raw AC function */
    val = ari_decode_14bits_s27_ext(st, &hPrivateData->acState, (const UWord16*) cumulativeFrequencyTable);

    /* meaning of the values of val: */
    /* esc_{0} IGF_MIN_ENC_SEPARATE ... IGF_MAX_ENC_SEPARATE esc_{IGF_SYMBOLS_IN_TABLE - 1} */
    test();
    IF ((val != 0) && (sub(val, IGF_SYMBOLS_IN_TABLE - 1) != 0))
    {
        x = add(val, - 1 + IGF_MIN_ENC_SEPARATE); /* (val - 1) + IGF_MIN_ENC_SEPARATE */


        x = sub(x, tableOffset);

        return x;
    }

    /* decode one of the tails of the distribution */
    /* decode extra with 4 bits */
    extra = arith_decode_bits_fx(hPrivateData, st, 4);
    IF (sub(extra, 15) == 0)   /* escape code 15 to indicate extra >= 15 */
    {
        /* decode addtional extra with 6 bits */
        extra = arith_decode_bits_fx(hPrivateData, st, 6);
        IF (sub(extra, 63) == 0)   /* escape code 63 to indicate extra >= 63 */
        {
            /* decode safety extra with 7 bits */
            extra = arith_decode_bits_fx(hPrivateData, st, 7);
            extra = add(63, extra);
        }
        extra = add(15, extra);
    }

    if (val == 0)
    {
        /* escape code 0 to indicate x <= IGF_MIN_ENC_SEPARATE - 1 */
        x = sub(IGF_MIN_ENC_SEPARATE - 1, extra);
    }
    if (sub(val, IGF_SYMBOLS_IN_TABLE - 1) == 0)
    {
        /* escape code (IGF_SYMBOLS_IN_TABLE - 1) to indicate x >= IGF_MAX_ENC_SEPARATE + 1 */
        x = add(IGF_MAX_ENC_SEPARATE + 1, extra);
    }

    x = sub(x, tableOffset);

    return x;
}

static void arith_decode_flush_fx(
    Decoder_State_fx *st  /* i/o: pointer to decoder state */
)
{

    get_next_indice_tmp_fx(st, -14); /* return back the least significant 14 bits to the bitstream */

}

static void decode_sfe_vector_fx(
    IGFSCFDEC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Decoder_State_fx *st,                    /* i/o: pointer to decoder state */
    Word16 t,                                /* i: counter reset to 0 at each independent block */
    Word16 *prev_x,                          /* i: previous vector */
    Word16 *x,                               /* o: current vector to decode */
    Word16 length                            /* i: number of elements to decode */
)
{
    /*
       f
       ^
       |  d a x
       |    c b
       |      e  --> t
    */
    Word16 f;
    Word16 pred;
    Word16 res;
    Word16 ctx;
    Word16 ctx_f;
    Word16 ctx_t;
    Word16 prev_offset;
    Word32 index1;
    Word32 index2;



    FOR (f = 0; f < length; ++f)
    {
        IF (t == 0)
        {
            IF (f == 0)   /* (t == 0) && (f == 0) */
            {
                /* decode one of the IGF_SYMBOLS_IN_TABLE == 27 alphabet symbols using the new raw AC function */
                res = ari_decode_14bits_s27_ext(st, &hPrivateData->acState, (const UWord16*) hPrivateData->cf_se00);

                pred = arith_decode_bits_fx(hPrivateData, st, 2); /* LSBs as 2 bit raw */
                x[f] = add(shl(res, 2), pred);
                move16();
            }
            ELSE if (sub(f, 1) == 0)   /* (t == 0) && (f == 1) */
            {
                res = arith_decode_residual_fx(hPrivateData,
                                               st,
                                               hPrivateData->cf_se01,
                                               hPrivateData->cf_off_se01);
                x[f] = add(x[0], res);
                move16(); /* f - increment is 0, pred = b */
            }
            ELSE   /* (t == 0) && (f >= 2) */
            {
                prev_offset = sub(f, 1);
                ctx = quant_ctx_fx(sub(x[prev_offset], x[sub(prev_offset, 1)])); /* Q(b - e) */
                /* index1 is (IGF_SYMBOLS_IN_TABLE + 1) * (CTX_OFFSET + ctx) */
                index1 = L_mac0((IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_OFFSET, (IGF_SYMBOLS_IN_TABLE + 1), ctx);
                /* index2 is IGF_CTX_OFFSET + ctx */
                index2 = L_mac0(IGF_CTX_OFFSET, 1, ctx);
                res = arith_decode_residual_fx(hPrivateData,
                st,
                hPrivateData->cf_se02 + index1,
                hPrivateData->cf_off_se02[index2]);
                x[f] = add(x[prev_offset], res);
                move16(); /* pred = b */
            }
        }
        ELSE   /* t == 1 */
        {
            IF (f == 0)   /* (t == 1) && (f == 0) */
            {
                res = arith_decode_residual_fx(hPrivateData,
                st,
                hPrivateData->cf_se10,
                hPrivateData->cf_off_se10);
                x[f] = add(prev_x[f], res);
                move16(); /* pred = a */
            }
            ELSE { /* (t == 1) && (f >= 1) */
                prev_offset = sub(f, 1);
                pred = add(prev_x[f], x[prev_offset]);
                pred = sub(pred, prev_x[prev_offset]); /* pred = a + b - c */
                ctx_f = quant_ctx_fx(sub(prev_x[f], prev_x[prev_offset])); /* Q(a - c) */
                ctx_t = quant_ctx_fx(sub(x[prev_offset], prev_x[prev_offset])); /* Q(b - c) */
                /* index1 is (IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT * (IGF_CTX_OFFSET + ctx_t)
                   + (IGF_SYMBOLS_IN_TABLE + 1) * (IGF_CTX_OFFSET + ctx_f) */
                index1 = L_mac0(
                    ((IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT + (IGF_SYMBOLS_IN_TABLE + 1)) * IGF_CTX_OFFSET,
                    (IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT, ctx_t);
                index1 = L_mac0(index1, (IGF_SYMBOLS_IN_TABLE + 1), ctx_f);
                /* index2 is IGF_CTX_COUNT * (IGF_CTX_OFFSET + ctx_t) + (IGF_CTX_OFFSET + ctx_f) */
                index2 = L_mac0((IGF_CTX_COUNT + 1) * IGF_CTX_OFFSET, IGF_CTX_COUNT, ctx_t);
                index2 = L_mac0(index2, 1, ctx_f);
                res = arith_decode_residual_fx(hPrivateData,
                st,
                hPrivateData->cf_se11 + index1,
                hPrivateData->cf_off_se11[index2]);
                x[f] = add(pred, res);
                move16();
            }
        }
    }

}

/**********************************************************************/ /**
resets the internal decoder memory (context memory)
**************************************************************************/
void IGFSCFDecoderReset(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
)
{


    /* reset of coder */
    hPublicData->t = 0; /* indicate that an independent block follows */
    /* we do not need to fill hPublicData->prev with zeros, because when t = 0 no previous information is used */

}

/**********************************************************************/ /**
main decoder function
**************************************************************************/
void IGFSCFDecoderDecode(
    IGFSCFDEC_INSTANCE_HANDLE               hPublicData,      /* i/o: handle to public data */
    Decoder_State_fx                       *st,               /* i/o: pointer to decoder state */
    Word16                                 *sfe,              /* o: pointer to an array which will contain the decoded quantized SCFs */
    Word16                                  indepFlag         /* i: 1 if the block is an independent block, 0 otherwise */
)
{



    /* insert data */
    hPublicData->bitsRead = st->next_bit_pos_fx;
    move16();
    ari_start_decoding_14bits(st, &hPublicData->acState); /* start AC decoding */

    /* check if coder needs a reset and do it if necessary */
    IF (indepFlag != 0)
    {
        /* reset of coder */
        IGFSCFDecoderReset(hPublicData);
    }

    decode_sfe_vector_fx(hPublicData,
                         st,
                         hPublicData->t,
                         hPublicData->prev,
                         sfe,
                         hPublicData->scfCountLongBlock
                        );

    arith_decode_flush_fx(st); /* finish AC decoding */


    /* advance history */
    Copy(sfe, hPublicData->prev, hPublicData->scfCountLongBlock);
    hPublicData->t = add(hPublicData->t, 1);

    hPublicData->bitsRead = sub(st->next_bit_pos_fx, hPublicData->bitsRead);

}
