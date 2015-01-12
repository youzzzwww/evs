/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "stat_enc_fx.h"
#include "stat_com.h"
#include "cnst_fx.h"
#include "basop_util.h"


/**********************************************************************/ /**
initialization of an instance of this module
**************************************************************************/
void IGFSCFEncoderOpen(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData,         /* i/o: handle to public data */
    Word16                                  scfCountLongBlock,   /* i: number of SCFs for a long block */
    Word32                                  bitRate,             /* i: bitrate in bps */
    Word16                                  mode                 /* i: operating mode */
    , Word16                                  rf_mode             /**< in: flag to signal the RF mode */
)
{


    hPublicData->ptrBitIndex       = 0;
    move16();
    hPublicData->bitCount          = 0;
    move16();
    hPublicData->tSave             = 0;
    move16();
    hPublicData->context_saved     = 0;
    move16();
    hPublicData->acState.low       = 0;
    move32();
    hPublicData->acState.high      = 0;
    move32();
    hPublicData->acState.vobf      = 0;
    move16();
    set16_fx(hPublicData->prev, 0, 64);
    set16_fx(hPublicData->prevSave, 0, 64);

    hPublicData->scfCountLongBlock = scfCountLongBlock;
    move16();
    hPublicData->t                 = 0;
    move16(); /* protect against the invalid request of starting encoding with a dependent block */

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


static Word16 quant_ctx(
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



static void arith_encode_bits(
    IGFSCFENC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Word16 *ptr,                             /* i/o: pointer to expanded bit buffer, one bit in each Word16 */
    Word16 x,                                /* i: value to encode */
    Word16 nBits                             /* i: number of bits to encode */
)
{
    Word16 i;
    Word16 bit;


    FOR (i = nBits - 1; i >= 0; --i)   /* nBits > 0 */
    {
        bit = s_and(shr(x, i), 1);
        hPrivateData->ptrBitIndex = ari_encode_14bits_sign(ptr,
                                    hPrivateData->ptrBitIndex,
                                    32767, /* disable the bit count limitation */
                                    &hPrivateData->acState,
                                    bit
                                                          );
    }

}

static void arith_encode_residual(
    IGFSCFENC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Word16 *ptr,                             /* i/o: pointer to expanded bit buffer, one bit in each Word16 */
    Word16 x,                                /* i: residual value to encode */
    const Word16 *cumulativeFrequencyTable,  /* i: cumulative frequency table to be used */
    Word16 tableOffset                       /* i: offset used to align the table */
)
{
    Word16 extra;
    Word16 extra_tmp;
    Word16 extra_safety;


    x = add(x, tableOffset);

    test();
    IF ((sub(x, IGF_MIN_ENC_SEPARATE) >= 0) && (sub(x, IGF_MAX_ENC_SEPARATE) <= 0))
    {
        x = sub(x, IGF_MIN_ENC_SEPARATE - 1); /* (x - IGF_MIN_ENC_SEPARATE) + 1 */
        /* encode one of the IGF_SYMBOLS_IN_TABLE == 27 alphabet symbols using the new raw AC function */
        hPrivateData->ptrBitIndex = ari_encode_14bits_ext(ptr,
                                    hPrivateData->ptrBitIndex,
                                    &hPrivateData->acState,
                                    x,
                                    (const UWord16*) cumulativeFrequencyTable
                                                         );

        return;
    }

    IF (sub(x, IGF_MIN_ENC_SEPARATE) < 0)
    {
        /* send escape code 0 to indicate x <= IGF_MIN_ENC_SEPARATE - 1 */
        extra = sub(IGF_MIN_ENC_SEPARATE - 1, x);
        hPrivateData->ptrBitIndex = ari_encode_14bits_ext(ptr,
                                    hPrivateData->ptrBitIndex,
                                    &hPrivateData->acState,
                                    0,
                                    (const UWord16*) cumulativeFrequencyTable
                                                         );
    }
    ELSE   /* x > IGF_MAX_ENC_SEPARATE */
    {
        /* send escape code (IGF_SYMBOLS_IN_TABLE - 1) to indicate x >= IGF_MAX_ENC_SEPARATE + 1 */
        extra = sub(x, IGF_MAX_ENC_SEPARATE + 1);
        hPrivateData->ptrBitIndex = ari_encode_14bits_ext(ptr,
        hPrivateData->ptrBitIndex,
        &hPrivateData->acState,
        IGF_SYMBOLS_IN_TABLE - 1,
        (const UWord16*) cumulativeFrequencyTable
                                                         );
    }

    /* encode one of the tails of the distribution */
    extra_tmp = sub(extra, 15);
    IF (extra_tmp < 0)
    {
        /* encode extra with 4 bits if extra < 15 */
        arith_encode_bits(hPrivateData, ptr, extra, 4);
    }
    ELSE   /* extra >= 15 */
    {
        /* send escape code 15 to indicate extra >= 15 */
        arith_encode_bits(hPrivateData, ptr, 15, 4);

        extra_safety = sub(extra_tmp, 63);
        IF (extra_safety < 0)
        {
            /* encode additional extra with 6 bits */
            arith_encode_bits(hPrivateData, ptr, extra_tmp, 6);
        }
        ELSE { /* extra_tmp >= 63 */
            /* encode safety extra with 7 bits */
            arith_encode_bits(hPrivateData, ptr, extra_safety, 7);
        }
    }

}


static void encode_sfe_vector(
    IGFSCFENC_INSTANCE_HANDLE hPrivateData,  /* i/o: instance handle */
    Word16 *ptr,                             /* i/o: pointer to expanded bit buffer, one bit in each Word16 */
    Word16 t,                                /* i: counter reset to 0 at each independent block */
    Word16 *prev_x,                          /* i: previous vector */
    Word16 *x,                               /* i: current vector to encode */
    Word16 length                            /* i: number of elements to encode */
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
                /* encode one of the IGF_SYMBOLS_IN_TABLE == 27 alphabet symbols using the new raw AC function */
                hPrivateData->ptrBitIndex = ari_encode_14bits_ext(ptr,
                                            hPrivateData->ptrBitIndex,
                                            &hPrivateData->acState,
                                            shr(x[f], 2),
                                            (const UWord16*) hPrivateData->cf_se00
                                                                 );
                arith_encode_bits(hPrivateData, ptr, s_and(x[f], 3), 2); /* LSBs as 2 bit raw */
            }
            ELSE if (sub(f, 1) == 0)   /* (t == 0) && (f == 1) */
            {
                res = sub(x[f], x[0]); /* pred = b */
                arith_encode_residual(hPrivateData,
                                      ptr,
                                      res,
                                      hPrivateData->cf_se01,
                                      hPrivateData->cf_off_se01);
            }
            ELSE   /* (t == 0) && (f >= 2) */
            {
                prev_offset = sub(f, 1);
                res = sub(x[f], x[prev_offset]);
                move16(); /* pred = b */
                ctx = quant_ctx(sub(x[prev_offset], x[sub(prev_offset, 1)])); /* Q(b - e) */
                /* index1 is (IGF_SYMBOLS_IN_TABLE + 1) * (CTX_OFFSET + ctx) */
                index1 = L_mac0((IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_OFFSET, (IGF_SYMBOLS_IN_TABLE + 1), ctx);
                /* index2 is IGF_CTX_OFFSET + ctx */
                index2 = L_mac0(IGF_CTX_OFFSET, 1, ctx);
                arith_encode_residual(hPrivateData,
                ptr,
                res,
                hPrivateData->cf_se02 + index1,
                hPrivateData->cf_off_se02[index2]);
            }
        }
        ELSE   /* t == 1 */
        {
            IF (f == 0)   /* (t == 1) && (f == 0) */
            {
                res = sub(x[f], prev_x[f]);
                move16(); /* pred = a */
                arith_encode_residual(hPrivateData,
                ptr,
                res,
                hPrivateData->cf_se10,
                hPrivateData->cf_off_se10);
            }
            ELSE { /* (t == 1) && (f >= 1) */
                prev_offset = sub(f, 1);
                pred = add(prev_x[f], x[prev_offset]);
                pred = sub(pred, prev_x[prev_offset]); /* pred = a + b - c */
                res = sub(x[f], pred);
                ctx_f = quant_ctx(sub(prev_x[f], prev_x[prev_offset])); /* Q(a - c) */
                ctx_t = quant_ctx(sub(x[prev_offset], prev_x[prev_offset])); /* Q(b - c) */
                /* index1 is (IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT * (IGF_CTX_OFFSET + ctx_t)
                   + (IGF_SYMBOLS_IN_TABLE + 1) * (IGF_CTX_OFFSET + ctx_f) */
                index1 = L_mac0(
                    ((IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT + (IGF_SYMBOLS_IN_TABLE + 1)) * IGF_CTX_OFFSET,
                    (IGF_SYMBOLS_IN_TABLE + 1) * IGF_CTX_COUNT, ctx_t);
                index1 = L_mac0(index1, (IGF_SYMBOLS_IN_TABLE + 1), ctx_f);
                /* index2 is IGF_CTX_COUNT * (IGF_CTX_OFFSET + ctx_t) + (IGF_CTX_OFFSET + ctx_f) */
                index2 = L_mac0((IGF_CTX_COUNT + 1) * IGF_CTX_OFFSET, IGF_CTX_COUNT, ctx_t);
                index2 = L_mac0(index2, 1, ctx_f);
                arith_encode_residual(hPrivateData,
                ptr,
                res,
                hPrivateData->cf_se11 + index1,
                hPrivateData->cf_off_se11[index2]);
            }
        }
    }

}


/**********************************************************************/ /**
resets the internal encoder memory (context memory)
**************************************************************************/
void IGFSCFEncoderReset(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
)
{


    /* reset of coder */
    hPublicData->t = 0;
    move16(); /* indicate that an independent block follows */
    /* we do not need to fill hPublicData->prev with zeros, because when t = 0 no previous information is used */

}

/**********************************************************************/ /**
main encoder function
**************************************************************************/
Word16 IGFSCFEncoderEncode(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData,      /* i/o: handle to public data */
    Encoder_State_fx                       *st,               /* i/o: pointer to encoder state */
    Word16                                  bitCount,         /* i: offset to the first bit in bitbuffer which should be written by the raw AC functions */
    Word16                                 *sfe,              /* i: pointer to an array which contains the quantized SCF energies to be encoded */
    Word16                                  indepFlag,        /* i: 1 if the block is an independent block, 0 otherwise */
    Word16                                  doRealEncoding    /* i: whether the real encoding is needed, otherwise only the number of bits is used */
)
{
    Word16 ptr[BITBUFSIZE];
    Word16 i;



    /* insert data: */
    hPublicData->ptrBitIndex = 0;
    move16();
    hPublicData->bitCount = bitCount;
    move16();
    ari_start_encoding_14bits(&hPublicData->acState); /* start AC encoding */

    /* check if coder needs a reset and do it if necessary */
    IF (indepFlag != 0)
    {
        /* reset of coder */
        IGFSCFEncoderReset(hPublicData);
    }

    encode_sfe_vector(hPublicData,
                      ptr,
                      hPublicData->t,
                      hPublicData->prev,
                      sfe,
                      hPublicData->scfCountLongBlock
                     );

    hPublicData->ptrBitIndex = ari_done_encoding_14bits(ptr,
                               hPublicData->ptrBitIndex,
                               &hPublicData->acState
                                                       ); /* finish AC encoding */
    hPublicData->bitCount = add(hPublicData->bitCount, hPublicData->ptrBitIndex);


    /* advance history */
    Copy(sfe, hPublicData->prev, hPublicData->scfCountLongBlock);
    hPublicData->t = add(hPublicData->t, 1);


    /* copy the bits from the temporary bit buffer, if doRealEncoding is enabled */
    IF (doRealEncoding != 0)
    {
        FOR (i = 0; i < hPublicData->ptrBitIndex; ++i)
        {
            push_next_indice_fx(st, ptr[i], 1);
        }
    }

    /* return next bit offset in the stream */
    return hPublicData->bitCount;
}

/**********************************************************************/ /**
for a closed loop encoder, the SCF encoder needs to memorize the context
**************************************************************************/
void IGFSCFEncoderSaveContextState(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
)
{


    hPublicData->tSave = hPublicData->t;
    move16();

    Copy(hPublicData->prev,
         hPublicData->prevSave,
         hPublicData->scfCountLongBlock
        );


}

/**********************************************************************/ /**
for a closed loop encoder, the SCF encoder needs to memorize the context
**************************************************************************/
void IGFSCFEncoderRestoreContextState(
    IGFSCFENC_INSTANCE_HANDLE               hPublicData  /* i/o: handle to public data */
)
{


    hPublicData->t = hPublicData->tSave;
    move16();

    Copy(hPublicData->prevSave,
         hPublicData->prev,
         hPublicData->scfCountLongBlock
        );


}
