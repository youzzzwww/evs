/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "stl.h"
#include <assert.h>
#include "prot_fx.h"

/********************************/
/*      Helper functions        */
/********************************/

/** Put nBits long encoded value from *pStream into bitstream. Using the function EncodeValue for encoding. */
static Word16 PutIntoBitstream(Word16 const ** pStream, TEncodeValue EncodeValue, Word16 index, Encoder_State_fx *st_fx, Word16 nBits)
{
    Word16 value;
    Word16 codedValue;

    move16();
    value = *(*pStream)++;
    codedValue = EncodeValue(value, index);

    push_next_indice_fx(st_fx, codedValue, nBits);

    return value;
}


/** Get nBits long value from bitstream into *pStream. */
static Word16 GetFromBitstream(Decoder_State_fx *st, TDecodeValue DecodeValue, Word16 index, Word16 nFixedBits, Word16 ** pStream)
{
    Word16 value;

    move16();
    move16();
    value = 0;

    IF (DecodeValue != NULL)
    {
        DecodeValue(st, index, &value);
    }
    ELSE
    {
        value = get_next_indice_fx(st, nFixedBits);
    }
    move16();
    *(*pStream)++ = value;

    return value;
}

static Word16 FixedWidthEncoding(Word16 value, Word16 index)
{
    (void)index;
    return value;
}

/********************************/
/*      Interface functions     */
/********************************/

void GetParameters(ParamsBitMap const * paramsBitMap, Word16 nArrayLength, void const * pParameter, Word16 ** pStream, Word16 * pnSize, Word16 * pnBits)
{
    Word16 index;
    Word16 iParam, nParams;
    Word16 value;
    void const * pSubStruct;


    assert((paramsBitMap != NULL) && (nArrayLength > 0) && (pParameter != NULL) && (pStream != NULL) && (pnSize != NULL) && (pnBits != NULL));

    move16();
    nParams = paramsBitMap->nParams;

    FOR (index = 0; index < nArrayLength; index++)
    {

        FOR (iParam = 0; iParam < nParams; iParam++)
        {
            ParamBitMap const * param;

            move16();
            param = & paramsBitMap->params[iParam];

            pSubStruct = param->GetParamValue(pParameter, index, &value);
            /* If a function for encoding/decoding value is defined than it should take care of 0 */
            IF ( s_or(param->fZeroAllowed != 0, param->EncodeValue != NULL) )
            {
                move16();
                *(*pStream)++ = value;
            }
            ELSE
            {
                move16();
                *(*pStream)++ = sub(value, 1);
            }

            move16();
            *pnSize = add(*pnSize, 1);

            IF (param->nBits != 0)
            {
                move16();
                *pnBits = add(*pnBits, param->nBits);
            }
            ELSE
            {
                move16();
                *pnBits = add(*pnBits, param->GetNumberOfBits(value, index));
            }

            IF ( s_and(param->pSubParamBitMap != NULL, value > 0) )
            {
                const void *pointer;

                move16();
                pointer = pParameter;
                if (pSubStruct != NULL)
                {
                    move16();
                    pointer = pSubStruct;
                }
                GetParameters(param->pSubParamBitMap, value, pointer, pStream, pnSize, pnBits);
            }
        }
    }

}

void SetParameters(ParamsBitMap const * paramsBitMap, Word16 nArrayLength, void * pParameter, Word16 const ** pStream, Word16 * pnSize)
{
    Word16 index;
    Word16 iParam, nParams;
    Word16 value;
    void * pSubStruct;
    void * pTmp;
    assert((paramsBitMap != NULL) && (nArrayLength > 0) && (pParameter != NULL) && (pStream != NULL) && (pnSize != NULL));
    nParams = paramsBitMap->nParams;

    FOR (index = 0; index < nArrayLength; index++)
    {
        FOR (iParam = 0; iParam < nParams; iParam++)
        {
            ParamBitMap const *param;
            /* If a function for encoding/decoding value is defined than it should take care of 0 */

            move16();
            param = &paramsBitMap->params[iParam];

            move16();
            value = 1;
            if ( s_or(param->fZeroAllowed!=0, param->EncodeValue != NULL) )
            {
                move16();
                value = 0;
            }
            value = add(value, *(*pStream)++);

            pSubStruct = param->SetParamValue(pParameter, index, value);
            move16();
            *pnSize = add(*pnSize, 1);

            IF ( s_and(param->pSubParamBitMap != NULL, value > 0) )
            {
                pTmp = pParameter;
                if(pSubStruct != NULL) pTmp = pSubStruct;
                SetParameters(param->pSubParamBitMap, value, pTmp, pStream, pnSize);
            }
        }
    }

}

void WriteToBitstream(ParamsBitMap const * paramsBitMap, Word16 nArrayLength, Word16 const ** pStream, Word16 * pnSize, Encoder_State_fx *st, Word16 * pnBits)
{
    Word16 index;
    Word16 iParam, nParams;
    assert((paramsBitMap != NULL) && (nArrayLength > 0) && (pStream != NULL) && (pnSize != NULL) && (st != NULL) && (pnBits != NULL));
    nParams = paramsBitMap->nParams;

    FOR (index = 0; index < nArrayLength; index++)
    {

        FOR (iParam = 0; iParam < nParams; iParam++)
        {
            ParamBitMap const *param;
            Word16 nBits;
            /* If a function for encoding/decoding value is defined than it should take care of 0 */
            Word16 fShiftValue;
            TEncodeValue EncodeValue;
            Word16 value;

            move16();
            param = &paramsBitMap->params[iParam];

            move16();
            nBits = param->nBits;
            IF (param->nBits == 0)
            {
                nBits = param->GetNumberOfBits(**pStream, index);
            }

            test();
            test();
            fShiftValue = s_and(param->fZeroAllowed==0, param->EncodeValue == NULL);
            move16();
            EncodeValue = param->EncodeValue;
            if (param->EncodeValue == NULL)
            {
                move16();
                EncodeValue = &FixedWidthEncoding;
            }
            value = PutIntoBitstream(pStream, EncodeValue, index, st, nBits);
            if (fShiftValue)
            {
                value = add(value, 1);
            }

            move16();
            *pnSize = add(*pnSize, 1);
            move16();
            *pnBits = add(*pnBits, nBits);

            IF ((param->pSubParamBitMap != NULL) && (value > 0))
            {
                WriteToBitstream(param->pSubParamBitMap, value, pStream, pnSize, st, pnBits);
            }

        }
    }
}

void ReadFromBitstream(ParamsBitMap const * paramsBitMap, Word16 nArrayLength, Decoder_State_fx *st, Word16 ** pStream, Word16 * pnSize)
{
    Word16 index;
    Word16 iParam, nParams;
    Word16 fShiftValue;
    Word16 value;
    assert((paramsBitMap != NULL) && (nArrayLength > 0) && (pStream != NULL) && (pnSize != NULL) && (st != NULL));
    move16();
    nParams = paramsBitMap->nParams;

    FOR (index = 0; index < nArrayLength; index++)
    {

        FOR (iParam = 0; iParam < nParams; iParam++)
        {
            ParamBitMap const * param;


            /* If a function for encoding/decoding value is defined than it should take care of 0 */
            move16();
            param = & paramsBitMap->params[iParam];

            test();
            test();
            fShiftValue = s_and(param->fZeroAllowed==0, param->EncodeValue == NULL);
            value = GetFromBitstream(st, param->DecodeValue, index, param->nBits, pStream);
            if (fShiftValue)
            {
                move16();
                value = add(value, 1);
            }

            IF ((param->pSubParamBitMap != NULL) && (value > 0))
            {

                ReadFromBitstream(param->pSubParamBitMap, value, st, pStream, pnSize);
            }
        }
    }
    move16();
    *pnSize = add(*pnSize, i_mult(nParams, nArrayLength));

}
