/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "stl.h"            /* required for wmc_tool */

/*------------------------------------------------------------------------*
 * decode_envelope_indices_fx()
 *
 * Decode envelope indices
 *------------------------------------------------------------------------*/

Word16 decode_envelope_indices_fx( /* o  : Number of bits                    */
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure */
    const Word16 start_norm,       /* i  : starting band index               */
    const Word16 num_sfm,          /* i  : Number of subbands                */
    const Word16 numnrmibits,      /* i  : Bitrate of fall-back coding mode  */
    Word16 *difidx,          /* o  : Diff indices/encoded diff indices */
    const Word16 flag_HQ2          /* i  : indicator of HQ2 core             */
    ,const Word16 is_transient      /* i  : indicator of HQ_TRANSIENT         */
)
{
    Word16 hcode_l;
    Word16 i,j;
    Word16 LCmode;
    Word16 startNormPlus1,numSfmMinus1,numSfmMinus2,offset;
    Word16 *pDifidx,*pDifidx1;

    test();
    IF( sub(flag_HQ2, LOW_RATE_HQ_CORE) == 0 || sub(flag_HQ2, LOW_RATE_HQ_CORE_TRAN) == 0 )
    {
        LCmode = (Word16)get_next_indice_fx ( st_fx, BITS_DE_HMODE);
        difidx[start_norm] = (Word16)get_next_indice_fx ( st_fx, BITS_DE_FCOMP);
    }
    ELSE
    {
        LCmode = (Word16)get_next_indice_fx( st_fx, 2 );
        difidx[start_norm] = (Word16)get_next_indice_fx( st_fx, NORM0_BITS );
    }

    test();
    IF( is_transient && sub(flag_HQ2, LOW_RATE_HQ_CORE_TRAN) == 0 )
    {
        hcode_l = 0;
        move16();
        IF( sub(LCmode, 1) == 0 )
        {
            hdecnrm_tran_fx(st_fx, num_sfm, &difidx[start_norm + 1] );
            j = add(start_norm, num_sfm);
            FOR( i = start_norm + 1; i < j; i++ )
            {
                hcode_l = add(hcode_l, huffsizn_tran[difidx[i]]);
            }
        }
        ELSE
        {
            hdecnrm_context_fx(st_fx, num_sfm, &difidx[start_norm], &hcode_l);
        }
    }
    ELSE
    {
        hcode_l = 0;
        IF( LCmode == 0 )
        {
            hdecnrm_context_fx( st_fx, num_sfm, &difidx[start_norm], &hcode_l);
        }
        ELSE IF( LCmode == 1 )
        {
            startNormPlus1 = add(start_norm, 1);
            hdecnrm_resize_fx( st_fx, num_sfm, &difidx[startNormPlus1] );

            pDifidx = &difidx[startNormPlus1];
            move16();
            numSfmMinus1 = sub(num_sfm, 1);
            FOR( i = 0; i < numSfmMinus1; i++ )
            {
                j = *pDifidx++;
                move16();
                hcode_l = add(hcode_l, resize_huffsizn[j]);
            }

            pDifidx1 = &difidx[startNormPlus1];
            move16();
            numSfmMinus2 = sub(num_sfm, 2);
            FOR( i = 0; i < numSfmMinus2; i++ )
            {
                pDifidx = pDifidx1++;
                move16();
                IF( *pDifidx > 17 )
                {
                    offset = sub(*pDifidx, 17);
                    offset = s_min(offset, 3);
                    *pDifidx1 = sub(*pDifidx1, offset);
                }
                ELSE IF( *pDifidx < 13 )
                {
                    offset = sub(*pDifidx, 13);
                    offset = s_max(offset, -3);
                    *pDifidx1 = sub(*pDifidx1, offset);
                }
            }
        }
        ELSE IF( LCmode == 2 )
        {
            startNormPlus1 = add(start_norm, 1);
            hdecnrm_fx( st_fx, num_sfm, &difidx[start_norm + 1] );

            pDifidx = &difidx[startNormPlus1];
            move16();
            numSfmMinus1 = sub(num_sfm, 1);
            FOR( i = 0; i < numSfmMinus1; i++ )
            {
                j = *pDifidx++;
                move16();
                hcode_l = add(hcode_l, huffsizn[j]);
            }
        }
        ELSE
        {
            startNormPlus1 = add(start_norm, 1);
            numSfmMinus1 = sub(num_sfm, 1);
            pDifidx = &difidx[startNormPlus1];
            FOR( i = 0; i < numSfmMinus1; i++ )
            {
                *pDifidx++ = (Word16)get_next_indice_fx( st_fx, NORMI_BITS );
                move16();
            }
            hcode_l = numnrmibits;
            move16();
        }
    }

    return hcode_l;
}

/*------------------------------------------------------------------------*
 * dequantize_norms_fx()
 *
 * De-quantization of norms
 *------------------------------------------------------------------------*/

void dequantize_norms_fx(
    const Word16 start_norm,    /* i  : First SDE encoded norm            */
    const Word16 num_sfm,       /* i  : Number of norms                   */
    const Word16 is_transient,  /* i  : Transient flag                    */
    Word16 *ynrm,               /* o  : Decoded norm indices              */
    Word16 *normqlg2            /* o  : Log2 of decoded norms             */
)
{
    Word16 i,j;
    Word16 idxbuf[NB_SFM];
    Word16 *pYnrm, *pNormqlg2;

    /* First sub-frame */
    i = ynrm[start_norm];
    move16();
    normqlg2[start_norm] = dicnlg2[i];
    move16();

    /* Other sub-frames */
    IF ( is_transient )
    {
        /* Recover quantization indices and quantized norms */
        idxbuf[0] = ynrm[0];
        move16();
        FOR ( i = 1; i < num_sfm; i++ )
        {
            idxbuf[i] = sub( add(ynrm[i],idxbuf[i-1]), 15 );
            move16();
        }

        recovernorm_fx( idxbuf, ynrm, normqlg2, num_sfm );
    }
    ELSE
    {
        pYnrm = &ynrm[start_norm];
        move16();
        pNormqlg2 = &normqlg2[start_norm+1];
        move16();
        FOR ( i = 1; i < num_sfm; i++ )
        {
            j = sub(*pYnrm++,15);
            move16();
            *pYnrm = add(*pYnrm,j);
            move16();
            *pNormqlg2++ = dicnlg2[*pYnrm];
            move16();
        }
    }

    return;
}

