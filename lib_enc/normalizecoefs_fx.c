/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "stl.h"
#include "basop_mpy.h"
#include "prot_fx.h"
#include "cnst_fx.h"    /* Common constants                       */


/*--------------------------------------------------------------------------
 * normalizecoefs_fx()
 *
 * Normalize MDCT coefficients with quantized norms
 *--------------------------------------------------------------------------*/

void normalizecoefs_fx(
    Word32 *coefs,                     /* i  : Input vector (Q12)                  */
    const Word16 *ynrm,                      /* i  : quantization indices for norms      */
    const Word16 num_bands,                  /* i  : Number of bands                     */
    const Word16 *band_start,                /* i  : Start of bands                      */
    const Word16 *band_end,                  /* i  : End of bands                        */
    Word16 *coefs_norm                 /* o  : Normalized output vector            */
)
{
    Word16 band, j, k, r, v;
    Word16 *pcoefs16;
    Word32 *pcoefs;
    Word16 subvec_start, subvec_end, num_coefs;
    pcoefs   = coefs;
    pcoefs16 = coefs_norm;

    FOR (band = 0; band < num_bands; band++)
    {
        r = s_and(ynrm[band], 1);
        v = shr(ynrm[band], 1);
        k = sub(sub(17, r), v);

        subvec_start = band_start[band];
        move16();
        subvec_end   = band_end[band];
        move16();
        num_coefs    = sub(subvec_end, subvec_start);

        FOR (j = 0; j < num_coefs; j++)
        {
            IF (r!=0)
            {
                *pcoefs = Mpy_32_16_1(*pcoefs, INV2POWHALF);
                move32();
            }
            BASOP_SATURATE_WARNING_OFF /* May saturate for strong peaks in a high band, in which case saturation is desirable */
            *pcoefs16++ = round_fx( L_shl(*pcoefs++, 16-k) ); /* Q12 */
            BASOP_SATURATE_WARNING_ON
        }
    }

    return;
}

