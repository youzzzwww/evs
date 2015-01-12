/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


/* Header files */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "prot_fx.h"
#include "stl.h"
#include "prot_fx.h"

void dlpc_bfi(
    const Word16 L_frame,
    Word16 *lsf_q,            /* o  : quantized LSFs                         */
    const Word16 *lsfold,           /* i  : past quantized LSF                     */
    const Word16 last_good,         /* i  : last good received frame               */
    const Word16 nbLostCmpt,        /* i  : counter of consecutive bad frames      */
    Word16 mem_MA[],          /* i/o: quantizer memory for MA model          */
    Word16 mem_AR[],          /* i/o: quantizer memory for AR model          */
    Word16 *stab_fac,         /* i  : LSF stability factor                   */
    Word16 *lsf_adaptive_mean,/* i  : LSF adaptive mean, updated when BFI==0 */
    Word16   numlpc,            /* i  : Number of division per superframe      */
    Word16 lsf_cng[],
    Word8   plcBackgroundNoiseUpdated,
    Word16 *lsf_q_cng,        /* o  : quantized LSFs                      */
    Word16 *old_lsf_q_cng,    /* o  : old quantized LSFs for background noise */
    const Word16* lsfBase,    /* i  : base for differential LSF coding        */
    Word16 tcxonly
)
{
    /**//*no local variables defined*/

    lsf_dec_bfi(&lsf_q[0], lsfold, lsf_adaptive_mean, lsfBase, mem_MA, mem_AR, *stab_fac,
                0, L_frame, last_good, nbLostCmpt,
                plcBackgroundNoiseUpdated, lsf_q_cng, lsf_cng, old_lsf_q_cng, 0, 0, tcxonly
               );
    IF ( sub(numlpc,2)==0 )
    {
        /* Decode the second LPC */
        lsf_dec_bfi(&lsf_q[M], &lsf_q[0], lsf_adaptive_mean, lsfBase, mem_MA, mem_AR, *stab_fac,
                    0, L_frame, last_good, nbLostCmpt+1,
                    plcBackgroundNoiseUpdated, lsf_q_cng, lsf_cng, old_lsf_q_cng, 0, 0, tcxonly
                   );
    }
    /**/ /*No local variabvles defined*/
}

