/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"

#include "prot_fx.h"
#include "stl.h"        /* required for wmc_tool */

/*--------------------------------------------------------------------------*/
/*  Function  reordernorm_fx                                                */
/*  ~~~~~~~~~~~~~~~~~~~~~                                                   */
/*                                                                          */
/*  Reorder quantization indices and norms                                  */
/*--------------------------------------------------------------------------*/

void reordernorm_fx(
    const Word16 *ynrm,          /* i  : quantization indices for norms     Q0 */
    const Word16 *normqlg2,      /* i  : quantized norms                    Q0 */
    Word16 *idxbuf,        /* o  : reordered quantization indices     Q0 */
    Word16 *normbuf,       /* o  : reordered quantized norms          Q0 */
    const Word16 nb_sfm          /* i  : number of bands                    Q0 */
)
{
    Word16 i;
    const Word16 *order = NULL;

    SWITCH(nb_sfm)
    {
    case NB_SFM:
        order = norm_order_48;
        BREAK;
    case SFM_N_SWB:
        order = norm_order_32;
        BREAK;
    case SFM_N_WB:
        order = norm_order_16;
        BREAK;
    default:
        order = norm_order_48;
        BREAK;
    }

    FOR (i = 0; i < nb_sfm; i++)
    {
        idxbuf[i] = ynrm[order[i]];
        move16();
        move16();
        normbuf[i] = normqlg2[order[i]];
        move16();
        move16();
    }

    return;
}
