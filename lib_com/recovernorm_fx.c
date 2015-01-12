/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "stl.h"        /* required for wmc_tool                  */
#include "prot_fx.h"    /* Function prototypes                    */
#include "cnst_fx.h"    /* Common constants                       */
#include "rom_com_fx.h" /* Static table prototypes                */

/*--------------------------------------------------------------------------*
 * recovernorm_fx()
 *
 * Recover reordered quantization indices and norms
 *--------------------------------------------------------------------------*/

void recovernorm_fx(
    Word16 *idxbuf,           /* i  : reordered quantization indices */
    Word16 *ynrm,             /* o  : recovered quantization indices */
    Word16 *normqlg2,         /* o  : recovered quantized norms      */
    Word16 nb_sfm             /* i  : number of SFMs                 */
)
{
    Word16 i,j,k;
    const Word16 *order = NULL;
    move16();

    SWITCH (nb_sfm)
    {
    case NB_SFM:
        order = norm_order_48;
        move16();
        BREAK;
    case SFM_N_SWB:
        order = norm_order_32;
        move16();
        BREAK;
    case SFM_N_WB:
        order = norm_order_16;
        move16();
        BREAK;
    default:
        order = norm_order_48;
        move16();
        BREAK;
    }

    FOR (i = 0; i < nb_sfm; i++)
    {
        j = order[i];
        move16();
        k = idxbuf[i];
        move16();
        ynrm[j] = k;
        move16();
        normqlg2[j] = dicnlg2[k];
        move16();
    }

    return;
}

