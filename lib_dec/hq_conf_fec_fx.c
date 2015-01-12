/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "prot_fx.h"
#include "cnst_fx.h"
#include "stl.h"                /* required for wmc_tool */


/*--------------------------------------------------------------------------*
 * hq_configure_bfi_fx()
 *
 * Initialization of HQ bands and subframes
 *--------------------------------------------------------------------------*/
void hq_configure_bfi_fx(
    Word16 *nb_sfm,             /* o  : Number of sub bands              Q0 */
    Word16 *num_Sb,             /* o  : Number of FEC sub bands ?        Q0 */
    Word16 *num_bands_p,        /* o  : FEC sub bands                    Q0 */
    Word16 const **sfmsize,     /* o  : Subband bandwidths                */
    Word16 const **sfm_start,   /* o  : Subband start coefficients        */
    Word16 const **sfm_end      /* o  : Subband end coefficients          */
)
{
    *num_Sb = MAX_SB_NB;
    move16();
    *nb_sfm = SFM_N_NB;
    move16();
    Copy( Num_bands_NB, num_bands_p, *num_Sb );
    *sfmsize = band_len_wb;
    *sfm_start = band_start_wb;
    *sfm_end = band_end_wb;



    return;
}
