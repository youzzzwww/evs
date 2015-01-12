/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */

#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"            /* required for wmc_tool */

/*--------------------------------------------------------------------------*
 * noise_adjust_fx()
 *
 * Calculate attenuation
 *--------------------------------------------------------------------------*/

Word16 noise_adjust_fx(                  /* o  : index of noise attenuation     Q0  */
    const Word16 *coeffs_norm,           /* i  : normalized coefficients        Qx  */
    const Word16 qx,                     /* i  : Q value of coeffs_norm             */
    const Word16 *bitalloc,              /* i  : bit allocation                 Q0  */
    const Word16 *sfm_start,             /* i  : band start                     Q0  */
    const Word16 *sfm_end,               /* i  : band end                       Q0  */
    const Word16 core_sfm                /* i  : index of the end band for core Q0  */
)
{
    Word16 nf_idx, sfm, bin, num_coeffs;
    Word16 E, diff, inv_num_coeffs;

    E = 0;
    move16();
    num_coeffs = 0;
    move16();

    FOR (sfm = 0; sfm <= core_sfm; sfm++)
    {
        IF (bitalloc[sfm] == 0)
        {
            FOR (bin = sfm_start[sfm]; bin < sfm_end[sfm]; bin++)
            {
                IF (coeffs_norm[bin] == 0)
                {
                    E = sub(E, 1);
                }
                ELSE
                {
                    E = add(E, sub(15+8,add(qx,norm_s(coeffs_norm[bin]))));
                }
                num_coeffs = add(num_coeffs, 1);
            }
        }
    }

    IF (num_coeffs != 0)
    {
        inv_num_coeffs = div_s(1,num_coeffs); /* Q15 */
        E = mult(E,inv_num_coeffs); /* Q0 (0+15-15) */
    }
    ELSE
    {
        E = 0;
        move16();
    }

    diff = sub(7, E);
    nf_idx = s_min(s_max(diff,0),3);

    return nf_idx;
}
