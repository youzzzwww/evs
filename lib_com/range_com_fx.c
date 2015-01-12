/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool */


/*-------------------------------------------------------------------*
 * rc_get_bits2()
 *
 *  Get number of bits needed to finalize range coder
 *-------------------------------------------------------------------*/

Word16 rc_get_bits2_fx(             /* o: Number of bits needed         */
    const Word16 N,                 /* i: Number of bits currently used */
    const UWord32 range             /* i: Range of range coder          */
)
{
    /*return N + 32 - (log2_i(range) - 1); */
    return add(add(N, 2), norm_ul(range));
}

/*-------------------------------------------------------------------*
 * rc_get_bits_f2()
 *
 *  Get fractional number of bits needed to finialize range coder
 *-------------------------------------------------------------------*/

Word16 rc_get_bits_f2_fx(           /* o: Number of bits needed in Q3   */
    const Word16 N,                 /* i: Number of bits currently used */
    const UWord32 range             /* i: Range of range coder          */
)
{
    UWord32 r;
    Word16 i, k, n;

    /*n = log2_i(range) - 1; */
    n = 31 - norm_ul(range) - 1;
    /*r = range >> ((n+1) - 15);     // Q15 */
    r = L_lshr(range, sub(n, 14));

    FOR (i = 0; i < 3; i++)
    {
        /*r = (r*r) >> 15;        // Q15 */
        r = UL_lshr(UL_Mpy_32_32(r, r), 15);
        /*k = r >> 16;            // 1 if r >= 2 */
        k = extract_l(UL_lshr(r, 16));
        /*n = (n<<1) | k; */
        n = s_or(lshl(n, 1), k);
        /*r >>= k; */
        r = UL_lshr(r, k);
    }

    /*return (N << 3) + 256 - n;  // Q3 */
    return sub(add(lshl(N, 3), 256), n);
}

