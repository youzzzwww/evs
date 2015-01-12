/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"         /* Common constants                       */

/*--------------------------------------------------------------------------
   *  bitallocsum_fx()
   *
   *  Calculate the total number of bits allocated over frame
   *--------------------------------------------------------------------------*/
void bitallocsum_fx(
    Word16 *R,            /* i  : bit-allocation vector                         Q0 */
    const Word16 nb_sfm,        /* i  : number of sub-vectors                         Q0 */
    Word16 *sum,          /* o  : total number of bits allocated                Q0 */
    Word16 *Rsubband,     /* o  : rate per subband                              Q3 */
    const Word16 v,             /* i  : bit rate                                      Q0 */
    const Word16 length,        /* i  : length of spectrum (32 or 48 kHz samplerate)  Q0 */
    const Word16 *sfmsize       /* i  : band length                                   Q0 */
)
{
    Word16 i;
    Word16 total, tmp;
    Word16 diff;

    total = (Word16)0;
    move16();
    FOR (i = 0; i < nb_sfm; i++)
    {
        tmp = extract_l(L_mult0(R[i], sfmsize[i]));
        Rsubband[i] = shl(tmp, 3);
        move16();
        total = add(total, tmp);
    }
    *sum = total;

    IF ( sub(length, L_FRAME32k) <= 0 )
    {
        diff = sub(v, *sum);
        i = (Word16)0;
        move16();
        WHILE ( diff > 0 )
        {
            IF ( R[i] > 0 )
            {
                Rsubband[i] = add(Rsubband[i], 8);
                move16();
                diff = sub(diff, 1);
                *sum = add(*sum, 1);
            }
            i = add(i, 1);
            if ( sub(i, nb_sfm) >= 0 )
            {
                i = (Word16)0;
                move16();
            }
        }
    }
    return;
}
