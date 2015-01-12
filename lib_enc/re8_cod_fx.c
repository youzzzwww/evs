/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "prot_fx.h"
#include "stl.h"

/*------------------------------------------------------------------------
 * RE8_cod:
 *
 * MULTI-RATE INDEXING OF A POINT y in THE LATTICE RE8 (INDEX COMPUTATION)
 * note: the index I is defined as a 32-bit word, but only
 *       16 bits are required (long can be replaced by unsigned integer)
 *--------------------------------------------------------------------------*/

void re8_cod_fx(
    Word16 x[],  /* i  : point in RE8 (8-dimensional integer vector)                         */
    Word16 *n,   /* i  : codebook number (*n is an integer defined in {0,2,3,4,..,n_max})    */
    UWord16 *I,  /* o  : index of c (pointer to unsigned 16-bit word)                        */
    Word16 k[]   /* o  : index of v (8-dimensional vector of binary indices) = Voronoi index */
)
{
    Word16 ka_fx, c_fx[8];
    /*----------------------------------------------------------------------
     * decompose x as x = 2^r c + v, where r is an integer >=0, c is an element
     *  of Q0, Q2, Q3 or Q4, and v is an element of a Voronoi code in RE8
     *  (if r=0, x=c)
     *  this decomposition produces as a side-product the index k[] of v
     *  and the identifier ka of the absolute leader related to c
     *
     *  the index of y is split into 2 parts :
     *  - the index I of c
     *  - the index k[] of v
     ----------------------------------------------------------------------*/
    re8_vor_fx(x, n, k, c_fx, &ka_fx);

    /* compute the index I (only if c is in Q2, Q3 or Q4) */
    IF (*n > 0)
    {
        re8_compute_base_index_fx(c_fx, ka_fx, I);
    }

    return;
}
