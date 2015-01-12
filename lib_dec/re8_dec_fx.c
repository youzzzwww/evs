/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"

/*--------------------------------------------------------------------------
* re8_dec_fx()
*
* MULTI-RATE INDEXING OF A POINT y in THE LATTICE RE8 (INDEX DECODING)
* note: the index I is defined as a 32-bit word, but only
* 16 bits are required (long can be replaced by unsigned integer)
*--------------------------------------------------------------------------*/

void re8_dec_fx(
    Word16 n,     /* i  : codebook number (*n is an integer defined in {0,2,3,4,..,n_max})    */
    const UWord16 I,    /* i  : index of c (pointer to unsigned 16-bit word)                        */
    const Word16 k[],   /* i  : index of v (8-dimensional vector of binary indices) = Voronoi index */
    Word16 y[]    /* o  : point in RE8 (8-dimensional integer vector)                         */
)
{
    Word16 i, m, v[8];

    /*------------------------------------------------------------------------*
     * decode the sub-indices I and kv[] according to the codebook number n:
     *  if n=0,2,3,4, decode I (no Voronoi extension)
     *  if n>4, Voronoi extension is used, decode I and kv[]
     *------------------------------------------------------------------------*/
    IF( sub(n, 4) <= 0 )
    {
        re8_decode_base_index_fx( n, I, y );
    }
    ELSE
    {
        /*--------------------------------------------------------------------*
         * compute the Voronoi modulo m = 2^r where r is extension order
         *--------------------------------------------------------------------*/
        m = 0;
        move16();

        FOR (; n > 4; n -= 2)
        {
            m = add(m, 1);
        }

        /*--------------------------------------------------------------------*
         * decode base codebook index I into c (c is an element of Q3 or Q4)
         *  [here c is stored in y to save memory]
         *--------------------------------------------------------------------*/

        re8_decode_base_index_fx( n, I, y );

        /*--------------------------------------------------------------------*
         * decode Voronoi index k[] into v
         *--------------------------------------------------------------------*/
        re8_k2y_fx( k, m, v );

        /*--------------------------------------------------------------------*
         * reconstruct y as y = m c + v (with m=2^r, r integer >=1)
         *--------------------------------------------------------------------*/
        FOR( i=0; i<8; i++ )
        {
            /* y[i] = m*y[i] + v[i] */
            y[i] = add(shl(y[i], m), v[i]);
            move16();
        }
    }

    return;
}
