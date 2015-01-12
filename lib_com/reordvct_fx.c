/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"    /* Function prototypes                    */


/*--------------------------------------------------------------------------*
 * reordvct()
 *
 * Rearrange a vector in decreasing order
 *--------------------------------------------------------------------------*/

void reordvct_fx(
    Word16 *y,         /* i/o: vector to rearrange    */
    const Word16 N,          /* i  : dimensions             */
    Word16 *idx        /* o  : reordered vector index */
)
{
    Word16 i, j, k, n, im, temp;

    n = sub(N, 1);
    move16();
    FOR (i=0; i<n; i++)
    {
        im = i;
        move16();
        k = add(i, 1);
        move16();
        FOR (j=k; j<N; j++)
        {
            if ( sub(y[im], y[j]) < 0 )
            {
                im = j;
                move16();
            }
        }

        temp = y[i];
        move16();
        y[i] = y[im];
        move16();
        y[im] = temp;
        move16();
        j = idx[i];
        move16();
        idx[i] = idx[im];
        move16();
        idx[im] = j;
        move16();
    }

    return;
}


