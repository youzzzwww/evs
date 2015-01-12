/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "stl.h"


/**
 * \brief          inplace long shift right: a[] = a[] >> bits
 *                 Logical shift right of UWord32 vector a[] by 'bits' positions.
 *                 BASOP cycles:        FLC cycles
 *                     len = 1:  8      lena = 2: 24
 *                     len = 2: 13      lena = 4: 34
 *                     len = 3: 18      lena = 6: 44
 *                     len = 4: 23      lena = 8: 54
 * \param          UWord32 a[]
 *                 Input: vector of the length len
 * \param          Word16 bits
 *                 Input: number of bit positions to shift right in range 1..31
 *                 Note: 'bits' must not be 0, this would cause a shift-overflow
 * \param          Word16 len
 *                 Input: length of vector a[] in units of 'UWord32'
 *
 * \return         void
 */

void longshr(UWord32 a[], Word16 bits, Word16 len)
{
    Word16 fracb_u, k;

    assert ((bits > 0) && (bits < 32));

    fracb_u = sub(32,bits);
    len = sub(len,1);
    FOR (k=0; k < len; k++)
    {
        a[k] = L_or(L_lshr(a[k],bits),L_lshl(a[k+1],fracb_u));
        move32();
    }
    a[k] = L_lshr(a[k],bits);
    move32();

    return;
}
