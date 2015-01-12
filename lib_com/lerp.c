/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "basop_util.h"
#include "prot_fx.h"
#include <assert.h>
#include "stl.h"



#define shift_e (16-1)
#define pos_e   (16-1)

void lerp(Word16 *f, Word16 *f_out,  Word16 bufferNewSize, Word16 bufferOldSize)
{

    Word16 i, idx, n;
    Word16 diff;
    Word32 pos, shift;
    Word16 buf[2000];
    Word16 *ptr;

    ptr = f_out;
    test();
    test();
    test();
    if ( ((f <= f_out) && (f + bufferOldSize >= f_out)) || ((f_out <= f) && (f_out + bufferNewSize >= f)) )
    {
        ptr = buf;
        move16();
    }

    IF( sub(bufferNewSize, bufferOldSize) == 0 )
    {
        Copy(f, f_out, bufferNewSize);
        return;
    }

    shift = L_shl(L_deposit_l(div_s( bufferOldSize, shl(bufferNewSize, 4))), 4-shift_e+16);

    pos = L_sub(L_shr(shift, 1), FL2WORD32_SCALE(1.0f, 1+shift_e));

    /* Adjust interpolation shift to avoid accessing beyond end of input buffer. */
    if ( L_sub(shift, FL2WORD32_SCALE(0.3f, shift_e)) < 0)
    {
        pos = L_sub(pos, FL2WORD32_SCALE(0.13f, shift_e));
    }

    assert(pos_e == shift_e);

    /* first point of interpolation */
    IF (pos<0)
    {

        diff = shr(extract_l(pos), 1);
        /*buf[0]=f[0]+pos*(f[1]-f[0]);*/
        move16();
        *ptr++ = add(f[0], msu_r(L_mult(diff, f[1]),diff, f[0]));
    }
    ELSE
    {

        idx=extract_h(pos);

        diff = lshr(extract_l(pos), 1);

        move16();
        *ptr++ = add(f[idx], msu_r(L_mult(diff, f[idx+1]), diff, f[idx]));
    }

    pos = L_add(pos, shift);
    idx = s_max(0, extract_h(pos));

    n = sub(bufferNewSize, 1);
    FOR ( i=1; i<n; i++ )
    {
        diff = lshr(extract_l(pos), 1);
        if (pos < 0)
        {
            diff = sub(FL2WORD16(0.5f), diff);
        }

        move16();
        *ptr++ = add(f[idx], msu_r(L_mult(diff, f[idx+1]), diff, f[idx]));

        pos = L_add(pos, shift);
        idx = extract_h(pos);
    }

    /* last point */

    if ( L_sub(pos, L_deposit_h(sub(bufferOldSize,1))) > 0 )
    {
        idx = sub(bufferOldSize,2);
    }
    assert(idx <= 2000);

    /* diff = t - point;*/
    diff = lshr(extract_l(L_shr(L_sub(pos, L_deposit_h(idx)), 1)), 1);

    move16();
    *ptr++ = add(f[idx], shl(msu_r(L_mult(diff, f[idx+1]), diff, f[idx]), 1));

    test();
    test();
    test();
    IF ( ((f <= f_out) && (f + bufferOldSize >= f_out)) || ((f_out <= f) && (f_out + bufferNewSize >= f)) )
    {
        Copy( buf, f_out, bufferNewSize );
    }

}
