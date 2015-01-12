/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


#include "math_32.h"

/*-------------------------------------------------------------------------*
 * FUNCTION : edct_fx()
 *
 * PURPOSE : DCT transform
 *
 * INPUT ARGUMENTS :
 * _ (Word16) length             : length
 * _ (Word16*) x                 : input signal                      Qx
 * _ (Word16*) edct_table_128_fx : edct table                       Q16
 *
 * OUTPUT ARGUMENTS :
 * _ (Word16[]) y                : output transform                  Qx
 *-------------------------------------------------------------------------*/
void edct_fx(
    const Word32 *x,         /* i  : input signal        Qq         */
    Word32 *y,         /* o  : output transform    Qq         */
    Word16 length,     /* i  : length                         */
    Word16 *q          /* i  : Q value of input signal        */
)
{
    Word16 i;
    Word32 re;
    Word32 im;
    const Word16 *edct_table = 0;  /*Q16 */
    Word32 re2[L_FRAME48k/2+240];
    Word32 im2[L_FRAME48k/2+240];
    Word32 L_tmp;
    Word16 tmp;

    SWITCH (length)
    {
    case 1200:
        edct_table = edct_table_600_fx;
        *q = add(*q, 2);
        move16();
        BREAK;
    case 960:
        edct_table = edct_table_480_fx; /**q = add(*q, 2);*/ move16();
        BREAK;
    case 640:
        edct_table = edct_table_320_fx;
        move16();
        BREAK;
    case 320:
        edct_table = edct_table_160_fx;
        move16();
        BREAK;
    case 256:
        edct_table = edct_table_128_fx;
        move16();
        BREAK;
    case 240:
        edct_table = edct_table_120_fx;
        move16();
        BREAK;
    case 200:
        edct_table = edct_table_100_fx;
        move16();
        BREAK;
    case 160:
        edct_table = edct_table_80_fx;
        move16();
        BREAK;
    case 40:
        edct_table = edct_table_20_fx;
        move16();
        BREAK;
    case 800:
        edct_table = edct_table_400_fx;
        *q = add(*q, 2);
        move16();
        BREAK;
    case 512:
        edct_table = edct_table_256_fx;
        move16();
        BREAK;
    case 480:
        edct_table = edct_table_240_fx;
        move16();
        BREAK;
    case 384:
        edct_table = edct_table_192_fx;
        move16();
        BREAK;
    case 400:
        edct_table = edct_table_200_fx;
        move16();
        BREAK;
    case 128:
        edct_table = edct_table_64_fx;
        move16();
        BREAK;
    case 120:
        edct_table = edct_table_60_fx;
        move16();
        BREAK;
    case 80:
        edct_table = edct_table_40_fx;
        move16();
        BREAK;
    case 64:
        edct_table = edct_table_32_fx;
        move16();
        BREAK;
    case 32:
        edct_table = edct_table_16_fx;
        move16();
        BREAK;
    case 20:
        edct_table = edct_table_10_fx;
        *q = sub(*q, 2);
        move16();
        BREAK;
    default:
        BREAK;
    }

    /* Twiddling and Pre-rotate */
    FOR (i = 0; i < length/2; i++)
    {
        L_tmp = Mult_32_16(x[2*i], edct_table[i]); /*Q(q+1) */
        re2[i] = Madd_32_16(L_tmp, x[length-1-2*i], edct_table[length/2-1-i]); /*Q(q+1) */  move32();

        L_tmp = Mult_32_16(x[length-1-2*i], edct_table[i]); /*Q(q+1) */

        im2[i] = Msub_32_16(L_tmp, x[2*i], edct_table[length/2-1-i]); /*Q(q+1) */ move32();
    }

    *q = sub(15, *q);
    BASOP_cfft(re2, im2, shr(length,1), 1, q, y);

    tmp = div_s(1, length); /*Q15 */
    tmp = round_fx(L_shl(L_mult(tmp, 19302), 2)); /*Q15 */
    FOR (i = 0; i < length / 2; i++)
    {
        re = Msub_32_16(re2[i], im2[i], tmp);
        im = Madd_32_16(im2[i], re2[i], tmp);
        y[2 * i] = L_add(Mult_32_16(re, edct_table[i]), Mult_32_16(im, edct_table[length / 2 - 1 - i]));
        move32();
        y[length - 1 - 2 * i] = L_sub(Mult_32_16(re, edct_table[length / 2 - 1 - i]), Mult_32_16(im, edct_table[i]));
        move32();
    } /*Q(q-2) */

    *q = sub(15+2, *q);
    return;
}

/*-------------------------------------------------------------------------*
 * FUNCTION : edst_fx()
 *
 * PURPOSE : DST_IV transform
 *
 * INPUT ARGUMENTS :
 * _ (Word16) length             : length
 * _ (Word16*) x                 : input signal                      Qx
 * _ (Word16*) edct_table_128_fx : edct table                       Q16
 *
 * OUTPUT ARGUMENTS :
 * _ (Word16[]) y                : output transform                  Qx
 *-------------------------------------------------------------------------*/
void edst_fx(
    const Word32 *x,         /* i  : input signal        Qq         */
    Word32 *y,         /* o  : output transform    Qq         */
    Word16 length,     /* i  : length                         */
    Word16 *q          /* i  : Q value of input signal        */
)
{
    Word16 i;
    Word32 re;
    Word32 im;
    const Word16 *edct_table = 0;  /*Q16 */
    Word32 re2[L_FRAME48k/2+240];
    Word32 im2[L_FRAME48k/2+240];
    Word32 L_tmp;
    Word16 tmp;

    SWITCH (length)
    {
    case 1200:
        edct_table = edct_table_600_fx;
        *q = add(*q, 2);
        move16();
        BREAK;
    case 960:
        edct_table = edct_table_480_fx; /**q = add(*q, 2);*/ move16();
        BREAK;
    case 640:
        edct_table = edct_table_320_fx;
        move16();
        BREAK;
    case 320:
        edct_table = edct_table_160_fx;
        move16();
        BREAK;
    case 256:
        edct_table = edct_table_128_fx;
        move16();
        BREAK;
    case 240:
        edct_table = edct_table_120_fx;
        move16();
        BREAK;
    case 200:
        edct_table = edct_table_100_fx;
        move16();
        BREAK;
    case 160:
        edct_table = edct_table_80_fx;
        move16();
        BREAK;
    case 40:
        edct_table = edct_table_20_fx;
        move16();
        BREAK;
    case 800:
        edct_table = edct_table_400_fx;
        *q = add(*q, 2);
        move16();
        BREAK;
    case 512:
        edct_table = edct_table_256_fx;
        move16();
        BREAK;
    case 480:
        edct_table = edct_table_240_fx;
        move16();
        BREAK;
    case 384:
        edct_table = edct_table_192_fx;
        move16();
        BREAK;
    case 400:
        edct_table = edct_table_200_fx;
        move16();
        BREAK;
    case 128:
        edct_table = edct_table_64_fx;
        move16();
        BREAK;
    case 120:
        edct_table = edct_table_60_fx;
        move16();
        BREAK;
    case 80:
        edct_table = edct_table_40_fx;
        move16();
        BREAK;
    case 64:
        edct_table = edct_table_32_fx;
        move16();
        BREAK;
    case 32:
        edct_table = edct_table_16_fx;
        move16();
        BREAK;
    case 20:
        edct_table = edct_table_10_fx;
        *q = sub(*q, 2);
        move16();
        BREAK;
    default:
        BREAK;
    }

    /* Twiddling and Pre-rotate */
    FOR (i = 0; i < length/2; i++)
    {
        L_tmp = Mult_32_16(x[length-1-2*i], edct_table[i]);
        re2[i] = Madd_32_16(L_tmp, x[2*i], edct_table[length/2-1-i]);
        move32();

        L_tmp = Mult_32_16(x[2*i], edct_table[i]);
        im2[i] = Msub_32_16(L_tmp, x[length-1-2*i], edct_table[length/2-1-i]);
        move32();
    }

    *q = sub(15, *q);
    BASOP_cfft(re2, im2, shr(length,1), 1, q, y);

    tmp = div_s(1, length); /*Q15 */
    tmp = round_fx(L_shl(L_mult(tmp, 19302), 2)); /*Q15 */
    FOR (i = 0; i < length / 2; i++)
    {
        re = Msub_32_16(re2[i], im2[i], tmp);
        im = Madd_32_16(im2[i], re2[i], tmp);
        y[2 * i] = L_add(Mult_32_16(re, edct_table[i]), Mult_32_16(im, edct_table[length / 2 - 1 - i]));
        move32();
        y[length - 1 - 2 * i] = L_sub(Mult_32_16(im, edct_table[i]), Mult_32_16(re, edct_table[length / 2 - 1 - i]));
        move32();
    } /*Q(q) */

    *q = sub(15+2, *q);

    return;
}

/*========================================================================*/
/* FUNCTION : edct_fx()													  */
/*------------------------------------------------------------------------*/
/* PURPOSE : DCT transform												  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) length             : length								  */
/* _ (Word16*) x	             : input signal						Qx	  */
/* _ (Word16*) edct_table_128_fx : edct table                       Q15   */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ (Word16[]) y : output transform      Qx							  */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void edct_16fx(
    const Word16 *x,         /* i  : input signal        Qx		  */
    Word16 *y,         /* o  : output transform    Qx		*/
    Word16 length,      /* i  : length					  */
    Word16 bh			/* bit-headroom */
)
{
    Word16 i;
    Word16 re[L_FRAME48k/2];
    Word16 im[L_FRAME48k/2];
    const Word16 *edct_table = 0;
    Word16 re2[L_FRAME48k/2];
    Word16 im2[L_FRAME48k/2];

    Word32 L_tmp, Lacc, Lmax;
    Word16 tmp, fact;
    Word16 Q_edct;
    Word16 Len2, i2;
    const Word16 *px, *pt;
    Word16 *py;

    /*COMPLETE: some eDCT sub function are missing */

    IF (sub(length,L_FRAME32k) == 0)
    {
        edct_table = &edct_table_320_16fx[0];
        move16();
    }
    ELSE IF (sub(length,L_FRAME) == 0)
    {
        edct_table = &edct_table_128_16fx[0];
        move16();
    }
    ELSE IF (sub(length,L_FRAME16k) == 0)
    {
        edct_table = &edct_table_160_16fx[0];
        move16();
    }
    ELSE
    {
    }

    /* Twiddling and Pre-rotate */
    Lmax = L_deposit_l(0);
    Len2 = shr(length,1);
    px = x + length - 1;
    pt = edct_table + Len2 - 1;
    FOR (i = 0; i < Len2; i++)
    {
        i2 = shl(i,1);
        L_tmp = L_mult(x[i2],edct_table[i]);/*Q(Qx+16) */

        Lacc = L_mac(L_tmp,*px,*pt);/*Q(Qx+16) */

        Lmax = L_max(Lmax, Lacc);

        L_tmp = L_mult(*px,edct_table[i]);/*Q(Qx+16) */
        Lacc = L_msu(L_tmp,x[i2],*pt);/*Q(Qx+16) */
        Lmax = L_max(Lmax, Lacc);

        px -= 2;
        pt--;
    }

    tmp = Lmax==0?31:norm_l(Lmax);
    logic16();
    test();
    Q_edct = sub(tmp,bh); /*creating a bit-headroom */

    px = x + length - 1;
    pt = edct_table + Len2 - 1;
    FOR (i = 0; i < Len2; i++)
    {
        i2 = shl(i,1);

        L_tmp = L_mult(x[i2],edct_table[i]);/*Q(Qx+16) */
        Lacc = L_mac(L_tmp,*px,*pt);/*Q(Qx+16) */
        re2[i] = round_fx(L_shl(Lacc, Q_edct)); /* Q(Qx+Q_edct) */

        L_tmp = L_mult(*px,edct_table[i]);/*Q(Qx+16) */
        Lacc = L_msu(L_tmp,x[i2],*pt);/*Q(Qx+16) */
        im2[i] = round_fx(L_shl(Lacc, Q_edct)); /* Q(Qx+Q_edct) */

        px -= 2;
        pt--;
    }
    IF (sub(length,L_FRAME32k) == 0)
    {
        DoRTFT320_16fx(re2, im2);
    }
    ELSE IF (sub(length,L_FRAME )== 0)
    {
        DoRTFT128_16fx(re2, im2);
    }
    ELSE IF (sub(length,L_FRAME16k) == 0)
    {
        DoRTFT160_16fx(re2, im2);
    }
    ELSE
    {
    }
    tmp = div_s(1,length); /*Q15 */
    L_tmp = L_mult(tmp,19302); /*Q29, (3*PI/4) in Q13 */
    fact = round_fx(L_shl(L_tmp,2)); /*Q15 */
    FOR (i = 0; i < length/2; i++)
    {
        tmp = mult_r(im2[i],fact); /*Q(Qx+Q_edct) */
        re[i] = sub(re2[i],tmp); /*Q(Qx+Q_edct) */  move16();

        tmp = mult_r(re2[i],fact); /*Q(Qx+Q_edct) */
        im[i] = add(im2[i],tmp); /*Q(Qx+Q_edct) */  move16();
    }

    /* Post-rotate and obtain the output data */
    py = y + length - 1;
    pt = edct_table + Len2 - 1;
    FOR (i = 0; i < Len2; i++)
    {
        i2 = shl(i,1);

        L_tmp = L_mult(re[i],edct_table[i]);/*Q(Qx+Q_edct+16) */
        Lacc = L_mac(L_tmp,im[i],*pt);/*Q(Qx+Q_edct+16) */
        y[i2] = round_fx(L_shr(Lacc,Q_edct)); /* Q(Qx) */

        L_tmp = L_mult(re[i],edct_table[length/2-1-i]);/*Q(Qx+Q_edct+16) */
        Lacc = L_msu(L_tmp,im[i],edct_table[i]);/*Q(Qx+Q_edct+16) */
        *py = round_fx(L_shr(Lacc,Q_edct)); /* Q(Qx) */

        py -= 2;
        pt--;
    }
    return;
}


/*-----------------------------------------------------------------*
 * iedct_short_fx()
 *
 * Inverse EDCT for short frames
 *-----------------------------------------------------------------*/

void iedct_short_fx(
    const Word32 *in,                /* i  : input vector     */
    Word16 *Q,                 /* i/o: Q value of input */
    Word32 *out,               /* o  : output vector    */
    const Word16 segment_length      /* i  : length           */
)
{
    Word32 alias[MAX_SEGMENT_LENGTH];
    Word16 seg_len_div2, seg_len_div4, seg_len_3mul_div4;
    Word16 i;
    Word16 qtmp, tmp;

    qtmp = *Q;
    move16();
    tmp = 0;
    move16();
    seg_len_div2 = shr(segment_length, 1);
    seg_len_div4 = shr(segment_length, 2);
    seg_len_3mul_div4 = add(seg_len_div2, seg_len_div4);

    edct_fx(in, alias, seg_len_div2, Q);
    FOR (i = 0; i < seg_len_div2; i++)
    {
        IF (alias[i] != 0)
        {
            tmp = 1;
            move16();
            BREAK;
        }
    }
    if (tmp == 0)
    {
        *Q = qtmp;
        move16();
    }
    FOR (i = 0; i < seg_len_div4; i++)
    {
        out[i]                     =  alias[seg_len_div4 + i];
        move32();
        out[seg_len_div4 + i]      = L_negate(alias[seg_len_div2 - 1 - i]);
        move32();
        out[seg_len_div2 + i]      = L_negate(alias[seg_len_div4 - 1 - i]);
        move32();
        out[seg_len_3mul_div4 + i] = L_negate(alias[i]);
        move32();
    }

    return;
}
