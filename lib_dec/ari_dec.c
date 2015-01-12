/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include "assert.h"
#include "stl.h"
#include "basop_mpy.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"

/*---------------------------------------------------------------
 * Ari decode 14 bits routines
  -------------------------------------------------------------*/

Word16 ari_decode_overflow(TastatDec *s)
{
    return L_sub(L_sub(s->high, 1), s->low) <= 0;
}

/**
 * \brief 	Start ArCo decoding
 *
 * \param[i/o] st
 * \param[o] s
 */
void ari_start_decoding_14bits(Decoder_State_fx *st, TastatDec *s)
{
    Word32	val;

    val = L_and(L_deposit_l(get_next_indice_fx(st, cbitsnew)), 0xffffL);

    s->low  = L_deposit_l(0);
    s->high = ari_q4new+1;
    move32();
    s->vobf = val;
    move32();
}

Word16 ari_start_decoding_14bits_prm(const Word16 *ptr, Word16  bp, TastatDec *s)
{
    Word32	val;
    Word16	i;
    const Word16 *p;

    val = L_deposit_l(0);

    p = ptr+bp;

    FOR (i=0; i<cbitsnew; i++)
    {
        val = L_shl(val, 1);
        if (*(p+i))
        {
            val = L_add(val, 1);
        }
    }
    s->low  = L_deposit_l(0);
    s->high = ari_q4new+1;
    move32();
    s->vobf = val;
    move32();

    return add(bp,i);
}

static Word16 ari_lookup_s17(Word32 cum, Word32 range, UWord16 const *cum_freq)
{
    Word32 tmp;
    const UWord16 *p;
    Word16 range_l, range_h;

    p = cum_freq;

    /* Note: For each indirect addressing p[i], we assume a tmp pointer init followed by a costfree reading the value */
    /* If the value multiplied by range is greater than cum, the pointer p is set to the tmp pointer                  */
    /*    tmp_p = p+8; if (tmp_p[0]*range>cum) p = tmp_p;                                                             */

    /* max value in p[x] is 16384 => 14 bits */
    range_l = extract_l(L_and(range,0x7FFF));
    range_h = extract_l(L_shr(range,15));

    tmp = L_multi31x16_X2(range_h, range_l,p[8]);
    if (L_sub(tmp,cum) > 0)
    {
        p = p + 8;
    }

    tmp = L_multi31x16_X2(range_h, range_l,p[4]);
    if (L_sub(tmp,cum) > 0)
    {
        p = p + 4;
    }


    tmp = L_multi31x16_X2(range_h, range_l,p[2]);
    if (L_sub(tmp,cum) > 0)
    {
        p = p + 2;
    }

    tmp = L_multi31x16_X2(range_h, range_l,p[1]);
    IF (L_sub(tmp,cum) > 0)
    {
        p = p + 1;

        tmp = L_multi31x16_X2(range_h, range_l,p[1]);
        test();
        if ( ((Word32)(&cum_freq[15] - p) == 0) && (L_sub(tmp,cum) > 0) )
        {
            p = p + 1;
        }
    }

    /* return extract_l(L_shr(L_sub((Word32)p,(Word32)cum_freq),1)); */
    L_sub(0,0);
    L_shr(0,0); /* pointer subtraction */
    return extract_l(p - cum_freq);
}

static Word16 ari_lookup_s27(Word32 cum, Word32 range, UWord16 const *cum_freq)
{
    Word32 tmp;
    Word16 range_l, range_h;
    Word16 il, ih, im;

    /* Note: For each indirect addressing p[i], we assume a tmp pointer init followed by a costfree reading the value */
    /* If the value multiplied by range is greater than cum, the pointer p is set to the tmp pointer                  */
    /*    tmp_p = p+8; if (tmp_p[0]*range>cum) p = tmp_p;                                                             */

    /* max value in p[x] is 16384 => 14 bits */
    range_l = extract_l(L_and(range,0x7FFF));
    range_h = extract_l(L_shr(range,15));

    /* begin change when compared with ari_decode_14bits_s17_ext,
       starting with line: tmp = L_multi31x16_X2(range_h, range_l, p[8]); */
    il = 0;
    move16();
    ih = 27;
    move16();

    /* do a five step binary search, using the interval [il, ih) */
    im = 13;
    move16(); /* (il + ih) >> 1 */
    tmp = L_multi31x16_X2(range_h, range_l, cum_freq[im]);
    tmp = L_sub(tmp, cum);
    if (tmp > 0)
    {
        il = im;
        move16();
    }
    if (tmp <= 0)
    {
        ih = im;
        move16();
    }

    im = shr(add(il, ih), 1);
    tmp = L_multi31x16_X2(range_h, range_l, cum_freq[im]);
    tmp = L_sub(tmp, cum);
    if (tmp > 0)
    {
        il = im;
        move16();
    }
    if (tmp <= 0)
    {
        ih = im;
        move16();
    }

    im = shr(add(il, ih), 1);
    tmp = L_multi31x16_X2(range_h, range_l, cum_freq[im]);
    tmp = L_sub(tmp, cum);
    if (tmp > 0)
    {
        il = im;
        move16();
    }
    if (tmp <= 0)
    {
        ih = im;
        move16();
    }

    im = shr(add(il, ih), 1);
    tmp = L_multi31x16_X2(range_h, range_l, cum_freq[im]);
    tmp = L_sub(tmp, cum);
    if (tmp > 0)
    {
        il = im;
        move16();
    }
    if (tmp <= 0)
    {
        ih = im;
        move16();
    }

    IF (sub(sub(ih, il), 1) > 0)   /* if the interval has more than one symbol */
    {
        /* here, only ih == il + 2 is possible, which means two symbols in the interval */
        im = add(il, 1); /* (il + ih) >> 1 */
        tmp = L_multi31x16_X2(range_h, range_l, cum_freq[im]);
        tmp = L_sub(tmp, cum);
        if (tmp > 0)
        {
            il = im;
            move16();
        }
    }

    return il;
}

static Word16 ari_lookup_bit(Word32 cum, Word32 range, UWord16 const *cum_freq)
{
    Word16 symbol = 0; /* initialize just to avoid compiler warning */

    (void)cum_freq;

    range = L_shl(range, 13); /* L_multi31x16_X2(range_h, range_l, 8192) */
    cum = L_sub(range, cum);

    if (cum > 0)
    {
        symbol = 1;
        move16();
    }
    if (cum <= 0)
    {
        symbol = 0;
        move16();
    }

    return symbol;
}

static Word16 ari_decode_14bits_ext(
    Decoder_State_fx *st,
    TastatDec *s,
    UWord16 const *cum_freq,
    Word16 (*lookup_fn)(Word32 cum, Word32 range, UWord16 const *cum_freq)
)
{
    Word32  cum;
    Word16  symbol;
    Word32  low;
    Word32  high;
    Word32  range;
    Word32  value;
    Word16 i;

    low   = L_add(0,s->low);
    high  = L_add(0,s->high);
    value = L_add(0,s->vobf);

    range = L_sub(high, low);

    cum = L_add(L_shl(L_sub(value, low), stat_bitsnew), (1<<stat_bitsnew)-1);
    if (cum < 0)
    {
        cum = L_add(0,0x7fffffff);
    }

    symbol = lookup_fn(cum, range, cum_freq);

    high = L_add(low,mul_sbc_14bits(range,cum_freq[symbol]));
    low  = L_add(low,mul_sbc_14bits(range,cum_freq[symbol+1]));

    FOR (i = 0; i < 0x7FFF; i++)
    {
        Word32 L_msb_diff, L_msb_low, L_msb_high;

        L_msb_high = L_shr(L_sub(high,1),14);
        L_msb_low  = L_shr(low,14);
        L_msb_diff = L_sub(L_msb_high, L_msb_low);
        IF (L_sub(L_msb_diff,2) >= 0)
        {
            BREAK;
        }
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0CCC);
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0BBB);
        low   = L_msu(low,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        low   = L_shl(low,1);
        high  = L_msu(high,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        high  = L_shl(high,1);
        value = L_msu(value,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        value = L_mac0(L_shl(value,1),1,get_next_indice_1_fx(st));
    }

    s->low  = low;
    move32();
    s->high = high;
    move32();
    s->vobf = value;
    move32();

    return symbol;
}


/**
 * \brief Only for 17 symbols with new extended Tables
 */
Word16 ari_decode_14bits_s17_ext(Decoder_State_fx *st, TastatDec *s, UWord16 const *cum_freq)
{
    return ari_decode_14bits_ext(st, s, cum_freq, ari_lookup_s17);
}

/**
 * \brief Only for 27 symbols with new extended Tables
 */
Word16 ari_decode_14bits_s27_ext(Decoder_State_fx *st, TastatDec *s, UWord16 const *cum_freq)
{
    return ari_decode_14bits_ext(st, s, cum_freq, ari_lookup_s27);
}

/**
 * \brief Only for decoding one bit with uniform probability:
 * the equivalent cum_freq table used is {16384, 8192, 0}
 */
Word16 ari_decode_14bits_bit_ext(Decoder_State_fx *st, TastatDec *s)
{
    static const UWord16 cum_freq[3] = {16384, 8192, 0};
    return ari_decode_14bits_ext(st, s, cum_freq, ari_lookup_bit);
}

static Word16 ari_lookup_pow(TastatDec *s, Word16 base)
{
    Word32 cum, range;
    Word16 symbol;
    Word16 range_h, range_l;
    Word16 pows[12];    /* "base" to the power of 2, 4, 8,... 2^12 */
    Word16 lowlim, highlim, testval;
    Word16 k;

    range = L_sub(s->high, s->low);
    cum = L_add(L_shl(L_sub(s->vobf, s->low), stat_bitsnew), (1<<stat_bitsnew)-1);

    range_h = extract_l(L_shr(range,15));
    range_l = extract_l(L_and(range,0x7FFF));

    symbol = 0;
    move16();
    lowlim = shr(base, 1);
    highlim = 16384;
    move16();

    /* search for the interval where "cum" fits */
    IF (L_sub(L_multi31x16_X2(range_h, range_l, lowlim), cum) > 0)   /* below pow-1 */
    {
        pows[0] = base;
        move16();
        testval = base;
        move16();
        /* increase exponent until it is smaller than "cum" */
        FOR (k = 1; k < 12; k++)
        {
            highlim = testval;
            move16();
            pows[k] = mult_r(pows[k-1], pows[k-1]);
            move16();
            testval = mult_r(pows[k], base);

            IF (L_sub(L_multi31x16_X2(range_h, range_l, shr(testval, 1)), cum) <= 0)   /* found! big range is [lowlim,testval], (now narrow it down) */
            {
                lowlim = testval;
                move16();
                k = sub(k, 1);
                symbol = shl(1, k);
                BREAK;
            }
        }
        assert(k < 12); /* maximum 2^10-1*/

        /* narrow the range down */
        FOR (k = sub(k, 2); k >= 0; k--)
        {
            testval = mult_r(highlim, pows[k+1]);

            IF (L_sub(L_multi31x16_X2(range_h, range_l, shr(testval, 1)), cum) <= 0)
            {
                lowlim = testval;
                move16();
                symbol = sub(symbol, shl(1, k));
            }
            ELSE
            {
                highlim = testval;
                move16();
            }
        }

        highlim = shr(highlim, 1);
        lowlim = shr(lowlim, 1);
    }

    s->high = L_add(s->low, mul_sbc_14bits(range, highlim));
    move32();
    s->low = L_add(s->low, mul_sbc_14bits(range, lowlim));
    move32();

    return symbol;
}

static Word16 ari_lookup_sign(TastatDec *s, Word16 base)
{
    Word32 cum, range;
    Word16 symbol;

    (void)base;

    range = L_sub(s->high, s->low);
    cum = L_sub(s->vobf, s->low);
    range = L_shr(range, 1);

    IF (L_sub(range, cum) > 0)
    {
        symbol = 1;
        move16();
        s->high = L_add(s->low, range);
        move32();
    }
    ELSE
    {
        symbol = 0;
        move16();
        s->low = L_add(s->low, range);
        move32();
    }

    return symbol;
}

static Word16 ari_decode_14bits_notbl(
    Word16 *ptr,
    Word16 bp,
    Word16 bits,
    Word16 *res,
    TastatDec *s,
    Word16 base,
    Word16 (*lookup_fn)(TastatDec *s, Word16 base)
)
{
    Word16 symbol;
    Word32 low, high, value;

    symbol = lookup_fn(s, base);

    low = L_add(s->low, 0);
    high = L_add(s->high, 0);
    value = L_add(s->vobf, 0);

    FOR (; bp<bits; ++bp)
    {
        Word32 L_msb_diff, L_msb_low, L_msb_high;

        L_msb_high = L_shr(L_sub(high,1),14);
        L_msb_low  = L_shr(low,14);
        L_msb_diff = L_sub(L_msb_high, L_msb_low);
        IF (L_sub(L_msb_diff,2) >= 0)
        {
            BREAK;
        }
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0CCC);
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0BBB);
        low   = L_msu(low,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        low   = L_shl(low,1);
        high  = L_msu(high,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        high  = L_shl(high,1);
        value = L_msu(value,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        value = L_mac0(L_shl(value,1),1,ptr[bp]);
    }

    s->low = low;
    move32();
    s->high = high;
    move32();
    s->vobf = value;
    move32();

    *res = symbol;
    move16();

    return bp;
}

/*------------------------------------------------------------------------
 * Function: ari_decode_14bits_pow
 *
 * Decode a symbol which follows the exponential distribution. That is,
 * symbols are in the following intervals
 *
 * p(x = 0) = 1 - exp(- 0.5 * base * 2)
 * p(x = q>0) = exp(- (q-0.5)*base* 2) - exp(- (q+0.5)*base*2 )
 *
 *-------------------------------------------------------------------------*/
Word16 ari_decode_14bits_pow(Word16 *ptr, Word16 bp, Word16 bits, Word16 *res, TastatDec *s, Word16 base)
{
    return ari_decode_14bits_notbl(ptr, bp, bits, res, s, base, ari_lookup_pow);
}

/*------------------------------------------------------------------------
 * Function: ari_decode_14bits_sign
 *
 * Decode a sign with equal probabilities.
 *-------------------------------------------------------------------------*/
Word16 ari_decode_14bits_sign(Word16 *ptr, Word16 bp, Word16 bits, Word16 *res, TastatDec *s)
{
    return ari_decode_14bits_notbl(ptr, bp, bits, res, s, 0, ari_lookup_sign);
}

