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

/**
 * \brief 	Copy state
 *
 * \param[i] source
 * \param[o] dest
 *
 * \return none
 */
void ari_copy_states(TastatEnc *source, TastatEnc *dest)
{
    dest->low  = source->low;
    move32();
    dest->high = source->high;
    move32();
    dest->vobf = source->vobf;
    move16();
}

/*---------------------------------------------------------------
  Ari encoder 14 bits routines
  -------------------------------------------------------------*/


/**
 * \brief 	Start ArCo encoding
 *
 * \param[o] s
 *
 * \return none
 */
void ari_start_encoding_14bits(TastatEnc *s)
{
    s->low  = L_deposit_l(0);
    s->high = ari_q4new + 1;
    move32();
    s->vobf = 0;
    move16();
}


/* Returns: new bit-stream position */
static Word16 ari_put_bit_plus_follow(
    Word16 ptr[],          /* o: bit-stream                              */
    Word16 bp,             /* i: bit-stream position                     */
    Word16 bits_to_follow, /* i: number of opposite bits to follow 'bit' */
    Word16 bit             /* i: bit to send                             */
)
{
    assert(bit == 0 || bit == 1);
    ptr[bp++] = bit;     /* send initially a zero or one */     move16();
    bit = s_xor(bit, 1); /* invert bit to send           */
    FOR ( ; bits_to_follow > 0; bits_to_follow--)
    {
        ptr[bp++] = bit; /* send inverted bit */                  move16();
    }
    return bp;
}

/**
 * \brief 	Finish ArCo encoding
 *
 * \param[o] ptr
 * \param[i] bp
 * \param[i] s
 *
 * \return bit consumption
 */
Word16 ari_done_encoding_14bits(Word16 *ptr, Word16 bp, TastatEnc *s)
{
    Word16 bit;

    bit = 0;
    move16();
    if ( L_sub(s->low,ari_q1new) >= 0 )
    {
        bit = s_xor(bit,1);
    }
    return ari_put_bit_plus_follow(ptr, bp, add(s->vobf, 1), bit);
}



/**
 * \brief encode function for extended proba tables: less branches needed for coding
 *
 * \param[o]   ptr
 * \param[i]   bp
 * \param[i/o] s
 * \param[i]   symbol
 * \param[i]   cum_freq
 *
 * \return bit consumption
 */
Word16 ari_encode_14bits_ext(
    Word16  *ptr,
    Word16   bp,
    TastatEnc *s,
    Word32   symbol,
    UWord16 const *cum_freq
)
{
    Word32 low;
    Word32 high;
    Word32 range;
    Word16 bits_to_follow;
    Word16 i;
    UWord16 temp;
    Word32 L_temp1, L_temp2;

    high  = L_add(s->high, 0);
    low   = L_add(s->low, 0);
    range = L_sub(high, low);

    L_temp1 = L_shl(range,15-stat_bitsnew/*both are constants*/);
    Mpy_32_16_ss(L_temp1, cum_freq[symbol+1], &L_temp2, &temp);
    if (symbol != 0)   /* when symbol is 0, range remains unchanged */
    {
        Mpy_32_16_ss(L_temp1, cum_freq[symbol], &range, &temp);
    }
    high = L_add(low, range);
    low = L_add(low, L_temp2);

    bits_to_follow = s->vobf;
    move16();

    FOR (i = 0; i < 0x7FFF; i++)
    {
        IF (L_sub(high, ari_q2new) <= 0)
        {
            bp = ari_put_bit_plus_follow(ptr, bp, bits_to_follow, 0);
            bits_to_follow = 0;
            move16();
        }
        ELSE IF (L_sub(low, ari_q2new) >= 0)
        {
            bp = ari_put_bit_plus_follow(ptr, bp, bits_to_follow, 1);
            bits_to_follow = 0;
            move16();
            low = L_sub(low, ari_q2new);
            high = L_sub(high, ari_q2new); /* Subtract offset to top.  */
        }
        ELSE
        {
            test();
            IF (L_sub(low, ari_q1new) >= 0 && L_sub(high, ari_q3new) <= 0)
            {
                /* Output an opposite bit   */
                /* later if in middle half. */
                bits_to_follow = add(bits_to_follow, 1);
                low = L_sub(low, ari_q1new); /* Subtract offset to middle*/
                high = L_sub(high, ari_q1new);
            }
            ELSE {
                BREAK; /* Otherwise exit loop.     */
            }
        }

        low = L_add(low, low);
        high = L_add(high, high); /* Scale up code range.     */
    }

    s->low  = low;
    move32();
    s->high = high;
    move32();
    s->vobf = bits_to_follow;
    move16();

    return bp;
}

Word16 ari_encode_overflow(TastatEnc *s)
{
    return L_sub(L_sub(s->high, 1), s->low) <= 0;
}


static Word16 ari_encode_14bits_high_low(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s, Word32 high, Word32 low)
{
    Word16 bits_to_follow, tmp;

    bits_to_follow = s->vobf;
    move16();

    /* while there are more than 16 bits left */
    tmp = sub(16, bits);
    WHILE (add(add(bp, bits_to_follow), tmp) < 0)
    {
        IF (L_sub(high, ari_q2new) <= 0)
        {
            bp = ari_put_bit_plus_follow(ptr, bp, bits_to_follow, 0);
            bits_to_follow = 0;
            move16();
        }
        ELSE IF (L_sub(low, ari_q2new) >= 0)
        {
            bp = ari_put_bit_plus_follow(ptr, bp, bits_to_follow, 1);
            bits_to_follow = 0;
            move16();
            low = L_sub(low, ari_q2new);
            high = L_sub(high, ari_q2new); /* Subtract offset to top.  */
        }
        ELSE
        {
            test();
            IF (L_sub(low, ari_q1new) >= 0 && L_sub(high, ari_q3new) <= 0)
            {
                /* Output an opposite bit   */
                /* later if in middle half. */
                bits_to_follow = add(bits_to_follow, 1);
                low = L_sub(low, ari_q1new); /* Subtract offset to middle*/
                high = L_sub(high, ari_q1new);
            }
            ELSE {
                BREAK; /* Otherwise exit loop.     */
            }
        }

        low = L_add(low, low);
        high = L_add(high, high); /* Scale up code range.     */
    }

    s->low  = low;
    move32();
    s->high = high;
    move32();
    s->vobf = bits_to_follow;
    move16();

    return bp;
}

/*------------------------------------------------------------------------
 * Function: ari_encode_14bits_range
 *
 * Encode an cumulative frequency interval.
 *-------------------------------------------------------------------------*/

Word16 ari_encode_14bits_range(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s, Word16 cum_freq_low, Word16 cum_freq_high)
{
    Word32 low, high, range;

    range = L_sub(s->high, s->low);

    high = L_add(s->low, mul_sbc_14bits(range, cum_freq_high));
    low = L_add(s->low, mul_sbc_14bits(range, cum_freq_low));

    return ari_encode_14bits_high_low(ptr, bp, bits, s, high, low);
}


/*------------------------------------------------------------------------
 * Function: ari_encode_14bits_sign
 *
 * Encode a sign with equal probabilities.
 *-------------------------------------------------------------------------*/
Word16 ari_encode_14bits_sign(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s, Word16 sign)
{
    Word32 low, high, range;
    Word32 L_tmp;

    high = L_add(s->high, 0);
    low  = L_add(s->low, 0);
    range = L_sub(high, low);

    L_tmp = L_shr(range, 1);
    if (sign != 0)
    {
        high = L_add(low, L_tmp);
    }
    if (sign == 0)
    {
        low = L_add(low, L_tmp);
    }

    return ari_encode_14bits_high_low(ptr, bp, bits, s, high, low);
}

/*------------------------------------------------------------------------
 * Function: ari_done_cbr_encoding_14bits
 *
 * Finish up encoding in CBR mode.
 *-------------------------------------------------------------------------*/
Word16 ari_done_cbr_encoding_14bits(Word16 *ptr, Word16 bp, Word16 bits, TastatEnc *s)
{
    Word16 high, tmp, k;

    tmp = sub(bits, 16);
    WHILE (sub(sub(tmp, bp), s->vobf) > 0)
    {
        bp = ari_encode_14bits_sign(ptr, bp, bits, s, 0);
    }

    high = extract_l(L_sub(s->high, 1));

    bp = ari_put_bit_plus_follow(ptr, bp, s->vobf, lshr(high, 15));
    high = lshl(high, 1);

    tmp = s_min(15, sub(bits, bp));
    FOR (k=0; k<tmp; ++k)
    {
        ptr[bp++] = lshr(high, 15);
        move16();
        high = lshl(high, 1);
    }

    return bp;
}

