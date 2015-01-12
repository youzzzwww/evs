/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <limits.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */

#include "stl.h"

static void rc_enc_shift_fx(Encoder_State_fx *st_fx);
static void rc_enc_write_fx(Encoder_State_fx *st_fx, Word16 byte, Word16 bits);

/*-------------------------------------------------------------------*
 * rc_enc_init()
 *
 *  Initalize range coder
 *-------------------------------------------------------------------*/

void rc_enc_init_fx(
    Encoder_State_fx *st_fx,    /* i/o: Encoder state       */
    Word16 tot_bits             /* i  : Total bit budget    */
)
{
    st_fx->rc_low_fx = L_deposit_l(0);
    st_fx->rc_range_fx = 0xffffffff;
    move32();
    st_fx->rc_cache_fx = -1;
    move16();
    st_fx->rc_carry_fx = 0;
    move16();
    st_fx->rc_carry_count_fx = 0;
    move16();
    st_fx->rc_num_bits_fx = 0;
    move16();
    st_fx->rc_tot_bits_fx = tot_bits;
    move16();
    st_fx->rc_offset_fx = 0;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * rc_encode()
 *
 *  Encode symbol with range coder
 *-------------------------------------------------------------------*/

void rc_encode_fx(
    Encoder_State_fx *st_fx,   /* i/o: Encoder state                       */
    UWord32 cum_freq,          /* i  : Cumulative frequency up to symbol   */
    UWord32 sym_freq,          /* i  : Symbol probability                  */
    UWord32 tot                /* i  : Total cumulative frequency          */
)
{
    UWord32 r, tmp, inv_tot, lsb;
    Word16 exp;
    UWord16 carry;

    inv_tot = UL_inverse(tot, &exp);
    Mpy_32_32_uu(st_fx->rc_range_fx, inv_tot, &tmp, &lsb);  /*0+exp-32 */
    r = UL_lshr(tmp, sub(exp, 32));  /* exp-32-exp3+32 = 0 */
    tmp = UL_Mpy_32_32(r, cum_freq);

    st_fx->rc_low_fx = UL_addNs(st_fx->rc_low_fx, tmp, &carry);
    if (carry != 0)
    {
        st_fx->rc_carry_fx = carry;
        move16();
    }

    st_fx->rc_range_fx = UL_Mpy_32_32(r, sym_freq);

    WHILE (st_fx->rc_range_fx < 1<<24)
    {
        L_sub(0, 0);   /* Comparison in while */
        st_fx->rc_range_fx = UL_lshl(st_fx->rc_range_fx, 8);
        st_fx->rc_num_bits_fx = add(st_fx->rc_num_bits_fx, 8);
        rc_enc_shift_fx(st_fx);
    }

    return;
}

/*-------------------------------------------------------------------*
 * rc_enc_finish()
 *
 *  Finalize range coder
 *-------------------------------------------------------------------*/

void rc_enc_finish_fx(
    Encoder_State_fx *st_fx           /* i/o: Encoder state       */
)
{
    UWord32 val, mask, high;
    Word16 bits;
    UWord16 over1, over2;

    /*bits = 32 - log2_i(st->rc_range); */
    bits = add(norm_ul(st_fx->rc_range_fx), 1);
    mask = UL_lshr(0xffffffff, bits);

    val = UL_addNs(st_fx->rc_low_fx, mask, &over1);
    high = UL_addNs(st_fx->rc_low_fx, st_fx->rc_range_fx, &over2);

    val = L_and(val, ~mask);
    L_xor(0,0);   /* For bit not */

    IF ( (L_xor(over1, over2)) == 0 )
    {
        L_sub(0, 0);   /* For comparision in if */
        IF (UL_addNsD(val, mask) >= high)
        {
            bits = add(bits, 1);
            mask = UL_lshr(mask, 1);
            val = UL_and(UL_addNsD(st_fx->rc_low_fx, mask), ~mask);
            L_xor(0,0);   /* For bit not */
        }

        if (val < st_fx->rc_low_fx)
        {
            st_fx->rc_carry_fx = 1;
            move16();
        }
    }

    st_fx->rc_low_fx = val;
    move32();

    IF ( sub(bits, sub(st_fx->rc_tot_bits_fx, st_fx->rc_num_bits_fx)) > 0 )
    {
        bits = sub(st_fx->rc_tot_bits_fx, st_fx->rc_num_bits_fx);

    }

    st_fx->rc_num_bits_fx = add(st_fx->rc_num_bits_fx, bits);
    FOR ( ; bits > 0; bits -= 8)
    {
        rc_enc_shift_fx(st_fx);
    }
    bits = add(bits, 8);

    IF ( st_fx->rc_carry_count_fx > 0 )
    {
        rc_enc_write_fx(st_fx, add(st_fx->rc_cache_fx, st_fx->rc_carry_fx), 8);

        FOR ( ; st_fx->rc_carry_count_fx > 1; st_fx->rc_carry_count_fx--)
        {
            rc_enc_write_fx(st_fx, (st_fx->rc_carry_fx + 0xff), 8);
        }
        rc_enc_write_fx(st_fx, s_and(add(st_fx->rc_carry_fx, 0xff), sub(lshl(1, bits), 1)), bits);
    }
    ELSE
    {
        rc_enc_write_fx(st_fx, lshr(add(st_fx->rc_cache_fx, st_fx->rc_carry_fx), sub(8, bits)), bits);
    }

    bits = st_fx->rc_num_bits_fx;
    move16();
    WHILE (sub(bits, sub(st_fx->rc_tot_bits_fx, 16)) < 0)
    {
        rc_enc_write_fx(st_fx, 0, 16);
        bits = add(bits, 16);
    }

    bits = sub(st_fx->rc_tot_bits_fx, bits);
    IF (bits > 0)
    {
        rc_enc_write_fx(st_fx, 0, bits);
    }

    return;
}

/*-------------------------------------------------------------------*
 * rc_enc_shift()
 *
 *  Shift a byte out to bitstream
 *-------------------------------------------------------------------*/

static void rc_enc_shift_fx(
    Encoder_State_fx *st_fx           /* i/o: Encoder state       */
)
{
    test();
    L_sub(0, 0);   /* For comparision in if */
    IF (st_fx->rc_low_fx < (0xff000000) || sub(st_fx->rc_carry_fx, 1) == 0)
    {
        IF (st_fx->rc_cache_fx >= 0)
        {
            rc_enc_write_fx(st_fx, add(st_fx->rc_cache_fx, st_fx->rc_carry_fx), 8);
        }

        WHILE (st_fx->rc_carry_count_fx > 0)
        {
            rc_enc_write_fx(st_fx, s_and(add(st_fx->rc_carry_fx, 0xff), 255), 8);
            st_fx->rc_carry_count_fx = sub(st_fx->rc_carry_count_fx, 1);
        }

        st_fx->rc_cache_fx = u_extract_l(UL_lshr(st_fx->rc_low_fx, 24));
        st_fx->rc_carry_fx = 0;
        move16();
    }
    ELSE
    {
        st_fx->rc_carry_count_fx = add(st_fx->rc_carry_count_fx, 1);
    }
    st_fx->rc_low_fx = UL_lshl(st_fx->rc_low_fx, 8);

    return;
}

/*-------------------------------------------------------------------*
 * rc_enc_bits()
 *
 *
 *-------------------------------------------------------------------*/

void rc_enc_bits_fx(
    Encoder_State_fx *st_fx,    /* i/o: Encoder state       */
    UWord32 value,              /* i  : Value to encode     */
    Word16 bits                 /* i  : Number of bits used */
)
{
    IF ( sub(add(rc_get_bits2_fx(st_fx->rc_num_bits_fx, st_fx->rc_range_fx), bits),  st_fx->rc_tot_bits_fx) <= 0)
    {
        st_fx->rc_num_bits_fx = add(st_fx->rc_num_bits_fx, bits);

        IF ( sub(bits, 16) > 0 )
        {
            push_indice_fx(st_fx, sub(IND_RC_END, st_fx->rc_offset_fx), u_extract_l(UL_lshr(value, 16)), sub(bits, 16));
            st_fx->rc_offset_fx = add(st_fx->rc_offset_fx, 1);
            push_indice_fx(st_fx, sub(IND_RC_END, st_fx->rc_offset_fx), u_extract_l(UL_and(value, 0x0000ffff)), 16);
            st_fx->rc_offset_fx = add(st_fx->rc_offset_fx, 1);
        }
        ELSE
        {
            push_indice_fx(st_fx, sub(IND_RC_END, st_fx->rc_offset_fx), u_extract_l(value), bits);
            st_fx->rc_offset_fx = add(st_fx->rc_offset_fx, 1);
        }
    }
    ELSE
    {
    }

    return;
}

/*-------------------------------------------------------------------*
 * rc_enc_uniform()
 *
 * Encode with uniform distribution
 *-------------------------------------------------------------------*/

void rc_enc_uniform_fx(
    Encoder_State_fx *st_fx,    /* i/o: Encoder state       */
    UWord32 value,              /* i  : Value to encode     */
    UWord32 tot                 /* i  : Maximum value       */
)
{
    Word16 n;

    /*n = log2_i(tot-1)+1; */
    n = sub(32, norm_ul(UL_subNsD(tot, 1)));

    IF (sub(n, 8) <= 0)
    {
        rc_encode_fx(st_fx, value, 1, tot);
    }
    ELSE
    {
        n = sub(n, 8);
        rc_encode_fx(st_fx, UL_lshr(value, n), 1, UL_addNsD(UL_lshr(tot, n), 1));
        rc_enc_bits_fx(st_fx, UL_and(value, UL_subNsD(UL_lshl(1, n), 1)), n);
    }

    return;
}

/*-------------------------------------------------------------------*
 * rc_enc_write()
 *
 *  Write a byte to bitstream
 *-------------------------------------------------------------------*/

static void rc_enc_write_fx(
    Encoder_State_fx *st_fx,    /* i/o: Encoder state         */
    Word16 byte,                /* i  : Byte to write         */
    Word16 bits                 /* i  : Number of bits        */
)
{
    push_indice_fx(st_fx, IND_RC_START, byte, bits);

    return;
}
