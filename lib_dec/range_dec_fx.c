/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */


#include "stl.h"


static Word16 rc_dec_read_fx(Decoder_State_fx *st_fx);

/*-------------------------------------------------------------------*
 * rc_dec_init()
 *
 * Initialize range coder
 *-------------------------------------------------------------------*/

void rc_dec_init_fx(
    Decoder_State_fx *st_fx,          /* i/o: Decoder State       */
    Word16 tot_bits             /* i  : Total bit budget    */
)
{
    Word16 i;

    st_fx->rc_low_fx = L_deposit_l(0);
    st_fx->rc_range_fx = 0xffffffff;
    move32();
    st_fx->rc_num_bits_fx = 0;
    move16();
    st_fx->rc_offset_fx = add(tot_bits, st_fx->next_bit_pos_fx);
    st_fx->rc_end_fx = st_fx->rc_offset_fx;
    move16();

    FOR (i = 0; i < 4; i++)
    {
        st_fx->rc_low_fx = UL_addNsD(UL_lshl(st_fx->rc_low_fx, 8), UL_deposit_l(rc_dec_read_fx(st_fx)));
    }
}

/*-------------------------------------------------------------------*
 * rc_decode()
 *
 *  Decode symbol
 *-------------------------------------------------------------------*/

UWord32 rc_decode_fx(           /* o  : Decoded cumulative frequency    */
    Decoder_State_fx *st_fx,          /* i/o: Decoder State                   */
    UWord32 tot                 /* i  : Total cumulative frequency      */
)
{
    UWord32 inv, lsb, val, UL_tmp1, UL_tmp2;
    Word16 exp_num, exp_den, exp;

    inv = UL_inverse(tot, &exp);
    Mpy_32_32_uu(st_fx->rc_range_fx, inv, &(st_fx->rc_help_fx), &lsb);/*0+exp-32*/
    st_fx->rc_help_fx = UL_lshr(st_fx->rc_help_fx, sub(exp, 32));

    exp_den = norm_ul(st_fx->rc_help_fx);
    UL_tmp2 = UL_lshl(st_fx->rc_help_fx, exp_den);
    exp_num = sub(norm_ul(st_fx->rc_low_fx), 1);
    UL_tmp1 = UL_lshl(st_fx->rc_low_fx, exp_num);
    exp = add(32, sub(exp_num, exp_den));

    val = UL_div(UL_tmp1, UL_tmp2);
    val = UL_lshr(val, exp);

    UL_tmp1 = UL_Mpy_32_32(val, st_fx->rc_help_fx);
    UL_tmp2 = UL_Mpy_32_32(UL_addNsD(val, 1), st_fx->rc_help_fx);
    UL_tmp1 = UL_subNsD(st_fx->rc_low_fx, UL_tmp1);
    UL_tmp2 = UL_subNsD(st_fx->rc_low_fx, UL_tmp2);
    if (UL_tmp2 < UL_tmp1)
    {
        val = UL_addNsD(val, 1);
    }

    return val;
}

/*-------------------------------------------------------------------*
 * rc_dec_update()
 *
 *  Update range coder
 *-------------------------------------------------------------------*/

void rc_dec_update_fx(
    Decoder_State_fx *st_fx,              /* i/o: Decoder State           */
    UWord32 cum_freq,               /* i  : Cumulative frequency    */
    UWord32 sym_freq                /* i  : Symbol frequency        */
)
{
    st_fx->rc_low_fx = UL_subNsD(st_fx->rc_low_fx, UL_Mpy_32_32(cum_freq, st_fx->rc_help_fx));    /*0+0*/
    st_fx->rc_range_fx = UL_Mpy_32_32(st_fx->rc_help_fx, sym_freq);

    WHILE (st_fx->rc_range_fx < 1<<24)
    {
        L_sub(0, 0); /* For comparision in while*/
        st_fx->rc_num_bits_fx = add(st_fx->rc_num_bits_fx, 8);
        st_fx->rc_low_fx = UL_addNsD(UL_lshl(st_fx->rc_low_fx, 8), UL_deposit_l(rc_dec_read_fx(st_fx)));
        st_fx->rc_range_fx = UL_lshl(st_fx->rc_range_fx, 8);
    }
}

/*-------------------------------------------------------------------*
 * rc_dec_bits()
 *
 *  Encode bits
 *-------------------------------------------------------------------*/

Word32 rc_dec_bits_fx(          /* i  : Decoded value   */
    Decoder_State_fx *st_fx,          /* i/o: Decoder State   */
    Word16 bits                 /* i  : Number of bits  */
)
{
    Word32 value;

    st_fx->rc_num_bits_fx = add(st_fx->rc_num_bits_fx, bits);

    IF (sub(bits, 16) > 0)
    {
        st_fx->rc_offset_fx = sub(st_fx->rc_offset_fx, sub(bits, 16));
        value = UL_lshl(UL_deposit_l(get_indice_fx(st_fx, st_fx->rc_offset_fx, sub(bits, 16))), 16);
        st_fx->rc_offset_fx = sub(st_fx->rc_offset_fx, 16);
        value = UL_or(value, UL_deposit_l(get_indice_fx(st_fx, st_fx->rc_offset_fx, 16)));
    }
    ELSE
    {
        st_fx->rc_offset_fx = sub(st_fx->rc_offset_fx, bits);
        value = UL_deposit_l(get_indice_fx(st_fx, st_fx->rc_offset_fx, bits));
    }

    return value;
}

/*-------------------------------------------------------------------*
 * rc_dec_uniform()
 *
 * Encode with uniform distribution
 *-------------------------------------------------------------------*/

UWord32 rc_dec_uniform_fx(         /* i  : Decoded value   */
    Decoder_State_fx *st_fx,              /* i/o: Decoder State   */
    UWord32 tot                     /* i  : Maximum value   */
)
{
    UWord32 value;
    Word16 n;

    /*n = log2_i(tot - 1) + 1;*/
    n = sub(32, norm_ul(tot - 1));

    IF (sub(n, 8) <= 0)
    {
        value = rc_decode_fx(st_fx, tot);
        rc_dec_update_fx(st_fx, value, 1);
    }
    ELSE
    {
        n = sub(n, 8);
        value = rc_decode_fx(st_fx, UL_addNsD(UL_lshr(tot, n), 1));
        rc_dec_update_fx(st_fx, value, 1);
        value = UL_lshl(value, n);
        value = UL_or(value, rc_dec_bits_fx(st_fx, n));
    }

    return value;
}

/*-------------------------------------------------------------------*
 * rc_dec_finish()
 *
 *  Finalize range decoder
 *-------------------------------------------------------------------*/

void rc_dec_finish_fx(Decoder_State_fx *st_fx)
{
    st_fx->next_bit_pos_fx = st_fx->rc_end_fx;
}

/*-------------------------------------------------------------------*
 * rc_dec_read()
 *
 *  Read a byte from bit stream
 *-------------------------------------------------------------------*/


static Word16 rc_dec_read_fx(Decoder_State_fx *st_fx)
{
    return get_next_indice_fx(st_fx, 8);
}


