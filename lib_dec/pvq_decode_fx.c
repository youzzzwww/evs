/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"
#include "rom_com_fx.h"

/*-------------------------------------------------------------------*
* Function pvq_decode_fx()                                             *
*                                                                   *
* PVQ subvector decoding algorithm                                  *
*-------------------------------------------------------------------*/

void pvq_decode_fx(
    Decoder_State_fx *st_fx,
    Word16 *xq,        /* o:   decoded vector (Q15)             */
    Word16 *y,         /* o:   decoded vector (non-scaled int)  */
    const Word16 k_val,      /* i:   number of allocated pulses       */
    const Word16 dim,        /* i:   Length of vector                 */
    const Word16 neg_gain    /* i:   Gain    (negated to fit 1.0 in Q15 as -1.0)  */
)
{
    Word16 i;

    UWord32  h_mem[1+KMAX_NON_DIRECT_FX+1];              /* allocate max offset memory for dim  6 */

    PvqEntry_fx entry;

    Word16 neg_gain_norm, shift_num,shift_den,shift_tot;
    Word32 L_yy,L_isqrt,L_tmp;
    UWord16 u16_tmp;

    entry = get_size_mpvq_calc_offset_fx(dim, k_val, h_mem); /* get size & prepare H(adaptive table for entry.size=N_MPVQ(dim,k_val) */

    IF( sub(dim, 1) != 0)
    {
        entry.lead_sign_ind  = (short)rc_dec_bits_fx(st_fx, 1);
        entry.index = rc_dec_uniform_fx(st_fx, entry.size);


    }
    ELSE
    {
        entry.lead_sign_ind = (short)rc_dec_bits_fx(st_fx, 1);   /* always a single sign bit */
        entry.index         = L_deposit_l(0);
    }

    mpvq_decode_vec_fx(&entry, h_mem, y);

    IF( neg_gain == 0  )
    {
        FOR(i=0; i<dim ; i++)
        {
            xq[i]=0;
            move16();
        }
    }
    ELSE
    {
        L_yy = L_deposit_l(0);
        FOR(i=0; i<dim ; i++)
        {
            L_yy=L_mac(L_yy,y[i],y[i]);      /* Q1 */
        }

        L_isqrt=Isqrt(L_shr(L_yy,1));         /*Q31*/       /* one single gain fac  as in flt not computed  for now */

        shift_num= norm_s(k_val);                           /* account for max possible pulseamp in y */
        shift_den= norm_s(neg_gain);                        /* account for downscaling shift         */
        neg_gain_norm  =shl(neg_gain,shift_den);            /* 10 db loss in minSNR without this in L_qx , at HQ128 FB*/
        shift_tot=sub(add(shift_num,shift_den),15);

        L_isqrt=L_negate(L_isqrt);
        FOR( i = 0; i < dim; i++)
        {
            /* upshifted y[i]  used    */

            Mpy_32_16_ss( L_isqrt, shl(y[i],shift_num) , &L_tmp, &u16_tmp);   /*  Q31*Q(0+x)  *2*/
            Mpy_32_16_ss( L_tmp, neg_gain_norm , &L_tmp, &u16_tmp);           /*  Q31*Q(0+x) *Q15 *2*/
            L_tmp=L_shr(L_tmp,shift_tot);
            xq[i]   = round_fx(L_tmp);                                        /* Q15 , array move   */
        }
    }



    return;
}

