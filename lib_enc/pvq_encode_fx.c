/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"
#include "stl.h"
#include "prot_fx.h"
#include "rom_com_fx.h"

#include <assert.h>

/*   PVQ MIXED_SEARCH_LOOP:
                           low precision 16/32 +  energy selective high precision 32/64,
                           mixed perf , 10 dB SEGSNR better than the low precision loop only,
                           active  if k>=128 and accumulated energy is high enough,
                           comes at a controlled complexity cost, as dimensions decrease for high k's*/

static
Word16 max_val_fx(                     /* o  : maximum value in the input vector              */
    const Word16 *vec,   /* i  : input vector                                   */
    const Word16 lvec    /* i  : length of input vector                         */
)
{
    Word16 j,tmp;

    tmp = vec[0];
    move16();
    FOR ( j=1 ; j<lvec ; j++ )
    {
        tmp = s_max(vec[j],tmp);
    }
    return tmp;
}

/* The inner search loop for one single additional unit pulse, starting from  pulse_tot  ,
    with information about required energy precision/down scaling for the dim loop in  en_dn_shift,
    and the current max_xabs absolute value to be used for an near optimal  correlation upscaling.
    returns the index of the best positioned unit pulse in imax
*/
static
Word16 one_pulse_search(const Word16 dim,       /* vector dimension       */
                        const Word16* x_abs,    /* absolute vector values */
                        Word16* y,              /* output vector          */
                        Word16 *pulse_tot_ptr,
                        Word32* L_xy_ptr,       /* accumulated correlation */
                        Word32* L_yy_ptr,       /* accumulated energy      */
                        Word16 high_prec_active,
                        Word16 en_dn_shift,
                        Word16 max_xabs)         /* current accumulated max amplitude for pulses */
{
    Word16  i, corr_up_shift, corr_tmp, imax,  corr_sq_tmp, en_max_den, cmax_num, en_tmp;
    Word32  L_tmp_en_lc, L_tmp_corr  ;
    Word32  L_tmp_en, L_en_max_den, L_corr_sq_max, L_tmp_corr_sq;
    Word32  L_left_h, L_right_h;
    UWord32 UL_left_l, UL_right_l, UL_dummy;
    Word32  L_tmp;
    UWord16 u_sgn;

    en_tmp           = en_dn_shift; /* dummy assignment to avoid compiler warning for unused parameter  */

    /* maximize correlation precision, prior to every unit pulse addition in the vector */
    corr_up_shift     =  norm_l(L_mac(*L_xy_ptr, 1, max_xabs)); /* pre analyze worst case L_xy update in the dim  loop        , 2 ops */
    imax              = -1;             /* not needed for search, only added to avoid compiler warning     */

    /* clean BE code, with split out low/high precision loops                                                      */
    /* activate low complexity en/corr search section conditionally if resulting vector energy is within limits    */
    /* typical case for higher dimensions                                                                          */

    IF( high_prec_active == 0 )
    {
        en_max_den        =  0; /*move16()*/; /* OPT: move saved by  using high_prec_active as en_max_den */         /*      1 op   */
        cmax_num          = -1;
        move16();    /* req. to force a 1st update for n==0   */                            /*      1 op   */

        FOR(i = 0; i < dim; i++)                                                                                     /* FOR 3 ops  */
        {
            L_tmp_corr        = L_shl(L_mac(*L_xy_ptr,1,x_abs[i]), corr_up_shift );     /*  actual in-loop target    value, 2 ops  */
            corr_tmp          = round_fx(L_tmp_corr);                                                                /*     1 op   */
            corr_sq_tmp       = mult(corr_tmp, corr_tmp);  /* CorrSq, is a 16bit for low compelxity cross multiplication    1 op   */

            L_tmp_en_lc       = L_mac(*L_yy_ptr, 1,y[i] );   /*Q1 result ,  energy may span up to ~14+1(Q1)+1(sign)=16 bits,  1 op */
            /* extract_l without shift can always be used for this section as energy is guaranteed to stay in the lower word, 1 op */
            en_tmp     = extract_l(L_tmp_en_lc); /* L_shl + round_fx could also be used also but then adds an uphift cost (2-3 ops)*/

            /* 16/32 bit comparison    WC (4 +1+1 + (1+1+1) = 9                                                                   */
            /*  corr_sq_tmp/en_tmp_den    >   cmax_num/en_max_den  */
            /*  corr_sq_tmp * en_max_den  >   cmax_num * en_tmp    */                  /* IF                                4 ops  */
            IF( L_msu(L_mult(corr_sq_tmp, en_max_den),cmax_num , en_tmp)  > 0)         /* use L_mult and then a L_msu,      2 ops  */
            {
                cmax_num    = corr_sq_tmp;
                move16();    /* 1 op */
                en_max_den  = en_tmp;
                move16();    /* 1 op */
                imax        = i;
                move16();    /* 1 op */
            }
        } /* dim  */

    }
    ELSE
    {
        /* High resolution section activated when vector energy is becoming high  (peaky or many pulses)                    */
        /* BASOP operator Mpy32_32_ss used to allow higher resolution for both the CorrSq term and the Energy term          */
        /* Performance:   close to float reference                                                                          */


        L_en_max_den  = L_deposit_l(0);                                                                            /* 1 op  */
        L_corr_sq_max = L_deposit_l(-1); /* req. to force a 1st update   */                                        /* 1 op  */

        FOR(i = 0; i < dim; i++)                                                                               /* FOR 3 ops */
        {
            L_tmp_corr        = L_shl(L_mac(*L_xy_ptr,1,x_abs[i]), corr_up_shift );     /* actual in  loop WC value 2 ops   */
            Mpy_32_32_ss(L_tmp_corr,L_tmp_corr, &L_tmp_corr_sq, &UL_dummy);             /* CorrSq 32 bits,          4 ops   */

            L_tmp_en = L_mac(*L_yy_ptr, 1, y[i]);                         /* Q1,energy may span up to sign+19 bits , 1 op    */
            /* For highest accuracy use pairs of maximum upshifted 32x32 bit signed values              */
            /*  (L_tmp_corr_sq / L_tmp_en)     >  (L_corr_sq_max/L_en_max_den)                          */
            /*  (L_tmp_corr_sq * L_en_max_den) >  (L_corr_sq_max * L_tmp_en)                            */
            Mpy_32_32_ss( L_en_max_den, L_tmp_corr_sq, &L_left_h, &UL_left_l);                  /* 4 ops */
            Mpy_32_32_ss( L_tmp_en, L_corr_sq_max, &L_right_h, &UL_right_l);                    /* 4 ops */

            /* STL optimized "Lazy evaluation"  of:
               IF( (L_left_h > L_right_h)  ||  ( (L_left_h == L_right_h) &&  (UL_left_l > UL_right_l) )
             */
            /* 32/64 bit Lazy eval comparison WC cost is    (1+  1+1+1 + 4 +(2+2+1) = 13 ,  and average  is  ~12 */
            /* Unoptimized 32/64 bit comparison  WC cost is (1+1+ 2x2  + 4 +(2+2+1) = 15 */
            L_tmp  = L_sub(L_left_h, L_right_h);              /* high  signed  word check            1 op  */
            u_sgn = 0;
            move16();                    /* 1 op  */
            if(L_tmp == 0) /* L_tmp high Word testing is always needed */
            {
                /* The returned UL value from UL_subNs is not needed,  only u_sgn is needed  */
                UL_subNs(UL_right_l, UL_left_l, &u_sgn);  /* low unsigned word check, note left/right order switch of ">"  due to ">=" inside UL_subNs, 1 op */
            }
            if( u_sgn != 0)
            {
                L_tmp = L_add(L_tmp, 1);    /* 0+1  --> 1 use wrap/sign result of low Word u_sgn check */         /* 1 op  */
            }
            IF(  L_tmp > 0 )   /* IF  4 ops */
            {
                L_corr_sq_max =  L_add(L_tmp_corr_sq, 0);  /* 1-2 ops */
                L_en_max_den  =  L_add(L_tmp_en, 0);       /* 1-2 ops */
                imax          =  i;
                move16(); /* 1 op  */
            }
        }  /* dim loop */

    }
    /* Complexity comparison per coeff for low precision vs. high precision
        low  precision: pulse_tot <= 127, 16 bit:  WC  2+3 +(15)*dim    ops,            dim=5  --> 5+15*5 = 90  ops, 18 ops/coeff
        high precision: pulse_tot  > 127, 32 bit:  WC  1+3+3 +(26-28)*dim  ops, WC-band dim=5  --> 7+28*5 = 147 ops, 29 ops/coeff  ~61% increase
    */

    /*  finally add found unit pulse contribution to past L_xy, Lyy,  for next pulse loop    */
    *L_xy_ptr = L_mac(*L_xy_ptr, x_abs[imax], 1);            /* xy_flt += xabs[imax];  Q12+1 */
    *L_yy_ptr = L_mac(*L_yy_ptr, 1, y[imax]);                /* yy_flt += y[imax];           */

    y[imax]           = add(y[imax],1);
    move16();       /* Q0 added pulse              */
    (*pulse_tot_ptr)  = add((*pulse_tot_ptr) ,1);             /* increment total pulse sum   */


    return imax;
}

/*-----------------------------------------------------------------------*
 * Function pvq_encode_fx()                                              *
 *                                                                       *
 * Basic PVQ search algorithm:                                           *
 *  Selective L1-norm projection to the lower ~(K-1) pyramid             *
 *  flollowed by an ACELP-like unit pulse addition fine search           *
 *-----------------------------------------------------------------------*/

void pvq_encode_fx(
    Encoder_State_fx *st_fx,
    const Word16 *x,         /* i:   vector to quantize             Q15-3=>Q12       */
    Word16 *y,         /* o:   raw pulses  (non-scaled short) Q0               */
    Word16 *xq,        /* o:   quantized vector               Q15              */
    Word32 *L_xq,      /* o:   quantized vector               Q31 fot eval     */
    const Word16 pulses,     /* i:   number of allocated pulses                      */
    const Word16 dim,        /* i:   Length of vector                                */
    const Word16 neg_gain    /* i:  - Gain       use - negative gain in  Q15  0..1   */
)
{
    Word16  i;
    Word16 pulse_tot;
    Word16 xabs[PVQ_MAX_BAND_SIZE];
    Word16 max_xabs;
    Word32 L_xsum;
    Word32 L_proj_fac;
    Word32 L_yy, L_xy;
    Word16  max_amp_y, imax;
    Word16  k, en_margin, en_dn_shift, high_prec_active ;

    Word32 L_num, L_tmp;
    Word16 proj_fac, tmp, shift_den,shift_num,shift_delta, num,den;

    UWord16 u16_tmp;
    Word16 dim_m1;
    Word32 L_isqrt;
    Word16 neg_gain_norm, shift_tot;
    Word16    high_pulse_density_flag;
    PvqEntry_fx entry;

    L_proj_fac = 4096;

    /* Create absolute vector and calculate sum of abs vector  */
    L_xsum   =  L_deposit_h(0);
    max_xabs =  -1;
    move16();

    FOR( i = 0; i < dim; i++)
    {
        xabs[i]    = abs_s(x[i]);
        move16();           /* Q12 */
        max_xabs   = s_max(max_xabs, xabs[i] );                       /* for efficient  search correlation scaling */
        L_xsum     = L_mac0(L_xsum, 1, xabs[i] );                     /* stay in Q12 */
        y[i]       = 0;
        move16();          /* init, later only non-zero values need to be normalized */
    }

    test();
    IF( L_xsum == 0  ||  neg_gain == 0  )
    {
        /*  zero input  or  zero gain  case */
        /*  put a "half"-pulsesum first and last,  or all pulses in pos 0 */
        pulse_tot = pulses;
        move16();
        dim_m1    = sub(dim,1);
        y[dim_m1] = 0;
        move16();
        y[0]      = shr(pulses,1);
        move16();
        y[dim_m1] = add(y[dim_m1], sub(pulses, y[0]));
        move16();
        L_yy      = L_mult(y[0],y[0]); /* L_yy needed for normalization */
        if(dim_m1 != 0)
        {
            L_yy = L_mac(L_yy, y[dim_m1],y[dim_m1]);  /* (single basop) */
        }
    }
    ELSE
    {
        /* regular search */
        /* proj_fac_flt = (pulses - 1)/xsum_flt; */  /* normalize to L1 norm = abs pyramid */
        num = sub(pulses,1);
        high_pulse_density_flag = 0;
        move16();
        if ( sub(pulses, shr(dim, 1)) > 0 )
        {
            high_pulse_density_flag = 1;
            move16();
        }

        test();
        IF( (num > 0) && (high_pulse_density_flag != 0) )
        {
            shift_den = norm_l(L_xsum);                      /* x_sum input  Q12                         */
            den       = extract_h(L_shl(L_xsum, shift_den)); /* now in Q12+shift_den                     */

            L_num     = L_deposit_l(num);
            shift_num = sub(norm_l(L_num)   ,1);
            L_num     = L_shl(L_num, shift_num) ;             /* now in Q0 +shift_num -1                  */
            proj_fac  = div_l(L_num,  den );                  /* L_num always has to be less than den<<16 */

            shift_delta=sub(shift_num,shift_den);
            L_proj_fac = L_shl(L_deposit_l(proj_fac), sub(9, shift_delta));  /* bring  to a fixed  Q12     */
        }

        pulse_tot = 0;
        move16();
        L_yy = L_deposit_l(0);
        L_xy = L_deposit_l(0);

        /* Find a start position on a lower sub pyramid,  if the pulse density is larger than 0.5 */
        /* and projection is required */
        test();
        IF( (num > 0 ) && (high_pulse_density_flag != 0 ) )
        {
            FOR( i = 0; i < dim ; i++) /* max 64 */
            {
                /* y_flt[i]   = (short)floor(xabs_flt[i] * proj_fac_flt); */    /* FLOAT  pyramid truncation */
                /* y[i]       =  shr(xabs[i]*L_proj_fac);                 */    /* BASOP  pyramid truncation */
                Mpy_32_16_ss(L_proj_fac,xabs[i],&L_tmp,&u16_tmp);  /*Q12 *Q12  +1 */
                y[i]        = extract_l(L_shr( L_tmp, 12+12-16+1 ));
                move16();/* Q12 *Q12  ->  Q0 */  /* the pyramid-"truncation" */

                pulse_tot   = add(pulse_tot, y[i]);          /* Q0                                         */
                L_yy        = L_mac(L_yy, y[i], y[i]);       /* Energy, result will scale up by 2 by L_mac */
                L_xy        = L_mac(L_xy, xabs[i], y[i]);    /* Corr, Q0*Q12  +1 --> Q13                   */
            }
        }

        /*  PVQ fine search loop  description            */
        /*  Run an ACELP-like full CorrSq/Energy  search */
        /*  the basic search  logic is to test adding one unit pulse to each position n in the output vector y
            and update the correlation and energy terms iteratively.

           Rxy(k,n)   = Rxy(k-1) + 1*abs(x(k,n))         % add target times unit correlation
           Ryy(k,n)   = Ryy(k-1) + 2*y(k-1,n) + 1^2      % added unit energy contribution
           RxySq(k,n) = Rxy(k,n)*Rxy(k,n)

           minimize ( -Rxy(k,n)/Sqrt(Ryy(k,n)) ),          over n  and k
           ( or equiv. maximize ( RxySq(k,n)/Ryy(k,n) ),  over n  and k  )

              RxySq(k,n)          bestRxySq
              -----------  >     ---------    ? update best candidate
              Ryy(k,n)              bestRyy

              bestRyy *  RxySq(k,n)  >   bestRxySq *  Ryy(k,n)   ?  update best candidate

              Misc:
               1^2 in Ryy(k,n) can be added before the dimension loop
               Ryy(k,n)  can optionally be pre down-scaled by 2.0 to avoid one multiplication/shift
               Rxy(k,n) can dynamically scaled for each unit pulse loop to optimize operators/precision of RxySq.
               CorrSqTerm =(Rxy(k,n))^2, should have maximum precision in a Word16/Word32 variable

               Energy term:Ryy(k, n)
                 should not become zero, i.e. most often have enough precision to not become truncated
                 i.e. preferably use a 32 bit variable if more than 127 pulses are accumulated in Q1.

                 k=127=>log2(127^2 ) = 13.9774 bits,  fits in  Q1 signed, with Q1 the  pre division by 2  works
                 k=255=>log2(255^2 ) = 15.9887 bits,  i.e 1 bits scaling, fits in Q1 unsigned 16 bit value

               Dimension aspects:
                 dim 6 and higher is always 16 bit energy safe as max pulses is 96 (for 1+31 bits short codewords)
                 dims 5,4,3,2  may be allocated  higher than 127 pulses (or lower)
                 dim 1 does not really need a search, only the  sign is relevant.
                 Currently we have  KMAX=512,  or more precisly for DIM 5       kmax is 238
                                                                    DIM 4,3,2,1 kmax is 512
               Unsigned DSP arithmetic could also be used,
                  but then the "startup" case/ now a negative sentinel has to be handled in a special way
                  and further currently UL_mac,UL_mac0,UL_msu,UL_msu0 is not available as STL unsigned basops

                 To make the PVQ search code work effciently we do:
                         1. Always use near optimal norm_l based dynamic up-scaling of the correlation term,
                             by analyzing the  accumulated correlation sofar.  (Rxy for k-1 pulses)
                             and using a pre-analyzed maximum possible target value  max_xabs.
                         2. Selectively use 16 or 32 bit representation of the energy term, depending on the pre-accumulated energy.
                             further if the energy term Ryy needs more than 16 bits, we switch to a 32 bit energy representation
                             and also a higher 32 bit precision for teh correlation Rxy and
                             increase the precision in the distortion cross-multiplication calculations to near 64 bits.
        */

        /* Rxy_max=(k,*)= max( L_xy + 1*max_x_abs)                analysis needed for  near optimal L_xy normalization (corr, corrSq)    */
        /* Ryy_max(k,*) = max(L_yy(k-1) + .5  + 1*max_amp_y(k-1)) analysis needed for energy (L_yy) precision selection when pulses > 127 , ( (127^2)*2 fits in Signed Word16) */

        L_yy=L_shr(L_yy,1);   /* scale down by 2  for search loop  domain  */
        IF (sub(pulses,127)<=0 )
        {
            /* LC inner loop, enters here always for dimensions 6 and higher, and also sometimes for dimensions 1 .. 5  */
            /* ( if  high energy precision is inactive,  max_amp_y is not needed , no max_amp_y(k-1) update )           */
            FOR (k=pulse_tot; k<pulses; k++)
            {
                L_yy = L_add(L_yy,1);  /* .5 added to energy for pulse_tot+1 */
                imax = one_pulse_search(dim, xabs,y,&pulse_tot,&L_xy,&L_yy,0, 0,max_xabs);
            }
        }
        ELSE
        {  /* HC or LC+HC inner loops */
            max_amp_y = max_val_fx(y, dim); /* this loops over max 5 values (as pulses are dimension  restricted)     */
            /* max_amp_y from projected y is needed when pulses_sum  exceeds 127      */

            /* First section with 32 bit energy inactive,   max_amp_y kept updated though    */
            FOR( k=pulse_tot; k<128; k++)
            {
                L_yy        = L_add(L_yy,1);                            /* .5 added */
                imax        = one_pulse_search(dim, xabs,y,&pulse_tot,&L_xy,&L_yy,0,0,max_xabs);
                max_amp_y   = s_max(max_amp_y, y[imax]);
            }

            /* Second section with higher number of pulses, 32 bit energy precission adaptively selected, max_amp_y kept updated                */
            FOR( k=pulse_tot; k<pulses; k++)
            {
                L_yy        = L_add(L_yy,1);                    /* .5 added                                                  */
                en_margin   = norm_l(L_mac(L_yy,1, max_amp_y)); /* find max current energy "addition", margin,  ~ 2 ops      */
                en_dn_shift = sub(16, en_margin);               /* calc. shift to lower byte for fixed use of extract_l      */

                high_prec_active = 1;
                move16();
                if( en_dn_shift <= 0 )
                {
                    /* only use 32 bit energy if actually needed */
                    high_prec_active = 0;
                    move16();
                }
                /* 32 bit energy and corr adaptively active,  max_amp_y kept updated */
                imax        = one_pulse_search(dim,xabs,y,&pulse_tot,&L_xy,&L_yy,high_prec_active, en_dn_shift, max_xabs);
                max_amp_y   = s_max(max_amp_y, y[imax]);
            }
        }
        L_yy = L_shl(L_yy,1);  /*  yy= yy*2.0  */  /* compensate search loop analysis energy downshift by 1,
                                                   to make energy right for unit/inverse gain calculation */
    }

    /* Apply unit energy normalization scaling,  always at least one pulse so no div-by-zero check is needed */
    L_isqrt        = L_deposit_l(0);
    IF( neg_gain != 0  )
    {
        L_isqrt = Isqrt(L_shr(L_yy,1));      /* Note: one single gain factor  as in flt �s  not computed */
    }

    shift_num     = norm_s(pulse_tot);                      /* account for max possible pulse amplitude in y,
                                                               can be used even when max_amp_y is not avail.  */
    shift_den     = norm_s(neg_gain);                        /* account for gain downscaling shift            */
    neg_gain_norm = shl(neg_gain, shift_den);                /* up to 10 dB loss without this norm            */
    shift_tot     = sub(add(shift_num, shift_den), 15);

    L_isqrt = L_negate(L_isqrt);
    FOR( i = 0; i < dim; i++)
    {
        tmp = shl(y[i],shift_num);  /* upshifted abs(y[i]) used  in scaling */
        if( x[i] < 0 )
        {
            tmp = negate(tmp);                                    /* apply sign */
        }

        if( y[i] != 0 )
        {
            y[i]      = shr(tmp, shift_num);
            move16();           /* updates sign of y[i} , ~range -512 + 512),  array move */
        }
        Mpy_32_16_ss( L_isqrt, tmp, &L_tmp, &u16_tmp);            /* Q31*Q(0+x)  +1         */
        Mpy_32_16_ss( L_tmp, neg_gain_norm , &L_tmp, &u16_tmp);   /* Q31*Q(0+x) *Q15 +1     */
        L_tmp   = L_shr(L_tmp, shift_tot);                        /* Q31+x                  */
        xq[i]   = round_fx(L_tmp);                                /* Q15, array move        */
        L_xq[i] = L_tmp;                                          /* Q31 currently  unused  */
    }

    /* index the found PVQ vector into short codewords */
    entry = mpvq_encode_vec_fx(y, dim, pulses);

    /* send the short codeword(s) to the range encoder */
    rc_enc_bits_fx(st_fx,  UL_deposit_l(entry.lead_sign_ind) , 1);  /* 0 or 1 */
    IF( sub( dim, 1) != 0 )
    {
        rc_enc_uniform_fx(st_fx, entry.index, entry.size);
    }

    return;
}
