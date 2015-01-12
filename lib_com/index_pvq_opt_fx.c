/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"           /* required for wmc_tool */
#include "basop_util.h"
#include <assert.h>

#define N_OPT_FX             5       /* complexity setting,  direct functional calculation  limit & low dim recursion limit  */
#define TABLE_LIM_OPT_FX     96      /* odd divisor table , N-row_recursion  limit setting, due to dim                       */


/* local typedefs for optimized pvq indexing,  used locally c-file to vectorize common function calls  */
typedef  void    (*VEC2INDFUNCM) (const Word16* ,  Word16* , UWord32*, UWord32*);
typedef  UWord32 (*NFUNCM)       (Word16);
typedef  UWord32 (*H_FUNCM)      ( UWord32 );
typedef  void    (*IND2VECFUNCM) ( Word16, Word16, UWord32, Word16* ) ;
typedef  void    (*NDIM_FUNCM)   ( Word16, Word16, UWord32, Word16* );

/* local constants for indexing functions  */
#define SIGNBIT_FX   0x80000000u
#define SIGNBIT_SHRT_FX   0x8000
#define UDIVBY3_FX   2863311531U

/*-------------------------------------------------------------------*
 * f_odd_exact_div_opt_fx()
 *
 * find   1/(den1*2+1) * ( num1p*num2p - num3) ,
 *        if the result is known a priori to be exactly a 32bit UWord32
 *-------------------------------------------------------------------*/
static
UWord32  f_odd_exact_div_opt_fx(  /* o  :  see Eq.        */
    UWord32 num1p,        /* i  :  see Eq.        */  /* (2n-1) or  n ,  i.e can be short also */
    UWord32 num2p,        /* i  :  see Eq.        */
    UWord32 num3,         /* i  :  see Eq.        */
    Word16  den1          /* i  :  see Eq.        */  /*range [0..127] can be made to short */
)
{
    UWord32 UL_tmp;
    UL_tmp = UL_Mpy_32_32(exactdivodd_fx[den1],UL_subNsD(UL_Mpy_32_32(num1p,num2p),num3));

    return (UL_tmp);
}

/*---------------------------------------------------------------------------*
 * f_even_exact_div_opt_fx()
 *
 *    returns  (num1p*num2p - num3 )/ den1
 *     if the result is known a priori to be exactly a 32bit unsigned integer
 *--------------------------------------------------------------------------*/
static
UWord32 f_even_exact_div_opt_fx(  /* o  :  see Eq.        */
    UWord32  UL_num1p,         /* i  :  see Eq.    2n-1 or n   can be short input  */
    UWord32 UL_num2p,         /* i  :  see Eq.   range should be larger than num1p   */
    UWord32 UL_num3,          /* i  :  see Eq.                   */
    Word16  den1              /* i  :  see Eq.              */
)
{
    UWord32  UL_tmp, UL_oddfactor;
    Word16   den1_m1, even_sh;
    UWord32 UL_tmp_h;
    UWord16 sgn;

    den1_m1      = sub(den1,1);      /*   remove top bit */
    even_sh      =  sub(15, norm_s(s_xor(den1_m1, den1)));   /* STL signed ops ok as den1 <= 127 */

    UL_oddfactor = exactdivodd_fx[lshr(den1_m1,even_sh)];
    move32();
    even_sh      = sub(even_sh,1);


    Mpy_32_32_uu(UL_num1p, UL_num2p, &UL_tmp_h, &UL_tmp);   /* cost ~4  */
    UL_tmp = UL_subNs(UL_tmp,UL_num3,&sgn);   /* may wrap  for underflow */
    if(sgn)    /* underflow */
    {
        UL_tmp_h = UL_subNsD(UL_tmp_h,1U);  /* single basicop  ->  if */
    }
    UL_tmp = UL_or(UL_lshl(UL_tmp_h,sub(32,even_sh)), UL_lshr(UL_tmp,even_sh));
    /*  total cost  9-11  , old solution had 15-16*/

    /* now use tabled modular multiplicative inverse for the odd part division */
    return  UL_Mpy_32_32(UL_tmp, UL_oddfactor);
}

/*  direct calculation functions for smaller dimensions to speed up indexing

N_MPVQ(1,k) = 1;
N_MPVQ(2,k) = k*2;
N_MPVQ(3,k) = 1+2*(k^2);
N_MPVQ(4,k) = k/3 * 4*(k^2+2);
N_MPVQ(5,k) = 1 +  2*(k*k*(5+k*k))/3;
N_MPVQ(*,k) = iterations   =  1 + A(n,k) + A(n+1,k);

N_PVQ(n,k) = 2*N_MPVQ(n,k);


A(1,k) = 1;
A(2,k) = -1 + 2k;
A(3,k) = 1+2(k-1)k;
A(4,k) = 1/3*(((4k-6)k+8)*k-3),
A(5,k) = 1/3*(3+ k(10+2(k-2)k)k-8);
A(*,k) = recursive iterations;


U(n,k) = (A(n,k)-1)/2;
U(1,k) = 0;
U(2,k) = k-1;
U(3,k) = k*(k-1)
U(4,k) = (1/3)*((k - 1)*(2*k^2 - k + 3))
U(5,k) = (1/3)*(k*(k - 1)*(k^2 - k + 4))
U(*,k) = recursive iterations;

U(n,k) =  U(k,n);
U(n,k) =  func(n, U(n,k-1), U(n,k-2) , 1/(k-1) );
U(n,k) =  1 + U(n-1,k-1)    + U(n-1,k)     + U(n,k-1);
U(n,k) =  1 + A(n-1,k-1)>>1 + A(n-1,k)>>1  + A(n,k-1)>>1;  A(n,k) is always odd if k>0
*/

/*-------------------------------------------------------------------*
 * a_three_fx()
 *-------------------------------------------------------------------*/
static
UWord32 a_three_fx(                     /* o:  offset for dim 3 */
    UWord32  k_val     /* i:  nb unit pulses   */
)                  /* k_val may be higher than  16 bit signed   */
{
    IF( k_val )
    {
        /* return (ONE_U + k_val*((k_val - ONE_U) << ONE)); */
        return UL_addNsD(1U,UL_Mpy_32_32(k_val,UL_lshl(UL_subNsD(k_val,1U),1)));
    }
    ELSE
    {
        return 0;
    }
}

/*-------------------------------------------------------------------*
 * a_four_fx()
 *-------------------------------------------------------------------*/
static
UWord32  a_four_fx(           /* o:  offset for dim 4 */
    UWord32  k_val   /* i:  nb unit pulses   */
)
{
    UWord32 UL_2k;
    IF(k_val)
    {
        /* return  UDIVBY3*((k_val<<ONE)*(4 + ((k_val<<ONE) - 3)*k_val)  -   3);  */
        UL_2k = UL_lshl(k_val,1);
        return  UL_Mpy_32_32(UDIVBY3_FX,
                             UL_subNsD(UL_Mpy_32_32(UL_2k,UL_addNsD(4U,UL_Mpy_32_32(UL_subNsD(UL_2k,3U),k_val)))
                                       ,3U)
                            );
    }
    return 0;
}

/*-------------------------------------------------------------------*
 * a_five_fx()
 *-------------------------------------------------------------------*/
static
UWord32 a_five_fx(                      /* o:  offset for dim 5 */
    UWord32 k_val          /* i:  nb unit pulses   */
)
{
    /*  k=uint64(256); offset = 1 +  2*idivide( (((k-2)*k + 5)*k -4) * k ,3) , log2(double(3*(double(offset)-1)/2)) */

    IF(k_val==0)
    {
        return 0;
    }
    ELSE IF( UL_subNsD(k_val,1U)==0)
    {
        return 1;
    }
    ELSE
    {
        /*UL_offset =  ONE + (((((k_val - TWO)*k_val + 5)*k_val - 4)*k_val)*(UDIVBY3_FX))<<ONE; */
        return UL_addNsD(1U,
        UL_lshl(
            UL_Mpy_32_32(UL_Mpy_32_32(UL_subNsD(UL_Mpy_32_32(UL_addNsD(UL_Mpy_32_32(UL_subNsD(k_val,2U),k_val),5U),k_val),4U),k_val),UDIVBY3_FX)
            ,1)
                        );   /* cost about ~13 cycles*/
    }
}

/*-------------------------------------------------------------------*
 * direct_msize_fx()
 * direct m_sizes  for N=1..5
 *-------------------------------------------------------------------*/
static
UWord32 direct_msize_fx(Word16 dim_in, Word16 k_in)
{
    UWord32  UL_msize, k, ksq;

    UL_msize = 1; /* k==0 or dim==1 , and base fot other dims */
    IF(k_in != 0)
    {
        k   = UL_deposit_l(k_in);   /*k = (UWord32) k_in;*/
        ksq = UL_Mpy_32_32(k,k);     /*ksq= k*k; */
        SWITCH (dim_in)
        {
        case (5):
            /* k*k = 238*238 < 2^16 ,  to remember for FIP*/
            /*UL_msize += ( ((ksq*(5  +  ksq))* UDIVBY3_FX )<<ONE ); */
            UL_msize=UL_addNsD( UL_msize ,UL_lshl(UL_Mpy_32_32(UL_Mpy_32_32(ksq,UL_addNsD(5U,ksq)),UDIVBY3_FX),1));
            BREAK;
        case(4 ):
            /*UL_msize  =   4*idivide(k*(k.^2+2),3);*/
            UL_msize = UL_lshl(UL_Mpy_32_32(UL_Mpy_32_32(k,UL_addNsD(ksq,2U)),UDIVBY3_FX),2);  /* ((k*(ksq + 2))*UDIVBY3_FX)  <<TWO; */
            BREAK;
        case ( 3 ):
            UL_msize =  UL_addNsD(UL_msize, UL_lshl(ksq,1)) ;  /* +=  ((ksq)<<ONE) ;*/
            BREAK;
        case ( 2 ):
            UL_msize =  UL_lshl(k,1); /* k<<ONE; */
            BREAK;
        }
    }

    return UL_msize;
}

/* update h_mem[0.. k_val_in+1] ,  with starting offsets for A+U recursion */
static
void initOffsets_fx( Word16 dim_in , UWord32* h_mem, Word16 k_val_in)
{
    UWord32 k_val_curr, k_val_prev;
    UWord32 k_val, UL_k_val_in;


    h_mem[0]          =  UL_deposit_l(0);                             /*  A(=>0,k=0)      */
    h_mem[1]          =  UL_deposit_l(1);                             /*  A(*,k=1)        */

    UL_k_val_in = UL_deposit_l(k_val_in);
    IF(sub(dim_in,2)==0)
    {
        FOR( k_val = 2 ; k_val <= UL_k_val_in ; k_val++ )
        {
            h_mem[k_val] = UL_subNsD(UL_lshl(k_val,1),1U);
            move32();   /* A(2, 2 .. k ) */
        }
        h_mem[k_val] =  UL_k_val_in;
        move32();                       /* U(2,k+1) */
    }
    ELSE
    {
        k_val_prev            =   UL_deposit_l(1U);
        FOR(k_val_curr = 2; k_val_curr<=UL_k_val_in; k_val_curr++)
        {
            h_mem[k_val_curr] = UL_addNsD(1U , UL_Mpy_32_32(k_val_curr, UL_lshl(k_val_prev,1)));
            move32();
            k_val_prev        = UL_addNsD(k_val_curr, 0U);       /* 1 op*/
        }
        h_mem[k_val_curr] = UL_Mpy_32_32(k_val_curr,k_val_prev);
        move32();  /* % U(3,k_val_in+1)  u_three(k+1) */
    }

    return;
}

/*-------------------------------------------------------------------*
 * a_fwd_fx()
 *
 *  create offsets for A(n,k)  from lower A(n-1,k)
 *-------------------------------------------------------------------*/
static
void a_fwd_fx(
    UWord32     *a_in,         /* i/o: offsets   */
    Word16      n_items         /* i  :  items, k's  */
)
{
    UWord32  a_1, a_in0 ;
    Word16   i ;
    UWord32* a_in_prev_ptr;

    a_in0   = UL_deposit_l(1);

    a_in_prev_ptr=&(a_in[-1]);        /* single loop ptr setup  not  counted; */
    FOR(i=1; i<=n_items; i++)         /* basic  A  fwd row  recursion */
    {
        a_1              = UL_addNsD(a_in0, UL_addNsD(a_in_prev_ptr[i], a_in[i])) ;
        a_in_prev_ptr[i] = a_in0;
        move32();
        a_in0            = UL_addNsD(a_1, 0U);
    }
    a_in_prev_ptr[i]  =  a_in0;
    move32();
    return;
}

/*-------------------------------------------------------------------*
 * a_bwd_fx()
 *
 *  create offsets for A(n,k)  from higher A(n+1,k)
 *-------------------------------------------------------------------*/
static
void a_bwd_fx(
    UWord32    *a_in,      /* i/o: offsets   */
    Word16     n_items     /* i:  n_items  */
)
{
    UWord32  a_1, a_in0;
    Word16  i;
    UWord32* a_in_prev_ptr;

    a_in0         = UL_deposit_l(0);
    a_in_prev_ptr = &(a_in[-1]);

    FOR(i = 1; i<=n_items; i++) /*basic A   reverse row  recursion */
    {
        /*  2x[i] ptr memory access below are treated as ptr access  */
        a_1              = UL_subNsD(UL_subNsD(a_in[i], a_in0), a_in_prev_ptr[i]);
        a_in_prev_ptr[i] = a_in0;
        move32();
        a_in0            = UL_addNsD(a_1, 0U);

    }
    a_in_prev_ptr[i]     = a_in0;
    move32();
    return;
}

static
UWord32 direct_row_A2U_rec_calc_fx(Word16 dim_in , Word16 k_val_in, UWord32 a_km2, UWord32 a_km1)
{
    /*  U(n,k) =  (A(n,k-2)-1)/2   +                ((2*n-1)*A(n,k-1) - A(n,k-2) )/2*(k-1)           */
    /*  U(n,k) = floor(A(n,k-2)/2) + (n*A(n,k-1) - floor(A(n,k-1)/2) - floor(A(n,k-2)/2) +1 )/(k-1)  */
    /*  U(n,k) = floor(A(n,k-2)/2) + (n*A(n,k-1) - (floor(A(n,k-1)/2) + floor(A(n,k-2)/2) +1) )/(k-1) */

    UWord32  km2_size, UL_um2, UL_dim ;
    Word16   divisor;

    divisor  = sub(k_val_in,1);
    UL_um2   = UL_lshr(a_km2,1U);
    UL_dim   = UL_deposit_l(dim_in);
    km2_size = UL_addNsD(UL_addNsD(UL_lshr(a_km1,1), UL_um2), 1U);
    IF(s_and(divisor,0x1) != 0)
    {
        /* odd */
        return  UL_addNsD(UL_um2, f_odd_exact_div_opt_fx( UL_dim, a_km1, km2_size, shr(divisor,1)));
    }
    ELSE
    {
        /* even divisor,   */
        return  UL_addNsD(UL_um2, f_even_exact_div_opt_fx(UL_dim, a_km1, km2_size,        divisor));
    }
}

static
void a_u_fwd_fx(UWord32 *a_u_in,
                Word16  k_val_in,
                Word16  mem_size_m1)
{
    UWord32 u_kp1_prev, u_kp1;
    UWord32 u_k_prev ;

    u_kp1_prev = a_u_in[mem_size_m1];
    move32(); /* previous  n  U (n,k+1) value*/
    u_k_prev   = UL_lshr(a_u_in[k_val_in],1);              /* previous  n  A(n,k) value*/

    a_fwd_fx(&a_u_in[1], k_val_in);           /* a_u_in[k==ZERO] = zero if n>0 */

    /*      low dynamic last offset entry mixed recursion */
    /*      used for size calculation  */
    /*      U(n,k+1) = 1 + U(n-1,k+1) + U(n-1,k)        + U(n,k)                            */
    /*      U(n,k+1) = 1 + U(n-1,k+1) + (A(n-1,k)-1)/2  + (A(n,k)-1)/2                      */
    /*                  Note, A(n,k) always odd for k>0 , subtracted one always shifted out */

    u_kp1               = UL_lshr(a_u_in[k_val_in],1);
    a_u_in[mem_size_m1] = UL_addNsD(1U, UL_addNsD(u_kp1_prev, UL_addNsD(u_k_prev,u_kp1)));
    return;
}

/*-------------------------------------------------------------------*
 * nm_h_prep_opt_fx()
 *
 * find  and return  N_MPVQ(n,k) and also offsets A(n, 0  to  k ) and  U(n,k+1).
 *-------------------------------------------------------------------*/
static
UWord32 nm_h_prep_opt_fx(      /* o:  msize for dim     */
    Word16 dim_in,       /* i:  dimension         */
    Word16 k_val_in,     /* i:  nb unit pulses    */
    UWord32 *h     /* o:  A/U offsets array */
)
{
    Word16 mem_size_m1, k_val,tmp ;
    Word16 dim_tmp, d_start;
    UWord32 h_saveA, h_saveB, u_kp1,a_k;   /*  registers   for alternating  A(n,k-1), A(n,k-2)*/
    UWord32 numDsub1;
    Word16 end_loop, add_last_odd ;

    h[0]          =  UL_deposit_l(0);                         /*    % A(=>0,k=0)      */
    h[1]          =  UL_deposit_l(1);                         /*    % A(*,k=1)        */

    mem_size_m1 = add(k_val_in,1);

    assert(dim_in > N_OPT_FX);  /* code now optimized with direct functions for dim <= N_OPT_FX ) */
    IF(  (sub(k_val_in, TABLE_LIM_OPT_FX) > 0)  )
    {
        d_start = 2;
        move16();
        if( sub(dim_in,3) >= 0 )
        {
            /* start from A(3),  U(3) */
            d_start = 3;
            move16();  /* single op */
        }
        initOffsets_fx(d_start, h, k_val_in);
        FOR(dim_tmp = d_start; dim_tmp < dim_in; dim_tmp++)
        {
            a_u_fwd_fx(h, k_val_in, mem_size_m1);
        }
        a_k   =  h[k_val_in];
        move32();
        u_kp1 =  h[mem_size_m1];
        move32();
    }
    ELSE
    {
        numDsub1 = UL_deposit_l(sub(shl(dim_in,1),1));
        h[2]     = numDsub1;
        move32();

        /* interleaved " odd, even [odd]"  divisor calls */
        h_saveA      =   numDsub1;
        move32();
        h_saveB      =   UL_deposit_l(1);

        /* OPT: makes sure that the STL  FOR loop is not broken */
        tmp            =  sub(k_val_in,3);
        add_last_odd   =  0;
        move16() ; /*k_val_in=0 1 2*/
        if(tmp == 0)
        {
            add_last_odd = 1;
            move16();  /*k_val_in =3 */
        }
        k_val          =  3;
        move16();
        IF(  tmp > 0 )
        {
            /* k_val_in = 3,4, 5,6, 7 ... */
            end_loop      =  mem_size_m1;
            move16();
            add_last_odd  =  s_and(k_val_in,0x1) ;
            move16();  /* odd -> 0x00100*/
            /* even loop limits,  and odd tail  exists , and   */
            if(add_last_odd != 0 )
            {
                end_loop = sub(end_loop,1);  /* make initial loop to even number of  (odd-even )pairs *//* one basicop */
            }
            FOR(k_val = 3; k_val < end_loop  ; k_val++ )
            {
                /* the optimized non broken loop k=(3,4)(5,6)...(odd,even)*/
                /* A(n,k)  =  A(n,k-2) + ((2*n-1)*A(n,k-1)-A(n,k-2)) /(k-1)  */
                /* first odd  k, even divisor */
                h_saveB   = UL_addNsD(h_saveB, f_even_exact_div_opt_fx(numDsub1, h_saveA, h_saveB, sub(k_val,1)));
                h[k_val]  = h_saveB;
                move32();

                /* next even k, odd divisor */
                /*k_val++; */
                h_saveA   = UL_addNsD(h_saveA, f_odd_exact_div_opt_fx(numDsub1, h_saveB, h_saveA,shr(k_val,1)));
                k_val++;   /* ptr incr */
                h[k_val]  = h_saveA;
                move32();
            }
        }

        IF( add_last_odd != 0 )
        {
            /* add a  last odd call as needed  , not to be called if k_val_in is [0,1,2]    */
            h_saveB     = UL_addNsD(h_saveB,f_even_exact_div_opt_fx(numDsub1, h_saveA, h_saveB, sub(k_val,1)));
            h[k_val_in] = h_saveB;
            move32();
        }

        /*  always do the last (k+1) recursion based  on  U(n,k+1) = func( A(n-2,k+1), A(n-1,k+1) )   */
        a_k            = h[k_val_in] ;
        move32();
        u_kp1          = direct_row_A2U_rec_calc_fx(dim_in, mem_size_m1 , h[sub(mem_size_m1,2)], a_k );
        h[mem_size_m1] = u_kp1;
        move32();
    }

    /*  N_MPVQ(n,k) = 1 + U(n,k+1) + U(n,k) =  1 + U(n,k+1) + floor(A(n,k))/2) ;   */ /* as A(n,k) always odd */
    return ( UL_addNsD(1U, UL_addNsD(u_kp1, UL_lshr(a_k,1))));
}

/*
   find_amp_split_offset_func_mem_fx()
   find first offset  in range 0..k_val_in  that is less than ind_in
   using a tree search with direct function calls [ or memory iteration]
*/
static
Word16 find_amp_split_offset_func_mem_fx(                      /* o:  found  k_value  */
    UWord32 ind_in,
    Word16  high,              /* i: k_val_in (high bound) */
    H_FUNCM h_func_ptr,        /* i: offset function pointer  */
    UWord32 *UL_tmp_offset)    /* o:  offset found  */
{
    Word16  not_ready, low, k_test=0;
    UWord16 sgn ;
    UWord32 UL_tmp;
    /* split over  A(n,k) = h_mem(k), or use direct A function evaluation */

    low  = 0;
    move16();
    move32();                                      /* account for adaptive  function ptr  setup */
    not_ready = 1 ;
    move16();

    WHILE( not_ready != 0)
    {
        k_test  = shr(add(low,high),1);             /*% split range in half */
        *UL_tmp_offset = (*h_func_ptr)(UL_deposit_l(k_test));   /* call direct A offset-function */

        UL_tmp = UL_subNs(*UL_tmp_offset, ind_in, &sgn );
        IF ( sgn )
        {
            /* (*tmp_offset < ind_in) */
            low =  add(1, k_test) ;
            if( sub(k_test ,high) >= 0)
            {
                not_ready = 0;
                move16(); /* single basicop */
            }
        }
        ELSE
        {
            /* (ind_in  <= *tmp_offset )  */
            high   = sub(k_test,1);
            if( UL_tmp == 0)
            {
                /* (*tmp_offset == ind_in)  */
                not_ready = 0;
                move16(); /* single basicop */
            }
        }
    }
    return k_test;
}

/*
  get_lead_sign_fx()
  updated index and return leading sign
*/
static
Word16 get_lead_sign_fx(UWord32 *ind)
{
    Word16 leading_sign;

    leading_sign = 1;
    move16();
    if( UL_and(*ind,1) != 0 )
    {
        /*  leading sign stored in LSB  */
        leading_sign =  -1;
        move16();
    }
    (*ind) = UL_lshr(*ind,1);

    return leading_sign;
}

/*-------------------------------------------------------------------*
 * mind2vec_one_fx()
 *-------------------------------------------------------------------*/
static
void mind2vec_one_fx(
    Word16 k_val_in,       /* i:  nb unit pulses       */
    Word16 leading_sign,   /* i: leading sign  -1, 0, 1*/
    UWord32 ind,           /* i:  index                */   /* parameter needed as it is used in a function array */
    Word16*  vec_out       /* o:  pulse train          */
)
{
    /*  NB input k_val_in can be zero   */
    /* *vec_out = leading_sign*k_val_in; */
    *vec_out = (Word16)ind;                /* dummy assignment to avoid gcc "unused parameter" warning for ind, i.e no move16() needed */

    /* *vec_out = extract_l(L_mult0(leading_sign,k_val_in));  move16();  // 3 ops */
    if( leading_sign < 0 )
    {
        k_val_in = negate(k_val_in);    /* single basicop -->  if */
    }
    *vec_out   = k_val_in;
    move16();  /* 1 op */
}

static
void mind2vec_two_fx(
    Word16 k_val_in,        /* i:  nb unit pulses   */
    Word16 leading_sign,    /* i: leading sign  -1,0, 1 */
    UWord32 ind_in,          /* i:  index            */
    Word16 *vec_out          /* o:  pulse train      */
)
{
    UWord32 UL_ind_tmp;
    Word16  val1;

    IF (ind_in == 0)
    {
        /* ind_in == 0 */
        mind2vec_one_fx( k_val_in,leading_sign,ind_in,vec_out);
    }
    ELSE IF ( sub((Word16)u_extract_l(ind_in), sub(shl(k_val_in,1),1)) == 0 )
    {
        /* signed ops fine as 2*KMAX  <<  32767) */
        /* (ind_in == ( (unsigned int)(k_val_in<<ONE) - ONE_U) ) */
        mind2vec_one_fx( k_val_in,leading_sign,ind_in,&(vec_out[1]) );
    }
    ELSE
    {
        UL_ind_tmp    =  UL_subNsD(ind_in,1U);
        val1          =  (Word16)u_extract_l(UL_addNsD(1U,UL_lshr(UL_ind_tmp,1))) ;  /*(Word16) to avoid warning */

        mind2vec_one_fx( sub(k_val_in, val1), leading_sign, ind_in, vec_out);

        if(UL_and(UL_ind_tmp,1) != 0 )
        {
            val1 =   negate(val1) ;     /*single basicop */
        }
        vec_out[1]  =   val1;
        move16();
    }
}

static
Word16  setval_update_sign_fx(Word16  k_delta,
                              Word16  k_max_local,
                              Word16  *leading_sign,
                              UWord32 *ind_in,
                              Word16*  vec_out
                             )
{
    IF(k_delta != 0 )
    {
        mind2vec_one_fx( k_delta, *leading_sign, *ind_in, vec_out);
        *leading_sign  =  get_lead_sign_fx(ind_in);
        k_max_local    = sub( k_max_local ,k_delta);
    }
    return k_max_local;
}

/*-------------------------------------------------------------------*
 * mind2vec_three_fx()
 *-------------------------------------------------------------------*/
static
void mind2vec_three_fx(
    Word16  k_max_local,     /* i:  nb unit pulses   */
    Word16  leading_sign,    /* i: leading sign */
    UWord32 ind_in,          /* i:  index            */
    Word16* vec_out          /* o:  pulse train      */
)
{
    /*
       use direct calculation of first amplitude
       (to find amplitudes faster than using split or linear iteration)
    */
    Word16 k_delta ;
    Word16  acc_val;

    IF( ind_in != 0 )
    {
        /* acc_val=idivide(uint32(floor(real(sqrt(double(ind)*2-1))))+1, 2);  %   (exact_integer_sqrt((ind*2-1) +1)*2  */
        acc_val     = lshr(add(1,getSqrtWord32(UL_subNsD(UL_lshl(ind_in,1),1U))),1);
        k_delta     = sub(k_max_local, acc_val);
        ind_in      = UL_subNsD(ind_in, a_three_fx(UL_deposit_l(acc_val)));    /* remove amplitude offset A(3,k_acc) */

        k_max_local = setval_update_sign_fx(k_delta, k_max_local, &leading_sign, &ind_in, vec_out);

        mind2vec_two_fx( k_max_local, leading_sign, ind_in ,&vec_out[1] );
    }
    ELSE
    {
        /* vec_out[0]= leading_sign*k_max_local; */
        mind2vec_one_fx( k_max_local,leading_sign,ind_in,vec_out);
    }
    return;
}

/*-------------------------------------------------------------------*
 * mind2vec_direct_fx ,
    general function for direct decoding using direct funstions
    (no  memory recursion)
 *-------------------------------------------------------------------*/
static
void mind2vec_direct_fx(
    Word16      k_max_local ,     /* i:  nb unit pulses          */
    Word16      leading_sign,     /* i: leading sign             */
    UWord32     ind,              /* i:  index                   */
    H_FUNCM     h_func_ptr,       /* i : offset function         */
    NDIM_FUNCM  nd_func_ptr ,     /* i : next dimension function  */
    Word16*     vec_out           /* o:  pulse train              */
)

{
    Word16    k_delta, k_test ;
    UWord32   UL_tmp_offset;

    IF( ind != 0 )
    {
        k_test  = find_amp_split_offset_func_mem_fx(ind,k_max_local, h_func_ptr, &UL_tmp_offset);
        k_delta =  sub(k_max_local,k_test);
        ind     =  UL_subNsD(ind,UL_tmp_offset);    /* %  remove amplitude offset A(n,k_acc) */

        k_max_local = setval_update_sign_fx(k_delta, k_max_local, &leading_sign, &ind, vec_out);

        move32();      /* account for adaptive function ptr  setup */
        (*nd_func_ptr)( k_max_local, leading_sign, ind , &vec_out[1] );  /* next lower dimension */
    }
    ELSE
    {
        mind2vec_one_fx( k_max_local,leading_sign,ind,vec_out);
    }
    return;
}

/*-------------------------------------------------------------------*
 * mind2vec_four_fx()
 *-------------------------------------------------------------------*/
static
void mind2vec_four_fx(
    Word16       k_val_in,       /* i:  nb unit pulses   */
    Word16       leading_sign,   /* i: leading sign */
    UWord32      ind_in ,        /* i:  index            */
    Word16*      vec_out         /* o:  pulse train      */
)
{
    mind2vec_direct_fx(k_val_in,leading_sign, ind_in, a_four_fx, mind2vec_three_fx, vec_out);
    return;
}

/*-------------------------------------------------------------------*
 * mind2vec_five_fx()
 *-------------------------------------------------------------------*/
static
void mind2vec_five_fx(
    Word16  k_val_in ,       /* i:  nb unit pulses   */
    Word16  leading_sign,    /* i: leading sign      */
    UWord32 ind_in,          /* i:  index            */
    Word16* vec_out          /* o:  pulse train      */
)
{
    mind2vec_direct_fx(k_val_in,leading_sign, ind_in, a_five_fx, mind2vec_four_fx, vec_out);
    return;
}

/*-------------------------------------------------------------------*
 * mind2vec_fx()
 *-------------------------------------------------------------------*/
static
void mind2vec_fx(
    Word16   dim_in,           /* i:  dimension        */
    Word16   k_max_local,      /* i:  nb unit pulses   */
    Word16   leading_sign,     /* i: leading sign  */
    UWord32  ind,              /* i:  index            */
    Word16*  vec_out,          /* o:  pulse train      */
    UWord32* h_in              /* i:  offset vector   A=1+2U  */
)
{
    Word16 pos, k_acc, k_delta;
    UWord32 UL_tmp_offset, UL_diff;
    UWord16 sgn;

    k_acc = k_max_local;
    move16();
    FOR( pos = 0; pos < dim_in; pos++ )
    {
        /* first to last position decoding */

        IF(ind != 0)
        {
            /* regular linear magnitude search */
            k_acc         = k_max_local;
            move16();            /* ptr init */

            UL_tmp_offset = UL_addNsD(h_in[k_acc], 0U);      /* memory load init */

            UL_diff       = UL_subNs(ind, UL_tmp_offset ,&sgn);

            WHILE( sgn  /*(ind - UL_tmp_offset)<0*/  )      /* WHILE costs,  4 cycles every iter */
            {
                UL_diff = UL_subNs(ind, h_in[--k_acc],&sgn);      /* one cycle*/
            }

            ind = UL_addNsD(UL_diff, 0U);    /* save amplitude index offset A(n, k_acc) */

            k_delta   = sub(k_max_local,k_acc);  /* amplitude decoding */
        }
        ELSE
        {
            mind2vec_one_fx( k_max_local, leading_sign,ind,&vec_out[pos]);
            BREAK ;  /* "fast"  recursion exit*/
        }

        k_max_local = setval_update_sign_fx(k_delta, k_max_local, &leading_sign, &ind, &vec_out[pos]);

        /* move from  A(n,kmax) to A(n-1, k_max_local), */
        a_bwd_fx( h_in, add(k_max_local,1)  );  /* [0 ... k_max_local], no need to update U(n,k_max_local+1) in index decoding   */
    }

    return;
}

/*-------------------------------------------------------------------*
 * get_size_mpvq_calc_offset_fx()
 *
 *  unsigned int h_mem[1 + KMAX +1 ];
 *   example using fixed size of offset vector input help variable
 *-------------------------------------------------------------------*/
PvqEntry_fx get_size_mpvq_calc_offset_fx( /* o : size, dim, k_val        */
    Word16  dim_in,                  /* i : dimension                */
    Word16 k_val_in,                 /* i : nb unit pulses           */
    UWord32* h_mem                   /* o : offsets                  */
)
{
    PvqEntry_fx entry;

    entry.dim   = dim_in;
    move16();
    entry.k_val = k_val_in;
    move16();

    entry.index         = L_deposit_l(0);
    entry.lead_sign_ind = 0;
    move16();

    IF(sub(dim_in, N_OPT_FX) > 0 )      /* non-direct solutions,  use A+U relation */
    {
        entry.size = nm_h_prep_opt_fx(entry.dim, entry.k_val, h_mem);
    }
    ELSE
    {
        entry.size =  direct_msize_fx(dim_in, entry.k_val);
    }

    return entry;
}

/*-------------------------------------------------------------------*
 * mpvq_decode_vec_fx()
 *-------------------------------------------------------------------*/
void mpvq_decode_vec_fx(        /* o :  void                        */
    const PvqEntry_fx* entry,  /* i :  sign_ind, index, dim, k_val */
    UWord32* h_mem,            /* i :  A/U offsets                 */
    Word16* vec_out            /* o :  pulse train                 */
)
{
    Word16 i, leading_sign;
    IND2VECFUNCM  mind2vec_f_fx[N_OPT_FX+1] = { (IND2VECFUNCM)NULL, mind2vec_one_fx, mind2vec_two_fx, mind2vec_three_fx, mind2vec_four_fx, mind2vec_five_fx };

    FOR(i=0; i<entry->dim; i++)
    {
        vec_out[i]=0;
        move16(); /* set all of short vector to zero , required for fast/early  exit logic  */
    }

    leading_sign    =  1;
    move16();
    if(entry->lead_sign_ind != 0)
    {
        leading_sign = -1;
        move16();
    }

    IF(entry->k_val != 0)
    {
        IF(sub(entry->dim,N_OPT_FX)> 0 )  /* N_OPT_FX  */
        {
            /*  generic */
            mind2vec_fx(entry->dim, entry->k_val, leading_sign, entry->index, vec_out, h_mem);
        }
        ELSE
        {
            /*  specialized functions,  with direct offset calculations */
            (mind2vec_f_fx[entry->dim])(entry->k_val, leading_sign, entry->index, vec_out);
        }
    }

    return;
}

/*-------------------------------------------------------------------*
* vec2mind_one_fx()
*-------------------------------------------------------------------*/
static
void vec2mind_one_fx(
    const Word16* vec_in,               /* i : PVQ  pulse train        */
    Word16 *k_val_out_ptr ,             /* o : number of unit pulses    */ /* parameter needed as it is used in a function array */
    UWord32 *next_sign_ind,             /* i/o: next sign ind           */
    UWord32* ind                        /* o: MPVQ index                */
)
{
    *ind   = (Word32)(*k_val_out_ptr) ; /* dummy assignment to avoid gcc "unused parameter" warning for *k_val_out_ptr, i.e no move32 needed() */
    *ind   = UL_deposit_l(0);

    *next_sign_ind = UL_deposit_l(0);
    if( *vec_in < 0 )
    {
        *next_sign_ind = UL_deposit_l(1); /*single basicop */
    }
    return ;
}

/*-------------------------------------------------------------------*
* vec2mind_two_fx()
*-------------------------------------------------------------------*/
static
void vec2mind_two_fx(
    const Word16* vec_in,               /* i : PVQ  pulse train        */
    Word16 *k_val_out_ptr ,             /* o : number of unit pulses    */
    UWord32 *next_sign_ind,             /* i/o: next sign ind           */
    UWord32* ind                        /* o: MPVQ index                */
)
{
    UWord32 lead_sign_ind_add;
    Word16 abs0,abs1,abs01,sptr;

    abs0           = abs_s(vec_in[0]);
    abs1           = abs_s(vec_in[1]);
    abs01          = add(abs0, abs1);
    *k_val_out_ptr = abs01;
    move16(); /* can be zero */
    *ind           = UL_deposit_l(0);             /* [KMAX 0 ] , and dual zeros */

    *next_sign_ind = UL_deposit_h(SIGNBIT_SHRT_FX); /*  "unset"  sign flag set */ /* dual zeroes  can happen in a recursive encoding call  */


    IF( abs01 != 0 )
    {
        sptr           =  0;
        move16();  /*used as ptr to vec0 or vec1 value  */
        *next_sign_ind = UL_deposit_l(sptr);

        test();
        IF(abs0 != 0 && abs1 != 0 )
        {
            /* likely most frequent/common case */
            /* [ KMAX-1 1],[ KMAX-1 -1] ...  [ 1  +(KMAX-1) ],[ 1 -(KMAX-1)] */
            /* sign always shifted to first pos */
            lead_sign_ind_add = UL_deposit_l(1) ;
            if( vec_in[1]  < 0)
            {
                lead_sign_ind_add = UL_deposit_l(2);  /* single op */
            }
            *ind =   UL_addNsD(UL_deposit_l((UWord16)lshl(sub(abs1,1),1)),lead_sign_ind_add);
        }
        ELSE
        {
            /* one value is a zero */
            IF( abs0 == 0 )
            {
                /* [ 0 KMAX]*/
                *ind            =  UL_deposit_l((UWord16) sub(lshl(abs1,1),1));
                sptr            =  1;
                move16();
            }
        }

        /* *next_sign_ind= (unsigned int)(vec_in[*next_sign_ind]<0); */
        if(  vec_in[sptr] < 0  )
        {
            *next_sign_ind  = UL_deposit_l(1);    /*single instruction */
        }
    }
    return;
}

static
void enc_push_sign(Word16 val, UWord32* next_sign_ind, UWord32 *index)
{
    /*
    %   Check if the  leading sign 'bit' is to be added
    %   here the leading sign bit is put in LSB as it saves about 3 cycles in sign-decoding
    %   (one can also put it in the MSB , but then one needs to access h_mem twice and shift the sign bit into position  )
    */
    test();
    IF( (UL_and(*next_sign_ind,SIGNBIT_FX) == 0)   &&   (val != 0))
    {
        *index   = UL_addNsD(UL_lshl(*index,1),*next_sign_ind);
    }

    /* push  sign to next non_zero position  */
    /* *next_sign_ind = *next_sign_ind ;*/  /* default is to keep stored sign index */
    if( val < 0 )
    {
        *next_sign_ind = UL_deposit_l(1);  /* single basicop */
    }
    if( val > 0)
    {
        *next_sign_ind = UL_deposit_l(0);  /* single basicop */
    }
}

/*-------------------------------------------------------------------*
 *  vec2mind_gen345_fx( vec_in kval,   next_dim_func , offset_func,....)
 *   generic call saves  PROM ,
 *-------------------------------------------------------------------*/

static
void vec2mind_gen345_fx(
    const Word16* vec_in,               /* i : PVQ abs pulse train          */
    Word16 *k_val_out_ptr,              /* o : number of unit pulses    */
    UWord32  *next_sign_ind ,           /* i/o: next sign ind */
    UWord32* index ,                    /* o: MPVQ index */
    VEC2INDFUNCM vec2indfunc_ptr,       /* i: */
    H_FUNCM    a_func_ptr               /*i:  offset function  */
)
{
    Word16  tmp_val;

    tmp_val = vec_in[0];
    move16();
    move32();                    /* adaptive function call setup */
    (*vec2indfunc_ptr)(&vec_in[1], k_val_out_ptr, next_sign_ind, index);

    enc_push_sign(tmp_val, next_sign_ind, index);

    move32();        /* adaptive  function call setup */
    *index   =   UL_addNsD(*index,(*a_func_ptr)(UL_deposit_l(*k_val_out_ptr)));

    *k_val_out_ptr = add(*k_val_out_ptr, abs_s(tmp_val));

    return ;
}

/*-------------------------------------------------------------------*
 * vec2mind_three_fx()
 *-------------------------------------------------------------------*/
static
void vec2mind_three_fx(
    const Word16* vec_in,             /* i : PVQ   pulse train          */
    Word16* k_val_out_ptr,            /* o : number of unit pulses    */
    UWord32 *next_sign_ind,           /* i/o: next sign ind */
    UWord32 *index                    /* o: MPVQ index */
)
{
    vec2mind_gen345_fx(vec_in,k_val_out_ptr, next_sign_ind, index, vec2mind_two_fx, a_three_fx);
    return ;
}


/*-------------------------------------------------------------------*
 * vec2mind_four_fx()
 *-------------------------------------------------------------------*/
static
void vec2mind_four_fx(
    const Word16* vec_in,                /* i : PVQ  pulse train          */
    Word16*        k_val_out_ptr,        /* o : number of unit pulses    */
    UWord32*       next_sign_ind ,        /* i/o: next sign ind */
    UWord32*       index                  /* o: MPVQ index */
)
{
    vec2mind_gen345_fx(vec_in, k_val_out_ptr, next_sign_ind, index, vec2mind_three_fx, a_four_fx);
    return ;
}

/*-------------------------------------------------------------------*
 * vec2mind_five_fx()
 *-------------------------------------------------------------------*/
static
void vec2mind_five_fx(
    const Word16* vec_in,               /* i : PVQ abs pulse train          */
    Word16 *k_val_out_ptr,              /* o : number of unit pulses    */
    UWord32 *next_sign_ind ,            /* i/o: next sign ind */
    UWord32* index                      /* o: MPVQ index */
)
{
    vec2mind_gen345_fx(vec_in,k_val_out_ptr, next_sign_ind, index, vec2mind_four_fx, a_five_fx);
    return ;
}

/*-------------------------------------------------------------------*
* vec2mind_fx()
*-------------------------------------------------------------------*/
static
void vec2mind_fx(Word16 dim_in,                /* i :  dim                       */
                 Word16 k_val_in,              /* i :  number of unit pulses     */
                 const Word16* vec_in,         /* i :  PVQ pulse train           */
                 UWord32 *next_sign_ind,       /* o :  pushed leading sign       */
                 UWord32  *index  ,            /* o :  MPVQ index                */
                 UWord32  *N_MPVQ_ptr,         /* o :  size(N_MPVQ(dim,K_val_in))*/
                 UWord32* h_mem)               /* o :  offsets                   */
{
    Word16  pos,  mem_size_m1, k_val_acc, tmp_val;
    UWord32 tmp_h;

    /*
    %%   main steps
    % quick encode two rightmost pos
    % for every position  from  dim-3 to 0  (right to left)
    %   check if an sign is to be encoded ,
    %     add its offset
    %  check(and add) amplitude offset(for accumulated pulse sum) up to this point
    %  update total pulse sum
    %  update offset vector recursively (except for pos==0 )
    % end
    % calculate size
    */

    mem_size_m1    = add(k_val_in,1);
    *next_sign_ind = UL_deposit_h(SIGNBIT_SHRT_FX);   /* highest bit set signals no sign found yet, should always be 0 or 1 out, */

    pos            =  sub(dim_in,2);               /*  adress 2nd last sample */
    vec2mind_two_fx(&vec_in[pos],&k_val_acc,next_sign_ind ,index);
    initOffsets_fx( 3, h_mem, k_val_in) ;         /*  start recursions at 3rd sample */

    tmp_h          = h_mem[k_val_acc];
    move32();
    FOR (pos--; pos>=0; pos--)
    {
        tmp_val = vec_in[pos];
        move16();
        enc_push_sign(tmp_val, next_sign_ind, index);

        /* now add  indexing offset up to this reverse (r_l) accumulated  amplitude point */
        *index  =  UL_addNsD(*index, tmp_h);            /* k_val_acc==0 ==>0 */

        /* k_val_acc = k_val_acc + vec_abs[pos];*/   /* now increase acc k value for next N  */
        k_val_acc = add(k_val_acc, abs_s(tmp_val));

        IF( pos != 0 )
        {
            a_u_fwd_fx(h_mem, k_val_in ,mem_size_m1);
            /* update A(n,k=1:k_val_in) and U(n,k_val_in+1) */
            /* NB here  (k_val_in + 2 elements always has to be updated */
        }
        tmp_h = UL_addNsD(h_mem[k_val_acc], 0U);
    }

    /*  size is needed for the subseqent arithmetic encoding/transmission of the index.
        use relation N_MPVQ(n,K) =  1 +  (A(n, K)-1)/2 + U(n, 1 + K)
        =            N_MPVQ(n,K) =  1 +  (A(n, K)>>1)  + U(n, 1 + K) ,  as A(n,K) is odd)   */
    *N_MPVQ_ptr   =   UL_addNsD(1U,UL_addNsD(UL_lshr(tmp_h,1),h_mem[ mem_size_m1]));
    move32();/* calc total size  */

    return;
}

/*--------------------------------------------------------------------------*
 * mpvq_encode_vec_fx()
 *
 * returns struct with lead sign index, MPVQ-index, dim and  N_MPVQ size
 *-------------------------------------------------------------------------*/

PvqEntry_fx mpvq_encode_vec_fx(                          /* o : leading_sign_index, index, size, k_val        */
    const Word16* vec_in,        /* i : signed pulse train        */
    Word16        dim_in,        /* i : dimension                 */
    Word16        k_val_local    /* i : nb unit pulses            */
)
{
    PvqEntry_fx result;
    UWord32 h_mem[ 1 + KMAX_NON_DIRECT_FX + 1 ] ;   /* now always assign max offset buffer for dim 6 ,
                                                       actually only 1+k_val_in+1 needed ) */
    UWord32 lead_sign_ind;

    VEC2INDFUNCM  vec2mind_f[1+N_OPT_FX] = { (VEC2INDFUNCM)NULL, vec2mind_one_fx, vec2mind_two_fx, vec2mind_three_fx, vec2mind_four_fx, vec2mind_five_fx };

    result.k_val =  k_val_local;
    move16();
    result.dim   =  dim_in;
    move16();
    /* NB , k_val_local  may be changed in some sub encoding routines */
    IF( sub(dim_in, N_OPT_FX) > 0 )
    {
        /* use the generic dimension  function */
        vec2mind_fx(dim_in, k_val_local, vec_in, &lead_sign_ind,  &result.index, &result.size,  h_mem);
    }
    ELSE /* if (dim_in<=N_OPT), h_mem not used */
    {
        move32();          /* adaptive function ptr setup */
        (vec2mind_f[dim_in])(vec_in, &k_val_local, &lead_sign_ind, &result.index);
        result.size             =  direct_msize_fx(dim_in, k_val_local);
    }
    result.lead_sign_ind = u_extract_l(lead_sign_ind);

    return result;
}


