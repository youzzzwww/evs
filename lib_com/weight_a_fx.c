/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"       /* Function prototypes                    */
#include "prot_fx.h"    /* Function prototypes                    */
#include <assert.h>

/*------------------------------------------------------------------
 * weight_a_subfr()
 *
 * Weighting of LP filter coefficients for multiple subframes,
 * ap[i] = a[i] * (gamma^i)
 *------------------------------------------------------------------*/

void weight_a_subfr_fx(
    const Word16 nb_subfr,   /* i  : number of subframes                 */
    const Word16 *A,         /* i  : LP filter coefficients          Q12 */
    Word16 *Aw,        /* o  : weighted LP filter coefficients Q12 */
    const Word16 gamma,      /* i  : weighting factor                    */
    const Word16 order       /* i  : order of LP filter                  */
)
{
    Word16 k, orderp1;

    /* Smoothing aka spreading aka masking envelope generation */
    orderp1 = add(order,1);
    FOR (k=0; k<nb_subfr; k++)
    {
        weight_a_fx(&A[k*(orderp1)], &Aw[k*(orderp1)], gamma, order);
    }

    return;
}

/*==============================================================================*/
/* FUNCTION      :  void weight_a_lc_fx ( )										*/
/*------------------------------------------------------------------------------*/
/* PURPOSE       : Weighting of LP filter coefficients, ap[i] = a[i] * (gamma^i)*/
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :															*/
/*		const Word16 a[],			   i:  LP filter coefficients           Q12 */
/*		const Word16 *gammatbl,		   i:  weighting factor                 Q15 */
/*		const Word16 m				   i:  order of LP filter               Q0  */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :															*/
/*		Word16 ap[],				   o:  weighted LP filter coefficients  Q12 */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : 															*/
/*------------------------------------------------------------------------------*/
/* CALLED FROM : TX/RX															*/
/*==============================================================================*/
void weight_a_lc_fx(
    const Word16 a[],         /* i:  LP filter coefficients           Q12 */
    Word16 ap[],        /* o:  weighted LP filter coefficients  Q12 */
    const Word16 *gammatbl,   /* i:  weighting factor                 Q15 */
    const Word16 m            /* i:  order of LP filter               Q0  */
)
{
    Word16 i;
    Word32 Amax;
    Word16 shift;
    const Word16 *ptr_gamma;

    ptr_gamma = gammatbl;
    Amax = L_mult( 16384, a[0] );
    FOR (i = 1; i < m; i++)
    {
        Amax = L_max( Amax, L_abs( L_mult0( *ptr_gamma++, a[i] ) ) );
    }
    Amax = L_max( Amax, L_abs( L_mult0( *ptr_gamma++, a[m] ) ) );
    shift = norm_l( Amax );
    ptr_gamma = gammatbl;
    ap[0] = shl( a[0], sub(shift,1) );
    move16();
    FOR (i = 1; i < m; i++)
    {
        ap[i] = round_fx(L_shl(L_mult0(a[i], *ptr_gamma++),shift));
    }
    ap[m] = round_fx(L_shl(L_mult0(a[m], *ptr_gamma++),shift));

    return;
}

/*------------------------------------------------------------------
 * weight_a:
 *
 * Weighting of LP filter coefficients, ap[i] = a[i] * (gamma^i)
 *------------------------------------------------------------------*/
void weight_a_fx(
    const Word16 a[],         /* i:  LP filter coefficients           Q12 */
    Word16 ap[],        /* o:  weighted LP filter coefficients  Q12 */
    const Word16 gamma,       /* i:  weighting factor                 Q15 */
    const Word16 m            /* i:  order of LP filter               Q0  */
)
{
    Word16 i, fac;
    Word32 Amax;
    Word16 shift;

    fac = gamma;
    move16();
    Amax = L_mult( 16384, a[0] );
    FOR (i = 1; i < m; i++)
    {
        Amax = L_max( Amax, L_abs( L_mult0( fac, a[i] ) ) );
        fac = mult_r( fac, gamma );
    }
    Amax = L_max( Amax, L_abs( L_mult0( fac, a[m] ) ) );
    shift = norm_l( Amax );
    fac = gamma;
    move16();
    ap[0] = shl( a[0], sub(shift,1) );
    move16();
    FOR (i = 1; i < m; i++)
    {
        ap[i] = round_fx(L_shl(L_mult0(a[i], fac),shift));
        fac = mult_r( fac, gamma );
    }
    ap[m] = round_fx(L_shl(L_mult0(a[m], fac),shift));

    return;
}

/*
 * E_LPC_a_weight_inv
 *
 * Parameters:
 *    a              I: LP filter coefficients			Q12
 *    ap             O: weighted LP filter coefficients Q12
 *    inv_gamma      I: inverse weighting factor				Q14
 *    m              I: order of LP filter
 *
 * Function:
 *    Weighting of LP filter coefficients, ap[i] = a[i] * (inv_gamma^i).
 *
 * Returns:
 *    void
 */
void E_LPC_a_weight_inv(const Word16 *a, Word16 *ap, const Word16 inv_gamma, const Word16 m)
{
    Word16 i;
    static const Word16 inv_gamma_tab_12k8[16] = { 17809, 19357, 21041, 22870, 24859, 27020, 29370, 31924,   /* Q14 */
                                                   17350, 18859, 20499, 22281, 24219, 26325, 28614, 31102
                                                 }; /* Q13 */
    static const Word16 inv_gamma_tab_16k[16]  = { 17430, 18542, 19726, 20985, 22324, 23749, 25265, 26878,   /* Q14 */
                                                   14297, 15209, 16180, 17213, 18312, 19480, 20724, 22047
                                                 }; /* Q13 */
    const Word16 *inv_gamma_tab;
    Word32 L_tmp;
    Word32 Amax;
    Word16 shift;



    IF (inv_gamma == 16384)
    {
        FOR (i = 0; i <= m; i++)
        {
            ap[i] = a[i];
            move16();
        }
        return;
    }

    assert( inv_gamma==GAMMA1_INV || inv_gamma==GAMMA16k_INV );
    assert( m==16 );

    inv_gamma_tab = inv_gamma_tab_12k8;
    move16();
    if (sub(inv_gamma,GAMMA16k_INV) == 0)
    {
        inv_gamma_tab = inv_gamma_tab_16k;
        move16();
    }


    Amax = L_mult( 16384, a[0] );
    FOR (i = 1; i < 9; i++)
    {
        Amax = L_max( Amax, L_abs( L_mult( a[i], inv_gamma_tab[i-1] ) ) );
    }
    FOR (i = 9; i < 17; i++)
    {
        Amax = L_max( Amax, L_abs( L_shl( L_mult( a[i], inv_gamma_tab[i-1] ), 1 ) ) );
    }
    shift = norm_l( Amax );
    ap[0] = shl( a[0], sub(shift,1) );
    move16();
    FOR (i = 1; i < 9; i++)
    {
        L_tmp = L_mult( a[i], inv_gamma_tab[i-1] );
        ap[i] = round_fx( L_shl( L_tmp, shift ) );
    }
    shift = add(shift,1);
    FOR (i = 9; i < 17; i++)
    {
        L_tmp = L_mult( a[i], inv_gamma_tab[i-1] );
        ap[i] = round_fx( L_shl( L_tmp, shift ) );
    }



    return;
}
