/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"       /* Function prototypes                    */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*--------------------------------------------------------------------*
 * Residu3_lc_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *
 * Optimized Version: Use when Past[0..m-1] is 0 & a[0] is 1 (in Q12)
 *--------------------------------------------------------------------*/
void Residu3_lc_fx(
    const Word16 a[],   /* i  :   prediction coefficients                 Q12 */
    const Word16 m,     /* i  :   order of LP filter                      Q0  */
    const Word16 x[],   /* i  :   input signal (usually speech)           Qx  */
    Word16 y[],   /* o  :   output signal (usually residual)        Qx  */
    const Word16 lg,    /* i  :   vector size                             Q0  */
    const Word16 shift
)
{
    Word16 i, j;
    Word32 s;
    Word16 q;

    q = add( norm_s(a[0]), 1 );
    if (shift > 0)
        q = add(q, shift);
    *y++ = shl(x[0], shift);
    move16();

    FOR (i = 1; i < m; i++)
    {
        s = L_mult(x[i], a[0]);
        /* Stop at i to Avoid Mults with Zeros */
        FOR (j = 1; j <= i; j++)
        {
            s = L_mac(s, x[i-j], a[j]);
        }

        s = L_shl(s, q);
        *y++ = round_fx(s);
    }

    FOR (; i < lg; i++)
    {
        s = L_mult(x[i], a[0]);
        FOR (j = 1; j <= m; j++)
        {
            s = L_mac(s, x[i-j], a[j]);
        }

        s = L_shl(s, q);
        *y++ = round_fx(s);
    }
}

/*--------------------------------------------------------------------*
 * Residu3_10_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *--------------------------------------------------------------------*/
void Residu3_10_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-10..-1] are needed)    */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i :  vector size                             Q0  */
    const Word16 shift
)
{
    Word16 i;
    Word32 s;
    Word16 q;

    q = add( norm_s(a[0]), 1 );
    if (shift != 0)
        q = add(q, shift);
    FOR (i = 0; i < lg; i++)
    {
        s = L_mult(x[i], a[0]);
        s = L_mac(s, x[i-1], a[1]);
        s = L_mac(s, x[i-2], a[2]);
        s = L_mac(s, x[i-3], a[3]);
        s = L_mac(s, x[i-4], a[4]);
        s = L_mac(s, x[i-5], a[5]);
        s = L_mac(s, x[i-6], a[6]);
        s = L_mac(s, x[i-7], a[7]);
        s = L_mac(s, x[i-8], a[8]);
        s = L_mac(s, x[i-9], a[9]);
        s = L_mac(s, x[i-10], a[10]);

        s = L_shl(s, q);
        y[i] = round_fx(s);
    }
}

/*--------------------------------------------------------------------*
 * Residu3_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *--------------------------------------------------------------------*/
void Residu3_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-M..-1] are needed)     */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i :  vector size                             Q0  */
    const Word16 shift
)
{
    Word16 i;
    Word32 s;
    Word16 q;

    q = add( norm_s(a[0]), 1 );
    if (shift != 0)
        q = add(q, shift);
    FOR (i = 0; i < lg; i++)
    {
        s = L_mult(x[i], a[0]);
        s = L_mac(s, x[i-1], a[1]);
        s = L_mac(s, x[i-2], a[2]);
        s = L_mac(s, x[i-3], a[3]);
        s = L_mac(s, x[i-4], a[4]);
        s = L_mac(s, x[i-5], a[5]);
        s = L_mac(s, x[i-6], a[6]);
        s = L_mac(s, x[i-7], a[7]);
        s = L_mac(s, x[i-8], a[8]);
        s = L_mac(s, x[i-9], a[9]);
        s = L_mac(s, x[i-10], a[10]);
        s = L_mac(s, x[i-11], a[11]);
        s = L_mac(s, x[i-12], a[12]);
        s = L_mac(s, x[i-13], a[13]);
        s = L_mac(s, x[i-14], a[14]);
        s = L_mac(s, x[i-15], a[15]);
        s = L_mac(s, x[i-16], a[16]);

        s = L_shl(s, q);
        y[i] = round_fx(s);
    }
}

/*==========================================================================*/
/* FUNCTION      : 	void calc_residu()									    */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : Compute the LP residual by filtering the input through	*/
/*													A(z) in all subframes	*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/* Word16 *speech          i  : weighted speech signal                  Qx  */
/* Word16 L_frame          i  : order of LP filter						Q0  */
/* Word16 *p_Aq            i  : quantized LP filter coefficients        Q12 */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* Word16 *res             o  : residual signal                        Qx+1 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 															*/
/*==========================================================================*/

void calc_residu_fx(
    Encoder_State_fx *st,          /* i/o: state structure                           */
    const Word16 *speech,      /* i  : weighted speech signal                    */
    Word16 *res,         /* o  : residual signal                           */
    const Word16 *p_Aq,        /* i  : quantized LP filter coefficients          */
    const Word16 vad_hover_flag
)
{
    Word16 i_subfr;
    Word16 i;
    Word16 att;
    Word16 offset;

    FOR( i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR )
    {
        /* calculate the residual signal */
        Residu3_fx( p_Aq, &speech[i_subfr], &res[i_subfr], L_SUBFR, 1 );

        /* next subframe */
        p_Aq += (M+1);
    }
    /* smoothing in case of CNG */
    test();
    IF( (st->Opt_DTX_ON_fx != 0 ) && (vad_hover_flag != 0) )   /* corresponds to line 504 in FLT acelp_core_enc.c */
    {
        st->burst_ho_cnt_fx = add(st->burst_ho_cnt_fx,1);

        st->burst_ho_cnt_fx = s_min(st->burst_ho_cnt_fx, HO_HIST_SIZE);

        IF( sub(st->bwidth_fx, NB) != 0)
        {
            offset = 5;
            test();
            if( sub(st->bwidth_fx, WB) == 0 && st->CNG_mode_fx >= 0 )
            {
                offset = st->CNG_mode_fx;
                move16();
            }

            att = CNG_burst_att_fx[offset][sub(st->burst_ho_cnt_fx,1)];  /*Q15*/

            FOR( i = 0; i < st->L_frame_fx; i++ )
            {
                res[i] = mult_r(res[i], att);
                move16();
            }
        }
    }
    ELSE
    {
        st->burst_ho_cnt_fx = 0;
        move16();
    }

    return;
}

