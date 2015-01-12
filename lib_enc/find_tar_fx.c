/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "cnst_fx.h"       /* Common constants                       */
#include "basop_util.h"
#include "stl.h"



void find_targets_fx(
    const Word16 *speech,     /* i  : pointer to the speech frame                      Q_new-1*/
    const Word16 *mem_syn,    /* i  : memory of the synthesis filter                   Q_new-1*/
    const Word16 i_subfr,     /* i  : subframe index                                   */
    Word16 *mem_w0,     /* i/o: weighting filter denominator memory              Q_new-1*/
    const Word16 *p_Aq,       /* i  : interpolated quantized A(z) filter               Q12*/
    const Word16 *res,        /* i  : residual signal                                  Q_new*/
    const Word16 L_subfr,     /* i  : length of vectors for gain quantization          */
    const Word16 *Ap,         /* i  : unquantized A(z) filter with bandwidth expansion Q12*/
    Word16 tilt_fac,    /* i  : tilt factor                                  Q15 */
    Word16 *xn,         /* o  : Close-loop Pitch search target vector            Q_new-1*/
    Word16 *cn          /* o  : target vector in residual domain                 Q_new*/
    ,Word16 *h1
)
{
    Word16 i;
    Word16 temp[M+5*L_SUBFR];       /* error of quantization */
    Word16 scale,scaleq,j,d,s,s2,tmp;
    Word16 Aqs[M+1];
    Word32 Ltmp;
    /*------------------------------------------------------------------------*
     * Find the target vector for excitation search:
     *
     *             |------|  res[n]
     * speech[n]---| A(z) |--------
     *             |------|       |   |--------| error[n]  |------|
     *                   zero -- (-)--| 1/A(z) |-----------| W(z) |-- target
     *                   exc          |--------|           |------|
     *
     * Instead of subtracting the zero-input response of filters from
     * the weighted input speech, the above configuration is used to
     * compute the target vector.
     *-----------------------------------------------------------------------*/
    FOR (i=0; i<M; i++)
    {
        temp[i] = sub(speech[i+i_subfr-M], mem_syn[i]);
        move16();
    }
    Syn_filt_s(1, p_Aq, M, &res[i_subfr], temp+M, L_subfr, temp, 0); /* error in Q_new -1 */

    Residu3_fx(Ap, temp+M, xn, L_subfr, 0);                           /* xn in Q_new -1*/

    deemph_fx(xn, tilt_fac, L_subfr, mem_w0);                         /* xn in Q_new -1 */



    /*-----------------------------------------------------------------*
     * Find target in residual domain (cn[]) for innovation search
     *--------------------------------------------------------------*/
    IF( cn != NULL )
    {
        /* first half: xn[] --> cn[] */
        temp[0] = 0;
        move16();
        preemph_copy_fx(xn, cn, tilt_fac, L_SUBFR/2, temp);
        syn_filt_s_lc_fx(1, Ap, cn, temp, L_SUBFR/2);                   /* Q-1 -> Q-2 */
        Residu3_lc_fx(p_Aq, M, temp, cn, L_SUBFR/2, 1);                 /* Q-2 -> Q-1 */
        Scale_sig(cn, L_SUBFR/2,1);

        /* second half: res[] --> cn[] (approximated and faster) */
        Copy(&res[i_subfr+(L_SUBFR/2)], cn+(L_SUBFR/2), L_SUBFR/2);
    }

    /*---------------------------------------------------------------*
    * Compute impulse response, h1[], of weighted synthesis filter  *
    *---------------------------------------------------------------*/

    scale = norm_s( Ap[0] );
    scaleq = norm_s( p_Aq[0] );
    d = sub( scaleq, scale );
    IF ( d >= 0 )
    {
        Copy( p_Aq, Aqs, M+1 );
        s = add( scaleq, 1 );
        s2 = shr( 16384, d );
    }
    ELSE
    {
        Copy_Scale_sig( p_Aq, Aqs, M+1, d );
        s = add( scale, 1 );
        s2 = 16384;
    }
    Overflow  = 0;
    move16();
    FOR (i = 0; i < M; i++)
    {
        Ltmp = L_mult(Ap[i], s2);
        FOR (j = 1; j <= i; j++)
        {
            Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
        }
        h1[i] = round_fx(L_shl(Ltmp, s));
    }
    Ltmp = L_mult(Ap[i], s2);
    FOR (j = 1; j <= M; j++)
    {
        Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
    }
    h1[M] = round_fx(L_shl(Ltmp, s));
    FOR (i=M+1; i < L_subfr; i++)
    {
        Ltmp = L_msu(0, Aqs[1], h1[i-1]);
        FOR (j = 2; j <= M; j++)
        {
            Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
        }
        h1[i] = round_fx(L_shl(Ltmp, s));
    }
    IF(Overflow)
    {
        s = sub(s,1);
        FOR (i = 0; i < M; i++)
        {
            Ltmp = L_mult(Ap[i], s2);
            FOR (j = 1; j <= i; j++)
            {
                Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
            }
            h1[i] = round_fx(L_shl(Ltmp, s));
        }
        Ltmp = L_mult(Ap[i], s2);
        FOR (j = 1; j <= M; j++)
        {
            Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
        }
        h1[M] = round_fx(L_shl(Ltmp, s));
        FOR (i=M+1; i < L_subfr; i++)
        {
            Ltmp = L_msu(0, Aqs[1], h1[i-1]);
            FOR (j = 2; j <= M; j++)
            {
                Ltmp = L_msu(Ltmp, Aqs[j], h1[i-j]);
            }
            h1[i] = round_fx(L_shl(Ltmp, s));
        }
    }

    tmp = 0;
    Deemph2(h1, tilt_fac, L_subfr, &tmp);

    return;

}
