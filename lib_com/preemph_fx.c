/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------*
 * preemph_copy_fx()
 *
 * Preemphasis: filtering through 1 - mu z^-1
 *-------------------------------------------------------------*/
void preemph_copy_fx(
    const Word16 x[],   /* i  : input signal             Qx  */
    Word16 y[],   /* o  : output signal            Qx  */
    const Word16 mu,    /* i  : preemphasis coefficient  Q15 */
    const Word16 lg,    /* i  : vector size              Q0  */
    Word16 *mem   /* i/o: memory (x[-1])           Qx  */
)
{
    Word16 i, temp;

    temp = x[lg - 1];
    move16();
    FOR (i = sub(lg, 1); i > 0; i--)
    {
        y[i] = msu_r(L_deposit_h(x[i]), x[i - 1], mu);
        move16();
    }
    y[0] = msu_r(L_deposit_h(x[0]), *mem, mu);
    move16();

    *mem = temp;
    move16();
}

/*
 * E_UTIL_f_preemph2
 *
 * Parameters:
 *    shift          I: scale output
 *    signal       I/O: signal				Qx/Qx+shift
 *    mu             I: preemphasis factor	Q15
 *    L              I: vector size
 *    mem          I/O: memory (x[-1])
 *
 * Function:
 *    Filtering through 1 - mu z^-1
 *
 * Returns:
 *    void
 */
void E_UTIL_f_preemph2(Word16 shift, Word16 *signal, const Word16 mu, const Word16 lg, Word16 *mem)
{
    Word16 i, temp;
    Word32 L_tmp;

    temp = signal[lg - 1];
    move16();

    FOR (i = sub(lg, 1); i > 0; i--)
    {
        L_tmp = L_mult(signal[i], 16384);
        L_tmp = L_msu0(L_tmp, signal[i - 1], mu);
        L_tmp = L_shl(L_tmp, add(shift,1));
        signal[i] = round_fx(L_tmp);
    }

    L_tmp = L_mult(signal[0], 16384);
    L_tmp = L_msu0(L_tmp, *mem, mu);
    L_tmp = L_shl(L_tmp, add(shift,1));
    signal[0] = round_fx(L_tmp);

    *mem = temp;
    move16();

    return;
}


Word16 E_UTIL_f_preemph3(Word16 *signal, const Word16 mu, const Word16 lg, Word16 *mem, Word16 bits)
{
    Word16 i, QVal, mus, tmp_fixed, Q_new;
    Word32 L_tmp, L_maxloc;



    QVal = shl(1, sub(15,bits));
    mus = shr(mu, bits);

    L_tmp = L_mult(signal[0], QVal);
    L_tmp = L_msu(L_tmp, *mem, mus);
    L_maxloc = L_abs(L_tmp);

    FOR (i = 1; i < lg; i++)
    {
        L_tmp = L_mult(signal[i], QVal);
        L_tmp = L_msu(L_tmp, signal[i - 1], mus);
        L_tmp = L_abs(L_tmp);
        L_maxloc = L_max(L_tmp, L_maxloc);
    }

    tmp_fixed = extract_h(L_maxloc);

    Q_new = Q_MAX;
    move16();
    IF (tmp_fixed != 0)
    {
        Q_new = sub(norm_s(tmp_fixed), bits);
        Q_new = s_max(Q_new, 0);
        Q_new = s_min(Q_new, Q_MAX);
    }

    tmp_fixed = signal[lg - 1];
    move16();

    FOR (i = sub(lg,1); i > 0; i--)
    {
        L_tmp = L_mult(signal[i], QVal);
        L_tmp = L_msu(L_tmp, signal[i - 1], mus);
        L_tmp = L_shl(L_tmp, Q_new);
        signal[i] = round_fx(L_tmp);
    }

    L_tmp = L_mult(signal[0], QVal);
    L_tmp = L_msu(L_tmp, *mem, mus);
    L_tmp = L_shl(L_tmp, Q_new);
    signal[0] = round_fx(L_tmp);

    *mem = tmp_fixed;
    move16();

    return Q_new;
}

