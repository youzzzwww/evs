/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <math.h>
#include <assert.h>
#include "stl.h"
#include "basop_util.h"
#include "cnst_fx.h"
#include "prot_fx.h"


void tfaCalcEnv_fx(const Word16* shb_speech, Word32* enr)
{
    Word16 i, j, k;

    k = 0;
    move16();
    FOR (i=0; i<N_TEC_TFA_SUBFR; i++)
    {
        enr[i] = L_deposit_l(0);
        FOR (j=0; j<L_TEC_TFA_SUBFR16k; j++)
        {
            enr[i] = L_mac0(enr[i], shb_speech[k], shb_speech[k]);
            k = add(k, 1);
        }
    }
}

Word16 tfaEnc_TBE_fx(Word32* enr,
                     Word16 last_core,
                     Word16* voicing,   /* Q15 */
                     Word16* pitch_buf, /* Q6 */
                     Word16 Q_enr
                    )
{
    Word16 i;
    Word32 m_g, m_a; /* m_g: geometrical mean, m_a: arithmetical mean */
    Word16 voicing_sum;
    Word16 pitch_buf_sum;
    Word32 m_a_bottom;
    Word16 tfa_flag;

    Word32 L_tmp, L_tmp1;
    Word16 exp;

    m_a_bottom = L_shl(625, Q_enr); /*  10000.0 / N_TEC_TFA_SUBFR in Q_enr */

    tfa_flag = 0;
    move16();

    L_tmp = L_deposit_l(0);
    m_a = L_deposit_l(0);
    m_g = L_deposit_l(0);

    FOR (i = 0; i < N_TEC_TFA_SUBFR; i++)
    {
        IF(enr[i] != 0)
        {
            m_a = L_add(m_a, L_shr(enr[i], 4)); /* Q_enr */
            exp = norm_l(enr[i]);
            L_tmp = BASOP_Util_Log2(L_shl(enr[i], exp));
            exp = sub(sub(31, exp), Q_enr);
            L_tmp = L_add(L_shl(L_deposit_h(exp), 9), L_tmp);
            m_g = L_add(m_g, L_shr(L_tmp, 4));
        }
    }

    /* energy lower limit */
    IF(L_sub(m_a, m_a_bottom) < 0)
    {
        tfa_flag = 0;
        move16();
    }
    ELSE
    {
        exp = norm_l(m_a);
        L_tmp = BASOP_Util_Log2(L_shl(m_a, exp));
        exp = sub(sub(31, exp), Q_enr);
        m_a = L_add(L_shl(L_deposit_h(exp), 9), L_tmp);/* Q25 */
        L_tmp = L_add(m_a, FL2WORD32_SCALE(log10(0.7f)/log10(2.f), 31-25));
        L_tmp1 = L_add(m_a, FL2WORD32_SCALE(log10(0.5f)/log10(2.f), 31-25));

        voicing_sum = add(shr(voicing[0], 1), shr(voicing[1], 1));

        pitch_buf_sum = shr(add(shr(pitch_buf[0], 1), shr(pitch_buf[1], 1)), 1);
        pitch_buf_sum = add(pitch_buf_sum, shr(add(shr(pitch_buf[2], 1), shr(pitch_buf[3], 1)), 1));

        test();
        test();
        test();
        test();
        test();
        IF ((L_sub(m_g, L_tmp) > 0 && sub(pitch_buf_sum, FL2WORD16_SCALE(110, 15-6)) > 0 && sub(voicing_sum, FL2WORD16(0.70)) > 0) ||
        (sub(last_core, TCX_20_CORE) == 0 && L_sub(m_g, L_tmp1) > 0 && sub(voicing_sum, FL2WORD16(0.70)) < 0))
        {
            tfa_flag = 1;
            move16();
        }
    }

    return tfa_flag;
}

