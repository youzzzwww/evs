/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"

Word32 get_gain(     /* output: codebook gain (adaptive or fixed)   Q16 */
    Word16 x[],        /* input : target signal                        */
    Word16 y[],        /* input : filtered codebook excitation         */
    Word16 n           /* input : segment length                       */
)
{
    Word32 tcorr, tener, Lgain;
    Word16 exp_c, exp_e, exp, tmp;


    tcorr = L_deposit_l(0);
    tener = L_deposit_l(0);



    /*----------------------------------------------------------------*
     * Find gain based on inter-correlation product
     *----------------------------------------------------------------*/

    tcorr = Dot_product16HQ( 0, x, y,  n, &exp_c );
    tener = Dot_productSq16HQ( 0, y, n, &exp_e );

    BASOP_Util_Divide_MantExp(round_fx(tcorr), exp_c, s_max(round_fx(tener),1), exp_e, &tmp,&exp);
    Lgain = L_shl(L_deposit_l(tmp)/*Q15*/,add(1,exp))/*Q16*/;

    return Lgain;
}

Word32 get_gain2(     /* output: codebook gain (adaptive or fixed)   Q16 */
    Word16 x[],        /* input : target signal                        */
    Word16 y[],        /* input : filtered codebook excitation         */
    Word16 n           /* input : segment length                       */
)
{
    Word32 tcorr, tener, Lgain;
    Word16 m_corr, m_ener, negative, Q_corr, Q_ener;

    negative = 0;
    move16();

    /*----------------------------------------------------------------*
     * Find gain based on inter-correlation product
     *----------------------------------------------------------------*/
    tcorr = Dot_product16HQ(0, x, y, n, &Q_corr);
    tener = Dot_productSq16HQ(0, y, n, &Q_ener);

    tener = L_max(tener, 1);

    if (tcorr <= 0)
    {
        negative = 1;
        move16();
    }
    BASOP_SATURATE_WARNING_OFF /*tcorr max be negative maxvall -  not critical*/
    tcorr = L_abs(tcorr);
    BASOP_SATURATE_WARNING_ON

    m_corr = extract_h(tcorr);

    m_ener = extract_h(tener);

    IF (sub(m_corr, m_ener) > 0)
    {
        m_corr = shr(m_corr, 1);
        Q_corr = add(Q_corr,1);
    }
    if (m_ener==0)
    {
        move16();
        m_corr = 0x7FFF;
    }
    if (m_ener != 0)
    {
        m_corr = div_s(m_corr, m_ener);
    }

    Q_corr = sub(Q_corr,Q_ener);

    Lgain = L_shl(L_deposit_l(m_corr), add(Q_corr, 1)); /* Lgain in Q16 */

    if (negative != 0)
    {
        Lgain = L_negate(Lgain);           /* Lgain in Q16 */
    }


    return Lgain;
}
