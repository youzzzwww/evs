/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*===========================================================================*/
/* FUNCTION : syn_12k8_fx()													 */
/*---------------------------------------------------------------------------*/
/* PURPOSE : perform the synthesis filtering 1/A(z).						 */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :														 */
/* _ (Word16) st_fx->L_frame :length of the frame			                 */
/* _ (Word16[]) Aq		: LP filter coefficients                       Q12   */
/* _ (Word16) exc		: input signal  						       Q_exc */
/* _ (Word16) update_m		: update memory flag: 0-->no memory update       */
/*												  1 --> update of memory     */
/* _ (Word16) Q_exc		: Excitation scaling                             	 */
/* _ (Word16) Q_syn		: Synthesis scaling                             	 */
/*---------------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS :													 */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													     */
/* _ (Word16[]) synth		: initial filter states                    Q_syn */
/*---------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->mem_syn2_fx: initial filter states              Q_syn */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														 */
/* _ None																	 */
/*===========================================================================*/
void syn_12k8_fx(
    Word16 L_frame,
    const Word16 *Aq,       /* i  : LP filter coefficients                       Q12   */
    const Word16 *exc,     /* i  : input signal                                 Q_exc */
    Word16 *synth,    /* o  : output signal                                Q_syn */
    Word16 *mem,      /* i/o: initial filter states                        Q_syn */
    const Word16 update_m,  /* i  : update memory flag: 0 --> no memory update   Q0    */
    /*                          1 --> update of memory         */
    const Word16 Q_exc,     /* i  : Excitation scaling                           Q0    */
    const Word16 Q_syn      /* i  : Synthesis scaling                            Q0    */
)
{
    const Word16 *p_Aq;
    Word16 i_subfr;
    Word16 shift;

    shift = sub(Q_exc, Q_syn);
    p_Aq = Aq;
    move16();
    FOR (i_subfr = 0; i_subfr < L_frame; i_subfr += L_SUBFR)
    {
        Syn_filt_s(shift, p_Aq, M, &exc[i_subfr], &synth[i_subfr], L_SUBFR, mem, update_m);
        p_Aq  += (M+1);
        move16();   /* interpolated LPC parameters for next subframe */
    }
    return;
}
