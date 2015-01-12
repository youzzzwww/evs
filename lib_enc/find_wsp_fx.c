/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <memory.h>
#include <assert.h>
#include "stl.h"
#include "prot_fx.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"



/*
 * find_wsp
 *
 * Parameters:
 *    Az          I:   A(z) filter coefficients		          Q12
 *    speech      I:   pointer to the denoised speech frame   Q_new - preemph_bits
 *    wsp         O:   pointer to the weighted speech frame   Q_new - preemph_bits
 *	  mem_wsp	  I/O: W(z) denominator memory
 *    preemph_fac I:   pre-emphasis factor                    Q15
 *    L_frame	  I:   length of the frame
 *    lookahead   I:   length of a look-ahead
 *    L_subfr     I:   length of the sub-frame
 *
 * Function:
 *    Find weighted speech (formula from AMR-WB)
 *
 * Returns:
 *    void
 */
void find_wsp(
    const Word16 Az[],
    const Word16 speech[],
    Word16 wsp[],
    Word16 *mem_wsp,
    const Word16 preemph_fac,
    const Word16 L_frame,
    const Word16 lookahead,
    const Word16 L_subfr,
    Word16 *Aw,         /* o  : weighted A(z) filter coefficients     */
    const Word16 gamma,        /* i  : weighting factor                      */
    const Word16 nb_subfr    /* i  : number of subframes                   */
)
{
    Word16 i_subfr, wtmp;
    const Word16 *p_Az;
    /*-----------------------------------------------------------------*
     *  Compute weighted A(z) unquantized for subframes
     *-----------------------------------------------------------------*/

    weight_a_subfr_fx( nb_subfr, Az, Aw, gamma, M );


    /*----------------------------------------------------------------*
    *  Compute weighted speech for all subframes
    *----------------------------------------------------------------*/
    BASOP_SATURATE_WARNING_OFF
    p_Az = Aw;																	/*move16();*/
    FOR (i_subfr = 0; i_subfr < L_frame; i_subfr += L_subfr)
    {
        Residu3_fx(p_Az, &speech[i_subfr], &wsp[i_subfr], L_subfr, 0);
        p_Az += (M+1);
    }
    p_Az -= (M+1);
    BASOP_SATURATE_WARNING_ON
    /*----------------------------------------------------------------*
     *  Weighted speech computation is extended on look-ahead
     *----------------------------------------------------------------*/
    deemph_fx(wsp, preemph_fac, L_frame, mem_wsp); /* use Deemph2 to prevent saturation */

    IF ( lookahead != 0 )
    {
        Residu3_fx(p_Az, &speech[L_frame], &wsp[L_frame], lookahead, 0);
        wtmp = *mem_wsp;
        move16();
        deemph_fx(&wsp[L_frame], preemph_fac, lookahead, &wtmp);
    }

}



