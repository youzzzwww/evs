/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"      /* Debug prototypes                       */
#include "stl.h"        /* required for wmc_tool */

/*-------------------------------------------------------------------*
 * syn_output()
 *
 * Output synthesis signal with compensation for saturation
 *-------------------------------------------------------------------*/

void syn_output_fx(
    const Word16 codec_mode,   /* i  : MODE1 or MODE2                       */
    Word16 *synth,       /* i/o: float synthesis signal               */
    const Word16 output_frame, /* i  : output frame length                  */
    Word16 *synth_out,   /* o  : integer 16 bits synthesis signal     */
    const Word16 Q_syn2        /* i  : Synthesis scaling factor             */
)
{
    Word16 i, tmp;
    Word32 L_tmp;

    /*tmp = sub(Q_syn2, 1); */
    tmp = Q_syn2;

    /*-----------------------------------------------------------------*
     * Output synthesis signal with compensation for saturation
     *-----------------------------------------------------------------*/

    test();
    IF( codec_mode == MODE2 || sub(output_frame,L_FRAME8k) == 0 )
    {
        /* integer conversion */
        /*mvr2s( synth, synth_out, output_frame );   */
        FOR (i = 0; i < output_frame; i++)
        {
            L_tmp = L_deposit_h(synth[i]);
            synth_out[i] = round_fx(L_shr(L_tmp, tmp));
        }
    }
    ELSE
    {
        Copy_Scale_sig( synth, synth_out, output_frame, negate(tmp) );
    }

    return;
}
/*-------------------------------------------------------------------*
 * Local function
 * unscale_AGC
 *
 * Output synthesis signal with compensation for saturation
 *-------------------------------------------------------------------*/
void unscale_AGC(
    const Word16 x[],    /* i:   16kHz synthesis                 Qx */
    const Word16 Qx,     /* i:   scale factor of x                  */
    Word16       y[],    /* o:   output vector                   Q0 */
    Word16       mem[],  /* i/o: mem[2] should be init to [0,0]     */
    const Word16 n       /* i:   vector size                        */
)
{
    Word16 i, fac, tmp, frame_fac, max;
    Word32 L_tmp;

    /*----------------------------------------------------------------*
     * calculate AGC factor to avoid saturation
     *----------------------------------------------------------------*/

    max = abs_s(x[0]);
    FOR (i = 1; i < n; i++)
    {
        max = s_max(max, abs_s(x[i]));
    }
    BASOP_SATURATE_WARNING_OFF
    tmp = shl(30000, Qx); /* saturation can occurs here */
    BASOP_SATURATE_WARNING_ON
    frame_fac = 0;
    move16();
    IF (sub(max, tmp) > 0)
    {
        frame_fac = sub(16384, div_s(shr(tmp, 1), max)); /* frame fac in Q15 */
    }

    /*----------------------------------------------------------------*
     * AGC
     *----------------------------------------------------------------*/
    /* update AGC factor (slowly) */
    fac = mac_r(L_mult(32440, mem[0]), 328, frame_fac);

    L_tmp = L_mult(x[0], 16384);
    L_tmp = L_msu0(L_tmp, fac, x[0]);
    L_tmp = L_msu(L_tmp, fac, mem[1]);
    L_tmp = L_shr(L_tmp, -1);    /* saturation can occur here */

    y[0] = round_fx(L_tmp);

    FOR (i = 1; i < n; i++)
    {
        /* update AGC factor (slowly) */
        fac = mac_r(L_mult(32440, fac), 328, frame_fac);

        L_tmp = L_deposit_h(x[i]);
        L_tmp = L_msu(L_tmp, fac, x[i]);
        L_tmp = L_msu(L_tmp, fac, x[i-1]);
        y[i] = round_fx(L_tmp);
    }

    mem[0] = fac;
    move16();
    mem[1] = shr(x[sub(i, 1)], 1);
    move16();
}
