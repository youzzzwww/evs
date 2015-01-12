/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "basop_util.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define kLagWinThGain1 19661 /* 0.6f in Q15 */
#define kLagWinThGain2 9830  /* 0.3f in Q15 */

/*-------------------------------------------------------------*
 * procedure lag_wind:                                         *
 *           ~~~~~~~~~                                         *
 * lag windowing of the autocorrelations                       *
 *-------------------------------------------------------------*/

void lag_wind(
    Word16 r_h[],           /* in/out: autocorrelations                                       */
    Word16 r_l[],           /* in/out: autocorrelations                                       */
    Word16 m,               /* input : order of LP filter                                     */
    Word32 sr,              /* input : sampling rate                                          */
    Word16 strength         /* input : LAGW_WEAK, LAGW_MEDIUM, or LAGW_STRONG                 */
)
{
    Word16 i;
    Word32 tmp;
    const Word16 *wnd_h, *wnd_l;


    assert(0 <= strength && strength <= NUM_LAGW_STRENGTHS);
    SWITCH (sr)
    {
    case 8000:
        assert(m <= 16);
        assert(strength == LAGW_STRONG);
        wnd_h = lag_window_8k[0];
        wnd_l = lag_window_8k[1];
        BREAK;
    case 12800:
        assert(m <= 16);
        wnd_h = lag_window_12k8[strength][0];
        wnd_l = lag_window_12k8[strength][1];
        BREAK;
    case 16000:
        assert(m <= 16);
        wnd_h = lag_window_16k[strength][0];
        wnd_l = lag_window_16k[strength][1];
        BREAK;
    case 24000:
    case 25600:
        assert(m <= 16);
        wnd_h = lag_window_25k6[strength][0];
        wnd_l = lag_window_25k6[strength][1];
        BREAK;
    case 32000:
        assert(m <= 16);
        wnd_h = lag_window_32k[strength][0];
        wnd_l = lag_window_32k[strength][1];
        BREAK;
    case 48000:
        assert(m <= 16);
        assert(strength == LAGW_STRONG);
        wnd_h = lag_window_48k[0];
        wnd_l = lag_window_48k[1];
        BREAK;
    default:
        assert(!"Lag window not implemented for this sampling rate");
        return;
    }

    FOR (i = 1; i <= m; i++)
    {
        tmp = Mpy_32(r_h[i], r_l[i], wnd_h[i-1], wnd_l[i-1]);
        L_Extract(tmp, &r_h[i], &r_l[i]);
    }

}

void adapt_lag_wind(
    Word16 r_h[],           /* in/out: autocorrelations                                       */
    Word16 r_l[],           /* in/out: autocorrelations                                       */
    Word16 m,               /* input : order of LP filter                                     */
    const Word16 Top,       /* input : open loop pitch lag                                    */
    const Word16 Tnc,       /* input : open loop pitch gain                                   */
    Word32 sr               /* input : sampling rate                                          */
)
{
    Word16 strength, pitch_lag;
    Word16 pitch_gain;

    pitch_lag = Top;
    move16();
    pitch_gain = Tnc;
    move16();

    IF (sub(pitch_lag, 80) < 0)
    {
        strength = LAGW_STRONG;
        move16();
        if (sub(pitch_gain, kLagWinThGain1) <= 0)
        {
            strength = LAGW_MEDIUM;
            move16();
        }
    }
    ELSE IF (sub(pitch_lag, 160) < 0)
    {
        strength = LAGW_MEDIUM;
        move16();
        if (sub(pitch_gain, kLagWinThGain2) <= 0)
        {
            strength = LAGW_WEAK;
            move16();
        }
    }
    ELSE
    {
        strength = LAGW_WEAK;
        move16();
    }

    lag_wind(r_h, r_l, m, sr, strength);
}
