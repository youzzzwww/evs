/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "stl.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"

static Word32 dot(const Word16 *X, const Word16 *Y, Word16 n)
{
    Word32 acc;
    Word16 i;


    acc = L_deposit_l(0);

    FOR (i = 0; i < n; i++)
    {
        acc = L_mac0(acc, X[i], Y[i]);
    }


    return acc;
}

static Word32 interpolate_corr(         /* o  : interpolated value   */
    const Word32 *x,      /* i  : input vector         */
    const Word16 frac,    /* i  : fraction of lag      */
    const Word16 frac_max /* i  : max fraction         */
)
{
    Word32 s;
    const Word16 *c, *win;


    win = E_ROM_inter4_1;
    if (sub(frac_max, 6) == 0) win = E_ROM_inter6_1;


    s = L_deposit_l(0);

    c = &win[frac];
    s = L_add(s, Mpy_32_16_1(x[0],  *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[-1], *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[-2], *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[-3], *c));

    c = &win[frac_max-frac];
    s = L_add(s, Mpy_32_16_1(x[1], *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[2], *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[3], *c));
    c += frac_max;
    s = L_add(s, Mpy_32_16_1(x[4], *c));


    return s;
}

void tcx_ltp_pitch_search(
    Word16 pitch_ol,
    Word16 *pitch_int,
    Word16 *pitch_fr,
    Word16 *index,
    Word16 *norm_corr,
    const Word16 len,
    Word16 *wsp,
    Word16 pitmin,
    Word16 pitfr1,
    Word16 pitfr2,
    Word16 pitmax,
    Word16 pitres
)
{
    Word16 i, t, t0, t1, step, fraction, t0_min, t0_max, t_min, t_max, delta, temp_m, temp_e, s, s_wsp;
    Word32 cor_max, cor[256], *pt_cor, temp;
    Word16 wsp2[L_FRAME_PLUS+PIT_MAX_MAX+L_INTERPOL1];



    delta = 16;
    move16();
    if ( sub(pitres, 6) == 0 )
    {
        delta = 8;
        move16();
    }

    t0_min = sub(pitch_ol, shr(delta, 1));
    t0_max = sub(add(t0_min, delta), 1);

    IF ( sub(t0_min, pitmin) < 0 )
    {
        t0_min = pitmin;
        move16();
        t0_max = sub(add(t0_min, delta), 1);
    }

    IF ( sub(t0_max, pitmax) > 0 )
    {
        t0_max = pitmax;
        move16();
        t0_min = add(sub(t0_max, delta), 1);
    }

    t_min = sub(t0_min, L_INTERPOL1);
    t_max = add(t0_max, L_INTERPOL1);

    /* normalize wsp */
    assert(len+t_max <= L_FRAME_PLUS+PIT_MAX_MAX+L_INTERPOL1);
    s_wsp = getScaleFactor16(wsp - t_max, add(len, t_max));
    s_wsp = sub(s_wsp, 4);
    FOR (t = negate(t_max); t < len; t++)
    {
        wsp2[t+t_max] = shl(wsp[t], s_wsp);
        move16();
    }
    wsp = wsp2 + t_max;

    pt_cor = cor;

    FOR ( t=t_min; t<=t_max; t++ )
    {
        *pt_cor = dot(wsp, wsp-t, len);
        pt_cor++;
    }

    pt_cor = cor + L_INTERPOL1;
    cor_max = L_add(*pt_cor++, 0);
    t1 = t0_min;
    move16();

    FOR ( t = add(t0_min, 1); t <= t0_max; t++ )
    {
        IF ( *pt_cor > cor_max  )
        {
            cor_max = *pt_cor;
            t1 = t;
        }
        pt_cor++;
    }

    temp = dot(wsp, wsp, len);
    s = norm_l(temp);
    temp_m = extract_h(L_shl(temp, s));
    temp_e = negate(s);

    temp = dot(wsp-t1, wsp-t1, len);
    s = norm_l(temp);
    temp_m = mult(temp_m, extract_h(L_shl(temp, s)));
    temp_e = sub(temp_e, s);

    temp_m = Sqrt16(temp_m, &temp_e);

    if (temp_m == 0)
    {
        temp_m = 1;
        move16();
    }
    s = sub(norm_l(cor_max), 1);
    temp_m = divide1616(extract_h(L_shl(cor_max, s)), temp_m);
    temp_e = sub(negate(s), temp_e);

    BASOP_SATURATE_WARNING_OFF
    *norm_corr = shl(temp_m, temp_e);
    BASOP_SATURATE_WARNING_ON

    IF ( sub(t1, pitfr1) >= 0 )
    {
        *pitch_int = t1;
        move16();
        *pitch_fr = 0;
        move16();

        *index = add(sub(t1, pitfr1), extract_l(L_mac0(L_mult0(sub(pitfr2, pitmin), pitres),
                                                sub(pitfr1, pitfr2), shr(pitres,1))));
        move16();

        return;
    }

    /*------------------------------------------------------------------*
     * Search fractional pitch with 1/4 subsample resolution.
     * search the fractions around t0 and choose the one which maximizes
     * the interpolated normalized correlation.
     *-----------------------------------------------------------------*/

    pt_cor = cor + sub(L_INTERPOL1, t0_min);
    t0 = t1;
    move16();

    step = 1;
    move16();
    if (sub(t0, pitfr2) >= 0)
    {
        step = 2;
        move16();
    }
    fraction = step;
    move16();

    IF (sub(t0, t0_min) == 0)        /* Limit case */
    {
        fraction = 0;
        move16();
        cor_max = interpolate_corr( &pt_cor[t0], fraction, pitres );
    }
    ELSE                     /* Process negative fractions */
    {
        t0 = sub(t0, 1);
        cor_max = interpolate_corr( &pt_cor[t0], fraction, pitres );

        FOR ( i = add(fraction, step); i < pitres; i += step )
        {
            temp = interpolate_corr( &pt_cor[t0], i, pitres );

            IF (L_sub(temp, cor_max) > 0)
            {
                cor_max = L_add(temp, 0);
                fraction = i;
                move16();
            }
        }
    }

    i = 0;
    FOR ( i = 0; i < pitres; i += step )     /* Process positive fractions */
    {
        temp = interpolate_corr( &pt_cor[t1], i, pitres );

        IF (L_sub(temp, cor_max) > 0)
        {
            cor_max = L_add(temp, 0);
            fraction = i;
            move16();
            t0 = t1;
            move16();
        }
    }

    *pitch_int = t0;
    move16();
    *pitch_fr = fraction;
    move16();

    IF ( sub(t0, pitfr2) >= 0 )
    {
        *index = add( extract_l(L_mac0(L_mult0(sub(t0, pitfr2), shr(pitres,1)),
                                       sub(pitfr2, pitmin), pitres)), shr(fraction,1) );
        move16();
    }
    ELSE
    {
        *index = add(imult1616(sub(t0, pitmin), pitres), fraction);
        move16();
    }

}


static void tcx_ltp_find_gain( Word16 *speech, Word16 *pred_speech, Word16 L_frame, Word16 *gain, Word16 *gain_index )
{
    Word32 corr, ener;
    Word16 i, g, s1, s2, tmp;



    s1 = sub(getScaleFactor16(speech, L_frame), 4);
    s2 = sub(getScaleFactor16(pred_speech, L_frame), 4);

    /* Find gain */
    corr = L_deposit_l(0);
    ener = L_deposit_l(1);

    FOR (i = 0; i < L_frame; i++)
    {
        tmp = shl(pred_speech[i], s2);
        corr = L_mac0(corr, shl(speech[i], s1), tmp);
        ener = L_mac0(ener, tmp, tmp);
    }

    s1 = sub(1, add(s1, s2));
    s2 = sub(1, shl(s2, 1));

    tmp = sub(norm_l(corr), 1);
    corr = L_shl(corr, tmp);
    s1 = sub(s1, tmp);

    tmp = norm_l(ener);
    ener = L_shl(ener, tmp);
    s2 = sub(s2, tmp);

    g = divide1616(round_fx(corr), round_fx(ener));
    BASOP_SATURATE_WARNING_OFF
    g = shl(g, sub(s1, s2));
    BASOP_SATURATE_WARNING_ON

    /* Quantize gain */
    g = shr(sub(g, 0x1000), 13);
    g = s_max(g, -1);

    *gain_index = g;
    move16();

    /* Dequantize gain */
    *gain = imult1616(add(g, 1), 0x1400);
    move16();


}

void tcx_ltp_encode( Word8 tcxltp_on,
                     Word8 tcxOnly,
                     Word16 tcxMode,
                     Word16 L_frame,
                     Word16 L_subfr,
                     Word16 *speech,
                     Word16 *speech_ltp,
                     Word16 *wsp,
                     Word16 Top,
                     Word16 *ltp_param,
                     Word16 *ltp_bits,
                     Word16 *pitch_int,
                     Word16 *pitch_fr,
                     Word16 *gain,
                     Word16 *pitch_int_past,
                     Word16 *pitch_fr_past,
                     Word16 *gain_past,
                     Word16 *norm_corr_past,
                     Word16 last_core,
                     Word16 pitmin,
                     Word16 pitfr1,
                     Word16 pitfr2,
                     Word16 pitmax,
                     Word16 pitres,
                     struct TransientDetection const * pTransientDetection,
                     Word8 SideInfoOnly,
                     Word16 *A,
                     Word16 lpcorder
                   )
{
    Word16 n;
    Word16 norm_corr;
    Word16 pred_speech[L_FRAME32k];
    Word16 tempFlatness;
    Word16 maxEnergyChange;
    Word16 buf_zir[M+L_SUBFR], *zir;
    Word16 Aest[M+1];
    Word16 alpha, step;



    norm_corr = 0;
    move16();

    /* Reset memory if past frame is acelp */
    IF (last_core == ACELP_CORE)
    {
        *pitch_int_past = L_frame;
        move16();
        *pitch_fr_past = 0;
        move16();
        *gain_past = 0;
        move16();
    }

    /* By default, LTP is off */
    ltp_param[0] = 0;
    move16();

    test();
    IF (tcxltp_on != 0 || SideInfoOnly != 0)
    {
        Word16 nPrevSubblocks;
        Word8 isTCX10 = 0;

        if (sub(tcxMode, TCX_10) == 0)
        {
            isTCX10 = 1;
            move16();
        }

        /* Find pitch lag */
        tcx_ltp_pitch_search( Top, pitch_int, pitch_fr, &ltp_param[1], &norm_corr, L_frame, wsp, pitmin, pitfr1, pitfr2, pitmax, pitres );

        nPrevSubblocks = extract_h(L_mac(0x17fff, NSUBBLOCKS, div_s(*pitch_int, L_frame)));
        nPrevSubblocks = add(s_min(nPrevSubblocks, NSUBBLOCKS), 1);

        tempFlatness = GetTCXAvgTemporalFlatnessMeasure(pTransientDetection, NSUBBLOCKS, nPrevSubblocks);

        maxEnergyChange = GetTCXMaxenergyChange(pTransientDetection,
                                                (const Word8) isTCX10,
                                                NSUBBLOCKS, nPrevSubblocks);

        /* Switch LTP on */
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        BASOP_SATURATE_WARNING_OFF;
        if ( (
                    tcxOnly == 0 &&
                    sub(tcxMode, TCX_20) == 0 &&
                    sub(mult(norm_corr, *norm_corr_past), 0x2000) > 0 &&
                    sub(tempFlatness, FL2WORD16_SCALE(3.5f, AVG_FLAT_E)) < 0
                ) ||
                (
                    tcxOnly != 0 &&
                    sub(tcxMode, TCX_10) == 0 &&
                    sub(s_max(norm_corr, *norm_corr_past), 0x4000) > 0 &&
                    sub(maxEnergyChange, FL2WORD16_SCALE(3.5f, NRG_CHANGE_E)) < 0
                ) ||
                ( /* Use LTP for lower correlation when pitch lag is big, L_frame*(1.2f-norm_corr) < pitch_int <=> norm_corr > 1.2f-pitch_int/L_frame */
                    tcxOnly != 0 &&
                    sub(norm_corr, FL2WORD16(0.44f)) > 0 &&
                    L_msu(L_mult(L_frame, sub(FL2WORD16_SCALE(1.2f, 1), shr(norm_corr, 1))), *pitch_int, 1<<14) < 0 /* L_frame*(1.2f-norm_corr) < pitch_int */
                ) ||
                (
                    tcxOnly != 0 &&
                    sub(tcxMode, TCX_20) == 0 &&
                    sub(norm_corr, FL2WORD16(0.44f)) > 0 &&
                    (
                        sub(tempFlatness, FL2WORD16_SCALE(6.0f, AVG_FLAT_E)) < 0 ||
                        (
                            sub(tempFlatness, FL2WORD16_SCALE(7.0f, AVG_FLAT_E)) < 0 &&
                            sub(maxEnergyChange, FL2WORD16_SCALE(22.0f, NRG_CHANGE_E)) < 0
                        )
                    )
                )
           )
        {
            ltp_param[0] = 1;
            move16();
        }
        BASOP_SATURATE_WARNING_ON;
    }

    IF (ltp_param[0] != 0)
    {
        /* Find predicted signal */
        predict_signal( speech, pred_speech, *pitch_int, *pitch_fr, pitres, L_frame );

        /* Find gain */
        tcx_ltp_find_gain( speech, pred_speech, L_frame, gain, &ltp_param[2] );

        /* Total number of bits for LTP */
        IF (add(ltp_param[2], 1) != 0)   /* gain > 0 */
        {
            *ltp_bits = 12;
            move16();
        }
        ELSE   /* gain <= 0 -> turn off LTP */
        {
            ltp_param[0] = 0;
            move16();
        }
    }

    IF (ltp_param[0] == 0)
    {
        /* No LTP -> set everything to zero */
        *pitch_int = L_frame;
        move16();
        *pitch_fr = 0;
        move16();
        ltp_param[1] = 0;
        move16();
        set16_fx( pred_speech, 0, L_frame );
        *gain = 0;
        move16();
        ltp_param[2] = 0;
        move16();

        *ltp_bits = 0;
        move16();
        test();
        if (tcxltp_on != 0 || SideInfoOnly != 0)
        {
            *ltp_bits = 1;
            move16();
        }
    }

    if (SideInfoOnly != 0)
    {
        *gain = 0;
        move16();
    }

    test();
    IF (*gain_past == 0 && *gain == 0)
    {
        Copy(speech, speech_ltp, L_subfr);
    }
    ELSE IF (*gain_past == 0)
    {
        alpha = 0;
        move16();

        /* step = 1.f/(float)(L_subfr); */
        step = shl(2, norm_s(L_subfr));
        if (s_and(L_subfr, sub(L_subfr, 1)) != 0)
        {
            step = mult_r(step, FL2WORD16(64.f/80.f));
        }

        FOR (n = 0; n < L_subfr; n++)
        {
            speech_ltp[n] = sub(speech[n], mult_r(*gain, mult_r(alpha, pred_speech[n])));
            move16();
            BASOP_SATURATE_WARNING_OFF;
            alpha = add(alpha, step);
            BASOP_SATURATE_WARNING_ON;
        }
    }
    ELSE
    {
        IF (A == NULL)
        {
            tcx_ltp_get_lpc(speech-L_frame, L_frame, Aest, lpcorder);
            A = Aest;
        }

        IF (*gain > 0)
        {
            predict_signal(speech-lpcorder, buf_zir, *pitch_int, *pitch_fr, pitres, lpcorder);
        }
        ELSE {
            set16_fx(buf_zir, 0, lpcorder);
        }

        FOR (n = 0; n < lpcorder; n++)
        {
            buf_zir[n] = add(sub(speech_ltp[n-lpcorder], speech[n-lpcorder]), mult_r(*gain, buf_zir[n]));
            move16();
        }

        zir = buf_zir + lpcorder;

        set16_fx(zir, 0, L_subfr);

        E_UTIL_synthesis(0, A, zir, zir, L_subfr, buf_zir, 0, lpcorder);

        alpha = 0x7FFF;
        /* step = 1.f/(float)(L_subfr/2); */
        step = shl(4, norm_s(L_subfr));
        if (s_and(L_subfr, sub(L_subfr, 1)) != 0)
        {
            step = mult_r(step, FL2WORD16(64.f/80.f));
        }

        FOR (n = shr(L_subfr, 1); n < L_subfr; n++)
        {
            zir[n] = mult_r(zir[n], alpha);
            move16();
            alpha = sub(alpha, step);
        }

        FOR (n = 0; n < L_subfr; n++)
        {
            speech_ltp[n] = add(sub(speech[n], mult_r(*gain, pred_speech[n])), zir[n]);
            move16();
        }
    }

    test();
    IF ( SideInfoOnly || *gain == 0)
    {
        FOR ( n=L_subfr; n<L_frame; n++ )
        {
            speech_ltp[n] = speech[n];
            move16();
        }
    }
    ELSE
    {
        FOR ( n=L_subfr; n<L_frame; n++ )
        {
            speech_ltp[n] = sub(speech[n], mult(*gain, pred_speech[n]));
            move16();
        }
    }

    /* Update */
    *pitch_int_past = *pitch_int;
    move16();
    *pitch_fr_past = *pitch_fr;
    move16();
    *gain_past = *gain;
    move16();
    *norm_corr_past = norm_corr;
    move16();

}
