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

#define ALPHA   FL2WORD16(0.85f)


void tcx_ltp_get_lpc(Word16 *x, Word16 L, Word16 *A, Word16 order)
{
    Word16 i, j, s, s2, tmp;
    Word32 r, L_tmp;

    Word16 tmpbuf[L_FRAME_MAX], *p = x;
    Word16 r_l[TCXLTP_LTP_ORDER+1], r_h[TCXLTP_LTP_ORDER+1];


    assert(L <= L_FRAME_MAX);

    /* calc r[0], determine shift */
    s = 0;
    move16();
    r = L_deposit_l(0);
    FOR (j = 0; j < L; j++)
    {
        L_tmp = L_sub(r, 0x40000000);
        if (L_tmp > 0) s = sub(s, 1);
        if (L_tmp > 0) r = L_shr(r, 2);

        tmp = shl(x[j], s);
        r = L_mac0(r, tmp, tmp);
    }
    r = L_max(r, L_shl(100, shl(s, 1)));
    r = Mpy_32_16_1(r, FL2WORD16_SCALE(1.0001f, 1));
    s2 = norm_l(r);
    r = L_shl(r, s2);
    s2 = sub(s2, 1);
    r_l[0] = L_Extract_lc(r, &r_h[0]);
    move16();
    move16();

    IF (s < 0)
    {
        /* shift buffer by s, recompute r[0] to reduce risk of instable LPC */
        r = L_deposit_l(0);
        tmp = lshl((Word16)0x8000, s); /* factor corresponding to right shift by -s */

        FOR (j = 0; j < L; j++)
        {
            tmpbuf[j] = mult_r(x[j], tmp);
            move16();
            r = L_mac0(r, tmpbuf[j], tmpbuf[j]);
        }
        r = L_max(r, L_shl(100, shl(s, 1)));
        r = Mpy_32_16_1(r, FL2WORD16_SCALE(1.0001f, 1));
        s2 = norm_l(r);
        r = L_shl(r, s2);
        s2 = sub(s2, 1);
        r_l[0] = L_Extract_lc(r, &r_h[0]);
        move16();
        move16();

        p = tmpbuf;
    }

    /* calc r[1...] */
    FOR (i = 1; i <= order; i++)
    {
        r = L_deposit_l(0);

        tmp = sub(L, i);
        FOR (j = 0; j < tmp; j++)
        {
            r = L_mac0(r, p[j], p[j+i]);
        }
        r = L_shl(r, s2);
        r_l[i] = L_Extract_lc(r, &r_h[i]);
        move16();
        move16();
    }

    E_LPC_lev_dur(r_h, r_l, A, NULL, order, NULL);

}

static void tcx_ltp_get_zir( Word16 *zir, Word16 length, Word16 *synth_ltp, Word16 *synth, Word16 *A, Word16 lpcorder, Word16 gain, Word16 pitch_int, Word16 pitch_fr, Word16 pitres, Word16 filtIdx )
{
    Word16 buf[TCXLTP_LTP_ORDER], alpha, step;
    Word16 *x0, *x1;
    Word16 *y0, *y1;
    Word32 s, s2;
    const Word16 *w0, *w1, *v0, *v1;
    Word16 i, j, k, L;

    x0 = &synth_ltp[-pitch_int];
    x1 = x0 - 1;
    y0 = synth;
    y1 = y0 - 1;

    assert(filtIdx >= 0);

    w0 = &tcxLtpFilters[filtIdx].filt[pitch_fr];
    w1 = &tcxLtpFilters[filtIdx].filt[sub(pitres, pitch_fr)];
    v0 = &tcxLtpFilters[filtIdx].filt[0];
    v1 = &tcxLtpFilters[filtIdx].filt[pitres];
    L = tcxLtpFilters[filtIdx].length;
    move16();

    FOR (j = 0; j < lpcorder; j++)
    {
        s = L_deposit_l(0);
        s2 = L_deposit_l(0);
        k = 0;
        move16();
        FOR (i = 0; i < L; i++)
        {
            s = L_mac(L_mac(s, w0[k], x0[i]), w1[k], x1[-i]);
            s2 = L_mac(L_mac(s2, v0[k], y0[i]), v1[k], y1[-i]);
            k = add(k, pitres);
        }

        /* s2 *= ALPHA;
           buf[j] = ( synth[j] - gain * s2 ) - ( synth_ltp[j] - gain * s ); */
        i = sub(round_fx(s), mult_r(round_fx(s2), ALPHA));
        buf[j] = add(sub(synth[j], synth_ltp[j]), mult_r(gain, i));
        move16();

        x0++;
        x1++;
        y0++;
        y1++;
    }

    set16_fx(zir, 0, length);

    E_UTIL_synthesis(0, A, zir, zir, length, buf, 0, lpcorder);

    alpha = 0x7FFF;
    move16();
    /* step = 1.f/(float)(length/2); */
    step = shl(4, norm_s(length));
    if (s_and(length, sub(length, 1)) != 0)
    {
        step = mult_r(step, FL2WORD16(64.f/80.f));
    }
    if (sub(length, 240) == 0)
    {
        step = FL2WORD16(1.f/120.f);
        move16();
    }

    FOR (j = shr(length, 1); j < length; j++)
    {
        zir[j] = mult_r(zir[j], alpha);
        move16();
        alpha = sub(alpha, step);
    }
}

void predict_signal(
    const Word16 excI[],  /* i  : input excitation buffer  */
    Word16 excO[],  /* o  : output excitation buffer */
    const Word16 T0,      /* i  : integer pitch lag        */
    Word16 frac,    /* i  : fraction of lag          */
    const Word16 frac_max,/* i  : max fraction             */
    const Word16 L_subfr  /* i  : subframe size            */
)
{
    Word16 j;
    Word32 s;
    const Word16 *x0, *win;




    x0 = &excI[-T0-1];
    frac = negate(frac);

    IF (frac < 0)
    {
        frac = add(frac, frac_max);
        x0--;
    }

    win = &inter4_2tcx2[frac][0];
    if (sub(frac_max, 6) == 0) win = &inter6_2tcx2[frac][0];

    FOR (j = 0; j < L_subfr; j++)
    {
        s = L_mult(win[0], x0[0]);
        s = L_mac(s, win[1], x0[1]);
        s = L_mac(s, win[2], x0[2]);
        excO[j] = mac_r(s, win[3], x0[3]);
        move16();

        x0++;
    }

}

static void tcx_ltp_synth_filter( Word16 *synth_ltp,
                                  Word16 *synth,
                                  Word16 length,
                                  Word16 pitch_int,
                                  Word16 pitch_fr,
                                  Word16 gain,
                                  Word16 pitch_res
                                  ,Word16 *zir       /* can be NULL */
                                  ,Word16 fade       /* 0=normal, +1=fade-in, -1=fade-out */
                                  ,Word16 filtIdx
                                )
{
    Word16 *x0, *x1;
    Word16 *y0, *y1;
    Word32 s, s2;
    const Word16 *v0, *v1;
    const Word16 *w0, *w1;
    Word16 alpha, step = 0; /* initialize just to avoid compiler warning */
    Word16 i, j, k, L;

    IF (gain > 0)
    {
        x0 = &synth_ltp[-pitch_int];
        x1 = x0 - 1;
        y0 = synth;
        y1 = y0 - 1;

        assert(filtIdx >= 0);

        w0 = &tcxLtpFilters[filtIdx].filt[pitch_fr];
        w1 = &tcxLtpFilters[filtIdx].filt[sub(pitch_res, pitch_fr)];
        v0 = &tcxLtpFilters[filtIdx].filt[0];
        v1 = &tcxLtpFilters[filtIdx].filt[pitch_res];

        L = tcxLtpFilters[filtIdx].length;
        move16();

        alpha = 0;
        move16();
        IF (fade != 0)
        {
            if (fade < 0)
            {
                alpha = 0x7FFF;
                move16();
            }

            /* step = 1.f/(float)(length); */
            step = shl(2, norm_s(length));
            if (s_and(length, sub(length, 1)) != 0)
            {
                step = mult_r(step, FL2WORD16(64.f/80.f));
            }
            if (sub(length, 240) == 0)
            {
                step = FL2WORD16(1.f/240.f);
                move16();
            }

            if (fade < 0) step = negate(step);
        }

        FOR (j = 0; j < length; j++)
        {
            s = L_deposit_l(0);
            s2 = L_deposit_l(0);
            k = 0;
            move16();
            FOR (i = 0; i < L; i++)
            {
                s = L_mac(L_mac(s, w0[k], x0[i]), w1[k], x1[-i]);
                s2 = L_mac(L_mac(s2, v0[k], y0[i]), v1[k], y1[-i]);
                k = add(k, pitch_res);
            }

            /* s2 *= ALPHA;
               normal:      synth_ltp[j] = synth[j] - gain * s2 + gain * s;
               zir:         synth_ltp[j] = synth[j] - gain * s2 + gain * s - zir[j];
               fade-in/out: synth_ltp[j] = synth[j] - alpha * gain * s2 + alpha * gain * s; */
            i = sub(round_fx(s), mult_r(round_fx(s2), ALPHA));
            k = mult_r(gain, i);
            if (fade != 0) k = mult_r(k, alpha);
            k = add(synth[j], k);
            if (zir != NULL) k = sub(k, zir[j]);

            synth_ltp[j] = k;
            move16();

            BASOP_SATURATE_WARNING_OFF;
            if (fade != 0) alpha = add(alpha, step);
            BASOP_SATURATE_WARNING_ON;

            x0++;
            x1++;
            y0++;
            y1++;
        }
    }
    ELSE
    {
        Copy( synth, synth_ltp, length );
    }
}


void tcx_ltp_decode_params( Word16 *ltp_param,
                            Word16 *pitch_int,
                            Word16 *pitch_fr,
                            Word16 *gain,
                            Word16 pitmin,
                            Word16 pitfr1,
                            Word16 pitfr2,
                            Word16 pitmax,
                            Word16 pitres
                          )
{
    Word16 tmp, tmp2;



    /* Decode Pitch and Gain */
    test();
    IF (ltp_param != 0 && ltp_param[0] != 0)
    {
        tmp = imult1616(sub(pitfr2, pitmin), pitres);

        IF ( sub(ltp_param[1], tmp) < 0 )
        {
            tmp2 = idiv1616U(ltp_param[1], pitres);

            *pitch_int = add(pitmin, tmp2);
            move16();
            *pitch_fr = sub(ltp_param[1], imult1616(tmp2, pitres));
            move16();
        }
        ELSE
        {
            pitres = shr(pitres, 1);
            tmp2 = imult1616(sub(pitfr1, pitfr2), pitres);

            IF ( sub(ltp_param[1], add(tmp, tmp2)) < 0 )
            {
                tmp2 = idiv1616U(sub(ltp_param[1], tmp), pitres);

                *pitch_int = add(pitfr2, tmp2);
                move16();
                *pitch_fr = shl( sub(sub(ltp_param[1], tmp), imult1616(tmp2, pitres)), 1 );
                move16();
            }
            ELSE {
                *pitch_int = sub(add(ltp_param[1], pitfr1), add(tmp, tmp2));
                move16();
                *pitch_fr = 0;
                move16();
            }
        }

        *gain = imult1616(add(ltp_param[2], 1), 0x1400);
        move16();
    }
    ELSE
    {
        *pitch_int = pitmax;
        move16();
        *pitch_fr = 0;
        move16();
        *gain = 0;
        move16();
    }

}

void tcx_ltp_post( Word8 tcxltp_on,
                   Word16 core,
                   Word16 L_frame,
                   Word16 L_frame_core,
                   Word16 delay,
                   Word16 *sig,
                   Word16 *tcx_buf,
                   Word16 tcx_buf_len,
                   Word16 bfi,
                   Word16 pitch_int,
                   Word16 pitch_fr,
                   Word16 gain,
                   Word16 *pitch_int_past,
                   Word16 *pitch_fr_past,
                   Word16 *gain_past,
                   Word16 *filtIdx_past,
                   Word16 pitres,
                   Word16 *pitres_past,
                   Word16 damping,
                   Word16 SideInfoOnly,
                   Word16 *mem_in,
                   Word16 *mem_out,
                   Word32 bitrate
                 )
{
    Word16 tmp, L_transition, lpcorder, filtIdx;
    Word16 gain2;
    Word32 tmp32;
    Word16 zir[L_FRAME_PLUS/4], A[TCXLTP_LTP_ORDER+1];
    Word16 buf_in[TCXLTP_MAX_DELAY+L_FRAME48k+TCXLTP_MAX_DELAY], buf_out[2*L_FRAME48k];
    Word16 *sig_in, *sig_out;

    filtIdx = 0;  /* just to avoid comilation warnings */

    /******** Init ********/


    /* Parameters */
    L_transition = shr(L_frame, 2);
    lpcorder = TCXLTP_LTP_ORDER;
    move16();

    /* Input buffer */
    sig_in = buf_in + tcx_buf_len;
    Copy( mem_in, buf_in, tcx_buf_len );
    Copy( sig, buf_in+tcx_buf_len, L_frame );
    IF ( core > ACELP_CORE )
    {
        Copy( tcx_buf, sig_in+L_frame, tcx_buf_len );
    }
    Copy( sig+L_frame-tcx_buf_len, mem_in, tcx_buf_len );

    /* Output buffer */
    sig_out = buf_out + L_frame;
    Copy( mem_out, buf_out, L_frame );

    /* TCX-LTP parameters: integer pitch, fractional pitch, gain */
    test();
    test();
    IF ( !(SideInfoOnly != 0 || tcxltp_on != 0) || sub(core, ACELP_CORE) == 0 )
    {
        /* No LTP */
        pitch_int = 0;
        move16();
        pitch_fr = 0;
        move16();
        gain = 0;
        move16();
    }
    ELSE IF (bfi == 0)
    {
        /* LTP and good frame */
        IF (sub(L_frame, L_frame_core) != 0)
        {
            tmp = div_s(L_frame, shl(L_frame_core, 2)); /* Q13 */
            tmp32 = L_mult0(add(imult1616(pitch_int, pitres), pitch_fr), tmp); /* Q13 */
            tmp = round_fx(L_shl(tmp32, 3)); /* Q0 */
            pitch_int = idiv1616U(tmp, pitres);
            pitch_fr = sub(tmp, imult1616(pitch_int, pitres));
        }
        test();
        test();
        IF ( L_sub(bitrate, 48000) == 0 && sub(L_frame_core, L_FRAME16k) == 0 )
        {
            gain = mult_r(gain, FL2WORD16(0.32f));
        }
        ELSE IF ( L_sub(bitrate, 48000) == 0 && sub(L_frame_core, 512) == 0 )
        {
            gain = mult_r(gain, FL2WORD16(0.40f));
        }
        ELSE
        {
            gain = mult_r(gain, FL2WORD16(0.64f));
        }
    }
    ELSE
    {
        /* PLC: [TCX: Fade-out]
         * PLC: LTP and bad frame (concealment) */
        pitch_int = *pitch_int_past;
        move16();
        pitch_fr  = *pitch_fr_past;
        move16();
        gain = shl(mult_r(*gain_past, damping), 1);
        pitres = *pitres_past;
    }


    IF ( SideInfoOnly != 0 )
    {
        gain = 0;
        move16();
        if ( bfi != 0 )
        {
            *gain_past = 0;
            move16();
        }
    }
    gain2 = gain;
    move16();

    IF (sub(L_frame_core, L_FRAME) == 0)
    {
        SWITCH ( L_frame )
        {
        case L_FRAME8k:
            filtIdx = 0;
            move16();
            BREAK;
        case L_FRAME16k:
            filtIdx = 1;
            move16();
            BREAK;
        case L_FRAME32k:
            filtIdx = 2;
            move16();
            BREAK;
        case L_FRAME48k:
            filtIdx = 3;
            move16();
            BREAK;
        default:
            assert(0);
        }
    }
    ELSE IF (sub(L_frame_core, L_FRAME16k) == 0)
    {
        SWITCH ( L_frame )
        {
        case L_FRAME8k:
            filtIdx = 4;
            move16();
            BREAK;
        case L_FRAME16k:
            filtIdx = 5;
            move16();
            BREAK;
        case L_FRAME32k:
            filtIdx = 6;
            move16();
            BREAK;
        case L_FRAME48k:
            filtIdx = 7;
            move16();
            BREAK;
        default:
            assert(0);
        }
    }
    ELSE IF (sub(L_frame_core, 512) == 0)
    {
        SWITCH ( L_frame )
        {
        case L_FRAME8k:
            filtIdx = 8;
            move16();
            BREAK;
        case L_FRAME16k:
            filtIdx = 9;
            move16();
            BREAK;
        case L_FRAME32k:
            filtIdx = 10;
            move16();
            BREAK;
        case L_FRAME48k:
            filtIdx = 11;
            move16();
            BREAK;
        default:
            assert(0);
        }
    }
    ELSE
    {
        filtIdx = -1;
        move16();
    }


    /******** Previous-frame part ********/

    tcx_ltp_synth_filter( sig_out,
                          sig_in,
                          delay,
                          *pitch_int_past,
                          *pitch_fr_past,
                          *gain_past,
                          *pitres_past,
                          NULL,
                          0,
                          *filtIdx_past
                        );

    /******** Transition part ********/

    test();
    test();
    test();
    IF ( gain==0 && *gain_past==0 )
    {
        Copy( sig_in+delay, sig_out+delay, L_transition );
    }
    ELSE IF ( *gain_past==0 )
    {
        tcx_ltp_synth_filter( sig_out+delay,
                              sig_in+delay,
                              L_transition,
                              pitch_int,
                              pitch_fr,
                              gain,
                              pitres,
                              NULL,
                              1,
                              filtIdx
                            );
    }
    ELSE IF ( gain==0 )
    {
        tcx_ltp_synth_filter( sig_out+delay,
                              sig_in+delay,
                              L_transition,
                              *pitch_int_past,
                              *pitch_fr_past,
                              *gain_past,
                              *pitres_past,
                              NULL,
                              -1,
                              *filtIdx_past
                            );
    }
    ELSE IF ( sub(gain, *gain_past) == 0 && sub(pitch_int, *pitch_int_past) == 0 && sub(pitch_fr, *pitch_fr_past) == 0 )
    {
        tcx_ltp_synth_filter( sig_out+delay,
                              sig_in+delay,
                              L_transition,
                              pitch_int,
                              pitch_fr,
                              gain,
                              pitres,
                              NULL,
                              0,
                              filtIdx
                            );
    }
    ELSE
    {
        tcx_ltp_get_lpc( sig_out+delay-L_frame, L_frame, A, lpcorder );

        tcx_ltp_get_zir( zir, L_transition, sig_out+delay-lpcorder, sig_in+delay-lpcorder, A, lpcorder, gain, pitch_int, pitch_fr, pitres, filtIdx );

        tcx_ltp_synth_filter( sig_out+delay,
        sig_in+delay,
        L_transition,
        pitch_int,
        pitch_fr,
        gain,
        pitres,
        zir,
        0,
        filtIdx
                            );
    }

    /******** Current-frame part ********/

    tcx_ltp_synth_filter( sig_out+(delay+L_transition),
                          sig_in+(delay+L_transition),
                          L_frame-(delay+L_transition),
                          pitch_int,
                          pitch_fr,
                          gain,
                          pitres,
                          NULL,
                          0,
                          filtIdx
                        );


    /******** Output ********/


    /* copy to output */

    Copy( sig_out, sig, L_frame );

    /* Update */
    *pitch_int_past = pitch_int;
    move16();
    *pitch_fr_past = pitch_fr;
    move16();
    *gain_past = gain2;
    move16();
    *filtIdx_past = filtIdx;
    move16();
    *pitres_past = pitres;
    Copy( sig_out, mem_out, L_frame );

}
