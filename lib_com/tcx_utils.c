/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "stl.h"
#include "options.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "rom_basop_util.h"
#include "basop_util.h"

#define inv_int InvIntTable


Word16 getInvFrameLen(Word16 L_frame) /* returns 1/L_frame in Q21 format */
{
    Word16 idx, s;

    s = norm_s(L_frame);
    idx = shl(L_frame, s);

    assert((idx == 0x4000) || (idx == 0x4B00) || (idx == 0x5000) || (idx == 0x5A00) || (idx == 0x6000) || (idx == 0x6400) || (idx == 0x7800));

    idx = mult_r(idx, 0x10); /* idx = shr(add(idx, 0x0400), 11); */
    idx = s_and(idx, 7);


    return shl(L_frame_inv[idx], sub(s, 7));
}

static void tcx_get_windows(
    TCX_config const * tcx_cfg,       /* i: TCX configuration                         */
    Word16 left_mode,                 /* i: overlap mode of left window half          */
    Word16 right_mode,                /* i: overlap mode of right window half         */
    Word16 *left_overlap,             /* o: left overlap length                       */
    PWord16 const **left_win,         /* o: left overlap window                       */
    Word16 *right_overlap,            /* o: right overlap length                      */
    PWord16 const **right_win,        /* o: right overlap window                      */
    Word8 fullband                    /* i: fullband flag                             */
)
{
    IF (fullband == 0)
    {
        /* Left part */
        SWITCH (left_mode)
        {
        case TRANSITION_OVERLAP: /* ACELP->TCX transition */
            *left_overlap = tcx_cfg->tcx_mdct_window_trans_length;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_trans;
            BREAK;
        case MIN_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_min_length;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_minimum;
            BREAK;
        case HALF_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_half_length;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_half;
            BREAK;
        case RECTANGULAR_OVERLAP:
            *left_overlap = 0;
            move16();
            *left_win = NULL;
            BREAK;
        case FULL_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_length;
            move16();
            *left_win = tcx_cfg->tcx_aldo_window_1_trunc;
            BREAK;
        default:
            assert(!"Not supported overlap");
        }

        /* Right part */
        SWITCH (right_mode)
        {
        case MIN_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_min_length;
            move16();
            *right_win = tcx_cfg->tcx_mdct_window_minimum;
            BREAK;
        case HALF_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_half_length;
            move16();
            *right_win = tcx_cfg->tcx_mdct_window_half;
            BREAK;
        case RECTANGULAR_OVERLAP:
            *right_overlap = 0;
            move16();
            *right_win = NULL;
            BREAK;
        case FULL_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_delay;
            move16();
            *right_win = tcx_cfg->tcx_aldo_window_2;
            BREAK;
        default:
            assert(!"Not supported overlap");
        }
    }
    ELSE
    {
        /* Left part */
        SWITCH (left_mode)
        {
        case TRANSITION_OVERLAP: /* ACELP->TCX transition */
            *left_overlap = tcx_cfg->tcx_mdct_window_trans_lengthFB;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_transFB;
            BREAK;
        case MIN_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_min_lengthFB;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_minimumFB;
            BREAK;
        case HALF_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_half_lengthFB;
            move16();
            *left_win = tcx_cfg->tcx_mdct_window_halfFB;
            BREAK;
        case RECTANGULAR_OVERLAP:
            *left_overlap = 0;
            move16();
            *left_win = NULL;
            BREAK;
        case FULL_OVERLAP:
            *left_overlap = tcx_cfg->tcx_mdct_window_lengthFB;
            move16();
            *left_win = tcx_cfg->tcx_aldo_window_1_FB_trunc;
            BREAK;
        default:
            assert(!"Not supported overlap");
        }

        /* Right part */
        SWITCH (right_mode)
        {
        case MIN_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_min_lengthFB;
            move16();
            *right_win = tcx_cfg->tcx_mdct_window_minimumFB;
            BREAK;
        case HALF_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_half_lengthFB;
            move16();
            *right_win = tcx_cfg->tcx_mdct_window_halfFB;
            BREAK;
        case RECTANGULAR_OVERLAP:
            *right_overlap = 0;
            move16();
            *right_win = NULL;
            BREAK;
        case FULL_OVERLAP:
            *right_overlap = tcx_cfg->tcx_mdct_window_delayFB;
            move16();
            *right_win = tcx_cfg->tcx_aldo_window_2_FB;
            BREAK;
        default:
            assert(!"Not supported overlap");
        }
    }
}

static void tcx_windowing_analysis(
    Word16 const *signal,       /* i: signal vector                              */
    Word16 L_frame,             /* i: frame length                               */
    Word16 left_overlap,        /* i: left overlap length                        */
    PWord16 const *left_win,    /* i: left overlap window                        */
    Word16 right_overlap,       /* i: right overlap length                       */
    PWord16 const *right_win,   /* i: right overlap window                       */
    Word16 *output              /* o: windowed signal vector                     */
)
{
    Word16 w, n;

    /* Left overlap */
    n = shr(left_overlap, 1);
    FOR (w = 0; w < n; w++)
    {
        *output++ = mult_r(*signal++, left_win[w].v.im);
        move16();
    }
    FOR (w = 0; w < n; w++)
    {
        *output++ = mult_r(*signal++, left_win[n-1-w].v.re);
        move16();
    }

    /* Non overlapping region */
    n = sub(L_frame, shr(add(left_overlap, right_overlap), 1));
    FOR (w = 0; w < n; w++)
    {
        *output++ = *signal++;
        move16();
    }

    /* Right overlap */
    n = shr(right_overlap, 1);
    FOR (w = 0; w < n; w++)
    {
        *output++ = mult_r(*signal++, right_win[w].v.re);
        move16();
    }
    FOR (w = 0; w < n; w++)
    {
        *output++ = mult_r(*signal++, right_win[n-1-w].v.im);
        move16();
    }
}

void WindowSignal(
    TCX_config const *tcx_cfg,                /* input: configuration of TCX              */
    Word16 offset,                            /* input: left folding point offset relative to the input signal pointer */
    Word16 left_overlap_mode,                 /* input: overlap mode of left window half  */
    Word16 right_overlap_mode,                /* input: overlap mode of right window half */
    Word16 * left_overlap_length,             /* output: TCX window left overlap length   */
    Word16 * right_overlap_length,            /* output: TCX window right overlap length  */
    Word16 const in[],                        /* input: input signal                      */
    Word16 * L_frame,                         /* input/output: frame length               */
    Word16 out[]                              /* output: output windowed signal           */
    ,Word8 fullband                           /* input: fullband flag                     */
)
{
    Word16 l, r;
    PWord16 const * left_win;
    PWord16 const * right_win;


    /*-----------------------------------------------------------*
     * Init                                                      *
     *-----------------------------------------------------------*/

    tcx_get_windows(tcx_cfg, left_overlap_mode, right_overlap_mode, &l, &left_win, &r, &right_win, fullband);

    /* Init lengths */

    /* if past frame is ACELP */
    IF (sub(left_overlap_mode, TRANSITION_OVERLAP) == 0)
    {
        /* Increase frame size for 5ms */
        IF (fullband == 0)
        {
            *L_frame = add(*L_frame, tcx_cfg->tcx5Size);
            move16();
            offset = negate(shr(tcx_cfg->tcx_mdct_window_trans_length, 1));
        }
        ELSE
        {
            *L_frame = add(*L_frame, tcx_cfg->tcx5SizeFB);
            move16();
            offset = negate(shr(tcx_cfg->tcx_mdct_window_trans_lengthFB, 1));
        }
    }

    /*-----------------------------------------------------------*
     * Windowing                                                 *
     *-----------------------------------------------------------*/

    tcx_windowing_analysis(in-shr(l,1)+offset, *L_frame, l, left_win, r, right_win, out);

    IF (sub(left_overlap_mode, FULL_OVERLAP) == 0)
    {
        /* fade truncated ALDO window to avoid discontinuities */
        Word16 i, tmp;
        const PWord16 *p;

        p = tcx_cfg->tcx_mdct_window_minimum;
        tmp = shr(tcx_cfg->tcx_mdct_window_min_length, 1);
        IF (fullband != 0)
        {
            p = tcx_cfg->tcx_mdct_window_minimumFB;
            tmp = shr(tcx_cfg->tcx_mdct_window_min_lengthFB, 1);
        }

        FOR (i = 0; i < tmp; i++)
        {
            out[i] = mult_r(out[i], p[i].v.im);
            move16();
        }
        FOR (i = 0; i < tmp; i++)
        {
            out[i+tmp] = mult_r(out[i+tmp], p[tmp-1-i].v.re);
            move16();
        }
    }

    *left_overlap_length = l;
    move16();
    *right_overlap_length = r;
    move16();

}

void tcx_windowing_synthesis_current_frame(
    Word16 *signal,             /* i/o: signal vector                            */
    const PWord16 *window,      /* i: TCX window vector                          */
    const PWord16 *window_half, /* i: TCX window vector for half-overlap window  */
    const PWord16 *window_min,  /* i: TCX minimum overlap window                 */
    Word16 window_length,       /* i: TCX window length                          */
    Word16 window_half_length,  /* i: TCX half window length                     */
    Word16 window_min_length,   /* i: TCX minimum overlap length                 */
    Word16 left_rect,           /* i: left part is rectangular                   */
    Word16 left_mode,           /* i: overlap mode of left window half           */
    Word16 *acelp_zir,          /* i: acelp ZIR                                  */
    Word16 *old_syn,
    Word16 *syn_overl,
    Word16 *A_zir,
    const PWord16 *window_trans,
    Word16 acelp_zir_len,
    Word16 acelp_mem_len,
    Word16 bfi,
    Word16 last_core_bfi,       /* i :  last core                                */
    Word8 last_is_cng
    ,Word16 fullbandScale
)
{

    Word16 i, overlap, n, tmp, tmp2;
    Word16 tmp_buf[L_FRAME_MAX/2];


    /* Init */

    overlap = shr(window_length, 1);

    /* Past-frame is TCX concealed as CNG and current-frame is TCX */
    test();
    IF ( sub(last_is_cng, 1)==0 && left_rect==0 )
    {
        IF (!fullbandScale)
        {
            set16_fx(acelp_zir, 0, acelp_zir_len);
            E_UTIL_synthesis(0, A_zir, acelp_zir, acelp_zir, acelp_zir_len, signal+overlap+acelp_mem_len-M, 0, M);
        }
        ELSE
        {
            lerp(acelp_zir, tmp_buf, acelp_zir_len, idiv1616U(shl(acelp_zir_len, LD_FSCALE_DENOM), fullbandScale));
            acelp_zir = tmp_buf;
        }

        FOR (i = 0; i < acelp_zir_len; i++)
        {
            /*signal[i] *= (float)(i)/(float)(acelp_zir_len);
            signal[i] += acelp_zir[i]*(float)(acelp_zir_len-i)/(float)(acelp_zir_len);*/
            move16();
            signal[i] = add(mult_r(signal[i], div_s(i, acelp_zir_len)), mult_r(acelp_zir[i], div_s(sub(acelp_zir_len, i), acelp_zir_len)));
        }
    }
    ELSE
    /* Rectangular window (past-frame is ACELP) */
    test();
    test();
    test();
    IF ( sub(left_rect, 1)==0 && last_core_bfi==ACELP_CORE )
    {
        tmp = sub(overlap,acelp_mem_len);
        FOR (i=0; i< tmp ; i++)
        {
            move16();
            signal[i] = 0;
        }

        IF (fullbandScale == 0)
        {

            tmp = shl(acelp_mem_len, 1);

            /*OLA with ACELP*/
            FOR (i = 0; i < acelp_mem_len; i++)
            {

                /*window decoded TCX with aliasing*/
                tmp2 = mult_r(signal[i+overlap-acelp_mem_len], window_trans[i].v.im);

                /*Time TDAC: 1)forward part of ACELP*/
                tmp2 = add(tmp2, mult_r(old_syn[acelp_zir_len-tmp+i], mult_r(window_trans[i].v.re, window_trans[i].v.re)));

                /*Time TDAC: 1)reward part of ACELP*/
                tmp2 = add(tmp2, mult_r(old_syn[acelp_zir_len-i-1], mult_r(window_trans[i].v.im, window_trans[i].v.re)));

                move16();
                signal[i+overlap-acelp_mem_len] = tmp2;
            }
            FOR ( ; i < tmp; i++)
            {
                Word16 tmp2;

                /*window decoded TCX with aliasing*/
                tmp2 = mult_r(signal[i+overlap-acelp_mem_len], window_trans[tmp-1-i].v.re);

                /*Time TDAC: 1)forward part of ACELP*/
                tmp2 = add(tmp2, mult_r(old_syn[acelp_zir_len-tmp+i], mult_r(window_trans[tmp-1-i].v.im, window_trans[tmp-1-i].v.im)));

                /*Time TDAC: 1)reward part of ACELP*/
                tmp2 = add(tmp2, mult_r(old_syn[acelp_zir_len-i-1], mult_r(window_trans[tmp-1-i].v.re, window_trans[tmp-1-i].v.im)));

                move16();
                signal[i+overlap-acelp_mem_len] = tmp2;
            }

            FOR (i=0; i<M; i++)
            {
                move16();
                signal[overlap+acelp_mem_len-M+i] = sub(signal[overlap+acelp_mem_len-M+i], old_syn[acelp_zir_len-M+i]);
            }
        }
        /* ZIR at the end of the ACELP frame */
        move16();
        acelp_zir_len=64;

        IF (fullbandScale == 0)
        {
            set16_fx(acelp_zir, 0, acelp_zir_len);
            E_UTIL_synthesis(0, A_zir, acelp_zir, acelp_zir, acelp_zir_len, signal+overlap+acelp_mem_len-M, 0, M);
        }
        ELSE
        {
            tmp = extract_l(L_shr(L_mult0(acelp_zir_len, fullbandScale), LD_FSCALE_DENOM));
            lerp(acelp_zir, tmp_buf, tmp, acelp_zir_len);
            acelp_zir_len = tmp;
            move16();
            acelp_zir = tmp_buf;
        }

        FOR (i = 0; i < acelp_zir_len; i++)
        {
            /*remove reconstructed ZIR and add ACELP ZIR*/
            move16();
            signal[i+overlap+acelp_mem_len] = sub(signal[i+overlap+acelp_mem_len], mult_r(acelp_zir[i], div_s(sub(acelp_zir_len, i), acelp_zir_len)));
        }
        /* Rectangular window (past-frame is TCX) */
    }
    ELSE IF ( left_rect==1 && last_core_bfi!=ACELP_CORE )
    {
        n = add(overlap, acelp_mem_len);
        FOR (i=0; i<n; i++)
        {
            move16();
            signal[i] = 0;
        }

        IF ( bfi == 0 )
        {
            n = shr(window_length,1);
            FOR (i=0; i<n; i++)
            {
                move16();
                signal[i+overlap+acelp_mem_len] = mult_r(signal[i+overlap+acelp_mem_len], window[i].v.im);
            }
            FOR (; i<window_length; i++)
            {
                move16();
                signal[i+overlap+acelp_mem_len] = mult_r(signal[i+overlap+acelp_mem_len], window[window_length-1-i].v.re);
            }
        }
        ELSE
        {
            n = shr(window_half_length,1);
            FOR (i=0; i<n; i++)
            {
                move16();
                signal[i+overlap+acelp_mem_len] = mult_r(signal[i+overlap+acelp_mem_len], window_half[i].v.im);
            }
            FOR (; i<window_half_length; i++)
            {
                move16();
                signal[i+overlap+acelp_mem_len] = mult_r(signal[i+overlap+acelp_mem_len], window_half[window_half_length-1-i].v.re);
            }
        }

        /* Normal window (past-frame is ACELP) */
    }
    ELSE IF (left_rect != 1 && last_core_bfi == ACELP_CORE)
    {

        n = shr(window_length,1);
        FOR (i=0; i<n; i++)
        {
            move16();
            signal[i] = mult_r(signal[i], window[i].v.im);
        }
        FOR (; i<window_length; i++)
        {
            move16();
            signal[i] = mult_r(signal[i], window[window_length-1-i].v.re);
        }

        FOR (i=0; i<window_length /*acelp_zir_len*/; i++)
        {
            move16();
            signal[i] = add(signal[i], syn_overl[i]);
        }

        /* Normal window (past-frame is TCX) */
    }
    ELSE
    {
        IF ( sub(left_mode, 2) == 0 )     /* min. overlap */
        {
            n = shr(sub(window_length, window_min_length), 1);
            FOR (i = 0; i < n; i++)
            {
                *signal++ = 0;
                move16();
            }

            n = shr(window_min_length, 1);
            FOR (i = 0; i < n; i++)
            {
                *signal = mult_r(*signal, window_min[i].v.im);
                move16();
                signal++;
            }
            FOR (i = 0; i < n; i++)
            {
                *signal = mult_r(*signal, window_min[n-1-i].v.re);
                move16();
                signal++;
            }
        }
        ELSE IF ( sub(left_mode, 3) == 0 )     /* half OL */
        {
            Word16 w;

            n = shr(sub(window_length,window_half_length),1);
            FOR (i = 0; i < n; i++)
            {
                move16();
                signal[i] = 0;
            }
            n = shr(window_half_length,1);
            FOR (w = 0; w < n; w++)
            {
                move16();
                signal[i] = mult_r(signal[i], window_half[w].v.im);
                i = add(i,1);
            }
            FOR (w = 0; w < n; w++)
            {
                move16();
                signal[i] = mult_r(signal[i], window_half[window_half_length/2-1-w].v.re);
                i = add(i,1);
            }
        }
        ELSE {   /* normal full/maximum overlap */

            n = shr(window_length,1);
            FOR (i = 0; i < n; i++)
            {
                move16();
                signal[i] = mult_r(signal[i], window[i].v.im);
            }
            FOR (; i < window_length; i++)
            {
                move16();
                signal[i] = mult_r(signal[i], window[window_length-1-i].v.re);
            }
        }
    }

    /* Right part asymmetric window : with ALDO do nothing->can be skipped*/

}

void tcx_windowing_synthesis_past_frame(
    Word16 *signal,             /* i/o: signal vector                            */
    const PWord16 *window,      /* i: TCX window vector                          */
    const PWord16 *window_half, /* i: TCX window vector for half-overlap window  */
    const PWord16 *window_min,  /* i: TCX minimum overlap window                 */
    Word16 window_length,       /* i: TCX window length                          */
    Word16 window_half_length,  /* i: TCX half window length                     */
    Word16 window_min_length,   /* i: TCX minimum overlap length                 */
    Word16 right_mode           /* i: overlap mode (left_mode of current frame)  */
)
{

    Word16 i, n;


    IF ( sub(right_mode, 2) == 0 )    /* min. overlap */
    {
        signal += shr(sub(window_length, window_min_length), 1);

        n = shr(window_min_length, 1);
        FOR (i = 0; i < n; i++)
        {
            *signal = mult_r(*signal, window_min[i].v.re);
            move16();
            signal++;
        }
        FOR (i = 0; i < n; i++)
        {
            *signal = mult_r(*signal, window_min[n-1-i].v.im);
            move16();
            signal++;
        }

        n = shr(sub(window_length, window_min_length), 1);
        FOR (i = 0; i < n; i++)
        {
            *signal = 0;
            move16();
            signal++;
        }
    }
    ELSE IF ( sub(right_mode,3) == 0)    /* half OL */
    {
        Word16 w;

        i = shr(sub(window_length, window_half_length),1);
        n = shr(window_half_length,1);
        FOR (w=0 ; w < n; w++)
        {
            signal[i] = mult_r(signal[i], window_half[w].v.re);
            move16();
            i = add(i,1);
        }
        FOR (w=0 ; w < n; w++)
        {
            signal[i] = mult_r(signal[i], window_half[window_half_length/2-1-w].v.im);
            move16();
            i = add(i,1);
        }
        FOR (; i < window_length; i++)
        {
            move16();
            signal[i] = 0;
        }
    }
    ELSE     /* normal full/maximum overlap */
    {

        n = shr(window_length,1);
        FOR (i = 0; i < n; i++)
        {
            move16();
            signal[i] = mult_r(signal[i], window[i].v.re);
            move16();
            signal[window_length-1-i] = mult_r(signal[window_length-1-i], window[i].v.im);
        }
    }

}

void lpc2mdct(Word16 *lpcCoeffs, Word16 lpcOrder,
              Word16 *mdct_gains, Word16 *mdct_gains_exp,
              Word16 *mdct_inv_gains, Word16 *mdct_inv_gains_exp)
{
    Word32 RealData[FDNS_NPTS];
    Word32 ImagData[FDNS_NPTS];
    Word16 i, j, k, sizeN, step, scale, s, tmp16;
    Word16 g, g_e, ig, ig_e;
    Word32 tmp32;
    const PWord16 *ptwiddle;
    Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH];



    sizeN = shl(FDNS_NPTS, 1);

    BASOP_getTables(NULL, &ptwiddle, &step, sizeN);

    /*ODFT*/
    assert(lpcOrder < FDNS_NPTS);

    /* pre-twiddle */
    FOR (i=0; i<=lpcOrder; i++)
    {
        RealData[i] = L_mult(lpcCoeffs[i], ptwiddle->v.re);
        move32();
        ImagData[i] = L_negate(L_mult(lpcCoeffs[i], ptwiddle->v.im));
        move32();
        ptwiddle += step;
    }

    /* zero padding */
    FOR ( ; i<FDNS_NPTS; i++)
    {
        RealData[i] = L_deposit_l(0);
        ImagData[i] = L_deposit_l(0);
    }

    /* half length FFT */
    scale = add(norm_s(lpcCoeffs[0]),1);

    BASOP_cfft(RealData, ImagData, FDNS_NPTS, 1, &scale, workBuffer);


    /*Get amplitude*/
    j = sub(FDNS_NPTS, 1);
    k = 0;
    move16();

    FOR (i=0; i<FDNS_NPTS/2; i++)
    {
        s = sub(norm_l(L_max(L_abs(RealData[i]), L_abs(ImagData[i]))), 1);

        tmp16 = extract_h(L_shl(RealData[i], s));
        tmp32 = L_mult(tmp16, tmp16);

        tmp16 = extract_h(L_shl(ImagData[i], s));
        tmp16 = mac_r(tmp32, tmp16, tmp16);

        s = shl(sub(scale, s), 1);

        if (tmp16 == 0)
        {
            s = -16;
            move16();
        }
        if (tmp16 == 0)
        {
            tmp16 = 1;
            move16();
        }

        BASOP_Util_Sqrt_InvSqrt_MantExp(tmp16, s, &g, &g_e, &ig, &ig_e);

        if (mdct_gains != 0)
        {
            mdct_gains[k] = g;
            move16();
        }

        if (mdct_gains_exp != 0)
        {
            mdct_gains_exp[k] = g_e;
            move16();
        }

        if (mdct_inv_gains != 0)
        {
            mdct_inv_gains[k] = ig;
            move16();
        }

        if (mdct_inv_gains_exp != 0)
        {
            mdct_inv_gains_exp[k] = ig_e;
            move16();
        }

        k = add(k, 1);


        s = sub(norm_l(L_max(L_abs(RealData[j]), L_abs(ImagData[j]))), 1);

        tmp16 = extract_h(L_shl(RealData[j], s));
        tmp32 = L_mult(tmp16, tmp16);

        tmp16 = extract_h(L_shl(ImagData[j], s));
        tmp16 = mac_r(tmp32, tmp16, tmp16);

        s = shl(sub(scale, s), 1);

        if (tmp16 == 0)
        {
            s = -16;
            move16();
        }
        if (tmp16 == 0)
        {
            tmp16 = 1;
            move16();
        }

        BASOP_Util_Sqrt_InvSqrt_MantExp(tmp16, s, &g, &g_e, &ig, &ig_e);

        if (mdct_gains != 0)
        {
            mdct_gains[k] = g;
            move16();
        }

        if (mdct_gains_exp != 0)
        {
            mdct_gains_exp[k] = g_e;
            move16();
        }

        if (mdct_inv_gains != 0)
        {
            mdct_inv_gains[k] = ig;
            move16();
        }

        if (mdct_inv_gains_exp != 0)
        {
            mdct_inv_gains_exp[k] = ig_e;
            move16();
        }

        j = sub(j, 1);
        k = add(k, 1);
    }

}

/**
 * \brief Perform mdct shaping. In the floating point software there are two functions,
 *        mdct_noiseShaping and mdct_preShaping, which are combined here into a single function.
 * \param x spectrum mantissas
 * \param lg spectrum length
 * \param gains shaping gains mantissas
 * \param gains_exp shaping gains exponents
 */
void mdct_shaping(Word32 x[], Word16 lg, Word16 const gains[], Word16 const gains_exp[])
{

    Word16 i, k, l;
    Word16 m, n, k1, k2, j;
    Word32 * px = x;
    Word16 const * pgains = gains;
    Word16 const * pgainsexp = gains_exp;

    /* FDNS_NPTS = 64 */
    k = shr(lg, 6);
    m = s_and(lg, 0x3F);

    IF (m != 0)
    {
        IF ( sub( m, FDNS_NPTS/2 ) <= 0 )
        {
            n = idiv1616U(FDNS_NPTS,m);
            k1 = k;
            move16();
            k2 = add(k,1);
        }
        ELSE
        {
            n = idiv1616U(FDNS_NPTS,sub(FDNS_NPTS,m));
            k1 = add(k,1);
            k2 = k;
            move16();
        }

        i = 0;
        move16();
        j = 0;
        move16();

        WHILE (sub(i, lg) < 0)
        {

            k = k2;
            move16();
            if (j != 0)
            {
                k = k1;
                move16();
            }

            j = add(j, 1);
            if ( sub(j, n) == 0 )
            {
                j = 0;
                move16();
            }

            /* Limit number of loops, if end is reached */
            k = s_min(k, sub(lg, i));

            FOR (l=0; l < k; l++)
            {
                *x = L_shl(Mpy_32_16_1(*x, *gains), *gains_exp);
                move32();
                x++;
            }
            i = add(i, k);

            gains++;
            gains_exp++;
        }
    }
    ELSE
    {
        FOR (l=0; l < k; l++)
        {
            x = &px[l];
            gains = pgains;
            gains_exp = pgainsexp;
            FOR (i=0; i<FDNS_NPTS; i++)
            {
                *x = L_shl(Mpy_32_16_1(*x, *gains), *gains_exp);
                move32();
                x += k;
                gains++;
                gains_exp++;
            }
        }
    }

}

void mdct_shaping_16(Word16 const x[], Word16 lg, Word16 lg_total, Word16 const gains[], Word16 const gains_exp[], Word16 gains_max_exp, Word32 y[])
{
    Word16 i, k, l;
    Word16 m, n, k1, k2, j, gain_exp;
    Word16 const * px;
    Word32 * py;
    Word16 const * pgains = gains;
    Word16 gains_exp_loc[FDNS_NPTS];
    Word16 const * pgains_exp;

    /* FDNS_NPTS = 64 */
    k = shr(lg, 6);
    m = s_and(lg, 0x3F);

    IF (m != 0)
    {
        pgains_exp = gains_exp;
        px = x;
        py = y;

        IF ( sub( m, FDNS_NPTS/2 ) <= 0 )
        {
            n = idiv1616U(FDNS_NPTS,m);
            k1 = k;
            move16();
            k2 = add(k,1);
        }
        ELSE
        {
            n = idiv1616U(FDNS_NPTS,sub(FDNS_NPTS,m));
            k1 = add(k,1);
            k2 = k;
            move16();
        }

        i = 0;
        move16();
        j = 0;
        move16();

        WHILE (sub(i, lg) < 0)
        {

            k = k2;
            move16();
            if (j != 0)
            {
                k = k1;
                move16();
            }

            j = add(j, 1);
            if ( sub(j, n) == 0 )
            {
                j = 0;
                move16();
            }

            /* Limit number of loops, if end is reached */
            k = s_min(k, sub(lg, i));

            gain_exp = sub(*pgains_exp, gains_max_exp);

            FOR (l=0; l < k; l++)
            {
                *py = L_shl(L_mult(*px, *pgains), gain_exp);
                move32();
                py++;
                px++;
            }
            i = add(i, k);

            pgains++;
            pgains_exp++;
        }
    }
    ELSE
    {
        FOR (i=0; i<FDNS_NPTS; i++)
        {
            gains_exp_loc[i] = sub(gains_exp[i], gains_max_exp);
            move16();
        }
        FOR (l=0; l < k; l++)
        {
            px = &x[l];
            py = &y[l];
            pgains = gains;
            pgains_exp = gains_exp_loc;
            FOR (i=0; i<FDNS_NPTS; i++)
            {
                *py = L_shl(L_mult(*px, *pgains), *pgains_exp);
                move32();
                px += k;
                py += k;
                pgains++;
                pgains_exp++;
            }
        }
    }

    gain_exp = sub(gains_exp[FDNS_NPTS-1], gains_max_exp);
    FOR (i = lg; i < lg_total; i++)
    {
        y[i] = L_shl(L_mult(x[i], gains[FDNS_NPTS-1]), gain_exp);
        move32();
    }
}

void mdct_noiseShaping_interp(Word32 x[], Word16 lg, Word16 gains[], Word16 gains_exp[])
{
    Word16 i, j, jp, jn, k, l;
    Word16 g, pg, ng, e, tmp;


    assert(lg % FDNS_NPTS == 0);
    k = shr(lg, 6); /* FDNS_NPTS = 64 */

    IF (gains)
    {
        /* Linear interpolation */
        IF (sub(k, 4) == 0)
        {
            jp = 0;
            move16();
            j = 0;
            move16();
            jn = 1;
            move16();

            FOR (i = 0; i < lg; i += 4)
            {
                pg = gains[jp];
                move16();
                g  = gains[j];
                move16();
                ng = gains[jn];
                move16();

                /* common exponent for pg and g */
                tmp = sub(gains_exp[j], gains_exp[jp]);
                if (tmp > 0) pg = shr(pg, tmp);
                if (tmp < 0) g = shl(g, tmp);
                e = s_max(gains_exp[j], gains_exp[jp]);

                tmp = mac_r(L_mult(pg, FL2WORD16(0.375f)), g, FL2WORD16(0.625f));
                x[i] = L_shl(Mpy_32_16_1(x[i], tmp), e);
                move32();

                tmp = mac_r(L_mult(pg, FL2WORD16(0.125f)), g, FL2WORD16(0.875f));
                x[i+1] = L_shl(Mpy_32_16_1(x[i+1], tmp), e);
                move32();

                /* common exponent for g and ng */
                g = gains[j];
                move16();
                tmp = sub(gains_exp[j], gains_exp[jn]);
                if (tmp > 0) ng = shr(ng, tmp);
                if (tmp < 0) g = shl(g, tmp);
                e = s_max(gains_exp[j], gains_exp[jn]);

                tmp = mac_r(L_mult(g, FL2WORD16(0.875f)), ng, FL2WORD16(0.125f));
                x[i+2] = L_shl(Mpy_32_16_1(x[i+2], tmp), e);
                move32();

                tmp = mac_r(L_mult(g, FL2WORD16(0.625f)), ng, FL2WORD16(0.375f));
                x[i+3] = L_shl(Mpy_32_16_1(x[i+3], tmp), e);
                move32();

                jp = j;
                move16();
                j = jn;
                move16();
                jn = s_min(add(jn, 1), FDNS_NPTS-1);
            }
        }
        ELSE IF (sub(k, 5) == 0)
        {
            jp = 0;
            move16();
            j = 0;
            move16();
            jn = 1;
            move16();

            FOR (i = 0; i < lg; i += 5)
            {
                pg = gains[jp];
                move16();
                g  = gains[j];
                move16();
                ng = gains[jn];
                move16();

                /* common exponent for pg and g */
                tmp = sub(gains_exp[j], gains_exp[jp]);
                if (tmp > 0) pg = shr(pg, tmp);
                if (tmp < 0) g = shl(g, tmp);
                e = s_max(gains_exp[j], gains_exp[jp]);

                tmp = mac_r(L_mult(pg, FL2WORD16(0.40f)), g, FL2WORD16(0.60f));
                x[i]   = L_shl(Mpy_32_16_1(x[i], tmp), e);
                move32();

                tmp = mac_r(L_mult(pg, FL2WORD16(0.20f)), g, FL2WORD16(0.80f));
                x[i+1] = L_shl(Mpy_32_16_1(x[i+1], tmp), e);
                move32();


                x[i+2] = L_shl(Mpy_32_16_1(x[i+2], gains[j]), gains_exp[j]);
                move32();

                /* common exponent for g and ng */
                g = gains[j];
                move16();
                tmp = sub(gains_exp[j], gains_exp[jn]);
                if (tmp > 0) ng = shr(ng, tmp);
                if (tmp < 0) g = shl(g, tmp);
                e = s_max(gains_exp[j], gains_exp[jn]);

                tmp = mac_r(L_mult(g, FL2WORD16(0.80f)), ng, FL2WORD16(0.20f));
                x[i+3] = L_shl(Mpy_32_16_1(x[i+3], tmp), e);
                move32();

                tmp = mac_r(L_mult(g, FL2WORD16(0.60f)), ng, FL2WORD16(0.40f));
                x[i+4] = L_shl(Mpy_32_16_1(x[i+4], tmp), e);
                move32();

                jp = j;
                move16();
                j = jn;
                move16();
                jn = s_min(add(jn, 1), FDNS_NPTS-1);
            }
        }
        ELSE   /* no interpolation */
        {
            FOR (i = 0; i < FDNS_NPTS; i++)
            {
                FOR (l = 0; l < k; l++)
                {
                    *x = L_shl(Mpy_32_16_1(*x, *gains), *gains_exp);
                    move32();
                    x++;
                }

                gains++;
                gains_exp++;
            }
        }
    }

}

void PsychAdaptLowFreqDeemph(Word32 x[],
                             const Word16 lpcGains[], const Word16 lpcGains_e[],
                             Word16 lf_deemph_factors[]
                            )
{
    Word16 i;
    Word16 max, max_e, fac, min, min_e, tmp, tmp_e;
    Word32 L_tmp;



    assert(lpcGains[0] >= 0x4000);

    max = lpcGains[0];
    move16();
    max_e = lpcGains_e[0];
    move16();
    min = lpcGains[0];
    move16();
    min_e = lpcGains_e[0];
    move16();

    /* find minimum (min) and maximum (max) of LPC gains in low frequencies */
    FOR (i = 1; i < 9; i++)
    {
        IF (compMantExp16Unorm(lpcGains[i], lpcGains_e[i], min, min_e) < 0)
        {
            min = lpcGains[i];
            move16();
            min_e = lpcGains_e[i];
            move16();
        }

        IF (compMantExp16Unorm(lpcGains[i], lpcGains_e[i], max, max_e) > 0)
        {
            max = lpcGains[i];
            move16();
            max_e = lpcGains_e[i];
            move16();
        }
    }

    min_e = add(min_e, 5); /* min *= 32.0f; */

    test();
    IF ((compMantExp16Unorm(max, max_e, min, min_e) < 0) && (min > 0))
    {
        /* fac = tmp = (float)pow(max / min, 0.0078125f); */
        tmp_e = min_e;
        move16();
        tmp = Inv16(min, &tmp_e);
        L_tmp = L_shl(L_mult(tmp, max), add(tmp_e, max_e)); /* Q31 */
        L_tmp = BASOP_Util_Log2(L_tmp); /* Q25 */
        L_tmp = L_shr(L_tmp, 7); /* 0.0078125f = 1.f/(1<<7) */
        L_tmp = BASOP_Util_InvLog2(L_tmp); /* Q31 */
        tmp = round_fx(L_tmp); /* Q15 */
        fac = tmp; /* Q15 */                                                        move16();

        /* gradual lowering of lowest 32 bins; DC is lowered by (max/tmp)^1/4 */
        FOR (i = 31; i >= 0; i--)
        {
            x[i] = Mpy_32_16_1(x[i], fac);
            move32();
            if (lf_deemph_factors != NULL)
            {
                lf_deemph_factors[i] = mult_r(lf_deemph_factors[i], fac);
                move16();
            }
            fac = mult_r(fac, tmp);
        }
    }

}

void AdaptLowFreqDeemph(Word32 x[], Word16 x_e,
                        Word16 tcx_lpc_shaped_ari,
                        Word16 lpcGains[], Word16 lpcGains_e[],
                        const Word16 lg,
                        Word16 lf_deemph_factors[]
                       )
{

    Word16 i, i_max, i_max_old, lg_4;
    Word32 v2, v4, tmp32;

    tmp32 = 0;  /* to avoid compilation warnings */


    IF (tcx_lpc_shaped_ari == 0)
    {
        v2 = L_shl(2, sub(31, x_e));    /* 2.0 */
        v4 = L_shl(v2, 1);              /* 4.0 */
        lg_4 = shr(lg, 2);              /* lg/4 */

        /* 1. find first magnitude maximum in lower quarter of spectrum */
        i_max = -1;
        move16();

        FOR (i = 0; i < lg_4; i++)
        {
            IF (L_sub(L_abs(x[i]), v4) >= 0)
            {

                /* Debug initialization to catch illegal x[i] values. */
                tmp32 = 0;

                if (x[i] < 0) tmp32 = L_add(x[i], v2);
                if (x[i] > 0) tmp32 = L_sub(x[i], v2);

                assert(tmp32 != 0);

                x[i] = tmp32;
                move32();
                i_max = i;
                move16();
                BREAK;
            }
        }

        /* 2. expand value range of all xi up to i_max: two extra steps */
        FOR (i = 0; i < i_max; i++)
        {
            x[i] = L_shr(x[i], 1);
            move32();
            lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
            move16();
        }

        /* 3. find first magnitude maximum in lower quarter of spectrum */
        i_max_old = i_max;
        move16();

        IF (i_max_old >= 0)
        {
            i_max = -1;
            move16();

            FOR (i = 0; i < lg_4; i++)
            {
                IF (L_sub(L_abs(x[i]), v4) >= 0)
                {
                    assert(x[i] != 0);
                    if (x[i] < 0) tmp32 = L_add(x[i], v2);
                    if (x[i] >= 0) tmp32 = L_sub(x[i], v2);
                    x[i] = tmp32;
                    move32();
                    i_max = i;
                    move16();
                    BREAK;
                }
            }
        }

        /* 4. expand value range of all xi up to i_max: two extra steps */
        FOR (i = 0; i < i_max; i++)
        {
            x[i] = L_shr(x[i], 1);
            move32();
            lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
            move16();
        }

        /* 5. always expand two lines; lines could be at index 0 and 1! */
        i_max = s_max(i_max, i_max_old);
        i = add(i_max, 1);

        IF (x[i] < 0)
        {
            tmp32 = L_sub(x[i], L_negate(v4));

            if (tmp32 > 0)
            {
                lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
                move16();
            }
            if (tmp32 <= 0)
            {
                x[i] = L_add(x[i], v2);
                move32();
            }
            if (tmp32 > 0)
            {
                x[i] = L_shr(x[i], 1);
                move32();
            }
        }
        ELSE
        {
            tmp32 = L_sub(x[i], v4);

            if (tmp32 < 0)
            {
                lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
                move16();
            }
            if (tmp32 >= 0)
            {
                x[i] = L_sub(x[i], v2);
                move32();
            }
            if (tmp32 < 0)
            {
                x[i] = L_shr(x[i], 1);
                move32();
            }
        }
        i = add(i, 1);

        IF (x[i] < 0)
        {
            tmp32 = L_sub(x[i], L_negate(v4));

            if (tmp32 > 0)
            {
                lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
                move16();
            }
            if (tmp32 <= 0)
            {
                x[i] = L_add(x[i], v2);
                move32();
            }
            if (tmp32 > 0)
            {
                x[i] = L_shr(x[i], 1);
                move32();
            }
        }
        ELSE
        {
            tmp32 = L_sub(x[i], v4);

            if (tmp32 < 0)
            {
                lf_deemph_factors[i] = shr(lf_deemph_factors[i], 1);
                move16();
            }
            if (tmp32 >= 0)
            {
                x[i] = L_sub(x[i], v2);
                move32();
            }
            if (tmp32 < 0)
            {
                x[i] = L_shr(x[i], 1);
                move32();
            }
        }
    }
    ELSE   /*if(!tcx_lpc_shaped_ari)*/
    {
        PsychAdaptLowFreqDeemph(x, lpcGains, lpcGains_e, lf_deemph_factors);
    }/*if(!tcx_lpc_shaped_ari)*/

}

void tcx_noise_filling(
    Word32 *Q,
    Word16 Q_e,
    Word16 seed,
    Word16 iFirstLine,
    Word16 lowpassLine,
    Word16 nTransWidth,
    Word16 L_frame,
    Word16 tiltCompFactor,
    Word16 fac_ns,
    Word16 *infoTCXNoise
)
{
    Word16 i, m, segmentOffset;
    Word16 win; /* window coefficient */
    Word16 tilt_factor;
    Word32 nrg;
    Word16 tmp1, tmp2, s;
    Word32 tmp32;


    /* get inverse frame length */
    tmp1 = getInvFrameLen(L_frame);

    /* tilt_factor = (float)pow(max(0.375f, tiltCompFactor), 1.0f/(float)L_frame); */
    tmp32 = BASOP_Util_Log2(L_deposit_h(s_max(0x3000, tiltCompFactor))); /* 6Q25 */
    tmp32 = L_shr(Mpy_32_16_1(tmp32, tmp1), 6);
    BASOP_SATURATE_WARNING_OFF;
    tilt_factor = round_fx(BASOP_Util_InvLog2(tmp32));
    BASOP_SATURATE_WARNING_ON;

    /* find last nonzero line below iFirstLine, use it as start offset */
    tmp1 = shr(iFirstLine, 1);
    FOR (i = iFirstLine; i > tmp1; i--)
    {
        IF (Q[i] != 0)
        {
            BREAK;
        }
    }
    /* fac_ns *= (float)pow(tilt_factor, (float)i); */
    FOR (m = 0; m < i; m++)
    {
        fac_ns = mult_r(fac_ns, tilt_factor);
    }

    nrg = L_deposit_l(1);
    win = 0;
    move16();
    i = add(i, 1);
    segmentOffset = i;
    move16();

    FOR (; i < lowpassLine; i++)
    {
        fac_ns = mult_r(fac_ns, tilt_factor);

        IF (Q[i] != 0)
        {
            IF (win > 0)
            {
                /* RMS-normalize current noise-filled segment */
                tmp1 = BASOP_Util_Divide3216_Scale(nrg, sub(i, segmentOffset), &s); /* mean */
                s = add(s, 9-15); /* scaling */
                tmp1 = ISqrt16(tmp1, &s); /* 1/RMS */
                tmp1 = mult_r(tmp1, inv_int[nTransWidth]); /* compensate win */
                s = add(s, sub(16, Q_e)); /* scaling */

                tmp2 = sub(i, win);
                IF (sub(segmentOffset, tmp2) < 0)
                {
                    FOR (m = segmentOffset; m < tmp2; m++)
                    {
                        Q[m] = L_shl(Mpy_32_16_1(Q[m], tmp1), s);
                        move32();
                    }
                }

                tmp2 = mult(tmp1, inv_int[nTransWidth]);
                tmp1 = extract_l(L_mult0(tmp2, win));
                FOR (m = sub(i, win); m < i; m++)
                {
                    Q[m] = L_shl(Mpy_32_16_1(Q[m], tmp1), s);
                    move32();
                    win = sub(win, 1);
                    tmp1 = sub(tmp1, tmp2);
                }

                nrg = L_deposit_l(1); /* start new segment: reset noise segment energy */
            }
            segmentOffset = add(i, 1);
        }
        ELSE   /* line is zero, so fill line and update window and energy */
        {
            if (sub(win, nTransWidth) < 0)
            {
                win = add(win, 1);
            }

            seed = own_random2_fx(seed);
            Q[i] = L_mult0(mult(seed, fac_ns), win);
            move32();

            tmp1 = shr(seed, 4);
            nrg = L_mac0(nrg, tmp1, tmp1);   /* sum up energy of current noise segment */

            if(infoTCXNoise)   /* set noiseflags for IGF */
            {
                infoTCXNoise[i] = 1;
                move16();
            }
        }
    }

    IF (win > 0)
    {
        /* RMS-normalize uppermost noise-filled segment */
        tmp1 = BASOP_Util_Divide3216_Scale(nrg, sub(lowpassLine, segmentOffset), &s); /* mean */
        s = add(s, 9-15); /* compensate energy scaling */
        tmp1 = ISqrt16(tmp1, &s); /* 1/RMS */
        tmp1 = mult_r(tmp1, inv_int[nTransWidth]); /* compensate win */
        s = add(s, sub(16, Q_e)); /* compensate noise scaling */

        FOR (m = segmentOffset; m < lowpassLine; m++)
        {
            Q[m] = L_shl(Mpy_32_16_1(Q[m], tmp1), s);
            move32();
        }
    }

}



/*---------------------------------------------------------------
 * Residual Quantization
 *--------------------------------------------------------------*/


void InitTnsConfigs(Word32 nSampleRate, Word16 L_frame,
                    STnsConfig tnsConfig[2][2]
                    ,Word16 igfStopFreq
                    ,Word32 bitrate
                   )
{
    IF (L_sub(bitrate,ACELP_32k) > 0)
    {
        InitTnsConfiguration(nSampleRate, shr(L_frame,1), &tnsConfig[0][0], igfStopFreq, bitrate);
    }
    InitTnsConfiguration(nSampleRate, L_frame,   &tnsConfig[1][0], igfStopFreq, bitrate);
    InitTnsConfiguration(nSampleRate, add(L_frame, shr(L_frame,2)), &tnsConfig[1][1], igfStopFreq, bitrate);
}


void SetTnsConfig(TCX_config * tcx_cfg, Word8 isTCX20, Word8 isAfterACELP)
{
    move16();
    tcx_cfg->pCurrentTnsConfig = &tcx_cfg->tnsConfig[isTCX20][isAfterACELP];
    assert(tcx_cfg->pCurrentTnsConfig != NULL);
}


void tcx_get_gain(Word32 *x,        /* i: spectrum 1 */
                  Word16 x_e,       /* i: spectrum 1 exponent */
                  Word32 *y,        /* i: spectrum 2 */
                  Word16 y_e,       /* i: spectrum 2 exponent */
                  Word16 n,         /* i: length */
                  Word16 *gain,     /* o: gain */
                  Word16 *gain_e,   /* o: gain exponent */
                  Word32 *en_y,     /* o: energy of y (optional) */
                  Word16 *en_y_e    /* o: energy of y exponent (optional) */
                 )
{
    Word32 maxX, minX, maxY, minY;
    Word32 corr, ener;
    Word16 sx, sy, corr_e, ener_e;
    Word16 i, tmp;


    maxX = L_deposit_l(1);
    maxY = L_deposit_l(1);
    minX = L_deposit_l(-1);
    minY = L_deposit_l(-1);
    FOR (i = 0; i < n; i++)
    {
        if (x[i] > 0) maxX = L_max(maxX, x[i]);
        if (x[i] < 0) minX = L_min(minX, x[i]);

        if (y[i] > 0) maxY = L_max(maxY, y[i]);
        if (y[i] < 0) minY = L_min(minY, y[i]);
    }
    sx = s_min(norm_l(maxX), norm_l(minX));
    sy = s_min(norm_l(maxY), norm_l(minY));
    sx = sub(sx, 4);
    sy = sub(sy, 4);

    ener = L_deposit_l(0);
    corr = L_deposit_l(0);
    FOR (i = 0; i < n; i++)
    {
        tmp = round_fx(L_shl(y[i], sy));
        ener = L_mac0(ener, tmp, tmp);
        corr = L_mac0(corr, tmp, round_fx(L_shl(x[i], sx)));
    }

    if (ener == 0) ener = L_deposit_l(1);

    ener_e = add(shl(sub(y_e, sy), 1), 1);
    corr_e = add(sub(add(x_e, y_e), add(sx, sy)), 1);

    tmp = sub(norm_l(corr), 1);
    corr = L_shl(corr, tmp);
    corr_e = sub(corr_e, tmp);

    tmp = norm_l(ener);
    ener = L_shl(ener, tmp);
    ener_e = sub(ener_e, tmp);

    tmp = div_s(abs_s(round_fx(corr)), round_fx(ener));
    if (corr < 0) tmp = negate(tmp);

    *gain = tmp;
    move16();
    *gain_e = sub(corr_e, ener_e);
    move16();

    if (en_y != NULL)
    {
        *en_y = ener;
        move32();
    }
    if (en_y_e != NULL)
    {
        *en_y_e = ener_e;
        move16();
    }

}

void init_TCX_config(TCX_config *tcx_cfg,
                     Word16 L_frame,
                     Word16 fscale
                     ,Word16 L_frameTCX
                     ,Word16 fscaleFB
                    )
{
    /* Initialize the TCX MDCT windows */
    tcx_cfg->tcx_mdct_window_length = extract_l(L_shr(L_mult0(L_LOOK_12k8, fscale), LD_FSCALE_DENOM));
    tcx_cfg->tcx_mdct_window_delay = tcx_cfg->tcx_mdct_window_length;
    move16();

    tcx_cfg->tcx_mdct_window_half_length = extract_l(L_shr(L_mult0(L_LOOK_12k8 - NS2SA(12800, 5000000L), fscale), LD_FSCALE_DENOM));

    tcx_cfg->tcx_mdct_window_min_length = shr(L_frame, 4); /* 1.25ms */
    tcx_cfg->tcx_mdct_window_trans_length = shr(L_frame, 4); /* 1.25ms */

    tcx_cfg->tcx5Size = shr(L_frame, 2); /* 5ms */

    tcx_cfg->tcx_mdct_window_lengthFB = extract_l(L_shr(L_mult0(L_LOOK_12k8, fscaleFB), LD_FSCALE_DENOM));
    tcx_cfg->tcx_mdct_window_delayFB = tcx_cfg->tcx_mdct_window_lengthFB;
    move16();

    tcx_cfg->tcx_mdct_window_half_lengthFB = extract_l(L_shr(L_mult0(L_LOOK_12k8 - NS2SA(12800, 5000000L), fscaleFB), LD_FSCALE_DENOM));

    tcx_cfg->tcx_mdct_window_min_lengthFB = shr(L_frameTCX, 4); /* 1.25ms */
    tcx_cfg->tcx_mdct_window_trans_lengthFB = shr(L_frameTCX, 4); /* 1.25ms */

    tcx_cfg->tcx5SizeFB = shr(L_frameTCX, 2); /* 5ms */

    mdct_window_sine( &tcx_cfg->tcx_mdct_window, tcx_cfg->tcx_mdct_window_length );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_half, tcx_cfg->tcx_mdct_window_half_length );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_minimum, tcx_cfg->tcx_mdct_window_min_length );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_trans, tcx_cfg->tcx_mdct_window_trans_length );

    mdct_window_sine( &tcx_cfg->tcx_mdct_windowFB, tcx_cfg->tcx_mdct_window_lengthFB );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_halfFB, tcx_cfg->tcx_mdct_window_half_lengthFB );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_minimumFB, tcx_cfg->tcx_mdct_window_min_lengthFB );
    mdct_window_sine( &tcx_cfg->tcx_mdct_window_transFB, tcx_cfg->tcx_mdct_window_trans_lengthFB );

    /*ALDO windows for MODE2*/
    mdct_window_aldo(tcx_cfg->tcx_aldo_window_1, tcx_cfg->tcx_aldo_window_1_trunc, tcx_cfg->tcx_aldo_window_2, L_frame);
    mdct_window_aldo(tcx_cfg->tcx_aldo_window_1_FB, tcx_cfg->tcx_aldo_window_1_FB_trunc, tcx_cfg->tcx_aldo_window_2_FB, L_frameTCX);
}

