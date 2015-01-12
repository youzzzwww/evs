/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "options.h"

#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"
#define inv_int InvIntTable



Word16 tcxGetNoiseFillingTilt(Word16 A[], Word16 lpcorder, Word16 L_frame, Word16 mode, Word16 *noiseTiltFactor)
{
    Word16 firstLine;
    Word32 tmp;
    Word16 As[M+1];


    IF (mode != 0)
    {
        firstLine = idiv1616U(L_frame, 6);
        *noiseTiltFactor = FL2WORD16(0.5625f);
        move16();
    }
    ELSE
    {
        firstLine = shr(L_frame, 3);

        Copy_Scale_sig( A, As, lpcorder+1, sub(norm_s(A[0]),2) );
        tmp = get_gain(As+1, As, lpcorder);
        BASOP_SATURATE_WARNING_OFF;
        *noiseTiltFactor = add(round_fx(L_shl(tmp, 15)), FL2WORD16(0.09375f));
        move16();
        BASOP_SATURATE_WARNING_ON;
    }


    return firstLine;
}


void tcxFormantEnhancement(
    Word16 xn_buf[],
    Word16 gainlpc[], Word16 gainlpc_e[],
    Word32 spectrum[], Word16 *spectrum_e,
    Word16 L_frame,
    Word16 L_frameTCX
)
{
    Word16 i, j, k, l, n;
    Word16 fac, fac0, fac1, fac_e, d, tmp;
    Word16 xn_buf_e, xn_one, m, e;


    k = shr(L_frame, 6); /* FDNS_NPTS = 64 */
    l = 0;
    move16();

    /* get exponent */
    xn_buf_e = 0;
    move16();
    FOR (i = 0; i < FDNS_NPTS; i++)
    {
        xn_buf_e = s_max(xn_buf_e, gainlpc_e[i]);
    }
    xn_buf_e = shr(add(xn_buf_e, 1), 1);    /* max exponent after sqrt */
    xn_one = shr(0x4000, sub(xn_buf_e, 1)); /* 1.0 scaled to xn_buf_e */

    /* Formant enhancement via square root of the LPC gains */
    e = gainlpc_e[0];
    move16();
    m = Sqrt16(gainlpc[0], &e);
    xn_buf[0] = shl(m, sub(e, xn_buf_e));
    move16();

    e = gainlpc_e[1];
    move16();
    m = Sqrt16(gainlpc[1], &e);
    xn_buf[1] = shl(m, sub(e, xn_buf_e));
    move16();

    fac0 = s_min(xn_buf[0], xn_buf[1]);
    fac_e = xn_buf_e;
    move16();
    fac0 = Inv16(fac0, &fac_e);

    FOR (i = 1; i < FDNS_NPTS-1; i++)
    {
        e = gainlpc_e[i+1];
        move16();
        m = Sqrt16(gainlpc[i+1], &e);
        xn_buf[i+1] = shl(m, sub(e, xn_buf_e));
        move16();

        test();
        IF ((sub(xn_buf[i-1], xn_buf[i]) <= 0) && (sub(xn_buf[i+1], xn_buf[i]) <= 0))
        {
            m = s_max(xn_buf[i-1], xn_buf[i+1]);
            e = xn_buf_e;
            move16();
            m = Inv16(m, &e);

            fac1 = m;
            move16();
            tmp = sub(e, fac_e);

            if (tmp > 0) fac0 = shr(fac0, tmp);
            if (tmp < 0) fac1 = shl(fac1, tmp);

            if (tmp > 0)
            {
                fac_e = e;
                move16();
            }

            d = sub(fac1, fac0);
            n = sub(i, l);
            assert(n <= 64);

            xn_buf[l] = xn_one;
            move16();
            FOR (j = 1; j < n; j++)
            {
                fac = add(fac0, mult(d, extract_l(L_mult0(j, inv_int[n]))));
                BASOP_SATURATE_WARNING_OFF;
                xn_buf[l+j] = s_min(xn_one, shl(mult(xn_buf[l+j], fac), fac_e));
                move16();
                BASOP_SATURATE_WARNING_ON;
            }

            l = i;
            move16();

            fac0 = m;
            move16();
            fac_e = e;
            move16();
        }
    }
    /* i = FDNS_NPTS - 1; Completing changes to gains */
    m = s_min(xn_buf[i-1], xn_buf[i]);
    e = xn_buf_e;
    move16();
    m = Inv16(m, &e);

    fac1 = m;
    move16();
    tmp = sub(e, fac_e);

    if (tmp > 0) fac0 = shr(fac0, tmp);
    if (tmp < 0) fac1 = shl(fac1, tmp);

    if (tmp > 0)
    {
        fac_e = e;
        move16();
    }

    d = sub(fac1, fac0);
    n = sub(i, l);
    assert(n <= 64);

    xn_buf[l] = xn_one;
    move16();
    FOR (j = 1; j < n; j++)
    {
        fac = add(fac0, mult(d, extract_l(L_mult0(j, inv_int[n]))));
        BASOP_SATURATE_WARNING_OFF;
        xn_buf[l+j] = s_min(xn_one, shl(mult(xn_buf[l+j], fac), fac_e));
        move16();
        BASOP_SATURATE_WARNING_ON;
    }

    xn_buf[i] = xn_one;
    move16();

    /* Application of changed gains onto decoded MDCT lines */
    FOR (i = 0; i < L_frame; i += k)
    {
        FOR (l = 0; l < k; l++)
        {
            *spectrum = Mpy_32_16_1(*spectrum, *xn_buf);
            move32();
            spectrum++;
        }
        xn_buf++;
    }

    tmp = sub(L_frameTCX, L_frame);
    FOR (i = 0; i < tmp; i++)
    {
        spectrum[i] = L_shr(spectrum[i], xn_buf_e);
        move32();
    }
    *spectrum_e = add(*spectrum_e, xn_buf_e);
    move16();

}

void tcxInvertWindowGrouping(TCX_config *tcx_cfg,
                             Word32 xn_buf[],
                             Word32 spectrum[],
                             Word16 L_frame,
                             Word8 fUseTns,
                             Word16 last_core,
                             Word16 index,
                             Word16 frame_cnt,
                             Word16 bfi)
{
    Word16 i, L_win, L_spec;
    Word32 *p;


    L_win = shr(L_frame, 1);
    L_spec = tcx_cfg->tnsConfig[0][0].iFilterBorders[0];
    move16();

    test();
    test();
    if ((frame_cnt != 0) && (bfi == 0) && (last_core != ACELP_CORE))   /* fix sub-window overlap */
    {
        tcx_cfg->tcx_last_overlap_mode = tcx_cfg->tcx_curr_overlap_mode;
        move16();
    }
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF (((bfi==0) &&((tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) ||
                     ((tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP) && (frame_cnt == 0) && (index == 0))))
        ||
        ((bfi!=0) &&((tcx_cfg->tcx_last_overlap_mode != FULL_OVERLAP) &&
                     !(tcx_cfg->tcx_curr_overlap_mode == FULL_OVERLAP))))
    {
        /* ungroup sub-windows: deinterleave MDCT bins into separate windows */
        p = xn_buf;
        FOR (i = 1; i < L_win; i += 2)
        {
            *p++ = spectrum[i];
            move32();
        }

        p = spectrum;
        FOR (i = 0; i < L_frame; i += 2)
        {
            *p++ = spectrum[i];
            move32();
        }

        p = spectrum + L_frame - 1;
        FOR (i = sub(L_frame, 1); i > L_win; i -= 2)
        {
            *p-- = spectrum[i];
            move32();
        }
        Copy32(xn_buf, spectrum + L_win, shr(L_win, 1));

        test();
        test();
        IF ((tcx_cfg->fIsTNSAllowed != 0) && (bfi == 0) && (fUseTns != 0))
        {
            /* rearrange LF sub-window lines prior to TNS synthesis filtering */
            IF (sub(L_spec, L_frame) < 0)
            {
                Copy32(spectrum+8, spectrum+16, sub(shr(L_spec,1),8));
                Copy32(spectrum+L_frame/2, spectrum+8, 8);
                Copy32(spectrum+L_frame/2+8, spectrum+L_spec/2+8, sub(shr(L_spec,1),8));
            }
            ELSE
            {
                Copy32(spectrum+L_win, xn_buf, 8);
                Copy32(spectrum+8, spectrum+16, sub(L_win, 8));
                Copy32(xn_buf, spectrum+8, 8);
            }
        }
    }

}


