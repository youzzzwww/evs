/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/



#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"



Word16 bass_pf_enc(
    Word16 *orig,               /* (i) : 12.8kHz original signal                      Q0 */
    Word16 *syn,                /* (i) : 12.8kHz synthesis to postfilter              Q0 */
    Word16 *T_sf,               /* (i) : Pitch period for all subframes (T_sf[16])    Q0 */
    Word16 *gainT_sf,           /* (i) : Pitch gain for all subframes (gainT_sf[16])  Q14 */
    Word16 l_frame,             /* (i) : frame length (should be multiple of l_subfr) Q0 */
    Word16 l_subfr,             /* (i) : sub-frame length (60/64)                     Q0 */
    Word16 *gain_factor_param,  /* (o) : quantized gain factor                        Q0 */
    Word16 mode,                /* (i) : coding mode of adapt bpf                        */
    struct MEM_BPF *mem_bpf     /* i/o : memory state                                    */
)
{
    Word16 i, j, sf, i_subfr, T, lg, l_filt;
    Word16 gain, d, tmp16, hr, s1, s2, s2_old, s3, s4, st, st2, st3;
    Word32 nrg, tmp, nrg1, nrg2, n, snr, lp_error, tmp32;
    Word16 noise_buf[L_FILT16k+2*L_SUBFR], *noise, *noise_in;
    Word16 error_buf[L_FILT16k+2*L_SUBFR], *error, *error_in;
    Word32 cross_n_d, nrg_n;
    const Word16 *pFilt;
    Word32 ener2;


    IF (sub(l_frame, L_FRAME16k) != 0)
    {
        pFilt = filt_lp;
        l_filt = L_FILT;
        move16();
    }
    ELSE
    {
        pFilt = filt_lp_16kHz;
        l_filt = L_FILT16k;
        move16();
    }

    noise = noise_buf + l_filt;
    noise_in = noise_buf + shl(l_filt, 1);
    error = error_buf;
    error_in = error_buf + l_filt;

    sf = 0;
    move16();
    snr = L_deposit_l(0);
    nrg_n = L_deposit_l(0);
    cross_n_d = L_deposit_l(0);
    lp_error = L_shl(mem_bpf->lp_error, 0);
    s2_old = mem_bpf->noise_shift_old;
    move16();
    s3 = s4 = 0;            /* initialization of s3 and s4 to suppress compiler warnings;
                             s3 and s4 get initialized for i_subfr == 0                */

    nrg1 = nrg2 = 0;        /* initialization fo nrg1 and nrg2 to suppress compiler warnings;
                             nrg1 and nrg1 get initialized for i_subfr == 0            */

    FOR (i_subfr = 0; i_subfr < l_frame; i_subfr += l_subfr)
    {
        T = T_sf[sf];
        move16();

        lg = sub(sub(l_frame, T), i_subfr);
        if (lg < 0)
        {
            lg = 0;
            move16();
        }
        if (sub(lg, l_subfr) > 0)
        {
            lg = l_subfr;
            move16();
        }

        IF (gainT_sf[sf] > 0)
        {
            /* get headroom for used part of syn */
            tmp16 = s_max(add(lg, shl(T, 1)), add(l_subfr, T));
            hr = getScaleFactor16(syn + sub(i_subfr, T), tmp16);
            s1 = sub(hr, 3);

            tmp = L_deposit_l(1);
            nrg = L_deposit_l(1);

            IF (lg > 0)
            {
                FOR (i = 0; i < lg; i++)
                {
                    tmp32 = L_mult(syn[i+i_subfr-T], 0x4000);
                    tmp32 = L_mac(tmp32, syn[i+i_subfr+T], 0x4000);
                    tmp16 = round_fx(L_shl(tmp32, s1)); /* Q0+s1 */

                    tmp = L_mac0(tmp, shl(syn[i+i_subfr], s1), tmp16); /* Q0+2*s1 */
                    nrg = L_mac0(nrg, tmp16, tmp16); /* Q0+2*s1 */
                }
            }

            IF (sub(lg, l_subfr) < 0)
            {
                FOR (i = lg; i < l_subfr; i++)
                {
                    tmp16 = shl(syn[i+i_subfr-T], s1); /* Q0+s1 */
                    tmp = L_mac0(tmp, shl(syn[i+i_subfr], s1), tmp16); /* Q0+2*s1 */
                    nrg = L_mac0(nrg, tmp16, tmp16); /* Q0+2*s1 */
                }
            }

            /* gain = tmp/nrg; */
            gain = BASOP_Util_Divide3232_Scale(tmp, nrg, &tmp16);
            BASOP_SATURATE_WARNING_OFF;
            gain = shl(gain, tmp16); /* Q15 */
            BASOP_SATURATE_WARNING_ON;

            if (gain < 0)
            {
                gain = 0;
                move16();
            }

            st = sub(norm_l(lp_error), 3);
            test();
            if ((sub(st, s1) < 0) && (lp_error != 0))
            {
                s1 = st;
                move16();
            }

            ener2 = L_deposit_l(0);

            IF (lg > 0)
            {
                FOR (i = 0; i < lg; i++)
                {
                    tmp32 = L_msu0(0, gain, syn[i+i_subfr-T]);
                    tmp32 = L_msu0(tmp32, gain, syn[i+i_subfr+T]);
                    tmp16 = mac_r(tmp32, gain, syn[i+i_subfr]); /* Q0 */

                    lp_error = Mpy_32_16_1(lp_error, FL2WORD16(0.9f));
                    lp_error = L_mac(lp_error, tmp16, 0x1000);  /* Q13 */

                    tmp16 = round_fx(L_shl(lp_error, s1)); /* Q0+s1-3 */
                    ener2 = L_mac0(ener2, tmp16, tmp16); /* Q0+(s1-3)*2 */
                }
            }

            IF (sub(lg, l_subfr) < 0)
            {
                FOR (i = lg; i < l_subfr; i++)
                {
                    tmp32 = L_mult0(gain, syn[i+i_subfr]);
                    tmp32 = L_msu0(tmp32, gain, syn[i+i_subfr-T]); /* Q0 */
                    tmp16 = round_fx(tmp32);

                    lp_error = Mpy_32_16_1(lp_error, FL2WORD16(0.9f));
                    lp_error = L_mac(lp_error, tmp16, 0x1000);  /* Q13 */

                    tmp16 = round_fx(L_shl(lp_error, s1)); /* Q0+s1-3 */
                    ener2 = L_mac0(ener2, tmp16, tmp16); /* Q0+(s1-3)*2 */
                }
            }

            st = shl(sub(s1, 3), 1);

            IF (ener2 > 0)
            {
                ener2 = L_shr(BASOP_Util_Log2(ener2), 9); /* 15Q16 */
                ener2 = L_add(ener2, L_deposit_h(sub(31, st)));
            }
            ELSE
            {
                ener2 = 0xFFF95B2C; /* log2(0.01) (15Q16) */                            move32();
            }

            mem_bpf->lp_error_ener = L_add(Mpy_32_16_1(L_sub(mem_bpf->lp_error_ener, ener2), FL2WORD16(0.99f)), ener2); /* 15Q16 */

            st = add(st, 6);
            ener2 = L_sub(mem_bpf->lp_error_ener, L_deposit_h(sub(31, st)));
            IF (ener2 >= 0)
            {
                tmp16 = add(extract_h(ener2), 1);
                ener2 = L_sub(ener2, L_deposit_h(tmp16));
                tmp = L_shr(tmp, tmp16);
                nrg = L_shr(nrg, tmp16);
            }
            ener2 = BASOP_Util_InvLog2(L_shl(ener2, 9));  /* Q0+2*s1 */

            tmp32 = L_add(L_shr(nrg, 1), L_shr(ener2, 1));
            if (tmp32 == 0) tmp32 = L_deposit_l(1);
            tmp16 = BASOP_Util_Divide3232_Scale(tmp, tmp32, &st);
            BASOP_SATURATE_WARNING_OFF;
            tmp16 = shl(tmp16, sub(st, 2)); /* Q15 */

            if (sub(tmp16, FL2WORD16(0.5f)) > 0)
            {
                tmp16 = FL2WORD16(0.5f);
                move16();
            }
            if (tmp16 < 0)
            {
                tmp16 = 0;
                move16();
            }
            BASOP_SATURATE_WARNING_ON;

            s2 = hr;
            move16();

            IF (lg > 0)
            {
                FOR (i = 0; i < lg; i++)
                {
                    tmp32 = L_msu0(0, tmp16, syn[i+i_subfr-T]);
                    tmp32 = L_msu0(tmp32, tmp16, syn[i+i_subfr+T]);
                    tmp32 = L_mac(tmp32, tmp16, syn[i+i_subfr]);
                    noise_in[i] = round_fx(L_shl(tmp32, s2)); /* Q0+s2 */

                    error_in[i] = sub(orig[i+i_subfr], syn[i+i_subfr]); /*Q0*/            move16();
                }
            }

            IF (sub(lg, l_subfr) < 0)
            {
                FOR (i = lg; i < l_subfr; i++)
                {
                    tmp32 = L_mult0(tmp16, syn[i+i_subfr]);
                    tmp32 = L_msu0(tmp32, tmp16, syn[i+i_subfr-T]);
                    noise_in[i] = round_fx(L_shl(tmp32, s2)); /* Q0+s2 */

                    error_in[i] = sub(orig[i+i_subfr], syn[i+i_subfr]); /*Q0*/            move16();
                }
            }
        }
        ELSE
        {
            set16_fx(noise_in, 0, l_subfr);
            set16_fx(error_in, 0, l_subfr);
            s2 = s2_old;
        }

        tmp16 = shl(l_filt, 1);

        /* copy history buffers (rescale noise history to new exponent) */
        st = sub(s2, s2_old);
        FOR (i = 0; i < tmp16; i++)
        {
            noise_buf[i] = shl(mem_bpf->noise_buf[i], st);
            move16();
        }
        Copy(noise_buf+l_subfr, mem_bpf->noise_buf, tmp16);
        s2_old = s2;
        move16();

        Copy(mem_bpf->error_buf, error_buf, l_filt);
        Copy(error_buf+l_subfr, mem_bpf->error_buf, l_filt);

        /* get noise shift */
        st = getScaleFactor16(noise-l_filt, add(l_subfr, shl(l_filt, 1)));
        st = add(sub(st, 3), s2);
        if (i_subfr == 0)
        {
            s3 = st;
            move16();
        }
        tmp16 = sub(st, s3);
        IF (tmp16 < 0)
        {
            nrg_n = L_shl(nrg_n, s_max(-15, shl(tmp16, 1)));
            cross_n_d = L_shl(cross_n_d, tmp16);
            s3 = st;
            move16();
        }

        /* get error shift */
        st = getScaleFactor16(error, l_subfr);
        st = sub(st, 3);
        if (i_subfr == 0)
        {
            s4 = st;
            move16();
        }
        tmp16 = sub(st, s4);
        IF (tmp16 < 0)
        {
            cross_n_d = L_shl(cross_n_d, tmp16);
            nrg1 = L_shl(nrg1, shl(tmp16, 1));
            nrg2 = L_shl(nrg2, shl(tmp16, 1));
            s4 = st;
            move16();
        }

        nrg1 = L_deposit_l(1);
        nrg2 = L_deposit_l(1);

        /* substract from voiced speech low-pass filtered noise */
        st = sub(s_min(s3, s4), 1);
        st2 = sub(s3, s2);
        st3 = sub(st,s2);
        FOR (i = 0; i < l_subfr; i++)
        {
            n = L_mult(pFilt[0], noise[i]); /* Q16+s2 */

            FOR (j = 1; j <= l_filt; j++)
            {
                n = L_mac(n, pFilt[j], noise[i-j]);
                n = L_mac(n, pFilt[j], noise[i+j]);
            }

            /*for optimal g*/
            tmp16 = round_fx(L_shl(n, st2)); /* Q0+s3 */
            d = shl(error[i], s4); /* Q0+s4 */
            nrg_n = L_mac0(nrg_n, tmp16, tmp16); /* Q0+2*s3 */
            cross_n_d = L_mac0(cross_n_d, tmp16, d); /* Q0+s3+s4 */

            /*for evaluating SNR*/
            tmp16 = round_fx(L_shl(n, st3)); /* Q0+st */
            tmp16 = add(tmp16, shl(error[i], st)); /* Q0+st */
            nrg1 = L_mac0(nrg1, tmp16, tmp16); /* Q0+2*st */
            nrg2 = L_mac0(nrg2, d, d); /* Q0+2*s4 */
        }

        /*SegSNR*/
        snr = L_add(snr, L_shr(L_sub(BASOP_Util_Log2(nrg2), BASOP_Util_Log2(nrg1)), 9)); /* 15Q16 */
        snr = L_add(snr, L_deposit_h(shl(sub(st, s4), 1)));

        sf = add(sf, 1);
    }

    if (nrg_n == 0)
    {
        nrg_n = L_deposit_l(1);
    }

    /*Compute and quantize optimal gain*/
    /* optimal gain = -<n,d>/<n,n> */
    *gain_factor_param = 2;
    move16();
    IF (sub(mode, 2) == 0)
    {
        /* *gain_factor_param = (int)(-2.f*(cross_n_d/nrg_n)+0.5f); */
        tmp16 = BASOP_Util_Divide3232_Scale(cross_n_d, nrg_n, &st); /* Q15-st-s3+s4 */
        BASOP_SATURATE_WARNING_OFF;
        tmp16 = shl(negate(tmp16), add(sub(add(st, s3), s4), 1-14)); /* Q1 */
        tmp16 = shr(add(tmp16, 1), 1); /* Q0 */
        BASOP_SATURATE_WARNING_ON;

        *gain_factor_param = tmp16;
        move16();
        if (sub(tmp16, 3) > 0)
        {
            *gain_factor_param = 3;
            move16();
        }
        if (tmp16 < 0)
        {
            *gain_factor_param = 0;
            move16();
        }

        /*If optimal gain negatif or zero but snr still positif->gain=0.5f*/
        test();
        if (snr > 0 && *gain_factor_param == 0)
        {
            *gain_factor_param = 1;
            move16();
        }
    }

    mem_bpf->lp_error = lp_error;
    move32();
    mem_bpf->noise_shift_old = s2_old;
    move16();


    return 0;
}

