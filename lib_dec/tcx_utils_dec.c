/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "stl.h"
#include "options.h"
#include "prot_fx.h"
#include "rom_basop_util.h"
#include "basop_util.h"
#include "rom_com_fx.h"

void tcx_decoder_memory_update(
    Word16 *xn_buf,         /* i/o: mdct output buffer used also as temporary buffer */
    Word16 *synthout,       /* o: synth                                              */
    Word16 L_frame_glob,    /* i: global frame length                                */
    Word16 *A,              /* i: Quantized LPC coefficients                         */
    Decoder_State_fx *st,
    Word16 *syn,            /* o: st->syn                                            */
    Word8 fb                /* i: fullband flag                                      */
)
{
    Word16 tmp;
    Word16 *synth;
    Word16 buf[1+M+LFAC+L_FRAME_PLUS];
    Word16 preemph;


    preemph = st->preemph_fac;
    move16();

    /* Output synth */
    Copy(xn_buf, synthout, L_frame_glob);

    /* Update synth */

    synth = buf + M+1;
    Copy(syn, buf, M+1);
    Copy(xn_buf, synth, L_frame_glob);
    Copy(synth + sub(L_frame_glob, M+1), syn, M+1);


    IF (!fb)
    {

        /* Emphasis of synth -> synth_pe */
        tmp = synth[-M-1];
        move16();
        st->Q_syn = E_UTIL_f_preemph3(synth - M, preemph, add(M, L_frame_glob), &tmp, 1);
        st->prev_Q_syn = st->Q_syn = st->Q_syn - 1;
        Copy(synth + sub(L_frame_glob, M), st->mem_syn2_fx, M);
        Copy(synth + sub(L_frame_glob, L_SYN_MEM), st->mem_syn_r, L_SYN_MEM);

        test();
        IF ( st->tcxonly == 0 || sub(L_frame_glob,L_FRAME16k)<=0)
        {
            /* Update excitation */
            IF(sub(st->Q_syn+1,st->Q_exc) != 0)
            {
                Scale_sig(st->old_exc_fx, L_EXC_MEM_DEC, sub(st->Q_syn+1,st->Q_exc));
            }
            st->Q_exc = st->Q_syn + 1;
            st->Q_subfr[0] = st->Q_subfr[1] = st->Q_subfr[2] = st->Q_subfr[3] = st->Q_subfr[4] = st->Q_exc;

            IF (sub(L_frame_glob, L_EXC_MEM_DEC) < 0)
            {
                Copy( st->old_exc_fx + L_frame_glob, st->old_exc_fx, sub(L_EXC_MEM_DEC, L_frame_glob) );
                Residu3_fx(A, synth, st->old_exc_fx + sub(L_EXC_MEM_DEC, L_frame_glob), L_frame_glob, 1);
            }
            ELSE
            {
                Residu3_fx(A, synth + sub(L_frame_glob, L_EXC_MEM_DEC), st->old_exc_fx, L_EXC_MEM_DEC, 1);
            }
        }

        /* Update old_Aq */
        Copy(A, st->old_Aq_12_8_fx, M+1);
    }

}

/* Returns: number of bits used (including "bits") */
Word16 tcx_ari_res_invQ_spec(
    Word32 x_Q[],           /* i/o: quantized spectrum                Q31-e */
    Word16 x_Q_e,           /* i: quantized spectrum exponent         Q0 */
    Word16 L_frame,         /* i: number of lines                     Q0 */
    const Word16 prm[],     /* i: bit-stream                          Q0 */
    Word16 target_bits,     /* i: number of bits available            Q0 */
    Word16 bits,            /* i: number of bits used so far          Q0 */
    Word16 deadzone,        /* i: quantizer deadzone                  Q15 */
    const Word16 x_fac[]    /* i: spectrum post-quantization factors  Q14 */
)
{
    Word16 i, j, num_zeros;
    Word16 zeros[L_FRAME_PLUS];
    Word16 fac_p, sign;
    Word32 L_tmp;
    Word16 s;


    /* Limit the number of residual bits */
    target_bits = s_min(target_bits, NPRM_RESQ);


    /* Requantize the spectrum line-by-line */
    /* fac_m = deadzone * 0.5f; */
    num_zeros = 0;
    move16();

    FOR (i=0; i < L_frame; i++)
    {
        IF (sub(bits, target_bits) >= 0)   /* no bits left */
        {
            BREAK;
        }

        IF (x_Q[i] != 0)
        {
            sign = x_fac[i];
            move16();
            if (x_Q[i] < 0) sign = negate(sign);

            /* x_Q[i] += sign*(prm[bits++] * 0.5f - fac_m); */
            x_Q[i] = L_sub(x_Q[i], L_shr(L_mult(sign, add(deadzone, lshl(prm[bits], 15))), x_Q_e));
            move32();
            bits = add(bits, 1);
        }
        ELSE
        {
            zeros[num_zeros] = i;
            move16();
            num_zeros = add(num_zeros, 1);
        }
    }

    /* Requantize zeroed-lines of the spectrum */
    fac_p = msu_r(FL2WORD32(0.33f*2.0f), deadzone, FL2WORD16(0.33f*2.0f));  /* Q15 */
    target_bits = sub(target_bits, 1); /* reserve 1 bit for the check below */

    s = sub(x_Q_e, 1);
    FOR (j = 0; j < num_zeros; j++)
    {
        IF (sub(bits, target_bits) >= 0)   /* 1 or 0 bits left */
        {
            BREAK;
        }

        i = zeros[j];
        move16();

        IF (prm[bits] != 0)
        {
            bits = add(bits, 1);
            L_tmp = L_mult(fac_p, x_fac[i]); /* Q30 */
            if (prm[bits] == 0) L_tmp = L_negate(L_tmp);
            x_Q[i] = L_shr(L_tmp, s);
            move32();
        }
        bits = add(bits, 1);
    }


    return bits;
}

Word16 tcx_res_invQ_gain(
    Word16 *gain_tcx,
    Word16 *gain_tcx_e,
    Word16 *prm,
    Word16 resQBits
)
{
    Word16 bits;
    Word16 gain, tmp1, tmp2;


    gain = *gain_tcx;
    move16();

    /* make sure we have a bit of headroom */
    IF (sub(gain, 0x7000) > 0)
    {
        gain = shr(gain, 1);
        *gain_tcx_e = add(*gain_tcx_e, 1);
        move16();
    }

    /*Refine the gain quantization*/
    tmp1 = s_min(resQBits, TCX_RES_Q_BITS_GAIN);
    FOR (bits=0; bits < tmp1; bits++)
    {
        tmp2 = gain_corr_fac[bits];
        move16();
        if (prm[bits] == 0)
        {
            tmp2 = gain_corr_inv_fac[bits];
            move16();
        }

        gain = mult_r(gain, tmp2);
        if (prm[bits] != 0) gain = shl(gain, 1);
    }

    *gain_tcx = gain;
    move16();


    return bits;
}

Word16 tcx_res_invQ_spec(
    Word32 *x,
    Word16 x_e,
    Word16 L_frame,
    Word16 *prm,
    Word16 resQBits,
    Word16 bits,
    Word16 sq_round,
    const Word16 lf_deemph_factors[]
)
{
    Word16 i;
    Word16 fac_m, fac_p;
    Word16 lf_deemph_factor, c, s;
    Word32 tmp;


    /* Limit the number of residual bits */
    resQBits = s_min(resQBits, NPRM_RESQ);

    /* Requantize the spectrum line-by-line */
    fac_m = shr(sq_round, 1);
    fac_p = sub(0x4000, fac_m);

    lf_deemph_factor = 0x4000;
    move16();
    s = sub(x_e, 1);

    FOR (i = 0; i < L_frame; i++)
    {
        IF (sub(bits, resQBits) >= 0)
        {
            BREAK;
        }

        test();
        test();
        IF ((x[i] != 0) && ((lf_deemph_factors == NULL) || (sub(lf_deemph_factors[i], 0x2000) > 0)))
        {
            if (lf_deemph_factors != NULL)
            {
                lf_deemph_factor = lf_deemph_factors[i];
                move16();
            }

            IF (prm[bits] == 0)
            {

                /* Debug initialization to catch illegal cases of x[i] */
                tmp = 0;

                if (x[i] > 0) tmp = L_mult(fac_m, lf_deemph_factor);
                if (x[i] < 0) tmp = L_mult(fac_p, lf_deemph_factor);

                assert(tmp != 0);

                x[i] = L_sub(x[i], L_shr(tmp, s));
                move32();
            }
            ELSE
            {

                /* Debug initialization to catch illegal cases of x[i] */
                tmp = 0;

                if (x[i] > 0) tmp = L_mult(fac_p, lf_deemph_factor);
                if (x[i] < 0) tmp = L_mult(fac_m, lf_deemph_factor);

                assert(tmp != 0);

                x[i] = L_add(x[i], L_shr(tmp, s));
                move32();
            }
            bits = add(bits, 1);
        }
    }

    /*Quantize zeroed-line of the spectrum*/
    resQBits = sub(resQBits, 1);

    IF (lf_deemph_factors == NULL)
    {
        FOR (i = 0; i < L_frame; i++)
        {
            IF (sub(bits, resQBits) >= 0)
            {
                BREAK;
            }

            IF (x[i] == 0)
            {
                IF (prm[bits] != 0)
                {
                    bits = add(bits, 1);

                    tmp = L_mult(FL2WORD16_SCALE(1.32f, 1), fac_p);
                    if (prm[bits] == 0) tmp = L_negate(tmp);

                    x[i] = L_shr(tmp, s);
                    move32();
                }
                bits = add(bits, 1);
            }
        }
    }
    ELSE
    {
        c = sub(FL2WORD16(0.66f), mult_r(sq_round, FL2WORD16(0.66f)));

        FOR (i = 0; i < L_frame; i++)
        {
            IF (sub(bits, resQBits) >= 0)
            {
                BREAK;
            }

            test();
            IF ((x[i] == 0) && (sub(lf_deemph_factors[i], 0x2000) > 0))
            {
                IF (prm[bits] != 0)
                {
                    bits = add(bits, 1);

                    tmp = L_mult(c, lf_deemph_factors[i]);
                    if (prm[bits] == 0) tmp = L_negate(tmp);

                    x[i] = L_shr(tmp, s);
                    move32();
                }
                bits = add(bits, 1);
            }
        }
    }

    return bits;
}
