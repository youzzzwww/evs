/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "prot_fx.h"

/* Returns: number of bits consumed */
static Word16 tcx_arith_decode(
    Word16 L_frame,                   /* i: number of spectral lines      Q0 */
    const Word16 envelope[],          /* i: scaled envelope               Q15-e */
    Word16 envelope_e,                /* i: scaled envelope exponent      Q0 */
    Word16 target_bits,               /* i: target bit budget             Q0 */
    Word16 prm[],                     /* i: bit-stream                    Q0 */
    Word32 q_spectrum[],              /* o: scalar quantized spectrum     Q31-e */
    Word16 *q_spectrum_e,             /* o: spectrum exponent             Q0 */
    Word16 *nf_seed                   /* o: noise filling seed            Q0 */
)
{
    Word16 bp, k, q, s;
    TastatDec as;
    Word16 exp_k;
    Word16 tmp;
    Word32 L_tmp, Q;

    bp = ari_start_decoding_14bits_prm(prm, 0, &as);

    tmp = sub(envelope_e, 1+15);
    L_tmp = L_deposit_l(0);
    FOR (k = 0; k < L_frame; k++)
    {
        exp_k = round_fx(expfp(envelope[k], tmp));
        /* decode line magnitude */
        bp = ari_decode_14bits_pow(prm, bp, target_bits, &q, &as, exp_k);

        if (q == 0)
        {
            q_spectrum[k] = L_deposit_l(0);
        }
        IF (q != 0)   /* line is non-zero, decode sign */
        {
            bp = ari_decode_14bits_sign(prm, bp, target_bits, &s, &as);

            L_tmp = L_macNs(L_tmp, q, k);

            Q = L_mult(q, -1 << (30 - SPEC_EXP_DEC));
            if (s == 0) Q = L_mult(q, 1 << (30 - SPEC_EXP_DEC));
            q_spectrum[k] = Q;
            move32();
        }

        IF (ari_decode_overflow(&as))
        {
            assert(bp >= target_bits);
            BREAK;    /* no bits left, so exit loop */
        }
    }
    *q_spectrum_e = SPEC_EXP_DEC;
    move16();

    set32_fx(q_spectrum+k, 0, sub(L_frame, k));

    /* noise filling seed */
    *nf_seed = extract_l(L_tmp);


    return bp;
}

void tcx_arith_decode_envelope(
    Word32 q_spectrum[],                    /* o: quantised MDCT coefficients     Q31-e */
    Word16 *q_spectrum_e,                   /* o: MDCT exponent                   Q0 */
    Word16 L_frame,                         /* i: frame or MDCT length            Q0 */
    Word16 L_spec,                          /* i: length w/o BW limitation        Q0 */
    Decoder_State_fx *st,
    const Word16 A_ind[],                   /* i: quantised LPC coefficients      Q12 */
    Word16 target_bits,                     /* i: number of available bits        Q0 */
    Word16 prm[],                           /* i: bitstream parameters            Q0 */
    Word8 use_hm,                           /* i: use HM in current frame?        */
    Word16 prm_hm[],                        /* i: HM parameter area               Q0 */
    Word16 tcxltp_pitch,                    /* i: TCX LTP pitch in FD, -1 if n/a  Q0*/
    Word16 *arith_bits,                     /* o: bits used for ari. coding       Q0 */
    Word16 *signaling_bits,                 /* o: bits used for signaling         Q0 */
    Word16 *nf_seed                         /* o: noise filling seed              Q0 */
    ,Word16 low_complexity                  /* i: low-complexity flag           Q0 */
)
{
    Word32 env[N_MAX_ARI];             /* unscaled envelope */
    Word16 *envelope; /* scaled envelope */
    Word16 envelope_e;
    Word16 L_spec_core;
    TCX_config *tcx_cfg;
    Word16 gamma_w, gamma_uw;
    Word16 hm_bits;


    assert(L_spec<=N_MAX_ARI);


    tcx_cfg = &st->tcx_cfg;
    *signaling_bits = 0;
    move16();
    assert(st->enableTcxLpc);
    gamma_w  = FL2WORD16(1.0f);
    move16();
    gamma_uw = st->inv_gamma;
    move16();

    tcx_arith_render_envelope(
        A_ind,
        L_frame,
        L_spec,
        tcx_cfg->preemph_fac,
        gamma_w,
        gamma_uw,
        env
    );

    IF (use_hm != 0)
    {
        IF (prm_hm[0] != 0)
        {
            tcx_hm_decode(
                L_spec,
                env,
                target_bits,
                tcx_cfg->coder_type,
                prm_hm,
                tcxltp_pitch,
                &hm_bits
            );
        }
        ELSE
        {
            hm_bits = 1;
            move16();
        }
        *signaling_bits = add(*signaling_bits, hm_bits);
        move16();
    }
    ELSE
    {
        prm_hm[0] = 0;  /* just to be sure */                                       move16();
    }

    L_spec_core = L_spec;
    move16();
    if (st->igf)
    {
        L_spec_core = s_min(L_spec_core, st->hIGFDec.infoIGFStartLine);
    }
    envelope = (Word16*)env;

    tcx_arith_scale_envelope(
        L_spec,
        L_spec_core,
        env,
        target_bits,
        low_complexity,
        envelope,
        &envelope_e
    );

    *arith_bits = tcx_arith_decode(
                      L_spec,
                      envelope,
                      envelope_e,
                      target_bits,
                      prm,
                      q_spectrum,
                      q_spectrum_e,
                      nf_seed
                  );
    move16();

    set32_fx(q_spectrum + L_spec, 0, sub(L_frame, L_spec));
}
