/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"
#include "cnst_fx.h"
#include "stl.h"

#define HVQ_ENC_NOISE_DELTA  ((Word16)3277) /* 0.1 in Q15 */

static Word16 quant_lc(const Word16, Word16 *);


/*--------------------------------------------------------------------------*
 * hvq_enc_fx()
 *
 * Harmonic VQ encoder
 *--------------------------------------------------------------------------*/

Word16 hvq_enc_fx(                  /*o  : Consumed bits                    */
    Encoder_State_fx *st_fx,         /*i/o: encoder state structure          */
    const Word32 brate,             /*i  : Total bit rate                   */
    const Word16 hvq_bits,          /*i  : HVQ bit budget                   */
    const Word16 Npeaks,            /*i  : Number of peaks                  */
    const Word16 *ynrm,             /* i  : Envelope coefficients          */
    Word16 *R,                /* i/o: Bit allocation/updated bit allocation */
    Word16 *peaks,            /* i  : Peak pos. / Encoded peak pos.   */
    Word32 *nf_gains,         /* i/o: Noise fill gains / Quant. nf gains */
    Word16 *noise_level,      /* o  : Quantized noise level           */
    const Word32 *pe_gains,         /* i  : Peak gains                      */
    const Word32 *coefs,            /* i  : spectrum coefficients in Q12    */
    Word32 *coefs_out         /* o  : encoded spectrum coefficients in Q12 */
)
{
    const Word32 *pCoefs;
    Word16 bin_th,j,i,n;
    Word16 nf_cnt;
    Word16 q_noise_level_idx[HVQ_BWE_NOISE_BANDS];
    Word16 q_noise_level[HVQ_BWE_NOISE_BANDS];
    Word32 d, nf_mean;
    Word32 nf, pe, pe_mean;
    Word16 bits_used, nBits;
    Word16 lb_nfpe;
    UWord16 dontCare;
    Word32 acc, numerator, denominator;
    Word16 expPeMean, expNfMean, expNfpe, expNfpe3, expo, expo3;
    Word16 manPeMean, manNfMean, manNfpe, man;
    Word16 tmp16, adjust;

    bits_used = 0;
    move16();

    IF ( brate == HQ_24k40 )
    {
        bin_th = HVQ_THRES_BIN_24k;
        move16();
        n = (L_FRAME32k - HVQ_THRES_BIN_24k)/HVQ_BWE_NOISE_BANDS;
        move16();
    }
    ELSE
    {
        bin_th = HVQ_THRES_BIN_32k;
        move16();
        n = (L_FRAME32k - HVQ_THRES_BIN_32k)/HVQ_BWE_NOISE_BANDS;
        move16();
    }

    nf = 800*4096L;     /* Q12 */                                               move32();
    pe = 800*4096L;     /* Q12 */                                               move32();
    pCoefs = &coefs[bin_th];

    /* Find HB noise level */
    FOR( i = 0; i < HVQ_BWE_NOISE_BANDS; i++ )
    {
        nf_cnt = 0;
        move16();
        nf_mean = L_deposit_l(0);
        pe_mean = L_deposit_l(0);
        FOR( j = 0; j < n; j++)
        {
            d = L_abs(*pCoefs++);     /* Q12 */

            IF( L_sub(d, pe) > 0L )
            {
                /* W*pe + (1 - W)*d = (pe - d)*W + d */
                acc = L_sub(pe, d);
                Mpy_32_16_ss(acc, HVQ_BWE_WEIGHT2_FX, &acc, &dontCare);
                pe = L_add(acc, d);  /* in Q12 and always positive */
            }
            ELSE
            {
                /* W*pe + (1 - W)*d = (pe - d)*W + d */
                acc = L_sub(pe, d);
                Mpy_32_16_ss(acc, HVQ_BWE_WEIGHT1_FX, &acc, &dontCare);
                pe = L_add(acc, d);  /* in Q12 and always positive */

                IF( L_sub(d, nf) > 0L )
                {
                    acc = L_sub(nf, d);
                    Mpy_32_16_ss(acc, HVQ_BWE_WEIGHT1_FX, &acc, &dontCare);
                    nf = L_add(acc, d);  /* in Q12 and always positive */
                }
                ELSE
                {
                    acc = L_sub(nf, d);
                    Mpy_32_16_ss(acc, HVQ_BWE_WEIGHT2_FX, &acc, &dontCare);
                    nf = L_add(acc, d);  /* in Q12 always positive */
                }
                nf_mean = L_add(nf_mean, nf);  /* in Q12 and always positive */
                nf_cnt = add(nf_cnt, 1);       /* Q0 */
            }

            pe_mean = L_add(pe_mean, pe);  /* in Q12 and always positive */
        }

        IF (pe_mean > 0)
        {
            expPeMean = norm_l(pe_mean);                      /* exponent */
            manPeMean = extract_h(L_shl(pe_mean, expPeMean)); /* mantissa */
            expNfMean = norm_l(nf_mean);                      /* exponent */
            manNfMean = extract_h(L_shl(nf_mean, expNfMean)); /* mantissa */

            numerator = L_mult0(manNfMean, n);
            IF ( nf_cnt > 0 )
            {
                denominator = L_mult0(manPeMean, nf_cnt);     /* in Q15 */
            }
            ELSE
            {
                denominator = L_mult0(manPeMean, 1);          /* in Q15 */
            }
            manNfpe = ratio(numerator, denominator, &expo);   /* manNfpe in Q14 */
            expNfpe = add(sub(expNfMean, expPeMean), expo);

            tmp16 = mult_r(manNfpe, manNfpe);             /* in Q(14+14+1-16) = Q13 */
            tmp16 = mult_r(tmp16, manNfpe);               /* in Q(13+14+1-16) = Q12 */
            acc = L_mult(tmp16, HVQ_NFPE_FACTOR_CUBE_FX); /* in Q(12+6+1) = Q19 */
            expNfpe3 = extract_l(L_mult0(expNfpe, 3));    /* Cube operation */
            /* Number of bits required to adjust to Q15 */
            adjust = add(19 - (15 + 16), expNfpe3); /* +16 is due to the following extract_h(). */
            noise_level[i] = extract_h(L_shr(acc, adjust));  /* noise_level[] in Q15 */

            q_noise_level_idx[i] = quant_lc( noise_level[i], &q_noise_level[i] );
        }
        ELSE
        {
            q_noise_level_idx[i] = 0;
            move16();
            q_noise_level[i] = 0;
            move16();
        }
        push_indice_fx(st_fx, IND_HVQ_BWE_NL, q_noise_level_idx[i], 2 );
        bits_used = add(bits_used, 2);

        noise_level[i] = q_noise_level[i];  /* in Q15 */
    }

    FOR( i = 0; i < HVQ_NF_GROUPS; i ++ )
    {
        IF (pe_gains[i] != 0)
        {
            /* Neither pe_gains[] nor nf_gains[] is zero. */
            man = ratio(nf_gains[i], pe_gains[i], &expo);    /* man in Q14 */
            tmp16 = mult_r(man, man);                        /* in Q(14+14+1-16) = Q13 */
            tmp16 = mult_r(tmp16, man);                      /* in Q(13+14+1-16) = Q12 */
            acc = L_mult(tmp16, HVQ_LB_NFPE_FACTOR_CUBE_FX); /* in Q(12+9+1) = Q22 */
            expo3 = extract_l(L_mult0(expo, 3));             /* Cube operation. */
            /* Number of bits required to adjust to Q15 */
            adjust = add(22 - (15 + 16), expo3); /* +16 is due to the following extract_h(). */
            lb_nfpe = extract_h(L_shr(acc, adjust));  /* noise_level[] in Q15 */

            IF( lb_nfpe > 16384 ) /* in Q15 */
            {
                lb_nfpe = 16384;
                move16();
            }
            Mpy_32_16_ss(nf_gains[i], shl(lb_nfpe, 1), &nf_gains[i], &dontCare); /* nf_gains[] in Q12 */
        }
        ELSE
        {
            nf_gains[i] = 0;
            move16();
        }
    }
    nBits = peak_vq_enc_fx( st_fx, coefs, coefs_out, brate, sub(hvq_bits, bits_used),
                            Npeaks, ynrm, R, peaks, &nf_gains[0] );
    bits_used = add(bits_used, nBits);
    return bits_used;
}

/*-----------------------------------------------------------------------------
 * quant()
 *
 * Quantize the noise to one of the levels in {0, 0.1, 0.2, 0.3}
 *----------------------------------------------------------------------------*/
static Word16 quant_lc(const Word16 x, Word16 *qx)
{
    Word16 indx;

    IF (sub(x, HVQ_ENC_NOISE_DELTA/2) < 0)
    {
        indx = 0;
        move16();
        *qx = 0;
        move16();
    }
    ELSE IF (sub(x, 3*HVQ_ENC_NOISE_DELTA/2) < 0)
    {
        indx = 1;
        move16();
        *qx = HVQ_ENC_NOISE_DELTA;
        move16();
    }
    ELSE IF (sub(x, 5*HVQ_ENC_NOISE_DELTA/2) < 0)
    {
        indx = 2;
        move16();
        *qx = 2*HVQ_ENC_NOISE_DELTA;
        move16();
    }
    ELSE
    {
        indx = 3;
        move16();
        *qx = 3*HVQ_ENC_NOISE_DELTA;
        move16();
    }

    return indx;
}

