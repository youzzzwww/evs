/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*--------------------------------------------------------------------------*
 * mdct_classifier()
 *
 * MDCT signal classifier
 *--------------------------------------------------------------------------*/

#define MDCT_CLASSIFER_SMOOTH_FILT_COEFF 26214  /* 0.8 in Q15 */
#define MDCT_CLASSIFER_THRESH_UP 13107  /* 1.6 in Q13 */
#define MDCT_CLASSIFER_THRESH_DOWN 9011 /* 1.1 in Q13 */
#define MDCT_CLASSIFER_HQ_LOCAL (3 << 13) /* Q13, Define those local to make the filtering operation robust in case classes numbers are changed */
#define MDCT_CLASSIFER_TCX_LOCAL (1 << 13) /* Q13 */


/*-----------------------------------------------------------------------------
 * dft_mag_square_fx()
 *
 * Square magnitude of fft spectrum
 *----------------------------------------------------------------------------*/
static void dft_mag_square_fx(
    const Word16 x[],     /* i : Input vector: re[0], re[1], ..., re[n/2], im[n/2 - 1], im[n/2 - 2], ..., im[1] */
    Word32 magSq[], /* o : Magnitude square spectrum */
    const Word16 n        /* i : Input vector length */
)
{
    Word16 i, l;
    const Word16 *pRe, *pIm;
    Word32 *pMagSq, acc;

    /* Magnitude square at 0. */
    pMagSq = &magSq[0];
    pRe = &x[0];
    *pMagSq++ = L_mult0(*pRe, *pRe);
    pRe++;
    move32();

    /* From 1 to (N/2 - 1). */
    l = sub(shr(n, 1), 1);  /* N/2 - 1. */
    pIm = &x[n];
    pIm--;
    FOR (i = 0; i < l; i++)
    {
        acc = L_mult0(*pRe, *pRe);
        pRe++;
        *pMagSq++ = L_mac0(acc, *pIm, *pIm);
        pIm--;
        move32();
    }

    /* The magnitude square at N/2 */
    *pMagSq = L_mult0(*pRe, *pRe);
    move32();
    return;
}

Word16 mdct_classifier_fx(   /* o: MDCT A/B decision                         */
    const Word16 *Y,         /* i: re[0], re[1], ..., re[n/2], im[n/2 - 1], im[n/2 - 2], ..., im[1] */
    Encoder_State_fx *st_fx,    /* i/o: Encoder state variable                  */
    Word16 vadflag
    , Word32 *cldfbBuf_Ener
)
{
    Word16 c;
    Word32 magSq[129], *pMagSq, nf, pe;
    Word16 k;
    Word16 np;
    Word32 max_cand;
    Word16 max_i;
    Word32 p_energy_man, n_energy_man, man;
    Word16 p_energy_exp, n_energy_exp, expo;
    Word16 d_acc;
    Word16 pos_last;
    Word16 clas_sec;
    Word16 clas_final;
    Word16 condition1, condition2;
    Word16 factor;
    Word32 acc;
    UWord16 lsb16;
    UWord32 lsb32;
    Word32 gain1, gain2, gain3, gain11, gain4;
    Word32 peak_l, peak_h, avrg_l, avrg_h, peak_H1, avrg_H1, peak_H2, avrg_H2;
    Word16 condition3, condition4;

    dft_mag_square_fx(Y, magSq, 256);

    nf = L_add(magSq[0], 0);
    pe = L_add(magSq[0], 0);
    np = 0;
    move16();
    max_cand = L_negate(1);
    max_i = 0;
    move16();
    p_energy_man = L_deposit_l(0);
    n_energy_man = L_deposit_l(0);
    p_energy_exp = n_energy_exp = 32;
    move16();
    move16();
    d_acc = 0;
    move16();
    pos_last = -1;
    move16();

    pMagSq = magSq;
    FOR (k = 0; k < 128; k++)
    {
        /* NB: a*f + b*(1 - f)      needs two multiplies
         *     = (a - b)*f + b      saves one multiply     */
        IF (L_sub(*(++pMagSq), nf) > 0L)
        {
            factor = 31385;
            move16();/* 0.9578 in Q15 */
        }
        ELSE
        {
            factor = 21207;
            move16();/* 0.6472 in Q15 */
        }
        acc = L_sub(nf, *pMagSq);
        Mpy_32_16_ss(acc, factor, &acc, &lsb16);
        nf = L_add(acc, *pMagSq);
        IF (L_sub(*pMagSq, pe) > 0L)
        {
            factor = 13840;
            move16();/* 0.42237 in Q15 */
        }
        ELSE
        {
            factor = 26308;
            move16();/* 0.80285 in Q15 */
        }
        acc = L_sub(pe, *pMagSq);
        Mpy_32_16_ss(acc, factor, &acc, &lsb16);
        pe = L_add(acc, *pMagSq);
        Mpy_32_16_ss(pe, 20972, &acc, &lsb16); /* 0.64 in Q15 */
        IF (L_sub(*pMagSq, acc) > 0L)
        {
            IF (L_sub(*pMagSq, max_cand) > 0L)
            {
                max_cand = L_add(*pMagSq, 0);
                max_i = add(2, k);
            }
        }
        ELSE
        {
            IF (max_i > 0)
            {
                IF (sub(np, 0) > 0)
                {
                    d_acc = sub(add(d_acc, max_i), pos_last);
                }
                np = add(np, 1);
                pos_last = max_i;
                move16();
            }

            max_cand = L_negate(1);
            max_i = 0;
            move16();
        }

        IF (pe != 0)
        {
            expo = norm_l(pe);
            man = L_shl(pe, expo);
            Mpy_32_32_ss(man, man, &man, &lsb32);  /* pe square */
            expo = shl(expo, 1); /* Multiply by 2 due to squaring. */
            floating_point_add(&p_energy_man, &p_energy_exp, man, expo);
        }
        IF (nf != 0)
        {
            expo = norm_l(nf);
            man = L_shl(nf, expo);
            Mpy_32_32_ss(man, man, &man, &lsb32);  /* nf square */
            expo = shl(expo, 1); /* Multiply by 2 due to squaring. */
            floating_point_add(&n_energy_man, &n_energy_exp, man, expo);
        }
    }

    gain1 = L_deposit_l(0);
    gain2 = L_deposit_l(0);
    gain3 = L_deposit_l(0);
    FOR (k = 0; k < 8; k++)
    {
        gain1 = L_add(gain1, L_shr(cldfbBuf_Ener[k], 3));
        gain2 = L_add(gain2, L_shr(cldfbBuf_Ener[k + 8], 3));
        gain3 = L_add(gain3, L_shr(cldfbBuf_Ener[k + 16], 3));
    }

    /* gain11 = 8*(gain1 - cldfbBuf_Ener[0]/8)/7; */
    acc = L_shr(cldfbBuf_Ener[0], 3);
    acc = L_sub(gain1, acc);
    acc = Mult_32_16(acc, 4681);
    gain11 = L_shl(acc, 3);
    gain4 = L_deposit_l(0);
    FOR (k = 0; k < 12; k++)
    {
        gain4 = L_add(gain4, Mult_32_16(cldfbBuf_Ener[k + 12], 2731));
    }


    peak_H1 = L_add(cldfbBuf_Ener[25], 0);
    Mpy_32_16_ss(cldfbBuf_Ener[25], 6554, &avrg_H1, &lsb16);
    FOR (k = 1; k < 5; k++)
    {
        IF(L_sub(cldfbBuf_Ener[k + 25], peak_H1) > 0)
        {
            peak_H1 = L_add(cldfbBuf_Ener[k + 25], 0);
        }
        avrg_H1 = L_add(avrg_H1, Mult_32_16(cldfbBuf_Ener[k + 25], 6554));
    }

    peak_H2 = L_add(cldfbBuf_Ener[20], 0);
    Mpy_32_16_ss(cldfbBuf_Ener[20], 6554, &avrg_H2, &lsb16);
    FOR (k = 1; k < 5; k++)
    {
        IF (L_sub(cldfbBuf_Ener[k + 20], peak_H2) > 0)
        {
            peak_H2 = L_add(cldfbBuf_Ener[k + 20], 0);
        }
        avrg_H2 = L_add(avrg_H2, Mult_32_16(cldfbBuf_Ener[k + 20], 6554));
    }


    /* This part should use X which I don't know the Q-value */
    peak_l = L_deposit_l(0);
    avrg_l = L_deposit_l(0);
    peak_h = L_deposit_l(0);
    avrg_h = L_deposit_l(0);
    FOR (k = 0; k < 32; k++)
    {
        avrg_l = L_add(avrg_l, L_shr(magSq[k + 20], 5));
        avrg_h = L_add(avrg_h, L_shr(magSq[k + 96], 5));
        IF (L_sub(magSq[k + 20], peak_l) > 0)
        {
            peak_l = L_add(magSq[k + 20], 0);
        }
        IF (L_sub(magSq[k + 96], peak_h) > 0)
        {
            peak_h = L_add(magSq[k + 96], 0);
        }
    }

    /* Compute: d_acc - 12*(np -1). */
    acc = L_deposit_l(d_acc);
    IF (L_msu(acc, 12/2, sub(np, 1)) > 0) /* 12/2 is to compensate the fractional mode multiply */
    {
        condition1 = 1; /* Signifies: d_acc/(np - 1) > 12 */                    move16();
    }
    ELSE
    {
        condition1 = 0; /* Signifies: d_acc/(np - 1) <= 12 */                   move16();
        /* NB: For np = 0 or 1, it fits this condition. */
    }

    /* Compute: p_energy - 147.87276*n_energy. */
    IF (n_energy_man != 0)
    {
        Mpy_32_16_ss(n_energy_man, 18928, &acc, &lsb16); /* 147.87276 in Q7 */
        expo = sub(n_energy_exp, 15 - 7);  /* Due to 18928 in Q7 */
        acc = L_negate(acc);  /* To facilitate the following floating_point_add() to perform subtraction. */
        floating_point_add(&acc, &expo, p_energy_man, p_energy_exp);
    }
    ELSE
    {
        acc = L_deposit_l(0);
    }
    IF (acc > 0)
    {
        condition2 = 1; /* Signifies: p_energy / n_energy > 147.87276 */        move16();
    }
    ELSE
    {
        condition2 = 0; /* Signifies: p_energy / n_energy <= 147.87276 */       move16();
    }

    condition3 = 0;
    move16();
    condition4 = 0;
    move16();

    test();
    test();
    test();
    test();
    IF (L_sub(Mult_32_16(gain3, 27307), gain2) > 0 || (L_sub(gain3, Mult_32_16(gain2, 26214)) >= 0 && L_sub(peak_H1, L_shl(avrg_H1, 1)) > 0)
        || (L_sub(Mult_32_32(peak_l, avrg_h), Mult_32_32(Mult_32_16(peak_h, 14564), avrg_l)) < 0 || L_sub(Mult_32_32(Mult_32_16(peak_l, 14564), avrg_h), Mult_32_32(peak_h, avrg_l)) > 0))
    {
        condition3 = 1;
        move16();
    }

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF ((L_sub(gain4, Mult_32_16(gain11, 26214)) > 0 && L_sub(Mult_32_32(peak_l, avrg_h), Mult_32_32(Mult_32_16(peak_h, 12800), avrg_l))  > 0 && L_sub(Mult_32_32(Mult_32_16(peak_l, 6400), avrg_h), Mult_32_32(peak_h, avrg_l)) < 0)
        || (L_sub(gain4, Mult_32_16(gain11, 9830)) > 0 && L_sub(Mult_32_16(peak_h, 21845), avrg_h) < 0 && L_sub(Mult_32_16(peak_H2, 21845), avrg_H2) < 0)
        || (L_sub(Mult_32_32(peak_l, avrg_h), Mult_32_32(Mult_32_16(peak_h, 12800), avrg_l)) < 0 && L_sub(Mult_32_16(peak_h, 21845), avrg_h) > 0)
        || (L_sub(Mult_32_32(Mult_32_16(peak_l, 12800), avrg_h), Mult_32_32(peak_h, avrg_l)) > 0 && L_sub(Mult_32_16(peak_h, 21845), avrg_h) < 0) )
    {
        condition4 = 1;
        move16();
    }

    test();
    test();
    test();
    test();
    IF ((L_sub(st_fx->total_brate_fx, HQ_32k) == 0 && (s_xor(condition1, condition2) != 0 || condition3))
        || (L_sub(st_fx->total_brate_fx, HQ_24k40) == 0 && condition4))
    {
        c = MDCT_CLASSIFER_HQ_LOCAL;    /* Q13 */                               move16();
    }
    ELSE
    {
        c = MDCT_CLASSIFER_TCX_LOCAL;   /* Q13 */                               move16();
    }

    /* Smooth decision from instantaneous decision*/
    acc = L_mult(st_fx->clas_sec_old_fx, MDCT_CLASSIFER_SMOOTH_FILT_COEFF); /* st_fx->clas_sec_old_fx in Q13 */
    clas_sec = mac_r(acc, c, 0x7fff - MDCT_CLASSIFER_SMOOTH_FILT_COEFF); /* clas_sec and c are in Q13 */
    /* Do thresholding with hysteresis */
    test();
    test();
    test();
    test();
    test();
    test();
    IF ((sub(st_fx->clas_final_old_fx, HQ_CORE) == 0 || sub(st_fx->clas_final_old_fx, TCX_20_CORE) == 0)
        && ((L_sub(st_fx->last_gain1, L_shr(gain1, 1)) > 0 && L_sub(st_fx->last_gain1, L_shl(gain1, 1)) < 0)
            && (L_sub(st_fx->last_gain2, L_shr(gain2, 1)) > 0 && L_sub(st_fx->last_gain2, L_shl(gain2, 1)) < 0)))
    {
        clas_final = st_fx->clas_final_old_fx;
        move16();
    }
    ELSE IF (sub(clas_sec, st_fx->clas_sec_old_fx) > 0 && sub(clas_sec, MDCT_CLASSIFER_THRESH_UP) > 0) /* Going up? */
    {
        clas_final = HQ_CORE;     /* Q0 */                                  move16();
    }
    ELSE IF (sub(clas_sec, MDCT_CLASSIFER_THRESH_DOWN) < 0)/* Going down */
    {
        clas_final = TCX_20_CORE;
        move16();
    }
    ELSE
    {
        clas_final = st_fx->clas_final_old_fx;
        move16();
    }

    test();
    test();
    test();
    /* Prevent the usage of MDCTA on noisy-speech or inactive */
    if ( sub(st_fx->mdct_sw_enable, MODE2) == 0 && (sub(st_fx->flag_noisy_speech_snr, 1) == 0 || vadflag == 0 ) && sub(clas_final, HQ_CORE) == 0 )
    {
        clas_final = TCX_20_CORE;
        move16();
    }


    /* Memory update */
    st_fx->clas_sec_old_fx = clas_sec;
    move16(); /* Q13 */
    st_fx->clas_final_old_fx = clas_final;
    move16();  /* Q0 */
    st_fx->last_gain1 = gain1;
    move32();
    st_fx->last_gain2 = gain2;
    move32();

    return clas_final;  /* Q0 */
}


