/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "options.h"
#include "stl.h"
#include "rom_com_fx.h"
#include "rom_dec_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "cnst_fx.h"
#include "prot_fx.h"

/**
 * \brief Arithmetic decoder
 * \param[i/o] st
 * \param[o] x
 * \param[i] nt
 * \param[i] nbbits
 * \param[i] resQMaxBits
 * \return resQBits
 */
Word16 ACcontextMapping_decode2_no_mem_s17_LC(
    Decoder_State_fx *st,/* i/o: decoder state */
    Word16 *x,           /* o: decoded spectrum */
    Word16 nt,           /* i: size of spectrum */
    Word16 nbbits,       /* i: bit budget */
    Word16 resQMaxBits,  /* i: residual coding maximum bits*/
    CONTEXT_HM_CONFIG *hm_cfg /* i: context-based harmonic model configuration */
)
{
    TastatDec as;
    Word16 start_bit_pos, lsbs_bit_pos, overflow_bit_pos;
    Word16 a, b, t, a1, b1, a1_i, b1_i, k, L;
    Word16 lev, pki, esc_nb;
    Word16 rateFlag;
    Word16 r, lastnz, n;
    Word16 resQBits;
    Word16 rest_bits, rest_bits_overflow;
    Word16 tmp=0; /* initialize just to avoid compiler warning */
    Word16 nt_half;
    Word16 c[2], *ctx;
    Word16 p1, p2;
    Word16 ii[2], idx1, idx2, idx;
    Word16 numPeakIndicesOrig, numHoleIndices;
    get_next_coeff_function get_next_coeff;
    Word16 nbbits_m2;
    Word32 La, Lb;

    r = 0;        /* to avoid compilation warnings */
    esc_nb = 0;   /* to avoid compilation warnings */

    set16_fx(x, 0, nt);

    /* Rate flag */
    rateFlag = 0;
    move16();
    if (sub(nbbits, 400) > 0)
    {
        rateFlag = 2 << NBITS_CONTEXT;
        move16();
    }

    /*Decode number of ntuples*/
    start_bit_pos = st->next_bit_pos_fx;
    move16();
    lsbs_bit_pos = add(start_bit_pos, sub(nbbits, 1));

    n = 0;
    move16();

    L = sub(14, norm_s(negate(nt)));
    n = add(get_next_indice_fx(st, L), 1);

    /* Init */
    nt_half = shr(nt, 1);
    c[0] = c[1] = 0;
    move16();
    move16();

    t = 0;
    move16();

    lastnz = shl(n, 1);

    IF (hm_cfg)   /* mapped domain */
    {
        numPeakIndicesOrig = hm_cfg->numPeakIndices;
        move16();
        hm_cfg->numPeakIndices = s_min(hm_cfg->numPeakIndices, lastnz);
        move16();
        numHoleIndices = sub(lastnz, hm_cfg->numPeakIndices);

        /* Mark hole indices beyond lastnz as pruned */
        FOR (k=numHoleIndices; k<hm_cfg->numHoleIndices; ++k)
        {
            hm_cfg->holeIndices[k] = add(hm_cfg->holeIndices[k], nt);
            move16();
        }

        ii[0] = numPeakIndicesOrig;
        move16();
        ii[1] = 0;
        move16();

        get_next_coeff = get_next_coeff_mapped;
        move16();
    }
    ELSE   /* unmapped domain */
    {
        ii[0] = 0;
        move16();

        p1 = p2 = 0;
        move16();
        move16();

        get_next_coeff = get_next_coeff_unmapped;
        move16();
    }

    /* Start Decoding */
    ari_start_decoding_14bits(st, &as);

    overflow_bit_pos = st->next_bit_pos_fx;
    move16();
    nbbits_m2 = add(nbbits, cbitsnew - 2);
    rest_bits_overflow = rest_bits = negate(nbbits_m2);
    move16();

    /*Main Loop through the 2-tuples*/
    FOR (k = 0; k < lastnz; k += 2)
    {
        /* First coefficient */
        a1_i = get_next_coeff(ii, &p1, &idx1, hm_cfg);

        /* Second coefficient */
        b1_i = get_next_coeff(ii, &p2, &idx2, hm_cfg);

        idx = s_min(idx1, idx2);

        /* Get context */
        ctx = &c[s_or(p1, p2)];
        move16();

        t = add(*ctx, rateFlag);
        if (sub(idx, nt_half) > 0)
        {
            t = add(t, 1 << NBITS_CONTEXT);
        }

        La = L_deposit_l(0);
        Lb = L_deposit_l(0);
        /* MSBs decoding */
        FOR (lev = 0; lev < 0x7FFF; lev++)
        {
            esc_nb = s_min(lev, 3);
            tmp = add(t, Tab_esc_nb[esc_nb]);
            assert(tmp >= 0 && tmp < 4096);
            pki = ari_lookup_s17_LC[tmp];
            move16();
            r = ari_decode_14bits_s17_ext(st, &as, ari_pk_s17_LC_ext[pki]);

            IF (sub(r, VAL_ESC) < 0)
            {
                BREAK;
            }

            /*LSBs decoding*/
            La = L_mac0(La, pow2tab[lev], get_indice_1_fx(st, lsbs_bit_pos--));
            Lb = L_mac0(Lb, pow2tab[lev], get_indice_1_fx(st, lsbs_bit_pos--));
        }

        /*MSBs contributions*/
        b1 = shr(r, 2);
        a1 = s_and(r, 0x3);
        a = extract_l(L_mac0(La, pow2tab[lev], a1));
        b = extract_l(L_mac0(Lb, pow2tab[lev], b1));

        /*lsbs bits sign bits*/
        rest_bits = add(rest_bits, shl(lev, 1));

        if (a > 0)
        {
            rest_bits = add(rest_bits, 1);
        }
        if (b > 0)
        {
            rest_bits = add(rest_bits, 1);
        }

        /*Dectect overflow*/
        IF (add(sub(st->next_bit_pos_fx, start_bit_pos), rest_bits) > 0)
        {
            /* Roll back bit-stream position to overflow_bit_pos */
            get_next_indice_tmp_fx(st, sub(overflow_bit_pos, st->next_bit_pos_fx));
            rest_bits = rest_bits_overflow;
            move16();
            x[a1_i] = 0;
            move16();
            x[b1_i] = 0;
            move16();
            BREAK;
        }

        overflow_bit_pos = st->next_bit_pos_fx;
        move16();
        rest_bits_overflow = rest_bits;
        move16();

        /* Store decoded data */
        x[a1_i] = a;
        move16();
        x[b1_i] = b;
        move16();

        /* Update context for next 2-tuple */
        IF (sub(p1, p2) == 0)   /* peak-peak or hole-hole context */
        {
            lev = sub(esc_nb, 1);
            if (lev >  0) t = add(12, lev);
            if (lev <= 0) t = add(a1, b1);
            if (lev == 0) t = add(t, t);

            *ctx = add(shl(s_and(*ctx, 0xf), 4), add(t, 1));
            move16();
        }
        ELSE   /* mixed context */
        {
            IF (s_and(idx1, 1))   /* update first context */
            {
                c[p1] = update_mixed_context(c[p1], a);
                move16();
            }

            IF (s_and(idx2, 1))   /* update second context */
            {
                c[p2] = update_mixed_context(c[p2], b);
                move16();
            }
        }
    }

    /* Total number of decoded AC bits */
    get_next_indice_tmp_fx(st, -(cbitsnew - 2));

    /*detect overflow*/
    IF (sub(k, lastnz) != 0)
    {
        rest_bits = add(rest_bits, nbbits_m2);
        /* Set bit-stream position to (start_bit_pos+nbbits-rest_bits) */
        get_next_indice_tmp_fx(st, sub((sub(add(start_bit_pos, nbbits), rest_bits)), st->next_bit_pos_fx));
    }

    /*Decode signs*/
    if (hm_cfg)
    {
        tmp = nt;
        move16();
    }
    if (!hm_cfg)
    {
        tmp = lastnz;
        move16();
    }
    FOR (k = 0; k < tmp; k++)
    {
        IF (x[k] > 0)
        {
            if (get_next_indice_1_fx(st) != 0)
            {
                x[k] = negate(x[k]);
                move16();
            }
        }
    }

    /*Decode Residual Q*/
    resQBits = s_min(resQMaxBits, sub(add(lsbs_bit_pos,1), st->next_bit_pos_fx));
    FOR (k=0; k<resQBits; ++k)
    {
        x[nt+k] = get_indice_1_fx(st, sub(lsbs_bit_pos, k));
        move16();
    }

    /* Set bit-stream pointer to end of buffer */
    get_next_indice_tmp_fx(st, sub(add(start_bit_pos, nbbits), st->next_bit_pos_fx));


    return resQBits;
}

