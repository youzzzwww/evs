/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "options.h"
#include "stl.h"
#include "basop_util.h"
#include "stl.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"

/**
 * \brief Arithmetic encoder
 * \param[i] x
 * \param[o] ntuples amount of encoded tuple (2 lines per tuple)
 * \param[i] nt
 * \param[i] nbbits
 * \param[i] resQMaxBits
 * \return none
 */
Word16 ACcontextMapping_encode2_no_mem_s17_LC(
    Encoder_State_fx *st,
    Word16 *x,
    Word16 nt,
    Word16 lastnz,
    Word16 nbbits,
    Word16 resQMaxBits,
    CONTEXT_HM_CONFIG *hm_cfg)
{
    Word16 ptr[BITBUFSIZE];
    TastatEnc as, as_overflow;
    Word16 bp, bp_overflow;
    Word16 a1, b1, a1_i, b1_i, k;
    Word16 t, pki, lev1;
    Word16 rateFlag;
    Word16 value;
    Word16 nbbits_ntuples, nbbits_lsbs, nbbits_signs, nbbits_signs_overflow, nbbits_lsbs_overflow, flag_overflow;
    Word16 *lsbs_bits2;
    Word16 tmp;
    Word16 nt_half;
    Word16 c[2], *ctx;
    Word16 p1, p2;
    Word16 ii[2], idx1, idx2, idx;
    Word16 numPeakIndicesOrig, numHoleIndices;
    get_next_coeff_function get_next_coeff;
    Word16 signs[N_MAX];
    Word16 nbbits_m2;

    a1 = 0;       /* to avoid compilation warnings */
    b1 = 0;       /* to avoid compilation warnings */


    /* Rate flag */
    rateFlag = 0;
    move16();
    if (sub(nbbits, 400) > 0)
    {
        rateFlag = 2 << NBITS_CONTEXT;
        move16();
    }

    /* Init */
    nt_half = shr(nt, 1);
    c[0] = c[1] = 0;
    move16();
    move16();

    /* Bits for encoding the number of encoded tuples */
    nbbits_ntuples = sub(14, norm_s(negate(nt)));

    t = 0;
    move16();
    bp = nbbits_ntuples;
    move16();
    nbbits_signs = 0;
    move16();
    nbbits_lsbs = 0;
    move16();
    nbbits_m2 = sub(nbbits, 2);
    flag_overflow = 0;
    move16();

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

        /* Find last non-zero tuple */
        /* ensure termination of while loop by dummy value */
        a1 = x[0];
        move16();
        x[0] = 1;
        move16();

        WHILE (s_or(x[lastnz-1], x[lastnz-2]) == 0)
        {
            lastnz = sub(lastnz, 2);
        }
        x[0] = a1;
        move16();
    }

    lsbs_bits2 = ptr+nbbits-1;

    /*Start Encoding*/
    ari_start_encoding_14bits(&as);

    /*Main Loop through the 2-tuples*/
    b1_i = -1;
    move16();
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

        /* Init current 2-tuple encoding */
        if (flag_overflow != 0)
        {
            x[a1_i] = 0;
            move16();
        }
        if (flag_overflow != 0)
        {
            x[b1_i] = 0;
            move16();
        }

        a1 = abs_s(x[a1_i]);
        b1 = abs_s(x[b1_i]);

        lev1 = -1;
        move16();

        /*Copy states*/
        ari_copy_states(&as, &as_overflow);
        bp_overflow = bp;
        move16();
        nbbits_signs_overflow = nbbits_signs;
        move16();
        nbbits_lsbs_overflow = nbbits_lsbs;
        move16();

        /*Signs encoding*/
        signs[nbbits_signs] = lshr(x[a1_i], 15);
        move16();
        if (a1 > 0)
        {
            nbbits_signs = add(nbbits_signs, 1);
        }
        signs[nbbits_signs] = lshr(x[b1_i], 15);
        move16();
        if (b1 > 0)
        {
            nbbits_signs = add(nbbits_signs, 1);
        }

        /* MSBs coding */
        WHILE (sub(s_max(a1, b1), A_THRES) >= 0)
        {
            tmp = add(t, Tab_esc_nb[lev1+1]);
            assert(tmp >= 0 && tmp < 4096);
            pki = ari_lookup_s17_LC[tmp];
            move16();
            bp = ari_encode_14bits_ext(ptr, bp, &as, VAL_ESC, ari_pk_s17_LC_ext[pki]);

            *lsbs_bits2-- = s_and(a1, 1);
            move16();
            *lsbs_bits2-- = s_and(b1, 1);
            move16();

            /* LSBs bit counting */
            nbbits_lsbs = add(nbbits_lsbs, 2);

            a1 = shr(a1, 1);
            b1 = shr(b1, 1);

            lev1 = s_min(add(lev1, 1), 2);
        }

        tmp = add(t, Tab_esc_nb[lev1+1]);
        assert(tmp >= 0 && tmp < 4096);
        pki = ari_lookup_s17_LC[tmp];
        move16();
        bp = ari_encode_14bits_ext(ptr, bp, &as, add(a1, shl(b1, A_THRES_SHIFT)), ari_pk_s17_LC_ext[pki]);

        /* Check bit budget */
        IF (sub(add(add(add(bp, extract_l(as.vobf)), nbbits_signs), nbbits_lsbs), nbbits_m2) > 0)
        {
            ari_copy_states(&as_overflow, &as);
            bp = bp_overflow;
            move16();

            IF (flag_overflow == 0)
            {
                nbbits_signs = nbbits_signs_overflow;
                move16();
                nbbits_lsbs = nbbits_lsbs_overflow;
                move16();
                IF (hm_cfg)
                {
                    flag_overflow = 1;
                    move16();

                    /* Code from now only zeros */
                    x[a1_i] = 0;
                    move16();
                    x[b1_i] = 0;
                    move16();
                    lev1 = -1;
                    move16();

                    assert(t >= 0 && t < 4096);
                    pki = ari_lookup_s17_LC[t];
                    move16();
                    bp = ari_encode_14bits_ext(ptr, bp, &as, 0, ari_pk_s17_LC_ext[pki]);

                    IF (sub(add(add(add(bp, extract_l(as.vobf)), nbbits_signs), nbbits_lsbs), nbbits_m2) > 0)
                    {
                        ari_copy_states(&as_overflow, &as);
                        bp = bp_overflow;
                        move16();
                        BREAK;
                    }
                }
                ELSE
                {
                    BREAK;
                }
            }
            ELSE
            {
                BREAK;
            }
        }

        /* Update context for next 2-tuple */
        IF (sub(p1, p2) == 0)   /* peak-peak or hole-hole context */
        {
            if (lev1 >  0) t = add(12, lev1);
            if (lev1 <= 0) t = add(a1, b1);
            if (lev1 == 0) t = add(t, t);

            *ctx = add(shl(s_and(*ctx, 0xf), 4), add(t, 1));
            move16();
        }
        ELSE   /* mixed context */
        {
            IF (s_and(idx1, 1))   /* update first context */
            {
                c[p1] = update_mixed_context(c[p1], abs_s(x[a1_i]));
                move16();
            }

            IF (s_and(idx2, 1))   /* update second context */
            {
                c[p2] = update_mixed_context(c[p2], abs_s(x[b1_i]));
                move16();
            }
        }
    } /*end of the 2-tuples loop*/

    /* End arithmetic coder, overflow management */
    bp = ari_done_encoding_14bits(ptr, bp, &as);

    /* Overflow is detected */
    IF (sub(k, lastnz) != 0)
    {
        IF (hm_cfg)
        {
            /* Fill with zero to be sure that the decoder finishes the MSB decoding at the same position */
            tmp = sub(nbbits, add(nbbits_signs, nbbits_lsbs));
            set16_fx(&ptr[bp], 0, sub(tmp, bp));
            bp = tmp;
            move16();
        }
        if (!hm_cfg)
        {
            lastnz = k;
            move16();
        }
    }

    /* Push number of encoded tuples */
    value = sub(shr(lastnz, 1), 1);
    push_next_indice_fx(st, value, nbbits_ntuples);

    /* Push arithmetic coded bits */
    push_next_bits_fx(st, &ptr[nbbits_ntuples], sub(bp, nbbits_ntuples));

    /* Push sign bits */
    push_next_bits_fx(st, signs, nbbits_signs);
    bp = add(bp, nbbits_signs);

    /*write residual Quantization bits*/
    tmp = s_min(sub(sub(nbbits, bp), nbbits_lsbs), resQMaxBits);
    FOR (k = 0; k < tmp; k++)
    {
        ptr[nbbits-1-nbbits_lsbs-k] = x[nt+k];
        move16();
    }
    /* Write filler bits */
    tmp = sub(nbbits, add(bp, nbbits_lsbs));
    FOR (; k<tmp; ++k)
    {
        ptr[nbbits-1-nbbits_lsbs-k]=0;
        move16();
    }
    /*   fprintf(stderr, "Budget: %d Fill bits: %d ResQ Bits: %d\n",(int) (nbbits),(int) (nbbits-bp-nbbits_lsbs),(int)i);*/

    /* Check for debugging */
    assert( bp+k <= nbbits );

    /* Push the rest of the buffer */
    push_next_bits_fx(st, &ptr[bp], sub(nbbits, bp));

    return (bp+nbbits_lsbs); /*return only for debug plot*/
}

static Word16 find_last_nz_pair(
    const Word16 x[],
    Word16 length,
    const CONTEXT_HM_CONFIG *hm_cfg
)
{
    Word16 last_nz, i, stage, lobs[4];
    const Word16 *tmp = NULL;

    if (hm_cfg)
    {
        tmp = hm_cfg->indexBuffer;
        move16();
    }

    lobs[0] = 4;
    move16();
    lobs[1] = shr(length, 1); /* length/2 */ move16();
    lobs[2] = add(lobs[1], shr(length, 2));
    move16();
    lobs[3] = add(lobs[2], shr(length, 3));
    move16();

    last_nz = 0;
    move16();
    i = length;
    move16();
    FOR (stage=3; stage>=0; --stage)
    {
        IF (hm_cfg)   /* mapped kernel */
        {
            FOR (; i >= lobs[stage]; i-=2)
            {
                if (x[tmp[i-2]] != 0)
                {
                    last_nz = s_max(last_nz, i);
                }
                if (x[tmp[i-1]] != 0)
                {
                    last_nz = s_max(last_nz, i);
                }
            }
        }
        ELSE   /* unmapped kernel */
        {
            FOR (; i >= lobs[stage]; i-=2)
            {
                if (x[i-2] != 0)
                {
                    last_nz = s_max(last_nz, i);
                }
                if (x[i-1] != 0)
                {
                    last_nz = s_max(last_nz, i);
                }
            }
        }
        IF (last_nz > 0)
        {
            BREAK;
        }
    }

    return s_max(last_nz, 2);
}

Word16 ACcontextMapping_encode2_estimate_no_mem_s17_LC(
    const Word16 *x,
    Word16 nt,
    Word16 *lastnz_out,
    Word16 *nEncoded,
    Word16 target,
    Word16 *stop,
    CONTEXT_HM_CONFIG *hm_cfg
)
{
    Word16 a1, b1, a1_i, b1_i;
    Word16 k, t, pki, lev1;
    Word16 lastnz, lastnz2;
    Word16 rateFlag;
    Word16 nbits_old, nbits;
    Word16 stop2;
    Word16 cum_freq_norm, cum_freq_norm_e;
    Word32 proba;
    Word16 nlz;
    const UWord16 *cum_freq;
    Word16 symbol;
    const Word8 *lookup;
    Word16 nt_half;
    Word16 c[2], *ctx;
    Word16 p1, p2;
    Word16 ii[2], idx1, idx2, idx;
    Word16 numPeakIndicesOrig=0, numHoleIndices=0; /* initialize just to avoid compiler warning */
    get_next_coeff_function get_next_coeff;


    /* Rate flag */
    rateFlag = 0;
    move16();
    if (sub(target, 400) > 0)
    {
        rateFlag = 2 << NBITS_CONTEXT;
        move16();
    }

    /* proba coded on 14bits -> proba=1 */
    proba = L_deposit_l(16384);

    /* Init */
    nt_half = shr(nt, 1);
    stop2 = 0;
    move16();
    c[0] = c[1] = 0;
    move16();
    move16();

    /* bits to encode lastnz + 2 bits headroom */
    nbits = sub(2+14, norm_s(sub(nt, 1)));
    nbits_old = nbits;
    move16();
    nbits = sub(nbits, target);

    /* Find last non-zero tuple in the mapped domain signal */
    lastnz = find_last_nz_pair(x, nt, hm_cfg);

    /* At least one tuple is coded */
    lastnz2 = 2;
    move16();

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

    /* Main Loop through the 2-tuples */
    FOR (k=0; k<lastnz; k+=2)
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

        /* Init current 2-tuple encoding */
        a1 = abs_s(x[a1_i]);
        b1 = abs_s(x[b1_i]);

        lev1 = -1 << (NBITS_CONTEXT+NBITS_RATEQ);
        move16();

        /* Signs Bits */
        if (a1 > 0)
        {
            nbits = add(nbits, 1);
        }

        if (b1 > 0)
        {
            nbits = add(nbits, 1);
        }

        /* pre-compute address of ari_pk_s17_LC_ext[0][Val_esc] to avoid doing it multiple times inside the loop */

        /* check while condition */

        /* MSBs coding */
        lookup = &ari_lookup_s17_LC[t] + (1 << (NBITS_CONTEXT+NBITS_RATEQ)); /* address calculation not counted */
        WHILE (sub(s_max(a1, b1), A_THRES) >= 0)
        {
            pki = lookup[lev1];
            move16();
            cum_freq = ari_pk_s17_LC_ext[pki] + VAL_ESC; /* address calculation not counted */
            /*p1*p2=proba on 28 bits: p=0.5->power(2,27)*/

            cum_freq_norm_e = norm_s(cum_freq[0]);
            cum_freq_norm = shl(cum_freq[0], cum_freq_norm_e);
            proba = Mpy_32_16_1(proba, cum_freq_norm);
            proba = L_shl(proba, sub(14, cum_freq_norm_e));

            /*Number of leading zero computed in one cycle=norm_l() in BASOP*/
            nlz = sub(norm_l(proba), 2);
            proba = L_shl(proba, sub(nlz, 15));

            nbits = add(nbits, nlz);
            /*addition added as shift not done in norm_l(): real shift = 14-nlz*/
            /*proba is rounded down on 14 bits ->automatic over-estimation of bit consumption*/

            a1 = shr(a1, 1);
            b1 = shr(b1, 1);
            lev1 = s_min(add(lev1, 1 << (NBITS_CONTEXT+NBITS_RATEQ)), 2 << (NBITS_CONTEXT+NBITS_RATEQ));
        }

        pki = lookup[lev1];
        move16();
        symbol = add(a1, shl(b1, A_THRES_SHIFT));
        cum_freq = ari_pk_s17_LC_ext[pki] + symbol; /* address calculation not counted */
        /*p1*p2=proba on 28 bits: p=0.5->power(2,27)*/

        cum_freq_norm = sub(cum_freq[0], cum_freq[1]);
        cum_freq_norm_e = norm_s(cum_freq_norm);
        cum_freq_norm = shl(cum_freq_norm, cum_freq_norm_e);
        proba = Mpy_32_16_1(proba, cum_freq_norm);
        proba = L_shl(proba, sub(15, cum_freq_norm_e));

        /*Number of leading zero computed in one cycle=norm_l() in BASOP*/
        nlz = sub(norm_l(proba), 3);
        proba = L_shl(proba, sub(nlz, 14));

        nbits = add(nbits, nlz);
        /*addition added as shift not done in norm_l(): real shift = 14-nlz*/
        /*proba is rounded down on 14 bits ->automatic over-estimation of bit consumption*/

        /* Should we truncate? */

        IF (nbits > 0)
        {
            stop2 = 1;
            move16();

            IF (*stop)
            {
                BREAK;
            }
        }
        ELSE
        {
            test();
            test();
            IF (hm_cfg || k==0 || s_or(x[a1_i], x[b1_i]))
            {
                nbits_old = add(nbits, target);
                lastnz2 = add(b1_i, 1);
            }
        }

        /* Update context for next 2-tuple */
        IF (sub(p1, p2) == 0)   /* peak-peak or hole-hole context */
        {
            lev1 = shr(lev1, NBITS_CONTEXT+NBITS_RATEQ);
            if (lev1 >  0) t = add(12, lev1);
            if (lev1 <= 0) t = add(a1, b1);
            if (lev1 == 0) t = add(t, t);

            *ctx = add(shl(s_and(*ctx, 0xf), 4), add(t, 1));
            move16();
        }
        ELSE   /* mixed context */
        {
            IF (s_and(idx1, 1))   /* update first context */
            {
                c[p1] = update_mixed_context(c[p1], abs_s(x[a1_i]));
                move16();
            }

            IF (s_and(idx2, 1))   /* update second context */
            {
                c[p2] = update_mixed_context(c[p2], abs_s(x[b1_i]));
                move16();
            }
        }
    } /* end of the 2-tuples loop */

    nbits = add(nbits, target);

    /* Output */
    if (*stop)
    {
        nbits = nbits_old;
        move16();
    }

    if (stop2)
    {
        stop2 = nbits;
        move16();
    }
    *nEncoded = lastnz2;
    move16();
    *stop = stop2;
    move16();
    if (lastnz_out)
    {
        *lastnz_out = lastnz;
        move16();
    }

    IF (hm_cfg)
    {
        /* Restore hole indices beyond lastnz */
        FOR (k=numHoleIndices; k<hm_cfg->numHoleIndices; ++k)
        {
            hm_cfg->holeIndices[k] = sub(hm_cfg->holeIndices[k], nt);
            move16();
        }
        hm_cfg->numPeakIndices = numPeakIndicesOrig;
        move16();
    }


    return nbits_old;
}


