/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <memory.h>
#include <assert.h>
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "rom_enc_fx.h"

#define _1_Q11 (FL2WORD16_SCALE(1.0f, 15-11)) /* 1.0f in 4Q11 */

static void E_ACELP_update_cor(
    const Word16 pos[],    /* i */
    Word16 nb_pulse,       /* i */
    const Word16 sign[],   /* i */
    const Word16 R[],      /* i */
    const Word16 cor_in[], /* i */
    Word16 cor_out[]       /* o */
)
{
    Word16 sign_x, sign_y;
    const Word16 *pRx, *pRy;
    Word16 i, tmp;

    IF (sub(nb_pulse, 2) == 0)
    {
        /* Update product of autocorrelation and already fixed pulses. with the
         * two newly found ones */
        sign_x = sign[pos[0]];
        move16();
        sign_y = sign[pos[1]];
        move16();

        IF (s_xor(sign_x, sign_y) < 0)
        {
            i = 1;
            move16();
            if (sign_x > 0)
            {
                i = 0;
                move16();
            }
            pRx = R-pos[i];
            pRy = R-pos[1-i];
            /* different sign x and y */
            FOR (i=0; i<L_SUBFR; i++)
            {
                tmp = sub(pRx[i], pRy[i]);
                if (cor_in != NULL)
                {
                    tmp = add(tmp, cor_in[i]);
                }
                cor_out[i] = tmp;
                move16();
            }
        }
        ELSE
        {
            pRx = R-pos[0];
            pRy = R-pos[1];
            IF (sign_x > 0)
            {
                /* sign x and y is positive */
                FOR (i=0; i<L_SUBFR; i++)
                {
                    tmp = add(pRx[i], pRy[i]);
                    if (cor_in != NULL)
                    {
                        tmp = add(tmp, cor_in[i]);
                    }
                    cor_out[i] = tmp;
                    move16();
                }
            }
            ELSE
            {
                /* sign x and y is negative */
                FOR (i=0; i<L_SUBFR; i++)
                {
                    tmp = add(pRx[i], pRy[i]);
                    if (cor_in != NULL)
                    {
                        tmp = sub(cor_in[i], tmp);
                    }
                    if (cor_in == NULL)
                    {
                        tmp = negate(tmp);
                    }
                    cor_out[i] = tmp;
                    move16();
                }
            }
        }
    }
    ELSE IF (sub(nb_pulse, 4) == 0)
    {
        E_ACELP_update_cor(pos, 2, sign, R, cor_in, cor_out);
        E_ACELP_update_cor(pos+2, 2, sign, R, cor_out, cor_out);
    }
    else
    {
        assert(!"Number of pulses not supported");
    }
}


/* Iterations: nb_pos_ix*16 */
static void E_ACELP_2pulse_searchx(Word16 nb_pos_ix, Word16 track_x,
                                   Word16 track_y, Word16 *R, Word16 *ps, Word16 *alp,
                                   Word16 *ix, Word16 *iy, Word16 dn[],
                                   Word16 *dn2, Word16 cor[], Word16 sign[], Word16 sign_val_2)
{
    Word16 i,x;
    Word32 y;
    Word16 *pos_x, pos[2];
    Word32 xy_save;
    Word16 ps0, ps1, alp2_16, ps2, sq;
    Word32 alp0, alp1, alp2, s;
    Word16 *pR, sgnx;
    Word16 sqk[2], alpk[2], ik;


    /* eight dn2 max positions per track */
    pos_x = &dn2[shl(track_x,3)];
    move16();
    /* save these to limit memory searches */
    ps0 = *ps;
    move16();
    /*alp0 = *alp + 2.0f*R[0];                         move16();*/
    alp0 = L_deposit_h(*alp);               /* Qalp = Q_R*Q_signval */
    alp0 = L_mac(alp0, R[0], sign_val_2);

    /* Ensure that in the loop below s > 0 in the first iteration, the actual values do not matter. */
    sqk[0] = -1;
    move16();
    alpk[0] = 1;
    move16();
    x = pos_x[0];
    move16();
    sgnx = sign[track_y];
    move16();
    if (sign[x] < 0)
    {
        sgnx = negate(sgnx);
    }
    if (mac_r(L_mac(L_mac(alp0, cor[x], sign[x]), cor[track_y], sign[track_y]), R[track_y-x], sgnx) < 0)
    {
        sqk[0] = 1;
        move16();
    }
    ik = 0;
    move16();

    xy_save = L_mac0(L_deposit_l(track_y), track_x, L_SUBFR);

    /* loop track 1 */
    FOR (i=0; i<nb_pos_ix; i++)
    {
        x = pos_x[i];
        move16();
        sgnx = sign[x];
        move16();
        /* dn[x] has only nb_pos_ix positions saved */
        /*ps1 = ps0 + dn[x];                            INDIRECT(1);ADD(1);*/
        ps1 = add(ps0, dn[x]);
        /*alp1 = alp0 + 2*sgnx*cor[x];                  INDIRECT(1);MULT(1); MAC(1);*/
        alp1 = L_mac(alp0, cor[x], sgnx); /* Qalp = (Q_R=Q_cor)*Q_signval */

        pR = R-x;

        FOR (y = track_y; y < L_SUBFR; y += 4)
        {
            /*ps2 = ps1 + dn[y];                         ADD(1);*/
            ps2 = add(ps1, dn[y]);

            /*alp2 = alp1 + 2.0f*sign[y]*(cor[y] + sgnx*pR[y]);   MULT(1); MAC(2);*/
            /*alp2 = alp1 + 2.0f*sign[y]*cor[y] + 2.0f*sign[y]*sgnx*pR[y];   MULT(1); MAC(2);*/
            assert(sign[y] == sign_val_2 || sign[y] == -sign_val_2);

            /* Compiler warning workaround (not instrumented) */
            assert(sgnx != 0);
            alp2_16 = 0;

            alp2 = L_mac(alp1, cor[y], sign[y]); /* Qalp = (Q_R=Q_cor)*Q_signval */
            if (sgnx > 0)
            {
                alp2_16 = mac_r(alp2, pR[y], sign[y]); /* Qalp = (Q_R=Q_cor)*Q_signval */
            }
            if (sgnx < 0)
            {
                alp2_16 = msu_r(alp2, pR[y], sign[y]);	/* Qalp = (Q_R=Q_cor)*Q_signval */
            }
            alpk[1-ik] = alp2_16;
            move16();

            /*sq = ps2 * ps2;                            MULT(1);*/
            sq = mult_r(ps2, ps2);	/* (3+3)Q -> 6Q9 */
            sqk[1-ik] = sq;
            move16();


            /*s = (alpk * sq) - (sqk * alp2);            MULT(1);MAC(1);*/
            s = L_msu(L_mult(alpk[ik], sq), sqk[ik], alp2_16);	/* Q_sq = Q_sqk, Q_alpk = Q_alp */
            if (s > 0)
            {
                ik = sub(1, ik);
            }
            if (s > 0)
            {
                xy_save = L_mac0(y, x, L_SUBFR);
            }
            assert( ((s >= 0 && i==0 && y == track_y)) || (y > track_y) || (i > 0));
        }
    }
    ps1 = extract_l(xy_save);
    pos[1] = s_and(ps1, L_SUBFR-1);
    move16();
    pos[0] = lshr(ps1, 6);
    move16();
    /* Update numerator */
    *ps = add(add(ps0,dn[pos[0]]),dn[pos[1]]);
    move16();

    /* Update denominator */
    *alp = alpk[ik];
    move16();

    E_ACELP_update_cor(pos, 2, sign, R, cor, cor);

    *ix = pos[0];
    move16();
    *iy = pos[1];
    move16();

    assert(((pos[0] & 3) == track_x) && ((pos[1] & 3) == track_y)); /* sanity check */
}


/* static */
static void E_ACELP_1pulse_searchx(UWord8 tracks[2],
                                   Word16 *R, Word16 *ps, Word16 *alp,
                                   Word16 *ix, Word16 dn[],
                                   Word16 cor[], Word16 sign[], Word16 sign_val_1)
{
    Word16 x, x_save = 0;
    Word16 ps0;
    Word32 alp0;
    Word16 ps1, sq;
    Word16 alp1;
    Word32 s;
    Word16 ntracks, t;
    Word16 sqk[2], alpk[2], ik;

    /* save these to limit memory searches */
    /*alp0 = *alp + R[0];                              INDIRECT(1);*/
    ps0 = *ps;
    move16();
    alp0 = L_deposit_h(*alp);
    alp0 = L_mac(alp0, R[0], sign_val_1);    /* Qalp = (Q_R=Q_cor)*Q_signval */

    /* Ensure that in the loop below s > 0 in the first iteration, the actual values do not matter. */
    move16();
    move16();
    alpk[0] = 1;
    sqk[0] = -1;
    ik = 0;
    move16();
    if (mac_r(alp0, cor[tracks[0]], sign[tracks[0]]) < 0)
    {
        sqk[0] = 1;
        move16();
    }

    x_save = tracks[0];
    move16();

    ntracks = 1;
    if (sub(tracks[1], tracks[0]) != 0)
    {
        ntracks = 2;
        move16();
    }
    FOR (t=0; t<ntracks; ++t)
    {
        FOR (x = tracks[t]; x < L_SUBFR; x += 4)
        {
            /* ps1 = ps0 + dn[x];                             ADD(1);*/
            ps1 = add(ps0, dn[x]);
            /* alp1 = alp0 + 2*sign[x]*cor[x];                MAC(1); MULT(1);*/
            assert(sign[x] == sign_val_1<<1 || sign[x] == -sign_val_1<<1);
            alp1 = mac_r(alp0, cor[x], sign[x]);  /* Qalp = (Q_R=Q_cor)*Q_signval */
            alpk[1-ik] = alp1;
            move16();


            /*sq = ps1 * ps1;                                MULT(1);*/
            sq = mult_r(ps1, ps1);   /* 6Q9 */
            sqk[1-ik] = sq;
            move16();

            /*s = (alpk[ik] * sq) - (sqk[ik] * alp1);                MULT(1);MAC(1);*/
            s = L_msu(L_mult(alpk[ik], sq), sqk[ik], alp1);

            if (s > 0)
            {
                ik = sub(1, ik);
            }
            if (s > 0)
            {
                x_save = x;
                move16();
            }
            assert( t>0 || ((s >= 0) && (x == tracks[t])) || x > tracks[t]);
        }
    }

    *ps = add(ps0, dn[x_save]);
    move16();
    *alp = alpk[ik];
    move16();
    *ix = x_save;
    move16();
}


/* Autocorrelation method for searching pulse positions effectively
 * Algorithm is identical to traditional covariance method. */
void E_ACELP_4tsearchx(Word16 dn[], const Word16 cn[], Word16 Rw[], Word16 code[], const PulseConfig *config, Word16 ind[])
{
    Word16 sign[L_SUBFR], vec[L_SUBFR];
    Word16 cor[L_SUBFR];
    Word16 R_buf[2*L_SUBFR-1], *R;
    Word16 dn2[L_SUBFR];
    Word16 ps2k, ps /* same format as dn[] */, ps2, alpk, alp = 0 /* Q13 and later Q_Rw*Q_signval=Q_cor*Q_signval */;
    Word32 s;
    Word16 codvec[NB_PULSE_MAX];
    Word16 pos_max[4];
    Word16 dn2_pos[8 * 4];
    UWord8 ipos[NB_PULSE_MAX];
    Word16 i, j, k, st, pos = 0;
    Word16 scale;
    Word16 sign_val_1, sign_val_2;
    Word16 nb_pulse, nb_pulse_m2;

    ps = 0;      /* to avoid compilation warnings */



    alp = config->alp; /* Q13 */                                                move16();
    nb_pulse = config->nb_pulse;
    move16();
    nb_pulse_m2 = sub(nb_pulse, 2);

    /* Init to avoid crash when the search does not find a solution */
    FOR (k=0; k<nb_pulse; k++)
    {
        codvec[k] = k;
        move16();
    }

    scale = 0;
    move16();
    s = L_mult0(Rw[0], Rw[0]);
    FOR (i = 1; i < L_SUBFR; i++)
    {
        s = L_mac0(s, Rw[i], Rw[i]);
    }
    if (s_and(sub(nb_pulse, 9) >= 0, L_sub(s, 0x800000) > 0))
    {
        scale = -1;
        move16();
    }
    if (s_and(sub(nb_pulse, 13) >= 0, L_sub(s, 0x4000000) > 0))
    {
        scale = -2;
        move16();
    }
    IF (sub(nb_pulse, 18) >= 0)
    {
        if (L_sub(s, 0x200000) > 0)
        {
            scale = -1;
            move16();
        }
        if (L_sub( s, 0x400000 ) > 0)
        {
            scale = -2;
            move16();
        }
        if (L_sub( s, 0x4000000 ) > 0)
        {
            scale = -3;
            move16();
        }
    }
    if (s_and(sub(nb_pulse, 28) >= 0, L_sub(s, 0x800000) > 0))
    {
        scale = -3;
        move16();
    }
    if (s_and(sub(nb_pulse, 36) >= 0, L_sub(s, 0x4000000) > 0))
    {
        scale = -4;
        move16();
    }

    /* Set up autocorrelation vector */
    R = R_buf+L_SUBFR-1;
    Copy_Scale_sig(Rw, R, L_SUBFR, scale);
    FOR (k=1; k<L_SUBFR; k++)
    {
        R[-k] = R[k];
        move16();
    }

    /* Sign value */
    sign_val_2 = 0x2000;
    move16();
    if (sub(nb_pulse, 24) >= 0)
    {
        sign_val_2 = shr(sign_val_2, 1);
    }
    sign_val_1 = shr(sign_val_2, 1);

    /*
     * Find sign for each pulse position.
     */
    E_ACELP_pulsesign(cn, dn, dn2, sign, vec, alp, sign_val_2, L_SUBFR);

    /*
     * Select the most important 8 position per track according to dn2[].
     */
    E_ACELP_findcandidates(dn2, dn2_pos, pos_max);

    /*
     * Deep first search:
     */

    /* Ensure that in the loop below s > 0 in the first iteration, the actual values do not matter. */
    ps2k = -1;
    move16();
    alpk = 1;
    move16();

    /* Number of iterations */
    FOR (k = 0; k < config->nbiter; k++)
    {
        E_ACELP_setup_pulse_search_pos(config, k, ipos);

        /* index to first non-fixed position */
        pos = config->fixedpulses;
        move16();

        IF (config->fixedpulses == 0)/* 1100, 11, 1110, 1111, 2211 */
        {
            ps = 0;
            move16();
            alp = 0;
            move16();
            set16_fx(cor, 0, L_SUBFR);
        }
        ELSE
        {
            assert(config->fixedpulses == 2 || config->fixedpulses == 4);

            /* set fixed positions */
            FOR (i=0; i<pos; ++i)
            {
                ind[i] = pos_max[ipos[i]];
                move16();
            }

            /* multiplication of autocorrelation with signed fixed pulses */
            E_ACELP_update_cor(ind, config->fixedpulses, sign, R, NULL, cor);

            /* normalisation contribution of fixed part */
            s = L_mult0(cor[ind[0]], sign[ind[0]]);
            ps = dn[ind[0]];
            move16();
            FOR (i=1; i<pos; ++i)
            {
                s = L_mac0(s, cor[ind[i]], sign[ind[i]]);   /*Q12+Q9+1=Q6 */
                ps = add(ps, dn[ind[i]]);
            }
            alp = round_fx(s);                          /*mac0 >>1 sign = 2*/
        }

        /* other stages of 2 pulses */
        st = 0;
        move16();
        FOR (j = pos; j < nb_pulse; j += 2)
        {
            IF (sub(nb_pulse_m2, j) >= 0) /* pair-wise search */
            {
                /*
                 * Calculate correlation of all possible positions
                 * of the next 2 pulses with previous fixed pulses.
                 * Each pulse can have 16 possible positions.
                 */

                E_ACELP_2pulse_searchx(config->nbpos[st], ipos[j], ipos[j + 1], R, &ps, &alp,
                                       &ind[j], &ind[j+1], dn, dn2_pos, cor, sign, sign_val_2);

            }
            ELSE /* single pulse search */
            {
                E_ACELP_1pulse_searchx(&ipos[j], R, &ps, &alp,
                &ind[j], dn, cor, sign, sign_val_1);
            }


            st = add(st, 1);
        }

        /* memorise the best codevector */
        /*ps2 = ps * ps;                                            MULT(1);*/
        ps2 = mult(ps, ps);

        /*s = (alpk * ps2) - (ps2k * alp);                          MULT(2);ADD(1);*/
        s = L_msu(L_mult(alpk, ps2), ps2k, alp);

        IF (s > 0)
        {
            ps2k = ps2;
            move16();
            alpk = alp;
            move16();
            Copy(ind, codvec, nb_pulse);
        }
    }


    /*
     * Store weighted energy of code, build the codeword and index of codevector.
     */
    E_ACELP_build_code(nb_pulse, codvec, sign, code, ind);
}
