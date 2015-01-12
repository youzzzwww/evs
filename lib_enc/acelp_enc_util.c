/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <memory.h>
#include <assert.h>
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"


#define _1_Q9 0x200


/*
 * E_ACELP_toeplitz_mul
 *
 * Parameters:
 *   R          I: coefficients of Toeplitz matrix    (Q9)
 *   c          I: input vector Q_xn
 *   d          O: output vector, exponent = j
 *
 * Function:
 *   Multiplication of Toeplitz matrix with vector c, such that
 *      d = toeplitz(R)*c
 *   Vector length is L_SUBFR
*/
Word16 E_ACELP_toeplitz_mul(const Word16 R[], const Word16 c[], Word16 d[], const Word16 L_subfr, const Word16 highrate)
{
    static const Word16 step = 4;
    Word16 k, j, i;
    Word32 s;
    Word32 y32[L_SUBFR16k], L_maxloc, L_tot;


    assert(L_subfr <= L_SUBFR16k);


    /* first keep the result on 32 bits and find absolute maximum */
    L_tot = L_deposit_l(1);

    FOR (k = 0; k < step; k++)
    {
        L_maxloc = L_deposit_l(0);
        FOR (i = k; i < L_subfr; i += step)
        {
            s = L_mult(R[i], c[0]);
            FOR (j = 1; j < i; j++)
            {
                s = L_mac(s, R[i-j], c[j]);
            }
            FOR (; j<L_subfr; j++)
            {
                s = L_mac(s, R[j-i], c[j]);
            }

            y32[i] = s;
            move32();
            s = L_abs(s);
            L_maxloc = L_max(s, L_maxloc);
        }
        /* tot += 3*max / 8 */
        L_maxloc = L_shr(L_maxloc, 2);
        /* Do not warn saturation of L_tot, since its for headroom estimation. */
        BASOP_SATURATE_WARNING_OFF
        L_tot = L_add(L_tot, L_maxloc);           /* +max/4 */
        L_maxloc = L_shr(L_maxloc, 1);
        L_tot = L_add(L_tot, L_maxloc);           /* +max/8 */
        if ( highrate )
        {
            L_tot = L_add(L_tot, L_maxloc);       /* +max/8 */
        }
        L_maxloc = L_shr(L_maxloc, 1);
        if ( highrate )
        {
            L_tot = L_add(L_tot, L_maxloc);       /* +max/16 */
        }
        BASOP_SATURATE_WARNING_ON
    }

    /* Find the number of right shifts to do on y32[] so that    */
    /* 6.0 x sumation of max of dn[] in each track not saturate. */
    /* high-rate: 9.0 x sumation of max of dn[] in each track    */

    /* Limit exponent to avoid overflows elsewhere. */
    j = s_min(sub(norm_l(L_tot), 4+16), 15-16); /* 4 -> 16 x tot */

    Copy_Scale_sig_32_16(y32, d, L_subfr, j);

    return j;
}

void E_ACELP_weighted_code(
    const Word16 code[], /* i: code             */
    const Word16 H[],    /* i: impulse response */
    Word16 Q,            /* i: Q format of H    */
    Word16 y[]           /* o: weighted code    */
)
{
    Word16 i, j, k, one, n, nz[L_SUBFR];
    Word32 L_tmp;

    /* Collect nonzeros */
    n = 0;
    move16();
    FOR (i=0; i<L_SUBFR; ++i)
    {
        if (code[i] != 0)
        {
            nz[n++] = i;
            move16();
        }
    }
    assert(n > 0);

    one = shl(1, Q);
    Q = sub(15, Q);

    /* Generate weighted code */
    j = nz[0];
    move16();
    set16_fx(y, 0, j);
    FOR (k=0; k<L_SUBFR-j; k++)
    {
        L_tmp = L_mult(code[j], H[k]);
        y[j+k] = extract_h(L_shl(L_tmp, Q));
    }

    FOR (i=1; i<n; ++i)
    {
        j = nz[i];
        move16();
        FOR (k=0; k<L_SUBFR-j; k++)
        {
            L_tmp = L_mult(y[j+k], one);
            L_tmp = L_mac(L_tmp, code[j], H[k]);
            y[j+k] = extract_h(L_shl(L_tmp, Q));
        }
    }
}

void E_ACELP_conv(
    const Word16 xn2[], /* i */
    const Word16 h2[],  /* i */
    Word16 cn2[]        /* o */
)
{
    Word16 i, k;
    Word32 L_tmp;

    FOR (k=0; k<L_SUBFR; k++)
    {
        /*cn2[k] = xn2[k];     */
        L_tmp = L_mult(xn2[k], 0x800);
        FOR (i=0; i<k; i++)
        {
            /*cn2[k]-=cn2[i]*h2[k-i];*/
            L_tmp = L_msu0(L_tmp,cn2[i],h2[k-i]);  /*h2 4Q11*/
        }
        cn2[k] = round_fx(L_shl(L_tmp,5));
    }
}

void E_ACELP_build_code(
    Word16 nb_pulse,       /* i */
    const Word16 codvec[], /* i */
    const Word16 sign[],   /* i */
    Word16 code[],         /* o */
    Word16 ind[]           /* o */
)
{
    Word16 i, k, val, index, track, tmp, vec[4];

    set16_fx(code, 0, L_SUBFR);
    set16_fx(ind, -1, NPMAXPT * 4);

    /* use vec to store point counter */
    vec[0] = NPMAXPT*0-1;
    move16();
    vec[1] = NPMAXPT*1-1;
    move16();
    vec[2] = NPMAXPT*2-1;
    move16();
    vec[3] = NPMAXPT*3-1;
    move16();

    FOR (k = 0; k<nb_pulse; ++k)
    {
        i = codvec[k];  /* read pulse position  */                              move16();
        val = sign[i];  /* read sign            */                              move16();

        index = shr(i, 2);      /* pos of pulse (0..15) */
        track = s_and(i, 4-1);  /* i % 4;  */

        tmp = add(code[i], _1_Q9);
        if (val <= 0)
        {
            tmp = sub(code[i], _1_Q9);
        }
        code[i] = tmp;
        move16();

        if (val <= 0)
        {
            index = add(index, 16);
        }

        /* Calculate Current Store Index (we started at -1) so we increment first */
        i = add(vec[track], 1);
        /* Save Next Store Index */
        vec[track] = i;
        move16();

        ind[i] = index;
        move16();
    }
}

void E_ACELP_setup_pulse_search_pos(
    const PulseConfig *config, /* i: pulse configuration    */
    Word16 k,                  /* i: interation number      */
    UWord8 ipos[]              /* o: pulse search positions */
)
{
    Word16 restpulses, iPulse;

    /* copy search order from hash-table */
    assert(config->nb_pulse+(k*4) <= 40);

    copyWord8((const Word8*)E_ROM_tipos+(k * 4), (Word8*)ipos, config->nb_pulse);

    /* if all tracks do not have equal number of pulses */
    restpulses = s_and(config->nb_pulse, 3);

    IF (restpulses)
    {
        SWITCH (config->codetrackpos)
        {
        case TRACKPOS_FIXED_FIRST:  /* fixed track positions, starting from left */
            /* add tracks from left */
            FOR (iPulse=0; iPulse<restpulses; iPulse++)
            {
                ipos[config->nb_pulse-restpulses+iPulse] = (UWord8)iPulse;
                move16();
            }
            /* Put the same track on the next position, because the 1-pulse search
             * will access it to determine if this could be in any track. */
            ipos[config->nb_pulse] = ipos[config->nb_pulse-1];
            move16();
            BREAK;
        case TRACKPOS_FIXED_EVEN:  /* fixed track positions, odd tracks */
            /* odd tracks, switch order for every iteration */
            ipos[config->nb_pulse-restpulses] = (UWord8)s_and(lshl(k,1),2);
            move16();/* 0 for even k, 2 for odd */
            ipos[config->nb_pulse-restpulses+1] = (UWord8)s_xor(ipos[config->nb_pulse-restpulses], 2);
            move16();/* 2 for even k, 0 for odd */
            BREAK;
        case TRACKPOS_FIXED_TWO:  /* two tracks instead of four */
            /* Put the next track on the next position, because the 1-pulse search
             * will access it to determine if this could be in any track. */
            ipos[config->nb_pulse] = (UWord8)s_and(add(ipos[config->nb_pulse-1],1),3);
            move16();
            BREAK;
        default:              /* one or three free track positions */
            /* copy an extra position from table - 1pulse search will access this */
            ipos[config->nb_pulse] = E_ROM_tipos[add(shl(k,2),config->nb_pulse)];
            move16();
            BREAK;
        }
    }
}

