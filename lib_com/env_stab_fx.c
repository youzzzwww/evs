/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "rom_com_fx.h"  /* Static table prototypes             */
#include "stl.h"         /* required by wmc_tool                       */

/*--------------------------------------------------------------------------*
 * Local constants
 *--------------------------------------------------------------------------*/
#define ENV_STAB_SMO_HO 10                      /* number of hangover frames when switching from music to speech state */

/*--------------------------------------------------------------------------*/
/*  Function  env_stability_fx                                              */
/*  ~~~~~~~~~~~~~~~~~~~~~                                                   */
/*                                                                          */
/*  Envelope stability measure                                              */
/*--------------------------------------------------------------------------*/

Word16 env_stability_fx(        /* in Q15 */
    const Word16 *ynrm,         /*i:   Norm vector for current frame */
    const Word16 nb_sfm,        /*i:   Number of sub-bands */
    Word16 *mem_norm,     /*i/o: Norm vector memory from past frame */
    Word16 *mem_env_delta /*i/o: Envelope stability memory for smoothing in Q12 */
)
{
    Word16 env_delta;
    Word16 env_stab;
    Word16 tmp, tmp_stab;
    Word16 i;

    Word16 exp, exp2;
    Word32 L_tmp, L_env_delta;
    Word16 inv_nb_sfm;

    /* Calculate envelope stability parameter */
    L_env_delta = L_deposit_l(0);
    FOR (i = 0; i < nb_sfm; i++)
    {
        tmp = sub(mem_norm[i],ynrm[i]);
        L_env_delta = L_mac0(L_env_delta, tmp, tmp);
        mem_norm[i] = ynrm[i];
        move16();
    }

    inv_nb_sfm = 19418; /* Q19 */   move16();
    if (nb_sfm == 26)
    {
        inv_nb_sfm = 20165; /* Q19 */   move16();
    }
    exp = norm_l(L_env_delta);
    L_env_delta = Mult_32_16(L_shl(L_env_delta, exp), inv_nb_sfm);  /* 0+exp+19-15 */

    L_tmp = Sqrt_l(L_env_delta, &exp2);     /* exp+4+31+exp2 */

    exp =  add(35, add(exp, exp2));
    if ( sub(s_and(exp, 1), 1) == 0 )
    {
        L_tmp = Mult_32_16(L_tmp, 23170);   /* 1/sqrt(2) in Q15 */
    }
    exp = shr(exp, 1);

    env_delta = round_fx(L_shl(L_tmp, sub(26, exp))); /* Q10 */

    L_tmp = L_mult0(26214, env_delta); /* 26214 is 0.1 in Q18. Q28 */
    L_tmp = L_mac(L_tmp, 29491, *mem_env_delta);   /* 29491 is 0.9 in Q15. Q28 */

    *mem_env_delta = round_fx(L_tmp);   /* Q12 */
    Overflow = 0;
    move16();
    env_delta = round_fx(L_shl(L_tmp, 1));   /* Q13 */

    IF (Overflow != 0) /* Saturated due to the above up-shifting operation. */
    {
        return stab_trans_fx[L_STAB_TBL-1];  /* The highest quantized index. */
    }

    /* If tmp_stab > (D_STAB_TBL*L_STAB_TBL + M_STAB_TBL), i.e., 0.103138*10+2.51757=3.603137,
     * the quantized index is equal to 9. Hence, we only need to worry about any tmpStab < 4.
     * In this case, Q13 is good enough.
     */
    tmp_stab = sub(env_delta, M_STAB_TBL_FX); /* in Q13 */
    tmp_stab = abs_s(tmp_stab);

    /* Table lookup for smooth transitions
     * First, find the quantization level, i, of tmpStab. */
#if L_STAB_TBL > 10
#error env_stability_fx: Use more efficient usquant()
#endif
    tmp_stab = sub(tmp_stab, HALF_D_STAB_TBL_FX); /* in Q13 */
    FOR (i = 0; i < L_STAB_TBL-1; i++)
    {
        IF (tmp_stab < 0)
        {
            BREAK;
        }
        ELSE
        {
            tmp_stab = sub(tmp_stab, D_STAB_TBL_FX); /* in Q13 */
        }
    }

    env_stab = stab_trans_fx[i];
    move16();
    if(sub(env_delta, M_STAB_TBL_FX) < 0)
    {
        env_stab = sub(0x7FFF,stab_trans_fx[i]);
    }

    return env_stab;
}

/*--------------------------------------------------------------------------*
 * env_stab_smo_fx()
 *
 *
 *--------------------------------------------------------------------------*/
Word16 env_stab_smo_fx(                         /* Q0 */
    Word16 env_stab,                            /*i  : env_stab value                        Q15 */
    Word16 *env_stab_state_p,                   /*i/o: env_stab state probabilities          Q15 */
    Word16 *ho_cnt                              /*i/o: hangover counter for speech state      */
)
{
    Word16 state, prev_state;
    Word16 maxval, pp[NUM_ENV_STAB_PLC_STATES], pa[NUM_ENV_STAB_PLC_STATES];
    Word16 i;
    Word16 tmp, sum, exp;

    /* get previous state */
    prev_state = maximum_fx(env_stab_state_p, NUM_ENV_STAB_PLC_STATES, &maxval);

    /* assume two states: speech(0), music(1) */
    /* set a posteriori likelihoods for the two states according to env_stab */
    /* re-scale. Unclear if needed */
    /* env_stab = (env_stab - stab_trans_fx[L_STAB_TBL-1])/(1-2*stab_trans_fx[L_STAB_TBL-1]); */
    tmp = sub(env_stab, stab_trans_fx[L_STAB_TBL-1]);
    tmp = round_fx(L_shl(L_mult(tmp, INV_STAB_TRANS_FX), 1)); /* Q15 */

    pp[0] = sub(32767, tmp);
    move16();   /* 1 in Q15 */
    pp[1] = tmp;
    move16();

    /* calculate a priori likelihoods */
    pa[0] = round_fx(Dot_product(env_stab_tp_fx[0], env_stab_state_p, NUM_ENV_STAB_PLC_STATES)); /* Q15*/
    pa[1] = round_fx(Dot_product(env_stab_tp_fx[1], env_stab_state_p, NUM_ENV_STAB_PLC_STATES));

    /* multiply elementwise with a posteriori likelihoods */
    sum = 0;
    move16();
    FOR (i = 0; i < NUM_ENV_STAB_PLC_STATES; i++)
    {
        env_stab_state_p[i] = mult_r(pa[i], pp[i]);
        move16();   /* Q15 */
        sum = add(sum, env_stab_state_p[i]);
    }

    /* renormalize state probabilities */
    exp = norm_s(sum);
    tmp = div_s(16384, shl(sum, exp)); /* Q(14-exp) */
    /*tmp = shl(tmp, add(exp, 1));*/ /* Q15 */
    FOR (i = 0; i < NUM_ENV_STAB_PLC_STATES; i++)
    {
        env_stab_state_p[i] = round_fx(L_shl(L_mult(env_stab_state_p[i], tmp), add(exp, 1))); /* Q15 */
    }

    /* find maximum index as return value */
    state = maximum_fx(env_stab_state_p, NUM_ENV_STAB_PLC_STATES, &maxval);

    /* apply some hangover for speech */
    test();
    if (state == 0 && sub(prev_state, 1) == 0)
    {
        *ho_cnt = ENV_STAB_SMO_HO;
        move16();
    }

    IF (*ho_cnt > 0)
    {
        *ho_cnt = sub(*ho_cnt, 1);
        move16();
    }

    return state;
}
