/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "prot_fx.h"    /* Function prototypes                    */
#include "basop_util.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define DIST_ISF_MAX      307  /* 120 Hz (6400Hz=16384) */
#define DIST_ISF_THRES    154  /* 60     (6400Hz=16384) */
#define GAIN_PIT_THRES  14746  /* 0.9 in Q14 */
#define GAIN_PIT_MIN     9830  /* 0.6 in Q14 */

#define ALPHA1          32113  /* 0.98f */
#define ALPHA4          32440  /* 0.99f */
#define WINDOW_SIZE        50
#define THRESH_TYPE     13926  /* 0.85f in Q14 */
#define THRESH_VOICING  14090  /* 0.86f in Q14 */

#define GPCLIP_E (6+2)

#define ALPHA1_M1       FL2WORD32_SCALE(1.0f-0.98,1)
#define ALPHA4_M1       FL2WORD32_SCALE(1.0f-0.99f,1)

/*-------------------------------------------------------------------*
 * init_gp_clip
 *
 * Pitch Gain clipping initializations
 *-------------------------------------------------------------------*/
void init_gp_clip_fx(
    Word16 mem[]      /* o  : memory of gain of pitch clipping algorithm */
)
{
    mem[0] = DIST_ISF_MAX;
    move16(); /* Q0 */
    mem[1] = GAIN_PIT_MIN;
    move16(); /* 1Q14 */
    mem[2] = 0;
    move16(); /* 8Q7 */ /* old energy of target (dB) */
    mem[3] = 0;
    move16(); /* Q0 */
    mem[4] = 0;
    move16(); /* Q14 */
    mem[5] = 13107;/*0.8*/ move16(); /* Q14 */

    return;
}

/*-------------------------------------------------------------------*
 * Function gp_clip
 *
 * The gain needs to be limited (gain pitch < 1.0) when one of the
 * following cases occurs:
 * - a resonance on LPC filter (lp_disp < 60 Hz)  AND a good pitch
 *   prediction (lp_gp > 0.9)
 * - target energy drops by 6 dB AND a good pitch prediction (lp_gp>1.0)
 *-------------------------------------------------------------------*/

Word16 gp_clip_fx(
    const Word16 *voicing,   /* i  : normalized correlations (from OL pitch)    */
    const Word16 i_subfr,    /* i  : subframe index                             */
    const Word16 coder_type, /* i  : type of coder                              */
    const Word16 xn[],       /* i  : target vector                              */
    Word16 mem[],      /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 Q_new       /* i  : scaling factor                             */
)
{
    Word16 clip;
    Word16 i, wener;
    Word16 e_ener, f_ener;
    Word32 ener;
    Word32 L_tmp;

    clip = 0;
    move16();
    test();
    if (sub(mem[0], DIST_ISF_THRES) < 0 && sub(mem[1], GAIN_PIT_THRES) > 0)
    {
        clip = 1;
        move16();
    }

    ener = L_mac(1L, xn[0], xn[0]);
    FOR (i=1; i<L_SUBFR; i++)
    {
        ener = L_mac(ener, xn[i], xn[i]);
    }

    /* ener = 10.0f*(float)log10(ener) */
    e_ener = norm_l(ener);
    f_ener = Log2_norm_lc(L_shl(ener, e_ener));
    e_ener = sub(30, e_ener);
    e_ener = sub(e_ener, Q_new);
    ener = Mpy_32_16(e_ener, f_ener, LG10);
    wener  = round_fx(L_shl(ener, 10));

    test();
    if (sub(wener, sub(mem[2], 1536)) < 0 && sub(mem[1], 16384) > 0)
    {
        clip = 1;
        move16();
    }

    mem[2] = wener;
    move16();

    L_tmp = L_mult(ALPHA1, mem[4]);

    test();
    test();
    if (sub(coder_type,GENERIC) == 0 || sub(coder_type,TRANSITION) == 0 || sub(coder_type,INACTIVE) == 0 )
    {
        /* mem[4] = (1-ALPHA1) + ALPHA1 * mem[4], if branch taken */
        /* mem[4] = ALPHA1 * mem[4], otherwise */
        L_tmp = L_add(L_tmp, 32768L*(32768-ALPHA1));
    }
    mem[4] = round_fx(L_tmp);

    L_tmp = L_mult(ALPHA4, mem[5]);
    if (i_subfr == 0)
    {
        /* mem[5] = (1-ALPHA4) * voicing[0] + ALPHA4 * mem[5] */
        mem[5] = mac_r(L_tmp, (32768-ALPHA4)/2, voicing[0]);
        move16(); /* /2 to put voicing from Q15 to Q14 */
    }

    if (sub(i_subfr, 2*L_SUBFR) == 0)
    {
        /* mem[5] = (1-ALPHA4) * voicing[1] + ALPHA4 * mem[5] */
        mem[5] = mac_r(L_tmp, (32768-ALPHA4)/2, voicing[1]);
        move16(); /* /2 to put voicing from Q15 to Q14 */
    }

    IF (sub(mem[3], WINDOW_SIZE) > 0)
    {
        test();
        if (sub(mem[4], THRESH_TYPE) > 0 && sub(mem[5], THRESH_VOICING) > 0)
        {
            clip = 1;
            move16();
        }
    }
    ELSE
    {
        mem[3] = add(mem[3], 1);
        move16();
    }

    return (clip);
}

/*-------------------------------------------------------------------*
 * gp_clip_test_lsf()
 *
 * check the minimum distance of LSFs for pitch gain clipping flag
 *-------------------------------------------------------------------*/

void gp_clip_test_isf_fx(
    const Word16 isf[],         /* i  : isf values (in frequency domain)           */
    Word16 mem[],         /* i/o: memory of gain of pitch clipping algorithm */
    const Word16 Opt_AMR_WB     /* i  : flag indicating AMR-WB IO mode             */
)
{
    Word16 i, dist, dist_min, m;

    dist_min = sub(isf[1], isf[0]);

    m = M;
    move16();
    if ( sub(Opt_AMR_WB,1)==0 )
    {
        m = M-1;
        move16();
    }

    move16(); /* ptr init*/
    FOR (i = 2; i < m; i++)
    {
        dist = sub(isf[i], isf[i - 1]);
        dist_min = s_min(dist, dist_min);
    }

    dist = extract_h(L_mac(L_mult(26214, mem[0]), 6554, dist_min));

    dist = s_min(dist, DIST_ISF_MAX);
    mem[0] = dist;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * gp_clip_test_gain_pit()
 *
 * low-pass filtering of the pitch gain for pitch gain clipping flag
 *-------------------------------------------------------------------*/

void gp_clip_test_gain_pit_fx(
    const Word16 gain_pit,  /* i  : gain of quantized pitch                     Q14 */
    Word16 mem[]      /* i/o: memory of gain of pitch clipping algorithm 1Q14 */
)
{
    Word16 gain;
    Word32 L_tmp;

    L_tmp = L_mult(29491, mem[1]);
    L_tmp = L_mac(L_tmp, 3277, gain_pit);
    gain = extract_h(L_tmp);
    gain = s_max(gain, GAIN_PIT_MIN);
    mem[1] = gain;
    move16();

    return;
}


/*-------------------------------------------------------------------*
 * Function gp_clip
 *
 * The gain needs to be limited (gain pitch < 1.0) when one of the
 * following cases occurs:
 * - a resonance on LPC filter (lp_disp < 60 Hz)  AND a good pitch
 *   prediction (lp_gp > 0.9)
 * - target energy drops by 6 dB AND a good pitch prediction (lp_gp>1.0)
 *-------------------------------------------------------------------*/
Word16 Mode2_gp_clip(
    const Word16 voicing[3],  /* i  : normalized correlations from OL pitch  Q15 */
    const Word16 i_subfr,     /* i  : subframe index                             */
    const Word16 coder_type,  /* i  : type of coder                              */
    const Word16 xn[],        /* i  : target vector                         Q_xn */
    Word16 mem[],       /* i/o: memory of gain of pitch clipping algorithm */
    /*      mem[0]:      Q0                            */
    /*      mem[1]:     1Q14                           */
    /*      mem[2]:     8Q7                            */
    /*      mem[3]:      Q0   (integer)                */
    /*      mem[4]:      Q14                           */
    /*      mem[5]:      Q14                           */
    const Word16 L_subfr,
    const Word16 Q_xn         /* i  : scaling factor of vector xn[]              */
)
{
    Word16 clip, tmp, exp_xn;
    Word16 i;
    Word32 wener, Ltmp;

    move16();
    clip = 0;

    test();
    if ((sub(mem[0],DIST_ISF_THRES) < 0) && (sub(mem[1],GAIN_PIT_THRES) > 0))
    {
        move16();
        clip = 1;
    }

    /*ener_exp = exp_xn * 2 + 1*/
    exp_xn = add(shl(sub(15,Q_xn),1), 1);
    wener = L_shr(FL2WORD32(0.01f), s_min(31,exp_xn));
    wener = L_max(1,wener);

    FOR (i=0; i<L_subfr; i++)
    {
        wener = L_mac0(wener, xn[i], xn[i]);
    }

    /*wener = 10.0f*(float)log10(wener);*/
    wener = BASOP_Util_Log2(wener);
    wener = L_add(wener,L_shl(exp_xn,31-LD_DATA_SCALE));
    wener = Mpy_32_16_1(wener, LG10);   /* wener in 8Q7 */
#if (GPCLIP_E != 6+2)
    wener = shl(wener, GPCLIP_E-(6+2));
#endif
    tmp = round_fx(wener);
    /* exponent of wener = 6+2 */

    test();
    if (sub(tmp, sub(mem[2], FL2WORD16_SCALE(6.0f,GPCLIP_E))) < 0 &&
            sub(mem[1],FL2WORD16_SCALE(1.0f,1)) > 0)
    {
        move16();
        clip = 1;
    }

    move16();
    mem[2] = tmp;                      /* wener  in 8Q7 format */
    Ltmp = Mpy_32_16_1(ALPHA1, mem[4]);  /* mem[4] in Q14 format, Ltmp in Q14 */

    if( s_or(sub(coder_type,GENERIC) == 0, sub(coder_type,TRANSITION) == 0) )
    {
        Ltmp = L_add(Ltmp, ALPHA1_M1);
    }
    mem[4] = round_fx(Ltmp);

    Ltmp = Mpy_32_16_1(ALPHA4, mem[5]);  /* mem[5] in Q14 format, Ltmp in Q14 */
    IF( i_subfr == 0 )
    {
        move16();                        /* voicing: Q15 */
        mem[5] = round_fx(L_add(Mpy_32_16_1(ALPHA4_M1, voicing[0]), Ltmp));
    }
    ELSE IF( sub(i_subfr,shl(L_subfr,1)) == 0 )
    {
        move16();
        mem[5] = round_fx(L_add(Mpy_32_16_1(ALPHA4_M1, voicing[1]), Ltmp));
    }

    IF( sub(mem[3],WINDOW_SIZE) > 0 )
    {
        test();
        if( ( sub(mem[4],THRESH_TYPE) > 0 ) && ( sub(mem[5],THRESH_VOICING) > 0 ))
        {
            move16();
            clip = 1;
        }
    }
    ELSE
    {
        move16();
        mem[3] = add(mem[3], 1);
    }


    return (clip);
}

/*-------------------------------------------------------------------*
* gp_clip_test_lsf:
*
* check the minimum distance of LSFs for pitch gain clipping flag
*-------------------------------------------------------------------*/
void gp_clip_test_lsf_fx(
    const Word16 lsf[],     /* i  : lsf values (in frequency domain)	14Q1*1.28 */
    Word16 mem[],     /* i/o: memory of gain of pitch clipping algorithm  */
    const Word16 m			/* i  : dimension of lsf							*/
)
{
    Word16 i;
    Word16 dist, dist_min;

    dist_min = sub(lsf[1],lsf[0]);

    FOR (i=2; i<m-1; i++)
    {
        dist = sub(lsf[i],lsf[i-1]);
        dist_min = s_min(dist,dist_min);
    }
    /*dist = 0.8f*mem[0] + 0.2f*dist_min;*/
    dist = s_min(DIST_ISF_MAX,mac_r(L_mult(FL2WORD16(0.8f),mem[0]),FL2WORD16(0.2f),dist_min));

    mem[0] = dist;
    move16();


    return;
}
