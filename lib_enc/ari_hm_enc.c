/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stl.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"
#include "prot_fx.h"

Word16 EncodeIndex(
    Word16 Bandwidth,
    Word16 PeriodicityIndex,
    Encoder_State_fx *st
)
{
    Word16 NumRatioBitsBwLtpIndx;

    IF ( s_and(PeriodicityIndex, kLtpHmFlag) != 0 )
    {
        Word16 LtpPitchIndex = shr(PeriodicityIndex, 9);
        assert(0 <= LtpPitchIndex && LtpPitchIndex <= 16);

        PeriodicityIndex = sub(PeriodicityIndex, 1);
        assert((PeriodicityIndex & 0xff) < (1 << NumRatioBits[Bandwidth][LtpPitchIndex]));

        NumRatioBitsBwLtpIndx = NumRatioBits[Bandwidth][LtpPitchIndex];
        move16();

        push_next_indice_fx(st, s_and(PeriodicityIndex, 0xff), NumRatioBitsBwLtpIndx);
        return NumRatioBitsBwLtpIndx;
    }
    ELSE
    {
        push_next_indice_fx(st, PeriodicityIndex, 8);
        return 8;
    }
}

static Word16 SearchPeriodicityIndex_Single(
    const Word16 AbsMdct3[],
    Word16 NumToConsider,
    Word32 Lag,
    Word16 FractionalResolution
)
{
    Word16 HighestMultiplier;
    Word32 AbsMeanCurrent3;        /* Mean for BucketWidth == 3 */
    Word32 Limit, OldIndex, i;
    Word16 Result, tmp1, tmp2;


    Limit = L_deposit_l(sub(NumToConsider, 1));
    Limit = L_shl(Limit, FractionalResolution);
    AbsMeanCurrent3 = L_deposit_l(0);
    HighestMultiplier = 0;
    move16();

    FOR (i=Lag; i<Limit; i+=Lag)
    {
        OldIndex = L_shr(i, FractionalResolution);
        tmp1 = Weight[s_min(HighestMultiplier, 85)];
        move16();
        AbsMeanCurrent3 = L_add(AbsMeanCurrent3, L_shr(L_mult(AbsMdct3[OldIndex], tmp1), 7));
        HighestMultiplier = add(HighestMultiplier, 1);
    }

    tmp1 = sub(norm_l(AbsMeanCurrent3), 1);
    tmp2 = norm_s(HighestMultiplier);
    Result = div_s( round_fx(L_shl(AbsMeanCurrent3, tmp1)), s_max(shl(HighestMultiplier, tmp2), 0x4000) );
    if (HighestMultiplier == 0)
    {
        tmp2 = 14 + 16;
        move16();
    }
    Result = shr(Result, s_min(15, sub(sub(tmp1, tmp2), 7-15)));

    return Result;
}

static void SearchPeriodicityIndex_Range(
    const Word16 AbsMdct3[],
    Word16 NumToConsider,
    Word16 Lo,
    Word16 Hi,
    Word16 FractionalResolution,
    Word16 Adj,
    Word16 Spacing,
    Word16 *PeriodicityIndex,
    Word16 *Score
)
{
    Word16 Index, BestIndex;
    Word16 CurrentScore, BestScore;
    Word16 B;

    BestScore = -1;
    move16();
    BestIndex = 0;
    move16();

    FOR (Index = Lo; Index < Hi; Index += Spacing)
    {
        CurrentScore = SearchPeriodicityIndex_Single(
                           AbsMdct3,
                           NumToConsider,
                           add(Index, Adj),
                           FractionalResolution
                       );

        if (sub(CurrentScore, BestScore) > 0)
        {
            BestIndex = Index;
            move16();
        }
        BestScore = s_max(BestScore, CurrentScore);
    }

    if (sub(BestScore, *Score) > 0)
    {
        *PeriodicityIndex = BestIndex;
        move16();
    }
    BestScore = s_max(BestScore, *Score);

    B = sub(BestIndex, shr(Spacing, 1));
    B = s_max(Lo, B);

    FOR (Index = B; Index < BestIndex; ++Index)
    {
        CurrentScore = SearchPeriodicityIndex_Single(
                           AbsMdct3,
                           NumToConsider,
                           add(Index, Adj),
                           FractionalResolution
                       );

        if (sub(CurrentScore, BestScore) > 0)
        {
            *PeriodicityIndex = Index;
            move16();
        }
        BestScore = s_max(BestScore, CurrentScore);
    }

    B = add(BestIndex, shr(Spacing, 1));

    FOR (Index = add(BestIndex, 1); Index <= B; ++Index)
    {
        CurrentScore = SearchPeriodicityIndex_Single(
                           AbsMdct3,
                           NumToConsider,
                           add(Index, Adj),
                           FractionalResolution
                       );

        if (sub(CurrentScore, BestScore) > 0)
        {
            *PeriodicityIndex = Index;
            move16();
        }
        BestScore = s_max(BestScore, CurrentScore);
    }

    *Score = BestScore;
    move16();
}

/* Returns: PeriodicityIndex */
Word16 SearchPeriodicityIndex(
    const Word32 Mdct[],               /* (I) Coefficients, Mdct[0..NumCoeffs-1]                      */
    const Word32 UnfilteredMdct[],     /* (I) Unfiltered coefficients, UnfilteredMdct[0..NumCoeffs-1] */
    Word16 NumToConsider,              /* (I) Number of coefficients                                  */
    Word16 TargetBits,                 /* (I) Target bit budget (excl. Done flag)                     */
    Word16 LtpPitchLag,
    Word16 LtpGain,                    /* (I) LTP gain                                                */
    Word16 *RelativeScore              /* (O) Energy concentration factor                      (2Q13) */
)
{
    Word16 AbsMdct3[MAX_LENGTH];
    Word32 A, B, C;
    Word16 i;
    Word16 MaxAt;
    Word16 Score, CurrentScore;
    Word16 PeriodicityIndex;
    Word32 AbsTotal; /* 16Q15 */
    Word16 Multiplier;
    Word16 LtpPitchIndex;
    Word8  Bandwidth;
    Word32 Lag;
    Word16 s, tmp, tmp2, tmp3;
    Word32 tmp32;

    /* Debug init (not instrumented) */
    C = -3000;

    PeriodicityIndex = 0;
    move16();
    Score = -1;
    move16();

    s = sub(Find_Max_Norm32(Mdct, NumToConsider), 2);

    A = L_shl(L_abs(Mdct[0]), s);
    B = L_shl(L_abs(Mdct[1]), s);

    tmp = sub(NumToConsider, 3);
    FOR (i = 1; i < tmp; i += 3)
    {
        C = L_shl(L_abs(Mdct[i + 1]), s);
        AbsMdct3[i] = round_fx(L_add(L_add(A, B), C));

        A = L_shl(L_abs(Mdct[i + 2]), s);
        AbsMdct3[i + 1] = round_fx(L_add(L_add(A, B), C));

        B = L_shl(L_abs(Mdct[i + 3]), s);
        AbsMdct3[i + 2] = round_fx(L_add(L_add(A, B), C));
    }

    IF (sub(i, sub(NumToConsider, 1)) < 0)
    {
        C = L_shl(L_abs(Mdct[i + 1]), s);
        AbsMdct3[i] = round_fx(L_add(L_add(A, B), C));
    }

    IF (sub(i, sub(NumToConsider, 2)) < 0)
    {
        A = L_shl(L_abs(Mdct[i + 2]), s);
        assert(C != -3000);
        AbsMdct3[i + 1] = round_fx(L_add(L_add(A, B), C));
    }

    AbsTotal = L_deposit_l(0);
    IF (UnfilteredMdct != NULL)
    {
        FOR (i = 0; i < NumToConsider; ++i)
        {
            AbsTotal = L_add(AbsTotal, L_shr(L_abs(UnfilteredMdct[i]), 16));
        }
        /* balance difference between filtered and unfiltered mdct samples */
        AbsTotal = L_shr(AbsTotal, sub(4, s));
    }
    ELSE
    {
        tmp = sub(NumToConsider, 1);
        FOR (i = 1; i < tmp; i += 3)
        {
            AbsTotal = L_mac0(AbsTotal, AbsMdct3[i], 1);
        }
    }

    test();
    IF ((LtpPitchLag > 0) && (sub(LtpGain, kLtpHmGainThr) > 0))
    {
        Bandwidth = 0;
        move16();
        if (sub(NumToConsider, 256) >= 0)
        {
            Bandwidth = 1;
            move16();
        }

        LtpPitchIndex = sub(mult_r(LtpPitchLag, 1 << (15-kLtpHmFractionalResolution)), 2);
        assert(0 <= LtpPitchIndex && LtpPitchIndex <= 16);

        tmp32 = L_shl(L_deposit_l(sub(NumToConsider, 2)), kLtpHmFractionalResolution);
        tmp = shl(1, NumRatioBits[Bandwidth][LtpPitchIndex]);
        FOR (Multiplier = 1; Multiplier <= tmp; ++Multiplier)
        {
            Lag = L_shr(L_mult0(LtpPitchLag, Ratios[Bandwidth][LtpPitchIndex][Multiplier-1]), 8);

            test();
            IF ((L_sub(Lag, 4<<kLtpHmFractionalResolution) >= 0) && (L_sub(Lag, tmp32) <= 0))
            {
                CurrentScore = SearchPeriodicityIndex_Single(
                                   AbsMdct3,
                                   NumToConsider,
                                   Lag,
                                   kLtpHmFractionalResolution
                               );

                if (sub(CurrentScore, Score) > 0)
                {
                    PeriodicityIndex = s_or(Multiplier, kLtpHmFlag);
                }
                Score = s_max(Score, CurrentScore);
            }
        }
        PeriodicityIndex = s_or(PeriodicityIndex, shl(LtpPitchIndex, 9));
    }
    ELSE
    {
        IF (UnfilteredMdct != NULL)
        {
            MaxAt = 1;
            move16();
            A = L_shr(AbsMdct3[1], 6);

            FOR (i = 4; i < NumToConsider - 1; i += 3)
            {
                if (L_sub(AbsMdct3[i], AbsMdct3[MaxAt]) > 0)
                {
                    MaxAt = i;
                    move16();
                }
                A = L_add(A, L_shr(AbsMdct3[i], 6));
            }

            if (L_sub(L_shr(AbsMdct3[MaxAt], 6), Mpy_32_16_1(A, FL2WORD16(0.7))) > 0)
            {
                NumToConsider = s_min( NumToConsider, add(MaxAt, 4) );
            }
        }

        SearchPeriodicityIndex_Range(
            AbsMdct3,
            NumToConsider,
            0, 16,
            3,
            GET_ADJ2(0, 6, 3),
            4,
            &PeriodicityIndex,
            &Score
        );

        SearchPeriodicityIndex_Range(
            AbsMdct3,
            NumToConsider,
            16, 80,
            4,
            GET_ADJ2(16, 8, 4),
            4,
            &PeriodicityIndex,
            &Score
        );

        SearchPeriodicityIndex_Range(
            AbsMdct3,
            NumToConsider,
            80, 208,
            3,
            GET_ADJ2(80, 12, 3),
            4,
            &PeriodicityIndex,
            &Score
        );

        IF (sub(NumToConsider, 128) <= 0)    /* no long lags for band-limited MDCTs */
        {
            SearchPeriodicityIndex_Range(
                AbsMdct3,
                NumToConsider,
                208, add(88, NumToConsider),
                0,
                GET_ADJ2(224, 188, 0),
                1,
                &PeriodicityIndex,
                &Score
            );
        }
        ELSE {
            test();
            IF (sub(TargetBits, kSmallerLagsTargetBitsThreshold) > 0 && sub(NumToConsider, 256) >= 0)
            {
                SearchPeriodicityIndex_Range(
                    AbsMdct3,
                    NumToConsider,
                    208, 224,
                    1,
                    GET_ADJ2(208, 28, 1),
                    1,
                    &PeriodicityIndex,
                    &Score
                );
                SearchPeriodicityIndex_Range(
                    AbsMdct3,
                    NumToConsider,
                    224, 256,
                    0,
                    GET_ADJ2(224, 188, 0),
                    1,
                    &PeriodicityIndex,
                    &Score
                );
            }
            ELSE {
                SearchPeriodicityIndex_Range(
                    AbsMdct3,
                    NumToConsider,
                    208, 256,
                    1,
                    GET_ADJ2(208, 28, 1),
                    1,
                    &PeriodicityIndex,
                    &Score
                );
            }
        }
    }

    IF (AbsTotal > 0)
    {
        tmp32 = L_mult0(Score, NumToConsider); /* -> 16Q15 */
        tmp = sub(norm_l(tmp32), 1);
        tmp2 = norm_l(AbsTotal);
        tmp3 = div_s( round_fx(L_shl(tmp32, tmp)), round_fx(L_shl(AbsTotal, tmp2)) );
        BASOP_SATURATE_WARNING_OFF
        *RelativeScore = shr(tmp3, add(sub(tmp, tmp2), 2)); /* -> 2Q13 */           move16();
        BASOP_SATURATE_WARNING_ON
    }
    ELSE
    {
        *RelativeScore = 0;
        move16();
    }

    return PeriodicityIndex;
}

static void PeakFilter(
    const Word32 x[],                    /* (I) absolute spectrum                              */
    Word32 y[],                          /* (O) filtered absolute spectrum, must not alias x[] */
    Word16 L_frame                       /* (I) number of spectral lines                       */
)
{
    Word16 flen, i;
#define kPeakElevationThreshold 1.0f
    Word16 m;
    Word32 a;

    flen = shr(L_frame, 4);
    /* m = kPeakElevationThreshold / (float)(2*flen + 1); */
    m = shr(div_s(FL2WORD16_SCALE(kPeakElevationThreshold, 12), add(shl(flen,1), 1)), 3);

    a = L_deposit_l(0);
    FOR (i=0; i<flen; ++i)
    {
        a = L_add(a, L_shr(x[i], 4));
    }

    FOR (i=0; i<flen; ++i)
    {
        y[i] = L_max(0, L_sub(L_shr(x[i],4), Mpy_32_16_1(a, m)));
        move32();
        a = L_add(a, L_shr(x[i+flen], 4));
    }
    sub(0,0);
    FOR (; i<L_frame-flen; ++i)
    {
        y[i] = L_max(0, L_sub(L_shr(x[i],4), Mpy_32_16_1(a, m)));
        move32();
        a = L_sub(a, L_shr(x[i-flen], 4));
        a = L_add(a, L_shr(x[i+flen], 4));
    }

    FOR (; i<L_frame; ++i)
    {
        y[i] = L_max(0, L_sub(L_shr(x[i],4), Mpy_32_16_1(a, m)));
        move32();
        a = L_sub(a, L_shr(x[i-flen], 4));
    }

}


/* Returns: RE error */
static Word32 tcx_hm_get_re(
    const Word16 x[],  /* i: absolute spectrum                       */
    Word16 gain,       /* o: quantized harmonic model gain     Q11   */
    Word32 lag,        /* i: pitch lag                         Q0    */
    Word16 fract_res,  /* i: fractional resolution of the lag  Q0    */
    Word16 p[],        /* i: harmonic model                    Q13   */
    Word32 env[],      /* i: envelope                          Q16   */
    Word16 L_frame     /* i: number of spectral lines          Q0    */
)
{
    Word16 i, inv_G, tmp;
    Word16 s, s_ne, s_x_ne;
    Word16 *x_ne;
    Word32 ne[N_MAX_ARI], G, e;

    x_ne = (Word16*)ne;
    assert(L_frame <= N_MAX_ARI);

    /* Calculate new envelope with "gain" harmonic gain ***********************/

    Copy32(env, ne, L_frame);

    tcx_hm_modify_envelope(
        gain,
        lag,
        fract_res,
        p,
        ne,
        L_frame
    );

    /* Normalize **************************************************************/

    s_ne = getScaleFactor32(ne, L_frame);

    G = L_deposit_l(0);
    FOR (i=0; i<L_frame; ++i)
    {
        x_ne[i] = mult(x[i], extract_h(L_shl(ne[i], s_ne))); /* exp: x_e+15-s_ne */ move16();
        G = L_mac0(G, x_ne[i], 1); /* exp: x_e + 31 - s_ne */
    }
    s = norm_l(G);
    inv_G = div_s(0x4000, extract_h(L_shl(G, s))); /* exp: 1 - (x_e + 31 - s_ne - s) */

    /* Calculate error ********************************************************/
    s_x_ne = sub(getScaleFactor16(x_ne, L_frame), 2);
    e = L_deposit_l(0);
    FOR (i=0; i<L_frame; ++i)
    {
        tmp = shl(x_ne[i], s_x_ne); /* exp: x_e + 15 - s_ne - s_x_ne */
        tmp = mult(tmp, tmp); /* exp: 2 * (x_e + 15 - s_ne - s_x_ne) */
        e = L_mac(e, tmp, tmp); /* exp: 4 * (x_e + 15 - s_ne - s_x_ne) */
    }

    e = Mpy_32_16_1(e, inv_G);
    e = Mpy_32_16_1(e, inv_G);
    e = Mpy_32_16_1(e, inv_G);
    e = Mpy_32_16_1(e, inv_G); /* exp: 4 * (s - s_x_ne - 15) */

    e = L_shl(e, shl(sub(sub(s, s_x_ne), 15), 2)); /* Q31 */


    return e;
}

static void tcx_hm_quantize_gain(
    const Word32 x[],      /* i: absolute spectrum                 Q31-e */
    Word16 *x_e,           /* i: absolute spectrum exponent        Q0    */
    Word32 env[],          /* i: envelope                          Q16   */
    Word32 lag,            /* i: pitch lag                         Q0    */
    Word16 fract_res,      /* i: fractional resolution of the lag  Q0    */
    Word16 p[],            /* i: harmonic model                    Q13   */
    Word16 L_frame,        /* i: number of spectral lines          Q0    */
    Word16 coder_type,     /* i: coder_type                        Q0    */
    Word16 relative_score, /* i: periodicity score                 Q13   */
    Word16 *gain_idx,      /* o: quantization index                Q0    */
    Word16 *gain           /* o: quantized harmonic model gain     Q11   */
)
{
    Word16 g;
    Word16 pe; /* Q14 */
    Word32 be; /* Q31 */
    Word32 e;  /* Q31 */
    Word16 i;
    Word16 x16[N_MAX_ARI], s_x;
    Word16 s;


    assert(coder_type==VOICED || coder_type == GENERIC);
    s=0;
    move16();
    if (sub(coder_type,VOICED) == 0)
    {
        s=1;
        move16();
    }



    *gain = 0;
    move16();

    /* Disable the harmonic model if periodicity is very low */
    IF ( sub(relative_score,kLowPeriodicityThr[s]) < 0 )
    {
        return;
    }

    (void)x_e;

    s_x = getScaleFactor32(x, L_frame);
    FOR (i=0; i<L_frame; ++i)
    {
        x16[i] = extract_h(L_shl(x[i], s_x));
    }

    be = tcx_hm_get_re(x16, *gain, lag, fract_res, p, env, L_frame);

    IF ( sub(coder_type,GENERIC) == 0 )
    {
        e = tcx_hm_get_re(
                x16,
                qGains[s][0],
                lag,
                fract_res,
                p,
                env,
                L_frame
            );

        pe = FL2WORD16_SCALE(1.05,1);
        move16();

        /* pe is Q14 */
        IF ( L_sub(L_shl(Mpy_32_16_1(e,pe),1),be) < 0 )
        {
            *gain_idx = 0;
            move16();
            *gain     = qGains[s][0];
            move16();
        }
    }
    ELSE
    {
        /* Iterate over all possible gain values */
        FOR (g=0; g<(1 << kTcxHmNumGainBits); ++g)
        {
            e = tcx_hm_get_re(
                x16,
                qGains[s][g],
                lag,
                fract_res,
                p,
                env,
                L_frame
            );

            /* Add bit penalty */
            pe = FL2WORD16_SCALE(1.0,1);
            move16();
            if ( *gain == 0 )
            {
                pe = FL2WORD16_SCALE(1.05,1);
                move16();
            }

            /* Minimum selection, pe is Q14 */
            IF ( L_sub(L_shl(Mpy_32_16_1(e,pe),1),be) < 0 )
            {
                be        = L_add(e, 0);
                *gain_idx = g;
                move16();
                *gain     = qGains[s][g];
                move16();
            }
        }
    }

}

static Word16 tcx_hm_search(
    const Word32 abs_spectrum[], /* i:   absolute spectrum            Q31-e */
    Word16 L_frame,              /* i:   number of spectral lines     Q0    */
    Word16 targetBits,           /* i:   target bit budget            Q0    */
    Word16 LtpPitchLag,          /* i:   LTP pitch lag or -1 if none  Q0    */
    Word16 LtpGain,              /* i:   LTP gain                     Q15   */
    Word16 *RelativeScore        /* o:   Energy concentration factor  Q13   */
)
{
    Word32 fspec[N_MAX_ARI];

    /* Filter out noise and keep the peaks */
    PeakFilter(abs_spectrum, fspec, L_frame);

    /* Get the best lag index */
    return SearchPeriodicityIndex(
               fspec,
               abs_spectrum,
               L_frame,
               targetBits,
               LtpPitchLag,
               LtpGain,
               RelativeScore
           );
}

void tcx_hm_analyse(
    const Word32 abs_spectrum[], /* i:   absolute spectrum            Q31-e */
    Word16 *spectrum_e,          /* i:   absolute spectrum exponent   Q0    */
    Word16 L_frame,              /* i:   number of spectral lines     Q0    */
    Word32 env[],                /* i/o: envelope shape               Q16   */
    Word16 targetBits,           /* i:   target bit budget            Q0    */
    Word16 coder_type,           /* i:  coder type                    Q0    */
    Word16 prm_hm[],             /* o:   HM parameters                Q0    */
    Word16 LtpPitchLag,          /* i:   LTP pitch lag or -1 if none  Q0    */
    Word16 LtpGain,              /* i:   LTP gain                     Q15   */
    Word16 *hm_bits_out          /* o:   bit consumption              Q0    */
)
{
    Word32 lag;
    Word16 fract_res;
    Word16 RelativeScore;                   /* Q13 */
    Word16 gain;                            /* Q11 */
    Word16 p[2*kTcxHmParabolaHalfWidth+1];  /* Q13 */
    Word16 hm_bits, bw_flag;


    /* Disable HM for non-GENERC, VOICED modes */
    if ( sub(coder_type, VOICED) < 0 )
    {
        *hm_bits_out = 0;
        move16();
        prm_hm[0] = 0;
        move16();

        return;
    }

    bw_flag = 0;
    move16();
    if (sub(L_frame, 256) >= 0)
    {
        bw_flag = 1;
        move16();
    }

    /* Bit consumption for the HM off case: 1 bit flag */
    hm_bits = 1;
    move16();
    move16();
    prm_hm[1] = tcx_hm_search(
                    abs_spectrum,
                    L_frame,
                    sub(targetBits, hm_bits),
                    LtpPitchLag,
                    LtpGain,
                    &RelativeScore
                );

    /* Convert the index to lag */
    UnmapIndex(
        prm_hm[1],
        bw_flag,
        LtpPitchLag,
        (( sub(sub(targetBits, hm_bits),kSmallerLagsTargetBitsThreshold) <= 0 ) || !bw_flag),
        &fract_res,
        &lag
    );
    test();

    /* Render harmonic model */
    tcx_hm_render(
        lag,
        fract_res,
        p
    );

    /* Calculate and quantize gain */
    gain = 0;
    move16();

    tcx_hm_quantize_gain(
        abs_spectrum,
        spectrum_e,
        env,
        lag,
        fract_res,
        p,
        L_frame,
        coder_type,
        RelativeScore,
        &prm_hm[2],
        &gain
    );

    /* Decision */
    IF ( gain > 0 )
    {
        prm_hm[0] = 1; /* flag: on */                                               move16();

        hm_bits = add(hm_bits, CountIndexBits(bw_flag, prm_hm[1]));

        if (sub(coder_type, VOICED) == 0)
        {
            hm_bits = add(hm_bits, kTcxHmNumGainBits);
        }

        tcx_hm_modify_envelope(
            gain,
            lag,
            fract_res,
            p,
            env,
            L_frame
        );
    }
    ELSE
    {
        prm_hm[0] = 0;  /* flag: off   */                                           move16();
        prm_hm[1] = -1; /* pitch index */                                           move16();
        prm_hm[2] = 0;  /* gain index  */                                           move16();
    }

    *hm_bits_out = hm_bits;
    move16();
}

