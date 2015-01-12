/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "rom_dec_fx.h" /* Static table prototypes                */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define FEC_MAX 512
#define FEC_NB_PULSE_MAX 20
#define FEC_FFT_MAX_SIZE 512
#define FEC_DCIM_FILT_SIZE_MAX 60

#define ENV_STAB_DEC_THR                      16384                  /* Q15 st->env_stab based threshold for hq-ecu technology decision */

#define PHASE_DITH_fx                         25736  /* 2*pi in Q12 */
#define DELTA_CORR                            6                                     /* Tuning parameter - defining range for phase correction around peak */
#define THRESH_TR_DB_FX                       10
#define THRESH_TR_LIN_BY2_FX                  (10/2) /* 10.0^(THRESH_TR_DB_FX/10.0)/2. Divided by 2 to facilitate fixed-point implementation. */
#define MAX_INCREASE_GRPPOW_FX                0      /* max. amplification in case of transients (in dB scale) */
#define MAX_INCREASE_GRPPOW_LIN_FX            32767  /* in Q15-- 10.0^(MAX_INCREASE_GRPPOW_FX/10.0)  (in linear scale) */

#define PHASE_DITH_SCALE_SHIFT                16     /* The number of bit shift equivalent to multiply by PHASE_DITH_SCALE */

#define BURST_PHDITH_THRESH                   (4-1)                                 /* speech start phase dither with <burst_phdith_thresh> losses in a row */
#define BURST_PHDITH_RAMPUP_LEN               2                                     /* speech ramp up degree of phase dither over a length of <burst_phdith_rampup_len> frames */
#define BURST_ATT_THRESH                      (3-1)                                 /* speech start attenuate with <burst_att_thresh> losses in a row */
#define ATT_PER_FRAME                         4                                     /* speech attenuation in dB */
#define BETA_MUTE_THR                         10                      /* time threshold to start beta-noise attenuation */

#define LGW32K                                7
#define LGW16K                                6
#define LGW8K                                 5
#define LGW48K                                LGW32K+1                              /* Use the same frequency groups as for SWB + 1 */

#define LTRANALOG32K                          8
#define LTRANALOG16K                          7
#define LTRANALOG8K                           6
#define PFIND_SENS_FX                         31785  /* 0.97 in Q15 */
#define CMPLMNT_PFIND_SENS_FX                 983    /* (1.0 - pfind_sen) in Q15 */

#define FEC_HQ_ECU_POINT5  (0x4000)    /* 0.5 in Q15. Prefix with FEC_HQ namespace to avoid naming conflict. */
#define FEC_HQ_ECU_ROOT2   (0x5a83)    /* sqrt(2) in Q14 */
#define FEC_HQ_HAMM_A0     17695       /* 0.54 in Q15 */
#define FEC_HQ_HAMM_A1     15073       /* 0.46 in Q15 */
#define FEC_HQ_WIN_A0      FEC_HQ_HAMM_A0
#define FEC_HQ_WIN_A1      FEC_HQ_HAMM_A1


static  Word16 sqrt2ndOrder(const Word16);

static void windowing(const Word16*, Word16*, const Word16*, const Word16, const Word16);
static void windowing_ROM_optimized(const Word16*, Word16*, const Word16, const Word16, const Word16);
static void fft_spec2_fx(const Word16 [],  Word32 [], const Word16);
static void trans_ana_fx(const Word16*, Word16*, Word16*, Word16*, const Word16, const Word16, const Word16
                         ,const Word16
                         ,Word16*, Word16*, Word16*, Word16*

                        );
static void peakfinder_fx(const Word16*, const Word16, Word16*, Word16*, const Word16);
static Word16 imax_fx( const Word16 *, const Word16);
static void spec_ana_fx(const Word16*, Word16*, Word32*, Word16*, Word16*, const Word16, Word16*);
static void subst_spec_fx(const Word16*, const Word32*, Word16*, const Word16, Word16*, const Word16*, const Word16, const Word16*, const Word16, Word16*
                          ,const Word16*, const Word16*, Word16, const Word16*
                         );
Word16 rand_phase_fx(const Word16 seed, Word16 *sin_F, Word16 *cos_F);


/*------------------------------------------------------------------*
 * rand_phase()
 *
 * randomized phase in form of sin and cos components
 *------------------------------------------------------------------*/
Word16 rand_phase_fx(const Word16 seed, Word16 *sin_F, Word16 *cos_F)
{
    const Word16 *sincos = sincos_t_ext_fx + 128;
    Word16 seed2 = own_random2_fx(seed);
    Word16 seed2_shift = shr(seed2, 8);

    *sin_F = negate(*(sincos + seed2_shift));
    move16();
    if (s_and(seed2, 0x40) != 0)
    {
        *sin_F = *(sincos + seed2_shift);
        move16();
    }

    *cos_F = negate(*(sincos - seed2_shift));
    move16();
    if (s_and(seed2, 0x80) != 0)
    {
        *cos_F = *(sincos - seed2_shift);
        move16();
    }

    return seed2;
}

/*-----------------------------------------------------------------------------
 * fft_spec2_fx()
 *
 * Square magnitude of fft spectrum
 *----------------------------------------------------------------------------*/
static void fft_spec2_fx(
    const Word16 x[],    /* i : Input vector: complex spectrum */
    Word32 xMagSq[],     /* o : Magnitude square spectrum */
    const Word16 N       /* i : Input vector length */
)
{
    Word16 i, l;
    const Word16 *pRe, *pIm;
    Word32 *pMagSq, acc;

    /* Magnitude at 0. */
    pMagSq = &xMagSq[0];
    pRe = &x[0];
    *pMagSq++ = L_mult0(*pRe, *pRe);
    pRe++; /* Non-fractional multiply gives subsequent group power accumulation a bit headroom. */

    /* From 1 to (N/2 - 1). */
    l = sub(shr(N, 1), 1);  /* N/2 - 1. */
    pIm = &x[N];
    pIm--;
    FOR (i = 0; i < l; i++)
    {
        acc = L_mult0(*pRe, *pRe);
        pRe++;  /* Non-fractional mode multiply. */
        *pMagSq++ = L_mac0(acc, *pIm, *pIm);
        pIm--;
        move32();
    }

    /* The magnitude square at N/2 */
    *pMagSq = L_mult0(*pRe, *pRe);
    move32();
    return;
}

/*-----------------------------------------------------------------------------
 * trans_ana_fx()
 *
 * Transient analysis
 *----------------------------------------------------------------------------*/
static void trans_ana_fx(
    const Word16 *xfp,               /* i  : Input signal                                       Q0  */
    Word16 *mag_chg,           /* o  : Magnitude modification                             Q15 */
    Word16 *ph_dith,           /* o  : Phase dither, 2*PI is not included (Q15, i.e., between 0.0 and 1.0) */
    Word16 *mag_chg_1st,       /* i/o: per band magnitude modifier for transients         Q15 */
    const Word16 output_frame,       /* i  : Frame length                                           */
    const Word16 time_offs,          /* i  : Time offset (integral multiple of output_frame)        */
    const Word16 est_mus_content,    /* i  : 0.0=speech_like ... 1.0=Music    (==st->env_stab )     */
    const Word16 last_fec,           /* i  : signal that previous frame was concealed with fec_alg  */
    Word16 *alpha,             /* o  : Magnitude modification factors for fade to average     */
    Word16 *beta,              /* o  : Magnitude modification factors for fade to average     */
    Word16 *beta_mute,         /* o  : Factor for long-term mute                              */
    Word16 *Xavg               /* o  : Frequency group average gain to fade to                */
)
{
    const Word16 *w_hamm, *pFftTbl;
    Word16 att_val, attDegreeFrames;
    Word16 xfp_left[Ltrana48k], xfp_right[Ltrana48k];
    Word32 magSqLeft[Ltrana48k/2+1], magSqRight[Ltrana48k/2+1];
    Word32 *pLeft, *pRight, *pGrPowLeft, *pGrPowRight;
    Word32 gr_pow_left[Lgw_max], gr_pow_right[Lgw_max];
    const Word16 *pXfp, *pGw;
    Word16 Ltrana, Ltrana_2, Lprot, three4thLength, LtranaLogMinus1, Lgw, i, k, l, burst_len;
    Word16 man, expo;
    Word16 att_always = 0;          /* fixed attenuation per frequency group if set to  1 */
    Word16 oneOverFrame, roundEstMusContent, tmp16, headroom, lowerEdge;
    Word16 burst_phdith_thresh = BURST_PHDITH_THRESH;     /*speech settings */
    Word16 burst_att_thresh = BURST_ATT_THRESH ;
    Word16 att_per_frame = ATT_PER_FRAME;
    Word16 burst_phdith_rampup_len = BURST_PHDITH_RAMPUP_LEN;
    Word16 tr_dec[Lgw_max];
    UWord16 lsb;
    Word32 acc;

    Lgw = 0;
    LtranaLogMinus1 = 0;
    pFftTbl = NULL;

    /* Initialisation to prevent warnings */
    oneOverFrame = 102;  /* 1/320 in Q15 */                                 move16();
    w_hamm = w_hamm16k_2_fx;
    move16();
    Lprot = 512;
    move16();

    /* check burst error */
    IF (sub(output_frame, L_FRAME48k) == 0) /* output_frame = (sampling frequency)/50 */
    {
        oneOverFrame = 34;  /* 1/960 in Q15 */                                  move16();
        w_hamm = w_hamm48k_2_fx;
        move16();
        Lgw = LGW48K;
        move16();
        Lprot = 1536; /* (2*output_frame)*1024/1280; */                         move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        oneOverFrame = 51;  /* 1/640 in Q15 */                                  move16();
        w_hamm = w_hamm32k_2_fx;
        move16();
        Lgw = LGW32K;
        move16();
        Lprot = 1024;
        move16();
        pFftTbl = FFT_W128;  /* Table for 256 real input radix-2 FFT */         move16();
        LtranaLogMinus1 = LTRANALOG32K - 1;
        move16();
    }
    ELSE IF (sub(output_frame, L_FRAME16k) == 0)
    {
        oneOverFrame = 102;  /* 1/320 in Q15 */                                 move16();
        w_hamm = w_hamm16k_2_fx;
        move16();
        Lgw = LGW16K;
        move16();
        Lprot = 512;
        move16();
        pFftTbl = FFT_W64;  /* Table for 128 real input radix-2 FFT */          move16();
        LtranaLogMinus1 = LTRANALOG16K - 1;
        move16();
    }
    ELSE IF (sub(output_frame, L_FRAME8k) == 0)
    {
        oneOverFrame = 205;  /* 1/160 in Q15 */                                 move16();
        w_hamm = w_hamm8k_2_fx;
        move16();
        Lgw = LGW8K;
        move16();
        Lprot = 256;
        move16();
        pFftTbl = phs_tbl_dec;  /* Table for 64 real input radix-2 FFT */       move16();
        LtranaLogMinus1 = LTRANALOG8K - 1;
        move16();
    }
    burst_len = add(mult_r(time_offs, oneOverFrame), 1);

    *ph_dith = 0;
    move16(); /*  typical pattern is 2*pi*[0  0 .25 .50 .75 1.0 1.0 1.0 ..] */

    IF (sub(output_frame, L_FRAME32k) >= 0) /*currently est_mus_content only calculated for SWB and FB */
    {
        roundEstMusContent = 0;
        move16();
        if (sub(est_mus_content, FEC_HQ_ECU_POINT5) >= 0) /* est_mus_content is in [0.0, 1.0]. */
        {
            roundEstMusContent = 1;
            move16();
        }

        /* softly shift attenuation just a bit later  for estimated "stable" music_content */
        burst_phdith_thresh = add(BURST_PHDITH_THRESH, roundEstMusContent);
        burst_att_thresh = add(BURST_ATT_THRESH, roundEstMusContent);
        att_per_frame  = sub(ATT_PER_FRAME, roundEstMusContent);     /* only slighty less att for music */
    }
    ELSE
    {
        /* lock to music-like stable envelope  setting for now */
        burst_phdith_thresh = add(BURST_PHDITH_THRESH, 1);  /* in Q0 */
        burst_att_thresh = add(BURST_ATT_THRESH, 1);        /* in Q0 */
        att_per_frame  = sub(ATT_PER_FRAME, 1);             /* in Q0 */
    }

    IF (sub(burst_len, burst_phdith_thresh) > 0)
    {
        /* increase degree of dither */
#if BURST_PHDITH_RAMPUP_LEN != 2
#error The implementation of phase_dith=min(1.0, (burst_len - burst_phdith_thresh)/burst_phdith_rampup_len)) is incorrect
#endif
        *ph_dith = 32767;  /* 1.0 in Q15. N.B. 2*PI is not included. */       move16();
        tmp16 = sub(burst_len, burst_phdith_thresh);
        if (sub(tmp16, burst_phdith_rampup_len) < 0)
        {
            *ph_dith = 16384;  /* 0.5 in Q15. N.B. 2*PI is not included. */   move16();
        }
    }

    attDegreeFrames = 0;
    move16();
    IF (sub(burst_len, burst_att_thresh) > 0)
    {
        att_always = 1;
        move16();
        /* increase degree of attenuation */

        /* N.B. To facilitate the subsequent 10^(-att_degree/20) implementation
         * so as to use direct table-lookup,
         * (burstLen - burst_att_thresh) is NOT multiplied by "att_per_frame". */
        attDegreeFrames = sub(burst_len, burst_att_thresh); /* Not multiplied by att_per_frqme! */
        /* Furthermore, in order to minimize the size of the lookup-table required to
         * implement 10^(-att_degree/10), hard limit attDegreeFrames to (30% of 50)=15.
         * If attDegreeFrames is greater than 15, it means there are more than 15 successive
         * bad frames. In this case, no matter what to do, the sound quality will be bad.
         */
        if (sub(attDegreeFrames, OFF_FRAMES_LIMIT) > 0)
        {
            attDegreeFrames = OFF_FRAMES_LIMIT; /* Hard limit the no. of frames */ move16();
        }
    }

    Ltrana   = shr(Lprot, 2);
    Ltrana_2 = shr(Ltrana, 1);

    test();
    test();
    IF (sub(burst_len, 1) <= 0 || (sub(burst_len, 2) == 0 && last_fec != 0))
    {

        set16_fx(alpha, 32767, Lgw_max);
        set16_fx(beta, 0, Lgw_max);
        *beta_mute = BETA_MUTE_FAC_INI;
        move16();

        /* Apply Hamming window */
        windowing(xfp, xfp_left, w_hamm, 0, Ltrana_2);   /* 1st quarter */
        three4thLength = sub(Lprot, Ltrana);
        pXfp = xfp + three4thLength;
        windowing(pXfp, xfp_right, w_hamm, 0, Ltrana_2); /* 4th quarter */

        /* spectrum */
        IF (sub(output_frame, L_FRAME48k) == 0)
        {
            fft3_fx(xfp_left,  xfp_left,  Ltrana);
            fft3_fx(xfp_right, xfp_right, Ltrana);
        }
        ELSE
        {
            r_fft_fx_lc(pFftTbl, Ltrana, Ltrana_2, LtranaLogMinus1, xfp_left,  xfp_left,  1);
            r_fft_fx_lc(pFftTbl, Ltrana, Ltrana_2, LtranaLogMinus1, xfp_right, xfp_right, 1);
        }

        /* square representation */
        fft_spec2_fx(xfp_left,  magSqLeft,  Ltrana);
        fft_spec2_fx(xfp_right, magSqRight, Ltrana);

        /* band powers in frequency groups
         * exclude bin at PI from calculation */
        magSqLeft[Ltrana_2]  = L_deposit_l(0);
        magSqRight[Ltrana_2] = L_deposit_l(0);
    }

    pGrPowLeft  = &gr_pow_left[0];
    pGrPowRight = &gr_pow_right[0];
    pGw = gw_fx;
    FOR ( k = 0; k < Lgw; k++ )
    {
        test();
        test();
        IF (sub(burst_len, 1) <= 0 || (sub(burst_len, 2) == 0 && last_fec != 0))
        {
            lowerEdge = *pGw++;
            move16();
            l = sub(*pGw, lowerEdge);
            headroom = GR_POW_HEADROOM[k]; /* Number of bits to scale down preventing from saturation in accumulation.*/ move16();
            pLeft  = magSqLeft + lowerEdge;
            pRight = magSqRight + lowerEdge;
            *pGrPowLeft = L_deposit_l(0);
            *pGrPowRight = L_deposit_l(0);
            FOR (i = 0; i < l; i++)
            {
                acc = L_shr(*pLeft++,  headroom);  /* Scale down to prevent from saturation. */
                *pGrPowLeft  = L_add(*pGrPowLeft,  acc);
                move32();
                acc = L_shr(*pRight++, headroom);
                *pGrPowRight = L_add(*pGrPowRight, acc);
                move32();
            }

            /*Xavg[k] = sqrt(0.5f*(gr_pow_left[k]+gr_pow_right[k])/(float)(gw[k+1]-gw[k]));*/
            acc = L_shr(L_add(*pGrPowLeft, *pGrPowRight), 1);
            acc = Mult_32_16(acc, gw_len_inv_fx[k]);   /* -headroom */

            acc = Sqrt_l(acc, &expo);   /* -headroom+31+expo */

            expo = sub(add(expo, 31), headroom);
            if ( sub(s_and(expo, 1), 1) == 0)
            {
                acc = Mult_32_16(acc, 23170);   /* 1/sqrt(2) in Q15 */
            }
            expo = shr(expo, 1);
            Xavg[k] = round_fx(L_shl(acc, sub(sub(16, expo), 2))); /* Q0, additional right shift by 2 to account for that Xavg is
                                                                      calculated using lenght N/4 fft but is applied on a fft of length N */

            /*dither phase in case of transient */
            /* separate transition detection and application of forced burst dithering */
            tr_dec[k] = 0;
            move16();
            Mpy_32_16_ss(*pGrPowLeft, THRESH_TR_LIN_BY2_FX, &acc, &lsb); /* To facilitate fixed-point implementation, divide threshold by 2. */
            acc = L_or(L_shl(acc,16), L_and(0xffffL,lsb)); /* Equivalent to concatenate acc and lsb, and then down shift by 16 bits. */
            if (L_sub(*pGrPowRight, acc) > 0) /* gr_pow_right > thres_tr_lin*gr_pow_left */
            {
                tr_dec[k] = 1;
                move16();
            }
            Mpy_32_16_ss(*pGrPowRight, THRESH_TR_LIN_BY2_FX, &acc, &lsb);
            acc = L_or(L_shl(acc,16), L_and(0xffffL,lsb)); /* Equivalent to concatenate acc and lsb, and then down shift by 16 bits. */
            if (L_sub(*pGrPowLeft, acc) > 0)  /* gr_pow_left > thres_tr_lin*gr_pow_right */
            {
                tr_dec[k] = 1;
                move16();
            }

            /* magnitude modification */
            IF ( add(tr_dec[k], att_always) != 0)
            {

#if MAX_INCREASE_GRPPOW_FX != 0
#error trans_ana_fx-- The following implementation is incorrect
#endif
                att_val = 32767;
                move16();
                IF (L_sub(*pGrPowRight, 0) > 0)
                {
                    IF (L_sub(*pGrPowRight, *pGrPowLeft) < 0)  /* i.e., (gr_pow_right/gr_pow_left) < 1.0 */
                    {
                        /* Compute sqrt(grp_pow_chg), where grp_pow_chg = gr_pow_right/gr_pow_left. */
                        tmp16 = ratio(*pGrPowRight, *pGrPowLeft, &expo); /* tmp16 in Q14 */
                        expo = sub(expo, (15-14)); /* Now, tmp16 is considered in Q15 */
                        i = norm_s(tmp16);
                        man = shl(tmp16, i);   /* Mandatory normalization before sqrtNthOrder(). */
                        expo = add(expo, i);
                        man = sqrt2ndOrder(man);
                        if (s_and(expo,1) != 0) /* Check even or odd. */
                        {
                            man = mult_r(man,FEC_HQ_ECU_ROOT2);
                        }
                        expo = shr(expo, 1);    /* Divided by 2-- square root operation. */
                        att_val = shr(man, expo); /* Denormalize the mantissa back to Q15. */
                    }
                    /* ELSE do nothing because (gr_pow_right/gr_pow_left) >= 1.0 (i.e.,
                     * max_increase_grppow_lin) */
                }

                mag_chg_1st[k] = att_val;
                move16();
                mag_chg[k] = att_val;
                move16();
            }
            ELSE
            {
                mag_chg_1st[k] = 32767;
                move16();
                mag_chg[k] = 32767;
                move16();  /* Set to 1.0 in Q15 */
            }
        }
        ELSE
        {
            /* Since attDegreeFrames is discrete (integer) and hard limited to OFF_FRAMES_LIMIT,
             * it is much easier to implement 10^(-att_degree/20.0) by a simply direct
             * table-lookup. Also, att_per_frame is discrete as well and can be
             * either ATT_PER_FRAME-1 or ATT_PER_FRAME and nothing else. This
             * means only 2 tables of size=(OFF_FRAMES_LIMIT+1) each are required.
             * To take square root into account, it is divided by 20 instead of 10. */
            IF (sub(att_per_frame, ATT_PER_FRAME) == 0) /* Select the corresponding lookup-table. */
            {
                att_val = POW_ATT_TABLE0[attDegreeFrames]; /* 10^(-attDegreeFrames*(ATT_PER_FRAME)/20) */  move16();
            }
            ELSE
            {
                att_val = POW_ATT_TABLE1[attDegreeFrames]; /* 10^(-attDegreeFrames*(ATT_PER_FRAME - 1)/20) */ move16();
            }
            mag_chg[k] = mult_r(mag_chg_1st[k], att_val); /* Q15 */

            if (sub(burst_len, BETA_MUTE_THR) > 0)
            {
                *beta_mute = shr(*beta_mute, 1);
            }
            alpha[k] = mag_chg[k];
            move16();
            /*beta[k] = sqrt(1.0f - SQR(alpha[k])) * *beta_mute;*/
            acc = L_sub(1073741824, L_mult0(alpha[k], alpha[k]));
            acc = Sqrt_l(acc, &expo);
            expo = add(30, add(31, expo));
            if (sub(s_and(expo, 1), 1) == 0)
            {
                acc = Mult_32_16(acc, 23170);   /* 1/sqrt(2) in Q15 */
            }
            expo = shr(expo, 1);
            beta[k] = mult_r(*beta_mute, round_fx(L_shl(acc, sub(31, expo))));
            move16();

            IF (sub(k, LGW32K-1) >= 0)
            {
                beta[k] = mult_r(beta[k], 3277);    /* 0.1 in Q15 */
            }
            ELSE IF (sub(k, LGW16K-1) >= 0)
            {
                beta[k] = mult_r(beta[k], 16384);    /* 0.5 in Q15 */
            }
        }
        pGrPowLeft++;
        pGrPowRight++;
    }

    return;
}

/*-----------------------------------------------------------------------------
 * peakfinder_fx()
 *
 * Peak-picking algorithm
 *----------------------------------------------------------------------------*/
static void peakfinder_fx(
    const Word16 *x0,    /* i : vector from which the maxima will be found                    */
    const Word16 len0,   /* i : length of input vector                                        */
    Word16 *plocs, /* o : the indices of the identified peaks in x0                  Q0 */
    Word16 *cInd,  /* o : number of identified peaks                                 Q0 */
    const Word16 sel     /* i : The amount above surrounding data for a peak to be identified */
)
{
    const Word16 *pX0;
    Word16 minMag, tempMag, leftMin;
    Word16 dx0[Lprot48k_2], x[Lprot48k_2+1], peakMag[MAX_PLOCS];
    Word16 *pDx0, *pDx01, *pX;
    Word16 i, len, tempLoc, foundPeak, ii, xInd, tmp16, threshold, xAt0, xAt1, xAt2;
    Word16 len0Minus1, len0Minus2, lenMinus1;
    Word16 indarr[Lprot48k_2+1], peakLoc[MAX_PLOCS];
    Word16 *pInd;

    tempLoc = 0;

    /* Find derivative */
    len0Minus1 = sub(len0, 1);
    pX0 = x0 + 1;
    Vr_subt(pX0, x0, dx0, len0Minus1);

    FOR (i=0; i<len0Minus1; i++)
    {
        if (dx0[i] == 0)
        {
            dx0[i] = -1;
            move16();
        }
    }

    /* Find where the derivative changes sign
       Include endpoints in potential peaks and valleys */
    pX = x;
    pX0 = x0;
    pInd = indarr;
    pDx01 = dx0;
    pDx0 = pDx01 + 1;
    *pX++ = *pX0++;
    move16();
    *pInd++ = 0;
    move16();
    len = 2;
    move16();
    len0Minus2 = sub(len0, 2);
    FOR (i = 0; i < len0Minus2; i++)
    {
        IF (s_xor(*pDx01++, *pDx0++) < 0) /* Detect sign change. */
        {
            *pInd++ = add(i,1);
            *pX++ = *pX0;
            move16();
            len = add(len, 1);
        }
        pX0++;
    }
    *pInd = len0Minus1;
    move16();
    *pX = *pX0;
    move16();

    /* x[] only has the peaks, valleys, and endpoints */
    minimum_fx(x, len, &minMag);

    pInd = indarr;
    IF (sub(len, 2) > 0)
    {
        /* Set initial parameters for loop */
        tempMag = minMag;
        move16();
        foundPeak = 0;
        move16();
        leftMin = minMag;
        move16();
        threshold = add(leftMin, sel);

        /* Deal with first point a little differently since tacked it on
           Calculate the sign of the derivative since we took the first point
           on it does not necessarily alternate like the rest. */

        /* The first point is larger or equal to the second */
        pX = x;
        xAt0 = *pX++;
        move16();
        xAt1 = *pX++;
        move16();
        xAt2 = *pX--; /* After decrement, pX points to x[1]. */                 move16();
        IF (sub(xAt0, xAt1) >= 0)
        {
            ii = -1;
            move16();
            IF (sub(xAt1, xAt2) >= 0) /* x[1] is not extremum -> overwrite with x[0] */
            {
                *pX = xAt0;        /* x[1] = x[0] */                            move16();
                tmp16 = *pInd++;
                move16();
                *pInd++ = tmp16;   /* ind[1] = ind[0] */                        move16();
                len = sub(len, 1);
            }
            pX--; /* After decrement, pX points to x[0]. */
        }
        ELSE /* First point is smaller than the second */
        {
            ii = 0;
            IF (sub(xAt1, xAt2) < 0) /* x[1] is not extremum -> overwrite with x[0] */
            {
                *pX = xAt0;        /* x[1] = x[0] */                            move16();
                tmp16 = *pInd++;
                move16();
                *pInd++ = tmp16;   /* ind[1] = ind[0] */                        move16();
                len = sub(len, 1);
            }
        }
        pX--; /* After decrement, pX points to either x[-1] or x[0]. */

        *cInd = 0;
        move16();
        /*Loop through extrema which should be peaks and then valleys*/
        lenMinus1 = sub(len, 1);
        FOR (;;)
        {
            ii = add(ii, 1);   /* This is a peak */

            /* Make sure we don't iterate past the length of our vector */
            IF (sub(ii, lenMinus1) >= 0)
            {
                BREAK;
            }

            /*Reset peak finding if we had a peak and the next peak is bigger
              than the last or the left min was small enough to reset.*/
            IF (sub(foundPeak,0) > 0)
            {
                tempMag = minMag;
                move16();
                foundPeak = 0;
                move16();
            }

            /* Found new peak that was larger than temp mag and selectivity larger
               than the minimum to its left. */
            IF (sub(*(++pX), tempMag) > 0)
            {
                IF ( sub(*pX, threshold) > 0)  /* threshold = leftMin + sel */
                {
                    tempLoc = ii;
                    move16();
                    tempMag = *pX;
                    move16();
                }
            }

            ii = add(ii, 1); /* Move onto the valley */
            pX++;

            /* Come down at least sel from peak */
            IF (foundPeak == 0)
            {
                IF (sub(tempMag, add(sel, *pX)) > 0)
                {
                    foundPeak = 1;               /* We have found a peak */     move16();
                    leftMin = *pX;
                    move16();
                    threshold = add(leftMin, sel);
                    peakLoc[*cInd] = tempLoc;    /* Add peak to index */        move16();
                    peakMag[*cInd] = tempMag;
                    move16();
                    *cInd = add(*cInd, 1);
                }
            }
            IF (foundPeak == 0) /* The above IF-block has not found the peak yet. */
            {
                IF (sub(*pX, leftMin) < 0)/* New left minimum */
                {
                    leftMin = *pX;
                    move16();
                    threshold = add(leftMin, sel);
                }
            }
        }

        /* Check end point */
        IF (sub(x[lenMinus1], tempMag) > 0)
        {
            IF (sub(x[lenMinus1], threshold) > 0) /* threshold = leftMin + sel */
            {
                peakLoc[*cInd] = lenMinus1;
                move16();
                peakMag[*cInd] = x[lenMinus1];
                move16();
                *cInd = add(*cInd, 1);
                foundPeak = 1;
                move16();
            }
        }
        IF (foundPeak == 0)  /* Check if we still need to add the last point */
        {
            IF (sub(tempMag, minMag) > 0)
            {
                peakLoc[*cInd] = tempLoc;
                move16();
                peakMag[*cInd] = tempMag;
                move16();
                *cInd = add(*cInd, 1);
            }
        }

        /* Create output */
        FOR  (i = 0; i < *cInd; i++)
        {
            plocs[i] = *(indarr + peakLoc[i]);
            move16();
            move16();
        }
    }
    ELSE /* This is a monotone function where an endpoint is the only peak */
    {
        xInd = 1;
        move16();
        if (sub(x[0], x[1]) > 0)
        {
            xInd = 0;
            move16();
        }

        peakMag[0] = x[xInd];
        move16();
        IF (sub(peakMag[0], add(minMag, sel)) > 0)
        {
            plocs[0] = *(indarr + xInd);
            move16();
            *cInd = 1;
            move16();
        }
        ELSE
        {
            *cInd = 0;
            move16();
        }
    }
}

/*-----------------------------------------------------------------------------
* imax_fx()
*
* Get interpolated maximum position
*-----------------------------------------------------------------------------*/
static Word16 imax_fx( /* o: The location, relative to the middle of the 3 given data point, of the maximum. (Q15) */
    const Word16 *y,   /* i: The 3 given data points. */
    const Word16 special /* i: -1 = left edge special case, 0 = normal, +1 = right edge special case */
)
{
    Word16 posi;
    Word16 y1, y2, y3, man, expo, edge;
    const Word16 *pY;
    Word32 numer, denom, sign, acc, y3_y1;

    /* Seek the extremum of the parabola P(x) defined by 3 consecutive points
       so that P([-1 0 1]) = [y1 y2 y3] */
    pY = y;
    y1 = *pY++, y2 = *pY++, y3 = *pY;
    move16();
    move16();
    move16();

    /* The extremum value:
     *   y2i = -0.125f * SQR(y3_y1) / (y1+y3-2*y2)+y2
     * is not computed. Alternatively, the derivative of the parabola evaluated at y=0,
     * dP/dy|y=0, is used to determine whether the extremum is maximum or not.
     */

    /* Compute the extremum location: posi = (y3 - y1)/(4*y2 - 2*y1 - 2*y3). */
    y3_y1 = L_sub(y3, y1);
    acc   = L_shl(y2,1);     /* N.B. y2 is multiplied by 2 not 4. */
    acc   = L_sub(acc, y1);  /* N.B. Y1 is not multiplied by 2. */
    denom = L_sub(acc, y3);  /* N.B. Y3 is not multiplied by 2. */
    sign = L_xor(y3_y1, denom); /* Preserve the sign since div_s() only takes positive arguments. */
    numer = L_abs(y3_y1);
    denom = L_abs(denom);
    IF (numer == 0)
    {
        return 0;
    }
    IF (denom == 0)
    {
        return 0;
    }
    /* Although the output of ratio() is in Q14, adding the missing factor of 2 (See above)
     * in the denominator, the output is now considered to be in Q15. */
    man = ratio(numer, denom, &expo); /* The mantissa is considered in Q15 */
    posi = shr(man, expo);  /* in Q15 (Due to saturation, it is automatically bound inside [-1.0,1.0].) */

    if (sign < 0) /* Restore the sign. */
    {
        posi = negate(posi);
    }

    /* For both edges (left and right), the extremum found above may be minimum.
     * It needs to reject the minimum. */
    IF (sub(special,0) != 0) /* Either edge specical case. */
    {
        edge = 0x7fff; /* 1 in Q15 for the right edge special case */           move16();
        if (sub(special,0) < 0)
        {
            edge = 0;  /* Left edge special case */                            move16();
        }

        /* The derivative (slope) of the interpolating parabola = 2*A*y + B,
         *   where A = (y3 + y1)/2 - y2
         *     and B = (y3 - y1)/2.
         * Therefore, the slope at y=0 is simply B. Use this slope to determine
         * if the parabola is concave upward or downward.
         */
        IF (sub(posi,0) > 0) /* The extremum is in between the middle and the right given data points. */
        {
            IF (sub(y3, y1) <= 0) /* Check the slope at y=0, i.e., at the middle given data point. */
            {
                posi = edge;  /* minimum case */                                move16();
            }
            ELSE
            {
                posi = sub(0x7fff, posi); /* maximum case */
            }
        }
        ELSE /* The extremum is in between the left and the middle given data points. */
        {
            IF (sub(y3, y1) >= 0)
            {
                posi = edge;  /* minimum case */                                move16();
            }
            ELSE
            {
                posi = add(0x7fff, posi); /* maximum case */
            }
        }
    }
    return posi; /* Q15. The position either left or right relative to the index of the middle of the 3 given data points. */
}

/*-----------------------------------------------------------------------------
* spec_ana_fx()
*
* Spectral analysis
*-----------------------------------------------------------------------------*/
static void spec_ana_fx(
    const Word16 *prevsynth,        /* i : Input signal                                         */
    Word16 *plocs,            /* o : The indicies of the identified peaks             Q0  */
    Word32 *plocsi,           /* o : Interpolated positions of the identified peaks   Q16 */
    Word16 *num_plocs,        /* o : Number of identified peaks                       Q0  */
    Word16 *X_sav,            /* o : Stored fft spectrum                                  */
    const Word16 output_frame,      /* i : Frame length                                     Q0  */
    Word16 *Q                 /* o : Q value of the fft spectrum                          */
)
{
    Word16 Lprot, LprotLog2Minus1=0, hamm_len2=0, Lprot2, Lprot2_1, m, n;
    const Word16 *pFftTbl = NULL;
    Word16 xfp[Lprot48k];
    Word32 magSq[Lprot48k/2+1], *pMagSq;
    Word16 *pXfp, *pXfp1, *pXsav, *pPlocs;
    Word16 Xmax, Xmin, sel, man, expo, expoBy2;
    Word16 sinTblOffset, rectLength, fraction, special;
    Word32 *pPlocsi;
    Word32 acc;

    Lprot = 512; /* 1536=(2*output_frame)*1024/1280; */                move16();

    sinTblOffset = 0;

    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        Lprot = Lprot48k; /* 1536=(2*output_frame)*1024/1280; */                move16();
        hamm_len2 = Lprot_hamm_len2_48k;  /* half Hamming window = 288 */       move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        Lprot = Lprot32k;   /* 1024 */                                          move16();
        sinTblOffset = 4;
        move16();
        hamm_len2 = Lprot_hamm_len2_32k;  /* half Hamming window = 192 */       move16();
        pFftTbl = FFT_W512;  /* Table for 1024-point real input FFT */
        LprotLog2Minus1 = 9; /* FFT stages for complex input FFT */             move16();
    }
    ELSE IF (output_frame == L_FRAME16k)
    {
        Lprot = 512;
        move16();
        sinTblOffset = 8;
        move16();
        hamm_len2 = Lprot_hamm_len2_16k;  /* half Hamming window = 96 */        move16();
        pFftTbl = FFT_W256;  /* Table for 512-point real input FFT */
        LprotLog2Minus1 = 8; /* FFT stages for complex input FFT */             move16();
    }
    ELSE IF (output_frame == L_FRAME8k)
    {
        Lprot = 256;
        move16();
        sinTblOffset = 16;
        move16();
        hamm_len2 = Lprot_hamm_len2_8k;  /* half Hamming window = 48 */         move16();
        pFftTbl = FFT_W128;   /* Table for 256-point real input FFT */
        LprotLog2Minus1 = 7;  /* FFT stages for complex input FFT */            move16();
    }

    Lprot2 = shr(Lprot, 1);
    Lprot2_1 = add(Lprot2, 1);
    rectLength = sub(Lprot, shl(hamm_len2,1)); /* The length of the rectangular portion of the Hamming-Rectangular window. */

    *Q = s_max(0, sub(Exp16Array(Lprot, prevsynth), 1));
    move16();
    Copy_Scale_sig(prevsynth, xfp, Lprot, *Q);

    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        /* Apply hamming-rect window */
        windowing(xfp, xfp, w_hamm_sana48k_2_fx, rectLength, hamm_len2);
        /* Spectrum */
        fft3_fx(xfp, xfp, Lprot);
    }
    ELSE
    {
        /* Apply hamming-rect window */
        windowing_ROM_optimized(xfp, xfp, sinTblOffset, rectLength, hamm_len2);
        /* Spectrum */
        r_fft_fx_lc(pFftTbl, Lprot, Lprot2, LprotLog2Minus1, xfp,  xfp, 1);
    }

    pXfp = xfp;
    pXsav = X_sav;
    FOR (m = 0; m < Lprot; m++)
    {
        *pXsav++ = *pXfp++;
        move16();
    }

    /* Magnitude representation */
    fft_spec2_fx(xfp, magSq, Lprot);

    /* Compute xfp[m] = sqrt(magSq[m]) */
    pXfp = xfp;
    pMagSq = magSq;
    FOR (m = 0; m < Lprot2_1; m++)
    {
        IF (*pMagSq == 0)
        {
            *pXfp++ = extract_l(*pMagSq++); /* magSq[] is zero */
        }
        ELSE
        {
            expo = norm_l(*pMagSq);                   /* exponent */
            man = extract_h(L_shl(*pMagSq++, expo));  /* mantissa */
            man = sqrt2ndOrder(man);
            expoBy2 = shr(expo, 1);  /* Divided by 2-- square root operation. */
            IF (s_and(expo,1) == 0)  /* Check even or odd. */
            {
                man = mult_r(man,FEC_HQ_ECU_ROOT2); /* FEC_HQ_ECU_ROOT2 is sqrt(2) in Q14 */
                expoBy2 = sub(expoBy2, 1);
            }
            *pXfp++ = shr(man, expoBy2);
            move16();/* Denormalize the mantissa back to Q0. */
        }
    }

    /* Find maximum and minimum. */
    maximum_fx(xfp, Lprot2_1, &Xmax);
    minimum_fx(xfp, Lprot2_1, &Xmin);
    sel = mult_r(sub(Xmax, Xmin), CMPLMNT_PFIND_SENS_FX);
    peakfinder_fx(xfp, Lprot2_1, plocs, num_plocs, sel);

    /* Refine peaks */
    pPlocsi = plocsi;
    pPlocs  = plocs;
    n = sub(*num_plocs, 1); /* -1 so as to exclude the very last peak. */
    /* Special case-- The very 1st peak if it is at 0 index position */
    IF (sub(*pPlocs, 0) == 0) /* Only the very 1st peak is possible the peak at 0 index position. */
    {
        fraction = imax_fx(xfp, -1);   /* -1 signifies special left edge case. */
        acc = L_deposit_h(*pPlocs++);  /* N.B., (*pPlocs) must be zero here. */
        *pPlocsi++ = L_mac(acc, fraction, 1);
        move32();/* in Q16 */
        n = sub(n, 1);  /* This special case is taken care of-- one less to go */
    }
    /* All peaks except the very last peak but including the very 1st one if it has not been taken care of. */
    pXfp1 = xfp - 1;
    FOR (m = 0; m < n; m++) /* Loop through up to the last but one peak. (The last one is excluded.) */
    {
        pXfp = pXfp1 + *pPlocs;
        fraction = imax_fx(pXfp, 0);  /* in Q15 */
        acc = L_deposit_h(*pPlocs++);
        *pPlocsi++ = L_mac(acc, fraction, 1);
        move32();/* in Q16. Append the fractional part to the integral part. */
    }
    /* Special case-- The very last peak */
    pXfp = pXfp1 + *pPlocs;
    IF (sub(*pPlocs, Lprot2) == 0) /* Only the very last peak is possible the peak at Lprot2 index position. */
    {
        pXfp--;   /* Special case needs extra decrement */
        special = 1;  /* Signify special right edge case. */                    move16();
    }
    ELSE
    {
        special = 0;
        move16();
    }
    fraction = imax_fx(pXfp, special);   /* in Q15 */
    acc = L_deposit_h(*pPlocs);
    *pPlocsi = L_mac(acc, fraction, 1);
    move32();/* in Q16. Append the fractional part to the integral part. */
}

/*-------------------------------------------------------------------*
* subst_spec_fx()
*
* Substitution spectrum calculation
*-------------------------------------------------------------------*/

static void subst_spec_fx(
    const Word16 *plocs,         /* i   : The indices of the identified peaks                Q0  */
    const Word32 *plocsi,        /* i   : Interpolated positions of the identified peaks     Q16 */
    Word16 *num_plocs,     /* i/o : Number of identified peaks                         Q0  */
    const Word16 time_offs,      /* i   : Time offset                                        Q0  */
    Word16 *X,             /* i/o : FFT spectrum                                           */
    const Word16 *mag_chg,       /* i   : Magnitude modification                             Q15 */
    const Word16 ph_dith,        /* i   : Phase dither, 2*PI is not included. (Q15, i.e., between 0.0 and 1.0) */
    const Word16 *is_trans,      /* i   : Transient flags (either 0 or 1)                        */
    const Word16 output_frame,   /* i   : Frame length                                       Q0  */
    Word16 *seed,          /* i/o : Random seed                                            */
    const Word16 *alpha,         /* i   : Magnitude modification factors for fade to average Q15 */
    const Word16 *beta,          /* i   : Magnitude modification factors for fade to average Q15 */
    Word16 beta_mute,      /* i   : Factor for long-term mute                          Q15 */
    const Word16 *Xavg           /* i   : Frequency group averages to fade to                Q0  */
)
{
    Word16 Xph_short;
    Word32 corr_phase[MAX_PLOCS], Xph;
    Word32 *pCorrPhase;
    Word16 cos_F, sin_F, tmp;
    Word16 Lprot, m, i, e, im_ind, delta_corr_up, delta_corr_dn, delta_tmp;
    UWord16 lsb;
    Word16 j, re, im, *pReX, *pImX, lastPeak, lprotBy2Minus1, segmentLen;
    Word16 pkLocation_1, pkLocation, pkLocation1;
    const Word16 *pPlocs;
    const Word32 *pPlocsi;
    Word32 acc;
    Word16 Lecu;
    Word16 Lprot_inv;
    Word16 k;
    Word16 tmp2;
    Word16 alpha_local;
    Word16 beta_local;
    Word16 expo;

    Word16 mag_chg_local;   /*for peak attenuation in burst */

    Lprot = 512;
    move16();
    Lprot_inv = 8192;
    move16();
    Lecu = shl(output_frame, 1);

    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        Lprot = Lprot48k; /* 1536=(2*output_frame)*1024/1280; */                move16();
        Lprot_inv = 2731;   /* Q22 */                                           move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        Lprot = Lprot32k;   /* 1024 */                                          move16();
        Lprot_inv = 4096;   /* Q22 */                                           move16();
    }
    ELSE IF (output_frame == L_FRAME16k)
    {
        Lprot = 512;
        move16();
        Lprot_inv = 8192;   /* Q22 */                                           move16();
    }
    ELSE IF (output_frame == L_FRAME8k)
    {
        Lprot = 256;
        move16();
        Lprot_inv = 16384;   /* Q22 */                                           move16();
    }

    /* Correction phase of the identified peaks */
    IF (s_or(is_trans[0], is_trans[1]) != 0)
    {
        *num_plocs = 0;
        move16();
    }
    ELSE
    {
        tmp = NS2SA(output_frame*50,PH_ECU_ALDO_OLP2_NS-PH_ECU_LOOKAHEAD_NS);
        tmp = add(tmp, sub(Lecu, shr(sub(Lecu, Lprot), 1)));
        tmp = sub(tmp, shr(output_frame, 1));
        tmp = add(tmp, time_offs);
        tmp = round_fx(L_shl(L_mult0(tmp, Lprot_inv), 4)); /* 0+22+4-16=10 */

        pPlocsi = plocsi;
        pCorrPhase = corr_phase;
        FOR (m = 0; m < *num_plocs; m++)
        {
            Mpy_32_16_ss(*pPlocsi++, tmp, &acc, &lsb);  /* plocsi[] in Q16, tmp in Q10 and tmp does not include 2*PI. */
            acc = L_add(L_shl(acc, 5), lshr(lsb, 11));
            *pCorrPhase++ = acc; /* in Q16. 2*PI is not included. */            move32();
        }
    }

    lprotBy2Minus1 = sub(shr(Lprot, 1), 1);
    i = 1;
    move16();
    k = 0;
    move16();
    im_ind = sub(Lprot, 1);
    move16();
    pReX    = X + i;
    pImX    = X + im_ind;
    pPlocs      = plocs;
    pCorrPhase  = corr_phase;
    pkLocation  = *pPlocs;   /* N.B. No post-increment */                       move16();
    pkLocation1 = *pPlocs++;
    move16();
    lastPeak = sub(*num_plocs, 1);
    FOR (m = 0; m < *num_plocs; m++)
    {
        delta_corr_dn = DELTA_CORR;
        move16();
        delta_corr_up = DELTA_CORR;
        move16();

        pkLocation_1 = pkLocation;    /* plocs[m - 1] */                        move16();
        pkLocation   = pkLocation1;   /* plocs[m] */                            move16();
        pkLocation1  = *pPlocs++;     /* plocs[m + 1] */                        move16();
        IF (m > 0)
        {
            delta_tmp = shr(sub(sub(pkLocation, pkLocation_1), 1), 1);
            if (sub(delta_tmp, DELTA_CORR) < 0)
            {
                delta_corr_dn = delta_tmp;
                move16();
            }
        }

        IF (sub(m, lastPeak) < 0 )
        {
            delta_tmp = shr(sub(sub(pkLocation1, pkLocation), 1), 1);
            if (sub(delta_tmp, DELTA_CORR ) < 0)
            {
                delta_corr_up = delta_tmp;
                move16();
            }
        }

        /* Input Xph */
        segmentLen = sub(sub(pkLocation, delta_corr_dn), i);
        /* i = add(i, segmentLen); */
        FOR (j = 0; j < segmentLen; j++)
        {
            *seed = rand_phase_fx(*seed, &sin_F, &cos_F);

            re = *pReX;
            move16();
            im = *pImX;
            move16();
            tmp = sub(mult_r(re, cos_F), mult_r(im, sin_F));
            im  = add(mult_r(re, sin_F), mult_r(im, cos_F));
            IF (sub(alpha[k], 32766) < 0)
            {
                *seed = rand_phase_fx(*seed, &sin_F, &cos_F);
                tmp2 = mult_r(beta[k], Xavg[k]);
                *pReX++ = add( mult_r(alpha[k], tmp), mult_r(tmp2, cos_F) );
                move16();
                *pImX-- = add( mult_r(alpha[k], im),  mult_r(tmp2, sin_F) );
                move16();
            }
            ELSE
            {
                *pReX++ = mult_r(mag_chg[k], tmp);
                move16();
                *pImX-- = mult_r(mag_chg[k], im);
                move16();
            }
            i = add(i, 1);
            if (sub(i, gwlpr_fx[k+1]) >= 0)
            {
                k = add(k, 1);
            }
        }

        e = add(pkLocation, delta_corr_up);
        if (sub(e, lprotBy2Minus1) > 0)
        {
            e = lprotBy2Minus1;
            move16();
        }

        Xph = *pCorrPhase;
        Xph_short = s_and(extract_l(L_shr(Xph, 16 - 10)), 0x3ff); /* 10 bits precision after radix point */
        IF (sub(Xph_short, 512) >=0)
        {
            sin_F = negate(sincos_t_ext_fx[Xph_short - 512]);
            IF (sub(Xph_short, 768) < 0)
            {
                cos_F = negate(sincos_t_ext_fx[Xph_short - (512 - 256)]);
            }
            ELSE
            {
                cos_F = sincos_t_ext_fx[-Xph_short + (1024 + 256)];
                move16();
            }
        }
        ELSE
        {
            sin_F = sincos_t_ext_fx[Xph_short];
            move16();
            IF (sub(Xph_short, 256) < 0)
            {
                cos_F = sincos_t_ext_fx[Xph_short + 256];
                move16();
            }
            ELSE
            {
                cos_F = negate(sincos_t_ext_fx[-Xph_short + (256 + 512)]);
            }
        }

        segmentLen = add(sub(e, i), 1);
        /* i = add(i, segmentLen); */
        FOR (j = 0; j < segmentLen; j++ )
        {
            mag_chg_local = mag_chg[k];
            move16();
            IF (ph_dith != 0 )
            {
                Xph = *pCorrPhase;  /* in Q16. 2*PI is not included. */
                *seed = own_random2_fx(*seed); /* in Q0 */
                acc = L_mult(*seed, ph_dith); /* N.B. ph_dith[i] is in Q15, i.e., in between 0 and 1.0 (2*PI not included) */
                acc = L_shr(acc, PHASE_DITH_SCALE_SHIFT);
                Xph = L_add(Xph, acc);         /* in Q16. */

                IF (ph_dith > 0 ) /* up to 6 dB additional att of peaks in non_transient longer bursts, (when  peak phase is randomized ) */
                {
                    /* mag_chg_local *= 0.5 + (1.0 - ph_dith[i])/2 where 0.5~= sqrt((float)pow(10.0,-6/10.0)) and ph_dith=0..1.0--> scale=1.0 ...5 */
                    mag_chg_local = mult_r(mag_chg_local, sub(32767, shr(ph_dith, 1)));
                }
                Xph_short = s_and(extract_l(L_shr(Xph, 16 - 10)), 0x3ff);
                IF (sub(Xph_short, 512) >= 0)
                {
                    sin_F = negate(sincos_t_ext_fx[Xph_short - 512]);
                    IF (sub(Xph_short, 768) < 0)
                    {
                        cos_F = negate(sincos_t_ext_fx[Xph_short - (512 - 256)]);
                    }
                    ELSE
                    {
                        cos_F = sincos_t_ext_fx[-Xph_short + (1024 + 256)];
                        move16();
                    }
                }
                ELSE
                {
                    sin_F = sincos_t_ext_fx[Xph_short];
                    move16();
                    IF (sub(Xph_short, 256) < 0)
                    {
                        cos_F = sincos_t_ext_fx[Xph_short + 256];
                        move16();
                    }
                    ELSE
                    {
                        cos_F = negate(sincos_t_ext_fx[-Xph_short + (256 + 512)]);
                    }
                }
            }

            re = *pReX;
            move16();
            im = *pImX;
            move16();
            tmp = sub(mult_r(re, cos_F), mult_r(im, sin_F));
            im  = add(mult_r(re, sin_F), mult_r(im, cos_F));
            IF (sub(alpha[k], 32766) < 0)
            {
                alpha_local = mag_chg_local;
                move16();

                acc = L_sub(1073741824L, L_mult0(alpha_local, alpha_local));
                acc = Sqrt_l(acc, &expo);
                expo = add(30, add(31, expo));
                if (sub(s_and(expo, 1), 1) == 0)
                {
                    acc = Mult_32_16(acc, 23170);   /* 1/sqrt(2) in Q15 */
                }
                expo = shr(expo, 1);
                beta_local = mult_r(beta_mute, round_fx(L_shl(acc, sub(31, expo))));

                IF (sub(k, LGW32K-1) >= 0)
                {
                    beta_local = mult_r(beta_local, 3277);    /* 0.1 in Q15 */
                }
                ELSE if (sub(k, LGW16K-1) >= 0)
                {
                    beta_local = mult_r(beta_local, 16384);    /* 0.5 in Q15 */
                }

                *seed = rand_phase_fx(*seed, &sin_F, &cos_F);
                tmp2 = mult_r(beta_local, Xavg[k]);
                *pReX++ = add( mult_r(alpha_local, tmp), mult_r(tmp2, cos_F) );
                move16();
                *pImX-- = add( mult_r(alpha_local, im),  mult_r(tmp2, sin_F) );
                move16();
            }
            ELSE
            {
                *pReX++ = mult_r(mag_chg_local, tmp);
                move16();
                *pImX-- = mult_r(mag_chg_local, im);
                move16();
            }

            i = add(i, 1);
            if (sub(i, gwlpr_fx[k+1]) >= 0)
            {
                k = add(k, 1);
            }
        }
        pCorrPhase++;
    }

    segmentLen = sub(shr(Lprot,1), i);
    FOR (j = 0; j < segmentLen; j++)
    {
        *seed = rand_phase_fx(*seed, &sin_F, &cos_F);

        re = *pReX;
        move16();
        im = *pImX;
        move16();
        tmp = sub(mult_r(re, cos_F), mult_r(im, sin_F));
        im  = add(mult_r(re, sin_F), mult_r(im, cos_F));
        IF (sub(alpha[k], 32766) < 0)
        {
            *seed = rand_phase_fx(*seed, &sin_F, &cos_F);
            tmp2 = mult_r(beta[k], Xavg[k]);
            *pReX++ = add( mult_r(alpha[k], tmp), mult_r(tmp2, cos_F) );
            move16();
            *pImX-- = add( mult_r(alpha[k], im),  mult_r(tmp2, sin_F) );
            move16();
        }
        ELSE
        {
            *pReX++ = mult_r(mag_chg[k], tmp);
            move16();
            *pImX-- = mult_r(mag_chg[k], im);
            move16();
        }

        i = add(i, 1);
        if (sub(i, gwlpr_fx[k+1]) >= 0)
        {
            k = add(k, 1);
        }
    }
}

/*--------------------------------------------------------------------------
 *  rec_wtda()
 *
 *  Windowing and TDA of reconstructed frame
 *--------------------------------------------------------------------------*/

static void rec_wtda_fx(
    Word16 *X,                     /* i  : FFT spectrum                          */
    Word32 *ecu_rec,               /* o  : Reconstructed frame in tda domain     */
    const Word16 output_frame,           /* i  : Frame length                          */
    const Word16 Lprot,                  /* i  : Prototype frame length                */
    const Word32 fs
)
{
    Word16 l, Lprot2, timesh;
    Word16 rec_buf[3*L_FRAME48k];
    Word16 *xsubst_,*out_ptr;
    Word16 Qin;
    Word16 xf_len;
    Word16 i, idx;
    Word16 *p_ecu;
    Word16 g;
    Word16 tbl_delta;

    xsubst_ = rec_buf + output_frame;
    Lprot2 = shr(Lprot, 1);

    /* Initialize to WB constants */
    xf_len = 26;
    move16();
    tbl_delta = 10082; /* Q12 */    move16();
    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        xf_len = 78;
        move16();
        tbl_delta = 3361;  /* Q12 */    move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        xf_len = 52;
        move16();
        tbl_delta = 5041;  /* Q12 */    move16();
    }

    /* extract reconstructed frame with aldo window */
    l = sub(output_frame, Lprot2);
    set16_fx(xsubst_,0 , l);
    Copy(X, xsubst_ + l, Lprot);
    set16_fx(xsubst_ + add(output_frame, Lprot2), 0, l);

    /* Smoothen onset of ECU frame */
    p_ecu = xsubst_ + (output_frame - Lprot2);
    FOR ( i = 0; i < xf_len; i++)
    {
        idx = extract_l(L_shr(L_mult0(i, tbl_delta), 12));
        g = sincos_t_fx[idx];
        g = mult(g, g);
        *p_ecu = mult(g, (*p_ecu));
        move16();
        p_ecu++;
    }

    timesh = NS2SA_fx2(fs, 10000000L - PH_ECU_ALDO_OLP2_NS);

    set16_fx(rec_buf, 0, output_frame);
    Qin = 0;
    out_ptr = rec_buf + sub(shl(output_frame,1), timesh);
    wtda_fx(out_ptr, &Qin, ecu_rec, NULL, 0, ALDO_WINDOW, ALDO_WINDOW, /* window overlap of current frame (0: full, 2: none, or 3: half) */
            output_frame);

    return;
}

/*--------------------------------------------------------------------------
 *  rec_frame_fx()
 *
 *  Frame reconstruction
 *--------------------------------------------------------------------------*/
static void rec_frame_fx(
    Word16 *X,              /* i  : FFT spectrum */
    Word32 *ecu_rec,        /* o  : Reconstructed frame in tda domain */
    const Word16 output_frame,    /* i  : Frame length */
    const Word16 Q
)
{
    const Word16 *pFftTbl;
    Word16 Lprot, lprotLog2Minus1;
    Word32 fs;

    fs = L_mult0(output_frame, 50);

    /* Initialize to WB constants */
    Lprot = 512;
    move16();
    lprotLog2Minus1 = 9 - 1;
    move16();
    pFftTbl = FFT_W256;   /* Table for 512-point real input FFT */
    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        Lprot = Lprot48k; /* 1536 = (2*output_frame)*1024/1280 */               move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        Lprot = Lprot32k;   /* 1024 */                                          move16();
        lprotLog2Minus1 = 10 - 1;
        move16();
        pFftTbl = FFT_W512;  /* Table for 1024-point real input FFT */
    }

    /* extend spectrum and IDFT */
    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        ifft3_fx(X, X, Lprot);
    }
    ELSE
    {
        r_fft_fx_lc(pFftTbl, Lprot, shr(Lprot, 1), lprotLog2Minus1, X, X, 0); /* Inverse FFT */
    }
    Scale_sig(X, Lprot, -Q);

    rec_wtda_fx(X, ecu_rec, output_frame, Lprot, fs);

    return;
}

static
Word32 mult_32_32_q(const Word32 a, const Word32 b, const Word16 q)
{
    Word32 hi;
    UWord32 lo;
    Mpy_32_32_ss(a, b, &hi, &lo);

    return L_or(L_shl(hi, 32 - q - 1), L_lshr(lo, q + 1));
}

static
void fir_dwn_fx(
    const Word16 x[],         /* i  : input vector Q(x_Q)                       */
    const Word16 h[],         /* i  : impulse response of the FIR filter Q(h_Q)  */
    const Word16 h_Q,        /* H's Q                                          */
    Word16 y[],               /* o  : output vector (result of filtering) Q~    */
    const Word16 L,            /* i  : input vector size                         */
    const Word16 K,            /* i  : order of the FIR filter (K+1 coefs.)      */
    const Word16 decimation    /* i  : decimation                                */
)
{
    Word32    s;
    Word16 i, j;
    const Word16 *ptr_h, *ptr_x;
    Word16 *ptr_y;
    Word16 Kdiv2;
    Word16 centering;
    Word16 tmp;

    centering = sub(16, h_Q);
    Kdiv2 = shr(K,1);

    ptr_y = y;
    /* do the filtering */
    FOR (i = Kdiv2; i < K; i+=decimation)
    {
        s = L_deposit_l(0);
        ptr_h = h + 1;
        ptr_x = x + i - 1;
        move16();

        FOR (j = 1; j <= i; j++)
        {
            s = L_mac0(s, *ptr_h++, *ptr_x--);
        }

        *ptr_y++ = extract_h(L_shl(s, centering));
    }
    FOR (i = K; i < L; i+=decimation)
    {
        s = L_deposit_l(0);
        ptr_h = h + 1;
        ptr_x = x + i - 1;
        move16();

        FOR (j = 1; j <= K; j++)
        {
            s = L_mac0(s, *ptr_h++, *ptr_x--);
        }

        *ptr_y++ = extract_h(L_shl(s, centering));

    }
    tmp = add(L,Kdiv2);
    FOR (i = i; i < tmp; i+=decimation)
    {
        s = L_deposit_l(0);
        ptr_h = h + i - L + 1;
        move16();
        ptr_x = x + L - 1;
        move16();

        FOR (j = add(sub(i,L),1); j <= K; j++)
        {
            s = L_mac0(s, *ptr_h++, *ptr_x--);
        }

        *ptr_y++ = extract_h(L_shl(s, centering));
    }

    return;
}

static
void fec_ecu_pitch_fx(
    const Word16 *prevsynth_fx, /*Q15 16 */
    Word16 *prevsynth_LP_fx, /* Q15 16 */
    const Word16 L,
    Word16 *N,
    Word16 *min_corr_fx,   /* Q15 16 */
    Word16 *decimatefator,
    const Word16 HqVoicing
)
{

    Word16 i,filt_size;
    Word16 QAsr,Ryy,cb_start, tmpQLP;
    Word32 Ryytmp;
    Word32 accA,accB, accBisqrt,accC, accCisqrt;
    Word16 delay_ind,k;
    const Word16 *Asr_LP_fx;
    Word16 *ptr_LP, *ptr_LP2, *ptr_LP3, *ptr_LP4;
    Word16 cb_end;
    Word16 Lmul2, Lon20mul6, Lon20mul28, Lon20mul33, Lon20mul34;


    SWITCH(L)
    {
    case L_FRAME48k:
        *decimatefator=6;
        move16();
        filt_size=60;
        move16();
        Asr_LP_fx = Asr_LP48_fx;
        QAsr = 17;
        move16();
        Lon20mul6 = 48;
        move16();
        Lon20mul28 = 224;
        move16();
        Lon20mul33 = 264;
        move16();
        Lon20mul34 = 272;
        move16();
        Lmul2 = 1920;
        move16();
        BREAK;

    case L_FRAME32k:
        *decimatefator=4;
        move16();
        filt_size=40;
        move16();
        Asr_LP_fx = Asr_LP32_fx;
        QAsr = 15;
        move16();
        Lon20mul6 = 48;
        move16();
        Lon20mul28 = 224;
        move16();
        Lon20mul33 = 264;
        move16();
        Lon20mul34 = 272;
        move16();
        Lmul2 = 1280;
        move16();
        BREAK;

    case L_FRAME16k:
        *decimatefator=2;
        move16();
        filt_size=20;
        move16();
        Asr_LP_fx = Asr_LP16_fx;
        QAsr = 15;
        move16();
        Lon20mul6 = 48;
        move16();
        Lon20mul28 = 224;
        move16();
        Lon20mul33 = 264;
        move16();
        Lon20mul34 = 272;
        move16();
        Lmul2 = 640;
        move16();
        BREAK;

    default:
        *decimatefator=2;
        move16();
        filt_size=40;
        move16();
        Asr_LP_fx = Asr_LP16_fx;
        QAsr = 15;
        move16();
        Lon20mul6 = 48;
        move16();
        Lon20mul28 = 224;
        move16();
        Lon20mul33 = 264;
        move16();
        Lon20mul34 = 272;
        move16();
        Lmul2 = 320;
        move16();
        BREAK;
    }


    /* Resampling to work at 8Khz */
    fir_dwn_fx(prevsynth_fx, Asr_LP_fx, QAsr, prevsynth_LP_fx, Lmul2, filt_size, *decimatefator); /* resampling without delay */


    tmpQLP = Find_Max_Norm16(prevsynth_LP_fx,320);
    Scale_sig(prevsynth_LP_fx,320,sub(tmpQLP,3)); /* to avoid over scaling */



    /* Correlation analysis */
    *min_corr_fx = 0;
    move16();
    accC = L_deposit_l(0);

    ptr_LP = prevsynth_LP_fx + Lon20mul34;
    FOR (k = 0; k < Lon20mul6; k++)
    {
        accC = L_mac0 (accC, *ptr_LP, *ptr_LP);
        ptr_LP++;

    }



    IF (HqVoicing==1)
    {
        cb_start=0;
        cb_end=Lon20mul33;
    }
    ELSE
    {
        cb_start=0;
        cb_end=Lon20mul28;
    }

    accB=0;
    delay_ind=cb_start;

    FOR (i=cb_start; i<cb_end; i++) /* cb_end = 35 let 6 ms min of loop size */
    {
        accA=0;
        IF (i==cb_start)
        {
            accB=0;
            ptr_LP = prevsynth_LP_fx;
            ptr_LP2 = prevsynth_LP_fx + Lon20mul34;
            FOR (k = 0; k < Lon20mul6; k++)
            {
                accA = L_mac0 (accA, *ptr_LP, *ptr_LP2);
                accB = L_mac0 (accB, *ptr_LP, *ptr_LP);
                ptr_LP++;
                ptr_LP2++;

            }

        }
        ELSE
        {


            accB=L_mac0(L_msu0(accB,prevsynth_LP_fx[i-1],prevsynth_LP_fx[i-1]),prevsynth_LP_fx[i+Lon20mul6-1],prevsynth_LP_fx[i+Lon20mul6-1]); /* tmpQLP-5  */


            ptr_LP3 = prevsynth_LP_fx + i;
            ptr_LP4 = prevsynth_LP_fx + Lon20mul34;

            FOR (k = 0; k < Lon20mul6; k++)
            {
                accA = L_mac0 (accA, *ptr_LP3, *ptr_LP4); /*2* tmpQLP-5  */
                ptr_LP3++;
                ptr_LP4++;
            }

        }

        /*accB*/
        /*accC*/
        /*accA Q 6-2*tmpQLP*/

        accBisqrt = Isqrt(accB); /* Q31 - 3 +  tmpQLP */
        accCisqrt = Isqrt(accC); /* Q31 - 3 +  tmpQLP*/


        Ryytmp = mult_32_32_q (accA, accCisqrt, 16+3-tmpQLP);   /*Q   6 -2*tmpQLP +  Q31 - 3 +  tmpQLP -16 -3 +tmpQLP      = Q15*/
        Ryytmp = mult_32_32_q (Ryytmp, accBisqrt, 15-3+tmpQLP); /*Q15 + Q31 - 3 +  tmpQLP -15 + 3- tmpQLP = Q31*/
        Ryy = extract_h(Ryytmp); /* Q15 */



        IF( Ryy  > *min_corr_fx  )
        {
            *min_corr_fx = Ryy;
            delay_ind = i;
        }

        test();
        IF ( HqVoicing == 0 && *min_corr_fx > 31130)
        {
            BREAK;
        }
    }

    *N= sub(Lon20mul34, delay_ind);


    Scale_sig(prevsynth_LP_fx,320,negate(sub(tmpQLP,3)));
    return;
}

static
void sin_cos_est_fx(Word32 phi, Word16 *cosfreq, Word16 *sinfreq)
{


    /* i phi : normalized frequency beteween 0 and Pi (nyquist) in Q30 */
    /* o  cosfreq & sinfreq : cos(phi) and sin (phi) in Q15 */

    Word16 i;
    Word32 delta;
    Word32 imin;
    Word16 sinb, cosb;
    Word32 sinv, cosv, tmp;

    i=0;
    move16();
    FOR (imin=0; imin<phi; imin+=4392264)
    {
        i = add(i,1);
    }

    delta = L_sub(phi, imin); /*Q30 */
    sinb = sincos_t_rad3_fx[i];
    move16();/*Q15 */
    cosb = sincos_t_rad3_fx[384-i];
    move16();/*Q15 */

    /*sinv = sin(phi) = sinb*cos(delta)+cosb*sin(delta) */
    /*cosv = cos(phi) = cosb*cos(delta)-sinb*sin(delta) */
    /*sin(delta) is approximated by delta for very small delta; cos(delta) is approximated by 1 */
    tmp = L_shl(Mult_32_16(delta, cosb),1); /*Q31 */
    sinv = L_add(L_deposit_h(sinb), tmp);
    tmp = L_shl(Mult_32_16(delta, sinb),1); /*Q31 */
    cosv = L_sub(L_deposit_h(cosb), tmp);

    *sinfreq = round_fx(sinv);
    *cosfreq = round_fx(cosv);

    return;

}

static
Word16 abs_iter_fx(Word16 re, Word16 im, Word16 N)
{
    Word16 A, tmp, L_tmp1, L_tmp2;
    Word16 i, exp;

    /*const Word16 cor[10] =  { 23170,      20724,      20106,      19950,      19911,      19902,      19899,      19899,    19899,      19898}; */

    exp = norm_s(re);
    exp = s_min(exp, norm_s(im));
    exp = sub(exp, 2);

    re = shl(re, exp);
    im = shl(im, exp);


    IF ( im < 0)
    {
        im = negate(im);
    }
    ELSE
    {
        re = negate(re);
    }
    tmp = re;
    move16();
    re = im;
    move16();
    im = tmp;
    move16();


    FOR (i=0; i<N; i++)
    {
        L_tmp1 = shr(im, i);
        L_tmp2 = shr(re, i);
        IF (im < 0)
        {
            L_tmp1 = negate(L_tmp1);

        }
        ELSE
        {
            L_tmp2 = negate(L_tmp2);
        }
        re = add(re, L_tmp1);
        im = add(im, L_tmp2);
    }
    i = s_min(sub(i,1), 9);
    tmp = abs_s(re);
    /*A = round_fx(L_shr(L_mult(tmp, cor[i]), exp)); //this can be ommited, if we don't need the exact abs value */
    A = shr(tmp, exp);

    return A;
}

static
void fec_ecu_dft_fx(
    const Word16 *prevsynth_LP, /*Qin */
    const Word16 N,
    Word16 *Tfr,  /*Qout */
    Word16 *Tfi, /*Qout */
    Word32 *sum_Tf_abs, /*Qout; */
    Word16 *Tf_abs, /*Qout */
    Word16 *Nfft,
    Word16 *exp /*Qout = Qin+exp */
)
{
    Word32 L_tmp, Tmp, Tfr32[512], Tfi32[512], fac, *Pt1, *Pt2;
    Word16 i, tmp, tmp_short,N_LP, target[2*L_FRAME48k], Tfr16[FEC_FFT_MAX_SIZE], *pt1, *pt2, *pt3;
    Word16 tmp_loop;

    tmp = sub(296,N);
    Copy(&prevsynth_LP[tmp], target, N);



    /* DFT  */

    L_tmp=L_deposit_l(N);
    FOR (tmp = 0; L_tmp <= 16384; tmp++)
    {
        L_tmp <<= 1;

    }
    *Nfft = shl(1,sub(15,tmp));


    set32_fx(Tfr32,0,*Nfft);
    set32_fx(Tfi32,0,*Nfft);
    Tfr16[0] = target[0];
    move16();
    Tfr16[*Nfft-1] = target[N-1];
    move16();

    IF(sub(*Nfft,N)==0)
    {
        Copy(&target[1],&Tfr16[1],*Nfft-2);
    }
    ELSE
    {

        tmp = div_s(sub(N,1), sub(*Nfft,1));
        Tmp = L_deposit_l(tmp);
        fac = L_add(Tmp, 0);
        tmp_loop = sub(*Nfft,1);
        FOR (i=1; i<tmp_loop; i++) /* interpolation for FFT */
        {
            tmp_short = extract_l(L_shr(Tmp,15));
            tmp = extract_l(L_msu(Tmp,tmp_short,16384));
            L_tmp = L_mult(sub(target[tmp_short+1],target[tmp_short]), tmp); /*Qin+16 */
            Tfr16[i] = add(target[tmp_short],round_fx(L_tmp));
            move16();/*Qin */
            Tmp = L_add(Tmp,fac);
        }

    }


    /*to avoid overflow in DoRTFTn_fx() */
    tmp = Exp16Array(*Nfft, Tfr16);
    *exp = add(tmp,add(2, norm_s(*Nfft)));
    Copy_Scale_sig_16_32(Tfr16, Tfr32, *Nfft, *exp); /*Qin+exp; */

    DoRTFTn_fx(Tfr32,Tfi32,*Nfft);
    N_LP = shr(*Nfft,1);


    L_tmp = L_deposit_l(0);

    pt1 = Tfr;
    pt2 = Tfi;
    pt3 = Tf_abs;
    Pt1 = Tfr32;
    Pt2 = Tfi32;
    FOR (i=0; i<N_LP; i++)
    {
        *pt1 = extract_h(*Pt1); /*Qin+exp-16 */
        *pt2 = extract_h(*Pt2); /*Qin+exp-16 */
        *pt3 = abs_iter_fx(Tfr[i], Tfi[i], 5);
        move16(); /*Qin+exp-16 */
        L_tmp = L_mac0(L_tmp, *pt3,1); /*Qin+exp-16 */
        pt1++;
        pt2++;
        pt3++;
        Pt1++;
        Pt2++;
    }
    *sum_Tf_abs = L_tmp;
    move32();
    *exp = sub(*exp,16);
    return;
}

static
void singenerator_fx(
    const Word16 L,       /* i  : size of output */
    const Word16 cosfreq, /* i  : cosine of 1-sample dephasing at the given frequency */
    const Word16 sinfreq, /* i  : sine   of 1-sample dephasing at the given frequency */
    const Word16 a_re,    /* i  : real part of complex spectral coefficient at the given frequency */ /*Qin */
    const Word16 a_im,    /* i  : imag part of complex spectral coefficient at the given frequency */ /*Qin */
    Word32 xx[]      /* o  : output vector */ /*Qin+16 */
)
{

    Word32 *ptr, L_C0, L_S0, L_C1, L_S1;
    Word16 C0, S0, C1, S1;
    Word16 i;

    L_S0 = L_deposit_l(0);  /*prevent warning*/
    L_C0 = L_deposit_h(a_re); /*Qin+16 */
    S0 = a_im;
    move16();

    ptr = xx;

    *ptr = L_add(*ptr, L_C0);
    move32();
    ptr++;

    FOR (i=0; i<L/2-1; i++)
    {
        C0 = extract_h(L_C0); /*Qin */
        L_C1 = L_mult(C0, cosfreq); /*Qin+16 */
        L_C1 = L_msu(L_C1, S0, sinfreq); /*Qin+16 */
        L_S1 = L_mult(C0, sinfreq);
        S1 = mac_r(L_S1, S0, cosfreq);
        *ptr = L_add(*ptr, L_C1);
        move32(); /*Qin+16 */
        ptr++;

        C1 = extract_h(L_C1);
        L_C0 = L_mult(C1, cosfreq);
        L_C0 = L_msu(L_C0, S1, sinfreq);
        L_S0 = L_mult(C1, sinfreq);
        S0 = mac_r(L_S0, S1, cosfreq);
        *ptr = L_add(*ptr, L_C0);
        move32();
        ptr++;
    }

    C0 = extract_h(L_C0);
    S0 = extract_h(L_S0);
    L_C1 = L_mult(C0, cosfreq);
    L_C1 = L_msu(L_C1, S0, sinfreq);
    *ptr = L_add(*ptr, L_C1);
    move32();
    ptr++;

    return;
}

static
void sinusoidal_synthesis_fx(
    const Word16 *Tfr, /*Qin */
    const Word16 *Tfi, /*Qin */
    Word16 *Tf_abs, /*Qin */
    const Word16 N,
    const Word16 L,
    const Word16 decimate_factor,
    const Word16 Nfft,
    const Word32 sum_Tf_abs, /*Qin */
    Word16 *synthesis, /*Qin */
    const Word16 HqVoicing,
    Word16 exp
)
{
    Word16 i,k,nb_pulses,indmax = 0,nb_pulses_final;
    Word16 pulses[FEC_MAX/2];
    Word16 mmax, maxi;
    Word32 cumsum, freq, L_tmp;
    Word16 freqi[FEC_NB_PULSE_MAX], tmp, q, inv_den,new_s,old,cpt;
    Word16 a_re[FEC_NB_PULSE_MAX], a_im[FEC_NB_PULSE_MAX];
    Word16 Lon20_10 = 80;
    Word16 flag, Len;
    Word16 cosfreq, sinfreq, sN,PL,glued;
    Word32 synthesis_fx[2*L_FRAME48k];
    Word16 *pt1, *pt2, *pt3, *pt4;

    flag = HqVoicing;
    move16();
    if (sub(N, Lon20_10)>0 )
    {
        flag = 1;
        move16(); /*flag corresponds to condition sub(N, Lon20_10)>0 || HqVoicing */
    }




    pt4 = pulses;
    nb_pulses=0;
    move16();
    PL=0;
    move16();
    cpt=0;
    move16();
    old=0;
    move16();
    glued=1;
    move16();
    new_s=Tf_abs[1];
    move16();
    if (flag )
    {
        PL=1;
        move16();
    }
    tmp=sub(shr(N,1),3);
    WHILE(sub(cpt,tmp)<=0)
    {
        test();
        IF(sub(Tf_abs[cpt],old)>0 && sub(Tf_abs[cpt],new_s)>0)
        {
            Word16 tmp2;

            glued=cpt;
            move16();

            tmp2 = add(add(cpt,PL),1);
            FOR (i=glued; i<tmp2; i++)
            {
                *pt4++=i;
                move16();
                nb_pulses++;
                move16();
            }
            old=Tf_abs[add(cpt,PL)];
            move16();
            new_s=Tf_abs[add(add(cpt,2),PL)];
            move16();
            cpt=add(add(cpt,PL),1);
            move16();
            glued=1;
            move16();
        }
        ELSE
        {
            old=Tf_abs[cpt];
            move16();
            new_s=Tf_abs[add(cpt,2)];
            move16();
            cpt++;
            glued=0;
            move16();
        }
    }


    nb_pulses_final = 0;
    move16();

    sN=sub(13,norm_s(Nfft)); /*for amplitude normalization by 2/nfft */

    cumsum = L_deposit_l(0);



    L_tmp = Mult_32_16(sum_Tf_abs, 22938);

    pt1 = a_re;
    pt2 = a_im;
    pt3 = freqi;

    maxi=s_min(FEC_NB_PULSE_MAX,nb_pulses);
    nb_pulses_final = maxi;
    move16();

    Len = shl(L,1);
    IF (HqVoicing)
    {
        FOR ( i=0; i<maxi; i++)
        {
            mmax = 0;
            move16();
            pt4 = pulses;
            FOR ( k=0; k<nb_pulses; k++)
            {
                tmp = *pt4++;
                move16();
                if (sub(Tf_abs[tmp], mmax)>0)
                {
                    indmax = tmp;
                    move16();
                }
                mmax = s_max(Tf_abs[tmp], mmax);
            }

            *pt1++ = Tfr[indmax];
            move16(); /*L_shr(Tfr[indmax], sN); //instead shr -> scaling of a_re is Qin+sN */
            *pt2++ = Tfi[indmax];
            move16(); /*L_shr(Tfi[indmax], sN); //instead shr -> scaling of a_im is Qin+sN */
            *pt3++ = indmax;
            move16();
            Tf_abs[indmax] = -1;
            move16();
        }

    }
    ELSE
    {
        DO
        {
            mmax = 0;
            move16();
            pt4 = pulses;
            FOR ( k=0; k<nb_pulses; k++)
            {
                tmp = *pt4++;
                move16();
                if (sub(Tf_abs[tmp], mmax)>0)
                {
                    indmax = tmp;
                    move16();
                }
                mmax = s_max(Tf_abs[tmp], mmax);
            }

            cumsum = L_mac0(cumsum, mmax,1);

            *pt1++ = Tfr[indmax];
            move16(); /*L_shr(Tfr[indmax], sN); //instead shr -> scaling of a_re is Qin+sN */
            *pt2++ = Tfi[indmax];
            move16(); /*L_shr(Tfi[indmax], sN); //instead shr -> scaling of a_im is Qin+sN */
            *pt3++ = indmax;
            move16();
            Tf_abs[indmax] = -1;
            move16();

            maxi=sub(maxi,1);
        } WHILE( maxi>0 && L_sub(cumsum, L_tmp)<0);

        nb_pulses_final = sub(nb_pulses_final,maxi);
        move16();
    }


    /* sinusoidal synthesis */


    set32_fx(synthesis_fx,0,Len);

    exp = add(exp,sN);


    pt1 = a_re;
    pt2 = a_im;
    pt3 = freqi;
    q = shr_r(N,2);
    if (sub(N,shl(q,2))>0)
    {
        q = add(q,1);
    }

    inv_den = i_mult2(N,decimate_factor);         /*Q0 */

    /*tmp = div_s(12868,inv_den);*/ /*Q15 */


    FOR ( i=0; i<nb_pulses_final; i++)
    {

        tmp = div_s(shl(*pt3,1),inv_den); /*Q15 */  /* ind*2/(N*decim) */
        freq = L_shl(L_mult(tmp, 25736),1); /*Q30 */ /* ind*2/(N*decim)*pi/4*4    never greater than PI/2 */
        sin_cos_est_fx(freq, &cosfreq, &sinfreq); /*cosfreq & sinfreq in Q15 */
        singenerator_fx( Len, cosfreq, sinfreq, *pt1, *pt2, synthesis_fx);  /*Qin      */

        pt1++;
        pt2++;
        pt3++;
    }
    Copy_Scale_sig_32_16(synthesis_fx, synthesis, Len, negate(add(exp,16))); /*Qin */



    return;
}

static
void fec_noise_filling_fx(
    const Word16 *prevsynth_fx, /*Qsynth   */
    Word16 *synthesis_fx, /*Qsynth */
    Word16 *ni_seed_forfec,
    const Word16 L,
    const Word16 N,
    const Word16 HqVoicing,
    Word16 *gapsynth_fx /*Qsynth */
)
{

    Word16 Rnd_N_noise;
    Word16 k,kk,i;
    Word16 N_noise;

    Word16 tmp_fx, ind, q1, q2, L20, flag;
    Word16 noisevect_fx[34*L_FRAME48k/20], SS_fx[L_FRAME48k/2];
    Word16 *pt1, *pt2, *pt3, *pt4, *pt5;
    const Word16 *pt6;
    Word32 L_tmp;
    const Word16 *sinq_tab;



    IF ( sub(L, L_FRAME32k) == 0 )
    {
        sinq_tab = sinq_32k;
    }
    ELSE IF ( sub(L, L_FRAME48k) == 0 )
    {
        sinq_tab = sinq_48k;
    }
    ELSE
    {
        sinq_tab = sinq_16k;
    }

    L20 = extract_h(L_mult(1639,L)); /*L/20 */

    /*N=47*L/20-delay_ind*decimatefator-6*L/20; */

    tmp_fx = sub(sub(shl(L,1),i_mult2(3,L20)),N);
    Copy(prevsynth_fx+tmp_fx,noisevect_fx,N);

    /* Noise addition on full band  */
    /* residual  */

    tmp_fx = s_min(N,L);
    N_noise = shr(tmp_fx,1);
    ind = sub(N, tmp_fx);
    pt1 = noisevect_fx;
    pt2 = pt1 + ind;
    move16();
    pt3 = &synthesis_fx[ind];
    move16();
    FOR (k=0; k<tmp_fx; k++)
    {
        (*pt1++) = sub((*pt2++),(*pt3++));
        move16();
    }

    IF (HqVoicing)
    {
        Scale_sig(noisevect_fx, N, -2);
    }

    kk = 0;
    move16();
    k = 0;
    move16();
    Rnd_N_noise = N_noise;
    move16();

    ind = shl(L,1);
    flag = ind;
    move16();
    pt5 = synthesis_fx;
    WHILE ( flag > 0 )
    {
        tmp_fx = Random(ni_seed_forfec);

        L_tmp = L_mac(1503264768, tmp_fx ,9830);
        if ( kk == 0 )
        {
            L_tmp = L_mac(1073741824, tmp_fx ,6554);
        }

        kk = sub(1,kk) ;
        tmp_fx = round_fx(L_tmp);
        Rnd_N_noise = extract_h(L_mult(N_noise, tmp_fx)); /*Q0 */


        tmp_fx = div_s(1, Rnd_N_noise); /*Q15 */
        tmp_fx = round_fx(L_shl(L_mult(tmp_fx, 25736),2)); /*Q15 */

        sinq_fx( shr(tmp_fx,1), shr(tmp_fx,2), Rnd_N_noise, SS_fx);

        pt2 = &noisevect_fx[N_noise];
        pt1 = pt2 - Rnd_N_noise;
        pt3 = SS_fx;
        pt4 = pt3 + sub(Rnd_N_noise,1);
        tmp_fx = s_min(Rnd_N_noise, flag);
        FOR ( i=0 ; i<tmp_fx; i++ )
        {
            L_tmp = L_mult((*pt1++), (*pt3++)); /*Qsynth+16 */
            L_tmp = L_mac(L_tmp, (*pt2++), (*pt4--)); /*Qsynth+16 */
            *pt5 = add(*pt5, round_fx(L_tmp));
            move16(); /*Qsynth */
            pt5++;
        }
        flag = sub(flag, tmp_fx);
    }

    q1 = i_mult2(7,L20);
    q2 = i_mult2(33,L20);

    Copy(synthesis_fx, synthesis_fx+q1,q2);
    Copy(synthesis_fx+L,gapsynth_fx,L);
    Copy(prevsynth_fx+sub(i_mult2(37, L20),q1),synthesis_fx,q1);
    pt1 = &synthesis_fx[q1];
    q2 = i_mult2(37,L20);
    pt6 = &prevsynth_fx[q2];
    tmp_fx = i_mult2(3,L20);

    /* overlappadd with the ms of valid mdct of the last frame */
    FOR (k=0; k<tmp_fx; k++)
    {
        L_tmp = L_mult(*sinq_tab, *sinq_tab); /*Q30 */
        sinq_tab++;
        q2 = round_fx(L_sub(2147483647, L_tmp)); /*Q15 */
        q1 = round_fx(L_tmp); /*Q15 */
        L_tmp = L_mult((*pt1), q1); /*Qsynth+16 */
        L_tmp = L_mac(L_tmp, (*pt6++), q2); /*Qsynth+16 */
        (*pt1++) = round_fx(L_tmp); /*Qsynth */
    }
}

static
void fec_alg_fx(
    const Word16 *prevsynth, /*Qin */
    const Word16 *prevsynth_LP, /*Qin */
    Word16 *ni_seed_forfec,
    Word32 *ecu_rec, /*Qin+16 (Qin+15 to be coherent witch other scaling) */
    const Word16 output_frame,
    const Word16 N,
    const Word16 decimatefactor,
    const Word16 HqVoicing,
    Word16 *gapsynth /*Qin */
)
{
    Word16 Nfft;
    Word32 sum_Tf_abs;
    Word16 Tfr[FEC_FFT_MAX_SIZE];
    Word16 Tfi[FEC_FFT_MAX_SIZE];
    Word16 Tf_abs[FEC_FFT_MAX_SIZE/2];
    Word16 synthesis[2*L_FRAME48k];
    Word16 exp;
    Word16 n,Q;

    fec_ecu_dft_fx( prevsynth_LP, N,
                    Tfr, Tfi, &sum_Tf_abs, Tf_abs, &Nfft, &exp );

    sinusoidal_synthesis_fx( Tfr, Tfi, Tf_abs, N, output_frame, decimatefactor, Nfft, sum_Tf_abs, synthesis, HqVoicing, exp );

    fec_noise_filling_fx(prevsynth,synthesis,ni_seed_forfec,output_frame,i_mult2(N, decimatefactor),HqVoicing,gapsynth);

    n = R1_48-R2_48;
    move16();

    test();
    IF (sub(output_frame,L_FRAME32k)==0 || sub(output_frame,L_FRAME16k)==0 )
    {
        n=R1_16-R2_16;
        move16();

        if(sub(output_frame,L_FRAME32k)==0)
        {
            n=2*N16_CORE_SW;
            move16();
        }

    }
    Q=0;
    move16();
    wtda_fx(synthesis+sub(output_frame,n),&Q,ecu_rec,
            NULL,
            NULL,ALDO_WINDOW,ALDO_WINDOW,output_frame);  /* return Q15 */

    return;
}

/*--------------------------------------------------------------------------
 *  hq_phase_ecu_fx()
 *
 *  Main routine for HQ phase ECU
 *--------------------------------------------------------------------------*/
static void hq_phase_ecu_fx(
    const Word16 *prevsynth,            /* i  : buffer of previously synthesized signal   */
    Word32 *ecu_rec,              /* o  : reconstructed frame in tda domain         */
    Word16 *time_offs,            /* i/o: Sample offset for consecutive frame losses*/
    Word16 *X_sav,                /* i/o: Stored spectrum of prototype frame        */
    Word16 *Q_spec,               /* i/o: Q value of stored spectrum                */
    Word16 *num_p,                /* i/o: Number of identified peaks                */
    Word16 *plocs,                /* i/o: Peak locations                            */
    Word32 *plocsi,               /* i/o: Interpolated peak locations           Q16 */
    const Word16 env_stab,              /* i  : Envelope stability parameter              */
    Word16 *last_fec,             /* i/o: Flag for usage of pitch dependent ECU     */
    Word16 *ph_ecu_active,        /* i  : Phase ECU active flag                     */
    const Word16 prev_bfi,              /* i   : indicating burst frame error             */
    const Word16 old_is_transient[2],   /* i   : flags indicating previous transient frames */
    Word16 *mag_chg_1st,          /* i/o: per band magnitude modifier for transients*/
    Word16 *Xavg,                 /* i/o: Frequency group average gain to fade to   */
    Word16 *beta_mute,            /* o   : Factor for long-term mute                */
    const Word16 output_frame           /* i   : frame length                             */
)
{
    Word16 lprot, offset;
    Word16 mag_chg[Lgw_max], ph_dith, X[Lprot48k];
    Word16 seed;
    Word16 alpha[Lgw_max], beta[Lgw_max];

    IF (sub(output_frame, L_FRAME48k) == 0)
    {
        lprot = Lprot48k; /* 1536 = (2*output_frame)*1024/1280 */               move16();
    }
    ELSE IF (sub(output_frame, L_FRAME32k) == 0)
    {
        lprot = Lprot32k;   /* 1024 */                                          move16();
    }
    ELSE IF (output_frame == L_FRAME16k)
    {
        lprot = 512;
        move16();
    }
    ELSE
    {
        lprot = 256;
        move16();
    }

    test();
    test();
    test();
    IF ( prev_bfi == 0 || (prev_bfi != 0 && *last_fec != 0 && (sub(*time_offs, output_frame) == 0)) )
    {
        test();
        if( !(prev_bfi != 0 && *last_fec != 0) )
        {
            *time_offs = 0;
            move16();
        }

        offset = sub(sub(shl(output_frame, 1), lprot), *time_offs);

        trans_ana_fx(prevsynth + offset, mag_chg, &ph_dith, mag_chg_1st, output_frame, *time_offs, env_stab,
                     *last_fec, alpha, beta, beta_mute, Xavg);

        spec_ana_fx(prevsynth + offset, plocs, plocsi, num_p, X_sav, output_frame, Q_spec);

        test();
        IF( prev_bfi != 0 && *last_fec != 0 )
        {
            *time_offs = add(*time_offs, output_frame);
        }
    }
    ELSE
    {
        *time_offs = add(*time_offs, output_frame);

        offset = sub(shl(output_frame, 1), lprot);
        trans_ana_fx(prevsynth + offset, mag_chg, &ph_dith, mag_chg_1st, output_frame, *time_offs, env_stab,
        0, alpha, beta, beta_mute, Xavg);
    }

    Copy(X_sav, X, lprot);

    /* seed for own_rand2 */
    seed = X[10];
    move16();
    seed = add(seed, *time_offs);

    subst_spec_fx(plocs, plocsi, num_p, *time_offs, X, mag_chg, ph_dith, old_is_transient, output_frame, &seed,
                  alpha, beta, *beta_mute, Xavg);

    /* reconstructed frame in tda domain */
    rec_frame_fx(X, ecu_rec, output_frame, *Q_spec);

    *last_fec = 0;
    *ph_ecu_active = 1;
    move16();
}


/*--------------------------------------------------------------------------
 *  hq_ecu()
 *
 *  Main routine for HQ ECU
 *--------------------------------------------------------------------------*/
void hq_ecu_fx(
    const Word16 *prevsynth,            /* i  : buffer of previously synthesized signal     */
    Word32 *ecu_rec,              /* o  : reconstructed frame in tda domain           */
    Word16 *time_offs,            /* i/o: Sample offset for consecutive frame losses  */
    Word16 *X_sav,                /* i/o: Stored spectrum of prototype frame          */
    Word16 *Q_spec,               /* i/o: Q value of stored spectrum                  */
    Word16 *num_p,                /* i/o: Number of identified peaks                  */
    Word16 *plocs,                /* i/o: Peak locations                              */
    Word32 *plocsi,               /* i/o: Interpolated peak locations             Q16 */
    const Word16 env_stab,              /* i  : Envelope stability parameter                */
    Word16 *last_fec,             /* i/o: Flag for usage of pitch dependent ECU       */
    const Word16 ph_ecu_HqVoicing,      /* i  : HQ Voicing flag                             */
    Word16 *ph_ecu_active,        /* i  : Phase ECU active flag                       */
    Word16 *gapsynth,             /* o  : Gap synthesis                               */
    const Word16 prev_bfi,              /* i  : indicating burst frame error                */
    const Word16 old_is_transient[2],   /* i  : flags indicating previous transient frames  */
    Word16 *mag_chg_1st,          /* i/o: per band magnitude modifier for transients  */
    Word16 *Xavg,                 /* i/o: Frequency group average gain to fade to     */
    Word16 *beta_mute,            /* o   : Factor for long-term mute                  */
    const Word16 output_frame,          /* i   : frame length                               */
    Decoder_State_fx *st_fx                 /* i/o: decoder state structure                     */
)
{
    Word16 N;
    Word16 decimatefactor;
    Word16 corr; /*Q15 */
    Word16 prevsynth_LP[2*L_FRAME8k];

    /* init (values ar changed after) */
    decimatefactor=4;
    N=shr(output_frame,2);


    /* find pitch and R value */

    IF (!(sub(output_frame,L_FRAME16k) < 0) )
    {
        fec_ecu_pitch_fx( prevsynth+NS2SA(output_frame*50,ACELP_LOOK_NS/2-PH_ECU_LOOKAHEAD_NS), prevsynth_LP, output_frame, &N, &corr, &decimatefactor, ph_ecu_HqVoicing);
    }
    ELSE
    {
        corr = 0;
        move16();  /* just to avoid using uninitialized value in if statement below */
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
    test();
    test();
    test();
    test();
    test();
    test();
    IF ( (L_sub(st_fx->total_brate_fx,48000) >= 0 &&
          ( sub(output_frame, L_FRAME16k) >= 0 && !prev_bfi && (!old_is_transient[0] || old_is_transient[1] ) && (ph_ecu_HqVoicing != 0 || ( ((st_fx->env_stab_plc_fx != 0) && (sub(corr,19661)<0)) || (!(st_fx->env_stab_plc_fx != 0) && (sub(corr, 27853) > 0 )))))) ||
         (L_sub(st_fx->total_brate_fx,48000) < 0  && ( ( ph_ecu_HqVoicing || sub(corr, 27853) > 0 ) && !prev_bfi && (!old_is_transient[0] || old_is_transient[1]) )) )
    {
        fec_alg_fx( prevsynth+NS2SA(output_frame*50,ACELP_LOOK_NS/2-PH_ECU_LOOKAHEAD_NS), prevsynth_LP, &st_fx->ni_seed_forfec, ecu_rec, output_frame, N, decimatefactor, ph_ecu_HqVoicing, gapsynth);

        *last_fec = 1;
        *ph_ecu_active = 0;
        move16();
        *time_offs = output_frame;
        move16();
    }
    ELSE
    {
        hq_phase_ecu_fx( prevsynth, ecu_rec, time_offs, X_sav, Q_spec, num_p, plocs, plocsi,
        env_stab, last_fec, ph_ecu_active, prev_bfi, old_is_transient, mag_chg_1st, Xavg, beta_mute, output_frame);
    }

    return;
}

/*******************************************************************************
 * The square root of x which MUST be 0.5 <= x < 1, i.e., x must be normalized.
 * sqrt(x) is approximated by a polynomial of degree n.
 *
 * sqrt(x) = a0 x^n + a1 x^(n-1) + a2 x^(n-2) + ... + an
 *         = (...((a0 x + a1) x + a2) x + ...) x + an
 *
 * The coefficients can be readily obtained by the following open source Octave
 * (or commercial Matlab) script:
 * order = 2;
 * N = 400;
 * x = linspace(0.5, 1.0, N);
 * y = sqrt(x);
 * p = polyfit(x, y, order)
 * z = polyval(p, x);
 * err = y - z;
 * plot(err);
 ******************************************************************************/

static Word16 sqrt2ndOrder(  /* o: in Q15 (2nd order least square approx.) */
    const Word16 x   /* i: x must be in between 0.5 and 1.0 (Q15). */
)
{
    Word32 acc;
    Word16 z;

    acc = 1890205600L;           /*  0.880195572812922 in Q31 */                move32();
    z = mac_r(acc, x, -6506);    /* -0.198537395405340 in Q15 */
    acc = 682030261L;            /*  0.317595089462249 in Q31 */                move32();
    z = mac_r(acc, z, x);        /* in Q15 */
    return z;
}

/*-----------------------------------------------------------------------------
 * windowing()
 *
 * Apply a symmetric Hamming or Hamming-Rectangular window to the signal.
 * If the "rectLength" parameter is zero, it is Hamming window; otherwise, the
 * rectLength signifies the length of the rectangular part of the Hamming-Rectangular
 * window.
 *--------------------------------------------------------------------------- */
static void windowing(
    const Word16 *x,         /* i: Input signal */
    Word16 *y,               /* o: Windowed output */
    const Word16 *win,       /* i: Window coefficients */
    const Word16 rectLength, /* i: Offset in between the 1st and 2nd symmetric halves of the Hamming window */
    const Word16 halfLength  /* i: Half of the total length of a complete Hamming window. */
)
{
    Word16 i;
    Word16 *pY;
    const Word16 *pX, *pW;
    pX = x;
    pW = win;
    pY = y;
    FOR (i = 0; i < halfLength; i++) /* 1st symmetric half of the Hamming window */
    {
        *pY++ = mult_r(*pX++, *pW++);
        move16();
    }
    FOR (i = 0; i < rectLength; i++) /* If rectLength is zero, it's a pure Hamming window; otherwise Hamming-Rectangular. */
    {
        *pY++ = *pX++;
        move16();
    }
    FOR (i = 0; i < halfLength; i++) /* 2nd symmetric half of the Hamming window. */
    {
        *pY++ = mult_r(*pX++, *(--pW));
        move16();
    }
}

/*-----------------------------------------------------------------------------
 * windowing_ROM_optimized()
 *
 * The coefficients of the Hamming window are derived from the sine table
 * shared with fft3_fx().
 * The entire Hamming-Rectangular window is decomposed into 5 segments:
 *   1. 1st half of the left half of the Hamming window
 *   2. 2nd half of the left half of the Hamming window
 *   3. The flat part of the rectangular region
 *   4. 1st half of the right half of the Hamming window
 *   5. 2nd half of the right half of the Hamming window
 *----------------------------------------------------------------------------*/
static void windowing_ROM_optimized(
    const Word16 *x,         /* i: Input signal */
    Word16 *y,               /* o: Windowed output */
    const Word16 downSamples,/* i: Offset in accessing the sine table. */
    const Word16 rectLength, /* i: Length of the rectangular portion (excluding the Hamming window part) */
    const Word16 halfLength  /* i: Half of the total length of the Hamming (excluding rectangular part) window */
)
{
    Word16 i, hamm, quarterLen, initOffset;
    Word16 *pY;
    const Word16 *pX, *pSine;
    Word32 acc;

    quarterLen = shr(halfLength, 1); /* 1/4 length of the entire Hamming (excluding the rectangular part) window. */
    initOffset = add(T_SIN_PI_2, shr(downSamples,1));
    pSine = sincos_t_rad3_fx + initOffset;
    pX = x;
    pY = y;

    /* 1st half of the left half of the Hamming window. */
    FOR (i = 0; i < quarterLen; i++)
    {
        pSine -= downSamples;   /* Decrement address counter */
        acc = L_deposit_h(FEC_HQ_WIN_A0);  /* Derive the Hamming window coefficient from the sine table. */
        hamm = msu_r(acc, *pSine, FEC_HQ_WIN_A1);
        *pY++ = mult_r(hamm, *pX++);
        move16();
    }

    /* 2nd half of the left half of the Hamming window. */
    FOR (i = 0; i < quarterLen; i++)
    {
        acc = L_deposit_h(FEC_HQ_WIN_A0);
        hamm = mac_r(acc, *pSine, FEC_HQ_WIN_A1);
        *pY++ = mult_r(hamm, *pX++);
        move16();
        pSine += downSamples;   /* Increment address counter */
    }

    /* The rectangular flat region */
    FOR (i = 0; i < rectLength; i++)
    {
        *pY++ = *pX++;
        move16();
    }

    /* 1st half of the right half of the Hamming window. */
    FOR (i = 0; i < quarterLen; i++)
    {
        pSine -= downSamples;   /* Decrement address counter */
        acc = L_deposit_h(FEC_HQ_WIN_A0);
        hamm = mac_r(acc, *pSine, FEC_HQ_WIN_A1);
        *pY++ = mult_r(hamm, *pX++);
        move16();
    }

    /* 2nd half of the right half of the Hamming window. */
    FOR (i = 0; i < quarterLen; i++)
    {
        acc = L_deposit_h(FEC_HQ_WIN_A0);
        hamm = msu_r(acc, *pSine, FEC_HQ_WIN_A1);
        *pY++ = mult_r(hamm, *pX++);
        move16();
        pSine += downSamples;   /* Increment address counter */
    }
}
