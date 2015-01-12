/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include "cnst_fx.h"


#ifndef EPS
#define EPS                     ( 1e-12f)
#endif

#define ENV_SCALE_OFFSET_1_FX   FL2WORD32(90.309f/128.0) /* 10*log10(2^30) */  /* scaled by 1/128.0 */
#define MAX_TEC_BW_LO             (12)
#define MAX_NB_TEC_LOW_BAND       ( 3)

static const Word16 TecSmoothingDeg = 5;
static const Word16 NbTecLowBand = 3;

#define LOBUF_NO_SMOOTHING_MODE               1

static const float ratioHiLoFac = 1.5894f;
static const float thRatio = 0.3649f;
static const float thRatio2 = 2.0288f;
static const float thCorrCoef = 0.8795f;
static const float ratioHiLoFacDec = 1.5894f * 0.75f;


void
resetTecDec_Fx(
    HANDLE_TEC_DEC_FX hTecDec
)
{

    set16_fx(hTecDec->pGainTemp_m, 0, CLDFB_NO_COL_MAX);
    set16_fx(hTecDec->pGainTemp_e, 0, CLDFB_NO_COL_MAX);

    set16_fx(hTecDec->loBuffer, 0, CLDFB_NO_COL_MAX + MAX_TEC_SMOOTHING_DEG);
    set16_fx(hTecDec->loTempEnv, 0, CLDFB_NO_COL_MAX);

    return;
}


void
resetTecEnc_Fx(HANDLE_TEC_ENC_FX hTecEnc, Word16 flag)
{
    IF (flag == 0)
    {
        set16_fx(hTecEnc->loBuffer, 0, CLDFB_NO_COL_MAX + MAX_TEC_SMOOTHING_DEG + DELAY_TEMP_ENV_BUFF_TEC);
        set16_fx(hTecEnc->loTempEnv, 0, CLDFB_NO_COL_MAX);
        set16_fx(hTecEnc->loTempEnv_ns, 0, CLDFB_NO_COL_MAX);
        set16_fx(hTecEnc->hiTempEnv, 0, CLDFB_NO_COL_MAX + DELAY_TEMP_ENV_BUFF_TEC + EXT_DELAY_HI_TEMP_ENV);
    }
    ELSE
    {
        set16_fx(hTecEnc->loBuffer, 0, MAX_TEC_SMOOTHING_DEG);
    }
    return;
}


static Word32 calcVar_Fix(const Word32 in[], Word32 len, Word32* x)
{
    Word32 xx;
    Word32 i;
    Word16 exp1;
    Word32 ans;
    Word16 r_sft;
    Word32 tmpX;

    xx = L_deposit_l(0);
    *x = L_deposit_l(0);
    FOR(i=0; i<len; i++)
    {

        exp1 = norm_l(in[i]);
        r_sft =sub(add(exp1,exp1),24);
        tmpX  = L_shr(Mpy_32_32(L_shl(in[i], exp1),L_shl(in[i], exp1)),r_sft);
        xx = L_add(xx,tmpX);
        *x = L_add(*x,in[i]);

    }

    ans = L_deposit_l(0);
    exp1 = norm_l(*x);
    r_sft = sub(add(exp1,exp1),24);
    tmpX = Mpy_32_32(L_shl(*x, exp1),L_shl(*x, exp1));
    tmpX = L_shr(tmpX,r_sft);
    IF( len == 0 )
    {
        return 0;
    }
    ELSE IF( L_sub(len,16) == 0)
    {
        ans = L_sub(xx,L_shr(tmpX,4) );

    }
    ELSE
    {
        /* Len = 16 Only */
        ans = L_deposit_l(0);

    }
    return ans;
}

static Word32
calcCorrelationCoefficient2_Fix(
    const Word32 in_vec1[], const Word32 in_vec2[], Word32 len,
    Word32 var_x, Word32 var_y, Word32 x, Word32 y)
{

    Word16 i;
    Word32 xy;
    Word32 xy2;
    Word32 ans;
    Word16 exp1;
    Word16 exp2;
    Word16 exp3;
    Word16 exp4;
    Word16 r_sft;
    Word32 tmpXY;
    Word32 tmpCor;
    Word32 Val1,Val2;


    xy = L_deposit_l(0);

    FOR(i=0; i<len; i++)
    {
        exp1 = norm_l(in_vec1[i]);
        exp2 = norm_l(in_vec2[i]);
        r_sft = sub(add(exp1,exp2),24);
        tmpXY  = L_shr(Mpy_32_32(L_shl(in_vec1[i], exp1),L_shl(in_vec2[i], exp2)),r_sft);
        xy = L_add(xy,tmpXY);
    }


    IF( L_sub(len,16) ==0)
    {
        exp1 = norm_l(x);
        exp2 = norm_l(y);
        r_sft = sub(add(exp1,exp2),24);
        xy2 =  L_shr(Mpy_32_32(L_shl(x, exp1),L_shl(y, exp2)),r_sft+4);
        Val1 = L_sub(xy,xy2);

        test();
        IF( var_x == 0 || var_y == 0)
        {
            ans = L_deposit_l(0);
        }
        ELSE
        {

            Word32 sqrtVal1;
            Word32 sqrtVal2;


            exp1 = norm_l(var_x);
            if( (exp1 & 0x1) !=0)
            {
                exp1 = sub(exp1,1);
            }
            exp2 = norm_l(var_y);
            if( (exp2 & 0x1) !=0)
            {
                exp2 = sub(exp2,1);
            }


            sqrtVal1 =  Sqrt_l(L_shl(var_x, exp1),&exp3);

            sqrtVal1 = L_shr(sqrtVal1,(shr(add(exp1,24),1)));

            sqrtVal2 =  Sqrt_l(L_shl(var_y, exp2),&exp4);
            sqrtVal2 = L_shr(sqrtVal2,(shr(add(exp2,24),1)));


            exp1 = norm_l(sqrtVal1);
            exp2 = norm_l(sqrtVal2);



            r_sft = sub(add(exp1,exp2),24);
            Val2 = L_shr(Mpy_32_32(L_shl(sqrtVal1,exp1),L_shl(sqrtVal2,exp2)),r_sft);


            exp1 = sub(norm_l(Val1),1);
            exp2 = norm_l(Val2);

            tmpCor = L_deposit_l(0);
            test();
            if( Val1 != 0 && Val2 != 0)
            {
                tmpCor = divide3232(L_shl(Val1, exp1), L_shl(Val2, exp2));
            }

            r_sft = sub(8,sub(exp2,exp1)) ;
            tmpCor = L_shr(tmpCor,r_sft);
            ans =tmpCor;
        }

    }
    ELSE
    {
        ans = 0;
    }


    return ans;
}

static void
calcLoBufferEnc_Fx (Word32 **pCldfbPow_Fx, /* Q31 */
                    Word16 scale,
                    Word16   startPos,
                    Word16   stopPos,
                    Word16   bandOffsetBottom,
                    Word16  *loBuffer /* Q8 = Q(15 - (LD_DATA_SCALE + 1)) */
                   )
{
    Word16 lb;
    Word16 li;
    Word16 ui;
    Word16 slot;
    Word32 nrg;
    Word32 nrgLog;
    Word32 tmp;

    FOR (slot=startPos; slot < stopPos; slot++)
    {
        tmp = L_deposit_l(0);

        /* scaling nrg : scale - 1 (sum up nrg) + 1 (inv_bw = 0.5) */

        FOR (lb=0; lb < NbTecLowBand; lb++)
        {
            li = TecLowBandTable[lb];
            move16();
            ui = sub(TecLowBandTable[lb+1],1);
            move16();

            assert( (ui-li) == 1 );

            /* sum up maximum of 2 nrg values, thus leave 1 bits headroom */
            nrg = L_add(L_shr(pCldfbPow_Fx[slot][li+bandOffsetBottom],1),
                        L_shr(pCldfbPow_Fx[slot][ui+bandOffsetBottom],1));


            /* assemble log2(EPS) */
            nrgLog = FL2WORD32(-0.31143075889);
            move32();

            IF ( nrg != 0 )
            {
                /* assemble log2 value and shift factor */
                nrgLog = L_sub(L_shr(BASOP_Util_Log2(nrg),1), L_shl(scale,(WORD32_BITS-1)-(LD_DATA_SCALE+1)));

                /* /\* assemble log2 value and shift factor *\/ */
                /* nrgLog = L_shr(BASOP_Util_Log2(nrg),1); */
            }

            tmp = L_add(tmp, nrgLog);
        }

        /* 0.50171665944 = 10 * log10(2.0) / NbTecLowBand / 2.0 */
        loBuffer[slot] = extract_h(L_shl(Mpy_32_16_1(tmp,FL2WORD16(0.50171665944)),1));

    }
}

static void
calcHiTempEnv_Fx(Word32 **pCldfbPow, /* Q31 */
                 Word16 scale,
                 Word16   startPos,
                 Word16   stopPos,
                 Word16   lowSubband,
                 Word16   highSubband,
                 Word16  *hiTempEnv /* Q7 = Q(15 - (LD_DATA_SCALE + 2)) */
                )
{
    Word16 k;
    Word16 s1;
    Word16 s2;
    Word16 bwHigh;
    Word32 nrg;
    Word32 nrgLog;
    Word16 normFac;
    Word16 timeIndex;

    s1 = 3;
    move16();

    bwHigh = sub(highSubband,lowSubband);

    normFac = getNormReciprocalWord16(bwHigh);

    scale = sub(scale, s1);
    move16();

    FOR (timeIndex=startPos; timeIndex < stopPos; timeIndex++)
    {
        nrg = L_deposit_l(0);

        FOR (k=lowSubband; k < highSubband; k++)
        {
            nrg = L_add(nrg,L_shr(pCldfbPow[timeIndex][k], s1));
        }


        s2 = norm_l(nrg);
        nrg = L_shl(nrg,s2);

        /* assemble log2(EPS) */
        nrgLog = FL2WORD32(-0.31143075889);
        move32();

        if ( nrg != 0 )
        {
            /* assemble log2 value and shift factor */
            nrgLog = L_shr(BASOP_Util_Log2(Mpy_32_16_1(nrg, normFac)),1);
            nrgLog = L_sub(nrgLog, L_shl(add(scale,s2),(WORD32_BITS-1)-(LD_DATA_SCALE+1)));
        }

        /* 0.75257498916 = 10 * log10(2.0) / 4.0 */
        hiTempEnv[timeIndex] = extract_h(L_shr(L_shl(Mpy_32_16_1(nrgLog,FL2WORD16(0.75257498916)),2),1));

    }
}

static void
calcLoBufferDec_Fx(Word32 **pCldfbReal,
                   Word32 **pCldfbImag,
                   Word16  *loBuffer,
                   Word16   startPos,
                   Word16   stopPos,
                   Word16   offset,
                   Word16   scale
                  )
{
    Word16 k;
    Word16 lb;
    Word16 s1;
    Word16 s2;
    Word16 slot;
    Word32 nrg32;
    Word32 tmp;
    Word32 maxVal;


    FOR (slot=startPos; slot < stopPos; slot++)
    {
        tmp = L_deposit_l(0);

        FOR (lb=0; lb < NbTecLowBand; lb++)
        {
            Word16 li;
            Word16 ui;

            li = TecLowBandTable[lb];
            move16();
            ui = TecLowBandTable[lb+1];
            move16();

            assert( (ui-li) == 2 );

            maxVal = L_deposit_l(0);

            /* determine maximum value */
            FOR (k=li; k < ui; k++)
            {
                maxVal = L_max(maxVal,L_abs(pCldfbReal[slot][k+offset]));
                maxVal = L_max(maxVal,L_abs(pCldfbImag[slot][k+offset]));
            }

            IF ( maxVal != 0 )
            {
                /* sum up maximum of 4 squared values, thus leave 2 bits headroom */
                s1 = sub(norm_l(maxVal),2);

                nrg32 = L_deposit_l(0);

                FOR (k=li; k < ui; k++)
                {
                    Word16 val;

                    val   = extract_h(L_shl(pCldfbReal[slot][k+offset], s1));
                    nrg32 = L_mac(nrg32, val, val);
                    val   = extract_h(L_shl(pCldfbImag[slot][k+offset], s1));
                    nrg32 = L_mac(nrg32, val, val);
                }

                /* square(scale) + square(s1) + 1(inv_bw = 0.5) */
                s2 = add(shl(add(scale,s1),1),1);

                /* assemble log value */
                tmp = L_add(tmp,L_shr(BASOP_Util_Log2(nrg32),1));

                /* add shift factors */
                tmp = L_sub(tmp,L_shl(s2,(WORD32_BITS-1)-(LD_DATA_SCALE+1)));

                /* add scale of reference */
                tmp = L_add(tmp,L_shl(30,(WORD32_BITS-1)-(LD_DATA_SCALE+1)));

                /* 0.50171665944 = 10 * log10(2.0) / NbTecLowBand / 2.0 */
                loBuffer[slot] = extract_h(L_shl(Mpy_32_16_1(tmp,FL2WORD16(0.50171665944)),1));
            }
            ELSE
            {
                /* 10 * log10(EPS) / 64.0 / 2.0 */
                loBuffer[slot] = FL2WORD16(-0.9375);
                move16();
            }
        }
    }
}

static void
calcLoTempEnv_Fx(Word16* loBuffer_Fx, /* Q8 = Q(15 - (LD_DATA_SCALE+1)) */
                 Word16 noCols,
                 Word16* loTempEnv_Fx, /* Q7 = Q(15 - (LD_DATA_SCALE+2)) */
                 Word16 adjFac_Fx
                )
{
    Word16 i;
    Word16 slot;
    Word32 accu;

    /* TecSC_Fx values are scaled by factor 2.0 */
    FOR (slot = 0; slot < noCols; slot++)
    {
        accu = L_mult0(TecSC_Fx[0], loBuffer_Fx[slot]);
        FOR (i=1; i < TecSmoothingDeg + 1; i++)
        {
            accu = L_mac0(accu,TecSC_Fx[i], loBuffer_Fx[sub(slot, i)]);
        }
        /* adjFac is scaled by factor 0.5 */
        loTempEnv_Fx[slot] = extract_h(Mpy_32_16_1(accu,adjFac_Fx));
    }
}

static void calcLoTempEnv_ns_Fx(Word16* loBuffer_Fx, /* Q8 = Q(15 - (LD_DATA_SCALE+1)) */
                                Word16 noCols,
                                Word16* loTempEnv_Fx /* Q7 = Q(15 - (LD_DATA_SCALE+2)) */
                               )
{
    Word16 slot;

    FOR (slot = 0; slot < noCols; slot++)
    {
        loTempEnv_Fx[slot] = shr(loBuffer_Fx[slot], 1);
        move16();
    }

    return;
}

static void
calcLoTempEnv_TBE_Fx(Word16* loBuffer_Fx, /* Q8 = Q(15 - (LD_DATA_SCALE+1)) */
                     Word16 noCols,
                     Word16* loTempEnv_Fx, /* Q7 = Q(15 - (LD_DATA_SCALE+2)) */
                     Word16 adjFac_Fx
                    )
{
    Word16 i;
    Word16 slot;
    Word32 accu;
    Word16 delay = 1;

    /* TecSC_Fx values are scaled by factor 2.0 */
    FOR (slot = 0; slot < noCols; slot++)
    {
        accu = L_mult0(TecSC_Fx[0], loBuffer_Fx[slot - delay]);
        FOR (i=1; i < TecSmoothingDeg + 1; i++)
        {
            accu = L_mac0(accu,TecSC_Fx[i], loBuffer_Fx[slot-i-delay]);
        }
        /* adjFac is scaled by factor 0.5 */
        loTempEnv_Fx[slot] = extract_h(Mpy_32_16_1(accu,adjFac_Fx));
    }
}

static void calcLoTempEnv_ns_TBE_Fx(Word16* loBuffer_Fx, /* Q8 = Q(15 - (LD_DATA_SCALE+1)) */
                                    Word16 noCols,
                                    Word16* loTempEnv_Fx /* Q7 = Q(15 - (LD_DATA_SCALE+2)) */
                                   )
{
    Word16 slot;
    Word16 delay = 1;
    Word16 fac = FL2WORD16(1.4f * 0.5f);

    FOR (slot = 0; slot < noCols; slot++)
    {
        /* fac is scaled by factor 0.5 */
        loTempEnv_Fx[slot] = mult_r(fac,  loBuffer_Fx[sub(slot, delay)]);
        move16();
    }

    return;
}

static void calcGainLinear_TBE_Fx(const Word16* loTempEnv_m, /* Q7 = Q(15 - (LD_DATA_SCALE+2)) */
                                  const Word16 startPos,
                                  const Word16 stopPos,
                                  Word16*  pGainTemp_m, /* Q0 */
                                  Word16*  pGainTemp_e  /* Q0 */)
{

    Word16 slot;
    Word16   c;
    Word32 logScaleFactor;


    /* 0.1 * Log2(10.0) scaled by 2.0 */
    c = FL2WORD16(2.0*0.33219280948);
    move16();

    FOR (slot=startPos; slot < stopPos; slot++)
    {
        Word16 s;
        Word16 s2;
        Word32 tmp32;

        s = 0;
        move16();

        /* adapt scale to LD_DATA_SCALE */
        tmp32 = L_shl(L_mult(c,loTempEnv_m[slot]),1);

        IF ( tmp32 > 0 )
        {
            IF  ( L_sub(tmp32,FL2WORD32(0.46875)) >= 0 )
            {
                s = add(s,sub(WORD32_BITS,norm_l(BASOP_Util_InvLog2(L_sub(tmp32,FL2WORD32(0.46875))))));
                s = add(s,30);
            }
            ELSE
            {
                s = add(s,sub(WORD32_BITS,norm_l(BASOP_Util_InvLog2(tmp32))));
            }

            /* scalefactor for logarithmic domain */
            logScaleFactor = L_shl(L_mult0(FL2WORD16(1.0/(1<<LD_DATA_SCALE)),s),16);

            /* scale in logaritmic domain */
            tmp32 = L_sub(tmp32,logScaleFactor);
        }

        /* inverse log2 */
        tmp32 = BASOP_Util_InvLog2(tmp32);

        /* determine headroom */
        s2 = norm_l(tmp32);

        pGainTemp_m[slot] = extract_h(L_shl(tmp32,s2));
        pGainTemp_e[slot] = sub(s,s2);
    }
}

void
calcGainTemp_TBE_Fx(
    Word32** pCldfbRealSrc_Fx,
    Word32** pCldfbImagSrc_Fx,
    Word16 cldfb_exp,
    Word16* loBuffer_Fx,
    Word16 startPos,          /*!<  Start position of the current envelope. */
    Word16 stopPos,           /*!<  Stop position of the current envelope. */
    Word16 lowSubband,   /* lowSubband */
    Word16* pGainTemp_m,
    Word16* pGainTemp_e
    , Word16 code
)

{
    Word16 loTempEnv_Fx[16];

    const Word16 BW_LO = TecLowBandTable[NbTecLowBand];
    Word16 slot;

    Word16 noCols = stopPos - startPos;
    Word16 bandOffset = lowSubband - BW_LO;

    assert(lowSubband >= BW_LO);

    calcLoBufferDec_Fx(
        pCldfbRealSrc_Fx,
        pCldfbImagSrc_Fx,
        loBuffer_Fx+MAX_TEC_SMOOTHING_DEG,
        startPos,
        stopPos,
        bandOffset,
        15 - cldfb_exp
    );

    IF (code > 0)
    {
        IF (sub(code, 2) != 0)
        {
            calcLoTempEnv_TBE_Fx(loBuffer_Fx + MAX_TEC_SMOOTHING_DEG, noCols,
                                 loTempEnv_Fx, FL2WORD16(0.5f * ratioHiLoFacDec));
        }
        ELSE
        {
            calcLoTempEnv_ns_TBE_Fx(loBuffer_Fx + MAX_TEC_SMOOTHING_DEG, noCols, loTempEnv_Fx);
        }

        calcGainLinear_TBE_Fx(loTempEnv_Fx, startPos, stopPos,
                              pGainTemp_m, pGainTemp_e);
    }

    FOR (slot = 0; slot < MAX_TEC_SMOOTHING_DEG; slot++)
    {
        loBuffer_Fx[slot] = loBuffer_Fx[slot + stopPos];
        move16();
    }
}

static void setSubfrConfig_Fix(Word16 i_offset, Word16* k_offset, Word16* n_subfr
                               , Word16 l_subfr
                              )
{
    *n_subfr = sub((Word16)N_TEC_TFA_SUBFR,i_offset);
    *k_offset = i_mult(i_offset,l_subfr);
}

static Word16 calcSum_Fx(Word16* vec_m, /*Q0*/
                         Word16* vec_e, /*Q0*/
                         const Word16 len,
                         Word16* sum16_m /*Q0*/)
{
    Word16 slot;
    Word16 sum16_e; /* Q0 */

    *sum16_m = 0;
    sum16_e = 0;
    FOR (slot=0; slot<len; slot++)
    {
        sum16_e = BASOP_Util_Add_MantExp (
                      *sum16_m,
                      sum16_e,
                      vec_m[slot],
                      vec_e[slot],
                      sum16_m
                  );
    }

    return sum16_e;
}


static Word16 calcSubfrNrg_Fx(Word16* hb_synth_Fx,
                              Word16 exp_syn,
                              const Word16 i_offset,
                              Word16* enr_m,
                              Word16* enr_e,
                              Word16 k_offset,
                              Word16* sum16_m
                              ,Word16 l_subfr
                             )
{
    Word16 i, j, k;
    Word16 s;
    Word16 sum16_e;
    Word16 s2 = 1; /* headroom for a summatoin of length l_subfr ( < 2^s2 = 64 ) */

    k = k_offset;
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        Word32 nrg32;

        nrg32 = L_deposit_l(0);

        FOR (j=0; j<l_subfr; j++)
        {
            Word32 sq;
            Word32 sq_shr;

            sq = L_mult(hb_synth_Fx[k], hb_synth_Fx[k]);
            sq_shr = L_shr(sq, s2);

            nrg32 = L_add(nrg32, sq_shr);
            k++;
        }

        IF (nrg32 > 0)
        {
            Word16 tmp;
            s = norm_l(nrg32);
            tmp = extract_h(L_shl(nrg32, s));
            enr_m[i] = tmp;
            move16();
            enr_e[i] = - s;
            move16();
        }
        ELSE
        {
            enr_m[i] = 0;
            move16();
            enr_e[i] = - 15;
            move16();
        }
    }

    sum16_e = enr_e[i_offset];
    *sum16_m = enr_m[i_offset];
    FOR (i=i_offset+1; i<N_TEC_TFA_SUBFR; i++)
    {

        sum16_e = BASOP_Util_Add_MantExp (
                      *sum16_m,
                      sum16_e,
                      enr_m[i],
                      enr_e[i],
                      sum16_m
                  );
    }

    /* exp_syn is not used up to here */
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        enr_e[i] = add(s2, add(enr_e[i], shl(exp_syn, 1)));
        move16();
    }
    sum16_e = add(s2, add(sum16_e, shl(exp_syn, 1)));

    return sum16_e;
}

static Word16 procTec_Fx(Word16* hb_synth_Fx,
                         Word16 exp_syn,
                         Word16* gain_m,
                         Word16* gain_e,

                         Word16 i_offset,
                         Word16 l_subfr
                         , Word16 code
                        )
{
    Word16 i,j,k;
    Word16 k_offset, n_subfr;

    Word16 max_inv_curr_enr_m;
    Word16 max_inv_curr_enr_e;

    Word16 min_curr_enr_m;
    Word16 min_curr_enr_e;

    Word16 lower_limit_gain_m;
    Word16 lower_limit_gain_e;

    Word16 upper_limit_gain_m;
    Word16 upper_limit_gain_e;

    Word16 enr_m[N_TEC_TFA_SUBFR];
    Word16 enr_e[N_TEC_TFA_SUBFR];

    Word16 gain_ave_m;
    Word16 gain_ave_e;

    Word16 enr_ave_m;
    Word16 enr_ave_e;

    Word16 inv_curr_enr_m[N_TEC_TFA_SUBFR];
    Word16 inv_curr_enr_e[N_TEC_TFA_SUBFR];

    Word16 inv_n_subfr;

    Word16 s;
    Word16 shift[L_FRAME_MAX];
    Word16 min_shift;

    setSubfrConfig_Fix(i_offset, &k_offset, &n_subfr, l_subfr);

    assert(8 /* = 2^3 */ < n_subfr);
    inv_n_subfr = getNormReciprocalWord16Scale(n_subfr, 3);

    enr_ave_e = calcSubfrNrg_Fx(hb_synth_Fx,
                                exp_syn,
                                i_offset,
                                enr_m,
                                enr_e,
                                k_offset,
                                &enr_ave_m
                                ,l_subfr
                               );

    /* divided by n_subfr */
    enr_ave_m = mult_r(enr_ave_m, inv_n_subfr);
    enr_ave_e = sub(enr_ave_e, 3);

    /* calculate the average of gain */
    gain_ave_e = calcSum_Fx(&gain_m[i_offset],
                            &gain_e[i_offset],
                            n_subfr,
                            &gain_ave_m);

    gain_ave_m = mult_r(gain_ave_m, inv_n_subfr);
    gain_ave_e = sub(gain_ave_e, 3);

    k = k_offset;
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        IF (enr_m[i] > 0)
        {
            BASOP_Util_Divide_MantExp(enr_ave_m, enr_ave_e,
                                      enr_m[i], enr_e[i],
                                      &inv_curr_enr_m[i], &inv_curr_enr_e[i]);
        }
        ELSE
        {
            /* 1.0e+12 */
            inv_curr_enr_m[i] = FL2WORD16(0.8631);
            move16();
            inv_curr_enr_e[i] = 39;
            move16();
        }

        s = norm_s(inv_curr_enr_m[i]);
        inv_curr_enr_m[i] = shl(inv_curr_enr_m[i], s);
        move16();
        inv_curr_enr_e[i] = sub(inv_curr_enr_e[i], s);
        move16();

        IF (gain_m[i] > 0)
        {
            BASOP_Util_Divide_MantExp(gain_m[i], gain_e[i],
                                      gain_ave_m, gain_ave_e,
                                      &gain_m[i], &gain_e[i]);
        }
        ELSE
        {
            return exp_syn;
        }
    }

    /* find the maximum of inv_curr_enr */
    max_inv_curr_enr_e = inv_curr_enr_e[i_offset];
    move16();
    max_inv_curr_enr_m = inv_curr_enr_m[i_offset];
    move16();

    FOR (i=i_offset+1; i<N_TEC_TFA_SUBFR; i++)
    {
        test();
        test();
        IF ( (max_inv_curr_enr_e < inv_curr_enr_e[i]) || (max_inv_curr_enr_e == inv_curr_enr_e[i] && max_inv_curr_enr_m < inv_curr_enr_m[i]))
        {
            max_inv_curr_enr_e = inv_curr_enr_e[i];
            move16();
            max_inv_curr_enr_m = inv_curr_enr_m[i];
            move16();
        }
    }

    /* evaluate lower limit of the gain's */
    BASOP_Util_Divide_MantExp(32767, 0,
                              max_inv_curr_enr_m, max_inv_curr_enr_e,
                              &min_curr_enr_m, &min_curr_enr_e);

    s = norm_s(min_curr_enr_m);
    min_curr_enr_m = shl(min_curr_enr_m, s);
    min_curr_enr_e = sub(min_curr_enr_e, s);

    lower_limit_gain_e = - 3;
    move16();
    lower_limit_gain_m = FL2WORD16(0.1 * 8); /* norm = 0 */move16();

    test();
    test();
    IF ( (sub(lower_limit_gain_e, min_curr_enr_e) > 0) || (sub(lower_limit_gain_e, min_curr_enr_e) == 0 && sub(lower_limit_gain_m, min_curr_enr_m) > 0) )
    {
        lower_limit_gain_m = min_curr_enr_m;
        move16();
        lower_limit_gain_e = sub(min_curr_enr_e, 1);
        move16();
    }

    /* upper_limit_gain */
    upper_limit_gain_m = FL2WORD16(0.6f /*1.2f * 0.5f*/); /* norm = 0 */ move16();
    upper_limit_gain_e = 1;
    move16();
    IF (sub(code,  LOBUF_NO_SMOOTHING_MODE) == 0)
    {
        upper_limit_gain_m = FL2WORD16(0.75f /*3.0f * 0.25f*/); /* norm = 0 */ move16();
        upper_limit_gain_e = 2;
        move16();
    }

    min_shift = 15; /* min_shift <= 15 */                  move16();

    k = k_offset;
    move16();
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        test();
        test();
        IF ( (sub(lower_limit_gain_e, gain_e[i]) > 0) || (sub(lower_limit_gain_e, gain_e[i]) == 0 && sub(lower_limit_gain_m, gain_m[i]) > 0))
        {
            gain_m[i] = lower_limit_gain_m;
            move16();
            gain_e[i] = lower_limit_gain_e;
            move16();
        }

        gain_m[i] = mult_r(gain_m[i], inv_curr_enr_m[i]);
        gain_e[i] = add(gain_e[i], inv_curr_enr_e[i]);

        s = norm_s(gain_m[i]);
        gain_m[i] = shl(gain_m[i], s);
        move16();
        gain_e[i] = sub(gain_e[i], s);
        move16();

        test();
        test();
        IF ( (sub(upper_limit_gain_e, gain_e[i]) < 0) || (sub(upper_limit_gain_e, gain_e[i]) == 0 && sub(upper_limit_gain_m, gain_m[i]) < 0))
        {

            gain_m[i] = upper_limit_gain_m;
            move16();
            gain_e[i] = upper_limit_gain_e;
            move16();
        }

        gain_m[i] = Sqrt16(gain_m[i], &gain_e[i]);
        move16();
        s = norm_s(gain_m[i]);
        gain_m[i] = shl(gain_m[i], s);
        move16();
        gain_e[i] = sub(gain_e[i], s);
        move16();

        FOR(j=0; j<l_subfr; j++)
        {
            s = norm_s(hb_synth_Fx[k]);
            hb_synth_Fx[k] = mult_r(gain_m[i], shl(hb_synth_Fx[k], s));
            move16();
            shift[k] = s - gain_e[i];
            move16();

            if (sub(min_shift, shift[k]) > 0)
            {
                min_shift = shift[k];
                move16();
            }
            k = add(k, 1);
        }
    }

    s = exp_syn;
    move16();

    exp_syn = sub(s, min_shift);
    if (exp_syn > 15)
    {
        exp_syn = 15;
        move16();
    }

    if (exp_syn < 0)
    {
        exp_syn = 0;
        move16();
    }

    min_shift = sub(s, exp_syn); /* exp_syn(old) - exp_syn(new) */

    k = k_offset;
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        FOR (j=0; j<l_subfr; j++)
        {
            s = sub(shift[k], min_shift);

            if (s > 0)
            {
                hb_synth_Fx[k] = shr(hb_synth_Fx[k], s);
                move16();
            }
            k = add(k, 1);
        }
    }

    return exp_syn;
}

static Word16 procTfa_Fx(Word16* hb_synth_Fx,
                         Word16 exp_syn,
                         Word16 i_offset,
                         Word16 l_subfr
                        )
{
    Word16 i,j,k;
    Word16 k_offset, n_subfr;

    Word16 enr_m[N_TEC_TFA_SUBFR];
    Word16 enr_e[N_TEC_TFA_SUBFR];

    Word16 enr_ave_m;
    Word16 enr_ave_e;

    Word16 inv_n_subfr;

    Word16 s;
    Word16 shift[L_FRAME_MAX];
    Word16 min_shift;

    Word16 gain_m[N_TEC_TFA_SUBFR];
    Word16 gain_e[N_TEC_TFA_SUBFR];

    setSubfrConfig_Fix(i_offset, &k_offset, &n_subfr, l_subfr);

    assert(8 /* = 2^3 */ < n_subfr);
    inv_n_subfr = getNormReciprocalWord16Scale(n_subfr, 3);

    enr_ave_e = calcSubfrNrg_Fx(hb_synth_Fx,
                                exp_syn,
                                i_offset,
                                enr_m,
                                enr_e,
                                k_offset,
                                &enr_ave_m
                                ,l_subfr
                               );

    enr_ave_m = mult_r(enr_ave_m, inv_n_subfr);
    enr_ave_e = sub(enr_ave_e, 3);

    min_shift = 15; /* min_shift <= 15 */   move16();

    k = k_offset;
    move16();
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        IF (enr_m[i] > 0)
        {
            BASOP_Util_Divide_MantExp(enr_ave_m, enr_ave_e,
                                      enr_m[i], enr_e[i],
                                      &gain_m[i], &gain_e[i]);
        }
        ELSE
        {
            /* gain = 1 */
            gain_m[i] = 32767;
            move16();
            gain_e[i] = 0;
            move16();
        }

        gain_m[i] = Sqrt16(gain_m[i], &gain_e[i]);
        move16();

        s = norm_s(gain_m[i]);
        gain_m[i] = shl(gain_m[i], s);
        move16();
        gain_e[i] = sub(gain_e[i], s);
        move16();

        FOR (j=0; j<l_subfr; j++)
        {
            s = norm_s(hb_synth_Fx[k]);
            hb_synth_Fx[k] = mult_r(gain_m[i], shl(hb_synth_Fx[k], s));
            move16();
            shift[k] = sub(s, gain_e[i]);
            move16();

            if (sub(min_shift, shift[k]) > 0)
            {
                min_shift = shift[k];
                move16();
            }
            k = add(k, 1);
        }
    }

    /* determin new exponent for hb_synth */
    s = exp_syn;
    move16();
    exp_syn = sub(s, min_shift);
    if (exp_syn > 15)
    {
        exp_syn = 15;
        move16();
    }

    if (exp_syn < 0)
    {
        exp_syn = 0;
        move16();
    }

    min_shift = sub(s, exp_syn); /* exp_syn(old) - exp_syn(new) */

    k = k_offset;
    move16();
    FOR (i=i_offset; i<N_TEC_TFA_SUBFR; i++)
    {
        FOR (j=0; j<l_subfr; j++)
        {
            s = sub(shift[k], min_shift);

            if (s > 0)
            {
                hb_synth_Fx[k] = shr(hb_synth_Fx[k], s);
                move16();
            }
            k = add(k, 1);
        }
    }

    return exp_syn;
}


Word16 procTecTfa_TBE_Fx(Word16 *hb_synth_Fx,
                         Word16 hb_synth_fx_exp,
                         Word16 *gain_m,
                         Word16 *gain_e,
                         Word16 flat_flag,
                         Word16 last_core
                         , Word16 l_subfr
                         , Word16 code
                        )
{
    Word16 i_offset = 0;
    Word16 exp_syn_frame = sub(15, hb_synth_fx_exp);


    IF (flat_flag)
    {
        exp_syn_frame = procTfa_Fx(hb_synth_Fx,
                                   exp_syn_frame,
                                   i_offset,
                                   l_subfr
                                  );
    }
    ELSE
    {
        if (sub(last_core, ACELP_CORE) != 0)
        {
            i_offset = 1;
            move16();
        }


        exp_syn_frame = procTec_Fx(hb_synth_Fx,
        exp_syn_frame,
        gain_m,
        gain_e,
        i_offset,
        l_subfr
        , code
                                  );

    }

    hb_synth_fx_exp = sub(15, exp_syn_frame);

    return hb_synth_fx_exp;
}


void
calcHiEnvLoBuff_Fix(
    Word16 noCols,
    Word16* pFreqBandTable,        /*!<  freqbandTable. */
    Word16 nSfb,                   /*!<  Number of scalefactors. */
    Word32** pCldfbPow_Fix			 /*float** pCldfbPow*/,
    Word16* loBuffer_Fix           /*float* loBuffer    Q8*/,
    Word16* hiTempEnvOrig_Fix      /*float* hiTempEnvOrig*/,
    Word16  pCldfbPow_FixScale
)
{
    const Word16 BW_LO = TecLowBandTable[NbTecLowBand];
    const Word16 lowSubband = pFreqBandTable[0];
    const Word16 highSubband = pFreqBandTable[nSfb];

    Word16 bandOffsetBottom;

    Word16* hiTempEnv = hiTempEnvOrig_Fix + EXT_DELAY_HI_TEMP_ENV;

    move16();
    move16();
    move16();
    bandOffsetBottom = sub(lowSubband, BW_LO);

    assert(bandOffsetBottom > 0);
    assert(noCols <= CLDFB_NO_COL_MAX);

    /* ============================================================ */
    /* =         calc hiTempEnv                                   = */

    calcHiTempEnv_Fx(pCldfbPow_Fix,sub(pCldfbPow_FixScale,30),0,noCols,lowSubband,highSubband,hiTempEnv + DELAY_TEMP_ENV_BUFF_TEC);

    /* =                                                          = */
    /* ============================================================ */

    /* ============================================================ */
    /* =         calc loBuffer                                    = */

    calcLoBufferEnc_Fx(pCldfbPow_Fix,sub(pCldfbPow_FixScale,30),0, noCols,bandOffsetBottom,loBuffer_Fix + MAX_TEC_SMOOTHING_DEG + DELAY_TEMP_ENV_BUFF_TEC);

    /* =                                                          = */
    /* ============================================================ */
}

void
calcLoEnvCheckCorrHiLo_Fix(
    Word16 noCols,
    Word16* pFreqBandTable,        /*!<  freqbandTable. */
    Word16* loBuffer_Fix           /*float* loBuffer    Q8*/,
    Word16* loTempEnv_Fix          /*float* loTempEnv  Q7*/,
    Word16* loTempEnv_ns_Fix       /*  float* loTempEnv_ns*/,
    Word16* hiTempEnvOrig_Fix      /*float* hiTempEnvOrig*/,
    Word16* corrFlag              /*int* corrFlag*/
)
{
    const Word16 BW_LO = TecLowBandTable[NbTecLowBand];
    const Word16 lowSubband = pFreqBandTable[0];
    Word16 i;

    Word16 bandOffsetBottom;

    Word32 hiVar_Fix, loVar_Fix;
    Word32 hiSum_Fix, loSum_Fix;
    Word32 hiTempEnv32_Fix[CLDFB_NO_COL_MAX + DELAY_TEMP_ENV_BUFF_TEC];
    Word32 loTempEnv32_Fix[CLDFB_NO_COL_MAX + DELAY_TEMP_ENV_BUFF_TEC];
    Word32 loTempEnv32_ns_Fix[CLDFB_NO_COL_MAX + DELAY_TEMP_ENV_BUFF_TEC];

    Word32 corrCoef_Fix;

    Word16 EQ1;
    Word32 EQ2,EQ3;
    Word32 EQ4,EQ5,EQ6;

    Word16 code = 0; /* SET TENTATIVELY */
    Word32 loVar_ns_Fix;
    Word32 diff_hi_lo_sum_Fix;
    Word32 loSum_ns_Fix;
    Word16* hiTempEnv = hiTempEnvOrig_Fix + EXT_DELAY_HI_TEMP_ENV;


    move16();
    move16();
    move16();
    bandOffsetBottom = sub(lowSubband, BW_LO);

    assert(bandOffsetBottom > 0);
    assert(noCols <= CLDFB_NO_COL_MAX);


    FOR( i = 0; i < noCols+ DELAY_TEMP_ENV_BUFF_TEC; i++)
    {
        hiTempEnv32_Fix[i] = L_deposit_l(hiTempEnv[i]);
    }
    hiVar_Fix = calcVar_Fix(hiTempEnv32_Fix, (Word32)noCols, &hiSum_Fix);


    /* ============================================================ */
    /* =         calc loTempEnv                                   = */



    calcLoTempEnv_Fx(loBuffer_Fix+MAX_TEC_SMOOTHING_DEG,noCols,loTempEnv_Fix,FL2WORD16(0.5 * ratioHiLoFac));


    calcLoTempEnv_ns_Fx(loBuffer_Fix+MAX_TEC_SMOOTHING_DEG,noCols,loTempEnv_ns_Fix);

    /* =                                                          = */
    /* ============================================================ */


    FOR( i = 0; i < noCols; i++)
    {
        loTempEnv32_ns_Fix[i] = L_deposit_l(loTempEnv_ns_Fix[i]);
    }
    loVar_ns_Fix = calcVar_Fix(loTempEnv32_ns_Fix, noCols, &loSum_ns_Fix);

    diff_hi_lo_sum_Fix = L_sub(loSum_ns_Fix,hiSum_Fix);

    EQ4 = L_sub(L_shr(hiVar_Fix,7),800);
    EQ5 = L_sub(L_shr(loVar_ns_Fix,7),720);;
    EQ6 = L_sub(L_shr(diff_hi_lo_sum_Fix,7),100); ;
    test();
    test();
    if (EQ4 > 0 && EQ5 > 0 && EQ6 < 0)
    {
        code = 1;
        move16();
    }



    *corrFlag = 0;
    move16();

    assert(code == 0 || code == 1);

    IF (code)
    {
        /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
        /* ++++                     code == 1                      +++++*/
        /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
        Word16 maxPosHi, maxPosLo;
        Word16 maxHiFix, maxLoFix;

        maxHiFix = hiTempEnv[0];
        move16();
        maxLoFix = loTempEnv_ns_Fix[0];
        move16();
        maxPosHi = maxPosLo = 0;
        move16();
        FOR (i = 1; i < noCols; i++)
        {
            if ( sub(maxHiFix , hiTempEnv[i]) < 0)
            {
                maxPosHi = i;
                move16();
            }
            maxHiFix = s_max(maxHiFix, hiTempEnv[i]);

            if ( sub(maxLoFix , loTempEnv_ns_Fix[i]) < 0)
            {
                maxPosLo = i;
                move16();
            }
            maxLoFix = s_max(maxLoFix, loTempEnv_ns_Fix[i]);
        }

        if (sub(abs_s( sub(maxPosHi , maxPosLo)), 2) < 0)
        {
            *corrFlag = 2;
            move16();
        }

        {

            Word16 feature_max_Fix = 0;
            Word16 pos_feature_max = 0;
            Word16 feature_Fix[16];
            Word16 min_local_Fix, max_local_Fix;
            Word16 j;
            Word16 len_window = EXT_DELAY_HI_TEMP_ENV + 1;

            Word16* curr_pos_Fix = hiTempEnv;

            move16();
            move16();
            move16();
            move16();

            feature_max_Fix = 0;
            move16();
            pos_feature_max = 0;
            move16();

            FOR (i = 0; i < 16; i++)
            {
                max_local_Fix = min_local_Fix = curr_pos_Fix[0];
                move16();
                move16();

                FOR (j = 1; j < len_window; j++)
                {
                    if (  sub( max_local_Fix , curr_pos_Fix[-j]) < 0)
                    {
                        max_local_Fix = curr_pos_Fix[-j];
                        move16();
                    }

                    if ( sub(min_local_Fix, curr_pos_Fix[-j] ) > 0)
                    {
                        min_local_Fix = curr_pos_Fix[-j];
                        move16();
                    }
                }
                feature_Fix[i] = sub(max_local_Fix , min_local_Fix);

                if (  sub(feature_max_Fix , feature_Fix[i]) < 0)
                {
                    pos_feature_max = i;
                    move16();
                }
                feature_max_Fix = s_max(feature_max_Fix, feature_Fix[i]);
                curr_pos_Fix += 1;
            }

            IF (*corrFlag > 0)
            {
                test();
                if ((sub(feature_max_Fix, shl(20, 7)) <= 0 || sub(abs_s(sub(pos_feature_max, maxPosHi)), 3) >= 0 ))
                {
                    *corrFlag = 0;
                    move16();
                }
            }
        }

    }
    ELSE
    {
        /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
        /* ++++                     code == 0                       ++++*/
        /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


        /* ============================================================ */
        /* =         calc the variance of loTempEnv                   = */

        FOR( i = 0; i < noCols; i++)
        {
            loTempEnv32_Fix[i] = L_deposit_l(loTempEnv_Fix[i]);
        }
        loVar_Fix = calcVar_Fix(loTempEnv32_Fix, noCols, &loSum_Fix);
        /* =                                                          = */
        /* ============================================================ */

        /* ============================================================ */
        /* =        calc correlation coefficient between              = */
        /* =        loTempEnv and hiTempEnv                           = */
        corrCoef_Fix = calcCorrelationCoefficient2_Fix(
            hiTempEnv32_Fix, loTempEnv32_Fix,
            noCols, hiVar_Fix, loVar_Fix, hiSum_Fix, loSum_Fix);
        /* =                                                          = */
        /* ============================================================ */




        EQ1 = sub(extract_l(corrCoef_Fix),shr(FL2WORD16(thCorrCoef),8) );
        EQ2 = L_sub(L_shl(hiVar_Fix,0),Mpy_32_16_1(loVar_Fix,FL2WORD16(thRatio)));
        EQ3 = L_sub(L_shr(hiVar_Fix,2),Mpy_32_16_1(loVar_Fix,FL2WORD16_SCALE(thRatio2,2)));

        test();
        test();
        if (EQ1 >= 0 && EQ2 > 0 /*ratio > thRatio*/ && EQ3 < 0)
        {
            *corrFlag = 1;
            move16();
        }

    }

    FOR (i = 0; i < MAX_TEC_SMOOTHING_DEG + DELAY_TEMP_ENV_BUFF_TEC; i++)
    {
        loBuffer_Fix[i] = loBuffer_Fix[noCols + i];
        move16();
    }

    FOR (i = 0; i < DELAY_TEMP_ENV_BUFF_TEC + EXT_DELAY_HI_TEMP_ENV; i++)
    {
        hiTempEnvOrig_Fix[i] = hiTempEnvOrig_Fix[noCols + i];
        move16();
    }

}

void tecEnc_TBE_fx(Word16* corrFlag, const Word16* voicing, Word16 coder_type)
{
    Word16 voice_sum;
    Word16 voice_diff;

    /*-----------------------------------------------------------------*
     * TEC updates
     *-----------------------------------------------------------------*/

    voice_sum = add(shr(voicing[0], 1), shr(voicing[1], 1)); /*voice_sum = voicing[0] + voicing[1];*/
    voice_diff = sub(voicing[0], voicing[1]); /*voice_diff = voicing[0] - voicing[1];*/

    if( voice_diff < 0 )
    {
        voice_diff = negate(voice_diff);/*voice_diff *= -1.0f;*/
    }

    IF( sub(*corrFlag, 1) == 0 )
    {
        test();
        test();
        test();
        /*if( ((voice_sum > 0.35 * 2 && voice_sum < 0.55 * 2) && (voice_diff < 0.2)) )*/
        if( sub(coder_type,INACTIVE) == 0 || ((sub(voice_sum, FL2WORD16(0.35)) > 0 && sub(voice_sum, FL2WORD16(0.55)) < 0) && (sub(voice_diff, FL2WORD16(0.2) < 0))) )
        {
            *corrFlag = 0;
            move16();
        }
    }
    if( sub(voice_sum, FL2WORD16(0.6)) > 0 ) /*if( voice_sum > 0.6 * 2 )*/
    {
        *corrFlag = 0;
        move16();
    }
}

void set_TEC_TFA_code_fx(const Word16 corrFlag, Word16* tec_flag, Word16* tfa_flag)
{
    *tec_flag = 0;
    move16();
    IF (*tfa_flag == 0)
    {
        test();
        if (sub(corrFlag, 1) == 0 || sub(corrFlag, 2) == 0)
        {
            *tec_flag = 1;
            move16();
        }

        if (sub(corrFlag, 2) == 0)
        {
            *tfa_flag = 1;
            move16();
        }
    }
}

