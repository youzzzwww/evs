/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include <assert.h>


#define MODE_DECISION_BASED_ON_PEAK_DETECTION


/*  static void setnoiseLevelMemory()
 *
 *      Helper function - updates buffer for minimumStatistics function
 */
static void setnoiseLevelMemory(Word16 f, Word16* new_noiseEstimate_e, Word16* noiseLevelMemory_e, Word16* noiseLevelMemory, Word16* currLevelIndex)
{
    noiseLevelMemory[*currLevelIndex] = f;
    move16();
    noiseLevelMemory_e[*currLevelIndex] = *new_noiseEstimate_e;
    move16();
}


/* PLC: [Common: Fade-out]
 * PLC: and for PLC fade out */

void minimumStatistics(Word16*       noiseLevelMemory,      /* Qx, internal state */
                       Word16*       noiseLevelIndex,       /* Q0, internal state */
                       Word16*       currLevelIndex,        /* Q0, internal state (circular buffer) */
                       Word16*       noiseEstimate,         /* Qx, previous estimate of background noise */
                       Word16*       lastFrameLevel,        /* Qx, level of the last frame */
                       Word16        currentFrameLevel,     /* Qx, level of the current frame */
                       Word16*       noiseLevelMemory_e,    /* scaling factor for noiseLevelMemory  */
                       Word16  const noiseEstimate_e,       /* exponent of noiseEstimate */
                       Word16*       new_noiseEstimate_e,   /* new exponent of noise Estimate*/
                       Word16* const lastFrameLevel_e,      /* exponent of lastFrameLevel    */
                       Word16        currentFrameLevel_e)   /* exponent of currentFrameLevel */
{
    Word16 aOpt, aOpt_e;
    Word16 f, p, i;
    Word16 tmp,tmp2, tmp_e;
    Word32 tmp32;
    move16();
    aOpt_e = 0;


    BASOP_SATURATE_WARNING_OFF
    IF (sub(shl(currentFrameLevel, currentFrameLevel_e),PLC_MIN_CNG_LEV) < 0)
    {
        BASOP_SATURATE_WARNING_ON
        currentFrameLevel = PLC_MIN_CNG_LEV;
        move16();
        move16();
        currentFrameLevel_e = 0;
    }
    BASOP_SATURATE_WARNING_ON

    /* compute optimal factor aOpt for recursive smoothing of frame minima */
    tmp2 = BASOP_Util_Add_MantExp(*lastFrameLevel,*lastFrameLevel_e,negate(*noiseEstimate),noiseEstimate_e,&tmp);
    IF (tmp >= 0)
    {
        /* aOpt = *noiseEstimate / *lastFrameLevel; */
        aOpt = BASOP_Util_Divide1616_Scale(*noiseEstimate, *lastFrameLevel, &aOpt_e);
        aOpt_e = add(aOpt_e, sub(noiseEstimate_e, *lastFrameLevel_e));
    }
    ELSE
    {
        /* aOpt = *lastFrameLevel / *noiseEstimate; */
        aOpt = BASOP_Util_Divide1616_Scale(*lastFrameLevel, *noiseEstimate, &aOpt_e);
        aOpt_e = add(aOpt_e, sub(*lastFrameLevel_e, noiseEstimate_e));
    }
    aOpt = mult_r(aOpt, aOpt); /* Q15 */
    aOpt_e = shl(aOpt_e,1);
    if (aOpt == 0)
    {
        move16();
        aOpt_e = 0;
    }

    *lastFrameLevel = currentFrameLevel;
    move16();
    move16();
    *lastFrameLevel_e = currentFrameLevel_e;

    /* recursively compute smoothed frame minima using optimal factor aOpt */
    tmp = *currLevelIndex;
    move16();
    move16();
    if (tmp == 0)
    {
        tmp = PLC_MIN_STAT_BUFF_SIZE;
        move16();

    }
    /*f = msu_r(L_mult(aOpt, noiseLevelMemory[sub(tmp, 1)]),  add(aOpt, 0x8000),  currentFrameLevel);*/
    /*f = (aOpt * noiseLevelMemory[tmp-1]) - (currentFrameLevel * (aOpt-1))*/
    /*tmp32*/                           /*tmp*/

    tmp32 =  L_mult(aOpt,noiseLevelMemory[tmp-1]); /*Q_tmp32 = aOpt_e + noiseLevelMemory_e[tmp - 1]*/
    move16();
    tmp_e = tmp;


    tmp2 = BASOP_Util_Add_MantExp(aOpt,aOpt_e,negate(32768/2),1,&tmp);
    tmp = mult_r(tmp,currentFrameLevel);                 /*Q_tmp = tmp2 + currentFrameLevel_e*/
    tmp2 = add(tmp2,currentFrameLevel_e);

    *new_noiseEstimate_e = BASOP_Util_Add_MantExp(round_fx(tmp32),add(aOpt_e,noiseLevelMemory_e[tmp_e - 1]),negate(s_max(tmp,-32767)/*to avoid negate(-32768)*/),tmp2,&f);

    assert(f >= 0);

    /* if current frame min is a new local min, set index to current index */
    p = *noiseLevelIndex;
    move16();
    tmp2 = BASOP_Util_Add_MantExp(noiseLevelMemory[p],noiseLevelMemory_e[p],negate(f),*new_noiseEstimate_e,&tmp);
    IF (tmp >= 0)
    {

        /*rescale noiseLevelMemory*/

        setnoiseLevelMemory(f,new_noiseEstimate_e,noiseLevelMemory_e, noiseLevelMemory, currLevelIndex);
        p = *currLevelIndex;
        move16();
    }
    ELSE
    {
        move16();

        setnoiseLevelMemory(f,new_noiseEstimate_e, noiseLevelMemory_e, noiseLevelMemory, currLevelIndex);

        /* current min is not a new min, so check if min must be re-searched */
        IF (sub(p, *currLevelIndex) != 0)
        {
            f = noiseLevelMemory[p];   /* min is still in memory, so return it */
            move16();
            *new_noiseEstimate_e = noiseLevelMemory_e[p];
        }
        ELSE {
            /* p == currLevelIndex; min was removed from memory, re-search min */
            FOR (i = 0; i < PLC_MIN_STAT_BUFF_SIZE; i++)
            {
                tmp2 = BASOP_Util_Add_MantExp(noiseLevelMemory[p],noiseLevelMemory_e[p],negate(noiseLevelMemory[i]),noiseLevelMemory_e[i],&tmp);
                if ( tmp > 0)
                {
                    p = i;
                    move16();
                }
            }
            f = noiseLevelMemory[p];
            move16();
            *new_noiseEstimate_e = noiseLevelMemory_e[p];
        }
    }

    /* update local-minimum-value index and current circular-buffer index */
    *noiseLevelIndex = p;
    move16();
    p = add(*currLevelIndex,1);
    *currLevelIndex = add(*currLevelIndex, 1);
    move16();
    if (sub(*currLevelIndex, PLC_MIN_STAT_BUFF_SIZE) == 0)
    {
        *currLevelIndex = 0;
        move16();
    }

    *noiseEstimate = f;
    move16();
}

/*----------------------------------------------------------------------*
 * PLC: [ACELP: Fade-out]
 * PLC: getLevelSynDeemph: derives on frame or subframe basis the level
 *      of LPC synthesis and deeemphasis based on the given input
 *----------------------------------------------------------------------*/
Word16 getLevelSynDeemph( /*10Q5*/
    Word16        h1Init[],     /* i: input value or vector to be processed */ /* Q15 */
    Word16  const A[],          /* i: LPC coefficients                      */ /* Qx  */
    Word16  const lpcorder,     /* i: LPC order                             */ /* Q0  */
    Word16  const lenLpcExc,    /* i: length of the LPC excitation buffer   */ /* Q0  */
    Word16  const preemph_fac,  /* i: preemphasis factor                    */ /* Q15 */
    Word16  const numLoops,     /* i: number of loops                       */ /* Q0  */
    Word16        *Exp          /* o: exponent of return value Q15          */
)
{
    Word32  levelSynDeemphSub;
    Word32  levelSynDeemph ;
    Word16  h1[L_FRAME_PLUS/4]; /*Q15*/
    Word16  mem[M];
    Word16  tmp;
    Word16  loop;
    Word16 s16, tmp16, Hr16;
    Word16 Q_h1;


    levelSynDeemphSub = L_deposit_l(0);
    levelSynDeemph = L_deposit_l(0);
    tmp = 0;
    Q_h1 = 9; /*synthesis scaling for */                                          move16();

    /*calculate headroom for dotproduct*/
    Hr16 = sub(15,norm_s(lenLpcExc));

    Q_h1 = s_max(sub(Q_h1,Hr16),0); /*compensate synthesis scaling with Headroom as much as possible to retain as much precision as possible*/

    /*Factor to be multiplied in order to calculate dotproduct with headroom*/
    tmp16 = shr(32768/2,sub(Hr16,1));

    /*moved from inside loop, before synthesis*/
    h1Init[0] = mult_r(h1Init[0],tmp16);
    move16();

    FOR (loop = 0; loop  < numLoops; loop++)
    {
        set16_fx(h1, 0, lenLpcExc);
        set16_fx(mem, 0, lpcorder);

        Copy(h1Init, h1, 1);
        /*h1 will be scaled down, Q_h1 */
        E_UTIL_synthesis(Q_h1, A, h1, h1, lenLpcExc, mem, 0, lpcorder);
        deemph_fx(h1, preemph_fac, lenLpcExc, &tmp);
        A += (M+1);

        /* gain introduced by synthesis+deemphasis */
        /*levelSynDeemphSub = (float)sqrt(dot_product( h1, h1, lenLpcExc));*/
        levelSynDeemphSub = Dot_product12_offs(h1, h1, lenLpcExc, &s16, 0);
        s16 = sub(shl(add(Q_h1,Hr16),1), sub(30, s16));

        levelSynDeemphSub = Sqrt32(levelSynDeemphSub,&s16); /*Q31*/

        /* mean of the above across all subframes  -- moved outta loop*/
        /*levelSynDeemph += (1.0/(float)numLoops) * levelSynDeemphSub;*/
        tmp16 = FL2WORD16(1.0f);
        move16();

        if (sub(numLoops , 1) > 0)
        {
            tmp16 = div_s(1,numLoops);
        }

        levelSynDeemph = L_add(levelSynDeemph , L_shl(Mpy_32_16_1(levelSynDeemphSub,tmp16),sub(s16,10))); /*10Q21*/

    }
    s16 = norm_l(levelSynDeemph);
    levelSynDeemph = L_shl(levelSynDeemph, s16);
    move16();
    *Exp = sub(10,s16); /*Set exponent in order to transform returnvalue to Q15*/

    return round_fx(levelSynDeemph); /*Q15*/
}

/* BASOP version: up to date with rev 7422 */
void genPlcFiltBWAdap(const Word32 sr_core, Word16 *lpFiltAdapt, const Word16 type, const Word16 alpha
                     )
{
    Word16 a, b, exp;


    assert(type == 0 || type == 1);

    IF ( L_sub(sr_core, 16000) == 0 )
    {
        IF (type == 0)
        {
            move16();
            move16();
            move16();
            *lpFiltAdapt++ = FL2WORD16(  0.4000f/(2.f*0.4000f+1.f));
            *lpFiltAdapt++ = FL2WORD16(      1.f/(2.f*0.4000f+1.f));
            *lpFiltAdapt   = FL2WORD16(  0.4000f/(2.f*0.4000f+1.f));
        }
        ELSE
        {
            a = mult_r(FL2WORD16(0.4000f), alpha);
            exp = 0;
            move16();
            b = Inv16(add(a, FL2WORD16(0.5f)), &exp);
            b = shr(b, sub(1, exp));
            a = negate(mult_r(a, b));
            move16();
            move16();
            move16();
            *lpFiltAdapt++ = a;
            *lpFiltAdapt++ = b;
            *lpFiltAdapt   = a;
        }
    }
    ELSE
    {
        IF (type == 0)
        {
            move16();
            move16();
            move16();
            *lpFiltAdapt++ = FL2WORD16(  0.2813f/(2.f*0.2813f+1.f));
            *lpFiltAdapt++ = FL2WORD16(      1.f/(2.f*0.2813f+1.f));
            *lpFiltAdapt   = FL2WORD16(  0.2813f/(2.f*0.2813f+1.f));
        }
        ELSE {
            a = mult_r(FL2WORD16(0.2813f), alpha);
            exp = 0;
            move16();
            b = Inv16(add(a, FL2WORD16(0.5f)), &exp);
            b = shr(b, sub(1, exp));
            a = negate(mult_r(a, b));
            move16();
            move16();
            move16();
            *lpFiltAdapt++ = a;
            *lpFiltAdapt++ = b;
            *lpFiltAdapt   = a;
        }
    }

}


/*-----------------------------------------------------------------*
 * PLC: [ACELP: general]
 * PLC: high pass filtering
 *-----------------------------------------------------------------*/
/*VERSIONINFO: This port is up to date with trunk rev. 32434*/
void highPassFiltering(
    const   Word16 last_good,    /* i:   short  last classification type                            */
    const   Word16 L_buffer,     /* i:   int    buffer length                                       */
    Word16 exc2[],       /* i/o: Qx     unvoiced excitation before the high pass filtering  */
    const   Word16 hp_filt[],    /* i:   Q15    high pass filter coefficients                       */
    const   Word16 l_fir_fer)    /* i:        high pass filter length                               */

{
    Word16   i; /*int*/

    IF( sub(last_good , UNVOICED_TRANSITION)> 0 )
    {

        FOR( i=0 ; i< L_buffer; i++ )
        {
            exc2[i] = round_fx(L_sub(Dot_product(&exc2[i], hp_filt, l_fir_fer), 1));
        }
    }
}

/*----------------------------------------------------------------------------------*
 * PLC: [Common: mode decision]
 * PLC: Decide which Concealment to use. Update pitch lags if needed
 *----------------------------------------------------------------------------------*/
Word16 GetPLCModeDecision(
    Decoder_State_fx *st              /* i/o:    decoder memory state pointer */
)
{
    Word16 /*int*/ core;
    Word16 numIndices = 0;


    IF( sub(st->flagGuidedAcelp,1) == 0 )
    {
        st->old_pitch_buf_fx[2*st->nb_subfr] = L_deposit_h(st->guidedT0);
        st->old_pitch_buf_fx[2*st->nb_subfr+1] = L_deposit_h(st->guidedT0);
        st->mem_pitch_gain[0] = st->mem_pitch_gain[1] = FL2WORD16_SCALE(1.f,1);/*Q14*/
    }
    st->plc_use_future_lag = 0;
    move16();
    test();
    test();
    if(( st->last_core_fx > ACELP_CORE && st->tcxltp_last_gain_unmodified!=0 )
            || ( sub(st->flagGuidedAcelp,1) == 0 )
      )
    {
        /* no updates needed here, because already updated in last good frame */
        st->plc_use_future_lag = 1;
        move16();
    }

    IF (sub(st->last_core_fx,-1) == 0)
    {
        core = TCX_20_CORE;
        move16();
        st->last_core_fx = ACELP_CORE;
        move16();
        if(st->Opt_AMR_WB_fx)
        {
            core = ACELP_CORE;
            move16();
        }

    }
    ELSE
    {
        core = ACELP_CORE;
        move16();
        if (sub(st->nbLostCmpt,1) > 0)
        {
            core = st->last_core_bfi;
            move16();
        }
        IF (sub(st->nbLostCmpt,1) == 0)
        {
            st->tonal_mdct_plc_active = 0;
            move16();
            test();
            test();
            test();
            IF ( !(st->rf_flag && st->use_partial_copy && (sub(st->rf_frame_type, RF_TCXTD1) == 0 || sub(st->rf_frame_type, RF_TCXTD2) == 0)))
            {
                test();
                test();
                test();
                test();
                test();
                test();
                IF ((sub(st->last_core_fx,TCX_20_CORE) == 0) && (sub(st->second_last_core,TCX_20_CORE) == 0)
                && ((L_sub(st->old_fpitch,L_deposit_h(shr(st->L_frame_fx,1)))) <= 0
                || (sub(st->tcxltp_last_gain_unmodified,FL2WORD16(0.4f)) <= 0))
                /* it is fine to call the detection even if no ltp information
                   is available, meaning that st->old_fpitch ==
                   st->tcxltp_second_last_pitch == st->L_frame */
                && (L_sub(st->old_fpitch, st->tcxltp_second_last_pitch) == 0)
                && !st->last_tns_active && !st->second_last_tns_active)
                {
                    Word16 *ptr = NULL;
                    Word32 pitch;


                    ptr = NULL;
                    move16();

                    pitch = L_deposit_h(0);
                    if(st->tcxltp_last_gain_unmodified > 0)
                    {
                        pitch = L_add(st->old_fpitch, 0);
                    }
                    TonalMDCTConceal_Detect(&st->tonalMDCTconceal,
                                            pitch,
                                            ptr,
                                            &numIndices);

                    test();
                    test();
                    test();
                    test();
                    test();
                    test();
                    IF ((sub(numIndices,10) > 0)
                        || ((sub(numIndices,5) > 0)
                            && (L_sub(L_abs(L_sub(st->tcxltp_third_last_pitch,st->tcxltp_second_last_pitch)),FL2WORD32_SCALE(0.5f,15)) < 0))
                        || ((numIndices > 0) && ((sub(st->last_good_fx,UNVOICED_TRANSITION) <= 0) || (sub(st->tcxltp_last_gain_unmodified,FL2WORD16(0.4f)) <= 0))
                            && (L_sub(L_abs(L_sub(st->tcxltp_third_last_pitch,st->tcxltp_second_last_pitch)),FL2WORD32_SCALE(0.5f,15)) < 0)))
                    {
                        core = TCX_20_CORE;
                        move16();
                        st->tonal_mdct_plc_active = 1;
                        move16();
                    }
                    ELSE if (sub(st->tcxltp_last_gain_unmodified,FL2WORD16(0.4f)) <= 0)
                    {
                        core = TCX_20_CORE;
                        move16();
                    }
                }
                ELSE IF (st->last_core_fx != ACELP_CORE)
                {
                    test();
                    if (sub(st->last_good_fx,UNVOICED_TRANSITION) <= 0 || sub(st->tcxltp_last_gain_unmodified,FL2WORD16(0.4f))<=0)
                    {
                        core = st->last_core_fx;
                        move16();
                    }
                }
            }
        }
    }
    return core;
}
