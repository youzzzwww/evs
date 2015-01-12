/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "stl.h"


/*==========================================================================*/
/* FUNCTION      : Word16 WB_BWE_gain_pred_fx ()             */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : predict WB frequency envelopes for 0b WB BWE        */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                            */
/* _Word16 *core_dec_freq        i  : Frequency domain core decoded signal  */
/* _Word16 coder_type            i  : coding type                           */
/* _Word16 prev_coder_type       i  : coding type of last frame             */
/* _Word16 prev_WB_fenv          i  : envelope for last frame               */
/* _Word16 *voice_factors        i  : voicing factors      //Q15            */
/* _Word16 pitch_buf[]           i  : pitch buffer         //Q6             */
/* _Word16  last_core_brate      i  : previous frame core bitrate           */
/* _Word16 last_wb_bwe_ener      i  : previous frame wb bwe signal energy   */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/* _Word16 *WB_fenv,             o  : WB frequency envelopes   Q3           */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*          _Word16 mode                                                    */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                                                            */
/*==========================================================================*/

Word16 WB_BWE_gain_pred_fx(
    Word16 *WB_fenv,                 /* o  : WB frequency envelopes               */
    const Word16 *core_dec_freq,           /* i  : Frequency domain core decoded signal */
    const Word16 coder_type,               /* i  : coding type                          */
    Word16 prev_coder_type,          /* i  : coding type of last frame            */
    Word16 prev_WB_fenv,             /* i  : envelope for last frame              */
    Word16 *voice_factors,           /* i  : voicing factors      //Q15           */
    const Word16 pitch_buf[],              /* i  : pitch buffer         //Q6            */
    Word32 last_core_brate,          /* i  : previous frame core bitrate          */
    Word16 last_wb_bwe_ener ,        /* i  : previous frame wb bwe signal energy */
    Word16 Q_syn
)
{
    Word32 enerL;
    Word16 n_freq, mode,pitch;
    Word16 ener_var_flag = 0;
    Word16 voice_factor, enerL_16, enerL_40, enerL_64;
    Word16 env_var_flag = 0;
    Word16 exp;
    Word16 tmp, tmp1;
    Word32 L_tmp;
    Word32 L_WB_fenv0, L_WB_fenv1;
    Word16 alfa = 32767;
    move16();

    mode = NORMAL;
    move16();

    enerL = L_deposit_l(0);
    FOR (n_freq = 128; n_freq<192; n_freq++)
    {
        enerL = L_mac0( enerL, core_dec_freq[n_freq], core_dec_freq[n_freq] ); /*2(Q_syn)  */
    }

    L_WB_fenv0 = L_deposit_l(0);
    FOR (n_freq = 192; n_freq<224; n_freq++)
    {
        L_WB_fenv0 = L_mac0(L_WB_fenv0, core_dec_freq[n_freq],core_dec_freq[n_freq]);/*2*Q_syn */
    }

    L_WB_fenv1 = L_deposit_l(0);
    FOR (n_freq = 224; n_freq<256; n_freq++)
    {
        L_WB_fenv1= L_mac0(L_WB_fenv1, core_dec_freq[n_freq],core_dec_freq[n_freq]);/*2*Q_syn */
    }

    L_tmp = sum16_32_fx( voice_factors, 4);
    voice_factor = extract_l(L_shr(L_tmp, 2));/*Q13 */
    pitch = sum16_fx(pitch_buf, 4) ;
    move16(); /*Q6 */

    L_tmp = L_shr(enerL, 4);
    test();
    if(L_sub(L_max(L_WB_fenv1, L_WB_fenv0), L_tmp) > 0  && sub(19712, pitch) > 0)
    {
        ener_var_flag = 1;
        move16();
    }

    test();
    IF(L_sub(L_WB_fenv0, L_shl(L_WB_fenv1,1)) > 0)
    {
        exp = norm_l(L_WB_fenv0);
        tmp = extract_h(L_shl(L_WB_fenv0, exp));
        tmp = div_s(16384, tmp); /*Q(15+14-2*Q_syn-exp) */
        L_tmp = L_shr(Mult_32_16(L_shl(L_WB_fenv1, 1), tmp), sub(15, exp));/*2*Q_syn+15+exp-15->2*Q_syn+exp */
        /*L_tmp Q15   */
        tmp = extract_l(L_tmp);
        alfa = s_max(tmp, 3277);/*Q15 */
        L_WB_fenv0 = Mult_32_16(L_WB_fenv0, alfa);/*2*Q_syn+15-15->2*Q_syn */
    }
    ELSE IF (L_sub(L_WB_fenv1, L_shl(L_WB_fenv0, 1)) > 0 && sub(coder_type,UNVOICED) != 0)
    {
        exp = norm_l(L_WB_fenv1);
        tmp = extract_h(L_shl(L_WB_fenv1, exp));
        tmp = div_s(16384, tmp); /*Q(15+14-2*Q_syn-exp) */
        L_tmp = L_shr(Mult_32_16(L_shl(L_WB_fenv0, 1), tmp), sub(15, exp));/*2*Q_syn+15+exp-15->2*Q_syn+exp */
        /*L_tmp Q15 */
        tmp  = extract_l(L_tmp);
        alfa = s_max(tmp, 3277);/*Q15 */
        L_WB_fenv1 = Mult_32_16(L_WB_fenv1, alfa);/*2*Q_syn+15-15->2*Q_syn                      */
    }

    if(L_WB_fenv0 == 0)
    {
        L_WB_fenv0 = L_deposit_l(1);
    }

    if(L_WB_fenv1 == 0)
    {
        L_WB_fenv1 = L_deposit_l(1);
    }

    if(enerL == 0)
    {
        enerL = L_deposit_l(1);
    }

    L_tmp = L_add(L_WB_fenv0, L_WB_fenv1);  /* In 2*Q_syn */
    exp = norm_l(L_tmp);
    tmp = extract_h(L_shl(L_tmp, exp));
    /*exp = sub(exp, 30-(2*Q_syn+6)); //+6(/64) */
    exp = sub(exp, sub(30,add(shl(Q_syn,1),6))); /*+6(/64) */

    tmp = div_s(16384, tmp);
    L_tmp = L_deposit_h(tmp);
    L_tmp = Isqrt_lc(L_tmp, &exp);

    WB_fenv[0] = round_fx(L_shl(L_tmp, sub(exp, 12))); /* Q3 */

    test();
    test();
    IF(sub(coder_type,AUDIO) != 0&& sub(coder_type,UNVOICED) != 0 && ener_var_flag == 0)
    {
        WB_fenv[0]= add(WB_fenv[0], mult_r(WB_fenv[0], 16384));
        move16();
    }

    exp = norm_l(enerL);
    tmp = extract_h(L_shl(enerL, exp));
    exp = sub(exp, sub(30,shl(Q_syn,1)));

    tmp = div_s(16384, tmp);
    L_tmp = L_deposit_h(tmp);
    L_tmp = Isqrt_lc(L_tmp, &exp);
    enerL_16 = round_fx(L_shl(L_tmp, sub(exp, 15))); /* Q0 */
    enerL_40 = mult_r(6554, enerL_16); /*1/40 in Q18  ->Q3    */

    test();
    test();
    test();
    test();
    test();
    test();
    IF( sub(coder_type,TRANSITION) != 0 && sub(coder_type,AUDIO) != 0 && sub(coder_type,UNVOICED) != 0 &&
        sub(enerL_40, WB_fenv[0]) > 0 && sub(alfa,29491) > 0 && !(sub(coder_type, prev_coder_type) == 0 &&
                sub(WB_fenv[0],prev_WB_fenv) > 0) )
    {
        IF(WB_fenv[0] != 0)
        {
            exp = norm_s(WB_fenv[0]);
            tmp = div_s(shl(1,sub(14,exp)), WB_fenv[0]); /*Q(29-exp-3) */
            L_tmp = L_mult(enerL_40, tmp); /*Q(30-exp) */
            tmp = round_fx(L_shl(L_tmp, sub(exp,2))); /*Q12 */
            tmp = s_min(tmp, 16384);
            L_tmp = L_shr( L_mult0(tmp, WB_fenv[0]), 12);/*Q15 */
            WB_fenv[0] = extract_l(L_tmp);/*Q3 */
        }

        IF( sub(WB_fenv[0],prev_WB_fenv ) > 0)
        {
            /*WB_fenv[0]= add(mult_r(9830, WB_fenv[0]), mult_r(22938, prev_WB_fenv));    move16();//Q3 */
            WB_fenv[0] = round_fx(L_mac(L_mult(9830, WB_fenv[0]), 22938, prev_WB_fenv));/*Q3 */
        }
    }

    L_tmp = L_mult0(voice_factor, 77);
    tmp1 = extract_l(L_shr(L_tmp, 13));

    exp = norm_s(pitch);
    tmp = div_s(shl(1,sub(14,exp)), pitch); /*Q(29-exp-6) */
    L_tmp = L_mult(tmp1, tmp); /*30-exp-6->24-exp */
    tmp = round_fx(L_shl(L_tmp, add(exp,6))); /*14          */

    tmp1= s_max(tmp, 8192);
    alfa = s_min(24576, tmp1);/*Q14 */

    L_tmp = L_mult0(alfa, WB_fenv[0]);/*Q14+Q3->Q17 */
    L_tmp = L_shr(L_tmp, 14);/*Q3 */
    tmp = extract_l(L_tmp);

    enerL = L_deposit_l(enerL_16);
    enerL = L_shl(enerL, 6); /*Q6 */
    tmp1 = i_mult(3, WB_fenv[0]);/*Q3 */
    L_tmp = L_mult0(tmp1, WB_fenv[0]); /*Q6 */

    test();
    test();
    IF( sub(shr(enerL_16, 3), tmp) > 0 && L_sub(enerL, L_tmp) > 0 && sub(prev_coder_type,UNVOICED) != 0 )
    {
        env_var_flag = 1;
        move16();
        enerL_64 = mult_r(4096, enerL_16);/* 1/64 in Q18  ->Q3 */

        exp = norm_s(WB_fenv[0]);
        tmp = div_s(shl(1,sub(14,exp)), WB_fenv[0]); /*Q(29-exp-3) */
        L_tmp = L_mult(enerL_64 ,tmp); /*Q(30-exp) */
        tmp = round_fx(L_shl(L_tmp, sub(exp,2))); /*Q12 */
        tmp = s_min(tmp, 16384);
        L_tmp = L_shr( L_mult0(tmp, WB_fenv[0]), 12);/*Q3 */
        WB_fenv[0] = extract_l(L_tmp);/*Q3 */

        IF( sub(WB_fenv[0],prev_WB_fenv) > 0 )
        {
            /*WB_fenv[0] = add(mult_r(9830, WB_fenv[0]), mult_r(22938, prev_WB_fenv));//Q3 */
            WB_fenv[0] = round_fx(L_mac(L_mult(9830, WB_fenv[0]), 22938, prev_WB_fenv));/*Q3 */
        }
    }

    test();
    IF(sub(coder_type,UNVOICED) == 0 || sub(prev_coder_type,UNVOICED) == 0)
    {
        WB_fenv[0] = shr(WB_fenv[0], 1);
        move16();/*Q3 */
    }

    IF(sub(coder_type,AUDIO) != 0)
    {
        tmp = mult_r(voice_factor, 19661);  /*Q12 */
        tmp = s_max(tmp, 4096);
        exp = norm_s(tmp);
        tmp = div_s(shl(1,sub(14,exp)), tmp); /*Q(29-exp-12) */
        L_tmp = L_mult(WB_fenv[0], tmp); /*Q(21-exp) */

        WB_fenv[0] = round_fx(L_shl(L_tmp, sub(exp,2))); /*Q3 */
        tmp1 = mult_r(164, pitch);/*Q7 */
        tmp = s_min(s_max(tmp1, 16), 256);/*Q7 */
        L_tmp = L_shr( L_mult0(WB_fenv[0], tmp),7);/*Q3 */
        /*WB_fenv[0] = saturate(L_tmp); //Q3 */
        WB_fenv[0] = round_fx(L_shl(L_tmp,16)); /*Q3 */
    }
    test();
    IF( L_sub(last_core_brate,ACELP_8k00) > 0 && sub(WB_fenv[0],last_wb_bwe_ener) > 0 )
    {
        /*WB_fenv[0]= add(mult_r(29491, last_wb_bwe_ener), mult_r(3277, WB_fenv[0]));//Q3 */
        WB_fenv[0] = round_fx(L_mac(L_mult(29491, last_wb_bwe_ener), 3277, WB_fenv[0]));/*Q3 */
    }

    IF(env_var_flag == 1)
    {
        WB_fenv[1] = add(WB_fenv[0], mult_r(WB_fenv[0], 16384));
        move16();
        WB_fenv[0] = mult_r(24576, WB_fenv[0]);
        move16();/*Q3                 */
    }
    ELSE
    {
        WB_fenv[1] = WB_fenv[0];
        move16();
    }

    test();
    IF(sub(coder_type,UNVOICED) == 0 || sub(prev_coder_type,UNVOICED) == 0)
    {
        WB_fenv[1] = shr(WB_fenv[1], 1);
        move16();/*Q3 */
    }

    return (mode);
}

/*-------------------------------------------------------------------*
* calc_norm_envelop_lf_fx()
*
* calc_envelope of low frequency spectrum
*-------------------------------------------------------------------*/
static
void calc_norm_envelop_lf_fx(
    const Word32 SWB_signal[],        /* i  : SWB spectrum                                    */ /* Q12 */
    Word32 *envelope,           /* o  : normalized envelope                             */ /* Q12 */
    Word16 *L_swb_norm,         /* i/o : length of envelope                              */
    const Word16 HQ_mode,             /* i  : HQ mode                                         */ /* Q0 */
    const Word16 hq_generic_offset,   /* i  : frequency offset for representing hq generic    */ /* Q0 */
    Word16 *sfreq,              /* i  : starting frequency index                        */
    Word16 *efreq               /* i  : ending frequency index                          */
)
{
    Word16 lookback, env_index, n_freq, n_lag_now, n_lag;

    *sfreq = 2;
    move16();
    IF ( sub(hq_generic_offset, HQ_GENERIC_FOFFSET_24K4)  ==  0 )
    {
        *efreq = 146;
        move16();
        if ( sub(HQ_mode, HQ_GEN_FB) == 0 )
        {
            *efreq = 306;
            move16();
        }
        IF ( sub(add(shl(sub(328,*efreq),1),1),*L_swb_norm) < 0)
        {
            *L_swb_norm = add(shl(sub(328,*efreq),1),1);
        }
    }
    ELSE
    {
        *efreq = 130;
        move16();
        if ( sub(HQ_mode, HQ_GEN_FB) == 0 )
        {
            *efreq = 290;
            move16();
        }
        IF ( sub(add(shl(sub(400,*efreq),1),1),*L_swb_norm) < 0)
        {
            *L_swb_norm = add(shl(sub(400,*efreq),1),1);
        }
    }
    lookback = shr(*L_swb_norm,1);
    env_index = 0;
    move16();
    n_lag_now = *L_swb_norm;
    move16();

    FOR (n_freq = 0; n_freq < lookback; n_freq++)
    {
        Word16 tmp;
        Word32 L_tmp;

        L_tmp = L_deposit_l(1);
        tmp = add(lookback,n_freq);
        FOR (n_lag=0; n_lag<tmp; n_lag++)
        {
            L_tmp = L_add( L_tmp, abs(SWB_signal[n_lag]) );
        }
        envelope[env_index] = L_tmp;
        move32();
        env_index = add(env_index,1);
    }

    FOR (; n_freq < *efreq; n_freq++)
    {
        /* Apply MA filter */
        Word32 L_tmp = L_deposit_l(1);
        FOR (n_lag=0; n_lag < n_lag_now; n_lag++)
        {
            L_tmp = L_add( L_tmp, abs(SWB_signal[add(sub(n_freq,lookback),n_lag)]) );
        }
        envelope[env_index] = L_tmp;
        move32();
        env_index = add(env_index,1);
    }

    return;
}

void calc_normal_length_fx(
    const Word16 core,             /* i  : core                   */
    const Word16 *sp,              /* i  : input signal           */
    const Word16 mode,             /* i  : input mode             */
    const Word16 extl,             /* i  : extension layer        */
    Word16 *L_swb_norm,      /* o  : normalize length       */
    Word16 *prev_L_swb_norm, /*i/o : last normalize length  */
    Word16 Q_syn
)
{
    Word16 i, n_freq, n_band, THRES;
    const Word16 *pit;
    Word16 peak,  mag;
    Word16 L_swb_norm_trans, L_swb_norm_norm, L_swb_norm_harm, L_swb_norm_cur;
    Word16 N;
    Word32 L_mean ,L_tmp,L_tmp1;

    THRES = 4;
    move16();
    test();
    test();
    if( sub(core,HQ_CORE) == 0 || sub(extl,SWB_BWE) == 0 || sub(extl, FB_BWE) == 0 )
    {
        THRES = 8;
        move16();
    }

    N = 16;
    move16();
    test();
    test();
    if( sub(core,HQ_CORE) == 0 && (sub(mode,HQ_HARMONIC) == 0 || sub(mode,HQ_HVQ) == 0) )
    {
        N = 13;
        move16();
    }

    n_band = 0;
    move16();
    pit = sp;
    move16();/*Q_syn */
    FOR(i = 0; i < N; i ++)
    {
        peak = 0;
        move16();
        L_mean = L_deposit_l(0);

        FOR(n_freq = 0; n_freq < 16; n_freq ++)
        {
            mag = abs_s(*pit);

            peak = s_max(peak,mag);
            L_mean = L_add(L_mean,mag);/*Q_syn */
            pit ++;
        }

        L_tmp = L_mult0(peak,15+THRES);/*Q_syn */
        IF ( sub(THRES,8) == 0 )
        {
            L_tmp1 = L_shl(Mult_32_16(L_mean,32767), 3);
        }
        ELSE
        {
            L_tmp1 = L_shl(Mult_32_16(L_mean,32767), 2);
        }

        test();
        if(  L_sub(L_tmp,L_tmp1)>0 && (sub(peak,shl(10,Q_syn)) > 0))
        {
            n_band = add(1,n_band);
        }
    }

    IF( sub(core, ACELP_CORE) == 0 )
    {
        L_swb_norm_trans = add(4, mult(n_band, 8192));
        L_swb_norm_norm = add(8, mult(n_band, 16384));

        L_tmp = L_add(65536, L_mult0(n_band, 4096)); /*Q16 */
        L_swb_norm_harm = s_max(round_fx(L_shl(L_tmp, 5)), 24); /* Q0 */

        IF( sub(mode,HARMONIC) == 0 )
        {
            L_swb_norm_cur = L_swb_norm_harm;
            move16();
        }
        ELSE IF( sub(mode,NORMAL) == 0 )
        {
            L_swb_norm_cur = L_swb_norm_norm;
            move16();
        }
        ELSE
        {
            L_swb_norm_cur = L_swb_norm_trans;
            move16();
        }
        *L_swb_norm = shr(add(L_swb_norm_cur,*prev_L_swb_norm),1);
        move16();
        *prev_L_swb_norm = L_swb_norm_cur;
        move16();
    }
    ELSE
    {
        test();
        IF( sub(mode,HQ_HARMONIC) == 0 || sub(mode,HQ_HVQ) == 0 )
        {
            L_tmp = L_add(65536, L_mult(n_band, 2560));
            L_swb_norm_cur = round_fx(L_shl(L_tmp, 5));/*Q0 */
        }
        ELSE
        {
            L_tmp = L_add(65536, L_mult(n_band, 2048)); /*Q16 */
            L_swb_norm_cur = round_fx(L_shl(L_tmp, 3));/*Q0 */
        }

        /**L_swb_norm = add(mult_r(L_swb_norm_cur, 3277), mult_r(*prev_L_swb_norm, 29491)); */
        *L_swb_norm = round_fx(L_mac(L_mult(L_swb_norm_cur, 3277), *prev_L_swb_norm, 29491));
        *prev_L_swb_norm = L_swb_norm_cur;
        move16();
    }

    return;
}
Word32 calc_tilt_bwe_fx(          /* o  : Tilt in Q24       */
    const Word16 *sp,       /* i  : input signal      */
    const Word16 exp_sp,    /* i  : Exp of inp signal */
    const Word16 N          /* i  : signal length     */
)
{
    Word16 i, j;
    Word32 L_ener, L_ener_tot, L_temp;
    Word16 tmp1, tmp2;
    const Word16 *ptr;
    Word16 exp2;

    BASOP_SATURATE_WARNING_OFF

    /* this is required for adaptative precision energy summation loop, do not remove */
    Overflow = 0;
    move16();
    exp2 = 0;
    move16();

    ptr = sp;
    move16();
    L_ener_tot = L_deposit_l(1);

    /* Divide Frame Length by 32 */
    FOR (j = shr(N, 5); j > 0; j--)
    {
        tmp1 = mult_r(*ptr++, 8192); /* Divide by 4 */
        L_ener = L_mult0(tmp1, tmp1);
        /* With the shift by 4 and the L_mult0, no overflow possible for 32 samples */
        FOR (i = 1; i < 32; i++)
        {
            tmp1 = mult_r(*ptr++, 8192); /* Divide by 4 */
            L_ener = L_mac0(L_ener, tmp1, tmp1);
        }
        L_ener = L_shr(L_ener, exp2);
        L_temp = L_add(L_ener_tot, L_ener);
        IF (Overflow != 0)
        {
            L_ener_tot = L_shr(L_ener_tot, 1);
            L_ener = L_shr(L_ener, 1);
            exp2 = add(exp2, 1);
            /* this is required, do not remove */
            Overflow = 0;
            move16();
        }
        L_ener_tot = L_add(L_ener_tot, L_ener);
    }
    L_ener = L_deposit_l(abs_s(sub(sp[1], sp[0])));
    FOR (i = 2; i < N; i++)
    {
        /* Eq to (sp[i] - sp[i-1]) * (sp[i-1] - sp[i-2]) < 0 */
        tmp1 = sub(sp[i], sp[i-1]);
        tmp2 = sub(sp[i-1], sp[i-2]);
        tmp2 = mult(tmp1, tmp2);
        tmp1 = abs_s(tmp1);
        /* to Get either 0 or -1 in 'tmp2' */
        tmp2 = shr(tmp2, 15);
        /* this allows this code */
        L_ener = L_msu0(L_ener, tmp2, tmp1);
        /* instead of this one */
        /* test(); */
        /* if (tmp2 < 0) */
        /* { */
        /*     L_ener = L_mac0(L_ener, 1, tmp1); */
        /* } */
        /* it saves one op */
    }

    tmp1 = norm_l(L_ener_tot);
    L_temp = L_shl(L_ener_tot, tmp1);
    tmp1 = sub(add(31+4, exp2), add(tmp1, shl(exp_sp, 1)));
    L_temp = Isqrt_lc(L_temp, &tmp1);

    /* *tilt_flt = (float)(r1/sqrt(r0)); */
    exp2 = norm_l(L_ener);
    L_temp = Mult_32_16(L_temp, round_fx(L_shl(L_ener, exp2)));
    exp2 = sub(exp2, tmp1);
    exp2 = add(exp2, exp_sp);

    /* Put in Q24 */
    L_temp = L_shr(L_temp, sub(exp2, 24));

    BASOP_SATURATE_WARNING_ON

    return L_temp;
}
void calc_norm_envelop_fx(
    const Word16 SWB_signal[],    /* i  : SWB spectrum            Q_syn*/
    Word32 *envelope,       /* o  : normalized envelope     Q_syn*/
    const Word16 L_swb_norm,      /* i  : length of envelope     Q0    */
    const Word16 SWB_flength,     /* i  : Length of input/output       */
    const Word16 st_offset        /* i  : offset                       */
)
{
    Word16 i, lookback, env_index, n_freq, n_lag_now, n_lag, tmp;

    lookback = shr(L_swb_norm, 1);
    move16();
    env_index = add(swb_bwe_subband_fx[0], st_offset);
    n_lag_now = L_swb_norm;
    move16();
    tmp = sub(add(SWB_flength, st_offset), L_swb_norm);
    FOR (n_freq = sub(add(swb_bwe_trans_subband_fx[0], st_offset), lookback); n_freq<tmp; n_freq++)
    {
        /* Apply MA filter */
        Word32 L_tmp = L_deposit_l(0);
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_add(L_tmp, abs_s(SWB_signal[n_freq+n_lag]));
        }
        envelope[env_index] = L_tmp;
        move32();
        env_index++;
    }

    i = 0;
    move16();
    tmp = sub(add(SWB_flength, st_offset), lookback);
    FOR( n_freq = sub(add(SWB_flength,st_offset), L_swb_norm); n_freq<tmp; n_freq++)
    {
        Word32 L_tmp;

        n_lag_now = sub(L_swb_norm, i);
        /* Apply MA filter */
        L_tmp = L_deposit_l(0);
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_add(L_tmp, abs_s(SWB_signal[n_freq+n_lag]));
        }
        envelope[env_index] = L_tmp;
        move32();
        env_index++;
        i++;
    }

    return;
}

/*==========================================================================*/
/* FUNCTION      : void WB_BWE_decoding_fx ()                               */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : WB BWE decoder                                           */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/* _Word16 *core_dec_freq     i  : Frequency domain core decoded signal Q_syn*/
/* _Word16 *WB_fenv       i  : WB frequency envelopes   Q3                  */
/* _Word16 WB_flength         i  : Length of input/output                   */
/* _Word16 mode               i  : classification for WB signal             */
/* _Word16 prev_mode          i  : classification for last frame            */
/* _Word16 last_extl          i  : extl. layer for last frame               */
/* _Word16 extl               i  : extension layer                          */
/* _Word16 coder_type         i  : coding type                              */
/* _Word16 total_brate        i  : core layer bitrate                       */
/* _Word16 prev_coder_type    i  : coding type of last frame                */
/* _Word16 Q_syn        i  : Q format                                       */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/* _Word16 *WB_signal,         o  : WB signal in MDCT domain          Q_syn */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                         */
/* _Word32 *prev_Energy,       i/o: energy for last frame           Q_syn   */
/* _Word16 *prev_WB_fenv,      i/o: envelope for last frame         Q3      */
/* _Word16 *prev_L_wb_norm,    i/o: length for last frame wb norm   Q0      */
/* _Word16 *Seed,              i/o: random generator seed           Q15     */
/* _Word16 *prev_flag,         i/o: attenu flag of last frame        Q0     */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                           */
/*           _ None                         */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                              */
/*==========================================================================*/
void WB_BWE_decoding_fx(
    const Word16 *core_dec_freq,    /* i  : Frequency domain core decoded signal */
    Word16 *WB_fenv,          /* i  : WB frequency envelopes               */
    Word32 *WB_signal_32,      /* o  : WB signal in MDCT domain             */
    const Word16 WB_flength,        /* i  : Length of input/output               */
    const Word16 mode,              /* i  : classification for WB signal         */
    const Word16 last_extl,         /* i  : extl. layer for last frame           */
    Word32 *prev_Energy,      /* i/o: energy for last frame                */
    Word16 *prev_WB_fenv,     /* i/o: envelope for last frame              */
    Word16 *prev_L_wb_norm,   /* i/o: length for last frame wb norm        */
    const Word16 extl,              /* i  : extension layer                      */
    const Word16 coder_type,        /* i  : coding type                          */
    const Word32 total_brate,       /* i  : core layer bitrate                   */
    Word16 *Seed,             /* i/o: random generator seed                */
    Word16 *prev_flag,        /* i/o: attenu flag of last frame            */
    Word16 prev_coder_type ,  /* i  : coding type of last frame            */
    Word16 Q_syn,
    Word16 *Q_syn_hb          /*o   : Q value of WB_signal_32              */
)
{
    Word16 n_freq, n_band;
    Word16 i, L;
    Word32 envelope[L_FRAME16k];
    Word32 energy, EnergyL;
    Word32 *pit1;
    Word16 WB_signal[L_FRAME16k];
    Word16 L_wb_norm ,wfenv[2];
    Word16 alfa, beta;
    Word16 flag = 0;
    Word16 core_type = 1;
    Word16 tmp,tmp1,exp,tmp2;
    Word32 L_tmp,L_tmp1,L_tmp2;
    Word32 prev_ener_alpha,prev_ener_beta;
    Word16 signum[L_FRAME16k];
    Word16 inv_L_wb_norm, weight;

    calc_normal_length_fx( ACELP_CORE, core_dec_freq, mode, extl, &L_wb_norm, prev_L_wb_norm ,Q_syn);
    set32_fx( WB_signal_32, 0, L_FRAME16k );
    set16_fx( WB_signal, 0, L_FRAME16k );

    /* copy excitation */
    test();
    if( sub(coder_type,AUDIO) != 0 && L_sub(total_brate,ACELP_8k00) <= 0 )
    {
        core_type = 0;
        move16();
    }

    IF( core_type == 0 )
    {
        Copy(&core_dec_freq[160], &WB_signal[240], 80);
    }
    ELSE
    {
        Copy(&core_dec_freq[80], &WB_signal[240], 80);
    }

    /* calculate envelope */
    calc_norm_envelop_fx(WB_signal, envelope, L_wb_norm, WB_flength, 0);
    test();
    IF( coder_type != UNVOICED && total_brate <= ACELP_8k00 )
    {
        exp = norm_s(L_wb_norm);
        inv_L_wb_norm = shl(div_s(shl(1,sub(14,exp)),L_wb_norm),sub(exp,14)); /* Q(15) */
        IF(sub(mode,HARMONIC) != 0)
        {
            tmp = add(shl(inv_L_wb_norm,1),inv_L_wb_norm) ;
            weight = s_max(s_min(tmp,16384),8192);
        }
        ELSE
        {
            weight = 8192;
            move16();
        }
        FOR(n_freq = swb_bwe_subband_fx[0]; n_freq<swb_bwe_subband_fx[4]; n_freq++)
        {
            signum[n_freq] = 1;
            IF (WB_signal[n_freq]<0)
            {
                signum[n_freq] = -1;
                move16();
                WB_signal[n_freq] = negate(WB_signal[n_freq]);
                move16();
            }
            L_tmp = Mult_32_16(envelope[n_freq],inv_L_wb_norm); /* Q_syn */
            L_tmp = Mult_32_16(L_tmp,14746); /* Q_syn */
            L_tmp1 = L_deposit_l(WB_signal[n_freq]); /* Q_syn */
            L_tmp = L_sub(L_tmp1,L_tmp); /* Q_syn */
            IF(L_tmp > 0)
            {
                tmp = sub(18022, weight); /* Q15 */
                WB_signal[n_freq] = extract_l(Mult_32_16(L_tmp,tmp)); /* Q_syn */
            }
            if(sub(signum[n_freq],1) != 0)
            {
                WB_signal[n_freq] = negate(WB_signal[n_freq]);
                move16();
            }
        }
    }

    /* Normalize with envelope */
    FOR (n_freq = swb_bwe_subband_fx[0]; n_freq<swb_bwe_subband_fx[4]; n_freq++)
    {
        IF(envelope[n_freq] != 0)
        {
            exp = norm_l(envelope[n_freq]);
            tmp = extract_h(L_shl(envelope[n_freq],exp));
            exp = sub(sub(30,exp),Q_syn);
            tmp = div_s(16384,tmp);
            L_tmp = L_shr(L_mult0(WB_signal[n_freq],tmp),add(exp,Q_syn)); /*Q15 */
            WB_signal[n_freq] = extract_l(L_tmp); /*Q15 */
        }
        ELSE
        {
            WB_signal[n_freq] = shl(WB_signal[n_freq],sub(15,Q_syn)); /*Q15 */  move16();
        }
    }

    L = 1;
    move16();
    if( sub(mode,HARMONIC) == 0 )
    {
        L = 4;
        move16();
    }

    IF( sub(coder_type,UNVOICED) == 0 )
    {
        FOR ( n_freq = swb_bwe_subband_fx[0]; n_freq < swb_bwe_subband_fx[4]; n_freq++ )
        {
            *Seed = extract_l(L_mac0(20101L, *Seed, 12345));
            L_tmp = L_shl(*Seed, add(Q_syn, 1));
            WB_signal_32[n_freq] = L_tmp;
            move32();
        }
    }
    ELSE
    {
        FOR( n_band = 0; n_band < 4; n_band += L )
        {
            energy = L_deposit_l(0);
            FOR (n_freq = swb_bwe_subband_fx[n_band]; n_freq<swb_bwe_subband_fx[n_band+L]; n_freq++)
            {
                L_tmp = L_mult(WB_signal[n_freq],WB_signal[n_freq]); /*Q31 */
                energy = L_add(energy,L_shr(L_tmp,6)); /*Q25 */
            }

            tmp = sub(swb_bwe_subband_fx[n_band+L] , swb_bwe_subband_fx[n_band]);
            tmp = div_s(1,tmp);/*Q15 */
            energy = Mult_32_16(energy,tmp); /*Q(15+25-15)->Q(25) */

            exp = norm_l(energy);
            L_tmp1 = L_shl(energy, exp);
            exp = 31-exp-(25);
            move16();
            L_tmp1 = Isqrt_lc(L_tmp1, &exp); /*Q(31-exp) */

            FOR (n_freq = swb_bwe_subband_fx[n_band]; n_freq<swb_bwe_subband_fx[n_band+L]; n_freq++)
            {
                L_tmp2 = Mult_32_16(L_tmp1, WB_signal[n_freq]);
                WB_signal_32[n_freq] = L_shl(L_tmp2, sub(add(exp, Q_syn), 15));
                move32();
            }
        }
    }

    EnergyL = L_deposit_l(0);
    IF( core_type == 1 )
    {
        test();
        IF( sub(prev_coder_type,AUDIO) != 0 && L_sub(total_brate,ACELP_8k00) <= 0 )
        {
            FOR(i=160; i<240; i++)
            {
                EnergyL = L_add(abs_s(core_dec_freq[i]),EnergyL);
            }
        }
        ELSE
        {
            FOR(i=80; i<240; i++)
            {
                EnergyL = L_add(abs_s(core_dec_freq[i]),EnergyL);
            }
        }

        IF(L_sub(total_brate,ACELP_8k00) <= 0)
        {
            alfa = 26214;
            move16(); /*0.8f in Q15; */
            beta = 10240;
            move16();/*1.25f in Q13; */
        }
        ELSE
        {
            alfa =16384;
            move16();/* 0.5f in Q15; */
            beta = 16384;
            move16();/*2.0f in Q13 */
        }
    }
    ELSE
    {
        IF( sub(prev_coder_type,AUDIO) == 0 )
        {
            FOR(i=80; i<240; i++)
            {
                EnergyL = L_add(abs_s(core_dec_freq[i]), EnergyL);/*Q_syn */
            }
        }
        ELSE
        {
            FOR(i=160; i<240; i++)
            {
                EnergyL = L_add(abs_s(core_dec_freq[i]), EnergyL);
            }
        }

        test();
        IF( sub(prev_coder_type,coder_type) == 0 && sub(WB_fenv[0],prev_WB_fenv[0]) > 0 )
        {
            alfa = 13107;
            move16();/*.4 in Q15 */
            beta = 20480;
            move16(); /*2.5 in Q13 */
        }
        ELSE
        {
            alfa = 19661;
            move16();/*.6 in Q15 */
            beta = 13681;
            move16();/*1.67 in Q13 */
        }

        test();
        test();
        test();
        IF( sub(coder_type,GENERIC) == 0 || ((L_sub(EnergyL,L_shr(*prev_Energy,1)) > 0 && L_sub(*prev_Energy,L_shr( EnergyL, 1))>0 && sub(*prev_flag,1) == 0)) )
        {
            WB_fenv[0] = shr( WB_fenv[0], 1);
            move16();
            WB_fenv[1] = shr( WB_fenv[1], 1);
            move16();
            flag = 1;
            move16();
        }
    }
    L_tmp1 = Mult_32_16(EnergyL,prev_WB_fenv[0]);/*Qsyn+3-15 */
    L_tmp2 = Mult_32_16(*prev_Energy,WB_fenv[0]);/*Q_syn+3-15 */
    prev_ener_alpha = Mult_32_16( *prev_Energy, alfa);/*Q_syn+15-15->Q_syn */
    prev_ener_beta = L_shl( Mult_32_16( *prev_Energy ,beta),2); /*Q_syn+13-15+2 ->Q_syn */

    test();
    test();
    IF( (sub(mode,HARMONIC) == 0 &&sub(shr(WB_fenv[0], 2), WB_fenv[1]) > 0) || sub(mode,NORMAL) == 0 )
    {
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
        IF( sub(last_extl,WB_BWE) == 0 &&
            ( (sub(prev_coder_type,AUDIO) == 0 && sub(coder_type,AUDIO) != 0) ||
              (sub(prev_coder_type,AUDIO) != 0 && sub(coder_type,AUDIO) == 0)) && L_sub(total_brate,ACELP_8k00) <= 0 )
        {
            IF( sub(WB_fenv[0],prev_WB_fenv[0]) > 0 )
            {
                /*wfenv[0]= add(mult_r(9830, WB_fenv[0]), mult_r(22938, prev_WB_fenv[0]));//Q3 */
                wfenv[0]= round_fx(L_mac(L_mult(9830, WB_fenv[0]), 22938, prev_WB_fenv[0]));/*Q3 */
                /*wfenv[1]= add(mult_r(9830, WB_fenv[1]), mult_r(22938, prev_WB_fenv[1]));//Q3 */
                wfenv[1]= round_fx(L_mac(L_mult(9830, WB_fenv[1]), 22938, prev_WB_fenv[1]));/*Q3 */
            }
            ELSE
            {
                /*wfenv[0]= add(mult_r(16384,WB_fenv[0]),mult_r(16384,prev_WB_fenv[0]));//Q3 */
                wfenv[0]= round_fx(L_mac(L_mult(16384,WB_fenv[0]),16384,prev_WB_fenv[0]));/*Q3 */
                /*wfenv[1]= add(mult_r(13108,WB_fenv[1]),mult_r(13108,prev_WB_fenv[1]));//Q3 */
                wfenv[1]= round_fx(L_mac(L_mult(13108,WB_fenv[1]),13108,prev_WB_fenv[1]));/*Q3 */
            }
        }
        ELSE IF ( sub(last_extl,WB_BWE) == 0 && L_sub(L_tmp1,L_tmp2)<0 && sub(WB_fenv[0],prev_WB_fenv[0]) > 0&&
                  sub(coder_type,AUDIO) != 0 && sub(coder_type,UNVOICED) != 0 && L_sub(total_brate,ACELP_8k00) <= 0)
        {
            /*wfenv[0]= add(mult_r(9830,WB_fenv[0]),mult_r(22938,prev_WB_fenv[0]));//Q3 */
            wfenv[0]= round_fx(L_mac(L_mult(9830,WB_fenv[0]),22938,prev_WB_fenv[0]));/*Q3 */
            /*wfenv[1]= add(mult_r(9830,WB_fenv[1]),mult_r(22938,prev_WB_fenv[1]));//Q3 */
            wfenv[1]= round_fx(L_mac(L_mult(9830,WB_fenv[1]),22938,prev_WB_fenv[1]));/*Q3 */
        }
        ELSE IF ( sub(last_extl,WB_BWE) == 0 && L_sub(EnergyL,prev_ener_alpha) >0 && L_sub(prev_ener_beta,EnergyL) >0 &&
                  sub(prev_coder_type,UNVOICED) != 0 )
        {
            /*wfenv[0] = add(shr(WB_fenv[0],1), shr(prev_WB_fenv[0],1));//Q3 */
            wfenv[0] = round_fx(L_mac(L_mult(WB_fenv[0],16384), prev_WB_fenv[0],16384));/*Q3 */
            /*wfenv[1] = add(shr(WB_fenv[1],1), shr(prev_WB_fenv[1],1));//Q3 */
            wfenv[1] = round_fx(L_mac(L_mult(WB_fenv[1],16384), prev_WB_fenv[1],16384));/*Q3 */
        }
        ELSE
        {
            wfenv[0] = WB_fenv[0];
            move16();
            wfenv[1] = WB_fenv[1];
            move16();
        }
        FOR (n_freq = swb_bwe_subband_fx[0]; n_freq<swb_bwe_subband_fx[2]; n_freq++)
        {
            WB_signal_32[n_freq] = Mult_32_16(WB_signal_32[n_freq], wfenv[0]);
            move32();/* Q_syn+3+1 */
        }

        FOR (n_freq = swb_bwe_subband_fx[2]; n_freq<swb_bwe_subband_fx[4]; n_freq++)
        {
            WB_signal_32[n_freq] = Mult_32_16(WB_signal_32[n_freq], wfenv[1]);
            move32();/* Q_syn+3+1 */
        }

        prev_WB_fenv[0] = wfenv[0];
        move16();
        prev_WB_fenv[1] = wfenv[1];
        move16();
    }
    ELSE
    {
        wfenv[0] = add(shr(WB_fenv[0],1), shr(WB_fenv[1],1));/*Q3 */

        test();
        test();
        IF(sub(last_extl,WB_BWE) == 0 && L_sub(EnergyL,L_shr(*prev_Energy,1)) > 0 && L_sub(*prev_Energy,L_shr(EnergyL,1))>0)
        {
            L_tmp1 = L_mac(L_mult(8192,wfenv[0]),12288, prev_WB_fenv[0]);
            wfenv[0] = round_fx(L_mac(L_tmp1, 12288, prev_WB_fenv[1]));
        }
        FOR (n_freq = swb_bwe_subband_fx[0]; n_freq<swb_bwe_subband_fx[4]; n_freq++)
        {
            WB_signal_32[n_freq] = Mult_32_16(WB_signal_32[n_freq], wfenv[0]);
            move32();/* Q_syn+3+1 */
        }
        prev_WB_fenv[0] = wfenv[0];
        move16();
        prev_WB_fenv[1] = wfenv[0];
        move16();
    }

    *prev_flag = flag;
    move16();
    *prev_Energy = EnergyL;
    move32();
    pit1 = &WB_signal_32[240];

    FOR(n_freq=0; n_freq<16; n_freq++)
    {
        tmp1 = extract_l(L_mult0(n_freq, 1638));/*Q15 */
        tmp2 = add(6554, tmp1);/*Q15 */
        L_tmp1 = Mult_32_16(*pit1, tmp2);    /*Q_syn+3+1 */
        *(pit1++) = L_tmp1;
        move32();
    }

    IF( sub(core_type,1) == 0 )
    {
        pit1 = &WB_signal_32[280];
        FOR(n_freq=0; n_freq<40; n_freq++)
        {
            tmp1 = extract_l(L_mult0(n_freq,655));/*Q15 */
            tmp2 = sub(32767,tmp1);
            L_tmp1 = Mult_32_16(*pit1, tmp2);    /*Q_syn+3+1 */
            *(pit1++) = L_tmp1;
            move32();
        }
    }
    ELSE
    {
        pit1 = &WB_signal_32[300];
        FOR(n_freq=0; n_freq<20; n_freq++)
        {
            tmp1 = extract_l(L_mult0(n_freq, 1311));/*Q15 */
            tmp2 = sub(32767, tmp1);
            L_tmp1 = Mult_32_16(*pit1, tmp2);    /*Q_syn+3+1 */
            *(pit1++) = L_tmp1;
            move32();
        }
    }
    pit1 = &WB_signal_32[240];
    tmp = Find_Max_Norm32(pit1, 80);
    FOR (i = 0; i < 80; i++)
    {
        L_tmp = *pit1;
        *(pit1++) = L_shl(L_tmp,sub(tmp,1));
        move32();
    }
    *Q_syn_hb= add(Q_syn,add(tmp,3));
    return;
}


/*==========================================================================*/
/* FUNCTION      : void SWB_BWE_decoding_fx()                               */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :    SWB BWE decoder                                       */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS                                                          */
/* _(Word16*) core_dec_freq     :Frequency domain core decoded signal Q_syn */
/* _(Word16) SWB_flength    :Length of input/output         Q0              */
/* _(Word16) mode         :classification for SWB signal                    */
/* _(Word16) tilt_nb      :tilt of synthesis wb signal      Q11             */
/* _(Word16) st_offset      :offset value due to different core             */
/* _(Word16) Q_syn          :Q format                                       */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*  _(Word16*)SWB_fenv        : SWB frequency envelopes         Q3          */
/*  _(Word16*)prev_Energy     : energy for last frame           Q3          */
/*  _(Word16*)prev_SWB_fenv     : envelope for last frame       Q3          */
/*  _(Word16*)prev_L_swb_norm : length for last frame wb norm   Q0          */
/*  _(Word16*)Seed        : random generator seed           Q0              */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*  _(Word16*)SWB_signal    : SWB signal in MDCT domain       Q0            */
/*  _(Word16*)frica_flag    : fricative signal flag           Q0            */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*   _ None                                                                 */
/*--------------------------------------------------------------------------*/

void SWB_BWE_decoding_fx(
    const Word16 *core_dec_freq,    /* i  : Frequency domain core decoded signal  */
    Word16 *SWB_fenv,         /* i/o: SWB frequency envelopes               */
    Word32 *SWB_signal_32,    /* o  : SWB signal in MDCT domain             */
    const Word16 SWB_flength,       /* i  : Length of input/output                */
    const Word16 mode,              /* i  : classification for SWB signal         */
    Word16 *frica_flag,       /* o  : fricative signal flag                 */
    Word16 *prev_Energy,      /* i/o: energy for last frame                 */
    Word16 *prev_SWB_fenv,    /* i/o: envelope for last frame               */
    Word16 *prev_L_swb_norm,  /* i/o: length for last frame wb norm         */
    const Word16 tilt_nb,           /* i  : tilt of synthesis wb signal           */
    Word16 *Seed,             /* i/o: random generator seed                 */
    const Word16 st_offset,         /* i  : offset value due to different core    */
    Word16 *prev_weight,      /* i/o: excitation weight value of last frame */
    const Word16 extl ,             /* i  : extension layer                       */
    Word16 Q_syn
)
{
    Word16 n_freq, n_band, L, L_swb_norm;
    Word32 *pit1_32;
    Word16 *pit1;
    Word32 envelope[L_FRAME32k];
    Word32 fenvL, EnergyL, Energy, energy,   L_mean;
    Word16 fenvL_16, EnergyL_16, Energy_16, tmp, exp, exp1;
    Word16 SWB_signal[L_FRAME32k];
    Word16 mean,factor, factor1, tmp1, tmp2, tmp3, tmp_exp, tmp_ener, weight, wfenv;
    Word32 L_tmp, L_tmp3, L_tmp4, Ltmp_ener, L_tmp1;
    Word32 L_energy;
    Word16 signum[L_FRAME32k];
    Word16 inv_L_swb_norm;

    fenvL = L_deposit_l(0);
    EnergyL = L_deposit_l(0);
    FOR(n_freq=224+st_offset; n_freq<swb_bwe_trans_subband_fx[0]+st_offset; n_freq++)
    {
        fenvL = L_mac0(fenvL,core_dec_freq[n_freq],core_dec_freq[n_freq]); /*2*Q_syn */
    }

    FOR( n_freq = 16; n_freq < L_FRAME; n_freq++ )
    {
        EnergyL = L_mac0(EnergyL,core_dec_freq[n_freq],core_dec_freq[n_freq]); /*2*Q_syn */
    }

    fenvL_16 = 0;
    move16();
    IF (fenvL != 0)
    {
        exp = norm_l(fenvL);                            /* In 2*Q_syn */
        tmp = extract_h(L_shl(fenvL, exp));
        exp = sub(exp, sub(30,add(shl(Q_syn,1),4))); /*+4(/16) */

        tmp = div_s(16384, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp);

        fenvL_16 = round_fx(L_shl(L_tmp, sub(exp, 12))); /* Q3 */
    }

    EnergyL = Mult_32_16(EnergyL,17476); /*2*Q_syn+3; 17476=(1/15) in Q18 */
    EnergyL_16 = 0;
    move16();
    IF (EnergyL != 0)
    {
        exp = norm_l(EnergyL);                            /* In 2*Q_syn+3 */
        tmp = extract_h(L_shl(EnergyL, exp));
        exp = sub(exp, sub(30,add(shl(Q_syn,1),3+4))); /*+4(/16) */

        tmp = div_s(16384, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp);

        EnergyL_16 = round_fx(L_shl(L_tmp, sub(exp, 12))); /* Q3 */
    }
    calc_normal_length_fx( ACELP_CORE, core_dec_freq, mode, extl, &L_swb_norm, prev_L_swb_norm ,Q_syn);

    set16_fx( SWB_signal, 0, L_FRAME32k );
    IF( sub(mode,TRANSIENT) == 0 )
    {
        Energy = L_deposit_l(0);
        FOR(n_band = 0; n_band < SWB_FENV_TRANS; n_band++)
        {
            Energy = L_mac(Energy,SWB_fenv[n_band],SWB_fenv[n_band]); /*Q(2*3+1)->Q7 */
        }
        exp = norm_s(SWB_FENV_TRANS);
        tmp = div_s(shl(1,sub(14,exp)), SWB_FENV_TRANS); /*Q(29-exp) */
        L_tmp = Mult_32_16(Energy, tmp); /*Q(7+29-exp+1-16)->Q(21-exp) */
        Energy_16 = round_fx(L_shl(L_tmp, sub(exp,2))); /*Q3 */

        /* Reconstruct excitation from LF signal */
        Copy(&core_dec_freq[112], &SWB_signal[240+st_offset], 128);
        Copy(&core_dec_freq[112], &SWB_signal[368+st_offset], 128);
        Copy(&core_dec_freq[176], &SWB_signal[496+st_offset], 64);

        /* calculate envelope */
        calc_norm_envelop_fx(SWB_signal, envelope, L_swb_norm, SWB_flength, st_offset);

        /* Normalize with envelope */
        tmp_exp = sub(15, Q_syn);
        FOR (n_freq = swb_bwe_trans_subband_fx[0]+st_offset; n_freq<swb_bwe_trans_subband_fx[SWB_FENV_TRANS]+st_offset; n_freq++)
        {
            IF(envelope[n_freq] != 0)
            {
                exp = norm_l(envelope[n_freq]);
                tmp = extract_h(L_shl(envelope[n_freq], exp));
                exp = sub(sub(30,exp), Q_syn);
                tmp = div_s(16384,tmp); /*Q(15+exp) */
                L_tmp = L_shr(L_mult0(SWB_signal[n_freq],tmp),add(exp,Q_syn)); /*Q15 */
                SWB_signal[n_freq] = extract_l(L_tmp); /*Q15 */
            }
            ELSE
            {
                SWB_signal[n_freq] = shl(SWB_signal[n_freq], tmp_exp);
                move16();/*Q15 */
            }
        }

        FOR(n_band=0; n_band<SWB_FENV_TRANS; n_band++)
        {
            energy = L_deposit_l(0);
            tmp = add(swb_bwe_trans_subband_fx[n_band+1],st_offset);
            FOR (n_freq = add(swb_bwe_trans_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
            {
                L_tmp = L_mult(SWB_signal[n_freq],SWB_signal[n_freq]); /*Q31 */
                energy = L_add(energy,L_shr(L_tmp,6)); /*Q25 */
            }

            IF(energy == 0)
            {
                Ltmp_ener = L_mult(sqrt_swb_bwe_trans_subband_width_fx[n_band],SWB_fenv[n_band]);/*Q13 */
                tmp = add(swb_bwe_trans_subband_fx[n_band+1],st_offset);
                FOR (n_freq = add(swb_bwe_trans_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
                {
                    SWB_signal_32[n_freq] = L_shl(Mult_32_16(Ltmp_ener,SWB_signal[n_freq]),2 + Q_syn);
                    move32();/*15+Qsyn */
                }
            }
            ELSE
            {
                exp1 = norm_s(swb_bwe_trans_subband_width_fx[n_band]);
                tmp = div_s(shl(1,sub(14,exp1)),swb_bwe_trans_subband_width_fx[n_band]); /*Q(29-exp1) */
                energy = Mult_32_16(energy,tmp); /*Q(29-exp1+25-15)->Q(-exp1+39) */

                exp = norm_l(energy);
                L_tmp = L_shl(energy,exp);
                /*exp = 31-exp-(-exp1+39); */
                exp = sub(sub(exp1,exp),8);
                L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */

                Ltmp_ener = Mult_32_16(L_tmp,SWB_fenv[n_band]);/*Q(31-exp+1+1-16)->Q(17-exp) */
                tmp = add(swb_bwe_trans_subband_fx[n_band+1],st_offset);
                tmp_exp = add(Q_syn,sub(exp,2));
                FOR (n_freq = add(swb_bwe_trans_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
                {
                    SWB_signal_32[n_freq] = L_shl(Mult_32_16(Ltmp_ener,SWB_signal[n_freq]), tmp_exp);
                    move32(); /*15+Qsyn */
                }
            }
        }

        FOR( n_band = 0; n_band < 8; n_band++ )
        {
            L_tmp = L_mult(SWB_fenv[n_band/4],SWB_fenv[n_band/4]); /*Q7 */
            prev_SWB_fenv[n_band] = round_fx(L_shl(L_tmp,12)); /*Q3 */
        }

        FOR( n_band = 0; n_band < 6; n_band++ )
        {
            L_tmp = L_mult(SWB_fenv[2+n_band/3],SWB_fenv[2+n_band/3]); /*Q7 */
            prev_SWB_fenv[8+n_band] = round_fx(L_shl(L_tmp,12)); /*Q3 */
        }

        *prev_weight = 16384;
        move16();
    }
    ELSE
    {
        Energy_16 = 0;
        move16();
        L_energy = L_deposit_l(0);
        FOR(n_band = 0; n_band < SWB_FENV; n_band++)
        {
            L_energy = L_add(L_energy,SWB_fenv[n_band]); /*Q1 */
        }
        exp = norm_s(SWB_FENV);
        tmp = div_s(shl(1,sub(14,exp)),SWB_FENV); /*Q(29-exp) */
        L_tmp = Mult_32_16(L_energy,tmp); /*Q(1+29-exp+1)->Q(15-exp) */
        Energy_16 = round_fx(L_shl(L_tmp,add(exp,4))); /*Q3 */

        IF( mode == HARMONIC )
        {
            Copy( core_dec_freq, &SWB_signal[240+st_offset], 240 );
            Copy( &core_dec_freq[128], &SWB_signal[480+st_offset], 80 );
            /* calculate envelope */
            calc_norm_envelop_fx(SWB_signal, envelope, L_swb_norm, SWB_flength, st_offset);
        }
        ELSE
        {
            test();
            test();
            test();
            test();
            test();
            test();
            IF( sub(mode, NOISE) == 0 || ((sub(Energy_16,EnergyL_16) > 0 || (sub(tilt_nb,14336) > 0 && sub(Energy_16,shr(EnergyL_16,1)) > 0) ||
            sub(tilt_nb,24576) > 0) && sub(Energy_16,600) > 0 && sub(fenvL_16,200) > 0))
            {
                tmp = add(swb_bwe_subband_fx[SWB_FENV], st_offset);
                FOR (n_freq=add(swb_bwe_subband_fx[0],st_offset); n_freq<tmp; n_freq++)
                {
                    *Seed = extract_l(L_mac0(20101L, 12345,*Seed));
                    SWB_signal[n_freq] = mult_r(*Seed, 32767);
                    move16();/*Q15 */
                }
                if( sub(mode, NOISE) != 0 )
                {
                    *frica_flag = 1;
                    move16();
                }
            }
            ELSE
            {
                /* modify SHB frequency envelopes when SHB spectrum is unflat */
                FOR(n_band=0; n_band <13; n_band++)
                {
                    IF(sub(mult_r(SWB_fenv[n_band],29491), SWB_fenv[n_band+1]) > 0)
                    {
                        tmp = extract_l(L_mac0(26214, n_band, 492)); /*Q15; 0.015 in Q15 = 492  */
                        SWB_fenv[n_band+1] = mult_r(SWB_fenv[n_band+1], tmp);
                        move16();/*Q3 */
                    }

                    IF(sub(mult_r(SWB_fenv[n_band+1],29491),SWB_fenv[n_band]) > 0)
                    {
                        tmp = extract_l(L_mac0(26214, n_band,492)); /*Q15; 0.015 in Q15 = 492  */
                        SWB_fenv[n_band] = mult_r(SWB_fenv[n_band],tmp);
                        move16();/*Q3 */
                    }
                }
                Copy(&core_dec_freq[112], &SWB_signal[240+st_offset], 128);
                Copy(&core_dec_freq[112], &SWB_signal[368+st_offset], 128);
                Copy(&core_dec_freq[176], &SWB_signal[496+st_offset], 64);

                tmp1 = add(abs_s(SWB_signal[368+st_offset]), abs_s(SWB_signal[369+st_offset])); /*Q_syn */
                tmp2 = add(abs_s(SWB_signal[365+st_offset]), abs_s(SWB_signal[366+st_offset])); /*Q_syn */
                pit1 = &SWB_signal[368+st_offset];
                move16();

                test();
                IF((tmp2 == 0) || (sub(tmp2, mult_r(tmp1, 9830)) < 0))
                {
                    tmp3 = 9830;
                    move16();/*0.3 in Q15 */
                    WHILE(sub(tmp3,32767) < 0)
                    {
                        *pit1 = mult_r(*pit1,tmp3);
                        move16(); /*Q_syn */
                        pit1++;
                        tmp3 = add(tmp3,3277); /*Q15 */
                    }
                }
                ELSE IF(sub(tmp2, tmp1) < 0)
                {
                    exp = norm_s(tmp1);
                    tmp = div_s(shl(1,sub(14,exp)),tmp1); /*Q(29-exp) */
                    tmp3 = round_fx(L_shl(L_mult(tmp2,tmp),add(exp,2))); /*Q15 */
                    WHILE(sub(tmp3, 32767) < 0)
                    {
                        *pit1 = mult_r(*pit1,tmp3);
                        move16(); /*Q_syn */
                        pit1++;
                        tmp3 = add(tmp3,3277); /*Q15 */
                    }
                }

                pit1 = &SWB_signal[367+st_offset];
                move16();
                IF(sub(mult_r(tmp1,6554),tmp2) > 0)
                {
                    /*20480 = 5 in Q12 */
                    FOR(tmp3 = 20480; tmp3 > 4096; tmp3 -= 2048)
                    {
                        *pit1 = round_fx(L_shl(L_mult(*pit1,tmp3),3)); /*Q_syn */
                        pit1--;
                    }
                }

                tmp1 = add(abs_s(SWB_signal[496+st_offset]),abs_s(SWB_signal[497+st_offset])); /*Q_syn */
                tmp2 = add(add(abs_s(SWB_signal[492+st_offset]),abs_s(SWB_signal[493+st_offset])),add(abs_s(SWB_signal[494+st_offset]),abs_s(SWB_signal[495+st_offset])));
                pit1 = &SWB_signal[496+st_offset];
                move16();

                test();
                IF((tmp2 == 0) || (sub(tmp2,mult_r(tmp1,9830)) < 0))
                {
                    tmp3 = 9830;
                    move16(); /*0.3 in Q15 */
                    WHILE(sub(tmp3,32767) < 0)
                    {
                        *pit1 = mult_r(*pit1,tmp3);
                        move16(); /*Q_syn */
                        pit1++;
                        tmp3 = add(tmp3,3277); /*Q15 */
                    }
                }
                ELSE IF(sub(tmp2,tmp1) < 0)
                {
                    exp = norm_s(tmp1);
                    tmp = div_s(shl(1,sub(14,exp)),tmp1); /*Q(29-exp) */
                    tmp3 = round_fx(L_shl(L_mult(tmp2,tmp),add(exp,2))); /*Q15 */
                    WHILE(sub(tmp3,32767) < 0)
                    {
                        *pit1 = mult_r(*pit1,tmp3);
                        move16();/*Q_syn */
                        pit1++;
                        tmp3 = add(tmp3,3277); /*Q15 */
                    }
                }
                pit1 = &SWB_signal[495+st_offset];

                L_tmp3 = L_deposit_h(tmp1); /*Q17 */
                L_tmp4 = Mult_32_16(L_tmp3,1638); /*Q17 */
                exp = 14;
                move16();
                IF(tmp2 != 0)
                {
                    exp = norm_s(tmp2);
                    tmp = div_s(shl(1,sub(14,exp)),tmp2); /*Q(29-exp) */
                    L_tmp3 = L_shr(L_mult(tmp1,tmp),1); /*Q(30-exp+1)->Q(30-exp) (+1) due to *0.5 */
                    L_tmp4 = Mult_32_16(L_tmp3,1638); /*Q(30-exp) */
                }

                WHILE(L_sub(L_tmp3,(1<<(30-exp))) > 0)
                {
                    L_tmp = Mult_32_16(L_tmp3,*pit1); /*Q(16-exp) */
                    *pit1-- = round_fx(L_shl(L_tmp,exp)); /*Q_syn */
                    L_tmp3 = L_sub(L_tmp3,L_tmp4);
                }

                /* calculate envelope */
                calc_norm_envelop_fx(SWB_signal, envelope, L_swb_norm, SWB_flength, st_offset);
            }
        }

        /* Normalize with envelope */
        test();
        IF( *frica_flag == 0 && sub(mode, NOISE) != 0 )
        {
            L = add(swb_bwe_subband_fx[0],st_offset);
            exp = norm_s(L_swb_norm);
            inv_L_swb_norm = shl(div_s(shl(1,sub(14,exp)),L_swb_norm),sub(exp,14)); /* Q15 */

            IF(sub(mode,HARMONIC) != 0)
            {
                tmp = add(shl(inv_L_swb_norm,1),inv_L_swb_norm) ;
                weight = s_max(s_min(tmp,16384),8192);
            }
            ELSE
            {
                weight = 8192;
                move16();
            }

            weight = mac_r(L_mult(13107,weight) , 19661,(*prev_weight));

            FOR (n_freq = L; n_freq<swb_bwe_subband_fx[SWB_FENV]+st_offset; n_freq++)
            {
                signum[n_freq] = 1;
                IF (SWB_signal[n_freq]<0)
                {
                    signum[n_freq] = -1;
                    move16();
                    SWB_signal[n_freq] = negate(SWB_signal[n_freq]);
                    move16();
                }
                L_tmp = Mult_32_16(envelope[n_freq],inv_L_swb_norm); /* Q_syn */
                L_tmp1 = L_deposit_l(SWB_signal[n_freq]); /* Q_syn */
                L_tmp = L_sub(L_tmp1,L_tmp); /* Q_syn */
                IF(L_tmp > 0)
                {
                    tmp = shr(weight,1); /* Q14 */
                    tmp = sub(19661,tmp); /* Q14 */
                    SWB_signal[n_freq] = extract_l(L_shl(Mult_32_16(L_tmp,tmp),1)); /* Q_syn */
                }
                IF(sub(signum[n_freq],1) != 0)
                {
                    SWB_signal[n_freq] = negate(SWB_signal[n_freq]);
                    move16();
                }
            }

            tmp_exp = sub(15,Q_syn);
            FOR (n_freq = L; n_freq<swb_bwe_subband_fx[SWB_FENV]+st_offset; n_freq++)
            {
                IF(envelope[n_freq] != 0)
                {
                    exp = norm_l(envelope[n_freq]);
                    tmp = extract_h(L_shl(envelope[n_freq], exp));
                    exp = sub(sub(30,exp), Q_syn);
                    tmp = div_s(16384,tmp); /* Q(15+exp) */
                    L_tmp = L_shr(L_mult0(SWB_signal[n_freq],tmp), add(exp,Q_syn)); /* Q15 */
                    SWB_signal[n_freq] = extract_l(L_tmp); /* Q15 */
                }
                ELSE
                {
                    SWB_signal[n_freq] = shl(SWB_signal[n_freq], tmp_exp);
                    move16(); /* Q15 */
                }
            }
            *prev_weight = weight;
        }
        ELSE
        {
            exp = norm_s(L_swb_norm);
            tmp = shl(div_s(shl(1,sub(14,exp)),L_swb_norm),sub(exp,14)); /* Q15 */
            tmp = add(shl(tmp,1),tmp);
            *prev_weight = s_max(s_min(tmp,16384),6554); /* Q15 */
        }

        IF(sub(mode,HARMONIC) == 0)
        {
            pit1 = &SWB_signal[swb_bwe_subband_fx[0]+st_offset];
            move16();
            FOR(n_band=0; n_band<19; n_band++)
            {
                L_mean = L_deposit_l(0);
                FOR(n_freq=0; n_freq<16; n_freq++)
                {
                    L_mean = L_add(L_mean,abs_s(*pit1)); /*Q15 */
                    pit1++;
                }
                mean = extract_l(Mult_32_16(L_mean,2048)); /*Q15  */
                pit1 -= 16;
                move16();
                FOR(n_freq=0; n_freq<16; n_freq++)
                {
                    if(sub(abs_s(*pit1),mean) < 0)
                    {
                        *pit1 = mult_r(*pit1,6554); /*Q15*/ move16();
                    }
                    pit1++;
                }
            }
        }

        L = 1;
        move16();
        if(sub(mode,HARMONIC) == 0)
        {
            L = 2;
            move16();
        }

        FOR(n_band=0; n_band<SWB_FENV; n_band+=L)
        {
            energy = L_deposit_l(0);
            tmp = add(swb_bwe_subband_fx[n_band+L],st_offset);
            FOR (n_freq = add(swb_bwe_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
            {
                L_tmp = L_mult(SWB_signal[n_freq],SWB_signal[n_freq]); /*Q31 */
                energy = L_add(energy,L_shr(L_tmp,6)); /*Q25 */
            }

            IF(energy == 0)
            {
                tmp_ener = sqrt_swb_bwe_subband_fx_L2[n_band];/*Q12 */
                IF(sub(L,1) == 0)
                {
                    tmp_ener = sqrt_swb_bwe_subband_fx_L1[n_band];/*Q12 */
                }

                tmp = add(swb_bwe_subband_fx[n_band+L],st_offset);
                tmp_exp = sub(Q_syn,12);
                FOR (n_freq = add(swb_bwe_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
                {
                    SWB_signal[n_freq] = round_fx(L_shl(L_mult(tmp_ener,SWB_signal[n_freq]),tmp_exp)); /*Qsyn */
                }
            }
            ELSE
            {
                tmp = sub(swb_bwe_subband_fx[n_band+L], swb_bwe_subband_fx[n_band]);
                exp1 = norm_s(tmp);
                tmp = div_s(shl(1,sub(14,exp1)), tmp); /*Q(29-exp1) */
                energy = Mult_32_16(energy, tmp); /*Q(29-exp1+25-15)->Q(-exp1+39) */

                exp = norm_l(energy);
                L_tmp = L_shl(energy, exp);
                /*exp = 31-exp-(-exp1+39);move16(); */
                exp = sub(sub(exp1,exp),8);
                Ltmp_ener = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */

                tmp = add(swb_bwe_subband_fx[n_band+L],st_offset);
                tmp_exp = add(Q_syn,sub(exp,15));
                FOR(n_freq = add(swb_bwe_subband_fx[n_band],st_offset); n_freq<tmp; n_freq++)
                {
                    SWB_signal_32[n_freq] = L_shl(Mult_32_16(Ltmp_ener,SWB_signal[n_freq]),tmp_exp);
                    move32();/*Qsyn+16 */
                }
            }
        }

        IF(sub(*prev_Energy,add(Energy_16,shr(Energy_16,2))) > 0 && Energy_16 > 0)
        {
            weight = shr(div_s(Energy_16,*prev_Energy),1); /*Q15 */
        }
        ELSE
        {
            weight = 16384;
            move16();/*Q15 */
        }
        L_tmp = L_mult(weight, prev_SWB_fenv[0]); /*Q19 */
        L_tmp = L_mac(L_tmp,sub(32767,weight), SWB_fenv[0]);/*Q19 */
        wfenv = round_fx(L_tmp); /*Q1 */

        tmp = norm_s(wfenv);
        IF ( sub(tmp,4) > 0 )
        {
            tmp = 12;
            move16();
            factor = fenvL_16;
            move16();/*Q3 */
            factor1 = mult_r(sub(shl(wfenv,2),fenvL_16),4096); /*Q3  */
        }
        ELSE
        {
            tmp = 14;
            move16();
            factor = shr(fenvL_16,2);/*Q1 */
            factor1 = mult_r(sub(wfenv,factor), 4096); /*Q1 */
        }

        tmp2 = add(add(swb_bwe_subband_fx[0],8),st_offset);
        FOR (n_freq = add(swb_bwe_subband_fx[0],st_offset); n_freq < tmp2; n_freq++)
        {
            L_tmp1 = Mult_32_16(SWB_signal_32[n_freq], factor);
            SWB_signal_32[n_freq] = L_shl(L_tmp1, tmp-1);
            move32();/*15+Qsyn */
            factor = add(factor, factor1); /*Q3 */
        }

        FOR(n_band = 0; n_band < 12; n_band++)
        {
            L_tmp = L_mult(weight,prev_SWB_fenv[n_band+1]); /*Q19 */
            L_tmp = L_mac(L_tmp, sub(32767,weight), SWB_fenv[n_band+1]);/*Q19 */
            wfenv = round_fx(L_tmp); /*Q1 */
            factor = SWB_fenv[n_band];
            move16(); /*Q1 */
            factor1 = mult_r(sub(wfenv,SWB_fenv[n_band]), smooth_factor_fx[n_band]); /*Q1 */
            tmp = norm_s(factor);
            IF ( sub(tmp,4) > 0)
            {
                tmp = 12;
                move16();
                factor = shl(factor, 2);
                factor1 = shl(factor1, 2);
            }
            ELSE
            {
                tmp = 14;
                move16();
            }

            tmp2 = add(swb_bwe_sm_subband_fx[n_band+1],st_offset);
            FOR (; n_freq < tmp2; n_freq++)
            {
                L_tmp1 = Mult_32_16(SWB_signal_32[n_freq], factor);
                SWB_signal_32[n_freq] = L_shl(L_tmp1, tmp-1);
                move32(); /*15+Qsyn */
                factor = add(factor, factor1); /*Q1 */
            }
        }
        L_tmp = L_mult(weight, prev_SWB_fenv[13]); /*Q19 */
        L_tmp = L_mac(L_tmp,sub(32767,weight), SWB_fenv[13]);/*Q19 */
        wfenv = round_fx(L_tmp); /*Q1 */
        factor = SWB_fenv[12];
        move16();/*Q1 */
        factor1 = mult_r(sub(wfenv, SWB_fenv[12]), smooth_factor_fx[12]); /*Q1 */
        tmp2 = add(swb_bwe_sm_subband_fx[13],st_offset);
        FOR ( ; n_freq < tmp2; n_freq++)
        {
            L_tmp1 = Mult_32_16(SWB_signal_32[n_freq], factor);
            SWB_signal_32[n_freq] = L_shl(L_tmp1,13);
            move32();/*15+Qsyn */
            factor = add(factor,factor1); /*Q1 */
        }

        FOR(n_band=13; n_band<SWB_FENV; n_band++)
        {
            L_tmp = L_mult(weight,prev_SWB_fenv[n_band]); /*Q19 */
            L_tmp = L_mac(L_tmp,sub(32767,weight),SWB_fenv[n_band]);/*Q19 */
            wfenv = round_fx(L_tmp); /*Q1 */
            tmp2 = add(swb_bwe_subband_fx[n_band+1],st_offset);
            FOR ( ; n_freq<tmp2; n_freq++)
            {
                L_tmp1 = Mult_32_16(SWB_signal_32[n_freq], factor);
                SWB_signal_32[n_freq] = L_shl(L_tmp1,13);
                move32(); /*15+Qsyn */
            }
        }
        FOR(n_band = 0; n_band < SWB_FENV; n_band++)
        {
            prev_SWB_fenv[n_band] = SWB_fenv[n_band];
            move16();/*Q1 */
        }
    }
    pit1_32 = &SWB_signal_32[240+st_offset]; /*15+Qsyn */
    FOR(n_freq=0; n_freq<4; n_freq++)
    {
        L_tmp1 = Mult_32_16(*pit1_32, 16384);    /*15+Qsyn */
        *(pit1_32++) = L_tmp1;
        move32();
    }
    *prev_Energy = Energy_16;
    move16();

    return;
}

/*==========================================================================*/
/* FUNCTION      : void time_envelop_shaping_fx()                           */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :   Time shaping of SHB signal                             */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS                                                          */
/* _(Word16) L          :length                                             */
/* _(Word16) Q_syn      :Q format                                           */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*   _None                                                                  */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*   _(Word16[])werr      : SHB synthesis                                   */
/*   _(Word16[])SWB_tenv    : frequency envelope                            */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*   _ None                                                                 */
/*--------------------------------------------------------------------------*/
void time_envelop_shaping_fx(
    Word16 werr[],             /* i/o: SHB synthesis           Q_synth*/
    Word32 SWB_tenv[],         /* i/o: frequency envelope          Q15*/
    const Word16 L ,                 /* i  : frame length                   */
    Word16 *Q_synth
)
{
    Word16 *pit;
    Word32 Energy;
    Word16 i, j;
    Word16 tmp_ener, Energy_16;
    Word16 exp_L, exp, frac, tmp, inv_L;
    Word32 L_tmp;

    pit = werr;
    move16();
    exp_L = norm_s(L);
    inv_L = div_s(shl(1,sub(14,exp_L)), L); /*Q(29-exp_L) */
    FOR(i=0; i<SWB_TENV; i++)
    {
        Energy = L_deposit_l(0);
        FOR(j=0; j<L/4; j++)
        {
            Energy = L_mac0(Energy, *pit, *pit); /*(2*Q_synth) */
            pit++;
        }
        Energy = Mult_32_16(Energy, inv_L); /*Q(29-exp_L-15) -> Q(-exp_L+14+2*Q_synth) */
        Energy_16 = 0;
        move16();
        /*exp = 31-(-exp_L+14 +(2*(*Q_synth))); */
        exp = sub(17,sub(shl((*Q_synth),1),exp_L));

        IF(Energy != 0)
        {
            exp = norm_l(Energy);
            frac = extract_h(L_shl(Energy, exp));
            /*exp = sub(exp, 30-(-exp_L+14-2+(2*(*Q_synth))));  */
            exp = sub(exp,sub(30,add(sub(shl((*Q_synth),1),exp_L),14-2)));

            tmp = div_s(16384, frac);
            L_tmp = L_deposit_h(tmp);
            Energy = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */
            Energy_16 = round_fx(L_shl(Energy, sub(exp,15))); /*Q0 */
        }

        test();
        IF(L_sub(SWB_tenv[i], 65536) < 0 && L_sub(Energy, L_shl(SWB_tenv[i], sub(16,exp))) < 0)
        {
            *Q_synth = add(*Q_synth, 3);
            move16();
        }
        ELSE
        {
            pit -= L/4;
            move16();
            tmp_ener = 0;
            move16();
            exp = 0;
            move16();

            IF(Energy_16 != 0)
            {
                exp = norm_s(Energy_16);
                tmp_ener = div_s(shl(1,sub(14,exp)), Energy_16); /*Q(29-exp) */
            }

            L_tmp = Mult_32_16(SWB_tenv[i], tmp_ener); /*Q(29-exp) */
            tmp = round_fx(L_tmp); /*Q(13-exp) */

            FOR (j = 0; j < L/4; j++)
            {
                *pit = round_fx(L_shl(L_mult(tmp, *pit), sub(exp,1))); /*Q(13-exp+1)->Q(14-exp)->Qsyn-3 */
                pit++;
            }
        }
    }

    return;
}

/*==========================================================================*/
/* FUNCTION      : void time_reduce_pre_echo_fx()                           */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :   Windowing and time-domain aliasing                     */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS                                                          */
/* _(Word16*) synth         :ACELP core synthesis    Q_syn                  */
/* _(Word16) L          :subframe length                                    */
/* _(Word16) Q_syn        :Q format                                         */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*   _(Word16*)error        : SHB BWE synthesis       Q_syn                 */
/*   _(Word16)prev_td_energy    : last td energy          Q_syn             */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*   _None                                                                  */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*   _ None                                                                 */
/*--------------------------------------------------------------------------*/
void time_reduce_pre_echo_fx(
    const Word16 *synth,             /* i  : ACELP core synthesis    Q_syn*/
    Word16 *error,             /* i/o: SHB BWE synthesis          Q0*/
    Word16 prev_td_energy,     /* o  : last td energy             Q0*/
    const Word16 L,                  /* i  : subframe length              */
    Word16 Q_syn,
    Word16 Q_synth
)
{
    Word16 i, j, pos = 0, Len;
    Word32 energy;
    Word16 energy_16;
    Word32 energyL[4];
    Word16 tmp_ener;
    Word16 *pit;
    Word16 tmpi, tmp_exp;
    Word16 exp_L, exp, frac, inv_L, exp_j, tmp;
    Word32 L_tmp, L_tmp1, Ltmp_ener;

    exp_L = norm_s(L);
    inv_L = div_s(shl(1,sub(14,exp_L)), L); /*Q(29-exp_L) */
    FOR(i=0; i<4; i++)
    {
        Len = i_mult(L, i);

        L_tmp = L_deposit_l(0);
        FOR(j=0; j<L; j++)
        {
            L_tmp = L_mac0(L_tmp, synth[Len+j], synth[Len+j]); /*2*Q_syn */
        }
        energyL[i] = Mult_32_16(L_tmp, inv_L);
        move32();/*Q(2*Q_syn+29-exp_L-15) -> Q(2*Q_syn-exp_L+14) */

        IF(energyL[i] != 0)
        {
            exp = norm_l(energyL[i]);
            frac = extract_h(L_shl(energyL[i], exp));
            exp = sub(exp,sub(16,sub(shl(Q_syn,1),exp_L)));

            tmp = div_s(16384, frac);
            L_tmp = L_deposit_h(tmp);
            energyL[i] = L_shl(Isqrt_lc(L_tmp, &exp), sub(exp, 16));
            move32();/*Q(31-exp + (exp-16)) -> Q15 */
        }
    }

    FOR(i=0; i<3; i++)
    {
        L_tmp = Mult_32_16(energyL[i], 29491); /*Q14 */
        test();
        IF(L_sub(L_shr(energyL[i+1], 1), L_tmp) > 0 && L_sub(energyL[i+1], 1638400) > 0)
        {
            pos = add(i, 1);
            move16();
            BREAK;
        }
    }

    IF (pos > 0)
    {
        if(sub(pos, 3) < 0)
        {
            pos = add(pos, 1);
        }
        energy = L_deposit_l(0);
        j = i_mult(L, pos);
        move16();
        FOR(i=0; i<j; i++)
        {
            energy = L_mac0(energy, error[i], error[i]); /*Q(2*Q_synth) */
        }

        exp_j = norm_s(j);
        tmp = div_s(shl(1,sub(14,exp_j)), j); /*Q(29-exp_j) */
        energy = Mult_32_16(energy, tmp); /*Q(29-exp_j+1-16) -> Q(-exp_j+14 +2*Q_synth) */
        energy_16 = 0;
        move16();

        IF(energy != 0)
        {
            exp = norm_l(energy);
            frac = extract_h(L_shl(energy, exp));
            /*exp = sub(exp, 30-(-exp_j+14 +2*Q_synth));  */
            exp = sub(exp, sub(14, sub(shl(Q_synth, 1), exp_j)));
            tmp = div_s(16384, frac);
            L_tmp = L_deposit_h(tmp);
            energy = Isqrt_lc(L_tmp, &exp);  /*Q(31-exp) */
            energy_16 = round_fx(L_shl(energy, sub(exp,15))); /*Q0 */
        }

        tmp = mult_r(energy_16, 6554); /*Q0 */
        if(sub(prev_td_energy, tmp) < 0)
        {
            prev_td_energy = tmp;
            move16();
        }

        tmp_ener = 0;
        move16();
        exp = 0;
        move16();
        IF(energy_16 != 0)
        {
            exp = norm_s(energy_16);
            tmp_ener = div_s(shl(1,sub(14,exp)), energy_16); /*Q(29-exp) */
        }
        L_tmp = L_mult(prev_td_energy, tmp_ener); /*Q(30-exp) */
        tmp_exp = add(1, exp);
        FOR (i = 0; i < j; i++)
        {
            error[i] = round_fx(L_shl(Mult_32_16(L_tmp, error[i]), tmp_exp)); /*Q(30-exp+1-16)->Q(15-exp)->Q_synth */
        }

        energy = L_deposit_l(0);
        FOR(i=j; i<(j+L); i++)
        {
            energy = L_mac0(energy, error[i], error[i]); /*(2*Q_synth) */
        }

        energy = Mult_32_16(energy, inv_L); /*Q(29-exp_L+1-16) -> Q(-exp_L+14) */
        energy_16 = 0;
        move16();
        IF(energy != 0)
        {
            exp = norm_l(energy);
            frac = extract_h(L_shl(energy, exp));
            /*exp = sub(exp, 30-(-exp_L+14+2*Q_synth));  */
            exp = sub(exp, sub(14, sub(shl(Q_synth, 1), exp_L)));

            tmp = div_s(16384, frac);
            L_tmp = L_deposit_h(tmp);
            energy = Isqrt_lc(L_tmp, &exp); /*Q(31-exp) */
            energy_16 = round_fx(L_shl(energy, sub(exp,15))); /*Q0 */
        }

        tmp_ener = 0;
        move16();
        exp = 0;
        move16();
        IF(energy_16 != 0)
        {
            exp = norm_s(energy_16);
            tmp_ener = div_s(shl(1,sub(14,exp)), energy_16); /*Q(29-exp) */
        }
        Ltmp_ener = L_mult(prev_td_energy, tmp_ener); /*Q(30-exp) */
        L_tmp1 = L_shl(1, sub(30, exp));

        pit = &error[j];
        move16();
        FOR (i = 0; i < L; i++)
        {
            tmpi = round_fx(L_shl(L_mult(i, inv_L), add(1, exp_L))); /*Q15 */
            L_tmp = L_sub(L_tmp1, Ltmp_ener); /*Q(30-exp) */
            L_tmp = Mult_32_16(L_tmp, tmpi); /*Q(30-exp) */
            L_tmp = L_add(Ltmp_ener, L_tmp); /*Q(30-exp) */
            tmp = round_fx(L_shl(Mult_32_16(L_tmp, *pit), add(1, exp))); /*Q0 */
            *pit++ = tmp;
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * calc_normal_length_fx_32()
 *
 *-------------------------------------------------------------------*/
void calc_normal_length_fx_32(
    const Word16 core,             /* i  : core                   : Q0  */
    const Word32 *sp,              /* i  : input signal           : Q12 */
    const Word16 mode,             /* i  : input mode             : Q0  */
    const Word16 extl,             /* i  : extension layer        : Q0  */
    Word16 *L_swb_norm,      /* o  : normalize length       : Q0  */
    Word16 *prev_L_swb_norm  /*i/o : last normalize length  : Q0  */
)
{
    Word16 i, n_freq, n_band, THRES;
    Word16 L_swb_norm_trans, L_swb_norm_norm, L_swb_norm_harm, L_swb_norm_cur;
    Word16 N;

    const Word32 *pit;
    Word32 peak, mean, mag;
    Word32 L_tmp1, L_tmp2;

    THRES = 4;
    move16();
    test();
    test();
    if( sub(core,HQ_CORE) == 0 || sub(extl,SWB_BWE) == 0 || sub(extl,FB_BWE) == 0 )
    {
        THRES = 8;
        move16();
    }

    N = 16;
    move16();
    test();
    test();
    if( sub(core,HQ_CORE) == 0 && (sub(mode,HQ_HARMONIC) == 0 || sub(mode,HQ_HVQ) == 0) )
    {
        N = 13;
        move16();
    }

    n_band = 0;
    move16();
    pit = sp;
    move16();
    FOR (i = 0; i < N; i ++)
    {
        peak = 0;
        move16();
        mean = 0;
        move16();

        FOR (n_freq = 0; n_freq < 16; n_freq ++)
        {
            mag = L_abs(*pit);
            if (L_sub(mag , peak) > 0)
            {
                peak = mag;
                move16();
            }
            mean = L_add(mean, mag);
            pit++;
        }

        L_tmp1 = Mult_32_16(peak, shl(15+THRES, 10));
        L_tmp2 = Mult_32_16(mean, shl(THRES, 10));
        test();
        if (L_sub(L_tmp1,L_tmp2) > 0 && L_sub(peak,40960) > 0)
        {
            n_band = add(n_band, 1);
        }
    }

    IF( sub(core,ACELP_CORE) == 0 )
    {
        L_swb_norm_trans = add(4, shr(n_band, 2));
        L_swb_norm_norm = add(8, shr(n_band, 1));
        L_swb_norm_harm = s_max(add(32, shl(n_band, 1)), 24);

        IF( mode == HARMONIC )
        {
            L_swb_norm_cur = L_swb_norm_harm ;
            move16();
        }
        ELSE IF ( mode == NORMAL )
        {
            L_swb_norm_cur = L_swb_norm_norm;
            move16();
        }
        ELSE
        {
            L_swb_norm_cur = L_swb_norm_trans;
            move16();
        }

        *L_swb_norm = shr(add(L_swb_norm_cur, *prev_L_swb_norm), 1);
        move16();
        *prev_L_swb_norm = L_swb_norm_cur;
        move16();
    }
    ELSE
    {
        test();
        IF( mode == HQ_HARMONIC || mode == HQ_HVQ )
        {
            L_swb_norm_cur = add(32, add(shl(n_band, 1), shr(n_band, 1)));
        }
        ELSE
        {
            L_swb_norm_cur = add(8, shr(n_band, 1));
        }

        *L_swb_norm = extract_h(L_add(L_mac(L_mult(L_swb_norm_cur, 3276), *prev_L_swb_norm, 29491),32768));
        *prev_L_swb_norm = L_swb_norm_cur;
        move16();
    }

    return;
}


/*-------------------------------------------------------------------*
 * calc_norm_envelop_fx_32()
 *
 *-------------------------------------------------------------------*/
void calc_norm_envelop_fx_32(
    const Word32 SWB_signal_fx[],      /* i  : SWB spectrum           : Q12 */
    Word32 *envelope_fx,               /* o  : normalized envelope    : Q16 */
    const Word16 L_swb_norm,           /* i  : length of envelope     : Q0  */
    const Word16 SWB_flength,          /* i  : Length of input/output : Q0  */
    const Word16 st_offset             /* i  : offset                 : Q0  */
)
{
    Word16 i, lookback, env_index, n_freq, n_lag_now, n_lag, tmp;

    lookback = L_swb_norm/2;
    move16();
    env_index = swb_bwe_subband_fx[0]+st_offset;
    move16();
    n_lag_now = L_swb_norm;
    move16();
    tmp = sub(add(SWB_flength,st_offset),L_swb_norm);
    FOR (n_freq = sub(add(swb_bwe_trans_subband_fx[0],st_offset),lookback); n_freq<tmp; n_freq++)
    {
        /* Apply MA filter */
        envelope_fx[env_index] = 0;
        move16();
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            envelope_fx[env_index] = L_add(envelope_fx[env_index], L_abs(SWB_signal_fx[add(n_freq,n_lag)]));
            move32();
        }
        env_index++;
    }
    i = 0;
    move16();
    tmp = sub(add(SWB_flength,st_offset),lookback);
    FOR (n_freq = sub(add(SWB_flength,st_offset),L_swb_norm); n_freq<tmp; n_freq++)
    {
        Word32 L_tmp;

        n_lag_now = L_swb_norm-i;
        move16();
        /* Apply MA filter */
        L_tmp = L_deposit_l(0);
        FOR (n_lag=0; n_lag<n_lag_now; n_lag++)
        {
            L_tmp = L_add(L_tmp, L_abs(SWB_signal_fx[n_freq+n_lag]));
        }
        envelope_fx[env_index] = L_tmp;
        move32();
        env_index=add(env_index,1);
        i++;
    }

    return;
}

/*-------------------------------------------------------------------*
 * hq_generic_decoding_fx()
 *
 *-------------------------------------------------------------------*/
void hq_generic_decoding_fx(
    const Word16 HQ_mode,                       /* i  : HQ mode                                      : Q0  */
    Word32 *coeff_out1_fx,                  /* i/o: BWE input & temporary buffer                 : Q12 */
    const Word16 *hq_generic_fenv_fx,           /* i  : SWB frequency envelopes                      : Q3  */
    Word32 *coeff_out_fx,                /* o  : SWB signal in MDCT domain                    : Q12 */
    const Word16 hq_generic_offset,             /* i  : frequency offset for representing hq generci : Q0  */
    Word16 *prev_L_swb_norm,                    /* i/o: last normalize length                        : Q0  */
    const Word16 hq_generic_exc_clas            /* i  : bwe excitation class                         " Q0  */
)
{
    Word16 i, n_freq, n_band, L_swb_norm;
    Word16 k;
    Word16 nenv;
    Word16 tenv;

    Word16 exp, exp1, exp2, frac, tmp, tmp2, cs;
    Word32 L_tmp, L_tmp1, L_tmp2, max_coeff_fx;
    Word16 fenvL_fx, wfenv_fx, factor_fx;
    Word32* pit1_fx;
    Word16 tmp1_fx, tmp2_fx, tmp3_fx, tmp4_fx;
    Word32 energy_fx;

    Word32 envelope_fx[L_FRAME16k];
    Word32 mean_vector_fx[20];
    Word16 rn_weight0_fx;
    Word16 s;
    Word16 blen,nband_lf,sfidx,efidx;
    Word16 bwe_seed;
    Word16 signum[L_FRAME16k];

    nenv = sub(SWB_FENV,2);
    if ( sub(hq_generic_offset, HQ_GENERIC_FOFFSET_24K4) <= 0 )
    {
        nenv = SWB_FENV;
        move16();
    }

    tenv = nenv;
    move16();
    if ( sub(HQ_mode, HQ_GEN_FB) == 0 )
    {
        tenv = add(nenv , DIM_FB);
    }

    max_coeff_fx = 0;
    move16();
    tmp = add(swb_bwe_subband_fx[0] , hq_generic_offset);
    FOR ( n_freq = add(HQ_GENERIC_ST_FREQ , hq_generic_offset); n_freq<tmp; n_freq++ )
    {
        max_coeff_fx = L_max(max_coeff_fx, L_abs(coeff_out1_fx[n_freq]));
    }
    cs = norm_l(max_coeff_fx);

    L_tmp = 0;
    move16();
    tmp2 = add(swb_bwe_subband_fx[0] , hq_generic_offset);
    FOR ( n_freq = add(HQ_GENERIC_ST_FREQ , hq_generic_offset); n_freq<tmp2; n_freq++ )
    {
        tmp = extract_h(L_shl(coeff_out1_fx[n_freq], cs));/*12 + cs - 16 */
        L_tmp1 = L_mult0(tmp, tmp);/*2*(cs-2) */
        L_tmp = L_add(L_tmp, L_shr(L_tmp1, 5));/*2*(cs-2) - 5 */
    }
    cs = sub(shl(cs, 1), 9);
    fenvL_fx = 0;
    move16();
    IF (L_tmp != 0)
    {
        exp = norm_l(L_tmp);
        frac = round_fx(L_shl(L_tmp, exp));/*cs+exp-16 */
        tmp = div_s(16384, frac);/*15 + 14 - (cs+exp-16) */
        exp = sub(add(cs, exp), 30);
        L_tmp = Isqrt_lc(L_deposit_h(tmp), &exp);/*Q31 - exp */
        fenvL_fx = round_fx(L_shl(L_tmp, sub(exp, 14)));/*Q1 */
    }

    calc_normal_length_fx_32(HQ_CORE, coeff_out1_fx, HQ_GEN_SWB, -1, &L_swb_norm, prev_L_swb_norm);

    calc_norm_envelop_lf_fx( coeff_out1_fx, envelope_fx, &L_swb_norm, HQ_mode, hq_generic_offset, &sfidx, &efidx);

    blen = 16;
    move16();

    IF ( sub(hq_generic_exc_clas, HQ_GENERIC_EXC0) == 0 )
    {
        rn_weight0_fx = 819;
        move16();/* 0.8 Q10 */
    }
    ELSE IF ( sub(hq_generic_exc_clas, HQ_GENERIC_EXC1) == 0 )
    {
        rn_weight0_fx = 51;
        move16();/* 0.05 Q10*/
    }
    ELSE
    {
        rn_weight0_fx = 205;
        move16();/* 0.02 Q10 */
    }

    tmp = sub(efidx,sfidx);
    IF( sub(tmp,0) == 0 )
    {
        nband_lf = 0;
    }
    ELSE
    {
        exp = norm_s(tmp);
        nband_lf = shl(tmp,sub(exp,1));
        exp1 = norm_s(blen);
        tmp = shl(blen,exp1);
        nband_lf = shr(div_s(nband_lf,tmp), add(sub(14,exp1),exp) );/* 15 + exp-1 - exp1, Q0*/
    }

    FOR ( n_freq = sfidx; n_freq < efidx; n_freq++ )
    {
        IF ( coeff_out1_fx[n_freq] < 0 )
        {
            signum[n_freq] = -1;
            move16();
            coeff_out1_fx[n_freq] = L_negate(coeff_out1_fx[n_freq]);
            move32();
        }
        ELSE
        {
            signum[n_freq] = 1;
            move16();
        }
    }

    /* applying whitening */
    FOR ( n_freq = sfidx; n_freq < efidx; n_freq++ )
    {
        exp = norm_l(coeff_out1_fx[n_freq]) - 1;
        exp1 = norm_l(envelope_fx[n_freq]);

        L_tmp = L_shl(coeff_out1_fx[n_freq],exp);
        L_tmp1 = L_shl(envelope_fx[n_freq],exp1);

        logic16();
        coeff_out1_fx[n_freq] =Div_32(L_tmp, extract_h(L_tmp1), extract_l(L_shr(L_tmp1,1)) & 0x00007fff);
        move32();/*31 + exp1 - exp2*/

        exp = add(31,sub(exp,exp1));
        coeff_out1_fx[n_freq] = L_shl(coeff_out1_fx[n_freq], sub(12,exp));
        move32();/*Q12*/
    }

    /* mean vector generation for controlling dynamic range */
    FOR( k =0 ; k < nband_lf; ++k )
    {
        energy_fx = 1;
        move16();

        tmp = add(i_mult2(add(k,1),blen),sfidx);
        FOR ( i = add(i_mult2(k,blen),sfidx); i < tmp; ++i )
        {
            energy_fx = L_add(energy_fx,coeff_out1_fx[i]);
        }
        exp = sub(norm_l(energy_fx),1);
        L_tmp = L_shl(energy_fx,exp);
        exp1 = norm_l( L_and(blen, 0x00007fff ) );
        L_tmp1 = L_shl( blen, exp1 );
        logic16();
        mean_vector_fx[k] = Div_32(L_tmp, extract_h(L_tmp1), extract_l(L_shr(L_tmp1,1)) & 0x00007fff);
        move32();/*31 + 12 + exp1 - exp2*/
        exp = add(43,sub(exp,exp1));
        mean_vector_fx[k] = L_shl(mean_vector_fx[k], sub(12,exp));
        move32();/*Q12*/
    }

    /* dynamics control */
    FOR( k =0 ; k < nband_lf; ++k )
    {
        tmp = add( i_mult2(add(k,1),blen), sfidx);
        FOR ( i = add(i_mult2(k,blen),sfidx); i < tmp; ++i )
        {
            L_tmp = L_sub(coeff_out1_fx[i],mean_vector_fx[k]);
            exp = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp); /* exp+12*/
            exp1 = norm_l(rn_weight0_fx);
            L_tmp1 = L_shl(rn_weight0_fx, exp1); /* exp1+10*/
            L_tmp = L_mult( extract_h(L_tmp), extract_h(L_tmp1) );
            L_tmp = L_shr(L_tmp, add(exp,sub(exp1,21) )); /* Q12*/
            coeff_out1_fx[i] = L_sub(coeff_out1_fx[i], L_tmp);
            move32();
        }
    }

    IF ( sub(hq_generic_exc_clas, HQ_GENERIC_EXC0) == 0 )
    {

        L_tmp = L_add(L_add(coeff_out1_fx[1],coeff_out1_fx[2]),L_add(coeff_out1_fx[3],coeff_out1_fx[4]));
        IF( L_tmp < 0 )
        {
            L_tmp = L_negate( L_tmp );
            L_tmp = L_shr( L_tmp, 12 );
            L_tmp = L_negate( L_tmp );
            bwe_seed = extract_l(L_shr( L_tmp, 2 ));
        }
        ELSE
        {
            bwe_seed = extract_l(L_shr(L_tmp , 14));
        }

        FOR ( n_freq = sfidx; n_freq < efidx; n_freq++ )
        {
            IF ( signum[n_freq] < 0 )
            {
                coeff_out1_fx[n_freq] = L_negate(coeff_out1_fx[n_freq]);
                move32();
            }

            IF ( Random(&bwe_seed) < 0 )
            {
                coeff_out1_fx[n_freq] = L_negate(coeff_out1_fx[n_freq]);
                move32();
            }
        }
    }
    ELSE
    {
        FOR ( n_freq = sfidx; n_freq < efidx; n_freq++ )
        {
            IF ( signum[n_freq] < 0 )
            {
                coeff_out1_fx[n_freq] = L_negate(coeff_out1_fx[n_freq]);
                move32();
            }
        }
    }

    /* normalizing modified low frequency spectrum */
    FOR( k =0 ; k < nband_lf; ++k )
    {
        energy_fx = 1;
        move16();
        tmp = add(i_mult2(add(k,1),blen),sfidx);
        FOR ( i = add((i_mult2(k,blen)),sfidx); i < tmp; ++i )
        {
            exp = norm_l(coeff_out1_fx[i]);
            L_tmp1 = L_shl(coeff_out1_fx[i],exp);/* exp + 12*/

            L_tmp = Mult_32_32( L_tmp1, L_tmp1);
            L_tmp = L_shr( L_tmp, sub(i_mult2(2,exp),19) );/*Q12  */

            energy_fx = L_add( energy_fx, L_tmp );
        }

        exp = norm_l(energy_fx);
        L_tmp = L_shl(energy_fx,sub(exp,1));
        exp1 = norm_s(blen);
        L_tmp1 = L_shl((Word32)blen,add(exp1,16));

        L_tmp = Div_32( L_tmp, extract_h(L_tmp1), extract_l( L_shr(L_tmp1,1) ) & 0x00007fff );
        exp = sub( add( 26 , exp ), exp1 );
        L_tmp = L_shr( L_tmp, sub(exp,31) );
        exp = 31;
        move16();

        exp = sub( 31 , exp );
        IF (exp & 0x1)
        {
            L_tmp = L_shr( L_tmp , 1 );
            exp = add(exp,1);
        }
        L_tmp = Sqrt_l(L_tmp, &exp1);
        exp = add( 31 , sub( shr(exp1,1) , shr(exp,1) ) );
        energy_fx = L_shl(L_tmp, sub(31,exp));  /*Q31*/

        tmp = add(i_mult2(add(k,1),blen),sfidx);
        FOR ( i = add((i_mult2(k,blen)),sfidx); i < tmp; ++i )
        {
            IF ( L_sub( abs( coeff_out1_fx[i] ), coeff_out1_fx[i] ) != 0 )
            {
                s = -1;
                move16();
                coeff_out1_fx[i] = L_abs(coeff_out1_fx[i]);
                move32();
            }
            ELSE
            {
                s = 0;
                move16();
            }
            exp = norm_l(coeff_out1_fx[i]);
            L_tmp = L_shl(coeff_out1_fx[i],sub(exp,1));
            exp1 = norm_l(energy_fx);
            L_tmp1 = L_shl((Word32)energy_fx,exp1);
            logic16();
            L_tmp = Div_32( L_tmp, extract_h(L_tmp1), extract_l(L_shr(L_tmp1,1)) & 0x00007fff );
            exp = add ( sub( 11 , exp1 ), exp );
            coeff_out1_fx[i] = L_shl(L_tmp,add(sub(12,exp),15));
            move32();      /* Q12 -> Q27 */
            IF( s )
            {
                coeff_out1_fx[i] = L_negate( coeff_out1_fx[i] );
                move32();
            }
        }
    }

    Copy32(&coeff_out1_fx[HQ_GENERIC_OFFSET], &coeff_out_fx[add(HQ_GENERIC_HIGH0,hq_generic_offset)], HQ_GENERIC_LEN0);
    Copy32(&coeff_out1_fx[HQ_GENERIC_OFFSET], &coeff_out_fx[add(HQ_GENERIC_HIGH1,hq_generic_offset)], HQ_GENERIC_LEN0);

    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4) <= 0)
    {
        Copy32( &coeff_out1_fx[HQ_GENERIC_LOW0], &coeff_out_fx[add(HQ_GENERIC_HIGH2,hq_generic_offset)], sub(HQ_GENERIC_END_FREQ , HQ_GENERIC_HIGH2) );
    }

    IF ( sub(HQ_mode , HQ_GEN_FB) == 0 )
    {
        IF ( sub(hq_generic_offset, HQ_GENERIC_FOFFSET_24K4) <= 0 )
        {
            Copy32(&coeff_out1_fx[sub(add(HQ_GENERIC_LOW0 , HQ_GENERIC_END_FREQ),HQ_GENERIC_HIGH2)], &coeff_out_fx[fb_bwe_subband[0]], 160);
        }
        ELSE
        {
            Copy32(&coeff_out1_fx[add(HQ_GENERIC_OFFSET , HQ_GENERIC_LEN0)], &coeff_out_fx[fb_bwe_subband[0]], 160);
        }
    }


    L_tmp1 = L_deposit_l(0);
    L_tmp2 = L_deposit_l(0);
    FOR(i=0; i<5; ++i)
    {
        L_tmp1 = L_add(L_tmp1, L_abs(coeff_out_fx[add(add(HQ_GENERIC_HIGH1,hq_generic_offset),i)]));
        L_tmp2 = L_add(L_tmp2, L_abs(coeff_out_fx[sub(add(sub(HQ_GENERIC_HIGH1,2),hq_generic_offset),i)]));
    }

    pit1_fx = &coeff_out_fx[add(HQ_GENERIC_HIGH1,hq_generic_offset)];
    L_tmp1 = L_max(L_tmp1,1);
    L_tmp2 = L_max(L_tmp2,1);
    exp1 = norm_l(L_tmp1);
    exp2 = sub(norm_l(L_tmp2), 1);
    tmp1_fx = extract_h(L_shl(L_tmp1, exp1));
    tmp2_fx = extract_h(L_shl(L_tmp2, exp2));
    tmp3_fx = div_s(tmp2_fx, tmp1_fx);/*15 + exp2 + 15 - (exp1 + 15) */
    tmp3_fx = shr(tmp3_fx, add(5, sub(exp2, exp1)));/*10 */

    if (sub(tmp3_fx , 307) < 0)
    {
        tmp3_fx = 307;
        move16();
    }
    FOR ( ; tmp3_fx < 1024; tmp3_fx += 102)
    {
        *pit1_fx = Mult_32_16(*pit1_fx, shl(tmp3_fx, 5));
        move32();/*15 + 5 + 10 -15 */
        pit1_fx++;
    }

    pit1_fx = &coeff_out_fx[add(sub(HQ_GENERIC_HIGH1,1),hq_generic_offset)];

    exp1 = sub(norm_l(L_tmp1), 1);
    exp2 = norm_l(L_tmp2);
    tmp1_fx = extract_h(L_shl(L_tmp1, exp1));
    tmp2_fx = extract_h(L_shl(L_tmp2, exp2));
    tmp3_fx = div_s(tmp1_fx, tmp2_fx);/*15 + exp2 + 15 - (exp1 + 15) */
    tmp3_fx = shr(tmp3_fx, add(5, sub(exp1, exp2)));/*10 */

    IF (sub(tmp3_fx , 5120)>0)
    {
        FOR ( tmp3_fx = 5120; tmp3_fx > 1024; tmp3_fx -= 512)
        {
            L_tmp1 = Mult_32_16(L_shl(*pit1_fx, 5), tmp3_fx);      /*15 + 5 + 10 -15 */
            *pit1_fx-- = L_tmp1;
            move32();
        }
    }

    IF ( sub(hq_generic_offset , HQ_GENERIC_FOFFSET_24K4) <= 0  )
    {
        L_tmp1 = L_add(L_abs(coeff_out_fx[add(HQ_GENERIC_HIGH2 , hq_generic_offset)]), L_abs(coeff_out_fx[add(add(HQ_GENERIC_HIGH2,1),hq_generic_offset)]));
        L_tmp2 = L_add(L_abs(coeff_out_fx[add(sub(HQ_GENERIC_HIGH2,4),hq_generic_offset)]), L_add(L_abs(coeff_out_fx[add(sub(HQ_GENERIC_HIGH2,3),hq_generic_offset)]),
                       L_add( L_abs(coeff_out_fx[add(sub(HQ_GENERIC_HIGH2,2),hq_generic_offset)]), L_abs(coeff_out_fx[add(sub(HQ_GENERIC_HIGH2,1),hq_generic_offset)]))));

        pit1_fx = &coeff_out_fx[add(HQ_GENERIC_HIGH2,hq_generic_offset)];

        L_tmp1 = L_max(L_tmp1,1);
        L_tmp2 = L_max(L_tmp2,1);
        exp1 = norm_l(L_tmp1);
        exp2 = sub(norm_l(L_tmp2), 1);
        tmp1_fx = extract_h(L_shl(L_tmp1, exp1));
        tmp2_fx = extract_h(L_shl(L_tmp2, exp2));
        tmp3_fx = div_s(tmp2_fx, tmp1_fx);/*15 + exp2 + 15 - (exp1 + 15) */
        tmp3_fx = shr(tmp3_fx, add(5, sub(exp2, exp1)));/*10 */

        if (sub(tmp3_fx, 307) < 0)
        {
            tmp3_fx = 307;
            move16();
        }
        FOR ( ; tmp3_fx < 1024; tmp3_fx += 102)
        {
            L_tmp = L_shl(Mult_32_16(*pit1_fx, tmp3_fx), 5);      /*15 + 5 + 10 -15 */
            *pit1_fx++ = L_tmp;
            move32();
        }

        pit1_fx = &coeff_out_fx[add(sub(HQ_GENERIC_HIGH2,1),hq_generic_offset)];

        exp1 = sub(norm_l(L_tmp1), 1);
        exp2 = norm_l(L_tmp2);
        tmp1_fx = extract_h(L_shl(L_tmp1, exp1));
        tmp2_fx = extract_h(L_shl(L_tmp2, exp2));
        tmp3_fx = div_s(tmp1_fx, tmp2_fx);/*15 + exp2 + 15 - (exp1 + 15) */
        tmp3_fx = shr(tmp3_fx, add(5, sub(exp1, exp2)));/*10 */
        tmp3_fx = shr(tmp3_fx, 1);
        tmp4_fx = mult_r(tmp3_fx, 1638);
        WHILE (tmp3_fx > 1024)
        {
            L_tmp1 = Mult_32_16(L_shl(*pit1_fx, 5), tmp3_fx);      /*15 + 5 + 10 -15 */
            *pit1_fx-- = L_tmp1;
            move32();
            tmp3_fx = sub(tmp3_fx, tmp4_fx);
        }
    }


    wfenv_fx = hq_generic_fenv_fx[0];
    move16();/*1 */
    i = 0;
    move16();
    tmp2 = add(add(swb_bwe_subband_fx[0],hq_generic_offset),8);
    FOR (n_freq = add(swb_bwe_subband_fx[0],hq_generic_offset); n_freq<tmp2; n_freq++)
    {
        factor_fx = shl(i, 12);/*15 */
        L_tmp1 = L_mult(sub(32767, factor_fx), fenvL_fx);/*17 */
        L_tmp2 = L_mult(factor_fx, wfenv_fx);/*17 */
        L_tmp1 = L_add(L_tmp1, L_tmp2);/*17 */

        cs = norm_l(L_tmp1);
        tmp = extract_h(L_shl(L_tmp1, cs));/*17 + cs - 16 */
        L_tmp = Mult_32_16(coeff_out_fx[n_freq] , tmp);/*12 + 15 + 17 + cs - 16 - 15 */
        coeff_out_fx[n_freq] = L_shr(L_tmp, add(1, cs));
        move32();/*12 */
        i++;
        move16();
    }

    k = sub(nenv,2);
    FOR (n_band=0; n_band<k; n_band++)
    {
        wfenv_fx = hq_generic_fenv_fx[add(n_band,1)];
        move16();/*1 */

        tmp2 = swb_bwe_sm_subband_fx[add(n_band,1)]+hq_generic_offset;
        FOR ( i=0; n_freq<tmp2; i++)
        {
            L_tmp1 = L_mult(sub(wfenv_fx, hq_generic_fenv_fx[n_band]), smooth_factor_fx[n_band]);/*17 */
            L_tmp1 = Mult_32_16(L_tmp1, shl(i, 10));/*17 + 10 - 15 */
            L_tmp1 = L_add(L_tmp1, L_shl(hq_generic_fenv_fx[n_band], 11));/*12 */

            cs = norm_l(L_tmp1);
            tmp = extract_h(L_shl(L_tmp1, cs));/*12 + cs - 16 */
            L_tmp = Mult_32_16(coeff_out_fx[n_freq] , tmp);/*12 + 15 + 12 + cs - 16 - 15 */
            coeff_out_fx[n_freq] = L_shl(L_tmp, sub(4, cs));
            move32();/*12 */
            n_freq++;
        }
    }

    wfenv_fx = hq_generic_fenv_fx[sub(nenv,1)];
    move16();/*1 */
    tmp2 = add(swb_bwe_sm_subband_fx[sub(nenv,1)],hq_generic_offset);
    FOR ( i=0; n_freq<tmp2; i++)
    {
        L_tmp1 = L_mult(sub(wfenv_fx, hq_generic_fenv_fx[sub(nenv,2)]), smooth_factor_fx[sub(nenv,2)]);/*17 */
        L_tmp1 = Mult_32_16(L_tmp1, shl(i, 10));/*17 + 10 - 15 */
        L_tmp1 = L_add(L_tmp1, L_shl(hq_generic_fenv_fx[sub(nenv,2)], 11));/*12 */

        cs = norm_l(L_tmp1);
        tmp = extract_h(L_shl(L_tmp1, cs));/*12 + cs - 16 */
        L_tmp = Mult_32_16(coeff_out_fx[n_freq] , tmp);/*12 + 15 + 12 + cs - 16 - 15 */
        coeff_out_fx[n_freq] = L_shl(L_tmp, sub(4, cs));
        move32();/*12 */
        n_freq++;
    }

    IF ( sub(HQ_mode , HQ_GEN_SWB) == 0 )
    {
        FOR(n_band=sub(nenv,1); n_band<nenv; ++n_band)
        {
            wfenv_fx = hq_generic_fenv_fx[n_band];
            move16();/*1 */
            tmp2 = add(swb_bwe_subband_fx[add(n_band,1)],hq_generic_offset);
            FOR ( ; n_freq<tmp2; n_freq++)
            {
                L_tmp = Mult_32_16(coeff_out_fx[n_freq], wfenv_fx);/*15 + 12 + 1 - 15 */
                coeff_out_fx[n_freq] = L_shr(L_tmp, 1);
                move32();/*12 */
            }
        }
    }
    ELSE
    {
        test();
        IF ( hq_generic_fenv_fx[sub(nenv,1)] - hq_generic_fenv_fx[nenv] > 30 ||  hq_generic_fenv_fx[nenv] < 10)
        {
            wfenv_fx = hq_generic_fenv_fx[sub(nenv,1)];
            move16();/*1 */
            FOR ( i=0; n_freq<fb_bwe_subband[0]; i++)
            {
                L_tmp = Mult_32_16(coeff_out_fx[n_freq], wfenv_fx);/*15 + 12 + 1 - 15 */
                coeff_out_fx[n_freq] = L_shr(L_tmp, 1);
                move32();/*12 */
                n_freq++;
            }

            FOR (n_band=0; n_band<DIM_FB ; n_band++)
            {
                wfenv_fx = hq_generic_fenv_fx[add(n_band , nenv)];
                move16();/*1 */
                tmp2 = fb_bwe_subband[add(n_band,1)];
                FOR ( i=0; n_freq<tmp2; i++)
                {
                    L_tmp = Mult_32_16(coeff_out_fx[n_freq], wfenv_fx);/*15 + 12 + 1 - 15 */
                    coeff_out_fx[n_freq] = L_shr(L_tmp, 1);
                    move32();/*12 */
                    n_freq++;
                }
            }
        }
        ELSE
        {
            FOR (n_band=0; n_band<DIM_FB ; n_band++)
            {
                wfenv_fx = hq_generic_fenv_fx[sub(add(n_band,nenv),1)];
                move16();/*1 */

                FOR ( i=0; n_freq<fb_bwe_sm_subband[n_band]; i++)
                {
                    L_tmp1 = L_mult(sub(wfenv_fx, hq_generic_fenv_fx[add(n_band,nenv)]), fb_smooth_factor_fx[n_band]);/*17 */
                    L_tmp1 = Mult_32_16(L_tmp1, shl(i, 9));/*17 + 9 - 15 */
                    L_tmp1 = L_add(L_tmp1, L_shl(hq_generic_fenv_fx[add(n_band,nenv)], 10));/*11 */

                    cs = norm_l(L_tmp1);
                    tmp = extract_h(L_shl(L_tmp1, cs));/*11 + cs - 16 */
                    L_tmp = Mult_32_16(coeff_out_fx[n_freq] , tmp);/*12 + 15 + 11 + cs - 16 - 15 */
                    coeff_out_fx[n_freq] = L_shl(L_tmp, sub(5, cs));
                    move32();/*12 */
                    n_freq++;
                }
            }

            wfenv_fx = hq_generic_fenv_fx[sub(tenv,1)];
            move16();/*1 */

            FOR ( ; n_freq<fb_bwe_subband[DIM_FB]; n_freq++)
            {
                L_tmp = Mult_32_16(coeff_out_fx[n_freq], wfenv_fx);/*15 + 12 + 1 - 15 */
                coeff_out_fx[n_freq] = L_shr(L_tmp, 1);
                move32();/*12 */
            }
        }
    }

    return;
}



/*-------------------------------------------------------------------*
 * save_old_syn()
 *
 * Save and delay the ACELP core synthesis signal by
 * DELAY_FD_BWE_ENC_xxkx to be used by SWB BWE
 *-------------------------------------------------------------------*/

void save_old_syn_fx(
    const Word16 L_frame,        /* i  : frame length                */
    const Word16 syn[],          /* i  : ACELP synthesis             */
    Word16 old_syn[],      /* o  : old synthesis buffer        */
    Word16 old_syn_mem[],  /* i/o: old synthesis buffer memory */
    const Word16 preemph_fac,    /* i  : preemphasis factor          */
    Word16 *mem_deemph     /* i/o: deemphasis filter memory    */
)
{
    Word16 tmps;

    tmps = NS2SA(16000, DELAY_FD_BWE_ENC_16k_NS);
    move16();
    if( sub(L_frame,L_FRAME) == 0 )
    {
        tmps = NS2SA(12800, DELAY_FD_BWE_ENC_12k8_NS);
        move16();
    }
    Copy( old_syn_mem, old_syn, tmps);
    Copy( syn, old_syn + tmps, L_frame - tmps);
    Copy( syn+ L_frame - tmps, old_syn_mem, tmps);

    deemph_fx( old_syn, preemph_fac, L_frame, mem_deemph );

    return;
}
