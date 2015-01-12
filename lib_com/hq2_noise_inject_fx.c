/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "math_op.h"
#include "math_32.h"
#include "oper_32b.h"
#include "move.h"
#include "count.h"

/*--------------------------------------------------------------------------*
 * hq2_noise_inject()
 *
 * HQ2 noise injection for WB signals
 *--------------------------------------------------------------------------*/
void hq2_noise_inject_fx(
    Word32 L_y2[],
    const Word16 band_start[],
    const Word16 band_end[],
    const Word16 band_width[],
    Word32 Ep_fx[],
    Word32 Rk_fx[],
    const Word16 npulses[],
    Word16 ni_seed,
    const Word16 bands,
    const Word16 ni_start_band,
    const Word16 bw_low,
    const Word16 bw_high,
    const Word32 enerL_fx,
    const Word32 enerH_fx,
    Word32 last_ni_gain_fx[],
    Word16 last_env_fx[],
    Word16 *last_max_pos_pulse,
    Word16 *p2a_flags,
    Word16 p2a_bands,
    const Word16 hqswb_clas,
    const Word16 bwidth,
    const Word32 bwe_br
)
{
    Word32 L_tmp,L_tmp2,L_tmp2x,L_tmp3,L_tmp1;
    Word16 exp,exp2,Q_speech;
    Word16 pd_fx[BANDS_MAX], rand_fx, peak_fx[BANDS_MAX], fac_fx;

    Word16 tmp,tmpx,tmp1,tmp2,tmp3,tmp4,Q_env_fx[BANDS_MAX],Q_Ep_fx[BANDS_MAX];

    Word16 Qs=SWB_BWE_LR_Qs;
    Word32 env_fx[BANDS_MAX];
    Word16 env_fx2[BANDS_MAX];
    Word32 ni_gain_fx[BANDS_MAX];
    Word16 y2hat_fx[L_FRAME48k];

    Word16 i, j, k, ni_end_band, satur, count[BANDS_MAX], max_pos_pulse, pos;
    Word16 sb = bands;

    satur = 0;
    move16();

    FOR(i = 0 ; i < bands; i++)
    {
        Ep_fx[i] = L_shl(Ep_fx[i], 6);/* Q-6 -> Q0 */  move32();
    }

    tmp = add(band_end[bands-1], 1);
    FOR (k = 0; k < tmp; k++)
    {
        y2hat_fx[k] = (Word16)L_min(L_max(L_shr(L_y2[k],Qs),-32768),32767);
        move16(); /* Extract_l or something else is missing here */
    }

    test();
    test();
    test();
    test();
    IF( (sub(hqswb_clas,HQ_HARMONIC) ==0  || sub(hqswb_clas,HQ_NORMAL) ==0  ) && (L_sub(bwe_br,HQ_16k40) ==0  || L_sub(bwe_br,HQ_13k20) ==0 ) && sub(bwidth,SWB) ==0  )
    {
        sb = (L_sub(bwe_br,HQ_16k40) == 0) ? 19 : 17;
    }

    /* calculate the envelopes/ the decoded peak coeff./number of the decoded coeff./ the last subbands of the bit-allocated/saturation of bit-allocation */
    ni_end_band = bands;
    max_pos_pulse = bands;
    FOR (k = ni_start_band; k < ni_end_band; k++)
    {
        tmp = div_s(1, band_width[k]);             /*Q15 */
        L_tmp = Mult_32_16(Rk_fx[k],tmp);/*Q(16+15-15=16) */
        pd_fx[k] = extract_h(L_shl(L_tmp,10));    /*16+10-16 =Q10 */

        L_tmp2 = L_add(0,Ep_fx[k]);/*Q0 */
        L_tmp = L_max(1, L_tmp2);
        exp = norm_l(L_tmp);
        tmp = extract_h(L_shl(L_tmp, exp));

        L_tmp3 = L_add(0,(Word32)band_width[k]);
        exp2 = norm_l(L_tmp3);
        tmp2 = extract_h(L_shl(L_tmp3, exp2));

        exp2 = sub(exp, exp2); /* Denormalize and substract */

        tmp3 = sub(tmp2, tmp);
        if (tmp3 > 0)
        {
            tmp2 = shr(tmp2, 1);
        }
        if (tmp3 > 0)
        {
            exp2 = add(exp2, 1);
        }
        tmp = div_s(tmp2, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp2);
        env_fx[k] = L_tmp;
        move32();/*Q(31-exp2)   move32(); */
        Q_env_fx[k] = sub(31,exp2);
        move16();
        tmp = sub(17,Q_env_fx[k]);
        env_fx2[k] = extract_h(L_shl(env_fx[k],tmp));/*Q1 */
        peak_fx[k] = 0;
        move16();
        count[k] = 0;
        move16();

        IF(npulses[k] != 0)
        {
            FOR (i = band_start[k]; i <= band_end[k]; i++)
            {
                L_tmp =L_mult0(y2hat_fx[i],y2hat_fx[i]); /*0 */
                Ep_fx[k] =L_sub(Ep_fx[k],L_tmp);
                move32();/*0 */
                IF(sub(abs_s(y2hat_fx[i]),peak_fx[k]) > 0)
                {
                    peak_fx[k] = abs_s(y2hat_fx[i]);
                    move16();/*0 */
                }

                IF(y2hat_fx[i] != 0)
                {
                    count[k] = add(count[k],1);
                    move16();
                }
            }

            max_pos_pulse = k;
            L_tmp2 = L_add(0,Ep_fx[k]);
            L_tmp = L_max(1, L_tmp2);
            exp = norm_l(L_tmp);
            tmp = extract_h(L_shl(L_tmp, exp));

            L_tmp3 = (Word32)band_width[k];
            exp2 = norm_l(L_tmp3);
            tmp2 = extract_h(L_shl(L_tmp3, exp2));

            exp2 = sub(exp, exp2); /* Denormalize and substract  */

            tmp3 = sub(tmp2, tmp);
            if (tmp3 > 0)
            {
                tmp2 = shr(tmp2, 1);
            }
            if (tmp3 > 0)
            {
                exp2 = add(exp2, 1);
            }
            tmp = div_s(tmp2, tmp);
            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp2);
            Ep_fx[k] = L_tmp;
            move32();/*Q(31-exp2) */
            Q_Ep_fx[k] = sub(31,exp2);
            move16();
        }
        ELSE
        {
            Ep_fx[k] = env_fx[k];
            move32();/*Q(Q_env_fx[k]) */
            Q_Ep_fx[k] = Q_env_fx[k];
            move16();/*31-exp2          */
        }
    }

    FOR(k = ni_start_band; k < ni_end_band; k++)
    {
        /* calculate the noise gain */
        satur =0;
        move16();
        if(sub(pd_fx[k],819)>= 0)
        {
            satur =1;
            move16();
        }

        test();
        IF (satur == 0 && Ep_fx[k] > 0)
        {
            IF(npulses[k] != 0)
            {
                IF( sub(bwidth,SWB) ==0)
                {
                    IF(sub(hqswb_clas,HQ_TRANSIENT) !=0 )
                    {
                        IF(peak_fx[k]!=0)
                        {
                            Q_speech = norm_s(peak_fx[k]);
                            tmp = shl(peak_fx[k],Q_speech);/*Q(Q_speech) */
                            tmp = div_s(16384,tmp);/*Q(15+14-Q_speech) */
                        }
                        ELSE
                        {
                            tmp = 0x7fff;
                            move16();
                            Q_speech = 0;
                            move16();
                        }
                        L_tmp2x = Mult_32_16(Ep_fx[k],tmp);/* Q(Q_Ep_fx[k]+29-Q_speech-15 = Q_Ep_fx[k]-Q_speech+14) */
                        tmp = sub(Q_Ep_fx[k],Q_speech);
                        tmpx = add(tmp,1);
                        tmp2 = extract_l(L_shr(L_tmp2x,tmpx));/*Q13 Ep[k]/peak[k] */

                        IF(sub(hqswb_clas,HQ_HARMONIC) == 0 )
                        {
                            tmp = sub(1536,pd_fx[k]); /*Q10 */
                            tmp3 = shl(tmp,4); /*Q14 */
                            L_tmp = Mult_32_16(env_fx[k],tmp3);/*Q(Q_env_fx[k]+14-15 = Q_env_fx[k]-1) */
                            L_tmp = Mult_32_16(L_tmp,6144);/*Q(Q_env_fx[k]-1+10-15 = Q_env_fx[k]-6) */

                            IF(peak_fx[k]!=0)
                            {
                                Q_speech = norm_s(peak_fx[k]);
                                tmp = shl(peak_fx[k],Q_speech);/*Q(Q_speech) */
                                tmp = div_s(16384,tmp);/*Q(15+14-Q_speech) */
                            }
                            ELSE
                            {
                                tmp = 0x7fff;
                                move16();
                                Q_speech = 0;
                                move16();
                            }

                            L_tmp2 = Mult_32_16(Ep_fx[k],tmp);/* Q(Q_Ep_fx[k]+29-Q_speech-15=Q_Ep_fx[k]-Q_speech+14) */
                            L_tmp3 = Mult_32_16(L_tmp,tmp);/* Q(Q_env_fx[k]-6+29-Q_speech-15=Q_env_fx[k]-Q_speech+8) */
                            L_tmp = Mult_32_32(L_tmp2,L_tmp3); /*Q(Q_Ep_fx[k]-Q_speech+14+Q_env_fx[k]-Q_speech+8-31=Q_Ep_fx[k]+Q_env_fx[k]-2*Q_speech-9) */

                            tmp = add(Q_Ep_fx[k],Q_env_fx[k]);
                            tmp = sub(tmp,Q_speech);
                            tmp = sub(tmp,Q_speech);
                            tmp = sub(37,tmp);
                            tmp1= extract_h(L_shl(L_tmp,tmp));/*Q12 //6.0f*(1.5f - pd[k])*env[k]*Ep[k]/(peak[k]*peak[k])  */

                            fac_fx = tmp1;
                            move16();/*Q12  */
                            if(sub(k,sb) > 0)
                            {
                                fac_fx =mult(24576,tmp2);/*//Q(14+13-15=12) */
                            }
                        }
                        ELSE
                        {
                            IF(sub(k,sb) <= 0)
                            {
                                tmp = sub(1536,pd_fx[k]); /*Q10 */
                                tmp3 = shl(tmp,4); /*Q14 */
                                L_tmp = Mult_32_16(L_tmp2x,tmp3);/*Q(Q_Ep_fx[k]-Q_speech+14+14-15 = Q_Ep_fx[k]-Q_speech+13) */
                                L_tmp = Mult_32_16(L_tmp,20480);/*Q(Q_Ep_fx[k]-Q_speech+13+12-15 = Q_Ep_fx[k]-Q_speech+10) */
                                fac_fx= extract_h(L_shl(L_tmp,sub(add(18,Q_speech),Q_Ep_fx[k])));/*Q_Ep_fx[k]-Q_speech+10 +18+Q_speech-Q_Ep_fx[k] -16 =12 */
                            }
                            ELSE
                            {
                                fac_fx =shl(mult(32767,tmp2),1);/*//Q(13+13-15+1=12) */
                            }
                        }
                    }
                    ELSE
                    {
                        fac_fx = 4505;
                        move16();/*Q12 */
                    }
                }
                ELSE
                {
                    tmp = sub(1536,pd_fx[k]); /*Q10 */
                    tmp2 = min(1024,tmp); /*q10 */
                    tmp2 = shl(tmp2,4); /*Q14 */
                    L_tmp = Mult_32_16(env_fx[k],tmp2);/*Q(Q_env_fx[k]+14-15 = Q_env_fx[k]-1) */
                    L_tmp = Mult_32_16(L_tmp,20480);/*Q(Q_env_fx[k]-1+10-15 = Q_env_fx[k]-6) */

                    IF(peak_fx[k]!=0)
                    {
                        Q_speech = norm_s(peak_fx[k]);
                        tmp = shl(peak_fx[k],Q_speech);/*Q(Q_speech) */
                        tmp = div_s(16384,tmp);/*Q(15+14-Q_speech) */
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        move16();
                        Q_speech = 0;
                        move16();
                    }

                    L_tmp2 = Mult_32_16(Ep_fx[k],tmp);/* Q(Q_Ep_fx[k]+29-Q_speech-15=Q_Ep_fx[k]-Q_speech+14) */
                    L_tmp3 = Mult_32_16(L_tmp,tmp);/* Q(Q_env_fx[k]-6+29-Q_speech-15=Q_env_fx[k]-Q_speech+8) */
                    L_tmp = Mult_32_32(L_tmp2,L_tmp3); /*Q(Q_Ep_fx[k]-Q_speech+14+Q_env_fx[k]-Q_speech+8-31=Q_Ep_fx[k]+Q_env_fx[k]-2*Q_speech-9) */

                    tmp = add(Q_Ep_fx[k],Q_env_fx[k]);
                    tmp = sub(tmp,Q_speech);
                    tmp = sub(tmp,Q_speech);
                    tmp = sub(37,tmp);

                    fac_fx = extract_h(L_shl(L_tmp,tmp));/*Q12 */

                    test();
                    IF(sub(k,1) > 0 && sub(k,sub(ni_end_band,1)) < 0)
                    {
                        IF(env_fx2[k]!=0)
                        {
                            Q_speech = norm_s(env_fx2[k]);
                            tmp = shl(env_fx2[k],Q_speech);/*Q(Q_speech+1) */
                            tmp = div_s(16384,tmp);/*Q(15+14-Q_speech-1=28-Q_speech) */
                            Q_speech = sub(28,Q_speech);
                        }
                        ELSE
                        {
                            tmp =0x7fff;
                            move16();
                            Q_speech = 0;
                            move16();
                        }
                        tmp1 = mult(env_fx2[add(k,1)],16384);/*Q(1+15-15=1) Q1 */
                        tmp2 = sub(env_fx2[k],tmp1);
                        tmp1 = mult(env_fx2[k],16384);/*Q(1+15-15=1) Q1 */
                        tmp3 = sub(tmp1,env_fx2[sub(k,1)]);
                        tmp1 = mult(peak_fx[k],16384);/*Q(0+15-15=0) Q0 */
                        tmp4 = sub(tmp1,shr(env_fx2[k],1));
                        test();
                        test();
                        test();
                        IF(count[add(k,1)] == 0 && tmp2 > 0 && tmp3 < 0)
                        {
                            L_tmp = L_mult(env_fx2[add(k,1)],tmp);/* Q(1+Q_speech+1 = Q_speech+2) */
                            L_tmp = Mult_32_16(L_tmp,24576); /*Q(Q_speech+2+14-15=Q_speech+1) */
                            fac_fx = extract_h(L_shl(L_tmp,sub(27,Q_speech)));/*Q12 */
                        }
                        ELSE IF(count[sub(k,1)] == 0 && tmp4 > 0)
                        {
                            L_tmp = L_mult(env_fx2[sub(k,1)],tmp); /* Q(1+Q_speech+1 = Q_speech+2) */
                            fac_fx = extract_h(L_shl(L_tmp,sub(26,Q_speech)));/*Q12 */
                        }
                    }

                    IF(sub(k,sub(ni_end_band,p2a_bands)) >= 0)
                    {
                        L_tmp = Mult_32_16(enerH_fx, bw_low);
                        L_tmp2= Mult_32_16(enerL_fx, bw_high);
                        L_tmp = L_sub(L_tmp,L_tmp2);
                        tmp1 = mult(peak_fx[k],16384);/*Q(0+15-15=0) Q0 */
                        tmp4 = sub(tmp1,shr(env_fx2[k],1));
                        test();
                        IF(L_tmp > 0 && tmp4 < 0)
                        {
                            IF(peak_fx[k]!=0)
                            {
                                Q_speech = norm_s(peak_fx[k]);
                                tmp = shl(peak_fx[k],Q_speech);/*Q(Q_speech) */
                                tmp = div_s(16384,tmp);/*Q(15+14-Q_speech) */
                            }
                            ELSE
                            {
                                tmp = 0x7fff;
                                move16();
                                Q_speech = 0;
                                move16();
                            }
                            L_tmp2 = Mult_32_16(Ep_fx[k],tmp);/* Q(Q_Ep_fx[k]+29-Q_speech-15 = Q_Ep_fx[k]-Q_speech+14) */
                            tmp = sub(Q_Ep_fx[k],Q_speech);
                            tmp = add(tmp,1);
                            tmp = extract_l(L_shr(L_tmp2,tmp));/*Q13 */
                            tmp = sub(16384,tmp);/*Q13 */
                            fac_fx = extract_h(L_shl(L_mult(fac_fx,tmp),2));/*Q12*/
                        }

                        IF(p2a_flags[k] == 0)
                        {
                            L_tmp2 = Mult_32_16(Ep_fx[k],fac_fx);/*Q(Q_Ep_fx[k]+12-15 = Q_Ep_fx[k]-3) */
                            Q_speech = norm_l(L_tmp2);
                            tmp = extract_h(L_shl(L_tmp2,Q_speech));/*Q(Q_Ep_fx[k]-3+Q_speech-16 = Q_Ep_fx[k]+Q_speech-19) */
                            IF(tmp != 0)
                            {
                                tmp = div_s(16384,tmp);/*Q(15+14-Q_Ep_fx[k]-Q_speech+19 = 48-Q_Ep_fx[k]-Q_speech) */
                                L_tmp2 = Mult_32_16(env_fx[k],tmp);/*Q(Q_env_fx[k]+48-Q_Ep_fx[k]-Q_speech-15 = Q_env_fx[k]-Q_Ep_fx[k]-Q_speech+33) */
                                L_tmp2 = Mult_32_16(L_tmp2,20480);/*Q(Q_env_fx[k]-Q_Ep_fx[k]-Q_speech+33+14-15 = Q_env_fx[k]-Q_Ep_fx[k]-Q_speech+32) */
                                tmp = sub(Q_env_fx[k],Q_Ep_fx[k]);
                                tmp = sub(tmp,Q_speech);
                                tmp = add(tmp,25);
                                L_tmp = L_shr(L_tmp2,tmp);/*Q7 */
                                tmp = extract_l(L_min(L_tmp,192));/* */
                                fac_fx = extract_h(L_shl(L_mult(fac_fx,tmp),8));/*Q12 */
                            }
                            ELSE
                            {
                                tmp = 0x7fff;/*Q0 */
                                L_tmp2 = Mult_32_16(env_fx[k],tmp);/*Q(Q_env_fx[k]+0-15 = Q_env_fx[k]-15) */
                                L_tmp2 = Mult_32_16(L_tmp2,20480);/*Q(Q_env_fx[k]-15+14-15 = Q_env_fx[k]-16) */
                                tmp = sub(Q_env_fx[k],23);
                                L_tmp = L_shr(L_tmp2,tmp);/*Q7 */
                                tmp = extract_l((L_min(L_tmp,192)));/* */
                                fac_fx = extract_h(L_shl(L_mult(fac_fx,tmp),8));/*Q12 */
                            }
                        }
                    }
                }
            }
            ELSE
            {
                fac_fx = (hqswb_clas == HQ_HARMONIC && bwidth == SWB) ? 3277 : 4505;
            }

            L_tmp = Mult_32_16(Ep_fx[k],fac_fx);/*Q(Q_Ep_fx[k]+12-15 = Q_Ep_fx[k]-3) */
            ni_gain_fx[k] = L_shr(L_tmp,sub(Q_Ep_fx[k],20));
            move32();/*Q17 */
        }
        ELSE
        {
            ni_gain_fx[k] = L_deposit_l(0);
        }

        /* smooth the noise gain between the current frame and the previous frame */
        pos = sub(bwidth,SWB) ==0 ? sub(ni_end_band,1) : max(max_pos_pulse, *last_max_pos_pulse);

        IF(sub(k,pos) <=0 )
        {
            test();
            IF(k > 0 && add(sub(k,ni_end_band),1) < 0)
            {
                tmp1 = mult(last_env_fx[k],16384);/*Q(1+15-15=1) Q1 */
                tmp2 = sub(env_fx2[k],tmp1);/*>0 */
                tmp1 = mult(env_fx2[k],16384);/*Q(1+15-15=1) Q1 */
                tmp3 = sub(tmp1,last_env_fx[k]);/*<0 */
                L_tmp = L_add((Word32)env_fx2[k],(Word32)env_fx2[sub(k,1)]);
                L_tmp = L_add(L_tmp,(Word32)env_fx2[add(k,1)]);/*Q1 */
                L_tmp1 = L_add((Word32)last_env_fx[k],(Word32)last_env_fx[sub(k,1)]);
                L_tmp1 = L_add(L_tmp1,(Word32)last_env_fx[add(k,1)]);/*Q1 */
                L_tmp2 = Mult_32_16(L_tmp1,16384);/*Q(1+15-15) Q1 */
                L_tmp2 = L_sub(L_tmp,L_tmp2);/*>0 */
                L_tmp3 = Mult_32_16(L_tmp,16384);/*Q(1+15-15) Q1 */
                L_tmp3 = L_sub(L_tmp3,L_tmp1);/*<0 */
                test();
                test();
                test();
                IF( (tmp2 > 0 && tmp3 < 0) ||(L_tmp2 > 0 && L_tmp3 < 0))
                {
                    IF( L_sub(ni_gain_fx[k],last_ni_gain_fx[k]) > 0 )
                    {
                        L_tmp = Mult_32_16(ni_gain_fx[k],6554);/*Q(17+15-15 = 17) */
                        L_tmp1 = Mult_32_16(last_ni_gain_fx[k],26214);/*Q17 */
                        ni_gain_fx[k] = L_add(L_tmp,L_tmp1);
                        move32();
                    }
                    ELSE
                    {
                        L_tmp = Mult_32_16(ni_gain_fx[k],19661);/*Q(17+15-15 = 17) */
                        L_tmp1 = Mult_32_16(last_ni_gain_fx[k],13107);/*Q17 */
                        ni_gain_fx[k] = L_add(L_tmp,L_tmp1);
                        move32();
                    }
                }
            }
            ELSE IF (add(sub(k,ni_end_band),1) == 0)
            {
                tmp1 = mult(last_env_fx[k],16384);/*Q(1+15-15=1) Q1 */
                tmp2 = sub(env_fx2[k],tmp1);/*>0 */
                tmp1 = mult(env_fx2[k],16384);/*Q(1+15-15=1) Q1 */
                tmp3 = sub(tmp1,last_env_fx[k]);/*<0 */
                L_tmp = L_add((Word32)env_fx2[k],(Word32)env_fx2[sub(k,1)]);/*Q1 */
                L_tmp1 = L_add((Word32)last_env_fx[k],(Word32)last_env_fx[sub(k,1)]);/*Q1 */
                L_tmp2 = Mult_32_16(L_tmp1,16384);/*Q(1+15-15) Q1 */
                L_tmp2 = L_sub(L_tmp,L_tmp2);/*>0 */
                L_tmp3 = Mult_32_16(L_tmp,16384);/*Q(1+15-15) Q1 */
                L_tmp3 = L_sub(L_tmp3,L_tmp1);/*<0 */

                test();
                test();
                test();
                IF( (tmp2 > 0 && tmp3 < 0) ||(L_tmp2 > 0 && L_tmp3 < 0))
                {
                    IF( L_sub(ni_gain_fx[k],last_ni_gain_fx[k]) > 0 )
                    {
                        L_tmp = Mult_32_16(ni_gain_fx[k],6554);/*Q(17+15-15 = 17) */
                        L_tmp1 = Mult_32_16(last_ni_gain_fx[k],26214);/*Q17 */
                        ni_gain_fx[k] = L_add(L_tmp,L_tmp1);
                        move32();
                    }
                    ELSE
                    {
                        L_tmp = Mult_32_16(ni_gain_fx[k],19661);/*Q(17+15-15 = 17) */
                        L_tmp1 = Mult_32_16(last_ni_gain_fx[k],13107);/*Q17 */
                        ni_gain_fx[k] = L_add(L_tmp,L_tmp1);
                        move32();
                    }
                }
            }
        }

        /* inject noise into the non-decoded coeffs */
        test();
        test();
        IF(add(sub(k,ni_end_band),p2a_bands) >=0 && p2a_flags[k] == 0 && sub(bwidth,SWB) !=0 )
        {
            FOR (i = band_start[k]; i <= band_end[k]; i++)
            {
                IF (L_y2[i] != 0)
                {
                    L_y2[i] = Mult_32_16(L_y2[i],26215);
                    move32();/*Q(12+15-15=12) */
                }
            }
        }

        test();
        test();
        test();
        IF(sub(k,max_pos_pulse) == 0  && add(sub(k,bands),p2a_bands)< 0 && sub(satur,1) != 0 && sub(bwidth,SWB) !=0)
        {
            j = 0;
            Q_speech = norm_l(ni_gain_fx[k]);
            tmp = extract_h(L_shl(ni_gain_fx[k],Q_speech));/*Q(Q_speech+1) */
            IF(tmp != 0)
            {
                tmp = div_s(16384,tmp);/*Q(15+14-Q_speech-1 = 28-Q_speech) */
                L_tmp = Mult_32_16(Ep_fx[k],tmp);    /*Q(Q_Ep_fx[k]+28-Q_speech-15 = Q_Ep_fx[k]+13-Q_speech) */
                tmp = sub(Q_Ep_fx[k],Q_speech);
                tmp = sub(15,tmp);
                tmp = extract_h(L_shl(L_tmp,tmp));/*Q12 */
            }
            ELSE
            {
                tmp = 0x7fff;/*Q0 */
                L_tmp = Mult_32_16(Ep_fx[k],tmp);    /*Q(Q_Ep_fx[k]+0-15 = Q_Ep_fx[k]-15) */
                tmp = sub(43,Q_Ep_fx[k]);
                tmp = extract_h(L_shl(L_tmp,tmp));/*Q12 */
            }
            fac_fx =max(tmp,4096);/*Q12 */

            FOR (i = band_start[k]; i <= band_end[k]; i++)
            {
                IF (L_y2[i] == 0)
                {
                    rand_fx = Random(&ni_seed);  /*Q15 */
                    IF(band_width[k] != 0)
                    {
                        Q_speech = norm_s(band_width[k]);
                        tmp = shl(band_width[k],Q_speech);/*Q(Q_speech) */
                        tmp = div_s(16384,tmp);/*Q(15+14-Q_speech) */
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        Q_speech = 0;
                    }
                    tmp1 = sub(fac_fx,4096);/*Q12 */
                    L_tmp = L_mult(tmp1,j);/*Q13 */
                    L_tmp = Mult_32_16(L_tmp,tmp);/*Q(13+29-Q_speech-15 = 27-Q_speech) */
                    tmp = extract_h(L_shl(L_tmp,add(1,Q_speech)));/*Q12 */
                    tmp = sub(fac_fx,tmp);/*Q12 */
                    L_tmp = Mult_32_16(ni_gain_fx[k],tmp);/*Q(17+12-15=14) */
                    L_y2[i] = L_add(L_y2[i],L_shr(Mult_32_16(L_tmp,rand_fx),2));
                    move32();/*Q12                   */
                }
                j=add(j,1);
            }
        }
        ELSE
        {
            FOR (i = band_start[k]; i <= band_end[k]; i++)
            {
                IF (L_y2[i] == 0)
                {
                    rand_fx = Random(&ni_seed);  /*Q15 */
                    L_tmp = Mult_32_16(ni_gain_fx[k],rand_fx);/*Q(17+15-15=17) */
                    L_y2[i] = L_add(L_y2[i],L_shr(L_tmp,5));
                    move32();/*Q12 */
                }
            }
        }
    }

    Copy(env_fx2,last_env_fx,ni_end_band);
    Copy32(ni_gain_fx,last_ni_gain_fx,ni_end_band);
    *last_max_pos_pulse = max_pos_pulse;
    move16();
    return;
}
