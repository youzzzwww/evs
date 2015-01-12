/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

#define THR_CORR_FX       (56<<15)      /* 56 in Q15 starting threshold of multi-harm. correlation */
#define THR_CORR_MAX_FX   30720         /* 60 in Q9 upper threshold of multi-harm. correlation */
#define THR_CORR_MIN_FX   25088         /* 49 in Q9 lower threshold of multi-harm. correlation */
#define THR_CORR_STEP_FX  102           /* 0.2 in Q9 step for the threshold of multi-harm. correlation */

/*---------------------------------------------------------------------*
 * multi_harm()
 *
 * Perform multi-harmonic analysis, information used for UV and VAD decision
 *---------------------------------------------------------------------*/

Word16 multi_harm_fx(						/* o  : frame multi-harmonicity (1-harmonic, 0-not)   		*/
    const Word16 Bin_E[],					/* i  : log-energy spectrum of the current frame      Q7	*/
    Word16 old_S[],					/* i/o: prev. log-energy spectrum w. subtracted floor Q7	*/
    Word16 cor_map_LT[],			/* i/o: LT correlation map                            Q15	*/
    Word16 *multi_harm_limit,		/* i/o: multi harminic threshold                      Q9	*/
    const Word32 total_brate,             /* i  : total bitrate             					  Q0	*/
    const Word16 bwidth,                  /* i  : input signal bandwidth                        Q0    */
    Word16 *cor_strong_limit,		/* i/o: HF correlation indicator  					  Q0	*/
    Word16 *st_mean_avr_dyn,		/* i/o: long term average dynamic 					  Q7	*/
    Word16 *st_last_sw_dyn,			/* i/o: last dynamic              					  Q7	*/
    Word16 *cor_map_sum				/*													  Q8	*/
    , Word16 *sp_floor                /* o: noise floor estimate                            Q7    */
)
{
    Word16 i, j, k, L, stemp, N_mins, ind_mins[L_FFT/4], *pt_mins, harm;
    Word16 S[L_FFT/2], flor, step, sign_fx, tmp16, tmp2, ExpInd, tmpdB, ExpdB, Expx2, Expy2;
    Word16 corx2, cory2, corxy, cor, cor_map[L_FFT/2], *pt1, *pt2, cor_strong;
    Word32 L_acc;
    Word32 Lcorx2, Lcory2, Lcorxy, Lcor_map_LT_sum;
    Word16 mean_dyn;

    /*------------------------------------------------------------------*
     * initialization
     *------------------------------------------------------------------*/

    /* length of the useful part of the spectrum (up to 6.4kHz) */
    L = L_FFT/2;
    move16();
    if ( sub(bwidth,NB) == 0 )
    {
        /* length of the useful part of the spectrum (up to 3.6kHz) */
        L = 76;
        move16();
    }

    Copy(Bin_E, S, L);

    /*------------------------------------------------------------------*
     * searching of spectral maxima and minima
     *------------------------------------------------------------------*/

    pt_mins = ind_mins;
    move16();

    /* index of the first minimum */
    if (sub(Bin_E[0],Bin_E[1]) < 0)
    {
        *pt_mins++ = 0;
        move16();
    }

    FOR (i=1; i<L-1; i++)
    {
        /* minimum found */
        test();
        if ( sub(Bin_E[i], Bin_E[i-1]) < 0 && sub(Bin_E[i], Bin_E[i+1]) < 0 )
        {
            *pt_mins++ = i;
            move16();
        }
    }

    /* index of the last minimum */
    if (sub(Bin_E[L-1],Bin_E[L-2]) < 0)
    {
        *pt_mins++ = L-1;
        move16();
    }

    /* total number of minimas found */
    N_mins = (Word16)(pt_mins - ind_mins - 1);
    move16();

    /*------------------------------------------------------------------*
     * calculation of the spectral floor
     * subtraction of the spectral floor
     *------------------------------------------------------------------*/

    set16_fx(S, 0, L);

    IF (N_mins > 0)
    {
        L_acc = L_deposit_l(0);
        FOR (i=0; i<N_mins; ++i)
        {
            L_acc = L_mac0(L_acc, Bin_E[ind_mins[i]], 1);
        }
        *sp_floor = extract_l(Mult_32_16(L_acc, div_s(1, N_mins)));

        set16_fx( S, 0, ind_mins[0]);
        set16_fx( &S[ind_mins[N_mins]], 0, sub(shr(L_FFT,1),ind_mins[N_mins]));

        pt_mins = ind_mins;
        move16();
        flor = 0;
        move16();
        step = 0;
        move16();

        FOR (i=ind_mins[0]; i<ind_mins[N_mins]; i++)
        {
            /* we are at the end of the next minimum */
            IF (sub(i,*pt_mins) == 0)
            {
                pt_mins++;
                flor = Bin_E[i];
                move16();/*Q7*/
                /* calculate the new step */
                /*step = (Bin_E[*pt_mins] - Bin_E[i]) / (*pt_mins-i);*/
                tmp16 = sub(*pt_mins, i);
                tmpdB = sub(Bin_E[*pt_mins], Bin_E[i]);
                sign_fx =  shr(tmpdB, 15); /* 0 if positive else -1 */
                ExpdB = sub(norm_s(tmpdB), 1);
                tmpdB = abs_s(shl(tmpdB, ExpdB));
                ExpInd = norm_s(tmp16);
                tmp16 = shl(tmp16, ExpInd);
                tmp16 = div_s(tmpdB, tmp16);
                tmp16 = sub(s_xor(tmp16, sign_fx), sign_fx);
                step =  shr(tmp16, add(sub(ExpdB, ExpInd), 15));    /* Q7 */
            }

            /* subtract the floor */
            S[i] = s_max(sub(Bin_E[i],flor), 0);
            move16();

            /* update the floor */
            flor = add(flor,step);/*Q7*/
        }
    }

    /* Calculate the maximum dynamic per band */
    /* since we are processing 40 bins we will use 1/40 in Q15 to find the mean */
    /* mean_dyn = mean(&S[L-40], 40);*/
    L_acc = L_deposit_l(0);
    FOR(i=L-40; i<L; i++)
    {
        L_acc = L_mac(L_acc,S[i],819);
    }
    mean_dyn = round_fx(L_acc); /*Q7*/

    /*mean_dyn = 0.6f * *st_mean_avr_dyn + 0.4f * mean_dyn;*/
    L_acc = L_mult(13107/*0.4f*/,mean_dyn);/*Q23*/
    L_acc = L_mac(L_acc,19661/*0.6f*/,*st_mean_avr_dyn);/*Q23*/
    mean_dyn = round_fx(L_acc);/*Q7*/

    test();
    IF ( sub(mean_dyn,1229) < 0/*9.6f*/ && *cor_strong_limit != 0 )
    {
        *cor_strong_limit = 0;
        move16();
        *st_last_sw_dyn = mean_dyn;
        move16();
    }
    ELSE IF (sub(sub(mean_dyn , *st_last_sw_dyn),576) > 0/*4.5f*/)
    {
        *cor_strong_limit = 1;
        move16();
    }
    test();
    if( L_sub(total_brate,ACELP_9k60) < 0|| L_sub(total_brate,ACELP_16k40) > 0 )
    {
        *cor_strong_limit = 1;
        move16();
    }

    *st_mean_avr_dyn = mean_dyn;
    move16();

    /*------------------------------------------------------------------*
     * calculation of the correlation map
     *------------------------------------------------------------------*/

    set16_fx(cor_map, 0,L);
    IF (N_mins > 0)
    {
        Lcorx2 = L_deposit_l(0);
        Lcorxy = L_deposit_l(0);
        stemp = ind_mins[0];
        move16();
        Lcory2 = L_mult(old_S[stemp],old_S[stemp]);
        k = 1;
        move16();

        FOR (i = add(stemp,1); i <= ind_mins[N_mins]; i++)
        {
            IF (sub(i,ind_mins[k]) == 0)
            {
                /* include the last peak point (new minimum) to the corr. sum */
                Lcory2 = L_mac(Lcory2, old_S[i], old_S[i]);

                /* calculation of the norm. peak correlation */
                test();
                IF (Lcorx2 != 0 && Lcory2 != 0)
                {
                    /* corxy * corxy*/
                    tmp16 = sub(norm_l(Lcorxy),1);
                    corxy = extract_h(L_shl(Lcorxy, tmp16));
                    corxy = mult_r(corxy, corxy);
                    /* (corx2 * cory2) */
                    Expx2 = norm_l(Lcorx2);
                    Expy2 = norm_l(Lcory2);
                    corx2 = extract_h(L_shl(Lcorx2, Expx2));
                    cory2 = extract_h(L_shl(Lcory2, Expy2));
                    corx2 = mult_r(corx2, cory2);
                    Expx2 = add(Expy2, Expx2);
                    /* Validate num < den */
                    cor = sub(corx2, corxy);
                    cor = shr(cor, 15);
                    /* Add 1 to tmp16 & shr by 2 if corxy > corx2 */
                    tmp16 = sub(tmp16, cor);
                    corxy = shl(corxy, cor);
                    corxy = shl(corxy, cor);
                    /* cor = corxy * corxy / (corx2 * cory2) */
                    corxy = div_s(corxy, corx2);
                    cor = shr(corxy, sub(shl(tmp16, 1), Expx2));  /* Q15 */
                }
                ELSE
                {
                    cor = 0;
                    move16();
                }

                /* save the norm. peak correlation in the correlation map */
                FOR (j=ind_mins[k-1]; j<ind_mins[k]; j++)
                {
                    old_S[j] = S[j];
                    move16();
                    S[j] = shr(cor,8);
                    move16();
                    cor_map[j] = cor;
                    move16();
                }

                Lcorx2 = L_deposit_l(0);
                Lcory2 = L_deposit_l(0);
                Lcorxy = L_deposit_l(0);

                k++;
            }

            Lcorx2 = L_mac(Lcorx2, S[i], S[i]);
            Lcory2 = L_mac(Lcory2, old_S[i], old_S[i]);
            Lcorxy = L_mac(Lcorxy, S[i], old_S[i]);
        }

        Copy( S, old_S, ind_mins[0]);
        Copy( &S[ind_mins[N_mins]], &old_S[ind_mins[N_mins]], sub(L , ind_mins[N_mins]));
    }
    ELSE
    {
        *sp_floor = Bin_E[0];
        move16();
    }
    *sp_floor = mult(*sp_floor, 14231);
    move16(); /* Convert to log10() */

    /*------------------------------------------------------------------*
     * updating of the long-term correlation map
     * summation of the long-term correlation map
     *------------------------------------------------------------------*/

    Lcor_map_LT_sum = L_deposit_l(0);
    tmp2 = 0;
    move16();

    cor_strong = 0;
    move16();
    pt1 = cor_map_LT;
    move16();
    pt2 = cor_map;
    move16();
    FOR (i=0; i<L; i++)
    {
        /* tmp2 += S[i]; */
        tmp2 = add(tmp2, shl(S[i],1)); /* tmp2 in Q8; max value is 128) */

        /* *pt1 = M_ALPHA_FX * *pt1 + (1-M_ALPHA_FX) * *pt2++ */
        *pt1 = mac_r(L_mult(ONE_MINUS_M_ALPHA, *pt2), M_ALPHA_FX, *pt1);
        move16();

        /* cor_map_LT_sum += *pt1 */
        Lcor_map_LT_sum = L_add(Lcor_map_LT_sum, *pt1); /* cor_map_LT_sum in Q15; max value is 128) */

        if(sub(*pt1, 31130) > 0/*0.95f*/)
        {
            cor_strong = 1;
            move16();
        }

        pt1++;
        pt2++;
    }

    IF ( sub(bwidth,NB) == 0 )
    {
        /* cor_map_LT_sum *= 1.53f; */
        /* tmp2 *= 1.53f; */
        Lcor_map_LT_sum = L_shl(Mult_32_16(Lcor_map_LT_sum, 25068), 1);
        tmp2 = round_fx(L_mac(L_mult(tmp2, 32767), tmp2, 17367));
    }
    *cor_map_sum = tmp2;
    move16();

    /* final decision about multi-harmonicity */
    harm = 0;
    move16();
    test();
    if ( (L_msu0(Lcor_map_LT_sum, *multi_harm_limit, 64) > 0) || (cor_strong != 0) )
    {
        harm = 1;
        move16();
    }

    /*------------------------------------------------------------------*
     * updating of the decision threshold
     *------------------------------------------------------------------*/

    stemp = add(*multi_harm_limit, THR_CORR_STEP_FX);
    if (L_sub(Lcor_map_LT_sum, THR_CORR_FX) > 0)    /* Q15 */
    {
        /* *multi_harm_limit -= THR_CORR_STEP_FX */
        stemp = sub(*multi_harm_limit, THR_CORR_STEP_FX);
    }

    stemp = s_min(stemp, THR_CORR_MAX_FX);
    *multi_harm_limit = s_max(stemp, THR_CORR_MIN_FX);
    move16();

    IF (N_mins <= 0)
    {
        set16_fx(old_S, 0, L);
    }

    return harm;
}
