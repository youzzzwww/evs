/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "stat_enc_fx.h" /* Static table prototypes                */
#include "stl.h"


#define RATEWIN         600     /* length of the rate control window. This is 600 active speech frames. This equals roughly 12s of active speech */

/*=================================================================================*/
/* FUNCTION      :  update_average_rate_fx										   */
/*---------------------------------------------------------------------------------*/
/* PURPOSE       :  SC-VBR update average data rate								   */
/*---------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                              */
/*   _ (struct DTFS_STRUCTURE_FX)												   */
/*---------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                        */
/*  st_fx->global_avr_rate_fx										 Q13		   */
/*	st_fx->sum_of_rates_fx											 Q13		   */
/*	st_fx->SNR_THLD_fx												 Q8			   */
/*	st_fx->Q_to_F_fx												 Q0			   */
/*	st_fx->pattern_m_fx												 Q0			   */
/*	st_fx->rate_control_fx											 Q0			   */
/*---------------------------------------------------------------------------------*/
/*/* OUTPUT ARGUMENTS :															   */
/*                    _ None                                                       */
/*---------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                                      */
/*---------------------------------------------------------------------------------*/
/* CALLED FROM :																   */
/*=================================================================================*/
void update_average_rate_fx(
    Encoder_State_fx *st_fx           /* i/o: encoder state structure */
)
{
    Word32 avratetarg_fx;           /* target rate for next RATEWIN active frames */
    Word32 target_fx;               /* target set by VBR_ADR_MAX_TARGET*RATEWIN*10 */
    Word16 tmp;
    Word32 L_tmp;
    Word32 L_tmp1,L_tmp2;

    Word16 exp, recip, Qrecip;

    IF ( sub(st_fx->numactive_fx,RATEWIN) == 0 )  /* goes into rate control only the numactive ==RATEWIN. So rate control is triggered after each RATEWIN avtive frames */
    {
        /* after 1000 blocks of RATEWIN frames, we change the way we control the average rate by using
           st->global_avr_rate=0.99*st->global_avr_rate+0.01*st->sum_of_rates. This will avoid
           veriables growing indefinitely while providing a good long term average rate */

        IF ( L_sub(st_fx->frame_cnt_ratewin_fx,1000) < 0 )
        {
            st_fx->frame_cnt_ratewin_fx = add(st_fx->frame_cnt_ratewin_fx,1);

            /*st->global_avr_rate = (st->global_avr_rate * (st->global_frame_cnt-1) + st->sum_of_rates) / st->global_frame_cnt; */
            exp = norm_s(st_fx->frame_cnt_ratewin_fx);
            tmp = shl(st_fx->frame_cnt_ratewin_fx,exp);
            recip = div_s(16384,tmp);
            Qrecip = 15-(exp-14);

            IF(L_sub(st_fx->frame_cnt_ratewin_fx,1) >0)
            {
                tmp = div_s(sub(st_fx->frame_cnt_ratewin_fx,1),st_fx->frame_cnt_ratewin_fx); /*Q15*/
                L_tmp1 = Mult_32_16(st_fx->global_avr_rate_fx, tmp); /* Q13*Q15 = Q13 */

                L_tmp2 = Mult_32_16(st_fx->sum_of_rates_fx, recip); /*Q13*Qrecip = 13+Qrecip+1-16 = Qrecip-2 */

                st_fx->global_avr_rate_fx = L_add(L_tmp1, L_shl(L_tmp2,13-(Qrecip-2)));  /*Q13 */
            }
            ELSE
            {
                st_fx->global_avr_rate_fx = st_fx->sum_of_rates_fx;   /*handle the first frame*/
            }
            /* Q13 */
        }
        ELSE
        {
            /* st->global_avr_rate = 0.01f * st->sum_of_rates + 0.99f * st->global_avr_rate; */
            st_fx->global_avr_rate_fx = L_add(Mult_32_16(st_fx->sum_of_rates_fx,328),Mult_32_16(st_fx->global_avr_rate_fx,32441) ); /*Q13 */
        }


        IF ( st_fx->sum_of_rates_fx == 0 )
        {
            /* st->sum_of_rates = (float) (RATEWIN * VBR_ADR_MAX_TARGET * 10); */
            st_fx->sum_of_rates_fx = L_shl(L_mult0(RATEWIN ,VBR_ADR_MAX_TARGET_x10_Q1 ),12); /*Q13 */
        }

        /* target = VBR_ADR_MAX_TARGET * 10 * RATEWIN; */
        target_fx = L_shl(L_mult0(VBR_ADR_MAX_TARGET_x10_Q1,RATEWIN ),12);    /*Q13 */

        IF ( L_sub(target_fx,st_fx->global_avr_rate_fx) < 0 )  /* Action is taken to reduce the averge rate. Only initiated if the global rate > target rate */
        {
            /* Check the vad snr values to table the noisey/not noisey decision */

            test();
            IF ( sub(st_fx->SNR_THLD_fx , 17152) < 0 ) /*Q8  */ /* Currently in QFF mode. The bumpup thresholds are slightly relaxed for noisy speech. */
            {
                /*  Increase the threshold so the the bumpup procedure is done using the noisy thresholds.
                    Use 3.5 steps to quickly ramp up the rate control to reduce the settling time */

                /*  st->SNR_THLD += 3.5f;  */
                st_fx->SNR_THLD_fx = add(st_fx->SNR_THLD_fx , 896 ); /*Q8 */
            }
            ELSE IF ( st_fx->mode_QQF_fx == 0 && L_sub(st_fx->sum_of_rates_fx, target_fx)>0 ) /* Now SNR_THLD is in the max allowed. Sill the global average is higher and
                                                                          last RATEWIN frames have a higher agerage than the target rate. Now slightly
                                                                          more aggresive rate control is used by changing the mode to QQF. Still the
                                                                          same strict bumpups (more bumpups,higher rate) are used. */
            {
                /* Kick in QQF mode */
                st_fx->mode_QQF_fx = 1;
                move16();
            }
            ELSE IF ( L_sub(st_fx->sum_of_rates_fx , target_fx) > 0 )  /* Actions (1) and (2) are not sufficient to control the rate. Still the last RATEWIN active
                                                      frames have a higher average rate than the target rate. More aggresive rate control is
                                                      needed. At this point the rate_control flag is set. This will enable the more relaxed
                                                      bump up thresholds (less bump ups->reduced rate)*/
            {
                /* Relaxed bump ups are used */
                st_fx->rate_control_fx = 1;
                move16();
                /* This will be triggered only if the gloabl average rate is considerablly higher than the target rate.
                   Keep a higher threshold to avoid short term rate increases over the target rate. */
                IF ( L_sub(st_fx->global_avr_rate_fx ,L_add(target_fx,3440640)) > 0 ) /* Last resort rate control. This is a safer rate control mechanism by increasing NELPS */
                {
                    st_fx->Last_Resort_fx = 1;
                    move16();   /* compute based on a larger window as the last resort */
                }
                ELSE
                {
                    st_fx->Last_Resort_fx = 0;
                    move16();
                }
            }
            ELSE IF ( L_sub(st_fx->sum_of_rates_fx, target_fx ) < 0) /* If the average rate of last RATEWIN frames is controlled by above actions, disable the most
                                                     aggresive rate control mechanisms. Still keep QQF mode as the global rate is not under
                                                     the target rate*/
            {
                st_fx->Last_Resort_fx = 0;
                move16();
                st_fx->mode_QQF_fx = 1;
                move16();
                st_fx->rate_control_fx = 0;
                move16();
            }
        }
        ELSE
        {
            /* floding back to lesser and leser aggresive rate control mechanisms gradually if global rate is under control */
            st_fx->Last_Resort_fx = 0;
            move16();

            IF ( sub(st_fx->rate_control_fx,1) == 0 )
            {
                st_fx->rate_control_fx = 0;
                move16();
            }
            ELSE IF ( sub(st_fx->mode_QQF_fx,1) == 0) /* now rate control is not active and still the global rate is below the target. so go to QFF mode */
            {
                st_fx->mode_QQF_fx = 0;
                move16();
            }
            ELSE
            {
                IF ( sub(st_fx->SNR_THLD_fx, 15360) >= 0 )
                {
                    st_fx->SNR_THLD_fx =sub(st_fx->SNR_THLD_fx ,384 ); /*Q8 */
                }
                ELSE
                {
                    st_fx->SNR_THLD_fx = 15360;
                    move16();
                }
            }
        }

        IF ( L_sub(st_fx->global_avr_rate_fx , L_sub(target_fx,983040)) < 0 )  /* In QFF mode and global rate is less than target rate-0.2kbps. We can send some Q frames
                                                    to F frames to improve the quality */
        {
            /* kick in bouncing back from Q to F */
            st_fx->Q_to_F_fx = 1;
            move16();

            /* average rate for next 600ms = global_rate * 2 - rate of the past RATEWIN active frames */
            /* avratetarg = (float)((RATEWIN * 10) * 2 * VBR_ADR_MAX_TARGET - st->global_avr_rate); */
            avratetarg_fx = L_sub(L_shl(L_mult0(RATEWIN ,VBR_ADR_MAX_TARGET_x10_Q1 ),13), st_fx->global_avr_rate_fx );
            /* Q13 */


            /* compute the percentage of frames that needed to be sent to F. st->pattern_m is computed as % val * 1000. eg. if % is 10%, then
               st->pattern_m=100 . Later this value is used in voiced.enc to bump up 10% of PPP frames to F frames. */
            /*  st->pattern_m = (short)(1000 * (avratetarg - 6.15f * RATEWIN * 10)/(10 * RATEWIN * 0.1f) ); */

            L_tmp =  L_mult0(RATEWIN ,VBR_ADR_MAX_TARGET_x10_Q1 );
            L_tmp = L_shl(L_tmp , 12);
            L_tmp = L_sub(avratetarg_fx , L_tmp);
            tmp = extract_h(L_shl(Mult_32_16(L_tmp,27307),4));
            st_fx->pattern_m_fx =tmp;
            move16();

            if ( st_fx->pattern_m_fx < 0 )
            {
                st_fx->pattern_m_fx = 0;
                move16();    /* no bump up will ever happen */
            }

            if ( sub(st_fx->pattern_m_fx,1000) > 0 )
            {
                st_fx->pattern_m_fx = 1000;
                move16();/* 10% of bump ups */
            }

            st_fx->patterncount_fx = 0;
            move16();
        }
        ELSE
        {
            st_fx->Q_to_F_fx = 0;
            move16();
        }

        st_fx->sum_of_rates_fx = 0;
        move16();
        st_fx->numactive_fx = 0;
        move16();

    }

    st_fx->numactive_fx = add(st_fx->numactive_fx,1);

    /* sum the total number of bits (in kbytes) * 10 here */
    /*st->sum_of_rates += (st_fx->core_brate_fx / 1000.0f) * 10; */
    L_tmp  = L_shl(Mult_32_16(st_fx->core_brate_fx ,20972),7);
    st_fx->sum_of_rates_fx = L_add(st_fx->sum_of_rates_fx  ,L_tmp);

    return;
}

