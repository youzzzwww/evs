/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"

/*-----------------------------------------------------------------*
 * Local constants
 *-----------------------------------------------------------------*/

#define ALPHA_FX         3277     /*.1 Q15*/
#define ALPHAM1_FX       29491    /*.9 Q15*/

#define TH_PC_FX         12  /* max sum of pitch differencies */

#define BCKR_SLOW_UPDATE_SCALE_FX 3277  /*Q15 Step size for slow bckr update */
#define COR_MIN16_FX     17039          /*.52 Q15 */
#define COR_MAX16_FX     27853          /*.85 Q15 */

/* 10.0e5f causes problems with music - as the noise estimate starts to track the music */
#define TH_STA16_FX    (350000<<10)      /* MODE1: 3.5e5f */

#define TH_EPS16_FX       3277   /* Q11 (1.6) tuned for speech only, not for music */
#define K_16_FX          25690   /* Q20 (0.0245f) */
#define C_16_FX          -1925   /* Q13 (-0.235f) */
#define ALPHA_MAX16_FX   32440   /* Q15 (0.99)    */

#define COR_MIN8_FX      21299     /*.65 Q15 */
#define COR_MAX8_FX      22938     /*.70 Q15 */
#define TH_EPS8_FX       21299     /* 10.4 in Q11 Thresholds for NB processing - ... */
#define TH_STA8_FX      (500000<<10)
#define K_8_FX           9452     /* Q20 (0.0091f) */
#define C_8_FX           2609     /* Q13 (0.0091f) */
#define ALPHA_MAX8_FX    32735    /* Q15 (0.999)   */

#define HC_CNT_SLOW_FX     80      /* limit for harm corr count slow */

#define HE_LT_CNT_PRE_FX       50
#define HE_LT_CNT_INIT_FX      150
#define HE_LT_CNT_PRE_SHORT_FX 250
#define HE_LT_CNT_FX           30

#define LN_E_MIN_PLUS_ONE_FX      1      /* max(1, ln(E_MIN+1.0))  = max(1,ln(0.0035f+1f)) in  Q8 */
#define COR_MAX_NNE_FX            27853   /*  0.85 Q15    */

#define HE_LT_THR1_FX   2560    /*10.0  Q8*/
#define HE_LT_THR2_FX   7680      /*30.0 Q8*/

/*-----------------------------------------------------------------*
 * noise_est_AR1_Qx()
 *
 * y(n)(Qx) = alpha(Q15) * x(Qx)  +  (1.0f-alpha)* y(n-1) (Qx)
 *-----------------------------------------------------------------*/

Word16 noise_est_AR1_Qx(                /* o : Qx   y(n)  */
    Word16 x,     /* i : Qx  x(n)   */
    Word16 y,     /* i : Qx  y(n-1) */
    Word16 alpha /*i : Q15 scaling of driving x(n)  */
)
{
    Word16 alpham1;
    /*alpham1 = negate(add((Word16)-32768, alpha)); */
    alpham1 =  sub(32767, alpha); /* one cycle less */

    return mac_r(L_mult(y, alpham1), x, alpha);
}

/*-----------------------------------------------------------------*
 * noise_est_ln_q8_fx()
 *
 * q8  = (float)log( L_enr[i] + add1po*1.0 );
 *-----------------------------------------------------------------*/

static Word16  noise_est_ln_q8_fx(
    Word32 L_enr,
    Word16 flag_add1p0,  /* flag to  add 1.0 */
    Word16 q_new_plus_q_scale )
{
    Word16 e_ener, f_ener;
    Word32 L_tmp;

    L_tmp  = L_add(L_enr, L_shl((Word32)1L, q_new_plus_q_scale)); /*  +1.0f */
    if( flag_add1p0 == 0 )
    {
        L_tmp = L_add(L_enr, 0);   /* +0 , no offset */
    }

    L_tmp = L_max(L_tmp,(Word32)1L);  /* make sure log2_norm_lc does not cause table reading out of bounds */

    e_ener = norm_l(L_tmp);
    f_ener = Log2_norm_lc(L_shl(L_tmp, e_ener));
    e_ener = sub(sub(30,e_ener),q_new_plus_q_scale);
    L_tmp  = Mpy_32_16(e_ener, f_ener, 22713); /* Q16 (22713 = Ln(2) in Q15)*/
    return   round_fx(L_shl(L_tmp, 8)); /* Q8 */
}

/*-----------------------------------------------------------------*
 * eps_quota_fx()
 *
 *
 *-----------------------------------------------------------------*/

static Word32 eps_quota_fx(     /* o:  eps_num/eps_den  in  q_out */
    Word16 eps_num_h,           /* num high ,*/
    Word16 eps_num_l,           /* num low (signed) ,*/
    Word16 eps_den_h,           /* den low  */
    Word16 eps_den_l,           /* den low (signed),*/
    Word16 q_out                /* range 15...0 , tested with 11 and 12 */
)
{
    Word32 L_tmp_num, L_tmp_den;
    Word16 exp_num, exp_den;
    Word16 m_num, m_den;
    Word16 num_shift;


    L_tmp_num   = L_Comp(eps_num_h, eps_num_l);
    L_tmp_den   = L_Comp(eps_den_h, eps_den_l);

    exp_num = sub(norm_l(L_tmp_num), 1);   /*  make sure m_ num is lower than  m_den  */
    m_num   = extract_h(L_shl(L_tmp_num, exp_num));
    exp_den = norm_l(L_tmp_den);
    m_den   = extract_h(L_shl(L_tmp_den, exp_den));

    exp_num = sub(exp_num, exp_den);
    if (m_den != 0)
    {
        assert(m_den >= m_num);
        m_num = div_s(m_num, m_den); /* single basop */
    }

    num_shift = add(15-q_out, exp_num);
    if (m_den == 0)
    {
        /* no division made due to zero denominator  */
        m_num = 0;
        move16();
    }

    return L_shr(m_num, num_shift);
}

/*-----------------------------------------------------------------*
 * noise_est_init_fx()
 *
 * Initialization of Noise estimator
 *-----------------------------------------------------------------*/

void noise_est_init_fx(
    Word16 *totalNoise,            /* o  : noise estimate over all critical bands     */
    Word16 *first_noise_updt,      /* o  : noise update initialization flag           */
    Word32 bckr[],                 /* o  : per band background noise energy estimate  */
    Word32 enrO[],                 /* o  : per band old input energy                  */
    Word32 ave_enr[],              /* o  : per band long-term average energies        */
    Word16 *pitO,                  /* o  : open-loop pitch values from preceed. frame */
    Word16 *aEn,                   /* o  : noise adaptation hangover counter          */
    Word16 *st_harm_cor_cnt,       /* i/o: 1st harm correlation timer                */
    Word16 *bg_cnt,                /* i/o: pause burst length counter                */
    Word16 *lt_tn_track,           /*  Q15 */
    Word16 *lt_tn_dist,            /*  Q8 */
    Word16 *lt_Ellp_dist,          /* Etot low lp same domain as  *Etot_l_lp, Q8 */
    Word16 *lt_haco_ev,            /*  Q15 */
    Word16 *low_tn_track_cnt       /*  Q0  */
)
{
    Word16 i;

    FOR( i=0; i<NB_BANDS; i++ )
    {
        enrO[i]    = E_MIN_FX;
        move32();
        bckr[i]    = E_MIN_FX;
        move32();
        ave_enr[i] = E_MIN_FX;
        move32();
    }

    pitO[0] = 0;
    move16();
    *totalNoise = 0;
    move16();
    *first_noise_updt = 0;
    move16();

    *aEn = 6;
    move16();

    *st_harm_cor_cnt = 0;
    move16();
    *bg_cnt = 0;
    move16();

    *lt_tn_track      = 6554; /*.20 in Q15*/ move16();
    *lt_tn_dist       =  0;
    move16();
    *lt_Ellp_dist     =  0 ;
    move16();
    *lt_haco_ev       = 13107; /*.40 in Q15*/ move16();
    *low_tn_track_cnt = 0;
    move16();

    return;
}


/*-----------------------------------------------------------------*
 * noise_est_pre_fx()
 *
 * Track energy and signal dynamics
 *-----------------------------------------------------------------*/

void noise_est_pre_fx(
    const Word16 Etot,              /* i  : Energy of current frame  */
    const Word16 ini_frame_fx,      /* i  : Frame number (init)      */
    Word16 *Etot_l,           /* i/o: Track energy from below  */ /*Q8*/
    Word16 *Etot_h,           /* i/o: Track energy from above  */ /*Q8*/
    Word16 *Etot_l_lp,        /* i/o: Smoothed low energy      */ /*Q8*/
    Word16 *Etot_last,        /* i/o: Energy of last frame     */ /*Q8*/
    Word16 *Etot_v_h2,        /* i/o: Energy variations        */ /*Q8*/
    Word16 *sign_dyn_lp,      /* i/o: Smoother signal dynamics */ /*Q8*/
    Word16 harm_cor_cnt,      /* i :  */
    Word16 *Etot_lp           /* i/o: Smoothed  energy */
)
{
    Word16 tmp;

    IF (sub(ini_frame_fx, 1) <= 0)
    {
        *Etot_h    = Etot;
        move16();
        *Etot_l    = Etot;
        move16();
        *Etot_l_lp = Etot;
        move16();
        *Etot_last = Etot;
        move16();
        *Etot_v_h2 = 0;
        move16();
        *Etot_lp   = Etot;
        move16();
        *sign_dyn_lp = 0;
        move16();
    }
    ELSE
    {
        /* *Etot_lp = 0.20f*Etot + 0.80f* *Etot_lp;  */
        *Etot_lp = mac_r(L_mult(6554, Etot), 26214, *Etot_lp);
        move16();

        *Etot_h = sub(*Etot_h, 10);
        move16();  /* 10=0.04 in  Q8 */
        *Etot_h = s_max(*Etot_h, Etot);
        move16();
        *Etot_l = add(*Etot_l, 20);
        move16(); /* 20 =  .08 in Q8 */


        /* Could even be higher but it also delays first entry to DTX */
        IF ( sub(harm_cor_cnt,HE_LT_CNT_PRE_FX) > 0 )
        {
            test();
            IF( ( sub( ini_frame_fx , s_min(HE_LT_CNT_INIT_FX ,MAX_FRAME_COUNTER-1) ) < 0  )
            &&   (sub(sub(*Etot_h, *Etot_lp),(Word16)3*256)  < 0 ) /* 3.0 Q8 */
              )
            {
                /* *Etot_l += min(2,(*Etot_last-*Etot_l)*0.1f); */
                tmp     =  mult_r(sub(*Etot_last, *Etot_l), 3277);   /* factor in Q15 3277  .1*32768 */
                tmp     =  s_min(512, tmp); /* 2.0 in Q8 is 512*/
                *Etot_l   =  add(*Etot_l, tmp);
                move16();/* Q8 */
            }

            /* Avoids large steps in short active segments */
            test();
            IF ( ( sub(sub(*Etot_last, *Etot_l), HE_LT_THR2_FX ) > 0 )  /* 30.0f*Q8 */
                 && (sub(harm_cor_cnt,HE_LT_CNT_PRE_SHORT_FX)>0)
               )
            {
                /* *Etot_l += (*Etot_last-*Etot_l)*0.02f; */
                *Etot_l = add(*Etot_l, mult_r(sub(*Etot_last, *Etot_l), 655));
                move16();/* 0.02 = 655  Q8*/
            }
            ELSE IF (sub(sub(*Etot_last, *Etot_l), HE_LT_THR1_FX ) > 0) /* 10.0 in Q8*/
            {
                *Etot_l = add(*Etot_l, 20);
                move16();/* 0.08 is 20  in Q8*/
            }

        }

        *Etot_l = s_min(*Etot_l, Etot);

        test();
        test();
        test();
        test();
        IF ( ( (sub(harm_cor_cnt, HE_LT_CNT_FX) > 0 )
               &&  (sub(sub(*Etot_last, *Etot_l), HE_LT_THR2_FX ) > 0 )
             )
             ||  ( sub(harm_cor_cnt, HE_LT_CNT_FX > 0 )  &&  (sub(ini_frame_fx, HE_LT_CNT_INIT_FX) < 0 ) )
             || (sub(sub(*Etot_l_lp, *Etot_l), HE_LT_THR2_FX ) > 0 )
           )
        {
            /**Etot_l_lp = 0.03f * *Etot_l + (1.0f - 0.03f) * *Etot_l_lp; */
            *Etot_l_lp = mac_r(L_mult(983, *Etot_l), 31785, *Etot_l_lp);
            move16();

        }
        ELSE
        {
            /*  *Etot_l_lp = 0.02f * *Etot_l + (1.0f - 0.02f) * *Etot_l_lp; */
            *Etot_l_lp = round_fx(L_mac(L_mult(655 , *Etot_l), 32113 , *Etot_l_lp ));
        }
        /**sign_dyn_lp = 0.1f * (*Etot_h - *Etot_l) + (1.0f - 0.1f) * *sign_dyn_lp;*/
        *sign_dyn_lp = round_fx(L_mac(L_mult(3277, sub(*Etot_h, *Etot_l)), 29491, *sign_dyn_lp));
    }

    return;
}

/*==================================================================================*/
/* FUNCTION : noise_est_down_fx()                                                   */
/*----------------------------------------------------------------------------------*/
/* PURPOSE :   Down-ward noise udatation routine                                    */
/*----------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                                                               */
/* _ (Word32[]) fr_bands   : per band input energy                     Q_new+QSCALE */
/* _ (Word32[]) bckr       : per band background noise energy estimate Q_new+QSCALE */
/* _ (Word16  ) min_band   : minimum critical band                               Q0 */
/* _ (Word16  ) max_band   : maximum critical band                               Q0 */
/* _ (Word32[]) bckr_he    : per band background noise energy estimate Q_new+QSCALE */
/* _ (Word16  ) Etot       : Energy of current frame                             Q8 */
/* _ (Word16* ) Etot_last  : Energy of last frame                                Q8 */
/* _ (Word16* ) Etot_v_h2   : Energy variaions of noise frames                   Q8 */
/* _ (Word16  ) Q_new      : Qformat                                                */
/*----------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                               */
/* _ (Word32[]) bckr       : per band background noise energy estimate Q_new+QSCALE */
/* _ (Word32[]) tmpN       : temporary noise update                    Q_new+QSCALE */
/* _ (Word32[]) enr        : averaged energy over both subframes       Q_new+QSCALE */
/* _ (Word16* ) totalNoise : noise estimate over all critical bands              Q8 */
/* _ (Word16  ) Etot       : Energy of current frame                             Q8 */
/* _ (Word16* ) Etot_last  : Energy of last frame                                Q8 */
/* _ (Word16* ) Etot_v_h2   : Energy variaions of noise frames                   Q8 */
/* _ (Word32[]) tmpN_he1   : temporary noise update 1                  Q_new+QSCALE */
/*----------------------------------------------------------------------------------*/


void noise_est_down_fx(
    const Word32 fr_bands[],        /* i  : per band input energy (contains 2 vectors) */
    Word32 bckr[],            /* i/o: per band background noise energy estimate  */
    Word32 tmpN[],             /* o  : temporary noise update                     */
    Word32 enr[],             /* o  : averaged energy over both subframes        */
    const Word16 min_band,          /* i  : minimum critical band                      */
    const Word16 max_band,          /* i  : maximum critical band                      */
    Word16 *totalNoise,       /* o  : noise estimate over all critical bands     */
    Word16 Etot,              /* i  : Energy of current frame                    */
    Word16 *Etot_last,        /* i/o: Energy of last frame            Q8         */
    Word16 *Etot_v_h2,        /* i/o: Energy variations of noise frames  Q8      */
    Word16 Q_new,
    const Word32 e_min              /* i  : minimum energy scaled    Q_new + QSCALE    */
)

{
    Word32  Ltmp, L_tmp;
    const Word32 *pt1, *pt2;
    Word16 i;
    Word16 e_Noise, f_Noise;
    Word16 scale;
    Word32 totalNoise_temp;
    Word32 L_Etot, L_Etot_last, L_Etot_v_h2, L_Etot_v;

    L_Etot      = L_shl(Etot, 16);               /*Q24 for later AR1 computations*/
    L_Etot_last = L_shl(*Etot_last, 16);
    L_Etot_v_h2 = L_shl(*Etot_v_h2, 16);
    scale = add(Q_new,QSCALE);
    move16();

    /*-----------------------------------------------------------------*
     * Estimate total noise energy
     *-----------------------------------------------------------------*/

    totalNoise_temp = L_deposit_l(0);
    FOR (i = min_band; i <= max_band; i++)
    {
        totalNoise_temp = L_add(totalNoise_temp, bckr[i]);            /*Q_new+QSCALE*/
    }
    totalNoise_temp = L_max(totalNoise_temp,L_shl(e_min,4));


    totalNoise_temp = L_max(totalNoise_temp,(Word32)1L);  /* make sure log2_norm_lc does not cause table reading out of bounds */

    /*totalNoise = 10.0f * (float)log10( *totalNoise );*/
    e_Noise = norm_l(totalNoise_temp);
    f_Noise = Log2_norm_lc(L_shl(totalNoise_temp, e_Noise));
    e_Noise = sub(30, e_Noise);
    e_Noise = sub(e_Noise, scale);
    Ltmp = Mpy_32_16(e_Noise, f_Noise, LG10);
    *totalNoise = round_fx(L_shl(Ltmp, 10));                            /*Q8*/

    /*-----------------------------------------------------------------*
     * Average energy per frame for each frequency band
     *-----------------------------------------------------------------*/

    pt1 = fr_bands;                        /*Q_new+QSCALE*/
    pt2 = fr_bands + NB_BANDS;

    FOR( i=0 ; i < NB_BANDS; i++ )
    {
        Ltmp = L_add(L_shr_r(*pt1,1),L_shr_r(*pt2,1));
        /*Ltmp = L_shr_r(L_add(*pt1,*pt2),1);*/
        enr[i] = Ltmp;
        move32();/*Q_new+QSCALE*/
        pt1++;
        pt2++;
    }

    /*-----------------------------------------------------------------*
     * Background noise energy update
     *-----------------------------------------------------------------*/

    FOR( i=0; i< NB_BANDS; i++ )
    {
        /* tmpN[i] = (1-ALPHA) * bckr[i] + ALPHA * enr[i]; */
        L_tmp   = Mult_32_16(enr[i], ALPHA_FX); /*ALPHA * enr2*/
        tmpN[i] = Madd_32_16(L_tmp, bckr[i], ALPHAM1_FX);
        move32();/*Q_new+QSCALE*/
        tmpN[i] =L_max( tmpN[i] , e_min); /* handle  div by zero in find_tilt_fx  */

        /*  if( tmpN[i] < bckr[i] ) { bckr[i] = tmpN[i];  }*/
        /* Defend to increase noise estimate: keep as it is or decrease  */
        bckr[i] = L_max(L_min(bckr[i], tmpN[i]), e_min);
        move32();/*Q_new+QSCALE*/
    }

    /*------------------------------------------------------------------*
     * Energy variation update
     *------------------------------------------------------------------*/
    /*Etot_v = (float) fabs(*Etot_last - Etot);*/
    L_Etot_v      = L_abs(L_sub(L_Etot_last, L_Etot)); /* Q24 */

    /* *Etot_v_h2 = (1.0f-0.02f) * *Etot_v_h2 + 0.02f * min(3.0f, Etot_v); */
    L_Etot_v_h2  = Mult_32_16(L_Etot_v_h2, 32113);  /*.98 in Q15 , Q24+Q15+1 -16 => Q24 */
    L_Etot_v     = L_min((Word32)(3*(1<<24)), L_Etot_v);
    L_tmp        = Mult_32_16(L_Etot_v, 655);      /*.02 in Q15  , Q24+Q15+1 -16 ==> Q24 ) */

    *Etot_v_h2  = round_fx(L_add(L_Etot_v_h2, L_tmp));  /*Q24->Q8*/

    /* if (*Etot_v_h2 < 0.1f) {  *Etot_v_h2 = 0.1f; } */
    *Etot_v_h2  = s_max(*Etot_v_h2, 26 );  /* 0.1 in Q8*/

    return;
}

/*-----------------------------------------------------------------*
* noise_est_fx()
*
* Noise energy estimation (noise energy is updated in case of noise-only frame)
*-----------------------------------------------------------------*/
void  noise_est_fx(
    Encoder_State_fx *st_fx,                /* i/o: state structure                                                        */
    const Word32 tmpN[],                  /* i  : temporary noise update                            Q_new + QSCALE    */
    const Word16 *pit,                    /* i  : open-loop pitch values for each half-frame           Q0             */
    const Word16 cor[],                   /* i  : normalized correlation for all half-frames           Q15            */
    const Word16 epsP_h[],                /* i  : msb prediction error energies                        Q_r-1           */
    const Word16 epsP_l[],                /* i  : msb prediction error energies                        Q_r-1           */
    const Word16 Etot,                    /* i  : total channel E (see find_enr_fx.c)                  Q8              */
    const Word16 relE,                    /* i  : (VA_CHECK addition) relative frame energy            Q8?            */
    const Word16 corr_shift,              /* i  : normalized correlation correction                    Q15             */
    const Word32 enr[],                   /* i  : averaged energy over both subframes               Q_new + Q_SCALE    */
    Word32 fr_bands[],              /* i  : spectrum per critical bands of the current frame  Q_new + Q_SCALE   */
    Word16 *cor_map_sum,            /* o  :                                                         Q8                */
    Word16 *sp_div,                 /* o  :                                                      Q_sp_div            */
    Word16 *Q_sp_div,               /* o  :    Q factor for sp_div                                                  */
    Word16 *non_staX,               /* o  : non-stationarity for sp/mus classifier                              */
    Word16 *loc_harm ,              /* o  :   multi-harmonicity flag for UV classifier                          */
    const Word32 *lf_E,                   /* i  : per bin energy  for low frequencies             Q_new + Q_SCALE -2    */
    Word16 *st_harm_cor_cnt,        /* i/o  : 1st harm correlation timer                         Q0               */
    const Word16 Etot_l_lp,               /* i    : Smoothed low energy                                Q8             */
    const Word16 Etot_v_h2,               /* i    : Energy variations                                  Q8               */
    Word16 *bg_cnt,                 /* i    : Background burst length timer                      Q0               */
    Word16 EspecdB[],               /* i/o:  log E spectrum (with f=0) of the current frame Q7  for multi harm  */
    Word16 Q_new,                   /* i  : SCaling of current frame                                            */
    const Word32 Le_min_scaled,           /*i  : Minimum energy value     in  Q_new + Q_SCALE                         */
    Word16 *sp_floor                /* o  : noise floor estimate                                 Q7             */
)
{
    Word16 alpha,  alpha2, alpha2m1, alpham1;
    Word16 cor_min, cor_max, num, den, ExpNum, ExpDen, noise_chartmp;
    Word16 wtmp1,wtmp, ExpLmax, ExpLmax2, tmpExp, nchar_thr, cor_tmp;
    Word16 i, tmp_pc, pc,th_eps;
    Word32 th_sta, Lnum, Lden, non_sta, LepsP, Ltmpden;
    Word16 e_ener, f_ener;
    Word32 Ltmp, Ltmp1,Lsum_num, Lsum_den, *pt1, *pt2,Ltmp2, Lnon_sta2;
    Word16 spec_div, noise_char;
    Word16 log_enr16;
    Word16 updt_step ; /* Q15 */
    Word16 aE_bgd,sd1_bgd,bg_bgd2;
    Word16 tn_ini;
    Word16 epsP_0_2,epsP_0_2_ad,epsP_0_2_ad_lp_max;
    Word16 epsP_2_16,epsP_2_16_dlp,epsP_2_16_dlp_max;
    Word16 PAU, BG_1, NEW_POS_BG;

    Word16 haco_ev_max;
    Word16 Etot_l_lp_thr;
    Word16 comb_ahc_epsP, comb_hcm_epsP;

    Word16 enr_bgd,cns_bgd,lp_bgd,ns_mask;
    Word16 lt_haco_mask, bg_haco_mask;
    Word16 SD_1,SD_1_inv, bg_bgd3,PD_1,PD_2,PD_3,PD_4,PD_5;

    Word16 non_staB; /* Q8 */
    Word32 L_tmp_enr, L_tmp_ave_enr, L_tmp_ave_enr2;
    Word16 tmp_Q;
    Word16 tmp, tmp2;   /* general temp registers */
    Word16 tmp_enr, tmp_floor  ; /* constants in Q8 */
    Word16 vad_bwidth_fx;    /* vad ns  control variabel for input bwidth from teh BWD  */

    /*-----------------------------------------------------------------*
     * Initialization
     *-----------------------------------------------------------------*/

    vad_bwidth_fx = st_fx->input_bwidth_fx;
    move16();

    /*st_fx->ener_RAT = 10.0f * (float)log10( mean(lf_E, 8));*/
    Ltmp = L_shr(lf_E[0],3);
    FOR(i = 1; i < 8; i++)
    {
        Ltmp = L_add(Ltmp, L_shr(lf_E[i],3));
    }
    Ltmp = L_max(Ltmp,(Word32)1L);  /* make sure log2_norm_lc does not cause table reading out of bounds */
    e_ener = norm_l(Ltmp);
    f_ener = Log2_norm_lc(L_shl(Ltmp, e_ener));
    e_ener = sub(30, e_ener);
    e_ener = sub(e_ener, sub(add(Q_new,QSCALE),2));
    Ltmp = Mpy_32_16(e_ener, f_ener, LG10);
    wtmp = round_fx(L_shl(Ltmp, 10));/*Q8*/

    /* st_fx->ener_RAT /= (Etot + 0.01f);   */
    wtmp1 = add(Etot,3);/*3 is 0.01 in Q8  */
    /*  st_fx->ener_RAT_fx = wtmp/wtmp1 */
    st_fx->ener_RAT_fx = 0;
    move16();
    IF( wtmp > 0 )
    {
        st_fx->ener_RAT_fx = 32767;
        move16(); /*Q15*/
        if(sub(wtmp1, wtmp ) >= 0 )
        {
            st_fx->ener_RAT_fx = div_s(wtmp, wtmp1); /*Q15*//* wtmp1 gte than wtmp */
        }
    }

    /*-----------------------------------------------------------------*
     * Set the threshold for eps & non_sta based on input sampling rate
     * The reason is that in case of 8kHz sampling input, there is nothing
     * between 4kHz-6.4kHz. In noisy conditions, this makes a fast
     * transition even in noise-only parts, hence producing a "higher
     * order" spectral envelope => the epsP ratio is much less effective.
     *-----------------------------------------------------------------*/

    IF (sub(vad_bwidth_fx,NB) != 0)     /* WB input */
    {
        th_eps = TH_EPS16_FX;
        move16();/*Q11*/
        th_sta = TH_STA16_FX;
        move16();/*Q10  */
        cor_min = COR_MIN16_FX;
        move16();/*Q15*/
        cor_max = COR_MAX16_FX;
        move16();/*Q15*/
    }
    ELSE                                /* NB input */
    {
        th_eps = TH_EPS8_FX;
        move16();/* Q11 */
        th_sta = TH_STA8_FX;
        move16(); /* Q10 */
        cor_min = COR_MIN8_FX;
        move16();
        cor_max = COR_MAX8_FX;
        move16();
    }


    /*-----------------------------------------------------------------*
     * Estimation of pitch stationarity
     *-----------------------------------------------------------------*/

    /* pc = abs(pit[0] - pitO) + abs(pit[1] - pit[0]) */ /* needed in signal_clas() */
    wtmp  = abs_s(sub(pit[0], st_fx->pitO_fx));
    wtmp1 = abs_s(sub(pit[1], pit[0]));
    pc = add(wtmp, wtmp1);


    Ltmp = L_deposit_h(corr_shift);
    Ltmp = L_mac(Ltmp, cor[0], 10923);
    Ltmp = L_mac(Ltmp, cor[1], 10923);
    wtmp = mac_r(Ltmp, cor[2], 10923);

    tmp_pc = pc;
    move16();
    if (sub(wtmp, cor_min) < 0)
    {
        tmp_pc = TH_PC_FX;
        move16();  /* low correlation -> probably inactive signal */
    }

    st_fx->pitO_fx = pit[1];
    move16();  /* Update */

    /*-----------------------------------------------------------------*
     * Multi-harmonic analysis
     *-----------------------------------------------------------------*/

    *loc_harm = multi_harm_fx( EspecdB, st_fx->old_S_fx, st_fx->cor_map_fx, &st_fx->multi_harm_limit_fx, st_fx->total_brate_fx,
                               st_fx->bwidth_fx, &st_fx->cor_strong_limit_fx, &st_fx->mean_avr_dyn_fx, &st_fx->last_sw_dyn_fx, cor_map_sum, sp_floor );

    /*-----------------------------------------------------------------*
     * Detection of frames with non-stationary spectral content
     *-----------------------------------------------------------------*/

    /* weighted sum of spectral changes per critical bands */
    Lsum_num = L_deposit_l(0);
    Lsum_den = L_deposit_l(0);

    /* Find a proper scaling to prevent overflow, but acheiving good computation on low level signals */
    tmpExp = 0;
    move16();
    ExpLmax = sub(30, norm_l(fr_bands[10]));
    ExpLmax2  = sub(30, norm_l(st_fx->fr_bands2_fx[10]));
    tmpExp =  s_max(tmpExp, sub(shl(s_max(ExpLmax,ExpLmax2), 1), s_min(ExpLmax, ExpLmax2)));
    FOR (i = 11; i<= st_fx->max_band_fx; i++)
    {
        ExpLmax = sub(30, norm_l(fr_bands[i]));
        ExpLmax2  = sub(30, norm_l(st_fx->fr_bands2_fx[i]));
        tmpExp = s_max(tmpExp, sub(shl(s_max(ExpLmax,ExpLmax2), 1), s_min(ExpLmax, ExpLmax2)));
    }
    tmpExp = sub(tmpExp, 30-4-4);     /* 4bits for internal summation and 4 bits for comparaison */

    pt1 = fr_bands + 10;
    pt2 = st_fx->fr_bands2_fx + 10;
    FOR (i=10; i<=st_fx->max_band_fx; i++)
    {
        Lnum = L_max(*pt1, *pt2);     /* Don't need if anymore */
        Lsum_den = L_add(Lsum_den, Lnum);
        Ltmpden = L_min(*pt1, *pt2);
        Ltmpden =((Ltmpden==0)?1:Ltmpden);
        ExpNum = sub(norm_l(Lnum), 1);
        num = extract_h(L_shl(Lnum, ExpNum));
        num = mult_r(num, num);
        ExpDen = norm_l(Ltmpden);
        den = extract_h(L_shl(Ltmpden, ExpDen));
        num = div_s(num, den);
        Ltmp = L_shr(num, add(sub(sub(shl(ExpNum, 1), ExpDen), 15+1), tmpExp));
        Lsum_num = L_add(Lsum_num, Ltmp);

        pt1++;
        pt2++;
    }
    Lsum_den = L_shr(Lsum_den, tmpExp);


    /* calculation of spectral diversity */
    /* THR_SPDIV_FX = 5 , 1/5 Q15 =  6554 */
    spec_div = 0;
    move16();
    if (L_sub(Mult_32_16(Lsum_num, 6554), Lsum_den) > 0) /* Qx+Q15+1-16  ==> Qx */
    {
        spec_div = 1;
        move16();
    }

    /* *sp_div = Lsum_num / (Lsum_den + 1e-5f); */
    ExpNum = sub(norm_l(Lsum_num), 1);
    num = extract_h(L_shl(Lsum_num, ExpNum));

    Lsum_den = L_add(Lsum_den,1);

    ExpDen = norm_l(Lsum_den);
    den = extract_h(L_shl(Lsum_den, ExpDen));

    *sp_div = div_s(num, den);
    move16();

    *Q_sp_div = add(15,sub(ExpNum ,ExpDen));
    move16();

    /*-----------------------------------------------------------------*
     * Detection of frames with high energy content in high frequencies
     *-----------------------------------------------------------------*/

    /* calculation of energy in first 10 critical bands */
    Ltmp = sum32_fx( &fr_bands[st_fx->min_band_fx], sub(10, st_fx->min_band_fx) );

    /* calculation of energy in the rest of bands */
    Ltmp2 = sum32_fx( &fr_bands[10], sub(st_fx->max_band_fx,9) );

    wtmp = shl(1, sub(add(Q_new, QSCALE), 1));

    test();
    IF (L_msu(Ltmp, 100, wtmp) < 0 || L_msu(Ltmp2, 100, wtmp) < 0)
    {
        noise_chartmp = 0;
        move16();
    }
    ELSE
    {
        /* ftemp2 /= ftemp */
        ExpNum = sub(norm_l(Ltmp2), 1);
        num = extract_h(L_shl(Ltmp2, ExpNum));

        ExpDen = norm_l(Ltmp);
        den = extract_h(L_shl(Ltmp, ExpDen));
        num = div_s(num, den);
        noise_chartmp = extract_h(L_shr(num, add(sub(ExpNum, ExpDen), 4-16)));   /* Q11 */
    }

    noise_chartmp = s_min(noise_chartmp, (Word16)10<<11);  /* Q11 */

    /* update LT value of the final parameter */
    /* *st_noise_char = M_ALPHA * *st_noise_char + (1-M_ALPHA) * noise_chartmp */
    st_fx->noise_char_fx = mac_r(L_mult(M_ALPHA_FX, st_fx->noise_char_fx), ONE_MINUS_M_ALPHA, noise_chartmp);


    nchar_thr = THR_NCHAR_WB_FX;
    move16();   /* 1.0 Q11 */
    if( sub(vad_bwidth_fx,NB) == 0 )
    {
        nchar_thr = THR_NCHAR_NB_FX;
        move16(); /* 1.0 Q11 */
    }

    noise_char = 0;
    move16();
    if (sub(st_fx->noise_char_fx, nchar_thr) > 0)
    {
        noise_char = 1;
        move16();
    }

    /* save the 2 last spectra per crit. bands for the future */
    Copy32(st_fx->fr_bands1_fx, st_fx->fr_bands2_fx, NB_BANDS);
    Copy32(fr_bands+NB_BANDS, st_fx->fr_bands1_fx, NB_BANDS);

    /*-----------------------------------------------------------------*
     * Non-stationarity estimation for each band
     * Handicap high E frames in average computing
     *-----------------------------------------------------------------*/

    /* set averaging factor */
    /* ftemp = relE; */
    /* if( ftemp < 0.0f ) { ftemp = 0.0f;     } */
    tmp = s_max(relE,0); /* Q8  */

    /* alpha =  0.064f * ftemp  +  0.75f; */
    Ltmp  = Mult_32_16((Word32)137438953L, tmp); /* Q31(.064)+Q8+1-16      -->  Q24 */
    Ltmp  =  L_mac(Ltmp,256,24576) ;   /* Q8+Q15(.75)+1  --> Q24    */
    alpha =  round_fx(L_shl(Ltmp,7));  /*Q24 +7 --> Q31    Q15*/

    /*if( alpha > 0.999f  {     alpha = 0.999f;} */
    alpha   = s_min(alpha, 32735 ); /*.999 in Q15*/
    alpham1 = negate(add(-32768, alpha));  /* 1.0 - alpha  */
    /*--------------------------------------------------------------*
     * during significant attacks, replace the LT energy by the
     * current energy this will cause non_sta2 failures to occur in
     * different frames than non_sta failures
     *--------------------------------------------------------------*/

    alpha2   = alpha;
    move16();
    alpha2m1 = alpham1;
    move16();
    IF (spec_div > 0)
    {
        alpha2 = 0;
        move16();
        alpha2m1 = 32767;
        move16();
    }
    Lnon_sta2 = L_deposit_l(1<<10);

    non_sta = L_deposit_l(1<<10);
    *non_staX = 0;
    move16();
    non_staB = 0;
    move16();

    FOR( i = st_fx->min_band_fx; i <= st_fx->max_band_fx; i++ )
    {
        /*  + 1.0f added to reduce sensitivity to non stationarity in low energies  */
        /* tmp_enr = enr[i] + 1.0f; */
        tmp_Q         =  add(Q_new, Q_SCALE);
        Ltmp          =  L_shl((Word32)1L, tmp_Q);   /* 1.0 added in the right dynamic domain */
        L_tmp_enr     =  L_add(enr[i]              , Ltmp );  /* enr      scale  dynamic     */
        L_tmp_ave_enr =  L_add(st_fx->ave_enr_fx[i], Ltmp);   /* ave__enr  scale dynamic  */

        IF (L_sub(non_sta, th_sta) <= 0) /* Just to limit the saturation */
        {
            /* if( enr[i] > st_ave_enr2[i] ) */
            /* non_sta2 = non_sta2 * ((enr[i]+1) / (st_ave_enr2[i]+1)) */
            Lnum = L_max(L_tmp_enr, L_tmp_ave_enr);

            /* else */
            /* non_sta2 = non_sta2 * ((st_ave_enr2[i]+1) / (enr[i]+1)) */
            Lden = L_min(L_tmp_enr, L_tmp_ave_enr);

            ExpNum = sub(norm_l(Lnum), 1);
            num = extract_h(L_shl(Lnum, ExpNum));
            Lnum = L_shl(Lnum, ExpNum);
            ExpDen = norm_l(Lden);
            den = extract_h(L_shl(Lden, ExpDen));
            num = div_s(num, den);
            Ltmp =  Mult_32_16(non_sta, num);
            non_sta = L_shr(Ltmp, sub(ExpNum, ExpDen)); /* Q10 */
        }

        /*    st->ave_enr[i] = alpha * st->ave_enr[i] + (1-alpha) * enr[i];*/  /* update long-term average */
        Ltmp                 = Mult_32_16(st_fx->ave_enr_fx[i], alpha);
        Ltmp                 = L_add(Ltmp, Mult_32_16(enr[i], alpham1));
        st_fx->ave_enr_fx[i] = L_max(Le_min_scaled, Ltmp);
        move32();

        /* calculation of another non-stationarity measure (following attacks) */
        /*if( non_sta2 <= th_sta ){
             tmp_ave2 =  st->ave_enr2[i] + 1.0f;
             if( tmp_enr > tmp_ave2 ){
                 non_sta2 = non_sta2 * ( tmp_enr / tmp_ave2 );
             } else {
                 non_sta2 = non_sta2 * (tmp_ave2 / tmp_enr );
             }
         } */

        /* ave_enr2:: calculation of another non-stationarity measure (following attacks)  */
        Ltmp = L_shl((Word32)1L, tmp_Q );                        /* 1.0 added in the right dynamic domain */
        /*L_tmp_enr    =  L_add(enr[i]              , Ltmp );*/  /* enr      scale  dynamic    , done above  */
        L_tmp_ave_enr2 =  L_add(st_fx->ave_enr2_fx[i], Ltmp);    /* ave__enr  scale dynamic  */

        IF (L_sub(Lnon_sta2, th_sta ) <= 0)                        /* Just to limit the saturation */
        {
            Lnum = L_max(L_tmp_enr,   L_tmp_ave_enr2 );
            Lden = L_min(L_tmp_enr,   L_tmp_ave_enr2 );

            ExpNum = sub(norm_l(Lnum), 1);
            num = extract_h(L_shl(Lnum, ExpNum));
            Lnum = L_shl(Lnum, ExpNum);
            ExpDen = norm_l(Lden);
            den = extract_h(L_shl(Lden, ExpDen));
            num = div_s(num, den);
            Ltmp1  = Mult_32_16(Lnon_sta2, num);
            Lnon_sta2 = L_shr(Ltmp1, sub(ExpNum, ExpDen)); /* Q10 */
        }

        /* st_ave_enr2[i] = (float)alpha2 * st_ave_enr2[i]
                                + (1.0f - alpha2) * (enr[i]) */
        Ltmp1                 = Mult_32_16(st_fx->ave_enr2_fx[i], alpha2);
        Ltmp1                 = L_add(Ltmp1, Mult_32_16(enr[i], alpha2m1));
        st_fx->ave_enr2_fx[i] = L_max(Le_min_scaled, Ltmp1);
        move32();

        /* calculation of non-stationarity measure for speech/music classification */
        test();
        IF ( sub(i,START_BAND_SPMUS) >= 0 && sub(i,NB_BANDS_SPMUS+START_BAND_SPMUS) < 0 )
        {
            /* log_enr = (float)ln_fx(enr[i]); */
            log_enr16  = noise_est_ln_q8_fx( enr[i], 0 ,tmp_Q);
            wtmp = abs_s(sub(log_enr16, st_fx->past_log_enr_fx[i-START_BAND_SPMUS]));
            *non_staX                 = add(*non_staX, wtmp);
            move16();   /* Q8 */
            st_fx->past_log_enr_fx[i-START_BAND_SPMUS] = log_enr16;
            move16();

            /* calculate non-stationarity feature relative background */
            tmp_enr    =  noise_est_ln_q8_fx( enr[i], 1 , tmp_Q);  /* 1.0f  added */
            tmp_floor  =  LN_E_MIN_PLUS_ONE_FX  ;
            move16();    /* non dynamic init constant in Q8 */
            IF ( sub(st_fx->ini_frame_fx, HE_LT_CNT_INIT_FX) >= 0 )
            {
                tmp_floor  =  noise_est_ln_q8_fx(  st_fx->bckr_fx[i], 1, tmp_Q );
            }
            non_staB       = add(non_staB, abs_s(sub(tmp_enr, tmp_floor)));  /*  Q8  */
        }

    } /* end of band loop FOR( i = st_fx->min_band_fx; i <= st_fx->max_band_fx; i++ ) */

    IF (sub(Etot,-1280) < 0 )
    {
        non_sta   = L_deposit_l(1024);      /* 1.0 in  Q10  */
        Lnon_sta2 = L_deposit_l(1024);      /* 1.0 in  Q10 */
    }

    /*-----------------------------------------------------------------*
     * Count frames since last correlation or harmonic event
     *-----------------------------------------------------------------*/

    Ltmp = L_mult(cor[0], 16384);
    Ltmp = L_mac(Ltmp, cor[1], 16384);

    test();
    test();
    *st_harm_cor_cnt = add(*st_harm_cor_cnt , 1);
    if( (Etot > 0) && ( (*loc_harm  > 0 ) || (sub(round_fx(Ltmp), COR_MAX_NNE_FX ) > 0) ))
    {
        *st_harm_cor_cnt = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Energy based pause length counter
     *-----------------------------------------------------------------*/
    test();
    IF(  (*bg_cnt >= 0) && (sub(sub(Etot , Etot_l_lp),1280) > 0/*5.0 in Q8*/))
    {
        /* Possible speech burst */
        *bg_cnt = -1;
        move16();
    }
    ELSE
    {
        test();
        if( sub(*bg_cnt,-1) == 0 && ( sub(sub(Etot , Etot_l_lp),1280) < 0 )/*5 in Q8*/ )
        {
            /* Possible start of speech pause */
            *bg_cnt = 0;
            move16();
        }
    }
    if (*bg_cnt >= 0)
    {
        *bg_cnt = add(*bg_cnt,1);
        move16();
    }

    /*-----------------------------------------------------------------*
     * Linear predition efficiency 0 to 2 order
     *-----------------------------------------------------------------*/

    /*epsP_0_2 = max(0 , min(8, epsP[0] / epsP[2])); */
    Ltmp      =  eps_quota_fx(epsP_h[0], epsP_l[0],
                              epsP_h[2], epsP_l[2] , 12 );  /* Word32  Q12 */
    BASOP_SATURATE_WARNING_OFF                              /* may saturate*/
    epsP_0_2  =  round_fx(L_shl(Ltmp,16));                  /* Q12+16  -16 ->  Q12 , NB saturation in Q12 sets max value to  7,999 */
    BASOP_SATURATE_WARNING_ON

    epsP_0_2  =  s_max(0, epsP_0_2);                         /* min value is 0 , Q12 */


    /* st->epsP_0_2_lp = 0.15f * epsP_0_2 + (1.0f-0.15f) * st->epsP_0_2_lp; */
    alpha                 = 4915;
    move16();                   /*0.15 in Q15 */
    st_fx->epsP_0_2_lp_fx = noise_est_AR1_Qx(epsP_0_2, st_fx->epsP_0_2_lp_fx ,alpha);

    /* epsP_0_2_ad = (float) fabs(epsP_0_2 - st->epsP_0_2_lp );  */
    epsP_0_2_ad = abs_s(sub(epsP_0_2, st_fx->epsP_0_2_lp_fx)); /* Q12 */

    /*if (epsP_0_2_ad < st->epsP_0_2_ad_lp)  {
        st->epsP_0_2_ad_lp = 0.1f * epsP_0_2_ad + (1.0f - 0.1f) * st->epsP_0_2_ad_lp;
    } else {
        st->epsP_0_2_ad_lp = 0.2f * epsP_0_2_ad + (1.0f - 0.2f) * st->epsP_0_2_ad_lp;
     } */
    alpha =        6554;
    move16();/* 0.2 Q15 */
    if (sub(epsP_0_2_ad, st_fx->epsP_0_2_ad_lp_fx) < 0 )
    {
        alpha = shr(alpha,1); /* 0.1 Q15 */
    }
    st_fx->epsP_0_2_ad_lp_fx = noise_est_AR1_Qx(epsP_0_2_ad, st_fx->epsP_0_2_ad_lp_fx ,alpha);

    /* epsP_0_2_ad_lp_max = max(epsP_0_2_ad,st->epsP_0_2_ad_lp);*/
    epsP_0_2_ad_lp_max = s_max(epsP_0_2_ad, st_fx->epsP_0_2_ad_lp_fx); /* Q12 */


    /*-----------------------------------------------------------------*
     * Linear predition efficiency 2 to 16 order
     *-----------------------------------------------------------------*/

    /*  epsP_2_16 = max(0 , min(8, epsP[2] / epsP[16])); */
    Ltmp       = eps_quota_fx(epsP_h[2], epsP_l[2],
                              epsP_h[16], epsP_l[16] , 12 );  /* Word32  Q12 */
    BASOP_SATURATE_WARNING_OFF                              /* may saturate*/
    epsP_2_16  =  round_fx(L_shl(Ltmp,16));                  /* Q12+16  -16 ->  Q12 ,
                                                    NB saturation in Q12 sets max value to  7,999 */
    BASOP_SATURATE_WARNING_ON

    epsP_2_16 = s_max(0, epsP_2_16);                         /* min value is 0 , Q12 */


    /* if (epsP_2_16 > st->epsP_2_16_lp){
          st->epsP_2_16_lp = 0.2f * epsP_2_16 + (1.0f-0.2f) * st->epsP_2_16_lp;
      } else {
          st->epsP_2_16_lp = 0.03f * epsP_2_16 + (1.0f-0.03f) * st->epsP_2_16_lp;
      }

      st->epsP_2_16_lp2 = 0.02f * epsP_2_16 + (1.0f-0.02f) * st->epsP_2_16_lp2; */

    alpha =        983 ;
    move16();/* 0.03  Q15 */
    if (sub(epsP_2_16 , st_fx->epsP_2_16_lp_fx) > 0 )
    {
        alpha = 6554;
        move16();/* 0.2  Q15 */
    }
    st_fx->epsP_2_16_lp_fx = noise_est_AR1_Qx(epsP_2_16, st_fx->epsP_2_16_lp_fx , alpha);

    st_fx->epsP_2_16_lp2_fx = noise_est_AR1_Qx(epsP_2_16, st_fx->epsP_2_16_lp2_fx , 655); /* 0.02 */

    epsP_2_16_dlp = sub(st_fx->epsP_2_16_lp_fx, st_fx->epsP_2_16_lp2_fx);


    /* if (epsP_2_16_dlp < st->epsP_2_16_dlp_lp2 )  {
        st->epsP_2_16_dlp_lp2 = 0.02f * epsP_2_16_dlp + (1.0f-0.02f) * st->epsP_2_16_dlp_lp2;
    } else {
        st->epsP_2_16_dlp_lp2 = 0.05f * epsP_2_16_dlp + (1.0f-0.05f) * st->epsP_2_16_dlp_lp2;
    }*/
    alpha    = 1638;
    move16();/* 0.05  Q15 */
    if (sub(epsP_2_16_dlp , st_fx->epsP_2_16_dlp_lp2_fx) < 0 )
    {
        alpha = 655;
        move16();/* 0.02  Q15 */
    }
    st_fx->epsP_2_16_dlp_lp2_fx = noise_est_AR1_Qx(epsP_2_16_dlp, st_fx->epsP_2_16_dlp_lp2_fx , alpha);

    /* epsP_2_16_dlp_max  = max(epsP_2_16_dlp,st->epsP_2_16_dlp_lp2); */
    epsP_2_16_dlp_max  = s_max(epsP_2_16_dlp, st_fx->epsP_2_16_dlp_lp2_fx);

    /*-----------------------------------------------------------------*
     * long term extensions of frame features
     *-----------------------------------------------------------------*/

    tmp = sub(Etot, st_fx->totalNoise_fx ); /* Q8 */
    /* st->lt_tn_track = 0.03f* (Etot - st->totalNoise < 10) + 0.97f*st->lt_tn_track; */
    tmp2 = 0;
    move16();
    if( sub(tmp, 2560 ) < 0 )  /*10 in Q8 */
    {
        tmp2=32767;
        move16();
    }
    st_fx->lt_tn_track_fx = noise_est_AR1_Qx(tmp2, st_fx->lt_tn_track_fx , 983); /*0.03 in Q15 ,Q15 state*/

    /* st->lt_tn_dist = 0.03f* (Etot - st->totalNoise) + 0.97f*st->lt_tn_dist; */
    st_fx->lt_tn_dist_fx = noise_est_AR1_Qx(tmp, st_fx->lt_tn_dist_fx , 983); /*0.03 in Q15 ,Q8 state*/

    /* st->lt_Ellp_dist = 0.03f* (Etot - st->Etot_l_lp) + 0.97f*st->lt_Ellp_dist;*/
    tmp = sub(Etot,st_fx->Etot_l_lp_fx); /* Q8 */
    st_fx->lt_Ellp_dist_fx = noise_est_AR1_Qx(tmp, st_fx->lt_Ellp_dist_fx, 983); /*0.03 in Q15 ,Q8 state*/



    /* if (st->harm_cor_cnt == 0)  {
        st->lt_haco_ev = 0.03f*1.0 + 0.97f*st->lt_haco_ev;
    } else {
        st->lt_haco_ev = 0.99f*st->lt_haco_ev;
    } */
    IF ( *st_harm_cor_cnt == 0)
    {
        st_fx->lt_haco_ev_fx = noise_est_AR1_Qx((Word16)32767, st_fx->lt_haco_ev_fx, 983); /*.03 in Q15 , Q15 state */
    }
    ELSE
    {
        st_fx->lt_haco_ev_fx = mult_r(32440, st_fx->lt_haco_ev_fx); /*.99 in Q15 , Q15 state */
    }


    /* if (st->lt_tn_track < 0.05f)  {
        st->low_tn_track_cnt++;
    } else {
        st->low_tn_track_cnt=0;
    }*/
    tmp = 0;
    move16();
    move16();
    if( sub( st_fx->lt_tn_track_fx , 1638 ) < 0 )      /* 0.05 in Q15*/
    {
        tmp = add(st_fx->low_tn_track_cnt_fx, 1);
    }
    st_fx->low_tn_track_cnt_fx = tmp;
    move16();


    /* update of the long-term non-stationarity measure (between 0 and 1) */
    /* if ( (non_sta > th_sta) || (*loc_harm > 0) ) {
        st->act_pred = M_GAMMA * st->act_pred + (1-M_GAMMA) * 1;
    } else {
        st->act_pred = M_GAMMA * st->act_pred + (1-M_GAMMA) * 0;
    }*/
    Ltmp = L_mult(M_GAMMA_FX, st_fx->act_pred_fx);  /*Q15*Q15+1 --> Q31 , 32440= .99 Q15 */
    tmp  = round_fx(Ltmp);   /* Q15 */
    test();
    if ( ( L_sub (non_sta, th_sta) > 0)  /* float th_sta  NB 5e10 ,  WB 3.5e10*/
            || (*loc_harm > 0)
       )
    {
        tmp = mac_r(Ltmp, (-32768+M_GAMMA_FX), -32768);  /* (-0.01)*(-1.0) */
    }
    st_fx->act_pred_fx = tmp ;
    move16();



    /*-----------------------------------------------------------------*
     * Background noise adaptation enable flag
     *-----------------------------------------------------------------*/
    Ltmp = L_mult(cor[0], 16384);
    Ltmp = L_mac(Ltmp, cor[1], 16384);
    cor_tmp = mac_r(Ltmp, corr_shift,MAX_16);

    LepsP = eps_quota_fx(epsP_h[2] , epsP_l[2],
                         epsP_h[16], epsP_l[16] , 11 );  /* L_epsP in Q11 */
    /* note  this  epsP2/eps16  is not limited to 8  as,  epsP_2_16  is  !!  */


    st_fx->vad_2nd_stage_fx =  0;
    move16();      /* background noise present - decrement counter */

    /*
     if( ( (*st_harm_cor_cnt < 3*HC_CNT_SLOW )
          && ( ( non_sta > th_sta ) ||
          ( tmp_pc < TH_PC ) ||
          ( noise_char > 0) )
          )
          ||
          ( (st->ini_frame > 150) && (Etot - Etot_l_lp) > 10 ) ||
          ( 0.5f * (voicing[0]+voicing[1]) > cor_max ) ||
          ( epsP[2] / epsP[16] > th_eps ) ||
          ( *loc_harm > 0) ||
          ((st->act_pred > 0.8f) && (non_sta2 > th_sta))
          ) */

    Ltmp    = L_mult(cor[0], 16384);       /* Q15 + Q15(.5)) + 1    ->  Q31  */
    cor_tmp = mac_r(Ltmp, cor[1], 16384 ); /* Q31 -16               ->  Q15  */
    if ( Etot < 0 )
    {
        cor_tmp = 0;
        move16();
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

    if( (     ( sub(*st_harm_cor_cnt, (3*HC_CNT_SLOW_FX) ) < 0 )
              && ( ( L_sub(non_sta, th_sta) > 0) ||  (sub(tmp_pc, TH_PC_FX) < 0) || ( noise_char > 0)  ) )
            ||
            ( (sub(st_fx->ini_frame_fx, HE_LT_CNT_INIT_FX ) > 0 )  &&  ( sub(sub(Etot, Etot_l_lp), 2560) > 0 )  ) ||
            (  sub(cor_tmp, cor_max) > 0 ) ||  /* Q15 */
            ( L_sub(LepsP, th_eps)   > 0 ) ||  /* Q11 */
            ( *loc_harm > 0 )  ||
            ((sub(st_fx->act_pred_fx, 26214) > 0) && (L_sub(Lnon_sta2, th_sta) > 0)  ) /*act_pred in Q15 , th_sta in Q10 */
      )
    {
        st_fx->vad_2nd_stage_fx   =   1;
        move16();    /*  active signal present - increment counter  */
    }

    tmp     =  2;
    move16();    /* Signal present  */
    if(st_fx->vad_2nd_stage_fx == 0 )
    {
        tmp = -1;
        move16();    /* Background  present  */
    }
    st_fx->aEn_fx = add(st_fx->aEn_fx, tmp);


    st_fx->aEn_fx = s_min(st_fx->aEn_fx,6);
    st_fx->aEn_fx = s_max(st_fx->aEn_fx,0);




    /* Additional NNE detectors */

    /* comb_ahc_epsP = max(max(st->act_pred, st->lt_haco_ev), epsP_2_16_dlp); */
    /*                             Q15            Q15          Q12 */
    comb_ahc_epsP = s_max(s_max(shr(st_fx->act_pred_fx,15-12), shr(st_fx->lt_haco_ev_fx, 15-12)), epsP_2_16_dlp); /* Q12 */


    /* comb_hcm_epsP = max(max(st->lt_haco_ev,epsP_2_16_dlp_max),epsP_0_2_ad_lp_max); */
    /*                            Q15         Q12                 Q12              */
    comb_hcm_epsP = max(max(shr(st_fx->lt_haco_ev_fx,15-12), epsP_2_16_dlp_max), epsP_0_2_ad_lp_max); /* Q12 */

    /*haco_ev_max = max(*st_harm_cor_cnt==0,st->lt_haco_ev); */
    tmp = 0;
    move16();
    if ( *st_harm_cor_cnt == 0 )
    {
        tmp = (Word16)32767;
        move16();
    }
    haco_ev_max = s_max(tmp, st_fx->lt_haco_ev_fx);   /* Q15 */

    /* Etot_l_lp_thr = st->Etot_l_lp + (1.5f + 1.5f * (st->Etot_lp<50.0f))*st->Etot_v_h2; */
    tmp = 12288;
    move16();/* 1.5 Q13 */
    if( sub(st_fx->Etot_lp_fx, 12800 ) < 0 ) /* 50.0 in Q8 */
    {
        tmp =shl(tmp,1); /*1.5 + 1.5  Q13 */
    }
    Ltmp          = L_deposit_h(st_fx->Etot_l_lp_fx);
    Etot_l_lp_thr = round_fx(L_add(Ltmp, L_shl(L_mult(tmp, Etot_v_h2),2))); /* Q13+Q8+1 +2 = Q24 -> Q8*/

    /* enr_bgd = Etot < Etot_l_lp_thr; */
    enr_bgd     = 0;
    move16();
    if(sub(Etot, Etot_l_lp_thr ) < 0 ) /* Q8 */
    {
        enr_bgd = 1;
        move16();/* Q0 */
    }

    /* cns_bgd = (epsP_0_2 > 7.95f) && (non_sta< 1e3f); */
    cns_bgd     = 0 ;
    move16();
    test();
    if(  (sub(epsP_0_2, 32563) >0 )          /* 7.95 in Q12 */
            && (L_sub(non_sta, 1024000L) < 0 ) )     /* 1e3f in  Q10 ? */
    {
        cns_bgd = 1;
        move16(); /* Q0 */
    }

    /*lp_bgd  = epsP_2_16_dlp_max < 0.10f; */
    lp_bgd = 0;
    move16();
    if( sub(epsP_2_16_dlp_max,  410) <0 ) /*0.10 Q12 */
    {
        lp_bgd = 1;
        move16(); /* Q0 */
    }


    /* ns_mask = non_sta < 1e5f; */
    ns_mask = 0;
    move16();
    if( L_sub(non_sta, (Word32)102400000L ) < 0) /*  (1e5f in  Q10)*/
    {
        ns_mask = 1;
        move16(); /* Q0 */
    }


    /* lt_haco_mask = st->lt_haco_ev < 0.5f; */
    lt_haco_mask = 0;
    move16();
    if(  sub(st_fx->lt_haco_ev_fx, 16384 ) < 0 ) /*  ( .5 in  Q15)*/
    {
        lt_haco_mask = 1;
        move16(); /* Q0 */
    }

    /* bg_haco_mask = haco_ev_max < 0.4f;   */
    bg_haco_mask = 0;
    move16();
    if( sub(haco_ev_max, 13107) < 0 )  /* ( 0.4 in  Q15)*/
    {
        bg_haco_mask = 1;
        move16(); /* Q0 */
    }


    /* SD_1 = ( (epsP_0_2_ad > 0.5f) && (epsP_0_2 > 7.95f) ); */
    SD_1 = 0;
    move16();
    test();
    if( (sub(epsP_0_2_ad,2048) > 0)         /* 0.5 in Q12 */
            &&  (sub(epsP_0_2, 32563) > 0 ))   /* 7.95 in Q12 */
    {
        SD_1 = 1;
        move16(); /* Q0 */
    }
    SD_1_inv = sub(1, SD_1); /* Q0 */

    /* NB "STL::test()";   has a cost of 2,  using  bitwise   "s_and" ,  "s_or"  at a cost of 1 */
    /* NB only lowest bit position is used,  result is always  0 or 1  */

    /* bg_bgd3 = enr_bgd || ( ( cns_bgd || lp_bgd ) && ns_mask && lt_haco_mask && SD_1==0 ); */
    tmp     = s_and(s_and(s_and(s_or(cns_bgd, lp_bgd), ns_mask), lt_haco_mask ), SD_1_inv);
    bg_bgd3 = s_or(enr_bgd, tmp);

    /*PD_1 = (epsP_2_16_dlp_max < 0.10f ) ; */
    PD_1 = 0;
    move16();
    if( (sub( epsP_2_16_dlp_max,  410) < 0)   )   /* 0.10  in Q12 */
    {
        PD_1 = 1;
        move16(); /* Q0 */
    }

    /*PD_2 = (epsP_0_2_ad_lp_max < 0.10f ) ; */
    PD_2 = 0;
    move16();
    if( (sub( epsP_0_2_ad_lp_max,  410) < 0)   )   /* 0.10  in Q12 */
    {
        PD_2 = 1;
        move16(); /* Q0 */
    }

    /*PD_3 = (comb_ahc_epsP < 0.85f ); */
    PD_3 = 0;
    move16();
    if( (sub(comb_ahc_epsP, 3482 ) < 0)   )   /* 0.85  in Q12 */
    {
        PD_3 = 1;
        move16(); /* Q0 */
    }

    /* PD_4 = comb_ahc_epsP < 0.15f; */
    PD_4 = 0;
    move16();
    if( (sub(comb_ahc_epsP, 614) < 0)   )   /* 0.15  in Q12 */
    {
        PD_4 = 1;
        move16(); /* Q0 */
    }

    /*PD_5 = comb_hcm_epsP < 0.30f;   */
    PD_5 = 0;
    move16();
    if( (sub(comb_hcm_epsP, 1229) < 0)   )   /* 0.30  in Q12 */
    {
        PD_5 = 1;
        move16(); /* Q0 */
    }

    /* BG_1 = ( (SD_1==0) || (Etot < Etot_l_lp_thr) )
               && bg_haco_mask && (st->act_pred < 0.85f) && (st->Etot_lp < 50.0f); */
    BG_1 = 0;
    move16();
    test();
    test();
    test();
    test();
    if(  ( (SD_1 == 0) || (sub(Etot, Etot_l_lp_thr) < 0 ) )
            && (bg_haco_mask != 0) && ( sub(st_fx->act_pred_fx, 27853 ) < 0 ) /* 0.85f in Q15 */
            && (sub(st_fx->Etot_lp_fx, 50*256) < 0 ))    /* 50.0 in Q8 */
    {
        BG_1 = 1;
        move16();
    }

    /*  PAU = (st->aEn==0)
          || ( (Etot < 55.0f) && (SD_1==0)
                && ( ( PD_3 && (PD_1 || PD_2 ) ) ||   ( PD_4 || PD_5 )  ) ); */
    PAU=0;
    move16();/*Q0*/
    if(st_fx->aEn_fx == 0)
    {
        PAU = 1;
        move16();/*Q0*/
    }
    tmp = 0;
    move16();/*Q0*/
    if( sub(Etot, 55*256) <0) /*55.0 in Q8 */
    {
        tmp = 1;
        move16();/*Q0*/
    }
    tmp  =  s_and(tmp, SD_1_inv);
    PAU  =  s_or(PAU, s_and(tmp, s_or(s_and(PD_3, s_or(PD_1, PD_2 )) , s_or( PD_4, PD_5 )))) ;


    /* NEW_POS_BG = (PAU | BG_1) & bg_bgd3;   note bitwise logic in float */
    NEW_POS_BG = s_and(s_or(PAU, BG_1),bg_bgd3);

    /* Original silence detector works in most cases */
    /* aE_bgd = (st->aEn == 0);*/
    aE_bgd = 0;
    move16();
    if(st_fx->aEn_fx == 0)
    {
        aE_bgd = 1;
        move16();
    }



    /* When the signal dynamics is high and the energy is close to the background estimate */
    /* sd1_bgd =     (st->sign_dyn_lp > 15)
                  && (Etot - st->Etot_l_lp ) < 2*st->Etot_v_h2
                  && st->harm_cor_cnt > 20; */
    sd1_bgd  =  0;
    move16();
    test();
    test();
    if (  ( sub(st_fx->sign_dyn_lp_fx, 15*256) > 0 )                                /* 15 in Q8  */
            && ( sub(sub(Etot, st_fx->Etot_l_lp_fx ), shl(Etot_v_h2, 1) ) < 0 )  /*  Q8 , Etot_v_h2 has limited dynmics can be upscaled*/
            && (sub(*st_harm_cor_cnt, 20) > 0 ) )
    {
        sd1_bgd = 1;
        move16();
    }

    /* tn_ini = st->ini_frame < 150 && st->harm_cor_cnt > 5 &&
                                 ( (st->act_pred < 0.59f && st->lt_haco_ev <0.23f ) ||
                                    st->act_pred < 0.38f ||
                                    st->lt_haco_ev < 0.15f ||
                                    non_staB < 50.0f ||
                                    aE_bgd );*/

    tmp = 0;
    move16();
    test();
    test();
    test();
    test();
    test();
    if (  ( (sub(st_fx->act_pred_fx, 19333) < 0 ) && ( sub(st_fx->lt_haco_ev_fx, 7537) < 0 ) ) /* .59 in Q15 .23 in Q15 */
            || (sub(st_fx->act_pred_fx, 12452)  < 0 )  /* .38 in Q15 */
            || (sub(st_fx->lt_haco_ev_fx, 4915) < 0 ) /* .15 in Q15 */
            || (sub(non_staB, 50*256 ) < 0 ) /* 50.0 in Q8  */
            || aE_bgd != 0  )
    {
        tmp = 1;
        move16();
    }

    tn_ini = 0;
    move16();
    test();
    test();
    if ( ( sub(st_fx->ini_frame_fx, HE_LT_CNT_INIT_FX ) < 0)
            &&  ( sub(st_fx->harm_cor_cnt_fx,5) > 0 )      /* > 5  Q0 */
            &&  ( tmp != 0) )
    {
        tn_ini = 1;
        move16();
    }

    /* Energy close to the background estimate serves as a mask for other background detectors */
    /* bg_bgd2 = Etot < Etot_l_lp_thr || tn_ini ; */
    bg_bgd2 = 0;
    move16();
    test();
    if ( ( sub(Etot, Etot_l_lp_thr) < 0 )
            || (tn_ini != 0 ) )
    {
        bg_bgd2 = 1;
        move16(); /* Q0 */
    }

    updt_step = 0;
    move16(); /* Q15 */
    /*if (( bg_bgd2 && ( aE_bgd || sd1_bgd || st->lt_tn_track >0.90f || NEW_POS_BG ) )
         ||  tn_ini ) */
    tmp = 0;
    move16();
    if(  sub(st_fx->lt_tn_track_fx, 29491 ) > 0 ) /* .90 in Q15 */
    {
        tmp = 1;
        move16();
    }

    IF (  s_or(s_and(bg_bgd2  , s_or(aE_bgd, s_or(sd1_bgd, s_or(tmp, NEW_POS_BG)))), tn_ini) )
    {
        /*if( (  ( st->act_pred < 0.85f )
               && (aE_bgd !=0)
               && ( st->lt_Ellp_dist < 10  || sd1_bgd )
               && (st->lt_tn_dist<40)
               && ( ( Etot - st->totalNoise ) < 10.0f )
             )
               || ( (st->first_noise_updt == 0)  && (st->harm_cor_cnt > 80)  && (aE_bgd!=0) && (st->lt_aEn_zero > 0.5f) )
               || ( (tn_ini!=0) && ( aE_bgd != 0)  || (non_staB < 10.0) || (st->harm_cor_cnt > 80) )
               )*/

        test();
        test();
        test();
        test();
        test();
        test();
        test();  /* for the ELSE IF below*/
        test();
        test();
        test();
        test();
        test();
        test();         /* for the ELSE IF below*/

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
        test();
        test();
        IF( (   ( sub(st_fx->act_pred_fx, 27853) < 0 )                                    /* 0.85 in Q15 */
                &&  ( aE_bgd != 0 )
                &&  ( (sub(st_fx->lt_Ellp_dist_fx, 10*256) < 0)  || ( sd1_bgd != 0 )  )       /* 10.0 in Q8*/
                &&  ( sub(st_fx->lt_tn_dist_fx, 40*256 ) < 0 )                                /* 40.0 in Q8*/
                &&  ( sub(sub(Etot, st_fx->totalNoise_fx), 10*256)  < 0 )                     /* 10.0 in Q8*/
            )
            || ( (st_fx->first_noise_updt_fx == 0)  && (sub(st_fx->harm_cor_cnt_fx,80) > 0)
                 && ( aE_bgd != 0 ) && (sub(st_fx->lt_aEn_zero_fx, 16384) > 0)  /*.5 in Q15*/
               )
            ||  ( (tn_ini != 0 ) && ( ( aE_bgd != 0 ) || ( sub(non_staB, 10*256) < 0 ) || (sub(st_fx->harm_cor_cnt_fx, 80) > 0 ) ) /* 10.0 in Q8*/
                )
          )

        {
            updt_step = 32767;
            move16();
            st_fx->first_noise_updt_fx = 1;
            FOR( i=0; i< NB_BANDS; i++ )
            {
                st_fx->bckr_fx[i] = tmpN[i];
                move32();
            }
        }
        /* else if ( ( ( st->act_pred < 0.80f ) && ( aE_bgd  || PAU )  &&  st->lt_haco_ev < 0.10f )
                     || ( ( st->act_pred < 0.70f ) && ( aE_bgd || non_staB < 17.0f ) && PAU &&  st->lt_haco_ev < 0.15f )
                     || ( st->harm_cor_cnt > 80 && st->totalNoise > 5.0f && Etot < max(1.0f,Etot_l_lp + 1.5f* st->Etot_v_h2) )
                  ||
                  ( st->harm_cor_cnt > 50 && st->first_noise_updt > 30 && aE_bgd  && st->lt_aEn_zero>0.5f )
                  || tn_ini
                ) */
        ELSE IF ( ( ( sub(st_fx->act_pred_fx, 26214) < 0     )             /* .8 in Q15*/
                    && ( ( aE_bgd != 0 )   ||  ( PAU != 0 ) )
                    && (sub(st_fx->lt_haco_ev_fx, 3277) < 0 ) )           /* .10 in q15*/
                  || (  ( sub(st_fx->act_pred_fx, 22938 ) < 0 )            /* 0.70 in Q15 */
                        && ( (aE_bgd!=0 ) || ( sub(non_staB, 17*256 ) < 0 ) )/* 17.0 in Q8 */
                        && ( PAU != 0 )
                        &&  ( sub(st_fx->lt_haco_ev_fx,4915) < 0 )            /* 0.15 in Q15 */
                     )
                  || ( (sub(st_fx->harm_cor_cnt_fx, 80)> 0 ) && (sub(st_fx->totalNoise_fx, 5*256) > 0 ) /* 5.0 in Q8 */
                       && ( sub(Etot, s_max((Word16)1*256, add(Etot_l_lp, add(st_fx->Etot_v_h2_fx,shr(st_fx->Etot_v_h2_fx,1))))) < 0) /* 1.5= 1.0+.5 */
                     )
                  || ( (sub(st_fx->harm_cor_cnt_fx,50) >0) && (sub(st_fx->first_noise_updt_fx, 30) > 0)
                       && (aE_bgd != 0)  && (sub(st_fx->lt_aEn_zero_fx, 16384) > 0) ) /*.5 in Q15*/
                  || ( tn_ini != 0 )
                )

        {
            updt_step = 3277;
            move16(); /* 0.1 in Q15 */
            /* if ( !aE_bgd &&  st->harm_cor_cnt < 50
                   &&  ( (st->act_pred > 0.6f)
                         || ( (tn_ini==0) && (Etot_l_lp - st->totalNoise < 10.0f)  && non_staB > 8.0f )
                        )
                 )
             */
            test();
            test();
            test();
            test();
            test();
            IF (    (  aE_bgd==0  )
                    &&  (  sub(st_fx->harm_cor_cnt_fx, 50) < 0 )
                    &&  ( ( sub(st_fx->act_pred_fx, 19661) > 0  )                          /* 0.6 in Q15*/
                          || (    ( tn_ini==0  )
                                  &&  (sub(sub(Etot_l_lp, st_fx->totalNoise_fx),10*256) < 0 )  /* 10.0 in Q8 */
                                  &&  (sub(non_staB, 8*256) > 0)   /*  8.0 in in Q8*/
                             )
                        )
               )
            {
                updt_step = 328;
                move16(); /* 0.01 Q15 */
            }
            /*
              IF (updt_step > 0  )
              {
              */
            st_fx->first_noise_updt_fx  =  1;
            move16();
            FOR( i=0; i< NB_BANDS; i++ )
            {
                /* st->bckr[i] = st->bckr[i] + updt_step * (tmpN[i]-st->bckr[i]);*/
                /* 32 bit state update */
                Ltmp              = L_sub(tmpN[i], st_fx->bckr_fx[i]);  /*Q_new+Q_SCALE*/
                Ltmp              = Mult_32_16(Ltmp, updt_step );       /* Q_new+Q_SCALE+15+1  -16*/
                st_fx->bckr_fx[i] = L_add(Ltmp, st_fx->bckr_fx[i]);
                move32();
            }
            /*
             } */
        }
        /*else if (aE_bgd || st->harm_cor_cnt > 100 )*/
        ELSE IF  ( (aE_bgd !=0) || (sub(st_fx->harm_cor_cnt_fx, 100) > 0))
        {
            st_fx->first_noise_updt_fx = add(st_fx->first_noise_updt_fx,1);
        }
    }
    ELSE
    {
        /* If in music lower bckr to drop further */
        test();
        test();
        IF (   (sub(st_fx->low_tn_track_cnt_fx, 300) > 0)
        && (sub(st_fx->lt_haco_ev_fx, 29491 ) > 0 )  /*.9 in Q15 */
        && (st_fx->totalNoise_fx > 0 ) )
        {
            updt_step = -655;
            move16();  /* for debug purposes */
            FOR( i=0; i< NB_BANDS;  i++ )
            {
                IF( L_sub(st_fx->bckr_fx[i], L_shl(Le_min_scaled, 1L) ) > 0 )  /* 2*E_MIN(float) in float,  here we use  2*Le_min_scaled  Q_new+Q_SCALE  */
                {
                    /* st->bckr[i] = 0.98f*st->bckr[i];  */
                    st_fx->bckr_fx[i] = Mult_32_16(st_fx->bckr_fx[i], 32113);  /* .98 in Q15 */
                    move32();  /* move to array */
                }
            }
        }
        /*st->lt_aEn_zero = 0.2f * (st->aEn==0) + (1-0.2f)  *st->lt_aEn_zero;*/
        /*  y(n+1)=        alpha*tmp            + (1-alpha)*y(n)           */
        tmp=0;
        move16();
        if( st_fx->aEn_fx == 0 )
        {
            tmp=32767;
            move16();
        }
        st_fx->lt_aEn_zero_fx  = noise_est_AR1_Qx(tmp, st_fx->lt_aEn_zero_fx, 6554); /* alpha=0.2 , Q15 */

    }


    return;
}


