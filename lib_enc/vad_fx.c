/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_enc_fx.h"
#include "stl.h"



/*-----------------------------------------------------------------*
 * Local constants
 *-----------------------------------------------------------------*/
/* old fx constants */
#define HANGOVER_LONG             10         /* Hangover for CNG */
#define HANGOVER_LONG_HE          20         /* Hangover of CNG */
#define HANGOVER_LONG_NB          8          /* Hangover for CNG */
#define ACTIVE_FRAMES             3          /* Number of consecutive active SPEECH frames necessary to trigger HO */

/* SNR threshold curve constants for WB input */
#define SK16_2_FX    16930   /* Q14  (1.0333f)-> Linear function for clean speech */
#define SC16_2_FX    -4608   /* Q8   (-18)*/
#define NK16_2_FX    13529   /* Q15  (.41287)-> Linear function for noisy speech  */
#define NC16_2_FX     3394   /* Q8   (13.259625)*/
/* SNR threshold curve constants for NB input */
#define NK8_1_FX     3509    /*Q15 (0.1071f) Linear function for noisy speech  */
#define NC8_1_FX     4224    /*Q8  (16.5f) */
#define SK8_1_FX    12406    /*Q15 (0.3786f)  Linear function for clean speech */
#define SC8_1_FX     2834    /*Q8  (11.07f) */
#define SIGN_THR_FX	40       /*Q4   (2.5f)  Significanse threshold for per band snr calculation */
#define MIN_SNR_FX	2        /*Q4   Minimum snr used for per band snr calculation */


#define THR_M0_FX		  3379       /*   Q8 (13.2)  Zero ofset for threshod */
#define THR_K_BG_FX	      -8192      /*   Q15(-0.25) Coefficient for dependence on background level */
#define THR_K_BG_OFS_FX	  5120       /*   Q8 (20.0f) Zero offset for background level */
#define THR_K_SNR_FX      3277       /*   Q15 (0.1f) Coefficient for dependence on SNR */
#define THR_K_EV_FX       18022      /*   Q15 (0.55) Coefficient for dependence on noise variations */


#define HO_DTX_CLEAN      1          /* Hangover for dtx in clean conditions */
#define HO_DTX_NOISY      10         /* Hangover for dtx in noisy conditions */
#define HO_DTX_NOISY2     4          /* Hangover for dtx in very noisy conditions */
#define VAD_THR_MIN_FX    2688

#define ONE_LG10              2466        /*  1.0*log10(2)  in Q13 */


#define HANGOVER_LONG_FX             10         /* Hangover for CNG */
#define HANGOVER_LONG_MUSIC_FX       20         /* Hangover of CNG */
#define HANGOVER_LONG_HE_FX          20         /* Hangover of CNG */
#define HANGOVER_LONG_NB_FX          8          /* Hangover for CNG */
#define ACTIVE_FRAMES_FX             3          /* Number of consecutive active SPEECH frames necessary to trigger HO */

/* SNR threshold curve constants for WB input */
#define TH16_2_FX     8960           /* Q8   (35) -> lp SNR that separates the curves for clean speech and noisy speech */
#define TH8_1_FX     5120            /*Q8  (20.0f)  long-term SNR that separates the curves for clean speech and noisy speech */

#define TH16_2_NFLAG_FX  8960         /* Q8 (35) */
#define TH8_1_NFLAG_FX   8960         /* Q8 (35) */


#define SNR_OUTLIER_WGHT_1_FX        16384  /* Q14  (1.00)*/
#define SNR_OUTLIER_WGHT_2_FX        16548  /* Q14  (1.01)*/
#define SNR_OUTLIER_WGHT_3_FX        16712  /* Q14  (1.02)*/
#define INV_OUTLIER_THR_1_FX         3277   /* (1/10.0f) in Q15*/
#define INV_OUTLIER_THR_2_FX         5461   /* (1/6.0f) in Q15            */

#define MAX_SNR_OUTLIER_IND_FX       17     /*Q0 */
#define MAX_SNR_OUTLIER_1_FX         160    /*Q4 (10.0f)*/
#define MAX_SNR_OUTLIER_2_FX         400    /*Q4 (25.0f)*/
#define MAX_SNR_OUTLIER_3_FX         800    /*Q4 (50.0f)*/


/* snr_sum  =   "scale"   * (float)log10( L_snr_sum ) ;*/
static
Word16  vad_snr_log_fx(         /* o: Q8        */
    Word32 L_snr  /* i: Q4        */
    , Word16 scale)/*  i: scale Q13  , 10.0*log10(2)  or  1.0*log10(2) */
{
    Word16 e_snr,f_snr;
    Word32 L_tmp;

    e_snr = norm_l(L_snr);
    f_snr = Log2_norm_lc(L_shl(L_snr, e_snr));
    e_snr = sub(30-4, e_snr);
    L_tmp=Mpy_32_16(e_snr, f_snr, scale);
    return   round_fx(L_shl(L_tmp, 10));   /* Q8 */
}


void wb_vad_init_fx(
    Word16 *nb_active_frames,      /* o  : nb of consecutive active speech frames */
    Word16 *hangover_cnt,
    Word16 *lp_speech,             /* o  : long-term active speech level          */
    Word16 *nb_active_frames_he,   /* o  : nb of consecutive active speech frames */
    Word16 *hangover_cnt_he,
    Word16 *bcg_flux,              /* o  : background noise fluctuation                           */
    Word16 *soft_hangover,         /* o  : soft hangover counter                                  */
    Word16 *voiced_burst,          /* o  : consecutive voiced speech counter                      */
    Word16 *bcg_flux_init,         /* o  : initialization period for noise fluctuation estimation */
    Word16 *nb_active_frames_he1,  /* o  : nb of consecutive active speech frames 1               */
    Word16 *hangover_cnt_he1,
    Word32 *L_vad_flag_reg_H,
    Word32 *L_vad_flag_reg_L,
    Word32 *L_vad_prim_reg,
    Word16 *vad_flag_cnt_50,
    Word16 *vad_prim_cnt_16,
    Word16 *hangover_cnt_dtx
    , Word16 *hangover_cnt_music
)
{
    *hangover_cnt     = 0;
    move16();                /* Hangover counter initialized to 0                    */
    *nb_active_frames = ACTIVE_FRAMES_FX;
    move16();  /* The counter of SPEECH frames necessary to trigger HO */
    /* is set to max (-> start with hangover)          */
    *lp_speech        = 11520;
    move16();  /*Q8 (45.0)    */ /* Initialize the long-term active speech level in dB   */

    *L_vad_flag_reg_H = L_deposit_l(0);
    *L_vad_flag_reg_L = L_deposit_l(0);
    *L_vad_prim_reg = L_deposit_l(0);
    *vad_flag_cnt_50 = 0;
    move16();
    *vad_prim_cnt_16 = 0;
    move16();

    *hangover_cnt_dtx = HANGOVER_LONG_FX;
    move16();       /* hangover for DTX                                     */
    *hangover_cnt_music = HANGOVER_LONG_MUSIC_FX;
    move16();      /* hangover for MUSIC DTX                                     */

    *hangover_cnt_he = 0;
    move16();                /* Hangover counter initialized to 0                    */
    *nb_active_frames_he = ACTIVE_FRAMES_FX;
    move16();    /* The counter of SPEECH frames necessary to trigger HO */
    *bcg_flux = 1120;
    move16();               /*70 in Q4 */
    *soft_hangover = 0;
    move16();
    *voiced_burst = 0;
    move16();
    *bcg_flux_init = 50;
    move16();
    *nb_active_frames_he1 = ACTIVE_FRAMES_FX;
    move16();
    *hangover_cnt_he1 = 0;
    move16();
    return;
}


/*-----------------------------------------------------------------*
 * sign_thr_snr_acc_fx()
 *
 * accumulate snr_sum with significance thresholds
 *-----------------------------------------------------------------*/
static
void sign_thr_snr_acc_fx(
    Word32 *L_snr_sum,  /* o: Q4 */
    Word32 L_snr,       /* i: Q4 */
    Word16 sign_thr,    /* i: Q4 */
    Word16 min_snr  )   /* i: Q4 */
{
    /*if( snr >= sign_thr ) */
    Word32 L_tmp;

    L_tmp = L_deposit_l(min_snr);
    if( L_sub(L_snr, L_deposit_l(sign_thr)) >= 0 )
    {
        L_tmp = L_add(L_snr, 0);
    }
    BASOP_SATURATE_WARNING_OFF             /* may saturate in BASOP */
    *L_snr_sum = L_add(*L_snr_sum, L_tmp); /* Q4 */
    BASOP_SATURATE_WARNING_ON
}

/*-----------------------------------------------------------------*
 * dtx_hangover_addition_fx()
 *
 *-----------------------------------------------------------------*/

Word16 dtx_hangover_addition_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure */
    const Word16  localVAD,              /* i   Q0   */
    const Word16  vad_flag,              /* i   Q0   */
    const Word16  lp_snr,                /* i   Q8   */
    const Word16  cldfb_subtraction,          /* i   Q0  number of DTX-HO frames CLDFB wants to reduce  */
    Word16 *vad_hover_flag_ptr
)
{
    Word16 hangover_short_dtx  ;
    Word16 flag_dtx ;
    Word16 tmp;


    flag_dtx = 0;
    move16();

    /* Determine initial hangover length */
    hangover_short_dtx = 2;
    move16();

    test();
    test();
    if ( ( ( sub(lp_snr,(16*256)) < 0 )
            && ( sub(st_fx->input_bwidth_fx, NB)  != 0 ))
            ||  ( sub(st_fx->prim_act_he_fx, 31130) > 0 ) )  /*.95*Q15*/
    {
        hangover_short_dtx = 3;
        move16();
    }

    /* Adjust hangover according to activity history */
    if (sub(st_fx->vad_prim_cnt_16_fx, 12) > 0 )  /* 12 requires roughly > 80% primary activity */
    {
        hangover_short_dtx = add(hangover_short_dtx,2);
    }

    if (sub(st_fx->vad_flag_cnt_50_fx, 40)  > 0) /* 40 requires roughtly > 80% flag activity */
    {
        hangover_short_dtx = add(hangover_short_dtx,5);
    }

    /* Keep hangover_short lower than maximum hangover count */
    if (sub(hangover_short_dtx, HANGOVER_LONG_FX-1) > 0 )
    {
        hangover_short_dtx = (HANGOVER_LONG_FX-1);
        move16();
    }

    /* Only allow short HO if not sufficient active frames in clean speech*/

    tmp = 3;
    move16();       /* default for EVS*/
    if (sub(st_fx->core_fx,AMR_WB_CORE) == 0 )
    {
        tmp = 2;
        move16();        /* default for AMRWBIO*/
    }

    /* need to be a bit stricter with the DTXHO in very clean WB, SWB cond for EVS12k8VAD section */
    test();
    test();
    if (   ( sub(st_fx->input_bwidth_fx, NB) != 0 ) /* WB or SWB or FB */
            && ( sub(st_fx->core_fx, AMR_WB_CORE) != 0 )
            && ( sub(lp_snr, 25*256) > 0 )
       )
    {
        tmp = 2;
        move16();
    }

    /*    limit dtx hangover addition up to "tmp"  frames  in clean cond  */
    IF ( tmp != 0 )
    {
        test();
        test();
        test();
        if ( (sub(hangover_short_dtx, tmp) > 0 )
                && ( (sub(st_fx->vad_prim_cnt_16_fx, 7) < 0 )
                     || ( (sub(lp_snr, (16*256)) > 0)
                          && (sub(st_fx->prim_act_he_fx, 27853) < 0) /*0.85f*2^15 */
                        )
                   )
           )
        {
            hangover_short_dtx = tmp;
            move16();
        }
    }


    /* hangover adjustment from  combined  FFT+CLDFBVAD */
    IF (sub(st_fx->core_fx,AMR_WB_CORE) != 0 )
    {
        hangover_short_dtx = sub(hangover_short_dtx, cldfb_subtraction);
        hangover_short_dtx = s_max(hangover_short_dtx, 0);
    }

    IF ( vad_flag != 0 )  /* Speech present */
    {
        flag_dtx = 1;
        move16();

        /* Add hangover after sufficient # of active frames or sufficient activity during last second */
        test();
        if ( ( sub(st_fx->nb_active_frames_fx, ACTIVE_FRAMES_FX) >=0 )
                || (sub(st_fx->vad_flag_cnt_50_fx,45) > 0)  ) /* 45 requires roughly > 90% flag activity */
        {
            st_fx->hangover_cnt_dtx_fx = 0;
            move16();
        }

        /* inside HO period */
        test();
        if( ( sub(st_fx->hangover_cnt_dtx_fx, HANGOVER_LONG_FX) < 0)
                &&  (st_fx->hangover_cnt_dtx_fx != 0) )
        {
            st_fx->hangover_cnt_dtx_fx = add(st_fx->hangover_cnt_dtx_fx, 1);
        }
        st_fx->hangover_terminate_flag_fx = 0;
        move16();/* float fix FIX_HO_TERMINATE  */

        /* Music hangover when music detected */
        test();
        test();
        test();
        if ( (sub(st_fx->prim_act_he_fx,31129) > 0)
                && (sub(st_fx->Etot_lp_fx,40*256) > 0)
                && (sub(st_fx->vad_prim_cnt_16_fx,14) > 0)
                && (sub(st_fx->vad_flag_cnt_50_fx,48) > 0)  ) /* 45 requires roughly > 95% flag activity */
        {
            st_fx->hangover_cnt_music_fx = 0;
            move16();
        }

        /* inside Music HO period */
        test();
        if( ( sub(st_fx->hangover_cnt_music_fx, HANGOVER_LONG_MUSIC_FX) < 0)
                &&  (st_fx->hangover_cnt_music_fx != 0) )
        {
            st_fx->hangover_cnt_music_fx = add(st_fx->hangover_cnt_music_fx, 1);
        }
    }
    ELSE
    {
        /* Reset the counter of speech frames necessary to start hangover algorithm */
        if(sub(st_fx->hangover_cnt_dtx_fx,HANGOVER_LONG_FX) <0 )    /* inside HO period */
        {
            st_fx->hangover_cnt_dtx_fx = add(st_fx->hangover_cnt_dtx_fx,1);
        }
        if(sub(st_fx->hangover_cnt_music_fx,HANGOVER_LONG_MUSIC_FX) <0 )    /* inside HO period */
        {
            st_fx->hangover_cnt_music_fx = add(st_fx->hangover_cnt_music_fx,1);
        }

        /* fast terminate DTX hangover if st->hangover_terminate_flag is set */
        IF (  st_fx->hangover_terminate_flag_fx != 0  )
        {
            st_fx->hangover_cnt_fx = HANGOVER_LONG_FX;
            move16();
            st_fx->hangover_cnt_dtx_fx = HANGOVER_LONG_FX;
            move16();
            st_fx->hangover_terminate_flag_fx = 0;
            move16();
            /* Only shorten music hangover when low energy frames */
            if (sub(st_fx->Etot_lp_fx,20*256)<0)
            {
                st_fx->hangover_cnt_music_fx = HANGOVER_LONG_MUSIC_FX;
                move16();
            }
        }

        if( sub(st_fx->hangover_cnt_dtx_fx, hangover_short_dtx) <= 0)  /* "hard" hangover  */
        {
            flag_dtx = 1;
            move16();
        }

        if( sub(st_fx->hangover_cnt_music_fx, 15) <= 0)  /* "hard" hangover music */
        {
            flag_dtx = 1;
            move16();
        }
    }


    /*    NB!!  the odd  DTX-ON/VBR  dependency removed in FIP      */

    test();
    if ( flag_dtx != 0 && localVAD == 0 )
    {
        *vad_hover_flag_ptr = 1;
        move16();
    }

    return flag_dtx ;
}


/* new simplified and harmonized code  */
Word16 wb_vad_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure										*/
    const Word32 fr_bands[],              /* i  : per band input energy (contains 2 vectors)		Q_new+QSCALE*/
    Word16 *localVAD,
    Word16 *noisy_speech_HO,        /* o  : SC-VBR noisy speech HO flag                */
    Word16 *clean_speech_HO,        /* o  : SC-VBR clean speech HO flag                */
    Word16 *NB_speech_HO,           /* o  : SC-VBR NB speech HO flag                   */
    Word16 *snr_sum_he,			  /* o  : Output snr_sum as weighted spectral measure*/
    Word16 *localVAD_HE_SAD,
    Word8 *flag_noisy_speech_snr  ,   /* o  :   */
    const Word16 Q_new					  /* i  : scaling factor											Q0	*/
)
{
    Word16 i, flag=0, hangover_short;

    Word16 snr_sum, thr1=0, thr1_nb_mod, thr2=0, nk=0, nc=0, th_clean=0;
    Word16 lp_snr;  /* Q8 */
    const Word32 *pt1;
    const Word32 *pt2;
    const Word32 *pt3;

    Word16 min_snr, sign_thr;

    Word32 L_snr, L_snr_sum ;
    Word32 ftmp, ftmp1, ftmp2;
    Word16 m_noise, e_noise, e_num, m_num, snr, snr_tmp, shift_snr;

    Word16 snr_sumt;
    Word32 L_vad_thr;
    Word16 hangover_hd;
    Word16 snr_idx;
    Word16 delta1, delta2, delta3,delta4;

    Word16 flag_he1;
    Word16 stmp;
    Word32 L_msnr, L_mssnr=0, L_mssnr_hov;
    Word16 j, tmp, tmp1, tmp2 ;
    Word32 L_tmp, L_tmp1, L_tmp2;

    Word32 L_snr18,  L_snr19;          /* Q4 */
    Word32 L_msnr18, L_msnr19;         /* Q13 */
    Word16 nb_sig_snr;                 /* Q0 */

    Word16 nv;
    Word16 nv_ofs;            /* Q8 */
    Word32 L_snr_sum_HE_SAD; /* Q4 */
    Word16 snr_sum_HE_SAD;   /*Q8 log */
    Word16 sign_thr_HE_SAD, min_snr_HE_SAD;


    Word16  thr1_ol;
    Word32  L_snr_sum_ol;
    Word16  snr_sum_ol;  /* Q8 log */

    Word32 L_snr_outlier;
    Word16 snr_outlier_index;
    Word32 L_accum_ener_L;
    Word32 L_accum_ener_H;
    Word16 vad_bwidth_fx;

    vad_bwidth_fx = st_fx->input_bwidth_fx;
    move16();

    L_snr_outlier = L_deposit_l(0);
    snr_outlier_index = 0;
    move16();
    L_accum_ener_L = L_deposit_l(0);
    L_accum_ener_H = L_deposit_l(0);

    L_snr18 = L_deposit_l(0); /*   Q4*/
    L_snr19 = L_deposit_l(0); /*  Q4 */
    L_msnr18 = L_deposit_l(8192);  /* 1.0 Q13*/
    L_msnr19 = L_deposit_l(8192);  /* 1.0 Q13 */


    IF( sub(vad_bwidth_fx, NB) == 0)
    {
        st_fx->min_band_fx = 1;
        move16();
        st_fx->max_band_fx = 16;
        move16();
    }
    ELSE
    {
        st_fx->min_band_fx = 0;
        move16();
        st_fx->max_band_fx = 19;
        move16();
    }

    /*---------------------------------------------------------------------*
     * set SNR thresholds depending on the input bandwitdh
     *---------------------------------------------------------------------*/
    IF( sub(st_fx->max_band_fx,19) == 0 )  /* WB input */ /* or SWB input */
    {
        nk              = 3277;
        move16(); /*0.1 Q15 */
        nc              = 4122 ;
        move16(); /*16.1  Q8 */
        nv              = 525;
        move16();/* 2.05 Q8*/
        nv_ofs          = 422;
        move16();/* 1.65 Q8*/
        th_clean        = TH16_2_FX;
        move16();/* 35 Q8 */
        sign_thr        = 21;
        move16(); /*1.3 Q4 */
        tmp = sub(vad_bwidth_fx, WB);
        if ( tmp != 0 )
        {
            sign_thr =  28;
            move16();/*1.75f; Q4 SWB */
        }
        min_snr         = 13;
        move16(); /*0.8 Q4 WB */
        if ( tmp != 0 )
        {
            min_snr   =  4;
            move16(); /*0.25f; Q4 SWB */
        }

        sign_thr_HE_SAD = 40;
        move16();/* 2.5f Q4 */;
        min_snr_HE_SAD  =  3;
        move16(); /* 0.2f Q4 */;
    }
    ELSE                    /* NB input */
    {
        move16();
        nk              = 3277;
        move16(); /*  0.1 Q15 */
        nc              = 4096;
        move16(); /*  16.0  Q8 */
        nv              = 1024;
        move16(); /*  4.0  Q8 */
        nv_ofs          = 294 ;
        move16(); /*1.15 Q8*/
        th_clean        = TH8_1_FX;
        move16(); /*20 Q8 */
        sign_thr        = 28;
        move16(); /* 1.75 * Q4 SIGN_THR  */
        min_snr         = 4;
        move16(); /* .25 *Q4  MIN_SNR */
        sign_thr_HE_SAD = 42;
        move16(); /* 2.65f Q4 */;
        min_snr_HE_SAD  = 1;
        move16(); /* 0.05f Q4 */;
    }

    hangover_short = 0;
    move16();

    /* IF( st_fx->Opt_SC_VBR_fx != 0 ) */
    *noisy_speech_HO = 0;
    move16();
    *clean_speech_HO = 0;
    move16();
    *NB_speech_HO    = 0;
    move16();
    /* } */

    /*---------------------------------------------------------------------*
     * compute SNR for each band & total
     *---------------------------------------------------------------------*/

    lp_snr = sub(st_fx->lp_speech_fx, st_fx->lp_noise_fx); /*Q8 */

    snr_idx = 2;
    move16();
    if( sub(lp_snr,4608) > 0 )  /*18.0 Q8*/
    {
        snr_idx = 1;
        move16();
    }
    if ( sub(lp_snr,6144) > 0 ) /*24.0 Q8*/
    {
        snr_idx = 0;
        move16();
    }


    IF ( snr_idx == 0 )
    {
        stmp   = 6;
        move16();
        delta1 = 0;
        move16(); /*0.0f in Q13 */
        delta2 = 0;
        move16(); /*0.0f in Q13 */
        delta3 = 0;
        move16(); /*0.0f in Q13 */
        delta4 = 0;
        move16();


        /*vad_thr = 2.4f*lp_snr - 42.2f;
          vad_thr = min(vad_thr,  80 ); */

        L_vad_thr = -345702;
        move32()  ; /* -42.2  Q13*/
        L_vad_thr =  L_mac0(L_vad_thr,77,lp_snr)  ; /* (2.4)Q5*(lp_snr)Q8   */
        L_vad_thr =  L_min(L_vad_thr, 80*(1<<13) );
    }
    ELSE IF ( sub(snr_idx,1) == 0 )
    {
        stmp   = 6;
        move16();
        delta1 = 819;
        move16(); /*0.1f in Q13 */
        delta2 = 1638;
        move16(); /*0.2f in Q13 */
        delta3 = 1638;
        move16(); /*0.2f in Q13 */
        delta4 = 1638;
        move16(); /*0.2f in Q13 */

        /*   vad_thr = 2.4f*lp_snr - 40.2f;
             vad_thr = min(vad_thr, 80);
        */
        L_vad_thr = -329318;
        move32()  ; /* -40.2  Q13*/
        L_vad_thr =  L_mac0(L_vad_thr,77,lp_snr)  ; /* (2.4)Q5*(lp_snr)Q8   */
        L_vad_thr =  L_min(L_vad_thr, 80*(1<<13) );
    }
    ELSE
    {
        stmp   = 9;
        move16();
        delta1 = 1638;
        move16(); /*0.2f in Q13 */
        delta2 = 3277;
        move16(); /*0.4f in Q13 */
        delta3 = 2458;
        move16(); /*0.3f in Q13 */
        delta4 = 3277;
        move16(); /*0.4f in Q13 */
        /* vad_thr = 2.5f*lp_snr - 10.0f;
           vad_thr = max(vad_thr, 1);
        */
        L_vad_thr = -81920;
        move32()   ; /* -10  Q13*/
        L_vad_thr =  L_mac0(L_vad_thr,80,lp_snr)  ; /* (2.5)Q5*(lp_snr)Q8   */
        L_vad_thr =  L_max(L_vad_thr, 1*(1<<13) );
    }

    nb_sig_snr = 20;
    move16();

    pt1 = fr_bands;
    pt2 = fr_bands + NB_BANDS;
    pt3 = st_fx->bckr_fx;

    L_snr_sum        = L_deposit_l(0);
    L_snr_sum_HE_SAD = L_deposit_l(0);
    snr_sumt         = 0;
    move16();
    L_mssnr_hov      = L_deposit_l(0);
    *snr_sum_he      = 0;
    move16();
    snr_sum_HE_SAD   = 0;
    move16();


    FOR( i=st_fx->min_band_fx; i<=st_fx->max_band_fx; i++ )
    {
        ftmp  = L_add(*pt1++, 0);
        ftmp1 = L_add(*pt2++, 0);
        ftmp2 = L_add(*pt3++, 0);

        /*fr_enr = ( 0.2f * st->enrO[i] + 0.4f * ftmp + 0.4f * ftmp1 );*/
        L_tmp = Mult_32_16(st_fx->enrO_fx[i], 13107);    /* L_tmp(high word)  =   Qenr0fx*Q16+1  -16  -> Qener0+1 */
        L_tmp1 = Madd_32_16(L_tmp, ftmp, 26214);         /* 26214 = .4 in Q16 */
        L_tmp1 = Madd_32_16(L_tmp1, ftmp1, 26214);       /* L_tmp1 re_used a bit later for  final snr[i]*/

        L_tmp2 = Madd_32_16(L_tmp, ftmp, 19661);          /* 19661 =   0.3 in Q16 */
        L_tmp2 = Msub_32_16(L_tmp2, ftmp1, -32768);       /* -32768=  -0.5 in Q16 */

        IF (L_sub(ftmp,ftmp1) > 0)
        {
            /*snr[i] = ( 0.2f * st->enrO[i] + 0.4f * ftmp + 0.4f * ftmp1 ) / ftmp2 ;*/
            /*snr[i] = L_tmp1/(ftmp2) */
            IF (ftmp2 != 0)
            {
                e_num = norm_l(L_tmp1);
                m_num = extract_h(L_shl(L_tmp1,e_num));

                e_noise = norm_l(ftmp2);
                m_noise= extract_h(L_shl(ftmp2,e_noise));

                m_num = shr(m_num,1);
                shift_snr = add(sub(e_num, e_noise),15-4);

                snr_tmp = div_s(m_num, m_noise);
                L_snr = L_shr(snr_tmp, shift_snr); /* L_snr in Q4 */
            }
            ELSE
            {
                e_num = norm_l(L_tmp1);
                m_num = extract_h(L_shl(L_tmp1,e_num));

                /* if bckr[i] == 0; approx. L_snr */
                e_noise = add(30+1, abs_s(Q_new));

                m_num = shr(m_num, 1);
                shift_snr = add(sub(e_num, e_noise), 15-4);

                snr_tmp = div_s(m_num, 32767);
                L_snr = L_shr(snr_tmp, shift_snr);  /*L_snr in Q4*/
            }
        }
        ELSE
        {
            /*snr[i] = ( 0.2f * st->enrO[i] + 0.3f * ftmp + 0.5f * ftmp1 ) /  ftmp2 ;*/
            /*snr[i] =L_tmp2/( ftmp2 ) */
            IF (ftmp2 != 0)
            {
                e_num = norm_l(L_tmp2);
                m_num = extract_h(L_shl(L_tmp2,e_num));

                e_noise = norm_l(ftmp2);
                m_noise= extract_h(L_shl(ftmp2,e_noise));

                m_num = shr(m_num,1);
                shift_snr = add(sub(e_num, e_noise),15-4);

                snr_tmp = div_s(m_num, m_noise);
                L_snr = L_shr(snr_tmp, shift_snr); /* L_snr in Q4 */
            }
            ELSE
            {
                e_num = norm_l(L_tmp2);
                m_num = extract_h(L_shl(L_tmp2,e_num));

                /* if bckr[i] == 0; approx. L_snr */
                e_noise = add(30+1, abs_s(Q_new));

                m_num = shr(m_num, 1);
                shift_snr = add(sub(e_num, e_noise), 15-4);

                snr_tmp = div_s(m_num, 32767);
                L_snr = L_shr(snr_tmp, shift_snr);  /*L_snr in Q4*/
            }
        }

        if(L_sub(L_snr,2*(1<<4))<0 )
        {
            nb_sig_snr=sub(nb_sig_snr,1); /* nb_sig_snr--; */
        }
        L_snr = L_max(L_snr, 1*(1<<4)); /* if ( snr[i] < 1 ){snr[i] = 1;}*/


        /* snr[i] = (float)log10(snr[i]);  */
        snr = vad_snr_log_fx(L_snr, ONE_LG10);

        /* snr_sumt += snr[i];*/
        snr_sumt = add(snr_sumt,shr(snr,4)); /*Q4 */


        tmp = shl(snr,5);  /* Q8 -> Q13 */
        IF (sub(i,2) < 0)
        {
            tmp = add(tmp,delta1); /*Q13 */
        }
        ELSE IF (sub(i,7) < 0)
        {
            tmp = add(tmp,delta2); /*Q13 */
        }
        ELSE IF (sub(i,18) < 0)
        {
            tmp = add(tmp,delta3); /*Q13 */
        }
        ELSE
        {
            tmp = add(tmp,delta4); /*Q13 */
        }

        tmp1=tmp;
        move16();   /* ftmp1 = ftmp; */
        sub(0,0);
        if ( i < 7 )
        {
            tmp1=add(tmp,3277); /*.4 in Q13  ftmp1 = ftmp + 0.4f; */
        }

        tmp=s_min(tmp,16384); /* Q13, ftmp = min(ftmp, 2.0f); */
        tmp1=s_min(tmp1,16384); /* Q13, ftmp1 = min(ftmp1, 2.0f); */

        L_msnr = L_deposit_l(tmp);  /*msnr = 1*tmp;*/
        FOR (j=1; j<stmp; j++)
        {
            /* Q13*Q13 +1   -16   +2 = Q13 */
            L_msnr = L_shl(Mult_32_16(L_msnr,tmp),2); /*Q13 , msnr *= ftmp;*/
        }
        L_mssnr = L_add(L_mssnr,L_msnr);  /*Q13 mssnr += msnr;*/

        if ( sub(i,18) == 0 )
        {
            L_msnr18 = L_add(L_msnr, 0);    /*Q13  msnr18 = msnr;  */
        }

        if ( sub(i,19) == 0 )
        {
            L_msnr19 = L_add(L_msnr, 0);  /* Q13 , msnr19 = msnr; */
        }

        L_msnr = L_deposit_l(tmp1);  /* Q13,  msnr = 1*tmp1 ;*/

        FOR (j=1; j<stmp; j++)
        {
            L_msnr = L_shl(Mult_32_16(L_msnr,tmp1),2); /*Q13 msnr *= ftmp1;*/
        }
        L_mssnr_hov = L_add(L_mssnr_hov,L_msnr); /*Q13 mssnr_hov += msnr; */

        /* recompute after he1 modifications */
        /* snr[i] = fr_enr / st->bckr[i]  =  L_tmp1/st->bckr[i];*/
        IF (st_fx->bckr_fx[i] != 0)
        {
            e_num = norm_l(L_tmp1);
            m_num = extract_h(L_shl(L_tmp1,e_num));

            e_noise = norm_l(st_fx->bckr_fx[i]);
            m_noise= extract_h(L_shl(st_fx->bckr_fx[i],e_noise));

            m_num = shr(m_num,1);
            shift_snr = add(sub(e_num, e_noise),15-4);

            snr_tmp = div_s(m_num, m_noise);
            L_snr = L_shr(snr_tmp, shift_snr); /* L_snr in Q4 */
        }
        ELSE
        {
            e_num = norm_l(L_tmp1);
            m_num = extract_h(L_shl(L_tmp1,e_num));

            /* if bckr[i] == 0; approx. L_snr */
            e_noise = add(30+1, abs_s(Q_new));

            m_num = shr(m_num, 1);
            shift_snr = add(sub(e_num, e_noise), 15-4);

            snr_tmp = div_s(m_num, 32767);
            L_snr = L_shr(snr_tmp, shift_snr);  /*L_snr in Q4*/
        }


        /* conditional snrsum, snr_sum = snr_sum + snr[i];*/
        sign_thr_snr_acc_fx(&L_snr_sum_HE_SAD,L_snr, sign_thr_HE_SAD, min_snr_HE_SAD);
        sign_thr_snr_acc_fx(&L_snr_sum,       L_snr, sign_thr,        min_snr);

        L_snr = L_max(L_snr, 16);  /*Q4,  if( snr[i] < 1.0f ) { snr[i] = 1.0f;} */

        /* float saves all snrs in an snr[] vector ,
           in fix we only save two bands          */
        if ( sub(i,18) == 0 )
        {
            L_snr18 = L_add(L_snr, 0);    /*Q4   */
        }
        if ( sub(i,19) == 0 )
        {
            L_snr19 = L_add(L_snr, 0);  /* Q4  */
        }

        /* accumulate background noise energy in bands [0-2]  and in bands [3-19]*/
        IF(sub(i,3) < 0)
        {
            L_accum_ener_L = L_add(L_accum_ener_L , st_fx->bckr_fx[i]);/*Q_new+QSCALE */
        }
        ELSE
        {
            L_accum_ener_H = L_add(L_accum_ener_H , st_fx->bckr_fx[i]);/*Q_new+QSCALE */
        }

        /* Identify the outlier band */
        IF( L_sub(L_snr, L_snr_outlier) > 0 )
        {
            L_snr_outlier = L_add(L_snr, 0);        /*Q4*/
            snr_outlier_index = i;
            move16();
        }
    }  /* end of band loop */

    test();
    test();
    test(); /* one additional test for ELSE IF */
    IF (   (sub(st_fx->max_band_fx, 19) == 0 )
           && ( L_sub(L_snr18, 5*(1<<4)) > 0 )
           && ( L_sub(L_snr19, 5*(1<<4)) > 0 ) )
    {
        /* mssnr = (mssnr + 3*(msnr18 + msnr19)) * 0.77f; */
        /* mssnr = (mssnr*.77f + 2.31f*(msnr18 + msnr19)); */
        L_tmp1 = Mult_32_16(L_mssnr,                 25231 );            /* Q13+Q15+1-16 -->  Q13 */
        L_tmp  = Mult_32_16(L_shl(L_add(L_msnr18, L_msnr19),2),18924 );  /* Q(13+2)+Q(15-2)+1-16 --> Q13 */
        L_tmp  = L_add( L_tmp1, L_tmp);
        if ( L_sub(L_tmp, L_mssnr) > 0 )
        {
            L_mssnr = L_tmp;
        }
    }
    ELSE IF ( (snr_idx != 0)
              &&  sub(nb_sig_snr, 13) > 0 )
    {
        L_tmp = -126976;
        move32()  ;  /* -15.5  Q13 */
        L_tmp =  L_mac0(L_tmp,80,lp_snr)  ;  /* 2.5f(Q5)*lp_snr(Q8) - 15.5f  */
        if (  L_tmp > 0 )                    /* 2.5f*lp_snr - 15.5f > 0 */
        {
            L_mssnr = L_add(L_mssnr, L_tmp);   /*  mssnr += 2.5f*lp_snr - 15.5f; */
        }
    }


    /* Separated  SNR_SUM outlier modification  */
    L_snr_sum_ol = L_snr_sum;                      /* snr_sum_ol = snr_sum;  */

    test();
    test();
    test();
    IF(  ( sub(st_fx->max_band_fx, 19) == 0 )
         && L_sub(L_snr_outlier , MAX_SNR_OUTLIER_3_FX) < 0
         && sub(snr_outlier_index, 3) > 0
         && sub(snr_outlier_index, MAX_SNR_OUTLIER_IND_FX) < 0)
    {
        /* Update the total SNR only for WB signals */


        /*    corresponding float section
             if( (accum_ener_L > OUTLIER_THR_1 * accum_ener_H ) || (snr_outlier < MAX_SNR_OUTLIER_1) )
             {
                 snr_sum_ol = SNR_OUTLIER_WGHT_1 * (snr_sum_ol - snr_outlier);
             }
             else if( (accum_ener_L > OUTLIER_THR_2 * accum_ener_H ) || (snr_outlier < MAX_SNR_OUTLIER_2) )
             {
                 snr_sum_ol = SNR_OUTLIER_WGHT_2 * (snr_sum_ol - snr_outlier);
             }
             else
             {
                 snr_sum_ol = SNR_OUTLIER_WGHT_3 * (snr_sum_ol - snr_outlier);
             }
         } */

        test();
        test();
        IF( L_sub(L_accum_ener_H,  Mult_32_16(L_accum_ener_L,INV_OUTLIER_THR_1_FX)) <  0    /* float::  (accum_ener_L*INV_OUTLIER_THR_1 > accum_ener_H  )  !!!  */
            ||  L_sub(L_snr_outlier,MAX_SNR_OUTLIER_1_FX) < 0 )

        {

            L_snr_sum_ol =  L_sub(L_snr_sum_ol,L_snr_outlier); /*Q4 */

        }
        ELSE IF( L_sub(L_accum_ener_H, Mult_32_16(L_accum_ener_L,INV_OUTLIER_THR_2_FX)) <  0  /* float:: (accum_ener_L *INV_OUTLIER_THR_2 >  accum_ener_H ) !!! */
                 ||   L_sub(L_snr_outlier,MAX_SNR_OUTLIER_2_FX) < 0 )
        {
            /* L_snr_sum = SNR_OUTLIER_WGHT_2 * (snr_sum - snr_outlier); */

            /*            1.01*x    ->   (1*x + 0.01*x)    to not drop down to Q3 */
            L_tmp        = L_sub(L_snr_sum_ol,L_snr_outlier);
            L_tmp2       = Mult_32_16(L_tmp, 20972); /* 0.01(in Q21)= 20972   Q4*Q21+1-16  ->  Q10 */
            L_snr_sum_ol = L_add(L_tmp, L_shr(L_tmp2,6)); /* Q4 */
        }
        ELSE
        {
            /* L_snr_sum = SNR_OUTLIER_WGHT_3 * (snr_sum - snr_outlier);*/

            L_tmp        = L_sub(L_snr_sum_ol,L_snr_outlier);
            L_tmp2       = Mult_32_16(L_tmp, 20972); /* 0.02(in Q20)= 20972   Q4*Q20+1-16  ->  Q9 */
            L_snr_sum_ol = L_add(L_tmp, L_shr(L_tmp2, 5)); /* Q4 */

        }
    }
    /*st_fx->snr_sum_vad_fx = 0.5f * st->snr_sum_vad + 0.5f * snr_sum_ol;*/
    st_fx->L_snr_sum_vad_fx =  L_shr(L_add(st_fx->L_snr_sum_vad_fx ,L_snr_sum_ol ),1); /*Q4*/

    /*   snr_sum_ol = 10.0f * (float)log10( snr_sum_ol );   */
    snr_sum_ol = vad_snr_log_fx(L_snr_sum_ol, LG10);
    snr_sum = snr_sum_ol;
    move16();   /* note for NB no  outlier modification */

    /*  snr_sum_HE_SAD = 10.0f * (float)log10( snr_sum_HE_SAD );  */
    snr_sum_HE_SAD  = vad_snr_log_fx( L_snr_sum_HE_SAD, LG10);

    *snr_sum_he=snr_sum_HE_SAD;
    move16();  /* *snr_sum_he=snr_sum_HE_SAD; */


    /*---------------------------------------------------------------------*
    * compute thr1 for SAD decision
    *---------------------------------------------------------------------*/
    lp_snr = sub(st_fx->lp_speech_fx,st_fx->lp_noise_fx); /*Q8*/

    sub(0,0);
    IF ( sub(lp_snr, st_fx->sign_dyn_lp_fx) <0 )
    {
        lp_snr = add(lp_snr,1<<8);  /*  lp_snr += 1; */

        if (sub(lp_snr,  st_fx->sign_dyn_lp_fx) > 0 )
        {
            lp_snr  = st_fx->sign_dyn_lp_fx;
            move16();
        }
    }

    /*thr1 = nk * lp_snr + nc*1.0 +  nv * ( st->Etot_v_h2 - nv_ofs); */     /* Linear function for noisy speech */

    L_tmp = L_shl(L_mult(sub(st_fx->Etot_v_h2_fx, nv_ofs), nv ),7);   /*   Q8+Q8+1 +7 --> Q24  */
    L_tmp = L_mac(L_tmp, nc, (Word16)32767);  /*  Q8+Q15+1      =  Q24              */
    thr1  = mac_r(L_tmp,  lp_snr, nk );       /*  Q8+Q15+1             - 16 --> Q8  */


    IF (sub(lp_snr, (Word16)20*(1<<8)) > 0 )  /* if (lp_snr > 20.0f )*/
    {
        /* thr1 = thr1 + 0.3f * (lp_snr - 20.0f); */
        thr1 =   add(thr1, mult(9830, sub(lp_snr,(Word16) 20*(1<<8) )));  /* Q15*Q8+1 -16 --> Q8 */

        test();
        test();
        test();
        if( sub(st_fx->max_band_fx,16) == 0
                && sub(lp_snr,40*256) > 0
                && sub(thr1,6600) > 0
                && sub(st_fx->lp_speech_fx,11520) < 0 )
        {
            thr1 = 6600;
        }
    }




    /*---------------------------------------------------------------------*
     * WB input
     * SNR threshold computing
     * Hangover control & final VAD decision
     *---------------------------------------------------------------------*/

    IF( sub(vad_bwidth_fx, NB) != 0 )
    {

        /* Outlier Detection first calculates thr1_ol and snr_sum_ol instead of
           modyfying thr1 and snr_sum */

        thr1_ol        = thr1;
        move16();
        hangover_short = 3;
        move16();

        IF( sub(lp_snr,th_clean) < 0 )
        {
            hangover_short = 4;
            move16();

            /*   In this section the modified nk, and nc are used */
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
            IF( ( (sub(snr_outlier_index, 4) <= 0) && (sub(st_fx->last_coder_type_fx, UNVOICED) > 0) && ( st_fx->Opt_SC_VBR_fx != 0 ) )  ||
                ( (sub(snr_outlier_index, 4) <= 0) && (sub(st_fx->last_7k2_coder_type_fx, UNVOICED) > 0) && ( st_fx->Opt_SC_VBR_fx == 0 ) ) )


            {
                thr1_ol = sub(thr1_ol,(Word16)(1<<8)); /*thr1_ol = thr1 - 1.0f ; */
                /*snr_sum_ol = 10.0f * (float)log10( st_fx->L_snr_sum_vad_fx );*/
                snr_sum_ol  = vad_snr_log_fx(st_fx->L_snr_sum_vad_fx, LG10);  /* snr in Q8 */

            }
            ELSE IF ( ((sub(st_fx->last_coder_type_fx, UNVOICED) <= 0)  && (L_sub(L_snr_outlier,MAX_SNR_OUTLIER_2_FX) < 0)  && (st_fx->Opt_SC_VBR_fx != 0) ) ||
                      ((sub(st_fx->last_7k2_coder_type_fx, UNVOICED) <= 0)  && (L_sub(L_snr_outlier,MAX_SNR_OUTLIER_2_FX) < 0)  && ( st_fx->Opt_SC_VBR_fx == 0 ) ) )

            {
                /*  thr1_ol = thr1 + (float)(1.0f - 0.04f * snr_outlier); */
                L_tmp2   = Msub_32_16(  (Word32)(1<<(24-16)), L_snr_outlier, 20972 );  /* (1.0)Q24(Q8 in high 32bit word)  -  Q4*Q19+1 */
                tmp2     = round_fx( L_shl(L_tmp2,16));                                /* high word is in Q8 */
                thr1_ol  = add(thr1_ol, tmp2 );                                           /* (Q8 , Q8) */
            }
            ELSE
            {
                /*thr1_ol = thr1 + max(0, (float)(0.6f - 0.01f * L_snr_outlier));  */
                thr1_ol = thr1;
                move16();
                L_tmp2  = Msub_32_16( (Word32)614  ,  L_snr_outlier, 20972);   /* .6*1024= */ /* 0.6 Q26(Q10 in high word) -  Q4*Q21+1  */
                tmp2    = round_fx(L_shl(L_tmp2,14));                          /* Q10(high word)+ 14 -16 --> Q8*/
                if(L_tmp2 > 0)
                {
                    thr1_ol = add(thr1_ol, tmp2 );                              /* Q24 >>16 + Q8 */
                }
            }
        }

        /* apply outlier modification */
        snr_sum = snr_sum_ol;
        move16();        /*NB  s*/
        thr1    = thr1_ol;
        move16();

        /* DTX HANGOVER  is in pre_proc_fx()  */
        flag_he1  = 0;
        move16();

        IF ( L_sub(L_mssnr, L_vad_thr) > 0 )
        {
            flag_he1 = 1;
            move16();  /* he1 primary decision */
            st_fx->nb_active_frames_he1_fx = add(st_fx->nb_active_frames_he1_fx,1);   /* Counter of consecutive active speech frames */

            IF ( sub(st_fx->nb_active_frames_he1_fx,ACTIVE_FRAMES_FX) >= 0 )
            {
                st_fx->nb_active_frames_he1_fx = ACTIVE_FRAMES_FX;
                move16();
                st_fx->hangover_cnt_he1_fx = 0;
                move16();   /* Reset the counter of hangover frames after at least "active_frames" speech frames */
            }

            /* inside HO period */
            test();
            if ( sub(st_fx->hangover_cnt_he1_fx, HANGOVER_LONG_HE_FX < 0)  && st_fx->hangover_cnt_he1_fx != 0 )
            {
                st_fx->hangover_cnt_he1_fx = add(st_fx->hangover_cnt_he1_fx ,1);
            }

            if ( st_fx->soft_hangover_fx > 0 )
            {
                st_fx->soft_hangover_fx = sub(st_fx->soft_hangover_fx ,1);
            }
        }
        ELSE
        {
            /* Reset the counter of speech frames necessary to start hangover algorithm */
            st_fx->nb_active_frames_he1_fx = 0;
            move16();
        }




        IF ( sub(st_fx->voiced_burst_fx, 3) > 0 )
        {
            IF ( sub(st_fx->bcg_flux_fx, 640) < 0 ) /* Q4 */
            {
                st_fx->soft_hangover_fx = hangover_sf_tbl_fx[add(snr_idx,3)];
                move16();
            }
            ELSE
            {
                st_fx->soft_hangover_fx = hangover_sf_tbl_fx[snr_idx];
                move16();
            }
        }


        hangover_hd = hangover_hd_tbl_fx[snr_idx];
        move16();

        IF ( sub(st_fx->bcg_flux_fx, 640) < 0 )
        {
            hangover_hd = add(shr(hangover_hd,1), 1);
            move16();
        }

        /* VAD hangover for he1 */
        test();
        IF ( flag_he1 == 0  && st_fx->soft_hangover_fx > 0 )
        {
            IF ( L_sub(L_mssnr_hov, L_vad_thr) > 0 )
            {
                flag_he1 = 1;
                move16();
                st_fx->soft_hangover_fx=sub(st_fx->soft_hangover_fx,1);
            }
            ELSE
            {
                st_fx->soft_hangover_fx=0;
                move16();
            }

            if( st_fx->soft_hangover_fx < 0)
            {
                st_fx->soft_hangover_fx=0;
                move16();
            }
        }

        test();
        test();
        IF ( (flag_he1 == 0)
             && (sub(st_fx->hangover_cnt_he1_fx, hangover_hd) < 0 )
             && (st_fx->soft_hangover_fx == 0 ) )
        {
            flag_he1 = 1;
            move16();
            st_fx->hangover_cnt_he1_fx = add(st_fx->hangover_cnt_he1_fx,1);
        }



        /* Calculate background stationarity */
        test();
        IF ( flag_he1 == 0 && st_fx->first_noise_updt_fx > 0 )
        {
            IF ( sub(snr_sumt, st_fx->bcg_flux_fx) > 0 )
            {
                IF ( st_fx->bcg_flux_init_fx-- > 0 )
                {
                    IF ( sub(snr_sumt,add(st_fx->bcg_flux_fx,800)) > 0 )
                    {
                        /*st->bcg_flux = 0.9f * st->bcg_flux + (1-0.9f)*(st->bcg_flux+50);*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,29491),add(st_fx->bcg_flux_fx,800),3277); /*Q4 */
                    }
                    ELSE
                    {
                        /*st->bcg_flux = 0.9f * st->bcg_flux + (1-0.9f)*snr_sumt*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,29491),snr_sumt,3277); /*Q4 */
                    }
                }
                ELSE
                {
                    IF ( sub(snr_sumt,add(st_fx->bcg_flux_fx,160)) > 0 )
                    {
                        /*st->bcg_flux = 0.99f * st->bcg_flux + (1-0.99f)*(st->bcg_flux+10);*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,32440),add(st_fx->bcg_flux_fx,160),328); /*Q4 */
                    }
                    ELSE
                    {
                        /*st->bcg_flux = 0.99f * st->bcg_flux + (1-0.99f)*snr_sumt;*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,32440),snr_sumt,328); /*Q4 */
                    }
                }
            }
            ELSE
            {
                IF ( st_fx->bcg_flux_init_fx-- > 0 )
                {
                    IF ( sub(snr_sumt,sub(st_fx->bcg_flux_fx,480)) < 0 )
                    {
                        /*st->bcg_flux = 0.95f * st->bcg_flux + (1-0.95f)*(st->bcg_flux-30);*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,31130),sub(st_fx->bcg_flux_fx,480),1638); /*Q4 */
                    }
                    ELSE
                    {
                        /*st->bcg_flux = 0.95f * st->bcg_flux + (1-0.95f)*snr_sumt;*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,31130),snr_sumt,1638); /*Q4 */
                    }
                }
                ELSE
                {
                    IF ( sub(snr_sumt,sub(st_fx->bcg_flux_fx,160)) < 0 )
                    {
                        /*st->bcg_flux = 0.9992f * st->bcg_flux + (1-0.9992f)*(st->bcg_flux-10);*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,32742),sub(st_fx->bcg_flux_fx,160),26); /*Q4 */
                    }
                    ELSE
                    {
                        /*st->bcg_flux = 0.9992f * st->bcg_flux + (1-0.9992f)*snr_sumt;*/
                        st_fx->bcg_flux_fx =  mac_r(L_mult(st_fx->bcg_flux_fx,32742),snr_sumt,26); /*Q4 */
                    }
                }
            }

            st_fx->bcg_flux_init_fx = s_max(st_fx->bcg_flux_init_fx,0);
        }

        flag = 0;
        move16();
        *localVAD = 0;
        move16();
        /* if ( snr_sum > thr1 && flag_he1 == 1 ) *//* Speech present */
        test();

        IF ( (sub(snr_sum, thr1) > 0) &&  (sub(flag_he1,1) == 0)) /* Speech present */
        {
            flag      = 1;
            move16();
            *localVAD = 1;
            move16();

            st_fx->nb_active_frames_fx = add(st_fx->nb_active_frames_fx,1);   /* Counter of consecutive active speech frames */

            IF ( sub(st_fx->nb_active_frames_fx,ACTIVE_FRAMES_FX) >= 0 )
            {
                st_fx->nb_active_frames_fx = ACTIVE_FRAMES_FX;
                move16();
                st_fx->hangover_cnt_fx = 0;
                move16(); /* Reset the counter of hangover frames after at least "active_frames" speech frames */
            }

            /* inside HO period */
            test();
            if( sub(st_fx->hangover_cnt_fx,HANGOVER_LONG_FX) < 0
                    && st_fx->hangover_cnt_fx != 0 )
            {
                st_fx->hangover_cnt_fx = add(st_fx->hangover_cnt_fx,1);
            }
        }
        ELSE
        {
            /* Reset the counter of speech frames necessary to start hangover algorithm */
            st_fx->nb_active_frames_fx = 0;
            move16();

            if( sub(st_fx->hangover_cnt_fx,HANGOVER_LONG_FX) < 0 )    /* inside HO period */
            {
                st_fx->hangover_cnt_fx = add(st_fx->hangover_cnt_fx,1);
            }


            IF( sub(st_fx->hangover_cnt_fx, hangover_short) <= 0 )  /* "hard" hangover  */
            {
                test();
                test();
                if ( (sub(lp_snr,th_clean) < 0)
                && (st_fx->Opt_SC_VBR_fx != 0 )
                && (sub(st_fx->hangover_cnt_fx, 2) >= 0) )
                {
                    *noisy_speech_HO = 1;
                    move16();
                }
                test();
                test();
                if ( (sub(lp_snr,th_clean) >= 0)
                        && (st_fx->Opt_SC_VBR_fx != 0 )
                        && (sub(st_fx->hangover_cnt_fx, 2) >= 0) )
                {
                    *clean_speech_HO = 1;
                    move16();
                }
                flag = 1;
                move16();  /*HO*/
            }
        }


        /* localVAD and vad_flag for HE-SAD - in parallel with normal localVAD and vad_flag */

        *localVAD_HE_SAD = 0;
        move16();

        test();
        IF ( (sub(snr_sum_HE_SAD, thr1) > 0)
             && (sub(flag_he1, 1) == 0) ) /* Speech present */
        {

            *localVAD_HE_SAD = 1;
            move16();
        }
    } /* end of WB SWB  */

    /*---------------------------------------------------------------------*
     * NB input
     * SNR threshold computing
     * Hangover control & final VAD decision
     *---------------------------------------------------------------------*/

    ELSE                                          /* NB input */
    {
        /* Add localVAD_HE_SAD also for NB operation for use with speech music classifier */
        *localVAD_HE_SAD = 0;
        move16();
        if (sub(snr_sum_HE_SAD, thr1) > 0 )
        {
            *localVAD_HE_SAD = 1;
            move16();
        }

        *localVAD = 0;
        move16();         /* safety inits for fx */
        IF ( sub(snr_sum,thr1) > 0 )                     /* Speech present,  possibly in hangover */
        {
            st_fx->nb_active_frames_fx = add(st_fx->nb_active_frames_fx,1);                /* Counter of consecutive active speech frames */
            IF ( sub(st_fx->nb_active_frames_fx,ACTIVE_FRAMES_FX) >= 0 )
            {
                st_fx->nb_active_frames_fx = ACTIVE_FRAMES_FX;
                move16();
                st_fx->hangover_cnt_fx = 0;
                move16();/* Reset the counter of hangover frames after at least "active_frames" speech frames */
            }

            *localVAD = 1;
            move16();
        }
        ELSE
        {
            st_fx->nb_active_frames_fx = 0;
            move16();       /* Reset the counter of speech frames necessary to start hangover algorithm */
            /* *localVAD                  = 0;        move16(); */  /* set above */
        }

        thr1_nb_mod = thr1;
        move16();                             /* thr1 may be adjusted after this point */
        IF( sub(st_fx->hangover_cnt_fx,HANGOVER_LONG_NB_FX) < 0)
        {
            st_fx->hangover_cnt_fx = add(st_fx->hangover_cnt_fx,1);

            IF( sub(lp_snr, 4864 ) < 0)   /*19.0f Q8*/         /* very low SNR */
            {
                thr1_nb_mod  = sub(thr1_nb_mod , 1331);        /*thr1 -= 5.2f;*/
            }
            ELSE IF( sub(lp_snr, 8960) < 0 )    /*35 in Q8 */  /* low SNR */
            {
                thr1_nb_mod  = sub(thr1_nb_mod , 512);         /*thr1 -= 2.0f;*/
            }
        }


        thr2 = sub(thr1_nb_mod, 384);            /*thr2 = thr1 - 1.5f; , clean speech */

        /*    -dtx  condition  dependency   in noisy speech */
        tmp = 333;
        move16();              /*  1.3f;  */
        if ( st_fx->Opt_DTX_ON_fx == 0 )
        {
            tmp = 282;
            move16();              /*  1.10f; */
        }
        if (sub(lp_snr,th_clean) < 0)
        {
            thr2 = sub(thr1_nb_mod, tmp);      /*thr2 = thr1 - [ 1.10 || 1.3 ];*/
        }


        flag = 0;
        move16();
        IF ( sub(snr_sum, thr1_nb_mod) > 0 )                  /* Speech assumed present, even though lowered thr1   */
        {
            flag = 1;
            move16();
        }



        test();
        IF ( (sub(snr_sum, thr1_nb_mod) < 0)
             && (sub(snr_sum, thr2) > 0) )           /* Speech present */
        {
            flag          = 1;
            move16();
            *localVAD     = 0;
            move16();
            *NB_speech_HO = 1;
            move16();
        }
        thr1 = thr1_nb_mod ;
        move16(); /* needed for  st_fx->vadnoise_fx update below */
    } /* end of NB */



    /* *flag_noisy_speech_snr is a  Word8 parameter  */
    *flag_noisy_speech_snr = 0;
    move16();
    IF( vad_bwidth_fx != NB )
    {
        if(sub(lp_snr, TH16_2_NFLAG_FX ) < 0 )           /*now 27, original threshold: 35dB*/
        {
            *flag_noisy_speech_snr = 1;
            move16();
        }
    }
    ELSE
    {
        if(sub(lp_snr, TH8_1_NFLAG_FX ) < 0 )           /* now 20.0  */
        {
            *flag_noisy_speech_snr = 1;
            move16();
        }
    }

    /* SC-VBR */
    st_fx->vadsnr_fx   = snr_sum;
    move16(); /* for ppp, voiced_enc */
    st_fx->vadnoise_fx = thr1;
    move16(); /* used in nb for find_uv */

    /* Updates */
    st_fx->prim_act_quick_fx = mult_r(26214,st_fx->prim_act_quick_fx); /*Q15 */

    if(*localVAD != 0)
    {
        st_fx->prim_act_quick_fx = add(6554,st_fx->prim_act_quick_fx  ); /*Q15 */
    }

    st_fx->prim_act_slow_fx =  mult_r(32440,st_fx->prim_act_slow_fx); /*Q15 */

    if(*localVAD != 0)
    {
        st_fx->prim_act_slow_fx =  add(328, st_fx->prim_act_slow_fx );   /*Q15 */
    }

    tmp = st_fx->prim_act_slow_fx;
    move16();
    if (sub(st_fx->prim_act_quick_fx,st_fx->prim_act_slow_fx) <= 0)
    {
        tmp=st_fx->prim_act_quick_fx;
        move16();
    }
    /*st->prim_act = 0.1f * tmp + (1.0f-0.1f)* st->prim_act;*/
    st_fx->prim_act_fx = mac_r(L_mult(3277,tmp),29491,st_fx->prim_act_fx);



    st_fx->prim_act_quick_he_fx = mult_r(26214,st_fx->prim_act_quick_he_fx); /*Q15 */
    if(*localVAD_HE_SAD != 0)
    {
        st_fx->prim_act_quick_he_fx = add(6554,st_fx->prim_act_quick_he_fx  ); /*Q15 */
    }

    st_fx->prim_act_slow_he_fx =  mult_r(32440,st_fx->prim_act_slow_he_fx); /*Q15 */
    if(*localVAD_HE_SAD != 0)
    {
        st_fx->prim_act_slow_he_fx =  add(328, st_fx->prim_act_slow_he_fx ); /*Q15 */
    }

    tmp = st_fx->prim_act_slow_he_fx;
    move16();
    if (sub(st_fx->prim_act_quick_he_fx,st_fx->prim_act_slow_he_fx) <= 0)
    {
        tmp = st_fx->prim_act_quick_he_fx;
        move16();
    }
    st_fx->prim_act_he_fx = mac_r(L_mult(3277,tmp),29491,st_fx->prim_act_he_fx);


    if (L_and(st_fx->L_vad_flag_reg_H_fx, (Word32) 0x40000L) != 0) /* 0x4000L = 0x01L << 18 */
    {
        st_fx->vad_flag_cnt_50_fx = sub(st_fx->vad_flag_cnt_50_fx,1);
    }

    st_fx->L_vad_flag_reg_H_fx = L_shl(L_and(st_fx->L_vad_flag_reg_H_fx, (Word32) 0x3fffffffL ), 1);


    if (L_and(st_fx->L_vad_flag_reg_L_fx, (Word32) 0x40000000L) != 0)
    {
        st_fx->L_vad_flag_reg_H_fx = L_or(st_fx->L_vad_flag_reg_H_fx,  0x01L);
    }

    st_fx->L_vad_flag_reg_L_fx = L_shl(L_and(st_fx->L_vad_flag_reg_L_fx, (Word32) 0x3fffffffL ), 1);


    IF ( flag != 0 )  /* should not include the extra DTX hangover */
    {
        st_fx->L_vad_flag_reg_L_fx = L_or(st_fx->L_vad_flag_reg_L_fx, 0x01L);
        st_fx->vad_flag_cnt_50_fx  = add(st_fx->vad_flag_cnt_50_fx, 1);
    }


    if (L_and(st_fx->L_vad_prim_reg_fx, (Word32) 0x8000L) != 0) /* 0x8000L = 1L << 15 */
    {
        st_fx->vad_prim_cnt_16_fx = sub(st_fx->vad_prim_cnt_16_fx,1);
    }

    st_fx->L_vad_prim_reg_fx = L_shl(L_and(st_fx->L_vad_prim_reg_fx, (Word32) 0x3fffffffL ), 1);

    IF ( *localVAD != 0 )
    {
        st_fx->L_vad_prim_reg_fx = L_or(st_fx->L_vad_prim_reg_fx, 0x01L);
        st_fx->vad_prim_cnt_16_fx = add(st_fx->vad_prim_cnt_16_fx,1);
    }

    return flag;
}




