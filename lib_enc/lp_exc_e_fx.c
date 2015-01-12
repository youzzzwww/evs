/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define  GAIN_PIT_MAX   19661
#define  HIGH_LTP_LIMIT 1.0f
#define  LOW_LTP_LIMIT 0.55f

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/

static Word16 adpt_enr_fx(const Word16 codec_mode, const Word16 *exc, const Word16 *h1, Word16 *y1, const Word16 L_subfr,
                          Word16 *gain, Word16 *g_corr, const Word16 clip_gain, const Word16 *xn, Word16 *xn2, Word16 *exp_ener
                          ,Word16 use_prev_sf_pit_gain
                         );

/*-------------------------------------------------------------------*
 * function lp_filt_exc_enc_fx()
 *
 * Low-pass filtering of the adaptive excitation
 * Innovation target construction
 * Gain quantization limitation
 *-------------------------------------------------------------------*/

Word16 lp_filt_exc_enc_fx(
    const Word16 codec_mode,                 /* i  : MODE1 or MODE2                                  Q0 */
    const Word32 core_brate,                 /* i  : core bitrate                                    Q0 */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode                  Q0 */
    const Word16 coder_type,                 /* i  : coding type                                     Q0 */
    const Word16 i_subfr,                    /* i  : subframe index                                  Q0 */
    Word16 *exc,                       /* i/o: pointer to excitation signal frame              Q_new */
    const Word16 *h1,                        /* i  : weighted filter input response                  Q(14+shift) */
    const Word16 *xn,                        /* i  : target vector                                   Q_new-1+shift */
    Word16 *y1,                        /* o  : zero-memory filtered adaptive excitation        Q_new-1+shift */
    Word16 *xn2,                       /* o  : target vector for innovation search             Q_new-1+shift */
    const Word16 L_subfr,                    /* i  : length of vectors for gain quantization         Q0 */
    const Word16 L_frame,                    /* i  : frame size                                      Q0 */
    Word16 *g_corr,                    /* o  : ACELP correlation values                        mant/exp */
    const Word16 clip_gain,                  /* i  : adaptive gain clipping flag                     Q0 */
    Word16 *gain_pit,                  /* o  : adaptive excitation gain                        Q14 */
    Word16 *lp_flag                    /* i/o: mode selection                                  Q0 */
)
{
    Word16 gain1, gain2, g_corr2[4], exc_tmp[5*L_SUBFR], xn2_tmp[5*L_SUBFR];
    Word16 y1_tmp[5*L_SUBFR];
    Word16 select, i, exp_ener, exp_ener1;
    Word16 wtmp, wtmp1;
    Word32 Ltmp;

    Word16 use_prev_sf_pit_gain = 0;

    gain1 = 0;
    move16();
    gain2 = 0;
    move16();

    /*-----------------------------------------------------------------*
     * Select LP filtering flag
     *-----------------------------------------------------------------*/

    IF ( sub(codec_mode,MODE1) == 0 )
    {
        test();
        test();
        IF ( (L_sub(core_brate,ACELP_3k60)==0) && (sub(i_subfr,L_SUBFR)==0 || sub(i_subfr,3*L_SUBFR)==0) )
        {
            use_prev_sf_pit_gain = 1;
        }
        test();
        test();
        test();
        test();
        IF ( ( Opt_AMR_WB || sub(coder_type,GENERIC) == 0|| sub(coder_type,TRANSITION) == 0 ) && L_sub(core_brate,ACELP_11k60) < 0 )
        {
            *lp_flag = LOW_PASS;
            move16();
        }
        ELSE IF ( L_sub(core_brate,ACELP_11k60) >= 0 && sub(coder_type,AUDIO) != 0 )
        {
            *lp_flag = NORMAL_OPERATION;
            move16();
        }
        ELSE
        {
            *lp_flag = FULL_BAND;
            move16();
        }

    }

    /*----------------------------------------------------------------*
     * Find the target energy if the adaptive exc. is not filtered
     *----------------------------------------------------------------*/
    test();
    IF( sub(codec_mode,MODE2) == 0 && sub(coder_type,100) == 0)
    {
        use_prev_sf_pit_gain = 1;
    }
    exp_ener = 0;
    move16();
    wtmp = 0;
    move16();
    test();
    IF( sub(*lp_flag,FULL_BAND) == 0 || sub(*lp_flag,NORMAL_OPERATION) == 0 )
    {
        wtmp = adpt_enr_fx( codec_mode, &exc[i_subfr], h1, y1, L_subfr, &gain1, g_corr, clip_gain, xn, xn2, &exp_ener, use_prev_sf_pit_gain);
        move16();

    }

    /*----------------------------------------------------------------*
     * Filter the adaptive excitation
     * Find the target energy if the adapt. exc. is filtered
     *----------------------------------------------------------------*/

    exp_ener1 = 0;
    move16();
    wtmp1 = 0;
    move16();
    test();
    IF( (sub(*lp_flag,LOW_PASS) == 0) || (sub(*lp_flag,NORMAL_OPERATION) == 0) )
    {
        test();
        IF( sub(codec_mode,MODE2) == 0 && sub(L_frame,L_FRAME16k) == 0 )
        {
            FOR ( i=0; i<L_subfr; i++ )
            {
                Ltmp = L_mult(6881, exc[i - 1 + i_subfr]);   /* constants in Q15 */
                Ltmp = L_mac(Ltmp, 19005, exc[i + i_subfr]);
                Ltmp = L_mac(Ltmp, 6881, exc[i + 1 + i_subfr]);
                exc_tmp[i] = round_fx(Ltmp);
            }
        }
        ELSE
        {
            FOR ( i=0; i<L_subfr; i++ )
            {
                Ltmp = L_mult(5898, exc[i - 1 + i_subfr]);   /* constants in Q15 */
                Ltmp = L_mac(Ltmp, 20972, exc[i + i_subfr]);
                Ltmp = L_mac(Ltmp, 5898, exc[i + 1 + i_subfr]);
                exc_tmp[i] = round_fx(Ltmp);
            }
        }

        wtmp1 = adpt_enr_fx( codec_mode, exc_tmp, h1, y1_tmp,L_subfr, &gain2, g_corr2, clip_gain, xn, xn2_tmp, &exp_ener1, use_prev_sf_pit_gain);

    }

    if ( sub(exp_ener, exp_ener1) < 0 )
    {
        wtmp = shr(wtmp, 1);
    }

    if ( sub(exp_ener, exp_ener1) > 0 )
    {
        wtmp1 = shr(wtmp1, 1);
    }

    /*-----------------------------------------------------------------*
     * use the best prediction (minimize quadratic error)
     *-----------------------------------------------------------------*/

    test();
    test();
    IF( ( (sub(wtmp1,wtmp) < 0) && (sub(*lp_flag,NORMAL_OPERATION) == 0) ) || (sub(*lp_flag,LOW_PASS) == 0) )
    {
        /* use the LP filter for pitch excitation prediction */
        select = LOW_PASS;
        move16();
        Copy( exc_tmp, &exc[i_subfr], L_subfr );
        Copy( y1_tmp, y1, L_subfr );
        Copy( xn2_tmp, xn2, L_subfr );

        IF(use_prev_sf_pit_gain == 0)
        {
            *gain_pit = gain2;
            move16();
            g_corr[0] = g_corr2[0];
            move16();
            g_corr[1] = g_corr2[1];
            move16();
            g_corr[2] = g_corr2[2];
            move16();
            g_corr[3] = g_corr2[3];
            move16();
        }
    }
    ELSE
    {
        /* no LP filter used for pitch excitation prediction */
        select = FULL_BAND;
        move16();
        IF(use_prev_sf_pit_gain == 0)
        {
            *gain_pit = gain1;
            move16();
        }
    }

    return select;
}

/*-------------------------------------------------------------------*
 * adpt_enr_fx()
 *
 * Find  adaptive excitation energy
 * This serves to decide about the filtering of the adaptive excitation
 *-------------------------------------------------------------------*/

static Word16 adpt_enr_fx(		/* o  : adaptive excitation energy             mant     */
    const Word16 codec_mode,  /* i  : MODE1 or MODE2                                  */
    const Word16 *exc,        /* i  : excitation vector                      Q_new    */
    const Word16 *h1,         /* i  : impuls response                        Q15      */
    Word16 *y1,         /* o  : zero-memory filtered adpt. excitation  12 bits  */
    const Word16 L_subfr,     /* i  : vector length								    */
    Word16 *gain,       /* o  : subframe adaptive gain                 Q14	    */
    Word16 *g_corr,     /* o  : correlations for adptive gain           mant/exp*/
    const Word16 clip_gain,   /* i  : adaptive gain clipping flag             Q0		*/
    const Word16 *xn,         /* i  : adaptive codebook target               12 bits	Q_new-1+shift*/
    Word16 *xn2,        /* o  : algebraic codebook target              12 bits	Q_new-1+shift*/
    Word16 *exp_ener	/* o  : adaptive excitation energy             exp      */
    , Word16 use_prev_sf_pit_gain /* i : flag to use prev sf pitch gain or not */
)
{
    Word16 ener, i;
    Word16 exc_tmp[L_FRAME16k], xn_tmp[L_FRAME16k];
    Word32 Ltmp;

    Overflow = 0;
    move16();
    conv_fx( exc, h1, y1, L_subfr );

    IF (use_prev_sf_pit_gain == 0)
    {
        *gain = corr_xy1_fx( xn, y1, g_corr, L_subfr, codec_mode == MODE2 );
        move16();

        test();
        IF( sub(L_subfr, L_SUBFR) > 0 && Overflow )
        {
            FOR(i = 0; i< L_subfr; i++)
            {
                exc_tmp[i] = mult(exc[i], 8192);
                move16();
                xn_tmp[i] = mult(xn[i], 8192);
                move16();
            }

            conv_fx( exc_tmp, h1, y1, L_subfr );
            *gain = corr_xy1_fx( xn_tmp, y1, g_corr, L_subfr, codec_mode == MODE2 );
            move16();
        }

        /* clip gain, if necessary to avoid problems at decoder */
        test();
        if(sub(clip_gain,1) == 0 && sub(*gain,15565) > 0)   /* constant in Q14 */
        {
            *gain = 15565;
            move16();
        }

        test();
        if( sub(clip_gain,2) == 0 && sub(*gain,10650) > 0 )
        {
            *gain = 10650;
            move16();
        }
    }

    /* find energy of new target xn2[] */
    updt_tar_fx( xn, xn2, y1, *gain, L_subfr );

    IF(sub(L_subfr, L_SUBFR) > 0)
    {
        /* could possibly happen in GSC */
        Ltmp = Calc_Energy_Autoscaled(xn2, 0, L_subfr, exp_ener);
        i = norm_l(Ltmp);
        ener = extract_h(L_shl(Ltmp,i));
        i = sub(31, i);
        *exp_ener = sub(i, *exp_ener);
        move16();
    }
    ELSE
    {
        ener = extract_h(Dot_product12(xn2, xn2, L_SUBFR, exp_ener));
    }

    return ener;
}

/*-------------------------------------------------------------------*
 * corr_xy1()
 *
 * Find the correlations between the target xn[] and the filtered adaptive
 * codebook excitation y1[]. ( <y1,y1>  and -2<xn,y1> )
 *-------------------------------------------------------------------*/

Word16 corr_xy1_fx(           	/* o  : pitch gain  (0..GAIN_PIT_MAX)         */
    const Word16 xn_1[],          /* i  : target signal                         */
    const Word16 y1_1[],     	    /* i  : filtered adaptive codebook excitation */
    Word16 g_corr[],  	    /* o  : correlations <y1,y1>  and -2<xn,y1>   */
    const Word16 L_subfr,    	    /* i  : vector length                         */
    const Word16 norm_flag        /* i  : flag for constraining pitch contribution */
)
{
    Word16 i;
    Word16 tmp, xx, xy, yy, exp_xy, exp_xx, exp_yy, exp_div, gain, gain_p_snr;
    Word32 Ltmp1, Ltmp2;
    Word16 xn[L_FRAME16k], y1[L_FRAME16k];

    /*----------------------------------------------------------------*
     * Find the ACELP correlations and the pitch gain
     *----------------------------------------------------------------*/

    /* Compute scalar product <y1[],y1[]> */
    Copy(xn_1, xn, L_subfr);
    Copy(y1_1, y1, L_subfr);
    Overflow  = 0;
    move16();
    Ltmp1 = Dot_product12(y1, y1, L_subfr, &exp_yy);

    IF( Overflow )
    {
        FOR(i = 0; i < L_subfr; i++)
        {
            xn[i] = mult_r(xn_1[i], 4096);
            move16();
            y1[i] = mult_r(y1_1[i], 4096);
            move16();
        }

        Ltmp1 = Dot_product12(y1, y1, L_subfr, &exp_yy);
        exp_yy = add(exp_yy, 6);
        yy = extract_h(Ltmp1);

        /* Compute scalar product <xn[],y1[]> */
        Ltmp2 = Dot_product12(xn, y1, L_subfr, &exp_xy);
        xy = extract_h(Ltmp2);
        exp_xy = add(exp_xy, 6);

        g_corr[0] = yy;
        move16();
        g_corr[1] = exp_yy;
        move16();
        /* -2.0*temp1 + 0.01 is done in Gain_enc_2 function */
        g_corr[2] = xy;
        move16();
        g_corr[3] = exp_xy;
        move16();
    }
    ELSE
    {
        yy = extract_h(Ltmp1);
        /* Ltmp1 = L_shr(Ltmp1, sub(30, exp_yy));*/

        /* Compute scalar product <xn[],y1[]> */
        Ltmp2 = Dot_product12(xn, y1, L_subfr, &exp_xy);
        xy = extract_h(Ltmp2);
        /* Ltmp2 = L_shr(Ltmp2, sub(30, exp_xy));*/

        g_corr[0] = yy;
        move16();
        g_corr[1] = exp_yy;
        move16();
        /* -2.0*temp1 + 0.01 is done in Gain_enc_2 function*/
        g_corr[2] = xy;
        move16();
        g_corr[3] = exp_xy;
        move16();
    }

    /* find pitch gain and bound it by [0,GAIN_PIT_MAX] */
    test();
    IF ( xy >= 0 && sub(s_or(yy, xy), 16384) != 0 )
    {
        /* compute gain = xy/yy */
        xy = shr(xy, 1);                       /* be sure that xy < yy */
        gain = div_s(xy, yy);
        i = sub(exp_xy, exp_yy);
        gain = shl(gain, i);                   /* saturation can occur here */

        gain = s_max(gain, 0);
        gain = s_min(gain, GAIN_PIT_MAX);      /* 1.2 in Q14 */
    }
    ELSE
    {
        gain = 0;
        move16();
    }

    /* Limit the energy of pitch contribution */
    IF (norm_flag)
    {
        /* Compute scalar product <xn[],xn[]> */
        xx = round_fx(Dot_product12_offs(xn, xn, L_subfr, &exp_xx, 1));

        /* gain_p_snr = sqrt(<xn,xn>/<y1,y1>) */
        tmp = BASOP_Util_Divide1616_Scale(xx, yy, &exp_div);
        exp_xx = add(sub(exp_xx, exp_yy), exp_div);
        tmp = Sqrt16(tmp, &exp_xx);

        /* Note: shl works as shl or shr. */
        exp_xx = sub(exp_xx,1);
        BASOP_SATURATE_WARNING_OFF
        gain_p_snr = round_fx(L_shl(Mpy_32_16_1( FL2WORD32(ACELP_GAINS_CONST), tmp), exp_xx));
        BASOP_SATURATE_WARNING_ON

        gain = s_min(gain, gain_p_snr);
    }

    return gain;
}
