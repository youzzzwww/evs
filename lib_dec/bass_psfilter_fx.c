/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"     /* Function prototypes                    */
#include "cnst_fx.h"     /* Common constants                       */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "rom_dec_fx.h"  /* Static table prototypes                */

#include "stl.h"
#include "basop_mpy.h"
#include "basop_util.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define NBPSF_L_EXTRA 120
#define BPF_STOP_STOPBAND_16     16

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/
static Word16 Pit_track_fx(Word16 syn[], Word16 T);

/*---------------------------------------------------------------------*
 * bass_psfilter_init()
 *
 * Initialisation of postfiltering variables
 *---------------------------------------------------------------------*/
void bass_psfilter_init_fx(
    Word16 old_syn[],        /* o  : Old synthesis buffer 1        */
    Word16 *mem_deemph_err,  /* o  : Error deemphasis memory       */
    Word16 *lp_ener          /* o  : long_term error signal energy */
)
{
    /* post-filter memories */
    *mem_deemph_err = 0;
    move16();
    *lp_ener = 0;
    move16();/*0 in Q8 */
    set16_fx(old_syn, 0, NBPSF_PIT_MAX);

    return;
}

/*=========================================================================*/
/* FUNCTION      : void bass_psfilter_fx ()                   */
/*-------------------------------------------------------------------------*/
/* PURPOSE       :                                  */
/*-------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                             */
/*  _ Word16 synth_in_fx[],  Q_syn2-1 i  :input synthesis (at 16kHz)        */
/*  _ const Word16 L_frame,       i  : length of the last frame         */
/*  _ Word16 pitch_buf_fx[],Q6       i  : pitch for every subfr [0,1,2,3]  */
/*  _ const Word16 bpf_off,       i  : do not use BPF when set to 1     */
/*  _ Word16 v_stab_fx,      Q15   i  : stability factor                 */
/*  _ Word16 Q_syn           i  : Q format of synth_in_fx       */
/*-------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                             */
/*   _ Word16 *mem_deemph_err, Q_syn2-1                     */
/*                  o  : Error deemphasis memory           */
/*   _ Word16 *lp_ener,      Q8  o  : long_term error signal energy     */
/*-------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                           */
/*   _ Word16 old_syn_fx[],    Q_syn2-1 i/o: NBPSF_PIT_MAX                */
/*   _ Word16 old_syn2_fx[],  Q_syn2-1 i/o: 2*PST_L_FILT                 */
/*   _ Word16 *v_stab_smooth_fx,Q15     i/o: smoothed stability factor      */
/*-------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                             */
/*           _ None                           */
/*-------------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                             */
/*=========================================================================*/

void bass_psfilter_fx(
    const Word16 Opt_AMR_WB,            /* i  : AMR-WB IO flag                   */
    Word16 synth_in_fx[],      /* i  : input synthesis (at 16kHz)        */
    const Word16 L_frame,        /* i  : length of the last frame          */
    Word16 pitch_buf_fx[],      /* i  : pitch for every subfr [0,1,2,3]   */
    Word16 old_syn_fx[],      /* i/o: NBPSF_PIT_MAX                     */
    Word16 *mem_deemph_err,    /* o  : Error deemphasis memory           */
    Word16 *lp_ener,        /* o  : long_term error signal energy     */
    const Word16 bpf_off,        /* i  : do not use BPF when set to 1      */
    Word16 v_stab_fx,        /* i  : stability factor                  */
    Word16 *v_stab_smooth_fx,    /* i/o: smoothed stability factor         */
    Word16 *mem_mean_pit,      /* i/o: average pitch memory               */
    Word16 *Track_on_hist,    /* i/o: History of half frame usage        */
    Word16 *vibrato_hist,      /* i/o: History of frames declared as vibrato*/
    Word16 *psf_att,        /* i/o: Post filter attenuation factor     */
    const Word16 coder_type,      /* i  : coder_type                         */
    Word16 Q_syn,
    Word16 bpf_noise_buf[]        /* o  : BPF error signal (at int_fs)       */
)
{
    Word16 i, j, i_subfr, T, exp, exp2;
    Word16 tmp, gain, alpha, tmp2;
    Word32 Lcorr, Lener;
    Word32 Lcorr0,Lener0,Ltmp;
    Word16 syn_buf_fx[NBPSF_PIT_MAX+L_FRAME16k+NBPSF_PIT_MAX], *syn_fx;
    Word16 syn2_buf_fx[L_FRAME16k+(2*PST_L_FILT)], *syn2_fx;
    Word16 err[L_HALFR16k], *sigPtr, *sigPtr1;
    Word16 T_sf[NB_SUBFR16k];
    Word16 subfr_pos;
    Word16 nb_subfr;
    Word16 delta_v_stab;
    Word16 T_update = 0;
    Word16 dist_pit_diff, idx_pit_min, idx_pit_max, vibrato, Track_on;
    Word16 loc_pit_max, loc_pit_min, diff_pit;
    Word16 TrackOnR, vibratR, alp_tmp;
    Word16 Q_syn2x;


    Track_on = 0;
    move16();
    vibrato = 0;
    move16();

    nb_subfr = shr(L_frame,6);
    move16();

    /*-------------------------------------------------------
     *  Initialize pointers to various synthesis buffers
     *
     *   |--------------------syn_buf--------------------------------|
     *   |-----old_syn-----|-------synth_in--------|----extrapol---- |
     *   |--NBPSF_PIT_MAX--| sf1 | sf2 | sf3 | sf4 |--NBPSF_PIT_MAX--|
     *                     |------syn2_buf---------|
     *                     |------L_frame----------|
     *                     |----bpf_noise_buf------|
     *
     *-------------------------------------------------------*/
    Q_syn2x = shl(Q_syn, 1);
    move16();

    Copy(old_syn_fx, syn_buf_fx, NBPSF_PIT_MAX);
    Copy(synth_in_fx, syn_buf_fx+NBPSF_PIT_MAX, L_frame);

    test();
    IF( !(pitch_buf_fx == NULL || bpf_off) )
    {
        FOR(i = L_TRACK_HIST-1; i > 0; i--)
        {
            mem_mean_pit[i] = mem_mean_pit[i-1];
            move16(); /*Q6 */
        }
        Ltmp = L_deposit_l(0);
        FOR(j=0; j<nb_subfr; j++)
        {
            Ltmp = L_mac(Ltmp, pitch_buf_fx[j], 8192) ; /*Q6 /4 in Q15  */
        }
        tmp = round_fx(Ltmp);   /*Q4*/
        /*tmp2 = div_s(1,nb_subfr); //Q15 */
        tmp = mult_r(tmp,8192);  /*divide per 4 (when L_frame == L_FRAME)  -> Q4*/
        if(sub(nb_subfr,5)==0)
        {
            tmp = mult_r(tmp, 26214);       /* multiply by 0.8 for case where L_frame == L_FRAME16k*/
        }
        mem_mean_pit[i] = tmp;    /*Q4 */ move16();
    }
    idx_pit_min = minimum_fx(mem_mean_pit, L_TRACK_HIST, &loc_pit_min);
    idx_pit_max = maximum_fx(mem_mean_pit, L_TRACK_HIST, &loc_pit_max);

    dist_pit_diff = abs_s(sub(idx_pit_max,idx_pit_min));
    diff_pit = sub(loc_pit_max,loc_pit_min); /*Q4 */

    if( sub(L_frame,L_FRAME16k) == 0)
    {
        diff_pit = mult_r(diff_pit,26214); /*Q4 */
    }

    test();
    test();
    test();
    if( coder_type != INACTIVE && sub(diff_pit,2<<4) >= 0  && sub(diff_pit,10<<4) < 0 && sub(dist_pit_diff,3)>= 0)
    {
        vibrato = 1;
        move16();
    }

    tmp = sum16_fx(Track_on_hist,L_TRACK_HIST);
    TrackOnR = round_fx(L_shl(L_mult0(tmp,3277),16)); /*Q15  */

    vibratR = sum16_fx(vibrato_hist,L_TRACK_HIST); /*Q0 */

    alp_tmp = sub(32767,TrackOnR); /*Q15 */

    IF( vibrato )
    {
        /*  vibratR = vibratR * vibratR * -0.009f + 1.0f;move16(); */
        tmp = round_fx(L_shl(L_mult0(vibratR,vibratR),16)); /*Q0 */
        tmp = round_fx(L_shl(L_mult(tmp,18874),9)); /*Q15 */
        vibratR = sub(32767,tmp); /*Q15 */
        alp_tmp = mult_r(alp_tmp,vibratR); /*Q15 */
    }

    alp_tmp = s_max( 3277, alp_tmp );

    IF( sub(alp_tmp,*psf_att) > 0 )
    {
        /**psf_att = add(mult_r(1638,alp_tmp),mult_r(31130,*psf_att)); //Q15 */
        *psf_att = round_fx(L_mac(L_mult(1638,alp_tmp),31130,*psf_att)); /*Q15 */
    }
    ELSE
    {
        /**psf_att = add(mult_r(13107,alp_tmp),mult_r(19661,*psf_att)); //Q15 */
        *psf_att = round_fx(L_mac(L_mult(13107,alp_tmp),19661,*psf_att)); /*Q15 */
    }
    test();
    IF( pitch_buf_fx == NULL || bpf_off )
    {
        /* do not use BPF for HQ core */
        T_update = 80;
        move16();
        set16_fx( T_sf, 0, 5 );
        syn_fx = &syn_buf_fx[NBPSF_PIT_MAX+L_frame];
        sigPtr = syn_fx-T_update;
        FOR (i=0; i<NBPSF_PIT_MAX; i++)
        {
            syn_fx[i] = sigPtr[i];
            move16();
        }
    }
    ELSE
    {
        /* extrapolation of synth_in_fx */
        FOR ( i=0; i<nb_subfr; i++ )
        {
            /* copy subframe pitch values [1,2,3,4] of the current frame */
            /*T_sf[i] = (short)(pitch_buf_fx[i] + 0.5f); */
            tmp = add(pitch_buf_fx[i],32);
            T_sf[i] = shr(tmp,6);

            IF(sub(L_frame,L_FRAME16k) == 0)
            {
                T_sf[i] = s_min(T_sf[i], NBPSF_PIT_MAX);
                move16();
            }
            ELSE
            {
                T_sf[i] = s_min(T_sf[i], PIT_MAX);
                move16();
            }
        }

        T = T_sf[sub(i,1)];
        move16();
        syn_fx = &syn_buf_fx[add(NBPSF_PIT_MAX,L_frame)];
        sigPtr = syn_fx-T;
        FOR (i=0; i<NBPSF_PIT_MAX; i++)
        {
            syn_fx[i] = sigPtr[i];
            move16();
        }
    }

    subfr_pos = 0;
    move16();
    FOR (i_subfr=0; i_subfr<L_frame; i_subfr+=L_SUBFR)
    {

        T = T_sf[subfr_pos];
        move16();

        syn_fx  = &syn_buf_fx[add(NBPSF_PIT_MAX,i_subfr)];
        syn2_fx = &syn2_buf_fx[i_subfr];

        IF (T != 0)
        {
            test();
            IF( sub(T,PIT_MIN)>=0 && Opt_AMR_WB )
            {
                T = Pit_track_fx(syn_fx, T);

                if(sub(T,T_sf[subfr_pos]) != 0)
                {
                    Track_on = 1;
                    move16();
                }
            }

            /* symetric pitch prediction : phase is opposite between harmonic */
            sigPtr = syn_fx - T;
            sigPtr1 = syn_fx + T;
            FOR (i=0; i<L_SUBFR; i++)
            {
                /* syn2_fx[i] = 0.5f*(syn_fx[i-T] + syn_fx[i+T]) */
                syn2_fx[i] = mac_r(L_mult(sigPtr[i], 16384), sigPtr1[i], 16384);
                move16();
            }

            /* gain of prediction */
            Lcorr = L_deposit_l(2);
            Lener = L_deposit_h(-32768);
            /* by using the maximum negative value
            it is possible to gain another bit
            of range for the 'Lener' sum.
            It cannot be done for Lcorr
            because it is a signed sum. */
            FOR (j=0; j<L_SUBFR; j+=2)
            {
                Lcorr0 = L_mult0( syn_fx[j], syn2_fx[j]);
                Lener0 = L_mult0(syn2_fx[j], syn2_fx[j]);
                Lcorr0 = L_mac0(Lcorr0,  syn_fx[j+1], syn2_fx[j+1]);
                Lener0 = L_mac0(Lener0, syn2_fx[j+1], syn2_fx[j+1]);

                Lcorr = L_add(Lcorr, L_shr(Lcorr0, 4));
                Lener = L_add(Lener, L_shr(Lener0, 4));
                /* this loop is not efficient but it provide a
                respectable precision while avoiding overflow. */
            }

            Lcorr = L_shr(Lcorr, 1);
            Lener = L_shr(Lener, 1);
            Lener = L_add(Lener, 0x40000001L);
            /* 'Lener' & 'Lcorr' are divided by 2 */

            Ltmp = L_abs(Lcorr);
            exp = norm_l(Ltmp);
            tmp = round_fx(L_shl(Ltmp, exp));
            exp2 = norm_l(Lener);
            tmp2 = round_fx(L_shl(Lener, exp2));
            if (sub(tmp, tmp2) > 0)
            {
                exp = sub(exp, 1);
            }
            exp2 = sub(exp, exp2); /* exponent num - exponent denom */
            /* gain = corr / ener */
            gain = div_s(round_fx(L_shl(Ltmp, exp)), tmp2);
            if (Lcorr < 0)
            {
                gain = sub(0, gain);
            }

            /* error of prediction for noise estimator */
            FOR (i=0; i<L_SUBFR; i++)
            {
                /* err[i] = syn_fx[i] - gain*syn2_fx[i] */
                err[i] = msu_r(L_shr(L_mult0(gain, syn2_fx[i]), exp2), syn_fx[i], 16384);
                move16();
                /* the sign is inverted but it is not important because
                we calculate energy with 'err[i]' x 'err[i]'
                we also divide it be two to gain some range
                so energy will be /4 */
            }
            /* alpha = post-filtering factor (0=OFF, 1=100%) */
            /* Lener += pow(10.0, 0.1 * *lp_ener) */
            /* limit alpha (post-filtering) when noise is high */
            /* 10^(0.1 x lp_ener) = 2^(0.1 x lp_ener x Log2(10)) */
            exp2 = L_Extract_lc(Mpy_32_16_1(2786635L, *lp_ener), &tmp2); /* 2786635L is Q31 : 0.1xLog2(10) / 256 (/256 because lp_ener is Q8) */
            /* Note: 'Ltmp' is still the absolute value of 'Lcorr'
                 'tmp'  is still 'Ltmp' normalized and rounded to 16 bits
            However, exp has to be recalculated because it is affected by exp = sub(exp, 1) before the div_s */
            exp = norm_l(Ltmp);
            tmp2 = sub(tmp2, 1+4); /* substract 1 because 'Lener' is divided by two */
            tmp2 = add(tmp2, Q_syn2x);
            Lener = L_add(Lener, Pow2(tmp2, exp2));
            exp2 = norm_l(Lener);
            tmp2 = round_fx(L_shl(Lener, exp2));
            if (sub(tmp, tmp2) > 0)
            {
                exp = sub(exp, 1);
            }
            exp2 = sub(exp, exp2); /* exponent num - exponent denom */
            /* alpha = corr / ener */
            alpha = shr(div_s(round_fx(L_shl(Ltmp, exp)), tmp2), exp2); /*Q15 */
            if (Lcorr < 0)
            {
                alpha = sub(0, alpha);
            }

            alpha = s_min(alpha, 16384);/*Q15 */

            alpha = mult_r(alpha,*psf_att);
            test();
            test();
            IF (sub(alpha,9830) > 0 && Track_on)
            {
                alpha = 9830;
                move16();
            }
            ELSE if (sub(alpha,13107) > 0 && vibrato )
            {
                alpha = 13107;
                move16();
            }

            alpha = s_max(alpha, 0);

            /**v_stab_smooth_fx = 0.8f * v_stab_fx + 0.2f * (*v_stab_smooth_fx); */
            tmp = mult(*v_stab_smooth_fx,6554);     /*Q15 and 6554 is 0.2 in Q15 */
            tmp2= mult(v_stab_fx,26214);/*Q15 and 26214 is 0.8 in Q15 */
            *v_stab_smooth_fx = add(tmp2,tmp);
            move16();

            /*delta_v_stab = (float) fabs(*v_stab_smooth_fx - v_stab_fx); */
            tmp = sub(*v_stab_smooth_fx,v_stab_fx); /*Q15 */
            delta_v_stab = abs_s(tmp);

            /*v_stab_fx = (float) pow(v_stab_fx, 0.5f); */
            v_stab_fx = Frac_sqrt(v_stab_fx);

            /*alpha = (1.0f + 0.15f*v_stab_fx - 2.0f*delta_v_stab) * alpha; */
            tmp = mult(2458,v_stab_fx);/*Q14 */
            tmp = add(16384,tmp);/*Q14 */
            tmp = sub(tmp,delta_v_stab);/*Q14 */
            alpha = mult(tmp,alpha);/*Q14 */

            FOR (i=0; i<L_SUBFR; i++)
            {
                /*syn2_fx[i] = alpha*(syn_fx[i]-syn2_fx[i]); */
                tmp = sub(syn_fx[i],syn2_fx[i]);
                syn2_fx[i] = mult(alpha,shl(tmp,1));
                move16();/*Q_syn2 */
            }

            /* low-frequency noise estimator (lp_ener is average in dB) */
            deemph_fx(err, 29491/*0.9*/, L_SUBFR, mem_deemph_err);   /* +20dB at 50Hz */
            Lener = L_deposit_h(-32768);
            /* by using the maximum negative value
            it is possible to gain another bit
            of range for the 'Lener' sum.
            This sum is divided by 4 */
            FOR (i=0; i<L_SUBFR; i++)
            {
                Lener = L_mac0(Lener, err[i], err[i]);
            }
            exp2 = -1-2;
            move16();
            /* 'Lener' is divided by 2 */
            IF (L_sub(Lener, 2147483647L) == 0)
            {
                Lener = L_deposit_h(-32768);
                sigPtr = err + L_SUBFR/2;
                FOR (i=0; i<L_SUBFR/2; i++)
                {
                    Lener0 = L_mult0(sigPtr[i], sigPtr[i]);
                    Lener = L_add(Lener, L_shr(L_mac0(Lener0, err[i], err[i]), 5));
                }
                exp2 = sub(exp2, 5);
            }
            Lener = L_shr(Lener, 1);
            Lener = L_add(Lener, 0x40000001L);
            /* ener = 10.0*log10(ener) */
            /* *lp_ener = 0.99f * *lp_ener + 0.01f * ener */
            tmp = norm_l(Lener);
            exp = Log2_norm_lc(L_shl(Lener, tmp));
            tmp = sub(30, tmp);
            tmp = sub(tmp, Q_syn2x);
            tmp = sub(tmp, exp2); /* take care of exponent of 'Lener' */

            Lener = L_Comp(tmp, exp);
            Lener = Mpy_32_16_1(Lener, 31565); /* 31565 = Log10(2) x 0.1 x 2^15 x 32 (*32 to utilize all 15 bits) */
            Lener = L_shl(Lener, 3);
            *lp_ener = mac_r(Lener, *lp_ener, 32440);
            move16();

            /* just write out the error signal */
            Copy( syn2_fx, bpf_noise_buf + i_subfr, L_SUBFR );
        }
        ELSE
        {
            /* symetric pitch prediction : phase is opposite between harmonic */
            FOR (i=0; i<L_SUBFR; i++)
            {
                /*syn2_fx[i] = 0.5f*(syn_fx[i-T_update]+syn_fx[i+T_update]); */
                syn2_fx[i] = mac_r(L_mult(syn_fx[i-T_update], 16384), syn_fx[i+T_update], 16384);
                move16();
            }

            Track_on = 0;
            move16();
            if( sub(coder_type,AUDIO) == 0 )  /* GSC mode without temporal component */
            {
                Track_on = 1;
                move16();
            }

            /* gain of prediction */
            Lcorr = L_deposit_l(2);
            Lener = L_deposit_h(-32768);
            /* by using the maximum negative value
            it is possible to gain another bit
            of range for the 'Lener' sum.
            It cannot be done for Lcorr
            because it is a signed sum. */
            sigPtr = syn_fx + 1;
            sigPtr1 = syn2_fx + 1;
            FOR (j=0; j<L_SUBFR; j+=2)
            {
                Lcorr0 = L_mult0( syn_fx[j], syn2_fx[j]);
                Lener0 = L_mult0(syn2_fx[j], syn2_fx[j]);
                Lcorr0 = L_mac0(Lcorr0,  sigPtr[j], sigPtr1[j]);
                Lener0 = L_mac0(Lener0, sigPtr1[j], sigPtr1[j]);

                Lcorr = L_add(Lcorr, L_shr(Lcorr0, 4));
                Lener = L_add(Lener, L_shr(Lener0, 4));
                /* this loop is not efficient but it provide a
                respectable precision while avoiding overflow. */
            }

            Lcorr = L_shr(Lcorr, 1);
            Lener = L_shr(Lener, 1);
            Lener = L_add(Lener, 0x40000001L);
            /* 'Lener' & 'Lcorr' are divided by 2 */

            Ltmp = L_abs(Lcorr);
            exp = norm_l(Ltmp);
            tmp = round_fx(L_shl(Ltmp, exp));
            exp2 = norm_l(Lener);
            tmp2 = round_fx(L_shl(Lener, exp2));
            if (sub(tmp, tmp2) > 0)
            {
                exp = sub(exp, 1);
            }
            exp2 = sub(exp, exp2); /* exponent num - exponent denom */
            /* gain = corr / ener */
            gain = div_s(round_fx(L_shl(Ltmp, exp)), tmp2);
            if (Lcorr < 0)
            {
                gain = sub(0, gain);
            }

            /* error of prediction for noise estimator */
            FOR (i=0; i<L_SUBFR; i++)
            {
                /* err[i] = syn_fx[i] - gain*syn2_fx[i] */
                err[i] = msu_r(L_shr(L_mult0(gain, syn2_fx[i]), exp2), syn_fx[i], 16384);
                move16();
                /* the sign is inverted but it is not important because
                we calculate energy with 'err[i]' x 'err[i]'
                we also divide it be two to gain some range
                so energy will be /4 */
            }
            /* alpha = post-filtering factor (0=OFF, 1=100%) */
            /* Lener += pow(10.0, 0.1 * *lp_ener) */
            /* limit alpha (post-filtering) when noise is high */
            /* 10^(0.1 x lp_ener) = 2^(0.1 x lp_ener x Log2(10)) */
            exp2 = L_Extract_lc(Mpy_32_16_1(2786635L, *lp_ener), &tmp2); /* 2786635L is Q31 : 0.1xLog2(10) / 256 (/256 because lp_ener is Q8) */
            /* Note: 'Ltmp' is still the absolute value of 'Lcorr'
                 'tmp'  is still 'Ltmp' normalized and rounded to 16 bits
            However, exp has to be recalculated because it is affected by exp = sub(exp, 1) before the div_s */
            exp = norm_l(Ltmp);
            tmp2 = sub(tmp2, 1+4); /* substract 1 because 'Lener' is divided by two */
            tmp2 = add(tmp2, Q_syn2x);
            Lener = L_add(Lener, Pow2(tmp2, exp2));
            exp2 = norm_l(Lener);
            tmp2 = round_fx(L_shl(Lener, exp2));
            if (sub(tmp, tmp2) > 0)
            {
                exp = sub(exp, 1);
            }
            exp2 = sub(exp, exp2); /* exponent num - exponent denom */
            alpha = 0;
            move16();

            /**v_stab_smooth_fx = 0.8f * v_stab_fx + 0.2f * (*v_stab_smooth_fx); */
            tmp = mult(*v_stab_smooth_fx,6554);/*Q15 and 6554 is 0.2 in Q15 */
            tmp2= mult(v_stab_fx,26214);/*Q15 and 26214 is 0.8 in Q15 */

            *v_stab_smooth_fx = add(tmp2,tmp);
            move16();

            FOR (i=0; i<L_SUBFR; i++)
            {
                syn2_fx[i] = 0;
                move16();
            }

            /* low-frequency noise estimator (lp_ener is average in dB) */
            deemph_fx(err, 29491/*0.9*/, L_SUBFR, mem_deemph_err);   /* +20dB at 50Hz */

            Lener = L_deposit_h(-32768);
            /* by using the maximum negative value
            it is possible to gain another bit
            of range for the 'Lener' sum.
            This sum is divided by 4 */
            FOR (i=0; i<L_SUBFR; i++)
            {
                Lener = L_mac0(Lener, err[i], err[i]);
            }
            exp2 = -1-2;
            move16();
            /* 'Lener' is divided by 2 */
            IF (L_sub(Lener, 2147483647L) == 0)
            {
                Lener = L_deposit_h(-32768);
                sigPtr = err + L_SUBFR/2;
                FOR (i=0; i<L_SUBFR/2; i++)
                {
                    Lener0 = L_mult0(sigPtr[i], sigPtr[i]);
                    Lener = L_add(Lener, L_shr(L_mac0(Lener0, err[i], err[i]), 5));
                }
                exp2 = sub(exp2, 5);
            }
            Lener = L_shr(Lener, 1);
            Lener = L_add(Lener, 0x40000001L);
            /* ener = 10.0*log10(ener) */
            /* *lp_ener = 0.99f * *lp_ener + 0.01f * ener */
            tmp = norm_l(Lener);
            exp = Log2_norm_lc(L_shl(Lener, tmp));
            tmp = sub(30, tmp);
            tmp = sub(tmp, Q_syn2x);
            tmp = sub(tmp, exp2); /* take care of exponent of 'Lener' */

            Lener = L_Comp(tmp, exp);
            Lener = Mpy_32_16_1(Lener, 31565); /* 31565 = Log10(2) x 0.1 x 2^15 x 32 (*32 to utilize all 15 bits) */
            Lener = L_shl(Lener, 3);
            *lp_ener = mac_r(Lener, *lp_ener, 32440);
            move16();

            /* just write out the error signal */
            Copy( syn2_fx, bpf_noise_buf + i_subfr, L_SUBFR );
        }

        subfr_pos = add(subfr_pos, 1);
    }

    /*-------------------------------------------------------*
     * update memory for next frame
     *-------------------------------------------------------*/

    FOR( i = L_TRACK_HIST-1; i > 0; i-- )
    {
        Track_on_hist[i] = Track_on_hist[i-1];
        move16();
        vibrato_hist[i] = vibrato_hist[i-1];
        move16();
    }

    Track_on_hist[i] = Track_on;
    move16();
    vibrato_hist[i] = vibrato;
    move16();

    Copy( syn_buf_fx+L_frame, old_syn_fx, NBPSF_PIT_MAX );


    return;
}

/*==============================================================================*/
/* FUNCTION      :  Word16 Pit_track_fx ( )                    */
/*------------------------------------------------------------------------------*/
/* PURPOSE       :                                 */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                              */
/*    Word16 syn[],    st_fx->Q_syn2   i  : synthesis [-PIT_MAX..L_SUBFR] */
/*    Word16 T      Q0         i  : pitch period (>= PIT_MIN)    */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                               */
/*     Word16 T      Q0                          */
/*------------------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                              */
/*==============================================================================*/

static Word16 Pit_track_fx(      /* o  : Pitch                          */
    Word16 syn[],                   /* i  : synthesis [-PIT_MAX..L_SUBFR] */
    Word16 T                        /* i  : pitch period (>= PIT_MIN)     */
)
{
    Word16 T2;
    Word16 i, j;
    Word32 Ltmp, Lcorr, Lener;
    Word32 Ltmp0, Lcorr0, Lener0;
    Word16 *v1, *v2;
    Word16 exp1, exp2;

    /*----------------------------------------------------------------*
     * Test pitch/2 to avoid continuous pitch doubling
     * (short pitch is limited to PIT_MIN (34 = 376Hz) by the encoder
     *----------------------------------------------------------------*/

    T2 = shr(T, 1);

    v1 = &syn[-NBPSF_L_EXTRA];
    v2 = &syn[add(-T2, -NBPSF_L_EXTRA)];

    Lener = L_deposit_h(-32768);
    Ltmp  = L_deposit_h(-32768);
    Lcorr = L_deposit_l(2);
    /* by using the maximum negative value
       it is possible to gain another bit
       of range for the 'Lener' sum.
       It cannot be done for Lcorr
       because it is a signed sum. */
    FOR (j = 0; j < 14; j++)
    {
        Lener0 = L_mult0(*v1, *v1);
        Ltmp0  = L_mult0(*v2, *v2);
        Lcorr0 = L_mult0(*v1++, *v2++);
        FOR (i = 1; i < (L_HALFR16k+NBPSF_L_EXTRA)/14; i++)
        {
            Lener0 = L_mac0(Lener0, *v1, *v1);
            Ltmp0  = L_mac0(Ltmp0,  *v2, *v2);
            Lcorr0 = L_mac0(Lcorr0, *v1++, *v2++);
        }
        test();
        test();
        test();
        IF (L_sub(Lener0, 2147483647L) == 0 ||
            L_sub(Ltmp0,  2147483647L) == 0 ||
            L_sub(Lcorr0, 2147483647L) == 0 || L_sub(Lcorr0, -2147483647-1L) == 0)
        {
            v1 -= i;
            move16();
            v2 -= i;
            move16();
            FOR (i = 0; i < (L_HALFR16k+NBPSF_L_EXTRA)/14; i++)
            {
                Lener = L_add(Lener, L_shr(L_mult0(*v1, *v1), 6));
                Ltmp  = L_add(Ltmp,  L_shr(L_mult0(*v2, *v2), 6));
                Lcorr = L_add(Lcorr, L_shr(L_mult0(*v1++, *v2++), 6));
            }
        }
        ELSE
        {
            Lener = L_add(Lener, L_shr(Lener0, 6));
            Ltmp  = L_add(Ltmp,  L_shr(Ltmp0,  6));
            Lcorr = L_add(Lcorr, L_shr(Lcorr0, 6));
        }
    }

    Lener = L_shr(Lener, 1);
    Lener = L_add(Lener, 0x40000001L);
    Ltmp = L_shr(Ltmp, 1);
    Ltmp = L_add(Ltmp, 0x40000001L);
    Lcorr = L_shr(Lcorr, 1);
    /* 'Lener', 'Ltmp' & 'Lcorr' are divided by 2 */

    /* cn = corr / (float)sqrt(ener*tmp) */
    exp1 = norm_l(Lener);
    exp2 = norm_l(Ltmp);
    /* Multiply the Most Significant 16 bits */
    Ltmp = L_mult0(round_fx(L_shl(Lener, exp1)), round_fx(L_shl(Ltmp, exp2)));
    exp1 = add(exp1, exp2);
    /* Correct if Odd # of Shifts */
    exp2 = s_and(exp1, 1);
    exp1 = sub(exp1, exp2);
    Ltmp = L_shr(Ltmp, exp2);
    /* Call the Integer Square Root (it will normalize again if req.) */
    Ltmp = Isqrt(Ltmp);
    /* We now do corr * 1 / sqrt(1/product) with high part of ratio only */
    exp1 = mac_r((16-1)*65536L-0x8000L, exp1, 16384);
    exp2 = norm_l(Ltmp);
    Ltmp = L_shl(Ltmp, exp2);
    exp1 = sub(exp1, exp2);
    exp2 = norm_l(Lcorr);
    Lcorr= L_shl(Lcorr, exp2);
    exp1 = sub(exp1, exp2);
    Ltmp = Mpy_32_16_1(Lcorr, round_fx(Ltmp));
    /* Go to Q31 */
    Ltmp = L_shl(Ltmp, exp1);

    /* cn = normalized correlation of pitch/2 */
    if (L_sub(Ltmp, 2040109466L) > 0) /* 0.95f in Q31 */
    {
        T = T2;
        move16();
    }

    return T;
}

/*---------------------------------------------------------------------*
 * addBassPostFilter()
 *
 * Add BPF component in cldfb domain
 *---------------------------------------------------------------------*/

void addBassPostFilterFx (const Word16            *harm_timeIn_Fx,
                          Word32                 **rAnalysis_Fx,
                          Word32                 **iAnalysis_Fx,
                          HANDLE_CLDFB_FILTER_BANK   cldfbBank_bpf_Fx,
                          Word32                  *workBuffer,
                          const Word16 timeIn_e,
                          const Word16 nTimeSlots,
                          const Word16 nTimeSlotsTotal,
                          const Word16 nBandsTotal,
                          CLDFB_SCALE_FACTOR *cldfb_scale
                         )
{
    Word16   i, scale1, scale2;
    Word16   b;
    Word16   maxBand;
    Word16   nChan;
    CLDFB_SCALE_FACTOR scale;
    const Word16  *weights_Fx;
    Word32 *tmp_R_Fx[CLDFB_NO_COL_MAX];
    Word32 *tmp_I_Fx[CLDFB_NO_COL_MAX];
    Word32 cldfbBufferReal[CLDFB_NO_COL_MAX][20];
    Word32 cldfbBufferImag[CLDFB_NO_COL_MAX][20];

    nChan = cldfbBank_bpf_Fx->no_channels;
    scale1 = scale2 = 0;
    move16();
    move16();

    weights_Fx = bpf_weights_16_Fx;
    maxBand = s_min(nChan, BPF_STOP_STOPBAND_16);

    assert(nChan <= 20);

    FOR (i=0; i < nTimeSlots; i++)
    {
        tmp_R_Fx[i] = cldfbBufferReal[i];
        tmp_I_Fx[i] = cldfbBufferImag[i];
    }

    /* do the CLDFB anlysis of filtered signal */
    cldfbAnalysisFiltering( cldfbBank_bpf_Fx,
                            tmp_R_Fx,
                            tmp_I_Fx,
                            &scale,
                            harm_timeIn_Fx,
                            timeIn_e,
                            nTimeSlots,
                            workBuffer
                          );

    /* now do the subtraction */

    IF (nTimeSlots > 0)
    {
        /* Find common scale. */
        b = s_max(cldfb_scale->lb_scale, scale.lb_scale);
        scale1 = limitScale32(sub(cldfb_scale->lb_scale, b));
        scale2 = limitScale32(sub(scale.lb_scale, b));
        cldfb_scale->lb_scale = b;
        move16();
        cldfb_scale->hb_scale = add(cldfb_scale->hb_scale, scale1);
        move16();
        /* Rescale time slots which do not have BPF contribution. */
        FOR (i=nTimeSlots; i < nTimeSlotsTotal; i++)
        {
            Scale_sig32(rAnalysis_Fx[i], nBandsTotal, scale1);
            Scale_sig32(iAnalysis_Fx[i], nBandsTotal, scale1);
        }
    }

    FOR (i=0; i < nTimeSlots; i++)
    {
        /* Compensate first bpf_weights coefficient which is scaled by 0.5 */
        rAnalysis_Fx[i][0] = L_sub(L_shl(rAnalysis_Fx[i][0],scale1),L_shl(Mpy_32_16_1(tmp_R_Fx[i][0],weights_Fx[0]),add(1, scale2)));
        move32();
        iAnalysis_Fx[i][0] = L_sub(L_shl(iAnalysis_Fx[i][0],scale1),L_shl(Mpy_32_16_1(tmp_I_Fx[i][0],weights_Fx[0]),add(1, scale2)));
        move32();

        /* loop over remaining low frequency bands */
        FOR (b=1; b < maxBand; b++)
        {
            rAnalysis_Fx[i][b] = L_sub(L_shl(rAnalysis_Fx[i][b],scale1),L_shr(Mpy_32_16_1(tmp_R_Fx[i][b],weights_Fx[b]), scale2));
            move32();
            iAnalysis_Fx[i][b] = L_sub(L_shl(iAnalysis_Fx[i][b],scale1),L_shr(Mpy_32_16_1(tmp_I_Fx[i][b],weights_Fx[b]), scale2));
            move32();
        }

        /* Rescale Bands with no BPF contribution. */
        Scale_sig32(rAnalysis_Fx[i]+maxBand, sub(nBandsTotal, maxBand), scale1);
        Scale_sig32(iAnalysis_Fx[i]+maxBand, sub(nBandsTotal, maxBand), scale1);
    }

    return;
}
