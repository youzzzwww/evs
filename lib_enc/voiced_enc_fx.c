/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "prot_fx.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local functions
 *--------------------------------------------------------------------*/
void synthesis_filter_fx (Word16 b[], Word16 x[], Word16 y[], Word16 buf[],
                          Word16 P, Word16 N );


/*=======================================================================================*/
/* FUNCTION      :  ppp_voiced_encoder_fx()                                              */
/*---------------------------------------------------------------------------------------*/
/* PURPOSE       :                                                     */
/*---------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                                    */
/*   _ (Word16)       delay_fx: open loop pitch, Q6 - WRONG, it is Q0                */
/*   _ (Word16)      vadsnr_fx: SNR for current frame Q7                   */
/*   _ (Word16)  prevCW_lag_fx: Previous lag, Q0                     */
/*   _ (Word16 *)    in_fx : residual signal (Q_res)                   */
/*   _ (Word16 *)    lpc1_fx : prev frame de-emphasized LPC Q12              */
/*   _ (Word16 *)    lpc2_fx : current frame de-emphasized LPC Q12             */
/*   _ (Word16 *)    exc_fx  : prrevious frame quantized excitation (Q_exc)       */
/*   _ (Word16)          Q_res: Q factor for res                         */
/*   _ (Word16)          Q_exc: Q factor for exc                         */
/*---------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                           */
/*   _ (Word16*) pitch_fx: floating pitch values for each subframe(Q6)                */
/*   _ (Word16*) out_fx: Quantized residual signal (Q0)                           */
/*   _ Encoder_State_fx *st_fx:                                */
/*                _ lastLgainE_fx - Q11                                         */
/*                _ lastHgainE_fx - Q11                                         */
/*                _ lasterbE_fx   - Q13                                         */
/*---------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                 */
/*   _ Encoder_State_fx *st_fx:                                */
/*          _  st_fx->dtfs_enc_xxxx                         */
/*          _  a nd b in   st_fx->dtfs_enc_Q                     */
/*            rest all in                 Q0                     */
/*          - bump_up_fx    - Q0                           */
/*---------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                                    */
/*  _ None.                                         */
/*---------------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                                      */
/*=======================================================================================*/

void ppp_voiced_encoder_fx(
    Encoder_State_fx *st_fx,    /* i/o: state structure */
    Word16 *in_fx,    /* i : residual signal */
    Word16 *out_fx,    /* o : Quantized residual signal */
    Word16 delay_fx,  /* i : open loop pitch */
    Word16 *lpc1_fx,  /* i : prev frame de-emphasized LPC */
    Word16* lpc2_fx,  /* i : current frame de-emphasized LPC */
    Word16 *exc_fx,    /* i: previous frame quantized excitation */
    Word16 *pitch_fx,  /* o: floating pitch values for each subframe */
    Word16 vadsnr_fx,  /* i: current frame SNR Q7, later shl by 1 for compare */
    Word16 Qres
)
{
    Word16 i;
    Word16 spike_near_edge = 0;
    Word16 flag;
    Word16 delta_lag_E = 0, PPP_MODE_E, Q_delta_lag = 0;
    Word16 out_of_bound = 0;
    Word16 tmp, tmptmp, tmptmp1;
    Word16 pl, l;
    Word16 interp_delay[3], temp_pl, temp_l;
    Word16 upper_cut_off_freq_of_interest_fx = 0;
    Word16 upper_cut_off_freq_of_interest_norm_fx = 0, upper_cut_off_freq_norm_fx = 0;

    Word16 S_fx[PIT_MAX*4+1], C_fx[PIT_MAX*4+1];
    Word16 Qtmpres;
    Word32 Ltemp, logLag, Ltemp_q;
    Word32 Lacc,Lacc1;
    Word16 Ql, Qh,n,sft,flag1;

    Word16 pf_temp1[MAXLAG_WI]; /*maynot need more than MAXLAG_WI/2+1 */
    Word16 pf_temp2[MAXLAG_WI];
    Word16 pf_temp[MAXLAG_WI];
    Word16 pf_n2[MAXLAG_WI];
    Word16 exp,expa,expb,fraca,fracb,scale,frac;
    Word32 L_tmp;

    Word16 x_fx;
    Word16 impzi_fx[160],impzo_fx[160];
    Word16 exp_ee, frac_ee;
    Word16 Qtmp;
    Word32 res_enratio_fx =0 ;
    Word16 mem_fx[10];
    Word32 energy_impz_fx = 0,tmpres_fx;
    Word32 pos_nq0_fx,neg_nq0_fx,Ltmp;
    Word32 Ltmp_32, Ltmp1_32, Ltemp1, Ltemp2, Ltemp_fx;
    Word16 Qadj;

    Word32 tmp_fx = 0,sp_hb_enratio_fx = 0,sp_enratio_fx = 0;
    Word32 low_band_en_fx;
    Word32 curr_Engy,prev_Engy;
    Word16 temp_Fs;


    DTFS_STRUCTURE_FX *CURRP_NQ_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *TMPDTFS_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *TMPDTFS2_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *TMPDTFS3_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *CURRP_Q_E_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *dtfs_temp_fx = DTFS_new_fx();



    temp_Fs = 8000;
    move16();

    if ( sub( st_fx->bwidth_fx, WB) == 0 )
    {
        temp_Fs  = 16000;
        move16();
    }

    test();
    IF (sub(st_fx->bwidth_fx,WB) == 0 || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        upper_cut_off_freq_of_interest_fx = 4000;
        move16();
        upper_cut_off_freq_of_interest_norm_fx = 10240;
        move16();/*value normalized by 12800 */
        upper_cut_off_freq_norm_fx = 16384;
        move16();/*value normalized by 12800 */
    }
    ELSE IF (sub(st_fx->bwidth_fx,NB) == 0)
    {
        upper_cut_off_freq_of_interest_fx = 3300;
        move16();
        upper_cut_off_freq_of_interest_norm_fx = 8448;
        move16();/*value normalized by 12800 */
        upper_cut_off_freq_norm_fx = 10240;
        move16();/*value normalized by 12800 */
    }

    /* Initialization */
    IF (st_fx->firstTime_voicedenc_fx)
    {
        st_fx->firstTime_voicedenc_fx = 0;
        move16();
        st_fx->dtfs_enc_lag_fx = 0;
        move16();
        st_fx->dtfs_enc_nH_fx = 0;
        move16();
        st_fx->dtfs_enc_nH_4kHz_fx = 0;
        move16();
        st_fx->dtfs_enc_upper_cut_off_freq_of_interest_fx = 3300;
        move16();
        st_fx->dtfs_enc_upper_cut_off_freq_fx = 4000;
        move16();
        set16_fx(st_fx->dtfs_enc_a_fx, 0, MAXLAG_WI);
        set16_fx(st_fx->dtfs_enc_b_fx, 0, MAXLAG_WI);
    }
    test();
    /* Figure out the PPP_MODE */
    IF ( sub(st_fx->last_ppp_mode_fx,1) == 0 && !st_fx->mode_QQF_fx )
    {
        st_fx->bump_up_fx = 1;
        move16();
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }

    /* Use the aggresive bumpups if there are two consecutive Q frames */
    /* Aggresive bump upsare only used in the second Q frame */
    if ( st_fx->last_ppp_mode_fx == 1 )
    {
        st_fx->rate_control_fx = 0;
    }

    PPP_MODE_E = 'Q';
    move16();
    pl = s_min(PIT_MAX, rint_new_fx(L_shl(st_fx->old_pitch_buf_fx[(2*NB_SUBFR)-1],10)));
    move16();
    l = s_min(PIT_MAX, rint_new_fx(L_deposit_h(delay_fx)));
    move16();
    /* st_fx->old_pitch_buf_fx in Q6*/

    /* Bump up if the lag is out_fx of range */
    test();
    IF (sub(sub(l,pl),13)>0 || sub(sub(l,pl),-11) < 0)
    {
        st_fx->bump_up_fx = 1;
        move16();
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }

    IF (sub(st_fx->last_ppp_mode_fx,1)!=0)
    {
        /* Obtain DTFS of last pl values of past excitation */
        GetSinCosTab_fx(pl,S_fx,C_fx);
        DTFS_to_fs_fx(exc_fx-pl, pl, dtfs_temp_fx, temp_Fs, 0,S_fx,C_fx);
    }

    if (sub(st_fx->last_coder_type_raw_fx,UNVOICED) == 0)
    {
        pl = l;
        move16(); /* if prev frame was sil/uv */
    }

    /* Use the out_fx array as a temp storage for currp */
    spike_near_edge = ppp_extract_pitch_period_fx(in_fx, out_fx, l, &out_of_bound,Qres) ;
    move16();
    /* out_fx in Qres */

    IF (out_of_bound == 1)
    {
        st_fx->bump_up_fx = 1;
        move16();
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }

    /* Get DTFS of current prototype */
    GetSinCosTab_fx(l,S_fx,C_fx);

    DTFS_to_fs_fx(out_fx, l, CURRP_NQ_FX, temp_Fs,  0,S_fx,C_fx);
    /* This requires input out_fx in Q0, but currently in Qres */

    /* Ensure the extracted prototype is time-synchronous to the
     * last l samples of the frame. This proves to eliminate
     * some of the PPP-CELP transition problems.
     * Convert last samples into DTFS  */
    IF (spike_near_edge != 0)
    {
        /* These two function calls are combined in one call DTFS_alignment_extract_td_fx() */
        /* DTFS_to_fs(in+L_FRAME-l, l, TMPDTFS,(short) st->input_Fs,0); */
        /* tmp = DTFS_alignment_extract(*TMPDTFS, *CURRP_NQ, 0.0, lpc2) ; */
        tmp = DTFS_alignment_extract_td_fx(out_fx, in_fx+L_FRAME-l, l) ;
        move16(); /*Q0 */
        tmp = negate(shl(tmp,2));/*Q2 */
        Q2phaseShift_fx(CURRP_NQ_FX, tmp, l, S_fx,C_fx) ;
        /* output CURRP_NQ is correct */
    }
    temp_pl = pl;
    move16();
    temp_l = l;
    move16();

    FOR(i = 0; i < NB_SUBFR; i++)
    {
        /* do the linear pitch_fx interp to drive the nb_post_filt */
        Interpol_delay_fx(interp_delay, temp_pl, temp_l, i, frac_4sf_fx); /* interp_delay in Q4 */
        pitch_fx[i] = shl(interp_delay[0],2);
        move16();/* pitch_fx in Q6 */
    }
    curr_Engy=DTFS_getEngy_P2A_fx(CURRP_NQ_FX); /*2Q where Q=CURRP_NQ_FX->Q */

    /* Restoring PPP memories when the last frame is non-PPP */
    IF (sub(st_fx->last_ppp_mode_fx,1)!=0)
    {

        st_fx->ph_offset_E_fx = 0 ;
        move16();

        /* st->prev_cw_en = DTFS_getEngy(*dtfs_temp); */
        Lacc=DTFS_getEngy_P2A_fx(dtfs_temp_fx); /*2Q where Q = dtfs_temp_fx->Q   */


        prev_Engy = L_add(Lacc, 0);

        st_fx->Q_prev_cw_en_fx=norm_l(Lacc); /* = K = headroom */

        /*st_fx->Q_prev_cw_en_fx = (Lacc==0)?31: st_fx->Q_prev_cw_en_fx; */
        if (Lacc==0)
        {
            st_fx->Q_prev_cw_en_fx = 31;
            move16();
        }

        st_fx->prev_cw_en_fx=(Word32) L_shl(Lacc,st_fx->Q_prev_cw_en_fx); /*2Q+K */
        st_fx->Q_prev_cw_en_fx = add(st_fx->Q_prev_cw_en_fx,shl(dtfs_temp_fx->Q,1));
        /* st_fx->Q_prev_cw_en_fx = 2*(dtfs_temp_fx->Q) + K */

        DTFS_copy_fx(TMPDTFS_FX,*dtfs_temp_fx); /* output = TMPDTFS_FX */

        DTFS_car2pol_fx(TMPDTFS_FX);

        logLag=log10_fx(TMPDTFS_FX->lag_fx); /* logLag=10*log10(pl), Q23 */
        Ltemp_q=L_shl(L_mult(shl(TMPDTFS_FX->Q,1),24660),9); /* Ltemp_q=2Q*10log10(2), Q23 */

        /* Process low band */
        Ltemp=DTFS_setEngyHarm_fx(236,2828,0,2828, 1, 0, &Ql,TMPDTFS_FX);/* Q of Ltemp = 2*(TMPDTFS_FX->Q) = Ql ?  */
        /* Compensate for Q factor of energy to get log10(lag*eng)  */
        Ltemp=log10_fx(Ltemp); /* Ltemp=10log10(eng), Q23 */
        Ltemp=L_add(L_sub(Ltemp,Ltemp_q),logLag);  /* Ltemp=10*log10(lag*eng), Q23 */
        /*st_fx->lastLgainE_fx=round_fx(L_shl((Word32)Mpy_32_16(extract_h(Ltemp),extract_l(Ltemp),0x6666),1)); // Q11, 0x6666 = 0.1 in Q18 */
        st_fx->lastLgainE_fx=round_fx(L_shl(Mult_32_16(Ltemp,0x6666),1)); /* Q11, 0x6666 = 0.1 in Q18 */


        /* Process high band */
        Ltemp=DTFS_setEngyHarm_fx(2828,upper_cut_off_freq_of_interest_norm_fx,2828,upper_cut_off_freq_norm_fx, 1, 0, &Qh,TMPDTFS_FX);
        Ltemp=log10_fx(Ltemp);
        Ltemp=L_add(L_sub(Ltemp,Ltemp_q),logLag); /* Ltemp=10*log10(lag*eng), Q23 */
        st_fx->lastHgainE_fx=round_fx(L_shl(Mult_32_16(Ltemp,0x6666),1)); /* Q11 */

        /* Need to unify the Q factors of both bands */
        TMPDTFS_FX->Q=s_min(Ql,Qh);
        move16();/* set Q factor to be the smaller one of Ql and Qh */
        n=sub(Ql,Qh); /* compare band Q factors */

        IF (n<0)
        {
            rshiftHarmBand_fx(TMPDTFS_FX,2828, upper_cut_off_freq_norm_fx,n);
        }
        ELSE IF (n>0)
        {
            rshiftHarmBand_fx(TMPDTFS_FX,0, 2828, sub(Qh,Ql));
        }

        DTFS_to_erb_fx(*TMPDTFS_FX,st_fx->lasterbE_fx); /* output lasterbE_fx in Q13  */
        Lacc1 = L_max(prev_Engy, 1);
    }
    ELSE
    {
        /* Copy DTFS related parameters from 'st_fx' to 'dtfs_temp' structure */
        dtfs_temp_fx->lag_fx = st_fx->dtfs_enc_lag_fx;
        move16();
        dtfs_temp_fx->nH_fx = st_fx->dtfs_enc_nH_fx;
        move16();
        dtfs_temp_fx->nH_4kHz_fx = st_fx->dtfs_enc_nH_4kHz_fx;
        move16();
        dtfs_temp_fx->upper_cut_off_freq_of_interest_fx = st_fx->dtfs_enc_upper_cut_off_freq_of_interest_fx;
        move16();
        dtfs_temp_fx->upper_cut_off_freq_fx = st_fx->dtfs_enc_upper_cut_off_freq_fx;
        move16();

        Copy(st_fx->dtfs_enc_a_fx, dtfs_temp_fx->a_fx, MAXLAG_WI);
        Copy(st_fx->dtfs_enc_b_fx, dtfs_temp_fx->b_fx, MAXLAG_WI);

        dtfs_temp_fx->Q = st_fx->dtfs_enc_Q;
        move16();
        Lacc1=DTFS_getEngy_P2A_fx(dtfs_temp_fx);
        prev_Engy = L_add(Lacc1, 0);
    }

    /*-----Open-loop Bump-Up-------- */

    /* Energy ratio calculation in_fx residual and speech domain */
    /* Also, compute correlation between the previous and the */
    /* current prototype */

    /* res_enratio = DTFS_getEngy(*CURRP_NQ) / DTFS_getEngy(*dtfs_temp); */
    Lacc = L_add(curr_Engy, 0);
    /* Lacc1 has been handled above */

    sft=add(shl(sub(CURRP_NQ_FX->Q,dtfs_temp_fx->Q),1),4);
    IF (sft>0)
    {
        if (L_sub(Lacc1,L_shr(Lacc,sft))<0)
        {
            res_enratio_fx=0x7FFF;
            move16();
        }
    }
    ELSE
    {
        if (L_sub(L_shr(Lacc1,negate(sft)),Lacc)<0)
        {
            res_enratio_fx=0x7FFF;
            move16();
        }
    }
    /* max value res_enratio compared against is 0x7400 (14.5 in Q11) */

    IF (L_sub(res_enratio_fx,0x7FFF)!=0)
    {

        expb = norm_l(Lacc);
        fracb = extract_h(L_shl(Lacc,expb));
        expb = sub(30,add(expb, shl(CURRP_NQ_FX->Q, 1)));


        expa = norm_l(Lacc1);
        fraca = extract_h(L_shl(Lacc1,expa));
        expa = sub(30, add(expa, shl(dtfs_temp_fx->Q, 1)));

        scale = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale);
        expb = sub(expb,scale);

        tmp = div_s(fracb,fraca);
        exp = sub(expb,expa);
        res_enratio_fx = shl(tmp,sub(exp,4));
    }
    /* res_enratio_fx is Q11 */

    /* Copy over CURRP_NQ into TMPDTFS */
    DTFS_copy_fx(TMPDTFS_FX, *CURRP_NQ_FX); /* output = TMPDTFS_FX with Q = CURRP_NQ_FX->Q */

    /* Copy over dtfs_temp into TMPDTFS2 */
    DTFS_copy_fx(TMPDTFS2_FX, *dtfs_temp_fx); /* output = TMPDTFS2_FX with Q = dtfs_temp_fx->Q */

    tmptmp = DTFS_alignment_full_fx(*TMPDTFS2_FX,*TMPDTFS_FX,st_fx->ph_offset_E_fx,S_fx,C_fx
                                    , 0
                                   );

    tmptmp1 = sub(shl(TMPDTFS_FX->lag_fx,1), tmptmp);/* (C_l-tmptmp) , Q1     */

    Q2phaseShift_fx(TMPDTFS_FX, negate(shl(tmptmp1,1)), TMPDTFS_FX->lag_fx,S_fx,C_fx); /* fixed bug , phase shift by tmp computed in_fx TMP.lag domain (above) */

    /*tmpres = (float)(DTFS_freq_corr(*TMPDTFS, *TMPDTFS2, 100.0f, 3700.0f));*/
    tmpres_fx = DTFS_freq_corr_fx(*TMPDTFS_FX, *TMPDTFS2_FX, 100, 3700, &Qtmpres);     /* tmpres_fx has Q factor tmpres */



    poleFilter_setup_fx(lpc2_fx, M+1, *TMPDTFS_FX, S_fx, C_fx, pf_temp1, pf_temp2, pf_temp, pf_n2);
    DTFS_poleFilter_fx_9(TMPDTFS_FX, pf_temp1, pf_temp2, pf_temp, pf_n2);
    /* lpc2_fx in Q12 */

    DTFS_adjustLag_fx(TMPDTFS2_FX, TMPDTFS_FX->lag_fx); /* operate in_fx CL domain */

    DTFS_poleFilter_fx(TMPDTFS2_FX,lpc1_fx,M+1, S_fx, C_fx); /* lpc1_fx in Q12 */

    tmp_fx = DTFS_freq_corr_fx(*TMPDTFS_FX, *TMPDTFS2_FX, 100, 3700, &Qtmp);
    /* tmp_fx Q = Qtmp */


    /*******************************************************************************
    if ( DTFS_getEngy(*TMPDTFS2) > 0 )
    {
        sp_enratio = DTFS_getEngy(*TMPDTFS)/DTFS_getEngy(*TMPDTFS2);
    }
    else
    {
        sp_enratio = 0.0f;
    }
    *******************************************************************************/
    Ltmp_32 = (DTFS_getEngy_fx(TMPDTFS2_FX)); /* Output Q=2*(TMPDTFS2_FX->Q) */
    IF ( Ltmp_32 > 0 )
    {
        /*sp_enratio = DTFS_getEngy(*TMPDTFS)/DTFS_getEngy(*TMPDTFS2); in Q15 */
        Lacc=DTFS_getEngy_P2A_fx(TMPDTFS_FX);
        Lacc1=Ltmp_32;
        /* IF (L_sub(sp_enratio_fx,0x7FFF)!=0) */
        {
            expb = norm_l(Lacc);
            fracb = extract_h(L_shl(Lacc,expb));

            expb = sub(30, add(expb, shl(TMPDTFS_FX->Q, 1)));

            expa = norm_l(Lacc1);
            fraca = extract_h(L_shl(Lacc1,expa));

            expa = sub(30, add(expa, shl(TMPDTFS2_FX->Q, 1)));

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            tmp = div_s(fracb,fraca);
            exp = sub(expb,expa);
            sp_enratio_fx = L_shl(tmp, exp); /* Q15 */
        }
    }
    ELSE
    {
        sp_enratio_fx = L_deposit_l(0);/* Q15 */
    }

    /*******************************************************************************/
    IF (sub(PPP_MODE_E,'Q') == 0)
    {
        /* Bump up if the lag is out_fx of range */
        test();
        IF (sub(sub(l,pl),13)>0 || sub(sub(l,pl),-11) < 0)
        {
            PPP_MODE_E = 'B' ;
            move16();
        }
        ELSE
        {
            delta_lag_E= sub(l,pl);
        }

        /* Bump up if big change between the previous and the current CWs */
        IF ( sub(shl(vadsnr_fx,1) ,st_fx->SNR_THLD_fx) < 0 ) /*Q8  */
        {
            /*if ( res_enratio > 5.0 && tmp < 0.65 ) */
            /* 5 in Q11, 0.65 in Q15 // L_shl(tmp_fx,sub(31,Qtmp)) makes tmp_fx FIXED Q31 */
            test();
            IF ((L_sub(res_enratio_fx,10240)>0) && (sub(extract_h(L_shl(tmp_fx,sub(31,Qtmp))),21299)<0))
            {
                PPP_MODE_E = 'B';
                move16();
            }
        }
        ELSE
        {
            /* if ( res_enratio > 3.0 && tmp < 1.2 )  */
            /*3 in Q11, 1.2 in Q14  // L_shl(tmp_fx,sub(31,Qtmp)) makes tmp_fx FIXED Q14 */
            test();
            IF ( (L_sub(res_enratio_fx,6144)>0) && (sub(extract_h(L_shl(tmp_fx,sub(30,Qtmp))),19661)<0) )
            {
                PPP_MODE_E = 'B';
                move16();
            }
        }
    }

    /* Rapid rampdown frame where time resolution is important */
    /* Not a suitable PPP frame -> Bump to CELP */

    IF ( sub(shl(vadsnr_fx,1) ,st_fx->SNR_THLD_fx) < 0 ) /*Q8  */
    {
        /* if (res_enratio < 0.025) */
        IF (L_sub(L_shl(res_enratio_fx,4),819)<0)  /*0x0333 = 0.025 in Q15, res_enratio_fx in Q15 after shl 4 */
        {
            st_fx->bump_up_fx = 1;
            move16();
            free(CURRP_NQ_FX);
            free(TMPDTFS_FX);
            free(TMPDTFS2_FX);
            free(TMPDTFS3_FX);
            free(CURRP_Q_E_FX);
            free(dtfs_temp_fx);
            return;
        }
    }
    ELSE
    {
        /* if ( res_enratio < 0.092f) */
        if ( L_sub(L_shl(res_enratio_fx,4), 3015)< 0 )/*3015 = 0.092 in Q15, res_enratio_fx in Q15 after shl 4 */
        {
            st_fx->bump_up_fx = 1;
            move16();
        }
    }

    /*  if (min(res_enratio, sp_enratio) < 0.075 && tmp < -0.5f)) : 2458 = 0.075 in Q15 */
    test();
    if ( L_sub(L_min(L_shl(res_enratio_fx,4), sp_enratio_fx), 2458) < 0 && L_sub(tmp_fx , shl(-1,sub(Qtmp,1)))<0 )
    {
        st_fx->bump_up_fx = 1;
        move16();
    }

    /* Rapid rampup frame where time resolution is important */
    /* Not a suitable PPP frame -> Bump to CELP */
    IF ( sub(shl(vadsnr_fx,1) ,st_fx->SNR_THLD_fx) < 0 ) /*Q8  */
    {
        IF (L_sub(res_enratio_fx,29696)>0)  /*14.5 in Q11 */
        {
            st_fx->bump_up_fx = 1;
            move16();
            free(CURRP_NQ_FX);
            free(TMPDTFS_FX);
            free(TMPDTFS2_FX);
            free(TMPDTFS3_FX);
            free(CURRP_Q_E_FX);
            free(dtfs_temp_fx);
            return;
        }
    }
    ELSE
    {
        if (L_sub(res_enratio_fx,14336)>0)  /* 7.0 in Q11 */
        {
            st_fx->bump_up_fx = 1;
            move16();
        }
    }

    IF ( st_fx->bump_up_fx == 1 )
    {
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }

    /* Bump up when the previous frame is an unvoiced or a silent frame */
    IF (sub(st_fx->last_coder_type_raw_fx,UNVOICED) == 0)
    {
        st_fx->bump_up_fx = 1;
        move16();
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }
    /* -----End Open-loop Bump-Up */

    /* PPP-WI Quantization */
    IF (sub(PPP_MODE_E,'Q') == 0)
    {
        flag = 1;
        move16();
        IF (sub(PPP_MODE_E,'Q') == 0)
        {
            flag = ppp_quarter_encoder_fx(CURRP_Q_E_FX, TMPDTFS_FX, dtfs_temp_fx->lag_fx, *CURRP_NQ_FX,
                                          lpc2_fx, &st_fx->lastLgainE_fx, &st_fx->lastHgainE_fx, st_fx->lasterbE_fx, *dtfs_temp_fx, S_fx, C_fx, st_fx);
            move16();
        }

        IF (flag)
        {
            /* TMPDTFS : Target prototype: Amp Quantized + Phase Unquantized         */
            /* TMPDTFS2: Quantized prototype: Amp Quantized + Phase Quantized        */
            /* TMPDTFS3: Delta prototype: Diff betw. target and quant. in_fx speech dom */

            /* ----- Closed-loop Bump-Up ---------- */
            Word32 pos_nq_fx, neg_nq_fx,pos_q_fx,neg_q_fx;
            Word16 Qposnq, Qnegnq, Qposq,Qnegq;

            DTFS_peaktoaverage_fx(*TMPDTFS_FX,&pos_nq_fx,&Qposnq,&neg_nq_fx,&Qnegnq);
            DTFS_peaktoaverage_fx(*CURRP_Q_E_FX, &pos_q_fx, &Qposq,&neg_q_fx,&Qnegq);


            /* Before we perform the peak-to-average ratio comparison, we have to */
            /* ensure that the energy is not decaying and also the pitch_fx pulse */
            /* is clearly defined */

            /*  Usually triggers in the slow ramp down frames.  Does not fall under the test condition (res_enratio < 0.025) as
                both frames have little energy and the ratio is not very small. Not  suitable for PPP */

            IF ( sub(CURRP_Q_E_FX->upper_cut_off_freq_fx , 4000) >0 )
            {
                Ltemp2 = DTFS_getEngy_band_wb_fx(*CURRP_Q_E_FX, 0, 2000);
                /* Use this bump-up only for WB signals */
                IF ( Ltemp2 > 0 )
                {
                    /* sp_hb_enratio = DTFS_getEngy_band_wb(*CURRP_Q_E, 2000.0, 6400.0)/DTFS_getEngy_band_wb(*CURRP_Q_E, 0.0, 2000.0); */
                    Ltemp1 = DTFS_getEngy_band_wb_fx(*CURRP_Q_E_FX, 2000, 6400);/*Q13 */
                    /*sp_hb_enratio_fx = divide_dp(Ltemp1,Ltemp2,0, 1);//Q29 */
                    Qadj = 0;
                    move16();
                    /*----------------------------------------------------------*/
                    /* Ltemp_fx = (Word32)divide_dp(Ltemp1, Ltemp2, Qadj,1); Q29*/
                    /*----------------------------------------------------------*/
                    if(Ltemp1<0)
                    {
                        Ltemp1 = L_negate(Ltemp1);
                    }
                    expa = norm_l(Ltemp2);
                    fraca = extract_h(L_shl(Ltemp2,expa));
                    expa = sub(30,expa);

                    expb = norm_l(Ltemp1);
                    fracb = round_fx(L_shl(Ltemp1,expb));
                    expb = sub(30, add(expb, Qadj));

                    scale = shr(sub(fraca,fracb),15);
                    fracb = shl(fracb,scale);
                    expb = sub(expb,scale);

                    tmp = div_s(fracb,fraca);
                    exp = sub(expb,expa);
                    Ltemp_fx = L_shl(tmp, add(exp,14));
                    /*-------------------------------------------*/
                    sp_hb_enratio_fx = L_add(Ltemp_fx, 0);  /* Q29 */
                }
                ELSE
                {
                    sp_hb_enratio_fx = L_deposit_l(0);
                }
                low_band_en_fx = Ltemp2; /*Q13 */

                /* if ( low_band_en < 25.0f && sp_hb_enratio < 1.6f ) */
                /* 25.0 in Q13 = 204800, 1.6 in Q29 = 858993459 */
                test();
                IF ( L_sub(low_band_en_fx , 204800 )< 0  && L_sub(sp_hb_enratio_fx , 858993459 ) < 0 )
                {
                    PPP_MODE_E = 'B';
                    move16();
                }
            }

            Ltmp_32=DTFS_getEngy_fx(CURRP_NQ_FX); /*Q = 2*(CURRP_NQ_FX->Q) */
            Qadj = sub(st_fx->Q_prev_cw_en_fx, shl(CURRP_NQ_FX->Q, 1));

            Ltmp_32 = L_shl(Ltmp_32,Qadj); /* shift left required to adjust Q of CURRP_NQ_FX = Q_prev_cw_en_fx */

            /* Ltmp1_32 = 0.8f * st->prev_cw_en */
            Ltmp1_32 = Mult_32_16(st_fx->prev_cw_en_fx,26214); /* Q = (Q_prev_cw_en_fx + Q15+1)-Q16 = Q_prev_cw_en_fx */

            IF ( sub(shl(vadsnr_fx,1) ,st_fx->SNR_THLD_fx) < 0 ) /*Q8  */
            {
                /* if ( DTFS_getEngy(*CURRP_NQ) > 0.8f * st->prev_cw_en && max(pos_nq, neg_nq) > 3.0f && st->rate_control ) */
                /* pos_nq_fx and neg_nq_fx in Q28 ???? */
                test();
                test();
                IF ( L_sub( Ltmp_32, Ltmp1_32) > 0 && L_sub(L_max(pos_nq_fx, neg_nq_fx) , 805306368) > 0 && st_fx->rate_control_fx )
                {
                    /*if ( pos_nq > neg_nq && pos_nq > 2.0f * pos_q )  */
                    test();
                    IF ( L_sub(pos_nq_fx , neg_nq_fx) > 0 && L_sub(Mult_32_16(pos_nq_fx,16384), pos_q_fx) > 0 )
                    {
                        PPP_MODE_E = 'B';
                        move16();
                    }

                    test();
                    /*if ( pos_nq < neg_nq && neg_nq > 2.0f * neg_q ) */
                    IF ( L_sub(pos_nq_fx , neg_nq_fx) < 0 && L_sub(Mult_32_16(neg_nq_fx,16384), neg_q_fx) > 0 )
                    {
                        PPP_MODE_E = 'B';
                        move16();
                    }
                }
            }

            ELSE
            {
                /* if ((((DTFS_getEngy(*CURRP_NQ) >(st->prev_cw_en))&&(max(pos_nq,neg_nq)>3.5))&&(st->rate_control))||  */
                /*    (((DTFS_getEngy(*CURRP_NQ) >0.8*(st->prev_cw_en))&&(max(pos_nq,neg_nq)>3.0))&&(!st->rate_control))) */
                test();
                test();
                test();
                test();
                test();
                IF ((((L_sub(Ltmp_32 ,(st_fx->prev_cw_en_fx)>0))&&(L_sub(L_max(pos_nq_fx,neg_nq_fx),939524096)>0))&&(st_fx->rate_control_fx))||
                (((L_sub( Ltmp_32, Ltmp1_32) > 0 ) && (L_sub(L_max(pos_nq_fx,neg_nq_fx),805306368)>0))&&(!st_fx->rate_control_fx)))
                {
                    /* if (((pos_nq > neg_nq) && (pos_nq > 2.5*pos_q)&&(st->rate_control))||
                           ((pos_nq > neg_nq) && (pos_nq > 2.0*pos_q)&&(!st->rate_control))) */
                    test();
                    test();
                    test();
                    test();
                    test();
                    IF ((L_sub(pos_nq_fx , neg_nq_fx)>0 && L_sub(Mult_32_16(pos_nq_fx ,13107),pos_q_fx)>0 && (st_fx->rate_control_fx))||
                    (L_sub(pos_nq_fx , neg_nq_fx)>0 && L_sub(Mult_32_16(pos_nq_fx,16384),pos_q_fx) >0 && (!st_fx->rate_control_fx)))
                    {
                        PPP_MODE_E='B';
                        move16();
                    }

                    /* if ((((pos_nq < neg_nq) && (neg_nq > 2.5*neg_q))&&(st->rate_control))||
                            ((pos_nq < neg_nq) && (neg_nq > 2.0*neg_q)&&(!st->rate_control))) */
                    test();
                    test();
                    test();
                    test();
                    test();
                    IF ((L_sub(pos_nq_fx , neg_nq_fx)<0 && L_sub(Mult_32_16(neg_nq_fx ,13107),neg_q_fx)>0 && (st_fx->rate_control_fx))||
                        (L_sub(pos_nq_fx , neg_nq_fx)<0 && L_sub(Mult_32_16(neg_nq_fx,16384),neg_q_fx) >0 && (!st_fx->rate_control_fx)))
                    {
                        PPP_MODE_E='B';
                        move16();
                    }
                }


                IF ( st_fx->rate_control_fx )
                {

                    DTFS_peaktoaverage_fx(*CURRP_NQ_FX,&pos_nq0_fx,&Qposnq,&neg_nq0_fx,&Qnegnq);

                    impzi_fx[0]=1;
                    move16();
                    FOR (x_fx=1; x_fx<160; x_fx++)
                    {
                        impzi_fx[x_fx]=0;
                        move16();
                    }

                    FOR (x_fx=0; x_fx<160; x_fx++)
                    {
                        impzo_fx[x_fx]=0;
                        move16();
                    }

                    FOR (x_fx=0; x_fx<10; x_fx++)
                    {
                        mem_fx[x_fx]=0;
                        move16();
                    }

                    /* lpc2_fx in Q12, so Qadj is set to 3 toi bring it to Q15 */
                    Qadj = 15-12;
                    move16();
                    synthesis_filter_fx(lpc2_fx,&impzi_fx[0],&impzo_fx[0],&mem_fx[0],10,160 );

                    /* compute energy of impz */
                    FOR (x_fx=0; x_fx<160; x_fx++)
                    {
                        energy_impz_fx = L_add(energy_impz_fx , L_mult0(impzo_fx[x_fx],impzo_fx[x_fx]));
                    }

                    /*energy_impz = (float)(10*log10((float)energy_impz)); */
                    exp_ee = norm_l(energy_impz_fx);
                    frac_ee = Log2_norm_lc(L_shl(energy_impz_fx, exp_ee));
                    exp_ee = sub(30, exp_ee);    /*30-exp-Q0 */
                    Ltmp = Mpy_32_16(exp_ee, frac_ee, LG10);    /* Ltmp Q14  */
                    energy_impz_fx =L_shr(Ltmp, 3);          /* 16+11(4bits for 15 no) = 14+x => x= 11 */
                    /* energy_impz_fx is Q11 */

                    Ltmp_32 = DTFS_getEngy_fx(CURRP_Q_E_FX); /*Q = 2*(CURRP_Q_E_FX->Q) */
                    Qadj = sub(st_fx->Q_prev_cw_en_fx, shl(CURRP_Q_E_FX->Q, 1));
                    Ltmp_32 = L_shl(Ltmp_32,Qadj); /* shift left required to adjust Q of CURRP_Q_E_FX = Q_prev_cw_en_fx */

                    /* if ((DTFS_getEngy(*CURRP_Q_E) > st->prev_cw_en)&&(max(pos_q,neg_q)>3.5) && energy_impz>15.0 && tmpres>0.7) */
                    test();
                    test();
                    test();
                    IF ((L_sub(Ltmp_32, st_fx->prev_cw_en_fx )>0) && (L_sub(L_max(pos_q_fx,neg_q_fx),939524096)>0) && (L_sub(energy_impz_fx,30720)>0)
                        && (L_sub(Mult_32_16(tmpres_fx,23265),shl(1,sub(Qtmpres,1)))>0)   )
                    {
                        /* if ((pos_q > neg_q) && ((pos_q>3.0*pos_nq0) || ((pos_q > 1.5*pos_nq0) && (neg_q < 1.5*neg_nq0)))) */
                        test();
                        test();
                        test();
                        IF ( (L_sub(pos_q_fx , neg_q_fx)>0)
                             && ((L_sub(Mult_32_16(pos_q_fx,10923), L_shr(pos_nq0_fx,sub(Qposnq,28)) )>0)
                                 || ((L_sub( Mult_32_16(pos_q_fx,21845),L_shr(pos_nq0_fx,sub(Qposnq,28)) )>0)
                                     && (L_sub(Mult_32_16(neg_q_fx,21846), L_shr(neg_nq0_fx,sub(Qnegnq,28)) )<0) ))
                           )
                        /* 10923 = (1/3) oin Q15, pos_q_fx is Q28, so result of Mult_32_16(pos_q_fx,10923) = Q28 */
                        /* L_shr(pos_nq0_fx,sub(Qposnq,28)) brings pos_nq0_fx with variable Q to fixed Q28 */
                        {
                            PPP_MODE_E='B';
                            move16();
                        }
                        test();
                        test();
                        test();
                        /* if ((pos_q <= neg_q) && ((neg_q>3.0*neg_nq0)|| ((neg_q > 1.5*neg_nq0) && (pos_q < 1.5*pos_nq0)))) */
                        IF ( (L_sub(pos_q_fx ,neg_q_fx)<=0)
                             && ((L_sub(Mult_32_16(neg_q_fx,10923),  L_shr(neg_nq0_fx,sub(Qnegnq,28)))>0)
                                 || ((L_sub(Mult_32_16(neg_q_fx,21846),  L_shr(neg_nq0_fx,sub(Qnegnq,28)))>0)
                                     && (L_sub( Mult_32_16( pos_q_fx,21846),L_shr(pos_nq0_fx,sub(Qposnq,28)))<0)))
                           )
                        {
                            PPP_MODE_E='B';
                            move16();
                        }
                    }
                }
            }

            DTFS_copy_fx(TMPDTFS2_FX,*CURRP_Q_E_FX);
            DTFS_poleFilter_fx_9(TMPDTFS_FX, pf_temp1, pf_temp2, pf_temp, pf_n2);
            DTFS_poleFilter_fx_9(TMPDTFS2_FX, pf_temp1, pf_temp2, pf_temp, pf_n2);

            *TMPDTFS3_FX = DTFS_sub_fx(*TMPDTFS_FX,*TMPDTFS2_FX);


            /* operate in ADR mode only the rate control is active. This adds some bumpups to improve the speech quality */
            /* if ((DTFS_getEngy_band(*TMPDTFS, 1500.0, upper_cut_off_freq_of_interest)/DTFS_getEngy(*TMPDTFS) > 0.05)&&(!st->rate_control)) */
            Ltemp1 = DTFS_getEngy_band_fx(*TMPDTFS_FX, 1500, upper_cut_off_freq_of_interest_fx); /* Q = 2*TMPDTFS_FX->Q*/
            Ltemp2 = DTFS_getEngy_fx(TMPDTFS_FX); /* Q = 2*TMPDTFS_FX->Q */

            expb = norm_l(Ltemp1);
            fracb = extract_h(L_shl(Ltemp1,expb));
            expb = sub(30, add(expb,shl(TMPDTFS_FX->Q, 1)));
            expa = norm_l(Ltemp2);
            fraca = extract_h(L_shl(Ltemp2,expa));
            expa = sub(30, add(expa,shl(TMPDTFS_FX->Q, 1)));

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            tmp = div_s(fracb,fraca); /* tmp in Q15 */
            exp = sub(expb,expa);     /* ans = tmp*2^(exp) */
            Ltemp_fx = L_shl(tmp, add(exp,12)); /* make tmp Q27 */

            test();
            IF (L_sub(Ltemp_fx , 6710886)>0 && (!st_fx->rate_control_fx)) /* 0.05 in Q27 = 6710886 */
            {
                /*if (10.0*log10(DTFS_getEngy_band(*TMPDTFS,1500.0,upper_cut_off_freq_of_interest)/ */
                /*DTFS_getEngy_band(*TMPDTFS3,1500.0,upper_cut_off_freq_of_interest)) < 0.1) */

                Ltemp1 = DTFS_getEngy_band_fx(*TMPDTFS_FX,1500,upper_cut_off_freq_of_interest_fx);
                Ltemp2 = DTFS_getEngy_band_fx(*TMPDTFS3_FX,1500,upper_cut_off_freq_of_interest_fx);

                /*--------------------------------------------------------------*/
                /* Ltemp_fx = (Word32)divide_dp(Ltemp1, Ltemp2, Qadj,1);//Q29+1 */
                /*--------------------------------------------------------------*/
                expa = norm_l(Ltemp2);
                fraca = extract_h(L_shl(Ltemp2,expa));
                expa = sub(30, expa);

                expb = norm_l(Ltemp1);
                fracb = round_fx(L_shl(Ltemp1,expb));
                expb = sub(30,expb);

                scale = shr(sub(fraca,fracb),15);
                fracb = shl(fracb,scale);
                expb = sub(expb,scale);

                tmp = div_s(fracb,fraca);
                exp = sub(expb,expa);
                Ltemp_fx = L_shl(tmp,add(exp,14)); /* answer in Q29 */
                /*-------------------------------------------*/

                /* 10.0*log10(Ltemp_fx) */
                exp_ee = norm_l(Ltemp_fx);
                frac_ee = Log2_norm_lc(L_shl(Ltemp_fx, exp_ee));
                exp_ee = sub(30,add(exp_ee,29));  /* 30 fixed here, 29 is the Q of Ltemp_fx */
                Ltmp = Mpy_32_16(exp_ee, frac_ee, LG10);    /* LG10 in Q13, so answer Ltmp in Q14 */

                IF(L_sub(Ltmp,1638)<0) /* 1638 = 0.1 in Q14 */
                {
                    /* if (res_enratio > 0.8) */
                    if (L_sub(res_enratio_fx , 1638)>0) /* 1638 = 0.8 in Q11, res_enratio_fx in Q11 */
                    {
                        PPP_MODE_E = 'B';
                        move16();
                    }
                }
            }

            /* To increase bump up, raise first threshold, lower second  */
            /*tmp = (float)(10.0*log10(DTFS_getEngy(*TMPDTFS)/DTFS_getEngy(*TMPDTFS3)));*/
            Lacc = DTFS_getEngy_P2A_fx(TMPDTFS_FX);  /* Q = 2*(TMPDTFS_FX->Q) */
            Lacc1= DTFS_getEngy_P2A_fx(TMPDTFS3_FX); /* Q = 2*(TMPDTFS3_FX->Q) */


            sft=shl(sub(TMPDTFS_FX->Q,TMPDTFS3_FX->Q),1);/* to check if Lacc<=2*Lacc1 */
            flag1=0;
            move16();
            IF (sft>0)
            {
                /*if (L_sub40_40(L_shr40(Lacc,sft),Lacc1)<=0) */
                Lacc = L_shr(Lacc,sft);
                flag1=1;
                move16(); /* do the divide */
            }
            ELSE
            {
                /*if (L_sub40_40(Lacc,L_shr40(Lacc1,negate(sft)))<=0) */
                Lacc1 = L_shr(Lacc1,negate(sft));
                flag1=1;
                move16();/* do the divide */
            }
            IF (sub(flag1,1)==0)
            {
                expb = norm_l(Lacc);
                fracb = extract_h(L_shl(Lacc,expb));
                expb = sub(30,add(expb,shl(TMPDTFS_FX->Q, 1)));

                expa = norm_l(Lacc1);
                fraca = extract_h(L_shl(Lacc1,expa));
                expa = sub(30,add(expa,shl(TMPDTFS3_FX->Q, 1)));

                scale = shr(sub(fraca,fracb),15);
                fracb = shl(fracb,scale);
                expb = sub(expb,scale);

                tmp = div_s(fracb,fraca); /* tmp is always Q15 */
                exp = sub(expb,expa);     /* Answer after division Lacc/Lacc1 = (2^exp)*(tmp/2^15) */

                L_tmp = L_deposit_h(tmp); /* tmp is always Q15, L_tmp is always Q31 */
                expa = norm_l(L_tmp);
                L_tmp = L_shl(L_tmp,expa);
                exp = sub(30,add(expa,sub(31,exp)));
                frac = Log2_norm_lc(L_tmp);
                L_tmp = Mpy_32_16(exp,frac,12330);  /* L_tmp is always Q13 */
                Ltemp = L_shl(L_tmp,10);            /* Ltemp is always Q23 */
            }
            ELSE
            {
                Ltemp = L_add(MAX_32, 0);
            }

            test();
            if ((Ltemp <= 0)&&(!st_fx->rate_control_fx))
            {
                PPP_MODE_E = 'B';
                move16();
            }

            IF ( sub(shl(vadsnr_fx,1) ,st_fx->SNR_THLD_fx) < 0 ) /* Q8 */
            {
                /* if ((( tmp < 3.05 && max(res_enratio,sp_enratio) > 0.8  ) && (st->rate_control))||
                       (( tmp < 2.8 && max(res_enratio,sp_enratio)  > 0.65 ) && (!st->rate_control))) */
                /* First comparison in Q23, Second comparison in Q15 */
                test();
                test();
                test();
                test();
                test();
                if ((( L_sub(Ltemp, 25585254 ) < 0 && L_sub(L_max(L_shl(res_enratio_fx,4),sp_enratio_fx) , 6554)> 0 )&&(st_fx->rate_control_fx))||
                        (( L_sub(Ltemp, 23488102 ) < 0 && L_sub(L_max(L_shl(res_enratio_fx,4),sp_enratio_fx) , 5325)> 0 )&&(!st_fx->rate_control_fx)))
                {
                    PPP_MODE_E = 'B';
                    move16();
                }
            }
            ELSE
            {
                /* if ((( tmp < 2.4 && max(res_enratio,sp_enratio) > 0.94) && (st->rate_control))||
                       (( tmp < 4.5 && max(res_enratio,sp_enratio) > 0.5 ) && (!st->rate_control))) */
                test();
                test();
                test();
                test();
                test();
                if ((( L_sub(Ltemp ,20132659 ) < 0 && L_sub(L_max(L_shl(res_enratio_fx,4),sp_enratio_fx) , 7700)> 0 )&&(st_fx->rate_control_fx))||
                (( L_sub(Ltemp, 37748736 ) < 0 && L_sub(L_max(L_shl(res_enratio_fx,4),sp_enratio_fx) , 4096)> 0 )&&(!st_fx->rate_control_fx)))
                {
                    PPP_MODE_E = 'B';
                    move16();
                }
            }

            /* -----End closed-loop Bump-Up */
        }
        ELSE
        {
            PPP_MODE_E = 'B' ;
            move16();/*Amplitude quantization is failing*/
        }
    }
    ELSE
    {
    }
    if (PPP_MODE_E == 'B')
    {
        st_fx->bump_up_fx = 1;
        move16();
        free(CURRP_NQ_FX);
        free(TMPDTFS_FX);
        free(TMPDTFS2_FX);
        free(TMPDTFS3_FX);
        free(CURRP_Q_E_FX);
        free(dtfs_temp_fx);
        return;
    }

    IF ( st_fx->Q_to_F_fx )
    {
        st_fx->patterncount_fx = add(st_fx->patterncount_fx ,st_fx->pattern_m_fx);

        IF (sub(st_fx->patterncount_fx , 1000)>=0)
        {
            st_fx->patterncount_fx = sub (st_fx->patterncount_fx , 1000);
            PPP_MODE_E = 'B';
            move16();
            st_fx->bump_up_fx = 1;
            move16();
            free(CURRP_NQ_FX);
            free(TMPDTFS_FX);
            free(TMPDTFS2_FX);
            free(TMPDTFS3_FX);
            free(CURRP_Q_E_FX);
            free(dtfs_temp_fx);
            return;
        }

    }

    /* packetization of the delta lag in_fx PPP */
    IF (sub(PPP_MODE_E,'Q') == 0)
    {
        Q_delta_lag = add(delta_lag_E,11); /* to make it positive always */

        push_indice_fx( st_fx, IND_DELTALAG, Q_delta_lag, 5 );
    }

    WIsyn_fx(*dtfs_temp_fx, CURRP_Q_E_FX, lpc2_fx, &(st_fx->ph_offset_E_fx), out_fx, L_FRAME,0,S_fx,C_fx,
             pf_temp1,pf_temp2,pf_temp,pf_n2);
    /* i/o ph_offset_fx in Q15, out_fx in Q0 */

    DTFS_copy_fx(dtfs_temp_fx, *CURRP_Q_E_FX);
    Lacc = DTFS_getEngy_P2A_fx(CURRP_NQ_FX);
    st_fx->Q_prev_cw_en_fx=norm_l(Lacc);

    /* st_fx->Q_prev_cw_en_fx = (Lacc==0)?31: st_fx->Q_prev_cw_en_fx;move16(); */
    if (Lacc==0)
    {
        st_fx->Q_prev_cw_en_fx = 31;
        move16();
    }

    st_fx->prev_cw_en_fx=(Word32) L_shl(Lacc,st_fx->Q_prev_cw_en_fx); /*2Q+Q_prev_cw_en_fx */
    st_fx->Q_prev_cw_en_fx=add(st_fx->Q_prev_cw_en_fx,shl(CURRP_NQ_FX->Q,1));
    /* Copy DTFS related parameters from 'dtfs_temp' to 'st_fx' structure */
    st_fx->dtfs_enc_lag_fx = dtfs_temp_fx->lag_fx;
    move16();
    st_fx->dtfs_enc_nH_fx = dtfs_temp_fx->nH_fx;
    move16();
    st_fx->dtfs_enc_nH_4kHz_fx = dtfs_temp_fx->nH_4kHz_fx;
    move16();
    st_fx->dtfs_enc_upper_cut_off_freq_of_interest_fx = dtfs_temp_fx->upper_cut_off_freq_of_interest_fx;
    move16();
    st_fx->dtfs_enc_upper_cut_off_freq_fx = dtfs_temp_fx->upper_cut_off_freq_fx;
    move16();
    Copy(dtfs_temp_fx->a_fx, st_fx->dtfs_enc_a_fx, MAXLAG_WI);
    Copy(dtfs_temp_fx->b_fx, st_fx->dtfs_enc_b_fx, MAXLAG_WI);

    st_fx->dtfs_enc_Q = dtfs_temp_fx->Q;
    move16();

    free(CURRP_NQ_FX);
    free(TMPDTFS_FX);
    free(TMPDTFS2_FX);
    free(TMPDTFS3_FX);
    free(CURRP_Q_E_FX);
    free(dtfs_temp_fx);

    return;
}


/*===================================================================*/
/* FUNCTION      :  synthesis_filter_fx ()                           */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  IIR-filter residual by the LPC sysnthesis filter */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*                                                                   */
/*   _ (Word16 []) b   : filter coefficients (Qc).                   */
/*   _ (Word16 []) x   : residual input  (Qn).                       */
/*   _ (Word16)    P   : filter order.                               */
/*   _ (Word16)    N   : number of input samples.                    */
/*   _ (Word16)    Qa  : Q factor compensation (Qa=15-Qc)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                                                                   */
/*   _ (Word16 []) y : output speech  (Qn)                           */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                                                                   */
/*   _ (Word32 []) buf : filter memory (Q16+Qn)                      */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/
/*      y(n)=sum(i=1 to 10){                                         */
/*              rxLpcCoeff(i)*rxFormSynthMem(n-i)                    */
/*              }                                                    */
/*           +fres(n)                                                */
/*===================================================================*/
void synthesis_filter_fx (Word16 b[], Word16 x[], Word16 y[], Word16 buf[],
                          Word16 P, Word16 N )
{
    Word32 acc;
    Word16 i, j;

    FOR (i = 0; i < N; i++)
    {
        acc = L_deposit_h(*x++);/*Q16 */
        acc = L_shr(acc, 3);/*Q13 */

        FOR (j = P - 1; j > 0; j--)
        {
            /* acc = L_sub(acc, L_mult(memory[j], coef[j])); */
            acc = L_msu(acc, buf[j], b[j]);/*Q13 */
            buf[j] = buf[j - 1];
            move16();
        }
        /* acc = L_sub(acc, L_mult(memory[0], coef[0])); */
        acc = L_msu(acc, buf[0], b[0]);
        acc = L_shl(acc, 3);

        *y++ = round_fx(acc);
        buf[0] = round_fx(acc);
    }
}





