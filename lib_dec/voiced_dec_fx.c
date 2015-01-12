/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "log2.h"
#include "stl.h"

/*===================================================================*/
/* FUNCTION      : void ppp_voiced_decoder_fx ()					 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :	   			                                     */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*  _ Word16 bfi_fx - Q0  bad frame indicator				         */
/*  _ const Word16 *lpc2_fx - Q12		current frame LPC            */
/*  _ Word16 *exc_fx		- Q0		previous frame excitation    */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ Decoder_State_fx *st_fx:										 */
/*                _ lastLgainD_fx - Q11                              */
/*                _ lastHgainD_fx - Q11                              */
/*                _ lasterbD_fx   - Q13                              */
/*		_ Word16 *pitch	- Q6 floating pitch values for each subframe */
/*      _ Word16 *out_fx - Q0				     residual signal	 */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ Decoder_State_fx *st_fx:										 */
/*				  _	lsp_old_fx	-               Q15					 */
/*				  _	st_fx->dtfs_dec_xxxx							 */
/*				  _	a nd b in   st_fx->dtfs_dec_Q					 */
/*				  	rest all in                 Q0					 */
/*                _  gainp_ppp                  Q14                  */
/* 	 _ Word16 *pitch_buf_fx	- Q6 fixed pitch values for each subframe */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*					 _ None											 */
/*-------------------------------------------------------------------*/
/* CALLED FROM : RX													 */
/*===================================================================*/
void ppp_voiced_decoder_fx(
    Decoder_State_fx *st_fx,			/* i/o: state structure */
    Word16 *out_fx,				/* o : residual signal */
    const Word16 *lpc2_fx,		/* i : current frame LPC */
    Word16 *exc_fx,				/* i : previous frame excitation */
    Word16 *pitch,				/* o : floating pitch values for each subframe */
    Word16 bfi					/* i : Frame error rate */
)
{
    Word16 k, delta_lag_D = 0,temp,Ql,Qh,diff;
    Word16 upper_cut_off_freq_of_interest = 0, upper_cut_off_freq = 0;
    Word16 pl, l,n,rem_fx;
    Word16 temp_l_fx, temp_pl_fx,interp_delay_fx[3];
    Word32 logLag,Ltemp_q,Ltemp,rem32,temp32_fx,tempnH_fx;
    Word32 en_temp1_fx,en_temp2_fx;
    Word16 exp,tmp;
    Word32 L_tmp;
    Word16 expa,expb,fraca,fracb,scale;

    DTFS_STRUCTURE_FX *TMPDTFS_FX = DTFS_new_fx();
    DTFS_STRUCTURE_FX *CURRP_Q_D_FX = DTFS_new_fx();

    DTFS_STRUCTURE_FX *dtfs_temp_fx = DTFS_new_fx();

    Word16 pf_temp1[MAXLAG_WI]; /*maynot need more than MAXLAG_WI/2+1 */
    Word16 pf_temp2[MAXLAG_WI];
    Word16 pf_temp[MAXLAG_WI];
    Word16 pf_n2[MAXLAG_WI];
    Word16 S_fx[4*PIT_MAX+1], C_fx[4*PIT_MAX+1];
    Word16 temp_Fs;

    test();
    IF (sub(st_fx->bwidth_fx,WB) == 0 || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        upper_cut_off_freq_of_interest = 0x2800;
        move16();/*4000 normalized to 12800 in Q15 */
        upper_cut_off_freq = 0x4000;
        move16();/*6400 normalized to 12800 in Q15 */
    }
    ELSE IF (sub(st_fx->bwidth_fx,NB) == 0 )
    {
        upper_cut_off_freq_of_interest = 0x2100;
        move16();/*3300 normalized to 12800 in Q15 */
        upper_cut_off_freq = 0x2800;
        move16();
    }

    temp_Fs = 8000;

    if ( sub( st_fx->bwidth_fx, WB) == 0 )
    {
        temp_Fs  = 16000;
        move16();
    }



    /* Initialization */
    IF (st_fx->firstTime_voiceddec_fx)
    {
        st_fx->firstTime_voiceddec_fx=0;
        move16();

        /* (st_fx->PREV_CW_D) = DTFS_new();*/
        st_fx->dtfs_dec_lag_fx = 0;
        move16();
        st_fx->dtfs_dec_nH_fx = 0;
        move16();
        st_fx->dtfs_dec_nH_4kHz_fx = 0;
        move16();
        st_fx->dtfs_dec_upper_cut_off_freq_of_interest_fx = 3300;
        move16();
        st_fx->dtfs_dec_upper_cut_off_freq_fx = 4000;
        move16();

        FOR(k = 0; k < MAXLAG_WI; k++)
        {
            st_fx->dtfs_dec_a_fx[k] = 0;
            move16();
            st_fx->dtfs_dec_b_fx[k] = 0;
            move16();
        }
        st_fx->dtfs_dec_Q = 0;
        move16();
    }
    pl = s_min(rint_new_fx(st_fx->old_pitch_buf_fx[(2*NB_SUBFR)-1]),PIT_MAX);
    delta_lag_D = (Word16) get_next_indice_fx( st_fx, 5 );
    l = s_min(2*MAXLAG_WI-1,add(pl,sub(delta_lag_D,11)));

    temp_pl_fx =  pl;
    move16();
    temp_l_fx  =  l;
    move16();

    IF (sub(temp_pl_fx,temp_l_fx) != 0)
    {
        FOR(k=0; k<NB_SUBFR; k++)
        {
            /* do the linear pitch interp to drive the nb_post_filt */
            Interpol_delay_fx(interp_delay_fx,temp_pl_fx,temp_l_fx,(Word16)k,frac_4sf_fx);
            pitch[k] = shl(interp_delay_fx[0],2);
            move16();/*Q6 */
        }
    }
    ELSE
    {
        set16_fx(pitch, shl(temp_l_fx,6), NB_SUBFR);/*Q6 */
    }


    if (sub(st_fx->last_coder_type_fx,UNVOICED) == 0)
    {
        pl = l;
        move16(); /* if prev frame was sil/uv*/
    }

    temp = mult(shl(l,3),30310);/*Q3+14+1-16 = Q2   30310 is 1.85 in Q14 */
    temp = (temp>=0)?add(temp,2):sub(temp,2);
    temp = shr(temp,2);/*Q0 */

    if (sub(pl,temp) > 0)
    {
        pl = shr(pl,1);
    }

    temp = mult(shl(l,3),8857);/*Q3+14+1-16 = Q2   8847 is 0.54 in Q14 */
    temp = (temp>=0)?add(temp,2):sub(temp,2);
    temp = shr(temp,2);/*Q0 */

    test();
    if (sub(shl(pl,1),PIT_MAX) <= 0 && sub(pl,temp) <= 0)
    {
        pl = shl(pl,1);
    }

    /* Restoring PPP memories when the last frame is non-PPP or full-rate PPP */
    IF (sub(st_fx->last_ppp_mode_dec_fx,1) != 0)
    {

        GetSinCosTab_fx(pl, S_fx, C_fx);
        DTFS_to_fs_fx(exc_fx-pl, pl, dtfs_temp_fx, temp_Fs, 0, S_fx, C_fx);

        st_fx->ph_offset_D_fx = 0 ;
        move16();

        /* Copy over PREV_CW_D into TMPDTFS */
        DTFS_copy_fx(TMPDTFS_FX, *dtfs_temp_fx);

        DTFS_car2pol_fx(TMPDTFS_FX);

        /*st_fx->lastLgainD = (float) log10(TMPDTFS->lag*DTFS_setEngyHarm(92.0,1104.5,0.0,1104.5,1.0,TMPDTFS)); */

        L_tmp = L_deposit_h(TMPDTFS_FX->lag_fx);/*Q16 */
        exp = norm_l(L_tmp);
        tmp = Log2_norm_lc(L_shl(L_tmp,exp));
        exp = (30-exp-16);
        L_tmp = Mpy_32_16(exp,tmp,12330);/* Q13 */ /* 10*log10(2) in Q12*/
        logLag = L_shl(L_tmp, 10);/*Q23 */

        Ltemp_q=L_shl(L_mult(shl(TMPDTFS_FX->Q,1),24660),9); /* Ltemp_q=2Q*10log10(2), Q23 */
        /* Process low band */
        Ltemp=DTFS_setEngyHarm_fx(236,2828,0,2828,1,0,&Ql,TMPDTFS_FX);/* Ltemp in 2*TMP.Q */
        /* Compensate for Q factor of energy to get log10(lag*eng)  */


        Ltemp=log10_fx(Ltemp); /* Ltemp=10log10(eng), Q23 */
        Ltemp=L_add(L_sub(Ltemp,Ltemp_q),logLag);  /* Ltemp=10*log10(lag*eng), Q23 */
        st_fx->lastLgainD_fx=round_fx(L_shl((Word32)Mpy_32_16(extract_h(Ltemp),extract_l(Ltemp),0x6666),1)); /* Q11 */



        /* Process high band */
        Ltemp=DTFS_setEngyHarm_fx(2828,upper_cut_off_freq_of_interest,2828,upper_cut_off_freq,1,0,&Qh,TMPDTFS_FX);


        Ltemp=log10_fx(Ltemp);
        Ltemp=L_add(L_sub(Ltemp,Ltemp_q),logLag); /* Ltemp=10*log10(lag*eng), Q23 */
        st_fx->lastHgainD_fx=round_fx(L_shl((Word32)Mpy_32_16(extract_h(Ltemp),extract_l(Ltemp),0x6666),1)); /* Q11 */


        /* Need to unify the Q factors of both bands */
        TMPDTFS_FX->Q=s_min(Ql,Qh);  /* set Q factor to be the smaller one */
        n=sub(Ql,Qh); /* compare band Q factors */

        /*This logic adjusts difference between Q formats of both bands */
        IF (n<0)
        {
            rshiftHarmBand_fx(TMPDTFS_FX,2828, upper_cut_off_freq,n);
        }
        ELSE IF (n>0)
        {
            rshiftHarmBand_fx(TMPDTFS_FX,0, 2828, sub(Qh,Ql));
        }

        DTFS_to_erb_fx(*TMPDTFS_FX,st_fx->lasterbD_fx);

    }
    ELSE
    {
        /* Copy DTFS related parameters from 'st' to 'dtfs_temp' structure */
        dtfs_temp_fx->lag_fx = st_fx->dtfs_dec_lag_fx;
        move16();
        dtfs_temp_fx->nH_fx = st_fx->dtfs_dec_nH_fx;
        move16();
        dtfs_temp_fx->nH_4kHz_fx = st_fx->dtfs_dec_nH_4kHz_fx;
        move16();
        dtfs_temp_fx->upper_cut_off_freq_of_interest_fx = st_fx->dtfs_dec_upper_cut_off_freq_of_interest_fx;
        move16();
        dtfs_temp_fx->upper_cut_off_freq_fx = st_fx->dtfs_dec_upper_cut_off_freq_fx;
        move16();

        Copy(st_fx->dtfs_dec_a_fx, dtfs_temp_fx->a_fx, MAXLAG_WI);
        Copy(st_fx->dtfs_dec_b_fx, dtfs_temp_fx->b_fx, MAXLAG_WI);
        dtfs_temp_fx->Q = st_fx->dtfs_dec_Q;
        move16();
    }

    CURRP_Q_D_FX->lag_fx = l;
    move16();

    /* compute nH for lag */
    Ltemp = L_shl((Word32)upper_cut_off_freq,13);/*Q28 */
    upper_cut_off_freq = (Word16)find_remd(Ltemp, 20971,&rem32);/*denormalize upper_cut_off_freq */



    /*	temp32_fx = (Word32)divide_dp((Word40)819200,(Word40)L_shl((Word32)CURRP_Q_D_FX->lag_fx,6),-23,1);//Q6 */
    exp = norm_s(CURRP_Q_D_FX->lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),CURRP_Q_D_FX->lag_fx);/*29-exp */
    L_tmp =L_shl(L_mult0(tmp,12800),exp - 7);
    temp32_fx = round_fx(L_tmp);
    diff = round_fx(L_shl(temp32_fx,16-6));/*Q0 */

    CURRP_Q_D_FX->nH_fx = find_rem(upper_cut_off_freq,diff,&rem_fx);/*Q0 */

    exp = norm_s(diff);
    tmp = div_s(shl(1,sub(14,exp)),diff);/*29-exp */
    L_tmp = L_shl(L_mult0(4000,tmp),exp - 7);
    tempnH_fx = extract_h(L_tmp);
    CURRP_Q_D_FX->nH_4kHz_fx = round_fx(L_shl(tempnH_fx,16-6));/*Q0 */


    IF(sub(sub(upper_cut_off_freq,shr((Word16)L_mult(diff,CURRP_Q_D_FX->nH_fx),1)),diff)>=0)
    {
        CURRP_Q_D_FX->nH_fx = add(CURRP_Q_D_FX->nH_fx,1);
    }
    tempnH_fx = L_mult0(extract_l(temp32_fx),CURRP_Q_D_FX->nH_4kHz_fx);/* */
    tempnH_fx = L_sub((Word32)256000,tempnH_fx);/*Q6 */

    if(L_sub(tempnH_fx,temp32_fx)>=0)
    {
        CURRP_Q_D_FX->nH_4kHz_fx = add(CURRP_Q_D_FX->nH_4kHz_fx,1);
    }

    CURRP_Q_D_FX->upper_cut_off_freq_fx = dtfs_temp_fx->upper_cut_off_freq_fx;
    move16();
    CURRP_Q_D_FX->upper_cut_off_freq_of_interest_fx = dtfs_temp_fx->upper_cut_off_freq_of_interest_fx;
    move16();
    GetSinCosTab_fx(CURRP_Q_D_FX->lag_fx,S_fx,C_fx);

    IF ( bfi == 0 )
    {
        ppp_quarter_decoder_fx(CURRP_Q_D_FX,dtfs_temp_fx->lag_fx,&(st_fx->lastLgainD_fx),
                               &(st_fx->lastHgainD_fx),st_fx->lasterbD_fx,bfi,S_fx,C_fx,*dtfs_temp_fx, st_fx);
    }

    WIsyn_fx(*dtfs_temp_fx, CURRP_Q_D_FX, lpc2_fx, &(st_fx->ph_offset_D_fx), out_fx, (Word16)L_FRAME, 0,
             S_fx, C_fx, pf_temp1, pf_temp2, pf_temp, pf_n2);
    IF ( bfi == 0 )
    {
        /* Update FER coefficients */
        en_temp1_fx = DTFS_getEngy_fx(CURRP_Q_D_FX);/*Q13 */
        en_temp2_fx = DTFS_getEngy_fx(dtfs_temp_fx);/*Q13 */
        IF(en_temp1_fx)
        {
            expa = norm_l(en_temp1_fx);
            fraca = extract_h(L_shl(en_temp1_fx,expa));
            expa = 30-expa - shl(CURRP_Q_D_FX->Q, 1);


            expb = norm_l(en_temp2_fx);
            fracb = round_fx(L_shl(en_temp2_fx,expb));
            expb = 30-expb - shl(dtfs_temp_fx->Q + st_fx->Q_exc, 1);


            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            tmp = div_s(fracb,fraca);
            exp = sub(expb,expa);

            L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /*Q(31-exp) */

            expa = norm_l(L_tmp);
            tmp = extract_h(L_shl(L_tmp,expa));
            exp = sub(sub(30,expa),31-exp);
            tmp = div_s(16384,tmp); /*Q(15+exp) */

            L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /*Q(31-exp) */

            st_fx->gainp_ppp_fx	= s_min(16384,round_fx(L_shl(L_tmp,exp-1))); /* Q14 */
        }
        ELSE
        {
            st_fx->gainp_ppp_fx = 0;
        }
    }
    ELSE
    {
        FOR(k=0; k<L_FRAME; k++)
        {
            out_fx[k] = mult(out_fx[k],st_fx->FadeScale_fx);
            move16();/*Q(0+15+1-16) = Q0 */
        }
    }

    DTFS_copy_fx(dtfs_temp_fx, *CURRP_Q_D_FX);

    /* Copy DTFS related parameters from 'dtfs_temp' to 'st' structure */
    st_fx->dtfs_dec_lag_fx = dtfs_temp_fx->lag_fx;
    move16();
    st_fx->dtfs_dec_nH_fx = dtfs_temp_fx->nH_fx;
    move16();
    st_fx->dtfs_dec_nH_4kHz_fx = dtfs_temp_fx->nH_4kHz_fx;
    move16();
    st_fx->dtfs_dec_upper_cut_off_freq_of_interest_fx = dtfs_temp_fx->upper_cut_off_freq_of_interest_fx;
    move16();
    st_fx->dtfs_dec_upper_cut_off_freq_fx = dtfs_temp_fx->upper_cut_off_freq_fx;
    move16();

    Copy(dtfs_temp_fx->a_fx, st_fx->dtfs_dec_a_fx,  MAXLAG_WI);
    Copy(dtfs_temp_fx->b_fx, st_fx->dtfs_dec_b_fx, MAXLAG_WI);

    st_fx->dtfs_dec_Q = dtfs_temp_fx->Q;
    move16();

    free(TMPDTFS_FX);
    free(CURRP_Q_D_FX);
    free(dtfs_temp_fx);

    return;
}
