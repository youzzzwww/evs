/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*=======================================================================================*/
/* FUNCTION      :  ppp_quarter_encoder_fx()                                             */
/*---------------------------------------------------------------------------------------*/
/* PURPOSE       :  Quarter rate PPP encoder main routine                                */
/*---------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                                    */
/*   _ (Word16 []) curr_lpc_fx: LPC coefficients, Q12                                    */
/*   _ (struct DTFS_STRUCTURE_FX) CURRCW_NQ_FX :  prototype in Cartesian domain          */
/*                (Word16) lag_fx: length of prototype									 */
/*                (Word16 []) a/b: harmonics, normalized								 */
/*                (Word16) Q: norm factor of a											 */
/*   _ (struct DTFS_STRUCTURE_FX) PREV_CW_E_FX :  past dtfs in Cartesian domain          */
/*                (Word16) lag: length of prototype										 */
/*                (Word16 []) a/b: harmonics, normalized								 */
/*                (Word16) Q: norm factor of a											 */
/*   _ (Word16)   prevCW_lag_fx: Previous lag, Q0										 */
/*   _ (Word16 *)		exc : Global input (Q0)											 */
/*   _ (Word16 []) sinTab, Q15 : sin(2pi/4L*n), n=0,1,...,4L-1                           */
/*   _ (Word16 []) cosTab, Q15 : cos(2pi/4L*n), n=0,1,...,4L-1                           */
/*---------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                					 */
/*   _ (Word16) pidx: Power index                                    					 */
/*   _ (Word16[]) aidx: Amplitude indices, 2 words                   					 */
/*   _ (struct DTFS_fx *) CURRCW_Q_FX :  quantized prototype in Cartesian domain		 */
/*                (Word16) lag_fx: length of prototype in time domain					 */
/*                (Word16 []) a/b: harmonics, normalized								 */
/*                (Word16) Q: norm factor of a											 */
/*   _ (struct DTFS_fx *) TARGETCW_FX : Target prototype in Cartesian domain			 */
/*                (Word16) lag_fx: length of prototype in time domain					 */
/*                (Word16 []) a/b: harmonics, normalized								 */
/*                (Word16) Q: norm factor of a                                           */
/*---------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																 */
/*   _ (Word16[]) lasterbE_fx: ERB history for differential                              */
/*                quantization, Q13                                                      */
/*   _ (Word16) lastLgainE_fx: low band power history, log domain,						 */
/*                                                            Q11                        */
/*   _ (Word16) lastHgainE_fx: high band power history, log domain,						 */
/*                                                            Q11                        */
/*---------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                                    */
/*   _ (Word16) returnFlag: flag indicating success/failure								 */
/*                   (TRUE/FALSE)                                                        */
/*---------------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                                      */
/*=======================================================================================*/

Word16 ppp_quarter_encoder_fx(
    DTFS_STRUCTURE_FX *CURRCW_Q_FX,                  /* o  : Quantized (amp/phase) DTFS				*/
    DTFS_STRUCTURE_FX *TARGETCW_FX,                  /* o  : DTFS with quant phase but unquant Amp	*/
    Word16   prevCW_lag,					/* i  : previous lag							*/
    DTFS_STRUCTURE_FX vCURRCW_NQ_FX,					/* i  : Unquantized DTFS						*/
    const Word16 *curr_lpc_fx,					/* i  : LPCS									*/
    Word16 *lastLgainE_fx,					/* i/o: last low band gain						*/
    Word16 *lastHgainE_fx,					/* i/o: last high band gain						*/
    Word16 *lasterbE_fx,					/* i/o: last ERB vector							*/
    DTFS_STRUCTURE_FX PREV_CW_E_FX,                  /* i  : past DTFS								*/
    Word16 *S_fx,                           /* i  : sin table, Q15                          */
    Word16 *C_fx,                           /* i  : cos table, Q15                          */
    Encoder_State_fx *st_fx
)
{
    DTFS_STRUCTURE_FX *PREVDTFS_FX = DTFS_new_fx();
    Word16 tmp_fx, temp_pl_fx, temp_l_fx;
    Word16 temp;
    Word16 l;
    Word16 returnFlag = 1;
    Word16 POWER_IDX_FX;    /* Codebook index for the power quantization for PPP */
    Word16 AMP_IDX_fx[2];   /* Codebook index for the Amplitude quantization for PPP */
    Word16 Erot_fx = 0;
    /* Word16 S_fx[PIT_MAX*4+1], C_fx[PIT_MAX*4+1];*/
    Word32 Ltempd,Ltempn;
    Word32 L_tmp, L_tmp1;
    Word16 tmp, exp;
    DTFS_copy_fx( CURRCW_Q_FX, vCURRCW_NQ_FX );
    DTFS_copy_fx( PREVDTFS_FX, PREV_CW_E_FX );

    l = CURRCW_Q_FX->lag_fx;
    move16();
    temp_l_fx =  CURRCW_Q_FX->lag_fx;
    move16();
    temp_pl_fx = prevCW_lag;
    move16();

    DTFS_adjustLag_fx( PREVDTFS_FX, l );


    /* z = ((L_FRAME-temp_l)*(temp_l+temp_pl))/(2*temp_l*temp_pl); */
    /* Erot = (float) (temp_l - rint_new(temp_l*(z - floor(z)))); */
    temp = sub(L_FRAME,temp_l_fx); /*Q0 */
    exp = norm_s(temp_pl_fx);
    tmp = div_s(shl(1,sub(14,exp)),temp_pl_fx); /*Q(29-exp) */
    L_tmp = L_mult(temp,tmp); /*Q(31-exp); +1 due to /2 */
    L_tmp = L_shl(L_tmp,sub(exp,15)); /*Q16 */

    exp = norm_s(temp_l_fx);
    tmp = div_s(shl(1,sub(14,exp)),temp_l_fx); /*Q(29-exp) */
    L_tmp1 = L_mult(temp,tmp); /*Q(31-exp); +1 due to /2 */
    L_tmp1 = L_shl(L_tmp1,sub(exp,15)); /*Q16 */

    L_tmp = L_add(L_tmp,L_tmp1); /*Q16 */

    tmp = lshr(extract_l(L_tmp),1); /*Q15 */
    L_tmp = L_mult(temp_l_fx,tmp); /*Q16 */
    temp = rint_new_fx(L_tmp);
    Erot_fx = sub(temp_l_fx,temp); /*Q0 */

    GetSinCosTab_fx(CURRCW_Q_FX->lag_fx,S_fx,C_fx);/*get cos and sin tables for lag */
    Q2phaseShift_fx(PREVDTFS_FX,shl(Erot_fx,2),CURRCW_Q_FX->lag_fx,S_fx,C_fx);

    DTFS_copy_fx(TARGETCW_FX,*CURRCW_Q_FX);
    /* Amplitude Quantization */
    DTFS_car2pol_fx(CURRCW_Q_FX); /* at this point currCW_q=curr_nq */

    /*As the upper cut of freqencies are normalized to 12800, we have to multiply upper cut off freq by
            2.56(1/12800 in Q15) */
    Ltempn = L_mult(CURRCW_Q_FX->upper_cut_off_freq_fx,10486);/* Q0+Q27 = Q28 */
    CURRCW_Q_FX->upper_cut_off_freq_fx = (Word16)L_shr(Ltempn,13);/*Q15 */
    Ltempn = L_mult(CURRCW_Q_FX->upper_cut_off_freq_of_interest_fx,10486);/* Q0+Q27 = Q28 */
    CURRCW_Q_FX->upper_cut_off_freq_of_interest_fx = (Word16)L_shr(Ltempn,13);/*Q15 */

    returnFlag = DTFS_quant_cw_fx(CURRCW_Q_FX,prevCW_lag,curr_lpc_fx,&POWER_IDX_FX,
                                  AMP_IDX_fx,lastLgainE_fx,lastHgainE_fx,lasterbE_fx,S_fx,C_fx);
    move16();

    /*De-normalize cut off frequencies */
    Ltempn = L_shl((Word32)CURRCW_Q_FX->upper_cut_off_freq_fx,13);/*Q28 */
    CURRCW_Q_FX->upper_cut_off_freq_fx = (Word16)find_remd(Ltempn, 20971,&Ltempd);
    move16();
    Ltempn = L_shl((Word32)CURRCW_Q_FX->upper_cut_off_freq_of_interest_fx,13);/*Q28 */
    CURRCW_Q_FX->upper_cut_off_freq_of_interest_fx = (Word16)find_remd(Ltempn, 20971,&Ltempd);

    push_indice_fx( st_fx, IND_AMP0, AMP_IDX_fx[0], 6 );
    push_indice_fx( st_fx, IND_AMP1, AMP_IDX_fx[1], 6 );
    push_indice_fx( st_fx, IND_POWER, POWER_IDX_FX, 6);

    /*Phase copying is done through copy_phase instead of car2pol and pol2car */
    copy_phase_fx(TARGETCW_FX,*CURRCW_Q_FX,TARGETCW_FX);
    /*Phase copying is done through copy_phase instead of car2pol and pol2car */
    copy_phase_fx(PREVDTFS_FX,*CURRCW_Q_FX,CURRCW_Q_FX);
    /* Copying phase spectrum over */
    /*mvr2r(PREVDTFS->b, CURRCW_Q->b, (short)(CURRCW_Q->lag>>1)+1 ); */

    /*DTFS_pol2car(CURRCW_Q); */
    /*DTFS_pol2car(TARGETCW); */

    tmp_fx = DTFS_alignment_fine_new_fx(*TARGETCW_FX,*CURRCW_Q_FX,S_fx,C_fx);
    move16();

    test();
    IF (sub(add(tmp_fx,12),28)>0 || add(tmp_fx,12)<0)
    {
        tmp_fx = 0;
        move16();
        returnFlag = 0;
        move16();
    }

    /*DTFS_phaseShift( CURRCW_Q,(float)(PI2*tmp/CURRCW_Q->lag) ); */
    Q2phaseShift_fx(CURRCW_Q_FX,tmp_fx,CURRCW_Q_FX->lag_fx,S_fx,C_fx);

    push_indice_fx( st_fx, IND_GLOBAL_ALIGNMENT, shr(add(tmp_fx,12),2), 3 );

    free(PREVDTFS_FX);
    return returnFlag;
}

/*-------------------------------------------------------------------*
* set_ppp_mode_fx()
 *
 * Determine if the current frame should be coded by PPP or not
 * Impose PPP - CELP - CELP pattern
 *-------------------------------------------------------------------*/
void set_ppp_mode_fx(
    Encoder_State_fx *st_fx,            /* i/o: state structure */
    Word16 *coder_type       /* i : coding type      */
    ,const Word16 noisy_speech_HO,      /* i  : SC-VBR noisy speech HO flag */
    const Word16 clean_speech_HO,      /* i  : SC-VBR clean speech HO flag */
    const Word16 NB_speech_HO,         /* i  : SC-VBR NB speech HO flag */
    const Word16 localVAD,
    const Word16 localVAD_he,          /* i  : HE-SAD flag without hangover */
    Word16 *vad_flag
    ,Word16 T_op_fx[]             /* i : open loop pitch lag */
    ,Word16 sp_aud_decision1      /* i  : Speech Audio Decision */
)
{

    test();
    test();
    test();
    test();
    test();

    IF (  sub( *vad_flag, 1) == 0 &&
          ( sub( noisy_speech_HO , 1 ) == 0 || sub( clean_speech_HO, 1 ) == 0 || sub( NB_speech_HO, 1) == 0 ) &&
          ( localVAD == 0 || localVAD_he == 0 ) )

    {
        *coder_type = UNVOICED;
        move16();
    }

    test();
    test();
    IF ( sub( *coder_type , INACTIVE ) == 0 && ( *vad_flag == 0 ) && sub( st_fx->last_nelp_mode_fx, 1 ) == 0 ) /* avoid HO frame go to GSC */
    {
        *coder_type = UNVOICED;
        move16();
    }


    /* force the coder to NELP mode during the first five frames */
    /* this will indicate the decoder that the coder is operating in the VBR mode */
    IF ( sub( st_fx->ini_frame_fx, 5) <0 )
    {
        *coder_type = UNVOICED;
        move16();
        *vad_flag = 1;
        move16();
    }
    /* Pattern PPP-CELP-CELP (pppcountE holds number of consecutive PPP frames) */
    test();
    IF ( sub( *coder_type, VOICED ) != 0 || sub( st_fx->last_coder_type_fx, TRANSITION ) == 0 )
    {
        /* ensure no transient to PPP transition */
        st_fx->pppcountE_fx = 0;
        move16();
    }
    ELSE
    {
        /* current mode is voiced */
        st_fx->pppcountE_fx = add( st_fx->pppcountE_fx, 1);
        test();
        test();
        test();
        test();
        IF ( ( sub( st_fx->pppcountE_fx, 1 ) == 0 && sub( st_fx->last_last_ppp_mode_fx, 1) != 0 && st_fx->rate_control_fx == 0 ) ||
        ( sub( st_fx->pppcountE_fx, 1 ) == 0 && st_fx->mode_QQF_fx != 0) )
        {
            st_fx->ppp_mode_fx = 1;
            move16();
            st_fx->core_brate_fx = PPP_NELP_2k80;
            move32();
        }
        ELSE IF ( sub(st_fx->pppcountE_fx, 2 ) == 0 )
        {
            test();
            IF ( st_fx->last_ppp_mode_fx != 0 && st_fx->mode_QQF_fx == 0 )
            {
                /* QFF mode */
                st_fx->ppp_mode_fx = 0;
                move16();
            }
            ELSE
            {
                /* QQF Mode */
                st_fx->ppp_mode_fx = 1;
                move16();
                st_fx->core_brate_fx = PPP_NELP_2k80;
                move32();
            }
        }
        ELSE
        {
            st_fx->ppp_mode_fx = 0;
            move16();
            st_fx->pppcountE_fx = 0;
            move16();
        }
    }


    test();
    IF ( st_fx->ppp_mode_fx == 0 && sub( st_fx->set_ppp_generic_fx, 1) == 0 )
    {
        st_fx->set_ppp_generic_fx = 0;
        move16();
        *coder_type = GENERIC;
        move16();
    }

    IF ( st_fx->last_core_fx == HQ_CORE )
    {
        st_fx->ppp_mode_fx = 0;
        move16();
        st_fx->set_ppp_generic_fx = 0;
        move16();
        *coder_type = TRANSITION;
        move16();
    }

    test();
    test();
    test();
    test();
    IF ( (st_fx->last_ppp_mode_fx != 0 ) && ( st_fx->ppp_mode_fx == 0 ) && ( sp_aud_decision1 != 0)
         && sub(st_fx->bwidth_fx, NB) == 0 && st_fx->Opt_SC_VBR_fx != 0 ) /*if it were about to go from ppp->HQ*/
    {
        st_fx->avoid_HQ_VBR_NB = 1;
        move16();
        *coder_type = GENERIC;
        move16();
    }

    test();
    test();
    test();
    IF ( (st_fx->last_nelp_mode_fx != 0) && ( sp_aud_decision1 != 0) && sub( st_fx->bwidth_fx, NB) == 0 && ( st_fx->Opt_SC_VBR_fx != 0 ) ) /*if it were about to go from nelp->HQ*/
    {
        st_fx->avoid_HQ_VBR_NB = 1;
        move16();
        *coder_type = GENERIC;
        move16();
    }


    test();
    test();
    test();
    IF( (  sub(st_fx->old_pitch_buf_fx[(2*NB_SUBFR)-1], PPP_LAG_THRLD_Q6) > 0 ||
           sub(T_op_fx[1], PPP_LAG_THRLD)>0 || !st_fx->last_Opt_SC_VBR_fx ) &&
        sub(st_fx->ppp_mode_fx,1)==0 )
    {
        st_fx->ppp_mode_fx=0;
        move16();
        st_fx->core_brate_fx = ACELP_7k20;
        move32();
    }


    return;
}
