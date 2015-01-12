/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define L_ENR            (NB_SSF+2)



/*-------------------------------------------------------------------*
 * find_ener_decrease_fx()
 *
 * Find maximum energy ration between short subblocks in case
 * energy is trailing off after a spike
 *-------------------------------------------------------------------*/

static Word16 find_ener_decrease_fx(     /* o  : maximum energy ratio       Q10                    */
    const Word16 ind_deltaMax,           /* i  : index of the beginning of maximum energy search   */
    const Word32 *pt_enr_ssf             /* i  : Pointer to the energy buffer                      */
)
{
    Word16 i, j, end, flag;
    Word16 wtmp0, wtmp1;
    Word32 maxEnr, minEnr;
    Word16 dE2, exp0, exp1;

    dE2 = 0;
    move16();

    j = ind_deltaMax+2;
    move16();
    end = j+L_ENR;
    move16();
    maxEnr = L_add(pt_enr_ssf[j], 0);
    j = add(j, 1);
    flag = 0;
    move16();
    FOR( i=j; i<end; i++ )
    {
        test();
        IF( (L_sub(pt_enr_ssf[i], maxEnr)>0) && (flag == 0) )
        {
            maxEnr = L_add(pt_enr_ssf[i], 0);        /*Q0*/
            j = add(j, 1);
        }
        ELSE
        {
            flag = 1;
            move16();
        }
    }

    minEnr = L_add(maxEnr, 0);
    FOR( i=j; i<end; i++ )
    {
        minEnr = L_min(minEnr,  pt_enr_ssf[i]);
    }


    minEnr = L_add(minEnr, 100000);

    exp0 = norm_l(minEnr);
    wtmp0 = extract_h(L_shl(minEnr, exp0));
    exp1 = sub(norm_l(maxEnr),1);
    wtmp1 = extract_h(L_shl(maxEnr, exp1));
    wtmp1 = div_s(wtmp1,wtmp0);
    dE2 = shr_r(wtmp1, add(sub(exp1, exp0),15-10));  /*Q10*/

    return dE2;
}

/*-------------------------------------------------------------------*
* find_uv_fx()
*
* Decision about coder type
*-------------------------------------------------------------------*/

Word16 find_uv_fx(                           /* o  : coding type                                               */
    Encoder_State_fx *st_fx,                 /* i/o: encoder state structure                                   */
    const Word16 *T_op_fr,                   /* i  : pointer to adjusted fractional pitch (4 val.)             Q6*/
    const Word16 *voicing_fr,                /* i  : refined correlation for each subframes                    Q15*/
    const Word16 *voicing,                   /* i  : correlation for 3 half-frames                            Q15*/
    const Word16 *speech,                    /* i  : pointer to speech signal for E computation              Q_new*/
    const Word16 localVAD,                   /* i  : vad without hangover                                       */
    const Word32 *ee,                        /* i  : lf/hf Energy ratio for present frame                     Q6*/
    const Word16 corr_shift,                 /* i  : normalized correlation correction in noise                Q15*/
    const Word16 relE,                       /* i  : relative frame energy                                     Q8*/
    const Word16 Etot,                       /* i  : total energy                                             Q8*/
    const Word32 hp_E[],                     /* i  : energy in HF                                Q_new + Q_SCALE*/
    const Word16 Q_new,
    Word16 *flag_spitch,               /* i/o: flag to indicate very short stable pitch and high correlation */
    const Word16 voicing_sm                  /* i  : smoothed open-loop pitch gains                            */
)
{
    Word16 coder_type, i;
    Word32 mean_ee, dE1, fac_32;
    const Word16 *pt_speech;
    Word32 L_tmp, enr_ssf[2*NB_SSF+2*NB_SSF+2], E_min_th;
    Word16 dE2;
    Word16 ind_deltaMax, tmp_offset_flag;
    Word32 Ltmp0, *pt_enr_ssf, *pt_enr_ssf1, dE2_th;
    Word16 exp0, exp1, Q_in;
    Word16 wtmp0, wtmp1;
    Word16 fac, mean_voi3,dE3;
    Word16 relE_thres;
    Word16 mean_voi3_offset;
    Word16 voicing_m, dpit1, dpit2, dpit3;
    Word16 ee0_th, ee1_th, voi_th, nb_cond, flag_low_relE;

    Q_in = sub(Q_new,1);

    /*-----------------------------------------------------------------*
     * Detect sudden energy increases to catch voice and music
     * temporal events (dE1)
     *
     * - Find maximum energy per short subblocks.
     *   Two subblock sets are used shifted by half the subblock length
     * - Find maximum energy ratio between adjacent subblocks
     *-----------------------------------------------------------------*/

    /* Find maximum energy per short subblocks  */
    pt_speech = speech - SSF;
    pt_enr_ssf = enr_ssf + 2*NB_SSF;
    FOR( i=0 ; i < 2*(NB_SSF+1) ; i++ )
    {
        emaximum_fx(Q_in, pt_speech, SSF, pt_enr_ssf );
        pt_speech += (SSF/2);
        pt_enr_ssf++;
    }

    dE1 = 0;
    move16();
    ind_deltaMax = 0;
    move16();
    pt_enr_ssf = enr_ssf + 2*NB_SSF;
    pt_enr_ssf1 = pt_enr_ssf + 2;

    /* Test on energy increase between adjacent sub-subframes */
    exp1 = 0;
    move16();
    FOR( i=0; i < 2*NB_SSF; i++ )
    {
        /*fac = *pt_enr_ssf1 / (*pt_enr_ssf + 1);*/
        Ltmp0 = L_max(*pt_enr_ssf, 1);
        exp0 = norm_l(Ltmp0);
        wtmp0 = extract_h(L_shl(Ltmp0, exp0));
        exp1 = sub(norm_l(*pt_enr_ssf1),1);
        wtmp1 = extract_h(L_shl(*pt_enr_ssf1, exp1));
        fac = div_s(wtmp1,wtmp0);
        fac_32 = L_shr(L_deposit_l(fac), add(sub(exp1, exp0),15-13));         /* fac32 in Q13*/

        if(L_sub(fac_32, dE1 ) > 0)
        {
            ind_deltaMax = i;
            move16();
        }

        dE1 = L_max(dE1, fac_32);

        pt_enr_ssf++;
        pt_enr_ssf1++;
    }

    /*-----------------------------------------------------------------*
     * Average spectral tilt
     * Average voicing (normalized correlation)
     *-----------------------------------------------------------------*/

    /*mean_ee = 1.0f/3.0f * (st->ee_old + ee[0] + ee[1]); */   /* coefficients take into account the position of the window */
    mean_ee = L_add(L_add(st_fx->ee_old_fx, ee[0]), ee[1]);
    mean_ee = Mult_32_16(mean_ee, 10923);        /*Q6*/

    /* mean_voi3 = 1.0f/3.0f * (voicing[0] + voicing[1] + voicing[2]);*/
    Ltmp0 = L_mult( voicing[0], 10923);
    Ltmp0 = L_mac(Ltmp0, voicing[1], 10923);
    mean_voi3 = mac_r(Ltmp0, voicing[2], 10923);    /*Q15*/

    /*-----------------------------------------------------------------*
     * Total frame energy difference (dE3)
     *-----------------------------------------------------------------*/

    dE3 = sub(Etot, st_fx->Etot_last_fx);  /*Q8*/

    /*-----------------------------------------------------------------*
    * Energy decrease after spike (dE2)
    *-----------------------------------------------------------------*/

    /* set different thresholds and conditions for NB and WB input */
    dE2_th = 30<<10;
    move32();
    nb_cond = 1;
    move16(); /* no additional condition for WB input */
    IF ( sub(st_fx->input_bwidth_fx,NB) == 0 )
    {
        dE2_th = 21<<10;
        move32();
        if(sub(add(mean_voi3, corr_shift), 22282) >= 0)  /*( mean_voi3 + corr_shift ) >= 0.68f*/
        {
            nb_cond = 0;
            move16();
        }
    }

    /* calcualte maximum energy decrease */
    dE2 = 0;
    move16();                                         /* Test on energy decrease after an energy spike  */
    pt_enr_ssf = enr_ssf + 2*NB_SSF;

    test();
    IF( L_sub(dE1, 30<<13) > 0  && nb_cond)  /*>30 Q13*/
    {
        IF( sub(sub(shl(NB_SSF,1), ind_deltaMax),L_ENR) < 0 )
        {
            st_fx->old_ind_deltaMax_fx = ind_deltaMax;
            move16();
            Copy32( pt_enr_ssf, st_fx->old_enr_ssf_fx, 2*NB_SSF );
        }
        ELSE
        {
            st_fx->old_ind_deltaMax_fx = -1;
            move16();
            dE2 = find_ener_decrease_fx( ind_deltaMax, pt_enr_ssf );        /*Q10*/

            if( L_sub(dE2,dE2_th) > 0)
            {
                st_fx->spike_hyst_fx = 0;
                move16();
            }
        }
    }
    ELSE
    {
        IF( st_fx->old_ind_deltaMax_fx >= 0 )
        {
            Copy32( st_fx->old_enr_ssf_fx, enr_ssf, 2*NB_SSF );
            dE2 = find_ener_decrease_fx( st_fx->old_ind_deltaMax_fx, enr_ssf );

            if( L_sub(dE2,dE2_th) > 0)
            {
                st_fx->spike_hyst_fx = 1;
                move16();
            }
        }

        st_fx->old_ind_deltaMax_fx = -1;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Detection of voiced offsets (tmp_offset_flag)
     *-----------------------------------------------------------------*/

    tmp_offset_flag = 1;
    move16();

    IF ( sub(st_fx->input_bwidth_fx, NB) != 0 )
    {
        ee0_th = 154;    /*2.4 in Q6 */     move16();
        voi_th = 24248;  /*0.74f Q15 */     move16();
    }
    ELSE
    {
        ee0_th = 627;    /*9.8f Q6 */      move16();
        voi_th = 24904;  /*0.76f Q15*/     move16();
    }

    test();
    test();
    test();
    if( ( sub(st_fx->last_coder_type_raw_fx,UNVOICED) == 0 ) ||             /* previous frame was unvoiced  */
            ( ( L_sub(ee[0],ee0_th) < 0 ) && ( L_sub(hp_E[0],L_shl(E_MIN_FX,Q_new)) > 0 ) &&         /* energy is concentrated in high frequencies provided that some energy is present in HF */
              ( sub(add(voicing[0],corr_shift),voi_th) < 0 ) ) )             /* normalized correlation is low */
    {
        tmp_offset_flag = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Decision about UC
     *-----------------------------------------------------------------*/

    /* SC-VBR - set additional parameters and thresholds for SC-VBR */
    mean_voi3_offset = 0;
    move16();
    flag_low_relE = 0;
    move16();
    ee1_th = 608;     /*9.5 Q6*/  move16();
    IF ( st_fx->Opt_SC_VBR_fx )
    {
        ee1_th = 544;   /*8.5f Q6*/ move16();

        /* SC-VBR - determine the threshold on relative energy as a function of lp_noise */
        IF ( sub(st_fx->input_bwidth_fx,NB) != 0 )
        {
            /*relE_thres = 0.700f * st->lp_noise - 33.5f; (lp_noise in Q8, constant Q8<<16) */
            L_tmp = L_mac(-562036736, 22938, st_fx->lp_noise_fx);
            if ( st_fx->Last_Resort_fx == 0 )
            {
                /*relE_thres = 0.650f * st->lp_noise - 33.5f; (lp_noise in Q8, constant Q8<<16)*/
                L_tmp = L_mac(-562036736, 21299, st_fx->lp_noise_fx);
            }
            relE_thres = round_fx(L_tmp);
        }
        ELSE
        {

            /*relE_thres = 0.60f * st->lp_noise - 28.2f; (lp_noise in Q8, constant Q8<<16)*/
            L_tmp = L_mac(-473117491, 19661, st_fx->lp_noise_fx);
            relE_thres = round_fx(L_tmp);
        }
        relE_thres  = s_max(relE_thres , -6400);  /* Q8 */

        /* SC-VBR = set flag on low relative energy */
        if ( sub(relE,relE_thres) < 0 )
        {
            flag_low_relE = 1;
            move16();
        }

        /* SC-VBR - correction of voicing threshold for NB inputs (important only in noisy conditions) */
        test();
        if ( sub(st_fx->input_bwidth_fx,NB) == 0 && sub(st_fx->vadnoise_fx,20<<8) < 0 ) /* vadnoise in Q8, constant Q0<<8 */
        {
            mean_voi3_offset = 1638;  /*0.05f Q15*/     move16();
        }
    }

    /* make decision whether frame is unvoiced */
    E_min_th = L_shl(E_MIN_FX,Q_new);
    coder_type = GENERIC;
    move16();
    IF ( sub(st_fx->input_bwidth_fx,NB) == 0 )
    {
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
        if( ( ( sub(add(mean_voi3, corr_shift),add(22282,mean_voi3_offset)) <  0) &&     /* normalized correlation low  */
                ( ( sub(add(voicing[2], corr_shift),25887) ) < 0 ) &&               /* normalized correlation low on look-ahead - onset detection */
                ( L_sub(ee[0], 640) < 0 ) && ( L_sub(hp_E[0], E_min_th) > 0 ) &&    /* energy concentrated in high frequencies provided that some energy is present in HF...  */
                ( L_sub(ee[1], ee1_th) < 0 ) && ( L_sub(hp_E[1], E_min_th) > 0 ) && /* ... biased towards look-ahead to detect onsets  */
                ( tmp_offset_flag == 0 ) &&                                         /* Take care of voiced offsets */
                ( st_fx->music_hysteresis_fx == 0 ) &&                              /*  ... and in segment after AUDIO frames   */
                ( L_sub(dE1, 237568) <= 0 ) &&                                      /* Avoid on sharp energy spikes  */
                ( L_sub(st_fx->old_dE1_fx,237568) <= 0 ) &&                         /*   + one frame hysteresis   */
                ( st_fx->spike_hyst_fx < 0 ) ) ||                                   /* Avoid after sharp energy spikes followed by decay (e.g. castanets) */
                flag_low_relE )                                                     /* low relative frame energy (only for SC-VBR) */
        {
            coder_type = UNVOICED;
            move16();
        }
    }
    ELSE
    {
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
        if( ( ( sub(add(mean_voi3, corr_shift),add(22774,mean_voi3_offset)) <  0) &&         /* normalized correlation low  */
        ( ( sub(add(voicing[2], corr_shift),25887) ) < 0 ) &&               /* normalized correlation low on look-ahead - onset detection */
        ( L_sub(ee[0], 397) < 0 ) && ( L_sub(hp_E[0], E_min_th) > 0 ) &&    /* energy concentrated in high frequencies provided that some energy is present in HF...  */
        ( L_sub(ee[1], 397) < 0 ) && ( L_sub(hp_E[1], E_min_th) > 0 ) &&    /* ... biased towards look-ahead to detect onsets  */
        ( tmp_offset_flag == 0 ) &&                                         /* Take care of voiced offsets */
        ( st_fx->music_hysteresis_fx == 0 ) &&                              /*  ... and in segment after AUDIO frames   */
        ( L_sub(dE1, 245760) <= 0 ) &&                                      /* Avoid on sharp energy spikes  */
        ( L_sub(st_fx->old_dE1_fx,245760) <= 0 ) &&                         /*   + one frame hysteresis   */
        ( st_fx->spike_hyst_fx < 0 ) )                                      /* Avoid after sharp energy spikes followed by decay (e.g. castanets) */
        || ( flag_low_relE
        && ( L_sub(st_fx->old_dE1_fx,245760) <= 0 )
           )
          )                                                  /* low relative frame energy (only for SC-VBR) */
        {
            coder_type = UNVOICED;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Decision about VC
     *-----------------------------------------------------------------*/

    st_fx->set_ppp_generic_fx = 0;
    move16();

    test();
    IF( sub(localVAD,1) == 0 && sub(coder_type,GENERIC) == 0 )
    {
        dpit1 = abs_s( sub(T_op_fr[1], T_op_fr[0]));
        dpit2 = abs_s( sub(T_op_fr[2], T_op_fr[1]));
        dpit3 = abs_s( sub(T_op_fr[3], T_op_fr[2]));

        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF(  ( sub(voicing_fr[0],19825)> 0 ) &&          /* normalized correlation high in 1st sf.  */
             ( sub(voicing_fr[1],19825) > 0) &&          /* normalized correlation high in 2st sf.  */
             ( sub(voicing_fr[2],19825) > 0) &&          /* normalized correlation high in 3st sf.  */
             ( sub(voicing_fr[3],19825) > 0) &&          /* normalized correlation high in 4st sf.  */
             ( L_sub(mean_ee,256) > 0 ) &&               /* energy concentrated in low frequencies  */
             ( sub(dpit1,3<<6)  < 0 ) &&
             ( sub(dpit2,3<<6)  < 0 ) &&
             ( sub(dpit3,3<<6)  < 0 ) )
        {
            coder_type = VOICED;
            move16();
        }
        ELSE IF ( st_fx->Opt_SC_VBR_fx && sub(st_fx->input_bwidth_fx,NB) == 0 && sub(st_fx->vadnoise_fx,20<<8) < 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF( sub(voicing_fr[0],8192)> 0 &&              /* normalized correlation high in 1st sf.  */
                ( sub(voicing_fr[1],8192) > 0) &&          /* normalized correlation high in 2st sf.  */
                ( sub(voicing_fr[2],8192) > 0) &&          /* normalized correlation high in 3st sf.  */
                ( sub(voicing_fr[3],8192) > 0) &&          /* normalized correlation high in 4st sf.  */
                ( L_sub(mean_ee,64) > 0 ) &&               /* energy concentrated in low frequencies  */
                ( sub(dpit1,5<<6)  < 0 ) &&
                ( sub(dpit2,5<<6)  < 0 ) &&
                ( sub(dpit3,5<<6)  < 0 ) )
            {
                st_fx->set_ppp_generic_fx = 1;
                move16();
                coder_type = VOICED;
                move16();
            }
        }

        /* set VOICED mode for frames with very stable pitch and high correlation
           and avoid to switch to AUDIO/MUSIC later                              */
        voicing_m = mac_r(L_mac(L_mac(L_mult(voicing_fr[3], 8192), voicing_fr[2], 8192),voicing_fr[1], 8192),voicing_fr[0], 8192);
        test();
        test();
        test();
        test();
        test();
        IF ( *flag_spitch || ( sub(dpit1,3<<6) <= 0 && sub(dpit2,3<<6) <= 0 && sub(dpit3,3<<6) <= 0 &&
                               sub(voicing_m, 31130) > 0 && sub(voicing_sm, 31785) > 0 ) )
        {
            coder_type = VOICED;
            move16();
            *flag_spitch = 1;
            move16();/*to avoid switch to AUDIO/MUSIC later*/
        }
    }

    /*-----------------------------------------------------------------*
     * Channel-aware mode - set RF mode and total bitrate
     *-----------------------------------------------------------------*/

    st_fx->rf_mode = st_fx->Opt_RF_ON;
    move16();

    IF ( sub ( coder_type, GENERIC ) == 0 )
    {
        test();
        test();
        test();
        test();
        IF( ( sub(voicing_fr[0],6554) < 0) &&          /* normalized correlation high in 2st sf.  */
            ( sub(voicing_fr[1],6554) < 0) &&          /* normalized correlation high in 2st sf.  */
            ( sub(voicing_fr[2],6554) < 0) &&          /* normalized correlation high in 3rd sf.  */
            ( sub(voicing_fr[3],6554) < 0) &&          /* normalized correlation high in 4th sf.  */
            ( sub(st_fx->vadnoise_fx, 25 << 8 ) > 0 )) /* when speech is clean */

        {
            st_fx->rf_mode = 0;
            move16();
            /* Current frame cannot be compressed to pack the partial redundancy;*/
        }
    }





    /*-----------------------------------------------------------------*
     * Updates
     *-----------------------------------------------------------------*/

    /* update spike hysteresis parameters */
    test();
    if( st_fx->spike_hyst_fx >= 0 && sub(st_fx->spike_hyst_fx,2) < 0 )
    {
        st_fx->spike_hyst_fx = add(st_fx->spike_hyst_fx,1);
    }

    /* reset spike hysteresis */
    test();
    test();
    test();
    if( ( sub(st_fx->spike_hyst_fx,1) > 0 ) &&
            ( sub(dE3,5<<8) > 0 ||                                                           /* energy increases               */
              ( sub(relE, -3328) > 0 && ( sub(add(mean_voi3, corr_shift),22774) > 0) ) ) )   /* normalized correlation is high */
    {
        st_fx->spike_hyst_fx = -1;
        move16();
    }

    /* update tilt parameters */
    st_fx->ee_old_fx = ee[1];
    move32();        /*Q6*/
    st_fx->old_dE1_fx = dE1;
    move32();        /*Q13*/

    /* save the raw coder_type for various modules later in the codec (the reason is that e.g. UNVOICED is lost at higher rates) */
    st_fx->coder_type_raw_fx = coder_type;
    move16();

    return coder_type;
}
