/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constantes
 *-------------------------------------------------------------------*/
#define NB_VOIC_FX         13
#define DIV_NB_VOIC_FX     2521

#define ALPA_FX         31130
#define ALPAM1_FX       (32768-ALPA_FX)

#define BETA_FX            819
#define AFREQ_THR       2

#define HANGOVER_DELAY  2

/*======================================================================*/
/* FUNCTION : Pit_exc_contribution_len_fx()								*/
/*----------------------------------------------------------------------*/
/* PURPOSE : Determine up to which band the pit contribution is significant*/
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    */
/* _ (Struct)	st_fx		  : encoder static memory					*/
/* _ (Word16[]) dct_res		  : DCT of residual		    Qnew		    */
/* _ (Word16[])	dct_pitex	  : DCT of pitch contribution Qnew		    */
/* _ (Word16[]) pitch_buf	  : Pitch per subframe		    Q6	        */
/* _ (Word16[]) nb_subfr	  : Number of subframe considered           */
/* _ (Word16) hangover		  : hangover for the time contribution switching*/
/* _ (Word16) Qnew		      :											*/
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16[]) dct_res		  : DCT of residual		    Qnew		    */
/* _ (Word16[])	dct_pitex	  : DCT of pitch contribution Qnew		    */
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/

Word16 Pit_exc_contribution_len_fx(   /* o  : bin where pitch contribution is significant */
    Encoder_State_fx *st_fx,              /* i/o: state structure                                  */
    const Word16 *dct_res,         /* i  : DCT of residual                 */
    Word16 *dct_pitex,       /* i/o: DCT of pitch contribution       */
    Word16 *pitch_buf,       /* i/o: Pitch per subframe              */
    const Word16 nb_subfr,         /* i  : Number of subframe considered   */
    Word16 *hangover,         /* i  : hangover for the time contribution switching */
    const Word16 coder_type,        /* i  : coding type */
    Word16 Qnew
)
{

    Word16 corr_dct_pit[MBANDS_LOC];
    Word32 corr_tmp,L_tmp;
    Word16 av_corr, min_corr, ftmp, tmp_ex, tmp_res;
    Word16 freq, i, j;
    Word16 last_pit_band, pit_contr_idx, last_pit_bin;
    Word32 ener_res;
    Word32 ener_pit;
    Word16 low_pit, F1st_harm, F8th_harm;
    Word16 corr_dct_pit_tmp[MBANDS_LOC];
    Word16 time_flg = 0;
    Word16 Len, max_len;
    Word16 tmp_dec;
    Word16 Mbands_loc = MBANDS_LOC-2;
    Word16 exp1,tmp,exp2;
    Word32 L_tmp1, ener_init;
    Word16 exp_norm;
    Word16 norm;
    Word16 val_thrs;

    if( sub(st_fx->L_frame_fx,L_FRAME16k) == 0)
    {
        Mbands_loc = MBANDS_LOC;
        move16();
    }

    minimum_fx( pitch_buf, nb_subfr, &low_pit);
    exp1 = norm_s(low_pit);
    tmp = shl(low_pit,exp1);
    {
        /*F1st_harm = 12800.0f/low_pit;*/
        tmp = div_s(12800,tmp);  /*15-6-exp1(->9-exp1)*/
        F1st_harm = shr_r(tmp,sub(5,exp1));  /*Q4*/
    }

    /*F8th_harm = 8.0f*F1st_harm;*/
    F8th_harm = extract_l(L_shr_r(L_mult0(F1st_harm,8),2));  /*Q2*/

    freq = 0;
    move16();
    ener_init = L_shl(3,2*Qnew-5);    /*(0.1->3 in Q5)  2*Qnew*/
    FOR (i = 0; i <Mbands_loc; i++) /* up to maximum allowed voiced critical band */
    {
        corr_tmp = L_deposit_l(0);
        ener_res = L_add(ener_init, 0);
        ener_pit = L_add(ener_init, 0);

        FOR (j = 0; j < mfreq_bindiv_loc_fx[i]; j++) /* up to maximum allowed voiced critical band */
        {
            tmp_ex = mult_r(dct_pitex[j+freq],8192);
            tmp_res = mult_r(dct_res[j+freq],8192);
            corr_tmp = L_mac0(corr_tmp,tmp_res,tmp_ex);  /*2*Qnew*/
            ener_res = L_mac0(ener_res,tmp_res,tmp_res);	/*2*Qnew*/
            ener_pit = L_mac0(ener_pit,tmp_ex,tmp_ex); /*2*Qnew*/
        }

        L_tmp1 = Mult_32_32(ener_res,ener_pit);
        exp2 = norm_l(L_tmp1);
        L_tmp1 = L_shl(L_tmp1, exp2);
        exp_norm = sub(30, exp2);
        L_tmp1 = Isqrt_lc(L_tmp1, &exp_norm);
        norm = extract_h(L_tmp1);  /*15-exp_norm*/
        L_tmp1 = L_shl(Mult_32_16(corr_tmp,norm),exp_norm);
        corr_dct_pit[i] = round_fx(L_shl(L_tmp1,14)); /*Q14*/

        freq = add(freq,mfreq_bindiv_loc_fx[i]);
    }

    val_thrs = 8192;
    move16(); /* 0.5 in Q14*/
    /* Smooth the inter-correlation value and skip the last band for the average (since last band is almost always 0)*/
    tmp = mac_r(L_mult(ALPA_FX,corr_dct_pit[0]),ALPAM1_FX,corr_dct_pit[1]);/*Qnew*/
    tmp  = s_max(tmp , val_thrs);

    corr_dct_pit_tmp[0] = shl(sub(tmp,val_thrs),1);
    move16();

    FOR (i = 1; i <Mbands_loc-1; i++) /* up to maximum allowed voiced critical band */
    {
        L_tmp = L_mult(BETA_FX,corr_dct_pit[i-1]);
        L_tmp = L_mac(L_tmp, BETA_FX,corr_dct_pit[i+1]);

        tmp = mac_r(L_tmp, ALPA_FX, corr_dct_pit[i]);
        tmp  = s_max(tmp , val_thrs);

        corr_dct_pit_tmp[i] = shl(sub(tmp,val_thrs),1);
        move16();
    }
    tmp = mac_r(L_mult(ALPA_FX,corr_dct_pit[i]),ALPAM1_FX,corr_dct_pit[i-1]);/*Qnew*/
    tmp  = s_max(tmp , val_thrs);
    corr_dct_pit_tmp[i] = shl(sub(tmp,val_thrs),1);
    move16();

    Copy(corr_dct_pit_tmp, corr_dct_pit, Mbands_loc);

    L_tmp1 = L_mult(DIV_NB_VOIC_FX,corr_dct_pit[0]); /*Qnew*/
    FOR (i = 1; i <NB_VOIC_FX; i++) /* up to maximum allowed voiced critical band */
    {
        L_tmp1 = L_mac(L_tmp1,DIV_NB_VOIC_FX,corr_dct_pit[i]);
    }
    av_corr = round_fx(L_tmp1); /*Qnew*/

    /* Find the cut-off freq similarly to HSX */
    last_pit_band = 0;
    move16();
    av_corr = round_fx(L_shl(L_mult0(av_corr,6400),16-12)); /*Q14*Q0-12=Q2*/

    if( L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0 )
    {
        av_corr = shl(av_corr,1);     /*Q2 Correlation really poor at low rate, time domain still valide*/
    }
    min_corr = abs_s(sub(mfreq_loc_Q2fx[0],av_corr));  /*Q2*/

    FOR (i = 1; i <Mbands_loc; i++) /* up to maximum allowed voiced critical band */
    {
        ftmp = abs_s(sub(mfreq_loc_Q2fx[i],av_corr));  /*Q2*/

        IF(sub(ftmp,min_corr) < 0)
        {
            last_pit_band = i;
            move16();
            min_corr = ftmp;
            move16();
        }
    }

    IF( sub(F8th_harm,mfreq_loc_Q2fx[last_pit_band]) > 0 )
    {
        DO
        {
            last_pit_band = add(last_pit_band,1);
        }
        WHILE( sub(F8th_harm,mfreq_loc_Q2fx[last_pit_band]) >=  0 );
    }
    test();
    test();
    test();
    IF( sub(last_pit_band,7+BAND1k2) > 0 && (L_sub(st_fx->core_brate_fx,CFREQ_BITRATE) < 0 || sub(st_fx->bwidth_fx,NB) == 0) )/*Added for 9.1*/
    {
        last_pit_band = 7+BAND1k2;
        move16();
    }
    ELSE IF ( sub(last_pit_band,10+BAND1k2) > 0 && L_sub(st_fx->core_brate_fx,CFREQ_BITRATE) >= 0 )
    {
        last_pit_band = add(10,BAND1k2);
    }

    time_flg = 0;
    move16();
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
    IF( (st_fx->mem_last_pit_band_fx > 0 && sub(st_fx->old_corr_fx,16384) > 0 && sub(st_fx->mold_corr_fx,16384) > 0 && sub(st_fx->lt_gpitch_fx,19661) >= 0/*1.5f*GPIT_THR*/ )
        || (sub(last_pit_band,6) > 0)
        || (sub(last_pit_band,4) >= 0 && sub(st_fx->lt_gpitch_fx,19661) >= 0/*1.5f*GPIT_THR*/ && sub(st_fx->old_corr_fx,22938) > 0)
        || (sub(last_pit_band,BAND1k2) > 0 &&  sub(st_fx->mold_corr_fx,26214) > 0 && sub(st_fx->lt_gpitch_fx,13107) >= 0/*GPIT_THR*/)
      )
    {
        tmp_dec = 1;
        move16();
    }
    ELSE
    {
        tmp_dec = 0;
        move16();
    }

    /* Different past and current decision */
    test();
    test();
    test();
    IF ( (st_fx->mem_last_pit_band_fx == 0 && sub(tmp_dec,1) == 0) || (st_fx->mem_last_pit_band_fx > 0 && tmp_dec == 0) )
    {
        IF( *hangover == 0 )
        {
            time_flg = tmp_dec;
            move16();
            *hangover = HANGOVER_DELAY;
            move16();
        }
        ELSE
        {
            time_flg = 0;
            move16();
            if( st_fx->mem_last_pit_band_fx > 0 )
            {
                time_flg = 1;
                move16();
            }

            (*hangover) = sub((*hangover),1);
            if( *hangover < 0 )
            {
                *hangover = 0;
                move16();
            }
        }
    }
    ELSE
    {
        time_flg = tmp_dec;
        move16();
        *hangover = HANGOVER_DELAY;
        move16();
    }

    /* Decicison on final lenght of time contribution */
    pit_contr_idx = 0;
    move16();
    test();
    test();
    IF( sub(time_flg,1) == 0 || sub(coder_type,INACTIVE) != 0 || st_fx->GSC_noisy_speech_fx )
    {
        test();
        test();
        /*if(st_fx->core_brate_fx  <ACELP_9k60)*/
        IF(L_sub(st_fx->core_brate_fx,ACELP_9k60)  < 0 && sub(low_pit , 4096) < 0)
        {
            last_pit_band = add(9 , BAND1k2);
            if(sub(st_fx->bwidth_fx,NB) == 0)
            {
                last_pit_band = add(7,BAND1k2);
            }
        }
        ELSE IF(L_sub(st_fx->core_brate_fx,ACELP_9k60)  < 0 && sub(low_pit , 8192) < 0)
        {
            last_pit_band = add(5 , BAND1k2);
        }
        ELSE IF(L_sub(st_fx->core_brate_fx,ACELP_9k60)  < 0 )
        {
            last_pit_band = add(3 , BAND1k2);
        }
        ELSE IF( sub(last_pit_band,add(BAND1k2,1)) < 0 )
        {
            last_pit_band = add(BAND1k2,1);
        }
        last_pit_bin = mfreq_loc_div_25[last_pit_band];
        move16();

        st_fx->bpf_off_fx = 0;
        move16();

        max_len = sub(st_fx->L_frame_fx,last_pit_bin);

        if( sub(st_fx->bwidth_fx,NB) == 0 )
        {
            max_len = sub(160,last_pit_bin);
        }

        Len = 80;
        move16();
        if(sub(max_len,80) < 0)
        {
            Len = max_len;
            move16();
        }
        test();
        IF((L_sub(st_fx->core_brate_fx,ACELP_8k00) == 0) && (sub(st_fx->bwidth_fx,NB) != 0))
        {
            move16();     /*ptr init*/
            FOR (i=0; i < max_len; i++)
            {
                dct_pitex[i+last_pit_bin] = 0;
                move16();
            }
        }
        ELSE
        {

            FOR (i = 0; i < Len; i++)
            {
                dct_pitex[i+last_pit_bin] = mult_r(dct_pitex[i+last_pit_bin],sm_table_fx[i]);
            }
            FOR (; i < max_len; i++)
            {
                dct_pitex[i+last_pit_bin] = 0;
                move16();
            }
        }
        st_fx->mem_last_pit_band_fx = last_pit_band;
        move16();
        pit_contr_idx = sub(last_pit_band,BAND1k2);
    }
    ELSE
    {
        set16_fx(dct_pitex, 0, st_fx->L_frame_fx);
        st_fx->bpf_off_fx = 1;
        move16();
        last_pit_bin = 0;
        move16();
        last_pit_band = 0;
        move16();
        pit_contr_idx = 0;
        move16();
        st_fx->mem_last_pit_band_fx = 0;
        move16();

        {
            set16_fx( pitch_buf, shl(L_SUBFR,6), NB_SUBFR );
        }
        /* pitch contribution useless - delete all previously written indices belonging to pitch contribution */
        FOR ( i = TAG_ACELP_SUBFR_LOOP_START; i < TAG_ACELP_SUBFR_LOOP_END; i++ )
        {
            IF ( st_fx->ind_list_fx[i].nb_bits != -1 )
            {
                st_fx->nb_bits_tot_fx = sub(st_fx->nb_bits_tot_fx,st_fx->ind_list_fx[i].nb_bits);
                st_fx->ind_list_fx[i].nb_bits = -1;
                move16();
            }
        }

        IF ( st_fx->ind_list_fx[IND_ES_PRED].nb_bits != -1 )
        {
            st_fx->nb_bits_tot_fx = sub(st_fx->nb_bits_tot_fx,st_fx->ind_list_fx[IND_ES_PRED].nb_bits);
            st_fx->ind_list_fx[IND_ES_PRED].nb_bits = -1;
            move16();
        }
    }
    IF( L_sub(st_fx->core_brate_fx,CFREQ_BITRATE) < 0 )
    {
        IF(L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0)
        {
            if(pit_contr_idx>0)
            {
                pit_contr_idx=1;
                move16();
            }

            IF( sub(coder_type,INACTIVE) == 0 )
            {
                push_indice_fx( st_fx, IND_PIT_CONTR_IDX, pit_contr_idx, 1 );
            }
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_PIT_CONTR_IDX, pit_contr_idx, 3 );
        }
    }
    ELSE
    {
        push_indice_fx( st_fx, IND_PIT_CONTR_IDX, pit_contr_idx, 4 );
    }

    return last_pit_bin;

}
