/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"
#include "stl.h"
#include "basop_mpy.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/
#define ALPHA_ENER_FAST_FX   29491      /* Fast adaptation (noise down, speech up) */
#define ALPHA_ENER_SLOW_FX   32440      /* Fast adaptation (noise down, speech up) */

#define MIN_CNT           50            /* Minimum frame number before SID interval adaptation */

#define SNR_H_FX          13056			/* Estimated SNR and corresponding SID interval        */
#define SNR_L_FX          9216          /* 51dB corresponds to 25dB SNR before noise supressor */
#define INT_H             50
#define INT_L             8
#define RATIO             (INT_H - INT_L)/((SNR_H_FX - SNR_L_FX)/256)

#define LTE_VAR_FX              -1024               /* Q8, -4.0f */

#define CNG_TYPE_HO       20      /* hangover for switching between CNG types */

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/

static void update_SID_cnt( Encoder_State_fx *st_fx );


/*==================================================================================*/
/* FUNCTION : dtx_fx()	 									        	            */
/*----------------------------------------------------------------------------------*/
/* PURPOSE :  Discontinuous transmission operation                                  */
/*----------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    		    */
/* _ (Encoder_State_Fx) st_fx : encoder state structure                             */
/* _ (Word16)   vad	          : vad flag		    	                       Q0   */
/* _ (Word16[]) speech_fx     : Pointer to the speech frame               qSpeech   */
/* _ (Word16)   qSpeech       : speech buffer qformat value                         */
/* _ (Word16*)  qener	      : frame_ener/lt_ener_voiced/lt_ener_noise buf qformat */
/*----------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :															    */
/* _ (Word16*)  qener	      : frame_ener/lt_ener_voiced/lt_ener_noise buf qformat */
/* _ (Encoder_State_Fx) st_fx : encoder state structure                             */
/*----------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :															    */
/* _ None																		    */
/*==================================================================================*/
void dtx_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    const Word16 vad,						/* i  : vad flag                                 */
    const Word16 speech[],				/* i  : Pointer to the speech frame              */
    Word16 Q_speech					/* i  : Q factor for speech		                 */

)
{
    Word16 alpha,j,i,Q_speech2;
    Word32 L_tmp;

    /* Initialization */
    IF( st_fx->ini_frame_fx == 0 )
    {
        st_fx->active_fr_cnt_fx = CNG_TYPE_HO;
        move16();

        st_fx->cng_type_fx = FD_CNG;
        move16();
        test();
        if( sub( st_fx->codec_mode, MODE1 ) == 0 || st_fx->Opt_AMR_WB_fx )
        {
            st_fx->cng_type_fx = LP_CNG;
            move16();
        }
    }

    /*------------------------------------------------------------------------*
     * Select SID or FRAME_NO_DATA frame if DTX is enabled
     *------------------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    IF( st_fx->Opt_DTX_ON_fx && vad == 0 &&
        sub(st_fx->ini_frame_fx,2) > 0  &&                     /* CNG coding starts after 2 frames */
        ( L_sub(st_fx->total_brate_fx,ACELP_24k40) <= 0 || sub(st_fx->lp_noise_fx, 1280) < 0) &&
        st_fx->fd_cng_reset_flag == 0 )
    {
        /* reset counter */
        st_fx->active_fr_cnt_fx = 0;
        move16();

        IF( st_fx->cnt_SID_fx == 0 )
        {
            /* this will be a SID frame */
            IF ( st_fx->Opt_AMR_WB_fx )
            {
                st_fx->core_brate_fx = SID_1k75;
                move32();
            }
            ELSE
            {
                st_fx->core_brate_fx = SID_2k40;
                move32();
            }
        }
        ELSE
        {
            /* this will be a no data frame */
            st_fx->core_brate_fx = FRAME_NO_DATA;
            move32();
        }

        test();
        test();
        IF( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 && sub(st_fx->last_core_fx,ACELP_CORE) != 0 && !st_fx->Opt_AMR_WB_fx )
        {
            /* force SID frame when switching from HQ core or AMR-WB IO mode into inactive frame in ACELP core when DTX is on */
            st_fx->core_brate_fx = SID_2k40;
            move32();
        }

        test();
        IF( sub(st_fx->cng_type_fx,FD_CNG) == 0 && L_sub(st_fx->total_brate_fx,ACELP_24k40) <= 0 )     /* at highest bit-rates, use exclusively LP_CNG */
        {
            test();
            test();
            IF ( L_sub(st_fx->total_brate_fx,ACELP_9k60) == 0 || L_sub(st_fx->total_brate_fx,ACELP_16k40) == 0 || L_sub(st_fx->total_brate_fx,ACELP_24k40) == 0 )
            {
                st_fx->codec_mode = MODE2;
                move16();
            }
        }
        ELSE
        {
            st_fx->cng_type_fx = LP_CNG;
            move16();
            IF ( st_fx->codec_mode == MODE2 )
            {
                st_fx->lp_cng_mode2 = 1;
                move16();
            }
            st_fx->codec_mode = MODE1;
            move16();
        }


    }

    /*------------------------------------------------------------------------*
     * Reset counters when in active frame (not in SID or FRAME_NO_DATA frame)
     *------------------------------------------------------------------------*/
    /* NB core bit rate can be "-1"  at startup , so one can not use   core_brate_fx <=2400 */
    test();
    test();
    IF ( (L_sub(st_fx->core_brate_fx ,SID_2k40) != 0 ) && (L_sub(st_fx->core_brate_fx, SID_1k75) != 0 ) && (st_fx->core_brate_fx != 0))
    {
        st_fx->cnt_SID_fx = 0;
        move16();

        /* change SID update rate */
        /* first SID update is only 8 (3) frames after the active speech end */
        IF( !st_fx->Opt_AMR_WB_fx )
        {
            st_fx->max_SID_fx = FIXED_SID_RATE;
            move16();
        }
        ELSE
        {
            st_fx->max_SID_fx = 3;
            move16();             /* first SID update is only 3 frames after the active speech end */
        }

        IF ( sub(st_fx->interval_SID_fx,st_fx->max_SID_fx) < 0 )
        {
            st_fx->max_SID_fx = st_fx->interval_SID_fx;
            move16();/* change SID update rate */
        }

        st_fx->cng_cnt_fx = 0;
        move16();                      /* reset the counter of CNG frames for averaging */

        test();
        IF( sub(st_fx->active_fr_cnt_fx,CNG_TYPE_HO) >= 0 && st_fx->Opt_AMR_WB_fx == 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF( sub(st_fx->cng_type_fx,LP_CNG) == 0 && ( (sub(st_fx->input_bwidth_fx,NB) == 0 && L_sub(st_fx->bckr_tilt_lt,FL2WORD32_SCALE(9.f,15)) > 0) || (sub(st_fx->input_bwidth_fx,NB) > 0 && L_sub(st_fx->bckr_tilt_lt,FL2WORD32_SCALE(45.f,15)) > 0) ) )
            {
                st_fx->cng_type_fx = FD_CNG;
                move16();
            }
            ELSE IF( sub(st_fx->cng_type_fx,FD_CNG) == 0 && ( (sub(st_fx->input_bwidth_fx,NB) == 0 && L_sub(st_fx->bckr_tilt_lt,FL2WORD32_SCALE(2.f,15)) < 0) || (sub(st_fx->input_bwidth_fx,NB) > 0 && L_sub(st_fx->bckr_tilt_lt,FL2WORD32_SCALE(10.f,15)) < 0) ) )
            {
                st_fx->cng_type_fx = LP_CNG;
                move16();
            }
        }
        ELSE IF( st_fx->Opt_AMR_WB_fx )
        {
            st_fx->cng_type_fx = LP_CNG;
            move16();
        }
        st_fx->active_fr_cnt_fx = add( st_fx->active_fr_cnt_fx, 1 );
        st_fx->active_fr_cnt_fx = s_min(st_fx->active_fr_cnt_fx, 200);
    }

    /*------------------------------------------------------------------------*
     * Update speech and background noise long-term energy
     *------------------------------------------------------------------------*/

    st_fx->frame_ener_fx = L_deposit_l(0);

    IF ( st_fx->Opt_DTX_ON_fx )
    {
        Q_speech2 = add(shl(Q_speech, 1), 7);
        FOR (j=0; j<16; j++)
        {
            L_tmp = L_mult0(*speech, *speech);
            speech++;
            FOR (i = 1; i< L_FRAME/16; i++)
            {
                L_tmp = L_mac0(L_tmp, *speech, *speech);
                speech++;
            }
            st_fx->frame_ener_fx = L_add(st_fx->frame_ener_fx, L_shr(L_tmp, Q_speech2));/*Q(-7) */
        }

        /* Active speech (voiced) */

        IF ( sub(st_fx->clas_fx,VOICED_CLAS) == 0 )
        {
            alpha = ALPHA_ENER_SLOW_FX;
            move16();
            if ( L_sub(st_fx->frame_ener_fx,st_fx->lt_ener_voiced_fx) > 0 )
            {
                alpha = ALPHA_ENER_FAST_FX;
                move16();/*Q15 */
            }

            /*st_fx->lt_ener_voiced_fx = alpha * st_fx->lt_ener_voiced_fx + (1.0f-alpha) * st_fx->frame_ener_fx;*/
            L_tmp = L_sub(st_fx->lt_ener_voiced_fx, st_fx->frame_ener_fx);
            L_tmp = Mult_32_16(L_tmp, alpha);
            st_fx->lt_ener_voiced_fx = L_add(L_tmp, st_fx->frame_ener_fx);	 		/*Q(-7)                   */

            st_fx->VarDTX_cnt_voiced_fx = add(st_fx->VarDTX_cnt_voiced_fx,1);

            st_fx->VarDTX_cnt_voiced_fx = s_min(st_fx->VarDTX_cnt_voiced_fx, MIN_CNT);
        }
        /* Background noise */
        ELSE IF( !st_fx->Opt_AMR_WB_fx )
        {
            alpha = ALPHA_ENER_SLOW_FX;
            move16();
            if (L_sub(st_fx->frame_ener_fx,st_fx->lt_ener_noise_fx) < 0)
            {
                alpha = ALPHA_ENER_FAST_FX;
                move16();
            }

            /*st_fx->lt_ener_noise_fx = alpha * st_fx->lt_ener_noise_fx + (1.0f-alpha) * st_fx->frame_ener_fx;*/
            L_tmp = L_sub(st_fx->lt_ener_noise_fx, st_fx->frame_ener_fx);
            L_tmp = Mult_32_16(L_tmp, alpha);
            st_fx->lt_ener_noise_fx = L_add(L_tmp, st_fx->frame_ener_fx);
            move32();/*Q(-7)                */

            st_fx->VarDTX_cnt_noise_fx = add(st_fx->VarDTX_cnt_noise_fx,1);

            st_fx->VarDTX_cnt_noise_fx = s_min(st_fx->VarDTX_cnt_noise_fx, MIN_CNT);
        }
    }

    /* Update of the SID counter */
    update_SID_cnt( st_fx );

    /* Update encoded bandwidth */
    test();
    test();
    IF(  st_fx->Opt_DTX_ON_fx && (st_fx->core_brate_fx == SID_2k40 || st_fx->core_brate_fx == FRAME_NO_DATA ) )
    {

        st_fx->bwidth_fx = st_fx->last_bwidth_fx;
        move16();

        test();
        test();
        IF( st_fx->Opt_RF_ON && (L_sub(st_fx->total_brate_fx, ACELP_13k20) == 0) && (sub(st_fx->bwidth_fx, NB) == 0))
        {
            st_fx->codec_mode = MODE1;
            move16();
            reset_rf_indices(st_fx);
            st_fx->Opt_RF_ON = 0;
            move16();
            st_fx->rf_mode = 0;
            move16();
        }

    }

    return;
}

/*---------------------------------------------------------------------*
 * update_SID_cnt()
 *
 * Update of the SID counter
 *---------------------------------------------------------------------*/

static void update_SID_cnt(
    Encoder_State_fx *st_fx       /* i/o: State structure                     */
)
{
    Word16 EstimatedSNR, delta, frac, exp;
    Word32 L_tmp1, L_tmp2;

    test();
    test();
    IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || L_sub(st_fx->core_brate_fx, SID_1k75) == 0 || st_fx->core_brate_fx == FRAME_NO_DATA )
    {
        /* Adapt the SID interval */
        test();
        test();
        IF ( st_fx->var_SID_rate_flag_fx != 0 && sub(st_fx->VarDTX_cnt_voiced_fx, MIN_CNT) == 0 && sub(st_fx->VarDTX_cnt_noise_fx, MIN_CNT) == 0 )
        {
            /* EstimatedSNR = 10.0f * (float)log10( (0.01f + st_fx->lt_ener_voiced) / (0.01f + st_fx->lt_ener_noise) ); */

            L_tmp1 = L_max(st_fx->lt_ener_voiced_fx, 1);
            exp = norm_l(L_tmp1);
            frac = Log2_norm_lc(L_shl(L_tmp1, exp));
            exp = sub(30, exp);
            L_tmp1 = L_Comp(exp, frac);
            L_tmp2 = L_max(st_fx->lt_ener_noise_fx, 1);
            exp = norm_l(L_tmp2);
            frac = Log2_norm_lc(L_shl(L_tmp2, exp));
            exp = sub(30, exp);
            L_tmp1 = L_sub(L_tmp1, L_Comp(exp, frac));
            /* 10 x Log10(a/b) = 10 x Log10(2) x [Log2(a) - Log2(b)] */
            /* 10 x Log10(2) = ~3.0103 */
            L_tmp1 = Mpy_32_16_1(L_tmp1, 24660);    /* mult by 3.0103 / 4 in Q15 */
            L_tmp1 = L_shl(L_tmp1, 2+8);      /* mult by 4 and shift left 8 to go in Q24 */
            EstimatedSNR = round_fx(L_tmp1);  /* now in Q8 */
            IF ( sub(EstimatedSNR,SNR_H_FX) > 0 )
            {
                st_fx->interval_SID_fx = INT_H;
                move16();
            }
            ELSE IF ( sub(EstimatedSNR,SNR_L_FX) < 0 )
            {
                st_fx->interval_SID_fx = INT_L;
                move16();
            }
            ELSE
            {
                st_fx->interval_SID_fx = extract_h(L_mac(INT_L*65536L-SNR_L_FX/256*65536L*RATIO,(32768/256)*RATIO, EstimatedSNR));
            }
            test();

            if( st_fx->Opt_AMR_WB_fx == 0 || sub(st_fx->max_SID_fx,3) != 0 )
            {
                st_fx->max_SID_fx = st_fx->interval_SID_fx;
                move16();    /* change SID update rate */
            }
        }
        test();
        IF( st_fx->Opt_DTX_ON_fx != 0 && st_fx->cnt_SID_fx != 0 )
        {
            L_tmp1 = L_max(st_fx->lt_ener_noise_fx, 1);
            exp = norm_l(L_tmp1);
            frac = Log2_norm_lc(L_shl(L_tmp1, exp));
            exp = sub(30, exp);
            L_tmp1 = L_Comp(exp, frac);
            L_tmp2 = L_max(st_fx->lt_ener_last_SID_fx, 1);
            exp = norm_l(L_tmp2);
            frac = Log2_norm_lc(L_shl(L_tmp2, exp));
            exp = sub(30, exp);
            L_tmp1 = L_sub(L_tmp1, L_Comp(exp, frac));
            /* 10 x Log10(a/b) = 10 x Log10(2) x [Log2(a) - Log2(b)] */
            /* 10 x Log10(2) = ~3.0103 */
            L_tmp1 = Mpy_32_16_1(L_tmp1, 24660);    /* mult by 3.0103 / 4 in Q15 */
            L_tmp1 = L_shl(L_tmp1, 2+8);      /* mult by 4 and shift left 8 to go in Q24 */
            delta = round_fx(L_tmp1);        /* now in Q8 */
            test();
            test();
            if ( sub(delta,LTE_VAR_FX) < 0 && sub(st_fx->VarDTX_cnt_voiced_fx,MIN_CNT) == 0 && sub(st_fx->VarDTX_cnt_noise_fx, MIN_CNT) == 0 )
            {
                /* Send SID frame, and reset lt_ener_noise */
                st_fx->lt_ener_noise_fx = st_fx->frame_ener_fx;
                move32();
            }
        }
        ELSE
        {
            /* If SID frame was sent, update long-term energy */
            st_fx->lt_ener_last_SID_fx = st_fx->lt_ener_noise_fx;
            move32();
        }
        st_fx->cnt_SID_fx = add(st_fx->cnt_SID_fx,1);

        IF( st_fx->var_SID_rate_flag_fx )
        {
            test();
            test();
            test();

            if( st_fx->Opt_AMR_WB_fx != 0 && sub(st_fx->max_SID_fx,3) == 0 && sub(st_fx->cnt_SID_fx,3) == 0 )
            {
                /* set the size of CNG history buffer for averaging to DTX_HIST_SIZE frames */
                /* be sure that DTX_HIST_SIZE >= INT_L */
                st_fx->cng_hist_size_fx = 3;
                move16();
            }
            test();
            /*else if ( st_fx->max_SID_fx != 3 && st_fx->cnt_SID_fx == DTX_HIST_SIZE )//compile error */
            if( sub(st_fx->max_SID_fx,3) != 0 && sub(st_fx->cnt_SID_fx,DTX_HIST_SIZE) == 0 )
            {
                /* set the size of CNG history buffer for averaging to 3 frames */
                st_fx->cng_hist_size_fx = DTX_HIST_SIZE;
                move16();
            }
        }
        test();
        IF( st_fx->var_SID_rate_flag_fx == 0 && sub(st_fx->interval_SID_fx,1) > 0 )
        {
            /* set the size of CNG history buffer for averaging to interval_SID frames */
            st_fx->cng_hist_size_fx = st_fx->interval_SID_fx;
            move16();
            if ( sub(st_fx->cng_hist_size_fx, DTX_HIST_SIZE) > 0 )
            {
                st_fx->cng_hist_size_fx = DTX_HIST_SIZE;
                move16();
            }
        }
        IF( sub(st_fx->cnt_SID_fx,st_fx->max_SID_fx) >= 0 )
        {
            /* adaptive SID update interval */
            st_fx->max_SID_fx = st_fx->interval_SID_fx;
            move16();
            st_fx->cnt_SID_fx = 0;
            move16();
        }
    }

    return;
}


void dtx_hangover_control_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    const Word16 lsp_new_fx[M]          /* i  : current frame LSPs                       */
)
{
    Word16 ptr;
    Word16 i,j,m;
    Word16 tmp_lsp[max(DTX_HIST_SIZE,HO_HIST_SIZE)*M];
    Word32 tmp_enr[max(DTX_HIST_SIZE,HO_HIST_SIZE)];
    Word16 tmp[max(DTX_HIST_SIZE,HO_HIST_SIZE)*M];
    Word16 enr_new;
    Word16 weights;
    Word32 enr_est;
    Word16 enr_est_log;
    Word16 lsp_est[M];
    Word16 Dlsp,Denr;
    Word16 lsf_tmp[M];
    Word32 C[M];
    Word32 max[2];
    Word16 max_idx[2];
    Word16 ftmp_fx;
    Word16 Dlsp_n2e,Denr_n2e;
    Word16 exp,fra,exp2,fra2;
    Word16 S_max;
    Word16 S_tmp;
    Word32 L_tmp;


    /* get current frame exc energy in log2 */
    exp = norm_l(st_fx->ho_ener_circ_fx[st_fx->ho_circ_ptr_fx]);
    fra = Log2_norm_lc(L_shl(st_fx->ho_ener_circ_fx[st_fx->ho_circ_ptr_fx],exp));
    exp = sub(sub(30,exp),6);
    L_tmp = L_Comp(exp,fra);
    enr_new = round_fx(L_shl(L_tmp,8)); /*Q8 */

    if ( enr_new < 0 )
    {
        enr_new = 0;
        move16(); /*Q8 */
    }

    /* get energies and lsps of hangover frames  */
    ptr = sub(st_fx->ho_circ_ptr_fx,sub(st_fx->burst_ho_cnt_fx,1));
    IF ( ptr < 0 )
    {
        ptr = add(st_fx->ho_circ_size_fx,ptr);
    }

    FOR ( i=0; i<st_fx->burst_ho_cnt_fx-1; i++ )
    {
        Copy( &(st_fx->ho_lsp_circ_fx[ptr*M]), &(tmp_lsp[i*M]), M );
        tmp_enr[i] = st_fx->ho_ener_circ_fx[ptr];
        move32();/*Q6 */

        ptr = add(ptr,1);
        if ( sub(ptr,st_fx->ho_circ_size_fx) == 0 )
        {
            ptr = 0;
            move16();
        }
    }

    /* get estimated CNG energy and lsps assuming terminate hangover at current frame */
    ptr = sub(st_fx->burst_ho_cnt_fx,2);
    enr_est = Mpy_32_16_1(tmp_enr[ptr],W_DTX_HO_FX[0]); /*Q6 */
    weights = W_DTX_HO_FX[0];
    move16();/*Q15 */
    Copy( &(tmp_lsp[ptr*M]), tmp, M );
    m = 1;
    move16();

    FOR ( i=1; i<st_fx->burst_ho_cnt_fx-2; i++ )
    {
        test();
        IF ( L_sub(Mpy_32_16_1(tmp_enr[ptr-i],ONE_OVER_BUF_H_NRG_FX),tmp_enr[ptr]) < 0 &&
             L_sub(tmp_enr[ptr-i],Mpy_32_16_1(tmp_enr[ptr], BUF_L_NRG_FX)) > 0 )
        {
            enr_est = L_add(enr_est,Mpy_32_16_1(tmp_enr[ptr-i],W_DTX_HO_FX[i])); /*Q6 */
            weights = add(weights,W_DTX_HO_FX[i]); /*Q15 */
            Copy( &tmp_lsp[(ptr-i)*M], &tmp[m*M], M );
            m = add(m,1);
        }
    }

    exp = norm_l(enr_est);
    fra = round_fx(L_shl(enr_est,exp));
    exp2 = norm_s(weights);
    fra2 = shl(weights,exp2);
    exp = sub(sub(exp,16),exp2);
    IF ( sub(fra,fra2) > 0 )
    {
        fra = shr(fra,1);
        exp = sub(exp,1);
    }
    L_tmp = L_deposit_l(div_s(fra,fra2));
    enr_est = L_shr(L_tmp,exp); /*Q6 */

    if ( L_sub(enr_est,64) < 0 )
    {
        enr_est = 64;
        move16();/*Q6 */
    }

    exp = norm_l(enr_est);
    fra = Log2_norm_lc(L_shl(enr_est,exp));
    exp = sub(sub(30,exp),6);
    L_tmp = L_Comp(exp,fra);
    enr_est_log = round_fx(L_shl(L_tmp,8)); /*Q8 */
    Denr_n2e = abs_s(sub(enr_new,enr_est_log)); /*Q8 */

    IF ( sub(m,3) < 0 )
    {
        enr_est = L_add(Mpy_32_16_1(enr_est,26214),Mpy_32_16_1(st_fx->ho_ener_circ_fx[st_fx->ho_circ_ptr_fx],6554)); /*Q6 */
    }
    ELSE
    {
        enr_est = L_add(Mpy_32_16_1(enr_est,31130),Mpy_32_16_1(st_fx->ho_ener_circ_fx[st_fx->ho_circ_ptr_fx],1638)); /*Q6 */
    }

    exp = norm_l(enr_est);
    fra = Log2_norm_lc(L_shl(enr_est,exp));
    exp = sub(sub(30,exp),6);
    L_tmp = L_Comp(exp,fra);
    enr_est_log = round_fx(L_shl(L_tmp,8)); /*Q8 */

    if ( enr_est_log < 0 )
    {
        enr_est_log = 0;
        move16();
    }

    set32_fx( max, 0, 2 );
    set16_fx( max_idx, 0, 2 );

    FOR( i=0; i<m; i++ )
    {
        IF ( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
        {
            lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_FX );
            ftmp_fx = 964;
            move16();/*QX2.56  */
            S_tmp = sub(16384,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
            C[i] = L_mult0(S_tmp,S_tmp); /*QX6.5536 */
        }
        ELSE
        {
            lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_16k );
            ftmp_fx = 1205;
            move16();/*QX2.56 */
            S_tmp = sub(20480,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
            C[i] = L_mult0(S_tmp,S_tmp); /*QX6.5536 */
        }

        S_tmp = sub(lsf_tmp[0],ftmp_fx); /*QX2.56 */
        C[i] = L_mac0(C[i],S_tmp,S_tmp); /*QX6.5536       */
        FOR ( j=0; j<M-1; j++ )
        {
            S_tmp = sub(sub(lsf_tmp[j+1],lsf_tmp[j]),ftmp_fx); /*QX2.56 */
            C[i] = L_mac0(C[i],S_tmp,S_tmp); /*QX6.5536 */
        }

        C[i] = Mpy_32_16_1(C[i],1928); /*QX6.5536 */

        IF ( L_sub(C[i],max[0]) > 0 )
        {
            max[1] = max[0];
            move16();
            max_idx[1] = max_idx[0];
            move16();
            max[0] = C[i];
            move16();
            max_idx[0] = i;
            move16();
        }
        ELSE IF ( L_sub(C[i],max[1]) > 0 )
        {
            max[1] = C[i];
            move16();
            max_idx[1] = i;
            move16();
        }
    }

    IF ( sub(m,1) == 0 )
    {
        Copy( tmp, lsp_est, M );
    }
    ELSE IF ( sub(m,4) < 0 )
    {
        FOR ( i=0; i<M; i++ )
        {
            lsp_est[i] = 0;
            move16(); /*Q15 */
            FOR ( j=0; j<m; j++ )
            {
                lsp_est[i] = add(lsp_est[i],tmp[j*M+i]); /*Q15 */
            }

            lsp_est[i] = sub(lsp_est[i],tmp[max_idx[0]*M+i]); /*Q15 */
            S_tmp = div_s(1,sub(m,1)); /*Q15 */
            lsp_est[i] = mult_r(lsp_est[i],S_tmp); /*Q15 */
        }
    }
    ELSE
    {
        FOR ( i=0; i<M; i++ )
        {
            lsp_est[i] = 0;
            move16();/*Q15 */
            FOR ( j=0; j<m; j++ )
            {
                lsp_est[i] = add(lsp_est[i],tmp[j*M+i]); /*Q15 */
            }

            lsp_est[i] = sub(lsp_est[i],add(tmp[max_idx[0]*M+i],tmp[max_idx[1]*M+i])); /*Q15 */
            S_tmp = div_s(1,sub(m,2)); /*Q15 */
            lsp_est[i] = mult_r(lsp_est[i],S_tmp); /*Q15 */
        }
    }

    Dlsp_n2e = 0;
    move16(); /*Q15 */
    FOR ( i=0; i<M; i++ )
    {
        Dlsp_n2e = add(Dlsp_n2e,abs_s(sub(lsp_new_fx[i],lsp_est[i]))); /*Q15 */
        lsp_est[i] = add(mult_r(26214,lsp_est[i]),mult_r(6554,lsp_new_fx[i])); /*Q15 */
    }

    /* get deviation of CNG parameters between newly estimated and current state memory */
    Dlsp = 0;
    move16();
    S_max = 0;
    move16();

    FOR ( i=0; i<M; i++ )
    {
        S_tmp = abs_s(sub(st_fx->lspCNG_fx[i],lsp_est[i])); /*Q15 */
        Dlsp = add(Dlsp,S_tmp); /*Q15 */
        IF ( sub(S_tmp,S_max) > 0 )
        {
            S_max = S_tmp; /*Q15 */
        }
    }

    exp = norm_l(st_fx->lp_ener_fx);
    fra = Log2_norm_lc(L_shl(st_fx->lp_ener_fx,exp));
    exp = sub(sub(30,exp),6);
    L_tmp = L_Comp(exp,fra);
    S_tmp = round_fx(L_shl(L_tmp,8)); /*Q8 */
    Denr = abs_s(sub(S_tmp,enr_est_log)); /*Q8 */

    /* make decision if DTX hangover can be terminated */
    st_fx->hangover_terminate_flag_fx = 0;
    move16(); /*Q0 */

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
    IF ( ( sub(Dlsp,13107) < 0 && sub(Denr,359) < 0 && sub(S_max,3277) < 0
           && sub(Dlsp_n2e,13107) < 0 && sub(Denr_n2e,308) < 0 && st_fx->Opt_SC_VBR_fx == 1 ) ||
         ( sub(Dlsp,13107) < 0 && sub(Denr,205) < 0 && sub(S_max,3277) < 0
           && sub(Dlsp_n2e,13107) < 0 && sub(Denr_n2e,205) < 0 && st_fx->Opt_SC_VBR_fx == 0 ) )

    {
        st_fx->hangover_terminate_flag_fx = 1;
        move16(); /*Q0   */
    }

    return;
}
