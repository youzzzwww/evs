/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

static Word16 VDQ_vec_fx( Word16 *Qvec_out_fx, const Word16 *mean_dic_fx, const Word16 *dic_fx,
                          const Word16 index_fx, const Word16 vec_en_fx );

/*========================================================================*/
/* FUNCTION : void Comp_and_apply_gain_enc_fx						      */
/*------------------------------------------------------------------------*/
/* PURPOSE :  Compute and apply the quantized per band gain               */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16[]) Ener_per_bd_iQ     : Target ener per band             Q12 */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _ (Word16[]) exc_diffQ      : Quantized excitation     Qexc		      */
/* _ (Word16[]) Ener_per_bd_yQ : Ener per band for norm vectori->Q12/o->Q2*/
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ None                                                            	  */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void Comp_and_apply_gain_fx(
    Word16 exc_diffQ[],       /* i/o: Quantized excitation                  */
    Word16 Ener_per_bd_iQ[],  /* i  : Target ener per band              Q13 */
    Word16 Ener_per_bd_yQ[],  /* i/o  : Ener per band for norm vector     i->Q13/o->Q13 */
    Word16 Mbands_gn,         /* i  : number of bands                  */
    const Word16 ReUseGain,         /* i  : Reuse the gain in Ener_per_bd_yQ  */
    Word16 Qexc_diff,
    Word16 Q_exc
)
{
    Word16 i, i_band;
    Word16 StartBin, NB_Qbins;
    Word16 y_gain;
    Word16 L16, frac, exp1, tmp_exp;
    Word32 L32;

    /* Recreate excitation for local synthesis and decoder */
    StartBin = 0;
    move16();
    NB_Qbins  = 0;
    move16();

    tmp_exp = add(14,sub(Q_exc,Qexc_diff));  /* In case of reuse, it can be computed outside the loop*/
    FOR( i_band = 0; i_band < Mbands_gn; i_band++ )
    {
        StartBin = add(StartBin, NB_Qbins);
        NB_Qbins = mfreq_bindiv_loc[i_band];
        move16();
        IF( sub(ReUseGain,1) == 0 )
        {
            y_gain = Ener_per_bd_yQ[i_band];
            move16();

            FOR(i = StartBin ; i < NB_Qbins + StartBin ; i++)
            {
                L32 = L_shl(L_mult(exc_diffQ[i], y_gain),tmp_exp); /*Q_exc+16 */
                exc_diffQ[i] = round_fx(L32);/*Q_exc */
            }
        }
        ELSE
        {
            /*-----------------------------------------------------------------*
             * y_gain  = pow(10.0, (Ener_per_bd_iQ[i_band]-Ener_per_bd_yQ[i_band]))
             * = pow(2, 3.321928*(Ener_per_bd_iQ[i_band]-Ener_per_bd_yQ[i_band]))
             *-----------------------------------------------------------------*/
            L16 = sub(Ener_per_bd_iQ[i_band], Ener_per_bd_yQ[i_band]);/*Q12 */
            L32 = L_mult(L16, 27213); /* 3.321928 in Q13 -> Q26 */
            L32 = L_shr(L32, 10);   /* From Q26 to Q16 */
            frac = L_Extract_lc(L32, &exp1); /* Extract exponent of gcode0 */
            y_gain = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */
            Ener_per_bd_yQ[i_band] = shl(y_gain, sub(exp1, 13));
            move16();/*Q1     */
            tmp_exp = add(add(exp1,1),sub(Q_exc,Qexc_diff));

            FOR(i = StartBin ; i < NB_Qbins + StartBin ; i++)
            {
                L32 = L_mult(exc_diffQ[i], y_gain); /*Qexc_diff+15 */
                exc_diffQ[i] = round_fx(L_shl(L32,tmp_exp)); /*Q_exc */
            }
        }
    }

    return;
}


/*========================================================================*/
/* FUNCTION : Ener_per_band_comp_fx()									  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Compute the energy per band in log domain for quantization   */
/*			 purposes.													  */
/*           Loops are decomposed to accomodate the PVQ quantization	  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16*) edct_table_128_fx  : edct table                       Q15  */
/* _ (Word16*) Q_exc_diff         : input format of exc_diff              */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ (Word16[]) y_gain4 : Energy per band to quantize      Q12			  */
/* _ (Word32*) etmp14	: Energy band 14				   Q_exc_diff*2+1 */
/* _ (Word32*) etmp15	: Energy band 15				   Q_exc_diff*2+1 */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
static Word16 Comp_band_log_ener(    /* o  : Band gain Q12 */
    const Word16 *pt_fx,        /* i  : Dct input Q_sc */
    const Word16 Len,     /* i  : Lenght en energy accumulation */
    const Word16 Q_sc,    /* i  : scaling of input    */
    const Word16 E_sc     /* i  : Additional scaling factor for energy */
)
{
    Word32 L_tmp;
    Word16 e_tmp, f_tmp, tmp16, ener_exp;

    /*for(i = 0; i < 8; i++){etmp += (*pt * *pt);pt++;}*/
    L_tmp = Calc_Energy_Autoscaled(pt_fx, Q_sc, Len, &ener_exp);

    /*y_gain4[j] = (float)log10(sqrt(etmp<<E_sc));*/
    e_tmp = norm_l(L_tmp);
    f_tmp = Log2_norm_lc(L_shl(L_tmp, e_tmp));
    e_tmp = sub(sub(add(30,E_sc),e_tmp),ener_exp);
    L_tmp = Mpy_32_16(e_tmp, f_tmp, 19728); /* Q16 */ /*log10(2) in Q17 */
    tmp16 = round_fx(L_shl(L_tmp, 12-2)); /* Q12 -1 is to compensate Q17 */
    return tmp16;
}

void Ener_per_band_comp_fx(
    const Word16 exc_diff_fx[],    /* i  : target signal                     Q_exc_diff     */
    Word16 y_gain4_fx[],     /* o  : Energy per band to quantize		 Q12            */
    const Word16 Q_exc,            /* i  : frame length                      */
    const Word16 Mband,            /* i  : Max band                          */
    const Word16 Eflag             /* i  : flag of highest band              */
)
{
    const Word16 *pt_fx;
    Word16 j;

    pt_fx  = exc_diff_fx;
    FOR(j = 0; j < 2; j++)
    {
        y_gain4_fx[j] = Comp_band_log_ener(pt_fx, 8, Q_exc, 1);
        move16();
        pt_fx += 8;
    }

    FOR(j = 1; j < Mband-2; j++)
    {
        y_gain4_fx[j+1] = Comp_band_log_ener(pt_fx, 16, Q_exc, 0);
        move16();
        pt_fx += 16;
    }

    IF( sub(Eflag,1) == 0 )
    {
        y_gain4_fx[j+1] = Comp_band_log_ener(pt_fx, 32, Q_exc, -1);
        move16();
        pt_fx += 32;
    }

    return;
}

/*-------------------------------------------------------------------*
 * gsc_gainQ()
 *
 * Quantization of the energy per band
 *-------------------------------------------------------------------*/
static void GSC_gain_adj(
    const Word16 coder_type,      /* i  : Coder type        */
    const Word32 core_brate,      /* i  : Bit rate          */
    const Word16 mean_g,          /* i  : Average gain Q12  */
    Word16 *old_y_gain,     /* i/o: Previous frame dequantized vector */
    const Word16 *y_gain_tmp,     /* i  : Dequantized gains */
    Word16 *y_gainQ         /* i/o: Output gains Q12  */
)
{
    /* Gain adjustment to fit ACELP generic inactive coding gain at low rate */
    Word16 Gain_off, i;

    IF( sub(coder_type,INACTIVE) != 0 )
    {
        FOR( i = 0; i < MBANDS_GN; i++ )
        {
            old_y_gain[i] =  y_gain_tmp[i];
            move16();
            y_gainQ[i] = add(y_gain_tmp[i], mean_g);
            move16();
        }
    }
    ELSE
    {
        Gain_off = 0;
        move16();
        IF(L_sub(core_brate,ACELP_7k20) <= 0 )
        {
            Gain_off = 32767;
            move16();   /* 8 -> Q12 */
        }
        ELSE IF (L_sub(core_brate,ACELP_8k00) <= 0)
        {
            Gain_off = 27034;
            move16();   /* 6.6f -> Q12 */
        }
        ELSE IF (L_sub(core_brate,ACELP_9k60) <= 0)
        {
            Gain_off = 19661;
            move16();   /*4.8f-> Q12 */
        }
        ELSE IF (L_sub(core_brate,ACELP_11k60) <= 0)
        {
            Gain_off = 14336;
            move16();   /* 3.5f -> Q12 */
        }
        ELSE IF (L_sub(core_brate,ACELP_13k20) <= 0)
        {
            Gain_off = 12288;
            move16(); /* 3.0f -> Q12 dB */
        }

        /*mimic ACELP decay of energy for low rates*/
        FOR( i = 0; i < MBANDS_GN; i++ )
        {
            old_y_gain[i] =  y_gain_tmp[i];
            move16();
            /*y_gainQ[i] = y_gain_tmp[i]+mean_4g[0]-(i*(Gain_off/20.f)/((float) Mbands_gn));*/
            y_gainQ[i] = add(y_gain_tmp[i], sub(mean_g, i_mult2(i, mult_r(Gain_off, 102))));
            move16();
        }
    }

    return;
}
/*==========================================================================*/
/* FUNCTION : Word16 gsc_gaindec_fx	()							            */
/*--------------------------------------------------------------------------*/
/* PURPOSE  :  Generic signal frequency band decoding and application       */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word16) pvq_bits_fx 	   : core used					Q0			    */
/* _ (Word16) coder_type_fx    : coding type				Q0			    */
/* _ (Word16) core_fx          : core used                  Q0              */
/* _ (Word16) bwidth_fx        : input signal bandwidth     Q0              */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ (Word16[]) y_gainQ_fx	   : quantized gain per band                    */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/* _ (Word16[]) old_y_gain_fx  : AR gain quantizer for low rate          	*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _ (Word16) :					average frequency gain 					    */
/*==========================================================================*/

Word16 gsc_gaindec_fx(                  /* o  : average frequency gain    */
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure   */
    Word16 y_gainQ_fx[],        /* o  : quantized gain per band   */
    const Word32 core_brate_fx,       /* i  : core used                 */
    Word16 old_y_gain_fx[],     /* i/o: AR gain quantizer for low rate */
    const Word16 coder_type_fx,       /* i  : coding type               */
    const Word16 bwidth_fx            /* i  : input signal bandwidth    */
)
{
    Word16 idx_g_fx, i;
    Word16 mean_4g_fx;
    Word16 y_gain_tmp3_fx[MBANDS_GN];

    test();
    test();
    IF( (sub(coder_type_fx,AUDIO) == 0 || sub(coder_type_fx,INACTIVE) == 0) && sub(bwidth_fx,NB) == 0 )
    {
        idx_g_fx = (Word16) get_next_indice_fx( st_fx, 6 );
        VDQ_vec_fx(&mean_4g_fx, Gain_meanNB_fx, Gain_mean_dicNB_fx, idx_g_fx, 1 );

        idx_g_fx = (Word16) get_next_indice_fx( st_fx, 6 );
        move16();
        VDQ_vec_fx(y_gainQ_fx, Mean_dic_NB_fx, Gain_dic1_NB_fx, idx_g_fx, 3 );

        IF(L_sub(core_brate_fx,ACELP_9k60) < 0)
        {
            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );
            VDQ_vec_fx(y_gainQ_fx+3, Mean_dic_NB_fx+3, Gain_dic2_NB_fx, idx_g_fx, 3 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 4 );
            VDQ_vec_fx(y_gainQ_fx+6, Mean_dic_NB_fx+6, Gain_dic3_NB_fx, idx_g_fx, 4 );
        }
        ELSE
        {
            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 6 );
            VDQ_vec_fx(y_gainQ_fx+3, Mean_dic_NB_fx+3, Gain_dic2_NBHR_fx, idx_g_fx, 3 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 7 );
            VDQ_vec_fx(y_gainQ_fx+6, Mean_dic_NB_fx+6, Gain_dic3_NBHR_fx, idx_g_fx, 4 );
        }
        test();
        IF( L_sub(core_brate_fx,ACELP_9k60) <= 0 && sub(coder_type_fx,INACTIVE) == 0 )
        {
            /* Some energy is needed in high band for stat_noise_uv_enc
              to be functional in inactive speech */
            y_gainQ_fx[10] = mean_fx(y_gainQ_fx+6, 3);
            move16();
            y_gainQ_fx[11] = mean_fx(y_gainQ_fx+7, 3);
            move16();
            y_gainQ_fx[12] = mean_fx(y_gainQ_fx+8, 3);
            move16();
            y_gainQ_fx[13] = mean_fx(y_gainQ_fx+9, 3);
            move16();
            y_gainQ_fx[14] = mean_fx(y_gainQ_fx+10, 3);
            move16();
            y_gainQ_fx[15] = mean_fx(y_gainQ_fx+11, 3);
            move16();
        }
        ELSE
        {
            set16_fx( y_gainQ_fx + 10, 0, MBANDS_GN - 10 );
        }
    }
    ELSE
    {
        idx_g_fx = (Word16) get_next_indice_fx( st_fx, 6 );

        VDQ_vec_fx(&mean_4g_fx, mean_m_fx, mean_gain_dic_fx, idx_g_fx, 1 );

        IF(L_sub(core_brate_fx,ACELP_9k60) <= 0)
        {
            /*--------------------------------------------------------------------------------------*
             * UQ of the first 8 bands and half of the last 8 bands
             *--------------------------------------------------------------------------------------*/
            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );
            VDQ_vec_fx(y_gainQ_fx, YGain_mean_LR_fx, YGain_dic1_LR_fx, idx_g_fx, 3 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );
            VDQ_vec_fx(y_gainQ_fx+3, YGain_mean_LR_fx+3, YGain_dic2_LR_fx, idx_g_fx, 4 );

            /*----------------------------------------------------------------------*
             * Interpolation of the last 4 Q bands to create bands 8-16
             * And scaling
             *----------------------------------------------------------------------*/

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );

            VDQ_vec_fx(y_gainQ_fx+7, YGain_mean_LR_fx+7, YGain_dic3_LR_fx, idx_g_fx, 5 );

            Copy(y_gainQ_fx+8, y_gain_tmp3_fx, 4);
            set16_fx(y_gainQ_fx+12, 0, 4);

            fft_rel_fx(y_gainQ_fx+8, 4, 2);

            y_gainQ_fx[15] = y_gainQ_fx[11];
            move16();
            y_gainQ_fx[11] = 0;
            move16();
            ifft_rel_fx(y_gainQ_fx+8, 8, 3);
            FOR(i = 8; i < 16; i++)
            {
                /*y_gainQ_fx[i] *=  1.41f;*/
                y_gainQ_fx[i] =  round_fx(L_shl(L_mult(y_gainQ_fx[i] , 23101),1));/*Q12 */
            }
            /*----------------------------------------------------------------------*
             * Copy the true Q values in the specific bands
             *----------------------------------------------------------------------*/
            y_gainQ_fx[8] = y_gain_tmp3_fx[0];
            move16();
            y_gainQ_fx[10]= y_gain_tmp3_fx[1];
            move16();
            y_gainQ_fx[12]= y_gain_tmp3_fx[2];
            move16();
            y_gainQ_fx[14]= y_gain_tmp3_fx[3];
            move16();
        }
        ELSE
        {
            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 6 );
            VDQ_vec_fx(y_gainQ_fx, YG_mean16_fx, YG_dicMR_1_fx, idx_g_fx, 4 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );
            VDQ_vec_fx(y_gainQ_fx+4, YG_mean16_fx+4, YG_dicMR_2_fx, idx_g_fx, 4 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 5 );
            VDQ_vec_fx(y_gainQ_fx+8, YG_mean16_fx+8, YG_dicMR_3_fx, idx_g_fx, 4 );

            idx_g_fx = (Word16) get_next_indice_fx( st_fx, 4 );
            VDQ_vec_fx(y_gainQ_fx+12, YG_mean16_fx+12, YG_dicMR_4_fx, idx_g_fx, 4 );
        }
    }

    /* Gain adjustment to fit ACELP generic inactive coding gain at low rate */
    GSC_gain_adj(coder_type_fx, core_brate_fx, mean_4g_fx, old_y_gain_fx, y_gainQ_fx, y_gainQ_fx);

    return mean_4g_fx;

}

/*-------------------------------------------------------------------*
 * gsc_gainQ()
 *
 * Quantization of the energy per band
 *-------------------------------------------------------------------*/
Word16 gsc_gainQ_fx(  /*Q12*/
    Encoder_State_fx *st_fx,             /* i/o: decoder state structure          */
    const Word16 y_gain4[],         /* i  : Energy per band              Q12 */
    Word16 y_gainQ[],         /* o  : quantized energy per band    Q12 */
    const Word32 core_brate,        /* i  : Core rate                        */
    Word16 *old_y_gain,       /* i/o: AR mem for low rate ener Q   Q12 */
    const Word16 coder_type,        /* i  : coding type                      */
    const Word16 bwidth             /* i  : input signal bandwidth           */
)
{
    Word16 y_gain_tmp[MBANDS_GN], y_gain_tmp2[MBANDS_GN];
    Word16 i, idx_g = 0;
    Word16 mean_4g[1] = {0}, tmp16,tmp1, tmp2;
    Word16 Mbands_gn = MBANDS_GN;
    Word16 y_gain_tmp3[MBANDS_GN];
    Word16 cnt;
    Word32 L_tmp;

    mean_4g[0] = 0;

    test();
    test();
    IF( (sub(coder_type,AUDIO) == 0 || sub(coder_type,INACTIVE) == 0) && sub(bwidth,NB) == 0 )
    {

        /*ftmp1 =  mean(y_gain4, 10)-0.6f;*/
        L_tmp = L_deposit_l(0);
        FOR(cnt = 0 ; cnt < 10 ; cnt++)
        {
            L_tmp = L_mac(L_tmp,y_gain4[cnt], 3277);
        }
        tmp16 = sub(round_fx(L_tmp), 4915);

        FOR(i = 0; i < Mbands_gn; i++)
        {
            y_gain_tmp2[i] = y_gain4[i];
            move16();
            /*if(y_gain4[i] < ftmp1-0.6f)*/
            y_gain_tmp2[i] = s_max(y_gain_tmp2[i], tmp16);
            move16();
        }

        L_tmp = L_deposit_l(0);
        FOR(i = 0; i < 10; i++)
        {
            L_tmp = L_mac(L_tmp,y_gain_tmp2[i], 3277);
        }

        /* Quantized mean gain without clipping */
        mean_4g[0] = round_fx(L_tmp);
        idx_g = vquant_fx(mean_4g, Gain_meanNB_fx, mean_4g, Gain_mean_dicNB_fx, 1, 64);
        push_indice_fx( st_fx, IND_MEAN_GAIN2, idx_g, 6 );

        FOR(i = 0; i < Mbands_gn; i++)
        {
            y_gain_tmp[i] = sub(y_gain_tmp2[i],mean_4g[0]);
            move16();
        }
        /*if(y_gain_tmp[9] < -0.3f){y_gain_tmp[9] = -0.3f;}*/
        y_gain_tmp[9] = s_max(y_gain_tmp[9], -1229);
        move16();
        set16_fx(y_gain_tmp+10, 0, MBANDS_GN-10);
        idx_g = vquant_fx(y_gain_tmp, Mean_dic_NB_fx, y_gain_tmp, Gain_dic1_NB_fx, 3, 64);
        push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 6 );

        IF(L_sub(core_brate,ACELP_9k60) < 0)
        {
            idx_g = vquant_fx(y_gain_tmp+3, Mean_dic_NB_fx+3, y_gain_tmp+3, Gain_dic2_NB_fx, 3, 32);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            idx_g = vquant_fx(y_gain_tmp+6, Mean_dic_NB_fx+6, y_gain_tmp+6, Gain_dic3_NB_fx, 4, 16);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 4 );
        }
        ELSE
        {
            idx_g = vquant_fx(y_gain_tmp+3, Mean_dic_NB_fx+3, y_gain_tmp+3, Gain_dic2_NBHR_fx, 3, 64);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 6 );
            idx_g = vquant_fx(y_gain_tmp+6, Mean_dic_NB_fx+6, y_gain_tmp+6, Gain_dic3_NBHR_fx, 4, 128);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 7 );
        }/*add end */

        test();
        IF( L_sub(core_brate,ACELP_9k60) <= 0 && sub(coder_type,INACTIVE) == 0 )
        {
            /* Some energy is needed in high band for stat_noise_uv_enc
              to be functional in inactive speech */
            y_gain_tmp[10] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[6],8192),y_gain_tmp[7],8192),y_gain_tmp[8],8192));
            y_gain_tmp[11] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[7],8192),y_gain_tmp[8],8192),y_gain_tmp[9],8192));

            y_gain_tmp[12] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[8],8192),y_gain_tmp[9],8192),y_gain_tmp[10],8192));
            y_gain_tmp[13] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[9],8192),y_gain_tmp[10],8192),y_gain_tmp[11],8192));
            y_gain_tmp[14] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[10],8192),y_gain_tmp[11],8192),y_gain_tmp[12],8192));
            y_gain_tmp[15] = round_fx(L_mac(L_mac(L_mult(y_gain_tmp[11],8192),y_gain_tmp[12],8192),y_gain_tmp[13],8192));
        }
        ELSE
        {
            set16_fx( y_gain_tmp + 10, 0, MBANDS_GN - 10 );
        }
    }
    ELSE
    {
        /*ftmp1 =  mean(y_gain4, 16);*/

        L_tmp =0;
        FOR(cnt = 0 ; cnt < 16 ; cnt++)
        {
            L_tmp = L_mac(L_tmp,y_gain4[cnt], 2048);
        }
        tmp16 = round_fx(L_tmp);

        tmp1 = sub(tmp16,4915);
        tmp2 = add(tmp16,4915);
        L_tmp =0;
        FOR(i = 0; i < 16; i++)
        {
            y_gain_tmp2[i] = y_gain4[i];
            move16();
            /*if(y_gain4[i] < ftmp1-0.6f)*/
            y_gain_tmp2[i] = s_max(y_gain_tmp2[i], tmp1);
            move16();
            /*else if(y_gain4[i] > ftmp1+0.6f)*/
            y_gain_tmp2[i] = s_min(y_gain_tmp2[i], tmp2);
            move16();
            L_tmp = L_mac(L_tmp,y_gain_tmp2[i], 2048);
        }
        FOR(; i < Mbands_gn; i++)
        {
            y_gain_tmp2[i] = y_gain4[i];
            /*if(y_gain4[i] < ftmp1-0.6f)*/
            y_gain_tmp2[i] = s_max(y_gain_tmp2[i], tmp1);  /* Just the last move is needed, because s_max and s_min could be done in 1 line*/
            /*else if(y_gain4[i] > ftmp1+0.6f)*/
            y_gain_tmp2[i] = s_min(y_gain_tmp2[i], tmp2);
            move16();
        }

        /* Quantized mean gain without clipping */
        mean_4g[0] = round_fx(L_tmp);


        /*idx_g = (short)vquant(mean_4g, mean_m, mean_4g, mean_gain_dic, 1, 64);*/
        idx_g = vquant_fx(mean_4g, mean_m_fx, mean_4g, mean_gain_dic_fx, 1, 64);
        push_indice_fx( st_fx, IND_MEAN_GAIN2, idx_g, 6 );

        FOR(i = 0; i < Mbands_gn; i++)
        {
            y_gain_tmp[i] = sub(y_gain_tmp2[i],mean_4g[0]);
            move16();
        }

        IF( L_sub(core_brate,ACELP_9k60) < 0 )
        {
            /*mvr2r(y_gain_tmp, y_gain_tmp2, 8); */
            Copy(y_gain_tmp, y_gain_tmp2, 8);

            y_gain_tmp2[8] = y_gain_tmp[8];
            move16();
            y_gain_tmp2[9] = y_gain_tmp[10];
            move16();
            y_gain_tmp2[10] =y_gain_tmp[12];
            move16();
            y_gain_tmp2[11] =y_gain_tmp[14];
            move16();

            idx_g = 0;

            /*idx_g = (short)vquant(y_gain_tmp2, YGain_mean_LR, y_gain_tmp2, YGain_dic1_LR, 3, 32);*/
            idx_g = vquant_fx(y_gain_tmp2, YGain_mean_LR_fx, y_gain_tmp2, YGain_dic1_LR_fx, 3, 32 );
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            /*idx_g = (short)vquant(y_gain_tmp2+3, YGain_mean_LR+3, y_gain_tmp2+3, YGain_dic2_LR, 4, 32);*/
            idx_g = vquant_fx(y_gain_tmp2+3, YGain_mean_LR_fx+3, y_gain_tmp2+3, YGain_dic2_LR_fx, 4, 32 );
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            /*idx_g = (short)vquant(y_gain_tmp2+7, YGain_mean_LR+7, y_gain_tmp2+7, YGain_dic3_LR, 5, 32);*/
            idx_g = vquant_fx(y_gain_tmp2+7, YGain_mean_LR_fx+7, y_gain_tmp2+7, YGain_dic3_LR_fx, 5, 32);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            /*set_f(y_gain_tmp2+12, 0, MBANDS_GN-12);*/
            set16_fx(y_gain_tmp2+12, 0, MBANDS_GN-12);

            /* Update to quantized vector */
            Copy(y_gain_tmp2, y_gain_tmp, 8);

            Copy(y_gain_tmp2+8, y_gain_tmp3, 4);
            set16_fx(y_gain_tmp+8, 0,8);
            fft_rel_fx(y_gain_tmp2+8, 4, 2);

            Copy(y_gain_tmp2+8, y_gain_tmp+8, 3);
            y_gain_tmp[15] = y_gain_tmp2[11];
            ifft_rel_fx(y_gain_tmp+8, 8, 3);

            FOR(i = 8; i < 16; i++)
            {
                /*y_gain_tmp[i] *=  1.41f;*/
                y_gain_tmp[i] = shl( mult_r(y_gain_tmp[i] , 23101),1) ;
                move16();
            }

            y_gain_tmp[8] = y_gain_tmp3[0];
            move16();
            y_gain_tmp[10]= y_gain_tmp3[1];
            move16();
            y_gain_tmp[12]= y_gain_tmp3[2];
            move16();
            y_gain_tmp[14]= y_gain_tmp3[3];
            move16();
        }
        ELSE
        {
            idx_g = vquant_fx(y_gain_tmp, YG_mean16_fx, y_gain_tmp, YG_dicMR_1_fx, 4, 64);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 6 );
            idx_g = vquant_fx(y_gain_tmp+4, YG_mean16_fx+4, y_gain_tmp+4, YG_dicMR_2_fx, 4, 32);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            idx_g = vquant_fx(y_gain_tmp+8, YG_mean16_fx+8, y_gain_tmp+8, YG_dicMR_3_fx, 4, 32);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 5 );
            idx_g = vquant_fx(y_gain_tmp+12, YG_mean16_fx+12, y_gain_tmp+12, YG_dicMR_4_fx, 4, 16);
            push_indice_fx( st_fx, IND_Y_GAIN_TMP, idx_g, 4 );
        }
    }

    /* Gain adjustment to fit ACELP generic inactive coding gain at low rate */
    GSC_gain_adj(coder_type, core_brate, mean_4g[0], old_y_gain, y_gain_tmp, y_gainQ);
    return  mean_4g[0];  /*Q12*/
}
/*-------------------------------------------------------------------*
 * VDQ_vec()
 *
 * Return the dequantized vector of index
 *-------------------------------------------------------------------*/
static Word16 VDQ_vec_fx(
    Word16 *Qvec_out_fx,      /* o:  Quanitzed vector */
    const Word16 *mean_dic_fx,      /* i:  average codebook */
    const Word16 *dic_fx,           /* i:  codebook         */
    const Word16 index_fx,          /* i:  index of codebook*/
    const Word16 vec_en_fx          /* i:  vector length    */
)
{
    Word16 i, j;

    /*j =  shr_r(extract_l(L_mult(index_fx,vec_en_fx)),1);*/
    j = i_mult2(index_fx,vec_en_fx);
    FOR ( i = 0; i < vec_en_fx; i++)
    {
        Qvec_out_fx[i] = dic_fx[j++];
        move16();
    }

    FOR(i = 0; i < vec_en_fx; i++)
    {
        Qvec_out_fx[i] = add(Qvec_out_fx[i],mean_dic_fx[i]);
        move16();
    }

    return index_fx;
}
