/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/

static Word16 edyn_fx(const Word16 *vec, const Word16 lvec, Word16 Qnew);

static void gsc_enc_fx( Encoder_State_fx *st_fx, Word16 res_dct_in[],Word16 exc_dct_in[],
                        const Word16 Diff_len,const Word16 bits_used, const Word16 nb_subfr,const Word16 coder_type,
                        Word16 *lsf_new,Word16 *exc_wo_nf, Word16 *tmp_noise, Word16 Q_exc );

/*-------------------------------------------------------------------*
  * encod_audio()
  *
  * Encode audio (AC) frames
  *-------------------------------------------------------------------*/
void encod_audio_fx(
    Encoder_State_fx *st_fx,				/* i/o: State structure                                  		*/
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 speech[],              	/* i  : input speech                                Q_new     	*/
    const Word16 Aw[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq[],                 /* i  : 12k8 Lp coefficient                               */
    const Word16 T_op[],                	/* i  : open loop pitch                                  		*/
    const Word16 voicing[],             	/* i  : voicing                                     Q15     	*/
    const Word16 *res,                  	/* i  : residual signal                             Q_new     	*/
    Word16 *synth,                	/* i/o: core synthesis                              Q-1     	*/
    Word16 *exc,                  	/* i/o: current non-enhanced excitation             Q_new     	*/
    Word16 *pitch_buf,				/* i/o: floating pitch values for each subframe     Q6     		*/
    Word16 *voice_factors,			/* o  : voicing factors                             Q15   		*/
    Word16 *bwe_exc,				/* o  : excitation for SWB TBE						Q0			*/
    const Word16 attack_flag,			/* i  : Flag that point to an attack coded with AC mode (GSC)	*/
    const Word16 coder_type,				/* i  : coding type                                             */
    Word16 *lsf_new,                /* i  : current frame ISF vector 								*/
    Word16 *tmp_noise,              /* o  : noise energy   */
    Word16 Q_new,
    Word16 shift
)
{
    const Word16 *p_Aq;
    Word16 i, i_subfr, nb_subfr, last_pit_bin;
    Word16 T0_tmp, T0_frac_tmp, nb_subfr_flag;
    Word16 tmp_nb_bits_tot = 0;
    Word16 Es_pred;
    Word16 dct_res[L_FRAME], dct_epit[L_FRAME];
    Word16 m_mean = 0;
    Word16 saved_bit_pos;
    Word16 exc_wo_nf[L_FRAME];
    Word32 Lm_mean;
    Word16 nb_bits;
    Word16 indice;

    m_mean = 0;
    move16();
    tmp_nb_bits_tot = 0;
    move16();

    T0_tmp = 64;
    move16();
    T0_frac_tmp = 0;
    move16();
    Copy(mem->mem_syn, st_fx->mem_syn_tmp_fx, M);
    st_fx->mem_w0_tmp_fx = mem->mem_w0;
    move16();
    Es_pred = 0;
    move16();

    /*---------------------------------------------------------------*
     * Encode GSC attack flag (used to reduce possible pre-echo)
     * Encode GSC SWB speech flag
     *---------------------------------------------------------------*/
    push_indice_fx( st_fx, IND_GSC_ATTACK, attack_flag, 1 );

    test();
    IF( sub(coder_type,INACTIVE ) != 0&& L_sub(st_fx->total_brate_fx,ACELP_13k20) >= 0)
    {
        push_indice_fx( st_fx,IND_GSC_SWB_SPEECH, st_fx->GSC_noisy_speech_fx, 1);
    }
    /*---------------------------------------------------------------*
     * Find and encode the number of subframes
     *---------------------------------------------------------------*/
    test();
    IF ( L_sub(st_fx->core_brate_fx,ACELP_9k60) >= 0&& L_sub(st_fx->core_brate_fx,ACELP_13k20) <= 0 )
    {
        FOR( i = 0; i < 5; i++)
        {
            test();
            if( sub(abs_s(st_fx->gsc_lt_diff_etot_fx[MAX_LT-i-1]),1536) > 0 && sub(st_fx->cor_strong_limit_fx,1) == 0 )
            {
                st_fx->cor_strong_limit_fx = 0;
                move16();
            }
        }
    }
    IF( st_fx->GSC_noisy_speech_fx )
    {
        nb_subfr = NB_SUBFR;
        move16();
        st_fx->cor_strong_limit_fx = 0;
        move16();
        nb_subfr_flag = 1;
        move16();
    }
    ELSE
    {
        test();
        test();
        IF( (st_fx->cor_strong_limit_fx == 0 || sub(coder_type,INACTIVE) == 0) && L_sub(st_fx->core_brate_fx,ACELP_9k60) >= 0 )
        {
            nb_subfr = 2;
            move16();
            nb_subfr_flag = 0;
            move16();
            st_fx->cor_strong_limit_fx = 0;
            move16();
        }
        ELSE
        {
            nb_subfr = SWNB_SUBFR;
            move16();
            nb_subfr_flag = 1;
            move16();
        }
        IF( L_sub(st_fx->core_brate_fx,ACELP_9k60) >= 0 )
        {
            /* nb_subfr_flag can only have the value 0 or 1 */
            push_indice_fx( st_fx, IND_HF_NOISE, nb_subfr_flag, 1);
        }
    }

    /*---------------------------------------------------------------*
     * Compute adaptive (pitch) excitation contribution
     *---------------------------------------------------------------*/

    test();
    IF( st_fx->GSC_noisy_speech_fx && sub(nb_subfr,NB_SUBFR ) == 0 )
    {
        nb_bits = Es_pred_bits_tbl[BIT_ALLOC_IDX_fx(st_fx->core_brate_fx, GENERIC, -1, -1)];
        move16();
        Es_pred_enc_fx( &Es_pred, &indice, L_FRAME, res, voicing, nb_bits,0, Q_new
                      );
        push_indice_fx( st_fx, IND_ES_PRED, indice, nb_bits );
    }

    enc_pit_exc_fx( st_fx, mem, speech, Aw, Aq,Es_pred, T_op, voicing, res, synth, exc, &T0_tmp,
                    &T0_frac_tmp, pitch_buf, nb_subfr, &st_fx->lt_gpitch_fx, &saved_bit_pos, Q_new, shift );

    /*---------------------------------------------------------------*
     * DCT transform
     *---------------------------------------------------------------*/
    edct_16fx( exc, dct_epit, L_FRAME, 5 );
    edct_16fx( res, dct_res, L_FRAME, 5 );

    /*---------------------------------------------------------------*
     * Calculate energy dynamics
     *---------------------------------------------------------------*/
    Lm_mean = L_deposit_l(0);
    FOR( i = 7; i < 15; i++ )
    {
        /*m_mean = add(m_mean,edyn_fx( dct_res+i*16, 16, Q_new )); */
        Lm_mean = L_mac(Lm_mean, edyn_fx( dct_res+i*16, 16, Q_new ), 4096);/*Q7*/
    }
    m_mean = round_fx(Lm_mean);/*Q7*/

    IF( sub(m_mean,st_fx->mid_dyn_fx) > 0 )
    {
        /*st_fx->mid_dyn_fx = 0.2f * st_fx->mid_dyn_fx + 0.8f * m_mean;*/
        st_fx->mid_dyn_fx = round_fx(L_mac(L_mult(26214,m_mean),6554,st_fx->mid_dyn_fx));/*Q7*/
    }
    ELSE
    {
        /*st_fx->mid_dyn_fx = 0.6f * st_fx->mid_dyn_fx + 0.4f * m_mean;*/
        st_fx->mid_dyn_fx = round_fx(L_mac(L_mult(13107,m_mean),19661,st_fx->mid_dyn_fx));/*Q7*/
    }
    IF( sub(coder_type,INACTIVE) != 0 )
    {
        st_fx->noise_lev_fx = sub((NOISE_LEVEL_SP3+1), usquant_fx(st_fx->mid_dyn_fx, &m_mean, MIN_DYNAMIC_FX, shr(GSF_NF_DELTA_FX,1), GSC_NF_STEPS));

        st_fx->noise_lev_fx = s_min(st_fx->noise_lev_fx, NOISE_LEVEL_SP3);
    }

    st_fx->past_dyn_dec_fx = st_fx->noise_lev_fx;
    move16();
    IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0)
    {
        st_fx->noise_lev_fx = s_max(st_fx->noise_lev_fx, NOISE_LEVEL_SP2);
        push_indice_fx( st_fx, IND_NOISE_LEVEL, sub(st_fx->noise_lev_fx, NOISE_LEVEL_SP2), 2 );
    }
    ELSE IF( st_fx->GSC_noisy_speech_fx )
    {
        st_fx->noise_lev_fx = NOISE_LEVEL_SP3;
        move16();
    }
    ELSE
    {
        push_indice_fx( st_fx, IND_NOISE_LEVEL, sub(st_fx->noise_lev_fx, NOISE_LEVEL_SP0), 3 );
    }

    /*---------------------------------------------------------------*
     * Find and encode the last band where the adaptive (pitch) contribution is significant
     *---------------------------------------------------------------*/

    last_pit_bin = Pit_exc_contribution_len_fx( st_fx, dct_res, dct_epit, pitch_buf, nb_subfr, &st_fx->pit_exc_hangover, coder_type, Q_new );

    IF( last_pit_bin == 0 )
    {
        mem->tilt_code = 0;
        move16();
    }
    ELSE
    {
        /*last_pit_bin++;*/
        last_pit_bin = add(last_pit_bin,1);
    }

    /*--------------------------------------------------------------------------------------*
     * GSC encoder
     *--------------------------------------------------------------------------------------*/

    /* Find the current total number of bits used */
    tmp_nb_bits_tot = st_fx->nb_bits_tot_fx;
    move16();


    if( st_fx->extl_brate_fx > 0 )
    {
        /* subtract 1 bit for TBE/BWE BWE flag (bit counted in extl_brate) */
        tmp_nb_bits_tot = sub(tmp_nb_bits_tot,1);
    }
    test();
    if( sub(coder_type,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_9k60) <= 0 )
    {
        /* add 5 bits for noisiness */
        tmp_nb_bits_tot = add(tmp_nb_bits_tot,5);
    }

    gsc_enc_fx( st_fx, dct_res, dct_epit, last_pit_bin, tmp_nb_bits_tot, nb_subfr, coder_type, lsf_new, exc_wo_nf, tmp_noise, Q_new );

    /*--------------------------------------------------------------------------------------*
     * iDCT transform
     *--------------------------------------------------------------------------------------*/

    edct_16fx( dct_epit, exc, L_FRAME, 5 );
    edct_16fx( exc_wo_nf, exc_wo_nf, L_FRAME, 5 );

    /*--------------------------------------------------------------------------------------*
     * Remove potential pre-echo in case an onset has been detected
     *--------------------------------------------------------------------------------------*/

    pre_echo_att_fx( &st_fx->Last_frame_ener_fx, exc, attack_flag, Q_new
                     ,st_fx->last_coder_type_fx
                   );

    /*--------------------------------------------------------------------------------------*
     * Update BWE excitation
     *--------------------------------------------------------------------------------------*/

    set16_fx( voice_factors, 0, NB_SUBFR );
    interp_code_5over2_fx( exc, bwe_exc, L_FRAME );

    /*--------------------------------------------------------------------------------------*
     * Synthesis
     *--------------------------------------------------------------------------------------*/

    p_Aq = Aq;
    FOR (i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR)
    {
        Syn_filt_s( 1, p_Aq, M, &exc_wo_nf[i_subfr], &synth[i_subfr], L_SUBFR, mem->mem_syn, 1 );
        p_Aq += (M+1);
    }

    /*--------------------------------------------------------------------------------------*
     * Updates
     *--------------------------------------------------------------------------------------*/

    mem->mem_w0 = st_fx->mem_w0_tmp_fx;
    move16();
    Copy( exc_wo_nf, exc, L_FRAME );

    return;
}

/*================================================================================*/
/* FUNCTION : void gsc_enc_fx	()   							                  */
/*--------------------------------------------------------------------------------*/
/* PURPOSE  :  Generic audio signal encoder                                       */
/*--------------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :															  */
/* _ (Word16) res_dct_in		 : dct of residual signal				 Q_exc    */
/* _ (Word16) Diff_len 	         : Lenght of the difference signal       Q0       */
/* _ (Word16) coder_type         : coding type				             Q0       */
/* _ (Word16) bits_used          : Number of bit used before frequency Q Q0       */
/* _ (Word16) nb_subfr           : Number of subframe considered         Q0       */
/* _ (Word16)  Qexc              : Q format of exc_dct_in			              */
/*--------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													          */
/* _ None                                                                         */
/*--------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											          */
/*   Encoder_State_fx *st_fx:Encoder State Structure                              */
/* _ (Word16[]) exc_dct_in : dctof pitch-only excitation / total excitation Q_exc */
/*--------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													          */
/* _None                                                     			          */
/*================================================================================*/

#define Q_FPC_OUT 10

static void gsc_enc_fx(
    Encoder_State_fx *st_fx,              /* i/o: State structure                               */
    Word16 res_dct_in[],		  /* i  : dct of residual signal                        */
    Word16 exc_dct_in[],		  /* i/o: dct of pitch-only excitation / total excitation */
    const Word16 Diff_len,
    const Word16 bits_used,
    const Word16 nb_subfr,
    const Word16 coder_type,
    Word16 *lsf_new,			  /* i  : ISFs at the end of the frame                  */
    Word16 *exc_wo_nf,			  /* o  : excitation (in f domain) without noisefill    */
    Word16 *tmp_noise,			  /* o  : noise energy                                  */
    Word16 Q_exc
)
{
    Word16 y2_filt[L_FRAME];
    Word16 exc_diffQ[L_FRAME];
    Word16 exc_diff[L_FRAME];
    Word16 bit,tmp;
    Word16 nb_subbands;
    Word16 pvq_len, i;
    Word16 bits_per_bands[MBANDS_GN];
    Word16 tmp_band;
    Word16 concat_in[L_FRAME];
    Word16 concat_out[L_FRAME];
    Word16 max_ener_band[MBANDS_GN], j;
    Word16 Ener_per_bd_iQ[MBANDS_GN];
    Word16 last_bin, mean_gain;
    Word16 bitallocation_band[MBANDS_GN];
    Word16 bitallocation_exc[2];

    Word16 inpulses_fx[NB_SFM];
    Word16 imaxpulse_fx[NB_SFM];
    Word16 Q_tmp;

    set16_fx( inpulses_fx, 0, NB_SFM );
    set16_fx( imaxpulse_fx, 0, NB_SFM );

    /*--------------------------------------------------------------------------------------*
     * Initialization
     *--------------------------------------------------------------------------------------*/

    bit = bits_used;
    move16();
    set16_fx( exc_diffQ, 0, L_FRAME );
    set16_fx( y2_filt, 0, L_FRAME );

    /*--------------------------------------------------------------------------------------*
     * Calculate the difference between the residual spectrum and the spectrum of adaptive excitation
     * (non valuable temporal content present in exc_dct_in is already zeroed)
     *--------------------------------------------------------------------------------------*/

    Vr_subt( res_dct_in, exc_dct_in, exc_diff, L_FRAME );
    exc_diff[0] = 0;
    move16();

    /*--------------------------------------------------------------------------------------*
     * Multiply the difference spectrum with the normalized spectral shape of the residual signal
     * This improves the stability of the differnece spectrum since the spectral shape of the
     * residual signal is less suseptible to rapid changes than the difference spectrum
     *--------------------------------------------------------------------------------------*/
    IF( Diff_len == 0 )
    {
        tmp_band = 0;
        move16();
    }
    ELSE
    {
        tmp_band = st_fx->mem_last_pit_band_fx;
        move16();
    }

    Ener_per_band_comp_fx( exc_diff, Ener_per_bd_iQ, Q_exc, MBANDS_GN, 1 );

    /*--------------------------------------------------------------------------------------*
     * Gain quantizaion
     *--------------------------------------------------------------------------------------*/

    mean_gain = gsc_gainQ_fx( st_fx, Ener_per_bd_iQ, Ener_per_bd_iQ, st_fx->core_brate_fx, st_fx->old_y_gain_fx, coder_type, st_fx->bwidth_fx );
    *tmp_noise = mult_r(320,mean_gain);  /*10 in Q5  lp_gainc in Q3 */

    /*--------------------------------------------------------------------------------------*
     * Frequency encoder
     *--------------------------------------------------------------------------------------*/

    bands_and_bit_alloc_fx( st_fx->cor_strong_limit_fx, st_fx->noise_lev_fx, st_fx->core_brate_fx, Diff_len, bit, &bit, Ener_per_bd_iQ,
                            max_ener_band, bits_per_bands, &nb_subbands, exc_diff, concat_in, &pvq_len, coder_type, st_fx->bwidth_fx, st_fx->GSC_noisy_speech_fx );

    Q_tmp = Q_exc;
    move16();

    tmp = pvq_core_enc_fx( st_fx, concat_in, concat_out, &Q_tmp, bit, nb_subbands, gsc_sfm_start, gsc_sfm_end,
                           gsc_sfm_size, bits_per_bands, NULL, inpulses_fx, imaxpulse_fx, ACELP_CORE );

    Scale_sig( concat_out, gsc_sfm_end[nb_subbands-1], sub(Q_FPC_OUT, Q_tmp) );

    bit = sub(bit,tmp);
    /* write unused bits */
    WHILE( bit > 0 )
    {
        i = s_min( bit, 16 );
        push_indice_fx( st_fx, IND_UNUSED, 0, i );
        bit = sub(bit,i);
    }
    /* Reorder Q bands */
    last_bin = 0;
    move16();
    set16_fx( bitallocation_band, 0, MBANDS_GN );

    FOR(j = 0; j < nb_subbands; j++)
    {
        Copy( concat_out+j*16, exc_diffQ + max_ener_band[j]*16, 16 );/*Q12*/

        last_bin = s_max(last_bin , max_ener_band[j]);

        bitallocation_band[ max_ener_band[j]] = 1;
        move16();
    }
    test();
    IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) == 0 && sub(st_fx->bwidth_fx,NB) != 0 )
    {
        bitallocation_exc[0] = 0;
        move16();
        bitallocation_exc[1] = 0;
        move16();
        if( exc_diffQ[L_FRAME8k - 2] != 0 )
        {
            bitallocation_exc[0] = 1;
            move16();
        }
        if( exc_diffQ[L_FRAME8k - 1] != 0 )
        {
            bitallocation_exc[1] = 1;
            move16();
        }
    }

    /*--------------------------------------------------------------------------------------*
     * Skip adaptive (pitch) contribution frequency band (no noise added over the adaptive (pitch) contribution)
     * Find x pulses between 1.6-3.2kHz to code in the spectrum of the residual signal
     * Gain is based on the inter-correlation gain between the pulses found and residual signal
     *--------------------------------------------------------------------------------------*/

    freq_dnw_scaling_fx( st_fx->cor_strong_limit_fx, coder_type, st_fx->noise_lev_fx, st_fx->core_brate_fx, exc_diffQ, Q_FPC_OUT );

    /*--------------------------------------------------------------------------------------*
     * Estimate noise level
     *--------------------------------------------------------------------------------------*/

    highband_exc_dct_in_fx( st_fx->core_brate_fx, mfreq_bindiv_loc_fx, last_bin, Diff_len, st_fx->noise_lev_fx, tmp_band, exc_diffQ,
                            &st_fx->seed_tcx_fx, Ener_per_bd_iQ, nb_subfr, exc_dct_in, st_fx->last_coder_type_raw_fx, bitallocation_band, lsf_new,
                            st_fx->last_exc_dct_in_fx, &st_fx->last_ener_fx, st_fx->last_bitallocation_band_fx, bitallocation_exc, 0, coder_type,
                            st_fx->bwidth_fx, exc_wo_nf, Q_FPC_OUT, Q_exc, st_fx->GSC_noisy_speech_fx );

    exc_dct_in[0] = 0;
    move16();

    return;
}

/*======================================================================*/
/* FUNCTION : edyn_fx()										*/
/*----------------------------------------------------------------------*/
/* PURPOSE :  Calculate energy dynamics in a vector						*/
/*            (ratio of energy maximum to energy mean)					*/
/*																		*/
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    */
/* _ (Word16 *) vec		: ratio of max to mean 		    Qnew	        */
/* _ (Word16)	lvec	: input vector     		                        */
/* _ (Word16) Q_new		: 												*/
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16) dyn	    : ratio of energy maximum to energy mean (Q7)	 */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/

static Word16 edyn_fx(         /* o  : ratio of max to mean    */
    const Word16 *vec,      /* i  : input vector            */
    const Word16 lvec,       /* i  : length of input vector  */
    Word16 Qnew
)
{
    Word16 j=0;
    Word16 dyn;
    Word32 L_tmp, ener_max, ener_mean;
    Word16 tmp,exp2,tmp2,tmp1,exp1,exp3;
    Word16 scale;

    ener_mean = L_shl(1,shl(Qnew,1)); /*2*Qnew*/
    ener_max = L_shl(1,shl(Qnew,1));

    FOR( j=0; j<lvec; j++ )
    {
        L_tmp = L_mult0(vec[j], vec[j]);    /*2*Qnew*/
        ener_max = L_max(ener_max, L_tmp);
        ener_mean = L_add(ener_mean,L_tmp);
    }
    /*dyn = 10.0f * (ener_max / ener_mean);*/
    ener_mean = Mult_32_16(ener_mean,div_s(1,lvec));  /*2*Qnew*/

    IF(ener_mean > 0)
    {
        exp1 = norm_l(ener_mean);
        tmp1 = round_fx(L_shl(ener_mean,exp1));
        exp1 = sub(30,exp1);

        exp2 = norm_l(ener_max);
        tmp2 = extract_h(L_shl(ener_max,exp2));
        exp2 = sub(30,exp2);

        scale = shr(sub(tmp1, tmp2), 15);
        tmp2 = shl(tmp2, scale);
        exp2 = sub(exp2, scale);

        exp3 = sub(exp1,exp2);

        tmp = div_s(tmp2, tmp1);             /*Q(15+exp3)*/

        L_tmp = L_shr_r(L_mult(tmp,10),exp3);
        dyn = round_fx(L_shl(L_tmp,7));  /*Q7*/
    }
    ELSE
    {
        dyn = 1280;
        move16();
    }
    return dyn;

}
