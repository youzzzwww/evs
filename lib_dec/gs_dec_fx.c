/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "stl.h"
#include "options.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"

/*=========================================================================*/
/* FUNCTION : void decod_audio_fx();								       */
/*-------------------------------------------------------------------------*/
/* PURPOSE :  Decode audio (AC) frames                          		   */
/*-------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												       */
/* _ (Word16[]) Aq  			: LP filter coefficient		Q12			   */
/* _ (Word16) coder_type 		: coding type				Q0			   */
/* _(Word16) Q_exc              :Q format of excitation                    */
/*-------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													   */
/* _ (Word16[]) pitch_buf_fx	: floating pitch values for each subframe Q6*/
/* _ (Word16[])	voice_factors_fx: frame error rate				Q15		    */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/*  Decoder_State_fx *st_fx     : decoder  memory structure                 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q_exc)			    */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q_exc)       */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _ None																    */
/*==========================================================================*/
void decod_audio_fx(
    Decoder_State_fx *st_fx,             /* i/o: decoder static memory                     */
    Word16 dct_epit[],           /* o  : GSC excitation in DCT domain              */
    const Word16 *Aq,                  /* i  : LP filter coefficient                     */
    const Word16 coder_type,           /* i  : coding type                               */
    Word16 *pitch_buf,           /* o  : floating pitch values for each subframe   */
    Word16 *voice_factors,       /* o  : voicing factors                           */
    Word16 *exc,                 /* i/o: adapt. excitation exc                     */
    Word16 *exc2,                /* i/o: adapt. excitation/total exc               */
    Word16 *bwe_exc,              /* o  : excitation for SWB TBE                    */
    Word16 *lsf_new              /* i  : ISFs at the end of the frame              */
    , Word16 *gain_buf              /*Q14*/
)
{
    Word16 tmp_nb_bits_tot, pit_band_idx;
    Word16 code[L_SUBFR];
    Word16 Diff_len, nb_subfr, i;
    Word16 nb_frame_flg;
    Word16 Es_pred = 0;
    Word16 Len, max_len;
    Word16 gsc_attack_flag;

    Word16 low_pit;
    Word16 last_bin;
    Word16 nbits;

    Word16 exc_wo_nf[L_FRAME];


    /*---------------------------------------------------------------*
     * Initialization
     *---------------------------------------------------------------*/
    Diff_len  = 0;
    move16();

    /* decode GSC attack flag (used to reduce possible pre-echo) */
    gsc_attack_flag = (Word16) get_next_indice_fx( st_fx, 1 );

    /* decode GSC SWB speech flag */
    test();
    IF( sub(coder_type,INACTIVE) != 0 && L_sub(st_fx->total_brate_fx,ACELP_13k20) >= 0 )
    {
        st_fx->GSC_noisy_speech_fx = (Word16) get_next_indice_fx( st_fx, 1 );
    }
    /*---------------------------------------------------------------*
     * Decode energy dynamics
     *---------------------------------------------------------------*/
    IF( sub(st_fx->GSC_noisy_speech_fx,1) == 0 )
    {
        nb_subfr = NB_SUBFR;
        move16();
        st_fx->cor_strong_limit_fx = 0;
        move16();
        st_fx->noise_lev_fx = NOISE_LEVEL_SP3;
        move16();
    }
    ELSE
    {
        IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0 )
        {
            st_fx->noise_lev_fx = add((Word16)get_next_indice_fx( st_fx, 2 ), NOISE_LEVEL_SP2);
        }
        ELSE
        {
            st_fx->noise_lev_fx = add((Word16)get_next_indice_fx( st_fx, 3 ),  NOISE_LEVEL_SP0);
        }

        /*---------------------------------------------------------------*
        * Decode number of subframes
        *---------------------------------------------------------------*/

        st_fx->cor_strong_limit_fx = 1;
        move16();
        nb_subfr = SWNB_SUBFR;
        move16();

        IF( L_sub(st_fx->core_brate_fx,ACELP_9k60) >= 0 )
        {
            nbits = 1;
            move16();
            if( L_sub(st_fx->core_brate_fx,MIN_RATE_4SBFR) >= 0 )
            {
                nbits = 2;
                move16();
            }

            nb_frame_flg = (Word16)get_next_indice_fx( st_fx, nbits );

            IF( s_and(nb_frame_flg,0x1) == 0)
            {
                nb_subfr = 2*SWNB_SUBFR;
                move16();
                st_fx->cor_strong_limit_fx = 0;
                move16();
            }
            ELSE IF ( L_sub(st_fx->core_brate_fx,MIN_RATE_4SBFR) >= 0)
            {
                nb_subfr = 2*SWNB_SUBFR;
                move16(); /* cor_strong already set to 1 */
            }

            if( sub(shr(nb_frame_flg,1),1) == 0)
            {
                nb_subfr = shl(nb_subfr,1);
            }
        }
    }

    /*---------------------------------------------------------------*
     * Decode the last band where the adaptive (pitch) contribution is significant
     *---------------------------------------------------------------*/

    IF( L_sub(st_fx->core_brate_fx,CFREQ_BITRATE) < 0 )
    {
        nbits = 3;
        move16();
        test();
        if( L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0 && (sub(coder_type,INACTIVE) == 0))
        {
            nbits = 1;
            move16();
        }
    }
    ELSE
    {
        nbits = 4;
        move16();
    }
    test();
    IF( L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0 && sub(coder_type,INACTIVE) != 0 )
    {
        pit_band_idx = 1;
        move16();
    }
    ELSE
    {
        pit_band_idx = (Word16)get_next_indice_fx( st_fx, nbits );
    }

    IF( pit_band_idx != 0 )
    {
        IF( L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0 )
        {
            pit_band_idx = 7+BAND1k2;
            move16();  /* At low rate, if pitch model is chosen, then for to be use on extented and constant frequency range */
        }
        ELSE
        {
            pit_band_idx = add(pit_band_idx, BAND1k2);
        }
        Diff_len = mfreq_loc_div_25[pit_band_idx];
        move16();
    }
    st_fx->Last_GSC_pit_band_idx_fx = pit_band_idx;
    move16();


    /*--------------------------------------------------------------------------------------*
     * Decode adaptive (pitch) excitation contribution
     * Reset unvaluable part of the adaptive (pitch) excitation contribution
     *--------------------------------------------------------------------------------------*/
    IF( sub(pit_band_idx,BAND1k2) > 0 )
    {
        /*---------------------------------------------------------------*
         * Decode adaptive (pitch) excitation contribution
         *---------------------------------------------------------------*/
        test();
        IF( sub(st_fx->GSC_noisy_speech_fx,1) == 0 && sub(nb_subfr,NB_SUBFR) == 0 )
        {
            Es_pred_dec_fx( st_fx, &Es_pred, GENERIC, st_fx->core_brate_fx );
        }

        dec_pit_exc_fx( st_fx, Aq, coder_type, Es_pred, pitch_buf, code, exc, bwe_exc, nb_subfr
                        , gain_buf
                      );

        IF( L_sub(st_fx->core_brate_fx,ACELP_9k60) < 0 )
        {
            minimum_fx( pitch_buf, shr(L_FRAME,6), &low_pit);
            low_pit = shr(low_pit, 6);       /*Q6 -> Q0 */

            IF( sub(low_pit,64) < 0)
            {
                pit_band_idx = 9+BAND1k2;
                move16();
                if(sub(st_fx->bwidth_fx,NB) == 0)
                {
                    pit_band_idx = 7+BAND1k2;
                    move16();
                }
            }
            ELSE IF ( sub(low_pit,128) < 0 )
            {
                pit_band_idx = 5+BAND1k2;
                move16();
            }
            ELSE
            {
                pit_band_idx = 3+BAND1k2;
                move16();
            }

            Diff_len = mfreq_loc_div_25[pit_band_idx];
            move16();
            st_fx->Last_GSC_pit_band_idx_fx = pit_band_idx;
            move16();
        }

        /*---------------------------------------------------------------*
         * DCT transform
         *---------------------------------------------------------------*/

        edct_16fx( exc, dct_epit, L_FRAME, 5 );

        /*---------------------------------------------------------------*
         * Reset unvaluable part of the adaptive (pitch) excitation contribution
         *---------------------------------------------------------------*/

        max_len = sub( L_FRAME, Diff_len );

        if(sub(st_fx->bwidth_fx,NB) == 0)
        {
            max_len = sub(160,Diff_len);
        }

        Len = 80;
        move16();
        if( max_len < 80 )
        {
            Len = max_len;
            move16();
        }

        test();
        IF(L_sub(st_fx->core_brate_fx,ACELP_8k00) == 0 && sub(st_fx->bwidth_fx,NB) != 0 )
        {
            FOR (i=0; i < max_len; i++)
            {
                dct_epit[i+Diff_len] = 0;
                move16();
            }
        }
        ELSE
        {
            FOR (i = 0; i < Len; i++)
            {
                dct_epit[i + Diff_len] = mult_r(dct_epit[i + Diff_len],sm_table_fx[i]);
                move16();
            }

            FOR (; i < max_len; i++)
            {
                dct_epit[i + Diff_len] = 0;
                move16();
            }
        }
        st_fx->bfi_pitch_fx  = mean_fx(pitch_buf, nb_subfr);
        move16();
        st_fx->bfi_pitch_frame_fx = L_FRAME;
        move16();

        Diff_len = add(Diff_len,1);
        st_fx->bpf_off_fx = 0;
        move16();
    }
    ELSE
    {
        /* No adaptive (pitch) excitation contribution */
        st_fx->bpf_off_fx = 1;
        move16();
        set16_fx( dct_epit, 0, L_FRAME );

        IF(sub(st_fx->L_frame_fx , L_FRAME16k) == 0 )
        {
            set16_fx( pitch_buf, shl(L_SUBFR16k,6), NB_SUBFR16k );
        }
        ELSE
        {
            set16_fx( pitch_buf, shl(L_SUBFR,6), NB_SUBFR );
        }

        set16_fx( gain_buf, 0, NB_SUBFR16k);

        st_fx->bfi_pitch_fx = shl(L_SUBFR,6);
        st_fx->bfi_pitch_frame_fx = L_FRAME;
        move16();
        st_fx->lp_gainp_fx = 0;
        move16();
        st_fx->lp_gainc_fx = 0;
        move16();
        st_fx->tilt_code_fx = 0;
        move16();
        pit_band_idx = 0;
        move16();
        Diff_len = 0;
        move16();
    }

    /*--------------------------------------------------------------------------------------*
     * GSC decoder
     *--------------------------------------------------------------------------------------*/

    /* find the current total number of bits used */

    tmp_nb_bits_tot = st_fx->next_bit_pos_fx;
    move16();

    if( st_fx->extl_brate_fx > 0 )
    {
        /* subtract 1 bit for TBE/BWE BWE flag (bit counted in extl_brate) */
        tmp_nb_bits_tot = sub(tmp_nb_bits_tot, 1);
    }


    test();
    if( sub(coder_type,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_9k60) <= 0 )
    {
        tmp_nb_bits_tot = add(tmp_nb_bits_tot,5);
    }

    gsc_dec_fx(st_fx, dct_epit, pit_band_idx, Diff_len, tmp_nb_bits_tot, nb_subfr, coder_type, &last_bin, lsf_new, exc_wo_nf, st_fx->Q_exc );

    /*--------------------------------------------------------------------------------------*
     * iDCT transform
     *--------------------------------------------------------------------------------------*/

    edct_16fx( dct_epit, exc, L_FRAME, 5 );
    edct_16fx( exc_wo_nf, exc_wo_nf, L_FRAME, 5 );

    /*----------------------------------------------------------------------*
     * Remove potential pre-echo in case an onset has been detected
     *----------------------------------------------------------------------*/

    pre_echo_att_fx( &st_fx->Last_frame_ener_fx, exc, gsc_attack_flag
                     ,st_fx->Q_exc
                     ,st_fx->last_coder_type_fx
                   );

    /*--------------------------------------------------------------------------------------*
     * Update BWE excitation
     *--------------------------------------------------------------------------------------*/

    set16_fx( voice_factors, 0, NB_SUBFR16k );
    interp_code_5over2_fx( exc, bwe_exc, L_FRAME );

    /*--------------------------------------------------------------------------------------*
     * Updates
     *--------------------------------------------------------------------------------------*/

    Copy( exc, exc2, L_FRAME );
    Copy( exc_wo_nf, exc, L_FRAME );

    /*--------------------------------------------------------------------------------------*
     * Channel aware mode parameters
     *--------------------------------------------------------------------------------------*/

    set16_fx( st_fx->tilt_code_dec_fx, 0, NB_SUBFR16k );
    set32_fx( st_fx->gain_code_fx, 0, NB_SUBFR16k );


    return;
}

/*==========================================================================*/
/* FUNCTION : void gsc_dec_fx	()   							            */
/*--------------------------------------------------------------------------*/
/* PURPOSE  :  Generic audio signal decoder                                 */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word16) pit_band_idx       : bin position of the cut-off frequency Q0 */
/* _ (Word16) Diff_len 	         : Lenght of the difference signal       Q0 */
/* _ (Word16) coder_type         : coding type				             Q0 */
/* _ (Word16) bits_used          : Number of bit used before frequency Q Q0 */
/* _ (Word16) nb_subfr           : Number of subframe considered         Q0 */
/* _ (Word16)  Qexc              : Q format of exc_dct_in			        */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ None                                                                   */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/*   Decoder_State_fx *st_fx:Decoder State Structure                        */
/* _ (Word16[]) exc_dct_in : dctof pitch-only excitation / total excitation Qexc*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _None                                                     			    */
/*==========================================================================*/
void gsc_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: State structure                                     */
    Word16 exc_dct_in[],    /* i/o: dct of pitch-only excitation / total excitation     */
    const Word16 pit_band_idx,	/* i  : bin position of the cut-off frequency               */
    const Word16 Diff_len,		/* i  : Lenght of the difference signal (before pure spectral)*/
    const Word16 bits_used,       /* i  : Number of bit used before frequency Q               */
    const Word16 nb_subfr,		/* i  : Number of subframe considered                       */
    const Word16 coder_type,      /* i  : coding type                                         */
    Word16 *last_bin,       /* i  : last bin of bit allocation                          */
    Word16 *lsf_new,        /* i  : ISFs at the end of the frame                        */
    Word16 *exc_wo_nf,      /* o  : excitation (in f domain) without noisefill          */
    Word16 Q_exc
)
{
    Word16 i, j, bit, nb_subbands, pvq_len;
    Word16 bitallocation_band[MBANDS_GN];
    Word16 bitallocation_exc[2];
    Word16 Ener_per_bd_iQ[MBANDS_GN];
    Word16 max_ener_band[MBANDS_GN];
    Word16 exc_diffQ[L_FRAME];
    Word16 bits_per_bands[MBANDS_GN];
    Word16 concat_out[L_FRAME];
    Word16 inpulses_fx[NB_SFM];
    Word16 imaxpulse_fx[NB_SFM];
    Word16 mean_gain;
#define Q_FPC_OUT 10
    Word16 Mbands_gn = 16;
    Word16 Qexc_diffQ=Q_FPC_OUT;
    Word32 L_tmp;
    Word16 Q_tmp;

    set16_fx(inpulses_fx, 0,NB_SFM);
    set16_fx(imaxpulse_fx, 0,NB_SFM);

    /*--------------------------------------------------------------------------------------*
     * Initialization
     *--------------------------------------------------------------------------------------*/
    bit = bits_used;
    move16();
    set16_fx( exc_diffQ, 0, L_FRAME );

    /*--------------------------------------------------------------------------------------*
     * Gain decoding
     *--------------------------------------------------------------------------------------*/
    IF( st_fx->bfi_fx )
    {
        /* copy old gain */
        Copy( st_fx->old_y_gain_fx, Ener_per_bd_iQ, Mbands_gn );
        mean_gain = mult_r(st_fx->lp_gainc_fx,3277);  /*Q3*/
        FOR( i=0; i<Mbands_gn; i++ )
        {
            Ener_per_bd_iQ[i] = add(Ener_per_bd_iQ[i],shl(mean_gain,9)); /*Q12*/  move16();
        }

        st_fx->lp_gainc_fx = mult_r(st_fx->lp_gainc_fx,32112);  /*Q3*/
    }
    ELSE
    {
        mean_gain = gsc_gaindec_fx( st_fx, Ener_per_bd_iQ, st_fx->core_brate_fx, st_fx->old_y_gain_fx, coder_type, st_fx->bwidth_fx );

        st_fx->lp_gainc_fx  = mult_r(640,mean_gain);  /*10 in Q6 x Q12 -> lp_gainc in Q3 */
    }

    *last_bin = 0;
    move16();
    test();
    IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) == 0 && sub(st_fx->bwidth_fx,NB) != 0 )
    {
        bitallocation_exc[0] = 0;
        move16();
        bitallocation_exc[1] = 0;
        move16();
    }

    set16_fx( bitallocation_band, 0, MBANDS_GN );

    IF( sub(st_fx->bfi_fx,1) == 0 )
    {
        /*--------------------------------------------------------------------------------------*
         * Copy old spectrum
         * reduce spectral dynamic
         * save spectrum
         *--------------------------------------------------------------------------------------*/
        test();
        IF( sub(st_fx->last_good_fx,INACTIVE_CLAS) == 0 || sub(st_fx->Last_GSC_noisy_speech_flag_fx,1) == 0 )
        {
            FOR( i=0; i<L_FRAME; i++ )
            {
                L_tmp = L_shr(L_mult(Random(&st_fx->seed_tcx_fx),26214),5);  /*Q10*/
                L_tmp = L_mac(L_tmp, st_fx->Last_GSC_spectrum_fx[i],6554);
                st_fx->Last_GSC_spectrum_fx[i] = round_fx(L_tmp); /*Q10*/
            }
        }

        Copy( st_fx->Last_GSC_spectrum_fx, exc_diffQ, L_FRAME );

        FOR( i=0; i<L_FRAME; i++ )
        {
            st_fx->Last_GSC_spectrum_fx[i] = mult_r(st_fx->Last_GSC_spectrum_fx[i],24576); /*Q10*/  move16();
        }

    }
    ELSE
    {
        /*--------------------------------------------------------------------------------------*
         * PVQ decoder
         *--------------------------------------------------------------------------------------*/

        bands_and_bit_alloc_fx( st_fx->cor_strong_limit_fx, st_fx->noise_lev_fx, st_fx->core_brate_fx, Diff_len, bit, &bit, Ener_per_bd_iQ,
        max_ener_band, bits_per_bands, &nb_subbands, NULL, NULL, &pvq_len, coder_type, st_fx->bwidth_fx, st_fx->GSC_noisy_speech_fx );

        pvq_core_dec_fx( st_fx, gsc_sfm_start, gsc_sfm_end, gsc_sfm_size, concat_out, &Q_tmp, bit, nb_subbands, bits_per_bands, NULL, inpulses_fx, imaxpulse_fx, ACELP_CORE );
        Scale_sig(concat_out, gsc_sfm_end[nb_subbands-1], sub(Q_FPC_OUT, Q_tmp));

        /* Reorder Q bands */
        FOR(j = 0; j < nb_subbands; j++)
        {
            Copy( concat_out+j*16, exc_diffQ + max_ener_band[j]*16, 16);

            *last_bin = s_max(*last_bin,max_ener_band[j]);
            move16();

            bitallocation_band[max_ener_band[j]] = 1;
            move16();
        }
        test();
        IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) == 0 && sub(st_fx->bwidth_fx,NB) != 0 )
        {
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

        Copy( exc_diffQ, st_fx->Last_GSC_spectrum_fx, L_FRAME );

        /*--------------------------------------------------------------------------------------*
         * Skip adaptive (pitch) contribution frequency band (no noise added over the time contribution)
         * Find x pulses between 1.6-3.2kHz to code in the spectrum of the residual signal
         * Gain is based on the inter-correlation gain between the pulses found and residual signal
         *--------------------------------------------------------------------------------------*/
        freq_dnw_scaling_fx( st_fx->cor_strong_limit_fx, coder_type, st_fx->noise_lev_fx, st_fx->core_brate_fx, exc_diffQ, Qexc_diffQ );
    }

    /*--------------------------------------------------------------------------------------*
     * Estimate noise level
     *--------------------------------------------------------------------------------------*/

    highband_exc_dct_in_fx( st_fx->core_brate_fx, mfreq_bindiv_loc, *last_bin, Diff_len, st_fx->noise_lev_fx, pit_band_idx, exc_diffQ,
                            &st_fx->seed_tcx_fx, Ener_per_bd_iQ, nb_subfr, exc_dct_in, st_fx->last_coder_type_fx, bitallocation_band, lsf_new,
                            st_fx->last_exc_dct_in_fx, &st_fx->last_ener_fx, st_fx->last_bitallocation_band_fx, bitallocation_exc, st_fx->bfi_fx, coder_type,
                            st_fx->bwidth_fx, exc_wo_nf, Qexc_diffQ, Q_exc, st_fx->GSC_noisy_speech_fx );

    exc_dct_in[0] = 0;
    move16();

    return;

}
