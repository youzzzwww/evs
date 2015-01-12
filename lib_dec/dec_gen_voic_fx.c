/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"


/*======================================================================*/
/* FUNCTION : decod_gen_voic_fx()                                       */
/*----------------------------------------------------------------------*/
/* PURPOSE : Decode generic (GC), voiced (VC) and AMR-WB IO frames      */
/*                                                                      */
/*----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS :                                             */
/* _ (Struct)    st_fx            : decoder static memory               */
/* _ (Word16) L_frame_fx        : length of the frame                   */

/* _ (Word16[]) Aq_fx            : LP filter coefficient        Q12     */
/* _ (Word16) coder_type_fx        : coding type                Q12     */
/* _ (Word16) Es_pred_fx        : predicted scaled innov. energy Q8     */
/* _ (Word16[]) pitch_buf_fx    : floating pitch values for each subframe Q6*/
/* _ (Word16[])    voice_factors_fx: frame error rate           Q15     */
/*----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                   */
/* _ (Word16[]) exc_fx            : adapt. excitation exc (Q_exc)       */
/* _ (Word16[]) exc2_fx           : adapt. excitation/total exc (Q_exc) */
/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                   */
/* _ None                                                               */
/*======================================================================*/

void decod_gen_voic_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder static memory                     */
    const Word16 L_frame_fx,              /* i  : length of the frame                       */
    const Word16 sharpFlag_fx,            /* i  : formant sharpening flag                   */
    const Word16 *Aq_fx,                  /* i  : LP filter coefficient                     */
    const Word16 coder_type_fx,           /* i  : coding type                               */
    const Word16 Es_pred_fx,              /* i  : predicted scaled innov. energy            */
    const Word16 do_WI_fx,                /* i  : do interpolation after a FER              */
    Word16 *pitch_buf_fx,           /* o  : floating pitch values for each subframe   */
    Word16 *voice_factors_fx,       /* o  : voicing factors                           */
    Word16 *exc_fx,                 /* i/o: adapt. excitation exc                     */
    Word16 *exc2_fx,                /* i/o: adapt. excitation/total exc               */
    Word16 *bwe_exc_fx,             /* o  : excitation for SWB TBE                    */
    Word16 *unbits,                 /* number of unused bits                          */
    Word16 *gain_buf
)
{


    Word16 T0_fx, T0_frac_fx, T0_min_fx, T0_max_fx;/* integer pitch variables                     */
    Word16 gain_pit_fx = 0;               /* pitch gain               Q14                         */
    Word32 gain_code_fx=0;                /* gain/normalized gain of the algebraic excitation Q16 */
    Word32 norm_gain_code_fx=0;           /* normalized gain of the algebraic excitation    Q16   */
    Word16 gain_inov_fx=0;                /* Innovation gain  Q12                                 */
    Word32 gc_mem[NB_SUBFR-1];            /* gain_code from previous subframes                    */
    Word16 gp_mem[NB_SUBFR-1];            /* gain_pitch from previous subframes                   */
    Word16 voice_fac_fx;                  /* voicing factor  Q15                                  */
    Word16 code_fx[L_SUBFR];              /* algebraic codevector      Q12                        */

    const Word16 *p_Aq_fx;                /* Pointer to frame LP coefficient Q12                  */
    Word16 *pt_pitch_fx;                  /* pointer to floating pitch     Q6                     */
    Word16 i_subfr_fx, i;                 /* tmp variables                                        */
    Word16 error_fx = 0;
    Word16 gain_preQ_fx = 0;              /* Gain of prequantizer excitation                      */
    Word16 code_preQ_fx[L_SUBFR];         /* Prequantizer excitation                              */
    Word32 norm_gain_preQ_fx;
    Word16 pitch_limit_flag_fx;

    Word16 tmp1_fx,gain_code16;
    Word32 L_tmp_GC;
    Word32 L_tmp;

    Word16 harm_flag_acelp;

    Word16 shft_prev, ph_offset_fx;
    Word32 prev_res_nrg;
    Word32 prev_spch_nrg;
    Word32 curr_res_nrg;
    Word32 curr_spch_nrg;
    Word16 rint_bfi_pitch, rint_pitch;
    Word16 fraca, fracb, expa, expb, scale, exp1;
    Word16 *p_exc;
    Word16 mem_tmp_fx[M];
    Word16 syn_tmp_fx[L_FRAME16k];
    Word16 shft_curr;
    Word16 *p_syn;
    Word16 sp_enratio, Qsp_enratio;
    Word16 enratio, Qenratio;
    DTFS_STRUCTURE_FX *PREVP, *CURRP;
    Word16 S_fx[PIT_MAX*4+1], C_fx[PIT_MAX*4+1];
    Word16 dummy2[2];
    Word16 out_fx[L_FRAME16k];

    Word16 pf_temp1[MAXLAG_WI]; /*may not need more than MAXLAG_WI/2+1 */
    Word16 pf_temp2[MAXLAG_WI];
    Word16 pf_temp[MAXLAG_WI];
    Word16 pf_n2[MAXLAG_WI];



    T0_fx = PIT_MIN;
    move16();
    T0_frac_fx = 0;
    move16();

    /* read harmonicity flag */
    harm_flag_acelp = 0;
    move16();
    test();
    test();
    IF( (L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && L_sub(st_fx->core_brate_fx,ACELP_32k) <= 0) && sub(coder_type_fx,GENERIC) == 0 )
    {
        harm_flag_acelp = (Word16)get_next_indice_fx( st_fx, 1 );
    }

    /*------------------------------------------------------------------*
     * ACELP subframe loop
     *------------------------------------------------------------------*/

    p_Aq_fx = Aq_fx;
    move16();    /* pointer to interpolated LPC parameters */
    pt_pitch_fx = pitch_buf_fx;
    move16();    /* pointer to the pitch buffer */
    norm_gain_preQ_fx = 0;
    move16();
    gain_preQ_fx = 0;
    move16();
    set16_fx( code_preQ_fx, 0, L_SUBFR );

    FOR( i_subfr_fx = 0; i_subfr_fx < L_frame_fx; i_subfr_fx += L_SUBFR )
    {
        /*----------------------------------------------------------------------*
         * Decode pitch lag
         *----------------------------------------------------------------------*/

        *pt_pitch_fx = pit_decode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, i_subfr_fx, coder_type_fx, &pitch_limit_flag_fx,
                                      &T0_fx, &T0_frac_fx, &T0_min_fx, &T0_max_fx, L_SUBFR );
        move16(); /*Q6*/

        /*--------------------------------------------------------------*
         * Find the adaptive codebook vector
         *--------------------------------------------------------------*/

        pred_lt4( &exc_fx[i_subfr_fx], &exc_fx[i_subfr_fx], T0_fx, T0_frac_fx, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP );

        tbe_celp_exc( L_frame_fx,i_subfr_fx,T0_fx, T0_frac_fx, &error_fx, bwe_exc_fx );

        /*--------------------------------------------------------------*
         * LP filtering of the adaptive excitation
         *--------------------------------------------------------------*/

        lp_filt_exc_dec_fx( st_fx, MODE1, st_fx->core_brate_fx, 0, coder_type_fx, i_subfr_fx, L_SUBFR, L_frame_fx, 0, exc_fx );

        /*-----------------------------------------------------------------*
         * Transform-domain contribution decoding (active frames)
         *-----------------------------------------------------------------*/

        test();
        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 && sub(coder_type_fx,INACTIVE) != 0 )
        {
            gain_code_fx = 0;
            move16();
            transf_cdbk_dec_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, harm_flag_acelp, i_subfr_fx, -1, Es_pred_fx, gain_code_fx, &st_fx->mem_preemp_preQ_fx,
                                &gain_preQ_fx, &norm_gain_preQ_fx, code_preQ_fx, unbits );
        }

        /*--------------------------------------------------------------*
         * Innovation decoding
         *--------------------------------------------------------------*/

        inov_decode_fx( st_fx, st_fx->core_brate_fx, 0, L_frame_fx, coder_type_fx,
                        sharpFlag_fx, i_subfr_fx, -1, p_Aq_fx, st_fx->tilt_code_fx, *pt_pitch_fx, code_fx );

        /*--------------------------------------------------------------*
         * Gain decoding
         * Estimate spectrum tilt and voicing
         *--------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_8k00) <= 0)
        {
            gain_dec_lbr_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, i_subfr_fx, code_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx, gc_mem, gp_mem );
        }
        ELSE IF( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 )
        {
            gain_dec_SQ_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, i_subfr_fx, -1, code_fx, Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );
        }
        ELSE
        {
            gain_dec_mless_fx( st_fx, st_fx->core_brate_fx, L_frame_fx, coder_type_fx, i_subfr_fx, -1, code_fx, Es_pred_fx, &gain_pit_fx, &gain_code_fx, &gain_inov_fx, &norm_gain_code_fx );
        }
        st_fx->tilt_code_fx = est_tilt_fx( exc_fx+i_subfr_fx, gain_pit_fx, code_fx, gain_code_fx, &voice_fac_fx, st_fx->Q_exc );

        /*-----------------------------------------------------------------*
         * Transform domain contribution decoding
         *-----------------------------------------------------------------*/
        test();
        IF( L_sub( st_fx->core_brate_fx,ACELP_24k40) > 0 && sub(coder_type_fx,INACTIVE) == 0 )
        {
            transf_cdbk_dec_fx( st_fx, st_fx->core_brate_fx, coder_type_fx, harm_flag_acelp, i_subfr_fx, -1, Es_pred_fx, gain_code_fx, &st_fx->mem_preemp_preQ_fx,
                                &gain_preQ_fx, &norm_gain_preQ_fx, code_preQ_fx, unbits );
        }

        /* update LP filtered gains for the case of frame erasures */
        lp_gain_updt_fx( i_subfr_fx, gain_pit_fx, L_add(norm_gain_code_fx,norm_gain_preQ_fx), &st_fx->lp_gainp_fx, &st_fx->lp_gainc_fx, L_frame_fx );

        /*----------------------------------------------------------------------*
        * Find the total excitation
        *----------------------------------------------------------------------*/

        IF ( sub(L_frame_fx,L_FRAME) == 0 ) /* Rescaling for 12.8k core */
        {
            Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr_fx], &bwe_exc_fx[i_subfr_fx * HIBND_ACB_L_FAC], st_fx->last_exc_dct_in_fx,
                         L_SUBFR, L_SUBFR * HIBND_ACB_L_FAC, gain_code_fx, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, i_subfr_fx, coder_type_fx );

        }
        ELSE    /* Rescaling for 16k core */
        {
            L_tmp_GC = L_max(gain_code_fx, L_shl(gain_preQ_fx,16));   /* Chose the maximum of gain_code or the prequantizer excitation x4 to keep some room*/
            Rescale_exc( st_fx->dct_post_old_exc_fx, &exc_fx[i_subfr_fx], &bwe_exc_fx[i_subfr_fx * 2], st_fx->last_exc_dct_in_fx,
            L_SUBFR, L_SUBFR * 2, L_tmp_GC, &(st_fx->Q_exc), st_fx->Q_subfr, exc2_fx, i_subfr_fx, coder_type_fx );
        }

        gain_code16 = round_fx(L_shl(gain_code_fx,st_fx->Q_exc)); /*Q_exc*/

        /*-----------------------------------------------------------------*
         * Add the ACELP pre-quantizer contribution
         *-----------------------------------------------------------------*/

        IF( L_sub(st_fx->core_brate_fx,ACELP_24k40) > 0 )
        {
            tmp1_fx = add(15-Q_AVQ_OUT_DEC-2,st_fx->Q_exc);
            FOR( i = 0; i < L_SUBFR; i++ )
            {
                Word32 Ltmp1;
                /* Contribution from AVQ layer */
                Ltmp1 = L_mult(gain_preQ_fx, code_preQ_fx[i]);    /* Q2 + Q6 -> Q9*/
                Ltmp1 = L_shl(Ltmp1,tmp1_fx);                     /* Q16 + Q_exc */

                /* Compute exc2 */
                L_tmp = L_shl(L_mult(gain_pit_fx,exc_fx[i+i_subfr_fx]),1);
                exc2_fx[i+i_subfr_fx] = round_fx(L_add(L_tmp, Ltmp1));

                /* code in Q9, gain_pit in Q14 */
                L_tmp = L_mult(gain_code16, code_fx[i]);
                L_tmp = L_shl(L_tmp, 5);
                L_tmp = L_mac(L_tmp, exc_fx[i + i_subfr_fx], gain_pit_fx);
                L_tmp = L_shl(L_tmp, 1); /* saturation can occur here */

                exc_fx[i+i_subfr_fx] = round_fx(L_add(L_tmp, Ltmp1));
            }
        }
        ELSE
        {
            Acelp_dec_total_exc( exc_fx, exc2_fx, gain_code16, gain_pit_fx, i_subfr_fx, code_fx );
        }

        /*-----------------------------------------------------------------*
         * Prepare TBE excitation
         *-----------------------------------------------------------------*/

        prep_tbe_exc_fx( L_frame_fx, i_subfr_fx, gain_pit_fx, gain_code_fx, code_fx, voice_fac_fx,
                         &voice_factors_fx[i_subfr_fx/L_SUBFR], bwe_exc_fx, gain_preQ_fx, code_preQ_fx,
                         st_fx->Q_exc, T0_fx, T0_frac_fx, coder_type_fx, st_fx->core_brate_fx );


        /*----------------------------------------------------------------*
         * Excitation enhancements (update of total excitation signal)
         *----------------------------------------------------------------*/

        test();
        IF( L_sub(st_fx->core_brate_fx,ACELP_32k) > 0 || sub(coder_type_fx,INACTIVE) == 0 )
        {
            Copy( exc_fx+i_subfr_fx, exc2_fx+i_subfr_fx, L_SUBFR );
        }
        ELSE
        {
            enhancer_fx( st_fx->core_brate_fx, 0, coder_type_fx, i_subfr_fx, L_frame_fx, voice_fac_fx, st_fx->stab_fac_fx,
            norm_gain_code_fx, gain_inov_fx, &st_fx->gc_threshold_fx, code_fx, exc2_fx, gain_pit_fx, &(st_fx->dm_fx), st_fx->Q_exc );
        }

        p_Aq_fx += (M+1);
        move16();
        pt_pitch_fx++;
        gain_buf[i_subfr_fx/L_SUBFR] = gain_pit_fx;
        move16();
    }

    /* FEC fast recovery */

    IF( do_WI_fx )
    {
        /* shft_prev = L_EXC_MEM - rint_new_fx(st_fx->bfi_pitch_fx);*/
        L_tmp = L_shl(L_deposit_l(st_fx->bfi_pitch_fx),10); /*Q16*/
        rint_bfi_pitch = rint_new_fx(L_tmp);                /*Q0*/
        shft_prev = sub( L_EXC_MEM, rint_bfi_pitch );       /*Q0*/

        p_exc = st_fx->old_exc2_fx + shft_prev;
        move16();
        p_syn =  st_fx->old_syn2_fx + shft_prev;
        move16();

        prev_res_nrg = L_deposit_l(1);
        prev_spch_nrg = L_deposit_l(1);
        FOR( i=0; i<rint_bfi_pitch; i++ )
        {
            prev_res_nrg  = L_mac0(prev_res_nrg,  *p_exc, *p_exc); /* 2*st_fx->prev_Q_exc_fr */
            prev_spch_nrg = L_mac0(prev_spch_nrg, *p_syn, *p_syn); /* 2*st_fx->prev_Q_syn_fr */
            p_exc++;
            p_syn++;
        }

        Copy( st_fx->mem_syn2_fx, mem_tmp_fx, M );

        syn_12k8_fx( st_fx->L_frame_fx, Aq_fx, exc2_fx, syn_tmp_fx, mem_tmp_fx, 1, st_fx->Q_exc, st_fx->Q_syn );

        L_tmp = L_shl(L_deposit_l(pitch_buf_fx[NB_SUBFR16k-1]),10);   /*Q16*/
        rint_pitch = rint_new_fx(L_tmp);                              /*Q0*/
        shft_curr = sub( st_fx->L_frame_fx, rint_pitch );             /*Q0*/

        p_exc = exc2_fx + shft_curr;
        move16();
        p_syn = syn_tmp_fx + shft_curr;
        move16();

        curr_res_nrg = L_deposit_l(1);
        curr_spch_nrg = L_deposit_l(1);
        FOR( i=0; i<rint_pitch; i++ )
        {
            curr_res_nrg  = L_mac0(curr_res_nrg,  *p_exc, *p_exc); /* 2*st_fx->Q_exc */
            curr_spch_nrg = L_mac0(curr_spch_nrg, *p_syn, *p_syn); /* 2*st_fx->Q_syn */
            p_exc++;
            p_syn++;
        }

        /* enratio = (curr_res_nrg / prev_res_nrg); */
        IF(prev_res_nrg>0)
        {
            expa = norm_l(prev_res_nrg);
            fraca = extract_h(L_shl(prev_res_nrg,expa));
            expa = sub(30,expa+(2*st_fx->prev_Q_exc_fr));

            expb = norm_l(curr_res_nrg);
            fracb = round_fx(L_shl(curr_res_nrg,expb));
            expb =  sub(30,expb+(2*st_fx->Q_exc));

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            enratio = div_s(fracb,fraca);
            exp1 = sub(expb,expa);
            Qenratio = 15-exp1;
        }
        ELSE
        {
            enratio = 0;
            Qenratio = 0;
        }

        /* sp_enratio = curr_spch_nrg/prev_spch_nrg */
        IF(prev_spch_nrg>0)
        {
            expa = norm_l(prev_spch_nrg);
            fraca = extract_h(L_shl(prev_spch_nrg,expa));
            expa = sub(30,expa+(2*st_fx->prev_Q_syn_fr));

            expb = norm_l(curr_spch_nrg);
            fracb = round_fx(L_shl(curr_spch_nrg,expb));
            expb =  sub(30,expb+(2*st_fx->Q_syn));

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            sp_enratio = div_s(fracb,fraca);
            exp1 = sub(expb,expa);
            Qsp_enratio = 15-exp1;
        }
        ELSE
        {
            sp_enratio = 0;
            Qsp_enratio = 0;
        }

        test();
        test();
        test();
        test();
        IF ( sub(shl_r(enratio,15-Qenratio), 8192) > 0 &&      /*compare with 0.25 in Q15*/
             sub(shl_r(enratio,10-Qenratio), 15360) < 0 &&     /*compare with 15.0 in Q10*/
             sub(shl_r(sp_enratio,15-Qsp_enratio), 4915) > 0 &&/*compare with 0.15 in Q15*/
             st_fx->bfi_pitch_fx < 9600  &&                    /*Q6*/
             pitch_buf_fx[ sub( NB_SUBFR16k, 1 ) ] < 9600 )    /*Q6*/
        {
            PREVP = DTFS_new_fx();
            CURRP = DTFS_new_fx();

            GetSinCosTab_fx(rint_bfi_pitch,S_fx,C_fx);
            DTFS_to_fs_fx( st_fx->old_exc2_fx + shft_prev, rint_bfi_pitch, PREVP, (Word16)st_fx->output_Fs_fx, do_WI_fx, S_fx, C_fx);
            PREVP->Q = add(PREVP->Q, st_fx->prev_Q_exc_fr);

            GetSinCosTab_fx(rint_pitch,S_fx,C_fx);
            DTFS_to_fs_fx( exc2_fx + shft_curr, rint_pitch, CURRP,  (Word16)st_fx->output_Fs_fx, do_WI_fx, S_fx, C_fx);
            CURRP->Q = add(CURRP->Q, st_fx->Q_exc);

            ph_offset_fx = 0;
            move16();
            WIsyn_fx(*PREVP, CURRP, dummy2, &(ph_offset_fx), out_fx, (Word16)st_fx->L_frame_fx, 1, S_fx, C_fx, pf_temp1, pf_temp2, pf_temp, pf_n2);


            Copy_Scale_sig( out_fx, exc2_fx, st_fx->L_frame_fx, st_fx->Q_exc );
            Copy_Scale_sig( out_fx, exc_fx,  st_fx->L_frame_fx, st_fx->Q_exc );

            /* update bwe_exc for SWB-TBE */
            FOR (i_subfr_fx = 0; i_subfr_fx < L_frame_fx; i_subfr_fx += L_SUBFR)
            {
                interp_code_4over2_fx( exc_fx + i_subfr_fx, bwe_exc_fx + (i_subfr_fx*2), L_SUBFR );
            }

            free(PREVP);
            free(CURRP);
        }

    }

    /* SC-VBR */
    st_fx->prev_gain_pit_dec_fx = gain_pit_fx;
    move16(); /*Q14*/
    st_fx->prev_tilt_code_dec_fx = st_fx->tilt_code_fx;
    move16(); /*Q15*/

    return;
}
