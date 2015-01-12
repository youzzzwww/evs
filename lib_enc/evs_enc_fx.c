/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"
#include "cnst_fx.h"        /* Common constants                       */
#include "stl.h"            /* Debug prototypes                       */
#include "rom_com_fx.h"     /* Common constants                       */


static void writeFrameHeader_loc( Encoder_State_fx *st );
static void configure_core_coder_loc( Encoder_State_fx *st, Word16  *coder_type,const Word16 localVAD);

/*-------------------------------------------------------------------*
 * evs_enc()
 *
 * Principal encoder routine
 *-------------------------------------------------------------------*/

void evs_enc_fx(
    Encoder_State_fx *st,                               /* i/o: encoder state structure  */
    const Word16 *data,                               /* i  : input signal             */
    const Word16 n_samples                            /* i  : number of input samples  */
)
{
    Word16 i, input_frame, delay;
    Word16 old_inp_12k8[L_INP_12k8], *inp = 0;        /* buffer of input signal @ 12k8            */
    Word16 old_inp_16k[L_INP];                        /* buffer of input signal @ 16kHz           */
    Word16 sp_aud_decision1;                          /* 1st stage speech/music classification    */
    Word16 sp_aud_decision2;                          /* 2nd stage speech/music classification    */
    Word32 fr_bands[2*NB_BANDS];                      /* energy in frequency bands                */
    Word16 vad_flag;
    Word16 localVAD;
    Word16 Etot;                                      /* total energy; correlation shift          */
    Word32 ener;                                      /* residual energy from Levinson-Durbin     */
    Word16 pitch[3];                                  /* open-loop pitch values for quantization  */
    Word16 voicing[3];                                /* OL maximum normalized correlation        */
    Word16 A[NB_SUBFR16k*(M+1)];                      /* A(z) unquantized for subframes           */
    Word16 Aw[NB_SUBFR16k*(M+1)];                     /* weighted A(z) unquantized for subframes  */
    Word16 epsP_h[M+1];                               /* LP prediction errors                     */
    Word16 epsP_l[M+1];                               /* LP prediction errors                     */
    Word32 epsP[M+1];                                 /* LP prediction errors                     */
    Word16 lsp_new[M];                                /* LSPs at the end of the frame             */
    Word16 lsp_mid[M];                                /* ISPs in the middle of the frame          */
    Word16 coder_type;                                /* coder type                               */
    Word16 sharpFlag;                                 /* formant sharpening flag                  */
    Word16 vad_hover_flag;
    Word16 hq_core_type;                              /* HQ core type (HQ, or LR-MDCT)            */

    Word16 attack_flag;                               /* flag signalling attack encoded by the AC mode (GSC) */
    Word16 new_inp_resamp16k[L_FRAME16k];             /* new input signal @16kHz, non pre-emphasised, used by the WB TBE/BWE */
    Word16 old_syn_12k8_16k[L_FRAME16k];              /* ACELP core synthesis at 12.8kHz or 16kHz to be used by the SWB BWE */
    Word16 shb_speech[L_FRAME16k];
    Word16 hb_speech[L_FRAME16k/4];
    Word16 new_swb_speech[L_FRAME48k];
    Word32 bwe_exc_extended[L_FRAME32k + NL_BUFF_OFFSET];   /* non-linear bandwidth extension buffer */
    Word16 voice_factors[NB_SUBFR16k];
    Word16 fb_exc[L_FRAME16k];
    Word16 Voicing_flag;
    Word16 pitch_buf[NB_SUBFR16k];
    Word16 unbits;

    Word16 padBits;
    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX]; /* real buffer */
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX]; /* imag buffer */
    CLDFB_SCALE_FACTOR cldfbScale;
    Word16 Q_new, shift, Q_synth;
    Word16 Q_r[2];
    Word16 Q_shb_spch, Q_fb_exc;
    Word32 L_tmp;
    UWord16 lsb;
    Word16 tmp;

    Q_shb_spch = 0;
    move16();       /* to avoid compiler warnings */




    /*------------------------------------------------------------------*
     * Initializiation
     *-----------------------------------------------------------------*/

    input_frame = st->input_frame_fx;
    move16();
    st->core_fx = -1;
    move16();
    st->extl_fx = -1;
    move16();
    st->core_brate_fx = -1;
    move32();
    st->input_bwidth_fx = st->last_input_bwidth_fx;
    move16();
    st->bwidth_fx = st->last_bwidth_fx;
    move16();
    hq_core_type = -1;
    move16();
    unbits = 0;
    move16();

    st->bits_frame_core = 0;
    move16(); /* For getting bit consumption in core coder */
    st->lp_cng_mode2 = 0;
    move16();
    st->mdct_sw_enable = 0;
    move16();
    st->mdct_sw = 0;
    move16();
    shift = st->old_wsp_shift;
    move16();
    st->rate_switching_reset = 0;
    move16();

    /*----------------------------------------------------------------*
     * set input samples buffer
     *----------------------------------------------------------------*/

    /* get delay to synchronize ACELP and MDCT frame */
    delay = NS2SA_fx2(st->input_Fs_fx, DELAY_FIR_RESAMPL_NS);

    Copy( st->input - delay, st->old_input_signal_fx, input_frame+delay );

    /*----------------------------------------------------------------*
     * convert 'short' input data to 'float'
     *----------------------------------------------------------------*/

    Copy(data, st->input, input_frame);
    IF( sub(n_samples,input_frame) < 0)
    {
        set16_fx( st->input + n_samples, 0, sub(input_frame, n_samples)  );
    }

    /*----------------------------------------------------------------*
     * HP filtering
     *----------------------------------------------------------------*/

    hp20( st->input, 1, input_frame, st->mem_hp20_in_fx, st->input_Fs_fx );

    /*----------------------------------------------------------------*
     * Updates in case of AMR-WB IO mode -> EVS primary mode switching
     *----------------------------------------------------------------*/

    IF( sub(st->last_core_fx, AMR_WB_CORE) == 0 )
    {
        updt_IO_switch_enc_fx( st, input_frame );
        set16_fx(st->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k);
        cldfb_reset_memory( st->cldfbAna_Fx );
        cldfb_reset_memory( st->cldfbSyn_Fx );
    }

    /*---------------------------------------------------------------------*
     * Pre-processing
     *---------------------------------------------------------------------*/

    pre_proc_fx( st, input_frame, st->input, old_inp_12k8, old_inp_16k, &inp, &sp_aud_decision1,
                 &sp_aud_decision2, fr_bands, &vad_flag, &localVAD, &Etot, &ener, pitch, voicing,
                 A, Aw, epsP_h, epsP_l, epsP, lsp_new, lsp_mid, &coder_type, &sharpFlag, &vad_hover_flag,
                 &attack_flag, new_inp_resamp16k, &Voicing_flag, realBuffer, imagBuffer, &cldfbScale, st->LPDmem.old_exc,
                 &hq_core_type,
                 &Q_new, &shift, Q_r );

    st->sharpFlag = sharpFlag;

    IF (sub(st->mdct_sw,MODE2) == 0)
    {
        writeFrameHeader_loc( st );

        test();
        test();
        test();
        IF ((L_sub(st->total_brate_fx,ACELP_24k40) > 0 && L_sub(st->total_brate_fx,HQ_96k) < 0) || (L_sub(st->total_brate_fx,ACELP_24k40) == 0 && sub(st->bwidth_fx,WB) >= 0))
        {
            st->L_frame_fx = L_FRAME16k;
            move16();
            st->gamma = GAMMA16k;
            move16();
            st->preemph_fac = PREEMPH_FAC_16k;
            move16();

            weight_a_subfr_fx( NB_SUBFR16k, A, Aw, GAMMA16k, M );
            test();
            IF (sub(st->last_L_frame_fx,L_FRAME) == 0 && st->ini_frame_fx != 0)
            {
                /* this is just an approximation, but it is sufficient */
                Copy( st->lsp_old1_fx, st->lspold_enc_fx, M );
            }
        }
        ELSE
        {
            st->L_frame_fx = L_FRAME;
            move16();
            st->gamma = GAMMA1;
            move16();
            st->preemph_fac = PREEMPH_FAC;
            move16();
        }

        st->sr_core = L_mult0(50,st->L_frame_fx);
        st->core_brate_fx = st->total_brate_fx;
        move32();

        st->igf = 0;
        move16();
        hq_core_type = NORMAL_HQ_CORE;
        move16();
        test();
        test();
        IF( (sub(st->bwidth_fx,SWB) == 0 || sub(st->bwidth_fx,WB) == 0) && L_sub(st->total_brate_fx,LRMDCT_CROSSOVER_POINT) <= 0 )
        {
            /* note that FB (bit-rate >= 24400bps) is always coded with NORMAL_HQ_CORE */
            hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
        ELSE IF( sub(st->bwidth_fx,NB) == 0 )
        {
            hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
    }

    IF( sub(st->codec_mode,MODE1) == 0 )
    {
        /*---------------------------------------------------------------------*
         * Write signalling info into the bitstream
         *---------------------------------------------------------------------*/
        signalling_enc_fx( st, coder_type, sharpFlag );

        /*---------------------------------------------------------------------*
         * Preprocessing (preparing) for ACELP/HQ core switching
         *---------------------------------------------------------------------*/

        core_switching_pre_enc_fx( st,&(st->LPDmem), input_frame, old_inp_12k8, old_inp_16k);

        /*---------------------------------------------------------------------*
         * ACELP core encoding
         *---------------------------------------------------------------------*/
        IF( sub(st->core_fx,ACELP_CORE) == 0 )
        {
            acelp_core_enc_fx( st, &(st->LPDmem), inp, vad_flag, ener,
                               pitch, voicing, A, Aw, epsP_h, epsP_l, lsp_new, lsp_mid, coder_type, sharpFlag, vad_hover_flag,
                               attack_flag, bwe_exc_extended, voice_factors, old_syn_12k8_16k, pitch_buf, &unbits, Q_new, shift );
        }

        /*---------------------------------------------------------------------*
         * HQ core encoding
         *---------------------------------------------------------------------*/

        IF( sub(st->core_fx,HQ_CORE) == 0 )
        {
            hq_core_enc_fx( st, st->input - delay, input_frame, hq_core_type, Voicing_flag);
        }

        /*---------------------------------------------------------------------*
         * Postprocessing for ACELP/HQ core switching
         *---------------------------------------------------------------------*/
        core_switching_post_enc_fx( st, old_inp_12k8, old_inp_16k, pitch, voicing,
                                    A, shift, Q_new, st->Q_syn2, &Q_synth );
    }
    ELSE   /* MODE2 */
    {
        /*----------------------------------------------------------------*
         * Configuration of core coder/SID
         * Write Frame Header
         *----------------------------------------------------------------*/

        configure_core_coder_loc( st, &coder_type, localVAD );

        IF (st->mdct_sw != MODE1)
        {
            writeFrameHeader_loc( st );
        }

        /*----------------------------------------------------------------*
         * Core-Coder
         *----------------------------------------------------------------*/

        /* Call main encoding function */
        enc_acelp_tcx_main( old_inp_16k + L_INP_MEM, st, coder_type, pitch, voicing, Aw, lsp_new, lsp_mid,
        st->hFdCngEnc_fx, bwe_exc_extended, voice_factors, pitch_buf
        , vad_hover_flag, &Q_new, &shift );

        /*---------------------------------------------------------------------*
         * Postprocessing for codec switching
         *---------------------------------------------------------------------*/
        /* TBE interface */
        test();
        IF ( st->igf != 0 && L_sub(st->core_brate_fx,SID_2k40) > 0 )
        {
            IF( sub(st->core_fx,ACELP_CORE) == 0 )
            {
                SWITCH (st->bwidth_fx)
                {
                case WB:
                    st->extl_fx = WB_TBE;
                    move16();
                    st->extl_brate_fx = WB_TBE_0k35;
                    move32();
                    BREAK;

                case SWB:
                    st->extl_fx = SWB_TBE;
                    move16();
                    st->extl_brate_fx = SWB_TBE_1k6;
                    move32();
                    BREAK;

                case FB:
                    st->extl_fx = FB_TBE;
                    move16();
                    st->extl_brate_fx = FB_TBE_1k8;
                    move32();
                    BREAK;
                }
            }
            ELSE
            {
                coder_type = -1;
                move16();
                st->extl_fx = IGF_BWE;
                move16();
                st->extl_brate_fx = 0;
                move32();
            }

            st->core_brate_fx = L_sub(st->total_brate_fx, st->extl_brate_fx);

            IF( sub(st->tec_tfa, 1) == 0 )
            {
                st->core_brate_fx = L_sub(st->core_brate_fx, BITS_TEC);
                st->core_brate_fx = L_sub(st->core_brate_fx, BITS_TFA);
            }
        }

        /*----------------------------------------------------------------*
         * Complete Bitstream Writing
         *----------------------------------------------------------------*/

        padBits = 0;

        test();
        test();
        IF( st->igf != 0 && sub(st->core_fx,ACELP_CORE) == 0 && L_sub(st->core_brate_fx,SID_2k40) > 0 )
        {
            /* padBits = ((st->bits_frame+7)/8)*8 - (st->nb_bits_tot + (st->rf_target_bits_write - ((st->Opt_RF_ON==1)?1:0) ) + get_tbe_bits(st->total_brate, st->bwidth, st->rf_mode )); */
            tmp = add(get_tbe_bits_fx(st->total_brate_fx, st->bwidth_fx, st->rf_mode), sub(st->rf_target_bits_write, st->Opt_RF_ON));
            padBits = sub(sub(shl(shr(add(st->bits_frame,7),3),3), st->nb_bits_tot_fx), tmp);
        }
        ELSE
        {
            /* padBits = ((st->bits_frame+7)/8)*8 - (st->nb_bits_tot + (st->rf_target_bits_write - ((st->Opt_RF_ON==1)?1:0) )); */
            tmp = sub(st->rf_target_bits_write, st->Opt_RF_ON);
            padBits = sub(shl(shr(add(st->bits_frame,7),3),3), add(st->nb_bits_tot_fx, tmp));
        }

        FOR( i = 0; i<padBits; i++ )
        {
            push_next_indice_fx(st, 0, 1);
        }

    }

    /*---------------------------------------------------------------------*
     * WB TBE encoding
     * WB BWE encoding
     *---------------------------------------------------------------------*/

    test();
    IF ( L_sub(st->input_Fs_fx,16000 ) >= 0 && (sub(st->bwidth_fx, SWB) < 0) )
    {
        /* Common pre-processing for WB TBE and WB BWE */
        wb_pre_proc_fx( st, new_inp_resamp16k, hb_speech );
        /* o: new_inp_resamp16k at Q = -1 */
    }

    IF ( sub(st->extl_fx,WB_TBE) == 0 )
    {
        /* WB TBE encoder */
        wb_tbe_enc_fx( st, coder_type, hb_speech, bwe_exc_extended, Q_new, voice_factors, pitch_buf, voicing);

        IF( sub(st->codec_mode,MODE2) == 0 )
        {
            tbe_write_bitstream_fx( st );
        }
    }
    ELSE IF ( sub(st->extl_fx, WB_BWE) == 0 )
    {
        /* WB BWE encoder */
        wb_bwe_enc_fx( st, new_inp_resamp16k, coder_type );

    }

    /*---------------------------------------------------------------------*
     * SWB(FB) TBE encoding
     * SWB BWE encoding
     *---------------------------------------------------------------------*/
    test();
    IF (!st->Opt_SC_VBR_fx && L_sub(st->input_Fs_fx,32000) >= 0 )
    {
        /* Common pre-processing for SWB(FB) TBE and SWB BWE */
        swb_pre_proc_fx(st, st->input, new_swb_speech, shb_speech, &Q_shb_spch, realBuffer, imagBuffer, &cldfbScale);
    }

    /* SWB TBE encoder */
    test();
    test();
    test();
    test();
    test();
    test();
    IF ( sub(st->extl_fx, SWB_TBE) == 0 || sub(st->extl_fx, FB_TBE) == 0 || ( st->igf != 0 && sub(st->core_fx, ACELP_CORE) == 0 && sub(st->extl_fx, WB_TBE) != 0 ) )
    {
        test();
        IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st->core_brate_fx,SID_2k40) != 0 )
        {
            swb_tbe_enc_fx( st, coder_type, shb_speech, bwe_exc_extended, voice_factors, fb_exc, &Q_fb_exc, Q_new, Q_shb_spch, voicing, pitch_buf );

            IF ( sub(st->extl_fx,FB_TBE) == 0 )
            {
                /* FB TBE encoder */
                fb_tbe_enc_fx( st, st->input, fb_exc, Q_fb_exc );
            }

            IF( sub(st->codec_mode,MODE2) == 0 )
            {
                IF( sub(st->tec_tfa, 1) == 0 )
                {
                    tecEnc_TBE_fx(&st->tecEnc.corrFlag, voicing, coder_type);

                    IF( sub(coder_type, INACTIVE) == 0 )
                    {
                        st->tec_flag = 0;
                        move16();
                        st->tecEnc.corrFlag = 0;
                        move16();
                    }
                    st->tfa_flag = tfaEnc_TBE_fx( st->tfa_enr, st->last_core_fx, voicing, pitch_buf, shl(Q_shb_spch, 1));
                    set_TEC_TFA_code_fx( st->tecEnc.corrFlag, &st->tec_flag, &st->tfa_flag );
                }
                ELSE
                {
                    st->tec_flag = 0;
                    move16();
                    st->tecEnc.corrFlag = 0;
                    move16();
                    st->tfa_flag = 0;
                    move16();
                }

                tbe_write_bitstream_fx( st );
            }
        }
    }
    ELSE IF ( sub(st->extl_fx,SWB_BWE) == 0 || sub(st->extl_fx,FB_BWE) == 0 )
    {
        /* SWB BWE encoder */
        swb_bwe_enc_fx( st, old_inp_12k8, old_inp_16k, old_syn_12k8_16k, new_swb_speech, shb_speech, coder_type, Q_shb_spch, sub(Q_new, 1) );
    }
    ELSE IF( sub(st->extl_fx,SWB_BWE_HIGHRATE) == 0 || sub(st->extl_fx,FB_BWE_HIGHRATE) == 0 )
    {
        /* SWB HR BWE encoder */
        swb_bwe_enc_hr_fx(st, st->input - delay, st->Q_syn2, input_frame, coder_type, unbits );
    }

    /*---------------------------------------------------------------------*
     * SWB DTX/CNG encoding
     *---------------------------------------------------------------------*/

    test();
    IF ( st->Opt_DTX_ON_fx && sub(input_frame,L_FRAME32k) >= 0 )
    {
        swb_CNG_enc_fx( st, shb_speech, old_syn_12k8_16k );
    }



    /*---------------------------------------------------------------------*
     * Channel-aware mode - write signaling information into the bit-stream
     *---------------------------------------------------------------------*/
    signalling_enc_rf( st );

    /*---------------------------------------------------------------------*
     * Updates - MODE1
     *---------------------------------------------------------------------*/

    st->last_sr_core = st->sr_core;
    move16();
    st->last_codec_mode = st->codec_mode;
    move16();
    st->last_L_frame_fx = st->L_frame_fx;
    move16();
    st->last_core_fx = st->core_fx;
    move16();

    st->last_total_brate_fx = st->total_brate_fx;
    move32();
    st->last_core_brate_fx = st->core_brate_fx;
    move32();
    st->last_extl_fx = st->extl_fx;
    move16();
    st->last_input_bwidth_fx = st->input_bwidth_fx;
    move16();
    st->last_bwidth_fx = st->bwidth_fx;
    move16();
    st->Etot_last_fx = Etot;
    move16();
    st->last_coder_type_raw_fx = st->coder_type_raw_fx;
    move16();

    st->prev_Q_new = Q_new;

    if( L_sub(st->core_brate_fx,SID_2k40) > 0 )
    {
        st->last_active_brate_fx = st->total_brate_fx;
        move32();
    }
    IF ( sub(st->core_fx,HQ_CORE) == 0 )
    {
        /* in the HQ core, coder_type is not used so it could have been set to anything */
        st->prev_coder_type_fx = GENERIC;
    }
    ELSE
    {
        st->prev_coder_type_fx = coder_type;
    }

    test();
    IF( L_sub(st->core_brate_fx,SID_2k40) > 0 && sub(st->first_CNG_fx,1) == 0 )
    {
        if( sub(st->act_cnt_fx,BUF_DEC_RATE) >= 0 )
        {
            st->act_cnt_fx = 0;
            move16();
        }

        st->act_cnt_fx = add(st->act_cnt_fx,1);

        test();
        if( sub(st->act_cnt_fx,BUF_DEC_RATE) == 0 && st->ho_hist_size_fx > 0 )
        {
            st->ho_hist_size_fx = sub(st->ho_hist_size_fx,1);
        }

        st->act_cnt2_fx = add(st->act_cnt2_fx,1);
        if( sub(st->act_cnt2_fx,MIN_ACT_CNG_UPD) >= 0 )
        {
            st->act_cnt2_fx = MIN_ACT_CNG_UPD;
            move16();
        }
    }

    test();
    test();
    if ( L_sub(st->core_brate_fx,SID_2k40) <= 0 && st->first_CNG_fx == 0 && sub(st->cng_type_fx,LP_CNG) == 0 )
    {
        st->first_CNG_fx = 1;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Increase the counter of initialization frames
     * Limit the max number of init. frames
     *-----------------------------------------------------------------*/

    if( sub(st->ini_frame_fx,MAX_FRAME_COUNTER) < 0 )
    {
        st->ini_frame_fx = add(st->ini_frame_fx, 1);
    }

    /* synchronisation of CNG seeds */
    test();
    IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 && L_sub(st->core_brate_fx, SID_2k40) != 0 )
    {
        Random( &(st->cng_seed_fx) );
        Random( &(st->cng_ener_seed_fx) );
    }

    /*---------------------------------------------------------------------*
     * Updates - MODE2
     *---------------------------------------------------------------------*/
    IF( sub(st->mdct_sw,MODE2) == 0 )
    {
        st->codec_mode = MODE2;
        move16();
        st->sr_core = getCoreSamplerateMode2( st->total_brate_fx, st->bwidth_fx, st->rf_mode);
        Mpy_32_16_ss(st->sr_core, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
        st->L_frame_fx = extract_l(L_shr(L_tmp, 3)); /* Q0 */
        assert(st->L_frame_fx == st->sr_core / 50);
        IF ( L_sub(st->sr_core,12800) == 0 )
        {
            st->preemph_fac = PREEMPH_FAC;
            move16();
            st->gamma = GAMMA1;
            move16();
        }
        ELSE
        {
            st->preemph_fac = PREEMPH_FAC_16k;
            move16();
            st->gamma = GAMMA16k;
            move16();
        }

        st->igf = getIgfPresent(st->total_brate_fx, st->bwidth_fx, st->rf_mode);
    }

    /* update FER clas */
    st->last_clas_fx = st->clas_fx;

    core_encode_update( st );
    if( sub(st->mdct_sw,MODE1) == 0 )
    {
        st->codec_mode = MODE1;
        move16();
    }
    if( st->lp_cng_mode2 )
    {
        st->codec_mode = MODE2;
        move16();
    }



    st->rf_mode_last = st->rf_mode;
    IF(sub(st->Opt_RF_ON,1)==0)
    {
        st->L_frame_fx = L_FRAME;
        st->rf_mode = 1;
    }

    return;
}



/*-------------------------------------------------------------------*
 * writeFrameHeader()
 *
 * Write MODE2 frame header
 *-------------------------------------------------------------------*/
static void writeFrameHeader_loc( Encoder_State_fx *st )
{

    IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 )
    {
        /* SID flag at 2.4kbps */
        IF( L_sub(st->core_brate_fx,SID_2k40) == 0 )
        {
            IF ( sub(st->cng_type_fx,FD_CNG) == 0 )
            {
                /* write SID/CNG type flag */
                push_next_indice_fx( st, 1, 1 );

                /* write bandwidth mode */
                push_next_indice_fx( st, st->bwidth_fx, 2 );

                /* write L_frame */
                IF( sub(st->L_frame_fx,L_FRAME) == 0 )
                {
                    push_next_indice_fx( st, 0, 1 );
                }
                ELSE
                {
                    push_next_indice_fx( st, 1, 1 );
                }
            }
        }
        ELSE /* active frames */
        {
            IF(st->rf_mode == 0)
            {
                push_next_indice_fx( st, sub(st->bwidth_fx,FrameSizeConfig[st->frame_size_index].bandwidth_min), FrameSizeConfig[st->frame_size_index].bandwidth_bits);
            }
        }

        /* Write reserved bit */
        test();
        IF( FrameSizeConfig[st->frame_size_index].reserved_bits && st->rf_mode == 0)
        {
            push_next_indice_fx( st, 0, FrameSizeConfig[st->frame_size_index].reserved_bits );
        }
    }

    return;
}

/*------------------------------------------------------------------------*
* Configuration of core coder/SID
*------------------------------------------------------------------------*/

static void configure_core_coder_loc(
    Encoder_State_fx *st,            /* i/o: encoder state structure         */
    Word16 *coder_type,    /* i  : coder type                      */
    const Word16 localVAD
)
{
    Word16 n;
    IF( L_sub(st->core_brate_fx, SID_2k40) == 0 )
    {
        /*Get size of frame*/
        st->bits_frame       = FRAME_2_4;
        move16();
        st->bits_frame_core = add(st->bits_frame_core, FRAME_2_4-4);     /*1 bit for SID on/off + 3 bits for bandwith in case of SID.*/
        st->frame_size_index = 2;
        move16();
    }
    ELSE IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        st->bits_frame       = FRAME_0;
        move16();
        st->bits_frame_core = add(st->bits_frame_core,st->bits_frame);
        st->frame_size_index = 0;
        move16();
    }
    ELSE
    {
        FOR( n=0; n<FRAME_SIZE_NB; n++ )
        {
            IF( sub(FrameSizeConfig[n].frame_bits,st->bits_frame_nominal) == 0 )
            {
                st->frame_size_index = n;
                move16();
                st->bits_frame = FrameSizeConfig[n].frame_bits;
                move16();
                st->bits_frame_core = FrameSizeConfig[n].frame_net_bits;
                move16();
                BREAK;
            }
        }

        if( st->tcxonly )
        {
            *coder_type = GENERIC;
            move16();
        }

        st->tcx_cfg.coder_type = *coder_type;
        move16();


        test();
        test();
        if( !st->tcxonly && !localVAD && sub(st->tcx_cfg.coder_type,GENERIC) == 0 )
        {
            st->tcx_cfg.coder_type = UNVOICED;
            move16();
        }
    }

    st->igf = getIgfPresent(st->total_brate_fx, st->bwidth_fx, st->rf_mode);

    test();
    if( L_sub(st->core_brate_fx,SID_2k40) != 0 && L_sub(st->core_brate_fx,FRAME_NO_DATA) != 0 )
    {
        st->core_brate_fx = st->total_brate_fx;
        move32();
    }

    return;
}
