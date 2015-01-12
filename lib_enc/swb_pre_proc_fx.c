/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "basop_util.h"

#include <assert.h>

/*========================================================================*/
/* FUNCTION :  wb_pre_proc_fx()											  */
/*------------------------------------------------------------------------*/
/* PURPOSE :  Resampling of input signal when input signal sample rate    */
/*              is above 16kHz 				                              */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* Encoder_State_fx *st_fx   : Encoder State Structure	     			  */
/* _ (Word16*) input         : original input signal                 	  */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _None                                                    		      */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ (Word16*) new_wb_speech : original input signal at 16kHz    Q-1      */
/*------------------------------------------------------------------------*/

/* st_fx->old_wtda_wb_fx                                                  */
/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/

void wb_pre_proc_fx(
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure                   */
    const Word16 *new_inp_resamp16k,   /* i  : original input signal in Q-1              */
    Word16 *hb_speech            /* o  : HB target signal (6-8kHz) at 16kHz in Q-1 */
)
{
    Word16 Sample_Delay_WB_BWE;
    Word16 ramp_flag;
    Word16 old_input[NS2SA(16000, DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME16k];
    Word16 *highband_new_speech;
    Word16 highband_old_speech[(L_LOOK_12k8+L_SUBFR+L_FRAME)*5/16];
    Word16 temp_buf[320];
    Word16 Q_wb_sp, i, max_wb;

    set16_fx( old_input, 0, NS2SA_fx2(16000, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME16k );

    max_wb = 1;
    move16();
    FOR (i = 0; i < L_FRAME16k; i++ )
    {
        max_wb = s_max(max_wb, abs_s(new_inp_resamp16k[i]));
    }
    Q_wb_sp = norm_s(max_wb);
    Q_wb_sp = sub(Q_wb_sp, 3);  /* leave 3 bit headroom */
    Copy_Scale_sig(new_inp_resamp16k, temp_buf, L_FRAME16k, Q_wb_sp);
    Scale_sig(st_fx->decim_state1_fx, (2*ALLPASSSECTIONS_STEEP+1), Q_wb_sp);
    Scale_sig(st_fx->decim_state2_fx, (2*ALLPASSSECTIONS_STEEP+1), Q_wb_sp);
    /* temp_buf, and the memory states are in Q_wb_sp */

    test();
    test();
    IF ( sub(st_fx->extl_fx, WB_BWE) == 0 || sub(st_fx->extl_fx, WB_TBE) == 0 || st_fx->igf != 0 )
    {
        ramp_flag = 0;
        test();
        test();
        IF( sub(st_fx->last_extl_fx, WB_TBE) != 0 && sub(st_fx->last_extl_fx, WB_BWE) != 0 && st_fx->igf == 0 )
        {
            ramp_flag = 1;
        }

        IF ( !st_fx->ppp_mode_fx)
        {
            /* temp_buf is in Q_wb_sp
               hb_speech and the two decimator memories are in Q_wb_sp */
            flip_spectrum_and_decimby4_fx( temp_buf, hb_speech, L_FRAME16k, st_fx->decim_state1_fx, st_fx->decim_state2_fx, ramp_flag );

            /* rescale the hb_speech and memories back to Q-1 to keep the downstream BWE coding unaffected */
            Scale_sig(hb_speech, L_FRAME16k/4, -Q_wb_sp);
            Scale_sig(st_fx->decim_state1_fx, (2*ALLPASSSECTIONS_STEEP+1), -Q_wb_sp);
            Scale_sig(st_fx->decim_state2_fx, (2*ALLPASSSECTIONS_STEEP+1), -Q_wb_sp);

            IF( sub(st_fx->extl_fx, WB_TBE) != 0 )
            {
                /* Update the previous wideband speech buffer in case of a WB_BWE frame */
                Sample_Delay_WB_BWE = (L_LOOK_12k8 + L_SUBFR) * 5/16;

                highband_new_speech = highband_old_speech + Sample_Delay_WB_BWE;
                Copy( hb_speech, highband_new_speech, L_FRAME16k / 4 );
                Copy( highband_old_speech + L_FRAME16k / 4, st_fx->old_speech_wb_fx, Sample_Delay_WB_BWE );
            }
        }
    }
    ELSE
    {
        set16_fx( st_fx->decim_state1_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
        set16_fx( st_fx->decim_state2_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );
        set16_fx( st_fx->old_speech_wb_fx, 0, (L_LOOK_12k8 + L_SUBFR) * 5/16 );
    }

    /* st->old_input_wb and st->old_wtda_wb must be updated each frame, or there are often some clicks during WB TBE <-> WB BWE switching */
    test();
    test();
    test();
    IF ( (sub(st_fx->extl_fx, WB_BWE) != 0 || (sub(st_fx->extl_fx, WB_BWE) == 0 && L_sub(st_fx->total_brate_fx, ACELP_8k00) <= 0)) && !st_fx->ppp_mode_fx )
    {
        Sample_Delay_WB_BWE = NS2SA_fx2( 16000, DELAY_FD_BWE_ENC_12k8_NS );

        Copy( new_inp_resamp16k, &old_input[Sample_Delay_WB_BWE], L_FRAME16k );
        Copy( st_fx->old_input_wb_fx, old_input, Sample_Delay_WB_BWE );
        Copy( new_inp_resamp16k + L_FRAME16k - Sample_Delay_WB_BWE, st_fx->old_input_wb_fx, Sample_Delay_WB_BWE );
        Copy( old_input, st_fx->L_old_wtda_swb_fx, L_FRAME16k );
    }
    return;
}


/*========================================================================*/
/* FUNCTION :  swb_pre_proc_fx()										  */
/*------------------------------------------------------------------------*/
/* PURPOSE :   Calculate the 6 to 14 kHz (or 7.5 - 15.5 kHz)			  */
/*       SHB target signal for SWB TBE or SWB BWE coding                  */
/*       Common SWB TBE and SWB BWE pre-processing						  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* Encoder_State_fx *st_fx   : Encoder State Structure	     		Q0	  */
/* _ (Word16*) input_fx         : original input signal             Q0    */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _None                                                    		      */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ (Word16*) new_swb_speech_fx : original input signal at 16kHz    Q0   */
/* _ (Word16*) shb_speech_fx	 : original input signal at 16kHz    Q0   */
/*------------------------------------------------------------------------*/

/* st_fx->old_input_fx													  */
/* st_fx->old_wtda_shb_fx                                                 */
/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/

void swb_pre_proc_fx(
    Encoder_State_fx *st_fx,                                              /* i/o: encoder state structure                */
    const Word16 *input_fx,                                             /* i  : original input signal                  */
    Word16 *new_swb_speech_fx,                                    /* o  : original input signal at 32kHz         */
    Word16 *shb_speech_fx,                                        /* o  : SHB target signal (6-14kHz) at 16kHz   */
    Word16 *Q_shb_spch,
    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],   /* i : real CLDFB buffer for target synthesis */
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],   /* i : imag CLDFB buffer for target synthesis */
    const CLDFB_SCALE_FACTOR *cldfbScale                          /* i : scale data of real and imag CLDFB buffers */
)
{
    Word16 Sample_Delay_SWB_BWE, delay;
    Word16 inner_frame;
    UWord16 inner_Fs;
    Word16 old_input_fx[NS2SA(48000, DELAY_FD_BWE_ENC_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME48k];
    Word16 spchTmp_fx[L_FRAME32k];
    Word16 i;
    Word16 startB, endB;
    Word16 j;
    Word32 *realBufferFlipped[CLDFB_NO_COL_MAX];
    Word32 *imagBufferFlipped[CLDFB_NO_COL_MAX];
    Word32 realBufferTmp[CLDFB_NO_COL_MAX][20];
    Word32 imagBufferTmp[CLDFB_NO_COL_MAX][20];
    Word32 cldfbWorkBuffer[256];
    Word16 ts, nB, uB;
    /* Highband energy computation using CLDFB */
    Word32 CldfbHB_ener;
    Word16 realQ_neg1, imagQ_neg1, exp, Cldfbtemp1;

    FOR( j=0; j < CLDFB_NO_COL_MAX; j++ )
    {
        set32_fx(realBufferTmp[j], 0, 20);
        set32_fx(imagBufferTmp[j], 0, 20);
        realBufferFlipped[j] = realBufferTmp[j];
        imagBufferFlipped[j] = imagBufferTmp[j];
    }

    set16_fx( old_input_fx, 0, NS2SA_fx2(48000, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) + L_FRAME48k );

    IF( L_sub(st_fx->input_Fs_fx, 32000) == 0 )
    {
        Copy(input_fx, new_swb_speech_fx, L_FRAME32k); /*Q0 */
        test();
        IF( sub(st_fx->last_extl_fx, SWB_BWE) != 0 && sub(st_fx->last_extl_fx, FB_BWE) != 0 )
        {
            Sample_Delay_SWB_BWE = NS2SA_fx2( 32000, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS );
            Copy( st_fx->old_fdbwe_speech_fx, &old_input_fx[Sample_Delay_SWB_BWE], L_FRAME32k );
            set16_fx( old_input_fx, 0, Sample_Delay_SWB_BWE );
            Copy( st_fx->old_fdbwe_speech_fx + L_FRAME32k - Sample_Delay_SWB_BWE, st_fx->old_input_fx, Sample_Delay_SWB_BWE );

            Copy( old_input_fx, st_fx->L_old_wtda_swb_fx, L_FRAME32k );
        }

        test();
        IF( sub(st_fx->extl_fx, SWB_BWE) != 0 && sub(st_fx->extl_fx, FB_BWE) != 0 )
        {
            Copy( input_fx, st_fx->old_fdbwe_speech_fx, L_FRAME32k );
        }
    }
    ELSE /* 48 kHz */
    {
        /* 48kHz sampled processing needs review of FD2 memory handling/Q-factors */
        IF( sub(st_fx->codec_mode,MODE1) == 0 )
        {
            test();
            test();
            IF( sub(st_fx->extl_fx,SWB_BWE) != 0 && sub(st_fx->extl_fx,FB_BWE) != 0 && sub(st_fx->core_fx,ACELP_CORE) == 0 )
            {
                /* move the resampling out of the TDBWE path as new_swb_speech is not needed for TDBWE. */
                Copy( input_fx, st_fx->old_fdbwe_speech_fx, L_FRAME48k );
            }
            ELSE
            {
                test();
                IF( sub(st_fx->last_extl_fx,SWB_BWE) != 0 && sub(st_fx->last_extl_fx,FB_BWE) != 0 )
                {
                    /* resample 48 kHz to 32kHz */
                    IF( sub(st_fx->last_bwidth_fx,FB) == 0 )
                    {
                        inner_frame = L_FRAME48k;
                        inner_Fs = 48000;
                        Copy( st_fx->old_fdbwe_speech_fx, new_swb_speech_fx, L_FRAME48k );
                    }
                    ELSE
                    {
                        inner_frame = L_FRAME32k;
                        inner_Fs = 32000;
                        decimate_2_over_3_allpass_fx( st_fx->old_fdbwe_speech_fx, L_FRAME48k, new_swb_speech_fx, st_fx->dec_2_over_3_mem_fx, allpass_poles_3_ov_2,
                        decimate_3_ov_2_lowpass_num, decimate_3_ov_2_lowpass_den, st_fx->dec_2_over_3_mem_lp_fx );
                    }

                    Sample_Delay_SWB_BWE = NS2SA_fx2( inner_Fs, DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS );
                    Copy( new_swb_speech_fx, &old_input_fx[Sample_Delay_SWB_BWE], inner_frame );
                    set16_fx( old_input_fx, 0, Sample_Delay_SWB_BWE );
                    Copy( new_swb_speech_fx + inner_frame - Sample_Delay_SWB_BWE, st_fx->old_input_fx, Sample_Delay_SWB_BWE );
                    Copy( old_input_fx, st_fx->L_old_wtda_swb_fx, inner_frame );
                }
                /* resample 48 kHz to 32kHz */
                IF( sub(st_fx->bwidth_fx,FB) == 0 )
                {
                    Copy( input_fx, new_swb_speech_fx, L_FRAME48k );
                }
                ELSE
                {
                    decimate_2_over_3_allpass_fx( input_fx, L_FRAME48k, new_swb_speech_fx, st_fx->dec_2_over_3_mem_fx, allpass_poles_3_ov_2,
                    decimate_3_ov_2_lowpass_num, decimate_3_ov_2_lowpass_den, st_fx->dec_2_over_3_mem_lp_fx );
                }
            }
        }
        ELSE
        {
            /* resample 48 kHz to 32kHz */
            IF( sub(st_fx->bwidth_fx,FB) == 0 )
            {
                Copy( input_fx, new_swb_speech_fx, L_FRAME48k );
            }
            ELSE
            {
                decimate_2_over_3_allpass_fx( input_fx, L_FRAME48k, new_swb_speech_fx, st_fx->dec_2_over_3_mem_fx, allpass_poles_3_ov_2,
                decimate_3_ov_2_lowpass_num, decimate_3_ov_2_lowpass_den, st_fx->dec_2_over_3_mem_lp_fx );
            }
        }
    }

    test();
    test();
    test();
    test();
    test();
    IF( ( sub(st_fx->core_fx, ACELP_CORE) == 0 && sub( st_fx->extl_fx, SWB_BWE_HIGHRATE) != 0 && sub(st_fx->extl_fx, FB_BWE_HIGHRATE) != 0 )
        || ( ( L_sub(st_fx->total_brate_fx, 9600) == 0 || st_fx->rf_mode != 0 ) && sub(st_fx->bwidth_fx, SWB) == 0 ) )
    {
        IF( sub(st_fx->L_frame_fx, L_FRAME) == 0 )
        {
            startB= 34;
            endB= 14;
            FOR( ts = 0; ts < CLDFB_NO_COL_MAX; ts++ )
            {
                uB = 0;
                FOR( nB = startB; nB > endB; nB-- )
                {
                    realBufferFlipped[ts][uB] =  (ts & 0x0001)?(L_negate(realBuffer[ts][nB])):(realBuffer[ts][nB]);
                    imagBufferFlipped[ts][uB] =  (ts & 0x0001)?(imagBuffer[ts][nB]):(L_negate(imagBuffer[ts][nB]));
                    uB = add(uB, 1); /* uB ++ */
                }
            }
        }
        ELSE
        {
            startB = 39;
            endB = 19;
            FOR( ts = 0; ts < CLDFB_NO_COL_MAX; ts++ )
            {
                uB = 0;
                FOR( nB = startB; nB > endB; nB-- )
                {
                    realBufferFlipped[ts][uB] =  L_negate(realBuffer[ts][nB]);
                    imagBufferFlipped[ts][uB] =  imagBuffer[ts][nB];
                    uB = add(uB, 1); /* uB ++ */
                }
            }
        }

        /* Highband energy computation for gain shape control in case of bandwidth switching fix*/
        {
            CldfbHB_ener = 0;
            FOR (nB = 0; nB < 10; nB++)
            {
                FOR (ts = 0; ts < CLDFB_NO_COL_MAX; ts++)
                {
                    realQ_neg1 = extract_l(L_shr(realBufferFlipped[ts][nB], 31-(15+cldfbScale->hb_scale)+1));
                    imagQ_neg1 = extract_l(L_shr(imagBufferFlipped[ts][nB], 31-(15+cldfbScale->hb_scale)+1)); /* Q(-1), headroom needed */

                    CldfbHB_ener = L_mac0(CldfbHB_ener, realQ_neg1, realQ_neg1);
                    CldfbHB_ener = L_mac0(CldfbHB_ener, imagQ_neg1, imagQ_neg1); /* Q(-2) */
                }
            }

            exp = norm_l(CldfbHB_ener);
            CldfbHB_ener = L_shl(CldfbHB_ener, exp); /* CldfbHB_ener = CldfbHB_fl*2^(exp) */
            Cldfbtemp1 = (Log2_norm_lc(CldfbHB_ener));
            Cldfbtemp1 = add(shr(Cldfbtemp1, 6), shl(sub(30, sub(exp, 2)), 9));/* Log2_norm_lc(CldfbHB_ener) = 2^15*(log2(CldfbHB_ener/2^30)) = 2^15*(log2(CldfbHB_fl*(2^-2)*2^exp/2^30)) = 2^15*(log2(CldfbHB_fl) + exp-2-30) => 2^(-6)*l2nc + 2^9(20-(exp-2)) = 2^9*log2(CldfbHB_fl), Q9 */
            CldfbHB_ener = L_mult(sub(Cldfbtemp1, FL2WORD16_SCALE(3.401, 15 - 9)), 3495); /* 3495 = Q19 log10(2)*0.1/log10(32768), Q = 19+9+1 = 29 */
            st_fx->cldfbHBLT = mac_r(CldfbHB_ener, FL2WORD16(0.9), st_fx->cldfbHBLT); /* cldfbHBLT is in Q13 */
        }

        cldfbSynthesisFiltering( st_fx->cldfbSyn_Fx, realBufferFlipped, imagBufferFlipped,
                                 cldfbScale, shb_speech_fx, 0, CLDFB_NO_COL_MAX, cldfbWorkBuffer );
        *Q_shb_spch = 0; /*shb_speech_fx : Q0*/

        test();
        test();
        IF( sub(st_fx->extl_fx, WB_TBE) != 0 && sub(st_fx->extl_fx, SWB_TBE) != 0 && sub(st_fx->extl_fx, FB_TBE) != 0 )
        {
            /* Update the previous superwideband speech buffer in case of a SWB_BWE frame - this code is in swb_tbe_enc */
            delay = L_LOOK_16k + L_SUBFR16k;
            Copy( shb_speech_fx + L_FRAME16k - delay, st_fx->old_speech_shb_fx, delay );
        }
    }
    ELSE
    {
        IF( sub(st_fx->bwidth_fx, FB) == 0 || sub(st_fx->core_fx, ACELP_CORE) == 0)
        {
            set16_fx( st_fx->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k );
            set16_fx( shb_speech_fx, 0, L_FRAME16k );   /* shb_speech for FB/SWB BWE_HIGHRATE is not used at 64kbps */
        }
        ELSE
        {
            /* flip the spectrm */
            Copy( new_swb_speech_fx, spchTmp_fx, L_FRAME32k );

            FOR( i = 0; i < L_FRAME32k; i = i+2 )
            {
                spchTmp_fx[i] = negate(spchTmp_fx[i]);
            }

            Decimate_allpass_steep_fx( spchTmp_fx, st_fx->state_ana_filt_shb_fx, L_FRAME32k, shb_speech_fx );
            Copy( shb_speech_fx + L_FRAME16k - (L_LOOK_16k + L_SUBFR16k), st_fx->old_speech_shb_fx, L_LOOK_16k + L_SUBFR16k );
        }

        /* Reset CLDFB synthesis buffer */
        set16_fx( st_fx->cldfbSyn_Fx->FilterStates, 0, st_fx->cldfbSyn_Fx->p_filter_length + st_fx->cldfbSyn_Fx->no_channels*st_fx->cldfbSyn_Fx->no_col );
    }
    IF( st_fx->last_extl_fx == -1 )
    {
        delay = NS2SA(st_fx->input_Fs_fx, DELAY_FIR_RESAMPL_NS);
        FOR( i = 0; i < delay; i++ )
        {
            shb_speech_fx[i] = mult_r( mult_r(i, FL2WORD16(0.03f)), shb_speech_fx[2*delay-1-i] );
        }
    }

    return;
}

