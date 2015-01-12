/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>


#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"
#include "stat_enc_fx.h"
#include "rom_enc_fx.h"

Word16 vad_init(T_CldfbVadState *vad_state)
{
    Word16 i = 0;

    IF(vad_state == NULL)
    {
        return -1;
    }

    vad_state->frameloop=0;
    move16();
    vad_state->continuous_noise_num = 0;
    move16();
    vad_state->continuous_speech_num = 0;
    move16();
    vad_state->continuous_speech_num2 = 0;
    move16();
    vad_state->fg_energy_est_start = L_deposit_l(0);
    vad_state->speech_flag = 0;
    move16();
    vad_state->frame_sb_energy_scale = 0;
    move16();
    vad_state->speech_flag2 = 0;
    move16();
    vad_state->updateNumWithSnr=0;
    move16();

    FOR(i = 0; i < SPEC_AMP_NUM; i++)
    {
        vad_state->smooth_spec_amp[i] = L_deposit_l(0);
    }

    FOR(i = 0; i < PRE_SNR_NUM; i++)
    {
        vad_state->pre_snr[i] = L_deposit_l(0);
    }

    FOR(i = 0; i < BG_ENG_NUM; i++)
    {
        vad_state->frame_sb_energy[i] = L_deposit_l(0);
    }

    vad_state->sfm[0] = CONST_16_Q15(0.88f);
    move16();
    vad_state->sfm[1] = CONST_16_Q15(0.92f);
    move16();
    vad_state->sfm[2] = CONST_16_Q15(0.92f);
    move16();
    vad_state->sfm[3] = CONST_16_Q15(0.92f);
    move16();
    vad_state->sfm[4] = CONST_16_Q15(0.9f);
    move16();
    vad_state->l_silence_snr_count = L_deposit_l(1);
    vad_state->l_speech_snr_count = L_deposit_l(1);
    vad_state->lt_snr_org = 33554432;
    move32();
    vad_state->lf_snr_smooth = CONST_32_Q25(5.0f);
    move32();
    vad_state->fg_energy = 1073741824;
    move32();
    vad_state->fg_energy_scale = 41;
    move16();

    vad_state->bg_energy = 1073741824;
    move32();
    vad_state->bg_energy_scale = 57;
    move16();
    vad_state->lt_noise_sp_center_diff_counter = L_deposit_l(4);

    vad_state->t_bg_energy = 1374389535;
    move32();
    vad_state->scale_t_bg_energy = 37;
    move16();

    vad_state->t_bg_energy_sum.s16Exp = 37;
    move16();
    vad_state->t_bg_energy_sum.s32Mantissa = 1374389535;
    move32();
    vad_state->tbg_energy_count = 1;
    move16();
    vad_state->fg_energy_count = 16;
    move16();
    vad_state->bg_energy_count = 16;
    move16();

    vad_state->bg_update_count = 0;
    move16();
    vad_state->frame_energy_smooth = 1073741824;
    move32();
    vad_state->frame_energy_smooth_scale = 30;
    move16();
    vad_state->Q_frames_power_32 = 31;
    move16();
    vad_state->lt_bg_highf_eng = CONST_32_Q16(2.0f);
    move16();
    vad_state->lt_noise_sp_center0 = CONST_16_Q10(1.8f);
    move16();
    vad_state->lt_noise_sp_center3 = CONST_16_Q10(2.0f);
    move16();
    vad_state->music_background_rate = CONST_16_Q15(0.46f);
    move16();
    vad_state->tonality_rate3 = CONST_16_Q15(0.46f);
    move16();
    vad_state->lt_noise_sp_center_diff_sum = CONST_32_Q10(0.4f);
    move32();
    vad_state->l_silence_snr = CONST_32_Q16(0.5f);
    move32();
    vad_state->l_speech_snr =  CONST_32_Q16(5.0f);
    move32();

    FOR(i = 0; i < SP_CENTER_NUM; i++)
    {
        vad_state->sp_center[i] = CONST_16_Q10(1.2f);
        move16();
    }

    FOR(i = 0; i < STABLE_NUM; i++)
    {
        vad_state->ltd_stable_rate[i] = CONST_16_Q15(0.07f);
        move16();
    }

    FOR(i = 0; i < BG_ENG_NUM; i++)
    {
        vad_state->sb_bg_energy[i] =1374389535;
        move32();
    }
    vad_state->sb_bg_energy_scale = 37;
    move16();

    vad_state->f_tonality_rate[0] = CONST_16_Q14(0.48f);
    move16();
    vad_state->f_tonality_rate[1] = CONST_16_Q14(0.48f);
    move16();
    vad_state->f_tonality_rate[2] = CONST_16_Q14(0.48f);
    move16();


    FOR(i = 0; i < PRE_SPEC_DIF_NUM; i++)
    {
        vad_state->pre_spec_low_dif[i] = 4095;
        move16();
    }
    vad_state->scale_spec_low_dif = 12;
    move16();

    FOR (i = 0; i < 56; i++)
    {
        vad_state->frames_power_32[i] = L_deposit_l(0);
    }
    vad_state->update_num = L_deposit_l(16);
    vad_state->update_sum = L_deposit_l(0);
    vad_state->update_hangover = L_deposit_l(0);


    return 0;
}


void UpdateState(T_CldfbVadState *vad_state,
                 Word16 vad_flag,
                 Word32 frame_energy,       /*(i)  current frame energy*/
                 Word16 sacle_sbpower,      /*(i)  the Scaling of current frame energy*/
                 Word32 update_flag,        /*(i)  current frame update flag*/
                 Word16 music_backgound_f,  /*(i)  backgound music flag*/
                 Word32 HB_Power,           /*(i)  current frame high frequency energy*/
                 Word16 HB_Power_Q          /*(i)  the Scaling of current frame high frequency energy*/
                )
{
    Word16 lt_bg_energy_scal;
    Word32 tmp,tmp2;


    tmp = MUL_F(vad_state->frame_energy_smooth, CONST_16_Q15(0.95f));
    tmp2 = MUL_F(frame_energy, 26214);
    vad_state->frame_energy_smooth = VAD_L_ADD(tmp, vad_state->frame_energy_smooth_scale, tmp2, add(4, sacle_sbpower), &lt_bg_energy_scal);
    move32();
    vad_state->frame_energy_smooth_scale = lt_bg_energy_scal;
    move16();


    IF( vad_flag == 0 )
    {
        vad_state->lt_bg_highf_eng = L_add(MUL_F(vad_state->lt_bg_highf_eng, 31130), L_shr(MUL_F(HB_Power, 1638), sub(HB_Power_Q, lt_bg_highf_eng_Q)));
    }

    if(sub(vad_state->frameloop, 1000) < 0)
    {
        vad_state->frameloop = add(vad_state->frameloop, 1);
        move16();
    }

    background_update(vad_state,
                      sacle_sbpower,
                      frame_energy,
                      update_flag,
                      music_backgound_f
                     );
    IF( vad_flag== 0)
    {
        vad_state->continuous_speech_num2 = 0;
        move16();
        IF(sub(vad_state->continuous_noise_num, 10) > 0)
        {
            vad_state->continuous_speech_num = 0;
            move16();
        }
        ELSE IF(L_sub(vad_state->continuous_speech_num, 9) > 0)
        {
            vad_state->continuous_speech_num = 9;
            move16();
        }
        vad_state->continuous_noise_num = add(vad_state->continuous_noise_num, 1);
        move16();

        if(sub(vad_state->continuous_noise_num, 2048) > 0)
        {
            vad_state->continuous_noise_num = 2048;
            move16();
        }
    }
    ELSE
    {
        vad_state->continuous_noise_num = 0;
        move16();

        vad_state->continuous_speech_num2 = add(vad_state->continuous_speech_num2, 1);
        vad_state->continuous_speech_num = add(vad_state->continuous_speech_num, 1);
        if(sub(vad_state->continuous_speech_num, 2048) > 0)
        {
            vad_state->continuous_speech_num = 2048;
            move16();
        }

        if(sub(vad_state->continuous_speech_num2, 2048) > 0)
        {
            vad_state->continuous_speech_num2 = 2048;
            move16();
        }
    }

}


Word16 vad_proc(T_CldfbVadState *vad_st,
                Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],    /* i: real values */
                Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],    /* i: imag values */
                Word16 riBuffer_exp,                                           /* i: exponent of real & imag Buffer */
                Word16 *cldfb_addition,                                /*o: adjust the harmonized hangover */
                Word32 enerBuffer[CLDFB_NO_CHANNELS_MAX],            /* i: energy vector per band */
                Word16 enerBuffer_exp,                               /* i: exponent of energy vector */
                Word16 numBands,                                     /* i: band width 1: NB; 2:WB;3:SWB;4:FB*/
                Word16 vada_flag
               )
{
    Word16 i;
    Word16 bandwidth;
    Word16 music_backgound_f;
    Word16 Q_cldfb;
    Word16 frame_energy2_Q, HB_Power_Q;
    Word16 sb_power_Q,frame_energy_Q;
    Word32 frame_energy, frame_energy2, HB_Power;
    Word32 spec_amp[120];
    Word32 update_flag,snr_flux,lt_snr_org,lt_snr,lf_snr;
    Word32 snr,tsnr;
    Word16 vad_flag;
    Word32 *cldfbBufferReal[CLDFB_NO_COL_MAX]; /* dynamic scaling; cldfbBufferReal_float[x][y] = cldfbBufferReal[x][y] * 2^(-Q_cldfb) */
    Word32 *cldfbBufferImag[CLDFB_NO_COL_MAX]; /* dynamic scaling; cldfbBufferImag_float[x][y] = cldfbBufferReal[x][y] * 2^(-Q_cldfb) */


    music_backgound_f = add(0,0);
    frame_energy = L_add(0,0);


    IF(sub(numBands, 20) < 0)
    {
        bandwidth = 1;
        move16();
    }
    ELSE IF(sub(numBands, 40) < 0)
    {
        bandwidth = 2;
        move16();
    }
    ELSE
    {
        bandwidth = 3;
        move16();
    }

    vad_st->bw_index = bandwidth;
    move16();
    FOR (i=0; i<CLDFB_NO_COL_MAX; i++)
    {
        cldfbBufferReal[i] = realBuffer[i];
        move16();
        cldfbBufferImag[i] = imagBuffer[i];
        move16();
    }


    Q_cldfb        = sub(31, riBuffer_exp);

    Q_cldfb = sub(Q_cldfb, 16);


    est_energy(enerBuffer,
               enerBuffer_exp,
               vad_st->frame_sb_energy,
               &frame_energy2,
               &HB_Power,
               &frame_energy,
               &sb_power_Q,
               &frame_energy2_Q,
               &HB_Power_Q,
               &frame_energy_Q,
               &vad_st->frame_sb_energy_scale,
               bandwidth
              );

    subband_FFT(cldfbBufferReal,
                cldfbBufferImag,
                spec_amp,
                0,
                &Q_cldfb
               );

    frame_spec_dif_cor_rate(vad_st, spec_amp,
                            add(Q_cldfb, 8),
                            vad_st->f_tonality_rate
                           );

    spec_center(enerBuffer,
                vad_st->sp_center,
                bandwidth,
                sb_power_Q
               );

    ltd_stable(vad_st,
               vad_st->ltd_stable_rate,
               frame_energy,
               vad_st->frameloop,
               frame_energy_Q
              );

    spec_flatness(spec_amp,
                  vad_st->smooth_spec_amp,
                  vad_st->sfm
                 );

    bg_music_decision(vad_st,
                      &music_backgound_f,
                      frame_energy,
                      frame_energy_Q
                     );

    snr_calc(vad_st,
             frame_energy2_Q,
             &snr,
             &tsnr,
             frame_energy2,
             bandwidth
            );

    calc_snr_flux(tsnr,
                  vad_st->pre_snr,
                  &snr_flux
                 );

    calc_lt_snr(vad_st,
                &lt_snr_org,
                &lt_snr,
                vad_st->fg_energy,
                vad_st->fg_energy_count,
                vad_st->bg_energy,
                vad_st->bg_energy_count,
                bandwidth,
                vad_st->lt_noise_sp_center0
               );

    calc_lf_snr(&vad_st->lf_snr_smooth,
                &lf_snr,
                vad_st->l_speech_snr,
                vad_st->l_speech_snr_count,
                vad_st->l_silence_snr,
                vad_st->l_silence_snr_count,
                vad_st->fg_energy_count,
                vad_st->bg_energy_count,
                bandwidth
               );


    vad_flag = comvad_decision(vad_st,
                               lf_snr,
                               lt_snr_org,
                               lt_snr,
                               snr_flux,
                               snr,
                               tsnr,
                               frame_energy2,
                               music_backgound_f,
                               frame_energy2_Q,
                               cldfb_addition,
                               vada_flag
                              );


    update_flag = update_decision(vad_st,
                                  frame_energy,
                                  HB_Power,
                                  vad_st->frameloop,
                                  bandwidth,
                                  frame_energy_Q,
                                  HB_Power_Q,
                                  snr,
                                  tsnr,
                                  vad_flag,
                                  music_backgound_f
                                 );


    UpdateState(vad_st,
                vad_flag,
                frame_energy2,
                frame_energy2_Q,
                update_flag,
                music_backgound_f,
                HB_Power,
                HB_Power_Q
               );


    return vad_flag;
}



