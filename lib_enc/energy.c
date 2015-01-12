/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <memory.h>


#include "basop_util.h"
#include "stl.h"
#include "rom_enc_fx.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"

void est_energy(
    Word32 enerBuffer[CLDFB_NO_CHANNELS_MAX],              /* i: energy vector per band */
    Word16 enerBuffer_exp,                               /* i: exponent of energy vector */
    Word32 *frame_sb_energy,       /*(o) energy of sub-band divided non-uniformly*/
    Word32 *frame_energy2_p,       /*(o) frame energy 2*/
    Word32 *HB_Power_p,	           /*(o) high frequency energy*/
    Word32 *frame_energy_p,        /*(o) frame energy 1*/
    Word16 *sb_power_Q,	           /*(o) the scaling of sb_power*/
    Word16 *frame_energy2_Q,       /*(o) the scaling of frame_energy*/
    Word16 *HB_Power_Q,            /*(o) the scaling of HB_Power*/
    Word16 *frame_energy_Q,        /*(o) the Scaling of frame_energy*/
    Word16 *frame_sb_energy_scale, /*(o) the Scaling of frame_sb_energy[]*/
    Word32 bandwidth               /*(i) band width*/
)
{
    Word32 i,j;
    Word32 frame_energy2,HB_Power,tmpspec_amp;
    Word32 sb_power_tmp;
    Word32 frame_energy,s32CopyPower;
    Word32 SNR_sb_num;
    Word16 shr_tmp;
    Word32 BandNum;
    Word16 widthsb,s16MaxCoefNorm;
    const Word16 *Nregion_index;
    Word32 *sb_power = enerBuffer;
    Word32 Ltmp32 = 0;

    SNR_sb_num = SNR_SUB_BAND_NUM[bandwidth-CLDFBVAD_NB_ID];
    move16();
    Nregion_index = REGION_INDEX[bandwidth-CLDFBVAD_NB_ID];
    move16();

    shr_tmp = BAND_SCALE_AJ[bandwidth];
    move16();
    BandNum = BAND_NUM_TAB[bandwidth];
    move16();

    frame_energy2 = L_shr(sb_power[1], shr_tmp);
    HB_Power = L_shr(sb_power[6], shr_tmp);
    FOR(i = 2; i < BandNum; i++)
    {
        Ltmp32 = L_shr(sb_power[i],shr_tmp);
        frame_energy2 = L_add(frame_energy2, Ltmp32 );
        if(i > 6)  HB_Power = L_add(HB_Power, Ltmp32 );
    }
    frame_energy2 = L_sub(frame_energy2, Ltmp32 );

    sb_power_tmp = L_shr(sb_power[0],shr_tmp);

    IF(L_sub(bandwidth,1)==0)
    {
        frame_energy = L_add(frame_energy2,MUL_F(sb_power_tmp,0x147a));
    }
    ELSE IF(L_sub(bandwidth,2)==0)
    {
        frame_energy = L_add(frame_energy2,MUL_F(sb_power_tmp,0x1eb8));
    }
    ELSE IF(L_sub(bandwidth,3)==0)
    {
        frame_energy = L_add(frame_energy2,MUL_F(sb_power_tmp,0x23d7));
    }
    ELSE IF(L_sub(bandwidth,4)==0)
    {
        frame_energy = L_add(frame_energy2,MUL_F(sb_power_tmp,0x23d7));
    }
    ELSE
    {
        frame_energy = L_add(frame_energy2,MUL_F(sb_power_tmp,0x23d7));
    }

    *frame_energy2_p = frame_energy2;
    move32();
    *HB_Power_p = HB_Power;
    move32();
    *frame_energy_p = frame_energy;
    move32();
    /* enerBuffer(float) = enerBuffer(fixed) * 2^(-(31-enerBuffer_exp)) */
    /* +30 is to keep original precision */
    *sb_power_Q = sub(31+30,enerBuffer_exp);
    move16();
    *frame_energy2_Q = sub(*sb_power_Q,shr_tmp);
    move16();
    *HB_Power_Q = sub(*sb_power_Q,shr_tmp);
    move16();
    *frame_energy_Q = sub(*sb_power_Q,shr_tmp);
    move16();

    FOR(i=0; i<6; i++)
    {
        frame_sb_energy[i] = sb_power[i];
        move32();
    }

    FOR(i=6; i<SNR_sb_num; i++)
    {
        s32CopyPower = L_add(0,0);
        widthsb =  sub(Nregion_index[i+1],Nregion_index[i]);
        s16MaxCoefNorm = sub(ffr_getSfWord32((sb_power+Nregion_index[i]),widthsb),Nregion_preoff[i]);

        FOR(j=Nregion_index[i]; j<Nregion_index[i+1]; j++)
        {
            tmpspec_amp = L_shl(sb_power[j],s16MaxCoefNorm);
            s32CopyPower = L_add(s32CopyPower, tmpspec_amp);
        }
        frame_sb_energy[i] = L_shr(s32CopyPower, s16MaxCoefNorm);
        move32();
    }

    *frame_sb_energy_scale = *sb_power_Q;
    move16();
}

static
void NormEnergyWord32(Word32 *vector, Word32 len, Word16 *pi_scale, Word16* normscale)
{
    Word16 minVal;
    Word16 i;


    minVal = pi_scale[0];
    move16();

    FOR(i=1; i<len; i++)
    {
        minVal = s_min(minVal, pi_scale[i]);
    }

    FOR (i=0; i<len; i++)
    {
        vector[i] = L_shr(vector[i], sub(pi_scale[i], minVal));
        move32();
    }
    *normscale = minVal;
    move16();

}
static void update_sb_bg_energy(
    Word32 *sb_bg_energy,
    Word16 *tbg_energy_count,
    Word16* p_scale_sb_energy,
    const Word32 SNR_sb_num,
    const Word32* frame_sb_energy,
    const Word16 frame_sb_energy_scale,
    const Word16 FAC_16Q15_a,
    const Word16 tmp_Q_add,
    const Word16 FAC_16Q19_b
)
{
    Word32 sb_bg_energy_ti, tmp;
    Word16 tmpQ, i;

    *tbg_energy_count = add(*tbg_energy_count, 1);
    move16();
    tmpQ = add(tmp_Q_add, frame_sb_energy_scale);

    FOR(i=0; i<SNR_sb_num; i++)
    {
        sb_bg_energy_ti = MUL_F(sb_bg_energy[i],FAC_16Q15_a);
        tmp = MUL_F(frame_sb_energy[i], FAC_16Q19_b);
        sb_bg_energy[i] =  VAD_L_ADD( sb_bg_energy_ti , p_scale_sb_energy[i], tmp,tmpQ,&p_scale_sb_energy[i]);
        move32();
    }
}


void background_update(T_CldfbVadState *st,        /*(io) vad state*/
                       Word16 scale,             /*(i) the scaling of frame energy*/
                       Word32 frame_energy,      /*(i) current frame energy*/
                       Word32 update_flag,       /*(i) update flag*/
                       Word16 music_backgound_f  /*(i) background music flag*/
                      )
{
    Word32 tmp;
    Word16 i, tmpQ, cmp_lt_frame, cmp_pre_frame;
    Word32 SNR_sb_num;
    Word16 normscal,scale_sb_energy;
    Word32 *sb_bg_energy;
    Word32 *frame_sb_energy;
    Word32 t_bg_energy	;
    Word32 sb_bg_energy_ti ;
    Word16 q_divout;
    T_VAD_EXP  exp_frame_energy,CONST32fix;
    T_VAD_EXP t_bg_energy_sum;
    Word16 p_scale_sb_energy[12] ;
    Word16 *f_tonality_rate = st->f_tonality_rate;
    Word16 *ltd_stable_rate = st->ltd_stable_rate;
    Word32 bandwith = st->bw_index;
    Word16 cmp_tmp;


    move32();
    move16();
    move16();


    tmp = L_deposit_l(-1);

    CONST32fix.s16Exp = 44;
    move16();
    CONST32fix.s32Mantissa = 1759218560;
    move32();
    SNR_sb_num = SNR_SUB_BAND_NUM[bandwith - CLDFBVAD_NB_ID];
    move16();
    scale_sb_energy = st->sb_bg_energy_scale;
    move16();

    sb_bg_energy = st->sb_bg_energy;
    move32();
    frame_sb_energy = st->frame_sb_energy;
    move32();
    t_bg_energy = L_add(st->t_bg_energy,0);
    move32();

    t_bg_energy_sum = st->t_bg_energy_sum;
    move32();
    normscal = norm_l(frame_energy);
    exp_frame_energy.s16Exp = add(scale,normscal);
    exp_frame_energy.s32Mantissa = L_shl(frame_energy,normscal);
    exp_frame_energy = VAD_AddExp(exp_frame_energy,CONST32fix);
    cmp_lt_frame = VAD_L_CMP(exp_frame_energy.s32Mantissa,exp_frame_energy.s16Exp,46,0);

    FOR(i=0; i<SNR_sb_num; i++)
    {
        p_scale_sb_energy[i] = scale_sb_energy;
        move16();
    }

    test();
    test();
    test();
    test();
    test();
    IF( (sub(st->frameloop, 60) < 0) && (sub(st->frameloop, 5) > 0) &&(sub(f_tonality_rate[0],CONST_16_Q14(0.56))<0)
        && (sub(f_tonality_rate[1],CONST_16_Q14(0.5))<0) && (sub(ltd_stable_rate[1],CONST_16_Q15(0.06))<0) && (cmp_lt_frame<0 ))

    {
        t_bg_energy_sum = VAD_AddExp(t_bg_energy_sum,exp_frame_energy);

        update_sb_bg_energy(sb_bg_energy, &st->tbg_energy_count, p_scale_sb_energy, SNR_sb_num, frame_sb_energy, st->frame_sb_energy_scale,
                            CONST_16_Q15(0.90f), 3, 26214);

    }

    test();
    test();
    IF((L_sub(update_flag,1)==0) && (sub(st->frameloop, 2) > 0) && music_backgound_f==0)
    {
        IF(sub(st->bg_update_count, 16) < 0)
        {
            t_bg_energy_sum = VAD_AddExp(t_bg_energy_sum, exp_frame_energy);
            update_sb_bg_energy(sb_bg_energy, &st->tbg_energy_count, p_scale_sb_energy, SNR_sb_num, frame_sb_energy, st->frame_sb_energy_scale,
                                CONST_16_Q15(0.96f), 4, CONST_16_Q19(0.04f));

            st->bg_update_count = add(st->bg_update_count, 1);
        }
        ELSE
        {

            cmp_lt_frame = VAD_L_CMP(t_bg_energy, st->scale_t_bg_energy, exp_frame_energy.s32Mantissa, exp_frame_energy.s16Exp);
            cmp_pre_frame = VAD_L_CMP(MUL_F(st->frame_energy_smooth, 24576), sub(st->frame_energy_smooth_scale, 5), exp_frame_energy.s32Mantissa, exp_frame_energy.s16Exp);
            cmp_tmp = VAD_L_CMP(MUL_F(t_bg_energy, 24576), sub(st->scale_t_bg_energy, 4), exp_frame_energy.s32Mantissa, exp_frame_energy.s16Exp);

            test();
            IF( (cmp_lt_frame < 0) && (cmp_pre_frame < 0) )
            {
                tmpQ = add(9, st->frame_sb_energy_scale);
                FOR(i=0; i<SNR_sb_num; i++)
                {
                    sb_bg_energy_ti = MUL_F(sb_bg_energy[i], CONST_16_Q15(0.999f));
                    tmp = MUL_F(frame_sb_energy[i], CONST_16_Q24(0.001f));
                    sb_bg_energy[i] =  VAD_L_ADD(sb_bg_energy_ti, p_scale_sb_energy[i], tmp, tmpQ, &p_scale_sb_energy[i]);
                    move32();
                }
            }
            /*ELSE IF(L_sub(tmp, -1) == 0) fixed bug*/
            ELSE IF(cmp_tmp < 0)
            {
                t_bg_energy_sum = VAD_AddExp(t_bg_energy_sum, exp_frame_energy);
                update_sb_bg_energy(sb_bg_energy, &st->tbg_energy_count, p_scale_sb_energy, SNR_sb_num, frame_sb_energy, st->frame_sb_energy_scale,
                                    CONST_16_Q15(0.96f), 4, CONST_16_Q19(0.04f));

            }
            ELSE
            {
                cmp_pre_frame = VAD_L_CMP(t_bg_energy, st->scale_t_bg_energy, exp_frame_energy.s32Mantissa, exp_frame_energy.s16Exp);
                IF(cmp_pre_frame>0)
                {
                    t_bg_energy_sum = VAD_AddExp(t_bg_energy_sum, exp_frame_energy);
                    update_sb_bg_energy(sb_bg_energy, &st->tbg_energy_count, p_scale_sb_energy, SNR_sb_num, frame_sb_energy, st->frame_sb_energy_scale,
                    CONST_16_Q15(0.95f), 4, CONST_16_Q19(0.05f));

                }
                ELSE
                {
                    t_bg_energy_sum = VAD_AddExp(t_bg_energy_sum, exp_frame_energy);
                    update_sb_bg_energy(sb_bg_energy, &st->tbg_energy_count, p_scale_sb_energy, SNR_sb_num, frame_sb_energy, st->frame_sb_energy_scale,
                    CONST_16_Q15(0.96f), 4, CONST_16_Q19(0.04f));

                }
            }
        }
    }
    ELSE
    {
        cmp_pre_frame = VAD_L_CMP(t_bg_energy, st->scale_t_bg_energy, MUL_F(exp_frame_energy.s32Mantissa, 32000), sub(exp_frame_energy.s16Exp, 9));
        cmp_lt_frame = VAD_L_CMP(sb_bg_energy[0], scale_sb_energy, MUL_F(frame_sb_energy[0], 20480), sub(st->frame_sb_energy_scale, 4));

        test();
        IF( (cmp_pre_frame > 0) && (cmp_lt_frame > 0) )
        {
            tmpQ = add(4, st->frame_sb_energy_scale);
            FOR(i=0; i<SNR_sb_num; i++)
            {
                sb_bg_energy_ti = MUL_F(sb_bg_energy[i], CONST_16_Q15(0.96f));
                tmp = MUL_F(frame_sb_energy[i], CONST_16_Q19(0.04f));
                sb_bg_energy[i] = VAD_L_ADD(sb_bg_energy_ti, p_scale_sb_energy[i], tmp, tmpQ, &p_scale_sb_energy[i]);
                move32();
            }
        }
        ELSE
        {
            cmp_pre_frame = VAD_L_CMP(t_bg_energy, st->scale_t_bg_energy, MUL_F(exp_frame_energy.s32Mantissa, 20480), sub(exp_frame_energy.s16Exp, 4));
            IF( cmp_pre_frame > 0 )
            {
                tmpQ = add(9, st->frame_sb_energy_scale);
                FOR(i=0; i<SNR_sb_num; i++)
                {
                    sb_bg_energy_ti = MUL_F(sb_bg_energy[i], CONST_16_Q15(0.999f));
                    tmp = MUL_F(frame_sb_energy[i], CONST_16_Q24(0.001f));
                    sb_bg_energy[i] = VAD_L_ADD(sb_bg_energy_ti, p_scale_sb_energy[i], tmp, tmpQ, &p_scale_sb_energy[i]);
                    move32();
                }
            }
        }
    }

    tmp = L_mult0(160, (Word16)st->tbg_energy_count);
    cmp_pre_frame = VAD_L_CMP(t_bg_energy_sum.s32Mantissa, t_bg_energy_sum.s16Exp, tmp, 0);
    IF(cmp_pre_frame > 0)
    {
        i = norm_l(tmp);
        t_bg_energy_sum.s32Mantissa = L_shl(tmp, i);
        t_bg_energy_sum.s16Exp = i;
        move16();
    }
    NormEnergyWord32(sb_bg_energy, SNR_sb_num, p_scale_sb_energy, &scale_sb_energy);
    cmp_pre_frame = VAD_L_CMP(t_bg_energy, st->scale_t_bg_energy, 1, 0);

    test();
    test();
    test();
    IF( (sub(music_backgound_f, 1) == 0) && (L_sub(st->lt_snr_org, CONST_32_Q25(3.2)) < 0)
        && (cmp_pre_frame > 0) && update_flag == 0)
    {
        tmp = L_shr(CONST_32_Q31(0.000001f), sub(31, scale_sb_energy));
        FOR(i=0; i<SNR_sb_num; i++)
        {
            sb_bg_energy[i] = L_add(MUL_F(sb_bg_energy[i], CONST_16_Q15(0.98f)), tmp);
            move32();
        }
    }

    IF(sub(music_backgound_f,1) == 0)
    {
        cmp_pre_frame = VAD_L_CMP(exp_frame_energy.s32Mantissa, exp_frame_energy.s16Exp, MUL_F(t_bg_energy, 5000), sub(st->scale_t_bg_energy, 15));
        IF(cmp_pre_frame < 0)
        {
            tmp = L_shr(CONST_32_Q31(0.000001f), sub(31, scale_sb_energy));
            FOR(i=0; i<SNR_sb_num; i++)
            {
                sb_bg_energy[i] = L_add(MUL_F(sb_bg_energy[i], CONST_16_Q15(0.98f)), tmp);
                move32();
            }
        }
    }

    IF( (L_sub(st->tbg_energy_count, 64) == 0))
    {
        st->tbg_energy_count = 48;
        move16();
        t_bg_energy_sum.s32Mantissa = MUL_F(t_bg_energy_sum.s32Mantissa, CONST_16_Q15(0.75f));
    }

    t_bg_energy = VAD_L_div(t_bg_energy_sum.s32Mantissa, st->tbg_energy_count, t_bg_energy_sum.s16Exp, 0, &q_divout);
    st->scale_t_bg_energy = q_divout;
    move16();
    st->t_bg_energy = t_bg_energy;
    move32();
    st->sb_bg_energy_scale = scale_sb_energy;
    move16();
    st->t_bg_energy_sum = t_bg_energy_sum;
    move16();
    move32();

}

