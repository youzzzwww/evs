/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>

#include "basop_util.h"
#include "options.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "rom_enc_fx.h"
#include "prot_fx.h"
#include "rom_enc_fx.h"



void calc_lf_snr(
    Word32  *lf_snr_smooth,            /*(o) smoothed lf_snr*/
    Word32  *lf_snr,                   /*(o) long time frequency domain
                                                        SNR calculated by l_speech_snr and l_silence_snr*/
    Word32   l_speech_snr,             /*(i) sum of active frames snr */
    Word32   l_speech_snr_count,       /*(i) amount of the active frame  */
    Word32   l_silence_snr,            /*(i) sum of the nonactive frames snr*/
    Word32   l_silence_snr_count,      /*(i) amount of the nonactive frame */
    Word16   fg_energy_count,          /*(i) amount of the foreground energy frame */
    Word16   bg_energy_count,          /*(i) amount of the background energy frame */
    Word16   bw_index                  /*(i) band width index*/
)
{

    Word32 l_snr,div1,div2,tmp;
    Word16 q_divout,q_divout1;


    div1 = VAD_L_div(l_speech_snr,l_speech_snr_count,16,0,&q_divout);
    div2 = VAD_L_div(l_silence_snr,l_silence_snr_count,16,0,&q_divout1);
    l_snr = VAD_L_ADD(div1,q_divout,L_negate(div2),q_divout1,&q_divout);

    *lf_snr_smooth = MUL_F(*lf_snr_smooth, CONST_16_Q15(0.9f));
    move32();
    tmp = MUL_F(l_snr, 26214);
    *lf_snr_smooth =  VAD_L_ADD( *lf_snr_smooth, 25, tmp, add(3,q_divout), &q_divout1);
    move32();
    *lf_snr_smooth =  L_shr(*lf_snr_smooth, sub(q_divout1, 25));
    move32();
    l_snr  = L_shr(l_snr, sub(q_divout, 25));

    test();
    if(( sub(bg_energy_count, 56) < 0) || (sub(fg_energy_count, 56) < 0 ))
    {
        l_snr = L_add(0, CONST_32_Q25(4.8f));
    }

    l_snr = MUL_F(l_snr, CONST_16_Q15(0.12f));
    l_snr = L_sub(l_snr, CONST_32_Q25(0.36));

    l_snr = L_max(0, l_snr);
    l_snr = L_min(l_snr, MAX_LF_SNR_TAB[bw_index]);

    *lf_snr = l_snr;
    move32();

}

void calc_lt_snr(T_CldfbVadState *st,              /*(io) vad state*/
                 Word32 *lt_snr_org_fp,          /*(o) original long time SNR*/
                 Word32 *lt_snr_fp,              /*(o) long time SNR calculated by fg_energy and bg_energy*/
                 Word32 fg_energy,               /*(i) foreground energy sum  */
                 Word16 fg_energy_count,         /*(i) amount of the foreground energy frame */
                 Word32 bg_energy,               /*(i) background energy sum  */
                 Word16 bg_energy_count,         /*(i) amount of the background energy frame */
                 Word16 bw_index,                /*(i) band width index*/
                 Word16 lt_noise_sp_center0      /*(i) long time noise spectral center by 0*/
                )
{
    Word16 tmp_lt_noise_sp_center;
    Word16 q_div1,q_div2,q_divout,q_divout1;
    Word32 lt_snr_org;
    Word32 lt_snr,div1,div2,tmp;


    tmp_lt_noise_sp_center = sub(lt_noise_sp_center0,CONST_16_Q10(1.4f));
    if(sub(tmp_lt_noise_sp_center, CONST_16_Q10(0.8)) > 0)
    {
        tmp_lt_noise_sp_center = CONST_16_Q10(0.8f);
        move16();
    }

    if(tmp_lt_noise_sp_center<0)
    {
        tmp_lt_noise_sp_center = 0;
        move16();
    }

    div1  = MUL_F(fg_energy,bg_energy_count);
    div1 = VAD_L_ADD(div1,st->fg_energy_scale, 1, 126 ,&q_div1);
    div2  = MUL_F(bg_energy,fg_energy_count);
    div2 = VAD_L_ADD(div2,st->bg_energy_scale, 1, 126 ,&q_div2);
    if( div2 == 0 )
    {
        div2 = 1;
        move32(); /* div2==0 , may occur for >30000 frames  all zero input  */
        if(div1 != 0)
        {
            st->bg_energy_scale = add(st->fg_energy_scale, 50);
        }
    }
    div2 = VAD_L_div(div1,div2,q_div1,q_div2,&q_divout);
    lt_snr_org  = VAD_Log2(div2,q_divout);
    lt_snr_org = MUL_F(lt_snr_org, 9864);
    lt_snr = L_add(0, lt_snr_org);
    *lt_snr_org_fp = lt_snr;
    move32();

    test();
    IF(sub(bg_energy_count, 56)<0||sub(fg_energy_count,56)<0)
    {
        lt_snr = L_add(0, CONST_32_Q25(2.1f));
    }

    IF(sub(bw_index, CLDFBVAD_NB_ID)== 0)
    {
        lt_snr = L_sub(L_shr(lt_snr,1), CONST_32_Q25(0.75f));
    }
    ELSE IF(sub(bw_index, CLDFBVAD_WB_ID)== 0)
    {
        lt_snr = L_sub(L_shr(lt_snr,1), CONST_32_Q25(0.75f));
    }
    ELSE
    {
        lt_snr = MUL_F(lt_snr, CONST_16_Q15(0.46f));
        lt_snr = L_sub(lt_snr, CONST_32_Q25(0.69f));
    }

    tmp = MUL_F(lt_snr,CONST_16_Q15(0.4f));
    tmp = L_add(L_shr(tmp,1), CONST_16_Q24(0.1f));
    tmp = MUL_F(tmp, CONST_16_Q15(0.4f));
    tmp = MUL_F(tmp,tmp_lt_noise_sp_center);
    lt_snr = VAD_L_ADD(lt_snr, 25, tmp,19, &q_divout1);
    lt_snr = L_shr(lt_snr, sub(q_divout1,25));

    lt_snr = L_max(0, lt_snr);

    if(L_sub(lt_snr,CONST_32_Q25(2.0))>0)
    {
        lt_snr = L_add(0, CONST_32_Q25(2.0));
    }

    *lt_snr_fp = lt_snr;
    move32();

}


void calc_snr_flux(
    Word32  tsnr,           /*(i)  time-domain SNR*/
    Word32 *pre_snr,        /*(io) time-domain SNR storage*/
    Word32 *snr_flux_fp     /*(o)  average tsnr*/
)
{
    Word32 i;
    Word32 tmp, snr_flux;
    Word16 s16MaxCoefNorm;
    /*save a new time-domain SNR to pre_snr[0]*/


    test();
    IF( (L_sub(L_shr(tsnr,1) , CONST_32_Q25(2.6f/2.0f))<0 )&&tsnr>0)
    {
        pre_snr[0] = tsnr;
        move32();
    }
    ELSE IF(tsnr <= 0)
    {
        pre_snr[0] = 0;
        move32();
    }
    ELSE
    {
        pre_snr[0] = CONST_32_Q25(2.6f);
        move32();
    }

    /*calculate snr_flux*/
    snr_flux = L_add(0, 0);
    s16MaxCoefNorm = sub(ffr_getSfWord32(pre_snr, 32), 5);
    FOR(i=0; i<32; i++)
    {
        tmp = L_shl(pre_snr[i],s16MaxCoefNorm);
        snr_flux = L_add(snr_flux, tmp);
    }
    snr_flux = L_shr(snr_flux,add(s16MaxCoefNorm,5));
    *snr_flux_fp = snr_flux;
    move32();

    /*update the tsnr storage pre_snr*/
    FOR(i=PRE_SNR_NUM-1; i>0; i--)
    {
        pre_snr[i] = pre_snr[i-1];
        move32();
    }

}



void snr_calc(T_CldfbVadState *st,          /*(io) vad state*/
              Word16  sacle_sbpower,  /*(i) the Scaling of sbpower*/
              Word32  *snr,           /*(o) frequency domain SNR */
              Word32  *tsnr,          /*(o) time domain SNR */
              Word32  frame_energy,   /*(i) current frame energy */
              Word32  bandwith        /*(i) band width*/
             )
{
    Word32 i;
    Word32 tmpframe_eg,tmpsb_eg,constff,div1,div2;
    Word32 snr_tmp, tmp;

    Word32 SNR_sb_num;
    Word32 *sb_bg_energy ;
    Word32 *frame_sb_energy ;
    Word32 t_bg_energy;
    Word16 tmp_addQ1,tmp_addQ2,minscale,minscale1,minscale2,s16MaxCoefNorm,q_divout;
    Word32 tmpspec_amp;
    Word32 const CONSTfix= 1759218560;
    Word32 snr_tmpidx[12] = {0};


    SNR_sb_num = add(0, SNR_SUB_BAND_NUM[bandwith-CLDFBVAD_NB_ID]);
    sb_bg_energy = st->sb_bg_energy;
    frame_sb_energy = st->frame_sb_energy;
    t_bg_energy = L_add(0, st->t_bg_energy);

    snr_tmp = L_add(0, 0);
    FOR(i=0; i<SNR_sb_num; i++)
    {
        div1 = VAD_L_ADD(frame_sb_energy[i], st->frame_sb_energy_scale, CONSTfix, 44, &tmp_addQ1);
        div2 = VAD_L_ADD(sb_bg_energy[i], st->sb_bg_energy_scale, CONSTfix, 44, &tmp_addQ2);
        tmp  = VAD_L_div(div1, div2, tmp_addQ1, tmp_addQ2, &q_divout);
        tmp  = VAD_Log2(tmp, q_divout);

        if(L_sub(tmp, CONST_32_Q25(-0.33219F))>0)
        {
            snr_tmpidx[i] = tmp;
            move32();
        }
    }

    s16MaxCoefNorm = sub(ffr_getSfWord32(snr_tmpidx, (Word16)SNR_sb_num), 4);
    FOR(i=0; i< SNR_sb_num; i++)
    {
        tmpspec_amp = L_shl(snr_tmpidx[i], s16MaxCoefNorm);
        snr_tmp = L_add(snr_tmp, tmpspec_amp);
    }
    snr_tmp = L_max(0, snr_tmp);
    snr_tmp = MUL_F(snr_tmp,BAND_MUL[bandwith-CLDFBVAD_NB_ID]);
    *snr = L_shr(snr_tmp,s16MaxCoefNorm);
    move32();

    IF(bandwith == CLDFBVAD_SWB_ID)
    {
        IF(t_bg_energy)
        {
            minscale =  norm_l(t_bg_energy);
            minscale2 = sub(s_min(add(minscale,st->scale_t_bg_energy),31),1);
            tmpsb_eg = L_shr(t_bg_energy,sub(st->scale_t_bg_energy,minscale2));
            constff = L_shr(1,sub(31,minscale2));
        }
        ELSE
        {
            tmpsb_eg = L_add(0, 0);
            constff = L_add(0, 1);
            minscale2 = 31;
            move16();
        }
        div2 = L_add(tmpsb_eg,constff);
        tmp = VAD_L_div(frame_energy, div2, sacle_sbpower, minscale2, &q_divout);
        IF(tmp)
        {
            minscale =  norm_l(tmp);
            minscale2 = sub(s_min(add(minscale,q_divout), 31), 1);
            tmpsb_eg = L_shr(tmp, limitScale32(sub(q_divout, minscale2)));
            constff = L_shr(1, sub(31, minscale2));
            tmp = L_add(tmpsb_eg, constff);
        }
        ELSE
        {
            tmp = L_add(0, 1);
            minscale2 = 31;
            move16();
        }
        *tsnr = VAD_Log2(tmp, minscale2);
        move32();
    }
    ELSE
    {
        IF(frame_energy)
        {
            minscale = norm_l(frame_energy);
            minscale1 = sub(s_min(add(minscale,sacle_sbpower),44),1);
            tmpframe_eg = L_shr(frame_energy,sub(sacle_sbpower,minscale1));
            constff = L_shr(CONSTfix,sub(44,minscale1));
        }
        ELSE
        {
            tmpframe_eg = L_add(0, 0);
            constff = L_add(0, CONSTfix);
            minscale1 = 44;
            move16();
        }
        div1 = L_add(tmpframe_eg, constff);
        IF(t_bg_energy)
        {
            minscale = norm_l(t_bg_energy);
            minscale2 = sub(s_min(add(minscale,st->scale_t_bg_energy),44),1);
            tmpsb_eg = L_shr(t_bg_energy,sub(st->scale_t_bg_energy,minscale2));
            constff = L_shr(CONSTfix,sub(44,minscale2));
        }
        ELSE
        {
            tmpsb_eg = L_add(0, 0);
            constff = L_add(0, CONSTfix);
            minscale2 = 44;
            move16();
        }
        div2 = L_add(tmpsb_eg, constff);
        tmp = VAD_L_div(div1, div2, minscale1, minscale2, &q_divout);
        *tsnr = VAD_Log2(tmp,q_divout);
        move32();
    }

}

Word32 construct_snr_thresh(   Word16 sp_center[],                   /*(i) spectral center*/
                               Word32 snr_flux,                      /*(i) snr flux*/
                               Word32 lt_snr,                        /*(i) long time time domain snr*/
                               Word32 l_snr,                         /*(i) long time frequency domain snr*/
                               Word32 continuous_speech_num,         /*(i) amount of continuous speech frames*/
                               Word16 continuous_noise_num,          /*(i) amount of continuous noise frames*/
                               Word32 fg_energy_est_start,           /*(i) whether if estimated energy*/
                               Word16 bw_index                       /*(i) band width index*/
                           )
{

    Word32 bw_snr,tmp_snr,snr_delta,test_l_snr,tmp,div1,div2;




    snr_delta = L_add(COMVAD_INIT_SNR_DELTA[bw_index],0);
    bw_snr = L_add(lt_snr,0);



    test_l_snr  = L_add(lt_snr,0);
    IF(sub(bw_index, CLDFBVAD_SWB_ID)== 0)
    {

        IF(sub(sp_center[3], CONST_16_Q10(2.80f))>0)
        {
            snr_delta = L_add(snr_delta,0);
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(2.6))>0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.03f));
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(1.6))>0)
        {
            snr_delta = L_add(snr_delta ,CONST_32_Q25(0.05f));
        }
        ELSE IF(sub(sp_center[3], CONST_16_Q10(1.4))>0)
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.10f));
        }
        ELSE
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.40f));
        }

        tmp = MUL_F(l_snr, CONST_16_Q15(0.1f));
        tmp = L_add(tmp,CONST_32_Q25(0.6));

        test();
        test();
        IF(L_sub(continuous_speech_num, 8) > 0&& L_sub(fg_energy_est_start, 1) ==0)
        {
            snr_delta  = L_sub(snr_delta, CONST_32_Q25(0.2f));
        }
        ELSE IF(sub(continuous_noise_num,12) > 0&&(L_sub(snr_flux, tmp)>0))
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.1f));
        }
        ELSE IF(sub(continuous_noise_num, 24) > 0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.2f));
        }
        ELSE IF((sub(continuous_noise_num, 4) > 0))
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.1f));
        }
    }
    ELSE IF(sub(bw_index, CLDFBVAD_WB_ID) == 0)
    {

        IF(sub(sp_center[3], CONST_16_Q10(2.80f))>0)
        {
            snr_delta = L_add(snr_delta,0);
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(2.6))>0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.03f));
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(1.6))>0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.05f));
        }
        ELSE IF(sub(sp_center[3], CONST_16_Q10(1.4))>0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.10f));
        }
        ELSE
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.30f));
        }

        tmp = MUL_F(bw_snr, CONST_16_Q15(0.1f));
        tmp = L_add(tmp,CONST_32_Q25(0.6));

        test();
        test();
        IF(L_sub(continuous_speech_num, 8) > 0 && L_sub(fg_energy_est_start, 1) == 0)
        {
            snr_delta = L_sub(snr_delta, CONST_32_Q25(0.10f));
        }
        ELSE IF(sub(continuous_noise_num,12)>0 && (L_sub(snr_flux,tmp) > 0))
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.10f));
        }
        ELSE IF(sub(continuous_noise_num,24) > 0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.20f));
        }
        ELSE IF((sub(continuous_noise_num,4) > 0))
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.10f));
        }
    }
    ELSE IF(sub(bw_index, CLDFBVAD_NB_ID)== 0)
    {

        IF(sub(sp_center[3], CONST_16_Q10(3.0))>0)
        {
            snr_delta = L_add(snr_delta,0);
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(2.6))>0)
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.02f));
        }
        ELSE IF(sub(sp_center[2],CONST_16_Q10(1.6))>0)
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.04f));
        }
        ELSE IF(sub(sp_center[2], CONST_16_Q10(1.46))>0)
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.10f));
        }
        ELSE
        {
            snr_delta = L_add(snr_delta , CONST_32_Q25(0.18f));
        }

        tmp = MUL_F(l_snr, CONST_16_Q15(0.1f));
        div1 = L_add(tmp,CONST_32_Q25(0.2));
        div2 = L_add(tmp,CONST_32_Q25(0.6));

        test();
        test();
        test();
        test();
        test();
        IF(L_sub(continuous_speech_num, 80) > 0 && L_sub(fg_energy_est_start, 1) == 0 && (sub(sp_center[0],CONST_16_Q10(1.4))>0))
        {
            snr_delta = L_sub(snr_delta, CONST_32_Q25(0.32f));
        }
        ELSE IF(L_sub(continuous_speech_num,8) > 0 && L_sub(fg_energy_est_start, 1)==0 && (L_sub(snr_flux,div1)>0))
        {
            snr_delta = L_sub(snr_delta, CONST_32_Q25(0.1f));
        }
        ELSE IF(sub(continuous_noise_num,12) > 0 && (L_sub(snr_flux,div2) >0))
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.10f));
        }
        ELSE IF(sub(continuous_noise_num, 24) > 0)
        {
            snr_delta = L_add(snr_delta, CONST_32_Q25(0.2f));
        }
    }
    ELSE
    {
        snr_delta = L_add(CONST_32_Q25(1.0f),0);
    }
    tmp_snr  = L_add(snr_delta, test_l_snr);


    return tmp_snr;
}

