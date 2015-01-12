/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"

void ltd_stable(T_CldfbVadState *st,       /*(io) vad state*/
                Word16 *ltd_stable_rate, /*(o) time-domain stable rate*/
                Word32 frame_energy,     /*(i) current frame energy*/
                Word16 frameloop,        /*(i) amount of  frames*/
                Word16 Q_frames_power    /*(i) the Scaling of  frames_power*/
               )
{

    Word32 i;
    Word32 zerop001,maxVal;
    Word32 mid_frame_amp_mul,tmp_mul;
    Word32 *frames_power_32;
    Word32 frame_energy_Sqr32;
    Word32 seg_amp32;
    Word32 mid_frame_ampadd32[20];
    Word16 tmp16[20];
    Word32 mid_frame_amp32[20];
    Word32 dif32, apow32;
    Word32 tmp32[20];

    Word16 Q_frames_power_last_32;
    Word16 Q_dif,Q_apow;
    Word16 frame_energy_Sqr;
    Word16 Q_frames_power32;
    Word16 leadingzero_tmp32;
    Word16 leadingzero_midamp;
    Word16 Qsum_dif32,Qsum_apow32;
    Word16 ltd_stable_rate_Qtmp;
    Word16 scale1;
    Word32 seg_amp32tmp;
    Word16 tmp;

    zerop001 = L_add(0, 0);
    Q_dif = 0;
    move16();
    Q_apow = 0;
    move16();
    frames_power_32 = st->frames_power_32;
    Q_frames_power_last_32 = st->Q_frames_power_32;
    move16();
    leadingzero_midamp = 31;
    move16();
    leadingzero_tmp32 = 31;
    move16();
    Q_frames_power32 = Q_frames_power;
    move16();

    frame_energy_Sqr = FixSqrt(frame_energy, &Q_frames_power32);
    frame_energy_Sqr32 = L_deposit_l(frame_energy_Sqr);
    frame_energy_Sqr32 = L_shl(frame_energy_Sqr32,16);
    Q_frames_power32 = add(Q_frames_power32, 16);

    /* +0.1	 	*/
    IF (sub(Q_frames_power32, 40) >= 0)
    {
        zerop001 = L_shr(CNT0P001, 1);
        frame_energy_Sqr32 = L_shr(frame_energy_Sqr32,sub(Q_frames_power32, 39));
        Q_frames_power32 = 39;
        move16();
    }
    ELSE
    {
        Q_frames_power32 = sub(Q_frames_power32, 1);
        frame_energy_Sqr32 = L_shr(frame_energy_Sqr32,1);
        zerop001 = L_shr(CNT0P001, sub(40, Q_frames_power32));
    }
    frames_power_32[0] = L_add(frame_energy_Sqr32, zerop001);
    move32();

    IF(sub(frameloop, 3) < 0)
    {
        FOR(i=1; i<40; i++)
        {
            frames_power_32[i] = frames_power_32[0];
            move32();
        }
    }
    ELSE
    {
        Word16 leadingzero;
        maxVal = L_add(0, 0);
        FOR(i=1; i<40; i++)
        {
            maxVal = L_max(maxVal,frames_power_32[i]);
        }
        leadingzero = 31;
        move16();
        if(maxVal)
            leadingzero = norm_l(maxVal);
        Q_frames_power_last_32 = add(Q_frames_power_last_32, leadingzero);


        IF (sub(Q_frames_power_last_32,Q_frames_power32)>0)
        {
            scale1 = sub(Q_frames_power_last_32, Q_frames_power32);
            scale1 = sub(scale1, leadingzero);
            FOR(i=1; i<40; i++)
            {
                frames_power_32[i] = L_shr(frames_power_32[i],scale1);
                move32();
            }
        }
        ELSE
        {
            scale1 = sub(Q_frames_power32, Q_frames_power_last_32);
            frames_power_32[0] = L_shr(frames_power_32[0],scale1);
            move32();
            Q_frames_power32 = Q_frames_power_last_32;
            move16();
            FOR(i=1; i<40; i++)
            {
                frames_power_32[i] = L_shl(frames_power_32[i], leadingzero);
                move32();
            }
        }
    }

    FOR(i=0; i<20; i++)
    {
        mid_frame_ampadd32[i] = L_add(L_shr(frames_power_32[2*i], 1), L_shr(frames_power_32[2*i+1], 1));
        move32();
    }

    maxVal = L_add(0, 0);
    FOR(i=0; i<20; i++)
    {
        maxVal = L_max(maxVal,mid_frame_ampadd32[i]);
    }
    leadingzero_midamp = 31;
    move16();
    if(maxVal)
        leadingzero_midamp = norm_l(maxVal);

    FOR(i=0; i<20; i++)
    {
        mid_frame_amp32[i] = L_shl(mid_frame_ampadd32[i], leadingzero_midamp);
        move32();
    }

    seg_amp32 = L_add(0, 0);
    FOR(i=0; i<20; i++)
    {
        seg_amp32 = L_add(seg_amp32, L_shr(mid_frame_amp32[i], 5));
    }
    seg_amp32 = MUL_F(seg_amp32, 0x0666);

    dif32 = L_add(0, 0);
    apow32 = L_add(0, 0);

    seg_amp32tmp = L_shl(seg_amp32, 5);
    FOR(i=0; i<20; i++)
    {
        tmp32[i] = L_sub(mid_frame_amp32[i],seg_amp32tmp);
        move32();
    }

    maxVal = L_add(0, 0);
    FOR(i=0; i<20; i++)
    {
        maxVal = L_max(maxVal,L_abs(tmp32[i]));
    }
    leadingzero_tmp32 = 31;
    move16();
    if(maxVal)
        leadingzero_tmp32 = norm_l(maxVal);

    FOR(i=0; i<20; i++)
    {
        tmp16[i] = extract_h(L_shl(tmp32[i], leadingzero_tmp32));
    }

    FOR(i=0; i<20; i++)
    {
        tmp_mul = L_mult(tmp16[i],tmp16[i]);
        tmp_mul = L_shr(tmp_mul,5);
        dif32 = L_add(dif32,tmp_mul);

        tmp = extract_h(mid_frame_amp32[i]);
        mid_frame_amp_mul = L_mult(tmp,tmp);
        mid_frame_amp_mul = L_shr(mid_frame_amp_mul,5);
        apow32 = L_add(apow32,mid_frame_amp_mul);
    }


    IF (dif32==0)
    {
        ltd_stable_rate[5] = 0;
        move16();
    }
    ELSE
    {
        Q_dif = sub(norm_l(dif32), 1);
        Q_apow = norm_l(apow32);
        dif32 = L_shl(dif32,Q_dif);
        apow32 = L_shl(apow32,Q_apow);

        ltd_stable_rate[5] = div_l(dif32,extract_h(apow32));
        move16();
    }

    ltd_stable_rate_Qtmp = sub(Q_dif, Q_apow);
    ltd_stable_rate_Qtmp = add(ltd_stable_rate_Qtmp, leadingzero_tmp32);
    ltd_stable_rate_Qtmp = add(ltd_stable_rate_Qtmp, leadingzero_tmp32);
    ltd_stable_rate_Qtmp = add(ltd_stable_rate_Qtmp, 15);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, ITD_STABLE_RATE_Q);

    ltd_stable_rate_Qtmp = limitScale16(ltd_stable_rate_Qtmp);

    ltd_stable_rate[5] = shr(ltd_stable_rate[5],ltd_stable_rate_Qtmp);
    move16();


    maxVal = L_add(0, 0);
    FOR(i=0; i<14; i++)
    {
        maxVal = L_max(maxVal, L_abs(mid_frame_ampadd32[i]));
    }
    leadingzero_midamp = 31;
    move16();
    if(maxVal)
        leadingzero_midamp = norm_l(maxVal);

    FOR(i=0; i<14; i++)
    {
        mid_frame_amp32[i] = L_shl(mid_frame_ampadd32[i],leadingzero_midamp);
        move32();
    }

    seg_amp32 = L_add(0, 0);
    FOR(i=0; i<14; i++)
    {
        seg_amp32 = L_add(seg_amp32, L_shr(mid_frame_amp32[i],4));
    }
    seg_amp32 = MUL_F(seg_amp32, 0x0924);

    dif32 = L_add(0, 0);
    apow32 = L_add(0, 0);
    seg_amp32tmp = L_shl(seg_amp32, 4);
    FOR(i=0; i<14; i++)
    {
        tmp32[i] = L_sub(mid_frame_amp32[i], seg_amp32tmp);
        move32();
    }

    maxVal = L_add(0, 0);
    FOR(i=0; i<14; i++)
    {
        maxVal = L_max(maxVal,L_abs(tmp32[i]));
    }
    leadingzero_tmp32 = 31;
    move16();
    if(maxVal)
        leadingzero_tmp32 = norm_l(maxVal);

    FOR(i=0; i<14; i++)
    {
        tmp32[i] = L_shl(tmp32[i], leadingzero_tmp32);
        move32();
    }

    FOR(i=0; i<14; i++)
    {
        tmp = extract_h(tmp32[i]);
        tmp_mul = L_mult(tmp, tmp);
        tmp_mul = L_shr(tmp_mul, 4);
        dif32 = L_add(dif32, tmp_mul);

        tmp = extract_h(mid_frame_amp32[i]);
        mid_frame_amp_mul = L_mult(tmp,tmp);
        mid_frame_amp_mul = L_shr(mid_frame_amp_mul, 4);
        apow32 = L_add(apow32, mid_frame_amp_mul);
    }

    Qsum_apow32 = add(Q_frames_power32, Q_frames_power32);
    Qsum_apow32 = add(Qsum_apow32, leadingzero_midamp);
    Qsum_apow32 = add(Qsum_apow32, leadingzero_midamp);
    Qsum_apow32 = sub(Qsum_apow32, 37);

    Qsum_dif32 = add(Qsum_apow32, leadingzero_tmp32);
    Qsum_dif32 = add(Qsum_dif32, leadingzero_tmp32);

    /* +0.1	 	*/
    IF (sub(Qsum_apow32,44)>=0)
    {
        zerop001 = L_shr(CNT0P0001, 1);
        apow32 = L_shr(apow32,limitScale32(sub(Qsum_apow32,43)));
        Qsum_apow32 = 43;
        move16();
    }
    ELSE
    {
        Qsum_apow32 = sub(Qsum_apow32, 1);
        apow32 = L_shr(apow32, 1);
        zerop001 = L_shr(CNT0P0001, limitScale32(sub(44, Qsum_apow32)));
    }
    apow32 = L_add(apow32, zerop001);
    IF (apow32 == 0)
    {
        apow32 = L_add(0, CNT0P0001);
        Qsum_apow32 = 44;
        move16();
    }

    IF (dif32 == 0)
    {
        ltd_stable_rate[1] = 0;
        move16();
    }
    ELSE
    {
        Q_dif=sub(norm_l(dif32), 1);
        Q_apow=norm_l(apow32);
        dif32 = L_shl(dif32,Q_dif);
        apow32 = L_shl(apow32,Q_apow);

        ltd_stable_rate[1] = div_l(dif32,extract_h(apow32));
        move16();
    }

    ltd_stable_rate_Qtmp = add(Qsum_dif32, Q_dif);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, Qsum_apow32);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, Q_apow);
    ltd_stable_rate_Qtmp = add(ltd_stable_rate_Qtmp, 15);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, ITD_STABLE_RATE_Q);

    ltd_stable_rate_Qtmp = limitScale16(ltd_stable_rate_Qtmp);

    ltd_stable_rate[1] = shr(ltd_stable_rate[1],ltd_stable_rate_Qtmp);
    move16();

    maxVal = L_add(0, 0);
    FOR(i=0; i<8; i++)
    {
        maxVal = L_max(maxVal,L_abs(mid_frame_ampadd32[i]));
    }
    leadingzero_midamp = 31;
    move16();
    if(maxVal)
        leadingzero_midamp = norm_l(maxVal);

    FOR(i=0; i<8; i++)
    {
        mid_frame_amp32[i] = L_shl(mid_frame_ampadd32[i], leadingzero_midamp);
        move32();
    }

    seg_amp32 = L_add(0, 0);
    FOR(i=0; i<8; i++)
    {
        seg_amp32 = L_add(seg_amp32, L_shr(mid_frame_amp32[i], 3));
    }
    seg_amp32 = MUL_F(seg_amp32, 0x1000);

    dif32 = L_add(0, 0);
    apow32 = L_add(0, 0);
    seg_amp32tmp = L_shl(seg_amp32, 3);
    FOR(i=0; i<8; i++)
    {
        tmp32[i] = L_sub(mid_frame_amp32[i], seg_amp32tmp);
        move32();
    }

    maxVal = L_add(0, 0);
    FOR(i=0; i<8; i++)
    {
        maxVal = L_max(maxVal,L_abs(tmp32[i]));
    }
    leadingzero_tmp32 = 31;
    move16();
    if(maxVal)
        leadingzero_tmp32 = norm_l(maxVal);

    FOR(i=0; i<8; i++)
    {
        tmp32[i] = L_shl(tmp32[i],leadingzero_tmp32);
        move32();
    }

    FOR(i=0; i<8; i++)
    {
        tmp = extract_h(tmp32[i]);
        tmp_mul = L_mult(tmp,tmp);
        tmp_mul = L_shr(tmp_mul, 3);
        dif32 = L_add(dif32, tmp_mul);

        tmp = extract_h(mid_frame_amp32[i]);
        mid_frame_amp_mul = L_mult(tmp,tmp);
        mid_frame_amp_mul = L_shr(mid_frame_amp_mul,3);
        apow32 = L_add(apow32, mid_frame_amp_mul);
    }

    Qsum_apow32 = add(Q_frames_power32,Q_frames_power32);
    Qsum_apow32 = add(Qsum_apow32,leadingzero_midamp);
    Qsum_apow32 = add(Qsum_apow32,leadingzero_midamp);
    Qsum_apow32 = sub(Qsum_apow32,36);

    Qsum_dif32 = add(Qsum_apow32,leadingzero_tmp32);
    Qsum_dif32 = add(Qsum_dif32,leadingzero_tmp32);

    /* +0.1	 	*/
    IF (sub(Qsum_apow32,44) >= 0)
    {
        zerop001 = L_shr(CNT0P0001, 1);
        apow32 = L_shr(apow32,limitScale32(sub(Qsum_apow32,43)));
        Qsum_apow32 = 43;
        move16();
    }
    ELSE
    {
        Qsum_apow32 = sub(Qsum_apow32, 1);
        apow32 = L_shr(apow32,1);
        zerop001 = L_shr(CNT0P0001, limitScale32(sub(44, Qsum_apow32)));
    }
    apow32 = L_add(apow32, zerop001);
    IF (apow32 == 0)
    {
        apow32 = L_add(0, CNT0P0001);
        Qsum_apow32 = 44;
        move16();
    }

    IF (dif32 == 0)
    {
        ltd_stable_rate[2] = 0;
        move16();
    }
    ELSE
    {
        Q_dif = sub(norm_l(dif32), 1);
        Q_apow = norm_l(apow32);
        dif32 = L_shl(dif32, Q_dif);
        apow32 = L_shl(apow32, Q_apow);

        ltd_stable_rate[2] = div_l(dif32, extract_h(apow32));
        move16();
    }

    ltd_stable_rate_Qtmp = add(Qsum_dif32, Q_dif);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, Qsum_apow32);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, Q_apow);
    ltd_stable_rate_Qtmp = add(ltd_stable_rate_Qtmp, 15);
    ltd_stable_rate_Qtmp = sub(ltd_stable_rate_Qtmp, ITD_STABLE_RATE_Q);

    ltd_stable_rate_Qtmp = limitScale16(ltd_stable_rate_Qtmp);

    ltd_stable_rate[2] = shr(ltd_stable_rate[2], ltd_stable_rate_Qtmp);
    move16();
    ltd_stable_rate[3] = add(mult(ltd_stable_rate[3], 0x7333), mult(ltd_stable_rate[2], 0x0ccc));
    move16();

    FOR(i=55; i>0; i--)
    {
        frames_power_32[i] = frames_power_32[i-1];
        move32();
    }
    st->Q_frames_power_32 = Q_frames_power32;
    move16();

}
