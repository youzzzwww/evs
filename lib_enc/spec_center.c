/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"
#include "rom_enc_fx.h"



void spec_center(Word32* sb_power,  /*(i) energy of sub-band divided uniformly*/
                 Word16* sp_center, /*(o) spectral center*/
                 Word32  bandwith,  /*(i) band width*/
                 Word16  Q_sb_p     /*(i) the Scaling of sb_power*/
                )
{
    Word16 i;

    Word32 t_sp_center, frame_power;
    Word32 t_sp_center_num, frame_power_den;
    Word32 sb_power_mlt;
    Word32 t_sp_center_nb, frame_power_nb;
    Word32 zerop1;
    Word32 sb_power_shr[24];

    Word16 d_t_sp_center;
    Word16 Q_t_sp_center, Q_frame_power;
    Word16 Q_t_sc,Q_f_p;
    Word16 d_t_sp_center_Qtmp;


    zerop1 = L_add(0, 0);
    t_sp_center = L_add(0, 0);
    frame_power = L_add(0, 0);

    FOR (i=0; i<10; i++)
    {
        sb_power_shr[i] = L_shr(sb_power[i],5);
        move32();
    }

    FOR(i=0; i<10; i++)
    {
        sb_power_mlt = Mpy_32_16_1(sb_power[i],i_t_1[i]);
        t_sp_center = L_add(L_shr(sb_power_mlt,6), t_sp_center);
        frame_power = L_add(sb_power_shr[i], frame_power);/*0-9 */
    }

    t_sp_center_nb = L_add(0, t_sp_center);
    frame_power_nb = L_add(0, frame_power);

    /*+0.1	*/
    Q_t_sc = sub(Q_sb_p, 10);
    IF (sub(Q_t_sc,34)>=0)
    {
        t_sp_center = L_shr(t_sp_center,sub(Q_t_sc, 33));
        zerop1 = L_shr(CNT0P1,1);
        Q_t_sc = 33;
        move16();
    }
    ELSE
    {
        Q_t_sc = sub(Q_t_sc, 1);
        t_sp_center = L_shr(t_sp_center,1);
        zerop1 = L_shr(CNT0P1, sub(34, Q_t_sc));
    }
    t_sp_center_num = L_add(t_sp_center, zerop1);

    Q_f_p = sub(Q_sb_p, 5);
    IF (sub(Q_f_p,34)>=0)
    {
        frame_power = L_shr(frame_power,sub(Q_f_p, 33));
        zerop1 = L_shr(CNT0P1,1);
        Q_f_p = 33;
        move16();
    }
    ELSE
    {
        Q_f_p = sub(Q_f_p, 1);
        frame_power = L_shr(frame_power,1);
        zerop1 = L_shr(CNT0P1, sub(34, Q_f_p));
    }

    frame_power_den = L_add(frame_power, zerop1);
    IF (frame_power == 0)
    {
        frame_power_den = L_add(CNT0P1, 0);
        Q_f_p = 34;
        move16();
    }
    /*div */
    Q_t_sp_center = sub(norm_l(t_sp_center_num), 1);
    Q_frame_power = norm_l(frame_power_den);
    t_sp_center_num = L_shl(t_sp_center_num,Q_t_sp_center);
    frame_power_den = L_shl(frame_power_den,Q_frame_power);

    d_t_sp_center = div_l(t_sp_center_num,extract_h(frame_power_den));

    d_t_sp_center_Qtmp = add(Q_t_sc,Q_t_sp_center);
    d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_frame_power);
    d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_f_p);
    d_t_sp_center_Qtmp = add(d_t_sp_center_Qtmp,15);
    d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,SP_CENTER_Q);

    d_t_sp_center = shr(d_t_sp_center,d_t_sp_center_Qtmp);
    sp_center[0] = add(mult(sp_center[0],0x5999),mult(d_t_sp_center,0x2666));
    move16();
    sp_center[2] = d_t_sp_center;
    move16();

    t_sp_center = L_add(0, 0);
    frame_power = L_add(0, 0);
    IF(L_sub(bandwith,CLDFBVAD_WB_ID)==0)
    {
        FOR (i=10; i<20; i++)
        {
            sb_power_shr[i] = L_shr(sb_power[i],5);
            move32();
        }

        FOR(i=1; i<20; i++)
        {
            sb_power_mlt = Mpy_32_16_1(sb_power[i],i_t_2[i-1]);
            t_sp_center = L_add(L_shr(sb_power_mlt,8), t_sp_center);
            frame_power = L_add(sb_power_shr[i], frame_power);/*1-19 */
        }
        /*+0.1	*/
        Q_t_sc = sub(Q_sb_p, 13);

        IF (sub(Q_t_sc, 34) >= 0)
        {
            t_sp_center = L_shr(t_sp_center,sub(Q_t_sc, 33));
            zerop1 = L_shr(CNT0P1,1);
            Q_t_sc = 33;
            move16();
        }
        ELSE
        {
            Q_t_sc = sub(Q_t_sc, 1);
            t_sp_center = L_shr(t_sp_center,1);
            zerop1 = L_shr(CNT0P1, s_min(31, sub(34, Q_t_sc)));
        }
        t_sp_center_num = L_add(t_sp_center, zerop1);

        Q_f_p = sub(Q_sb_p, 5);
        IF (sub(Q_f_p,34)>=0)
        {
            frame_power = L_shr(frame_power, sub(Q_f_p, 33));
            zerop1 = L_shr(CNT0P1,1);
            Q_f_p = 33;
            move16();
        }
        ELSE
        {
            Q_f_p = sub(Q_f_p, 1);
            frame_power = L_shr(frame_power,1);
            zerop1 = L_shr(CNT0P1, sub(34, Q_f_p));
        }
        frame_power_den = L_add(frame_power, zerop1);
        IF (frame_power == 0)
        {
            frame_power_den = L_add(0, CNT0P1);
            Q_f_p = 34;
            move16();
        }

        /*div */
        Q_t_sp_center = sub(norm_l(t_sp_center_num), 1);
        Q_frame_power = norm_l(frame_power_den);
        t_sp_center_num = L_shl(t_sp_center_num,Q_t_sp_center);
        frame_power_den = L_shl(frame_power_den,Q_frame_power);

        d_t_sp_center = div_l(t_sp_center_num,extract_h(frame_power_den));

        d_t_sp_center_Qtmp = add(Q_t_sc,Q_t_sp_center);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_frame_power);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_f_p);
        d_t_sp_center_Qtmp = add(d_t_sp_center_Qtmp,15);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,SP_CENTER_Q);

        sp_center[3]= shr(d_t_sp_center,d_t_sp_center_Qtmp);
        move16();
    }
    ELSE IF(L_sub(bandwith, CLDFBVAD_SWB_ID)==0)
    {
        FOR (i=10; i<24; i++)
        {
            sb_power_shr[i] = L_shr(sb_power[i],5);
            move32();
        }

        FOR(i=1; i<24; i++)
        {
            sb_power_mlt = Mpy_32_16_1(sb_power[i],i_t_2[i-1]);
            t_sp_center = L_add(L_shr(sb_power_mlt,9), t_sp_center);
            frame_power = L_add(sb_power_shr[i], frame_power);/*1-23 */
        }

        /*+0.1	*/
        Q_t_sc = sub(Q_sb_p, 14);

        IF (sub(Q_t_sc,34)>=0)
        {
            t_sp_center = L_shr(t_sp_center, limitScale32(sub(Q_t_sc, 33)));
            zerop1 = L_shr(CNT0P1,1);
            Q_t_sc = 33;
            move16();
        }
        ELSE
        {
            Q_t_sc = sub(Q_t_sc, 1);
            t_sp_center = L_shr(t_sp_center,1);
            zerop1 = L_shr(CNT0P1, limitScale32(sub(34, Q_t_sc)));
        }
        t_sp_center_num = L_add(t_sp_center, zerop1);

        Q_f_p = sub(Q_sb_p, 5);
        IF (sub(Q_f_p,34)>=0)
        {
            frame_power = L_shr(frame_power,sub(Q_f_p, 33));
            zerop1 = L_shr(CNT0P1,1);
            Q_f_p = 33;
            move16();
        }
        ELSE
        {
            Q_f_p = sub(Q_f_p, 1);
            frame_power = L_shr(frame_power,1);
            zerop1 = L_shr(CNT0P1, sub(34, Q_f_p));
        }
        frame_power_den = L_add(frame_power, zerop1);
        IF (frame_power == 0)
        {
            frame_power_den = L_add(0, CNT0P1);
            Q_f_p = 34;
            move16();
        }

        /*div*/
        Q_t_sp_center=sub(norm_l(t_sp_center_num), 1);
        Q_frame_power=norm_l(frame_power_den);
        t_sp_center_num = L_shl(t_sp_center_num,Q_t_sp_center);
        frame_power_den = L_shl(frame_power_den,Q_frame_power);

        d_t_sp_center = div_l(t_sp_center_num,extract_h(frame_power_den));

        d_t_sp_center_Qtmp = add(Q_t_sc,Q_t_sp_center);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_frame_power);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,Q_f_p);
        d_t_sp_center_Qtmp = add(d_t_sp_center_Qtmp,15);
        d_t_sp_center_Qtmp = sub(d_t_sp_center_Qtmp,SP_CENTER_Q);
        sp_center[3]= shr(d_t_sp_center,d_t_sp_center_Qtmp);
        move16();
    }
    ELSE
    {
        t_sp_center = L_add(0, t_sp_center_nb);
        frame_power = L_add(0, frame_power_nb);

        IF (frame_power==0)
        {
            d_t_sp_center = 0;
            move16();
        }
        ELSE
        {
            /*div*/
            Q_t_sp_center = sub(norm_l(t_sp_center), 1);
            Q_frame_power = norm_l(frame_power);
            t_sp_center_num = L_shl(t_sp_center,Q_t_sp_center);
            frame_power_den = L_shl(frame_power,Q_frame_power);

            d_t_sp_center = div_l(t_sp_center_num,extract_h(frame_power_den));
            d_t_sp_center = shr(d_t_sp_center,sub(add(sub(Q_t_sp_center,Q_frame_power),10),SP_CENTER_Q));
        }

        sp_center[0] = add(mult(sp_center[0], 0x5999), mult(d_t_sp_center, 0x2666));
        sp_center[2] = d_t_sp_center;
        move16();
        move16();

        t_sp_center = L_add(0, 0);
        frame_power = L_add(0, 0);
        FOR(i=1; i<10; i++)
        {
            sb_power_mlt = Mpy_32_16_1(sb_power[i],i_t_1[i-1]);
            t_sp_center = L_add(L_shr(sb_power_mlt,6), t_sp_center);
            frame_power = L_add(sb_power_shr[i], frame_power);	/*1-9	*/
        }
        IF (frame_power==0)
        {
            sp_center[3] = 0;
            move16();
        }
        ELSE
        {
            /*div */
            Q_t_sp_center = sub(norm_l(t_sp_center), 1);
            Q_frame_power = norm_l(frame_power);
            t_sp_center_num = L_shl(t_sp_center,Q_t_sp_center);
            frame_power_den = L_shl(frame_power,Q_frame_power);

            d_t_sp_center = div_l(t_sp_center_num,extract_h(frame_power_den));
            sp_center[3] = shr(d_t_sp_center,sub(add(sub(Q_t_sp_center, Q_frame_power), 10),SP_CENTER_Q));
            move16();
        }
    }

}
