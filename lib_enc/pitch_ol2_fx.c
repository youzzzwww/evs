/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_enc_fx.h"    /* Encoder static table prototypes        */
#include "rom_com_fx.h"    /* Encoder static table prototypes        */
#include "rom_dec_fx.h"
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define MAX_DELTA    16                   /* half-length of the delta search      */
#define COR_BUF_LEN  (L_INTERPOL1*2 + MAX_DELTA*2 + 1)

/*-------------------------------------------------------------------*
 * pitch_ol2()
 *
 * Open-loop pitch precision improvement with 1/4 resolution
 * The pitch is searched in the interval <pitch_ol-delta, pitch_ol+delta),
 * i.e. the value pitch_ol + delta is not a part of the interval
 *-------------------------------------------------------------------*/
void pitch_ol2_fx(
    const Word16 pit_min,        /* i  : minimum pitch value (20 or 29)                   */
    const Word16 pitch_ol,		 /* i  : pitch to be improved                             */
    Word16 *pitch_fr_fx,   /* o  : adjusted 1/4 fractional pitch                    */ /*Q7*/
    Word16 *voicing_fr_fx, /* o  : adjusted 1/4 fractional voicing                  */ /*Q15*/
    const Word16 pos,			 /* i  : position in frame where to calculate the improv. */
    const Word16 *wsp_fx,        /* i  : weighted speech for current frame and look-ahead */ /*Q_new-1+shift*/
    const Word16 delta           /* i  : delta for pitch search (2 or 7)                  */
)
{
    Word16 i, t, step, fraction, t0_min, t0_max, t_min, t_max;
    const Word16 *pt_wsp_fx;
    Word16 wsp_fr_fx[L_SUBFR];
    Word16 temp_fx, cor_max_fx, cor_fx[COR_BUF_LEN], *pt_cor_fx;
    Word32 cor_32[COR_BUF_LEN], *pt_cor_32, t0, t1;
    Word16 t0s, t1s;
    Word16 exp3;
    Word32 R1, R2;
    Word16 R0, exp_R0, exp_R1, exp_R2, j;
    Word16 pit_max;

    /* initialization */
    pit_max = PIT_MAX_EXTEND;
    move16();
    t0_min = sub(pitch_ol, delta);
    t0_max = add(pitch_ol, sub(delta, 1));
    t0_min = s_max(t0_min, pit_min);
    t_min = sub(t0_min, L_INTERPOL1);

    t0_max = s_min(t0_max, pit_max);
    t_max = add(t0_max, L_INTERPOL1);

    pt_wsp_fx = wsp_fx + pos;
    pt_cor_32 = cor_32;
    t1 = L_deposit_l(0);
    FOR (t=t_min; t<=t_max; t++)
    {
        t0 = Dot_product(pt_wsp_fx, pt_wsp_fx-t, L_SUBFR);
        *pt_cor_32++ = t0;
        move32();
        t0 = L_abs(t0);
        t1 = L_max(t1, t0);
    }
    exp3 = norm_l(t1);
    pt_cor_32 = cor_32;
    pt_cor_fx = cor_fx;
    FOR ( t=t_min; t<=t_max; t++ )
    {
        t0 = L_shl(*pt_cor_32++, exp3);
        *pt_cor_fx++ = round_fx(t0);
    }

    pt_cor_fx = cor_fx + L_INTERPOL1;
    cor_max_fx = *pt_cor_fx++;
    move16();
    t1s = t0_min;
    move16();
    FOR (t=add(t0_min, 1); t<=t0_max; t++)
    {
        if (sub(*pt_cor_fx, cor_max_fx) > 0)
        {
            t1s = t;
            move16();
        }
        cor_max_fx = s_max(cor_max_fx, *pt_cor_fx);
        pt_cor_fx++;
    }

    /*----------------------------------------------------------------*
     * Search fractionnal pitch with 1/4 subsample resolution.
     * search the fractions around t0 and choose the one which maximizes
     * the interpolated normalized correlation.
     *----------------------------------------------------------------*/
    pt_cor_fx = cor_fx + sub(L_INTERPOL1, t0_min);
    t0s = t1s;
    move16();

    step = 1;
    move16();                     /* 1/4 subsample resolution */
    fraction = 1;
    move16();

    IF (sub(t0s, t0_min) != 0) /* Process negative fractions */
    {
        t0s = sub(t0s,1);
        cor_max_fx = Interpol_4(&pt_cor_fx[t0s], fraction);
        move16();
        FOR (i=add(fraction, step); i<=3; i+=step)
        {
            temp_fx = Interpol_4(&pt_cor_fx[t0s], i);
            if (sub(temp_fx, cor_max_fx) > 0)
            {
                fraction = i;
                move16();
            }
            cor_max_fx = s_max(temp_fx, cor_max_fx);
        }
    }
    ELSE /* Limit case */
    {
        fraction = 0;
        move16();
        cor_max_fx = Interpol_4(&pt_cor_fx[t0s], fraction);
        move16();
    }
    FOR (i = 0; i<=3; i+=step)     /* Process positive fractions */
    {
        temp_fx = Interpol_4(&pt_cor_fx[t1s], i);
        move16();

        IF (sub(temp_fx,cor_max_fx) > 0)
        {
            cor_max_fx = temp_fx;
            move16();
            fraction = i;
            move16();
            t0s = t1s;
            move16();
        }
    }

    *pitch_fr_fx = shl(add(shl(t0s, 2), fraction), 4);
    move16();			/*Q7*/

    pred_lt4(pt_wsp_fx, wsp_fr_fx, t0s, fraction, L_SUBFR, pitch_inter4_1, 4, PIT_UP_SAMP);

    R0 = cor_max_fx;
    move16();
    R1 = L_mult(pt_wsp_fx[0], pt_wsp_fx[0]);
    R2 = L_mult(wsp_fr_fx[0], wsp_fr_fx[0]);
    FOR (j = 1; j < L_SUBFR; j++)
    {
        R1 = L_mac(R1, pt_wsp_fx[j], pt_wsp_fx[j]);
        R2 = L_mac(R2, wsp_fr_fx[j], wsp_fr_fx[j]);
    }

    /* *voicing_fr = cor_max * inv_sqrt(enr_wsp * enr_old) */
    /* *voicing_fr = R0 * inv_sqrt(R1 * R2) */
    exp_R0 = norm_s(R0);
    R0 = shl(R0, exp_R0);

    exp_R1 = norm_l(R1);
    R1 = L_shl(R1, exp_R1);

    exp_R2 = norm_l(R2);
    R2 = L_shl(R2, exp_R2);

    R1 = L_mult(round_fx(R1), round_fx(R2));

    i = norm_l(R1);
    R1 = L_shl(R1, i);

    exp_R1 = add(exp_R1, exp_R2);
    exp_R1 = add(exp_R1, i);
    exp_R1 = sub(62, exp_R1);

    R1 = Isqrt_lc(R1, &exp_R1);

    R1 = L_mult(R0, round_fx(R1));
    exp_R0 = sub(31, exp_R0);
    exp_R0 = sub(add(exp_R0, exp_R1),exp3);

    *voicing_fr_fx = round_fx(L_shl(R1, exp_R0));	 	/*Q15*/

    return;
}
/*-------------------------------------------------------------------*
 * StableHighPitchDetect()
 *
 * stable very short pitch detection
 *-------------------------------------------------------------------*/
void StableHighPitchDetect_fx(
    Word16 *flag_spitch,       /* o  : flag to indicate very short stable pitch */
    Word16 pitch[],            /* i/o: OL pitch buffer                         */
    const Word16 voicing[],          /* i  : OL pitch gains                          */
    const Word16 wsp[],              /* i  : weighted speech                         */
    const Word16 localVAD,
    Word16 *voicing_sm,        /* i/o: smoothed open-loop pitch gains          */
    Word16 *voicing0_sm,       /* i/o: smoothed high pitch gains               */
    Word16 *LF_EnergyRatio_sm, /* i/o: smoothed [0, 300Hz] relative peak energy*/
    Word16 *predecision_flag,  /* i/o: predecision flag                        */
    Word32 *diff_sm,           /* i/o: smoothed pitch frequency difference     */
    Word32 *energy_sm  ,        /* i/o: smoothed energy around pitch frequency  */
    Word16 Q_new,
    Word16 EspecdB[]
)
{
    Word16 i, pitch_freq_point;
    Word16 T, Tp, pit_min;
    Word16 energy0_16, energy1_16,ratio, voicing_m;
    Word32 energy0, energy1, cor_max, diff, sum_energy;
    const Word16 *pt_wsp;
    Word16 tmp,tmp1,exp , diff16,cor_max16 ,exp1,exp2,pit_min_up;
    Word32 L_tmp,L_tmp1;
    Word16 Top;

    /*voicing = (voicing[0] + voicing[1] + voicing[2] )/3;*/
    L_tmp = L_mult(voicing[0],10923);
    L_tmp = L_mac(L_tmp, voicing[1],10923);
    L_tmp = L_mac(L_tmp, voicing[2],10923);
    voicing_m = round_fx(L_tmp);

    /**voicing_sm = 0.75f*(*voicing_sm) + 0.25f*voicing;*/
    *voicing_sm = round_fx(L_mac(L_mult(*voicing_sm , 24576), voicing_m , 8192));

    /* pitch_freq_point = (short)(L_FFT/(mult_fact*T_op[1])+0.5f);*/
    Top = pitch[1];
    exp = norm_s(Top);
    tmp = div_s(shl(1,sub(14,exp)), Top);/*Q(29 - exp)*/
    L_tmp = L_mult0(tmp ,L_FFT);/*Q(29 - exp)*/
    pitch_freq_point = extract_h(L_add (L_shl(L_tmp , sub(exp, 13)),32768)) ; /* Q0*/
    diff = L_deposit_l(0);
    sum_energy = L_deposit_l(0);
    FOR( i=1; i<2*pitch_freq_point; i++ )
    {
        diff = L_add(diff ,sub(EspecdB[pitch_freq_point], EspecdB[i]));
        sum_energy = L_add(sum_energy, EspecdB[i]);
    }
    /*sum_energy /= (2*pitch_freq_point-1);*/
    tmp = sub(shl(pitch_freq_point,1) ,1);
    exp = norm_s(tmp);
    tmp1 = div_s(shl(1,sub(14,exp)),tmp);/*Q(29-exp)*/
    L_tmp = Mult_32_16(sum_energy ,tmp1);
    sum_energy = L_shl(L_tmp, sub(exp,14));
    /**diff_sm = 0.2f * diff  + 0.8f * *diff_sm;*/
    *diff_sm = L_add(Mult_32_16(diff,6554),Mult_32_16(*diff_sm,26214));
    move32();
    /**energy_sm = 0.2f * sum_energy + 0.8f * *energy_sm;*/
    *energy_sm = L_add(Mult_32_16(sum_energy,6554),Mult_32_16(*energy_sm,26214));
    move32();
    /*diff /= sum_energy;*/

    IF(sum_energy)
    {
        exp = norm_l(sum_energy);
        tmp = extract_h(L_shl(sum_energy,exp));
        exp = sub(sub(30,exp),7);
        IF(tmp < 0)
        {
            tmp = abs_s(tmp);
            tmp = div_s(16384,tmp); /*Q(15+exp)*/
            BASOP_SATURATE_WARNING_OFF
            diff = L_negate( L_shr(Mult_32_16(diff,tmp),sub(exp+7 ,31) ));
            BASOP_SATURATE_WARNING_ON
            diff16 = round_fx(diff);
        }
        ELSE
        {
            tmp = div_s(16384,tmp); /*Q(15+exp)*/
            BASOP_SATURATE_WARNING_OFF
            diff = L_shr(Mult_32_16(diff,tmp),sub(exp+7 ,31));
            BASOP_SATURATE_WARNING_ON
            diff16 = round_fx(diff);
        }
    }
    ELSE
    {
        diff16 = round_fx(L_shl(diff , 25));
    }
    test();
    test();
    IF( L_sub(*diff_sm , -1280)<0 && L_sub( *energy_sm , 4928)< 0 && sub( diff16 , - 26214) <0 )
    {
        *predecision_flag = 1;
        move16();
    }
    test();
    test();
    if(  L_sub(*diff_sm ,1280)>0 && L_sub( *energy_sm , 10624)>0 &&  sub(diff16 ,16384)>0 )
    {
        *predecision_flag = 0;
        move16();
    }

    /* short pitch possiblity pre-decision */
    maximum_fx(EspecdB, 7, &energy0_16);
    maximum_fx(EspecdB+8, 7, &energy1_16);
    ratio = s_max(sub(energy1_16,energy0_16),0); /*Q7 */
    /*ratio *= max(voicing,0);*/
    tmp = s_max(voicing_m,0);
    ratio = mult_r(ratio,tmp);/*Q7*/
    /**LF_EnergyRatio_sm = (15*(*LF_EnergyRatio_sm) + ratio)/16;*/
    L_tmp = L_mult(ratio,2048);
    L_tmp = L_mac(L_tmp, *LF_EnergyRatio_sm,30720);
    *LF_EnergyRatio_sm = round_fx(L_tmp);
    test();
    if( sub(*LF_EnergyRatio_sm , 4480)>0 || sub( ratio ,6400)>0 )
    {
        *predecision_flag=1;
        move16();
    }

    if(  sub(*LF_EnergyRatio_sm , 2048)<0 )
    {
        *predecision_flag=0;
        move16();
    }

    /* short pitch candidate detection */
    Tp = pitch[1];
    move16();
    cor_max = 0;
    move16();
    pt_wsp = wsp + 3*L_SUBFR;
    pit_min = PIT_MIN_DOUBLEEXTEND;
    move16();
    pit_min_up = PIT_MIN;
    move16();
    FOR( T=pit_min; T<=pit_min_up; T++ )
    {
        energy1 = Dot_product( pt_wsp, pt_wsp-T, L_SUBFR  );
        test();
        IF( (L_sub(energy1,cor_max)>0) || (sub(T,pit_min) ==0) )
        {
            cor_max = L_add(energy1, 0);
            Tp = T;
            move16();
        }
    }
    energy0 = Dot_product12( pt_wsp, pt_wsp, L_SUBFR , &exp1 ) ;
    exp1 = sub(exp1 ,shl(Q_new,1));
    energy1 = Dot_product12( pt_wsp-Tp, pt_wsp-Tp, L_SUBFR ,&exp2) ;
    exp2 = sub(exp2 ,shl(Q_new,1));
    /* cor_max *= inv_sqrt( energy0*energy1 );*/
    L_tmp = Mult_32_32(energy0,energy1);
    exp = norm_l(L_tmp);
    L_tmp1 = L_shl(L_tmp,exp);

    exp = 31-exp-(31-exp1 -exp2);
    move16();
    L_tmp1 = Isqrt_lc(L_tmp1, &exp); /*Q(31-exp)*/
    cor_max = Mult_32_32(cor_max, L_tmp1);
    exp = 31 - (shl(Q_new,1)+1) - (31-exp) +31;
    cor_max16 = round_fx(L_shl(cor_max, exp));  /*Q15*/
    /**voicing0_sm = add(mult_r(24576 ,(*voicing0_sm)) , mult_r(8192 , cor_max16));*/
    *voicing0_sm = round_fx(L_mac(L_mult(24576 ,*voicing0_sm), 8192 , cor_max16));

    /* final short pitch detection */
    test();
    test();
    test();
    *flag_spitch = 0;
    move16();
    IF( (sub(localVAD,1)==0) && (sub(*predecision_flag,1)==0) &&
        (sub(*voicing0_sm,16384)>0) && (sub(*voicing0_sm, mult_r(*voicing_sm,21299))>0 ))
    {
        *flag_spitch = 1;
        move16();
        pitch[0] = Tp;
        move16();
        pitch[1] = Tp;
        move16();
        pitch[2] = Tp;
        move16();
    }

    return;
}
