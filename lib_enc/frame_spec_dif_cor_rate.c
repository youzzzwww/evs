/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <limits.h>

#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"



void frame_spec_dif_cor_rate(T_CldfbVadState *st,             /*(io) vad state*/
                             Word32 *spec_amp,              /*(i) spectral amplitude*/
                             Word16 sacle,                  /*(i) the scaling of spec_amp*/
                             Word16 f_tonality_rate[3]		/*(o) tonality rate*/
                            )

{
    Word32 i,tmp;
    Word16 spec_low_dif_tmp,tmpq,tmpq2,*p_dx_Q,dx_Q=0;
    Word32 tmpspec_low_dif[PRE_SPEC_DIF_NUM];
    Word32 maxVal;
    Word16 resu;
    Word16 scalefactor,spec_dif_cor_rate;
    Word32 m,dx,dy;
    Word16 *pre_spec_low_dif = st->pre_spec_low_dif;
    const Word32  FIX_cost1 = FIX_32(0.001f);



    p_dx_Q = &dx_Q;
    maxVal = L_add(0,0);
    FOR(i=0; i< PRE_SPEC_DIF_NUM; i++)
    {
        tmp = L_sub(spec_amp[i+6] ,spec_amp[i+5]);
        if ( tmp  < 0 )
        {
            tmp = L_add(0,0);
        }
        tmpspec_low_dif[i] = tmp;
        move32();
        maxVal = L_max(maxVal,tmp);
    }
    resu = 31;
    move16();
    if ( maxVal )
    {
        resu = norm_l(maxVal);
    }

    m  = L_add(0,0);
    dx = L_add(0,0);
    dy = L_add(0,0);

    scalefactor =  sub(resu,3);

    FOR (i = 0; i < PRE_SPEC_DIF_NUM; i++)
    {
        spec_low_dif_tmp =  round_fx(L_shl(tmpspec_low_dif[i],scalefactor));
        m = L_mac0(m,spec_low_dif_tmp,pre_spec_low_dif[i]);
        dx =  L_mac0(dx, spec_low_dif_tmp, spec_low_dif_tmp);
        dy  =  L_mac0(dy, pre_spec_low_dif[i], pre_spec_low_dif[i]);
        pre_spec_low_dif[i] = spec_low_dif_tmp;
        move16();
    }
    dx = L_mult0(round_fx(dx),round_fx(dy));
    tmpq = add(sacle,scalefactor);
    tmpq = sub(tmpq,16);
    tmpq2 = add(tmpq,st->scale_spec_low_dif);
    *p_dx_Q = shl(tmpq2,1);
    move16();
    *p_dx_Q = sub(*p_dx_Q ,32);
    move16();
    IF(sub(*p_dx_Q , 31)<0)
    {
        dx = L_add(dx,L_shr(FIX_cost1,limitScale32(sub(31,*p_dx_Q))));
    }
    ELSE
    {
        dx = L_add(L_shr(dx,limitScale32(sub(*p_dx_Q,31))),FIX_cost1);
        *p_dx_Q  = 31;
        move16();
    }

    dx = vad_Sqrt_l(dx,p_dx_Q);

    m = L_shr(m,limitScale32(add(sub(tmpq2,*p_dx_Q),1)));
    spec_dif_cor_rate = 16384;
    move16();

    if(dx)
    {
        spec_dif_cor_rate = divide3232(m,dx);
    }

    f_tonality_rate[0] = spec_dif_cor_rate;
    move16();
    tmp =  L_mac0(L_mult0(f_tonality_rate[1],CONST_16_Q15(0.96f)), spec_dif_cor_rate,CONST_16_Q15(0.04f));
    f_tonality_rate[1] = shl(round_fx(tmp),1);
    move16();
    tmp =  L_mac0(L_mult0(f_tonality_rate[2],CONST_16_Q15(0.90f)), spec_dif_cor_rate,CONST_16_Q15(0.1f));
    f_tonality_rate[2] = shl(round_fx(tmp),1);
    move16();
    st->scale_spec_low_dif = tmpq;
    move16();

}

