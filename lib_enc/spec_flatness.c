/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "vad_const.h"
#include "prot_fx.h"

void spec_flatness(Word32 *spec_amp,          /*(i) spectral amplitude*/
                   Word32 smooth_spec_amp[],  /*(i) smoothed spectral amplitude*/
                   Word16 sSFM[5]             /*(o) spectral flatness rate*/
                  )
{
    Word32 i;
    Word32 *smooth_spec_amp32;
    Word32 prods,prods_Exp;
    Word32 sums,prods_ExpM;
    Word32 zerop1;

    Word16 smooth_spec_amp16;
    Word16 Qnorm_prods,Qnorm_sums;
    Word16 SFM,min_sfm;
    Word16 prods_s,prods_Exps;
    Word16 prods_leadingzero,prods_Expleadingzero;

    Word16 leadingzero_prod, leadingzero_spec_amp;
    Word16 prods_Q_last,prods_ExpQ;
    Word16 SFM_Qtmp;
    Word16 prods_Q;


    smooth_spec_amp32 = smooth_spec_amp;
    zerop1 = L_add(0,0);
    prods_Q = 0;
    move16();

    FOR(i=5; i<65; i++)
    {
        smooth_spec_amp32[i] = L_add(MUL_F(smooth_spec_amp32[i],0x5999),MUL_F(spec_amp[i],0x2666));
        move32();
    }

    /*sSFM1*/
    sums = L_add(0,0);
    prods = L_add(1,0);
    prods_Q = 0;
    move16();

    FOR(i=5; i<20; i++)
    {
        sums = L_add(sums, smooth_spec_amp32[i]);
        leadingzero_spec_amp = norm_l(smooth_spec_amp32[i]);
        smooth_spec_amp16 = extract_h(L_shl(smooth_spec_amp32[i],leadingzero_spec_amp));
        leadingzero_prod = norm_l(prods);
        prods = L_shl(prods,leadingzero_prod);
        prods_s = extract_h(prods);
        prods = L_mult(prods_s, smooth_spec_amp16);
        prods_Q = add(add(prods_Q, leadingzero_spec_amp),leadingzero_prod);
    }
    prods_Q = sub(prods_Q, 255);

    prods_Q_last = prods_Q;
    move16();
    prods_ExpM = L_mult(prods_Q_last,-2184);

    prods = VAD_Pow(prods,0x08888888,0,31,&prods_Q);
    prods_Exp  = VAD_Pow2(prods_ExpM,16, &prods_ExpQ);

    prods_leadingzero = norm_l(prods);
    prods_Expleadingzero = norm_l(prods_Exp);
    prods_s = extract_h(L_shl(prods,prods_leadingzero));
    prods_Exps = extract_h(L_shl(prods_Exp,prods_Expleadingzero));

    prods = L_mult(prods_s, prods_Exps);
    prods_Q = add(prods_Q, prods_leadingzero);
    prods_Q = add(prods_Q, prods_ExpQ);
    prods_Q = add(prods_Q, prods_Expleadingzero);
    prods_Q = sub(prods_Q, 31);

    prods = L_max(prods, 0);

    if(prods <= 0)
    {
        prods_Q = 34;
        move16();
    }
    sums = MUL_F(sums, 0x0888);

    /*+0.1	*/
    IF (sub(prods_Q, 34) >= 0)
    {
        prods = L_shr(prods, sub(prods_Q, 33));
        zerop1 = L_shr(CNT0P1, 1);
        prods_Q = 33;
        move16();
    }
    ELSE
    {
        prods_Q = sub(prods_Q, 1);
        prods = L_shr(prods, 1);
        zerop1 = L_shr(CNT0P1, sub(34, prods_Q));
    }
    prods = L_add(prods, zerop1);

    zerop1 = L_shr(CNT0P1, 20);
    sums = L_add(sums, zerop1);

    /*div*/
    Qnorm_prods = sub(norm_l(prods), 1);
    Qnorm_sums = norm_l(sums);
    prods = L_shl(prods,Qnorm_prods);
    sums = L_shl(sums,Qnorm_sums);

    SFM = div_l(prods, extract_h(sums));

    SFM_Qtmp = add(prods_Q, Qnorm_prods);
    SFM_Qtmp = sub(SFM_Qtmp, Qnorm_sums);
    SFM_Qtmp = add(SFM_Qtmp, 15);
    SFM_Qtmp = sub(SFM_Qtmp, SPEC_AMP_Q);
    SFM_Qtmp = sub(SFM_Qtmp, SFM_Q);

    sSFM[0] = add(mult(sSFM[0], 0x6ccc), shr(mult(SFM, 0x1333), SFM_Qtmp));
    move16();

    /*sSFM2*/
    sums = L_add(0,0);
    prods = L_add(1,0);
    prods_Q = 0;
    move16();

    FOR(i=20; i<40; i++)
    {
        sums = L_add(sums, smooth_spec_amp32[i]);
        leadingzero_spec_amp = norm_l(smooth_spec_amp32[i]);
        smooth_spec_amp16 = extract_h(L_shl(smooth_spec_amp32[i],leadingzero_spec_amp));
        leadingzero_prod = norm_l(prods);
        prods = L_shl(prods,leadingzero_prod);
        prods_s = extract_h(prods);
        prods = L_mult(prods_s, smooth_spec_amp16);
        prods_Q = add(add(prods_Q, leadingzero_spec_amp),leadingzero_prod);
    }
    prods_Q = sub(prods_Q, 340);

    prods_Q_last = prods_Q;
    move16();
    prods_ExpM = L_mult(prods_Q_last,-1638);

    prods = VAD_Pow(prods,0x06666666,0,31,&prods_Q);
    prods_Exp  = VAD_Pow2(prods_ExpM,16, &prods_ExpQ);

    prods_leadingzero = norm_l(prods);
    prods_Expleadingzero = norm_l(prods_Exp);
    prods_s = extract_h(L_shl(prods,prods_leadingzero));
    prods_Exps = extract_h(L_shl(prods_Exp,prods_Expleadingzero));

    prods = L_mult(prods_s, prods_Exps);
    prods_Q = add(prods_Q, prods_leadingzero);
    prods_Q = add(prods_Q, prods_ExpQ);
    prods_Q = add(prods_Q, prods_Expleadingzero);
    prods_Q = sub(prods_Q, 31);

    prods = L_max(prods, 0);

    if(prods<=0)
    {
        prods_Q = 34;
        move16();
    }
    sums = MUL_F(sums, 0x0666);

    /*+0.1	*/
    IF (sub(prods_Q, 34) >= 0)
    {
        prods = L_shr(prods,sub(prods_Q, 33));
        zerop1 = L_shr(CNT0P1,1);
        prods_Q = 33;
        move16();
    }
    ELSE
    {
        prods_Q = sub(prods_Q,1);
        prods = L_shr(prods,1);
        zerop1 = L_shr(CNT0P1, sub(34, prods_Q));
    }
    prods = L_add(prods, zerop1);

    zerop1 = L_shr(CNT0P1, 20);
    sums = L_add(sums, zerop1);

    /*div*/
    Qnorm_prods=sub(norm_l(prods), 1);
    Qnorm_sums=norm_l(sums);
    prods = L_shl(prods, Qnorm_prods);
    sums = L_shl(sums, Qnorm_sums);

    SFM = div_l(prods, extract_h(sums));

    SFM_Qtmp = add(prods_Q, Qnorm_prods);
    SFM_Qtmp = sub(SFM_Qtmp, Qnorm_sums);
    SFM_Qtmp = add(SFM_Qtmp, 15);
    SFM_Qtmp = sub(SFM_Qtmp, SPEC_AMP_Q);
    SFM_Qtmp = sub(SFM_Qtmp, SFM_Q);

    sSFM[1] = add(mult(sSFM[1],0x6ccc),shr(mult(SFM,0x1333),SFM_Qtmp));
    move16();

    min_sfm = sSFM[1];
    move16();

    /*sSFM3*/
    sums = L_add(0,0);
    prods = L_add(1,0);
    prods_Q = 0;
    move16();

    FOR(i=40; i<65; i++)
    {
        sums = L_add(sums, smooth_spec_amp32[i]);
        leadingzero_spec_amp = norm_l(smooth_spec_amp32[i]);
        smooth_spec_amp16 = extract_h(L_shl(smooth_spec_amp32[i],leadingzero_spec_amp));
        leadingzero_prod = norm_l(prods);
        prods = L_shl(prods,leadingzero_prod);
        prods_s = extract_h(prods);
        prods = L_mult(prods_s, smooth_spec_amp16);
        prods_Q = add(add(prods_Q, leadingzero_spec_amp),leadingzero_prod);
    }
    prods_Q = sub(prods_Q, 425);

    prods_Q_last = prods_Q;
    move16();
    prods_ExpM = L_mult(prods_Q_last,-1310);

    prods = VAD_Pow(prods,0x051eb851,0,31,&prods_Q);
    prods_Exp = VAD_Pow2(prods_ExpM,16, &prods_ExpQ);

    prods_leadingzero = norm_l(prods);
    prods_Expleadingzero = norm_l(prods_Exp);
    prods_s = extract_h(L_shl(prods,prods_leadingzero));
    prods_Exps = extract_h(L_shl(prods_Exp,prods_Expleadingzero));

    prods = L_mult(prods_s, prods_Exps);
    prods_Q = add(prods_Q, prods_leadingzero);
    prods_Q = add(prods_Q, prods_ExpQ);
    prods_Q = add(prods_Q, prods_Expleadingzero);
    prods_Q = sub(prods_Q, 31);

    prods = L_max(prods, 0);
    if(prods <= 0)
    {
        prods_Q = 34;
        move16();
    }

    sums = MUL_F(sums, 0x051e);

    /*+0.1	*/
    IF (sub(prods_Q, 34)>=0)
    {
        prods = L_shr(prods,sub(prods_Q, 33));
        zerop1 = L_shr(CNT0P1,1);
        prods_Q = 33;
        move16();
    }
    ELSE
    {
        prods_Q = sub(prods_Q, 1);
        prods = L_shr(prods,1);
        zerop1 = L_shr(CNT0P1, sub(34, prods_Q));
    }
    prods = L_add(prods, zerop1);

    zerop1 = L_shr(CNT0P1, 20);
    sums = L_add(sums, zerop1);

    /*div*/
    Qnorm_prods = sub(norm_l(prods), 1);
    Qnorm_sums = norm_l(sums);
    prods = L_shl(prods,Qnorm_prods);
    sums = L_shl(sums,Qnorm_sums);

    SFM = div_l(prods,extract_h(sums));

    SFM_Qtmp = add(prods_Q, Qnorm_prods);
    SFM_Qtmp = sub(SFM_Qtmp, Qnorm_sums);
    SFM_Qtmp = add(SFM_Qtmp, 15);
    SFM_Qtmp = sub(SFM_Qtmp, SPEC_AMP_Q);
    SFM_Qtmp = sub(SFM_Qtmp, SFM_Q);

    sSFM[2] = add(mult(sSFM[2],0x6ccc),shr(mult(SFM,0x1333),SFM_Qtmp));
    move16();

    if(sub(min_sfm,sSFM[2])>0)
    {
        min_sfm = sSFM[2];
        move16();
    }
    sSFM[4] = min_sfm;
    move16();

}



















