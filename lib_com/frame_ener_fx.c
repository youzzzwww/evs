/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*----------------------------------------------------------------------------------*
 * frame_ener()
 *
 * Estimation of pitch-synchronous (voiced) or mean half-frame (unvoiced) energy
 *----------------------------------------------------------------------------------*/
Word16 frame_ener_fx(
    const Word16 L_frame,   /* i  : length of the frame                            */
    const Word16 clas,      /* i  : frame classification                           */
    const Word16 *synth,    /* i  : synthesized speech at Fs = 12k8 Hz       Q_new */
    const Word16 pitch,     /* i  : pitch period                             Q0    */
    Word32 *enr_q,    /* o  : pitch-synchronous or half_frame energy   Q0    */
    const Word16 offset,    /* i  : speech pointer offset (0 or L_FRAME)           */
    const Word16 Q_new,     /* i  : Scaling factor                                 */
    Word16 shift,     /* i  : Shift need to obtain 12 bits vectors           */
    const Word16 enc        /* i  : Encoder/decoder                                */
)
{
    Word16 len, exp_enrq, exp_tmp, pos;
    Word16 i;
    const Word16 *pt_synth;
    Word32 Ltmp;

    exp_enrq = 0;
    move16();
    test();
    test();
    IF( (sub(clas, VOICED_CLAS) == 0) || (sub(clas, ONSET) == 0) || (sub(clas, SIN_ONSET) == 0) )              /* current frame is voiced */
    {
        /* current frame is voiced */
        len = pitch;
        move16(); /* pitch value at the end of frame */
        pt_synth = synth;
        move16();
        if (offset != 0)
        {
            pt_synth = synth + sub(L_frame, len);
        }
        emaximum_fx(Q_new, pt_synth, len, enr_q);
        move16();/* pitch synchronous E */
        IF (enc != 0)
        {
            exp_enrq  = norm_l(*enr_q);
            *enr_q    = L_shl(*enr_q, exp_enrq);
            move32();
            exp_enrq  = sub(exp_enrq, 2);
        }
    }
    ELSE
    {
        /* current frame is unvoiced */
        pos = 0;
        move16();

        if (offset != 0)
        {
            pos = sub(L_frame, 2*L_SUBFR);
        }
        Ltmp = L_mult(synth[pos], synth[pos]);
        FOR (i = 1; i < 2*L_SUBFR; i++)
        {
            Ltmp = L_mac(Ltmp, synth[pos+i], synth[pos+i]);
        }
        test();
        IF (L_sub(Ltmp, MAX_32) == 0 || enc != 0)
        {
            /* scale down when overflow occurs */
            *enr_q = Energy_scale(synth+pos, 2*L_SUBFR, shift, &exp_enrq);
            move32();
        }
        ELSE
        {
            shift = 0;
            move16();
            /* Normalize acc in Q31 (energy already calculated) */
            pos = norm_l(Ltmp);
            Ltmp = L_shl(Ltmp, pos);
            exp_enrq = sub(30, pos); /* exponent = 0..30 */
            *enr_q = Ltmp;
            move32();
        }

        /* enr2 = 1.0f/L_FRAME2 * dot_product(synth, synth, L_FRAME2) */
        exp_enrq = sub(exp_enrq, shl(shift, 1));

        IF (enc != 0)
        {
            exp_tmp = add(shl(Q_new, 1), -2+7); /*  L_subfr == L_SUBFR */
            exp_enrq = sub(exp_enrq, exp_tmp);
            exp_enrq = sub(31, exp_enrq);
        }
        ELSE
        {
            exp_enrq = sub(exp_enrq, add(Q_new, Q_new));
            exp_enrq = sub(exp_enrq, 31+7);     /* 7 -> 1/L_FRAME2 */
            *enr_q = L_shl(*enr_q, exp_enrq);
            move32();
            *enr_q  = L_add(*enr_q, 1);
            move32();
        }
    }

    return exp_enrq;
}

/*------------------------------------------------------------------------*
 * frame_energy()
 *
 * Compute pitch-synchronous energy at the frame end
 *------------------------------------------------------------------------*/
Word16 frame_energy_fx(        /* o  : Frame energy in                               Q8 */
    Word16 L_frame,
    const Word16 *pitch,       /* i  : pitch values for each subframe                Q6 */
    const Word16 *speech,      /* i  : pointer to speech signal for E computation  Q_syn*/
    const Word16 lp_speech,    /* i  : long term active speech energy average      Q8   */
    Word16 *frame_ener,  /* o  : pitch-synchronous energy at frame end       Q8   */
    const Word16 Q_syn         /* i  : Synthesis scaling                                */
)
{
    Word32 Ltmp;
    const Word16 *pt1;
    Word16 tmp16, exp1, frac1, tmp1, tmp2;
    Word16 len, enern;

    /* len = (0.5f * (pitch[2]/64.0 + pitch[3]/64.0) + 0.5f) */
    len = mult_r(add(pitch[2], pitch[3]), 256);

    if(sub(len,L_SUBFR) < 0 )
    {
        len = shl(len, 1);
    }
    pt1 = speech + sub(L_frame,len);

    /* *frame_ener = 10.0f * log10(dot_product(pt1, pt1, len) / (float)len) */

    tmp1 = norm_s(len);
    tmp2 = shl(len, tmp1);
    tmp1 = sub(15, tmp1);

    tmp16 = extract_h(Dot_product12(pt1, pt1, len, &exp1));

    /* frac1 is -1 if tmp16 > tmp2 */
    frac1 = shr(sub(tmp2, tmp16), 15);
    tmp16 = shl(tmp16, frac1);
    exp1 = sub(exp1, frac1);
    tmp16 = div_s(tmp16, tmp2);
    exp1 = sub(exp1, tmp1);
    Ltmp = L_deposit_h(tmp16);
    tmp16 = norm_l(Ltmp);
    frac1 = Log2_norm_lc(L_shl(Ltmp, tmp16));
    tmp16 = sub(30, tmp16);

    exp1 = sub(add(exp1, 31+1-3), add(tmp16, shl(Q_syn, 1)));
    frac1 = mult_r(frac1, tmp2);
    Ltmp = Mpy_32_16(exp1, frac1, LG10);

    /* enern = *frame_ener - lp_speech */

    *frame_ener = extract_l(L_shr(Ltmp, 14-8));
    enern = sub(*frame_ener, lp_speech);

    return enern;
}
