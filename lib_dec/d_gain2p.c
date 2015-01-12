/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*-------------------------------------------------------------------*
 * Decoding of pitch and codebook gains  (see q_gain2_plus.c)        *
 *-------------------------------------------------------------------*/
#include <stdlib.h>
#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "rom_com_fx.h"


/*********************
 * private functions *
 *********************/
static Word32 calc_gcode0(
    Word16 *gcode0,
    Word16 *exp_gcode0
)
{
    Word32 L_tmp;

    /*gcode0 = (float)pow(10.0,(gcode0)*0.05);*/   /* predicted gain */

    L_tmp = L_mult(*gcode0, FL2WORD16(0.166096f));
    *exp_gcode0 = add(1,extract_l(L_shr(L_tmp, 24)));
    L_tmp = L_lshl(L_tmp, 7);
    L_tmp = L_and(0x7FFFFFFF, L_tmp);

    L_tmp = Pow2(30,round_fx(L_tmp));
    *gcode0 = round_fx(L_tmp);

    return L_tmp;
}

static Word32 calc_gain_code(Word16 g_code, Word16 gcode0, Word16 exp_gcode0)
{
    Word32 L_tmp;

    L_tmp = L_mult(g_code, gcode0);            /* Q11*Q15 -> Q27 */
    exp_gcode0 = add(exp_gcode0,-11);
    L_tmp = L_shl(L_tmp, exp_gcode0);       /*   Q27 -> Q16 */


    return L_tmp;
}

/*--------------------------------------------------------------------------*
* Mode2_gain_dec_mless
*
* Decoding of pitch and codebook gains without updating long term energies
*-------------------------------------------------------------------------*/

static void Mode2_gain_dec_mless(
    Word16 index,                /* i  : Quantization index vector        Q0  */
    Word16 *code,                /* i  : algebraic code excitation        Q9  */
    Word16 lcode,                /* i  : Subframe size                    Q0  */
    Word16 *gain_pit,            /* o  : Quantized pitch gain            1Q14 */
    Word32 *gain_code,           /* o  : Quantized codebook gain          Q16 */
    Word16 mean_ener,            /* i  : mean_ener defined in open-loop   Q8  */
    Word16 *past_gpit,           /* i/o: past gain of pitch              1Q14 */
    Word32 *past_gcode,          /* i/o: past energy of code              Q16 */
    Word16 *gain_inov,           /* o  : unscaled innovation gain        3Q12 */
    Word16 coder_type            /* i  : coder type for number of bits        */
)
{

    Word16 ener_code;
    const Word16 *t_qua_gain;
    Word16 exp_L_tmp1;
    Word16 gcode0, exp_gcode0;
    Word32 L_tmp, L_tmp1;



    /**gain_inov = 1.0f / (float)sqrt( ( dot_product( code, code, lcode ) + 0.01f ) / lcode);*/
    L_tmp = calc_gain_inov(code, lcode, &L_tmp1, &exp_L_tmp1);
    move16();
    *gain_inov = round_fx(L_shl(L_tmp, 15-3));  /* gain_inov in Q12 */

    /*-----------------------------------------------------------------*
     * Select the gains quantization table
     *-----------------------------------------------------------------*/
    t_qua_gain = E_ROM_qua_gain7b_const;

    if( coder_type == 0 )
    {
        t_qua_gain = E_ROM_qua_gain5b_const;
    }

    if(sub(coder_type,1) == 0)
    {
        t_qua_gain = E_ROM_qua_gain6b_const;
    }
    if(sub(coder_type,9) == 0)
    {
        t_qua_gain = E_ROM_qua_gain8b_const;
    }

    /*-----------------------------------------------------------------*
     * decode pitch gain
     *-----------------------------------------------------------------*/
    *gain_pit = t_qua_gain[index*2];

    /*-----------------------------------------------------------------*
     * calculate the predicted gain code
     *-----------------------------------------------------------------*/
    /*ener_code = 10 * log10((dot_product(code, code, lcode) + 0.01) / lcode) */
    L_tmp = BASOP_Util_Log2(L_tmp1);
    L_tmp = L_add(L_tmp,L_shl(L_deposit_l(exp_L_tmp1),31-LD_DATA_SCALE));

    L_tmp = Mpy_32_16_1(L_tmp, FL2WORD16((10.0f/3.3219280948873623478703194294894f)/4.0f));
    /* exponent of L_tmp = 6+2 */
    ener_code = round_fx(L_shl(L_tmp, 6+2-7)); /* Q8 */

    /* predicted codebook gain */
    gcode0 = sub(mean_ener, ener_code);            /* Q8 */

    /*gcode0 = (float)pow(10.0,(gcode0)*0.05);*/   /* predicted gain */

    calc_gcode0(&gcode0, &exp_gcode0);

    /*-----------------------------------------------------------------*
     * decode normalized codebook gain
     *-----------------------------------------------------------------*/
    /* *gain_code = t_qua_gain[index*2+1] * gcode0;*/

    L_tmp = calc_gain_code(t_qua_gain[index*2+1], gcode0, exp_gcode0);

    *gain_code = L_tmp;
    *past_gpit = *gain_pit;
    /**past_gcode = *gain_code / *gain_inov;   */
    /* Q16/Q12 => Q5 */
    L_tmp1 = L_deposit_h(BASOP_Util_Divide3216_Scale(L_tmp,*gain_inov,&exp_L_tmp1));
    *past_gcode = L_shl(L_tmp1,sub(exp_L_tmp1,15-12));



    return;
}

/*---------------------------------------------------------------------*
 * gain_dec_uv
 *
 * Decoding of pitch and codebook gains for Unvoiced mode
 *---------------------------------------------------------------------*/

static void gain_dec_uv(
    Word16 index,                /* i  : Quantization index vector        Q0  */
    Word16 *code,                /* i  : algebraic code excitation        Q9  */
    Word16 lcode,                /* i  : Subframe size                    Q0  */
    Word16 *gain_pit,            /* o  : Quantized pitch gain            1Q14 */
    Word32 *gain_code,           /* o  : Quantized codebook gain          Q16 */
    Word16 *past_gpit,           /* i/o: past gain of pitch              1Q14 */
    Word32 *past_gcode,          /* i/o: past energy of code              Q16 */
    Word16 *gain_inov            /* o  : unscaled innovation gain        3Q12 */
)
{
    Word16 i, exp_L_tmp1;
    Word32 L_tmp, L_tmp1;


    /*-----------------------------------------------------------------*
     * Innovation energy (without gain)
     *-----------------------------------------------------------------*/
    /* *gain_inov = 1.0f / (float)sqrt( ( dot_product( code, code, lcode ) + 0.01f ) / lcode );*/
    L_tmp = calc_gain_inov(code, lcode, &L_tmp1, &exp_L_tmp1);
    move16();
    *gain_inov = round_fx(L_shl(L_tmp, 15-3));  /* gain_inov in Q12 */

    /*-----------------------------------------------------------------*
    * Decode pitch gain
    *-----------------------------------------------------------------*/
    *gain_pit = 0;
    move16();

    /*-----------------------------------------------------------------*
     * Decode codebook gain
     *-----------------------------------------------------------------*/
    /* *gain_code= (float)pow(10.f,(((index*1.9f)-30.f)/20.f));*/
    L_tmp = L_mac(FL2WORD32_SCALE(-0.166096*30.0f, 7-1),shl(index, 16-7), FL2WORD16(0.166096f*1.9f));
    i = add(1,extract_l(L_shr(L_tmp, 25)));
    L_tmp = L_lshl(L_tmp, 6);
    L_tmp = L_and(0x7FFFFFFF, L_tmp);

    L_tmp = Pow2(30,round_fx(L_tmp));
    L_tmp = L_shl(L_tmp, i-(31-16)); /* Q16 */


    /*-----------------------------------------------------------------*
     * past gains for error concealment
     *-----------------------------------------------------------------*/
    *past_gpit = *gain_pit;
    *past_gcode = L_tmp;
    L_tmp = L_shl(Mpy_32_16_1(L_tmp, *gain_inov), 3); /* Q16*Q12 -> Q13 -> Q16 */
    *gain_code = L_tmp;
    move32();


    return;
}

/*---------------------------------------------------------------------*
 * gain_dec_gacelp_uv
 *
 * Decoding of pitch and codebook gains for Unvoiced mode
 *---------------------------------------------------------------------*/

static void gain_dec_gacelp_uv(
    Word16 index,                /* i  : Quantization index vector        Q0  */
    Word16 *code,                /* i  : algebraic code excitation        Q9  */
    Word16 *code2,               /* i  : algebraic code excitation        Q9  */
    Word16 mean_ener,            /* i  :                                 Q8  */
    Word16 lcode,                /* i  : Subframe size                    Q0  */
    Word16 *gain_pit,            /* o  : Quantized pitch gain            1Q14 */
    Word32 *gain_code,           /* o  : Quantized codebook gain          Q16 */
    Word32 *gain_code2,          /* o  : Quantized codebook gain          Q16 */
    Word16 *past_gpit,           /* i/o: past gain of pitch              1Q14 */
    Word32 *past_gcode,          /* i/o: past energy of code              Q16 */
    Word16 *gain_inov            /* o  : unscaled innovation gain        3Q12 */
)
{
    Word16 i, exp_L_tmp1;
    Word16 exp_gcode;
    Word16 g_code;
    Word32 L_tmp, L_tmp1;
    Word32 pred_nrg_frame;
    Word16 exp_gcode2, g_code2, norm_code2;
    Word16 index2, s;




    /* pred_nrg_frame = (float)pow(10.0,mean_ener/20.0); */
    L_tmp = L_mult(mean_ener, FL2WORD16(0.166096f * 2)); /* 6Q25 */
    pred_nrg_frame = BASOP_Util_InvLog2(L_sub(L_tmp, FL2WORD32_SCALE(15.f, 6))); /* 15Q16 */

    /*-----------------------------------------------------------------*
     * Prediction gains
     *-----------------------------------------------------------------*/
    /* gain_inov = 1.0f / sqrt((dot_product(code, code, L_SUBFR) + 0.01) / L_SUBFR) */
    L_tmp = calc_gain_inov(code, lcode, NULL, NULL);
    *gain_inov = round_fx(L_shl(L_tmp, 15-3));  /* gain_inov in Q12 */

    /* gcode = pred_nrg_frame * (*gain_inov); */
    L_tmp = Mpy_32_16_1(pred_nrg_frame, *gain_inov); /* 18Q13 */
    i = norm_l(L_tmp);
    g_code = round_fx(L_shl(L_tmp, i));
    exp_gcode = sub(18, i);

    /* norm_code2 = 1.0f / sqrt((dot_product(code2, code2, lcode) + 0.01f) / lcode); */
    L_tmp = calc_gain_inov(code2, lcode, NULL, NULL);
    norm_code2 = round_fx(L_shl(L_tmp, 15-3));  /* Q12 */

    /* g_code2 = pred_nrg_frame * norm_code2; */
    L_tmp = Mpy_32_16_1(pred_nrg_frame, norm_code2); /* 18Q13 */
    i = norm_l(L_tmp);
    g_code2 = round_fx(L_shl(L_tmp, i));
    exp_gcode2 = sub(18, i);

    /*-----------------------------------------------------------------*
     * Decode pitch gain
     *-----------------------------------------------------------------*/
    *gain_pit = 0;
    move16();
    *past_gpit = *gain_pit;
    move16();

    /*-----------------------------------------------------------------*
     * past gains for error concealment
     *-----------------------------------------------------------------*/
    index2=shr(index,5);
    index=s_and(index,0x1F);

    /**gain_code= (float)pow(10.f,(((index*1.25f)-20.f)/20.f))*gcode;*/

    L_tmp = L_mac(FL2WORD32_SCALE(-0.166096*20.0f, 7-1),shl(index, 16-7), FL2WORD16(0.166096f*1.25f));

    i = add(1,extract_l(L_shr(L_tmp, 25)));
    L_tmp = L_lshl(L_tmp, 6);
    L_tmp = L_and(0x7FFFFFFF, L_tmp);

    L_tmp = Pow2(30,round_fx(L_tmp));
    L_tmp = L_shl(L_tmp, i-(31-16)); /* Q16 */

    /* *past_gcode = L_tmp * pred_nrg_frame; */
    i = norm_l(L_tmp);
    L_tmp1 = L_shl(L_tmp, i);
    exp_L_tmp1 = sub(15, i);

    i = norm_l(pred_nrg_frame);
    L_tmp1 = Mpy_32_32(L_tmp1, L_shl(pred_nrg_frame, i));
    exp_L_tmp1 = add(exp_L_tmp1, sub(15, i));

    *past_gcode = L_shl(L_tmp1, sub(exp_L_tmp1, 15)); /* Q16 */              move32();

    *gain_code = L_shl(Mpy_32_16_1(*past_gcode, *gain_inov), 3);
    move32();


    L_tmp = Mpy_32_16_1(*gain_code, BASOP_Util_Divide1616_Scale(g_code2, g_code, &s));
    L_tmp = L_shl(L_tmp, sub(sub(add(s, exp_gcode2), exp_gcode), 2)); /* Q16 */
    L_tmp1 = L_add(L_tmp, 0);
    FOR (i = 0; i < index2; i++)
    {
        L_tmp1 = L_add(L_tmp1, L_tmp);
    }
    *gain_code2 = L_tmp1;


    return;
}

/*********************
 * public functions  *
 *********************/

void decode_acelp_gains(
    Word16 *code,                /* i  : algebraic code excitation        Q9  */
    Word16 gains_mode,
    Word16 mean_ener_code,       /* i  : mean_ener defined in open-loop   Q8  */
    Word16 *gain_pit,            /* o  : Quantized pitch gain            1Q14 */
    Word32 *gain_code,           /* o  : Quantized codebook gain          Q16 */
    Word16 **pt_indice,
    Word16 *past_gpit,           /* i/o: past gain of pitch              1Q14 */
    Word32 *past_gcode,          /* i/o: past energy of code              Q16 */
    Word16 *gain_inov,           /* o  : unscaled innovation gain        3Q12 */
    Word16 L_subfr,              /* i  : Subframe size                    Q0  */
    Word16 *code2,               /* i  : algebraic code excitation        Q9  */
    Word32 *gain_code2           /* o  : Quantized codebook gain          Q16 */
)
{
    Word16 index = 0;


    index = **pt_indice;
    (*pt_indice)++;

    IF ( s_or( s_and(gains_mode > 0, sub(gains_mode,4) < 0), sub(gains_mode,10) == 0) )
    {
        /* ACELP gains quantizer (5bits/subfr) */
        Mode2_gain_dec_mless(index, code, L_subfr, gain_pit, gain_code, mean_ener_code, past_gpit, past_gcode, gain_inov, gains_mode-1 );
    }
    ELSE IF (s_or(sub(gains_mode,4) == 0, sub(gains_mode,5) == 0))
    {
        /* AMR-WB gains quantizer (6bits/subfr (mode 2) or 7bits/subfr (mode 3)) */
        assert(0);
    }
    ELSE IF ( sub(gains_mode,6) == 0)
    {
        /* UV gains quantizer (6bits/subfr) */
        gain_dec_uv( index, code, L_subfr, gain_pit, gain_code, past_gpit, past_gcode, gain_inov );
    }
    ELSE IF (sub(gains_mode,7) == 0)
    {
        /* GACELP_UV gains quantizer (7=5-2bits/subfr) */
        gain_dec_gacelp_uv( index, code, code2, mean_ener_code, L_subfr, gain_pit, gain_code, gain_code2, past_gpit, past_gcode, gain_inov );
    }
    ELSE
    {
        fprintf(stderr, "invalid gains coding for acelp!\n");
        assert(0);
    }

}


/*---------------------------------------------------------------------*
 * d_gain_pred :
 *
 * decode the predicted value for the scaled
 * innovation energy in all subframes
 *---------------------------------------------------------------------*/
void d_gain_pred(
    Word16 nrg_mode,     /* i  : NRG moe                                   */
    Word16 *Es_pred,     /* o  : predicted scaled innovation energy    Q8  */
    Word16 **pt_indice   /* i/o: pointer to the buffer of indices          */
)
{
    Word16 indice;

    indice = (Word16)**pt_indice;
    (*pt_indice)++;

    *Es_pred = 0;
    move16();

    if( sub(nrg_mode,1) == 0 )
    {
        *Es_pred = Es_pred_qua[indice];
        move16();
    }

    if( sub(nrg_mode,2) == 0 )
    {
        *Es_pred = Es_pred_qua_2[indice];
        move16();
    }

    IF( sub(nrg_mode,2) > 0 )
    {
        move16();
        *Es_pred= extract_l(L_mac(FL2WORD32_SCALE( -20.f, 15-8), indice, FL2WORD16_SCALE( 1.75f, 15-7))); /*(Q8 - ((Q0*Q7)=Q8))*/
    }

    return;
}
