/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"


/*===========================================================================*/
/* FUNCTION : Es_pred_dec_fx()												 */
/*---------------------------------------------------------------------------*/
/* PURPOSE : Decoding of scaled predicted innovation energy to be			 */
/*			 used in all subframes											 */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :														 */
/* _ (Word16) coder_type	     : coder type								 */
/* _ (Word32) core_brate		 : core bitrate						         */
/* _ (Word16*) Es_pred_qua_nb_fx : Gain quantization - quantization table    */
/*								   for scaled innovation energy prediciton Q8*/
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													     */
/* _ (Word16*)	Es_pred			:  predicited scaled innovation energy  Q8	 */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														 */
/* _ None																	 */
/*===========================================================================*/
void Es_pred_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    Word16 *Es_pred,          /* o  : predicited scaled innovation energy       Q8*/
    const Word16 coder_type,        /* i  : coder type                                  */
    const  Word32 core_brate        /* i  : core bitrate                               */
)
{
    Word16 enr_idx, nb_bits;
    {
        nb_bits = Es_pred_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, -1, -1)];
        move16();
        enr_idx = (Word16)get_next_indice_fx( st_fx, nb_bits );
        move16();
    }
    SWITCH ( nb_bits )
    {
    case 5:
        *Es_pred = Es_pred_qua_5b_fx[enr_idx];
        move16();
        BREAK;
    case 4:
        *Es_pred = Es_pred_qua_4b_fx[enr_idx];
        move16();
        BREAK;
    default:
        *Es_pred = Es_pred_qua_5b_fx[enr_idx];
        move16();
        BREAK;
    }
}
/*======================================================================================*/
/* FUNCTION      : void gain_dec_tc_fx ()											    */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE       : Decoding of pitch and codebook gains and updating long term energies */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :																	*/
/* Word32  core_brate_fx     i  : core bitrate											*/
/* Word16 *code_fx           i  : algebraic code excitation								*/
/* Word16 L_frame_fx         i  : length of the frame									*/
/* Word16 i_subfr_fx         i  : subframe number										*/
/* Word16 tc_subfr_fx        i  :  TC subframe index									*/
/* Word16 Es_pred_fx         i  :  predicted scaled innov. energy				Q8		*/
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																	*/
/* Word16 *gain_pit_fx       o  : pitch gain									Q14			*/
/* Word32 *gain_code_fx      o  : Quantized codeebook gain						Q16		*/
/* Word16 *gain_inov_fx      o  : unscaled innovation gain						Q12		*/
/* Word32 *norm_gain_code_fx o  : norm. gain of the codebook excit.				Q16		*/
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																*/
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																	*/
/*					 _ None																*/
/*--------------------------------------------------------------------------------------*/
/* CALLED FROM : 																		*/
/*======================================================================================*/

void gain_dec_tc_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    const Word32  core_brate_fx,     /* i  : core bitrate                        */
    const Word16 *code_fx,          /* i  : algebraic code excitation           */
    const Word16 L_frame_fx,        /* i  : length of the frame                 */
    const Word16 i_subfr_fx,        /* i  : subframe number                     */
    const Word16 tc_subfr_fx,       /* i  :  TC subframe index                  */
    const Word16 Es_pred_fx,        /* i  :  predicted scaled innov. energy     */
    Word16 *gain_pit_fx,      /* o  : pitch gain                          */
    Word32 *gain_code_fx,     /* o  : Quantized codeebook gain            */
    Word16 *gain_inov_fx,     /* o  : unscaled innovation gain            */
    Word32 *norm_gain_code_fx /* o  : norm. gain of the codebook excit.   */
)
{
    Word16 index, nBits;
    Word16 gcode0_fx;
    Word16 Ei_fx;
    Word16 expg, expg2, e_tmp, f_tmp, exp_gcode0, tmp_fx, frac;
    Word32 L_tmp, L_tmp1;
    Word16 wgain_code=0;
    move16();
    *gain_pit_fx = 0;
    move16();

    /*----------------------------------------------------------------*
    * find number of bits for gain dequantization
    *----------------------------------------------------------------*/
    IF( sub(L_frame_fx,L_FRAME) == 0)
    {
        nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate_fx, TRANSITION, i_subfr_fx, TC_SUBFR2IDX_fx(tc_subfr_fx))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate_fx, TRANSITION, i_subfr_fx, TC_SUBFR2IDX_16KHZ_fx(tc_subfr_fx))];
        move16();
    }

    /*-----------------------------------------------------------------*
     * calculate the predicted gain code
     *-----------------------------------------------------------------*/

    /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;
    *gain_inov = 1.0f / (float)sqrt( Ecode );*/
    L_tmp = Dot_product12(code_fx, code_fx, L_SUBFR, &expg);
    expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    expg2 = expg;
    move16();
    L_tmp1 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
    L_tmp = Isqrt_lc(L_tmp, &expg);

    *gain_inov_fx = extract_h(L_shl(L_tmp, sub(expg, 3)));
    move16(); /* gain_inov in Q12 */


    /*     Ei = 10 * (float)log10( Ecode );*/
    e_tmp = norm_l(L_tmp1);
    f_tmp = Log2_norm_lc(L_shl(L_tmp1, e_tmp));
    e_tmp = sub(expg2,add(1,e_tmp));
    L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/
    Ei_fx = round_fx(L_shl(L_tmp1, 11)); /* Q8 */
    /*      gcode0 = (float) pow(10, 0.05 * (Es_pred - Ei));*/
    gcode0_fx = sub(Es_pred_fx, Ei_fx); /* Q8 */

    /*-----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     * = pow(2, 3.321928*gcode0/20)
     * = pow(2, 0.166096*gcode0)
     *-----------------------------------------------------------------*/
    L_tmp = L_mult(gcode0_fx, 21771); /* *0.166096 in Q17 -> Q26 */
    L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */
    gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767 */
    exp_gcode0 = sub(exp_gcode0, 14);
    /*------------------------------------------------------------------------------------------*
     * Select the gain quantization table and dequantize the gain
     *------------------------------------------------------------------------------------------*/

    /*      index = (Word16)get_indice( st_fx,"gain_code", i_subfr_fx, ACELP_CORE);move16();*/
    index = (Word16)get_next_indice_fx( st_fx, nBits );
    move16();


    IF( sub(nBits,3) > 0 )
    {
        wgain_code = gain_dequant_fx( index, G_CODE_MIN_TC_Q15, G_CODE_MAX_TC_Q0, nBits, &expg );
        wgain_code = shl(wgain_code,add(expg,13));     /* wgain_code in Q13*/
    }
    ELSE IF( sub(nBits,2) == 0 )
    {
        /* 2-bit -> 3-bit decoding */
        wgain_code = tbl_gain_code_tc_fx[index * 2];
        move16();
    }
    ELSE /* nBits == 3 */
    {
        wgain_code = tbl_gain_code_tc_fx[index];
        move16();
    }

    /*-----------------------------------------------------------------*
     * decode normalized codebook gain
     *-----------------------------------------------------------------*/

    /* *gain_code *= gcode0;*/
    L_tmp = L_mult(wgain_code, gcode0_fx);         /* Q13*Q0 -> Q14 */
    *gain_code_fx= L_shl(L_tmp, add(exp_gcode0, 2));
    move32(); /* Q14 -> Q16 */

    /**norm_gain_code = *gain_code / *gain_inov;*/
    expg = sub(norm_s(*gain_inov_fx),1);
    expg = s_max(expg, 0);

    tmp_fx = div_s(shr(8192,expg),*gain_inov_fx);
    *norm_gain_code_fx = L_shr(Mult_32_16(*gain_code_fx, tmp_fx),sub(1,expg));
    move32();

    return;
}
/*======================================================================================*/
/* FUNCTION : gain_dec_mless_fx()														*/
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Decoding of pitch and codebook gains without updating long term energies	*/
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :																	*/
/* _ (Word32) core_brate_fx : core bitrate												*/
/* _ (Word16) L_frame_fx : length of the frame											*/
/* _ (Word16) coder_type_fx : coding type												*/
/* _ (Word16) i_subfr_fx : subframe index												*/
/* _ (Word16) tc_subfr_fx : TC subframe index											*/
/* _ (Word16*[]) code_fx : algebraic code excitation (Q12)								*/
/* _ (Word16) Es_pred_fx : predicted scaled innov. energy  (Q8)							*/
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																	*/
/* _ (Word16*) gain_pit_fx : quantized pitch gain (Q14)									*/
/* _ (Word32*) gain_code_fx : quantized codebook gain (Q16)								*/
/* _ (Word16*) gain_inov_fx : gain of the innovation (used for normalization) (Q12)		*/
/* _ (Word32*) norm_gain_code_fx : norm. gain of the codebook excitation (Q16)			*/
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																*/
/* _ None																				*/
/*--------------------------------------------------------------------------------------*/

/* _ None																				*/
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																	*/
/* _ None																				*/
/*======================================================================================*/
void gain_dec_mless_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    const Word32  core_brate_fx,       /* i  : core bitrate								*/
    const Word16 L_frame_fx,          /* i  : length of the frame						*/
    const Word16 coder_type_fx,       /* i  : coding type								*/
    const Word16 i_subfr_fx,          /* i  : subframe number							*/
    const Word16 tc_subfr_fx,         /* i  : TC subframe index							*/
    const Word16 *code_fx,            /* i  : algebraic code excitation                 */
    const Word16 Es_pred_fx,          /* i  : predicted scaled innov. energy            */
    Word16 *gain_pit_fx,        /* o  : Quantized pitch gain                   Q14*/
    Word32 *gain_code_fx,       /* o  : Quantized codeebook gain               Q16*/
    Word16 *gain_inov_fx,       /* o  : unscaled innovation gain               Q12*/
    Word32 *norm_gain_code_fx   /* o  : norm. gain of the codebook excitation  Q16*/
)
{
    Word16 index, nBits;
    Word16 gcode0_fx, Ei_fx, gain_code16;
    const Word16 *qua_table_fx;
    Word16 expg, expg2, e_tmp, f_tmp, exp_gcode0, tmp_fx, frac;
    Word32 L_tmp, L_tmp1;

    /*-----------------------------------------------------------------*
     * decode pitch gain
     *-----------------------------------------------------------------*/
    IF( sub(L_frame_fx,L_FRAME) == 0)
    {
        nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate_fx, coder_type_fx, i_subfr_fx, TC_SUBFR2IDX_fx(tc_subfr_fx))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate_fx, coder_type_fx, i_subfr_fx, TC_SUBFR2IDX_16KHZ_fx(tc_subfr_fx))];
        move16();
    }
    test();
    test();
    test();
    test();
    test();
    IF( (sub(tc_subfr_fx,3*L_SUBFR ) == 0 && sub(i_subfr_fx,3*L_SUBFR) == 0 && sub(L_frame_fx,L_FRAME) == 0) ||
        (sub(tc_subfr_fx,4*L_SUBFR ) == 0 && sub(i_subfr_fx,4*L_SUBFR) == 0 && sub(L_frame_fx,L_FRAME16k) == 0) )
    {
        /* decode pitch gain */
        index = (Word16)get_next_indice_fx( st_fx, nBits>>1 );
        move16();

        /*Ei = (G_PITCH_MAX_TC192 - G_PITCH_MIN_TC192) / ((1 << (nBits>>1)) - 1);*/  /* set quantization step */
        tmp_fx = div_s(1,sub(shl(1,shr(nBits,1)),1)); /*Q15*/
        Ei_fx = mult_r(G_PITCH_MAX_MINUS_MIN_TC192_Q13,tmp_fx); /*Q13*/

        /**gain_pit = usdequant( index, G_PITCH_MIN_TC192, Ei );*/
        *gain_pit_fx = usdequant_fx( index, G_PITCH_MIN_TC192_Q14, Ei_fx );
        move16(); /*Q14*/

        /* calculate the predicted gain code */
        /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;
        *gain_inov = 1.0f / (float)sqrt( Ecode );*/
        L_tmp = Dot_product12(code_fx, code_fx, L_SUBFR, &expg);
        expg = sub(expg, 18 + 6); /* exp: -18 (code in Q12), -6 (/L_SUBFR) */
        expg2 = expg;
        move16();
        L_tmp1 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
        L_tmp = Isqrt_lc(L_tmp, &expg);

        *gain_inov_fx = extract_h(L_shl(L_tmp, sub(expg, 3)));  /* gain_inov in Q12 */

        /*Ei = 10 * (float)log10( Ecode );*/
        e_tmp = norm_l(L_tmp1);
        f_tmp = Log2_norm_lc(L_shl(L_tmp1, e_tmp));
        e_tmp = sub(expg2,add(1,e_tmp));
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/
        Ei_fx = round_fx(L_shl(L_tmp1, 11)); /* Q8 */

        /*-----------------------------------------------------------------*
         * calculate the predicted gain code
         *-----------------------------------------------------------------*/

        /*gcode0 = (float) pow(10, 0.05 * (Es_pred - Ei));*/
        gcode0_fx = sub(Es_pred_fx, Ei_fx); /* Q8 */

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, gcode0/20)
         * = pow(2, 3.321928*gcode0/20)
         * = pow(2, 0.166096*gcode0)
         *-----------------------------------------------------------------*/
        L_tmp = L_mult(gcode0_fx, 21771); /* *0.166096 in Q17 -> Q26 */
        L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /* decode normalized codebook gain */
        /*index = (short)get_indice( st_fx, "gain_code", i_subfr_fx, ACELP_CORE );move16();*/
        index = (Word16)get_next_indice_fx( st_fx, (nBits+1)>>1 );
        move16();

        /**gain_code = gain_dequant( index, G_CODE_MIN_TC192, G_CODE_MAX_TC192, (nBits+1)>>1 );*/
        gain_code16 = gain_dequant_fx( index, G_CODE_MIN_TC192_Q15, G_CODE_MAX_TC192_Q0, shr(add(nBits,1),1), &expg );
        move16();

        /**gain_code *= gcode0;*/
        L_tmp = L_mult(gain_code16,gcode0_fx); /*Q0*Q0 -> Q1*/
        *gain_code_fx = L_shl(L_tmp,add(add(expg,exp_gcode0),15));
        move32();  /*Q16*/
    }
    ELSE
    {
        SWITCH ( nBits )
        {
        case 7:
            {
                qua_table_fx = gain_qua_mless_7b_fx;
                move16();
                BREAK;
            }
        case 6:
            {
                qua_table_fx = gain_qua_mless_6b_fx;
                move16();
                BREAK;
            }
        case 5:
            {
                qua_table_fx = gain_qua_mless_5b_fx;
                move16();
                BREAK;
            }
        default:
            {
                qua_table_fx = gain_qua_mless_6b_fx;
                move16();
                BREAK;
            }
        }

        test();
        if( sub(coder_type_fx,INACTIVE) == 0&& sub(nBits,6) == 0  )
        {
            nBits = sub(nBits, 1);
        }

        index = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();

        *gain_pit_fx = qua_table_fx[index * 2];
        move16();

        /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;
        *gain_inov = 1.0f / (float)sqrt( Ecode );*/

        L_tmp = Dot_product12(code_fx, code_fx, L_SUBFR, &expg);
        expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
        expg2 = expg;
        move16();
        L_tmp1 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
        L_tmp = Isqrt_lc(L_tmp, &expg);

        *gain_inov_fx = extract_h(L_shl(L_tmp, sub(expg, 3)));  /* gain_inov in Q12 */

        /*Ei = 10 * (float)log10( Ecode );*/
        e_tmp = norm_l(L_tmp1);
        f_tmp = Log2_norm_lc(L_shl(L_tmp1, e_tmp));
        e_tmp = sub(expg2,add(1,e_tmp));
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/
        Ei_fx = round_fx(L_shl(L_tmp1, 11)); /* Q8 */

        /*-----------------------------------------------------------------*
         * calculate the predicted gain code
         *-----------------------------------------------------------------*/

        /*gcode0 = (float) pow(10, 0.05 * (Es_pred - Ei));*/
        gcode0_fx = sub(Es_pred_fx, Ei_fx); /* Q8 */

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, gcode0/20)
         * = pow(2, 3.321928*gcode0/20)
         * = pow(2, 0.166096*gcode0)
         *-----------------------------------------------------------------*/
        L_tmp = L_mult(gcode0_fx, 21771); /* *0.166096 in Q17 -> Q26 */
        L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /*-----------------------------------------------------------------*
         * decode normalized codebook gain
         *-----------------------------------------------------------------*/

        /**gain_code = qua_table[index * 2 + 1] * gcode0;*/
        L_tmp = L_mult(qua_table_fx[add(shl(index,1),1)] , gcode0_fx); /* Q9*Q0 -> Q10 */
        *gain_code_fx = L_shl(L_tmp, add(exp_gcode0, 6));
        move32();  /* Q10 -> Q16*/
    }

    /**norm_gain_code = *gain_code / *gain_inov;*/
    expg = sub(norm_s(*gain_inov_fx),1);
    expg = s_max(expg, 0);

    tmp_fx = div_s(shr(8192,expg),*gain_inov_fx);
    *norm_gain_code_fx = L_shr(Mult_32_16(*gain_code_fx, tmp_fx),sub(1,expg));
    move32();

    return;
}

/*==================================================================================*/
/* FUNCTION : gain_dec_lbr_fx()														*/
/*----------------------------------------------------------------------------------*/
/* PURPOSE : Decoding of pitch and codebook gains in ACELP at 6.6 and 7.5 kbps		*/
/*----------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :																*/
/* _ (Word32) core_brate : core bitrate												*/
/* _ (Word16) coder_type : coding type												*/
/* _ (Word16) i_subfr : subframe index												*/
/* _ (Word16*[]) code_fx : algebraic excitation (Q12)								*/
/*----------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :																*/
/* _ (Word16*) gain_pit_fx : quantized pitch gain (Q14)							    */
/* _ (Word32*) gain_code_fx : quantized codebook gain (Q16)						    */
/* _ (Word16*) gain_inov_fx : gain of the innovation (used for normalization) (Q12) */
/* _ (Word32*) norm_gain_code_fx : norm. gain of the codebook excitation (Q12)		*/
/*----------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :															*/
/* _ None																			*/
/*----------------------------------------------------------------------------------*/

/* _ None																			*/
/*----------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :																*/
/* _ None																			*/
/*==================================================================================*/
void gain_dec_lbr_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure                                         */
    const Word32  core_brate,           /* i  : core bitrate                                                    */
    const Word16 coder_type,            /* i  : coding type                                                     */
    const Word16 i_subfr,               /* i  : subframe index                                                  */
    const Word16 *code_fx,              /* i  : algebraic excitation                                         Q9 */
    Word16 *gain_pit_fx,          /* o  : quantized pitch gain                                         Q14*/
    Word32 *gain_code_fx,         /* o  : quantized codebook gain                                      Q16*/
    Word16 *gain_inov_fx,         /* o  : gain of the innovation (used for normalization)              Q12*/
    Word32 *norm_gain_code_fx,    /* o  : norm. gain of the codebook excitation                        Q16*/
    Word32 gc_mem[],              /* i/o: gain_code from previous subframes                               */
    Word16 gp_mem[]               /* i/o: gain_pitch from previous subframes                              */
)
{
    Word16 index, nBits, n_pred, ctype;
    Word16  gcode0_fx, aux_fx[10];
    Word32 L_tmp, L_tmp1, L_tmp2;
    Word16 expg, expg2, e_tmp, exp_gcode0, f_tmp, frac, tmp_fx;
    const Word16 *b_fx ,*cdbk_fx = 0;
    /*      Ecode = ( dotp( code, code, L_SUBFR ) + 0.01f ) / L_SUBFR;
            *gain_inov = 1.0f / (float)sqrt(Ecode);	*/

    L_tmp = Dot_product12(code_fx, code_fx, L_SUBFR, &expg);
    expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    expg2 = expg;
    move16();
    L_tmp2 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
    L_tmp = Isqrt_lc(L_tmp, &expg);

    *gain_inov_fx = extract_h(L_shl(L_tmp, sub(expg, 3)));  /* gain_inov in Q12 */


    /*-----------------------------------------------------------------*
     * select the codebook, size and number of bits
     * set the gains searching range
     *-----------------------------------------------------------------*/

    nBits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, 0)];
    move16();

    ctype = shl(sub(coder_type, 1),1);

    /*-----------------------------------------------------------------*
     * calculate prediction of gcode
     * search for the best codeword
     *-----------------------------------------------------------------*/
    IF (i_subfr == 0)
    {
        b_fx = b_1sfr_fx;
        move16();
        n_pred = 2;
        move16();
        cdbk_fx = gp_gamma_1sfr_6b_fx;
        SWITCH ( nBits )
        {
        case 8:
            {
                cdbk_fx = gp_gamma_1sfr_8b_fx;    /* Q14/Q9*/
                move16();
                BREAK;
            }
        case 7:
            {
                cdbk_fx = gp_gamma_1sfr_7b_fx;    /* Q14/Q9*/
                move16();
                BREAK;
            }
        case 6:
            {
                cdbk_fx = gp_gamma_1sfr_6b_fx;    /* Q14/Q9*/
                move16();
                BREAK;
            }
        }

        /* calculate predicted gain */
        aux_fx[0] = 4096;
        move16();
        aux_fx[1] = shl(ctype,12);

        /*     gcode0 = (float)pow(10, dotp(b, aux, n_pred) - 0.5f * (float)log10(Ecode));
               gcode0 = (float)pow(10, dotp(b, aux, n_pred) - 0.05f * 10 * (float)log10(Ecode));
               gcode0 = (float)pow(10, 0.05(20 * dotp(b, aux, n_pred) - 10 * (float)log10(Ecode))); */

        e_tmp = norm_l(L_tmp2);
        f_tmp = Log2_norm_lc(L_shl(L_tmp2, e_tmp));
        e_tmp = sub(expg2,add(1,e_tmp));
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/

        L_tmp = Dot_product(b_fx, aux_fx, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp,160);/*Q13, 20 in Q3*/
        L_tmp = L_sub(L_tmp,L_tmp1);/*Q13*/

        gcode0_fx = round_fx(L_shl(L_tmp, 11)); /* Q8 */


        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, gcode0/20)
         * = pow(2, 3.321928*gcode0/20)
         * = pow(2, 0.166096*gcode0)
         *-----------------------------------------------------------------*/

        L_tmp = L_mult(gcode0_fx, 21771); /* *0.166096 in Q17 -> Q26 */
        L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /* retrieve the codebook index and calculate both gains */
        /*index = (Word16)get_indice( st_fx,"gain", i_subfr, ACELP_CORE);move16();*/
        index = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();

        *gain_pit_fx = cdbk_fx[index * 2];
        move16();

        L_tmp = L_mult(cdbk_fx[add(shl(index,1),1)] , gcode0_fx);  /* Q9*Q0 -> Q10 */
        *gain_code_fx = L_shl(L_tmp, add(exp_gcode0, 6));
        move16();  /* Q10 -> Q16*/

        gc_mem[0] = *gain_code_fx;
        move32();      /*Q16*/
        gp_mem[0] = *gain_pit_fx;
        move16();     /*Q14*/
    }
    ELSE IF (sub(i_subfr,L_SUBFR) == 0)
    {
        b_fx = b_2sfr_fx;
        move16();
        n_pred = 4;
        move16();

        cdbk_fx = gp_gamma_1sfr_6b_fx;
        SWITCH ( nBits )
        {
        case 7:
            {
                cdbk_fx = gp_gamma_2sfr_7b_fx;    /* Q14/Q9*/
                move16();
                BREAK;
            }
        case 6:
            {
                cdbk_fx = gp_gamma_2sfr_6b_fx;    /* Q14/Q9*/
                move16();
                BREAK;
            }
        }

        /* calculate predicted gain */
        aux_fx[0] = 4096;
        move16();
        aux_fx[1] = shl(ctype,12);

        /*aux_fx[2] = (float)log10(gc_mem[0]);
                    = log2(gc_mem[0])*log10(2);*/
        e_tmp = norm_l(gc_mem[0]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[0], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[0])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux_fx[3] = shr(gp_mem[0],2); /*Q12*/

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b_fx, aux_fx, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /* retrieve the codebook index and calculate both gains */
        index = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();

        *gain_pit_fx = cdbk_fx[index * 2];
        move16();

        L_tmp = L_mult(cdbk_fx[add(shl(index,1),1)] , gcode0_fx);  /* Q9*Q0 -> Q10 */
        *gain_code_fx = L_shl(L_tmp, add(exp_gcode0, 6));
        move16();  /* Q10 -> Q16*/

        gc_mem[1] = *gain_code_fx;
        move32();
        gp_mem[1] = *gain_pit_fx;
        move16();
    }
    ELSE IF (sub(i_subfr,2*L_SUBFR) == 0)
    {
        b_fx = b_3sfr_fx;
        move16();
        n_pred = 6;
        move16();

        cdbk_fx = gp_gamma_3sfr_6b_fx;
        move16();  /* Q14/Q9*/

        /* calculate predicted gain */
        aux_fx[0] = 4096;
        move16();
        aux_fx[1] = shl(ctype,12);
        move16();

        /*aux_fx[2] = (float)log10(gc_mem[0]);
                    = log2(gc_mem[0])*log10(2);*/
        e_tmp = norm_l(gc_mem[0]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[0], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[0])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        /*aux[3] = (float)log10(gc_mem[1]);
                 =  log2(gc_mem[1])*log10(2);*/
        e_tmp = norm_l(gc_mem[1]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[1], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[1])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[3] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux_fx[4] = shr(gp_mem[0],2);
        move16();
        aux_fx[5] = shr(gp_mem[1],2);
        move16();

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b_fx, aux_fx, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /* retrieve the codebook index and calculate both gains */
        index = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();

        *gain_pit_fx = cdbk_fx[index * 2];
        move16();

        L_tmp = L_mult(cdbk_fx[add(shl(index,1),1)] , gcode0_fx);  /* Q9*Q0 -> Q10 */
        *gain_code_fx = L_shl(L_tmp, add(exp_gcode0, 6));  /* Q10 -> Q16*/

        gc_mem[2] = *gain_code_fx;
        move32();
        gp_mem[2] = *gain_pit_fx;
        move16();
    }
    ELSE IF (sub(i_subfr,3*L_SUBFR) == 0)
    {
        b_fx = b_4sfr_fx;
        move16();
        n_pred = 8;
        move16();

        cdbk_fx = gp_gamma_4sfr_6b_fx;
        move16(); /* Q14/Q9*/

        /* calculate predicted gain */
        aux_fx[0] = 4096;
        move16();
        aux_fx[1] = shl(ctype,12);
        move16();

        /*aux[2] = (float)log10(gc_mem[0]);
                    = log2(gc_mem[0])*log10(2);*/
        e_tmp = norm_l(gc_mem[0]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[0], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[0])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[2] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        /*aux[3] = (float)log10(gc_mem[1]);
                 =  log2(gc_mem[1])*log10(2);*/
        e_tmp = norm_l(gc_mem[1]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[1], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[1])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[3] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        /*aux[4] = (float)log10(gc_mem[2]);
                 =  log2(gc_mem[2])*log10(2);*/
        e_tmp = norm_l(gc_mem[2]);
        f_tmp = Log2_norm_lc(L_shl(gc_mem[2], e_tmp));
        e_tmp = sub(sub(30,e_tmp),16); /*Q_format(gc_mem[2])=16*/
        L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */
        aux_fx[4] = round_fx(L_shl(L_tmp1, 12)); /* Q12 */

        aux_fx[5] = shr(gp_mem[0],2);/*Q12*/  move16();
        aux_fx[6] = shr(gp_mem[1],2);/*Q12*/  move16();
        aux_fx[7] = shr(gp_mem[2],2);/*Q12*/  move16();

        /*-----------------------------------------------------------------*
         * gcode0 = pow(10.0, dotp(b, aux, n_pred)
         * = pow(2, 3.321928*dotp(b, aux, n_pred)
         *-----------------------------------------------------------------*/
        L_tmp = Dot_product(b_fx, aux_fx, n_pred); /*Q25*/
        L_tmp = Mult_32_16(L_tmp, 27213); /* *3.321928 in Q13 -> Q23 */
        L_tmp = L_shr(L_tmp, 7); /* From Q23 to Q16 */
        frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

        gcode0_fx = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp_gcode0 = sub(exp_gcode0, 14);

        /* retrieve the codebook index and calculate both gains */
        index = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();
        *gain_pit_fx = cdbk_fx[index * 2];
        move16();

        L_tmp = L_mult(cdbk_fx[add(shl(index,1),1)] , gcode0_fx);  /* Q9*Q0 -> Q10 */
        *gain_code_fx = L_shl(L_tmp, add(exp_gcode0, 6));
        move32();  /* Q10 -> Q16*/
    }

    /* *norm_gain_code = *gain_code / *gain_inov; */
    expg = sub(norm_s(*gain_inov_fx),1);
    expg = s_max(expg, 0);

    tmp_fx = div_s(shr(8192,expg),*gain_inov_fx);
    *norm_gain_code_fx = L_shr(Mult_32_16(*gain_code_fx, tmp_fx),sub(1,expg));
    move32();

    return;
}

/*====================================================================== */
/* FUNCTION : lp_gain_updt_fx()                                          */
/*-----------------------------------------------------------------------*/
/* PURPOSE :  Update of LP pitch and code gains (FEC)                    */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :													 */
/* _ (Word16) i_subfr : subframe number               Q0                 */
/* _ (Word16) gain_pit : Decoded gain pitch           Q14                */
/* _ (Word32) norm_gain_code : Normalised gain code   Q16                */
/* _ (Word16) L_frame : length of the frame           Q0                 */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/* _ (Word16 *) T0 : close loop integer pitch                            */
/* _ (Word16 *) T0_frac : close loop fractional part of the pitch        */
/* _ (Word16 ) pitch : pitch value                     Q6                */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/* _ (Word16 *) lp_gainp : LP-filtered pitch gain(FEC) Q14               */
/* _ (Word16 *) lp_gainc : LP-filtered code gain (FEC) Q3                */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ None                                                                */
/*=======================================================================*/


void lp_gain_updt_fx(
    const Word16 i_subfr,                    /* i  :  subframe number      Q0    */
    const Word16 gain_pit,                   /* i  : Decoded gain pitch    Q14   */
    const Word32 norm_gain_code,             /* i  : Normalised gain code   Q16  */
    Word16 *lp_gainp,                  /* i/o: LP-filtered pitch gain(FEC) Q14 */
    Word16 *lp_gainc,                  /* i/o: LP-filtered code gain (FEC) Q3 */
    const Word16 L_frame                     /* i  : length of the frame         */
)
{
    Word16 tmp;

    tmp = extract_h(L_shl(norm_gain_code,3));   /*(16+3)-16 -> Q3*/
    IF( sub(L_frame,L_FRAME) == 0)
    {
        IF(i_subfr == 0)
        {
            *lp_gainp = mult(3277,gain_pit);
            move16();   /*0.1 in Q15 = 3277 , (15+14)-15 -> Q14*/
            *lp_gainc = mult_r(3277,tmp);
            move16();    /* (15+3)-15 -> Q3*/
        }
        ELSE IF( sub(i_subfr,L_SUBFR) == 0)
        {
            *lp_gainp = add(*lp_gainp, mult(6554, gain_pit));
            move16();   /*Q14  (0.2 in Q15 = 6554)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 6554, tmp);
            move16();  /*Q3*/
        }
        ELSE IF( sub(i_subfr,2*L_SUBFR) == 0)
        {
            *lp_gainp = add( *lp_gainp, mult(9830, gain_pit));
            move16();  /*Q14 (0.3 in Q15 = 9830)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 9830, tmp);
            move16();  /*Q3*/
        }
        ELSE  /* i_subfr == 3*L_SUBFR */
        {
            *lp_gainp = add( *lp_gainp, mult(13107, gain_pit));
            move16();   /*Q14  (0.4 in Q15 = 13107)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 13107, tmp);
            move16();  /*Q3*/
        }
    }
    ELSE
    {
        IF( i_subfr == 0 )
        {
            *lp_gainp = mult(2185,gain_pit);
            move16();   /*(1.0/15.0) in Q15 = 2185 , (15+14)-15 -> Q14*/
            *lp_gainc = mult_r(2185,tmp);
            move16();    /* (15+3)-15 -> Q3*/
        }
        ELSE IF( sub(i_subfr,L_SUBFR ) == 0)
        {
            *lp_gainp = add(*lp_gainp, mult(4369, gain_pit));
            move16();  /*Q14  (2.0/15.0 in Q15 = 4369)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 4369, tmp);
            move16();  /*Q3*/
        }
        ELSE IF( sub(i_subfr,2*L_SUBFR) == 0)
        {
            *lp_gainp = add(*lp_gainp, mult(6554, gain_pit));
            move16();  /*Q14  (3.0/15.0 in Q15 = 6554)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 6554, tmp);
            move16();  /*Q3*/
        }
        ELSE IF( sub(i_subfr,3*L_SUBFR) == 0)
        {
            *lp_gainp = add(*lp_gainp, mult(8738, gain_pit));
            move16();  /*Q14  (4.0/15.0 in Q15 = 8738)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 8738, tmp);
            move16();  /*Q3*/
        }
        ELSE  /* i_subfr == 4*L_SUBFR */
        {
            *lp_gainp = add(*lp_gainp, mult(10923, gain_pit));
            move16();   /*Q14  (5.0/15.0 in Q15 = 10923)*/
            *lp_gainc = mac_r(L_deposit_h(*lp_gainc), 10923, tmp);
            move16();  /*Q3*/
        }
    }
    return;

}

/*-------------------------------------------------*
 *  Gain_dec_gaus_vbr
 *
 * Decode gains of purely unvoiced sounds
 *-------------------------------------------------*/
Word32 gain_dec_gaus_fx(            /* o  : quantized codebook gain                Q16  */
    Word16 index,             /* i  : quantization index                          */
    const Word16 bits,              /* i  : number of bits to quantize                  */
    const Word16 lowBound,          /* i  : lower bound of quantizer (dB)               */
    const Word16 topBound,          /* i  : upper bound of quantizer (dB)               */
    const Word16 inv_gain_inov,     /* o  : unscaled innovation gain                Q12 */
    Word32 *L_norm_gain_code  /* o  : gain of normalized gaussian excitation  Q16 */
)
{
    Word16 stepSize, gain, expg, frac, expi, tmp_igi;
    Word32 L_tmp, L_enr_q, L_gain;

    /*------------------------------------------------------------------------------------------*
     * Quantize linearly the log E
     *------------------------------------------------------------------------------------------*/

    stepSize = shl(sub(topBound,lowBound),sub(14, bits)); /* Q14 */

    /*------------------------------------------------------------------------------------------*
     * Gaussian codebook gain
     *------------------------------------------------------------------------------------------*/

    /* enr_q = (float)index*stepSize ,lowBound); */
    L_enr_q = L_mult(index, stepSize);      /* Q0 * Q14 -> Q15 */
    L_enr_q = L_shl(L_enr_q, 9);            /* Q15 -> Q24 */
    L_enr_q = L_add(L_enr_q, L_shl(L_deposit_h(lowBound),8)); /* Q24 */

    /*------------------------------------------------------------*
     * gain = pow(10.0, enr/20)
     * = pow(2, 3.321928*enr/20)
     * = pow(2, 0.166096*enr)
     *------------------------------------------------------------*/

    /* gain = (float)pow( 10.0f, enr/20.0f ); quantized codebook gain */
    L_tmp = Mult_32_16(L_enr_q, 21771);      /* *0.166096 in Q17 -> Q26 */
    L_tmp = L_shr(L_tmp, 10);               /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &expg);      /* Extract exponent of enr */
    L_gain = Pow2(30, frac);                /* Put 30 as exponent so that the */
    expg = add(expg, 16-30);                /* output of Pow2() will be */
    /* Normalized, set result in Q16 */
    gain = round_fx(L_gain);
    L_gain = L_shl(L_gain, expg); /* In Q16*/
    /* *norm_gain_code = gain / *inv_gain_inov;*/
    expi = norm_s(inv_gain_inov);
    tmp_igi = shl(inv_gain_inov, expi);
    L_tmp = div_s(shr(gain,1), tmp_igi);
    L_tmp = L_shl(L_tmp, add(1,expi));
    *L_norm_gain_code = L_shl(L_tmp, add(expg,13)); /* Q16 */ move32();

    return L_gain;
}

/*--------------------------------------------------------------------------*
* gain_dec_SQ()
*
* Decoding of pitch and codebook gains using scalar quantizers
*-------------------------------------------------------------------------*/

void gain_dec_SQ_fx(
    Decoder_State_fx *st_fx,         /* i/o: decoder state structure */
    const Word32 core_brate,       /* i  : core bitrate                             */
    const Word16 coder_type,       /* i  : coding type                              */
    const Word16 i_subfr,          /* i  : subframe number                          */
    const Word16 tc_subfr,         /* i  : TC subframe index                        */
    const Word16 *code,            /* i  : algebraic code excitation             Q9*/
    const Word16 Es_pred,          /* i  : predicted scaled innov. energy        Q8 */
    Word16 *gain_pit,        /* o  : Quantized pitch gain                  Q14*/
    Word32 *gain_code,       /* o  : Quantized codeebook gain              Q16*/
    Word16 *gain_inov,       /* o  : unscaled innovation gain              Q12*/
    Word32 *norm_gain_code   /* o  : norm. gain of the codebook excitation Q16*/
)
{
    Word16 index, nBits;
    Word16 gcode0, Ei;
    Word16 tmp16, expg, expg2, e_tmp, f_tmp, exp_gcode0, frac;
    Word32 L_tmp, L_tmp1;

    /*-----------------------------------------------------------------*
     * get number of bits
     *-----------------------------------------------------------------*/

    nBits = gain_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr)) ];
    move16();

    /*-----------------------------------------------------------------*
     * decode pitch gain
     *-----------------------------------------------------------------*/

    index = (Word16)get_next_indice_fx(st_fx,  shr(nBits,1) );

    /*Ei = (G_PITCH_MAX - G_PITCH_MIN) / ((1 << (nBits>>1)) - 1);   set quantization step */
    tmp16 = div_s(1,sub(shl(1,shr(nBits,1)),1));                /* Q15*/
    Ei = mult_r(G_PITCH_MAX_MINUS_MIN_Q13,tmp16);     /* Q13*/

    /**gain_pit = usdequant( index, G_PITCH_MIN, Ei );*/
    *gain_pit = usdequant_fx( index, G_PITCH_MIN_Q14, Ei );
    move16();   /*Q14  */

    /*-----------------------------------------------------------------*
     * calculate the predicted gain code
     *-----------------------------------------------------------------*/

    /*Ecode = (dotp( code, code, L_SUBFR) + 0.01f) / L_SUBFR;*/
    /**gain_inov = 1.0f / (float)sqrt( Ecode );*/

    L_tmp = Dot_product12(code, code, L_SUBFR, &expg);
    expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    expg2 = expg;
    move16();
    L_tmp1 = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
    L_tmp = Isqrt_lc(L_tmp, &expg);

    *gain_inov = extract_h(L_shl(L_tmp, sub(expg, 3)));  /* gain_inov in Q12 */

    /*Ei = 10 * (float)log10( Ecode );*/
    e_tmp = norm_l(L_tmp1);
    f_tmp = Log2_norm_lc(L_shl(L_tmp1, e_tmp));
    e_tmp = sub(expg2,add(1,e_tmp));
    L_tmp1 = Mpy_32_16(e_tmp, f_tmp, 12330); /* Q13 */ /* 10*log10(2) in Q12*/
    Ei = round_fx(L_shl(L_tmp1, 11)); /* Q8 */

    /*gcode0 = (float) pow(10, 0.05 * (Es_pred - Ei));*/
    gcode0 = sub(Es_pred, Ei); /* Q8 */

    /* gcode0 = pow(10.0, gcode0/20) = pow(2, 3.321928*gcode0/20) = pow(2, 0.166096*gcode0) */
    L_tmp = L_mult(gcode0, 21771); /* *0.166096 in Q17 -> Q26 */
    L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &exp_gcode0); /* Extract exponent of gcode0 */

    gcode0 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that output of Pow2() will be: 16384 < Pow2() <= 32767 */
    exp_gcode0 = sub(exp_gcode0, 14);

    /*-----------------------------------------------------------------*
     * decode normalized codebook gain
     *-----------------------------------------------------------------*/

    index = (Word16)get_next_indice_fx(st_fx, shr(add(nBits,1),1) );

    tmp16 = gain_dequant_fx( index, G_CODE_MIN_TC_Q15, G_CODE_MAX_TC_Q0, shr(add(nBits,1),1), &expg );

    /**gain_code *= gcode0;*/
    L_tmp = L_mult(tmp16,gcode0);           /* Q0*Q0 -> Q1*/
    /**gain_code = L_shl(L_tmp,add(expg,15));       Q16*/
    *gain_code = L_shl(L_tmp,add(add(expg,exp_gcode0),15));
    move32();  /*Q16*/

    /**norm_gain_code = *gain_code / *gain_inov;*/
    expg = sub(norm_s(*gain_inov),1);
    expg = s_max(expg, 0);

    tmp16 = div_s(shr(8192,expg),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code,tmp16),sub(1,expg));
    move32();

    return;
}

/*---------------------------------------------------------------------*
 * gain_dec_amr_wb()
 *
 * Decoding of pitch and fixed codebook gains (used also in AMR-WB IO mode)
 *---------------------------------------------------------------------*/

void gain_dec_amr_wb_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    const Word32 core_brate,       /* i  : core bitrate                          */
    Word16 *gain_pit,        /* o  : Quantized pitch gain                  */
    Word32 *gain_code,       /* o  : Quantized codeebook gain              */
    Word16 *past_qua_en,     /* i/o: gain quantization memory (4 words)    */
    Word16 *gain_inov,       /* o  : unscaled innovation gain              */
    const Word16 *code,            /* i  : algebraic code excitation             */
    Word32 *norm_gain_code   /* o  : norm. gain of the codebook excitation */
)
{
    Word16 i, index, index2;
    Word16 nbits;
    Word16 gcode0, qua_en;
    const Word16 *t_qua_gain;
    Word16 tmp;
    Word32 L_tmp;
    Word16 expg, exp_gcode0, fracg;

    /**gain_inov = 1.0f/ (float)sqrt( ( dotp( code, code, L_SUBFR ) + 0.01f ) / L_SUBFR );*/

    L_tmp = Dot_product12(code, code, L_SUBFR, &expg);
    expg = sub(expg, 18 + 6); /* exp: -18 (code in Q9), -6 (/L_SUBFR) */
    L_tmp = Isqrt_lc(L_tmp, &expg);

    *gain_inov = extract_h(L_shl(L_tmp, sub(expg, 3))); /* gain_inov in Q12 */

    /*-----------------------------------------------------------------*
     * Select the gain quantization table
     *-----------------------------------------------------------------*/
    nbits = 7;
    move16();
    t_qua_gain = t_qua_gain7b_fx;

    IF( L_sub(core_brate,ACELP_12k65) < 0)
    {
        nbits = 6;
        move16();
        t_qua_gain = t_qua_gain6b_fx;
    }

    /*-----------------------------------------------------------------*
     * predicted code gain
     *-----------------------------------------------------------------*/

    /* start with predicting code energy in dB */
    /**for (i=0; i<GAIN_PRED_ORDER; i++) {gcode0 += pred_gain[i] * past_qua_en[i];}*/
    /*gcode0 += (float)(20.0 * log10( *gain_inov ) );*/
    /* predicted energy */
    L_tmp = L_deposit_h(MEAN_ENER); /* MEAN_ENER in Q16 */
    L_tmp = L_shl(L_tmp, 9); /* From Q16 to Q25 */

    FOR (i=0; i<GAIN_PRED_ORDER; i++)
    {
        /* pred_code += pred[i]*past_qua_en[i]; */
        L_tmp = L_mac(L_tmp, pred_gain_fx[i], past_qua_en[i]); /* Q14*Q10 -> Q25 */
    }

    /* predicted codebook gain */
    gcode0 = extract_h(L_tmp); /* From Q25 to Q9 */

    /*-----------------------------------------------------------------*
     * gcode0 = pow(10.0, gcode0/20)
     * = pow(2, 3.321928*gcode0/20)
     * = pow(2, 0.166096*gcode0)
     *-----------------------------------------------------------------*/
    L_tmp = L_mult(gcode0, 21771); /* *0.166096 in Q17 -> Q27 */
    L_tmp = L_shr(L_tmp, 9+2); /* From Q27 to Q16 */
    L_Extract(L_tmp, &exp_gcode0, &fracg); /* Extract exponent of gcode0 */

    gcode0 = extract_l(Pow2(14, fracg));/* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767 */
    exp_gcode0 = sub(exp_gcode0, 14);

    /*-----------------------------------------------------------------*
     * Decode pitch gain
     *-----------------------------------------------------------------*/

    index = (Word16)get_next_indice_fx( st_fx, nbits );
    index2 = shl(index,1);
    *gain_pit = t_qua_gain[index2];
    move16();

    /*-----------------------------------------------------------------*
     * Decode code gain
     *-----------------------------------------------------------------*/
    qua_en = t_qua_gain[add(index2,1)];
    move16();

    /* *gain_code = t_qua_gain[indice*2+1] * gcode0; */
    L_tmp = L_mult(qua_en, gcode0); /* Q11*Q0 -> Q12 */
    tmp = round_fx(L_tmp);
    move16(); /* Q-4 */
    *gain_code = L_shl(L_tmp, add(exp_gcode0, 4));
    move32(); /* Q12 -> Q16 */

    /* adjust gain according to energy of code */
    L_tmp = Mult_32_16(*gain_code, *gain_inov);
    *gain_code = L_shl(L_tmp, 3);
    move32();  /* gcode_inov in Q12*/

    /*-----------------------------------------------------------------*
     * update table of past quantized energies
     *-----------------------------------------------------------------*/

    FOR (i=GAIN_PRED_ORDER-1; i>0; i--)
    {
        past_qua_en[i] = past_qua_en[i-1];
        move16();
    }
    /*past_qua_en[0] = (float)(20.0*log10(qua_en));*/
    /*----------------------------------------------------------*
     * past_qua_en[0] = 20*log10(t_qua_gain[indice*2+1])
     * = 6.0206*log2(t_qua_gain[indice*2+1])
     * = 6.0206*(log2(t_qua_gain[indice*2+1]Q11 -11)
     *----------------------------------------------------------*/
    tmp = norm_l(qua_en);
    fracg = Log2_norm_lc(L_shl(qua_en, tmp));
    expg = sub(30,tmp);
    expg = sub(expg, 11);
    L_tmp = Mpy_32_16(expg, fracg, 24660);   /* x 6.0206 in Q12 */
    qua_en = extract_h(L_shl(L_tmp, 13)); /* result in Q10 */

    past_qua_en[0] = qua_en;
    move16(); /* in Q10 */

    /*-----------------------------------------------------------------*
     * Normalized code gain
     *-----------------------------------------------------------------*/
    /**norm_gain_code = *gain_code / *gain_inov;*/
    expg = sub(norm_s(*gain_inov),1);
    expg = s_max(expg, 0);

    tmp = div_s(shr(8192,expg),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code,tmp),sub(1,expg));
    move32();

    return;
}
