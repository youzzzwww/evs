/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*-----------------------------------------------------------------*
 * Transform domain contribution decoding
 *-----------------------------------------------------------------*/
void transf_cdbk_dec_fx(
    Decoder_State_fx *st_fx,       /* i/o: decoder state structure */
    const Word32 core_brate,     /* i  : core bitrate                                    */
    const Word16 coder_type,     /* i  : coding type                                     */
    const Word16 harm_flag_acelp,/* i  : harmonic flag for higher rates ACELP            */
    const Word16 i_subfr,        /* i  : subframe index                                  */
    const Word16 tc_subfr,       /* i  : TC subframe index                               */
    const Word16 Es_pred,        /* i  : predicited scaled innovation energy (Q8)        */
    const Word32 gain_code,      /* i  : innovative excitation gain (Q16)                */
    Word16 *mem_preemp,    /* i/o: dequantizer preemhasis memory                   */
    Word16 *gain_preQ,     /* o  : prequantizer excitation gain (Q2)               */
    Word32 *norm_gain_preQ,/* o  : normalized prequantizer excitation gain (Q16)   */
    Word16 code_preQ[],    /* o  : prequantizer excitation (Q8)                    */
    Word16 *unbits         /* o  : number of AVQ unused bits                       */
)
{
    Word16 i, index, nBits;
    Word16 nq[L_SUBFR/WIDTH_BAND];
    Word16 gain16, exp16, tmp16;
    Word32 L_tmp;
    Word32 dct_code32[L_SUBFR];
    Word16 qdct;

    /*--------------------------------------------------------------*
     * Set bit-allocation
     *--------------------------------------------------------------*/

    nBits = AVQ_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx( core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
    move16();

    /* increase # of AVQ allocated bits by unused bits from the previous subframe */
    nBits = add(nBits,*unbits);

    /*--------------------------------------------------------------*
     * Dequantize prequantizer excitation gain
     *--------------------------------------------------------------*/

    index = (Word16)get_next_indice_fx(st_fx,  G_AVQ_BITS );

    IF( sub(coder_type,INACTIVE) == 0 )
    {
        IF( L_sub(core_brate,ACELP_64k) == 0 )
        {
            gain16 = usdequant_fx( index, G_AVQ_MIN_INACT_64k_Q12, G_AVQ_DELTA_INACT_64k_Q12>>1 );
        }
        ELSE IF( L_sub(core_brate,ACELP_48k) == 0 )
        {
            gain16 = usdequant_fx( index, G_AVQ_MIN_INACT_48k_Q12, G_AVQ_DELTA_INACT_48k_Q12>>1 );
        }
        ELSE
        {
            gain16 = usdequant_fx( index, G_AVQ_MIN_INACT_Q12, G_AVQ_DELTA_INACT_Q12>>1 );
        }

        L_tmp = Mult_32_16(gain_code,gain16);   /* Q16 * Q12 - 15 -> Q13*/
        L_tmp = L_shl(L_tmp,5);                 /* Q13 -> Q18*/
        *gain_preQ = round_fx(L_tmp);           /* Q2*/
    }
    ELSE
    {
        IF( L_sub(core_brate,ACELP_32k) <= 0 )
        {
            gain16 = gain_dequant_fx( index, G_AVQ_MIN_32kbps_Q15, G_AVQ_MAX_Q0, G_AVQ_BITS, &exp16 );
        }
        ELSE
        {
            gain16 = gain_dequant_fx( index, G_AVQ_MIN_Q15, G_AVQ_MAX_Q0, G_AVQ_BITS, &exp16 );
        }

        IF( Es_pred < 0 )
        {
            tmp16 = shr(negate(Es_pred),2);
            L_tmp = L_mult(gain16,tmp16);       /* Q0*Q8 -> Q9*/
        }
        ELSE
        {
            L_tmp = L_mult(gain16,Es_pred);     /* Q0*Q8 -> Q9*/
        }
        L_tmp = L_shl(L_tmp,add(exp16,9));      /* Q18*/
        *gain_preQ = round_fx(L_tmp);           /* Q2*/
    }

    /*--------------------------------------------------------------*
     * Demultiplex and decode subvectors from bit-stream
     *--------------------------------------------------------------*/

    AVQ_demuxdec_fx(st_fx, code_preQ, &nBits, 8, nq );

    FOR( i=0; i<L_SUBFR; i++ )
    {
        code_preQ[i] = shl(code_preQ[i],Q_AVQ_OUT_DEC);
        move16();    /* code_preQ in Q6*/
    }

    /* save # of AVQ unused bits for next subframe */
    *unbits = nBits;
    move16();

    /*--------------------------------------------------------------*
     * iDCT transform
     *--------------------------------------------------------------*/

    test();
    test();
    IF( sub(coder_type,INACTIVE) == 0 || L_sub(core_brate,ACELP_32k) > 0 || harm_flag_acelp )
    {
        qdct = 0;
        move16();
        edct2_fx( L_SUBFR, 1, code_preQ, dct_code32, &qdct, ip_edct2_64_fx, w_edct2_64_fx );
        /*qdct = sub(Q_AVQ_OUT_DEC,qdct+Q_AVQ_OUT_DEC);*/
        qdct = negate(qdct);
        Copy_Scale_sig_32_16(dct_code32, code_preQ,  L_SUBFR, qdct);    /* Output in Q_AVQ_OUT_DEC */
        /*qdct = Q_AVQ_OUT_DEC;*/
    }

    /*--------------------------------------------------------------*
     * Preemphasise
     *--------------------------------------------------------------*/
    /* in extreme cases at subframe boundaries, lower the preemphasis memory to avoid a saturation */
    test();
    if( (nq[7] != 0) && (sub( sub(st_fx->last_nq_preQ_fx, nq[0]), 7) > 0) )
    {
        /* *mem_preemp /= 16; */
        *mem_preemp = shr(*mem_preemp,4);
        move16();
    }
    st_fx->last_nq_preQ_fx = nq[7];
    move16();

    preemph_fx( code_preQ, FAC_PRE_AVQ_FX, L_SUBFR, mem_preemp );

    /*--------------------------------------------------------------*
     * Compute normalized prequantizer excitation gain for FEC
     *
     * somewhat attenuate pre-quantizer normalized gain for FEC
     *--------------------------------------------------------------*/

    /*Ecode = (sum2_f( code_preQ, L_SUBFR ) + 0.01f) / L_SUBFR;*/
    /*norm_gain_preQ = 0.8f * (*gain_preQ) * (float)sqrt( Ecode );*/

    L_tmp = Dot_product12(code_preQ, code_preQ, L_SUBFR, &exp16);

    IF( L_sub(L_tmp,L_shl(1,sub(30,exp16))) == 0 )
    {
        /* pre-quantizer contribution is zero */
        *norm_gain_preQ = 1;
        move16();
    }
    ELSE
    {
        exp16 = sub(exp16, Q_AVQ_OUT_DEC*2 + 6);             /* exp: (code_preQ in Q_AVQ_OUT_DEC), -6 (/L_SUBFR) */
        L_tmp = Isqrt_lc(L_tmp, &exp16);
        tmp16 = extract_h(L_tmp);
        exp16 = 15 - 10 - exp16;                /* tmp16 in Q10+exp16*/
        tmp16 = div_s(16384,tmp16);             /* Q15+Q14-(Q10+Qexp16) = Q19-exp16*/

        L_tmp = L_mult(*gain_preQ,tmp16);       /* Q2+Q19-exp16+1 -> Q22-exp16    */
        L_tmp = Mult_32_16(L_tmp, 26214 );      /* Q22-Qexp16+Q15+1-16 -> Q22-exp16*/
        *norm_gain_preQ = L_shr(L_tmp,6-exp16);
        move32();  /* Q22-exp16 -> Q16*/
    }


    st_fx->use_acelp_preq = 1;
    move16();

    return;

}

/*==========================================================================*/
/* FUNCTION      : Word16 gain_dequant_fx ()								*/
/*--------------------------------------------------------------------------*/
/* PURPOSE       :															*/
/*	* Returns decoded gain quantized between the specified					*/
/*  * range using the specified number of levels.	   						*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/*	Word16 index			i: quantization index							*/
/*	Word16 min		i : value of lower limit								*/
/*	Word16 max		i : value of upper limit								*/
/*	Word16 bits		i : number of bits to dequantize						*/
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* _Word16 *expg	o :														*/
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 Word16 gain									Q0		*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 														    */
/*==========================================================================*/
Word16 gain_dequant_fx( /* o: decoded gain */
    Word16 index, /* i: quantization index */
    const Word16 min, /* i: value of lower limit */
    const Word16 max, /* i: value of upper limit */
    const Word16 bits, /* i: number of bits to dequantize */
    Word16 *expg
)
{
    Word16 gain, c_min, c_max;
    Word16 levels;
    Word16 e_tmp, f_tmp;
    Word16 tmp, frac;
    Word32 L_tmp;
    levels = 1<<bits;

    /*c_min = (float)log10(min);
    c_mult = (float) ((levels-1)/(log10(max)-c_min));
    gain = (float)pow( 10.0, (((float)index)/c_mult) + c_min );*/

    /*
    y =(index/cmult) + c_min
      = index/((levels-1)/(log10(max)-log10(min))) + log10(min)
      = (index*(log10(max)-log10(min)))/(levels-1) + log10(min)
      = x*log10(max) + (1-x)*log10(min)
      where x = index/(levels-1)

    gain = pow(10.0,y)
         = pow(2,3.321928*y)
    */

    tmp = div_s(1,sub(levels,1)); /*Q15*/
    tmp = extract_l(L_shr(L_mult(index,tmp),1)); /*Q15*/

    e_tmp = norm_l(max);
    f_tmp = Log2_norm_lc(L_shl(max, e_tmp));
    e_tmp = sub(30,e_tmp);/*Q(max)=0*/
    L_tmp = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */ /*log10(2) in Q15*/
    c_max = round_fx(L_shl(L_tmp, 14)); /* Q14 */

    e_tmp = norm_l(min);
    f_tmp = Log2_norm_lc(L_shl(min, e_tmp));
    e_tmp = sub(sub(30,e_tmp),15);/*Q(min)=15*/
    L_tmp = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */ /*log10(2) in Q15*/
    c_min = round_fx(L_shl(L_tmp, 14)); /* Q14 */

    L_tmp = L_mult(tmp,c_max); /*Q30*/
    L_tmp = L_mac(L_tmp,sub(32767,tmp),c_min); /*Q30*/

    L_tmp = Mult_32_16(L_tmp,27213); /*Q28, 3.321928 in Q13*/
    L_tmp = L_shr(L_tmp,12); /*Q28->Q16*/

    frac = L_Extract_lc(L_tmp, expg); /* Extract exponent of gcode0 */

    gain = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767 */
    *expg = sub(*expg, 14);

    return( gain );
}
