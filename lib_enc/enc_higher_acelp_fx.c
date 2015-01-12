/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/
static void find_cn_fx( const Word16 xn[], const Word16 Ap[], const Word16 *p_Aq, Word16 cn[] );

/*-----------------------------------------------------------------*
 * Transform domain contribution encoding
 *-----------------------------------------------------------------*/
#define Q_MINUS 4
void transf_cdbk_enc_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure                         */
    const Word32 core_brate,     /* i  : core bitrate                                    */
    const Word16 extl,           /* i  : extension layer                                 */
    const Word16 coder_type,     /* i  : coding type                                     */
    const Word16 harm_flag_acelp,/* i  : harmonic flag for higher rates ACELP            */
    const Word16 i_subfr,        /* i  : subframe index                                  */
    const Word16 tc_subfr,       /* i  : TC subframe index                               */
    Word16 cn[],           /* i/o: target vector in residual domain                */
    Word16 exc[],          /* i/o: pointer to excitation signal frame              */
    const Word16 *p_Aq,          /* i  : 12k8 Lp coefficient                             */
    const Word16 Ap[],           /* i  : weighted LP filter coefficients                 */
    const Word16 h1[],           /* i  : weighted filter input response                  */
    Word16 xn[],           /* i/o: target vector                                   */
    Word16 xn2[],          /* i/o: target vector for innovation search             */
    Word16 y1[],           /* i/o: zero-memory filtered adaptive excitation        */
    const Word16 y2[],           /* i  : zero-memory filtered innovative excitation      */
    const Word16 Es_pred,        /* i  : predicited scaled innovation energy             */
    Word16 *gain_pit,      /* i/o: adaptive excitation gain                        */
    const Word32 gain_code,      /* i  : innovative excitation gain                      */
    Word16 g_corr[],       /* o  : ACELP correlation values                        */
    const Word16 clip_gain,      /* i  : adaptive gain clipping flag                     */
    Word16 *mem_deemp,     /* i/o: prequantizer deemhasis memory                   */
    Word16 *mem_preemp,    /* i/o: prequantizer preemhasis memory                  */
    Word16 *gain_preQ,     /* o  : prequantizer excitation gain                    */
    Word16 code_preQ[],    /* o  : prequantizer excitation                         */
    Word16 *unbits,        /* o  : number of AVQ unused bits                       */
    const Word16 Q_new,          /* i  : Current frame scaling                           */
    const Word16 shift           /* i  : shifting applied to y1, xn,...                  */
)
{
    Word16 i, index, nBits, Nsv, Es_pred_loc;
    Word16 x_in[L_SUBFR], x_tran[L_SUBFR], gcode16, stmp;
    Word16 e_corr, m_corr, e_ener, m_ener, m_den, e_den;
    Word16 x_norm[L_SUBFR+L_SUBFR/WIDTH_BAND];
    Word32 L_corr, L_ener, Ltmp, Ltmp1;
    Word16 nq[L_SUBFR/WIDTH_BAND];
    Word32 out32[L_SUBFR];
    Word16 Qdct;

    /*--------------------------------------------------------------*
     * Set bit-allocation
     *--------------------------------------------------------------*/

    Nsv = 8;
    move16();
    nBits = AVQ_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
    move16();

    /* increase # of AVQ allocated bits by unused bits from the previous subframe */
    nBits = add(nBits, *unbits);

    /*--------------------------------------------------------------*
     * Compute/Update target
     * For inactive frame, find target in residual domain
     * Deemphasis
     *--------------------------------------------------------------*/
    IF (sub(coder_type,INACTIVE) == 0)
    {
        gcode16 = round_fx(L_shl(gain_code, Q_new));
        FOR( i=0; i<L_SUBFR; i++ )
        {
            /*x_tran[i] = xn[i] - *gain_pit * y1[i] - gain_code * y2[i];*/
            Ltmp = L_mult(gcode16, y2[i]);
            Ltmp = L_shl(Ltmp, add(5, shift));
            Ltmp = L_negate(Ltmp);
            Ltmp = L_mac(Ltmp, xn[i], 16384);
            Ltmp = L_msu(Ltmp, y1[i], *gain_pit);
            Ltmp = L_shl(Ltmp, sub(1, shift));
            x_tran[i] = round_fx(Ltmp);     /*Q_new-1  */
        }
        find_cn_fx( x_tran, Ap, p_Aq, x_in );
    }
    ELSE
    {
        updt_tar_fx( cn, x_in, &exc[i_subfr], *gain_pit, L_SUBFR );
    }
    Deemph2( x_in, FAC_PRE_AVQ_FX, L_SUBFR, mem_deemp );

    /*--------------------------------------------------------------*
     * DCT-II
     *--------------------------------------------------------------*/

    test();
    test();
    test();
    IF( sub(coder_type,INACTIVE) != 0 && L_sub(core_brate,ACELP_32k) <= 0 && L_sub(core_brate,ACELP_24k40) > 0 && !harm_flag_acelp )
    {
        Copy_Scale_sig( x_in, x_tran, L_SUBFR,-Q_MINUS+1 ); /*Q_new-1 -> Q_new-4*/
        /*Copy( x_in, x_tran, L_SUBFR );*/
        Qdct = sub(Q_new,Q_MINUS);
    }
    ELSE
    {
        Qdct = 0;
        move16();
        edct2_fx( L_SUBFR, -1, x_in, out32, &Qdct, ip_edct2_64_fx, w_edct2_64_fx );
        Qdct = negate(Qdct);
        Copy_Scale_sig_32_16(out32, x_tran,  L_SUBFR, sub(Qdct,Q_MINUS-1));    /* Output in Q_new-4 */
        Qdct = sub(Q_new,Q_MINUS);
    }

    /*--------------------------------------------------------------*
     * Split algebraic vector quantizer based on RE8 lattice
     *--------------------------------------------------------------*/
    AVQ_cod_fx( x_tran, x_norm, nBits, Nsv, 0 );

    /*--------------------------------------------------------------*
     * Find prequantizer excitation gain
     * Quantize the gain
     *--------------------------------------------------------------*/
    L_corr = L_deposit_l(0);
    L_ener = L_deposit_l(0);

    FOR (i = 0; i < Nsv*8; i++)
    {
        /*fcorr += fx_tran[i]*(float)ix_norm[i];*/
        /*fener += (float)ix_norm[i]*(float)ix_norm[i];*/
        stmp = shl(x_norm[i],Q_AVQ_OUT );
        L_corr = L_mac(L_corr,  x_tran[i], stmp);
        L_ener = L_mac(L_ener,  stmp, stmp);
    }

    L_ener = L_max(L_ener,1);

    /* No negative gains allowed in the quantizer*/
    L_corr = L_max(L_corr,0);

    e_corr = norm_l(L_corr);
    m_corr = extract_h(L_shl(L_corr, e_corr));
    e_corr = sub(30, add(e_corr,sub(Qdct,Q_AVQ_OUT)));
    e_ener = norm_l(L_ener);
    m_ener = extract_h(L_shl(L_ener, e_ener));
    e_ener = sub(30, e_ener);

    IF(sub(m_corr,m_ener)>0)
    {
        m_corr = shr(m_corr,1);
        e_corr = add(e_corr,1);
    }
    m_corr = div_s(m_corr, m_ener);
    e_corr = sub(e_corr, e_ener);

    Ltmp = L_shl(m_corr, add(e_corr,1));    /* Lgain in Q16 */

    IF ( sub(coder_type,INACTIVE) == 0 )
    {
        Ltmp1 = L_max(gain_code,1);
        e_den = norm_l(Ltmp1);
        m_den = extract_h(L_shl(Ltmp1, e_den));
        /* ensure m_corr < m_den */
        test();
        IF( m_corr>0 && m_den >0)
        {
            m_corr = div_s(16384, m_den);
            e_corr = sub(14+4, e_den);
            Ltmp = L_shr(Mult_32_16(Ltmp, m_corr), e_corr);  /*Q12*/
            stmp = round_fx(L_shl(Ltmp,16));
        }
        ELSE
        {
            stmp = 0;
            move16();
        }
        IF( L_sub(core_brate,ACELP_64k) == 0 )
        {
            index = usquant_fx( stmp, &stmp, G_AVQ_MIN_INACT_64k_Q12, G_AVQ_DELTA_INACT_64k_Q12>>1, (1 << G_AVQ_BITS) );
        }
        ELSE IF( L_sub(core_brate,ACELP_48k) == 0 )
        {
            index = usquant_fx( stmp, &stmp, G_AVQ_MIN_INACT_48k_Q12, G_AVQ_DELTA_INACT_48k_Q12>>1, (1 << G_AVQ_BITS) );
        }
        ELSE
        {
            index = usquant_fx( stmp, &stmp, G_AVQ_MIN_INACT_Q12, G_AVQ_DELTA_INACT_Q12>>1, (1 << G_AVQ_BITS) );
        }
        Ltmp = Mult_32_16(gain_code,stmp);   /* Q16 * Q12 - 15 -> Q13*/
        Ltmp = L_shl(Ltmp,5);                 /* Q13 -> Q18*/
        *gain_preQ = round_fx(Ltmp);           /* Q2*/
    }
    ELSE
    {
        IF( Es_pred < 0  )
        {
            Es_pred_loc = shr(negate(Es_pred),2);
        }
        ELSE
        {
            Es_pred_loc = Es_pred;
            move16();
        }

        e_den = norm_s(Es_pred_loc);
        m_den = shl(Es_pred_loc, e_den);
        /* ensure m_corr < m_den */
        test();
        IF( m_corr>0 && m_den >0)
        {
            m_corr = div_s(16384, m_den);
            e_corr = sub(14-8, e_den);
            Ltmp = L_shr(Mult_32_16(Ltmp, m_corr), e_corr);
        }
        ELSE
        {
            Ltmp = L_deposit_l(0);
        }
        test();
        IF( L_sub(core_brate,ACELP_32k) <= 0 && L_sub(core_brate,ACELP_24k40) > 0 )
        {
            index = gain_quant_fx(&Ltmp, &stmp, LG10_G_AVQ_MIN_32kbps_Q14, LG10_G_AVQ_MAX_Q13, G_AVQ_BITS, &e_den );
        }
        ELSE
        {
            index = gain_quant_fx(&Ltmp, &stmp, LG10_G_AVQ_MIN_Q14, LG10_G_AVQ_MAX_Q13, G_AVQ_BITS, &e_den );
        }
        Ltmp = L_mult(stmp,Es_pred_loc);      /* Q0*Q8 -> Q9*/
        Ltmp = L_shl(Ltmp,add(e_den,9));      /* Q18*/
        *gain_preQ = round_fx(Ltmp);          /* Q2*/
    }
    push_indice_fx( st_fx, IND_AVQ_GAIN, index, G_AVQ_BITS );

    /*--------------------------------------------------------------*
     * Encode and multiplex subvectors into bit-stream
     *--------------------------------------------------------------*/

    AVQ_encmux_fx( st_fx, -1, x_norm, &nBits, Nsv, nq );

    /* save # of AVQ unused bits for next subframe */
    *unbits = nBits;
    move16();

    /* at the last subframe, write AVQ unused bits */
    test();
    test();
    IF( sub(i_subfr,4*L_SUBFR) == 0 && sub(extl,SWB_BWE_HIGHRATE) != 0 && sub(extl,FB_BWE_HIGHRATE) != 0 )
    {
        WHILE( *unbits > 0 )
        {
            i = s_min(*unbits, 16);
            push_indice_fx( st_fx, IND_UNUSED, 0, i );
            *unbits -= i;
        }
    }

    /*--------------------------------------------------------------*
     * DCT transform
     *--------------------------------------------------------------*/

    FOR( i=0; i<Nsv*WIDTH_BAND; i++ )
    {
        x_tran[i] = shl(x_norm[i],Q_AVQ_OUT_DEC);
        move16();
    }
    set16_fx( x_tran+Nsv*WIDTH_BAND, 0, sub(L_SUBFR,i_mult2(WIDTH_BAND,Nsv)) );

    test();
    test();
    test();

    IF(sub(coder_type,INACTIVE) != 0 && L_sub(core_brate,ACELP_32k) <= 0 && L_sub(core_brate,ACELP_24k40) > 0 && !harm_flag_acelp )
    {
        Copy( x_tran, code_preQ, L_SUBFR );
    }
    ELSE
    {
        Qdct = 0;
        move16();
        edct2_fx( L_SUBFR, 1, x_tran, out32, &Qdct, ip_edct2_64_fx, w_edct2_64_fx );
        /*qdct = sub(Q_AVQ_OUT_DEC,qdct+Q_AVQ_OUT_DEC);*/
        Qdct = negate(Qdct);
        Copy_Scale_sig_32_16(out32, code_preQ,  L_SUBFR, Qdct);    /* Output in Q_AVQ_OUT_DEC */
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
     * For inactive segments
     * - Zero-memory filtered pre-filter excitation
     * - Update of targets and gain_pit
     * For inactive segments
     * - Update xn[L_subfr-1] for updating the memory of the weighting filter
     *--------------------------------------------------------------*/

    IF ( sub(coder_type,INACTIVE) == 0 )
    {
        /*ftemp = fcode_preQ[0] *fh1[L_SUBFR-1];*/
        Ltmp = L_mult(code_preQ[0], h1[L_SUBFR-1]);  /*1+14+shift + Q_AVQ_OUT */
        FOR( i=1; i<L_SUBFR; i++ )
        {
            /*ftemp += fcode_preQ[i] * fh1[L_SUBFR-1-i];*/
            Ltmp = L_mac(Ltmp, code_preQ[i], h1[L_SUBFR-1-i]);
        }
        /*fxn[L_SUBFR-1] -= *fgain_preQ * ftemp;*/
        Ltmp = L_shr(Mult_32_16(Ltmp,*gain_preQ),sub(add(Q_AVQ_OUT_DEC,2),Q_new));   /* (2 + 1 + 14 +shift+Q_AVQ_OUT)-(Q_AVQ_OUT+2-Q_new) = 15 + Q_new + shift */
        xn[L_SUBFR-1] = round_fx(L_sub(L_mult(xn[L_SUBFR-1],32767),Ltmp)); /* -> Q_new + shift -1 */
    }
    ELSE
    {
        conv_fx( code_preQ, h1, x_tran, L_SUBFR );
        updt_tar_HR_fx( cn, cn, code_preQ, *gain_preQ, sub(Q_new, add(-15+2,Q_AVQ_OUT_DEC)), L_SUBFR );

        updt_tar_HR_fx( xn, xn, x_tran, *gain_preQ, sub(Q_new, add(-15+2,Q_AVQ_OUT_DEC)), L_SUBFR );
        *gain_pit = corr_xy1_fx( xn, y1, g_corr, L_SUBFR, 0 );

        /* clip gain if necessary to avoid problems at decoder */
        test();
        if( sub(clip_gain,1) == 0 && sub(*gain_pit, 15565)  > 0)
        {
            *gain_pit = 15565;
            move16();
        }
        updt_tar_fx( xn, xn2, y1, *gain_pit, L_SUBFR );
    }

    st_fx->use_acelp_preq = 1;
    move16();

    return;

}
/*-------------------------------------------------------------------*
 * Find target in residual domain - cn[]
 *-------------------------------------------------------------------*/

static void find_cn_fx(
    const Word16 xn[],           /* i  : target signal                                   */
    const Word16 Ap[],           /* i  : weighted LP filter coefficients                 */
    const Word16 *p_Aq,          /* i  : 12k8 LP coefficients                            */
    Word16 cn[]            /* o  : target signal in residual domain                */
)
{
    Word16 tmp, tmp_fl[L_SUBFR+M];

    set16_fx( tmp_fl, 0, M );
    Copy( xn, tmp_fl+M, L_SUBFR );
    tmp = 0;
    move16();
    preemph_fx( tmp_fl+M, PREEMPH_FAC_16k, L_SUBFR, &tmp );
    syn_filt_s_lc_fx(0, Ap, tmp_fl+M, tmp_fl+M, L_SUBFR);
    Residu3_lc_fx( p_Aq, M, tmp_fl+M, cn, L_SUBFR, 1 );

    return;
}


/*-----------------------------------------------------------------*
 * Transform domain contribution encoding
 *-----------------------------------------------------------------*/
Word16 gain_quant_fx(         /* o:  quantization index             */
    Word32 *gain,       /* i:  quantized gain (Q16)           */
    Word16 *gain16,     /* o:  quantized gain (expg)          */
    const Word16 c_min,       /* i:  log10 of lower limit in Q14    */
    const Word16 c_max,       /* i:  log10 of upper limit in Q13    */
    const Word16 bits,        /* i:   number of bits to quantize    */
    Word16 *expg        /* o:  output exponent of gain16      */
)
{
    Word16 index, levels;
    Word16 c_gain;
    Word16 e_tmp, f_tmp, exp;
    Word16 tmp, tmp1, tmp2, frac;
    Word32 L_tmp;

    levels = shl(1, bits);
    /* Prevent gain to be smaller than 0.0003.                       */
    /* This is to avoid an overflow when the gain is very small      */
    /* the log10 give a high negative value in Q13 that overflow     */
    /* on this code (the resulting value of 'index' is not affected. */
    /*   tmp2 = msu_r(L_deposit_h(c_gain),c_min,16384)               */
    L_tmp = L_max(*gain, 20);

    /*c_min = (float)log10(min);*/
    /*c_mult = (float) ((levels-1)/(log10(max)-c_min));*/

    /*tmp = c_mult * ((float)log10(*gain) - c_min);
    	= ((levels-1)/(log10(max)-log10(min)))*((float)log10(*gain) - log10(min));*/

    e_tmp = norm_l(L_tmp);
    f_tmp = Log2_norm_lc(L_shl(L_tmp, e_tmp));
    e_tmp = sub(30-16,e_tmp);/*Q(min)=16*/
    L_tmp = Mpy_32_16(e_tmp, f_tmp, 9864); /* Q16 */ /*log10(2) in Q15*/
    c_gain = round_fx(L_shl(L_tmp, 13)); /* Q13 */

    /*tmp1 = sub(c_max,c_min); Q14*/
    /*tmp2 = sub(c_gain,c_min); Q14*/

    tmp1 = msu_r(L_deposit_h(c_max/*in Q13 already*/),c_min, 16384); /*Q13*/
    tmp2 = msu_r(L_deposit_h(c_gain/*in Q13 already*/),c_min,16384); /*Q13*/
    IF(tmp1 != 0)
    {
        exp = norm_s(tmp1);
        frac = div_s(shl(1,sub(14,exp)),tmp1);  /*Q(15-exp)*/
        L_tmp = L_mult(tmp2,frac);              /*Q(30-exp)*/
        L_tmp = Mult_32_16(L_tmp,sub(levels,1));     /*Q(15-exp)*/
        index = extract_l(L_shr(L_add(L_tmp,shr(1<<14,exp)),sub(15,exp)));
    }
    ELSE
    {
        L_tmp = L_mult(tmp2,sub(levels,1)); /*Q15*/
        index = extract_l(L_shr(L_add(L_tmp,1<<14),15));
    }

    index = s_max(index ,0);
    index = s_min(index ,sub(levels,1));

    /**gain = (float)pow( 10.0, (((float)index)/c_mult) + c_min );
    y = index/c_mult + c_min;
      = (index/(levels-1))*(log10(max) - log10(min)) + log10(min);
      = z*log10(max) + (1-z)*log10(min)
      z = (index/(levels-1))*/
    tmp = div_s(index,sub(levels,1)); /*Q15*/
    L_tmp = L_mult(tmp,c_max);/*Q29*/
    L_tmp = L_mac0(L_tmp,sub(32767,tmp),c_min); /*Q29*/

    L_tmp = Mult_32_16(L_tmp,27213); /*Q27, 3.321928 in Q13*/
    L_tmp = L_shr(L_tmp,11); /*Q27->Q16*/

    frac = L_Extract_lc(L_tmp, expg); /* Extract exponent of gcode0 */

    *gain16 = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767 */
    *expg = sub(*expg, 14);
    move16();

    return(index);
}
