/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "rom_com_fx.h"   /* Static table prototypes                */
#include "rom_enc_fx.h"   /* Static table prototypes                */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define NMAX          8  /* Control of the routine's complexity */
/* #define FAC_DELTA  16.0f */
#define SFAC_DELTA  11

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/

static Word16 cod_2pos_fx( const Word16 ind1, const Word16 ind2, const Word16 sign1, const Word16 sign2, const Word16 n );

static void gauss2v_fx( Encoder_State_fx *st_fx, const Word16 h[], const Word16 xn[], const Word16 dn[], Word16 code[],
                        Word16 y1[], Word32 *gain, const Word16 lg, const Word16 shift, const Word16 Q_new, const Word16 nb_bits );

/*-------------------------------------------------------------------*
 * Gaus_encode
 *
 * Encoder UnVoiced excitation coding using Gaussian codebooks
 * - ACELP quantized Gaussian excitation
 * - gain quantization
 * - Total excitation for UnVoiced coders
 * - Updates
 *-------------------------------------------------------------------*/
Word16 gaus_encode_fx(
    Encoder_State_fx *st_fx,                    /* i/o: encoder state structure */
    const Word16 i_subfr,          			/* i  : subframe index                              	*/
    const Word16 *h1,              			/* i  : weighted filter input response              	*/
    const Word16 *xn,              			/* i  : target vector                               	*/
    Word16 *exc,             			/* o  : pointer to excitation signal frame          	*/
    Word16 *mem_w0,          			/* o  : weighting filter denominator memory         	*/
    Word16 *clip_gain,       			/* o  : memory of gain of pitch clipping algorithm  	*/
    Word16 *tilt_code,       			/* o  : synthesis excitation spectrum tilt          	*/
    Word16 *code,        				/* o  : algebraic excitation                     Q9 	*/
    Word32 *gain_code,   				/* o  : Code gain.                               Q16	*/
    Word16 *y2,          				/* o  : zero-memory filtered adaptive excitation Q9 	*/
    Word16 *gain_inov,	   			/* o  : innovation gain                          Q12	*/
    Word16 *voice_fac,       			/* o  : voicing factor                           Q15	*/
    Word16 *gain_pit,			   		/* o  : adaptive excitation gain                 Q14	*/
    const Word16 Q_new,            			/* i  : scaling factor                              	*/
    const Word16 shift,		   				/* i  : scaling factor                              	*/
    Word32 *norm_gain_code,		    /* o  : normalized innovative cb. gain			 Q16 	*/
    const Word32  core_brate         		/* i  : core bitrate									*/
)
{
    Word16 nb_bits, idx;
    Word16 i=0;
    Word32 Ltmp;
    Word16 dn[L_SUBFR], exp_code, gcode;    /* Correlation between xn and h1 */
    Word16 exp,tmp;

    /*----------------------------------------------------------------*
     *  Encode gaussian excitation
     *----------------------------------------------------------------*/

    /* Correlation between target xn2[] and impulse response h1[] */
    corr_xh_fx(xn, dn, h1);

    nb_bits = FCB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, UNVOICED, i_subfr, TC_SUBFR2IDX_fx(-1))];

    gauss2v_fx(st_fx, h1, xn, dn, code, y2, gain_code, L_SUBFR, shift, Q_new, shr(nb_bits,1));

    /*----------------------------------------------------------------*
      *  Encode gaussian gain
      *----------------------------------------------------------------*/

    /* codeword energy computation */
    Ltmp = Dot_product12(code, code, L_SUBFR, &exp_code);

    exp_code = sub(exp_code, 18 + 6);         /* exp: -18 (code in Q9), -6 (L_subfr = 64) */
    Ltmp = Isqrt_lc(Ltmp, &exp_code);
    *gain_inov = extract_h(L_shl(Ltmp, sub(exp_code, 3)));  /* g_code_inov in Q12 */

    nb_bits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, UNVOICED, i_subfr, TC_SUBFR2IDX_fx(-1))];
    /* low bound = -30; stepSize  = 1.71875; inv_stepSize  = 0.5818181 */
    idx = gain_enc_gaus_fx(gain_code, nb_bits, -7680, 28160, 19065);
    push_indice_fx( st_fx, IND_GAIN, idx, nb_bits );

    /*----------------------------------------------------------------*
     * Total excitation for Unvoiced coders
     *----------------------------------------------------------------*/
    gcode = round_fx(L_shl(*gain_code, Q_new)); /* scaled gain_code with Qnew */
    FOR (i = 0; i < L_SUBFR;  i++)
    {
        exc[i+i_subfr] = round_fx(L_shl(L_mult(gcode, code[i]), 15-9));
    }

    /*----------------------------------------------------------------*
     * Updates: last value of new target is stored in mem_w0
     *----------------------------------------------------------------*/

    Ltmp = L_mult(gcode, y2[L_SUBFR - 1]);
    Ltmp = L_shl(Ltmp, add(5, shift));
    Ltmp = L_negate(Ltmp);
    Ltmp = L_mac(Ltmp, xn[L_SUBFR - 1], 16384);
    Ltmp = L_shl(Ltmp, sub(1, shift));
    *mem_w0 = round_fx(Ltmp);
    init_gp_clip_fx(clip_gain);        /* reset pitch clipping parameters  */

    *gain_pit = 0;
    *tilt_code = 0;
    move16(); /* purely unvoiced */
    *voice_fac = -32768;
    move16(); /* purely unvoiced */
    exp = sub(norm_s(*gain_inov),1);
    exp = s_max(exp, 0);

    tmp = div_s(shr(8192,exp),*gain_inov);
    *norm_gain_code = L_shr(Mult_32_16(*gain_code, tmp),sub(1,exp));
    move16();

    return (L_SUBFR<<6);
}

/*-------------------------------------------------------------------*
 * gauss2v()
 *
 * encoder of Gaussian Codebook for unvoiced
 * consisting of addition of 2 Gaussian vectors
 *
 * One Gaussian vector of 192 values vectors delayed by 2
 *-------------------------------------------------------------------*/
void gauss2v_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word16 h[],     			/* i  : weighted LP filter impulse response     Q15 */
    const Word16 xn[],    			/* i  : target signal                           Q12 */
    const Word16 dn[],    			/* i  : backward filtered target                Q12 */
    Word16 code[],  			/* o  : gaussian excitation                     Q9  */
    Word16 y1[],    			/* o  : zero-memory filtered gauss. excitation  Q8  */
    Word32 *gain,   			/* o  : excitation gain. 32-bit number in       Q16 */
    const Word16 lg,      			/* i  : subframe size                           Q0  */
    const Word16 shift,   			/* i  : Scaling factor                          Q0  */
    const Word16 Q_new,   			/* i  : Scaling factor                          Q0  */
    const Word16 nb_bits  			/* i  : nb ob bits per track (max 6)                */
)
{
    Word16 i, j, ind1, ind2;
    Word16 nvec, step;
    Word32 cor, cora, dotprod;
    Word16 enerw;
    Word32 eneri,cor2;
    Word32 enerw32,cor2w32;
    Word16 *cpt1;
    Word16 *pt1, *pt2;
    Word32 max[NMAX+1];
    Word16 *pos[NMAX+1];
    Word32 sign[NMAX+1];
    Word32 ener[NMAX+1], corr[NMAX+1], ener1;
    Word16 dico2[L_SUBFR*NMAX];
    Word16 exp_num;
    Word16 exp_den;
    Word16 Num;
    Word16 Den;
    Word32 GainPortion1;
    Word32 GainPortion2;
    Word32 cor_abs;
    Word16 cor_neg;
    Word16 div_result;
    Word32 ener_sqrt;
    Word32 Portion;
    Word16 sign1,sign2;
    Word16 enerw_norm,enerw_mantissa;
    Word16 cor2w_norm,cor2w_mantissa;
    Word16 eneri_norm,eneri_mantissa;
    Word16 cor2_norm ,cor2_mantissa;
    Word16 difference_norm;
    Word32 cor32; /* 32-bit intermediate value*/
    Word16 hi1, lo1;
    Word16 update_best;
    Word16 idx;
    Word32 Lc0,Lc1, Lnum, Lden;
    Word16 gxx, gcc,index_delta, delta, m_sign, inv_delta;
    Word16 hg[190],Gaus_dico2[190];
    Word16 shiftP3;

    /*----------------------------------------------------------------*
     * Encode the tilt of gaussian excitation
     *----------------------------------------------------------------*/

    /* Compute spectral tilt of target */
    Lc0 = L_mult(xn[1], xn[1]);
    Lc1 = L_mult(xn[1], xn[0]);
    FOR (i=2; i<L_SUBFR; i++)
    {
        /* fc0 += xn[i]*xn[i]   */
        /* fc1 += xn[i]*xn[i-1] */
        Lc0 = L_mac(Lc0, xn[i], xn[i]);
        Lc1 = L_mac(Lc1, xn[i], xn[i-1]);
    }
    /* fgxx = fc1/fc0 */
    exp_num = sub(norm_l(Lc1), 1);
    Num = extract_h(L_shl(Lc1,exp_num));
    m_sign = s_or(shr(Num, 16), 1);       /* Remove sign */
    Num = abs_s(Num);
    Lc0 = L_max(Lc0, 1);
    exp_den = norm_l(Lc0);
    Den = extract_h(L_shl(Lc0,exp_den));
    gxx = shr(div_s(Num, Den), sub(exp_num, sub(exp_den, 2)));  /* Q13 */
    gxx = i_mult2(gxx, m_sign);            /* Apply sign */

    set16_fx(hg, 0, 190);                     /* Compute spectral tilt of filtered codebook */
    Copy(h, hg, L_SUBFR);
    conv_fx(gaus_dico_fx, hg, Gaus_dico2, 190);

    Lc0 = L_mult(Gaus_dico2[1], Gaus_dico2[1]);
    Lc1 = L_mult(Gaus_dico2[1], Gaus_dico2[0]);
    FOR (i=2; i<190; i++)
    {
        /* fc0 += fgaus_dico2[i]*fgaus_dico2[i]   */
        /* fc1 += fgaus_dico2[i]*fgaus_dico2[i-1] */
        Lc0 = L_mac(Lc0, Gaus_dico2[i], Gaus_dico2[i]);
        Lc1 = L_mac(Lc1, Gaus_dico2[i], Gaus_dico2[i-1]);
    }

    /* fgcc = fc1/fc0 */

    exp_num = sub(norm_l(Lc1), 1);
    Num = extract_h(L_shl(Lc1,exp_num));
    m_sign = s_or(shr(Num, 16), 1);  /* Remove sign */
    Num = abs_s(Num);

    Lc0 = L_max(Lc0, 1);
    exp_den = norm_l(Lc0);
    Den = extract_h(L_shl(Lc0, exp_den));
    gcc = shr(div_s(Num, Den), sub(exp_num, sub(exp_den, 2))); /* Q13 */
    gcc = i_mult2(gcc, m_sign);      /* Apply sign */

    /* fdelta = (1-fgcc*fgxx) / (2*fgcc+fgxx) Compute and quantize spectral tilt modification factor */
    Lnum = L_sub(134217728L, L_mult(gcc, gxx));         /* Q30 */
    Lden = L_mac(L_mult(gxx, 8192), gcc, 16384);        /* Q30 */

    exp_num = sub(norm_l(Lnum), 1);
    Num = extract_h(L_shl(Lnum,exp_num));
    m_sign = s_or(shr(Num, 16), 1);                     /* Remove sign */
    Num = abs_s(Num);

    Lden = L_max(Lden, 1);
    exp_den = norm_l(Lden);
    Den = extract_h(L_shl(Lden,exp_den));

    delta = shr(div_s(Num, Den), sub(exp_num, exp_den));/* Q15 */
    delta = i_mult2(delta, m_sign);                      /* Apply sign */
    /* index_delta = (short)(FAC_DELTA * fdelta) */
    index_delta = shr(delta, SFAC_DELTA);

    /* index_delta  [0,7] */
    index_delta = s_max(index_delta, 0);
    index_delta = s_min(index_delta, 7);

    /* fdelta = STEP_DELTA * (float)index_delta */
    delta = shl(index_delta, 11);     /* delta in Q15 */

    IF (delta > 0)                    /* Adapt spectral tilt of initial codebook */
    {
        /* Computation of 1 / (1+fdelta*fdelta) */
        inv_delta = inv_delta_tab[sub(index_delta, 1)];
        move16();  /* Q15 */

        /* fgaus_dico2[0] = gaus_dico[0] */
        Gaus_dico2[0] = gaus_dico_fx[0];
        FOR (i=1; i<190; i++)
        {
            /* fgaus_dico2[i] = (gaus_dico[i] - fdelta*gaus_dico[i-1]) / (1 + fdelta*fdelta) */
            Lnum = L_msu(L_deposit_h(gaus_dico_fx[i]), delta, gaus_dico_fx[i-1]);
            Gaus_dico2[i] = round_fx(Mpy_32_16_1(Lnum, inv_delta));
        }
    }
    ELSE
    {
        FOR (i=0; i<190; i++)
        {
            /* fgaus_dico2[i] = gaus_dico[i] */
            Gaus_dico2[i] = gaus_dico_fx[i];
            move16();
        }
    }

    /*----------------------------------------------------------------*
     * Initializations
     *----------------------------------------------------------------*/

    ind1 = 0;
    move16();
    ind2 = 0;
    move16();

    nvec = shl(1, nb_bits);
    step = shr(0x80, nb_bits);

    /*----------------------------------------------------------------*
     * dot product between dn and gaussian codevectors,
     * keep NMAX best vectors
     *----------------------------------------------------------------*/

    set32_fx(max, 0, NMAX+1);
    set32_fx(sign, 0, NMAX+1);

    FOR (i = 0; i < NMAX+1; i++)
    {
        pos[i] = (Word16 *)Gaus_dico2;
    }

    cpt1 = Gaus_dico2;
    move16();

    FOR (i=0; i< nvec; i++)
    {
        /* Dot product without normalization, because values are compared with each other afterwards. */
        cor = Dot_product(cpt1, dn, lg); /* Q12 * Q12 * length of 64 + 1 left shift ==> Q31*/
        cora = L_abs(cor);
        j = NMAX-1;
        move16();

        DO
        {
            IF (L_sub(cora, max[j]) >= 0)
            {
                max[j+1]  = max[j];
                move32();   /*Q31*/
                pos[j+1]  = pos[j];
                move16();   /*Pointer*/
                sign[j+1] = sign[j];
                move32();   /*Q31*/
                max[j]    = cora;
                move32();   /*Q31*/
                pos[j]    = cpt1;
                move16();   /*Pointer*/
                sign[j]   = cor;
                move32();   /*Q31*/
            }
            j--;
        }
        WHILE (j >= 0);
        cpt1 += step;
    }

    /*----------------------------------------------------------------*
     * filter selected vectors
     * put sign
     * compute energy
     *----------------------------------------------------------------*/

    pt1 = dico2;
    move16();
    FOR (i=0; i<NMAX; i++)
    {
        /* Input vector (pos) Q12, filter coefs in Q15, result in same format as input vector (Q12) */
        conv_fx(pos[i], h, pt1, lg);

        /* put sign and compute energy */
        IF (sign[i] < 0)
        {
            FOR(j=0; j<lg; j++)
            {
                pt1[j] = negate(pt1[j]);
                move16(); /*Store into dico2*/
            }
        }
        ener[i] = Dot_product(pt1,pt1,lg);  /* pt1 points to filtered vector in dico2, in Q12 */
        move32();                           /* Result is for Q12 * Q12 with length of 64 (6 bits) + 1 left shift => Q31 */
        corr[i] = Dot_product(pt1,xn,lg);   /* must be equal to sign[i] !! */
        move32();                           /* pt1 points into dico2, in Q12. xn is in Q12 */
        /* Result is for Q12 * Q12 with length of 64 (6 bits) + 1 left shift => Q31 */
        pt1 += L_SUBFR;
    }

    /*------------------------------------------------------------------------*
     * try all combinations of NMAX best vectors
     *------------------------------------------------------------------------*/

    pt1 = dico2;
    move16();

    /* Initial values for search algorithm */
    enerw32 = L_deposit_h(0x80);
    cor2w32 = L_deposit_l(-2);
    enerw_norm = norm_l(enerw32);
    cor2w_norm = norm_l(cor2w32);
    cor2w_mantissa = round_fx(L_shl(cor2w32,cor2w_norm));
    enerw_mantissa = round_fx(L_shl(enerw32,enerw_norm));

    FOR (i=0; i< NMAX; i++)
    {
        pt2 = pt1;
        move16();
        FOR (j=i; j<NMAX; j++)
        {
            cor32 = L_add(corr[i], corr[j]);   /* Q31 */

            dotprod=Dot_product(pt1, pt2, lg); /* Q12 * Q12 * length of 64 + 1 left shift ==> Q31 */

            /* eneri = round_fx(ener[i]) + round_fx(ener[j]) + 2*round_fx(dotprod) */
            /* Use ScalingShift to stay aligned with ener[] */
            eneri=L_shl(dotprod, 1);      /* One left shift added for factor of 2 */
            eneri=L_add(ener[i], eneri);
            eneri=L_add(ener[j], eneri);  /* Q31 */

            lo1 = L_Extract_lc(cor32, &hi1);
            cor2 = Sad_32(0, hi1, lo1); /* Square + Add */

            cor2_norm = norm_l(cor2);
            eneri_norm = norm_l(eneri);
            cor2_mantissa = round_fx(L_shl(cor2, cor2_norm));
            eneri_mantissa = round_fx(L_shl(eneri, eneri_norm));

            difference_norm = sub(add(cor2_norm, enerw_norm), add(cor2w_norm, eneri_norm));

            update_best = 0;
            move16();

            IF (difference_norm > 0)
            {
                if (L_sub(L_shr(L_mult(cor2_mantissa, enerw_mantissa), difference_norm),
                          L_mult(cor2w_mantissa, eneri_mantissa)) > 0)
                {
                    update_best = 1;
                    move16();
                }
            }
            ELSE
            {
                if (L_msu(L_shl(L_mult(cor2w_mantissa, eneri_mantissa), difference_norm), cor2_mantissa, enerw_mantissa) < 0)
                {
                    update_best=1;
                    move16();
                }
            }
            IF (update_best != 0)
            {
                cor2w_mantissa = cor2_mantissa;
                move16();
                cor2w_norm = cor2_norm;
                move16();
                enerw_mantissa = eneri_mantissa;
                move16();
                enerw_norm = eneri_norm;
                move16();
                ind1 = i;
                move16();
                ind2 = j;
                move16();
            }
            pt2 += L_SUBFR;
        }
        pt1 += L_SUBFR;
    }

    enerw = round_fx(L_shr(L_deposit_h(enerw_mantissa), enerw_norm));

    /*----------------------------------------------------------------*
     * Compute zero-memory filtered gauss. excitation y
     *----------------------------------------------------------------*/

    pt1 = dico2 + ind1*L_SUBFR;
    move16(); /*Pointer arithmetic*/
    pt2 = dico2 + ind2*L_SUBFR;
    move16();

    shiftP3 = add(shift, 3);
    FOR (i=0; i<lg; i++)
    {
        /* Sum of 2 Q12 values, must give a Q1.8 */
        y1[i] = shr(add(pt1[i], pt2[i]), shiftP3);
        move16();  /* Compensate for "shift" */
    }

    /*----------------------------------------------------------------*
     * signs of vectors
     *----------------------------------------------------------------*/

    sign1 = (-32768);
    move16();
    if (sign[ind1] >= 0)
    {
        sign1 = 32767;
        move16();
    }

    sign2 = (-32768);
    move16();
    if (sign[ind2] >= 0)
    {
        sign2 = 32767;
        move16();
    }

    /*----------------------------------------------------------------*
     * Compute code
     *----------------------------------------------------------------*/

    pt1 = pos[ind1];
    move16();  /* Points to gaussian vector (gaus_dico_fx) in Q12 */
    pt2 = pos[ind2];
    move16();  /* Points to gaussian vector (gaus_dico_fx) in Q12 */

    /* sign[ind1] and sign[ind2] */
    FOR (i=0; i<lg; i++)
    {
        /* code[i]=(pt1[i]*sign1 + pt2[i]*sign2) /8 */
        /* Division by 8 (shift by 3) is for scaling (Q12 to Q0.9 output) */
        code[i] = shr(add(mult(pt1[i], sign1), mult(pt2[i], sign2)), 3);
        move16();
    }

    cor = L_add(corr[ind1], corr[ind2]);

    /*----------------------------------------------------------------*
     * Compute index
     *----------------------------------------------------------------*/

    i = (Word16)((pos[ind1] - Gaus_dico2) / step);  /* Division by step can be replaced by shift. Pointer arithmetic */
    j = (Word16)((pos[ind2] - Gaus_dico2) / step);  /* Division by step can be replaced by shift. Pointer arithmetic */

    idx = cod_2pos_fx(i, j, sign1, sign2, nvec);
    move16();

    push_indice_fx( st_fx, IND_GAUS_CDBK_INDEX, idx, 2*nb_bits+1 );
    push_indice_fx( st_fx, IND_TILT_FACTOR, index_delta, 3 );

    /*----------------------------------------------------------------*
     * Find quantized gain
     *----------------------------------------------------------------*/

    /* Divide cor/enerw: intermediate result stored into GainPortion1 */
    cor_neg = 0;
    move16();
    if (cor < 0)     /* Make Num positive. */
    {
        cor_neg = 1;
        move16();
    }
    cor_abs = L_abs(cor);

    exp_num = sub(norm_l(cor_abs), 1);
    exp_den = norm_s(enerw);
    Num = round_fx(L_shl(cor_abs, exp_num));
    Den = shl(enerw, exp_den);

    GainPortion1 = L_deposit_l(0); /* Unexpected division by zero. Eliminate this gain contribution */
    IF (Den !=0) /* Protection against division by zero */
    {
        div_result = div_s(Num, Den); /* Q15 */
        IF (cor_neg != 0)
        {
            div_result = negate(div_result); /* Retrieve sign */
        }
        /* Re-scale to compensate for normalization*/
        GainPortion1 = L_shr(L_deposit_l(div_result), sub(exp_num, exp_den));
    }

    ener1 = Dot_product(xn, xn, lg); /* Q12 * Q12 * length of 64 + 1 left shift ==> Q31 */

    exp_num = sub(norm_s(enerw), 1);
    exp_den = norm_l(ener1);
    Num = shl(enerw, exp_num);
    Den = round_fx(L_shl(ener1, exp_den));

    GainPortion2 = L_deposit_l(0); /* Unexpected division by zero. Eliminate this gain contribution */
    IF (Den != 0) /* Protection against division by zero */
    {
        div_result = div_s(Num, Den); /* Q15 */

        /* Re-scale to compensate for normalization*/
        GainPortion2 = L_shr(L_deposit_l(div_result), sub(exp_num, exp_den));
    }

    ener_sqrt = Isqrt(L_shl(GainPortion2, 1));  /* Make value a Q16 prior to division (align on power of 4) */
    ener_sqrt = L_shr(ener_sqrt, 8);            /* Left-shift Q23 result to make a Q15 result */

    Portion = Mult_32_16(GainPortion1, 19661);  /* Performs GainPortion1*.6 */
    Portion = Madd_32_16(Portion, ener_sqrt, 13107); /* Performs ener_sqrt*.4 */

    /* Gain must be output in a 32-bit variable as a Q16 */
    /* Compensate for Q_new */
    *gain = L_shl(Portion, sub(13, Q_new));
    move32();

    return;
}


/*---------------------------------------------------------------------*
 * Put selected codevector positions and signs into quantization index
 *---------------------------------------------------------------------*/
static Word16 cod_2pos_fx(	/* o  :  codebook quantization index  */
    const Word16 ind1,    		/* i  :  index of 1st gaussian vector */
    const Word16 ind2,    		/* i  :  index of 2nd gaussian vector */
    const Word16 sign1,   		/* i  :  sign of 1st gaussian vector  */
    const Word16 sign2,   		/* i  :  sign of 2nd gaussian vector  */
    const Word16 n        		/* i  :  nb. of codebook vectors      */
)
{
    Word16 i1, i2, index, s1, s2;
    s1 = 1;
    move16();

    if (sign1 > 0)
    {
        s1 = 0;
        move16();
    }
    s2 = 1;
    move16();
    if (sign2 > 0)
    {
        s2 = 0;
        move16();
    }

    IF (sub(s1, s2)==0)
    {
        IF (sub(ind1, ind2) <= 0)
        {
            i1 = ind1;
            move16();
            i2 = ind2;
            move16();
        }
        ELSE
        {
            i1 = ind2;
            move16();
            i2 = ind1;
            move16();
        }
    }
    ELSE
    {
        IF (sub(ind1, ind2)>0)
        {
            i1 = ind1;
            move16();
            i2 = ind2;
            move16();
        }
        ELSE
        {
            i1 = ind2;
            move16();
            i2 = ind1;
            move16();
            s1 = s2;
            move16();
        }
    }

    index = extract_l(L_mult(i1, n));
    index = add(index, shl(i2, 1));
    index = add(index, s1);

    return index;
}
