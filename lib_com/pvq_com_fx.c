/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */

#include "prot_fx.h"       /* Function prototypes                    */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"        /* required for wmc_tool */


#define OFFSET 21
#define STEP   9



/*-------------------------------------------------------------------*
 * own_cos_fx()
 *
 *  Bit-exact cosine
 *-------------------------------------------------------------------*/

Word16 own_cos_fx(
    const Word16 x
)
{
    const Word16 a[4] = {14967,   -25518,   3415,   32351};
    Word32 b;
    Word16 out;
    UWord16 lsb;


    b = L_deposit_l(a[0]);
    b = L_shl((Word32)add(a[1], extract_h(L_mult0((Word16)b, x))), 1);
    Mpy_32_16_ss(b, x, &b, &lsb);
    b = L_add((Word32)a[2], b);
    Mpy_32_16_ss(b, x, &b, &lsb);
    b = L_add((Word32)a[3], b);
    out = extract_l(b);
    return out;
}

/*-------------------------------------------------------------------*
 * get_angle_res_fx()
 *
 *
 *-------------------------------------------------------------------*/

Word16 get_angle_res_fx(
    const Word16 dim,
    const Word16 bits
)
{
    static const Word16 pow2_angle_res[8] = {16384, 17867, 19484, 21247, 23170, 25268, 27554, 30048}; /* Q14 */
    Word32 acc;
    Word16 r, tmp, expo;
    Word16 bits_min = 28;
    Word16 bits_max = 96;

    Word16 angle_res, angle_bits;

    tmp = sub(shl(dim, 1), 1);
    acc = L_add(bits, L_mult0(tmp, bits_min));
    r = ratio(acc, (Word32)tmp, &expo);  /* r in Q14 */
    r = shr(r, add(14, expo));           /* r in Q0 */
    tmp = sub(bits, bits_max);

    angle_bits = r;
    move16();
    if (sub(tmp, r) < 0)
    {
        angle_bits = tmp;
        move16();
    }

    if (sub(56, angle_bits) < 0)
    {
        angle_bits = 56;
        move16();
    }

    IF (sub(angle_bits, 4) < 0)
    {
        angle_res = 1;
        move16();
    }
    ELSE
    {
        tmp = s_and(angle_bits, 0x7);
        expo = sub(14, shr(angle_bits, 3));
        angle_res = shr(pow2_angle_res[tmp], expo);
        angle_res = shl(shr(add(angle_res, 1), 1), 1);
    }

    return angle_res;
}

/*-------------------------------------------------------------------*
 * get_pulse_fx ()
 *
 *
 *-------------------------------------------------------------------*/

Word16 get_pulse_fx(
    const Word16 q
)
{
    Word16 s, m, k, tmp;
    Word32 acc;

    IF (sub(q, OFFSET) <= 0)
    {
        return q;
    }
    ELSE
    {
        tmp = sub(q, OFFSET);
        s = extract_h(L_mult(tmp, 3641));  /* Time 1/9.0 */
        m = extract_l(L_sub(L_sub((Word32)q, (Word32)OFFSET), L_mult0(s, STEP)));
        k = shl(shr(255, sub(8, s)), 1);

        acc = L_add(L_add(OFFSET, L_mult0(STEP, k)), L_shl(m, add(s, 1)));
        tmp = KMAX_FX;
        move16();
        if (L_sub(KMAX_FX, acc) >= 0)
        {
            tmp = extract_l(acc);
        }
        return tmp;
    }
}

/*-------------------------------------------------------------------*
 * bits2pulses_fx()
 *
 *
 *-------------------------------------------------------------------*/

Word16 bits2pulses_fx(
    const Word16 N,
    const Word16 bits,
    const Word16 strict_bits
)
{
    const unsigned char *tab;
    Word16 B, mid, low, high, q;
    Word16 i;

    tab = hBitsN[N];
    low = 1;
    high = (Word16)tab[0];
    B = sub(bits, 1);

    IF (sub(tab[high], B) <= 0)
    {
        q =  high;
        move16();
    }
    ELSE
    {
        FOR (i = 0; i < 6; i++)
        {
            mid = (low + high)>>1;

            IF (tab[mid] >= B)
            {
                high = mid;
                move16();
            }
            ELSE
            {
                low = mid;
                move16();
            }
        }

        IF (strict_bits != 0)
        {
            q = low;
            move16();
        }
        ELSE IF (sub(sub(B, tab[low]), sub(tab[high], B)) <= 0)
        {
            q = low;
            move16();
        }
        ELSE
        {
            q = high;
        }
    }

    return q;
}

/*-------------------------------------------------------------------*
 * bits2pulses_fx()
 *
 *
 *-------------------------------------------------------------------*/

Word16 pulses2bits_fx(
    const Word16 N,
    const Word16 P
)
{
    const unsigned char *tab;
    Word16 bits;

    tab = hBitsN[N];

    bits = 0;
    move16();
    if (P != 0)
    {
        bits = add(tab[P], 1);
    }

    return bits;
}

/*--------------------------------------------------------------------------*
 * log2_div_fx2()
 *
 *  Logarithm of division
 *--------------------------------------------------------------------------*/

Word16 log2_div_fx(            /* o : Log2 of division, Q11 */
    const Word16 input_s,      /* i : Numerator         Q15 */
    const Word16 input_c       /* i : Denominator       Q15 */
)
{
    Word16 mc, nc, ms, ns, d, z;
    Word16 result;
    Word32 acc;

    /* log2|sin(x)/cos(x)| = log2|sin(x)| - log2(cos(x)|
     *                     = log2|ms*2^-ns| - log2|mc*2^-nc|, where 0.5 <= ms < 1.0 and 0.5 <= mc < 1.0
     *                     = log2|ms| - ns - log2|mc| + nc
     *
     * Approximate log2(y) by a 2nd order least square fit polynomial. Then,
     *
     * log2|sin(x)/cos(x)| ~ (a0*ms^2 + a1*ms + a2) - ns - (a0*mc^2 + a1*mc + a2) + nc
     *                     = a0*(ms^2 - mc^2) + a1*(ms - mc) - ns + nc
     *                     = a0*(ms + mc)*(ms - mc) + a1*(ms - mc) - ns + nc
     *                     = (a0*(ms + mc) + a1)*(ms - mc) - ns + nc
     *                     = (a0*ms + a0*mc + a1)*(ms - mc) - ns + nc
     */
    ns = norm_s(input_s );    /* exponent */
    nc = norm_s(input_c );    /* exponent */

    ms  = shl(input_s, ns);   /* mantissa */
    mc  = shl(input_c, nc);   /* mantissa */

    acc = L_mac(538500224L, mc, -2776);  /* a0*mc + a1, acc(Q27), a0(Q11), a1(Q27) */
    z = mac_r(acc, ms, -2776);           /* z in Q11, a0 in Q11 */
    d = sub(ms, mc);                     /* d in Q15 */
    z = mult_r(z, d);                    /* z in Q11 */

    result = add(z, shl(sub(nc, ns), 11));

    return result;
}

/*--------------------------------------------------------------------------*
 * apply_gain()
 *
 * Apply gain
 *--------------------------------------------------------------------------*/

void apply_gain_fx(
    const Word16 *ord,                       /* i  : Indices for energy order                       */
    const Word16 *band_start,                /* i  : Sub band start indices                         */
    const Word16 *band_end,                  /* i  : Sub band end indices                           */
    const Word16 num_sfm,                    /* i  : Number of bands                                */
    const Word16 *gains,                     /* i  : Band gain vector                       Q12     */
    Word16 *xq                         /* i/o: Float synthesis / Gain adjusted synth  Q15/Q12 */
)
{
    Word16 band,i;
    Word16 g;   /* Q12 */

    FOR ( band = 0; band < num_sfm; band++)
    {
        g = gains[ord[band]];

        FOR( i = band_start[band]; i < band_end[band]; i++)
        {
            /*xq[i] *= g; */
            xq[i] = mult_r(g, xq[i]);
            move16();   /*12+15+1-16=12 */
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * fine_gain_quant()
 *
 * Fine gain quantization
 *--------------------------------------------------------------------------*/

void fine_gain_quant_fx(
    Encoder_State_fx *st_fx,
    const Word16 *ord,                       /* i  : Indices for energy order                     */
    const Word16 num_sfm,                    /* i  : Number of bands                              */
    const Word16 *gain_bits,                 /* i  : Gain adjustment bits per sub band            */
    Word16 *fg_pred,                   /* i/o: Predicted gains / Corrected gains        Q12 */
    const Word16 *gopt                       /* i  : Optimal gains                            Q12 */
)
{
    Word16 band;
    Word16 gbits;
    Word16 idx;
    Word16 gain_db,gain_dbq;
    Word16 err;

    Word16 tmp1, tmp2, exp1, exp2;
    Word32 L_tmp;
    UWord16 lsb;

    FOR ( band = 0; band < num_sfm; band++)
    {
        gbits = gain_bits[ord[band]];
        test();
        IF ( fg_pred[band] != 0 && gbits > 0 )
        {
            /*err = gopt[band] / fg_pred[band]; */
            exp1 = norm_s(gopt[band]);
            exp1 = sub(exp1, 1);
            tmp1 = shl(gopt[band], exp1);
            exp2 = norm_s(fg_pred[band]);
            tmp2 = shl(fg_pred[band], exp2);
            exp1 = add(15, sub(exp1, exp2));
            err = div_s(tmp1, tmp2);

            /*gain_db = 20*(float)log10(err); */
            tmp1 = norm_s(err);
            exp2 = Log2_norm_lc(L_deposit_h(shl(err, tmp1)));
            tmp1 = sub(14, tmp1);
            tmp1 = sub(tmp1, exp1);
            L_tmp = L_Comp(tmp1, exp2);
            Mpy_32_16_ss(L_tmp, 24660, &L_tmp, &lsb);   /* 24660 = 20*log10(2) in Q12 */ /*16+12-15=13 */
            gain_db = round_fx(L_shl(L_tmp, 17));

            idx = squant_fx(gain_db, &gain_dbq, finegain_fx[gbits-1], gain_cb_size[gbits-1]);
            push_indice_fx( st_fx, IND_PVQ_FINE_GAIN, idx, gbits );

            /* Update prediced gain with quantized correction */
            /*fg_pred[band] *= (float)pow(10, gain_dbq * 0.05f);  */
            L_tmp = L_mult0(gain_dbq, 21771);   /* 21771=0.05*log2(10) */   /* 14+17=31 */
            L_tmp = L_shr(L_tmp, 15);
            tmp1 = L_Extract_lc(L_tmp, &exp1);
            tmp1 = abs_s(tmp1);
            tmp1 = extract_l(Pow2(14, tmp1));
            exp1 = sub(14, exp1);

            L_tmp = L_mult0(fg_pred[band], tmp1);   /*12+exp1 */
            fg_pred[band] = round_fx(L_shl(L_tmp, sub(16, exp1))); /*12+exp1+16-exp1-16=12 */
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * srt_vec_ind()
 *
 *  sort vector and save  sorting indeces
 *-------------------------------------------------------------------*/

void srt_vec_ind16_fx (
    const Word16 *linear,      /* linear input */
    Word16 *srt,         /* sorted output*/
    Word16 *I,           /* index for sorted output  */
    Word16 length
)
{
    Word16 pos,npos;
    Word16 idxMem;
    Word16 valMem;

    /*initilize */
    FOR (pos = 0; pos < length; pos++)
    {
        I[pos] = pos;
        move16();
    }

    Copy(linear, srt,length);

    /* now iterate */
    FOR (pos = 0; pos < (length - 1); pos++)
    {
        FOR (npos = (pos + 1); npos < length;  npos++)
        {
            IF (sub(srt[npos], srt[pos]) < 0)
            {
                idxMem    = I[pos];
                move16();
                I[pos]    = I[npos];
                move16();
                I[npos]   = idxMem;
                move16();

                valMem    = srt[pos];
                move16();
                srt[pos]  = srt[npos];
                move16();
                srt[npos] = valMem;
                move16();
            }
        }
    }

    return;
}

/*-----------------------------------------------------------------------------
 * atan2_fx():
 *
 * Approximates arctan piecewise with various 4th to 5th order least square fit
 * polynomials for input in 5 segments:
 *   - 0.0 to 1.0
 *   - 1.0 to 2.0
 *   - 2.0 to 4.0
 *   - 4.0 to 8.0
 *   - 8.0 to infinity
 *---------------------------------------------------------------------------*/
Word16 atan2_fx(  /* o: Angle between 0 and PI/2 radian (Q14) */
    const Word32 y,  /* i: Argument must be positive (Q15) */
    const Word32 x   /* i: Q15 */
)
{
    Word32 acc, arg;
    Word16 man, expo, reciprocal;
    Word16 angle, w, z;

    IF (x == 0)
    {
        return 25736; /* PI/2 in Q14 */
    }
    man = ratio(y, x, &expo); /* man in Q14 */
    expo = sub(expo, (15 - 14)); /* Now, man is considered in Q15 */
    arg = L_shr((Word32)man, expo);

    IF (L_shr(arg, 3+15) != 0)
    /*===============================*
     *      8.0 <= x < infinity      *
     *===============================*/
    {
        /* atan(x) = PI/2 - 1/x + 1/(3x^3) - 1/(5x^5) + ...
         *         ~ PI/2 - 1/x, for x >= 8.
         */
        expo = norm_l(arg);
        man = extract_h(L_shl(arg, expo));
        reciprocal = div_s(0x3fff, man);
        expo = sub(15 + 1, expo);
        reciprocal = shr(reciprocal, expo);   /* Q14 */
        angle = sub(25736, reciprocal);       /* Q14   (PI/2 - 1/x) */

        /* For 8.0 <= x < 10.0, 1/(5x^5) is not completely negligible.
         * For more accurate result, add very small correction term.
         */
        if (L_sub(L_shr(arg, 15), 10L) < 0)
        {
            angle = add(angle, 8); /* Add tiny correction term. */
        }
    }
    ELSE IF (L_shr(arg, 2+15) != 0)
    /*==========================*
     *      4.0 <= x < 8.0      *
     *==========================*/
    {
        /* interval: [3.999, 8.001]
         * atan(x) ~ (((a0*x     +   a1)*x   + a2)*x   + a3)*x   + a4
         *         = (((a0*8*y   +   a1)*8*y + a2)*8*y + a3)*8*y + a4   Substitute 8*y -> x
         *         = (((a0*8^3*y + a1*8^2)*y + a2*8)*y + a3)*8*y + a4
         *         = (((    c0*y +     c1)*y +   c2)*y + c3)*8*y + c4,
         *  where y = x/8
         *   and a0 = -1.28820869667651e-04, a1 = 3.88263533346295e-03,
         *       a2 = -4.64216306484597e-02, a3 = 2.75986060068931e-01,
         *       a4 = 7.49208077809799e-01.
         */
        w = extract_l(L_shr(arg, 3));              /* Q15  y = x/8 */
        acc = L_add(533625337L, 0);                /* Q31  c1 = a1*8^2 */
        z = mac_r(acc, w, -2161);                  /* Q15  c0 = a0*8^3 */
        acc = L_add(-797517542L, 0);               /* Q31  c2 = a2*8 */
        z = mac_r(acc, w, z);                      /* Q15 */
        acc = L_add(592675551L, 0);                /* Q31  c3 = a3 */
        z = mac_r(acc, w, z);                      /* z (in:Q15, out:Q12) */
        acc = L_add(201114012L, 0);                /* Q28  c4 = a4 */
        acc = L_mac(acc, w, z);                    /* Q28 */
        angle = extract_l(L_shr(acc, (28 - 14)));  /* Q14 result of atan(x), where 4 <= x < 8 */
    }
    ELSE IF (L_shr(arg, 1+15) != 0)
    /*==========================*
     *      2.0 <= x < 4.0      *
     *==========================*/
    {
        /* interval: [1.999, 4.001]
         * atan(x) ~ (((a0*x    + a1)*x   +   a2)*x   + a3)*x   + a4
         *         = (((a0*4*y  + a1)*4*y +   a2)*4*y + a3)*4*y + a4   Substitute 4*y -> x
         *         = (((a0*16*y + a1*4)*y +   a2)*4*y + a3)*4*y + a4
         *         = (((a0*32*y + a1*8)*y + a2*2)*2*y + a3)*4*y + a4
         *         = (((   c0*y +   c1)*y +   c2)*2*y + c3)*4*y + c4,
         *  where y = x/4
         *   and a0 = -0.00262378195660943, a1 = 0.04089687039888652,
         *       a2 = -0.25631148958325911, a3 = 0.81685854627399479,
         *       a4 = 0.21358070563097167
         * */
        w = extract_l(L_shr(arg, 2));              /* Q15  y = x/4 */
        acc = L_add(702602883L, 0);                /* Q31  c1 = a1*8 */
        z = mac_r(acc, w, -2751);                  /* Q15  c0 = a0*32 */
        acc = L_add(-1100849465L, 0);              /* Q31  c2 = a2*2 */
        z = mac_r(acc, w, z);                      /* z (in:Q15, out:Q14) */
        acc = L_add(877095185L, 0);                /* Q30  c3 = a3 */
        z = mac_r(acc, w, z);                      /* z (in:Q14, out:Q12) */
        acc = L_add(57332634L, 0);                 /* Q28  c4 = a4 */
        acc = L_mac(acc, w, z);                    /* Q28 */
        angle = extract_l(L_shr(acc, (28 - 14)));  /* Q14  result of atan(x) where 2 <= x < 4 */
    }
    ELSE IF (L_shr(arg, 15) != 0)
    /*==========================*
     *      1.0 <= x < 2.0      *
     *==========================*/
    {
        /* interval: [0.999, 2.001]
         * atan(x) ~ (((a0*x   +  1)*x   + a2)*x   +   a3)*x   + a4
         *         = (((a0*2*y + a1)*2*y + a2)*2*y +   a3)*2*y + a4    Substitute 2*y -> x
         *         = (((a0*4*y + a1*2)*y + a2)*2*y +   a3)*2*y + a4
         *         = (((a0*4*y + a1*2)*y + a2)*y   + a3/2)*4*y + a4
         *         = (((  c0*y +   c1)*y + c2)*y   +   c3)*4*y + c4,
         *  where y = x/2
         *   and a0 = -0.0160706457245251, a1 = 0.1527106504065224,
         *       a2 = -0.6123208404800871, a3 = 1.3307896976322915,
         *       a4 = -0.0697089375247448
         */
        w = extract_l(L_shr(arg, 1));              /* Q15  y= x/2 */
        acc = L_add(655887249L, 0);                /* Q31  c1 = a1*2 */
        z = mac_r(acc, w, -2106);                  /* Q15  c0 = a0*4 */
        acc = L_add(-1314948992L, 0);              /* Q31  c2 = a2 */
        z = mac_r(acc, w, z);
        acc = L_add(1428924557L, 0);               /* Q31  c3 = a3/2 */
        z = mac_r(acc, w, z);                      /* z (in:Q15, out:Q13) */
        acc = L_add(-37424701L, 0);                /* Q29  c4 = a4 */
        acc = L_mac(acc, w, z);                    /* Q29 */
        angle = extract_l(L_shr(acc, (29 - 14)));  /* Q14  result of atan(x) where 1 <= x < 2 */
    }
    ELSE
    /*==========================*
     *      0.0 <= x < 1.0      *
     *==========================*/
    {
        /* interval: [-0.001, 1.001]
         * atan(x) ~ ((((a0*x   +   a1)*x   + a2)*x + a3)*x + a4)*x + a5
         *         = ((((a0*2*x + a1*2)*x/2 + a2)*x + a3)*x + a4)*x + a5
         *         = ((((  c0*x +   c1)*x/2 + c2)*x + c3)*x + c4)*x + c5
         *  where
         *    a0 = -5.41182677118661e-02, a1 = 2.76690449232515e-01,
         *    a2 = -4.63358392562492e-01, a3 = 2.87188466598566e-02,
         *    a4 =  9.97438122814383e-01, a5 = 5.36158556179092e-05.
         */
        w = extract_l(arg);                         /* Q15 */
        acc = L_add(1188376431L, 0);                /* Q31  c1 = a1*2 */
        z = mac_r(acc, w, -3547);                   /* Q15  c0 = a0*2 */
        acc = L_add(-995054571L, 0);                /* Q31  c2 = a2 */
        z = extract_h(L_mac0(acc, w, z));           /* Q15  non-fractional mode multiply */
        acc = L_add(61673254L, 0);                  /* Q31  c3 = a3 */
        z = mac_r(acc, w, z);
        acc = L_add(2141982059L, 0);                /* Q31  c4 = a4 */
        z = mac_r(acc, w, z);
        acc = L_add(115139L, 0);                    /* Q31  c5 = a5 */
        acc = L_mac(acc, w, z);                     /* Q31 */
        angle = extract_l(L_shr(acc, 31 - 14));     /* Q14  result of atan(x), where 0 <= x < 1 */
    }

    return angle;  /* Q14 between 0 and PI/2 radian. */
}



