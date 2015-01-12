/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <memory.h>
#include <assert.h>
#include "stl.h"
#include "prot_fx.h"
#include "options.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"


#define _2_ 0x4000 /*Q12*/
#define _1_ 0x2000 /*Q12*/
#define _1_Q9 0x200
/*
 * E_ACELP_h_vec_corrx
 *
 * Parameters:
 *    h              I: scaled impulse response
 *    vec            I: vector to correlate with h[]
 *    track          I: track to use
 *    sign           I: sign vector
 *    rrixix         I: correlation of h[x] with h[x]
 *    cor            O: result of correlation (16 elements)
 *
 * Function:
 *    Calculate the correlations of h[] with vec[] for the specified track
 *
 * Returns:
 *    void
 */
void E_ACELP_h_vec_corr1(Word16 h[], Word16 vec[], UWord8 track,
                         Word16 sign[], Word16 (*rrixix)[16],
                         Word16 cor[], Word16 dn2_pos[],
                         Word16 nb_pulse)
{
    Word16 i, j;
    Word16 dn, corr;
    Word16 *dn2;
    Word16 *p0, *p1, *p2;
    Word32 L_sum;

    dn2 = &dn2_pos[shl(track,3)];
    p0 = rrixix[track];

    FOR (i = 0; i < nb_pulse; i++)
    {
        dn = dn2[i];
        move16();
        L_sum = L_deposit_l(0);
        p1 = h;
        p2 = &vec[dn];
        FOR (j = dn; j < L_SUBFR-1; j++)
        L_sum = L_mac(L_sum, *p1++, *p2++);

        corr = mac_r(L_sum, *p1++, *p2++);   /*Q9*/

        /*cor[dn >> 2] = sign[dn] * s + p0[dn >> 2];*/
        j = shr(dn,2);
        if(sign[dn] > 0)
        {
            corr = add(p0[j], corr);
        }
        if(sign[dn] < 0)
        {
            corr = sub(p0[j], corr);
        }

        cor[j] = corr;
        move16();
    }
}

void E_ACELP_h_vec_corr2(Word16 h[], Word16 vec[], UWord8 track,
                         Word16 sign[], Word16 (*rrixix)[16],
                         Word16 cor[])
{
    Word16 i, j, pos, corr;
    Word16 *p0, *p1, *p2;
    Word32 L_sum;

    p0 = rrixix[track];

    pos = track;
    move16();
    FOR (i = 0; i < 16; i++)
    {
        L_sum = L_deposit_l(0);
        p1 = h;
        p2 = &vec[pos];
        FOR (j = pos; j < L_SUBFR-1; j++)
        L_sum = L_mac(L_sum, *p1++, *p2++);

        corr = mac_r(L_sum, *p1++, *p2++);   /*Q9*/

        /*cor[i] = s * sign[track] + p0[i];*/

        if(sign[pos] > 0)
        {
            corr = add(*p0++, corr);
        }
        if(sign[pos] < 0)
        {
            corr = sub(*p0++, corr);
        }
        cor[i] = corr;
        move16();

        pos = add(pos,4);
    }
}


/*
 * E_ACELP_2pulse_search
 *
 * Parameters:
 *    nb_pos_ix      I: nb of pos for pulse 1 (1..8)
 *    track_x        I: track of pulse 1
 *    track_y        I: track of pulse 2
 *    ps           I/O: correlation of all fixed pulses
 *    alp          I/O: energy of all fixed pulses
 *    ix             O: position of pulse 1
 *    iy             O: position of pulse 2
 *    dn             I: corr. between target and h[]
 *    dn2            I: vector of selected positions
 *    cor_x          I: corr. of pulse 1 with fixed pulses
 *    cor_y          I: corr. of pulse 2 with fixed pulses
 *    rrixiy         I: corr. of pulse 1 with pulse 2
 *
 * Function:
 *    Find the best positions of 2 pulses in a subframe
 *
 * Returns:
 *    void
 */
static void E_ACELP_2pulse_search(Word16 nb_pos_ix, UWord8 track_x,
                                  UWord8 track_y, Word16 *ps, Word16 *alp,
                                  Word16 *ix, Word16 *iy, Word16 dn[],
                                  Word16 *dn2, Word16 cor_x[],
                                  Word16 cor_y[], Word16 (*rrixiy)[256])
{
    Word16 x, x2, y, i, *pos_x;
    Word16 ps0, ps1, alp2_16, ps2, sq;
    Word32 alp0, alp1, alp2, s;
    Word16 *p1, *p2;
    Word16 sqk[2], alpk[2], ik;
    Word32 xy_save;
    Word16 check = 0; /* debug code not instrumented */


    /* eight dn2 max positions per track */
    /*pos_x = &dn2[track_x << 3];                      SHIFT(1); PTR_INIT(1);*/
    pos_x = &dn2[shl(track_x, 3)];
    move16();

    /* save these to limit memory searches */
    alp0 = L_deposit_h(*alp);
    ps0 = *ps;
    move16();

    alpk[0] = 1;
    move16();
    sqk[0] = -1;
    move16();
    x2 = shr(pos_x[0], 2);
    if (mac_r(L_mac(L_mac(alp0, cor_x[x2], _1_), cor_y[0], _1_), rrixiy[track_x][shl(x2,4)], _1_) < 0)
    {
        sqk[0] = 1;
        move16();
    }
    ik = 0;
    move16();
    xy_save = L_mac0(L_deposit_l(track_y), track_x, L_SUBFR);

    /* loop track 1 */
    FOR (i = 0; i < nb_pos_ix; i++)
    {
        x = pos_x[i];
        move16();
        x2 = shr(x, 2);
        /* dn[x] has only nb_pos_ix positions saved */
        /*ps1 = ps0 + dn[x];*/
        ps1 = add(ps0, dn[x]);

        /*alp1 = alp0 + cor_x[x2];*/
        alp1 = L_mac(alp0, cor_x[x2], _1_);  /*Q22*/

        p1 = cor_y;
        p2 = &rrixiy[track_x][shl(x2,4)];

        FOR (y = track_y; y < L_SUBFR; y += 4)
        {
            /*ps2 = ps1 + dn[y];*/
            ps2 = add(ps1, dn[y]);

            /*alp2 = alp1 + (*p1++) + (*p2++);*/
            alp2 = L_mac(alp1, *p1++, _1_);
            alp2_16 = mac_r(alp2, *p2++, _1_);  /*Q6*/
            alpk[1-ik] = alp2_16;
            move16();

            /*sq = ps2 * ps2;*/
            sq = mult(ps2, ps2);
            sqk[1-ik] = sq;
            move16();

            /*s = (alpk[ik] * sq) - (sqk[0] * alp2);*/
            s = L_msu(L_mult(alpk[ik], sq), sqk[ik], alp2_16); /*Q16*/

            if (s > 0)
            {
                ik = sub(1, ik);
                check = 1; /* debug code not instrumented */
            }
            if (s > 0)
            {
                xy_save = L_mac0(y, x, L_SUBFR);
            }
        }
    }

    assert(check); /* debug code not instrumented */

    ps2 = extract_l(xy_save);
    *iy = s_and(ps2, L_SUBFR-1);
    move16();
    *ix = lshr(ps2, 6);
    move16();

    /**ps = ps0 + dn[*ix] + dn[*iy];*/
    *ps = add(ps0, add(dn[*ix], dn[*iy]));
    move16();

    *alp = alpk[ik];
    move16();
}



/*
 * E_ACELP_1pulse_search
 *
 * Parameters:
 *    track_x        I: track of pulse 1
 *    track_y        I: track of pulse 2
 *    ps           I/O: correlation of all fixed pulses
 *    alp          I/O: energy of all fixed pulses
 *    ix             O: position of pulse 1
 *    dn             I: corr. between target and h[]
 *    cor_x          I: corr. of pulse 1 with fixed pulses
 *    cor_y          I: corr. of pulse 2 with fixed pulses
 *
 * Function:
 *    Find the best positions of 1 pulse in a subframe
 *
 * Returns:
 *    void
 */
static void E_ACELP_1pulse_search(UWord8 tracks[2],
                                  Word16 *ps,
                                  Word16 *alp,
                                  Word16 *ix,
                                  Word16 dn[],
                                  Word16 cor_x[],
                                  Word16 cor_y[])
{
    Word16 x, x_save = 0;
    Word16 ps0;
    Word16 ps1, sq;
    Word16 alp1;
    Word32 s, alp0;
    Word16 sqk[2], alpk[2], ik;
    Word16 ntracks, t;
    Word16 check = 0; /* debug code not instrumented */

    /* save these to limit memory searches */
    alp0 = L_deposit_h(*alp);
    ps0 = *ps;
    move16();

    alpk[0] = 1;
    move16();
    sqk[0] = -1;
    move16();
    if (mac_r(alp0, cor_x[shr(tracks[0],2)], _1_) < 0)
    {
        sqk[0] = 1;
        move16();
    }
    ik = 0;
    move16();

    ntracks = 1;
    if (sub(tracks[1], tracks[0]) != 0)
    {
        ntracks = 2;
        move16();
    }
    FOR (t=0; t<ntracks; ++t)
    {
        if (t != 0)
        {
            cor_x = cor_y;
            move16();
        }
        FOR (x = tracks[t]; x < L_SUBFR; x += 4)
        {
            /*ps1 = ps0 + dn[x];                             ADD(1);*/
            ps1 = add(ps0, dn[x]);

            /*alp1 = alp0 + cor_x[x>>2];                     SHIFT(1);ADD(1);*/
            alp1 = mac_r(alp0, cor_x[shr(x,2)], _1_);      /*Q6*/
            alpk[1-ik] = alp1;
            move16();

            /*sq = ps1 * ps1;                                MULT(1);*/
            sq = mult(ps1, ps1);
            sqk[1-ik] = sq;
            move16();

            /*s = (alpk * sq) - (sqk * alp1);                MULT(1);MAC(1); */
            s = L_msu(L_mult(alpk[ik], sq), sqk[ik], alp1);/*Q16*/

            if (s > 0)
            {
                ik = sub(1, ik);
                check = 1; /* debug code not instrumented */
            }
            if (s > 0)
            {
                x_save = x;
                move16();
            }
        }

        assert( check ); /* debug code not instrumented */
    }
    *ps = add(ps0, dn[x_save]);
    move16();
    *alp = alpk[ik];
    move16();
    *ix = x_save;
    move16();
}

/*
 * E_ACELP_xh_corr
 *
 * Parameters:
 *    h           I: impulse response (of weighted synthesis filter) (Q12)
 *    x           I: target signal (Q0)
 *    y           O: correlation between x[] and h[] <12b
 *
 * Function:
 *    Compute the correlation between the target signal and the impulse
 *    response of the weighted synthesis filter.
 *
 *           y[i]=sum(j=i,l-1) x[j]*h[j-i], i=0,l-1
 *
 *    Vector size is L_SUBFR
 *
 * Returns:
 *    void
 */
static void E_ACELP_xh_corr(Word16 *x, Word16 *y, Word16 *h, Word16 L_subfr)
{
    Word16 i, j,k;
    Word32 L_tmp, y32[L_SUBFR16k], L_maxloc, L_tot;

    assert(L_subfr <= L_SUBFR16k);

    /* first keep the result on 32 bits and find absolute maximum */
    L_tot = L_deposit_l(1);

    FOR (k = 0; k < 4; k++)
    {
        L_maxloc = L_deposit_l(0);
        FOR (i = k; i < L_subfr; i += 4)
        {
            L_tmp = L_mac0(1L, x[i], h[0]); /* 1 -> to avoid null dn[] */
            FOR (j = i; j < L_subfr-1; j++)
            {
                L_tmp = L_mac0(L_tmp, x[j+1], h[j+1 - i]);
            }

            y32[i] = L_tmp;
            move32();
            L_tmp = L_abs(L_tmp);
            L_maxloc = L_max(L_tmp, L_maxloc);
        }
        /* tot += 3*max / 8 */
        L_maxloc = L_shr(L_maxloc, 2);
        /* Do not warn saturation of L_tot, since its for headroom estimation. */
        BASOP_SATURATE_WARNING_OFF
        L_tot = L_add(L_tot, L_maxloc);           /* +max/4 */
        L_tot = L_add(L_tot, L_shr(L_maxloc, 1)); /* +max/8 */
        BASOP_SATURATE_WARNING_ON
    }

    /* Find the number of right shifts to do on y32[] so that    */
    /* 6.0 x sumation of max of dn[] in each track not saturate. */

    j = sub(norm_l(L_tot), 4+16); /* 4 -> 16 x tot */

    Copy_Scale_sig_32_16(y32, y, L_subfr, j);
}

/**
 * \brief calculate autocorrelation of vector x
 * \param x input vector 4Q11
 * \param y output (autocorrelation coefficients)
 * \param L_subfr length of x (and y)
 * \param bits amount of target headroom bits for y
 * \return exponent of y
 */
Word16 E_ACELP_hh_corr(Word16 *x, Word16 *y, Word16 L_subfr, Word16 bits)
{
    Word16 i, j, k = 0; /* initialize just to avoid compiler warning */
    Word32 L_tmp, L_sum;

    FOR (i = 0; i < L_subfr-1; i++)
    {
        L_tmp = L_mult0( x[i], x[0] );
        FOR (j = i+2; j < L_subfr; j+=2)
        {
            L_tmp = L_mac0( L_tmp, x[j], x[j-i] );
        }
        L_sum = L_shr( L_tmp, 1 );

        L_tmp = L_mult0( x[i+1], x[1] );
        FOR (j = i+3; j < L_subfr; j+=2)
        {
            L_tmp = L_mac0( L_tmp, x[j], x[j-i] );
        }
        L_sum = L_add( L_sum, L_shr( L_tmp, 1 ) );

        if (i == 0)
        {
            k = norm_l(L_sum);
        }
        if (i == 0)
        {
            k = sub(k, bits);
        }

        y[i] = round_fx( L_shl( L_sum, k ) );
    }

    L_tmp = L_mult0( x[i], x[0] );
    L_sum = L_shr( L_tmp, 1 );
    y[i] = round_fx( L_shl( L_sum, k ) );

    k = add(1, k);

    return k;
}

/*
 * E_ACELP_xy1_corr
 *
 * Parameters:
 *    xn          I: target signal
 *    y1          I: filtered adaptive codebook excitation
 *    g_coeff     O: correlations <y1,y1>  and -2<xn,y1>
 *    norm_flag   I: flag to trigger normalization of the result
 *    L_subfr     I: length of data
 *    exp_xn      I: common exponent of xn[] and y1[]
 *
 * Function:
 *    Find the correlations between the target xn[] and the filtered adaptive
 *    codebook excitation y1[]. ( <y1,y1>  and -2<xn,y1> )
 *    Subframe size = L_SUBFR
 *
 * Returns:
 *    pitch gain  (0 ... 1.2F) (Q14)
 */
Word16 E_ACELP_xy1_corr(Word16 xn[], Word16 y1[], ACELP_CbkCorr *g_corr, Word16 norm_flag, Word16 L_subfr, Word16 exp_xn)
{
    Word16 i, Q_xn;
    Word16 xy, yy, exp_xy, exp_yy, gain;
    Word32 L_off;

    L_off = L_shr(FL2WORD32(0.01f/2.0f), s_min(add(exp_xn,exp_xn), 31));
    L_off = L_max(1,L_off); /* ensure at least a '1' */

    /* Compute scalar product t1: <y1[] * y1[]> */
    yy = round_fx(Dot_product15_offs(y1, y1, L_subfr, &exp_yy, L_off));

    /* Compute scalar product t0: <xn[] * y1[]> */
    xy = round_fx(Dot_product12_offs(xn, y1, L_subfr, &exp_xy, L_off));

    /* Compute doubled format out of the exponent */
    Q_xn = shl(sub(15,exp_xn),1);
    g_corr->y1y1   = yy;
    move16();
    g_corr->y1y1_e = sub(exp_yy, Q_xn);
    move16();
    g_corr->xy1    = xy;
    move16();
    g_corr->xy1_e  = sub(exp_xy, Q_xn);
    move16();

    /* If (xy < 0) gain = 0 */
    IF (xy < 0)
    {
        move16();
        gain = 0;
        GOTO bail;
    }

    /* compute gain = xy/yy */

    xy = mult_r(xy,0x4000);                /* Be sure xy < yy */
    gain = div_s(xy, yy);

    i = add(exp_xy, 1 - 1);                /* -1 -> gain in Q14 */
    i = sub(i, exp_yy);
    BASOP_SATURATE_WARNING_OFF
    gain = shl(gain, i);                   /* saturation can occur here */
    BASOP_SATURATE_WARNING_ON
    /* gain = s_max(0, gain); */ /* see above xy < 0. */

    /* if (gain > 1.2) gain = 1.2  in Q14 */

    gain = s_min(FL2WORD16_SCALE(1.2f, 1) /* 19661 */, gain);

    /*Limit the energy of pitch contribution*/
    IF (norm_flag)
    {
        Word16 tmp, exp_tmp, exp_div;

        /* Compute scalar product <xn[],xn[]> */
        tmp = round_fx(Dot_product12_offs(xn, xn, L_subfr, &exp_tmp, 1));
        /* gain_p_snr = sqrt(<xn,xn>/<y1,y1>) */
        tmp = BASOP_Util_Divide1616_Scale(tmp, yy, &exp_div);
        exp_tmp = add(sub(exp_tmp, exp_yy), exp_div);

        tmp = Sqrt16(tmp, &exp_tmp);

        /* Note: shl works as shl or shr. */
        exp_tmp = sub(exp_tmp,1);
        BASOP_SATURATE_WARNING_OFF
        tmp = round_fx(L_shl(Mpy_32_16_1( FL2WORD32(ACELP_GAINS_CONST), tmp), exp_tmp));
        BASOP_SATURATE_WARNING_ON

        gain = s_min(gain, tmp);
    }

bail:


    return (gain);
}

/*
 * E_ACELP_xy2_corr
 *
 * Parameters:
 *    xn          I: target signal in Q_xn
 *    y1          I: filtered adaptive codebook excitation in Q_xn
 *    y2          I: filtered fixed codebook excitation in Q9
 *    g_corr      O: correlations <y2,y2>, -2<xn,y2>, 2<y1,y2>
 *    L_subfr     I: subframe size
 *
 * Function:
 *    Find the correlations between the target xn[], the filtered adaptive
 *    codebook exc. y1[], and the filtered fixed codebook innovation y2[].
 *    ( <y2,y2> , -2<xn,y2> and 2<y1,y2> )
 *    Subrame size = L_SUBFR
 *
 * Returns:
 *    pitch gain  (0 ... 1.2F)
 */
void E_ACELP_xy2_corr(Word16 xn[], Word16 y1[], Word16 y2[],
                      ACELP_CbkCorr *g_corr, Word16 L_subfr, Word16 exp_xn)
{
    Word16 xny2, y2y2, y1y2, xx, exp_xny2, exp_y2y2, exp_y1y2, exp_xx;
    Word32 L_off;

    BASOP_SATURATE_ERROR_ON;

    /* Compute scalar product <y2[],y2[]> */
    y2y2 = extract_h(Dot_product15_offs(y2, y2, L_subfr, &exp_y2y2, FL2WORD32_SCALE(0.01f, 31-19)));

    /* L_off = 1L; */
    L_off = L_shr(FL2WORD32(0.01f/2.0f), sub(30-9, exp_xn));

    /* Compute scalar product <xn[],y2[]> */
    xny2 = extract_h(Dot_product12_offs(xn, y2, L_subfr, &exp_xny2, L_off));

    /* Compute scalar product <y1[],y2[]> */
    y1y2 = extract_h(Dot_product12_offs(y1, y2, L_subfr, &exp_y1y2, L_off));

    /* Compute scalar product <xn[],xn[]> */
    L_off = L_shr(FL2WORD32(0.01f), s_min(31, sub(30, shl(exp_xn, 1))));
    xx = extract_h(Dot_product12_offs(xn, xn, L_subfr, &exp_xx, L_off));


    g_corr->y2y2   = y2y2;
    move16();
    g_corr->y2y2_e = exp_y2y2;
    move16();
    g_corr->xy2    = xny2;
    move16();
    g_corr->xy2_e  = exp_xny2;
    move16();
    g_corr->y1y2    = y1y2;
    move16();
    g_corr->y1y2_e  = exp_y1y2;
    move16();
    g_corr->xx      = xx;
    move16();
    g_corr->xx_e    = exp_xx;
    move16();


    BASOP_SATURATE_ERROR_OFF;
}



/*
 * E_ACELP_codebook_target_update
 *
 * Parameters:
 *    x           I: old target (for pitch search) (Q_xn)
 *    x2          O: new target (for codebook search) (Q_xn)
 *    y           I: filtered adaptive codebook vector (Q_xn)
 *    gain        I: adaptive codebook gain (Q14)
 *
 * Function:
 *    Update the target vector for codebook search.
 *    Subframe size = L_SUBFR
 * Returns:
 *    void
 */
void E_ACELP_codebook_target_update(Word16 *x, Word16 *x2, Word16 *y,
                                    Word16 gain, Word16 L_subfr)
{
    Word16 i, Q15_flag;
    Word32 L_tmp;

    assert(gain >= 0);

    Q15_flag = 0;
    move16();
    if (sub(gain, 1<<14) < 0)
    {
        Q15_flag = 1;
        move16();
    }
    gain = shl(gain, Q15_flag);

    FOR (i = 0; i < L_subfr; i++)
    {
        L_tmp = L_deposit_h(x[i]);
        if (Q15_flag == 0)
        {
            L_tmp = L_msu(L_tmp, y[i], gain);
        }
        x2[i] = msu_r(L_tmp, y[i], gain);
        move16();
    }
}


/*
 * E_ACELP_pulsesign
 *
 * Parameters:
 *    cn          I: residual after long term prediction   <12b
 *    dn          I: corr. between target and h[].         <12b
 *    dn2         I/O: dn2[] = mix of dn[] and cn[]
 *    sign        O: sign of pulse 0 or -1
 *    vec         O: negative sign of pulse
 *    alp         I: energy of all fixed pulses Q13
 *    sign_val    I: value for signs
 *    L_subfr     I: subframe length
 *
 * Function:
 *    Determine sign of each pulse position, store them in "sign"
 *    and change dn to all positive.
 *    Subframe size = L_SUBFR
 * Returns:
 *    void
 */
void E_ACELP_pulsesign(const Word16 cn[], Word16 dn[], Word16 dn2[], Word16 sign[], Word16 vec[], const Word16 alp, const Word16 sign_val, const Word16 L_subfr)
{
    Word16 i;
    Word32 Lval, Lcor;
    Word16 k_cn, k_dn, sign_neg, e_dn, e_cn;
    Word16 signs[3];
    Word16 *ptr16;
    Word16 val, index;


    /* calculate energy for normalization of cn[] and dn[] */
    Lval = L_mac0(1, cn[0], cn[0]);
    Lcor = L_mac0(1, dn[0], dn[0]);

    FOR (i = 1; i < L_subfr; i++)
    {
        Lval = L_mac0(Lval, cn[i], cn[i]);
        Lcor = L_mac0(Lcor, dn[i], dn[i]);
    }

    e_dn = 31;
    move16();
    e_cn = 31;
    move16();

    Lval = Sqrt32(Lval, &e_dn);
    Lcor = Sqrt32(Lcor, &e_cn);
    i = sub(e_dn,e_cn);
    if(i < 0)
        Lval = L_shl(Lval, i);
    if(i > 0)
        Lcor = L_shr(Lcor, i);

    k_dn = round_fx(Lval);
    k_cn = round_fx(Lcor);

    k_cn = mult_r(0x2000, k_cn);              /* 1 in Q13 */
    k_dn = mult_r(alp, k_dn);              /* alp in Q13 */

    sign_neg = negate(sign_val);

    signs[0] = sign_neg;
    move16();
    signs[1] = sign_val;
    move16();
    signs[2] = sign_neg;
    move16();
    ptr16 = &signs[1];

    FOR (i = 0; i < L_subfr; i++)
    {
        /*cor = (s * cn[i]) + (alp * dn[i]);                        MULT(1);MAC(1);*/
        Lcor = L_mult(cn[i], k_cn);
        Lcor = L_mac(Lcor, dn[i], k_dn);
        val = round_fx(L_shl(Lcor,4));							/*shifting by 4 may overflow but improves accuracy*/

        index = shr(val, 15);
        sign[i] = ptr16[index];
        move16();        /* yields -1 (when ps < 0) or 0 (when ps >= 0) */
        vec[i] = ptr16[index+1];
        move16();

        if (val < 0)
        {
            dn[i] = negate(dn[i]);
            move16();
        }
        dn2[i] = abs_s(val);
        move16();	/* dn2[] = mix of dn[] and cn[]            */
    }
}


void E_ACELP_findcandidates(Word16 dn2[], Word16 dn2_pos[], Word16 pos_max[])
{
    Word16 i, k, j, i8;
    Word16 *ps_ptr;

    FOR (i = 0; i < 4; i++)
    {
        i8 = shl(i, 3);
        FOR (k = i8; k < i8+8; k++)
        {
            ps_ptr = &dn2[i];

            FOR (j = i+4; j < L_SUBFR; j += 4)
            {
                if (sub(dn2[j], *ps_ptr) > 0)
                {
                    ps_ptr = &dn2[j];
                    move16();
                }
            }

            *ps_ptr = -1;           /* dn2 < 0 when position is selected */     move16();
            dn2_pos[k] = (Word16)(ps_ptr - dn2);
            move16();
        }
        pos_max[i] = dn2_pos[i8];
        move16();
    }
}


static void E_ACELP_apply_sign(Word16 *p0, Word16 *psign0)
{
    p0[0]  = mult_r( p0[0]  , psign0[ 0]);
    move16();
    p0[1]  = mult_r( p0[1]  , psign0[ 4]);
    move16();
    p0[2]  = mult_r( p0[2]  , psign0[ 8]);
    move16();
    p0[3]  = mult_r( p0[3]  , psign0[12]);
    move16();
    p0[4]  = mult_r( p0[4]  , psign0[16]);
    move16();
    p0[5]  = mult_r( p0[5]  , psign0[20]);
    move16();
    p0[6]  = mult_r( p0[6]  , psign0[24]);
    move16();
    p0[7]  = mult_r( p0[7]  , psign0[28]);
    move16();
    p0[8]  = mult_r( p0[8]  , psign0[32]);
    move16();
    p0[9]  = mult_r( p0[9]  , psign0[36]);
    move16();
    p0[10] = mult_r( p0[10] , psign0[40]);
    move16();
    p0[11] = mult_r( p0[11] , psign0[44]);
    move16();
    p0[12] = mult_r( p0[12] , psign0[48]);
    move16();
    p0[13] = mult_r( p0[13] , psign0[52]);
    move16();
    p0[14] = mult_r( p0[14] , psign0[56]);
    move16();
    p0[15] = mult_r( p0[15] , psign0[60]);
    move16();
}

void E_ACELP_vec_neg(Word16 h[], Word16 h_inv[], Word16 L_subfr)
{
    Word16 i;

    FOR(i = 0; i < L_subfr; i ++)
    {
        h_inv[i] = negate(h[i]);
        move16();
    }
}


void E_ACELP_corrmatrix(Word16 h[], Word16 sign[], Word16 vec[], Word16 rrixix[4][16], Word16 rrixiy[4][256])
{

    Word16 *p0, *p1, *p2, *p3, *psign0, *psign1, *psign2, *psign3;
    Word16 *ptr_h1, *ptr_h2, *ptr_hf;
    Word32 cor;
    Word16 i, /* j, */ k,pos;

    /*
    * Compute rrixix[][] needed for the codebook search.
    */

    /* storage order --> i3i3, i2i2, i1i1, i0i0 */

    /* Init pointers to last position of rrixix[] */
    p0 = &rrixix[0][16 - 1];                          /* Q9 */
    p1 = &rrixix[1][16 - 1];
    p2 = &rrixix[2][16 - 1];
    p3 = &rrixix[3][16 - 1];

    ptr_h1 = h;
    cor = L_deposit_l(0);
    FOR (i = 0; i < 16; i++)
    {
        cor = L_mac(cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p3-- = round_fx(L_shr(cor, 1));
        cor = L_mac(cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p2-- = round_fx(L_shr(cor, 1));
        cor = L_mac(cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p1-- = round_fx(L_shr(cor, 1));
        cor = L_mac(cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p0-- = round_fx(L_shr(cor, 1));            /* Q9 */
    }


    /*
     * Compute rrixiy[][] needed for the codebook search.
     */

    /* storage order --> i2i3, i1i2, i0i1, i3i0 */

    pos = 256 - 1;
    ptr_hf = h + 1;
    FOR (k = 0; k < 16; k++)
    {
        p3 = &rrixiy[2][pos];
        p2 = &rrixiy[1][pos];
        p1 = &rrixiy[0][pos];
        p0 = &rrixiy[3][pos - 16];

        cor = L_deposit_h(0);
        ptr_h1 = h;
        ptr_h2 = ptr_hf;

        FOR (i = k; i < 16-1; i++)
        {
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p3 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p2 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p1 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p0 = round_fx(cor);

            p3 -= (16 + 1);
            p2 -= (16 + 1);
            p1 -= (16 + 1);
            p0 -= (16 + 1);
        }
        cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
        *p3 = round_fx(cor);
        cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
        *p2 = round_fx(cor);
        cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
        *p1 = round_fx(cor);

        pos -= 16;
        ptr_hf += 4;
    }

    /* storage order --> i3i0, i2i3, i1i2, i0i1 */

    pos = 256 - 1;
    ptr_hf = h + 3;
    FOR (k = 0; k < 16; k++)
    {
        p3 = &rrixiy[3][pos];
        p2 = &rrixiy[2][pos - 1];
        p1 = &rrixiy[1][pos - 1];
        p0 = &rrixiy[0][pos - 1];

        cor = L_deposit_h(0);
        ptr_h1 = h;
        ptr_h2 = ptr_hf;
        FOR (i = k; i < 16-1; i++)
        {
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p3 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p2 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p1 = round_fx(cor);
            cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
            *p0 = round_fx(cor);

            p3 -= (16 + 1);
            p2 -= (16 + 1);
            p1 -= (16 + 1);
            p0 -= (16 + 1);
        }
        cor = L_mac(cor, *ptr_h1++, *ptr_h2++);
        *p3 = round_fx(cor);

        pos--;
        ptr_hf += 4;
    }

    /*
     * Modification of rrixiy[][] to take signs into account.
     */

    p0 = &rrixiy[0][0];

    /* speed-up: 11% */
    p1 = &rrixiy[1][0];
    p2 = &rrixiy[2][0];
    p3 = &rrixiy[3][0];

    FOR(i = 0; i < L_SUBFR; i += 4)
    {

        psign0 = &vec[1];
        if (sign[i+0] > 0 )  psign0 = &sign[1];

        psign1 = &vec[2];
        if (sign[i+1] > 0 )  psign1 = &sign[2];

        psign2 = &vec[3];
        if (sign[i+2] > 0 )  psign2 = &sign[3];

        psign3 = &vec[0];
        if (sign[i+3] > 0 )  psign3 = &sign[0];

        E_ACELP_apply_sign(p0, psign0);
        p0 += 16;

        E_ACELP_apply_sign(p1, psign1);
        p1 += 16;

        E_ACELP_apply_sign(p2, psign2);
        p2 += 16;

        E_ACELP_apply_sign(p3, psign3);
        p3 += 16;
    }
}

void E_ACELP_4tsearch(Word16 dn[], const Word16 cn[], const Word16 H[], Word16 code[],
                      const PulseConfig *config, Word16 ind[], Word16 y[])
{
    Word16 sign[L_SUBFR], vec[L_SUBFR];
    Word16 cor_x[16], cor_y[16], h_buf[4 * L_SUBFR];
    Word16 rrixix[4][16];
    Word16 rrixiy[4][256];
    Word16 dn2[L_SUBFR];
    Word16 psk, ps, alpk, alp = 0;
    Word16 codvec[NB_PULSE_MAX];
    Word16 pos_max[4];
    Word16 dn2_pos[8 * 4];
    UWord8 ipos[NB_PULSE_MAX];
    Word16 *p0, *p1, *p2, *p3;
    Word16 *h, *h_inv;
    Word16 i, j, k, l, st, pos;
    Word16 val, tmp, scale;
    Word32 s, L_tmp;
    Word16 nb_pulse, nb_pulse_m2;
    Word16 check = 0; /* debug code not instrumented */


    alp = config->alp; /* Q13 */ /* initial value for energy of all fixed pulses */ move16();
    nb_pulse = config->nb_pulse;
    move16();
    nb_pulse_m2 = sub(nb_pulse, 2);

    set16_fx(codvec, 0, nb_pulse);

    /*
     * Find sign for each pulse position.
     */

    E_ACELP_pulsesign(cn, dn, dn2, sign, vec, alp, 0x7fff, L_SUBFR);

    /*
     * Select the most important 8 position per track according to dn2[].
     */
    E_ACELP_findcandidates(dn2, dn2_pos, pos_max);

    /*
     * Compute h_inv[i].
     */
    set16_fx(h_buf, 0, L_SUBFR);

    set16_fx(h_buf + (2 * L_SUBFR),0,L_SUBFR);

    h = h_buf + L_SUBFR;
    h_inv = h_buf + (3 * L_SUBFR);

    /*Check the energy if it is too high then scale to prevent an overflow*/
    scale = 0;
    move16();
    L_tmp = L_deposit_l(0);
    BASOP_SATURATE_WARNING_OFF
    FOR (i = 0; i < L_SUBFR; i++)
    {
        L_tmp = L_mac(L_tmp, H[i], H[i]);
    }
    val = extract_h(L_tmp);
    BASOP_SATURATE_WARNING_ON

    if (sub(val, 0x2000) > 0)
    {
        scale = -1;
        move16();
    }
    if (sub(val, 0x7000) > 0)
    {
        scale = -2;
        move16();
    }

    Copy_Scale_sig(H, h, L_SUBFR, scale);

    E_ACELP_vec_neg(h, h_inv, L_SUBFR);


    /*
     * Compute correlation matrices needed for the codebook search.
     */
    E_ACELP_corrmatrix(h, sign, vec, rrixix, rrixiy);


    /*
     * Deep first search:
     * ------------------
     * 20 bits (4p):  4 iter x ((4x16)+(8x16))              = 768 tests
     * 36 bits (8p):  4 iter x ((1x1)+(4x16)+(8x16)+(8x16)) = 1280 tests
     * 52 bits (12p): 3 iter x ((1x1)+(1x1)+(4x16)+(6x16)
     *                                      +(8x16)+(8x16)) = 1248 tests
     * 64 bits (16p): 2 iter x ((1x1)+(1x1)+(4x16)+(6x16)
     *                        +(6x16)+(8x16)+(8x16)+(8x16)) = 1280 tests
     */
    psk = -1;
    move16();
    alpk = 1;
    move16();

    /*Number of iterations*/
    FOR (k = 0; k < config->nbiter; k++)
    {
        E_ACELP_setup_pulse_search_pos(config, k, ipos);

        /* format of alp changes to Q(15-ALP2_E) */

        pos = config->fixedpulses;
        move16();

        IF (config->fixedpulses == 0) /* 1100, 11, 1110, 1111, 2211 */
        {
            ps = 0;
            move16();
            alp = 0;
            move16();
            set16_fx(vec, 0, L_SUBFR);
        }
        ELSE IF (sub(config->fixedpulses, 2) == 0) /* 2222 and 3322 */
        {
            /* first stage: fix 2 pulses */
            ind[0] = pos_max[ipos[0]];
            move16();
            ind[1] = pos_max[ipos[1]];
            move16();
            ps = add(dn[ind[0]], dn[ind[1]]);

            /*alp = rrixix[ipos[0]][ind[0] >> 2] + rrixix[ipos[1]][ind[1] >> 2] +
                    rrixiy[ipos[0]][((ind[0] >> 2) << 4) + (ind[1] >> 2)];*/

            i = shr(ind[0], 2);
            j = shr(ind[1], 2);
            l = add(shl(i, 4), j);
            s = L_mult(rrixix[ipos[0]][i], _1_);             /* Q9+Q12+1 */
            s = L_mac(s, rrixix[ipos[1]][j], _1_);
            alp = mac_r(s, rrixiy[ipos[0]][l], _1_);

            p0 = h - ind[0];
            if (sign[ind[0]] < 0)
            {
                p0 = h_inv - ind[0];
            }

            p1 = h - ind[1];
            if (sign[ind[1]] < 0)
            {
                p1 = h_inv - ind[1];
            }

            FOR (i = 0; i < L_SUBFR; i++)
            {
                vec[i] = add(*p0++, *p1++);
                move16();
            }
        }
        ELSE /* 3333 and above */
        {
            /* first stage: fix 4 pulses */

            ind[0] = pos_max[ipos[0]];
            move16();
            ind[1] = pos_max[ipos[1]];
            move16();
            ind[2] = pos_max[ipos[2]];
            move16();
            ind[3] = pos_max[ipos[3]];
            move16();

            /*ps = dn[ind[0]] + dn[ind[1]] + dn[ind[2]] + dn[ind[3]];*/
            ps = add(add(add(dn[ind[0]], dn[ind[1]]), dn[ind[2]]), dn[ind[3]]);

            p0 = h - ind[0];
            if (sign[ind[0]] < 0)
            {
                p0 = h_inv - ind[0];
            }

            p1 = h - ind[1];
            if (sign[ind[1]] < 0)
            {
                p1 = h_inv - ind[1];
            }

            p2 = h - ind[2];
            if (sign[ind[2]] < 0)
            {
                p2 = h_inv - ind[2];
            }

            p3 = h - ind[3];
            if (sign[ind[3]] < 0)
            {
                p3 = h_inv - ind[3];
            }

            FOR (i = 0; i < L_SUBFR; i++)
            {
                vec[i] = add(add(add(*p0++, *p1++), *p2++), *p3++);
                move16();
            }

            L_tmp = L_mult(vec[0], vec[0]);
            FOR (i = 1; i < L_SUBFR; i++)
            L_tmp = L_mac(L_tmp, vec[i], vec[i]);

            alp = round_fx(L_shr(L_tmp, 3));

            /*alp *= 0.5F;      */
        }

        /* other stages of 2 pulses */
        st = 0;
        move16();
        FOR (j = pos; j < nb_pulse; j += 2)
        {
            IF (sub(nb_pulse_m2, j) >= 0) /* pair-wise search */
            {
                /*
                 * Calculate correlation of all possible positions
                 * of the next 2 pulses with previous fixed pulses.
                 * Each pulse can have 16 possible positions.
                 */
                E_ACELP_h_vec_corr1(h, vec, ipos[j], sign, rrixix, cor_x, dn2_pos, config->nbpos[st]);

                E_ACELP_h_vec_corr2(h, vec, ipos[j + 1], sign, rrixix, cor_y);

                /*
                 * Find best positions of 2 pulses.
                 */
                E_ACELP_2pulse_search(config->nbpos[st], ipos[j], ipos[j + 1], &ps, &alp,
                                      &ind[j], &ind[j+1], dn, dn2_pos, cor_x, cor_y, rrixiy);

            }
            ELSE /* single pulse search */
            {
                E_ACELP_h_vec_corr2(h, vec, ipos[j], sign, rrixix, cor_x);

                E_ACELP_h_vec_corr2(h, vec, ipos[j + 1], sign, rrixix, cor_y);

                E_ACELP_1pulse_search(&ipos[j], &ps, &alp,
                &ind[j], dn, cor_x, cor_y);
            }

            IF (0 < sub(nb_pulse_m2, j))
            {
                p0 = h - ind[j];
                if (sign[ind[j]] < 0)
                {
                    p0 = h_inv - ind[j];
                }

                p1 = h - ind[j + 1];
                if (sign[ind[j + 1]] < 0)
                {
                    p1 = h_inv - ind[j + 1];
                }


                FOR (i = 0; i < L_SUBFR; i++)
                {
                    tmp = add(*p0++, *p1++);
                    vec[i] = add(vec[i], tmp); /* can saturate here. */         move16();
                }

            }
            st = add(st, 1);
        }

        /* memorise the best codevector */

        /*ps = ps * ps;                                            MULT(1);*/
        ps = mult(ps, ps);
        /*s = (alpk * ps) - (psk * alp);                            MULT(2);ADD(1);*/
        s = L_msu(L_mult(alpk, ps), psk, alp);    /*Q9+Q6+1=Q16*/

        if (psk < 0)
        {
            s = 1;
        }
        IF (s > 0)
        {
            psk = ps;
            move16();
            alpk = alp;
            move16();
            Copy(ind,codvec, nb_pulse);
            check = 1; /* debug code not instrumented */
        }
    }

    assert( check ); /* debug code not instrumented */

    /*
     * Build the codeword, the filtered codeword and index of codevector, as well as store weighted correlations.
     */

    E_ACELP_build_code(nb_pulse, codvec, sign, code, ind);

    set16_fx(y, 0, L_SUBFR);
    FOR (k = 0; k<nb_pulse; ++k)
    {
        i = codvec[k];
        move16();
        p0 = h_inv - i;
        if (sign[i] > 0)
        {
            p0 -= 2*L_SUBFR;
        }
        FOR (i=0; i<L_SUBFR; i++)
        {
            y[i] = add(y[i], *p0++);
            move16();
        }
    }
}




/*
 * E_ACELP_4t
 *
 * Parameters:
 *    dn          I: corr. between target and h[].
 *    cn          I: residual after Word32 term prediction
 *    H           I: impulse response of weighted synthesis filter (Q12)
 *    code        O: algebraic (fixed) codebook excitation (Q9)
 *    y           O: filtered fixed codebook excitation (Q9)
 *    nbbits      I: 20, 36, 44, 52, 64, 72 or 88 bits
 *    mode        I: speech mode
 *    _index      O: index
 *
 * Function:
 *    20, 36, 44, 52, 64, 72, 88 bits algebraic codebook.
 *    4 tracks x 16 positions per track = 64 samples.
 *
 *    20 bits 5 + 5 + 5 + 5 --> 4 pulses in a frame of 64 samples.
 *    36 bits 9 + 9 + 9 + 9 --> 8 pulses in a frame of 64 samples.
 *    44 bits 13 + 9 + 13 + 9 --> 10 pulses in a frame of 64 samples.
 *    52 bits 13 + 13 + 13 + 13 --> 12 pulses in a frame of 64 samples.
 *    64 bits 2 + 2 + 2 + 2 + 14 + 14 + 14 + 14 -->
 *                                  16 pulses in a frame of 64 samples.
 *    72 bits 10 + 2 + 10 + 2 + 10 + 14 + 10 + 14 -->
 *                                  18 pulses in a frame of 64 samples.
 *    88 bits 11 + 11 + 11 + 11 + 11 + 11 + 11 + 11 -->
 *                                  24 pulses in a frame of 64 samples.
 *
 *    All pulses can have two (2) possible amplitudes: +1 or -1.
 *    Each pulse can sixteen (16) possible positions.
 *
 * Returns:
 *    void
 */
static
void E_ACELP_4t(
    Word16 dn[], Word16 cn[] /* Q_xn */, Word16 H[],
    Word16 R[], Word8 acelpautoc,
    Word16 code[],
    Word16 cdk_index, Word16 _index[])
{
    const PulseConfig *config;
    Word16 ind[NPMAXPT*4];
    Word16 y[L_SUBFR];

    config = &PulseConfTable[cdk_index];
    move16();

    IF (acelpautoc)
    {
        E_ACELP_4tsearchx(dn, cn, R, code, config, ind);
    }
    ELSE
    {
        E_ACELP_4tsearch(dn, cn, H, code, config, ind, y);
    }
    E_ACELP_indexing(code, config, NB_TRACK_FCB_4T, _index);
}

static void E_ACELP_indexing_shift(
    Word16 wordcnt,      /* i: 16-bit word count including the newly shifted-in bits */
    Word16 shift_bits,   /* i: number of bits to shift in from the lsb               */
    UWord16 lsb_bits,    /* i: bits to shift in from the lsb                         */
    const UWord16 src[], /* i: source buffer                                         */
    UWord16 dst[]        /* o: destination buffer                                    */
)
{
    Word16 right_shift, i;

    assert(shift_bits <= 16);

    right_shift = sub(16, shift_bits);

    FOR (i=sub(wordcnt, 1); i>0; --i)
    {
        dst[i] = s_or(lshl(src[i], shift_bits), lshr(src[i-1], right_shift));
        move16();
    }
    dst[i] = s_or(lshl(src[i], shift_bits), lsb_bits);
    move16();
}

#define MAX_IDX_LEN 9

Word16 E_ACELP_indexing(
    const Word16 code[],
    const PulseConfig *config,
    Word16 num_tracks,
    Word16 prm[]
)
{
    Word16 track, shift_bits;
    Word16 p[NB_TRACK_FCB_4T], wordcnt;
    UWord32 s[NB_TRACK_FCB_4T], n[NB_TRACK_FCB_4T];
    UWord16 idx[MAX_IDX_LEN];
    Word16 saved_bits;

    saved_bits = 0;
    move16();

    /*
     * Code state of pulses of all tracks
     * */
    wordcnt = shr(add(config->bits, 15), 4);     /* ceil(bits/16) */

    set16_fx((Word16*)idx, 0, wordcnt);

    IF (sub(config->bits, 43) == 0)    /* EVS pulse indexing */
    {
        saved_bits = E_ACELP_code43bit(code, s, p, idx);
    }
    ELSE
    {
        FOR (track = 0; track < num_tracks; track++)
        {
            /* Code track of length 2^4 where step between tracks is 4. */
            E_ACELP_codearithp(code+track, &n[track], &s[track], &p[track]);
        }
        fcb_pulse_track_joint(idx, wordcnt, s, p, num_tracks);
    }

    /* check if we need to code track positions */
    track = 0;
    move16();
    shift_bits = 0;
    move16();
    SWITCH (config->codetrackpos)
    {
    case TRACKPOS_FIXED_TWO:
        /* Code position of consecutive tracks with single extra pulses */
        /* Find track with one pulse less. */
        if (sub(p[0], p[1]) != 0)
        {
            /* Either 0110 or 1001 */
            track = 1;
            move16();
        }
        if (sub(p[3], p[1]) > 0)
        {
            track = add(track, 2);
        }
        shift_bits = 2;
        move16();
        BREAK;

    case TRACKPOS_FIXED_FIRST_TWO:
        /* Code position of consecutive tracks with single extra pulses */
        /* Find track with one pulse less. */
        if (sub(p[0], p[1]) <= 0)
        {
            track = 1;
            move16();
        }
        shift_bits = 1;
        move16();
        BREAK;

    case TRACKPOS_FREE_THREE:
        /* Code position of track with one pulse less than others */
        /* Find track with one pulse less. */
        if (sub(p[1], p[0]) < 0)
        {
            track = 1;
            move16();
        }
        if (sub(p[2], p[0]) < 0)
        {
            track = 2;
            move16();
        }
        if (sub(p[3], p[0]) < 0)
        {
            track = 3;
            move16();
        }
        shift_bits = 2;
        move16();
        BREAK;

    case TRACKPOS_FREE_ONE:
        /* Code position of track with one pulse less than others */
        /* Find track with one pulse less. */
        if (sub(p[1], p[0]) > 0)
        {
            track = 1;
            move16();
        }
        if (sub(p[2], p[0]) > 0)
        {
            track = 2;
            move16();
        }
        if (sub(p[3], p[0]) > 0)
        {
            track = 3;
            move16();
        }
        shift_bits = 2;
        move16();
        BREAK;

    case TRACKPOS_FIXED_EVEN:
    case TRACKPOS_FIXED_FIRST:
        BREAK;

    default:
        printf("Codebook mode not implemented.\n");
        assert(0);   /* mode not yet implemented*/
        BREAK;
    }

    E_ACELP_indexing_shift(wordcnt, shift_bits, track, idx, (UWord16*)prm);

    return saved_bits;
}

/*--------------------------------------------------------------------------*
 * E_ACELP_adaptive_codebook
 *
 * Find adaptive codebook.
 *--------------------------------------------------------------------------*/
void E_ACELP_adaptive_codebook(
    Word16 *exc,          /* i/o: pointer to the excitation frame            Q_new */
    Word16 T0,            /* i  : integer pitch lag                             Q0 */
    Word16 T0_frac,       /* i  : fraction of lag                                  */
    Word16 T0_res,        /* i  : pitch resolution                                 */
    Word16 T0_res_max,    /* i  : maximum pitch resolution                         */
    Word16 mode,          /* i  : filtering mode (0: no, 1: yes, 2: adaptive)      */
    Word16 i_subfr,       /* i  : subframe index                                   */
    Word16 L_subfr,       /* i  : subframe length                                  */
    Word16 L_frame,       /* i  : subframe length                                  */
    Word16 *h1,           /* i  : impulse response of weighted synthesis filter 1Q14+shift */
    Word16 clip_gain,     /* i  : flag to indicate ???                             */
    Word16 *xn,           /* i  : Close-loop Pitch search target vector       Q_xn */
    Word16 *y1,           /* o  : zero-memory filtered adaptive excitation    Q_xn */
    ACELP_CbkCorr *g_corr,/* o  : ACELP correlation values                         */
    Word16 **pt_indice,   /* i/o: quantization indices pointer                     */
    Word16 *pitch_gain,   /* o  : adaptive codebook gain                      1Q14 */
    Word16 exp_xn         /* i  : exponent of xn (Q_xn-15)                         */
    ,Word16 rf_mode
    ,Word16 rf_coder_type
    ,Word16* lp_select

)
{
    Word16 y2[L_SUBFR], xn2[L_SUBFR], code[L_SUBFR];
    ACELP_CbkCorr g_corr2;
    Word16 gain1 = 0, gain2 = 0, fac_m, fac_n;
    Word16 i, select, exp_ener;
    Word32 L_tmp, L_ener;
    const Word16 *pitch_inter;
    Word16 pit_L_interpol, pit_up_samp;
    Word16 use_prev_sf_pit_gain = 0;

    if( rf_mode == 1 && rf_coder_type == 100)
    {
        use_prev_sf_pit_gain = 1;
    }

    BASOP_SATURATE_ERROR_ON;

    L_ener = L_deposit_l(0);

    /* find pitch excitation */
    /*for &exc[i_subfr]*/
    if (sub(T0_res, shr(T0_res_max, 1)) == 0)
    {
        T0_frac = shl(T0_frac, 1);
    }

    IF (sub(T0_res_max, 6) == 0 && rf_mode == 0)
    {
        pitch_inter = pitch_inter6_2;
        pit_L_interpol = PIT_L_INTERPOL6_2;
        move16();
        pit_up_samp = PIT_UP_SAMP6;
        move16();
    }
    ELSE
    {
        pitch_inter = pitch_inter4_2;
        pit_L_interpol = L_INTERPOL2;
        move16();
        pit_up_samp = PIT_UP_SAMP;
        move16();
    }

    pred_lt4( &exc[i_subfr], &exc[i_subfr], T0, T0_frac, L_SUBFR+1, pitch_inter, pit_L_interpol, pit_up_samp);

    test();
    IF(sub(mode,NORMAL_OPERATION)==0 || (sub(mode,FULL_BAND)==0))
    {
        E_UTIL_f_convolve(&exc[i_subfr], h1, y1,L_subfr);

        IF(use_prev_sf_pit_gain == 0)
        {
            gain1 = E_ACELP_xy1_corr(xn, y1, g_corr,1,L_subfr,exp_xn);

            /* clip gain if necessary to avoid problem at decoder */ test();
            if (clip_gain && sub(gain1,FL2WORD16_SCALE(0.95,1)) > 0)
            {
                gain1 = FL2WORD16_SCALE(0.95f,1);
                move16();
            }
            *pitch_gain = gain1;
            move16();
        }

        /* find energy of new target xn2[] */
        E_ACELP_codebook_target_update(xn, xn2, y1, gain1, L_subfr);
        L_ener = Dot_product12_offs(xn2, xn2, L_subfr, &exp_ener, 0);
        L_ener = L_shr(L_ener,sub(31,exp_ener));
    }

    /*-----------------------------------------------------------------*
     * - find pitch excitation filtered by 1st order LP filter.        *
     * - find filtered pitch exc. y2[]=exc[] convolved with h1[])      *
     * - compute pitch gain2                                           *
     *-----------------------------------------------------------------*/ test();
    IF(sub(mode,NORMAL_OPERATION)==0 || sub(mode,LOW_PASS)==0)
    {
        /* find pitch excitation with lp filter */
        fac_m = FL2WORD16(0.64f);
        move16();
        if ( sub(L_frame,L_FRAME16k)==0 )
        {
            fac_m = FL2WORD16(0.58f);
            move16();
        }
        /* fac_n = 0.5*(1.0-fac_m); */
        fac_n = mult_r(sub(0x7FFF,fac_m),0x4000);
        FOR (i=0; i<L_subfr; i++)
        {
            L_tmp   = L_mult(      fac_n, exc[i-1+i_subfr]);
            L_tmp   = L_mac(L_tmp, fac_m, exc[i+0+i_subfr]);
            code[i] = mac_r(L_tmp, fac_n, exc[i+1+i_subfr]);
            move16();
        }
        E_UTIL_f_convolve(code, h1, y2,L_subfr);
        gain2 = E_ACELP_xy1_corr(xn, y2, &g_corr2, 1, L_subfr, exp_xn);

        /* clip gain if necessary to avoid problem at decoder */  test();
        if (clip_gain && sub(gain2,FL2WORD16_SCALE(0.95,1)) > 0)
        {
            gain2 = FL2WORD16_SCALE(0.95f,1);
            move16();
        }

        /* find energy of new target xn2[] */
        E_ACELP_codebook_target_update(xn, xn2, y2, gain2, L_subfr);
        L_tmp = Dot_product12_offs(xn2, xn2, L_subfr, &exp_ener,0);
        L_tmp = L_shr(L_tmp,sub(31,exp_ener));

        /*-----------------------------------------------------------------*
         * use the best prediction (minimise quadratic error).             *
         *-----------------------------------------------------------------*/ test();
        IF (sub(mode,LOW_PASS)==0 || L_sub(L_tmp,L_ener) < 0)
        {
            /* use the lp filter for pitch excitation prediction */
            select = LOW_PASS;
            move16();
            Copy(code, &exc[i_subfr], L_subfr);
            Copy(y2, y1, L_subfr);
            *pitch_gain = gain2;
            move16();
            g_corr->y1y1 = g_corr2.y1y1;
            move16();
            g_corr->xy1 = g_corr2.xy1;
            move16();
            g_corr->y1y1_e = g_corr2.y1y1_e;
            move16();
            g_corr->xy1_e = g_corr2.xy1_e;
            move16();
        }
        ELSE
        {
            /* no filter used for pitch excitation prediction */
            select = FULL_BAND;
            move16();
            *pitch_gain = gain1;
            move16();
        }

        IF(sub(mode,NORMAL_OPERATION)==0)
        {
            **pt_indice = select;
            (*pt_indice)++;
            move16();
        }
    }
    ELSE
    {
        /* no filter used for pitch excitation prediction */
        select = FULL_BAND;
        move16();
    }
    *lp_select=select;

    BASOP_SATURATE_ERROR_OFF;
}


/*--------------------------------------------------------------------------*
 * E_ACELP_innovative_codebook
 *
 * Find innovative codebook.
 *--------------------------------------------------------------------------*/
void E_ACELP_innovative_codebook(
    Word16 *exc,        /* i  : pointer to the excitation frame                Q_new  */
    Word16 T0,          /* i  : integer pitch lag                                 Q0 */
    Word16 T0_frac,     /* i  : fraction of lag                                   Q0 */
    Word16 T0_res,      /* i  : pitch resolution                                  Q0 */
    Word16 gain_pit,    /* i  : adaptive codebook gain                          1Q14 */
    Word16 tilt_code,   /* i  : tilt factor                                      Q15 */
    Word16 mode,        /* i  : innovative codebook mode                          Q0 */
    Word16 formant_enh, /* i  : use formant enhancement                           Q0 */
    Word16 formant_tilt, /* i  : use tilt of formant enhancement                  Q0 */
    const Word16 formant_enh_num, /* i  : formant sharpening numerator weighting           */
    const Word16 formant_enh_den, /* i  : formant sharpening denominator weighting         */
    Word16 pitch_sharpening, /* i  : use pitch sharpening                         Q0 */
    Word16 pre_emphasis,
    Word16 phase_scrambling,
    Word16 i_subfr,     /* i  : subframe index                                       */
    const Word16 *Aq,         /* i  : quantized LPC coefficients                      3Q12 */
    Word16 *h1,         /* i  : impulse response of weighted synthesis filter   1Q14+shift */
    Word16 *xn,         /* i  : Close-loop Pitch search target vector           Q_xn  */
    Word16 *cn,         /* i  : Innovative codebook search target vector        Q_new */
    Word16 *y1,         /* i  : zero-memory filtered adaptive excitation         Q_xn */
    Word16 *y2,         /* o  : zero-memory filtered algebraic excitation          Q9 */
    Word8  acelpautoc,  /* i  : autocorrelation mode enabled                          */
    Word16 **pt_indice, /* i/o: quantization indices pointer                          */
    Word16 *code,       /* o  : innovative codebook Q9                             Q9 */
    Word16 shift        /* i  : Scaling to get 12 bits                                */
)
{
    Word16 xn2[L_SUBFR] /* Q_xn */, cn2[L_SUBFR] /* Q_xn */, dn[L_SUBFR] /* Rw2*cn2 */, h2[L_SUBFR] /* 4Q11 */;
    Word16 Rw2[L_SUBFR];
    Word16 pitch;


    pitch = T0;
    move16();
    if (sub(T0_frac, shr(T0_res, 1)) > 0)
    {
        pitch = add(pitch,1);
    }

    BASOP_SATURATE_ERROR_ON;

    /* Update target vector for ACELP codebook search */
    E_ACELP_codebook_target_update(xn, xn2, y1, gain_pit, L_SUBFR);

    /* Include fixed-gain pitch contribution into impulse resp. h1[] */
    Copy_Scale_sig(h1, h2, L_SUBFR, sub(-3, shift)); /*h2 1Q14+shift -> 4Q11, 1bit of headroom for Residu and xh_corr*/

    E_UTIL_cb_shape(pre_emphasis, pitch_sharpening, phase_scrambling, formant_enh, formant_tilt,
                    formant_enh_num, formant_enh_den, Aq, h2, tilt_code, pitch);

    /* Correlation between target xn2[] and impulse response h1[] */
    IF (acelpautoc)
    {
        /* h2: 4Q11, Rw2: (Rw2_e)Q */
        /* Rw2_e = */ E_ACELP_hh_corr(h2, Rw2, L_SUBFR, 3);

        E_ACELP_conv(xn2, h2, cn2);

        /* dn_e -> Rw2_e*Q_xn */
        /*dn_e = */ E_ACELP_toeplitz_mul(Rw2,cn2,dn,L_SUBFR,sub((Word16)PulseConfTable[mode].nb_pulse, 24) > 0);

    }
    ELSE
    {
        BASOP_SATURATE_WARNING_OFF;
        E_ACELP_codebook_target_update(cn, cn2, &exc[i_subfr], gain_pit, L_SUBFR);
        BASOP_SATURATE_WARNING_ON;
        Scale_sig(cn2, L_SUBFR, shift);
        E_ACELP_xh_corr(xn2, dn, h2, L_SUBFR);
    }

    /* Innovative codebook search */
    assert(mode < ACELP_FIXED_CDK_NB);
    E_ACELP_4t(dn, cn2, h2, Rw2, acelpautoc, code,
               mode, *pt_indice);
    *pt_indice += 8;

    /* Generate weighted code */
    E_ACELP_weighted_code(code, h2, 11, y2);

    /*-------------------------------------------------------*
     * - Add the fixed-gain pitch contribution to code[].    *
     *-------------------------------------------------------*/

    E_UTIL_cb_shape(pre_emphasis, pitch_sharpening, phase_scrambling, formant_enh, formant_tilt,
                    formant_enh_num, formant_enh_den, Aq, code, tilt_code, pitch);

    BASOP_SATURATE_ERROR_OFF;
}



/*--------------------------------------------------------------------------*
 * E_ACELP_codearithp
 *
 * Fixed bit-length arithmetic coding of pulses
 * v - (input) pulse vector
 * ps - (output) encoded state
 * n - (output) range of possible states (0...n-1)
 * p - (output) number of pulses found
 *--------------------------------------------------------------------------*/
void E_ACELP_codearithp(const Word16 v[], UWord32 *n, UWord32 *ps, Word16 *p)
{
    Word16 k, nb_pulse, i, t, pos[NPMAXPT], posno;
    Word16 sign, m;
    UWord32 s;

    /* Collect different pulse positions to pos[], number of them to posno */
    posno = 0;
    move16();
    t = 0;
    move16();
    FOR (k=0; k<L_SUBFR; k+=4)
    {
        if (v[k] != 0)
        {
            pos[posno++] = t;
            move16();
        }
        t = add(t, 1);
    }

    /* Iterate over the different pulse positions */
    s = L_deposit_l(0);
    t = 0;
    move16();
    nb_pulse = 0;
    move16();
    FOR (k=0; k<posno; ++k)
    {
        sign = shr(v[shl(pos[k], 2)], 9); /* sign with multiplicity */
        m = abs_s(sign); /* multiplicity */
        nb_pulse = add(nb_pulse, m);
        /* Code m-1 pulses */
        FOR (i=1; i<m; ++i)
        {
            Carry = 0;
            s = L_add_c(s, pulsestostates[pos[k]][t]);
            t = add(t, 1);
        }

        /* Code sign */
        /* We use L_add_c since we want to work with unsigned UWord32 */
        /* Therefore, we have to clear carry */
        Carry = 0;
        s = L_lshl(s, 1);
        if (sign < 0)
        {
            s = L_add_c(s, 1);
        }

        /* Code last pulse */
        Carry = 0;
        s = L_add_c(s, pulsestostates[pos[k]][t]);
        t = add(t, 1);
    }

    *ps = s;
    move32();
    *n = L_deposit_l(0);
    if (nb_pulse)
    {
        *n = pulsestostates[NB_POS_FCB_4T][nb_pulse-1];
        move32();
    }

    *p = nb_pulse;
    move16();
}

void fcb_pulse_track_joint(UWord16 *idxs, Word16 wordcnt, UWord32 *index_n, Word16 *pulse_num, Word16 track_num)
{
    static const Word16 hi_to_low_tmpl[10] = { 0, 0, 0, 3, 9, 5, 3, 1, 8, 8};
    static const Word16 low_len[10]   = { 0, 0, 8, 5, 7, 11, 13, 15, 16, 16};
    static const Word32 low_mask[10] = { 0, 0, 255, 31,127, 2047, 8191, 32767, 65535, 65535};
    static const Word16 indx_fact[10] = { 0, 0, 2, 172, 345, 140, 190, 223, 463, 1732};
    static const Word16 index_len[3] = {0, 5, 9};
    Word16 hi_to_low[10];
    UWord32 index, index_mask;
    Word32 indx_tmp;
    Word16 indx_flag, indx_flag_1;
    Word16 track, track_num1, pulse_num0, pulse_num1;
    Word16 indx_flag_2;

    Copy(hi_to_low_tmpl, hi_to_low, 10);

    indx_flag = 0;
    move16();
    indx_flag_1 = 0;
    move16();
    indx_flag_2 = 0;
    move16();

    FOR (track = 0; track < track_num; track++)
    {
        indx_flag = add(indx_flag, shr(pulse_num[track], 2));
        indx_flag_1 = add(indx_flag_1, shr(pulse_num[track], 1));
        indx_flag_2 = add(indx_flag_2, shr(pulse_num[track], 3));
    }

    IF (sub(indx_flag_2, 1) >= 0)
    {
        hi_to_low[7] = 9;
        move16();
        index_mask = 0xFFFFFF;
        move32();
    }
    ELSE
    {
        if (sub(indx_flag, track_num) < 0)
        {
            hi_to_low[4] = 1;
            move16();
        }
        index_mask = L_shr(0xFFFF, sub(9, hi_to_low[4]));
    }

    IF (sub(indx_flag_1, track_num) >= 0)
    {
        indx_tmp = L_deposit_l(0);
        index = L_shr(index_n[0], low_len[pulse_num[0]]);
        FOR (track = 1; track < track_num; track++)
        {
            pulse_num0 = pulse_num[track - 1];
            move16();
            pulse_num1 = pulse_num[track];
            move16();
            indx_tmp = L_lshr(index_n[track], low_len[pulse_num1]);
            /* index = index * indx_fact[pulse_num1] + indx_tmp; */
            index = UL_Mpy_32_32(index, UL_deposit_l(indx_fact[pulse_num1]));
            index = UL_addNsD(index, indx_tmp);
            index_n[track - 1] = L_add(L_and(index_n[track - 1], low_mask[pulse_num0]),
                                       L_and(L_lshl(index, low_len[pulse_num0]), index_mask));

            index = L_lshr(index, hi_to_low[pulse_num0]);
        }
        track_num1 = sub(track_num, 1);
        pulse_num1 = pulse_num[track_num1];
        move16();
        index_n[track_num1] = L_and(L_add(L_and(index_n[track_num1], low_mask[pulse_num1]),
                                          L_lshl(index, low_len[pulse_num1])), index_mask);
        index = L_lshr(index, hi_to_low[pulse_num1]);
        IF (sub(indx_flag, track_num) >= 0)
        {
            IF (sub(indx_flag_2, 1) >= 0)
            {
                idxs[0] = extract_l(index_n[0]);
                idxs[1] = extract_l(L_add(L_lshl(index_n[1], 8), L_lshr(index_n[0], 16)));
                idxs[2] = extract_l(L_lshr(index_n[1], 8));
                idxs[3] = extract_l(index_n[2]);
                idxs[4] = extract_l(L_add(L_lshl(index_n[3], 8), L_lshr(index_n[2], 16)));
                idxs[5] = extract_l(L_lshr(index_n[3], 8));
                track = 6;
                move16();
            }
            ELSE
            {
                FOR (track = 0; track < track_num; track++)
                {
                    idxs[track] = extract_l(index_n[track]);
                }
            }
        }
        ELSE
        {
            idxs[0] = extract_l(L_add(L_lshl(index_n[0], 8), index_n[1]));
            idxs[1] = extract_l(L_add(L_lshl(index_n[2], 8), index_n[3]));
            IF (sub(track_num, 4) == 0)
            {
                track = 2;
                move16();
            }
            ELSE
            {
                idxs[2] = extract_l(L_add(L_lshl(index_n[4], 8), L_and(index, 0xFF)));
                index = L_lshr(index, 8);
                track = 3;
                move16();
            }
        }
    }
    ELSE
    {
        index = index_n[0];
        move32();
        FOR (track = 1; track < 4; track++)
        {
            pulse_num1 = pulse_num[track];
            index = L_add(L_lshl(index, index_len[pulse_num1]), index_n[track]);
        }
        IF (sub(track_num, 4) == 0)
        {
            track = 0;
            move16();
        }
        ELSE
        {
            idxs[0] = extract_l(index);
            index = L_lshr(index, 16);
            pulse_num1 = pulse_num[4];
            index = L_add(L_lshl(index, index_len[pulse_num1]), index_n[4]);
            track = 1;
            move16();
        }
    }
    FOR (; track < wordcnt; track++)
    {
        idxs[track] = extract_l(index);
        index = L_lshr(index, 16);
    }
}
