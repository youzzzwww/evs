/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "cnst_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define TILT_COMP_LIM_FX       24576    /* 0.75 in Q15 */
#define GE_SHIFT 6
#define P1 (32768-ISP_SMOOTHING_QUANT_A1_FX-1)
#define P9 (32767-P1)

/*---------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------*/

static Word16 calc_tilt_fx(const Word16 *x, const Word16 Q_shift, const Word16 len);
Word32 L_Sqrt_Q0(const Word32 x);
/*--------------------------------------------------------------------*
 * stat_noise_uv_mod()
 *
 * Modifies excitation signal in stationary noise segments
 *--------------------------------------------------------------------*/

void stat_noise_uv_mod_fx(
    const Word16 coder_type,       /* i  : Coder type									   */
    Word16 noisiness,        /* i  : noisiness parameter                           */
    const Word16 *lsp_old,         /* i  : old LSP vector at 4th sfr                     */
    const Word16 *lsp_new,         /* i  : LSP vector at 4th sfr						   */
    const Word16 *lsp_mid,         /* i  : LSP vector at 2nd sfr						   */
    Word16 *Aq,              /* o  : A(z)   quantized for the 4 subframes		   */
    Word16 *exc2,            /* i/o: excitation buffer							   */
    Word16 Q_exc,            /* i  : Q of exc2 excitation buffer [11..-1] expected */
    const Word16 bfi  ,            /* i  : Bad frame indicator						   */
    Word32 *ge_sm,           /* i/o: smoothed excitation gain                      */
    Word16 *uv_count,        /* i/o: unvoiced counter							   */
    Word16 *act_count,       /* i/o: activation counter							   */
    Word16 lspold_s[],       /* i/o: old LSP									   */
    Word16 *noimix_seed,     /* i/o: mixture seed								   */
    Word16 *st_min_alpha,    /* i/o: minimum alpha								   */
    Word16 *exc_pe,          /* i/o: scale Q_stat_noise							   */
    const Word32 bitrate,          /* i  : core bitrate                                  */
    const Word16 bwidth_fx,        /* i  : input bandwidth                                */
    Word16 *Q_stat_noise,    /* i/o: noise scaling                                  */
    Word16 *Q_stat_noise_ge  /* i/o: noise scaling                                  */
)
{
    Word16 exctilt;                 /* Q15 */
    Word32 vare;                    /* Q31 */
    Word16 randval;                 /* Q?? */
    Word16 alpha;                   /* Q15 */
    Word16 alpha_m1;                /* (1-alpha) Q15 */
    Word16 min_alpha;               /* Q15 */
    Word16 lspnew_s[M];             /* Same for all LSP (Q15) */
    Word16 oldlsp_mix[M];
    Word16 midlsp_mix[M];
    Word16 newlsp_mix[M];
    Word16 beta;                    /* Q15 */
    Word16 Noimix_fract;            /* (noimix_fac - 1.0) in  Q15 */
    /* noimix_fax * x <-> x + Noimix_fract * x */
    Word16 i_subfr;
    Word16 i, k;

    /* Work variables for div and sqrt */
    Word16 tmp_nom,tmp_den,tmp_shift,tmp_res;
    Word16 Qdiff,Q_local; /* new Q to be used for states Exc_pe and Ge_sm, and Exc2_local */
    Word32 L_tmp_res,L_tmp, L_tmp3,L_Ge;

    Word16 En_shift,Tmp;
    Word16 Exc2_local[L_FRAME]; /* local_copy in scaled Q_local*/

    /*---------------------------------------------------------*
     * Init local variables
     *---------------------------------------------------------*/
    alpha     = 32767;
    move16();
    min_alpha = 16384;
    move16();

    test();
    test();
    test();
    IF (sub(coder_type,INACTIVE) == 0 && ( L_sub(bitrate,ACELP_9k60) == 0 || (L_sub(bitrate,ACELP_9k60) < 0 && sub(bwidth_fx,NB) > 0) ) )
    {
        min_alpha = *st_min_alpha;
        move16();
        /*---------------------------------------------------------*
         * decode noisiness parameter
         *---------------------------------------------------------*/
        IF (bfi == 0)
        {
            tmp_den = 31;
            move16();
            tmp_shift = norm_s(tmp_den);
            move16();
            L_tmp_res = L_deposit_h(noisiness);
            L_tmp_res = L_shl(L_tmp_res,sub(tmp_shift,1));
            tmp_den = shl(tmp_den,tmp_shift);
            move16();
            tmp_res = div_l(L_tmp_res,tmp_den);
            move16();
            min_alpha = add(tmp_res, 16384);
            move16();

            /**st_min_alpha = sub(*st_min_alpha, 1638); move16();*/
            min_alpha = s_max(min_alpha, sub(*st_min_alpha, 1638));

            *st_min_alpha = min_alpha;
            move16();
        }
    }

    /*---------------------------------------------------------*
     * Mix excitation signal with random noise
     *---------------------------------------------------------*/
    test();
    test();
    test();
    IF ( sub(coder_type,INACTIVE) == 0 && ( L_sub(bitrate,ACELP_9k60) == 0 || (L_sub(bitrate,ACELP_9k60) < 0 && sub(bwidth_fx,NB) > 0) ) )
    {
        /* use a local working copy for scaling and filtering, not needed if input Q-range is fixed */
        Copy(exc2, Exc2_local, L_FRAME);

        /* bound Q for internal use, optimization possible */
        Q_local = s_min(11, s_max(-1, Q_exc));
        /* local excitation Q and incoming excitation Q*/
        Qdiff = sub(Q_local, Q_exc);
        /* only shift if incoming Q is outside [11..-1] shift is done in energy calculations aswell */
        Scale_sig(Exc2_local, L_FRAME, Qdiff);
        /* current excitation Q and previous stat_noise states Q */
        Qdiff = sub(Q_local, *Q_stat_noise);

        *Q_stat_noise_ge = GE_SHIFT;
        move16(); /* assign the fixed Q for Ge_sm */

        IF (Qdiff != 0)
        {
            Scale_sig(exc_pe, 1, Qdiff);
        }

        En_shift = 0;
        move16();
        if (sub(Q_local, 3) > 0)
        {
            /* increase margin for energy accumulation in calc_tilt and vare accumulation */
            En_shift = sub(Q_local, 3);
        }

        IF (sub(min_alpha, TILT_COMP_LIM_FX) < 0)
        {
            FOR (i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR)
            {
                exctilt = calc_tilt_fx(&Exc2_local[i_subfr], En_shift, L_SUBFR); /*Q15 */
                exctilt = mult(shl(sub(TILT_COMP_LIM_FX, min_alpha), 2), exctilt); /*Q15  */

                preemph_fx(&Exc2_local[i_subfr],exctilt,L_SUBFR,exc_pe);
            }
        }

        (*uv_count)++;

        IF (sub(*uv_count,START_NG) <= 0)
        {
            alpha = 32767;
            move16();
            *act_count = 3;
            move16();
        }
        ELSE
        {
            *uv_count = s_min(*uv_count , FULL_NG);

            tmp_nom = sub(*uv_count,START_NG);
            tmp_den = sub(FULL_NG,START_NG);
            tmp_shift = norm_s(tmp_den);
            tmp_den = shl(tmp_den,tmp_shift);
            tmp_res = div_s(tmp_nom,tmp_den);
            tmp_res = shl(tmp_res,tmp_shift);
            alpha = add(32767, mult(tmp_res, sub(min_alpha, 32767)));

            *act_count = 0;
            move16();
        }

        /*---------------------------------------------------------*
         * calculate lowpass filtered excitation gain
         *---------------------------------------------------------*/
        Tmp = shr(Exc2_local[0], En_shift);
        vare = L_mult(Tmp, Tmp); /* positive accumulation only */
        FOR (i=1; i<L_FRAME; i++)
        {
            Tmp = shr(Exc2_local[i], En_shift);
            vare = L_mac(vare, Tmp, Tmp); /* positive accumulation only */
        }

        /* obtain Ge in Q_local with safety saturation */
        L_Ge = L_shl(L_Sqrt_Q0(L_shr(vare,1)),add(sub(*Q_stat_noise_ge,4),En_shift)); /* L_Ge in Q_local*/

        /* st->ge_sm = ISP_SMOOTHING_QUANT_A1 * st->ge_sm + (1.0f-ISP_SMOOTHING_QUANT_A1) * ge */

        L_tmp = Mult_32_16(L_Ge,P1); /* 0.1*ge still in Q local */
        L_tmp3 = Mult_32_16(*ge_sm,P9); /* 0.9*ge_sm still in Q_ge  */

        *ge_sm = L_add(L_shr(L_tmp,Q_local),L_tmp3);
        move32(); /* addition in  Q_ge domain*/

        /*--------------------------------------------------------------------*
         * generate mixture of excitation and noise
         * float:
         *    noimix_fac = 1.0f/(float)sqrt(alpha*alpha + (1-alpha)*(1-alpha))
         *--------------------------------------------------------------------*/

        beta = shl(sub(alpha, 16384), 1);
        alpha_m1 = sub(32767, alpha);
        L_tmp_res = L_mac(0, alpha, alpha);
        L_tmp_res = L_mac(L_tmp_res, alpha_m1, alpha_m1);
        tmp_den = round_fx(L_Frac_sqrtQ31(L_tmp_res));

        tmp_nom = sub(32767, tmp_den);
        tmp_shift = norm_s(tmp_den);
        tmp_den = shl(tmp_den, tmp_shift);
        tmp_res = div_s(tmp_nom, tmp_den);

        Noimix_fract = shr(tmp_res, tmp_shift);              /* float value is in range 0.0 to 0.42 */

        /* L_Ge might be 0 in unvoiced WB */
        L_Ge = L_max(L_Ge, 1);
        tmp_shift = norm_l(L_Ge);
        tmp_den = extract_h(L_shl(L_Ge, tmp_shift));    /* Q_local+Q_ge+tmp_shift-16 */
        tmp_res = div_s(1<<14, tmp_den);                /* 15+14-Q_local-tmp_shift-Q_ge+16 */
        L_tmp_res = Mult_32_16(*ge_sm, tmp_res);        /* Q_stat_noise_ge+45-Q_local-Q_ge-tmp_shift-15 */
        L_tmp_res = Mult_32_16(L_tmp_res, sub(32767, beta)); /*30-Q_local-tmp_shift+15-15         */
        L_tmp_res = L_add(L_shl(L_tmp_res, sub(add(Q_local, tmp_shift), 15)), beta); /* Q15 */
        tmp_res = extract_h(L_shl(L_tmp_res, 15));  /* 15+15-16=14 */

        Noimix_fract = extract_l(Mult_32_16(L_tmp_res, Noimix_fract));    /*15+15-15 */

        FOR (i=0; i<L_FRAME; i++)
        {
            /*--------------------------------------------------------------------*
             *  flt: exc2[i] = noimix_fac*exc2[i] * alpha           + st->ge_sm*Rnd*((1.0f)-alpha)
             *  flt: exc2[i] = (noimix_fract*exc2[i]+exc2 )* alpha  + st->ge_sm*Rnd*((1.0f)-alpha)
             *  NB: currently uses 32bit accumulation for best low level performance,
             *       possibly overkill if input is always up-scaled
             *--------------------------------------------------------------------*/

            /* (1-alpha)*(float)sqrt(12.0f) * ((float)own_random(&(st->noimix_seed))/65536.0f) */
            randval = Random(noimix_seed);                     /* +/-32767 */
            randval = mult_r(28378, randval);       /* Q downscaled by 2 bits ends up in Q14 */ /*sqrt(12.0f) in Q13*/
            randval = extract_l(L_shl(Mult_32_16(L_Ge, randval), 1-*Q_stat_noise_ge));   /*Q_local+Q_ge+14-15+1-Q_ge=Q_local */

            L_tmp = L_mult(Exc2_local[i], alpha);       /* Q_local + 16 */
            L_tmp = L_mac(L_tmp, randval, alpha_m1);    /* Q_local + 16 */
            L_tmp3 = Mult_32_16(L_tmp, Noimix_fract);   /* Q_local+16+15-15 */
            L_tmp = L_add(L_tmp3, L_shl(Mult_32_16(L_tmp, tmp_res), 1)); /* Q_local+16+14-15+1 */

            Exc2_local[i] = extract_h(L_tmp);           /*Q_local */
        }
        *Q_stat_noise = Q_local;         /* update for next call, routine can only be called once every frame */
        Qdiff = sub(Q_exc, Q_local);           /* local excitation and incoming excitation */
        Scale_sig(Exc2_local, L_FRAME, Qdiff);
        Copy(Exc2_local, exc2, L_FRAME);

        /*--------------------------------------------------------------------*
         * Generate low-pass filtered version of ISP coefficients
         *--------------------------------------------------------------------*/
        FOR (k=0; k<M; k++)
        {
            move16();
            lspnew_s[k] = add(
                              mult(ISP_SMOOTHING_QUANT_A1_FX, lspold_s[k]),
                              mult(32767-ISP_SMOOTHING_QUANT_A1_FX, lsp_new[k]));
        }

        /*--------------------------------------------------------------------*
         * replace LPC coefficients
         *--------------------------------------------------------------------*/

        /*--------------------------------------------------------------------*
         * pre-calculation of (1-beta)
         *--------------------------------------------------------------------*/
        FOR (i=0; i<M; i++)
        {
            move16();
            move16();
            move16();
            oldlsp_mix[i] = add(mult(beta, lsp_old[i]),
                                mult(sub(32767, beta), lspold_s[i]));

            midlsp_mix[i] = add(mult(beta,lsp_mid[i]),
                                mult(sub(32767, beta), add(shr(lspold_s[i], 1),
                                        shr(lspnew_s[i], 1))));

            newlsp_mix[i] = add(mult(beta, lsp_new[i]),
                                mult(sub(32767, beta),lspnew_s[i]));
        }

        int_lsp4_fx( L_FRAME, oldlsp_mix, midlsp_mix, newlsp_mix, Aq, M, UNVOICED_CLAS, 0);
        Copy(lspnew_s,lspold_s,M);
    }
    ELSE /* (unvoiced_vad != 0) */
    {
        (*act_count)++;
        IF (sub(*act_count,3) > 0)
        {
            *act_count = 3;
            move16();
            *uv_count = 0;
            move16();
        }
    }

}
/*---------------------------------------------------------------------------*
 * calc_tilt()
 *
 * Calculate spectral tilt by means of 1st-order LP analysis
 *---------------------------------------------------------------------------*/

static Word16 calc_tilt_fx(    /* o  : Excitation tilt  Q15*/
    const Word16 *x,        /* i  : Signal input    */
    const Word16 Q_shift,   /* i  : input scaling   */
    const Word16 len        /* i  : lenght          */
)
{
    Word16 i;
    Word16 tmp_shift;
    Word32 L_tmp_res;
    Word16 tmp_sign,xi,xi_p1;
    Word32 r0, r1;

    r0 = L_deposit_l(0);
    r1 = L_deposit_l(0);
    xi = shr(x[0], Q_shift);
    move16();

    FOR (i=0; i<len-1; i++)
    {
        /* r0 = L_mac(r0,x[i],x[i])                                        */
        /* r1 = L_mac(r1,x[i],x[i+1]) -> correlation loop can be optimized */
        r0 = L_mac(r0,xi,xi);

        xi_p1 = shr(x[i+1], Q_shift);
        r1 = L_mac(r1, xi, xi_p1);

        xi = xi_p1;
        move16();
    }

    if (r0 == 0)
    {
        r0 = L_shl(327, 16);
    }

    tmp_shift = norm_l(r0);
    move16();
    r0 = L_shl(r0,tmp_shift);
    tmp_sign = 1;
    move16();
    if (r1 >= 0)
    {
        tmp_sign = 0;
        move16();
    }
    r1 = L_abs(r1);

    L_tmp_res = Div_32(r1,extract_h(r0), extract_l(r0));
    L_tmp_res = L_shl(L_tmp_res, tmp_shift); /*Q31 */

    if (tmp_sign != 0)
    {
        L_tmp_res = L_negate(L_tmp_res); /*Q31 */
    }

    return extract_h(L_tmp_res); /*Q15 */
}

/*---------------------------------------------------------------------------*
 * L_Sqrt_Q0
 *
 * Calculate square root from fractional values (Q0 -> Q0)
 * Uses 32 bit internal representation for precision
 *---------------------------------------------------------------------------*/
Word32 L_Sqrt_Q0(		  /* o  : Square root of input */
    const Word32 x        /* i  : Input                */
)
{
    Word32 log2_work;

    Word16 log2_int;
    Word16 log2_frac;

    IF (x > 0)
    {
        log2_int = norm_l(x);
        log2_frac = Log2_norm_lc(L_shl(x, log2_int));

        log2_work = L_mac0(30*32768L, log2_frac, 1);
        log2_work = L_msu(log2_work, log2_int, 16384);
        log2_frac = L_Extract_lc(log2_work, &log2_int);

        return Pow2(log2_int, log2_frac);
    }
    return 0;
}

