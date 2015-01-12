/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"            /* Weighted mops computation related code */
#include "rom_dec_fx.h"
#include "cnst_fx.h"

#define FORMAT_POST_FILT_G1 FL2WORD16(0.75f) /*0.75f*/ /*denominator 0.9,0.75,0.15,0.9*/
#define FORMAT_POST_FILT_G2 FL2WORD16(0.7f) /*0.7f*/ /*numerator 0.75,0.7,0.1,0.7*/
#define FORMAT_POST_FILT_G1_MAX FL2WORD16(0.8f) /*for low bit-rates on clean speech*/
#define FORMAT_POST_FILT_G1_MIN FL2WORD16(0.75f) /*for high bit-rates on clean speech and noisy speech*/

/*--------------------------------------------------------------------------
 * Local functions
 *--------------------------------------------------------------------------*/

static void Dec_postfilt( PFSTAT * pfstat, const Word16 t0, const Word16 *signal_ptr, const Word16 *coeff,
                          Word16 *sig_out, const Word16 gamma1, const Word16 gamma2, const Word16 Gain_factor, const Word16 disable_hpf );

static void pst_ltp( Word16 t0, Word16 * ptr_sig_in, Word16 * ptr_sig_pst0, Word16 gain_factor );

static void search_del( Word16 t0, Word16 * ptr_sig_in, Word16 * ltpdel, Word16 * phase, Word16 * num_gltp, Word16 * den_gltp,
                        Word16 * sh_num_gltp, Word16 * sh_den_gltp, Word16 * y_up, Word16 * off_yup );

static void filt_plt( Word16 * s_in, Word16 * s_ltp, Word16 * s_out, Word16 gain_plt );

static void compute_ltp_l( Word16 * s_in, Word16 ltpdel, Word16 phase, Word16 * y_up, Word16 * num, Word16 * den, Word16 * sh_num, Word16 * sh_den );

static Word16 select_ltp( Word16 num1, Word16 den1, Word16 sh_num1, Word16 sh_den1, Word16 num2, Word16 den2, Word16 sh_num2, Word16 sh_den2 );

static void calc_st_filt( Word16 * apond2, Word16 * apond1, Word16 * parcor0, Word16 * sig_ltp_ptr, Word16 * mem_zero );

static void modify_pst_param( const Word16 lp_noise, Word16 *g1, Word16 *g2, const Word16 coder_type, Word16 *gain_factor );

static void Dec_formant_postfilt( PFSTAT *pfstat, Word16 *signal_ptr, Word16 *coeff, Word16 *sig_out, Word16 gamma1, Word16 gamma2 );


/*--------------------------------------------------------------------------
 *  Init_post_filter
 *
 *  post filter initialization
 *--------------------------------------------------------------------------*/
void Init_post_filter(
    PFSTAT * pfstat          /* i : core decoder parameters */
)
{
    /* It is off by default */
    pfstat->on = 0;

    /* Reset */
    pfstat->reset = 0;

    /* Initialize arrays and pointers */
    set16_fx(pfstat->mem_pf_in, 0, L_SUBFR);

    /* res2 =  A(gamma2) residual */
    set16_fx(pfstat->mem_res2, 0, DECMEM_RES2);

    /* 1/A(gamma1) memory */
    set16_fx(pfstat->mem_stp, 0, L_SUBFR);

    /* null memory to compute i.r. of A(gamma2)/A(gamma1) */
    set16_fx(pfstat->mem_zero, 0, M);

    /* for gain adjustment */
    pfstat->gain_prec = 16384; /*Q14*/                                          move16();

    return;
}

/*--------------------------------------------------------------------------
 *  NB_post_filt:
 *
 *  Main routine to perform post filtering on NB synthesis
 *--------------------------------------------------------------------------*/

void nb_post_filt(
    const Word16 L_frame,       /* i  : frame length                            */
    PFSTAT *Pfstat,       /* i/o: Post filter related memories            */
    Word16 *psf_lp_noise, /* i  : Long term noise                   Q8    */
    const Word16 tmp_noise,     /* i  : noise energy                       Q0   */
    Word16 *Synth,        /* i  : 12k8 synthesis                    Qsyn  */
    const Word16 *Aq,           /* i  : LP filter coefficient             Q12   */
    const Word16 *Pitch_buf,    /* i  : Fractionnal subframe pitch buffer Q6    */
    const Word16 coder_type,    /* i  : coder_type                              */
    const Word16 disable_hpf    /* i  : flag to diabled HPF                     */
)
{
    Word16 i, j, Post_G1, Post_G2, Gain_factor;
    Word16 T0_first, *Pf_in;
    const Word16 *p_Aq;
    Word16 pf_in_buffer[M+L_FRAME16k];


    /* update long-term background noise energy during inactive frames */
    IF( sub(coder_type,INACTIVE) == 0 )
    {
        *psf_lp_noise = round_fx(L_mac(L_mult(31130, *psf_lp_noise), 26214 /*0.05 Q19*/, shl(tmp_noise,4))); /*Q8*Q15 + Q19*Q4 -> Q8 */
    }

    modify_pst_param( *psf_lp_noise, &Post_G1, &Post_G2, coder_type, &Gain_factor );

    if(Pfstat->reset)
    {
        set16_fx(Pfstat->mem_res2, 0, DECMEM_RES2);
        Copy( &Synth[L_frame-L_SYN_MEM], Pfstat->mem_pf_in, L_SYN_MEM);
        Copy( &Synth[L_frame-L_SYN_MEM], Pfstat->mem_stp, L_SYN_MEM );
        Pfstat->gain_prec = 16384;
        move16();
        Pfstat->reset = 0;
        move16();
        return;
    }
    Pf_in = &pf_in_buffer[M];
    Copy( Pfstat->mem_pf_in+L_SYN_MEM-M, &Pf_in[-M], M );
    Copy( Synth, Pf_in, L_frame );
    Copy( &Synth[L_frame - L_SYN_MEM], Pfstat->mem_pf_in, L_SYN_MEM );
    /* deactivation of the post filter in case of AUDIO because it causes problems to singing sequences */
    if( sub(coder_type,AUDIO) == 0 )
    {
        Post_G1 = 32767;
        move16();
        Post_G2 = 32767;
        move16();
        Gain_factor = 32767;
        move16();
    }


    /* run the post filter */
    p_Aq = Aq;
    move16();
    j = 0;
    move16();
    FOR (i = 0; i < L_frame; i += L_SUBFR)
    {
        T0_first = Pitch_buf[j];

        Dec_postfilt( Pfstat, T0_first, &Pf_in[i], p_Aq, &Synth[i], Post_G1, Post_G2, Gain_factor, disable_hpf );

        p_Aq += (M+1);
        j = add(j,1);
    }


    return;
}

/*----------------------------------------------------------------------------
 * Dec_postfilt()
 *
 * Post - adaptive postfilter main function
 *   Short term postfilter :
 *     Hst(z) = Hst0(z) Hst1(z)
 *     Hst0(z) = 1/g0 A(gamma2)(z) / A(gamma1)(z)
 *     if {hi} = i.r. filter A(gamma2)/A(gamma1) (truncated)
 *     g0 = SUM(|hi|) if > 1
 *     g0 = 1. else
 *     Hst1(z) = 1/(1 - |mu|) (1 + mu z-1)
 *     with mu = k1 * gamma3
 *     k1 = 1st parcor calculated on {hi}
 *     gamma3 = gamma3_minus if k1<0, gamma3_plus if k1>0
 *   Long term postfilter :
 *     harmonic postfilter :   H0(z) = gl * (1 + b * z-p)
 *       b = gamma_g * gain_ltp
 *       gl = 1 / 1 + b
 *     computation of delay p on A(gamma2)(z) s(z)
 *     sub optimal search
 *       1. search around 1st subframe delay (3 integer values)
 *       2. search around best integer with fract. delays (1/8)
 *----------------------------------------------------------------------------*/
static void Dec_postfilt(
    PFSTAT * pfstat,      /* i/o: states strucure                              */
    const Word16 t0,            /* i  : pitch delay given by coder                   */
    const Word16 * signal_ptr,  /* i  : input signal (pointer to current subframe    */
    const Word16 * coeff,       /* i  : LPC coefficients for current subframe        */
    Word16 * sig_out,     /* o  : postfiltered output                          */
    const Word16 gamma1,        /* i  : short term postfilt. den. weighting factor   */
    const Word16 gamma2,        /* i  : short term postfilt. num. weighting factor   */
    const Word16 Gain_factor,   /* i  : Gain Factor (Q15)                            */
    const Word16 disable_hpf
)
{
    /* Local variables and arrays */
    Word16 apond1[M+1];  /* s.t. denominator coeff. */
    Word16 apond2[LONG_H_ST];
    Word16 sig_ltp[L_SUBFR+1]; /* H0 output signal  */
    Word16 res2[SIZ_RES2];

    Word16 *sig_ltp_ptr;
    Word16 *res2_ptr;
    Word16 *ptr_mem_stp;

    Word16 parcor0;


    /* Init pointers and restore memories */
    res2_ptr = res2 + DECMEM_RES2;
    ptr_mem_stp = pfstat->mem_stp + L_SYN_MEM - 1;
    Copy(pfstat->mem_res2, res2, DECMEM_RES2);

    /* Compute weighted LPC coefficients */
    weight_a_fx(coeff, apond1, gamma1, M);
    weight_a_fx(coeff, apond2, gamma2, M);
    set16_fx(&apond2[M+1], 0, LONG_H_ST-(M+1));

    /* Compute A(gamma2) residual */
    Residu3_fx(apond2, signal_ptr, res2_ptr, L_SUBFR, 1);

    /* Harmonic filtering */
    sig_ltp_ptr = sig_ltp + 1;

    IF (disable_hpf == 0)
    {
        pst_ltp( t0, res2_ptr, sig_ltp_ptr, Gain_factor );
    }
    ELSE
    {
        Copy(res2_ptr, sig_ltp_ptr, L_SUBFR);
    }

    /* Save last output of 1/A(gamma1) */
    /* (from preceding subframe)       */
    sig_ltp[0] = *ptr_mem_stp;
    move16();

    /* Controls short term pst filter gain and compute parcor0 */
    calc_st_filt(apond2, apond1, &parcor0, sig_ltp_ptr, pfstat->mem_zero );

    E_UTIL_synthesis(1, apond1, sig_ltp_ptr, sig_ltp_ptr, L_SUBFR, pfstat->mem_stp+L_SYN_MEM-M, 0, M);
    Copy( sig_ltp_ptr+L_SUBFR-L_SYN_MEM, pfstat->mem_stp, L_SYN_MEM );

    /* Tilt filtering */
    Filt_mu(sig_ltp, sig_out, parcor0, L_SUBFR);

    /* Gain control */
    scale_st(signal_ptr, sig_out, &pfstat->gain_prec, L_SUBFR);

    /* Update for next subframe */
    Copy(&res2[L_SUBFR], pfstat->mem_res2, DECMEM_RES2);


    return;
}

/*--------------------------------------------------------------------------
 *  formant_post_filt:
 *
 *  Main routine to perform formant post filtering
 *--------------------------------------------------------------------------*/
void formant_post_filt(
    PFSTAT *pfstat,       /* i/o: Post filter related memories      */
    Word16 *synth_in,        /* i  : 12k8 synthesis                    */
    Word16 *Aq,           /* i  : LP filter coefficient             */
    Word16 *synth_out,  /* i/o: input signal                      */
    Word16 L_frame,
    Word32 lp_noise,  /* (i) : background noise energy (15Q16) */
    Word32 rate,         /* (i) : bit-rate */
    const Word16 off_flag            /* i  : off flag                        */
)
{
    Word16 i_subfr;
    Word16 *p_Aq;
    Word16 post_G1, post_G2;


    /*default parameter for noisy speech and high bit-rates*/
    IF (sub(L_frame, L_FRAME) == 0)
    {
        post_G2 = FL2WORD16(0.7f);
        move16();
        IF (L_sub(lp_noise, LP_NOISE_THRESH) < 0)
        {
            /*Clean speech*/
            IF (L_sub(rate, ACELP_13k20) < 0)
            {
                /*Low rates*/

                post_G1 = FL2WORD16(0.8f);
                move16();
            }
            ELSE IF (L_sub(rate, ACELP_24k40) < 0)
            {
                /*Low rates*/

                post_G1 = FL2WORD16(0.75f);
                move16();
            }
            ELSE
            {
                post_G1 = FL2WORD16(0.72f);
                move16();
            }
        }
        ELSE   /*Noisy speech*/
        {
            post_G1 = FL2WORD16(0.7f);
            move16();
            if (L_sub(rate, ACELP_15k85) < 0)
            {
                /*Low rates*/
                post_G1 = FL2WORD16(0.75f);
                move16();
            }
        }
    }
    ELSE
    {
        post_G2 = FL2WORD16(0.76f);
        move16();
        test();
        IF (L_sub(lp_noise, LP_NOISE_THRESH) >= 0)
        {
            post_G1 = FL2WORD16(0.76f);
        }
        ELSE IF (L_sub(rate, ACELP_13k20) == 0)
        {
            post_G1 = FL2WORD16(0.82f);
            move16();
        }
        ELSE IF (L_sub(rate, ACELP_16k40) == 0)
        {
            post_G1 = FL2WORD16(0.80f);
            move16();
        }
        ELSE IF (L_sub(rate, ACELP_24k40) == 0 || L_sub(rate, ACELP_32k) == 0)
        {
            post_G1 = FL2WORD16(0.78f);
            move16();
        }
        ELSE
        {
            post_G1 = FL2WORD16(0.76f);
            move16();
        }
    }

    /* Switch off post-filter */
    if( off_flag != 0 )
    {
        post_G1 = post_G2;
        move16();
    }

    /* Reset post filter */
    if( pfstat->reset != 0 )
    {
        post_G1 = MAX16B;
        move16();
        post_G2 = MAX16B;
        move16();
        pfstat->reset = 0;
        move16();
        Copy( &synth_in[L_frame-L_SYN_MEM], pfstat->mem_pf_in, L_SYN_MEM);
        Copy( &synth_in[L_frame-L_SYN_MEM], pfstat->mem_stp, L_SYN_MEM );
        pfstat->gain_prec = 16384;
        move16();
        Copy( synth_in,synth_out, L_frame );

        return;
    }

    /* input memory*/
    Copy( pfstat->mem_pf_in, synth_in-L_SYN_MEM, L_SYN_MEM);
    Copy( &synth_in[L_frame-L_SYN_MEM], pfstat->mem_pf_in, L_SYN_MEM);

    move16();
    p_Aq = Aq;
    FOR (i_subfr = 0; i_subfr < L_frame; i_subfr += L_SUBFR )
    {
        Dec_formant_postfilt( pfstat, &synth_in[i_subfr], p_Aq, &synth_out[i_subfr], post_G1, post_G2 );
        p_Aq += (M+1);
    }

}

/*----------------------------------------------------------------------------
 * Dec_postfilt
 *
 * Post - adaptive postfilter main function
 *   Short term postfilter :
 *     Hst(z) = Hst0(z) Hst1(z)
 *     Hst0(z) = 1/g0 A(gamma2)(z) / A(gamma1)(z)
 *     if {hi} = i.r. filter A(gamma2)/A(gamma1) (truncated)
 *     g0 = SUM(|hi|) if > 1
 *     g0 = 1. else
 *     Hst1(z) = 1/(1 - |mu|) (1 + mu z-1)
 *     with mu = k1 * gamma3
 *     k1 = 1st parcor calculated on {hi}
 *     gamma3 = gamma3_minus if k1<0, gamma3_plus if k1>0
 *----------------------------------------------------------------------------*/
static void Dec_formant_postfilt(
    PFSTAT *pfstat,       /* i/o: states strucure                           */
    Word16 *signal_ptr,   /* i  : input signal (pointer to current subframe */
    Word16 *coeff,        /* i  : LPC coefficients for current subframe     */
    Word16 *sig_out,      /* o  : postfiltered output                       */
    Word16 gamma1,        /* i  : short term postfilt. den. weighting factor*/
    Word16 gamma2         /* i  : short term postfilt. num. weighting factor*/
)
{
    /* Local variables and arrays */
    Word16 apond1[M+1];         /* s.t. denominator coeff. */
    Word16 apond2[LONG_H_ST];
    Word16 res2[L_SUBFR];
    Word16 resynth[L_SUBFR+1];
    Word16 parcor0;


    /* Compute weighted LPC coefficients */
    weight_a_fx(coeff, apond1, gamma1, M);
    weight_a_fx(coeff, apond2, gamma2, M);
    set16_fx(&apond2[M+1], 0, LONG_H_ST-(M+1));

    /* Compute A(gamma2) residual */
    Residu3_fx(apond2, signal_ptr, res2, L_SUBFR, 1);

    /* Controls short term pst filter gain and compute parcor0 */
    calc_st_filt(apond2, apond1, &parcor0, res2, pfstat->mem_zero );

    /* 1/A(gamma1) filtering, mem_stp is updated */
    resynth[0] = *(pfstat->mem_stp + sub(L_SYN_MEM, 1));
    move16();

    E_UTIL_synthesis(1, apond1, res2, &(resynth[1]), L_SUBFR, pfstat->mem_stp+L_SYN_MEM-M, 0, M);

    Copy( &(resynth[1])+L_SUBFR-L_SYN_MEM, pfstat->mem_stp, L_SYN_MEM );

    /* Tilt filtering */
    Filt_mu(resynth, sig_out, parcor0, L_SUBFR);

    /* Gain control */
    scale_st(signal_ptr, sig_out, &pfstat->gain_prec, L_SUBFR);


    return;
}


/*------------------------------------------------------------------------------------
 * modify_pst_param()
 *
 * Modify gamma1 and gamma2 values in function of the long term noise level
 *-----------------------------------------------------------------------------------*/

static void modify_pst_param(
    const Word16 lp_noise,     /* i  : Long term noise energy     Q8           */
    Word16 *g1,          /* o  : Gamma1 used in post filter Q15          */
    Word16 *g2,          /* o  : Gamma1 used in post filter Q15          */
    const Word16 coder_type,   /* i  : Vad information decoded in UV frame     */
    Word16 *gain_factor  /* o  : Gain factor applied in post filtering   */
)
{
    Word16 tmp;
    Word16 lp_noiseQ12;
    Word32 L_tmp;


    test();
    IF( sub(coder_type,INACTIVE) != 0 && sub(lp_noise, LP_NOISE_THR_FX) < 0 )
    {
        lp_noiseQ12 = shl(lp_noise, 4); /* to go from Q8 to Q12 */

        /* ftmp = lp_noise*BG1_FX + CG1_FX */
        tmp = mac_r(CG1_FX*65536L, lp_noiseQ12, BG1_FX<<3); /* 3 to go from Q12 to Q15 */

        tmp = s_min(tmp, POST_G1_FX );
        tmp = s_max(tmp, GAMMA1_PST12K_MIN_FX );

        *g1 = tmp;
        move16();

        /* ftmp = lp_noise*BG2_FX + CG2_FX */
        L_tmp = L_mac0(CG2_FX/2*65536L, lp_noiseQ12, BG2_FX<<3);/* L_mac0 and /2 to go from Q12 to Q14 */
        /* we go to Q30 to avoid overflow CG2_FX*/

        L_tmp = L_min(L_tmp, POST_G2_FX*65536L/2); /* /2 because L_tmp is Q30 */
        L_tmp = L_max(L_tmp, GAMMA2_PST12K_MIN_FX*65536L/2);

        *g2 = extract_h(L_shl(L_tmp, 1));       /* Q30=>Q31=>Q15 */
    }
    ELSE
    {
        *g1 = GAMMA1_PST12K_NOIS_FX;
        move16();
        *g2 = GAMMA2_PST12K_NOIS_FX;
        move16();
    }

    /* Set gain_factor of the harmonic filtering*/
    /* ftmp = (lp_noise - K_LP_NOISE)*C_LP_NOISE_FX */
    L_tmp = L_mac(-CK_LP_NOISE_FX, lp_noise, C_LP_NOISE_FX); /* tmp is in Q24 (from Q8) */

    L_tmp = L_min(L_tmp, 64*65536L); /* 0.25 in Q24 */
    L_tmp = L_max(L_tmp, 0);

    *gain_factor = extract_h(L_shl(L_tmp, 7));      /* Q24=>Q31=>Q15 */


    return;
}

/*----------------------------------------------------------------------------
 * pst_ltp
 *
 * Perform harmonic postfilter
 *----------------------------------------------------------------------------*/
static void pst_ltp(
    Word16 t0,              /* i  : pitch delay given by coder       */
    Word16 * ptr_sig_in,    /* i  : postfilter i  filter (residu2)   */
    Word16 * ptr_sig_pst0,  /* o  : harmonic postfilter o            */
    Word16 gain_factor      /* i  : Gain Factor (Q15)                */
)
{
    Word32 L_temp;

    Word16 y_up[SIZ_Y_UP];
    Word16 sig_cadr[SIZ_RES2];

    Word16 *ptr_y_up;
    Word16 *ptr_sig;
    Word16 *ptr_sig_cadr;

    Word16 i;
    Word16 temp;
    Word16 ltpdel, phase;
    Word16 num_gltp, den_gltp;
    Word16 num2_gltp, den2_gltp;
    Word16 sh_num, sh_den;
    Word16 sh_num2, sh_den2;
    Word16 gain_plt;
    Word16 off_yup;
    Word16 nb_sh_sig;



    /* i  signal justified on 13 bits */
    ptr_sig = ptr_sig_in - DECMEM_RES2;
    nb_sh_sig = getScaleFactor16(ptr_sig, add(DECMEM_RES2, L_SUBFR));
    nb_sh_sig = sub(3, nb_sh_sig);

    FOR (i = 0; i < DECMEM_RES2+L_SUBFR; i++)
    {
        /* nb_sh_sig may be >0, <0 or =0 */
        sig_cadr[i] = shr(ptr_sig[i], nb_sh_sig);
        move16();
    }
    ptr_sig_cadr = sig_cadr + DECMEM_RES2;

    /* Sub optimal delay search */
    search_del(t0, ptr_sig_cadr, &ltpdel, &phase, &num_gltp, &den_gltp, &sh_num, &sh_den, y_up, &off_yup);

    IF (num_gltp == 0)
    {
        Copy(ptr_sig_in, ptr_sig_pst0, L_SUBFR);
    }
    ELSE
    {
        IF (phase == 0)
        {
            ptr_y_up = ptr_sig_in - ltpdel;
        }
        ELSE
        {
            /* Filtering with long filter */
            compute_ltp_l(ptr_sig_cadr, ltpdel, phase, ptr_sig_pst0, &num2_gltp, &den2_gltp, &sh_num2, &sh_den2);

            IF (sub(select_ltp(num_gltp, den_gltp, sh_num, sh_den, num2_gltp, den2_gltp, sh_num2, sh_den2), 1) == 0)
            {
                /* select short filter */
                temp = sub(phase, 1);
                L_temp = L_mult0(temp, L_SUBFR + 1);
                temp = extract_l(L_temp);
                temp = add(temp, off_yup);

                /* ptr_y_up = y_up + (phase-1) * (L_SUBFR+1) + off_yup */
                ptr_y_up = y_up + temp;
            }
            ELSE
            {
                /* select long filter */
                num_gltp = num2_gltp;
                move16();
                den_gltp = den2_gltp;
                move16();
                sh_num = sh_num2;
                move16();
                sh_den = sh_den2;
                move16();
                ptr_y_up = ptr_sig_pst0;
            }

            /* rescale y_up */
            FOR (i = 0; i < L_SUBFR; i++)
            {
                /* nb_sh_sig may be >0, <0 or =0 */
                ptr_y_up[i] = shl(ptr_y_up[i], nb_sh_sig);
                move16();
            }
        }

        temp = sub(sh_num, sh_den);
        IF (temp >= 0)
        {
            den_gltp = shr(den_gltp, temp);
        }
        ELSE
        {
            num_gltp = shl(num_gltp, temp); /* >> (-temp) */
        }
        IF (sub(num_gltp, den_gltp) >= 0)
        {
            /* beta bounded to 1 */
            gain_plt = MIN_GPLT_FX;
            move16();
        }
        ELSE
        {
            /* GAMMA_G = 0.5 */
            /* gain_plt = den_gltp x 2**15 / (den_gltp + 0.5 num_gltp) */
            /* shift 1 bit to avoid overflows in add */
            num_gltp = shr(num_gltp, 2);
            den_gltp = shr(den_gltp, 1);
            temp = add(den_gltp, num_gltp);
            gain_plt = div_s(den_gltp, temp); /* Q15 */
        }

        /* decrease gain in noisy condition */
        /* gain_plt += (1.0f-gain_plt) * gain_factor */
        /* gain_plt = gain_plt + gain_factor - gain_plt*gain_factor */
        gain_plt = msu_r(L_msu(L_deposit_h(gain_plt), gain_plt, gain_factor), -32768, gain_factor);

        /** filtering by H0(z) = harmonic filter **/
        filt_plt(ptr_sig_in, ptr_y_up, ptr_sig_pst0, gain_plt);
    }

}

/*----------------------------------------------------------------------------
 * search_del:
 *
 * Computes best (shortest) integer LTP delay + fine search
 *---------------------------------------------------------------------------*/
static void search_del(
    Word16 t0,            /* i  : pitch delay given by coder        */
    Word16 * ptr_sig_in,  /* i  : i  signal (with delay line)       */
    Word16 * ltpdel,      /* o  : delay = *ltpdel - *phase / f_up   */
    Word16 * phase,       /* o  : phase                             */
    Word16 * num_gltp,    /* o  : 16 bits numerator of LTP gain     */
    Word16 * den_gltp,    /* o  : 16 bits denominator of LTP gain   */
    Word16 * sh_num_gltp, /* o  : justification for num_gltp        */
    Word16 * sh_den_gltp, /* o  : justification for den_gltp        */
    Word16 * y_up,        /* o  : LT delayed signal if fract. delay */
    Word16 * off_yup      /* o  : offset in y_up                    */
)
{
    Word32 L_den0[F_UP_PST - 1];
    Word32 L_den1[F_UP_PST - 1];

    Word32 *ptr_L_den0, *ptr_L_den1;

    Word32 L_num_int, L_den_int, L_den_max;
    Word32 L_temp0, L_temp1;
    Word32 L_acc;
    Word32 L_temp;

    const Word16 *ptr_h;
    Word16 *ptr_sig_past, *ptr_sig_past0;
    Word16 *ptr1, *ptr_y_up;

    Word16 i, n;
    Word16 num, den0, den1;
    Word16 den_max, num_max;
    Word32 L_numsq_max;
    Word16 ener;
    Word16 sh_num, sh_den, sh_ener;
    Word16 i_max, lambda, phi, phi_max, ioff;
    Word16 temp;


    /*-------------------------------------
     * Computes energy of current signal
     *-------------------------------------*/

    L_acc = L_mult(ptr_sig_in[0], ptr_sig_in[0]);
    FOR(i = 1; i < L_SUBFR; i++)
    {
        L_acc = L_mac(L_acc, ptr_sig_in[i], ptr_sig_in[i]);
    }
    IF (L_acc == 0)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }
    sh_ener = sub(16, norm_l(L_acc));
    /* save energy for final decision */
    sh_ener = s_max(0, sh_ener);
    ener = extract_l(L_shr(L_acc, sh_ener));

    /*-------------------------------------
     * Selects best of 3 integer delays
     * Maximum of 3 numerators around t0
     *-------------------------------------*/
    lambda = sub(t0, 1);
    ptr_sig_past = ptr_sig_in - lambda;
    L_num_int = L_deposit_l(-1);

    /* initialization used only to suppress Microsoft Visual C++ warnings */
    i_max = (Word16) 0;
    move16();

    FOR (i = 0; i < 3; i++)
    {
        L_acc = L_mult(ptr_sig_in[0], ptr_sig_past[0]);
        FOR (n = 1; n < L_SUBFR; n++)
        {
            L_acc = L_mac(L_acc, ptr_sig_in[n], ptr_sig_past[n]);
        }
        L_acc = L_max(L_acc, 0);
        L_temp = L_sub(L_acc, L_num_int);
        if (L_temp > 0L)
        {
            i_max = (Word16) i;
            move16();
        }
        L_num_int = L_max(L_num_int, L_acc);
        ptr_sig_past--;
    }

    IF (L_num_int == 0)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }

    /* Compute den for i_max */
    lambda = add(lambda, (Word16) i_max);
    ptr_sig_past = ptr_sig_in - lambda;
    temp = *ptr_sig_past++;
    move16();
    L_acc = L_mult(temp, temp);
    FOR (i = 1; i < L_SUBFR; i++)
    {
        temp = *ptr_sig_past++;
        move16();
        L_acc = L_mac(L_acc, temp, temp);
    }
    IF (L_acc == 0L)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }
    L_den_int = L_add(0, L_acc); /* sets to 'L_acc' in 1 clock */

    /*----------------------------------
     * Select best phase around lambda
     *----------------------------------
     * Compute y_up & denominators
     *----------------------------------*/

    ptr_y_up = y_up;
    L_den_max = L_add(0, L_den_int); /* sets to 'L_acc' in 1 clock */
    ptr_L_den0 = L_den0;
    ptr_L_den1 = L_den1;
    ptr_h = Tab_hup_s;
    temp = sub(lambda, LH_UP_S - 1);
    ptr_sig_past0 = ptr_sig_in - temp;

    /* Loop on phase */
    FOR (phi = 1; phi < F_UP_PST; phi++)
    {
        /* Compute y_up for lambda+1 - phi/F_UP_PST */
        /* and lambda - phi/F_UP_PST */

        ptr_sig_past = ptr_sig_past0;
        FOR (n = 0; n <= L_SUBFR; n++)
        {
            ptr1 = ptr_sig_past++;

            L_acc = L_mult(ptr_h[0], ptr1[0]);
            FOR (i = 1; i < LH2_S; i++)
            {
                L_acc = L_mac(L_acc, ptr_h[i], ptr1[-i]);
            }
            ptr_y_up[n] = round_fx(L_acc);
        }

        /* compute den0 (lambda+1) and den1 (lambda) */

        /* part common to den0 and den1 */
        L_acc = L_mult(ptr_y_up[1], ptr_y_up[1]);
        FOR (n = 2; n < L_SUBFR; n++)
        {
            L_acc = L_mac(L_acc, ptr_y_up[n], ptr_y_up[n]);
        }
        L_temp0 = L_add(0, L_acc); /* sets to 'L_acc' in 1 clock (saved for den1) */

        /* den0 */
        L_acc = L_mac(L_acc, ptr_y_up[0], ptr_y_up[0]);
        *ptr_L_den0 = L_acc;
        move32();

        /* den1 */
        L_acc = L_mac(L_temp0, ptr_y_up[L_SUBFR], ptr_y_up[L_SUBFR]);
        *ptr_L_den1 = L_acc;
        move32();

        IF (sub(abs_s(ptr_y_up[0]), abs_s(ptr_y_up[L_SUBFR])) > 0)
        {
            L_den_max = L_max(*ptr_L_den0, L_den_max);
        }
        ELSE
        {
            L_den_max = L_max(*ptr_L_den1, L_den_max);
        }
        ptr_L_den0++;
        ptr_L_den1++;
        ptr_y_up += (L_SUBFR+1);
        ptr_h += LH2_S;
    }

    IF (L_den_max == 0)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }

    sh_den = sub(16, norm_l(L_den_max));
    /* if sh_den <= 0 : dynamic between current frame */
    /* and delay line too high */
    IF (sh_den <= 0)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }

    /* search sh_num to justify correlations */
    /* sh_num = Max(sh_den, sh_ener) */
    sh_num = sh_ener;
    move16();
    if (sub(sh_den, sh_ener) >= 0)
    {
        sh_num = sh_den;
        move16();
    }

    /* Computation of the numerators */
    /* and selection of best num*num/den */
    /* for non null phases */

    /* Initialize with null phase */
    L_acc = L_shr(L_den_int, sh_den); /* sh_den > 0 */
    den_max = extract_l(L_acc);
    L_acc = L_shr(L_num_int, sh_num); /* sh_num > 0 */
    num_max = extract_l(L_acc);
    L_numsq_max = L_mult(num_max, num_max);

    phi_max = 0;
    move16();
    ioff = 1;
    move16();

    ptr_L_den0 = L_den0;
    ptr_L_den1 = L_den1;
    ptr_y_up = y_up;


    /* if den_max = 0 : will be selected and declared unvoiced */
    /* if num!=0 & den=0 : will be selected and declared unvoiced */
    /* degenerated seldom cases, switch off LT is OK */

    /* Loop on phase */
    FOR (phi = 1; phi < F_UP_PST; phi++)
    {
        /* compute num for lambda+1 - phi/F_UP_PST */
        L_acc = L_mult(ptr_sig_in[0], ptr_y_up[0]);
        FOR (n = 1; n < L_SUBFR; n++)
        {
            L_acc = L_mac(L_acc, ptr_sig_in[n], ptr_y_up[n]);
        }
        L_acc = L_shr(L_acc, sh_num); /* sh_num > 0 */
        L_acc = L_max(0, L_acc);
        num = extract_l(L_acc);

        /* selection if num**2/den0 max */
        L_temp1 = L_mult(num, num);
        L_temp0 = Mpy_32_16_1(L_temp1, den_max);
        L_acc = L_add(*ptr_L_den0++, 0);
        L_acc = L_shr(L_acc, sh_den); /* sh_den > 0 */
        den0 = extract_l(L_acc);
        L_temp = Msub_32_16(L_temp0, L_numsq_max, den0);
        IF (L_temp > 0L)
        {
            num_max = num;
            move16();
            L_numsq_max = L_add(0, L_temp1); /* sets to 'L_temp1' in 1 clock */
            den_max = den0;
            move16();
            ioff = 0;
            move16();
            phi_max = phi;
            move16();
        }

        /* compute num for lambda - phi/F_UP_PST */
        ptr_y_up++;

        L_acc = L_mult(ptr_sig_in[0], ptr_y_up[0]);
        FOR (n = 1; n < L_SUBFR; n++)
        {
            L_acc = L_mac(L_acc, ptr_sig_in[n], ptr_y_up[n]);
        }
        L_acc = L_shr(L_acc, sh_num); /* sh_num > 0 */
        L_acc = L_max(0, L_acc);
        num = extract_l(L_acc);

        /* selection if num**2/den1 max */
        L_temp1 = L_mult(num, num);
        L_temp0 = Mpy_32_16_1(L_temp1, den_max);
        L_acc = L_add(*ptr_L_den1++, 0);
        L_acc = L_shr(L_acc, sh_den); /* sh_den > 0 */
        den1 = extract_l(L_acc);
        L_temp = Msub_32_16(L_temp0, L_numsq_max, den1);
        IF (L_temp > 0L)
        {
            num_max = num;
            move16();
            L_numsq_max = L_add(0, L_temp1); /* sets to 'L_temp1' in 1 clock */
            den_max = den1;
            move16();
            ioff = 1;
            move16();
            phi_max = phi;
            move16();
        }

        ptr_y_up += L_SUBFR;
    }

    /*---------------------------------------------------
     * test if normalized crit0[iopt] > THRESHCRIT
     *--------------------------------------------------*/
    test();
    IF (num_max == 0 || sub(den_max, 1) <= 0)
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();

        return;
    }

    /* compare num**2 */
    /* to ener * den * 0.5 */
    /* (THRESHCRIT = 0.5) */
    L_temp1 = L_mult(den_max, ener);

    /* temp = 2 * sh_num - sh_den - sh_ener + 1 */
    /* 16 bits with no overflows */
    temp = shl(sh_num, 1);
    temp = sub(temp, sh_den);
    temp = sub(temp, sh_ener);
    temp = add(temp, 1);
    IF (temp < 0)
    {
        temp = negate(temp); /* no overflow */
        L_numsq_max = L_shr(L_numsq_max, temp);
    }
    ELSE
    {
        if (temp > 0)
        {
            L_temp1 = L_shr(L_temp1, temp);
        }
    }
    L_temp = L_sub(L_numsq_max, L_temp1);
    IF (L_temp >= 0L)
    {
        temp = add(lambda, 1);
        *ltpdel = sub(temp, ioff);
        *off_yup = ioff;
        move16();
        *phase = phi_max;
        move16();
        *num_gltp = num_max;
        move16();
        *den_gltp = den_max;
        move16();
        *sh_den_gltp = sh_den;
        move16();
        *sh_num_gltp = sh_num;
        move16();
    }
    ELSE
    {
        *num_gltp = 0;
        move16();
        *den_gltp = 1;
        move16();
        *ltpdel = 0;
        move16();
        *phase = 0;
        move16();
    }


    return;
}

/*----------------------------------------------------------------------------
 *  filt_plt:
 *
 * Perform long term postfilter
 *----------------------------------------------------------------------------*/
static void filt_plt(
    Word16 * s_in,      /* i  : i  signal with past         */
    Word16 * s_ltp,     /* i  : filtered signal with gain 1 */
    Word16 * s_out,     /* o  : signal                      */
    Word16 gain_plt     /* i  : filter gain                 */
)
{

    /* Local variables */
    Word32 L_acc;

    Word16 n;
    Word16 gain_plt_1;


    gain_plt_1 = sub(32767, gain_plt);
    gain_plt_1 = add(gain_plt_1, 1); /* 2**15 (1 - g) */

    FOR (n = 0; n < L_SUBFR; n++)
    {
        /* s_out(n) = gain_plt x s_in(n) + gain_plt_1 x s_ltp(n) */
        L_acc = L_mult(gain_plt, s_in[n]);
        s_out[n] = mac_r(L_acc, gain_plt_1, s_ltp[n]);
        move16(); /* no overflow */
    }


    return;
}


/*----------------------------------------------------------------------------
 * compute_ltp_l :
 *
 * compute delayed signal, num & den of gain for fractional delay
 * with long interpolation filter
 *----------------------------------------------------------------------------*/
static void compute_ltp_l(
    Word16 * s_in,      /* i/o: signal with past            */
    Word16 ltpdel,      /* i  : delay factor                */
    Word16 phase,       /* i  : phase factor                */
    Word16 * y_up,      /* i  : delayed signal              */
    Word16 * num,       /* i  : numerator of LTP gain       */
    Word16 * den,       /* i  : denominator of LTP gain     */
    Word16 * sh_num,    /* i  : justification factor of num */
    Word16 * sh_den     /* i  : justification factor of den */
)
{
    Word32 L_acc;
    Word16 *ptr2;
    const Word16 *ptr_h;
    Word16 n, i;
    Word16 temp;


    temp = sub(phase, 1);
    temp = shl(temp, L2_LH2_L);
    ptr_h = Tab_hup_l + temp; /* Tab_hup_l + LH2_L * (phase-1) */

    temp = sub(LH_UP_L, ltpdel);
    ptr2 = s_in + temp;

    /* Compute y_up */
    FOR (n = 0; n < L_SUBFR; n++)
    {
        L_acc = L_mult(ptr_h[0], *ptr2--);
        FOR (i = 1; i < LH2_L; i++)
        {
            L_acc = L_mac(L_acc, ptr_h[i], *ptr2--);
        }
        y_up[n] = round_fx(L_acc);
        ptr2 += LH2_L_P1;
    }

    /* Compute num */
    L_acc = L_mult(y_up[0], s_in[0]);
    FOR (n = 1; n < L_SUBFR; n++)
    {
        L_acc = L_mac(L_acc, y_up[n], s_in[n]);
    }
    IF (L_acc < 0L)
    {
        *num = 0;
        move16();
        *sh_num = 0;
        move16();
    }
    ELSE
    {
        temp = sub(16, norm_l(L_acc));
        temp = s_max(temp, 0);
        L_acc = L_shr(L_acc, temp); /* with temp >= 0 */
        *num = extract_l(L_acc);
        *sh_num = temp;
        move16();
    }

    /* Compute den */
    L_acc = L_mult(y_up[0], y_up[0]);
    FOR (n = 1; n < L_SUBFR; n++)
    {
        L_acc = L_mac(L_acc, y_up[n], y_up[n]);
    }
    temp = sub(16, norm_l(L_acc));
    temp = s_max(temp, 0);
    L_acc = L_shr(L_acc, temp); /* with temp >= 0 */
    *den = extract_l(L_acc);
    *sh_den = temp;
    move16();


    return;
}

/*----------------------------------------------------------------------------
 *  select_ltp:
 *
 *  selects best of (gain1, gain2)
 *  with gain1 = num1 * 2** sh_num1 / den1 * 2** sh_den1
 *  and  gain2 = num2 * 2** sh_num2 / den2 * 2** sh_den2
 *----------------------------------------------------------------------------*/
static Word16 select_ltp(  /* o  : 1 = 1st gain, 2 = 2nd gain  */
    Word16 num1,    /* i  : numerator of gain1          */
    Word16 den1,    /* i  : denominator of gain1        */
    Word16 sh_num1, /* i  : just. factor for num1       */
    Word16 sh_den1, /* i  : just. factor for den1       */
    Word16 num2,    /* i  : numerator of gain2          */
    Word16 den2,    /* i  : denominator of gain2        */
    Word16 sh_num2, /* i  : just. factor for num2       */
    Word16 sh_den2  /* i  : just. factor for den2       */
)
{
    Word32 L_temp1, L_temp2;
    Word32 L_temp;

    Word16 temp1, temp2;


    IF (den2 == 0)
    {
        return 1;
    }

    /* compares criteria = num**2/den */
    L_temp1 = L_mult(num1, num1);
    L_temp1 = Mpy_32_16_1(L_temp1, den2);

    L_temp2 = L_mult(num2, num2);
    L_temp2 = Mpy_32_16_1(L_temp2, den1);

    /* temp1 = sh_den2 + 2 * sh_num1 */
    temp1 = shl(sh_num1, 1);
    temp1 = add(temp1, sh_den2);
    /* temp2 = sh_den1 + 2 * sh_num2; */
    temp2 = shl(sh_num2, 1);
    temp2 = add(temp2, sh_den1);

    temp2 = sub(temp2, temp1);
    if (temp2 > 0)
    {
        L_temp1 = L_shr(L_temp1, temp2); /* temp2 > 0 */
    }
    if (temp2 < 0)
    {
        L_temp2 = L_shl(L_temp2, temp2); /* temp2 < 0 */
    }

    L_temp = L_sub(L_temp2, L_temp1);
    temp1 = 1;
    move16();
    if (L_temp > 0L)
    {
        temp1 = 2;
        move16();
    }

    return temp1;
}

/*----------------------------------------------------------------------------
 * calc_st_filt
 *
 * computes impulse response of A(gamma2) / A(gamma1)
 * controls gain : computation of energy impulse response as
 *                 SUMn  (abs (h[n])) and computes parcor0
 *---------------------------------------------------------------------------- */
static void calc_st_filt(
    Word16 * apond2,      /* i  : coefficients of numerator             */
    Word16 * apond1,      /* i  : coefficients of denominator           */
    Word16 * parcor0,     /* o  : 1st parcor calcul. on composed filter */
    Word16 * sig_ltp_ptr, /* i/o: i  of 1/A(gamma1) : scaled by 1/g0    */
    Word16 * mem_zero     /* i  : All zero memory                       */
)
{
    Word32 L_g0;

    Word16 h[LONG_H_ST];

    Word16 g0, temp;
    Word16 i;


    temp = sub( 2, norm_s( apond2[0] ) );

    /* compute i.r. of composed filter apond2 / apond1 */
    E_UTIL_synthesis(temp, apond1, apond2, h, LONG_H_ST, mem_zero, 0, M);

    /* compute 1st parcor */
    Calc_rc0_h(h, parcor0);

    /* compute g0 */
    L_g0 = L_mult0(1, abs_s(h[0]));
    FOR (i = 1; i < LONG_H_ST; i++)
    {
        L_g0 = L_mac0(L_g0, 1, abs_s(h[i]));
    }
    g0 = extract_h(L_shl(L_g0, 14));

    /* Scale signal i  of 1/A(gamma1) */
    IF (sub(g0, 1024) > 0)
    {
        temp = div_s(1024, g0); /* temp = 2**15 / gain0 */
        FOR (i = 0; i < L_SUBFR; i++)
        {
            sig_ltp_ptr[i] = mult_r(sig_ltp_ptr[i], temp);
            move16();
        }
    }


    return;
}

/*----------------------------------------------------------------------------
 * filt_mu
 *
 * tilt filtering with : (1 + mu z-1) * (1/1-|mu|)
 *      computes y[n] = (1/1-|mu|) (x[n]+mu*x[n-1])
 *---------------------------------------------------------------------------*/
void Filt_mu(
    Word16 * sig_in,        /* i  : signal (beginning at sample -1)     */
    Word16 * sig_out,       /* o  : signal with tilt                    */
    Word16 parcor0,         /* i  : parcor0 (mu = parcor0 * gamma3)     */
    Word16 L_subfr          /* i  : the length of subframe              */
)
{
    Word32 L_acc, L_temp, L_fact;

    Word16 *ptrs;

    Word16 n;
    Word16 mu, mu2, ga, temp;
    Word16 fact, sh_fact;


    IF (parcor0 > 0)
    {
        mu = mult_r(parcor0, GAMMA3_PLUS_FX);
        /* GAMMA3_PLUS_FX < 0.5 */
        sh_fact = 14;
        move16(); /* sh_fact */
        fact = (Word16) 0x4000;
        move16(); /* 2**sh_fact */
        L_fact = (Word32) L_deposit_l(0x2000); /* fact >> 1 */
    }
    ELSE
    {
        mu = mult_r(parcor0, GAMMA3_MINUS_FX);
        /* GAMMA3_MINUS_FX < 0.9375 */
        sh_fact = 11;
        move16(); /* sh_fact */
        fact = (Word16) 0x0800;
        move16(); /* 2**sh_fact */
        L_fact = (Word32) L_deposit_l(0x0400); /* fact >> 1 */
    }

    temp = sub(1, abs_s(mu));
    BASOP_SATURATE_WARNING_OFF;
    mu2 = add(32767, temp); /* 2**15 (1 - |mu|) */
    BASOP_SATURATE_WARNING_ON;
    ga = div_s(fact, mu2); /* 2**sh_fact / (1 - |mu|) */

    ptrs = sig_in; /* points on sig_in(-1) */

    sh_fact = sub(sh_fact,16);  /* to remove the saturate(), should shl by 16 before rounding */

    FOR (n = 0; n < L_subfr; n++)
    {
        L_acc = L_mult0(mu, *ptrs++);
        L_temp = L_mac(L_acc, 16384, *ptrs); /* sig_in(n) * 2**15 */

        L_temp = Madd_32_16(L_fact, L_temp, ga);
        L_temp = L_shr(L_temp, sh_fact); /* mult. temp x ga */

        BASOP_SATURATE_WARNING_OFF;
        /*sig_out[n] = saturate(L_temp); move16();*/
        sig_out[n] = round_fx(L_temp);
        BASOP_SATURATE_WARNING_ON;
    }


    return;
}


/*----------------------------------------------------------------------------
 * scale_st()
 *
 * control of the subframe gain
 * gain[n] = AGC_FAC_FX * gain[n-1] + (1 - AGC_FAC_FX) g_in/g_out
 *---------------------------------------------------------------------------*/
void scale_st(
    const Word16 * sig_in,    /* i  : postfilter i signal             */
    Word16 * sig_out,   /* i/o: postfilter o signal             */
    Word16 * gain_prec, /* i/o: last value of gain for subframe */
    Word16 L_subfr
)
{
    Word32 L_acc, L_temp;

    Word16 i;
    Word16 scal_in, scal_out;
    Word16 s_g_in, s_g_out, temp, sh_g0, g0;
    Word16 gain = 0;


    /* compute i  gain */
    L_acc = L_deposit_l(0);
    FOR (i = 0; i < L_subfr; i++)
    {
        if(sig_in[i] > 0)
        {
            L_acc = L_mac0(L_acc, 1, sig_in[i]);
        }
        if(sig_in[i] < 0)
        {
            L_acc = L_msu0(L_acc, 1, sig_in[i]);
        }
    }

    g0 = 0;
    move16();
    IF (L_acc != 0L)
    {
        scal_in = norm_l(L_acc);
        L_acc = L_shl(L_acc, scal_in);
        s_g_in = extract_h(L_acc); /* normalized */

        /* Compute o   gain */
        L_acc = L_mult0(1, abs_s(sig_out[0]));
        FOR (i = 1; i < L_subfr; i++)
        {
            L_acc = L_mac0(L_acc, 1, abs_s(sig_out[i]));
        }
        IF (L_acc == 0L)
        {
            *gain_prec = 0;
            move16();

            return;
        }
        scal_out = norm_l(L_acc);
        L_acc = L_shl(L_acc, scal_out);
        s_g_out = extract_h(L_acc); /* normalized */

        sh_g0 = add(scal_in, 1);
        sh_g0 = sub(sh_g0, scal_out); /* scal_in - scal_out + 1 */
        IF (sub(s_g_in, s_g_out) < 0)
        {
            g0 = div_s(s_g_in, s_g_out); /* s_g_in/s_g_out in Q15 */
        }
        ELSE
        {
            temp = sub(s_g_in, s_g_out); /* sufficient since normalized */
            g0 = shr(div_s(temp, s_g_out), 1);
            g0 = add(g0, (Word16) 0x4000); /* s_g_in/s_g_out in Q14 */
            sh_g0 = sub(sh_g0, 1);
        }
        /* L_gain_in/L_gain_out in Q14 */
        /* overflows if L_gain_in > 2 * L_gain_out */
        g0 = shr(g0, sh_g0); /* sh_g0 may be >0, <0, or =0 */

        g0 = mult_r(g0, AGC_FAC1_FX); /* L_gain_in/L_gain_out * AGC_FAC1_FX */
    }

    /* gain(n) = AGC_FAC gain(n-1) + AGC_FAC1 gain_in/gain_out */
    /* sig_out(n) = gain(n) sig_out(n) */
    gain = *gain_prec;
    move16();
    FOR (i = 0; i < L_subfr; i++)
    {
        temp = mult_r(AGC_FAC_FX, gain);
        gain = add(temp, g0); /* in Q14 */
        L_temp = L_mult(gain, sig_out[i]);
        L_temp = L_shl(L_temp, 1);
        sig_out[i] = round_fx(L_temp);
    }
    *gain_prec = gain;
    move16();


    return;
}

