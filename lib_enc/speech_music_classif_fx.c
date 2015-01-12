/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_enc_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/
#define ATT_NSEG              32
#define ATT_SEG_LEN           (L_FRAME/ATT_NSEG)
#define ATT_3LSUB_POS         (3 * ATT_NSEG / NB_SUBFR)
#define ATT_3LSUB_POS_16k     26 /* (short)((4.0f * ATT_NSEG / (float)NB_SUBFR16k) + 0.5f) */

#define LOG_PROB_CONST        11292 /*0.5f * N_FEATURES * LOG_PI2 in Q10 */

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static Word16 sp_mus_classif_gmm_fx( Encoder_State_fx *st_fx, const Word16 localVAD, const Word16 pitch[3], const Word16 voicing[3],
                                     const Word16 lsp_new[M], const Word16 cor_map_sum, const Word32 epsP[M+1], const Word32 PS[],
                                     Word16 non_sta, Word16 relE, Word16 *voi_fv, Word16 *cor_map_sum_fv, Word16 *LPCErr, Word16 Q_esp
                                     , Word16 *high_lpn_flag_ptr);


static void sp_mus_classif_2nd_fx( Encoder_State_fx *st, const Word16 sp_aud_decision1, Word16 *sp_aud_decision2,
                                   const Word16 pitch[3], const Word16 Etot, Word16 *coder_type, Word16 *attack_flag,
                                   const Word16 *inp, const Word16 Qx, const Word16 localVAD, const Word16 vad_flag );

static void music_mixed_classif_improv_fx( Encoder_State_fx *st, const Word16 *new_inp, Word16 *sp_aud_decision1, Word16 vad_flag,
        const Word16 *voicing, const Word32 *epsP, Word16 Q_epsP, Word16 etot, Word16 old_cor,
        Word16 cor_map_sum );

static void tonal_context_improv_fx( Encoder_State_fx *st_fx, const Word32 PS[], Word16 *sp_aud_decision1, Word16 *sp_aud_decision2,
                                     const Word16 vad_flag, const Word16 pitch[3], const Word16 voicing[3], const Word16 voi_fv,
                                     const Word16 cor_map_sum_fv, const Word16 LPCErr, const Word16 Qx );

static void var_cor_calc_fx( const Word16 old_corr, Word16 *mold_corr, Word16 var_cor_t[], Word16 *high_stable_cor );

static Word16 attack_det_fx( const Word16 *inp, const Word16 Qx, const Word16 last_clas, const Word16 localVAD, const Word16 coder_type
                             ,const Word32 total_brate

                           );

static void order_spectrum_fx( Word16 *vec, Word16 len );

static void detect_sparseness_fx( Encoder_State_fx *st_fx, const Word16 localVAD_HE_SAD, Word16 *sp_aud_decision1,
                                  Word16 *sp_aud_decision2, const Word16 voi_fv );


/*---------------------------------------------------------------------*
 * speech_music_classif()
 *
 * Speech/music classification
 *
 * The following technologies are used based on the outcome of the sp/mus classifier
 * sp_aud_decision1  sp_aud_decision2
 *       0                 0             use ACELP (+TD BWE)
 *       1                 0             use ACELP (+FD BWE) or HQ/LR-MDCT depending on bitrate
 *       1                 1             use GSC (+FD BWE) or HQ/LR-MDCT depending on bitrate
 *
 *       0                 1             exceptionally use GSC (+FD BWE) instead of LR-MDCT at 13.2 kbps (WB/SWB) for sparse spectra
 *---------------------------------------------------------------------*/

void speech_music_classif_fx(
    Encoder_State_fx *st,           /* i/o: state structure                                 */
    Word16 *sp_aud_decision0,
    Word16 *sp_aud_decision1, /* o  : 1st stage speech/music decision for GSC         */
    Word16 *sp_aud_decision2, /* o  : 2nd stage speech/music decision for GSC         */
    const Word16 *new_inp,          /* i  : new input signal                                */
    const Word16 *inp,              /* i  : input signal to locate attach position          */
    const Word16 vad_flag,
    const Word16 localVAD,
    const Word16 localVAD_HE_SAD,   /* i  : HE-SAD flag without hangover                    */
    const Word16 pitch[3],          /* i  : open-loop pitch estimate in three subframes     */
    const Word16 voicing[3],        /* i  : voicing estimate in three subframes         Q15 */
    const Word16 lsp_new[M],        /* i  : LSPs in current frame                       Q15 */
    const Word16 cor_map_sum,       /* i  : correlation map sum (from multi-harmonic anal.)Q8*/
    const Word32 epsP[M+1],         /* i  : LP prediciton error                         Q_esp*/
    const Word32 PS[],              /* i  : energy spectrum                     Q_new+QSCALE*/
    const Word16 Etot,              /* i  : total frame energy                          Q8  */
    const Word16 old_cor,           /* i  : max correlation from previous frame         Q15 */
    Word16 *coder_type,       /* i/o: coding type                                     */
    Word16 *attack_flag,      /* o  : flag to indicate if attack is to be treated by TC or GSC */
    Word16 non_sta,           /* i  : unbound non-stationarity for sp/mus classifier */
    Word16 relE,              /* i  : relative frame energy */
    Word16 Q_esp,             /* i  : scaling of esP */
    Word16 Q_inp,             /* i  : scaling of input */
    Word16 *high_lpn_flag_ptr,    /* o  :    noise log prob flag for NOISE_EST        */
    Word16 flag_spitch        /* i  : flag to indicate very short stable pitch                  */
)
{
    Word16 voi_fv, cor_map_sum_fv, LPCErr;


    /* 1st stage speech/music classifier based on the GMM model */
    *sp_aud_decision1 = sp_mus_classif_gmm_fx( st, localVAD_HE_SAD, pitch, voicing, lsp_new, cor_map_sum,
                        epsP, PS, non_sta, relE, &voi_fv, &cor_map_sum_fv, &LPCErr,


                        Q_esp, high_lpn_flag_ptr );

    test();
    IF ( sub( st->codec_mode, MODE1) == 0 || L_sub(st->sr_core, 12800) == 0 )
    {


        /* Improvement of the 1st stage decision on mixed/music content */
        test();
        IF ( st->Opt_SC_VBR_fx == 0 && ( L_sub(st->total_brate_fx, ACELP_24k40) != 0 ) )
        {


            music_mixed_classif_improv_fx( st, new_inp, sp_aud_decision1, vad_flag, voicing, epsP, Q_esp, Etot,
                                           old_cor, cor_map_sum );


        }

        *sp_aud_decision0 = *sp_aud_decision1;


        /* 2nd stage speech/music classifier (rewrite music to speech in onsets) */
        *sp_aud_decision2 = *sp_aud_decision1;
        move16();

        IF (  sub(st->bwidth_fx,NB) > 0 )
        {
            sp_mus_classif_2nd_fx( st, *sp_aud_decision1, sp_aud_decision2, pitch, Etot, coder_type,
                                   attack_flag, inp, Q_inp-1, localVAD, vad_flag );

            /* avoid switch to AUDIO/MUSIC class for very short stable high pitch
            	and/or stable pitch with high correlation at low bitrates*/
            test();
            test();
            IF ( flag_spitch && sub(st->bwidth_fx,WB) == 0 && L_sub(st->total_brate_fx,ACELP_13k20) < 0 )
            {
                *sp_aud_decision2 = 0;
                move16();
            }
        }



        /* Context-based improvement of 1st and 2nd stage decision on stable tonal signals */
        test();
        IF ( st->Opt_SC_VBR_fx == 0 && ( L_sub(st->total_brate_fx, ACELP_24k40) != 0 ) )
        {
            tonal_context_improv_fx( st, PS, sp_aud_decision1, sp_aud_decision2, vad_flag, pitch, voicing,
                                     voi_fv, cor_map_sum_fv, LPCErr, Q_inp + QSCALE -2 );
        }

        /* Avoid using LR-MDCT on sparse spectra, use GSC instead at 13.2 kbps (WB/SWB) */
        test();
        test();
        test();
        test();
        IF ( !st->Opt_SC_VBR_fx && L_sub(st->total_brate_fx, ACELP_13k20) == 0 && sub(vad_flag, 1) == 0 &&
             ( sub(st->bwidth_fx, WB) == 0 || sub(st->bwidth_fx, SWB) == 0 ) )
        {
            detect_sparseness_fx( st, localVAD_HE_SAD, sp_aud_decision1, sp_aud_decision2, voi_fv );
        }


        /* override speech/music classification to ACELP when background noise level reaches certain level */
        /* this is a patch against mis-classifications during active noisy speech segments */
        IF ( sub(st->lp_noise_fx, 3072) > 0 )
        {
            *sp_aud_decision1 = 0;
            move16();
            *sp_aud_decision2 = 0;
            move16();
        }


        /* select GSC on SWB noisy speech (only on active unvoiced SWB noisy speech segments) */
        st->GSC_noisy_speech_fx = 0;
        move16();
        test();
        test();
        test();
        test();
        test();
        test();
        IF ( sub(vad_flag,1) == 0 && L_sub(st->total_brate_fx,ACELP_13k20) >= 0 && L_sub(st->total_brate_fx,ACELP_24k40) < 0 &&
             sub(st->lp_noise_fx,3072) > 0  && *sp_aud_decision1 == 0 && sub(st->bwidth_fx,SWB) >= 0 &&
             sub(st->coder_type_raw_fx,UNVOICED) == 0 )
        {
            st->GSC_noisy_speech_fx = 1;
            move16();
        }

        /* Select AUDIO frames */
        test();
        test();
        test();
        test();
        IF ( sub(st->codec_mode,MODE1) == 0 && ( *sp_aud_decision2 || st->GSC_noisy_speech_fx ) )
        {
            *coder_type = AUDIO;
            move16();
            st->noise_lev_fx = NOISE_LEVEL_SP0;
            move16();
        }

    }

    return;
}

/*---------------------------------------------------------------------*
 * sp_mus_classif_gmm_fx()
 *
 * Speech/music classification based on GMM model
 *---------------------------------------------------------------------*/

static Word16 sp_mus_classif_gmm_fx(   /* o  : decision flag (1-music, 0-speech or noise)      */
    Encoder_State_fx *st_fx,         /* i/o: state structure                                 */
    const Word16 localVAD,
    const Word16 pitch[3],             /* i  : open-loop pitch estimate in three subframes    Q0   */
    const Word16 voicing[3],           /* i  : voicing estimate in three subframes            Q15  */
    const Word16 lsp_new[M],           /* i  : LSPs in current frame                          Q15  */
    const Word16 cor_map_sum,          /* i  : correlation map sum (from multi-harmonic anal.)Q8   */
    const Word32 epsP[M+1],            /* i  : LP prediciton error                            Q_esp */
    const Word32 PS[],                 /* i  : energy spectrum                                Q_new+Qscale-2 */
    Word16 non_sta,              /* i  : unbound non-stationarity for sp/mus classifier */
    Word16 relE,                 /* i  : relative frame energy                          */
    Word16 *voi_fv,              /* o  : scaled voicing feature                          */
    Word16 *cor_map_sum_fv,      /* o  : scaled correlation map feature                  */
    Word16 *LPCErr,              /* o  : scaled LP prediction error feature              */
    Word16 Q_esp                 /* i  : scaling of epsP */
    ,Word16  *high_lpn_flag_ptr       /* o  :    noise log prob flag for NOISE_EST        */
)
{
    Word16 i, k, p, dec, vad;

    Word16 lsp[M], FV[N_FEATURES], *pFV = FV;
    const Word32 *pSF_a;
    const Word16 *pSF_m;
    Word16 lsf2acos_fact, wrelE, dlp, wdrop, wght;

    Word32 mx;
    Word32 sum_PS;
    Word16 ftmp, tmp16;
    Word16  xm[N_FEATURES];
    Word16 lps, lpm;
    Word16 lpn;
    Word16 e_tmp, f_tmp;
    Word32 L_tmp;
    Word16 exp1;
    Word32 ps_sta;
    Word32 ps_diff;
    Word16 ps_diff_16;
    Word32 dPS[128], PS_norm[128];
    Word32 lepsP1;
    Word32 max_s=0, max_m=0, py_s, py_m;
    Word32 max_n, py_n;  /* pyn */
    Word16 ishift[12] = {8,0,2,2,2,2,2,1,0,2,2,1};
    Word16 tmp;
    Word16 tmp1,tmp2,exp2,scale,exp3;

    /*------------------------------------------------------------------*
     * Initialization
     *------------------------------------------------------------------*/

    vad = localVAD;
    move16();

    /*------------------------------------------------------------------*
     * Preparation of the feature vector
     *------------------------------------------------------------------*/

    /* [0] OL pitch Q0 */
    /*(float)(pitch[0] + pitch[1] + pitch[2]) / 3.0f;*/
    L_tmp = L_mult(pitch[0], 10923);
    L_tmp = L_mac(L_tmp, pitch[1], 10923);
    L_tmp = L_mac(L_tmp, pitch[2], 10923);

    test();
    IF ( sub(st_fx->tc_cnt_fx,1) == 0 || sub(st_fx->tc_cnt_fx,2) == 0 )
    {
        *pFV++ = pitch[2];
        move16();
    }
    ELSE
    {
        *pFV++ = round_fx(L_tmp);
    }

    /* [1] voicing Q15 */
    /*(float)(voicing[0] + voicing[1] + voicing[2]) / 3.0f*/
    test();
    IF ( sub(st_fx->tc_cnt_fx,1) == 0 || sub(st_fx->tc_cnt_fx,2) == 0 )
    {
        *pFV++ = voicing[2];
        move16();
    }
    ELSE
    {
        L_tmp = L_mult(voicing[0], 10923);
        L_tmp = L_mac(L_tmp, voicing[1], 10923);
        L_tmp = L_mac(L_tmp, voicing[2], 10923);
        *pFV++ = round_fx(L_tmp);
    }

    /* [2,3,4,5,6] LSFs Q15*/
    Copy( lsp_new, lsp, M );
    lsf2acos_fact = 25735;
    move16();   /* PI/6400 -> Q27 */

    /*ftmp = (float)acos(lsp[1...5]);*/
    /**pFV++ = ftmp + st->last_lsp[1...5];*/
    /*st->last_lsp[1...5] = ftmp;*/
    FOR(i= 1; i < M_LSP_SPMUS; i++)
    {
        L_tmp = sub_lsp2lsf_fx(lsp[i]);
        tmp16 = round_fx(L_shl(L_mult0(extract_l(L_tmp),lsf2acos_fact),2));
        *pFV++ = add(tmp16,st_fx->last_lsp_fx[i]);
        move16();   /*Q13*/
        st_fx->last_lsp_fx[i] = tmp16;
        move16();
    }

    /* [7] cor_map_sum Q8 */
    *pFV++ = round_fx(L_mac(L_mult(cor_map_sum, 16384), st_fx->last_cor_map_sum_fx, 16384));      /* Q8 ->Q7*/
    st_fx->last_cor_map_sum_fx = cor_map_sum;
    move16();

    /* [8] non_sta Q8*/
    *pFV++ = round_fx(L_mac(L_mult(non_sta,16384), st_fx->last_non_sta_fx, 16384));      /* Q8 -> Q7 */
    st_fx->last_non_sta_fx = non_sta;
    move16();

    /* [9] epsP Q10 */
    IF ( sub(st_fx->bwidth_fx,NB) == 0)
    {
        *pFV++ = -1687;
        move16();    /*Q10*/
    }
    ELSE
    {
        /*lepsP1 = (float)log(epsP[1] + 1e-5f);*/
        IF(epsP[1] != 0)
        {
            e_tmp = norm_l(epsP[1]);
            f_tmp = Log2_norm_lc(L_shl(epsP[1],e_tmp));
            e_tmp = sub(30,add(e_tmp,Q_esp));
            lepsP1 = Mpy_32_16(e_tmp, f_tmp, 22713); /* Q16 */ /* 22713 = ln(2) in Q15 */
        }
        ELSE
        {
            lepsP1 = L_deposit_l(0);
        }

        /*ftmp = (float)log(epsP[13]);*/
        IF(epsP[13] != 0)
        {
            e_tmp = norm_l(epsP[13]);
            f_tmp = Log2_norm_lc(L_shl(epsP[13],e_tmp));
            e_tmp = sub(30,add(e_tmp,Q_esp));
            L_tmp = Mpy_32_16(e_tmp, f_tmp, 22713); /* Q16 */ /* 22713 = ln(2) in Q15 */
        }
        ELSE
        {
            L_tmp = L_deposit_l(0);
        }

        /*ftmp = (float)log(epsP[13]) - lepsP1;*/
        L_tmp = L_sub(L_tmp, lepsP1); /*Q16 */
        ftmp = round_fx(L_shl(L_tmp,10)); /*Q10 */

        /**pFV++ = ftmp + st->past_epsP2;*/
        *pFV++ = add(ftmp,st_fx->past_epsP2_fx);
        move16(); /*Q10 */

        /*st->past_epsP2 = ftmp;*/
        st_fx->past_epsP2_fx = ftmp;
        move16(); /*Q10 */
    }

    /* calculation of differential normalized power spectrum */
    sum_PS = L_deposit_l(0);
    FOR ( i = LOWEST_FBIN; i < HIGHEST_FBIN; i++ )
    {
        sum_PS = L_add(sum_PS,PS[i]);
    }
    exp1 = norm_l(sum_PS);
    tmp1 = round_fx(L_shl(sum_PS,exp1));
    exp1 = sub(30,exp1);

    FOR ( i = LOWEST_FBIN; i < HIGHEST_FBIN; i++ )
    {
        /*PS_norm[i] = PS[i] / sum_PS;*/
        /*dPS[i] = (float)fabs(PS_norm[i] - st->past_PS[i]);*/
        exp2 = norm_l(PS[i]);
        tmp2 = round_fx(L_shl(PS[i],exp2));
        exp2 = sub(30,exp2);

        scale = shr(sub(tmp1, tmp2), 15);
        tmp2 = shl(tmp2, scale);
        exp2 = sub(exp2, scale);

        exp3 = sub(exp1,exp2);

        tmp = div_s(tmp2, tmp1);             /*Q(15+exp3) */
        PS_norm[i] = L_shl(tmp,sub(10,exp3));
        move32(); /*Q25 */
        dPS[i] = L_abs(L_sub(PS_norm[i],st_fx->past_PS_fx[i]));
        move32();   /*Q25 */
    }

    /* [10] ps_diff (spectral difference) Q10*/
    ps_diff = 0;
    move16();
    FOR ( i = LOWEST_FBIN; i < HIGHEST_FBIN; i++ )
    {
        /*ps_diff += dPS[i];*/
        ps_diff = L_add(ps_diff,dPS[i]); /*Q25*/
    }

    /*ps_diff = (float)log(ps_diff + 1e-5f);*/
    IF( ps_diff != 0 )
    {
        e_tmp = norm_l(ps_diff);
        f_tmp = Log2_norm_lc(L_shl(ps_diff,e_tmp));
        e_tmp = sub(30-25,e_tmp);
        ps_diff = Mpy_32_16(e_tmp, f_tmp, 22713);/* Q16 */ /* 22713 = ln(2) in Q15 */
        ps_diff_16 = round_fx(L_shl(ps_diff,10)); /*Q10 */
    }
    ELSE
    {
        ps_diff_16 = -11789;
        move16(); /*Q10 */
    }

    *pFV++ = add(ps_diff_16, st_fx->past_ps_diff_fx);
    move16();/*Q10 */
    st_fx->past_ps_diff_fx = ps_diff_16;
    move16(); /*Q10 */

    /* [11] ps_sta (spectral stationarity) Q11 */
    ps_sta = 0;
    move16();
    FOR ( i = LOWEST_FBIN; i < HIGHEST_FBIN; i++ )
    {
        /*mx = PS_norm[i] > st->past_PS[i] ? PS_norm[i] : st->past_PS[i];*/
        IF (L_sub(PS_norm[i],st_fx->past_PS_fx[i]) > 0)
        {
            mx = PS_norm[i];
            move16();  /*Q25 */
        }
        ELSE
        {
            mx = st_fx->past_PS_fx[i];
            move16();  /*Q25 */
        }

        /*ps_sta += mx / (dPS[i] + 1e-5f);*/
        IF( !dPS[i] )
        {
            ps_sta = L_add(ps_sta,L_shr(mx,9));  /*Q16 */
        }
        ELSE
        {
            exp1 = norm_l(L_add(dPS[i],336));
            tmp1 = round_fx(L_shl(L_add(dPS[i],336),exp1));
            exp1 = sub(30,exp1);

            exp2 = norm_l(mx);
            tmp2 = round_fx(L_shl(mx,exp2));
            exp2 = sub(30,exp2);

            scale = shr(sub(tmp1, tmp2), 15);
            tmp2 = shl(tmp2, scale);
            exp2 = sub(exp2, scale);

            exp3 = sub(exp1,exp2);
            move16();

            tmp = div_s(tmp2, tmp1);        /*Q(15+exp3) */
            L_tmp = L_shl(tmp,sub(1,exp3)); /*Q16 */
            ps_sta = L_add(ps_sta,L_tmp);   /*Q16 */
        }
    }

    /**pFV++ = (float)log(ps_sta + 1e-5f);*/
    ps_sta = L_add(ps_sta, 336);
    e_tmp = norm_l(ps_sta);
    f_tmp = Log2_norm_lc(L_shl(ps_sta,e_tmp));
    e_tmp = sub(30-16,e_tmp);
    L_tmp = Mpy_32_16(e_tmp, f_tmp, 22713); /* Q16 */ /* 22713 = ln(2) in Q15 */
    *pFV++ = round_fx(L_shl(L_tmp,11));	 /*Q11 */

    /* update PS vector */
    Copy32( &PS_norm[LOWEST_FBIN], &st_fx->past_PS_fx[LOWEST_FBIN], HIGHEST_FBIN-LOWEST_FBIN );

    /*------------------------------------------------------------------*
     * Scaling of the feature vector
     *------------------------------------------------------------------*/

    /* FV[0]      -> Q0 */
    /* FV[1...6]  -> Q13*/
    /* FV[7,8]    -> Q7 */
    /* FV[9,10]   -> Q10 */
    /* FV[11]     -> Q11 */


    pFV = FV;
    move16();
    IF ( sub(st_fx->bwidth_fx,NB) == 0 )
    {
        pSF_m = SF_8k_mult_fx;
        pSF_a = SF_8k_add_fx;
    }
    ELSE
    {
        pSF_m = SF_mult_fx;
        pSF_a = SF_add_fx;
    }

    FOR ( i=0; i<N_FEATURES; i++)
    {
        /**pFV = pSF[0] * *pFV + pSF[1];*/
        *pFV = round_fx(L_shl(L_mac(pSF_a[i], *pFV, pSF_m[i]), ishift[i]));
        pFV++;
    }

    *voi_fv = FV[1];
    move16();
    *cor_map_sum_fv = FV[7];
    move16();
    *LPCErr = FV[9];
    move16();


    /*------------------------------------------------------------------*
     * Calculation of posterior probability
     * Log-probability
     *------------------------------------------------------------------*/

    max_s = L_add(MIN_32, 0);
    max_m = L_add(MIN_32, 0);
    /* pyn = 1e-5f;*/
    max_n = L_add(MIN_32, 0);



    FOR ( k = 0; k < N_MIXTURES; k++ )
    {
        /* for each mixture, calculate the probability of speech or noise and the probability of music */
        /* active frames - calculate the probability of speech */
        FOR ( p = 0; p < N_FEATURES; p++ )
        {
            /* xm[p] = FV[p] - m_speech[k*N_FEATURES+p];*/
            xm[p] = sub(FV[p],m_speech_fx[k*N_FEATURES+p]);
            move16();/*Q15 */
        }

        /*py = lvm_speech[k] + dot_product_mat(xm, &invV_speech[k*N_FEATURES*N_FEATURES], N_FEATURES );*/
        L_tmp = dot_product_mat_fx(xm, &invV_speech_fx[k*N_FEATURES*N_FEATURES], N_FEATURES);  /*Q10 */
        py_s = L_add(lvm_speech_fx[k],L_tmp);	/*Q10 */
        max_s = L_max(py_s, max_s);


        /* pys += (float)exp(py);  */

        /* inactive frames - calculate the probability of noise */
        FOR ( p = 0; p < N_FEATURES; p++ )
        {
            /*xm[p] = FV[p] - m_noise[k*N_FEATURES+p];*/
            xm[p] = sub(FV[p], m_noise_fx[k*N_FEATURES+p]);
            move16();/*Q15 */
        }

        /*py = lvm_noise[k] + dot_product_mat(xm, &invV_noise[k*N_FEATURES*N_FEATURES], N_FEATURES );*/
        L_tmp = dot_product_mat_fx(xm, &invV_noise_fx[k*N_FEATURES*N_FEATURES], N_FEATURES);  /*Q10 */
        /* pyn += (float)exp(py); */
        py_n = L_add(lvm_noise_fx[k],L_tmp); /*Q10 */
        max_n = L_max(py_n, max_n);


        /* either active or inactive frames - calculate the probability of music */
        FOR ( p = 0; p < N_FEATURES; p++ )
        {
            /*xm[p] = FV[p] - m_music[k*N_FEATURES+p];*/
            xm[p] = sub(FV[p], m_music_fx[k*N_FEATURES+p]);
            move16();	/*Q15 */
        }

        /*py = lvm_music[k] + dot_product_mat(xm, &invV_music[k*N_FEATURES*N_FEATURES], N_FEATURES );*/
        L_tmp = dot_product_mat_fx(xm, &invV_music_fx[k*N_FEATURES*N_FEATURES], N_FEATURES); /*Q10 */
        py_m = L_add(lvm_music_fx[k],L_tmp); /*Q10 */
        max_m = L_max(py_m, max_m);

        /*pym += (float)exp(py);#######*/
    }

    /* calculate log-probability */
    /*log(0.0001)-0.5f * N_FEATURES * LOG_PI2 in Q9 */
    lps = extract_h(L_shl(L_sub(max_s,LOG_PROB_CONST),16-1)); /*Q9 */
    lps = s_max( lps, -10832 );

    lpm = extract_h(L_shl(L_sub(max_m,LOG_PROB_CONST),16-1)); /*Q9 */
    lpm = s_max( lpm, -10832 );
    /*
     lpn = (float)log(pyn) - 0.5f * N_FEATURES * (float)log(2*PI);
    */
    lpn = extract_h(L_shl(L_sub(max_n,LOG_PROB_CONST),16-1)); /*Q9 */
    lpn = s_max( lpn, -10832 );

    *high_lpn_flag_ptr = 0;
    move16();
    test();
    if (      (sub(lpn, lps) > 0)
              &&  (sub(lpn, lpm) > 0)  )
    {
        *high_lpn_flag_ptr = 1;
        move16();
    }


    IF ( !vad )
    {
        /* increase log-probability of noise */
        /* lps  =  lpn * 1.2f; */
        lps = add(lpn,mult_r(6554,lpn));    /* Q9 */
    }

    st_fx->lpm_fx = lpm;
    move16();
    st_fx->lps_fx = lps;
    move16();

    /* determine HQ GENERIC speech class */
    st_fx->hq_generic_speech_class_fx = 0;
    move16();
    if( sub(lps,add(lpm,256)) > 0 )
    {
        st_fx->hq_generic_speech_class_fx = 1;
        move16();
    }


    /*------------------------------------------------------------------*
     * State machine (sp_mus_state < 0 .. inactive, > 0 .. entry, = 0 .. active )
     *------------------------------------------------------------------*/

    IF ( vad )
    {
        test();
        test();
        test();
        IF ( sub(relE,-20*256) < 0 || (sub(lps,-5*512) <= 0 && sub(lpm,-5*512) <= 0) )
        {
            IF ( st_fx->sp_mus_state_fx > 0 )
            {
                if ( sub(st_fx->sp_mus_state_fx,HANG_LEN) < 0 )
                {
                    /* energy is too low but we are in entry period -> reset the inactive counter to allow new entry later */
                    st_fx->inact_cnt_fx = 0;
                    move16();
                }

                /* energy is too low -> we are going to instable state */
                st_fx->sp_mus_state_fx = 0;
                move16();
            }
            ELSE IF ( sub(st_fx->sp_mus_state_fx,-HANG_LEN) > 0 )
            {
                /* energy is still too low -> we are still in instable state */
                st_fx->sp_mus_state_fx = sub(st_fx->sp_mus_state_fx,1);
            }
        }
        ELSE IF ( st_fx->sp_mus_state_fx <= 0 )
        {
            IF ( st_fx->inact_cnt_fx == 0 )
            {

                st_fx->sp_mus_state_fx = 1;
                move16();
            }
            ELSE
            {

                st_fx->sp_mus_state_fx = HANG_LEN;
                move16();
            }

            st_fx->inact_cnt_fx = 12;
            move16();
        }
        ELSE IF ( st_fx->sp_mus_state_fx > 0 && sub(st_fx->sp_mus_state_fx,HANG_LEN) < 0 )
        {
            /* we are inside an entry period -> increment the counter of entry frames */
            st_fx->sp_mus_state_fx = add(st_fx->sp_mus_state_fx,1);
        }

        test();
        if ( st_fx->sp_mus_state_fx < 0 && st_fx->inact_cnt_fx > 0 )
        {
            st_fx->inact_cnt_fx = sub(st_fx->inact_cnt_fx,1);
        }
    }
    ELSE
    {
        test();
        IF ( st_fx->sp_mus_state_fx > 0 && sub(st_fx->sp_mus_state_fx,HANG_LEN) < 0 )
        {
            st_fx->inact_cnt_fx = 0;
            move16();
        }
        ELSE IF ( st_fx->inact_cnt_fx > 0 )
        {
            st_fx->inact_cnt_fx = sub(st_fx->inact_cnt_fx,1);
        }

        test();
        IF ( st_fx->sp_mus_state_fx > 0 && sub(st_fx->sp_mus_state_fx,HANG_LEN) < 0 )
        {

            st_fx->sp_mus_state_fx = -HANG_LEN;
            move16();
        }
        ELSE IF ( st_fx->sp_mus_state_fx > 0 )
        {

            st_fx->sp_mus_state_fx = -1;
            move16();
        }
        ELSE IF ( sub(st_fx->sp_mus_state_fx,-HANG_LEN) > 0 )
        {
            /* we are in inactive state */
            st_fx->sp_mus_state_fx = sub(st_fx->sp_mus_state_fx,1);
        }
    }

    /*------------------------------------------------------------------*
     * Decision without hangover
     * Weighted decision
     *------------------------------------------------------------------*/

    /* decision without hangover (0 - speech/noise, 1 - music) */
    logic16();
    dec = sub(lpm,lps) > 0;
    move16();
    dlp = sub(lpm,lps);     /*Q9*/

    IF ( !vad )
    {
        dec = 0;
        move16();
        dlp = 0;
        move16();
    }

    /* calculate weight based on relE (close to 0.01 in low-E regions, close to 1 in high-E regions) */
    /*wrelE = 1.0f + relE/15;*/
    wrelE = add(2048, mult_r(relE,17476));   /* 1/15 in Q18 -> 17476 result in Q11 */


    wrelE = s_min(wrelE, 2048);
    wrelE = s_max(wrelE, 20);

    /* calculate weight based on drops of dlp (close to 1 during sudden drops of dlp, close to 0 otherwise) */
    test();
    IF ( dlp < 0 && sub(dlp,st_fx->past_dlp_fx[0]) < 0 )
    {
        IF ( st_fx->past_dlp_fx[0] > 0 )
        {
            st_fx->wdrop_fx = negate(dlp);   /*Q9*/
        }
        ELSE
        {
            st_fx->wdrop_fx = add(st_fx->wdrop_fx, sub(st_fx->past_dlp_fx[0], dlp));  /*Q9*/
        }
    }
    ELSE
    {
        st_fx->wdrop_fx = 0;
        move16();
    }

    /*wdrop = st->wdrop/20;*/
    wdrop = mult_r(st_fx->wdrop_fx, 26214);   /*Q9*Q19->Q13*/
    wdrop = s_min(wdrop,8192); /* limitation [0.1,1] Q13 */
    wdrop = s_max(wdrop,819);

    /* combine weights into one */
    /*wght = wrelE * wdrop;*/
    wght = mult_r(wrelE, wdrop);   /* Q11*Q13 -> Q9*/
    wght = s_max(wght,5);

    /* calculate weighted decision */
    /*st->wdlp_0_95_sp = wght * dlp + (1 - wght) * st->wdlp_0_95_sp;*/
    /*                 =  Q9 * Q9 + (Q9-Q9)*Q9 */
    L_tmp = L_mac(L_mult(wght, dlp), sub(512, wght), st_fx->wdlp_0_95_sp_fx);
    st_fx->wdlp_0_95_sp_fx = round_fx(L_shl(L_tmp, 6));

    if ( sub(st_fx->sp_mus_state_fx,-HANG_LEN) == 0 )
    {
        st_fx->wdlp_0_95_sp_fx = 0;
        move16();
    }

    /*------------------------------------------------------------------*
     * Final speech/music decision
     *------------------------------------------------------------------*/

    test();
    test();
    IF ( !vad && sub(st_fx->sp_mus_state_fx,-HANG_LEN) == 0 )
    {
        /* inactive state */
        dec = 0;
        move16();
    }
    ELSE IF ( st_fx->sp_mus_state_fx <= 0 )
    {
        /* transition from active to inactive state or instable state */
        dec = st_fx->past_dec_fx[0];
        move16();
    }
    ELSE IF ( st_fx->sp_mus_state_fx > 0 && sub(st_fx->sp_mus_state_fx,HANG_LEN) < 0 )
    {
        /* entry state -> final decision is calculated based on weighted average of past non-binary decisions */
        L_tmp = L_mult(w_fx[st_fx->sp_mus_state_fx-1][0], dlp);   /*Q15*Q9 */

        /*ftmp += dotp( &w[st_fx->sp_mus_state_fx-1][1], st_fx->past_dlp_fx, HANG_LEN-1 );*/
        L_tmp = L_add(L_tmp, Dot_product( &w_fx[st_fx->sp_mus_state_fx-1][1], st_fx->past_dlp_fx, HANG_LEN-1 ));
        logic16();
        move16();

        /*dec = ftmp > 2.0f;*/
        dec = L_sub(L_tmp, 2*(1<<25))>0;
    }
    ELSE
    {
        /* stable active state */
        test();
        test();
        test();
        test();
        IF ( st_fx->wdlp_0_95_sp_fx > 0 && st_fx->past_dec_fx[0] == 0 && st_fx->past_dec_fx[1] == 0 && st_fx->past_dec_fx[2] == 0 )
        {
            /* switching from speech to music */
            dec = 1;
            move16();
        }
        ELSE IF ( st_fx->past_dec_fx[0] == 1 && st_fx->wdlp_0_95_sp_fx < 0 )
        {
            /* switching from music to speech */
            dec = 0;
            move16();
        }
        ELSE
        {
            dec = st_fx->past_dec_fx[0];
            move16();
        }
    }


    /*------------------------------------------------------------------*
     * Updates
     *------------------------------------------------------------------*/

    /* update the buffer of past non-binary decisions */
    Copy( &st_fx->past_dlp_fx[0], &st_fx->past_dlp_fx[1], HANG_LEN-1 );
    st_fx->past_dlp_fx[0] = dlp;
    move16();

    /* update the buffer of past binary decisions */
    Copy( &st_fx->past_dec_fx[0], &st_fx->past_dec_fx[1], HANG_LEN-1 );
    st_fx->past_dec_fx[0] = dec;
    move16();

    return dec;
}


/*---------------------------------------------------------------------*
 * sp_mus_classif_2nd_fx()
 *
 * 2nd stage speech/music classifier (convert music to speech for onsets)
 *---------------------------------------------------------------------*/

static void sp_mus_classif_2nd_fx(
    Encoder_State_fx *st,                   /* i/o: Encoder state structure                */
    const Word16 sp_aud_decision1,          /* i  : 1st stage decision flag                */
    Word16 *sp_aud_decision2,         /* i/o: 2nd stage decision flag                */
    const Word16 pitch[3],                  /* i  : open-loop pitch estimate in three subframes */
    const Word16 Etot,                      /* i  : total frame energy                     */
    Word16 *coder_type,               /* i/o: coder type                             */
    Word16 *attack_flag,              /* i/o: attack flag (GSC or TC)                */
    const Word16 *inp,                      /* i  : input signal                           */
    const Word16 Qx,
    const Word16 localVAD,
    const Word16 vad_flag
)
{
    Word16 attack;

    /* initialization */
    *attack_flag = 0;
    move16();

    /* signal stability estimation */
    stab_est_fx( Etot, st->gsc_lt_diff_etot_fx, &st->gsc_mem_etot_fx, &st->gsc_last_bfi_count_fx, 0,
                 &st->gsc_nb_thr_3_fx, &st->gsc_nb_thr_1_fx, st->gsc_thres_fx, &st->gsc_last_music_flag_fx, vad_flag );

    /* calculate variance of correlation */
    var_cor_calc_fx( st->old_corr_fx, &st->mold_corr_fx, st->var_cor_t_fx, &st->high_stable_cor_fx );

    /* attack detection */
    attack = attack_det_fx( inp, Qx, st->clas_fx, localVAD, *coder_type
                            , st->total_brate_fx);

    test();
    test();
    test();
    test();
    test();
    test();
    IF( sub(sp_aud_decision1,1) == 0 )
    {
        test();
        test();
        test();
        IF( sub(st->ener_RAT_fx,5898) < 0 && sub (st->lt_dec_thres_fx,7680) > 0 )
        {
            *sp_aud_decision2 = 0;
            move16();
        }
        ELSE IF( sub(st->high_stable_cor_fx, 1) == 0 && sub(pitch[0], 130) >= 0 )
        {
            /* prevent GSC in highly correlated signal with low energy variation */
            /* this is basically a patch against bassoon-type of music */
            *sp_aud_decision2 = 0;
            move16();

            test();
            IF( sub(st->codec_mode,MODE1) == 0 && sub(*coder_type,TRANSITION) == 0 )
            {
                *coder_type = GENERIC;
                move16();
            }
        }
        ELSE IF( sub(st->gsc_lt_diff_etot_fx[MAX_LT-1],1152) > 0 &&
                 sub(sub(st->gsc_lt_diff_etot_fx[MAX_LT-1], st->gsc_lt_diff_etot_fx[MAX_LT-2]),2560) > 0 )  /* 10.0f in Q8 */
        {
            IF ( sub(st->tc_cnt_fx,1) == 0 )
            {
                *sp_aud_decision2 = 0;
                move16();

                IF( sub(st->codec_mode,MODE1) == 0 )
                {
                    *coder_type = TRANSITION;
                    move16();
                }
            }
            ELSE
            {
                IF( sub(attack, ATT_3LSUB_POS) >= 0 )
                {
                    /* do TC coding if attack is located in the last subframe */
                    *sp_aud_decision2 = 0;
                    move16();
                    *attack_flag = 1;
                    move16();
                    IF( sub(st->codec_mode,MODE1) == 0 )
                    {
                        *coder_type = TRANSITION;
                        move16();
                    }
                }
                ELSE IF( sub(attack,ATT_SEG_LEN/2) >= 0 )
                {
                    /* do GSC coding if attack is located after the first quarter of the first subframe */
                    /* (pre-echo will be treated at the decoder side) */
                    *attack_flag = 1;
                    move16();
                }
            }
        }
    }
    ELSE IF( sub(localVAD,1) == 0 && sub(*coder_type,GENERIC) == 0 &&
             ( (sub(attack,ATT_3LSUB_POS) >= 0 && L_sub(st->total_brate_fx,ACELP_24k40) < 0) ||
               (sub(attack,ATT_3LSUB_POS_16k) >= 0 && L_sub(st->total_brate_fx,ACELP_24k40) >= 0 && L_sub(st->total_brate_fx,ACELP_48k) < 0) )
           )
    {
        /* do TC coding if attack is located in the last subframe */
        *attack_flag = 1;
        move16();
        IF( sub(st->codec_mode,MODE1) == 0 )
        {
            *coder_type = TRANSITION;
            move16();
        }
    }

    return;
}


/*---------------------------------------------------------------------*
 * var_cor_calc_fx()
 *
 * Calculate variance of correlation
 *---------------------------------------------------------------------*/

static void var_cor_calc_fx(
    const Word16 old_corr,
    Word16 *mold_corr,
    Word16 var_cor_t[],
    Word16 *high_stable_cor
)
{
    Word16 i, var_cor;

    /* update buffer of old correlation values */
    FOR( i = VAR_COR_LEN-1; i > 0; i-- )
    {
        var_cor_t[i] = var_cor_t[i-1];     /*Q11*/  move16();
    }
    var_cor_t[i] = old_corr;
    move16();

    /* calculate variance of correlation */
    var_cor = var_fx( var_cor_t, 11, VAR_COR_LEN );

    *high_stable_cor = 0;
    move16();
    test();
    IF( sub(*mold_corr,26214) > 0 && sub(var_cor,2) < 0 )
    {
        *high_stable_cor = 1;
        move16();
    }

    /* update average correlation */
    /*st->mold_corr = 0.1f * st->old_corr + 0.9f * st->mold_corr;*/
    *mold_corr = mac_r(L_mult(3277,old_corr),29491,*mold_corr); /*Q15 */

    return;
}

/*---------------------------------------------------------------------*
 * attack_det_fx()
 *
 * Attack detection
 *---------------------------------------------------------------------*/

static Word16 attack_det_fx(       /* o  : attack flag                            */
    const Word16 *inp,                 /* i  : input signal                           */
    const Word16 Qx,
    const Word16 last_clas,            /* i  : last signal clas                       */
    const Word16 localVAD,
    const Word16 coder_type            /* i  : coder type                             */
    ,const Word32 total_brate           /* i  : total bit-rate                         */
)
{
    Word16 i, j, tmp, tmp1, attack, exp1;
    Word32 L_tmp, etmp, etmp2, finc[ATT_NSEG];
    Word16 att_3lsub_pos;

    att_3lsub_pos = ATT_3LSUB_POS;
    move16();
    if( L_sub(total_brate,ACELP_24k40) >= 0 )
    {
        att_3lsub_pos = ATT_3LSUB_POS_16k;
        move16();
    }

    /* compute energy per section */
    FOR( i=0; i<ATT_NSEG; i++ )
    {
        L_tmp = L_mult0(inp[i*ATT_SEG_LEN],inp[i*ATT_SEG_LEN]); /*2*Qx */

        FOR(j=1; j<ATT_SEG_LEN; j++)
        {
            L_tmp = L_mac0(L_tmp,inp[i*ATT_SEG_LEN+j],inp[i*ATT_SEG_LEN+j]); /*2*Qx */
        }

        finc[i] = L_tmp;
        move32();
    }

    attack = maximum_32_fx( finc, ATT_NSEG, &etmp );
    test();
    IF( sub(localVAD,1) == 0 && sub(coder_type,GENERIC) == 0 )
    {
        /*----------------------------------------------------------------------*
         * Detect if there is a strong onset in the last subframe
         * - if detected, TC is used to better code the onset
         *----------------------------------------------------------------------*/

        /* compute mean energy in the first three subframes */
        exp1 = norm_s(att_3lsub_pos);
        tmp = div_s(shl(1,sub(14,exp1)),att_3lsub_pos); /*Q(29-exp1) */

        L_tmp = L_shr(finc[0],Qx); /*Qx */

        FOR(i=1; i<att_3lsub_pos; i++)
        {
            L_tmp = L_add(L_tmp,L_shr(finc[i],Qx)); /*Qx */
        }
        L_tmp = Mult_32_16(L_tmp,tmp); /*Q(14-exp1+Qx) */
        etmp = L_shl(L_tmp,sub(exp1,14)); /*Qx */

        tmp1 = sub(ATT_NSEG,attack);
        exp1 = norm_s(tmp1);
        tmp = div_s(shl(1,sub(14,exp1)),tmp1); /*Q(29-exp1) */

        L_tmp = L_shr(finc[attack],Qx); /*Qx */
        FOR(i=1; i<tmp1; i++)
        {
            L_tmp = L_add(L_tmp,L_shr(finc[i+attack],Qx)); /*Qx */
        }
        L_tmp = Mult_32_16(L_tmp,tmp); /*Q(14-exp1+Qx) */
        etmp2 = L_shl(L_tmp,sub(exp1,14)); /*Qx */

        /* and compare them */
        if( L_sub(etmp,L_shr(etmp2,3)) > 0 )
        {
            /* stop, if the attack is not sufficiently strong */
            attack = 0;
            move16();
        }

        test();
        if( sub(last_clas,VOICED_CLAS) == 0 && L_sub(L_add(L_shl(etmp,4),L_shl(etmp,2)),etmp2) > 0 )
        {
            /* stop, if the signal was voiced and the attack is not sufficiently strong */
            attack = 0;
            move16();
        }

        /* compare also wrt. other sections (reduces a misclassification) */
        IF( attack > 0 )
        {
            etmp2 = L_add(finc[attack], 0);
            etmp = Mult_32_16(etmp2, 16384); /* etmp2 / 2.0  = (etmp2*0.5) */
            FOR( i=2; i<ATT_3LSUB_POS-2; i++ )
            {
                IF( L_sub(finc[i],etmp) > 0 )
                {
                    attack = 0;
                    move16();
                    BREAK;
                }
            }
        }
    }
    ELSE IF( attack > 0 )
    {
        etmp2 = L_add(finc[attack], 0);
        etmp = Mult_32_16(etmp2, 25206); /* etmp2 / 1.3  = (etmp2*0.76923) */
        FOR( i=2; i<att_3lsub_pos-2; i++ )
        {
            /*if( i != attack && finc[i] * 1.3f > etmp2 ) -> finc[i] > (etmp2*0.76923) */
            test();
            IF( sub(i,attack) != 0 && L_sub(finc[i],etmp) > 0 )
            {
                attack = 0;
                move16();
                BREAK;
            }
        }
    }

    return attack;
}

/*---------------------------------------------------------------------*
 * mode_decision_fx()
 *
 *
 *---------------------------------------------------------------------*/

static Word16 mode_decision_fx(
    Encoder_State_fx *st,                     /* i  : endoer state structure                          */
    Word16 len,                         /* i  : buffering status                                */
    Word16 *dec_mov,                    /* i/o: moving average of classifier decision           Q15*/
    Word16 *buf_flux,                   /* i  : buffer storing spectral energy fluctuation      Q7*/
    Word16 *buf_epsP_tilt,              /* i  : buffer storing LP prediciton error tilt         Q15*/
    Word16 *buf_pkh,                    /* i  : buffer storing highband spectral peakiness      Q1*/
    Word16 *buf_cor_map_sum,            /* i  : buffer storing correlation map sum              Q8*/
    Word16 *buf_Ntonal,                 /* i  : buffer storing No.of 1st spectral tone          Q0*/
    Word16 *buf_Ntonal2,                /* i  : buffer storing No.of 2nd spectral tone          Q0*/
    Word16 *buf_Ntonal_lf,              /* i  : buffer storing low band spectral tone ratio     Q0*/
    Word16 *buf_dlp                     /* i  : buffer storing log probability diff between speech and music Q9*/
)
{
    Word16 mode;
    Word16 i;
    Word16 voiced_cnt;
    Word16 M_pkh;
    Word16 M_cor_map_sum;
    Word16 M_Ntonal;
    Word16 M_flux;
    Word32 V_epsP_tilt;
    Word16 lf_Ntonal_ratio;
    Word16 tmp, tmp1;
    Word32 L_tmp;
    Word16 inv_len;
    Word16 j;
    Word16 M_flux10;


    mode = *dec_mov > 16384;
    logic16();
    move16();

    IF ( sub(len,5) <= 0 )
    {
        return (mode);
    }
    ELSE
    {
        IF ( sub(len,10) < 0 )
        {
            inv_len = div_s(1,len); /*Q15 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<len; i++)
            {
                L_tmp = L_add(L_tmp,buf_pkh[BUF_LEN-len+i]); /*Q1 */
            }
            L_tmp = Mult_32_16(L_tmp,inv_len); /*Q1 */
            M_pkh = extract_l(L_tmp); /*Q1 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<len; i++)
            {
                L_tmp = L_add(L_tmp,buf_cor_map_sum[BUF_LEN-len+i]); /*Q8 */
            }
            L_tmp = Mult_32_16(L_tmp,inv_len); /*Q8 */
            M_cor_map_sum = extract_l(L_tmp); /*Q8 */

            tmp = 0;
            move16();
            FOR(i=0; i<len; i++)
            {
                tmp = add(tmp,shl(buf_Ntonal[BUF_LEN-len+i],2)); /*Q2 */
            }
            M_Ntonal = mult_r(tmp,inv_len); /*Q2 */

            V_epsP_tilt = var_fx_32( buf_epsP_tilt+BUF_LEN-len, 15, len ); /*Q31 */

            voiced_cnt = 0;
            move16();
            FOR ( i=9; i>3; i-- )
            {
                if ( buf_dlp[i] > 0 )
                {
                    voiced_cnt = add(voiced_cnt,1);
                }
            }

            test();
            test();
            test();
            test();
            IF ( (sub(M_pkh,2200) > 0 || L_sub(V_epsP_tilt,171799) < 0 || sub(M_cor_map_sum,25600) > 0) && voiced_cnt < 4 )
            {
                mode = 1;
                move16();
            }
            ELSE IF ( sub(M_Ntonal,108) > 0 && voiced_cnt < 4 ) /*27 in Q2 */
            {
                mode = 1;
                move16();
            }
        }
        ELSE
        {
            voiced_cnt = 0;
            move16();
            FOR ( i=0; i<10; i++ )
            {
                if ( buf_dlp[i] > 0 )
                {
                    voiced_cnt = add(voiced_cnt,1);
                }
            }

            inv_len = 3277; /*Q15 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<10; i++)
            {
                L_tmp = L_add(L_tmp,L_shl(buf_flux[BUF_LEN-10+i],2)); /*Q9 */
            }
            L_tmp = Mult_32_16(L_tmp,inv_len); /*Q9 */
            M_flux10 = extract_l(L_tmp); /*Q9 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<10; i++)
            {
                L_tmp = L_add(L_tmp,buf_pkh[BUF_LEN-10+i]); /*Q1 */
            }
            L_tmp = Mult_32_16(L_tmp,inv_len); /*Q1 */
            M_pkh = extract_l(L_tmp); /*Q1 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<10; i++)
            {
                L_tmp = L_add(L_tmp,buf_cor_map_sum[BUF_LEN-10+i]); /*Q8 */
            }
            L_tmp = Mult_32_16(L_tmp,inv_len); /*Q8 */
            M_cor_map_sum = extract_l(L_tmp); /*Q8 */

            V_epsP_tilt = var_fx_32( buf_epsP_tilt+BUF_LEN-10, 15, 10 ); /*Q31 */

            L_tmp = L_deposit_l(0);
            FOR(i=0; i<5; i++)
            {
                L_tmp = L_add(L_tmp,L_shl(buf_flux[BUF_LEN-5+i],2)); /*Q9 */
            }
            L_tmp = Mult_32_16(L_tmp,6554); /*Q9 */
            tmp = extract_l(L_tmp); /*Q9 */

            test();
            test();
            test();
            test();
            test();
            test();
            IF ( (sub(M_flux10,4352) < 0 || (L_sub(V_epsP_tilt,2147484) < 0 && sub(M_flux10,6144) < 0)|| sub(M_pkh,2100) > 0 || sub(M_cor_map_sum,25600) > 0) &&
                 sub(voiced_cnt,3) < 0 && sub(tmp,7680) < 0 )
            {
                mode = 1;
                move16();
                *dec_mov = 32767;
                move16();
                return ( mode );
            }

            test();
            test();
            test();
            test();
            test();
            IF ( sub(M_flux10,8192) > 0 || (sub(M_flux10,7680) > 0 && voiced_cnt > 2) || sub(tmp,9728) > 0 || (sub(buf_flux[59],2560) >= 0 && sub(st->lps_fx,st->lpm_fx) > 0) )
            {
                mode = 0;
                move16();
                *dec_mov = 0;
                move16();
                return ( mode );
            }

            FOR ( i=10; i<len; i++ )
            {
                inv_len = div_s(1,i); /*Q15 */

                L_tmp = L_deposit_l(0);
                FOR(j=0; j<i; j++)
                {
                    L_tmp = L_add(L_tmp,L_shl(buf_flux[BUF_LEN-i+j],2)); /*Q9 */
                }
                L_tmp = Mult_32_16(L_tmp,inv_len); /*Q9 */
                M_flux = extract_l(L_tmp); /*Q9 */

                L_tmp = L_deposit_l(0);
                FOR(j=0; j<i; j++)
                {
                    L_tmp = L_add(L_tmp,buf_pkh[BUF_LEN-i+j]); /*Q1 */
                }
                L_tmp = Mult_32_16(L_tmp,inv_len); /*Q1 */
                M_pkh = extract_l(L_tmp); /*Q1 */

                L_tmp = L_deposit_l(0);
                FOR(j=0; j<i; j++)
                {
                    L_tmp = L_add(L_tmp,buf_cor_map_sum[BUF_LEN-i+j]); /*Q8 */
                }
                L_tmp = Mult_32_16(L_tmp,inv_len); /*Q8 */
                M_cor_map_sum = extract_l(L_tmp); /*Q8 */

                V_epsP_tilt = var_fx_32( buf_epsP_tilt+BUF_LEN-i, 15, i ); /*Q31 */

                test();
                test();
                test();
                test();
                test();
                IF ( ((sub(M_flux,add(6144,mult_r(1638,shl(sub(len,10),9)))) < 0 && sub(M_flux10,7680) < 0) ||
                      L_sub(V_epsP_tilt,L_add(214748,L_shl(L_mult0(19327,(len-10)),1))) < 0 ||
                      sub(M_pkh,sub(2100,extract_l(L_mult0(10,sub(len,10))))) > 0 ||
                      sub(M_cor_map_sum,sub(24320,extract_l(L_mult0(77,sub(len,10))))) > 0) && sub(voiced_cnt,3) < 0 )
                {
                    mode = 1;
                    move16();
                    return( mode );
                }
            }

            IF ( sub(len,BUF_LEN) == 0 )
            {
                tmp = 0;
                move16();
                FOR(i=0; i<len; i++)
                {
                    tmp = add(tmp,shl(buf_Ntonal[i],2)); /*Q2 */
                }
                M_Ntonal = mult_r(tmp,546); /*Q2 */

                tmp = 0;
                move16();
                FOR(i=0; i<len; i++)
                {
                    tmp = add(tmp,buf_Ntonal_lf[i]); /*Q0 */
                }
                tmp1 = 0;
                move16();
                FOR(i=0; i<len; i++)
                {
                    tmp1 = add(tmp1,buf_Ntonal2[i]); /*Q0 */
                }
                lf_Ntonal_ratio = 0;
                move16(); /*Q15 */
                if ( tmp1 != 0 )
                {
                    lf_Ntonal_ratio = div_s(tmp,tmp1); /*Q15 */
                }

                test();
                IF ( sub(M_Ntonal,72) > 0 || sub(lf_Ntonal_ratio,6554) < 0 )
                {
                    mode = 1;
                    move16();
                }
                ELSE IF ( sub(M_Ntonal,4) < 0 )
                {
                    mode = 0;
                    move16();
                }
            }
        }
    }

    return ( mode );
}

/*---------------------------------------------------------------------*
* tonal_dist_fx()
*
*
*---------------------------------------------------------------------*/

static void tonal_dist_fx(
    Word16 *p2v_map,                   /* i  : spectral peakiness map                          Q7*/
    Word16 *buf_pkh,                   /* i/o: buffer storing highband spectral peakiness      Q1*/
    Word16 *buf_Ntonal,                /* i/o: buffer storing No.of 1st spectral tone          Q0*/
    Word16 *buf_Ntonal2,               /* i/o: buffer storing No.of 2nd spectral tone          Q0*/
    Word16 *buf_Ntonal_lf              /* i/o: buffer storing low band spectral tone ratio     Q0*/
)
{
    Word16 i;
    Word32 pk;
    Word16 Ntonal;
    Word16 Ntonal2;
    Word16 Ntonal_lf;


    /* find number of tonals, number of tonals at low-band,
    spectral peakiness at high-band */
    pk = L_deposit_l(0);
    Ntonal = 0;
    move16();
    Ntonal2 = 0;
    move16();
    Ntonal_lf = 0;
    move16();
    FOR ( i=0; i<64; i++ )
    {
        if ( sub(p2v_map[i],7040) > 0 )
        {
            Ntonal = add(Ntonal,1);
        }

        IF ( sub(p2v_map[i],10240) > 0 )
        {
            Ntonal2 = add(Ntonal2,1);
            Ntonal_lf = add(Ntonal_lf,1);
        }
    }

    FOR ( i=64; i<127; i++ )
    {
        if ( p2v_map[i] != 0 )
        {
            pk = L_add(pk,p2v_map[i]); /*Q7 */
        }
        if ( sub(p2v_map[i],7040) > 0 )
        {
            Ntonal = add(Ntonal,1);
        }
        if ( sub(p2v_map[i],10240) > 0 )
        {
            Ntonal2 = add(Ntonal2,1);
        }
    }

    /* update buffers */
    FOR ( i=0; i<BUF_LEN-1; i++ )
    {
        buf_pkh[i] = buf_pkh[i+1];
        move16();
        buf_Ntonal[i] = buf_Ntonal[i+1];
        move16();
        buf_Ntonal2[i] = buf_Ntonal2[i+1];
        move16();
        buf_Ntonal_lf[i] = buf_Ntonal_lf[i+1];
        move16();
    }

    buf_pkh[i] = extract_l(L_shr_r(pk,6)); /*Q1 */
    buf_Ntonal[i] = Ntonal;
    move16();/*Q0 */
    buf_Ntonal2[i] = Ntonal2;
    move16(); /*Q0 */
    buf_Ntonal_lf[i] = Ntonal_lf;
    move16(); /*Q0 */

    return;
}

/*---------------------------------------------------------------------*
* flux_fx()
*
*
*---------------------------------------------------------------------*/

static void flux_fx(
    Word16 *Bin_E,                     /* i  : log energy spectrum of the current frame        Q7*/
    Word16 *p2v_map,                   /* i  : spectral peakiness map                          Q7*/
    Word16 *old_Bin_E,                 /* i/o: log energy spectrum of the frame 60ms ago       Q7*/
    Word16 *buf_flux,                  /* i/o: buffer storing spectral energy fluctuation      Q7*/
    Word16 attack_hangover,            /* i/o: hangover preventing flux buffering              Q0*/
    Word16 dec_mov                     /* i/o: moving average of classifier decision           Q15*/
)
{
    Word16 i;
    Word16 *pt1,*pt2,*pt3,*pt4,*pt5,*pt6;
    Word16 flux;
    Word32 L_flux;
    Word16 cnt;
    Word16 tmp;

    /* calculate flux */
    L_flux = L_deposit_l(0);
    cnt = 0;
    move16();
    FOR ( i=0; i<N_OLD_BIN_E; i++ )
    {
        IF ( p2v_map[i] != 0 )
        {
            L_flux = L_add(L_flux,abs_s(sub(Bin_E[i],old_Bin_E[i]))); /*Q7 */
        }
        if ( p2v_map[i] != 0 )
        {
            cnt = add(cnt,1);
        }
    }

    flux = 640;
    move16();/*5 in Q7 */
    IF ( cnt != 0 )
    {
        tmp = div_s(1,cnt); /*Q15 */
        flux = extract_l(Mult_32_16(L_flux,tmp)); /*Q7 */
    }

    test();
    if ( sub(flux,2560) > 0 && sub(dec_mov,26214) > 0 )
    {
        flux = 2560;
        move16(); /*20 in Q7 */
    }

    /* update old Bin_E buffer */
    pt1 = old_Bin_E;
    pt2 = old_Bin_E + N_OLD_BIN_E;
    pt3 = Bin_E;
    pt4 = old_Bin_E + N_OLD_BIN_E;
    pt5 = old_Bin_E + 2*N_OLD_BIN_E;
    pt6 = old_Bin_E + 2*N_OLD_BIN_E;

    FOR ( i=0; i<N_OLD_BIN_E; i++ )
    {
        *pt1++ = *pt2++;
        move16();
        *pt4++ = *pt5++;
        move16();
        *pt6++ = *pt3++;
        move16();
    }
    /* update flux buffer */
    IF ( attack_hangover <= 0 )
    {
        FOR ( i=0; i<BUF_LEN-1; i++ )
        {
            buf_flux[i] = buf_flux[i+1];
            move16();
        }
        buf_flux[i] = flux;
        move16();
    }

    return;
}

/*---------------------------------------------------------------------*
 * spec_analysis_fx()
 *
 *
 *---------------------------------------------------------------------*/

static void spec_analysis_fx(
    Word16 *Bin_E,                     /* i  : log energy spectrum of the current frame    Q7*/
    Word16 *p2v_map                    /* o  : spectral peakiness map                      Q7*/
)
{
    Word16 i, k, m;
    Word16 peak[65];
    Word16 valley[65];
    Word16 peak_idx[65];
    Word16 valey_idx[65];
    Word16 p2v[65];

    /* find spectral peaks */
    k = 0;
    move16();
    FOR ( i=1; i<126; i++ )
    {
        test();
        IF ( sub(Bin_E[i],Bin_E[i-1]) > 0 && sub(Bin_E[i],Bin_E[i+1]) > 0 )
        {
            peak[k] = Bin_E[i];
            move16();
            peak_idx[k] = i;
            move16();
            k = add(k,1);
        }
    }
    assert(k+1<65);
    peak_idx[k] = -1;
    move16();
    peak_idx[k+1] = -1;
    move16();
    IF ( k == 0 )
    {
        FOR ( i=0; i<127; i++ )
        {
            p2v_map[i] = 0;
            move16();
        }

        return;
    }

    /* find spectral valleys */
    m = 0;
    move16();

    IF ( sub(Bin_E[0],Bin_E[1]) < 0 )
    {
        valley[0] = Bin_E[0];
        move16();
        valey_idx[0] = 0;
        move16();
        m = add(m,1);
    }

    k = 126;
    move16();
    FOR ( i=125; i>=0; i-- )
    {
        IF (sub(Bin_E[i+1],Bin_E[i]) <= 0)
        {
            BREAK;
        }
        k = i;
        move16();
    }

    FOR ( i=1; i<k; i++ )
    {
        test();
        IF ( sub(Bin_E[i],Bin_E[i-1]) < 0 && sub(Bin_E[i],Bin_E[i+1]) < 0 )
        {
            valley[m] = Bin_E[i];
            move16();
            valey_idx[m] = i;
            move16();
            m = add(m,1);
        }
    }
    valley[m] = Bin_E[k];
    move16();
    valey_idx[m] = k;
    move16();

    /* find spectral peak to valley distances */
    k = 0;
    move16();
    FOR (i=0; i<m; i++)
    {
        test();
        IF ( sub(peak_idx[k],valey_idx[i]) > 0 && sub(peak_idx[k],valey_idx[i+1]) < 0 )
        {
            p2v[k] = sub(shl(peak[k],1),add(valley[i],valley[i+1]));
            k = add(k,1);
        }
    }

    FOR ( i=0; i<127; i++ )
    {
        p2v_map[i] = 0;
        move16();
    }

    FOR ( i=0; i<k; i++ )
    {
        p2v_map[peak_idx[i]] = p2v[i];
        move16();
    }
}

static void music_mixed_classif_improv_fx(
    Encoder_State_fx *st,                    /* i  : encoder state structure                           */
    const Word16 *new_inp,                   /* i  : new input signal                                  */
    Word16 *sp_aud_decision1,          /* i/o: 1st stage speech/music decision                   */
    Word16 vad_flag,
    const Word16 *voicing,                   /* i  : voicing estimate                                Q15*/
    const Word32 *epsP,                      /* i  : LP prediciton error                             Q_epsP*/
    Word16 Q_epsP,
    Word16 etot,                       /* i  : total frame energy                              Q8*/
    Word16 old_cor,                    /* i  : normalized correlation                          Q15*/
    Word16 cor_map_sum                 /* i  : correlation map sum                             Q8*/
)
{
    Word16 i, max_spl, dec, len, percus_flag, lt_diff, log_max_spl, epsP_tilt, p2v_map[128];
    Word16 exp, frac, expn, fracn, expd, fracd, scale;
    Word16 tmp;
    Word32 L_tmp, ftmp, ftmp1, epsP_max = MIN_32;

    /* find sample with maximum absolute amplitude */
    max_spl = 0;
    move16();
    FOR ( i=0; i<L_FRAME; i++ )
    {
        max_spl = s_max(abs_s(new_inp[i]),max_spl);
    }

    /* music is considered only appearing in high SNR condition and active signal */
    test();
    IF ( vad_flag == 0 || sub(sub(st->lp_speech_fx,st->lp_noise_fx),6400) < 0 )  /* 25 in Q8 */
    {
        /* st->dec_mov = 0.5f; */
        /* st->dec_mov1 = 0.5f; */
        st->dec_mov_fx = 16384;
        move16();
        st->dec_mov1_fx = 16384;
        move16();

        if ( vad_flag == 0 )
        {
            st->onset_cnt_fx = 0;
            move16();
        }

        return;
    }

    st->onset_cnt_fx = add(st->onset_cnt_fx,1);
    st->onset_cnt_fx = s_min(st->onset_cnt_fx, 9);

    IF ( sub(st->onset_cnt_fx,1) == 0 )
    {
        set16_fx( st->buf_flux_fx, -12800, BUF_LEN );  /*-100.0 in Q7 */
    }

    /* spectral analysis */
    spec_analysis_fx( st->lgBin_E_fx, p2v_map );

    /* percussive music detection */
    log_max_spl = 0;
    move16();
    IF ( max_spl )
    {
        L_tmp = L_deposit_h(max_spl); /*Q16 */
        exp = norm_l(L_tmp);
        frac = Log2_norm_lc(L_shl(L_tmp,exp));
        exp = sub(sub(30,exp),16);
        L_tmp = Mpy_32_16(exp,frac,28391); /*Q12 */
        log_max_spl = round_fx(L_shl(L_tmp,11)); /*Q7 */
    }

    lt_diff = sub(log_max_spl,st->mov_log_max_spl_fx); /*Q7 */

    FOR ( i=0; i<3; i++ )
    {
        st->buf_etot_fx[i] = st->buf_etot_fx[i+1];
        move16(); /*Q8 */
    }
    st->buf_etot_fx[i] = etot;
    move16();/*Q8 */

    percus_flag = 0;
    move16();
    test();
    test();
    IF ( sub(sub(st->buf_etot_fx[1],st->buf_etot_fx[0]),1536) > 0 &&
         sub(st->buf_etot_fx[2],st->buf_etot_fx[1]) < 0 &&
         sub(sub(st->buf_etot_fx[1],st->lp_speech_fx),768) > 0 )    /* 3 in Q8 */
    {
        /*tmp = add(shr(voicing[0],2),shr(voicing[1],2)); //Q15 */
        /*tmp = add(tmp,shr(old_cor,1)); //Q15 */
        tmp = mac_r(L_mac(L_mult(voicing[0],8192),voicing[1],8192),old_cor, 16384);
        test();
        test();
        IF ( sub(sub(st->buf_etot_fx[1],st->buf_etot_fx[3]),768) > 0 &&
             sub(st->buf_etot_fx[3],st->buf_etot_fx[2]) < 0 &&
             sub(tmp,24576) < 0 )     /* 0.75 in Q15 */
        {
            IF ( sub(st->dec_mov_fx,26214) > 0 )  /* 0.8 in Q15 */
            {
                percus_flag = 1;
                move16();
            }
            ELSE
            {
                test();
                test();
                test();
                IF ( sub(old_cor,24576) < 0 && sub(voicing[0],24576) < 0 && sub(voicing[1],24576) < 0 && sub(st->old_lt_diff_fx[0],1280) > 0 )
                {
                    percus_flag = 1;
                    move16();
                }
            }
        }
    }

    /* sound attack detection */
    test();
    test();
    test();
    IF (    sub(sub(st->buf_etot_fx[3],st->buf_etot_fx[2]),1536) > 0
            && sub(st->dec_mov_fx,29491) > 0
            && sub(sub(etot,st->lp_speech_fx),1280) > 0
            && sub(st->old_lt_diff_fx[0],640) > 0 )
    {
        st->attack_hangover_fx = 3;
        move16();
    }

    test();
    IF ( sub(voicing[0],29491) > 0 && sub(voicing[1],29491) > 0 )
    {
        IF ( sub(log_max_spl,st->mov_log_max_spl_fx) > 0 )
        {
            /**mov_log_max_spl = add(mult_r(31130,(*mov_log_max_spl)),mult_r(1638,log_max_spl)); //Q7 */
            st->mov_log_max_spl_fx = round_fx(L_mac(L_mult(31130,st->mov_log_max_spl_fx),1638,log_max_spl)); /*Q7 */
        }
        ELSE
        {
            /**mov_log_max_spl = add(mult_r(32604,(*mov_log_max_spl)),mult_r(164,log_max_spl)); //Q7 */
            st->mov_log_max_spl_fx = round_fx(L_mac(L_mult(32604,st->mov_log_max_spl_fx),164,log_max_spl)); /*Q7 */
        }
    }

    st->old_lt_diff_fx[0] = st->old_lt_diff_fx[1];
    move16();  /*Q7 */
    st->old_lt_diff_fx[1] = lt_diff;
    move16();   /*Q7 */

    /* calculate and buffer spectral energy fluctuation */
    flux_fx( st->lgBin_E_fx, p2v_map, st->old_Bin_E_fx, st->buf_flux_fx, st->attack_hangover_fx, st->dec_mov_fx );

    st->attack_hangover_fx = sub(st->attack_hangover_fx,1);
    st->attack_hangover_fx = s_max(st->attack_hangover_fx,0);

    /* identify flux buffer buffering status */
    len = 0;
    move16();
    FOR ( i=BUF_LEN-1; i>=0; i-- )
    {
        IF (st->buf_flux_fx[i] < 0)
        {
            BREAK;
        }

        len = add(len,1);
    }

    /* reset flux buffer if percussive music is detected */
    IF ( sub(percus_flag,1) == 0 )
    {
        set16_fx( &st->buf_flux_fx[BUF_LEN-len], 640, len ); /* 5 in Q7 */
    }

    /* calculate and buffer the tilt of residual LP energies */
    ftmp = 0;
    move16();
    ftmp1 = 0;
    move16();
    FOR( i=1; i<=16; i++ )
    {
        epsP_max = L_max(epsP_max , epsP[i]);
    }

    FOR ( i=1; i<16; i++ )
    {
        IF(L_sub(epsP[i], epsP_max) == 0)
        {
            tmp = -32768;
            move16();
            L_tmp = Mult_32_16(epsP[i],tmp); /* Q_epsP */
            ftmp = L_sub(ftmp,L_shr(L_tmp,4)); /* Q(Q_epsP-4) */
        }
        ELSE
        {
            expn = norm_l(epsP[i]);
            fracn = extract_h(L_shl(epsP[i],expn));
            expn = sub(sub(30,expn),Q_epsP);

            expd = norm_l(epsP_max);
            fracd = extract_h(L_shl(epsP_max,expd));
            expd = sub(sub(30,expd),Q_epsP);

            scale = shr(sub(fracd,fracn),15);
            fracn = shl(fracn,scale);
            expn = sub(expn,scale);

            tmp = div_s(fracn,fracd); /*Q(15+expd-expn) */
            tmp = shl(tmp,sub(expn,expd)); /*Q15 */

            L_tmp = Mult_32_16(epsP[i],tmp); /*Q_epsP */
            ftmp = L_add(ftmp,L_shr(L_tmp,4)); /*Q(Q_epsP-4) */
        }
    }

    FOR ( i=1; i<16; i++ )
    {
        IF(L_sub(epsP[i], epsP_max) == 0)
        {
            tmp = -32768;
            move16();
            L_tmp = Mult_32_16(epsP[i+1],tmp); /*Q_epsP */
            ftmp1 = L_sub(ftmp1,L_shr(L_tmp,4)); /*Q(Q_epsP-4) */
        }
        ELSE IF(L_sub(epsP[i+1],epsP_max) == 0)
        {
            tmp = -32768;
            move16();
            L_tmp = Mult_32_16(epsP[i],tmp); /*Q_epsP */
            ftmp1 = L_sub(ftmp1,L_shr(L_tmp,4)); /*Q(Q_epsP-4) */
        }
        ELSE
        {
            expn = norm_l(epsP[i]);
            fracn = extract_h(L_shl(epsP[i],expn));
            expn = sub(sub(30,expn),Q_epsP);

            expd = norm_l(epsP_max);
            fracd = extract_h(L_shl(epsP_max,expd));
            expd = sub(sub(30,expd),Q_epsP);

            scale = shr(sub(fracd,fracn),15);
            fracn = shl(fracn,scale);
            expn = sub(expn,scale);

            tmp = div_s(fracn,fracd); /*Q(15+expd-expn) */
            tmp = shl(tmp,sub(expn,expd)); /*Q15 */

            L_tmp = Mult_32_16(epsP[i+1],tmp); /*Q_epsP */
            ftmp1 = L_add(ftmp1,L_shr(L_tmp,4)); /*Q(Q_epsP-4) */
        }
    }

    /* epsP_tilt = ftmp1/ftmp; */
    expn = norm_l(ftmp1);
    fracn = extract_h(L_shl(ftmp1,expn));
    expn = sub(sub(30,expn),Q_epsP-4);

    expd = norm_l(ftmp);
    fracd = round_fx(L_shl(ftmp,expd));
    expd = sub(sub(30,expd),sub(Q_epsP,4));

    scale = shr(sub(fracd,fracn),15);
    fracn = shl(fracn,scale);
    expn = sub(expn,scale);

    tmp = div_s(fracn,fracd); /*Q(15+expd-expn) */

    epsP_tilt = shl(tmp,sub(expn,expd)); /*Q15 */

    FOR ( i=0; i<BUF_LEN-1; i++ )
    {
        st->buf_epsP_tilt_fx[i] = st->buf_epsP_tilt_fx[i+1];
        move16(); /*Q15 */
    }
    st->buf_epsP_tilt_fx[i] = epsP_tilt;
    move16(); /*Q15 */

    /* calculate and buffer highband spectral peakness */
    tonal_dist_fx( p2v_map, st->buf_pkh_fx, st->buf_Ntonal_fx, st->buf_Ntonal2_fx, st->buf_Ntonal_lf_fx );

    /* buffer sum of correlation map */
    FOR ( i=0; i<BUF_LEN-1; i++ )
    {
        st->buf_cor_map_sum_fx[i] = st->buf_cor_map_sum_fx[i+1];
        move16(); /*Q8 */
    }
    st->buf_cor_map_sum_fx[i] = cor_map_sum;
    move16(); /*Q8 */

    /* buffer voicing metric */
    FOR ( i=0; i<9; i++ )
    {
        st->buf_dlp_fx[i] = st->buf_dlp_fx[i+1];
        move16();
    }
    st->buf_dlp_fx[i] = sub(st->lps_fx, st->lpm_fx);
    move16();/*Q9 */

    /* classification */
    dec = mode_decision_fx( st, len, &st->dec_mov_fx, st->buf_flux_fx, st->buf_epsP_tilt_fx, st->buf_pkh_fx,
                            st->buf_cor_map_sum_fx, st->buf_Ntonal_fx, st->buf_Ntonal2_fx, st->buf_Ntonal_lf_fx,
                            st->buf_dlp_fx );
    move16();

    /* update long term moving average of the classification decisions */
    IF ( sub(len,30) > 0 )
    {
        IF( dec == 0 )
        {
            st->dec_mov_fx = mult_r(31785,st->dec_mov_fx); /*Q15 */
            st->dec_mov1_fx = mult_r(31785,st->dec_mov1_fx); /*Q15 */
        }
        ELSE
        {
            st->dec_mov_fx = add(mult_r(31785,st->dec_mov_fx),983); /*Q15 */
            st->dec_mov1_fx = add(mult_r(31785,st->dec_mov1_fx),983); /*Q15 */
        }
    }

    /* update long term unvoiced counter */
    test();
    test();
    test();
    IF ( (sub(st->coder_type_raw_fx,UNVOICED) == 0 || sub(st->coder_type_raw_fx,INACTIVE) == 0) &&
         sub(etot,384) > 0 && sub(st->buf_Ntonal2_fx[59],2) < 0 )
    {
        st->UV_cnt1_fx = sub(st->UV_cnt1_fx,8);
    }
    ELSE
    {
        st->UV_cnt1_fx = add(st->UV_cnt1_fx,1);
    }

    st->UV_cnt1_fx = s_min(st->UV_cnt1_fx,300);
    st->UV_cnt1_fx = s_max(st->UV_cnt1_fx,0);

    /**LT_UV_cnt1 = add(mult_r(29491,*LT_UV_cnt1),mult_r(3277,shl(*UV_cnt1,6)));*/ /* Q6  */
    st->LT_UV_cnt1_fx = round_fx(L_mac(L_mult(29491,st->LT_UV_cnt1_fx),3277,shl(st->UV_cnt1_fx,6))); /*Q6  */

    /* revert classification decision due to long-term unvoiced counter */
    test();
    test();
    IF ( sub(dec,1) == 0 && sub(st->dec_mov1_fx,6554) < 0 && sub(st->LT_UV_cnt1_fx,12800) < 0 )
    {
        dec = 0;
        move16();
    }

    /* overwrite 1st stage speech/music decision to music */
    IF (sub(dec,1) == 0 )
    {
        *sp_aud_decision1 = 1;
        move16();
    }

    return;
}



/*----------------------------------------------------------------------------------*
 * tonal_context_improv_fx()
 *
 * Context-based improvement of 1st/2nd stage speech/music decision on stable tonal signals
 *----------------------------------------------------------------------------------*/

static void tonal_context_improv_fx(
    Encoder_State_fx *st_fx,	          /* i/o: Encoder state structure                */
    const Word32 PS[],              /* i  : energy spectrum                        */
    Word16 *sp_aud_decision1, /* i/o: 1st stage speech/music decision        */
    Word16 *sp_aud_decision2, /* i/o: 2nd stage speech/music decision       */
    const Word16 vad_flag,
    const Word16 pitch[3],          /* i  : open-loop pitch estimate in three subframes     */
    const Word16 voicing[3],        /* i  : voicing estimate in three subframes             */
    const Word16 voi_fv,            /* i  : scaled voicing feature                          */
    const Word16 cor_map_sum_fv,    /* i  : scaled correlation map feature                  */
    const Word16 LPCErr,            /* i  : scaled LP prediction error feature              */
    const Word16 Qx
)
{
    Word16 t2_fx, t3_fx, tL_fx, err_fx, cor_fx, dft_fx;
    Word16 exp, expa, expb, fraca, fracb, scale, exp1, exp2, exp3, tmp;
    Word16 voi_mean, lt_pitch_diff;
    Word32 L_tmp, tonality, tonality1, tonality2, tonality3, sort_max, sort_avg, sort_val[80];

    /* estimate maximum tonality in bands [0-1 kHz], [1-2kHz] and [2-4kHz] */
    Copy32( PS, sort_val, 80 );

    /* tonality in band [0-1 kHz] */
    sort_32_fx(sort_val, 0, 19);
    sort_max = L_add(sort_val[19], 0);
    sort_avg = sum32_fx(&sort_val[0], 10);

    /* tonality1 = sort_max / sort_avg; */
    IF( sort_avg )
    {
        expa = norm_l(sort_max);
        fraca = extract_h(L_shl(sort_max,expa));
        expa = sub(30,add(expa, Qx));

        expb = norm_l(sort_avg);
        fracb = extract_h(L_shl(sort_avg,expb));
        expb =  sub(30,add(expb, Qx));

        scale = shr(sub(fracb,fraca),15);
        fraca = shl(fraca,scale);
        expa = sub(expa,scale);

        tmp = div_s(fraca,fracb);
        exp1 = sub(expa,expb);

        tonality1 = L_shl(tmp , exp1);
    }
    ELSE
    {
        tonality1 = L_shl(sort_max,sub(15,Qx)); /*Q15 */
    }

    /* tonality in band [1-2 kHz] */
    sort_32_fx(sort_val, 20, 39);
    sort_max = sort_val[39];
    sort_avg = sum32_fx(&sort_val[20], 10);

    IF( sort_avg )
    {
        /*  tonality2 = sort_max / sort_avg; */
        expa = norm_l(sort_max);
        fraca = extract_h(L_shl(sort_max,expa));
        expa = sub(30,add(expa, Qx));


        expb = norm_l(sort_avg);
        fracb = extract_h(L_shl(sort_avg,expb));
        expb =  sub(30,add(expb, Qx));

        scale = shr(sub(fracb,fraca),15);
        fraca = shl(fraca,scale);
        expa = sub(expa,scale);

        tmp = div_s(fraca,fracb);
        exp2 = sub(expa,expb);

        tonality2 = L_shl(tmp , exp2);
    }
    ELSE
    {
        tonality2 = L_shl(sort_max,sub(15,Qx)); /*Q15 */
    }

    /* tonality in band [2-4 kHz] */
    sort_32_fx(sort_val, 40, 79);
    sort_max = sort_val[79];
    sort_avg = sum32_fx(&sort_val[40], 20);

    IF( sort_avg )
    {
        /* tonality3 = sort_max / sort_avg; */
        expa = norm_l(sort_max);
        fraca = extract_h(L_shl(sort_max,expa));
        expa = sub(30,add(expa, Qx));

        expb = norm_l(sort_avg);
        fracb = extract_h(L_shl(sort_avg,expb));
        expb =  sub(30,add(expb, Qx));

        scale = shr(sub(fracb,fraca),15);
        fraca = shl(fraca,scale);
        expa = sub(expa,scale);

        tmp = div_s(fraca,fracb);
        exp3 = sub(expa,expb);

        tonality3 = L_shl(tmp , exp3);
    }
    ELSE
    {
        tonality3 = L_shl(sort_max,sub(15,Qx)); /*Q15 */
    }

    tonality = L_max(L_max(tonality1, tonality2), tonality3);

    /* voi_mean = 0.33f * (voicing[0] + voicing[1] + voicing[2]); */
    L_tmp = L_mult(voicing[0], 10923);
    L_tmp = L_mac(L_tmp, voicing[1], 10923);
    voi_mean = mac_r(L_tmp, voicing[2], 10923);  /* Q15 */

    test();
    IF( sub(st_fx->hangover_cnt_fx,10) == 0&& sub(vad_flag,1) == 0 )
    {
        /* long-term voicing parameter */
        st_fx->lt_voicing = round_fx(L_mac(L_mult(3277,st_fx->lt_voicing),29491, voi_mean));

        /* long-term correlation value */
        st_fx->lt_corr = round_fx(L_mac(L_mult(3277,st_fx->lt_corr),29491, st_fx->old_corr_fx));

        /* long-term tonality measure */
        st_fx->lt_tonality = L_add(Mult_32_16(st_fx->lt_tonality,3277),Mult_32_16(tonality,29491));
    }
    ELSE
    {
        /* long-term voicing parameter */
        st_fx->lt_voicing = round_fx(L_mac(L_mult(22938,st_fx->lt_voicing),9830, voi_mean));

        /* long-term correlation value */
        st_fx->lt_corr = round_fx(L_mac(L_mult(22938,st_fx->lt_corr),9830, st_fx->old_corr_fx));

        /* long-term tonality measure */
        st_fx->lt_tonality = L_add(Mult_32_16(st_fx->lt_tonality,16384),Mult_32_16(tonality,16384));
    }

    /* Pitch difference w.r.t to past 3 frames */
    lt_pitch_diff = abs_s(sub(st_fx->lt_corr_pitch[0], pitch[0]));
    lt_pitch_diff = add(lt_pitch_diff , abs_s(sub(st_fx->lt_corr_pitch[1], pitch[0])));
    lt_pitch_diff = add(lt_pitch_diff,abs_s(sub(st_fx->lt_corr_pitch[2], pitch[0])));

    st_fx->lt_corr_pitch[0] = st_fx->lt_corr_pitch[1];
    move16();
    st_fx->lt_corr_pitch[1] = st_fx->lt_corr_pitch[2];
    move16();
    st_fx->lt_corr_pitch[2] = pitch[0];
    move16();

    st_fx->lt_old_mode[0] = st_fx->lt_old_mode[1];
    move16();
    st_fx->lt_old_mode[1] = st_fx->lt_old_mode[2];
    move16();

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF ( *sp_aud_decision1 == 1 &&
         ( L_sub(L_min(L_min(tonality1, tonality2), tonality3),1638400) > 0 ) &&
         ( L_sub(L_add(tonality1, tonality2),6553600) > 0 && L_sub(L_add(tonality2, tonality3),6553600) > 0 && L_sub(L_add(tonality1, tonality3),6553600) > 0 ) &&
         ( L_sub(st_fx->lt_tonality,655360000) < 0 ) &&
         ( ( L_sub(st_fx->lt_tonality,32768000) > 0 && sub(s_max(st_fx->lt_voicing, voi_mean),32440) > 0 ) ||
           ( L_sub(st_fx->lt_tonality,49152000) > 0 && sub(st_fx->lt_corr,32440) > 0 ) ||
           ( L_sub(st_fx->lt_tonality,98304000) > 0 && sub(st_fx->lowrate_pitchGain,15729) > 0 ) ||
           ( lt_pitch_diff == 0 && sub(st_fx->lowrate_pitchGain,14582) > 0 ) ) )
    {
        IF( sub(sum16_fx(st_fx->lt_old_mode, 2),2) < 0 )
        {
            /* probably speech - change the decision to speech */
            *sp_aud_decision1 = 0;
            move16();
            *sp_aud_decision2 = 0;
            move16();

            if( st_fx->lt_hangover == 0 )
            {
                st_fx->lt_hangover = 6;
                move16();
            }
        }
    }
    ELSE
    {
        /* not speech, but still in the hangover period - change the decision to speech */
        IF( st_fx->lt_hangover > 0 )
        {
            *sp_aud_decision1 = 0;
            move16();
            *sp_aud_decision2 = 0;
            move16();

            st_fx->lt_hangover = sub(st_fx->lt_hangover,1);
        }
    }

    /* calculate standard deviation of log-tonality */
    Copy( st_fx->tonality2_buf_fx + 1, st_fx->tonality2_buf_fx, HANG_LEN_INIT - 1 );
    /* st->tonality2_buf[HANG_LEN_INIT - 1] = 0.2f*(float)log10(tonality2); */
    exp = norm_l(tonality2);
    tmp = Log2_norm_lc(L_shl(tonality2, exp));/*15 */
    exp = sub(30, add(exp, 16));
    L_tmp = Mpy_32_16(exp, tmp, 15783);/*19 //3945, 0.2*log10(2), Q18 */
    st_fx->tonality2_buf_fx[HANG_LEN_INIT - 1] = round_fx(L_shl(L_tmp, 11));/*14 */
    /* t2 = std( st->tonality2_buf, HANG_LEN_INIT ); */
    t2_fx  = std_fx( st_fx->tonality2_buf_fx, HANG_LEN_INIT ); /*14 */

    Copy( st_fx->tonality3_buf_fx + 1, st_fx->tonality3_buf_fx, HANG_LEN_INIT - 1 );
    /* st->tonality3_buf[HANG_LEN_INIT - 1] = 0.2f*(float)log10(tonality3); */
    exp = norm_l(tonality3);
    tmp = Log2_norm_lc(L_shl(tonality3, exp));/*15 */
    exp = sub(30, add(exp, 16));
    L_tmp = Mpy_32_16(exp, tmp, 15783);/*19 //3945, 0.2*log10(2), Q18 */
    st_fx->tonality3_buf_fx[HANG_LEN_INIT - 1] = round_fx(L_shl(L_tmp, 11));/*14 */
    t3_fx  = std_fx( st_fx->tonality3_buf_fx, HANG_LEN_INIT ); /*14 */

    /* tL  = 0.2f*(float)log10(st->lt_tonality); */
    exp = norm_l(st_fx->lt_tonality);
    tmp = Log2_norm_lc(L_shl(st_fx->lt_tonality, exp));/*15 */
    exp = sub(30, add(exp, 16));
    L_tmp = Mpy_32_16(exp, tmp, 15783);/*19 //3945, 0.2*log10(2), Q18 */
    tL_fx = round_fx(L_shl(L_tmp, 11));/*14 */

    /* calculate standard deviation of residual LP energy */
    Copy( st_fx->LPCErr_buf_fx + 1, st_fx->LPCErr_buf_fx, HANG_LEN_INIT - 1 );
    st_fx->LPCErr_buf_fx[HANG_LEN_INIT - 1] = LPCErr;
    /* err = std( st->LPCErr_buf, HANG_LEN_INIT ); */
    err_fx  = std_fx( st_fx->LPCErr_buf_fx, HANG_LEN_INIT );

    cor_fx = s_max(sub(voi_fv, cor_map_sum_fv), 0);/*15 */
    dft_fx = abs_s(sub(st_fx->tonality2_buf_fx[HANG_LEN_INIT - 1], st_fx->tonality3_buf_fx[HANG_LEN_INIT - 1]));/*14 */


    /* state machine for strong music */
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF ( (sub(*sp_aud_decision1, 1) == 0) && st_fx->lt_music_state_fx == 0 && st_fx->lt_music_hangover_fx == 0 &&
         (sub(t2_fx, 8847) < 0) && (sub(t2_fx, 4260) > 0) && (sub(t3_fx, 3604) > 0) && (sub(tL_fx, 8847) < 0) && (sub(tL_fx, 4260) > 0) && (sub(err_fx, 8192) > 0) )
    {
        st_fx->lt_music_state_fx = 1;
        move16();
        st_fx->lt_music_hangover_fx = 6;
        move16();
    }
    ELSE IF( sub(st_fx->lt_music_state_fx, 1) == 0 && st_fx->lt_music_hangover_fx == 0 &&
             (sub(t2_fx, 5571) < 0) && (sub(t3_fx, 4260) < 0) && (sub(tL_fx, 7373) < 0) )
    {
        st_fx->lt_music_state_fx = 0;
        move16();
        st_fx->lt_music_hangover_fx = 6;
        move16();
    }

    IF ( st_fx->lt_music_hangover_fx > 0 )
    {
        st_fx->lt_music_hangover_fx = sub(st_fx->lt_music_hangover_fx,1);
    }

    /* state machine for strong speech */
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF ( (sub(*sp_aud_decision1, 1) == 0) && st_fx->lt_speech_state_fx == 0 && st_fx->lt_speech_hangover_fx == 0 &&
         (sub(cor_fx, 13107) > 0) && (sub(dft_fx, 1638) < 0) && sub(shr(voi_fv,1), add(cor_map_sum_fv, 1966)) > 0 &&
         (sub(t2_fx, shr(cor_fx, 1)) < 0) && (sub(t3_fx, shr(cor_fx, 1)) < 0) && (sub(tL_fx, shr(cor_fx, 1)) < 0) &&
         (sub(cor_map_sum_fv, cor_fx) < 0) && (sub(voi_fv, cor_fx) > 0) && (sub(voi_fv, 24903) > 0) )
    {
        st_fx->lt_speech_state_fx = 1;
        move16();
        st_fx->lt_speech_hangover_fx = 6;
        move16();
    }
    ELSE IF ( (sub(st_fx->lt_speech_state_fx, 1) == 0) && st_fx->lt_speech_hangover_fx == 0 && (sub(cor_fx, 13107) < 0) )
    {
        st_fx->lt_speech_state_fx = 0;
        move16();
        st_fx->lt_speech_hangover_fx = 6;
        move16();
    }

    IF ( st_fx->lt_speech_hangover_fx > 0)
    {
        st_fx->lt_speech_hangover_fx = sub(st_fx->lt_speech_hangover_fx,1);
    }

    /* final decision */
    test();
    test();
    IF ( sub(*sp_aud_decision1,1) == 0 && sub(st_fx->lt_speech_state_fx,1) == 0 )
    {
        /* strong speech - probably error in speech/music classification */
        *sp_aud_decision1 = 0;
        move16();
        *sp_aud_decision2 = 0;
        move16();
    }
    ELSE IF ( *sp_aud_decision1 == 0 && sub(st_fx->lt_speech_state_fx,1) == 0 )
    {
        /* strong music - probably error in speech/music classification */
        *sp_aud_decision1 = 0;
        move16();
        *sp_aud_decision2 = 0;
        move16();
    }

    /* update the buffer of past decisions */
    st_fx->lt_old_mode[2] = *sp_aud_decision1;
    move16();

    return;
}


static void detect_sparseness_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure                */
    const Word16 localVAD_HE_SAD,      /* i  : HE-SAD flag without hangover           */
    Word16 *sp_aud_decision1,    /* i/o: 1st stage speech/music decision        */
    Word16 *sp_aud_decision2,    /* i/o: 2nd stage speech/music decision        */
    const Word16 voi_fv                /* i  : scaled voicing feature                 */
)
{
    Word16 sum, sumh;
    Word32 L_tmp,L_tmp1;
    Word16 tmp, tmp1;
    Word16 S1[128];
    Word16 i,j;
    Word16 hb_sp_high_flag = 0;
    Word16 lb_sp_high_flag = 0;
    Word16 sparse;
    Word16 tmp_buf[4];
    Word16 Mlpe=0, Mv=0, Msp;


    Copy(st_fx->lgBin_E_fx+1, S1, 127); /* wz: need to check with Vlad */
    S1[127] = st_fx->lgBin_E_fx[127];
    move16();

    L_tmp = L_deposit_l(0);
    FOR (i = 0; i < 80; i++)
    {
        if (S1[i] < 0)
        {
            S1[i] = 0;
            move16(); /* Q7 */
        }
        L_tmp = L_add(L_tmp, L_deposit_l(S1[i]));
    }

    L_tmp1 = L_deposit_l(0);
    FOR (i = 80; i < 128; i++)
    {
        if (S1[i] < 0)
        {
            S1[i] = 0;
            move16();
        }
        L_tmp1 = L_add(L_tmp1, L_deposit_l(S1[i]));
    }

    sumh = extract_l(L_shr(L_tmp1, 7)); /* Q0 */
    sum = add(extract_l(L_shr(L_tmp, 7)), sumh); /* Q0 */

    /* order spectral from max to min */
    order_spectrum_fx(S1, 128);

    /* calculate spectral sparseness in the range 0 - 6.4 kHz */
    j = 0;
    move16();
    L_tmp = 0;
    move16();
    L_tmp1 = L_deposit_l(mult(sum, 24576));
    FOR (i = 0; i < 128; i++)
    {
        L_tmp = L_add(L_tmp, L_deposit_l(S1[i]));
        IF (L_sub(L_shr(L_tmp, 7), L_tmp1) > 0)
        {
            j = i;
            move16();
            BREAK;
        }
    }

    FOR (i = 0; i < HANG_LEN_INIT-1; i++)
    {
        st_fx->sparse_buf_fx[i] = st_fx->sparse_buf_fx[i+1];
        move16();
    }

    sparse = j;
    move16();
    st_fx->sparse_buf_fx[i] = sparse;
    move16();

    IF (sub(st_fx->bwidth_fx, WB) == 0)
    {
        Msp = 0;
        move16();
        FOR (i = 0; i < 8; i++)
        {
            Msp = add(Msp, st_fx->sparse_buf_fx[i]);
        }
        Msp = shl(Msp, 5); /* Q8 */

        /* find long-term smoothed sparseness */
        IF (st_fx->last_vad_spa_fx == 0 )
        {
            set16_fx( &st_fx->sparse_buf_fx[0], sparse, HANG_LEN_INIT-1 );
            st_fx->LT_sparse_fx = sparse;
            move16();
        }
        ELSE
        {
            set16_fx(tmp_buf, 0, 4);

            FOR (i = 0; i < HANG_LEN_INIT; i++)
            {
                FOR (j = 0; j < 4; j++)
                {
                    IF (sub(st_fx->sparse_buf_fx[i], tmp_buf[j]) > 0)
                    {
                        Copy(&tmp_buf[j], &tmp_buf[j+1], sub(3, j));
                        tmp_buf[j] = st_fx->sparse_buf_fx[i];
                        move16();
                        BREAK;
                    }
                }
            }

            /* ftmp = 0.25f*(HANG_LEN_INIT*Msp - sum_f(tmp_buf, 4)) - st->LT_sparse; */
            tmp = shl(sum16_fx(tmp_buf, 4), 5);
            tmp = shl(sub(Msp, tmp), 1);
            tmp = sub(tmp, st_fx->LT_sparse_fx);

            st_fx->LT_sparse_fx = add(st_fx->LT_sparse_fx, shr(tmp, 2)); /* Q8 */
        }

        /* find high-band sparseness */
        Copy(st_fx->lgBin_E_fx+81, S1, 47);
        S1[47] = st_fx->lgBin_E_fx[127];
        move16();

        order_spectrum_fx(S1, 48);

        FOR (i = 0; i < HANG_LEN_INIT-1; i++)
        {
            st_fx->hf_spar_buf_fx[i] = st_fx->hf_spar_buf_fx[i+1];
            move16();
        }

        /* st_fx->hf_spar_buf_fx[i] = sum_f(S1, 5)/sumh; */
        L_tmp = L_deposit_l(0);
        FOR (i = 0; i < 5; i++)
        {
            if (S1[i] < 0)
            {
                S1[i] = 0;
                move16();
            }

            L_tmp = L_add(L_tmp, S1[i]);
        }

        tmp = extract_l(L_shr(L_tmp, 7));
        IF (tmp == 0)
        {
            st_fx->hf_spar_buf_fx[HANG_LEN_INIT-1] = 0;
            move16();
        }
        ELSE
        {
            st_fx->hf_spar_buf_fx[HANG_LEN_INIT-1] = div_s(tmp, sumh);
        }

        tmp = 0;
        move16();
        FOR (i = 0; i < 8; i++)
        {
            tmp = add(tmp, shr(st_fx->hf_spar_buf_fx[i], 3));
        }
        IF (sub(tmp, 6554) > 0)
        {
            hb_sp_high_flag = 1;
            move16();
        }

        /* find low-band sparseness */
        Copy(st_fx->lgBin_E_fx+1, S1, 59);
        S1[59] = st_fx->lgBin_E_fx[59];
        move16();

        order_spectrum_fx(S1, 60);
        L_tmp = L_deposit_l(0);
        L_tmp1 = L_deposit_l(0);
        FOR (i = 0; i < 5; i++)
        {
            if (S1[i] < 0)
            {
                S1[i] = 0;
                move16();
            }

            L_tmp = L_add(L_tmp, S1[i]);
        }

        FOR (; i < 60; i++)
        {
            if (S1[i] < 0)
            {
                S1[i] = 0;
                move16();
            }

            L_tmp1 = L_add(L_tmp1, S1[i]);
        }

        /* if ( sum_f(S1, 5)/sum_f(S1,60) > 0.18f ) */
        tmp = extract_l(L_shr(L_tmp, 7));
        IF (tmp != 0)
        {
            tmp = div_s(tmp, add(tmp, extract_l(L_shr(L_tmp1, 7))));
            if (sub(tmp, 5898) > 0)
            {
                lb_sp_high_flag = 1;
                move16();
            }
        }

        /* find smoothed linear prediction efficiency */
        FOR (i = 0; i < 7; i++)
        {
            st_fx->lpe_buf_fx[i] = st_fx->lpe_buf_fx[i+1];
            move16();
        }

        st_fx->lpe_buf_fx[i] = st_fx->past_epsP2_fx;
        move16();
        Mlpe = 0;
        move16();
        FOR (i = 0; i < 8; i++)
        {
            Mlpe = add(Mlpe, shr(st_fx->lpe_buf_fx[i], 3));
        }

        /* find smoothed voicing */
        FOR (i = 0; i < HANG_LEN_INIT-1; i++)
        {
            st_fx->voicing_buf_fx[i] = st_fx->voicing_buf_fx[i+1];
            move16();
        }

        st_fx->voicing_buf_fx[i] = voi_fv;
        move16();
        Mv = 0;
        move16();
        FOR (i = 0; i < 8; i++)
        {
            Mv = add(Mv, shr(st_fx->voicing_buf_fx[i], 3));
        }
    }

    /* avoid using LR-MDCT on sparse spectra */
    IF (sub(*sp_aud_decision1, 1) == 0)
    {
        tmp = 91;
        move16();
        if (sub(st_fx->bwidth_fx, WB) == 0)
        {
            tmp = 90;
        }

        IF (sub(sparse, tmp) > 0)
        {
            *sp_aud_decision1 = 0;
            move16();
            *sp_aud_decision2 = 1;
            move16();
            st_fx->gsc_hangover_fx = 1;
            move16();
        }
        ELSE IF (sub(st_fx->gsc_hangover_fx, 1) == 0)
        {
            IF (sub(sparse, 85) > 0)
            {
                *sp_aud_decision1 = 0;
                move16();
                *sp_aud_decision2 = 1;
                move16();
            }
            ELSE
            {
                tmp = 0;
                move16();
                FOR (i=0; i<st_fx->gsc_cnt_fx; i++)
                {
                    tmp = add(tmp, st_fx->sparse_buf_fx[HANG_LEN_INIT-1-st_fx->gsc_cnt_fx+i]);
                }
                tmp1 = div_s(1, st_fx->gsc_cnt_fx);
                tmp = mult(tmp, tmp1);

                IF (sub(abs_s(sub(sparse, tmp)), 7) < 0)
                {
                    *sp_aud_decision1 = 0;
                    move16();
                    *sp_aud_decision2 = 1;
                    move16();
                }
            }
        }

        IF (sub(st_fx->bwidth_fx, WB) == 0)
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF (sub(st_fx->LT_sparse_fx, 15360) > 0 && sub(sparse, 50) > 0 && sub(Mlpe, -1331) < 0 && sub(Mv, 27853) > 0 &&
                lb_sp_high_flag == 0 && ((hb_sp_high_flag == 0 && sub(sumh, mult_r(4915, sum)) > 0) || sub(sumh, mult_r(4915, sum)) <= 0))
            {
                *sp_aud_decision1 = 0;
                move16();
                *sp_aud_decision2 = 1;
                move16();
                st_fx->gsc_hangover_fx = 1;
                move16();
            }
            ELSE IF (sub(st_fx->gsc_hangover_fx, 1) == 0 && !( *sp_aud_decision1 == 0 && *sp_aud_decision2 == 1))
            {
                IF (sub(abs_s(sub(sparse, mean_fx(&st_fx->sparse_buf_fx[HANG_LEN_INIT-1-st_fx->gsc_cnt_fx], st_fx->gsc_cnt_fx))), 7) < 0)
                {
                    *sp_aud_decision1 = 0;
                    move16();
                    *sp_aud_decision2 = 1;
                    move16();
                }
            }
        }
    }

    /* update the counter of consecutive GSC frames with sparse spectrum */
    test();
    IF (*sp_aud_decision1 == 0 && sub(*sp_aud_decision2, 1) == 0)
    {
        st_fx->gsc_cnt_fx = add(st_fx->gsc_cnt_fx, 1);
        IF (sub(st_fx->gsc_cnt_fx, 7) > 0)
        {
            st_fx->gsc_cnt_fx = 7;
            move16();
        }
    }
    ELSE
    {
        st_fx->gsc_cnt_fx = 0;
        move16();
        st_fx->gsc_hangover_fx = 0;
        move16();
    }

    st_fx->last_vad_spa_fx = localVAD_HE_SAD;
    move16();

    return;
}

static void order_spectrum_fx(
    Word16 *vec,
    Word16 len
)
{
    Word16 i, j, tmp, tmp_loop;
    Word16 *pts, *pte;
    Word16 smax, smin;
    Word16 imax, imin;

    pts = &vec[0];
    pte = &vec[len-1];
    tmp_loop = shr(len, 1);
    FOR (i = 0; i < tmp_loop; i++)
    {
        smax = vec[i];
        move16();
        smin = vec[i];
        move16();
        imax = i;
        move16();
        imin = i;
        move16();
        tmp = sub(len, i);
        FOR (j = i; j < tmp; j++)
        {
            IF (sub(vec[j], smax) > 0)
            {
                smax = vec[j];
                move16();
                imax = j;
                move16();
            }
            ELSE
            {
                IF (sub(vec[j], smin) < 0)
                {
                    smin = vec[j];
                    move16();
                    imin = j;
                    move16();
                }
            }
        }

        vec[imax] = *pts;
        move16();
        vec[imin] = *pte;
        move16();
        *pts++ = smax;
        move16();
        *pte-- = smin;
        move16();
    }
}
