/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define INV_MAX_SNR_FX     745     /* Q15 {1/(45-1)} Max. SNR considered for noise subtraction in voiced segments  */
#define MAX_SNR_SNR1_FX    16756   /* Q14 45* (1/(max_snr1-1)) */

#define BIN_1KHZ       (short)(1000/BIN_16kdct_fx)
#define BIN_2KHZ       (short)(2000/BIN_16kdct_fx)
#define BIN_4KHZ       (short)(4000/BIN_16kdct_fx)

#define MAX_GN_R_Q14_FX   3277
#define ALPH_Q15_FX       32767
#define BET_Q15_FX        30310
#define MAXX_Q12_FX       (20480)

#define MAXX_FX   5

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/
static void analy_sp_dct_fx(const Word16 *dct_buf, Word32 *fr_bands, Word32 *lf_E, Word16 *etot, const Word16 Qdct);
static void find_enr_dct_fx(const Word16 data[], Word32 band[], Word32 *ptE, Word32 *Etot, const Word16 min_band,
                            const Word16 max_band, const Word16 Q_dct, const Word16 bin_freq );
static Word16 norm_lfe(const Word32 Lfe,const Word16 m_max,const Word16 e_max);
static void spectrum_mod_dct_fx(const Word16 Qdct,Word16 data[], const Word32 lf_E[], Word32 lf_EO[],
                                const Word32 fr_bands[], Word32 enrO[], const Word32 noiseE[], const Word16 minGain,
                                Word16 lp_gbin[], const Word16 music_flag, Word16 min_band, const Word16 MAX_GN, const Word16 MAX_band );
/*------------------------------------------------------------------------*
 * LD_music_post_filter()
 *
 * Music post-filter
 *------------------------------------------------------------------------*/
void LD_music_post_filter_fx
(
    const Word16 dtc_in[],         /* i   : input synthesis                       Qdct   */
    Word16 dtc_out[],        /* o   : output synthesis                      Qdct   */
    const Word32 core_brate,       /* i   : core bitrate                             Q0  */
    Word16 bfi,              /* i   : Bad frame indicator                      Q0  */
    Word16 *last_music_flag, /* i/o : Previous music detection ouptut          Q0  */
    Word16 *last_bfi_cnt,    /* i/o : number of frame since last bfi           Q0  */
    Word16 *thresh,          /* i/o : Detection thresold                       Q0  */
    Word16 *nb_thr_1,        /* i/o : Number of consecutives frames of level 1 Q0  */
    Word16 *nb_thr_3,        /* i/o : Number of consecutives frames of level 3 Q0  */
    Word16 *lt_diff_etot,    /* i/o : Long term total energy variation         Q8  */
    Word16 *mem_etot,        /* i/o : Total energy memory                      Q8  */
    const Word16 min_ns_gain,      /* i   : minimum gain for inter-harm noise red.   Q15 */
    Word32 bckr[],           /* i/o : per band bckgnd. noise energy estimate     */
    Word32 enro[],           /* i/o : per band old input energy                  */
    Word32 lf_EO[],          /* i/o : old per bin E for previous half frame    2*Qdct+10  */
    Word16 lp_gbin[],        /* i/o : smoothed suppression gain, per FFT bin   Q15  */
    Word16 *filt_lfE,        /* i   : post filter weighting coefficient        Q15  */
    Word16 *last_nonfull_music, /* i: Number of frames sinces la "speech like" frame Q0*/
    Word16 *Old_ener_Q,      /* i/o : Old energy scaling factor             */
    const Word16 coder_type,       /* i   : Coder type : -1 in case of IO            Q0  */
    const Word16 Last_coder_type,  /* i   : input scaling                            Q0  */
    const Word16 Qdct              /* i   : input scaling                            Q0  */
)
{
    Word32 fr_bands[MBANDS_GN_LD];
    Word32 lf_E[VOIC_BINS_HR];
    Word32 Ltmp, Ltmp_max;
    Word16 LG_etot;  /*Q8*/
    Word16 i, j, k;
    Word16 min_band = 0;
    Word16 local_min_gain = min_ns_gain;
    Word16 music_flag2 = 0;
    Word32 max_val;
    Word16 max_ovf_2k, max_ovf_4k, max_ovf_6k;
    Word16 min_g_2k, min_g_4k, min_g_6k;
    Word32 m_ave;
    Word16 tmp_lfE[DCT_L_POST];  /*Q12*/
    Word16 MAX_GN = MAX_GN_R_Q14_FX;
    Word16 MAX_band = MBANDS_GN_LD;
    Word16 mant, exp1, s_ave, tmp16, old_tmp16;
    Word16 diff_sc;


    /*------------------------------------------------------------------------*
     * Frequency analysis
     *------------------------------------------------------------------------*/

    analy_sp_dct_fx( dtc_in, fr_bands, lf_E, &LG_etot, Qdct);
    diff_sc = shl(sub(Qdct, *Old_ener_Q),1);
    *Old_ener_Q = Qdct;
    move16();

    Scale_sig32(lf_EO, VOIC_BINS_HR, diff_sc);

    /*------------------------------------------------------------------------*
     * Find signal classification
     *------------------------------------------------------------------------*/
    music_flag2 = stab_est_fx( LG_etot, lt_diff_etot, mem_etot, last_bfi_cnt, bfi, nb_thr_3, nb_thr_1, thresh, last_music_flag, 1 );

    *last_nonfull_music = add(*last_nonfull_music,1);
    move16();
    if( sub(music_flag2,4) < 0 )
    {
        *last_nonfull_music = 0;
        move16();
    }

    test();
    if ( L_sub(core_brate,ACELP_6k60) < 0
            || sub(Last_coder_type, AUDIO) != 0 )
    {
        /* do not perform music improvement on SID frames */
        music_flag2 = 0;
        move16();
    }

    *last_nonfull_music = s_min( 51, *last_nonfull_music );

    /*------------------------------------------------------------------------*
     * Remapping of bands
     * Section to "remap" the minimal band and the minimum gain for our needs
     *------------------------------------------------------------------------*/

    IF( sub(music_flag2,3)  > 0)
    {
        min_band = 2;
        move16();
        local_min_gain = 8231;        /*Q15->0.25119f;*/     move16();
    }
    ELSE IF( sub(music_flag2,3)  == 0)
    {
        min_band = 3;
        move16();
        local_min_gain = 8231;        /*Q15->0.25119f; */    move16();
    }
    ELSE IF( sub(music_flag2,2)  == 0)
    {
        min_band = 4;
        move16();
        local_min_gain = 11626;       /*Q15->0.35481f; */     move16();
    }
    ELSE IF( sub(music_flag2,1)  == 0)
    {
        min_band = 4;
        move16();
        local_min_gain = 16423;       /*Q15->0.50119f;*/      move16();
    }

    min_band = add(min_band, 4);

    MAX_GN = 1638; /*Q14*/                 move16();
    if( L_sub(core_brate,ACELP_9k60) > 0 )
    {
        /* overshoot not allowed, since GSC already matches the energy */
        MAX_GN = 0;
        move16();
    }

    if( sub(coder_type,AUDIO) == 0 )
    {
        /* with GSC we know for sure that we are in music */
        min_band = s_min( min_band, 3 );
    }

    /*------------------------------------------------------------------------*
     * Approximation of the inter-harmonic noise level
     * - sort the bin energy
     * - compupte the average energy per band excluding the maximum energy bin
     *------------------------------------------------------------------------*/

    j = 0;
    move16();
    Ltmp_max = L_deposit_l(0);
    FOR (i = 0; i < MBANDS_GN_LD; i++)
    {
        Ltmp  = L_deposit_l(0);
        max_val = L_deposit_l(0);

        FOR( k=j; k < mfreq_bindiv_LD[i]+j; k++ )
        {
            /*m_ave += lf_E[k];*/
            Ltmp = L_add(lf_E[k], Ltmp);
            max_val = L_max(max_val, lf_E[k]);
        }
        Ltmp_max = L_max(Ltmp_max, max_val);
        /*m_ave -= max_val;*/
        Ltmp = L_sub(Ltmp, max_val);
        /*m_ave /=(mfreq_bindiv_LD[i]-1);*/
        m_ave = Mult_32_16(Ltmp,inv_mfreq_bindiv_LD_M1_fx[i]);

        /*bckr[i] = m_ave*sc_qnoise[i];*/
        bckr[i] = Mult_32_16(m_ave,sc_qnoise_fx[i]);
        move32();

        j+=mfreq_bindiv_LD[i];
    }

    /* This is computed inside the loop i = maximum(lf_E, DCT_L_POST, &m_ave);*/
    /*------------------------------------------------------------------------*
     * - Normalisation of the energy vector between [0.72, 5], with the form of pow(x,4)
     * - Simple LP filtering along the frequency domain
     * - LT averaging with the past and in function of the stability factor
     *------------------------------------------------------------------------*/
    /*m_ave = ALPH/lf_E[i];*/
    exp1 = norm_l(Ltmp_max);
    mant = extract_h(L_shl(Ltmp_max, exp1));
    /*exp1 = sub(16,exp1);*/

    s_ave = div_s(16384, mant);
    exp1 = sub(14/*+15*/+16, exp1);   /*s_ave in Q15 + exp1*/

    old_tmp16 = norm_lfe(lf_E[0], s_ave, exp1);
    old_tmp16 = s_min(old_tmp16, MAXX_Q12_FX);
    tmp16 = norm_lfe(lf_E[1], s_ave, exp1);
    tmp16 = s_min(tmp16, MAXX_Q12_FX);
    tmp_lfE[0] = round_fx(L_mac(L_mult(16384, old_tmp16), 16384, tmp16));

    FOR(i = 1; i < DCT_L_POST-1; i++)
    {
        tmp16 = norm_lfe(lf_E[i], s_ave, exp1);
        tmp16 = s_min(tmp16, MAXX_Q12_FX);
        /*tmp_lfE[i] = 0.333f*old_ftmp + 0.333f*ftmp;  */
        Ltmp = L_mac(L_mult(10813, old_tmp16), 10813, tmp16);

        old_tmp16 = tmp16;
        move16();

        tmp16 = norm_lfe(lf_E[i+1], s_ave, exp1);
        /*ftmp = min(ftmp, MAXX);
        tmp_lfE[i] += 0.333f*ftmp;  */
        tmp16 = s_min(tmp16, MAXX_Q12_FX);
        Ltmp = L_mac(Ltmp, 10813, tmp16);
        tmp_lfE[i] = round_fx(Ltmp);
    }

    tmp16 = norm_lfe(lf_E[i], s_ave, exp1);
    /*ftmp = min(ftmp, MAXX);
    tmp_lfE[i] = 0.5f*old_ftmp + 0.5f*ftmp;*/
    tmp16 = s_min(tmp16, MAXX_Q12_FX);
    tmp_lfE[i] = round_fx(L_mac(L_mult(16384, old_tmp16), 16384, tmp16));

    FOR(i = 0; i < BIN_4KHZ; i++)
    {
        /*filt_lfE[i] = tmp_lfE[i]*.05f + .95f*filt_lfE[i] ;*/
        filt_lfE[i] = round_fx(L_mac(L_mult(tmp_lfE[i],1638), 31130, filt_lfE[i])) ;
    }

    FOR(; i < DCT_L_POST; i++)
    {
        /*filt_lfE[i] = tmp_lfE[i]*(.15f) + .85f*filt_lfE[i] ;*/
        filt_lfE[i] = round_fx(L_mac(L_mult(tmp_lfE[i],4915), 27853, filt_lfE[i])) ;
    }
    /*------------------------------------------------------------------------*
     * - Reduce inter-harmonic noise with SNR based method
     * - Second stage of spectral shaping modification based
     *   on the pow(x,4) energy spectrum
     *------------------------------------------------------------------------*/

    if( sub(coder_type,AUDIO) == 0 )
    {
        MAX_band = 16;
        move16();
    }

    Copy(dtc_in, dtc_out, DCT_L_POST);
    spectrum_mod_dct_fx(Qdct, dtc_out, lf_E, lf_EO, fr_bands, enro,
                        bckr, local_min_gain, lp_gbin, music_flag2, min_band, MAX_GN, MAX_band );

    i = 0;
    move16();
    IF( sub(music_flag2,1) >= 0 )
    {
        FOR(i = 0; i < BIN_1KHZ; i++)
        {
            tmp16 = s_min(4096,  filt_lfE[i]);
            dtc_out[i] = round_fx(L_shl(L_mult(dtc_out[i], tmp16),3));
        }
    }
    {
        IF( sub(*last_nonfull_music,40) > 0 )
        {
            max_ovf_2k = 5120;     /*1.25 Q12*/ move16();
            max_ovf_4k = 6144;     /*1.5 Q12*/  move16();
            max_ovf_6k = 6144;     /*1.5 Q12*/  move16();

            min_g_2k = 0;
            move16();
            min_g_4k = 0;
            move16();
            min_g_6k = 0;
            move16();

            IF( sub(coder_type,AUDIO ) == 0)
            {
                max_ovf_2k = 4096;  /*1.0 Q12*/ move16();
                max_ovf_4k = 4506;  /*1.1 Q12*/ move16();
                max_ovf_6k = 5120; /*1.25 Q12*/ move16();

                min_g_2k = 3072;   /*0.75 Q12*/ move16();
                min_g_4k = 2048;    /*0.5 Q12*/ move16();
                min_g_6k = 2048;    /*0.5 Q12*/ move16();

                IF( L_sub(core_brate,ACELP_9k60)  > 0 )
                {
                    max_ovf_4k = 4096;  /*1.0 Q12*/ move16();
                    max_ovf_6k = 4710; /*1.15 Q12*/ move16();

                    min_g_2k = 3789;  /*0.925 Q12*/ move16();
                    min_g_4k = 3379;  /*0.825 Q12*/ move16();
                    min_g_6k = 3072;   /*0.75  Q12*/ move16();
                }
            }
            ELSE IF( L_sub(core_brate,ACELP_12k65)  >= 0 )
            {
                max_ovf_2k = 4096;      /*1.0 Q12*/ move16();
                max_ovf_4k = 5120;     /*1.25 Q12*/ move16();

                IF( L_sub(core_brate,ACELP_15k85) > 0 )
                {
                    max_ovf_4k = 4096;  /*1.0 Q12*/ move16();
                    max_ovf_6k = 5120; /*1.25 Q12*/ move16();

                    min_g_2k = 3072;   /*0.75 Q12*/ move16();
                    min_g_4k = 2048;    /*0.5 Q12*/ move16();
                    min_g_6k = 2048;    /*0.5 Q12*/ move16();
                }
            }

            FOR(; i < BIN_2KHZ; i++)
            {
                tmp16 = s_min(max_ovf_2k,  filt_lfE[i]);
                tmp16 = s_max(min_g_2k, tmp16);
                /*DCT_buf[i] *= ftmp;*/
                dtc_out[i] = round_fx(L_shl(L_mult(dtc_out[i], tmp16),3));

            }

            FOR(; i < BIN_4KHZ; i++)
            {
                tmp16 = s_min(max_ovf_4k,  filt_lfE[i]);
                tmp16 = s_max(min_g_4k, tmp16);
                /*DCT_buf[i] *= ftmp;*/
                dtc_out[i] = round_fx(L_shl(L_mult(dtc_out[i], tmp16),3));
            }

            test();
            IF( sub(coder_type,AUDIO) != 0 || L_sub(core_brate,ACELP_8k85)  > 0 )
            {
                /* Do not modify HF when coded with GSC at LR, because the spectrum is just noise */
                FOR(; i < DCT_L_POST; i++)
                {
                    tmp16 = s_min(max_ovf_6k,  filt_lfE[i]);
                    tmp16 = s_max(min_g_6k, tmp16);
                    /*DCT_buf[i] *= ftmp;*/
                    dtc_out[i] = round_fx(L_shl(L_mult(dtc_out[i], tmp16),3));
                }
            }
        }
        ELSE IF( sub(*last_nonfull_music,25) > 0 )
        {
            /* When unsure on content type only slight clean-up allowed, no overshoot allowed */
            FOR(; i < DCT_L_POST; i++)
            {
                tmp16 = min(4096,  filt_lfE[i]);
                /*DCT_buf[i] *= ftmp;*/
                dtc_out[i] = round_fx(L_shl(L_mult(dtc_out[i], tmp16),3));
            }
        }
    }
}

/*---------------------------------------------------------------------------*
 * spectrum_mod_dct()
 *
 * spectrum enhancement according to the output of signal_type_clas()
 *---------------------------------------------------------------------------*/

static void spectrum_mod_dct_fx(
    const Word16 Qdct,        /* i  : scaling factor                               Q0       */
    Word16 data[],      /* i/o: dct spectrum                                          */
    const Word32 lf_E[],      /* i:   per bin E for first 46 bins (without DC)    2*Qdct+10 */
    Word32 lf_EO[],      /* i/o: old per bin E for previous half frame      2*Qdct+10 */
    const Word32 fr_bands[],   /* i:   per band input energy (contains 2 vectors) 2*Qdct+10 */
    Word32 enrO[],       /* i/o: per band old input energy                  2*Qdct+10 */
    const Word32 noiseE[],     /* i:   per band background noise energy estimate  2*Qdct+10 */
    const Word16 minGain,      /* i:   minimum suppression gain                   Q15 */
    Word16 lp_gbin[],    /* i/o: Smoothed suppression gain, per FFT bin     Q14*/
    const Word16 music_flag,   /* i:   music ? 1:0                                    */
    Word16 min_band,     /* i  : minimum band                                   */
    const Word16 MAX_GN,       /* i  : Maximum gain overshoot                         */
    const Word16 MAX_band      /* i  : minimum band                                   */
)
{
    Word32 maxNoise;
    Word32 binE[VOIC_BINS_HR], Lgain;
    Word16 gain = 0, minE;
    Word16 freq, slope, m_invno[MBANDS_GN_LD],e_invno[MBANDS_GN_LD];
    Word16 *pt_gbin, alpha, tmpN;
    Word16 i;
    Word32 Ltmp;
    Word16 scaling;
    Word16 tmp_snr;
    Word16 *pt;
    Word16 wtmp;
    Word16 e_tmp;
    Word16 m_binE, e_binE;
    Word16 e_gain;
    Word16 sqrt_gain;
    Word32 Lshift;
    Word32 dot5_scaled;
    const Word32 *Lpt2;

    gain = 0;
    move16();

    /*-----------------------------------------------------------------*
     * Compute the inverse of noise
     *-----------------------------------------------------------------*/

    scaling = add(shl(Qdct,1),10);
    dot5_scaled = L_shl(1, sub(scaling,1));
    FOR ( i=0; i<=MBANDS_GN_LD-1; i++)
    {
        /*inv_noise[i] = 1.0f / noiseE[i];*/
        IF (noiseE[i] != 0)
        {
            e_invno[i] = norm_l(noiseE[i]);
            move16();
            m_invno[i] = extract_h(L_shl(noiseE[i],e_invno[i]));
            e_invno[i] = sub(14, e_invno[i]);
            move16();
            m_invno[i] = div_s(16384, m_invno[i]);
            move16();
        }
        ELSE
        {
            /* noiseE[i] == 0 only if Q_new <0*/
            e_invno[i] = add(-16,Qdct);
            move16();
            m_invno[i] = MAX_16;
            move16();
        }
    }
    /*----------------------------------------------------------------------*
     * Perform noise reduction for 1 frames
     *----------------------------------------------------------------------*/
    FOR (i=0 ; i < VOIC_BINS_HR ; i++)
    {
        /*binE[i] = (float)(0.3 * lf_EO[i] + 0.7 * lf_E[i]);*/
        Ltmp = Mult_32_16(lf_EO[i], 9830);
        binE[i] = Madd_32_16(Ltmp, lf_E[i], 22938);
        move32();
    }
    Copy32( lf_E, lf_EO, VOIC_BINS_HR ); /* update */
    /*----------------------------------------------------------------------*
     * Find the maximum noise in a critical band
     *----------------------------------------------------------------------*/

    maxNoise = L_max(noiseE[0], noiseE[1]);
    FOR (i=2; i<=MBANDS_GN_LD-1; i++)
    {
        maxNoise = L_max(maxNoise, noiseE[i]);
    }

    /* pointer initialization */
    pt = &data[0];

    /*-----------------------------------------------------------------*
     * Initialization for active speech frames or VAD hangover frames,
     * (exclude Clean speech)
     *-----------------------------------------------------------------*/

    IF ( music_flag != 0 ) /* prevent subtraction on clean speech */
    {
        IF( L_sub(maxNoise, L_shl(10, scaling)) <= 0)
        {
            minE = 18432/2;      /*Q14*/ move16();
        }
        ELSE
        {
            minE = shr(mult_r(minGain, minGain),1);   /*Q14*/
        }

        Lpt2 = binE;
        freq = 0;
        move16();

        pt_gbin = lp_gbin;
        FOR (i=0; i < min_band; i++)
        {

            FOR (; freq <= mfreq_loc_LD_fx[i]; freq += BIN_16kdct_fx)
            {
                Lpt2++;
                /* Lgain is already saturate if it's > 1*/
                pt++;
                *pt_gbin = 16384;
                move16();
                pt_gbin++;
            }
        }
        /*-----------------------------------------------------------------*
         * Per Frequency BIN processing
         * For highly voiced and highly pitched speech, use per bin
         * subtraction in low frequencies (maximum up to 3700 Hz,
         * first 17 critical bands)
         *-----------------------------------------------------------------*/

        FOR (; i < MAX_band; i++)
        {

            /*tmp = INV_MAX_SNR_tab[i];
            slope =  tmp - tmp * minE;
            shift =  MAX_SNR_SNR1_tab[i] * minE - tmp;*/

            /*tmp = 1.0f/ (MAX_SNR1 - 1.0f);*/
            tmp_snr = INV_MAX_SNR_tab_FX[i];
            move16();
            /*slope = -tmp * minE + tmp;*/
            Ltmp = L_mult(tmp_snr, 16384);
            slope = msu_r(Ltmp,tmp_snr, minE); /*Q14*/

            /*shift = MAX_SNR1 * tmp * minE - tmp;*/
            Ltmp = L_mult(MAX_SNR_SNR1_tab_FX[i], minE);  /*Q14*Q14*/
            Lshift = L_msu(Ltmp, tmp_snr,8192);  /*Q15*Q13+Q29*/
            Lshift = L_shl(Lshift,1);   /*Q29 -> Q30*/

            /*tmpN = slope * inv_noise[i];*/
            tmpN = mult(slope, m_invno[i]);
            e_tmp = e_invno[i];
            move16();

            /*while (freq <= mfreq_loc_LD[i])*/
            FOR (; freq <= mfreq_loc_LD_fx[i]; freq += BIN_16kdct_fx)
            {
                /*gain = 1.0f;*/
                Lgain = L_deposit_h(16384);
                /*if (noiseE[i] >= 0.5f)*/
                IF (L_sub(noiseE[i], dot5_scaled) > 0 )/* Do not alter if noise E very low */
                {
                    /*gain = tmpN * *pt2 + shift;*/ /* limits: [x,y] = {[1, minE], [MAX_SNR1, 1]}, */
                    e_binE = norm_l(*Lpt2);
                    m_binE = extract_h(L_shl(*Lpt2, e_binE));

                    e_binE = sub(e_binE,0); /* lf_e divided by 4 in anal_sp*/

                    Ltmp = L_mult(tmpN, m_binE);
                    e_binE = sub(add(e_tmp, e_binE),15);
                    Ltmp = L_shr(Ltmp, e_binE);
                    Lgain = L_add(Ltmp, Lshift); /*Saturation can occure here result in Q30*/
                }

                Lpt2++;
                gain = round_fx(Lgain);  /*gain in Q30-16 = Q14*/
                /*if (gain < minE)gain = minE;*/
                gain = s_max(gain,minE);
                /*if (gain > 1.0f+MAX_GN)gain = 1.0f+MAX_GN;*/
                gain = s_min(gain,add(16384, MAX_GN));

                /* prepare gain to find sqrt */
                e_gain = norm_s(gain);
                Ltmp = L_shl(gain, add(16,e_gain));
                e_gain = negate(sub(e_gain,1));

                Ltmp = Isqrt_lc(Ltmp, &e_gain);
                wtmp = extract_h(Ltmp);
                sqrt_gain = div_s(16384,wtmp);


                /* the gain smoothing control: stronger lp filtering for lower gains */
                /*alpha = 1.0f - (float)sqrt(gain);*/
                /* keep gain in Q14*/
                sqrt_gain = shr(sqrt_gain, e_gain);
                /*alpha = 1.0f - gain;*/ /* the gain smoothing control: stronger LP filtering for lower gains */
                alpha = shl(sub(16384, sqrt_gain),1);

                /**pt_gbin = gain + alpha * *pt_gbin;*/
                Ltmp = L_mult(gain, 32767);
                *pt_gbin = round_fx(L_mac(Ltmp, alpha, *pt_gbin));  /*Q14*/
                /**pt++ *= *pt_gbin;*/
                *pt =  round_fx(L_shl(L_mult(*pt, *pt_gbin),1));
                pt++;
                pt_gbin++;
            }
        }
    }
    ELSE
    {
        freq = BIN_16kdct_fx;
        move16();
        pt_gbin = lp_gbin;
        move16();
        FOR (i=0; i < MBANDS_GN_LD; i++)
        {
            FOR (; freq <= mfreq_loc_LD[i]; freq += BIN_16kdct_fx)
            {
                /**pt_gbin = 0.9f* *pt_gbin + 0.1f;*/
                *pt_gbin = round_fx(L_mac(L_mult(29491, *pt_gbin), 32767, 1638));
                pt_gbin++;
            }
        }
    }

    /* Old energy Update */
    Copy32(fr_bands, enrO, MBANDS_GN_LD);

}


/*----------------------------------------------------------------------------------*
 * analy_sp_dct()
 *
 * Spectral analysis of the current synthesized frame
 *----------------------------------------------------------------------------------*/

static void analy_sp_dct_fx(
    const Word16 *dct_buf, /* i  input dct spectrum                             */
    Word32 *fr_bands,     /* o:  energy in critical frequency bands 2*Qdct+10   */
    Word32 *lf_E,         /* o:  per bin E for first...             2*Qdct+10 */
    Word16 *etot,         /* o:  total input energy                Q8 */
    const Word16 Qdct     /* i:  Scaling of dct                       */
)
{
    Word32 Letot = 0;
    Word16 exp_etot, frac_etot;

    Letot = L_deposit_l(0);
    /*-----------------------------------------------------------------*
     * find energy per critical frequency band and total energy in dB
     *-----------------------------------------------------------------*/
    find_enr_dct_fx(dct_buf, fr_bands, lf_E, &Letot, 0, MBANDS_GN_LD, Qdct, BIN_16kdct_fx );

    /* find average log total energy over both half-frames */
    /**etot = 10.0f * (float)log10(*etot) - 3.0103f;*/
    exp_etot = norm_l(Letot);
    frac_etot = Log2_norm_lc(L_shl(Letot, exp_etot));
    exp_etot = sub(30, exp_etot);
    exp_etot = sub(exp_etot, add(shl(Qdct,1),10+1)); /* +(1) */
    Letot = Mpy_32_16(exp_etot, frac_etot, LG10);
    /* Q8 Averaged the total energy over both half-frames in log10 */
    *etot = extract_l(L_shr(Letot, 14-8));

    return;
}

/*------------------------------------------------------------------------*
 * find_enr_dct)
 *
 * find input signal energy for each critical band and first 74 LF bins
 * The energy is normalized by the number of frequency bins in a channel
 *------------------------------------------------------------------------*/

static void find_enr_dct_fx(
    const Word16 data[],    /* i  : dct result, for the format           Qdct*/
    Word32 band[],    /* o  : per band energy                     2*Qdct+10*/
    Word32 *ptE,      /* o  : per bin energy  for low frequencies 2*Qdct+10*/
    Word32 *Etot,     /* o  : total energy                        2*Qdct+10*/
    const Word16 min_band,  /* i  : minimum critical band                Q0  */
    const Word16 max_band,  /* i  : maximum critical band                Q0  */
    const Word16 Q_dct,     /* i  : scaling factor                       Q0  */
    const Word16 bin_freq   /* i  : Number of frequency bins             Q0  */
)
{
    Word16 i;
    Word16 freq;
    const Word16 *ptR;
    Word32 LE_min, Ltmp, Ltmp1;

    LE_min = L_max(L_shl(E_MIN_Q15, sub(shl(Q_dct,1)+10,22)),1);

    ptR = &data[0];        /* pointer to first real coefficient */
    freq = 0;
    move16();
    FOR( i=0; i < max_band; i++ )
    {
        band[i] = 0;
        move16();
        Ltmp1 = L_deposit_l(0);
        FOR(; freq <= mfreq_loc_LD_fx[i]; freq +=  bin_freq)
        {
            /* energy  */
            /**ptE = *ptR * *ptR;           */
            Ltmp = L_mult(*ptR, *ptR);

            /* normalization */
            /**ptE *= 1.0f / (DCT_L_POST);*/
            Ltmp = Mult_32_16(Ltmp, 26214);    /*26214 = 1.0/640 ->Q15+9 --> 2*Q_dct + 9*/
            Ltmp = L_max(Ltmp, LE_min);
            *ptE = Ltmp;
            move32();

            /*band[i] += *ptE++;*/
            Ltmp1 = L_add(Ltmp, Ltmp1);

            ptE++;
            ptR++;
        }

        /* normalization per frequency bin */
        /*band[i] /= cnt;*/
        band[i] = L_max(Mult_32_16(Ltmp1, inv_mfreq_bindiv_LD_fx[i]), LE_min);
        move32();   /* 2*Q_dct + 9*/

    }

    /*-----------------------------------------------------------------*
    * Find the total energy over the input bandwidth
    *-----------------------------------------------------------------*/

    Ltmp = 0;
    move16();
    FOR( i = min_band; i <= NB_LIMIT_BAND; i++ )
    {
        /* total channel energy */
        Ltmp = L_add(band[i],Ltmp);
    }

    *Etot = Ltmp;
    move32();

    return;
}

/*------------------------------------------------------------------------*
 * Prep_music_postP()
 *
 * Performs the steps needed to do the music post processing
 *------------------------------------------------------------------------*/

void Prep_music_postP_fx(
    Word16 exc_buffer_in[],  /* i/o: excitation buffer   Q_exc*/
    Word16 dct_buffer_out[], /* o  : DCT output buffer   (qdct)*/
    Word16 filt_lfE[],       /* i/o: long term spectrum energy Q15?*/
    const Word16 last_core,        /* i  : last core  */
    const Word16 *pitch_buf,       /* i  : current frame pitch information Q6*/
    Word16 *LDm_enh_lp_gbin, /* o  : smoothed suppression gain, per dct bin Q14*/
    const Word16 Q_exc,            /* i  : excitation scaling         */
    Word16 *qdct             /* o  : Scaling factor of dct coefficient */
)
{
    Word16  i;
    Word16 *pt1;
    const Word16 *pt2;
    Word16 s_pit, fr_pit;

    Word16 exc16[DCT_L_POST];
    Word16 *pt1_out;

    s_pit = shr(pitch_buf[3],6);
    fr_pit = shr(sub(pitch_buf[3], shl(s_pit,6)),4);   /* Find fractional part */

    /*------------------------------------------------------------*
     * Resetting some memories in case of switching
     *------------------------------------------------------------*/

    IF( sub(last_core,HQ_CORE) == 0 )
    {
        set16_fx( filt_lfE, 4096, DCT_L_POST );
        set16_fx( LDm_enh_lp_gbin, 16384, VOIC_BINS_HR );
        pt1 = exc_buffer_in + OFFSET2 - 1;
        pt2 = pt1 + shr_r(pitch_buf[0],6);
        FOR( i = 0; i < OFFSET2; i++ )
        {
            *pt1 = *pt2;
            move16();
            pt1--;
            pt2--;
        }
    }

    /*------------------------------------------------------------*
     * Extrapolation of the last future part and windowing
     *------------------------------------------------------------*/
    pt1 = exc_buffer_in + DCT_L_POST - OFFSET2;
    pred_lt4(pt1, pt1, s_pit, fr_pit, OFFSET2, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
    /*------------------------------------------------------------*
     *  windowing right side
     *------------------------------------------------------------*/
    pt2 = post_dct_wind_fx;
    pt1_out = exc16 + DCT_L_POST - OFFSET2;
    FOR( i = 0; i < OFFSET2; i++ )
    {
        *pt1_out = mult_r(*pt1,*pt2);
        move16();
        pt1++;
        pt2++;
        pt1_out++;
    }
    /*------------------------------------------------------------*
     *  windowing left side
     *------------------------------------------------------------*/
    pt1 = exc_buffer_in;
    pt1_out = exc16;
    pt2--;
    FOR( i = 0; i < OFFSET2; i++ )
    {
        *pt1_out = mult_r(*pt1,*pt2);
        move16();
        pt1++;
        pt1_out++;
        pt2--;
    }
    /*------------------------------------------------------------*
     *  Copy remaining data (Flat part)
     *------------------------------------------------------------*/

    FOR(; i < L_FRAME+OFFSET2; i++ )
    {
        *pt1_out = *pt1;
        move16();
        pt1++;
        pt1_out++;
    }
    /*------------------------------------------------------------*
     *  EDCT and back to 16 bits
     *------------------------------------------------------------*/

    edct_16fx(exc16, dct_buffer_out,  DCT_L_POST, 6);
    *qdct = Q_exc;
    move16();

    return;
}
/*------------------------------------------------------------------------*
 * norm_lfe()
 *
 * Energy bins normalisation
 *------------------------------------------------------------------------*/
static Word16 norm_lfe(
    const Word32 Lfe,    /* i: energy bin to normalize 2*Qdct+10     */
    const Word16 m_norm, /* i: Normalisation factor Q e_norm         */
    const Word16 e_norm  /* i: Exponent of the normalisation factor  */
)
{
    Word32 Ltmp;
    Word16 exp2, tmp16, exp3 ;

    Ltmp = Mult_32_16(Lfe, m_norm);
    Ltmp = L_add(Ltmp, L_shl(BET_Q15_FX, sub(e_norm,15)));    /* Ltmp -> e_norm*/
    exp2 = norm_l(Ltmp);
    tmp16 = extract_h(L_shl(Ltmp,exp2));      /* exp2 -= 16 */
    exp2 = add(e_norm,exp2);

    tmp16 = mult_r(tmp16,tmp16);            /* tmp16 in Q exp2 */
    tmp16 = mult_r(tmp16,tmp16);
    tmp16 = mult_r(tmp16,tmp16);

    exp3 = sub(12+16, exp2);                /* tmp16 in Q exp2 */
    if(sub(exp2,31) !=0)
    {
        exp3 = sub(exp2, 12+16-3);          /* if exp2 < 31, means that tmp >= 1.0 */
        /* Need to shl by 3 to take into account the 3 multiplications */
    }
    tmp16 = shl(tmp16, exp3);               /* Result in Q12 */

    return tmp16;
}

/*------------------------------------------------------------------------*
 * Post_music_postP()
 *
 * Going back from frequency to time domain from the enhanced spectrum.
 * Retreive the aligned excitation and redo the synthesis
 *------------------------------------------------------------------------*/

void Post_music_postP_fx(
    Word16 dct_buffer_in[],  /* i/o: excitation buffer */
    Word16 *exc2,            /* i/o: Current excitation to be overwriten */
    const Word16 *mem_tmp,         /* i  : previous frame synthesis memory     */
    Word16 *st_mem_syn2,     /* i/o: current frame synthesis memory      */
    const Word16 *Aq,              /* i  : LPC filter coefficients             */
    Word16 *syn,             /* i/o: 12k8 synthesis                      */
    Word16 *Q_exc,           /* i  : excitation scaling                  */
    Word16 *prev_Q_syn,      /* i  : previsous frame synthesis scaling   */
    Word16 *Q_syn,           /* i  : Current frame synthesis scaling     */
    Word16 *mem_syn_clas_estim_fx,  /* i  : old 12k8 synthesis used for frame classification*/
    const Word16 IsIO,             /* i: Flag to indicate IO mode */
    Word16 *mem_deemph,         /* i/o: speech deemph filter memory                 */
    Word16 *st_pst_old_syn_fx,        /* i/o:  psfiler                                     */
    Word16 *st_pst_mem_deemp_err_fx,  /* i/o:  psfiler                                     */
    Word16 *mem_agc,
    PFSTAT *pf_stat,         /* i/o:  All memories related to NB post filter      */
    const Word16 *tmp_buffer        /* tmp_buffer in Q-1 */
    ,Word16 *mem_tmp2         /* Temporary memory used with scale_syn */
)
{
    Word16 exc16[DCT_L_POST];

    /*------------------------------------------------------------------------*
     * Go back to time domain
     *------------------------------------------------------------------------*/

    edct_16fx( dct_buffer_in, exc16, DCT_L_POST, 6);

    Copy( exc16 + OFFSET2, exc2, L_FRAME);

    Copy( mem_tmp, st_mem_syn2, M );

    /*------------------------------------------------------------------------*
     * Perform the synthesis filtering using the enhanced excitation
     *------------------------------------------------------------------------*/
    IF(IsIO == 0)              /* Rescaling already done in IO mode */
    {
        Rescale_mem(*Q_exc, prev_Q_syn, Q_syn, st_mem_syn2,mem_syn_clas_estim_fx, 4,
                    mem_deemph, st_pst_old_syn_fx, st_pst_mem_deemp_err_fx, mem_agc,
                    pf_stat, 1, tmp_buffer
                   );
        Copy( st_mem_syn2, mem_tmp2, M );
    }

    syn_12k8_fx( L_FRAME, Aq, exc2, syn, st_mem_syn2, 1 , *Q_exc, *Q_syn);

    return;
}


