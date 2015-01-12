/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"

#include "prot_fx.h"
#include "rom_com_fx.h"

#include "stl.h"                /* required for wmc_tool */

/*-------------------------------------------------------------------*
 * DecodeSWBGenericParameters()
 *
 * Decoding of generic subband coding parameters
 *-------------------------------------------------------------------*/

static void DecodeSWBGenericParameters_fx(
    Decoder_State_fx *st_fx,                    /* i/o: decoder state structure                     */
    Word16 *lagIndices_fx,            /* o  : lowband index for each subband              */
    const Word16 nBands_search_fx,          /* i  : number of subbnads for SSearch              */
    const Word16 BANDS_fx,                  /* i  : total number of subbands per frame          */
    const Word16 *p2a_flags_fx,             /* i  : HF tonal flag                               */
    const Word16 hq_swb_clas_fx             /* i  : mode of operation HQ_NORMAL or HQ_HARMONIC  */
)
{
    Word16 sb;

    /* lag index for each subband (except last two) */
    FOR (sb = 0; sb < nBands_search_fx; sb++)
    {
        IF( sub(hq_swb_clas_fx, HQ_HARMONIC) ==0 )
        {
            lagIndices_fx[sb] = get_next_indice_fx(st_fx, bits_lagIndices_mode0_Har_fx[sb]);
            move16();
        }
        ELSE
        {
            IF( p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+sb] == 0 )
            {
                lagIndices_fx[sb] = get_next_indice_fx(st_fx, bits_lagIndices_fx[sb]);
                move16();
            }
            ELSE
            {
                lagIndices_fx[sb] = 0;
                move16();
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * DecodeSWBSubbands()
 *
 * Main routine for generic SWB coding
 *
 * High-frequency subbands are replicated based on the lowband signal using a lowband index denoting
 * the selected lowband subband as well as linear and logarithmic domain gains
 *-------------------------------------------------------------------*/

static void DecodeSWBSubbands_fx(
    Decoder_State_fx *st_fx,                /* i/o: decoder state structure          */
    Word32 *L_spectra,                /* i/o: MDCT domain spectrum             */
    Word16 QsL,                       /* i  : Q value of spectra               */
    const Word16 fLenLow_fx,                /* i  : lowband length                   */
    const Word16 fLenHigh_fx,               /* i  : highband length                  */
    const Word16 nBands_fx,                 /* i  : number of subbands               */
    const Word16 nBands_search_fx,          /* i  : number of search subbands        */
    const Word16 *sbWidth_fx,               /* i  : subband lengths                  */
    const Word16 *subband_offsets_fx,       /* i  : subband offsets                  */
    Word16 *lagIndices_fx,            /* i  : lowband index for each subband   */
    Word16 *lagGains_fx,              /* i  : first gain for each subband      */
    Word16 *QlagGains,                /* i  : Q value of lagGains              */
    Word16 BANDS_fx,                  /* i  : number subbands per frame        */
    Word16 *band_start_fx,            /* i  : band start of each SB            */
    Word16 *band_end_fx,              /* i  : band end of each SB              */
    Word32 *L_band_energy,            /* i  : band energy of each SB           */
    Word16 Qbe,                       /* i  : Q value of band energy           */
    Word16 *p2a_flags_fx,             /* i  : HF tonal indicator               */
    const Word16 hqswb_clas_fx,             /* i  : class information                */
    const Word16 har_bands_fx,              /* i  : number of LF harmonic bands      */
    const Word16 *subband_search_offset_fx, /* i  : Number of harmonic LF bands      */
    Word16 *prev_frm_hfe2_fx,         /* i/o:                                  */
    Word16 *prev_stab_hfe2_fx,        /* i/o:                                  */
    Word16 band_width_fx[],           /* i  : subband band widths              */
    const Word32 L_spectra_ni[],            /* i/o: core coder with sparseness filled */
    Word16 *ni_seed_fx                /* i/o: random seed for search buffer NI  */
)
{
    Word16 i,k;

    Word16 sspectra_fx[L_FRAME32k];
    Word16 Qss;

    Word16 sspectra_ni_fx[L_FRAME32k], sspectra_diff_fx[L_FRAME32k];
    Word32 L_be_tonal[SWB_HAR_RAN1];
    Word16 ss_min_fx; /* Qss */
    Word32 L_th_g[NB_SWB_SUBBANDS];
    Word16 QbeL;
    GainItem_fx pk_sf_fx[N_NBIGGEST_SEARCH_LRG_B];
    Word16 pul_res_fx[NB_SWB_SUBBANDS];
    Word16 g_fx;       /* Q11 */
    Word16 imin_fx;
    Word16 Qg;

    Word16 lagIndices_real_fx[NB_SWB_SUBBANDS];
    Word32 L_xSynth_har[L_FRAME32k]; /* Qs */

    Word32 L_temp;
    Word16 temp_lo_fx, temp_hi_fx;

    Word16 har_freq_est1;
    Word16 har_freq_est2;
    Word16 flag_dis;
    Word16 pos_max_hfe2;


    har_freq_est1 = 0;
    move16();
    har_freq_est2 = 0;
    move16();
    flag_dis = 1;
    move16();
    pos_max_hfe2 = 0;
    move16();

    set16_fx(pul_res_fx,0,NB_SWB_SUBBANDS);

    IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
    {
        pos_max_hfe2 = har_est_fx( L_spectra, fLenLow_fx, &har_freq_est1, &har_freq_est2, &flag_dis, prev_frm_hfe2_fx, subband_search_offset_fx, sbWidth_fx, prev_stab_hfe2_fx );
        noise_extr_corcod_fx(L_spectra, L_spectra_ni, sspectra_fx, sspectra_diff_fx, sspectra_ni_fx, fLenLow_fx, st_fx->prev_hqswb_clas_fx, &(st_fx->prev_ni_ratio_fx), &Qss);
        IF( flag_dis == 0 )
        {
            test();
            if( sub(har_freq_est2, SWB_HAR_RAN1) != 0 || sub(har_freq_est2, *prev_frm_hfe2_fx) != 0 )
            {
                har_freq_est2 = add(har_freq_est2, lagIndices_fx[0]);
            }
        }

        /* Generate HF noise */
        genhf_noise_fx(sspectra_diff_fx, Qss, L_xSynth_har, QsL, sspectra_fx, BANDS_fx, har_bands_fx, har_freq_est2, pos_max_hfe2, pul_res_fx, pk_sf_fx, fLenLow_fx, fLenHigh_fx, sbWidth_fx, lagIndices_fx, subband_offsets_fx, subband_search_offset_fx);

        imin_fx = get_next_indice_fx(st_fx, 2);
        move16();
        /* g= pow(10.0f, gain, table[imin]) */
        L_temp = L_mult(gain_table_fx[imin_fx], 27213); /* Q14+Q13+1=Q28 log(10)/log(2)=3.3219 27213.23(Q13) */
        L_temp = L_shr(L_temp, 12);    /* Q28-Q12 -> Q16 */
        temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
        Qg = sub(14, temp_hi_fx);
        g_fx = extract_l(Pow2(14, temp_lo_fx));
        g_fx = shl(g_fx, sub(11, Qg));

        /* tonal energy estimation */
        ton_ene_est_fx(
            L_xSynth_har, QsL, L_be_tonal, &QbeL, L_band_energy, Qbe,
            band_start_fx, band_end_fx, band_width_fx, fLenLow_fx, fLenHigh_fx,
            BANDS_fx, har_bands_fx, g_fx, pk_sf_fx, Qss, pul_res_fx
        );

        /*HF Spectrum Generation*/
        Gettonl_scalfact_fx(
            L_xSynth_har, QsL, L_spectra_ni, fLenLow_fx, fLenHigh_fx, har_bands_fx, BANDS_fx, L_band_energy, Qbe, band_start_fx, band_end_fx,
            p2a_flags_fx, L_be_tonal, QbeL, pk_sf_fx ,Qss, pul_res_fx);

        IF( flag_dis == 0 )
        {
            *prev_frm_hfe2_fx = 0;
            move16();
        }
        ELSE
        {
            *prev_frm_hfe2_fx = har_freq_est2;
            move16();
        }

        FOR( k = har_bands_fx; k < BANDS_fx; k++ )
        {
            FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
            {
                L_spectra[i] = L_xSynth_har[i-fLenLow_fx];
                move32(); /* QsL */
            }
        }
    }
    ELSE IF ( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
    {
        ss_min_fx = spectrumsmooth_noiseton_fx(
                        L_spectra, /*QsL,*/ L_spectra_ni, sspectra_fx, sspectra_diff_fx, sspectra_ni_fx, &Qss, fLenLow_fx, ni_seed_fx);

        convert_lagIndices_pls2smp_fx( lagIndices_fx, nBands_search_fx, lagIndices_real_fx, sspectra_fx, sbWidth_fx, fLenLow_fx );
        FOR (k = 0; k < nBands_search_fx; k++)
        {
            if ( sub(p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+k], 1) == 0 )
            {
                lagIndices_real_fx[k] = 0;
                move16();
            }
        }

        GetlagGains_fx( sspectra_ni_fx, Qss,
                        &L_band_energy[BANDS_fx-NB_SWB_SUBBANDS], Qbe,
                        nBands_fx, sbWidth_fx, lagIndices_real_fx, fLenLow_fx, lagGains_fx, QlagGains );

        FOR(k=0; k<nBands_fx; k++)
        {
            IF ( sub(p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+k], 1) == 0 )
            {
                lagGains_fx[k] = 0;
                move16();
                QlagGains[k] = 15;
                move16();
            }
            ELSE
            {
                lagGains_fx[k] = mult_r(lagGains_fx[k], 29491);  /* lagGains[k]*0.9f; */
            }
        }

        FOR(k=0; k<NB_SWB_SUBBANDS; k++)
        {
            L_th_g[k] = L_deposit_l(0);
            IF(p2a_flags_fx[BANDS_fx-NB_SWB_SUBBANDS+k] == 0)
            {
                L_th_g[k] = L_shl( L_mult(lagGains_fx[k], ss_min_fx), sub(QsL, add(add(QlagGains[k], Qss), 1))); /* QlagGain+Qss -> Qs */
            }
        }

        /* Construct spectrum */
        GetSynthesizedSpecThinOut_fx(
            sspectra_ni_fx, Qss, L_xSynth_har, QsL, nBands_fx, sbWidth_fx,
            lagIndices_real_fx, lagGains_fx, QlagGains, fLenLow_fx
        );

        /* Level adjustment for the missing bands */
        noiseinj_hf_fx(
            L_xSynth_har, QsL, L_th_g, L_band_energy, Qbe, st_fx->prev_En_sb_fx,
            p2a_flags_fx, BANDS_fx, band_start_fx, band_end_fx, fLenLow_fx, fLenHigh_fx
        );

        FOR( k = sub(BANDS_fx, NB_SWB_SUBBANDS); k < BANDS_fx; k++ )
        {
            IF( p2a_flags_fx[k] == 0 )
            {
                FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
                {
                    L_spectra[i] = L_xSynth_har[i-fLenLow_fx];
                    move32(); /* QsL */
                }
            }
            ELSE
            {
                FOR( i = band_start_fx[k]; i <= band_end_fx[k]; i++ )
                {
                    L_spectra[i] = L_spectra_ni[i];
                    move32();
                }
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * swb_bwe_dec_lr()
 *
 * Main decoding routine of SWB BWE for the LR MDCT core
 *-------------------------------------------------------------------*/
void swb_bwe_dec_lr_fx(
    Decoder_State_fx *st_fx,                 /* i/o: decoder state structure                     */
    const Word32 L_m_core[],             /* i  : lowband synthesis                           */
    const Word16 QsL,                    /* i  : Q value of m_core                           */
    Word32 L_m[],                  /* o  : highband synthesis with lowband zeroed      */
    const Word32 L_total_brate,          /* i  : total bitrate for selecting subband pattern */
    Word16 BANDS_fx,               /* i  : Number subbands/Frame                       */
    Word16 *band_start_fx,         /* i  : Band Start of each SB                       */
    Word16 *band_end_fx,           /* i  : Band end of each SB                         */
    Word32 *L_band_energy,         /* i  : Band energy of each SB : Qbe                */
    Word16 Qbe,                    /* i  : Q value of band energy                      */
    Word16 *p2a_flags_fx,          /* i  : HF tonal Indicator                          */
    const Word16 hqswb_clas_fx,          /* i  : class information                           */
    Word16 lowlength_fx,           /* i  : Lowband Length                              */
    Word16 highlength_fx,          /* i  : Highband Length                             */
    const Word16 har_bands_fx,           /* i  : Number of LF harmonic bands                 */
    Word16 *prev_frm_hfe2_fx,      /* i/o:                                             */
    Word16 *prev_stab_hfe2_fx,     /* i/o:                                             */
    Word16 band_width_fx[],        /* i  : subband bandwidth                           */
    const  Word32 L_y2_ni[],              /* i/o: Sparse filled corecoder                     */
    Word16 *ni_seed_fx             /* i/o: random seed                                 */
)
{
    Word16 k;
    Word16 nBands_fx;
    Word16 nBands_search_fx;
    Word16 wBands_fx[NB_SWB_SUBBANDS];
    Word16 lagIndices_fx[NB_SWB_SUBBANDS];
    Word16 lagGains_fx[NB_SWB_SUBBANDS];
    Word16 QlagGains[NB_SWB_SUBBANDS];
    Word16 swb_lowband_fx, swb_highband_fx, allband_fx;

    const Word16 *subband_offsets_fx;
    const Word16 *subband_search_offset_fx;

    Word32 *p_L_m;

    subband_search_offset_fx = subband_search_offsets_13p2kbps_Har_fx;
    subband_offsets_fx = subband_offsets_sub5_13p2kbps_Har_fx;

    hf_parinitiz_fx(L_total_brate,hqswb_clas_fx,lowlength_fx,highlength_fx,wBands_fx,&subband_search_offset_fx,&subband_offsets_fx,&nBands_fx,&nBands_search_fx,&swb_lowband_fx,&swb_highband_fx);
    allband_fx = add(swb_lowband_fx, swb_highband_fx);
    move16();

    /* Decoding of the SWB parameters */
    DecodeSWBGenericParameters_fx( st_fx, lagIndices_fx, nBands_search_fx, BANDS_fx, p2a_flags_fx, hqswb_clas_fx );

    /* Copy WB synthesis for SWB decoding */
    Copy32( L_m_core, L_m, swb_lowband_fx + swb_highband_fx );

    /* Generic subband processing */
    DecodeSWBSubbands_fx(
        st_fx,
        L_m, QsL,
        swb_lowband_fx, swb_highband_fx, nBands_fx, nBands_search_fx, wBands_fx, subband_offsets_fx,
        lagIndices_fx, lagGains_fx, QlagGains,
        BANDS_fx, band_start_fx, band_end_fx,
        L_band_energy, Qbe,
        p2a_flags_fx, hqswb_clas_fx, har_bands_fx, subband_search_offset_fx,
        prev_frm_hfe2_fx, prev_stab_hfe2_fx, band_width_fx, L_y2_ni, ni_seed_fx
    );

    p_L_m = &L_m[sub(allband_fx, 1)];
    *p_L_m = Mult_32_16(*p_L_m,  2028);
    move32();
    p_L_m--; /* 0.0625 =  2028 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m,  4096);
    move32();
    p_L_m--; /* 0.125  =  4096 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m,  8192);
    move32();
    p_L_m--; /* 0.25   =  8192 (Q15) */
    *p_L_m = Mult_32_16(*p_L_m, 16384);
    move32();
    p_L_m--; /* 0.5    = 16384 (Q15) */

    /* set low frequencies to zero */
    FOR ( k = 0; k < swb_lowband_fx; k++ )
    {
        L_m[k] = L_deposit_l(0);
    }

    return;
}
