/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * find_tilt()
 *
 * Find LF/HF energy ratio
 *-------------------------------------------------------------------*/

void find_tilt_fx(
    const Word32 fr_bands[],  /* i  : energy in frequency bands						Q_new + Q_SCALE*/
    const Word32 bckr[],      /* i  : per band background noise energy estimate		Q_new + Q_SCALE*/
    Word32 ee[2],       /* o  : lf/hf E ration for present frame							 Q6*/
    const Word16 pitch[3],    /* i  : open loop pitch values for 3 half-frames					 Q0*/
    const Word16 voicing[3],  /* i  : normalized correlation for 3 half-frames					Q15*/
    const Word32 *lf_E,       /* i  : per bin energy  for low frequencies         Q_new + Q_SCALE - 2*/
    const Word16 corr_shift,  /* i  : normalized correlation correction							Q15*/
    const Word16 bwidth,      /* i  : input signal bandwidth                                         */
    const Word16 max_band,    /* i  : maximum critical band                                          */
    Word32 hp_E[],      /* o  : energy in HF                                    Q_new + Q_SCALE*/
    const Word16 codec_mode,  /* i  : MODE1 or MODE2                                      */
    const Word16 Q_new,       /* i  : scaling factor												   */
    Word32 *bckr_tilt_lt /* i/o: lf/hf E ratio of background noise  Q16                        */
    ,Word16 Opt_vbr_mode
)
{
    Word32 lp_bckr = 0, hp_bckr = 0,  lp_E, Ltmp, Lcnt;
    const Word32 *pt_E, *pt_bands, *pt_bckr, *hf_bands, *tmp_E;
    Word16 tmp, freq, f0, f1, f2, mean_voi, bin;
    Word16 i, nb_bands;
    Word16 e_tmp, m_tmp;
    Word16 m_Fs, e_Fs;
    Word16 m_cnt, e_cnt;
    Word16 m_hpE, e_hpE;
    Word16 scaling;

    /*-----------------------------------------------------------------*
     * Initializations
     *-----------------------------------------------------------------*/

    scaling = add(Q_new, QSCALE);
    IF( sub(bwidth,NB) != 0 )
    {
        /* WB processing */
        bin = BIN4_FX;
        move16();                /* First useful frequency bin ~ 50 Hz     */
        pt_bands = fr_bands;
        tmp_E = lf_E;
        pt_bckr = bckr;
        nb_bands = 10;
        move16();
    }
    ELSE
    {
        /* NB processing */
        bin = add(shl(BIN4_FX,1), BIN4_FX);             /* First useful frequency bin ~ 150 Hz    */
        pt_bands = fr_bands+1;      /* Exlcude 1st critical band              */
        tmp_E = lf_E + 2;           /* Start at the 3rd bin (150 Hz)          */
        pt_bckr = bckr+1;           /* Exlcude 1st critical band              */
        nb_bands = 9;
        move16();            /* Nb. of "low" frequency bands taken into account in NB processing      */
    }

    /*-----------------------------------------------------------------*
     * Find spectrum tilt
     *-----------------------------------------------------------------*/

    pt_E = tmp_E;                   /* Point at the 1st useful element of the per-bin energy vector  */
    hf_bands = fr_bands;

    /* bckr + voicing */
    /*lp_bckr = mean( pt_bckr, nb_bands );*/                              /* estimated noise E in first critical bands, up to 1270 Hz */
    lp_bckr = Mean32(pt_bckr, nb_bands);
    /*hp_bckr = 0.5f * (bckr[max_band-1] + bckr[max_band]);*/             /* estimated noise E in last 2 critical bands */
    hp_bckr = L_shr(L_add(bckr[max_band-1] , bckr[max_band]),1);
    if (hp_bckr == 0)   /* Avoid division by zero. */
    {
        hp_bckr = L_deposit_l(1);
    }
    tmp = BASOP_Util_Divide3232_Scale( lp_bckr, hp_bckr, &e_tmp );
    Ltmp = L_shr_r( L_deposit_h( tmp ), sub( 15, e_tmp ) );
    *bckr_tilt_lt = L_add( Mpy_32_16_r( *bckr_tilt_lt, 29491 ), Mpy_32_16_r( Ltmp, 3277 ) );

    test();
    IF ( sub(codec_mode,MODE2) == 0 || Opt_vbr_mode == 1)
    {
        /*lp_bckr *= FACT;*/
        /*hp_bckr *= FACT;*/
        lp_bckr = L_add(L_shl(lp_bckr,1),lp_bckr);
        hp_bckr = L_add(L_shl(hp_bckr,1),hp_bckr);
    }
    /*mean_voi = 0.5f * (voicing[1] + voicing[2]) + corr_shift;*/
    Ltmp = L_mult(voicing[1], 16384);
    Ltmp = L_mac(Ltmp, voicing[2], 16384);
    Ltmp = L_mac(Ltmp,corr_shift, 32767);
    mean_voi = round_fx(Ltmp);

    /*f0 = INT_FS_FX / pitch[2];*/
    e_tmp = norm_s(pitch[2]);
    m_tmp = shl(pitch[2], e_tmp);

    m_Fs = div_s(INT_FS_FX, m_tmp);
    e_Fs = sub(15, e_tmp);
    f0 = shr(m_Fs, sub(e_Fs,4));  /* Q4 */

    FOR( i=0; i<2; i++ )
    {
        /*hp_E[i] = 0.5f * (hf_bands[max_band-1] + hf_bands[max_band]) - hp_bckr; *//* averaged E in last 2 critical bands */
        Ltmp  = L_add(L_shr(hf_bands[max_band-1], 1), L_shr(hf_bands[max_band], 1));
        hp_E[i] = L_sub(Ltmp, hp_bckr);
        IF ( Opt_vbr_mode == 0 )
        {
            hp_E[i] = L_max(hp_E[i], L_shl(E_MIN_FX, Q_new));
            move32();
        }
        ELSE
        {
            hp_E[i] = L_max(hp_E[i],  L_shl(1, scaling));
            move32();
        }

        test();
        IF(sub(mean_voi,TH_COR_FX) > 0 && sub(pitch[2], TH_PIT_FX) < 0)  /* High-pitched voiced frames */
        {
            freq = bin;
            move16();                           /* 1st useful frequency bin */
            Lcnt = L_deposit_l(0);
            lp_E = L_deposit_l(0);

            f1 = add(shr(f0,1),f0);                 /* Middle between 2 harmonics */
            f2 = f0;
            move16();
            WHILE(sub(freq, 20320) <= 0)             /* End frequency of 10th critical band */
            {
                FOR (; freq <= f1; freq += BIN4_FX)
                {
                    /* include only bins sufficiently close to harmonics */
                    tmp = sub(freq, f2);
                    BASOP_SATURATE_WARNING_OFF
                    Ltmp = L_shr(L_mac0(-(Word32)TH_D_FX*TH_D_FX, tmp, tmp), 32);
                    BASOP_SATURATE_WARNING_ON
                    /* Add & Increment Count only if Ltmp < 0 */
                    lp_E = L_add(L_and(*pt_E, Ltmp), lp_E);
                    Lcnt = L_sub(Lcnt, Ltmp);
                    pt_E++;
                }
                f1 = add(f1,f0);
                f2 = add(f2,f0);
            }
            /*lp_E = lp_E / (float)cnt - lp_bckr;*/
            e_tmp = sub(norm_l(lp_E), 1);
            m_tmp = extract_h(L_shl(lp_E, e_tmp));

            e_tmp = sub(e_tmp,2);                   /* lf_e divided by 4 in anal_sp */

            m_cnt = extract_l(Lcnt);
            e_cnt = norm_s(m_cnt);
            m_cnt = shl(m_cnt, e_cnt);

            m_tmp = div_s(m_tmp, m_cnt);
            e_tmp = sub(e_tmp, e_cnt);

            lp_E = L_sub(L_shr(m_tmp, sub(e_tmp, 1)), lp_bckr);


            pt_E = tmp_E + VOIC_BINS;             /* Update for next half-frame */
        }
        ELSE                                      /* Other than high-pitched voiced frames */
        {
            /*lp_E = mean( pt_bands, nb_bands ) - lp_bckr;*/ /* averaged E in first critical bands, up to 1270 Hz */
            lp_E = L_sub(Mean32( pt_bands, nb_bands ) , lp_bckr);
        }
        IF ( Opt_vbr_mode == 0 )
        {
            lp_E  = L_max(lp_E, L_shl(E_MIN_FX,Q_new));
        }
        ELSE
        {
            lp_E = L_max(lp_E,  0);
        }
        /*ee[i] = lp_E / hp_E[i];*/                      /* LF/HF ratio */
        test();
        IF (lp_E != 0 && hp_E[i] != 0)
        {
            e_tmp = sub(norm_l(lp_E), 1);
            m_tmp = extract_h(L_shl(lp_E, e_tmp));
            e_hpE = norm_l(hp_E[i]);
            m_hpE = extract_h(L_shl(hp_E[i], e_hpE));
            m_tmp = div_s(m_tmp, m_hpE);
            e_tmp = sub(e_tmp, e_hpE);

            ee[i] = L_shr(m_tmp, add(e_tmp, 15-6));    /* ee in Q6 */
        }
        ELSE IF (lp_E == 0)
        {
            ee[i] = L_deposit_l(0);
        }
        ELSE
        {
            ee[i] = MAX_32;
        }

        IF( sub(bwidth,NB) == 0 )                        /* For NB input, compensate for the missing bands */
        {
            Ltmp = L_shl(ee[i], 3);
            IF (L_sub(Ltmp, MAX_32) == 0)     /* if Overflow: Compute with less precision */
            {
                Ltmp = Mult_32_16(ee[i], 24576);               /* 6/8 */
                ee[i] = L_shl(Ltmp, 3);
                move32();  /* x8  */
            }
            ELSE
            {
                ee[i] = Mult_32_16(Ltmp, 24576);
                move32();/* 6/8  */
            }
        }

        pt_bands += NB_BANDS;                     /* Update for next half-frame */
        hf_bands += NB_BANDS;
    }

    return;
}
