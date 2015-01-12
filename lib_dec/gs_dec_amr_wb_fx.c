/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define ALP_FX          22938 /* 0.70f */
#define MALP_FX         (Word16)(32768L-ALP_FX)
#define ALPMY_FX        28180 /* 0.86f */

#define BAND3k          15
#define BIN_1k2         48
#define BAND_2k         12
#define BAND_0k4        4

#define NORMALIZE_SPECS_Q_OUT 3/*6//7//8*/ /*9*/
#define ENER_FX_Q_GUARD 1

/*-------------------------------------------------------------------*
 * NoiseFill_fx()
 *
 * noise fill function for unvoiced/inactive frames (used only in AMR-WB IO mode)
 *-------------------------------------------------------------------*/

static void NoiseFill_fx(
    Word16 *exc_diffQ_fx,          /* i/o: Noise per band                          */
    Word16 *seed_tcx,              /* i  : Seed for noise                          */
    const Word16 Mbands_gn,              /* i  : number of bands                         */
    const Word16 Q_out                   /* i  : Q of exc_diffQ_fx[]                     */
)
{
    Word16 i_band, CurBin, EndBin;
    Word32 L_temp;
    Word16 fact;
    CurBin = 0;
    move16();
    fact = shr(24576/*0.75f*/, sub(15, Q_out));

    FOR( i_band = 0; i_band < Mbands_gn; i_band++ )
    {
        EndBin = add(CurBin, crit_bins[i_band]);
        FOR( ; CurBin<EndBin; CurBin++ )
        {
            L_temp = L_mult(Random(seed_tcx), fact);
            L_temp = L_msu(L_temp, exc_diffQ_fx[CurBin], -32768);
            exc_diffQ_fx[CurBin] = round_fx(L_temp);
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * Ener_per_band_fx()
 *
 * Computed the energy per band (used only in AMR-WB IO mode)
 *-------------------------------------------------------------------*/

static void Ener_per_band_fx(
    const Word16 exc_diff_fx[], /* i  : target signal                     */
    const Word16 exc_diff_exp,  /* i  : Exponent of exc_diff_fx           */
    Word32 y_gain4_fx[]   /* o  : Energy per band to quantize Q16   */
)
{
    const Word16 *ptr16;
    Word16 i,j;
    Word32 L_temp, L_temp2, L_epsilon;
    Word16 temp, exp;

    exp = shl(exc_diff_exp, 1);
    /* To Get 0.01f in Q of (exc_diff_exp-1)^2 */
    L_epsilon = L_shr(42949673L, sub(32+ENER_FX_Q_GUARD*2, exp));

    ptr16 = exc_diff_fx;
    FOR (j = 0; j < CRIT_NOIS_BAND; j++)
    {
        temp = shr(*ptr16++, ENER_FX_Q_GUARD);
        L_temp = L_mac0(L_epsilon, temp, temp);
        FOR (i = 1; i < crit_bins[j]; i++)
        {
            temp = shr(*ptr16++, ENER_FX_Q_GUARD);
            L_temp = L_mac0(L_temp, temp, temp);
        }
        L_temp2 = L_mult0(1, 6554); /* sqrt of 0.01f in Q16*/
        IF (L_temp != 0) /* avoid executing sqrt of 0 (because a div_s is used to invert and then call inv_sqrt) */
        {
            L_temp2 = Sqrt_Ratio32(L_temp, exp, 1, 0, &i);
            L_temp2 = L_shr(L_temp2, sub(15-ENER_FX_Q_GUARD, i));
        }
        y_gain4_fx[j] = L_temp2;
        move32();
    }

    return;
}


/*-------------------------------------------------------------------*
 * Apply_gain_fx()
 *
 * Rescaling of the modified excitation vector (used only in AMR-WB IO mode)
 *-------------------------------------------------------------------*/

static void Apply_gain_fx(
    Word16 exc_diffQ_fx[],   /* i/o: Quantized excitation              */
    Word32 L_Ener_per_bd_iQ[],/* i  : Target ener per band              */
    Word32 L_Ener_per_bd_yQ[] /* i  : Ener per band for norm vector     */
)
{
    Word16 i_band;
    Word16 CurBin, EndBin;
    Word16 y_gain_fx, y_gain_exp;
    Word16 exp3;
    Word32 L_temp;

    /*------------------------------------------------------------------
     * For all the bands
     * Find the energy ratio between modified vector and original vector
     *------------------------------------------------------------------*/

    CurBin = 0;
    move16();
    FOR(i_band = 0; i_band < CRIT_NOIS_BAND; i_band++)
    {
        EndBin = add(CurBin, crit_bins[i_band]);
        y_gain_exp = norm_l(L_Ener_per_bd_yQ[i_band]);
        exp3 = norm_l(L_Ener_per_bd_iQ[i_band]); /* use 'exp3' as temporary exponent of 'L_Ener_per_bd_iQ[]' */
        y_gain_fx = round_fx(Div_flt32_flt32(L_shl(L_Ener_per_bd_iQ[i_band], exp3), exp3,
                                             L_shl(L_Ener_per_bd_yQ[i_band], y_gain_exp), y_gain_exp, &y_gain_exp));
        y_gain_exp = sub(y_gain_exp, 31);
        exp3 = sub(y_gain_exp, 16-1);

        /*------------------------------------------------------------------
         * For bands below 400 Hz or for unvoiced/inactive frames
         *   only apply the energy ratio
         *------------------------------------------------------------------*/
        {
            FOR(; CurBin < EndBin; CurBin++)
            {
                L_temp = L_mult(exc_diffQ_fx[CurBin], y_gain_fx);
                L_temp = L_shr(L_temp, y_gain_exp);
                exc_diffQ_fx[CurBin] = round_fx(L_temp);
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * normalize_spec_fx()
 *
 * Spectrum normalization (zeroed of bins below a certain threshold) (used only in AMR-WB IO mode)
 *-------------------------------------------------------------------*/

static void normalize_spec_fx(
    Word16 fac_up_fx,        /* i  : Core bitrate (Q8)                  */
    Word16 fac_dwn_fx,       /* i  : Core bitrate (Q8)                  */
    Word16 fy_norm_fx[],     /* i/o: Frequency quantized parameter (Q8) */
    const Word16 L_frame,          /* i  : Section lenght                     */
    const Word16 Q_out             /* i  : Q of fy_norm_fx[]                  */
)
{
    Word16 idx, j;
    Word32 L_temp;
    Word16 temp, exp;

    idx = emaximum_fx(0/* Exponent is not Important */, fy_norm_fx, L_frame, &L_temp);
    exp = sub(Q_out, 8/*Q8 of Fac Up/down*/);
    /* Bring fac_dwn to Proper Q */
    fac_dwn_fx = shl(fac_dwn_fx, exp);
    temp = Invert16(abs_s(fy_norm_fx[idx]), &exp);
    L_temp = L_mult(temp, fac_up_fx);
    exp = sub(15, exp);
    L_temp = L_shl(L_temp, exp);

    FOR (j = 0; j < L_frame; j++)
    {
        fy_norm_fx[j] = round_fx(Mult_32_16(L_temp, fy_norm_fx[j]));
        if (sub(abs_s(fy_norm_fx[j]), fac_dwn_fx) < 0)
        {
            fy_norm_fx[j] = 0;
            move16();
        }
    }

    return;
}

/*-------------------------------------------------------------------*
  * gs_dec_amr_wb_fx()
  *
  * Modification of decoded excitation vector depending of the content type (used only in AMR-WB IO mode)
  *-------------------------------------------------------------------*/

static void gs_dec_amr_wb_fx(
    const long core_brate,        /* i  : bitrate allocated to the core       */
    Word16 *seed_tcx,        /* i/o: seed used for noise generation      */
    const Word16 dct_in_fx[],      /* i  : cdt of residual signal              */
    const Word16 Q_dct_in,         /* i  : Exponent of dct_in_fx               */
    Word16 dct_out_fx[],    /* o  : dct of pitch only excitation        */
    Word16 Q_dct_out,       /* o  : Exponent of dct_out_fx              */
    const Word16 pitch_fx[],      /* i  : pitch buffer                        */
    const Word16 voice_fac,       /* i  : gain pitch            Q15           */
    const Word16 clas,            /* i  : signal frame class                  */
    const Word16 coder_type       /* i  : coder type                          */
)
{
    Word16 i, mDiff_len;
    Word16 exc_diffQ_fx[L_FRAME16k];
    Word32 Ener_per_bd_iQ_fx[CRIT_NOIS_BAND];
    Word32 Ener_per_bd_yQ_fx[CRIT_NOIS_BAND];
    Word16 temp, exp, temp2;
    Word32 L_temp;

    /*--------------------------------------------------------------------------------------*
     * compute the energy per band for the decoded excitation (in frequency domain)
     *--------------------------------------------------------------------------------------*/

    Ener_per_band_fx( dct_in_fx, Q_dct_in, Ener_per_bd_iQ_fx );

    /*--------------------------------------------------------------------------------------*
     * adjust quantization noise for the low level to compensate for the poor 6 bit gainQ
     *--------------------------------------------------------------------------------------*/

    IF( L_sub(core_brate,ACELP_12k65) < 0)
    {
        temp = 0;
        move16();
        FOR(i = 0; i < CRIT_NOIS_BAND; i++)
        {
            temp  = s_max(round_fx(Ener_per_bd_iQ_fx[i]), temp);
        }

        test();
        test();
        IF((sub(coder_type,INACTIVE) == 0 || sub(clas,VOICED_TRANSITION) == 0) && sub(temp, 20) < 0 )
        {
            FOR(i = 0; i < CRIT_NOIS_BAND; i++)
            {
                Ener_per_bd_iQ_fx[i] = Mult_32_16(Ener_per_bd_iQ_fx[i], crit_bins_corr_fx[i]);
                move32();
            }
        }
    }

    /*--------------------------------------------------------------------------------------*
     * Find the lenght of the temporal contribution, with a minimum contribution of 1.2kHz
     *--------------------------------------------------------------------------------------*/
    temp = s_min(pitch_fx[0], pitch_fx[1]);
    temp = s_min(temp, pitch_fx[2]);
    temp = s_min(temp, pitch_fx[3]);

    /* etmp14 = 12800.0f/(temp/16.0f)*8.0f */
    exp = 6; /* Pitch in Q6*/ move16();
    temp = Invert16(temp, &exp);
    L_temp = L_mult(temp, 12800);
    L_temp = L_shl(L_temp, sub(3, exp)); /* *8.0f */

    if( L_sub(core_brate, ACELP_12k65) >= 0 )
    {
        L_temp = L_shl(L_temp, 1);
    }

    /* (Word16)(etmp14+0.5f) */
    mDiff_len = round_fx(L_temp);

    temp = 32767;
    move16();
    L_temp = L_deposit_l(0);
    FOR(i = 0; i < CRIT_NOIS_BAND; i++)
    {
        temp2 = sub(crit_bands_loc_fx[i], mDiff_len);
        temp2 = abs_s(temp2);
        if (sub(temp, temp2) > 0)
        {
            L_temp = L_msu(L_temp, crit_bins[i], -32768);
        }
        temp = s_min(temp, temp2);
    }

    mDiff_len = s_max(round_fx(L_temp), BIN_1k2);

    Copy(dct_in_fx, exc_diffQ_fx, mDiff_len);
    set16_fx(exc_diffQ_fx+mDiff_len, 0, sub(L_FRAME, mDiff_len));

    /*--------------------------------------------------------------------------------------*
     * normalization of the spectrum and noise fill
     *--------------------------------------------------------------------------------------*/

    normalize_spec_fx(4*256, 0*256, exc_diffQ_fx, mDiff_len, NORMALIZE_SPECS_Q_OUT); /* Factors in Q8 */

    NoiseFill_fx( exc_diffQ_fx, seed_tcx, CRIT_NOIS_BAND, NORMALIZE_SPECS_Q_OUT);

    /*--------------------------------------------------------------------------------------*
     * Recompute energy per band of the modified excitation vector (in frequency domain)
     *--------------------------------------------------------------------------------------*/

    Ener_per_band_fx( exc_diffQ_fx, NORMALIZE_SPECS_Q_OUT, Ener_per_bd_yQ_fx );

    /*--------------------------------------------------------------------------------------*
     * Compute tilt factor and amplify HF accordingly
     *--------------------------------------------------------------------------------------*/

    temp = mult_r(sub(32767, voice_fac), 16384); /*  Q15 */
    FOR(i = 240; i < L_FRAME; i++)
    {
        temp2 = msu_r(-7680<<16, -17564, shl(i,6)) ;   /*-15 in Q9; -0.067 in Q18 and i in Q6= Q9 */
        temp2 = mult_r(temp2, temp); /* Q15*Q9+1-16 -> Q9 */
        L_temp = L_mult(exc_diffQ_fx[i],s_max(temp2,512)); /*Q(Qexc_diffQ+10) */
        exc_diffQ_fx[i] = round_fx(L_shl(L_temp,16-10));/*Qexc_diffQ */
    }

    /*--------------------------------------------------------------------------------------*
     * Match the energy of the modified excitation vector to the decoded excitation
     *--------------------------------------------------------------------------------------*/

    Apply_gain_fx( exc_diffQ_fx, Ener_per_bd_iQ_fx, Ener_per_bd_yQ_fx );

    /*--------------------------------------------------------------------------------------*
     * Copy to the output vector
     *--------------------------------------------------------------------------------------*/

    Copy_Scale_sig(exc_diffQ_fx, dct_out_fx, L_FRAME, sub(Q_dct_out, NORMALIZE_SPECS_Q_OUT));

    return;
}

/*-------------------------------------------------------------------*
 * improv_amr_wb_gs_fx()
 *
 * Modify the decoded excitation to increase quality of
 * unvoiced and audio signals (used only in AMR-WB IO mode)
 *-------------------------------------------------------------------*/
void improv_amr_wb_gs_fx(
    const Word16 clas,                             /* i  : signal frame class                  */
    const Word16 coder_type,                       /* i  : coder type                          */
    const Word32 core_brate,                       /* i  : bitrate allocated to the core       */
    Word16 *seed_tcx,                        /* i/o: Seed used for noise generation      */
    Word16 *old_Aq_fx,                       /* i/o: old LPC filter coefficient          */
    Word16 *mem_syn2_fx,                     /* i/o: synthesis memory                    */
    const Word16 lt_voice_fac_fx,                  /* i/o: long term voice factor         Q14  */
    const Word16 locattack,                        /* i  : Flag for a detected attack          */
    Word16 *Aq_fx,                           /* i/o: Decoded LP filter coefficient       */
    Word16 *exc2_fx,                         /* i/o: Decoded complete excitation         */
    const Word16 Q_exc2,                           /* i  : Exponent of Exc2                    */
    Word16 *mem_tmp_fx,                      /* i/o: synthesis temporary memory          */
    Word16 *syn_fx,                          /*   o: Decoded synthesis to be updated     */
    const Word16 Q_syn,                            /* i  : Synthesis scaling             Q0    */
    const Word16 *pitch_buf_fx,                    /* i  : Decoded pitch buffer          Q6    */
    const Word16 Last_ener_fx                      /* i  : Last energy (Q8) */
    ,const Word16 last_coder_type_fx                /* i  : Last coder_type */
)
{
    Word16 i, exp_a, exp_b, exp_diff, j;
    Word16 dct_exc_in_fx[L_FRAME], dct_exc_out_fx[L_FRAME];

    /*------------------------------------------------------------*
     * Condition to enter the section on excitation modification
     *------------------------------------------------------------*/

    /* Enter the modification for all inactive frames and also for unvoiced frames if bit rate is below 8k85 */

    test();
    test();
    test();
    test();
    test();
    test();
    IF( ( locattack == 0 && L_sub(core_brate, ACELP_12k65) <= 0) &&
        ( (L_sub(core_brate, ACELP_8k85) < 0 && sub(clas, AUDIO_CLAS) != 0 &&
           (sub(clas, UNVOICED_CLAS) == 0 || sub(clas, VOICED_TRANSITION) == 0)) || sub(coder_type, INACTIVE) == 0 ) )
    {
        /*------------------------------------------------------------*
         * two differents paths:
         *   unvoiced or inactive
         *   generic audio sound
         * LP filter smoothing for inactive parts
         *------------------------------------------------------------*/
        /* last_coder_type_fx == UNVOICED should be understand as INACTIVE, but it is forced to UNVOICED in update_dec */
        test();
        test();
        IF( sub(coder_type, INACTIVE) == 0 && sub(Last_ener_fx, -3*256) > 0 && last_coder_type_fx == UNVOICED) /* 3.0 x 256 to Go to Q8 */
        {

            FOR(i =0; i < NB_SUBFR; i++)
            {

                exp_a = norm_s(Aq_fx[i*(M+1)]);
                exp_b = norm_s(old_Aq_fx[i*(M+1)]);
                exp_diff = sub(exp_a, exp_b);
                IF(exp_diff>0)
                {
                    Scale_sig(&old_Aq_fx[i*(M+1)],  (M+1), negate(exp_diff));
                }
                ELSE
                {
                    Scale_sig(&Aq_fx[i*(M+1)],  (M+1), exp_diff);

                }
                FOR(j = i*(M+1); j < (i+1)*(M+1); j++)
                {

                    Aq_fx[j] = round_fx(L_mac(L_mult(ALP_FX, old_Aq_fx[j]), MALP_FX, Aq_fx[j]));
                }
            }
        }

        /*------------------------------------------------------------*
         * Find frequency representation of the excitation
         * Do the excitation modification according to the content
         * Go back to time domain -> Overwrite exctiation
         *------------------------------------------------------------*/

        edct_16fx(exc2_fx, dct_exc_in_fx, L_FRAME, 4);

        gs_dec_amr_wb_fx( core_brate, seed_tcx, dct_exc_in_fx, Q_exc2, dct_exc_out_fx, Q_exc2, pitch_buf_fx, lt_voice_fac_fx, clas, coder_type );

        edct_16fx(dct_exc_out_fx, exc2_fx, L_FRAME, 4);

        /*------------------------------------------------------------*
         * Redo core synthesis at 12k8 Hz with the modified excitation
         *------------------------------------------------------------*/

        Copy( mem_tmp_fx, mem_syn2_fx, M );
        syn_12k8_fx(L_FRAME, Aq_fx, exc2_fx, syn_fx, mem_syn2_fx, 1, Q_exc2, Q_syn );
    }

    Copy( Aq_fx, old_Aq_fx, NB_SUBFR * (M+1) );

    return;
}
