/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "cnst_fx.h"            /* Audio core constants    */
#include "rom_com_fx.h"         /* Static table prototypes */
#include "prot_fx.h"
#include "stl.h"
#include "basop_mpy.h"

/*--------------------------------------------------------------------------*
 * mdct_spectrum_denorm()
 *
 *
 *--------------------------------------------------------------------------*/

void mdct_spectrum_denorm_fx(
    const Word16 inp_vector[],       /* i   : Q0  :                                       */
    Word32 L_y2[],             /* i/o : Qs  : decoded spectrum                      */
    const Word16 band_start[],       /* i   : Q0  : table of start freq for every subband */
    const Word16 band_end[],         /* i   : Q0  : table of end freq for every subband   */
    const Word16 band_width[],       /* i   : Q0  : table of bandwidth for every subband  */
    const Word32 L_band_energy[],    /* i   : Qbe : band energy                           */
    const Word16 npulses[],          /* i   : Q0  : number of coded spectrum              */
    const Word16 bands,              /* i   : Q0  : number of subbands                    */
    const Word16 ld_slope_fx,        /* i   : Q15 :                                       */
    const Word16 pd_thresh_fx        /* i   : Q15 :                                       */
)
{
    Word16 i, k;
    Word32 L_Eyy;
    Word32 L_tmp, L_temp;
    Word16 temp_fx, temp_lo_fx, temp_hi_fx;
    Word32 L_inp_tmp[L_FRAME48k];
    Word16 exp_norm;
    Word16 exp_safe;
    Word16 exp_normn, exp_normd;

    Word16 pd_fx;
    Word16 Qpd;

    Word16 div_pd_fx;
    Word16 Qdivpd;
    Word32 L_div_pd;

    Word16 frac, exp;

    Word16 gain_tweak_fx;
    Word16 Qtweak;

    Word16 exp_shift;

    Word16 QEyy;
    Word16 pow_fx;
    Word16 Qpow;
    Word16 Qdiv;
    Word16 Qgamma;
    Word16 gamma_fx;

    Word16 cond_fx;

    exp_safe = 4; /* safe bit for overflow */

    FOR (k = 0; k < bands; k++)
    {
        L_tmp = L_deposit_l(0);
        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            L_inp_tmp[i] = L_mult(inp_vector[i], inp_vector[i]);
            move32();  /* Q0+Q0+1 */
            L_tmp = L_or(L_tmp, L_inp_tmp[i]);
        }
        exp_norm = norm_l(L_tmp);
        exp_norm = sub(exp_norm, exp_safe);

        L_Eyy = L_deposit_l(0);
        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            /*Eyy += (float) inp_vector[i] * inp_vector[i]; */
            L_Eyy = L_add(L_Eyy, L_shl(L_inp_tmp[i], exp_norm));  /* Q1+exp_norm */
        }
        QEyy = add(1, exp_norm);

        IF ( L_Eyy > 0x0L )
        {
            /* Set gamma to be pulse gain which results in perfect quantized subband energy */
            /*gamma = (float) sqrt (pow (2.0f, band_energy[k]) / Eyy); */

            /* Pow part  (pow(2.0f, band_energy) ) */
            L_temp = L_shr(L_band_energy[k], sub(SWB_BWE_LR_Qbe, 16));
            temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
            Qpow = sub(14, temp_hi_fx);
            pow_fx = extract_l(Pow2(14, temp_lo_fx));        /* Qpow */

            /* Div part  ( pow (2.0f, band_energy[i])/Eyy ) */
            exp_normn = norm_s(pow_fx);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_l(L_Eyy);
            temp_fx = div_s( shl( pow_fx, exp_normn), extract_h(L_shl(L_Eyy, exp_normd)));
            Qdiv = add(sub(add(Qpow, exp_normn) , add(QEyy, exp_normd)), 31);

            exp_norm = norm_s(temp_fx);
            temp_fx = shl(temp_fx, exp_norm);
            Qdiv = add(Qdiv, exp_norm);

            /* Sqrt part sqrt(pow (2.0f, band_energy[i])/Eyy) */
            Qgamma = add(Qdiv, 16);
            IF ( s_and(Qdiv, 1) == 0 ) /* Qdiv % 2 == 0 */
            {
                L_temp = Sqrt_l(L_shr(L_deposit_h(temp_fx),1), &exp_norm);
                L_temp = L_shr(L_temp, exp_norm);
                Qgamma = sub(shr(Qgamma, 1), 1);
                gamma_fx = round_fx(L_temp);
            }
            ELSE
            {
                L_temp = Sqrt_l(L_deposit_h(temp_fx), &exp_norm);
                L_temp = L_shr(L_temp, exp_norm);
                Qgamma = shr(Qgamma, 1);
                gamma_fx = round_fx(L_temp);
            }

            /* Adjust gamma based on pulse density (0 bit MSE gain estimator) */
            /*pd = (float) npulses[k] / band_width[k]; */
            exp_normn = norm_s(npulses[k]);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_s(band_width[k]);
            pd_fx = div_s(shl(npulses[k],exp_normn), shl(band_width[k], exp_normd));
            Qpd = add(sub(exp_normn, exp_normd), 15);

            cond_fx = sub(shl(pd_fx, sub(15, Qpd)), pd_thresh_fx/*Q15*/);
            Overflow = 0;
            move16(); /* allow overflow happen. */
            IF ( cond_fx < 0 )
            {
                /*gain_tweak = (float) pow (2.0f, (ld_slope * log2_f (pd / pd_thresh))); */
                /* Div part */
                exp_normn = norm_s(pd_fx);
                exp_normn = sub(exp_normn, 1);
                exp_normd = norm_s(pd_thresh_fx);
                div_pd_fx = div_s(shl(pd_fx, exp_normn), shl(pd_thresh_fx, exp_normd)); /* Qpd+exp_normn - (15 + exp_normd) + 15 */
                Qdivpd = add(sub(add(Qpd, exp_normn), add(15, exp_normd)), 15);

                /* Log2 part */
                exp_norm = norm_s(div_pd_fx);
                L_div_pd = L_deposit_h(shl(div_pd_fx, exp_norm));
                Qdivpd = add(add(Qdivpd, exp_norm), 16);

                frac = Log2_norm_lc(L_div_pd);
                exp = sub(30, Qdivpd);
                L_tmp = L_Comp(exp, frac); /* Q16 */

                /* Mult part */
                L_tmp = Mpy_32_16_1(L_tmp, ld_slope_fx);

                /* Pow part */
                temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);
                Qtweak = sub(14, temp_hi_fx);
                gain_tweak_fx = extract_l(Pow2(14, temp_lo_fx));

                /*gamma *= gain_tweak; */
                L_tmp = L_mult(gamma_fx, gain_tweak_fx);  /* Qgamma+Qtweak+1 */
                exp_norm = norm_l(L_tmp);
                gamma_fx = round_fx(L_shl(L_tmp, exp_norm));
                Qgamma = sub(add(add(Qgamma, Qtweak), exp_norm), 15);/*Qgamma+Qtweak+1+exp_norm-16; */
            }

            exp_shift = sub(SWB_BWE_LR_Qs-1, Qgamma);
            FOR (i = band_start[k]; i <= band_end[k]; i++)
            {
                /*y2[i] = gamma * inp_vector[i]; */
                L_tmp = L_mult(gamma_fx, (Word16)inp_vector[i]); /* Qgamma+0+1=Qgamma+1 */
                L_y2[i] = L_shl(L_tmp, exp_shift);
                move32();
            }
        }
    }

    return;
}
/*==========================================================================*/
/* FUNCTION      : void hq2_core_configure_fx()                             */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :                                                          */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*    Word16 *qint                o:                                Q13     */
/*    Word16 *eref                o:                                Q10     */
/*    Word16 *bit_alloc_weight    o:                                Q13     */
/*    Word16 *p2a_th              o:                                Q11     */
/*    Word16 *pd_thresh           o:                                Q15     */
/*    Word16 *ld_slope            o:                                Q15     */
/*    Word16 *ni_coef             o:                                Q14     */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*                     _ None                                               */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                                                            */
/*==========================================================================*/

void hq2_core_configure_fx (
    const Word16 frame_length,
    const Word16 num_bits,
    const Word16 is_transient,
    Word16 *bands,
    Word16 *length,
    Word16 band_width[],
    Word16 band_start[],
    Word16 band_end[],
    Word32 *L_qint,
    Word16 *eref,
    Word16 *bit_alloc_weight,
    Word16 *gqlevs,
    Word16 *Ngq,
    Word16 *p2a_bands,
    Word16 *p2a_th,
    Word16 *pd_thresh,
    Word16 *ld_slope,
    Word16 *ni_coef,
    Word32 L_bwe_br
)
{
    const Xcore_Config_fx *xcore_config_fx;

    Word16 i, k;
    Word16 bands_sh;

    xcore_config_fx = &xcore_config_32kHz_013200bps_long_fx; /* default set for VC Warning */

    IF ( sub(frame_length, L_FRAME8k) == 0 )
    {
        IF( is_transient )
        {
            IF ( sub(num_bits, ACELP_7k20 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_007200bps_short_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_8k00 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_008000bps_short_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_9k60 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_009600bps_short_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_13k20 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_013200bps_short_fx;
            }
            ELSE
            {
                xcore_config_fx = &xcore_config_8kHz_016400bps_short_fx;
            }
        }
        ELSE
        {
            IF ( sub(num_bits, ACELP_7k20 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_007200bps_long_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_8k00 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_008000bps_long_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_9k60 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_8kHz_009600bps_long_fx;
            }
            ELSE IF ( sub(num_bits, ACELP_13k20 / 50) <= 0)
            {
                xcore_config_fx = &xcore_config_8kHz_013200bps_long_fx;
            }
            ELSE
            {
                xcore_config_fx = &xcore_config_8kHz_016400bps_long_fx;
            }
        }
    }
    ELSE IF ( sub(frame_length, L_FRAME16k)  == 0 )
    {
        IF (is_transient)
        {
            IF ( sub(num_bits, ACELP_13k20 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_16kHz_013200bps_short_fx;
                move16();
            }
            ELSE if ( sub(num_bits, ACELP_16k40 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_16kHz_016400bps_short_fx;
                move16();
            }
        }
        ELSE
        {
            IF ( sub(num_bits, ACELP_13k20 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_16kHz_013200bps_long_fx;
                move16();
            }
            ELSE if ( sub(num_bits, ACELP_16k40 / 50) <= 0 )
            {
                xcore_config_fx = &xcore_config_16kHz_016400bps_long_fx;
                move16();
            }
        }
    }
    ELSE    /* (frame_length == SWB) */
    {
        IF (is_transient)
        {
            IF ( L_sub(L_bwe_br, ACELP_13k20) <= 0 )
            {
                xcore_config_fx = &xcore_config_32kHz_013200bps_short_fx;
                move16();
            }
            ELSE if ( L_sub(L_bwe_br, ACELP_16k40) <= 0 )
            {
                xcore_config_fx = &xcore_config_32kHz_016400bps_short_fx;
                move16();
            }
        }
        ELSE
        {
            IF ( L_sub(L_bwe_br, ACELP_13k20) <= 0 )
            {
                xcore_config_fx = &xcore_config_32kHz_013200bps_long_fx;
                move16();
            }
            ELSE if ( L_sub(L_bwe_br, ACELP_16k40) <= 0 )
            {
                xcore_config_fx = &xcore_config_32kHz_016400bps_long_fx;
                move16();
            }
        }
    }

    *bands = xcore_config_fx->bands;
    move16();
    *length = xcore_config_fx->bw;
    move16();
    *L_qint = xcore_config_fx->L_qint;
    move32();

    *eref = xcore_config_fx->eref;
    move16();
    *bit_alloc_weight = xcore_config_fx->bit_alloc_weight;
    move16();
    *gqlevs = xcore_config_fx->gqlevs;
    move16();
    *Ngq = xcore_config_fx->Ngq;
    move16();

    *p2a_bands = xcore_config_fx->p2a_bands;
    move16();
    *p2a_th = xcore_config_fx->p2a_th;
    move16();

    *pd_thresh = xcore_config_fx->pd_thresh;
    move16();
    *ld_slope = xcore_config_fx->ld_slope;
    move16();
    *ni_coef = xcore_config_fx->ni_coef;
    move16();

    /*mvs2s_fx (xcore_config_fx->band_width, band_width, *bands); */
    Copy(xcore_config_fx->band_width, band_width, *bands);

    /* Expand band_width[] table for short windows */
    IF (is_transient)
    {
        bands_sh = *bands;
        move16();
        *bands = shl(bands_sh,2);
        *length = shl(*length, 2);

        FOR (i = 1; i <= 3; i++)
        {
            FOR (k = 0; k < bands_sh; k++)
            {
                band_width[i * bands_sh + k] = band_width[k];
                move16();
            }
        }
    }

    /* Formulate band_start and band_end tables from band_width table */
    band_start[0] = 0;
    move16();
    band_end[0] = sub(band_width[0], 1);
    move16();
    FOR (k = 1; k < *bands; k++)
    {
        band_start[k] = add( band_start[k - 1] , band_width[k - 1]);
        move16();
        band_end[k] = sub(add( band_start[k] , band_width[k]) , 1);
        move16();
    }


    return;
}

/*--------------------------------------------------------------------------*
 * reverse_transient_frame_energies()
 *
 *
 *--------------------------------------------------------------------------*/

void reverse_transient_frame_energies_fx(
    Word32 L_band_energy[],       /* o  : Q14 : band energies       */
    const Word16 bands                  /* i  : Q0  : number of bands     */
)
{
    Word16 k, k1, k2;
    Word32 L_be;
    Word16 bands_2, bands_4, bands_8;
    Word32 *p_be1, *p_be2;

    bands_2 = shr(bands, 1);
    bands_4 = shr(bands, 2);
    bands_8 = shr(bands, 3);

    k1 = bands_4;
    k2 = sub(bands_2, 1);
    p_be1 = &L_band_energy[k1];
    p_be2 = &L_band_energy[k2];
    FOR( k = 0; k < bands_8; k++ )
    {
        L_be = *p_be1;
        move32();
        *p_be1 = *p_be2;
        move32();
        *p_be2 = L_be;
        move32();
        p_be1++;
        p_be2--;
    }

    k1 = sub(bands, bands_4); /* 3*bands/4 */
    k2 = sub(bands, 1);
    p_be1 = &L_band_energy[k1];
    p_be2 = &L_band_energy[k2];
    FOR( k = 0; k < bands_8; k++ )
    {
        L_be = *p_be1;
        move32();
        *p_be1 = *p_be2;
        move32();
        *p_be2 = L_be;
        move32();
        p_be1++;
        p_be2--;
    }

    return;
}


/*--------------------------------------------------------------------------*
 * spt_shorten_domain_pre()
 *
 * Compute shorten subband if previous frame has spectral peak.
 *--------------------------------------------------------------------------*/

void spt_shorten_domain_pre_fx(
    const Word16 band_start[],           /* i:   Starting position of sub band             */
    const Word16 band_end[],             /* i:   End position of sub band                  */
    const Word16 prev_SWB_peak_pos[],    /* i:   Spectral peak                             */
    const Word16 BANDS,                  /* i:   total number of bands                     */
    const Word32 L_bwe_br,               /* i:   bitrate information                       */
    Word16 new_band_start[],       /* o:   Starting position of new shorten sub band */
    Word16 new_band_end[],         /* o:   End position of new shorten sub band      */
    Word16 new_band_width[]        /* o:   new sub band bandwidth                    */
)
{
    Word16 j;
    Word16 k;
    Word16 kpos;

    Word16 new_band_width_half;
    const Word16 *p_bw_SPT_tbl; /* pointer of bw_SPT_tbl */

    p_bw_SPT_tbl = bw_SPT_tbl[0];
    if( L_sub(L_bwe_br, HQ_16k40) == 0 )
    {
        p_bw_SPT_tbl = bw_SPT_tbl[1];
    }

    kpos = sub(NI_USE_PREV_SPT_SBNUM, SPT_SHORTEN_SBNUM);
    j = 0;
    move16();
    FOR(k=sub(BANDS,SPT_SHORTEN_SBNUM); k<BANDS; k++)
    {
        IF ( prev_SWB_peak_pos[kpos] != 0)
        {
            new_band_width[j] = p_bw_SPT_tbl[j];

            /*shorten the bandwidth for pulse resolution*/
            new_band_width_half = shr(new_band_width[j], 1);
            move16();
            new_band_start[j] = sub(prev_SWB_peak_pos[kpos], new_band_width_half);
            move16();
            new_band_end[j]   = add(prev_SWB_peak_pos[kpos], new_band_width_half);
            move16();

            IF( sub(new_band_start[j], band_start[k]) < 0 )
            {
                new_band_start[j] = band_start[k];
                move16();
                new_band_end[j] = add(new_band_start[j], sub(new_band_width[j],1));
                move16();
            }
            ELSE IF ( sub(new_band_end[j], band_end[k]) > 0 )
            {
                new_band_end[j]   = band_end[k];
                move16();
                new_band_start[j] = sub(new_band_end[j], sub(new_band_width[j],1));
                move16();
            }
        }

        kpos = add(kpos, 1);
        j = add(j, 1);
    }

    return;
}

/*--------------------------------------------------------------------------*
 * spt_shorten_domain_band_save()
 *
 * Store the original subband information
 *--------------------------------------------------------------------------*/

void spt_shorten_domain_band_save_fx(
    const Word16 bands,                  /* i:   total subband                */
    const Word16 band_start[],           /* i:   starting position of subband */
    const Word16 band_end[],             /* i:   end position of subband      */
    const Word16 band_width[],           /* i:   band width of subband        */
    Word16 org_band_start[],       /* o:   starting position of subband */
    Word16 org_band_end[],         /* o:   end position of subband      */
    Word16 org_band_width[]        /* o:   band width of subband        */
)
{
    Word16 k;
    Word16 kpos;

    kpos = 0;
    move16();
    FOR(k=sub(bands,SPT_SHORTEN_SBNUM); k<bands; k++)
    {
        org_band_start[kpos] = band_start[k];
        move16();
        org_band_end[kpos]   = band_end[k];
        move16();
        org_band_width[kpos] = band_width[k];
        move16();
        kpos = add(kpos, 1);
    }

    return;
}

/*--------------------------------------------------------------------------*
 * spt_shorten_domain_band_restore()
 *
 * Restrore the subband information
 *--------------------------------------------------------------------------*/

void spt_shorten_domain_band_restore_fx(
    const Word16 bands,                  /* i:   total subband                */
    Word16 band_start[],           /* i/o: starting position of subband */
    Word16 band_end[],             /* i/o: end position of subband      */
    Word16 band_width[],           /* i/o: band width of subband        */
    const Word16 org_band_start[],       /* o:   starting position of subband */
    const Word16 org_band_end[],         /* o:   end position of subband      */
    const Word16 org_band_width[]        /* o:   band width of subband        */
)
{
    Word16 k;
    Word16 kpos;

    kpos = 0;
    move16();
    FOR(k=sub(bands,SPT_SHORTEN_SBNUM); k<bands; k++)
    {
        band_start[k] = org_band_start[kpos];
        move16();
        band_end[k]   = org_band_end[kpos];
        move16();
        band_width[k] = org_band_width[kpos];
        move16();
        kpos = add(kpos, 1);
    }

    return;
}

/*--------------------------------------------------------------------------*
 * spt_swb_peakpos_tmp_save
 *
 * Save Peak position for every higher subband
 *--------------------------------------------------------------------------*/

void spt_swb_peakpos_tmp_save_fx(
    const Word32 L_y2[],                 /* i:   coded spectral information   */
    const Word16 bands,                  /* i:   total number of bands        */
    const Word16 band_start[],           /* i:   starting position of subband */
    const Word16 band_end[],             /* i:   end position of subband      */
    Word16 prev_SWB_peak_pos_tmp[] /* o:   spectral peaks               */
)
{

    Word16 i, j, k;
    Word32 L_peak_max;
    Word32 L_abs_y2;

    j = 0;
    move16();
    FOR(k=sub(bands, NI_USE_PREV_SPT_SBNUM); k<bands; k++)
    {
        L_peak_max = L_deposit_l(0);
        prev_SWB_peak_pos_tmp[j] = 0;
        move16();
        FOR(i=band_start[k]; i<=band_end[k]; i++)
        {
            L_abs_y2 = L_abs(L_y2[i]);
            move32();
            IF( L_sub( L_peak_max, L_abs_y2) < 0x0L )
            {
                L_peak_max = L_abs_y2;
                move32();
                prev_SWB_peak_pos_tmp[j] = i;
                move16();
            }
        }
        j = add(j, 1);
    }
    return;
}

void bit_allocation_second_fx(
    Word32 *Rk,
    Word32 *Rk_sort,
    Word16  BANDS,
    const Word16 *band_width,
    Word16 *k_sort,
    Word16 *k_num,
    const Word16 *p2a_flags,
    const Word16  p2a_bands,
    const Word16 *last_bitalloc,
    const Word16  input_frame
)
{
    Word16 k, k2 = 0;
    Word16 ever_bits[BANDS_MAX], ever_sort[BANDS_MAX];/*Q12 */
    Word16 class_flag = 0;
    Word16 rk_temp = 32767, ever_temp = 32767;/*Q12 */
    Word16 exp;
    Word16 tmp;
    Word32 L_tmp;

    FOR (k = 0; k < BANDS; k++)
    {
        test();
        test();
        test();
        IF((( sub(k_sort[k],sub(BANDS,p2a_bands)) >= 0 )&&( sub(p2a_flags[k_sort[k]],1) == 0 )) ||
           (( sub(k_sort[k],sub(BANDS,2)) >= 0 )&&( sub(last_bitalloc[k_sort[k]],1) == 0 )))
        {
            exp = norm_s(band_width[k_sort[k]]);
            tmp = shl(band_width[k_sort[k]],exp);/*Q(exp) */
            tmp = div_s(16384,tmp);/*Q(15+14-exp = 29-exp) */
            L_tmp = Mult_32_16(Rk_sort[k],tmp);/* Q(16+29-exp-15 = 30-exp) */
            tmp = sub(18,exp);
            ever_bits[k] = extract_l(L_shr(L_tmp,tmp));/*Q12 */
            IF( sub(ever_bits[k],rk_temp) < 0 )
            {
                rk_temp = ever_bits[k];
                move16();
                k2 = k;
                move16();
            }
            class_flag = 1;
        }
    }
    test();
    IF( class_flag ==0 || sub(input_frame,L_FRAME8k) == 0)
    {
        FOR(k = 0; k < BANDS; k++)
        {
            test();
            IF( sub(k_sort[k],sub(BANDS,p2a_bands)) < 0 && Rk_sort[k] > 0 )
            {
                exp = norm_s(band_width[k_sort[k]]);
                tmp = shl(band_width[k_sort[k]],exp);/*Q(exp) */
                tmp = div_s(16384,tmp);/*Q(15+14-exp = 29-exp) */
                L_tmp = Mult_32_16(Rk_sort[k],tmp);/* Q(16+29-exp-15 = 30-exp) */
                tmp = sub(18,exp);
                ever_sort[k] = extract_l(L_shr(L_tmp,tmp));/*Q12 */
                IF(sub(ever_sort[k],ever_temp) < 0)
                {
                    ever_temp = ever_sort[k];
                    move16();
                    k2 = k;
                    move16();
                }
            }
        }
    }

    k_num[0] = k2;
    IF(sub(k_sort[k2],sub(BANDS,1)) == 0)
    {
        FOR (k = 0; k < BANDS; k++)
        {
            if(sub(k_sort[k],sub(k_sort[k2],1)) == 0)
            {
                k_num[1] = k;
                move16();
            }
        }
    }
    ELSE IF(k_sort[k2] == 0)
    {
        FOR (k = 0; k < BANDS; k++)
        {
            if(sub(k_sort[k],add(k_sort[k2],1)) == 0)
            {
                k_num[1] = k;
                move16();
            }
        }
    }
    ELSE
    {
        IF ( L_sub( Rk[sub(k_sort[k2],1)],Rk[add(k_sort[k2],1)] ) < 0 )
        {
            FOR (k = 0; k < BANDS; k++)
            {
                if(sub(k_sort[k],sub(k_sort[k2],1)) == 0)
                {
                    k_num[1] = k;
                    move16();
                }
            }
        }
        ELSE
        {
            FOR (k = 0; k < BANDS; k++)
            {
                if(sub(k_sort[k],add(k_sort[k2],1)) == 0)
                {
                    k_num[1] = k;
                    move16();
                }
            }
        }
    }
}
