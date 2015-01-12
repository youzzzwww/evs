/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"
#include "stl.h"
#include "basop_mpy.h"

/*--------------------------------------------------------------------------*
 * Local functions
 *--------------------------------------------------------------------------*/

static  Word16 band_energy_quant_fx( Encoder_State_fx *st_fx, const Word32 *L_t_audio, const Word16 band_start_fx[], const Word16 band_end_fx[], Word32 L_band_energy[],
                                     const Word16 bands_fx, const Word32 L_qint, const Word16 eref_fx, const Word16 is_transient_fx );

static  Word16 p2a_threshold_quant_fx( Encoder_State_fx *st_fx, const Word32 *L_t_audio, const Word16 band_start[], const Word16 band_end[], const Word16 band_width[],
                                       const Word16 bands, const Word16 p2a_bands, const Word16 p2a_th, Word16 *p2a_flags );

static  void mdct_spectrum_fine_gain_enc_fx( Encoder_State_fx *st_fx, const Word32 L_ybuf[], Word32 L_y2[], const Word16 band_start[], const Word16 band_end[],
        const Word16 k_sort[], const Word16 bands,
        const Word32 L_qint, const Word16 Ngq, const Word16 gqlevs, const Word16 gqbits );

/*--------------------------------------------------------------------------*
 * spt_shorten_domain_set()
 *
 * Track the spectral peak based on peak -avg analysis
 *--------------------------------------------------------------------------*/

static  void spt_shorten_domain_set_fx(
    Encoder_State_fx *st_fx,              /* i:   encoder state structure             */
    const Word32 L_t_audio[],         /* i:   input spectrum                      */
    const Word16 p2a_flags[],         /* i:   p2a anlysis information             */
    const Word16 new_band_start[],    /* i:   new band start position             */
    const Word16 new_band_end[],      /* i:   new band end position               */
    const Word16 new_band_width[],    /* i:   new subband band width              */
    const Word16 bands,               /* i:   total number of subbands            */
    Word16 band_start[],        /* i/o: band start position                 */
    Word16 band_end[],          /* i/o: band end position                   */
    Word16 band_width[],        /* i:   sub band band width                 */
    Word16 *bit_budget          /* i/o: bit budget                          */
)
{
    Word16 i, j, k;
    Word16 kpos;
    Word32 L_max_y2;
    Word16 max_y2_pos;
    Word16 spt_shorten_flag[SPT_SHORTEN_SBNUM];

    kpos = sub(NI_USE_PREV_SPT_SBNUM, SPT_SHORTEN_SBNUM);
    j = 0;
    move16();
    FOR(k=sub(bands,SPT_SHORTEN_SBNUM); k<bands; k++)
    {
        IF( sub(p2a_flags[k], 1) == 0 )
        {
            spt_shorten_flag[j] = 0;
            move16();
            IF ( st_fx->prev_SWB_peak_pos_fx[kpos] != 0)
            {
                L_max_y2 = L_deposit_l(0);
                max_y2_pos = 0;
                move16();
                FOR(i=band_start[k]; i<=band_end[k]; i++)
                {
                    IF( L_sub( L_max_y2, L_abs(L_t_audio[i])) < 0 )
                    {
                        L_max_y2 = L_abs(L_t_audio[i]);
                        max_y2_pos = i;
                        move16();
                    }
                }
                test();
                IF( sub(max_y2_pos, new_band_start[j]) >= 0 && sub(max_y2_pos, new_band_end[j]) <= 0 )
                {
                    band_start[k] = new_band_start[j];
                    move16();
                    band_end[k]   = new_band_end[j];
                    move16();
                    band_width[k] = new_band_width[j];
                    move16();
                    spt_shorten_flag[j] = 1;
                    move16();
                }
            }
            push_indice_fx(st_fx, IND_HQ2_SPT_SHORTEN, spt_shorten_flag[j], 1);
            *bit_budget = sub(*bit_budget, 1);
        }

        kpos = add(kpos, 1);
        j = add(j, 1);
    }

    return;
}

/*--------------------------------------------------------------------------*
 * hq_lr_enc_fx()
 *
 * HQ Low rate encoding routine
 *--------------------------------------------------------------------------*/

void hq_lr_enc_fx(
    Encoder_State_fx *st_fx,           /* i/o:     : encoder state structure    */
    Word32 L_t_audio[],      /* i/o: Q12 : transform-domain coefs.    */
    const Word16 inner_frame_fx,   /* i  : Q0  : inner frame length         */
    Word16 *num_bits_fx,     /* i/o: Q0  : number of available bits   */
    const Word16 is_transient_fx   /* i  : Q0  : transient flag             */
)
{
    Word16 i, k1_fx, k2_fx;
    Word16 bit_budget_fx, pbits_fx;
    Word16 bands_fx, length_fx, ni_seed_fx, gqlevs_fx, gqbits_fx, Ngq_fx, p2a_bands_fx;
    Word16 p2a_flags_fx[BANDS_MAX];
    Word16 band_start[BANDS_MAX], band_end[BANDS_MAX], band_width[BANDS_MAX];
    Word32 L_band_energy[BANDS_MAX], L_Rk[BANDS_MAX];
    Word16 ebits_fx;

    Word32 L_qint;
    /*Word16 Qqint=29;*/
    Word16 eref_fx/*, Qeref=10*/;
    Word16 bit_alloc_weight_fx/*, Qbaw=13*/;
    Word16 ld_slope_fx/*, Qldslope=15*/;
    Word16 p2a_th_fx/*, Qp2ath=11*/;
    Word16 pd_thresh_fx/*, Qpdth=15*/;
    Word16 ni_coef_fx/*, Qnicoef=14*/;

    Word16 k_sort_fx[BANDS_MAX];
    Word16 npulses_fx[BANDS_MAX];
    Word16 inp_vector_fx[L_FRAME48k];
    Word32 L_y2[L_FRAME48k];
    Word32 L_y2_ni[L_FRAME48k];
    Word16 hqswb_clas_fx;
    Word16 lowlength_fx;
    Word16 highlength_fx;
    Word32 L_m[L_FRAME32k];
    Word16 har_bands_fx;
    Word16 bw_low, bw_high;
    Word32 L_band_energy_tmp[BANDS_MAX];
    Word32 L_bwe_br;
    Word16 trans_bit;
    Word16 adjustFlag;
    Word16 prev_SWB_peak_pos_tmp_fx[NI_USE_PREV_SPT_SBNUM];
    Word16  k,j;
    Word16 flag_spt_fx;
    Word16 org_band_start[SPT_SHORTEN_SBNUM];
    Word16 org_band_end[SPT_SHORTEN_SBNUM];
    Word16 org_band_width[SPT_SHORTEN_SBNUM];
    Word16 new_band_start[SPT_SHORTEN_SBNUM];
    Word16 new_band_end[SPT_SHORTEN_SBNUM];
    Word16 new_band_width[SPT_SHORTEN_SBNUM];

    Word16 k1_step_fx, k2_step_fx;
    Word16 exp_norm;

    Word16 length1_fx, length2_fx, length3_fx;
    Word16 lowband,highband,p2a_flags_tmp[BANDS_MAX];
    Word32 L_tmp,L_tmp2,L_tmp3;
    Word16 exp,exp2,tmp,tmp1,tmp2,tmp3,frac1,alpha_fx,Q_band_energy;
    Word32 enerH_fx;
    Word32 enerL_fx;
    Word32 Ep_fx[BANDS_MAX];
    Word32 Ep_avrg_fx, Ep_vari_fx;
    Word32 Ep_avrgL_fx;
    Word32 Ep_peak_fx;
    Word32 Ep_tmp_fx[BANDS_MAX];
    Word16 gama_fx;/*Q15 0.85f;// */
    Word16 beta_fx;/*Q14 1.05f; */

    set32_fx( L_y2, 0x0L, L_FRAME48k );
    set16_fx( inp_vector_fx, 0, inner_frame_fx );
    flag_spt_fx = 0;
    move16();
    set16_fx(prev_SWB_peak_pos_tmp_fx, 0, NI_USE_PREV_SPT_SBNUM);
    adjustFlag = 0;
    move16();
    bw_low = 0;
    move16();
    bw_high = 20;
    move16();
    enerL_fx = L_deposit_l(0);
    enerH_fx = L_deposit_l(0);

    tmp2 = 0;       /* to avoid compilation warnings */

    L_bwe_br = L_add(st_fx->core_brate_fx, 0);
    hqswb_clas_fx = HQ_NORMAL;
    move16();
    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        IF ( sub(is_transient_fx, 1) == 0 )
        {
            hqswb_clas_fx = HQ_TRANSIENT;
            move16();
        }
        ELSE
        {
            hqswb_clas_fx = peak_avrg_ratio_fx( st_fx->total_brate_fx, L_t_audio, NUMC_N, &st_fx->mode_count_fx, &st_fx->mode_count1_fx, SWB_BWE_LR_Qs );
        }

        /* write the classification information into the bitstream */
        push_indice_fx( st_fx, IND_HQ2_SWB_CLAS, hqswb_clas_fx, 2 );
        (*num_bits_fx) = sub(*num_bits_fx, 2);
        if( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
        {
            flag_spt_fx = 1;
            move16();
        }
    }
    ELSE
    {
        /* write the transient bit into the bitstream */
        push_indice_fx( st_fx, IND_HQ2_SWB_CLAS, is_transient_fx, 1 );

        /* subtract one bit for the transient flag */
        (*num_bits_fx)--;
    }

    hq2_core_configure_fx( inner_frame_fx, *num_bits_fx, is_transient_fx, &bands_fx, &length_fx, band_width, band_start, band_end,
                           &L_qint, &eref_fx, &bit_alloc_weight_fx, &gqlevs_fx, &Ngq_fx, &p2a_bands_fx, &p2a_th_fx, &pd_thresh_fx, &ld_slope_fx, &ni_coef_fx
                           ,L_bwe_br);

    highlength_fx = band_end[sub(bands_fx, 1)];
    move16();
    har_bands_fx = bands_fx;
    move16();

    test();
    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && is_transient_fx == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        /* reserve bits for HQ_NORMAL2 and HQ_HARMONIC modes */
        test();
        IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 || sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
        {
            (*num_bits_fx) = sub(*num_bits_fx, get_usebit_npswb_fx(hqswb_clas_fx));
        }
    }

    test();
    test();
    IF(( L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) && sub(st_fx->bwidth_fx, SWB) == 0 )
    {
        IF( sub(st_fx->prev_hqswb_clas_fx, HQ_NORMAL) != 0 )
        {
            j = 0;
            move16();
            FOR(k=sub(bands_fx,NI_USE_PREV_SPT_SBNUM); k<bands_fx; k++)
            {
                st_fx->prev_SWB_peak_pos_fx[j] = 0;
                move16();
                j = add(j, 1);
            }
        }
    }

    /* Check if input frame is larger than coded bandwidth */
    test();
    IF ( sub(inner_frame_fx, length_fx) > 0 && is_transient_fx )
    {
        /* If so, collapse transient frame (4 short transforms) to remove uncoded coefficients */

        k1_step_fx = shr(length_fx, 2);      /* k1 = length/NUM_TIME_SWITCHING_BLOCKS */
        k2_step_fx = shr(inner_frame_fx, 2); /* k2 = inner_frame/NUM_TIME_SWITCHING_BLOCKS */
        k1_fx = k1_step_fx;
        k2_fx = k2_step_fx;
        FOR (i = 1; i < NUM_TIME_SWITCHING_BLOCKS; i++)
        {
            /*k1 = i*length/NUM_TIME_SWITCHING_BLOCKS; */
            /*k2 = i*inner_frame/NUM_TIME_SWITCHING_BLOCKS; */

            Copy32( &L_t_audio[k2_fx], &L_t_audio[k1_fx], k1_step_fx );

            k1_fx = add(k1_fx, k1_step_fx);
            k2_fx = add(k2_fx, k2_step_fx);
        }
    }

    /* Spectral energy calculation/quantization */
    ebits_fx = band_energy_quant_fx( st_fx, L_t_audio, band_start, band_end, L_band_energy, bands_fx,
                                     L_qint, eref_fx, is_transient_fx );

    /* First pass bit budget for TCQ of spectral band information */
    exp_norm = norm_s(gqlevs_fx); /* gqbits_fx = (short int) log2_f ((float) gqlevs_fx); */
    gqbits_fx = sub(14, exp_norm);

    bit_budget_fx = sub(sub(*num_bits_fx, ebits_fx), round_fx(L_shl(L_mult(Ngq_fx, gqbits_fx), 15))); /* (*num_bits) - (short) ceil (ebits) - Ngq * gqbits; */


    pbits_fx = 0;
    move16();

    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        IF( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
        {
            har_bands_fx = add(sub(bands_fx, p2a_bands_fx), 1);
            set16_fx( p2a_flags_fx, 1, har_bands_fx );
            set16_fx( &p2a_flags_fx[har_bands_fx], 0, sub(bands_fx, har_bands_fx) );
        }
        ELSE
        {
            /* High band tonality detector based on per band peak-to-average ratio */
            pbits_fx = p2a_threshold_quant_fx( st_fx, L_t_audio, band_start, band_end, band_width, bands_fx, p2a_bands_fx, p2a_th_fx, p2a_flags_fx );
            bit_budget_fx = sub(bit_budget_fx, pbits_fx);

            IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
            {
                return_bits_normal2_fx( &bit_budget_fx, p2a_flags_fx, bands_fx, bits_lagIndices_fx );
            }
        }
    }
    ELSE
    {
        /* High band tonality detector based on per band peak-to-average ratio */
        pbits_fx = p2a_threshold_quant_fx( st_fx, L_t_audio, band_start, band_end, band_width, bands_fx, p2a_bands_fx, p2a_th_fx, p2a_flags_fx );
        bit_budget_fx = sub(bit_budget_fx, pbits_fx);
    }

    IF(sub(flag_spt_fx, 1) == 0)
    {
        spt_shorten_domain_band_save_fx(bands_fx, band_start, band_end, band_width, org_band_start, org_band_end, org_band_width);
        spt_shorten_domain_pre_fx(band_start, band_end, st_fx->prev_SWB_peak_pos_fx, bands_fx, L_bwe_br, new_band_start, new_band_end, new_band_width);
        spt_shorten_domain_set_fx(st_fx, L_t_audio, p2a_flags_fx, new_band_start, new_band_end, new_band_width, bands_fx, band_start, band_end, band_width, &bit_budget_fx);
    }

    /* Estimate number of bits per band */
    Q_band_energy = SWB_BWE_LR_Qbe;

    FOR(i = 0; i < bands_fx; i++)
    {
        L_tmp = L_shl(L_band_energy[i],sub(16,Q_band_energy));/*Q16 */

        frac1 = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
        L_tmp =  Pow2(30, frac1);
        exp = sub(exp, 30);
        Ep_fx[i] = L_shl(L_tmp , sub(exp,6)); /* Q -6 */
    }

    FOR( i = 0; i < bands_fx; i++ )
    {
        L_tmp2 = Ep_fx[i];
        L_tmp = L_max(1, L_tmp2);
        exp = norm_l(L_tmp);
        tmp = extract_h(L_shl(L_tmp, exp));

        L_tmp3 = (Word32)band_width[i];
        exp2 = norm_l(L_tmp3);
        tmp2 = extract_h(L_shl(L_tmp3, exp2));

        exp2 = sub(exp, exp2); /* Denormalize and substract */

        tmp3 = sub(tmp2, tmp);
        if (tmp3 > 0)
        {
            tmp2 = shr(tmp2, 1);
        }
        if (tmp3 > 0)
        {
            exp2 = add(exp2, 1);
        }
        tmp = div_s(tmp2, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp2);/*Q(31-exp2) */
        Ep_tmp_fx[i] = L_shr(L_tmp,sub(15,exp2));/*Q13 */
    }

    test();
    test();
    test();
    test();
    test();
    test();
    IF ( is_transient_fx == 0 && sub(inner_frame_fx, L_FRAME8k) == 0 && L_sub(st_fx->core_brate_fx, ACELP_13k20) <= 0 )
    {
        lowband = 6;
        move16();
        trans_bit = 2;
        move16();
        bit_budget_fx =sub(bit_budget_fx,trans_bit);
        gama_fx = 27852; /*Q15 0.85f;// */
        beta_fx = 17203;
        move16();/*Q14 1.05f; */

        set16_fx( &p2a_flags_tmp[sub(bands_fx,trans_bit)], 0, 2 );
        IF(L_sub(st_fx->core_brate_fx, ACELP_9k60)==0)
        {
            beta_fx = 18022;
            move16();/*q14 1.1f; */
            p2a_flags_tmp[sub(bands_fx,1)] = p2a_flags_fx[sub(bands_fx,1)];
        }
        ELSE IF(L_sub(st_fx->core_brate_fx, ACELP_13k20)==0)
        {
            beta_fx = 13107;
            move16();/*14 1.25f; */
            gama_fx = 31130;
            move16();/*0.95f; */
            Copy(&p2a_flags_fx[sub(bands_fx,trans_bit)], &p2a_flags_tmp[sub(bands_fx,trans_bit)], trans_bit);
        }
        /* calculate the the low band/high band energy and the variance/avrage of the envelopes */
        Ep_vari_fx = L_deposit_l(0);
        Ep_avrg_fx = L_deposit_l(0);
        Ep_avrgL_fx = L_deposit_l(0);
        Ep_peak_fx = L_deposit_l(0);
        FOR( i = 0; i < bands_fx; i++ )
        {
            IF( sub(i,lowband) >= 0)
            {
                Ep_vari_fx = L_add(Ep_vari_fx,L_abs(L_sub(Ep_tmp_fx[i],Ep_tmp_fx[sub(i,1)])));/*Q15 */
                Ep_avrg_fx = L_add(Ep_avrg_fx,Ep_tmp_fx[i]);/*Q15 */

            }
            ELSE
            {
                Ep_avrgL_fx = L_add(Ep_avrgL_fx,Ep_tmp_fx[i]);/*Q15 */
                IF(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
                {
                    Ep_peak_fx = L_add(Ep_tmp_fx[i], 0); /*Q15 */
                }
            }
        }
        /* modify the last p2a_bands subbands band_energies */
        Copy32( L_band_energy,L_band_energy_tmp,bands_fx ); /*Q_band_energy */
        L_tmp = Mult_32_16(Ep_peak_fx,24576);/*Q(13+14-15 = 12) 1.5 lowband = 6; */
        L_tmp2 =Mult_32_16(Ep_peak_fx,shl(sub(bands_fx,lowband),9));/*Q(13+9-15 = 7) */
        L_tmp3 =Mult_32_16(Ep_avrg_fx,1126);/*Q(13+9-15 = 7) */

        test();
        test();
        test();
        test();
        IF(( (L_sub(L_tmp, L_shr(Ep_avrgL_fx,1)) < 0  && L_sub(st_fx->core_brate_fx, ACELP_13k20) == 0 ) || L_sub(st_fx->core_brate_fx, ACELP_13k20) < 0 )&&
           L_sub(L_tmp2, L_tmp3) < 0 && L_sub(L_tmp2, L_shr(Ep_avrg_fx,7)) > 0)
        {
            FOR(i = lowband; i < bands_fx; i++)
            {
                L_tmp = Mult_32_16(Ep_avrg_fx,24576);/*Q(13+14-15 = 12) 1.5  */
                IF(L_sub(L_shr(Ep_tmp_fx[i],1), L_tmp) < 0)
                {
                    L_tmp = Mult_32_16(Ep_peak_fx,sub(bands_fx,lowband));/*Q(13+0-15 = -2) */
                    tmp = extract_h(L_shl(L_tmp,14));/*Q-4 */
                    IF(tmp != 0)
                    {
                        exp = norm_s(tmp);
                        tmp = shl(tmp,exp);/*Q(exp) */
                        tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp = L_shl(Mult_32_16(Ep_avrg_fx,tmp),sub(13,exp));/*Q(13+exp-15 +13-exp +4 = 15) */
                    L_tmp2 = L_add(L_tmp,13107); /*15 */
                    tmp2 = extract_l(L_min(L_max(L_tmp2,16384),gama_fx)); /*15 = 15 */
                    L_band_energy_tmp[i] = Mult_32_16(L_band_energy_tmp[i],tmp2);/*Q(Q_band_energy+15-15 = Q_band_energy) */
                }
            }
        }
        ELSE
        {
            FOR(i = sub(bands_fx,trans_bit); i < bands_fx; i++)
            {
                alpha_fx = 16384;
                move16();/*Q14 */
                IF( sub(p2a_flags_tmp[i],1) == 0)
                {
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],sub(bands_fx,lowband));/*Q(13+0-15 = -2) */
                    tmp = extract_h(L_shl(L_tmp,14));/*Q-4 */
                    IF(tmp != 0)
                    {
                        exp = norm_s(tmp);
                        tmp = shl(tmp,exp);/*Q(exp) */
                        tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp =Mult_32_16(Ep_vari_fx,3277);/*13+15-15=13 */
                    L_tmp = L_shl(Mult_32_16(L_tmp,tmp),sub(12,exp));/*Q(13+exp-15 +12-exp +4 = 14) */

                    tmp2 = extract_h(Ep_avrg_fx);/*Q13-16=-3 */
                    IF(tmp2 != 0)
                    {
                        exp = norm_s(tmp2);
                        tmp2 = shl(tmp2,exp);/*Q(exp) */
                        tmp2 = div_s(16384,tmp2);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp2 = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp2 =Mult_32_16(Ep_vari_fx,6554);/*13+15-15=13 */
                    L_tmp2 = L_shl(Mult_32_16(L_tmp2,tmp2),sub(13,exp));/*Q(13+exp-15 +13-exp +3 = 14) */
                    L_tmp=L_min(L_tmp,L_tmp2);/*14 */
                    tmp=extract_l(L_min(L_tmp,13107));/*14 */
                    alpha_fx =add(16384,tmp);

                }
                IF(sub(st_fx->last_bitalloc_max_band_fx[i],1) == 0)
                {
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],sub(bands_fx,lowband));/*Q(13+0-15 = -2) */
                    tmp = extract_h(L_shl(L_tmp,14));/*Q-2 */
                    IF(tmp != 0)
                    {
                        exp = norm_s(tmp);
                        tmp = shl(tmp,exp);/*Q(exp) */
                        tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp = L_shl(Mult_32_16(Ep_avrg_fx,tmp),sub(14,exp));/*Q(13+exp-15 +14-exp+2 = 14) */
                    L_tmp =L_max(L_tmp,16384); /*14 */
                    tmp=extract_l(L_min(L_tmp,beta_fx)); /*14 */
                    alpha_fx=shl(mult(alpha_fx,tmp),1);/*14+14-15 +1=14 */
                }
                ELSE
                {
                    tmp2 = extract_h(Ep_avrg_fx);/*13 -16 =-3 */
                    IF(tmp2 != 0)
                    {
                        exp = norm_s(tmp2);
                        tmp2 = shl(tmp2,exp);/*Q(exp) */
                        tmp2 = div_s(16384,tmp2);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp2 = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp = L_shl(Mult_32_16(Ep_tmp_fx[i],tmp2),sub(19,exp));/*Q(13+exp-15 +19-exp +3 = 20) */
                    L_tmp = Mult_32_16(L_tmp,shl(sub(bands_fx,lowband),9));/*20 +9 -15 =14 */
                    L_tmp =L_max(L_tmp,13926); /*14 */
                    tmp2 =extract_l(L_min(L_tmp,16384)); /*14 */
                    alpha_fx=shl(mult(alpha_fx,tmp2),1);/*14+14-15+1 =14 */
                }
                L_band_energy_tmp[i] = L_shl(Mult_32_16(L_band_energy_tmp[i],alpha_fx),1);/*Q(Q_band_energy+14-15 +1= Q_band_energy) */
            }
        }
        lowband = 3;
        move16();
        Ep_avrg_fx = L_deposit_l(0);
        Ep_avrgL_fx = L_deposit_l(0);
        Ep_peak_fx = L_deposit_l(0);
        FOR(i = 0; i < bands_fx; i++)
        {
            IF(sub(i,lowband) >=0 )
            {
                Ep_avrg_fx = L_add(Ep_avrg_fx,Ep_tmp_fx[i]);/*Q15 */
            }
            ELSE
            {
                Ep_avrgL_fx = L_add(Ep_avrgL_fx,L_shr(Ep_tmp_fx[i],1));/*Q12 */
                IF(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
                {
                    Ep_peak_fx = L_add(Ep_tmp_fx[i], 0); /*Q13 */
                }
            }
        }
        L_tmp = Mult_32_16(Ep_peak_fx,28262);/*Q(13+14-15 = 12) 1.725 lowband = 3; */
        L_tmp2 =Mult_32_16(Ep_avrgL_fx,24576);/*Q(12+14-15 = 11) */
        test();
        test();
        IF( L_sub(L_shr(Ep_avrg_fx,2), L_tmp2) > 0 && L_sub(L_shr(Ep_avrg_fx,4), L_tmp2) < 0 &&  L_sub(L_tmp, Ep_avrgL_fx)>0)
        {
            adjustFlag = 1;
            move16();
            FOR (i = 0; i < lowband; i++)
            {
                tmp = extract_h(Ep_avrgL_fx);/*Q-4 */
                IF(tmp != 0)
                {
                    exp = norm_s(tmp);
                    tmp = shl(tmp,exp);/*Q(exp) */
                    tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                    exp = sub(29,exp);
                }
                ELSE
                {
                    tmp = 0x7fff;
                    move16();
                    exp = 0;
                    move16();
                }
                L_tmp = Mult_32_16(Ep_peak_fx,tmp);/*Q(13+exp-15+4 = exp+2) */
                L_tmp = Mult_32_16(L_tmp,lowband);/*Q(exp+2+0-15 = exp-13) */
                L_tmp = Mult_32_16(L_tmp,18842);/*Q(exp-13+16-16 = exp-13) */
                L_tmp = L_shl(L_tmp,sub(27,exp));/*Q14 0.5 */
                tmp2=extract_l(L_min(L_tmp,19661));/*14 */
                L_tmp = Mult_32_16(L_band_energy_tmp[i],tmp2);/*Q(Q_band_energy+14-15 = Q_band_energy-1) */
                L_band_energy_tmp[i] = L_shl(L_tmp,1); /*Q_band_energy  */
            }
        }
        hq2_bit_alloc_fx( L_band_energy_tmp, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
                          *num_bits_fx, hqswb_clas_fx, st_fx->bwidth_fx, is_transient_fx );

        /* encode the last p2a_bands-1 subbands bit-allocation index of the previous frame */
        FOR(i = 1; i <= trans_bit; i++)
        {
            push_indice_fx ( st_fx, IND_HQ2_LAST_BA_MAX_BAND, st_fx->last_bitalloc_max_band_fx[bands_fx-i], 1 );
        }
    }
    ELSE IF( is_transient_fx == 0 && sub(inner_frame_fx, L_FRAME16k) == 0 )
    {
        bit_budget_fx = sub(bit_budget_fx,2);/* bits in high bands to indicate the last 2 subbands is allocated bits or not */
        FOR( i = 0; i < bands_fx; i++ )
        {
            Ep_tmp_fx[i] = L_shl(Ep_tmp_fx[i],2);
        }
        IF( L_sub( st_fx->core_brate_fx, ACELP_13k20 ) == 0)
        {
            lowband = 8;
            move16();
            highband = 15;
            move16();
            bw_low = sub(band_start[highband],band_start[lowband]);
            bw_high = sub(add(band_end[sub(bands_fx,1)],1),band_start[highband]);
        }
        ELSE
        {
            lowband = 8;
            move16();
            highband = 16;
            move16();
            bw_low = sub(band_start[highband],band_start[lowband]);
            bw_high = sub(add(band_end[sub(bands_fx,1)],1),band_start[highband]);
        }
        /* calculate the the low band/high band energy and the variance/avrage of the envelopes */
        enerL_fx = L_deposit_l(0);
        enerH_fx = L_deposit_l(0);
        Ep_vari_fx = L_deposit_l(0);
        Ep_avrg_fx = L_deposit_l(0);
        FOR( i = 0; i < bands_fx; i++ )
        {
            test();
            IF( sub(i,lowband) >= 0 && add(sub(i,bands_fx),p2a_bands_fx) < 0)
            {
                Ep_vari_fx = L_add(Ep_vari_fx,L_abs(L_sub(Ep_tmp_fx[i],Ep_tmp_fx[sub(i,1)])));/*Q15 */
                Ep_avrg_fx = L_add(Ep_avrg_fx,Ep_tmp_fx[i]);/*Q15 */
            }

            IF(sub(i,highband) >= 0)
            {
                enerH_fx  = L_add(enerH_fx,L_shl(Ep_fx[i],2));/*Q0 */
            }
            ELSE IF(sub(i,lowband) >= 0)
            {
                enerL_fx  = L_add(enerL_fx,L_shl(Ep_fx[i],2));/*Q0 */
            }
        }

        /* modify the last p2a_bands subbands band_energies */
        Copy32( L_band_energy,L_band_energy_tmp,bands_fx ); /*Q_band_energy */
        L_tmp = L_max(enerH_fx,enerL_fx);
        tmp = s_max(bw_low,bw_high);
        i = norm_l(L_tmp);
        j = norm_s(tmp);
        L_tmp = Mult_32_16(L_shl(enerH_fx,i), shl(bw_low,j)); /* i + j -15 */
        L_tmp2 = Mult_32_16(L_shl(enerL_fx,i), shl(bw_high,j)); /*i + j -15 */
        L_tmp2 = L_sub(L_tmp,L_tmp2);
        FOR( i = sub(bands_fx,p2a_bands_fx); i < bands_fx; i++ )
        {
            test();
            IF( sub(p2a_flags_fx[i],1) == 0 || L_tmp2 > 0 )
            {
                tmp = sub(bands_fx,p2a_bands_fx);
                tmp = sub(tmp,lowband);/*Q0 */

                tmp1 = extract_h(L_shl(Ep_avrg_fx,1));/*Q0 */
                IF(tmp1 != 0)
                {
                    exp = norm_s(tmp1);
                    tmp1 = shl(tmp1,exp);/*Q(exp) */
                    tmp1 = div_s(16384,tmp1);/*Q(15+14-exp = 29-exp) */
                    exp = sub(29,exp);
                }
                ELSE
                {
                    tmp1 = 0x7fff;
                    move16();
                    exp = 0;
                    move16();
                }
                L_tmp = Mult_32_16(Ep_tmp_fx[i],tmp1);/*Q(15+exp-15 = exp) */
                L_tmp = Mult_32_16(L_tmp,tmp);/*Q(exp+0-15 = exp-15) */
                L_tmp = Mult_32_16(L_tmp,16384);/*Q(exp-15+13-15 = exp-17) */
                L_tmp = L_shl(L_tmp,sub(32,exp));/*Q15 */
                tmp = extract_l(L_min(L_tmp,6554));/*Q15 */
                L_tmp = Mult_32_16(Ep_vari_fx,tmp1);/*Q(15+exp-15 = exp) */
                L_tmp = Mult_32_16(L_tmp,tmp);/*Q(exp+15-15 = exp) */
                L_tmp = L_shl(L_tmp,sub(15,exp));/*Q15 */
                tmp = extract_l(L_shr(L_min(L_tmp,13107),1));/*Q14 */
                alpha_fx = add(tmp,16384);/*Q14 */
            }
            ELSE
            {
                alpha_fx = 16384;
                move16();/*Q14 */
            }

            IF(add(sub(i,bands_fx),p2a_bands_fx) > 0)
            {
                IF(sub(st_fx->last_bitalloc_max_band_fx[i],1) == 0)
                {
                    tmp = sub(bands_fx,p2a_bands_fx);
                    tmp = sub(tmp,lowband);
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],tmp);/*Q(15+0-15 = 0) */
                    tmp = extract_h(L_shl(L_tmp,16));/*Q0 */
                    IF(tmp != 0)
                    {
                        exp = norm_s(tmp);
                        tmp = shl(tmp,exp);/*Q(exp) */
                        tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp = Mult_32_16(Ep_avrg_fx,tmp);/*Q(15+exp-15 = exp) */
                    L_tmp = L_shl(L_tmp,sub(14,exp));/*Q14 */
                    tmp = extract_l(L_min(L_max(L_tmp,16384),20480));/*Q14 */
                    L_tmp = L_mult(alpha_fx,tmp);/*Q(14+14+1=29) */
                    alpha_fx = extract_l(L_shr(L_tmp,15)); /*Q14                    */
                }
                ELSE
                {
                    tmp = sub(bands_fx,p2a_bands_fx);
                    tmp = sub(tmp,lowband);

                    tmp1 = extract_h(L_shl(Ep_avrg_fx,1));/*Q0 */
                    IF(tmp1 != 0)
                    {
                        exp = norm_s(tmp1);
                        tmp1 = shl(tmp1,exp);/*Q(exp) */
                        tmp1 = div_s(16384,tmp1);/*Q(15+14-exp=29-exp) */
                        exp = sub(29,exp);
                    }
                    ELSE
                    {
                        tmp1 = 0x7fff;
                        move16();
                        exp = 0;
                        move16();
                    }
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],tmp1);/*Q(15+exp-15 = exp) */
                    L_tmp = Mult_32_16(L_tmp,tmp);/*Q(exp+0-15 = exp-15) */
                    L_tmp = L_shl(L_tmp,sub(29,exp));/*Q14 */
                    tmp = extract_l(L_min(L_max(L_tmp,13926),16384));/*Q14 */
                    L_tmp = L_mult(alpha_fx,tmp);/*Q(14+14+1=29) */
                    alpha_fx = extract_l(L_shr(L_tmp,15)); /*Q14  */
                }
            }
            L_tmp = Mult_32_16(L_band_energy_tmp[i],alpha_fx);/*Q(Q_band_energy+14-15=Q_band_energy-1) */
            L_band_energy_tmp[i] = L_shl(L_tmp,1);/*Q Q_band_energy */
        }
        lowband = 6;
        move16();
        Ep_avrg_fx = L_deposit_l(0);
        Ep_avrgL_fx = L_deposit_l(0);
        Ep_peak_fx = L_deposit_l(0);
        FOR(i = 0; i < bands_fx; i++)
        {
            IF(sub(i,lowband) >= 0)
            {
                Ep_avrg_fx = L_add(Ep_avrg_fx,Ep_tmp_fx[i]);/*Q15 */
            }
            ELSE
            {
                Ep_avrgL_fx = L_add(Ep_avrgL_fx,Ep_tmp_fx[i]);/*Q15 */
                IF(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
                {
                    Ep_peak_fx = L_add(Ep_tmp_fx[i], 0); /*Q15 */
                }
            }
        }

        L_tmp = Mult_32_16(Ep_peak_fx,24576);/*Q(15+12-15 = 12) lowband = 6; */
        L_tmp2 =Mult_32_16(Ep_peak_fx,19661);/*Q(15+14-15 = 14) */
        L_tmp3 =Mult_32_16(Ep_avrgL_fx,24576);/*Q(15+12-15 = 12) */

        test();
        test();
        test();
        test();
        test();
        IF( (L_sub(L_shr(Ep_avrgL_fx,1), Ep_avrg_fx)>0 && L_sub(L_tmp,L_shr(Ep_avrgL_fx,2)) > 0  && L_sub(L_shr(Ep_avrgL_fx,1),L_tmp2) < 0 ) ||
            (L_sub(L_shr(Ep_avrg_fx,1), Ep_avrgL_fx)>0 && L_sub(L_shr(Ep_avrg_fx,3),L_tmp3) < 0 && L_sub(L_tmp,L_shr(Ep_avrgL_fx,2)) > 0 ) )
        {
            adjustFlag = 1;
            move16();
            FOR (i = 0; i < lowband; i++)
            {
                tmp = extract_h(L_shl(Ep_avrgL_fx,1));/*Q0 */
                IF(tmp != 0)
                {
                    exp = norm_s(tmp);
                    tmp = shl(tmp,exp);/*Q(exp) */
                    tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
                    exp = sub(29,exp);
                }
                ELSE
                {
                    tmp = 0x7fff;
                    move16();
                    exp = 0;
                    move16();
                }
                L_tmp = Mult_32_16(Ep_peak_fx,tmp);/*Q(15+exp-15 = exp) */
                L_tmp = Mult_32_16(L_tmp,lowband);/*Q(exp+0-15 = exp-15) */
                L_tmp = L_shl(L_tmp,sub(28,exp));/*Q14 0.5 */
                tmp = extract_l(L_min(L_tmp,19661));/*//Q14 */
                L_tmp = Mult_32_16(L_band_energy_tmp[i],tmp);/*Q(Q_band_energy+14-15 = Q_band_energy-1) */
                L_band_energy_tmp[i] = L_shl(L_tmp,1); /*Q_band_energy  */
            }
        }

        hq2_bit_alloc_fx( L_band_energy_tmp, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
                          *num_bits_fx, hqswb_clas_fx, st_fx->bwidth_fx, is_transient_fx );

        /* encode the last p2a_bands-1 subbands bit-allocation index of the previous frame */
        FOR(i = 1; i < p2a_bands_fx; i++)
        {
            push_indice_fx( st_fx, IND_HQ2_LAST_BA_MAX_BAND, st_fx->last_bitalloc_max_band_fx[bands_fx-i], 1 );
        }
    }
    ELSE IF( sub(st_fx->bwidth_fx, SWB) == 0 && sub(hqswb_clas_fx, HQ_HARMONIC) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) ==  0) )
    {
        /* bit allocation for harmonic mode */
        hq2_bit_alloc_har_fx( L_band_energy, bit_budget_fx, bands_fx, L_Rk, p2a_bands_fx, L_bwe_br, p2a_flags_fx, band_width );
    }
    ELSE
    {
        set16_fx(st_fx->last_bitalloc_max_band_fx, 0, BANDS_MAX);

        /* estimate number of bits per band */
        hq2_bit_alloc_fx(
            L_band_energy, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
            *num_bits_fx, hqswb_clas_fx, st_fx->bwidth_fx, is_transient_fx );
    }

    tcq_core_LR_enc_fx( st_fx, inp_vector_fx, /*t_audio, */L_t_audio, /*y2, */L_y2, bit_budget_fx, bands_fx, band_start, band_end, band_width, /*Rk*/L_Rk, npulses_fx, k_sort_fx,
                        p2a_flags_fx, p2a_bands_fx, st_fx->last_bitalloc_max_band_fx, inner_frame_fx, adjustFlag, is_transient_fx );
    /* Denormalize the coded MDCT spectrum */
    mdct_spectrum_denorm_fx( inp_vector_fx, L_y2, band_start, band_end, band_width, L_band_energy, npulses_fx, bands_fx, ld_slope_fx, pd_thresh_fx );

    /* Apply fine gain quantization to denormalized coded spectrum */
    mdct_spectrum_fine_gain_enc_fx( st_fx, L_t_audio, L_y2, band_start, band_end, k_sort_fx, bands_fx,
                                    L_qint, Ngq_fx, gqlevs_fx, gqbits_fx );


    /* Restore the band information */
    IF( sub(flag_spt_fx, 1) == 0 )
    {
        spt_shorten_domain_band_restore_fx(bands_fx, band_start, band_end, band_width, org_band_start, org_band_end, org_band_width);
    }

    /* Inject noise into components having relatively low pulse energy per band */
    ni_seed_fx = add(add(add(npulses_fx[0], shl(npulses_fx[1], 4)), shl(npulses_fx[2], 8)), shl(npulses_fx[3], 12));

    Copy32(L_y2, L_y2_ni, band_end[bands_fx-1]+1);
    hq2_noise_inject_fx( L_y2_ni, band_start, band_end, band_width, Ep_fx, L_Rk, npulses_fx, ni_seed_fx, bands_fx, 0, bw_low, bw_high, enerL_fx, enerH_fx,
                         st_fx->last_ni_gain_fx, st_fx->last_env_fx, &st_fx->last_max_pos_pulse_fx, p2a_flags_fx, p2a_bands_fx,
                         hqswb_clas_fx, st_fx->bwidth_fx, L_bwe_br );

    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        test();
        IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 || sub(hqswb_clas_fx, HQ_HARMONIC) == 0)
        {
            preset_hq2_swb_fx( hqswb_clas_fx, band_end, &har_bands_fx, p2a_bands_fx,length_fx, bands_fx, &lowlength_fx, &highlength_fx, L_m );

            swb_bwe_enc_lr_fx(
                st_fx,
                L_y2, SWB_BWE_LR_Qs, L_t_audio, L_m,
                L_bwe_br,
                bands_fx, band_start, band_end,
                L_band_energy, SWB_BWE_LR_Qbe,
                p2a_flags_fx,
                hqswb_clas_fx, lowlength_fx, highlength_fx,
                st_fx->prev_frm_index_fx,
                har_bands_fx,
                &st_fx->prev_frm_hfe2_fx, &st_fx->prev_stab_hfe2_fx,
                band_width, L_y2_ni, &ni_seed_fx
            );

            post_hq2_swb_fx( L_m, lowlength_fx, highlength_fx, hqswb_clas_fx, har_bands_fx, bands_fx, p2a_flags_fx, band_start, band_end, L_y2, npulses_fx );

            IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
            {
                spt_swb_peakpos_tmp_save_fx(L_y2, bands_fx, band_start, band_end, prev_SWB_peak_pos_tmp_fx);
            }
            Copy32(L_y2_ni, L_y2, lowlength_fx);
        }
        ELSE
        {
            Copy32(L_y2_ni, L_y2, add(band_end[bands_fx-1], 1));            /* HQ_TRANSIENT */
        }
    }

    /* Copy the coded MDCT coefficient to the output buffer */
    IF ( !is_transient_fx )
    {
        /* Copy the scratch buffer to the output */
        Copy32( L_y2, L_t_audio, length_fx );

        /* If the input frame is larger than coded bandwidth, zero out uncoded MDCT coefficients */
        IF ( sub(inner_frame_fx, length_fx) > 0 )
        {
            set32_fx( L_t_audio + length_fx, 0x0L, sub(inner_frame_fx, length_fx) );
        }
    }
    ELSE /* transient frame */
    {
        IF( sub(inner_frame_fx, length_fx) == 0 )
        {
            /* Copy the scratch buffer to the output */
            Copy32( L_y2, L_t_audio, length_fx );
        }
        ELSE
        {
            /* length/NUM_TIME_SWITCHING_BLOCKS */
            /*length1_fx = div_s_ss(length_fx, NUM_TIME_SWITCHING_BLOCKS); */
            length1_fx = shr(length_fx, 2); /* length1 = length/NUM_TIME_SWITCHING_BLOCKS */
            /* inner_frame/NUM_TIME_SWITCHING_BLOCKS */
            /*length2_fx = div_s_ss(inner_frame_fx, NUM_TIME_SWITCHING_BLOCKS); */
            length2_fx = shr(inner_frame_fx, 2); /* length2 = inner_frame/NUM_TIME_SWITCHING_BLOCKS */
            /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */
            /*length3_fx = div_s_ss(sub(inner_frame_fx, length_fx), NUM_TIME_SWITCHING_BLOCKS); */
            length3_fx = shr(sub(inner_frame_fx, length_fx) ,2); /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */

            k1_fx = 0;
            move16();
            k2_fx = 0;
            move16();

            /* un-collapse transient frame and interleave zeros */
            FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                /*k1 = i*length/NUM_TIME_SWITCHING_BLOCKS; */
                /*k2 = i*inner_frame/NUM_TIME_SWITCHING_BLOCKS; */

                Copy32( L_y2 + k1_fx, L_t_audio + k2_fx, length1_fx );
                set32_fx( L_t_audio + k2_fx + length1_fx, 0x0L, length3_fx);

                k1_fx = add(k1_fx, length1_fx);
                k2_fx = add(k2_fx, length2_fx);

            }
        }
    }

    test();
    test();
    test();
    IF( sub(hqswb_clas_fx, HQ_HARMONIC) != 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) && sub(st_fx->bwidth_fx, SWB) == 0 )
    {
        st_fx->prev_frm_index_fx[0] = -1;
        move16();
        st_fx->prev_frm_index_fx[1] = -1;
        move16();
        st_fx->prev_frm_hfe2_fx = 0;
        move16();
        st_fx->prev_stab_hfe2_fx = 0;
        move16();
    }

    /* update */
    test();
    test();
    IF( (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) && sub(st_fx->bwidth_fx, SWB) == 0 )
    {
        st_fx->prev_hqswb_clas_fx = hqswb_clas_fx;
        move16();
    }
    ELSE
    {
        st_fx->prev_hqswb_clas_fx = is_transient_fx;
        move16();
    }

    IF( sub(st_fx->bwidth_fx, SWB) != 0 )
    {
        /* reset HQ classifier memories */
        st_fx->mode_count_fx = 0;
        move16();
        st_fx->mode_count1_fx = 0;
        move16();
    }

    test();
    test();
    test();
    IF( (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) && sub(st_fx->bwidth_fx, SWB) == 0 && sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
    {
        j = 0;
        move16();
        FOR(k=sub(bands_fx, NI_USE_PREV_SPT_SBNUM); k<bands_fx; k++)
        {
            st_fx->prev_SWB_peak_pos_fx[j] = prev_SWB_peak_pos_tmp_fx[j];
            move16();
            j = add(j, 1);
        }
    }

    /* update number of unused bits */
    *num_bits_fx = 0;
    move16();

    st_fx->hvq_hangover_fx = 0;
    move16();

    return;
}

/*--------------------------------------------------------------------------*
 * small_symbol_enc_tran()
 *
 * Huffman encoding of differential energies, estimating or packing bits
 * if flag_pack = 0, LC mode info. is output else LC mode info. is input
 * if flag_pack = 0, estimatng else packing bits
 *--------------------------------------------------------------------------*/

static Word16 small_symbol_enc_tran_fx(  /* o  : bits                                     */
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure                  */
    const Word16  *qbidx,                /* i  : input of dequantized differential energy */
    const Word16  BANDS,                 /* i  : number of bands                          */
    Word16  *hLCmode,              /* i/o: LC mode info                             */
    const Word16  flag_pack,             /* i  : indicator of packing or estimating bits  */
    const Word16  is_transient
)
{
    Word16 i, bits;
    Word16 difidx[BANDS_MAX];

    FOR( i=0; i<BANDS; i++ )
    {
        difidx[i] = add(qbidx[i], LRMDCT_BE_OFFSET);
        move16();
    }

    FOR( i=0; i<BANDS; ++i )
    {
        test();
        IF ( sub(difidx[i],LRMDCT_BE_LIMIT) > 0 || difidx[i] < 0 )
        {
            /* Huffman cannot encode this vector */
            return -1;
        }
    }

    /* Preparing lossless coding input */
    IF ( flag_pack == 0 )
    {
        /* estimating # of bits */
        bits = encode_envelope_indices_fx(st_fx, BANDS, -1, difidx, hLCmode, flag_pack, LOW_RATE_HQ_CORE_TRAN , is_transient );
        bits = add(bits, BITS_DE_FCOMP);   /* xx bits for diff. energies + BITS_DE_FCOMP bits for first energies */
    }
    ELSE
    {
        bits = 0;
        move16();
        encode_envelope_indices_fx(st_fx, BANDS, -1, difidx, hLCmode, flag_pack, LOW_RATE_HQ_CORE_TRAN ,is_transient );
    }

    return add(bits, BITS_DE_HMODE);  /* xx bits for diff. energies + 1 bit for LC coding mode */
}


/*--------------------------------------------------------------------------*
 * small_symbol_enc()
 *
 * Huffman encoding of differential energies, estimating or packing bits
 * if flag_pack = 0, LC mode info. is output else LC mode info. is input
 * if flag_pack = 0, estimatng else packing bits
 *--------------------------------------------------------------------------*/

static Word16 small_symbol_enc_fx(  /* o  : bits                                     */
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure                  */
    const Word16 *qbidx,            /* i  : input of dequantized differential energy */
    const Word16 BANDS,             /* i  : number of bands                          */
    Word16 *hLCmode,          /* i/o: LC mode info                             */
    const Word16 flag_pack          /* i  : indicator of packing or estimating bits  */
    ,const Word16 is_transient
)
{
    Word16 i, bits;
    Word16 difidx[BANDS_MAX], LSB[BANDS_MAX];

    /* Preparing lossless coding input */
    difidx[0] = add(qbidx[0], DE_OFFSET0);
    move16();

    FOR( i=1; i<BANDS; ++i )
    {
        difidx[i] = add(qbidx[i], DE_OFFSET1);
        move16();
    }

    FOR( i=0; i<BANDS; ++i )
    {
        test();
        IF ( sub(difidx[i], DE_LIMIT) >= 0 || difidx[i] < 0 )
        {
            /* Huffman cannot encode this vector */
            return -1;
        }
    }

    /* splitting MSB and LSB */
    FOR( i=0; i<BANDS; ++i )
    {
        LSB[i] = s_and(difidx[i], 1);
        move16();
        difidx[i] = shr(difidx[i], 1);
        move16();
    }

    /* Preparing lossless coding input */
    IF ( flag_pack == 0 )
    {
        /* estimating # of bits */
        /* Encoding MSB bits */
        bits = encode_envelope_indices_fx( st_fx, BANDS, -1, difidx, hLCmode, flag_pack, LOW_RATE_HQ_CORE, is_transient );
        bits = add(bits, BITS_DE_FCOMP);   /* xx bits for diff. energies + BITS_DE_FCOMP bits for first energies */

        /* Encoding LSB bit packing */
        bits = add(bits, BANDS);
    }
    ELSE
    {
        /* Encoding MSB bits */
        bits = 0;
        move16();
        encode_envelope_indices_fx( st_fx, BANDS, -1, difidx, hLCmode, flag_pack, LOW_RATE_HQ_CORE, is_transient );

        /* Encoding LSB bit packing */
        FOR( i=0; i<BANDS; ++i )
        {
            push_indice_fx( st_fx, IND_HQ2_DIFF_ENERGY, LSB[i], BITS_DE_LSB);
        }
    }

    return add(bits, BITS_DE_HMODE);  /* xx bits for diff. energies + 1 bit for LC coding mode */
}

static Word16 large_symbol_enc_fx(            /* o  : bits                                     */
    Encoder_State_fx *st_fx,                     /* i  : encoder state structure                  */
    Word16   *qbidx,                  /* i  : input of dequantized differential energy */
    const Word16 BANDS,                     /* i  : number of bands                          */
    Word16 *hLCmode0,                 /* i/o: LC mode info                             */
    const Word16 flag_pack                  /* i  : indicator of packing or estimating bits  */
)
{
    Word16 i, bits, tmp;
    Word16 LSB1[BANDS_MAX];
    Word16 min_q,max_q,offset0;
    Word16 min_bits,min_bits_pos;
    Word16 tdifidx0[BANDS_MAX], tdifidx1[BANDS_MAX];
    Word16 basic_shift;
    Word16 bitsmode0,bitsmode1;
    Word16 lsbdepth1;
    Word16 cnt_outlyer,pos_outlyer,cnt_outlyer0;

    min_q = 513;
    move16();
    max_q = -1;
    move16();

    cnt_outlyer0 = 0;
    move16();
    cnt_outlyer = 0;
    move16();
    bitsmode0 = 0;
    move16();
    bitsmode1 = 0;
    move16();
    pos_outlyer = 0;
    move16();
    lsbdepth1 = 0;
    move16();

    test();
    test();
    IF ( flag_pack == 0 || ( sub(flag_pack, 1) == 0 && *hLCmode0 == 0) )
    {
        test();
        test();
        IF ( sub(qbidx[0], sub(ABS_ENG_OFFSET, 1)) > 0 || sub(qbidx[0],-ABS_ENG_OFFSET) < 0)
        {
            cnt_outlyer0 = 2;
            move16();
        }
        ELSE IF ( sub(qbidx[0],3) > 0 || sub(qbidx[0],-4) < 0 )
        {
            cnt_outlyer0 = 1;
            move16();
        }
        ELSE
        {
            cnt_outlyer0 = 0;
            move16();
        }

        cnt_outlyer = 0;
        move16();
        pos_outlyer = -1;
        move16();
        FOR( i=1; i<BANDS; ++i )
        {
            test();
            IF ( sub(qbidx[i],3) > 0 || sub(qbidx[i],-4) < 0 )
            {
                cnt_outlyer = add(cnt_outlyer, 1);
                pos_outlyer = i;
                move16();
            }

            test();
            if ( sub(qbidx[i], sub(ABS_ENG_OFFSET,1)) > 0 || sub(qbidx[i], -ABS_ENG_OFFSET) < 0 )
            {
                cnt_outlyer = add(cnt_outlyer, 1);
            }
        }

        test();
        test();
        IF ( cnt_outlyer0 == 0 && sub(cnt_outlyer, 1) <= 0 )
        {
            bitsmode0 = add(add(BITS_DE_8SMODE, BITS_DE_8SMODE_N0), BITS_DE_8SMODE_N1);
            IF ( sub(cnt_outlyer, 1)  == 0 )
            {
                /* 01 */
                bitsmode0 = add(bitsmode0, add(BITS_DE_8SPOS, BITS_ABS_ENG));
            }

            FOR( i=0; i<pos_outlyer; ++i )
            {
                tdifidx0[i] = qbidx[i];
                move16();
                bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
            }

            FOR( i=add(pos_outlyer, 1); i<BANDS; ++i )
            {
                tdifidx0[i] = qbidx[i];
                move16();
                bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
            }
        }
        ELSE IF ( sub(cnt_outlyer0, 1) == 0 && sub(cnt_outlyer, 1) <= 0 )
        {
            bitsmode0 = add(add(BITS_DE_8SMODE, BITS_DE_8SMODE_N0), BITS_DE_8SMODE_N1);
            tdifidx0[0] = qbidx[0];
            move16();
            bitsmode0 = add(bitsmode0, BITS_ABS_ENG);
            IF ( sub(cnt_outlyer, 1) == 0 )
            {
                /* 11 */
                bitsmode0 = add(bitsmode0, add(BITS_DE_8SPOS, BITS_ABS_ENG));
            }
            ELSE
            {
                pos_outlyer = 0;
                move16();
            }

            FOR( i=1; i<pos_outlyer; ++i )
            {
                tdifidx0[i] = qbidx[i];
                move16();
                bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
            }

            FOR( i=add(pos_outlyer, 1); i<BANDS; ++i )
            {
                tdifidx0[i] = qbidx[i];
                move16();
                bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
            }
        }
        ELSE
        {
            bitsmode0 = 20000;
            move16();
        }
    }

    test();
    IF ( flag_pack == 0 || *hLCmode0==1 )
    {
        /* components 0 range : -256~255 */
        max_q = MINIMUM_ENERGY_LOWBRATE;
        move16();
        min_q = MAXIMUM_ENERGY_LOWBRATE;
        move16();
        FOR( i=0; i<BANDS; ++i )
        {
            max_q = s_max(qbidx[i], max_q);
            min_q = s_min(qbidx[i], min_q);
        }

        /* Counting bits for transmitting all components using same method */
        FOR(i=0;; ++i)
        {
            /*if (max_q <= ((2<<(i+1))-1) && min_q >= -(2<<(i+1))) */
            test();
            IF ( sub(max_q, sub(shl(2, add(i,1)), 1)) <= 0 && sub(min_q, -shl(2,add(i,1))) >= 0 )
            {
                BREAK;
            }
        }
        basic_shift = i;
        move16();

        min_bits = 1000;
        move16();
        min_bits_pos = basic_shift;
        move16();
        tmp = add(basic_shift, 3);
        FOR( offset0=basic_shift; offset0<tmp; offset0++ )
        {
            max_q = MINIMUM_ENERGY_LOWBRATE;
            move16();
            min_q = MAXIMUM_ENERGY_LOWBRATE;
            move16();

            bitsmode1 = add(BITS_DE_8SMODE, BITS_MAX_DEPTH);
            FOR( i=0; i<BANDS; ++i)
            {
                bitsmode1 = add(bitsmode1, add(hessize_fx[add(shr(qbidx[i], offset0), 4)], offset0));
            }

            IF ( sub(min_bits, bitsmode1) > 0 )
            {
                min_bits_pos = offset0;
                move16();
                min_bits = bitsmode1;
                move16();
            }
        }

        bitsmode1 = min_bits;
        move16();
        lsbdepth1 = min_bits_pos;
        move16();

        FOR( i=0; i<BANDS; ++i )
        {
            LSB1[i] = s_and(qbidx[i], sub(shl(1, lsbdepth1), 1));
            tdifidx1[i] = shr(qbidx[i], lsbdepth1);
        }
    }

    /* Preparing lossless coding input */
    IF ( flag_pack == 0 )
    {
        /* estimating # of bits */
        /* Encoding MSB bits */
        IF ( sub(bitsmode0, bitsmode1) < 0 )
        {
            bits = bitsmode0;
            move16();
            *hLCmode0 = 0;
            move16();
        }
        ELSE
        {
            bits = bitsmode1;
            move16();
            *hLCmode0 = 1;
            move16();
        }
    }
    ELSE
    {
        /* Encoding MSB bits */
        IF ( *hLCmode0 == 0 )
        {
            push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE, 0, BITS_DE_8SMODE);
            bits = BITS_DE_8SMODE;
            move16();
            IF ( cnt_outlyer0 == 0 )
            {
                push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N0, 0, BITS_DE_8SMODE_N0);
                bits = add(bits, BITS_DE_8SMODE_N0);
                IF ( sub(cnt_outlyer, 1) == 0 )
                {
                    /* 01 */
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N1, 1, BITS_DE_8SMODE_N1);
                    bits = add(bits, BITS_DE_8SMODE_N1);
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SPOS, pos_outlyer, BITS_DE_8SPOS);
                    bits = add(bits, BITS_DE_8SPOS);
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, qbidx[pos_outlyer]+ABS_ENG_OFFSET, BITS_ABS_ENG);
                    bits = add(bits, BITS_ABS_ENG);
                }
                ELSE
                {
                    /* 00 */
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N1, 0, BITS_DE_8SMODE_N1);
                    bits = add(bits, BITS_DE_8SMODE_N1);
                }

                FOR( i=0; i<pos_outlyer; ++i )
                {
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, hescode_fx[tdifidx0[i]+4], hessize_fx[tdifidx0[i]+4]);
                    bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
                }

                FOR( i=add(pos_outlyer, 1); i<BANDS; ++i )
                {
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, hescode_fx[tdifidx0[i]+4], hessize_fx[tdifidx0[i]+4]);
                    bitsmode0 = add(bitsmode0, hessize_fx[tdifidx0[i]+4]);
                }
            }
            ELSE IF ( sub(cnt_outlyer0, 1) == 0 )
            {
                push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N0, 1, BITS_DE_8SMODE_N0);
                bits = add(bits, BITS_DE_8SMODE_N0);
                IF ( sub(cnt_outlyer, 1) == 0 )
                {
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N1, 1, BITS_DE_8SMODE_N1);
                    bits = add(bits, BITS_DE_8SMODE_N1);
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SPOS, pos_outlyer, BITS_DE_8SPOS);
                    bits = add(bits, BITS_DE_8SPOS);
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, qbidx[0]+ABS_ENG_OFFSET, BITS_ABS_ENG);
                    bits = add(bits, BITS_ABS_ENG);
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, qbidx[pos_outlyer]+ABS_ENG_OFFSET, BITS_ABS_ENG);
                    bits = add(bits, BITS_ABS_ENG);
                }
                ELSE
                {
                    push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE_N1, 0, BITS_DE_8SMODE_N1);
                    bits = add(bits, BITS_DE_8SMODE_N1);
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, qbidx[0]+ABS_ENG_OFFSET, BITS_ABS_ENG);
                    bits = add(bits, BITS_ABS_ENG);
                }

                FOR( i=1; i<pos_outlyer; ++i )
                {
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, hescode_fx[tdifidx0[i]+4], hessize_fx[tdifidx0[i]+4]);
                    bits = add(bits, hessize_fx[tdifidx0[i]+4]);
                }

                FOR( i=pos_outlyer+1; i<BANDS; ++i )
                {
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, hescode_fx[tdifidx0[i]+4], hessize_fx[tdifidx0[i]+4]);
                    bits = add(bits, hessize_fx[tdifidx0[i]+4]);
                }
            }
        }
        ELSE
        {
            bits = add(BITS_DE_8SMODE, BITS_MAX_DEPTH);
            push_indice_fx(st_fx, IND_HQ2_DENG_8SMODE, 1, BITS_DE_8SMODE);
            push_indice_fx(st_fx, IND_HQ2_DENG_8SDEPTH, lsbdepth1, BITS_MAX_DEPTH);

            FOR(i=0; i<BANDS; ++i)
            {
                push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, hescode_fx[tdifidx1[i]+4], hessize_fx[tdifidx1[i]+4]);
                bits = add(bits, hessize_fx[tdifidx1[i]+4]);
            }

            IF ( lsbdepth1 > 0 )
            {
                FOR( i=0; i<BANDS; ++i )
                {
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, LSB1[i], lsbdepth1);
                }
                /*bits += BANDS * lsbdepth1; */
                bits = add(bits, extract_h(L_shl(L_mult(BANDS, lsbdepth1), 15)));
            }
        }
    }

    return bits;  /* xx bits for diff. energies + 1 bit for LC coding mode */
}

/*-------------------------------------------------------------------*
 * band_energy_quant()
 *
 *
 *-------------------------------------------------------------------*/

static Word16 band_energy_quant_fx(
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure           */
    const Word32 *L_t_audio,         /* i  : Q12 : input MDCT signal (Qs)      */
    const Word16 band_start[],       /* i  : Q0  : band start table            */
    const Word16 band_end[],         /* i  : Q0  : band end table              */
    Word32 L_band_energy[],    /* i  : Q14 : band energy       (Qbe)     */
    const Word16 bands_fx,           /* i  : Q0  : number of bands             */
    const Word32 L_qint,             /* i  : Q29 */
    const Word16 eref_fx,            /* i  : Q10 */
    const Word16 is_transient_fx     /* i  : Q0  : indicator for HQ_TRANSIENT */
)
{
    Word16 i, k;
    Word16 ebits;
    Word16 hLCmode0,hLCmode1,deng_bits;
    Word16 deng_cmode = 0;
    Word16 hbits;

    Word32 L_E;
    Word16 QE;

    Word16 rev_qint_fx;  /* 1/qint */
    Word16 Qrev_qint;    /* Q value for 1/qint */

    Word16 bq0_fx;
    Word16 bq1_fx[BANDS_MAX];
    Word16 bq2_fx[BANDS_MAX];

    Word16 bq1_temp_fx[BANDS_MAX];
    Word16 bq2_temp_fx[BANDS_MAX];

    Word32 L_tmp;
    Word16 temp_fx;
    Word16 frac, exp;
    Word16 exp_safe;
    Word16 exp_norm;
    Word16 exp_norm2;
    Word16 exp_normd;

    /* Calculate the band energies */
    exp_safe = 4;
    move16(); /* 4: never overflow happen at L_E */
    FOR (k = 0; k < bands_fx; k++)
    {
        L_tmp = L_deposit_l(1295); /* 1295 = sqrt(0.1) (Qs) */
        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            L_tmp = L_or(L_tmp, L_abs(L_t_audio[i]));
        }
        exp_norm = norm_l(L_tmp);
        exp_norm = sub(exp_norm, exp_safe); /* safe_shift */

        QE = add(shl(sub(add(SWB_BWE_LR_Qs, exp_norm), 16), 1), 1);
        L_E = L_shl(590L, sub(QE,15)); /* 590   0.018f(Q15 -> QE) */
        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            /*E += yos[i] * yos[i]; */
            temp_fx = round_fx(L_shl(L_t_audio[i], exp_norm));
            L_E = L_mac(L_E, temp_fx, temp_fx); /* (Qs+exp_norm-16)*2+1 */
        }

        /*band_energy[k] = (float) log2_f (E + 1.0e-1f); */
        exp_norm2 = norm_l(L_E);
        exp = add(add(shl(sub(add(SWB_BWE_LR_Qs, exp_norm), 16), 1), 1),  exp_norm2);
        L_E = L_shl(L_E, exp_norm2);
        frac = Log2_norm_lc(L_E);
        exp = sub(30, exp);
        L_tmp = L_Comp(exp, frac);
        L_band_energy[k] = L_shr(L_tmp, 2); /* Q16->Qbe(Q14) */
    }

    IF (is_transient_fx)
    {
        reverse_transient_frame_energies_fx( L_band_energy, bands_fx );
    }


    /* Quantize the reference and band energies */
    exp_normd = norm_l(L_qint);
    rev_qint_fx = div_s(0x4000, round_fx(L_shl(L_qint, exp_normd))); /* Q14-(29+exp_normd-16)+15 */
    Qrev_qint = sub(14-(29-16)+15, exp_normd);

    bq0_fx = round_fx(L_shl(L_mult(eref_fx, rev_qint_fx), sub(5, Qrev_qint))); /* 16-(10+Qrev_qint+1) */
    FOR (k = 0; k < bands_fx; k++)
    {
        /*bq1[k] = round_f (band_energy[k] / qint); */
        L_tmp = Mpy_32_16_1(L_band_energy[k], rev_qint_fx); /* Q14+Qrev_qint-15 */
        bq1_fx[k] = round_fx( L_shl(L_tmp, sub(17, Qrev_qint)) ); /* 16-(14+Qrev_qint-15) */
    }

    IF(is_transient_fx)
    {

        Copy(bq1_fx, bq1_temp_fx, bands_fx);

        /* Calculate the differential energies */
        diffcod_lrmdct_fx(bands_fx, bq0_fx, bq1_temp_fx, bq2_temp_fx, is_transient_fx);
    }

    /* Calculate the differential energies */
    bq2_fx[0] = sub(bq1_fx[0], bq0_fx);
    FOR (k = 1; k < bands_fx; k++)
    {
        bq2_fx[k] = sub(bq1_fx[k], bq1_fx[k - 1]);
        move16();
    }

    /* Modifying qbidx to be located in the range -256~255 */
    FOR( i=0; i<bands_fx; ++i )
    {
        if ( sub(bq2_fx[i],MAXIMUM_ENERGY_LOWBRATE) > 0 )
        {
            bq2_fx[i] = MAXIMUM_ENERGY_LOWBRATE;
            move16();
        }
        if ( sub(bq2_fx[i], MINIMUM_ENERGY_LOWBRATE) < 0 )
        {
            bq2_fx[i] = MINIMUM_ENERGY_LOWBRATE;
            move16();
        }
    }

    /* Get number of bits by Huffman0 coding */
    ebits = large_symbol_enc_fx( st_fx, bq2_fx, bands_fx, &hLCmode0, 0 );

    IF(is_transient_fx)
    {
        /* Get number of bits by Huffman coding */
        hbits = small_symbol_enc_tran_fx(st_fx, bq2_temp_fx, bands_fx, &hLCmode1, 0 ,is_transient_fx);
    }
    ELSE
    {
        /* Get number of bits by Huffman coding */
        hbits = small_symbol_enc_fx( st_fx, bq2_fx, bands_fx, &hLCmode1, 0, is_transient_fx );
    }

    /* comparing used bits */
    test();
    IF ( sub(ebits, hbits) < 0 || sub(hbits, -1) == 0 )
    {
        deng_cmode = 0;
        move16();
        push_indice_fx ( st_fx, IND_HQ2_DENG_MODE, deng_cmode, BITS_DE_CMODE );
        large_symbol_enc_fx( st_fx, bq2_fx, bands_fx, &hLCmode0, 1 );
        deng_bits = add(ebits, BITS_DE_CMODE);
    }
    ELSE
    {
        /* setting energy difference coding mode and storing it */
        deng_cmode = 1;
        move16();
        push_indice_fx( st_fx, IND_HQ2_DENG_MODE, deng_cmode, BITS_DE_CMODE );

        deng_bits = add(hbits, BITS_DE_CMODE);

        /* packing indice */
        IF(is_transient_fx)
        {
            Copy(bq2_temp_fx, bq2_fx, bands_fx);
            small_symbol_enc_tran_fx(st_fx, bq2_fx, bands_fx, &hLCmode1, 1 ,is_transient_fx);
        }
        ELSE
        {
            small_symbol_enc_fx( st_fx, bq2_fx, bands_fx, &hLCmode1, 1, is_transient_fx );
        }
    }

    /* Reconstruct quantized spectrum */
    bq1_fx[0] = add(bq2_fx[0], bq0_fx);
    move16();
    FOR (k = 1; k < bands_fx; k++)
    {
        bq1_fx[k] = add(bq2_fx[k], bq1_fx[k - 1]);
        move16();
    }

    FOR (k = 0; k < bands_fx; k++)
    {
        L_band_energy[k] = Mpy_32_16_1(L_qint, bq1_fx[k]); /* 29+0-15 -> Qbe(Q14) */
    }

    IF (is_transient_fx)
    {
        reverse_transient_frame_energies_fx( L_band_energy, bands_fx );
    }

    return( deng_bits );
}


/*-------------------------------------------------------------------*
 * p2a_threshold_quant()
 *
 *
 *-------------------------------------------------------------------*/

static Word16 p2a_threshold_quant_fx(
    Encoder_State_fx *st,           /* i/o:     : encoder state structure */
    const Word32 *L_t_audio,    /* i  : Q12 : input spectrum          */
    const Word16 band_start[],  /* i  : Q0  : table of start freq for every subband */
    const Word16 band_end[],    /* i  : Q0  : table of end freq for every subband   */
    const Word16 band_width[],  /* i  : Q0  : table of bandwidth for every subband  */
    const Word16 bands,         /* i  : Q0  : number of subbands                    */
    const Word16 p2a_bands,     /* i  : Q0  : tonality indicator                    */
    const Word16 p2a_th_fx,     /* i  : Q11 : threshold tonal or not                */
    Word16 *p2a_flags_fx  /* i/o: Q0  : tonality flag                         */
)
{
    Word16 i, j, k;
    Word32 L_a, L_p, L_e;
    Word16 Qa;
    Word32 L_tmp;
    Word16 temp_fx;
    Word16 exp_norm;
    Word16 exp_safe;
    Word16 exp_normn, exp_normd;

    Word16 norm_a_fx,Qnorm_a;
    Word16 pa_fx, Qpa;

    Word16 exp, frac;

    Word32 L_p2a;
    Word16 p2a_fx;

    exp_safe=4; /* never happen overflow. */

    set16_fx(p2a_flags_fx, 1, bands);

    j = 0;
    move16();
    FOR (k = sub(bands, p2a_bands); k < bands; k++)
    {
        L_a = L_deposit_l(0);
        L_p = L_deposit_l(0);

        L_tmp = L_deposit_l(0);
        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            L_tmp = L_or(L_tmp, L_abs(L_t_audio[i]));
        }
        exp_norm = norm_l(L_tmp);
        exp_norm = sub(exp_norm, exp_safe);

        FOR (i = band_start[k]; i <= band_end[k]; i++)
        {
            temp_fx = round_fx(L_shl(L_t_audio[i], exp_norm)); /* Q12+exp_norm-16  -> exp_norm-4 */
            L_e = L_mult(temp_fx, temp_fx);

            if ( L_sub(L_e, L_p) > 0 )
            {
                L_p = L_add(L_e, 0);
            }
            L_a = L_add(L_a, L_e);
        }
        Qa = sub(shl(exp_norm, 1), 7); /* (exp_norm-4)*2+1 */

        IF ( L_a > 0x0L )
        {
            /* a /= band_width[k]; */
            exp_normn = norm_l(L_a);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_s(band_width[k]);
            norm_a_fx = div_l(L_shl(L_a, exp_normn), shl(band_width[k], exp_normd));
            Qnorm_a = sub(sub(add(Qa, exp_normn), exp_normd), 1); /* (Qa+exp_normn)-exp_normd-1); */

            /*p2a = 10.0f * (float) log10 (p / a); */
            p2a_fx = 0;
            move16();
            IF ( norm_a_fx > 0 )
            {
                exp_normn = norm_l(L_p);
                exp_normn = sub(exp_normn, 1);
                exp_normd = norm_s(norm_a_fx);
                pa_fx = div_l(L_shl(L_p, exp_normn), shl(norm_a_fx, exp_normd));
                Qpa = sub(sub(add(Qa, exp_normn), add(Qnorm_a, exp_normd)), 1);

                L_tmp = L_deposit_h(pa_fx);
                Qpa = add(Qpa, 16);
                exp = norm_l(L_tmp);
                frac = Log2_norm_lc(L_shl(L_tmp, exp));
                exp = sub(30, exp);
                exp = sub(exp, Qpa);
                L_tmp = L_Comp(exp, frac);

                /* 10/( log(10)/log(2) ) = 3.01029995663981195211  24660(Q13) */
                L_p2a = Mpy_32_16_1(L_tmp, 24660); /* 16+13-15 -> Q14 */

                p2a_fx = round_fx(L_shl(L_p2a, 13)); /*  27 -16  -> 11 */
            }

            if ( sub(p2a_fx, p2a_th_fx) <= 0 )
            {
                p2a_flags_fx[k] = 0;
                move16();
            }
        }
        ELSE
        {
            p2a_flags_fx[k] = 0;
            move16();
        }

        push_indice_fx( st, IND_HQ2_P2A_FLAGS, p2a_flags_fx[k], 1 );
        j = add(j, 1);
    }

    return( j );
}

/*-------------------------------------------------------------------*
 * mdct_spectrum_fine_gain_enc()
 *
 *
 *-------------------------------------------------------------------*/

static void mdct_spectrum_fine_gain_enc_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure                     */
    const Word32 L_ybuf[],      /* i  : Q12 : input spectrum                        */
    Word32 L_y2[],        /* i/o: Q12 : decoded spectrum                      */
    const Word16 band_start[],  /* i  : Q0  : table of start freq for every subband */
    const Word16 band_end[],    /* i  : Q0  : table of end freq for every subband   */
    const Word16 k_sort[],      /* i  : Q0  : sort table by band_energy             */
    const Word16 bands,         /* i  : Q0  : nubmber of subbands                   */
    const Word32 L_qint,        /* i  : Q29 :                                       */
    const Word16 Ngq,           /* i  : Q0  :                                       */
    const Word16 gqlevs,        /* i  : Q0  : quantized level                       */
    const Word16 gqbits         /* i  : Q0  : quantized bits                        */
)
{
    Word16 i, k;

    Word16 delta_fx, Qdelta;
    Word32 L_delta;
    Word32 L_q;

    Word16 gain_table_fx[MAX_GQLEVS];
    Word16 Qgt;
    Word16 gamma_fx;
    Word16 Qgamma;

    Word16 exp_safe;
    Word16 exp_normn, exp_normd;
    Word16 exp_norm;

    Word32 L_temp;
    Word16 temp_lo_fx, temp_hi_fx, temp_fx, temp2_fx;

    Word32 L_Eyy, L_Exy;
    /*Word16 QE; */

    Word16 d_fx;
    Word16 dmin_fx;
    Word16 imin_fx;

    /* Fine gain quantization on only the most significant energy bands */

    /*delta = qint / gqlevs; */
    exp_normn = norm_l(L_qint);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(gqlevs);
    delta_fx = div_l(L_shl(L_qint, exp_normn), shl(gqlevs, exp_normd));
    Qdelta = add(sub(exp_normn, exp_normd), 28); /* 29+exp_normn-(exp_normd)-1; */
    L_delta = L_shl(L_deposit_h(delta_fx), sub(13, Qdelta));
    /*q = (-qint + delta) / 2.0f; */
    L_q = L_shr(L_sub(L_delta, L_qint), 1);

    FOR (i = 0; i < gqlevs; i++)
    {
        /*gain_table[i] = (float) pow (2.0f, q * 0.5f); */
        L_temp = L_shr(L_shr(L_q, 1), sub(29, 16));
        temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
        Qgt = sub(14, temp_hi_fx);
        gain_table_fx[i] = extract_l(Pow2(14, temp_lo_fx));        /* Qgt */

        /*q += delta; */
        L_q = L_add(L_q, L_delta);
        gain_table_fx[i] = shl(gain_table_fx[i], sub(14, Qgt)); /* Qgt -> Q14 */
    }

    FOR (k = sub(bands, Ngq); k < bands; k++)
    {
        /*Eyy = 0.0f; */
        /*Exy = 0.0f; */
        /*for (i = band_start[k_sort[k]]; i <= band_end[k_sort[k]]; i++) */
        /*{ */
        /*    Eyy += y2[i] * y2[i]; */
        /*    Exy += ybuf[i] * y2[i]; */
        /*} */
        exp_safe = 4;
        move16();  /* 4 is too large. but never happen overflow */
        L_temp = L_deposit_l(0);
        FOR (i = band_start[k_sort[k]]; i <= band_end[k_sort[k]]; i++)
        {
            L_temp = L_or(L_temp, L_abs(L_y2[i]));
            L_temp = L_or(L_temp, L_abs(L_ybuf[i]));
        }
        exp_norm = norm_l(L_temp);
        exp_norm = sub(exp_norm, exp_safe); /* safe_shift */

        L_Eyy = L_deposit_l(0);
        L_Exy = L_deposit_l(0);
        /*QE = add(shl(add(Qs-16, exp_norm), 1), 1); //(Qs+exp_norm-16)*2+1; */
        FOR (i = band_start[k_sort[k]]; i <= band_end[k_sort[k]]; i++)
        {
            /*Eyy += y2[i] * y2[i]; */
            temp_fx = round_fx(L_shl(L_y2[i], exp_norm));
            L_Eyy = L_mac(L_Eyy, temp_fx, temp_fx);

            /*Exy += ybuf[i] * y2[i]; */
            temp2_fx = round_fx(L_shl(L_ybuf[i], exp_norm));
            L_Exy = L_mac(L_Exy, temp2_fx, temp_fx);
        }

        test();
        IF ( L_Eyy > 0x0L && L_Exy > 0x0L )
        {
            /*gamma = Exy / Eyy; */
            exp_normn = norm_l(L_Exy);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_l(L_Eyy);
            gamma_fx = div_l(L_shl(L_Exy, exp_normn), round_fx(L_shl(L_Eyy, exp_normd)));
            Qgamma = add(sub(exp_normn, exp_normd), 15); /* exp_normn - (exp_normd-16) - 1; */
            gamma_fx = shl(gamma_fx, sub(14, Qgamma)); /* Qgamma -> Q14 */

            dmin_fx = 32767;
            move16();
            imin_fx = -1;
            move16();
            FOR (i = 0; i < gqlevs; i++)
            {
                d_fx = abs_s (sub(gamma_fx, gain_table_fx[i]));
                IF ( sub(d_fx, dmin_fx) < 0 )
                {
                    dmin_fx = d_fx;
                    move16();
                    imin_fx = i;
                    move16();
                }
            }

            gamma_fx = gain_table_fx[imin_fx];
            move16(); /* Q14 */

            FOR (i = band_start[k_sort[k]]; i <= band_end[k_sort[k]]; i++)
            {
                /*y2[i] *= gamma; */
                L_y2[i] = L_shl(Mpy_32_16_1(L_y2[i], gamma_fx), 1); /* Q12+Q14-15+Qx; -> Q12 */
            }
        }
        ELSE
        {
            imin_fx = 0;
            move16();
        }

        push_indice_fx( st_fx, IND_HQ2_SUBBAND_GAIN, imin_fx, gqbits );
    }

    return;
}
