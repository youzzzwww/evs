/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "cnst_fx.h"
#include "rom_dec_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"                /* required for wmc_tool */
#include "basop_mpy.h"

/*--------------------------------------------------------------------------*
 * Local functions
 *--------------------------------------------------------------------------*/

static  Word16 p2a_threshold_dequant_fx( Decoder_State_fx *st_fx, Word16 *p2a_flags, const Word16 bands, const Word16 p2a_bands );

static  void mdct_spectrum_fine_gain_dec_fx( Decoder_State_fx *st_fx, Word32 L_y2[], const Word16 band_start[], const Word16 band_end[],
        const Word16 k_sort[], const Word16 bands,
        const Word32 L_qint,const Word16 Ngq, const Word16 gqlevs, const Word16 gqbits );

static  Word16 band_energy_dequant_fx( Decoder_State_fx *st_fx, Word32 L_band_energy[], const Word16 bands,
                                       const Word32 L_qint,const Word16 eref_fx, const Word16 is_transient_fx );

static Word16 Calc_inv(Word32 L_tmp, Word16 *exp)
{
    Word16 exp2, tmp;

    tmp = extract_h(L_tmp);

    IF(tmp != 0)
    {
        exp2 = norm_s(tmp);
        tmp = shl(tmp,exp2);/*Q(exp) */
        tmp = div_s(16384,tmp);/*Q(15+14-exp=29-exp) */
        *exp = sub(29,exp2);
        move16();
    }
    ELSE
    {
        tmp = 0x7fff;
        move16();
        *exp = 0;
        move16();
    }
    return tmp;
}

/*--------------------------------------------------------------------------*
 * spt_shorten_domain_set_dec()
 *
 * update the shorten band information based on p2a analysis
 *--------------------------------------------------------------------------*/

static  void spt_shorten_domain_set_dec_fx(
    Decoder_State_fx *st_fx,              /* i:   encoder state structure             */
    const Word16 p2a_flags[],         /* i:   p2a anlysis information             */
    const Word16 new_band_start[],    /* i:   new band start position             */
    const Word16 new_band_end[],      /* i:   new band end position               */
    const Word16 new_band_width[],    /* i:   new subband band width              */
    const Word16 bands,               /* i:   total number of subbands            */
    Word16 band_start[],        /* o:   band start position                 */
    Word16 band_end[],          /* o:   band end position                   */
    Word16 band_width[],        /* o:   sub band band width                 */
    Word16 *bit_budget          /* i/o: bit budget                          */
)
{
    Word16 j,k;
    Word16 kpos;
    Word16 spt_shorten_flag[SPT_SHORTEN_SBNUM];

    kpos = sub(NI_USE_PREV_SPT_SBNUM, SPT_SHORTEN_SBNUM);
    j = 0;
    move16();
    FOR( k = sub(bands,SPT_SHORTEN_SBNUM); k < bands; k++ )
    {
        spt_shorten_flag[j] = 0;
        move16();
        IF( sub(p2a_flags[k], 1) == 0)
        {
            spt_shorten_flag[j] = get_next_indice_fx (st_fx, 1 );
            *bit_budget = sub(*bit_budget, 1);
            test();
            IF( sub(spt_shorten_flag[j], 1) == 0 && st_fx->prev_SWB_peak_pos_fx[kpos] != 0)
            {
                band_start[k] = new_band_start[j];
                move16();
                band_end[k]   = new_band_end[j];
                move16();
                band_width[k] = new_band_width[j];
                move16();
            }
        }

        kpos = add(kpos, 1);
        j = add(j, 1);
    }

    return;
}

/*-------------------------------------------------------------------*
 * hq_lr_dec_fx()
 *
 * HQ low rate decoding routine
 *-------------------------------------------------------------------*/

void hq_lr_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o:     : decoder state structure         */
    Word32 L_yout[],          /* o  : Q12 : transform-domain output coefs.  */
    const Word16 inner_frame,       /* i  : Q0  : inner frame length              */
    Word16 num_bits,          /* i  : Q0  : number of available bits        */
    Word16 *is_transient_fx   /* o  : Q0  : transient flag                  */
)
{
    Word16 i, j, k;

    Word32 L_y2[L_FRAME48k];
    Word32 L_y2_ni[L_FRAME48k];
    Word32 L_y2_org[L_FRAME48k];
    Word16 inp_vector_fx[L_FRAME48k];
    Word16 flag_spt_fx;
    Word32 L_m[L_FRAME48k];
    Word32 L_band_energy[BANDS_MAX];
    Word32 L_band_energy_tmp[BANDS_MAX];

    Word16 npulses_fx[BANDS_MAX];
    Word16 lowlength_fx, highlength_fx, hqswb_clas_fx, har_bands_fx, bands_fx;
    Word16 p2a_flags_fx[BANDS_MAX];
    Word32 L_bwe_br;
    Word16 prev_SWB_peak_pos_tmp_fx[NI_USE_PREV_SPT_SBNUM];

    Word16 band_start[BANDS_MAX], band_end[BANDS_MAX], band_width[BANDS_MAX],trans_bit_fx;

    /* hq2_core_configure */
    Word32 L_qint;
    /*Word16 Qqint=29;*/

    Word16 eref_fx/*, Qeref=10*/;
    Word16 bit_alloc_weight_fx/*, Qbaw=13*/;
    Word16 ld_slope_fx/*, Qldslope=15*/;
    Word16 p2a_th_fx/*, Qp2ath=11*/;
    Word16 pd_thresh_fx/*, Qpdth=15*/;
    Word16 ni_coef_fx/*, Qnicoef=14*/;

    Word32 L_Rk[BANDS_MAX];
    Word16 bit_budget_fx;

    Word16 ni_seed_fx;
    Word16 length_fx, length1_fx, length2_fx, length3_fx;
    Word16 pbits_fx;

    Word16 k1_fx, k2_fx;
    Word16 gqlevs_fx, gqbits_fx, Ngq_fx, p2a_bands_fx;
    Word16 ebits_fx;
    Word16 exp_norm;
    Word16 org_band_start[SPT_SHORTEN_SBNUM];
    Word16 org_band_end[SPT_SHORTEN_SBNUM];
    Word16 org_band_width[SPT_SHORTEN_SBNUM];

    Word16 new_band_start[SPT_SHORTEN_SBNUM];
    Word16 new_band_end[SPT_SHORTEN_SBNUM];
    Word16 new_band_width[SPT_SHORTEN_SBNUM];

    Word16 k_sort_fx[BANDS_MAX];
    Word16 last_bitalloc_max_band[BANDS_MAX];
    Word32 L_tmp;
    Word16 lowband,highband,p2a_flags_tmp[BANDS_MAX];
    Word32 L_tmp2,L_tmp3;
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
    Word16 adjustFlag;
    Word16 bw_low, bw_high;

    tmp2 = 0;       /* to avoid compilation flags */

    set16_fx(last_bitalloc_max_band, 0, BANDS_MAX);
    set32_fx( L_y2, 0x0L, L_FRAME48k );
    set16_fx( inp_vector_fx, 0, inner_frame );
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

    L_bwe_br = L_add(st_fx->core_brate_fx, 0);
    hqswb_clas_fx = 0;
    move16();
    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && ( L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) )
    {
        hqswb_clas_fx = get_next_indice_fx(st_fx, 2);
        num_bits = sub(num_bits, 2);

        *is_transient_fx = 0;
        move16();
        if ( sub(hqswb_clas_fx, HQ_TRANSIENT) == 0 )
        {
            *is_transient_fx = 1;
            move16();
        }
    }
    ELSE
    {
        /* decode transient flag */
        *is_transient_fx = get_next_indice_fx(st_fx, 1);
        num_bits = sub(num_bits, 1);
    }

    /* Configure decoder for different bandwidths, bit rates, etc. */
    hq2_core_configure_fx( inner_frame, num_bits, *is_transient_fx, &bands_fx, &length_fx, band_width, band_start, band_end,
                           &L_qint, &eref_fx, &bit_alloc_weight_fx, &gqlevs_fx, &Ngq_fx, &p2a_bands_fx, &p2a_th_fx, &pd_thresh_fx, &ld_slope_fx, &ni_coef_fx
                           ,L_bwe_br);

    highlength_fx = band_end[bands_fx-1];
    move16();
    har_bands_fx = bands_fx;
    move16();

    test();
    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && *is_transient_fx == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        /* reserve bits for HQ_NORMAL2 and HQ_HARMONIC modes */
        test();
        IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 || sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
        {
            num_bits = sub(num_bits, get_usebit_npswb_fx(hqswb_clas_fx));
        }
        if( sub(hqswb_clas_fx, HQ_NORMAL) == 0)
        {
            flag_spt_fx = 1;
            move16();
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

    /* Spectral energy calculation/quantization */
    ebits_fx = band_energy_dequant_fx( st_fx, L_band_energy, bands_fx, L_qint, eref_fx, *is_transient_fx );

    /* First pass bit budget for TCQ of spectral band information */
    exp_norm = norm_s(gqlevs_fx);
    gqbits_fx = sub(14, exp_norm);

    bit_budget_fx = sub(sub(num_bits, ebits_fx), round_fx(L_shl(L_mult(Ngq_fx, gqbits_fx), 15))); /* (*num_bits) - (short) ceil (ebits) - Ngq * gqbits; */


    pbits_fx = 0;
    move16();
    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) )
    {
        IF ( sub(hqswb_clas_fx, HQ_HARMONIC) == 0 )
        {
            set16_fx( p2a_flags_fx, 1, har_bands_fx );
        }
        ELSE
        {
            pbits_fx = p2a_threshold_dequant_fx( st_fx, p2a_flags_fx, bands_fx, p2a_bands_fx );
            bit_budget_fx = sub(bit_budget_fx, pbits_fx);

            IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
            {
                return_bits_normal2_fx( &bit_budget_fx, p2a_flags_fx, bands_fx, bits_lagIndices_fx );
            }
        }
    }
    ELSE
    {
        pbits_fx = p2a_threshold_dequant_fx( st_fx, p2a_flags_fx, bands_fx, p2a_bands_fx );
        bit_budget_fx = sub(bit_budget_fx, pbits_fx);
    }

    IF( sub(flag_spt_fx, 1) == 0 )
    {
        /* initalize the desired parameters for SPT */
        spt_shorten_domain_band_save_fx(bands_fx, band_start, band_end, band_width, org_band_start, org_band_end, org_band_width);
        spt_shorten_domain_pre_fx(band_start, band_end, st_fx->prev_SWB_peak_pos_fx, bands_fx, L_bwe_br, new_band_start, new_band_end, new_band_width);
        spt_shorten_domain_set_dec_fx(st_fx, p2a_flags_fx, new_band_start, new_band_end, new_band_width, bands_fx, band_start, band_end, band_width, &bit_budget_fx);
    }
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
        Ep_tmp_fx[i] = L_shr(L_tmp,sub(15,exp2));/*Q13 */ move32();
    }

    test();
    test();
    test();
    test();
    test();
    test();
    IF ( *is_transient_fx == 0 && sub(inner_frame, L_FRAME8k) == 0 && L_sub(st_fx->core_brate_fx, ACELP_13k20) <= 0 )
    {
        /* decode the last p2a_bands-1 subbands bit-allocation index of the previous frame */
        FOR(i = 1; i <= 2; i++)
        {
            last_bitalloc_max_band[bands_fx-i] = get_next_indice_fx( st_fx, 1 );
        }
        lowband = 6;
        move16();
        trans_bit_fx = 2;
        move16();
        bit_budget_fx = sub(bit_budget_fx,trans_bit_fx);
        gama_fx = 27852;
        move16(); /*Q15 0.85f; */
        beta_fx = 17203;
        move16(); /*Q14 1.05f; */

        set16_fx(&p2a_flags_tmp[sub(bands_fx,trans_bit_fx)], 0, 2);
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
            Copy(&p2a_flags_fx[sub(bands_fx,trans_bit_fx)], &p2a_flags_tmp[sub(bands_fx,trans_bit_fx)], trans_bit_fx);
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
                if(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
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
                    tmp = Calc_inv(L_shl(L_tmp,14), &exp);
                    L_tmp = L_shl(Mult_32_16(Ep_avrg_fx,tmp),sub(13,exp));/*Q(13+exp-15 +13-exp +4 = 15) */
                    L_tmp2 = L_add(L_tmp,13107); /*15 */
                    tmp2 = extract_l(L_min(L_max(L_tmp2,16384),gama_fx)); /*15 = 15 */
                    L_band_energy_tmp[i] = Mult_32_16(L_band_energy_tmp[i],tmp2);/*Q(Q_band_energy+15-15 = Q_band_energy) */  move32();
                }
            }
        }
        ELSE
        {
            FOR(i = sub(bands_fx,trans_bit_fx); i < bands_fx; i++)
            {
                alpha_fx = 16384;
                move16();/*Q14 */
                IF( sub(p2a_flags_tmp[i],1) == 0)
                {
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],sub(bands_fx,lowband));/*Q(13+0-15  = -2) */
                    tmp = Calc_inv(L_shl(L_tmp,14), &exp);
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
                IF(sub(last_bitalloc_max_band[i],1) == 0)
                {
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],sub(bands_fx,lowband));/*Q(13+0-15 = -2) */
                    tmp = Calc_inv(L_shl(L_tmp,14), &exp);
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
                L_band_energy_tmp[i] = L_shl(Mult_32_16(L_band_energy_tmp[i],alpha_fx),1);/*Q(Q_band_energy+14-15 +1= Q_band_energy) */ move32();
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
                if(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
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
                tmp = Calc_inv(Ep_avrgL_fx, &exp);
                L_tmp = Mult_32_16(Ep_peak_fx,tmp);/*Q(13+exp-15+4 = exp+2) */
                L_tmp = Mult_32_16(L_tmp,lowband);/*Q(exp+2+0-15 = exp-13) */
                L_tmp = Mult_32_16(L_tmp,18842);/*Q(exp-13+16-16 = exp-13) */
                L_tmp = L_shl(L_tmp,sub(27,exp));/*Q14 0.5 */
                tmp2=extract_l(L_min(L_tmp,19661));/*14 */
                L_tmp = Mult_32_16(L_band_energy_tmp[i],tmp2);/*Q(Q_band_energy+14-15 = Q_band_energy-1) */
                L_band_energy_tmp[i] = L_shl(L_tmp,1); /*Q_band_energy  */  move32();
            }
        }
        hq2_bit_alloc_fx(
            L_band_energy_tmp, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
            num_bits, hqswb_clas_fx, st_fx->bwidth_fx, *is_transient_fx );
    }
    ELSE IF( *is_transient_fx == 0 && sub(inner_frame, L_FRAME16k) == 0 )
    {
        bit_budget_fx = sub(bit_budget_fx,2);/* bits in high bands to indicate the last 2 subbands is allocated bits or not */

        FOR( i = 1; i <= 2; i++ )
        {
            last_bitalloc_max_band[bands_fx-i] = get_next_indice_fx( st_fx,1);
            move16();
        }
        FOR( i = 0; i < bands_fx; i++ )
        {
            Ep_tmp_fx[i] = L_shl(Ep_tmp_fx[i],2);
            move32();
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
                IF(sub(last_bitalloc_max_band[i],1) == 0)
                {
                    tmp = sub(bands_fx,p2a_bands_fx);
                    tmp = sub(tmp,lowband);
                    L_tmp = Mult_32_16(Ep_tmp_fx[i],tmp);/*Q(15+0-15 = 0) */
                    tmp = Calc_inv(L_shl(L_tmp,16), &exp);
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
                if(L_sub(Ep_tmp_fx[i],Ep_peak_fx) > 0)
                {
                    Ep_peak_fx = L_add(Ep_tmp_fx[i], 0); /*Q15 */
                }
            }
        }


        L_tmp = Mult_32_16(Ep_peak_fx,24576);/*Q(15+13-15 = 13) lowband = 6; */
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
                tmp = Calc_inv(L_shl(Ep_avrgL_fx,1), &exp);
                L_tmp = Mult_32_16(Ep_peak_fx,tmp);/*Q(15+exp-15 = exp) */
                L_tmp = Mult_32_16(L_tmp,lowband);/*Q(exp+0-15 = exp-15) */
                L_tmp = L_shl(L_tmp,sub(28,exp));/*Q14 0.5 */
                tmp = extract_l(L_min(L_tmp,19661));/*//Q14 */
                L_tmp = Mult_32_16(L_band_energy_tmp[i],tmp);/*Q(Q_band_energy+14-15 = Q_band_energy-1) */
                L_band_energy_tmp[i] = L_shl(L_tmp,1); /*Q_band_energy  */  move32();
            }
        }

        hq2_bit_alloc_fx(
            L_band_energy_tmp, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
            num_bits, hqswb_clas_fx, st_fx->bwidth_fx, *is_transient_fx );
    }
    ELSE IF( sub(st_fx->bwidth_fx, SWB) == 0 && sub(hqswb_clas_fx, HQ_HARMONIC) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) )
    {
        hq2_bit_alloc_har_fx( L_band_energy, bit_budget_fx, bands_fx, L_Rk, p2a_bands_fx, L_bwe_br, p2a_flags_fx, band_width );
    }
    ELSE
    {
        hq2_bit_alloc_fx(
            L_band_energy, bands_fx, L_Rk, &bit_budget_fx, p2a_flags_fx, bit_alloc_weight_fx, band_width,
            num_bits, hqswb_clas_fx, st_fx->bwidth_fx, *is_transient_fx );
    }

    tcq_core_LR_dec_fx( st_fx, /*inp_vector*/inp_vector_fx, bit_budget_fx, bands_fx, band_start, band_width, /*Rk, */L_Rk, npulses_fx, k_sort_fx,
                        p2a_flags_fx, p2a_bands_fx, last_bitalloc_max_band, inner_frame, adjustFlag, is_transient_fx );

    /* Denormalize the coded MDCT spectrum */
    mdct_spectrum_denorm_fx( inp_vector_fx, L_y2, band_start, band_end, band_width, L_band_energy, npulses_fx, bands_fx, ld_slope_fx, pd_thresh_fx );

    /* Apply fine gain to denormalized coded spectrum */
    mdct_spectrum_fine_gain_dec_fx( st_fx, L_y2, band_start, band_end, k_sort_fx, bands_fx,
                                    L_qint, Ngq_fx, gqlevs_fx, gqbits_fx );

    test();
    test();
    test();
    /* Restore the band information */
    IF( sub(flag_spt_fx, 1) == 0 )
    {
        spt_shorten_domain_band_restore_fx(bands_fx, band_start, band_end, band_width, org_band_start, org_band_end, org_band_width);
    }

    Copy32( L_y2, L_y2_org, L_FRAME32k );

    /* Inject noise into components having relatively low pulse energy per band */
    ni_seed_fx = add(add(add(npulses_fx[0], shl(npulses_fx[1], 4)), shl(npulses_fx[2], 8)), shl(npulses_fx[3], 12));
    Copy32( L_y2, L_y2_ni, band_end[bands_fx-1]+1 );

    hq2_noise_inject_fx( L_y2_ni, band_start, band_end, band_width, Ep_fx, L_Rk, npulses_fx, ni_seed_fx, bands_fx, 0, bw_low, bw_high, enerL_fx, enerH_fx,
                         st_fx->last_ni_gain_fx, st_fx->last_env_fx, &st_fx->last_max_pos_pulse_fx, p2a_flags_fx, p2a_bands_fx,
                         hqswb_clas_fx, st_fx->bwidth_fx, L_bwe_br );

    test();
    test();
    IF( sub(st_fx->bwidth_fx, SWB) == 0 && (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0 ) )
    {
        test();
        IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 || sub(hqswb_clas_fx, HQ_HARMONIC) == 0)
        {
            preset_hq2_swb_fx( hqswb_clas_fx, band_end, &har_bands_fx, p2a_bands_fx,length_fx, bands_fx, &lowlength_fx, &highlength_fx, L_m );

            swb_bwe_dec_lr_fx(
                st_fx,
                L_y2, SWB_BWE_LR_Qs, L_m,
                L_bwe_br,
                bands_fx, band_start, band_end,
                L_band_energy, SWB_BWE_LR_Qbe,
                p2a_flags_fx, hqswb_clas_fx, lowlength_fx, highlength_fx, har_bands_fx,
                &st_fx->prev_frm_hfe2_fx, &st_fx->prev_stab_hfe2_fx
                ,  band_width, L_y2_ni,  &ni_seed_fx
            );

            post_hq2_swb_fx( L_m, lowlength_fx, highlength_fx, hqswb_clas_fx, har_bands_fx, bands_fx, p2a_flags_fx, band_start, band_end, L_y2, npulses_fx );

            IF( sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
            {
                spt_swb_peakpos_tmp_save_fx(L_y2, bands_fx, band_start, band_end, prev_SWB_peak_pos_tmp_fx);
            }
            Copy32( L_y2_ni, L_y2, lowlength_fx );
        }
        ELSE
        {
            Copy32( L_y2_ni, L_y2, add(band_end[bands_fx-1],1) ); /* HQ_TRANSIENT */
        }
    }
    ELSE
    {
        Copy32( L_y2_ni, L_y2, add(band_end[bands_fx-1],1) ); /* NB, WB */
    }

    test();
    IF( !(sub(st_fx->last_inner_frame_fx, L_FRAME16k) >= 0 && st_fx->bws_cnt_fx > 0 ))
    {
        k1_fx =	sub(bands_fx,2);
        if(sub(*is_transient_fx,1) != 0)
        {
            k1_fx =	sub(bands_fx,6);
        }
        L_tmp = L_deposit_l(0);
        FOR(i = k1_fx; i < bands_fx; i++)
        {
            tmp = div_s(1,sub(bands_fx,k1_fx));/*Q15 */
            L_tmp = L_add(L_tmp,Mult_32_16(Ep_tmp_fx[i],tmp));/*Q15 */
        }
        st_fx->prev_ener_shb_fx = extract_l(L_shr(L_tmp, 14));
        IF(L_sub(Ep_tmp_fx[sub(bands_fx,1)],Ep_tmp_fx[k1_fx]) >= 0)
        {
            st_fx->last_hq_tilt_fx = 26214;/*Q15 */
        }
        ELSE
        {
            exp = norm_l(Ep_tmp_fx[k1_fx]);
            L_tmp = L_shl(Ep_tmp_fx[k1_fx],exp);/*Q(15+exp) */
            tmp = extract_h(L_tmp);/*Q(15+exp-16=exp-1) */
            tmp =div_s(16384,tmp);/*Q(14+15-exp+1)=Q(30-exp) */
            L_tmp = Mult_32_16(Ep_tmp_fx[sub(bands_fx,1)],tmp);/*Q(15+30-exp-15)=Q(30-exp) */
            tmp = extract_h(L_shl(L_tmp,add(exp,1)));/*Q15 */
            st_fx->last_hq_tilt_fx = s_min(tmp,26214);
        }
    }
    IF( sub(st_fx->last_inner_frame_fx,L_FRAME32k) >= 0 )
    {
        set16_fx(st_fx->prev_SWB_fenv_fx, st_fx->prev_ener_shb_fx ,SWB_FENV);
    }

    /* Copy the coded MDCT coefficient to the output buffer */
    IF ( !(*is_transient_fx) )
    {
        /* Copy the scratch buffer to the output */
        Copy32( L_y2, L_yout, length_fx );

        /* If the input frame is larger than coded bandwidth, zero out uncoded MDCT coefficients */
        IF ( sub(inner_frame, length_fx) > 0 )
        {
            set32_fx( L_yout + length_fx, 0x0L, sub(inner_frame, length_fx) );
        }
    }
    ELSE /* transient frame */
    {
        test();
        IF( sub(inner_frame, length_fx) == 0 || st_fx->bws_cnt_fx > 0 )
        {
            /* Copy the scratch buffer to the output */
            Copy32( L_y2, L_yout, length_fx );
        }
        ELSE
        {
            /* length/NUM_TIME_SWITCHING_BLOCKS */
            length1_fx = shr(length_fx, 2); /* length/NUM_TIME_SWITCHING_BLOCKS */
            /* inner_frame/NUM_TIME_SWITCHING_BLOCKS */
            length2_fx = shr(inner_frame, 2); /* inner_frame/NUM_TIME_SWITCHING_BLOCKS */
            /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */
            length3_fx = shr(sub(inner_frame, length_fx), 2); /* (inner_frame-length)/NUM_TIME_SWITCHING_BLOCKS */

            k1_fx = 0;
            move16();
            k2_fx = 0;
            move16();

            /* un-collapse transient frame and interleave zeros */
            FOR( i = 0; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                /*k1 = i*length/NUM_TIME_SWITCHING_BLOCKS; */
                /*k2 = i*inner_frame/NUM_TIME_SWITCHING_BLOCKS; */

                Copy32( L_y2 + k1_fx, L_yout + k2_fx, length1_fx );
                set32_fx( L_yout + k2_fx + length1_fx, 0x0L, length3_fx);

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
        st_fx->prev_hqswb_clas_fx = *is_transient_fx;
        move16();
    }

    test();
    test();
    test();
    IF( (L_sub(L_bwe_br, HQ_16k40) == 0 || L_sub(L_bwe_br, HQ_13k20) == 0) && sub(st_fx->bwidth_fx, SWB) == 0 && sub(hqswb_clas_fx, HQ_NORMAL) == 0 )
    {
        j = 0;
        move16();
        FOR(k=sub(bands_fx,NI_USE_PREV_SPT_SBNUM); k<bands_fx; k++)
        {
            st_fx->prev_SWB_peak_pos_fx[j] = prev_SWB_peak_pos_tmp_fx[j];
            move16();
            j = add(j, 1);
        }
    }

    return;
}

/*------------------------------------------------------------------------------------
 * small_symbol_dec_tran_fx()
 *
 * Huffman decoding of differential energies
 *--------------------------------------------------------------------------------------*/

static Word16 small_symbol_dec_tran_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure   */
    Word16  *qbidx,           /* o  : output of dequantized differential energy */
    const Word16  bands,            /* i  : number of bands */
    const Word16  is_transient      /* i  : transient flag  */
)
{
    Word16 i,  bits;
    Word16 difidx[BANDS_MAX];

    /* Decoding differential energies*/
    bits = decode_envelope_indices_fx(st_fx, 0, bands, 0, difidx, LOW_RATE_HQ_CORE_TRAN ,is_transient);
    bits = add(bits, BITS_DE_FCOMP);

    /* counting 1 bit for band_energy_huff_coding_mode */
    bits = add(bits, BITS_DE_HMODE);

    /* converting to original values */
    FOR( i=0; i<bands; i++ )
    {
        qbidx[i] = sub(difidx[i], LRMDCT_BE_OFFSET);
        move16();
    }

    return( bits );
}


/*--------------------------------------------------------------------------
 * small_symbol_dec()
 *
 * Huffman decoding of differential energies (MSB and LSB)
 *--------------------------------------------------------------------------*/

static Word16 small_symbol_dec_fx( /* o  : bits                                        */
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure   */
    Word16 *qbidx,            /* o  : output of dequantized differential energy   */
    const Word16 bands,             /* i  : number of bands                             */
    const Word16 is_transient
)
{
    Word16 i, LSB, bits;
    Word16 difidx[BANDS_MAX];

    /* Decoding MSB bits */
    bits = decode_envelope_indices_fx( st_fx, 0, bands, 0, difidx, LOW_RATE_HQ_CORE, is_transient );
    bits = add(bits, BITS_DE_FCOMP);

    /* counting 1 bit for band_energy_huff_coding_mode */
    bits = add(bits ,BITS_DE_HMODE);

    /* Decoding LSB bit packing */
    FOR( i=0; i<bands; ++i )
    {
        LSB = get_next_indice_fx( st_fx, BITS_DE_LSB );
        difidx[i] = s_or(shl(difidx[i],1), LSB);
        move16();
    }

    /* counting bits for LSB */
    bits = add(bits, bands);

    /* converting to original values */
    FOR( i=1; i<bands; ++i )
    {
        qbidx[i] = sub(difidx[i], DE_OFFSET1);
        move16();
    }

    qbidx[0] = sub(difidx[0], DE_OFFSET0);
    move16();

    return( bits );
}


static Word16 decode_huff_8s_fx(
    Decoder_State_fx *st_fx,
    const Word16 *hufftab,
    Word16 *rbits
)
{
    Word16 bit;

    WHILE( *hufftab > 0 )
    {
        *rbits = add(*rbits, s_and(*hufftab, 0xf));
        bit = get_next_indice_fx( st_fx, s_and(*hufftab, 0xf) );
        hufftab += add(shr(*hufftab, 4), bit);
    }

    return negate(*hufftab);
}

static Word16 large_symbol_dec_fx( /* o  : bits                                        */
    Decoder_State_fx *st_fx,                    /* i/o: decoder state structure                     */
    Word16 *qbidx,                    /* o  : output of dequantized differential energy   */
    const Word16 bands                      /* i  : number of bands                             */
)
{
    Word16 i, bits;
    Word16 LSB[BANDS_MAX];
    Word16 basic_shift,cntbits,ns2mode;
    Word16 pos_outlyer;
    Word16 ns2mode0,ns2mode1;

    cntbits = BITS_DE_8SMODE;
    move16();
    ns2mode = get_next_indice_fx (st_fx, BITS_DE_8SMODE);

    IF ( ns2mode == 0 )
    {
        ns2mode0 = get_next_indice_fx (st_fx, BITS_DE_8SMODE_N0);
        ns2mode1 = get_next_indice_fx (st_fx, BITS_DE_8SMODE_N1);
        cntbits = add(cntbits, BITS_DE_8SMODE_N0+BITS_DE_8SMODE_N1);

        IF ( ns2mode0 == 0 )
        {
            IF ( sub(ns2mode1, 1) == 0 )
            {
                pos_outlyer = get_next_indice_fx (st_fx, BITS_DE_8SPOS);
                cntbits = add(cntbits, BITS_DE_8SPOS);
                qbidx[pos_outlyer] = sub(get_next_indice_fx (st_fx, BITS_ABS_ENG), ABS_ENG_OFFSET);
                move16();
                cntbits = add(cntbits, BITS_ABS_ENG);
            }
            ELSE
            {
                pos_outlyer = -1;
                move16();
            }

            FOR( i=0; i<pos_outlyer; ++i )
            {
                bits = 0;
                move16();
                qbidx[i] = sub(decode_huff_8s_fx(st_fx, hestable_fx, &bits), 4);
                move16();
                cntbits = add(cntbits, bits);
            }

            FOR( i=add(pos_outlyer, 1); i<bands; ++i )
            {
                bits = 0;
                move16();
                qbidx[i] = sub(decode_huff_8s_fx(st_fx, hestable_fx, &bits), 4);
                move16();
                cntbits = add(cntbits, bits);
            }
        }
        ELSE
        {
            IF ( sub(ns2mode1, 1) == 0 )
            {
                pos_outlyer = get_next_indice_fx (st_fx, BITS_DE_8SPOS);
                cntbits = add(cntbits, BITS_DE_8SPOS);
                qbidx[0] = sub(get_next_indice_fx (st_fx, BITS_ABS_ENG), ABS_ENG_OFFSET);
                move16();
                cntbits = add(cntbits, BITS_ABS_ENG);
                qbidx[pos_outlyer] = sub(get_next_indice_fx (st_fx, BITS_ABS_ENG), ABS_ENG_OFFSET);
                move16();
                cntbits = add(cntbits, BITS_ABS_ENG);
            }
            ELSE
            {
                pos_outlyer = 0;
                move16();
                qbidx[0] = sub(get_next_indice_fx (st_fx, BITS_ABS_ENG), ABS_ENG_OFFSET);
                move16();
                cntbits = add(cntbits, BITS_ABS_ENG);
            }

            FOR( i=1; i<pos_outlyer; ++i )
            {
                bits = 0;
                move16();
                qbidx[i] = sub(decode_huff_8s_fx(st_fx, hestable_fx, &bits), 4);
                move16();
                cntbits = add(cntbits, bits);
            }

            FOR( i=pos_outlyer+1; i<bands; ++i )
            {
                bits = 0;
                move16();
                qbidx[i] = sub(decode_huff_8s_fx(st_fx, hestable_fx, &bits), 4);
                move16();
                cntbits = add(cntbits, bits);
            }
        }
    }
    ELSE
    {
        basic_shift = get_next_indice_fx (st_fx, BITS_MAX_DEPTH);
        cntbits = add(cntbits, BITS_MAX_DEPTH);

        FOR(i=0; i<bands; ++i)
        {
            bits = 0;
            move16();
            qbidx[i] = sub(decode_huff_8s_fx(st_fx, hestable_fx, &bits), 4);
            qbidx[i] = shl(qbidx[i], basic_shift);
            move16();
            cntbits = add(cntbits, bits);
        }

        FOR(i=0; i<bands; ++i)
        {
            LSB[0] = get_next_indice_fx (st_fx, basic_shift);
            move16();
            qbidx[i] = add(qbidx[i], LSB[0]);
            move16();
            cntbits = add(cntbits, basic_shift);
        }
    }

    return cntbits;  /* xx bits for diff. energies + 1 bit for LC coding mode */
}


/*--------------------------------------------------------------------------*
 * band_energy_dequant()
 *
 *
 *--------------------------------------------------------------------------*/

static  Word16 band_energy_dequant_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure   */
    Word32 L_band_energy[],  /* o  : Q14 band energy           */
    const Word16 bands_fx,
    const Word32 L_qint,
    const Word16 eref_fx,
    const Word16 is_transient_fx
)
{
    Word16 k;
    Word16 deng_cmode;
    Word16 deng_bits;

    Word16 rev_qint_fx;
    Word16 Qrev_qint;

    Word16 bq0_fx;
    Word16 bq1_fx[BANDS_MAX];
    Word16 bq2_fx[BANDS_MAX];

    Word16 exp_normd;


    /* parsing energy difference coding mode */
    deng_cmode = get_next_indice_fx( st_fx, BITS_DE_CMODE );

    IF ( deng_cmode == 0 )
    {
        deng_bits = large_symbol_dec_fx( st_fx, bq2_fx, bands_fx );

        /* counting 1 bit for deng coding mode */
        deng_bits = add(deng_bits, BITS_DE_CMODE);
    }
    ELSE
    {
        IF(is_transient_fx)
        {
            deng_bits = small_symbol_dec_tran_fx(st_fx, bq2_fx, bands_fx , is_transient_fx);
        }
        ELSE
        {
            deng_bits = small_symbol_dec_fx( st_fx, bq2_fx, bands_fx, is_transient_fx );
        }

        /* counting 1 bit for deng coding mode */
        deng_bits = add(deng_bits, BITS_DE_CMODE);
    }

    exp_normd = norm_l(L_qint);
    rev_qint_fx = div_s(0x4000, round_fx(L_shl(L_qint, exp_normd))); /* Q14-(29+exp_normd-16)+15 */
    Qrev_qint = sub(14-(29-16)+15, exp_normd);

    bq0_fx = round_fx(L_shl(L_mult(eref_fx, rev_qint_fx), sub(5, Qrev_qint))); /* 16-(10+Qrev_qint+1) */

    bq1_fx[0] = add(bq2_fx[0], bq0_fx);
    move16();
    FOR (k = 1; k < bands_fx; k++)
    {
        bq1_fx[k] = add(bq2_fx[k], bq1_fx[k-1]);
        move16();
    }

    FOR (k = 0; k < bands_fx; k++)
    {
        L_band_energy[k] = Mpy_32_16_1(L_qint, bq1_fx[k]);
        move32();/* 29+0-15 -> Qbe(Q14) */
    }

    IF (is_transient_fx)
    {
        reverse_transient_frame_energies_fx( L_band_energy, bands_fx );
    }

    return( deng_bits );
}


/*--------------------------------------------------------------------------*
 * p2a_threshold_dequant()
 *
 *
 *--------------------------------------------------------------------------*/

static  Word16 p2a_threshold_dequant_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                   */
    Word16 *p2a_flags,      /* o  : tonaly indicator                          */
    const Word16 bands,           /* i  : number of subbands                        */
    const Word16 p2a_bands        /* i  : number of subbnads for computing tonality */
)
{
    Word16 j, k;

    j = sub(bands, p2a_bands);
    FOR( k = 0; k < j; k++ )
    {
        p2a_flags[k] = 1;
        move16();
    }

    j = 0;
    move16();
    FOR( k = sub(bands, p2a_bands); k < bands; k++ )
    {
        p2a_flags[k] = get_next_indice_fx( st_fx, 1 );
        move16();
        j = add(j, 1);
    }

    return( j );
}


/*--------------------------------------------------------------------------*
 * mdct_spectrum_fine_gain_dec()
 *
 *
 *--------------------------------------------------------------------------*/

static  void mdct_spectrum_fine_gain_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure                     */
    Word32 L_y2[],        /* i/o: Q12 : decoded spectrum                      */
    const Word16 band_start[],  /* i  : Q0  : table of start freq for every subband */
    const Word16 band_end[],    /* i  : Q0  : table of end freq for every subband   */
    const Word16 k_sort[],      /* i  : Q0  : sort table by band_energy             */
    const Word16 bands,         /* i  : Q0  : nubmber of subbands                   */
    const Word32 L_qint,        /* i  : Q0  :                                       */
    const Word16 Ngq,           /* i  : Q0  :                                       */
    const Word16 gqlevs,        /* i  : Q0  : quantized level                       */
    const Word16 gqbits         /* i  : Q0  : quantized bits                        */
)
{
    Word16 i, k, imin_fx;

    Word16 delta_fx, Qdelta;
    Word32 L_delta;
    Word32 L_q;

    Word16 gain_table_fx[MAX_GQLEVS];
    Word16 Qgt;
    Word16 gamma_fx; /* Q14 */

    Word16 exp_normn, exp_normd;

    Word32 L_temp;
    Word16 temp_lo_fx, temp_hi_fx;

    /* Fine gain quantization on only the most significant energy bands */

    exp_normn = norm_l(L_qint);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(gqlevs);
    delta_fx = div_l(L_shl(L_qint, exp_normn), shl(gqlevs, exp_normd));
    Qdelta = add(sub(exp_normn, exp_normd), 28); /* 29+exp_normn-(exp_normd)-1; */
    L_delta = L_shl(L_deposit_h(delta_fx), sub(13, Qdelta));

    L_q = L_shr(L_sub(L_delta, L_qint), 1);

    FOR ( i=0; i<gqlevs; i++)
    {
        /*gain_table[i] = (float) pow (2.0f, q * 0.5f); */
        L_temp = L_shr(L_shr(L_q, 1), sub(29, 16));
        temp_lo_fx = L_Extract_lc(L_temp, &temp_hi_fx);
        Qgt = sub(14, temp_hi_fx);
        gain_table_fx[i] = extract_l(Pow2(14, temp_lo_fx));        /* Qgt */

        /*q += delta; */
        L_q = L_add(L_q, L_delta);
        gain_table_fx[i] = shl(gain_table_fx[i], sub(14, Qgt)); /* Qgt -> Q14 */  move16();
    }

    FOR( k = sub(bands, Ngq); k < bands; k++ )
    {
        imin_fx = get_next_indice_fx( st_fx, gqbits );

        /*gamma = gain_table[imin]; */
        gamma_fx = gain_table_fx[imin_fx];
        move16();

        FOR (i = band_start[k_sort[k]]; i <= band_end[k_sort[k]]; i++)
        {
            L_y2[i] = L_shl(Mpy_32_16_1(L_y2[i], gamma_fx), 1); /* Q12+Q14-15+Qx; -> Q12 */ move32();
        }
    }

    return;
}
