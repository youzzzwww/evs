/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "prot_fx.h"
#include "stl.h"        /* required for wmc_tool */

#define MIN_BITS_FIX 0   /* QRk=18 */
#define HQ_16k40_BIT  (HQ_16k40/50) /* 16400/50=328 */
#define Qbf           14                  /* Q value for bits_fact */
#define C1_QRk        (1<<SWB_BWE_LR_QRk) /* 1 */
#define C1_Qbf        (1<<Qbf)            /* 1 */
#define BITS_FACT_1p10 18022 /* (Word16)(1.10f*(float)pow(2, Qbf)+0.5f) */
#define BITS_FACT_1p05 17203 /* (Word16)(1.05f*(float)pow(2, Qbf)+0.5f) */
#define BITS_FACT_1p00 16384 /* (Word16)(1.00f*(float)pow(2, Qbf)+0.5f) */
#define BITS_FACT_0p97 15892 /* (Word16)(0.97f*(float)pow(2, Qbf)+0.5f) */
#define BITS_FACT_0p92 15073 /* (Word16)(0.92f*(float)pow(2, Qbf)+0.5f) */

/*-------------------------------------------------------------------*
 * Bits2indvsb()
 *
 * Bit allocation to individual SB's in a group
 *-------------------------------------------------------------------*/

void Bits2indvsb_fx (
    const Word32 *L_be,                 /* i  : Qbe Band Energy of sub-band                       */
    const Word16 start_band,            /* i  : Q0  start band indices                            */
    const Word16 end_band,              /* i  : Q0  end band indices                              */
    const Word16 Bits,                  /* i  : Q0  Total number of bits allocated to a group     */
    const Word32 L_Bits_needed,         /* i  : QRk smallest bit number for allocation in group   */
    Word32 *L_Rsubband,           /* o  : QRk bit allocation of sub-band                    */
    Word16 *p2aflags_fx           /* i/o: Q0  peaky/noise subband flag                      */
)
{
    Word16 i,j,k;
    Word32 L_R_temp[14]; /* QRk = QL_Rsubband; */
    Word16 Ravg_fx;
    Word16 QRavg;

    const Word32 *L_y_ptr;
    Word32 *L_R_ptr;

    Word16 Bits_avg_fx;
    Word16 QBavg;
    Word16 scale_fact_fx;

    Word16 band_num_fx;
    Word16 index_fx[14];

    Word16 y_index_fx[14];

    Word16 be_sum_fx; /* Q0 */

    Word16 exp_normn, exp_normd;
    Word16 enr_diffcnt_fx;
    Word16 th_5_fx;
    Word16 Rcnt_fx;

    Word16 be_cnt_fx;
    Word16 *p2aflags_fx_ptr;

    Word32 L_temp1;
    Word32 L_temp2;

    band_num_fx = sub(end_band, start_band);
    L_y_ptr = L_be + start_band;
    L_R_ptr = L_Rsubband + start_band;
    p2aflags_fx_ptr = p2aflags_fx+start_band;

    FOR ( i = 0; i < band_num_fx; i++ )
    {
        y_index_fx[i] = extract_h(L_shr(L_y_ptr[i], sub(SWB_BWE_LR_Qbe,16)));
        index_fx[i] = i;
        move16();
    }


    /* Rearrange norm vector in decreasing order */
    reordvct_fx(y_index_fx, band_num_fx, index_fx);

    be_sum_fx = 0;
    move16();
    be_cnt_fx = 0;
    move16();
    FOR( j=0; j<band_num_fx; j++ )
    {
        test();
        IF ( y_index_fx[j] <= 0 || p2aflags_fx_ptr[index_fx[j]] == 0 )
        {
            y_index_fx[j] = 0;
            move16();
            L_R_temp[j] = L_deposit_l(0);
        }
        ELSE
        {
            L_R_temp[j] = C1_QRk;
            move32(); /* filled not zero value */
            be_cnt_fx = add(be_cnt_fx, 1);
        }
    }

    i = sub(be_cnt_fx, 1);
    FOR(k = 0; k <=i ; k++)
    {
        if( L_R_temp[k] > 0 )
        {
            be_sum_fx = add(be_sum_fx, y_index_fx[k]);
        }
    }
    QBavg = 0;
    move16();

    /*Ravg = (float) be_sum/be_cnt;*/
    Ravg_fx = 0;
    move16();
    QRavg = 0;
    move16();
    IF( be_cnt_fx != 0x0 )
    {
        exp_normn = norm_s(be_sum_fx);
        exp_normn = sub(exp_normn, 1);
        exp_normd = norm_s(be_cnt_fx);
        Ravg_fx = div_s(shl(be_sum_fx, exp_normn), shl(be_cnt_fx, exp_normd));

        Ravg_fx = shr(Ravg_fx, 2); /* safe shift */
        QRavg = add(sub(exp_normn, exp_normd), 15-2);
    }

    enr_diffcnt_fx = 0;
    move16();
    th_5_fx = shl(5, QRavg);
    FOR (j = 0; j < be_cnt_fx; j++)
    {
        if( sub(abs_s(sub(Ravg_fx, shl(y_index_fx[j], QRavg))), th_5_fx) > 0 )
        {
            enr_diffcnt_fx = add(enr_diffcnt_fx, 1);
        }
    }

    scale_fact_fx = 19661;
    move16(); /* 0.60f 19660.8(Q15) */
    if( enr_diffcnt_fx > 0 )
    {
        scale_fact_fx = 11468;
        move16(); /* 0.35f 11468.8(Q15) */
    }

    /* Bits allocation to individual SB's in a group based on Band Energies */
    FOR (j = 0; j < be_cnt_fx; j++)
    {
        Rcnt_fx = add(i, 1);

        /* Ravg = (float) be_sum/Rcnt; */
        exp_normn = norm_s(be_sum_fx);
        exp_normn = sub(exp_normn, 1);
        exp_normd = norm_s(Rcnt_fx);
        Ravg_fx = div_s(shl(be_sum_fx, exp_normn), shl(Rcnt_fx, exp_normd));
        Ravg_fx = shr(Ravg_fx, 2); /* safe shift */
        QRavg = add(sub(exp_normn, exp_normd), 15-2);

        if(be_sum_fx <= 0)
        {
            be_sum_fx = 1;
            move16();
        }

        /* Bits_avg = (float) Bits/(be_sum+EPSILON); */
        Bits_avg_fx = 0;
        move16();
        QBavg = 0;
        move16();
        IF ( Bits != 0 )
        {
            exp_normn = norm_s(Bits);
            exp_normn = sub(exp_normn, 1);
            exp_normd = norm_s(be_sum_fx);
            Bits_avg_fx = div_s(shl(Bits, exp_normn), shl(be_sum_fx, exp_normd));
            Bits_avg_fx = shr(Bits_avg_fx, 2); /* safe_shift */
            QBavg = add(sub(exp_normn, exp_normd), 15-2);
        }
        FOR (k = 0; k <=i; k++)
        {
            IF(L_R_temp[k] > 0) /* Rtemp -> SWB_BWE_LR_QRk */
            {
                /* Allocate more bits to SB, if SB bandenergy is higher than average energy */
                /* R_temp[k] = (float)( Bits_avg * y_index[k]+( scale_fact * (y_index[k] - Ravg))); */
                L_temp1 = L_mult(Bits_avg_fx, y_index_fx[k]);                              /* QBavg+1 */
                L_temp2 = L_mult(scale_fact_fx, sub(shl(y_index_fx[k], QRavg), Ravg_fx));  /* 15+QRavg+1 */
                L_R_temp[k] = L_add(L_shr(L_temp1, sub(add(QBavg, 1), SWB_BWE_LR_QRk)), L_shr(L_temp2, sub(add(QRavg, 16), SWB_BWE_LR_QRk))); /* SWB_BWE_LR_QRk */
            }
        }
        IF (  L_sub(L_R_temp[i], L_Bits_needed) < 0 )
        {
            L_R_temp[i] = L_deposit_l(0);

            p2aflags_fx_ptr[index_fx[i]] = 0;
            move16();

            /* be_sum -= y_index[i]; */
            be_sum_fx = sub(be_sum_fx, y_index_fx[i]);

            i = sub(i, 1);
        }
        ELSE
        {
            BREAK;
        }
    }

    /* Rearrange the bit allocation to align with original */
    FOR ( k = 0 ; k < band_num_fx; k++ )
    {
        j = index_fx[k];
        move16();
        L_R_ptr[j] = L_R_temp[k];
        move32();
    }

    return;
}

/*-------------------------------------------------------------------*
 * hq2_bit_alloc_har()
 *
 * Bit allocation mechanism for HQ_HARMONIC mode
 *-------------------------------------------------------------------*/

void hq2_bit_alloc_har_fx (
    const Word32 *L_y,              /* i  : Qbe band energy of sub-vectors               */
    Word16 B_fx,              /* i  : Q0  number of available bits                 */
    const Word16 N_fx,              /* i  : Q0  number of sub-vectors                    */
    Word32 *L_Rsubband,       /* o  : QRk sub-band bit-allocation vector           */
    Word16 p2a_bands_fx,      /* i  : highfreq bands                               */
    const Word32 L_core_brate,      /* i  : Q0  core bit rate                            */
    Word16 p2a_flags_fx[],    /* i/o: Q0  p2a_flags                                */
    const Word16 band_width_fx[]    /* i  : Q0  table of band_width                      */
)
{
    Word16 i, j, k;

    Word32 L_norm_sum;                     /* Qbe */
    Word32 L_Ravg_sub[GRP_SB];             /* Qbe */
    Word32 L_temp_band_energy[BANDS_MAX];  /* Qbe */

    Word16 j_fx, k_fx, Bits_grp_fx[GRP_SB];

    Word32 L_temp_band_energydiff[BANDS_MAX];
    Word16 G1_BE_DIFF_POS_fx; /* Q0 */
    Word32 L_G1_BE_DIFF_VAL;  /* Qbe  Word32 */
    Word16 final_gr_fact_pos_fx, gmax_range_fx[2], temp_fx;
    Word16 bits_fact_fx, bits_fact1_fx;   /* Q? */
    Word16 grp_rngmax_fx[2] = {0};
    Word16 index_fx[NB_SWB_SUBBANDS_HAR], y_index_fx[NB_SWB_SUBBANDS_HAR], esthf_bits_fx, grp_bit_avg_fx, harmonic_band_fx;
    Word32 L_norm_sum_avg;
    Word32 L_norm_diff; /* Qbe */
    Word16 bits_allocweigh_fx; /* Q15 */
    Word16 grp_bound_fx[5];
    Word32 L_grp_thr[GRP_SB]; /* not require Word32 precission */
    Word16 lf_hf_ge_r_fx; /* Q15 */
    Word32 L_avg_enhf_en_diff; /* Qbe */

    Word16 B_norm_fx;

    Word32 L_temp, L_temp2;
    Word16 exp, frac;

    Word32 L_THR1, L_THR2, L_THR3;

    Word16 exp_norm;
    Word16 norm_sum_fx;
    Word16 Qns; /* Q value for norm_sum_fx */
    Word16 Inv_norm_sum_fx; /* 1/norm_sum */
    Word16 QIns; /* Q value for Inv_norm_sum_fx */

    Word16 exp_normn, exp_normd;
    Word16 div_fx;

    Word16 Inv_p2a_bands_fx;
    Word16 QIpb;

    Word16 exp_shift;

    L_THR1 = L_shl(L_deposit_l(THR1), SWB_BWE_LR_QRk);
    L_THR2 = L_shl(L_deposit_l(THR2), SWB_BWE_LR_QRk);
    L_THR3 = L_shl(L_deposit_l(THR3), SWB_BWE_LR_QRk);

    set16_fx(Bits_grp_fx, 0, GRP_SB);

    /* Initialize subbands bits allocation vector based on harmonic bands */
    harmonic_band_fx = add(sub(N_fx, p2a_bands_fx), 1);
    /*printf("harmonic_band= %d %d\n", harmonic_band, harmonic_band_fx);*/
    FOR (k = 0; k < N_fx; k++)
    {
        L_Rsubband[k] = (Word32)(C1_QRk);
        move32(); /* Constant Value */
        L_temp_band_energy[k] = L_y[k];
        move32();   /* SWB_BWE_LR_Qbe */
    }
    final_gr_fact_pos_fx = 2;
    move16();
    bits_fact_fx =  C1_Qbf;
    move16();
    bits_fact1_fx = C1_Qbf;
    move16();

    gmax_range_fx[0]= G1_RANGE;
    move16();
    gmax_range_fx[1]= G1G2_RANGE;
    move16();

    IF( L_sub(L_core_brate, HQ_16k40) == 0 )
    {
        gmax_range_fx[1] = add(gmax_range_fx[1], 2);
        move16();
    }

    /* decide each group range, for grouping spectral coefficients */
    grp_rngmax_fx[1] = 16;
    move16();
    grp_rngmax_fx[0] =  7;
    move16();
    temp_fx = 0;
    move16();
    FOR( i=0; i<2; i++ )
    {
        j_fx = gmax_range_fx[i];
        move16();
        k_fx = 0;
        move16();
        WHILE( L_sub(L_temp_band_energy[gmax_range_fx[i]-1], L_temp_band_energy[j_fx] ) >= 0x0L && sub(j_fx, grp_rngmax_fx[i]) < 0x0 )
        {
            test();
            k_fx = add(k_fx, 1);
            j_fx = add(j_fx, 1);
        }

        temp_fx = k_fx;
        move16();
        IF( sub(temp_fx, 1) > 0 )
        {
            FOR( temp_fx = 2; temp_fx <= k_fx ; )
            {
                IF( L_sub(L_temp_band_energy[gmax_range_fx[i]+temp_fx-1], L_temp_band_energy[gmax_range_fx[i]+temp_fx]) < 0 )
                {
                    BREAK;
                }
                ELSE IF( L_sub(L_temp_band_energy[gmax_range_fx[i]+temp_fx-1], L_temp_band_energy[gmax_range_fx[i]+temp_fx]) >= 0 )
                {
                    temp_fx = add(temp_fx, 1);;
                    IF( sub(temp_fx, k_fx) > 0 )
                    {
                        temp_fx = sub(temp_fx, 1);
                        BREAK;
                    }
                }
            }

            gmax_range_fx[i] = add(gmax_range_fx[i], temp_fx);
            move16();
        }
        ELSE
        {
            gmax_range_fx[i] = add(gmax_range_fx[i], temp_fx);
            move16();
        }
    }

    grp_bound_fx[0] = 0;
    move16();
    FOR(i=1; i<GRP_SB-1; i++)
    {
        grp_bound_fx[i] = gmax_range_fx[i-1];
        move16();
    }
    grp_bound_fx[i] = harmonic_band_fx;
    move16();
    grp_bound_fx[i+1] = N_fx;
    move16();


    FOR(i=0; i<GRP_SB; i++)
    {
        L_Ravg_sub[i] = L_deposit_l(0);
        FOR ( j = grp_bound_fx[i]; j < grp_bound_fx[i+1]; j++ )
        {
            IF ( L_temp_band_energy[j] > 0x0L )
            {
                L_Ravg_sub[i] = L_add(L_Ravg_sub[i], L_temp_band_energy[j]);
                move32();
            }
        }
    }

    L_temp_band_energydiff[0] = L_temp_band_energy[0];
    move32();
    FOR ( j = 1; j < harmonic_band_fx; j++ )
    {
        L_temp_band_energydiff[j]= L_abs(L_sub(L_temp_band_energy[j], L_temp_band_energy[j-1]));
        move32();
    }

    G1_BE_DIFF_POS_fx = 0;
    move16();
    L_G1_BE_DIFF_VAL = L_deposit_l(0);

    FOR(j=1; j< harmonic_band_fx; j++)
    {
        IF( L_sub(L_temp_band_energydiff[j], L_G1_BE_DIFF_VAL) > 0 )
        {
            G1_BE_DIFF_POS_fx = j;
            move16();
            L_G1_BE_DIFF_VAL = L_add(0,L_temp_band_energydiff[j]);
        }
    }

    test();
    test();
    IF( sub(G1_BE_DIFF_POS_fx, gmax_range_fx[0] ) < 0 && G1_BE_DIFF_POS_fx > 0 )
    {
        final_gr_fact_pos_fx = 0;
        move16();
    }
    ELSE IF ( sub(G1_BE_DIFF_POS_fx, gmax_range_fx[0]) >= 0 && sub(G1_BE_DIFF_POS_fx, gmax_range_fx[1] ) < 0 )
    {
        final_gr_fact_pos_fx = 1;
        move16();
    }
    ELSE
    {
        final_gr_fact_pos_fx = 2;
        move16();
    }

    test();
    IF( final_gr_fact_pos_fx == 0 || sub(final_gr_fact_pos_fx, 1) == 0 )
    {
        IF( L_sub(L_core_brate, HQ_16k40 ) == 0 )
        {
            bits_fact_fx  = BITS_FACT_1p10;
            move16(); /* 1.10f; */ /* G1 */
            bits_fact1_fx = BITS_FACT_0p92;
            move16(); /* 0.92f; */ /* G3 */
        }
        ELSE
        {
            bits_fact_fx  = BITS_FACT_1p05;
            move16(); /* 1.05f; */ /* G1 */
            bits_fact1_fx = BITS_FACT_0p97;
            move16(); /* 0.97f; */ /* G3 */
        }
    }
    ELSE
    {
        IF( L_sub(L_core_brate, HQ_16k40) == 0 )
        {
            bits_fact_fx  = BITS_FACT_0p97;
            move16(); /* 0.97f; */ /* G1 */
            bits_fact1_fx = BITS_FACT_1p00;
            move16(); /* 1.00f; */ /* G3 */
        }
        ELSE
        {
            bits_fact_fx  = BITS_FACT_0p92;
            move16(); /* 0.92f; */ /* G1 */
            bits_fact1_fx = BITS_FACT_1p00;
            move16(); /* 1.00f; */ /* G3 */
        }
    }

    j = sub(N_fx, harmonic_band_fx);
    FOR ( i = 0; i < j; i++ )
    {
        y_index_fx[i] = extract_h(L_shl(L_temp_band_energy[harmonic_band_fx+i], sub(16, SWB_BWE_LR_Qbe)));
        index_fx[i] = add(harmonic_band_fx, i);
        move16();
    }

    reordvct_fx(y_index_fx, sub(N_fx, harmonic_band_fx), index_fx);

    /* Log2 */
    L_temp = L_deposit_l(band_width_fx[index_fx[0]]);
    exp = norm_l(L_temp);
    frac = Log2_norm_lc(L_shl(L_temp, exp));
    exp = sub(30, exp);
    L_temp = L_Comp(exp, frac);
    /* ceil */
    if( L_and(0x0000ffff, L_temp) > 0 )
    {
        L_temp = L_add(L_temp, 0x00010000);
    }
    esthf_bits_fx = extract_h(L_temp);

    L_grp_thr[0] = L_THR1;
    move32();
    L_grp_thr[1] = L_THR2;
    move32();
    L_grp_thr[2] = L_THR3;
    move32();
    L_grp_thr[3] = L_shl(L_deposit_l(esthf_bits_fx), SWB_BWE_LR_QRk);
    move16();

    L_norm_sum = L_deposit_l(1);
    FOR(i=0; i<3; i++)
    {
        L_norm_sum = L_add( L_norm_sum, L_Ravg_sub[i]);
    }

    /*reserve bits for HF coding */
    L_temp = L_add(L_norm_sum, L_Ravg_sub[GRP_SB-1]);
    exp_normn = norm_l(L_temp);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(N_fx);

    div_fx = div_l(L_shl(L_temp, exp_normn), shl(N_fx, exp_normd)); /* (Qbe+exp_normn)-(0+exp_normd)-1) */
    L_norm_sum_avg = L_shr(L_deposit_h(div_fx), add(sub(exp_normn, exp_normd), 15)); /* -> Qbe */

    exp_norm = norm_l(L_norm_sum);
    norm_sum_fx = extract_h( L_shl(L_norm_sum, exp_norm) ); /* SWB_BWE_LR_Qbe+exp_norm-16 */
    Qns = sub(add(SWB_BWE_LR_Qbe, exp_norm), 16);

    Inv_norm_sum_fx = div_s( 0x4000 /* Q15 */ , norm_sum_fx );
    QIns = sub(31, exp_norm); /* 14 - (14+exp_norm-16) + 15 */

    grp_bit_avg_fx = div_s_ss(B_fx, GRP_SB); /* Q0 */

    exp_normd = norm_s(p2a_bands_fx);
    Inv_p2a_bands_fx = div_s(0x3fff, shl(p2a_bands_fx, exp_normd)); /* 14-exp_normd+15 */
    QIpb = sub(29, exp_normd);

    L_temp = L_shl(Mult_32_16(L_Ravg_sub[GRP_SB-1], Inv_p2a_bands_fx), sub(SWB_BWE_LR_Qbe, sub(QIpb,1)));
    L_norm_diff = L_sub(L_temp, L_norm_sum_avg); /* Qbe */

    L_temp = Mult_32_16(L_Ravg_sub[GRP_SB-1], sub(GRP_SB, 1)); /* Qbe+0+1 */
    L_temp = Mult_32_16(L_temp, Inv_norm_sum_fx);             /* Qbe+1+QIpb+1 */
    lf_hf_ge_r_fx = round_fx(L_shl(L_temp, sub(15+16, sub(add(SWB_BWE_LR_Qbe, QIns),30))));

    exp_normn = norm_s(norm_sum_fx);
    exp_normn = sub(exp_normn, 1);
    exp_normd = norm_s(harmonic_band_fx);

    div_fx = div_s(shl(norm_sum_fx, exp_normn), shl(harmonic_band_fx, exp_normd));
    L_avg_enhf_en_diff = L_sub(L_temp_band_energy[index_fx[0]], L_shl(L_deposit_h(div_fx), sub(sub(SWB_BWE_LR_Qbe, (add(Qns,sub(exp_normn,exp_normd)))),31))); /* Qbe - (Qns+exp_normn-(exp_normd)+15) -16 */

    test();
    IF( sub(lf_hf_ge_r_fx , 26214) > 0x0 && L_sub(L_avg_enhf_en_diff, (Word32)(8<<SWB_BWE_LR_Qbe)) > 0x0L) /* 0.8=26214.4(Q15) 8.0f=131072(Qbe) */
    {
        bits_allocweigh_fx = 6554;
        move16(); /* 0.2 6553.6(Q15) */
        if(L_norm_diff < 0x0L)
        {
            bits_allocweigh_fx = 13107;
            move16();  /* 0.4 13107.2(Q15) */
        }

        /*allocate bits*/
        /*Bits_grp[GRP_SB-1] = (short)min((grp_bit_avg/p2a_bands + bits_allocweigh*norm_diff),10);*/
        L_temp = L_mult(grp_bit_avg_fx, Inv_p2a_bands_fx);    /* Q0+QIpb+1 */
        L_temp2 = Mult_32_16(L_norm_diff, bits_allocweigh_fx); /* Qbe+Q15-15 */

        L_temp = L_shr(L_temp, add(QIpb, 1));
        L_temp = L_add(L_shl(L_temp,SWB_BWE_LR_Qbe), L_temp2);

        Bits_grp_fx[GRP_SB-1] = extract_h(L_shl(L_temp, sub(16, SWB_BWE_LR_Qbe)));
        Bits_grp_fx[GRP_SB-1] = s_min(Bits_grp_fx[GRP_SB-1], 10);
        move16();

        if( sub(Bits_grp_fx[GRP_SB-1], esthf_bits_fx) < 0 )
        {
            Bits_grp_fx[GRP_SB-1] = 0;
            move16();
        }
        B_fx = sub(B_fx, Bits_grp_fx[GRP_SB-1]);
    }

    exp_shift = sub(add(SWB_BWE_LR_Qbe, QIns), 47); /* (SWB_BWE_LR_Qbe+14+1+QIns-15-16) */
    exp_norm = norm_s(B_fx);
    B_norm_fx = shl(B_fx, exp_norm);
    exp_shift = add(exp_shift, exp_norm);

    IF( sub(final_gr_fact_pos_fx, 1) == 0 )
    {
        L_temp = Mult_32_16(L_Ravg_sub[1], extract_h(L_mult(bits_fact_fx, B_norm_fx)));
        L_temp = Mult_32_16(L_temp, Inv_norm_sum_fx);
        Bits_grp_fx[1] = extract_h(L_shr(L_temp, exp_shift));

        L_temp = Mult_32_16(L_Ravg_sub[2], extract_h(L_mult(bits_fact1_fx, B_norm_fx)));
        L_temp = Mult_32_16(L_temp, Inv_norm_sum_fx);
        Bits_grp_fx[2] = extract_h(L_shr(L_temp, exp_shift));

        Bits_grp_fx[0] = sub(sub(B_fx, Bits_grp_fx[1]), Bits_grp_fx[2]);
        move16();
    }
    ELSE
    {
        L_temp = Mult_32_16(L_Ravg_sub[0], extract_h(L_mult(bits_fact_fx, B_norm_fx)));
        L_temp = Mult_32_16(L_temp, Inv_norm_sum_fx);
        Bits_grp_fx[0] = extract_h(L_shr(L_temp, exp_shift));

        L_temp = Mult_32_16(L_Ravg_sub[2], extract_h(L_mult(bits_fact1_fx, B_norm_fx)));
        L_temp = Mult_32_16(L_temp, Inv_norm_sum_fx);
        Bits_grp_fx[2] = extract_h(L_shr(L_temp, exp_shift));

        Bits_grp_fx[1] = sub(sub(B_fx, Bits_grp_fx[0]), Bits_grp_fx[2]);
        move16();
    }

    IF( sub(Bits_grp_fx[2], THR2 ) < 0 )
    {
        Bits_grp_fx[1] = add(Bits_grp_fx[1], Bits_grp_fx[2]);
        move16();
        Bits_grp_fx[2] = 0;
        move16();
    }

    FOR(i=0; i<GRP_SB; i++)
    {
        IF(Bits_grp_fx[i] > 0)
        {
            Bits2indvsb_fx( L_temp_band_energy, grp_bound_fx[i], grp_bound_fx[i+1] , Bits_grp_fx[i], L_grp_thr[i], L_Rsubband, p2a_flags_fx);
        }
        ELSE
        {
            set32_fx(L_Rsubband+grp_bound_fx[i], 0x0L, sub(grp_bound_fx[i+1], grp_bound_fx[i]));
            IF( sub(i, GRP_SB-1) == 0 )
            {
                set16_fx(p2a_flags_fx+grp_bound_fx[i], 0, sub(grp_bound_fx[i+1], grp_bound_fx[i]));
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * hq2_bit_alloc()
 *
 * HQ2 bit-allocation
 *--------------------------------------------------------------------------*/

Word32 hq2_bit_alloc_fx (
    const Word32 L_band_energy[],           /* i  : band energy of each subband                 */
    const Word16 bands,                     /* i  : total number of subbands in a frame         */
    Word32 L_Rk[],                    /* i/o: Bit allocation/Adjusted bit alloc.          */
    Word16 *bit_budget_fx,            /* i/o: bit bugdet                                  */
    Word16 *p2a_flags,                /* i  : HF tonal indicator                          */
    const Word16 weight_fx,                 /* i  : weight                                      */
    const Word16 band_width[],              /* i  : Sub band bandwidth                          */
    const Word16 num_bits,                  /* i  : available bits                              */
    const Word16 hqswb_clas,                /* i  : HQ2 class information                       */
    const Word16 bwidth,                    /* i  : input bandwidth                             */
    const Word16 is_transient               /* i  : indicator HQ_TRANSIENT or not               */
)
{
    Word16 j, k;
    Word16 tmp;

    Word32 L_Rcalc, L_Ravg, L_Rcalc1;

    Word16 exp_normn, exp_normd;

    Word16 Rcnt_fx;

    Word16 div_fx;
    Word16 Qdiv;

    Word32 L_tmp;
    Word16 tmp_fx;

    Word32 L_maxxy;
    Word16 maxdex_fx;
    Word32 L_dummy;

    Word16 bit_budget_temp_fx;

    Word16 negflag;

    Word32 L_THR1, L_THR2, L_THR3;

    L_THR1 = L_shl(L_deposit_l(THR1), SWB_BWE_LR_QRk);
    L_THR2 = L_shl(L_deposit_l(THR2), SWB_BWE_LR_QRk);
    L_THR3 = L_shl(L_deposit_l(THR3), SWB_BWE_LR_QRk);

    /* Init Rk to non-zero values for bands to be allocated bits */
    IF( sub(num_bits, HQ_16k40_BIT) <= 0 )
    {
        set32_fx( L_Rk, (Word32)(C1_QRk), bands); /* 1<<SWB_BWE_LR_QRk */

        test();
        IF( is_transient && sub(bands, 32) == 0 )
        {
            L_Rk[6] = L_deposit_l(0);
            L_Rk[7] = L_deposit_l(0);
            L_Rk[14] = L_deposit_l(0);
            L_Rk[15] = L_deposit_l(0);
            L_Rk[22] = L_deposit_l(0);
            L_Rk[23] = L_deposit_l(0);
            L_Rk[30] = L_deposit_l(0);
            L_Rk[31] = L_deposit_l(0);
        }
    }
    ELSE
    {
        /*mvs2r( p2a_flags, Rk, bands ); */
        FOR(k=0; k<bands; k++)
        {
            L_Rk[k] = L_shl(L_deposit_l(p2a_flags[k]), SWB_BWE_LR_QRk);
        }
    }

    L_Rcalc = L_deposit_l(0);
    L_Rcalc1 = L_deposit_l(0);

    FOR (j = 0; j < bands; j++)
    {
        Rcnt_fx = 0;
        move16();
        L_Ravg = L_add(0,0x0L);

        FOR (k = 0; k < bands; k++)
        {
            IF ( L_Rk[k] > 0 )
            {
                L_Ravg = L_add(L_Ravg, L_shl(L_band_energy[k], sub(SWB_BWE_LR_QRk, SWB_BWE_LR_Qbe)));  /* SWB_BWE_LR_QRk-SWB_BWE_LR_Qbe */
                Rcnt_fx = add(Rcnt_fx, 1);
            }
        }
        /* Ravg Qband_energy */

        /*L_Ravg /= Rcnt; */
        exp_normd = norm_l(L_Ravg);
        exp_normd = sub(exp_normd, 1);
        exp_normn = norm_s(Rcnt_fx);

        tmp = shl(Rcnt_fx, exp_normn);
        tmp = s_max(tmp,1);
        IF ( L_Ravg > 0 )
        {
            div_fx = div_l(L_shl(L_Ravg, exp_normd), tmp); /* Qdiv = 14+exp_normd-(exp_normn)-1 */
        }
        ELSE
        {
            div_fx = div_l(L_shl(L_abs(L_Ravg), exp_normd), tmp); /* Qdiv = 14+exp_normd-(exp_normn)-1 */
            div_fx = negate(div_fx);
        }

        Qdiv = sub(sub(add(SWB_BWE_LR_QRk, exp_normd), exp_normn), 1);

        L_Ravg = L_shr(L_deposit_l(div_fx), sub(Qdiv, SWB_BWE_LR_QRk));
        FOR (k = 0; k < bands; k++)
        {
            IF ( L_Rk[k] > 0)
            {
                /*Rk[k] = ((float) *bit_budget / Rcnt + weight * (band_energy[k] - Ravg)); */
                exp_normd = norm_s(*bit_budget_fx);
                exp_normd = sub(exp_normd, 1);
                exp_normn = norm_s(Rcnt_fx);
                div_fx = div_s(shl(*bit_budget_fx, exp_normd), shl(Rcnt_fx, exp_normn));
                Qdiv = add(sub(exp_normd, exp_normn), 15);

                L_tmp = Mult_32_16(L_sub(L_shl(L_band_energy[k], sub(SWB_BWE_LR_QRk, SWB_BWE_LR_Qbe)), L_Ravg), weight_fx); /* SWB_BWE_LR_QRk + Q13 - 15 */
                L_tmp = L_shl(L_tmp, 2); /* -> SWB_BWE_LR_QRk */

                L_Rk[k] = L_add(L_shr(L_deposit_l(div_fx), sub(Qdiv, SWB_BWE_LR_QRk)) , L_tmp);
                move32();
            }
        }

        negflag = 0;
        move16();
        L_Rcalc = L_deposit_l(0);
        FOR (k = 0; k < bands; k++)
        {
            IF ( L_sub(L_Rk[k], MIN_BITS_FIX) < 0 )
            {
                L_Rk[k] = L_deposit_l(0);
                negflag = 1;
                move16();
            }
            L_Rcalc = L_add( L_Rcalc , L_Rk[k]); /*SWB_BWE_LR_QRk */
        }

        /* prune noiselike bands with low allocation */
        test();
        IF ( sub(num_bits, HQ_16k40_BIT) <= 0 && negflag == 0)
        {
            L_maxxy = L_deposit_l(0);
            maxdex_fx = -1;
            move16();
            L_Rcalc = L_deposit_l(0);

            /* find worst under-allocation */
            FOR (k = sub(bands, 1); k >= 0; k--)
            {
                tmp_fx = s_min( band_width[k], s_max(12, shr( band_width[k], 2)));
                L_dummy = L_sub(L_shl(L_deposit_l(tmp_fx), SWB_BWE_LR_QRk), L_Rk[k]) ; /*SWB_BWE_LR_QRk */
                test();
                test();
                IF ( p2a_flags[k] == 0 && L_sub(L_dummy, L_maxxy) > 0 && L_Rk[k] > 0 )
                {
                    maxdex_fx = k;
                    move16();
                    L_maxxy = L_add(0,L_dummy); /*SWB_BWE_LR_QRk */
                }
            }

            /* prune worst allocation and recalculate total allocation */
            if ( sub(maxdex_fx, -1) > 0)
            {
                L_Rk[maxdex_fx] = L_deposit_l(0);
            }
            FOR (k = 0; k < bands; k++)
            {
                L_Rcalc = L_add(L_Rcalc, L_Rk[k]); /*SWB_BWE_LR_QRk */
            }
        }
        test();
        test();
        IF ( L_sub(L_Rcalc, L_Rcalc1) == 0 && sub(bwidth, SWB) == 0 )
        {
            /* Reallocate bits to individual subbands for HQ_NORMAL mode */
            /* if bits allocated to subbands areless than predefined threshold */
            test();
            IF( sub(hqswb_clas, HQ_NORMAL) == 0 && sub(num_bits, HQ_16k40_BIT) < 0 )
            {
                L_dummy = L_deposit_l(0);
                FOR( k = 0; k < bands; k++ )
                {
                    test();
                    test();
                    test();
                    test();
                    test();
                    IF( sub(k, 11) < 0 && L_sub(L_Rk[k], L_THR1) < 0 )
                    {
                        L_Rk[k] = L_deposit_l(0);
                    }
                    ELSE IF( sub(k, 11) >= 0 && sub(k, 16) < 0 && L_sub(L_Rk[k], L_THR2) < 0 )
                    {
                        L_Rk[k] = L_deposit_l(0);
                    }
                    ELSE if( sub(k, 16) >= 0 && sub(k, bands ) < 0 && L_sub(L_Rk[k], L_THR3) < 0 )
                    {
                        L_Rk[k] = L_deposit_l(0);
                    }

                    L_dummy = L_add(L_dummy, L_Rk[k]);
                }

                IF( L_sub(L_dummy, L_Rcalc ) == 0 )
                {
                    test();
                    IF( sub(hqswb_clas, HQ_NORMAL) == 0 && sub(num_bits, HQ_16k40_BIT) < 0)
                    {
                        bit_budget_temp_fx = *bit_budget_fx;
                        move16();
                        FOR( k=0; k<NB_SWB_SUBBANDS; k++ )
                        {
                            test();
                            IF( p2a_flags[bands-NB_SWB_SUBBANDS+k] == 1 && L_Rk[bands-NB_SWB_SUBBANDS+k] == 0.0f )
                            {
                                p2a_flags[bands-NB_SWB_SUBBANDS+k] = 0;
                                move16();
                                bit_budget_temp_fx = sub(bit_budget_temp_fx, bits_lagIndices_fx[k]);
                            }
                        }

                        IF(  sub(bit_budget_temp_fx, *bit_budget_fx ) < 0)
                        {
                            *bit_budget_fx = bit_budget_temp_fx;
                            move16();
                        }
                        ELSE IF( sub(bit_budget_temp_fx, *bit_budget_fx ) == 0 )
                        {
                            BREAK;
                        }
                    }
                    ELSE
                    {
                        BREAK;
                    }
                }
            }
            ELSE
            {
                BREAK;
            }
        }
        ELSE IF ( L_sub(L_Rcalc, L_Rcalc1 ) == 0 && sub(bwidth, SWB) != 0)
        {
            BREAK;
        }

        L_Rcalc1 = L_Rcalc;
        move32();

    }

    return L_Rcalc;
}

