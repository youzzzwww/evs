/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Function prototypes                    */



/*--------------------------------------------------------------------------*
 * Local constants
 *--------------------------------------------------------------------------*/

#define NUMSF       8
#define NUMSF_M1    (NUMSF-1)
#define NUMSF_M2    (NUMSF-2)
#define NUMSF_S2    (NUMSF/2)
#define LOG2_NUMSF       3
#define INV_NUMSF (float)0.125

/*--------------------------------------------------------------------------*
 * preecho_sb()
 *
 * Time-domain sub-band based pre-echo reduction
 *--------------------------------------------------------------------------*/

void preecho_sb_fx(
    const Word32  brate,              /* i   Q0  : core bit-rate                                           */
    Word32 *wtda_audio_fx,      /* i   q_sig32 : imdct signal, used to compute imdct_mem_fx when not 24400 bps */
    Word16 q_sig32,             /* i   Q value for wtda_audio_fx */
    Word16 *rec_sig_fx,         /* i   q_sig16  : reconstructed signal, output of the imdct transform     */
    Word16 *imdct_mem_fx,       /* i   q_sig16  : @ 24400 bps, memory of the imdct transform, used in the next frame   */
    Word16 q_sig16,             /* i   Q value for rec_sig_fx and imdct_mem_fx */
    const Word16 framelength,         /* i   Q0  : frame length                                            */
    Word16 *memfilt_lb_fx,      /* i/o Q0  : memory                                                  */
    Word32 *mean_prev_hb_fx,    /* i/o Q0  : memory                                                  */
    Word16 *smoothmem_fx,       /* i/o Q15 : memory                                                  */
    Word32 *mean_prev_fx,       /* i/o Q0  : memory                                                  */
    Word32 *mean_prev_nc_fx,    /* i/o Q0  : memory                                                  */
    Word16 *wmold_hb_fx,        /* i/o Q15 : memory                                                  */
    Word16 *prevflag,           /* i/o Q0  : flag                                                    */
    Word16 *pastpre,            /* i/o Q0  : flag                                                    */
    /* Word32 *InMDCT_fx,*/        /* i   Q0  : input MDCT vector for NBbandwidth detection             */
    /*const Word16 IsTransient,  */     /* i   Q0  : transient flag                                          */
    const Word16 bwidth               /* i   Q0  : bandwidth                                               */
)
{
    Word16 i, j, len3xLp20;
    Word16 zcr[9];                   /* 0..3 (0..7): zero crossing of the 4 (8) subframes, 4..5: (8..10) zero crossing of the future subframes */
    Word16 maxnzcr[8], cntnzcr;      /* max number of samples without zero crossing  */

    Word16 maxind, stind, stind_hb, cnt2, cnt5, adv, advmem;
    Word16 ind2, ind3, ind4, ind5, ind6, pluslim, ind2_m1, ind2_sfl, numsf_ind2;
    Word16 subframelength, subsubframelength;
    Word16 *ptr_fx, *fxptr1, *fxptr2, *fxptr3, *fxptr4, *fxptr5, *fxptr6/*, *fxptr7, *fxptr8*/;
    Word32 *fx32ptr1, *fx32ptr4, *fx32ptr5, *fx32ptr6;
    Word16 *sptr1, *sptr2, sptr1_loc, sptr2_loc;
    /*Word32 ener_1_4_fx, ener_5_8_fx; */
    Word16 /*framelength_s2, framelength_s4, framelength_s16,*/ framelength_m1;
    Word16 limzcr, limmaxnzcr;
    Word16 num_subsubframes, log2_num_subsubframes;
    Word16 nb_flag, smooth_len;
    Word16 firstnzcr;
    Word16 invsmoothlenp1_fx;
    Word16 subframelength_s2, subframelength_s34;
    Word16 tmp_fx1, tmp_fx2, tmp_fx3;
    Word32 tmp_fxL1, tmp_fxL2, tmp_fxL3;
    Word32 es_mdct_fx[9];               /* 0..3 (0..7): energy of the 4 (8) subframes, 4..5: (8..10) energy of the future subframes */
    Word32 es_mdct_hb_fx[9];            /* 0..3 (0..7): energy of the 4 (8) subframes, 4..5: (8..10) energy of the future subframes */
    Word32 es_mdct_half_fx[9];
    Word32 es_mdct_quart_fx[9];
    Word32 savehalfe_fx, last2_fx, maxcrit_fx, sum_plus_es_fx, mean_plus_es_fx[65];
    Word32 savehalfe_hb_fx, last2_hb_fx;
    Word32 plus_es_mdct_fx[64], max_es_fx, max_es_hb_fx, max_plus_es_mdct_fx;
    const Word16 inv_jp2[64] =
    {
        16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2185, 2048, 1928,
        1820, 1725, 1638, 1560, 1489, 1425, 1365, 1311, 1260, 1214, 1170, 1130, 1092, 1057, 1024, 993,
        964, 936, 910, 886, 862, 840, 819, 799, 780, 762, 745, 728, 712, 697, 683, 669,
        655, 643, 630, 618, 607, 596, 585, 575, 565, 555, 546, 537, 529, 520, 512, 504
    };
    Word16 rec_sig_lb_fx[960], rec_sig_hb_fx[960]; /* 960 max frame length at 48 kHz */

    Word16 min_g_fx[13], g_fx, gt_fx[13];
    Word16 min_g_hb_fx[13], gt_hb_fx[13];
    Word16 preechogain_fx[960+PREECHO_SMOOTH_LEN];
    Word16 preechogain_hb_fx[960];
    Word16 pre_g_ch_tab[9];
    Word32 eshbmean2_fx, eshbmean3_fx, sxyhb2_fx, sxylb3_fx;
    Word16 wmold_fx;
    Word16 lim16_fx, lim32_fx;
    Word16 fattnext_fx;
    Word16 oldgain_fx, oldgain_hb_fx;
    UWord16 tmp_u16;
    Word32 mean_prev_hb_fx_loc, mean_prev_nc_fx_loc, mean_prev_fx_loc;      /*                                                  */
    Word16 q16p1, qmemp1, qtmp;

    q16p1 = add(q_sig16, 1);
    qmemp1 = q16p1;

    IF(L_sub(brate, HQ_32k) <= 0)
    {

        mean_prev_fx_loc = L_add(*mean_prev_fx, 0);
        mean_prev_hb_fx_loc = L_add(*mean_prev_hb_fx, 0);
        mean_prev_nc_fx_loc = L_add(*mean_prev_nc_fx, 0);
        framelength_m1 = sub(framelength, 1);
        nb_flag = 0;
        move16();
        if( sub(bwidth, NB) == 0 )
        {
            nb_flag = 1;
            move16();
        }
        limzcr = 16;
        move16();
        smooth_len = 4;
        move16();
        invsmoothlenp1_fx = 6554;
        move16();
        IF( sub(nb_flag, 1) == 0 )
        {
            limzcr = 10;
            move16();
            smooth_len = PREECHO_SMOOTH_LEN;
            move16();
            invsmoothlenp1_fx = INV_PREECHO_SMOOTH_LENP1_FX;
            move16();
        }

        limmaxnzcr = mult(framelength, 1365); /*1/24*/
        num_subsubframes = 8;
        move16();
        log2_num_subsubframes = 3;
        move16();

        IF( sub(framelength, L_FRAME8k) == 0 )
        {
            num_subsubframes = 4;
            move16();
            log2_num_subsubframes = 2;
            move16();
        }

        len3xLp20 = mult_r(framelength, 7168);  /*7*framelength/32;*/
        /*            len3xLp20 = framelength/2-(short)((float)framelength*N_ZERO_MDCT/FRAME_SIZE_MS); in float*/

        fxptr1 = imdct_mem_fx;
        fx32ptr1 = wtda_audio_fx + len3xLp20 - 1;
        FOR( i = 0; i < len3xLp20; i++ )
        {
            *fxptr1++ = negate(extract_h(L_shl(*fx32ptr1--,15-q_sig32)));
            move16(); /*convert to Word16 Q-1 with saturation (saturation not a problem here) */
        }
        FOR( i = 0; i < L_shr(framelength, 1); i++ )
        {
            *fxptr1++ = negate(extract_h(L_shl(wtda_audio_fx[i], 15-q_sig32)));
            move16(); /*convert to Word16 Q-1 with saturation (saturation not a problem here) */
        }
        qmemp1 = 0; /*already in q-1*/

        subframelength = shr(framelength, LOG2_NUMSF);
        subsubframelength = shr(subframelength, log2_num_subsubframes);
        wmold_fx = *smoothmem_fx;
        move16();
        subframelength_s2 = shr(subframelength, 1);
        subframelength_s34 = mult(subframelength, 24576);

        cntnzcr = -1;
        move16();

        lim16_fx = 3277;
        move16();
        lim32_fx = 328;
        move16();
        savehalfe_fx = L_deposit_l(0);
        savehalfe_hb_fx = L_deposit_l(0);

        IF( *pastpre == 0 )
        {
            /* if past frame mean energies are not known (no preecho_sb in the past frame), limit max attenuation to 1*/
            lim16_fx = 32767;
            move16();
            lim32_fx = 32767;
            move16();
        }

        *pastpre = 2;
        move16();
        fxptr1 = rec_sig_lb_fx; /*q_sig16*/
        fxptr2 = rec_sig_fx;
        fxptr3 = rec_sig_fx + 1;
        fxptr4 = rec_sig_fx + 2;

        tmp_fxL1 = L_mult(shl(*memfilt_lb_fx, q_sig16), 8192); /* *memfilt_lb_fx in q0  */
        tmp_fxL1 = L_mac(tmp_fxL1, *fxptr3, 8192);
        *fxptr1 = mac_r(tmp_fxL1, *fxptr2, 16384);
        move16();
        fxptr1++;

        FOR(j = 2; j < framelength; j++)
        {
            tmp_fxL1 = L_mult(*fxptr2, 8192);
            tmp_fxL1 = L_mac(tmp_fxL1, *fxptr4, 8192);
            *fxptr1 = mac_r(tmp_fxL1, *fxptr3, 16384);
            move16();
            fxptr1++;
            fxptr2++;
            fxptr3++;
            fxptr4++;
        }

        tmp_fxL1 = L_mult(*fxptr2, 8192);
        *fxptr1 = mac_r(tmp_fxL1, *fxptr3, 16384);
        move16();
        fxptr1 = rec_sig_lb_fx; /*q_sig16*/
        fxptr2 = rec_sig_fx; /*q_sig16*/
        fxptr3 = rec_sig_hb_fx; /*q_sig16*/

        FOR(j = 0; j < framelength; j++)
        {
            *fxptr3 = sub(*fxptr2, *fxptr1);
            move16();
            fxptr1++;
            fxptr2++;
            fxptr3++;
        }

        fxptr2--;
        *memfilt_lb_fx = shr(*fxptr2, q_sig16);
        move16(); /* *memfilt_lb_fx in q0  */

        /* energy of low bands 8 present and 1 future sub-frames */
        sptr1 = zcr;
        sptr1_loc = 0;
        move16();
        sptr2 = maxnzcr;

        fxptr2 = rec_sig_fx;
        fxptr3 = rec_sig_hb_fx;
        fx32ptr1 = es_mdct_fx;
        fx32ptr5 = es_mdct_half_fx;
        fx32ptr6 = es_mdct_quart_fx;
        fx32ptr4 = es_mdct_hb_fx;
        firstnzcr = 0;
        move16();
        FOR (j = 0; j < NUMSF; j++)                 /* 8 present subframes */
        {
            tmp_fx2 = sub(j, 1);
            tmp_fx1 = shr(*fxptr2, q16p1); /*q-1 to avoisd saturation in energy*/
            tmp_fxL1 = L_mac0(25, tmp_fx1, tmp_fx1);

            tmp_fxL2 = L_mac0(100, *fxptr3, *fxptr3);

            sptr2_loc = 0;
            move16();

            fxptr2++;
            fxptr3++;

            FOR (i = 1; i < subframelength; i++)
            {
                if( sub(i, subframelength_s2) == 0 )
                {
                    *fx32ptr5 = tmp_fxL1;
                    move32();
                }

                if( sub(i, subframelength_s34) == 0 )
                {
                    *fx32ptr6 = tmp_fxL1;
                    move32();
                }
                tmp_fx1 = shr(*fxptr2, q16p1); /*q-1 to avoisd saturation in energy*/
                tmp_fxL1 = L_mac0(tmp_fxL1, tmp_fx1, tmp_fx1);

                tmp_fxL2 = L_mac0(tmp_fxL2, *fxptr3, *fxptr3);
                cntnzcr = add(cntnzcr, 1);
                IF( L_mult0(*fxptr2, *(fxptr2-1)) <= 0 )
                {
                    sptr1_loc = add(sptr1_loc, 1);
                    sptr2_loc = s_max(sptr2_loc, cntnzcr);

                    test();
                    if( (firstnzcr > 0) && (sub(cntnzcr, maxnzcr[tmp_fx2]) > 0) )
                    {
                        maxnzcr[tmp_fx2] = cntnzcr;
                        move16();
                    }

                    firstnzcr = 0;
                    move16();
                    cntnzcr = -1;
                    move16();
                }
                fxptr2++;
                fxptr3++;
            }
            if(sub(j, NUMSF_M1) < 0)
            {
                cntnzcr = add(cntnzcr, 1);
            }
            sptr2_loc = s_max(sptr2_loc, cntnzcr);
            *fx32ptr4 = tmp_fxL2;
            move32();
            fx32ptr4++;
            *sptr1 = sptr1_loc;
            move16();
            *sptr2 = sptr2_loc;
            move16();
            sptr1++;
            sptr2++;

            test();
            if( (firstnzcr > 0) && (sub(cntnzcr, maxnzcr[tmp_fx2]) > 0) )
            {
                maxnzcr[tmp_fx2] = cntnzcr;
                move16();
            }

            sptr1_loc = 0;
            move16();
            test();
            firstnzcr = 1;
            move16();
            IF((sub(j, NUMSF_M1) < 0) && (L_mult0(*fxptr2, *(fxptr2-1)) <= 0)) /* zcr between 2 subframes */
            {
                sptr1_loc = add(sptr1_loc, 1);    /* counts for the nexte subframe */
                cntnzcr = -1;
                move16();
                firstnzcr = 0;
                move16();
            }

            *fx32ptr1 = tmp_fxL1;
            move32();
            if( L_sub(*fx32ptr5, L_shr(*fx32ptr1, 1)) < 0 )
            {
                tmp_fxL1 = L_shl(L_sub(*fx32ptr1, *fx32ptr5),1);
            }
            *fx32ptr5 = tmp_fxL1;
            move32();

            fx32ptr1++;
            fx32ptr5++;
            fx32ptr6++;
        }

        fxptr2 = imdct_mem_fx; /* q_sig16 or q-1*/
        j = NUMSF;
        move16();                                  /* one future subframe but 96 samples (not 80) (enough with ALDO window) */
        tmp_fx1 = shr(*fxptr2, qmemp1); /* q-1 shr to avoid overflow in es_mdct_fx*/
        tmp_fxL1 = L_mac0(25, tmp_fx1, tmp_fx1);

        sptr1_loc = 0;
        move16();
        fxptr2++;
        tmp_fx3 = sub(len3xLp20,1);
        FOR (i = 1; i < len3xLp20; i++)
        {
            tmp_fx1 = shr(*fxptr2, qmemp1); /*q-1 to avoisd saturation in energy*/
            tmp_fxL1 = L_mac0(tmp_fxL1, tmp_fx1, tmp_fx1);

            if(*fxptr2 **(fxptr2-1) <= 0)
            {
                sptr1_loc = add(sptr1_loc,1);
            }

            fxptr2++;
        }
        *fx32ptr1 = tmp_fxL1;
        move32();
        *sptr1 = sptr1_loc;
        fxptr2 = imdct_mem_fx;
        fxptr3 = imdct_mem_fx + 1;
        fxptr4 = imdct_mem_fx + 2;
        tmp_fxL1 = L_mult(rec_sig_fx[framelength_m1], -8192);
        tmp_fxL1 = L_mac(tmp_fxL1, *fxptr3, -8192);
        tmp_fx1 = mac_r(tmp_fxL1, *fxptr2, 16384);

        tmp_fxL2 = L_deposit_l(100);
        tmp_fxL2 = L_mac0(tmp_fxL2, tmp_fx1, tmp_fx1);

        FOR(j = 1; j < tmp_fx3; j++)   /* tmp_fx3 still contains subframelength*1.2-1 */
        {
            tmp_fxL1 = L_mult(*fxptr2, -8192);
            tmp_fxL1 = L_mac(tmp_fxL1, *fxptr4, -8192);
            tmp_fx1 = mac_r(tmp_fxL1, *fxptr3, 16384);

            tmp_fxL2 = L_mac0(tmp_fxL2, tmp_fx1, tmp_fx1);
            fxptr2++;
            fxptr3++;
            fxptr4++;
        }

        tmp_fxL1 = L_mult(*fxptr2, -8192);
        tmp_fx1 = mac_r(tmp_fxL1, *fxptr3, 16384);
        es_mdct_hb_fx[NUMSF] = L_mac0(tmp_fxL2, tmp_fx1, tmp_fx1);
        move32();

        max_es_hb_fx = L_add(es_mdct_hb_fx[0], 0);                        /* for memorising the max energy */
        max_es_fx = L_add(es_mdct_fx[0], 0);                              /* for memorising the max energy */
        maxind = 0;
        move16();
        FOR (i = 1; i <= NUMSF; i++)
        {
            IF (L_sub(es_mdct_hb_fx[i], max_es_hb_fx) >= 0)               /* '='  to handle the first window*/
            {
                max_es_hb_fx = L_add(es_mdct_hb_fx[i], 0);                /* max energy low band, 8 present and 1 future subframes */
            }

            IF (L_sub(es_mdct_fx[i], max_es_fx) >= 0)               /* '='  to handle the first window*/
            {
                max_es_fx = L_add(es_mdct_fx[i], 0);                /* max energy low band, 8 present and 1 future subframes */
                maxind = i;
                move16();
            }
        }

        cnt2 = cnt5 = 0;
        move16();
        move16();
        test();
        if( *prevflag != 0 || L_sub(max_es_fx, L_mult0(subframelength, 2500)) < 0 )
        {
            maxind = 0;
            move16();
        }

        if( L_sub(max_es_fx, L_shl(mean_prev_fx_loc, 2)) < 0 ) /*OK if saturated*/
        {
            maxind = 0;
            move16();
        }
        *prevflag = 0;
        move16();

        FOR (i = 0; i < maxind; i++)                /* only subbands before max energy subband are handled */
        {
            g_fx = 32767;
            move16();                                 /* default gain */
            min_g_fx[i] = 32767;
            move16();
            min_g_hb_fx[i] = 32767;
            move16();

            Mpy_32_16_ss(es_mdct_half_fx[i], 328, &tmp_fxL1, &tmp_u16); /* 328 for 1/100*/
            Mpy_32_16_ss(es_mdct_half_fx[i], 3277, &tmp_fxL2, &tmp_u16); /* 3277 for 1/10*/
            Mpy_32_16_ss(es_mdct_fx[i], 5461, &tmp_fxL3, &tmp_u16); /* 5461 for 1/6*/
            test();
            test();
            test();
            IF( (  L_sub(tmp_fxL1, L_add(mean_prev_nc_fx_loc,125000)) > 0)  || /* less then 20% energy in 3/4 of the subframe -> starting onset in the last quarter */
                (( L_sub(tmp_fxL2, L_add(mean_prev_nc_fx_loc,125000)) > 0) &&
                 ((sub(zcr[i], limzcr) < 0) || ( L_sub(es_mdct_quart_fx[i], tmp_fxL3) < 0)) )) /* already an offset, plosif, do not touch */
            {
                maxind = i;
                move16();                        /* no preecho reduction after the first subframe with gain 1 */
                *prevflag = 1;
                move16();
                FOR(j = sub(i,1); j >= 0; j--)
                {
                    if (L_sub(es_mdct_fx[j], L_shr(es_mdct_fx[i],1)) > 0)
                    {
                        maxind = j;
                        move16();
                    }
                }
            }
            ELSE
            {
                IF (L_sub(es_mdct_fx[i], L_shr(max_es_fx, 4)) < 0)
                {
                    g_fx = lim16_fx;
                    move16();
                    cnt5 = add(cnt5,1);

                    IF (L_sub(es_mdct_fx[i], L_shr(max_es_fx, 5)) < 0)
                    {
                        g_fx = lim32_fx;
                        move16();
                        cnt2 = add(cnt2, 1);
                    }

                    IF(L_sub(mean_prev_fx_loc, es_mdct_fx[i]) < 0)
                    {
                        tmp_fx1 = norm_l(es_mdct_fx[i]);
                        tmp_fxL1 = L_shl(es_mdct_fx[i], tmp_fx1);
                        tmp_fxL2 = L_shl(mean_prev_fx_loc, tmp_fx1);
                        tmp_fx1 = round_fx(tmp_fxL1);
                        tmp_fx2 = round_fx(tmp_fxL2);
                        tmp_fx3 = div_s(tmp_fx2, tmp_fx1);
                        min_g_fx[i] = Frac_sqrt(tmp_fx3);
                        move16();
                    }

                    IF(L_sub(mean_prev_hb_fx_loc, es_mdct_hb_fx[i]) < 0)
                    {
                        tmp_fx1 = norm_l(es_mdct_hb_fx[i]);
                        tmp_fxL1 = L_shl(es_mdct_hb_fx[i], tmp_fx1);
                        tmp_fxL2 = L_shl(mean_prev_hb_fx_loc, tmp_fx1);
                        tmp_fx1 = round_fx(tmp_fxL1);
                        tmp_fx2 = round_fx(tmp_fxL2);
                        tmp_fx3 = div_s(tmp_fx2, tmp_fx1);
                        min_g_hb_fx[i] = Frac_sqrt(tmp_fx3);
                        move16();
                    }
                    test();
                    IF( (sub(zcr[i], limzcr/2) < 0) || (sub(maxnzcr[i], limmaxnzcr) > 0) )
                    {
                        if(sub(min_g_fx[i], 32767) < 0) /* *mean_prev < es_mdct[i]) */
                        {
                            mean_prev_fx_loc = L_add(es_mdct_fx[i], 0);
                        }
                        min_g_fx[i] = 32767;
                        move16();              /* not noise-like, do not touch the amplitude, but may do in HB*/
                    }
                }
                ELSE
                {
                    test();
                    if( i > 0 && sub(maxind, NUMSF) < 0 )
                    {
                        *prevflag = 1;
                        move16();
                    }
                    maxind = i;
                    move16();                   /* no preecho reduction after the first subframe with gain 1*/
                }
            }
            gt_fx[i] = g_fx;
            move16();
            gt_hb_fx[i] = g_fx;
            move16();
        }

        FOR ( i = maxind; i <= NUMSF; i++ )         /* also for the first memory subframe */
        {
            gt_fx[i] = 32767;
            move16();
            min_g_fx[i] = 32767;
            move16();
            gt_hb_fx[i] = 32767;
            move16();
            min_g_hb_fx[i] = 32767;
            move16();

        }

        ind2 = 0;
        move16();
        FOR( i = 0; i < NUMSF; i++ )
        {
            if( sub(gt_fx[i],  32767) < 0 )                         /*gt not yet limited by min_g*/
            {
                ind2 = add(i, 1);                         /* first subframe with gain = 1 after last gain < 1 --> frame with the attack*/
            }
        }

        test();
        if( (sub(wmold_fx,  16384) > 0) && (sub(add(cnt2,cnt5), 2) < 0) )    /* mini either 1 cnt2 (and so also cnt5) or 2 cnt5 */
        {
            /* maxind = 0; false alarm, no echo reduction */
            ind2 = 0;
            move16();
        }
        ind2_m1 = sub(ind2, 1);
        ind2_sfl = i_mult(subframelength,ind2);
        numsf_ind2 = sub(NUMSF, ind2);
        fxptr3 = gt_fx;
        fxptr4 = gt_hb_fx;
        fxptr5 = min_g_fx;
        fxptr6 = min_g_hb_fx;

        fxptr1 = preechogain_fx + smooth_len;
        pre_g_ch_tab[0] = smooth_len;
        move16(); /*1st after smoothmem*/
        fxptr2 = preechogain_hb_fx;
        FOR (i = 0; i < ind2; i++) /* only subbands before max energy subband are handled*/
        {
            *fxptr3 = s_max(*fxptr3, *fxptr5);
            move16();

            *fxptr4 = s_max(*fxptr4, *fxptr6);
            move16();

            FOR(j = 0; j < subframelength; j++)
            {
                *fxptr1 = *fxptr3;
                move16();
                *fxptr2 = *fxptr4;
                move16();
                fxptr1++;
                fxptr2++;
            }
            pre_g_ch_tab[add(i, 1)] = add(pre_g_ch_tab[i], subframelength);
            fxptr3++;
            fxptr4++;
            fxptr5++;
            fxptr6++;
        }

        max_plus_es_mdct_fx = L_deposit_l(0);
        adv = smooth_len;
        move16();                                   /* samples needed to have near 1 gain after smoothing at the beggining of the attack subframe*/
        advmem = adv;
        move16();

        test();
        test();
        IF( ind2 > 0 || sub(wmold_fx, 32767) < 0 || sub(*wmold_hb_fx, 32767) < 0 )
        {
            ptr_fx = imdct_mem_fx;
            qtmp = qmemp1;
            pluslim = num_subsubframes;
            move16(); /* if ind2 == NUMSF */
            IF( numsf_ind2 > 0 )
            {
                ptr_fx =  rec_sig_fx + ind2_sfl;
                qtmp = q16p1;
                pluslim = i_mult(numsf_ind2, num_subsubframes);
            }

            maxcrit_fx = L_add(mean_prev_nc_fx_loc, 0);
            IF( ind2 == 0 )
            {
                sum_plus_es_fx = L_add(mean_prev_nc_fx_loc, 0); /* 8 times mean sususb enenrgy (=maxcrit)*/
                pluslim = num_subsubframes;
                move16();
                oldgain_fx = wmold_fx;
                move16();
                oldgain_hb_fx = *wmold_hb_fx;
                move16();
            }
            ELSE /* ind2 > 0*/
            {
                sum_plus_es_fx = es_mdct_fx[ind2_m1];
                move32(); /* 8 times mean sususb enenrgy (=maxcrit)*/
                oldgain_fx = gt_fx[ind2_m1];
                move16();
                oldgain_hb_fx = gt_hb_fx[ind2_m1];
                move16();

                tmp_fx1 = mult_r(gt_fx[ind2_m1], gt_fx[ind2_m1]);
                Mpy_32_16_ss(es_mdct_fx[ind2_m1], tmp_fx1, &maxcrit_fx, &tmp_u16);
                Mpy_32_16_ss(max_es_fx, 410, &tmp_fxL1, &tmp_u16); /* 410 for 1/80*/

                test();
                if( (L_sub(tmp_fxL1, maxcrit_fx) > 0) && (sub(zcr[ind2], limzcr) > 0) )
                {
                    maxcrit_fx = L_add(tmp_fxL1, 0);              /* still 10 times smaller then mean max_es*/
                }
            }
            fx32ptr1 = plus_es_mdct_fx;
            fx32ptr4 = mean_plus_es_fx + 1;
            FOR (j = 0; j < pluslim; j++)           /* 8  sub-subframes */
            {
                tmp_fxL1 = 100;
                move16();
                FOR (i = 0; i < subsubframelength; i++)
                {

                    tmp_fx1 = shr(*ptr_fx, qtmp); /* q-1, to have same shift as es_mdct_.. */
                    tmp_fxL1 = L_mac0(tmp_fxL1, tmp_fx1, tmp_fx1);
                    ptr_fx++;
                }
                if( L_sub(tmp_fxL1, max_plus_es_mdct_fx) > 0 )
                {
                    max_plus_es_mdct_fx = L_add(tmp_fxL1, 0);
                }

                sum_plus_es_fx = L_add(sum_plus_es_fx, L_shl(tmp_fxL1, 2));
                *fx32ptr1 = tmp_fxL1;
                fx32ptr1++;
                Mpy_32_16_ss(sum_plus_es_fx, inv_jp2[j], fx32ptr4, &tmp_u16); /* 410 for 1/80*/
                if( L_sub(*fx32ptr4, maxcrit_fx) < 0)
                {
                    *fx32ptr4 = maxcrit_fx;
                    move32();
                }
                fx32ptr4++;
            }
            *fx32ptr4 = -1;
            move32(); /*mean_plus_es_fx[pluslim] = -1; */
            *mean_plus_es_fx = *plus_es_mdct_fx;
            move32(); /* index [0] */
            if( L_sub(*mean_plus_es_fx, maxcrit_fx) < 0)
            {
                *mean_plus_es_fx = maxcrit_fx;
                move32();
            }

            j = 0;
            move16();
            WHILE((L_sub(plus_es_mdct_fx[j], mean_plus_es_fx[j]) < 0) && (L_sub(plus_es_mdct_fx[j], max_plus_es_mdct_fx/8) < 0 ))
            {
                test();
                j = add(j,1);
            }
            tmp_fx3 = i_mult(j,subsubframelength);
            adv = sub(adv, tmp_fx3);
            IF( numsf_ind2 > 0 )                    /* onset not in future frame */
            {
                fxptr1 = preechogain_fx + ind2_sfl+smooth_len;
                fxptr2 = preechogain_hb_fx + ind2_sfl;

                FOR (i = 0; i < tmp_fx3; i++)
                {
                    *fxptr1 = oldgain_fx;
                    move16();       /*keep the gain of the previous subframe*/
                    *fxptr2 = oldgain_hb_fx;
                    move16();    /*keep the gain of the previous subframe*/
                    fxptr1++;
                    fxptr2++;
                }
            }
        }

        IF(ind2>0)
        {
            /* check increasing energy of preecho by regression last 3 subframes (if possible) */
            ind3 = add(ind2, shr(j, log2_num_subsubframes));  /* return (with rounding) to subframe basis */
            ind4 = sub(ind3, 1);
            ind5 = sub(ind3, 2);
            ind6 = sub(ind3, 3);
            IF( ind4 > 0 )
            {
                /* case of 3 points is simply */
                eshbmean2_fx = L_add(es_mdct_hb_fx[ind4], es_mdct_hb_fx[ind5]);

                sxyhb2_fx = L_sub(es_mdct_hb_fx[ind4], es_mdct_hb_fx[ind5]); /* / eshbmean2 * 2; 04042013:  division not needed, only sign of sxyhb2 is used*/

                IF( sub(ind3, 2) > 0 )
                {
                    tmp_fxL1 = L_add(eshbmean2_fx, es_mdct_hb_fx[ind6]);
                    Mpy_32_16_ss(tmp_fxL1, 4369, &eshbmean3_fx, &tmp_u16); /*10922 : 1/3*/
                    sxylb3_fx = L_sub(es_mdct_fx[ind4], es_mdct_fx[ind6]); /* /eslbmean3 / 2;           /2 for 3 points regression calc; 04042013:  division not needed, only sign of sxylb3 is used*/
                    tmp_fxL1 = L_sub(es_mdct_hb_fx[ind4], es_mdct_hb_fx[ind6]);
                    test();
                    IF ((L_sub(tmp_fxL1, eshbmean3_fx) < 0) || (sxylb3_fx < 0))
                    {
                        ind2 = 0;
                        move16();
                        ind2_sfl = 0;
                        move16();
                        adv = advmem;
                        move16();
                    }
                }
                ELSE
                {
                    IF (sxyhb2_fx < 0)
                    {
                        ind2 = 0;
                        move16();
                        ind2_sfl = 0;
                        move16();
                        adv = advmem;
                        move16();/* 04042013: small bug corection*/
                    }
                }

                tmp_fxL1 = L_add(eshbmean2_fx, es_mdct_hb_fx[ind3]);
                Mpy_32_16_ss(tmp_fxL1, 4369, &eshbmean3_fx, &tmp_u16); /*10922 : 1/3*/

                tmp_fxL1 = L_sub(es_mdct_hb_fx[ind3], es_mdct_hb_fx[ind5]);
                IF (L_sub(tmp_fxL1, eshbmean3_fx) < 0)
                {
                    ind2 = 0;
                    move16();
                    ind2_sfl = 0;
                    move16();
                    adv = advmem;
                    move16();
                }
            }
        }

        ind2_m1 = sub(ind2, 1);/*ind2_m1 needs to be recomputed as ind2 could have changed since*/

        stind = sub(ind2_sfl, adv);
        stind_hb = add(stind, advmem);
        if( stind < 0 )
        {
            stind = 0;
            move16();
        }

        if( stind_hb < 0 )
        {
            stind_hb = 0;
            move16();
        }

        tmp_fx1 = add(stind, smooth_len);
        fxptr1 = preechogain_fx + tmp_fx1;
        fxptr2 = preechogain_hb_fx + stind_hb;

        FOR (i = tmp_fx1; i < framelength; i++)       /* rest of the gains, without 4 (PREECHO_SMOOTH_LEN) 1 for fadeout */
        {
            *(fxptr1++) = 32767;
            move16();
        }
        pre_g_ch_tab[ind2] = s_min(tmp_fx1, framelength);
        move16();

        FOR (i = stind_hb; i < framelength; i++)    /* rest of the gains*/
        {
            *(fxptr2++) = 32767;
            move16();
        }

        fxptr1 = preechogain_fx;
        FOR (i = 0; i < smooth_len; i++)
        {
            *(fxptr1++) =  *smoothmem_fx;
            move16();
        }

        fattnext_fx = 32767;
        move16();
        if( sub(stind, framelength) > 0 )
        {
            fattnext_fx = gt_fx[ind2_m1];
            move16();
        }

        fxptr1 = preechogain_fx + framelength;
        FOR (i = 0; i < smooth_len; i++)
        {
            *(fxptr1++) = fattnext_fx;
            move16();
        }

        FOR (i = 0; i <= ind2; i++)
        {
            tmp_fx1 = pre_g_ch_tab[i];
            move16();
            tmp_fx2 = sub(tmp_fx1, smooth_len); /* any index in the previous subframe*/
            tmp_fx3 = mult_r(sub(preechogain_fx[tmp_fx1], preechogain_fx[tmp_fx2]), invsmoothlenp1_fx);/*step*/
            tmp_fx1 = tmp_fx3;
            move16(); /*cumulated step*/
            fxptr1 = preechogain_fx + tmp_fx2;
            FOR(j = 0; j < smooth_len; j++)
            {
                *fxptr1 = add(*fxptr1, tmp_fx1);
                move16();
                tmp_fx1 = add(tmp_fx1, tmp_fx3);
                fxptr1++;
            }
        }

        *smoothmem_fx = fattnext_fx;
        move16();
        *wmold_hb_fx = preechogain_hb_fx[framelength_m1];
        move16();

        /* apply gain */
        fxptr1 = preechogain_fx;
        fxptr2 = preechogain_hb_fx;
        fxptr3 = rec_sig_fx;
        fxptr4 = rec_sig_lb_fx;
        fxptr5 = rec_sig_hb_fx;
        FOR (i = 0; i < framelength; i++)
        {
            tmp_fxL1 = L_mult(*fxptr4, *fxptr1);
            *fxptr3 = mac_r(tmp_fxL1, *fxptr5, *fxptr2);
            move16();
            fxptr1++;
            fxptr2++;
            fxptr3++;
            fxptr4++;
            fxptr5++;
        }

        mean_prev_nc_fx_loc = L_add(es_mdct_fx[0], 0);                 /* compute mean not corrected by the actual gains*/

        FOR (i = 1; i < NUMSF; i++)                 /* all present subbands */
        {
            if( sub(i, NUMSF_S2) == 0 )
            {
                savehalfe_fx = L_add(mean_prev_nc_fx_loc, 0);
            }
            mean_prev_nc_fx_loc = L_add(mean_prev_nc_fx_loc, es_mdct_fx[i]);
        }

        if( L_sub(savehalfe_fx, L_shr(mean_prev_nc_fx_loc,1) ) < 0 )
        {
            mean_prev_nc_fx_loc = L_shl(L_sub(mean_prev_nc_fx_loc, savehalfe_fx), 1);
        }
        mean_prev_nc_fx_loc = L_shr(mean_prev_nc_fx_loc, 3);  /* >> LOG2_NUMSF in fixpoint */

        FOR( i = 0; i < ind2; i++ )                 /* only subbands before max energy subband are handled*/
        {
            tmp_fx1 = mult_r(gt_fx[i], gt_fx[i]);
            Mpy_32_16_ss(es_mdct_fx[i], tmp_fx1, &es_mdct_fx[i], &tmp_u16);

            tmp_fx1 = mult_r(gt_hb_fx[i], gt_hb_fx[i]);
            Mpy_32_16_ss(es_mdct_hb_fx[i], tmp_fx1, &es_mdct_hb_fx[i], &tmp_u16);

        }

        mean_prev_fx_loc =    L_shr(es_mdct_fx[0],3);                    /* compute mean used in next frame to limit gain*/
        mean_prev_hb_fx_loc = L_shr(es_mdct_hb_fx[0],3);             /* compute mean used in next frame to limit gain*/

        FOR (i = 1; i < NUMSF; i++)                 /* all present subbands */
        {
            IF(sub(i, NUMSF_S2) == 0)
            {
                savehalfe_fx = L_add(mean_prev_fx_loc, 0);
                savehalfe_hb_fx = L_add(mean_prev_hb_fx_loc, 0);
            }

            mean_prev_fx_loc =     L_add(mean_prev_fx_loc,    L_shr(es_mdct_fx[i], 3));
            mean_prev_hb_fx_loc = L_add(mean_prev_hb_fx_loc, L_shr(es_mdct_hb_fx[i], 3));
        }

        tmp_fxL1 = L_sub(mean_prev_fx_loc, savehalfe_fx);
        if( L_sub(savehalfe_fx, L_shr(mean_prev_fx_loc, 1) ) < 0 )
        {
            mean_prev_fx_loc = L_shl(tmp_fxL1, 1);
        }

        tmp_fxL1 = L_sub(mean_prev_hb_fx_loc, savehalfe_hb_fx);
        if( L_sub(savehalfe_hb_fx, L_shr(mean_prev_hb_fx_loc, 1) ) < 0 )
        {
            mean_prev_hb_fx_loc = L_shl(tmp_fxL1, 1);
        }

        last2_fx = L_shr(L_add(es_mdct_fx[NUMSF_M1], es_mdct_fx[NUMSF_M2]), 1);
        last2_hb_fx = L_shr(L_add(es_mdct_hb_fx[NUMSF_M1], es_mdct_hb_fx[NUMSF_M2]), 1);

        if( L_sub(last2_fx, mean_prev_fx_loc) > 0 )
        {
            mean_prev_fx_loc = L_add(last2_fx, 0);
        }

        if( L_sub(last2_hb_fx, mean_prev_hb_fx_loc) > 0 )
        {
            mean_prev_hb_fx_loc = L_add(last2_hb_fx, 0);
        }
        *mean_prev_fx = mean_prev_fx_loc;
        move32();
        *mean_prev_hb_fx = mean_prev_hb_fx_loc;
        move32();
        *mean_prev_nc_fx = mean_prev_nc_fx_loc;
        move32();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * Inverse_Transform()
 *
 * Inverse transform from the DCT domain to time domain
 *--------------------------------------------------------------------------*/

void Inverse_Transform(
    const Word32 *in_mdct,           /* i  : input MDCT vector              */
    Word16 *Q,                 /* i/o: Q value of input               */
    Word32 *out,               /* o  : output vector                  */
    const Word16 is_transient,       /* i  : transient flag                 */
    const Word16 L,                  /* i  : output frame length            */
    const Word16 L_inner             /* i  : length of the transform        */
)
{
    Word16 ta, seg, tmp16;
    Word16 segment_length;
    const Word16 *win, *win2;
    Word32 out_alias[L_FRAME48k];
    Word32 alias[MAX_SEGMENT_LENGTH];
    Word32 in_mdct_modif[L_FRAME48k];
    Word32 *in_segment_modif;
    const Word32 *in_segment;
    Word32 *out_segment;
    Word16 segment_length_div2, segment_length_div4;
    Word16 tmp, q_out;
    Word32 L_temp;
    /* This value is used to right shift all vectors returned by 'iedct_short_fx()' */
    /* to bring them to a scaling that is equal to the 1st 'Q' returned by the 1st  */
    /* call to 'iedct_short_fx()' minus these guard bits.                           */
#define N_GUARD_BITS (9+1) /* 9 is enough but we put one extra bit */

    IF (is_transient)
    {
        segment_length = shr(L, 1);
        segment_length_div2 = shr(L, 2);
        segment_length_div4 = shr(L, 3);

        IF (sub(L, L_FRAME48k) == 0)
        {
            win = short_window_48kHz_fx;
        }
        ELSE IF (sub(L, L_FRAME32k) == 0)
        {
            win = short_window_32kHz_fx;
        }
        ELSE IF( sub(L, L_FRAME16k) == 0 )
        {
            win = short_window_16kHz_fx;
        }
        ELSE  /* L == L_FRAME8k */
        {
            win = short_window_8kHz_fx;
        }

        set32_fx(out_alias, 0, L);

        in_segment = in_mdct;
        in_segment_modif = in_mdct_modif;

        tmp16 = sub(L, L_inner);
        IF( tmp16 == 0 )
        {
            Copy32(in_mdct, in_mdct_modif, L);
        }
        ELSE IF( tmp16 > 0 )
        {
            FOR( seg = 0; seg < NUM_TIME_SWITCHING_BLOCKS; seg++ )
            {
                FOR( ta = 0; ta < L_inner; ta += NUM_TIME_SWITCHING_BLOCKS )
                {
                    *in_segment_modif++ = *in_segment++;
                    move32();
                }

                FOR( ta = 0; ta < tmp16; ta += NUM_TIME_SWITCHING_BLOCKS )
                {
                    *in_segment_modif++ = 0L;
                    move32();
                }
            }
        }
        ELSE /* L < L_inner */
        {
            FOR( seg = 0; seg < NUM_TIME_SWITCHING_BLOCKS; seg++ )
            {
                FOR( ta = 0; ta < segment_length_div2; ta++ )
                {
                    *in_segment_modif++ = *in_segment++;
                    move32();
                }
                in_segment += shr(sub(L_inner, L), 2);
                move32();
            }
        }

        out_segment = out_alias - segment_length_div4;
        in_segment  = in_mdct_modif;

        tmp = *Q;
        /* output of 'iedct_short_fx' has up to 'output frame length'/2 # of Elements */
        iedct_short_fx( in_segment, &tmp, alias, segment_length );
        IF (sub(tmp, N_GUARD_BITS) > 0)
        {
            q_out = sub(tmp, N_GUARD_BITS);
            tmp = sub(tmp, q_out);
        }
        ELSE
        {
            q_out = 0;
            move16();
        }

        FOR( ta = segment_length_div4; ta < segment_length_div2; ta++ )
        {
            out_segment[ta] = L_shr(alias[ta], tmp);
            move32();
        }
        /* This previous loop fills the output buffer from [0..seg_len_div4-1] */

        win2 = &win[segment_length_div2];
        FOR( ta = segment_length_div2; ta < segment_length; ta++ )
        {
            out_segment[ta] = L_shr(Mult_32_16(alias[ta], *--win2), tmp);
            move32();
        }
        /* This previous loop fills the output buffer from [seg_len_div4..seg_len-seg_len_div4-1] */

        out_segment += segment_length_div2;
        in_segment  += segment_length_div2;

        FOR( seg = 1; seg < NUM_TIME_SWITCHING_BLOCKS-1; seg++ )
        {
            tmp = *Q;
            move16();
            /* output of 'iedct_short_fx' has up to 'output frame length'/2 # of Elements */
            iedct_short_fx( in_segment, &tmp, alias, segment_length );
            tmp = sub(tmp, q_out);

            FOR( ta = 0; ta < segment_length_div2; ta++ )
            {
                out_segment[ta] = L_add(out_segment[ta], L_shr(Mult_32_16(alias[ta], *win2++), tmp));
                move32();
            }
            FOR( ; ta < segment_length; ta++ )
            {
                out_segment[ta] = L_add(out_segment[ta], L_shr(Mult_32_16(alias[ta], *--win2), tmp));
                move32();
            }

            in_segment  += segment_length_div2;
            out_segment += segment_length_div2;
        }

        tmp = *Q;
        move16();
        iedct_short_fx( in_segment, &tmp, alias, segment_length );
        tmp = sub(tmp, q_out);

        FOR (ta = 0; ta < segment_length_div2; ta++)
        {
            out_segment[ta] = L_add(out_segment[ta], L_shr(Mult_32_16(alias[ta], *win2++), tmp));
            move32();
        }

        seg = add(segment_length_div2, shr(segment_length_div2, 1)); /* seg = 3*segment_length/4 */
        FOR (ta = segment_length_div2; ta < seg; ta++)
        {
            out_segment[ta] = L_shr(alias[ta], tmp);
            move32();
        }

        FOR (ta = 0; ta < segment_length; ta++)
        {
            L_temp      = L_add(out_alias[ta], 0);
            out[ta]     = out_alias[L-1-ta];
            move32();
            out[L-1-ta] = L_temp;
            move32();
        }

        *Q = q_out;
        move16();
    }
    ELSE
    {
        edct_fx(in_mdct, out, L, Q);
    }
}
