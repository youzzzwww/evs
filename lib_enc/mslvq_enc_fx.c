/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "prot_fx.h"
#include "rom_com_fx.h"
#include "cnst_fx.h"
#include "stl.h"


/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/

static Word32 quantize_data_fx( Word16 *data, const Word16 *w_in,  Word16 *qin, Word16 *cv_out, Word16 *idx_lead, Word16 *idx_scale,  const Word16 *sigma, const Word16 *inv_sigma,
                                const Word16 *scales, Word16 no_scales, const Word16 *no_lead );
static Word32 q_data_fx(Word16 *pTmp1, const Word16 *w1, Word16 *quant, Word16 *cv_out, Word16 *idx_lead, Word16 *idx_scale,
                        const Word16 *p_sigma, const Word16 *p_inv_sigma, const Word16 *p_scales, Word16 *p_no_scales,
                        const Word16 *p_no_lead);
static void prepare_data_fx( Word16 *xsort, Word16 *sign, Word16 *data, Word32 *w, const Word16 *w_in,
                             const Word16 *sigma, const Word16 * inv_sigma, Word16 *p_sig );
static Word32 calculate_min_dist_fx(Word16 cv_pot[LATTICE_DIM],Word16 no_scales, const Word16 *scale,
                                    const Word32 *w, Word16 *p_best_scale, Word16 *p_best_idx, const Word16 *no_leaders, Word16 sig,Word16 *indx);
static Word16 find_pos_fx( Word16 *c, Word16 len,	Word16 arg, Word16 *p );
static void take_out_val_fx( Word16 *v, Word16 *v_out, Word16 val, Word16 len );
static Word16 index_leaders_fx( Word16 *cv, Word16 idx_lead, Word16 dim );
static Word16 c2idx_fx( Word16 n, Word16 *p, Word16 k );
static Word16 encode_sign_pc1_fx( Word16 parity, Word16 *cv);
static Word32 encode_comb_fx( Word16 *cv, Word16 idx_lead );
static void sort_desc_ind_fx( Word16 *s, Word16 len, Word16 *ind );


/*-----------------------------------------------------------------*
 * mslvq()
 *
 * Encodes the LSF residual
 *-----------------------------------------------------------------*/
Word32 mslvq_fx (
    Word16 *pTmp,               /* i  : M-dimensional input vector  x2.56*/
    Word16 *quant,              /* o  : quantized vector  x2.56*/
    Word16 *cv_out,             /* o  : corresponding 8-dim lattice codevectors (without the scaling) Q13*/
    Word16   *idx_lead,         /* o  : leader index for each 8-dim subvector  */
    Word16   *idx_scale,        /* o  : scale index for each subvector */
    Word16 *w,                  /* i  : weights for LSF quantization  Q10*/
    Word16 mode,                /* i  : number indicating the coding type (V/UV/G...)*/
    Word16 mode_glb,            /* i  : LVQ coding mode */
    Word16 pred_flag,           /* i  : prediction flag (0: safety net, 1,2 - predictive  )*/
    Word16 no_scales[][2]
)
{
    Word32 dist, L_tmp;
    const Word16 * p_scales, *p_sigma, *p_inv_sigma;
    const Word16  *p_no_lead;
    Word16 * p_no_scales;


    dist = L_deposit_l(0);
    p_no_scales = no_scales[mode_glb];
    move16();

    IF ( pred_flag == 0 )
    {
        p_sigma = sigma_fx[mode];
        p_inv_sigma = inv_sigma_fx[mode];
        p_scales = scales_fx[mode_glb];
        p_no_lead = no_lead_fx[mode_glb];
    }
    ELSE
    {
        p_sigma = sigma_p_fx[mode];
        p_inv_sigma = inv_sigma_p_fx[mode];
        p_scales = scales_p_fx[mode_glb];
        p_no_lead = no_lead_p_fx[mode_glb];
    }


    dist = quantize_data_fx( pTmp, w, quant, cv_out, idx_lead, idx_scale,
                             p_sigma, p_inv_sigma, p_scales, p_no_scales[0], p_no_lead );

    L_tmp = quantize_data_fx( pTmp+LATTICE_DIM, w+LATTICE_DIM, quant+LATTICE_DIM,
                              cv_out+LATTICE_DIM, &idx_lead[1], &idx_scale[1],
                              p_sigma+LATTICE_DIM, p_inv_sigma+LATTICE_DIM, p_scales+MAX_NO_SCALES,
                              p_no_scales[1], p_no_lead+MAX_NO_SCALES );

    dist = L_add(dist,L_tmp);

    return dist;
}
/*-----------------------------------------------------------------*
 * q_data()
 *
 * (used for LSF quantization in CNG)
 *-----------------------------------------------------------------*/

static Word32 q_data_fx(
    Word16 *pTmp1,             /* i: M-dimensional input vector x2.56                         */
    const Word16 *w1,          /* i: M-dimensional weight vector    Q8                        */
    Word16 *quant,             /* o: quantized vector            x2.56                        */
    Word16 *cv_out,            /* o: non-scaled lattice codevector x2.56                      */
    Word16   *idx_lead,        /* o: leader indexes for each subvector                        */
    Word16   *idx_scale,       /* o: scale indexes for each subvector                         */
    const Word16 *p_sigma,     /* i: standard deviation x2.56                                 */
    const Word16 *p_inv_sigma, /* i: inverse standard deviation Q15                           */
    const Word16 *p_scales,    /* i: scale values Q11                                         */
    Word16 *p_no_scales,       /* i: number of scales/truncations for each subvector          */
    const Word16 *p_no_lead    /* i: number of leaders for each truncation and each subvector */
)
{
    Word32 dist, L_tmp;
    dist = quantize_data_fx( pTmp1, w1, quant, cv_out, idx_lead, idx_scale,
                             p_sigma, p_inv_sigma, p_scales, p_no_scales[0], p_no_lead );

    L_tmp = quantize_data_fx( pTmp1+LATTICE_DIM, w1+LATTICE_DIM,
                              quant+LATTICE_DIM, cv_out+LATTICE_DIM, &idx_lead[1], &idx_scale[1],
                              p_sigma+LATTICE_DIM, p_inv_sigma+LATTICE_DIM, p_scales+MAX_NO_SCALES,
                              p_no_scales[1], p_no_lead+MAX_NO_SCALES );

    dist = L_add(dist,L_tmp);

    return dist;
}

/*-----------------------------------------------------------------*
 * mslvq_cng()
 *
 * Encodes the LSF residual in SID frames with LVQ
 * LVQ has separate codebook for each individual first stage index
 *-----------------------------------------------------------------*/


Word32 mslvq_cng_fx (
    Word16 idx_cv,              /* i  : index of cv from previous stage                                  */
    Word16 *pTmp,               /* i  : 16 dimensional input vector                                 x2.56*/
    Word16 *quant,              /* o  : quantized vector                                            x2.56*/
    Word16 *cv_out,             /* o  : corresponding 8-dim lattice codevectors (without the scaling) Q13*/
    Word16   *idx_lead,         /* o  : leader index for each 8-dim subvector                            */
    Word16   *idx_scale,        /* o  : scale index for each subvector                                   */
    const Word16 *w,            /* i  : weights for LSF quantization                                  Q10*/
    Word16 * no_scales
)
{
    Word32 dist;
    const Word16 *p_scales, *p_sigma, *p_inv_sigma;
    const Word16 *p_no_lead;
    Word16   *p_no_scales;
    Word16 mode_glb, mode, i;
    Word16 pTmp1[M], w1[M];

    dist = L_deposit_l(0);
    mode = add(LVQ_COD_MODES, idx_cv);
    move16();

    /* for CNG there is only one bitrate but several quantizer structures, depending on the previous VQ stage */
    mode_glb = add(START_CNG, idx_cv);
    move16();

    p_sigma = sigma_fx[mode];
    move16();
    p_inv_sigma = inv_sigma_fx[mode];
    move16();
    p_scales = scales_fx[mode_glb];
    move16();
    p_no_lead = no_lead_fx[mode_glb];
    move16();
    p_no_scales = &no_scales[shl(mode_glb,1)];
    move16();

    /* check if different order or not */
    IF ( cng_sort_fx[idx_cv] )
    {
        /* change order in subvecs */
        FOR( i=0; i<M; i++ )
        {
            pTmp1[i] = pTmp[i];
            move16();
            w1[i] = w[i];
            move16();
        }

        permute_fx(pTmp1, perm_fx[idx_cv]);
        permute_fx(w1, perm_fx[idx_cv]);

        dist = q_data_fx( pTmp1, w1, quant, cv_out, idx_lead, idx_scale, p_sigma, p_inv_sigma, p_scales, p_no_scales, p_no_lead );

        permute_fx( quant, perm_fx[idx_cv] );
    }
    ELSE
    {
        dist = q_data_fx( pTmp, w, quant, cv_out, idx_lead, idx_scale, p_sigma, p_inv_sigma, p_scales, p_no_scales, p_no_lead );
    }

    return dist;
}

/*-----------------------------------------------------------------*
 * prepare_data_fx()
 *
 *-----------------------------------------------------------------*/
static void prepare_data_fx (
    Word16 *xsort,            /* o: normalized absolute valued input vector Q10 */
    Word16 *sign,             /* o: signs of input vector                       */
    Word16 *data,             /* i: input vector                          x2.56 */
    Word32 *w,                /* o; scaled weights                          Q-4 */
    const Word16 *w_in,       /* i: input weights                            Q8 */
    const Word16 *sigma,      /* i: standard deviation                    x2.56 */
    const Word16 * inv_sigma, /* i: inverse standard deviation              Q15 */
    Word16 *p_sig             /* o: parity of input vector                      */
)
{
    Word16  j, sig;
    Word16 s, inv_s;
    Word32 L_in;



    /* scale data */
    FOR( j=0; j<LATTICE_DIM; j++ )
    {
        inv_s = inv_sigma[j];
        move16(); /*Q15 */
        s = sigma[j];
        move16(); /*Qx2.56 */
        L_in = L_mult0(inv_s,data[j]);
        move16(); /*x2.56 + Q15 */
        L_in = Mult_32_16(L_in, 100); /*  100 = 25*4 Q15+6+4 = Q25 -Q15 = Q8 */
        xsort[j] = extract_l(L_shl(L_in,2)); /*Q10 */

        w[j] = L_shr(L_mult0(s,s),4);	/*x2.56 + x2.56 - Q4 */
        w[j] = Mult_32_16(L_shl(w_in[j],5),extract_l(w[j])); /*Q6 + Q5 +x2.56 +x2.56- Q15 */
        w[j] = L_shl(w[j],2);
    }


    sig = 1;
    move16();
    FOR( j=0; j<LATTICE_DIM; j++ )
    {
        IF ( xsort[j]  < 0 )
        {
            sign[j] = -1;
            move16();
            sig = negate(sig);
            move16();
            xsort[j] = negate(xsort[j]); /*Q10 */
        }
        ELSE
        {
            sign[j] = 1;
            move16();
        }
    }
    *p_sig = sig;
    move16();

    return;
}

/*-----------------------------------------------------------------*
 * calculate_min_dist()
 *
 *-----------------------------------------------------------------*/
static Word32 calculate_min_dist_fx(
    Word16 cv_pot[LATTICE_DIM],  /* i: sorted absolute valued normalized input vector            Q10 */
    Word16   no_scales,          /* i: number of scales                                              */
    const Word16 *scale,         /* i: scale values                                              Q11 */
    const Word32 *w,             /* i: scaled weights                                            Q-4 */
    Word16 *p_best_scale,        /* o: pointer to obtained scale value                               */
    Word16 *p_best_idx,          /* o: pointer to obtained leader index                              */
    const Word16 *no_leaders,    /* i: number of leaders for each truncation                         */
    Word16 sig,                  /* i: parity of input vector                                        */
    Word16 *indx                 /* i: permutation vector indicating the order of the sorted input vector */
)
{
    Word16 k, l, j, best_scale = -1, best_idx = -1;
    Word16 s, p;
    Word32 tmp_dist, min_dist;
    Word32 w_norm[LATTICE_DIM];


    Word16 wx[LATTICE_DIM];
    const Word16 * pl_crt;
    Word32 sum1[NO_LEADERS], sum2[NO_LEADERS];
    Word16 s2, p1;
    Word16 low_prec = 0;
    Word16 max_nb, nb, max_nb1;

    max_nb = 0;
    FOR(l=0; l<LATTICE_DIM; l++)
    {
        nb = sub(31, norm_l(w[l]));
        if (sub(nb, max_nb)>0)
        {
            max_nb = nb;
            move16();
        }
    }
    max_nb1 = 0;
    FOR(l=0; l<LATTICE_DIM; l++)
    {
        nb = sub(15, norm_s(cv_pot[l]));
        if (sub(nb, max_nb1)>0)
        {
            max_nb1 = nb;
            move16();
        }
    }
    nb = sub(max_nb1,13);
    if ( nb < 0 )
    {
        nb = 0;
    }

    nb = sub(30,add(max_nb,nb));
    FOR(l=0; l<LATTICE_DIM; l++)
    {
        w_norm[l] = L_shl(w[indx[l]], nb); /* Q(26-nb)  here nb = 30-nb;*/
    }

    /* compare first with the origin */
    min_dist = L_deposit_l(0);
    FOR(j=0; j<LATTICE_DIM; j++)
    {
        wx[j] = extract_h(L_shl(Mult_32_16(w_norm[j], cv_pot[j]),3)); /*Q(26-nb) + Q10 -Q15 +Q2 -Q16 = Q(7-nb)   //it is multiplicated by 2 */
    }

    s = scale[0];
    move16(); /*Q11 */
    s2 = extract_h(L_shl(L_mult(s,s),3)); /*Q11+Q11++Q1+Q3-Q16 = Q10 */
    pl_crt = &pl_fx[0];
    move16();

    FOR(j=0; j<no_leaders[0]; j++)
    {
        sum1[j] = L_deposit_l(0);
        sum2[j] = L_deposit_l(0);

        l=0;
        move16();
        WHILE (l<LATTICE_DIM-1)
        {
            p = *pl_crt;
            IF (p)
            {
                sum1[j] = L_mac(sum1[j], wx[l], p); /* Q(7-nb) + Q1 + Q1 = Q(9-nb) */
                p1 = i_mult2(p,p); /*Q2 */
                sum2[j] = L_add(sum2[j], Mult_32_16(w_norm[l], p1)); /* Q(26-nb) + Q2 -Q15 = Q(13-nb) */
                pl_crt++;
                l++;
            }
            ELSE
            {
                pl_crt += (sub(LATTICE_DIM,l));
                l = LATTICE_DIM;
                move16();
            }
        }
        IF (sub(l, LATTICE_DIM-1)==0)
        {
            p = *pl_crt;
            /* if it went up to 7th position */
            IF ( pl_par_fx[j] )
            {
                IF ( sub(sig,pl_par_fx[j]) != 0 )
                {
                    sum1[j] = L_msu(sum1[j], wx[l], p); /* Q(7-nb) + Q1 + Q1 = Q(9-nb) //Q-7 + Q1 + Q1 = Q-5 */
                    p1 = i_mult2(p,p); /*Q2 */
                    sum2[j] = L_add(sum2[j], Mult_32_16(w_norm[l], p1)); /* Q(26-nb) + Q2 -Q15 = Q(13-nb)   //Q12 + Q2 -Q15 = Q-1 */
                    pl_crt++;
                }
                ELSE
                {
                    sum1[j] = L_mac(sum1[j], wx[l], p); /* Q(7-nb) + Q1 + Q1 = Q(9-nb)   //Q-7 + Q1 + Q1 = Q-5 */
                    p1 = i_mult2(p,p); /*Q2 */
                    sum2[j] = L_add(sum2[j], Mult_32_16(w_norm[l], p1)); /* Q(26-nb) + Q2 -Q15 = Q(13-nb)    //Q12 + Q2 -Q15 = Q-1 */
                    pl_crt++;
                }
            }
            ELSE
            {
                sum1[j] = L_mac(sum1[j], wx[l], p); /* Q(7-nb) + Q1 + Q1 = Q(9-nb)   //Q-7 + Q1 + Q1 = Q-5 */
                p1 = i_mult2(p,p); /*Q2 */
                sum2[j] = L_add(sum2[j], Mult_32_16(w_norm[l], p1)); /* Q(26-nb) + Q2 -Q15 = Q(13-nb)  // Q12 + Q2 -Q15 = Q-1 */
                pl_crt++;
            }
        }

        tmp_dist = L_sub(Mult_32_16(sum2[j],s2), Mult_32_16(L_shl(sum1[j],3),s)); /* Q(13-nb) + Q10 -Q15= Q(8-nb)   Q(9-nb) + Q3+Q11 -Q15 = Q(8-nb) */

        IF ( L_sub(tmp_dist,min_dist) < 0 )
        {
            min_dist = L_add(tmp_dist, 0);
            best_scale = 0;
            move16();
            best_idx = j;
            move16();
        }
    }
    tmp_dist = L_add(min_dist,1); /* initialization */
    FOR(k=1; k<no_scales; k++)
    {
        s = scale[k];
        move16(); /*Q11 */
        IF (sub(16, norm_l(s)) <= 0)
        {
            s2 = extract_h(L_shl(L_mult(s,s),1)); /*Q11+Q11+Q1+Q1-Q16 = Q7 */
            low_prec = 1;
        }
        ELSE
        {
            s2 = extract_h(L_shl(L_mult(s,s),3)); /*Q11+Q11++Q1+Q3-Q16 = Q10 */
        }
        FOR(j=0; j<no_leaders[k]; j++)
        {
            IF (sub(low_prec,1)==0)
            {
                tmp_dist = L_sub(L_shl(Mult_32_16(sum2[j],s2),2), Mult_32_16(L_shl(sum1[j],3),s)); /* Q-1 + Q7 + 3-Q15= Q-6   Q-5 + Q3+Q11 -Q15 = Q-6 */
            }
            ELSE
            {
                tmp_dist = L_sub(Mult_32_16(sum2[j],s2), Mult_32_16(L_shl(sum1[j],3),s)); /* Q-1 + Q10 -Q15= Q-6   Q-5 + Q3+Q11 -Q15 = Q-6 */
            }
            IF ( L_sub(tmp_dist,min_dist) < 0 )
            {
                min_dist = L_add(tmp_dist, 0);
                best_scale = k;
                move16();
                best_idx = j;
                move16();
            }
        }
        low_prec = 0;
        move16();
    }
    *p_best_scale = best_scale;
    move16();
    *p_best_idx = best_idx;
    move16();

    return min_dist;
}

/*-----------------------------------------------------------------*
 * quantize_data_fx()
 *
 *-----------------------------------------------------------------*/


static Word32 quantize_data_fx(
    Word16 *data,            /* i  : residual LSF data to quantize                              x2.56*/
    const Word16 *w_in,      /* i  : weights                                                      Q10*/
    Word16 *qin,             /* o  : quantized output (scaled)                                  x2.56*/
    Word16 *cv_out,          /* o  : codevectors                                                   Q1*/
    Word16 *idx_lead,        /* o  : leader indexes for each subvector                               */
    Word16 *idx_scale,       /* o  : scale indexes for each subvector                                */
    const Word16 *sigma,     /* i  : standard deviation                                        x2.56 */
    const Word16 *inv_sigma, /* i  : inverse of standard deviation                               Q15 */
    const Word16 *scale,     /* i  : scales for each truncation                                   Q11*/
    Word16 no_scales,        /* i  : number of truncation for each subvector                         */
    const Word16 *no_leaders /* i  : number of leader vectors for each truncation of each subvector   */
)
{
    Word16 j;
    Word32 w[LATTICE_DIM];
    Word32 min_dist = 0;
    Word16 best_idx = 0, best_scale = -1;
    Word16 s;
    Word16 indx[LATTICE_DIM];
    Word16 sig, sign[LATTICE_DIM];
    Word16 cv_pot[LATTICE_DIM];
    Word16 id[8];
    Word16 smallest;
    Word32 L_tmp;

    IF ( no_scales > 0 )
    {
        prepare_data_fx( cv_pot, sign, data, w, w_in, sigma, inv_sigma, &sig );
        move16();
        sort_desc_ind_fx( cv_pot, LATTICE_DIM, indx );
        smallest = indx[LATTICE_DIM-1];
        move16();

        min_dist = calculate_min_dist_fx( cv_pot, no_scales, scale,  w, &best_scale, &best_idx ,no_leaders, sig, indx);

        IF ( add(best_scale,1) > 0 )
        {
            FOR(j=0; j<LATTICE_DIM; j++)
            {
                id[indx[j]] = j;
            }
            FOR(j=0; j<LATTICE_DIM; j++)
            {
                cv_out[j] = i_mult2(sign[j], pl_fx[best_idx*LATTICE_DIM+id[j]]);
                move16();
            }
            IF(pl_par_fx[best_idx] )
            {
                IF ( sub(sig,pl_par_fx[best_idx]) != 0 )
                {
                    cv_out[smallest] = negate(cv_out[smallest]);
                }
            }
            s = scale[best_scale];
            move16(); /*Q11 */
            FOR( j=0; j<LATTICE_DIM; j++ )
            {
                /*qin[j] = s * cv_out[j] * sigma[j]; */
                L_tmp = L_mult(cv_out[j],s); /* Q1+Q11+Q1  = Q13 */
                L_tmp = Mult_32_16(L_tmp,shl(sigma[j],2)); /* Q13 + Q2 +x2.56 -Q15 */

                qin[j] = extract_l(L_tmp); /*x2.56 */
            }
            *idx_lead = best_idx;
            move16();
            *idx_scale = best_scale;
            move16();
        }
        ELSE
        {
            FOR( j=0; j<LATTICE_DIM; j++ )
            {
                qin[j] = 0;
                move16();
            }

            *idx_lead = best_idx;
            move16();
            *idx_scale = best_scale;
            move16();
        }
    }
    ELSE
    {
        *idx_lead = 0;
        move16();
        *idx_scale = -1;
        move16();

        FOR( j=0; j<LATTICE_DIM; j++ )
        {
            cv_out[j] = 0;
            move16();
            qin[j] = 0;
            move16();
        }
    }

    return min_dist;
}

/*-----------------------------------------------------------------*
 * sort_desc_ind()
 *
 * sorts in descending order and computes indices in the sorted vector
 *-----------------------------------------------------------------*/

static void sort_desc_ind_fx(
    Word16 *s,         /* i/o: vector to be sorted  Q10*/
    Word16   len,        /* i  : vector length         */
    Word16   *ind        /* o  : array of indices      */
)
{
    Word16 i, k, sorted, a;
    Word16 t;

    FOR ( i=0 ; i<len ; i++ )
    {
        ind[i] = i;
        move16();
    }
    sorted = 0;
    FOR ( k=sub(len,1) ; k > 0 ; k-- )
    {
        IF (sorted)
        {
            BREAK;
        }

        sorted = 1;
        move16();
        FOR ( i=0 ; i < k ; i++ )
        {
            IF ( sub(s[i],s[i+1]) < 0 )
            {
                sorted = 0;
                move16();
                t = s[i];
                move16();
                s[i] = s[i+1];
                move16();
                s[i+1] = t;
                move16();
                a = ind[i];
                move16();
                ind[i] = ind[i+1];
                move16();
                ind[i+1] = a;
                move16();
            }
        }
    }

    return;
}

/*-----------------------------------------------------------------*
 * index_lvq()
 *
 * sorts in descending order and computes indices in the sorted vector
 *-----------------------------------------------------------------*/
void index_lvq_fx (
    Word16 *quant,            /* i : codevector to be indexed (2 8-dim subvectors)                   Q13*/
    Word16   *idx_lead,       /* i : leader class index for each subvector                              */
    Word16   *idx_scale,      /* i :scale index for each subvector                                      */
    Word16   mode,            /* i : integer signalling the quantizer structure for the current bitrate */
    Word16 *index,            /* o : encoded index (represented on 3 short each with 15 bits )          */
    Word32 * p_offset_scale1, /* i : scales for first subvector                                         */
    Word32 * p_offset_scale2, /* i : scales for second subvector                                        */
    Word16 * p_no_scales      /* i : number of scales for each subvector                                */
)
{
    Word32 index1, index2, tmp, idx[2];
    Word16 len_offset;

    len_offset = add(MAX_NO_SCALES,1);
    move16();

    index1 = 0;
    move16();

    /* for first subvector */
    IF ( add(idx_scale[0],1) >0 )
    {
        index1 = L_add(encode_comb_fx(quant, idx_lead[0]), L_add(table_no_cv_fx[idx_lead[0]] , p_offset_scale1[i_mult2(mode,len_offset) +idx_scale[0]]));
    }

    /* for second subvector */
    index2 = L_deposit_l(0);

    IF ( add(idx_scale[1], 1) >0 )
    {
        index2 = L_add(encode_comb_fx(&quant[LATTICE_DIM], idx_lead[1]), L_add(table_no_cv_fx[idx_lead[1]], p_offset_scale2[i_mult2(mode,len_offset)+idx_scale[1]]));
    }

    multiply32_32_64_fx(index1, p_offset_scale2[mode*len_offset+p_no_scales[mode*2+1]], idx);

    tmp = L_add(idx[0], index2);
    test();
    IF ( (L_sub(tmp, idx[0]) <0) || (L_sub(tmp, index2) < 0) )
    {
        idx[1] = L_add(idx[1], 1);
        move32();
    }

    idx[0] = tmp;
    move32();

    /* convert to 3 short */
    index[0] = ((idx[0])&(0x7fff));
    move16();
    index[1] = ((idx[0])>>15)&(0x7fff);
    move16();
    index[2] = (idx[1])&(0x7fff);
    move16();

    return;
}


/*-----------------------------------------------------------------*
 * encode_comb()
 *
 * creates an index for the lattice codevector
 *-----------------------------------------------------------------*/

static Word32 encode_comb_fx(        /* o  : index of the absolute valued codevector*/
    Word16 *cv,                      /* i  : codevector to be indexed            Q13*/
    Word16   idx_lead                /* i  : leader class index, to know the values */
)
{
    Word16 idx_sign, idx_ld_class;
    Word32 L_tmp;

    idx_sign = encode_sign_pc1_fx( pl_par_fx[idx_lead], cv );
    move16();
    idx_ld_class = index_leaders_fx( cv, idx_lead,  LATTICE_DIM );
    move16();

    L_tmp = L_mac0(idx_ld_class, idx_sign,pi0_fx[idx_lead]);

    return L_tmp;
}

/*-----------------------------------------------------------------*
 * index_leaders()
 *
 * gives the index in a class of leaders without considering the sign yet
 *-----------------------------------------------------------------*/

static Word16 index_leaders_fx(   /* o  : index                        */
    Word16 *cv,                   /* i  : codevector to be indexed  Q13*/
    Word16   idx_lead,            /* i  : leader class index           */
    Word16   dim                  /* i  : vector dimension             */
)
{
    Word16 index;
    Word16 i, no_vals_loc, nr, p[LATTICE_DIM], dim_loc;
    Word16 cv_copy[LATTICE_DIM], val_crt;

    no_vals_loc = no_vals_fx[idx_lead];
    move16();

    IF ( sub(no_vals_loc, 1) == 0 )
    {
        return 0;
    }

    FOR( i=0; i<LATTICE_DIM; i++ )
    {
        cv_copy[i] = abs_s(cv[i]); /*Q13 */
    }

    val_crt = vals_fx[idx_lead][0];
    move16();/*Q13 */
    nr = find_pos_fx(cv_copy, dim, val_crt, p);
    move16();
    index = c2idx_fx(LATTICE_DIM, p, nr);
    move16();

    IF ( sub(no_vals_loc, 2) == 0 )
    {
        return index;
    }

    take_out_val_fx( cv_copy, cv_copy, val_crt, dim );
    dim_loc = sub(dim,no_vals_ind_fx[idx_lead][0]);
    index = extract_l(L_mult0(index,C_fx[dim_loc][no_vals_ind_fx[idx_lead][1]]));

    val_crt = vals_fx[idx_lead][1];
    move16(); /*Q13 */
    nr = find_pos_fx( cv_copy, dim_loc, val_crt, p );
    move16();
    index = add(index,c2idx_fx( dim_loc, p, nr ));

    IF ( sub(no_vals_loc, 3) == 0 )
    {
        return index;
    }

    take_out_val_fx(cv_copy, cv_copy, val_crt, dim_loc);
    dim_loc = sub(dim_loc,no_vals_ind_fx[idx_lead][1]);
    index = extract_l(L_mult0(index,C_fx[dim_loc][no_vals_ind_fx[idx_lead][2]]));

    val_crt = vals_fx[idx_lead][2];
    move16(); /*Q13 */
    nr = find_pos_fx(cv_copy, dim_loc, val_crt, p);
    move16();
    index = add(index,c2idx_fx(dim_loc, p, nr));
    /* maximum 4 values */

    return index;
}

/*-----------------------------------------------------------------*
 * find_pos()
 *
 * Finds the positions in vector c for which the vector components are equal to 'arg'.
 * It returns the number of such positions and their values in the array 'p'.
 *-----------------------------------------------------------------*/

Word16 find_pos_fx(      /* o  : number of positions             */
    Word16 *c,           /* i  : input vector                 Q13*/
    Word16   len,        /* i  : input vector dim                */
    Word16 arg,          /* i  : argument to be compared with Q13*/
    Word16   *p          /* o  : vector of positions             */
)
{
    Word16 i=0, j=0;

    /* how many (j) and which (p) positions are in the relation pred(arg,c[i]) */
    FOR( i=0; i<len; i++)
    {
        IF( sub(arg, c[i]) == 0)
        {
            p[j++]=i;
            move16();
        }
    }

    return j;
}
/*-----------------------------------------------------------------*
 * encode_sign_pc1()
 *
 * Creates an index for signs of the significant codevector components
 * Gives the index of the signs - binary representation where negative sign stands for 1
 * and positive sign stands for 1.
 *-----------------------------------------------------------------*/
static Word16 encode_sign_pc1_fx( /* o  : index of signs                                             */
    Word16   parity,              /* i  : parity of the leader class to which the codevector belongs */
    Word16 *cv                    /* i  : input codevector                                        Q13*/
)
{
    Word16 idx_sign;
    Word16 cnt, i, len=LATTICE_DIM;

    idx_sign = 0;
    move16();
    cnt = 0;
    move16();

    if ( parity )
    {
        len = sub(len,1);
    }

    FOR( i=0; i<len; i++ )
    {
        IF ( cv[i] < 0 )
        {
            idx_sign = add(idx_sign,(shl(1,cnt)));
            cnt++;
        }

        if ( cv[i] > 0 )
        {
            cnt = add(cnt, 1);
        }
    }

    return idx_sign;
}

/*-----------------------------------------------------------------*
 * take_out_val()
 *
 * removes the value val from the vector v
 *-----------------------------------------------------------------*/

static void take_out_val_fx(
    Word16 *v,            /* i  : input vector                          x2.56*/
    Word16 *v_out,        /* o  : output vector without the value val        */
    Word16 val,           /* i  : value to be removed                   x2.56*/
    Word16   len          /* i  : input vector length                        */
)
{
    Word16 i, cnt;

    cnt = 0;
    move16();

    FOR( i=0; i<len; i++ )
    {
        IF (sub(v[i], val) != 0)
        {
            v_out[cnt++] = v[i];
            move16();
        }
    }

    return;
}

/*-----------------------------------------------------------------------*
 * c2idx()
 * Indexing of vectors having k non-nul positions located in the positions
 * given by the array p.
 *-----------------------------------------------------------------------*/
Word16 c2idx_fx(   /* o: index                    */
    Word16 n,      /* i: vector lenght            */
    Word16 *p,     /* i: non null value positions */
    Word16 k       /* i: non-nullnumber of values */
)
{
    Word16 skip, i, p0;
    Word16 tmp;

    IF ( sub(k, 1) == 0 )
    {
        return p[0];
    }
    ELSE
    {
        skip = 0;
        move16();
        FOR( i=1; i<=p[0]; i++ )
        {
            skip = add(skip,C_fx[sub(n,i)][sub(k,1)]);
        }

        p0 = p[0];
        move16();
        FOR( i=1; i<k; i++)
        {
            p[i] = sub(p[i],add(p0,1));
        }
        tmp = add(skip,c2idx_fx( sub(n,add(p0,1)), p+1, sub(k,1) ));

        return tmp;
    }
}

