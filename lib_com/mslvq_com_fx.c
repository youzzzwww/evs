/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "cnst_fx.h"
#include "stl.h"

/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/

static void make_offset_scale_fx( Word16 j, const Word32 tab_no_cv[], const Word16 no_ld[],
                                  Word16 no_scl, Word32 offset_scale[][MAX_NO_SCALES+1]);
static void init_offset_fx( Word32 offset_scale1[][MAX_NO_SCALES+1], Word32 offset_scale2[][MAX_NO_SCALES+1],
                            Word32 offset_scale1_p[][MAX_NO_SCALES+1], Word32 offset_scale2_p[][MAX_NO_SCALES+1],
                            Word16 no_scales[][2], Word16 no_scales_p[][2]);
static void decode_comb_fx(Word32 index,Word16 *cv,Word16 idx_lead);
static void decode_sign_pc1_fx( Word16 *c, Word16 idx_sign, Word16 parity );
static void put_value_fx(Word16 *cv, Word16 *p, Word16 val, Word16 dim, Word16 no_new_val);
static void decode_leaders_fx(Word16 index, Word16 idx_lead, Word16 *cv );
static void idx2c_fx(Word16 n, Word16 *p, Word16 k, Word16 val );
static void divide_64_32_fx(Word16 *xs,Word32 y, Word32 *result, Word32 *rem);
static void decode_indexes_fx(Word16 * index,Word16 no_bits,const Word16 * p_scales, Word16 * p_no_scales,
                              Word32 * p_offset_scale1, Word32 * p_offset_scale2,Word16 * x_lvq,Word16 mode_glb, Word16 *scales);
static Word32 divide_32_32_fx(Word32 y, Word32 x, Word32 * rem);
static Word16 divide_16_16_fx(Word16 y, Word16 x, Word16 *rem);

void permute_fx(
    Word16 *pTmp1,         /* i/o: vector whose components are to be permuted */
    const Word16 *perm     /* i  : permutation info (indexes that should be interchanged), max two perms */
)
{
    Word16 p1, p2;
    Word16 tmp;

    p1 = perm[0];
    move16();
    p2 = perm[1];
    move16();
    tmp = pTmp1[p1];
    move16();
    pTmp1[p1] = pTmp1[p2];
    move16();
    move16();
    pTmp1[p2] = tmp;
    move16();
    p1 = perm[2];
    move16();

    IF ( add(p1, 1) > 0 )
    {
        p2 = perm[3];
        move16();
        tmp = pTmp1[p1];
        move16();
        pTmp1[p1] = pTmp1[p2];
        move16();
        move16();
        pTmp1[p2] = tmp;
        move16();
    }

    return;
}


void init_lvq_fx(
    Word32 offset_scale1[][MAX_NO_SCALES+1],    /* o: lattice truncation index offset for the first LSF subvector - safety net structures*/
    Word32 offset_scale2[][MAX_NO_SCALES+1],    /* o: lattice truncation index offset for the second LSF subvector - safety net structures*/
    Word32 offset_scale1_p[][MAX_NO_SCALES+1],  /* o: lattice truncation index offset for the first LSF subvector - predictive structures*/
    Word32 offset_scale2_p[][MAX_NO_SCALES+1],  /* o: lattice truncation index offset for the second LSF subvector - predictive structures*/
    Word16 no_scales[][2],                      /* o: number of truncations for each LSF subvector at each MSLVQ structure - safety net  */
    Word16 no_scales_p[][2]                     /* o: number of truncations for each LSF subvector at each MSLVQ structure - predictive  */
)
{
    Word16 i, j;
    FOR(i=0; i<MAX_NO_MODES; i++)
    {
        j=0;
        move16();
        test();
        WHILE ((sub(j,MAX_NO_SCALES)<0) && (no_lead_fx[i][j] >0 ))
        {
            j++;
        }
        no_scales[i][0] = j;
        move16();
        j = MAX_NO_SCALES;
        move16();
        test();
        WHILE ((sub(j,shl(MAX_NO_SCALES,1))<0) && (no_lead_fx[i][j] >0 ))
        {
            j++;
        }
        no_scales[i][1] = sub(j, MAX_NO_SCALES);
        move16();
    }
    FOR(i=0; i<MAX_NO_MODES_p; i++)
    {
        j=0;
        move16();
        WHILE ((sub(j,MAX_NO_SCALES)<0) && (no_lead_p_fx[i][j] >0 ))
        {
            j++;
        }
        no_scales_p[i][0] = j;
        move16();
        j = MAX_NO_SCALES;
        move16();
        WHILE ((sub(j, shl(MAX_NO_SCALES,1))<0) && (no_lead_p_fx[i][j] >0 ))
        {
            j++;
        }
        no_scales_p[i][1] = sub(j,MAX_NO_SCALES);
        move16();
    }

    init_offset_fx( offset_scale1, offset_scale2, offset_scale1_p, offset_scale2_p, no_scales, no_scales_p );
}

/* make_offset_scale_fx() - calculates scale offset values for a particular MSLVQ structure  */
static
void make_offset_scale_fx(
    Word16 j,                             /* i: MSLVQ structure index */
    const Word32 tab_no_cv[],             /* i: cummulated number of codevectors in each leader class */
    const Word16 no_ld[],                 /* i: number of leaders in each truncation for the MSLVQ structure j*/
    Word16 no_scl,                        /* i: number of truncations in the MSLVQ structure j */
    Word32 offset_scale[][MAX_NO_SCALES+1]/* o: offset values */
)
{
    Word16 i;

    offset_scale[j][0] = L_deposit_l(1);
    FOR( i=1; i<=no_scl; i++ )
    {
        offset_scale[j][i] = L_add(offset_scale[j][sub(i,1)], tab_no_cv[no_ld[sub(i,1)]]);
        move32();
    }

    return;
}

void init_offset_fx(
    Word32 offset_scale1[][MAX_NO_SCALES+1],  /* o: lattice truncation index offset for the first LSF subvector - safety net structures*/
    Word32 offset_scale2[][MAX_NO_SCALES+1],  /* o: lattice truncation index offset for the second LSF subvector - safety net structures*/
    Word32 offset_scale1_p[][MAX_NO_SCALES+1],/* o: lattice truncation index offset for the first LSF subvector - predictive structures*/
    Word32 offset_scale2_p[][MAX_NO_SCALES+1],/* o: lattice truncation index offset for the second LSF subvector - predictive structures*/
    Word16 no_scales[][2],                    /* i: number of truncations for each LSF subvector at each MSLVQ structure - safety net  */
    Word16 no_scales_p[][2]                   /* i: number of truncations for each LSF subvector at each MSLVQ structure - predictive  */
)
{
    Word16 j;

    FOR( j=0; j<MAX_NO_MODES; j++ )
    {
        make_offset_scale_fx( j, table_no_cv_fx, no_lead_fx[j], no_scales[j][0], offset_scale1 );
        make_offset_scale_fx( j, table_no_cv_fx, &no_lead_fx[j][MAX_NO_SCALES], no_scales[j][1], offset_scale2 );
    }
    FOR( j=0; j<MAX_NO_MODES_p; j++ )
    {
        make_offset_scale_fx(j, table_no_cv_fx, no_lead_p_fx[j], no_scales_p[j][0], offset_scale1_p);
        make_offset_scale_fx(j, table_no_cv_fx, &no_lead_p_fx[j][MAX_NO_SCALES], no_scales_p[j][1], offset_scale2_p);
    }

    return;
}


static void decode_indexes_fx(
    Word16 * index,            /* i: LSF vector index, written as array of Word16 because it generally uses more than 16 bits */
    Word16 no_bits,            /* i: number of bits for the index */
    const Word16 * p_scales,   /* i: scale values for the MSLVQ structures */
    Word16 * p_no_scales,      /* i: number of truncations for each MSLVQ structure */
    Word32 * p_offset_scale1,  /* i: scale index offset for first LSF subvector */
    Word32 * p_offset_scale2,  /* i: scale index offset for second LSF subvector */
    Word16 * x_lvq,               /* o: decoded LSF vector in Q1 */
    Word16 mode_glb,           /* i: index of LSLVQ structure */
    Word16 * scales            /* o: scale values for the decoded MSLVQ LSF codevector */
)
{
    Word32 index1=0, index2=0;
    Word16 len_scales = MAX_NO_SCALES*2, no_modes;
    Word16 i, im1, idx_scale;
    Word16 tmp;

    no_modes = MAX_NO_SCALES+1;
    move16();

    IF (sub(no_bits,shl(LEN_INDICE,1)) <= 0) /* the third short is not used */
    {
        index[2] = 0;
        move16();
        if ( sub(no_bits,LEN_INDICE) <= 0 )
        {
            index[1] =0;
            move16();
        }
    }

    /* first subvector */
    tmp = i_mult2(mode_glb,no_modes);

    IF ( p_offset_scale2[add(tmp, p_no_scales[add(shl(mode_glb,1),1)])] > 0 )
    {
        divide_64_32_fx( index, p_offset_scale2[tmp+ p_no_scales[add(shl(mode_glb,1),1)]], &index1, &index2 );
    }
    ELSE
    {
        index1 = L_deposit_l(index[0]); /* this is for very low bitrates, so there is no loss in truncation */
        index2 = L_deposit_l(0);
    }
    IF ( index1 == 0 )
    {
        FOR( i=0; i<LATTICE_DIM; i++ )
        {
            x_lvq[i] = 0;
            move16();
        }
        scales[0] = 0;
    }
    ELSE
    {
        /* find idx_scale */
        i = 1;
        move16();
        WHILE( L_sub(index1, p_offset_scale1[tmp +i])>= 0 )
        {
            i = add(i, 1);
        }
        idx_scale = sub(i,1);
        move16();
        index1 = L_sub(index1, p_offset_scale1[tmp+idx_scale]);

        /* find idx_leader */
        i = 1;
        move16();

        WHILE( L_sub(index1, table_no_cv_fx[i]) >= 0 )
        {
            i = add(i, 1);
        }
        im1 = sub(i,1);
        decode_comb_fx(L_sub(index1,table_no_cv_fx[im1]), x_lvq, im1 );
        scales[0] = p_scales[mode_glb*len_scales+idx_scale];
    }

    /* second subvector */
    IF ( index2 == 0 )
    {
        FOR( i=LATTICE_DIM; i<2*LATTICE_DIM; i++ )
        {
            x_lvq[i] = 0;
            move16();
        }
        scales[1] = 0;
        move16();
    }
    ELSE
    {
        i = 1;
        move16();
        WHILE( L_sub(index2, p_offset_scale2[tmp+i]) >= 0 )
        {
            i = add(i, 1);
        }

        idx_scale = sub(i,1);
        index2 = L_sub(index2, p_offset_scale2[add(tmp,idx_scale)]);
        i = 1;
        move16();

        WHILE ( L_sub(index2, table_no_cv_fx[i]) >= 0 )
        {
            i = add(i, 1);
        }
        im1 = sub(i,1);
        decode_comb_fx( index2-table_no_cv_fx[im1], &x_lvq[LATTICE_DIM], im1 );
        scales[1] = p_scales[add(i_mult2(mode_glb,len_scales),add(MAX_NO_SCALES,idx_scale))];
        move16();
    }

    return;
}


void deindex_lvq_fx(
    Word16 *index,                  /* i  : index to be decoded, as an array of 3 Word16              */
    Word16 *x_lvq,                  /* o  : decoded codevector                             Q(x2.56)*/
    Word16 mode,                    /* i  : LVQ  coding mode/MSLVQ structure index (select scales & no_lead ), or idx_cv for CNG case */
    Word16 sf_flag,                 /* i  : safety net flag                                          */
    Word16 no_bits,                 /* i  : number of bits for lattice                              */
    Word32 *p_offset_scale1,           /* i  : offset for first subvector                              */
    Word32 *p_offset_scale2,        /* i  : offset for the second subvector                          */
    Word16 *p_no_scales             /* i  : number of scales for each truncation and each MSLVQ structure */
)
{
    Word16 i;
    const Word16 * p_scales;
    Word16 mode_glb;
    Word32 L_tmp;
    Word16 scales[2];

    IF ( sub(sf_flag,1) == 0 )
    {
        mode_glb = add(offset_lvq_modes_SN_fx[mode], offset_in_lvq_mode_SN_fx[mode][sub(no_bits,min_lat_bits_SN_fx[mode])]);
        p_scales = &scales_fx[0][0];
        move16();
    }
    ELSE
    {
        mode_glb = add(offset_lvq_modes_pred_fx[mode], offset_in_lvq_mode_pred_fx[mode][sub(no_bits,min_lat_bits_pred_fx[mode])]);
        /*CONVERT : scales_p and scales different between 1.8 and 0.9  -- seems to be done, CHECK! AV */
        p_scales = &scales_p_fx[0][0];
        move16();
    }


    decode_indexes_fx( index, no_bits, p_scales, p_no_scales, p_offset_scale1,
                       p_offset_scale2, x_lvq, mode_glb, scales    ); /* x_lvq is here Q1 */


    IF ( sub(sf_flag,1) == 0 )
    {
        IF(scales[0])
        {
            FOR( i=0; i<LATTICE_DIM; i++ )
            {
                L_tmp = L_mult(x_lvq[i],scales[0]); /* Q1+Q11+Q1  = Q13 */
                L_tmp = Mult_32_16(L_tmp,shl(sigma_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
                x_lvq[i]= extract_l(L_tmp);
            }
        }
        IF (scales[1])
        {
            FOR( i=LATTICE_DIM; i<2*LATTICE_DIM; i++ )
            {
                L_tmp = L_mult(x_lvq[i],scales[1]); /* Q1+Q11+Q1  = Q13 */
                L_tmp = Mult_32_16(L_tmp,shl(sigma_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
                x_lvq[i]= extract_l(L_tmp);
            }
        }
    }
    ELSE
    {
        IF(scales[0])
        {
            FOR( i=0; i<LATTICE_DIM; i++ )
            {
                L_tmp = L_mult(x_lvq[i],scales[0]); /* Q1+Q11+Q1  = Q13 */
                L_tmp = Mult_32_16(L_tmp,shl(sigma_p_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
                x_lvq[i]= extract_l(L_tmp);
            }
        }
        IF (scales[1])
        {
            FOR( i=LATTICE_DIM; i<2*LATTICE_DIM; i++ )
            {
                L_tmp = L_mult(x_lvq[i],scales[1]); /* Q1+Q11+Q1  = Q13 */
                L_tmp = Mult_32_16(L_tmp,shl(sigma_p_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
                x_lvq[i]= extract_l(L_tmp);
            }
        }
    }

    return;
}

/*-----------------------------------------------------------------*
 * deindex_lvq_cng()
 *
 *-----------------------------------------------------------------*/

void deindex_lvq_cng_fx(
    Word16 *index,                   /* i  : index to be decoded, as an array of 3 short */
    Word16 *x_lvq,                   /* o  : decoded codevector  Q9*/
    Word16 idx_cv,                   /* i  : relative mode_lvq, wrt START_CNG */
    Word16 no_bits,                  /* i  : number of bits for lattice */
    Word32 * p_offset_scale1,        /* i: scale index offset for first LSF subvector */
    Word32 * p_offset_scale2,        /* i: scale index offset for second LSF subvector */
    Word16 * p_no_scales             /* i: number of scales for each MSLVQ structure and each subvector */
)
{
    Word16 i;
    Word32 L_tmp;
    const Word16 *p_scales;
    Word16 mode_glb, mode;
    Word16 scales[2];

    mode_glb = add(START_CNG, idx_cv);
    mode = add(LVQ_COD_MODES, idx_cv);

    p_scales = &scales_fx[0][0];
    move16();

    decode_indexes_fx( index, no_bits, p_scales, p_no_scales, p_offset_scale1, p_offset_scale2, x_lvq, mode_glb ,scales);

    FOR(i=0; i<LATTICE_DIM; i++)
    {
        L_tmp = L_mult(x_lvq[i],scales[0]); /* Q1+Q11+Q1  = Q13 */
        L_tmp = Mult_32_16(L_tmp,shl(sigma_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
        x_lvq[i]= extract_l(L_tmp);
    }
    FOR(i=LATTICE_DIM; i<2*LATTICE_DIM; i++)
    {
        L_tmp = L_mult(x_lvq[i],scales[1]); /* Q1+Q11+Q1  = Q13 */
        L_tmp = Mult_32_16(L_tmp,shl(sigma_fx[mode][i],2)); /* Q13 + Q2 +x2.56 -Q15 */
        x_lvq[i]= extract_l(L_tmp);
    }

    /* check if permutting needed */
    IF ( cng_sort_fx[idx_cv] )
    {
        permute_fx( x_lvq, perm_fx[idx_cv] );
    }

    return;
}

static void idx2c_fx(
    Word16 n,                     /* i  : total number of positions (components)*/
    Word16 *p,                      /* o  : array with positions of the k components */
    Word16 k,                     /* i  : number of components whose position is to be determined */
    Word16 val                    /* i  : index to be decoded  */
)
{
    Word16 i, skip, pos, k1;

    skip = 0;
    move16();
    pos = 0;
    move16();
    k1 = sub(k,1);
    move16();
    WHILE( sub(add(skip,sub(C_fx[n-pos-1][k1] ,1)), val) < 0  )
    {
        skip = add(skip,C_fx[n-pos-1][k1]);
        move16();
        pos++;
        move16();
    }

    p[0] = pos;
    move16();
    n = sub(n,add(pos,1));
    val = sub(val,skip);
    IF ( sub(k, 1) == 0 )
    {
        return;
    }

    idx2c_fx( n, p+1, k1, val );

    /* pos+1 */
    FOR( i=1; i<k; i++ )
    {
        p[i] = add(p[i], add(pos,1));
        move16();
    }

    return;
}

static void decode_comb_fx(
    Word32 index,          /* i  : index to be decoded */
    Word16 *cv,            /* o  : decoded codevector Q1*/
    Word16 idx_lead        /* i  : leader class index */
)
{
    Word16 idx_sign;

    idx_sign = extract_l(div_l(L_shl(index,1), pi0_fx[idx_lead])); /*(index/pi0_fx[idx_lead]); */
    index = L_sub(index, L_mult0(idx_sign, pi0_fx[idx_lead]));
    decode_leaders_fx(extract_l(index), idx_lead, cv);
    decode_sign_pc1_fx(cv, idx_sign, pl_par_fx[idx_lead]);

    return;
}
void decode_sign_pc1_fx(
    Word16 *c,         /* o  : decoded codevector  Q1*/
    Word16 idx_sign,   /* i  : sign index */
    Word16 parity      /* i  : parity flag (+1/-1/0) */
)
{
    Word16 i, len = LATTICE_DIM, cnt_neg = 1;

    if ( parity )
    {
        len = sub(len,1);
    }

    FOR( i=0; i<len; i++ )
    {
        IF (c[i] > 0)
        {
            /*if (idx_sign % 2) */
            IF(s_and(idx_sign,1))
            {
                c[i] = negate(c[i]);
                move16();
                cnt_neg = negate(cnt_neg);
                move16();
            }
            idx_sign = shr(idx_sign,1); /*  >>= 1; */
        }
    }

    IF ( sub(len, LATTICE_DIM)<0 )
    {
        IF (sub(cnt_neg, parity) != 0)
        {
            c[len] = negate(c[len]);
            move16();
        }
    }

    return;
}


/*-----------------------------------------------------------------*
 * multiply32_32_64_fx()
 *
 * (function for int64 )
 *-----------------------------------------------------------------*/

void multiply32_32_64_fx(
    Word32 x,            /* i: first factor */
    Word32 y,            /* i: second factor */
    Word32 *res          /* o: multiplication result as array of 2 Word32*/
)
{
    Word32 tmp, high;
    Word16 x_tmp[2], y_tmp[2];

    x_tmp[0] = extract_l(L_and(x, 0x7fff)); /*extract_l(x); */ /* lowest 16 bits */
    x_tmp[1] = extract_l(L_and(L_shr(x,15),0x7fff)); /*extract_h(x); */
    y_tmp[0] = extract_l(L_and(y, 0x7fff)); /*extract_l(y); */
    y_tmp[1] = extract_l(L_and(L_shr(y,15),0x7fff)); /*extract_h(y); */
    tmp = L_mult0(x_tmp[0], y_tmp[0]);
    high = L_shr(tmp,15); /*extract_h(tmp); */
    res[0] = L_and(tmp, 0x7fff); /* extract_l(tmp); */
    tmp = L_mac0(L_mac0(high, x_tmp[1], y_tmp[0]), x_tmp[0],y_tmp[1]); /* this is not correct in general, but x and y are not using all 32 bits */
    high = L_shr(tmp,15);/*extract_h(tmp); */
    res[0] = L_add(res[0], L_shl(L_and(tmp,0x7fff), 15));
    move32();
    res[1] = L_mac0(high, x_tmp[1], y_tmp[1]);
    move32();

    return;
}

static void decode_leaders_fx(
    Word16 index,          /* i  : index to be decoded    */
    Word16 idx_lead,       /* i  : leader class index     */
    Word16 *cv             /* o  : decoded codevector   Q1*/
)
{
    Word16 i, no_vals_loc, no_vals_last, p[LATTICE_DIM], dim_loc, n_crt;
    Word16 index1;
    Word16 val_crt;

    no_vals_loc = no_vals_fx[idx_lead];
    move16();
    val_crt = vals_fx[idx_lead][no_vals_loc-1];
    move16(); /*Q1  */
    no_vals_last = no_vals_ind_fx[idx_lead][no_vals_loc-1];
    move16();

    FOR( i=0; i<no_vals_last; i++ )
    {
        cv[i] = val_crt;
        move16(); /*Q1 */
    }

    val_crt = 1;
    move16();
    dim_loc = no_vals_last;
    move16();

    SWITCH ( no_vals_loc )
    {
    case 1:
        BREAK;
    case 2:
        idx2c_fx(LATTICE_DIM, p, no_vals_ind_fx[idx_lead][0], index);
        put_value_fx(cv, p, vals_fx[idx_lead][0], no_vals_last, no_vals_ind_fx[idx_lead][0]);
        BREAK;
    case 4:
        dim_loc = add(dim_loc,no_vals_ind_fx[idx_lead][2]);
        n_crt = no_vals_ind_fx[idx_lead][2];
        index1 = divide_16_16_fx(index, C_fx[dim_loc][n_crt], &index);  /*  index1 = index/C_fx[dim_loc][n_crt]; */
        /*index = sub(index, i_mult2(index1,C_fx[dim_loc][n_crt]) ); */ /* index-= index1*C_fx[dim_loc][n_crt]; */ move16();
        idx2c_fx(dim_loc, p, n_crt, index);
        put_value_fx(cv, p, vals_fx[idx_lead][2], no_vals_last, no_vals_ind_fx[idx_lead][2]); /* Q1 */
        index = index1;
        move16();
        /* no break */
    case 3:
        dim_loc =  add(dim_loc, no_vals_ind_fx[idx_lead][1]);
        n_crt = no_vals_ind_fx[idx_lead][1];
        move16();
        index1 = divide_16_16_fx(index, C_fx[dim_loc][n_crt], &index);
        /*index  = sub(index, i_mult2(index1, C_fx[dim_loc][n_crt]));move16(); */
        idx2c_fx(dim_loc, p, n_crt, index);
        put_value_fx(cv, p, vals_fx[idx_lead][1], sub(dim_loc, n_crt), n_crt);
        idx2c_fx(LATTICE_DIM, p, no_vals_ind_fx[idx_lead][0], index1);
        move16();
        put_value_fx(cv, p, vals_fx[idx_lead][0], dim_loc, no_vals_ind_fx[idx_lead][0]);
        BREAK;
    }

    return;
}

/* divide_32_32_fx() :Division reminder - rem is the reminder of the division between y and x.  */
static Word32 divide_32_32_fx(Word32 y,   /* i */
                              Word32 x,   /* i */
                              Word32 *rem /* o */
                             )
{
    Word32 result, t, L_tmp;
    Word16 i, ny, nx, nyx;


    IF (L_sub(y, x) < 0)
    {
        result = L_deposit_l(0);
        *rem = y;
        move32();
    }
    ELSE
    {

        result = L_deposit_l(0);
        IF (y==0)
        {
            ny = 0;
            move16();
        }
        ELSE
        {
            ny = sub(31, norm_l(y));
        }
        IF (x==0)
        {
            nx = 0;
            move16();
        }
        ELSE
        {
            nx = sub(31, norm_l(x));
        }

        nyx = sub(ny,nx);

        /*t = L_and(L_shr(y, add(nyx,1)),sub(shl(1,sub(nx,1)),1)); */
        t = L_shr(y,add(nyx,1));
        FOR(i=0; i<=nyx; i++)
        {
            t = L_add(L_shl(t,1), L_and(L_shr(y,sub(nyx,i)),1));  /* L_and(y,L_shl(1, sub(nyx,i)))); */
            result = L_shl(result,1);
            L_tmp = L_sub(t,x);
            IF(L_tmp >= 0)
            {
                result = L_add(result,1);
                t = L_add(L_tmp, 0);
            }
        }
        *rem = t;
        move32();
    }
    return result;
}

/* divide_32_32_fx() :Division reminder for Word16 - rem is the reminder of the division between y and x.  */
static Word16 divide_16_16_fx(Word16 y,    /* i */
                              Word16 x,    /* i */
                              Word16 *rem  /* o */
                             )
{
    Word16 result, t, tmp;
    Word16 i, ny, nx, nyx;


    IF (L_sub(y, x) < 0)
    {
        result = 0;
        move16();
        *rem = y;
        move16();
    }
    ELSE
    {

        result = 0;
        move16();
        IF (y==0)
        {
            ny = 0;
            move16();
        }
        ELSE
        {
            ny = sub(15, norm_s(y));
        }
        IF (x==0)
        {
            nx = 0;
            move16();
        }
        ELSE
        {
            nx = sub(15, norm_s(x));
        }

        nyx = sub(ny,nx);

        t = s_and(shr(y, add(nyx,1)),sub(shl(1,sub(nx,1)),1));
        FOR(i=0; i<=nyx; i++)
        {
            t = add(shl(t,1), s_and(shr(y,sub(nyx,i)),1));  /* L_and(y,L_shl(1, sub(nyx,i)))); */
            result = shl(result,1);
            tmp = sub(t,x);
            IF(tmp >= 0)
            {
                result = add(result,1);
                t = tmp;
                move16();
            }
        }
        *rem = t;
        move16();
    }
    return result;
}

static void divide_64_32_fx(
    Word16 *xs,       /* i  : denominator as array of two int32 */
    Word32 y,         /* i  : nominator on 32 bits */
    Word32 *result,   /* o  : integer division result on 32 bits */
    Word32 *rem       /* o  : integer division reminder on 32 bits */
)
{
    Word16 nb_x1;
    Word32 r, x_tmp, x[2], q, q1;

    x[0] = L_add(L_add(L_shl(L_deposit_l(s_and(xs[2],1)),2*LEN_INDICE),L_shl(L_deposit_l(xs[1]), LEN_INDICE)),L_deposit_l(xs[0]));
    move32();
    x[1] = L_shr(L_deposit_l(xs[2]),1);
    move32();

    /*x[0] = (((xs[2])&(1)<<(LEN_INDICE*2)) + (xs[1]<<LEN_INDICE) + xs[0];
    x[1] = xs[2]>>1;                                                              */

    IF (x[1] ==0)
    {
        nb_x1 = 0;
        move16();
    }
    ELSE
    {
        nb_x1 = sub(31, norm_l(x[1])); /*get_no_bits_fx(x[1]); */
    }
    /* take the first 31 bits */
    IF ( nb_x1 > 0 )
    {
        x_tmp = L_add(L_shl(x[1],sub(31,nb_x1)),L_shr(x[0], nb_x1));
        /* x_tmp = (x[1]<<(32-nb_x1)) + (x[0]>>nb_x1);        */

        q = divide_32_32_fx(x_tmp,y, &r); /* q = x_tmp/y, reminder r */
        r = L_add(L_shl(r, nb_x1), L_and(x[0],L_deposit_l(sub(shl(1,nb_x1),1))));  /* this is the first reminder */
        /* r = (r<<nb_x1)+(x[0]&((1<<nb_x1) - 1));         */

        q1 = divide_32_32_fx(r, y, rem);
        *result = L_add(L_shl(q,nb_x1), q1);
        move32();
    }
    ELSE
    {
        *result = divide_32_32_fx(x[0], y, rem);
        move32();
    }

    return;
}

static void put_value_fx(
    Word16 *cv,                  /* i/o  : input codevector            Q1*/
    Word16 *p,                   /* i  : array with positions            */
    Word16 val,                  /* i  : value to be inserted          Q1*/
    Word16 dim,                  /* i  : vector dimension                */
    Word16 no_new_val            /* i  : number of values to be inserted */
)
{
    Word16 cv_out[LATTICE_DIM];
    Word16 i, occ[LATTICE_DIM], cnt, limit;

    limit = add(dim, no_new_val);
    FOR( i=0; i<limit; i++ )
    {
        occ[i] = 0;
        move16();
    }

    FOR( i=0; i<no_new_val; i++ )
    {
        cv_out[p[i]] = val;
        move16();
        occ[p[i]]  = 1;
        move16();
    }

    cnt = 0;
    move16();
    FOR( i=0; i<limit; i++ )
    {
        if (occ[i] == 0)
        {
            cv_out[i] = cv[cnt++];
            move16();
        }
    }

    FOR( i=0; i<limit; i++ )
    {
        cv[i] = cv_out[i];
        move16();
    }

    return;
}
