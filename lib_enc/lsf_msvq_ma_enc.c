/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"
#include "rom_enc_fx.h"


#define MAXINT32            2147483647
#define swap(x,y,type) {type u__p; u__p=x; x=y; y=u__p;}



#define depack_4_values(cbp, val0, val1, val2, val3) \
  val0 =  shr((cbp)[0], 4); \
  val1 =  shr((cbp)[1], 4); \
  val2 =  shr((cbp)[2], 4); \
  val3 =  add(add(shr(lshl((cbp)[2],12),4),lshr(lshl((cbp)[1],12),8)),s_and((cbp)[0],0xF));

static Word32 depack_mul_values(Word16 *Tmp, const Word16 *w, const Word16 *cbp, const Word16 N)
{
    Word16 i, val0, val1, val2, val3;
    Word32 en;

    en = L_add(0,0);
    FOR (i = 0; i < N; i+=4)
    {
        depack_4_values(cbp+(i>>2)*3, val0, val1, val2, val3)
        Tmp[i+0] = mult_r(shl(w[i+0],2),val0);
        move16();
        en = L_mac(en, val0, Tmp[i+0]);
        Tmp[i+1] = mult_r(shl(w[i+1],2),val1);
        move16();
        en = L_mac(en, val1, Tmp[i+1]);
        Tmp[i+2] = mult_r(shl(w[i+2],2),val2);
        move16();
        en = L_mac(en, val2, Tmp[i+2]);
        Tmp[i+3] = mult_r(shl(w[i+3],2),val3);
        move16();
        en = L_mac(en, val3, Tmp[i+3]);
    }

    return en;
}

static void depack_sub_values(Word16 *pTmp, const Word16 *p1, const Word16 *cbp, const Word16 N)
{
    Word16 j, val0, val1, val2, val3;

    FOR (j=0; j<N; j+=4)
    {
        depack_4_values(cbp+3*(j>>2), val0, val1, val2, val3)

        /*pTmp[i] = (p1[i] - cbp[i]);*/
        pTmp[j+0] = sub(p1[j+0], val0);
        move16();	/*3Q12*1.28*/
        pTmp[j+1] = sub(p1[j+1], val1);
        move16();	/*3Q12*1.28*/
        pTmp[j+2] = sub(p1[j+2], val2);
        move16();	/*3Q12*1.28*/
        pTmp[j+3] = sub(p1[j+3], val3);
        move16();	/*3Q12*1.28*/
    }

}

/* Unroll of inner search loop for maxC == 8 */
static Word16 msvq_enc_find_p_max_8(Word32 dist[])
{
    Word16 p_max;

    p_max = 0;
    move16();

    BASOP_SATURATE_WARNING_OFF
    if (L_sub(dist[1], dist[p_max]) > 0)
    {
        p_max = 1;
        move16();
    }
    if (L_sub(dist[2], dist[p_max]) > 0)
    {
        p_max = 2;
        move16();
    }
    if (L_sub(dist[3], dist[p_max]) > 0)
    {
        p_max = 3;
        move16();
    }
    if (L_sub(dist[4], dist[p_max]) > 0)
    {
        p_max = 4;
        move16();
    }
    if (L_sub(dist[5], dist[p_max]) > 0)
    {
        p_max = 5;
        move16();
    }
    if (L_sub(dist[6], dist[p_max]) > 0)
    {
        p_max = 6;
        move16();
    }
    if (L_sub(dist[7], dist[p_max]) > 0)
    {
        p_max = 7;
        move16();
    }
    BASOP_SATURATE_WARNING_ON
    return p_max;
}

/* Unroll of inner search loop for maxC == 8 */
static Word16 msvq_enc_find_p_max_6(Word32 dist[])
{
    Word16 p_max;

    p_max = 0;
    move16();

    BASOP_SATURATE_WARNING_OFF
    if (L_sub(dist[1], dist[p_max]) > 0)
    {
        p_max = 1;
        move16();
    }
    if (L_sub(dist[2], dist[p_max]) > 0)
    {
        p_max = 2;
        move16();
    }
    if (L_sub(dist[3], dist[p_max]) > 0)
    {
        p_max = 3;
        move16();
    }
    if (L_sub(dist[4], dist[p_max]) > 0)
    {
        p_max = 4;
        move16();
    }
    if (L_sub(dist[5], dist[p_max]) > 0)
    {
        p_max = 5;
        move16();
    }
    BASOP_SATURATE_WARNING_ON
    return p_max;
}


void msvq_enc
(
    const Word16 *const*cb,/* i  : Codebook (indexed cb[*stages][levels][p])         (0Q15) */
    const Word16 dims[],   /* i  : Dimension of each codebook stage (NULL: full dim.)       */
    const Word16 offs[],   /* i  : Starting dimension of each codebook stage (NULL: 0)      */
    const Word16 u[],      /* i  : Vector to be encoded (prediction and mean removed)(3Q12) */
    const Word16 *levels,	/* i  : Number of levels in each stage                           */
    const Word16 maxC,     /* i  : Tree search size (number of candidates kept from         */
    /*      one stage to the next == M-best)                         */
    const Word16 stages,	/* i  : Number of stages                                         */
    const Word16 w[],      /* i  : Weights                                                  */
    const Word16 N,        /* i  : Vector dimension                                         */
    const Word16 maxN,     /* i  : Codebook dimension                                       */
    Word16 Idx[]           /* o  : Indices                                                  */
)
{
    Word16  j;
    const Word16 *cbp;
    Word16 p2i;
    Word16 resid_buf[2*LSFMBEST_MAX*M_MAX], *resid[2];
    Word16 *pTmp,*p1;
    Word16 *indices[2], m, s, c, c2, p_max, i, Tmp[M_MAX];
    Word16 idx_buf[2*LSFMBEST_MAX*MAX_VQ_STAGES_USED], parents[LSFMBEST_MAX];
    Word32 dist_buf[2*LSFMBEST_MAX], *dist[2], t1, tmp, en, ss2;
    Word16 (*func_ptr)(Word32 *);
    Word16 N34;
    Word16 n, maxn, start;




    /*----------------------------------------------------------------*
     * Allocate memory for previous (parent) and current nodes.
     *   Parent node is indexed [0], current node is indexed [1].
     *----------------------------------------------------------------*/
    indices[0] = idx_buf;
    indices[1] = idx_buf + maxC*stages;                 /*move16();*/
    /*vr_iset(0, idx_buf, 2*stages*maxC);*/
    set16_fx(idx_buf, 0, (Word16)(2*stages*maxC));

    resid[0] = resid_buf;
    resid[1] = resid_buf + maxC*N;                      /*move16();*/

    dist[0] = dist_buf;
    dist[1] = dist_buf + maxC;                          /*move16();*/

    /*vr_iset(0, parents, maxC);*/
    set16_fx(parents, 0, maxC);


    func_ptr = msvq_enc_find_p_max_6;
    move16();
    if (sub(maxC,8) == 0)
    {
        func_ptr = msvq_enc_find_p_max_8;
        move16();
    }

    /*----------------------------------------------------------------*
     * LSF weights are normalized, so it is always better to multiply it first
     * Set up inital distance vector
     *----------------------------------------------------------------*/
    /* Q0/16 * Qw_norm/16 << 1 >> 16 => Qwnorm-15/16 * Q0/16 << 1 => Qwnorm-14/32 * 6.5536 */
    ss2 = L_mult(mult(u[0], shl( w[0], 2 )), u[0]);
    move16();
    FOR (j=1; j<N; j++)
    {
        ss2 = L_mac(ss2, mult(u[j], shl( w[j], 2 )), u[j]);
    }

    /* Set up inital error (residual) vectors */
    pTmp = resid[1];                                             /*move16();*/
    FOR (c=0; c<maxC; c++)
    {
        Copy(u, pTmp+c*N, N);
        dist[1][c] = ss2;
        move32();
    }

    /* Loop over all stages */
    m = 1;
    move16();
    FOR (s=0; s<stages; s++)
    {
        /* codebook pointer is set to point to first stage */
        cbp = cb[s];						 /*3Q12*1.28*/         move16();

        /* Set up pointers to parent and current nodes */
        swap (indices[0], indices[1], Word16*);
        move16();
        move16();
        move16();
        swap (resid[0], resid[1], Word16*);
        move16();
        move16();
        move16();
        swap (dist[0], dist[1], Word32*);
        move32();
        move32();
        move32();

        /* p_max points to maximum distortion node (worst of best) */
        p_max = 0;
        move16();

        n = N;
        move16();
        maxn = maxN;
        move16();
        if (dims)
        {
            n = dims[s];
            move16();
        }
        if (dims)
        {
            maxn = n;
            move16();
        }

        assert((maxn % 4) == 0);
        N34 = mult(maxn, FL2WORD16(0.75f));

        start = 0;
        move16();
        if (offs)
        {
            start = offs[s];
            move16();
        }

        set16_fx(Tmp, 0, start);
        set16_fx(Tmp + start + n, 0, sub(N, add(start, n)));

        /* Set distortions to a large value */
        FOR (j = 0 ; j < maxC ; j++)
        {
            dist[1][j] = MAXINT32;
            move32();
        }

        FOR (j=0; j<levels[s]; j++)
        {
            /* Compute weighted codebook element and its energy */
            en = depack_mul_values(Tmp+start, w+start, cbp, n);

            cbp += N34;				  													                /* pointer is incremented */

            /* Iterate over all parent nodes */
            FOR (c=0; c<m; c++)
            {
                pTmp = &resid[0][c*N];
                /*tmp = (*pTmp++) * Tmp[0];*/
                t1 = L_mult(pTmp[0], Tmp[0]);

                FOR ( i=1; i<N; i++ )
                {
                    t1 = L_mac(t1, pTmp[i], Tmp[i]);
                }

                BASOP_SATURATE_WARNING_OFF
                /*NOTE: as long as a shorter distance is found, saturation can be accepted.*/
                tmp = L_add(dist[0][c], L_sub(en, L_shl(t1,1)));
                t1 = L_sub(tmp ,dist[1][p_max]);
                BASOP_SATURATE_WARNING_ON

                IF (t1 <= 0)
                {
                    /* Replace worst */
                    dist[1][p_max] = tmp;
                    move32();
                    indices[1][p_max*stages+s] = j;
                    move16();
                    add(0,0);
                    mult(0,0);
                    parents[p_max] = c;
                    move16();

                    p_max = (*func_ptr)(dist[1]);

                } /*IF (L_sub(tmp,dist[1][p_max]) < 0) */
            } /* FOR (c=0; c<m; c++) */
        } /* FOR (j=0; j<levels[s]; j++) */

        /*------------------------------------------------------------*
         * Compute error vectors for each node
         *------------------------------------------------------------*/
        pTmp = resid[1];
        FOR (c=0; c<maxC; c++)
        {
            /* Subtract codebook entry from residual vector of parent node and multiply with scale factor */
            p1 = resid[0]+parents[c]*N;
            p2i = indices[1][c*stages+s];
            move16();

            Copy(p1, pTmp, start);
            depack_sub_values(pTmp+start, p1+start, &cb[s][p2i*N34], n);
            Copy(p1+start+n, pTmp+start+n, sub(N, add(start, n)));

            pTmp += N;

            /* Get indices that were used for parent node */
            /*mvs2s(indices[0]+parents[c]*stages, indices[1]+c*stages, s);*/
            Copy(indices[0]+parents[c]*stages, indices[1]+c*stages, s);
        } /* for (c=0; c<maxC; c++) */
        m = maxC;
        move16();
    } /* for (m=1, s=0; s<stages; s++) */

    /* Find the optimum candidate */
    c2 = findIndexOfMinWord32 (dist[1], maxC);
    /*mvi2i (indices[1]+c2*stages, Idx, stages);*/
    Copy(indices[1]+c2*stages, Idx, stages);


    return;
}


extern const Word16 tbl_mid_gen_wb_5b_fx[];
extern const Word16 tbl_mid_unv_wb_5b_fx[];


void midlsf_enc(
    const Word16 qlsf0[],   /* i: quantized lsf coefficients (3Q12)	*/
    const Word16 qlsf1[],   /* i: quantized lsf coefficients (3Q12)	*/
    const Word16 lsf[],     /* i: lsf coefficients           (3Q12)	*/
    Word16 *idx,      /* o: codebook index					*/
    const Word16 lpcorder   /* i: order of the lpc					*/
    , Word32 * Bin_Ener_128_fx
    ,const Word16 Q_ener
    ,Word8 narrowBand
    ,Word32 sr_core
    ,Word16 coder_type
)
{
    Word32 err, err_min, L_tmp;
    Word16 k, k1, j, tmp, size, qlsf[M], wghts[M];
    const Word16 *ratio;



    IF ( sub(coder_type, UNVOICED) == 0 )
    {
        ratio = tbl_mid_unv_wb_5b_fx;
    }
    ELSE
    {
        ratio = tbl_mid_gen_wb_5b_fx;
    }
    size = 32;
    move16();

    /* Weights */
    Unified_weighting_fx(
        Bin_Ener_128_fx, /* i  : FFT Bin energy 128 bins in two sets    Q_ener */
        Q_ener,
        lsf,          /* i  : LSF vector                             x2.56 */
        wghts,            /* o  : LP weighting filter (numerator)         Q8 */
        narrowBand,     /* i  : flag for Narrowband                     */
        sub(coder_type, UNVOICED) == 0,       /* i  : flag for Unvoiced frame                 */
        sr_core,        /* i  : sampling rate of core-coder             */
        lpcorder              /* i  : LP order                                */
    );
    err_min = MAXINT32;
    move16();
    *idx = 0;
    move16();
    k1 = 0;
    move16();
    FOR ( k = 0; k < size; k++ )
    {
        err = L_deposit_l(0);

        FOR (j=0; j<M; j++)
        {
            /*      qlsf[j] = (1.0f - ratio[k*M+j]) * qlsf0[j] + ratio[k*M+j] * qlsf1[j]; */
            L_tmp = L_mult(sub(0x2000, ratio[k1+j]), qlsf0[j]);
            L_tmp = L_mac(L_tmp,ratio[k1+j],qlsf1[j]);
            qlsf[j] = round_fx(L_shl(L_tmp,2));
            test();
            test();
            IF ( j > 0 && sub(j, M) < 0 && sub(qlsf[j], add(qlsf[j-1], LSF_GAP_MID_FX)) < 0)
            {
                qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
            }

            tmp = sub(lsf[j],qlsf[j]);
            /*        err +=  wghts[j] * ftemp * ftemp; */
            /* tmp is usually very small, we can have some extra precision with very rare saturation */
            tmp = shl(tmp, 4);
            tmp = mult_r(tmp, tmp);
            err = L_mac(err, tmp, wghts[j]);
        }
        err = L_shl( err, 2 );

        /*	  err = L_shl(err,Wscale); */
        err = Mult_32_16(err,LSF_1_OVER_256SQ);
        /*	  err = Mult_32_16(err,Wmult); */

        IF ( L_sub(err,err_min) < 0 )
        {
            err_min = L_add(err, 0);
            *idx = k;
            move16();
        }
        k1+=M;
        move16();
    }

    return;
}


/* Returns: number of indices */
Word16 Q_lsf_tcxlpc(
    /* const */ Word16 lsf[],     /* (I) original lsf      */
    Word16 lsf_q[],               /* (O) quantized lsf     */
    Word16 lsp_q_ind[],           /* (O) quantized lsp (w/o MA prediction) */
    Word16 indices[],             /* (O) VQ indices        */
    Word16 lpcorder,              /* (I) LPC order         */
    Word16 narrowband,            /* (I) narrowband flag   */
    Word16 cdk,                   /* (I) codebook selector */
    Word16 mem_MA[]               /* (I) MA memory         */
    , Word16 coder_type
    ,  Word32 * Bin_Ener
    ,const Word16 Q_ener
)
{
    Word16 weights[17];
    Word16 pred[M16k];
    Word16 i;
    Word16 NumIndices;
    Word16 lsf_q_ind[M16k];
    const Word16 *means;
    Word16 lsf_rem[M];
    Word16 lsf_rem_q_ind[M];

    Unified_weighting_fx( Bin_Ener, Q_ener, lsf, weights, narrowband, sub(coder_type,UNVOICED)==0, 12800, M );

    move16();
    NumIndices = 0;

    /* Put disabled flag */
    indices[NumIndices] = 0;
    move16();
    NumIndices = add(NumIndices, 1);

    /* Inter-frame prediction */

    means = lsf_means[narrowband]; /* 14Q1 * 1.28 */

    FOR (i=0; i<lpcorder; ++i)
    {
        pred[i] = add(means[i], mult_r(MU_MA_FX, mem_MA[i])); /* 14Q1 * 1.28  + ( 14Q1 * 1.28 * Q15 ) = 14Q1 * 1.28*/
    }

    /* Subtract prediction */

    FOR (i=0; i<lpcorder; ++i)
    {
        lsf[i] = sub(lsf[i], pred[i]); /* 14Q1 * 1.28 */
    }


    msvq_enc(
        lsf_codebook[narrowband][cdk],
        lsf_dims,
        lsf_offs,
        lsf,
        lsf_numlevels,
        kMaxC,
        TCXLPC_NUMSTAGES,
        weights,
        lpcorder,
        lpcorder,
        indices + NumIndices
    );
    msvq_dec(
        lsf_codebook[narrowband][cdk],
        lsf_dims,
        lsf_offs,
        TCXLPC_NUMSTAGES,
        lpcorder,
        lpcorder,
        indices + NumIndices,
        lsf_q
    );
    NumIndices = add(NumIndices, TCXLPC_NUMSTAGES);

    FOR (i=0; i<lpcorder; ++i)
    {
        lsf_q_ind[i] = lsf_q[i];
        move16();
    }

    /* Update flag */
    indices[0] = lsf_ind_is_active(lsf_q_ind, lsf_means[narrowband], narrowband, cdk);
    move16();

    /* Get residual vector */
    FOR (i=0; i<lpcorder; ++i)
    {
        lsf_rem[i] = add(sub(pred[i], lsf_means[narrowband][i]), sub(lsf[i], lsf_q_ind[i]));
    }

    /* Quantize using extra stage(s) */
    msvq_enc(
        lsf_ind_codebook[narrowband][cdk],
        lsf_ind_dims,
        lsf_ind_offs,
        lsf_rem,
        lsf_ind_numlevels,
        kMaxC,
        TCXLPC_IND_NUMSTAGES,
        weights,
        lpcorder,
        lpcorder,
        indices + NumIndices
    );
    /* Only add contribution if flag is enabled */
    IF (indices[0])
    {
        /* Decode */
        msvq_dec(
            lsf_ind_codebook[narrowband][cdk],
            lsf_ind_dims,
            lsf_ind_offs,
            TCXLPC_IND_NUMSTAGES,
            lpcorder,
            lpcorder,
            indices + NumIndices,
            lsf_rem_q_ind
        );
        NumIndices = add(NumIndices, TCXLPC_IND_NUMSTAGES);

        /* Add to MA-removed vector */
        FOR (i=0; i<lpcorder; ++i)
        {
            lsf_q_ind[i] = add(lsf_q_ind[i], lsf_rem_q_ind[i]);
        }
    }

    /* Add inter-frame prediction */
    FOR (i=0; i<lpcorder; ++i)
    {
        lsf_q[i] = add(lsf_q[i], pred[i]);
        lsf[i] = add(lsf[i], pred[i]);
    }

    reorder_lsf_fx(lsf_q, TCXLPC_LSF_GAP, lpcorder, INT_FS_FX);

    FOR (i=0; i<lpcorder; ++i)
    {
        lsf_q_ind[i] = add(lsf_q_ind[i], lsf_means[narrowband][i]);
    }
    reorder_lsf_fx(lsf_q_ind, TCXLPC_LSF_GAP, lpcorder, INT_FS_FX);

    IF (lsp_q_ind)
    {
        E_LPC_lsf_lsp_conversion/*lsf2lsp*/(lsf_q_ind, lsp_q_ind, lpcorder);
    }

    return NumIndices;
}

/* Returns: number of bits written */
Word16 enc_lsf_tcxlpc(
    Word16 **indices,             /* (I) Ptr to VQ indices  */
    Encoder_State_fx *st          /* (I/O) Encoder state    */
)
{
    Word16 i, NumBits;

    Word16 flag;

    /* Read flag */
    flag = (*indices)[0];
    move16();
    ++*indices;

    NumBits = TCXLPC_NUMBITS;
    move16();
    FOR (i=0; i<TCXLPC_NUMSTAGES; ++i)
    {
        push_next_indice_fx(st, **indices, lsf_numbits[i]);
        ++*indices;
    }

    IF (flag)
    {
        NumBits = add(NumBits, TCXLPC_IND_NUMBITS);
        FOR (i=0; i<TCXLPC_IND_NUMSTAGES; ++i)
        {
            push_next_indice_fx(st, **indices, lsf_ind_numbits[i]);
            ++*indices;
        }
    }
    return NumBits;
}



Word16 lsf_msvq_ma_encprm( Encoder_State_fx * st,
                           Word16 *param_lpc,
                           Word16 core,
                           Word16 acelp_mode,
                           Word16 acelp_midLpc,
                           Word16 * bits_param_lpc,
                           Word16 no_indices
                         )
{
    Word16 i, nbits_lpc;
    Word16  bits_midlpc=5;


    move16();
    nbits_lpc = 0;

    FOR (i=0; i<no_indices; i++)
    {

        push_next_indice_fx(st, *param_lpc, bits_param_lpc[i]);
        param_lpc++;
        nbits_lpc = add(nbits_lpc, bits_param_lpc[i]);
    }
    IF ( sub(acelp_mode,VOICED) != 0 )
    {
        test();
        IF ( core==0 && acelp_midLpc)
        {

            push_next_indice_fx(st, *param_lpc, bits_midlpc);
            nbits_lpc = add(nbits_lpc, bits_midlpc);
        }
    }

    return nbits_lpc;
}



Word16 lsf_bctcvq_encprm( Encoder_State_fx *st, Word16 *param_lpc, Word16 * bits_param_lpc, Word16 no_indices)
{
    Word16 i, nbits_lpc;

    nbits_lpc = 0;

    FOR (i=0; i<no_indices; i++)
    {
        push_next_indice_fx(st, *param_lpc, bits_param_lpc[i]);
        param_lpc++;
        nbits_lpc = add(nbits_lpc, bits_param_lpc[i]);
    }

    return nbits_lpc;
}

