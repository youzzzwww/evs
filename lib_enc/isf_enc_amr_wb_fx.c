/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                 */
#include "cnst_fx.h"      /* Common constants                     */
#include "rom_enc_fx.h"   /* Encoder static table prototypes      */
#include "rom_com_fx.h"   /* Static table prototypes              */
#include "prot_fx.h"      /* Function prototypes                  */
#include "stl.h"          /* required by wmc_tool                 */


/*-----------------------------------------------------------------*
 * Local constants
 *-----------------------------------------------------------------*/

#define DICO1_NS_19b     16       /* codebook dimensions for SID ISF quantizers */
#define DICO2_NS_19b     16
#define DICO3_NS_19b     16
#define DICO4_NS_19b     8
#define DICO5_NS_19b     16

#define DICO1_NS_28b     64
#define DICO2_NS_28b     64
#define DICO3_NS_28b     64
#define DICO4_NS_28b     32
#define DICO5_NS_28b     32

#define N_SURV_MAX       4        /* maximum number of survivors */

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static void qisf_ns_28b_fx( Encoder_State_fx *st, Word16 *isf );
static void qisf_2s_46b_fx( Encoder_State_fx *st, Word16 *isf, Word16 nb_surv, Word16 *mem_AR, Word16 *mem_MA );
static void qisf_2s_36b_fx( Encoder_State_fx *st, Word16 *isf, Word16 nb_surv, Word16 *mem_AR, Word16 *mem_MA );
static void VQ_stage1_fx(const Word16 *x, const Word16 *dico, const Word16 dim, const Word16 dico_size, Word16 *index, const Word16 surv);
static Word16 sub_VQ_fx(Word16 *x, const Word16 *dico, const Word16 dim, const Word16 dico_size, Word32 *distance);

/*-------------------------------------------------------------------*
 * isf_enc_amr_wb()
 *
 * Quantization of ISF parameters in AMR-WB IO mode
 *-------------------------------------------------------------------*/

void isf_enc_amr_wb_fx(
    Encoder_State_fx *st,        /* i/o: state structure                             */
    Word16 *isf_new,     /* i/o: quantized ISF vector                        */
    Word16 *isp_new,     /* i/o: ISP vector to quantize/quantized            */
    Word16 *Aq,          /* o  : quantized A(z) for 4 subframes              */
    Word16 clas,         /* i  : signal class                                */
    Word16 *stab_fac     /* o  : ISF stability factor                        */
)
{

    /* convert ISPs to ISFs */

    /*---------------------------------*
     * ISF quantization of SID frames
     *---------------------------------*/

    IF ( L_sub(st->core_brate_fx,SID_1k75) == 0 )
    {
        qisf_ns_28b_fx( st, isf_new );

        reorder_isf_fx( isf_new, ISF_GAP_FX, M, Fs_2);

        E_LPC_isf_isp_conversion( isf_new, isp_new, M);

        /* return if SID frame (conversion to A(z) done in the calling function) */
        return;
    }

    /* check resonance for pitch clipping algorithm */
    gp_clip_test_isf_fx( isf_new, st->clip_var_fx, 1 );

    /*---------------------------------------*
     * ISF quantization of all other frames
     *---------------------------------------*/

    IF ( L_sub(st->core_brate_fx,ACELP_6k60) == 0 )
    {
        qisf_2s_36b_fx( st, isf_new, 4, st->mem_AR_fx, st->mem_MA_fx );
    }
    ELSE IF( L_sub(st->core_brate_fx,ACELP_8k85) >= 0 )
    {
        qisf_2s_46b_fx( st, isf_new, 4, st->mem_AR_fx, st->mem_MA_fx );
    }

    reorder_isf_fx( isf_new, ISF_GAP_FX, M, Fs_2 );
    /* convert quantized ISFs back to ISPs */
    E_LPC_isf_isp_conversion( isf_new, isp_new, M);

    /*-------------------------------------------------------------------------------------*
     * ISP interpolation
     * A(z) calculation
     *-------------------------------------------------------------------------------------*/
    if(st->rate_switching_reset)
    {
        Copy( isf_new, st->lsf_old_fx, M );
        Copy( isp_new, st->lsp_old_fx, M );
    }
    int_lsp_fx( L_FRAME, st->lsp_old_fx, isp_new, Aq, M, clas, interpol_isp_amr_wb_fx, 1 );

    /*------------------------------------------------------------------*
     * Calculate ISF stability (distance between old ISF and current ISF)
     *------------------------------------------------------------------*/

    *stab_fac = lsf_stab_fx( isf_new, st->lsf_old_fx, 1, L_FRAME );


    return;
}

/*-------------------------------------------------------------------*
* qisf_ns_28b()
*
* ISF quantizer for SID frames (only in AMR-WB IO mode)
*-------------------------------------------------------------------*/

static void qisf_ns_28b_fx(
    Encoder_State_fx *st,      /* i/o: encoder state structure          */
    Word16 *isf        /* i/o: unquantized/quantized ISF vector */
)
{
    Word16 i, indice[5];
    Word32 tmp;

    FOR (i=0; i<M; i++)
    {
        /*isf[i] -= mean_isf_noise_amr_wb[i];*/
        isf[i] = sub(isf[i],  mean_isf_noise_amr_wb_fx[i]);
        move16();
    }

    indice[0] = sub_VQ_fx(&isf[0], dico1_ns_28b_fx, 2, DICO1_NS_28b, &tmp);
    move16();
    indice[1] = sub_VQ_fx(&isf[2], dico2_ns_28b_fx, 3, DICO2_NS_28b, &tmp);
    move16();
    indice[2] = sub_VQ_fx(&isf[5], dico3_ns_28b_fx, 3, DICO3_NS_28b, &tmp);
    move16();
    indice[3] = sub_VQ_fx(&isf[8], dico4_ns_28b_fx, 4, DICO4_NS_28b, &tmp);
    move16();
    indice[4] = add(sub_VQ_fx(&isf[12],dico5_ns_28b_fx+4, 4, DICO5_NS_28b-1, &tmp), 1);   /* First vector has a problem -> do not allow   */

    /* write indices to array */
    push_indice_fx( st, IND_ISF_0_0, indice[0], 6 );
    push_indice_fx( st, IND_ISF_0_1, indice[1], 6 );
    push_indice_fx( st, IND_ISF_0_2, indice[2], 6 );
    push_indice_fx( st, IND_ISF_0_3, indice[3], 5 );
    push_indice_fx( st, IND_ISF_0_4, indice[4], 5 );

    /* decoding the ISFs */
    disf_ns_28b_fx( indice, isf);

    return;
}


/*---------------------------------------------------------------------*
 * qisf_2s_36b()
 *
 * ISF quantizer for AMR-WB 6k60 frames
 *
 * The ISF vector is quantized using two-stage MA-prediction VQ with split-by-2
 * in 1st stage and split-by-3 in the second stage.
 *---------------------------------------------------------------------*/

static void qisf_2s_36b_fx(
    Encoder_State_fx *st,     /* i/o: encoder state structure             */
    Word16 *isf,      /* i/o: unquantized/quantized ISF vector    */
    Word16 nb_surv,   /* i  : number of survivors (1, 2, 3 or 4)  */
    Word16 *mem_AR,   /* o  : quantizer memory for AR model       */
    Word16 *mem_MA    /* i/o: quantizer memory for MA model       */
)
{
    Word16 i, k, indice[5], tmp_ind[2];
    Word16 surv1[N_SURV_MAX], tmp16;     /* indices of survivors from 1st stage */
    Word32 temp, min_err, distance;
    Word16 isf2[M];

    /*------------------------------------------------------------------------*
     * Subtract mean
     *------------------------------------------------------------------------*/

    FOR (i=0; i<M; i++)
    {
        /*isf[i] -= mean_isf_amr_wb[i] + MU_MA * mem_MA[i];*/
        tmp16 = sub(isf[i], mean_isf_amr_wb_fx[i]);
        isf[i] = sub(tmp16, mult(MU_MA_FX, mem_MA[i]));
        move16();

    }

    /*------------------------------------------------------------------------*
     * Quantize ISFs 0 - 8
     *------------------------------------------------------------------------*/

    VQ_stage1_fx(&isf[0], dico1_isf_fx, 9, SIZE_BK1, surv1, nb_surv);

    distance = MAX_32;
    move32();
    nb_surv = s_min(nb_surv, N_SURV_MAX);

    FOR (k=0; k<nb_surv; k++)
    {
        tmp16 = i_mult2(surv1[k], 9);
        FOR (i = 0; i < 9; i++)
        {
            /*isf2[i] = isf[i] - dico1_isf[i+surv1[k]*9];*/
            isf2[i] = sub(isf[i], dico1_isf_fx[i + tmp16]);
            move16();
        }

        tmp_ind[0] = sub_VQ_fx(&isf2[0], dico21_isf_36b_fx, 5, SIZE_BK21_36b, &min_err);
        temp = L_add(min_err, 0);

        tmp_ind[1] = sub_VQ_fx(&isf2[5], dico22_isf_36b_fx, 4, SIZE_BK22_36b, &min_err);
        temp = L_add(temp, min_err);

        IF (L_sub(temp,distance) < 0)
        {
            distance = L_add(temp, 0);
            indice[0] = surv1[k];
            move16();
            FOR (i=0; i<2; i++)
            {
                indice[i+2] = tmp_ind[i];
                move16();
            }
        }
    }

    /*------------------------------------------------------------------------*
     * Quantize ISFs 9 - 15
     *------------------------------------------------------------------------*/

    VQ_stage1_fx(&isf[9], dico2_isf_fx, 7, SIZE_BK2, surv1, nb_surv);

    distance = MAX_32;
    move32();
    FOR (k=0; k<nb_surv; k++)
    {
        tmp16 = i_mult2(surv1[k], 7);
        FOR (i = 0; i < 7; i++)
        {
            /*isf2[9+i] = isf[9+i] - dico2_isf[i+surv1[k]*7];*/
            isf2[i+9] = sub(isf[9 + i], dico2_isf_fx[i + tmp16]);
            move16();
        }

        tmp_ind[0] = sub_VQ_fx(&isf2[9], dico23_isf_36b_fx, 3, SIZE_BK23_36b, &min_err);
        move16();
        temp = L_add(min_err, 0);
        IF (L_sub(temp, distance) < 0)
        {
            distance = L_add(temp, 0);
            indice[1] = surv1[k];
            move16();
            indice[4] = tmp_ind[0];
            move16();
        }
    }

    /*------------------------------------------------------------------------*
     * decoding the ISFs
     *------------------------------------------------------------------------*/

    disf_2s_36b_fx( indice, isf, mem_AR, mem_MA, 0 );

    /*------------------------------------------------------------------------*
     * write indices to array
     *------------------------------------------------------------------------*/

    indice[0] = Indirect_dico1[indice[0]];
    move16(); /* Make interoperable with G722.2 */

    push_indice_fx( st, IND_ISF_0_0, indice[0], 8 );
    push_indice_fx( st, IND_ISF_0_1, indice[1], 8 );
    push_indice_fx( st, IND_ISF_1_0, indice[2], 7 );
    push_indice_fx( st, IND_ISF_1_1, indice[3], 7 );
    push_indice_fx( st, IND_ISF_1_2, indice[4], 6 );

    return;
}


/*-------------------------------------------------------------------*
 * qisf_2s_46b()
 *
 * ISF quantizer for all other AMR-WB frames
 *
 * The ISF vector is quantized using two-stage VQ with split-by-2
 * in 1st stage and split-by-5 in the second stage.
 *-------------------------------------------------------------------*/

static void qisf_2s_46b_fx(
    Encoder_State_fx *st,       /* i/o: encoder state structure      */
    Word16 *isf,      /* i/o: unquantized/quantized ISF vector      */
    Word16 nb_surv,   /* i  : number of survivors (1, 2, 3 or 4)    */
    Word16 *mem_AR,   /* o  : quantizer memory for AR model         */
    Word16 *mem_MA    /* i/o: quantizer memory for MA model         */
)
{
    Word16 i, k, indice[7], tmp_ind[5];
    Word16 surv1[N_SURV_MAX];     /* indices of survivors from 1st stage */
    Word32 temp, min_err, distance;
    Word16 tmp16, isf2[M];


    /*------------------------------------------------------------------------*
     * Subtract mean
     *------------------------------------------------------------------------*/

    FOR (i=0; i<M; i++)
    {
        /*isf[i] -= mean_isf_amr_wb[i] + MU_MA * mem_MA[i];*/
        tmp16 = sub(isf[i], mean_isf_amr_wb_fx[i]);
        isf[i] = sub(tmp16, mult(MU_MA_FX, mem_MA[i]));
        move16();
    }

    /*------------------------------------------------------------------------*
     * Quantize ISFs 0 - 8
     *------------------------------------------------------------------------*/

    VQ_stage1_fx(&isf[0], dico1_isf_fx, 9, SIZE_BK1, surv1, nb_surv);

    distance = MAX_32;
    move32();
    nb_surv = s_min(nb_surv, N_SURV_MAX);

    FOR (k=0; k<nb_surv; k++)
    {
        tmp16 = i_mult2(surv1[k], 9);
        FOR (i = 0; i < 9; i++)
        {
            /*isf2[i] = isf[i] - dico1_isf[i+surv1[k]*9];*/
            isf2[i] = sub(isf[i], dico1_isf_fx[i + tmp16]);
            move16();
        }

        tmp_ind[0] = sub_VQ_fx(&isf2[0], dico21_isf_46b_fx, 3, SIZE_BK21, &min_err);
        temp = L_add(min_err, 0);
        tmp_ind[1] = sub_VQ_fx(&isf2[3], dico22_isf_46b_fx, 3, SIZE_BK22, &min_err);
        temp = L_add(temp, min_err);
        tmp_ind[2] = sub_VQ_fx(&isf2[6], dico23_isf_46b_fx, 3, SIZE_BK23, &min_err);
        temp = L_add(temp, min_err);
        IF (L_sub(temp,distance) < 0)
        {
            distance = L_add(temp, 0);
            indice[0] = surv1[k];
            move16();
            FOR (i=0; i<3; i++)
            {
                indice[i+2] = tmp_ind[i];
                move16();
            }
        }
    }

    /*------------------------------------------------------------------------*
     * Quantize ISFs 9 - 15
     *------------------------------------------------------------------------*/

    VQ_stage1_fx(&isf[9], dico2_isf_fx, 7, SIZE_BK2, surv1, nb_surv);

    distance = MAX_32;
    move32();
    FOR (k=0; k<nb_surv; k++)
    {
        tmp16 = i_mult2(surv1[k], 7);
        FOR (i = 0; i < 7; i++)
        {
            /*isf2[9+i] = isf[9+i] - dico2_isf[i+surv1[k]*7];*/
            isf2[i+9] = sub(isf[9 + i], dico2_isf_fx[i + tmp16]);
            move16();
        }
        tmp_ind[0] = sub_VQ_fx(&isf2[9], dico24_isf_46b_fx, 3, SIZE_BK24, &min_err);
        move16();
        temp = L_add(min_err, 0);

        tmp_ind[1] = sub_VQ_fx(&isf2[12], dico25_isf_46b_fx, 4, SIZE_BK25, &min_err);
        move16();
        temp = L_add(temp, min_err);

        IF (L_sub(temp, distance) < 0)
        {
            distance = L_add(temp, 0);
            indice[1] = surv1[k];
            move16();
            FOR (i=0; i<2; i++)
            {
                indice[i+5]=tmp_ind[i];
                move16();
            }
        }
    }

    /*------------------------------------------------------------------------*
     * decoding the ISFs
     *------------------------------------------------------------------------*/

    disf_2s_46b_fx( indice, isf, mem_AR, mem_MA, 0 );

    /*------------------------------------------------------------------------*
     * write indices to array
     *------------------------------------------------------------------------*/
    indice[0] = Indirect_dico1[indice[0]];
    move16(); /* Make interoperable with G722.2 */

    push_indice_fx( st, IND_ISF_0_0, indice[0], 8 );
    push_indice_fx( st, IND_ISF_0_1, indice[1], 8 );
    push_indice_fx( st, IND_ISF_1_0, indice[2], 6 );
    push_indice_fx( st, IND_ISF_1_1, indice[3], 7 );
    push_indice_fx( st, IND_ISF_1_2, indice[4], 7 );
    push_indice_fx( st, IND_ISF_1_3, indice[5], 5 );
    push_indice_fx( st, IND_ISF_1_4, indice[6], 5 );

    return;
}

/*-------------------------------------------------------------------*
 * VQ_stage1()
 *
 * 1st stage of ISF quantization
 *-------------------------------------------------------------------*/

static void VQ_stage1_fx(
    const Word16 *x,         /* i  : ISF vector                         */
    const Word16 *dico,      /* i  : ISF codebook                       */
    const Word16 dim,        /* i  : codebook dimension                 */
    const Word16 dico_size,  /* i  : codebook size                      */
    Word16 *index,     /* o  : indices of best vector candidates  */
    const Word16 surv        /* i  : nb of surviving best candidates    */
)
{
    Word32 dist_min[N_SURV_MAX];
    Word32 dist;
    const Word16 *p_dico;
    Word16   i, j, k, l, temp;


    FOR (i=0; i<surv; i++)
    {
        dist_min[i] = MAX_32;
        move32();
        index[i] = i;
        move16();
    }

    p_dico = dico;

    FOR (i = 0; i < dico_size; i++)
    {
        dist = L_deposit_l(0);
        FOR (j = 0; j < dim; j++)
        {
            /*temp = x[j] - *p_dico++;*/
            temp = sub(x[j], *p_dico++);
            /*dist += temp * temp;*/
            dist = L_mac(dist, temp, temp);
        }

        FOR (k=0; k<surv; k++)
        {
            IF (L_sub(dist, dist_min[k]) < 0)
            {
                FOR (l=sub(surv,1); l>k; l--)
                {
                    dist_min[l] = dist_min[l - 1];
                    move32();
                    index[l] = index[l - 1];
                    move16();
                }
                dist_min[k] = dist;
                move32();
                index[k] = i;
                move16();
                BREAK;
            }
        }
    }
    return;
}

/*-------------------------------------------------------------------*
 * sub_VQ_fx()
 *
 * Quantization of a subvector in Split-VQ of ISFs
 *-------------------------------------------------------------------*/

static Word16 sub_VQ_fx(        /* o  : selected codebook vector index      */
    Word16 *x,         /* i/o: ISF vector                          */
    const Word16 *dico,      /* i  : ISF codebook                        */
    const Word16 dim,        /* i  : codebook dimension                  */
    const Word16 dico_size,  /* i  : codebook size                       */
    Word32 *distance   /* o  : quantization error (min. distance)  */
)
{
    Word32 dist_min, dist;
    const Word16 *p_dico;
    Word16   i, j, index, temp;


    dist_min = MAX_32;
    move32();
    p_dico = dico;
    move16();

    index = 0;
    move16();
    FOR (i = 0; i < dico_size; i++)
    {
        dist = L_deposit_l(0);
        FOR  (j = 0; j < dim; j++)
        {
            /*temp = x[j] - *p_dico++;*/
            temp = sub(x[j], *p_dico++);
            /*dist += temp * temp;*/
            dist = L_mac(dist, temp, temp);
        }

        IF (L_sub(dist,dist_min) < 0)
        {
            dist_min = L_add(dist, 0);
            index = i;
            move16();
        }
    }

    *distance = dist_min;
    move32();

    /* Reading the selected vector */
    p_dico = &dico[i_mult2(index, dim)];
    FOR  (j = 0; j < dim; j++)
    {
        x[j] = *p_dico++;
        move16();
    }
    return index;
}
