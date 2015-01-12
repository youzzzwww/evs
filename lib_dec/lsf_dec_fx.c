/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static void dqlsf_CNG_fx(
    Decoder_State_fx *st_fx,           /* i/o: decoder state structure     */
    Word16 *lsf_q,                     /* o  : decoded LSFs                 */
    Word32 * p_offset_scale1,          /* i  : offset for 1st LVQ subvector */
    Word32 * p_offset_scale2,          /* i  : offset for second LVQ subvector */
    Word16 * p_no_scales               /* i  : number of scales for LVQ struct */
)
{
    Word16 indice[4];

    indice[0] = (Word16)get_next_indice_fx( st_fx, 4 );
    move16();
    indice[1] = (Word16)get_next_indice_fx( st_fx, LEN_INDICE );
    move16();
    indice[2] = (Word16)get_next_indice_fx( st_fx, LSF_BITS_CNG - 4 - LEN_INDICE );
    move16();

    deindex_lvq_cng_fx( &indice[1], lsf_q, indice[0], LSF_BITS_CNG-4, p_offset_scale1, p_offset_scale2, p_no_scales );
    Vr_add( lsf_q, &CNG_SN1_fx[indice[0]*M], lsf_q, M );
    return;
}
/*===========================================================================*/
/* FUNCTION : lsf_dec_fx()                                                   */
/*---------------------------------------------------------------------------*/
/* PURPOSE : LSF decoder                                                     */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                         */
/* _ (Struct) st_fx       : decoder static memory                            */
/* _ (Word16) L_frame     : length of the frame                              */
/* _ (Word16) coder_type  : coding type                                      */
/* _ (Word16) bwidth      : input signal bandwidth                           */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/* _ (Word16*) Aq          : LP filter coefficient    Q12                    */
/* _ (Word16*) lsf_new      : LP filter coefficient    Q(x2.56)              */
/* _ (Word16*) lsp_new      : LP filter coefficient    Q15                   */
/* _ (Word16*) lsp_mid      : LP filter coefficient    Q15                   */
/*---------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->lsf_adaptive_mean_fx  : FEC - adaptive mean LSF       */
/*                        vector for FEC  Q(x2.56)                           */
/* _ (Word16[]) st_fx->mem_AR_fx : AR memory of LSF quantizer                */
/*                  (past quantized LSFs without mean) Q(x2.56)              */
/* _ (Word16) st_fx->stab_fac_fx : LSF stability factor    Q15               */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                        */
/* _ None                                                                    */
/*===========================================================================*/
void lsf_dec_fx(
    Decoder_State_fx *st_fx,        /* i/o: State structure                             */
    const Word16 tc_subfr,          /* i  : TC subframe index                           */
    const Word16 L_frame,           /* i  : length of the frame                         */
    const Word16 coder_type,        /* i  : coding type                                 */
    const Word16 bwidth,            /* i  : input signal bandwidth                      */
    Word16 *Aq,               /* o  : quantized A(z) for 4 subframes              */
    Word16 *lsf_new,          /* o  : de-quantized LSF vector                     */
    Word16 *lsp_new,          /* o  : de-quantized LSP vector                     */
    Word16 *lsp_mid           /* o  : de-quantized mid-frame LSP vector           */
)
{
    Word16 i;
    Word16 int_fs;
    Word32 L_tmp;
    Word16 nBits = 0;
    Word16 tmp_old[M+1], tmp_new[M+1];
    Word16 enr_old = 0, enr_new = 0;
    Word16 lsf_diff;
    /* initialize */
    int_fs = INT_FS_16k_FX;
    move16();
    if( sub(L_frame,L_FRAME) == 0 )
    {
        int_fs = INT_FS_FX;
        move16();
    }

    /* Find the number of bits for LSF quantization */
    IF ( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
    {
        nBits = LSF_BITS_CNG;
        move16();
    }
    ELSE
    {
        test();
        IF ( st_fx->nelp_mode_dec_fx == 0 && st_fx->ppp_mode_dec_fx == 0 )
        {
            nBits = LSF_bits_tbl[LSF_BIT_ALLOC_IDX_fx(st_fx->core_brate_fx, coder_type)];
            move16();
        }
        ELSE IF ( sub(st_fx->nelp_mode_dec_fx,1) == 0 )
        {
            IF ( sub(coder_type,UNVOICED) == 0 )
            {
                nBits = 30;
                move16();
                if ( sub(bwidth,NB) == 0 )
                {
                    nBits = 32;
                    move16();
                }
            }
        }
        ELSE IF ( sub(st_fx->ppp_mode_dec_fx,1) == 0 )
        {
            nBits = 26;
            move16();
        }
    }

    /* LSF de-quantization */
    lsf_end_dec_fx( st_fx, 0, coder_type, st_fx->bwidth_fx, nBits, lsf_new, st_fx->mem_AR_fx,st_fx->mem_MA_fx, int_fs, st_fx->core_brate_fx,
                    &st_fx->offset_scale1_fx[0][0], &st_fx->offset_scale2_fx[0][0], &st_fx->offset_scale1_p_fx[0][0], &st_fx->offset_scale2_p_fx[0][0],
                    &st_fx->no_scales_fx[0][0], &st_fx->no_scales_p_fx[0][0], &st_fx->safety_net_fx, NULL, NULL );

    /* convert quantized LSFs to LSPs */
    lsf2lsp_fx(lsf_new, lsp_new, M ,int_fs);
    IF ( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
    {
        /* return if SID frame (conversion to A(z) done in the calling function) */
        return;
    }

    /*-------------------------------------------------------------------------------------*
     * FEC - update adaptive LSF mean vector
     *-------------------------------------------------------------------------------------*/

    FOR (i=0; i<M; i++)
    {
        L_tmp = L_mult(lsf_new[i], 10922); /*Q(x2.56+16)*/
        L_tmp = L_mac(L_tmp, st_fx->lsfoldbfi1_fx[i], 10922); /*Q(x2.56+16)*/
        L_tmp = L_mac(L_tmp, st_fx->lsfoldbfi0_fx[i], 10922); /*Q(x2.56+16)*/
        st_fx->lsf_adaptive_mean_fx[i] = round_fx(L_tmp); /*Q(x2.56)*/
    }

    test();
    test();
    IF ( ( st_fx->prev_bfi_fx && (sub(coder_type,TRANSITION) == 0) && (sub(tc_subfr,sub(L_frame,L_SUBFR)) == 0) ) )
    {
        lsf_diff = 1205;
        move16();  /*int_fs / (float)(2*(M+1)); = 470.588 -> 1205 in Q2.56 */
        if( sub(L_frame,L_FRAME) == 0 )
        {
            lsf_diff = 964;
            move16();  /*int_fs / (float)(2*(M+1)); = 376.47 -> 964 in Q2.56 */
        }
        st_fx->lsf_old_fx[0] = lsf_diff;
        move16();

        FOR ( i=1; i<M; i++ )
        {
            st_fx->lsf_old_fx[i] = add(st_fx->lsf_old_fx[i-1], lsf_diff);
            move16();
        }
        lsf2lsp_fx( st_fx->lsf_old_fx, st_fx->lsp_old_fx, M, int_fs );
    }
    /*-------------------------------------------------------------------------------------*
     * Mid-frame LSF decoding
     * LSP interpolation and conversion of LSPs to A(z)
     *-------------------------------------------------------------------------------------*/
    if(st_fx->rate_switching_reset)
    {
        /*extrapolation in case of unstable LSF convert*/
        Copy(lsp_new,st_fx->lsp_old_fx,M);
        Copy(lsf_new,st_fx->lsf_old_fx,M);
    }
    {
        /* Mid-frame LSF decoding */
        lsf_mid_dec_fx( st_fx, int_fs, st_fx->lsp_old_fx, lsp_new, coder_type, lsp_mid, st_fx->core_brate_fx, st_fx->ppp_mode_dec_fx, st_fx->nelp_mode_dec_fx
                        ,st_fx->prev_bfi_fx, &(st_fx->mid_lsf_int_fx), st_fx->safety_net_fx);
    }
    test();
    test();
    IF ( !( st_fx->prev_bfi_fx && (sub(coder_type,TRANSITION) == 0) && (sub(tc_subfr,sub(L_frame,L_SUBFR)) == 0) ) )
    {
        IF ( st_fx->prev_bfi_fx)
        {
            /* check, if LSP interpolation can be relaxed */
            E_LPC_f_lsp_a_conversion( st_fx->lsp_old_fx, tmp_old, M);
            enr_old = Enr_1_Az_fx( tmp_old, 2*L_SUBFR );

            E_LPC_f_lsp_a_conversion( lsp_new, tmp_new, M);
            enr_new = Enr_1_Az_fx( tmp_new, 2*L_SUBFR );
        }
        IF ( st_fx->prev_bfi_fx )
        {
            IF( sub(enr_new, mult_r(FL2WORD16(0.3),enr_old)) < 0 )
            {
                /* OLD CODE : if( st->safety_net == 1), replaced with a decision similar to MODE2 */
                st_fx->relax_prev_lsf_interp_fx = -1;
                move16();
                test();
                test();
                test();
                test();
                if ( sub(st_fx->clas_dec, UNVOICED_CLAS) == 0 || sub(st_fx->clas_dec, SIN_ONSET) == 0 || sub(st_fx->clas_dec, INACTIVE_CLAS) == 0 || sub(coder_type, GENERIC) == 0 || sub(coder_type, TRANSITION) == 0 )
                {
                    st_fx->relax_prev_lsf_interp_fx = 1;
                    move16();
                }
            }
        }
    }
    test();
    IF( sub(st_fx->last_core_fx, HQ_CORE)==0 && sub(st_fx->core_fx,ACELP_CORE)==0 )
    {
        /* update old LSPs/LSFs in case of HQ->ACELP core switching */
        Copy( lsp_mid, st_fx->lsp_old_fx, M );
        lsp2lsf_fx( lsp_mid, st_fx->lsf_old_fx, M, int_fs );
    }

    /* LSP interpolation and conversion of LSPs to A(z) */
    int_lsp4_fx( L_frame, st_fx->lsp_old_fx, lsp_mid, lsp_new, Aq, M, 0, st_fx->relax_prev_lsf_interp_fx );

    /*------------------------------------------------------------------*
     * Check LSF stability (distance between old LSFs and current LSFs)
     *------------------------------------------------------------------*/

    st_fx->stab_fac_fx = lsf_stab_fx( lsf_new, st_fx->lsf_old_fx, 0, st_fx->L_frame_fx ); /*Q15*/

    return;
}
/*========================================================================*/
/* FUNCTION : lsf_end_dec_fx()                        */
/*------------------------------------------------------------------------*/
/* PURPOSE : De-quantize LSF vector                      */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                            */
/* _ (Word16) coder_type  : coding type                    */
/* _ (Word16) bwidth    : input signal bandwidth              */
/* _ (Word16) nBits      : number of bits used for ISF quantization      */
/* _ (Word32*) grid       : Table of 100 grid points for evaluating      */
/*                  Chebyshev polynomials          Q31        */
/* _ (Word16) int_fs    : sampling frequency                  */
/* _ (Word32) core_brate  : Coding Bit Rate                     */
/* _ (Word32*) p_offset_scale1    : offsets for LSF LVQ structure 1st   */
/*                    8-dim subvector          Q0  */
/* _ (Word32*) p_offset_scale2    : offsets for LSF LVQ structure 2nd   */
/*                    8-dim subvector          Q0  */
/* _ (Word32*) p_offset_scale1_p   : offsets for LSF LVQ structure, pred.*/
/*                    case, 1st 8-dim subvector      Q0  */
/* _ (Word32*) p_offset_scale2_p  : offsets for LSF LVQ structure,      */
/*                    pred. case, 2nd 8-dim subvector Q0  */
/* _ (Word16*)  p_no_scales        : LSF LVQ structure           Q0  */
/* _ (Word16*)  p_no_scales_p    : LSF LVQ structure            Q0  */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                          */
/* _ (Word16*) mem_AR  : quantizer memory for AR model        Q(x2.56)    */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                            */
/* _ (Word16*) qlsf    : quantized LSFs in the cosine domain Q(x2.56)    */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                            */
/* _ None                                  */
/*========================================================================*/

void lsf_end_dec_fx(
    Decoder_State_fx * st,              /* i/o: decoder state structure                 */
    Word16 mode2_flag,
    const Word16 coder_type_org,      /* i  : coding type                             */
    const Word16 bwidth,              /* i  : input signal bandwidth                  */
    const Word16 nBits_in,            /* i  : number of bits used for ISF quantization*/
    Word16 *qlsf,               /* o  : quantized LSFs in the cosine domain     */
    Word16 *mem_AR,             /* i/o: quantizer memory for AR model           */
    Word16 *mem_MA,             /* i/o: quantizer memory for MA model           */
    const Word32 int_fs,              /* i  : sampling frequency                      */
    Word32  core_brate,         /* i  : Coding Bit Rate                         */
    Word32 *p_offset_scale1,
    Word32 *p_offset_scale2,
    Word32 *p_offset_scale1_p,
    Word32 *p_offset_scale2_p,
    Word16 *p_no_scales,
    Word16 *p_no_scales_p,
    Word16 *safe_net,
    Word16 *lpc_param,
    Word16 * nb_indices
)
{
    Word16 pred0[M];               /* Prediction for the safety-net quantizer (usually mean)*/
    Word16 pred1[M], pred2[M];     /* Prediction for the predictive quantizer*/
    Word16 stages0;                /* Amount of stages used by safety-net quantizer*/
    Word16 stages1;                /* Amount of stages used by predictive quantizer*/
    Word16 levels0[MAX_VQ_STAGES]; /* Sizes of different codebook stages for safety-net quantizer*/
    Word16 levels1[MAX_VQ_STAGES]; /* Sizes of different codebook stages for predictive quantizer*/
    Word16 i;
    Word16 TCQIdx[M/2+4];
    Word16 bits0[MAX_VQ_STAGES], bits1[MAX_VQ_STAGES];
    Word16   cumleft;
    Word16 lindice[MAX_VQ_STAGES+3]; /* Predictor selector needs 1 bit and the LVQ indice uses 3 shorts */
    Word16 mode_lvq, mode_lvq_p;
    Word16 safety_net, predmode, stages, *levels;
    const Word16 *Bit_alloc1 = NULL, *bits;
    Word16 num_bits;
    Word16 * p_lpc_param;

    Word16 nr_ind;
    Word16 nBits;

    Word16 coder_type;

    nBits = nBits_in;
    move16();

    test();
    test();
    IF((sub(coder_type_org, GENERIC) == 0) && (L_sub(int_fs, INT_FS_16k) == 0) && (mode2_flag == 0))
    {
        coder_type = (Word16)get_next_indice_fx( st, 1 );
        coder_type = add(coder_type,2);
        if (sub(coder_type, GENERIC) == 0)
        {
            nBits = sub(nBits,1);
        }
    }
    ELSE
    {
        coder_type = coder_type_org;
        move16();
    }

    /*--------------------------------------------------------------------------------*
     * LSF de-quantization of SID frames
     *--------------------------------------------------------------------------------*/

    IF ( core_brate == SID_2k40 )
    {
        dqlsf_CNG_fx( st, qlsf, p_offset_scale1, p_offset_scale2, p_no_scales );
        sort_fx( qlsf, 0, M-1);
        reorder_lsf_fx( qlsf, MODE1_LSF_GAP_FX, M, int_fs );

        return;
    }


    predmode = find_pred_mode(coder_type, bwidth, int_fs, &mode_lvq, &mode_lvq_p, st->total_brate_fx);
    /*----------------------------------------------------------------*
     * Calculate number of stages and levels for each stage based on the allowed bit allocation
     * (subtract one bit for LSF predictor selection)
     *----------------------------------------------------------------*/
    lsf_allocate_fx( sub(nBits,shr(predmode,1)), mode_lvq, mode_lvq_p, &stages0, &stages1, levels0, levels1,
                     bits0, bits1);


    /*--------------------------------------------------------------------------*
     * Select safety_net or predictive mode
     *--------------------------------------------------------------------------*/
    p_lpc_param = lpc_param;

    nr_ind = 0;
    move16();
    IF ( predmode == 0 )
    {
        safety_net = 1;
        move16();
    }
    ELSE IF ( predmode == 1 )
    {
        safety_net = 0;
        move16();
    }
    ELSE
    {
        IF (sub(mode2_flag, 1) == 0)
        {
            nr_ind = add(nr_ind,1);
            /* read from param_lpc */
            safety_net = p_lpc_param[0];
            move16();
            p_lpc_param++;
        }
        ELSE
        {
            safety_net = (Word16)get_next_indice_fx( st, 1 );
        }
    }

    *safe_net = safety_net;
    move16();

    /*--------------------------------------------------------------------------*
     * Read indices from array
     *--------------------------------------------------------------------------*/

    IF ( safety_net )
    {
        stages = stages0;
        move16();
        levels = levels0;
        move16();
        bits = bits0;
        move16();
    }
    ELSE
    {
        stages = stages1;
        move16();
        levels = levels1;
        move16();
        bits = bits1;
        move16();
    }

    IF (sub(mode2_flag, 1) == 0)
    {
        /* VOICED_WB@16kHz */
        test();
        IF ( L_sub(int_fs, INT_FS_16k) == 0 && sub(coder_type, VOICED) == 0 )
        {
            *nb_indices = 10;
            move16();
            FOR(i=0; i<*nb_indices; i++)
            {
                TCQIdx[i] = (Word16)lpc_param[i];
                move16();
            }
        }
        ELSE
        {
            FOR ( i=0; i<stages-1; i++ )
            {
                num_bits = bits[i];
                move16();
                lindice[i+1] = *p_lpc_param++;
                move16();
                nr_ind = add(nr_ind,1);
            }

            cumleft = levels[stages-1];
            move16();
            WHILE ( cumleft > 0 )
            {
                IF ( sub(cumleft, LEN_INDICE) >0 )
                {
                    cumleft = sub(cumleft, LEN_INDICE);
                    num_bits = LEN_INDICE;
                    move16();
                }
                ELSE
                {
                    num_bits = (Word16)cumleft;
                    move16();
                    cumleft = 0;
                    move16();
                }

                lindice[i+1] = *p_lpc_param++;
                move16();
                nr_ind = add(nr_ind,1);
                i = add(i,1);
            }
            *nb_indices = nr_ind;
            move16();
        }
    }
    ELSE
    {
        /* VOICED_WB@16kHz */
        test();
        IF ( L_sub(int_fs, INT_FS_16k)== 0 && sub(coder_type, VOICED) == 0 )
        {
            Bit_alloc1 = &BC_TCVQ_BIT_ALLOC_40B[1];
            TCQIdx[0] = safety_net;
            move16();
            FOR ( i=0; i<(M/2)+3; i++ )
            {
                TCQIdx[i+1] = (Word16)get_next_indice_fx( st, Bit_alloc1[i] );
            }
        }
        ELSE
        {
            FOR ( i=0; i<stages-1; i++ )
            {
                num_bits = bits[i];
                move16();
                lindice[i+1] = (Word16)get_next_indice_fx( st, num_bits );
            }

            cumleft = levels[sub(stages,1)];
            WHILE ( cumleft > 0 )
            {
                IF ( sub(cumleft, LEN_INDICE) > 0 )
                {
                    cumleft = sub(cumleft, LEN_INDICE);
                    num_bits = LEN_INDICE;
                    move16();
                }
                ELSE
                {
                    num_bits = (Word16)cumleft;
                    move16();
                    cumleft = 0;
                    move16();
                }

                lindice[i+1] = (Word16)get_next_indice_fx( st, num_bits );
                i = add(i,1);
            }
        }
    }


    /*------------------------------------------------------------------------------------------*
     * De-quantize LSF vector
     *------------------------------------------------------------------------------------------*/

    /* VOICED_WB@16kHz */
    test();
    IF ( L_sub(int_fs, INT_FS_16k) == 0 && sub(coder_type, VOICED) == 0 )
    {
        /* BC-TCVQ decoder */
        safety_net = qlsf_ARSN_tcvq_Dec_16k_fx ( qlsf, TCQIdx, nBits-1 );

        /* Update mem_MA */
        Copy( qlsf, mem_MA, M );

        IF (safety_net)
        {
            Copy(ModeMeans_fx[mode_lvq], pred0, M);
        }
        ELSE
        {
            FOR(i = 0; i < M; i++)
            {
                pred0[i] = add(ModeMeans_fx[mode_lvq][i], mult(Predictors_fx[mode_lvq_p][i],(sub(mem_AR[i], ModeMeans_fx[mode_lvq][i])))); /* Q(x2.56)*/
            }
        }
        Vr_add( qlsf, pred0, qlsf, M );
    }
    ELSE
    {

        /* Safety-net */
        Copy( ModeMeans_fx[mode_lvq], pred0, M );
        /* for mem_MA update */
        FOR (i=0; i<M; i++)
        {
            pred1[i] = add(pred0[i], mult_r(MU_MA_FX, mem_MA[i]));
        }

        IF ( safety_net )
        {
            /* LVQ */
            vq_dec_lvq_fx( 1, qlsf, &lindice[1], stages0, M, mode_lvq, levels0[stages0-1],
            p_offset_scale1, p_offset_scale2, p_offset_scale1_p, p_offset_scale2_p,
            p_no_scales, p_no_scales_p );

            Vr_add( qlsf, pred0, qlsf, M );
            Vr_subt( qlsf, pred1, mem_MA, M);
        }
        ELSE
        {
            vq_dec_lvq_fx( 0, qlsf, &lindice[1], stages1, M, mode_lvq_p, levels1[stages1-1],
            p_offset_scale1, p_offset_scale2, p_offset_scale1_p, p_offset_scale2_p,
            p_no_scales, p_no_scales_p );
            IF (sub(predmode, 1) == 0) /* MA only */
            {
                Copy(qlsf, mem_MA, M);
                Vr_add( qlsf, pred1, qlsf, M );
            }
            ELSE
            {
                /* AR  */
                FOR ( i=0; i<M; i++ )
                {
                    pred2[i] = add(pred0[i], mult(Predictors_fx[mode_lvq_p][i], sub(mem_AR[i], pred0[i])));
                }
                Vr_add( qlsf, pred2, qlsf, M );
                Vr_subt( qlsf, pred1, mem_MA, M );
            }
        }
    }

    /*--------------------------------------------------------------------------*
     * Sort the quantized vector
     * Verify stability
     * Update AR-predictor memory
     *--------------------------------------------------------------------------*/

    /* Sort the quantized vector */
    sort_fx( qlsf, 0, M-1 );

    /* Verify stability */
    reorder_lsf_fx( qlsf, MODE1_LSF_GAP_FX, M, int_fs );
    /* Update predictor memory */
    Copy( qlsf, mem_AR, M );

    st->mode_lvq = mode_lvq;
    move16();

    return;
}


/*========================================================================*/
/* FUNCTION : lsf_mid_dec_fx()                        */
/*------------------------------------------------------------------------*/
/* PURPOSE : Decode mid-frame LSFs                      */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                            */
/* _ (Word16) coder_type  : Coder type                    */
/* _ (Word16) int_fs    : internal (ACELP) sampling frequency          */
/* _ (Word32) core_brate  : core bitrate                         */
/* _ (Word32)  ppp_mode    : PPP mode                       */
/* _ (Word32)  nelp_mode  : NELP mode                      */
/* _ (Word16[]) qlsp0    : quantized LSPs from frame beginning     Q15   */
/* _ (Word16[]) qlsp1    : quantized LSPs from frame end          Q15   */
/* _ (Word16) prev_bfi,                                              */
/* _ (Word16 *)mid_lsf_int,                                           */
/* _ (Word16) safety_net                                               */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                          */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                            */
/* _ (Word16[]) qlsp    : quantized LSPs                        Q15    */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                            */
/* _ None                                  */
/*========================================================================*/
void lsf_mid_dec_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    const Word16 int_fs,     /* i  : internal (ACELP) sampling frequency  */
    Word16 qlsp0[],    /* i  : quantized LSPs from frame beginning  Q15*/
    Word16 qlsp1[],    /* i  : quantized LSPs from frame endSQ15*/
    Word16 coder_type, /* i  : Coder type */
    Word16 qlsp[],     /* o  : quantized LSPs    Q15*/
    const  Word32 core_brate, /* i  : core bitrate */
    Word16 ppp_mode,
    Word16 nelp_mode,
    Word16 prev_bfi,
    Word16 *mid_lsf_int,
    Word16 safety_net
)
{
    Word16 j, idx;
    Word16 nb_bits;
    Word16 qlsf0[M], qlsf1[M], qlsf[M];
    Word32 L_tmp;
    Word16 bad_spacing;
    const Word16 *ratio = NULL;
    bad_spacing = 0;
    move16();


    /* Convert LSPs to LSFs */
    lsp2lsf_fx( qlsp0, qlsf0, M, int_fs);
    lsp2lsf_fx( qlsp1, qlsf1, M, int_fs);

    /* Codebook selection */
    IF ( sub(ppp_mode,1) == 0 )
    {
        nb_bits = 1;
        move16();
        ratio = &tbl_mid_voi_wb_1b_fx[0];
    }
    ELSE IF ( sub(nelp_mode,1) == 0 )
    {
        nb_bits = 4;
        move16();
        ratio = &tbl_mid_unv_wb_4b_fx[0];
    }
    ELSE
    {
        nb_bits = mid_LSF_bits_tbl[LSF_BIT_ALLOC_IDX_fx(core_brate, coder_type)];
        move16();

        /* codebook selection */

        IF ( sub(coder_type,VOICED) == 0 )
        {
            SWITCH ( nb_bits )
            {
            case 6:
                {
                    ratio = tbl_mid_voi_wb_6b_fx;
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_voi_wb_5b_fx;
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_voi_wb_4b_fx;
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_voi_wb_3b_fx;
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_voi_wb_2b_fx;
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_voi_wb_5b_fx;
                    BREAK;
                }
            }
        }
        ELSE IF ( coder_type == UNVOICED )
        {
            SWITCH ( nb_bits )
            {
            case 6:
                {
                    ratio = tbl_mid_unv_wb_6b_fx;
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_unv_wb_5b_fx;
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_unv_wb_4b_fx;
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_unv_wb_3b_fx;
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_unv_wb_2b_fx;
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_unv_wb_5b_fx;
                    BREAK;
                }
            }
        }
        ELSE
        {
            /* GENERIC, TRANSITION, AUDIO and INACTIVE */
            SWITCH ( nb_bits )
            {
            case 6:
                {
                    ratio = tbl_mid_gen_wb_6b_fx;
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_gen_wb_5b_fx;
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_gen_wb_4b_fx;
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_gen_wb_3b_fx;
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_gen_wb_2b_fx;
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_gen_wb_5b_fx;
                    BREAK;
                }
            }
        }
    }

    /* Retrieve mid-frame LSF index */
    idx = (Word16)get_next_indice_fx( st_fx, nb_bits );

    /* Calculation of mid-LSF vector */
    FOR (j=0; j<M; j++)
    {
        L_tmp = L_mult(sub(0x2000, ratio[idx*M+j]), qlsf0[j]); /*Q(x2.56+13+1)->Q(x2.56+14)*/
        L_tmp = L_mac(L_tmp, ratio[idx*M+j], qlsf1[j]); /*Q(x2.56+14)*/
        qlsf[j] = round_fx(L_shl(L_tmp,2)); /*Q(x2.56)*/
    }

    /* check for incorrect LSF ordering */
    IF ( sub(*mid_lsf_int, 1) == 0 )
    {
        FOR (j=1; j<M; j++)
        {
            IF ( sub(qlsf[j], qlsf[j-1]) <0 )
            {
                bad_spacing = 1;
                move16();
                BREAK;
            }
        }
    }

    /* Redo mid-LSF interpolation with 0.4 in case of LSF instability */
    test();
    test();
    IF ( prev_bfi || ( sub(*mid_lsf_int, 1) == 0 && bad_spacing ) )
    {
        FOR (j=0; j<M; j++)
        {
            /* redo mid-LSF interpolation with 0.4 */
            qlsf[j] = add(mult_r(13107, qlsf0[j]), mult_r(19661, qlsf1[j])); /* Q15 +x2.56 -Q15 13107 = 0.4(Q15), 19661 = 0.6 (Q15)*/ move16();

            /* ensure correct ordering of LSF indices */
            test();
            test();
            IF ( j > 0 && sub(j, M) <0 && sub(qlsf[j], add( qlsf[j-1], LSF_GAP_MID_FX))<0  )
            {
                qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
                move16();
            }
        }
    }
    ELSE
    {
        /* otherwise, use regular LSF spacing and ordering as in the encoder */
        FOR (j=0; j<M; j++)
        {
            test();
            test();
            IF ( j > 0 && sub(j, M) < 0 && sub(qlsf[j], add( qlsf[j-1],LSF_GAP_MID_FX))<0 )
            {
                qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
                move16();
            }
        }
    }

    if ( prev_bfi )
    {
        /* continue redoing mid-LSF interpolation with 0.4 in order not to propagate the error */
        *mid_lsf_int = 1;
        move16();
    }

    if ( safety_net )
    {
        /* safety-net encountered -> stop redoing mid-LSF interpolation with 0.4 */
        *mid_lsf_int = 0;
        move16();
    }

    reorder_lsf_fx( qlsf, LSF_GAP_MID_FX, M, int_fs );

    /* convert back to LSPs */
    lsf2lsp_fx( qlsf, qlsp, M, int_fs);

    return;
}
