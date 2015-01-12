/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"       /* Compilation switches                   */
#include "rom_enc_fx.h"
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "prot_fx.h"



/*-----------------------------------------------------------------*
* Local constants
*-----------------------------------------------------------------*/
#define MIN_LOG_FX          0
#define MIN_LOG_VAL_FX   -15360    /* -60.0f in Q8 */
#define MSVQ_MAXCNT      3000
#define MAXINT32         MAX_32

/*---------------------------------------------------------------------*
* Local functions
*---------------------------------------------------------------------*/

static void lsfq_CNG_fx( Encoder_State_fx *st_fx, const Word16 *lsf, const Word16 *wghts,  Word16 *qlsf, Word32 *p_offset_scale1, Word32 * p_offset_scale2,
                         Word16 * p_no_scales );

static Word32 vq_lvq_lsf_enc( Word16 pred_flag, Word16 mode, Word16 u[], Word16 * levels, Word16 stages, Word16 w[], Word16 Idx[], const Word16 * lsf,
                              const Word16 * pred, Word32  p_offset_scale1[][MAX_NO_SCALES+1], Word32  p_offset_scale2[][MAX_NO_SCALES+1],
                              Word16 p_no_scales[][2], Word16 *resq, Word16 * lsfq );

static void lsf_mid_enc_fx( Encoder_State_fx *st_fx,const Word16 int_fs, const Word16 qisp0[], const Word16 qisp1[], Word16 isp[], const Word16 coder_type,
                            const Word16 bwidth, const Word32 core_brate, Word32 Bin_Ener_old[], Word32 Bin_Ener[], Word16 Q_ener, Word16 ppp_mode, Word16 nelp_mode );


/*===========================================================================*/
/* FUNCTION : lsf_enc_fx()                                                   */
/*---------------------------------------------------------------------------*/
/* PURPOSE : Quantization of LSF vector                                      */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                         */
/* _ (Word16) L_frame       : length of the frame                            */
/* _ (Word16) coder_type    : coding type                                    */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/* _ (Word16*) Aq                : LP filter coefficient    Q12              */
/* _ (Word16*) lsf_ne     w      : LP filter coefficient    Q(x2.56)         */
/* _ (Word16) st_fx->stab_fac_fx : LSF stability factor     Q15              */
/*---------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                  */
/* _ (Struct)  st_fx        : state structure                                */
/* _ (Word16*) lsp_new      : LP filter coefficient    Q15                   */
/* _ (Word16*) lsp_mid      : LP filter coefficient    Q15                   */
/* _ (Word16[]) st_fx->mem_AR_fx : AR memory of LSF quantizer                */
/*                  (past quantized LSFs without mean)    x2.56              */
/* _ (Word16[]) st_fx->clip_var_fx : pitch gain clipping memory         x2.56*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                        */
/* _ None                                                                    */
/*===========================================================================*/
void lsf_enc_fx(
    Encoder_State_fx *st_fx,  /* i/o: state structure                   */
    const Word16 L_frame,     /* i  : length of the frame               */
    const Word16 coder_type,  /* i  : coding type                       */
    Word16 *lsf_new,          /* o  : quantized LSF vector              */
    Word16 *lsp_new,          /* i/o: LSP vector to quantize/quantized  */
    Word16 *lsp_mid,          /* i/o  : mid-frame LSP vector            */
    Word16 *Aq,               /* o  : quantized A(z) for 4 subframes    */
    Word16 *stab_fac,         /* o  : LSF stability factor              */
    const Word16 Nb_ACELP_frames,
    const  Word16 Q_new
)
{
    Word16 nBits = 0;
    Word16 int_fs;
    Word16 force_sf = 0;
    Word16 fec_lsf[M], stab;
    Word16 i;
    Word32 L_tmp;

    /* initialize */
    int_fs = INT_FS_16k_FX;
    move16();
    if( sub(L_frame, L_FRAME) == 0 )
    {
        int_fs = INT_FS_FX;
        move16();
    }

    /* convert LSPs to LSFs */
    lsp2lsf_fx( lsp_new, lsf_new, M, int_fs);

    /* check resonance for pitch clipping algorithm */
    gp_clip_test_lsf_fx( lsf_new, st_fx->clip_var_fx, 0 );

    /* Find the number of bits for LSF quantization */
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        nBits = LSF_BITS_CNG;
        move16();
    }
    ELSE
    {
        test();
        IF ( (st_fx->nelp_mode_fx == 0) && (st_fx->ppp_mode_fx == 0) )
        {
            nBits = LSF_bits_tbl[LSF_BIT_ALLOC_IDX_fx(st_fx->core_brate_fx, coder_type)];
            move16();
        }
        ELSE IF ( sub(st_fx->nelp_mode_fx, 1) == 0 )
        {
            nBits = 30;
            move16();

            if ( sub(st_fx->bwidth_fx,NB) == 0 )
            {
                nBits = 32;
                move16();
            }
        }
        ELSE IF ( sub(st_fx->ppp_mode_fx, 1) == 0 )
        {
            nBits = 26;
            move16();
        }
    }

    /* first three ACELP frames after an HQ frame shall be processed only with safety-net quantizer */
    if( (sub(Nb_ACELP_frames, 3) < 0) )
    {
        force_sf = 1;
        move16();
    }

    /* in case of unstable filter in decoder FEC, choose safety-net to help FEC */
    IF ( sub(st_fx->next_force_safety_net_fx ,1) == 0 )
    {
        force_sf = 1;
        move16();
        st_fx->next_force_safety_net_fx = 0;
        move16();
    }

    /*-------------------------------------------------------------------------------------*
    * Frame end LSF quantization
    *-------------------------------------------------------------------------------------*/

    lsf_end_enc_fx( st_fx, lsf_new, lsf_new, st_fx->mem_AR_fx, st_fx->mem_MA_fx, nBits, coder_type, st_fx->bwidth_fx,
                    st_fx->Bin_E_fx, Q_new+QSCALE-2, int_fs, st_fx->core_brate_fx, &st_fx->streaklimit_fx, &st_fx->pstreaklen_fx,
                    force_sf, 0, 0, NULL, NULL, NULL, st_fx->coder_type_raw_fx );

    /* convert quantized LSFs back to LSPs */
    lsf2lsp_fx( lsf_new, lsp_new, M, int_fs);

    test();
    IF ( sub(st_fx->last_core_fx, HQ_CORE) == 0 && sub(st_fx->core_fx,ACELP_CORE) == 0 )
    {
        /* don't use old LSF values if this is the first ACELP frame after HQ frames */
        Copy( lsf_new, st_fx->lsf_old_fx, M );
    }
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        /* return if SID frame (conversion to A(z) done in the calling function) */
        return;
    }

    /*-------------------------------------------------------------------------------------*
    * FEC - enforce safety-net in the next frame in case of unstable filter
    *-------------------------------------------------------------------------------------*/

    IF( sub(st_fx->last_L_frame_fx, st_fx->L_frame_fx) != 0 )
    {
        /* FEC - in case of core switching, use old LSFs */
        Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi1_fx, M );
        Copy( st_fx->lsf_old_fx, st_fx->lsfoldbfi0_fx, M );
        Copy( st_fx->lsf_old_fx, st_fx->lsf_adaptive_mean_fx, M );
    }

    FEC_lsf_estim_enc_fx( st_fx, st_fx->L_frame_fx, fec_lsf );

    /* in case of FEC in decoder - calculate LSF stability */
    stab = lsf_stab_fx( lsf_new, fec_lsf, 0, st_fx->L_frame_fx );

    test();
    test();
    test();
    /* If decoder FEC frame may be unstable force safety-net usage */
    IF ( (sub(st_fx->L_frame_fx, L_FRAME16k)==0) && (sub(stab, STAB_FAC_LIMIT_FX)< 0) && (sub(coder_type, GENERIC) == 0 ))
    {
        st_fx->next_force_safety_net_fx = 1;
        move16();
    }
    ELSE IF((sub(stab, STAB_FAC_LIMIT_FX) < 0) && (sub(st_fx->clas_fx, VOICED_CLAS)==0))
    {
        st_fx->next_force_safety_net_fx = 1;
        move16();
    }


    /* FEC - update adaptive LSF mean vector */
    FOR (i=0; i<M; i++)
    {
        L_tmp = L_mult(lsf_new[i], 10922); /*Q(x2.56+16)*/
        L_tmp = L_mac(L_tmp, st_fx->lsfoldbfi1_fx[i], 10922); /*Q(x2.56+16)*/
        L_tmp = L_mac(L_tmp, st_fx->lsfoldbfi0_fx[i], 10922); /*Q(x2.56+16)*/
        st_fx->lsf_adaptive_mean_fx[i] = round_fx(L_tmp); /*Q(x2.56)*/
    }

    /* FEC - update LSF memories */
    Copy( st_fx->lsfoldbfi0_fx, st_fx->lsfoldbfi1_fx, M );
    Copy( lsf_new, st_fx->lsfoldbfi0_fx, M );


    /*-------------------------------------------------------------------------------------*
    * Mid-frame LSF encoding
    * LSP interpolation and conversion of LSPs to A(z)
    *-------------------------------------------------------------------------------------*/
    if(st_fx->rate_switching_reset)
    {
        /*extrapolation in case of unstable LSF convert*/
        Copy( lsp_new, st_fx->lsp_old_fx, M );
        Copy( lsf_new, st_fx->lsf_old_fx, M );
    }
    /* Mid-frame LSF encoding */
    lsf_mid_enc_fx( st_fx, int_fs, st_fx->lsp_old_fx, lsp_new, lsp_mid, coder_type, st_fx->bwidth_fx, st_fx->core_brate_fx, st_fx->Bin_E_old_fx, st_fx->Bin_E_fx, Q_new+QSCALE-2, st_fx->ppp_mode_fx, st_fx->nelp_mode_fx);

    test();
    IF ( sub(st_fx->last_core_fx,HQ_CORE) == 0 && sub(st_fx->core_fx,ACELP_CORE) == 0 )
    {
        /* don't use old LSP/LSF values if this is the first ACELP frame after HQ frames */
        Copy( lsp_mid, st_fx->lsp_old_fx, M );
        lsp2lsf_fx( lsp_mid, st_fx->lsf_old_fx, M, int_fs );
    }

    /* LSP interpolation and conversion of LSPs to A(z) */
    int_lsp4_fx( L_frame, st_fx->lsp_old_fx, lsp_mid, lsp_new, Aq,  M, 0, 0 ); /* prev_Aq added !! AV. */

    /*------------------------------------------------------------------*
    * Check LSF stability (distance between old LSFs and current LSFs)
    *------------------------------------------------------------------*/

    *stab_fac = lsf_stab_fx( lsf_new, st_fx->lsf_old_fx, 0, st_fx->L_frame_fx );

    return;
}


/*-------------------------------------------------------------------*
* lsfq_CNG_fx()
*
* LSF quantizer for SID frames (uses 29 bits, 4 for VQ, 25 for LVQ)
*-------------------------------------------------------------------*/

static void lsfq_CNG_fx(
    Encoder_State_fx *st_fx,
    const Word16 *lsf,    /*x2.56  unquantized LSF vector */
    const Word16 *wghts,  /*Q10    LSF weights            */
    Word16 *qlsf,         /*x2.56  quantized LSF vecotor  */
    Word32 *p_offset_scale1,
    Word32 *p_offset_scale2,
    Word16 *p_no_scales
)
{
    Word16 i, j, idx_cv, idx_lvq[3];
    Word32 min_dist, dist;
    Word16 dd[M], ddq[M];
    const Word16 *p_cb;
    Word16 first_cb, last_cb;
    Word16 idx_lead_cng[2], idx_scale_cng[2];
    Word16 tmp;

    idx_cv = 0;
    move16();

    /* quantize first stage with 4 bits */
    IF ( sub(lsf[M-1],WB_LIMIT_LSF_FX) > 0 )
    {
        p_cb = &CNG_SN1_fx[0];
        move16();
        first_cb = 0;
        move16();
        last_cb = 6;
        move16();
    }
    ELSE
    {
        p_cb = &CNG_SN1_fx[6*M];
        move16();
        first_cb = 6;
        move16();
        last_cb = M;
        move16();
    }


    min_dist = L_add(MAXINT32, 0);
    FOR ( i = first_cb; i < last_cb; i++ )
    {
        tmp = sub(*p_cb,shl(lsf[0],1)); /*x2.56 */
        dist = Mult_32_16(L_mult0(wghts[0], *p_cb),tmp); /*Q8 + x2.56 -Q15 + x2.56 = Q-7 + x2.56+x.256 */
        p_cb++;
        FOR (j=1; j<M; j++)
        {
            tmp = sub(*p_cb,lsf[j]);
            tmp = sub(tmp,lsf[j]);

            dist = L_add(dist, Mult_32_16(L_mult0(wghts[j], *p_cb),tmp));
            p_cb++;
        }
        IF ( L_sub(dist,min_dist) < 0 )
        {
            min_dist = dist;
            move16();/*Q-4 */
            idx_cv = i;
            move16();
        }
    }

    /* calculate difference */
    FOR( i = 0; i < M; i++ )
    {
        dd[i] = sub(lsf[i],CNG_SN1_fx[idx_cv*M+i]); /*x2.56 */  move16();
    }

    /* quantize the difference with LVQ */
    mslvq_cng_fx( idx_cv, dd, qlsf, ddq, idx_lead_cng, idx_scale_cng, wghts, p_no_scales );

    index_lvq_fx( ddq, idx_lead_cng, idx_scale_cng, START_CNG + idx_cv, idx_lvq,
                  p_offset_scale1, p_offset_scale2, p_no_scales );
    Vr_add( qlsf, &CNG_SN1_fx[idx_cv*M], qlsf, M );

    /* write the VQ index to the bitstream */
    push_indice_fx( st_fx, IND_ISF_0_0, idx_cv, 4 );

    /* write the LVQ index to the bitstream */
    push_indice_fx( st_fx, IND_ISF_0_1, idx_lvq[0], LEN_INDICE );
    push_indice_fx( st_fx, IND_ISF_0_1, idx_lvq[1], LSF_BITS_CNG - 4 - LEN_INDICE );

    return;
}

/*-------------------------------------------------------------------*
* qlsf_Mode_Select_fx()
*
* Mode selection for LSF quantizer
*-------------------------------------------------------------------*/


static Word16 qlsf_Mode_Select_fx(
    Word16 *w,               /* i : weighting vector           Q8 */
    Word16 *pred1,           /* i : prediction vector       x2.56 */
    Word16 streaklimit,      /* i : predictive streak limit   Q15 */
    Word32 op_loop_thr       /* i : Open-loop Threshold           */
)
{
    Word16 pred_pow2[M];
    Word32 temp32, En = 0;
    Word16 safety_net;
    Word16 i, cs, cl;

    /* calculate the prediction residual */
    cl = 0;
    move16();
    FOR (i = 0; i < M; i ++)
    {
        cl = s_max(cl, abs_s(pred1[i]));
    }
    cs = norm_s(cl);
    En = 0;
    move16();
    FOR (i = 0; i < M; i ++)
    {
        pred_pow2[i] = shl(pred1[i], cs);
        move16();
        En = L_mac(En, mult(pred_pow2[i], shl(w[i],2) ), pred_pow2[i]); /* 2.56*2.56 at Q-4 */
    }

    cs = shl(cs, 1);
    En = L_shr(En, cs);
    temp32 = Mult_32_16(op_loop_thr, streaklimit);

    /* choose the mode */
    IF ( L_sub(En, temp32) > 0)
    {
        /* Safety-net */
        safety_net = 1;
        move16();
    }
    ELSE
    {
        /* Predictive */
        safety_net = 0;
        move16();
    }
    return safety_net;
}


/*========================================================================*/
/* FUNCTION : lsf_end_enc_fx()                                            */
/*------------------------------------------------------------------------*/
/* PURPOSE : Quantization of LSF parameters                               */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                      */
/* _ (Word16*) lsf        : LSF in the frequency domain (0..6400)   x2.56 */
/* _ (Word16) coder_type  : coding type                                   */
/* _ (Word16) bwidth      : input signal bandwidth                        */
/* _ (Word16) nBits       : number of bits used for ISF quantization      */
/* _ (Word16*) stable_isp : most recent stable ISP (can be                */
/*                          removed after passing to LSF)             Q15 */
/* _ (Word16*) stable_lsp : most recent stable LSP                    Q15 */
/* _ (Word32*) grid       : Table of 100 grid points for evaluating       */
/*                          Chebyshev polynomials                     Q31 */
/* _ (Word16) int_fs      : sampling frequency                            */
/* _ (Word32) core_brate  : Coding Bit Rate                               */
/* _ (Word16) force_sf    : Force safety-net usage if possible            */
/* _ (Word32*) Bin_Ener   : FFT Bin energy 128 *2 sets             Q_ener */
/* _ (Word16) Q_ener      : Q format of Bin_Ener                          */
/* _ (Word32*) offset_scale1: offsets for LSF LVQ structure 1st           */
/*                            8-dim subvector                          Q0 */
/* _ (Word32*) offset_scale2: offsets for LSF LVQ structure 2nd           */
/*                            8-dim subvector                          Q0 */
/* _ (Word32*) offset_scale1_p: offsets for LSF LVQ structure, pred.      */
/*                              case, 1st 8-dim subvector              Q0 */
/* _ (Word32*) offset_scale2_p: offsets for LSF LVQ structure,            */
/*                              pred. case, 2nd 8-dim subvector        Q0 */
/* _ (Word16*) no_scales      : LSF LVQ structure                      Q0 */
/* _ (Word16*) no_scales_p    : LSF LVQ structure                      Q0 */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                               */
/* _ (Word16*) mem_AR  : quantizer memory for AR model              x2.56 */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                     */
/* _ (Word16*) qlsf    : quantized LSFs in the cosine domain        x2.56 */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                     */
/* _ None                                                                 */
/*========================================================================*/
void lsf_end_enc_fx(
    Encoder_State_fx *st,  /* i/o: encoder state structure                                */
    const Word16 *lsf,     /* i  : LSF in the frequency domain (0..6400)                  */
    Word16 *qlsf,          /* o  : quantized LSF                                          */
    Word16 *mem_AR,        /* i/o: quantizer memory for AR model                          */
    Word16 *mem_MA,        /* i/o: quantizer memory for MA model                          */
    const Word16 nBits_in, /* i  : number of bits to spend on ISF quantization            */
    const Word16 coder_type_org,/* i  : coding type                                       */
    const Word16 bwidth,        /* i  : input signal bandwidth                            */
    Word32 *Bin_Ener,           /* i  : FFT Bin energy 128 *2 sets                        */
    Word16 Q_ener,              /* i  : Q valuen for Bin_Ener                             */
    const Word32 int_fs,        /* i  : sampling frequency                                */
    Word32  core_brate,         /* i  : ACELP core bitrate                                */
    Word16 *streaklimit,        /* i/o: Multiplier to limit consecutive predictive usage  */
    Word16 *pstreaklen,         /* i/o: Lentght of the current predictive mode streak     */
    Word16 force_sf,            /* i  : Force safety-net usage if coding type supports    */
    Word16 rf_flag,             /* i  : Channel aware mode has some special cases         */
    Word16 mode2_flag,         /* i  : MODE2 mode has some special cases                */
    Word16 * lpc_param,
    Word16 * no_indices,
    Word16 * bits_param_lpc,
    Word16 coder_type_raw      /* i  : Coder type (LSF coder_type have some special cases)*/
)
{
    Word16 i;
    Word16 Idx0[MAX_VQ_STAGES+3];      /* Optimal codebook indices for safety-net quantizer                 */
    Word16 Idx1[MAX_VQ_STAGES+3];      /* Optimal codebook indices for predictive quantizer                 */
    Word16 indice[MAX_VQ_STAGES+3];    /* Temp. array of indice for vector de-quantizer                     */
    Word16 mode_lvq = 0, mode_lvq_p = 0; /* LVQ mode and predictive LVQ mode                                */
    Word16 bits0[MAX_VQ_STAGES], bits1[MAX_VQ_STAGES];
    const Word16 *Bit_alloc1 = NULL;
    Word32 Err[2];                     /* Quantization error for safety-net(0) and predictive(1) quantizers */
    Word16 Tmp [M];                    /* Temporary target vector (mean and prediction removed)             */
    Word16 pred0[M];                   /* Prediction for the safety-net quantizer (usually mean)            */
    Word16 pred1[M];                   /* Prediction for the predictive quantizer                           */
    Word16 pred2[M];                   /* Prediction for the predictive quantizer                           */
    Word16 wghts[M];                   /* Weighting used for quantizer (currently GSM based)                */
    Word16 stages0;                    /* Amount of stages used by safety-net quantizer                     */
    Word16 stages1;                    /* Amount of stages used by predictive quantizer                     */
    Word16 levels0[MAX_VQ_STAGES];     /* Sizes of different codebook stages for safety-net quantizer       */
    Word16 levels1[MAX_VQ_STAGES];     /* Sizes of different codebook stages for predictive quantizer       */
    Word16 predmode;                   /* 0: safety-net only, 1: predictive only, 2: best of the two        */
    Word16 safety_net, cumleft, num_bits;
    Word16 *Idx, stages, *bits;
    Word16 Tmp2[M], Tmp1[M];
    Word32 abs_threshold;              /* Absolute threshold depending on signal bandwidth, that indicates
                                          very good perceptual LSF quantization performance                 */
    Word16 lsfq[M*2], resq[M*2];
    Word16 coder_type;                 /* coder type (from LSF quantizer point of view)                     */
    Word16 nBits;                      /* Number of bits                                                    */
    Word16 TCQIdx0[M+2];               /* Optimal codebook indices for VQ-TCQ quantizer                     */
    Word16 *TCQIdx;
    Word16 tmp;

    nBits = nBits_in;
    move16();
    /* Update LSF coder_type for LSF quantizer for some special cases */
    test();
    test();
    test();
    IF(sub(coder_type_org, GENERIC)== 0 && L_sub(int_fs, INT_FS_16k)== 0 && (rf_flag == 0) && (mode2_flag == 0))
    {
        IF (sub(coder_type_raw, VOICED) == 0)
        {
            coder_type = VOICED;
            move16(); /* Reflect Inactive mode */
        }
        ELSE
        {
            nBits = sub(nBits,1); /* This is for real Generic*/
            coder_type = coder_type_org;
            move16();
        }
    }
    ELSE
    {
        coder_type = coder_type_org;
        move16();
    }

    /*----------------------------------------------------------------------------------- -*
    * Calculate the number of stages and levels for each stage based on allowed bit budget
    * Set absolute threshold for codebook-type decision logic depending on signal bandwidth
    *------------------------------------------------------------------------------------ -*/
    IF ( sub(bwidth, NB) == 0 )
    {
        abs_threshold = L_add(SFNETLOWLIMIT_NB, 0);
    }
    ELSE
    {
        abs_threshold = L_add(SFNETLOWLIMIT_WB, 0);
    }
    /* Calculate LSF weighting coefficients */
    Unified_weighting_fx(&Bin_Ener[L_FFT/2], Q_ener, lsf, wghts, sub(bwidth, NB) == 0, sub(coder_type,UNVOICED) == 0, int_fs,M);

    /*--------------------------------------------------------------------------------*
    * LSF quantization of SID frames
    *--------------------------------------------------------------------------------*/
    IF ( L_sub(core_brate, SID_2k40) == 0 )
    {
        lsfq_CNG_fx( st, lsf,  wghts, qlsf, &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->no_scales_fx[0][0] );
        sort_fx( qlsf, 0, M-1 );
        reorder_lsf_fx( qlsf, MODE1_LSF_GAP_FX, M, int_fs );

        return;
    }
    /* Find allowed predictor mode for current coder_type. (SN only (0), SN/AR switched (2) or MA predictive (1) */
    predmode = find_pred_mode(coder_type, bwidth, int_fs, &mode_lvq, &mode_lvq_p,st->total_brate_fx);

    /*----------------------------------------------------------------*
    * Calculate number of stages and levels for each stage based on the allowed bit allocation
    * (subtract one bit for LSF predictor selection)
    *----------------------------------------------------------------*/
    lsf_allocate_fx( sub(nBits, shr(predmode,1)), mode_lvq, mode_lvq_p, &stages0, &stages1, levels0, levels1, bits0, bits1);


    /*--------------------------------------------------------------------------------*
    * LSF quantization of all other frames but SID frames
    * Select safety-net or predictive mode
    *--------------------------------------------------------------------------------*/

    Err[0] = MAXINT32;
    move32();
    Err[1] = MAXINT32;
    move32();
    /* for mem_MA update */
    FOR (i=0; i<M; i++)
    {
        pred1[i] = add(ModeMeans_fx[mode_lvq][i], mult_r(MU_MA_FX,mem_MA[i]));
        move16();
    }
    IF ( predmode == 0 )
    {
        /* Subtract only mean */
        Copy(ModeMeans_fx[mode_lvq], pred0, M);
        Vr_subt(lsf, pred0, Tmp, M);

        /* LVQ quantization (safety-net only) */
        Err[0] = vq_lvq_lsf_enc(0, mode_lvq, Tmp, levels0, stages0,wghts, Idx0, lsf, pred0,
                                st->offset_scale1_fx,st->offset_scale2_fx, st->no_scales_fx, resq, lsfq);
        safety_net = 1;
        move16();
        *pstreaklen = 0;
        move16();/* Streak is ended with safety-net */
    }
    ELSE IF (sub(predmode, 1) == 0) /* only MA prediction */
    {
        Vr_subt(lsf, pred1, Tmp1, M);
        Err[1] = vq_lvq_lsf_enc(2, mode_lvq_p, Tmp1, levels1, stages1, wghts, Idx1, lsf, pred1,
                                st->offset_scale1_p_fx,st->offset_scale2_p_fx,st->no_scales_p_fx,resq, lsfq );

        safety_net = 0;
        move16();
    }
    ELSE
    {
        /* Increase AR-predictive usage for VOICED mode */
        test();
        test();
        test();
        IF ( ((sub(*pstreaklen, (STREAKLEN+3))>0)&&(sub(coder_type, VOICED)== 0)) || ((sub(*pstreaklen, (STREAKLEN)) >0) &&(sub(coder_type, VOICED) != 0)))
        {
            *streaklimit = mult(*streaklimit,STREAKMULT_FX);
            move16();
        }

        IF ( *pstreaklen == 0 )
        {
            /* reset the consecutive AR-predictor multiplier */
            *streaklimit = 32767; /*1.0 in Q15 */   move16();
        }

        /* VOICED_WB@16kHz */
        test();
        IF ( L_sub(int_fs, INT_FS_16k) == 0 && sub(coder_type, VOICED) == 0 )
        {
            /* Subtract mean and AR prediction */
            Copy( ModeMeans_fx[mode_lvq], pred0, M );
            /* subtract only mean */
            Vr_subt(lsf, pred0, Tmp, M);

            FOR (i = 0; i < M; i++)
            {
                /* subtract mean and AR prediction */
                pred2[i] = mult(Predictors_fx[mode_lvq_p][i],sub(mem_AR[i],pred0[i]));
                Tmp2[i] = sub(Tmp[i], pred2[i]);
                pred2[i] = add(pred2[i], pred0[i]);
            }

            /* select safety_net or predictive */
            safety_net = qlsf_Mode_Select_fx( wghts, Tmp2, *streaklimit, OP_LOOP_THR_HVO );
            IF ( sub(force_sf, 1) == 0 )
            {
                safety_net = 1;
                move16();
            }

            IF ( safety_net )
            {
                /* Safety-net - BC-TCQ quantization : SN */
                Err[0] = qlsf_ARSN_tcvq_Enc_16k_fx( Tmp, lsfq, TCQIdx0, wghts, sub(nBits,1), safety_net);
                *pstreaklen = 0;
                move16();
            }
            ELSE
            {
                /* predictive - BC-TCQ quantization : AR */
                Err[1] = qlsf_ARSN_tcvq_Enc_16k_fx( Tmp2, lsfq, TCQIdx0, wghts, sub(nBits,1), safety_net);
                *pstreaklen=add(*pstreaklen,1);
            }
        }
        /* all other frames (not VOICED@16kHz) */
        ELSE
        {
            /* Subtract mean and AR prediction */
            Copy( ModeMeans_fx[mode_lvq], pred0, M );
            /* subtract only mean */
            Vr_subt(lsf, pred0, Tmp, M);

            FOR (i = 0; i < M; i++)
            {
                /* subtract mean and AR prediction */
                pred2[i] = add(pred0[i],mult(Predictors_fx[mode_lvq_p][i],sub(mem_AR[i],pred0[i])));
                Tmp2[i] = sub(lsf[i], pred2[i]);
            }

            /* safety-net */
            Err[0] = vq_lvq_lsf_enc(0, mode_lvq, Tmp, levels0, stages0, wghts, Idx0, lsf, pred0,
            st->offset_scale1_fx,st->offset_scale2_fx,st->no_scales_fx, resq, lsfq);
            /* Predictive quantizer is calculated only if it can be selected */
            test();
            IF (!force_sf || L_sub(Err[0],abs_threshold) > 0 )
            {
                Err[1] = vq_lvq_lsf_enc(2, mode_lvq_p, Tmp2, levels1, stages1, wghts, Idx1, lsf, pred2,
                st->offset_scale1_p_fx, st->offset_scale2_p_fx, st->no_scales_p_fx, &resq[M], &lsfq[M]);

            }
            test();
            test();
            /*                                                                Err[1]*1.05 */
            IF ( force_sf || L_sub(Mult_32_16(Err[0],(*streaklimit)),L_add(Err[1],Mult_32_16(Err[1],PREFERSFNET_FX))) < 0 || L_sub(Err[0], abs_threshold) < 0 )
            {
                safety_net = 1;
                move16();
                *pstreaklen = 0;
                move16();
            }
            ELSE
            {
                safety_net = 0;
                move16();
                *pstreaklen=add(*pstreaklen,1);
            }
        }
    }
    /*--------------------------------------------------------------------------*
    * Write indices to array
    *--------------------------------------------------------------------------*/

    IF (mode2_flag == 0)
    {
        /* write coder_type bit for VOICED@16kHz or GENERIC@16kHz */
        test();
        IF(sub(coder_type_org, GENERIC)==0 && L_sub(int_fs, INT_FS_16k)==0)
        {
            /* VOICED =2 and GENERIC=3, so "coder_type-2" means VOICED =0 and GENERIC=1*/
            push_indice_fx( st, IND_LSF_PREDICTOR_SELECT_BIT, sub(coder_type,2), 1 );
        }

        /* write predictor selection bit */
        IF ( sub(predmode, 2) == 0 )
        {
            push_indice_fx( st, IND_LSF_PREDICTOR_SELECT_BIT, safety_net, 1 );
        }

        test();
        IF ( sub(coder_type, VOICED)== 0 && L_sub(int_fs, INT_FS_16k) == 0 )
        {
            /* BC-TCVQ (only for VOICED@16kHz) */
            TCQIdx = &TCQIdx0[1];
            Bit_alloc1 = &BC_TCVQ_BIT_ALLOC_40B[1];
            FOR( i=0; i<(M/2)+3; i++ )
            {
                push_indice_fx( st, IND_LSF, TCQIdx[i], Bit_alloc1[i]);
            }
        }
        ELSE
        {
            cumleft = nBits;
            move16();
            IF (sub( predmode, 2 )==0)
            {
                /* subtract predictor selection bit */
                cumleft = sub(nBits, 1);
            }

            IF ( safety_net )
            {
                stages = stages0;
                move16();
                Idx = Idx0;
                move16();
                bits = bits0;
                move16();
            }
            ELSE
            {
                stages = stages1;
                move16();
                Idx = Idx1;
                move16();
                bits = bits1;
                move16();
            }

            tmp = sub(stages,1);
            FOR ( i=0; i<tmp; i++ )
            {
                indice[i] = Idx[i];
                move16();
                num_bits = bits[i];
                move16();
                cumleft -= num_bits;
                move16();
                push_indice_fx( st, IND_LSF, indice[i], num_bits );
            }

            WHILE ( cumleft > 0 )
            {
                indice[i] = Idx[i];
                move16();

                IF ( sub(cumleft, LEN_INDICE) >0 )
                {
                    num_bits = LEN_INDICE;
                    move16();
                }
                ELSE
                {
                    num_bits = cumleft;
                    move16();
                }

                cumleft = sub(cumleft, num_bits);
                push_indice_fx( st, IND_LSF, indice[i], num_bits );
                i=add(i,1);
            }
        }
    }
    ELSE
    {
        test();
        IF ( sub(coder_type, VOICED)==0 && L_sub(int_fs, INT_FS_16k)== 0 )
        {
            /* BC-TCVQ (only for VOICED@16kHz) */
            /* Number of quantization indices */
            *no_indices = 10;
            move16();
            FOR(i=0; i<*no_indices; i++)
            {
                lpc_param[i] = TCQIdx0[i];
                move16();
                bits_param_lpc[i] = BC_TCVQ_BIT_ALLOC_40B[i];
                move16();
            }
        }
        ELSE
        {
            /* Number of quantization indices */

            /* there are 31 bits */
            IF (sub(safety_net, 1) == 0)
            {
                Idx = Idx0;
                move16();
                *no_indices = add(stages0 ,1);
                FOR( i=0; i<stages0; i++ )
                {
                    lpc_param[i] = Idx[i];
                    move16();
                    indice[i] = Idx[i];
                    move16();
                    bits_param_lpc[i] = bits0[i];
                    move16();
                }
                lpc_param[stages0] = Idx[stages0];
                move16();
                indice[stages0] = Idx[stages0];
                move16();
                tmp = sub(stages0,1);
                bits_param_lpc[tmp] = LEN_INDICE;
                move16();
                bits_param_lpc[stages0] = sub(bits0[tmp], LEN_INDICE);

            }
            ELSE
            {
                *no_indices = add(stages1, 1);
                Idx = Idx1;
                move16();
                FOR(i=0; i<stages1; i++)
                {
                    lpc_param[i] = (Idx[i]);
                    move16();
                    indice[i] = Idx[i];
                    move16();
                    bits_param_lpc[i] = bits1[i];
                    move16();
                }
                lpc_param[stages1] = (Idx[stages1]);
                move16();
                indice[stages1] = Idx[stages1];
                move16();
                tmp = sub(stages1,1);
                bits_param_lpc[tmp] = LEN_INDICE;
                move16();
                bits_param_lpc[stages1] = sub(bits1[tmp], LEN_INDICE);
            }
            IF (sub(predmode,2) ==0 )
            {
                FOR (i=*no_indices; i>0; i--)
                {
                    tmp = sub(i,1);
                    lpc_param[i] = lpc_param[tmp];
                    move16();
                    bits_param_lpc[i] = bits_param_lpc[tmp];
                    move16();
                }
                lpc_param[0] = safety_net;
                move16();/* put the safety net info on the last param */
                bits_param_lpc[0] = 1;
                move16();
                *no_indices = add(*no_indices,1);
            }
        }
    }


    /*--------------------------------------------------------------------------*
    *  De-quantize encoded LSF vector
    *--------------------------------------------------------------------------*/

    IF ( safety_net )
    {
        /* Safety-net */
        test();
        IF ( sub(coder_type, VOICED) == 0 && L_sub(int_fs, INT_FS_16k) == 0 )
        {
            /* BC-TCQ */
            Copy( lsfq, mem_MA, M );
            Vr_add( lsfq, pred0, qlsf, M );
        }
        ELSE
        {
            vq_dec_lvq_fx( 1, qlsf, &indice[0], stages0, M, mode_lvq, levels0[stages0-1],
            &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->offset_scale1_p_fx[0][0], &st->offset_scale2_p_fx[0][0],
            &st->no_scales_fx[0][0], &st->no_scales_p_fx[0][0] );

            Vr_add( qlsf, pred0, qlsf, M );
            Vr_subt(qlsf, pred1,mem_MA, M);
        }
    }
    ELSE
    {
        test();
        IF ( sub(coder_type, VOICED)== 0 && L_sub(int_fs, INT_FS_16k) == 0 )
        {
            /* BC-TCVQ */
            Copy( lsfq, mem_MA, M );
            Vr_add( lsfq, pred2, qlsf, M );
        }
        ELSE
        {
            /* LVQ */
            vq_dec_lvq_fx( 0, qlsf, &indice[0], stages1, M, mode_lvq_p, levels1[stages1-1],
            &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->offset_scale1_p_fx[0][0],
            &st->offset_scale2_p_fx[0][0], &st->no_scales_fx[0][0], &st->no_scales_p_fx[0][0] );
            IF (sub(predmode,1) == 0)
            {
                Copy(qlsf, mem_MA, M);
                Vr_add( qlsf, pred1, qlsf, M );
            }
            ELSE
            {
                Vr_add( qlsf, pred2, qlsf, M );
                Vr_subt(qlsf, pred1, mem_MA, M);
            }
        }
    }

    /* Sort the quantized vector */
    sort_fx( qlsf, 0, M-1 );

    /* Verify stability */
    reorder_lsf_fx( qlsf, MODE1_LSF_GAP_FX, M, int_fs );

    /* Update AR-predictor memories */
    Copy( qlsf, mem_AR, M );
    return;
}




/*-------------------------------------------------------------------*
* first_VQstages()
*
*
*-------------------------------------------------------------------*/

static void first_VQstages(
    const Word16 *  const *cb,
    Word16 u[],              /* i  : vector to be encoded (prediction and mean removed)  */
    Word16 *levels,          /* i  : number of levels in each stage                      */
    Word16 stagesVQ,         /* i  : number of stages                                    */
    Word16 w[],              /* i  : weights                                             */
    Word16 N,                /* i  : vector dimension                                    */
    Word16 max_inner,        /* i  : maximum number of swaps in inner loop               */
    Word16 indices_VQstage[]
)
{
    Word16 resid_buf[2*LSFMBEST*M], *resid[2];
    Word32 dist_buf[2*LSFMBEST], *dist[2], en;
    Word32 f_tmp, L_tmp, L_tmp1, *pTmp32;
    Word16 Tmp[M], *pTmp, cs;
    Word16 *pTmp_short, idx_buf[2*LSFMBEST*MAX_VQ_STAGES], parents[LSFMBEST], counter=0, j,
                                                                              m, s,c, c2, p_max, *indices[2];
    Word16 maxC = LSFMBEST;

    /*float dd[16];*/
    const Word16 *cb_stage, *cbp;

    /* Set pointers to previous (parent) and current node (parent node is indexed [0], current node is indexed [1]) */
    indices[0] = idx_buf;
    move16();
    indices[1] = idx_buf + maxC*stagesVQ;
    move16();
    resid[0] = resid_buf;
    move16();
    resid[1] = resid_buf + maxC*N;
    move16();
    dist[0] = dist_buf;
    move16();
    dist[1] = dist_buf + maxC;
    move16();

    set16_fx( idx_buf, 0, (const Word16)(2*stagesVQ*maxC) );
    set16_fx( parents, 0, maxC ) ;

    /* Set up inital distance vector */
    L_tmp = L_deposit_l(0);
    FOR( j=0; j<N; j++ )
    {
        L_tmp1 = L_shl(L_mult0(u[j], w[j]),7); /*x2.56 + Q8 + Q7 */
        L_tmp1 = Mult_32_16(L_tmp1,u[j]);      /*x2.56 + Q15 + x2.56 -Q15 */
        L_tmp = L_add(L_tmp, L_tmp1);          /*Q0 + x2.56 +x2.56 */
    }
    set32_fx( dist[1], L_tmp, maxC ) ;

    /* Set up initial error (residual) vectors */
    pTmp = resid[1];
    move16();
    FOR( c = 0; c < maxC; c++ )
    {
        Copy( u, pTmp, N );
        pTmp += N;
    }

    /*----------------------------------------------------------------*
    * LSF quantization
    *----------------------------------------------------------------*/

    /* Loop over all stages */
    m = 1;
    move16();
    FOR (  s = 0; s < stagesVQ; s++ )
    {
        /* set codebook pointer to point to first stage */
        cbp = cb[s];
        move16();

        /* save pointer to the beginning of the current stage */
        cb_stage = cbp;
        move16();

        /* swap pointers to parent and current nodes */
        pTmp_short = indices[0];
        indices[0] = indices[1];
        move16();
        indices[1] = pTmp_short;
        move16();

        pTmp = resid[0];
        resid[0] = resid[1];
        move16();
        resid[1] = pTmp;
        move16();

        pTmp32 = dist[0];
        dist[0] = dist[1];
        move32();
        dist[1] = pTmp32;
        move32();

        /* p_max points to maximum distortion node (worst of best) */
        p_max = 0;
        move16();

        /* set distortions to a large value */
        set32_fx( dist[1], MAXINT32, maxC );

        FOR ( j = 0; j < levels[s]; j++ )
        {
            /* compute weighted codebook element and its energy */
            FOR ( c2 = 0; c2 < N; c2++ )
            {
                Tmp[c2] = shl( mult(w[c2],cbp[c2]),2); /* Q8 + x2.56 -Q15 +Q2 */  move16();

            }

            en = L_mult(cbp[0],Tmp[0]);

            FOR ( c2 = 1; c2 < N; c2++ )
            {
                en = L_mac(en,cbp[c2],Tmp[c2]); /*x2.56 + x2.56 + Q-5 +Q1 */
            }
            cbp += N ;
            move16();

            /* iterate over all parent nodes */
            FOR( c = 0; c < m; c++ )
            {
                pTmp = &resid[0][c*N];
                move16();
                L_tmp = L_mult(pTmp[0], Tmp[0]);
                FOR ( c2=1; c2<N; c2++ )
                {
                    L_tmp = L_mac(L_tmp,pTmp[c2],Tmp[c2]); /* */
                }

                L_tmp = L_add(dist[0][c], L_sub(en, L_shl(L_tmp, 1)));

                IF ( L_sub(L_tmp,dist[1][p_max]) <= 0 )
                {
                    /* replace worst */
                    dist[1][p_max] = L_tmp;
                    move32();
                    indices[1][p_max*stagesVQ+s] = j;
                    move16();
                    parents[p_max] = c;
                    move16();

                    /* limit number of times inner loop is entered */
                    IF ( sub(counter, max_inner) < 0 )
                    {
                        counter=add(counter,1);
                        IF ( sub(counter, max_inner) < 0 )
                        {
                            /* find new worst */
                            p_max =  maximum_32_fx(dist[1],maxC, &f_tmp);
                        }
                        ELSE
                        {
                            /* find minimum distortion */
                            p_max = minimum_32_fx(dist[1], maxC, &f_tmp);
                        }
                    }
                }
            }
        }

        /*------------------------------------------------------------*
        * Compute error vectors for each node
        *------------------------------------------------------------*/
        cs = 0;
        move16();
        FOR ( c = 0; c < maxC; c++ )
        {
            /* subtract codebook entry from the residual vector of the parent node */
            pTmp = resid[1]+c*N ;
            move16();
            Copy( resid[0]+parents[c]*N, pTmp, N );
            Vr_subt( pTmp, cb_stage+(indices[1][cs+s])*N, pTmp, N );

            /* get indices that were used for parent node */
            Copy( indices[0]+parents[c]*stagesVQ, indices[1]+cs, s );
            cs = add(cs,stagesVQ);
        }

        m = maxC;
        move16();
    }

    Copy(indices[1],indices_VQstage,maxC*stagesVQ );

    return;
}

/*---------------------------------------------------------------------------
* vq_enc_lsf_lvq()
*
*  Multi-stage VQ encoder for LSF quantization. Trained codebooks are used in initial stages
*  and lattice-VQ quantization is applied on residual vector in other stages.
*
* Note:
*    Compared to normal multistage VQ resulting LSF vector is sorted before
*    weighted error calculation at the final stage.
*
* Returns:
*    Weighted error
*--------------------------------------------------------------------------*/

static Word32 vq_lvq_lsf_enc(
    Word16 pred_flag,
    Word16 mode,
    Word16 u[],
    Word16 * levels,
    Word16 stages,
    Word16 w[],
    Word16 Idx[],
    const Word16 * lsf,
    const Word16 * pred,
    Word32  p_offset_scale1[][MAX_NO_SCALES+1],
    Word32  p_offset_scale2[][MAX_NO_SCALES+1],
    Word16  p_no_scales[][2],
    Word16 *resq,
    Word16 * lsfq
)
{
    Word16 i;
    const Word16 *const *cb, *cb_stage;
    Word16 cand[LSFMBEST][M];
    Word16 maxC=LSFMBEST, stagesVQ;
    Word16  mode_glb, j, indices_firstVQ[LSFMBEST*MAX_VQ_STAGES], c2;
    Word32 e[LSFMBEST], L_tmp, L_ftmp;
    Word16 quant[LSFMBEST][M], diff[M], dd[M];
    Word16 lat_cv[LSFMBEST][M];
    Word16 idx_lead[LSFMBEST][2], idx_scale[LSFMBEST][2];

    stagesVQ = sub(stages,1);
    /* Codebook selection */
    IF (pred_flag==0) /* safety net*/
    {
        cb = &Quantizers_fx[CB_fx[mode]];
        move16();
        mode_glb = add(offset_lvq_modes_SN_fx[mode], offset_in_lvq_mode_SN_fx[mode][sub(levels[stagesVQ] , min_lat_bits_SN_fx[mode])]);
    }
    ELSE /*  predictive */
    {
        cb = &Quantizers_p_fx[CB_p_fx[mode]];
        move16();
        mode_glb = add(offset_lvq_modes_pred_fx[mode], offset_in_lvq_mode_pred_fx[mode][sub(levels[stagesVQ], min_lat_bits_pred_fx[mode])]);
    }
    IF (stagesVQ>0)
    {
        /* first VQ stages */
        first_VQstages( cb, u, levels, stagesVQ, w, M, MSVQ_MAXCNT, indices_firstVQ );
    }


    FOR ( i=0; i<maxC; i++ )
    {
        Copy( pred, cand[i], M );
        FOR ( j=0; j<stagesVQ; j++ )
        {
            Idx[j] = indices_firstVQ[i*stagesVQ+j];
            move16();
        }

        FOR ( j=0; j<stagesVQ; j++ )
        {
            cb_stage = cb[j];
            move16();
            Vr_add( cand[i], cb_stage+Idx[j]*M, cand[i], M );
        }

        /* LVQ quantization */
        Vr_subt( lsf, cand[i], dd, M );
        mslvq_fx(dd, quant[i], lat_cv[i], idx_lead[i], idx_scale[i], w, mode, mode_glb, pred_flag, p_no_scales);
        Vr_add( cand[i], quant[i], cand[i], M );

        /* sort the LSF candidate prior to selection */
        sort_fx(cand[i],0,M-1);

        /* calculate the weighted MSE of sorted LSF vector*/
        Vr_subt( cand[i], lsf, diff, M );

        FOR (j = 0 ; j < M ; j++)
        {
            diff[j] = shl(diff[j], 4);
            move16();
        }
        L_tmp = L_mult(mult(diff[0], shl(w[0],1) ), diff[0]); /*(2.56+Q5+ Q10 -Q15) + 2.56+ Q5 + Q1 = 2.56 + 2.56 + Q6 */
        FOR (j=1; j<M; j++)
        {
            L_tmp = L_mac(L_tmp,mult(diff[j],shl(w[j],1) ), diff[j]); /*(2.56+Q5+ Q10 -Q15) + 2.56+ Q5 + Q1 = 2.56 + 2.56 + Q6 */
        }
        e[i] = L_tmp;
        move32();
    }

    /* find the optimal candidate */
    c2 = minimum_32_fx( e, maxC, &L_ftmp );
    set16_fx(resq, 0, M);
    FOR ( j=0; j<stagesVQ; j++ )
    {
        Idx[j] = indices_firstVQ[c2*stagesVQ+j];
        move16();
        cb_stage = cb[j];
        move16();
        Vr_add(resq, cb_stage+Idx[j]*M, resq, M);
    }
    Vr_add(resq, quant[c2],resq, M); /* quantized prediction residual */
    Copy(cand[c2], lsfq, M);
    index_lvq_fx( lat_cv[c2], idx_lead[c2], idx_scale[c2], mode_glb, &Idx[stagesVQ],
                  &p_offset_scale1[0][0], &p_offset_scale2[0][0], &p_no_scales[0][0] );

    return e[c2];
}

static void BcTcvq_1st_fx(
    Word16 x_fx[][2],             /*x2.56*/
    const Word16 CB_fx[][128][2], /*x2.56*/
    Word16 s[][16],
    Word16 c[][16],
    Word32 cDist_fx[][16],         /*2.56*2.56*Q(-5 - 2)*/
    Word16 Q_fx[][16][2],
    Word16 W_fx[][2]               /*Q10*/
)
{
    Word16 state, prev_state;
    Word16 index, bestCode;
    Word32 dist_fx, minDist_fx;
    Word16 temp16_fx;

    FOR (state = 0; state < NUM_STATE; state +=2)
    {
        prev_state = NTRANS[0][state];
        move16();
        index     = NTRANS[2][state];
        move16();
        temp16_fx = sub(x_fx[0][0], CB_fx[0][index][0]);
        minDist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[0][0]); /* 2.56*2.56*Q(-5) */
        temp16_fx = sub(x_fx[0][1], CB_fx[0][index][1]);
        minDist_fx = L_add(minDist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[0][1])); /* 2.56*2.56*Q(-5) */
        bestCode = index;
        move16();

        FOR (index = index+8; index < 128; index += 8)
        {
            temp16_fx = sub(x_fx[0][0], CB_fx[0][index][0]);
            dist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[0][0]); /* 2.56*2.56*Q(-5) */
            temp16_fx = sub(x_fx[0][1], CB_fx[0][index][1]);
            dist_fx = L_add(dist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[0][1])); /* 2.56*2.56*Q(-5) */

            if (L_sub(dist_fx, minDist_fx) < 0)
            {
                bestCode = index;
                move16();
            }
            minDist_fx = L_min(minDist_fx, dist_fx);
        }

        /* Update */
        s[0][state]    = prev_state;
        move16();
        c[0][state]    = bestCode;
        move16();

        cDist_fx[0][state] = L_shr(minDist_fx, 2);
        move32(); /*2.56*2.56*Q(-5 - 2)*/
        Q_fx[0][state][0] = CB_fx[0][bestCode][0];
        move16();
        Q_fx[0][state][1] = CB_fx[0][bestCode][1];
        move16();
    }

    return;
}


static void BcTcvq_2nd_fx(
    Word16 x_fx[][2],        /*x2.56*/
    const Word16 CB_fx[][128][2],  /*x2.56*/
    Word16 s[][16],
    Word16 c[][16],
    Word32 cDist_fx[][16],      /*2.56*2.56*Q(-5 - 2) */
    Word16 Q_fx[][16][2],      /*x2.56*/
    Word16 W_fx[][2],        /*Q10*/
    const Word16 itc_fx[][2][2]    /*Q15*/
)
{
    Word16 state, prev_state;
    Word16 index, bestCode;
    Word32 dist_fx, minDist_fx;
    Word16 pred_fx[N_DIM], target_fx[N_DIM];
    Word16 temp16_fx;

    FOR (state = 0; state < NUM_STATE; state++)
    {
        prev_state = NTRANS[0][state];
        move16();
        index      = NTRANS[2][state];
        move16();

        /* Prediction */
        pred_fx[0] = add(mult_r(itc_fx[0][0][0], Q_fx[0][prev_state][0]), mult_r(itc_fx[0][0][1], Q_fx[0][prev_state][1]));
        move16();
        pred_fx[1] = add(mult_r(itc_fx[0][1][0], Q_fx[0][prev_state][0]), mult_r(itc_fx[0][1][1], Q_fx[0][prev_state][1]));
        move16();
        target_fx[0] = sub(x_fx[1][0], pred_fx[0]);
        move16(); /* x2.65 */
        target_fx[1] = sub(x_fx[1][1], pred_fx[1]);
        move16(); /* x2.65 */

        temp16_fx = sub(target_fx[0], CB_fx[1][index][0]);
        minDist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[1][0]); /* 2.65*2.65*Q(-5) */
        temp16_fx = sub(target_fx[1], CB_fx[1][index][1]);
        minDist_fx = L_add(minDist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[1][1])); /* 2.65*2.65*Q(-5) */

        bestCode = index;
        move16();

        FOR (index = index + 8; index < 128; index += 8)
        {
            temp16_fx = sub(target_fx[0], CB_fx[1][index][0]);
            dist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[1][0]); /* 2.65*2.65*Q(-5) */
            temp16_fx = sub(target_fx[1], CB_fx[1][index][1]);
            dist_fx = L_add(dist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[1][1])); /* 2.65*2.65*Q(-5) */

            if (L_sub(dist_fx, minDist_fx) < 0)
            {
                bestCode = index;
                move16();
            }
            minDist_fx = L_min(minDist_fx, dist_fx);
        }

        /* Update */
        s[1][state]    = prev_state;
        move16();
        c[1][state]    = bestCode;
        move16();

        cDist_fx[1][state] = L_add(cDist_fx[0][prev_state], L_shr(minDist_fx, 2));
        move32(); /* 2.56*2.56*Q(-5 - 2) */
        Q_fx[1][state][0]  = add(CB_fx[1][bestCode][0], pred_fx[0]);
        move16();
        Q_fx[1][state][1]  = add(CB_fx[1][bestCode][1], pred_fx[1]);
        move16();
    }

    return;
}

static void BcTcvq_SubBlock_fx(
    Word16 x_fx[][2],        /*x2.56*/
    const Word16 CB_fx[][64][2],    /*x2.56*/
    Word16 s[][16],
    Word16 c[][16],
    Word32 cDist_fx[][16],      /*2.56*2.56*Q(-5 - 2)*/
    Word16 Q_fx[][16][2],
    Word16 stage,
    Word16 W_fx[][2],        /*Q10*/
    const Word16 itc_fx[][2][2]    /*Q15*/
)
{
    Word16 stage1, stage2, state, prev_state, branch;
    Word16 index, bestCode, brCode[N_DIM];
    Word16 temp16_fx;

    Word32 dist_fx, minDist_fx, brDist_fx[N_DIM];
    Word16 pred_fx[N_DIM], target_fx[N_DIM], brQuant_fx[N_DIM][N_DIM];

    stage1 = sub(stage, 1);
    stage2 = sub(stage, 2);

    FOR (state = 0; state < NUM_STATE; state ++)
    {

        /* 1st brarnch search */
        prev_state = NTRANS[0][state];
        move16();
        index     = NTRANS[2][state];
        move16();

        /* Prediction */
        pred_fx[0] = add(mult_r(itc_fx[stage1][0][0], Q_fx[stage1][prev_state][0]), mult_r(itc_fx[stage1][0][1], Q_fx[stage1][prev_state][1]));
        move16();
        pred_fx[1] = add(mult_r(itc_fx[stage1][1][0], Q_fx[stage1][prev_state][0]), mult_r(itc_fx[stage1][1][1], Q_fx[stage1][prev_state][1]));
        move16();
        target_fx[0] = sub(x_fx[stage][0], pred_fx[0]);
        move16();
        target_fx[1] = sub(x_fx[stage][1], pred_fx[1]);
        move16();

        temp16_fx = sub(target_fx[0], CB_fx[stage2][index][0]);
        minDist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]); /* 2.65*2.65*Q(-5) */
        temp16_fx = sub(target_fx[1], CB_fx[stage2][index][1]);
        minDist_fx = L_add(minDist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1])); /* 2.65*2.65*Q(-5) */

        bestCode = index;
        move16();

        FOR (index = index + 8; index < 64; index += 8)
        {
            temp16_fx = sub(target_fx[0], CB_fx[stage2][index][0]);
            dist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]);
            temp16_fx = sub(target_fx[1], CB_fx[stage2][index][1]);
            dist_fx = L_add(dist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1]));

            if (L_sub(dist_fx, minDist_fx) < 0)
            {
                bestCode = index;
                move16();
            }
            minDist_fx = L_min(minDist_fx, dist_fx);
        }

        brCode[0]    = bestCode;
        move16();

        brDist_fx[0] = L_add(cDist_fx[stage1][prev_state], L_shr(minDist_fx, 2));
        move32();
        brQuant_fx[0][0] = add(CB_fx[stage2][bestCode][0], pred_fx[0]);
        move16();
        brQuant_fx[0][1] = add(CB_fx[stage2][bestCode][1], pred_fx[1]);
        move16();

        /* 2nd branch search */
        prev_state = NTRANS[1][state];
        move16();
        index     = NTRANS[3][state];
        move16();

        /* Prediction */
        pred_fx[0] = add(mult_r(itc_fx[stage1][0][0], Q_fx[stage1][prev_state][0]), mult_r(itc_fx[stage1][0][1], Q_fx[stage1][prev_state][1]));
        move16();
        pred_fx[1] = add(mult_r(itc_fx[stage1][1][0], Q_fx[stage1][prev_state][0]), mult_r(itc_fx[stage1][1][1], Q_fx[stage1][prev_state][1]));
        move16();
        target_fx[0] = sub(x_fx[stage][0], pred_fx[0]);
        move16();
        target_fx[1] = sub(x_fx[stage][1], pred_fx[1]);
        move16();

        temp16_fx = sub(target_fx[0], CB_fx[stage2][index][0]);
        minDist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]); /* 2.65*2.65*Q(-5) */
        temp16_fx = sub(target_fx[1], CB_fx[stage2][index][1]);
        minDist_fx = L_add(minDist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1])); /* 2.65*2.65*Q(-5) */

        bestCode = index;
        move16();

        FOR (index = index + 8; index < 64; index += 8)
        {
            temp16_fx = sub(target_fx[0], CB_fx[stage2][index][0]);
            dist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]);
            temp16_fx = sub(target_fx[1], CB_fx[stage2][index][1]);
            dist_fx = L_add(dist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1]));
            if (L_sub(dist_fx, minDist_fx) < 0)
            {
                bestCode = index;
                move16();
            }
            minDist_fx = L_min(minDist_fx, dist_fx);
        }

        brCode[1]    = bestCode;
        move16();

        brDist_fx[1] = L_add(cDist_fx[stage1][prev_state], L_shr(minDist_fx, 2));
        move32();
        brQuant_fx[1][0] = add(CB_fx[stage2][bestCode][0], pred_fx[0]);
        move16();
        brQuant_fx[1][1] = add(CB_fx[stage2][bestCode][1], pred_fx[1]);
        move16();

        /* Select Best branch */
        branch = 1;
        move16();

        if (L_sub(brDist_fx[0], brDist_fx[1]) <= 0)
        {
            branch = 0;
            move16();
        }

        /* Update */
        s[stage][state]    = NTRANS[branch][state];
        move16();
        c[stage][state]    = brCode[branch];
        move16();

        cDist_fx[stage][state] = brDist_fx[branch];
        move32();
        Q_fx[stage][state][0]  = brQuant_fx[branch][0];
        move16();
        Q_fx[stage][state][1]  = brQuant_fx[branch][1];
        move16();
    }

    return;
}

static Word32 BcTcvq_FixSearch_fx(
    Word16 x_fx[][2],        /*x2.56*/
    const Word16 CB_fx[][32][2],    /*x2.56*/
    Word16 c[][4],
    Word16 Q_fx[][16][2],
    const Word16 FixBranch[][4][4],
    Word16 stage,
    Word16 inis,
    Word16 fins,
    Word16 *prev_state,
    Word16 W_fx[][2],        /*Q10*/
    const Word16 itc_fx[][2][2]    /*Q15*/
)
{
    Word16 stage1, stage4, branch;
    Word16 index, bestCode;
    Word32 dist_fx, minDist_fx;
    Word16 pred_fx[N_DIM], target_fx[N_DIM];
    Word16 temp16_fx;

    stage1 = sub(stage, 1);
    stage4 = sub(stage, 4);

    branch = FixBranch[inis>>2][fins][stage4];
    move16();
    index  = NTRANS2[branch+2][*prev_state];
    move16();

    /* Prediction */
    pred_fx[0] = add(mult_r(itc_fx[stage1][0][0], Q_fx[stage1][*prev_state][0]), mult_r(itc_fx[stage1][0][1], Q_fx[stage1][*prev_state][1]));
    move16();
    pred_fx[1] = add(mult_r(itc_fx[stage1][1][0], Q_fx[stage1][*prev_state][0]), mult_r(itc_fx[stage1][1][1], Q_fx[stage1][*prev_state][1]));
    move16();
    target_fx[0] = sub(x_fx[stage][0], pred_fx[0]);
    move16();
    target_fx[1] = sub(x_fx[stage][1], pred_fx[1]);
    move16();

    temp16_fx = sub(target_fx[0], CB_fx[stage4][index][0]);
    minDist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]); /* 2.65*2.65*Q(-5) */
    temp16_fx = sub(target_fx[1], CB_fx[stage4][index][1]);
    minDist_fx = L_add(minDist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1])); /* 2.65*2.65*Q(-5) */

    bestCode = index;
    move16();

    FOR (index = index + 8; index < 32; index += 8)
    {
        temp16_fx = sub(target_fx[0], CB_fx[stage4][index][0]);
        dist_fx = Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][0]);
        temp16_fx = sub(target_fx[1], CB_fx[stage4][index][1]);
        dist_fx = L_add(dist_fx, Mult_32_16(L_mult0(temp16_fx, temp16_fx), W_fx[stage][1]));

        if(L_sub(dist_fx, minDist_fx) < 0)
        {
            bestCode = index;
            move16();
        }
        minDist_fx = L_min(minDist_fx, dist_fx);
    }

    /* Update */
    *prev_state         = NTRANS2[branch][*prev_state];
    move16();
    c[fins][stage4]       = bestCode;
    move16();

    Q_fx[stage][*prev_state][0] = add(CB_fx[stage4][bestCode][0], pred_fx[0]);
    move16();
    Q_fx[stage][*prev_state][1] = add(CB_fx[stage4][bestCode][1], pred_fx[1]);
    move16();

    minDist_fx = L_shr(minDist_fx, 2);
    return minDist_fx;

}
static Word16 optimalPath_fx(
    Word32 cDist_fx[][16],
    Word32 blockDist_fx[],
    Word16 blockCodeword[][4],
    Word16 bestCodeword[],
    Word16 codeWord[][16],
    Word16 bestState[],
    Word16 preState[][16]
)
{
    Word16 stage, state;
    Word32 opDist_fx[NUM_STATE];
    Word32 minDist_fx;
    Word16 fBlock;
    Word16 prev_state;

    FOR (state = 0; state < NUM_STATE; state++)
    {
        opDist_fx[state] = L_add(L_shr(cDist_fx[3][state], 1), L_shr(blockDist_fx[state], 1));
        move32();
    }

    minDist_fx = L_add(opDist_fx[0], 0);
    fBlock  = 0;
    move16();

    FOR (state = 1; state < NUM_STATE; state++)
    {
        if (L_sub(opDist_fx[state], minDist_fx) < 0)
        {
            fBlock  = state;
            move16();
        }
        minDist_fx = L_min(minDist_fx, opDist_fx[state]);
    }

    prev_state = bestState[4] = fBlock;
    move16();
    move16();

    FOR (stage = N_STAGE_VQ - 5; stage >= 0; stage --)
    {
        bestCodeword[stage] = codeWord[stage][prev_state];
        move16();
        bestState[stage]  = preState[stage][prev_state];
        move16();
        prev_state        = bestState[stage];
        move16();
    }

    FOR (stage = 0; stage < 4; stage ++)
    {
        bestCodeword[stage + 4] = blockCodeword[fBlock][stage];
        move16();
    }

    return fBlock;
}

static void quantEnc_fx(
    Word16 *y_fx,
    Word16 c[],
    const Word16 CB_SUB1_fx[][128][2],
    const Word16 CB_SUB2_fx[][64][2],
    const Word16 CB_SUB3_fx[][32][2],
    const Word16 itc_fx[][2][2]
)
{
    Word16 i,j;
    Word16 stage;
    Word16 pred_fx[N_DIM], Y_fx[8][2];

    /* stage #1 */
    Y_fx[0][0]  = CB_SUB1_fx[0][c[0]][0];
    move16();
    Y_fx[0][1]  = CB_SUB1_fx[0][c[0]][1];
    move16();

    /* stage #2 */
    pred_fx[0]  = add(mult_r(itc_fx[0][0][0], Y_fx[0][0]), mult_r(itc_fx[0][0][1], Y_fx[0][1]));
    move16();
    pred_fx[1]  = add(mult_r(itc_fx[0][1][0], Y_fx[0][0]), mult_r(itc_fx[0][1][1], Y_fx[0][1]));
    move16();
    Y_fx[1][0] = add(CB_SUB1_fx[1][c[1]][0], pred_fx[0]);
    move16();
    Y_fx[1][1] = add(CB_SUB1_fx[1][c[1]][1], pred_fx[1]);
    move16();

    /* stage #3 - #4 */
    FOR (stage = 2; stage < N_STAGE_VQ-4; stage ++)
    {
        pred_fx[0]  = add(mult_r(itc_fx[stage-1][0][0], Y_fx[stage-1][0]), mult_r(itc_fx[stage-1][0][1], Y_fx[stage-1][1]));
        move16();
        pred_fx[1]  = add(mult_r(itc_fx[stage-1][1][0], Y_fx[stage-1][0]), mult_r(itc_fx[stage-1][1][1], Y_fx[stage-1][1]));
        move16();

        Y_fx[stage][0] = add(CB_SUB2_fx[stage-2][c[stage]][0], pred_fx[0]);
        move16();
        Y_fx[stage][1] = add(CB_SUB2_fx[stage-2][c[stage]][1], pred_fx[1]);
        move16();
    }

    /* stage #5 - #8 */
    FOR (stage = N_STAGE_VQ-4; stage < N_STAGE_VQ; stage ++)
    {
        pred_fx[0]    = add(mult_r(itc_fx[stage-1][0][0], Y_fx[stage-1][0]), mult_r(itc_fx[stage-1][0][1], Y_fx[stage-1][1]));
        move16();
        pred_fx[1]    = add(mult_r(itc_fx[stage-1][1][0], Y_fx[stage-1][0]), mult_r(itc_fx[stage-1][1][1], Y_fx[stage-1][1]));
        move16();

        Y_fx[stage][0] = add(CB_SUB3_fx[stage-4][c[stage]][0], pred_fx[0]);
        move16();
        Y_fx[stage][1] = add(CB_SUB3_fx[stage-4][c[stage]][1], pred_fx[1]);
        move16();
    }

    /* Transform Vector to Scalar */
    FOR (i = 0; i < N_STAGE_VQ; i++)
    {
        FOR (j = 0; j < N_DIM; j++)
        {
            y_fx[i*N_DIM+j] = Y_fx[i][j];
            move16();
        }
    }

    return;
}

static void buildCode_fx(
    Word16 *ind,
    Word16 fins,
    Word16 c[],
    Word16 s[]
)
{
    Word16 stage;
    Word16 BrIndex[4];

    set16_fx(BrIndex, 0, (N_STAGE_VQ - 4));


    FOR (stage = N_STAGE_VQ - 4; stage >= 1; stage--)
    {
        if(sub(s[stage], 7) > 0)
        {
            BrIndex[stage-1] =1;
            move16();
        }
    }
    ind[0] = fins;
    move16();

    /* stage #1 - #2 */
    FOR (stage = 0; stage < 2; stage++)
    {
        ind[stage+1]  = shl(BrIndex[stage], 4);
        move16();
        ind[stage+1] = add(ind[stage+1], shr(c[stage], 3));
        move16();
    }

    /* stage #3 - #4 */
    FOR (stage = 2; stage < N_STAGE_VQ - 4; stage++)
    {
        ind[stage+1]  = shl(BrIndex[stage], 3);
        move16();
        ind[stage+1] = add(ind[stage+1], shr(c[stage], 3));
        move16();
    }

    /* Stage #5 - #8 */
    FOR (stage = N_STAGE_VQ-4; stage < N_STAGE_VQ; stage++)
    {
        ind[stage+1]  = shr(c[stage], 3);
        move16();
    }

    return;
}
static void BcTcvq_fx(
    Word16 snFlag,
    const Word16 *x_fx,
    Word16 *y_fx,
    const Word16 *weight_fx,
    Word16 *ind
)
{
    Word16 X_fx[N_STAGE_VQ][N_DIM], W_fx[N_STAGE_VQ][N_DIM];

    /* Count Variable */
    Word16 i,j;

    /* TCVQ Structure */
    Word16 stage, state, prev_state;
    Word16 preState[N_STAGE_VQ][NUM_STATE];
    Word16 codeWord[N_STAGE_VQ][NUM_STATE];
    Word32 acumDist_fx[N_STAGE_VQ-4][NUM_STATE];
    Word16 inis, fins, ptr_fins;
    Word16 fBlock;
    Word16 fState[NUM_STATE];
    Word16 fCodeword[4][4];
    Word16 iniBlock[NUM_STATE];
    Word16 blockCodeword[NUM_STATE][4];

    /* Prediction variable */
    Word16 quant_fx[N_STAGE_VQ][NUM_STATE][N_DIM];

    /* Distortion variable */
    Word32 minDist_fx;
    Word32 fDist_fx;
    Word32 blockDist_fx[NUM_STATE];

    /* Decoding variable */
    Word16 bestCodeword[N_STAGE_VQ];
    Word16 bestState[N_STAGE_VQ];

    /* Code Share variable */
    const Word16 (*TCVQ_CB_SUB1_fx)[128][2], (*TCVQ_CB_SUB2_fx)[64][2], (*TCVQ_CB_SUB3_fx)[32][2]/**/;
    const Word16 (*IntraCoeff_fx)[2][2];

    /* Memoryless Module */
    IF (snFlag)
    {
        TCVQ_CB_SUB1_fx = SN_TCVQ_CB_SUB1_fx;
        TCVQ_CB_SUB2_fx = SN_TCVQ_CB_SUB2_fx;
        TCVQ_CB_SUB3_fx = SN_TCVQ_CB_SUB3_fx;
        IntraCoeff_fx   = SN_IntraCoeff_fx;
    }
    ELSE /* Memory Module */
    {
        TCVQ_CB_SUB1_fx = AR_TCVQ_CB_SUB1_fx;
        TCVQ_CB_SUB2_fx = AR_TCVQ_CB_SUB2_fx;
        TCVQ_CB_SUB3_fx = AR_TCVQ_CB_SUB3_fx;
        IntraCoeff_fx   = AR_IntraCoeff_fx;
    }

    /* Transform Scalar to Vector */
    FOR (i = 0; i < N_STAGE_VQ; i++)
    {
        FOR(j = 0; j < N_DIM; j++)
        {
            X_fx[i][j] = x_fx[(N_DIM*i) + j];
            move16();
            W_fx[i][j] = weight_fx[(N_DIM*i) + j];
            move16();
        }
    }

    /* Initialzie */
    FOR (i=0; i<N_STAGE_VQ-4; i++)
    {
        FOR(j=0; j<NUM_STATE; j++)
        {
            acumDist_fx[i][j] = L_deposit_l(0);
        }
    }

    /* BcTcvq Search */
    /* stage #1 */
    BcTcvq_1st_fx(X_fx, TCVQ_CB_SUB1_fx, preState, codeWord, acumDist_fx, quant_fx, W_fx);

    /* stage #2 */
    BcTcvq_2nd_fx(X_fx, TCVQ_CB_SUB1_fx, preState, codeWord, acumDist_fx, quant_fx, W_fx, IntraCoeff_fx);

    /* stage #3 - #4 */
    FOR (stage = 2; stage < N_STAGE_VQ - 4; stage++)
    {
        BcTcvq_SubBlock_fx(X_fx, TCVQ_CB_SUB2_fx, preState, codeWord, acumDist_fx, quant_fx, stage, W_fx, IntraCoeff_fx);
    }

    /* Search initial state at each block */
    FOR (state = 0; state < NUM_STATE; state++)
    {
        prev_state = state;
        move16();

        FOR (stage = N_STAGE_VQ - 5; stage >= 0; stage--)
        {
            prev_state = preState[stage][prev_state];
            move16();
        }
        iniBlock[state] = prev_state;
        move16();
    }

    /* stage #5 - #8 */
    FOR (state = 0; state < NUM_STATE; state++)
    {
        inis    = iniBlock[state];
        move16();
        ptr_fins  = shr(inis, 2);
        minDist_fx = L_add(MAX_32, 0);

        FOR (i = 0; i < 4; i++)
        {
            fins = add(shl(ptr_fins, 2), i);
            prev_state = state;
            move16();
            fDist_fx = BcTcvq_FixSearch_fx(X_fx, TCVQ_CB_SUB3_fx, fCodeword, quant_fx, FixBranch, N_STAGE_VQ-4, inis, i, &prev_state, W_fx, IntraCoeff_fx);

            FOR (stage = N_STAGE_VQ-3; stage < N_STAGE_VQ; stage++)
            {
                fDist_fx = L_add(fDist_fx, BcTcvq_FixSearch_fx(X_fx, TCVQ_CB_SUB3_fx, fCodeword, quant_fx, FixBranch, stage, inis, i, &prev_state, W_fx, IntraCoeff_fx));
            }
            IF (L_sub(fDist_fx, minDist_fx) < 0)
            {
                minDist_fx = L_add(fDist_fx, 0);
                blockDist_fx[state] = minDist_fx;
                move32();

                fState[state]   = fins;
                move16();

                blockCodeword[state][0] = fCodeword[i][0];
                move16();
                blockCodeword[state][1] = fCodeword[i][1];
                move16();
                blockCodeword[state][2] = fCodeword[i][2];
                move16();
                blockCodeword[state][3] = fCodeword[i][3];
                move16();
            }
        }
    }

    /* Select optimal path */
    fBlock = optimalPath_fx(acumDist_fx, blockDist_fx, blockCodeword, bestCodeword, codeWord, bestState, preState);

    /* Select Quantized Value */
    quantEnc_fx(y_fx, bestCodeword, TCVQ_CB_SUB1_fx, TCVQ_CB_SUB2_fx, TCVQ_CB_SUB3_fx, IntraCoeff_fx);

    /* Buid Code for Decoder */
    buildCode_fx(ind, fState[fBlock], bestCodeword, bestState);

    return;
}

static Word16 SVQ_2d_fx(
    Word16 *x_fx,
    Word16 *y_fx,
    const Word16 *W_fx,
    const Word16 CB_fx[][8],
    Word16 Size
)
{
    Word16 i, j;
    Word16 index = 0;
    Word32 distortion_fx;
    Word32 temp_fx;
    Word16 temp16_fx;

    temp_fx = L_add(MAX_32, 0);

    FOR (i = 0; i < Size; i++)
    {
        distortion_fx = L_deposit_l(0);
        FOR (j = 0; j < 8; j++)
        {
            temp16_fx = sub(x_fx[j], CB_fx[i][j]);
            distortion_fx = L_add(distortion_fx,
                                  L_shr(Mult_32_16(L_mult(temp16_fx, temp16_fx), W_fx[j]), 1));
        }

        IF (L_sub(distortion_fx, temp_fx) < 0)
        {
            temp_fx = L_add(distortion_fx, 0);
            index = i;
            move16();
        }
    }

    FOR (i = 0; i < M/2; i++)
    {
        y_fx[i] = CB_fx[index][i];
        move16();
    }

    return index;
}


Word32 qlsf_ARSN_tcvq_Enc_16k_fx (
    const Word16 *x_fx,    /* i  : Vector to be encoded  x2.65  */
    Word16 *y_fx,    /* o  : Quantized LSF vector  x2.65  */
    Word16 *indice,      /* o  : Indices                 */
    const Word16 *w_fx,    /* i  : LSF Weights           Q8  */
    const Word16 nBits,         /* i  : number of bits          */
    Word16 safety_net    /* i : safety_net flag */
)
{
    Word16 i;
    Word16 x_q_fx[M];
    Word16 yy_fx[M];
    Word16 error_svq_fx[M], error_svq_q_fx[M];
    Word16 cl, cs;
    Word32 temp_l;
    IF (sub(safety_net, 1) == 0)
    {
        indice[0] = 1;
        move16();
        BcTcvq_fx(1, /*x, x_q, w, */x_fx, x_q_fx, w_fx, &indice[1]);

        IF (sub(nBits, 30) > 0)
        {
            /* SVQ  */
            FOR (i = 0; i < M; i++)
            {
                error_svq_fx[i] = mult_r(sub(x_fx[i], x_q_fx[i]), scale_inv_ARSN_fx[i]);
                move16();
            }

            /* 5bits 1st Split VQ for Residual*/
            indice[10] = SVQ_2d_fx(error_svq_fx, error_svq_q_fx, w_fx, AR_SVQ_CB1_fx, 32);
            /* 4bits 2nd Split VQ for Residual*/
            indice[11] = SVQ_2d_fx(&error_svq_fx[8], &error_svq_q_fx[8], &w_fx[8], AR_SVQ_CB2_fx, 16 );

            FOR (i = 0; i < M; i++)
            {
                x_q_fx[i] = add(x_q_fx[i], extract_h(L_shl(L_mult0(error_svq_q_fx[i], scale_ARSN_fx[i]), 2)));
                move16();
            }
        }
    }
    ELSE
    {
        indice[0] = 0;
        move16();
        BcTcvq_fx(0, /*x, x_q, w, */x_fx, x_q_fx, w_fx, &indice[1]);

        IF (sub(nBits, 30) > 0)
        {
            /* SVQ */
            FOR (i = 0; i < M; i++)
            {
                error_svq_fx[i] = sub(x_fx[i], x_q_fx[i]);
                move16();
            }

            /* 5bits 1st Split VQ for Residual*/
            indice[10] = SVQ_2d_fx(error_svq_fx, error_svq_q_fx, w_fx, AR_SVQ_CB1_fx, 32);
            /* 4bits 2nd Split VQ for Residual*/
            indice[11] = SVQ_2d_fx(&error_svq_fx[8], &error_svq_q_fx[8], &w_fx[8], AR_SVQ_CB2_fx, 16 );

            FOR (i = 0; i < M; i++)
            {
                x_q_fx[i] = add(x_q_fx[i], error_svq_q_fx[i]);
                move16();
            }
        }
    }

    cl = 0;
    move16();
    FOR (i = 0; i < M; i ++)
    {
        yy_fx[i] = sub(x_fx[i], x_q_fx[i]);
        move16();
        cl = s_max(cl, abs_s(yy_fx[i]));
    }
    cs = norm_s(cl);
    temp_l = 0;
    move16();

    FOR (i = 0; i < M; i ++)
    {
        yy_fx[i] = shl(yy_fx[i], cs);
        move16();
        temp_l = L_mac(temp_l, mult(yy_fx[i], shl(w_fx[i],2) ), yy_fx[i]);
    }
    cs = shl(cs, 1);
    temp_l = L_shr(temp_l, cs);
    temp_l = Mult_32_16(temp_l, LSF_1_OVER_256SQ); /* Q-4 */

    /* Recover the quantized LSF */
    Copy(x_q_fx, y_fx, M);

    return temp_l;
}

static void FFT_Mid_Interpol_16k_fx(
    Word32 Bin_Ener_old[],         /* i/o: Old 2nd FFT Bin energy (128)               */
    Word32 Bin_Ener[],             /* i  : Current 2nd FFT Bin energy (128)           */
    Word32 Bin_Ener_mid[]          /* o  : LP weighting filter (numerator)            */
)
{
    Word16 i;

    FOR( i=0; i<L_FFT/2; i++ )
    {
        /* Interpolation */
        Bin_Ener_mid[i] = L_shr(L_add(Bin_Ener_old[i],Bin_Ener[i]),1);

        /* Memory update */
        Bin_Ener_old[i] = Bin_Ener[i];
        move32();
    }

    return;
}


/*========================================================================*/
/* FUNCTION : lsf_mid_enc_fx()                                            */
/*------------------------------------------------------------------------*/
/* PURPOSE : Mid-frame LSF quantization                                   */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                      */
/* _ (Word16) coder_type  : Coder type                                    */
/* _ (Word16) bwidth      : input signal bandwidth                        */
/* _ (Word16) int_fs      : internal (ACELP) sampling frequency           */
/* _ (Word32) core_brate  : core bitrate                                  */
/* _ (Word32)  ppp_mode   : PPP mode                                      */
/* _ (Word32)  nelp_mode  : NELP mode                                     */
/* _ (Word16[]) qlsp0     : quantized LSPs from frame beginning       Q15 */
/* _ (Word16[]) qlsp1     : quantized LSPs from frame end             Q15 */
/* _ (Word16[]) Bin_Ener  : per bin log energy spectrum            Q_ener */
/* _ (Word16) Q_ener      :                                               */
/* _ (Word16) ppp_mode    :                                               */
/* _ (Word16) nelp_mode   :                                               */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                               */
/* _ (Word16[]) lsp       : quantized LSPs                            Q15 */
/* _ (Word16[]) Bin_Ener_old : per bin old log energy spectrum     Q_ener */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                     */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                     */
/* _ None                                                                 */
/*========================================================================*/


static void lsf_mid_enc_fx(
    Encoder_State_fx *st_fx,
    const Word16 int_fs,            /* i  : internal (ACELP) sampling frequency*/
    const Word16 qlsp0[],           /* i  : quantized LSPs from frame beginning*/
    const Word16 qlsp1[],           /* i  : quantized LSPs from frame end      */
    Word16 lsp[],                   /* i/o: mid-frame LSP                      */
    const Word16 coder_type,        /* i  : coding type                        */
    const Word16 bwidth,            /* i  : input signal bandwidth             */
    const Word32  core_brate,       /* i  : core bitrate                       */
    Word32 Bin_Ener_old[],          /* i/o: per bin old log energy spectrum    */
    Word32 Bin_Ener[],              /* i  : per bin log energy spectrum        */
    Word16 Q_ener,                  /* i  : Q value of Bin_ener                */
    Word16 ppp_mode,
    Word16 nelp_mode
)
{
    Word16 lsf[M], qlsf[M], qlsf1[M], qlsf0[M], wghts[M];
    Word32 err, err_min;
    Word16 j, k, idx, nb_bits = 0, size = 0;
    Word32 Bin_Ener_mid[L_FFT/2];
    Word32 L_tmp;
    Word16 tmp, k1;
    const Word16 *ratio = NULL;

    /* convert LSPs to LSFs */
    lsp2lsf_fx( lsp, lsf, M, int_fs);
    lsp2lsf_fx( qlsp0, qlsf0, M, int_fs);
    lsp2lsf_fx( qlsp1, qlsf1, M, int_fs);

    /* calculate weights */
    FFT_Mid_Interpol_16k_fx( Bin_Ener_old, &Bin_Ener[L_FFT/2], Bin_Ener_mid );

    /* LSF weighting */
    Unified_weighting_fx( Bin_Ener_mid, Q_ener, lsf, wghts, sub(bwidth, NB) == 0, sub(coder_type, UNVOICED) == 0, int_fs, M );
    move16();
    /* codebook selection, number of bits, size of the codebook */
    test();
    IF ( ppp_mode == 0 && nelp_mode == 0 )
    {
        nb_bits = mid_LSF_bits_tbl[LSF_BIT_ALLOC_IDX_fx(core_brate, coder_type)];
        move16();

        /* codebook selection */
        IF ( sub(coder_type, VOICED) == 0)
        {
            SWITCH ( nb_bits )
            {
            case 6:
                {
                    ratio = tbl_mid_voi_wb_6b_fx;
                    move16();
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_voi_wb_5b_fx;
                    move16();
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_voi_wb_4b_fx;
                    move16();
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_voi_wb_3b_fx;
                    move16();
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_voi_wb_2b_fx;
                    move16();
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_voi_wb_5b_fx;
                    move16();
                    BREAK;
                }
            }
        }
        ELSE IF ( sub(coder_type, UNVOICED) == 0 )
        {
            SWITCH ( nb_bits )
            {
            case 6:
                {
                    ratio = tbl_mid_unv_wb_6b_fx;
                    move16();
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_unv_wb_5b_fx;
                    move16();
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_unv_wb_4b_fx;
                    move16();
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_unv_wb_3b_fx;
                    move16();
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_unv_wb_2b_fx;
                    move16();
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_unv_wb_5b_fx;
                    move16();
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
                    move16();
                    BREAK;
                }
            case 5:
                {
                    ratio = tbl_mid_gen_wb_5b_fx;
                    move16();
                    BREAK;
                }
            case 4:
                {
                    ratio = tbl_mid_gen_wb_4b_fx;
                    move16();
                    BREAK;
                }
            case 3:
                {
                    ratio = tbl_mid_gen_wb_3b_fx;
                    move16();
                    BREAK;
                }
            case 2:
                {
                    ratio = tbl_mid_gen_wb_2b_fx;
                    move16();
                    BREAK;
                }
            default:
                {
                    ratio = tbl_mid_gen_wb_5b_fx;
                    move16();
                    BREAK;
                }
            }
        }

        size = (Word16) pow2[nb_bits];
        move16();
    }
    ELSE IF ( sub(ppp_mode, 1) == 0 )
    {
        ratio = tbl_mid_voi_wb_1b_fx;
        move16();
        nb_bits = 1;
        move16();
        size = 2;
        move16();
    }
    ELSE IF ( sub(nelp_mode, 1) == 0 )
    {
        ratio = tbl_mid_unv_wb_4b_fx;
        move16();
        nb_bits = 4;
        move16();
        size = 16;
        move16();
    }

    /* loop over codevectors */
    err_min = MAXINT32;
    move16();
    idx = 0;
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
                move16();
            }

            tmp = sub(lsf[j],qlsf[j]);
            /*        err +=  wghts[j] * ftemp * ftemp; */
            /* tmp is usually very small, we can have some extra precision with very rare saturation */
            tmp = shl(tmp, 4);
            tmp = mult_r(tmp, tmp);
            err = L_mac(err, tmp, shl(wghts[j],2) );
        }
        /*    err = L_shl(err,Wscale); */
        err = Mult_32_16(err,LSF_1_OVER_256SQ);
        /*    err = Mult_32_16(err,Wmult); */

        IF ( L_sub(err,err_min) < 0 )
        {
            err_min = L_add(err, 0);
            idx = k;
            move16();
        }
        k1+=M;
        move16();
    }

    /* calculate the quantized LSF vector */
    FOR ( j = 0; j < M; j++ )
    {
        /*     qlsf[j] = (1.0f - ratio[idx*M+j]) * qlsf0[j] + ratio[idx*M+j] * qlsf1[j]; */
        L_tmp = L_mult(sub(0x2000, ratio[idx*M+j]), qlsf0[j]);
        L_tmp = L_mac(L_tmp,ratio[idx*M+j],qlsf1[j]);
        qlsf[j] = round_fx(L_shl(L_tmp,2));

        test();
        test();
        IF ( j > 0 && sub(j, M) < 0 && sub(qlsf[j], add(qlsf[j-1], LSF_GAP_MID_FX)) < 0 )
        {
            qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
            move16();
        }
    }

    reorder_lsf_fx( qlsf, LSF_GAP_MID_FX, M, int_fs );

    /* convert LSFs back to LSPs */
    lsf2lsp_fx( qlsf, lsp, M, int_fs);
    push_indice_fx( st_fx, IND_MID_FRAME_LSF_INDEX, idx, nb_bits );

    return;
}

