/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


#define BETA_FX 24576

void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m);

/*===========================================================================*/
/* FUNCTION : FEC_lsf_estim()                                                */
/*---------------------------------------------------------------------------*/
/* PURPOSE :                                                                 */
/*              * - LSF estimation in case of FEC                            */
/*              * - LSP calculation                                          */
/*              * - A(z) calculation                                         */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :                                                         */
/* _ (Struct)   st_fx            : decoder static memory                     */
/* _ (Word16[]) st_fx->mem_AR_fx : AR memory of LSF quantizer                */
/*                               (past quantized LSFs without mean) Q(x2.56) */
/* _ (Word16[]) st_fx->lsf_old_fx : old LSF vector at the                    */
/*                                    end of the frame              Q(x2.56) */
/* _ (Word16) st_fx->L_frame_fx : length of the frame                        */
/*                                                                           */
/* _ (Word16) st_fx->nbLostCmpt : FEC - counter of consecutive bad frames    */
/*                                       previous frame             Q(x2.56) */
/* _ (Word16[]) st_fx->lsp_old_fx : old LSP vector at the                    */
/*                                    end of the frame                   Q15 */
/* _ (Word16) st_fx->last_coder_type_fx : previous coder type                */
/* _ (Word16) st_fx->last_coder_type_fx : FEC - clas of last good received   */
/* _ (Word16) st_fx->Last_GSC_pit_band_idx_fx :AC mode (GSC) - Last          */
/*                                                    pitch band index       */
/* _ (Word16) st_fx->stab_fac_fx : LSF stability factor                  Q15 */
/* _ (Word16[]) st_fx->mem_MA_fx : MA memory of LSF quantizer                */
/*                                 (past quantized residual)        Q(x2.56) */
/* _ (Word16[]) st_fx->lsf_adaptive_mean_fx    : FEC - adaptive mean LSF     */
/*                                              vector for FEC      Q(x2.56) */
/* _ (Word16) L_frame          : length of the frame                         */
/* _ (Word16) coder_type       : coding type                                 */
/* _ (Word16) bwidth_fx        : input signal bandwidth                      */
/* _ (Word16) clas             : signal class                                */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/* _ (Word16*) Aq              : LP filter coefficient                   Q12 */
/* _ (Word16*) lsf             : LP filter coefficient              Q(x2.56) */
/* _ (Word16*) lsp             : LP filter coefficient                   Q15 */
/*---------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->mem_AR_fx : AR memory of LSF quantizer                */
/*                               (past quantized LSFs without mean) Q(x2.56) */
/* _ (Word16[]) st_fx->mem_MA_fx :MA memory of LSF quantizer                 */
/*                                 (past quantized residual)        Q(x2.56) */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                        */
/* _ None                                                                    */
/*===========================================================================*/

void FEC_lsf_estim_fx(
    Decoder_State_fx *st,            /* i/o: Decoder static memory                        */
    const Word16 L_frame,          /* i  : length of the frame                          */
    Word16 *Aq,              /* o  : calculated A(z) for 4 subframes              */
    Word16 *lsf,             /* o  : estimated LSF vector                         */
    Word16 *lsp              /* o  : estimated LSP vector                         */
)
{
    Word16 lsf_mean[M];
    Word16 alpha;
    Word16 tmp;
    Word16 i;
    /* Update inital guess to something stable, with proper sampling frequency and format (ISF/LSF)*/
    IF ( st->Opt_AMR_WB_fx )
    {
        Copy( Mean_isf_wb, lsf_mean, M );
    }
    ELSE
    {
        /* 12.8kHz ACELP sampling */
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            IF (st->bwidth_fx==NB)
            {
                Copy( GENB_Ave_fx, lsf_mean, M );
            }
            ELSE
            {
                Copy( GEWB_Ave_fx, lsf_mean, M );
            }
        }
        /* 16kHz ACELP sampling */
        ELSE
        {
            Copy( GEWB2_Ave_fx, lsf_mean, M );
        }
    }

    /*----------------------------------------------------------------------*
     * Initialize the alpha factor
     *----------------------------------------------------------------------*/
    test();
    test();
    test();
    test();
    IF( sub(st->nbLostCmpt,3) <= 0 )
    {
        IF( sub(st->last_coder_type_fx,UNVOICED)  == 0 )
        {
            /* clearly unvoiced */
            alpha = _ALPHA_UU_FX;
            move16();
        }
        ELSE IF( sub(st->last_coder_type_fx,AUDIO) == 0 || sub(st->last_good_fx,INACTIVE_CLAS) == 0 )
        {
            test();
            IF(st->Last_GSC_pit_band_idx_fx > 0 && sub(st->nbLostCmpt,1) > 0 )
            {
                alpha = 26214;
                move16();
            }
            ELSE IF( sub(st->nbLostCmpt,5) <= 0 )
            {
                alpha = 32604;
                move16();
            }
            ELSE
            {
                alpha = 31130;
                move16();
            }
        }
        ELSE IF( sub(st->last_good_fx,UNVOICED_CLAS) == 0 )
        {
            IF( sub(st->nbLostCmpt,1) <= 0 )
            {
                /* if stable, do not flatten the spectrum in the 1st erased frame  */
                alpha = add(mult(st->stab_fac_fx, 32768 - _ALPHA_U_FX_X_2), _ALPHA_U_FX_X_2);
            }
            ELSE IF( sub(st->nbLostCmpt,2) == 0 )
            {
                alpha = sub(_ALPHA_U_FX_X_2,shr(_ALPHA_U_FX,1));   /* 0.6 */
            }
            ELSE
            {
                /* go rapidly to CNG spectrum  */
                alpha = _ALPHA_U_FX;
                move16();
            }
        }
        ELSE IF( sub(st->last_good_fx,UNVOICED_TRANSITION) == 0 )
        {
            alpha = _ALPHA_UT_FX;
            move16();
        }
        ELSE IF(( sub(st->last_good_fx,VOICED_CLAS) == 0 || sub(st->last_good_fx,ONSET) == 0 ) && sub(st->nbLostCmpt,3) <= 0 )
        {
            /* clearly voiced -  mild convergence to the CNG spectrum for the first 3 erased frames */
            alpha = _ALPHA_V_FX;
            move16();
        }
        ELSE IF( sub(st->last_good_fx,SIN_ONSET) == 0)
        {
            alpha = _ALPHA_S_FX;
            move16();
        }
        ELSE
        {
            /* long erasures and onsets - rapid convergence to the CNG spectrum */
            alpha = _ALPHA_VT_FX;
            move16();
        }
    }
    ELSE
    {
        Word16 exp = 15;
        alpha = Inv16(st->nbLostCmpt, &exp); /*1.f/bfi_cnt;*/
        alpha = shl(alpha,exp);
    }

    /*----------------------------------------------------------------------*
     * Extrapolate LSF vector
     *----------------------------------------------------------------------*/
    tmp =  sub(32767, alpha);
    /* extrapolate the old LSF vector */
    FOR(i=0; i<M; i++)
    {
        /* calculate mean LSF vector */
        /* lsf_mean[i] = BETA * lsf_mean[i] + (1-BETA) * st->lsf_adaptive_mean[i]; */
        lsf_mean[i] = mac_r(L_mult(BETA_FEC_FX, lsf_mean[i]), 32768-BETA_FEC_FX, st->lsf_adaptive_mean_fx[i]);
        move16();
        /* move old LSF vector towards the mean LSF vector */
        /* lsf[i] = alpha * st->lsf_old[i] + (1.0f - alpha) * lsf_mean[i]; */
        lsf[i] = mac_r(L_mult(alpha, st->lsf_old_fx[i]), tmp, lsf_mean[i]);
        move16();/* towards the CNG spectral envelope */
    }
    /* check LSF stability through LSF ordering */
    IF ( st->Opt_AMR_WB_fx )
    {
        reorder_isf_fx( lsf, ISF_GAP_FX, M, Fs_2 );
    }
    ELSE
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            reorder_lsf_fx( lsf, MODE1_LSF_GAP_FX, M,INT_FS_FX);
        }
        ELSE/* L_frame == L_FRAME16k */
        {
            reorder_lsf_fx( lsf, MODE1_LSF_GAP_FX, M, INT_FS_16k_FX );
        }
    }
    /* update the AR memory to be used in the next frame */
    Copy(lsf,st->mem_AR_fx,M);

    /* update the MA memory to be used in the next frame */
    FOR(i=0; i<M; i++)
    {
        /* subtract the mean vector */
        tmp = sub(lsf[i], lsf_mean[i]);

        /* st->mem_MA[i] = (float)(tmp - MU_MA * st->mem_MA[i]); Update with quantized prediction error for MA model */
        /* mem_MA[i] *= 0.5f Attenuate the MA Q memory */
        /* the L_mult0 divides result by 2, for 'tmp' we do it with 16384 */
        st->mem_MA_fx[i] = mac_r(L_mult0(st->mem_MA_fx[i], -MU_MA_FX), tmp, 16384);
        move16();
    }

    /* convert LSFs to LSPs */
    IF ( st->Opt_AMR_WB_fx )
    {
        E_LPC_isf_isp_conversion( lsf, lsp, M);
    }
    ELSE
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            lsf2lsp_fx( lsf, lsp, M, INT_FS_FX );
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            lsf2lsp_fx( lsf, lsp, M, INT_FS_16k_FX);
        }
    }
    /*----------------------------------------------------------------------*
     * Interpolate LSP vector and find A(z)
     *----------------------------------------------------------------------*/
    IF ( st->Opt_AMR_WB_fx )
    {
        int_lsp_fx( L_frame, st->lsp_old_fx, lsp, Aq, M, 0, interpol_isp_amr_wb_fx, 1 );
    }
    ELSE
    {
        int_lsp_fx( L_frame, st->lsp_old_fx, lsp, Aq, M, 0, interpol_frac_fx, 0 );
    }

    return;


}

