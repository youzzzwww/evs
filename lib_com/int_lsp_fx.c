/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "stl.h"

/*========================================================================*/
/* FUNCTION : int_lsp4_fx()												  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Interpolate LSPs find the A[z] parameters for all subframes  */
/*			 by interpolating between old end-frame LSPs, current		  */
/*			 mid-frame LSPs and current end-frame LSPs					  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) L_frame  : length of the frame							  */
/* _ (Word16) m	  : order of LP filter									  */
/* _ (Word16) clas	  : signal frame class    							  */
/* _ (Word16[]) lsp_old	  : LSPs from past frame               Q15		  */
/* _ (Word16[]) lsp_mid	  : LSPs from mid-frame				   Q15		  */
/* _ (Word16[]) lsp_new	  : LSPs from present frame            Q15		  */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/* _ (Word16*) Aq		: LP coefficients in both subframes    Q12		  */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void int_lsp4_fx(
    const Word16 L_frame,    /* i  : length of the frame                   */
    const Word16 lsp_old[],  /* i  : LSPs from past frame               Q15*/
    const Word16 lsp_mid[],  /* i  : LSPs from mid-frame				  Q15*/
    const Word16 lsp_new[],  /* i  : LSPs from present frame            Q15*/
    Word16 *Aq,        /* o  : LP coefficients in both subframes  Q12*/
    const Word16 m,          /* i  : order of LP filter                    */
    const Word16 clas,       /* i  : signal frame class                    */
    Word16 relax_prev_lsf_interp /* i  : relax prev frame lsf interp after erasure */
)
{
    Word16 lsp[M16k];
    Word16 i,j, k;
    Word32 L_tmp;
    const Word16 *pt_int_coeffs;

    IF (sub(clas, SIN_ONSET) == 0)
    {
        E_LPC_f_lsp_a_conversion( lsp_new, Aq, m );     /* Find A(z) (not interpolated) */
        FOR( i=1; i<L_frame/L_SUBFR; i++ )
        {
            Copy( Aq, Aq + i*(m+1), m+1 );
        }
    }
    ELSE
    {
        IF( sub(L_frame,L_FRAME) == 0)
        {
            IF ( sub(relax_prev_lsf_interp,1) == 0)
            {
                pt_int_coeffs = interpol_frac_mid_relaxprev_12k8_fx;
            }
            ELSE IF ( sub(relax_prev_lsf_interp,2) == 0 )
            {
                pt_int_coeffs = interpol_frac_mid_FEC_fx;
            }
            ELSE IF ( sub(relax_prev_lsf_interp,-1) == 0 )
            {
                pt_int_coeffs = interpol_frac_mid_relaxprev_pred_12k8_fx;
            }
            ELSE
            {
                pt_int_coeffs = interpol_frac_mid_fx;
            }
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            IF ( sub(relax_prev_lsf_interp,1) == 0 )
            {
                pt_int_coeffs = interpol_frac_mid_relaxprev_16k_fx;
            }
            ELSE IF ( sub(relax_prev_lsf_interp,2) == 0 )
            {
                pt_int_coeffs = interpol_frac_mid_16k_FEC_fx;
            }
            ELSE IF ( sub(relax_prev_lsf_interp,-1) == 0 )
            {
                pt_int_coeffs = interpol_frac_mid_relaxprev_pred_16k_fx;
            }
            ELSE
            {
                pt_int_coeffs = interpol_frac_mid_16k_fx;
            }
        }
        k = sub(shr(L_frame,6),1);
        FOR( j=0; j<k; j++ )
        {
            FOR( i=0; i<m; i++ )
            {
                L_tmp = L_mult(lsp_old[i], *pt_int_coeffs); /*Q31 */
                L_tmp = L_mac(L_tmp, lsp_mid[i], *(pt_int_coeffs+1)); /*Q31 */
                lsp[i] = mac_r(L_tmp, lsp_new[i], *(pt_int_coeffs+2));
                move16();
            }
            pt_int_coeffs += 3;
            move16();

            E_LPC_f_lsp_a_conversion( lsp, Aq, m );
            Aq += (m+1);
            move16();
        }

        /* Last subframe */
        E_LPC_f_lsp_a_conversion( lsp_new, Aq, m );
    }

    return;
}

/*---------------------------------------------------------------------*
 * int_lsp_fx()
 *
 * Find the interpolated LSP parameters for all subframes
 *---------------------------------------------------------------------*/

void int_lsp_fx(
    const Word16 L_frame,     /* i  : length of the frame               */
    const Word16 lsp_old[],   /* i  : LSPs from past frame              */
    const Word16 lsp_new[],   /* i  : LSPs from present frame           */
    Word16 *Aq,         /* o  : LP coefficients in both subframes */
    const Word16 m,           /* i  : order of LP filter                */
    const Word16 clas,        /* i  : signal frame class                */
    const Word16 *int_coeffs, /* i  : interpolation coefficients        */
    const Word16 Opt_AMR_WB   /* i  : flag indicating AMR-WB IO mode    */
)
{
    Word16 lsp[M], fnew, fold;
    Word16 i, k;
    const Word16 *pt_int_coeffs=NULL;
    Word32 L_tmp;
    Word16 tmp;

    tmp = shr(L_frame,6);  /*L_frame/L_SUBFR */

    IF( sub(clas,SIN_ONSET) == 0 )
    {
        /* Do not interpolate A(z) */

        IF ( Opt_AMR_WB )
        {
            E_LPC_f_isp_a_conversion( lsp_new, Aq, m );
        }
        ELSE
        {
            E_LPC_f_lsp_a_conversion(lsp_new, Aq, m);
        }
        FOR( i=0; i<tmp; i++ )
        {
            Copy( Aq, Aq + i*(m+1), m+1 );
        }
    }
    ELSE
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            pt_int_coeffs = int_coeffs;
            move16();
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            pt_int_coeffs = interpol_frac_16k_fx;
        }
        FOR( k=0; k<tmp; k++ )
        {
            fnew = pt_int_coeffs[k];
            move16();
            fold = sub(32767, fnew); /* 1.0 - fac_new */

            FOR (i = 0; i < m; i++)
            {
                L_tmp = L_mult(lsp_old[i], fold);
                lsp[i] = mac_r(L_tmp, lsp_new[i], fnew);
                move16();
            }
            IF ( Opt_AMR_WB )
            {
                E_LPC_f_isp_a_conversion( lsp, Aq, m );
            }
            ELSE
            {
                E_LPC_f_lsp_a_conversion(lsp, Aq, m);
            }
            Aq +=(m+1);
        }
    }

    return;
}
