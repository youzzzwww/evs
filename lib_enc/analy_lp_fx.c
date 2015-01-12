/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "rom_enc_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * analy_lp()
 *
 * Perform LP analysis
 *
 * - autocorrelations + lag windowing
 * - Levinson-Durbin algorithm to find A(z)
 * - convert A(z) to LSPs
 * - find interpolated LSPs and convert back to A(z) for all subframes
 * - update LSPs for the next frame
 *-------------------------------------------------------------------*/

void analy_lp_fx(
    const Word16 speech[],    /* i  : pointer to the speech frame                      */
    const Word16 L_frame,     /* i  : length of the frame                              */
    const Word16 L_look,      /* i  : look-ahead                                       */
    Word32 *ener,       /* o  : residual energy from Levinson-Durbin             */
    Word16 A[],         /* o  : A(z) filter coefficients                         */
    Word16 epsP_h[],    /* o  : LP analysis residual energies for each iteration */
    Word16 epsP_l[],    /* o  : LP analysis residual energies for each iteration */
    Word16 lsp_new[],   /* o  : current frame LSPs                               */
    Word16 lsp_mid[],   /* o  : current mid-frame LSPs                           */
    Word16 lsp_old[],   /* i/o: previous frame unquantized LSPs                  */
    const Word16 Top[2],      /* i  : open loop pitch lag                              */
    const Word16 Tnc[2],      /* i  : open loop pitch gain                             */
    const Word32 Core_sr,     /* i  : Internal core sampling rate                      */
    Word16 Q_new,
    Word16 *Q_r
)
{
    Word16 r_h[M+1]; /* Autocorrelations of windowed speech MSB */
    Word16 r_l[M+1]; /* Autocorrelations of windowed speech LSB */
    Word32 LepsP[M+1];
    Word16 i, i_subfr, wind_length = 0;
    Word16 *lsp;
    const Word16 *wind = NULL;
    const Word16 *pt;
    Word16 half_frame;

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        wind_length = L_LP;
        move16();
        wind = Assym_window_W16fx;
    }
    ELSE /* L_frame == L_FRAME16k */
    {
        wind_length = L_LP_16k;
        move16();
        wind = assym_window_16k_fx;
    }
    lsp = lsp_mid;
    half_frame = shr(L_frame,1);

    FOR( i_subfr = 0; i_subfr <= 1; i_subfr++ )
    {
        pt = speech + sub(add(half_frame, L_look), wind_length);
        half_frame = shl(half_frame,1);

        /* Autocorrelations */
        autocorr_fx(pt, M, r_h, r_l, &Q_r[1-i_subfr], wind_length, wind, 0, 0 );

        /* Lag windowing */
        adapt_lag_wind( r_h, r_l, M, Top[i_subfr], Tnc[i_subfr], Core_sr );

        /* Levinson-Durbin */
        E_LPC_lev_dur(r_h, r_l, A, LepsP, M, NULL);
        FOR (i = 0; i <= M; i++)
        {
            L_Extract(LepsP[i], &epsP_h[i], &epsP_l[i]);
        }
        /*Q_r[... might not be needed from external...*/
        Q_r[1-i_subfr] = add(Q_r[1-i_subfr], shl(Q_new, 1));
        move16();

        /* Conversion of A(z) to LSPs */
        E_LPC_a_lsp_conversion( A, lsp, lsp_old, M );

        lsp = lsp_new;
    }

    /* LSP interpolation */
    int_lsp4_fx( L_frame, lsp_old, lsp_mid, lsp_new, A , M, 0, 0 );
    Copy (lsp_new, lsp_old, M);
    *ener = L_Comp(epsP_h[M],epsP_l[M]);
    move32();

    return;

}


/*-------------------------------------------------------------------*
 * analy_lp_AMR_WB()
 *
 * Perform LP analysis for AMR-WB IO mode
 *
 * - autocorrelations + lag windowing
 * - Levinson-Durbin algorithm to find A(z)
 * - convert A(z) to ISPs
 * - find interpolated ISPs and convert back to A(z) for all subframes
 * - update ISPs for the next frame
 *-------------------------------------------------------------------*/

void analy_lp_AMR_WB_fx(
    const Word16 speech[],    /* i  : pointer to the speech frame                      */
    Word32 *ener,       /* o  : residual energy from Levinson-Durbin             */
    Word16 A[],         /* o  : A(z) filter coefficients                         */
    Word16 epsP_h[],    /* o  : LP analysis residual energies for each iteration */
    Word16 epsP_l[],    /* o  : LP analysis residual energies for each iteration */
    Word16 isp_new[],   /* o  : current frame ISPs                               */
    Word16 isp_old[],   /* i/o: previous frame unquantized ISPs                  */
    Word16 isf_new[],   /* o  : current frame ISPs                               */
    Word16 Top,         /* i  : open loop pitch lag                              */
    Word16 Tnc,         /* i  : open loop pitch gain                             */
    Word16 Q_new,
    Word16 *Q_r
)
{
    Word16 r_h[M+1]; /* Autocorrelations of windowed speech MSB */
    Word16 r_l[M+1]; /* Autocorrelations of windowed speech LSB */
    Word32 LepsP[M+1];
    Word16 i, wind_length = 0;
    const Word16 *wind;

    /* Initialization */
    wind_length = L_LP_AMR_WB;
    move16();
    wind = Hamcos_Window;

    /* Autocorrelations */
    autocorr_fx( speech - L_SUBFR, M, r_h, r_l, &Q_r[0], wind_length, wind, 0, 0 );

    /* Lag windowing */
    adapt_lag_wind( r_h, r_l, M, Top, Tnc, INT_FS_FX );

    /* Levinson-Durbin  */
    /*lev_dur( A, r, M, epsP );*/
    E_LPC_lev_dur(r_h, r_l, A, LepsP, M, NULL);
    FOR (i = 0; i <= M; i++)
    {
        L_Extract(LepsP[i], &epsP_h[i], &epsP_l[i]);
    }
    /*Q_r[... might not be needed from external...*/
    Q_r[0] = add(Q_r[0], shl(Q_new, 1));
    move16();

    E_LPC_a_lsf_isf_conversion( A, isf_new, stable_ISF_fx, M, 0 );
    E_LPC_isf_isp_conversion( isf_new, isp_new, M );

    /* ISP interpolation */
    int_lsp_fx( L_FRAME, isp_old, isp_new, A, M, 0, interpol_isp_amr_wb_fx, 1 );

    /**ener = epsP[M];*/
    *ener = L_Comp(epsP_h[M],epsP_l[M]);
    move32();

    /* updates */
    Copy( isp_new, isp_old, M );

    return;
}
