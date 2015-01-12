/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "prot_fx.h"
#include "stl.h"
#include "rom_com_fx.h"

#include "basop_util.h"
#include "rom_basop_util.h"

void getLookAheadResSig( Word16 *speechLookAhead, Word16 *A_3Q12, Word16 *res, Word16 L_frame, Word16 numSubFrame )
{
    Word16 *p_A;
    Word16 i_subfr;
    Word16 subfr_len[2] = { L_SUBFR, L_SUBFR };

    if( sub( L_FRAME16k, L_frame )>0 )
    {
        subfr_len[1] = 48;
        move16(); /* 0.75 * L_SUBFR(64) */
    }

    p_A = A_3Q12;
    FOR(i_subfr=0; i_subfr<numSubFrame*L_SUBFR; i_subfr+=L_SUBFR)
    {
        /* calculate residual signal */
        Residu3_fx( p_A,
                    &speechLookAhead[i_subfr],
                    &res[i_subfr],
                    subfr_len[shr(i_subfr,6)],
                    0
                  );

        /* pointer initialization */
        p_A += (M+1);
    }

    return;
}


void updateLSFForConcealment( HANDLE_PLC_ENC_EVS decState, Word16 *lsf_14Q1, Word16 m )
{
    Word16 i;
    Word32 L_tmp = 0;
    const Word16 divide_by_3_Q15 = 10923;

    FOR (i=0; i<m; i++)
    {
        L_tmp = L_mult( divide_by_3_Q15, decState->lsfoldbfi1_14Q1[i] );
        L_tmp = L_mac( L_tmp, divide_by_3_Q15, decState->lsfoldbfi0_14Q1[i] );
        decState->lsf_adaptive_mean_14Q1[i] = mac_r( L_tmp, divide_by_3_Q15, lsf_14Q1[i] );
        decState->lsfoldbfi1_14Q1[i] = decState->lsfoldbfi0_14Q1[i];
        move16();
        decState->lsfoldbfi0_14Q1[i] = lsf_14Q1[i];
        move16();
    }

    return;
}

void getConcealedLP( HANDLE_PLC_ENC_EVS memDecState, Word16 *AqCon, const Word16 lsfBase[], Word16 last_good, Word16 L_frame)
{
    Word16 *lsf;
    Word16 lsp[(NB_DIV+1)*M];
    Word32 int_fs;

    move16();
    lsf = memDecState->lsf_con;


    dlpc_bfi( L_frame, &lsf[0], memDecState->lsfold_14Q1, last_good,
              1, memDecState->mem_MA_14Q1, memDecState->mem_AR, &(memDecState->stab_fac_Q15), memDecState->lsf_adaptive_mean_14Q1,
              1,
              NULL,
              0,
              NULL,
              NULL,
              lsfBase,
              0
            );
    Copy( memDecState->lspold_Q15, lsp, M );

    int_fs = INT_FS_FX;
    move32();
    if( sub(L_frame,L_FRAME_16k) == 0 )
    {
        int_fs = INT_FS_16k_FX;
        move32();
    }
    lsf2lsp_fx( lsf, &lsp[M], M, int_fs );

    int_lsp_fx( L_frame, &lsp[0], &lsp[M], AqCon, M, last_good, interpol_frac_fx, 0 );

    return;
}

void getConcealedLSF( HANDLE_PLC_ENC_EVS memDecState, const Word16 lsfBase[], Word16 last_good, Word16 L_frame)
{
    Word16 *lsf = memDecState->lsf_con;


    dlpc_bfi( L_frame, &lsf[0], memDecState->lsfold_14Q1, last_good,
              1, memDecState->mem_MA_14Q1, memDecState->mem_AR, &(memDecState->stab_fac_Q15), memDecState->lsf_adaptive_mean_14Q1,
              1,
              NULL,
              0,
              NULL,
              NULL,
              lsfBase,
              0
            );

    return;
}

static void reorder_lsfs(Word16 *lsf, const Word16 min_dist, const Word16 n, const Word32 sr_core);

void RecLpcSpecPowDiffuseLc( Word16 *lspq, Word16 *lsp_old, Word16 *lsfq, Decoder_State_fx *st)
{
    const Word16 *means;
    Word16 lsf_old[M];
    Word16 i;

    means = PlcGetLsfBase ( st->lpcQuantization,
                            st->narrowBand,
                            st->sr_core );

    Copy( st->lsf_old_fx, lsf_old, M );

    modify_lsf( lsf_old, M, st->sr_core );

    lsf2lsp_fx( lsf_old, lsp_old, M, st->sr_core );

    FOR ( i=0; i<M; i++ )
    {
        lsfq[i] = add(st->mem_MA_fx[i], means[i]);
        move16();
    }
    sort_fx( lsfq, 0, sub(M, 1) );

    reorder_lsfs( lsfq, LSF_GAP_FX, M, st->sr_core );
    lsf2lsp_fx( lsfq, lspq, M, st->sr_core );


    return;
}

void modify_lsf(
    Word16 *lsf,
    const Word16 n,
    const Word32 sr_core
)
{
    Word16 i, k, th_x1p28_Q14;
    Word16 gap, gap_sum;


    th_x1p28_Q14 = FL2WORD16_SCALE(1900.0f*1.28f, 14);
    move16();
    if( L_sub( sr_core, 16000 ) == 0 )
    {
        th_x1p28_Q14 = FL2WORD16_SCALE(2375.0f*1.28f, 14);
        move16();
    }

    FOR ( i=1; i<n; i++)
    {
        IF ( sub(lsf[i], th_x1p28_Q14) >= 0 )
        {
            BREAK;
        }
    }

    gap = mult_r(lsf[i - 1], InvIntTable[i]);

    move16();
    gap_sum = gap;
    i = sub(i,1);
    FOR(k = 0; k < i; k++)
    {
        move16();
        lsf[k] = gap_sum;
        gap_sum = add(gap_sum, gap);
    }

}


static void reorder_lsfs(
    Word16 *lsf,       /* i/o: vector of lsfs in the frequency domain (0..0.5)*/
    const Word16 min_dist0,  /* i  : minimum required distance */
    const Word16 n,          /* i  : LPC order                 */
    const Word32 sr_core     /* i  : input sampling frequency  */
)
{
    Word16 i;
    Word16 curr_min_dist;
    Word16 min_dist_fac2;
    Word16 min_dist_fac3;
    Word16 lsf_min;
    Word16 lsf_max;
    Word16 fs2;
    Word16 th1, th2;
    Word16 min_dist;


    fs2 = FL2WORD16_SCALE(6400.0 * 1.28, 14);
    move16();

    if(L_sub(sr_core, 16000) == 0)
    {
        fs2 = FL2WORD16_SCALE(8000.0 * 1.28, 14);
        move16();
    }

    /*-----------------------------------------------------------------*
     * Verify the LSF ordering and minimum GAP
     *-----------------------------------------------------------------*/
    IF( L_sub( sr_core, 16000 )==0 )
    {
        th1 = 3200;
        move16();
        th2 = 6080;
        move16();
        min_dist = add( min_dist0, shr(min_dist0,2) );
    }
    ELSE
    {
        th1 = 2560;
        move16();
        th2 = 4864;
        move16();
        min_dist = min_dist0;
        move16();
    }
    min_dist_fac2 = shl(min_dist, 1);
    min_dist_fac3 = add(min_dist, min_dist_fac2);
    curr_min_dist = min_dist_fac3;
    move16();

    lsf_min = curr_min_dist;
    move16();

    FOR (i = 0; i < n; i++)
    {
        IF (sub(lsf[i], th1) > 0)
        {
            curr_min_dist = min_dist_fac2;
            move16();
        }
        ELSE
        {
            if (sub(lsf[i], th2) > 0)
            {
                curr_min_dist = min_dist;
                move16();
            }
        }

        if (sub(lsf[i], lsf_min) < 0)
        {
            lsf[i] = lsf_min;
            move16();
        }

        lsf_min = add(lsf[i], curr_min_dist);

    }

    /*------------------------------------------------------------------------------------------*
     * Reverify the LSF ordering and minimum GAP in the reverse order (security)
     *------------------------------------------------------------------------------------------*/

    lsf_max = sub(fs2, curr_min_dist);

    IF (sub(lsf[n-1], lsf_max) > 0)    /* If danger of unstable filter in case of resonance in HF */
    {
        FOR (i = sub(n, 1); i >= 0; i--) /* Reverify the minimum ISF gap in the reverse direction */
        {
            IF (sub(lsf[i], th2) <= 0)
            {
                curr_min_dist = min_dist_fac2;
                move16();
            }
            ELSE
            {
                if (sub(lsf[i], th1) <= 0)
                {
                    curr_min_dist = min_dist_fac3;
                    move16();
                }
            }

            if (sub(lsf[i], lsf_max) > 0)
            {
                lsf[i] = lsf_max;
                move16();
            }
            lsf_max = sub(lsf[i], curr_min_dist);
        }
    }

    return;
}

