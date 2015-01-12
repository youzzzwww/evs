/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


/* Header files */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"

/* Constants */
#define M 16

#define BFI_FAC 0.9f


void lpc_unquantize(
    Decoder_State_fx * st,
    Word16 *lsfold,
    Word16 *lspold,
    Word16 *lsf,
    Word16 *lsp,
    const Word16 m,
    const Word16 lpcQuantization,
    Word16 *param_lpc,
    const Word16 numlpc,
    const Word16 core,
    Word16 *mem_MA,
    Word16 * mem_AR,
    Word16 *lspmid,
    Word16 *lsfmid,
    Word16 coder_type,
    Word16 acelp_midLpc,
    Word8 narrow_band,
    Word16 *seed_acelp,
    Word32 sr_core,
    Word16 *mid_lsf_int,
    Word16 prev_bfi,
    Word16 *safety_net
)
{
    Word16 nb_indices, k;
    Word16 i;

    nb_indices = 0;       /* to avoid compilation warnings */


    Copy(lsfold, &lsf[0], m);
    Copy(lspold, &lsp[0], m);

    IF( lpcQuantization == 0 )
    {
        nb_indices = dlpc_avq(param_lpc, &lsf[m], numlpc, st->sr_core);
        FOR ( k=0; k<numlpc; k++ )
        {
            E_LPC_lsf_lsp_conversion(&lsf[(k+1)*m], &lsp[(k+1)*m], m);
        }
    }
    ELSE IF ( sub(lpcQuantization, 1) == 0 )
    {
        test();
        IF ((L_sub(sr_core, INT_FS_16k) == 0) && (sub(coder_type, UNVOICED) == 0 ))
        {
            lsf_end_dec_fx( st, 1, GENERIC, sub(1,narrow_band) /* st->bwidth */ , 31, &lsf[m], mem_AR, mem_MA, sr_core, st->core_brate_fx,
                            &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->offset_scale1_p_fx[0][0], &st->offset_scale2_p_fx[0][0],
                            &st->no_scales_fx[0][0], &st->no_scales_p_fx[0][0], &st->safety_net_fx, param_lpc,  &nb_indices);
        }
        ELSE
        {
            IF (sub(st->core_fx, TCX_20_CORE)==0)
            {
                lsf_end_dec_fx( st, 1, AUDIO, sub(1, narrow_band) /* st->bwidth */ , 31, &lsf[m], mem_AR, mem_MA, sr_core, st->core_brate_fx,
                &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->offset_scale1_p_fx[0][0], &st->offset_scale2_p_fx[0][0],
                &st->no_scales_fx[0][0], &st->no_scales_p_fx[0][0], &st->safety_net_fx, param_lpc, &nb_indices);
            }
            ELSE
            {
                lsf_end_dec_fx( st, 1, coder_type, sub(1, narrow_band) /* st->bwidth */ , 31, &lsf[m], mem_AR, mem_MA, sr_core, st->core_brate_fx,
                &st->offset_scale1_fx[0][0], &st->offset_scale2_fx[0][0], &st->offset_scale1_p_fx[0][0], &st->offset_scale2_p_fx[0][0],
                &st->no_scales_fx[0][0], &st->no_scales_p_fx[0][0], &st->safety_net_fx, param_lpc, &nb_indices);
            }
        }

        lsf2lsp_fx(&lsf[m], &lsp[m], M, sr_core);
    }
    ELSE
    {
        assert(0);
    }

    *seed_acelp=0;
    move16();
    FOR(i=nb_indices-1; i>=0; i--)
    {
        *seed_acelp = extract_l(L_mac0(L_mac0(13849, shr(*seed_acelp, 1), 31821), param_lpc[i], 31821));
    }

    /* Decoded mid-frame isf */
    test();
    test();
    test();
    IF ( lpcQuantization && acelp_midLpc && core==ACELP_CORE && st->rate_switching_reset==0)
    {
        midlsf_dec ( &lsf[0], &lsf[m], param_lpc[nb_indices], lsfmid
                     ,coder_type
                     ,mid_lsf_int,
                     prev_bfi,
                     *safety_net
                   );
        reorder_lsf_fx( lsfmid, LSF_GAP_MID_FX, M, sr_core );
        lsf2lsp_fx(lsfmid, lspmid, M, sr_core);
    }


    return;
}

