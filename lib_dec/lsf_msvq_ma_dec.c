/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "stl.h"
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"


#define swap(x,y,type) {type u__p; u__p=x; x=y; y=u__p;}


Word16 lsf_msvq_ma_decprm( Decoder_State_fx * st, Word16 *param_lpc, Word16 core, Word16 acelp_mode, Word16 acelp_midLpc,
                           Word16 narrowBand, Word32 sr_core
                         )
{
    Word16 i, nbits_lpc, tmp;
    Word16  bits_midlpc;
    Word16 bits0[MAX_VQ_STAGES], bits1[MAX_VQ_STAGES], stages0, stages1, stages,
           levels0[MAX_VQ_STAGES], levels1[MAX_VQ_STAGES], * bits;
    Word16 predmode, mode_lvq, mode_lvq_p,  safety_net;

    bits_midlpc=5;
    move16();

    test();
    IF ((L_sub(sr_core, INT_FS_16k) == 0)&&(sub(acelp_mode, UNVOICED) == 0))
    {
        predmode = find_pred_mode(GENERIC, sub(1, narrowBand) /*st->bwidth*/, sr_core,
                                  &mode_lvq, &mode_lvq_p, st->total_brate_fx);
        move16();
    }
    ELSE
    {
        IF (sub(core, TCX_20_CORE) == 0)
        {
            predmode = find_pred_mode(AUDIO, sub(1,narrowBand)/*st->bwidth*/, sr_core,
            &mode_lvq, &mode_lvq_p, st->total_brate_fx );
            move16();
        }
        ELSE
        {
            predmode = find_pred_mode(acelp_mode, sub(1, narrowBand)/*st->bwidth*/, sr_core,
            &mode_lvq, &mode_lvq_p, st->total_brate_fx );
            move16();
        }
    }
    lsf_allocate_fx( sub(31, shr(predmode,1)), mode_lvq, mode_lvq_p, &stages0, &stages1, levels0, levels1,
                     bits0, bits1);


    nbits_lpc = 0;
    move16();

    IF (sub(predmode, 2) == 0)
    {
        /* there is choice between SN and AR prediction */
        safety_net = get_next_indice_fx(st, 1);

        IF (sub(safety_net,1) == 0)
        {
            stages = stages0;
            move16();
            bits = bits0;
            move16();
        }
        ELSE
        {
            stages = stages1;
            move16();
            bits = bits1;
            move16();
        }
        *param_lpc = safety_net;
        move16();
        param_lpc++;
        nbits_lpc++;

    }
    ELSE
    {
        stages = stages1;
        move16();
        bits = bits1;
        move16();
    }


    tmp = sub(stages,1);
    FOR (i=0; i<tmp; i++)
    {
        *param_lpc = get_next_indice_fx(st, bits[i]);
        param_lpc++;
        nbits_lpc = add(nbits_lpc,bits[i]);

    }
    *param_lpc = get_next_indice_fx(st, LEN_INDICE);
    param_lpc++;
    nbits_lpc = add(nbits_lpc,LEN_INDICE);


    *param_lpc = get_next_indice_fx(st, bits[i]-LEN_INDICE);
    param_lpc++;
    nbits_lpc = add(nbits_lpc, sub(bits[i], LEN_INDICE));


    test();
    test();
    IF ( sub(acelp_mode, VOICED) != 0 && core==0 && acelp_midLpc)
    {

        *param_lpc = get_next_indice_fx(st, bits_midlpc);
        nbits_lpc = add(nbits_lpc, bits_midlpc);
    }

    return nbits_lpc;
}



Word16 lsf_bctcvq_decprm(
    Decoder_State_fx * st,
    Word16 *param_lpc
)
{
    Word16 i, nbits_lpc;
    Word16 num_par;
    const Word16 *bits1;

    num_par = 10;
    bits1 = BC_TCVQ_BIT_ALLOC_40B;

    nbits_lpc = 0;


    FOR (i=0; i<num_par; i++)
    {
        *param_lpc = get_next_indice_fx(st, bits1[i]);
        param_lpc++;
        nbits_lpc = add(nbits_lpc, bits1[i]);
    }

    return nbits_lpc;
}

/* Returns: number of indices */
Word16 D_lsf_tcxlpc(
    const Word16 indices[],       /* (I) VQ indices        */
    Word16 lsf_q[],               /* (O) quantized LSF     */
    Word16 lsp_q_ind[],           /* (O) quantized LSP (w/o MA prediction) */
    Word16 narrowband,            /* (I) narrowband flag   */
    Word16 cdk,                   /* (I) codebook selector */
    Word16 mem_MA[]               /* (I) MA memory         */
)
{
    Word16 i;
    Word16 NumIndices;
    Word16 lsf_q_ind[M16k];
    Word16 pred[M16k];
    const Word16 *means;
    Word16 lsf_rem_q_ind[M];

    NumIndices = 1;
    move16();

    msvq_dec(
        lsf_codebook[narrowband][cdk],
        lsf_dims,
        lsf_offs,
        TCXLPC_NUMSTAGES,
        M,
        M,
        indices + NumIndices,
        lsf_q
    );

    NumIndices = add(NumIndices, TCXLPC_NUMSTAGES);

    FOR (i=0; i<M; ++i)
    {
        lsf_q_ind[i] = lsf_q[i];
        move16();
    }

    IF (indices[0])   /* Only add contribution if flag is enabled */
    {
        msvq_dec(
            lsf_ind_codebook[narrowband][cdk],
            lsf_ind_dims,
            lsf_ind_offs,
            TCXLPC_IND_NUMSTAGES,
            M,
            M,
            indices + NumIndices,
            lsf_rem_q_ind
        );
        NumIndices = add(NumIndices, TCXLPC_IND_NUMSTAGES);

        /* Add to MA-removed vector */
        FOR (i=0; i<M; ++i)
        {
            lsf_q_ind[i] = add(lsf_q_ind[i], lsf_rem_q_ind[i]);
            move16();
        }
    }

    /* Inter-frame prediction */
    move16();
    means = lsf_means[narrowband];

    FOR (i=0; i<M; ++i)
    {
        pred[i] = add(means[i], mult_r(MU_MA_FX, mem_MA[i]));
        move16();
    }

    /* Add prediction */
    FOR (i=0; i<M; ++i)
    {
        lsf_q[i] = add(lsf_q[i], pred[i]);
        move16();
        lsf_q_ind[i] = add(lsf_q_ind[i], means[i]);
        move16();
    }

    reorder_lsf_fx(lsf_q, TCXLPC_LSF_GAP, M, INT_FS_FX);

    reorder_lsf_fx(lsf_q_ind, TCXLPC_LSF_GAP, M, INT_FS_FX);
    IF (lsp_q_ind)
    {
        E_LPC_lsf_lsp_conversion/*lsf2lsp*/(lsf_q_ind, lsp_q_ind, M);
    }

    return NumIndices;
}

/* Returns: number of bits read */
Word16 dec_lsf_tcxlpc(
    Decoder_State_fx *st,         /* (I/O) Decoder state    */
    Word16 **indices,             /* (O) Ptr to VQ indices  */
    Word16 narrowband,            /* (I) narrowband flag    */
    Word16 cdk                    /* (I) codebook selector  */
)
{
    Word16 i, start_bit_pos;
    Word16 lsf_q_ind[M];
    Word16 *flag;

    flag = *indices; /* Save pointer */
    *flag = 0;
    move16(); /* Set flag to disabled */
    ++*indices;

    start_bit_pos = st->next_bit_pos_fx;
    move16();
    FOR (i=0; i<TCXLPC_NUMSTAGES; ++i)
    {
        **indices = get_next_indice_fx(st, lsf_numbits[i]);
        ++*indices;
        move16();
    }

    /* Decode independent lsf */
    msvq_dec(
        lsf_codebook[narrowband][cdk],
        lsf_dims,
        lsf_offs,
        TCXLPC_NUMSTAGES,
        M,
        M,
        flag+1,
        lsf_q_ind
    );

    /* Update flag */
    *flag = lsf_ind_is_active(lsf_q_ind, lsf_means[narrowband], narrowband, cdk);
    move16();
    IF (*flag)
    {
        FOR (i=0; i<TCXLPC_IND_NUMSTAGES; ++i)
        {
            **indices = get_next_indice_fx(st, lsf_ind_numbits[i]);
            ++*indices;
            move16();
        }
    }
    return sub(st->next_bit_pos_fx, start_bit_pos);
}
