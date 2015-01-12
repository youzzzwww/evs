/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"          /* Common prototypes           */
#include "prot_fx.h"          /* Common prototypes           */
#include "cnst_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Rescale_exc:
 *
 * Find absolute maximum of excitation
 * Fin scaling factor to apply the excitation and its related memory
 * Scale excitation and total excitation (exc2)
 *-------------------------------------------------------------------*/
Word16 Rescale_exc(
    Word16 dct_post_old_exc_fx[], /* i/o: Music post processing memory */
    Word16 exc[],        /* i/o: excitation to rescale           Q_exc */
    Word16 bwe_exc[],
    Word16 *last_exc_dct_in,
    Word16 lg,           /* i  : frame size                            */
    Word16 lg32,
    Word32 L_gain_code,  /* i  : decoded codebook gain           Q16   */
    Word16 *sQ_exc,      /* i/o: Excitation scaling factor             */
    Word16 *sQsubfr,     /* i/o: Past excitation scaling factors       */
    Word16 exc2[],       /* o  : local excitation vector               */
    Word16 i_subfr,       /* i  : subframe number                       */
    const Word16 coder_type
)
{
    Word16 i, tmp, max, new_Q;

    /*-------------------------------------------
     * find maximum of absolute excitation
     *-------------------------------------------*/
    max = s_max(abs_s(exc[0]), 1);
    FOR (i = 1; i < lg; i++)
    {
        tmp = abs_s(exc[i]);
        max = s_max(max, tmp);
    }

    /*----------------------------------------------
     * find scaling (tmp) to set max = [2048..4096[
     *----------------------------------------------*/
    tmp = sub(add(norm_s(max), *sQ_exc), 3);
    tmp = s_min(tmp, 12);

    /*----------------------------------------------
     * find scaling (new_Q) to keep gain_code < 2048
     *----------------------------------------------*/

    new_Q = add(tmp, 1);
    tmp = sub(norm_l(L_or(L_gain_code, 1)), 3); /* to get to 0x08000000L (L_or with 1 to avoid norm_l(0)) */
    tmp = s_min(tmp, new_Q);
    tmp = s_max(tmp, 0);
    tmp = sub(tmp, 1);

    /*#define REMOVE_EXCITATION_PER_FRAME_SCALING */

    /*----------------------------------------------
     * new_Q = smallest Q since 4 subframes (20ms)
     *----------------------------------------------*/
    IF( sub( coder_type, TRANSITION ) == 0  )
    {
        tmp = s_min(tmp, 7);
    }
    ELSE IF (sub(coder_type,INACTIVE)==0)
    {
        tmp = s_min(tmp, 13);
    }
    ELSE if( sub(lg,L_SUBFR) > 0 )/* --> can only happen in AUDIO mode */
    {
        tmp = s_min(tmp, 4);      /* Limitation of the scaling gain because the frequency domain will add much more energy to the excitation*/
    }

    new_Q  = s_min(tmp, sQsubfr[0]);
    IF(sub(lg, L_SUBFR)==0)
    {
        FOR(i = L_Q_MEM-1; i >= 1; i--)
        {
            new_Q  = s_min(new_Q, sQsubfr[i]);
            sQsubfr[i] = sQsubfr[i-1];
            move16();
        }
    }
    ELSE
    {
        IF(sub(lg, 2*L_SUBFR)==0)
        {
            new_Q  = s_min(new_Q, sQsubfr[L_Q_MEM-1]);
            FOR(i = L_Q_MEM-1; i >= 2; i--)
            {
                sQsubfr[i] = sQsubfr[1];
                move16();
            }
            sQsubfr[1] = tmp;
            move16();
            sQsubfr[0] = tmp;
            move16();
        }
        ELSE
        {
            set16_fx(sQsubfr, tmp, L_Q_MEM);
        }
    }
    sQsubfr[0] = tmp;
    move16();

    /*----------------------------------------------
     * rescale excitation and associated memories
     *----------------------------------------------*/

    tmp = sub(new_Q, *sQ_exc);

    IF (tmp != 0)
    {

        Scale_sig(exc-L_EXC_MEM_DEC, add(L_EXC_MEM_DEC, lg), tmp);
        IF(last_exc_dct_in != NULL)
        {
            Scale_sig(last_exc_dct_in, L_FRAME, tmp);
        }
        IF(bwe_exc != NULL)
        {
            Scale_sig(bwe_exc-PIT16k_MAX*2, add(PIT16k_MAX*2, lg32), tmp);
        }
        IF(exc2 != NULL)
        {
            Scale_sig(exc2, i_subfr, tmp);
        }
        IF(dct_post_old_exc_fx != NULL)
        {
            Scale_sig(dct_post_old_exc_fx, DCT_L_POST-OFFSET2, tmp);
        }
    }

    /* scaling factor of excitation (-1..12) */
    *sQ_exc = new_Q;
    move16();

    return tmp;
}

/*-------------------------------------------------------------------*
 * Rescale_mem:
 *
 * this function should be called after excitation update (4 subfr) and before frame synthesis
 * Rescale excitation related memories
 *-------------------------------------------------------------------*/
void Rescale_mem(
    const Word16 Q_exc,           /* i   : current excitation scaling (>=0)          */
    Word16 *prev_Q_syn,     /* i/o  : scaling factor of previous frame          */
    Word16 *Q_syn,          /* i/o  : scaling factor of frame                   */
    Word16 *mem_syn2        /* i/o  : modified synthesis memory                 */
    ,Word16 *mem_syn_clas_estim_fx /* i/o  : old 12k8 core memory for classification */
    ,const Word16 MaxScaling       /* i: Minimal difference between excitation scaling and synthesis scaling */
    ,Word16 *mem_deemph,         /* i/o: speech deemph filter memory                 */
    Word16 *pst_old_syn,        /* i/o:  psfiler                                     */
    Word16 *pst_mem_deemp_err  /* i/o:  psfiler                                     */
    ,Word16 *mem_agc
    ,PFSTAT *pf_stat            /* i/o:  All memories related to NB post filter      */
    ,const Word16 Vad_flag
    ,const Word16 *tmp_buffer       /* tmp_buffer in Q-1 */
)
{
    Word16 exp_scale, new_Q, tmp, i;

    /*-------------------------------------------------------------------*
     * find scaling of synthesis (based on min of current frame and last frame)
     * scaling factor of synthesis (-1..6)
     *-------------------------------------------------------------------*/
    new_Q = sub(Q_exc, MaxScaling);
    tmp = 1;
    move16();
    IF(tmp_buffer != NULL)
    {
        /* use the temporary synthesis in Q-1 to estimate the scaling */
        FOR (i = 0; i < L_FRAME; i++)
        {
            tmp = s_max(abs_s(tmp_buffer[i]), tmp);
        }
        /* we add Q_syn which represents the actual scaling of the memories prev_Q_syn represents the last potential scaling */
        tmp = sub(add(norm_s(tmp), -1), 3);   /* -2 ... 12 */
    }
    ELSE
    {
        FOR (i = 0; i < M; i++)
        {
            tmp = s_max(abs_s(mem_syn2[i]), tmp);
            tmp = s_max(abs_s(pst_old_syn[i]), tmp);
            tmp = s_max(abs_s(mem_syn_clas_estim_fx[i]), tmp);
        }
        FOR (; i < L_SUBFR; i++)
        {
            tmp = s_max(abs_s(pst_old_syn[i]), tmp);
            tmp = s_max(abs_s(mem_syn_clas_estim_fx[i]), tmp);
        }
        FOR (; i < L_SYN_MEM_CLAS_ESTIM; i++)
        {
            tmp = s_max(abs_s(mem_syn_clas_estim_fx[i]), tmp);
            tmp = s_max(abs_s(pst_old_syn[i]), tmp);
        }
        FOR (; i < NBPSF_PIT_MAX; i++)
        {
            tmp = s_max(abs_s(pst_old_syn[i]), tmp);
        }
        /* we add Q_syn which represents the actual scaling of the memories prev_Q_syn represents the last potential scaling */
        tmp = sub(add(norm_s(tmp), *Q_syn), 2);   /* -2 ... 12 */
    }


    IF(Vad_flag != 0)
    {
        new_Q = s_min(sub(Q_exc,2), tmp);
    }
    ELSE
    {
        new_Q = s_min(Q_exc, tmp);
    }
    new_Q = s_min(new_Q, 12);               /*  */
    new_Q = s_max(new_Q, -1);               /*  */

    /*#define REMOVE_SYNTHESIS_PER_FRAME_SCALING*/
    tmp = s_min(new_Q, *prev_Q_syn);
    *prev_Q_syn = new_Q;
    move16();

    exp_scale = sub(tmp, *Q_syn);
    *Q_syn = tmp;
    move16();

    /* rescale synthesis memory (mem_syn2) */
    Scale_sig(mem_syn2, M, exp_scale);
    Scale_sig(mem_syn_clas_estim_fx, L_SYN_MEM_CLAS_ESTIM, exp_scale);
    /*Scale_sig(core_old_syn, L_SYN_MEM, exp_scale);*/
    Scale_sig(mem_deemph, 1, exp_scale);
    Scale_sig(pst_old_syn, NBPSF_PIT_MAX, exp_scale);
    Scale_sig(pst_mem_deemp_err, 1, exp_scale);
    Scale_sig(pf_stat->mem_pf_in, L_SUBFR, exp_scale);           /* NB post_filter mem */
    Scale_sig(pf_stat->mem_res2, DECMEM_RES2, exp_scale);  /* NB post_filter mem */
    Scale_sig(pf_stat->mem_stp, L_SUBFR, exp_scale);             /* NB post_filter mem */
    Scale_sig(mem_agc, 1, exp_scale);             /* NB post_filter mem */
    return;
}

/*-------------------------------------------------------------------*
 * Scale_sig32
 * Note: In order to save complexity, call function only, if exp0 != 0
 * Up/down scale a 32 bits vector
 *-------------------------------------------------------------------*/
void scale_sig32(
    Word32 x[],  /* i/o: signal to scale                 Qx        */
    const Word16 lg,   /* i  : size of x[]                     Q0        */
    const Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx   exp  */
)
{
    Word16 i;

    FOR (i = 0; i < lg; i++)
    {
        /* saturation can occur here */
        x[i] = L_shl(x[i], exp0);
        move32();
    }
}


/*-------------------------------------------------------------------*
 * Rescale_mem:
 *
 * this function should be called after excitation update (4 subfr) and before frame synthesis
 * Rescale excitation related memories
 *-------------------------------------------------------------------*/
Word16 rescale_mem(
    const Word16 *Q_exc,          /* i    : current excitation scaling (>=0)          */
    Word16 *prev_Q_syn,     /* i/o  : scaling factor of previous frame          */
    Word16 *Q_syn,          /* i/o  : scaling factor of frame                   */
    Word16 *mem_syn2,       /* i/o  : modified synthesis memory                 */
    Word16 *syn,            /* i/o  : synthesis  to rescale           Q_syn     */
    Word16  mem_len,        /* i    : lenght of modified synthesis memory       */
    Word16  i_subfr         /* i  : subframe number                       */
)
{
    Word16 exp_scale, new_Q, tmp;

    /*-------------------------------------------------------------------*
     * find scaling of synthesis (based on min of current frame and last frame)
     * scaling factor of synthesis (-1..6)
     *-------------------------------------------------------------------*/
    new_Q = sub(*Q_exc, 6);
    new_Q = s_max(new_Q, -1);

    tmp = s_min(new_Q, *prev_Q_syn);
    *prev_Q_syn = new_Q;
    move16();

    exp_scale = sub(tmp, *Q_syn);
    *Q_syn = tmp;
    move16();
    /* rescale synthesis memory (mem_syn2) */
    Scale_sig(mem_syn2, mem_len, exp_scale);
    IF(syn != NULL)
    {
        Scale_sig(syn, i_subfr, exp_scale);
    }

    return exp_scale;
}
