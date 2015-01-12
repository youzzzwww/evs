/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"       /* Function prototypes                    */


/*---------------------------------------------------------*
 * stat_noise_uv_dec()
 *
 * Modifies excitation signal in UC mode when the noise is stationary
 *---------------------------------------------------------*/
void stat_noise_uv_dec_fx(
    Decoder_State_fx *st_fx,    /* i/o: Decoder static memory                */
    const Word16 coder_type,    /* i  : coding type                           */
    Word16 *lsp_new,      /* i  : end-frame LSP vector */
    Word16 *lsp_mid,      /* i  : mid-frame LSP vector */
    Word16 *Aq,           /* o  : A(z) quantized for the 4 subframes   */
    Word16 *exc2          /* i/o: excitation buffer                    */
)
{
    Word16 noisiness = 0, i;
    Word32 L_tmp;

    /*-----------------------------------------------------------------*
     * Decode the VAD flag
     *-----------------------------------------------------------------*/
    test();
    test();
    IF( sub(coder_type,UNVOICED ) == 0|| ( sub(coder_type,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_9k60) <= 0 ) )
    {
        /* read the noisiness parameter */
        noisiness = (Word16)get_next_indice_fx( st_fx, 5 );
        move16();
    }

    /*-----------------------------------------------------------------*
     * Update long-term energies for FEC
     * Update LSP vector for CNG
     *-----------------------------------------------------------------*/
    IF (sub(coder_type,INACTIVE) == 00)
    {
        IF (sub(st_fx->unv_cnt_fx,20) > 0)
        {
            /*ftmp = st->lp_gainc * st->lp_gainc;*/
            L_tmp = L_mult0(st_fx->lp_gainc_fx, st_fx->lp_gainc_fx);              /*Q3 * Q3 ->Q6*/
            /*st->lp_ener = 0.7f * st->lp_ener + 0.3f * ftmp;*/
            L_tmp = Mult_32_16(L_tmp, 9830);
            st_fx->lp_ener_fx = L_add(Mult_32_16(st_fx->lp_ener_fx, 22938), L_tmp);
            move16();  /*Q6 + Q6*/

            FOR( i=0 ; i<M ; i++ )/* AR Low pass filter */
            {
                /*st->lspCNG[i] = (float)(0.9f * st->lspCNG[i] + 0.1f * lspnew[i]);*/
                L_tmp = L_mult(lsp_new[i], 3277);
                st_fx->lspCNG_fx[i] = mac_r(L_tmp, st_fx->lspCNG_fx[i], 29491);
                move16();/*Q15*/
            }
        }
        ELSE
        {
            st_fx->unv_cnt_fx = add(st_fx->unv_cnt_fx,1);
        }
    }
    ELSE
    {
        st_fx->unv_cnt_fx = 0;
        move16();
    }

    IF (!st_fx->Opt_AMR_WB_fx)
    {
        stat_noise_uv_mod_fx( coder_type, noisiness, st_fx->lsp_old_fx, lsp_new, lsp_mid, Aq
                              ,exc2, st_fx->Q_exc, 0, &st_fx->ge_sm_fx, &st_fx->uv_count_fx, &st_fx->act_count_fx,
                              st_fx->lspold_s_fx, &st_fx->noimix_seed_fx, &st_fx->min_alpha_fx,
                              &st_fx->exc_pe_fx, st_fx->core_brate_fx, st_fx->bwidth_fx,
                              &st_fx->Q_stat_noise, &st_fx->Q_stat_noise_ge );
    }


    return ;
}
