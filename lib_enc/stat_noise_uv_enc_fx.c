/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"     /* Function prototypes                    */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "stl.h"

/*======================================================================*/
/* FUNCTION : stat_noise_uv_enc_fx                                      */
/*----------------------------------------------------------------------*/
/* PURPOSE :   Modifies excitation signal in UC mode                    */
/*         when the noise is stationary                                 */
/*----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                                                   */
/* _ (Encoder_State_fx) st_fx :  Encoder state Structure                */
/* _ (Word16) coder_type      :  coding type                            */
/* _ (Word16*) epsP           : LP prediction errors                    */
/* _ (Word16*) isp_new        : immittance spectral pairs at 4th sfr Q15 */
/* _ (Word16*) isp_mid        : immittance spectral pairs at 2nd sfr Q15 */
/* _ (Word16*) Aq             :  A(z) quantized for the 4 subframes  Q12 */
/*-----------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                              */
/* _ (Word16*) exc2             :  excitation buffer        Q_exc        */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/*-----------------------------------------------------------------------*/

/* _ None                                                                */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ None                                                                */
/*=======================================================================*/
void stat_noise_uv_enc_fx(
    Encoder_State_fx *st_fx,       /* i/o: state structure                      */
    const Word16 coder_type,    /* i  : coding type                          */
    const Word32 *LepsP,         /* i  : LP prediction errors                 */
    Word16 *isp_new,      /* i  : immittance spectral pairs at 4th sfr */
    Word16 *isp_mid,      /* i  : immittance spectral pairs at 2nd sfr */
    Word16 *Aq,           /* i  : A(z) quantized for the 4 subframes   */
    Word16 *exc2,         /* i/o: excitation buffer                    */
    Word16 Q_new
)
{
    Word16 noisiness = 0;
    Word16 num,den,expn,expd;

    test();
    test();
    IF ( sub(coder_type,UNVOICED) == 0 || ( sub(coder_type,INACTIVE) == 0 && L_sub(st_fx->core_brate_fx,ACELP_9k60) <= 0 ) )
    {

        /*-----------------------------------------------------------------*
         * Calculate and write the noisiness parameter
         *-----------------------------------------------------------------*/
        /* epsP[2] is located in LepsP[0] and epsP[16] in LepsP[1] */
        expn = sub(norm_l(LepsP[0]),1);
        num = extract_h(L_shl(LepsP[0],expn));/*expn-16*/
        expd = norm_l(LepsP[1]);
        den = extract_h(L_shl(LepsP[1],expd));/*expd-16*/

        num = div_s(num,den);/*expn-expd+15*/
        num = shr(num,add(sub(expn,expd),5));/*Q10*/
        num = sub(num,1024);/*num - 1*/

        test();
        IF ( sub(st_fx->bwidth_fx,NB) != 0 )
        {
            /* WB case */
            /* noisiness = (Word16)(((epsP[2] / epsP[16]) - 1)*2 * 32);*/
            noisiness = shr(num,4);/*Q10  x64 -> Q0 */
        }
        ELSE IF ( sub(coder_type,INACTIVE) == 0 && sub(st_fx->bwidth_fx,NB) == 0)
        {
            /* NB GSC case */
            /* noisiness = (Word16)(((epsP[2] / epsP[16]) - 1)*.25f * 32);*/
            noisiness = shr(num,4+3);/*Q10  x8 -> Q0 */
        }
        ELSE
        {
            /* NB case */
            noisiness = shr(num,4+2);/*Q16  x16 -> Q0 */
        }

        noisiness = s_max(noisiness, 0);
        noisiness = s_min(noisiness, 31);

        push_indice_fx( st_fx, IND_NOISINESS, noisiness, 5 );
    }

    /*-----------------------------------------------------------------*
     * Modify the stationary noise excitation signal
     *-----------------------------------------------------------------*/

    stat_noise_uv_mod_fx( coder_type, noisiness, st_fx->lsp_old_fx, isp_new, isp_mid, Aq
                          ,exc2, Q_new, 0, &st_fx->ge_sm_fx, &st_fx->uv_count_fx, &st_fx->act_count_fx,
                          st_fx->lspold_s_fx, &st_fx->noimix_seed_fx, &st_fx->min_alpha_fx, &st_fx->exc_pe_fx,
                          st_fx->core_brate_fx, st_fx->bwidth_fx, &st_fx->Q_stat_noise, &st_fx->Q_stat_noise_ge );


    return;
}
