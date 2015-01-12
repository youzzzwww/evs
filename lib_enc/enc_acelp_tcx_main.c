/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"


void enc_acelp_tcx_main(
    const Word16 new_samples[],          /* i  : new samples                         */
    Encoder_State_fx *st,                    /* i/o: encoder state structure             */
    const Word16 coder_type,             /* i  : coding type                         */
    const Word16 pitch[3],                /* i  : open-loop pitch values for quantiz. */
    const Word16 voicing[3],             /* i  : open-loop pitch gains               */
    Word16 Aw[NB_SUBFR16k*(M+1)],  /* i  : weighted A(z) unquant. for subframes*/
    const Word16 lsp_new[M],             /* i  : LSPs at the end of the frame        */
    const Word16 lsp_mid[M],             /* i  : LSPs at the middle of the frame     */
    HANDLE_FD_CNG_ENC hFdCngEnc,      /* i/o: FD CNG handle                      */
    Word32 bwe_exc_extended[],     /* i/o: bandwidth extended excitation       */
    Word16 *voice_factors,         /* o  : voicing factors                     */
    Word16 pitch_buf[],            /* o  : floating pitch for each subframe    */
    Word16 vad_hover_flag,
    Word16 *Q_new,
    Word16 *shift
)
{
    Word16 old_bwe_exc[(PIT16k_MAX + (L_FRAME16k + 1) + L_SUBFR16k) * 2]; /* excitation buffer */
    Word16 *ptr_bwe_exc;
    ptr_bwe_exc = old_bwe_exc + PIT16k_MAX * 2;

    IF( sub( st->last_core_fx, ACELP_CORE) == 0 )
    {
        set16_fx( old_bwe_exc + PIT16k_MAX * 2, 0, ((L_FRAME16k + 1) + L_SUBFR16k) * 2 );
        Copy( st->old_bwe_exc_fx, old_bwe_exc, PIT16k_MAX * 2 );
    }
    ELSE
    {
        set16_fx( old_bwe_exc, 0, ((L_FRAME16k + 1) + L_SUBFR16k + PIT16k_MAX) * 2 );
    }

    /* PLC: [Guided ACELP PLC] */
    gPLC_encInfo(&st->plcExt,
                 st->total_brate_fx,
                 st->bwidth_fx,
                 st->clas_fx,
                 coder_type
                );

    IF ( s_and(st->core_brate_fx!=FRAME_NO_DATA, st->core_brate_fx!=SID_2k40) )
    {

        /* Run Core Coder */
        IF (st->tcxonly == 0)
        {
            core_encode_openloop( st, coder_type,
                                  pitch,
                                  voicing, Aw, lsp_new, lsp_mid,
                                  pitch_buf, voice_factors, ptr_bwe_exc
                                  , vad_hover_flag, *Q_new, *shift );
        }
        ELSE
        {
            core_encode_twodiv( new_samples,
            st, coder_type, pitch, voicing, Aw, Q_new, shift );
        }
        /*-----------------------------------------------------------------*
         * Apply non linearity to the SHB excitation
         *-----------------------------------------------------------------*/


        test();
        IF( sub( st->core_fx, ACELP_CORE ) == 0 && st->igf != 0 )
        {
            non_linearity_fx( ptr_bwe_exc, bwe_exc_extended, L_FRAME32k, &st->bwe_non_lin_prev_scale_fx, *Q_new
                              , coder_type, voice_factors, st->L_frame_fx
                            );

            /* update the old BWE exe memory */
            Copy( &old_bwe_exc[L_FRAME32k], st->old_bwe_exc_fx, PIT16k_MAX * 2 );
        }
        ELSE
        {
            set16_fx( st->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );
            st->bwe_non_lin_prev_scale_fx = 0;
        }


    }
    ELSE
    {
        /* Run SID Coder */
        IF ( st->core_brate_fx == SID_2k40 )
        {
            FdCng_encodeSID( hFdCngEnc, st, st->preemph_fac );
        }

        /* Generate Comfort Noise */
        generate_comfort_noise_enc( st, *Q_new, 1 );

        /* Update Core Encoder */
        core_encode_update_cng( st, hFdCngEnc->hFdCngCom->timeDomainBuffer, hFdCngEnc->hFdCngCom->A_cng, Aw, *Q_new, *shift );
    }

    /* coreSwitching update of MODE1 parameters in the last frame */
    st->last_coder_type_fx = coder_type;


    return;
}
