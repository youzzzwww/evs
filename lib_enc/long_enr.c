/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * long_enr_fx()
 *
 * Compute relative energy, long-term average total noise energy and total active speech energy
 *-------------------------------------------------------------------*/
void long_enr_fx(
    Encoder_State_fx *st_fx,	    /* i/o: state structure                       	*/
    const Word16 Etot,				/* i  : total channel E (see lib_enc\analy_sp.c) */
    const Word16 localVAD_HE_SAD,      /* i  : HE-SAD flag without hangover             */
    Word16 high_lpn_flag
)
{
    Word16 tmp;
    Word16 alpha;

    /*-----------------------------------------------------------------*
      * Compute long term estimate of total noise energy
      * and total active speech energy
      *-----------------------------------------------------------------*/

    IF(  sub(st_fx->ini_frame_fx, 4 ) < 0 )

    {

        st_fx->lp_noise_fx = st_fx->totalNoise_fx;
        move16();
        tmp                = add(st_fx->lp_noise_fx, 2560);    /*10.0 in Q8*/
        st_fx->lp_speech_fx = s_max(st_fx->lp_speech_fx, tmp);
    }
    ELSE
    {
        /* if ( st->ini_frame < 150 ) {
             st->lp_noise = 0.95f * st->lp_noise + 0.05f * st->totalNoise;
         }  else {
             st->lp_noise = 0.98f * st->lp_noise + 0.02f * st->totalNoise;
         } */
        alpha   = 655;
        move16();/* 0.02 Q15 */
        if ( sub(st_fx->ini_frame_fx, 150) < 0 ) /* should match HE_LT_CNT_INIT_FX */
        {
            alpha = 1638 ;
            move16(); /* 0.05 Q15 */
        }
        st_fx->lp_noise_fx  =  noise_est_AR1_Qx( st_fx->totalNoise_fx, st_fx->lp_noise_fx , alpha); /* Q8 state, alpha in Q15 */

        test();
        IF ( (localVAD_HE_SAD != 0)
        && ( high_lpn_flag == 0) )
        {
            IF( sub(sub(st_fx->lp_speech_fx, Etot ), 10*256 ) < 0 ) /* 10.0 in Q8 */
            {
                /* st->lp_speech = 0.98f * st->lp_speech + 0.02f * Etot; */
                st_fx->lp_speech_fx = noise_est_AR1_Qx(Etot, st_fx->lp_speech_fx, 655); /* Q8 state, 0.02 in Q15 */
            }
            ELSE
            {
                st_fx->lp_speech_fx = sub(st_fx->lp_speech_fx, 13); /* st->lp_speech = st->lp_speech - 0.05f; linear decay*/
            }
        }

    }


    /*-----------------------------------------------------------------*
     * Initialize parameters for energy tracking and signal dynamics
     *-----------------------------------------------------------------*/
    IF( sub(st_fx->ini_frame_fx, 1) <= 0 )
    {
        st_fx->Etot_h_fx      = Etot;
        move16(); /* Q8 */
        st_fx->Etot_l_fx      = Etot;
        move16();
        st_fx->Etot_l_lp_fx   = Etot;
        move16();
        st_fx->Etot_last_fx   = Etot;
        move16();

        st_fx->Etot_v_h2_fx   = 0;
        move16();
        st_fx->sign_dyn_lp_fx = 0;
        move16();
    }
    return;
}
