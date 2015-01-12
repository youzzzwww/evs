/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * vad_param_updt()
 *
 * Update parameters used by VAD
 *--------------------------------------------------------------------*/

void vad_param_updt_fx(
    Encoder_State_fx *st_fx,	    /* i/o: state structure                       	*/
    Word16 pitch[3],        /* i  : open loop pitch lag for each half-frame                    Q0*/
    Word16 voicing[3],      /* i  : maximum normalized correlation for each half-frame         Q15*/
    Word16 corr_shift,      /* i  : correlation shift                                          Q15*/
    Word16 vad_flag,        /* i  : vad flag                                                   Q0*/
    const Word16 Az[]       /* i:  a coeffs                                                  Q12 */

)
{
    Word16   voice_tmp, pitch_tmp;
    Word32 L_tmp;


    Word16 refl[M+1];
    Word16 tmp1,tmp2;
    Word16 tmp_active_flag;

    IF( !st_fx->Opt_AMR_WB_fx )
    {
        /* fix explanation
           , after function dtx_fx,  the "vad_flag"  parameter can not be used for SID scheduling purposes any longer
           as  dtx_fx  can  schedules active  frames even if the initial analyzed  vad_flag is  0  )
           in the worst case without the fix an active frame could be classified as SID frame,   quite/very  unlikley though
         */
        tmp1            = vad_flag;
        move16(); /* kill MSVC warning */
        tmp_active_flag = 0;
        move16();
        test();
        if( (L_sub(st_fx->core_brate_fx, (Word32)SID_2k40) != 0) && (st_fx->core_brate_fx != 0)  ) /* Note,  core_brate_fx can be -1 */
        {
            tmp_active_flag = 1;
            move16(); /* reqires active coding according  to dtx_fx logic  */
        }
        test();
        test();
        IF( (st_fx->Opt_DTX_ON_fx != 0)  && (tmp_active_flag == 0) && ( sub(st_fx->ini_frame_fx,3) > 0) )


        {
            /* update the counter of consecutive inactive frames in DTX */
            st_fx->consec_inactive_fx = add(st_fx->consec_inactive_fx,1);
            IF( sub(st_fx->consec_inactive_fx,5) > 0 )
            {
                st_fx->consec_inactive_fx = 5;
                move16();
            }

            IF( sub(st_fx->consec_inactive_fx,5) == 0 )
            {
                /* compute spectral tilt parameter */
                a2rc_fx( &Az[1], refl, M ); /* cast to kill MSVC warning */
                /* i: Az   in Q12 */
                /* o: refl in Q15 */

                IF( sub(st_fx->spectral_tilt_reset_fx,1) == 0 )
                {
                    st_fx->spectral_tilt_reset_fx = 0;
                    move16();
                    st_fx->running_avg_fx = refl[0];
                    move16(); /*Q15*/
                    st_fx->ra_deltasum_fx = 0;
                    move16(); /*Q15*/
                }

                /* st_fx->ra_deltasum_fx += (0.80f * st_fx->running_avg_fx + 0.20f * refl[0]) - st_fx->running_avg_fx;
                   st_fx->running_avg_fx = 0.80f * st_fx->running_avg_fx + 0.20f * refl[0]; */
                tmp1 = mult(6553, st_fx->running_avg_fx); /* = -0.80f * st_fx->running_avg_fx + st_fx->running_avg_fx*/
                tmp2 = mult(6553, refl[0]);
                st_fx->ra_deltasum_fx = add(st_fx->ra_deltasum_fx, sub(tmp2,tmp1));
                move16();

                tmp1 = mult(26214, st_fx->running_avg_fx);
                st_fx->running_avg_fx = add(tmp1,tmp2);
                move16();

                IF( sub(abs_s(st_fx->ra_deltasum_fx), 6553) > 0 ) /*0.2 in Q15*/
                {
                    st_fx->spectral_tilt_reset_fx = 1;
                    move16();
                    st_fx->running_avg_fx = 0;
                    move16();
                    st_fx->ra_deltasum_fx = 0;
                    move16();
                    st_fx->trigger_SID_fx = 1;
                    move16();
                }
            }
        }
        ELSE
        {
            st_fx->trigger_SID_fx = 0;
            move16();
            st_fx->consec_inactive_fx = 0;
            move16();
        }

        IF( st_fx->trigger_SID_fx == 1 )
        {
            IF( st_fx->cng_cnt_fx >= 8 )
            {
                /* Declare SID frame due to spectral tilt changes */
                st_fx->cnt_SID_fx = 1;
                move16();
                st_fx->core_brate_fx = SID_2k40;
                move16();
                st_fx->trigger_SID_fx = 0;
                move16();
            }
            ELSE IF ( st_fx->core_brate_fx == SID_2k40 )
            {
                /* SID fame has already been declared before */
                st_fx->trigger_SID_fx = 0;
                move16();
            }
        }
    }



    /* (voicing[0] + voicing[1] + voicing[2]) / 3  + corr_shift */
    L_tmp = L_mult(voicing[0], 10923);
    L_tmp = L_mac(L_tmp, voicing[1], 10923);
    L_tmp = L_mac(L_tmp, voicing[2], 10923); /*Q15 */
    L_tmp = L_mac(L_tmp, corr_shift, 32767); /*Q15 */
    voice_tmp = round_fx(L_tmp); /*Q15 */

    /* abs(pitch[0] - *pitO) + abs(pitch[1] - pitch[0]) + abs(pitch[2] - pitch[1]) */
    pitch_tmp = abs_s(sub(pitch[0],st_fx->pitO_fx)); /*Q0 */
    pitch_tmp = add(pitch_tmp,abs_s(sub(pitch[1],pitch[0]))); /*Q0 */
    pitch_tmp = add(pitch_tmp,abs_s(sub(pitch[2],pitch[1]))); /*Q0 */


    /*    if( (voicing[0] + voicing[1] + voicing[2]) / 3 + corr_shift > 0.65 &&
            (short)(abs(pitch[0] - st->pitO) + abs(pitch[1] - pitch[0]) + abs(pitch[2] - pitch[1])) / 3 < 14 )
        {
            (st->voiced_burst)++;
        } else {
            st->voiced_burst = 0;
        } */

    st_fx->voiced_burst_fx = add( st_fx->voiced_burst_fx,1);
    move16();
    test();
    if ( ( sub(voice_tmp,21299) <= 0 )    /* 0.65 in Q15 */
            ||  ( sub(pitch_tmp,42) >= 0 ) )  /*3*14 = 42   Q0 */
    {
        st_fx->voiced_burst_fx = 0;
        move16();
    }

    /* Update previous voicing value for next frame use */
    /*  st->voicing_old = (voicing[0] + voicing[1] + voicing[2]) / 3 + corr_shift; */
    st_fx->voicing_old_fx = voice_tmp;
    move16();



    return;
}
