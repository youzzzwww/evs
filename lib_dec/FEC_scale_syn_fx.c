/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/
#define AGC_FX 32113  /* 0.98f */
#define SCLSYN_LAMBDA (FL2WORD16(0.3f))

/*========================================================================*/
/* FUNCTION : FEC_scale_syn_fx()										  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Smooth the speech energy evolution when					  */
/*		     recovering after a BAD frame								  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) L_frame       :  length of the frame						  */
/* _ (Word16) *update_flg   :  indication about resynthesis				  */
/* _ (Word16) st_fx->clas_dec: frame classification						  */
/* _ (Word16) last_good	    : last good frame classification     		  */
/* _ (Word16[]) synth       : synthesized speech at Fs = 12k8 Hz   Q_syn  */
/* _ (Word16[])  pitch	  : pitch values for each subframe   	    Q0	  */
/* _ (Word32)  L_enr_old  :energy at the end of previous frame    	Q0	  */
/* _ (Word16) L_enr_q	  : transmitted energy for current frame    Q0    */
/* _ (Word16) coder_type	  : coder type                                */
/* _ (Word16) st_fx->prev_bfi_fx: previous frame BFI                      */
/* _ (Word16) st_fx->last_core_brate_fx: previous frame core bitrate      */
/* _ (Word16[]) mem_tmp	  : temp. initial synthesis filter states   Q_syn */
/* _ (Word16) Q_exc	  : quantized LSPs from frame end    	              */
/* _ (Word16) Q_syn	  : quantized LSPs from frame end    	              */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _ (Word16[]) exc	  : excitation signal without enhancement      Q_exc  */
/* _ (Word16[]) exc2	  : excitation signal with enhancement     Q_exc  */
/* _ (Word16[]) Aq	  :  LP filter coefs (can be modified if BR)   Q12    */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/*------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->mem_syn2	  : initial synthesis filter states Q_syn */
/* _ (Word16) st_fx->old_enr_LP   : LP filter E of last            Q5     */
/*									good voiced frame                     */
/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/


void FEC_scale_syn_fx(
    const Word16 L_frame,          /* i  : length of the frame                     */
    Word16 *update_flg,      /* o: flag indicating re-synthesis after scaling*/
    Word16 clas,             /* i/o: frame classification                    */
    const Word16 last_good,        /* i:   last good frame classification          */
    Word16 *synth,           /* i/o: synthesized speech at Fs = 12k8 Hz      */
    const Word16 *pitch,           /* i:   pitch values for each subframe          */
    Word32 L_enr_old,          /* i:   energy at the end of previous frame     */
    Word32 L_enr_q,            /* i:   transmitted energy for current frame    */
    const Word16 coder_type,       /* i:   coder type                              */
    const Word16 prev_bfi,         /* i:   previous frame BFI                      */
    const Word32 last_core_brate,  /* i:   previous frame core bitrate             */
    Word16 *exc,             /* i/o: excitation signal without enhancement   */
    Word16 *exc2,            /* i/o: excitation signal with enhancement      */
    Word16 Aq[],             /* i/o: LP filter coefs (can be modified if BR) */
    Word16 *old_enr_LP,      /* i/o: LP filter E of last good voiced frame   */
    const Word16 *mem_tmp,         /* i:   temp. initial synthesis filter states   */
    Word16 *mem_syn,          /* o:   initial synthesis filter states         */
    Word16 Q_exc,
    Word16 Q_syn
)
{
    Word16 i;
    Word32 L_enr1, L_enr2;
    Word16 gain1, gain2, enr_LP;
    Word16 tmp, tmp2, exp, exp2;
    Word16 tmp3;
    Word32 L_tmp;

    enr_LP = 0;
    move16();
    gain2 = 0;
    move16();
    gain1 = 0;
    move16();
    *update_flg = 0;
    move16();
    L_enr_old = L_max(1, L_enr_old); /* to avoid division by zero (*L_enr_old is always >= 0) */
    L_enr_q = L_max(1, L_enr_q);     /* to avoid division by zero (  L_enr_q  is always >= 0) */


    /*-----------------------------------------------------------------*
     * Find the synthesis filter impulse response on voiced
     *-----------------------------------------------------------------*/
    test();
    IF( sub(clas,VOICED_TRANSITION) >= 0 && sub(clas,INACTIVE_CLAS) < 0 )
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            enr_LP = Enr_1_Az_fx(Aq+(NB_SUBFR-1)*(M+1), L_SUBFR );
        }
        ELSE  /* L_frame == L_FRAME16k */
        {
            enr_LP = Enr_1_Az_fx( Aq+(NB_SUBFR16k-1)*(M+1), L_SUBFR ); /*Q5*/
        }
    }

    /* previous frame erased and no TC frame */
    test();
    IF( prev_bfi && sub(coder_type,TRANSITION) != 0 )
    {
        /*-----------------------------------------------------------------*
         * Find the energy at the end of the frame
         *-----------------------------------------------------------------*/
        tmp = frame_ener_fx(L_frame,clas, synth, pitch[shr(L_frame,6)-1], &L_enr2/*Q0*/, 1, Q_syn, 3, 0);
        IF(L_enr_q == 1)
        {
            L_enr_q = L_add(0,L_enr2); /* sets to 'L_enr2' in 1 clock */
            test();
            test();
            test();
            IF( sub(last_good,VOICED_TRANSITION) >= 0 && sub(last_good,INACTIVE_CLAS) < 0 && sub(clas,VOICED_TRANSITION) >= 0 && sub(clas,INACTIVE_CLAS) < 0 )
            {
                /* Voiced-voiced recovery */

                test();
                IF ((*old_enr_LP != 0.0f) && (sub(enr_LP, shl(*old_enr_LP, 1)) > 0))
                {

                    /* enr_q /= enr_LP */
                    exp = norm_l(L_enr_q);
                    tmp = extract_h(L_shl(L_enr_q, exp));

                    exp2 = norm_s(enr_LP);
                    tmp2 = shl(enr_LP, exp2);

                    exp = sub(exp2, exp);

                    tmp3 = sub(tmp, tmp2);
                    IF (tmp3 > 0)
                    {
                        tmp = shr(tmp, 1);
                        exp = add(exp, 1);
                    }
                    tmp = div_s(tmp, tmp2);

                    /* L_enr_q *= 2 * *old_enr_LP */
                    L_enr_q = L_shl(L_mult(tmp, shl(*old_enr_LP, 1)), exp);
                }

                IF( L_sub(L_enr_q, L_enr_old) > 0) /* Prevent energy to increase on voiced */
                {
                    L_enr_q = L_add(Mpy_32_16_1(L_enr_old, 32767 - SCLSYN_LAMBDA), Mpy_32_16_1(L_enr_q, SCLSYN_LAMBDA));
                }
            }
        }

        L_enr_q = L_max(1, L_enr_q);

        /* gain2 = (float)sqrt( enr_q / enr2 );*/
        exp = norm_l(L_enr_q);
        tmp = extract_h(L_shl(L_enr_q, exp));

        exp2 = norm_l(L_enr2);
        tmp2 = extract_h(L_shl(L_enr2, exp2));

        exp2 = sub(exp, exp2); /* Denormalize and substract */

        tmp3 = sub(tmp2, tmp);
        IF (tmp3 > 0)
        {
            tmp2 = shr(tmp2, 1);
            exp2 = add(exp2, 1);
        }

        tmp = div_s(tmp2, tmp);

        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp2);
        gain2 = round_fx(L_shl(L_tmp, sub(exp2, 1))); /* in Q14 */

        /*-----------------------------------------------------------------*
         * Clipping of the smoothing gain at the frame end
         *-----------------------------------------------------------------*/

        gain2 = s_min(gain2, 19661);     /* Gain modification clipping */
        if (L_sub(L_enr_q, 1) < 0)
        {
            gain2 = s_min(gain2, 16384); /* Gain modification clipping */
        }

        /*-----------------------------------------------------------------*
          * Find the energy/gain at the beginning of the frame to ensure smooth transition after erasure(s)
          *-----------------------------------------------------------------*/
        test();
        test();
        test();
        test();
        test();
        test();
        IF( sub(clas,SIN_ONSET) == 0 )   /* slow increase */
        {
            gain1 = shr(gain2, 1);
        }
        /*------------------------------------------------------------*
         * voiced->unvoiced transition recovery
         *------------------------------------------------------------*/
        ELSE IF( (sub(last_good,VOICED_TRANSITION) >= 0 && sub(last_good,INACTIVE_CLAS) < 0 && (sub(clas,UNVOICED_CLAS) == 0 || sub(clas,INACTIVE_CLAS) == 0)) ||          /* voiced->unvoiced transition recovery */
                 L_sub(last_core_brate,SID_1k75) == 0 || L_sub(last_core_brate,SID_2k40) == 0 || L_sub(last_core_brate,FRAME_NO_DATA) == 0)                                      /* CNG -> active signal transition */
        {
            gain1 = gain2;
            move16();
        }
        ELSE
        {
            /*--------------------------------------------------------*
             * Find the energy at the beginning of the frame
             *--------------------------------------------------------*/
            tmp = frame_ener_fx(L_frame,clas, synth, pitch[0], &L_enr1, 0, Q_syn, 3, 0);

            /*gain1 = (float)sqrt( enr_old / enr1 );*/
            exp = norm_l(L_enr_old);
            tmp = extract_h(L_shl(L_enr_old, exp));
            exp2 = norm_l(L_enr1);
            tmp2 = extract_h(L_shl(L_enr1, exp2));

            exp2 = sub(exp, exp2); /* Denormalize and substract */

            tmp3 = sub(tmp2, tmp);

            IF (tmp3 > 0)
            {
                tmp2 = shr(tmp2, 1);
                exp2 = add(exp2, 1);
            }

            tmp = div_s(tmp2, tmp);

            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp2);
            gain1 = round_fx(L_shl(L_tmp, sub(exp2, 1))); /* in Q14 */
            /* exp2 is always <= 1 */

            gain1 = s_min(gain1, 19661);

            /*--------------------------------------------------------*
             * Prevent a catastrophy in case of offset followed by onset
             *--------------------------------------------------------*/
            test();
            if( ( sub(clas,ONSET) == 0 ) && (sub(gain1,gain2) > 0) )
            {
                gain1 = gain2;
                move16();
            }
        }
        /*-----------------------------------------------------------------*
         * Smooth the energy evolution by exponentially evolving from
         * gain1 to gain2
         *-----------------------------------------------------------------*/

        L_tmp = L_mult(gain2, (Word16)(32768 - AGC_FX));

        FOR( i=0; i<L_frame; i++ )
        {
            gain1 = mac_r(L_tmp, gain1, AGC_FX); /* in Q14 */
            exc[i] = mac_r(L_mult(exc[i],  gain1),  exc[i], gain1);
            move16();
            exc2[i] = mac_r(L_mult(exc2[i], gain1), exc2[i], gain1);
            move16();
        }

        Copy(mem_tmp, mem_syn, M );
        syn_12k8_fx( L_frame, Aq, exc2, synth, mem_syn, 1,Q_exc,Q_syn );
        *update_flg = 1;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Update the LP filter energy for voiced frames
     *-----------------------------------------------------------------*/
    test();
    if( sub(clas,VOICED_TRANSITION) >= 0 && sub(clas,INACTIVE_CLAS) < 0 )
    {
        *old_enr_LP = enr_LP;
        move16();
    }

    return;

}
