/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "basop_util.h"
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"


/*==============================================================================*/
/* FUNCTION : inov_encode_fx()										        		*/
/*------------------------------------------------------------------------------*/
/* PURPOSE :  Encode the algebraic innovation						            */
/*------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    		*/
/* _ (Word16) core_brate: core bitrate	                             Q0  		*/
/* _ (Word16) Opt_AMR_WB: flag indicating AMR-WB IO mode	         Q0  		*/
/* _ (Word16) bwidth	: input signal bandwidth	                 Q0  		*/
/* _ (Word16) L_frame_fx	: length of the frame		    	     Q0			*/
/* _ (Word16[]) h2		: weighted filter input response		     Q12		*/
/* _ (Word16[]) xn2		: target vector              			     Q_new      */
/* _ (Word16) coder_type_fx	: coding type							 Q0			*/
/* _ (Word16) i_subfr		: current sub frame indicator            Q0			*/
/* _ (Word16[]) exc_fx		: pointer to excitation signal frame     Q_new		*/
/* _ (Word16) L_subfr		: subframe length 			             Q0			*/
/* _ (Word16) clip_gain		: adaptive gain clipping flag            Q0			*/
/* _ (Word16) gain_pit		: adaptive excitation gain 			     Q14		*/
/* _ (Word16) sharpFlag		: formant sharpening flag				 Q0			*/
/* _ (Word16) tc_subfr		: TC subframe index 			         Q0		    */
/* _ (Word16) p_Aq			: LP filter coefficients				 Q12		*/
/* _ (Word16) Jopt_flag		:joint optimization flag				 Q0			*/
/* _ (Word16) y1			: Filtered adaptive excitation			 Q_new		*/
/* _ (Word16) y2		:zero-memory filtered algebraic excitation 	 Q_new		*/
/* _ (Word16) cn			: target vector in residual domain       Q_new		*/
/* _ (Word16) tilt_code		: tilt of the excitation of previous subframe Q15   */
/* _ (Word16) pt_pitch		: pointer to current subframe fractional pitchQ6    */
/* _ (Word16) index_buf_4T		:5Sx4Track buffer for PI 			  Q0		*/
/* _ (Word16) shift		:shift													*/
/* _ (Word16) Q_new		: 														*/
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :															*/
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)					*/
/* _ (Word16) cn			: target vector in residual domain       Q_new		*/
/* _ (Word16) code		:algebraic excitation 			              Q9    	*/
/* _ (Word16) y2		:zero-memory filtered algebraic excitation 	 Q_new		*/
/* _ (Word16) unbits		:number of unused bits for  PI  		  Q0		*/
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :															*/
/* _ None																		*/
/*==============================================================================*/

Word16 inov_encode_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word32  core_brate,       /* i  : core bitrate                                    */
    const Word16 Opt_AMR_WB,       /* i  : flag indicating AMR-WB IO mode                  */
    const Word16 L_frame,          /* i  : length of the frame                             */
    const Word16 last_L_frame,     /* i  : length of the last frame                        */
    const Word16 coder_type,       /* i  : coding type                                     */
    const Word16 bwidth,           /* i  : input signal bandwidth                          */
    const Word16 sharpFlag,        /* i  : formant sharpening flag                         */
    const Word16 i_subfr,          /* i  : subframe index                                  */
    const Word16 tc_subfr,         /* i  : TC subframe index                               */
    const Word16 *p_Aq,            /* i  : LP filter coefficients                          Q12*/
    const Word16 gain_pit,         /* i  : adaptive excitation gain                        Q14*/
    Word16 *cn,              /* i/o: target vector in residual domain                Q_new*/
    const Word16 *exc,             /* i  : pointer to excitation signal frame              Q_new*/
    Word16 *h2,              /* i/o: weighted filter input response                  Q12*/
    const Word16 tilt_code,        /* i  : tilt of the excitation of previous subframe     Q15*/
    const Word16 pt_pitch,         /* i  : pointer to current subframe fractional pitch    Q6*/
    const Word16 *xn2,             /* i  : target vector for innovation search             Q_new-1+shift*/
    Word16 *code,            /* o  : algebraic excitation                            Q9*/
    Word16 *y2,              /* o  : zero-memory filtered algebraic excitation       Q9*/
    Word16 *unbits,          /* o  : number of unused bits for  PI                   */
    Word16 shift
)
{
    Word16 dn[L_SUBFR];
    Word16 nBits, cmpl_flag;
    Word16 stack_pulses;
    Word16 g1, g2;
    Word16 Rw[L_SUBFR];
    Word16 acelpautoc;

    stack_pulses = 0;
    move16();

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        g1 = FORMANT_SHARPENING_G1;
        move16();
        g2 = FORMANT_SHARPENING_G2;
        move16();
    }
    ELSE
    {
        g1 = FORMANT_SHARPENING_G1_16k;
        move16();
        g2 = FORMANT_SHARPENING_G2_16k;
        move16();
    }

    /*----------------------------------------------------------------*
     * Update target vector for codebook search in residual domain
     * Preemphasize the impulse response and include fixed-gain pitch contribution into impulse resp. h1[] (pitch sharpenning)
     * Correlation between target xn2[] and impulse response h1[]
     *----------------------------------------------------------------*/

    test();
    IF (L_sub(core_brate, ACELP_13k20) > 0 && !Opt_AMR_WB)
    {
        acelpautoc = 1;
        move16();

        cb_shape_fx( 1, 1, 0, sharpFlag, 0, g1, g2, p_Aq, h2, tilt_code, shr(add(pt_pitch,26),6) );

        /* h2: Q11, Rw: (Rw_e)Q */
        /* Rw_e = */ E_ACELP_hh_corr(h2, Rw, L_SUBFR, 3);

        E_ACELP_conv(xn2, h2, cn);

        /* dn_e -> Rw_e*Q_xn */
        /*dn_e = */ E_ACELP_toeplitz_mul( Rw, cn, dn, L_SUBFR, 1 );
    }
    ELSE
    {
        acelpautoc = 0;
        move16();
        updt_tar_fx( cn, cn, &exc[i_subfr], gain_pit, L_SUBFR );
        /* scaling of cn[] to limit dynamic at 12 bits */
        Scale_sig(cn, L_SUBFR, shift);

        cb_shape_fx( 1, 1, 0, sharpFlag, 0, g1, g2, p_Aq, h2, tilt_code, shr(add(pt_pitch,26),6) );

        corr_xh_fx( xn2, dn, h2 );
    }

    /*-----------------------------------------------------------------*
     * Set complexity reduction flag to limit the number of iterations
     * in algebraic innovation search
     *-----------------------------------------------------------------*/
    cmpl_flag = 0;
    move16();
    test();
    IF( sub(L_frame,L_FRAME) == 0 && sub(coder_type,TRANSITION) == 0 )
    {
        test();
        test();
        if( L_sub(core_brate,ACELP_8k00) == 0 && i_subfr == 0 && sub(tc_subfr,L_SUBFR) < 0 )
        {
            cmpl_flag = 3;
            move16();
        }
        test();
        test();
        test();
        if( L_sub(core_brate,ACELP_9k60) == 0 && ( (i_subfr == 0 && sub(tc_subfr,L_SUBFR) < 0) || sub(tc_subfr,TC_0_0) == 0) )
        {
            cmpl_flag = 3;
            move16();
        }
        test();
        test();
        test();
        test();
        test();
        if( L_sub(core_brate,ACELP_11k60) == 0 && ( (i_subfr == 0 && sub(tc_subfr,L_SUBFR) < 0) || sub(tc_subfr,TC_0_0) == 0 || (sub(i_subfr,3*L_SUBFR ) == 0&& sub(tc_subfr,TC_0_64) == 0)) )
        {
            cmpl_flag = 3;
            move16();
        }
        test();
        test();
        test();
        test();
        if( (L_sub(core_brate,ACELP_13k20) == 0 || L_sub(core_brate,ACELP_12k15) == 0 ) && ( (i_subfr == 0 && sub(tc_subfr,L_SUBFR) < 0) || sub(tc_subfr,TC_0_64) <= 0 ) )
        {
            cmpl_flag = 3;
            move16();
        }
    }

    IF( sub(L_frame,L_FRAME16k) == 0)
    {
        IF( L_sub(core_brate,ACELP_32k) <= 0 )
        {
            cmpl_flag = 4;
            move16();

            test();
            IF( sub(coder_type,TRANSITION) == 0 && sub(bwidth,WB) > 0 )
            {
                IF( sub(i_subfr,L_SUBFR) <= 0 )
                {
                    cmpl_flag = sub(cmpl_flag,1);
                }
                ELSE
                {
                    cmpl_flag = sub(cmpl_flag,2);
                }
            }
        }
        ELSE IF( L_sub(core_brate,ACELP_48k) <= 0 )
        {
            cmpl_flag = 3;
            move16();

            IF( sub(coder_type,TRANSITION) == 0 )
            {
                IF( sub(i_subfr,L_SUBFR) <= 0 )
                {
                    cmpl_flag = sub(cmpl_flag,1);
                }
                ELSE
                {
                    cmpl_flag = sub(cmpl_flag,2);
                }
            }
        }
        ELSE
        {
            cmpl_flag = 4;
            move16();

            IF( sub(coder_type,TRANSITION) == 0 )
            {
                IF( sub(i_subfr,L_SUBFR) <= 0 )
                {
                    cmpl_flag = sub(cmpl_flag,1);
                }
                ELSE
                {
                    cmpl_flag = sub(cmpl_flag,2);
                }
            }

            if( sub(coder_type,INACTIVE) == 0 )
            {
                cmpl_flag = 4;
                move16();
            }
        }
    }

    test();
    test();
    test();
    IF( sub(L_frame,last_L_frame) != 0 && L_sub(core_brate,ACELP_13k20) > 0 && (L_sub(core_brate,ACELP_32k) < 0 || sub(bwidth,WB) == 0) )
    {
        if( sub(cmpl_flag,1) > 0 )
        {
            cmpl_flag = sub(cmpl_flag,1);
        }
    }

    /*-----------------------------------------------------------------*
     * Find and encode the algebraic innovation
     *-----------------------------------------------------------------*/

    set16_fx( y2, 0, L_SUBFR );

    IF ( !Opt_AMR_WB )
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            nBits = FCB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
            move16();
            test();
            if( sub(coder_type,INACTIVE) == 0 && L_sub(core_brate,ACELP_24k40) > 0 )
            {
                nBits = 12;
                move16();
            }
        }
        ELSE  /* L_frame == L_FRAME16k */
        {
            nBits = FCB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
            move16();

        }

        IF( sub(nBits,7) == 0 )
        {
            acelp_1t64_fx( st_fx, dn, h2, code, y2
                         );
        }
        ELSE IF( sub(nBits,12) == 0 )
        {
            acelp_2t32_fx(st_fx, dn, h2, code, y2
                         );
        }
        ELSE
        {
            *unbits = add(*unbits, acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, nBits, cmpl_flag, Opt_AMR_WB));
            move16();
        }
    }
    ELSE
    {
        IF (L_sub(core_brate,ACELP_6k60) == 0)
        {
            acelp_2t32_fx( st_fx, dn, h2, code, y2
                         );
        }
        ELSE IF( (L_sub(core_brate,ACELP_8k85) == 0) )
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 20, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_12k65) == 0)
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 36, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_14k25) == 0)
        {
            acelp_4t64_fx( st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 44, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_15k85) == 0)
        {
            acelp_4t64_fx( st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 52, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_18k25) == 0)
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 64, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_19k85) == 0)
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 72, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_23k05) == 0 )
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 88, cmpl_flag, Opt_AMR_WB );
        }
        ELSE IF( L_sub(core_brate,ACELP_23k85) == 0)
        {
            acelp_4t64_fx(st_fx, dn, cn, h2, Rw, acelpautoc, code, y2, 88, 1, Opt_AMR_WB);
        }
    }


    /*----------------------------------------------------------------*
     * Pitch sharpening
     *----------------------------------------------------------------*/

    cb_shape_fx( 1, 1, 0, sharpFlag, 0, g1, g2, p_Aq, code, tilt_code, shr(add(pt_pitch,26),6) );

    return stack_pulses;

}
