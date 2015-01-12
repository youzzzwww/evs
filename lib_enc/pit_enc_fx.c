/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_enc_fx.h"    /* Encoder static table prototypes        */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "rom_basop_util.h"

#define inv_T0_res InvIntTable

static void limit_T0_voiced2( Word16 res, const Word16 *T_op, Word16 *T0_min, Word16 *T0_min_frac,
                              Word16 *T0_max, Word16 *T0_max_frac, Word16 pit_min, Word16 pit_max, Word16 i_subfr );

/*==============================================================================*/
/* FUNCTION : pit_encode_fx()										        		*/
/*------------------------------------------------------------------------------*/
/* PURPOSE :   Close-loop pitch lag search and pitch lag quantization			*/
/*			   Adaptive excitation construction									*/
/*------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    		*/
/* _ (Word16) core_brate: core bitrate	                             Q0  		*/
/* _ (Word16) Opt_AMR_WB: flag indicating AMR-WB IO mode	         Q0  		*/
/* _ (Word16) bwidth	: input signal bandwidth	                 Q0  		*/
/* _ (Word16[]) T_op	: open loop pitch estimates in current frame Q0  		*/
/* _ (Word16) T0_min	:  lower limit for close-loop search	     Q0  		*/
/* _ (Word16) T0_max	: higher limit for close-loop search	     Q0  		*/
/* _ (Word16) T0		: close loop integer pitch	                 Q0  		*/
/* _ (Word16) T0_frac	: close loop fractional part of the pitch	 Q0  		*/
/* _ (Word16) L_frame_fx	: length of the frame		    	     Q0			*/
/* _ (Word16[]) h1		: weighted filter input response		     Q15		*/
/* _ (Word16[]) xn		: target vector              			     Q_new      */
/* _ (Word16) coder_type_fx	: coding type							 Q0			*/
/* _ (Word16) i_subfr		: current sub frame indicator            Q0			*/
/* _ (Word16[]) exc_fx		: pointer to excitation signal frame     Q_new		*/
/* _ (Word16[]) L_subfr		: subframe length 			             Q0			*/
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :															*/
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)					*/
/* _ (Word16) T0_min	:  lower limit for close-loop search	     Q0  		*/
/* _ (Word16) T0_max	: higher limit for close-loop search	     Q0  		*/
/* _ (Word16) T0		: close loop integer pitch	                 Q0  		*/
/* _ (Word16) T0_frac	: close loop fractional part of the pitch	 Q0  		*/
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :															*/
/* _ None																		*/
/*==============================================================================*/

Word16 pit_encode_fx(              /* o  : Fractional pitch for each subframe         */
    Encoder_State_fx *st_fx,           /* i/o: encoder state structure                    */
    const Word32 core_brate,       /* i  : core bitrate                               */
    const Word16 Opt_AMR_WB,       /* i  : flag indicating AMR-WB IO mode             */
    const Word16 L_frame,          /* i  : length of the frame                        */
    const Word16 coder_type,       /* i  : coding type                                */
    Word16 *limit_flag,      /* i/o: restrained(0) or extended(1) Q limits      */
    const Word16 i_subfr,          /* i  : subframe index                             */
    Word16 *exc,             /* i/o: pointer to excitation signal frame         */
    const Word16 L_subfr,          /* i  : subframe length                            */
    const Word16 *T_op,            /* i  : open loop pitch estimates in current frame */
    Word16 *T0_min,          /* i/o: lower limit for close-loop search          */
    Word16 *T0_max,          /* i/o: higher limit for close-loop search         */
    Word16 *T0,              /* i/o: close loop integer pitch                   */
    Word16 *T0_frac,         /* i/o: close loop fractional part of the pitch    */
    const Word16 *h1,              /* i  : weighted filter input response             */
    const Word16 *xn               /* i  : target vector                              */
)
{
    Word16 pitch;
    Word16 pit_flag, delta, mult_Top, nBits;


    /*----------------------------------------------------------------*
     * Set pit_flag to 0 for every subframe with absolute pitch search
     *----------------------------------------------------------------*/
    pit_flag = i_subfr;
    move16();
    if (sub(i_subfr,2*L_SUBFR) == 0)
    {
        pit_flag = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Limit range of pitch search
     * Fractional pitch search
     * Pitch quantization
     *-----------------------------------------------------------------*/
    mult_Top = 1;
    move16();

    IF( !Opt_AMR_WB )
    {
        /*----------------------------------------------------------------*
         * pitch Q: Set limit_flag to 0 for restrained limits, and 1 for extended limits
         *----------------------------------------------------------------*/
        test();
        test();
        IF( i_subfr == 0 )
        {
            *limit_flag = 1;
            move16();
            if( sub(coder_type,VOICED) == 0 )
            {
                *limit_flag = 2;
                move16();   /* double-extended limits */
            }
            test();
            if( sub(coder_type,GENERIC) == 0 && L_sub(core_brate,ACELP_7k20) == 0 )
            {
                *limit_flag = 0;
                move16();
            }
        }
        ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 && sub(coder_type,GENERIC) == 0 && L_sub(core_brate,ACELP_13k20) <= 0 )
        {
            /*if( *T0 > (PIT_FR1_EXTEND_8b + PIT_MIN)>>1 )*/
            if( sub(*T0,shr(add(PIT_FR1_EXTEND_8b , PIT_MIN), 1)) > 0)
            {
                *limit_flag = 0;
                move16();
            }
        }

        IF( *limit_flag == 0 )
        {
            test();
            test();
            IF( i_subfr == 0 && sub(T_op[0],PIT_MIN) < 0 )
            {
                mult_Top = 2;
                move16();
            }
            ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 && sub(T_op[1],PIT_MIN) < 0 )
            {
                mult_Top = 2;
                move16();
            }
        }
        /*-------------------------------------------------------*
         *  Retrieve the number of Q bits
         *-------------------------------------------------------*/
        nBits = 0;
        move16();
        IF( sub(coder_type,AUDIO) != 0 )
        {
            /* find the number of bits */
            IF( sub(L_frame,L_FRAME) == 0 )
            {
                {
                    nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, 0)];
                    move16();
                }
            }
            ELSE  /* L_frame == L_FRAME16k */
            {
                nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, 0)];
                move16();
            }
        }
        IF( sub(coder_type,AUDIO) == 0 )
        {
            /*-------------------------------------------------------*
             *  Pitch encoding in AUDIO mode
             *  (both ACELP@12k8 and ACELP@16k cores)
             *-------------------------------------------------------*/

            delta = 4;
            move16();
            test();
            test();
            if ( sub(L_subfr,L_frame/2) == 0 && i_subfr != 0 && sub(L_frame,L_FRAME) == 0 )
            {
                pit_flag = L_SUBFR;
                move16();
            }
            IF ( pit_flag == 0 )
            {
                nBits = 10;
                move16();
            }
            ELSE
            {
                nBits = 6;
                move16();
            }

            /* pitch lag search limitation */
            test();
            IF( i_subfr == 0 )
            {
                limit_T0_fx( L_frame, delta, pit_flag, *limit_flag, mult_Top*T_op[0], 0, T0_min, T0_max );
            }
            ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 && pit_flag == 0 )
            {
                limit_T0_fx( L_frame, delta, pit_flag, *limit_flag, mult_Top*T_op[1], 0, T0_min, T0_max );
            }

            /* search and encode the closed loop pitch period */
            IF( sub(L_frame,L_FRAME) == 0 )
            {
                IF( *limit_flag == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX, PIT_MAX, L_FRAME, L_subfr );
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX_EXTEND, PIT_MAX_EXTEND, L_FRAME, L_subfr );
                }
                pit_Q_enc_fx( st_fx, 0, nBits, delta, pit_flag, *limit_flag, *T0, *T0_frac, T0_min, T0_max
                            );
            }
            ELSE
            {
                IF( *limit_flag == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_MAX, PIT16k_MAX, L_FRAME16k, L_SUBFR );
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_FR2_EXTEND_10b, PIT16k_MAX, L_FRAME16k, L_SUBFR );
                }

                pit16k_Q_enc_fx( st_fx, nBits, *limit_flag, *T0, *T0_frac, T0_min, T0_max
                               );
            }
        }
        ELSE IF( sub(coder_type,VOICED) == 0 )
        {
            /*-------------------------------------------------------*
             *  Pitch encoding in VOICED mode (ACELP@12k8 core only)
             *-------------------------------------------------------*/

            delta = 4;
            move16();
            if ( sub(i_subfr,2*L_SUBFR) == 0 )
            {
                pit_flag = i_subfr;
                move16();
            }

            /* pitch lag search limitation */
            IF (i_subfr == 0)
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, mult_Top*T_op[0], 0, T0_min, T0_max );
            }

            /* search and encode the closed loop pitch period */
            test();
            test();
            IF( sub(nBits,8) == 0 || sub(nBits,4) == 0 )
            {

                IF( *limit_flag == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
                    move16();
                }
                ELSE IF( sub(*limit_flag,1) == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN_EXTEND, PIT_FR1_EXTEND_8b, L_FRAME, L_SUBFR );
                    move16();
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN_DOUBLEEXTEND, PIT_FR1_DOUBLEEXTEND_8b, L_FRAME, L_SUBFR );
                    move16();
                }

            }
            ELSE IF( sub(nBits,9) == 0 || sub(nBits,5) == 0 )
            {
                IF( *limit_flag == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_9b, PIT_FR1_9b, L_FRAME, L_SUBFR );
                    move16();
                }
                ELSE IF( sub(*limit_flag,1) == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_EXTEND_9b, PIT_FR1_EXTEND_9b, L_FRAME, L_SUBFR );
                    move16();
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_DOUBLEEXTEND_9b, PIT_FR1_DOUBLEEXTEND_9b, L_FRAME, L_SUBFR );
                    move16();
                }
            }
            ELSE IF( sub(nBits,10) == 0 )
            {
                IF( *limit_flag == 0 )
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX, PIT_MAX, L_FRAME, L_SUBFR );
                    move16();
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX_EXTEND, PIT_MAX_EXTEND, L_FRAME, L_SUBFR );
                    move16();
                }
            }

            pit_Q_enc_fx( st_fx, 0, nBits, delta, pit_flag, *limit_flag, *T0, *T0_frac, T0_min, T0_max
                        );

        }
        ELSE
        {
            /*-------------------------------------------------------*
             *  Pitch encoding in GENERIC mode
             *  (both ACELP@12k8 and ACELP@16k cores)
             *-------------------------------------------------------*/

            delta = 8;
            move16();

            /* pitch lag search limitation */
            IF( i_subfr == 0 )
            {
                limit_T0_fx( L_frame, delta, pit_flag, *limit_flag, mult_Top*T_op[0], 0, T0_min, T0_max );
            }
            ELSE IF( sub(i_subfr,2*L_SUBFR) == 0)
            {
                limit_T0_fx( L_frame, delta, pit_flag, *limit_flag, mult_Top*T_op[1], 0, T0_min, T0_max );
            }

            /* search and encode the closed loop pitch period */
            IF( sub(L_frame,L_FRAME) == 0 )
            {
                test();
                test();
                IF( sub(nBits,8) == 0 || sub(nBits,5) == 0 )
                {
                    IF( *limit_flag == 0 )
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
                    }
                    ELSE
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN_EXTEND, PIT_FR1_EXTEND_8b, L_FRAME, L_SUBFR );
                    }
                }
                ELSE IF( sub(nBits,9) == 0 || sub(nBits,6) == 0)
                {
                    IF( *limit_flag == 0 )
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_9b, PIT_FR1_9b, L_FRAME, L_SUBFR );
                    }
                    ELSE
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_EXTEND_9b, PIT_FR1_EXTEND_9b, L_FRAME, L_SUBFR );
                    }
                }
                ELSE IF( sub(nBits,10) == 0 )
                {
                    IF( *limit_flag == 0 )
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX, PIT_MAX, L_FRAME, L_SUBFR );
                    }
                    ELSE
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MAX_EXTEND, PIT_MAX_EXTEND, L_FRAME, L_SUBFR );
                    }
                }

                pit_Q_enc_fx( st_fx, 0, nBits, delta, pit_flag, *limit_flag, *T0, *T0_frac, T0_min, T0_max
                            );
            }
            ELSE  /* L_frame == L_FRAME16k */
            {
                test();
                IF( sub(nBits,9) == 0 || sub(nBits,6) == 0 )
                {
                    IF( *limit_flag == 0 )
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_FR2_9b, PIT16k_FR1_9b, L_FRAME16k, L_SUBFR );
                    }
                    ELSE
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_FR2_EXTEND_9b, PIT16k_FR1_EXTEND_9b, L_FRAME16k, L_SUBFR );
                    }
                }
                ELSE IF( nBits == 10 )
                {
                    IF( *limit_flag == 0 )
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_MAX, PIT16k_MAX, L_FRAME16k, L_SUBFR );
                    }
                    ELSE
                    {
                        *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT16k_FR2_EXTEND_10b, PIT16k_MAX_EXTEND, L_FRAME16k, L_SUBFR );
                    }
                }

                pit16k_Q_enc_fx( st_fx, nBits, *limit_flag, *T0, *T0_frac, T0_min, T0_max );
            }
        }
    }

    /*-------------------------------------------------------*
     *  Pitch encoding in AMR-WB IO mode
     *-------------------------------------------------------*/

    ELSE
    {
        delta = 8;
        move16();
        *limit_flag = 0;
        move16();

        IF( L_sub(core_brate,ACELP_6k60) == 0 )
        {
            nBits = 5;
            move16();

            /* pitch lag search limitation */
            IF( i_subfr == 0 )
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, i_mult2(mult_Top,T_op[0]), 0, T0_min, T0_max );
                nBits = 8;
                move16();
            }

            if( sub(i_subfr,2*L_SUBFR) == 0 )
            {
                /* rewrite pit_flag - it must not be zero */
                pit_flag = i_subfr;
                move16();
            }

            /* search and encode the closed loop pitch period */
            *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
        }
        ELSE IF( L_sub(core_brate,ACELP_8k85) == 0 )
        {
            nBits = 5;
            move16();

            /* pitch lag search limitation */
            IF( i_subfr == 0 )
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, i_mult2(mult_Top,T_op[0]), 0, T0_min, T0_max );
                nBits = 8;
                move16();
            }
            ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 )
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, i_mult2(mult_Top,T_op[1]), 0, T0_min, T0_max );
                nBits = 8;
                move16();
            }

            /* search and encode the closed loop pitch period */
            *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
        }
        ELSE
        {
            nBits = 6;
            move16();

            /* pitch lag search limitation */
            IF( i_subfr == 0 )
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, i_mult2(mult_Top,T_op[0]), 0, T0_min, T0_max );
                nBits = 9;
                move16();
            }
            ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 )
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, *limit_flag, i_mult2(mult_Top,T_op[1]), 0, T0_min, T0_max );
                nBits = 9;
                move16();
            }
            ELSE
            {
                limit_T0_fx( L_FRAME, delta, pit_flag, 0, *T0, 0, T0_min, T0_max );         /* T0_frac==0 to keep IO with AMR-WB */
            }

            /* search and encode the closed loop pitch period */
            *T0 = pitch_fr4_fx( &exc[i_subfr], xn, h1, *T0_min, *T0_max, T0_frac, pit_flag, *limit_flag, PIT_FR2_9b, PIT_FR1_9b, L_FRAME, L_SUBFR );
        }

        pit_Q_enc_fx( st_fx, 1, nBits, delta, pit_flag, *limit_flag, *T0, *T0_frac, T0_min, T0_max );
    }

    /*-------------------------------------------------------*
     * Compute floating pitch output
     *-------------------------------------------------------*/

    /*pitch = (float)(*T0) + (float)(*T0_frac)/4.0f;*/   /* save subframe pitch values  */
    pitch = shl(add(shl(*T0,2),*T0_frac),4);   /* save subframe pitch values Q6  */

    return pitch;

}

/*-------------------------------------------------------------------*
 * abs_pit_enc()
 *
 * Encode pitch lag absolutely with resolution for shortest pitches
 * depending on parameter 'fr_step':
 * fr_step = 2: pitch range encoded with 8 bits
 * fr_step = 4: pitch range encoded with 8 bits
 *-------------------------------------------------------------------*/

Word16 abs_pit_enc_fx(           /* o  : pitch index                                              */
    const Word16 fr_steps,    /* i  : fractional resolution steps (2 or 4) for shortest pitches*/
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) limits */
    const Word16 T0,          /* i  : integer pitch lag                                        */
    const Word16 T0_frac      /* i  : pitch fraction                                           */
)
{
    Word16 pitch_index = 0;

    IF( limit_flag == 0 )
    {
        IF( sub(fr_steps,2) == 0 )
        {
            /*-----------------------------------------------------------------*
             * The pitch range is encoded absolutely with 8 bits
             * and is divided as follows:
             *   PIT_MIN to PIT_FR1_8b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT_FR1_8b to PIT_MAX    resolution 1   (frac = 0)
             *-----------------------------------------------------------------*/

            IF (sub(T0,PIT_FR1_8b) < 0)
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT_MIN*2);*/
                pitch_index =   sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_MIN*2));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT_FR1_8b + ((PIT_FR1_8b-PIT_MIN)*2);*/
                pitch_index = add(sub(T0,PIT_FR1_8b),((PIT_FR1_8b-PIT_MIN)*2));
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            /*-------------------------------------------------------------------*
            * The pitch range is encoded absolutely with 9 bits
            * and is divided as follows:
            *   PIT_MIN    to PIT_FR2_9b-1  resolution 1/4 (frac = 0,1,2 or 3)
            *   PIT_FR2_9b to PIT_FR1_9b-1  resolution 1/2 (frac = 0 or 2)
            *   PIT_FR1_9b to PIT_MAX       resolution 1   (frac = 0)
            *-------------------------------------------------------------------*/

            IF (sub(T0,PIT_FR2_9b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT_MIN*4);*/
                pitch_index = add(shl(T0,2),sub(T0_frac,(PIT_MIN*4)));
            }
            ELSE IF (sub(T0,PIT_FR1_9b) < 0)
            {
                /*  pitch_index = T0*2 + (T0_frac>>1) - (PIT_FR2_9b*2) + ((PIT_FR2_9b-PIT_MIN)*4);*/
                pitch_index = 	add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_FR2_9b*2)),((PIT_FR2_9b-PIT_MIN)*4));
            }
            ELSE
            {
                /* pitch_index = T0 - PIT_FR1_9b + ((PIT_FR2_9b-PIT_MIN)*4) + ((PIT_FR1_9b-PIT_FR2_9b)*2);*/
                pitch_index =  add(add(sub(T0,PIT_FR1_9b),((PIT_FR2_9b-PIT_MIN)*4)),((PIT_FR1_9b-PIT_FR2_9b)*2));
            }
        }
        ELSE  /* fr_step == 0 */
        {
            /* not used in the codec */
            pitch_index = 0;
            move16();
        }
    }
    ELSE IF( sub(limit_flag,1) == 0 )   /* extended Q range */
    {
        IF( sub(fr_steps,2) == 0 )
        {
            /*-----------------------------------------------------------------*
             * The pitch range is encoded absolutely with 8 bits
             * and is divided as follows:
             *   PIT_MIN_EXTEND to PIT_FR1_EXTEND_8b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT_FR1_EXTEND_8b to PIT_MAX_EXTEND    resolution 1   (frac = 0)
             *-----------------------------------------------------------------*/

            IF( sub(T0,PIT_FR1_EXTEND_8b) < 0 )
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT_MIN_EXTEND*2);*/
                pitch_index =   sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_MIN_EXTEND*2));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT_FR1_EXTEND_8b + ((PIT_FR1_EXTEND_8b-PIT_MIN_EXTEND)*2);*/
                pitch_index = add(sub(T0,PIT_FR1_EXTEND_8b),((PIT_FR1_EXTEND_8b-PIT_MIN_EXTEND)*2));
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            /*-------------------------------------------------------------------*
             * The pitch range is encoded absolutely with 9 bits
             * and is divided as follows:
             *   PIT_MIN_EXTEND    to PIT_FR2__EXTEND9b-1  resolution 1/4 (frac = 0,1,2 or 3)
             *   PIT_FR2_EXTEND_9b to PIT_FR1__EXTEND9b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT_FR1_EXTEND_9b to PIT_MAX_EXTEND       resolution 1   (frac = 0)
             *-------------------------------------------------------------------*/

            IF( sub(T0,PIT_FR2_EXTEND_9b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT_MIN_EXTEND*4);*/
                pitch_index = add(shl(T0,2),sub(T0_frac,(PIT_MIN_EXTEND*4)));
            }
            ELSE IF( T0 < PIT_FR1_EXTEND_9b )
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT_FR2_EXTEND_9b*2) + ((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4);*/
                pitch_index = add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_FR2_EXTEND_9b*2)),((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT_FR1_EXTEND_9b + ((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4) + ((PIT_FR1_EXTEND_9b-PIT_FR2_EXTEND_9b)*2);*/
                pitch_index =  add(add(sub(T0,PIT_FR1_EXTEND_9b),((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4)),((PIT_FR1_EXTEND_9b-PIT_FR2_EXTEND_9b)*2));
            }

        }
        ELSE  /* fr_step == 0 */
        {
            /* not used in the codec */
            pitch_index = 0;
            move16();
        }
    }
    ELSE  /* double-extended Q range */
    {
        IF( sub(fr_steps,2) == 0 )
        {
            /*-----------------------------------------------------------------*
             * The pitch range is encoded absolutely with 8 bits
             * and is divided as follows:
             *   PIT_MIN_DOUBLEEXTEND    to PIT_FR1_DOUBLEEXTEND_8b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT_FR1_DOUBLEEXTEND_8b to PIT_MAX_EXTEND             resolution 1   (frac = 0)
             *-----------------------------------------------------------------*/

            IF( sub(T0,PIT_FR1_DOUBLEEXTEND_8b) < 0)
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT_MIN_DOUBLEEXTEND*2);*/
                pitch_index =   sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_MIN_DOUBLEEXTEND*2));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT_FR1_DOUBLEEXTEND_8b + ((PIT_FR1_DOUBLEEXTEND_8b-PIT_MIN_DOUBLEEXTEND)*2);               */
                pitch_index = add(sub(T0,PIT_FR1_DOUBLEEXTEND_8b),((PIT_FR1_DOUBLEEXTEND_8b-PIT_MIN_DOUBLEEXTEND)*2));
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            /*-------------------------------------------------------------------*
             * The pitch range is encoded absolutely with 9 bits
             * and is divided as follows:
             *   PIT_MIN_DOUBLEEXTEND    to PIT_FR2_DOUBLEEXTEND9b-1  resolution 1/4 (frac = 0,1,2 or 3)
             *   PIT_FR2_DOUBLEEXTEND_9b to PIT_FR1_DOOBLEEXTEND9b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT_FR1_DOUBLEEXTEND_9b to PIT_MAX_EXTEND           resolution 1   (frac = 0)
             *-------------------------------------------------------------------*/

            IF(sub(T0,PIT_FR2_DOUBLEEXTEND_9b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT_MIN_DOUBLEEXTEND*4);*/
                pitch_index = add(shl(T0,2),sub(T0_frac,(PIT_MIN_DOUBLEEXTEND*4)));
            }
            ELSE IF( sub(T0,PIT_FR1_DOUBLEEXTEND_9b) < 0 )
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT_FR2_DOUBLEEXTEND_9b*2) + ((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4);*/
                pitch_index = add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT_FR2_DOUBLEEXTEND_9b*2)),((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT_FR1_DOUBLEEXTEND_9b + ((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4) + ((PIT_FR1_DOUBLEEXTEND_9b-PIT_FR2_DOUBLEEXTEND_9b)*2);*/
                pitch_index =  add(add(sub(T0,PIT_FR1_DOUBLEEXTEND_9b),((PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4)),((PIT_FR1_DOUBLEEXTEND_9b-PIT_FR2_DOUBLEEXTEND_9b)*2));
            }
        }
        ELSE  /* fr_step == 0 */
        {
            /* not used in the codec */
            pitch_index = 0;
            move16();
        }
    }

    return pitch_index;
}


/*-------------------------------------------------------------------*
 * delta_pit_enc:
 *
 * Encode pitch lag differentially from T0_min to T0_max
 * with resolution depending on parameter 'fr_step':
 * fr_step = 0: resolusion 1   (frac = 0), or
 * fr_step = 2: resolusion 1/2 (frac = 0 or 2), or
 * fr_step = 4: resolution 1/4 (frac = 0, 1, 2, or 3)
 *-------------------------------------------------------------------*/

Word16 delta_pit_enc_fx(         /* o  : pitch index                         */
    const Word16 fr_steps,     /* i  : fractional resolution steps (2 or 4)*/
    const Word16 T0,           /* i  : integer pitch lag                   */
    const Word16 T0_frac,      /* i  : pitch fraction                      */
    const Word16 T0_min        /* i  : delta search min                    */
)
{
    Word16 pitch_index = 0;

    IF( fr_steps == 0 )
    {
        pitch_index = sub(T0,T0_min);
    }
    ELSE IF( sub(fr_steps,2) == 0 )
    {
        /* pitch_index = (T0 - T0_min) * 2 + (T0_frac>>1);*/
        pitch_index = add(shl(sub(T0,T0_min),1),shr(T0_frac,1));
    }
    ELSE IF( sub(fr_steps,4) == 0 )
    {
        /*pitch_index = (T0 - T0_min) * 4 + T0_frac;*/
        pitch_index = add(shl(sub(T0,T0_min),2),T0_frac);
    }

    return pitch_index;
}

/*-------------------------------------------------------------------*
 * pitch_fr4()
 *
 * Find the closed loop pitch period with 1/4 subsample resolution.
 *-------------------------------------------------------------------*/

Word16 pitch_fr4_fx(         /* o  : chosen integer pitch lag                 */
    const Word16 exc[],      /* i  : excitation buffer                          Q_new*/
    const Word16 xn[],       /* i  : target signal                              Q_new-1+shift*/
    const Word16 h[],        /* i  : weighted synthesis filter impulse response Q(14+shift)*/
    const Word16 t0_min,     /* i  : minimum value in the searched range.       Q0*/
    const Word16 t0_max,     /* i  : maximum value in the searched range.       Q0*/
    Word16 *pit_frac,  /* o  : chosen fraction (0, 1, 2 or 3)             */
    const Word16 i_subfr,    /* i  : flag to first subframe                     */
    const Word16 limit_flag, /* i  : flag for limits (0=restrained, 1=extended) */
    const Word16 t0_fr2,     /* i  : minimum value for resolution 1/2           */
    const Word16 t0_fr1,     /* i  : minimum value for resolution 1             */
    const Word16 L_frame,    /* i  : length of the frame                        */
    const Word16 L_subfr     /* i  : size of subframe                           */
)
{
    Word16 i;
    Word16 t_min, t_max;
    Word16 max, t0, t1, fraction, step, temp;
    Word16 *corr;
    Word16 corr_v[15+2*L_INTERPOL1+1];                     /* Total length = t0_max-t0_min+1+2*L_inter */
    Word16 pit_min;
    Word16 cor_max;

    /* initialization */
    IF( limit_flag == 0 )
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            pit_min = PIT_MIN;
            move16();
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            pit_min = PIT16k_MIN;
            move16();
        }
    }
    ELSE
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            pit_min = PIT_MIN_EXTEND;
            move16();
            IF( sub(limit_flag,2) == 0 )
            {
                pit_min = PIT_MIN_DOUBLEEXTEND;
                move16();
            }
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            pit_min = PIT16k_MIN_EXTEND;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * - Find interval to compute normalized correlation
     * - allocate memory to normalized correlation vector
     * - Compute normalized correlation between target and filtered
     *   excitation
     *-----------------------------------------------------------------*/

    t_min = sub(t0_min, L_INTERPOL1);
    t_max = add(t0_max, L_INTERPOL1);
    corr = &corr_v[-t_min];
    move16();
    move16();         /* corr[t_min..t_max] */

    norm_corr_fx( exc, xn, h, t_min, t_max, corr, L_subfr );

    /*-----------------------------------------------------------------*
     * Find integer pitch
     *-----------------------------------------------------------------*/

    max = corr[t0_min];
    move16();
    t0 = t0_min;
    move16();

    FOR (i = add(t0_min, 1); i <= t0_max; i++)
    {
        if (corr[i] >= max)
        {
            t0 = i;
            move16();
        }
        max = s_max(corr[i], max);
    }

    IF( sub(t0_fr1,pit_min) == 0 )
    {
        /* don't search fraction (for 7b/4b quant) */
        test();
        IF((i_subfr == 0) && (sub(t0,t0_fr2) >= 0))
        {
            i = shl(shr(t0,1),1);       /* 2 samples resolution */
            if (sub(add(i,2),PIT_MAX) > 0)
            {
                i = sub(i,2);
            }
            IF (sub(corr[i],corr[i+2]) > 0)
            {
                t0 = i;
                move16();
            }
            ELSE
            {
                t0 = add(i,2);
            }
        }
        *pit_frac = 0;
        move16();

        return(t0);
    }

    test();
    IF( (i_subfr == 0) && (sub(t0,t0_fr1) >= 0) )
    {
        *pit_frac = 0;
        move16();

        return(t0);
    }

    /*------------------------------------------------------------------*
     * Search fractionnal pitch with 1/4 subsample resolution.
     * search the fractions around t0 and choose the one which maximizes
     * the interpolated normalized correlation.
     *-----------------------------------------------------------------*/

    t1 = t0;
    move16();
    step = 1;
    move16();              /* 1/4 subsample resolution */
    fraction = 1;
    move16();
    test();
    test();
    IF (((i_subfr == 0) && (sub(t0,t0_fr2) >= 0)) || (sub(t0_fr2,pit_min) == 0))
    {
        step = 2;
        move16();          /* 1/2 subsample resolution */
        fraction = 2;
        move16();
    }

    IF (sub(t0,t0_min) == 0)        /* Limit case */
    {
        fraction = 0;
        move16();
        cor_max = Interpol_4( &corr[t0], fraction);
    }
    ELSE
    {
        t0 = sub(t0, 1);
        cor_max = Interpol_4( &corr[t0], fraction);
        FOR(i = add(fraction, step); i <= 3; i = (Word16) (i + step))
        {
            temp = Interpol_4( &corr[t0], i);
            IF (sub(temp,cor_max) > 0)
            {
                cor_max = temp;
                move16();
                fraction = i;
                move16();
            }
        }
    }

    FOR (i = 0; i <= 3; i = (Word16) (i + step))
    {
        temp = Interpol_4( &corr[t1], i);
        IF (sub(temp,cor_max) > 0)
        {
            cor_max = temp;
            move16();
            fraction = i;
            move16();
            t0 = t1;
            move16();
        }
    }

    *pit_frac = fraction;
    move16();

    return (t0);

}

/*-------------------------------------------------------------------*
 * norm_corr()
 *
 * Find the normalized correlation between the target vector and the
 * filtered past excitation (correlation between target and filtered
 * excitation divided by the square root of energy of filtered
 * excitation)
 *---------------------------------------------------------------------*/

void norm_corr_fx(
    const Word16 exc[],        /* i  : excitation buffer                          Q_new*/
    const Word16 xn[],         /* i  : target signal                              Q_new-1+shift*/
    const Word16 h[],          /* i  : weighted synthesis filter impulse response Q(14+shift)*/
    const Word16 t_min,        /* i  : minimum value of searched range            */
    const Word16 t_max,        /* i  : maximum value of searched range            */
    Word16 ncorr[],       /* o  : normalized correlation                     Q15 */
    const Word16 L_subfr       /* i  : subframe size                              */
)
{
    Word16 i, k, t;
    Word16 corr, exp_corr, norm, exp_norm, exp, scale;
    Word16 excf[L_FRAME16k];
    Word32 L_tmp;

    k = negate(t_min);

    /*-----------------------------------------------------------------*
     * compute the filtered excitation for the first delay t_min
     *-----------------------------------------------------------------*/

    conv_fx( &exc[k], h, excf, L_subfr );

    /* Compute rounded down 1/sqrt(energy of xn[]) */
    L_tmp = L_mac(1, xn[0], xn[0]);
    FOR (i = 1; i < L_subfr; i++)
    {
        L_tmp = L_mac(L_tmp, xn[i], xn[i]);
    }
    exp = norm_l(L_tmp);
    exp = sub(30, exp);

    exp = add(exp, 2);              /* energy of xn[] x 2 + rounded up     */
    scale = negate(shr(exp, 1));    /* (1<<scale) < 1/sqrt(energy rounded) */

    /*----------------------------------------------------------------*
     * loop for every possible period
     *----------------------------------------------------------------*/

    FOR (t = t_min; t <= t_max; t++)
    {
        /* Compute correlation between xn[] and excf[] */

        L_tmp = L_mac(1, xn[0], excf[0]);
        FOR (i = 1; i < L_subfr; i++)
        {
            L_tmp = L_mac(L_tmp, xn[i], excf[i]);
        }
        exp = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, exp);
        exp_corr = sub(30, exp);
        corr = extract_h(L_tmp);

        /* Compute 1/sqrt(energy of excf[]) */
        L_tmp = L_mac(1, excf[0], excf[0]);
        FOR (i = 1; i < L_subfr; i++)
        {
            L_tmp = L_mac(L_tmp, excf[i], excf[i]);
        }

        exp = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, exp);
        exp_norm = sub(30, exp);

        L_tmp = Isqrt_lc(L_tmp, &exp_norm);
        norm = extract_h(L_tmp);

        /* Normalize correlation = correlation * (1/sqrt(energy)) */
        L_tmp = L_mult(corr, norm);
        L_tmp = L_shl(L_tmp, add(add(exp_corr, exp_norm), scale));

        ncorr[t] = round_fx(L_tmp);

        /* update the filtered excitation excf[] for the next iteration */
        IF (sub(t, t_max) != 0)
        {
            k--;
            FOR (i = (Word16) (L_subfr - 1); i > 0; i--)
            {
                /* saturation can occur in add() */
                /*excf[i] = add(mult(exc[k], h[i]), excf[i - 1]);     move16();       */
                excf[i] = round_fx(L_mac(L_mult(excf[i - 1], 32767), exc[k], h[i]));
            }
            excf[0] = mult_r(exc[k], h[0]);
            move16();
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * pit_Q_enc()
 *
 * Encode subframe pitch lag
 *-------------------------------------------------------------------*/

void pit_Q_enc_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure */
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode          */
    const Word16 nBits,        /* i  : # of Q bits                             */
    const Word16 delta,        /* i  : Half the CL searched interval           */
    const Word16 pit_flag,     /* i  : absolute(0) or delta(1) pitch Q         */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) Q limits   */
    const Word16 T0,           /* i  : integer pitch lag                       */
    const Word16 T0_frac,      /* i  : pitch fraction                          */
    Word16 *T0_min,      /* i/o: delta search min                        */
    Word16 *T0_max       /* o  : delta search max                        */
)
{
    Word16 pitch_index;

    IF( sub(nBits,10) == 0 )         /* absolute encoding with 10 bits */
    {
        IF( limit_flag == 0 )
        {
            /* pitch_index = T0*4 + T0_frac - (PIT_MIN*4);*/
            pitch_index = sub(add(shl(T0 , 2), T0_frac), (PIT_MIN*4));
        }
        ELSE IF( sub(limit_flag,1) == 0 )
        {
            /*pitch_index = T0*4 + T0_frac - (PIT_MIN_EXTEND*4);*/
            pitch_index = sub(add(shl(T0 , 2),T0_frac),(PIT_MIN_EXTEND*4));
        }
        ELSE  /* limit_flag == 2 */
        {
            /*pitch_index = T0*4 + T0_frac - (PIT_MIN_DOUBLEEXTEND*4);*/
            pitch_index = sub(add(shl(T0 , 2) ,T0_frac) , (PIT_MIN_DOUBLEEXTEND*4));
        }
    }
    ELSE IF( sub(nBits,9) == 0 )     /* absolute encoding with 9 bits */
    {
        pitch_index = abs_pit_enc_fx( 4, limit_flag, T0, T0_frac );

        /* find T0_min and T0_max for delta search */
        IF( Opt_AMR_WB )
        {
            limit_T0_fx( L_FRAME, delta, pit_flag, 0, T0, 0, T0_min, T0_max );         /* T0_frac==0 to keep IO with AMR-WB */
        }
    }
    ELSE IF( sub(nBits,8) == 0 )     /* absolute encoding with 8 bits */
    {
        pitch_index = abs_pit_enc_fx( 2, limit_flag, T0, T0_frac );

        /* find T0_min and T0_max for delta search */
        IF( Opt_AMR_WB )
        {
            limit_T0_fx( L_FRAME, delta, pit_flag, 0, T0, 0, T0_min, T0_max );         /* T0_frac==0 to keep IO with AMR-WB */
        }
    }
    ELSE IF( sub(nBits,6) == 0 )     /* relative encoding with 6 bits */
    {
        pitch_index = delta_pit_enc_fx( 4, T0, T0_frac, *T0_min );
    }
    ELSE IF( sub(nBits,5) == 0 )     /* relative encoding with 5 bits */
    {
        IF( sub(delta,8) == 0 )
        {
            pitch_index = delta_pit_enc_fx( 2, T0, T0_frac, *T0_min );
        }
        ELSE  /* delta == 4 */
        {
            pitch_index = delta_pit_enc_fx( 4, T0, T0_frac, *T0_min );
        }
    }
    ELSE  /* nBits == 4 ) */  /* relative encoding with 4 bits */
    {
        IF( sub(delta,8) == 0 )
        {
            pitch_index = delta_pit_enc_fx( 0, T0, T0_frac, *T0_min );
        }
        ELSE  /* delta == 4 */
        {
            pitch_index = delta_pit_enc_fx( 2, T0, T0_frac, *T0_min );
        }
    }

    IF( !Opt_AMR_WB )
    {
        /* find T0_min and T0_max for delta search */
        limit_T0_fx( L_FRAME, delta, L_SUBFR, limit_flag, T0, T0_frac, T0_min, T0_max );
    }

    {
        push_indice_fx( st_fx, IND_PITCH, pitch_index, nBits );
    }

    return;
}

/*-------------------------------------------------------------------*
 * pit16k_Q_enc()
 *
 * Encode subframe pitch lag @16kHz core
 *-------------------------------------------------------------------*/

void pit16k_Q_enc_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    const Word16 nBits,        /* i  : # of Q bits                             */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) Q limits   */
    const Word16 T0,           /* i  : integer pitch lag                       */
    const Word16 T0_frac,      /* i  : pitch fraction                          */
    Word16 *T0_min,      /* i/o: delta search min                        */
    Word16 *T0_max       /* o  : delta search max                        */
)
{
    Word16 pitch_index;

    IF( sub(nBits,10) == 0 )         /* absolute encoding with 10 bits */
    {
        IF( limit_flag == 0 )
        {
            /*pitch_index = (T0 - PIT16k_MIN) * 4 + T0_frac;*/
            pitch_index = add(shl(sub(T0,PIT16k_MIN), 2), T0_frac);
        }
        ELSE  /* extended Q range */
        {
            IF( sub(T0,PIT16k_FR2_EXTEND_10b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT16k_MIN_EXTEND*4);*/
                pitch_index = add(shl(T0 , 2) , sub(T0_frac , (PIT16k_MIN_EXTEND*4)));
            }
            ELSE
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT16k_FR2_EXTEND_10b*2) + ((PIT16k_FR2_EXTEND_10b-PIT16k_MIN_EXTEND)*4);*/
                pitch_index =  add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT16k_FR2_EXTEND_10b*2)),((PIT16k_FR2_EXTEND_10b-PIT16k_MIN_EXTEND)*4));
            }
        }

        push_indice_fx( st_fx, IND_PITCH, pitch_index, nBits );
    }
    ELSE IF( sub(nBits,9) == 0 )     /* absolute encoding with 9 bits */
    {
        IF( limit_flag == 0 )
        {
            /*-------------------------------------------------------------------*
             * The pitch range is encoded absolutely with 9 bits
             * and is divided as follows:
             *   PIT16k_MIN    to PIT16k_FR2_9b-1  resolution 1/4 (frac = 0,1,2 or 3)
             *   PIT16k_FR2_9b to PIT16k_FR1_9b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT16k_FR1_9b to PIT16k_MAX       resolution 1   (frac = 0)
             *-------------------------------------------------------------------*/

            IF( sub(T0,PIT16k_FR2_9b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT16k_MIN*4);*/
                pitch_index = add(shl(T0 , 2) , sub(T0_frac , (PIT16k_MIN*4)));
            }
            ELSE IF( sub(T0,PIT16k_FR1_9b) < 0)
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT16k_FR2_9b*2) + ((PIT16k_FR2_9b-PIT16k_MIN)*4);*/
                pitch_index =  add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT16k_FR2_9b*2)),((PIT16k_FR2_9b-PIT16k_MIN)*4));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT16k_FR1_9b + ((PIT16k_FR2_9b-PIT16k_MIN)*4) + ((PIT16k_FR1_9b-PIT16k_FR2_9b)*2);*/
                pitch_index = add(add(sub(T0, PIT16k_FR1_9b) , ((PIT16k_FR2_9b-PIT16k_MIN)*4)), ((PIT16k_FR1_9b-PIT16k_FR2_9b)*2));
            }
        }
        ELSE  /* extended Q range */
        {
            /*-------------------------------------------------------------------*
             * The pitch range is encoded absolutely with 9 bits
             * and is divided as follows:
             *   PIT16k_EXTEND_MIN    to PIT16k_FR2_EXTEND_9b-1  resolution 1/4 (frac = 0,1,2 or 3)
             *   PIT16k_FR2_EXTEND_9b to PIT16k_FR1_EXTEND_9b-1  resolution 1/2 (frac = 0 or 2)
             *   PIT16k_FR1_EXTEND_9b to PIT16k_MAX_EXTEND       resolution 1   (frac = 0)
             *-------------------------------------------------------------------*/

            IF( sub(T0,PIT16k_FR2_EXTEND_9b) < 0)
            {
                /*pitch_index = T0*4 + T0_frac - (PIT16k_MIN_EXTEND*4);*/
                pitch_index = add(shl(T0 , 2) , sub(T0_frac , (PIT16k_MIN_EXTEND*4)));
            }
            ELSE IF( sub(T0,PIT16k_FR1_EXTEND_9b) < 0 )
            {
                /*pitch_index = T0*2 + (T0_frac>>1) - (PIT16k_FR2_EXTEND_9b*2) + ((PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4);*/
                pitch_index =  add(sub(add(shl(T0,1),shr(T0_frac,1)),(PIT16k_FR2_EXTEND_9b*2)),((PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4));
            }
            ELSE
            {
                /*pitch_index = T0 - PIT16k_FR1_EXTEND_9b + ((PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4) + ((PIT16k_FR1_EXTEND_9b-PIT16k_FR2_EXTEND_9b)*2);*/
                pitch_index = add(add(sub(T0, PIT16k_FR1_EXTEND_9b) , ((PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4)), ((PIT16k_FR1_EXTEND_9b-PIT16k_FR2_EXTEND_9b)*2));
            }
        }

        push_indice_fx( st_fx, IND_PITCH, pitch_index, 9 );
    }
    ELSE  /* nBits == 6 */    /* relative encoding with 6 bits */
    {
        /*pitch_index = (T0 - *T0_min) * 4 + T0_frac;*/
        pitch_index = add(shl(sub(T0,*T0_min),2),T0_frac);

        push_indice_fx( st_fx, IND_PITCH, pitch_index, nBits );
    }

    limit_T0_fx( L_FRAME16k, 8, L_SUBFR, limit_flag, T0, T0_frac, T0_min, T0_max );

    return;
}


/*------------------------------------------------------------------*
 * pit_encode:
 *
 * Close-loop pitch lag search and pitch lag quantization
 * Adaptive excitation construction
 *------------------------------------------------------------------*/
void Mode2_pit_encode(
    const Word16 coder_type,   /* i  : coding model                               */
    const Word16 i_subfr,      /* i  : subframe index                             */
    Word16 **pt_indice,  /* i/o: quantization indices pointer               */
    Word16 *exc,         /* i/o: pointer to excitation signal frame         */
    const Word16 *T_op,        /* i  : open loop pitch estimates in current frame */
    Word16 *T0_min,      /* i/o: lower limit for close-loop search          */
    Word16 *T0_min_frac, /* i/o: lower limit for close-loop search          */
    Word16 *T0_max,      /* i/o: higher limit for close-loop search         */
    Word16 *T0_max_frac, /* i/o: higher limit for close-loop search         */
    Word16 *T0,          /* i/o: close loop integer pitch                   */
    Word16 *T0_frac,     /* i/o: close loop fractional part of the pitch    */
    Word16 *T0_res,      /* i/o: close loop pitch resolution                */
    Word16 *h1,          /* i  : weighted filter impulse response 1Q14+shift*/
    Word16 *xn,          /* i  : target vector                              */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr1b,
    Word16 pit_fr2,
    Word16 pit_max,
    Word16 pit_res_max
)
{
    Word16 pit_flag;

    BASOP_SATURATE_ERROR_ON;

    /* Pitch flag */
    pit_flag = i_subfr;
    move16();
    if ( sub(i_subfr,(2*L_SUBFR)) == 0)
    {
        pit_flag = 0;
        move16();
    }

    /*-----------------------------------------------------------------*
     *  - Limit range of pitch search
     *  - Fractional pitch search
     *  - Pitch quantization
     *-----------------------------------------------------------------*/
    IF(coder_type == 0) /*Unvoiced Coding do nothing*/
    {
        *T0 = L_SUBFR;
        move16();
        *T0_frac = 0;
        move16();
        *T0_res = 1;
        move16();

    }
    ELSE IF(sub(coder_type,1) == 0) /* 8/4/4/4 (EVS) */
    {
        IF (i_subfr == 0)
        {
            limit_T0_voiced( 4, shr(pit_res_max,1), T_op[0], 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        ELSE
        {
            limit_T0_voiced( 4, shr(pit_res_max,1), *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, shr(pit_res_max,1), T0_frac, T0_res, pit_res_max,
                                        i_subfr, pit_min, pit_min, pit_fr1b, L_SUBFR);
        move16();

        IF (i_subfr == 0)
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max );
        }
        ELSE
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, shr(pit_res_max,1), *T0_min, *T0_min_frac, pt_indice );
        }

    }
    ELSE IF(sub(coder_type,2) == 0) /* 8/5/8/5 (EVS) */
    {
        IF (i_subfr == 0)
        {
            limit_T0_voiced( 5, shr(pit_res_max,1), T_op[0], 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        ELSE IF(i_subfr == 2*L_SUBFR)
        {
            limit_T0_voiced( 5, shr(pit_res_max,1), T_op[1], 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        ELSE
        {
            limit_T0_voiced( 5, shr(pit_res_max,1), *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, shr(pit_res_max,1), T0_frac, T0_res, pit_res_max,
                                        pit_flag, pit_min, pit_min, pit_fr1b, L_SUBFR);
        move16();

        IF (pit_flag == 0)
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max );
        }
        ELSE
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, shr(pit_res_max,1), *T0_min, *T0_min_frac, pt_indice );
        }
    }
    ELSE IF(sub(coder_type,3) == 0) /* 9/6/6/6 (HRs- VC) */
    {
        Word16 pit_res_max2 = pit_res_max;

        if ( sub(pit_min,PIT_MIN_16k)==0 )
        {
            pit_res_max2 = shr(pit_res_max, 1);
        }

        IF ( (i_subfr == 0) )
        {
            limit_T0_voiced2( pit_res_max2, T_op, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max, i_subfr );
        }
        ELSE
        {
            limit_T0_voiced( 6, pit_res_max2, *T0, 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max);
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, pit_res_max2, T0_frac, T0_res, pit_res_max,
                                        i_subfr, pit_min, pit_fr2, pit_fr1, L_SUBFR);
        move16();

        IF (i_subfr == 0) /* if 1st subframe */
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1, pit_fr2, pit_res_max );
        }
        ELSE
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, pit_res_max2, *T0_min, *T0_min_frac, pt_indice );
        }
    }
    ELSE IF(coder_type == 4) /* 9/6/9/6 (AMRWB) */
    {
        Word16 pit_res_max2 = pit_res_max;
        if ( sub(pit_min,PIT_MIN_16k) == 0 )
        {
            pit_res_max2 = shr(pit_res_max,1);
        }
        test();
        IF ( (i_subfr == 0) || sub(i_subfr,shl(L_SUBFR,1)) == 0 )
        {
            limit_T0_voiced2( pit_res_max2, T_op, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max, i_subfr );
        }
        ELSE
        {
            limit_T0_voiced( 6, pit_res_max2, *T0, 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max);
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, pit_res_max2, T0_frac, T0_res, pit_res_max,
                                        pit_flag, pit_min, pit_fr2, pit_fr1, L_SUBFR);
        IF (pit_flag == 0) /* if 1st/3rd/5th subframe */
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1, pit_fr2, pit_res_max );
        }
        ELSE /* if subframe 2 or 4 */
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, pit_res_max2, *T0_min, *T0_min_frac, pt_indice );
        }
    }
    ELSE IF(sub(coder_type,8) == 0) /* 8/5/5/5 (RF all pred mode) */
    {
        IF (i_subfr == 0)
        {
            limit_T0_voiced( 5, pit_res_max>>1, T_op[0], 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        ELSE
        {
            limit_T0_voiced( 5, pit_res_max>>1, *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, pit_res_max>>1, T0_frac, T0_res, pit_res_max,
                                        i_subfr, pit_min, pit_min, pit_fr1b, L_SUBFR );

        IF (i_subfr == 0)
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max );
        }
        ELSE
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, (pit_res_max>>1), *T0_min, *T0_min_frac, pt_indice );
        }
    }
    ELSE IF(sub(coder_type,9) == 0) /* 8/0/8/0 (RF mode Gen pred) */
    {
        IF (i_subfr == 0)
        {
            limit_T0_voiced( 4, pit_res_max>>1, T_op[0], 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        ELSE
        {
            limit_T0_voiced( 4, pit_res_max>>1, *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
        }
        *T0 = E_GAIN_closed_loop_search(exc, xn, h1, *T0_min, *T0_min_frac, *T0_max, *T0_max_frac, pit_res_max>>1, T0_frac, T0_res, pit_res_max,
                                        i_subfr, pit_min, pit_min, pit_fr1b, L_SUBFR );

        IF (i_subfr == 0)
        {
            Mode2_abs_pit_enc( *T0, *T0_frac, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max );
        }
        ELSE
        {
            Mode2_delta_pit_enc( *T0, *T0_frac, (pit_res_max>>1), *T0_min, *T0_min_frac, pt_indice );
        }
    }

    BASOP_SATURATE_ERROR_OFF;

    return;
}

static void limit_T0_voiced2(
    Word16 res,
    const Word16 *T_op,
    Word16 *T0_min,
    Word16 *T0_min_frac,
    Word16 *T0_max,
    Word16 *T0_max_frac,
    Word16 pit_min,
    Word16 pit_max,
    Word16 i_subfr
)
{
    Word16 t, temp1, temp2, res2;

    assert(res > 1 && res<=6);

    res2 = res;
    move16();
    if(sub(res,6) == 0)
    {
        res2 =shr(res2,1);
    }

    /* Lower-bound */
    IF (i_subfr == 0)
    {
        temp1 = sub(i_mult2(T_op[0],res),32);
    }
    ELSE
    {
        temp1 = sub(i_mult2(T_op[1],res),32);
    }

    IF (sub(T_op[0],T_op[1])<0)
    {
        t = sub(i_mult2(T_op[0],res),16);
    }
    ELSE
    {
        t = sub(i_mult2(T_op[1],res),16);
    }

    if (sub(temp1,t)<0)
    {
        temp1 = t;
    }

    temp2 = mult(temp1,inv_T0_res[res2]);
    if(sub(res,6) == 0)
    {
        temp2 = shr(temp2,1);
    }

    *T0_min = temp2;
    move16();

    *T0_min_frac = sub(temp1,i_mult2(temp2,res));
    move16();

    IF ( sub(*T0_min,pit_min) < 0)
    {
        *T0_min = pit_min;
        move16();
        *T0_min_frac = 0;
        move16();
    }

    /* Higher-bound */
    temp1 = add(i_mult2(*T0_min,res),add(*T0_min_frac,63));

    IF (T_op[0]<T_op[1])
    {
        t = add(i_mult2(T_op[1],res),add(15,res));
    }
    ELSE
    {
        t = add(i_mult2(T_op[0],res),add(15,res));
    }

    if (sub(temp1,t)>0)
    {
        temp1 = t;
        move16();
    }

    temp2 = mult(temp1,inv_T0_res[res]);

    *T0_max = temp2;
    move16();

    *T0_max_frac = sub(temp1, i_mult2(temp2,res));

    IF ( sub(*T0_max,pit_max) > 0)
    {
        *T0_max = pit_max;
        *T0_max_frac = sub(res,1);

        temp1 = add(sub(i_mult2(*T0_max,res),64),res);

        temp2 = mult(temp1,inv_T0_res[res2]);
        if(sub(res,6) == 0)
        {
            temp2 = shr(temp2,1);
        }

        *T0_min = temp2;
        move16();

        *T0_min_frac = sub(temp1,i_mult2(temp2,res));
        move16();
    }

}

/*-------------------------------------------------------------------*
 * abs_pit_enc:
 *
 * Encode pitch lag absolutely
 *-------------------------------------------------------------------*/

void Mode2_abs_pit_enc(
    Word16 T0,          /* i  : integer pitch lag              */
    Word16 T0_frac,     /* i  : pitch fraction                 */
    Word16 **pt_indice, /* i/o: pointer to Vector of Q indexes */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr2,
    Word16 pit_res_max
)
{
    Word16 pit_res_max_half;


    pit_res_max_half = shr(pit_res_max,1);

    IF (sub(T0, pit_fr2) < 0)
    {
        **pt_indice = add( i_mult2(T0, pit_res_max), sub( T0_frac, i_mult2(pit_min, pit_res_max) ) );
    }
    ELSE IF (sub(T0, pit_fr1) < 0)
    {
        **pt_indice = add( sub( add(i_mult2(T0,pit_res_max_half), T0_frac), i_mult2(pit_fr2,pit_res_max_half) ), i_mult2(sub(pit_fr2, pit_min), pit_res_max) );
    }
    ELSE
    {
        **pt_indice = add( add( sub(T0, pit_fr1), i_mult2( sub(pit_fr2, pit_min), pit_res_max) ), i_mult2( sub(pit_fr1, pit_fr2), pit_res_max_half) );
    }

    (*pt_indice)++;

}


/*-------------------------------------------------------------------*
 * delta_pit_enc:
 *
 * Encode pitch lag differentially
 *-------------------------------------------------------------------*/

void Mode2_delta_pit_enc(
    Word16 T0,          /* i  : integer pitch lag              */
    Word16 T0_frac,     /* i  : pitch fraction                 */
    Word16 T0_res,      /* i  : pitch resolution               */
    Word16 T0_min,      /* i/o: delta search min               */
    Word16 T0_min_frac, /* i/o: delta search min               */
    Word16 **pt_indice  /* i/o: pointer to Vector of Q indexes */
)
{
    /***pt_indice = (T0 - T0_min) * T0_res + T0_frac - T0_min_frac;*/

    **pt_indice = add(i_mult2(sub(T0, T0_min), T0_res), sub(T0_frac, T0_min_frac));
    move16();
    (*pt_indice)++;

}

