/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"

/*----------------------------------------------------------------------*
  * Local functions
  *----------------------------------------------------------------------*/

static void tc_dec_fx( Decoder_State_fx *st_fx, const Word16 L_frame,Word16 exc[], Word16 *T0,Word16 *T0_frac, const Word16 i_subfr,
                       const Word16 tc_subfr, Word16 *position, const Word32 core_brate, Word16 bwe_exc[], Word16 *Q_exc );

/*======================================================================*/
/* FUNCTION : transition_dec_fx()									    */
/*----------------------------------------------------------------------*/
/* PURPOSE : Principal function for TC decoding                    		*/
/*																		*/
/*----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS :												*/
/*    const Word32  core_brate, : core bitrate            Q0            */
/* _ (Word16) L_frame_fx		: length of the frame		Q0			*/
/* _ (Word16[]) pitch_buf_fx	: floating pitch values for each subframe Q6*/
/* _ (Word16[])	voice_factors_fx: frame error rate				Q15		*/
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													 */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)			 */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q0)		 */
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													 */
/* _ None																 */
/*=======================================================================*/

void transition_dec_fx(
    Decoder_State_fx *st_fx,       /* i/o: decoder state structure */
    const Word32  core_brate,  /* i  : core bitrate                            */
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode          */
    const Word16 L_frame,      /* i  : length of the frame                     */
    const Word16 i_subfr,      /* i  : subframe index                          */
    const Word16 coder_type,   /* i  : coder type                              */
    const Word16 tc_subfr,     /* i  : TC subframe index                       */
    Word16 *Jopt_flag,   /* i  : joint optimization flag                 */
    Word16 *exc,         /* o  : excitation signal                       */
    Word16 *T0,          /* o  : close loop integer pitch                */
    Word16 *T0_frac,     /* o  : close loop fractional part of the pitch */
    Word16 *T0_min,      /* i/o: delta search min for sf 2 & 4           */
    Word16 *T0_max,      /* i/o: delta search max for sf 2 & 4           */
    Word16 **pt_pitch,   /* o  : floating pitch values                   */
    Word16 *position,    /* i/o: first glottal impulse position in frame */
    Word16 *bwe_exc,     /* o  : excitation for SWB TBE                  */
    Word16 *Q_exc        /*i/o : scaling of excitation                   */
)
{
    Word16 pit_flag, pit_start, pit_limit, index, nBits;
    Word16 i, offset,temp,tmp;
    Word16 limit_flag;

    /* Set limit_flag to 0 for restrained limits, and 1 for extended limits */
    limit_flag = 0;

    /*---------------------------------------------------------------------*
     * zero adaptive contribution (glottal shape codebook search not
     *                             in first subframe(s) )
     *---------------------------------------------------------------------*/
    IF(sub(tc_subfr, add(i_subfr,TC_0_192)) > 0)
    {
        set16_fx(&exc[i_subfr], 0, L_SUBFR);

        IF( sub(L_frame,L_FRAME) == 0 )
        {
            set16_fx(&bwe_exc[i_subfr*HIBND_ACB_L_FAC], 0, (Word16) (L_SUBFR*HIBND_ACB_L_FAC));         /* set past excitation buffer to 0 */
        }
        ELSE
        {
            set16_fx(&bwe_exc[i_subfr*2], 0, (Word16) (L_SUBFR*2));         /* set past excitation buffer to 0 */
        }

        *T0 = L_SUBFR;
        move16();
        *T0_frac = 0;
        move16();
        **pt_pitch = L_SUBFR_Q6;
        move16();
    }

    /*---------------------------------------------------------------------*
     * glottal shape codebook search
     *---------------------------------------------------------------------*/

    ELSE IF(((sub(tc_subfr,i_subfr) >= 0) && (sub(tc_subfr,i_subfr) <= TC_0_192) ))
    {
        set16_fx( exc-L_EXC_MEM, 0, L_EXC_MEM );         /* set past excitation buffer to 0 */

        IF( sub(L_frame,L_FRAME) == 0 )
        {
            set16_fx( bwe_exc-PIT_MAX*HIBND_ACB_L_FAC, 0, PIT_MAX*HIBND_ACB_L_FAC);         /* set past excitation buffer to 0 */
        }
        ELSE
        {
            set16_fx( bwe_exc-PIT16k_MAX*2, 0, PIT16k_MAX*2);         /* set past excitation buffer to 0 */
        }

        /* glottal shape codebook contribution construction */
        tc_dec_fx( st_fx, L_frame, exc, T0, T0_frac, i_subfr, tc_subfr, position, core_brate, bwe_exc, Q_exc );

        **pt_pitch = shl(add(shl(*T0,2),*T0_frac),4);
        move16(); /* save subframe pitch values Q6 */

        *Jopt_flag = 1;
        move16();
    }

    /*---------------------------------------------------------------------*
     * Regular ACELP Decoding using GENERIC type decoder
     * (all subframes following subframe with glottal shape codebook seach)
     * - search the position of the 2nd glottal impulse in case that the first
     *   one is in the 1st subframe (different adaptive contribution
     *   construction and the pitch period coding is used)
     *---------------------------------------------------------------------*/

    ELSE IF ( sub(tc_subfr,i_subfr) < 0)
    {
        IF( sub(L_frame,L_FRAME) == 0)
        {
            *Jopt_flag = 1;
            move16();
            test();
            IF( (sub(sub(i_subfr,tc_subfr),L_SUBFR)  >= 0) && (sub(sub(i_subfr,tc_subfr),L_SUBFR + TC_0_192)  <= 0) )
            {
                pit_flag = 0;
                move16();
            }
            ELSE
            {
                pit_flag = L_SUBFR;
                move16();
            }
            IF( sub(tc_subfr,TC_0_0) == 0)
            {
                IF( sub(i_subfr,L_SUBFR) == 0 )
                {
                    limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max );
                }
                pit_flag = 1;
                move16();
            }

            /*-----------------------------------------------------------------*
             * get number of bits for pitch decoding
             *-----------------------------------------------------------------*/

            nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
            move16();

            /*------------------------------------------------------------*
             * first glottal impulse is in the 1st subframe
             *------------------------------------------------------------*/
            test();
            test();
            test();
            test();
            test();
            test();
            IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(tc_subfr,TC_0_128) >= 0)  )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 3rd or 4th subframe
                 * - build exc[] in 2nd subframe
                 *--------------------------------------------------------*/

                *T0 = 2*L_SUBFR;
                move16();
                *T0_frac = 0;
                move16();
                *Jopt_flag = 0;
                move16();

                /* set adaptive part of exciation for curent subframe to 0 */
                set16_fx( &exc[i_subfr], 0, (Word16)(L_SUBFR+1) );
                set16_fx( &bwe_exc[i_subfr*HIBND_ACB_L_FAC], 0, (Word16)(L_SUBFR*HIBND_ACB_L_FAC) );

            }
            ELSE IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(tc_subfr,TC_0_64) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 2nd subframe,
                 * - build exc[] in 2nd subframe
                 *--------------------------------------------------------*/
                pit_start = PIT_MIN;
                move16();
                if (sub(PIT_MIN,(*position)) > 0)
                {
                    pit_start = sub(L_SUBFR, *position);
                }

                pit_start = s_max(pit_start, PIT_MIN );
                pit_limit = add(shl(pit_start,1), *position);

                /* 7 bit pitch DECODER */
                index = (Word16)get_next_indice_fx( st_fx, nBits );

                *T0 = add(pit_start, shr((index),1));
                move16();
                *T0_frac = shl(sub((index),shl(sub(*T0, pit_start),1)),1);
                move16();

                limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max );   /* find T0_min and T0_max */

                /* Find the adaptive codebook vector - ACELP long-term prediction   */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);

                move16();
                move16();  /* penality for 2 ptrs initialization */

                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC - offset];
                    move16();
                }

            }
            ELSE IF( (sub(i_subfr,2*L_SUBFR) == 0) && (sub(tc_subfr,TC_0_128) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 3rd subframe
                 * - build exc[] in 3rd subframe
                 *--------------------------------------------------------*/

                /* 7bit pitch DECODER */
                pit_start = sub(2*L_SUBFR, (*position));
                index = (Word16)get_next_indice_fx( st_fx, nBits );

                *T0 = add(pit_start, shr((index),1));
                move16();
                *T0_frac = shl(sub((index),shl(sub(*T0, pit_start),1)),1);
                move16();
                limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max );   /* find T0_min and T0_max */

                /* Find the adaptive codebook vector. ACELP long-term prediction */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);
                move16();
                move16();      /* penality for 2 ptrs initialization */

                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC -  offset];
                    move16();
                }

            }
            ELSE IF((sub(i_subfr,2*L_SUBFR) == 0) && (sub(tc_subfr,TC_0_192) == 0))
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 4th subframe
                 * - build exc[] in 3rd subframe
                 *--------------------------------------------------------*/

                *T0 = 4*L_SUBFR;
                move16();
                *T0_frac = 0;
                move16();
                *Jopt_flag = 0;
                move16();

                /* set adaptive part of exciation for curent subframe to 0 */
                set16_fx( &exc[i_subfr], 0, (Word16)(L_SUBFR+1) );
                set16_fx( &bwe_exc[i_subfr*HIBND_ACB_L_FAC], 0, (Word16)(L_SUBFR*HIBND_ACB_L_FAC) );
            }
            ELSE IF( (sub(i_subfr,3*L_SUBFR) == 0) && (sub(tc_subfr,TC_0_192) == 0))
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 4th subframe
                 * - build exc[] in 4th subframe
                 *--------------------------------------------------------*/
                pit_start = sub(3*L_SUBFR, (*position));
                pit_limit = sub(2*L_FRAME - PIT_MAX, add(shl((*position),1), 2));


                index = (Word16)get_next_indice_fx( st_fx, nBits );

                IF( sub(index,shl(sub(pit_limit,pit_start),1)) < 0)
                {
                    *T0 = add( pit_start, shr(index,1));
                    move16();
                    *T0_frac = shl(sub(index, shl(sub((*T0), pit_start),1)),1);
                    move16();
                }
                ELSE
                {
                    *T0 = add(index, sub(pit_limit, shl(sub(pit_limit,pit_start),1)));
                    move16();

                    *T0_frac = 0;
                    move16();
                }

                /* Find the adaptive codebook vector. ACELP long-term prediction   */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);
                move16();
                move16();      /* penality for 2 ptrs initialization */
                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC - offset];
                }


            }
            ELSE IF( (sub(i_subfr,3*L_SUBFR) == 0) && (sub(tc_subfr,TC_0_128) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse in the 3rd subframe
                 * build exc[] in 4th subframe
                 *--------------------------------------------------------*/


                index = (Word16)get_next_indice_fx( st_fx, nBits );

                IF( sub(nBits,4) == 0 )
                {
                    delta_pit_dec_fx( 0, index, T0, T0_frac, *T0_min );
                }
                ELSE
                {
                    delta_pit_dec_fx( 2, index, T0, T0_frac, *T0_min );
                }

                /* Find the adaptive codebook vector. ACELP long-term prediction   */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);
                move16();
                move16();      /* penality for 2 ptrs initialization */
                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC - offset];
                    move16();
                }



            }
            /*------------------------------------------------------------*
             * first glottal impulse is NOT in the 1st subframe,
             * or two impulses are in the 1st subframe
             *------------------------------------------------------------*/
            ELSE
            {
                index = (Word16)get_next_indice_fx( st_fx, nBits );

                pit_Q_dec_fx( 0, index, nBits, 8, pit_flag, limit_flag, T0, T0_frac, T0_min, T0_max );

                /* Find the adaptive codebook vector */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);
                move16();
                move16();      /* penality for 2 ptrs initialization */
                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC -  offset];
                    move16();
                }

            }

            /*-----------------------------------------------------------------*
             * LP filtering of the adaptive excitation (if non-zero)
             *-----------------------------------------------------------------*/
            IF( *Jopt_flag )
            {
                lp_filt_exc_dec_fx( st_fx, MODE1, core_brate, Opt_AMR_WB, coder_type, i_subfr, L_SUBFR, L_frame, 0, exc );
            }

            /*---------------------------------------------------------------------*
             * fill the pitch buffer - needed for post-processing and FEC_clas_estim()
             *---------------------------------------------------------------------*/

            **pt_pitch = shl(add(shl(*T0,2),*T0_frac),4);
            move16();  /* save subframe pitch values Q6 */

            test();
            test();
            test();
            test();
            test();
            IF( (sub(tc_subfr,2*L_SUBFR) >= 0) && (sub(i_subfr,3*L_SUBFR) == 0) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch) -= 3;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch)++;
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();
            }
            ELSE IF( (sub(tc_subfr,L_SUBFR) == 0) && (sub(i_subfr,2*L_SUBFR) == 0) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch) -= 2;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch)++;
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();

            }
            ELSE IF( (sub(tc_subfr,TC_0_64) == 0) && (sub(i_subfr,L_SUBFR) == 0) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch) -= 1;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch)++;
            }
            ELSE IF( (sub(tc_subfr,TC_0_128) == 0) && (sub(i_subfr,2*L_SUBFR) == 0) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch) -= 2;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch)++;
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();
            }
            ELSE IF( (sub(tc_subfr,TC_0_192) == 0) && (sub(i_subfr,3*L_SUBFR) == 0) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch) -= 3;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch)++;
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();
                **pt_pitch = tmp;
                move16(); /*Q6*/
                (*pt_pitch) ++;
                move16();
            }
        }
        ELSE  /* L_frame == L_FRAME16k */
        {
            test();
            if( sub(i_subfr,2*L_SUBFR) >= 0)
            {
                limit_flag = 1;
                move16();
            }

            IF( sub(sub(i_subfr, tc_subfr), L_SUBFR) == 0 )
            {
                limit_T0_fx( L_FRAME16k, 8, 0, limit_flag, *T0, *T0_frac, T0_min, T0_max );
            }

            /*-----------------------------------------------------------------*
             * get number of bits and index for pitch decoding
             *-----------------------------------------------------------------*/

            nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
            move16();

            index = (Word16)get_next_indice_fx( st_fx, nBits );

            /*-----------------------------------------------------------------*
             * Find adaptive part of excitation, encode pitch period
             *-----------------------------------------------------------------*/
            test();
            IF( sub(nBits,10) == 0 || sub(nBits,9) == 0 )
            {
                pit16k_Q_dec_fx( index, nBits, limit_flag, T0, T0_frac, T0_min, T0_max );
            }
            ELSE IF( sub(nBits,8) == 0 )     /* tc_subfr==0 && i_subfr==L_SUBFR */
            {
                /*-----------------------------------------------------------------------------*
                 * The pitch range is encoded absolutely with 8 bits and is divided as follows:
                 *   PIT16k_MIN  to PIT16k_FR2_TC0_2SUBFR-1 resolution 1/4 (frac = 0,1,2 or 3)
                 *   PIT16k_FR2_TC0_2SUBFR to 2*L_SUBFR     resolution 1/2 (frac = 0 or 2)
                 *-----------------------------------------------------------------------------*/

                IF( sub(index,(PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4) < 0 )/*(PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4*/
                {
                    *T0 = add(PIT16k_MIN,shr(index,2));
                    move16();
                    temp = shl(sub(*T0,PIT16k_MIN),2);
                    move16();
                    *T0_frac = sub(index,temp);
                    move16();
                }
                ELSE
                {
                    index = sub(index,(PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4);   /* (PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4 */
                    *T0 =  add(PIT16k_FR2_TC0_2SUBFR,shr(index,1));
                    move16();
                    temp = shl(sub(*T0,PIT16k_FR2_TC0_2SUBFR),1);
                    move16();
                    *T0_frac = shl(sub(index,temp),1);
                    move16();
                }
            }
            ELSE IF( sub(nBits,6) == 0 )
            {
                delta_pit_dec_fx( 4, index, T0, T0_frac, *T0_min );
            }
            ELSE IF( sub(nBits,5) == 0)
            {
                delta_pit_dec_fx( 2, index, T0, T0_frac, *T0_min );
            }

            test();
            IF( sub(nBits,6) == 0 || sub(nBits,5) == 0 )
            {
                limit_T0_fx( L_FRAME16k, 8, L_SUBFR, limit_flag, *T0, *T0_frac, T0_min, T0_max );   /* find T0_min and T0_max */
            }

            /*-----------------------------------------------------------------*
             * - find the adaptive codebook vector
             * - LP filtering of the adaptive excitation (if non-zero)
             *-----------------------------------------------------------------*/
            test();
            IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(*T0,2*L_SUBFR) == 0) )
            {
                /* no adaptive excitation in the second subframe */
                set16_fx( &exc[i_subfr], 0, L_SUBFR+1 );
                get_next_indice_fx( st_fx, 1 );      /* this bit is actually not needed */
                set16_fx( &bwe_exc[i_subfr * 2], 0, L_SUBFR * 2 );
            }
            ELSE
            {
                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc[i_subfr], &exc[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);
                move16();
                move16();      /* penalty for 2 ptrs initialization */
                FOR (i=0; i<L_SUBFR*2; i++)
                {
                    /*  bwe_exc[i + i_subfr * 2] = bwe_exc[i + i_subfr * 2 - *T0 * 2 */
                    /*   - (int) ((float) *T0_frac * 0.5f + 4 + 0.5f) + 4];*/
                    bwe_exc[i + i_subfr * 2] = bwe_exc[i + i_subfr * 2 - offset];
                }

                lp_filt_exc_dec_fx( st_fx, MODE1, core_brate, Opt_AMR_WB, coder_type, i_subfr, L_SUBFR, L_frame, 0, exc );

                *Jopt_flag = 1;
                move16();
            }

            **pt_pitch = shl(add(shl(*T0,2),*T0_frac),4);
            move16(); /* save subframe pitch values Q6 */

            /*---------------------------------------------------------------------*
             * fill the pitch buffer - needed for post-processing and FEC_clas_estim()
             *---------------------------------------------------------------------*/
            test();
            test();
            IF( (sub(sub(i_subfr, tc_subfr),L_SUBFR) == 0) || (tc_subfr==0 && sub(i_subfr,2*L_SUBFR)==0) )
            {
                /*index = i_subfr/L_SUBFR;*/
                index = shr(i_subfr,6);

                /*Ptr */
                (*pt_pitch) -= index;
                move16();

                FOR( i=0; i<index; i++ )
                {
                    **pt_pitch = shl(add(shl(*T0,2),*T0_frac),4);
                    move16();   /* save subframe pitch values Q6 */

                    (*pt_pitch)++;
                }
            }
        }
    }

    return;
}


/*======================================================================*/
/* FUNCTION : tc_dec_fx()									            */
/*----------------------------------------------------------------------*/
/* PURPOSE : Principal function for TC decoding                    		*/
/*           *  Principal function for TC decoding.					    */
/*           *  - constructs glottal codebook contribution				*/
/*          *  - uses pitch sharpening								    */
/*          *  - uses gain_trans										*/
/*----------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS :												*/
/*    const Word32  core_brate, : core bitrate            Q0            */
/* _ (Word16) L_frame_fx		: length of the frame		Q0			*/
/*----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :												    */
/* _ (Word16[]) exc_fx			: adapt. excitation exc (Q0)		    */
/* _ (Word16[]) exc2_fx			: adapt. excitation/total exc (Q0)	    */
/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													*/
/* _ None																*/
/*======================================================================*/
static void tc_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word16 L_frame,         /* i  : length of the frame                         */
    Word16 exc[],           /* o  : glottal codebook contribution               */
    Word16 *T0,             /* o  : close-loop pitch period                     */
    Word16 *T0_frac,        /* o  : close-loop pitch period - fractional part   */
    const Word16 i_subfr,         /* i  : subframe index                              */
    const Word16 tc_subfr,        /* i  : TC subframe index                           */
    Word16 *position,       /* o  : first glottal impulse position in frame     */
    const Word32 core_brate,      /* i  : core bitrate                                */
    Word16 bwe_exc[],       /* o  : excitation for SWB TBE                      */
    Word16 *Q_exc           /*i/o : scaling of excitation                       */
)
{
    Word16 i, imp_shape, imp_pos, imp_sign, imp_gain, nBits;
    Word16 gain_trans,temp;
    const Word16 *pt_shape;
    Word16 j, sc;
    Word16 tempS;
    Word16 index;
    /*----------------------------------------------------------------*
     * find the number of bits
     *----------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
        move16();
    }

    /*----------------------------------------------------------------*
     * decode parameter T0 (pitch period)
     *----------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF ( ((i_subfr == 0) && ((tc_subfr == 0) || (sub(tc_subfr,TC_0_64) == 0) || (sub(tc_subfr,TC_0_128) == 0) || (sub(tc_subfr,TC_0_192) == 0) )) || (sub(tc_subfr,L_SUBFR) == 0) )
        {
            *T0 = L_SUBFR;
            move16();
            *T0_frac = 0;
            move16();
        }
        ELSE IF( (tc_subfr == 3*L_SUBFR) )
        {
            i = (Word16)get_next_indice_fx( st_fx, nBits );

            IF( sub(nBits,9) == 0 )
            {
                abs_pit_dec_fx( 4, i, 0, T0, T0_frac );
            }
            ELSE
            {
                abs_pit_dec_fx( 2, i, 0, T0, T0_frac );
            }
        }
        ELSE
        {
            i = (Word16)get_next_indice_fx( st_fx, nBits );
            move16();

            IF( i == 0 )
            {
                *T0 = L_SUBFR;
                move16();
                *T0_frac = 0;
                move16();
            }
            ELSE
            {
                IF( sub(tc_subfr,TC_0_0) == 0 )
                {
                    IF( sub(nBits,6) == 0 )
                    {
                        delta_pit_dec_fx( 2, i, T0, T0_frac, PIT_MIN-1 );
                    }
                    ELSE
                    {
                        delta_pit_dec_fx( 0, i, T0, T0_frac, PIT_MIN-1 );
                    }
                }
                ELSE
                {
                    delta_pit_dec_fx( 0, i, T0, T0_frac, PIT_MIN-1 );
                }
            }
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        i = (Word16)get_next_indice_fx( st_fx, nBits );
        move16();

        IF( sub(nBits,10) == 0 )
        {
            IF( sub(i,(PIT16k_FR2_EXTEND_10b-PIT16k_MIN_EXTEND)*4) < 0 )
            {
                *T0 = add(PIT16k_MIN_EXTEND,shr(i,2));
                move16();
                temp = shl(sub(*T0,PIT16k_MIN_EXTEND),2);
                *T0_frac = sub(i,temp);
                move16();
            }
            ELSE
            {
                index =  sub(i,shl(sub(PIT16k_FR2_EXTEND_10b,PIT16k_MIN_EXTEND),2));
                *T0 = add(PIT16k_FR2_EXTEND_10b,shr(index,1));
                *T0_frac = sub(index ,shl(sub(*T0,PIT16k_FR2_EXTEND_10b),1));
                (*T0_frac) = shl(*T0_frac,1);
            }
        }
        ELSE IF( sub(nBits,6) == 0)
        {
            *T0 = add(PIT16k_MIN ,shr(i,1));
            move16();
            *T0_frac = sub(i,shl(sub(*T0,PIT16k_MIN),1));
            *T0_frac = shl(*T0_frac,1);
            move16();
        }
        ELSE IF( sub(nBits,5) == 0 )
        {
            *T0 = add(PIT16k_MIN,i);
            move16();
            *T0_frac = 0;
            move16();
            move16();
        }
        ELSE
        {
            *T0 = L_SUBFR;
            move16();
            *T0_frac = 0;
            move16();
        }
    }

    /*----------------------------------------------------------------*
     * decode other TC parameters
     *----------------------------------------------------------------*/

    imp_shape = (Word16)get_next_indice_fx( st_fx, 3 );
    move16();
    imp_pos = (Word16)get_next_indice_fx( st_fx, 6 );
    move16();
    imp_sign = (Word16)get_next_indice_fx( st_fx, 1 );
    move16();
    imp_gain = (Word16)get_next_indice_fx( st_fx, 3 );
    move16();

    /*----------------------------------------------------------------*
    * - restore gain_trans
    * - build glottal codebook contribution
    *----------------------------------------------------------------*/

    gain_trans = tbl_gain_trans_tc_fx[imp_gain];
    move16();
    test();
    if( imp_sign == 0 )
    {
        gain_trans = negate(gain_trans);
    }

    /* local max for scaling need */
    /* maximum of impulse always takes 15 bits (except impulse #7 wich is using 14 bits) */

    tempS = 4;
    move16();
    test();
    if (sub(imp_gain, 3) <= 0)
    {
        tempS = 7;
        move16();
    }

    /* build glottal codebook contribution */
    sc = add(13-15, tempS); /* scaling of curent exc */
    sc = sub(*Q_exc, sc);

    pt_shape = &Glottal_cdbk_fx[add(sub(i_mult2(imp_shape, L_IMPULSE), imp_pos), L_IMPULSE2)];
    move16();

    j = s_max(0, sub(imp_pos, L_IMPULSE2));
    move16();       /* penalty for exc + i_subfr initialisation */
    FOR (i = 0; i < j; i++)
    {
        exc[i+i_subfr] = 0;
        move16();
    }
    j = s_min(L_SUBFR, add(imp_pos, L_IMPULSE2));
    FOR (; i <= j; i++)
    {
        exc[i+i_subfr] = round_fx(L_shl(L_mult(pt_shape[i], gain_trans), sc)); /* (Qx * Q14 ) */
    }
    FOR (; i < L_SUBFR; i++)
    {
        exc[i+i_subfr] = 0;
        move16();
    }

    /*--------------------------------------------------------------*
     * adapt. search of the second impulse in the same subframe
     * (when appears)
     *--------------------------------------------------------------*/

    pred_lt4_tc_fx( exc, *T0, *T0_frac, inter4_2_fx, imp_pos, i_subfr );

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        interp_code_5over2_fx(&exc[i_subfr], &bwe_exc[i_subfr * HIBND_ACB_L_FAC], L_SUBFR);
    }
    ELSE
    {
        interp_code_4over2_fx(&exc[i_subfr], &bwe_exc[i_subfr * 2], L_SUBFR);
    }

    *position = add(imp_pos, i_subfr);
    move16();
    return;
}

/*-------------------------------------------------------------------*
 * tc_classif_fx()
 *
 * TC subframe classification decoding
 *-------------------------------------------------------------------*/

Word16 tc_classif_fx(
    Decoder_State_fx *st_fx,        /* i/o: decoder state structure     */
    const Word16 L_frame        /* i  : length of the frame         */
)
{
    Word16 tc_subfr, indice;

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        if ( get_next_indice_fx( st_fx, 1 ) )
        {
            tc_subfr = TC_0_0;
            move16();
        }
        ELSE
        {
            IF ( get_next_indice_fx( st_fx, 1 ) )
            {
                tc_subfr = 0;
                move16();

                IF ( get_next_indice_fx( st_fx, 1 ) )
                {
                    tc_subfr = TC_0_192;
                    move16();
                }
                ELSE
                {
                    tc_subfr = TC_0_128;
                    move16();
                    if ( get_next_indice_fx( st_fx, 1 ) )
                    {
                        tc_subfr = TC_0_64;
                        move16();
                    }
                }
            }
            ELSE
            {
                IF ( get_next_indice_fx( st_fx, 1 ) )
                {
                    tc_subfr = L_SUBFR;
                    move16();
                }
                ELSE
                {
                    tc_subfr = 3*L_SUBFR;
                    move16();
                    if ( get_next_indice_fx( st_fx, 1 ) )
                    {
                        tc_subfr = 2*L_SUBFR;
                        move16();
                    }
                }
            }
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        indice = (Word16) get_next_indice_fx( st_fx, 2 );

        IF( sub(indice,3) < 0 )
        {
            tc_subfr = shl(indice, 6);
        }
        ELSE
        {
            tc_subfr = 4*L_SUBFR;
            move16();
            if( get_next_indice_fx( st_fx, 1 ) == 0 )
            {
                tc_subfr = 3*L_SUBFR;
                move16();
            }
        }
    }

    return( tc_subfr );
}
