/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-----------------------------------------------------------------*
   * Local functions
   *-----------------------------------------------------------------*/
static void gain_trans_enc_fx(Word32 gain_trans32,  Word16 exc[], Word16 *quant_index, Word16 *quant_sign, Word16 Q_new);
static void tc_enc_fx(Encoder_State_fx *st_fx, const Word32  core_brate, const Word16 L_frame, const Word16 i_subfr, Word16 *tc_subfr, Word16 *position,
                      const Word16 *h1_fx, const Word16 *xn_fx, Word16 *exc_fx, Word16 *yy1_fx, Word16 *T0_min, Word16 *T0_max,
                      Word16 *T0, Word16 *T0_frac, Word16 *gain_pit_fx, Word16 g_corr_fx[], Word16 *bwe_exc_fx, Word16 Q_new);


/*==========================================================================*/
/* FUNCTION   : void transition_enc_fx ()								    */
/*--------------------------------------------------------------------------*/
/* PURPOSE    :Principal function for adaptive excitation construction in TC*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/*   (Word32) core_brate		: core bitrate							Q0	*/
/*	 (Word16)  L_frame			: length of the frame    				Q0	*/
/*	 (Word16)  coder_type		: coding type          					Q0	*/
/*	 (Word16)  bwidth    		: input signal bandwidth				Q0	*/
/*	 (Word16)  i_subfr			: subrame index  						Q0	*/
/*	 (Word16*) Jopt_flag		: joint optimization flag				Q0	*/
/*	 (Word16[]) voicing_fx		: normalized correlations(from OL pitch)Q15	*/
/*	 (Word16[]) T_op_fx			: open loop pitch estimates in current frameQ0*/
/*	 (Word16*)  res_fx			: pointer to the LP residual signal frameQ_new*/
/*	 (Word16*) h1_fx			: weighted filter input response		Q14	*/
/*	 (Word16*) xn_fx			: target signal  						Q_new*/
/*	 (Word16)  Q_new			: input scaling	     			            */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/*	 (Word16*) y1_fx            : zero-memory filtered adaptive excitation Q12*/
/*	 (Word16*) xn2_fx	        : target vector for innovation search  Qnew */
/*	 (Word16*) gain_pit_fx	    : pitch gain  (0..GAIN_PIT_MAX)			 Q14*/
/*	 (Word16[])g_corr_fx	    : correlations <y1,y1>  and -2<xn,y1>  	    */
/*	 (Word16**)pt_pitch_fx	    : floating pitch values  	             Q6 */
/*	 (Word16*) bwe_exc_fx		: excitation for SWB TBE 			     Q0 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*	 (Word16*) tc_subfr		    : TC subframe index					    Q0	*/
/*	 (Word16*) position		    : index of the residual signal maximum 	Q0  */
/*	 (Word16*) T0_min   		: lower pitch limit					    Q0  */
/*	 (Word16*) T0_max		    : higher pitch limit					Q0  */
/*	 (Word16*) T0				: close loop integer pitch				Q0  */
/*	 (Word16*) T0_frac		    : close loop fractional part of the pitch Q0*/
/*	 (Word16*) exc_fx           : pointer to excitation signal frame 		*/
/*	 (Word16*) gp_cl_fx         : memory of gain of pitch clipping algorithm*/
/*	 (Word16*) clip_gain        : adaptive gain clipping flag			Q0	*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
void transition_enc_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure */
    const Word32  core_brate,     /* i  : core bitrate                                */
    const Word16 L_frame,        /* i  : length of the frame                         */
    const Word16 coder_type,     /* i  : coding type                                 */
    const Word16 i_subfr,        /* i  : subframe index                              */
    Word16 *tc_subfr,      /* i/o: TC subframe index                           */
    Word16 *Jopt_flag,     /* i  : joint optimization flag                     */
    Word16 *position,      /* i/o: maximum of residual signal index            */
    const Word16 voicing_fx[],      /* i  : normalized correlations (from OL pitch)     Q15*/
    const Word16 T_op_fx[],         /* i  : open loop pitch estimates in current frame  Q0*/
    Word16 *T0,            /* i/o: close loop integer pitch                    Q0*/
    Word16 *T0_frac,       /* i/o: close loop fractional part of the pitch     Q0*/
    Word16 *T0_min,        /* i/o: lower limit for close-loop search           Q0*/
    Word16 *T0_max,        /* i/o: higher limit for close-loop search          Q0*/
    Word16 *exc_fx,           /* i/o: pointer to excitation signal frame          Q_new*/
    Word16 *y1_fx,            /* o  : zero-memory filtered adaptive excitation    Q_new-1+shift*/
    const Word16 *res_fx,           /* i  : pointer to the LP residual signal frame     Q_new*/
    const Word16 *h1_fx,            /* i  : weighted filter input response              Q(14+shift)*/
    const Word16 *xn_fx,            /* i  : target vector                               Q_new-1+shift*/
    Word16 *xn2_fx,           /* o  : target vector for innovation search         Q_new-1+shift*/
    Word16 *gp_cl_fx,         /* i/o: memory of gain of pitch clipping algorithm  */
    Word16 *gain_pit_fx,      /* o  : adaptive excitation gain                    Q14*/
    Word16 *g_corr_fx,        /* o  : ACELP correlation values                    */
    Word16 *clip_gain,     /* i/o: adaptive gain clipping flag                 */
    Word16 **pt_pitch_fx,     /* o  : floating pitch values                       */
    Word16 *bwe_exc_fx,        /* o  : excitation for SWB TBE                      Q_new*/
    Word16 Q_new,           /* i  : Current scaling */
    Word16 shift            /* i  : downscaling needs for 12 bits convolutions */

)
{
    Word16 pit_flag, pit_start, pit_limit, index, nBits;
    Word16 tmp, tmp1;
    Word32 i, offset,temp;
    Word16 shift_wsp;
    Word16 limit_flag, mult_Top, lp_select, lp_flag;

    /* set limit_flag to 0 for restrained limits, and 1 for extended limits */
    limit_flag = 0;
    move16();

    pit_start = PIT_MIN;
    move16();
    shift_wsp = add(Q_new,shift);

    /*-----------------------------------------------------------------*
     * TC: subrame determination for glottal shape search
     * -------------------------------------------------------
     * tc_subfr == 0         - TC in 1st subframe
     * tc_subfr == TC_0_0    - TC in 1st subframe + information about T0
     * tc_subfr == L_SUBFR   - TC in 2nd subframe
     * tc_subfr == 2*L_SUBFR - TC in 3rd subframe
     * tc_subfr == 3*L_SUBFR - TC in 4th subframe
     *-----------------------------------------------------------------*/

    IF( i_subfr == 0 )
    {
        IF( sub(*tc_subfr,3*L_SUBFR) == 0 )
        {
            IF( sub(L_frame,L_FRAME) == 0 )
            {
                test();
                move16();
                *position = emaximum_fx( Q_new, res_fx + 3*L_SUBFR,(sub(add(T_op_fx[0],2),L_SUBFR)<0? add(T_op_fx[0],2): L_SUBFR), &temp ) + 3*L_SUBFR;
                move16();
                *tc_subfr = 3*L_SUBFR;
                move16();
            }
            ELSE /* L_frame == L_FRAME16k */
            {
                test();
                move16();
                *position = emaximum_fx( Q_new, res_fx + 4*L_SUBFR,(sub(add(T_op_fx[0],2),L_SUBFR)<0? add(T_op_fx[0],2): L_SUBFR), &temp ) + 4*L_SUBFR;
                move16();
                *tc_subfr = 4*L_SUBFR;
                move16();
            }
        }
        ELSE
        {
            *position = emaximum_fx( Q_new, res_fx, add(T_op_fx[0],2), &temp );

            /* correction in case of possibly wrong T_op (double-pitch values) */
            test();
            test();
            test();
            IF( (sub(L_frame,L_FRAME)    == 0 && sub(T_op_fx[0],2*PIT_MIN)>0   ) ||
            (sub(L_frame,L_FRAME16k) == 0 && sub(T_op_fx[0],2*PIT16k_MIN) >0)
              )
            {
                Word16 position_tmp, len, exp_aver=0, exp=0, exp2=0;
                Word32 aver, temp2, L_sum, L_temp1, L_temp2;

                len = add(shr(T_op_fx[0],1), 2);
                position_tmp = emaximum_fx( Q_new, res_fx, len, &temp2 );

                L_sum = L_mac(1L, res_fx[0], res_fx[0]);
                FOR (i = 1; i < len; i++)
                {
                    L_sum = L_mac0(L_sum, res_fx[i], res_fx[i]);
                }
                aver = L_sum; /*Q = 2*Q_new */
                aver = root_a_over_b_fx(aver, 2*Q_new, L_shl(len,15), 15, &exp_aver); /*Q = 31-exp_aver*/

                temp = root_a_fx(temp, 0, &exp);    /* Q=31-exp */
                temp2 = root_a_fx(temp2, 0, &exp2); /* Q=31-exp2 */

                L_temp2 = Mult_32_16(temp, 26214);  /* Q=31-exp */
                L_temp1 = Mult_32_16(temp, 8192);   /* Q=31-exp */

                test();
                IF( L_sub(temp2, L_shl(L_temp2,(31-exp2)-(31-exp))) > 0     &&
                    L_sub(aver,  L_shl(L_temp1,(31-exp_aver)-(31-exp))) < 0  )
                {
                    *position = position_tmp;
                }
            }
            *tc_subfr = s_and(*position, 0x7FC0);
        }
        mult_Top = 1;
        IF( limit_flag == 0 )
        {
            test();
            IF( sub(L_frame,L_FRAME) == 0 && sub(T_op_fx[1],PIT_MIN) < 0 )
            {
                mult_Top = 2;
                move16();
            }
            test();
            if( sub(L_frame,L_FRAME16k) == 0 && sub(T_op_fx[1],PIT16k_MIN) < 0 )
            {
                mult_Top = 2;
                move16();
            }
        }

        limit_T0_fx( L_frame, 8, 0, limit_flag, mult_Top*T_op_fx[1], 0, T0_min, T0_max );

    }
    /*-----------------------------------------------------------------*
     * zero adaptive excitation signal construction
     *-----------------------------------------------------------------*/
    IF ( sub(*tc_subfr,i_subfr) > 0 )
    {
        *gain_pit_fx = 0;
        move16();
        *clip_gain = 0;
        move16();
        g_corr_fx[0] = 16384;
        move16();
        g_corr_fx[1] = add(shl(sub(shift_wsp,1),1),1);
        move16();
        g_corr_fx[2] = -16384;
        move16();
        g_corr_fx[3] = shl(sub(shift_wsp,1),1);

        set16_fx(&exc_fx[i_subfr], 0, L_SUBFR);  /* set excitation for current subrame to 0 */

        IF( sub(L_frame,L_FRAME) == 0 )
        {
            set16_fx(&bwe_exc_fx[i_subfr*HIBND_ACB_L_FAC], 0, (Word16)(L_SUBFR*HIBND_ACB_L_FAC));         /* set past excitation buffer to 0 */
        }
        ELSE
        {
            set16_fx(&bwe_exc_fx[i_subfr*2], 0, L_SUBFR*2);         /* set past excitation buffer to 0 */
        }

        set16_fx(y1_fx, 0, L_SUBFR);             /* set filtered adaptive excitation to 0 */
        Copy(xn_fx, xn2_fx, L_SUBFR);           /* target vector for codebook search */
        *T0 = L_SUBFR;
        move16();
        *T0_frac = 0;
        move16();

        **pt_pitch_fx = shl(add(shl(*T0,2),*T0_frac),4);
        move16();  /* save subframe pitch values Q6 */

    }

    /*-----------------------------------------------------------------*
     * glottal codebook contribution construction
     *-----------------------------------------------------------------*/
    ELSE IF ( sub(*tc_subfr,i_subfr) == 0 )
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            set16_fx( bwe_exc_fx-PIT_MAX*HIBND_ACB_L_FAC, 0, PIT_MAX*HIBND_ACB_L_FAC);         /* set past excitation buffer to 0 */
        }
        ELSE
        {
            set16_fx( bwe_exc_fx-PIT16k_MAX*2, 0, PIT16k_MAX*2);         /* set past excitation buffer to 0 */
        }

        tc_enc_fx( st_fx, core_brate, L_frame, i_subfr, tc_subfr, position, h1_fx, xn_fx, exc_fx,
                   y1_fx, T0_min, T0_max, T0, T0_frac, gain_pit_fx, g_corr_fx, bwe_exc_fx,Q_new);

        *clip_gain = gp_clip_fx( voicing_fx, i_subfr, coder_type, xn_fx, gp_cl_fx, sub(shift_wsp,1) );
        updt_tar_fx( xn_fx, xn2_fx, y1_fx, *gain_pit_fx, L_SUBFR );

        **pt_pitch_fx = shl(add(shl(*T0,2),*T0_frac),4);
        move16();
        *Jopt_flag = 1;
        move16();
    }

    /*--------------------------------------------------------------*
     * other subframes -> GENERIC encoding type,
     * standard adaptive excitation contribution
     * - exemption only in case when first glottal impulse is
     * in the 1st subframe and the second one in 2nd subframe
     * and later
     *--------------------------------------------------------------*/
    ELSE IF ( sub(*tc_subfr,i_subfr) < 0)
    {
        IF( sub(L_frame,L_FRAME) == 0 )
        {
            *Jopt_flag = 1;
            move16();
            /* pit_flag for T0 bits number coding determination */
            test();
            IF( (sub(sub(i_subfr,*tc_subfr),L_SUBFR) == 0) || (sub(sub(i_subfr, *tc_subfr),L_SUBFR-TC_0_0) == 0) )
            {
                pit_flag = 0;
                move16();
            }
            ELSE
            {
                pit_flag = L_SUBFR;
                move16();
            }

            IF( sub(*tc_subfr,TC_0_0) == 0 )
            {
                IF( sub(i_subfr,L_SUBFR) == 0 )
                {
                    limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max );
                }
                pit_flag = 1;
                move16();
            }

            /*----------------------------------------------------------*
             * if tc_subfr==0, change tc_subfr corresponding to the
             * second glot. impulse position
             *----------------------------------------------------------*/
            test();
            IF( (*tc_subfr == 0) && (sub(i_subfr,L_SUBFR) == 0) )
            {
                IF( sub(PIT_MIN,(*position)) > 0 )
                {
                    pit_start = sub(L_SUBFR, (*position));
                }
                ELSE
                {
                    pit_start = PIT_MIN;
                    move16();
                }
                pit_start  = s_max(pit_start, PIT_MIN);

                pit_limit = add(shl(pit_start,1), *position);

                /* Find the closed loop pitch period */

                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, pit_start, pit_limit, L_FRAME, L_SUBFR );

                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);


                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC - offset];
                }


                test();
                IF( sub((*T0),sub(2*L_SUBFR,(*position)))> 0 )
                {
                    IF( sub(add((*T0), (*position)),3*L_SUBFR) >= 0)
                    {
                        /* second glottal impulse is in the 4th subframe */
                        *tc_subfr = TC_0_192;
                        move16();
                    }
                    ELSE
                    {
                        /* second glottal impulse is in the 3rd subframe */
                        *tc_subfr = TC_0_128;
                        move16();
                    }
                }
                ELSE IF( (*tc_subfr == 0) && (sub(i_subfr,L_SUBFR) == 0) )
                {
                    /* second glottal impulse is in the 2nd subframe */
                    *tc_subfr = TC_0_64;
                    move16();
                }
            }

            /*-----------------------------------------------------------------*
             * get number of bits for pitch encoding
             *-----------------------------------------------------------------*/

            nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_fx(*tc_subfr))];
            move16();

            /*-----------------------------------------------------------------*
             * Find adaptive part of excitation, encode pitch period
             *-----------------------------------------------------------------*/
            test();
            test();
            test();
            test();
            test();
            test();
            /* first glottal impulse is in the 1st subrame */
            IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_128) >= 0) )
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
                set16_fx( &exc_fx[i_subfr], 0, (Word16)(L_SUBFR+1) );
                set16_fx( &bwe_exc_fx[i_subfr*HIBND_ACB_L_FAC], 0, (Word16)(L_SUBFR*HIBND_ACB_L_FAC) );

            }
            ELSE IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_64) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 2nd subframe,
                 * - build exc[] in 2nd subframe

                *--------------------------------------------------------*/
                IF( sub(add(*T0,*position), L_SUBFR ) < 0 )
                {
                    /* impulse must be in the 2nd subframe (not in 1st) */
                    *T0 = sub(L_SUBFR, (*position));
                    move16();
                    *T0_frac = 0;
                    move16();
                }
                IF( sub(add(*T0,*position),2*L_SUBFR) >= 0 )
                {
                    /* impulse must be in the 2nd subframe (not in 3rd) */
                    *T0 = sub(2*L_SUBFR-1,(*position));
                    move16();
                    *T0_frac = 2;
                    move16();
                }

                limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max ); /* find T0_min and T0_max for delta search */

                /* 7bit ENCODER */
                /*  index = (*T0-pit_start)*2 + *T0_frac/2;*/
                index = add(shl(sub(*T0,pit_start),1), shr(*T0_frac,1));
                push_indice_fx( st_fx, IND_PITCH, index, nBits );

                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);


                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);

                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC - offset];

                }
            }
            ELSE IF( (sub(i_subfr,2*L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_128) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 3rd subframe
                 * - build exc[] in 3rd subframe
                 *--------------------------------------------------------*/

                pit_start = sub(2*L_SUBFR, (*position));
                pit_flag = 0;
                move16();

                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, pit_start, 3*L_SUBFR, L_FRAME, L_SUBFR );

                IF( sub(add((*T0),(*position)),2*L_SUBFR) < 0 )
                {
                    /* impulse must be in the 3rd subframe (not in 2nd) */
                    *T0 = sub(2*L_SUBFR, (*position));
                    move16();
                    *T0_frac = 0;
                    move16();
                }

                IF (sub(add((*T0),(*position)),3*L_SUBFR) >= 0)
                {
                    /* impulse must be in the 3rd subframe (not in 4th) */
                    *T0 = sub(3*L_SUBFR - 1, (*position));
                    move16();
                    *T0_frac = 2;
                    move16();
                }

                limit_T0_fx( L_FRAME, 8, pit_flag, limit_flag, *T0, 0, T0_min, T0_max ); /* find T0_min and T0_max for delta search */

                index = add(shl(sub(*T0, pit_start), 1), shr(*T0_frac,1));
                push_indice_fx( st_fx, IND_PITCH, index, nBits );

                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);


                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC - offset];
                }
            }
            ELSE IF( (sub(i_subfr,2*L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_192) == 0) )
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
                set16_fx( &exc_fx[i_subfr], 0, (Word16)(L_SUBFR+1) );
                set16_fx( &bwe_exc_fx[i_subfr*HIBND_ACB_L_FAC], 0, (Word16)(L_SUBFR*HIBND_ACB_L_FAC) );


            }
            ELSE IF( (sub(i_subfr,3*L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_192) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse is in the 4th subframe
                 * - build exc[] in 4th subframe
                 *--------------------------------------------------------*/
                /* always T0_frac = 0 */
                pit_flag = 0;
                move16();

                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );

                IF( sub(add(*T0, *position), 3*L_SUBFR) < 0 )
                {
                    /* impulse must be in the 4th subframe (not in 3rd) */
                    *T0 = sub(3*L_SUBFR, (*position));
                    move16();
                    *T0_frac = 0;
                    move16();
                }

                pit_start = sub(3*L_SUBFR, (*position));
                pit_limit = sub(2*L_FRAME - PIT_MAX -2, shl(*position,1));

                IF (sub((*T0),pit_limit) < 0)
                {
                    index = add(shl(sub(*T0, pit_start),1), shr(*T0_frac,1));
                }
                ELSE
                {
                    index = add(sub(*T0, pit_limit), shl(sub(pit_limit, pit_start),1));
                    *T0_frac = 0;
                    move16();
                }
                push_indice_fx( st_fx, IND_PITCH, index, nBits );

                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);

                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC - offset	];
                    move16();
                }
            }
            ELSE IF( (sub(i_subfr,3*L_SUBFR) == 0) && (sub(*tc_subfr,TC_0_128) == 0) )
            {
                /*--------------------------------------------------------*
                 * second glottal impulse in the 3rd subframe
                 * build exc[] in 4th subframe
                 *--------------------------------------------------------*/
                pit_flag = L_SUBFR;
                move16();
                IF( sub(nBits,4) == 0)
                {

                    *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, PIT_MIN, PIT_MIN, L_FRAME, L_SUBFR );

                    index = delta_pit_enc_fx( 0, *T0, 0, *T0_min );

                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
                    index = delta_pit_enc_fx( 2, *T0, *T0_frac, *T0_min );
                }
                push_indice_fx( st_fx, IND_PITCH, index, nBits );

                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);


                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC- offset];
                }

            }

            /*------------------------------------------------------------*
             * first glottal impulse is NOT in the 1st subframe,
             * or two impulses are in the 1st subframe
             *------------------------------------------------------------*/
            ELSE
            {
                test();
                IF( sub(nBits,8) == 0 || sub(nBits,5) == 0)
                {
                    test();
                    IF( !((*tc_subfr == 0) && (sub(i_subfr,L_SUBFR) == 0)) )
                    {
                        *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, PIT_MIN, PIT_FR1_8b, L_FRAME, L_SUBFR );
                    }
                }
                ELSE
                {
                    *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, pit_flag, limit_flag, PIT_FR2_9b, PIT_FR1_9b, L_FRAME, L_SUBFR );
                }
                pit_Q_enc_fx( st_fx, 0, nBits, 8, pit_flag, limit_flag, *T0, *T0_frac, T0_min, T0_max
                            );

                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);

                offset = tbe_celp_exc_offset(*T0, *T0_frac, L_frame);

                FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
                {
                    bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr * HIBND_ACB_L_FAC - offset];
                }

            }

            /*-----------------------------------------------------------------*
             * - gain clipping test to avoid unstable synthesis
             * - LP filtering of the adaptive excitation (if non-zero)
             * - codebook target computation
             *-----------------------------------------------------------------*/
            IF( *Jopt_flag == 0 )
            {
                /* adaptive/TC excitation is zero */
                Copy( xn_fx, xn2_fx, L_SUBFR );
                g_corr_fx[0] = 0;
                move16();
                g_corr_fx[1] = 0;
                move16();
                g_corr_fx[2] = 0;
                move16();
                g_corr_fx[3] = 0;
                move16();
                *clip_gain = 0;
                move16();
            }
            ELSE
            {
                *clip_gain = gp_clip_fx( voicing_fx, i_subfr, coder_type, xn_fx, gp_cl_fx, (Q_new+shift-1)  );

                lp_select = lp_filt_exc_enc_fx( MODE1, core_brate, 0, coder_type, i_subfr, exc_fx, h1_fx,
                xn_fx, y1_fx, xn2_fx, L_SUBFR, L_frame, g_corr_fx, *clip_gain, gain_pit_fx, &lp_flag );

                IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
                {
                    push_indice_fx( st_fx, IND_LP_FILT_SELECT, lp_select, 1 );
                }
            }

            **pt_pitch_fx = shl(add(shl(*T0,2),*T0_frac),4);
            move16();
            /*---------------------------------------------------------------------*
             * fill the pitch buffer - needed for post-processing
             *---------------------------------------------------------------------*/
            test();
            test();
            test();
            test();
            test();
            IF( (*tc_subfr >= 2*L_SUBFR) && (i_subfr == 3*L_SUBFR) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch_fx) -= 3;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
            }
            ELSE IF( (*tc_subfr == L_SUBFR) && (i_subfr == 2*L_SUBFR) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch_fx) -= 2;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
            }
            ELSE IF( (*tc_subfr == TC_0_64) && (i_subfr == L_SUBFR) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch_fx) -= 1;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
            }
            ELSE IF( (*tc_subfr == TC_0_128) && (i_subfr == 2*L_SUBFR) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch_fx) -= 2;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
            }
            ELSE IF( (*tc_subfr == TC_0_192) && (i_subfr == 3*L_SUBFR) )
            {
                tmp = shl(add(shl(*T0,2),*T0_frac),4);
                (*pt_pitch_fx) -= 3;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
                **pt_pitch_fx = tmp;
                move16();
                (*pt_pitch_fx)++;
                move16();
            }
        }
        ELSE  /* L_frame == L_FRAME16k */
        {
            if( sub(i_subfr,2*L_SUBFR) >= 0)
            {
                limit_flag = 1;
                move16();
            }
            IF( sub(i_subfr,2*L_SUBFR) <= 0)
            {
                IF( sub(i_subfr,2*L_SUBFR) < 0 )
                {
                    mult_Top = 1;
                    move16();
                    if( sub(T_op_fx[0],PIT16k_MIN) < 0 )
                    {
                        mult_Top = 2;
                        move16();
                    }

                    limit_T0_fx( L_FRAME16k, 8, 0, limit_flag, mult_Top*T_op_fx[0], 0, T0_min, T0_max );  /* TC0 second subfr. */

                }
                ELSE
                {
                    limit_T0_fx( L_FRAME16k, 8, 0, limit_flag, T_op_fx[1], 0, T0_min, T0_max );  /* TC0 third subfr., or TC64 third subfr. */
                }
            }

            /*-----------------------------------------------------------------*
             * get number of bits for pitch encoding
             *-----------------------------------------------------------------*/

            nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(*tc_subfr))];
            move16();

            /*-----------------------------------------------------------------*
             * Find adaptive part of excitation, encode pitch period
             *-----------------------------------------------------------------*/

            IF( sub(nBits,10) == 0 )
            {
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, limit_flag, PIT16k_FR2_EXTEND_10b, PIT16k_MAX, L_frame, L_SUBFR );
                pit16k_Q_enc_fx( st_fx, nBits, limit_flag, *T0, *T0_frac, T0_min, T0_max );
            }
            ELSE IF( sub(nBits,9) == 0 )
            {
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, limit_flag, PIT16k_FR2_EXTEND_9b, PIT16k_FR1_EXTEND_9b, L_frame, L_SUBFR );
                pit16k_Q_enc_fx( st_fx, nBits, limit_flag, *T0, *T0_frac, T0_min, T0_max );
            }
            ELSE IF( sub(nBits,8) == 0 )     /* tc_subfr==0 && i_subfr==L_SUBFR */
            {
                /*-----------------------------------------------------------------------------*
                 * The pitch range is encoded absolutely with 8 bits and is divided as follows:
                 *   PIT16k_MIN  to PIT16k_FR2_TC0_2SUBFR-1 resolution 1/4 (frac = 0,1,2 or 3)
                 *   PIT16k_FR2_TC0_2SUBFR to 2*L_SUBFR     resolution 1/2 (frac = 0 or 2)
                 *-----------------------------------------------------------------------------*/
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, limit_flag, PIT16k_FR2_TC0_2SUBFR, 2*L_SUBFR, L_frame, L_SUBFR );

                IF( sub(*T0_max,2*L_SUBFR) > 0)
                {
                    *T0 = 2*L_SUBFR;
                    move16();
                    *T0_frac = 0;
                    move16();
                }

                IF( sub(*T0,PIT16k_FR2_TC0_2SUBFR ) < 0)
                {
                    /*index = (*T0)*4 + (*T0_frac) - (PIT16k_MIN*4);*/
                    index = add(shl(*T0,2), sub(*T0_frac, PIT16k_MIN*4));
                }
                ELSE
                {
                    /*index = (*T0)*2 + ((*T0_frac)>>1) - (PIT16k_FR2_TC0_2SUBFR*2) + ((PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4);*/
                    index = add(sub(add(shl(*T0,1), shr(*T0_frac,1)),  (PIT16k_FR2_TC0_2SUBFR*2)),  (PIT16k_FR2_TC0_2SUBFR-PIT16k_MIN)*4);
                }
                push_indice_fx( st_fx, IND_PITCH, index, nBits );
            }
            ELSE IF( sub(nBits,6) == 0 )
            {
                /* delta search */
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, L_SUBFR, limit_flag, PIT16k_FR2_EXTEND_9b, PIT16k_FR1_EXTEND_9b, L_frame, L_SUBFR );

                index = delta_pit_enc_fx( 4, *T0, *T0_frac, *T0_min );
                push_indice_fx( st_fx, IND_PITCH, index, nBits );
            }
            ELSE IF( sub(nBits,5) == 0 )
            {
                /* delta search */
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, L_SUBFR, limit_flag, PIT16k_MIN_EXTEND, PIT16k_FR1_EXTEND_9b, L_frame, L_SUBFR );
                index = delta_pit_enc_fx( 2, *T0, *T0_frac, *T0_min );

                push_indice_fx( st_fx, IND_PITCH, index, nBits );
            }

            test();
            IF( sub(nBits,6) == 0 || sub(nBits,5) == 0 )
            {
                limit_T0_fx( L_FRAME16k, 8, L_SUBFR, limit_flag, *T0, *T0_frac, T0_min, T0_max );
            }

            /*-----------------------------------------------------------------*
             * - gain clipping test to avoid unstable synthesis
             * - LP filtering of the adaptive excitation
             * - codebook target computation
             *-----------------------------------------------------------------*/
            test();
            IF( (sub(i_subfr,L_SUBFR) == 0) && (sub(*T0,2*L_SUBFR) == 0) )
            {
                *gain_pit_fx = 0;
                move16();
                *clip_gain = 0;
                move16();
                g_corr_fx[0] = 0;
                move16();
                g_corr_fx[1] = 0;
                move16();
                *Jopt_flag = 0;
                move16();

                set16_fx( &exc_fx[i_subfr], 0, L_SUBFR+1 );   /* set excitation for current subrame to 0 */
                push_indice_fx( st_fx, IND_LP_FILT_SELECT, 0, 1 );      /* this bit is actually not needed */

                Copy( xn_fx, xn2_fx, L_SUBFR );              /* target vector for codebook search */
                set16_fx( y1_fx, 0, L_SUBFR );                /* set filtered adaptive excitation to 0 */
                set16_fx( &bwe_exc_fx[i_subfr * 2], 0, L_SUBFR * 2 );
            }
            ELSE
            {
                /* Find the adaptive codebook vector - ACELP long-term prediction */
                pred_lt4(&exc_fx[i_subfr], &exc_fx[i_subfr], *T0, *T0_frac, L_SUBFR+1, pitch_inter4_2, L_INTERPOL2, PIT_UP_SAMP);
                offset = L_deposit_l(0);

                tmp = extract_l(L_mult(*T0_frac,32));/*Q8, 0.25 in Q7*/
                tmp = add(512,tmp);/*Q8; 2 in Q8*/
                tmp = mult_r(tmp,256);/*Q16->Q0; 2 in Q7*/

                tmp1 = sub(*T0,2);/*Q0*/
                tmp1 = shl(tmp1,1);/*Q0 */

                offset = add(tmp,tmp1);/*Q0*/
                FOR (i=0; i<L_SUBFR * 2; i++)
                {
                    /* bwe_exc_fx[i + i_subfr * 2] = bwe_exc_fx[i + i_subfr * 2 - *T0 * 2  - (int) ((float) *T0_frac * 0.5f + 4 + 0.5f) + 4];move16();*/
                    bwe_exc_fx[i + i_subfr * 2] = bwe_exc_fx[i + i_subfr * 2 - offset + 4];
                }

                *clip_gain = gp_clip_fx( voicing_fx, i_subfr, coder_type, xn_fx, gp_cl_fx,Q_new );

                lp_select = lp_filt_exc_enc_fx( MODE1, core_brate, 0, coder_type, i_subfr, exc_fx, h1_fx,
                xn_fx, y1_fx, xn2_fx, L_SUBFR, L_frame, g_corr_fx, *clip_gain, gain_pit_fx, &lp_flag );

                IF( sub(lp_flag,NORMAL_OPERATION) == 0 )
                {
                    push_indice_fx( st_fx, IND_LP_FILT_SELECT, lp_select, 1 );
                }

                *Jopt_flag = 1;
                move16();
            }

            /***pt_pitch = (float)(*T0) + (float)(*T0_frac)/4.0f;*/   /* save subframe pitch value  */
            /***pt_pitch_fx = shl(add(*T0,shr(*T0_frac,2)),4); move16();*/
            tmp = shl(add(shl(*T0,2),*T0_frac),4);
            **pt_pitch_fx = tmp;
            move16();

            /*---------------------------------------------------------------------*
             * fill the pitch buffer - needed for post-processing
             *---------------------------------------------------------------------*/
            test();
            test();
            IF( (sub(sub(i_subfr, *tc_subfr),L_SUBFR) == 0) || (*tc_subfr==0 && sub(i_subfr,2*L_SUBFR)==0) )
            {
                index = shr(i_subfr,6);
                (*pt_pitch_fx) -= index;
                move16();

                FOR( i=0; i<index; i++ )
                {
                    **pt_pitch_fx = tmp;
                    move16();
                    (*pt_pitch_fx)++;
                }
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------------------------------*
  * tc_enc()
  *
  * Principal function for transition coding (TC) in encoder.
  * Glottal codebook contribution part:
  *
  *           |----|             |----|                                      xn
  *  imp_pos->||   |  imp_shape->| g1 |                                       |
  *           | |  |             | g2 |     ----   exc  |---|  y1   ----      |
  *           |  | |-------------|    |----|gain|-------| h |------|gain|----(-)---> xn2
  *           |   ||             | gn |     ----        |---|       ----
  *           |----|             |----|
  *          codebook          excitation gain_trans    h_orig     gain_pit
  *
  *-------------------------------------------------------------------------------------------*/
static void tc_enc_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure */
    const Word32  core_brate,     /* i  : core bitrate                            */
    const Word16 L_frame,        /* i  : length of the frame                     */
    const Word16 i_subfr,        /* i  : subrame index                           */
    Word16 *tc_subfr,      /* i/o: TC subframe index                       */
    Word16 *position,      /* i/o: index of the residual signal maximum    */
    const Word16 *h1_fx,            /* i  : weighted filter input response          Q(14+shift)*/
    const Word16 *xn_fx,            /* i  : target signal                           Q_new-1+shift*/
    Word16 *exc_fx,           /* o  : glottal codebook contribution           Q_new*/
    Word16 *yy1_fx,            /* o  : filtered glottal codebook contribution  */
    Word16 *T0_min,        /* o  : lower pitch limit                       Q0*/
    Word16 *T0_max,        /* o  : higher pitch limit                      Q0*/
    Word16 *T0,            /* o  : close loop integer pitch                Q0*/
    Word16 *T0_frac,       /* o  : close loop fractional part of the pitch Q0*/
    Word16 *gain_pit_fx,      /* o  : pitch gain  (0..GAIN_PIT_MAX)           Q14*/
    Word16 g_corr_fx[],       /* o  : correlations <y1,y1>  and -2<xn,y1>     */
    Word16 *bwe_exc_fx,        /* i/o: excitation for SWB TBE               Q_new*/
    Word16 Q_new            /* i  : input scaling  */
)
{
    Word16 i,imp_shape, imp_pos, index, nBits, h1_tmp_fx[L_SUBFR];
    Word16 pitch_index,pitch_sign_fx;
    Word32 gain_trans32;

    imp_pos = sub(*position, i_subfr);
    FOR (i = 0; i < L_SUBFR; i++)
    {
        h1_tmp_fx[i] = h1_fx[i];
        move16();
    }
    /*-----------------------------------------------------------------*
     * get number of bits for pitch encoding
     *-----------------------------------------------------------------*/

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_fx(*tc_subfr))];
        move16();
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, TRANSITION, i_subfr, TC_SUBFR2IDX_16KHZ_fx(*tc_subfr))];
        move16();
    }

    /*--------------------------------------------------------------*
     * Closed loop pitch search
     *--------------------------------------------------------------*/

    *T0_frac = 0;
    move16();

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        test();
        IF( (sub(*T0_min,L_SUBFR) <= 0) || (sub(*tc_subfr,3*L_SUBFR) == 0) )
        {
            IF( sub(nBits,9) == 0 )
            {
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 0, PIT_FR2_9b, PIT_FR1_9b, L_FRAME, L_SUBFR );
            }
            ELSE IF( sub(nBits,6) == 0 )
            {
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 0, PIT_MIN, L_SUBFR, L_FRAME, L_SUBFR );
            }
            ELSE
            {
                *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 0, PIT_MAX, PIT_MIN, L_FRAME, L_SUBFR );
            }
        }
        ELSE
        {
            *T0 = L_SUBFR;
            move16();
        }
        test();
        if( sub(*tc_subfr,L_SUBFR) == 0 && sub(*T0,L_SUBFR) < 0 )
        {
            *T0 = L_SUBFR;
            move16();
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        IF( sub(nBits,10) == 0 )
        {
            *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 1, PIT16k_FR2_EXTEND_10b, PIT16k_MAX, L_FRAME16k, L_SUBFR );
        }
        ELSE IF( sub(nBits,6) == 0 )
        {
            /* T0_frac with 1/2 sample resolution */
            *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 0, PIT16k_MIN, L_SUBFR, L_FRAME16k, L_SUBFR );
            IF( *T0 > L_SUBFR )
            {
                *T0 = L_SUBFR;
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE IF( sub(nBits,5) == 0)
        {
            /* T0_frac with 1 sample resolution */
            *T0 = pitch_fr4_fx( &exc_fx[i_subfr], xn_fx, h1_fx, *T0_min, *T0_max, T0_frac, 0, 0, PIT16k_MIN, PIT16k_MIN, L_frame, L_SUBFR );

            IF( sub(*T0,L_SUBFR) > 0 )
            {
                *T0 = L_SUBFR;
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE
        {
            *T0 = L_SUBFR;
            move16();
        }
    }


    /* set tc_subfr to TC_0_0 */
    test();
    test();
    test();
    if( i_subfr == 0 && sub(L_frame,L_FRAME) == 0 && ( sub(*T0,L_SUBFR) < 0 || sub(*tc_subfr,3*L_SUBFR) == 0) )
    {
        *tc_subfr = TC_0_0;
        move16();
    }

    /*--------------------------------------------------------------*
     * Builds glottal codebook contribution
     *--------------------------------------------------------------*/

    set_impulse_fx(xn_fx, h1_tmp_fx, &exc_fx[i_subfr], yy1_fx, &imp_shape, &imp_pos, &gain_trans32, Q_new);/*chk h1_tmp_fx*/

    /*--------------------------------------------------------------*
     * quantize gain_trans and scale glottal codebook contribution
     *--------------------------------------------------------------*/

    gain_trans_enc_fx( gain_trans32, &exc_fx[i_subfr], &pitch_index, &pitch_sign_fx, Q_new );

    /* set past excitation buffer to zeros */
    set16_fx( exc_fx-L_EXC_MEM, 0, L_EXC_MEM );
    /*--------------------------------------------------------------*
     * adapt. search of the second impulse in the same subframe
     * (when appears)
     *--------------------------------------------------------------*/

    pred_lt4_tc_fx( exc_fx, *T0, *T0_frac, inter4_2_fx, imp_pos, i_subfr );

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        interp_code_5over2_fx(&exc_fx[i_subfr], &bwe_exc_fx[i_subfr * HIBND_ACB_L_FAC], L_SUBFR);
    }
    ELSE
    {
        interp_code_4over2_fx(&exc_fx[i_subfr], &bwe_exc_fx[i_subfr * 2], L_SUBFR);
    }

    /*--------------------------------------------------------------*
     * compute glottal-shape codebook excitation
     *--------------------------------------------------------------*/

    /* create filtered glottal codebook contribution */
    conv_fx( &exc_fx[i_subfr], h1_fx, yy1_fx, L_SUBFR );

    /* gain_pit computation */
    *gain_pit_fx = corr_xy1_fx( xn_fx, yy1_fx, g_corr_fx, L_SUBFR, 0 );

    /*--------------------------------------------------------------*
     * Encode parameters and write indices
     *--------------------------------------------------------------*/
    IF( sub(L_frame,L_FRAME) == 0 )
    {
        test();
        test();
        IF( ( (i_subfr != 0) || (sub(*tc_subfr,TC_0_0) == 0) )
            && (sub(*tc_subfr,L_SUBFR) != 0))
        {
            test();
            /* write pitch index */
            IF( (sub(*T0,L_SUBFR) >= 0) && (sub(*tc_subfr,3*L_SUBFR) != 0) )
            {
                push_indice_fx( st_fx, IND_PITCH, 0, nBits );
            }
            ELSE IF( sub(*tc_subfr,3*L_SUBFR) == 0)
            {
                IF( sub(nBits,9) == 0 )
                {
                    index = abs_pit_enc_fx( 4, 0, *T0, *T0_frac );
                }
                ELSE
                {
                    index = abs_pit_enc_fx( 2, 0, *T0, *T0_frac );
                }
                push_indice_fx( st_fx, IND_PITCH, index, nBits );

                limit_T0_fx( L_FRAME, 8, 0, 0, *T0, 0, T0_min, T0_max );
            }
            ELSE
            {
                IF( sub(nBits,6) == 0 )
                {
                    index = delta_pit_enc_fx( 2, *T0, *T0_frac, PIT_MIN-1 );
                    push_indice_fx( st_fx, IND_PITCH, index, nBits );
                }
                ELSE
                {
                    index = delta_pit_enc_fx( 0, *T0, *T0_frac, PIT_MIN-1 );
                    push_indice_fx( st_fx, IND_PITCH, index, nBits );
                }
            }
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        IF( sub(nBits,10) == 0 )
        {
            pit16k_Q_enc_fx( st_fx, nBits, 1, *T0, *T0_frac, T0_min, T0_max );
        }
        ELSE IF( sub(nBits,6) == 0 )
        {
            index = add(shl(sub(*T0,PIT16k_MIN),1) , shr(*T0_frac,1));
            push_indice_fx( st_fx, IND_PITCH, index, nBits );
        }
        ELSE IF( sub(nBits,5) == 0 )
        {
            index = sub(*T0, PIT16k_MIN);
            push_indice_fx( st_fx, IND_PITCH, index, nBits );
        }
    }
    push_indice_fx( st_fx, IND_TC_IMP_SHAPE, imp_shape, 3 );
    push_indice_fx( st_fx, IND_TC_IMP_POS, imp_pos, 6 );
    push_indice_fx( st_fx, IND_TC_IMP_SIGN, pitch_sign_fx, 1 );
    push_indice_fx( st_fx, IND_TC_IMP_GAIN, pitch_index, 3 );

    *position = add(imp_pos, i_subfr);
    move16();
    return;
}


/*-----------------------------------------------------------------*
 * gain_trans_enc()
 *
 * Quantize gain_trans of TC (gains of glottal impulses).
 * - Uses scalar quantization prototypes tbl_gain_trans_tc[N_GAIN_TC].
 * - Gains the glottal codebook contibution signal.
 *-----------------------------------------------------------------*/
static void gain_trans_enc_fx(
    Word32 gain_trans32,  /* i  : gain for mode Tc Q7             */
    Word16 exc[],         /* i/o: glottal codebook contribution i:Q13 o:Q_new*gain_trans */
    Word16 *quant_index,  /* o  : index of quantized gain_trans   */
    Word16 *quant_sign,   /* o  : sign of quantized gain_trans    */
    Word16 Q_new          /* i  : curent scaling                  */
)
{
    Word16 i, imax, istart, tmp16, gain_trans, gscale;

    istart = 0;
    move16();
    imax = 4;
    move16();
    gscale = 7;
    move16();
    gain_trans = extract_h(L_shl(gain_trans32,16)); /* Q7 */

    IF (L_sub(L_abs(gain_trans32), 29862L)>0)
    {
        gain_trans = extract_h(L_shl(gain_trans32,16-3)); /* Q4 */
        istart = 4;
        move16();
        imax = N_GAIN_TC-1;
        move16();
        gscale = 4;
        move16();
    }

    /* qsign = 0 if *gain_trans < 0 else qsign = 1*/
    tmp16 = shr(gain_trans,16);
    *quant_sign = add(1, tmp16);
    move16();/* quantize sign */
    tmp16 = s_or(tmp16, 1);         /* Keep sign */
    gain_trans = abs_s(gain_trans);

    *quant_index = N_GAIN_TC-1;
    move16();
    FOR (i = istart; i < imax; i++)
    {
        IF (sub(gain_trans,tbl_gain_trans_tc_fx[i]) <= 0 )
        {
            *quant_index = i;
            move16();
            BREAK;
        }
    }

    gain_trans = i_mult2(tbl_gain_trans_tc_fx[i], tmp16);    /* Retreive quantized gain with sign */
    tmp16 = sub(Q_new,add(gscale,13-16+1));             /*remove 16 from rounding */
    FOR (i = 0; i < L_SUBFR; i++)
    {
        /*exc[i] *= (*gain_trans);*/
        exc[i] = round_fx(L_shl(L_mult(exc[i], gain_trans),tmp16));
    }
}

