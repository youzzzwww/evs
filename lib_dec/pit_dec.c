/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "cnst_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "options.h"

#include "rom_basop_util.h"
#define inv_T0_res InvIntTable

/*----------------------------------------------------------*
 * Mode2_pit_decode()
 *
 * Decode pitch lag
 *----------------------------------------------------------*/

Word32 Mode2_pit_decode(       /* o:   floating pitch value                      */
    const Word16 coder_type,   /* i:   coding model                              */
    Word16 i_subfr,      /* i:   subframe index                            */
    Word16 L_subfr,
    Word16 **pt_indice,  /* i/o: quantization indices pointer              */
    Word16 *T0,          /* i/o:   close loop integer pitch                */
    Word16 *T0_frac,     /* o:   close loop fractional part of the pitch   */
    Word16 *T0_res,      /* i/o: pitch resolution                          */
    Word16 *T0_min,      /* i/o: lower limit for close-loop search         */
    Word16 *T0_min_frac, /* i/o: lower limit for close-loop search         */
    Word16 *T0_max,      /* i/o: higher limit for close-loop search        */
    Word16 *T0_max_frac, /* i/o: higher limit for close-loop search        */
    Word16  pit_min,
    Word16  pit_fr1,
    Word16  pit_fr1b,
    Word16  pit_fr2,
    Word16  pit_max,
    Word16  pit_res_max
)
{
    Word32 pitch;

    IF(coder_type == 0)
    {
        *T0 = L_subfr;
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
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 4, pit_res_max>>1, *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = shr(pit_res_max,1);
            move16();
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE IF(sub(coder_type,2) == 0) /* 8/5/8/5 (EVS) */
    {
        test();
        IF ( ( i_subfr == 0 ) || ( sub(i_subfr,shl(L_subfr,1))  == 0) )
        {
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 5, shr(pit_res_max,1), *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = shr(pit_res_max,1);
            move16();
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE IF(sub(coder_type,3) == 0) /* 9/6/6/6 (HRs- VC) */
    {
        Word16 pit_res_max2 = pit_res_max;
        if ( sub(pit_min,PIT_MIN_16k)==0 )
        {
            pit_res_max2 = shr(pit_res_max,1);
        }

        IF ( ( i_subfr == 0 ) )
        {
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1, pit_fr2, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 6, pit_res_max2, *T0, 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = pit_res_max2;
            move16();
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE IF(sub(coder_type,4) == 0) /* 9/6/9/6 (AMRWB) */
    {
        Word16 pit_res_max2 = pit_res_max;
        if ( sub(pit_min,PIT_MIN_16k)==0 )
        {
            pit_res_max2 = shr(pit_res_max,1);
        }
        test();
        IF ( ( i_subfr == 0 ) || ( sub(i_subfr,shl(L_subfr,1)) == 0 ) )
        {
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1, pit_fr2, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 6, pit_res_max2, *T0, 0, 1, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = pit_res_max2;
            move16();
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE IF(sub(coder_type,8) == 0) /* 8/5/5/5 (RF all pred mode) */
    {
        IF (i_subfr == 0)
        {
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 5, shr(pit_res_max,1), *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = shr(pit_res_max,1);
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE IF(sub(coder_type,9) == 0) /* 8/0/8/0 (RF gen pred mode) */
    {
        IF (i_subfr == 0)
        {
            Mode2_abs_pit_dec( T0, T0_frac, T0_res, pt_indice, pit_min, pit_fr1b, pit_min, pit_res_max);
        }
        ELSE
        {
            limit_T0_voiced( 4, shr(pit_res_max,1), *T0, *T0_frac, *T0_res, T0_min, T0_min_frac, T0_max, T0_max_frac, pit_min, pit_max );
            *T0_res = shr(pit_res_max,1);
            Mode2_delta_pit_dec( T0, T0_frac, *T0_res, T0_min, T0_min_frac, pt_indice);
        }
    }
    ELSE
    {
        assert(0 && "LTP mode not supported");
    }

    /*-------------------------------------------------------*
     * Compute floating pitch output
     *-------------------------------------------------------*/
    assert(*T0_res > 0 || *T0_res <= 6);

    /*pitch = (float)(*T0) + (float)(*T0_frac)/(float)(*T0_res);*/   /* save subframe pitch values  */
    pitch = L_mac(L_deposit_h(*T0), *T0_frac,inv_T0_res[*T0_res]);


    return pitch;
}



/*---------------------------------------------------------------------*
 * Mode2_abs_pit_dec()
 *
 * Decode the absolute pitch
 *---------------------------------------------------------------------*/

void Mode2_abs_pit_dec(
    Word16 *T0,         /* o:   integer pitch lag              */
    Word16 *T0_frac,    /* o:   pitch fraction                 */
    Word16 *T0_res,     /* o:   pitch resolution               */
    Word16 **pt_indice, /* i/o: pointer to Vector of Q indexes */
    Word16 pit_min,
    Word16 pit_fr1,
    Word16 pit_fr2,
    Word16 pit_res_max
)
{
    Word16 index;
    Word16 pit_res_max_half, tmp1, tmp2, res;

    index = **pt_indice;
    move16();
    (*pt_indice)++;
    pit_res_max_half = shr(pit_res_max,1);

    tmp1 = i_mult(sub(pit_fr2,pit_min),pit_res_max);
    tmp2 = i_mult(sub(pit_fr1,pit_fr2),pit_res_max_half);
    IF (sub(index,tmp1) < 0)
    {
        assert(pit_res_max > 1 && pit_res_max<=6);

        res = pit_res_max;
        move16();
        if(sub(pit_res_max,6) == 0)
        {
            res =shr(res,1);
        }

        *T0 = mult(index,inv_T0_res[res]);
        if(sub(pit_res_max,6) == 0)
        {
            *T0 =shr(*T0,1);
        }

        *T0 = add(pit_min,*T0);
        move16();

        *T0_frac = sub(index,i_mult(sub(*T0,pit_min),pit_res_max));
        move16();
        *T0_res = pit_res_max;
        move16();
    }
    ELSE IF (index <  add(tmp1,tmp2) )
    {
        assert(pit_res_max > 1);

        index =  sub(index,tmp1);
        *T0 = add(pit_fr2,mult(index,inv_T0_res[pit_res_max_half]));
        move16();
        *T0_frac = sub(index, i_mult(sub(*T0,pit_fr2),pit_res_max_half));
        move16();
        *T0_res = pit_res_max_half;
        move16();
    }
    ELSE
    {
        *T0 = add(index,sub(pit_fr1,add(tmp1,tmp2)));
        move16();
        *T0_frac = 0;
        move16();
        *T0_res = 1;
        move16();
    }

    return;
}


/*---------------------------------------------------------------------*
 * Routine Mode2_delta_pit_dec()
 *
 * Decode delta pitch
 *---------------------------------------------------------------------*/
void Mode2_delta_pit_dec(
    Word16    *T0,          /* o:   integer pitch lag              */
    Word16    *T0_frac,     /* o:   pitch fraction                 */
    Word16    T0_res,       /* i:   pitch resolution               */
    Word16    *T0_min,      /* i/o: delta search min               */
    Word16    *T0_min_frac, /* i: delta search min                 */
    Word16    **pt_indice   /* i/o: pointer to Vector of Q indexes */
)
{
    Word16 index, res;

    assert(T0_res > 1 && T0_res<=6);

    res = T0_res;
    move16();
    if(sub(T0_res,6) == 0)
    {
        res =shr(res,1);
    }

    index = **pt_indice;
    move16();
    (*pt_indice)++;


    *T0 = mult(add(index,*T0_min_frac),inv_T0_res[res]);
    if(sub(T0_res,6) == 0)
    {
        *T0 =shr(*T0,1);
    }

    *T0 = add(*T0_min,*T0);
    move16();

    *T0_frac = add(index, sub(*T0_min_frac, i_mult(sub(*T0,*T0_min),T0_res)));

    return;
}


/*======================================================================*/
/* FUNCTION : pit_decode_fx() */
/*-----------------------------------------------------------------------*/
/* PURPOSE :  calculate pitch value                                      */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :													 */
/* _ (Word32) core_brate : Core bitrate                    Q0            */
/* _ (Word16) Opt_AMR_WB : flag indicating AMR-WB IO mode  Q0            */
/* _ (Word16) L_frame : length of the frame                Q0            */
/* _ (Word16) i_subfr : length of the frame                Q0            */
/* _ (Word16) coder_type :  coding type                    Q0            */
/* _ (Word16) L_subfr : subframe length                                  */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/* _ (Word16 *) T0 : close loop integer pitch                            */
/* _ (Word16 *) T0_frac : close loop fractional part of the pitch        */
/* _ (Word16 ) pitch : pitch value                         Q6            */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/* _ (Word16 *) T0_min : delta search min for sf 2 & 4                   */
/* _ (Word16 *) T0_max : delta search max for sf 2 & 4                   */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ (Word16 ) pitch : close loop integer pitch  Q6                      */
/*=======================================================================*/


Word16 pit_decode_fx(                       /* o  : floating pitch value                    */
    Decoder_State_fx *st_fx,                     /* i/o: decoder state structure                 */
    const Word32 core_brate,                 /* i  : core bitrate                            */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode          */
    const Word16 L_frame,                    /* i  : length of the frame                     */
    Word16 i_subfr,                    /* i  : subframe index                          */
    const Word16 coder_type,                 /* i  : coding type                             */
    Word16 *limit_flag,                /* i/o: restrained(0) or extended(1) Q limits   */
    Word16 *T0,                        /* o  : close loop integer pitch                */
    Word16 *T0_frac,                   /* o  : close loop fractional part of the pitch */
    Word16 *T0_min,                    /* i/o: delta search min for sf 2 & 4           */
    Word16 *T0_max,                    /* i/o: delta search max for sf 2 & 4           */
    const Word16 L_subfr                     /* i  : subframe length                         */
)
{
    Word16 pitch;    /*Q2*/
    Word16 pitch_index, nBits, pit_flag;

    pitch_index = 0;

    /*----------------------------------------------------------------*
     * Set pit_flag = 0 for every subframe with absolute pitch search
     *----------------------------------------------------------------*/
    pit_flag = i_subfr;
    move16();

    if (sub(i_subfr,PIT_DECODE_2XL_SUBFR) == 0)
    {
        pit_flag = 0;
        move16();
    }

    /*-------------------------------------------------------*
     * Retrieve the pitch index
     *-------------------------------------------------------*/
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

            if( sub(coder_type,VOICED) == 0)
            {
                *limit_flag = 2;
                move16(); /* double-extended limits */
            }
            test();
            if( sub(coder_type,GENERIC ) == 0 && L_sub(core_brate,ACELP_7k20) == 0)
            {
                *limit_flag = 0;
                move16();
            }
        }
        ELSE IF( sub(i_subfr,2*L_SUBFR) == 0 && L_sub(coder_type,GENERIC) == 0 && L_sub(core_brate,ACELP_13k20) <= 0 )
        {
            if( sub(*T0,shr(add(PIT_FR1_EXTEND_8b, PIT_MIN),1) ) > 0)
            {
                *limit_flag = 0;
                move16();
            }
        }

        /*-------------------------------------------------------*
         *  Retrieve the number of Q bits
         *-------------------------------------------------------*/

        nBits = 0;
        move16();
        IF( sub(coder_type, AUDIO) != 0)
        {
            /* find the number of bits */
            IF( sub(L_frame,L_FRAME) == 0)
            {
                nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, 0)];
                move16();
            }
            ELSE  /* L_frame == L_FRAME16k */
            {
                nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, 0)];
                move16();
            }

            pitch_index = (Word16)get_next_indice_fx( st_fx, nBits );
            move16();
        }

        /*-------------------------------------------------------*
         * Pitch decoding in AUDIO mode
         * (both ACELP@12k8 and ACELP@16k cores)
         *-------------------------------------------------------*/
        IF( sub(coder_type, AUDIO) == 0)
        {
            test();
            test();
            if( L_subfr == shr(L_frame,1) && i_subfr != 0 && L_frame == L_FRAME )
            {
                pit_flag = L_SUBFR;
                move16();
            }
            if( pit_flag == 0 )
            {
                nBits = 10;
                move16();
            }
            if( pit_flag != 0 )
            {
                nBits = 6;
                move16();
            }

            pitch_index = (Word16)get_next_indice_fx( st_fx, nBits );
            move16();
            IF( L_frame == L_FRAME )
            {
                pit_Q_dec_fx( 0, pitch_index, nBits, 4, pit_flag, *limit_flag, T0, T0_frac, T0_min, T0_max );
            }
            ELSE
            {
                pit16k_Q_dec_fx( pitch_index, nBits, *limit_flag, T0, T0_frac, T0_min, T0_max );
            }
        }
        ELSE IF( sub(coder_type,VOICED) == 0)
        {
            /*-------------------------------------------------------*
             * Pitch decoding in VOICED mode
             * (ACELP@12k8 core only)
             *-------------------------------------------------------*/
            if( sub(i_subfr,2*L_SUBFR) == 0)
            {
                pit_flag = i_subfr;
                move16();
            }

            pit_Q_dec_fx( 0, pitch_index, nBits, 4, pit_flag, *limit_flag, T0, T0_frac, T0_min, T0_max );
        }
        ELSE
        {
            /*-------------------------------------------------------*
             *  Pitch decoding in GENERIC mode
             * (both ACELP@12k8 and ACELP@16k cores)
             *-------------------------------------------------------*/
            IF( sub(L_frame,L_FRAME) == 0)
            {
                pit_Q_dec_fx( 0, pitch_index, nBits, 8, pit_flag, *limit_flag, T0, T0_frac, T0_min, T0_max );
            }
            ELSE
            {
                pit16k_Q_dec_fx( pitch_index, nBits, *limit_flag, T0, T0_frac, T0_min, T0_max );
            }
        }
    }

    /*-------------------------------------------------------*
     *  Pitch decoding in AMR-WB IO mode
     *-------------------------------------------------------*/

    ELSE
    {
        *limit_flag = 0;
        move16();
        test();
        test();
        IF( i_subfr == 0 || ( sub(i_subfr, 2*L_SUBFR) == 0 && L_sub(core_brate,ACELP_8k85) == 0 ) )
        {
            nBits = 8;
            move16();
        }
        ELSE
        {
            nBits = 5;
            move16();
        }
        IF( L_sub(core_brate, ACELP_8k85) > 0)
        {
            nBits = 6;
            move16();
            test();
            if( i_subfr == 0 || sub(i_subfr, 2*L_SUBFR) == 0)
            {
                nBits = 9;
                move16();
            }
        }

        pitch_index = (Word16)get_next_indice_fx( st_fx, nBits );

        pit_Q_dec_fx( 1, pitch_index, nBits, 8, pit_flag, *limit_flag, T0, T0_frac, T0_min, T0_max );
    }

    /*-------------------------------------------------------*
     * Compute floating pitch output
     *-------------------------------------------------------*/

    pitch = shl(add(shl(*T0,2),*T0_frac),4);   /* save subframe pitch values Q6 */

    return pitch;
}


/*----------------------------------------------------------*
 * pit_Q_dec_fx()
 *
 * Decode pitch lag
 *----------------------------------------------------------*/

void pit_Q_dec_fx(
    const Word16 Opt_AMR_WB,   /* i  : flag indicating AMR-WB IO mode          */
    const Word16 pitch_index,  /* i  : pitch index                             */
    const Word16 nBits,        /* i  : # of Q bits                             */
    const Word16 delta,        /* i  : Half the CL searched interval           */
    const Word16 pit_flag,     /* i  : absolute(0) or delta(1) pitch Q         */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) Q limits   */
    Word16 *T0,          /* o  : integer pitch lag                       */
    Word16 *T0_frac,     /* o  : pitch fraction                          */
    Word16 *T0_min,      /* i/o: delta search min                        */
    Word16 *T0_max       /* i/o: delta search max                        */
)
{
    IF( sub(nBits, 10) == 0)         /* absolute decoding with 10 bits */
    {
        IF( limit_flag == 0 )
        {
            *T0 = add(PIT_MIN,shr(pitch_index,2));
            *T0_frac = sub(pitch_index,shl(sub(*T0,PIT_MIN),2));
        }
        ELSE IF( sub(limit_flag,1) == 0 )
        {
            *T0 = add(PIT_MIN_EXTEND,shr(pitch_index,2));
            *T0_frac = sub(pitch_index ,shl(sub(*T0,PIT_MIN_EXTEND),2));
        }
        ELSE  /* limit_flag == 2 */
        {
            *T0 = add(PIT_MIN_DOUBLEEXTEND,shr(pitch_index,2));
            *T0_frac = sub(pitch_index ,shl(sub(*T0,PIT_MIN_DOUBLEEXTEND),2));
        }
    }
    ELSE IF( sub(nBits, 9) == 0)     /* absolute decoding with 9 bits */
    {
        abs_pit_dec_fx( 4, pitch_index, limit_flag, T0, T0_frac );

        /* find T0_min and T0_max for delta search */
        IF( Opt_AMR_WB )
        {
            limit_T0_fx( L_FRAME, delta, pit_flag, 0, *T0, 0, T0_min, T0_max );        /* T0_frac==0 to keep IO with AMR-WB */
        }
    }
    ELSE IF( sub(nBits, 8) == 0 )     /* absolute decoding with 8 bits */
    {
        abs_pit_dec_fx( 2, pitch_index, limit_flag, T0, T0_frac );

        /* find T0_min and T0_max for delta search */
        IF( Opt_AMR_WB )
        {
            limit_T0_fx( L_FRAME, delta, pit_flag, 0, *T0, 0, T0_min, T0_max );        /* T0_frac==0 to keep IO with AMR-WB */
        }
    }
    ELSE IF( sub(nBits, 6) == 0)     /* relative decoding with 6 bits */
    {
        delta_pit_dec_fx( 4, pitch_index, T0, T0_frac, *T0_min );
    }
    ELSE IF( sub(nBits, 5) == 0 )     /* relative decoding with 5 bits */
    {
        IF( sub(delta,8) == 0 )
        {
            delta_pit_dec_fx( 2, pitch_index, T0, T0_frac, *T0_min );
        }
        ELSE  /* delta == 4 */
        {
            delta_pit_dec_fx( 4, pitch_index, T0, T0_frac, *T0_min );
        }
    }
    ELSE /* nBits == 4 */     /* relative decoding with 4 bits */
    {
        IF( sub(delta,8) == 0 )
        {
            delta_pit_dec_fx( 0, pitch_index, T0, T0_frac, *T0_min );
        }
        ELSE  /* delta == 4 */
        {
            delta_pit_dec_fx( 2, pitch_index, T0, T0_frac, *T0_min );
        }
    }
    IF( !Opt_AMR_WB )
    {
        /* find T0_min and T0_max for delta search */
        limit_T0_fx( L_FRAME, delta, L_SUBFR, limit_flag, *T0, *T0_frac, T0_min, T0_max );
    }

    return;
}

/*-------------------------------------------------*
  * pit16k_Q_dec()
  *
  * pitch decoding @16kHz core
  *-------------------------------------------------*/

void pit16k_Q_dec_fx(
    const Word16  pitch_index,  /* i  : pitch index                             */
    const Word16  nBits,        /* i  : # of Q bits                             */
    const Word16  limit_flag,   /* i  : restrained(0) or extended(1) limits     */
    Word16  *T0,          /* o  : integer pitch lag                       */
    Word16  *T0_frac,     /* o  : pitch fraction                          */
    Word16  *T0_min,      /* i/o: delta search min                        */
    Word16  *T0_max       /* i/o: delta search max                        */
)
{
    Word16  index;

    IF( sub(nBits,10) == 0)         /* absolute decoding with 10 bits */
    {
        IF( limit_flag == 0 )
        {
            *T0 = add(PIT16k_MIN, shr(pitch_index,2));
            move16();
            *T0_frac = sub(pitch_index, shl(sub(*T0, PIT16k_MIN),2));
            move16();
        }
        ELSE  /* extended Q range */
        {
            IF( sub(pitch_index,shl((PIT16k_FR2_EXTEND_10b-PIT16k_MIN_EXTEND),2)) < 0 )
            {
                *T0 = add(PIT16k_MIN_EXTEND, shr(pitch_index,2));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT16k_MIN_EXTEND),2));
                move16();
            }
            ELSE
            {
                index =  sub(pitch_index, shl((PIT16k_FR2_EXTEND_10b-PIT16k_MIN_EXTEND),2));
                *T0 = add(PIT16k_FR2_EXTEND_10b, shr(index,1));
                *T0_frac = sub(index, shl(sub(*T0, PIT16k_FR2_EXTEND_10b),1));
                /*(*T0_frac) *= 2;*/
                (*T0_frac) = shl(*T0_frac,1);

            }
        }

    }
    ELSE IF ( sub(nBits,9) == 0 )     /* absolute decoding with 9 bits */
    {
        IF( limit_flag == 0 )
        {
            IF (sub(pitch_index,(PIT16k_FR2_9b-PIT16k_MIN)*4) < 0 )
            {
                *T0 = add(PIT16k_MIN, shr(pitch_index,2));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT16k_MIN),2));
                move16();
            }
            ELSE IF (sub(pitch_index,((PIT16k_FR2_9b-PIT16k_MIN)*4 + (PIT16k_FR1_9b-PIT16k_FR2_9b)*2)) < 0 )
            {
                index =  sub(pitch_index, (PIT16k_FR2_9b-PIT16k_MIN)*4);
                *T0 = add(PIT16k_FR2_9b, shr(index,1));
                move16();
                *T0_frac = sub(index, shl(sub(*T0, PIT16k_FR2_9b),1));
                move16();
                (*T0_frac) = shl((*T0_frac),1);
            }
            ELSE
            {
                *T0 = add(pitch_index, PIT16k_FR1_9b - ((PIT16k_FR2_9b-PIT16k_MIN)*4) - ((PIT16k_FR1_9b-PIT16k_FR2_9b)*2));
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE  /* extended Q range */
        {
            IF (sub(pitch_index,(PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4) < 0)
            {
                *T0 = add(PIT16k_MIN_EXTEND, shr(pitch_index,2));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT16k_MIN_EXTEND),2));
                move16();
            }
            ELSE IF (sub(pitch_index,( (PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4 + (PIT16k_FR1_EXTEND_9b-PIT16k_FR2_EXTEND_9b)*2)) < 0 )
            {
                index =  sub(pitch_index, (PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4);
                *T0 = add(PIT16k_FR2_EXTEND_9b, shr(index,1));
                move16();
                *T0_frac = sub(index, shl(sub(*T0, PIT16k_FR2_EXTEND_9b),1));
                move16();
                (*T0_frac) *= shl((*T0_frac),1);
            }
            ELSE
            {
                *T0 = add(pitch_index, PIT16k_FR1_EXTEND_9b - ((PIT16k_FR2_EXTEND_9b-PIT16k_MIN_EXTEND)*4) - ((PIT16k_FR1_EXTEND_9b-PIT16k_FR2_EXTEND_9b)*2));
                move16();
                *T0_frac = 0;
                move16();
            }
        }
    }
    ELSE  /* nBits == 6 */    /* relative decoding with 6 bits */
    {
        delta_pit_dec_fx( 4, pitch_index, T0, T0_frac, *T0_min );
    }

    /* find T0_min and T0_max for delta search */
    limit_T0_fx( L_FRAME16k, 8, L_SUBFR, limit_flag, *T0, *T0_frac, T0_min, T0_max );

    return;
}

/*----------------------------------------------------------*
 * abs_pit_dec_fx()
 *
 * Absolute pitch decoding
 *----------------------------------------------------------*/

void abs_pit_dec_fx(
    const Word16 fr_steps,    /* i:   fractional resolution steps (0, 2, 4)    */
    Word16 pitch_index, /* i:   pitch index                              */
    const Word16 limit_flag,   /* i  : restrained(0) or extended(1) limits     */
    Word16 *T0,         /* o:   integer pitch lag                        */
    Word16 *T0_frac     /* o:   pitch fraction                           */
)
{
    Word16 temp;

    IF( limit_flag == 0 )
    {
        IF(sub(fr_steps,2) == 0)
        {
            IF(sub(pitch_index,PIT_FR1_8b_MINUS_PIT_MIN_X2) < 0)
            {
                *T0= add(PIT_MIN,shr(pitch_index,1));
                move16();
                temp = shl(sub(*T0,PIT_MIN),1);
                *T0_frac = shl(sub(pitch_index,temp),1);
                move16();
            }
            ELSE
            {
                *T0 = add(pitch_index,PIT_FR1_8b_MINUS_PIT_FR1_8b_MINUS_PIT_MIN_X2);
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            IF(sub(pitch_index,PIT_FR2_9b_MINUS_PIT_MIN_X4) < 0)
            {
                *T0= add(PIT_MIN,shr(pitch_index,2));
                move16();
                temp = shl(sub(*T0,PIT_MIN),2);
                *T0_frac = sub(pitch_index,temp);
                move16();
            }
            ELSE IF (sub(pitch_index,PIT_DECODE_1) < 0)  /*( (PIT_FR2_9b-PIT_MIN)*4 + (PIT_FR1_9b-PIT_FR2_9b)*2) = 440*/
            {
                pitch_index = sub(pitch_index,PIT_DECODE_2);   /*pitch_index -=  (PIT_FR2_9b-PIT_MIN)*4(=376);*/
                *T0 = add(PIT_FR2_9b,shr(pitch_index,1));
                move16();
                temp = shl(sub(*T0,PIT_FR2_9b),1);
                *T0_frac = shl(sub(pitch_index,temp),1);
                move16();
            }
            ELSE
            {
                *T0 = add(pitch_index,PIT_DECODE_3);
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE  /* fr_steps == 0 */
        {
            /* not used in the codec */
        }
    }
    ELSE IF( sub(limit_flag, 1) == 0 )   /* extended Q range */
    {
        IF( sub(fr_steps,2) == 0 )
        {
            IF( sub(pitch_index, PIT_FR1_EXT8b_MINUS_PIT_MIN_EXT_X2) < 0 )
            {
                *T0 = add(PIT_MIN_EXTEND, shr(pitch_index,1));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_MIN_EXTEND),1));
                *T0_frac = shl(*T0_frac, 1);
                move16();
            }
            ELSE
            {
                *T0 = add(pitch_index, sub(PIT_FR1_EXTEND_8b, PIT_FR1_EXT8b_MINUS_PIT_MIN_EXT_X2));
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            IF( sub(pitch_index, PIT_FR2_EXT9b_MINUS_PIT_MIN_EXT_X4)  < 0 )
            {
                /**T0 = PIT_MIN_EXTEND + (pitch_index/4);*/
                *T0 = add(PIT_MIN_EXTEND, shr(pitch_index,2));
                move16();
                /**T0_frac = pitch_index - (*T0 - PIT_MIN_EXTEND)*4;*/
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_MIN_EXTEND),2));
                move16();
            }
            ELSE IF( sub(pitch_index,add(PIT_FR2_EXT9b_MINUS_PIT_MIN_EXT_X4, PIT_FR1_EXT9b_MINUS_PIT_FR2_EXT9b_X2)) < 0 )
            {
                /*pitch_index -=  (PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4;*/
                pitch_index =  sub(pitch_index, PIT_FR2_EXT9b_MINUS_PIT_MIN_EXT_X4);
                *T0 = add(PIT_FR2_EXTEND_9b, shr(pitch_index,1));
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_FR2_EXTEND_9b),1));
                (*T0_frac) = shl(*T0_frac,1);
                move16();
            }
            ELSE
            {
                /**T0 = pitch_index + PIT_FR1_EXTEND_9b - ((PIT_FR2_EXTEND_9b-PIT_MIN_EXTEND)*4) - ((PIT_FR1_EXTEND_9b-PIT_FR2_EXTEND_9b)*2);move16();*/
                *T0 = add(pitch_index, PIT_DECODE_7);
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE  /* fr_steps == 0 */
        {
            /* not used in the codec */
        }
    }
    ELSE  /* limit_flag == 2 */
    {
        IF( sub(fr_steps,2) == 0 )
        {
            IF( sub(pitch_index,PIT_FR1_DEXT8b_MINUS_PIT_MIN_DEXT_X2) < 0)
            {
                *T0 = add(PIT_MIN_DOUBLEEXTEND, shr(pitch_index,1));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_MIN_DOUBLEEXTEND),1));
                move16();
                *T0_frac = shl(*T0_frac,1);
                move16();
            }
            ELSE
            {
                /**T0 = pitch_index + PIT_FR1_DOUBLEEXTEND_8b - ((PIT_FR1_DOUBLEEXTEND_8b-PIT_MIN_DOUBLEEXTEND)*2);move16();*/
                *T0 = add(pitch_index, PIT_DECODE_8);
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE IF( sub(fr_steps,4) == 0 )
        {
            IF( sub(pitch_index, PIT_FR2_DEXT9b_MINUS_PIT_MIN_DEXT_X4) < 0)
            {
                *T0 = add(PIT_MIN_DOUBLEEXTEND, shr(pitch_index,2));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_MIN_DOUBLEEXTEND),2));
                move16();
            }
            ELSE IF( sub(pitch_index,PIT_DECODE_9) < 0)
            {
                /*pitch_index -=  (PIT_FR2_DOUBLEEXTEND_9b-PIT_MIN_DOUBLEEXTEND)*4;move16();*/
                pitch_index =  sub(pitch_index , PIT_FR2_DEXT9b_MINUS_PIT_MIN_DEXT_X4);
                move16();
                *T0 = add(PIT_FR2_DOUBLEEXTEND_9b, shr(pitch_index,1));
                move16();
                *T0_frac = sub(pitch_index, shl(sub(*T0, PIT_FR2_DOUBLEEXTEND_9b),1));
                move16();
                (*T0_frac) = shl(*T0_frac,1);
                move16();
            }
            ELSE
            {
                *T0 = add(pitch_index, PIT_DECODE_10);
                move16();
                *T0_frac = 0;
                move16();
            }
        }
        ELSE  /* fr_steps == 0 */
        {
            /* not used in the codec */
        }
    }

    return;
}

/*----------------------------------------------------------*
 * delta_pit_dec_fx()
 *
 * Delta pitch decoding
 *----------------------------------------------------------*/

void delta_pit_dec_fx(
    const Word16 fr_steps,                   /* i  : fractional resolution steps (0, 2, 4)   */
    const Word16 pitch_index,                /* i  : pitch index                             */
    Word16 *T0,                        /* o  : integer pitch lag                       */
    Word16 *T0_frac,                   /* o  : pitch fraction                          */
    const Word16 T0_min                      /* i  : delta search min                        */
)
{

    Word16 temp;
    IF( fr_steps == 0 )
    {
        *T0 = add(T0_min,pitch_index);
        move16();
        *T0_frac = 0;
        move16();
    }
    ELSE IF( sub(fr_steps,2) == 0 )
    {
        *T0 = add(T0_min,shr(pitch_index,1));
        move16();
        temp = shl(sub(*T0,T0_min),1);
        *T0_frac = shl(sub(pitch_index,temp),1);
        move16();
    }
    ELSE IF ( sub(fr_steps,4) == 0 )
    {
        *T0 = add(T0_min,shr(pitch_index,2));
        move16();
        temp = shl(sub(*T0,T0_min),2);
        *T0_frac = sub(pitch_index,temp);
        move16();
    }

    return;
}
