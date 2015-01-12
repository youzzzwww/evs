/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"
#include "rom_basop_util.h"


/*-------------------------------------------------*
 * Local constants
 *-------------------------------------------------*/

#define LIMIT_PIT_REL_LOWER       2       /* delta interval to extend pitch coding in relative Q */
#define LIMIT_PIT_REL_UPPER       0

/*-------------------------------------------------*
 * limit_T0()
 *
 * Close-loop pitch lag search limitation
 *-------------------------------------------------*/


void limit_T0_fx(
    const Word16 L_frame,  /* i  : length of the frame                                  */
    const Word16 delta,    /* i  : Half the close-loop searched interval                */
    const Word16 pit_flag,     /* i  : selecting absolute(0) or delta(1) pitch quantization */
    const Word16 limit_flag,   /* i  : flag for Q limits (0=restrained, 1=extended)         */
    const Word16 T0,       /* i  : rough pitch estimate around which the search is done */
    const Word16 T0_frac,  /* i  : pitch estimate fractional part                       */
    Word16 *T0_min,  /* o  : lower pitch limit                                    */
    Word16 *T0_max   /* o  : higher pitch limit                                   */
)
{

    Word16 delta2,T1;
    Word16 pit_min, pit_max;

    IF( limit_flag == 0 )  /* restrained Q limits */
    {
        /* set limits */
        IF( sub(L_frame,L_FRAME) == 0)
        {
            pit_max = PIT_MAX;
            move16();
            pit_min = PIT_MIN;
            move16();
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            pit_max = PIT16k_MAX;
            move16();
            pit_min = PIT16k_MIN;
            move16();
        }

        delta2 = sub(shl(delta,1),1);
        T1 = T0;
        move16();

        if( sub(T0_frac,2) >= 0 )
        {
            T1 = add(T1,1);
        }

        *T0_min = sub(T1,delta);
        move16();

        *T0_min = s_max(*T0_min,pit_min);

        *T0_max = add(*T0_min,delta2);
        move16();

        IF( sub(*T0_max,pit_max) > 0)
        {
            *T0_max = pit_max;
            move16();
            *T0_min = sub(*T0_max,delta2);
            move16();
        }
    }
    ELSE  /* extended Q limits */
    {
        /* set limits */
        IF( sub(L_frame, L_FRAME) == 0)
        {
            pit_max = PIT_MAX_EXTEND;
            move16();
            pit_min = PIT_MIN_EXTEND;
            move16();
            if( sub(limit_flag, 2) == 0 )
            {
                pit_min = PIT_MIN_DOUBLEEXTEND;
                move16();
            }
        }
        ELSE /* L_frame == L_FRAME16k */
        {
            pit_max = PIT16k_MAX_EXTEND;
            move16();
            pit_min = PIT16k_MIN_EXTEND;
            move16();
        }

        delta2 = sub(shl(delta,1),1) ;
        move16();
        T1 = T0;
        move16();
        if( sub(T0_frac,2) >= 0 )
        {
            T1 = add(T1,1);
        }
        *T0_min = sub(T1, delta);
        move16();
        IF( pit_flag == 0 )
        {
            /* subframes with absolute search: keep Q range */
            *T0_min = s_max(*T0_min,pit_min);

            *T0_max = add(*T0_min, delta2);
            move16();
            IF( sub(*T0_max,pit_max) > 0)
            {
                *T0_max = pit_max;
                move16();
                *T0_min = sub(*T0_max, delta2);
                move16();
            }
        }
        ELSE
        {
            /* subframes with relative search: extend Q range */
            *T0_min = s_max(*T0_min,sub(pit_min, LIMIT_PIT_REL_LOWER));
            move16();

            *T0_min = s_max(*T0_min,L_INTERPOL);
            *T0_max = *T0_min + delta2;
            move16();

            IF( sub(*T0_max, add(pit_max, LIMIT_PIT_REL_UPPER)) > 0 )
            {
                *T0_max = add(pit_max, LIMIT_PIT_REL_UPPER);
                move16();
                *T0_min = sub(*T0_max, delta2);
                move16();
            }
        }
    }
    return;
}


#define inv_T0_res InvIntTable

/*-------------------------------------------------*
* Routine limit_T0_voiced()
*
* Close-loop pitch lag search limitation
*-------------------------------------------------*/
void limit_T0_voiced(
    const Word16 nbits,
    const Word16 res,
    const Word16 T0,            /* i  : rough pitch estimate around which the search is done */
    const Word16 T0_frac,       /* i  : pitch estimate fractional part                       */
    const Word16 T0_res,        /* i  : pitch resolution                                     */
    Word16 *T0_min,       /* o  : lower pitch limit                                    */
    Word16 *T0_min_frac,  /* o  : lower pitch limit                                    */
    Word16 *T0_max,       /* o  : higher pitch limit                                   */
    Word16 *T0_max_frac,  /* o  : higher pitch limit                                   */
    const Word16 pit_min,       /* i  : Minimum pitch lag                                    */
    const Word16 pit_max        /* i  : Maximum pitch lag                                    */
)
{
    Word16 T1, temp1, temp2, res2;


    assert(res > 1 && res<=6);

    res2  = res;
    move16();
    if(sub(res,6) == 0)
    {
        res2 = shr(res2,1);
    }

    /* Mid-point */
    T1 = T0;
    test();
    if( sub(T0_res,1) > 0 && sub(T0_frac,(shr(T0_res,1))) >= 0 )
    {
        T1 = add(T1,1);
    }

    /* Lower-bound */
    temp1 = sub(i_mult(T1,res),shl(1,sub(nbits,1)));

    temp2 = mult(temp1,inv_T0_res[res2]);
    if(sub(res,6) == 0)
    {
        temp2 = shr(temp2,1);
    }

    *T0_min = temp2;
    move16();

    *T0_min_frac = sub(temp1,i_mult(temp2,res));
    move16();

    IF ( sub(*T0_min,pit_min) < 0)
    {
        *T0_min = pit_min;
        move16();
        *T0_min_frac = 0;
        move16();
    }

    /* Higher-bound */
    temp1 = add(i_mult(*T0_min,res),add(*T0_min_frac,sub(shl(1,nbits),1)));

    temp2 = mult(temp1,inv_T0_res[res2]);
    if(sub(res,6) == 0)
    {
        temp2 = shr(temp2,1);
    }

    *T0_max = temp2;
    move16();

    *T0_max_frac = sub(temp1,i_mult(temp2,res));
    move16();

    IF ( sub(*T0_max,pit_max) > 0)
    {
        *T0_max = pit_max;
        move16();

        *T0_max_frac = sub(res,1);
        move16();

        temp1 = add(i_mult(*T0_max,res),sub(*T0_max_frac,sub(shl(1,nbits),1)));

        temp2 = mult(temp1,inv_T0_res[res2]);
        if(sub(res,6) == 0)
        {
            temp2 = shr(temp2,1);
        }
        move16();
        *T0_min = temp2;

        *T0_min_frac = sub(temp1, i_mult(temp2,res));
        move16();
    }


    return;
}
