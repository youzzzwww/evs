/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


/*This file is up to date with trunk rev. 36531*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "cnst_fx.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "options.h"



/*-------------------------------------------------------------------*
 * - num x 32768
 * -------------------   Q15
 *    PIT_MAX x Fact
 *-------------------------------------------------------------------*/

#define PIT_MAX_DIV_8k(num, fact)   (   -(num *  32768)  /  (PIT_MAX_12k8 * fact) )
#define PIT_MAX_DIV_12k8(num, fact)   (   -(num *  32768)  /  (PIT_MAX_12k8 * fact) )
#define PIT_MAX_DIV_16k(num, fact)   (   -(num *  32768)  /  (PIT_MAX_16k * fact) )


/*-----------------------------------------------------------------*
 * Pitch prediction for frame erasure using linear fitting         *
 *-----------------------------------------------------------------*/
/*port is up to date with trunk 38840*/
void pitch_pred_linear_fit(
    const Word16 /*short*/ bfi_cnt,             /* i:   bfi counter                                  */ /*Q0 */
    const Word16 /*short*/ last_good,           /* i:   last classification type                     */ /*Q0 */
    Word32 /*float*/ *old_pitch_buf,      /* i:   pitch lag buffer                             */ /*Q16*/
    Word32 /*float*/ *old_fpitch,         /* i:                                                */ /*Q16*/
    Word32 /*float*/ *T0_out,             /* o:   estimated close loop pitch                   */ /*Q16*/
    Word16 /*  int*/ pit_min,             /* i:   Minimum pitch lag                            */ /*Q0 */
    Word16 /*  int*/ pit_max,             /* i:   Maximum pitch lag                            */ /*Q0 */
    Word16 /*float*/ *mem_pitch_gain,     /* i:   pitch gain [0] is the most recent subfr gain */ /*Q14*/
    Word16 /*  int*/ limitation,
    Word8  /*short*/ plc_use_future_lag,  /* i:   */                                              /*Q0 */
    Word16 /*short*/ *extrapolationFailed,/* o: flag if extrap decides not to change the pitch */ /*Q0 */
    Word16 nb_subfr                       /* i:   number of ACELP subframes*/
)
{
    Word32 pit, a, b, pita, pitb;
    Word16 sum0;
    Word32 T0;
    Word32 mdy, dy[5];
    Word16 lcor;
    Word16 imax, i;
    Word16 pg[8]; /* local buffer for pitch gain*/
    Word32 ml[8]; /* local buffer for mem_lag*/
    Word16 const timeWeight[5] = {FL2WORD16_SCALE(1.25f,1), FL2WORD16_SCALE(1.125f,1), FL2WORD16_SCALE(1.f,1), FL2WORD16_SCALE(0.875f,1), FL2WORD16_SCALE(.75f,1)}; /*Q14*/
    Word16 no_subfr_pred;
    Word16 a1, a2, a3, a4, a5, tmpa, tmpb, b1, b2, b3, b4, b5;
    Word16 a_e, b_e, sum0_q;
    Word32 mem_lag[2*NB_SUBFR16k+2];


    /* Inverse the order the pitch lag memory */
    IF ( sub(nb_subfr, 4) == 0 )
    {
        FOR (i = 0; i < 2*NB_SUBFR+2; i++)
        {
            mem_lag[i] = old_pitch_buf[2*NB_SUBFR+1 - i];
            move32();
        }
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        FOR (i = 0; i < 2*NB_SUBFR16k+2; i++)
        {
            mem_lag[i] = old_pitch_buf[2*NB_SUBFR16k+1 - i];
            move32();
        }
    }
    move16();
    move16();
    move16();
    move16();
    move16(); /*timeweight*/

    if ( 0 > sub(pit_max,extract_h(*old_fpitch)) )
    {
        *extrapolationFailed = 1;
        *T0_out = *old_fpitch;
        return;
    }

    move16();
    lcor = 5;
    T0 = L_deposit_l(0);

    test();
    test();
    IF (sub(bfi_cnt , 1) == 0 && sub(last_good , UNVOICED_TRANSITION) >= 0 && sub(last_good , ONSET) < 0)
    {
        move16();
        no_subfr_pred = 4;
        if (plc_use_future_lag!=0)
        {
            move16();
            no_subfr_pred = 2;
        }

        /* copy to local buffers, depending on availability of info about future subframes */
        Copy(mem_pitch_gain+no_subfr_pred-2,pg,8);
        Copy32(mem_lag+no_subfr_pred-2,ml,8);

        mdy = L_deposit_l(0);

        FOR (i = (sub(lcor,1)); i >= 0; i--)
        {
            move32();
            dy[i] = L_sub(ml[i] , ml[i+1]);
            mdy = L_add(mdy , dy[i]);
        }

        /*---------------------------------------------------*
         * remove maximum variation
         *---------------------------------------------------*/
        move16();
        imax = 0;
        pita = L_abs(dy[0]);
        FOR (i = 1; i <lcor; i++)
        {

            IF (L_sub(pita,L_abs(dy[i])) < 0)
            {
                pita = L_abs(dy[i]);
                move16();
                imax = i;
            }
        }
        test();
        test();
        IF (L_sub(L_abs(dy[imax]), Mpy_32_16_1(*old_fpitch, FL2WORD16(0.15f))) < 0  && ((sub(limitation , 1)) == 0 || (L_sub(L_abs(dy[imax]),L_abs(mdy))  < 0)))
        {

            FOR (i=0; i<lcor; i++)
            {
                pg[i] = mult(mult(pg[i] , pg[i]) , timeWeight[i]); /*Q12 'til pg[lcor-1], Q14 'til pg[8]*/  move16();
            }

            /* Linear prediction (estimation) of pitch */
            /*
                  sum0=(pg[1]+4*pg[2]+9*pg[3]+16*pg[4])*pg[0]+
                      (pg[2]+4*pg[3]+9*pg[4])*pg[1]+
                      (pg[3]+4*pg[4])*pg[2]+
                      pg[4]*pg[3];*/
            {
                Word32 t1, t2, t3, t4, t5, t6, t7;
                Word16 e1, e2, e3, e4, e5, e6, e7;
                t1 = L_mult0(pg[4], pg[3]); /*Q24*/ /* t1 = pg[4]*pg[3] */
                e1 = 7;
                t2 = L_add(L_deposit_l(pg[3]), L_shl(L_deposit_l(pg[4]), 2)); /*Q12*/
                e2 = norm_l(t2);
                t2 = L_shl(t2, e2); /*Q12,-e2*/
                t2 = Mpy_32_16_1(t2, pg[2]); /*Q9,-e2*/ /* t2 = (pg[3]+4*pg[4])*pg[2] */
                e2 = sub(22, e2);
                t3 = L_add(L_deposit_l(pg[2]), L_add(L_shl(L_deposit_l(pg[3]), 2), L_add(L_shl(L_deposit_l(pg[4]), 3), L_deposit_l(pg[4])))); /*Q12*/
                e3 = norm_l(t3);
                t3 = L_shl(t3, e3); /*Q12,-e3*/
                t3 = Mpy_32_16_1(t3, pg[1]); /*Q9,-e3*/ /* t3 = (pg[2]+4*pg[3]+9*pg[4])*pg[1] */
                e3 = sub(22, e3);
                t4 = L_add(pg[1], L_add(L_shl(L_deposit_l(pg[2]), 2), L_add(L_add(L_shl(L_deposit_l(pg[3]), 3), L_deposit_l(pg[3])), L_shl(L_deposit_l(pg[4]), 4)))); /*Q12*/
                e4 = norm_l(t4);
                t4 = L_shl(t4, e4); /*Q12,-e4*/
                t4 = Mpy_32_16_1(t4, pg[0]); /*Q9,-e4*/ /* t4 = (pg[1]+4*pg[2]+9*pg[3]+16*pg[4])*pg[0] */
                e4 = sub(22, e4);
                t5 = BASOP_Util_Add_Mant32Exp(t1, e1, t2, e2, &e5);
                t6 = BASOP_Util_Add_Mant32Exp(t3, e3, t4, e4, &e6);
                t7 = BASOP_Util_Add_Mant32Exp(t5, e5, t6, e6, &e7); /*Q31,e7*/
                sum0_q = norm_l(t7);
                sum0 = round_fx(L_shl(t7, sum0_q)); /*Q15,e7-sum0_q*/
                sum0_q = add(15, sub(sum0_q, e7)); /* sum0 is now Qsum0_q*/
            }

            pit = 0;
            move16();
            IF (sum0 != 0)
            {
                /* Shift to the right, changing Q as long as no precision is lost */
                WHILE (s_and(sum0, 1) == 0)
                {
                    sum0 = shr(sum0, 1);
                    sum0_q = sub(sum0_q, 1);
                }

                /* float:
                a=-(
                  (  3*pg[1]+4*pg[2]+3*pg[3])*pg[0]                                           *//*a1*//*
*ml[0]   +(
( 2*pg[2]+2*pg[3])*pg[1]-4*pg[1]*pg[0]                                   *//*a2*//*
)*ml[1]  +(
- 8*pg[2]*pg[0]-3*pg[2]*pg[1]+pg[3]*pg[2]                                *//*a3*//*
)*ml[2]  +(
-12*pg[3]*pg[0]-6*pg[3]*pg[1]-2*pg[3]*pg[2]                              *//*a4*//*
)*ml[3]  +(
-16*pg[4]*pg[0]   -9*pg[4]*pg[1]  -4*pg[4]*pg[2]   -pg[4]*pg[3]          *//*a5*//*
)*ml[4] ) /sum0;                                                                                MAC(19);MULT(9);DIV(1);
*/

                /*magic numbers: Q11 if not DIRECTLY marked otherwise*/
                a5 = mac_r(L_mac(L_mac(L_mult(mult_r(-32768,pg[0])  /*Q8*/,pg[4])/*Q5+16*/,  mult_r(-9*2048,pg[1])/*Q8*/ , pg[4]/*Q12*/ )/*Q5+16*/ ,mult_r(-4*2048,pg[2])/*Q8*/, pg[4]/*Q12*/)/*Q5+16*/,mult_r(pg[4],-4096/*Q12->Q9*/),mult_r(pg[3],16384/*Q12->Q11*/))/*Q5*/;
                a4 = mac_r(L_mac(L_mult(      mult_r(-12*2048,pg[0])/*Q8*/,pg[3] /*Q12*/)/*Q5+16*/,mult_r(-6*2048,pg[1])/*Q8*/,pg[3]/*Q12*/)/*Q5+16*/,mult_r(-2*2048,pg[2])/*Q8*/,pg[3]/*Q12*/)/*Q5*/;
                a3 = mac_r(L_mac(L_mult(      mult_r(-8*2048,pg[0]) /*Q8*/,pg[2]),mult_r(-3*2048,pg[1])/*Q8*/,pg[2]),mult_r(pg[2],4096/*Q12->Q9*/),mult_r(pg[3],16384/*12->Q11*/));/*Q5*/
                a2 = mac_r(L_mac(L_mult(      mult_r(2*2048,pg[1])  /*Q8*/,pg[2])/*Q5+16*/,mult_r(2*2048,pg[1])/*Q8*/,pg[3])/*Q5+16*/,mult_r(-4*2048,pg[0])/*Q8*/,pg[1]/*Q12*/)/*Q5*/;
                a1 = mac_r(L_mac(L_mult(      mult_r(3*2048,pg[0])  /*Q8*/,pg[1])/*Q5+16*/,mult_r(4*2048,pg[0])/*Q8*/,pg[2]/*Q12*/)/*Q5+16*/,mult_r(3*2048,pg[0])/*Q8*/,pg[3]/*Q12*/)/*Q5*/;

                a = L_mac(L_mac(L_mac(L_mac(L_mult(a1
                                                   , round_fx(L_shl(ml[0],4)))/*Q4*/
                                            , round_fx(L_shl(ml[1],4)) /*Q4*/, a2)
                                      , round_fx(L_shl(ml[2],4)) /*Q4*/, a3)
                                , round_fx(L_shl(ml[3],4)) /*Q4*/, a4)
                          , round_fx(L_shl(ml[4],4)) /*Q4*/, a5);    /*Q-6+16 = Q10*/

                a_e = norm_l(a);
                a = L_shl(a, a_e);

                a1 =  BASOP_Util_Divide3216_Scale(L_negate(a), /* Numerator  */ /*scalefactor 21*/
                                                  sum0,            /* Denominator*/  /*scalefactor 10*/
                                                  &tmpa);          /* scalefactor for result */

                /* Float:
                 b=((   pg[1]+2*pg[2]+3*pg[3]+4*pg[4])*pg[0]                        *//*b1*//*
*ml[0]   +
(( pg[2]+2*pg[3]+3*pg[4])*pg[1]-pg[1]*pg[0])                 *//*b2*//*
*ml[1]   +
( -2*pg[2]*pg[0]-pg[2]*pg[1]+(pg[3]+2*pg[4])*pg[2])          *//*b3*//*
*ml[2]   +
( -3*pg[3]*pg[0]-2*pg[3]*pg[1]-pg[3]*pg[2]+pg[4]*pg[3])      *//*b4*//*
*ml[3]   +
( -4*pg[4]*pg[0]-3*pg[4]*pg[1]-2*pg[4]*pg[2]-pg[4]*pg[3])    *//*b5*//*
*ml[4]   )/sum0;                                                                                MAC(22);MULT(9);DIV(1);*/

                /*magic numbers in Q13 if not DIRECTLY marked otherwise*/
                b1 = mac_r(L_mac(L_mac(L_mult(mult_r(pg[1],pg[0]),32768/4)/*Q7+16*/,mult_r(2*8192,pg[0])/*Q10*/,pg[2]/*Q12*/)/*Q7+16*/,mult_r(3*8192,pg[0])/*Q10*/,pg[3]/*Q12*/)/*Q7+16*/,  /*mult_r(4*8192,pg[0])*/   pg[0]/*Q10*/,pg[4]/*Q12*/)/*Q7*/;
                b2 = mac_r(L_mac(L_mac(L_mult(mult_r(pg[2],pg[1]),32768/4)/*Q7+16*/,mult_r(2*8192,pg[1]),pg[3]),mult_r(3*8192,pg[1]),pg[4])/*Q7+16*/,mult_r(pg[1],-32768/2/*Q12->Q12*/),mult_r(pg[0],32768/2/*Q12->Q10*/))/*Q7*/;
                b3 = mac_r(L_mac(L_mac(L_mult(mult_r(-2*8192,pg[0]),pg[2])/*Q7+16*/,mult_r(pg[2],-32768/2),mult_r(pg[1],32768/2)),mult_r(pg[3],32768/2),mult_r(pg[2],32768/2))/*Q5+16*/,mult_r(2*8192,pg[2]),pg[4])/*Q7*/;
                b4 = mac_r(L_mac(L_mac(L_mult(mult_r(-3*8192,pg[0]),pg[3]),mult_r(-2*8192,pg[1]),pg[3]),mult_r(-32768/2,pg[3]),mult_r(32768/2,pg[2])),mult_r(32768/2,pg[4]),mult_r(32768/2,pg[3]));/*Q7*/
                b5 = mac_r(L_mac(L_mac(L_mult(mult_r(-32768/*(-4*8192)*/,pg[0]),pg[4]),mult_r(-3*8192,pg[1]),pg[4]),mult_r(-2*8192,pg[2]),pg[4]),mult_r(-32768/2,pg[4]),mult_r(32768/2,pg[3]))/*Q7*/;

                b = L_mac(L_mac(L_mac(L_mac(L_mult(b1
                                                   , round_fx(L_shl(ml[0],4)))/*Q4*/
                                            , round_fx(L_shl(ml[1],4)) /*Q4*/, b2)
                                      , round_fx(L_shl(ml[2],4)) /*Q4*/, b3)
                                , round_fx(L_shl(ml[3],4)) /*Q4*/, b4)
                          , round_fx(L_shl(ml[4],4)) /*Q4*/, b5);    /*Q-4+16 = Q12*/
                /*predict pitch for 4th future subframe*/

                b_e = norm_l(b);
                b = L_shl(b, b_e);

                b1 =  BASOP_Util_Divide3216_Scale(b,       /* Numerator  */ /*scalefactor 19*/
                                                  sum0,                                  /* Denominator*/  /*scalefactor 10*/
                                                  &tmpb);                                /* scalefactor for result*/

                /*pit = a + b * ((float)no_subfr_pred + (float)nb_subfr);*/
                pita = L_shl(  L_deposit_l(a1),add(add(sum0_q, 16-10+1),sub(tmpa, a_e)))/*Q16*/;
                pitb = L_shl_r(L_mult(b1/*Q15*/,add(no_subfr_pred,nb_subfr)/*Q0*/ ),add(add(sum0_q, 16-12),sub(tmpb, b_e)));
                pit = L_add(  pita , pitb ); /*Q16*/
            }

            T0 = L_add(pit, 0);

            /*limit pitch to allowed range*/

            T0 = L_min(L_deposit_h(pit_max),T0);
            T0 = L_max(L_deposit_h(pit_min),T0);

            move16();
            *extrapolationFailed = 0;
        }
        ELSE
        {

            T0 = L_deposit_l(0);
            move16();
            *extrapolationFailed = 1;
        }
    }
    ELSE
    {
        T0 = L_add(*old_fpitch, 0);
        move16();
        *extrapolationFailed = 1;
    }
    move32();
    *T0_out = T0;

    return;
}

/* up to date with rev 8158*/
void get_subframe_pitch(
    Word16 nSubframes,   /* i:   number of subframes                              */   /*  Q0 */
    Word32 pitchStart,   /* i:   starting pitch lag (in subframe -1)              */   /*15Q16*/
    Word32 pitchEnd,     /* i:   ending pitch lag (in subframe nSubframes-1)      */   /*15Q16*/
    Word32 *pitchBuf     /* o:   interpolated pitch lag per subframe              */   /*15Q16*/
)
{
    Word16 i,s;
    Word32 pitchDelta;

    assert((nSubframes > 0) && (pitchBuf != NULL) && (pitchStart >= 0) && (pitchEnd >= 0));


    IF (pitchEnd == 0)
    {
        set32_fx(pitchBuf, pitchStart, nSubframes);
    }
    ELSE
    {
        /*pitchDelta = (pitchEnd - pitchStart)/nSubframes;*/
        pitchDelta = L_deposit_l(BASOP_Util_Divide3216_Scale(L_sub(pitchEnd,pitchStart),nSubframes, &s));/*Q15*/
        pitchDelta = L_shl(pitchDelta,add(s,1));/*Q16*/
        pitchBuf[0] = L_add(pitchStart,pitchDelta);
        FOR (i = 1; i < nSubframes; i++)
        {
            pitchBuf[i] = L_add(pitchBuf[i-1] , pitchDelta);
            move32();
        }
    }
}

