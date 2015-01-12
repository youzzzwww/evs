/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_dec_fx.h"    /* Decoder static table prototypes        */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*---------------------------------------------------------------------*
 * FEC_SinOnset()
 *
 * Create an artificial onset when it is lost
 *---------------------------------------------------------------------*/
void FEC_SinOnset_fx (
    Word16 *exc,        /* i/o : exc vector to modify                                           */
    Word16 puls_pos,    /* i   : last pulse position desired                                    */
    const Word16 T0,          /* i   : Pitch information of the 1 subfr                               */
    Word32 enr_q,       /* i   : energy provide by the encoder                                  */
    Word16 *Aq,         /* i   : A(z) filter   Q12                                              */
    const Word16 L_frame      /* i   : frame length                                                   */
    ,const Word16 Qold
)
{
    Word16 P0, onset_len, sign, i, len, L_subfr, L_subfr2;
    Word16 h1[L_SUBFR16k], mem[M], exc_tmp[L_FRAME16k+ MODE1_L_FIR_FER];
    Word16 *pt_end, *pt_exc, enr_LP, gain, H_low_s[5], exp_gain, exp2, tmp;
    Word32 L_tmp;
    Word16 Q_exc;

    sign = 0;
    move16();
    Q_exc = Qold;
    move16();
    L_subfr = L_SUBFR;
    move16();
    if( sub(L_frame,L_FRAME16k) == 0 )
    {
        L_subfr = L_SUBFR16k;
        move16();
    }

    L_subfr2 = shl(L_subfr,1);
    onset_len = s_max(T0, L_subfr2);

    P0 = puls_pos;
    move16();

    IF (P0 < 0)
    {
        sign = 1;
        move16();
        P0 = negate(P0);
    }

    test();
    test();
    IF ( sub(P0,PIT_MAX) > 0 && sub(L_frame,L_FRAME) == 0 )
    {
        P0 = PIT_MAX;
        move16();/* Should never be the case, however... */
    }
    ELSE if ( sub(P0,PIT16k_MAX) > 0 && sub(L_frame,L_FRAME16k) == 0 )
    {
        P0 = PIT16k_MAX;
        move16(); /* Should never be the case, however... */
    }
    set16_fx( exc_tmp, 0, add(L_frame, MODE1_L_FIR_FER));                      /* Reset excitation vector */

    /*-------------------------------------------------------------------------------*
     * Find LP filter impulse response energy and Generate the scaled pulse prototype
     *-------------------------------------------------------------------------------*/

    set16_fx(h1, 0, L_subfr); /* Find the impulse response */
    set16_fx(mem, 0, M);
    h1[0] = 1024;
    move16();
    Syn_filt_s(1, Aq, M, h1, h1, L_subfr, mem, 0);


    enr_LP = extract_h(Dot_product12(h1, h1, L_subfr, &exp_gain));
    exp_gain = sub(exp_gain, 10 + 10); /* h1 in Q10 */

    /* divide by LP filter E, scaled by transmitted E */
    /* gain = (float)sqrt( enr_q / enr_LP ); */

    enr_q = L_max(enr_q,1);

    exp2 = norm_l(enr_q);
    tmp = extract_h(L_shl(enr_q, exp2));
    tmp = mult(tmp, 24576); /* multpiply by 1.5 */

    IF(sub(tmp, 16384) < 0)
    {
        exp2 = add(exp2, 1);
        tmp = shl(tmp, 1);
    }
    exp2 = sub(30, exp2); /* in Q15 */


    IF(sub(enr_LP, tmp) > 0)
    {
        enr_LP = shr(enr_LP, 1);
        exp_gain = add(exp_gain, 1);
    }

    tmp = div_s(enr_LP, tmp);
    exp2 = sub(exp_gain, exp2);

    L_tmp = L_deposit_h(tmp);
    L_tmp = Isqrt_lc(L_tmp, &exp2);
    gain = round_fx(L_tmp);

    gain = mult_r(gain, 31457); /* multiply by .96 like floating point */
    exp2 = add(sub(exp2, 15), Q_exc); /* from Q15 to Q_exc */

    /* Find if rescaling needed */
    tmp = extract_h(L_mult(H_low[2], gain));
    exp_gain = norm_s(tmp);
    tmp = sub(exp_gain, exp2); /* difference */

    IF(tmp < 0)/* Need to correct scaling */
    {
        Q_exc = add(Q_exc, tmp);
        exp2 = add(exp2, tmp);
    }
    /* Scale pulse "prototype" energy */
    /* Generate the scaled pulse */
    FOR(i = 0; i < MODE1_L_FIR_FER; i++)
    {
        L_tmp = L_mult(gain, H_low[i]); /* Q_exc*Q15 -> Q_exc */
        H_low_s[i] = round_fx(L_shl(L_tmp, exp2));
    }
    /*------------------------------------------------------------------------------------------*
     * Construct the harmonic part as a train of low-pass filtered pulses
     *------------------------------------------------------------------------------------------*/
    move16();
    move16();
    move16();
    pt_exc = exc_tmp + sub(L_frame, add(add(1, MODE1_L_FIR_FER/2), P0)); /* beginning of the 1st pulse */
    pt_end = exc_tmp + onset_len;
    len = (Word16) (pt_exc - pt_end);

    len = s_min(len, MODE1_L_FIR_FER);
    IF(!sign)
    {

        FOR(i = 0; i < len; i++)
        {
            /* The filter response would have E=1 in full band. */
            pt_exc[i] = add(pt_exc[i], H_low_s[i]);
            move16();
        }
    }
    ELSE
    {
        FOR(i = 0; i < len; i++)
        {
            /* The filter response would have E=1 in full band. */
            pt_exc[i] = sub(pt_exc[i], H_low_s[i]);
            move16();
        }
    }
    Copy(&exc_tmp[L_frame - L_EXC_MEM], exc, L_EXC_MEM);
    return;
}

Word16 FEC_enhACB_fx(
    const Word16 L_frame,                   /* i   : frame length                                                              */
    Word16 *exc_io,                   /* i/o : adaptive codebook memory                                                  */
    const Word16 new_pit,                   /* i   : decoded first frame pitch                                                 */
    const Word16 puls_pos,                  /* i   : decoder position of the last glottal pulses decoded in the previous frame */
    const Word16 bfi_pitch                  /* i   : Q6 pitch used for concealment                                                */
)
{
    Word16 Tc, P0, sign, pit_search;
    Word16 Tlist[10], Terr, diff_pit, dist_Plast;
    Word16 tmp2;
    Word16 exc[L_FRAME16k + L_SUBFR];
    Word16 Do_WI = 1;
    move16();


    set16_fx(exc, 0, L_FRAME16k - L_EXC_MEM);
    set16_fx(exc+L_FRAME16k, 0, L_SUBFR);
    Copy(exc_io, exc + L_FRAME16k - L_EXC_MEM, L_EXC_MEM);

    Tc = shr(bfi_pitch,6);
    Copy(exc + L_FRAME16k - Tc, exc + L_FRAME16k, L_SUBFR);

    /*------------------------------------------------------------
     * Decode phase information transmitted in the bitstream
     * (The position of the absolute maximum glottal pulse from
     * the end of the frame and its sign)
     *------------------------------------------------------------*/

    P0 = puls_pos;
    move16();
    sign = 0;
    move16();
    IF (P0 < 0)
    {
        sign = 1;
        move16();
        P0 = negate(P0);
    }

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        P0 = s_min(PIT_MAX, P0);
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        P0 = s_min(PIT16k_MAX, P0);
    }

    /*----------------------------------------------------------------------------------
     * Find the position of the first the maximum(minimum) lp_filtered pulse
     * <----- Mem --->|<--------------------- L_frame ------------>|<----- L_SUBFR --->|
     *                |<-------pit_search---->                     |                   |
     *----------------------------------------------------------------------------------*/

    pit_search = Tc;
    move16();

    Tlist[0] = findpulse_fx( L_frame, exc+sub(L_frame,pit_search) , pit_search,DEC, &sign);

    /*Terr = (short) abs(pit_search-Tlist[0]-P0);*/
    Terr = abs_s(sub(pit_search,add(Tlist[0],P0)));

    dist_Plast = sub(Tc,Tlist[0]);

    Tlist[1] = findpulse_fx( L_frame, exc+sub(L_frame,pit_search) , add(pit_search,L_SUBFR),DEC, &sign);


    /*if(Terr > abs(Tlist[1]-Tc + P0))*/
    IF(sub(Terr,abs_s(add(sub(Tlist[1],Tc), P0))) > 0)
    {
        dist_Plast = sub(Tc,Tlist[1]);
        Terr = abs_s(add(sub(Tlist[1],Tc), P0));
    }

    diff_pit = abs_s(sub(new_pit, Tc));
    /*ftmp = (float) (int)((float)L_frame/(float)Tc+0.5);*/
    tmp2 = mult_r(div_s(16, Tc), shr(L_frame,4));
    test();
    test();
    IF (sub(Terr, i_mult(tmp2, diff_pit)) <= 0 &&
        Terr != 0 && /* If Terr = 0, no resynchronization required */
        sub(Terr, L_SUBFR) < 0)/* prevent catastrophy search */
    {
        /* performe excitation resynchronization here */
        FEC_synchro_exc_fx( L_frame, exc, P0, dist_Plast, Tc );
        Copy(exc + L_FRAME16k - L_EXC_MEM, exc_io, L_EXC_MEM);
    }
    ELSE
    {
        Do_WI = 0;
        move16();
    }

    return Do_WI;
}

void FEC_synchro_exc_fx(
    const Word16 L_frame,          /* i  : length of the frame                               */
    Word16 *exc,             /* i/o: exc vector to modify                              */
    const Word16 desire_puls_pos,  /* i  : Pulse position send by the encoder                */
    const Word16 true_puls_pos,    /* i  : Present pulse location                            */
    const Word16 Old_pitch         /* i  : Pitch use to create temporary adaptive codebook   */
)
{
    Word16 exc_tmp[L_FRAME16k+L_SUBFR];
    Word16 fact;
    Word32 L_min_energy, L_tmp;
    Word16 *pt_exc, *pt_exc1;
    Word16 i, j, point_to_remove, point_to_add, nb_min;
    Word16 min_pos[L_FRAME16k/PIT_MIN_DOUBLEEXTEND], points_by_pos[L_FRAME16k/PIT_MIN_DOUBLEEXTEND];
    Word16 total_point, tmp_len;
    Word16 *pt_pos, pos, start_search, tmp16;
    Word16 remaining_len;

    point_to_add = -1;
    move16();

    /* Init */
    FOR (i = 0; i < L_FRAME16k/PIT_MIN_DOUBLEEXTEND; i++)
    {
        min_pos[i] = 10000;
        move16();
        points_by_pos[i] = 0;
        move16();
    }

    /* Find number of point to remove and number of minimum */
    point_to_remove = sub(true_puls_pos, desire_puls_pos); /* if it is negative it means remove point else it means add point */

    pos = sub(L_frame, true_puls_pos);

    /* Find number of minimum energy region */
    /* nb_min = (L_FRAME - true_puls_pos)/Old_pitch */
    tmp16 = shl(Old_pitch, 5);
    tmp16 = div_s(pos, tmp16);
    nb_min = shr(tmp16, 10);
    /* if Old pitch < 128, must have at least 2 min */
    if (sub(Old_pitch, 128) <= 0)
    {
        nb_min = s_max(nb_min, 2);
    }
    /* Must have at least 1 min */
    nb_min = s_max(nb_min, 1);

    pt_exc = exc + pos;
    move16();

    /* Find starting point for minimum energy search */
    start_search = mult_r(Old_pitch, -24576);
    if(sub(s_and(Old_pitch,3),1) ==0)
    {
        /* Only be align with integer operation -3*Old_pitch/4 */
        start_search = add(start_search,1);
    }
    IF (add(start_search, pos) < 0)
    {
        start_search = negate(pos);
        IF (sub(abs_s(start_search), shr(Old_pitch, 3)) < 0)
        {
            /* it's not safe to remove/add point inside 1/8 of the pulse position */
            return;
        }
    }

    /* Find min energy in the first pitch section */
    /* --------------------------------------------------------------------
     * The minimum energy regions are determined by the computing the energy
     * using a sliding 5-sample window. The minimum energy position is set
     * at the middle of the window at which the energy is at minimum
     * --------------------------------------------------------------------*/
    L_min_energy = L_add(MAX_32, 0);
    L_tmp = L_mult(pt_exc[start_search], pt_exc[start_search]);
    L_tmp = L_mac(L_tmp, pt_exc[start_search + 1], pt_exc[start_search + 1]);
    L_tmp = L_mac(L_tmp, pt_exc[start_search + 2], pt_exc[start_search + 2]);
    L_tmp = L_mac(L_tmp, pt_exc[start_search + 3], pt_exc[start_search + 3]);
    L_tmp = L_mac(L_tmp, pt_exc[start_search + 4], pt_exc[start_search + 4]);

    IF (L_sub(L_tmp, L_min_energy) < 0)
    {
        L_min_energy = L_add(L_tmp, 0);
        min_pos[0] = add(add(pos, start_search), 2);
    }

    FOR (i = start_search; i < -5; i++)
    {
        L_tmp = L_msu(L_tmp, pt_exc[i], pt_exc[i]);
        L_tmp = L_mac(L_tmp, pt_exc[i + 5], pt_exc[i + 5]);

        IF (L_sub(L_tmp, L_min_energy) < 0)
        {
            L_min_energy = L_add(0,L_tmp); /* sets to 'L_tmp' in 1 clock */
            min_pos[0] = add(add(pos, i), 2);
        }
    }

    FOR (j = 1; j < nb_min; j++)
    {
        min_pos[j] = sub(min_pos[j-1], Old_pitch);
        /* If the first minimum is in the past, forget this minimum */
        IF (min_pos[j] < 0)
        {
            min_pos[j] = -10000;
            move16();
            nb_min = sub(nb_min, 1);
        }
    }

    IF(sub(nb_min,16)>0)  /* inv_sqi & sqi are built for a maximum of nb_min-2 = 14 values*/
    {
        return;
    }
    /*--------------------------------------------------------------------
     * Determine the number of samples to be added or removed at each pitch
     * cycle whereby less samples are added/removed at the beginning and
     * more towards the end of the frame
     * --------------------------------------------------------------------*/
    test();
    IF (sub(nb_min, 1) == 0 || sub(abs_s(point_to_remove), 1) == 0)
    {
        nb_min = 1;
        move16();
        points_by_pos[0] = abs_s(point_to_remove);
    }
    ELSE
    {
        /* First position */
        /* fact = (float)fabs(point_to_remove) / sqi[nb_min-2]; (nb_min*nb_min) */
        fact = mult_r(shl(abs_s(point_to_remove), 7), inv_sqi[nb_min - 2]); /*Q7 */
        points_by_pos[0] = mult_r(fact, 256); /*Q7 */
        total_point = points_by_pos[0];
        move16();

        FOR (i = 2; i <= nb_min; i++)
        {
            /* points_by_pos[i-1] = (Word16)(fact*(sqi[i-2]) - total_point+0.5) */
            points_by_pos[i - 1] = sub(shr(extract_l(L_mac0(64L, fact, sqi[i - 2])), 7), total_point);
            total_point = add(total_point, points_by_pos[i-1]);

            /* ensure a constant increase */
            IF (sub(points_by_pos[i-1], points_by_pos[i-2]) < 0)
            {
                tmp16 = points_by_pos[i-2];
                move16();
                points_by_pos[i-2] = points_by_pos[i-1];
                move16();
                points_by_pos[i-1] = tmp16;
                move16();
            }
        }
    }
    /* --------------------------------------------------------------------
     * Sample deletion or insertion is performed in minimum energy regions.
     * At the end of this section the last maximum pulse in the concealed
     * excitation is forced to align to the actual maximum pulse position
     * at the end of the frame which is transmitted in the future frame.
     * --------------------------------------------------------------------*/
    if (point_to_remove > 0)
    {
        point_to_add = point_to_remove;
        move16();
    }

    pt_exc = exc_tmp;
    move16();
    pt_exc1 = exc;
    move16();

    i = 0;
    move16();
    pt_pos = min_pos + sub(nb_min, 1);

    IF (point_to_add > 0)/* Samples insertion */
    {
        remaining_len = L_frame;
        move16();
        FOR (i = 0; i < nb_min; i++)
        {
            /* Copy section */
            /* Compute len to copy */
            tmp_len = *pt_pos;
            move16();
            IF (i != 0)
            {
                /* Compute len to copy */
                tmp_len = sub(sub(*pt_pos, *(pt_pos+1)), points_by_pos[i-1]);
            }
            /*Copy section */
            Copy(pt_exc1, pt_exc, tmp_len);
            remaining_len = sub(remaining_len, tmp_len);
            pt_exc1 += tmp_len;
            move16();
            pt_exc += tmp_len;
            move16();

            /* Find point to add and Add points */
            tmp16 = mult_r(*pt_exc1, -1638);
            FOR (j = 0; j < points_by_pos[i]; j++)
            {
                *pt_exc++ = tmp16;
                move16();/* repeat last point */
                tmp16 = negate(tmp16);
            }

            remaining_len = sub(remaining_len, points_by_pos[i]);
            pt_pos--;
        }
        /* Copy remaining data */
        Copy(pt_exc1, pt_exc, remaining_len);
        /* Update true excitation vector */
        Copy(exc_tmp, exc, L_frame);
    }
    ELSE /* Samples deletion */
    {
        remaining_len = L_frame;
        move16();

        FOR (i = 0; i < nb_min; i++)
        {
            /* Compute len to copy */
            tmp_len = *pt_pos;
            move16();
            IF (i != 0)
            {
                /* Compute len to copy */
                tmp_len = sub(sub(*pt_pos, *(pt_pos+1)), points_by_pos[i-1]);
            }
            Copy(pt_exc1, pt_exc, tmp_len);
            remaining_len = sub(remaining_len, tmp_len);
            pt_exc1 += tmp_len;
            move16();
            pt_exc += tmp_len;
            move16();
            /* Remove points */
            FOR (j = 0; j < points_by_pos[i]; j++)
            {
                pt_exc1++;
            }
            pt_pos--;
        }
        /* Copy remaining data */
        Copy(pt_exc1, pt_exc, remaining_len);
        /* Update true excitation vector */
        Copy(exc_tmp, exc, L_frame);
    }


}
