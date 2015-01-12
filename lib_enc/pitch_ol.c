/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <assert.h>
#include "prot_fx.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"

/*-----------------------------------------------------------------*
 * Local Constants
 *-----------------------------------------------------------------*/
#define PIT_MIN2     20      /* pit_min for pitch tracking                                                  */
#define PIT_MIN_1    44      /* for second pitch track */
#define PIT_MIN2_1   24

#define THR_relE  -2816   /* -11 (Q8) */

#define THRES0     4792   /* Threshold to favor smaller pitch lags; 1.17 (Q12) */
#define DELTA0     2      /* multiples' search range initial    */
#define STEP       1      /* multiples' search range increment  */

#define THRES1     13107  /* Threshold to favor pitch lags coherence for neighbours; 0.4 (Q15) */
#define DELTA_COH  14     /* Maximum pitch lags difference for neighbours to be considered as coherent */
#define THRES3     22938  /* Threshold to favor pitch lags coherence with previous frames; 0.7 (Q15) */

#define CORR_TH0   13107  /* Noise threshold for past frame correlations; 0.4 (Q15) */
#define CORR_TH1   16384  /* Noise threshold for past frame correlations; 0.5 (Q15) */

#define LEN_X     ((PIT_MAX/OPL_DECIM)-(PIT_MIN2/OPL_DECIM)+1)  /* Correlation buffer length */
#define COH_FAC     5734  /* Factor for measuring the pitch coherence; 1.4 (Q12) */

#define NSUBSECT 7
#define NSECT 4
#define NHFR  3
#define L_FIR_PO  5
#define L_MEM  (L_FIR_PO-2)


/*-----------------------------------------------------------------*
 * Local function prototypes
 *-----------------------------------------------------------------*/
static void LP_Decim2_Copy(
    const Word16 x[],
    Word16 y[],
    Word16 l,
    Word16 mem[]
);

static void pitch_neighbour_fx(
    Word16 sect0,
    Word16 pitch_tmp[],
    Word16 pitch[3][2*NSECT],
    Word16 corr_tmp[],
    Word16 corr[3][2*NSECT],
    Word16 thres1[2*NHFR],
    Word16 ind_tmp[2*NHFR]
);

static void find_mult_fx(
    Word16 *fac,
    Word16 pitch0,
    Word16 pitch1,
    Word16 pit_max0,
    Word16 *corr,
    Word16 *old_pitch,
    Word16 *old_corr,
    Word16 delta,
    Word16 step
);

static Word16 pitch_coherence_fx(
    Word16 pitch0,
    Word16 pitch1,
    Word16 fac_max,
    Word16 diff_max
);

static Word32 Dot_product12_OL(
    Word16 *sum1,
    const Word16 x[],
    const Word16 y[],
    const Word16 lg,
    const Word16 lg2,
    Word16 *exp,
    Word16 *exp2
);

static Word32 Dot_product12_OL_back(
    Word16 *sum1,
    const Word16 x[],
    const Word16 y[],
    const Word16 lg,
    const Word16 lg2,
    Word16 *exp,
    Word16 *exp2
);


/*-----------------------------------------------------------------*
 * pitch_ol_init()
 *
 * Open loop pitch variable initialization
 *-----------------------------------------------------------------*/
void pitch_ol_init_fx(
    Word16 *old_thres,  /* o  : threshold for reinforcement of past pitch influence */
    Word16 *old_pitch,  /* o  : pitch  of the 2nd half-frame of previous frame      */
    Word16 *delta_pit,  /* o  : pitch evolution extrapolation                       */
    Word16 *old_corr    /* o  : correlation                                         */
)
{
    *old_thres = 0;
    move16();
    *old_pitch = 0;
    move16();
    *delta_pit = 0;
    move16();
    *old_corr  = 0;
    move16();
}


/*==================================================================================*/
/* FUNCTION : pitch_ol_fx()                                       */
/*----------------------------------------------------------------------------------*/
/* PURPOSE :
 * Compute the open loop pitch lag.
 *
 * The pitch lag search is divided into two sets.
 * Each set is divided into three sections.
 * Each section cannot have a pitch multiple.
 * We find a maximum for each section.
 * We compare the maxima of each section.
 *
 *                               1st set              2nd set
 * 1st section: lag delay =  115 down to 62  and  115 down to 78
 * 2nd section: lag delay =   61 down to 32  and   77 down to 41
 * 3rd section: lag delay =   31 down to 17  and   40 down to 22
 * 4th section: lag delay =   16 down to 10  and   21 down to 12
 *
 * As there is a margin between section overlaps, especially for
 * longer delays, this section selection is more robust for not
 * to find multiples in the same section when pitch evolves rapidly.
 *
 * For each section, the length of the vectors to correlate is
 * greater/equal to the longest pitch delay.                                        */
/*----------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :                                    */
/* _ (Word16[]) old_pitch    : OL pitch of the 2nd half-frame of the last frame Q0 */
/* _ (Word16[]) old_corr_fx    : correlation                                  Q15 */
/* _ (Word16[]) corr_shift_fx : normalized correlation correction               Q15 */
/* _ (Word16[]) old_thres_fx  : maximum correlation weighting with respect          */
/*                              to past frame pitch                             Q15 */
/* _ (Word16[]) delta_pit    : old pitch extrapolation correction               Q0 */
/* _ (Word16[]) st_old_wsp2_fx: weighted speech memory                         qwsp */
/* _ (Word16[]) wsp_fx      : weighted speech for current frame & look-ahead qwsp */
/* _ (Word16[]) mem_decim2_fx : wsp decimation filter memory                   qwsp */
/* _ (Word16[]) relE_fx      : relative frame energy                            Q8 */
/* _ (Word16[]) L_look      : look-ahead                                       Q0 */
/* _ (Word16[]) Opt_SC_VBR    : SC-VBR flag                                      Q0 */
/* _ (Word16*) qwsp        : wsp & filter memory Qformat                */
/*----------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                  */
/* _ (Word16[]) pitch        : open loop pitch lag for each half-frame          Q0 */
/* _ (Word16[]) T_op          : open loop pitch lag for each half-frm for quant  Q0 */
/* _ (Word16[]) voicing_fx    : max normalized correlation for each half-frame  QIn */
/* _ (Word16[]) old_pitch    : OL pitch of the 2nd half-frame of the last frame Q0 */
/* _ (Word16[]) old_corr_fx    : correlation                                  Q15 */
/* _ (Word16[]) old_thres_fx  : maximum correlation weighting with respect          */
/*                              to past frame pitch                             Q15 */
/* _ (Word16[]) delta_pit    : old pitch extrapolation correction               Q0 */
/* _ (Word16[]) st_old_wsp2_fx: weighted speech memory                         qwsp */
/* _ (Word16[]) mem_decim2_fx : wsp decimation filter memory                   qwsp */
/*----------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                  */
/* _ None                                        */
/*==================================================================================*/

void pitch_ol_fx(
    Word16 pitch[3],        /* o  : open loop pitch lag for each half-frame in range [29,231]      Q0  */
    Word16 voicing[3],      /* o  : maximum normalized correlation for each half-frame in [0,1.0[  Q15 */
    Word16 *old_pitch,      /* i/o: pitch of the 2nd half-frame of previous frame (i.e. pitch[1])  Q0  */
    Word16 *old_corr,       /* i/o: correlation of old_pitch (i.e. voicing[1] or corr_mean)        Q15 */
    Word16 corr_shift,      /* i  : normalized correlation correction                              Q15 */
    Word16 *old_thres,      /* i/o: maximum correlation weighting with respect to past frame pitch Q15 */
    Word16 *delta_pit,      /* i/o: old pitch extrapolation correction in range [-14,+14]          Q0  */
    Word16 *st_old_wsp2,    /* i/o: weighted speech memory                                         qwsp */
    const Word16 *wsp,            /* i  : weighted speech for current frame and look-ahead               qwsp */
    Word16 mem_decim2[3],   /* i/o: wsp decimation filter memory                                   qwsp */
    const Word16 relE,            /* i  : relative frame energy                                          Q8  */
    const Word16 last_class,      /* i  : frame classification of last frame                                 */
    const Word16 bwidth,          /* i  : bandwidth                                                          */
    const Word16 Opt_SC_VBR       /* i  : SC-VBR flag                                                        */
)
{
    Word16 ftmp, old_wsp2[(L_WSP-L_INTERPOL)/OPL_DECIM], *wsp2;
    Word16 tmp_mem[3];

    Word16 scale1[2*DELTA_COH-1];
    Word16 scaled_buf[2*LEN_X + 3*(DELTA_COH-1)];
    Word16 scaled_buf_exp[2*LEN_X + 3*(DELTA_COH-1)], exp_sect[8], exp_sect1[8], exp_sect0;
    Word16 cor_buf[2*LEN_X];
    Word16 *pt_exp1, *pt_exp2, *pt_exp3, *pt_exp4;
    Word16 *pt1, *pt2, *pt3, *pt4, *pt5, *pt6;
    Word16 *pt_cor0, *pt_cor1, *pt_cor2, *pt_cor3, *pt_cor4, *pt_cor5, *pt_cor6;
    Word16 thres1[6];
    Word16 diff, cnt, ind, ind1, offset, offset1, offset_la, offset_la1, coh_flag, coh_flag1;
    Word16 ind_corX, ind1_corX;

    Word16 i, j, k, m, pit_min, pit_min1, sect0, subsect0, add_sect0, sub_sect0, old_tmp, old_tmp1, len_x, len_x1;
    Word16 len_temp;
    Word16 pitchX[NHFR][2*NSECT], pitch_tmp[2*NHFR], ind_tmp[2*NHFR], tmp_buf[NHFR+1];

    Word16 enr0[NSECT], enr0_exp[NSECT], enr0_1[NSECT], enr0_1_exp[NSECT], enr1, enr1_exp, enr2_exp;
    Word32 enr, enr2, Ltmp;
    Word16 fac, tmp16, tmp16_2;
    Word16 qCorX, qScaledX;
    Word16 scaledX[NHFR][2*NSECT], corX[NHFR][2*NSECT], cor_tmp[2*NHFR], cor_mean;
    const Word16 *len, *len1, *sublen, *sublen1, *pit_max, *sec_length, *sec_length1;

    Word16 pit_min_coding;

    /*--------------------------------------------------------------*
     * Initialization
     *--------------------------------------------------------------*/
    len = len_12k8_fx;
    len1 = len1_12k8_fx;
    sublen =  sublen_12k8_fx;
    sublen1 = sublen1_12k8_fx;
    pit_max = pit_max_12k8_fx;
    sec_length = sec_length_12k8_fx;
    sec_length1 = sec_length1_12k8_fx;

    test();
    if ((sub(last_class,VOICED_TRANSITION) < 0) && (sub(bwidth,NB) != 0))
    {
        /*reset last pitch reinforcement in case of unvoiced or transitions: it avoids some pitch doublings*/
        *old_thres = 0;
        move16();
    }

    pit_min_coding = PIT_MIN_EXTEND;
    move16();
    test();
    test();
    test();
    test();
    IF ( ( (sub(bwidth,NB) != 0) && (sub(*old_pitch,PIT_MIN) > 0 ) ) ||
         ( (sub(bwidth,NB) == 0) && ( (sub(*old_pitch,PIT_MIN2_1) > 0) || (sub(*old_thres,3277) < 0) ) ) ) /* 0.1 inQ15*/
    {
        pit_min = PIT_MIN/OPL_DECIM;
        move16();
        pit_min1= PIT_MIN_1/OPL_DECIM;
        move16();
        subsect0 = 2;
        move16();
        sect0 = 1;
        move16();
    }
    ELSE
    {
        pit_min = PIT_MIN2/OPL_DECIM;
        move16();
        pit_min1= PIT_MIN2_1/OPL_DECIM;
        move16();
        subsect0 = 0;
        move16();
        sect0 = 0;
        move16();
    }

    len_x = (PIT_MAX/OPL_DECIM - pit_min + 1);
    move16();
    len_x1= (PIT_MAX/OPL_DECIM - pit_min1 + 1);
    move16();

    /*--------------------------------------------------------------*
     * Find decimated weighted speech
     * Update wsp buffer with the memory
     * decimation of wsp[] to search pitch in LF and to reduce complexity
     * Extend the decimation of wsp to the end of the speech buffer
     * Update wsp memory
     *--------------------------------------------------------------*/
    Copy(st_old_wsp2, old_wsp2, (L_WSP_MEM-L_INTERPOL)/OPL_DECIM);
    wsp2 = old_wsp2 + ((L_WSP_MEM-L_INTERPOL)/OPL_DECIM);

    LP_Decim2_Copy(wsp, wsp2, L_FRAME, mem_decim2);

    /* Avoid uninitialized memory access */
    set16_fx(wsp2 + L_FRAME/2, 0, sizeof(old_wsp2)/sizeof(Word16)-((L_WSP_MEM-L_INTERPOL)/OPL_DECIM)-L_FRAME/2);
    tmp_mem[0] = mem_decim2[0];
    move16();
    tmp_mem[1] = mem_decim2[1];
    move16();
    tmp_mem[2] = mem_decim2[2];
    move16();

    LP_Decim2_Copy(&wsp[L_FRAME], &wsp2[shr(L_FRAME,1)], L_LOOK_12k8, tmp_mem);      /* shr() used instead of division by OPL_DECIM*/

    Copy(&old_wsp2[shr(L_FRAME,1)], st_old_wsp2, (L_WSP_MEM-L_INTERPOL)/OPL_DECIM);

    /*-----------------------------------------------------------------*
     * Attenuate the correlation correction factor due to noise.
     * Reset correlation buffer outside the useful range.
     * Find the scaling functions for immediate neigbours and
     * further ones.
     *-----------------------------------------------------------------*/

    corr_shift = shr(corr_shift, 1);

    set16_fx( scaled_buf, 0, DELTA_COH-1 );
    set16_fx( scaled_buf + (DELTA_COH-1) + len_x, 0, DELTA_COH-1 );
    set16_fx( scaled_buf + 2*(DELTA_COH-1) + len_x + len_x1, 0, DELTA_COH-1 );
    set16_fx( scaled_buf_exp, 0, len_x + len_x1 + 3*(DELTA_COH-1));

    pt1 = scale1 + DELTA_COH-1;
    pt2 = pt1;
    tmp16 = mult(negate(*old_thres), MAX_16/DELTA_COH);
    k = *old_thres;
    move16();
    FOR (i=0; i < DELTA_COH; i++)
    {
        /*
         * *pt1 = ( -(*old_thres)/DELTA_COH * i + *old_thres+1.0f );
         * To keep Q15 values, the following code does not add 1 to the result.
         * A scaling factor must be applied accordingly (see next use of scale1)
         */
        *pt1 = k;
        move16();
        k = add(k, tmp16);
        *pt2-- = *pt1++;
        move16();
    }

    /*-----------------------------------------------------------------------------*
     * Estimate the new pitch by extrapolating the old pitch value for 2 half-frames
     *-----------------------------------------------------------------------------*/
    old_tmp  = add(*old_pitch, *delta_pit);
    old_tmp  = s_min(old_tmp, PIT_MAX/OPL_DECIM);
    old_tmp  = s_max(old_tmp, pit_min);
    old_tmp1 = add(old_tmp, *delta_pit);
    old_tmp1 = s_min(old_tmp1, PIT_MAX/OPL_DECIM);
    old_tmp1 = s_max(old_tmp1, pit_min);

    /*-----------------------------------------------------------------*
     * Loop for all three half-frames (current frame + look-ahead)
     *-----------------------------------------------------------------*/
    pt_cor0 = scaled_buf + DELTA_COH-1;

    pt_cor2 = pt_cor0 - pit_min + old_tmp;
    pt_cor4 = pt_cor0 - pit_min1 + old_tmp + (DELTA_COH-1) + len_x;

    FOR( i=0; i<NHFR; i++ ) /* i = 0, 1, 2 */
    {
        pt1 = wsp2 + i*2*(L_SUBFR/OPL_DECIM);   /* *pt1 -> Q12 */
        pt2 = pt1 - pit_min;                    /* *pt2 -> Q12 */
        pt4 = pt1 - pit_min1;                   /* *pt4 -> Q12 */

        enr = L_deposit_l(1);

        pt_cor1 = pt_cor0;
        pt_cor3 = pt_cor0 + (DELTA_COH-1) + len_x;

        pt_exp1 = scaled_buf_exp + DELTA_COH-1;
        pt_exp2 = pt_exp1;
        pt_exp3 = scaled_buf_exp + 2*(DELTA_COH-1) + len_x;
        pt_exp4 = pt_exp3;

        IF( sub(i,NHFR-1) < 0 )    /* First two half-frames (current frame) */
        {
            pt3 = pt1;
            pt5 = pt1;

            FOR( j = sect0; j < NSECT; j++ )    /* loop for each section */
            {
                /*-----------------------------------------------------------------*
                 * Find fixed vector energy
                 *-----------------------------------------------------------------*/

                /* 1st set */
                k = (Word16)(pt1 - pt3);
                move16();

                FOR (k = add(k, len[j]); k > 0; k--)
                {
                    enr = L_mac0(enr, *pt3, *pt3);
                    pt3++;
                }
                /* keep Q15 normalized result */
                cnt = norm_l(enr);
                enr0[j] = extract_h(L_shl(enr, cnt));
                enr0_exp[j] = sub(30, cnt);
                move16();

                /* Reduce complexity (length of 'enr2' section is equal or larger than 'enr') */
                pt5 = pt3;
                enr2 = L_add(enr,0); /* sets to 'enr' in 1 clock */

                /* 2nd set */
                k = (Word16)(pt1 - pt5);
                move16();

                FOR (k = add(k, len1[j]); k > 0; k--)
                {
                    enr2 = L_mac0(enr2, *pt5, *pt5);
                    pt5++;
                }
                cnt = norm_l(enr2);
                enr0_1[j] = extract_h(L_shl(enr2, cnt));
                enr0_1_exp[j] = sub(30, cnt);
                move16();
            }

            /*----------------------------------------------------------*
             * Find correlation for the non-overlapping pitch lag values
             *----------------------------------------------------------*/
            exp_sect[subsect0] = 0;
            move16();
            pt_cor5 = pt_cor1;
            pt_cor6 = pt_cor3;

            tmp16 = exp_sect[subsect0];
            move16();

            k = (Word16)(pt2 - pt1 + pit_max[subsect0]);

            IF (k >= 0)
            {
                len_temp = sublen[0];
                move16();

                FOR (; k >= 0; k--)
                {
                    /* Keep Q15 normalized result */
                    /* shr by 1 to make room for scaling in the neighbourhood of the extrapolated pitch */
                    /* Update exponent to reflect shr by 1 */
                    *pt_cor1 = extract_h(L_shr(Dot_product12(pt1, pt2--, len_temp, pt_exp1), 1));

                    /* save the biggest exponent */
                    tmp16 = s_max(tmp16, *pt_exp1);

                    pt_cor1++;
                    pt_exp1++;
                }
            }
            exp_sect[subsect0] = tmp16;
            move16();

            /*----------------------------------------------------------*
             * For each subsection, find the correlation
             *----------------------------------------------------------*/
            FOR (j = subsect0; j < NSUBSECT; j++)
            {
                len_temp  = sublen[j];
                move16();

                k = (Word16)(pt2 - pt1);
                move16();
                k = add(k,pit_max[j+1]);
                exp_sect[j+1] = 0;
                move16();
                exp_sect1[j]  = 0;
                move16();

                IF (k >= 0)
                {
                    ind  = exp_sect[j+1];
                    move16();
                    ind1 = exp_sect1[j];
                    move16();

                    FOR (; k >= 0; k--)
                    {
                        /* Keep Q15 normalized result */
                        /* shr by 1 to make room for scaling in the neighbourhood of the extrapolated pitch */
                        /* Update exponent to reflect shr by 1 (done in Dot_product12_OL() for pt_cor3/pt_exp3) */
                        *pt_cor1 = extract_h(L_shr(Dot_product12_OL(pt_cor3, pt1, pt2--, sublen[j], sublen1[j], pt_exp1, pt_exp3), 1));
                        /* The line above replaces:
                         * *pt_cor1 = shr(extract_h(Dot_product12(pt1, pt2, Sublen[j], pt_exp1)),1); move16();
                         * *pt_cor3 = shr(extract_h(Dot_product12(pt1, pt2--, Sublen1[j+i*7], pt_exp3)),1); move16();
                         */

                        /* save the biggest exponent */
                        ind  = s_max(ind, *pt_exp1);
                        ind1 = s_max(ind1, *pt_exp3);

                        pt_cor1++;
                        pt_exp1++;
                        pt_cor3++;
                        pt_exp3++;
                    }
                    exp_sect[j+1] = ind;
                    move16();
                    exp_sect1[j]  = ind1;
                    move16();
                }   /* IF (k >= 0) */
            }       /* FOR (j = subsect0; ... */
        }
        ELSE                       /* 3rd half-frame (look-ahead) */
        {
            pt6 = pt1 + L_LOOK_12k8/OPL_DECIM - 1;
            pt3 = pt6;
            pt5 = pt6;

            /*-----------------------------------------------------------------*
             * For each section in both sets, find fixed vector energy
             *-----------------------------------------------------------------*/

            FOR( j = sect0; j < NSECT; j++ )    /* loop for each section */
            {
                /* 1st set */
                k = (Word16)(pt3 - pt6);
                move16();

                FOR (k = add(k, len[j]); k > 0; k--)
                {
                    enr = L_mac0(enr, *pt3, *pt3);
                    pt3--;
                }

                cnt = norm_l(enr);
                enr0[j] = extract_h(L_shl(enr, cnt));          /*qwsp+cnt-16*/
                enr0_exp[j] = sub(30, cnt);
                move16();

                /* Reduce complexity (length of 'enr2' section is equal or larger than 'enr') */
                pt5 = pt3;
                enr2 = L_add(enr,0);

                /* 2nd set */
                k = (Word16)(pt5 - pt6);
                move16();

                FOR (k = add(k, len1[j]); k > 0; k--)
                {
                    enr2 = L_mac0(enr2, *pt5, *pt5);
                    pt5--;
                }

                cnt = norm_l(enr2);
                enr0_1[j] = extract_h(L_shl(enr2, cnt));       /*qwsp+cnt-16*/
                enr0_1_exp[j] = sub(30, cnt);
                move16();
            }

            /* Set pointers */
            IF( sect0 != 0 )
            {
                pt2 = pt6 - add(pit_max[1],1);
                k = sub(pit_max[2],pit_max[1]);
                move16();
            }
            ELSE
            {
                pt2 = pt6 - pit_min;
                k = 2;
                move16();
            }

            /*-----------------------------------------------------------------*
             * Find correlation for the non-overlapping pitch lag values
             *-----------------------------------------------------------------*/
            exp_sect[subsect0] = 0;
            move16();
            pt_cor5 = pt_cor1;
            pt_cor6 = pt_cor3;

            tmp16 = exp_sect[subsect0];
            move16();

            IF (k > 0)
            {
                len_temp = sublen[0];
                move16();

                FOR ( ; k > 0; k-- )
                {
                    /* Following lines are equivalent of Dot_product12() but with a backward incrementing */
                    Ltmp = L_deposit_l(1);
                    FOR( m = 0; m < len_temp; m++ )
                    {
                        Ltmp = L_mac(Ltmp, pt6[-m], pt2[-m]);
                    }

                    /* Normalize acc in Q31 */
                    tmp16_2 = norm_l(Ltmp);
                    Ltmp = L_shl(Ltmp, tmp16_2);
                    *pt_exp1 = sub(30, tmp16_2);
                    move16();  /* exponent = 0..30 */

                    /* Save result */
                    *pt_cor1 = extract_h(L_shr(Ltmp,1));

                    /* Save the biggest exponent */
                    tmp16 = s_max(tmp16, *pt_exp1);

                    pt_cor1++;
                    pt_exp1++;
                    pt2--;
                }
                exp_sect[subsect0] = tmp16;
                move16();
            }

            /*-----------------------------------------------------------------*
             * For each subsection, find the correlation (overlapping pitch lag values)
             *-----------------------------------------------------------------*/

            FOR( j = subsect0; j < NSUBSECT; j++ )
            {
                exp_sect[j+1] = 0;
                move16();
                exp_sect1[j] = 0;
                move16();

                ind  = exp_sect[j+1];
                move16();
                ind1 = exp_sect1[j];
                move16();

                k = sub(pit_max[j+1], pit_max[j]);

                FOR( ; k > 0; k-- )
                {
                    *pt_cor1 = extract_h(L_shr(Dot_product12_OL_back(pt_cor3, pt6, pt2--, sublen[j], sublen1[j], pt_exp1, pt_exp3), 1));

                    /* Save the biggest exponent */
                    ind  = s_max(ind, *pt_exp1);
                    ind1 = s_max(ind1, *pt_exp3);

                    pt_cor1++;
                    pt_exp1++;
                    pt_cor3++;
                    pt_exp3++;
                }
                exp_sect[j+1] = ind;
                move16();
                exp_sect1[j]  = ind1;
                move16();
            }
        }                          /* 3rd half-frame (look-ahead) */

        /* Scale all values in each section to the same exponent for upcoming Find_max() */
        offset  = 0;
        move16();
        offset1 = 0;
        move16();
        exp_sect1[7] = 0; /* padding */                              move16();
        FOR (j = sect0; j < NSECT; j++)
        {
            exp_sect0 = s_max(exp_sect[j*2], exp_sect[j*2+1]);

            /* scaling of exp for track 1 */
            offset = add(offset, sec_length[j]);
            k = (Word16)(pt_cor0 - pt_cor5);
            move16();
            FOR (k = add(k, offset); k > 0; k--)
            {
                cnt = sub(exp_sect0, *pt_exp2);
                tmp16 = s_min(15, cnt);
                if (cnt > 0)
                {
                    tmp16 = shr(*pt_cor5, tmp16);
                }
                if (cnt > 0)
                {
                    *pt_cor5 = tmp16;
                    move16();
                }
                *pt_exp2 = s_max(*pt_exp2, exp_sect0);
                move16();
                pt_cor5++;
                pt_exp2++;
            }

            exp_sect0 = s_max(exp_sect1[j*2], exp_sect1[j*2+1]);

            /* scaling of exp for track 2 */
            offset1 = add(offset1, sec_length1[j]);
            k = (Word16)(pt_cor0 - pt_cor6 + (DELTA_COH-1));
            move16();
            k = add(k, len_x);
            FOR (k = add(k, offset1); k > 0; k--)
            {
                cnt = sub(exp_sect0, *pt_exp4);
                tmp16 = s_min(15, cnt);
                if (cnt > 0)
                {
                    tmp16 = shr(*pt_cor6, tmp16);
                }
                if (cnt > 0)
                {
                    *pt_cor6 = tmp16;
                    move16();
                }
                *pt_exp4 = s_max(*pt_exp4, exp_sect0);
                move16();
                pt_cor6++;
                pt_exp4++;
            }
        }   /* FOR (j = sect0; ... */

        Copy( pt_cor0, cor_buf, len_x ); /* Save unscaled correlation vector  */
        Copy( pt_cor0+(DELTA_COH-1)+len_x, cor_buf+len_x, len_x1 ) ;

        /*-----------------------------------------------------------------*
         * Scale correlation function in the neighbourhood of
         * the extrapolated pitch
         *-----------------------------------------------------------------*/
        pt_cor1 = pt_cor2 - (DELTA_COH-1);
        pt_cor3 = pt_cor4 - (DELTA_COH-1);
        pt2 = scale1;

        FOR( k=0 ; k < 2*DELTA_COH-1 ; k++ )
        {
            /* all Q15 here */
            *pt_cor1 = add(*pt_cor1, mult(*pt_cor1, *pt2));
            move16();
            *pt_cor3 = add(*pt_cor3, mult(*pt_cor3, *pt2++));
            move16();

            pt_cor1++;
            pt_cor3++;
        }

        /* Update for next half-frame & look-ahead */
        pt_cor2 = pt_cor0 - pit_min + old_tmp1;
        pt_cor4 = pt_cor0 - pit_min1 + old_tmp1 + (DELTA_COH-1) + len_x;

        /*-----------------------------------------------------------------*
         * For each section, find maximum correlation and compute
         * normalized correlation
         *-----------------------------------------------------------------*/

        pt_cor1 = pt_cor0;
        pt_exp1 = scaled_buf_exp + DELTA_COH-1;
        offset = 0;
        move16();
        pt_cor3 = pt_cor0 + (DELTA_COH-1) + len_x;
        pt_exp3 = scaled_buf_exp + 2*(DELTA_COH-1) + len_x;
        offset1 = 0;
        move16();

        FOR( j=sect0; j < NSECT; j++ )       /* loop for each section */
        {
            /* 1st set */
            offset_la = 0;
            move16();
            if( sub(i,2)==0 )
            {
                offset_la = sub(L_LOOK_12k8/OPL_DECIM,len[j]);
            }

            /* 2nd set */
            offset_la1 = 0;
            move16();
            if( sub(i,2)==0 )
            {
                offset_la1 = sub(L_LOOK_12k8/OPL_DECIM,len1[j]);
            }

            /* 1st set of candidates */
            ind = add(maximum_fx( pt_cor1, sec_length[j], &ftmp ), offset);
            pitchX[i][j] = add(ind, pit_min);
            move16();
            pt2 = pt1 - pitchX[i][j] +/*-*/ offset_la;                    /* selected moving vector  */

            enr1_exp = 0;
            move16();
            enr1 = add(extract_h(dotp_fx( pt2, pt2, len[j], &enr1_exp)), 1);

            enr2 = L_mult(enr0[j], enr1);
            enr2_exp = norm_l(enr2);
            enr2 = L_shl(enr2, enr2_exp);
            enr2_exp = sub(31,add(sub(28,add(enr0_exp[j],enr1_exp)),add(enr2_exp,1)));

            enr2 = Isqrt_lc(enr2, &enr2_exp);                         /* 1/sqrt(energy) */ /*31-enr2_exp*/
            enr1_exp = norm_l(enr2);
            enr1 = extract_h(L_shl(enr2, enr1_exp));                           /*31-enr2_exp+enr1_exp-16*/
            enr1_exp = sub(enr2_exp, enr1_exp);                                /*15-enr1_exp*/

            Ltmp = L_mult0(cor_buf[ind], enr1);
            qCorX = add(sub(15,enr1_exp), sub(14,pt_exp1[ind]));
            corX[i][j] = extract_h(L_shr(Ltmp, sub(qCorX,31)));
            qCorX = 31;
            move16();

            Ltmp = L_mult0(pt_cor0[ind], enr1);
            qScaledX = add(sub(15,enr1_exp),sub(14,pt_exp1[ind]));
            scaledX[i][j] = round_fx(L_shl(Ltmp, sub(16+12,qScaledX)));
            qScaledX =12;
            move16();

            pt_cor1 += sec_length[j];
            move16();
            offset = add(offset,sec_length[j]);

            /* 2nd set of candidates */
            ind1 = add(maximum_fx( pt_cor3, sec_length1[j], &ftmp ), offset1);
            pitchX[i][j+NSECT] = add(ind1, pit_min1);
            move16();
            pt4 = pt1 - pitchX[i][j+NSECT] +/*-*/ offset_la1;
            move16();  /* selected moving vector  */
            enr1_exp = 0;
            move16();
            enr1 = add(extract_h(dotp_fx( pt4, pt4, len1[j], &enr1_exp)), 1);

            enr2 = L_mult(enr0_1[j], enr1);
            enr2_exp = norm_l(enr2);
            enr2 = L_shl(enr2, enr2_exp);

            enr2_exp = sub(31,add(sub(28,add(enr0_1_exp[j],enr1_exp)),add(enr2_exp,1)));
            enr2 = Isqrt_lc(enr2, &enr2_exp);                         /* 1/sqrt(energy) */ /*31-enr2_exp*/
            enr1_exp = norm_l(enr2);
            enr1 = extract_h(L_shl(enr2, enr1_exp));                           /*31-enr2_exp+enr1_exp-16*/
            enr1_exp = sub(enr2_exp, enr1_exp);                                /*15-enr1_exp*/

            Ltmp = L_mult0(cor_buf[ind1+len_x], enr1);

            qCorX = add(sub(15,enr1_exp),sub(14,pt_exp3[ind1]));
            corX[i][j+NSECT] = extract_h(L_shr(Ltmp, qCorX-31));
            qCorX = 31;
            move16();

            Ltmp = L_mult0(pt_cor0[ind1+(DELTA_COH-1)+len_x], enr1);
            qScaledX = add(sub(15,enr1_exp),sub(14,pt_exp3[ind1]));
            scaledX[i][j+NSECT] = round_fx(L_shl(Ltmp, sub(16+12,qScaledX)));
            /*scaledX[i][j+NSECT] = saturate(L_shr(Ltmp, qScaledX-12));*/
            qScaledX =12;
            move16();

            pt_cor3 += sec_length1[j];
            move16();
            offset1 = add(offset1, sec_length1[j]);

        } /* FOR j < NSECT */
    } /* FOR i < NHFR */

    /*-----------------------------------------------------------------*
     * Favor a smaller delay if it happens that it has its multiple
     * in the longer-delay sections  (harmonics check)
     *-----------------------------------------------------------------*/

    FOR( i=0; i < 2; i++ )              /* loop for the 2 half-frames */
    {
        fac = THRES0;
        move16();
        find_mult_fx(&fac, pitchX[i][2], pitchX[i][3], pit_max[7], &scaledX[i][2], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 3rd section */
        find_mult_fx(&fac, pitchX[i][1], pitchX[i][2], pit_max[5], &scaledX[i][1], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 2nd section */
        test();
        IF((sect0==0) && sub(shl(pitchX[i][0],1),pit_min_coding)>=0)
        {
            find_mult_fx( &fac, pitchX[i][0], pitchX[i][1], pit_max[3], &scaledX[i][0], old_pitch, old_corr, DELTA0, STEP );  /* Multiples in 2nd section */
        }
        fac = THRES0;
        move16();
        find_mult_fx(&fac, pitchX[i][NSECT+2], pitchX[i][NSECT+3], pit_max[7], &scaledX[i][NSECT+2], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 3rd section */
        find_mult_fx(&fac, pitchX[i][NSECT+1], pitchX[i][NSECT+2], pit_max[6], &scaledX[i][NSECT+1], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 2nd section */
        test();
        IF((sect0==0) && sub(shl(pitchX[i][NSECT+0],1),pit_min_coding)>=0)
        {
            find_mult_fx( &fac, pitchX[i][NSECT+0], pitchX[i][NSECT+1], pit_max[4], &scaledX[i][NSECT+0], old_pitch, old_corr, DELTA0, STEP ); /* Multiples in 2nd section */
        }
    }

    fac = THRES0;
    move16(); /* the look-ahead */
    find_mult_fx(&fac, pitchX[i][2], pitchX[i][3], pit_max[7], &scaledX[i][2], old_pitch, old_corr, 2, 2); /* Multiples in 3rd section */
    find_mult_fx(&fac, pitchX[i][1], pitchX[i][2], pit_max[5], &scaledX[i][1], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 2nd section */
    test();
    IF((sect0==0) && sub(shl(pitchX[i][0],1),pit_min_coding)>=0)
    {
        find_mult_fx( &fac, pitchX[i][0], pitchX[i][1], pit_max[3], &scaledX[i][0], old_pitch, old_corr, DELTA0, STEP );  /* Multiples in 2nd section */
    }
    fac = THRES0;
    move16();
    find_mult_fx(&fac, pitchX[i][NSECT+2], pitchX[i][NSECT+3], pit_max[7], &scaledX[i][NSECT+2], old_pitch, old_corr, 2, 2); /* Multiples in 3rd section */
    find_mult_fx(&fac, pitchX[i][NSECT+1], pitchX[i][NSECT+2], pit_max[6], &scaledX[i][NSECT+1], old_pitch, old_corr, DELTA0, STEP); /* Multiples in 2nd section */
    test();
    IF((sect0==0) && sub(shl(pitchX[i][NSECT+0],1),pit_min_coding)>=0)
    {
        find_mult_fx( &fac, pitchX[i][NSECT+0], pitchX[i][NSECT+1], pit_max[4], &scaledX[i][NSECT+0], old_pitch, old_corr, DELTA0, STEP ); /* Multiples in 2nd section */                                                        /* Multiples in 2nd section */
    }

    /*-----------------------------------------------------------------*
     * Do 1st estimate for pitch values
     * Adjust the normalized correlation using estimated noise level
     * Compute the maximum scaling for the neighbour correlation
     * reinforcement
     *-----------------------------------------------------------------*/
    add_sect0 = add(NSECT,sect0);
    sub_sect0 = sub(NSECT,sect0);
    FOR (i=0; i < NHFR; i++)
    {
        /* 1st set of pitch candidates */
        ind = add(maximum_fx(scaledX[i]+sect0, sub_sect0, &ftmp), sect0);
        ind_tmp[i] = ind;
        move16();
        pitch_tmp[i] = pitchX[i][ind];
        move16();
        cor_tmp[i] = add(corX[i][ind], corr_shift);
        move16();

        /* Higher is the neighbour's correlation, higher is the weighting */
        /* operands are Q15, result is Q15 */
        thres1[i] = mult(THRES1, cor_tmp[i]);
        move16();

        /* 2nd set of pitch candidates */
        ind1 = add(maximum_fx(scaledX[i]+add_sect0, sub_sect0, &ftmp), add_sect0);
        ind_tmp[i+NHFR] = ind1;
        move16();
        pitch_tmp[i+NHFR] = pitchX[i][ind1];
        move16();
        cor_tmp[i+NHFR] = add(corX[i][ind1], corr_shift);
        move16();

        /* Higher is the neighbour's correlation, higher is the weighting */
        /* operands are Q15, result is Q15 */
        thres1[i+NHFR] = mult(THRES1, cor_tmp[i+NHFR]);
        move16();
    }
    /*-----------------------------------------------------------------*
     * Take into account previous and next pitch values of the present
     * frame and look-ahead. Choose the pitch lags and normalize
     * correlations for each half-frame & look-ahead
     *-----------------------------------------------------------------*/

    pitch_neighbour_fx(sect0, pitch_tmp, pitchX, cor_tmp, scaledX, thres1, ind_tmp);
    FOR( i = 0; i < NHFR; i++ )
    {
        ind = add(maximum_fx(scaledX[i]+sect0, sub_sect0, &ftmp), sect0);
        ind_corX = add(maximum_fx(corX[i]+sect0, sub_sect0, &ftmp), sect0);

        ind1 = add(maximum_fx(scaledX[i]+add_sect0, sub_sect0, &ftmp), add_sect0);
        ind1_corX = add(maximum_fx(corX[i]+add_sect0, sub_sect0, &ftmp), add_sect0);

        if ( sub(scaledX[i][ind1],scaledX[i][ind]) > 0 )
        {
            ind = ind1;
            move16();
        }
        test();
        if ( Opt_SC_VBR && L_sub(corX[i][ind1_corX],corX[i][ind_corX]) > 0 )
        {
            ind_corX = ind1_corX;
            move16();
        }
        test();
        test();
        test();
        IF (Opt_SC_VBR && (sub(mult(pitchX[i][ind], 13107 /*0.4 in Q15*/),pitchX[i][ind_corX]) < 0) &&
            (sub(mult(pitchX[i][ind], 19661 /*0.6 in Q15*/),pitchX[i][ind_corX]) > 0) &&
            (L_sub(corX[i][ind_corX],1932735283L/*0.9 in Q31*/)>=0))
        {
            pitch[i] = pitchX[i][ind_corX];
            move16();
            voicing[i] = corX[i][ind_corX];
            move16();
        }
        ELSE
        {
            pitch[i] = pitchX[i][ind];
            move16();
            voicing[i] = corX[i][ind];
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Increase the threshold for correlation reinforcement with
     * the past if correlation high and pitch stable
     *-----------------------------------------------------------------*/

    /* all Q15 here */
    /* cor_mean = 0.5f * (voicing[0] + voicing[1]) + corr_shift; */
    Ltmp = L_mult(voicing[0], 16384);
    Ltmp = L_mac(Ltmp, voicing[1], 16384);
    cor_mean = round_fx(L_add(Ltmp, corr_shift));

    /* pitch unstable in present frame or from previous frame or normalized correlation too low */
    coh_flag = pitch_coherence_fx(pitch[0], pitch[1], COH_FAC, DELTA_COH);
    move16();
    coh_flag1 = pitch_coherence_fx(pitch[0], *old_pitch, COH_FAC, DELTA_COH);
    move16();

    test();
    test();
    test();
    IF ((coh_flag == 0) || (coh_flag1 == 0) || (sub(cor_mean, CORR_TH0) < 0) || (sub(relE, THR_relE) < 0))
    {
        /* Reset the threshold */
        *old_thres = 0;
        move16();
    }
    ELSE
    {
        /* The threshold increase is directly dependent on normalized correlation */
        /* *old_thres += (0.16f * cor_mean); */
        *old_thres = round_fx(L_mac(L_deposit_h(*old_thres), 5243, cor_mean));
    }

    *old_thres = s_min(*old_thres, THRES3);
    move16();

    IF (sub(voicing[1], voicing[0]) > 0)
    {
        *old_corr = voicing[1];
        move16();
    }
    ELSE
    {
        *old_corr = cor_mean;
        move16();
    }

    /*-----------------------------------------------------------------*
     * Extrapolate the pitch value for the next frame by estimating
     * the pitch evolution. This value is added to the old_pitch
     * in the next frame and is then used when the normalized
     * correlation is reinforced by the past estimate
     *-----------------------------------------------------------------*/
    tmp_buf[0] = *old_pitch;
    move16();
    FOR( i = 0; i < NHFR; i++ )
    {
        tmp_buf[i+1] = pitch[i];
        move16();
    }

    *delta_pit = 0;
    move16();
    cnt = 0;
    move16();

    FOR ( i = 0; i < NHFR; i++ )
    {
        diff = sub(tmp_buf[i+1], tmp_buf[i]);
        move16();
        coh_flag = pitch_coherence_fx(tmp_buf[i], tmp_buf[i+1], COH_FAC, DELTA_COH);

        if (coh_flag != 0)
        {
            *delta_pit = add(*delta_pit, diff);
            move16();
        }
        cnt = add(cnt, coh_flag);
    }
    if (sub(cnt, 2) == 0)
    {
        /* *delta_pit /= 2; */
        *delta_pit = shr(*delta_pit, 1);
        move16();
    }
    IF (sub(cnt, 3) == 0)
    {
        k = *delta_pit;
        move16();
        /* *delta_pit /= 3; */
        if (k < 0)
        {
            *delta_pit = mult(*delta_pit, -32768);
            move16();
        }
        tmp16 = mult(*delta_pit, 10923);
        if (k < 0)
        {
            tmp16 = mult(tmp16, -32768);
        }
        *delta_pit = tmp16;
        move16();
    }

    /*--------------------------------------------------------------*
     * Update old pitch, upsample pitch,
     *--------------------------------------------------------------*/

    *old_pitch = pitch[1];
    move16();

    FOR ( i = 0; i < NHFR; i++ )
    {
        /* compensate decimation */
        pitch[i] = i_mult2(pitch[i], OPL_DECIM);
        move16();
    }

    return;
}


/*-----------------------------------------------------------------*
 * find_mult_fx
 *
 * Verifies whether max pitch delays in higher sections have multiples
 * in lower sections
 *-----------------------------------------------------------------*/
static void find_mult_fx(
    Word16 *fac,        /* i/o: correlation scaling factor Q12              */
    Word16 pitch0,      /* i  : pitch of max correlation in the c section   */
    Word16 pitch1,      /* i  : pitch of max correlation in the longer-delay section*/
    Word16 pit_max0,    /* i  : max pitch delay in the longer-delay section */
    Word16 *corr,       /* i/o: max correlation in the shorter-delay section Q12    */
    Word16 *old_pitch,  /* i  : pitch from previous frame                   */
    Word16 *old_corr,   /* i  : max correlation from previous frame         */
    Word16 delta,       /* i  : initial multiples search range              */
    Word16 step         /* i  : increment in range of multiples search      */
)
{
    Word16 pit_min;
    Word32 L_tmp;

    pit_min = shl(pitch0, 1);       /* double the higher section pitch */

    WHILE (sub(pit_min, add(pit_max0, delta)) <= 0)      /* check for section boundary */
    {
        IF (sub(abs_s(sub(pit_min, pitch1)), delta) <= 0)        /* if multiple in the allowed range */
        {
            L_tmp = L_shl(L_mult(*corr, *fac), 3);

            /* if ( *old_corr < 0.6f || (float)pitch0 > (float)*old_pitch * 0.4f ) */
            IF (s_max(sub(19660, *old_corr), sub(pitch0, mult(*old_pitch, 13107))) > 0)
            {
                /* reinforce the normalized correlation */
                /* operands are Q12, result is Q12 */
                *corr = extract_h(L_tmp);
            }
            /* operands are Q12, result is Q12 */
            *fac = extract_h(L_shl(L_mult(*fac, THRES0), 3));
        }
        pit_min = add(pit_min, pitch0);     /* next multiple */
        delta = add(delta, step);           /* the incertitude to the allowed range */
    }
}

/*---------------------------------------------------------------------------*
 * pitch_neighbour_fx
 *
 * Verifies if the maximum correlation pitch lag is coherent with neighbour
 * values
 *---------------------------------------------------------------------------*/
static void pitch_neighbour_fx(
    Word16 sect0,               /* i  : indicates whether section 0 (below PIT_MIN) is used       */
    Word16 pitch_tmp[],         /* i  : estimated pitch values for each half-frame & look-ahead   */
    Word16 pitch[3][2*NSECT],   /* i  : tested pitch values for each half-frame & look-ahead      */
    Word16 corr_tmp[],          /* i  : raw normalized correlation (before different scalings) Q15*/
    Word16 corr[3][2*NSECT],    /* i/o: normalized correlation for each half-frame & look-ahead Q12 */
    Word16 thres1[2*NHFR],      /* i  : maximum scaling for the immediate neighbours Q15          */
    Word16 ind_tmp[2*NHFR]      /* i  : best section index for each half-frame & look-ahead       */
)
{
    Word16 delta, i, j, k, K, coh_flag, fac;

    /*---------------------
     * 1st set of sections
     ---------------------*/
    FOR ( k = sect0; k < NSECT; k++ )    /* loop for each section */
    {
        K = 3;
        move16();
        if (sub(k, (NSECT-1)) == 0)  /* the number of tests depends on the section */
        {
            K = 2;
            move16();
        }
        /*pt = &pitch[i][k] and pt = &corr[i][k]*/
        FOR (i=0; i < K; i++)        /* for the 2 half-frames and look-ahead */
        {
            /* Compare pitch values of the present frame */
            FOR (j=0; j < K; j++)    /* Verify pitch coherence with neighbours (including past pitch) */
            {
                IF (sub(j, i) != 0)  /* Exclude itself, of course */
                {
                    IF (sub(corr_tmp[j], CORR_TH1) >= 0) /* reinforcement can happen only if the correlation is high enough */
                    {
                        delta = abs_s(sub(pitch[i][k], pitch_tmp[j]));              /* Find difference of pitch values */
                        coh_flag = pitch_coherence_fx(pitch[i][k], pitch_tmp[j], COH_FAC, DELTA_COH);

                        IF (coh_flag != 0)
                        {
                            /* Favour stability across sections, favour closer values */
                            IF (sub(ind_tmp[j], k) == 0)
                            {
                                /* corr[i][k] *= ( -thres1[j]/DELTA1 * delta + thres1[j]+1 ); */
                                /* operands are Q15, except corr[i][k] which is Q12 */
                                fac = mult(negate(thres1[j]), MAX_16/DELTA_COH);
                                fac = add(i_mult2(fac, delta), thres1[j]);
                            }
                            ELSE
                            {
                                /* corr[i][k] *= ( -thres1[j]/DELTA1 * 0.625f * delta + 0.625f * thres1[j] +1.0f ); */
                                fac = mult(negate(thres1[j]), 20479/DELTA_COH);
                                fac = add(i_mult2(fac,delta), mult(20479, thres1[j]));
                            }
                            corr[i][k] = add(corr[i][k], mult(fac, corr[i][k]));
                            move16();
                        }
                    }
                }
            }
        }
    }

    /*---------------------
     * 2nd set of sections
     ---------------------*/
    FOR ( k = sect0; k < NSECT; k++ )    /* loop for each section */
    {
        K = 3;
        move16();
        if (sub(k, (NSECT-1)) == 0)      /* the number of tests depends on the section */
        {
            K = 2;
            move16();
        }
        /*pt = &pitch[i][k] and pt = &corr[i][k]*/
        FOR (i=0; i < K; i++)/* BRANCH(1); for the 2 half-frames and look-ahead */
        {
            /* Compare pitch values of the present frame */
            FOR (j=0; j < K; j++)/* Verify pitch coherence with neighbours (including past pitch) */
            {
                IF (sub(j, i) != 0)/* Exclude itself, of course */
                {
                    IF (sub(corr_tmp[j+NHFR], CORR_TH1) >= 0)/* reinforcement can happen only if the correlation is high enough */
                    {
                        delta = abs_s(sub(pitch[i][NSECT+k], pitch_tmp[j+NHFR])); /* Find difference of pitch values */
                        coh_flag = pitch_coherence_fx(pitch[i][NSECT+k], pitch_tmp[j+NHFR], COH_FAC, DELTA_COH);

                        IF (coh_flag != 0)
                        {
                            /* Favour stability across sections, favour closer values */
                            IF (sub(ind_tmp[j+NHFR], add(NSECT, k)) == 0)
                            {
                                /* corr[i][k] *= ( -thres1[j+NHFR]/DELTA1 * delta + thres1[j+NHFR]+1 ); */
                                /* operands are Q15, except corr[i][NSECT+k] which is Q12 */
                                fac = mult(negate(thres1[j+NHFR]), MAX_16/DELTA_COH);
                                fac = add(extract_l(L_shr(L_mult(fac, delta), 1)), thres1[j+NHFR]);
                                corr[i][NSECT+k] = add(corr[i][NSECT+k], mult(fac, corr[i][NSECT+k]));
                                move16();
                            }
                            ELSE
                            {
                                /* corr[i][k] *= ( -thres1[j+NHFR]/DELTA1 * 0.625f * delta + 0.625f * thres1[j+NHFR] +1.0f ); */
                                fac = mult(negate(thres1[j+NHFR]), 20479/DELTA_COH);
                                fac = add(extract_l(L_shr(L_mult(fac,delta), 1)), mult(20479, thres1[j+NHFR]));
                                corr[i][NSECT+k] = add(corr[i][NSECT+k], mult(fac, corr[i][NSECT+k]));
                                move16();
                            }
                        }
                    }
                }
            }
        }
    }
}

/*-----------------------------------------------------------------*
 * pitch_coherence_fx
 *
 * Verify if pitch evolution is smooth
 *-----------------------------------------------------------------*/
static Word16 pitch_coherence_fx(
    Word16 pitch0,      /* i  : first pitch to compare        */
    Word16 pitch1,      /* i  : 2nd pitch to compare          */
    Word16 fac_max,     /* i  : max ratio of both values Q12  */
    Word16 diff_max     /* i  : max difference of both values */
)
{
    Word16 smaller, larger;
    Word16 pc;

    smaller = s_min(pitch0, pitch1);
    larger = s_max(pitch0, pitch1);

    pc = 0;
    move16();
    test();
    if( (sub(larger, extract_h(L_shl(L_mult(fac_max, smaller), 3))) <= 0) &&    /* Changed to <= to keep BE */
            (sub(sub(larger, smaller), diff_max) < 0))
    {
        pc = 1;
        move16();
    }

    return pc;
}

/*-----------------------------------------------------------------*
 * LP_Decim2_Copy:
 *
 * Decimate a vector by 2 with 2nd order fir filter.
 *-----------------------------------------------------------------*/
static void LP_Decim2_Copy(
    const Word16 x[],     /* i:   signal to process */
    Word16 y[],     /* o:   signal to process */
    Word16 l,       /* i  : size of filtering */
    Word16 mem[]    /* i/o: memory (size=3)   */
)
{
    Word16 *p_x, x_buf[L_FRAME + L_MEM];
    Word16 i, j, k;
    Word32 L_tmp;

    /* copy initial filter states into buffer */
    p_x = x_buf;
    FOR (i = 0; i < L_MEM; i++)
    {
        *p_x++ = mem[i];
        move16();
    }
    FOR (i = 0; i < l; i++)
    {
        *p_x++ = x[i];
        move16();
    }
    if (l & 1) /* Fix for valgrind error in case l is odd. Anyway this function will be removed. */
    {
        *p_x = *(p_x-1);
        move16();
    }

    FOR (i = 0; i < L_MEM; i++)
    {
        mem[i] = x[l - L_MEM + i];
        move16();
    }
    p_x = x_buf;
    j = 0;
    move16();
    FOR (i = 0; i < l; i += 2)
    {
        L_tmp = L_mult(*p_x, H_fir[0]);
        FOR (k = 1; k < L_FIR_PO; k++)
        {
            L_tmp = L_mac(L_tmp, p_x[k], H_fir[k]);
        }
        p_x+=2;

        y[j++] = round_fx(L_tmp);
    }
}
/*---------------------------------------------------------------------*
 * Dot_product12_OL
 *
 * two different length dot products of x and y
 *---------------------------------------------------------------------*/
static Word32 Dot_product12_OL( /* o  : Q31: normalized result (1 < val <= -1) */
    Word16 *sum1,         /* o  : Q31: normalized result 2 */
    const Word16 x[],           /* i  : 12bits: x vector */
    const Word16 y[],           /* i  : 12bits: y vector */
    const Word16 lg,            /* i  : vector length */
    const Word16 lg2,           /* i  : vector length 2 */
    Word16 *exp,          /* o  : exponent of result (0..+30) */
    Word16 *exp2          /* o  : exponent of result 2 (0..+30) */
)
{
    Word16 i, sft;
    Word32 L_sum, L_sum2;

    L_sum = L_mac(1, x[0], y[0]);
    IF (sub(lg, lg2) <= 0)
    {
        FOR (i = 1; i < lg; i++)
        {
            L_sum = L_mac(L_sum, x[i], y[i]);
        }
        /* sets to 'L_sum' in 1 clock */
        L_sum2 = L_add(0, L_sum);
        FOR (; i < lg2; i++)
        {
            L_sum2 = L_mac(L_sum2, x[i], y[i]);
        }
    }
    ELSE
    {
        FOR (i = 1; i < lg2; i++)
        {
            L_sum = L_mac(L_sum, x[i], y[i]);
        }
        /* sets to 'L_sum' in 1 clock */
        L_sum2 = L_add(0, L_sum);
        FOR (; i < lg; i++)
        {
            L_sum = L_mac(L_sum, x[i], y[i]);
        }
    }

    /* Q31 */
    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);
    *exp = sub(30, sft);
    move16(); /* exponent = 0..30 */

    sft = norm_l(L_sum2);
    L_sum2 = L_shl(L_sum2, sft);
    *exp2 = sub(30, sft);
    move16(); /* exponent = 0..30 */

    *sum1 = extract_h(L_shr(L_sum2, 1));

    return L_sum;
}

/*---------------------------------------------------------------------*
 * Dot_product12_OL_back()
 *
 * two different length dot products of x and y, computed backward
 *---------------------------------------------------------------------*/
static Word32 Dot_product12_OL_back(/* o  : Q31: normalized result (1 < val <= -1) */
    Word16 *sum1,             /* o  : Q31: normalized result 2 */
    const Word16 x[],               /* i  : 12bits: x vector */
    const Word16 y[],               /* i  : 12bits: y vector */
    const Word16 lg,                /* i  : vector length */
    const Word16 lg2,               /* i  : vector length 2 */
    Word16 *exp,              /* o  : exponent of result (0..+30) */
    Word16 *exp2              /* o  : exponent of result 2 (0..+30) */
)
{
    Word16 i, sft;
    Word32 L_sum, L_sum2;

    L_sum = L_mac(1, x[0], y[0]);
    IF (sub(lg, lg2) <= 0)
    {
        FOR (i = 1; i < lg; i++)
        {
            L_sum = L_mac(L_sum, x[-i], y[-i]);
        }
        /* sets to 'L_sum' in 1 clock */
        L_sum2 = L_add(0, L_sum);
        FOR (; i < lg2; i++)
        {
            L_sum2 = L_mac(L_sum2, x[-i], y[-i]);
        }
    }
    ELSE
    {
        FOR (i = 1; i < lg2; i++)
        {
            L_sum = L_mac(L_sum, x[-i], y[-i]);
        }
        /* sets to 'L_sum' in 1 clock */
        L_sum2 = L_add(0, L_sum);
        FOR (; i < lg; i++)
        {
            L_sum = L_mac(L_sum, x[-i], y[-i]);
        }
    }

    /* Q31 */
    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);
    *exp = sub(30, sft);
    move16(); /* exponent = 0..30 */

    sft = norm_l(L_sum2);
    L_sum2 = L_shl(L_sum2, sft);
    *exp2 = sub(30, sft);
    move16(); /* exponent = 0..30 */

    *sum1 = extract_h(L_shr(L_sum2, 1));

    return L_sum;
}


void pitchDoubling_det(
    Word16 *wspeech,
    Word16 *pitch_ol,
    Word16 *T_op_fr,
    Word16 *voicing_fr
)
{
    Word16 new_op_fr[2];
    Word16 new_voicing[2];
    Word16 new_Top[2];
    Word16 m, T;


    /*save initial values*/

    new_Top[0]=pitch_ol[0];
    move16();
    new_Top[1]=pitch_ol[1];
    move16();

    FOR(m=2; m<5; m++)
    {

        /* T= pitch_ol[0]/m; */
        T = mult(pitch_ol[0],One_div_fx[m-1]);

        IF(sub(T,PIT_MIN_12k8)>= 0)
        {
            pitch_ol2_fx( PIT_MIN_SHORTER, T, &(new_op_fr[0]), &new_voicing[0], 0, wspeech, 2 );
            pitch_ol2_fx( PIT_MIN_SHORTER, T, &(new_op_fr[1]), &new_voicing[1], L_SUBFR, wspeech, 2 );
            /* IF(sub(add(new_voicing[0],new_voicing[1]),add(voicing_fr[0],voicing_fr[1]))>0 */
            IF(  L_msu(L_msu(L_mac(L_mult(new_voicing[0], 8192),new_voicing[1], 8192), voicing_fr[0], 8192), voicing_fr[1], 8192) > 0 )
            {
                new_Top[0]=T;
                move16();
                T_op_fr[0]=new_op_fr[0];
                move16();
                T_op_fr[1]=new_op_fr[1];
                move16();
                voicing_fr[0]=new_voicing[0];
                move16();
                voicing_fr[1]=new_voicing[1];
                move16();
            }
        }

        /* T= pitch_ol[1]/m; */
        T = mult(pitch_ol[1],One_div_fx[m-1]);

        IF(sub(T,PIT_MIN_12k8)>= 0)
        {
            pitch_ol2_fx( PIT_MIN_SHORTER, T, &(new_op_fr[0]), &new_voicing[0], 2*L_SUBFR, wspeech, 2 );
            pitch_ol2_fx( PIT_MIN_SHORTER, T, &(new_op_fr[1]), &new_voicing[1], 3*L_SUBFR, wspeech, 2 );
            /* IF(sub(add(new_voicing[0],new_voicing[1]),add(voicing_fr[2],voicing_fr[3]))>0) */
            IF(  L_msu(L_msu(L_mac(L_mult(new_voicing[0], 8192),new_voicing[1], 8192), voicing_fr[2], 8192), voicing_fr[3], 8192) > 0 )
            {
                new_Top[1]=T;
                move16();
                T_op_fr[2]=new_op_fr[0];
                move16();
                T_op_fr[3]=new_op_fr[1];
                move16();
                voicing_fr[2]=new_voicing[0];
                move16();
                voicing_fr[3]=new_voicing[1];
                move16();
            }
        }
    }

    pitch_ol[0]=new_Top[0];
    move16();
    pitch_ol[1]=new_Top[1];
    move16();

}/*end of pitch doubling detection*/
