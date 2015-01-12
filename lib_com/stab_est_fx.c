/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"         /* required for wmc_tool */
#include "basop_mpy.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define BIN_4000       80    /* The frequency bin corresponding to 4kHz */
#define MAX_BANDEXC    20

#define NB_TH3_MIN     30
#define NB_TH1_MIN     30

#define TH_0_MIN2_FX  3840      /* Q11 -> 1.875  */
#define TH_1_MIN2_FX  2560      /* Q11 -> 1.25   */
#define TH_2_MIN2_FX  1920      /* Q11 -> 0.9375 */
#define TH_3_MIN2_FX  1280      /* Q11 -> 0.625  */
#define TH_0_MAX_FX   9600      /* Q11 -> 1.5*3.125  */
#define TH_1_MAX_FX   8640      /* Q11 -> 1.5*2.8125 */
#define TH_2_MAX_FX   6720      /* Q11 -> 1.5*2.1875 */
#define TH_3_MAX_FX   5760      /* Q11 -> 1.5*1.875  */

#define TH_UP_FX  320           /* Q11 -> 0.15625  */
#define TH_DW_FX  320           /* Q11 -> 0.15625  */

/*------------------------------------------------------------------------*
 * stab_est()
 *
 * Signal stability estimation based on energy variation
 *------------------------------------------------------------------------*/

Word16 stab_est_fx(
    Word16 etot,            /* i   : Total energy of the current frame   */
    Word16 *lt_diff_etot,   /* i/o : Long term total energy variation    */
    Word16 *mem_etot,       /* i/o : Total energy memory                 */
    Word16 *last_bfi_cnt,   /* i/o : number of frame since last bfi      */
    Word16 bfi,             /* i   : Bad frame indicator                 */
    Word16 *nb_thr_3,       /* i/o : Number of consecutives frames of level 3 */
    Word16 *nb_thr_1,       /* i/o : Number of consecutives frames of level 1 */
    Word16 *thresh,         /* i/o : Detection thresold                 */
    Word16 *last_music_flag,/* i/o : Previous music detection ouptut    */
    Word16 vad_flag
)
{
    Word16 i, music_flag2, tmp16, exp1, exp2;
    Word16 mean_diff;
    Word16 dev;
    Word32 L_tmp;

    /*------------------------------------------------------------------------*
     * Find mean of the past MAX_LT frames energy variation
     *------------------------------------------------------------------------*/

    L_tmp  = L_deposit_l(0);
    FOR (i = 1; i<MAX_LT; i++)
    {
        /*mean_diff += lt_diff_etot[i-1] * INV_MAX_LT;  divide by MAX_LT */
        L_tmp = L_mac(L_tmp, lt_diff_etot[i-1], INV_MAX_LT_FX);
        lt_diff_etot[i-1] = lt_diff_etot[i];
        move16();
    }
    /*mean_diff += lt_diff_etot[i-1] * INV_MAX_LT; */ /* divide by MAX_LT */
    L_tmp = L_mac(L_tmp, lt_diff_etot[i-1], INV_MAX_LT_FX);
    mean_diff = round_fx(L_tmp);					   /*Q8 */

    /*------------------------------------------------------------------------*
     * Find statistical deviation of the energy variation history
     * against the last 15 frames
     *------------------------------------------------------------------------*/

    tmp16 = sub(lt_diff_etot[MAX_LT-15], mean_diff);
    L_tmp = L_mult0(tmp16, tmp16);
    FOR(i = MAX_LT-15+1; i<MAX_LT; i++)
    {
        /*fcorr += ftmp_c*ftmp_c;*/
        tmp16 = sub(lt_diff_etot[i], mean_diff);
        L_tmp = L_mac0(L_tmp, tmp16, tmp16);
    }
    /*------------------------------------------------------------------------*
     * Update
     *------------------------------------------------------------------------*/
    lt_diff_etot[i-1] = sub(etot, *mem_etot);
    move16();
    *mem_etot = etot;
    move16();

    /*------------------------------------------------------------------------*
     * Compute statistical deviation
     *------------------------------------------------------------------------*/

    /*  dev = (float)0.7745967f*sqrt(fcorr / 15); */
    L_tmp = Mpy_32_16_1(L_tmp, 1311); /*-> 1/25 (1/(MAX_LT-15))*/

    exp1 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, exp1);
    tmp16 = round_fx(L_tmp);

    exp2 = sub(31, exp1);
    L_tmp = Isqrt_lc(L_tmp, &exp2);
    L_tmp = Mpy_32_16_1(L_tmp, tmp16);        /* we now have sqrt(L_corr) Q24 (8+16)*/
    exp2 = sub(31-15, sub(exp1, exp2)); /* for Q8 (because of -8^2 from Etot)*/

    L_tmp = L_shl(L_tmp, exp2);         /* Q8 + Q16*/
    dev = extract_h(L_shl(L_tmp, 3));   /* Q(24+3-16) -> Q11 */

    /*------------------------------------------------------------------------*
     * State machine to decide level of inter-harmonic noise reduction and
     * (only if this frame is GOOD or if we are already far from NB_BFI_THR)
     * (if music_flag2 is 0, the spectral modification is deactivated, otherwise, it is activated)
     *------------------------------------------------------------------------*/

    music_flag2 = 0;
    move16();/* deactivate spectral modification (if music_flag2 != 0 is activated) */
    test();
    IF (bfi == 0 && sub(*last_bfi_cnt,NB_BFI_THR) >= 0)
    {
        /*--------------------------------------------------------------------*
         * statistical deviation < thresh3 and last signal category type  >= 3
         * (last category was "tonal" and the new one is "very tonal")
         *--------------------------------------------------------------------*/
        test();
        test();
        test();
        IF ((sub(dev, thresh[3])< 0 ) && (sub(*last_music_flag,3) >= 0) )
        {
            music_flag2 = 4;
            move16();
            *nb_thr_3 += 1;
            move16();
            *nb_thr_1 = 0;
            move16();
        }

        /*--------------------------------------------------------------------*
         * statistical deviation < thresh2 and last signal category type  >= 2
         * (last category was "moderatly tonal" and the new one is a "tonal" )
         *--------------------------------------------------------------------*/
        ELSE IF ((sub(dev, thresh[2])< 0 ) && (sub(*last_music_flag,2) >= 0) )
        {
            music_flag2 = 3;
            move16();
            *nb_thr_3 += 1;
            move16();
            *nb_thr_1 = 0;
            move16();
        }

        /*--------------------------------------------------------------------*
         * statistical deviation < thresh1 and last signal category type  >= 1
         * (last category was "slightly tonal" and the new one is a "moderatly tonal")
         *--------------------------------------------------------------------*/
        ELSE IF ((sub(dev, thresh[1])< 0 ) && (sub(*last_music_flag,1) >= 0) )
        {
            music_flag2 = 2;
            move16();
        }

        /*--------------------------------------------------------------------*
         * statistical deviation < thresh0
         * (last category was "not tonal" and the new one is "slightly tonal")
         *--------------------------------------------------------------------*/
        ELSE IF ((sub(dev, thresh[0]) < 0 ) )
        {
            music_flag2 = 1;
            move16();/* [2000, 4000] Hz */
        }

        /*--------------------------------------------------------------------*
         * statistical deviation > thresh0
         * (Statistical deviation is high: the new tonal category is not tonal)
         *--------------------------------------------------------------------*/
        ELSE
        {
            *nb_thr_1 = add(*nb_thr_1,1);
            *nb_thr_3 = 0;
            move16();

        }

        /*------------------------------------------------------------------------*
         * Update the thresholds
         *------------------------------------------------------------------------*/
        IF (sub(*nb_thr_3,NB_TH3_MIN) > 0)
        {

            /* the number of consecutive categories type 3 or 4 (most tonal and tonal) */
            /* is greater than 30 frames ->increase the deviations thresholds to allow more variation */
            thresh[0] = add(thresh[0], TH_UP_FX);
            move16();	/*Q11 */
            thresh[1] = add(thresh[1], TH_UP_FX);
            move16();
            thresh[2] = add(thresh[2], TH_UP_FX);
            move16();
            thresh[3] = add(thresh[3], TH_UP_FX);
            move16();

        }
        ELSE IF (sub(*nb_thr_1,NB_TH1_MIN) > 0)
        {
            /* the number of consecutive categories type 0 (non tonal frames) */
            /* is greater than 30 frames -> decrease the deviations thresholds to allow less variation */
            thresh[0] = sub(thresh[0], TH_DW_FX);
            move16();	/*Q11 */
            thresh[1] = sub(thresh[1], TH_DW_FX);
            move16();
            thresh[2] = sub(thresh[2], TH_DW_FX);
            move16();
            thresh[3] = sub(thresh[3], TH_DW_FX);
            move16();
        }

        /* limitation of the threshold (this local macro stores the highest of the two and it also
            counts the # of operations) */

        move16();
        move16();
        move16();
        thresh[0] = s_max(thresh[0], TH_0_MIN2_FX);
        thresh[1] = s_max(thresh[1], TH_1_MIN2_FX);
        thresh[2] = s_max(thresh[2], TH_2_MIN2_FX);

        move16();
        move16();
        move16();
        thresh[0] = s_min(thresh[0], TH_0_MAX_FX);
        thresh[1] = s_min(thresh[1], TH_1_MAX_FX);
        thresh[2] = s_min(thresh[2], TH_2_MAX_FX);
        move16();
        move16();
        thresh[3] = s_max(thresh[3], TH_3_MIN2_FX);
        thresh[3] = s_min(thresh[3], TH_3_MAX_FX);
    }
    ELSE
    {
        /*--------------------------------------------------------------------*
         * bfi hysterisis not completed
         *--------------------------------------------------------------------*/

        *last_bfi_cnt = add(*last_bfi_cnt,1);
        music_flag2 = 0;
        move16();
        thresh[3] = TH_3_MIN2_FX;
        move16();
        thresh[2] = TH_2_MIN2_FX;
        move16();
        thresh[1] = TH_1_MIN2_FX;
        move16();
        thresh[0] = TH_0_MIN2_FX;
        move16();
        if (bfi != 0)
        {
            *last_bfi_cnt = 0;
            move16();
        }
    }

    /*------------------------------------------------------------------------*
     * Final update
     *------------------------------------------------------------------------*/

    *last_music_flag = music_flag2;
    move16();
    if (vad_flag == 0)
    {

        music_flag2 = 0;
        move16();
    }

    return music_flag2;
}
