/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define K_COR_FX      23405  /* Q13    2.857 */
#define C_COR_FX     -10535  /* Q13   -1.286 */

#define K_EE_FX        1365  /* Q15  0.04167 */
#define C_EE_FX           0

#define K_ZC_FX       -1311  /* Q15   -0.04 */
#define C_ZC_FX       19661  /* Q13    2.4  */

#define K_RELE_FX      1638  /* Q15    0.05 */
#define C_RELE_FX     14746  /* Q15    0.45 */

#define K_PC_FX       -2341  /* Q15 -0.07143*/
#define C_PC_FX       30425  /* Q1   1.857  */

#define K_SNR_FX     3541    /* Q15 .1111    */
#define C_SNR_FX     -10921  /* Q15 -0.3333f */


#define THRES_EEN    514206  /* 251.077 =>  (10^(1/(K_EE*10)))  Q11*/

/*-------------------------------------------------------------------*
 * signal_clas_fx()
 *
 * classification state machine for FEC
 * TC frames selection
 *-------------------------------------------------------------------*/

Word16 signal_clas_fx(        /* o  : classification for current frames              */
    Encoder_State_fx *st,         /* i/o: encoder state structure                           */
    Word16 *coder_type, /* i/o: coder type                                        */
    const Word16 voicing[3],  /* i  : normalized correlation for 3 half-frames          */
    const Word16 *speech,     /* i  : pointer to speech signal for E computation        */
    const Word16 localVAD,    /* i  : vad without hangover                              */
    const Word16 pit[3],      /* i  : open loop pitch values for 3 half-frames          */
    const Word32 *ee,         /* i  : lf/hf E ration for 2 half-frames                  */
    const Word16 relE,        /* i  : frame relative E to the long term average         */
    const Word16 L_look ,     /* i  : look-ahead                                        */
    Word16 *uc_clas     /* o  : temporary classification used in music/speech class*/
)
{
    Word32 Ltmp;
    Word16 mean_voi2, een, corn, zcn, relEn, pcn, fmerit1;
    Word16 i, clas, pc, zc, lo, lo2, hi, hi2, exp_ee, frac_ee;
    Word16 tmp16, tmpS;
    const Word16 *pt1;
    Word16 unmod_coder_type;

    /*----------------------------------------------------------------*
     * Calculate average voicing
     * Calculate average spectral tilt
     * Calculate zero-crossing rate
     * Calculate pitch stability
     *----------------------------------------------------------------*/

    /* average voicing on second half-frame and look-ahead */
    Ltmp = L_mult(voicing[1], 16384);
    mean_voi2 = mac_r(Ltmp, voicing[2], 16384);

    /* average spectral tilt in dB */
    lo = L_Extract_lc(ee[0], &hi);
    lo2 = L_Extract_lc(ee[1], &hi2);
    Ltmp = L_mult(lo, lo2);    /* Q5*Q5->Q11 */

    test();
    test();
    IF (L_sub(Ltmp, 2048) < 0)
    {
        een = 0;
        move16();
    }
    ELSE IF (L_sub(Ltmp, THRES_EEN) > 0 || hi > 0 || hi2 > 0)
    {
        een = 512;
        move16();
    }
    ELSE
    {
        /* mean_ee2 = 0.5f * 20.0f * (float)log10( tmp ); */
        /* een = K_EE_ENC * mean_ee2 + C_EE_ENC; */
        exp_ee = norm_l(Ltmp);
        frac_ee = Log2_norm_lc(L_shl(Ltmp, exp_ee));
        exp_ee = sub(30-11, exp_ee);
        Ltmp = Mpy_32_16(exp_ee, frac_ee, LG10);     /* Ltmp Q14 */
        een = round_fx(L_shl(Ltmp, 16-5));          /* Q14 -> Q9 */
        een = mac_r(C_EE_FX, een, K_EE_FX);
    }

    /* compute zero crossing rate */
    pt1 = speech + L_look - 1;
    tmpS = shr(*pt1, 15); /* sets 'tmpS to -1 if *pt1 < 0 */
    Ltmp = L_deposit_l(0);
    FOR (i = 0; i < L_FRAME; i++)
    {
        tmp16 = add(1, tmpS);
        pt1++;
        tmpS = shr(*pt1, 15); /* pt1 >=0 ---> 0 OTHERWISE -1   */
        Ltmp = L_msu0(Ltmp, tmpS, tmp16);
    }
    zc = extract_l(Ltmp);

    /* compute pitch stability */
    pc = add( abs_s(sub(pit[1], pit[0])), abs_s(sub(pit[2], pit[1])));

    /*-----------------------------------------------------------------*
     * Transform parameters to the range <0:1>
     * Compute the merit function
     *-----------------------------------------------------------------*/

    /* corn = K_COR * mean_voi2 + C_COR */
    Ltmp = L_mult(C_COR_FX, 32767);
    corn  = round_fx(L_shl(L_mac(Ltmp, mean_voi2, K_COR_FX),-4)); /*Q13+Q13*Q15 =>Q13->Q9*/
    /* Limit [0, 1] */
    corn = s_max(corn, 0);
    corn = s_min(corn, 512);

    Ltmp = L_mult(C_ZC_FX, 4);                                    /*Q13*Q2 -> Q16*/
    zcn = round_fx(L_shl(L_mac(Ltmp, zc, K_ZC_FX),16-7));         /*Q0*Q15 + Q16*/
    /* Limit [0, 1] */
    zcn = s_max(zcn, 0);
    zcn = s_min(zcn, 512);

    Ltmp = L_mult(C_RELE_FX, 256);                                /*Q15 ->Q24*/
    relEn = round_fx(L_shl(L_mac(Ltmp, relE, K_RELE_FX), 1));     /*relE in Q8 but relEn in Q9*/
    /* Limit [0.5, 1] */
    relEn = s_max(relEn, 256);
    relEn = s_min(relEn, 512);

    Ltmp = L_mult(C_PC_FX, 2);                                    /*Q14*Q1 -> Q16*/
    pcn = round_fx(L_shl(L_mac(Ltmp, pc, K_PC_FX),16-7));         /*Q16 + Q0*Q15*/
    /* Limit [0, 1] */
    pcn = s_max(pcn, 0);
    pcn = s_min(pcn, 512);

    Ltmp = L_mult(een, 10923);
    Ltmp = L_mac(Ltmp, corn, 21845);
    Ltmp = L_mac(Ltmp, zcn, 10923);
    Ltmp = L_mac(Ltmp, relEn, 10923);
    Ltmp = L_mac(Ltmp, pcn, 10923);

    fmerit1 = round_fx(L_shl(Ltmp, 16-10-1));  /* fmerit1 ->Q15 */

    /*-----------------------------------------------------------------*
     * FEC classification
     * Onset classification
     *-----------------------------------------------------------------*/


    /* FEC classification */
    test();
    test();
    IF (localVAD == 0 || sub(*coder_type,UNVOICED) == 0 || sub(relE,-1536) < 0)
    {
        clas = UNVOICED_CLAS;
        move16();
    }
    ELSE
    {
        SWITCH (st->last_clas_fx)
        {
        case VOICED_CLAS:
        case ONSET:
        case VOICED_TRANSITION:

            IF (sub(fmerit1, 16056) < 0)        /*0.49f*/
            {
                clas = UNVOICED_CLAS;
                move16();
            }
            ELSE IF (sub(fmerit1, 21626) < 0)   /*0.66*/
            {
                clas = VOICED_TRANSITION;
                move16();
            }
            ELSE
            {
                clas = VOICED_CLAS;
                move16();
            }
            BREAK;

        case UNVOICED_CLAS:
        case UNVOICED_TRANSITION:
            IF (sub(fmerit1, 20643) > 0)        /*0.63*/
            {
                clas = ONSET;
                move16();
            }
            ELSE IF (sub(fmerit1, 19169) > 0)  /*0.585*/
            {
                clas = UNVOICED_TRANSITION;
                move16();
            }
            ELSE
            {
                clas = UNVOICED_CLAS;
                move16();
            }
            BREAK;

        default:
            clas = UNVOICED_CLAS;
            move16();
            BREAK;
        }
    }

    /* set flag for unvoiced class, it will be used in sp/mus classifier */
    *uc_clas = clas;
    move16();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    if( ( (sub(*coder_type,UNVOICED) == 0) ||
            (sub(st->input_bwidth_fx,NB) != 0 && sub(fmerit1,13435) < 0 && sub(st->mold_corr_fx,21299) > 0 ) ||                  /* WB case */
            (sub(st->input_bwidth_fx,NB) == 0 && sub(mult_r(fmerit1,28836),13435) < 0 && sub(st->mold_corr_fx,18022) > 0 ) ) &&  /* NB case */
            sub(relE,-3840) > 0 && sub(st->lt_dec_thres_fx,768) < 0 )       /* to compute unvoiced on frame that tends to speech */
    {
        *uc_clas  = UNVOICED_CLAS;
        move16();
    }

    /* Onset classification */

    /* tc_cnt == -1: frame after TC frame in continuous block of GC/VC frames */
    /* tc_cnt ==  0: UC frame */
    /* tc_cnt ==  1: onset/transition frame, coded by GC mode */
    /* tc_cnt ==  2: frame after onset/transition frame, coded by TC mode */

    if( sub(clas,UNVOICED_CLAS ) == 0)
    {
        st->tc_cnt_fx = 0;
        move16();
    }

    test();
    if( sub(clas,VOICED_TRANSITION) >= 0 && st->tc_cnt_fx >= 0 )
    {
        st->tc_cnt_fx = add(st->tc_cnt_fx,1);
        move16();
    }

    if( sub(st->tc_cnt_fx,2) > 0 )
    {
        st->tc_cnt_fx = -1;
        move16();
    }

    IF ( sub(st->codec_mode,MODE1) == 0 )
    {
        /*---------------------------------------------------------------------*
         * Coder type modification
         *
         * Prevent UC mode in certain conditions
         * Prevent VC mode in certain conditions
         * Select TC mode in appropriate frames
         *---------------------------------------------------------------------*/

        /* At higher rates, use GC coding instead of UC coding to improve quality */
        test();
        if( L_sub(st->total_brate_fx,ACELP_9k60) > 0 && sub(*coder_type,UNVOICED) == 0 )
        {
            *coder_type = GENERIC;
            move16();
        }

        /* Prevent UC coding on mixed content at 9.6 kb/s */
        test();
        test();
        if( L_sub(st->total_brate_fx,ACELP_9k60) == 0 && sub(*coder_type,UNVOICED) == 0 && st->audio_frame_cnt_fx != 0 )
        {
            *coder_type = GENERIC;
            move16();
        }

        unmod_coder_type = *coder_type;
        move16();

        /* Enforce GC mode on inactive signal (this can be later overwritten to INACTIVE) */
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        if( localVAD == 0 && (
                    (
                        sub(*coder_type,UNVOICED) == 0
                        && ( ( st->Opt_SC_VBR_fx == 0) || ( ( st->Opt_SC_VBR_fx == 1 ) && st->vbr_generic_ho_fx == 0 && sub(st->last_coder_type_fx,UNVOICED) > 0 ))  )
                    || sub(*coder_type,TRANSITION) == 0 || sub(*coder_type,VOICED) == 0 )

          )
        {
            *coder_type = GENERIC;
            move16();
        }

        test();
        test();
        if( sub(*coder_type,GENERIC) == 0 && sub(unmod_coder_type,UNVOICED) == 0 && ( st->Opt_SC_VBR_fx == 1 ) )
        {
            st->vbr_generic_ho_fx = 1;
            move16();
        }

        test();
        if ( sub(*coder_type,UNVOICED) > 0 && ( st->Opt_SC_VBR_fx == 1 ) )
        {
            st->vbr_generic_ho_fx = 0;
            move16();
        }

        st->last_7k2_coder_type_fx = *coder_type;
        move16();
        test();
        if( localVAD == 0 && sub( *coder_type, UNVOICED ) == 0 )
        {
            st->last_7k2_coder_type_fx = GENERIC;
            move16();
        }

        /* Select TC mode for appropriate frames which is in general VOICED_TRANSITION, VOICED_CLAS or MODE1_ONSET frames following UNVOICED_CLAS frames */
        test();
        IF( localVAD != 0 && sub(st->tc_cnt_fx,1) >= 0 )   /* TC mode is allowed only in active signal */
        {
            /* frame after onset/transition frame is coded by TC mode */
            *coder_type = TRANSITION;
            move16();
            if ( sub(st->tc_cnt_fx,1) == 0 )
            {
                /* onset/transition frame is always coded using GC mode */
                *coder_type = GENERIC;
                move16();
            }
        }

        /* At higher rates and with 16kHz core, allow only GC and TC mode */
        test();
        test();
        if( (L_sub(st->total_brate_fx,ACELP_24k40) >= 0) && sub(*coder_type,GENERIC) != 0 && sub(*coder_type,TRANSITION) != 0 )
        {
            *coder_type = GENERIC;
            move16();
        }

        /* Patch for certain low-level signals for which the gain quantizer sometimes goes out of its dynamic range */
        test();
        test();
        test();
        if( sub(*coder_type,VOICED) == 0 && sub(st->input_bwidth_fx,NB) == 0 && sub(relE,-2560) < 0 && L_sub(st->total_brate_fx,ACELP_8k00) <= 0 )
        {
            *coder_type = GENERIC;
            move16();
        }
    }

    return clas;
}
