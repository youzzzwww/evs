/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/

static Word16 FEC_dec_class_fx( Decoder_State_fx *st_fx, const Word32  bitrate, const Word16 coder_type, Word32 *enr_q, const Word16 last_good);

static void Corre(const Word16 *x,const Word16 *y,const Word16 l,Word16 *gain);

/*---------------------------------------------------------------------*
 * Local Constants
 *---------------------------------------------------------------------*/
#define UNS6            10923/2
#define UNS5            13107/2

#define K_COR_FX          14004  /* Q14      0.8547f   <-0.29, 0.88> */
#define C_COR_FX      266180599  /* Q14+16   0.2479f                 */

#define K_TILT_FX         13653  /*Q14       0.8333f   <-0.35, 0.85> */
#define C_TILT_FX     313210491  /*Q14+16    0.2917f                 */

#define K_ZC_FX           -1310  /*Q15       -0.04f     <63, 38>     */
#define C_ZC_FX          165151  /*Q0+16     2.52       <63, 38>     */

#define K_ENR_FX           1311  /*Q15       0.04        <-14, 11>   */
#define C_ENR_FX        9395241  /*Q8+16     0.56f                   */

#define K_PC_FX           -1169  /*Q15       -0.0357f    <45, 17>    */
#define C_PC_FX          105323  /*Q0+16     1.6071f                 */



/*======================================================================*/
/* FUNCTION : FEC_clas_estim()                                          */
/*----------------------------------------------------------------------*/
/* PURPOSE :  Estimation of classification                              */
/*        information, if not available in the bitsream                 */
/*                                                                      */
/*======================================================================*/

void FEC_clas_estim_fx(
    Decoder_State_fx *st_fx ,   /* i/o: decoder state handle                    */
    const Word16 Opt_AMR_WB,          /* i  : flag indicating AMR-WB IO mode          */                /*A*/
    const Word16 L_frame,             /* i  : length of the frame                     */
    Word16 *clas,               /* i/o: frame classification                    */
    const Word16 coder_type,          /* i  : coder type                              */
    const Word16 *pitch,              /* i  : pitch values for each subframe      (Q6)*/
    Word16 *last_good,          /* i  : type of the last correctly received fr. */
    Word16 *syn,                /* i  : synthesis buffer                        */
    Word16 *lp_speech,          /* i/o: long term active speech energy average Q8 */
    Word16 *decision_hyst,      /* i/o: hysteresis of the music/speech decision */                /*A*/
    Word16 *UV_cnt,             /* i/o: number of consecutives frames classified as UV */         /*A*/
    Word16 *LT_UV_cnt,          /* i/o: long term consecutives frames classified as UV */         /*A*/
    Word16 *Last_ener,          /* i/o: last_energy frame                       */                /*A*/
    Word16 *locattack,          /* i/o: detection of attack (mainly to localized speech burst) */ /*A*/
    Word16 *lt_diff_etot,       /* i/o: long-term total energy variation        */                /*A*/
    Word16 *amr_io_class,       /* i/o: classification for AMR-WB IO mode       */                /*A*/
    const Word32  bitrate,            /* i  : Decoded bitrate                         */                /*A*/
    Word16 *Q_syn,              /* i  : Synthesis scaling                       */
    Word16 *class_para,         /* o  : classification para. fmerit1            */                /*A*/
    Word16 *mem_syn_clas_estim, /* i/o: memory of the synthesis signal for frame class estimation */
    Word16 *Q_mem_syn,          /* i/o: exponent for memory of synthesis signal for frame class estimation */ /*B*/
    Word16 pit_max,             /* i  : maximum pitch value, Q16                */                            /*B*/
    Word16 LTP_Gain,            /* i  : LTP gain is 0..0.6 or -1  Q15                                       *//*B*/
    Word16 mode,                /* i  : signal classifier mode                                              *//*B*/
    Word16 bfi,                 /* i  : bad frame indicator                                                 *//*B*/
    Word16 synthStart,          /* i  : starting point of synthesis buffer                                  *//*B*/
    Word16 synthLen             /* i  : length of synthesis buffer, relevant for rescaling                  *//*B*/
)
{
    Word16 i, j, pos;
    Word16 *pt1, *pt2, zc_frame, frame_ener =1 ,tmp_scale=0, tmp_scale_syn, tmp_scale_mem;
    Word16 tiltn, corn, zcn, pcn, fmerit1, enern, ener = 0/*not necessary, just to quiet a warning so not cmplxty counted*/, tilt, diff_ener;
    Word16 voicing, cor_max[4], *synth, tmp16, exp1, exp2;
    Word32 Ltmp, Ltmp1;
    Word16 tmpS, T0, pc, /*max,*/ tmp_x, tmp_y;
    Word16 old_synth[L_SYN_MEM_CLAS_ESTIM + L_FRAME16k];
    Word16 nb_subfr;
    Word16 pos_limit;
    Word16 codec_mode, narrowBand, tcxonly,preemph_fac;
    Word16 memmax;

    nb_subfr = shr(L_frame,6);
    memmax = 0;
    move16();

    codec_mode      = st_fx->codec_mode;
    tcxonly         = st_fx->tcxonly;           /* i  : tcxonly flag          *//*B*/
    narrowBand      = st_fx->narrowBand;        /* i  : narrowband flag       *//*B*/
    preemph_fac     = st_fx->preemph_fac;       /* i  : preemphasis factor    *//*B*/
    /*------------------------------------------------------------------------*
     * Copy synthesized into local buffer
     *------------------------------------------------------------------------*/
    synth  = old_synth + L_SYN_MEM_CLAS_ESTIM;

    /*Rescale synthesis mem buffer or synthesis buffer, if necessary -
    allign them to the same scaling in case of switching MODE2->MODE1*/
    IF(sub(codec_mode, MODE2) == 0)
    {
        memmax=1;
        move16();
        /*find maximum of mem syn*/
        FOR(i=0; i<L_SYN_MEM_CLAS_ESTIM; i++)
        {
            memmax = s_max(memmax, mem_syn_clas_estim[i]);
        }

        tmp_scale = sub(*Q_mem_syn, *Q_syn);
        IF(tmp_scale>0) /*mem syn is bigger, scale it down*/
        {
            tmp_scale_mem = negate(tmp_scale);

            Scale_sig(mem_syn_clas_estim, L_SYN_MEM_CLAS_ESTIM, tmp_scale_mem);
        }
        ELSE /*synthesis is bigger, scale it down*/
        {
            /*check for upscaling mem syn, first*/
            tmp_scale_mem = norm_s(sub(memmax,1));
            if (sub(memmax,1) == 0)
            {
                tmp_scale_mem = 14;
                move16();
            }
            tmp_scale_syn = sub(add(*Q_mem_syn,tmp_scale_mem) ,*Q_syn); /*if this is negative, syn can be scaled down*/
            test();
            IF(tmp_scale_syn > 0 || sub(mode , 1/*CLASSIFIER_TCX*/) == 0) /*dont scale up syn, but scale mem_syn, adequately*/
            {
                tmp_scale_mem = sub(tmp_scale_mem,tmp_scale_syn);
                tmp_scale_syn = 0;
                move16();
            }

            Scale_sig( mem_syn_clas_estim, L_SYN_MEM_CLAS_ESTIM, tmp_scale_mem );
            Scale_sig( syn-synthStart, synthLen, tmp_scale_syn );
            *Q_syn = add(*Q_syn,tmp_scale_syn);

        }

    }

    tmp_scale = -1;
    move16();

    Copy_Scale_sig( mem_syn_clas_estim, old_synth, L_SYN_MEM_CLAS_ESTIM, tmp_scale );
    Copy_Scale_sig( syn, synth, L_frame, tmp_scale );

    /*Tell MODE2 decoder the scaling of the buffer*/
    *Q_mem_syn  = add(*Q_syn, tmp_scale);
    /**Q_mem_syn = *Q_syn;  move16();*/

    test();
    IF(sub(codec_mode,MODE2) == 0 && sub(mode , 1/*CLASSIFIER_TCX*/)==0 )
    {
        /* TCX outputs non-pe-speech */
        move16();
        tmp16 = shl(syn[-1],tmp_scale); /*dont forget to scale the mem*/
        preemph_copy_fx(synth, synth, preemph_fac, L_frame, &tmp16);
    }

    /*------------------------------------------------------------------------*
    * GC, TC and AC frames
    *------------------------------------------------------------------------*/
    /* Do the classification only
    - MODE1: when the class is not transmitted in the bitstream
    - MODE2: on good frames (classifier is also called for bfi=1) */
    test();
    test();
    test();
    test();
    test();
    test();
    IF (( sub(codec_mode , MODE1) == 0 && ( L_sub(bitrate , ACELP_11k60) < 0
                                            || sub(coder_type , UNVOICED) <= 0 || Opt_AMR_WB)) ||
        (sub(codec_mode , MODE2) == 0 && sub(bfi,1) != 0 && !tcxonly ))

    {
        /*------------------------------------------------------------------------*
        * Overwrite classification decision using coder_type information
        *------------------------------------------------------------------------*/
        test();
        IF( sub(coder_type,VOICED) == 0 )
        {
            *clas = VOICED_CLAS;
            move16();
        }
        ELSE IF( sub(coder_type,UNVOICED) == 0 )
        {
            *clas = UNVOICED_CLAS;
            move16();
        }
        ELSE IF( sub(coder_type,INACTIVE) == 0 && !Opt_AMR_WB)
        {
            *clas = INACTIVE_CLAS;
            move16();
        }
        ELSE
        {
            /*------------------------------------------------------------------------*
            * Compute the zero crossing rate for all subframes
            *------------------------------------------------------------------------*/

            pt1 = (Word16 *)synth - 1;
            move16();
            tmpS = shr(*pt1, 15);           /* sets 'tmpS to -1 if *pt1 < 0 */
            Ltmp = L_deposit_l(0);
            FOR (j = 0; j < L_SUBFR*nb_subfr; j++)
            {
                tmp16 = add(1, tmpS);
                pt1++;
                tmpS = shr(*pt1, 15); /* 1Clk: >=0 ---> 0 OTHERWISE -1 */
                Ltmp = L_msu0(Ltmp, tmpS, tmp16);
            }

            zc_frame = extract_l(Ltmp);

            if( sub(L_frame,L_FRAME16k) == 0)
            {
                /*zc_frame *= 0.8f;*/       /* Renormalization for 12.8kHz core*/
                zc_frame = mult_r(zc_frame, 26214);
            }


            /*------------------------------------------------------------------------*
             * Compute the normalized correlation pitch-synch. at the end of the frame
             *------------------------------------------------------------------------*/
            T0 = shr(pitch[3], 6);
            Ltmp1 = L_mult(pitch[3], 256);

            if (sub(T0, L_SUBFR*3/2) > 0)
            {
                T0 = mac_r(Ltmp1, pitch[2], 256);
            }

            if(sub(codec_mode, MODE2) == 0 )
            {
                T0 = s_min(T0,pit_max);
            }

            pt1 = synth;
            move16();
            pos = sub(L_frame, T0); /* T0 [34 231] */


            Corre(&pt1[pos], &pt1[pos-T0], T0, &cor_max[0]);
            T0 = mult_r(add(pitch[2], pitch[3]), 256);
            pos_limit = sub(L_frame, L_SUBFR);
            j = s_min(1, s_max(0, sub(pos, pos_limit)));
            Ltmp = L_deposit_l(cor_max[0]);
            IF (j > 0)
            {
                j = 16384;
                move16();
                pos = sub(pos, T0); /* T0 [34 231] */
                Corre(&pt1[pos], &pt1[pos-T0], T0, &cor_max[1]);
                Ltmp = L_add(Ltmp, cor_max[1]);
                IF (sub(pos, pos_limit) > 0)
                {
                    j = 10923;
                    move16();
                    pos = sub(pos, T0); /* T0 [34 231] */
                    Corre(&pt1[pos], &pt1[pos-T0], T0, &cor_max[2]);
                    Ltmp = L_add(Ltmp, cor_max[2]);
                }
                IF (sub(pos, pos_limit) > 0)
                {
                    j = 8192;
                    move16();
                    pos = sub(pos, T0); /* T0 [34 231] */
                    Corre(&pt1[pos], &pt1[pos-T0], T0, &cor_max[3]);
                    Ltmp = L_add(Ltmp, cor_max[3]);
                }
            }

            voicing = cor_max[0];
            move16();
            IF(j > 0)
            {
                voicing = extract_l(Mult_32_16(Ltmp, j));
            }
            /*------------------------------------------------------------------------*
            * Compute pitch coherence
            *------------------------------------------------------------------------*/

            pc = 0;
            move16();
            test();
            test();
            IF (sub(codec_mode , MODE1) == 0 || !(sub(LTP_Gain , FL2WORD16(-1.f)) != 0 && sub(mode , CLASSIFIER_TCX) == 0) )
            {
                pc = shr(abs_s(sub(add(pitch[3], sub(pitch[2], pitch[1])), pitch[0])), 6);

                if(sub(L_frame,L_FRAME16k) == 0)
                {
                    pc = mult_r(pc, 26214);       /* Renormalization for 12.8kHz core*/
                }
            }

            /*------------------------------------------------------------------------*
             * Compute spectral tilt
             *------------------------------------------------------------------------*/
            pt1 = (Word16 *)synth + L_SUBFR;
            move16();
            pt2 = (Word16 *)synth + L_SUBFR - 1;
            move16();
            Ltmp  = L_mult0(*pt1, *pt1);
            Ltmp1 = L_mult0(*pt1, *pt2);
            FOR(j = 1; j < L_SUBFR*(nb_subfr-1); j++)
            {
                pt1++;
                pt2++;
                Ltmp  = L_mac0(Ltmp,  *pt1, *pt1);
                Ltmp1 = L_mac0(Ltmp1, *pt1, *pt2);
            }
            tilt = 0;
            move16();

            IF (Ltmp != 0)
            {
                BASOP_SATURATE_WARNING_OFF
                tmp16 = extract_l(L_or(L_shr(Ltmp1, 32), 1)); /* sets a flag -1 or 1 for sign of Ltmp1 */
                BASOP_SATURATE_WARNING_ON
                Ltmp1 = L_abs(Ltmp1);
                exp1 = norm_l(Ltmp1);
                tmp_y = extract_h(L_shl(Ltmp1, exp1));
                exp1 = sub(31-1+3, exp1);
                exp2 = norm_l(Ltmp);
                tmp_x = extract_h(L_shl(Ltmp, exp2));
                exp2 = sub(31-1+3, exp2);
                BASOP_SATURATE_WARNING_OFF
                tmpS = shr(sub(tmp_x, tmp_y), 16); /* if tmp_x >= tmp_y tmpS = 0, -1 otherwise */
                BASOP_SATURATE_WARNING_ON
                tmp_y = shl(tmp_y, tmpS);
                exp1  = sub(exp1, tmpS);

                tilt = div_s(tmp_y, tmp_x);
                tilt = shl(tilt, sub(exp1, exp2)); /* saturate to 1.0 */

                tilt = i_mult2(tilt, tmp16);
            }

            /*------------------------------------------------------------------------*
             * Compute pitch-synchronous energy at the frame end
             *------------------------------------------------------------------------*/
            ener = frame_energy_fx(L_frame, pitch, synth, *lp_speech, &frame_ener, *Q_mem_syn);

            /*------------------------------------------------------------------------*
             * transform parameters between 0 & 1
             * find unique merit function
             *------------------------------------------------------------------------*/
            enern  = mac_r(C_ENR_FX, K_ENR_FX, ener);                         /*Q8*/

            tiltn  = extract_h(L_shr(L_mac(C_TILT_FX, K_TILT_FX, tilt), 6));  /*Q14 -> Q8*/

            corn   = extract_h(L_shr(L_mac(C_COR_FX, K_COR_FX, voicing), 6)); /*Q14 -> Q8*/

            zcn    = extract_h(L_shl(L_mac(C_ZC_FX, K_ZC_FX, zc_frame), 8));  /* Q0 -> Q8*/

            BASOP_SATURATE_WARNING_OFF
            tmp16 = sub(LTP_Gain , FL2WORD16(-1.f));
            BASOP_SATURATE_WARNING_ON
            test();
            test();
            IF ( sub(codec_mode , MODE2) == 0 && tmp16 != 0 && sub(mode , CLASSIFIER_TCX) == 0 )
            {
                pcn   = round_fx(L_shl(Mpy_32_16_1(C_PC_FX/*Q16*/, LTP_Gain/*Q15*/),8)); /*Q16*/
            }
            ELSE
            {
                pcn    = extract_h(L_shl(L_mac(C_PC_FX, K_PC_FX,  pc),8));        /* Q0 -> Q8*/
            }

            pcn = s_min(256, pcn);
            pcn = s_max(  0, pcn);
            test();
            test();
            IF(sub(codec_mode,MODE2) == 0 && sub(LTP_Gain,FL2WORD16(-1.f)) == 0 && sub(mode, CLASSIFIER_TCX) == 0)
            {
                /*fmerit1 = (1.0f / 5.0f ) * ( tiltn + 2.0f * corn + zcn + *enern );*/
                Ltmp  = L_mult(tiltn, UNS5);
                Ltmp  = L_mac(Ltmp, corn, 2*UNS5);
                Ltmp  = L_mac(Ltmp, zcn, UNS5);
                Ltmp  = L_mac(Ltmp, enern, UNS5);
                BASOP_SATURATE_WARNING_OFF
                fmerit1 = round_fx(L_shl(Ltmp, 15-8));   /*Q15 can saturate to 1.0 */
                BASOP_SATURATE_WARNING_ON
            }
            ELSE
            {
                /* fmerit1 = (1.0f/6.0f) * (tiltn + 2.0f*corn + zcn + pcn + enern) */
                Ltmp  = L_mult(tiltn, UNS6);
                Ltmp  = L_mac(Ltmp, corn, 2*UNS6);
                Ltmp  = L_mac(Ltmp, zcn, UNS6);
                Ltmp  = L_mac(Ltmp, pcn, UNS6);
                Ltmp  = L_mac(Ltmp, enern, UNS6);
                BASOP_SATURATE_WARNING_OFF
                fmerit1 = round_fx(L_shl(Ltmp, 15-8));   /*Q15 can saturate to 1.0 */
                BASOP_SATURATE_WARNING_ON
            }
            test();
            if ( sub(codec_mode,MODE2) == 0 && narrowBand != 0 )
            {
                fmerit1 = mult_r(fmerit1, FL2WORD16(0.9f)); /* 0.90 */
            }
            IF(sub(codec_mode, MODE1) == 0)
            {
                *class_para = round_fx(L_shl(Ltmp, 14-8)); /*Q14 - cannot be saturated, degrades HF synthesis */
            }

            /*------------------------------------------------------------------------*
             * frame classification
             *------------------------------------------------------------------------*/
            test();
            test();
            if ( (sub(coder_type,VOICED) != 0 && L_sub(bitrate,ACELP_11k60) < 0) || Opt_AMR_WB )
            {
                SWITCH( *last_good )
                {
                case VOICED_CLAS:
                case ONSET:
                case SIN_ONSET:
                case VOICED_TRANSITION:
                    IF(sub(fmerit1, FL2WORD16(0.39f)) < 0)
                    {
                        *clas = UNVOICED_CLAS;
                        move16();
                    }
                    ELSE IF(sub(fmerit1, FL2WORD16(0.63f)) < 0
                            && (add(ener,3840) < 0 || sub(codec_mode,MODE2) == 0))
                    {
                        *clas = VOICED_TRANSITION;
                        move16();
                    }
                    ELSE
                    {
                        *clas = VOICED_CLAS;
                        move16();
                    }

                    BREAK;

                case UNVOICED_CLAS:
                case UNVOICED_TRANSITION:
                case INACTIVE_CLAS:
                    test();
                    if( sub(codec_mode,MODE1) == 0  && sub(L_frame,L_FRAME16k) == 0)
                    {
                        fmerit1 = mult_r(fmerit1,FL2WORD16(0.85f));
                    }

                    IF( sub(fmerit1, FL2WORD16(0.56f)) > 0 )
                    {
                        *clas = ONSET;
                        move16();
                    }
                    ELSE IF( sub(fmerit1, FL2WORD16(0.45f)) > 0 )
                    {
                        *clas = UNVOICED_TRANSITION;
                        move16();
                    }
                    ELSE
                    {
                        *clas = UNVOICED_CLAS;
                        move16();
                    }
                    BREAK;

                default:
                    *clas = UNVOICED_CLAS;
                    move16();

                    BREAK;
                }
            }
        }



        IF(sub(codec_mode,MODE1)==0)
        {
            /*------------------------------------------------------------------------*
             * Overwrite classification decision in case of music
             *------------------------------------------------------------------------*/
            IF( sub(coder_type,AUDIO) == 0 )
            {
                (*decision_hyst) = add(*decision_hyst,4);
                move16();
            }
            ELSE
            {
                (*decision_hyst) = sub(*decision_hyst,1);
                move16();
            }

            if( sub(coder_type,INACTIVE) == 0 )
            {
                *decision_hyst = sub(*decision_hyst,10);
                move16();
            }
            IF( sub(*decision_hyst,200) > 0 )
            {
                *decision_hyst = 200;
                move16();
            }
            ELSE if( *decision_hyst < 0 )
            {
                *decision_hyst = 0;
                move16();
            }

            test();
            test();
            if( sub(*decision_hyst,16) > 0 && sub(*clas,VOICED_CLAS) < 0 && sub(coder_type,AUDIO) == 0 )
            {
                *clas = VOICED_CLAS;
                move16();
            }
        } /*MODE1*/

        /*---------------------------------------------------------------------------------*
         * Measure energy on active voice frames (to improve FEC performance)
         *---------------------------------------------------------------------------------*/
        IF( sub(*clas,VOICED_CLAS) == 0 )
        {
            test();
            test();
            test();
            test();
            test();
            IF(    (sub(codec_mode,MODE2) == 0 && sub(coder_type, VOICED) == 0)
                   || (sub(codec_mode,MODE1) == 0 && (Opt_AMR_WB || (sub(coder_type,GENERIC) != 0 && sub(coder_type,TRANSITION) != 0) ) )
              )
            {
                /* pitch-synchronous energy at the frame end */
                enern = frame_energy_fx(L_frame,pitch, synth, *lp_speech, &frame_ener, *Q_mem_syn);
            }
            /* update of long-term active speech energy */
            Ltmp = L_mult0(655, frame_ener); /* 0.01 */
            *lp_speech = mac_r(Ltmp, 32440, *lp_speech);
            move16(); /* lp_speech update */
        }

        IF(sub(codec_mode, MODE1) == 0)
        {

            /*---------------------------------------------------------------------------------*
             * Overwrite classification decision to UNVOICED_CLAS in case of INACTIVE frame
             *---------------------------------------------------------------------------------*/
            test();
            if( sub(coder_type, INACTIVE) == 0 && sub(*clas,INACTIVE_CLAS) != 0 )
            {
                *clas = UNVOICED_CLAS;
                move16();
            }

            /*---------------------------------------------------------------------------------*
             * Classification refinement to improve noise coding (only in AMR-WB IO mode)
             *---------------------------------------------------------------------------------*/
            IF( Opt_AMR_WB )
            {
                *locattack = 0;
                move16();

                /*-----------------------------------------------------------------------------*
                 * Unvoiced signal but not silence
                 *-----------------------------------------------------------------------------*/

                test();
                IF( sub(*clas, UNVOICED_CLAS) == 0 && sub(ener, -9*256/*Q8*/) > 0 )
                {
                    IF ( sub(*lp_speech, 40*256/*Q8*/) <= 0 )
                    {
                        *UV_cnt = 16;
                        move16();
                    }
                    ELSE
                    {
                        move16();
                        *UV_cnt = sub(*UV_cnt, 8);
                    }
                }

                /*-----------------------------------------------------------------------------*
                 * Neither unvoiced nor clean silence
                 * Number of frames between UV is increased
                 *-----------------------------------------------------------------------------*/

                ELSE IF ( sub(ener, -12*256/*Q8*/) > 0 )
                {
                    move16();
                    *UV_cnt = add(*UV_cnt, 1);
                }

                /*-----------------------------------------------------------------------------*
                 * Maximum/minimum number of frames between UV reached
                 *-----------------------------------------------------------------------------*/

                /* Range 0..300 */            move16();
                *UV_cnt = s_max(s_min(300, *UV_cnt), 0);

                /*-----------------------------------------------------------------------------*
                 * IF VAD = 0 (no voice activity)
                 *   long-term average updated towards to speech
                 *   maximum number of frames between UV is limited to 125
                 * Else
                 *   update long-term average
                 *-----------------------------------------------------------------------------*/

                IF ( sub(coder_type, INACTIVE) == 0 )
                {
                    move16();
                    *LT_UV_cnt = mult_r(31130/*0.95f*/, *LT_UV_cnt); /* tend to speech if no activity */
                    *UV_cnt = s_min(125, *UV_cnt);
                    move16();
                }
                ELSE
                {
                    /* *LT_UV_cnt = 0.9f * *LT_UV_cnt + 0.1f * *UV_cnt */

                    Ltmp = L_mult(3277/*0.1f*/, *UV_cnt);
                    /* Bring to Q22 */
                    Ltmp = L_shl(Ltmp, 6);
                    /* Add to 0.9 x *LT_UV_cnt (Already in Q6) */
                    Ltmp = L_mac(Ltmp, 29491/*0.9f*/, *LT_UV_cnt); /* Q22*/
                    /* Store */
                    *LT_UV_cnt = round_fx(Ltmp);
                }

                /*-----------------------------------------------------------------------------*
                 * Compute frame energy difference
                 * IF long-term average is high and energy difference is relatively low
                 *   classification is overwritten to AUDIO
                 * IF energy difference > 6.0dB
                 *   consider an attack
                 *-----------------------------------------------------------------------------*/

                diff_ener = sub(ener, *Last_ener);
                *Last_ener = ener;
                move16();
                *amr_io_class = *clas;
                move16();
                test();
                if ( sub(*LT_UV_cnt, LT_UV_THR_FX) > 0 && sub(diff_ener, 12*256/*Q8*/) < 0 )
                {
                    move16();
                    *amr_io_class = AUDIO_CLAS;
                }
                test();
                test();
                if ( (sub(diff_ener, 6*256/*Q8*/) > 0 && sub(*clas, AUDIO_CLAS) == 0) || sub(diff_ener, 9*256/*Q8*/) > 0 )
                {
                    *locattack = 1;
                    move16();
                }

                /*------------------------------------------------------------------------*
                 * Find mean of the past 40 frames energy variation
                 *------------------------------------------------------------------------*/

                IF( sub(coder_type, INACTIVE) != 0 )
                {
                    Ltmp = L_deposit_l(0);
                    FOR (i = 1; i<MAX_LT; i++)
                    {
                        Ltmp = L_mac(Ltmp, INV_MAX_LT_FX, lt_diff_etot[i-1]); /* divide by MAX_LT */
                        lt_diff_etot[i-1] = lt_diff_etot[i];
                        move16();
                    }
                    Ltmp = L_mac(Ltmp, INV_MAX_LT_FX, lt_diff_etot[i-1]); /* divide by MAX_LT */
                    /* Ltmp is in Q24 (Q16+Q8) */

                    /*------------------------------------------------------------------------*
                     * Find statistical deviation of the energy variation history
                     * against the last 15 frames
                     *------------------------------------------------------------------------*/

                    Ltmp1 = L_deposit_l(1);
                    FOR (i = MAX_LT-15; i<MAX_LT; i++)
                    {
                        /* ftmp_c = lt_diff_etot[i] - mean_diff */
                        tmp16 = mac_r(Ltmp, lt_diff_etot[i], -32768L);
                        /* fcorr += ftmp_c*ftmp_c */
                        Ltmp1 = L_mac0(Ltmp1, tmp16, tmp16); /* in Q16 (Q8xQ8)*/
                    }
                    lt_diff_etot[i-1] = diff_ener;
                    move16();

                    /*------------------------------------------------------------------------*
                     * Compute statistical deviation
                     * Overwrite classification, if needed
                     *------------------------------------------------------------------------*/

                    /* dev = (float)sqrt(fcorr / (MAX_LT-15)) */
                    Ltmp = Sqrt_Ratio32(Ltmp1, 16/*Q16*/, MAX_LT-15, 0/*Q0*/, &tmp16);
                    tmp16 = round_fx(L_shr(Ltmp, sub(15-8, tmp16))); /* Put in Q8 (Don't care about Possible Saturation) */

                    test();
                    IF ( sub(*amr_io_class, AUDIO_CLAS) == 0 && sub(tmp16, 5*256/*Q8*/) > 0 )
                    {
                        *amr_io_class = *clas;
                        move16();
                        /* *UV_cnt =  (short)(80 + *UV_cnt*0.2f) */
                        *UV_cnt = add(80, mult(*UV_cnt, 6554));
                        move16();
                    }
                }
            } /*if (Opt_AMR_WB)*/
        } /*if (codec_mode==MODE1)*/


    }        /* Do the classification only
        - MODE1: when the class is not transmitted in the bitstream
        - MODE2: on good frames (classifier is also called for bfi=1) */


    if (sub(codec_mode,MODE2) == 0)
    {
        /* Update previous class */
        *last_good = *clas;
        move16();
    }


    /* update the memory of synthesis for frame class estimation */
    Copy( old_synth + L_frame, mem_syn_clas_estim, L_SYN_MEM_CLAS_ESTIM );


    return;
}

static Word16 FEC_dec_class_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word32 bitrate,            /* i  : core bitrate                            */
    const Word16 coder_type,         /* i  : coder type                              */
    Word32 *enr_q,             /* i  : decoded energy Q0                       */
    const Word16 last_good           /* i  : Last good FEC classification            */
)
{
    Word16 clas, tmpS;
    Word16 sfrac;
    Word32 L_tmp;

    clas = ONSET;
    move16();

    IF( sub(coder_type,VOICED) != 0 )
    {
        /* decode the class */
        tmpS = (Word16)get_next_indice_fx( st_fx, FEC_BITS_CLS );

        IF( tmpS == 0 )
        {
            clas = UNVOICED_CLAS;
            move16();
        }
        ELSE IF( sub(tmpS,1) == 0 )
        {
            IF( sub(last_good,VOICED_TRANSITION) >= 0 )
            {
                clas = VOICED_TRANSITION;
                move16();
            }
            ELSE
            {
                clas = UNVOICED_TRANSITION;
                move16();
            }
        }
        ELSE IF( sub(tmpS,2) == 0 )
        {
            clas = VOICED_CLAS;
            move16();
        }
    }
    ELSE
    {
        clas = VOICED_CLAS;
        move16();
    }

    /* decode the energy */
    test();
    test();
    IF( L_sub(bitrate,ACELP_14k80) >= 0 && sub(coder_type,TRANSITION) != 0 && sub(coder_type,AUDIO) < 0 )
    {
        tmpS = (Word16)get_next_indice_fx( st_fx, FEC_BITS_ENR );
        /* convert from logarithmic to linear domain (the range is 0 : 3.0 : 96 dB) */
        tmpS = mult_r(shl(tmpS,10), 24576);          /* Q10*Q13->Q8 */
        L_tmp = L_mult(tmpS, 10885);                 /* 0.332192 in Q15 */
        L_tmp = L_shr(L_tmp, 8);                    /* From Q24 to Q16 */
        sfrac = L_Extract_lc(L_tmp, &tmpS);
        *enr_q = Pow2(tmpS, sfrac);
        move32();
    }

    return clas;
}

Word16 FEC_pos_dec_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure                    */
    const Word16 L_frame,           /* i  : length of the frame                        */
    const Word16 coder_type,        /* i  : coder type                                 */
    const Word16 last_good,         /* i  : last good classfication                    */
    Word16 *last_pulse_pos,   /* o  : last glotal pulse position in the lost ACB */
    Word16 *clas,             /* o  : decoded classification                     */
    Word32 *enr_q,            /* o  : decoded energy     in Q0                   */
    const Word32 core_brate         /* i  : decoded bitrate                            */
)
{
    Word16 pitch_index, T0, T0_frac, T0_min, T0_max;
    Word16 bit_pos_pitch_index, nBits;

    T0 = 0;
    move16();
    IF( sub(coder_type,UNVOICED) > 0 )
    {
        /* decode the clas and energy information */
        IF( sub(coder_type,AUDIO) < 0 )
        {
            *clas = FEC_dec_class_fx( st_fx, core_brate, coder_type, enr_q, last_good);
            move16();

            test();
            test();
            test();
            IF( sub(coder_type,GENERIC) == 0 && sub(*clas,VOICED_CLAS) == 0 && ( sub(last_good,UNVOICED_CLAS) <= 0 || sub(last_good,INACTIVE_CLAS) == 0) )
            {
                *clas = SIN_ONSET;
                move16();
            }
        }

        test();
        IF( sub(coder_type,GENERIC) == 0 && L_sub(core_brate,ACELP_24k40) > 0 )
        {
            nBits = 0;
            move16();
            IF( sub(coder_type,AUDIO) != 0 )
            {
                /* find the number of bits */
                IF( sub(L_frame,L_FRAME) == 0 )
                {
                    nBits = ACB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, 0, 0)];
                    move16();
                }
                ELSE  /* L_frame == L_FRAME16k */
                {
                    nBits = ACB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, 0, 0)];
                    move16();
                }
            }

            /* use the absolute position of pitch index in the bitstream (this value is hard-coded and must be udpated when it changes in the encoder) */
            bit_pos_pitch_index = 71;
            move16();  /* 64 kbps WB,SWB and FB*/
            if( L_sub(core_brate,ACELP_32k) <= 0 )
            {
                bit_pos_pitch_index = 72;
                move16(); /* 32 kbp, WB*/
                if(sub(st_fx->bwidth_fx,WB) > 0)
                {
                    bit_pos_pitch_index = 73;
                    move16(); /* 32 kbp, SWB, FB*/
                }
            }

            /* retrieve the pitch index */
            pitch_index = (Word16)get_indice_fx( st_fx, bit_pos_pitch_index, nBits );

            /* decode pitch period */
            T0_min = PIT_MIN;
            move16();
            T0_max = PIT_MAX;
            move16();
            IF( sub(L_frame,L_FRAME) == 0 )
            {
                pit_Q_dec_fx( 0, pitch_index, 10, 8, 0, 1, &T0, &T0_frac, &T0_min, &T0_max );
            }
            ELSE  /* L_frame == L_FRAME16k */
            {
                pit16k_Q_dec_fx( pitch_index, 10, 1, &T0, &T0_frac, &T0_min, &T0_max );
            }

            /* decode last pulse position */
            *last_pulse_pos = (Word16)get_next_indice_fx( st_fx, FEC_BITS_POS );

            /* respect the sign */
            IF (sub(*last_pulse_pos,128) >= 0)
            {
                *last_pulse_pos = negate(s_and(*last_pulse_pos , 0x7F));
                move16();
            }
            if ( sub(T0,128) >= 0)
            {
                *last_pulse_pos = add(*last_pulse_pos,*last_pulse_pos);
                move16();
            }
        }
    }

    return T0;
}
/*----------------------------------------------------------------------*
 * Corre:
 *
 * Correlation function.  Signal x is compared to target signal y
 * Information about the similarity between vectors is returned in *gain
 *----------------------------------------------------------------------*/
static void Corre(
    const Word16 *x,    /* i  : vector 1                     Q12 */
    const Word16 *y,    /* i  : vector 2                     Q12 */
    const Word16 l,     /* i  : length of vectors                */
    Word16 *gain  /* o  : normalized correlation gain  Q15 */
)
{
    Word16 cor, cor_exp;
    Word16 den, den_exp;
    Word16 den2, den2_exp;
    Word32 tmp;
    Word16 tmp_exp;

    /* keep Q15 normalized result */
    cor = extract_h(Dot_product12(x, y, l, &cor_exp));
    den = add(extract_h(Dot_product12(y, y, l, &den_exp)), 1);
    den2 = extract_h(Dot_product12(x, x, l, &den2_exp));

    /* keep Q31 normalized result */
    tmp = L_mult(den, den2);
    tmp_exp = norm_l(tmp);
    tmp = L_shl(tmp, tmp_exp);
    tmp_exp = sub(add(den_exp, den2_exp), tmp_exp);

    tmp = Isqrt_lc(tmp, &tmp_exp);
    /* keep Q15 result */
    gain[0] = shl(mult_r(cor, extract_h(tmp)), add(cor_exp, tmp_exp));
    move16();
}

