/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*========================================================================*/
/* FUNCTION : FEC_pitch_estim_fx()										  */
/*------------------------------------------------------------------------*/
/* PURPOSE :  Estimation of pitch for FEC								  */
/*																		  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) st_fx->Opt_AMR_WB: flag indicating AMR-WB IO mode 		  */
/* _ (Word16) st_fx->L_frame_fx:  length of the frame					  */
/* _ (Word16) st_fx->clas_dec: frame classification						  */
/* _ (Word16) st_fx->last_good_fx: last good clas information     		  */
/* _ (Word16[])  pitch	  : pitch values for each subframe   	    Q6	  */
/* _ (Word16[])  old_pitch_buf:pitch values for each subframe   	Q6    */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/*------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->bfi_pitch	  : initial synthesis filter states   */
/* _ (Word16) st_fx->bfi_pitch_frame: LP filter E of last                 */
/* _ (Word16) st_fx->upd_cnt_fx: update counter                           */
/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void FEC_pitch_estim_fx(
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode             */
    const Word16 L_frame,                    /* i  : length of the frame                        */
    const Word16 clas,                       /* i  : current frame classification               */
    const Word16 last_good,                  /* i  : last good clas information                 */
    const Word16 pitch_buf[],                /* i  : Floating pitch   for each subframe         */
    const Word32 old_pitch_buf[],            /* i  : buffer of old subframe pitch values 15Q16  */
    Word16 *bfi_pitch,                 /* i/o: update of the estimated pitch for FEC      */
    Word16 *bfi_pitch_frame,           /* o  : frame length when pitch was updated        */
    Word16 *upd_cnt                    /* i/o: update counter                             */
    ,const Word16 coder_type                  /* i  : coder_type                                 */
)
{
    Word16 tmp,tmp1,tmp2,tmp3;
    Word16 tmp16k1,tmp16k2;
    Word16 tmp16k3,tmp16k4;

    tmp = mult_r(pitch_buf[1],22938);  /*Q6( 0.7f * pitch_buf[1] 0.7 in Q15)*/
    tmp1 = shl(tmp,1);                           /*Q6 (1.4f * pitch_buf[1])*/
    tmp2 = round_fx(L_shl(Mpy_32_16_1(old_pitch_buf[2*NB_SUBFR-1],22938), 6)); /*Q6 (0.7f * old_pitch_buf[2*NB_SUBFR-1])*/
    tmp3 = shl(tmp2,1);								  /*Q6 (1.4f * old_pitch_buf[2*NB_SUBFR-1])*/

    tmp16k1 = round_fx(L_shl(Mpy_32_16_1(old_pitch_buf[2*NB_SUBFR16k-1],22938), 6));  /*Q6 0.7f * old_pitch_buf[2*NB_SUBFR16k-1]*/
    tmp16k2 = shl(tmp16k1,1);                               /*Q6 1.4f * old_pitch_buf[2*NB_SUBFR16k-1]*/
    tmp16k3 = mult_r(shl(tmp16k1,1),20480);       /*Q6 1.25*0.7f * old_pitch_buf[2*NB_SUBFR16k-1]*/
    tmp16k4 = mult_r(shl(tmp16k2,1),20480);       /*Q6 1.25*1.4f * old_pitch_buf[2*NB_SUBFR16k-1]*/


    test();
    test();
    test();
    IF( (sub(clas,VOICED_CLAS) == 0 && sub(last_good,VOICED_TRANSITION) >= 0) || (Opt_AMR_WB && sub(clas,VOICED_CLAS) >= 0) )
    {
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( ( (sub(pitch_buf[3],tmp1) < 0) && (sub(pitch_buf[3],tmp) > 0) &&
              (sub(pitch_buf[1],tmp3) < 0) && (sub(pitch_buf[3],tmp2) > 0) &&
              (sub(L_frame,L_FRAME) == 0) ) ||
            ( (sub(pitch_buf[3],tmp1) < 0) && (sub(pitch_buf[3],tmp) > 0) &&
              (sub(pitch_buf[1],tmp16k4) < 0) && (sub(pitch_buf[3],tmp16k3) > 0) &&
              (sub(L_frame,L_FRAME16k) == 0) )
            || (sub(coder_type, TRANSITION) == 0) )
        {
            *bfi_pitch = pitch_buf[shr(L_frame,6)-1];
            move16();
            *bfi_pitch_frame = L_frame;
            move16();
            *upd_cnt = 0;
            move16();
        }
    }
}

