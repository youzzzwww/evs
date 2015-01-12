/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*======================================================================*/
/* FUNCTION : inov_decode_fx() */
/*-----------------------------------------------------------------------*/
/* PURPOSE :  Decode the algebraic innovation and do pitch sharpening    */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :													 */
/* _ (Word32) core_brate : Core bitrate Q0                               */
/* _ (Word16) Opt_AMR_WB : flag indicating AMR-WB IO mode  Q0            */
/* _ (Word16) L_frame : length of the frame  Q0                          */
/* _ (Word16) i_subfr : length of the frame  Q0                          */
/* _ (Word16) coder_type :  coding type                                  */
/* _ (Word16) L_subfr : subframe length                                  */
/* _ (Word16) sharpFlag :  formant sharpening flag                       */
/* _ (Word16) tc_subfr : TC subframe index                               */
/* _ (Word16 *) p_Aq : LP filter coefficients Q12                        */
/* _ (Word16) tilt_code : tilt of the excitation of previous subframe Q15*/
/* _ (Word16) pt_pitch :   current subframe fractional pitch Q6          */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/* _ (Word16 *[]) code : subframe length Q12                             */
/* _ (Word16 []) index_buf_4T : subframe length                          */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ None                                                                */
/*=======================================================================*/

void inov_decode_fx(
    Decoder_State_fx *st_fx,            /* i/o: decoder state structure */
    const Word32  core_brate,       /* i  : core bitrate                                */
    const Word16 Opt_AMR_WB,        /* i  : flag indicating AMR-WB IO mode              */
    const Word16 L_frame,           /* i  : length of the frame                         */
    const Word16 coder_type,        /* i  : coding type                                 */
    const Word16 sharpFlag,         /* i  : formant sharpening flag                     */
    const Word16 i_subfr,           /* i  : subframe index                              */
    const Word16 tc_subfr,          /* i  : TC subframe index                           */
    const Word16 *p_Aq,             /* i  : LP filter coefficients Q12                  */
    const Word16 tilt_code,         /* i  : tilt of the excitation of previous subframe Q15 */
    const Word16 pt_pitch,          /* i  : pointer to current subframe fractional pitch Q6*/
    Word16 *code              /* o  : algebraic excitation                        */
)
{
    Word16 nBits;
    Word16 g1, g2;

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        g1 = FORMANT_SHARPENING_G1;
        g2 = FORMANT_SHARPENING_G2;
    }
    ELSE
    {
        g1 = FORMANT_SHARPENING_G1_16k;
        g2 = FORMANT_SHARPENING_G2_16k;
    }

    IF ( !Opt_AMR_WB )
    {
        IF( sub(L_frame, L_FRAME) == 0)
        {
            nBits = FCB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_fx(tc_subfr))];
            move16();
            test();
            if( sub(coder_type, INACTIVE) == 0 && L_sub(core_brate,ACELP_24k40) > 0)
            {
                nBits = 12;
                move16();
            }
        }
        ELSE  /* L_frame == L_FRAME16k */
        {
            nBits = FCB_bits_16kHz_tbl[BIT_ALLOC_IDX_16KHZ_fx(core_brate, coder_type, i_subfr, TC_SUBFR2IDX_16KHZ_fx(tc_subfr))];
            move16();
        }
        IF(sub(nBits,7) == 0)
        {
            dec_acelp_1t64_fx(st_fx, code);
        }
        ELSE IF( sub(nBits,12) == 0)
        {
            dec_acelp_2t32_fx( st_fx, code );
        }
        ELSE
        {
            dec_acelp_4t64_fx( st_fx, nBits, code, Opt_AMR_WB );
        }
    }
    ELSE
    {
        IF ( L_sub(core_brate,ACELP_6k60) == 0)
        {
            dec_acelp_2t32_fx( st_fx, code );
        }
        ELSE IF ( L_sub(core_brate,ACELP_8k85) == 0 )
        {
            dec_acelp_4t64_fx( st_fx, 20, code, Opt_AMR_WB );
        }
        ELSE IF ( L_sub(core_brate,ACELP_12k65) == 0)
        {
            dec_acelp_4t64_fx( st_fx, 36, code, Opt_AMR_WB );
        }
        ELSE IF ( L_sub(core_brate,ACELP_14k25) == 0)
        {
            dec_acelp_4t64_fx( st_fx, 44, code, Opt_AMR_WB );
        }
        ELSE IF ( L_sub(core_brate,ACELP_15k85) == 0)
        {
            dec_acelp_4t64_fx( st_fx, 52, code, Opt_AMR_WB );
        }
        ELSE IF ( L_sub(core_brate,ACELP_18k25) == 0)
        {
            dec_acelp_4t64_fx( st_fx, 64, code, Opt_AMR_WB );
        }
        ELSE IF ( L_sub(core_brate,ACELP_19k85) == 0)
        {
            dec_acelp_4t64_fx( st_fx, 72, code, Opt_AMR_WB );
        }
        ELSE
        {
            dec_acelp_4t64_fx( st_fx, 88, code, Opt_AMR_WB );
        }
    }

    cb_shape_fx( 1, 1, 0, sharpFlag, 0, g1, g2, p_Aq, code, tilt_code, shr(add(pt_pitch,26),6) );

    return;

}
