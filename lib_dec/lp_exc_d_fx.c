/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*======================================================================*/
/* FUNCTION : lp_filt_exc_dec_fx()                                      */
/*-----------------------------------------------------------------------*/
/* PURPOSE :  Low-pass filtering of the adaptive exctitation             */
/*                                                                       */
/*-----------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :													 */
/* _ (Word32) core_brate : Core bitrate Q0                               */
/* _ (Word16) Opt_AMR_WB : flag indicating AMR-WB IO mode   Q0           */
/* _ (Word16) coder_type : coding type  Q0                               */
/* _ (Word16) i_subfr : subframe index  Q0                               */
/* _ (Word16) L_subfr :  subframe size Q0                               */
/*-----------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                    */
/*-----------------------------------------------------------------------*/
/* INPUT OUTPUT ARGUMENTS                                                */
/* _ (Word16 *) exc : excitation buffer Q0                                 */
/*-----------------------------------------------------------------------*/

/* 															             */
/*-----------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                    */
/* _ None                                                                */
/*=======================================================================*/

void lp_filt_exc_dec_fx(
    Decoder_State_fx *st_fx,                  /* i/o: decoder state structure */
    const Word16 codec_type,                 /* i : coder type                                       */
    const Word32  core_brate,                /* i  : core bitrate                                    */
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode                  */
    const Word16 coder_type,                 /* i  : coding type                                     */
    const Word16 i_subfr,                    /* i  : subframe index                                  */
    const Word16 L_subfr,                    /* i  : subframe size                                   */
    const Word16 L_frame,                    /* i  : frame size                                      */
    Word16 lp_flag,                    /* i  : operation mode signalling                       */
    Word16 *exc
)
{

    Word16 i, fac_n, fac_m;
    Word16 code[L_FRAME];
    Word32 L_tmp;

    /*-----------------------------------------------------------------*
     * Select LP filtering of the adaptive excitation
     *-----------------------------------------------------------------*/
    IF( sub(codec_type, MODE1) == 0)
    {
        test();
        test();
        test();
        IF ( ( Opt_AMR_WB || sub(coder_type,GENERIC) == 0|| sub(coder_type,TRANSITION) == 0) && L_sub(core_brate,ACELP_11k60) < 0)
        {
            lp_flag = LOW_PASS;
            move16();
        }
        ELSE IF ( L_sub(core_brate,ACELP_11k60) >= 0)
        {
            lp_flag = (Word16)get_next_indice_fx( st_fx, 1 );
            move16();
        }
        ELSE
        {
            lp_flag = FULL_BAND;
            move16();
        }
    }

    IF ( sub(lp_flag, LOW_PASS) == 0)
    {
        /* pointer positionning to avoid doing it inside the loop */
        test();
        IF(codec_type==MODE2 && L_frame==L_FRAME16k)
        {
            fac_n = FL2WORD16(0.21f);
            fac_m = FL2WORD16(0.58f);
        }
        ELSE
        {
            fac_n = FL2WORD16(0.18f);
            fac_m = FL2WORD16(0.64f);
        }

        FOR (i=0; i<L_subfr; i++)
        {
            L_tmp   = L_mult(      fac_n, exc[i-1+i_subfr]);
            L_tmp   = L_mac(L_tmp, fac_m, exc[i+0+i_subfr]);
            code[i] = mac_r(L_tmp, fac_n, exc[i+1+i_subfr]);
            move16();
        }


        Copy(code, &exc[i_subfr], L_subfr);
    }
}
