/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"       /* */

#include "stl.h"

/*
 * E_GAIN_f_pitch_sharpening
 *
 * Parameters:
 *    x            I/O: impulse response (or algebraic code)
 *    pit_lag        I: pitch lag
 *
 * Function:
 *    Performs Pitch sharpening routine for one subframe.
 *    pitch sharpening factor is 0.85
 *
 * Returns:
 *    void
 */
static void E_GAIN_f_pitch_sharpening(Word16 *x, Word16 pit_lag, Word16 L_subfr)
{
    Word16 i, tmp;

    FOR (i = pit_lag; i < L_subfr; i++)
    {
        /*x[i] += x[i - pit_lag] * F_PIT_SHARP;*/
        tmp = mult_r(x[i - pit_lag], FL2WORD16(F_PIT_SHARP));
        x[i] = add(x[i],tmp);
        move16();
    }
    return;
}

/*-------------------------------------------------------------------*
  * cb_shape()
  *
  * pre-emphasis, pitch sharpening and formant sharpening of the algebraic codebook
  *-------------------------------------------------------------------*/

void cb_shape_fx(
    const Word16 preemphFlag,          /* i  : flag for pre-emphasis                           */
    const Word16 pitchFlag,            /* i  : flag for pitch sharpening                       */
    const Word16 scramblingFlag,       /* i  : flag for phase scrambling                       */
    const Word16 sharpFlag,            /* i  : flag for formant sharpening                     */
    const Word16 formantTiltFlag,      /* i  : flag for formant tilt                           */
    const Word16 g1,                   /* i  : formant sharpening numerator weighting          */
    const Word16 g2,                   /* i  : formant sharpening denominator weighting        */
    const Word16 *p_Aq,                /* i  : LP filter coefficients                          */
    Word16 *code,                /* i/o: signal to shape                                 */
    const Word16 tilt_code,            /* i  : tilt of code                                    */
    const Word16 pt_pitch              /* i  : pointer to current subframe fractional pitch    */
)
{

    Word16 tmp, buff[L_SUBFR+M], A_num[M+1], A_den[M+1];
    Word16 i;
    Word32 L_tmp;

    tmp = 0;
    move16();

    /* Pre-emphasis */
    IF( preemphFlag )
    {
        preemph_copy_fx(code, code, tilt_code, L_SUBFR, &tmp);
    }

    /* pitch sharpening */
    IF( pitchFlag )
    {
        E_GAIN_f_pitch_sharpening( code, pt_pitch, L_SUBFR );
    }

    /* phase scrambling filter */
    IF( scramblingFlag )
    {
        buff[0] = code[0];
        move16();
        FOR (i = 1; i < L_SUBFR; i++)
        {
            buff[i]=code[i];
            move16();
            /*code[i] = 0.7f*buff[i] + buff[i-1] - 0.7f*code[i-1];      */
            L_tmp = L_mult(22938, buff[i]);
            tmp = mac_r(L_tmp,-22938, code[i-1]);
            code[i] = add(tmp,buff[i-1]);
            move16();
        }
    }

    test();
    IF ( sharpFlag || formantTiltFlag  )
    {
        weight_a_fx( p_Aq, A_num, g1, M );
        weight_a_fx( p_Aq, A_den, g2, M );

        IF( formantTiltFlag  )
        {
            Word16 tilt;
            Word16 mu;

            Copy(A_num, buff+M, M+1);

            E_UTIL_synthesis(1, A_den, buff+M, buff+M, L_SUBFR, buff, 0, M);

            /*Compute tilt of formant enhancement*/
            tilt = extract_l(L_shr(get_gain(buff+M+1, buff+M, L_SUBFR-1),1));

            /*Combine tilt of code and fe*/
            tmp = 0;
            move16();
            /*mu = 0.5f*tilt_code-0.25f*tilt;*/
            mu = sub(shr(tilt_code,1),shr(tilt,2));
            preemph_copy_fx(code, code, mu, L_SUBFR, &tmp);
        }
        ELSE
        {
            Copy( code, buff, L_SUBFR );

            Overflow = 0;
            move16();
            Residu3_lc_fx(A_num, M, buff, code, L_SUBFR, 0);
            {
                syn_filt_s_lc_fx(0, A_den, code, code, L_SUBFR);
            }
        }
    }

    return;
}


/*-------------------------------------------------------------------*
  * cb_shape()
  *
  * pre-emphasis, pitch sharpening and formant sharpening of the algebraic codebook
  *-------------------------------------------------------------------*/

void E_UTIL_cb_shape(
    const Word16 preemphFlag,          /* i  : flag for pre-emphasis                           */
    const Word16 pitchFlag,            /* i  : flag for pitch sharpening                       */
    const Word16 scramblingFlag,       /* i  : flag for phase scrambling                       */
    const Word16 formantFlag,          /* i  : use formant enhancement                     Q15 */
    const Word16 formantTiltFlag,      /* i  : use tilt of formant enhancement              Q0 */
    const Word16 g1,                   /* i  : formant sharpening numerator weighting          */
    const Word16 g2,                   /* i  : formant sharpening denominator weighting        */
    const Word16 *Aq,                  /* i  : quantized LPC coefficients                 3Q12 */
    Word16 *code,                /* i/o:                                            <14b */
    const Word16 tilt_code,            /* i  : tilt factor                                 Q15 */
    const Word16 pitch                 /* i  : pointer to current subframe fractional pitch    */
)
{
    Word16 tmp;
    Word16 buff[M+L_SUBFR], A_num[M+1], A_den[M+1];
    Word32 Ltmp;
    Word16 i;
    /* Pre-emphasis */
    IF( preemphFlag )
    {
        tmp = 0;
        move16();
        preemph_copy_fx(code, code, tilt_code, L_SUBFR, &tmp);
    }

    /* Pitch sharpening */
    IF( pitchFlag )
    {
        E_GAIN_f_pitch_sharpening(code, pitch, L_SUBFR);
    }

    IF( scramblingFlag )
    {
        buff[0]=code[0];
        move16();
        FOR (i = 1; i < L_SUBFR; i++)
        {
            buff[i]=code[i];
            move16();
            /*code[i] = 0.7f*buff[i] + buff[i-1] - 0.7f*code[i-1];      */
            Ltmp = L_mult(22938, buff[i]);
            tmp = mac_r(Ltmp,-22938, code[i-1]);
            code[i] = add(tmp,buff[i-1]);
            move16();
        }
    }

    /* Formant enhancement */
    test();
    IF( formantFlag || formantTiltFlag )
    {
        weight_a_fx( Aq, A_num, g1, M );
        weight_a_fx( Aq, A_den, g2, M );

        /*set_zero(buff, lpcorder+L_subfr);*/
        set16_fx(buff, 0, M+L_SUBFR);

        IF(formantTiltFlag)
        {
            Word16 tilt;
            Word16 mu;

            /*copyFLOAT(A_num, buff+M, M+1);*/
            Copy(A_num, buff+M, M+1);

            E_UTIL_synthesis(1, A_den, buff+M, buff+M, L_SUBFR, buff, 0, M);

            /*Compute tilt of formant enhancement*/
            tilt = extract_l(L_shr(get_gain(buff+M+1, buff+M, L_SUBFR-1),1));

            /*Combine tilt of code and fe*/
            tmp = 0;
            move16();
            /*mu = 0.5f*tilt_code-0.25f*tilt;*/
            mu = sub(shr(tilt_code,1),shr(tilt,2));
            preemph_copy_fx(code, code, mu, L_SUBFR, &tmp);
        }
        ELSE
        /*formant enhancement*/
        {
            Copy(code, buff+M, L_SUBFR);
            Residu3_fx(A_num, buff+M, code, L_SUBFR, 1);
            E_UTIL_synthesis(1, A_den, code, code, L_SUBFR, buff, 0, M);
        }
    }

    return;
}

