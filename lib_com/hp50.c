/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "options.h"


#define HP20_COEFF_SCALE    (2)

/*
 * hp20
 *
 * Function:
 *    2nd order high pass filter with nominal cut off frequency at 20 Hz.
 *
 * Returns:
 *    void
 */

static Word32 HP50_Mode2_Mpy_32_16_fix(Word32 a, Word16 b)
{
    Word32 result = Mpy_32_16_1(a,b);
    /* perform rounding towards lower value for negative results */
    if (result < 0)    result = L_add(result,1);
    return result;
}

static Word32 HP50_Mpy_32_32_fix(Word32 a, Word32 b)
{
    Word32 result = Mpy_32_32(a,b);
    /* perform rounding towards lower value for negative results */
    if (result < 0)    result = L_add(result,1);
    return result;
}


static void filter_2nd_order(
    Word16 signal[],
    const Word16 stride,
    const Word16 prescale,
    const Word16 lg,
    Word32 mem[4],
    Word32 a1,
    Word32 a2,
    Word32 b1,
    Word32 b2
)
{

    Word16 i;
    Word16 x2, x1;
    Word32 L_sum, L_y1, L_y2;


    /*
     * Saturation: The states of the filter, namely L_y1 and L_y2 shall
     * never saturate, because that causes error in the filter feedback.
     * The final output written into signal[] might saturate because of
     * unavoidable filter overshoot.
     */

    /* Execute first 2 iterations with 32-bit x anx y memory values */
    BASOP_SATURATE_ERROR_ON
    L_sum = HP50_Mpy_32_32_fix(b2,mem[2]);                 /* b2*x2 */
    L_sum = L_add(L_sum,HP50_Mpy_32_32_fix(b1,mem[3]));    /* b1*x1 */
    x2 = shr(signal[0*stride], prescale);
    L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b2,x2));        /* b2*x0 */
    L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(mem[0],a2));   /* y2*a2 */
    L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(mem[1],a1));   /* y1*a1 */

    L_y2  = L_shl(L_sum, HP20_COEFF_SCALE);
    BASOP_SATURATE_ERROR_OFF
    BASOP_SATURATE_WARNING_OFF
    signal[0*stride] = round_fx(L_shl(L_y2, prescale));
    BASOP_SATURATE_WARNING_ON

    BASOP_SATURATE_ERROR_ON
    L_sum = HP50_Mpy_32_32_fix(b2,mem[3]);                 /* b2*x2 */
    L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b1,x2));        /* b1*x1 */
    x1 = shr(signal[1*stride], prescale);
    L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b2,x1));        /* b2*x0 */
    L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(mem[1],a2));   /* y2*a2 */
    L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(L_y2,  a1));   /* y1*a1 */

    L_y1  = L_shl(L_sum, HP20_COEFF_SCALE);
    BASOP_SATURATE_ERROR_OFF
    BASOP_SATURATE_WARNING_OFF
    signal[1*stride] = round_fx(L_shl(L_y1, prescale));
    BASOP_SATURATE_WARNING_ON

    /* New we use a trick and toggle x1/x2 and L_y1/L_y2 to save a few cycles unrolling the loop by 2 */
    FOR (i = 2; i < lg; i+=2)
    {
        /* y[i+0] = b2*x[i-2] + b1*x[i-1] + b2*x[i-0] + a2*y[i-2] + a1*y[i-1];  */
        BASOP_SATURATE_ERROR_ON
        L_sum = HP50_Mode2_Mpy_32_16_fix(b2,x2);
        L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b1,x1));
        x2    = shr(signal[i*stride], prescale);
        L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b2,x2));
        L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(L_y2,a2));
        L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(L_y1,a1));

        L_y2  = L_shl(L_sum, HP20_COEFF_SCALE);
        BASOP_SATURATE_ERROR_OFF
        BASOP_SATURATE_WARNING_OFF
        signal[i*stride] = round_fx(L_shl(L_y2, prescale));
        BASOP_SATURATE_WARNING_ON
        /* y[i+1] = b2*x[i-1] + b1*x[i-0] + b2*x[i+1] + a2*y[i-1] + a1*y[i+0];  */
        BASOP_SATURATE_ERROR_ON
        L_sum = HP50_Mode2_Mpy_32_16_fix(b2,x1);
        L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b1,x2));
        x1    = shr(signal[(i+1)*stride], prescale);
        L_sum = L_add(L_sum,HP50_Mode2_Mpy_32_16_fix(b2,x1));
        L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(L_y1,a2));
        L_sum = L_add(L_sum, HP50_Mpy_32_32_fix(L_y2,a1));

        L_y1  = L_shl(L_sum, HP20_COEFF_SCALE);
        BASOP_SATURATE_ERROR_OFF
        BASOP_SATURATE_WARNING_OFF
        signal[(i+1)*stride] = round_fx(L_shl(L_y1, prescale));
        BASOP_SATURATE_WARNING_ON
    }
    /* update static filter memory from variables */
    mem[0] = L_y2;
    move32();
    mem[1] = L_y1;
    move32();
    mem[2] = L_deposit_h(x2);
    mem[3] = L_deposit_h(x1);


    return;
}


void hp20(Word16 signal[],     /* i/o: signal to filter                   any */
          const Word16 stride,       /* i  : stride to be applied accessing signal  */
          const Word16 lg,           /* i  : length of signal (integer)          Q0 */
          Word32 mem[4],       /* i/o: static filter memory with this layout: */
          /*      mem[0]: y[-2] (32-bit)                 */
          /*      mem[1]; y[-1] (32-bit)                 */
          /*      mem[2]: x[-2] << 16                    */
          /*      mem[3]: x[-1] << 16                    */
          /* Note: mem[0..3] need to be scaled per frame */
          const Word32 sFreq)        /* i  : input sampling rate                 Q0 */
{
    Word32 a1, b1, a2, b2;
    Word16 prescale, prescaleOld, diff;



    prescale = getScaleFactor16(signal, lg);
    prescaleOld = extract_l(mem[4]);
    diff = norm_l(L_shl(mem[2], prescaleOld));
    if (mem[2] != 0)
    {
        prescale = s_min(prescale, diff);
    }
    diff = norm_l(L_shl(mem[3], prescaleOld));
    if (mem[3] != 0)
    {
        prescale = s_min(prescale, diff);
    }
    prescale = s_max(-12, sub(1, prescale));
    IF (prescale != prescaleOld)
    {
        diff = sub(prescale, prescaleOld);
        mem[0] = L_shr(mem[0], diff);
        move32();
        mem[1] = L_shr(mem[1], diff);
        move32();
        mem[2] = L_shr(mem[2], diff);
        move32();
        mem[3] = L_shr(mem[3], diff);
        move32();
        mem[4] = L_deposit_l(prescale);
    }

    IF ( L_sub(sFreq,8000) == 0 )
    {
        /* hp filter 20Hz at 3dB for 8000 Hz input sampling rate
           [b,a] = butter(2, 20.0/4000.0, 'high');
           b = [0.988954248067140  -1.977908496134280   0.988954248067140]
           a = [1.000000000000000  -1.977786483776764   0.978030508491796]*/
        a1 = L_add(0,FL2WORD32_SCALE( 1.977786483776764, HP20_COEFF_SCALE));
        a2 = L_add(0,FL2WORD32_SCALE(-0.978030508491796, HP20_COEFF_SCALE));
        b1 = L_add(0,FL2WORD32_SCALE(-1.977908496134280, HP20_COEFF_SCALE));
        b2 = L_add(0,FL2WORD32_SCALE( 0.988954248067140, HP20_COEFF_SCALE));

    }
    ELSE IF ( L_sub(sFreq,12800) == 0)
    {
        /* hp filter 20Hz at 3dB for 12800 Hz input sampling rate */
        a1 = L_add(0,FL2WORD32_SCALE( 1.98611621154089, HP20_COEFF_SCALE));
        a2 = L_add(0,FL2WORD32_SCALE(-0.98621192916075, HP20_COEFF_SCALE));
        b1 = L_add(0,FL2WORD32_SCALE(-1.98616407035082, HP20_COEFF_SCALE));
        b2 = L_add(0,FL2WORD32_SCALE( 0.99308203517541, HP20_COEFF_SCALE));
    }

    ELSE IF ( L_sub(sFreq,16000) == 0 )
    {
        /* hp filter 20Hz at 3dB for 16000KHz sampling rate
           [b,a] = butter(2, 20.0/8000.0, 'high');
           b = [0.994461788958195  -1.988923577916390   0.994461788958195]
           a = [1.000000000000000  -1.988892905899653   0.988954249933127] */
        a1 = L_add(0,FL2WORD32_SCALE( 1.988892905899653, HP20_COEFF_SCALE));
        a2 = L_add(0,FL2WORD32_SCALE(-0.988954249933127, HP20_COEFF_SCALE));
        b1 = L_add(0,FL2WORD32_SCALE(-1.988923577916390, HP20_COEFF_SCALE));
        b2 = L_add(0,FL2WORD32_SCALE( 0.994461788958195, HP20_COEFF_SCALE));

    }
    ELSE IF ( L_sub(sFreq,32000) == 0 )
    {
        /* hp filter 20Hz at 3dB for 32000KHz sampling rate
           [b,a] = butter(2, 20.0/16000.0, 'high');
           b = [0.997227049904470  -1.994454099808940   0.997227049904470]
           a = [1.000000000000000  -1.994446410541927   0.994461789075954]*/
        a1 = L_add(0,FL2WORD32_SCALE( 1.994446410541927, HP20_COEFF_SCALE));
        a2 = L_add(0,FL2WORD32_SCALE(-0.994461789075954, HP20_COEFF_SCALE));
        b1 = L_add(0,FL2WORD32_SCALE(-1.994454099808940, HP20_COEFF_SCALE));
        b2 = L_add(0,FL2WORD32_SCALE( 0.997227049904470, HP20_COEFF_SCALE));
    }
    ELSE
    {
        assert (sFreq == 48000);
        /* hp filter 20Hz at 3dB for 48000KHz sampling rate
           [b,a] = butter(2, 20.0/24000.0, 'high');
           b =[0.998150511190452  -1.996301022380904   0.998150511190452]
           a =[1.000000000000000  -1.996297601769122   0.996304442992686]*/
        a1 = L_add(0,FL2WORD32_SCALE( 1.996297601769122, HP20_COEFF_SCALE));
        a2 = L_add(0,FL2WORD32_SCALE(-0.996304442992686, HP20_COEFF_SCALE));
        b1 = L_add(0,FL2WORD32_SCALE(-1.996301022380904, HP20_COEFF_SCALE));
        b2 = L_add(0,FL2WORD32_SCALE( 0.998150511190452, HP20_COEFF_SCALE));
    }


    filter_2nd_order(signal,
                     stride,
                     prescale,
                     lg,
                     mem,
                     a1,
                     a2,
                     b1,
                     b2);

    return;
}




