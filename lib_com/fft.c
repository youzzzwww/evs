/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"
#include "options.h"
#include "stl.h"


/**
 * \brief Profiling / Precision results
 *
 *        Profiling / Precision of complex valued FFTs: BASOP_cfft()
 *
 *                       WOPS FLC  WOPS BASOP  BASOP/FLC  Precision BASOP
 *        FFT5                 48          87     1.81       16.96
 *        FFT8                108         108     1.00       17.04
 *        FFT10             1.093         194     0.18       16.70
 *        FFT15               240         354     1.47       16.97
 *        FFT16               196         288     1.46       16.62
 *        FFT20             1.509         368     0.24       16.06
 *        FFT30               682         828     1.21       16.80
 *        FFT32               504         752     1.49       15.45   (cplx mult mit 3 mult und 3 add)
 *        FFT32               504         824     1.63       16.07   (cplx mult mit 4 mult und 2 add)
 *        FFT64  ( 8x 8)    3.824       3.129     0.82       15.16
 *        FFT80  (10x 8)    5.295       4.385     0.82       15.55
 *        FFT100 (20x 5)    7.163       6.518     0.91       15.65
 *        FFT120 (15x 8)   12.176       7.029     0.58       15.38
 *        FFT128 (16x 8)    8.872       6.777     0.76       15.28
 *        FFT160 (20x 8)   12.934       9.033     0.70       14.95
 *        FFT240 (30x 8)   12.403      14.961     1.21       15.49
 *        FFT256 (32x 8)   17.925      14.905     0.83       14.61   (cplx mult mit 3 mult und 3 add)
 *        FFT256 (32x 8)   17.925      15.265     0.85       15.04   (cplx mult mit 4 mult und 2 add)
 *        FFT320 (20x16)   25.092      21.517     0.86       15.21
 *
 *
 *        Profiling / Precision of real valued FFTs / iFFTs: BASOP_rfft()
 *
 *                       WOPS FLC  WOPS BASOP  BASOP/FLC  Precision BASOP
 *        rFFT40             1744         955     0.55       15.68
 *        rFFT64             2251        1635     0.73       16.17
 *
 *        irFFT40            1762        1116     0.63       15.36
 *        irFFT64            2269        1759     0.78       15.18
 *
 */


#define  Mpy_32_xx    Mpy_32_16_1

#define FFTC(x)       WORD322WORD16((Word32)x)

#define C31   (FFTC(0x91261468))      /* FL2WORD32( -0.86602540) -sqrt(3)/2 */

#define C51   (FFTC(0x79bc3854))      /* FL2WORD32( 0.95105652)   */
#define C52   (FFTC(0x9d839db0))      /* FL2WORD32(-1.53884180/2) */
#define C53   (FFTC(0xd18053ce))      /* FL2WORD32(-0.36327126)   */
#define C54   (FFTC(0x478dde64))      /* FL2WORD32( 0.55901699)   */
#define C55   (FFTC(0xb0000001))      /* FL2WORD32(-1.25/2)       */

#define C81   (FFTC(0x5a82799a))      /* FL2WORD32( 7.071067811865475e-1) */
#define C82   (FFTC(0xa57d8666))      /* FL2WORD32(-7.071067811865475e-1) */

#define C161  (FFTC(0x5a82799a))      /* FL2WORD32( 7.071067811865475e-1)  INV_SQRT2    */
#define C162  (FFTC(0xa57d8666))      /* FL2WORD32(-7.071067811865475e-1) -INV_SQRT2    */

#define C163  (FFTC(0x7641af3d))      /* FL2WORD32( 9.238795325112867e-1)  COS_PI_DIV8  */
#define C164  (FFTC(0x89be50c3))      /* FL2WORD32(-9.238795325112867e-1) -COS_PI_DIV8  */

#define C165  (FFTC(0x30fbc54d))      /* FL2WORD32( 3.826834323650898e-1)  COS_3PI_DIV8 */
#define C166  (FFTC(0xcf043ab3))      /* FL2WORD32(-3.826834323650898e-1) -COS_3PI_DIV8 */



#define cplxMpy3_0(a,b,c,d)           as = L_shr(a,1); \
                                      bs = L_shr(b,1); \
                                      a  = L_sub(Mpy_32_xx(as,c),Mpy_32_xx(bs,d)); \
                                      b  = L_add(Mpy_32_xx(as,d),Mpy_32_xx(bs,c));


#define cplxMpy4_4_0(re,im,a,b,c,d)   re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTOR60-SCALEFACTOR15); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTOR60-SCALEFACTOR15);

#define cplxMpy4_4_1(re,im,a,b)       re = L_shr(a,SCALEFACTOR60-SCALEFACTOR15); \
                                      im = L_shr(b,SCALEFACTOR60-SCALEFACTOR15);

#define cplxMpy4_8_0(re,im,a,b,c,d)   re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),1); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),1);

#define cplxMpy4_8_1(re,im,a,b)       re = L_shr(a,1); \
                                      im = L_shr(b,1);
/*
re = a*c - b*d
im = a*d + b*c
*/
#if (SCALEFACTOR20 == SCALEFACTOR10)
#define cplxMpy4_10_0(re,im,a,b,c,d)  re = L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)); move32(); \
                                      im = L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)); move32();

#define cplxMpy4_10_1(re,im,a,b)      re = (a); move32(); \
                                      im = (b); move32();
#else
#define cplxMpy4_10_0(re,im,a,b,c,d)  re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTOR20-SCALEFACTOR10); move32(); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTOR20-SCALEFACTOR10); move32();

#define cplxMpy4_10_1(re,im,a,b)      re = L_shr(a,SCALEFACTOR20-SCALEFACTOR10); move32(); \
                                      im = L_shr(b,SCALEFACTOR20-SCALEFACTOR10); move32();
#endif


#if (SCALEFACTOR20 == SCALEFACTOR30)
#define cplxMpy4_20_30_0(re,im,a,b,c,d)  re = L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)); move32(); \
                                        im = L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)); move32();

#define cplxMpy4_20_30_1(re,im,a,b)      re = (a); move32(); \
                                        im = (b); move32();
#else
#define cplxMpy4_20_30_0(re,im,a,b,c,d)  re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTOR20-SCALEFACTOR30); move32(); \
                                        im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTOR20-SCALEFACTOR30); move32();

#define cplxMpy4_20_30_1(re,im,a,b)      re = L_shr(a,SCALEFACTOR20-SCALEFACTOR30); move32(); \
                                        im = L_shr(b,SCALEFACTOR20-SCALEFACTOR30); move32();
#endif


#define cplxMpy4_12_0(re,im,a,b,c,d)  re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTOR16-SCALEFACTOR12); move32(); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTOR16-SCALEFACTOR12); move32();

#define cplxMpy4_12_1(re,im,a,b)      re = L_shr(a,SCALEFACTOR16-SCALEFACTOR12); move32(); \
                                      im = L_shr(b,SCALEFACTOR16-SCALEFACTOR12); move32();

#define cplxMpy4_16_0(re,im,a,b,c,d)  re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTOR16); move32(); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTOR16); move32();

#define cplxMpy4_16_1(re,im,a,b)      re = L_shr(a,SCALEFACTOR16); move32(); \
                                      im = L_shr(b,SCALEFACTOR16); move32();


#define cplxMpy4_5_0(re,im,a,b,c,d)   re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),SCALEFACTORN2); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),SCALEFACTORN2);

#define cplxMpy4_5_1(re,im,a,b)       re = L_shr(a,SCALEFACTORN2); \
                                      im = L_shr(b,SCALEFACTORN2);



/**
 * \brief    Function performs a complex 2-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR2 bits.
 *
 *           WOPS with 32x16 bit multiplications:  16 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft2(Word32 *re, Word32 *im /*, Word16 s*/)
{
    Word32 y00,y01,y02,y03;



    y00 = L_shr(re[0],SCALEFACTOR2);
    y01 = L_shr(im[0],SCALEFACTOR2);
    y02 = L_shr(re[1],SCALEFACTOR2);
    y03 = L_shr(im[1],SCALEFACTOR2);

    re[0] = L_add(y00,y02);
    move32();
    im[0] = L_add(y01,y03);
    move32();
    re[1] = L_sub(y00,y02);
    move32();
    im[1] = L_sub(y01,y03);
    move32();

}


/**
 * \brief    Function performs a complex 3-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR3 bits.
 *
 *           WOPS with 32x16 bit multiplications:  36 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft3(Word32 *re, Word32 *im, Word16 s)
{
    Word32 x0,x1,x2;
    Word32 y0,y1,y2;
    Word32 r1,r2,s1,s2;



    x0 = L_shr(re[s*0],SCALEFACTOR3);
    x1 = L_shr(re[s*1],SCALEFACTOR3);
    x2 = L_shr(re[s*2],SCALEFACTOR3);

    /* real part */
    r1 = L_add(x1,x2);
    r2 = Mpy_32_xx(L_sub(x1,x2),C31);
    re[0] = L_add(x0,r1);
    move32();
    r1 = L_sub(x0,L_shr(r1,1));

    /* imaginary part */
    y0 = L_shr(im[s*0],SCALEFACTOR3);
    y1 = L_shr(im[s*1],SCALEFACTOR3);
    y2 = L_shr(im[s*2],SCALEFACTOR3);

    s1 = L_add(y1,y2);
    s2 = Mpy_32_xx(L_sub(y1,y2),C31);
    im[0] = L_add(y0,s1);
    move32();
    s1 = L_sub(y0,L_shr(s1,1));

    /* combination */
    re[s*1] = L_sub(r1,s2);
    move32();
    re[s*2] = L_add(r1,s2);
    move32();
    im[s*1] = L_add(s1,r2);
    move32();
    im[s*2] = L_sub(s1,r2);
    move32();

}


/**
 * \brief    Function performs a complex 4-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR4 bits.
 *
 *           WOPS with 32x16 bit multiplications:  40 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft4(Word32 *re, Word32 *im, Word16 s)
{
    Word32 y00,y01,y02,y03;
    Word32 y04,y05,y06,y07;
    Word32 t0,t1,t2,t3;
    Word32 t4,t5,t6,t7;



    y00 = L_shr(re[s*0],SCALEFACTOR4);
    y01 = L_shr(im[s*0],SCALEFACTOR4);
    y02 = L_shr(re[s*1],SCALEFACTOR4);
    y03 = L_shr(im[s*1],SCALEFACTOR4);
    y04 = L_shr(re[s*2],SCALEFACTOR4);
    y05 = L_shr(im[s*2],SCALEFACTOR4);
    y06 = L_shr(re[s*3],SCALEFACTOR4);
    y07 = L_shr(im[s*3],SCALEFACTOR4);

    /* Pre-additions */
    t0 = L_add(y00,y04);
    t2 = L_sub(y00,y04);
    t1 = L_add(y01,y05);
    t3 = L_sub(y01,y05);
    t4 = L_add(y02,y06);
    t7 = L_sub(y02,y06);
    t5 = L_add(y07,y03);
    t6 = L_sub(y07,y03);

    /* Post-additions */
    re[s*0] = L_add(t0,t4);
    move32();
    im[s*0] = L_add(t1,t5);
    move32();
    re[s*1] = L_sub(t2,t6);
    move32();
    im[s*1] = L_sub(t3,t7);
    move32();
    re[s*2] = L_sub(t0,t4);
    move32();
    im[s*2] = L_sub(t1,t5);
    move32();
    re[s*3] = L_add(t2,t6);
    move32();
    im[s*3] = L_add(t3,t7);
    move32();

}


/**
 * \brief    Function performs a complex 5-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR5 bits.
 *
 *           WOPS FLC version:                     48 cycles
 *           WOPS with 32x16 bit multiplications:  88 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft5(Word32 *re, Word32 *im, Word16 s)
{
    Word32 x0,x1,x2,x3,x4;
    Word32 r1,r2,r3,r4;
    Word32 s1,s2,s3,s4;
    Word32 t;


    /*  */

    /* real part */
    x0   = L_shr(re[s*0],SCALEFACTOR5);
    x1   = L_shr(re[s*1],SCALEFACTOR5);
    x2   = L_shr(re[s*2],SCALEFACTOR5);
    x3   = L_shr(re[s*3],SCALEFACTOR5);
    x4   = L_shr(re[s*4],SCALEFACTOR5);

    r1   = L_add(x1,x4);
    r4   = L_sub(x1,x4);
    r3   = L_add(x2,x3);
    r2   = L_sub(x2,x3);
    t    = Mpy_32_xx(L_sub(r1,r3),C54);
    r1   = L_add(r1,r3);
    re[0] = L_add(x0,r1);
    move32();
    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
       the values as fracts */
    r1   = L_add(re[0],(L_shl(Mpy_32_xx(r1,C55),1)));
    r3   = L_sub(r1,t);
    r1   = L_add(r1,t);
    t    = Mpy_32_xx(L_add(r4,r2),C51);
    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
       the values as fracts */
    r4   = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2   = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0   = L_shr(im[s*0],SCALEFACTOR5);
    x1   = L_shr(im[s*1],SCALEFACTOR5);
    x2   = L_shr(im[s*2],SCALEFACTOR5);
    x3   = L_shr(im[s*3],SCALEFACTOR5);
    x4   = L_shr(im[s*4],SCALEFACTOR5);

    s1   = L_add(x1,x4);
    s4   = L_sub(x1,x4);
    s3   = L_add(x2,x3);
    s2   = L_sub(x2,x3);
    t    = Mpy_32_xx(L_sub(s1,s3),C54);
    s1   = L_add(s1,s3);
    im[0] = L_add(x0,s1);
    move32();
    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
       the values as fracts */
    s1   = L_add(im[0],L_shl(Mpy_32_xx(s1,C55),1));
    s3   = L_sub(s1,t);
    s1   = L_add(s1,t);
    t    = Mpy_32_xx(L_add(s4,s2),C51);
    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
       the values as fracts */
    s4   = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2   = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    re[s*1] = L_add(r1,s2);
    move32();
    re[s*4] = L_sub(r1,s2);
    move32();
    re[s*2] = L_sub(r3,s4);
    move32();
    re[s*3] = L_add(r3,s4);
    move32();

    im[s*1] = L_sub(s1,r2);
    move32();
    im[s*4] = L_add(s1,r2);
    move32();
    im[s*2] = L_add(s3,r4);
    move32();
    im[s*3] = L_sub(s3,r4);
    move32();

    /*  */
}


/**
 * \brief    Function performs a complex 8-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR8 bits.
 *
 *           WOPS FLC version:                    108 cycles
 *           WOPS with 32x16 bit multiplications: 108 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft8(Word32 *re, Word32 *im, Word16 s)
{
    Word32 x00,x01,x02,x03,x04,x05,x06,x07;
    Word32 x08,x09,x10,x11,x12,x13,x14,x15;
    Word32 t00,t01,t02,t03,t04,t05,t06,t07;
    Word32 t08,t09,t10,t11,t12,t13,t14,t15;
    Word32 s00,s01,s02,s03,s04,s05,s06,s07;
    Word32 s08,s09,s10,s11,s12,s13,s14,s15;



    /* Pre-additions */

    x00 = L_shr(re[s*0],SCALEFACTOR8);
    x01 = L_shr(im[s*0],SCALEFACTOR8);
    x02 = L_shr(re[s*1],SCALEFACTOR8);
    x03 = L_shr(im[s*1],SCALEFACTOR8);
    x04 = L_shr(re[s*2],SCALEFACTOR8);
    x05 = L_shr(im[s*2],SCALEFACTOR8);
    x06 = L_shr(re[s*3],SCALEFACTOR8);
    x07 = L_shr(im[s*3],SCALEFACTOR8);
    x08 = L_shr(re[s*4],SCALEFACTOR8);
    x09 = L_shr(im[s*4],SCALEFACTOR8);
    x10 = L_shr(re[s*5],SCALEFACTOR8);
    x11 = L_shr(im[s*5],SCALEFACTOR8);
    x12 = L_shr(re[s*6],SCALEFACTOR8);
    x13 = L_shr(im[s*6],SCALEFACTOR8);
    x14 = L_shr(re[s*7],SCALEFACTOR8);
    x15 = L_shr(im[s*7],SCALEFACTOR8);

    t00 = L_add(x00,x08);
    t02 = L_sub(x00,x08);
    t01 = L_add(x01,x09);
    t03 = L_sub(x01,x09);
    t04 = L_add(x02,x10);
    t06 = L_sub(x02,x10);
    t05 = L_add(x03,x11);
    t07 = L_sub(x03,x11);
    t08 = L_add(x04,x12);
    t10 = L_sub(x04,x12);
    t09 = L_add(x05,x13);
    t11 = L_sub(x05,x13);
    t12 = L_add(x06,x14);
    t14 = L_sub(x06,x14);
    t13 = L_add(x07,x15);
    t15 = L_sub(x07,x15);

    /* Pre-additions and core multiplications */

    s00 = L_add(t00,t08);
    s04 = L_sub(t00,t08);
    s01 = L_add(t01,t09);
    s05 = L_sub(t01,t09);
    s08 = L_sub(t02,t11);
    s10 = L_add(t02,t11);
    s09 = L_add(t03,t10);
    s11 = L_sub(t03,t10);
    s02 = L_add(t04,t12);
    s07 = L_sub(t04,t12);
    s03 = L_add(t05,t13);
    s06 = L_sub(t13,t05);

    t01 = L_add(t06,t14);
    t02 = L_sub(t06,t14);
    t00 = L_add(t07,t15);
    t03 = L_sub(t07,t15);

    s12 = Mpy_32_xx(L_add(t00,t02),C81);
    s14 = Mpy_32_xx(L_sub(t00,t02),C81);
    s13 = Mpy_32_xx(L_sub(t03,t01),C81);
    s15 = Mpy_32_xx(L_add(t01,t03),C82);

    /* Post-additions */

    re[s*0] = L_add(s00,s02);
    move32();
    re[s*4] = L_sub(s00,s02);
    move32();
    im[s*0] = L_add(s01,s03);
    move32();
    im[s*4] = L_sub(s01,s03);
    move32();
    re[s*2] = L_sub(s04,s06);
    move32();
    re[s*6] = L_add(s04,s06);
    move32();
    im[s*2] = L_sub(s05,s07);
    move32();
    im[s*6] = L_add(s05,s07);
    move32();
    re[s*3] = L_add(s08,s14);
    move32();
    re[s*7] = L_sub(s08,s14);
    move32();
    im[s*3] = L_add(s09,s15);
    move32();
    im[s*7] = L_sub(s09,s15);
    move32();
    re[s*1] = L_add(s10,s12);
    move32();
    re[s*5] = L_sub(s10,s12);
    move32();
    im[s*1] = L_add(s11,s13);
    move32();
    im[s*5] = L_sub(s11,s13);
    move32();

}


/**
 * \brief    Function performs a complex 10-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR10 bits.
 *
 *           WOPS FLC version:                    1093 cycles
 *           WOPS with 32x16 bit multiplications:  196 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft10(Word32 *re, Word32 *im, Word16 s)
{
    Word32 t;
    Word32 x0,x1,x2,x3,x4;
    Word32 r1,r2,r3,r4;
    Word32 s1,s2,s3,s4;
    Word32 y00,y01,y02,y03,y04,y05,y06,y07,y08,y09;
    Word32 y10,y11,y12,y13,y14,y15,y16,y17,y18,y19;



    /* 2 fft5 stages */

    /* real part */
    x0  = L_shr(re[s*0],SCALEFACTOR10);
    x1  = L_shr(re[s*2],SCALEFACTOR10);
    x2  = L_shr(re[s*4],SCALEFACTOR10);
    x3  = L_shr(re[s*6],SCALEFACTOR10);
    x4  = L_shr(re[s*8],SCALEFACTOR10);

    r1  = L_add(x3,x2);
    r4  = L_sub(x3,x2);
    r3  = L_add(x1,x4);
    r2  = L_sub(x1,x4);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y00 = L_add(x0,r1);
    r1  = L_add(y00,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s*0],SCALEFACTOR10);
    x1  = L_shr(im[s*2],SCALEFACTOR10);
    x2  = L_shr(im[s*4],SCALEFACTOR10);
    x3  = L_shr(im[s*6],SCALEFACTOR10);
    x4  = L_shr(im[s*8],SCALEFACTOR10);

    s1  = L_add(x3,x2);
    s4  = L_sub(x3,x2);
    s3  = L_add(x1,x4);
    s2  = L_sub(x1,x4);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y01 = L_add(x0,s1);
    s1  = L_add(y01,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y04 = L_add(r1,s2);
    y16 = L_sub(r1,s2);
    y08 = L_sub(r3,s4);
    y12 = L_add(r3,s4);

    y05 = L_sub(s1,r2);
    y17 = L_add(s1,r2);
    y09 = L_add(s3,r4);
    y13 = L_sub(s3,r4);

    /* real part */
    x0  = L_shr(re[s*5],SCALEFACTOR10);
    x1  = L_shr(re[s*1],SCALEFACTOR10);
    x2  = L_shr(re[s*3],SCALEFACTOR10);
    x3  = L_shr(re[s*7],SCALEFACTOR10);
    x4  = L_shr(re[s*9],SCALEFACTOR10);

    r1  = L_add(x1,x4);
    r4  = L_sub(x1,x4);
    r3  = L_add(x3,x2);
    r2  = L_sub(x3,x2);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y02 = L_add(x0,r1);
    r1  = L_add(y02,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s*5],SCALEFACTOR10);
    x1  = L_shr(im[s*1],SCALEFACTOR10);
    x2  = L_shr(im[s*3],SCALEFACTOR10);
    x3  = L_shr(im[s*7],SCALEFACTOR10);
    x4  = L_shr(im[s*9],SCALEFACTOR10);

    s1  = L_add(x1,x4);
    s4  = L_sub(x1,x4);
    s3  = L_add(x3,x2);
    s2  = L_sub(x3,x2);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y03 = L_add(x0,s1);
    s1  = L_add(y03,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y06 = L_add(r1,s2);
    y18 = L_sub(r1,s2);
    y10 = L_sub(r3,s4);
    y14 = L_add(r3,s4);

    y07 = L_sub(s1,r2);
    y19 = L_add(s1,r2);
    y11 = L_add(s3,r4);
    y15 = L_sub(s3,r4);

    /* 5 fft2 stages */
    re[s*0] = L_add(y00,y02);
    move32();
    im[s*0] = L_add(y01,y03);
    move32();
    re[s*5] = L_sub(y00,y02);
    move32();
    im[s*5] = L_sub(y01,y03);
    move32();

    re[s*2] = L_add(y04,y06);
    move32();
    im[s*2] = L_add(y05,y07);
    move32();
    re[s*7] = L_sub(y04,y06);
    move32();
    im[s*7] = L_sub(y05,y07);
    move32();

    re[s*4] = L_add(y08,y10);
    move32();
    im[s*4] = L_add(y09,y11);
    move32();
    re[s*9] = L_sub(y08,y10);
    move32();
    im[s*9] = L_sub(y09,y11);
    move32();

    re[s*6] = L_add(y12,y14);
    move32();
    im[s*6] = L_add(y13,y15);
    move32();
    re[s*1] = L_sub(y12,y14);
    move32();
    im[s*1] = L_sub(y13,y15);
    move32();

    re[s*8] = L_add(y16,y18);
    move32();
    im[s*8] = L_add(y17,y19);
    move32();
    re[s*3] = L_sub(y16,y18);
    move32();
    im[s*3] = L_sub(y17,y19);
    move32();

}



static void fft12(Word32 *pInput)
{
    Word32 aDst[24];
    Word32 *pSrc, *pDst;
    Word16 i;
    Word32 r1,r2,s1,s2,pD;
    Word32 re, im;
    Word16 vre, vim;



    move16();
    move16();
    pSrc = pInput;
    pDst = aDst;

    /* First 3*2 samples are shifted right by 2 before output */
    r1      = L_add(L_shr(pSrc[8], 2), L_shr(pSrc[16], 2));
    r2      = Mpy_32_16_1(L_sub(L_shr(pSrc[8], 2), L_shr(pSrc[16], 2)), C31);
    pD      = L_shr(pSrc[0], 2);
    pDst[0] = L_shr(L_add(pD, r1), 1);
    move32();
    r1      = L_sub(pD, L_shr(r1, 1));

    /* imaginary part */
    s1      = L_add(L_shr(pSrc[9], 2), L_shr(pSrc[17], 2));
    s2      = Mpy_32_16_1(L_sub(L_shr(pSrc[9], 2), L_shr(pSrc[17], 2)), C31);
    pD      = L_shr(pSrc[1], 2);
    pDst[1] = L_shr(L_add(pD, s1), 1);
    move32();
    s1      = L_sub(pD, L_shr(s1,1));

    r1 = L_shr(r1, 1);
    r2 = L_shr(r2, 1);
    s1 = L_shr(s1, 1);
    s2 = L_shr(s2, 1);

    /* combination */
    pDst[2] = L_sub(r1, s2);
    move32();
    pDst[3] = L_add(s1, r2);
    move32();
    pDst[4] = L_add(r1, s2);
    move32();
    pDst[5] = L_sub(s1, r2);
    move32();
    pSrc += 2;
    pDst += 6;

    vre = add(0x6eda,0);
    vim = add(0x4000,0);

    FOR(i=0; i<2; i++)
    {
        /* sample 0,1 are shifted right by 2 before output */
        /* sample 2,3 4,5 are shifted right by 1 and complex multiplied before output */

        r1      = L_add(L_shr(pSrc[8],2), L_shr(pSrc[16],2));
        r2      = Mpy_32_16_1(L_sub(L_shr(pSrc[8],2), L_shr(pSrc[16],2)), C31);
        pD      = L_shr(pSrc[0], 2);
        pDst[0] = L_shr(L_add(pD, r1), 1);
        move32();
        r1      = L_sub(pD, L_shr(r1, 1));

        /* imaginary part */
        s1      = L_add(L_shr(pSrc[9],2), L_shr(pSrc[17],2));
        s2      = Mpy_32_16_1(L_sub(L_shr(pSrc[9],2), L_shr(pSrc[17],2)), C31);
        pD      = L_shr(pSrc[1], 2);
        pDst[1] = L_shr(L_add(pD, s1), 1);
        move32();
        s1      = L_sub(pD, L_shr(s1,1));

        r1 = L_shr(r1, 1);
        r2 = L_shr(r2, 1);
        s1 = L_shr(s1, 1);
        s2 = L_shr(s2, 1);

        /* combination */
        re = L_sub(r1, s2);
        im = L_add(s1, r2);

        cplxMpy_32_16(&pDst[3], &pDst[2], im, re, vre, vim);
        re = L_add(r1, s2);
        im = L_sub(s1, r2);

        vre = add(0x4000,0);
        if (i == 1)
            vre = negate(vre);  /* 0xC000 */
        if (i==0)
            vim = add(0x6eda,0);

        cplxMpy_32_16(&pDst[5], &pDst[4], im, re, vre, vim);

        pDst += 6;
        pSrc += 2;
    }
    /* sample 0,1 are shifted right by 2 before output */
    /* sample 2,3 is shifted right by 1 and complex multiplied with (0.0,+1.0) */
    /* sample 4,5 is shifted right by 1 and complex multiplied with (-1.0,0.0) */
    r1      = L_add(L_shr(pSrc[8], 2), L_shr(pSrc[16], 2));
    r2      = Mpy_32_16_1(L_sub(L_shr(pSrc[8], 2), L_shr(pSrc[16], 2)), C31);
    pD      = L_shr(pSrc[0], 2);
    pDst[0] = L_shr(L_add(pD,  r1), 1);
    move32();
    r1      = L_sub(pD, L_shr(r1, 1));

    /* imaginary part */
    s1      = L_add(L_shr(pSrc[9], 2), L_shr(pSrc[17], 2));
    s2      = Mpy_32_16_1(L_sub(L_shr(pSrc[9], 2), L_shr(pSrc[17], 2)), C31);
    pD      = L_shr(pSrc[1], 2);
    pDst[1] = L_shr(L_add(pD, s1), 1);
    move32();
    s1      = L_sub(pD, L_shr(s1, 1));

    r1 = L_shr(r1, 1);
    r2 = L_shr(r2, 1);
    s1 = L_shr(s1, 1);
    s2 = L_shr(s2, 1);

    /* combination */
    move32();
    move32();
    move32();
    move32();
    pDst[2] = L_add(s1, r2);
    pDst[3] = L_sub(s2, r1);
    pDst[4] = L_negate(L_add(r1, s2));
    pDst[5] = L_sub(r2, s1);
    /* Perform 3 times the fft of length 4. The input samples are at the address of aDst and the
     output samples are at the address of pInput. The input vector for the fft of length 4 is built
     of the interleaved samples in aDst, the output samples are stored consecutively at the address
     of pInput.
     */
    move16();
    move16();
    pSrc = aDst;
    pDst = pInput;
    FOR(i=0; i<3; i++)
    {
        /* inline FFT4 merged with incoming resorting loop */
        r1 = L_add(L_shr(pSrc[0], 2), L_shr(pSrc[12], 2));  /* Re A + Re B */
        r2 = L_add(L_shr(pSrc[6], 2), L_shr(pSrc[18], 2));  /* Re C + Re D */
        s1 = L_add(L_shr(pSrc[1], 2), L_shr(pSrc[13], 2));  /* Im A + Im B */
        s2 = L_add(L_shr(pSrc[7], 2), L_shr(pSrc[19], 2));  /* Im C + Im D */

        pDst[0] = L_add(r1, r2);         /* Re A' = Re A + Re B + Re C + Re D */   move32();
        pDst[1] = L_add(s1, s2);         /* Im A' = Im A + Im B + Im C + Im D */   move32();

        re = L_sub(r1, L_shr(pSrc[12], 1));          /* Re A - Re B */
        im = L_sub(s1, L_shr(pSrc[13], 1));          /* Im A - Im B */

        pDst[12] = L_sub(r1, r2);        /* Re C' = Re A + Re B - Re C - Re D */   move32();
        pDst[13] = L_sub(s1, s2);        /* Im C' = Im A + Im B - Im C - Im D */   move32();

        r2 = L_sub(r2, L_shr(pSrc[18], 1));           /* Re C - Re D */
        s2 = L_sub(s2, L_shr(pSrc[19], 1));           /* Im C - Im D */

        pDst[ 6] = L_add(re, s2);       /* Re B' = Re A - Re B + Im C - Im D */    move32();
        pDst[18] = L_sub(re, s2);       /* Re D' = Re A - Re B - Im C + Im D */    move32();
        pDst[ 7] = L_sub(im, r2);       /* Im B' = Im A - Im B - Re C + Re D */    move32();
        pDst[19] = L_add(im, r2);       /* Im D' = Im A - Im B + Re C - Re D */    move32();

        pSrc += 2;
        pDst += 2;
    }

}

/**
 * \brief    Function performs a complex 15-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR15 bits.
 *
 *           WOPS FLC version:                     240 cycles
 *           WOPS with 32x16 bit multiplications:  354 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft15(Word32 *re, Word32 *im, Word16 s)
{
    Word32 t;
    Word32 r1,r2,r3,r4;
    Word32 s1,s2,s3,s4;
    Word32 x00,x01,x02,x03,x04,x05,x06,x07,x08,x09;
    Word32 x10,x11,x12,x13,x14,x15,x16,x17,x18,x19;
    Word32 x20,x21,x22,x23,x24,x25,x26,x27,x28,x29;
    Word32 y00,y01,y02,y03,y04,y05,y06,y07,y08,y09;
    Word32 y10,y11,y12,y13,y14,y15,y16,y17,y18,y19;
    Word32 y20,y21,y22,y23,y24,y25,y26,y27,y28,y29;



    x00 = L_shr(re[s* 0],SCALEFACTOR15);
    x01 = L_shr(im[s* 0],SCALEFACTOR15);
    x02 = L_shr(re[s* 3],SCALEFACTOR15);
    x03 = L_shr(im[s* 3],SCALEFACTOR15);
    x04 = L_shr(re[s* 6],SCALEFACTOR15);
    x05 = L_shr(im[s* 6],SCALEFACTOR15);
    x06 = L_shr(re[s* 9],SCALEFACTOR15);
    x07 = L_shr(im[s* 9],SCALEFACTOR15);
    x08 = L_shr(re[s*12],SCALEFACTOR15);
    x09 = L_shr(im[s*12],SCALEFACTOR15);

    x10 = L_shr(re[s* 5],SCALEFACTOR15);
    x11 = L_shr(im[s* 5],SCALEFACTOR15);
    x12 = L_shr(re[s* 8],SCALEFACTOR15);
    x13 = L_shr(im[s* 8],SCALEFACTOR15);
    x14 = L_shr(re[s*11],SCALEFACTOR15);
    x15 = L_shr(im[s*11],SCALEFACTOR15);
    x16 = L_shr(re[s*14],SCALEFACTOR15);
    x17 = L_shr(im[s*14],SCALEFACTOR15);
    x18 = L_shr(re[s* 2],SCALEFACTOR15);
    x19 = L_shr(im[s* 2],SCALEFACTOR15);

    x20 = L_shr(re[s*10],SCALEFACTOR15);
    x21 = L_shr(im[s*10],SCALEFACTOR15);
    x22 = L_shr(re[s*13],SCALEFACTOR15);
    x23 = L_shr(im[s*13],SCALEFACTOR15);
    x24 = L_shr(re[s* 1],SCALEFACTOR15);
    x25 = L_shr(im[s* 1],SCALEFACTOR15);
    x26 = L_shr(re[s* 4],SCALEFACTOR15);
    x27 = L_shr(im[s* 4],SCALEFACTOR15);
    x28 = L_shr(re[s* 7],SCALEFACTOR15);
    x29 = L_shr(im[s* 7],SCALEFACTOR15);

    /* 1. FFT5 stage */

    /* real part */
    r1  = L_add(x02,x08);
    r4  = L_sub(x02,x08);
    r3  = L_add(x04,x06);
    r2  = L_sub(x04,x06);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y00 = L_add(x00,r1);
    r1  = L_add(y00,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x03,x09);
    s4  = L_sub(x03,x09);
    s3  = L_add(x05,x07);
    s2  = L_sub(x05,x07);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y01 = L_add(x01,s1);
    s1  = L_add(y01,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y02 = L_add(r1,s2);
    y08 = L_sub(r1,s2);
    y04 = L_sub(r3,s4);
    y06 = L_add(r3,s4);

    y03 = L_sub(s1,r2);
    y09 = L_add(s1,r2);
    y05 = L_add(s3,r4);
    y07 = L_sub(s3,r4);

    /* 2. FFT5 stage */

    /* real part */
    r1  = L_add(x12,x18);
    r4  = L_sub(x12,x18);
    r3  = L_add(x14,x16);
    r2  = L_sub(x14,x16);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y10 = L_add(x10,r1);
    r1  = L_add(y10,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x13,x19);
    s4  = L_sub(x13,x19);
    s3  = L_add(x15,x17);
    s2  = L_sub(x15,x17);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y11 = L_add(x11,s1);
    s1  = L_add(y11,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y12 = L_add(r1,s2);
    y18 = L_sub(r1,s2);
    y14 = L_sub(r3,s4);
    y16 = L_add(r3,s4);

    y13 = L_sub(s1,r2);
    y19 = L_add(s1,r2);
    y15 = L_add(s3,r4);
    y17 = L_sub(s3,r4);

    /* 3. FFT5 stage */

    /* real part */
    r1  = L_add(x22,x28);
    r4  = L_sub(x22,x28);
    r3  = L_add(x24,x26);
    r2  = L_sub(x24,x26);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y20 = L_add(x20,r1);
    r1  = L_add(y20,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x23,x29);
    s4  = L_sub(x23,x29);
    s3  = L_add(x25,x27);
    s2  = L_sub(x25,x27);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y21 = L_add(x21,s1);
    s1  = L_add(y21,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y22 = L_add(r1,s2);
    y28 = L_sub(r1,s2);
    y24 = L_sub(r3,s4);
    y26 = L_add(r3,s4);

    y23 = L_sub(s1,r2);
    y29 = L_add(s1,r2);
    y25 = L_add(s3,r4);
    y27 = L_sub(s3,r4);

    /* 1. FFT3 stage */

    /* real part */
    r1 = L_add(y10,y20);
    r2 = Mpy_32_xx(L_sub(y10,y20),C31);
    re[s*0] = L_add(y00,r1);
    move32();
    r1 = L_sub(y00,L_shr(r1,1));

    /* imaginary part */
    s1 = L_add(y11,y21);
    s2 = Mpy_32_xx(L_sub(y11,y21),C31);
    im[s*0] = L_add(y01,s1);
    move32();
    s1 = L_sub(y01,L_shr(s1,1));

    /* combination */
    re[s*10] = L_sub(r1,s2);
    move32();
    re[s* 5] = L_add(r1,s2);
    move32();
    im[s*10] = L_add(s1,r2);
    move32();
    im[s* 5] = L_sub(s1,r2);
    move32();

    /* 2. FFT3 stage */

    /* real part */
    r1 = L_add(y12,y22);
    r2 = Mpy_32_xx(L_sub(y12,y22),C31);
    re[s*6] = L_add(y02,r1);
    move32();
    r1 = L_sub(y02,L_shr(r1,1));

    /* imaginary part */
    s1 = L_add(y13,y23);
    s2 = Mpy_32_xx(L_sub(y13,y23),C31);
    im[s*6] = L_add(y03,s1);
    move32();
    s1 = L_sub(y03,L_shr(s1,1));

    /* combination */
    re[s* 1] = L_sub(r1,s2);
    move32();
    re[s*11] = L_add(r1,s2);
    move32();
    im[s* 1] = L_add(s1,r2);
    move32();
    im[s*11] = L_sub(s1,r2);
    move32();

    /* 3. FFT3 stage */

    /* real part */
    r1 = L_add(y14,y24);
    r2 = Mpy_32_xx(L_sub(y14,y24),C31);
    re[s*12] = L_add(y04,r1);
    move32();
    r1 = L_sub(y04,L_shr(r1,1));

    /* imaginary part */
    s1 = L_add(y15,y25);
    s2 = Mpy_32_xx(L_sub(y15,y25),C31);
    im[s*12] = L_add(y05,s1);
    move32();
    s1 = L_sub(y05,L_shr(s1,1));

    /* combination */
    re[s* 7] = L_sub(r1,s2);
    move32();
    re[s* 2] = L_add(r1,s2);
    move32();
    im[s* 7] = L_add(s1,r2);
    move32();
    im[s* 2] = L_sub(s1,r2);
    move32();

    /* 4. FFT3 stage */

    /* real part */
    r1 = L_add(y16,y26);
    r2 = Mpy_32_xx(L_sub(y16,y26),C31);
    re[s*3] = L_add(y06,r1);
    move32();
    r1 = L_sub(y06,L_shr(r1,1));

    /* imaginary part */
    s1 = L_add(y17,y27);
    s2 = Mpy_32_xx(L_sub(y17,y27),C31);
    im[s*3] = L_add(y07,s1);
    move32();
    s1 = L_sub(y07,L_shr(s1,1));

    /* combination */
    re[s*13] = L_sub(r1,s2);
    move32();
    re[s* 8] = L_add(r1,s2);
    move32();
    im[s*13] = L_add(s1,r2);
    move32();
    im[s* 8] = L_sub(s1,r2);
    move32();

    /* 5. FFT3 stage */

    /* real part */
    r1 = L_add(y18,y28);
    r2 = Mpy_32_xx(L_sub(y18,y28),C31);
    re[s*9] = L_add(y08,r1);
    move32();
    r1 = L_sub(y08,L_shr(r1,1));

    /* imaginary part */
    s1 = L_add(y19,y29);
    s2 = Mpy_32_xx(L_sub(y19,y29),C31);
    im[s*9] = L_add(y09,s1);
    move32();
    s1 = L_sub(y09,L_shr(s1,1));

    /* combination */
    re[s* 4] = L_sub(r1,s2);
    move32();
    re[s*14] = L_add(r1,s2);
    move32();
    im[s* 4] = L_add(s1,r2);
    move32();
    im[s*14] = L_sub(s1,r2);
    move32();

}


/**
 * \brief    Function performs a complex 16-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR16 bits.
 *
 *           WOPS FLC version:                                 196 cycles
 *           WOPS with 32x16 bit multiplications (scale on ):  288 cycles
 *           WOPS with 32x16 bit multiplications (scale off):  256 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
void fft16(Word32 *re, Word32 *im, Word16 s, Word16 bScale)
{
    Word32 x0,x1,x2,x3,x4,x5,x6,x7;
    Word32 t0,t1,t2,t3,t4,t5,t6,t7;
    Word32 y00,y01,y02,y03,y04,y05,y06,y07;
    Word32 y08,y09,y10,y11,y12,y13,y14,y15;
    Word32 y16,y17,y18,y19,y20,y21,y22,y23;
    Word32 y24,y25,y26,y27,y28,y29,y30,y31;



    IF (bScale)
    {
        x0 = L_shr(re[s* 0],SCALEFACTOR16);
        x1 = L_shr(im[s* 0],SCALEFACTOR16);
        x2 = L_shr(re[s* 4],SCALEFACTOR16);
        x3 = L_shr(im[s* 4],SCALEFACTOR16);
        x4 = L_shr(re[s* 8],SCALEFACTOR16);
        x5 = L_shr(im[s* 8],SCALEFACTOR16);
        x6 = L_shr(re[s*12],SCALEFACTOR16);
        x7 = L_shr(im[s*12],SCALEFACTOR16);

        /* Pre-additions */
        t0 = L_add(x0,x4);
        t2 = L_sub(x0,x4);
        t1 = L_add(x1,x5);
        t3 = L_sub(x1,x5);
        t4 = L_add(x2,x6);
        t7 = L_sub(x2,x6);
        t5 = L_add(x7,x3);
        t6 = L_sub(x7,x3);

        /* Post-additions */
        y00 = L_add(t0,t4);
        y01 = L_add(t1,t5);
        y02 = L_sub(t2,t6);
        y03 = L_sub(t3,t7);
        y04 = L_sub(t0,t4);
        y05 = L_sub(t1,t5);
        y06 = L_add(t2,t6);
        y07 = L_add(t3,t7);

        x0 = L_shr(re[s* 1],SCALEFACTOR16);
        x1 = L_shr(im[s* 1],SCALEFACTOR16);
        x2 = L_shr(re[s* 5],SCALEFACTOR16);
        x3 = L_shr(im[s* 5],SCALEFACTOR16);
        x4 = L_shr(re[s* 9],SCALEFACTOR16);
        x5 = L_shr(im[s* 9],SCALEFACTOR16);
        x6 = L_shr(re[s*13],SCALEFACTOR16);
        x7 = L_shr(im[s*13],SCALEFACTOR16);

        /* Pre-additions */
        t0 = L_add(x0,x4);
        t2 = L_sub(x0,x4);
        t1 = L_add(x1,x5);
        t3 = L_sub(x1,x5);
        t4 = L_add(x2,x6);
        t7 = L_sub(x2,x6);
        t5 = L_add(x7,x3);
        t6 = L_sub(x7,x3);

        /* Post-additions */
        y08 = L_add(t0,t4);
        y09 = L_add(t1,t5);
        y10 = L_sub(t2,t6);
        y11 = L_sub(t3,t7);
        y12 = L_sub(t0,t4);
        y13 = L_sub(t1,t5);
        y14 = L_add(t2,t6);
        y15 = L_add(t3,t7);

        x0 = L_shr(re[s* 2],SCALEFACTOR16);
        x1 = L_shr(im[s* 2],SCALEFACTOR16);
        x2 = L_shr(re[s* 6],SCALEFACTOR16);
        x3 = L_shr(im[s* 6],SCALEFACTOR16);
        x4 = L_shr(re[s*10],SCALEFACTOR16);
        x5 = L_shr(im[s*10],SCALEFACTOR16);
        x6 = L_shr(re[s*14],SCALEFACTOR16);
        x7 = L_shr(im[s*14],SCALEFACTOR16);

        /* Pre-additions */
        t0 = L_add(x0,x4);
        t2 = L_sub(x0,x4);
        t1 = L_add(x1,x5);
        t3 = L_sub(x1,x5);
        t4 = L_add(x2,x6);
        t7 = L_sub(x2,x6);
        t5 = L_add(x7,x3);
        t6 = L_sub(x7,x3);

        /* Post-additions */
        y16 = L_add(t0,t4);
        y17 = L_add(t1,t5);
        y18 = L_sub(t2,t6);
        y19 = L_sub(t3,t7);
        y20 = L_sub(t1,t5);
        y21 = L_sub(t4,t0);
        y22 = L_add(t2,t6);
        y23 = L_add(t3,t7);

        x0 = L_shr(re[s* 3],SCALEFACTOR16);
        x1 = L_shr(im[s* 3],SCALEFACTOR16);
        x2 = L_shr(re[s* 7],SCALEFACTOR16);
        x3 = L_shr(im[s* 7],SCALEFACTOR16);
        x4 = L_shr(re[s*11],SCALEFACTOR16);
        x5 = L_shr(im[s*11],SCALEFACTOR16);
        x6 = L_shr(re[s*15],SCALEFACTOR16);
        x7 = L_shr(im[s*15],SCALEFACTOR16);

        /* Pre-additions */
        t0 = L_add(x0,x4);
        t2 = L_sub(x0,x4);
        t1 = L_add(x1,x5);
        t3 = L_sub(x1,x5);
        t4 = L_add(x2,x6);
        t7 = L_sub(x2,x6);
        t5 = L_add(x7,x3);
        t6 = L_sub(x7,x3);

        /* Post-additions */
        y24 = L_add(t0,t4);
        y25 = L_add(t1,t5);
        y26 = L_sub(t2,t6);
        y27 = L_sub(t3,t7);
        y28 = L_sub(t0,t4);
        y29 = L_sub(t1,t5);
        y30 = L_add(t2,t6);
        y31 = L_add(t3,t7);
    }
    ELSE
    {
        /* Pre-additions */
        t0 = L_add(re[s* 0],re[s* 8]);
        t2 = L_sub(re[s* 0],re[s* 8]);
        t1 = L_add(im[s* 0],im[s* 8]);
        t3 = L_sub(im[s* 0],im[s* 8]);
        t4 = L_add(re[s* 4],re[s*12]);
        t7 = L_sub(re[s* 4],re[s*12]);
        t5 = L_add(im[s*12],im[s* 4]);
        t6 = L_sub(im[s*12],im[s* 4]);

        /* Post-additions */
        y00 = L_add(t0,t4);
        y01 = L_add(t1,t5);
        y02 = L_sub(t2,t6);
        y03 = L_sub(t3,t7);
        y04 = L_sub(t0,t4);
        y05 = L_sub(t1,t5);
        y06 = L_add(t2,t6);
        y07 = L_add(t3,t7);

        /* Pre-additions */
        t0 = L_add(re[s* 1],re[s* 9]);
        t2 = L_sub(re[s* 1],re[s* 9]);
        t1 = L_add(im[s* 1],im[s* 9]);
        t3 = L_sub(im[s* 1],im[s* 9]);
        t4 = L_add(re[s* 5],re[s*13]);
        t7 = L_sub(re[s* 5],re[s*13]);
        t5 = L_add(im[s*13],im[s* 5]);
        t6 = L_sub(im[s*13],im[s* 5]);

        /* Post-additions */
        y08 = L_add(t0,t4);
        y09 = L_add(t1,t5);
        y10 = L_sub(t2,t6);
        y11 = L_sub(t3,t7);
        y12 = L_sub(t0,t4);
        y13 = L_sub(t1,t5);
        y14 = L_add(t2,t6);
        y15 = L_add(t3,t7);

        /* Pre-additions */
        t0 = L_add(re[s* 2],re[s*10]);
        t2 = L_sub(re[s* 2],re[s*10]);
        t1 = L_add(im[s* 2],im[s*10]);
        t3 = L_sub(im[s* 2],im[s*10]);
        t4 = L_add(re[s* 6],re[s*14]);
        t7 = L_sub(re[s* 6],re[s*14]);
        t5 = L_add(im[s*14],im[s* 6]);
        t6 = L_sub(im[s*14],im[s* 6]);

        /* Post-additions */
        y16 = L_add(t0,t4);
        y17 = L_add(t1,t5);
        y18 = L_sub(t2,t6);
        y19 = L_sub(t3,t7);
        y20 = L_sub(t1,t5);
        y21 = L_sub(t4,t0);
        y22 = L_add(t2,t6);
        y23 = L_add(t3,t7);

        /* Pre-additions */
        t0 = L_add(re[s* 3],re[s*11]);
        t2 = L_sub(re[s* 3],re[s*11]);
        t1 = L_add(im[s* 3],im[s*11]);
        t3 = L_sub(im[s* 3],im[s*11]);
        t4 = L_add(re[s* 7],re[s*15]);
        t7 = L_sub(re[s* 7],re[s*15]);
        t5 = L_add(im[s*15],im[s* 7]);
        t6 = L_sub(im[s*15],im[s* 7]);

        /* Post-additions */
        y24 = L_add(t0,t4);
        y25 = L_add(t1,t5);
        y26 = L_sub(t2,t6);
        y27 = L_sub(t3,t7);
        y28 = L_sub(t0,t4);
        y29 = L_sub(t1,t5);
        y30 = L_add(t2,t6);
        y31 = L_add(t3,t7);
    }

    /* rotation */

    x0  = Mpy_32_xx(y22,C162);
    x1  = Mpy_32_xx(y23,C162);
    y22 = L_sub(x0,x1);
    y23 = L_add(x0,x1);

    x0  = Mpy_32_xx(y28,C162);
    x1  = Mpy_32_xx(y29,C162);
    y28 = L_sub(x0,x1);
    y29 = L_add(x0,x1);

    x0  = Mpy_32_xx(y12,C161);
    x1  = Mpy_32_xx(y13,C161);
    y12 = L_add(x0,x1);
    y13 = L_sub(x1,x0);

    x0  = Mpy_32_xx(y18,C161);
    x1  = Mpy_32_xx(y19,C161);
    y18 = L_add(x0,x1);
    y19 = L_sub(x1,x0);

    x0  = Mpy_32_xx(y10,C163);
    x1  = Mpy_32_xx(y11,C166);
    x2  = Mpy_32_xx(y10,C166);
    x3  = Mpy_32_xx(y11,C163);
    y10 = L_sub(x0,x1);
    y11 = L_add(x2,x3);

    x0  = Mpy_32_xx(y14,C165);
    x1  = Mpy_32_xx(y15,C164);
    x2  = Mpy_32_xx(y14,C164);
    x3  = Mpy_32_xx(y15,C165);
    y14 = L_sub(x0,x1);
    y15 = L_add(x2,x3);

    x0  = Mpy_32_xx(y26,C165);
    x1  = Mpy_32_xx(y27,C164);
    x2  = Mpy_32_xx(y26,C164);
    x3  = Mpy_32_xx(y27,C165);
    y26 = L_sub(x0,x1);
    y27 = L_add(x2,x3);

    x0  = Mpy_32_xx(y30,C164);
    x1  = Mpy_32_xx(y31,C165);
    x2  = Mpy_32_xx(y30,C165);
    x3  = Mpy_32_xx(y31,C164);
    y30 = L_sub(x0,x1);
    y31 = L_add(x2,x3);

    /* Pre-additions */

    t0 = L_add(y00,y16);
    t2 = L_sub(y00,y16);
    t1 = L_add(y01,y17);
    t3 = L_sub(y01,y17);
    t4 = L_add(y08,y24);
    t7 = L_sub(y08,y24);
    t5 = L_add(y25,y09);
    t6 = L_sub(y25,y09);

    /* Post-additions */

    re[s* 0] = L_add(t0,t4);
    move32();
    im[s* 0] = L_add(t1,t5);
    move32();
    re[s* 4] = L_sub(t2,t6);
    move32();
    im[s* 4] = L_sub(t3,t7);
    move32();
    re[s* 8] = L_sub(t0,t4);
    move32();
    im[s* 8] = L_sub(t1,t5);
    move32();
    re[s*12] = L_add(t2,t6);
    move32();
    im[s*12] = L_add(t3,t7);
    move32();

    /* Pre-additions */

    t0 = L_add(y02,y18);
    t2 = L_sub(y02,y18);
    t1 = L_add(y03,y19);
    t3 = L_sub(y03,y19);
    t4 = L_add(y10,y26);
    t7 = L_sub(y10,y26);
    t5 = L_add(y27,y11);
    t6 = L_sub(y27,y11);

    /* Post-additions */

    re[s* 1] = L_add(t0,t4);
    move32();
    im[s* 1] = L_add(t1,t5);
    move32();
    re[s* 5] = L_sub(t2,t6);
    move32();
    im[s* 5] = L_sub(t3,t7);
    move32();
    re[s* 9] = L_sub(t0,t4);
    move32();
    im[s* 9] = L_sub(t1,t5);
    move32();
    re[s*13] = L_add(t2,t6);
    move32();
    im[s*13] = L_add(t3,t7);
    move32();

    /* Pre-additions */

    t0 = L_add(y04,y20);
    t2 = L_sub(y04,y20);
    t1 = L_add(y05,y21);
    t3 = L_sub(y05,y21);
    t4 = L_add(y12,y28);
    t7 = L_sub(y12,y28);
    t5 = L_add(y29,y13);
    t6 = L_sub(y29,y13);

    /* Post-additions */

    re[s* 2] = L_add(t0,t4);
    move32();
    im[s* 2] = L_add(t1,t5);
    move32();
    re[s* 6] = L_sub(t2,t6);
    move32();
    im[s* 6] = L_sub(t3,t7);
    move32();
    re[s*10] = L_sub(t0,t4);
    move32();
    im[s*10] = L_sub(t1,t5);
    move32();
    re[s*14] = L_add(t2,t6);
    move32();
    im[s*14] = L_add(t3,t7);
    move32();

    /* Pre-additions */

    t0 = L_add(y06,y22);
    t2 = L_sub(y06,y22);
    t1 = L_add(y07,y23);
    t3 = L_sub(y07,y23);
    t4 = L_add(y14,y30);
    t7 = L_sub(y14,y30);
    t5 = L_add(y31,y15);
    t6 = L_sub(y31,y15);

    /* Post-additions */

    re[s* 3] = L_add(t0,t4);
    move32();
    im[s* 3] = L_add(t1,t5);
    move32();
    re[s* 7] = L_sub(t2,t6);
    move32();
    im[s* 7] = L_sub(t3,t7);
    move32();
    re[s*11] = L_sub(t0,t4);
    move32();
    im[s*11] = L_sub(t1,t5);
    move32();
    re[s*15] = L_add(t2,t6);
    move32();
    im[s*15] = L_add(t3,t7);
    move32();

}


/**
 * \brief    Function performs a complex 20-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR20 bits.
 *
 *           WOPS FLC version:                    1509 cycles
 *           WOPS with 32x16 bit multiplications:  432 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft20(Word32 *re, Word32 *im, Word16 s)
{
    Word32 r1,r2,r3,r4;
    Word32 s1,s2,s3,s4;
    Word32 x0,x1,x2,x3,x4;
    Word32 t,t0,t1,t2,t3,t4,t5,t6,t7;
    Word32 y00,y01,y02,y03,y04,y05,y06,y07,y08,y09;
    Word32 y10,y11,y12,y13,y14,y15,y16,y17,y18,y19;
    Word32 y20,y21,y22,y23,y24,y25,y26,y27,y28,y29;
    Word32 y30,y31,y32,y33,y34,y35,y36,y37,y38,y39;


    /*  */

    /* 1. FFT5 stage */

    /* real part */
    x0  = L_shr(re[s* 0],SCALEFACTOR20);
    x1  = L_shr(re[s*16],SCALEFACTOR20);
    x2  = L_shr(re[s*12],SCALEFACTOR20);
    x3  = L_shr(re[s* 8],SCALEFACTOR20);
    x4  = L_shr(re[s* 4],SCALEFACTOR20);

    r1  = L_add(x1,x4);
    r4  = L_sub(x1,x4);
    r3  = L_add(x2,x3);
    r2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y00 = L_add(x0,r1);
    r1  = L_add(y00,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s* 0],SCALEFACTOR20);
    x1  = L_shr(im[s*16],SCALEFACTOR20);
    x2  = L_shr(im[s*12],SCALEFACTOR20);
    x3  = L_shr(im[s* 8],SCALEFACTOR20);
    x4  = L_shr(im[s* 4],SCALEFACTOR20);

    s1  = L_add(x1,x4);
    s4  = L_sub(x1,x4);
    s3  = L_add(x2,x3);
    s2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y01 = L_add(x0,s1);
    s1  = L_add(y01,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y08 = L_add(r1,s2);
    y32 = L_sub(r1,s2);
    y16 = L_sub(r3,s4);
    y24 = L_add(r3,s4);

    y09 = L_sub(s1,r2);
    y33 = L_add(s1,r2);
    y17 = L_add(s3,r4);
    y25 = L_sub(s3,r4);

    /* 2. FFT5 stage */

    /* real part */
    x0  = L_shr(re[s* 5],SCALEFACTOR20);
    x1  = L_shr(re[s* 1],SCALEFACTOR20);
    x2  = L_shr(re[s*17],SCALEFACTOR20);
    x3  = L_shr(re[s*13],SCALEFACTOR20);
    x4  = L_shr(re[s* 9],SCALEFACTOR20);

    r1  = L_add(x1,x4);
    r4  = L_sub(x1,x4);
    r3  = L_add(x2,x3);
    r2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y02 = L_add(x0,r1);
    r1  = L_add(y02,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s* 5],SCALEFACTOR20);
    x1  = L_shr(im[s* 1],SCALEFACTOR20);
    x2  = L_shr(im[s*17],SCALEFACTOR20);
    x3  = L_shr(im[s*13],SCALEFACTOR20);
    x4  = L_shr(im[s* 9],SCALEFACTOR20);

    s1  = L_add(x1,x4);
    s4  = L_sub(x1,x4);
    s3  = L_add(x2,x3);
    s2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y03 = L_add(x0,s1);
    s1  = L_add(y03,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y10 = L_add(r1,s2);
    y34 = L_sub(r1,s2);
    y18 = L_sub(r3,s4);
    y26 = L_add(r3,s4);

    y11 = L_sub(s1,r2);
    y35 = L_add(s1,r2);
    y19 = L_add(s3,r4);
    y27 = L_sub(s3,r4);

    /* 3. FFT5 stage */

    /* real part */
    x0  = L_shr(re[s*10],SCALEFACTOR20);
    x1  = L_shr(re[s* 6],SCALEFACTOR20);
    x2  = L_shr(re[s* 2],SCALEFACTOR20);
    x3  = L_shr(re[s*18],SCALEFACTOR20);
    x4  = L_shr(re[s*14],SCALEFACTOR20);

    r1  = L_add(x1,x4);
    r4  = L_sub(x1,x4);
    r3  = L_add(x2,x3);
    r2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y04 = L_add(x0,r1);
    r1  = L_add(y04,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s*10],SCALEFACTOR20);
    x1  = L_shr(im[s* 6],SCALEFACTOR20);
    x2  = L_shr(im[s* 2],SCALEFACTOR20);
    x3  = L_shr(im[s*18],SCALEFACTOR20);
    x4  = L_shr(im[s*14],SCALEFACTOR20);

    s1  = L_add(x1,x4);
    s4  = L_sub(x1,x4);
    s3  = L_add(x2,x3);
    s2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y05 = L_add(x0,s1);
    s1  = L_add(y05,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y12 = L_add(r1,s2);
    y36 = L_sub(r1,s2);
    y20 = L_sub(r3,s4);
    y28 = L_add(r3,s4);

    y13 = L_sub(s1,r2);
    y37 = L_add(s1,r2);
    y21 = L_add(s3,r4);
    y29 = L_sub(s3,r4);

    /* 4. FFT5 stage */

    /* real part */
    x0  = L_shr(re[s*15],SCALEFACTOR20);
    x1  = L_shr(re[s*11],SCALEFACTOR20);
    x2  = L_shr(re[s* 7],SCALEFACTOR20);
    x3  = L_shr(re[s* 3],SCALEFACTOR20);
    x4  = L_shr(re[s*19],SCALEFACTOR20);

    r1  = L_add(x1,x4);
    r4  = L_sub(x1,x4);
    r3  = L_add(x2,x3);
    r2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y06 = L_add(x0,r1);
    r1  = L_add(y06,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4, C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    x0  = L_shr(im[s*15],SCALEFACTOR20);
    x1  = L_shr(im[s*11],SCALEFACTOR20);
    x2  = L_shr(im[s* 7],SCALEFACTOR20);
    x3  = L_shr(im[s* 3],SCALEFACTOR20);
    x4  = L_shr(im[s*19],SCALEFACTOR20);

    s1  = L_add(x1,x4);
    s4  = L_sub(x1,x4);
    s3  = L_add(x2,x3);
    s2  = L_sub(x2,x3);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y07 = L_add(x0,s1);
    s1  = L_add(y07,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y14 = L_add(r1,s2);
    y38 = L_sub(r1,s2);
    y22 = L_sub(r3,s4);
    y30 = L_add(r3,s4);

    y15 = L_sub(s1,r2);
    y39 = L_add(s1,r2);
    y23 = L_add(s3,r4);
    y31 = L_sub(s3,r4);


    /* 1. FFT4 stage */

    /* Pre-additions */
    t0 = L_add(y00,y04);
    t2 = L_sub(y00,y04);
    t1 = L_add(y01,y05);
    t3 = L_sub(y01,y05);
    t4 = L_add(y02,y06);
    t7 = L_sub(y02,y06);
    t5 = L_add(y07,y03);
    t6 = L_sub(y07,y03);

    /* Post-additions */
    re[s* 0] = L_add(t0,t4);
    move32();
    im[s* 0] = L_add(t1,t5);
    move32();
    re[s* 5] = L_sub(t2,t6);
    move32();
    im[s* 5] = L_sub(t3,t7);
    move32();
    re[s*10] = L_sub(t0,t4);
    move32();
    im[s*10] = L_sub(t1,t5);
    move32();
    re[s*15] = L_add(t2,t6);
    move32();
    im[s*15] = L_add(t3,t7);
    move32();

    /* 2. FFT4 stage */

    /* Pre-additions */
    t0 = L_add(y08,y12);
    t2 = L_sub(y08,y12);
    t1 = L_add(y09,y13);
    t3 = L_sub(y09,y13);
    t4 = L_add(y10,y14);
    t7 = L_sub(y10,y14);
    t5 = L_add(y15,y11);
    t6 = L_sub(y15,y11);

    /* Post-additions */
    re[s* 4] = L_add(t0,t4);
    move32();
    im[s* 4] = L_add(t1,t5);
    move32();
    re[s* 9] = L_sub(t2,t6);
    move32();
    im[s* 9] = L_sub(t3,t7);
    move32();
    re[s*14] = L_sub(t0,t4);
    move32();
    im[s*14] = L_sub(t1,t5);
    move32();
    re[s*19] = L_add(t2,t6);
    move32();
    im[s*19] = L_add(t3,t7);
    move32();


    /* 3. FFT4 stage */

    /* Pre-additions */
    t0 = L_add(y16,y20);
    t2 = L_sub(y16,y20);
    t1 = L_add(y17,y21);
    t3 = L_sub(y17,y21);
    t4 = L_add(y18,y22);
    t7 = L_sub(y18,y22);
    t5 = L_add(y23,y19);
    t6 = L_sub(y23,y19);

    /* Post-additions */
    re[s* 8] = L_add(t0,t4);
    move32();
    im[s* 8] = L_add(t1,t5);
    move32();
    re[s*13] = L_sub(t2,t6);
    move32();
    im[s*13] = L_sub(t3,t7);
    move32();
    re[s*18] = L_sub(t0,t4);
    move32();
    im[s*18] = L_sub(t1,t5);
    move32();
    re[s* 3] = L_add(t2,t6);
    move32();
    im[s* 3] = L_add(t3,t7);
    move32();

    /* 4. FFT4 stage */

    /* Pre-additions */
    t0 = L_add(y24,y28);
    t2 = L_sub(y24,y28);
    t1 = L_add(y25,y29);
    t3 = L_sub(y25,y29);
    t4 = L_add(y26,y30);
    t7 = L_sub(y26,y30);
    t5 = L_add(y31,y27);
    t6 = L_sub(y31,y27);

    /* Post-additions */
    re[s*12] = L_add(t0,t4);
    move32();
    im[s*12] = L_add(t1,t5);
    move32();
    re[s*17] = L_sub(t2,t6);
    move32();
    im[s*17] = L_sub(t3,t7);
    move32();
    re[s* 2] = L_sub(t0,t4);
    move32();
    im[s* 2] = L_sub(t1,t5);
    move32();
    re[s* 7] = L_add(t2,t6);
    move32();
    im[s* 7] = L_add(t3,t7);
    move32();

    /* 5. FFT4 stage */

    /* Pre-additions */
    t0 = L_add(y32,y36);
    t2 = L_sub(y32,y36);
    t1 = L_add(y33,y37);
    t3 = L_sub(y33,y37);
    t4 = L_add(y34,y38);
    t7 = L_sub(y34,y38);
    t5 = L_add(y39,y35);
    t6 = L_sub(y39,y35);

    /* Post-additions */
    re[s*16] = L_add(t0,t4);
    move32();
    im[s*16] = L_add(t1,t5);
    move32();
    re[s* 1] = L_sub(t2,t6);
    move32();
    im[s* 1] = L_sub(t3,t7);
    move32();
    re[s* 6] = L_sub(t0,t4);
    move32();
    im[s* 6] = L_sub(t1,t5);
    move32();
    re[s*11] = L_add(t2,t6);
    move32();
    im[s*11] = L_add(t3,t7);
    move32();

    /*  */
}

/**
 * \brief    Function performs a complex 30-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR30 bits.
 *
 *           WOPS FLC version:                     682 cycles
 *           WOPS with 32x16 bit multiplications:  828 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft30(Word32 *re, Word32 *im, Word16 s)
{
    Word32 t;
    Word32 r1,r2,r3,r4;
    Word32 s1,s2,s3,s4;
    Word32 x00,x01,x02,x03,x04,x05,x06,x07,x08,x09;
    Word32 x10,x11,x12,x13,x14,x15,x16,x17,x18,x19;
    Word32 x20,x21,x22,x23,x24,x25,x26,x27,x28,x29;

    Word32 y00,y01,y02,y03,y04,y05,y06,y07,y08,y09;
    Word32 y10,y11,y12,y13,y14,y15,y16,y17,y18,y19;
    Word32 y20,y21,y22,y23,y24,y25,y26,y27,y28,y29;

    Word32 z00,z01,z02,z03,z04,z05,z06,z07,z08,z09;
    Word32 z10,z11,z12,z13,z14,z15,z16,z17,z18,z19;
    Word32 z20,z21,z22,z23,z24,z25,z26,z27,z28,z29;
    Word32 z30,z31,z32,z33,z34,z35,z36,z37,z38,z39;
    Word32 z40,z41,z42,z43,z44,z45,z46,z47,z48,z49;
    Word32 z50,z51,z52,z53,z54,z55,z56,z57,z58,z59;

    Word32 *rel  = &re[s* 0];
    Word32 *reh  = &re[s*15];

    Word32 *iml  = &im[s* 0];
    Word32 *imh  = &im[s*15];



    /* 1. FFT15 stage */
    x00 = L_shr(re[s* 0],SCALEFACTOR30_1);
    x01 = L_shr(im[s* 0],SCALEFACTOR30_1);
    x02 = L_shr(re[s*18],SCALEFACTOR30_1);
    x03 = L_shr(im[s*18],SCALEFACTOR30_1);
    x04 = L_shr(re[s* 6],SCALEFACTOR30_1);
    x05 = L_shr(im[s* 6],SCALEFACTOR30_1);
    x06 = L_shr(re[s*24],SCALEFACTOR30_1);
    x07 = L_shr(im[s*24],SCALEFACTOR30_1);
    x08 = L_shr(re[s*12],SCALEFACTOR30_1);
    x09 = L_shr(im[s*12],SCALEFACTOR30_1);

    x10 = L_shr(re[s*20],SCALEFACTOR30_1);
    x11 = L_shr(im[s*20],SCALEFACTOR30_1);
    x12 = L_shr(re[s* 8],SCALEFACTOR30_1);
    x13 = L_shr(im[s* 8],SCALEFACTOR30_1);
    x14 = L_shr(re[s*26],SCALEFACTOR30_1);
    x15 = L_shr(im[s*26],SCALEFACTOR30_1);
    x16 = L_shr(re[s*14],SCALEFACTOR30_1);
    x17 = L_shr(im[s*14],SCALEFACTOR30_1);
    x18 = L_shr(re[s* 2],SCALEFACTOR30_1);
    x19 = L_shr(im[s* 2],SCALEFACTOR30_1);

    x20 = L_shr(re[s*10],SCALEFACTOR30_1);
    x21 = L_shr(im[s*10],SCALEFACTOR30_1);
    x22 = L_shr(re[s*28],SCALEFACTOR30_1);
    x23 = L_shr(im[s*28],SCALEFACTOR30_1);
    x24 = L_shr(re[s*16],SCALEFACTOR30_1);
    x25 = L_shr(im[s*16],SCALEFACTOR30_1);
    x26 = L_shr(re[s* 4],SCALEFACTOR30_1);
    x27 = L_shr(im[s* 4],SCALEFACTOR30_1);
    x28 = L_shr(re[s*22],SCALEFACTOR30_1);
    x29 = L_shr(im[s*22],SCALEFACTOR30_1);

    /* 1. FFT5 stage */

    /* real part */
    r1  = L_add(x02,x08);
    r4  = L_sub(x02,x08);
    r3  = L_add(x04,x06);
    r2  = L_sub(x04,x06);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y00 = L_add(x00,r1);
    r1  = L_add(y00,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x03,x09);
    s4  = L_sub(x03,x09);
    s3  = L_add(x05,x07);
    s2  = L_sub(x05,x07);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y01 = L_add(x01,s1);
    s1  = L_add(y01,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y02 = L_add(r1,s2);
    y08 = L_sub(r1,s2);
    y04 = L_sub(r3,s4);
    y06 = L_add(r3,s4);

    y03 = L_sub(s1,r2);
    y09 = L_add(s1,r2);
    y05 = L_add(s3,r4);
    y07 = L_sub(s3,r4);

    /* 2. FFT5 stage */

    /* real part */
    r1  = L_add(x12,x18);
    r4  = L_sub(x12,x18);
    r3  = L_add(x14,x16);
    r2  = L_sub(x14,x16);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y10 = L_add(x10,r1);
    r1  = L_add(y10,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x13,x19);
    s4  = L_sub(x13,x19);
    s3  = L_add(x15,x17);
    s2  = L_sub(x15,x17);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y11 = L_add(x11,s1);
    s1  = L_add(y11,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y12 = L_add(r1,s2);
    y18 = L_sub(r1,s2);
    y14 = L_sub(r3,s4);
    y16 = L_add(r3,s4);

    y13 = L_sub(s1,r2);
    y19 = L_add(s1,r2);
    y15 = L_add(s3,r4);
    y17 = L_sub(s3,r4);

    /* 3. FFT5 stage */

    /* real part */
    r1  = L_add(x22,x28);
    r4  = L_sub(x22,x28);
    r3  = L_add(x24,x26);
    r2  = L_sub(x24,x26);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y20 = L_add(x20,r1);
    r1  = L_add(y20,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x23,x29);
    s4  = L_sub(x23,x29);
    s3  = L_add(x25,x27);
    s2  = L_sub(x25,x27);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y21 = L_add(x21,s1);
    s1  = L_add(y21,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y22 = L_add(r1,s2);
    y28 = L_sub(r1,s2);
    y24 = L_sub(r3,s4);
    y26 = L_add(r3,s4);

    y23 = L_sub(s1,r2);
    y29 = L_add(s1,r2);
    y25 = L_add(s3,r4);
    y27 = L_sub(s3,r4);

    /* 1. FFT3 stage */

    /* real part */
    r1  = L_add(y10,y20);
    r2  = Mpy_32_xx(L_sub(y10,y20),C31);
    z00 = L_add(y00,r1);
    r1  = L_sub(y00,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y11,y21);
    s2  = Mpy_32_xx(L_sub(y11,y21),C31);
    z01 = L_add(y01,s1);
    s1  = L_sub(y01,L_shr(s1,1));

    /* combination */
    z20 = L_sub(r1,s2);
    z10 = L_add(r1,s2);
    z21 = L_add(s1,r2);
    z11 = L_sub(s1,r2);

    /* 2. FFT3 stage */

    /* real part */
    r1  = L_add(y12,y22);
    r2  = Mpy_32_xx(L_sub(y12,y22),C31);
    z12 = L_add(y02,r1);
    r1  = L_sub(y02,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y13,y23);
    s2  = Mpy_32_xx(L_sub(y13,y23),C31);
    z13 = L_add(y03,s1);
    s1  = L_sub(y03,L_shr(s1,1));

    /* combination */
    z02 = L_sub(r1,s2);
    z22 = L_add(r1,s2);
    z03 = L_add(s1,r2);
    z23 = L_sub(s1,r2);

    /* 3. FFT3 stage */

    /* real part */
    r1  = L_add(y14,y24);
    r2  = Mpy_32_xx(L_sub(y14,y24),C31);
    z24 = L_add(y04,r1);
    r1  = L_sub(y04,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y15,y25);
    s2  = Mpy_32_xx(L_sub(y15,y25),C31);
    z25 = L_add(y05,s1);
    s1  = L_sub(y05,L_shr(s1,1));

    /* combination */
    z14 = L_sub(r1,s2);
    z04 = L_add(r1,s2);
    z15 = L_add(s1,r2);
    z05 = L_sub(s1,r2);

    /* 4. FFT3 stage */

    /* real part */
    r1  = L_add(y16,y26);
    r2  = Mpy_32_xx(L_sub(y16,y26),C31);
    z06 = L_add(y06,r1);
    r1  = L_sub(y06,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y17,y27);
    s2  = Mpy_32_xx(L_sub(y17,y27),C31);
    z07 = L_add(y07,s1);
    s1  = L_sub(y07,L_shr(s1,1));

    /* combination */
    z26 = L_sub(r1,s2);
    z16 = L_add(r1,s2);
    z27 = L_add(s1,r2);
    z17 = L_sub(s1,r2);

    /* 5. FFT3 stage */

    /* real part */
    r1  = L_add(y18,y28);
    r2  = Mpy_32_xx(L_sub(y18,y28),C31);
    z18 = L_add(y08,r1);
    r1  = L_sub(y08,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y19,y29);
    s2  = Mpy_32_xx(L_sub(y19,y29),C31);
    z19 = L_add(y09,s1);
    s1  = L_sub(y09,L_shr(s1,1));

    /* combination */
    z08 = L_sub(r1,s2);
    z28 = L_add(r1,s2);
    z09 = L_add(s1,r2);
    z29 = L_sub(s1,r2);


    /* 2. FFT15 stage */
    x00 = L_shr(re[s*15],SCALEFACTOR30_1);
    x01 = L_shr(im[s*15],SCALEFACTOR30_1);
    x02 = L_shr(re[s* 3],SCALEFACTOR30_1);
    x03 = L_shr(im[s* 3],SCALEFACTOR30_1);
    x04 = L_shr(re[s*21],SCALEFACTOR30_1);
    x05 = L_shr(im[s*21],SCALEFACTOR30_1);
    x06 = L_shr(re[s* 9],SCALEFACTOR30_1);
    x07 = L_shr(im[s* 9],SCALEFACTOR30_1);
    x08 = L_shr(re[s*27],SCALEFACTOR30_1);
    x09 = L_shr(im[s*27],SCALEFACTOR30_1);

    x10 = L_shr(re[s* 5],SCALEFACTOR30_1);
    x11 = L_shr(im[s* 5],SCALEFACTOR30_1);
    x12 = L_shr(re[s*23],SCALEFACTOR30_1);
    x13 = L_shr(im[s*23],SCALEFACTOR30_1);
    x14 = L_shr(re[s*11],SCALEFACTOR30_1);
    x15 = L_shr(im[s*11],SCALEFACTOR30_1);
    x16 = L_shr(re[s*29],SCALEFACTOR30_1);
    x17 = L_shr(im[s*29],SCALEFACTOR30_1);
    x18 = L_shr(re[s*17],SCALEFACTOR30_1);
    x19 = L_shr(im[s*17],SCALEFACTOR30_1);

    x20 = L_shr(re[s*25],SCALEFACTOR30_1);
    x21 = L_shr(im[s*25],SCALEFACTOR30_1);
    x22 = L_shr(re[s*13],SCALEFACTOR30_1);
    x23 = L_shr(im[s*13],SCALEFACTOR30_1);
    x24 = L_shr(re[s* 1],SCALEFACTOR30_1);
    x25 = L_shr(im[s* 1],SCALEFACTOR30_1);
    x26 = L_shr(re[s*19],SCALEFACTOR30_1);
    x27 = L_shr(im[s*19],SCALEFACTOR30_1);
    x28 = L_shr(re[s* 7],SCALEFACTOR30_1);
    x29 = L_shr(im[s* 7],SCALEFACTOR30_1);

    /* 1. FFT5 stage */

    /* real part */
    r1  = L_add(x02,x08);
    r4  = L_sub(x02,x08);
    r3  = L_add(x04,x06);
    r2  = L_sub(x04,x06);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y00 = L_add(x00,r1);
    r1  = L_add(y00,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x03,x09);
    s4  = L_sub(x03,x09);
    s3  = L_add(x05,x07);
    s2  = L_sub(x05,x07);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y01 = L_add(x01,s1);
    s1  = L_add(y01,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y02 = L_add(r1,s2);
    y08 = L_sub(r1,s2);
    y04 = L_sub(r3,s4);
    y06 = L_add(r3,s4);

    y03 = L_sub(s1,r2);
    y09 = L_add(s1,r2);
    y05 = L_add(s3,r4);
    y07 = L_sub(s3,r4);

    /* 2. FFT5 stage */

    /* real part */
    r1  = L_add(x12,x18);
    r4  = L_sub(x12,x18);
    r3  = L_add(x14,x16);
    r2  = L_sub(x14,x16);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y10 = L_add(x10,r1);
    r1  = L_add(y10,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x13,x19);
    s4  = L_sub(x13,x19);
    s3  = L_add(x15,x17);
    s2  = L_sub(x15,x17);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y11 = L_add(x11,s1);
    s1  = L_add(y11,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y12 = L_add(r1,s2);
    y18 = L_sub(r1,s2);
    y14 = L_sub(r3,s4);
    y16 = L_add(r3,s4);

    y13 = L_sub(s1,r2);
    y19 = L_add(s1,r2);
    y15 = L_add(s3,r4);
    y17 = L_sub(s3,r4);

    /* 3. FFT5 stage */

    /* real part */
    r1  = L_add(x22,x28);
    r4  = L_sub(x22,x28);
    r3  = L_add(x24,x26);
    r2  = L_sub(x24,x26);
    t   = Mpy_32_xx(L_sub(r1,r3),C54);
    r1  = L_add(r1,r3);
    y20 = L_add(x20,r1);
    r1  = L_add(y20,(L_shl(Mpy_32_xx(r1,C55),1)));
    r3  = L_sub(r1,t);
    r1  = L_add(r1,t);
    t   = Mpy_32_xx((L_add(r4,r2)),C51);
    r4  = L_add(t,L_shl(Mpy_32_xx(r4,C52),1));
    r2  = L_add(t,Mpy_32_xx(r2,C53));

    /* imaginary part */
    s1  = L_add(x23,x29);
    s4  = L_sub(x23,x29);
    s3  = L_add(x25,x27);
    s2  = L_sub(x25,x27);
    t   = Mpy_32_xx(L_sub(s1,s3),C54);
    s1  = L_add(s1,s3);
    y21 = L_add(x21,s1);
    s1  = L_add(y21,L_shl(Mpy_32_xx(s1,C55),1));
    s3  = L_sub(s1,t);
    s1  = L_add(s1,t);
    t   = Mpy_32_xx(L_add(s4,s2),C51);
    s4  = L_add(t,L_shl(Mpy_32_xx(s4,C52),1));
    s2  = L_add(t,Mpy_32_xx(s2,C53));

    /* combination */
    y22 = L_add(r1,s2);
    y28 = L_sub(r1,s2);
    y24 = L_sub(r3,s4);
    y26 = L_add(r3,s4);

    y23 = L_sub(s1,r2);
    y29 = L_add(s1,r2);
    y25 = L_add(s3,r4);
    y27 = L_sub(s3,r4);

    /* 1. FFT3 stage */

    /* real part */
    r1  = L_add(y10,y20);
    r2  = Mpy_32_xx(L_sub(y10,y20),C31);
    z30 = L_add(y00,r1);
    r1  = L_sub(y00,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y11,y21);
    s2  = Mpy_32_xx(L_sub(y11,y21),C31);
    z31 = L_add(y01,s1);
    s1  = L_sub(y01,L_shr(s1,1));

    /* combination */
    z50 = L_sub(r1,s2);
    z40 = L_add(r1,s2);
    z51 = L_add(s1,r2);
    z41 = L_sub(s1,r2);

    /* 2. FFT3 stage */

    /* real part */
    r1  = L_add(y12,y22);
    r2  = Mpy_32_xx(L_sub(y12,y22),C31);
    z42 = L_add(y02,r1);
    r1  = L_sub(y02,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y13,y23);
    s2  = Mpy_32_xx(L_sub(y13,y23),C31);
    z43 = L_add(y03,s1);
    s1  = L_sub(y03,L_shr(s1,1));

    /* combination */
    z32 = L_sub(r1,s2);
    z52 = L_add(r1,s2);
    z33 = L_add(s1,r2);
    z53 = L_sub(s1,r2);

    /* 3. FFT3 stage */

    /* real part */
    r1  = L_add(y14,y24);
    r2  = Mpy_32_xx(L_sub(y14,y24),C31);
    z54 = L_add(y04,r1);
    r1  = L_sub(y04,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y15,y25);
    s2  = Mpy_32_xx(L_sub(y15,y25),C31);
    z55 = L_add(y05,s1);
    s1  = L_sub(y05,L_shr(s1,1));

    /* combination */
    z44 = L_sub(r1,s2);
    z34 = L_add(r1,s2);
    z45 = L_add(s1,r2);
    z35 = L_sub(s1,r2);

    /* 4. FFT3 stage */

    /* real part */
    r1  = L_add(y16,y26);
    r2  = Mpy_32_xx(L_sub(y16,y26),C31);
    z36 = L_add(y06,r1);
    r1  = L_sub(y06,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y17,y27);
    s2  = Mpy_32_xx(L_sub(y17,y27),C31);
    z37 = L_add(y07,s1);
    s1  = L_sub(y07,L_shr(s1,1));

    /* combination */
    z56 = L_sub(r1,s2);
    z46 = L_add(r1,s2);
    z57 = L_add(s1,r2);
    z47 = L_sub(s1,r2);

    /* 5. FFT3 stage */

    /* real part */
    r1  = L_add(y18,y28);
    r2  = Mpy_32_xx(L_sub(y18,y28),C31);
    z48 = L_add(y08,r1);
    r1  = L_sub(y08,L_shr(r1,1));

    /* imaginary part */
    s1  = L_add(y19,y29);
    s2  = Mpy_32_xx(L_sub(y19,y29),C31);
    z49 = L_add(y09,s1);
    s1  = L_sub(y09,L_shr(s1,1));

    /* combination */
    z38 = L_sub(r1,s2);
    z58 = L_add(r1,s2);
    z39 = L_add(s1,r2);
    z59 = L_sub(s1,r2);


    /* 1. FFT2 stage */
    r1 = L_shr(z00,SCALEFACTOR30_2);
    r2 = L_shr(z30,SCALEFACTOR30_2);
    r3 = L_shr(z01,SCALEFACTOR30_2);
    r4 = L_shr(z31,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 2. FFT2 stage */
    r1 = L_shr(z16,SCALEFACTOR30_2);
    r2 = L_shr(z46,SCALEFACTOR30_2);
    r3 = L_shr(z17,SCALEFACTOR30_2);
    r4 = L_shr(z47,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 3. FFT2 stage */
    r1 = L_shr(z02,SCALEFACTOR30_2);
    r2 = L_shr(z32,SCALEFACTOR30_2);
    r3 = L_shr(z03,SCALEFACTOR30_2);
    r4 = L_shr(z33,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 4. FFT2 stage */
    r1 = L_shr(z18,SCALEFACTOR30_2);
    r2 = L_shr(z48,SCALEFACTOR30_2);
    r3 = L_shr(z19,SCALEFACTOR30_2);
    r4 = L_shr(z49,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 5. FFT2 stage */
    r1 = L_shr(z04,SCALEFACTOR30_2);
    r2 = L_shr(z34,SCALEFACTOR30_2);
    r3 = L_shr(z05,SCALEFACTOR30_2);
    r4 = L_shr(z35,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 6. FFT2 stage */
    r1 = L_shr(z20,SCALEFACTOR30_2);
    r2 = L_shr(z50,SCALEFACTOR30_2);
    r3 = L_shr(z21,SCALEFACTOR30_2);
    r4 = L_shr(z51,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 7. FFT2 stage */
    r1 = L_shr(z06,SCALEFACTOR30_2);
    r2 = L_shr(z36,SCALEFACTOR30_2);
    r3 = L_shr(z07,SCALEFACTOR30_2);
    r4 = L_shr(z37,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 8. FFT2 stage */
    r1 = L_shr(z22,SCALEFACTOR30_2);
    r2 = L_shr(z52,SCALEFACTOR30_2);
    r3 = L_shr(z23,SCALEFACTOR30_2);
    r4 = L_shr(z53,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 9. FFT2 stage */
    r1 = L_shr(z08,SCALEFACTOR30_2);
    r2 = L_shr(z38,SCALEFACTOR30_2);
    r3 = L_shr(z09,SCALEFACTOR30_2);
    r4 = L_shr(z39,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 10. FFT2 stage */
    r1 = L_shr(z24,SCALEFACTOR30_2);
    r2 = L_shr(z54,SCALEFACTOR30_2);
    r3 = L_shr(z25,SCALEFACTOR30_2);
    r4 = L_shr(z55,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 11. FFT2 stage */
    r1 = L_shr(z10,SCALEFACTOR30_2);
    r2 = L_shr(z40,SCALEFACTOR30_2);
    r3 = L_shr(z11,SCALEFACTOR30_2);
    r4 = L_shr(z41,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 12. FFT2 stage */
    r1 = L_shr(z26,SCALEFACTOR30_2);
    r2 = L_shr(z56,SCALEFACTOR30_2);
    r3 = L_shr(z27,SCALEFACTOR30_2);
    r4 = L_shr(z57,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 13. FFT2 stage */
    r1 = L_shr(z12,SCALEFACTOR30_2);
    r2 = L_shr(z42,SCALEFACTOR30_2);
    r3 = L_shr(z13,SCALEFACTOR30_2);
    r4 = L_shr(z43,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 14. FFT2 stage */
    r1 = L_shr(z28,SCALEFACTOR30_2);
    r2 = L_shr(z58,SCALEFACTOR30_2);
    r3 = L_shr(z29,SCALEFACTOR30_2);
    r4 = L_shr(z59,SCALEFACTOR30_2);
    *reh = L_add(r1,r2);
    move32();
    *rel = L_sub(r1,r2);
    move32();
    *imh = L_add(r3,r4);
    move32();
    *iml = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

    /* 15. FFT2 stage */
    r1 = L_shr(z14,SCALEFACTOR30_2);
    r2 = L_shr(z44,SCALEFACTOR30_2);
    r3 = L_shr(z15,SCALEFACTOR30_2);
    r4 = L_shr(z45,SCALEFACTOR30_2);
    *rel = L_add(r1,r2);
    move32();
    *reh = L_sub(r1,r2);
    move32();
    *iml = L_add(r3,r4);
    move32();
    *imh = L_sub(r3,r4);
    move32();
    rel+=s, reh+=s, iml+=s;
    imh+=s;

}

/**
 * \brief    Function performs a complex 32-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR32 bits.
 *
 *           WOPS FLC version:                     504 cycles
 *           WOPS with 32x16 bit multiplications:  752 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft32(Word32 * re, Word32 * im, Word16 s)
{
    Word32 as,bs;
    Word32 x00,x01,x02,x03,x04,x05,x06,x07;
    Word32 x08,x09,x10,x11,x12,x13,x14,x15;
    Word32 t00,t01,t02,t03,t04,t05,t06,t07;
    Word32 t08,t09,t10,t11,t12,t13,t14,t15;
    Word32 s00,s01,s02,s03,s04,s05,s06,s07;
    Word32 s08,s09,s10,s11,s12,s13,s14,s15;

    Word32 y00,y01,y02,y03,y04,y05,y06,y07;
    Word32 y08,y09,y10,y11,y12,y13,y14,y15;
    Word32 y16,y17,y18,y19,y20,y21,y22,y23;
    Word32 y24,y25,y26,y27,y28,y29,y30,y31;
    Word32 y32,y33,y34,y35,y36,y37,y38,y39;
    Word32 y40,y41,y42,y43,y44,y45,y46,y47;
    Word32 y48,y49,y50,y51,y52,y53,y54,y55;
    Word32 y56,y57,y58,y59,y60,y61,y62,y63;



    /* 1. FFT8 stage */
    x00 = L_shr(re[s* 0],SCALEFACTOR32_1);
    x01 = L_shr(im[s* 0],SCALEFACTOR32_1);
    x02 = L_shr(re[s* 4],SCALEFACTOR32_1);
    x03 = L_shr(im[s* 4],SCALEFACTOR32_1);
    x04 = L_shr(re[s* 8],SCALEFACTOR32_1);
    x05 = L_shr(im[s* 8],SCALEFACTOR32_1);
    x06 = L_shr(re[s*12],SCALEFACTOR32_1);
    x07 = L_shr(im[s*12],SCALEFACTOR32_1);
    x08 = L_shr(re[s*16],SCALEFACTOR32_1);
    x09 = L_shr(im[s*16],SCALEFACTOR32_1);
    x10 = L_shr(re[s*20],SCALEFACTOR32_1);
    x11 = L_shr(im[s*20],SCALEFACTOR32_1);
    x12 = L_shr(re[s*24],SCALEFACTOR32_1);
    x13 = L_shr(im[s*24],SCALEFACTOR32_1);
    x14 = L_shr(re[s*28],SCALEFACTOR32_1);
    x15 = L_shr(im[s*28],SCALEFACTOR32_1);

    t00 = L_add(x00,x08);
    t02 = L_sub(x00,x08);
    t01 = L_add(x01,x09);
    t03 = L_sub(x01,x09);
    t04 = L_add(x02,x10);
    t06 = L_sub(x02,x10);
    t05 = L_add(x03,x11);
    t07 = L_sub(x03,x11);
    t08 = L_add(x04,x12);
    t10 = L_sub(x04,x12);
    t09 = L_add(x05,x13);
    t11 = L_sub(x05,x13);
    t12 = L_add(x06,x14);
    t14 = L_sub(x06,x14);
    t13 = L_add(x07,x15);
    t15 = L_sub(x07,x15);

    /* Pre-additions and core multiplications */
    s00 = L_add(t00,t08);
    s04 = L_sub(t00,t08);
    s01 = L_add(t01,t09);
    s05 = L_sub(t01,t09);
    s08 = L_sub(t02,t11);
    s10 = L_add(t02,t11);
    s09 = L_add(t03,t10);
    s11 = L_sub(t03,t10);
    s02 = L_add(t04,t12);
    s07 = L_sub(t04,t12);
    s03 = L_add(t05,t13);
    s06 = L_sub(t13,t05);
    t01 = L_add(t06,t14);
    t02 = L_sub(t06,t14);
    t00 = L_add(t07,t15);
    t03 = L_sub(t07,t15);

    s12 = Mpy_32_xx(L_add(t00,t02),C81);
    s14 = Mpy_32_xx(L_sub(t00,t02),C81);
    s13 = Mpy_32_xx(L_sub(t03,t01),C81);
    s15 = Mpy_32_xx(L_add(t01,t03),C82);

    /* Post-additions */
    y00 = L_add(s00,s02);
    y08 = L_sub(s00,s02);
    y01 = L_add(s01,s03);
    y09 = L_sub(s01,s03);
    y04 = L_sub(s04,s06);
    y12 = L_add(s04,s06);
    y05 = L_sub(s05,s07);
    y13 = L_add(s05,s07);
    y06 = L_add(s08,s14);
    y14 = L_sub(s08,s14);
    y07 = L_add(s09,s15);
    y15 = L_sub(s09,s15);
    y02 = L_add(s10,s12);
    y10 = L_sub(s10,s12);
    y03 = L_add(s11,s13);
    y11 = L_sub(s11,s13);

    /* 2. FFT8 stage */
    x00 = L_shr(re[s* 1],SCALEFACTOR32_1);
    x01 = L_shr(im[s* 1],SCALEFACTOR32_1);
    x02 = L_shr(re[s* 5],SCALEFACTOR32_1);
    x03 = L_shr(im[s* 5],SCALEFACTOR32_1);
    x04 = L_shr(re[s* 9],SCALEFACTOR32_1);
    x05 = L_shr(im[s* 9],SCALEFACTOR32_1);
    x06 = L_shr(re[s*13],SCALEFACTOR32_1);
    x07 = L_shr(im[s*13],SCALEFACTOR32_1);
    x08 = L_shr(re[s*17],SCALEFACTOR32_1);
    x09 = L_shr(im[s*17],SCALEFACTOR32_1);
    x10 = L_shr(re[s*21],SCALEFACTOR32_1);
    x11 = L_shr(im[s*21],SCALEFACTOR32_1);
    x12 = L_shr(re[s*25],SCALEFACTOR32_1);
    x13 = L_shr(im[s*25],SCALEFACTOR32_1);
    x14 = L_shr(re[s*29],SCALEFACTOR32_1);
    x15 = L_shr(im[s*29],SCALEFACTOR32_1);

    t00 = L_add(x00,x08);
    t02 = L_sub(x00,x08);
    t01 = L_add(x01,x09);
    t03 = L_sub(x01,x09);
    t04 = L_add(x02,x10);
    t06 = L_sub(x02,x10);
    t05 = L_add(x03,x11);
    t07 = L_sub(x03,x11);
    t08 = L_add(x04,x12);
    t10 = L_sub(x04,x12);
    t09 = L_add(x05,x13);
    t11 = L_sub(x05,x13);
    t12 = L_add(x06,x14);
    t14 = L_sub(x06,x14);
    t13 = L_add(x07,x15);
    t15 = L_sub(x07,x15);

    /* Pre-additions and core multiplications */
    s00 = L_add(t00,t08);
    s04 = L_sub(t00,t08);
    s01 = L_add(t01,t09);
    s05 = L_sub(t01,t09);
    s08 = L_sub(t02,t11);
    s10 = L_add(t02,t11);
    s09 = L_add(t03,t10);
    s11 = L_sub(t03,t10);
    s02 = L_add(t04,t12);
    s07 = L_sub(t04,t12);
    s03 = L_add(t05,t13);
    s06 = L_sub(t13,t05);
    t01 = L_add(t06,t14);
    t02 = L_sub(t06,t14);
    t00 = L_add(t07,t15);
    t03 = L_sub(t07,t15);

    s12 = Mpy_32_xx(L_add(t00,t02),C81);
    s14 = Mpy_32_xx(L_sub(t00,t02),C81);
    s13 = Mpy_32_xx(L_sub(t03,t01),C81);
    s15 = Mpy_32_xx(L_add(t01,t03),C82);

    /* Post-additions */
    y16 = L_add(s00,s02);
    y24 = L_sub(s00,s02);
    y17 = L_add(s01,s03);
    y25 = L_sub(s01,s03);
    y20 = L_sub(s04,s06);
    y28 = L_add(s04,s06);
    y21 = L_sub(s05,s07);
    y29 = L_add(s05,s07);
    y22 = L_add(s08,s14);
    y30 = L_sub(s08,s14);
    y23 = L_add(s09,s15);
    y31 = L_sub(s09,s15);
    y18 = L_add(s10,s12);
    y26 = L_sub(s10,s12);
    y19 = L_add(s11,s13);
    y27 = L_sub(s11,s13);

    /* 3. FFT8 stage */
    x00 = L_shr(re[s* 2],SCALEFACTOR32_1);
    x01 = L_shr(im[s* 2],SCALEFACTOR32_1);
    x02 = L_shr(re[s* 6],SCALEFACTOR32_1);
    x03 = L_shr(im[s* 6],SCALEFACTOR32_1);
    x04 = L_shr(re[s*10],SCALEFACTOR32_1);
    x05 = L_shr(im[s*10],SCALEFACTOR32_1);
    x06 = L_shr(re[s*14],SCALEFACTOR32_1);
    x07 = L_shr(im[s*14],SCALEFACTOR32_1);
    x08 = L_shr(re[s*18],SCALEFACTOR32_1);
    x09 = L_shr(im[s*18],SCALEFACTOR32_1);
    x10 = L_shr(re[s*22],SCALEFACTOR32_1);
    x11 = L_shr(im[s*22],SCALEFACTOR32_1);
    x12 = L_shr(re[s*26],SCALEFACTOR32_1);
    x13 = L_shr(im[s*26],SCALEFACTOR32_1);
    x14 = L_shr(re[s*30],SCALEFACTOR32_1);
    x15 = L_shr(im[s*30],SCALEFACTOR32_1);

    t00 = L_add(x00,x08);
    t02 = L_sub(x00,x08);
    t01 = L_add(x01,x09);
    t03 = L_sub(x01,x09);
    t04 = L_add(x02,x10);
    t06 = L_sub(x02,x10);
    t05 = L_add(x03,x11);
    t07 = L_sub(x03,x11);
    t08 = L_add(x04,x12);
    t10 = L_sub(x04,x12);
    t09 = L_add(x05,x13);
    t11 = L_sub(x05,x13);
    t12 = L_add(x06,x14);
    t14 = L_sub(x06,x14);
    t13 = L_add(x07,x15);
    t15 = L_sub(x07,x15);

    /* Pre-additions and core multiplications */
    s00 = L_add(t00,t08);
    s04 = L_sub(t00,t08);
    s01 = L_add(t01,t09);
    s05 = L_sub(t01,t09);
    s08 = L_sub(t02,t11);
    s10 = L_add(t02,t11);
    s09 = L_add(t03,t10);
    s11 = L_sub(t03,t10);
    s02 = L_add(t04,t12);
    s07 = L_sub(t04,t12);
    s03 = L_add(t05,t13);
    s06 = L_sub(t13,t05);
    t01 = L_add(t06,t14);
    t02 = L_sub(t06,t14);
    t00 = L_add(t07,t15);
    t03 = L_sub(t07,t15);

    s12 = Mpy_32_xx(L_add(t00,t02),C81);
    s14 = Mpy_32_xx(L_sub(t00,t02),C81);
    s13 = Mpy_32_xx(L_sub(t03,t01),C81);
    s15 = Mpy_32_xx(L_add(t01,t03),C82);

    /* Post-additions */
    y32 = L_add(s00,s02);
    y40 = L_sub(s00,s02);
    y33 = L_add(s01,s03);
    y41 = L_sub(s01,s03);
    y36 = L_sub(s04,s06);
    y44 = L_add(s04,s06);
    y37 = L_sub(s05,s07);
    y45 = L_add(s05,s07);
    y38 = L_add(s08,s14);
    y46 = L_sub(s08,s14);
    y39 = L_add(s09,s15);
    y47 = L_sub(s09,s15);
    y34 = L_add(s10,s12);
    y42 = L_sub(s10,s12);
    y35 = L_add(s11,s13);
    y43 = L_sub(s11,s13);

    /* 4. FFT8 stage */
    x00 = L_shr(re[s* 3],SCALEFACTOR32_1);
    x01 = L_shr(im[s* 3],SCALEFACTOR32_1);
    x02 = L_shr(re[s* 7],SCALEFACTOR32_1);
    x03 = L_shr(im[s* 7],SCALEFACTOR32_1);
    x04 = L_shr(re[s*11],SCALEFACTOR32_1);
    x05 = L_shr(im[s*11],SCALEFACTOR32_1);
    x06 = L_shr(re[s*15],SCALEFACTOR32_1);
    x07 = L_shr(im[s*15],SCALEFACTOR32_1);
    x08 = L_shr(re[s*19],SCALEFACTOR32_1);
    x09 = L_shr(im[s*19],SCALEFACTOR32_1);
    x10 = L_shr(re[s*23],SCALEFACTOR32_1);
    x11 = L_shr(im[s*23],SCALEFACTOR32_1);
    x12 = L_shr(re[s*27],SCALEFACTOR32_1);
    x13 = L_shr(im[s*27],SCALEFACTOR32_1);
    x14 = L_shr(re[s*31],SCALEFACTOR32_1);
    x15 = L_shr(im[s*31],SCALEFACTOR32_1);

    t00 = L_add(x00,x08);
    t02 = L_sub(x00,x08);
    t01 = L_add(x01,x09);
    t03 = L_sub(x01,x09);
    t04 = L_add(x02,x10);
    t06 = L_sub(x02,x10);
    t05 = L_add(x03,x11);
    t07 = L_sub(x03,x11);
    t08 = L_add(x04,x12);
    t10 = L_sub(x04,x12);
    t09 = L_add(x05,x13);
    t11 = L_sub(x05,x13);
    t12 = L_add(x06,x14);
    t14 = L_sub(x06,x14);
    t13 = L_add(x07,x15);
    t15 = L_sub(x07,x15);

    /* Pre-additions and core multiplications */
    s00 = L_add(t00,t08);
    s04 = L_sub(t00,t08);
    s01 = L_add(t01,t09);
    s05 = L_sub(t01,t09);
    s08 = L_sub(t02,t11);
    s10 = L_add(t02,t11);
    s09 = L_add(t03,t10);
    s11 = L_sub(t03,t10);
    s02 = L_add(t04,t12);
    s07 = L_sub(t04,t12);
    s03 = L_add(t05,t13);
    s06 = L_sub(t13,t05);
    t01 = L_add(t06,t14);
    t02 = L_sub(t06,t14);
    t00 = L_add(t07,t15);
    t03 = L_sub(t07,t15);

    s12 = Mpy_32_xx(L_add(t00,t02),C81);
    s14 = Mpy_32_xx(L_sub(t00,t02),C81);
    s13 = Mpy_32_xx(L_sub(t03,t01),C81);
    s15 = Mpy_32_xx(L_add(t01,t03),C82);

    /* Post-additions */
    y48 = L_add(s00,s02);
    y56 = L_sub(s00,s02);
    y49 = L_add(s01,s03);
    y57 = L_sub(s01,s03);
    y52 = L_sub(s04,s06);
    y60 = L_add(s04,s06);
    y53 = L_sub(s05,s07);
    y61 = L_add(s05,s07);
    y54 = L_add(s08,s14);
    y62 = L_sub(s08,s14);
    y55 = L_add(s09,s15);
    y63 = L_sub(s09,s15);
    y50 = L_add(s10,s12);
    y58 = L_sub(s10,s12);
    y51 = L_add(s11,s13);
    y59 = L_sub(s11,s13);

    /* apply twiddle factors */
    y00 = L_shr(y00,SCALEFACTOR32_2);
    y01 = L_shr(y01,SCALEFACTOR32_2);
    y02 = L_shr(y02,SCALEFACTOR32_2);
    y03 = L_shr(y03,SCALEFACTOR32_2);
    y04 = L_shr(y04,SCALEFACTOR32_2);
    y05 = L_shr(y05,SCALEFACTOR32_2);
    y06 = L_shr(y06,SCALEFACTOR32_2);
    y07 = L_shr(y07,SCALEFACTOR32_2);
    y08 = L_shr(y08,SCALEFACTOR32_2);
    y09 = L_shr(y09,SCALEFACTOR32_2);
    y10 = L_shr(y10,SCALEFACTOR32_2);
    y11 = L_shr(y11,SCALEFACTOR32_2);
    y12 = L_shr(y12,SCALEFACTOR32_2);
    y13 = L_shr(y13,SCALEFACTOR32_2);
    y14 = L_shr(y14,SCALEFACTOR32_2);
    y15 = L_shr(y15,SCALEFACTOR32_2);
    y16 = L_shr(y16,SCALEFACTOR32_2);
    y17 = L_shr(y17,SCALEFACTOR32_2);
    y32 = L_shr(y32,SCALEFACTOR32_2);
    y33 = L_shr(y33,SCALEFACTOR32_2);
    y48 = L_shr(y48,SCALEFACTOR32_2);
    y49 = L_shr(y49,SCALEFACTOR32_2);
    y40 = L_shr(y40,SCALEFACTOR32_2);
    y41 = L_shr(y41,SCALEFACTOR32_2);

    cplxMpy3_0(y18, y19, RotVector_32[2* 0+0], RotVector_32[2* 0+1]);
    cplxMpy3_0(y20, y21, RotVector_32[2* 1+0], RotVector_32[2* 1+1]);
    cplxMpy3_0(y22, y23, RotVector_32[2* 2+0], RotVector_32[2* 2+1]);
    cplxMpy3_0(y24, y25, RotVector_32[2* 3+0], RotVector_32[2* 3+1]);
    cplxMpy3_0(y26, y27, RotVector_32[2* 4+0], RotVector_32[2* 4+1]);
    cplxMpy3_0(y28, y29, RotVector_32[2* 5+0], RotVector_32[2* 5+1]);
    cplxMpy3_0(y30, y31, RotVector_32[2* 6+0], RotVector_32[2* 6+1]);
    cplxMpy3_0(y34, y35, RotVector_32[2* 7+0], RotVector_32[2* 7+1]);
    cplxMpy3_0(y36, y37, RotVector_32[2* 8+0], RotVector_32[2* 8+1]);
    cplxMpy3_0(y38, y39, RotVector_32[2* 9+0], RotVector_32[2* 9+1]);
    cplxMpy3_0(y42, y43, RotVector_32[2*10+0], RotVector_32[2*10+1]);
    cplxMpy3_0(y44, y45, RotVector_32[2*11+0], RotVector_32[2*11+1]);
    cplxMpy3_0(y46, y47, RotVector_32[2*12+0], RotVector_32[2*12+1]);
    cplxMpy3_0(y50, y51, RotVector_32[2*13+0], RotVector_32[2*13+1]);
    cplxMpy3_0(y52, y53, RotVector_32[2*14+0], RotVector_32[2*14+1]);
    cplxMpy3_0(y54, y55, RotVector_32[2*15+0], RotVector_32[2*15+1]);
    cplxMpy3_0(y56, y57, RotVector_32[2*16+0], RotVector_32[2*16+1]);
    cplxMpy3_0(y58, y59, RotVector_32[2*17+0], RotVector_32[2*17+1]);
    cplxMpy3_0(y60, y61, RotVector_32[2*18+0], RotVector_32[2*18+1]);
    cplxMpy3_0(y62, y63, RotVector_32[2*19+0], RotVector_32[2*19+1]);

    /* 1. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y00,y32);
    t02 = L_sub(y00,y32);
    t01 = L_add(y01,y33);
    t03 = L_sub(y01,y33);
    t04 = L_add(y16,y48);
    t07 = L_sub(y16,y48);
    t05 = L_add(y49,y17);
    t06 = L_sub(y49,y17);

    /* Post-additions */
    re[s* 0] = L_add(t00,t04);
    move32();
    im[s* 0] = L_add(t01,t05);
    move32();
    re[s* 8] = L_sub(t02,t06);
    move32();
    im[s* 8] = L_sub(t03,t07);
    move32();
    re[s*16] = L_sub(t00,t04);
    move32();
    im[s*16] = L_sub(t01,t05);
    move32();
    re[s*24] = L_add(t02,t06);
    move32();
    im[s*24] = L_add(t03,t07);
    move32();

    /* 2. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y02,y34);
    t02 = L_sub(y02,y34);
    t01 = L_add(y03,y35);
    t03 = L_sub(y03,y35);
    t04 = L_add(y18,y50);
    t07 = L_sub(y18,y50);
    t05 = L_add(y51,y19);
    t06 = L_sub(y51,y19);

    /* Post-additions */
    re[s* 1] = L_add(t00,t04);
    move32();
    im[s* 1] = L_add(t01,t05);
    move32();
    re[s* 9] = L_sub(t02,t06);
    move32();
    im[s* 9] = L_sub(t03,t07);
    move32();
    re[s*17] = L_sub(t00,t04);
    move32();
    im[s*17] = L_sub(t01,t05);
    move32();
    re[s*25] = L_add(t02,t06);
    move32();
    im[s*25] = L_add(t03,t07);
    move32();

    /* 3. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y04,y36);
    t02 = L_sub(y04,y36);
    t01 = L_add(y05,y37);
    t03 = L_sub(y05,y37);
    t04 = L_add(y20,y52);
    t07 = L_sub(y20,y52);
    t05 = L_add(y53,y21);
    t06 = L_sub(y53,y21);

    /* Post-additions */
    re[s* 2] = L_add(t00,t04);
    move32();
    im[s* 2] = L_add(t01,t05);
    move32();
    re[s*10] = L_sub(t02,t06);
    move32();
    im[s*10] = L_sub(t03,t07);
    move32();
    re[s*18] = L_sub(t00,t04);
    move32();
    im[s*18] = L_sub(t01,t05);
    move32();
    re[s*26] = L_add(t02,t06);
    move32();
    im[s*26] = L_add(t03,t07);
    move32();

    /* 4. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y06,y38);
    t02 = L_sub(y06,y38);
    t01 = L_add(y07,y39);
    t03 = L_sub(y07,y39);
    t04 = L_add(y22,y54);
    t07 = L_sub(y22,y54);
    t05 = L_add(y55,y23);
    t06 = L_sub(y55,y23);

    /* Post-additions */
    re[s* 3] = L_add(t00,t04);
    move32();
    im[s* 3] = L_add(t01,t05);
    move32();
    re[s*11] = L_sub(t02,t06);
    move32();
    im[s*11] = L_sub(t03,t07);
    move32();
    re[s*19] = L_sub(t00,t04);
    move32();
    im[s*19] = L_sub(t01,t05);
    move32();
    re[s*27] = L_add(t02,t06);
    move32();
    im[s*27] = L_add(t03,t07);
    move32();

    /* 5. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y08,y41);
    t02 = L_sub(y08,y41);
    t01 = L_sub(y09,y40);
    t03 = L_add(y09,y40);
    t04 = L_add(y24,y56);
    t07 = L_sub(y24,y56);
    t05 = L_add(y57,y25);
    t06 = L_sub(y57,y25);

    /* Post-additions */
    re[s* 4] = L_add(t00,t04);
    move32();
    im[s* 4] = L_add(t01,t05);
    move32();
    re[s*12] = L_sub(t02,t06);
    move32();
    im[s*12] = L_sub(t03,t07);
    move32();
    re[s*20] = L_sub(t00,t04);
    move32();
    im[s*20] = L_sub(t01,t05);
    move32();
    re[s*28] = L_add(t02,t06);
    move32();
    im[s*28] = L_add(t03,t07);
    move32();

    /* 6. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y10,y42);
    t02 = L_sub(y10,y42);
    t01 = L_add(y11,y43);
    t03 = L_sub(y11,y43);
    t04 = L_add(y26,y58);
    t07 = L_sub(y26,y58);
    t05 = L_add(y59,y27);
    t06 = L_sub(y59,y27);

    /* Post-additions */
    re[s* 5] = L_add(t00,t04);
    move32();
    im[s* 5] = L_add(t01,t05);
    move32();
    re[s*13] = L_sub(t02,t06);
    move32();
    im[s*13] = L_sub(t03,t07);
    move32();
    re[s*21] = L_sub(t00,t04);
    move32();
    im[s*21] = L_sub(t01,t05);
    move32();
    re[s*29] = L_add(t02,t06);
    move32();
    im[s*29] = L_add(t03,t07);
    move32();

    /* 7. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y12,y44);
    t02 = L_sub(y12,y44);
    t01 = L_add(y13,y45);
    t03 = L_sub(y13,y45);
    t04 = L_add(y28,y60);
    t07 = L_sub(y28,y60);
    t05 = L_add(y61,y29);
    t06 = L_sub(y61,y29);

    /* Post-additions */
    re[s* 6] = L_add(t00,t04);
    move32();
    im[s* 6] = L_add(t01,t05);
    move32();
    re[s*14] = L_sub(t02,t06);
    move32();
    im[s*14] = L_sub(t03,t07);
    move32();
    re[s*22] = L_sub(t00,t04);
    move32();
    im[s*22] = L_sub(t01,t05);
    move32();
    re[s*30] = L_add(t02,t06);
    move32();
    im[s*30] = L_add(t03,t07);
    move32();

    /* 8. FFT4 stage */

    /* Pre-additions */
    t00 = L_add(y14,y46);
    t02 = L_sub(y14,y46);
    t01 = L_add(y15,y47);
    t03 = L_sub(y15,y47);
    t04 = L_add(y30,y62);
    t07 = L_sub(y30,y62);
    t05 = L_add(y63,y31);
    t06 = L_sub(y63,y31);

    /* Post-additions */
    re[s* 7] = L_add(t00,t04);
    move32();
    im[s* 7] = L_add(t01,t05);
    move32();
    re[s*15] = L_sub(t02,t06);
    move32();
    im[s*15] = L_sub(t03,t07);
    move32();
    re[s*23] = L_sub(t00,t04);
    move32();
    im[s*23] = L_sub(t01,t05);
    move32();
    re[s*31] = L_add(t02,t06);
    move32();
    im[s*31] = L_add(t03,t07);
    move32();

}


/**
 * \brief Combined FFT
 *
 * \param    [i/o] re     real part
 * \param    [i/o] im     imag part
 * \param    [i  ] W      rotation factor
 * \param    [i  ] len    length of fft
 * \param    [i  ] dim1   length of fft1
 * \param    [i  ] dim2   length of fft2
 * \param    [i  ] sx     stride real and imag part
 * \param    [i  ] sc     stride phase rotation coefficients
 * \param    [tmp] x      32-bit workbuffer of length=2*len
 * \param    [i  ] Woff   offset for addressing the rotation vector table
 *
 * \return void
 */
static void fftN2(
    Word32 *re,
    Word32 *im,
    const Word16 *W,
    Word16 len,
    Word16 dim1,
    Word16 dim2,
    Word16 sx,
    Word16 sc,
    Word32 *x,
    Word16 Woff
)
{
    Word16 i,j;
    Word32 y[2*20];


    assert( len == (dim1*dim2) );
    assert( (dim1==3) || (dim1==5) || (dim1==8) || (dim1==10) ||                (dim1==15) || (dim1==16) || (dim1==20) || (dim1==30) || (dim1==32) );
    assert( (dim2==4) ||              (dim2==8) || (dim2==10) || (dim2==12)                || (dim2==16) || (dim2==20) );

    FOR (i=0; i<dim2; i++)
    {
        FOR(j=0; j<dim1; j++)
        {
            x[2*i*dim1+2*j]   = re[sx*i+sx*j*dim2];
            move32();
            x[2*i*dim1+2*j+1] = im[sx*i+sx*j*dim2];
            move32();
        }
    }

    SWITCH (dim1)
    {
    case   3:
        FOR(i=0; i<dim2; i++)
        {
            fft3(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case   5:
        FOR(i=0; i<dim2; i++)
        {
            fft5(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case   8:
        FOR(i=0; i<dim2; i++)
        {
            fft8(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case  10:
        FOR(i=0; i<dim2; i++)
        {
            fft10(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;

    case  15:
        FOR(i=0; i<dim2; i++)
        {
            fft15(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case  16:
        FOR(i=0; i<dim2; i++)
        {
            fft16(&x[i*2*dim1],&x[i*2*dim1+1],2,1);
        }
        BREAK;
    case  20:
        FOR(i=0; i<dim2; i++)
        {
            fft20(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case  30:
        FOR(i=0; i<dim2; i++)
        {
            fft30(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    case  32:
        FOR(i=0; i<dim2; i++)
        {
            fft32(&x[i*2*dim1],&x[i*2*dim1+1],2);
        }
        BREAK;
    }

    SWITCH (dim2)
    {
    case  4:
        {
            Word32 x00,x01,x02,x03,x04,x05,x06,x07;
            Word32 t00,t01,t02,t03,t04,t05,t06,t07;

            assert((sc == 2) || (sc == 4));
            j = add(8,0);    /* stride for RotVector_480 */
            FOR(i=0; i<dim1; i++)
            {
                cplxMpy4_4_1(x00,x01,x[2*i+2*0*dim1],x[2*i+2*0*dim1+1])
                IF (i==0)
                {
                    cplxMpy4_4_1(x02,x03,x[2*i+2*1*dim1],x[2*i+2*1*dim1+1])
                    cplxMpy4_4_1(x04,x05,x[2*i+2*2*dim1],x[2*i+2*2*dim1+1])
                    cplxMpy4_4_1(x06,x07,x[2*i+2*3*dim1],x[2*i+2*3*dim1+1])
                }
                ELSE
                {
                    cplxMpy4_4_0(x02,x03,x[2*i+2*1*dim1],x[2*i+2*1*dim1+1],W[sc*i+j*1*dim1-Woff],W[sc*i+j*1*dim1+1-Woff])
                    cplxMpy4_4_0(x04,x05,x[2*i+2*2*dim1],x[2*i+2*2*dim1+1],W[sc*i+j*2*dim1-Woff],W[sc*i+j*2*dim1+1-Woff])
                    cplxMpy4_4_0(x06,x07,x[2*i+2*3*dim1],x[2*i+2*3*dim1+1],W[sc*i+j*3*dim1-Woff],W[sc*i+j*3*dim1+1-Woff])
                }
                t00 = L_add(x00,x04);
                t02 = L_sub(x00,x04);
                t01 = L_add(x01,x05);
                t03 = L_sub(x01,x05);
                t04 = L_add(x02,x06);
                t07 = L_sub(x02,x06);
                t05 = L_add(x07,x03);
                t06 = L_sub(x07,x03);
                re[sx*i+sx*0*dim1] = L_add(t00,t04);
                move32();
                im[sx*i+sx*0*dim1] = L_add(t01,t05);
                move32();
                re[sx*i+sx*1*dim1] = L_sub(t02,t06);
                move32();
                im[sx*i+sx*1*dim1] = L_sub(t03,t07);
                move32();
                re[sx*i+sx*2*dim1] = L_sub(t00,t04);
                move32();
                im[sx*i+sx*2*dim1] = L_sub(t01,t05);
                move32();
                re[sx*i+sx*3*dim1] = L_add(t02,t06);
                move32();
                im[sx*i+sx*3*dim1] = L_add(t03,t07);
                move32();
            }
            BREAK;
        }
    case   8:
        {
            Word32 x00,x01,x02,x03,x04,x05,x06,x07,x08,x09,x10,x11,x12,x13,x14,x15;
            Word32 t00,t01,t02,t03,t04,t05,t06,t07,t08,t09,t10,t11,t12,t13,t14,t15;
            Word32 s00,s01,s02,s03,s04,s05,s06,s07,s08,s09,s10,s11,s12,s13,s14,s15;

            FOR(i=0; i<dim1; i++)
            {
                cplxMpy4_8_1(x00,x01,x[2*i+2*0*dim1],x[2*i+2*0*dim1+1])
                IF (i==0)
                {
                    cplxMpy4_8_1(x02,x03,x[2*i+2*1*dim1],x[2*i+2*1*dim1+1])
                    cplxMpy4_8_1(x04,x05,x[2*i+2*2*dim1],x[2*i+2*2*dim1+1])
                    cplxMpy4_8_1(x06,x07,x[2*i+2*3*dim1],x[2*i+2*3*dim1+1])
                    cplxMpy4_8_1(x08,x09,x[2*i+2*4*dim1],x[2*i+2*4*dim1+1])
                    cplxMpy4_8_1(x10,x11,x[2*i+2*5*dim1],x[2*i+2*5*dim1+1])
                    cplxMpy4_8_1(x12,x13,x[2*i+2*6*dim1],x[2*i+2*6*dim1+1])
                    cplxMpy4_8_1(x14,x15,x[2*i+2*7*dim1],x[2*i+2*7*dim1+1])
                }
                ELSE
                {
                    cplxMpy4_8_0(x02,x03,x[2*i+2*1*dim1],x[2*i+2*1*dim1+1],W[sc*i+sc*1*dim1-Woff],W[sc*i+sc*1*dim1+1-Woff])
                    cplxMpy4_8_0(x04,x05,x[2*i+2*2*dim1],x[2*i+2*2*dim1+1],W[sc*i+sc*2*dim1-Woff],W[sc*i+sc*2*dim1+1-Woff])
                    cplxMpy4_8_0(x06,x07,x[2*i+2*3*dim1],x[2*i+2*3*dim1+1],W[sc*i+sc*3*dim1-Woff],W[sc*i+sc*3*dim1+1-Woff])
                    cplxMpy4_8_0(x08,x09,x[2*i+2*4*dim1],x[2*i+2*4*dim1+1],W[sc*i+sc*4*dim1-Woff],W[sc*i+sc*4*dim1+1-Woff])
                    cplxMpy4_8_0(x10,x11,x[2*i+2*5*dim1],x[2*i+2*5*dim1+1],W[sc*i+sc*5*dim1-Woff],W[sc*i+sc*5*dim1+1-Woff])
                    cplxMpy4_8_0(x12,x13,x[2*i+2*6*dim1],x[2*i+2*6*dim1+1],W[sc*i+sc*6*dim1-Woff],W[sc*i+sc*6*dim1+1-Woff])
                    cplxMpy4_8_0(x14,x15,x[2*i+2*7*dim1],x[2*i+2*7*dim1+1],W[sc*i+sc*7*dim1-Woff],W[sc*i+sc*7*dim1+1-Woff])
                }
                t00 = L_shr(L_add(x00,x08),SCALEFACTORN2-1);
                t02 = L_shr(L_sub(x00,x08),SCALEFACTORN2-1);
                t01 = L_shr(L_add(x01,x09),SCALEFACTORN2-1);
                t03 = L_shr(L_sub(x01,x09),SCALEFACTORN2-1);
                t04 = L_shr(L_add(x02,x10),SCALEFACTORN2-1);
                t06 = L_sub(x02,x10);
                t05 = L_shr(L_add(x03,x11),SCALEFACTORN2-1);
                t07 = L_sub(x03,x11);
                t08 = L_shr(L_add(x04,x12),SCALEFACTORN2-1);
                t10 = L_shr(L_sub(x04,x12),SCALEFACTORN2-1);
                t09 = L_shr(L_add(x05,x13),SCALEFACTORN2-1);
                t11 = L_shr(L_sub(x05,x13),SCALEFACTORN2-1);
                t12 = L_shr(L_add(x06,x14),SCALEFACTORN2-1);
                t14 = L_sub(x06,x14);
                t13 = L_shr(L_add(x07,x15),SCALEFACTORN2-1);
                t15 = L_sub(x07,x15);

                s00 = L_add(t00,t08);
                s04 = L_sub(t00,t08);
                s01 = L_add(t01,t09);
                s05 = L_sub(t01,t09);
                s08 = L_sub(t02,t11);
                s10 = L_add(t02,t11);
                s09 = L_add(t03,t10);
                s11 = L_sub(t03,t10);
                s02 = L_add(t04,t12);
                s07 = L_sub(t04,t12);
                s03 = L_add(t05,t13);
                s06 = L_sub(t13,t05);

                t01 = L_shr(L_add(t06,t14),SCALEFACTORN2-1);
                t02 = L_shr(L_sub(t06,t14),SCALEFACTORN2-1);
                t00 = L_shr(L_add(t07,t15),SCALEFACTORN2-1);
                t03 = L_shr(L_sub(t07,t15),SCALEFACTORN2-1);

                s12 = Mpy_32_xx(L_add(t00,t02),C81);
                s14 = Mpy_32_xx(L_sub(t00,t02),C81);
                s13 = Mpy_32_xx(L_sub(t03,t01),C81);
                s15 = Mpy_32_xx(L_add(t01,t03),C82);

                re[sx*i+sx*0*dim1] = L_add(s00,s02);
                move32();
                im[sx*i+sx*0*dim1] = L_add(s01,s03);
                move32();
                re[sx*i+sx*1*dim1] = L_add(s10,s12);
                move32();
                im[sx*i+sx*1*dim1] = L_add(s11,s13);
                move32();
                re[sx*i+sx*2*dim1] = L_sub(s04,s06);
                move32();
                im[sx*i+sx*2*dim1] = L_sub(s05,s07);
                move32();
                re[sx*i+sx*3*dim1] = L_add(s08,s14);
                move32();
                im[sx*i+sx*3*dim1] = L_add(s09,s15);
                move32();
                re[sx*i+sx*4*dim1] = L_sub(s00,s02);
                move32();
                im[sx*i+sx*4*dim1] = L_sub(s01,s03);
                move32();
                re[sx*i+sx*5*dim1] = L_sub(s10,s12);
                move32();
                im[sx*i+sx*5*dim1] = L_sub(s11,s13);
                move32();
                re[sx*i+sx*6*dim1] = L_add(s04,s06);
                move32();
                im[sx*i+sx*6*dim1] = L_add(s05,s07);
                move32();
                re[sx*i+sx*7*dim1] = L_sub(s08,s14);
                move32();
                im[sx*i+sx*7*dim1] = L_sub(s09,s15);
                move32();
            }
            BREAK;
        }

    case  10:
        assert(dim1 == 20 || dim1 == 10); /* cplxMpy4_10_0 contains shift values hardcoded for 20x10 */
        FOR(j=0; j<dim2; j++)
        {
            cplxMpy4_10_1(y[2*j],y[2*j+1],x[2*0+2*j*dim1],x[2*0+2*j*dim1+1])
        }
        fft10(&y[0],&y[1],2);
        FOR(j=0; j<dim2; j++)
        {
            re[sx*0+sx*j*dim1] = y[2*j];
            move32();
            im[sx*0+sx*j*dim1] = y[2*j+1];
            move32();
        }
        FOR(i=1; i<dim1; i++)
        {
            cplxMpy4_10_1(y[2*(0+0)],y[2*(0+0)+1],x[2*i+2*(0+0)*dim1],x[2*i+2*(0+0)*dim1+1])
            FOR(j=1; j<dim2; j++)
            {
                cplxMpy4_10_0(y[2*(j+0)],y[2*(j+0)+1],x[2*i+2*(j+0)*dim1],x[2*i+2*(j+0)*dim1+1],W[sc*i+sc*j*dim1-Woff],W[sc*i+sc*j*dim1+1-Woff])
            }
            fft10(&y[0],&y[1],2);
            FOR(j=0; j<dim2; j++)
            {
                re[sx*i+sx*j*dim1] = y[2*j];
                move32();
                im[sx*i+sx*j*dim1] = y[2*j+1];
                move32();
            }
        }
        BREAK;

    case  12:
        assert(dim1 == 16); /* cplxMpy4_12_0 contains shift values hardcoded for 16x12 */
        FOR(j=0; j<dim2; j++)
        {
            cplxMpy4_12_1(y[2*j],y[2*j+1],x[2*0+2*j*dim1],x[2*0+2*j*dim1+1])
        }
        fft12(y);
        FOR(j=0; j<dim2; j++)
        {
            re[sx*0+sx*j*dim1] = y[2*j];
            move32();
            im[sx*0+sx*j*dim1] = y[2*j+1];
            move32();
        }
        FOR(i=1; i<dim1; i++)
        {
            cplxMpy4_12_1(y[2*(0+0)],y[2*(0+0)+1],x[2*i+2*(0+0)*dim1],x[2*i+2*(0+0)*dim1+1])
            cplxMpy4_12_0(y[2*(0+1)],y[2*(0+1)+1],x[2*i+2*(0+1)*dim1],x[2*i+2*(0+1)*dim1+1],W[len+sc*i+0*dim1-Woff],W[len+sc*i+0*dim1+1-Woff])
            FOR(j=2; j<dim2; j=j+2)
            {
                cplxMpy4_12_0(y[2*(j+0)],y[2*(j+0)+1],x[2*i+2*(j+0)*dim1],x[2*i+2*(j+0)*dim1+1],W[    sc*i+j*dim1-Woff],W[    sc*i+j*dim1+1-Woff])
                cplxMpy4_12_0(y[2*(j+1)],y[2*(j+1)+1],x[2*i+2*(j+1)*dim1],x[2*i+2*(j+1)*dim1+1],W[len+sc*i+j*dim1-Woff],W[len+sc*i+j*dim1+1-Woff])
            }
            fft12(y);
            FOR(j=0; j<dim2; j++)
            {
                re[sx*i+sx*j*dim1] = y[2*j];
                move32();
                im[sx*i+sx*j*dim1] = y[2*j+1];
                move32();
            }
        }
        BREAK;

    case  16:
        /*assert(dim1 == 20); */ /* cplxMpy4_16_0 contains shift values hardcoded for 20x16 */
        FOR(j=0; j<dim2; j++)
        {
            cplxMpy4_16_1(y[2*j],y[2*j+1],x[2*0+2*j*dim1],x[2*0+2*j*dim1+1])
        }
        fft16(&y[0],&y[1],2,0);
        FOR(j=0; j<dim2; j++)
        {
            re[sx*0+sx*j*dim1] = y[2*j];
            move32();
            im[sx*0+sx*j*dim1] = y[2*j+1];
            move32();
        }
        FOR(i=1; i<dim1; i++)
        {
            cplxMpy4_16_1(y[2*(0+0)],y[2*(0+0)+1],x[2*i+2*(0+0)*dim1],x[2*i+2*(0+0)*dim1+1])
            cplxMpy4_16_0(y[2*(0+1)],y[2*(0+1)+1],x[2*i+2*(0+1)*dim1],x[2*i+2*(0+1)*dim1+1],W[len+sc*i+0*dim1-Woff],W[len+sc*i+0*dim1+1-Woff])
            FOR(j=2; j<dim2; j=j+2)
            {
                cplxMpy4_16_0(y[2*(j+0)],y[2*(j+0)+1],x[2*i+2*(j+0)*dim1],x[2*i+2*(j+0)*dim1+1],W[    sc*i+j*dim1-Woff],W[    sc*i+j*dim1+1-Woff])
                cplxMpy4_16_0(y[2*(j+1)],y[2*(j+1)+1],x[2*i+2*(j+1)*dim1],x[2*i+2*(j+1)*dim1+1],W[len+sc*i+j*dim1-Woff],W[len+sc*i+j*dim1+1-Woff])
            }
            fft16(&y[0],&y[1],2,0);
            FOR(j=0; j<dim2; j++)
            {
                re[sx*i+sx*j*dim1] = y[2*j];
                move32();
                im[sx*i+sx*j*dim1] = y[2*j+1];
                move32();
            }
        }
        BREAK;

    case  20:
        assert(dim1 == 20 || dim1 == 30); /* cplxMpy4_10_0 contains shift values hardcoded for 20x10 */
        IF ( sub(dim1,20) == 0)
        {
            FOR(j=0; j<dim2; j++)
            {
                cplxMpy4_10_1(y[2*j],y[2*j+1],x[2*0+2*j*dim1],x[2*0+2*j*dim1+1])
            }
            fft20(&y[0],&y[1],2);
            FOR(j=0; j<dim2; j++)
            {
                re[sx*0+sx*j*dim1] = y[2*j];
                move32();
                im[sx*0+sx*j*dim1] = y[2*j+1];
                move32();
            }
            FOR(i=1; i<dim1; i++)
            {
                cplxMpy4_10_1(y[2*(0+0)],y[2*(0+0)+1],x[2*i+2*(0+0)*dim1],x[2*i+2*(0+0)*dim1+1])
                cplxMpy4_10_0(y[2*(0+1)],y[2*(0+1)+1],x[2*i+2*(0+1)*dim1],x[2*i+2*(0+1)*dim1+1],W[len+sc*i+0*dim1-Woff],W[len+sc*i+0*dim1+1-Woff])
                FOR(j=2; j<dim2; j=j+2)
                {
                    cplxMpy4_10_0(y[2*(j+0)],y[2*(j+0)+1],x[2*i+2*(j+0)*dim1],x[2*i+2*(j+0)*dim1+1],W[    sc*i+j*dim1-Woff],W[    sc*i+j*dim1+1-Woff])
                    cplxMpy4_10_0(y[2*(j+1)],y[2*(j+1)+1],x[2*i+2*(j+1)*dim1],x[2*i+2*(j+1)*dim1+1],W[len+sc*i+j*dim1-Woff],W[len+sc*i+j*dim1+1-Woff])
                }
                fft20(&y[0],&y[1],2);
                FOR(j=0; j<dim2; j++)
                {
                    re[sx*i+sx*j*dim1] = y[2*j];
                    move32();
                    im[sx*i+sx*j*dim1] = y[2*j+1];
                    move32();
                }
            }
        }
        ELSE
        {
            FOR(j=0; j<dim2; j++)
            {
                cplxMpy4_20_30_1(y[2*j],y[2*j+1],x[2*0+2*j*dim1],x[2*0+2*j*dim1+1])
            }
            fft20(&y[0],&y[1],2);
            FOR(j=0; j<dim2; j++)
            {
                re[sx*0+sx*j*dim1] = y[2*j];
                move32();
                im[sx*0+sx*j*dim1] = y[2*j+1];
                move32();
            }
            FOR(i=1; i<dim1; i++)
            {
                cplxMpy4_20_30_1(y[2*(0+0)],y[2*(0+0)+1],x[2*i+2*(0+0)*dim1],x[2*i+2*(0+0)*dim1+1])
                cplxMpy4_20_30_0(y[2*(0+1)],y[2*(0+1)+1],x[2*i+2*(0+1)*dim1],x[2*i+2*(0+1)*dim1+1],W[len+sc*i+0*dim1-Woff],W[len+sc*i+0*dim1+1-Woff])
                FOR(j=2; j<dim2; j=j+2)
                {
                    cplxMpy4_20_30_0(y[2*(j+0)],y[2*(j+0)+1],x[2*i+2*(j+0)*dim1],x[2*i+2*(j+0)*dim1+1],W[    sc*i+j*dim1-Woff],W[    sc*i+j*dim1+1-Woff])
                    cplxMpy4_20_30_0(y[2*(j+1)],y[2*(j+1)+1],x[2*i+2*(j+1)*dim1],x[2*i+2*(j+1)*dim1+1],W[len+sc*i+j*dim1-Woff],W[len+sc*i+j*dim1+1-Woff])
                }
                fft20(&y[0],&y[1],2);
                FOR(j=0; j<dim2; j++)
                {
                    re[sx*i+sx*j*dim1] = y[2*j];
                    move32();
                    im[sx*i+sx*j*dim1] = y[2*j+1];
                    move32();
                }
            }
        }
        BREAK;
    }

}


/**
 * \brief Complex valued FFT
 *
 * \param    [i/o] re          real part
 * \param    [i/o] im          imag part
 * \param    [i  ] sizeOfFft   length of fft
 * \param    [i  ] s           stride real and imag part
 * \param    [i  ] scale       scalefactor
 *
 * \return void
 */
void BASOP_cfft(Word32 *re, Word32 *im, Word16 sizeOfFft, Word16 s, Word16 *scale, Word32 x[2*BASOP_CFFT_MAX_LENGTH])
{
    SWITCH(sizeOfFft)
    {
    case 2:
        fft2(re,im /*,s*/);
        s = add(*scale,SCALEFACTOR2);
        BREAK;

    case 3:
        assert(re+1 == im);
        fft3(re,im,s);
        s = add(*scale,SCALEFACTOR3);
        BREAK;

    case 4:
        fft4(re,im,s);
        s = add(*scale,SCALEFACTOR4);
        BREAK;

    case 5:
        fft5(re,im,s);
        s = add(*scale,SCALEFACTOR5);
        BREAK;

    case 8:
        fft8(re,im,s);
        s = add(*scale,SCALEFACTOR8);
        BREAK;

    case 10:
        fft10(re,im,s);
        s = add(*scale,SCALEFACTOR10);
        BREAK;

    case 12:
        assert( re+1 == im);
        fft12(re);
        s = add(*scale,SCALEFACTOR12);
        BREAK;

    case 15:
        fft15(re,im,s);
        s = add(*scale,SCALEFACTOR15);
        BREAK;

    case 16:
        fft16(re,im,s,1);
        s = add(*scale,SCALEFACTOR16);
        BREAK;

    case 20:
        fft20(re,im,s);
        s = add(*scale,SCALEFACTOR20);
        BREAK;

    case 24:
        {
            fftN2(re,im,RotVector_24,24,3,8,s,2,x,6);
            s = add(*scale,SCALEFACTOR24);
            BREAK;
        }

    case 30:
        fft30(re,im,s);
        s = add(*scale,SCALEFACTOR30);
        BREAK;

    case 32:
        fft32(re,im,s);
        s = add(*scale,SCALEFACTOR32);
        BREAK;

    case 40:
        {
            fftN2(re,im,RotVector_320,40,5,8,s,8,x,40);
            s = add(*scale,SCALEFACTOR40);
            BREAK;
        }

    case 60:
        {
            fftN2(re,im,RotVector_480,60,15,4,s,4,x,60);
            s = add(*scale,SCALEFACTOR60);
            BREAK;
        }

    case 64:
        {
            fftN2(re,im,RotVector_256,64,8,8,s,8,x,64);
            s = add(*scale,SCALEFACTOR64);
            BREAK;
        }

    case 80:
        {
            fftN2(re,im,RotVector_320,80,10,8,s,4,x,40);
            s = add(*scale,SCALEFACTOR80);
            BREAK;
        }
    case 100:
        {
            fftN2(re,im,RotVector_400,100,10,10,s,4,x,40);
            s = add(*scale,SCALEFACTOR100);
            BREAK;
        }
    case 120:
        {
            fftN2(re,im,RotVector_480,120,15,8,s,4,x,60);
            s = add(*scale,SCALEFACTOR120);
            BREAK;
        }

    case 128:
        {
            fftN2(re,im,RotVector_256,128,16,8,s,4,x,64);
            s = add(*scale,SCALEFACTOR128);
            BREAK;
        }

    case 160:
        {
            fftN2(re,im,RotVector_320,160,20,8,s,2,x,40);
            s = add(*scale,SCALEFACTOR160);
            BREAK;
        }

    case 192:
        {
            fftN2(re,im,RotVector_192,192,16,12,s,2,x,32);
            s = add(*scale,SCALEFACTOR192);
            BREAK;
        }

    case 200:
        {
            fftN2(re,im,RotVector_400,200,20,10,s,2,x,40);
            s = add(*scale,SCALEFACTOR200);
            BREAK;
        }

    case 240:
        {
            fftN2(re,im,RotVector_480,240,30,8,s,2,x,60);
            s = add(*scale,SCALEFACTOR240);
            BREAK;
        }

    case 256:
        {
            fftN2(re,im,RotVector_256,256,32,8,s,2,x,64);
            s = add(*scale,SCALEFACTOR256);
            BREAK;
        }

    case 320:
        {
            fftN2(re,im,RotVector_320,320,20,16,s,2,x,40);
            s = add(*scale,SCALEFACTOR320);
            BREAK;
        }

    case 400:
        {
            fftN2(re,im,RotVector_400,400,20,20,s,2,x,40);
            s = add(*scale,SCALEFACTOR400);
            BREAK;
        }

    case 480:
        {
            fftN2(re,im,RotVector_480,480,30,16,s,2,x,60);
            s = add(*scale,SCALEFACTOR480);
            BREAK;
        }
    case 600:
        {
            fftN2(re,im,RotVector_600,600,30,20,s,2,x,60);
            s = add(*scale,SCALEFACTOR600);
            BREAK;
        }
    default:
        assert(0);
    }
    *scale = s;
    move16();
}



#define RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1, w2, xb0, xb1, xt0, xt1) \
{ \
  xb0 = L_shr(x[2*i+0],2); \
  xb1 = L_shr(x[2*i+1],2); \
  xt0 = L_shr(x[sizeOfFft-2*i+0],2); \
  xt1 = L_shr(x[sizeOfFft-2*i+1],2); \
  t1 = L_sub(xb0,xt0); \
  t2 = L_add(xb1,xt1); \
  t3 = L_sub(Mpy_32_16_1(t1,w1),Mpy_32_16_1(t2,w2)); \
  t4 = L_add(Mpy_32_16_1(t1,w2),Mpy_32_16_1(t2,w1)); \
  t1 = L_add(xb0,xt0); \
  t2 = L_sub(xb1,xt1); \
}

#define RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1, w2, xb0, xb1, xt0, xt1) \
{ \
  xb0 = L_shr(x[2*i+0],2); \
  xb1 = L_shr(x[2*i+1],2); \
  xt0 = L_shr(x[sizeOfFft-2*i+0],2); \
  xt1 = L_shr(x[sizeOfFft-2*i+1],2); \
  t1 = L_sub(xb0,xt0); \
  t2 = L_add(xb1,xt1); \
  t3 = L_add(Mpy_32_16_1(t1,w1),Mpy_32_16_1(t2,w2)); \
  t4 = L_sub(Mpy_32_16_1(t2,w1),Mpy_32_16_1(t1,w2)); \
  t1 = L_add(xb0,xt0); \
  t2 = L_sub(xb1,xt1); \
}

/**
 * \brief Real valued FFT
 *
 *        forward rFFT (isign == -1):
 *        The input vector contains sizeOfFft real valued time samples. The output vector contains sizeOfFft/2 complex valued
 *        spectral values. The spectral values resides interleaved in the output vector. x[1] contains re[sizeOfFft], because
 *        x[1] is zero by default. This allows use of sizeOfFft length buffer instead of sizeOfFft+1.
 *
 *        inverse rFFT (isign == +1):
 *        The input vector contains sizeOfFft complex valued spectral values. The output vector contains sizeOfFft real valued
 *        time samples. The spectral values resides interleaved in the input vector. x[1] contains re[sizeOfFft].
 *        (see also forward rFFT)
 *
 * \param    [i/o] x           real input / real and imag output interleaved
 * \param    [i  ] sizeOfFft   length of fft
 * \param    [i  ] scale       scalefactor
 * \param    [i  ] isign       forward (-1) / backward (+1)
 *
 * \return void
 */
void BASOP_rfft(Word32 *x, Word16 sizeOfFft, Word16 *scale, Word16 isign)
{
    Word16 i, s=0, sizeOfFft2, sizeOfFft4, sizeOfFft8, wstride;  /* clear s to calm down compiler */
    Word32 t1, t2, t3, t4, xb0, xb1, xt0, xt1;
    const PWord16 *w1;
    Word16 c1;
    Word16 c2;
    Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH];



    sizeOfFft2 = shr(sizeOfFft,1);
    sizeOfFft4 = shr(sizeOfFft,2);
    sizeOfFft8 = shr(sizeOfFft,3);

    BASOP_getTables(NULL, &w1, &wstride, sizeOfFft2);

    SWITCH (isign)
    {
    case -1:

        BASOP_cfft(&x[0], &x[1], sizeOfFft2, 2, scale, workBuffer);

        xb0  = L_shr(x[0],1);
        xb1  = L_shr(x[1],1);
        x[0] = L_add(xb0,xb1);
        move32();
        x[1] = L_sub(xb0,xb1);
        move32();

        FOR (i=1; i<sizeOfFft8; i++)
        {
            RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1[i*wstride].v.im, w1[i*wstride].v.re, xb0, xb1, xt0, xt1)
            x[2*i]             =  L_sub(t1,t3);
            move32();
            x[2*i+1]           =  L_sub(t2,t4);
            move32();
            x[sizeOfFft-2*i]   =  L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] =  L_negate(L_add(t2,t4));
            move32();
        }

        FOR (i=sizeOfFft8; i<sizeOfFft4; i++)
        {
            RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1[(sizeOfFft4-i)*wstride].v.re, w1[(sizeOfFft4-i)*wstride].v.im, xb0, xb1, xt0, xt1)
            x[2*i]             =  L_sub(t1,t3);
            move32();
            x[2*i+1]           =  L_sub(t2,t4);
            move32();
            x[sizeOfFft-2*i]   =  L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] =  L_negate(L_add(t2,t4));
            move32();
        }

        x[sizeOfFft-2*i]   =  L_shr(x[2*i+0],1);
        move32();
        x[sizeOfFft-2*i+1] =  L_negate(L_shr(x[2*i+1],1));
        move32();

        *scale = add(*scale,1);
        move16();
        BREAK;

    case +1:

        xb0 = L_shr(x[0],2);
        xb1 = L_shr(x[1],2);
        x[0] = L_add(xb0,xb1);
        move32();
        x[1] = L_sub(xb1,xb0);
        move32();

        FOR (i=1; i<sizeOfFft8; i++)
        {
            RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1[i*wstride].v.im, w1[i*wstride].v.re, xb0, xb1, xt0, xt1)

            x[2*i]             = L_sub(t1,t3);
            move32();
            x[2*i+1]           = L_sub(t4,t2);
            move32();
            x[sizeOfFft-2*i]   = L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] = L_add(t2,t4);
            move32();
        }

        FOR (i=sizeOfFft8; i<sizeOfFft4; i++)
        {
            RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1[(sizeOfFft4-i)*wstride].v.re, w1[(sizeOfFft4-i)*wstride].v.im, xb0, xb1, xt0, xt1)

            x[2*i]             = L_sub(t1,t3);
            move32();
            x[2*i+1]           = L_sub(t4,t2);
            move32();
            x[sizeOfFft-2*i]   = L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] = L_add(t2,t4);
            move32();
        }

        x[sizeOfFft-2*i]     = L_shr(x[2*i+0],1);
        move32();
        x[sizeOfFft-2*i+1]   = L_shr(x[2*i+1],1);
        move32();

        BASOP_cfft(&x[0], &x[1], sizeOfFft2, 2, scale, workBuffer);

        SWITCH (sizeOfFft)
        {
        case 40:
        case 80:
        case 320:
        case 640:
            c1 = FFTC(0x66666680);
            c2 = FFTC(0x99999980);
            FOR(i=0; i<sizeOfFft2; i++)
            {
                x[2*i  ] = Mpy_32_xx(x[2*i  ],c1);
                move32();
                x[2*i+1] = Mpy_32_xx(x[2*i+1],c2);
                move32();
            }
            BREAK;

        case 64:
        case 256:
        case 512:
            FOR(i=0; i<sizeOfFft2; i++)
            {
                x[2*i+1] = L_negate(x[2*i+1]);
                move32();
            }
            BREAK;

        default:
            assert(0);
        }

        SWITCH (sizeOfFft)
        {
        case 40:
            s = add(*scale,2-5);
            BREAK;

        case 64:
            s = add(*scale,2-6);
            BREAK;

        case 80:
            s = add(*scale,2-6);
            BREAK;

        case 256:
            s = add(*scale,2-8);
            BREAK;

        case 320:
            s = add(*scale,2-8);
            BREAK;

        case 512:
            s = add(*scale,2-9);
            BREAK;

        case 640:
            s = add(*scale,2-9);
            BREAK;

        default:
            assert(0);
        }
        *scale = s;
        move16();
        BREAK;
    }

}


