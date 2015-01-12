/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/
/*!
  \file
  \brief  Complex cldfb analysis/synthesis, $Revision: 1214 $
  This module contains the cldfb filterbank for analysis [ cplxAnalysisCldfbFiltering() ] and
  synthesis [ cplxSynthesisCldfbFiltering() ]. It is a polyphase implementation of a complex
  exponential modulated filter bank. The analysis part usually runs at half the sample rate
  than the synthesis part. (So called "dual-rate" mode.)

  \anchor PolyphaseFiltering <h2>About polyphase filtering</h2>
  The polyphase implementation of a filterbank requires filtering at the input and output.
  This is implemented as part of cplxAnalysisCldfbFiltering() and cplxSynthesisCldfbFiltering().

*/

#include "stl.h"
#include "cnst_fx.h"
#include "stat_com.h"
#include "rom_com_fx.h"
#include "basop_util.h"
#include "prot_fx.h"
#include <assert.h>

#define STATE_BUFFER_SIZE             ( 9+16 )

#define CLDFB_NO_POLY                 ( 5 )
#define SYN_FILTER_HEADROOM           ( 1 )

#define CLDFB_LDCLDFB_PFT_SCALE           ( 0 )
#define CLDFB_CLDFB80_O24_PFT_SCALE     ( 1 )
#define CLDFB_CLDFB80_O5_PFT_SCALE      ( 1 )
#define CLDFB_CLDFB80_PFT_SCALE         ( 1 )



#define SYN_FILTER_HEADROOM_1MS       ( 2 )
#define SYN_FILTER_HEADROOM_2MS       ( 2 )
#define SYN_FILTER_HEADROOM_2_5MS     ( 2 )
#define SYN_FILTER_HEADROOM_8MS       ( 1 )

#define N8                            (  4 )
#define N10                           (  5 )
#define N16                           (  8 )
#define N20                           ( 10 )
#define N30                           ( 15 )
#define N32                           ( 16 )
#define N40                           ( 20 )
#define N60                           ( 30 )

static void
cldfb_init_proto_and_twiddles(HANDLE_CLDFB_FILTER_BANK hs);

#define cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi)       zr = L_sub(Mpy_32_16_1(*xr,*cr), Mpy_32_16_1(*xi,*ci)); \
                                                                zi = L_add(Mpy_32_16_1(*xr,*ci), Mpy_32_16_1(*xi,*cr)); \
                                                                *yr = zr;                                 move32(); \
                                                                *yi = zi;                                 move32(); \
                                                                yr+=syr, yi+=syi, xr+=sxr, xi+=sxi, cr++, ci++

#define cplxMpy(ryr,ryi,iyr,iyi,rxr,rxi,ixr,ixi,cr,ci,g,sx,sr)  ryr = Mpy_32_16_1(L_sub(Mpy_32_16_1(*rxr,*cr),Mpy_32_16_1(*rxi,*ci)),g);   \
                                                                ryi = Mpy_32_16_1(L_add(Mpy_32_16_1(*rxr,*ci),Mpy_32_16_1(*rxi,*cr)),g);   \
                                                                iyr = Mpy_32_16_1(L_sub(Mpy_32_16_1(*ixr,*cr),Mpy_32_16_1(*ixi,*ci)),g);   \
                                                                iyi = Mpy_32_16_1(L_add(Mpy_32_16_1(*ixr,*ci),Mpy_32_16_1(*ixi,*cr)),g);   \
                                                                rxr+=sx, rxi+=sx, ixr+=sx, ixi+=sx, cr+=sr, ci+=sr

#define add1(y1,y2,y3,y4,rr12,ri12,ir12,ii12,s)                 *y1 = round_fx(L_shl(L_negate(L_add(rr12,ii12)),s));                 \
                                                                *y2 = round_fx(L_shl(L_negate(L_add(ri12,ir12)),s));                 \
                                                                *y3 = round_fx(L_shl(L_sub(rr12,ii12),s));                           \
                                                                *y4 = round_fx(L_shl(L_sub(ir12,ri12),s));                           \
                                                                y1+=2, y2-=2, y3-=2, y4+=2


#define add2(y1,y2,y3,y4,rr12,ri12,ir12,ii12,s)                 *y1 = round_fx(L_shl(L_add(ri12,ir12),s));                           \
                                                                *y2 = round_fx(L_shl(L_add(rr12,ii12),s));                           \
                                                                *y3 = round_fx(L_shl(L_sub(ir12,ri12),s));                           \
                                                                *y4 = round_fx(L_shl(L_sub(rr12,ii12),s));                           \
                                                                y1+=2, y2-=2, y3-=2, y4+=2

#define ptrUpdate16(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22)                                 \
                                                                y11 += 2*N8, y12 -= 2*N8, y13 -= 2*N8, y14 += 2*N8, r11 -= 1*N8;     \
                                                                x11 -= 2*N8, x12 -= 2*N8, x13 -= 2*N8, x14 -= 2*N8, r12 -= 1*N8;     \
                                                                y21 += 2*N8, y22 -= 2*N8, y23 -= 2*N8, y24 += 2*N8, r21 += 1*N8;     \
                                                                x21 += 2*N8, x22 += 2*N8, x23 += 2*N8, x24 += 2*N8, r22 += 1*N8

#define ptrUpdate20(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22)                                  \
                                                                y11 += 2*N10, y12 -= 2*N10, y13 -= 2*N10, y14 += 2*N10, r11 -= 1*N10; \
                                                                x11 -= 2*N10, x12 -= 2*N10, x13 -= 2*N10, x14 -= 2*N10, r12 -= 1*N10; \
                                                                y21 += 2*N10, y22 -= 2*N10, y23 -= 2*N10, y24 += 2*N10, r21 += 1*N10; \
                                                                x21 += 2*N10, x22 += 2*N10, x23 += 2*N10, x24 += 2*N10, r22 += 1*N10



static void cplxMult10(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    Word32 zr, zi;

    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
}

static void cplxMult16(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    Word32 zr, zi;

    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
    cplxMpyS(yr,yi,xr,xi,cr,ci,syr,syi,sxr,sxi,zr,zi);
}

static void cplxMult20(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    cplxMult10(&yr[0*N10*syr], &yi[0*N10*syi], &xr[0*N10*sxr], &xi[0*N10*sxi], &cr[0*N10], &ci[0*N10], syr, syi, sxr, sxi);
    cplxMult10(&yr[1*N10*syr], &yi[1*N10*syi], &xr[1*N10*sxr], &xi[1*N10*sxi], &cr[1*N10], &ci[1*N10], syr, syi, sxr, sxi);
}

static void cplxMult30(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    cplxMult10(&yr[0*N10*syr], &yi[0*N10*syi], &xr[0*N10*sxr], &xi[0*N10*sxi], &cr[0*N10], &ci[0*N10], syr, syi, sxr, sxi);
    cplxMult10(&yr[1*N10*syr], &yi[1*N10*syi], &xr[1*N10*sxr], &xi[1*N10*sxi], &cr[1*N10], &ci[1*N10], syr, syi, sxr, sxi);
    cplxMult10(&yr[2*N10*syr], &yi[2*N10*syi], &xr[2*N10*sxr], &xi[2*N10*sxi], &cr[2*N10], &ci[2*N10], syr, syi, sxr, sxi);
}

static void cplxMult32(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    cplxMult16(&yr[0*N16*syr], &yi[0*N16*syi], &xr[0*N16*sxr], &xi[0*N16*sxi], &cr[0*N16], &ci[0*N16], syr, syi, sxr, sxi);
    cplxMult16(&yr[1*N16*syr], &yi[1*N16*syi], &xr[1*N16*sxr], &xi[1*N16*sxi], &cr[1*N16], &ci[1*N16], syr, syi, sxr, sxi);
}

static void cplxMult40(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    cplxMult20(&yr[0*N20*syr], &yi[0*N20*syi], &xr[0*N20*sxr], &xi[0*N20*sxi], &cr[0*N20], &ci[0*N20], syr, syi, sxr, sxi);
    cplxMult20(&yr[1*N20*syr], &yi[1*N20*syi], &xr[1*N20*sxr], &xi[1*N20*sxi], &cr[1*N20], &ci[1*N20], syr, syi, sxr, sxi);
}

static void cplxMult60(Word32 *yr, Word32 *yi, Word32 *xr, Word32 *xi, const Word16 *cr, const Word16 *ci, Word16 syr, Word16 syi, Word16 sxr, Word16 sxi)
{
    cplxMult30(&yr[0*N30*syr], &yi[0*N30*syi], &xr[0*N30*sxr], &xi[0*N30*sxi], &cr[0*N30], &ci[0*N30], syr, syi, sxr, sxi);
    cplxMult30(&yr[1*N30*syr], &yi[1*N30*syi], &xr[1*N30*sxr], &xi[1*N30*sxi], &cr[1*N30], &ci[1*N30], syr, syi, sxr, sxi);
}

static void cplxMultAdd10_1(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

static void cplxMultAdd10_2(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

static void cplxMultAdd16_1(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

static void cplxMultAdd16_2(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

static void cplxMultAdd20_1(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add1(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

static void cplxMultAdd20_2(Word16 *rY1, Word16 *rY2, Word16 *rY3, Word16 *rY4,
                            Word32 *rXR, Word32 *rXI, Word32 *iXR, Word32 *iXI,
                            const Word16 *cr, const Word16 *ci, Word16 gv, Word16 s,
                            Word16 sx, Word16 sr)
{
    Word32 rr12, ri12, ir12, ii12;

    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
    cplxMpy(rr12, ri12, ir12, ii12, rXR, rXI, iXR, iXI, cr, ci, gv, sx, sr);
    add2(rY1, rY2, rY3, rY4, rr12, ri12, ir12, ii12, s);
}

/* calcModulation

   Parameters:
     *rYR              O: pointer to real output samples (DST)
     *rYI              O: pointer to imaginary output samples (DST)
     *rXR              I: pointer to real input samples (DST)
     *rXI              I: pointer to imaginary input samples (DST)
     srYR              I: offset for update of pointer to real output samples (DST)
     srYI              I: offset for update of pointer to imaginary output samples (DST)
     srXR              I: offset for update of pointer to real input samples (DST)
     srXI              I: offset for update of pointer to imaginary input samples (DST)
     *iYR              O: pointer to real output samples (DCT)
     *iYI              O: pointer to imaginary output samples (DCT)
     *iXR              I: pointer to real input samples (DCT)
     *iXI              I: pointer to imaginary input samples (DCT)
     siYR              I: offset for update of pointer to real output samples (DCT)
     siYI              I: offset for update of pointer to imaginary output samples (DCT)
     siXR              I: offset for update of pointer to real input samples (DCT)
     siXI              I: offset for update of pointer to imaginary input samples (DCT)
     m                 I: processed cldfb bands

   Function:
     The function applies for each cldfb length a unrolled complex modulation

   Returns:
      void
*/
static void calcModulation( Word32 *rYR,
                            Word32 *rYI,
                            Word32 *rXR,
                            Word32 *rXI,
                            Word16  srYR,
                            Word16  srYI,
                            Word16  srXR,
                            Word16  srXI,
                            Word32 *iYR,
                            Word32 *iYI,
                            Word32 *iXR,
                            Word32 *iXI,
                            Word16  siYR,
                            Word16  siYI,
                            Word16  siXR,
                            Word16  siXI,
                            const Word16 *rRotVctr,
                            const Word16 *iRotVctr,
                            Word16 m
                          )
{

    SWITCH (m)
    {
    case 10:
        cplxMult10(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult10(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    case 16:
        cplxMult16(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult16(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    case 20:
        cplxMult20(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult20(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    case 32:
        cplxMult32(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult32(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    case 40:
        cplxMult40(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult40(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    case 60:
        cplxMult60(rYR, rYI, rXR, rXI, rRotVctr, iRotVctr, srYR, srYI, srXR, srXI);
        cplxMult60(iYR, iYI, iXR, iXI, rRotVctr, iRotVctr, siYR, siYI, siXR, siXI);
        BREAK;
    default:
        assert(0);
    }

}


/* calcModulationAndFolding

   Parameters:
     *rY               O: pointer to folded samples (DST + DCT)
     *rX               I: pointer to real input samples (DST)
     *iX               I: pointer to imaginary input samples (DCT)
     *rRotVctr         I: pointer to real modulation coefficients
     *iRotVctr         I: pointer to imaginary modulation coefficients
     gain              I: gain value
     scale             I: scale factor
     m                 I: processed cldfb bands
     m2                I: half of processed cldfb bands

   Function:
     The function applies for each cldfb length a unrolled complex modulation with subsequent data folding

   Returns:
      void
*/
static void calcModulationAndFolding( Word16 *rY,
                                      Word32 *rX,
                                      Word32 *iX,
                                      const Word16 *rRotVctr,
                                      const Word16 *iRotVctr,
                                      Word16 gain,
                                      Word16 scale,
                                      Word16 m,
                                      Word16 m2
                                    )
{
    Word16 *y11, *y12, *y13, *y14;
    Word16 *y21, *y22, *y23, *y24;
    Word32 *x11, *x12, *x13, *x14;
    Word32 *x21, *x22, *x23, *x24;
    const Word16 *r11, *r12, *r21, *r22;



    y11 = &rY[m+m2+1];
    y12 = &rY[m2-2];
    y13 = &rY[m+m2-2];
    y14 = &rY[m2+1];

    y21 = &rY[m+m2];
    y22 = &rY[m2-1];
    y23 = &rY[m+m2-1];
    y24 = &rY[m2];

    x11 = &rX[m-2];
    x12 = &rX[m-1];
    x13 = &iX[m-2];
    x14 = &iX[m-1];
    x21 = &rX[0];
    x22 = &rX[1];
    x23 = &iX[0];
    x24 = &iX[1];

    r11 = &rRotVctr[m2-1];
    r12 = &iRotVctr[m2-1];
    r21 = &rRotVctr[0];
    r22 = &iRotVctr[0];

    SWITCH (m)
    {
    case 10:
        cplxMultAdd10_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd10_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        BREAK;
    case 16:
        cplxMultAdd16_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd16_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        BREAK;
    case 20:
        cplxMultAdd20_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd20_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        BREAK;
    case 32:
        cplxMultAdd16_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd16_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        ptrUpdate16(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22);
        cplxMultAdd16_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd16_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        ptrUpdate16(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22);
        BREAK;
    case 60:
        cplxMultAdd20_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd20_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        ptrUpdate20(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22);
        /* no break */
    case 40:
        cplxMultAdd20_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd20_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        ptrUpdate20(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22);
        cplxMultAdd20_1(y11, y12, y13, y14, x11, x12, x13, x14, r11, r12, gain, scale, -2, -1);
        cplxMultAdd20_2(y21, y22, y23, y24, x21, x22, x23, x24, r21, r22, gain, scale,  2,  1);
        ptrUpdate20(y11,y12,y13,y14,r11,x11,x12,x13,x14,r12,y21,y22,y23,y24,r21,x21,x22,x23,x24,r22);
        BREAK;
    default:
        assert(0);
    }

}


/* cldfbAnalysisFiltering

   Parameters:
     cldfbBank          I/O: handle to analysis CLDFB filter struct
     **rAnalysis      O:   matrix holding the real part of the subband samples
     **iAnalysis      O:   matrix holding the imaginary part of the subband samples
     *scaleFactor     O:   pointer to cldfb scalefactor struct
     *timeIn          I:   pointer to time domain data
     stride           I:   stride for time domain data
     *pWorkBuffer     I:   pointer to scratch buffer, needed for buffer of size 2*cldfbbands*Word32

   Function:
      Performs complex-valued subband filtering of the time domain data of timeIn and stores the real
      part of the subband samples in rAnalysis, and the imaginary part in iAnalysis

   Returns:
      void
*/
void cldfbAnalysisFiltering( HANDLE_CLDFB_FILTER_BANK cldfbBank,
                             Word32 **rAnalysis,
                             Word32 **iAnalysis,
                             CLDFB_SCALE_FACTOR *scaleFactor,
                             const Word16 *timeIn,
                             const Word16 timeIn_e,
                             const Word16 nTimeSlots,
                             Word32 *pWorkBuffer
                           )

{
    Word16  i,k;
    Word16  L2,L3,L4,m,m2,M4;
    Word16  M0M2,M2M1,L3M1,L4M1;
    Word16  scale;
    Word16  offset;
    Word16  p_stride;
    Word16  nSamples;
    Word16  nSamplesUpd;
    Word16  stride;

    Word32  r1,r2;
    Word32  i1,i2;
    Word32 *rBuffer;
    Word32 *iBuffer;
    Word16 *pStates;

    Word16 *pStates1;
    Word16 *pStates2;
    Word16 *pStates3;
    Word16 *pStates4;
    Word16 *pStates6;
    Word16 *pStates5;

    const Word16 *rRotVctr;
    const Word16 *iRotVctr;
    const Word16 *pFilter;

    const Word16 *pFilter1;
    const Word16 *pFilter2;
    const Word16 *pFilter3;
    const Word16 *pFilter4;
    const Word16 *pFilter6;
    const Word16 *pFilter5;
    Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH];



    stride = 1;                                                                                      /* constant */
    m  = cldfbBank->no_channels;
    move16();
    L2 = shl(m,1);
    m2 = shr(m,1);
    M4 = shr(m,2);
    M4 = sub(m2,M4);

    L3 = sub(L2,m2);
    L4 = add(L2,m2);

    M0M2 = sub(0,m2);
    M2M1 = sub(m2,1);
    L3M1 = sub(L3,1);
    L4M1 = sub(L4,1);

    rBuffer  = &pWorkBuffer[0];
    iBuffer  = &pWorkBuffer[m];

    rRotVctr = cldfbBank->rRotVctr;
    iRotVctr = cldfbBank->iRotVctr;

    pStates  = cldfbBank->FilterStates;
    pStates1 = &pStates[L3M1];
    pStates2 = &pStates[L3];
    pStates3 = &pStates[m2];
    pStates4 = &pStates[M2M1];
    pStates5 = &pStates[L4M1];
    pStates6 = &pStates[M0M2];

    p_stride = CLDFB_NO_POLY;
    pFilter  = &cldfbBank->p_filter[p_stride - CLDFB_NO_POLY];
    pFilter1 = &pFilter[p_stride*L3M1];
    pFilter2 = &pFilter[p_stride*L3];
    pFilter3 = &pFilter[p_stride*m2];
    pFilter4 = &pFilter[p_stride*M2M1];
    pFilter5 = &pFilter[p_stride*L4M1];
    pFilter6 = &pFilter[p_stride*M0M2];

    nSamples = i_mult(nTimeSlots, cldfbBank->no_channels);
    nSamplesUpd = i_mult(cldfbBank->no_col, cldfbBank->no_channels);
    offset   = sub(sub(cldfbBank->p_filter_length, cldfbBank->no_channels),cldfbBank->zeros);

    /* Determine states scale */
    scale = -15;
    move16();
    k = 0;
    move16();
    FOR (i=0; i<offset; i+=cldfbBank->no_channels)
    {
        cldfbBank->FilterStates_e[k] = cldfbBank->FilterStates_e[k+cldfbBank->no_col];
        move16();
        assert((size_t)k < sizeof(cldfbBank->FilterStates_e)/sizeof(cldfbBank->FilterStates_e[0]));
        scale = s_max(scale, cldfbBank->FilterStates_e[k]);
        k = add(k,1);
    }
    FOR (i=0; i<nSamples; i+=cldfbBank->no_channels)
    {
        cldfbBank->FilterStates_e[k] = timeIn_e;
        move16();
        assert((size_t)k < sizeof(cldfbBank->FilterStates_e)/sizeof(cldfbBank->FilterStates_e[0]));
        scale = s_max(scale, cldfbBank->FilterStates_e[k]);
        k = add(k,1);
    }
    i = s_max(scale, timeIn_e);
    scale = sub(cldfbBank->FilterStates_eg, i);
    cldfbBank->FilterStates_eg = i;
    move16();

    /* if nTimeSlots==0, make sure we have a value. */
    scaleFactor->lb_scale = add(cldfbBank->anaScalefactor, add(cldfbBank->FilterStates_eg, 5));
    move16();

    /* move and scale filter states */
    FOR (i=0; i<offset; i++)
    {
        pStates[i] = shl(pStates[i+nSamplesUpd], scale);
        move16();
    }

    /* copy and scale current time signal */
    scale = sub(timeIn_e, cldfbBank->FilterStates_eg);
    FOR (i=0; i<nSamples; i++)
    {
        pStates[offset+i] = shl(*timeIn, scale);
        move16();
        timeIn = timeIn + stride;
    }

    FOR (k=0; k < nTimeSlots; k++)
    {
        FOR (i=0; i < M4; i++)
        {
            /* prototype filter */
            r1 = L_msu0(0 , pFilter1[0 - p_stride * 2 * i], pStates1[0 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[1 - p_stride * 2 * i], pStates1[1 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[2 - p_stride * 2 * i], pStates1[2 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[3 - p_stride * 2 * i], pStates1[3 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[4 - p_stride * 2 * i], pStates1[4 * L2 - 2 * i]);

            r2 = L_msu0(0 , pFilter2[0 + p_stride * 2 * i], pStates2[0 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter2[1 + p_stride * 2 * i], pStates2[1 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter2[2 + p_stride * 2 * i], pStates2[2 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter2[3 + p_stride * 2 * i], pStates2[3 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter2[4 + p_stride * 2 * i], pStates2[4 * L2 + 2 * i]);

            i1 = L_msu0(0 , pFilter3[0 + p_stride * 2 * i], pStates3[0 * L2 + 2 * i]);
            i1 = L_msu0(i1, pFilter3[1 + p_stride * 2 * i], pStates3[1 * L2 + 2 * i]);
            i1 = L_msu0(i1, pFilter3[2 + p_stride * 2 * i], pStates3[2 * L2 + 2 * i]);
            i1 = L_msu0(i1, pFilter3[3 + p_stride * 2 * i], pStates3[3 * L2 + 2 * i]);
            i1 = L_msu0(i1, pFilter3[4 + p_stride * 2 * i], pStates3[4 * L2 + 2 * i]);

            i2 = L_msu0(0 , pFilter4[0 - p_stride * 2 * i], pStates4[0 * L2 - 2 * i]);
            i2 = L_msu0(i2, pFilter4[1 - p_stride * 2 * i], pStates4[1 * L2 - 2 * i]);
            i2 = L_msu0(i2, pFilter4[2 - p_stride * 2 * i], pStates4[2 * L2 - 2 * i]);
            i2 = L_msu0(i2, pFilter4[3 - p_stride * 2 * i], pStates4[3 * L2 - 2 * i]);
            i2 = L_msu0(i2, pFilter4[4 - p_stride * 2 * i], pStates4[4 * L2 - 2 * i]);

            /* folding */
            rBuffer[2*i]   = L_sub(r1,r2);
            move32();
            rBuffer[2*i+1] = L_negate(L_add(i1,i2));
            move32();

            /* folding */
            iBuffer[2*i]   = L_add(r1,r2);
            move32();
            iBuffer[2*i+1] = L_sub(i1,i2);
            move32();
        }

        FOR (i=M4; i < m2; i++)
        {
            /* prototype filter */
            r1 = L_msu0(0 , pFilter1[0 - p_stride * 2 * i], pStates1[0 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[1 - p_stride * 2 * i], pStates1[1 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[2 - p_stride * 2 * i], pStates1[2 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[3 - p_stride * 2 * i], pStates1[3 * L2 - 2 * i]);
            r1 = L_msu0(r1, pFilter1[4 - p_stride * 2 * i], pStates1[4 * L2 - 2 * i]);

            r2 = L_msu0(0 , pFilter6[0 + p_stride * 2 * i], pStates6[0 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter6[1 + p_stride * 2 * i], pStates6[1 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter6[2 + p_stride * 2 * i], pStates6[2 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter6[3 + p_stride * 2 * i], pStates6[3 * L2 + 2 * i]);
            r2 = L_msu0(r2, pFilter6[4 + p_stride * 2 * i], pStates6[4 * L2 + 2 * i]);

            i1 = L_msu0(0 , pFilter5[0 - p_stride * 2 * i], pStates5[0 * L2 - 2 * i]);
            i1 = L_msu0(i1, pFilter5[1 - p_stride * 2 * i], pStates5[1 * L2 - 2 * i]);
            i1 = L_msu0(i1, pFilter5[2 - p_stride * 2 * i], pStates5[2 * L2 - 2 * i]);
            i1 = L_msu0(i1, pFilter5[3 - p_stride * 2 * i], pStates5[3 * L2 - 2 * i]);
            i1 = L_msu0(i1, pFilter5[4 - p_stride * 2 * i], pStates5[4 * L2 - 2 * i]);

            i2 = L_msu0(0 , pFilter3[0 + p_stride * 2 * i], pStates3[0 * L2 + 2 * i]);
            i2 = L_msu0(i2, pFilter3[1 + p_stride * 2 * i], pStates3[1 * L2 + 2 * i]);
            i2 = L_msu0(i2, pFilter3[2 + p_stride * 2 * i], pStates3[2 * L2 + 2 * i]);
            i2 = L_msu0(i2, pFilter3[3 + p_stride * 2 * i], pStates3[3 * L2 + 2 * i]);
            i2 = L_msu0(i2, pFilter3[4 + p_stride * 2 * i], pStates3[4 * L2 + 2 * i]);

            /* folding */
            rBuffer[2*i]   = L_add(r1,r2);
            move32();
            rBuffer[2*i+1] = L_sub(i1,i2);
            move32();

            /* folding */
            iBuffer[2*i]   = L_sub(r1,r2);
            move32();
            iBuffer[2*i+1] = L_add(i1,i2);
            move32();
        }

        /* pre modulation of DST IV and DCT IV */
        calcModulation(&rBuffer[0], &rBuffer[1], &rBuffer[0], &rBuffer[1], 2, 2, 2, 2,
                       &iBuffer[0], &iBuffer[1], &iBuffer[0], &iBuffer[1], 2, 2, 2, 2,
                       rRotVctr, iRotVctr, m);

        /* FFT of DST IV */
        scale = 0;
        move16();
        BASOP_cfft(&rBuffer[0], &rBuffer[1], m2, 2, &scale, workBuffer);

        /* store analysis scalefactor for cldfb */
        scaleFactor->lb_scale = add(cldfbBank->anaScalefactor, add(cldfbBank->FilterStates_eg, scale));
        move16();

        /* FFT of DCT IV */
        BASOP_cfft(&iBuffer[0], &iBuffer[1], m2, 2, &scale, workBuffer);

        /* post modulation of DST IV and DCT IV */
        calcModulation(&rAnalysis[k][m-1], &rAnalysis[k][0], &rBuffer[0], &rBuffer[1],-2, 2, 2, 2,
                       &iAnalysis[k][0], &iAnalysis[k][m-1], &iBuffer[0], &iBuffer[1], 2,-2, 2, 2,
                       rRotVctr, iRotVctr, m);


        /* update states pointer */
        pStates1 = &pStates1[cldfbBank->no_channels];
        pStates2 = &pStates2[cldfbBank->no_channels];
        pStates3 = &pStates3[cldfbBank->no_channels];
        pStates5 = &pStates5[cldfbBank->no_channels];
        pStates4 = &pStates4[cldfbBank->no_channels];
        pStates6 = &pStates6[cldfbBank->no_channels];
    }

}


/* cldfbSynthesisFiltering

   Parameters:
     cldfbBank          I/O: handle to analysis CLDFB filter struct
     **rAnalysis      I:   matrix holding the real part of the subband samples
     **iAnalysis      I:   matrix holding the imaginary part of the subband samples
     *scaleFactor     I:   pointer to cldfb scalefactor struct
     ov_len           I:   determines overlapping area in time slots (obsolete)
     *timeOut         O:   pointer to time domain data
     stride           I:   stride for time domain data
     *pWorkBuffer     I:   pointer to scratch buffer, needed for buffer of size 2*cldfbbands*Word32 + 2*cldfbbands*Word16

   Function:
      Performs inverse complex-valued subband filtering of the subband samples in rAnalysis and iAnalysis
      and stores the time domain data in timeOut

   Returns:
      void
*/
void
cldfbSynthesisFiltering( HANDLE_CLDFB_FILTER_BANK cldfbBank,
                         Word32 **rAnalysis,
                         Word32 **iAnalysis,
                         const CLDFB_SCALE_FACTOR *scaleFactor,
                         Word16 *timeOut,
                         const Word16 timeOut_e,
                         const Word16 nTimeSlots,
                         Word32 *pWorkBuffer
                       )
{
    Word16  i;
    Word16  k;
    Word16  L2;
    Word16  m;
    Word16  m2;
    Word16  Lz;
    Word16  Mz;
    Word32  acc;
    Word16  offset1;
    Word16  offset2;
    Word16  channels0;
    Word16  channels1;
    Word16  channels2;
    Word16  channels3;
    Word16  channels4;
    Word16  statesSizeM1;
    Word16  statesSizeM2;
    Word16  stride;

    Word16  scale, scaleMod;
    Word16  outScale;
    Word16  scaleLB;
    Word16  scaleHB;

    Word32 *rAnalysisS;
    Word32 *iAnalysisS;

    Word32 *rBuffer;
    Word32 *iBuffer;
    Word16 *nBuffer;

    Word16 *pStates;
    Word16 *pStatesI;
    Word16 *pStatesR;

    const Word16 *pFilterS;
    const Word16 *pFilterM;

    const Word16 *rRotVctr;
    const Word16 *iRotVctr;
    Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH];



    m   = cldfbBank->no_channels;
    move16();
    L2  = shl(m,1);
    m2  = shr(m,1);
    Lz  = s_min(cldfbBank->lsb, sub(m,cldfbBank->bandsToZero));
    Mz  = s_min(cldfbBank->usb, sub(m,cldfbBank->bandsToZero));
    stride = 1;                                                                                     /* constant */

    channels0 = sub(m,cldfbBank->zeros);
    channels1 = sub(m,1);
    channels2 = shl(m,1);
    channels3 = add(m,channels2);
    channels4 = shl(channels2,1);

    statesSizeM1 = sub(shl(cldfbBank->p_filter_length,1),m);
    statesSizeM2 = sub(statesSizeM1,m);

    offset1 = sub(channels1,cldfbBank->zeros);
    offset2 = add(offset1,cldfbBank->no_channels);

    rBuffer = &pWorkBuffer[0];
    iBuffer = &pWorkBuffer[m];
    nBuffer = (Word16*)(&pWorkBuffer[L2]);

    rAnalysisS = &pWorkBuffer[3*m];
    iAnalysisS = &pWorkBuffer[4*m];

    rRotVctr = cldfbBank->rRotVctr;
    iRotVctr = cldfbBank->iRotVctr;

    scale = scaleFactor->lb_scale;
    move16();
    if ( sub(Lz, Mz) != 0 )
    {
        scale = s_max(scale, scaleFactor->hb_scale);
    }
    scaleLB = limitScale32(sub(scale, scaleFactor->lb_scale));
    scaleHB = limitScale32(sub(scale, scaleFactor->hb_scale));

    outScale = cldfbBank->synFilterHeadroom;
    move16();

    scaleMod = sub(add(scale, cldfbBank->outScalefactor), outScale);

    /* Increase CLDFB synthesis states for low level signals */
    IF ( sub(scale, 8) < 0)
    {
        scaleMod = add(scaleMod, 2);
        outScale = sub(outScale, 2);
    }
    scaleMod = sub(scaleMod, timeOut_e);
    scale = add(outScale, timeOut_e);
    IF ( sub(scale, cldfbBank->FilterStates_eg) != 0)
    {
        Scale_sig(cldfbBank->FilterStates, statesSizeM2, sub(cldfbBank->FilterStates_eg, scale));
        cldfbBank->FilterStates_eg = scale;
        move16();
    }

    FOR (k=0; k < nTimeSlots; k++)
    {
        {
            FOR (i=0; i < Lz; i+=2)
            {
                rAnalysisS[i] = L_shr(rAnalysis[k][i],scaleLB);
                move32();
                iAnalysisS[i] = L_negate(L_shr(iAnalysis[k][i],scaleLB));
                move32();
            }

            IF ( sub(i,Mz) < 0 )
            {
                FOR ( ; i < Mz; i+=2)
                {
                    rAnalysisS[i] = L_shr(rAnalysis[k][i],scaleHB);
                    move32();
                    iAnalysisS[i] = L_negate(L_shr(iAnalysis[k][i],scaleHB));
                    move32();
                }
            }

            IF ( sub(i,m) < 0 )
            {
                FOR ( ; i < m; i+=2)
                {
                    rAnalysisS[i] = L_deposit_l(0);
                    iAnalysisS[i] = L_deposit_l(0);
                }
            }

            FOR (i=1; i < Lz; i+=2)
            {
                rAnalysisS[i] = L_shr(rAnalysis[k][i],scaleLB);
                move32();
                iAnalysisS[i] = L_shr(iAnalysis[k][i],scaleLB);
                move32();
            }

            IF ( sub(i,Mz) < 0 )
            {
                FOR ( ; i < Mz; i+=2)
                {
                    rAnalysisS[i] = L_shr(rAnalysis[k][i],scaleHB);
                    move32();
                    iAnalysisS[i] = L_shr(iAnalysis[k][i],scaleHB);
                    move32();
                }
            }

            IF ( sub(i,m) < 0 )
            {
                FOR ( ; i < m; i+=2)
                {
                    rAnalysisS[i] = L_deposit_l(0);
                    iAnalysisS[i] = L_deposit_l(0);
                }
            }
        }

        /* pre modulation */
        calcModulation(&rBuffer[0], &rBuffer[1], &rAnalysisS[0], &rAnalysisS[m-1], 2, 2, 2,-2,
                       &iBuffer[0], &iBuffer[1], &iAnalysisS[0], &iAnalysisS[m-1], 2, 2, 2,-2,
                       rRotVctr, iRotVctr, m);


        /* FFT of DST IV */
        scale = 0;
        move16();
        BASOP_cfft(&rBuffer[0], &rBuffer[1], m2, 2, &scale, workBuffer);

        /* FFT of DCT IV */
        scale = scaleMod;
        move16();
        BASOP_cfft(&iBuffer[0], &iBuffer[1], m2, 2, &scale, workBuffer);

        /* post modulation and folding */
        calcModulationAndFolding(nBuffer, rBuffer, iBuffer, rRotVctr, iRotVctr, cldfbBank->synGain, scale, m, m2);

        /* prototype filter */
        pStates  = &cldfbBank->FilterStates[k*L2];
        pFilterS = &cldfbBank->p_filter[0];
        pFilterM = &cldfbBank->p_filter[shr(cldfbBank->p_filter_length,1)];

        FOR (i=0; i < channels0; i++)
        {
            pStatesI = &pStates[i];
            pStatesR = &pStates[i+channels3];

            acc = L_mult(    *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, nBuffer[channels1-i], *pFilterM++);

            BASOP_SATURATE_WARNING_OFF
            timeOut[(offset1-i)*stride] = round_fx(L_shl(acc,outScale));
            BASOP_SATURATE_WARNING_ON
        }

        FOR ( ; i<cldfbBank->no_channels; i++)
        {
            pStatesI = &pStates[i+channels2];
            pStatesR = &pStates[i+channels2+channels3];

            acc = L_mult(    *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);
            pStatesR += channels4;
            pStatesI += channels4;

            acc = L_mac(acc, *pStatesI, *pFilterS++);
            acc = L_mac(acc, *pStatesR, *pFilterM++);

            acc = L_mac(acc, nBuffer[channels1+m-i], *pFilterS++);
            pFilterM++;

            BASOP_SATURATE_WARNING_OFF
            timeOut[(offset2-i)*stride] = round_fx(L_shl(acc,outScale));
            BASOP_SATURATE_WARNING_ON
        }

        FOR (i=0; i<cldfbBank->no_channels; i++)
        {
            pStates[statesSizeM1+i] = nBuffer[channels1-i];
            move16();
            pStates[statesSizeM2+i] = nBuffer[channels1+m-i];
            move16();
        }

        timeOut = &timeOut[m*stride];
    }

    /* move filter states */
    Copy(&cldfbBank->FilterStates[i_mult(nTimeSlots,L2)], cldfbBank->FilterStates, statesSizeM2);
    set16_fx(&cldfbBank->FilterStates[statesSizeM2],0,L2);

}


/*-------------------------------------------------------------------*
 * configureClfdb()
 *
 * configures a CLDFB handle
 *--------------------------------------------------------------------*/

void configureCldfb ( HANDLE_CLDFB_FILTER_BANK h_cldfb,         /*!< Returns handle */
                      const Word16 no_channels,                              /*!< Number of channels (bands) */
                      const Word16 frameSize                                 /*!< FrameSize */
                    )
{

    h_cldfb->no_channels = no_channels;
    move16();
    assert(h_cldfb->no_channels >= 10);
    h_cldfb->no_col = div_l(frameSize,shr(h_cldfb->no_channels,1));

    /* was cldfbInitFilterBank()*/
    h_cldfb->anaScalefactor = 0;
    move16();
    h_cldfb->synScalefactor = 0;
    move16();
    h_cldfb->bandsToZero = 0;
    move16();
    h_cldfb->filtermode = 0;
    move16();
    h_cldfb->memory = 0;
    move16();
    h_cldfb->memory_length = 0;
    move16();

    h_cldfb->p_filter_length = i_mult(10,h_cldfb->no_channels);
    move16();

    h_cldfb->flags = s_or(h_cldfb->flags,CLDFB_FLAG_2_5MS_SETUP);
    h_cldfb->filterScale = CLDFB_CLDFB80_PFT_SCALE;
    move16();

    h_cldfb->zeros = 0;
    move16();
    h_cldfb->synFilterHeadroom = SYN_FILTER_HEADROOM_2_5MS;
    move16();

    cldfb_init_proto_and_twiddles (h_cldfb);

    h_cldfb->lsb = no_channels;
    move16();
    h_cldfb->usb = s_min(no_channels, h_cldfb->no_channels); /* Does this make any sense? in the previous implemenatation lsb, usb and no_channels are all maxCldfbBands */  move16();

    h_cldfb->FilterStates = (void*)h_cldfb->FilterStates;
    h_cldfb->outScalefactor = h_cldfb->synScalefactor;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * openClfdb()
 *
 * open and configures a CLDFB handle
 *--------------------------------------------------------------------*/
void openCldfb ( HANDLE_CLDFB_FILTER_BANK *h_cldfb,   /*!< Returns handle */
                 const Word16 type,                                  /*!< analysis or synthesis */
                 const Word16 maxCldfbBands,                         /*!< number of cldfb bands */
                 const Word16 frameSize                              /*!< FrameSize */
               )
{
    HANDLE_CLDFB_FILTER_BANK hs;

    hs = (HANDLE_CLDFB_FILTER_BANK) calloc(1, sizeof (struct CLDFB_FILTER_BANK));


    hs->type = type;
    move16();

    IF (type == CLDFB_ANALYSIS)
    {
        hs->FilterStates = (Word16 *) calloc(STATE_BUFFER_SIZE*maxCldfbBands, sizeof (Word16));
    }
    ELSE
    {
        hs->FilterStates = (Word16 *) calloc( 2 * STATE_BUFFER_SIZE * maxCldfbBands, sizeof (Word16));
    }
    hs->flags &= ~CLDFB_FLAG_KEEP_STATES;

    configureCldfb (hs, maxCldfbBands, frameSize );

    hs->memory = NULL;
    hs->memory_length = 0;
    move16();

    IF( hs->type == CLDFB_ANALYSIS)
    {
        test();
        IF ( (s_and(hs->flags,CLDFB_FLAG_KEEP_STATES) == 0) && (hs->FilterStates != 0) )
        {
            set16_fx(hs->FilterStates, 0, i_mult(STATE_BUFFER_SIZE,hs->no_channels));
            set16_fx(hs->FilterStates_e, 0, sizeof(hs->FilterStates_e)/sizeof(hs->FilterStates_e[0]));

            hs->FilterStates_eg = 0;
            move16();
        }
    }
    ELSE IF (hs->type == CLDFB_SYNTHESIS )
    {
        IF ( hs->FilterStates != 0 )
        {
            IF ( s_and(hs->flags,CLDFB_FLAG_KEEP_STATES) == 0 )
            {
                set16_fx(hs->FilterStates, 0, i_mult( shl(STATE_BUFFER_SIZE,1), hs->no_channels));
            }
        }
        hs->FilterStates_eg = 0;
        move16();
    }

    if (h_cldfb != NULL)
    {
        *h_cldfb = hs;
    }

    return;
}


/*-------------------------------------------------------------------*
* resampleCldfb()
*
* Change sample rate of filter bank
*--------------------------------------------------------------------*/
void resampleCldfb (HANDLE_CLDFB_FILTER_BANK hs,
                    const Word16 newCldfbBands,
                    const Word16 frameSize,
                    const Word8 firstFrame
                   )

{
    Word16 timeOffset;
    Word16 timeOffsetOld;
    Word16 noChannelsOld;

    noChannelsOld = hs->no_channels;
    move16();
    timeOffsetOld = sub(sub(hs->p_filter_length,hs->no_channels),hs->zeros);

    /* change all CLDFB bank parameters that depend on the no of channels */
    hs->flags = s_or(hs->flags,CLDFB_FLAG_KEEP_STATES);
    move16();

    /* new settings */
    configureCldfb (hs, newCldfbBands, frameSize);

    /* resample cldfb state buffer */
    timeOffset = sub(sub(hs->p_filter_length,hs->no_channels),hs->zeros);

    IF( firstFrame == 0 )
    {
        /*low complexity-resampling only stored previous samples that are needed for next frame modulation */
        lerp(hs->FilterStates+(noChannelsOld*hs->no_col), hs->FilterStates+(noChannelsOld*hs->no_col), timeOffset, timeOffsetOld);
        Copy(hs->FilterStates+(noChannelsOld*hs->no_col), hs->FilterStates+frameSize, timeOffset);
    }

    return;
}

/*
   AnalysisPostSpectrumScaling_Fx

    Parameters:
    cldfbBank          I: CLDFB handle
       **rSubband32    I: matrix holding real part of the CLDFB subsamples
    **iSubband32    I: matrix holding imaginary part of the CLDFB subsamples
       **rSubband16    O: matrix holding real part of the CLDFB subsamples
    **iSubband16    O: matrix holding imaginary part of the CLDFB subsamples
     *cldfbScale        O: cldfb lowband scalefactor

    Function:
       performs dynamic spectrum scaling for all subband

    Returns:
       headroom
*/
Word16
AnalysisPostSpectrumScaling_Fx (HANDLE_CLDFB_FILTER_BANK cldfbBank,  /*!< Handle of cldfbBank  */
                                Word32 **rSubband32,             /*!< Real bands */
                                Word32 **iSubband32,             /*!< Imaginary bands */
                                Word16 **rSubband16,             /*!< Real bands */
                                Word16 **iSubband16,             /*!< Imaginary bands */
                                Word16  *cldfbScale              /*!< CLDFB lowband scalefactor */
                               )
{
    Word16 i;
    Word16 j;
    Word16 headRoom;



    headRoom = BASOP_util_norm_l_dim2_cplx (
                   (const Word32 * const*) rSubband32,
                   (const Word32 * const*) iSubband32,
                   0,
                   cldfbBank->no_channels,
                   0,
                   cldfbBank->no_col
               );

    FOR (i=0; i < cldfbBank->no_col; i++)
    {
        FOR (j=0; j < cldfbBank->no_channels; j++)
        {
            rSubband16[i][j] = round_fx(L_shl(rSubband32[i][j], headRoom));
            iSubband16[i][j] = round_fx(L_shl(iSubband32[i][j], headRoom));
        }
    }

    *cldfbScale = add(*cldfbScale,headRoom);
    move16();


    return headRoom;
}


/*-------------------------------------------------------------------*
* analysisCLDFBEncoder()
*
* Encoder CLDFB analysis + energy stage
*--------------------------------------------------------------------*/

void analysisCldfbEncoder_fx(
    Encoder_State_fx *st_fx,              /* i/o: encoder state structure                    */
    const Word16 *timeIn,
    Word32 realBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word32 imagBuffer[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word16 realBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word16 imagBuffer16[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX],
    Word32 enerBuffSum[CLDFB_NO_CHANNELS_MAX],
    Word16 *enerBuffSum_exp,
    CLDFB_SCALE_FACTOR * scale
)
{
    Word16 i;
    CLDFB_SCALE_FACTOR enerScale;
    Word32 *ppBuf_Real[CLDFB_NO_COL_MAX];
    Word32 *ppBuf_Imag[CLDFB_NO_COL_MAX];
    Word16 *ppBuf_Real16[CLDFB_NO_COL_MAX];
    Word16 *ppBuf_Imag16[CLDFB_NO_COL_MAX];
    Word32 workBuffer[256];
    Word16 num_slots = 1;

    FOR (i=0; i<CLDFB_NO_COL_MAX; i++)
    {
        ppBuf_Real[i] = &realBuffer[i][0];
        ppBuf_Imag[i] = &imagBuffer[i][0];
        ppBuf_Real16[i] = &realBuffer16[i][0];
        ppBuf_Imag16[i] = &imagBuffer16[i][0];
    }

    /* perform analysis */
    cldfbAnalysisFiltering (
        st_fx->cldfbAna_Fx,
        ppBuf_Real,
        ppBuf_Imag,
        scale,
        timeIn,
        0,
        CLDFB_NO_COL_MAX,
        workBuffer
    );

    enerScale.lb_scale = negate(scale->lb_scale);
    enerScale.lb_scale16 = negate(scale->lb_scale);

    /* get 16bit respresentation */
    AnalysisPostSpectrumScaling_Fx (
        st_fx->cldfbAna_Fx,
        ppBuf_Real,
        ppBuf_Imag,
        ppBuf_Real16,
        ppBuf_Imag16,
        &enerScale.lb_scale16
    );

    /* get the energy */
    GetEnergyCldfb( &st_fx->energyCoreLookahead_Fx,
                    &st_fx->sf_energyCoreLookahead_Fx,
                    num_slots,
                    ppBuf_Real16,
                    ppBuf_Imag16,
                    enerScale.lb_scale16,
                    st_fx->cldfbAna_Fx->no_channels,
                    st_fx->cldfbAna_Fx->no_col,
                    &st_fx->currEnergyHF_fx,
                    &st_fx->currEnergyHF_e_fx,
                    enerBuffSum,
                    enerBuffSum_exp,
                    &st_fx->tecEnc
                  );

    return;
}

void
GetEnergyCldfb( Word32 *energyLookahead,    /*!< o: Q(*sf_energyLookahead) |   pointer to the result in the core look-ahead slot */
                Word16 *sf_energyLookahead, /*!< o:         pointer to the scalefactor of the result in the core look-ahead slot  */
                const Word16 numLookahead,  /*!< i: Q0      the number of look-ahead time-slots */
                Word16 **realValues,        /*!< i: Q(sf_Values) |   the real part of the CLDFB subsamples */
                Word16 **imagValues,        /*!< i: Q(sf_Values) |   the imaginary part of the CLDFB subsamples */
                Word16   sf_Values,         /*!< i:         scalefactor of the CLDFB subcamples - apply as a negated Exponent */
                Word16   numberBands,       /*!< i: Q0  |   number of CLDFB bands */
                Word16   numberCols,        /*!< i: Q0  |   number of CLDFB subsamples */
                Word32  *energyHF,          /*!< o: Q31 |   pointer to HF energy */
                Word16  *energyHF_Exp,      /*!< o:         pointer to exponent of HF energy */
                Word32 *energyValuesSum,    /*!< o: Q(2*sf_Values-4) |   pointer to sum array of energy values, not initialized*/
                Word16 *energyValuesSum_Exp,/*!< o:         pointer to exponents of energyValuesSum, not initialized */
                HANDLE_TEC_ENC_FX hTecEnc
              )
{
    Word16 j;
    Word16 k;
    Word16 s;
    Word16 sm;
    Word32 nrg;
    Word16 numberColsL;
    Word16 numberBandsM;
    Word16 numberBandsM20;
    Word32 energyValues[CLDFB_NO_COL_MAX][CLDFB_NO_CHANNELS_MAX];
    Word16 energyValuesSumE[CLDFB_NO_CHANNELS_MAX];
    Word16 freqTable[2] = {20, 40};


    FOR (k=0; k<numberCols; k++)
    {
        FOR (j=0; j<numberBands; j++)
        {
            nrg = L_mult0(realValues[k][j], realValues[k][j]);
            nrg = L_mac0(nrg,imagValues[k][j], imagValues[k][j]);

            energyValues[k][j] = nrg;
            move32();
        }
    }

    IF(sub(numberBands, freqTable[1]) >= 0)
    {
        Word32 *tempEnergyValuesArry[CLDFB_NO_COL_MAX];
        Word16 ScaleX2;
        assert(numberCols == CLDFB_NO_COL_MAX);
        FOR (j=0; j<numberCols; j++)
        {
            tempEnergyValuesArry[j] = &energyValues[j][0];
        }

        ScaleX2 = shl(sf_Values,1);
        calcHiEnvLoBuff_Fix(
            numberCols,
            freqTable,
            1,
            tempEnergyValuesArry,
            hTecEnc->loBuffer,
            hTecEnc->hiTempEnv,
            ScaleX2
        );
    }

    FOR (j=0; j< numberBands; j++)
    {
        energyValuesSum[j] = L_deposit_l(0);
        energyValuesSumE[j] = 31;
        move16();
        FOR (k=0; k<CLDFB_NO_COL_MAX; k++)
        {
            nrg = L_shr_r( energyValues[k][j], sub( energyValuesSumE[j], 31 ) );
            IF ( L_sub( maxWord32, nrg ) < energyValuesSum[j] )
            {
                energyValuesSumE[j] = add( energyValuesSumE[j], 1 );
                move16();
                energyValuesSum[j] = L_shr_r( energyValuesSum[j], 1 );
                move32();
                nrg = L_shr_r( energyValues[k][j], 1 );
            }
            energyValuesSum[j] = L_add( energyValuesSum[j], nrg );
            move32();
        }
        test();
        IF ( j == 0 || sub(energyValuesSumE[j],*energyValuesSum_Exp) > 0 )
        {
            *energyValuesSum_Exp = energyValuesSumE[j];
        }
    }
    FOR (j=0; j< numberBands; j++)
    {
        energyValuesSum[j] = L_shr_r( energyValuesSum[j], sub( *energyValuesSum_Exp, energyValuesSumE[j] ) );
        move32();
    }
    *energyValuesSum_Exp = sub( *energyValuesSum_Exp, shl( sf_Values, 1 ) );
    move16();

    IF ( sub(numberBands,20) > 0 )
    {
        numberBandsM  = s_min(numberBands, 40);
        numberBandsM20 = sub(numberBandsM, 20);

        numberColsL = sub(numberCols, numLookahead);

        /* sum up CLDFB energy above 8 kHz */
        s = BASOP_util_norm_s_bands2shift(i_mult(numberColsL, numberBandsM20));
        s = sub(s,4);
        nrg = L_deposit_l(0);

        FOR (k=0; k < numberColsL; k++)
        {
            FOR (j=20; j < numberBandsM; j++)
            {
                nrg = L_add(nrg, L_shr(energyValues[k][j], s));
            }
        }

        s  = sub(sub(shl(sf_Values, 1), 1), s);
        sm = sub(s_min(s, *sf_energyLookahead), 1);

        *energyHF = L_add(L_shr(nrg, limitScale32(sub(s, sm))),
                          L_shr(*energyLookahead,sub(*sf_energyLookahead, sm)));
        move32();

        *energyHF_Exp = negate(sm);
        move16();

        /* process look-ahead region */
        s = BASOP_util_norm_s_bands2shift(i_mult(numLookahead, numberBandsM20));
        s = sub(s, 2);
        nrg = L_deposit_l(0);

        FOR (k = numberColsL; k < numberCols; k++)
        {
            FOR (j=20; j < numberBandsM; j++)
            {
                nrg = L_add(nrg, L_shr(energyValues[k][j], s));
            }
        }

        s = sub(shl(sf_Values, 1), s);
        sm = sub(s_min(s, 44), 1);
        BASOP_SATURATE_WARNING_OFF
        /* nrg + 6.1e-5f => value 0x40000000, scale 44 */
        *energyLookahead = L_add(L_shr(nrg, sub(s, sm)),
                                 L_shr(0x40000000, sub(44, sm)));
        move32();
        BASOP_SATURATE_WARNING_ON
        *sf_energyLookahead = sm;
        move16();

        return;
    }



    *energyHF = 0x40000000;
    move32();
    *energyHF_Exp = 17;
    move16();


}


Word16
CLDFB_getNumChannels(Word32 sampleRate)
{

    Word16 nChannels = 0;


    SWITCH (sampleRate)
    {
    case 48000:
        move16();
        nChannels = 60;
        BREAK;
    case 32000:
        move16();
        nChannels = 40;
        BREAK;
    case 25600:
        move16();
        nChannels = 32;
        BREAK;
    case 16000:
        move16();
        nChannels = 20;
        BREAK;
    case 12800:
        move16();
        nChannels = 16;
        BREAK;
    case 8000:
        move16();
        nChannels = 10;
        BREAK;
    default:
        BREAK;
    }

    return (nChannels);
}

/*-------------------------------------------------------------------*
* cldfb_get_memory_length()
*
* Return length of filter state for recovery
*--------------------------------------------------------------------*/
static Word16
cldfb_get_memory_length (HANDLE_CLDFB_FILTER_BANK hs)
{
    IF (sub(hs->type,CLDFB_ANALYSIS)==0)
    {
        return (i_mult(hs->no_channels,STATE_BUFFER_SIZE));
    }
    ELSE
    {
        return (i_mult(hs->no_channels,(9*2)) );
    }
}

/*-------------------------------------------------------------------*
* GetEnergyCldfb()
*
* Remove handle
*--------------------------------------------------------------------*/
void
deleteCldfb (HANDLE_CLDFB_FILTER_BANK * h_cldfb)              /* i: cldfb handle */
{
    IF ( *h_cldfb != NULL )
    {
        IF ( (*h_cldfb)->FilterStates != NULL )
        {
            free((*h_cldfb)->FilterStates);
        }
        free(*h_cldfb);
    }
    *h_cldfb = NULL;
}


/*-------------------------------------------------------------------*
* cldfb_init_proto_and_twiddles()
*
* Initializes rom pointer
*--------------------------------------------------------------------*/
static void
cldfb_init_proto_and_twiddles(HANDLE_CLDFB_FILTER_BANK hs)     /* i: cldfb handle */
{

    /*find appropriate set of rotVecs*/
    SWITCH(hs->no_channels)
    {
    case 10:

        hs->rRotVctr = rRotVectr_10;
        hs->iRotVctr = iRotVectr_10;
        hs->synGain  = cldfb_synGain[0];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[0];
        hs->scale = cldfb_scale_2_5ms[0];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[0],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[0],hs->filterScale);
            move16();
        }
        break;

    case 16:
        hs->rRotVctr = rRotVectr_16;
        hs->iRotVctr = iRotVectr_16;
        hs->synGain  = cldfb_synGain[1];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[1];
        hs->scale = cldfb_scale_2_5ms[1];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[1],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[1],hs->filterScale);
            move16();
        }
        break;

    case 20:
        hs->rRotVctr = rRotVectr_20;
        hs->iRotVctr = iRotVectr_20;
        hs->synGain  = cldfb_synGain[2];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[2];
        hs->scale = cldfb_scale_2_5ms[2];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[2],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[2],hs->filterScale);
            move16();
        }
        break;

    case 32:
        hs->rRotVctr = rRotVectr_32;
        hs->iRotVctr = iRotVectr_32;
        hs->synGain  = cldfb_synGain[3];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[3];
        hs->scale = cldfb_scale_2_5ms[3];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[3],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[3],hs->filterScale);
            move16();
        }
        break;

    case 40:
        hs->rRotVctr = rRotVectr_40;
        hs->iRotVctr = iRotVectr_40;
        hs->synGain  = cldfb_synGain[4];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[4];
        hs->scale = cldfb_scale_2_5ms[4];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[4],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[4],hs->filterScale);
            move16();
        }
        break;

    case 60:
        hs->rRotVctr = rRotVectr_60;
        hs->iRotVctr = iRotVectr_60;
        hs->synGain  = cldfb_synGain[5];
        move16();
        hs->p_filter = cldfb_protoFilter_2_5ms[5];
        hs->scale = cldfb_scale_2_5ms[5];
        move16();
        IF (hs->type == CLDFB_SYNTHESIS )
        {
            hs->synScalefactor = add(cldfb_synScale[5],hs->filterScale);
            move16();
        }
        ELSE
        {
            hs->anaScalefactor = add(cldfb_anaScale[5],hs->filterScale);
            move16();
        }
        break;

    }
}


#define CLDFB_MEM_EXPONENTS (CLDFB_NO_COL_MAX+9)

/*-------------------------------------------------------------------*
* cldfb_save_memory()
*
* Save the memory of filter; to be restored with cldfb_restore_memory()
*--------------------------------------------------------------------*/
void
cldfb_save_memory (HANDLE_CLDFB_FILTER_BANK hs)   /* i: cldfb handle */
{

    hs->memory_length = cldfb_get_memory_length(hs);
    hs->memory = (Word16 *) calloc( hs->memory_length + CLDFB_MEM_EXPONENTS + 1, sizeof (Word16));


    /* save the memory */
    Copy (hs->FilterStates, hs->memory, hs->memory_length);
    Copy (hs->FilterStates_e, hs->memory+hs->memory_length, CLDFB_MEM_EXPONENTS);
    hs->memory[hs->memory_length+CLDFB_MEM_EXPONENTS] = hs->FilterStates_eg;
    move16();

    return;
}


/*-------------------------------------------------------------------*
* cldfb_restore_memory()
*
* Restores the memory of filter; memory to be save by cldfb_save_memory()
*--------------------------------------------------------------------*/
void
cldfb_restore_memory (HANDLE_CLDFB_FILTER_BANK hs)   /* i/o: cldfb handle */

{
    Word16 size;


    size = cldfb_get_memory_length(hs);

    /* read the memory */
    Copy (hs->memory, hs->FilterStates, hs->memory_length);
    Copy (hs->memory+hs->memory_length, hs->FilterStates_e, CLDFB_MEM_EXPONENTS);
    hs->FilterStates_eg = hs->memory[hs->memory_length+CLDFB_MEM_EXPONENTS];
    move16();

    /* adjust sample rate if it was changed in the meanwhile */
    IF (sub (hs->memory_length,size) != 0)
    {
        lerp(hs->FilterStates, hs->FilterStates, size, hs->memory_length);
    }

    hs->memory_length = 0;
    free(hs->memory);
    hs->memory = NULL;

    return;
}

/*-------------------------------------------------------------------*
* cldfb_reset_memory()
*
* Resets the memory of filter.
*--------------------------------------------------------------------*/
void
cldfb_reset_memory (HANDLE_CLDFB_FILTER_BANK hs)     /* i/o: cldfb handle */
{
    Word16 length;

    length = cldfb_get_memory_length(hs);
    /* erase the memory */
    set16_fx (hs->FilterStates, 0, length);
    set16_fx (hs->FilterStates_e, 0, sizeof(hs->FilterStates_e)/sizeof(hs->FilterStates_e[0]));
    hs->FilterStates_eg = 0;
    move16();
}

