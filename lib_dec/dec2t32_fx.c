/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"


/*==========================================================================*/
/* FUNCTION      : void dec_acelp_2t32_fx ()								*/
/*--------------------------------------------------------------------------*/
/* PURPOSE       :	                                                        */
/*	* 12 bits algebraic codebook decoder.									*/
/*  * 2 track x 32 positions per track = 64 samples.						*/
/*  * 12 bits --> 2 pulses in a frame of 64 samples.						*/
/*  * All pulses can have two (2) possible amplitudes: +1 or -1.			*/
/*  * Each pulse can have 32 possible positions.							*/
/*  * See cod2t32.c for more details of the algebraic code.					*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/* _Word16 i_subfr,   i  : subframe index                                   */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* _Word16 code[]     o  : algebraic (fixed) codebook excitation Q12        */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 															*/
/*==========================================================================*/


void dec_acelp_2t32_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    Word16 code[]    /* o:   algebraic (fixed) codebook excitation */
)
{

    Word16 index, i0, i1;

    index = (Word16) get_next_indice_fx( st_fx, 12 );
    move16();

    set16_fx( code, 0, L_SUBFR );

    /*------------------------------------------------------------------------------------------*
    * decode the positions and signs of pulses and build the codeword
    *------------------------------------------------------------------------------------------*/

    i0 = shl(s_and(shr(index, 6), NB_POS_FCB_2T-1), 1);

    i1 = add(shl(s_and(index, NB_POS_FCB_2T-1), 1), 1);


    code[i0] = -512;
    move16();
    if (s_and(index, 0x800) == 0)
    {
        code[i0] = 512;
        move16();
    }

    code[i1] = -512;
    move16();
    if (s_and(index, 0x20) == 0)
    {
        code[i1] = 512;
        move16();
    }

}


/*==========================================================================*/
/* FUNCTION      : void dec_acelp_1t64_fx ()								*/
/*--------------------------------------------------------------------------*/
/* PURPOSE       :	 * 7 bits algebraic codebook.							*/
/*  * 1 track x 64 positions per track = 64 samples.						*/
/*  * The pulse can have 64 possible positions and two (2) possible amplitudes: +1 or -1.*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/* _Word16 i_subfr,   i  : subframe index                                   */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* _Word16 code[]     o  : algebraic (fixed) codebook excitation Q12        */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 															*/
/*==========================================================================*/
void dec_acelp_1t64_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    Word16 code[]    /* o:   algebraic (fixed) codebook excitation Q12*/
)
{
    Word16 pos,sgn;

    /*-----------------------------------------------------------------*
    * decode the positions and signs of pulses and build the codeword
    *-----------------------------------------------------------------*/
    pos = (Word16)get_next_indice_fx( st_fx, 7 );
    move16();

    sgn = -512;
    move16();
    IF( sub(pos,L_SUBFR) >= 0)
    {
        pos = sub(pos, L_SUBFR);
        move16();
        sgn = 512;
        move16();
    }
    set16_fx(code, 0, L_SUBFR);
    code[pos] = sgn;
    move16();
    return;

}
