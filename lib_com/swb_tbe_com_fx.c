/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "basop_util.h"

/*-----------------------------------------------------------------*
* Local functions
*-----------------------------------------------------------------*/

static void create_random_vector_fx( Word16 output[], const Word16 length, Word16 seed[] );

static void flip_spectrum_fx( const Word16 input[], Word16 output[], const Word16 length );

static void Calc_st_filt_tbe(Word16 * apond2,Word16 * apond1,Word16 * parcor0,Word16 * sig_ltp_ptr,Word16 * mem_zero );

static void Hilbert_transform_fx( Word32 tmp_R[], Word32 tmp_I[], Word32 *tmpi_R, Word32 *tmpi_I, const Word16 length, const Word16 HB_stage_id );

static void Hilbert_transform_sp_fx(  Word16 tmp_R[], Word16 tmp_I[], Word32 *tmpi_R, Word32 *tmpi_I, const Word16 length, const Word16 HB_stage_id );

void Estimate_mix_factors_fx( const Word16 *shb_res, const Word16 Q_shb, const Word16 *exc16kWhtnd, const Word16 Q_bwe_exc, const Word16 *White_exc16k_frac,
                              const Word16 Q_frac, const Word32 pow1, const Word16 Q_pow1, const Word32 pow2, const Word16 Q_pow2, Word16 *vf_modified, Word16 *vf_ind );

/*-------------------------------------------------------------------*
* swb_tbe_reset()
*
* Reset the SWB TBE encoder
*-------------------------------------------------------------------*/


void swb_tbe_reset_fx(
    Word32 mem_csfilt[],
    Word16 mem_genSHBexc_filt_down_shb[],
    Word16 state_lpc_syn[],
    Word16 syn_overlap[],
    Word16 state_syn_shbexc[],
    Word16* tbe_demph,
    Word16* tbe_premph,
    Word16  mem_stp_swb[],
    Word16* gain_prec_swb
)
{
    set32_fx( mem_csfilt, 0, 2 );
    set16_fx( mem_genSHBexc_filt_down_shb, 0, (2*ALLPASSSECTIONS_STEEP+1));
    set16_fx( state_lpc_syn, 0, LPC_SHB_ORDER );

    set16_fx( syn_overlap, 0, L_SHB_LAHEAD );
    set16_fx( state_syn_shbexc, 0, L_SHB_LAHEAD );

    *tbe_demph = 0;
    move16();
    *tbe_premph = 0;
    move16();

    set16_fx(mem_stp_swb, 0, LPC_SHB_ORDER);
    *gain_prec_swb = 16384;
    move16();/*Q14 = 1 */

    return;
}


/*-------------------------------------------------------------------*
* swb_tbe_reset_synth()
*
* Reset the extra parameters needed for synthesis of the SWB TBE output
*-------------------------------------------------------------------*/


void swb_tbe_reset_synth_fx(
    Word32 genSHBsynth_Hilbert_Mem[],
    Word16 genSHBsynth_state_lsyn_filt_shb_local_fx[] )
{

    set32_fx( genSHBsynth_Hilbert_Mem, 0, HILBERT_MEM_SIZE );
    set16_fx( genSHBsynth_state_lsyn_filt_shb_local_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );

    return;
}


/*-------------------------------------------------------------------*
 * fb_tbe_reset_synth_fx()
 *
 * reset of FB TBE memories
 *-------------------------------------------------------------------*/

void fb_tbe_reset_synth_fx(
    Word32 fbbwe_hpf_mem_fx[][4],
    Word16 *prev_fbbwe_ratio_fx
)
{
    set32_fx( fbbwe_hpf_mem_fx[0], 0, 4 );
    set32_fx( fbbwe_hpf_mem_fx[1], 0, 4 );
    set32_fx( fbbwe_hpf_mem_fx[2], 0, 4 );
    set32_fx( fbbwe_hpf_mem_fx[3], 0, 4 );
    *prev_fbbwe_ratio_fx = 1;
    move16(); /*should be set to 1.0f, scaling unknown */

    return;
}


/*-------------------------------------------------------------------*
 * tbe_celp_exc_offset()
 *
 * Compute tbe bwe celp excitation offset
 *-------------------------------------------------------------------*/

Word16 tbe_celp_exc_offset(
    const Word16 T0_fx,               /* i  : Integer pitch */
    const Word16 T0_frac_fx,          /* i  : Fractional part of the pitch */
    const Word16 L_frame              /* i  : frame lenght */
)
{
    Word16 offset_fx, tmp_fx, tmp1_fx, tmp2_fx, tmp_fac;
    tmp_fac = 320;
    move16();  /*2.5 in Q7*/
    if(sub(L_frame, L_FRAME16k) == 0)
    {
        tmp_fac = 256;
        move16(); /*2.0 in Q7*/
    }
    tmp_fx = extract_l(L_mult(T0_frac_fx,32));/*Q8, 0.25 in Q7*/
    tmp_fx = add(512,tmp_fx);/*Q8; 2 in Q8*/
    tmp_fx = mult_r(tmp_fx,tmp_fac);/*Q16->Q0; 2.5 in Q7 or  2.0 in Q7 */

    tmp1_fx = sub(T0_fx,2);/*Q0*/

    tmp2_fx = shl(tmp1_fx,1);/*Q0 */

    IF(sub(L_frame, L_FRAME) == 0)
    {
        tmp2_fx = add(shl(tmp1_fx,1),shr(tmp1_fx,1));/*Q0; (5/2 = 2 + 1/2)*/
    }

    offset_fx = add(tmp_fx,tmp2_fx);/*Q0*/

    return offset_fx;
}

/*-------------------------------------------------------------------*
* swb_tbe_celp_exc()
*
* Compute tbe bwe celp excitation
*-------------------------------------------------------------------*/
void tbe_celp_exc(
    const Word16 L_frame_fx,          /* i  : Frame lenght */
    const Word16 i_subfr_fx,          /* i  : sub frame */
    const Word16 T0_fx,               /* i  : Integer pitch */
    const Word16 T0_frac_fx,          /* i  : Fractional part of the pitch */
    Word16 *error_fx,           /* i/o: Error */
    Word16 *bwe_exc_fx          /* i/o: bandwitdh extension signal */
)
{
    Word16 offset_fx, tmp_fx, i;
    IF( sub(L_frame_fx,L_FRAME) == 0 )
    {
        /*offset = T0 * HIBND_ACB_L_FAC + (int) ((float) T0_frac * 0.25f * HIBND_ACB_L_FAC + 2 * HIBND_ACB_L_FAC + 0.5f) - 2 * HIBND_ACB_L_FAC;
        for (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
        {
            bwe_exc[i + i_subfr * HIBND_ACB_L_FAC] = bwe_exc[i + i_subfr * HIBND_ACB_L_FAC - offset + (int) error];
        }
        error += (float) offset - (float) T0 * HIBND_ACB_L_FAC  - 0.25f * HIBND_ACB_L_FAC * (float) T0_frac;*/

        offset_fx = tbe_celp_exc_offset(T0_fx, T0_frac_fx, L_frame_fx);
        IF(*error_fx>0)
        {
            tmp_fx = shr(*error_fx,5);/*Q0*/
        }
        ELSE
        {
            tmp_fx = negate(shr(abs_s(*error_fx),5));/*Q0*/
        }

        FOR (i=0; i<L_SUBFR * HIBND_ACB_L_FAC; i++)
        {
            bwe_exc_fx[i + i_subfr_fx * HIBND_ACB_L_FAC] = bwe_exc_fx[i + i_subfr_fx * HIBND_ACB_L_FAC - offset_fx + tmp_fx];
            move16();
        }
        tmp_fx = extract_l(L_mult(T0_frac_fx,1));/*Q3; 0.25 in Q2*/
        tmp_fx = add(shl(T0_fx,3),tmp_fx);/*Q3*/
        tmp_fx = extract_l(L_mult(tmp_fx,5));/*Q5, 2.5 in Q1*/
        tmp_fx = sub(shl(offset_fx,5),tmp_fx);/*Q5*/
        *error_fx = add(*error_fx,tmp_fx);/*Q5*/
    }
    ELSE
    {
        /*  offset = T0*2.5 + (int) ((float) T0_frac * 0.25f*2.5 + 2*2.5 + 0.5f) - 2*2.5;  - case above*/
        /*  offset = T0*2   + (int) ((float) T0_frac * 0.25f*2   + 2*2   + 0.5f) - 2*2;    - case here*/

        /*(int) ((float) T0_frac * 0.25f*2   + 2*2   + 0.5f)*/
        offset_fx = tbe_celp_exc_offset(T0_fx, T0_frac_fx, L_frame_fx);
        IF(*error_fx>0)
        {
            tmp_fx = shr(*error_fx,5);/*Q0*/
        }
        ELSE
        {
            tmp_fx = negate(shr(abs_s(*error_fx),5));/*Q0*/
        }

        FOR (i=0; i<L_SUBFR * 2; i++)
        {
            bwe_exc_fx[i + i_subfr_fx * 2] = bwe_exc_fx[i + i_subfr_fx * 2 - offset_fx + tmp_fx];
            move16();
        }

        /* error += (float) offset - (float) T0 * 2 - 0.5f * (float) T0_frac;*/
        tmp_fx = extract_l(L_mult(T0_frac_fx,2));/*Q3; 0.5 in Q2*/
        tmp_fx = add(shl(T0_fx,4),tmp_fx);/* now tmp_fx = "T0_fx*2+ 0.5f*T0_frac_fx" in Q3*/
        tmp_fx = shl(tmp_fx,2);/*now above tmp_fx in Q5*/
        tmp_fx = sub(shl(offset_fx,5),tmp_fx);/*move offset_fx to Q5, tmp_fx in Q5, ans tmp_fx in Q5*/
        *error_fx = add(*error_fx,tmp_fx);/*error_fx in Q5*/
    }
}
/*===========================================================================*/
/* FUNCTION : flip_and_downmix_generic_fx() */
/*---------------------------------------------------------------------------*/
/* PURPOSE :flips the spectrum and downmixes the signals, lpf if needed*/
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS */
/* _(Word16[]) input :input spectrum */
/* _(Word16) length :length of spectra */
/* _(Word16) ramp_flag :flag to indicate slow ramp need after switching */
/* _(Word16[]) lpf_num :lpf numerator */
/* _(Word16[]) lpf_den :lpf denominator */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16[])output : output spectrum */
/*---------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _(Word32[9])mem1 : memory */
/* _(Word32[8])mem2 : memory */
/* _(Word32[8])mem3 : memory */
/* _(Word16) dm_frequency_inHz :Downmix frequency in Hz */
/* _(Word16*) phase_state :Phase state in case frequency isn't */
/* multiple of 50 Hz */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*---------------------------------------------------------------------------*/
void flip_and_downmix_generic_fx( Word16 input[],   /* i : input spectrum Qx*/
                                  Word16 output[],                            /* o : output spectrum Qx*/
                                  const Word16 length,                              /* i : length of spectra */
                                  Word32 mem1_ext[HILBERT_ORDER1],            /* i/o: memory Qx+16*/
                                  Word32 mem2_ext[2*HILBERT_ORDER2],          /* i/o: memory Qx+16*/
                                  Word32 mem3_ext[2*HILBERT_ORDER2],          /* i/o: memory Qx+16*/
                                  Word16* phase_state                         /* i/o: Phase state in case frequency isn't multiple of 50 Hz */ )
{
    Word16 i, j;
    Word16 tmp_16[L_FRAME32k + HILBERT_ORDER1];
    Word32 tmpi_R[L_FRAME32k];
    Word32 tmpi_I[L_FRAME32k];
    Word32 tmpi2_R[L_FRAME32k + HILBERT_ORDER2];
    Word32 tmpi2_I[L_FRAME32k + HILBERT_ORDER2];
    Word32 tmp_R[L_FRAME32k + HILBERT_ORDER2];
    Word32 tmp_I[L_FRAME32k + HILBERT_ORDER2];

    /*Word16 s_tmp[L_FRAME32k];*/
    /*Word16 factor;*/
    Word16 period;
    Word16 local_negsin_table17[17] = {     0,  -11793,  -22005,  -29268,  -32609,  -31580,
                                            -26319,  -17530,   -6393,    6393,   17530,   26319,
                                            31580,   32609,   29268,   22005,   11793
                                      };      /* Q15 */
    Word16 local_cos_table17[17] = {32767,   30571,   24279,   14732,   3212,   -8739,
                                    -19519,   -27683,   -32137,   -32137,   -27683,
                                    -19519,   -8739,   3212,   14732,   24279,   30571
                                   };   /* Q15 */
    Word16 *local_negsin_table, *local_cos_table;
    Word32 L_tmp;

    /* 1850 Hz downmix */
    period = 17;
    move16();
    local_negsin_table = local_negsin_table17;
    local_cos_table = local_cos_table17;


    FOR (i = 0; i < length; i = i + 2 )
    {
        input[i] = negate( input[i] );
        move16();
    }

    Copy(input, tmp_16 + HILBERT_ORDER1, length );

    /*Copy32( mem1_ext, tmp_16, 5 ); */
    FOR (i=0; i<HILBERT_ORDER1; i++)
    {
        tmp_16[i] = extract_h(mem1_ext[i]); /* mem1_ext (Qx+16) tmp16 (Qx) */
    }

    /* Hilber transform stage - 0 - single precision */
    Hilbert_transform_sp_fx(  tmp_16,                /* i: Real component of HB */
                              tmp_16,                /* i: Imag component of HB */
                              tmpi_R,                /* o: Real component of HB */
                              tmpi_I,                /* o: Imag. component of HB */
                              length,                /* i: length of the spectra */
                              0);                    /* i: HB transform stage */

    FOR (i=0; i<HILBERT_ORDER1; i++)
    {
        mem1_ext[i] = L_deposit_h(tmp_16[i+length]); /* mem1_ext (Qx+16) tmp16 (Qx) */
    }

    Copy32( mem2_ext, tmpi2_R, HILBERT_ORDER2 );
    Copy32( mem3_ext, tmpi2_I, HILBERT_ORDER2 );

    /* Hilber transform stage - 1 */
    Hilbert_transform_fx( tmpi_R,             /* i: Real component of HB */
                          tmpi_I,             /* i: Imag component of HB */
                          tmpi2_R,            /* o: Real component of HB */
                          tmpi2_I,            /* o: Imag. component of HB */
                          length,             /* i: length of the spectra */
                          1);                 /* i: HB transform stage */

    Copy32( mem2_ext+HILBERT_ORDER2, tmp_R, HILBERT_ORDER2 );
    Copy32( mem3_ext+HILBERT_ORDER2, tmp_I, HILBERT_ORDER2 );

    /* Hilber transform stage - 2 */
    Hilbert_transform_fx( tmpi2_R,            /* i: Real component of HB */
                          tmpi2_I,            /* i: Imag component of HB */
                          tmpi_R,             /* o: Real component of HB */
                          tmpi_I,             /* o: Imag. component of HB */
                          length,             /* i: length of the spectra */
                          2);                 /* i: HB transform stage */

    Copy32( tmpi2_R + length, mem2_ext, HILBERT_ORDER2 );
    Copy32( tmpi2_I + length, mem3_ext, HILBERT_ORDER2 );

    /* Hilber transform stage - 3 */
    Hilbert_transform_fx( tmpi_R,             /* i: Real component of HB */
                          tmpi_I,             /* i: Imag component of HB */
                          tmp_R,              /* o: Real component of HB */
                          tmp_I,              /* o: Imag. component of HB */
                          length,             /* i: length of the spectra */
                          3);                 /* i: HB transform stage */

    Copy32( tmp_R + length, mem2_ext+HILBERT_ORDER2, HILBERT_ORDER2 );
    Copy32( tmp_I + length, mem3_ext+HILBERT_ORDER2, HILBERT_ORDER2 );

    if ( *phase_state >= period )
    {
        *phase_state = 0;
        move16();
    }

    i = 0;
    move16();
    j = *phase_state;
    move16();

    WHILE (  i < length )
    {
        WHILE ( ( j < period ) && ( i < length ) )
        {
            L_tmp = Mult_32_16( tmp_R[i + 4], local_cos_table[j] ); /*//Qx+16 */
            L_tmp = Madd_32_16( L_tmp, tmp_I[i + 4], local_negsin_table[j] ); /*Qx+16 */
            output[i] = round_fx( L_tmp ); /*Qx */
            i++ ;
            j++ ;
        }

        if ( j >= period )
        {
            j = 0;
            move16();
        }
    }

    *phase_state = j;
    move16();
    return;
}


/*----------------------------------------------
 * Hilbert transform - Double precision
 *------------------------------------------------*/
static void Hilbert_transform_fx( Word32 tmp_R[],              /* i: Real component of HB */
                                  Word32 tmp_I[],              /* i: Real component of HB */
                                  Word32 tmpi_R[],             /* o: Real component of HB */
                                  Word32 tmpi_I[],             /* o: Imag. component of HB */
                                  const Word16 length,               /* i: input length */
                                  const Word16 HB_stage_id           /* i: HB transform stage */
                                )
{
    Word16 i, hb_filter_stage, offset;
    Word32 L_tmp;

    hb_filter_stage = 2*HB_stage_id;
    offset = (HB_stage_id == 0) ? 1 : 0;

    test();
    test();
    IF (HB_stage_id == 0 || HB_stage_id == 2)
    {
        FOR ( i = 0; i < length; i++ )
        {
            L_tmp     = Mult_32_16(        tmp_R[i + 4], Hilbert_coeffs_fx[hb_filter_stage][0 + offset] ); /*Qx+15 */
            L_tmp     = Madd_32_16( L_tmp, tmp_R[i + 2], Hilbert_coeffs_fx[hb_filter_stage][2 + offset] ); /*Qx+15 */
            L_tmp     = Madd_32_16( L_tmp, tmp_R[i]    , Hilbert_coeffs_fx[hb_filter_stage][4 + offset] );/*Qx+15 */
            tmpi_R[i] = L_shl( L_tmp, 1 );
            move32(); /*Qx+16 */

            L_tmp     = Mult_32_16(        tmp_I[i + 4 + offset], Hilbert_coeffs_fx[hb_filter_stage+1][0] ); /*Qx+15 */
            L_tmp     = Madd_32_16( L_tmp, tmp_I[i + 2 + offset], Hilbert_coeffs_fx[hb_filter_stage+1][2] ); /*Qx+15 */
            L_tmp     = Madd_32_16( L_tmp, tmp_I[i + offset]    , Hilbert_coeffs_fx[hb_filter_stage+1][4] ); /*Qx+15 */
            tmpi_I[i] = L_shl( L_tmp, 1 );
            move32(); /*Qx+16 */
        }
    }
    ELSE IF (HB_stage_id == 1 || HB_stage_id == 3)
    {
        FOR ( i = 0; i < length; i++ )
        {

            L_tmp           = Mult_32_16(        tmpi_R[i + 2], Hilbert_coeffs_fx[hb_filter_stage][2] ); /*Qx+15 */
            L_tmp           = Madd_32_16( L_tmp, tmpi_R[i]    , Hilbert_coeffs_fx[hb_filter_stage][4] ); /*Qx+15 */
            tmpi_R[i + 4]   = L_sub( tmp_R[i], L_shl( L_tmp, 1 ) );
            move32();/*Qx+16 */

            L_tmp           = Mult_32_16(        tmpi_I[i + 2], Hilbert_coeffs_fx[hb_filter_stage+1][2] ); /*Qx+15 */
            L_tmp           = Madd_32_16( L_tmp, tmpi_I[i]    , Hilbert_coeffs_fx[hb_filter_stage+1][4] ); /*Qx+15 */
            tmpi_I[i + 4]   = L_sub( tmp_I[i], L_shl( L_tmp, 1 ) );
            move32(); /*Qx+16 */
        }
    }
}


/*----------------------------------------------
* Hilbert transform - Single precision Stage 0
*------------------------------------------------*/
static void Hilbert_transform_sp_fx( Word16 tmp_R[],              /* i: Real component of HB */
                                     Word16 tmp_I[],              /* i: Real component of HB */
                                     Word32 tmpi_R[],             /* o: Real component of HB */
                                     Word32 tmpi_I[],             /* o: Imag. component of HB */
                                     const Word16 length,               /* i: input length */
                                     const Word16 HB_stage_id           /* i: HB transform stage */
                                   )
{
    Word16 i, hb_filter_stage, offset;
    Word32 L_tmp;

    hb_filter_stage = 2*HB_stage_id;
    offset = (HB_stage_id == 0) ? 1 : 0;

    /* Hilbert single precision stage 0 */
    FOR ( i = 0; i < length; i++ )
    {
        L_tmp     = L_mult(        tmp_R[i + 4], Hilbert_coeffs_fx[hb_filter_stage][0 + offset] ); /*Qx */
        L_tmp     = L_mac ( L_tmp, tmp_R[i + 2], Hilbert_coeffs_fx[hb_filter_stage][2 + offset] ); /*Qx */
        L_tmp     = L_mac ( L_tmp, tmp_R[i]    , Hilbert_coeffs_fx[hb_filter_stage][4 + offset] ); /*Qx */
        tmpi_R[i] = L_shl ( L_tmp, 1 );
        move32(); /*Qx+16 */

        L_tmp     = L_mult(        tmp_I[i + 4 + offset], Hilbert_coeffs_fx[hb_filter_stage+1][0] ); /*Qx */
        L_tmp     = L_mac ( L_tmp, tmp_I[i + 2 + offset], Hilbert_coeffs_fx[hb_filter_stage+1][2] ); /*Qx */
        L_tmp     = L_mac ( L_tmp, tmp_I[i + offset]    , Hilbert_coeffs_fx[hb_filter_stage+1][4] ); /*Qx */
        tmpi_I[i] = L_shl ( L_tmp, 1 );
        move32(); /*Qx+16 */
    }

    return;
}


/*----------------------------------------------
 * flip_spectrum_fx
 *----------------------------------------------*/
void flip_spectrum_fx(
    const Word16 input[],    /* i : input spectrum */
    Word16 output[],   /* o : output spectrum */
    const Word16 length      /* i : vector length */
)
{
    Word16 i;

    FOR ( i = 0; i < length; i = i + 2 )
    {
        output[i] = negate( input[i] );
        move16();
        output[i + 1] = input[i + 1];
        move16();
    }

    return;
}


/*----------------------------------------------------------------------------
 * calc_rc0_h
 *
 * computes 1st parcor from composed filter impulse response
 *---------------------------------------------------------------------------*/
void Calc_rc0_h(
    Word16 * h,   /* i  : impulse response of composed filter */
    Word16 * rc0  /* o  : 1st parcor */
)
{
    Word32 L_acc;
    Word16 *ptrs;
    Word16 acf0, acf1;
    Word16 temp, sh_acf;
    Word16 i;

    /* computation of the autocorrelation function acf */
    L_acc = L_mult(h[0], h[0]);
    FOR(i = 1; i < LONG_H_ST; i++)
    {
        L_acc = L_mac(L_acc, h[i], h[i]);
    }
    sh_acf = norm_l(L_acc);
    L_acc = L_shl(L_acc, sh_acf);
    acf0 = extract_h(L_acc);

    ptrs = h;

    temp = *ptrs++;
    move16();
    L_acc = L_mult(temp, *ptrs);
    FOR(i = 1; i < LONG_H_ST - 1; i++)
    {
        temp = *ptrs++;
        move16();
        L_acc = L_mac(L_acc, temp, *ptrs);
    }
    L_acc = L_shl(L_acc, sh_acf);
    acf1 = extract_h(L_acc);

    /* Compute 1st parcor */
    if (acf0 == 0)
    {
        *rc0 = 0;
        move16();
        return;
    }

    IF (sub(acf0, abs_s(acf1)) < 0)
    {
        *rc0 = 0;
        move16();
        return;
    }
    *rc0 = div_s(abs_s(acf1), acf0);
    move16();
    if (acf1 > 0)
    {
        *rc0 = negate(*rc0);
        move16();
    }
}

static void Calc_st_filt_tbe(
    Word16 * apond2,      /* i  : coefficients of numerator             */
    Word16 * apond1,      /* i  : coefficients of denominator           */
    Word16 * parcor0,     /* o  : 1st parcor calcul. on composed filter */
    Word16 * sig_ltp_ptr, /* i/o: i  of 1/A(gamma1) : scaled by 1/g0    */
    Word16 * mem_zero     /* i  : All zero memory                       */
)
{
    Word32 L_g0;

    Word16 h[LONG_H_ST];

    Word16 g0, temp;
    Word16 i;
    temp = sub( 2, norm_s( apond2[0] ) );
    /* compute i.r. of composed filter apond2 / apond1 */
    Syn_filt_s(temp, apond1, LPC_SHB_ORDER, apond2, h, LONG_H_ST, mem_zero, 0);
    /* compute 1st parcor */
    Calc_rc0_h(h, parcor0);

    /* compute g0 */
    L_g0 = L_mult0(1, abs_s(h[0]));
    FOR (i = 1; i < LONG_H_ST; i++)
    {
        L_g0 = L_mac0(L_g0, 1, abs_s(h[i]));
    }
    g0 = extract_h(L_shl(L_g0, 14));

    /* Scale signal i  of 1/A(gamma1) */
    IF (sub(g0, 1024) > 0)
    {
        temp = div_s(1024, g0); /* temp = 2**15 / gain0 */
        FOR (i = 0; i < L_SUBFR16k; i++)
        {
            sig_ltp_ptr[i] = mult_r(sig_ltp_ptr[i], temp);
            move16();
        }
    }
}

static void filt_mu_fx(
    const Word16 *sig_in,       /* i  : signal (beginning at sample -1) */
    Word16 *sig_out,      /* o  : output signal                   */
    const Word16 parcor0,       /* i  : parcor0 (mu = parcor0 * gamma3) */
    Word16 SubFrameLength /* i  : the length of subframe          */
)
{
    Word16 n;
    Word16 mu, ga, temp;
    const Word16 *ptrs;
    Word16 tmp,exp;


    IF ( SubFrameLength == L_SUBFR )
    {
        IF (parcor0 > 0)
        {
            mu = mult_r(parcor0 , GAMMA3_PLUS_FX);
        }
        ELSE
        {
            mu = mult_r(parcor0 , GAMMA3_MINUS_FX);
        }
    }
    ELSE
    {
        IF(parcor0 > 0)
        {
            mu = mult_r( parcor0 , GAMMA3_PLUS_WB_FX);
        }
        ELSE
        {
            mu = mult_r( parcor0 , GAMMA3_MINUS_WB_FX);
        }
    }

    tmp = abs_s(mu);
    tmp = sub(32767 , tmp);
    exp = norm_s(tmp);
    tmp = div_s((1<<(14-exp)),tmp);/*(14 - exp) */
    ga = shl(tmp ,exp);/*Q14 */


    /*    ga = (float) 1. / ((float) 1. - (float) fabs (mu)); */

    ptrs = sig_in;                /* points on sig_in(-1) */

    FOR (n = 0; n < SubFrameLength; n++)
    {
        temp = mult_r(mu , (*ptrs++));
        temp = add (temp ,*ptrs );/*Q12 */
        sig_out[n] = shl(mult_r( ga , temp),1);
        move16();/*Q12 */
    }

    return;
}

static void scale_st_swb(
    const Word16 * sig_in_fx,    /* i  : postfilter i signal             */
    Word16 * sig_out_fx,   /* i/o: postfilter o signal             */
    Word16 * gain_prec_fx, /* i/o: last value of gain for subframe */
    Word16 SubFrameLength
)
{
    Word16 i;
    Word16 agc_fac1_para_fx;
    Word16 agc_fac_para_fx;
    Word32 L_acc,L_temp;
    Word16 g0_fx, gain_fx;
    Word16 scal_in, scal_out;
    Word16 s_g_in, s_g_out,sh_g0,temp;


    IF( SubFrameLength == L_SUBFR )
    {
        agc_fac1_para_fx = AGC_FAC1_FX;
        move16();
        agc_fac_para_fx  = AGC_FAC_FX;
        move16();
    }
    ELSE /*IF( SubFrameLength == L_SUBFR16k ) */
    {
        agc_fac1_para_fx  = AGC_FAC1_WB_FX;
        move16();
        agc_fac_para_fx  = AGC_FAC_WB_FX;
        move16();
    }

    /* compute input gain */
    L_acc = L_mult0(1, abs_s(sig_in_fx[0]));/*0 +Q_bwe_exc-1 */
    FOR (i = 1; i < SubFrameLength; i++)
    {
        L_acc = L_mac0(L_acc, 1, abs_s(sig_in_fx[i]));  /*Q_bwe_exc-1 */
    }

    g0_fx = 0;
    move16();
    IF (L_acc != 0L)
    {
        scal_in = norm_l(L_acc);
        L_acc = L_shl(L_acc, scal_in);
        s_g_in = extract_h(L_acc); /* normalized */

        /* Compute o   gain */
        L_acc = L_mult0(1, abs_s(sig_out_fx[0]));
        FOR (i = 1; i < SubFrameLength; i++)
        {
            L_acc = L_mac0(L_acc, 1, abs_s(sig_out_fx[i]));
        }
        IF (L_acc == 0L)
        {
            *gain_prec_fx = 0;
            move16();

            return;
        }

        scal_out = norm_l(L_acc);
        L_acc = L_shl(L_acc, scal_out);
        s_g_out = extract_h(L_acc); /* normalized */


        sh_g0 = add(scal_in, 1);
        sh_g0 = sub(sh_g0, scal_out); /* scal_in - scal_out + 1 */
        IF (sub(s_g_in, s_g_out) < 0)
        {
            g0_fx = div_s(s_g_in, s_g_out); /* s_g_in/s_g_out in Q15 */
        }
        ELSE
        {
            temp = sub(s_g_in, s_g_out); /* sufficient since normalized */
            g0_fx = shr(div_s(temp, s_g_out), 1);
            g0_fx = add(g0_fx, (Word16) 0x4000); /* s_g_in/s_g_out in Q14 */
            sh_g0 = sub(sh_g0, 1);
        }
        /* L_gain_in/L_gain_out in Q14 */
        /* overflows if L_gain_in > 2 * L_gain_out */
        g0_fx = shr(g0_fx, sh_g0); /* sh_g0 may be >0, <0, or =0 */

        g0_fx = mult_r(g0_fx, agc_fac1_para_fx); /* L_gain_in/L_gain_out * AGC_FAC1_FX */
    }
    /* compute gain(n) = AGC_FAC gain(n-1) + (1-AGC_FAC)gain_in/gain_out */
    /* sig_out(n) = gain(n) sig_out(n) */
    gain_fx = *gain_prec_fx;
    move16(); /*14 */
    FOR (i = 0; i < SubFrameLength; i++)
    {
        temp = mult_r(agc_fac_para_fx, gain_fx);/*15 +14 -15 =14 */
        gain_fx = add(temp, g0_fx); /* in Q14 */
        L_temp = L_mult(gain_fx, sig_out_fx[i]);/*14 + Q_bwe_exc-1 +1 = 14 + Q_bwe_exc */
        L_temp = L_shl(L_temp, 1); /*14 + Q_bwe_exc +1 */
        sig_out_fx[i] = round_fx(L_temp); /*Q_bwe_exc +15 -16 = Q_bwe_exc-1 */
    }
    *gain_prec_fx =gain_fx;
    move16();

    return;
}

void PostShortTerm_fx(
    Word16 *sig_in,             /* i  : input signal (pointer to current subframe */
    Word16 *lpccoeff,           /* i  : LPC coefficients for current subframe */
    Word16 *sig_out,            /* o  : postfiltered output */
    Word16 *mem_stp,            /* i/o: postfilter memory*/
    Word16 *ptr_mem_stp,        /* i/o: pointer to postfilter memory*/
    Word16 *ptr_gain_prec,      /* i/o: for gain adjustment*/
    Word16 *mem_zero,           /* i/o: null memory to compute h_st*/
    Word16 formant_fac_fx       /* i  : Strength of post-filter*/
)
{
    Word16 apond1_fx[LPC_SHB_ORDER+1];        /* denominator coeff.*/
    Word16 apond2_fx[LONG_H_ST];              /* numerator coeff.  */
    Word16 sig_ltp_fx[L_SUBFR16k +1];         /* residual signal   */
    /*Word16 lpccoeff_fx[LPC_SHB_ORDER+1];//Q12 */
    Word16 g1_fx,g2_fx,parcor0_fx;                 /*Q15 */
    Word16 tmp;

    parcor0_fx = 0;
    move16();
    set16_fx( apond1_fx, 0, LPC_SHB_ORDER+1 );
    set16_fx( apond2_fx, 0, LONG_H_ST );
    set16_fx( sig_ltp_fx, 0, L_SUBFR16k+1 );

    /* Obtain post-filter weights  */
    tmp = extract_h(L_mult(GAMMA_SHARP_FX,formant_fac_fx));/*Q15 */
    g1_fx = add(GAMMA0_FX,tmp);/*Q15  */
    g2_fx = sub(GAMMA0_FX,tmp);/*Q15 */

    /* Compute weighted LPC coefficients */
    weight_a_fx(lpccoeff, apond1_fx, g1_fx, LPC_SHB_ORDER);
    weight_a_fx(lpccoeff, apond2_fx, g2_fx, LPC_SHB_ORDER);
    /* o: apond1_fx, apond2_fx in Q12 */

    /* Compute A(gamma2) residual */
    Residu3_10_fx( apond2_fx, sig_in, sig_ltp_fx+1, L_SUBFR16k, 0 );
    /* o: sig_ltp_fx in Q_bwe_exc */

    /* Save last output of 1/A(gamma1)  */
    sig_ltp_fx[0] = *ptr_mem_stp;
    move16();

    /* Control short term pst filter gain and compute parcor0   */
    Calc_st_filt_tbe(apond2_fx, apond1_fx, &parcor0_fx, sig_ltp_fx+1, mem_zero);
    /* o: parcor0 in Q15 */
    /* i/o: sig_ltp_fx in Q_bwe_exc */

    /* 1/A(gamma1) filtering, mem_stp is updated */
    Syn_filt_s(0,apond1_fx,LPC_SHB_ORDER,sig_ltp_fx+1,sig_ltp_fx+1,L_SUBFR16k,mem_stp,1);

    /* (1 + mu z-1) tilt filtering */
    filt_mu_fx(sig_ltp_fx, sig_out, parcor0_fx, L_SUBFR16k);
    /* o: sig_out in Q_bwe_exc */

    /* gain control */
    scale_st_swb(sig_in, sig_out, ptr_gain_prec, L_SUBFR16k );

    return;
}

void flip_spectrum_and_decimby4_fx(
    const Word16 input[], /* i : input spectrum Q_inp     */
    Word16 output[],                                /* o : output spectrum Q_inp    */
    const Word16 length,                            /* i : vector length            */
    Word16 mem1[],                                  /* i/o : memory Q_inp           */
    Word16 mem2[],                                  /* i/o : memory Q_inp           */
    const Word16
    ramp_flag                                      /*i: flag to trigger slow ramp-up of output following change of core (HQ to ACELP or 12k8 to 16k ACELP) */ )
{
    Word16 i;
    Word16 factor, tmp[L_FRAME16k/2];
    Word16 tmp1, tmp2;
    Word16 input_change[L_FRAME16k];

    IF ( ramp_flag )
    {
        factor = div_s( 4, length ); /* Q15 */
        FOR ( i = 0; i < length / 4; i += 2)
        {
            tmp1 = extract_l( L_mult0( i, factor ) ); /* Q15 */
            tmp2 = extract_l( L_mult0( add( i, 1 ), factor ) ); /*Q15 */
            input_change[i] = negate( mult_r( input[i], tmp1 ) );
            move16();
            input_change[i + 1] = mult_r( input[i + 1], tmp2 );
            move16();
        }
    }
    ELSE
    {
        i = 0;
        move16();
    }

    FOR (; i < length; i = i + 2 )
    {
        input_change[i] = negate( input[i] );
        move16();
        input_change[i + 1] = input[i + 1];
        move16();
    }

    Decimate_allpass_steep_fx( input_change, mem1, L_FRAME16k, tmp );
    Decimate_allpass_steep_fx( tmp, mem2, L_FRAME16k / 2, output );

    return;
}


/*==========================================================================*/
/* FUNCTION : void GenShapedWBExcitation_fx () */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Synthesize spectrally shaped highband excitation signal for the wideband */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _Word16 *lpc_shb i : lpc coefficients Q12 */
/* _Word16 coder_type i : coding type */
/* _Word16 *bwe_exc_extended i : bandwidth extended exciatation Q_bwe_exc*/
/* _Word16 Q_bwe_exc i : Q format */
/* _Word16 voice_factors[] i : voicing factor Q15 */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _Word16 *excSHB o : synthesized shaped shb exctiation Q_bwe_exc*/
/* _Word16 *exc4kWhtnd o : whitened synthesized shb excitation Q_bwe_exc*/
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _Word32 *mem_csfilt i/o : memory Q_bwe_exc+16*/
/* _Word16 *mem_genSHBexc_filt_down1 i/o : memory Q_bwe_exc */
/* _Word16 *mem_genSHBexc_filt_down2 i/o : memory Q_bwe_exc */
/* _Word16 *mem_genSHBexc_filt_down3 i/o : memory Q_bwe_exc */
/* _Word16 *state_lpc_syn i/o : memory Q_bwe_exc */
/* _Word16 bwe_seed[] i/o : random number generator seed */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/
/* CALLED FROM : */
/*==========================================================================*/
void GenShapedWBExcitation_fx( Word16* excSHB,  /* o : synthesized shaped shb exctiation Q_bwe_exc*/
                               const Word16* lpc_shb,                      /* i : lpc coefficients Q12*/
                               Word16* exc4kWhtnd,                         /* o : whitened synthesized shb excitation Q_bwe_exc*/
                               Word32* mem_csfilt,                         /* i/o : memory Q_bwe_exc+16*/
                               Word16* mem_genSHBexc_filt_down1,           /* i/o : memory Q_bwe_exc*/
                               Word16* mem_genSHBexc_filt_down2,           /* i/o : memory Q_bwe_exc*/
                               Word16* mem_genSHBexc_filt_down3,           /* i/o : memory Q_bwe_exc*/
                               Word16* state_lpc_syn,                      /* i/o : memory Q_bwe_exc*/
                               const Word16 coder_type,                    /* i : coding type */
                               const Word16* bwe_exc_extended,             /* i : bandwidth extended exciatation Q_bwe_exc*/
                               const Word16 Q_bwe_exc, Word16 bwe_seed[],  /* i/o : random number generator seed */
                               const Word16 voice_factors[],               /* i : voicing factor Q15*/
                               const Word16 uv_flag                        /* i : unvoiced flag */
                               , const Word16 igf_flag
                             )
{
    Word16 i, j, k;
    Word16 wht_fil_mem [ LPC_WHTN_ORDER_WB ];
    Word16 lpc_whtn[ LPC_WHTN_ORDER_WB + 1];
    Word16 R_h[LPC_WHTN_ORDER_WB + 2], R_l[LPC_WHTN_ORDER_WB + 2];
    Word16 Q_R;
    Word16 excTmp[ L_FRAME16k];
    Word16 excTmp2[ L_FRAME16k / 4];
    Word16 excTmp2_frac[ L_FRAME16k / 4];
    Word16 exc4k[ L_FRAME16k / 4];
    Word16 exc4k_frac[ L_FRAME16k / 4];
    Word32 exc4k_32[ L_FRAME16k / 4];
    Word32 pow1, pow2;
    Word16 scale;
    Word32 excNoisyEnv[ L_FRAME16k / 4];
    Word16 csfilt_num2[1]       = {1638}; /* Q15*/
    Word16 neg_csfilt_den2[2]   = {-32768, 31457}; /* Q15 */
    Word32 L_tmp, Ltemp1, Ltemp2;
    Word16 temp1, temp2, exp;
    Word32 Lmax;
    Word16 max, n1, n2, sc;
    Word32 LepsP[LPC_WHTN_ORDER_WB+1];
    Word16 tmp_vfac;
    Word16 avg_voice_fac;

    /*0.25f*sum_f(voice_factors, NB_SUBFR)*/
    L_tmp = L_mult(voice_factors[0], 8192);
    FOR (i=1; i<NB_SUBFR; i++)
    {
        L_tmp = L_mac(L_tmp, voice_factors[i], 8192);
    }
    avg_voice_fac = round_fx(L_tmp);

    test();
    test();
    test();
    test();
    IF( igf_flag != 0 && ( sub(coder_type, VOICED) == 0 || sub( avg_voice_fac, 11469 ) > 0 ) ) /*Q15 -> 0.35f*/
    {
        csfilt_num2[0] = 6554;
        move16();       /*Q15 -> 0.2f*/
        neg_csfilt_den2[1] = 26214;
        move16();  /*Q15 -> 0.8f*/
    }
    ELSE IF( igf_flag != 0 && ( sub(coder_type, UNVOICED) == 0 || sub( avg_voice_fac, 6654 ) < 0 ) ) /*Q15 -> 0.2f*/
    {
        csfilt_num2[0] = 328;
        move16();        /*Q15 -> 0.01f*/
        neg_csfilt_den2[1] = 32440;
        move16();  /*Q15 -> 0.99f*/
    }
    set16_fx( wht_fil_mem, 0, LPC_WHTN_ORDER_WB );
    Decimate_allpass_steep_fx( bwe_exc_extended, mem_genSHBexc_filt_down1,  L_FRAME32k, excTmp );
    flip_spectrum_and_decimby4_fx( excTmp, exc4k, L_FRAME16k, mem_genSHBexc_filt_down2, mem_genSHBexc_filt_down3, 0 );

    IF ( uv_flag )
    {
        create_random_vector_fx( exc4kWhtnd, L_FRAME16k / 4, bwe_seed );
        IF ( sub( Q_bwe_exc, 5 ) < 0 )
        {

            FOR ( i = 0; i < L_FRAME16k / 4; i++ )
            {
                exc4kWhtnd[i] = shl_r( exc4kWhtnd[i], sub( Q_bwe_exc, 5 ) );/*Q(Q_bwe_exc)/Q5(if Q_bwe_exc > 5) */
            }
        }
    }
    ELSE
    {
        autocorr_fx( exc4k, LPC_WHTN_ORDER_WB + 1, R_h, R_l, &Q_R,
        L_FRAME16k / 4, win_flatten_4k_fx, 0, 1 );

        /* Ensure R[0] isn't zero when entering Levinson Durbin */
        R_l[0] = s_max( R_l[0], 1 );
        move16();
        FOR ( i = 1; i <= LPC_WHTN_ORDER_WB; i++ )
        {
            L_tmp = Mpy_32( R_h[i], R_l[i], wac_h[i - 1], wac_l[i - 1] );
            L_Extract( L_tmp, &R_h[i], &R_l[i] );
        }

        E_LPC_lev_dur(R_h, R_l, lpc_whtn, LepsP, LPC_WHTN_ORDER_WB, NULL);

        Copy_Scale_sig( lpc_whtn, lpc_whtn, LPC_WHTN_ORDER_WB+1, sub(norm_s(lpc_whtn[0]),2) );

        fir_fx( exc4k, lpc_whtn, exc4kWhtnd, wht_fil_mem, L_FRAME16k / 4,
        LPC_WHTN_ORDER_WB, 0, 3 );

        /* Ensure pow1 is greater than zero when computing normalization */
        max = 0;
        FOR ( i = 0;  i < L_FRAME16k / 4; i++ )
        {
            excTmp2[i] = abs_s( exc4kWhtnd[i] );
            move16(); /* Q_bwe_exc */
            max = s_max( max, excTmp2[i] );
            move16();
        }

        IF ( max == 0 )
        {
            pow1 = 1;
            move16();
            n1 = 0;
            move16();
        }
        ELSE
        {
            n1 = norm_s( max );
            FOR ( i = 0; i < L_FRAME16k / 4; i++ )
            {
                excTmp2_frac[i] = shl( excTmp2[i], n1 );
                move16(); /* Q14 */
            }
            n1 = sub( sub( 14, n1 ), Q_bwe_exc );
            pow1 = 1;
            FOR ( i = 0;  i < L_FRAME16k / 4; i++ )
            {
                L_tmp = L_mult( excTmp2_frac[i], excTmp2_frac[i] ); /* Q29 */
                pow1 = L_add( pow1, L_shr( L_tmp, 7 ) ); /* Q22 */
            }
        }

        FOR ( i = 0; i < L_FRAME16k / 4; i++ )
        {
            excNoisyEnv[i] = L_add( *mem_csfilt, L_mult( csfilt_num2[0], excTmp2[i] ) );
            move32();  /* Q_bwe_exc+16  */
            *mem_csfilt = Mult_32_16( excNoisyEnv[i], neg_csfilt_den2[1] );
            move32();  /* Q_bwe_exc+16 */
        }

        create_random_vector_fx( exc4k, L_FRAME16k / 4, bwe_seed );

        /* Ensure pow2 is greater than zero when computing normalization */
        Lmax = 0;
        FOR ( i = 0;  i < L_FRAME16k / 4; i++ )
        {
            exc4k_32[i] = Mult_32_16( excNoisyEnv[i], exc4k[i] );
            move32();/* Q_bwe_exc+6 */
            Lmax = L_max( Lmax, L_abs( exc4k_32[i] ) );
        }

        IF ( Lmax == 0 )
        {
            pow2 = 1;
            move16();
            n2 = 0;
            move16();
            set16_fx( exc4k_frac, 0, L_FRAME16k / 4 );
        }
        ELSE
        {
            n2 = norm_l( Lmax );
            FOR ( i = 0; i < L_FRAME16k / 4; i++ )
            {
                exc4k_frac[i] = extract_h( L_shl( exc4k_32[i], n2 ) ); /* Q(14-n2) */
            }
            n2 = 30 - n2 - ( Q_bwe_exc + 6 );
            pow2 = 1;
            FOR ( i = 0;  i < L_FRAME16k / 4; i++ )
            {
                L_tmp = L_mult( exc4k_frac[i], exc4k_frac[i] ); /* Q29 */
                pow2 = L_add( pow2, L_shr( L_tmp, 7 ) ); /* Q22     */
            }
        }

        test();
        test();
        IF( sub(coder_type, UNVOICED) == 0 || ( igf_flag != 0 && sub( avg_voice_fac, 6654 ) < 0 ) )
        {
            L_tmp = root_a_over_b_fx( pow1, sub( 22, shl( n1, 1 ) ), pow2, sub( 22, shl( n2, 1 ) ), &exp );
            scale = round_fx( L_shl( L_tmp, exp ) ); /*Q15 */

            sc = sub( add( n2, Q_bwe_exc ), 14 );
            FOR ( i = 0; i < L_FRAME16k / 4; i++ )
            {
                exc4kWhtnd[i] = round_fx( L_shl( L_mult( exc4k_frac[i], scale ), sc ) ); /*  Q_bwe_exc+n2-10+16+ Q_bwe_exc + n2 -14 -16 = //Q_bwe_exc */
            }
        }
        ELSE
        {
            sc = sub( add( n2, Q_bwe_exc ), 14 ); /* Q_bwe_exc+n2-14*/

            k = 0;
            FOR ( i = 0; i < 4; i++ )
            {
                test();
                IF( igf_flag != 0 && sub(coder_type, VOICED) == 0 )
                {
                    /*tmp_vfac = 2*voice_factors[i];
                      tmp_vfac = min(1, tmp_vfac);*/
                    BASOP_SATURATE_WARNING_OFF
                    tmp_vfac = shl(voice_factors[i], 1);
                    BASOP_SATURATE_WARNING_ON
                }
                ELSE
                {
                    tmp_vfac = voice_factors[i];
                    move16();
                }

                Ltemp1 = root_a_fx( L_deposit_h( tmp_vfac ), 31, &exp );
                temp1 = round_fx( L_shl( Ltemp1, exp ) ); /* Q15 */

                L_tmp = Mult_32_16( pow1, sub( 32767, tmp_vfac ) ); /* Q22*/
                Ltemp2 = root_a_over_b_fx( L_tmp, sub( 22, shl( n1, 1 ) ), pow2, sub( 22, shl( n2, 1 ) ), &exp );
                temp2 = round_fx( L_shl( Ltemp2, exp ) ); /* Q15 */

                FOR ( j = 0; j < L_FRAME16k / 16; j++ )
                {
                    L_tmp = L_mult( temp1, exc4kWhtnd[k] );/* Q(16+Q_bwe_exc) */
                    L_tmp = L_add( L_tmp, L_shl( L_mult( temp2, exc4k_frac[k] ), sc ) ); /* Q(16+Q_bwe_exc) */
                    exc4kWhtnd[k] = round_fx( L_tmp ); /* Q_bwe_exc */
                    k++;
                }
            }
        }
    }

    Syn_filt_s( 0, lpc_shb, LPC_SHB_ORDER_WB, exc4kWhtnd, excSHB, L_FRAME16k / 4, state_lpc_syn, 1 );



    return;
}


/*-------------------------------------------------------------------*
* GenWBSynth()
*
* Generate 16 KHz sampled highband component from synthesized highband
*-------------------------------------------------------------------*/

void GenWBSynth_fx(
    const   Word16* input_synspeech,  /* i : input synthesized speech Qx*/
    Word16* shb_syn_speech_16k,                     /* o : output highband compnent Qx*/
    Word16* state_lsyn_filt_shb1,                   /* i/o: memory Qx*/
    Word16* state_lsyn_filt_shb2                    /* i/o: memory Qx*/ )
{
    Word16 speech_buf_16k1[L_FRAME16k], speech_buf_16k2[L_FRAME16k];
    Word16 i, maxm, nor;
    Word16 input_synspeech_temp[L_FRAME16k / 4];

    maxm = 0;
    move16();
    FOR(i = 0; i<L_FRAME16k / 4; i++)
    maxm = s_max(maxm, abs_s(input_synspeech[i]));
    FOR(i = 0; i<2*ALLPASSSECTIONS_STEEP + 1; i++)
    maxm = s_max(maxm, abs_s(state_lsyn_filt_shb1[i]));
    FOR(i = 0; i<2*ALLPASSSECTIONS_STEEP + 1; i++)
    maxm = s_max(maxm, abs_s(state_lsyn_filt_shb2[i]));

    nor = s_max(sub(norm_s(maxm),3),0); /* Headroom = 3 */
    IF(maxm == 0) nor = 15;
    move16();

    Copy_Scale_sig(input_synspeech, input_synspeech_temp, L_FRAME16k / 4, nor);
    Scale_sig(state_lsyn_filt_shb1, 2*ALLPASSSECTIONS_STEEP + 1, nor);
    Scale_sig(state_lsyn_filt_shb2, 2*ALLPASSSECTIONS_STEEP + 1, nor);
    Interpolate_allpass_steep_fx( input_synspeech_temp, state_lsyn_filt_shb1,  L_FRAME16k / 4, speech_buf_16k1 );
    Interpolate_allpass_steep_fx( speech_buf_16k1, state_lsyn_filt_shb2, L_FRAME16k / 2, speech_buf_16k2 );
    flip_spectrum_fx( speech_buf_16k2, shb_syn_speech_16k, L_FRAME16k );

    Scale_sig(shb_syn_speech_16k, L_FRAME16k, -nor);
    Scale_sig(state_lsyn_filt_shb1, 2*ALLPASSSECTIONS_STEEP + 1, -nor);
    Scale_sig(state_lsyn_filt_shb2, 2*ALLPASSSECTIONS_STEEP + 1, -nor);

    return;
}

/*======================================================================================*/
/* FUNCTION : void GenShapedSHBExcitation_fx () */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Synthesize spectrally shaped highband excitation signal */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) coder_type : coding type Q_bwe_exc */
/* _(Word16) bwidth : input signal bandwidth Q0 */
/* _(Word16*) bwe_exc_extended :bandwidth extended exciatation Q_bwe_exc */
/* _(Word16[]) voice_factors :voicing factors Q15 */
/* _(Word16*) lpc_shb :lpc coefficients Q12 */
/* _(Word16*) Q_bwe_exc :Q Format of bwe_exc_extended */
/* _(Word16) L_frame : Frame length - determines whether 12.8 or 16kHz core */
/* _(Word16) last_L_frame : last L_frame */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)excSHB :synthesized shaped shb excitation Q_bwe_exc */
/* _(Word16*)White_exc16k :white excitation for the Fullband extension Q_bwe_exc */
/* _(Word16*)slope :slope +ve (high freq > low freq), -ve or neutral Q12 */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _(Word16*)mem_csfilt :memory */
/* _(Word16*)mem_genSHBexc_filt_down_shb :memory */
/* _(Word16*)state_lpc_syn :memory */
/* _(Word16[]) bwe_seed :random number generator seed */
/* _(Word16[]) lpf_14k_mem :memory */
/* _(Word32[])Hilbert_Mem :memory */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------------------*/
/* CALLED FROM : RX */
/*======================================================================================*/
void GenShapedSHBExcitation_fx(
    Word16 *excSHB,                                 /* o : synthesized shaped shb excitation Q_bwe_exc*/
    const Word16 *lpc_shb,                                /* i : lpc coefficients Q12*/
    Word16 *White_exc16k_FB,                        /* o : white excitation for the Fullband extension Q_bwe_exc */
    Word32 *mem_csfilt,                             /* i/o: memory */
    Word16 *mem_genSHBexc_filt_down_shb,            /* i/o: memory */
    Word16 *state_lpc_syn,                          /* i/o: memory */
    const  Word16 coder_type,                             /* i : coding type */
    const Word16 *bwe_exc_extended,                       /* i : bandwidth extended excitation */
    Word16 bwe_seed[],                              /* i/o: random number generator seed */
    Word16 voice_factors[],                         /* i : voicing factor*/
    const Word16 FB_flag,                                 /* i : FB flag indicator */
    Word16 *tbe_demph,                              /* i/o: de-emphasis memory */
    Word16 *tbe_premph,                             /* i/o: pre-emphasis memory */
    Word16 *lpc_shb_sf,                             /* i:   LP coefficients  */
    const Word32 shb_ener_sf_32,                          /* i: input shb ener, Q31 */
    Word16 *shb_res_gshape,                         /* i: input res gain shape, Q14 */
    Word16 *shb_res,
    Word16 *vf_ind,
    const Word16 formant_fac,                             /* i   : Formant sharpening factor [0..1] */
    Word16 fb_state_lpc_syn[],                      /* i/o: memory */
    Word16 *fb_tbe_demph,                           /* i/o: fb de-emphasis memory */
    Word16 *Q_bwe_exc,
    Word16 *Q_bwe_exc_fb,
    const Word16 Q_shb
    ,Word16 n_mem2                 /* i :  n_mem2 scale factor to adjust 24.4/32kbps memories */
    ,Word16 prev_Q_bwe_syn         /* i :  st_fx->prev_Q_bwe_syn */
    ,Word32 bitrate
)
{
    Word16 i, j, k;
    Word16 wht_fil_mem[LPC_WHTN_ORDER];
    Word16 lpc_whtn[LPC_WHTN_ORDER + 1];
    Word16 R_h[LPC_WHTN_ORDER+ 2]; /* Autocorrelations of windowed speech MSB */
    Word16 R_l[LPC_WHTN_ORDER+ 2]; /* Autocorrelations of windowed speech LSB */
    Word16 Q_R;
    Word32 LepsP[LPC_WHTN_ORDER+1];
    Word16 exc32k[L_FRAME32k], exc16k[L_FRAME16k];
    Word32 pow1, pow2;
    Word16 scale, temp1, temp2;

    Word16 excTmp2[L_FRAME16k];
    Word16 *White_exc16k;
    Word16 excNoisyEnv[L_FRAME16k];
    Word16 csfilt_num2[1]  = {6554}; /*0.2 in Q15 */
    Word16 neg_csfilt_den2[2] = {-32767, 26214};   /* {1.0f, -0.8f} */
    Word16 varEnvShape;
    Word16 fb_deemph_fac = 15729; /*0.48f in Q15  */
    Word16 exc16kWhtnd[L_FRAME16k];

    Word32 L_tmp;
    Word16 vf_tmp;
    Word16 tmp, exp, tmp2;
    Word16 voiceFacEst[NB_SUBFR16k];
    Word16 zero_mem[LPC_SHB_ORDER];
    Word32 syn_shb_ener_sf[4];
    Word16 tempSHB[80];
    Word16 Q_pow1, Q_pow2;
    Word32 L_tmp2, L_tmp3, L_tmp4;
    Word16 temp;

    Word16 White_exc16k_FB_temp[L_FRAME16k];
    Word32 White_exc16k_32[L_FRAME16k];
    Word16 Q_temp;

    set16_fx( zero_mem, 0, LPC_SHB_ORDER );
    set16_fx( wht_fil_mem, 0, LPC_WHTN_ORDER );

    FOR(i = 0; i < L_FRAME32k; i=i+2)
    {
        exc32k[i] = negate(bwe_exc_extended[i]);
        move16();
        exc32k[i+1] = bwe_exc_extended[i+1];
        move16();
    }

    /* Decimate by 2 */
    Decimate_allpass_steep_fx( exc32k, mem_genSHBexc_filt_down_shb, 2*L_FRAME16k, exc16k );
    /* i: exc32k in Q_bwe_exc */
    /* o: exc16k in Q_bwe_exc */

    autocorr_fx( exc16k, LPC_WHTN_ORDER + 1, R_h, R_l, &Q_R, L_FRAME16k, win_flatten_fx, 0, 1 );
    /* Ensure R[0] isn't zero when entering Levinson Durbin */
    R_l[0] = s_max( R_l[0], 1 );
    move16();
    FOR ( i = 1; i <= LPC_WHTN_ORDER; i++ )
    {
        L_tmp = Mpy_32( R_h[i], R_l[i], wac_h[i - 1], wac_l[i - 1] );
        L_Extract( L_tmp, &R_h[i], &R_l[i] );
    }
    E_LPC_lev_dur(R_h, R_l, lpc_whtn, LepsP, LPC_WHTN_ORDER, NULL);
    Copy_Scale_sig( lpc_whtn, lpc_whtn, LPC_WHTN_ORDER+1, sub(norm_s(lpc_whtn[0]),2) );
    fir_fx( exc16k, lpc_whtn, exc16kWhtnd, wht_fil_mem, L_FRAME16k,  LPC_WHTN_ORDER, 0, 3 );

    /* i: exc16k in Q_bwe_exc */
    /* o: exc16kWhtnd in Q_bwe_exc */

    IF( L_sub(bitrate, ACELP_24k40) >= 0)
    {
        temp2 = 0;
        move16();
        FOR(j = 0; j < 4; j++)
        {
            temp1 = shb_res_gshape[j];
            move16();
            FOR(i = 0; i < 80; i++)
            {
                exc16kWhtnd[temp2 + i] = round_fx(L_shl(L_mult( exc16kWhtnd[temp2 + i], temp1 ), 1));
                /* exc16kWhtnd in Q_bwe_exc, shb_res_gshape in Q14 */
            }
            temp2 = add(temp2, 80);
        }
    }

    /* Estimate pow1 associated with Low band nonlinear extended excitation */
    /* pow1=0.00001f */
    tmp = sub(shl(*Q_bwe_exc, 1), 31);
    pow1 = L_shl(FL2WORD32(0.00001f), tmp);   /* 0.00001f in 2*(Q_bwe_exc) */
    FOR( k = 0; k < L_FRAME16k; k++)
    {
        /*excTmp2[k ] = (float)(fabs(exc16kWhtnd[k]));*/
        excTmp2[k] = abs_s( exc16kWhtnd[k] );
        move16();

        /* pow1 += exc16kWhtnd[k] * exc16kWhtnd[k]; */
        pow1 = L_mac0( pow1, exc16kWhtnd[k], exc16kWhtnd[k] );  /* 2*Q_bwe_exc */
    }
    Q_pow1 = shl(*Q_bwe_exc,1);

    test();
    IF( (L_sub( bitrate, ACELP_13k20 ) == 0) ||  (L_sub( bitrate, ACELP_9k60 ) == 0) )
    {
        /* varEnvShape = mean_fx(voice_factors, 4); */
        /* unroll the loop */
        L_tmp = L_mult(voice_factors[0], 8192);
        L_tmp = L_mac(L_tmp, voice_factors[1], 8192 );
        L_tmp = L_mac(L_tmp, voice_factors[2], 8192 );
        varEnvShape   = mac_r(L_tmp, voice_factors[3], 8192 );  /* varEnvShape in Q15 */
    }
    ELSE /* 16k core */
    {
        /* varEnvShape = mean_fx(voice_factors, 5); */
        /* unroll the loop */
        L_tmp = L_mult(voice_factors[0], 6554);
        L_tmp = L_mac(L_tmp, voice_factors[1], 6554 );
        L_tmp = L_mac(L_tmp, voice_factors[2], 6554 );
        L_tmp = L_mac(L_tmp, voice_factors[3], 6554 );
        varEnvShape = mac_r(L_tmp, voice_factors[4], 6554 );  /* varEnvShape in Q15 */
    }

    IF ( sub(FB_flag, FB_TBE) == 0 )
    {
        /*pow(varEnvShape,3) */
        tmp = mult_r(varEnvShape, varEnvShape);
        tmp = mult_r(tmp, varEnvShape);

        /* max((0.68f - (float)pow(varEnvShape, 3)), 0.48f); */
        fb_deemph_fac = sub(FL2WORD16(0.68f), tmp);
        fb_deemph_fac = max(fb_deemph_fac, FL2WORD16(0.48f));
    }

    /*varEnvShape = 1.09875f - 0.49875f * varEnvShape; */
    varEnvShape = msu_r(FL2WORD32(0.549375f), FL2WORD16(0.249375f), varEnvShape);

    /*varEnvShape = min( max(varEnvShape, 0.6f), 0.999f); */
    varEnvShape = s_max(varEnvShape, FL2WORD16(0.3f));
    varEnvShape = s_min(varEnvShape, FL2WORD16(0.4995f));
    varEnvShape = shl(varEnvShape, 1);
    csfilt_num2[0] = sub(MAX_16, varEnvShape);
    move16();
    neg_csfilt_den2[1] = varEnvShape;
    move16();

    test();
    test();
    test();
    IF ( *mem_csfilt == 0 && ( (L_sub( bitrate, ACELP_9k60 ) == 0) || (L_sub( bitrate, ACELP_16k40 ) == 0) || (L_sub( bitrate, ACELP_24k40 ) == 0) ) )
    {
        /* pre-init smoothing filter to avoid energy drop outs */
        L_tmp = L_mult(excTmp2[0], 1638);
        FOR (i = 1; i < L_SUBFR16k/4; i++)
        {
            L_tmp = L_mac(L_tmp, excTmp2[i], 1638); /*1638 = 1/20 in Q15*/
        }
        /*L_tmp = sum(excTmp2, L_SUBFR16k/4)*(1/20) where L_SUBFR16k/4 =20 */


        /* use weak smoothing for 1st frame after switching to make filter recover more quickly */
        varEnvShape = FL2WORD16(0.8f);
        move16();
        csfilt_num2[0] = sub(MAX_16, varEnvShape);
        move16();
        neg_csfilt_den2[1] = varEnvShape;
        move16();

        *mem_csfilt = Mult_32_16( L_tmp, varEnvShape );
        move32();
    }

    /* Track the low band envelope */
    L_tmp = *mem_csfilt;
    move32();
    FOR ( i = 0; i < L_FRAME16k; i++ )
    {
        excNoisyEnv[i] = mac_r( L_tmp, csfilt_num2[0], excTmp2[i] );
        move16();
        /* excNoisyEnv : Q_bwe_exc,
         *mem_csfilt: Q_bwe_exc+16, excTmp2: Q_bwe_exc, csfilt_num2[0] Q15  */
        L_tmp = L_mult( excNoisyEnv[i], neg_csfilt_den2[1] );     /* Q_bwe_exc+16 */
    }
    *mem_csfilt = L_tmp;
    move32();

    /* create a random excitation - Reuse exc16k memory */
    White_exc16k = exc16k;
    create_random_vector_fx( White_exc16k, 256, bwe_seed );
    create_random_vector_fx( White_exc16k + 256, L_FRAME16k - 256, bwe_seed );
    L_tmp = L_deposit_l(0);
    tmp = add(*Q_bwe_exc, 1);
    FOR ( k = 0; k < L_FRAME16k; k++ )
    {
        L_tmp4 = L_shl(L_deposit_l(White_exc16k[k]), tmp);
        if(excNoisyEnv[k] != 0)
        {
            L_tmp4 = L_mult(excNoisyEnv[k], White_exc16k[k]);/* (Q_bwe_exc)  +5 +1*/
        }
        White_exc16k_32[k] = L_tmp4;
        move32();
        L_tmp = L_max(L_tmp, White_exc16k_32[k]);
    }
    Q_temp = norm_l( L_tmp );
    if(L_tmp == 0)
    {
        Q_temp = 31;
    }
    /*Copy_Scale_sig( White_exc16k, White_exc16k, L_FRAME16k, sub(NOISE_QFAC, 5) );)*/
    /* White_exc16k in Q6 */

    /* calculate pow2 */
    /* pow2=0.00001f */
    tmp = sub(shl(sub(*Q_bwe_exc, NOISE_QADJ), 1), 31);
    pow2 = L_shl(FL2WORD32(0.00001f), tmp);   /* 0.00001f in 2*(Q_bwe_exc-NOISE_QADJ) */
    tmp = sub(NOISE_QFAC, 5);
    FOR ( k = 0; k < L_FRAME16k; k++ )
    {
        /* White_exc16k[k] *= excNoisyEnv[k]; */
        White_exc16k[k] = mult_r( excNoisyEnv[k], shl(White_exc16k[k], tmp) );
        move16();
        /* i: excNoisyEnv in (Q_bwe_exc)     */
        /* i: White_exc16k  in Q6               */
        /* o: White_exc16k in (Q_bwe_exc-NOISE_QADJ)  */
        /* pow2 += White_exc16k[k] * White_exc16k[k]; */
        pow2 = L_mac0( pow2, White_exc16k[k], White_exc16k[k] );  /* 2*(Q_bwe_exc-NOISE_QADJ)*/
    }
    /*Q_pow2 = sub( shl(*Q_bwe_exc,1), 18 );*/
    Q_pow2 = shl( sub( *Q_bwe_exc, NOISE_QADJ ), 1);


    IF( L_sub(bitrate, ACELP_24k40) >= 0)
    {
        IF( sub(*vf_ind,20) == 0) /* encoder side */
        {
            Estimate_mix_factors_fx(shb_res, Q_shb, exc16kWhtnd, *Q_bwe_exc,
                                    White_exc16k, (*Q_bwe_exc-NOISE_QADJ), pow1, Q_pow1,
                                    pow2, Q_pow2, voiceFacEst, vf_ind);
            tmp = voiceFacEst[0];
        }
        ELSE /* decoder side */
        {
            /* *vf_ind is an integer scale by 0.125f*/
            tmp = shl( *vf_ind, (15-3) );
        }
        tmp2 = MAX_16;
        if( sub(tmp, FL2WORD16(0.7f)) <= 0)
        {
            tmp2 = FL2WORD16(0.8f);
        }
        voice_factors[0] = mult_r(voice_factors[0], tmp2);
        move16();
        voice_factors[1] = mult_r(voice_factors[1], tmp2);
        move16();
        voice_factors[2] = mult_r(voice_factors[2], tmp2);
        move16();
        voice_factors[3] = mult_r(voice_factors[3], tmp2);
        move16();
        voice_factors[4] = mult_r(voice_factors[4], tmp2);
        move16();
    }

    FOR ( k = 0; k < L_FRAME16k; k++ )
    {
        White_exc16k_FB[k] = round_fx(L_shl(White_exc16k_32[k],Q_temp)); /* Q_bwe_exc +5 +1 +Q_temp -16 */
    }
    *Q_bwe_exc_fb = sub(add(*Q_bwe_exc,Q_temp),10);

    deemph_fx( White_exc16k, PREEMPH_FAC, L_FRAME16k, tbe_demph );
    /* i/o: White_exc16k (Q_bwe_exc-NOISE_QADJ) */
    /* i: tbe_demph (Q_bwe_exc-NOISE_QADJ) */

    IF ( sub(coder_type, UNVOICED)  == 0 )
    {
        L_tmp = root_a_over_b_fx( pow1, Q_pow1, pow2, Q_pow2, &exp );
        scale = round_fx( L_shl( L_tmp, exp ) );       /*Q15 */

        FOR ( k = 0; k < L_FRAME16k; k++ )
        {
            /* White_exc16k: (Q_bwe_exc-NOISE_QADJ), scale: Q15 */
            L_tmp = L_mult( White_exc16k[k], scale );
            /* L_tmp: (Q_bwe_exc-NOISE_QADJ) + 15 + 1 */
            exc16kWhtnd[k] = round_fx( L_shl(L_tmp, NOISE_QADJ) );
            /* exc16kWhtnd:  Q_bwe_exc */
        }
        preemph_fx( exc16kWhtnd, PREEMPH_FAC, L_FRAME16k, tbe_premph );
        /* i/o: exc16kWhtnd  (Q_bwe_exc) */
        /* i/o: tbe_premph (Q_bwe_exc) */
    }
    ELSE
    {
        Word16 nbSubFr, lSubFr;
        Word16 tempQ15;
        Word32 tempQ31;
        /*nbSubFr = ( bitrate < ACELP_24k40 )? NB_SUBFR : NB_SUBFR16k;*/
        nbSubFr = NB_SUBFR16k;
        lSubFr = (L_FRAME16k/NB_SUBFR16k);
        IF(L_sub(bitrate, ACELP_24k40) < 0)
        {
            nbSubFr = NB_SUBFR;
            move16();
            lSubFr = (L_FRAME16k/NB_SUBFR);
            move16();
        }
        k = 0;
        FOR( i = 0; i < nbSubFr; i++ )
        {
            test();
            IF( sub(coder_type, VOICED) == 0 && ( L_sub(bitrate, ACELP_24k40) < 0 ) )
            {
                exp = 0;
                tempQ15 = Sqrt16(voice_factors[i], &exp);  /* Q15 */
                temp = shl(tempQ15, exp);                   /* Q15 exc16kWhtnd scale factor */
                exp = 0;
                tempQ15 = Sqrt16(temp, &exp);  /* Q15 */
                temp1 = shl(tempQ15, exp);                   /* Q15 exc16kWhtnd scale factor */

                /*temp2 = root_a_over_b_fx( pow1 * (1.0f - temp), pow2 ); */
                temp = sub(MAX_16, temp);
                tempQ31 = Mult_32_16(pow1, temp);
                L_tmp = root_a_over_b_fx( tempQ31, Q_pow1, pow2, Q_pow2, &exp );
                temp2 = round_fx(L_shl(L_tmp, exp));    /* Q15 whiteEnvShapedExc scale factor */
            }
            ELSE
            {
                /* Adjust noise mixing for formant sharpening filter */
                tempQ15 = mult_r(SWB_NOISE_MIX_FAC_FX, formant_fac);
                /* vf_tmp = voice_factors[i] * (1.0f - vf_tmp); */
                vf_tmp = sub(MAX_16, tempQ15);
                vf_tmp = mult_r(voice_factors[i], vf_tmp);

                exp = 0;
                tempQ15 = Sqrt16(vf_tmp, &exp);  /* Q15 */
                temp1 = shl(tempQ15, exp);                   /* Q15 exc16kWhtnd scale factor */

                /*temp2 = root_a_over_b(pow1 * (1.0f - vf_tmp), pow2); */
                temp = sub(MAX_16, vf_tmp);
                tempQ31 = Mult_32_16(pow1, temp);
                L_tmp = root_a_over_b_fx( tempQ31, Q_pow1, pow2, Q_pow2, &exp );
                temp2 = round_fx(L_shl(L_tmp, exp));    /* Q15 whiteEnvShapedExc scale factor */
            }

            FOR( j = 0; j < lSubFr; j++)
            {
                /*exc16kWhtnd[k+j] = temp1 * exc16kWhtnd[k+j] + temp2 * White_exc16k[k+j]; */
                L_tmp = L_mult(temp2, White_exc16k[k+j]);  /* 16+(Q_bwe_exc-NOISE_QADJ)*/
                L_tmp = L_shl(L_tmp, NOISE_QADJ);   /* 16+(Q_bwe_exc) */
                exc16kWhtnd[k+j] = mac_r(L_tmp, temp1, exc16kWhtnd[k+j]);
                move16();
                /* Q_bwe_exc */
            }
            k = add(k, lSubFr);

            /* estimate the pre-emph factor */
            tempQ15 = sub(MAX_16, voice_factors[i]);
            exp = 0;
            temp = Sqrt16(tempQ15, &exp);
            temp = shl(temp, exp-1);

            temp2 = add( temp, shl(temp1, -1) );   /* shift right by 1 to avoid overflow */
            temp = div_s( temp, temp2 ); /* Q15 */
            temp = mult_r( PREEMPH_FAC, temp );

            preemph_fx( &exc16kWhtnd[i*lSubFr], temp, lSubFr, tbe_premph );
            /* exc16kWhtnd: Q_bwe_exc;
               tbe_premph: Q_bwe_exc*/
        }
    }


    IF ( L_sub(bitrate, ACELP_24k40) < 0 )
    {
        Syn_filt_s(0, lpc_shb, LPC_SHB_ORDER, exc16kWhtnd, excSHB, L_FRAME16k, state_lpc_syn, 1 );
        /* i: exc16kWhtnd in Q_bwe_exc */
        /* o: excSHB in Q_bwe_exc */
    }
    ELSE
    {
        set16_fx( zero_mem, 0, LPC_SHB_ORDER);

        Syn_filt_s(0, lpc_shb_sf, LPC_SHB_ORDER, exc16kWhtnd, tempSHB, 80, zero_mem, 1);
        syn_shb_ener_sf[0] = L_shr(sum2_fx(tempSHB, 80),3);
        move32();

        Syn_filt_s(0, lpc_shb_sf+(LPC_SHB_ORDER+1),   LPC_SHB_ORDER, exc16kWhtnd+ 80, tempSHB, 80, zero_mem, 1 );
        syn_shb_ener_sf[1] = L_shr(sum2_fx(tempSHB, 80),3);
        move32();

        Syn_filt_s(0, lpc_shb_sf+2*(LPC_SHB_ORDER+1), LPC_SHB_ORDER, exc16kWhtnd+160, tempSHB, 80, zero_mem, 1 );
        syn_shb_ener_sf[2] = L_shr(sum2_fx(tempSHB, 80),3);
        move32();

        Syn_filt_s(0, lpc_shb_sf+3*(LPC_SHB_ORDER+1), LPC_SHB_ORDER, exc16kWhtnd+240, tempSHB, 80, zero_mem, 1 );
        syn_shb_ener_sf[3] = L_shr(sum2_fx(tempSHB, 80),3);
        move32();

        /* i: exc16kWhtnd       in Q_bwe_exc        */
        /* o: tempSHB           in Q_bwe_exc        */
        /* o: syn_shb_ener_sf   in (2*Q_bwe_exc+1)  */

        L_tmp = sum32_fx(syn_shb_ener_sf, 4);

        /* find root_a(tempSHB[0]) = root_a_over_b(shb_ener_sf[0]), L_tmp) */
        tmp = shl(Q_shb, 1);
        tmp2 = add(shl(*Q_bwe_exc, 1), 1);
        L_tmp2 = root_a_over_b_fx(shb_ener_sf_32, tmp, L_tmp, tmp2, &exp); /* L_tmp2 in (Q31-exp) */

        *Q_bwe_exc = sub(*Q_bwe_exc, exp);
        move16(); /* compensate for the exp shift */
        tmp2 = add( prev_Q_bwe_syn, n_mem2 );
        if( sub( *Q_bwe_exc, tmp2) > 0 )
        {
            *Q_bwe_exc = tmp2;
            move16();
        }
        FOR(i = 0; i < L_FRAME16k; i++)
        {
            L_tmp3 = Mult_32_16(L_tmp2, exc16kWhtnd[i]);   /* *Q_bwe_exc + (31-exp) - 15 */
            exc16kWhtnd[i] = round_fx(L_tmp3);   /* *Q_bwe_exc - exp */
        }
        /* i: L_tmp2 in (Q31-exp)                       */
        /* i: exc16kWhtnd in Q_bwe_exc            */
        /* o: exc16kWhtnd in Q_bwe_exc: (Q_bwe_exc-exp)   */

        /* Rescale the past memories: LP synth and SHB look ahead buffers */
        tmp = sub(*Q_bwe_exc, prev_Q_bwe_syn);
        FOR( i = 0; i < LPC_SHB_ORDER; i++ )
        {
            state_lpc_syn[i] = shl( state_lpc_syn[i], tmp );
            move16();
        }
        FOR( i = -L_SHB_LAHEAD;  i < 0; i++ )
        {
            excSHB[i] = shl(excSHB[i], tmp);
            move16();
        }
        /* Do mem_stp_swb_fx scaling before PostShortTerm_fx */

        Syn_filt_s(0, lpc_shb_sf,                     LPC_SHB_ORDER, exc16kWhtnd,     excSHB,     80, state_lpc_syn, 1 );
        Syn_filt_s(0, lpc_shb_sf+(LPC_SHB_ORDER+1),   LPC_SHB_ORDER, exc16kWhtnd+ 80, excSHB+ 80, 80, state_lpc_syn, 1 );
        Syn_filt_s(0, lpc_shb_sf+2*(LPC_SHB_ORDER+1), LPC_SHB_ORDER, exc16kWhtnd+160, excSHB+160, 80, state_lpc_syn, 1 );
        Syn_filt_s(0, lpc_shb_sf+3*(LPC_SHB_ORDER+1), LPC_SHB_ORDER, exc16kWhtnd+240, excSHB+240, 80, state_lpc_syn, 1 );
        /* i: exc16kWhtnd in (Q_bwe_exc) */
        /* o: excSHB in (Q_bwe_exc) */
    }

    IF ( sub(FB_flag, FB_TBE) == 0)
    {
        Syn_filt_s(0, lpc_shb, LPC_SHB_ORDER, White_exc16k_FB, White_exc16k_FB_temp, L_FRAME16k, fb_state_lpc_syn, 1 );
        /* i: White_exc16k_FB       in (14-n2) */
        /* o: White_exc16k_FB_temp  in (14-n2) */

        FOR( i=0; i<10; i++ )
        {
            FOR( j=0; j<32; ++j )
            {
                White_exc16k_FB_temp[i*32+j] = mult_r(White_exc16k_FB_temp[i*32+j], cos_fb_exc_fx[j]);
                move16();
            }
        }

        *Q_bwe_exc_fb = add(*Q_bwe_exc_fb, 20);
        move16(); /**Q_bwe_exc_fb  +35 +1 -16*/
        flip_spectrum_fx( White_exc16k_FB_temp, White_exc16k_FB, L_FRAME16k );

        deemph_fx( White_exc16k_FB, fb_deemph_fac, L_FRAME16k, fb_tbe_demph );
    }
    ELSE
    {
        set16_fx( White_exc16k_FB, 0, L_FRAME16k);
    }

    return;
}


/*====================================================================================*/
/* FUNCTION : void GenSHBSynth_fx() */
/*------------------------------------------------------------------------------------*/
/* PURPOSE :Generate 32 KHz sampled highband component from synthesized highband*/
/*------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS */
/* _(Word16*)input_synspeech :input synthesized speech */
/* _(Word16) L_frame :ACELP frame length */
/*------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)shb_syn_speech_32k : output highband component */
/*------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _(Word16[]) allpass_mem : memory */
/* _(Word32[]) Hilbert_Mem : memory */
/*------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------------------------------*/

void GenSHBSynth_fx(
    const   Word16* input_synspeech,                /* i : input synthesized speech */
    Word16* shb_syn_speech_32k,             /* o : output highband component */
    Word32 Hilbert_Mem[],                   /* i/o: memory */
    Word16 allpass_mem[],                   /* i/o: memory */
    const   Word16 L_frame,                         /* i : ACELP frame length */
    Word16* syn_dm_phase
)
{
    Word16 i, speech_buf_32k[L_FRAME32k];
    Word16 maxm, nor, nor32, shift;
    Word16 input_synspeech_temp[L_FRAME16k];
    Word32 maxm32;


    maxm = 0;
    move16();
    maxm32 = L_deposit_l(0);
    FOR(i = 0; i<L_FRAME16k; i++)
    maxm = s_max(maxm, abs_s(input_synspeech[i]));
    FOR(i = 0; i<2*ALLPASSSECTIONS_STEEP + 1; i++)
    maxm = s_max(maxm, abs_s(allpass_mem[i]));
    FOR(i = 0; i<HILBERT_MEM_SIZE; i++)
    maxm32 = L_max(maxm32, L_abs(Hilbert_Mem[i]));
    nor = s_max(sub(norm_s(maxm),3),0);
    nor32 = s_max(sub(norm_l(maxm32),3),0);
    if(maxm == 0)
    {
        nor = 15;
        move16();
    }
    if(maxm32 == 0)
    {
        nor32 = 31;
        move16();
    }
    shift = s_min(nor, nor32);

    Copy_Scale_sig(input_synspeech, input_synspeech_temp, L_FRAME16k, shift);
    Scale_sig(allpass_mem, 2*ALLPASSSECTIONS_STEEP + 1, shift);
    Scale_sig32(Hilbert_Mem, HILBERT_MEM_SIZE, shift);
    Interpolate_allpass_steep_fx( input_synspeech_temp, allpass_mem, L_FRAME16k, speech_buf_32k );
    /*modify_Fs_fx( input_synspeech, L_FRAME16k, 16000, speech_buf_32k, 32000, allpass_mem, 0);*/
    IF ( L_frame == L_FRAME )
    {
        /* 12.8 k core flipping and downmixing */
        flip_and_downmix_generic_fx(speech_buf_32k, shb_syn_speech_32k, L_FRAME32k,
                                    Hilbert_Mem,
                                    Hilbert_Mem + HILBERT_ORDER1,
                                    Hilbert_Mem + (HILBERT_ORDER1+2*HILBERT_ORDER2),
                                    syn_dm_phase );
    }
    ELSE
    {
        /* 16 k core flipping and no downmixing */
        FOR(i = 0; i < L_FRAME32k; i=i+2)
        {
            shb_syn_speech_32k[i] = negate(speech_buf_32k[i]);
            move16();
            shb_syn_speech_32k[i+1] = speech_buf_32k[i+1];
            move16();
        }
    }

    Scale_sig(shb_syn_speech_32k, L_FRAME32k, -shift);
    Scale_sig(allpass_mem, 2*ALLPASSSECTIONS_STEEP + 1, -shift);
    Scale_sig32(Hilbert_Mem, HILBERT_MEM_SIZE, -shift);

    return;

}




/*==============================================================================*/
/* FUNCTION : void ScaleShapedSHB_fx() */
/*------------------------------------------------------------------------------*/
/* PURPOSE : */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) length : SHB overlap length */
/* _(Word16*) subgain : subframe gain Q15 */
/* _(Word32) frame_gain : frame gain Q18 */
/* _(Word16*) win : window Q15 */
/* _(Word16*) subwin : subframes window Q15 */
/* _(Word16) Q_bwe_exc : Q format */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16) Qx : Q factor of output */
/*------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _(Word16*) synSHB : synthesized shb signal input Q_bwe_exc / output Qx */
/* _(Word16*) overlap : buffer for overlap-add Q_bwe_exc /output Qx */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------------------------*/
/* CALLED FROM : RX */
/*==============================================================================*/


void ScaleShapedSHB_fx(
    const   Word16 length,          /* i    : SHB overlap length */
    Word16* synSHB,         /* i/o  : synthesized shb signal Q_bwe_exc/Qx   */
    Word16* overlap,        /* i/o  : buffer for overlap-add Q_bwe_exc/Qx   */
    const   Word16* subgain,        /* i    : subframe gain Q15*/
    const   Word32 frame_gain,      /* i    : frame gain Q18                        */
    const   Word16* win,            /* i    : window Q15*/
    const   Word16* subwin,         /* i    : subframes window Q15*/
    Word16 *Q_bwe_exc
    ,Word16* Qx              /* o    : newly computed Q factor for  synSHB   */
    ,Word16 n_mem3
    ,Word16 prev_Q_bwe_syn2
)
{
    const Word16* skip;
    Word16 i, j, k, l_shb_lahead, l_frame, l_frame_tmp;
    Word16 join_length, num_join;
    Word32 mod_syn[L_FRAME16k+L_SHB_LAHEAD];
    Word16 sum_gain;
    Word32 L_tmp;
    Word16 tmpQ15;
    Word16 Q_gFr_norm, gain_frame_Q16;
    Word32 L_tmp2;
    /* Init */
    set32_fx( mod_syn, 0, L_FRAME16k+L_SHB_LAHEAD );

    /* apply gain for each subframe, and store noise output signal using overlap-add */
    IF ( length == SHB_OVERLAP_LEN / 2 )
    {
        /* WB Gain shape and gain frame application with overlap */
        skip = skip_bands_WB_TBE;
        move16();
        l_frame = L_FRAME16k / 4;
        move16();
        l_shb_lahead = L_SHB_LAHEAD / 4;
        move16();

        sum_gain = 0;
        FOR ( k = 0; k < length / 2; k++ )
        {
            sum_gain = mult_r( subwin[2 * k + 2], subgain[0] ); /* Q15 */
            mod_syn[skip[0] + k] = L_mult( sum_gain, synSHB[skip[0] + k] ); /* Q_bwe_exc + 16 */  move32();
            mod_syn[skip[0] + k + length / 2] = L_mult( subgain[0], synSHB[skip[0] + k + length / 2] ); /* Q_bwe_exc + 16 */  move32();
        }
        FOR ( i = 1; i < NUM_SHB_SUBFR / 2; i++ )
        {
            FOR ( k = 0; k < length; k++ )
            {
                /* one bit headroom here, otherwise messes up the gain shape application */
                /* keep it as L_mult0 */
                L_tmp = L_mult0( subwin[k + 1], subgain[i] ); /* Q30 */
                sum_gain = round_fx( L_mac0( L_tmp, subwin[length - k - 1], subgain[i - 1] ) ); /* Q14 */
                mod_syn[skip[i] + k] = L_shl( L_mult( sum_gain, synSHB[skip[i] + k] ), 1 );     /* Q_bwe_exc + 16 */  move32();
            }
        }
        FOR ( k = 0; k < length / 2; k++ )
        {
            sum_gain = mult_r( subwin[length - 2 * k - 2], subgain[i - 1] ); /* Q15 */
            mod_syn[skip[i] + k] = L_mult( sum_gain, synSHB[skip[i] + k] );  /* Q_bwe_exc + 16 */ move32();
        }
    }
    ELSE
    {
        /* SWB Gain shape and gain frame application with overlap */
        l_frame = L_FRAME16k;
        move16();
        l_shb_lahead = L_SHB_LAHEAD;
        move16();
        skip = skip_bands_SWB_TBE;
        move16();

        num_join = NUM_SHB_SUBFR / NUM_SHB_SUBGAINS;
        move16();
        join_length = i_mult2( num_join, length );
        j = 0;
        FOR ( k = 0; k < length; k++ )
        {
            sum_gain = mult_r( subwin[k + 1], subgain[0] );         /* Q15 */
            mod_syn[j] = L_mult( synSHB[j], sum_gain );
            move32();   /* Q_bwe_exc + 16 */
            j++;
        }

        FOR ( i = 0; i < NUM_SHB_SUBGAINS - 1; i++ )
        {
            FOR ( k = 0; k < join_length - length; k++ )
            {
                mod_syn[j] = L_mult( synSHB[j], subgain[i * num_join] );
                move32(); /* Q_bwe_exc + 16 */
                j++;
            }

            FOR ( k = 0; k < length; k++ )
            {
                /* one bit headroom here, otherwise messes up the gain shape application */
                /* keep it as L_mult0 */
                L_tmp = L_mult0( subwin[k + 1], subgain[( i + 1 ) * num_join] ); /* Q30 */
                sum_gain = round_fx( L_mac0( L_tmp, subwin[length - k - 1], subgain[i * num_join] ) );/*Q14 */
                mod_syn[j] = L_shl( L_mult( sum_gain, synSHB[j] ), 1 );
                move32(); /* Q_bwe_exc + 16 */
                j++;
            }
        }
        FOR ( k = 0; k < join_length - length; k++ )
        {
            mod_syn[j] = L_mult( synSHB[j], subgain[( NUM_SHB_SUBGAINS - 1 ) * num_join] );
            move32(); /* Q_bwe_exc + 16 */
            j++;
        }
        FOR ( k = 0; k < length; k++ )
        {
            sum_gain = mult_r( subwin[length - k - 1], subgain[( NUM_SHB_SUBGAINS - 1 ) *num_join] ); /* Q15 */
            mod_syn[j] = L_mult( synSHB[j], sum_gain );
            move32(); /* Q_bwe_exc + 16 */
            j++;
        }
    }


    Q_gFr_norm = norm_l(frame_gain);
    if(frame_gain == 0)  Q_gFr_norm = 31;
    Q_gFr_norm = sub(Q_gFr_norm, 1);   /* give some headroom */

    gain_frame_Q16 = round_fx(L_shl(frame_gain, Q_gFr_norm)); /* Q = 18 + Q_gFr_norm - 16
                                                                   = (Q_gFr_norm + 2)    */

    *Q_bwe_exc = add(*Q_bwe_exc, Q_gFr_norm); /* compensate for the exp shift */
    *Q_bwe_exc = sub(*Q_bwe_exc, 13);    /* Keep Q-fac at => (Q_bwe_exc + Q_gFr_norm - 13) */

    /* check for headroom of previous buff memories: overlap, Hilbert, and interp all-pass mem */
    tmpQ15 = add( prev_Q_bwe_syn2, n_mem3 );
    if( sub( *Q_bwe_exc, tmpQ15) > 0 )
    {
        *Q_bwe_exc = tmpQ15;
        move16();
    }

    *Qx = *Q_bwe_exc;

    /* rescale the overlap memory */
    FOR( i = 0; i < L_SHB_LAHEAD; i++ )
    {
        overlap[i] = shl( overlap[i], (*Q_bwe_exc - prev_Q_bwe_syn2) );
        move16();   /* Q_bwe_exc + Q_gFr_norm - 13 */
    }

    FOR ( i = 0; i < l_shb_lahead; i++ )
    {
        L_tmp = Mult_32_16(mod_syn[i], gain_frame_Q16); /* Q_bwe_exc + 16 + Q_gFr_norm + 2 - 15 */
        L_tmp2 = Mult_32_16(L_tmp, win[i]);             /* (Q_bwe_exc + 16 + Q_gFr_norm + 2 - 15) + 15 + (1-16) */
        synSHB[i] = mac_r(L_tmp2, overlap[i], MAX_16);
        move16(); /* Q_bwe_exc + Q_gFr_norm - 13 */
        synSHB[i+l_shb_lahead] = round_fx(L_tmp);       /* Q_bwe_exc + Q_gFr_norm - 13 */
    }

    FOR ( ; i < l_frame; i++ )
    {
        L_tmp = Mult_32_16( mod_syn[i], gain_frame_Q16); /* Q_bwe_exc + 16 + Q_gFr_norm + 2 - 15 */
        synSHB[i] = round_fx( L_tmp );                   /* Q_bwe_exc + Q_gFr_norm - 13 */
    }

    l_frame_tmp = add(l_frame, l_shb_lahead);
    FOR ( ; i < l_frame_tmp; i++ )
    {
        L_tmp = Mult_32_16( mod_syn[i], gain_frame_Q16); /* Q_bwe_exc + 16 + Q_gFr_norm + 2 - 15 */
        L_tmp = Mult_32_16(L_tmp, win[l_frame +  l_shb_lahead - 1 - i]);  /* (Q_bwe_exc + 16 + Q_gFr_norm + 2 - 15) + 15 + (1-16) */
        overlap[i - l_frame] = round_fx( L_tmp );            /* Q_bwe_exc + Q_gFr_norm - 13 */
    }

    return;
}


void ScaleShapedWB_fx(
    const   Word16 length,          /* i    : SHB overlap length */
    Word16* synSHB,         /* i/o  : synthesized shb signal Q_bwe_exc/Qx   */
    Word16* overlap,        /* i/o  : buffer for overlap-add Q_bwe_exc/Qx   */
    const   Word16* subgain,        /* i    : subframe gain Q15*/
    const   Word32 frame_gain,      /* i    : frame gain Q18                        */
    const   Word16* win,            /* i    : window Q15*/
    const   Word16* subwin,         /* i    : subframes window Q15*/
    const   Word16 Q_bwe_exc
    ,Word16 L_frame          /* i : Frame length - determines whether 12.8 or 16kHz core in-use */
    ,Word16 dynQ             /* i    : indicate whether output is dynamic Q, or Q0 */
    ,Word16* Qx              /* o    : newly computed Q factor for  synSHB   */
    ,Word16 prev_Qx          /* i    : prev_Qx for memory scaling            */
    ,Word32* Hilbert_Mem     /* i    : Hilbert memory used for computing Qx  */
)
{
    const Word16* skip;
    Word16 i, j, k, l_shb_lahead, l_frame, l_frame_tmp;
    Word16 join_length, num_join;
    Word32 mod_syn[L_FRAME16k+L_SHB_LAHEAD];
    Word16 sum_gain;
    Word32 L_tmp;
    Word16 max, abs_sig, sc1, sc2, shift, max_headroom, min_shift, max_shift, max_shift2;
    /* Init */
    set32_fx( mod_syn, 0, L_FRAME16k+L_SHB_LAHEAD );

    /* apply gain for each subframe, and store noise output signal using overlap-add */
    IF ( sub(length,SHB_OVERLAP_LEN / 2 ) == 0)
    {
        /* WB Gain shape and gain frame application with overlap */
        skip = skip_bands_WB_TBE;
        move16();
        l_frame = L_FRAME16k / 4;
        move16();
        l_shb_lahead = L_SHB_LAHEAD / 4;
        move16();

        sum_gain = 0;
        move16();
        FOR ( k = 0; k < length / 2; k++ )
        {
            sum_gain = mult_r( subwin[2 * k + 2], subgain[0] ); /* Q15 */
            mod_syn[skip[0] + k] = L_mult( sum_gain, synSHB[skip[0] + k] );
            move32();/* Q_bwe_exc + 16 */
            mod_syn[skip[0] + k + length / 2] = L_mult( subgain[0], synSHB[skip[0] + k + length / 2] );
            move32();/* Q_bwe_exc + 16 */
        }
        FOR ( i = 1; i < NUM_SHB_SUBFR / 2; i++ )
        {
            FOR ( k = 0; k < length; k++ )
            {
                L_tmp = L_mult0( subwin[k + 1], subgain[i] ); /* Q30 */
                sum_gain = round_fx( L_mac0( L_tmp, subwin[length - k - 1], subgain[i - 1] ) ); /* Q14 */
                mod_syn[skip[i] + k] = L_shl( L_mult( sum_gain, synSHB[skip[i] + k] ), 1 );
                move32();   /* Q_bwe_exc + 16 */
            }
        }
        FOR ( k = 0; k < length / 2; k++ )
        {
            sum_gain = mult_r( subwin[length - 2 * k - 2], subgain[i - 1] ); /* Q15 */
            mod_syn[skip[i] + k] = L_mult( sum_gain, synSHB[skip[i] + k] );
            move32();/* Q_bwe_exc + 16 */
        }
    }
    ELSE
    {
        /* SWB Gain shape and gain frame application with overlap */
        l_frame = L_FRAME16k;
        move16();
        l_shb_lahead = L_SHB_LAHEAD;
        move16();
        skip = skip_bands_SWB_TBE;
        move16();

        num_join = NUM_SHB_SUBFR / NUM_SHB_SUBGAINS;
        move16();
        join_length = i_mult2( num_join, length );
        j = 0; /* ptr*/
        FOR ( k = 0; k < length; k++ )
        {
            sum_gain = mult_r( subwin[k + 1], subgain[0] );         /* Q15 */
            mod_syn[j] = L_mult( synSHB[j], sum_gain );
            move32();   /* Q_bwe_exc + 16 */
            j++;
        }

        FOR ( i = 0; i < NUM_SHB_SUBGAINS - 1; i++ )
        {
            FOR ( k = 0; k < join_length - length; k++ )
            {
                mod_syn[j] = L_mult( synSHB[j], subgain[i * num_join] );
                move32(); /* Q_bwe_exc + 16 */
                j++;
            }

            FOR ( k = 0; k < length; k++ )
            {
                L_tmp = L_mult0( subwin[k + 1], subgain[i_mult2(( i + 1 ), num_join)] ); /* Q30 */
                sum_gain = round_fx( L_mac0( L_tmp, subwin[length - k - 1], subgain[i_mult2(i, num_join)] ) );/*Q14 */
                mod_syn[j] = L_shl( L_mult( sum_gain, synSHB[j] ), 1 );
                move32(); /* Q_bwe_exc + 16 */
                j++;
            }
        }
        FOR ( k = 0; k < join_length - length; k++ )
        {
            mod_syn[j] = L_mult( synSHB[j], subgain[( NUM_SHB_SUBGAINS - 1 ) * num_join] );
            move32(); /* Q_bwe_exc + 16 */
            j++;
        }
        FOR ( k = 0; k < length; k++ )
        {
            sum_gain = mult_r( subwin[length - k - 1], subgain[( NUM_SHB_SUBGAINS - 1 ) *num_join] ); /* Q15 */
            mod_syn[j] = L_mult( synSHB[j], sum_gain );
            move32(); /* Q_bwe_exc + 16 */
            j++;
        }
    }



    max = 0;
    move16();
    FOR( i = 0; i < l_frame + l_shb_lahead; i++ )
    {
        abs_sig = abs_s( round_fx(mod_syn[i]) );
        if(sub(abs_sig,max)>0)
        {
            max = abs_sig;
            move16();
        }
    }

    FOR( i = 0; i < HILBERT_MEM_SIZE; i++ )
    {
        abs_sig = abs_s( round_fx(Hilbert_Mem[i]) );
        if(sub(abs_sig,max)>0)
        {
            max = abs_sig;
            move16();
        }
    }

    sc1 = norm_s( max );                /* max headroom in mod_syn[] */
    sc2 = norm_s( round_fx(frame_gain));/* headroom in GainFrame */

    IF(dynQ == 0 )
    {
        shift = sub(13, Q_bwe_exc); /* earlier = (10 - Q_bwe_exc) but we changed GainFrame Q21 to Q18 */
        *Qx = 0;
    }
    ELSE IF (sub(L_frame,L_FRAME) == 0) /* 12.8k core */
    {
        max_headroom = sub(add(sc1,sc2),4); /* Max headroom after multiplying = sc1 + sc2 -3 (keep 3 bit extra headroom) */
        /* 12.8k core needs extra headroom than 16k core */
        /* otherwise Hilbert transform inside flip_and_downmix have saturation, causes ringing in output */

        /* Qx = (Q_bwe_exc+3) + shift - 16  */
        /* make sure 14 > Qx > 2            */
        min_shift = 2-(Q_bwe_exc+3-16);
        max_shift = 13-(Q_bwe_exc+3-16);
        max_shift2 = s_min(max_shift,max_headroom); /* avoid shifting more than the available max headroom to avoid overflow */

        shift = s_min(min_shift,max_shift2);
        *Qx = (Q_bwe_exc+3) + shift - 16;
    }
    ELSE /* 16k core */
    {
        max_headroom = sub(add(sc1,sc2),1); /* Max headroom after multiplying = sc1 + sc2 -1 (keep 1 bit extra headroom) */

        /* Qx = (Q_bwe_exc+3) + shift - 16  */
        /* make sure 14 > Qx > 3            */
        min_shift = 3-(Q_bwe_exc+3-16);
        max_shift = 13-(Q_bwe_exc+3-16);
        max_shift2 = s_min(max_shift,max_headroom); /* avoid shifting more than the available max headroom to avoid overflow */

        shift = s_min(min_shift,max_shift2);
        *Qx = (Q_bwe_exc+3) + shift - 16;
    }

    /* bring memory st_fx->syn_overlap_fx[] = overlap[i] to new Q = Qx to prepare for addition */
    FOR ( i = 0; i < l_shb_lahead; i++ )
    {
        overlap[i] = shl(overlap[i], (*Qx - prev_Qx));
    }

    FOR ( i = 0; i < l_shb_lahead; i++ )
    {
        /* mod_syn in (16+Q_bwe_exc), frame_gain in Q18 */
        L_tmp = Mult_32_32( mod_syn[i], frame_gain ); /* L_tmp in (Q_bwe_exc+3) */
        synSHB[i] = round_fx( L_shl( Mult_32_16( L_tmp, win[i] ), shift) ); /* Qx */
        synSHB[i] = add( synSHB[i], overlap[i] );
        move16(); /* Qx */
        synSHB[i + l_shb_lahead] = round_fx( L_shl( L_tmp, shift) ); /* Qx */
    }

    FOR ( ; i < l_frame; i++ )
    {
        L_tmp = Mult_32_32( mod_syn[i], frame_gain); /* L_tmp in (Q_bwe_exc+3) */
        synSHB[i] = round_fx( L_shl(L_tmp, shift) );  /* Qx; */
    }

    l_frame_tmp = add(l_frame, l_shb_lahead);
    FOR ( ; i < l_frame_tmp; i++ )
    {
        L_tmp = Mult_32_32( mod_syn[i], frame_gain ); /* (Q_bwe_exc+3) */
        overlap[i - l_frame] = round_fx( L_shl( Mult_32_16( L_tmp, win[l_frame +  l_shb_lahead - 1 - i] ), shift ) );  /* Qx */
    }

    return;
}

static Word32 non_linearity_scaled_copy(
    const Word16 input[],
    Word16 j,
    Word16 length,
    Word32 output[],
    Word32 prev_scale,
    Word16 scale_step,
    Word16 en_abs
)
{
    Word16 i;
    Word32 L_tmp;

    IF (en_abs)
    {
        FOR ( i = 0; i < j; i++ )
        {
            L_tmp = L_mult( input[i], input[i] ); /* 2*Q_inp+1 */
            L_tmp = Mult_32_32( L_tmp, prev_scale ); /* 2*Q_inp */
            output[i] = L_tmp;
            move32();

            L_tmp = Mult_32_16( prev_scale, scale_step );  /* Q29 */
            prev_scale = L_shl( L_tmp, 1 );                /* Q30 */
        }
        FOR ( ; i < length; i++ )
        {
            L_tmp = L_mult( input[i], input[i] ); /* 2*Q_inp+1 */
            L_tmp = Mult_32_32( L_tmp, prev_scale ); /* 2*Q_inp */
            output[i] = L_tmp;
            move32();
        }
    }
    ELSE
    {
        FOR ( i = 0; i < j; i++ )
        {
            L_tmp = L_mult( input[i], input[i] ); /* 2*Q_inp+1 */
            L_tmp = Mult_32_32( L_tmp, prev_scale ); /* 2*Q_inp */

            if ( input[i] < 0 )
            {
                L_tmp = L_negate(L_tmp);
            }
            output[i] = L_tmp;
            move32();

            L_tmp = Mult_32_16( prev_scale, scale_step );  /* Q29 */
            prev_scale = L_shl( L_tmp, 1 );                /* Q30 */
        }

        FOR ( ; i < length; i++ )
        {
            L_tmp = L_mult( input[i], input[i] ); /* 2*Q_inp+1 */
            L_tmp = Mult_32_32( L_tmp, prev_scale ); /* 2*Q_inp */

            if ( input[i] < 0 )
            {
                L_tmp = L_negate(L_tmp);
            }
            output[i] = L_tmp;
            move32();
        }
    }
    return prev_scale;
}

/*==========================================================================*/
/* FUNCTION : void non_linearity() */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Apply a non linearity to the SHB excitation */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* Word16 input[] i : input signal Q_inp */
/* Word16 length i : input length */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* Word32 output[] o : output signal 2*Q_inp */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* Word32 *prev_scale i/o: memory Q30 */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/
/* CALLED FROM : */
/*==========================================================================*/

void non_linearity_fx(
    const   Word16 input[], /* i : input signal Q_inp */
    Word32 output[], /* o : output signal 2*Q_inp */
    const   Word16 length, /* i : input length */
    Word32* pPrevScale, /* i/o: memory Q30 */
    Word16 Q_inp
    ,Word16  coder_type,           /* i  : Coder Type          */
    Word16   *voice_factors,       /* i  : Voice Factors       */
    const Word16   L_frame			    /* i  : ACELP frame length  */

)
{
    Word16 i, j;
    Word16 max=0;
    Word32 scale;
    Word16 scale_step;
    Word16 exp, tmp;
    Word16 e_tmp, f_tmp;
    Word16 frac;
    Word32 L_tmp;
    Word32 L_tmp1;

    Word16 en_abs = 0;
    Word16 v_fac = 0;
    Word16 ths;
    Word16 nframes;
    Word32 prev_scale;
    Word16 length_half;


    IF ( sub(L_frame, L_FRAME16k ) == 0 )
    {
        nframes = 5;
        move16();
        ths = 17817;
        move16(); /* 0.87*5 in Q12 */
    }
    ELSE
    {
        nframes = 4;
        move16();
        ths = 15400;
        move16(); /* 0.94*4 in Q12 */
    }


    FOR ( i = 0; i < nframes; i++ )
    {
        v_fac = add( v_fac, shr( voice_factors[i], 3 ) ); /* Q12 */
    }

    test();
    if ( sub( coder_type, VOICED )==0  && sub( v_fac, ths ) > 0 )
    {
        en_abs = 1;
        move16();
    }

    length_half = shr(length, 1);
    prev_scale = *pPrevScale;
    move32();



    /* Delay Alignment in FX is done inside swb_tbe_enc_fx() */

    FOR ( i = j = 0; i < length_half; i++ )
    {
        tmp = abs_s(input[i]);
        if(sub(tmp,max)>0)
        {
            j = i;
            move16();
        }
        max = s_max(max, tmp);

    }


    IF ( sub(max, shl(1,Q_inp)) > 0 )
    {
        exp = norm_s( max );
        tmp = div_s( shl(1, sub( 14, exp)), max ); /* Q(29-exp-Q_inp) */
        scale = L_shl( L_mult( 21955, tmp ), add(exp, sub(Q_inp,14)) ); /* Q31 */
    }
    ELSE
    {
        scale = 1438814044;
        move32(); /* Q31; 0.67 in Q31 */
    }

    test();
    IF ( prev_scale <= 0 || L_sub( Mult_32_16( prev_scale, 32 ), scale ) > 0 )
    {
        scale_step = 16384;
        move16();               /* Q14 */
        prev_scale = L_shr( scale, 1 );             /* Q30 */
    }
    ELSE
    {

        /* Computing log2(scale) */
        IF ( j == 0 )
        {
            scale_step = 32767;
            move16();
        }
        ELSE
        {
            e_tmp = norm_l( scale );
            f_tmp = Log2_norm_lc( L_shl( scale, e_tmp ) );
            e_tmp = sub(-1, e_tmp);
            L_tmp = Mpy_32_16( e_tmp, f_tmp, 32767 ); /* Q16 */

            /* Computing log2(prev_scale) */
            e_tmp = norm_l( prev_scale );
            f_tmp = Log2_norm_lc( L_shl( prev_scale, e_tmp ) );
            e_tmp = negate(e_tmp);
            L_tmp1 = Mpy_32_16( e_tmp, f_tmp, 32767 ); /* Q16 */

            /* log2(scale / prev_scale) = log2(scale) - log2(prev_scale) */
            L_tmp = L_sub( L_tmp, L_tmp1 ); /* Q16 */

            /* Computing 1/j */
            exp = norm_s( j );
            tmp = div_s( shl(1, sub( 14, exp)), j ); /* Q(29-exp) */

            /* (log2(scale / prev_scale))/length */
            L_tmp = L_shl( Mult_32_16( L_tmp, tmp ), sub(exp, 14) ); /* Q(16+29-exp+1-16+exp-14)->Q16 */

            frac = L_Extract_lc( L_tmp, &exp ); /* Extract exponent of L_tmp */

            tmp = extract_l( Pow2( 14, frac ) );
            scale_step = shl( tmp, exp ); /* Q14 */
        }
    }

    prev_scale = non_linearity_scaled_copy( input, j, length_half, output, prev_scale, scale_step, en_abs );

    max = 0;
    move16();
    j = shr(length, 1);
    FOR ( i = length_half; i < length; i++ )
    {
        tmp = abs_s(input[i]);
        if(sub(tmp,max)>0)
        {
            j = i;
            move16();
        }
        max = s_max(max, tmp);
    }

    IF ( sub( max, shl( 1, Q_inp ) ) > 0 )
    {
        exp = norm_s( max );
        tmp = div_s( shl(1, sub( 14, exp)), max );                      /* Q(29-exp-Q_inp) */
        scale = L_shl( L_mult( 21955, tmp ), add(exp, sub(Q_inp, 14)) );    /* Q31 */
    }
    ELSE
    {
        scale = 1438814044;
        move32();                                /* Q31; 0.67 in Q31 */
    }

    test();
    IF ( prev_scale <= 0 || L_sub( Mult_32_16( prev_scale, 32 ), scale ) > 0 )
    {
        scale_step = 16384;
        move16(); /*Q14 */
        prev_scale = L_shr( scale, 1 ); /*Q30 */
    }
    ELSE
    {
        /*scale_step = (float) exp(1.0f / (float) (j - length/2) * (float) log(scale / prev_scale)); */
        /* Computing log2(scale) */
        IF ( sub(j,length_half) == 0 )
        {
            scale_step = 32767;
            move16();/*Q14 */
        }
        ELSE
        {
            e_tmp = norm_l( scale );
            f_tmp = Log2_norm_lc( L_shl( scale, e_tmp ) );
            e_tmp = sub(-e_tmp, 1);
            L_tmp = Mpy_32_16( e_tmp, f_tmp, 32767 ); /* Q16 */

            /* Computing log2(prev_scale) */
            e_tmp = norm_l( prev_scale );
            f_tmp = Log2_norm_lc( L_shl( prev_scale, e_tmp ) );
            e_tmp = negate(e_tmp);
            L_tmp1 = Mpy_32_16( e_tmp, f_tmp, 32767 ); /* Q16 */

            /* log2(scale / prev_scale) = log2(scale) - log2(prev_scale) */
            L_tmp = L_sub( L_tmp, L_tmp1 ); /* Q16 */

            /* Computing 1/(j - length/2) */

            tmp = sub( j, length_half );
            exp = norm_s( tmp );


            tmp = div_s( shl(1, sub( 14, exp)), tmp ); /* Q(29-exp) */

            /* (log2(scale / prev_scale))/length */
            L_tmp = L_shl( Mult_32_16( L_tmp, tmp ), sub(exp,14) ); /*Q(16+29-exp+1-16+exp-14)->Q16 */

            frac = L_Extract_lc( L_tmp, &exp ); /* Extract exponent of L_tmp */

            tmp = extract_l( Pow2( 14, frac ) );
            scale_step = shl( tmp, exp ); /*Q14 */
        }
    }

    prev_scale = non_linearity_scaled_copy( input+length_half, sub(j, length_half), sub(length, length_half), output+length_half, prev_scale, scale_step, en_abs );

    *pPrevScale = prev_scale;
    move32();

    /* Delay Alignment in FX is done inside swb_tbe_enc_fx() */

    return;
}




/*-------------------------------------------------------------------*
* create_random_vector()
*
* creates random number vector
* Note: the abs(max) value coming out of create_random_vector should
*       fit into the precision of Q6.
* -------------------------------------------------------------------*/

void create_random_vector_fx(
    Word16 output[], /* o : output random vector Q5*/
    const Word16 length, /* i : length of random vector */
    Word16 seed[] /* i/o: start seed */
)
{
    Word16 i, j, k;
    Word16 scale1, scale2;
    Word32 L_tmp;

    L_tmp = L_abs( Mult_32_16( 2144047674, Random( &seed[0] ) ) );/*Q23 */
    j = extract_l( L_shr( L_tmp, 23 ) );
    j = s_and( j, 0xff );

    L_tmp = L_abs( Mult_32_16( 2144047674, Random( &seed[1] ) ) );/*Q23 */
    k = extract_l( L_shr( L_tmp, 23 ) );
    k = s_and( k, 0xff );

    WHILE ( sub(k,j) == 0 )
    {
        L_tmp = L_abs( Mult_32_16( 2144047674, Random( &seed[1] ) ) );/*Q23 */
        k = extract_l( L_shr( L_tmp, 23 ) );
        k = s_and( k, 0xff );
    }

    scale1 = 18021;
    move16(); /* 200.00f * 0.35f/0.1243f; */
    if ( Random( &seed[0] ) < 0 )
    {
        scale1 = -18021;
        move16(); /*Q5 */ /* -200.00f * 0.35f/0.1243f; */
    }

    scale2 = 7208;
    move16(); /* 80.00f * 0.35f/0.1243f; */
    if ( Random( &seed[1] ) < 0 )
    {
        scale2 = -7208;
        move16(); /*Q5 */ /* -80.00f * 0.35f/0.1243f; */
    }

    FOR ( i = 0; i < length; i++ )
    {
        j = s_and( j, 0xff );
        k = s_and( k, 0xff );
        output[i] = round_fx( L_add( L_mult( scale1, gaus_dico_swb_fx[j] ), L_mult( scale2, gaus_dico_swb_fx[k] ) ) );  /*Q5 */
        j++;
        k++;
    }

    return;
}


/*======================================================================================*/
/* FUNCTION : interp_code_5over2_fx() */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Used to interpolate the excitation from the core sample rate */
/* of 12.8 kHz to 32 kHz. */
/* Simple linear interpolator - No need FOR precision here. */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16[]) inp_code_fx : input vector (Q12) */
/* _ (Word16) inp_length : length of input vector */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word16[]) interp_code_fx : output vector (Q12) */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------------------*/

/* _ None */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*======================================================================================*/

void interp_code_5over2_fx(
    const    Word16 inp_code_fx[],       /* i : input vector Qx*/
    Word16 interp_code_fx[],    /* o : output vector Qx*/
    const Word16 inp_length           /* i : length of input vector */
)
{
    Word16 i, kk, kkp1, i_len2;
    Word32 Ltemp;
    Word16 factor_i_fx[5]   = {6554, 19661, 32767, 19661, 6554};
    Word16 factor_j_fx[5]   = {26214, 13107, 0, 13107, 26214};

    interp_code_fx[0] = inp_code_fx[0];
    move16();/* Qx */

    Ltemp = L_mult( inp_code_fx[0], factor_i_fx[3] );/* Q(16+x) */
    Ltemp = L_mac( Ltemp, inp_code_fx[1], factor_j_fx[3] );/* Q(16+x) */
    interp_code_fx[1] = round_fx( Ltemp );/*Qx */

    Ltemp = L_mult( inp_code_fx[0], factor_i_fx[4] );/*Q(16+x) */
    Ltemp = L_mac( Ltemp, inp_code_fx[1], factor_j_fx[4] );/*Q(16+x) */
    interp_code_fx[2] = round_fx( Ltemp );              /* Qx */

    kk = 1;
    move16();
    kkp1 = 2;
    move16();
    i = 3;
    move16();
    /*i_len2 = ( inp_length - 2 ) * HIBND_ACB_L_FAC;  */ /*HIBND_ACB_L_FAC == 5/2 */
    i_len2 = sub(inp_length, 2);
    i_len2 = shr(add(shl(i_len2, 2),i_len2),1);     /* rounding below during shr makes it non BE*/

    FOR ( ; i < i_len2; i += 5 )
    {
        Ltemp = L_mult( inp_code_fx[kk], factor_j_fx[0] );/*Q(16+x) */
        Ltemp = L_mac( Ltemp, inp_code_fx[kkp1], factor_i_fx[0] );/*Q(16+x) */
        interp_code_fx[i] = round_fx( Ltemp );/*Qx */

        Ltemp = L_mult( inp_code_fx[kk], factor_j_fx[1] );/*Q(16+x) */
        Ltemp = L_mac( Ltemp, inp_code_fx[kkp1], factor_i_fx[1] );/*Q(16+x) */
        interp_code_fx[i + 1] = round_fx( Ltemp );/*Qx */

        Ltemp = L_mult( inp_code_fx[kkp1], factor_i_fx[2] );/*Q(16+x) */
        interp_code_fx[i + 2] = round_fx( Ltemp );/*Qx */

        kk++;
        kkp1++;

        Ltemp = L_mult( inp_code_fx[kk], factor_i_fx[3] );/*Q(16+x) */
        Ltemp = L_mac( Ltemp, inp_code_fx[kkp1], factor_j_fx[3] );/*Q(16+x) */
        interp_code_fx[i + 3] = round_fx( Ltemp ); /*Qx */

        Ltemp = L_mult( inp_code_fx[kk], factor_i_fx[4] );/*Q(16+x) */
        Ltemp = L_mac( Ltemp, inp_code_fx[kkp1], factor_j_fx[4] );/*Q(16+x) */
        interp_code_fx[i + 4] = round_fx( Ltemp ); /*Qx */

        kk++;
        kkp1++;
    }

    Ltemp = L_mult( inp_code_fx[kk], factor_j_fx[0] );/*Q(16+x) */
    interp_code_fx[i] = round_fx( Ltemp ); /*Qx */

    Ltemp = L_mult( inp_code_fx[kk], factor_j_fx[1] );/*Q(16+x) */
    interp_code_fx[i + 1] = round_fx( Ltemp ); /*Qx */

    return;
}



/*======================================================================================*/
/* FUNCTION : interp_code_4over2_fx() */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Used to interpolate the excitation from the core sample rate */
/* of 16 kHz to 32 kHz. */
/* Simple linear interpolator - No need for precision here. */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16[]) inp_code_fx : input vector (Qx) */
/* _ (Word16) inp_length : length of input vector */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word16[]) interp_code_fx : output vector (Qx) */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------------------*/

/* _ None */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*======================================================================================*/

void interp_code_4over2_fx(
    const   Word16 inp_code_fx[], /* i : input vector Qx*/
    Word16 interp_code_fx[], /* o : output vector Qx*/
    const Word16 inp_length /* i : length of input vector */
)
{
    Word16 i, j;
    j = 0;
    move16();
    FOR ( i = 0; i < inp_length - 1; i++ )
    {
        interp_code_fx[j] = inp_code_fx[i];
        move16();/*Qx */
        interp_code_fx[j + 1] = add( shr( inp_code_fx[i], 1 ), shr( inp_code_fx[i + 1], 1 ) );
        move16();
        move16();/*Qx */
        j = add( j, 2 );
    }

    interp_code_fx[j] = inp_code_fx[i];
    move16();
    interp_code_fx[j + 1] = shr( inp_code_fx[i], 1 );
    move16();/*Qx */

    return;
}


/*-------------------------------------------------------------------*
* wb_tbe_extras_reset_synth()
*
* Reset the extra parameters only required for WB TBE synthesis
*-------------------------------------------------------------------*/

void wb_tbe_extras_reset_synth_fx( Word16 state_lsyn_filt_shb[], Word16 state_lsyn_filt_dwn_shb[],
                                   Word16 state_32and48k_WB_upsample[]
                                   ,Word16 state_resamp_HB[]
                                 )
{
    set16_fx( state_lsyn_filt_shb, 0, 2 * L_FILT16k );
    set16_fx( state_lsyn_filt_dwn_shb, 0, 2 * L_FILT16k );
    set16_fx( state_32and48k_WB_upsample, 0, 2 * L_FILT16k );
    set16_fx( state_resamp_HB, 0, 2*L_FILT16k );
    return;
}

/*-------------------------------------------------------------------*
 * elliptic_bpf_48k_generic()
 *
 * 18th-order elliptic bandpass filter at 14.0 to 20 kHz sampled at 48 kHz
 * Implemented as 3 fourth order sections cascaded.
 *-------------------------------------------------------------------*/
void elliptic_bpf_48k_generic_fx(
    const Word16 input_fx[],                /* i  : input signal                            */
    Word16 Q_input_fx,
    Word16 output_fx[],               /* o  : output signal                          Q5 */
    Word32 memory_fx[][4],            /* i/o: 4 arrays of 4 for memory               Q16 */
    const Word16 full_band_bpf[][5]         /* i  : filter coefficients b0,b1,b2,a0,a1,a2  Q13 */
)
{
    Word16 i;
    Word32 L_tmp[L_FRAME48k], L_tmp2[L_FRAME48k], L_output[L_FRAME48k], L_tmpX;

    L_tmpX = Mult_32_16(memory_fx[0][0],full_band_bpf[0][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][1],full_band_bpf[0][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][2],full_band_bpf[0][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][3],full_band_bpf[0][1]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[0],full_band_bpf[0][0]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][3],full_band_bpf[3][1]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][2],full_band_bpf[3][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][1],full_band_bpf[3][3]));/*Q14*/
    L_tmp[0] = L_sub(L_tmpX,Mult_32_16(memory_fx[1][0],full_band_bpf[3][4]));/*Q14*/  move32();

    L_tmpX = Mult_32_16(memory_fx[0][1],full_band_bpf[0][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][2],full_band_bpf[0][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][3],full_band_bpf[0][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[0],full_band_bpf[0][1]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[1],full_band_bpf[0][0]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[0],full_band_bpf[3][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][3],full_band_bpf[3][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][2],full_band_bpf[3][3]));/*Q14*/
    L_tmp[1] = L_sub(L_tmpX,Mult_32_16(memory_fx[1][1],full_band_bpf[3][4]));/*Q14*/  move32();

    L_tmpX = Mult_32_16(memory_fx[0][2],full_band_bpf[0][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[0][3],full_band_bpf[0][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[0],full_band_bpf[0][2]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[1],full_band_bpf[0][1]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[2],full_band_bpf[0][0]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[1],full_band_bpf[3][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[0],full_band_bpf[3][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[1][3],full_band_bpf[3][3]));/*Q14*/
    L_tmp[2] = L_sub(L_tmpX,Mult_32_16(memory_fx[1][2],full_band_bpf[3][4]));/*Q14*/  move32();

    L_tmpX = Mult_32_16(memory_fx[0][3],full_band_bpf[0][4]);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[0],full_band_bpf[0][3]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[1],full_band_bpf[0][2]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[2],full_band_bpf[0][1]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shr(L_mult(input_fx[3],full_band_bpf[0][0]),Q_input_fx),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[2],full_band_bpf[3][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[1],full_band_bpf[3][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[0],full_band_bpf[3][3]),2));/*Q14*/
    L_tmp[3] = L_sub(L_tmpX,Mult_32_16(memory_fx[1][3],full_band_bpf[3][4]));/*Q14*/  move32();

    FOR( i=4; i<L_FRAME48k; i++ )
    {
        L_tmpX = L_shr(L_mult(input_fx[sub(i,4)],full_band_bpf[0][4]),Q_input_fx);/*Q14*/
        L_tmpX = L_add(L_shr(L_mult(input_fx[sub(i,3)],full_band_bpf[0][3]),Q_input_fx),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shr(L_mult(input_fx[sub(i,2)],full_band_bpf[0][2]),Q_input_fx),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shr(L_mult(input_fx[sub(i,1)],full_band_bpf[0][1]),Q_input_fx),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shr(L_mult(input_fx[i],full_band_bpf[0][0]),Q_input_fx),L_tmpX);/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[sub(i,1)],full_band_bpf[3][1]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[sub(i,2)],full_band_bpf[3][2]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[sub(i,3)],full_band_bpf[3][3]),2));/*Q14*/
        L_tmp[i] = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp[sub(i,4)],full_band_bpf[3][4]),2));/*Q14*/ move32();
    }

    memory_fx[0][0] = L_shl(L_deposit_l(input_fx[L_FRAME48k-4]),sub(16,Q_input_fx));/* Q16 */ move32();
    memory_fx[0][1] = L_shl(L_deposit_l(input_fx[L_FRAME48k-3]),sub(16,Q_input_fx));/* Q16 */ move32();
    memory_fx[0][2] = L_shl(L_deposit_l(input_fx[L_FRAME48k-2]),sub(16,Q_input_fx));/* Q16 */ move32();
    memory_fx[0][3] = L_shl(L_deposit_l(input_fx[L_FRAME48k-1]),sub(16,Q_input_fx));/* Q16 */ move32();

    L_tmpX = Mult_32_16(memory_fx[1][0],full_band_bpf[1][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][1],full_band_bpf[1][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][2],full_band_bpf[1][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][3],full_band_bpf[1][1]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[0],full_band_bpf[1][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][3],full_band_bpf[4][1]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][2],full_band_bpf[4][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][1],full_band_bpf[4][3]));/*Q14*/
    L_tmp2[0] = L_sub(L_tmpX,Mult_32_16(memory_fx[2][0],full_band_bpf[4][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[1][1],full_band_bpf[1][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][2],full_band_bpf[1][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][3],full_band_bpf[1][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[0],full_band_bpf[1][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[1],full_band_bpf[1][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[4][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][3],full_band_bpf[4][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][2],full_band_bpf[4][3]));/*Q14*/
    L_tmp2[1] = L_sub(L_tmpX,Mult_32_16(memory_fx[2][1],full_band_bpf[4][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[1][2],full_band_bpf[1][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[1][3],full_band_bpf[1][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[0],full_band_bpf[1][2]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[1],full_band_bpf[1][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[2],full_band_bpf[1][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[1],full_band_bpf[4][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[4][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[2][3],full_band_bpf[4][3]));/*Q14*/
    L_tmp2[2] = L_sub(L_tmpX,Mult_32_16(memory_fx[2][2],full_band_bpf[4][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[1][3],full_band_bpf[1][4]);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[0],full_band_bpf[1][3]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[1],full_band_bpf[1][2]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[2],full_band_bpf[1][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[3],full_band_bpf[1][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[2],full_band_bpf[4][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[1],full_band_bpf[4][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[4][3]),2));/*Q14*/
    L_tmp2[3] = L_sub(L_tmpX,Mult_32_16(memory_fx[2][3],full_band_bpf[4][4]));/*Q14*/ move32();

    FOR( i=4; i<L_FRAME48k; i++ )
    {
        L_tmpX = L_shl(Mult_32_16(L_tmp[sub(i,4)],full_band_bpf[1][4]),2);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[sub(i,3)],full_band_bpf[1][3]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[sub(i,2)],full_band_bpf[1][2]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[sub(i,1)],full_band_bpf[1][1]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp[i],full_band_bpf[1][0]),2),L_tmpX);/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[sub(i,1)],full_band_bpf[4][1]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[sub(i,2)],full_band_bpf[4][2]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[sub(i,3)],full_band_bpf[4][3]),2));/*Q14*/
        L_tmp2[i] = L_sub(L_tmpX,L_shl(Mult_32_16(L_tmp2[sub(i,4)],full_band_bpf[4][4]),2));/*Q14*/ move32();
    }

    memory_fx[1][0] = L_shl(L_tmp[L_FRAME48k-4],2);/*Q16*/  move32();
    memory_fx[1][1] = L_shl(L_tmp[L_FRAME48k-3],2);/*Q16*/  move32();
    memory_fx[1][2] = L_shl(L_tmp[L_FRAME48k-2],2);/*Q16*/  move32();
    memory_fx[1][3] = L_shl(L_tmp[L_FRAME48k-1],2);/*Q16*/  move32();

    L_tmpX = Mult_32_16(memory_fx[2][0],full_band_bpf[2][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][1],full_band_bpf[2][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][2],full_band_bpf[2][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][3],full_band_bpf[2][1]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[2][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][3],full_band_bpf[5][1]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][2],full_band_bpf[5][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][1],full_band_bpf[5][3]));/*Q14*/
    L_output[0] = L_sub(L_tmpX,Mult_32_16(memory_fx[3][0],full_band_bpf[5][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[2][1],full_band_bpf[2][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][2],full_band_bpf[2][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][3],full_band_bpf[2][2]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[2][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[1],full_band_bpf[2][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[0],full_band_bpf[5][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][3],full_band_bpf[5][2]));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][2],full_band_bpf[5][3]));/*Q14*/
    L_output[1] = L_sub(L_tmpX,Mult_32_16(memory_fx[3][1],full_band_bpf[5][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[2][2],full_band_bpf[2][4]);/*Q14*/
    L_tmpX = L_add(Mult_32_16(memory_fx[2][3],full_band_bpf[2][3]),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[2][2]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[1],full_band_bpf[2][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[2],full_band_bpf[2][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[1],full_band_bpf[5][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[0],full_band_bpf[5][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,Mult_32_16(memory_fx[3][3],full_band_bpf[5][3]));/*Q14*/
    L_output[2] = L_sub(L_tmpX,Mult_32_16(memory_fx[3][2],full_band_bpf[5][4]));/*Q14*/ move32();

    L_tmpX = Mult_32_16(memory_fx[2][3],full_band_bpf[2][4]);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[0],full_band_bpf[2][3]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[1],full_band_bpf[2][2]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[2],full_band_bpf[2][1]),2),L_tmpX);/*Q14*/
    L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[3],full_band_bpf[2][0]),2),L_tmpX);/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[2],full_band_bpf[5][1]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[1],full_band_bpf[5][2]),2));/*Q14*/
    L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[0],full_band_bpf[5][3]),2));/*Q14*/
    L_output[3] = L_sub(L_tmpX,Mult_32_16(memory_fx[3][3],full_band_bpf[5][4]));/*Q14*/ move32();


    FOR( i=4; i<L_FRAME48k; i++ )
    {
        L_tmpX = L_shl(Mult_32_16(L_tmp2[sub(i,4)],full_band_bpf[2][4]),2);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[sub(i,3)],full_band_bpf[2][3]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[sub(i,2)],full_band_bpf[2][2]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[sub(i,1)],full_band_bpf[2][1]),2),L_tmpX);/*Q14*/
        L_tmpX = L_add(L_shl(Mult_32_16(L_tmp2[i],full_band_bpf[2][0]),2),L_tmpX);/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[sub(i,1)],full_band_bpf[5][1]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[sub(i,2)],full_band_bpf[5][2]),2));/*Q14*/
        L_tmpX = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[sub(i,3)],full_band_bpf[5][3]),2));/*Q14*/
        L_output[i] = L_sub(L_tmpX,L_shl(Mult_32_16(L_output[sub(i,4)],full_band_bpf[5][4]),2));/*Q14*/ move32();
    }

    memory_fx[2][0] = L_shl(L_tmp2[L_FRAME48k-4],2);/*Q16*/ move32();
    memory_fx[2][1] = L_shl(L_tmp2[L_FRAME48k-3],2);/*Q16*/ move32();
    memory_fx[2][2] = L_shl(L_tmp2[L_FRAME48k-2],2);/*Q16*/ move32();
    memory_fx[2][3] = L_shl(L_tmp2[L_FRAME48k-1],2);/*Q16*/ move32();
    FOR( i=0; i<L_FRAME48k; i++)
    {
        output_fx[i] = extract_l(L_shr(L_output[i],9));/*Q5*/
    }
    memory_fx[3][0] = L_shl(L_output[L_FRAME48k-4],2);/*Q16*/ move32();
    memory_fx[3][1] = L_shl(L_output[L_FRAME48k-3],2);/*Q16*/ move32();
    memory_fx[3][2] = L_shl(L_output[L_FRAME48k-2],2);/*Q16*/ move32();
    memory_fx[3][3] = L_shl(L_output[L_FRAME48k-1],2);/*Q16*/ move32();

    return;

}
/*-------------------------------------------------------------------*
 * synthesise_fb_high_band()
 *
 * Creates the highband output for full band  - 14.0 to 20 kHz
 * Using the energy shaped white excitation signal from the SWB BWE.
 * The excitation signal input is sampled at 16kHz and so is upsampled
 * to 48 kHz first.
 * Uses a complementary split filter to code the two regions from
 * 14kHz to 16kHz and 16 kHz to 20 kHz.
 * One of 16 tilt filters is also applied afterwards to further
 * refine the spectral shape of the fullband signal.
 * The tilt is specified in dB per kHz. N.B. Only negative values are
 * accomodated.
 *-------------------------------------------------------------------*/

void synthesise_fb_high_band_fx(
    const Word16 excitation_in[],    /* i  : full band excitation                                */
    Word16 Q_fb_exc,
    Word16 output[],           /* o  : high band speech - 14.0 to 20 kHz                   */
    const Word32 fb_exc_energy,		 /* i  : full band excitation energy                         */
    const Word16 ratio,				 /* i  : energy ratio		                                */
    const Word16 L_frame,			 /* i  : ACELP frame length                                  */
    const Word16 bfi,                /* i  : fec flag			                                */
    Word16 *prev_fbbwe_ratio,  /* o  : previous frame energy for FEC                       */
    Word32 bpf_memory[][4],     /* i/o: memory for elliptic bpf 48k                         */
    Word16 Qout
)
{
    Word16 i, j;
    Word16 excitation_in_interp3[L_FRAME48k];
    Word16 tmp[L_FRAME48k];
    Word32 temp1;
    Word32 ratio2;
    Word32 L_tmp;
    Word16 tmp3,tmp1,tmp2,exp,exp2,exp_tmp;

    /* Interpolate the white energy shaped gaussian excitation from 16 kHz to 48 kHz with zeros */
    j = 0;
    /* white excitation from DC to 8 kHz resampled to produce DC to 24 kHz excitation.          */
    FOR( i=0; i<L_FRAME48k; i+=3)
    {
        excitation_in_interp3[i] = mult(excitation_in[j],24576);/* Q(Q_fb_exc+13-15 = Q_fb_exc-2) */  move16();
        excitation_in_interp3[i+1] = 0;
        move16();
        excitation_in_interp3[i+2] = 0;
        move16();
        j ++;
    }
    exp_tmp = sub(Q_fb_exc,2);

    IF( sub(L_frame,L_FRAME16k) == 0 )
    {
        /* for 16kHz ACELP core */
        elliptic_bpf_48k_generic_fx( excitation_in_interp3, exp_tmp, tmp, bpf_memory, full_band_bpf_3_fx );
    }
    ELSE
    {
        /* for 12.8kHz ACELP core */
        elliptic_bpf_48k_generic_fx( excitation_in_interp3, exp_tmp, tmp, bpf_memory, full_band_bpf_1_fx );
    }
    temp1 = sum2_fx(tmp,L_FRAME48k);

    L_tmp = L_max(1, fb_exc_energy); /*Q(2*Q_fb_exc + 1)*/
    exp = norm_l(L_tmp);
    tmp3 = extract_h(L_shl(L_tmp, exp));
    tmp1 = add(add(Q_fb_exc,Q_fb_exc),1);
    exp = sub(sub(31,tmp1), exp);

    exp2 = norm_l(temp1);
    tmp2 = extract_h(L_shl(temp1, exp2));
    exp2 = sub(31-11, exp2);/* in Q15 (temp1 in Q11)*/

    exp = sub(exp2, exp); /* Denormalize and substract */
    IF (sub(tmp2, tmp3) > 0)
    {
        tmp2 = shr(tmp2, 1);
        exp = add(exp, 1);
    }
    tmp3 = div_s(tmp2, tmp3);
    L_tmp = L_deposit_h(tmp3);
    L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp)*/
    ratio2 = Mult_32_16(L_tmp,ratio);/*Q(31-exp+0-15 = 16-exp)*/

    IF( !bfi )
    {
        *prev_fbbwe_ratio = ratio;
        move16();
    }
    ELSE
    {
        /**prev_fbbwe_ratio = ratio*0.5f;*/
        *prev_fbbwe_ratio = shr(ratio,1);
        move16();
    }
    FOR( i=0; i<L_FRAME48k; i++ )
    {
        L_tmp = Mult_32_16(ratio2,tmp[i]);/* Q(16-exp+5-15 = 6-exp) */
        output[i] = extract_l(L_shr(L_tmp,sub(sub(6,exp),Qout)));/*Qout*/
    }
    return;
}

/*-------------------------------------------------------------------*
* Estimate_mix_factors_fx()                                         *
*                                                                   *
* Estimate mix factors for SHB excitation generation                *
*-------------------------------------------------------------------*/
void Estimate_mix_factors_fx(
    const Word16 *shb_res,            /* i  : SHB LP residual in Q = Q_shb */
    const Word16 Q_shb,
    const Word16 *exc16kWhtnd,        /* i  : SHB transformed low band excitation Q_bwe_exc */
    const Word16 Q_bwe_exc,
    const Word16 *White_exc16k_frac,  /* i  : Modulated envelope shaped white noise Q_frac */
    const Word16 Q_frac,
    const Word32 pow1,                /* i  : SHB exc. power for normalization in Q_pow1 */
    const Word16 Q_pow1,
    const Word32 pow2,                /* i  : White noise excitation for normalization in Q_pow2 */
    const Word16 Q_pow2,
    Word16 *vf_modified,        /* o  : Estimated voice factors */
    Word16 *vf_ind              /* o  : voice factors VQ index */
)
{
    Word16 shb_res_local[L_FRAME16k], WN_exc_local[L_FRAME16k];
    Word32 pow3, temp_p1_p2, temp_p1_p3;
    Word16 temp_numer1[L_FRAME16k], temp_numer2[L_FRAME16k];
    Word16 i, length;
    Word16 exp1,exp2, expa, expb, fraca, fracb, scale, num_flag, den_flag;
    Word16 tmp, tmp1, sc1, sc2;
    Word32 L_tmp1, L_tmp2;

    Copy(shb_res, shb_res_local, L_FRAME16k);
    Copy(White_exc16k_frac, WN_exc_local, L_FRAME16k);
    /* WN_exc_local    in (Q_frac) */



    pow3 = Dot_product(shb_res_local, shb_res_local, L_FRAME16k); /* (2*Q_shb+1) */

    /* temp_p1_p2 = (float)sqrt(pow1/pow2); */
    temp_p1_p2 = root_a_over_b_fx(pow1, Q_pow1, pow2, Q_pow2, &exp1); /* temp_p1_p3 in (Q31+exp1) */

    /* temp_p1_p3 = (float)sqrt(pow1/pow3); */
    temp_p1_p3 = root_a_over_b_fx(pow1, Q_pow1, pow3, (2*Q_shb+1), &exp2); /* temp_p1_p3 in (Q31+exp2) */


    sc1 = Q_bwe_exc - (Q_frac - exp1);
    sc2 = Q_bwe_exc - (Q_shb - exp2);

    FOR(i = 0; i < L_FRAME16k; i++)
    {
        L_tmp1 = Mult_32_16(temp_p1_p2, WN_exc_local[i]); /* (Q_frac - exp1) +16 */
        WN_exc_local[i] = round_fx(L_tmp1);

        L_tmp2 = Mult_32_16(temp_p1_p3, shb_res_local[i]); /* (Q_shb - exp2) +16 */
        shb_res_local[i] = round_fx(L_tmp2);

        /* temp_numer1[i] = sub(shb_res_local[i], WN_exc_local[i]); */
        temp_numer1[i] = round_fx(L_sub(L_shl(L_tmp2, sc2), L_shl(L_tmp1, sc1)));
        /* (Q_bwe_exc) */

        /* temp_numer2[i] = sub(exc16kWhtnd[i], WN_exc_local[i]); */
        temp_numer2[i] = sub(exc16kWhtnd[i], round_fx(L_shl(L_tmp1, sc1 )));
        move16();
        /* (Q_bwe_exc) */
    }


    length = L_FRAME16k;
    move16();
    temp_p1_p2 = Dot_product(temp_numer1, temp_numer2, length);  /* 2*(Q_bwe_exc)+1 */
    temp_p1_p3 = Dot_product(temp_numer2, temp_numer2, length);  /* 2*(Q_bwe_exc)+1 */

    /* vf_modified[i] = min( max( (temp_p1_p2 / temp_p1_p3), 0.1f), 0.99f); */
    /* tmp = (temp_p1_p2 / temp_p1_p3); */
    IF(temp_p1_p3>0)
    {
        expa = norm_l(temp_p1_p3);
        fraca = extract_h(L_shl(temp_p1_p3,expa));
        expa = sub(30,expa);

        expb = norm_l(temp_p1_p2);
        fracb = round_fx(L_shl(temp_p1_p2,expb));
        expb =  sub(30,expb);

        num_flag = 0;
        move16();
        IF(fraca<0)
        {
            num_flag = 1;
            move16();
            fraca = negate(fraca);
        }

        den_flag = 0;
        move16();
        IF(fracb<0)
        {
            den_flag = 1;
            move16();
            fracb = negate(fracb);
        }

        scale = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale);
        expb = sub(expb,scale);

        tmp = div_s(fracb,fraca);
        exp1 = sub(expb,expa);
        tmp = shl(tmp,exp1);

        if(sub(num_flag,den_flag) != 0)
        {
            tmp = negate(tmp);
        }
    }
    ELSE
    {
        tmp = 0;
    }

    vf_modified[0] = min(max(tmp, 3277 /* 0.1f in Q15*/), 32440 /* 0.99f in Q15 */);
    move16();



    *vf_ind = usquant_fx(vf_modified[0], &tmp1, 4096 /* 0.125 in Q15 */, 2048 /* 0.125 in Q14 */, shl(1,NUM_BITS_SHB_VF));
    move16();

    vf_modified[0] = tmp1;
    move16();
    vf_modified[1] = tmp1;
    move16();
    vf_modified[2] = tmp1;
    move16();
    vf_modified[3] = tmp1;
    move16();
    vf_modified[4] = tmp1;
    move16();

    /* vf_modified in Q15 */

    return;
}

/*======================================================================================*/
/* FUNCTION : prep_tbe_exc_fx() */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : Prepare TBE excitation */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _ (Word16) L_frame_fx : length of the frame */
/* _ (Word16) i_subfr_fx : subframe index */
/* _ (Word16) gain_pit_fx : Pitch gain (14) */
/* _ (Word32) gain_code_fx : algebraic codebook gain (Q(16+Q_exc)) */
/* _ (Word16*[]) code_fx : algebraic excitation (Q9) */
/* _ (Word16) voice_fac_fx : voicing factor (Q15) */
/* _ (Word16) gain_preQ_fx : prequantizer excitation gain */
/* _ (Word16[]) code_preQ_fx : prequantizer excitation */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ (Word16*[]) voice_factors_fx : TBE voicing factor (Q15) */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ (Word16[]) bwe_exc_fx : excitation for TBE (Q_exc) */
/*--------------------------------------------------------------------------------------*/

/* _ None */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*======================================================================================*/

void prep_tbe_exc_fx(
    const Word16 L_frame_fx,                        /* i : length of the frame */
    const Word16 i_subfr_fx,                        /* i : subframe index */
    const Word16 gain_pit_fx,                       /* i : Pitch gain Q14*/
    const Word32 gain_code_fx,                      /* i : algebraic codebook gain 16+Q_exc*/
    const Word16 code_fx[],                         /* i : algebraic excitation Q9*/
    const Word16 voice_fac_fx,                      /* i : voicing factor Q15*/
    Word16 *voice_factors_fx,                 /* o : TBE voicing factor Q15*/
    Word16 bwe_exc_fx[],                      /* i/o: excitation for TBE Q_exc*/
    const Word16 gain_preQ_fx,                      /* i : prequantizer excitation gain */
    const Word16 code_preQ_fx[],                    /* i : prequantizer excitation */
    const Word16 Q_exc,                             /* i : Excitation, bwe_exc Q-factor */
    Word16 T0,                                /* i : integer pitch variables Q0 */
    Word16 T0_frac,                           /* i : Fractional pitch variables Q0*/
    const Word16 coder_type,                        /* i : coding type */
    Word32 core_brate                         /* i :core bitrate */
)
{
    Word16 i;
    Word16 tmp_code_fx[L_SUBFR * HIBND_ACB_L_FAC];
    Word16 tmp_code_preInt_fx[L_SUBFR];
    Word16 gain_code16 = 0;
    Word16 tmp /*, tmp1, tmp2*/;
    /*Word16 random_code[L_SUBFR * HIBND_ACB_L_FAC];*/
    Word16 pitch;

    Word32 L_tmp, Ltemp1, Ltemp2;
    Word32 tempQ31;
    Word16 tempQ15;

    /**voice_factors = VF_0th_PARAM + VF_1st_PARAM * voice_fac + VF_2nd_PARAM * voice_fac * voice_fac;
                     = VF_0th_PARAM + voice_fac * (VF_1st_PARAM  + VF_2nd_PARAM * voice_fac )
    *voice_factors = min( max(0.0f, *voice_factors), 1.0f); */
    tempQ31 = L_deposit_h( VF_1st_PARAM_FX );
    tempQ15 = mac_r(tempQ31, VF_2nd_PARAM_FX, voice_fac_fx);
    tempQ31 = L_deposit_h( VF_0th_PARAM_FX );
    tempQ15 = mac_r(tempQ31, voice_fac_fx, tempQ15);
    tempQ15 = s_max(tempQ15, 0);
    *voice_factors_fx = s_min(tempQ15, MAX_16);
    move16();

    tmp = 32767;
    move16();

    pitch = shl( add( shl( T0, 2 ), T0_frac ), 5 ); /* Q7 */

    test();
    test();
    IF ( ( ( sub(coder_type,VOICED) == 0 ) || ( sub( pitch, 14784 ) > 0 ) ) &&  ( L_sub(core_brate,ACELP_8k00) > 0 ) )
    {
        test();
        IF ( sub( pitch, 7392 ) <= 0 )
        {
            tmp = shl( sub( 10076, mult_r( pitch, 26424 ) ), 1 ); /*Q13 13212 = .0126 in Q20; Q(21+7-15)then shr by 3 = Q14 ; */
        }
        ELSE IF ( sub( pitch, 7392 ) > 0 && ( sub( pitch, 14784 ) < 0 ) )
        {
            tmp = shl( mult_r( pitch, 18245 ), 1 ); /*Q14 */
        }
        ELSE IF ( sub( pitch, 14784 ) >= 0 )
        {
            tmp = 16384; /*Q14 */
        }

        if ( sub( tmp, 16384 ) < 0 )
        {
            tmp = shl( tmp, 1 );
        }
        if ( sub( tmp, 16384 ) >= 0 )

        {
            tmp = 32767;
            move16();
        }
        *voice_factors_fx = mult_r( *voice_factors_fx, tmp );
    }

    IF ( sub(L_frame_fx,L_FRAME) == 0 )
    {
        interp_code_5over2_fx( code_fx, tmp_code_fx, L_SUBFR );   /* code: Q9, tmp_code: Q9 */
        gain_code16 = round_fx( L_shl( gain_code_fx, Q_exc ) ); /*Q_exc */
        FOR ( i = 0; i < L_SUBFR * HIBND_ACB_L_FAC; i++ )
        {
            L_tmp = L_mult( gain_code16, tmp_code_fx[i] );   /* Q9 + Q_exc + 1*/
            L_tmp = L_shl( L_tmp, 5 );    /* Q9 + Q_exc + Q6*/
            L_tmp = L_mac( L_tmp, gain_pit_fx, bwe_exc_fx[i + i_subfr_fx * HIBND_ACB_L_FAC] ); /*Q15+Q_exc */
            L_tmp = L_shl( L_tmp, 1 ); /*16+Q_exc */ /* saturation can occur here */
            bwe_exc_fx[i + i_subfr_fx * HIBND_ACB_L_FAC] = round_fx( L_tmp );  /*Q_exc */
        }
    }
    ELSE
    {
        IF( gain_preQ_fx != 0 )
        {
            FOR( i = 0; i < L_SUBFR; i++ )
            {
                /*code in the encoder is Q9 and there is no <<1 with Mult_32_16 Q16 * Q9 -> Q9 */
                Ltemp1 = Mult_32_16(gain_code_fx, code_fx[i]);    /* Q16 + Q9 + 1 - 16 = Q10 */
                Ltemp2 = L_mult(gain_preQ_fx, code_preQ_fx[i]);   /*Q2 * Q10 -> Q12  */

                Ltemp1 = L_shl( Ltemp1, Q_exc+6 /*Q_exc+16-19*/) ; /*Q_exc+16 */
                Ltemp2 = L_shl( Ltemp2, Q_exc+4 /*Q_exc+16-13*/  ) ; /*Q_exc+16 */

                tmp_code_preInt_fx[i] = round_fx(L_add(Ltemp1, Ltemp2)); /* Q_exc  */
            }
        }
        ELSE
        {
            FOR( i = 0; i < L_SUBFR; i++ )
            {
                /*code in the encoder is Q9 and there is no <<1 with Mult_32_16 Q16 * Q9 -> Q9 */
                Ltemp1 = Mult_32_16(gain_code_fx, code_fx[i]);    /* Q16 + Q9 + 1 - 16 = Q10 */
                Ltemp1 = L_shl( Ltemp1, Q_exc+6 /*Q_exc+16-19*/) ; /*Q_exc+16 */
                tmp_code_preInt_fx[i] = round_fx(Ltemp1); /* Q_exc  */
            }
        }

        interp_code_4over2_fx( tmp_code_preInt_fx, tmp_code_fx, L_SUBFR ); /* o: tmp_code in Q_exc */
        FOR ( i = 0; i < L_SUBFR * 2; i++ )
        {
            L_tmp = L_mult(gain_pit_fx, bwe_exc_fx[i + i_subfr_fx*2]); /*Q14+Q_exc+1 */
            tmp = round_fx(L_shl(L_tmp, 1 /* (Q_exc+16)-(14+Q_exc+1)*/ ));  /* tmp in Q_exc */
            bwe_exc_fx[i + i_subfr_fx * 2] = add(tmp, tmp_code_fx[i]);  /*Q_exc */  move16();
        }
    }

    return;
}


/*=============================================================================*/
/* FUNCTION : void swb_formant_fac_fx ( ) */
/*------------------------------------------------------------------------------*/
/* PURPOSE : * Find strength of adaptive formant postfilter using tilt */
/* of the high band. The 2nd lpc coefficient is used as a tilt approximation. */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* const Word16 lpc_shb2 : 2nd HB LPC coefficient Q12 */
/*------------------------------------------------------------------------------*/
/*INPUT/OUTPUT ARGUMENTS : */
/* Word16 *tilt_mem Q12 */
/* OUTPUT ARGUMENTS : */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* formant_fac :Formant filter strength [0,1] Q15 */
/*------------------------------------------------------------------------------*/
/* CALLED FROM : */
/*==============================================================================*/

Word16 swb_formant_fac_fx(  /* o : Formant filter strength [0,1] */
    const Word16   lpc_shb2, /* Q12 i : 2nd HB LPC coefficient */
    Word16*         tilt_mem /* i/o: Tilt smoothing memory (Q12) */
)
{
    Word16 formant_fac;
    Word16 tmp;

    /* Smoothen tilt value */
    /* tmp = 0.5f * (float)fabs(lpc_shb2) + 0.5f * *tilt_mem; */
    tmp = mult_r( 16384, abs_s( lpc_shb2 ) );
    tmp = add( tmp, mult_r( 16384, *tilt_mem ) ); /* Q12 */
    *tilt_mem = tmp;
    move16();/*Q12 */
    /* Map to PF strength */
    /* formant_fac = (tmp - SWB_TILT_LOW)*SWB_TILT_DELTA; */
    tmp = sub( tmp, SWB_TILT_LOW_FX ); /* Q12 */
    formant_fac = mult_r( tmp, SWB_TILT_DELTA_FX ); /* Q12 */


    IF ( sub( formant_fac, 4096 ) > 0 )
    {
        formant_fac = 4096;
        move16();
    }
    ELSE if ( formant_fac < 0 )
    {
        formant_fac = 0;
        move16();
    }
    /* now formant_fac in Q12 */

    /* formant_fac = 1.0f - 0.5f*formant_fac */
    tmp = mult_r(16384, formant_fac); /* 0.5 in Q12 */
    formant_fac = shl(sub(4096,tmp),3);

    return formant_fac; /*Q15 */
}


void wb_tbe_extras_reset_fx(
    Word16 mem_genSHBexc_filt_down_wb2[],
    Word16 mem_genSHBexc_filt_down_wb3[] )
{
    set16_fx( mem_genSHBexc_filt_down_wb2, 0, 2*ALLPASSSECTIONS_STEEP+1  );
    set16_fx( mem_genSHBexc_filt_down_wb3, 0, 2*ALLPASSSECTIONS_STEEP+1  );

    return;
}



Word16 get_tbe_bits_fx(
    Word32 bitrate,
    Word16 bandwidth
    ,Word16 rf_mode
)
{
    Word16 i, bits = 0;

    IF( sub(rf_mode,1)==0 )
    {
        /* TBE bits for core, primary frame */
        test();
        test();
        IF( (sub(bandwidth, WB ) == 0) && (L_sub(bitrate, ACELP_13k20) == 0) )
        {
            /* Gain frame: 4, Gain shapes: 0, and LSFs: 2 */
            bits = NUM_BITS_SHB_FrameGain_LBR_WB + NUM_BITS_LBR_WB_LSF;
            move16();
        }
        ELSE IF( (sub(bandwidth, SWB ) == 0) && (L_sub(bitrate, ACELP_13k20) == 0) )
        {
            /* Gain frame: 5, Gain shapes: 5, and lowrate LSFs: 8 */
            bits = NUM_BITS_SHB_FRAMEGAIN + NUM_BITS_SHB_SUBGAINS + 8;
            move16();
        }
    }
    ELSE
    {
        test();
        test();
        IF( (sub(bandwidth, WB ) == 0) && (L_sub(bitrate, ACELP_9k60) == 0) )
        {
            bits = NUM_BITS_LBR_WB_LSF + NUM_BITS_SHB_FrameGain_LBR_WB;
            move16();
        }
        ELSE IF( (sub( bandwidth, SWB ) == 0) || (sub( bandwidth, FB ) == 0) )
        {
            test();
            IF( L_sub(bitrate, ACELP_9k60) == 0 )
            {
                bits = NUM_BITS_SHB_FRAMEGAIN + NUM_BITS_SHB_SUBGAINS + 8;
                move16();
            }
            ELSE IF( (L_sub( bitrate, ACELP_13k20 ) >= 0 ) && (L_sub( bitrate, ACELP_32k ) <= 0 ) )
            {
                bits =  NUM_BITS_SHB_SUBGAINS + NUM_BITS_SHB_FRAMEGAIN + NUM_LSF_GRID_BITS + MIRROR_POINT_BITS;
                move16();

                FOR ( i=0; i<NUM_Q_LSF; i++ )
                {
                    bits = add( bits, lsf_q_num_bits[i] );
                }
            }

            if ( L_sub( bitrate, ACELP_24k40 ) >= 0 )
            {
                bits = add( bits, NUM_BITS_SHB_ENER_SF + NUM_BITS_SHB_VF + NUM_BITS_SHB_RES_GS*NB_SUBFR16k );
            }

            test();
            test();
            if( sub(bandwidth, SWB) == 0 && (L_sub(bitrate, ACELP_16k40) == 0 || L_sub(bitrate, ACELP_24k40) == 0) )
            {
                bits = add( bits, BITS_TEC+BITS_TFA );
            }

            if ( sub(bandwidth, FB) == 0 )
            {
                /* full band slope */
                bits = add( bits, 4 );
            }
        }
    }

    return bits;
}
