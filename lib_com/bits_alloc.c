/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "stl.h"
#include "options.h"
#include "rom_com_fx.h"


/*
 * function  BITS_ALLOC_init_config_acelp()
 *
 * description: initial configuration for ACELP
 *
 *  return: void
 */
void BITS_ALLOC_init_config_acelp(
    const Word32 bit_rate,
    const Word8 narrowBand,
    const Word16 nb_subfr,
    ACELP_config *pConfigAcelp        /*o:  configuration structure of ACELP*/
)
{
    Word8 rate_mode_index;


    move16();
    move16();
    move16();
    rate_mode_index=(bit_rate > ACELP_9k60);

    pConfigAcelp->mode_index=rate_mode_index;


    /*LPC: midLpc should be swithced off?*/
    pConfigAcelp->midLpc_enable = 1;
    move16();

    /*ACELP ICB config*/
    test();
    IF( (rate_mode_index==0) || narrowBand != 0 )
    {
        move16();
        move16();
        move16();
        move16();
        move16();
        move16();
        pConfigAcelp->pre_emphasis = 1;
        pConfigAcelp->formant_enh = 1;
        pConfigAcelp->formant_enh_num = FORMANT_SHARPENING_G1;
        pConfigAcelp->formant_enh_den = FORMANT_SHARPENING_G2;
        pConfigAcelp->formant_tilt = 0;
        pConfigAcelp->voice_tilt = 0;
    }
    ELSE
    {
        move16();
        move16();
        move16();
        move16();
        move16();
        move16();
        pConfigAcelp->pre_emphasis = 0;
        pConfigAcelp->formant_enh = 1;
        pConfigAcelp->formant_enh_num = FORMANT_SHARPENING_G1;
        pConfigAcelp->formant_enh_den = FORMANT_SHARPENING_G2;
        pConfigAcelp->formant_tilt = 1;
        pConfigAcelp->voice_tilt = 1;
    }

    /*Wide band @ 16kHz*/
    IF ( sub(nb_subfr,NB_SUBFR16k) == 0 )
    {
        move16();
        move16();
        move16();
        move16();
        move16();
        move16();
        pConfigAcelp->pre_emphasis = 1;
        pConfigAcelp->formant_enh = 1;
        pConfigAcelp->formant_enh_num = FORMANT_SHARPENING_G1_16k;
        pConfigAcelp->formant_enh_den = FORMANT_SHARPENING_G2_16k;
        pConfigAcelp->formant_tilt = 0;
        pConfigAcelp->voice_tilt = 2;
    }

}


/*
 * function  BITS_ALLOC_config_acelp()
 *
 * description: configure all acelp modes and allocate the bits
 *
 *  return: bit demand
 */
Word16 BITS_ALLOC_config_acelp(
    const Word16 bits_frame,                   /*i: remaining bit budget for the frame*/
    const Word16 coder_type,               /*i: acelp coder type*/
    ACELP_config *pConfigAcelp,                /*i/o:  configuration structure of ACELP*/
    const Word16 narrowBand,
    const Word16 nb_subfr
)
{
    Word16 mode_index;
    Word16 band_index;
    Word16 i;
    Word16 remaining_bits, bits;



    move16();
    move16();
    move16();
    mode_index = pConfigAcelp->mode_index;
    band_index = (narrowBand==0);
    bits=0;

    IF ( band_index==0 )
    {
        move16();
        pConfigAcelp->formant_enh = 1;
        if(sub(coder_type,INACTIVE) == 0)
        {
            move16();
            pConfigAcelp->formant_enh = 0;
        }
    }

    IF ( s_and(sub(band_index,1)==0, sub(nb_subfr,4)==0) )
    {
        IF(sub(coder_type,INACTIVE) == 0)
        {
            pConfigAcelp->pre_emphasis = 0;
            move16();
            pConfigAcelp->formant_enh = 0;
            move16();
            pConfigAcelp->formant_enh_num = FORMANT_SHARPENING_G1_16k;
            move16();
            pConfigAcelp->voice_tilt = 1;
            move16();
            pConfigAcelp->formant_tilt = 1;
            move16();
        }
        ELSE
        {
            pConfigAcelp->pre_emphasis = 1;
            move16();
            pConfigAcelp->formant_enh = 1;
            move16();
            pConfigAcelp->formant_enh_num = FORMANT_SHARPENING_G1;
            move16();
            pConfigAcelp->voice_tilt = 0;
            move16();
            pConfigAcelp->formant_tilt = 0;
            move16();
        }
    }
    IF (sub(coder_type,UNVOICED) == 0 )
    {
        IF(sub(ACELP_GAINS_MODE[mode_index][band_index][coder_type], 6) == 0)
        {
            pConfigAcelp->pitch_sharpening = 0;
            move16();
            pConfigAcelp->phase_scrambling = 1;
            move16();
        }
        ELSE
        {
            pConfigAcelp->pitch_sharpening = 0;
            move16();
            pConfigAcelp->phase_scrambling = 0;
            move16();
        }
    }
    ELSE
    {
        pConfigAcelp->pitch_sharpening = 1;
        move16();
        pConfigAcelp->phase_scrambling = 0;
        move16();
    }

    IF(sub(coder_type,ACELP_MODE_MAX) > 0)   /* keep pitch sharpening for RF_ALLPRED mode */
    {
        pConfigAcelp->pitch_sharpening = 0;
        pConfigAcelp->phase_scrambling = 0;
    }

    /*Allocate bits and different modes*/
    move16();
    pConfigAcelp->bpf_mode=ACELP_BPF_MODE[mode_index][band_index][coder_type];
    bits = add(bits, ACELP_BPF_BITS[pConfigAcelp->bpf_mode]);

    move16();
    move16();
    pConfigAcelp->nrg_mode=ACELP_NRG_MODE[mode_index][band_index][coder_type];
    pConfigAcelp->nrg_bits=ACELP_NRG_BITS[pConfigAcelp->nrg_mode];
    bits = add(bits, pConfigAcelp->nrg_bits);

    move16();
    pConfigAcelp->ltp_mode=ACELP_LTP_MODE[mode_index][band_index][coder_type];

    move16();
    pConfigAcelp->ltp_bits=0;

    move16();
    pConfigAcelp->ltf_mode=ACELP_LTF_MODE[mode_index][band_index][coder_type];

    move16();
    pConfigAcelp->ltf_bits=ACELP_LTF_BITS[pConfigAcelp->ltf_mode];
    if ( s_and(sub(nb_subfr,5)==0, sub(pConfigAcelp->ltf_bits,4)==0) )
    {
        pConfigAcelp->ltf_bits = add(pConfigAcelp->ltf_bits,1);
    }
    bits = add(bits,pConfigAcelp->ltf_bits);


    FOR ( i=0; i<nb_subfr; i++ )
    {
        pConfigAcelp->gains_mode[i] = 10;
        move16();
        test();
        if ( sub(bits_frame, 500) <= 0 || sub(nb_subfr, 5) != 0 )
        {
            pConfigAcelp->gains_mode[i] = ACELP_GAINS_MODE[mode_index][band_index][coder_type];
            move16();
        }

        /* skip subframe 1, 3 gain encoding, and use from subframe 0, and 3, respectively */
        test();
        test();
        IF(sub(coder_type,ACELP_MODE_MAX) >= 0 && (sub(i,1) == 0 || sub(i,3) == 0))
        {
            pConfigAcelp->gains_mode[i] = 0;
        }

        bits = add(bits, ACELP_GAINS_BITS[pConfigAcelp->gains_mode[i]]);

        move16();
        bits = add(bits, ACELP_LTP_BITS_SFR[pConfigAcelp->ltp_mode][i]);
        pConfigAcelp->ltp_bits= add( pConfigAcelp->ltp_bits,ACELP_LTP_BITS_SFR[pConfigAcelp->ltp_mode][i]);
    }

    /*Innovation*/

    if ( sub(bits_frame,bits) < 0)
    {
        printf("Warning: bits per frame too low\n");
        return -1;
    }

    IF( sub(coder_type,RF_ALLPRED) == 0 )
    {
        set16_fx(pConfigAcelp->fixed_cdk_index, -1, nb_subfr);
    }
    ELSE IF ( sub(coder_type,RF_GENPRED) == 0 )
    {
        pConfigAcelp->fixed_cdk_index[0] = 2;  /* 7 bits */
        pConfigAcelp->fixed_cdk_index[1] = -1;
        pConfigAcelp->fixed_cdk_index[2] = 2;  /* 7 bits */
        pConfigAcelp->fixed_cdk_index[3] = -1;
        pConfigAcelp->fixed_cdk_index[4] = -1;
        bits = add(bits,14);
    }
    ELSE IF( sub(coder_type,RF_NOPRED) == 0 )
    {
        set16_fx(pConfigAcelp->fixed_cdk_index, 2, nb_subfr);
        bits = add(bits,28);
    }
    ELSE
    {
        bits = add(bits, BITS_ALLOC_adjust_acelp_fixed_cdk(sub(bits_frame,bits), pConfigAcelp->fixed_cdk_index, nb_subfr ));
    }

    remaining_bits = sub(bits_frame, bits);

    /*Sanity check*/
    if (remaining_bits<0)
    {
        move16();
        bits = -1;
    }


    return(bits);
}


static
Word16 BITS_ALLOC_adjust_generic(
    const Word16 bits_frame, /*i: bit budget*/
    Word16 *fixed_cdk_index,
    const Word16 nb_subfr,
    const Word16 *pulseconfigbits,
    const Word16 pulseconfig_size
)
{
    Word16 bits_subframe2, inb_subfr;
    Word16 sfr, k, bitsused, bits_currsubframe;

    bits_subframe2 = bits_frame;
    move16();
    inb_subfr = FL2WORD16(1.0f/NB_SUBFR);
    move16();
    if ( sub(nb_subfr,NB_SUBFR16k) == 0 )
    {
        inb_subfr = FL2WORD16(1.0f/NB_SUBFR16k);
        move16();
    }

    IF ( sub(bits_subframe2, i_mult2(pulseconfigbits[0], nb_subfr)) < 0 )                    /* not in final code - not instrumented */
    {
        return add(bits_frame,1); /* Not enough bits for lowest mode. -> trigger alarm*/
    }

    /* search cdk-index for first subframe */
    FOR (k=0; k<pulseconfig_size-1; k++)
    {

        IF (i_mult2(pulseconfigbits[k], nb_subfr) > bits_subframe2)
        {
            k = sub(k,1);    /* previous mode did not exceed bit-budget */
            BREAK;
        }
    }

    if (i_mult2(pulseconfigbits[k], nb_subfr) > bits_subframe2)
    {
        k = sub(k,1);    /* previous mode did not exceed bit-budget */
    }

    move16();
    fixed_cdk_index[0] = k;
    bitsused = i_mult2(pulseconfigbits[k], nb_subfr);

    FOR (sfr=1; sfr < nb_subfr; sfr++)
    {
        /*bits_currsubframe = (int)(((float)sfr+1.0f)*bits_subframe) - bitsused;*/
        bits_currsubframe = sub(add(i_mult2(sfr, bits_subframe2), bits_subframe2), bitsused);

        /* try increasing mode while below threshold */
        WHILE ( (sub(k, pulseconfig_size-1) < 0) && (sub(i_mult2(pulseconfigbits[add(k,1)], nb_subfr),bits_currsubframe) <= 0) )
        {
            test();
            k = add(k,1);
        }

        /* try decreasing mode until below threshold */
        WHILE (i_mult2(pulseconfigbits[k], nb_subfr) > bits_currsubframe)
        {
            k = sub(k,1);

            IF (k == 0)
            {
                BREAK;
            }
        }

        /* store mode */
        move16();
        fixed_cdk_index[sfr] = k;
        bitsused = add(bitsused, i_mult2(pulseconfigbits[k], nb_subfr));
    }

    return mult_r(bitsused, inb_subfr);
}

Word16 BITS_ALLOC_adjust_acelp_fixed_cdk(
    const Word16 bits_frame, /*i: bit budget*/
    Word16 *fixed_cdk_index,
    const Word16 nb_subfr
)
{
    Word16 bitsused;


    bitsused = BITS_ALLOC_adjust_generic(bits_frame, fixed_cdk_index, nb_subfr, ACELP_CDK_BITS, ACELP_FIXED_CDK_NB);


    return bitsused;
}

