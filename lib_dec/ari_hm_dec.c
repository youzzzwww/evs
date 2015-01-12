/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stl.h"
#include "cnst_fx.h"
#include "basop_util.h"
#include "rom_com_fx.h"
#include "prot_fx.h"

Word16
DecodeIndex(
    Decoder_State_fx *st,
    Word16 Bandwidth,
    Word16 *PeriodicityIndex)
{
    test();
    IF ( (st->tcx_hm_LtpPitchLag > 0) && sub(st->tcxltp_gain, kLtpHmGainThr) > 0)
    {
        Word16 LtpPitchIndex = sub(mult_r(st->tcx_hm_LtpPitchLag, 1 << (15-kLtpHmFractionalResolution)), 2);

        *PeriodicityIndex = kLtpHmFlag;
        move16();
        *PeriodicityIndex = s_or(*PeriodicityIndex, get_next_indice_fx(st, NumRatioBits[Bandwidth][LtpPitchIndex]));
        *PeriodicityIndex = add(*PeriodicityIndex, 1);
        move16();
        *PeriodicityIndex = s_or(*PeriodicityIndex, shl(LtpPitchIndex, 9));
        move16();

        return NumRatioBits[Bandwidth][LtpPitchIndex];
    }
    ELSE
    {
        *PeriodicityIndex = get_next_indice_fx(st, 8);
        move16();
        return 8;
    }
}

static void tcx_hm_dequantize_gain(
    Word16 coder_type,     /* i : coder type          Q0  */
    Word16 gain_idx,       /* i: quantization index   Q0  */
    Word16 *gain           /* o: dequantized gain     Q11 */
)
{

    assert(0 <= coder_type && coder_type <= 1);
    assert(0 <= gain_idx && gain_idx < (1 << kTcxHmNumGainBits));

    *gain = qGains[coder_type][gain_idx];
    move16();

}

void tcx_hm_decode(
    Word16 L_frame,         /* i: number of spectral lines      Q0  */
    Word32 env[],           /* i/o: envelope shape              Q16 */
    Word16 targetBits,      /* i: target bit budget             Q0  */
    Word16 coder_type,      /* i: coder_type                    Q0  */
    Word16 prm_hm[],        /* i: HM parameters                 Q0  */
    Word16 LtpPitchLag,     /* i: LTP pitch lag or -1 if none   Q0  */
    Word16 *hm_bits         /* o: bit consumption               Q0  */
)
{
    Word16 NumTargetBits;
    Word16 fract_res;
    Word32 lag;
    Word16 gain;
    Word16 L_frame_m_256;
    Word16 p[2*kTcxHmParabolaHalfWidth+1];



    *hm_bits = 0;
    move16();

    L_frame_m_256 = sub(L_frame,256);

    assert(coder_type == VOICED || coder_type == GENERIC);

    NumTargetBits = CountIndexBits(
                        (L_frame_m_256 >= 0),
                        prm_hm[1]
                    );

    NumTargetBits = add(NumTargetBits,targetBits);

    if ( sub(coder_type,VOICED) == 0 )
    {
        NumTargetBits = add(NumTargetBits,kTcxHmNumGainBits);
    }

    *hm_bits = add(sub(NumTargetBits,  targetBits), 1);
    move16();

    /* Convert the index to lag */
    UnmapIndex(
        prm_hm[1],
        (L_frame_m_256 >= 0),
        LtpPitchLag,
        (( sub(NumTargetBits,kSmallerLagsTargetBitsThreshold) <= 0 ) || ( L_frame_m_256 < 0 )),
        &fract_res,
        &lag
    );
    test();
    test();

    /* Render the harmonic model */
    tcx_hm_render(
        lag,
        fract_res,
        p
    );


    /* Dequantize gain */
    tcx_hm_dequantize_gain(
        coder_type==VOICED,
        prm_hm[2],
        &gain
    );

    tcx_hm_modify_envelope(
        gain,
        lag,
        fract_res,
        p,
        env,
        L_frame
    );

}

