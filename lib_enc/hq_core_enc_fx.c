/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*--------------------------------------------------------------------------
 * hq_core_enc()
 *
 * HQ core encoder
 *--------------------------------------------------------------------------*/

void hq_core_enc_fx(
    Encoder_State_fx *st_fx,
    const Word16 *audio,           /* i  : input audio signal             Q0  */
    const Word16 input_frame_orig, /* i  : frame length                       */
    const Word16 hq_core_type,     /* i  : HQ core type                       */
    const Word16 Voicing_flag
)
{
    Word16 i, is_transient, num_bits, extra_unused;
    Word32 wtda_audio[2 * L_FRAME48k];
    Word32 t_audio[L_FRAME48k];         /* Q12 */
    Word16 Q_audio = 0;
    Word16 inner_frame, input_frame;
    Word16 ener_match;                  /* Q13/Q15 */

    Word16 tmp;
    Word32 L_tmp;
    UWord16 lsb;


    Word16 two_frames_buffer[2*L_FRAME48k];

    set32_fx( t_audio, 0, L_FRAME48k );
    st_fx->Nb_ACELP_frames_fx = 0;
    move16();

    /* set input_frame length */
    input_frame = input_frame_orig;
    move16();

    st_fx->tcx_cfg.tcx_last_overlap_mode = st_fx->tcx_cfg.tcx_curr_overlap_mode;
    move16();
    st_fx->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
    move16();

    /*--------------------------------------------------------------------------
     * Preprocessing in the first HQ frame after ACELP frame
     * Find the number of bits for PVQ coding
     * Write signalling information
     *--------------------------------------------------------------------------*/

    /*num_bits = (short)(st->total_brate / 50); */
    Mpy_32_16_ss(st_fx->total_brate_fx, 5243, &L_tmp, &lsb);  /* 5243 is 1/50 in Q18. (0+18-15=3) */
    num_bits = extract_l(L_shr(L_tmp, 3)); /*Q0 */
    extra_unused = 0;
    move16();

    /*--------------------------------------------------------------------------
     * Detect signal transition
     *--------------------------------------------------------------------------*/

    is_transient = detect_transient_fx( audio, input_frame, HQ_CORE, 0, st_fx);

    /*--------------------------------------------------------------------------
     * Windowing and time-domain aliasing
     * DCT transform
     *--------------------------------------------------------------------------*/

    Copy( st_fx->old_input_signal_fx, two_frames_buffer, input_frame );
    Copy( audio, two_frames_buffer+input_frame, input_frame );

    wtda_fx( two_frames_buffer+input_frame, &Q_audio, wtda_audio, NULL, 0,
             st_fx->tcx_cfg.tcx_last_overlap_mode, st_fx->tcx_cfg.tcx_curr_overlap_mode, input_frame );

    test();
    IF ( st_fx->last_core_fx == ACELP_CORE || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0 )
    {
        /* Preprocessing in the first HQ frame after ACELP frame */
        core_switching_hq_prepare_enc_fx( st_fx, &num_bits, input_frame, wtda_audio, two_frames_buffer+input_frame );

        /* During ACELP->HQ core switching, limit the HQ core bitrate to 48kbps */
        IF ( sub(num_bits, ACELP_48k_BITS) > 0 )
        {
            extra_unused = sub(num_bits, ACELP_48k_BITS);
            num_bits = ACELP_48k_BITS;
            move16();
        }
    }

    /* subtract signalling bits */
    num_bits = sub(num_bits, st_fx->nb_bits_tot_fx);

    direct_transform_fx( wtda_audio, t_audio, is_transient, input_frame, &Q_audio );

    /* scale coefficients to their nominal level (8kHz) */
    IF ( sub(input_frame, NORM_MDCT_FACTOR) != 0 )
    {
        IF (sub(input_frame, L_FRAME32k) == 0)
        {
            Q_audio = add(Q_audio, 1);          /* Divide by 2 */
        }
        ELSE
        {
            tmp = mult_r(input_frame, 410/2);  /* 1/8000 in Q15 */
            ener_match = hq_nominal_scaling[tmp];
            FOR( i=0; i < input_frame; i++ )
            {
                /*t_audio_q[i] *= ener_match; */
                Mpy_32_16_ss(t_audio[i], ener_match, &t_audio[i], &lsb);
                move16();  /* Q12 */
            }
        }
    }

    /* limit encoded band-width according to the command-line OR BWD limitation */
    inner_frame = inner_frame_tbl[st_fx->bwidth_fx];
    move16();

    IF( sub(input_frame, inner_frame) > 0 )
    {
        IF( sub(is_transient, 1) == 0 )
        {
            FOR ( i = 1; i < NUM_TIME_SWITCHING_BLOCKS; i++ )
            {
                tmp = i_mult2(i, shr(input_frame, 2));
                tmp = shr(inner_frame, 2);
                Copy32( t_audio + i_mult2(i, shr(input_frame, 2)), t_audio + i_mult2(i, tmp), tmp );
            }
        }

        set32_fx( t_audio + inner_frame, 0, sub(input_frame, inner_frame) );
    }

    /*--------------------------------------------------------------------------
     * Classify whether to put extra bits for FER mitigation
     *--------------------------------------------------------------------------*/

    test();
    IF ( sub(st_fx->last_core_fx, HQ_CORE) == 0 && L_sub(st_fx->core_brate_fx, MINIMUM_RATE_TO_ENCODE_VOICING_FLAG) > 0 )
    {
        IF ( Voicing_flag > 0 )
        {
            push_indice_fx( st_fx, IND_HQ_VOICING_FLAG, 1, 1 );
            num_bits = sub(num_bits, 1);

        }
        ELSE
        {
            push_indice_fx( st_fx, IND_HQ_VOICING_FLAG, 0, 1 );
            num_bits = sub(num_bits, 1);
        }
    }

    /*--------------------------------------------------------------------------
     * Transform-domain encoding
     *--------------------------------------------------------------------------*/

    IF ( sub(hq_core_type, LOW_RATE_HQ_CORE) == 0 )
    {
        /* HQ low rate encoder */
        FOR (i = 0; i < inner_frame; i++)
        {
            t_audio[i] = L_shr(t_audio[i], sub(Q_audio, 12));   /* Q12 */
        }
        hq_lr_enc_fx( st_fx, t_audio, inner_frame, &num_bits, is_transient );
        Q_audio = 12;
        move16();
    }
    ELSE
    {
        /* HQ high rate encoder */
        FOR (i = 0; i < inner_frame; i++)
        {
            t_audio[i] = L_shr(t_audio[i], sub(Q_audio, 12));   /* Q12 */
        }

        hq_hr_enc_fx( st_fx, t_audio, inner_frame, &num_bits, is_transient );
        Q_audio = 12;
        move16();
    }

    /* write all unused bits to the bitstream */
    num_bits = add(num_bits, extra_unused);

    WHILE( num_bits >= 16 )
    {
        push_indice_fx( st_fx, IND_UNUSED, 0, 16 );
        num_bits = sub(num_bits, 16);
    }

    IF ( num_bits != 0 )
    {
        push_indice_fx( st_fx, IND_UNUSED, 0, num_bits );
    }

    return;
}
