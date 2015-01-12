/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "stat_enc_fx.h"
#include "rom_com_fx.h"
#include "stl.h"


/*-----------------------------------------------------------------*
 * decision_matrix_enc()
 *
 * Select operating point (combination of technologies) based on input signal properties and command-line parameters:
 *
 *             7.20        8.00        9.60        13.20        16.40         24.40            32               48               64               96      128
 *  Mode       1           1           2           1            2             2                2                2                1                2       2
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NB
 *  speech     ACELP@12k8  ACELP@12k8  ACELP@12k8  ACELP@12k8
 *  audio      LR MDCT     LR MDCT     TCX         LR MDCT
 *  inactive   GSC@12k8    GSC@12k8    TCX         GSC@12k8
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  WB
 *  speech     ACELP@12k8  ACELP@12k8  ACELP@12k8  ACELP@12k8   ACELP@16k     ACELP@16k        ACELP@16k        TCX              ACELP@16k        TCX     TCX
 *             +0b WB BWE  +0b WB BWE  +TD WB BWE  +TD WB BWE
 *  audio      GSC@12k8    GSC@12k8    TCX         LR MDCT      TCX           TCX              HQ               TCX              HQ               TCX     TCX
 *             +0b WB BWE  +0b WB BWE  +IGF
 *  inactive   GSC@12k8    GSC@12k8    TCX         GSC@12k8     TCX           TCX              AVQ@16k          TCX              AVQ@16k          TCX     TCX
 *             +0b WB BWE  +0b WB BWE  +IGF        +FD WB BWE
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  SWB
 *  speech                                         ACELP@12k8   ACELP@16k     ACELP@16k        ACELP@16k        TCX              ACELP@16k        TCX     TCX
 *                                                 +TD SWB BWE  +TD SWB BWE   +TD SWB BWE      +TD SWB BWE      +IGF             +HR SWB BWE
 *  audio                                          LR MDCT/GSC  TCX           TCX              HQ               TCX              HQ               TCX     TCX
 *                                                 +FD SWB BWE  +IGF          +IGF             +FD SWB BWE      +IGF
 *  inactive                                       GSC@12k8     TCX           TCX              AVQ@16k          TCX              AVQ@16k          TCX     TCX
 *                                                 +FD SWB BWE  +IGF          +IGF             +FD SWB BWE      +IGF             +HR SWB BWE
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  FB
 *  speech                                                      ACELP@16k     ACELP@16k        ACELP@16k        TCX              ACELP@16k        TCX     TCX
 *                                                              +TD FB BWE    +TD FB BWE       +TD FB BWE       +IGF             +HR FB BWE
 *  audio                                                       TCX           TCX              HQ               TCX              HQ               TCX     TCX
 *                                                              +IGF          +IGF             +FD FB BWE       +IGF
 *  inactive                                                    TCX           TCX              AVQ@16k          TCX              AVQ@16k          TCX     TCX
 *                                                              +IGF          +IGF             +FD FB BWE       +IGF             +HR FB BWE
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------
 *
 * Note: the GSC technology is part of the ACELP core as AUDIO coder_type (it is used also at 13.2 and 16.4 kbps for SWB unvoiced noisy speech)
 * Note2: FB processing is optional and is activated via "-band FB" option on the encoder command line
 * Note3: NB (0-4kHz), WB (0-8kHz), SWB (0-16kHz), FB (0-20kHz)
 *
 * Signalling of modes (x marks a mode that must be signalled in the bitstream)
 *
 *                     7.20           8.00           9.6            13.2           16.4           24.4           32             48             64
 *                     NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB   NB WB SWB FB
 * GC, 12k8            x  x           x  x           x  x           x  x   x       x  x   x       x
 * UC, 12k8            x  x           x  x           x  x
 * VC, 12k8            x  x           x  x           x  x           x  x   x       x  x   x       x
 * TC, 12k8            x  x           x  x           x  x           x  x   x       x  x   x       x
 * GC, 16k                                                                                           x   x   x      x   x   x      x   x   x      x   x   x
 * TC, 16k                                                                                           x   x   x      x   x   x      x   x   x      x   x   x
 * AC(GSC)             x  x           x  x           x  x           x  x   x       x  x   x       x
 * IC                  x  x           x  x           x  x           x  x   x       x  x   x       x  x   x   x      x   x   x      x   x   x      x   x   x
 *
 * GC, 12k8, FS        x  x           x  x           x  x           x  x   x       x  x   x       x
 * GC, 16k, FS                                                                                       x   x   x      x   x   x      x   x   x          x   x
 * VC, 12k8, FS                                                     x  x   x       x  x   x       x
 * TC, 12k8, FS                                                                                   x
 * TC, 16k, FS                                                                                       x   x   x      x   x   x      x   x   x          x   x
 *
 * LR MDCT             x              x              x              x  x   x   x   x  x   x   x
 *
 *-----------------------------------------------------------------*/

void decision_matrix_enc_fx(
    Encoder_State_fx *st_fx,             /* i  : encoder state structure                   */
    const Word16 sp_aud_decision1,   /* i  : 1st stage speech/music classification     */
    const Word16 sp_aud_decision2,   /* i  : 2nd stage speech/music classification     */
    const Word16 coder_type,         /* i  : coder type                                */
    const Word16 vad_flag,
    Word16 *hq_core_type       /* o  : HQ core type                              */
)
{
    /* initialization */
    st_fx->core_fx = -1;
    move16();
    st_fx->extl_fx = -1;
    move16();
    st_fx->extl_brate_fx = 0;
    move16();
    *hq_core_type = -1;
    move16();
    st_fx->igf = 0;
    move16();
    /* SID and FRAME_NO_DATA frames */
    test();
    test();
    IF( st_fx->Opt_DTX_ON_fx && (L_sub(st_fx->core_brate_fx,SID_2k40) == 0 || L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) == 0 ) )
    {
        st_fx->core_fx = ACELP_CORE;
        move16();

        test();
        if( L_sub(st_fx->input_Fs_fx,32000) >= 0 && sub(st_fx->bwidth_fx,SWB) >= 0 )
        {
            st_fx->extl_fx = SWB_CNG;
            move16();
        }

        st_fx->rf_mode = 0;
        move16();

        return;
    }

    st_fx->core_brate_fx = L_deposit_l(0);

    /* SC-VBR */
    IF ( st_fx->Opt_SC_VBR_fx )
    {
        /* SC-VBR */
        st_fx->core_fx = ACELP_CORE;
        move16();
        st_fx->core_brate_fx = ACELP_7k20;
        move32();
        st_fx->total_brate_fx = ACELP_7k20;
        move32();

        test();
        test();
        test();
        IF ( sub(st_fx->ppp_mode_fx,1) == 0 )
        {
            /* PPP mode */
            st_fx->core_brate_fx = PPP_NELP_2k80;
            move16();
        }
        ELSE IF ( ( ( sub(coder_type,UNVOICED) == 0 || sub(coder_type,TRANSITION) == 0 ) && sp_aud_decision1 == 0 ) || sub(st_fx->bwidth_fx, NB) != 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF ( sub(coder_type,UNVOICED) == 0 && sub(vad_flag,1) == 0 &&
                 ( ( sub(st_fx->last_bwidth_fx,SWB) >= 0 && st_fx->last_Opt_SC_VBR_fx == 1 ) || sub(st_fx->last_bwidth_fx, SWB) < 0 ) &&
                 ( sub(st_fx->last_core_fx,HQ_CORE) != 0 || sub(st_fx->bwidth_fx, NB) != 0) )
            {
                /* NELP mode */
                st_fx->nelp_mode_fx = 1;
                move16();
                st_fx->core_brate_fx = PPP_NELP_2k80;
                move32();
            }
            ELSE IF ( sub(coder_type,TRANSITION) == 0 || ( sub(coder_type,UNVOICED) == 0 && sub(st_fx->nelp_mode_fx, 1) != 0 ) ||
                      ( ( sub(coder_type,AUDIO) == 0 || sub(coder_type,INACTIVE) == 0 ) && sub(st_fx->bwidth_fx, NB) != 0 ) )

            {
                /* silence portions */
                st_fx->core_brate_fx = ACELP_8k00;
                move32();
                st_fx->total_brate_fx = ACELP_8k00;
                move32();
            }
        }

        return;
    }

    /*---------------------------------------------------------------------*
     * NB
     *---------------------------------------------------------------------*/

    ELSE IF ( sub(st_fx->bwidth_fx,NB) == 0 )
    {
        st_fx->core_fx = ACELP_CORE;
        move16();

        test();
        if ( L_sub(st_fx->total_brate_fx,HQCORE_NB_MIN_RATE) >= 0 && sub(sp_aud_decision1,1) == 0 )
        {
            st_fx->core_fx = HQ_CORE;
            move16();
        }
    }

    /*---------------------------------------------------------------------*
     * WB
     *---------------------------------------------------------------------*/

    ELSE IF ( sub(st_fx->bwidth_fx,WB) == 0 )
    {
        st_fx->core_fx = ACELP_CORE;
        move16();

        test();
        test();
        IF ( ( L_sub(st_fx->total_brate_fx,HQCORE_WB_MIN_RATE) >= 0 && sub(sp_aud_decision1,1) == 0 ) ||
             L_sub(st_fx->total_brate_fx,HQ_96k) >= 0 )
        {
            st_fx->core_fx = HQ_CORE;
            move16();
        }
        ELSE
        {
            test();
            test();
            test();
            test();
            IF ( sub(st_fx->bwidth_fx,WB) == 0 && L_sub(st_fx->total_brate_fx,ACELP_9k60) < 0 )
            {
                st_fx->extl_fx = WB_BWE;
                move16();
            }
            ELSE IF ( sub(st_fx->bwidth_fx,WB) == 0 && L_sub(st_fx->total_brate_fx,ACELP_9k60) >= 0 && L_sub(st_fx->total_brate_fx,ACELP_16k40) <= 0 )
            {
                /* Note: WB BWE is used exceptionally at 13.2 kbps if GSC is selected instead of LR-MDCT */
                test();
                test();
                test();
                IF ( sub(sp_aud_decision1,1) == 0 || sub(coder_type,INACTIVE) == 0 || ( sp_aud_decision1 == 0 && sub(sp_aud_decision2,1) == 0) )
                {
                    st_fx->extl_fx = WB_BWE;
                    move16();
                    st_fx->extl_brate_fx = WB_BWE_0k35;
                    move32();
                }
                ELSE
                {
                    st_fx->extl_fx = WB_TBE;
                    move16();
                    IF ( L_sub(st_fx->total_brate_fx,ACELP_9k60) == 0 )
                    {
                        st_fx->extl_brate_fx = WB_TBE_0k35;
                        move32();
                    }
                    ELSE
                    {
                        st_fx->extl_brate_fx = WB_TBE_1k05;
                        move32();
                    }
                }
            }
        }
    }

    /*---------------------------------------------------------------------*
     * SWB and FB
     *---------------------------------------------------------------------*/

    ELSE IF ( sub(st_fx->bwidth_fx,SWB) == 0 || sub(st_fx->bwidth_fx,FB) == 0 )
    {
        test();
        test();
        IF ( ( L_sub(st_fx->total_brate_fx,HQCORE_SWB_MIN_RATE) >= 0 && sub(sp_aud_decision1,1) == 0) ||
             L_sub(st_fx->total_brate_fx,HQ_96k) >= 0 )
        {
            st_fx->core_fx = HQ_CORE;
            move16();
        }
        ELSE
        {
            st_fx->core_fx = ACELP_CORE;
            move16();

            test();
            test();
            IF ( L_sub(st_fx->total_brate_fx,ACELP_13k20) >= 0 && L_sub(st_fx->total_brate_fx,ACELP_48k) < 0 )
            {
                /* Note: SWB BWE is not used in case of GSC noisy speech */
                /* Note: SWB BWE is used exceptionally at 13.2 kbps if GSC is selected instead of LR-MDCT */
                test();
                test();
                test();
                test();
                IF ( (sub(sp_aud_decision1,1) == 0 || sub(coder_type,INACTIVE) == 0 || ( sp_aud_decision1 == 0 && sub(sp_aud_decision2,1) == 0 )) && st_fx->GSC_noisy_speech_fx == 0 )
                {
                    st_fx->extl_fx = SWB_BWE;
                    move16();
                    st_fx->extl_brate_fx = SWB_BWE_1k6;
                    move32();

                    test();
                    IF ( sub(st_fx->bwidth_fx,FB) == 0 && L_sub(st_fx->total_brate_fx,ACELP_24k40) >= 0 )
                    {
                        st_fx->extl_fx = FB_BWE;
                        move16();
                        st_fx->extl_brate_fx = FB_BWE_1k8;
                        move32();
                    }
                }
                ELSE
                {
                    st_fx->extl_fx = SWB_TBE;
                    move16();
                    st_fx->extl_brate_fx = SWB_TBE_1k6;
                    move32();
                    if( L_sub(st_fx->total_brate_fx,ACELP_24k40) >= 0 )
                    {
                        st_fx->extl_brate_fx = SWB_TBE_2k8;
                        move32();
                    }

                    test();
                    IF ( sub(st_fx->bwidth_fx,FB) == 0 && L_sub(st_fx->total_brate_fx,ACELP_24k40) >= 0 )
                    {
                        st_fx->extl_fx = FB_TBE;
                        move16();
                        st_fx->extl_brate_fx = FB_TBE_1k8;
                        move32();

                        if( L_sub(st_fx->total_brate_fx,ACELP_24k40) >= 0 )
                        {
                            st_fx->extl_brate_fx = FB_TBE_3k0;
                            move32();
                        }
                    }
                }
            }
            ELSE IF ( L_sub(st_fx->total_brate_fx,ACELP_48k) >= 0 )
            {
                st_fx->extl_fx = SWB_BWE_HIGHRATE;
                move16();
                st_fx->extl_brate_fx = SWB_BWE_16k;
                move32();

                if( sub(st_fx->bwidth_fx,FB) == 0 )
                {
                    st_fx->extl_fx = FB_BWE_HIGHRATE;
                    move32();
                }
            }
        }
    }

    /*-----------------------------------------------------------------*
     * Set HQ core type
     *-----------------------------------------------------------------*/

    IF( sub(st_fx->core_fx,HQ_CORE) == 0 )
    {
        *hq_core_type = NORMAL_HQ_CORE;
        move16();

        test();
        test();
        IF( (sub(st_fx->bwidth_fx,SWB) == 0 || sub(st_fx->bwidth_fx,WB) == 0) && L_sub(st_fx->total_brate_fx,LRMDCT_CROSSOVER_POINT) <= 0 )
        {
            /* note that FB is always coded with NORMAL_HQ_CORE */
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
        ELSE IF( sub(st_fx->bwidth_fx,NB) == 0 )
        {
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
    }

    /* set core bitrate */
    st_fx->core_brate_fx = L_sub(st_fx->total_brate_fx, st_fx->extl_brate_fx);

    IF ( st_fx->ini_frame_fx == 0 )
    {
        /* avoid switching in the very first frame */
        st_fx->last_core_fx = st_fx->core_fx;
        move16();
        st_fx->last_core_brate_fx = st_fx->core_brate_fx;
        move32();
        st_fx->last_extl_fx = st_fx->extl_fx;
        move16();
    }

    return;
}

/*---------------------------------------------------------------------*
 * signalling_mode1_tcx20_enc()
 *
 * write MODE1 TCX20 signalling information into the bit-stream
 *---------------------------------------------------------------------*/

Word16 signalling_mode1_tcx20_enc(
    Encoder_State_fx *st,                /* i  : encoder state structure   */
    Word16 push
)
{
    Word16 num_bits;
    Word16 nBits, idx, start_idx;

    assert(st->core_fx == TCX_20_CORE);

    num_bits = 0;
    move16();

    /* Use ACELP signaling for LR MDCT */
    IF ( L_sub(st->total_brate_fx, ACELP_16k40) <= 0 )
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        idx = 0;
        move16();
        WHILE ( L_sub(acelp_sig_tbl[idx], st->total_brate_fx) != 0 )
        {
            idx = add(idx, 1);
        }

        /* retrieve the number of bits for signalling */
        idx = add(idx, 1);
        nBits = extract_l(acelp_sig_tbl[idx]);

        /* retrieve the signalling index */
        idx = add(idx, 1);
        start_idx = idx;
        move16();
        WHILE ( L_sub(acelp_sig_tbl[idx], SIG2IND_fx(LR_MDCT, st->bwidth_fx, 0, 0)) != 0 )
        {
            idx = add(idx, 1);
        }

        num_bits = add(num_bits, nBits);
        IF (push != 0)
        {
            push_indice_fx( st, IND_ACELP_SIGNALLING, sub(idx, start_idx), nBits );
        }

        /* HQ/TCX core switching flag */
        num_bits = add(num_bits, 1);
        IF (push != 0)
        {
            push_indice_fx( st, IND_MDCT_CORE, 1, 1 );
        }
    }
    ELSE
    {
        IF ( L_sub(st->core_brate_fx, ACELP_64k) <= 0 )
        {
            /* write ACELP/HQ core indication flag */
            num_bits = add(num_bits, 1);
            IF (push != 0)
            {
                push_indice_fx( st, IND_CORE, 1, 1 );
            }
        }

        /* HQ/TCX core switching flag */
        num_bits = add(num_bits, 1);
        IF (push != 0)
        {
            push_indice_fx( st, IND_MDCT_CORE, 1, 1 );
        }

        num_bits = add(num_bits, 2);
        IF (push != 0)
        {
            /* write band-width (needed for different I/O sampling rate support) */
            IF ( sub(st->bwidth_fx, NB) == 0 )
            {
                push_indice_fx( st, IND_HQ_BWIDTH, 0, 2 );
            }
            ELSE IF ( sub(st->bwidth_fx, WB) == 0 )
            {
                push_indice_fx( st, IND_HQ_BWIDTH, 1, 2 );
            }
            ELSE IF ( sub(st->bwidth_fx, SWB) == 0 )
            {
                push_indice_fx( st, IND_HQ_BWIDTH, 2, 2 );
            }
            ELSE  /* st->bwidth == FB */
            {
                push_indice_fx( st, IND_HQ_BWIDTH, 3, 2 );
            }
        }
    }

    return num_bits;
}

/*---------------------------------------------------------------------*
 * signalling_enc()
 *
 * write signalling information into the bit-stream
 *---------------------------------------------------------------------*/

void signalling_enc_fx(
    Encoder_State_fx *st_fx,             /* i  : encoder state structure   */
    const Word16 coder_type,         /* i  : coder type                */
    const Word16 sharpFlag           /* i  : formant sharpening flag   */
)
{
    Word16 nBits, idx, start_idx;
    Word32 k;

    IF (sub(st_fx->mdct_sw, MODE2) == 0)
    {

        assert(!st_fx->tcxonly);
        assert(st_fx->core_fx == HQ_CORE);

        push_next_indice_fx(st_fx, 1, 1); /* TCX */
        push_next_indice_fx(st_fx, 1, 1); /* HQ_CORE */

        /* write ACELP->HQ core switching flag */
        test();
        IF ( sub(st_fx->last_core_fx, ACELP_CORE) == 0 || sub(st_fx->last_core_fx, AMR_WB_CORE) == 0 )
        {
            push_indice_fx( st_fx, IND_HQ_SWITCHING_FLG, 1, 1 );

            /* write ACELP L_frame info */
            IF( sub(st_fx->last_L_frame_fx, L_FRAME) == 0 )
            {
                push_indice_fx( st_fx, IND_LAST_L_FRAME, 0, 1 );
            }
            ELSE
            {
                push_indice_fx( st_fx, IND_LAST_L_FRAME, 1, 1 );
            }
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_HQ_SWITCHING_FLG, 0, 1 );
        }

        return;
    }
    IF( sub(st_fx->core_fx,ACELP_CORE ) == 0)
    {

        test();
        test();
        IF( sub(st_fx->ppp_mode_fx,1) == 0 || sub(st_fx->nelp_mode_fx,1) == 0 )
        {

            /* 1 bit to distinguish between 2.8kbps PPP/NELP frame and SID frame */
            push_indice_fx( st_fx, IND_CORE, 0, 1 );
            /* SC-VBR: 0 - PPP_NB, 1 - PPP_WB, 2 - NELP_NB, 3 - NELP_WB */
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF ( sub(coder_type,VOICED) == 0 && sub(st_fx->bwidth_fx,NB) == 0 && sub(st_fx->ppp_mode_fx,1) == 0 )
            {
                push_indice_fx( st_fx, IND_PPP_NELP_MODE, 0, 2 );
            }
            ELSE IF ( sub(coder_type,VOICED) == 0 && sub(st_fx->bwidth_fx,NB) != 0 && sub(st_fx->ppp_mode_fx,1) == 0 )
            {
                push_indice_fx( st_fx, IND_PPP_NELP_MODE, 1, 2 );
            }
            ELSE IF ( sub(coder_type,UNVOICED) == 0 && sub(st_fx->bwidth_fx,NB) == 0 && sub(st_fx->nelp_mode_fx,1) == 0 )
            {
                push_indice_fx( st_fx, IND_PPP_NELP_MODE, 2, 2);
            }
            ELSE IF ( sub(coder_type,UNVOICED) == 0 && sub(st_fx->bwidth_fx,NB) != 0 && sub(st_fx->nelp_mode_fx,1) == 0 )
            {
                push_indice_fx( st_fx, IND_PPP_NELP_MODE, 3, 2 );
            }
        }
        ELSE IF( L_sub(st_fx->core_brate_fx,SID_2k40) != 0 && L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) != 0 )
        {
            /* write the ACELP/HQ core selection bit */
            IF (L_sub(st_fx->total_brate_fx,ACELP_24k40) >= 0 )
            {
                push_indice_fx( st_fx, IND_CORE, 0, 1 );
            }

            /* find the section in the ACELP signalling table corresponding to bitrate */
            idx = 0;
            WHILE ( L_sub(acelp_sig_tbl[idx],st_fx->total_brate_fx) != 0 )
            {
                idx++;
            }

            /* retrieve the number of bits for signalling */
            nBits = (short) acelp_sig_tbl[++idx];

            /* retrieve the signalling index */
            start_idx = ++idx;
            k = SIG2IND_fx(coder_type, st_fx->bwidth_fx, sharpFlag, st_fx->rf_mode);
            WHILE( L_sub(acelp_sig_tbl[idx], k) != 0 )
            {
                idx++;
            }

            push_indice_fx( st_fx, IND_ACELP_SIGNALLING, idx - start_idx, nBits );
        }

        /* write extension layer flag to distinguish between TBE (0) and BWE (1) */
        IF( st_fx->extl_brate_fx > 0 )
        {
            test();
            test();
            test();
            test();
            IF( sub(st_fx->extl_fx,WB_TBE) == 0 || sub(st_fx->extl_fx,SWB_TBE) == 0 || sub(st_fx->extl_fx,FB_TBE) == 0 )
            {
                push_indice_fx( st_fx, IND_BWE_FLAG, 0, 1 );
            }
            ELSE IF( sub(st_fx->extl_fx,WB_BWE) == 0 || sub(st_fx->extl_fx,SWB_BWE) == 0 || sub(st_fx->extl_fx,FB_BWE) == 0 )
            {
                push_indice_fx( st_fx, IND_BWE_FLAG, 1, 1 );
            }
        }
    }
    ELSE  /* HQ core */
    {
        /* write ACELP->HQ switching frame flag */
        test();
        IF( sub(st_fx->last_core_fx,ACELP_CORE) == 0 || sub(st_fx->last_core_fx,AMR_WB_CORE) == 0 )
        {
            push_indice_fx( st_fx, IND_HQ_SWITCHING_FLG, 1, 1 );
            /* write ACELP L_frame info */
            IF( sub(st_fx->last_L_frame_fx, L_FRAME)==0 )
            {
                push_indice_fx( st_fx, IND_LAST_L_FRAME, 0, 1 );
            }
            ELSE
            {
                push_indice_fx( st_fx, IND_LAST_L_FRAME, 1, 1 );
            }
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_HQ_SWITCHING_FLG, 0, 1 );
        }

        /* HQ/TCX core switching flag */
        push_indice_fx( st_fx, IND_MDCT_CORE, 0, 1 );

        /* Use ACELP signaling for LR MDCT */
        IF ( L_sub(st_fx->total_brate_fx,ACELP_16k40) <= 0 )
        {
            /* find the section in the ACELP signalling table corresponding to bitrate */
            idx = 0;
            WHILE ( L_sub(acelp_sig_tbl[idx],st_fx->total_brate_fx) != 0 )
            {
                idx++;
            }

            /* retrieve the number of bits for signalling */
            nBits = extract_l(acelp_sig_tbl[++idx]);

            /* retrieve the signalling index */
            start_idx = ++idx;
            move16();
            k = SIG2IND_fx(LR_MDCT, st_fx->bwidth_fx, 0, 0);
            WHILE( L_sub(acelp_sig_tbl[idx], k) != 0 )
            {
                idx++;
            }

            push_indice_fx( st_fx, IND_ACELP_SIGNALLING, idx - start_idx, nBits );
        }
        ELSE
        {

            IF( L_sub(st_fx->core_brate_fx,ACELP_64k) <= 0 )
            {
                /* write ACELP/HQ core indication flag */
                push_indice_fx( st_fx, IND_CORE, 1, 1 );
            }

            /* write band-width (needed for different I/O sampling rate support) */
            IF( sub(st_fx->bwidth_fx,NB) == 0 )
            {
                push_indice_fx( st_fx, IND_HQ_BWIDTH, 0, 2 );
            }
            ELSE IF( sub(st_fx->bwidth_fx,WB) == 0 )
            {
                push_indice_fx( st_fx, IND_HQ_BWIDTH, 1, 2 );
            }
            ELSE IF( sub(st_fx->bwidth_fx,SWB) == 0 )
            {
                push_indice_fx( st_fx, IND_HQ_BWIDTH, 2, 2 );
            }
            ELSE  /* st_fx->bwidth_fx == FB */
            {
                push_indice_fx( st_fx, IND_HQ_BWIDTH, 3, 2 );
            }
        }
    }

    return;
}

/*---------------------------------------------------------------------*
 * signalling_enc_rf()
 *
 * write channel-aware signalling information into the bit-stream
 *---------------------------------------------------------------------*/

void signalling_enc_rf(
    Encoder_State_fx *st        /* i  : encoder state structure   */
)
{
    Word16 i, tmp_rf;
    Word16 sfr;


    /* write partial copy into bitstream */
    IF(sub(st->rf_mode,1) == 0)
    {
        enc_prm_rf(st, st->rf_indx_frametype[st->rf_fec_offset], st->rf_fec_offset);
        st->rf_indx_tbeGainFr[0] = st->rf_bwe_gainFr_ind;
    }

    /* Shift the RF indices such that the partial copy associated with
       (n-fec_offset)th frame is included in the bitstream in nth frame. */
    tmp_rf = st->rf_fec_offset;

    FOR(i = tmp_rf; i >= 0 ; i--)
    {
        /* rf frame type */
        st->rf_indx_frametype[i+1] = st->rf_indx_frametype[i];
        /* rf target bits buffer */
        st->rf_targetbits_buff[i+1] = st->rf_targetbits_buff[i];

        /* lsf indx */
        st->rf_indx_lsf[i+1][0] = st->rf_indx_lsf[i][0];
        st->rf_indx_lsf[i+1][1] = st->rf_indx_lsf[i][1];
        st->rf_indx_lsf[i+1][2] = st->rf_indx_lsf[i][2];

        /* ES pred energy */
        st->rf_indx_EsPred[i+1] = st->rf_indx_EsPred[i];

        /* LTF mode, sfr params: pitch, fcb and gain */
        FOR(sfr = 0; sfr < st->nb_subfr; sfr++)
        {
            st->rf_indx_ltfMode[i+1][sfr] = st->rf_indx_ltfMode[i][sfr];
            st->rf_indx_pitch[i+1][sfr] = st->rf_indx_pitch[i][sfr];
            st->rf_indx_fcb[i+1][sfr] = st->rf_indx_fcb[i][sfr];
            st->rf_indx_gain[i+1][sfr] = st->rf_indx_gain[i][sfr];
        }

        /* shift the nelp indices */
        st->rf_indx_nelp_iG1[i+1] = st->rf_indx_nelp_iG1[i];
        st->rf_indx_nelp_iG2[i+1][0] = st->rf_indx_nelp_iG2[i][0];
        st->rf_indx_nelp_iG2[i+1][1] = st->rf_indx_nelp_iG2[i][1];
        st->rf_indx_nelp_fid[i+1] = st->rf_indx_nelp_fid[i];

        /* tbe gain Fr shift */
        st->rf_indx_tbeGainFr[i+1] = st->rf_indx_tbeGainFr[i];
        st->rf_clas[i+1] = st->rf_clas[i];
        st->rf_gain_tcx[i+1] = st->rf_gain_tcx[i];
        st->rf_tcxltp_param[i+1] = st->rf_tcxltp_param[i];
    }

    return;
}


