/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "stat_dec_fx.h"
#include "rom_com_fx.h"
#include "stl.h"


/*-----------------------------------------------------------------*
 * decision_matrix_dec()
 *
 * ACELP/HQ core selection
 * Read ACELP signalling bits from the bitstream
 * Set extension layers
 *-----------------------------------------------------------------*/

void decision_matrix_dec_fx(
    Decoder_State_fx *st,                /* i/o: decoder state structure                   */
    Word16 *coder_type,        /* o  : coder type                                */
    Word16 *sharpFlag,         /* o  : formant sharpening flag                   */
    Word16 *hq_core_type,      /* o  : HQ core type                              */
    Word16 *core_switching_flag/* o  : ACELP->HQ switching frame flag            */
)
{
    Word16 start_idx;
    Word16 ppp_nelp_mode;
    Word32 ind;
    Word16 nBits;
    Word16 temp_core, temp_extl, temp_bwidth, temp_ppp_mode_dec, temp_nelp_mode_dec, temp_L_Frame;
    Word16 temp_codertype;
    Word32 temp_core_brate, temp_extl_brate;
    Word16 tmp16;

    temp_core = st->core_fx;
    move16();
    temp_core_brate = L_add(st->core_brate_fx, 0);
    temp_extl = st->extl_fx;
    move16();
    temp_extl_brate = st->extl_brate_fx;
    move16();
    temp_ppp_mode_dec = st->ppp_mode_dec_fx;
    move16();
    temp_nelp_mode_dec = st->nelp_mode_dec_fx;
    move16();
    temp_bwidth = st->bwidth_fx;
    move16();
    temp_L_Frame = st->L_frame_fx;
    move16();
    temp_codertype = *coder_type;
    move16();

    st->core_fx = -1;
    move16();
    st->core_brate_fx = L_deposit_l(0);
    st->extl_fx = -1;
    move16();
    st->extl_brate_fx = 0;
    move16();
    st->ppp_mode_dec_fx = 0;
    move16();
    st->nelp_mode_dec_fx = 0;
    move16();
    st->igf = 0;
    move16();

    if( L_sub(st->total_brate_fx,ACELP_8k00) > 0 )
    {
        st->vbr_hw_BWE_disable_dec_fx = 0;
        move16();
    }

    IF (sub(st->mdct_sw, MODE2) == 0)
    {
        st->core_fx = HQ_CORE;
        move16();
    }
    ELSE
    {
        test();
        IF( L_sub(st->total_brate_fx,FRAME_NO_DATA) == 0 || L_sub(st->total_brate_fx,SID_2k40) == 0 )
        {
            st->core_fx = ACELP_CORE;
            move16();
            st->core_brate_fx = st->total_brate_fx;
            move32();

            test();
            IF( L_sub(st->total_brate_fx,FRAME_NO_DATA) != 0 && sub(st->bfi_fx,2) != 0 )
            {
                st->cng_type_fx = get_next_indice_fx( st, 1 );

                IF( sub(st->cng_type_fx,LP_CNG) == 0 )
                {
                    st->L_frame_fx = L_FRAME;
                    move16();

                    tmp16 = get_next_indice_fx( st, 1 );
                    if( sub(tmp16,1) == 0 )
                    {
                        st->L_frame_fx = L_FRAME16k;
                        move16();
                    }
                }
                ELSE
                {
                    st->bwidth_fx = get_next_indice_fx(st, 2);

                    tmp16 = get_next_indice_fx(st, 1);
                    move16();

                    st->L_frame_fx = L_FRAME16k;
                    move16();
                    if( tmp16 == 0 )
                    {
                        st->L_frame_fx = L_FRAME;
                        move16();
                    }
                }
            }

            test();
            if( L_sub(st->output_Fs_fx,32000) >= 0 && sub(st->bwidth_fx,SWB) >= 0 )
            {
                st->extl_fx = SWB_CNG;
                move16();
            }

            IF( sub(st->bfi_fx,2) == 0 )
            {
                st->bfi_fx = 1;
                move16(); /* future frame N+FEC_OFFSET does not contain partial copy of current frame N, so erase current frame N */
                st->use_partial_copy = 0;
                move16();

                /* Reset parameters in case of FER */
                st->core_fx = temp_core;
                move16();
                st->core_brate_fx = temp_core_brate;
                move32();
                st->extl_fx = temp_extl;
                move16();
                st->extl_brate_fx = temp_extl_brate;
                move16();
                st->ppp_mode_dec_fx = temp_ppp_mode_dec;
                move16();
                st->nelp_mode_dec_fx = temp_nelp_mode_dec;
                move16();
                st->bwidth_fx = temp_bwidth;
                move16();
                st->L_frame_fx = temp_L_Frame;
                move16();
                *sharpFlag = 0;
                move16();
                *coder_type = temp_codertype;
                move16();
            }

            test();
            test();
            test();
            if( L_sub(st->total_brate_fx,FRAME_NO_DATA) == 0 && st->prev_bfi_fx && !st->bfi_fx && sub(st->L_frame_fx,L_FRAME16k) > 0 )
            {
                st->L_frame_fx = st->last_CNG_L_frame_fx;
                move16();
            }

            return;
        }

        /* SC-VBR */
        ELSE IF( L_sub(st->total_brate_fx,PPP_NELP_2k80) == 0 )
        {
            st->core_fx = ACELP_CORE;
            move16();
            st->core_brate_fx = PPP_NELP_2k80;
            move32();
            st->L_frame_fx = L_FRAME;
            move16();
            st->fscale = sr2fscale(INT_FS_FX);
            move16();

            IF ( st->ini_frame_fx == 0 )
            {
                /* avoid switching of internal ACELP Fs in the very first frame */
                st->last_L_frame_fx = st->L_frame_fx;
                st->last_core_fx = st->core_fx;
                st->last_core_brate_fx = st->core_brate_fx;
                st->last_extl_fx = st->extl_fx;
            }

            st->vbr_hw_BWE_disable_dec_fx = 1;
            move16();
            get_next_indice_fx( st, 1 );

            ppp_nelp_mode = get_next_indice_fx( st, 2 );

            /* 0 - PPP_NB, 1 - PPP_WB, 2 - NELP_NB, 3 - NELP_WB */
            IF( ppp_nelp_mode == 0 )
            {
                st->ppp_mode_dec_fx = 1;
                move16();
                *coder_type = VOICED;
                move16();
                st->bwidth_fx = NB;
                move16();
            }
            ELSE IF( sub(ppp_nelp_mode,1) == 0 )
            {
                st->ppp_mode_dec_fx = 1;
                move16();
                *coder_type = VOICED;
                move16();
                st->bwidth_fx = WB;
                move16();
            }
            ELSE IF( sub(ppp_nelp_mode,2) == 0 )
            {
                st->nelp_mode_dec_fx = 1;
                move16();
                *coder_type = UNVOICED;
                move16();
                st->bwidth_fx = NB;
                move16();
            }
            ELSE IF( sub(ppp_nelp_mode,3) == 0 )
            {
                st->nelp_mode_dec_fx = 1;
                move16();
                *coder_type = UNVOICED;
                move16();
                st->bwidth_fx = WB;
                move16();
            }


            return;
        }

        /*---------------------------------------------------------------------*
         * ACELP/HQ core selection
         *---------------------------------------------------------------------*/

        test();
        IF( L_sub(st->total_brate_fx,ACELP_24k40) < 0 )
        {
            st->core_fx = ACELP_CORE;
            move16();
        }
        ELSE IF( L_sub(st->total_brate_fx,ACELP_24k40) >= 0 && L_sub(st->total_brate_fx,ACELP_64k) <= 0 )
        {
            /* read the ACELP/HQ core selection bit */
            temp_core = get_next_indice_fx( st, 1 );

            st->core_fx = HQ_CORE;
            move16();
            if( temp_core == 0 )
            {
                st->core_fx = ACELP_CORE;
                move16();
            }
        }
        ELSE
        {
            st->core_fx = HQ_CORE;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Read ACELP signalling bits from the bitstream
     *-----------------------------------------------------------------*/
    test();
    IF( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->bfi_fx,2) !=0 )
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        start_idx = 0;
        move16();
        WHILE( L_sub(acelp_sig_tbl[start_idx],st->total_brate_fx) != 0 )
        {
            start_idx = add(start_idx,1);
            assert((start_idx < MAX_ACELP_SIG) && "ERROR: start_idx larger than acelp_sig_tbl[].\n");
        }

        /* skip the bitrate */
        start_idx = add(start_idx,1);

        /* retrieve the number of bits */
        nBits = extract_l(acelp_sig_tbl[start_idx]);
        start_idx = add(start_idx,1);

        /* retrieve the signalling indice */
        ind = acelp_sig_tbl[start_idx + get_next_indice_fx( st, nBits )];

        /* convert signalling indice into signalling information */
        *coder_type = extract_l(L_and(ind,0x7L));

        IF( sub(*coder_type,LR_MDCT) == 0 )
        {
            st->core_fx = HQ_CORE;
            move16();
            st->bwidth_fx = extract_l(L_shr(ind,3) & 0x7L);
        }
        ELSE
        {
            st->bwidth_fx = extract_l(L_and(L_shr(ind,3),0x7L));
            *sharpFlag = extract_l(L_and(L_shr(ind,6),0x1L));

        }
    }

    IF( sub(st->bfi_fx,2) == 0 ) /* future frame N+FEC_OFFSET may contain partial copy of current frame */
    {
        IF( st->rf_flag )    /* future frame N+FEC_OFFSET contains partial copy of current frame N */
        {
            IF( sub(st->bwidth_fx,WB) == 0 )
            {
                st->extl_fx = WB_TBE;
                move16();

                IF( L_sub(st->total_brate_fx,ACELP_13k20) == 0 )
                {
                    st->extl_brate_fx = WB_TBE_0k35;
                    move16();
                    get_next_indice_tmp_fx( st, 188);  /* parse through the remaining bits in the core packet to read the RF packet */
                }
                ELSE IF( L_sub(st->total_brate_fx,ACELP_16k40) == 0 )
                {
                    st->extl_brate_fx = WB_TBE_0k65;
                    move16();
                    get_next_indice_tmp_fx( st, 251);  /* parse through the remaining bits in the core packet to read the RF packet */
                }
            }
            ELSE IF( sub(st->bwidth_fx,SWB) == 0 )
            {
                st->extl_fx = SWB_TBE;
                move16();

                IF( L_sub(st->total_brate_fx,ACELP_13k20) == 0 )
                {
                    st->extl_brate_fx = SWB_TBE_1k6;
                    move16();
                    get_next_indice_tmp_fx( st, 188);  /* parse through the remaining bits in the core packet to read the RF packet */
                }
                ELSE IF( L_sub(st->total_brate_fx,ACELP_16k40) == 0 )
                {
                    st->extl_brate_fx = SWB_TBE_1k2;
                    move16();
                    get_next_indice_tmp_fx( st, 252);  /* parse through the remaining bits in the core packet to read the RF packet */
                }
            }

            st->total_brate_fx = ACELP_3k60;
            move32();
            st->bfi_fx = 0;
            move16();
            st->nbLostCmpt = 0;
            move16();
            st->use_partial_copy=1;
            move16();

        }
        ELSE
        {
            st->bfi_fx = 1;
            move16();/* future frame N+FEC_OFFSET does not contain partial copy of current frame N, so erase current frame N */
            st->use_partial_copy=0;
            move16();

            /* Reset the parameters in case of FER */
            st->core_fx = temp_core;
            move16();
            st->core_brate_fx = temp_core_brate;
            move16();
            st->extl_fx = temp_extl;
            move16();
            st->extl_brate_fx = temp_extl_brate;
            move16();
            st->ppp_mode_dec_fx = temp_ppp_mode_dec;
            move16();
            st->nelp_mode_dec_fx = temp_nelp_mode_dec;
            move16();
            st->bwidth_fx = temp_bwidth;
            move16();
            st->L_frame_fx = temp_L_Frame;
            move16();
            *sharpFlag = 0;
            move16();
            *coder_type = temp_codertype;
        }

        return;
    }

    /*-----------------------------------------------------------------*
     * Set extension layers
     *-----------------------------------------------------------------*/

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
    test();
    test();
    test();
    IF( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->bwidth_fx,WB) == 0 && L_sub(st->total_brate_fx,ACELP_9k60) < 0 )
    {
        if( st->vbr_hw_BWE_disable_dec_fx == 0 )
        {
            st->extl_fx = WB_BWE;
            move16();
        }
    }
    ELSE IF( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->bwidth_fx,WB) == 0 && L_sub(st->total_brate_fx,ACELP_16k40) <= 0 && sub(st->rf_flag,1) == 0 )
    {
        /* read the WB TBE/BWE selection bit */
        tmp16 = get_next_indice_fx( st, 1 );
        IF( tmp16 )
        {
            st->extl_fx = WB_BWE;
            move16();
            st->extl_brate_fx = WB_BWE_0k35;
            move16();
        }
        ELSE
        {
            st->extl_fx = WB_TBE;
            move16();
            st->extl_brate_fx = WB_TBE_0k65;
            move32();
            if( L_sub(st->total_brate_fx,ACELP_9k60) == 0 )
            {
                st->extl_brate_fx = WB_TBE_0k35;
                move32();
            }
        }
    }
    ELSE IF( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->bwidth_fx,WB) == 0 && L_sub(st->total_brate_fx,ACELP_9k60) >= 0 && L_sub(st->total_brate_fx,ACELP_16k40) <= 0 )
    {
        /* read the WB TBE/BWE selection bit */
        tmp16 = get_next_indice_fx( st, 1 );
        IF( sub(tmp16,1) == 0 )
        {
            st->extl_fx = WB_BWE;
            move16();
            st->extl_brate_fx = WB_BWE_0k35;
            move32();
        }
        ELSE
        {
            st->extl_fx = WB_TBE;
            move16();
            st->extl_brate_fx = WB_TBE_1k05;
            move32();
            if( L_sub(st->total_brate_fx,ACELP_9k60) == 0 )
            {
                st->extl_brate_fx = WB_TBE_0k35;
                move32();
            }
        }
    }
    ELSE IF( sub(st->core_fx,ACELP_CORE) == 0 && sub(st->bwidth_fx,SWB) == 0 && L_sub(st->total_brate_fx,ACELP_13k20) < 0 && sub(st->rf_flag,1) == 0 )
    {
        get_next_indice_tmp_fx( st, 1 );

        st->extl_fx = SWB_TBE;
        move16();
        st->extl_brate_fx = SWB_TBE_1k2;
        move32();
        if( L_sub(st->total_brate_fx,ACELP_9k60) == 0 )
        {
            st->extl_brate_fx = SWB_TBE_1k6;
            move32();
        }
    }

    ELSE IF( sub(st->core_fx,ACELP_CORE) == 0 && (sub(st->bwidth_fx,SWB) == 0 || sub(st->bwidth_fx,FB) == 0) && L_sub(st->total_brate_fx,ACELP_13k20) >= 0 )
    {
        IF( L_sub(st->total_brate_fx,ACELP_48k) >=0 )
        {
            st->extl_fx = SWB_BWE_HIGHRATE;
            move16();
            if( sub(st->bwidth_fx,FB) == 0 )
            {
                st->extl_fx = FB_BWE_HIGHRATE;
                move16();
            }

            st->extl_brate_fx = SWB_BWE_16k;
            move32();
        }

        /* read the SWB TBE/BWE selection bit */
        ELSE
        {
            tmp16 = get_next_indice_fx( st, 1 );
            IF( tmp16 )
            {
                st->extl_fx = SWB_BWE;
                move16();
                st->extl_brate_fx = SWB_BWE_1k6;
                move32();
            }
            ELSE
            {
                st->extl_fx = SWB_TBE;
                move16();
                st->extl_brate_fx = SWB_TBE_1k6;
                move32();
                if( L_sub(st->total_brate_fx,ACELP_24k40) >= 0 )
                {
                    st->extl_brate_fx = SWB_TBE_2k8;
                    move32();
                }
            }
        }

        /* set FB TBE and FB BWE extension layers */
        test();
        IF( sub(st->bwidth_fx,FB) == 0 && L_sub(st->total_brate_fx,ACELP_24k40) >= 0 )
        {
            IF( sub(st->extl_fx,SWB_BWE) == 0 )
            {
                st->extl_fx = FB_BWE;
                move16();
                st->extl_brate_fx = FB_BWE_1k8;
                move32();
            }
            ELSE IF( sub(st->extl_fx,SWB_TBE) == 0 )
            {
                st->extl_fx = FB_TBE;
                move16();
                st->extl_brate_fx = FB_TBE_1k8;
                move32();
                if( L_sub(st->total_brate_fx,ACELP_24k40) >= 0 )
                {
                    st->extl_brate_fx = FB_TBE_3k0;
                    move32();
                }
            }
        }
    }

    /* set core bitrate */
    st->core_brate_fx = L_sub(st->total_brate_fx,st->extl_brate_fx);

    /*-----------------------------------------------------------------*
     * Read HQ signalling bits from the bitstream
     * Set HQ core type
     *-----------------------------------------------------------------*/

    IF( sub(st->core_fx,HQ_CORE) == 0 )
    {
        IF( sub(st->mdct_sw, MODE2) != 0 )
        {
            /* skip the HQ/TCX core switching flag */
            get_next_indice_tmp_fx( st, 1 );
        }

        /* read ACELP->HQ core switching flag */
        *core_switching_flag = get_next_indice_fx( st, 1 );

        IF( sub(*core_switching_flag,1) == 0 )
        {
            st->last_L_frame_ori_fx = st->last_L_frame_fx;
            move16();

            /* read ACELP L_frame info */
            st->last_L_frame_fx = L_FRAME16k;
            move16();
            tmp16 = get_next_indice_fx( st, 1 );
            if( tmp16 == 0 )
            {
                st->last_L_frame_fx = L_FRAME;
                move16();
            }
        }

        IF( sub(st->mdct_sw, MODE2) != 0 )
        {

            /* read/set band-width (needed for different I/O sampling rate support) */
            IF( L_sub(st->total_brate_fx,ACELP_16k40) > 0 )
            {
                tmp16 = get_next_indice_fx( st, 2 );

                IF( tmp16 == 0 )
                {
                    st->bwidth_fx = NB;
                    move16();
                }
                ELSE IF( sub(tmp16,1) == 0 )
                {
                    st->bwidth_fx = WB;
                    move16();
                }
                ELSE IF( sub(tmp16,2) == 0 )
                {
                    st->bwidth_fx = SWB;
                    move16();
                }
                ELSE
                {
                    st->bwidth_fx = FB;
                    move16();
                }
            }
        }

        /* set HQ core type */
        *hq_core_type = NORMAL_HQ_CORE;
        move16();

        test();
        test();
        IF( (sub(st->bwidth_fx,SWB) == 0 || sub(st->bwidth_fx,WB) == 0) && L_sub(st->total_brate_fx,LRMDCT_CROSSOVER_POINT) <= 0 )
        {
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
        ELSE IF( sub(st->bwidth_fx,NB) == 0 )
        {
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Set ACELP frame lnegth
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( L_sub(st->core_brate_fx,FRAME_NO_DATA) == 0 )
    {
        /* prevent "L_frame" changes in CNG segments */
        st->L_frame_fx = st->last_L_frame_fx;
        move16();
    }
    ELSE IF( L_sub(st->core_brate_fx,SID_2k40) == 0 && sub(st->bwidth_fx,WB) == 0 && st->first_CNG_fx && sub(st->act_cnt2_fx,MIN_ACT_CNG_UPD) < 0 )
    {
        /* prevent "L_frame" changes in SID frame after short segment of active frames */
        st->L_frame_fx = st->last_CNG_L_frame_fx;
        move16();
    }
    ELSE IF( ( L_sub(st->core_brate_fx,SID_2k40) == 0 && L_sub(st->total_brate_fx,ACELP_9k60) >= 0 && sub(st->bwidth_fx,WB) == 0 ) ||
             ( L_sub(st->total_brate_fx,ACELP_24k40) > 0 && L_sub(st->total_brate_fx,HQ_96k) < 0 ) || ( L_sub(st->total_brate_fx,ACELP_24k40) == 0 && sub(st->bwidth_fx,WB) >= 0 ) )
    {
        st->L_frame_fx = L_FRAME16k;
        move16();
    }
    ELSE
    {
        st->L_frame_fx = L_FRAME;
        move16();
    }

    st->nb_subfr = NB_SUBFR;
    move16();
    if ( sub(st->L_frame_fx,L_FRAME16k) == 0)
    {
        st->nb_subfr = NB_SUBFR16k;
        move16();
    }

    test();
    IF( L_sub(st->output_Fs_fx,8000) == 0 )
    {
        st->extl_fx = -1;
        move16();
    }
    ELSE IF( L_sub(st->output_Fs_fx,16000) == 0 && sub(st->L_frame_fx,L_FRAME16k) == 0 )
    {
        st->extl_fx = -1;
        move16();
        st->extl_brate_fx = L_deposit_l(0);
    }

    IF( st->ini_frame_fx == 0 )
    {
        /* avoid switching of internal ACELP Fs in the very first frame */
        st->last_L_frame_fx = st->L_frame_fx;
        move16();
        st->last_core_fx = st->core_fx;
        move16();
        st->last_core_brate_fx = st->core_brate_fx;
        move32();
        st->last_extl_fx = st->extl_fx;
        move16();
    }

    return;
}
