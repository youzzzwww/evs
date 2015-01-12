/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "rom_com_fx.h"
#include "stat_dec_fx.h"
#include "stl.h"
#include "options.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"






/********************************
* External tables               *
********************************/

extern const Word16 T_DIV_L_Frame[];           /* format: 0Q15 * 2^-7 */


/*
   createFdCngDec

    Parameters:

    hFdCngDec              i/0  : pointer to cng decoder structure

    Function:
    create an instance of type FD_CNG
*/
void createFdCngDec (HANDLE_FD_CNG_DEC *hFdCngDec)
{
    HANDLE_FD_CNG_DEC hs;

    /* Allocate memory */
    hs = (HANDLE_FD_CNG_DEC) calloc(1, sizeof (FD_CNG_DEC));


    createFdCngCom(&(hs->hFdCngCom));

    *hFdCngDec = hs;
}

Word16 initFdCngDec (HANDLE_FD_CNG_DEC hs, Word16 scale)
{
    /* Initialize common */

    initFdCngCom( hs->hFdCngCom, scale );
    set16_fx( hs->olapBufferAna, 0, 320 );
    hs->hFdCngCom->olapBufferAna    = hs->olapBufferAna;
    move16();
    set16_fx( hs->olapBufferSynth2, 0, FFTLEN );
    hs->hFdCngCom->olapBufferSynth2 = hs->olapBufferSynth2;
    move16();

    /* Set some counters and flags */

    hs->flag_dtx_mode = 0;
    move16();
    hs->lp_noise  = FL2WORD32_SCALE(-20.f,8);  /* format: Q8.24 */
    hs->lp_speech = FL2WORD32_SCALE( 25.f,8);  /* format: Q8.24 */

    /* Initialization of the noise estimation algorithm */

    set32_fx( hs->bandNoiseShape, 0, FFTLEN2 );
    set16_fx( &hs->bandNoiseShape_exp, 0, 1);

    set32_fx (hs->partNoiseShape, 0, NPART );
    set16_fx( &hs->partNoiseShape_exp, 0, 1);

    set32_fx( hs->msPeriodog, 0, NPART_SHAPING );
    set16_fx( &hs->msPeriodog_exp, 0, 1);

    set32_fx( hs->msAlpha, 0, NPART_SHAPING );

    set32_fx( hs->msBminWin, 0, NPART_SHAPING );

    set32_fx( hs->msBminSubWin, 0, NPART_SHAPING );

    set16_fx( hs->msPsd, 0, NPART_SHAPING );
    set16_fx( hs->msNoiseFloor, 0, NPART_SHAPING );

    set32_fx( hs->msNoiseEst, 0, NPART_SHAPING );
    set16_fx( &hs->msNoiseEst_exp, 0, 1);

    set32_fx( hs->msMinBuf, FL2WORD32(1.0), MSNUMSUBFR*NPART_SHAPING );

    set32_fx( hs->msCurrentMin, FL2WORD32(1.0), NPART_SHAPING );

    set32_fx( hs->msCurrentMinOut, FL2WORD32(1.0), NPART_SHAPING );

    set32_fx( hs->msCurrentMinSubWindow, FL2WORD32(1.0), NPART_SHAPING );

    set16_fx( hs->msLocalMinFlag, 0, NPART_SHAPING  );
    set16_fx( hs->msNewMinFlag, 0, NPART_SHAPING );

    set16_fx( hs->msPsdFirstMoment, 0, NPART_SHAPING );

    set32_fx( hs->msPsdSecondMoment, 0, NPART_SHAPING );
    set16_fx( hs->msPeriodogBuf, 0, MSBUFLEN*NPART_SHAPING );

    hs->msPeriodogBufPtr = 0;
    move16();

    set16_fx( hs->msLogPeriodog, 0, NPART_SHAPING );
    set16_fx( hs->msLogNoiseEst, 0, NPART_SHAPING );

    return 0;
}

/*
   configureFdCngDec

    Parameters:

    hs                      i/o: Contains the variables related to the FD-based CNG process
    numSlots                i  : Number of time slots in CLDFB matrix
    numCoreBands            i  : Number of core bands
    regularStopBand         i  : Number of CLDFB bands to be considered
    CLDFBscale                i  : cldfb scale factor

    Function:
    configure FD_CNG

    Returns:
    void
*/
void configureFdCngDec (HANDLE_FD_CNG_DEC hsDec,   /* i/o: Contains the variables related to the CLDFB-based CNG process */
                        Word16 bandwidth,
                        Word32 bitrate,
                        Word16 L_frame
                       )
{
    Word16 j, stopBandFR;
    HANDLE_FD_CNG_COM hsCom = hsDec->hFdCngCom;


    hsCom->CngBandwidth = bandwidth;
    IF ( sub( hsCom->CngBandwidth, FB ) == 0 )
    {
        hsCom->CngBandwidth = SWB;
    }
    test();
    if ( bitrate != FRAME_NO_DATA && L_sub(bitrate, SID_2k40) != 0 )
    {
        hsCom->CngBitrate = L_add(bitrate, 0);
    }
    hsCom->numSlots = 16;
    move16();

    /* NB configuration */
    IF ( bandwidth == NB )
    {
        hsCom->FdCngSetup = FdCngSetup_nb;
        hsCom->numCoreBands = 16;
        move16();
        hsCom->regularStopBand = 16;
        move16();
    }

    /* WB configuration */
    ELSE IF ( sub(bandwidth, WB) == 0 )
    {
        /* FFT 6.4kHz, no CLDFB */
        test();
        test();
        IF ( L_sub(hsCom->CngBitrate, ACELP_8k00) <= 0
             && sub(L_frame,L_FRAME)==0
           )
        {
            hsCom->FdCngSetup = FdCngSetup_wb1;
            hsCom->numCoreBands = 16;
            move16();
            hsCom->regularStopBand = 16;
            move16();
        }
        /* FFT 6.4kHz, CLDFB 8.0kHz */
        ELSE IF ( L_sub(hsCom->CngBitrate, ACELP_13k20) <= 0
                  || sub(L_frame,L_FRAME)==0
                )
        {
            hsCom->FdCngSetup = FdCngSetup_wb2;
            hsCom->numCoreBands = 16;
            move16();
            hsCom->regularStopBand = 20;
            move16();
            IF (
                sub(L_frame,L_FRAME16k)==0
            )
            {
                hsCom->FdCngSetup = FdCngSetup_wb2;
                hsCom->numCoreBands = 20;
                move16();
                hsCom->regularStopBand = 20;
                move16();
                hsCom->FdCngSetup.fftlen = 640;
                move16();
                hsCom->FdCngSetup.stopFFTbin = 256;
                move16();
            }
        }
        /* FFT 8.0kHz, no CLDFB */
        ELSE
        {
            hsCom->FdCngSetup = FdCngSetup_wb3;
            hsCom->numCoreBands = 20;
            move16();
            hsCom->regularStopBand = 20;
            move16();
        }
    }

    /* SWB/FB configuration */
    ELSE
    {
        /* FFT 6.4kHz, CLDFB 14kHz */
        IF (
            sub(L_frame,L_FRAME)==0
        )
        {
            hsCom->FdCngSetup = FdCngSetup_swb1;
            hsCom->numCoreBands = 16;
            move16();
            hsCom->regularStopBand = 35;
            move16();
        }
        /* FFT 8.0kHz, CLDFB 16kHz */
        ELSE
        {
            hsCom->FdCngSetup = FdCngSetup_swb2;
            hsCom->numCoreBands = 20;
            move16();
            hsCom->regularStopBand = 40;
            move16();
        }
    }
    hsCom->fftlen = hsCom->FdCngSetup.fftlen;
    move16();
    hsCom->stopFFTbin = hsCom->FdCngSetup.stopFFTbin;
    move16();

    /* Configure the SID quantizer and the Confort Noise Generator */

    hsCom->startBand = 2;
    move16();
    hsCom->stopBand = add( hsCom->FdCngSetup.sidPartitions[hsCom->FdCngSetup.numPartitions-1], 1 );
    initPartitions(hsCom->FdCngSetup.sidPartitions, hsCom->FdCngSetup.numPartitions, hsCom->startBand, hsCom->stopBand, hsCom->part, &hsCom->npart, hsCom->midband, hsCom->psize, hsCom->psize_norm, &hsCom->psize_norm_exp, hsCom->psize_inv, 0);

    hsCom->nFFTpart = 21;
    move16();
    if ( sub(hsCom->stopFFTbin, 256) == 0 )
    {
        hsCom->nFFTpart = 20;
        move16();
    }
    if ( sub(hsCom->stopFFTbin, 160) == 0 )
    {
        hsCom->nFFTpart = 17;
        move16();
    }

    hsCom->nCLDFBpart = sub( hsCom->npart, hsCom->nFFTpart );
    FOR(j=0; j<hsCom->nCLDFBpart; j++)
    {
        hsCom->CLDFBpart[j] = sub( hsCom->part[j+hsCom->nFFTpart], (hsCom->stopFFTbin-hsCom->startBand) );
        move16();
        hsCom->CLDFBpsize_inv[j] = hsCom->psize_inv[j+hsCom->nFFTpart];
        move16();
    }

    stopBandFR = 1000/25;
    if ( sub(stopBandFR, hsCom->stopFFTbin) > 0 )
    {
        stopBandFR = hsCom->stopFFTbin;
        move16();
    }
    initPartitions(hsCom->FdCngSetup.shapingPartitions, hsCom->FdCngSetup.numShapingPartitions,
                   hsCom->startBand, hsCom->stopFFTbin, hsDec->part_shaping, &hsDec->npart_shaping, hsDec->midband_shaping,
                   hsDec->psize_shaping, hsDec->psize_shaping_norm, &hsDec->psize_shaping_norm_exp, hsDec->psize_inv_shaping,
                   stopBandFR );
    hsDec->nFFTpart_shaping = hsDec->npart_shaping;
    move16();

    SWITCH (hsCom->fftlen)
    {
    case 512:
        hsCom->fftlenShift = 8;
        move16();
        hsCom->fftlenFac = FL2WORD16(1.0);
        move16();
        BREAK;
    case 640:
        hsCom->fftlenShift = 9;
        move16();
        hsCom->fftlenFac = FL2WORD16(0.625);
        move16();
        BREAK;
    default:
        assert(!"Unsupported FFT length for FD-based CNG");
        BREAK;
    }
    BASOP_getTables( &hsCom->olapWinAna, NULL, NULL, shr(hsCom->fftlen, 1));
    BASOP_getTables( &hsCom->olapWinSyn, NULL, NULL, shr(hsCom->fftlen, 2));
    hsCom->frameSize = hsCom->fftlen >> 1;

}


/*
   deleteFdCngDec

    Parameters:

    hFdCngDec              i/0  : pointer to cng decoder structure

    Function:
    delete the instance of type FD_CNG

    Returns:
    void
*/
void deleteFdCngDec (HANDLE_FD_CNG_DEC *hFdCngDec)
{
    HANDLE_FD_CNG_DEC hsDec = *hFdCngDec;

    IF ( hsDec != NULL )
    {
        deleteFdCngCom (&(hsDec->hFdCngCom));
        free(hsDec);
        *hFdCngDec = NULL;
    }
}


/*
   ApplyFdCng

    Parameters:

    timeDomainInput,       i  : pointer to time domain input
    cldfbBufferReal          i/o: real part of the CLDFB buffer
    cldfbBufferImag          i/o: imaginary part of the CLDFB buffer
    cldfbBufferScale         o  : pointer to the scalefactor for real and imaginary part of the CLDFB buffer
    st                     i/o: pointer to FD_CNG structure containing all buffers and variables
    m_frame_type           i  : type of frame at the decoder side
    stcod                  i  : pointer to Coder_State structure
    stdec                  i  : pointer to Decoder_State structure
    bitrate                i  : bitrate
    concealWholeFrame      i  : binary flag indicating frame loss

    Function:
    apply the CLDFB-based CNG at the decoder

    Returns:
    error
*/
Word16 ApplyFdCng (Word16  *timeDomainInput,       /* i  : pointer to time domain input */
                   Word16 Q,
                   Word32 **cldfbBufferReal,         /* i/o: real part of the CLDFB buffer */
                   Word32 **cldfbBufferImag,         /* i/o: imaginary part of the CLDFB buffer */
                   Word16  *cldfbBufferScale,        /* o  : pointer to the scalefactor for real and imaginary part of the CLDFB buffer */
                   HANDLE_FD_CNG_DEC st,          /* i/o: pointer to FD_CNG structure containing all buffers and variables */
                   Word16 m_frame_type,            /* i  : type of frame at the decoder side */
                   Decoder_State_fx *stdec,
                   const Word16 concealWholeFrame  /* i  : binary flag indicating frame loss */
                   ,Word16 is_music
                  )
{
    Word16 j, k, nBins;
    Word16 s, s1, s2, num, denom;
    Word32 *cngNoiseLevel;
    Word16 *cngNoiseLevel_exp;
    Word32 L_tmp;
    Word16 L_tmp_exp;
    Word16 facTab[NPART];
    Word16 facTabExp[NPART];
    Word16 tmp_loop;
    Word32 L_c;
    Word16 lsp_cng[M];



    cngNoiseLevel = st->hFdCngCom->cngNoiseLevel;
    cngNoiseLevel_exp = &st->hFdCngCom->cngNoiseLevelExp;

    nBins = sub(st->hFdCngCom->stopFFTbin,st->hFdCngCom->startBand);

    SWITCH ( m_frame_type )
    {
    case ACTIVE_FRAME:

        /**************************
        * ACTIVE_FRAME at DECODER *
        **************************/

        st->hFdCngCom->inactive_frame_counter = 0;
        move16();
        st->hFdCngCom->sid_frame_counter = 0;
        move16();

        /* set noise estimation inactive during concealment, no update with noise generated by concealment should be performed. */

        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF (
            (concealWholeFrame == 0) &&
            (*timeDomainInput < MAXVAL_WORD16) && (*timeDomainInput > MINVAL_WORD16) && (*(timeDomainInput+st->hFdCngCom->frameSize-1) < MAXVAL_WORD16) && (*(timeDomainInput+st->hFdCngCom->frameSize-1) > MINVAL_WORD16)
            && ( ((st->flag_dtx_mode == 0) && (stdec->VAD != 0)) == 0 )
            && ( ((stdec->cng_type_fx == LP_CNG) && (st->flag_dtx_mode != 0)) == 0)
            && ( is_music == 0 )
        )
        {
            /* Perform noise estimation at the decoder */
            perform_noise_estimation_dec (
                timeDomainInput,
                Q,
                st
            );


            /* Update the shaping parameters */
            scalebands (
                st->msNoiseEst,
                st->part_shaping,
                st->nFFTpart_shaping,
                st->midband_shaping,
                st->nFFTpart_shaping,
                nBins,
                st->bandNoiseShape,
                1
            );

            st->bandNoiseShape_exp = st->msNoiseEst_exp;
            move16();


            /* Update CNG levels */
            test();
            IF ( st->flag_dtx_mode != 0 && stdec->cng_type_fx == FD_CNG )
            {
                /* This needs to be done only once per inactive phase */
                bandcombinepow (
                    st->bandNoiseShape,
                    st->bandNoiseShape_exp,
                    nBins,
                    st->hFdCngCom->part,
                    st->hFdCngCom->nFFTpart,
                    st->hFdCngCom->psize_inv,
                    st->partNoiseShape,
                    &st->partNoiseShape_exp
                );


                j = 0;
                move16();
                s2 = -(WORD32_BITS-1);
                move16();
                FOR (k=0; k < st->hFdCngCom->nFFTpart; k++)
                {
                    assert(st->partNoiseShape[k]>=0);
                    assert(st->hFdCngCom->sidNoiseEst[k]>=0);

                    /* add DELTA as it is done in FLC version, in order to avoid num > denom */
                    facTab[k]   = 0;
                    move16();
                    IF ( st->partNoiseShape[k] != 0)
                    {
                        s1        = norm_l(st->hFdCngCom->sidNoiseEst[k]);
                        L_tmp     = L_shl(st->hFdCngCom->sidNoiseEst[k], s1);
                        L_tmp_exp = sub(st->hFdCngCom->sidNoiseEstExp, s1);
                        L_tmp     = BASOP_Util_Add_Mant32Exp(L_tmp, L_tmp_exp, DELTA_MANTISSA_W32, DELTA_EXPONENT, &L_tmp_exp);
                        L_tmp     = L_shr(L_tmp, 1);
                        s         = add(L_tmp_exp, 1);
                        num       = extract_h(L_tmp);

                        s1        = norm_l(st->partNoiseShape[k]);
                        L_tmp     = L_shl(st->partNoiseShape[k], s1);
                        L_tmp_exp = sub(st->partNoiseShape_exp, s1);
                        L_tmp     = BASOP_Util_Add_Mant32Exp(L_tmp, L_tmp_exp, DELTA_MANTISSA_W32, DELTA_EXPONENT, &L_tmp_exp);
                        s         = sub(s, L_tmp_exp);
                        denom     = extract_h(L_tmp);

                        facTab[k] = div_s(num,denom);
                        move16();
                        facTabExp[k] = s;
                        move16();
                    }
                    /* Set unique exponent, if mantissa is equal to zero */
                    if (facTab[k] == 0)
                    {
                        facTabExp[k] = -(WORD32_BITS-1);
                        move16();
                    }
                    s2 = s_max(s2,facTabExp[k]);
                }

                FOR (k=0; k < st->hFdCngCom->nFFTpart; k++)
                {
                    s = sub(facTabExp[k],s2);
                    s = s_max(s_min(s,WORD32_BITS-1),-(WORD32_BITS-1));
                    FOR ( ; j <= st->hFdCngCom->part[k]; j++)
                    {
                        cngNoiseLevel[j] = L_shl(Mpy_32_16_1(st->bandNoiseShape[j],facTab[k]),s);
                        move32();
                    }
                }

                /* adapt scaling for rest of the buffer */
                IF (sub(s2,-(WORD32_BITS-1)) != 0)
                {
                    s = sub(*cngNoiseLevel_exp,add(st->bandNoiseShape_exp,s2));
                    FOR ( ; k < st->hFdCngCom->npart; k++)
                    {
                        FOR( ; j <= st->hFdCngCom->part[k]; j++)
                        {
                            cngNoiseLevel[j] = L_shl(cngNoiseLevel[j],s);
                            move32();
                        }
                    }

                    *cngNoiseLevel_exp = add(st->bandNoiseShape_exp,s2);
                    move16();
                }
            }
            ELSE
            {
                /* This sets the new CNG levels until a SID update overwrites it */
                Copy32 (
                    st->bandNoiseShape,
                    cngNoiseLevel,
                    nBins
                );

                *cngNoiseLevel_exp = st->bandNoiseShape_exp;
                move16();
            }
            /*stdec->cngTDLevel = (float)sqrt( (sumFLOAT(cngNoiseLevel, st->hFdCngCom->stopFFTbin - st->hFdCngCom->startBand) / 2 * st->hFdCngCom->fftlen) / stdec->Mode2_L_frame);*/
            tmp_loop = sub(st->hFdCngCom->stopFFTbin , st->hFdCngCom->startBand);
            L_tmp = L_deposit_h(0);
            L_c = L_deposit_h(0);
            FOR(j = 0 ; j < tmp_loop; j++)
            {

                Carry = 0;
                L_tmp = L_add_c(L_tmp,*(cngNoiseLevel+j));
                Overflow = 0;

                if(*(cngNoiseLevel+j) < 0)
                {
                    L_c = L_msuNs(L_c,0,0);
                }
                if(*(cngNoiseLevel+j) >= 0)
                {
                    L_c = L_macNs(L_c,0,0);
                }
            }
            L_tmp = norm_llQ31(L_c,L_tmp,&L_tmp_exp);
            L_tmp_exp = sub(add(L_tmp_exp,*cngNoiseLevel_exp),1);

            L_tmp = Mpy_32_16_1(L_tmp,st->hFdCngCom->fftlen); /*Q16*/

            L_tmp = Mpy_32_16_1(L_tmp,T_DIV_L_Frame[L_shl(L_mac(-28000,stdec->L_frame_fx,95),1-15)]);/*Q16,exp -7*/
            L_tmp_exp = add(L_tmp_exp,-7); /*->Q16, L_tmp_exp */
            L_tmp_exp = add(L_tmp_exp,31-16); /*->Q31, L_tmp_exp*/

            stdec->cngTDLevel = round_fx(Sqrt32(L_tmp, &L_tmp_exp));
            stdec->cngTDLevel_e = L_tmp_exp;
            move16();


        }
        test();
        test();
        IF (sub(concealWholeFrame,1)==0 && sub(stdec->nbLostCmpt,1)==0 && (cngNoiseLevel[0]>0))
        {
            /* update isf cng estimate for concealment. Do that during concealment, in order to avoid addition clean channel complexity*/
            lpc_from_spectrum(cngNoiseLevel, *cngNoiseLevel_exp, st->hFdCngCom->startBand, st->hFdCngCom->stopFFTbin, st->hFdCngCom->fftlen, st->hFdCngCom->A_cng,
                              M, 0);
            E_LPC_a_lsp_conversion( st->hFdCngCom->A_cng, lsp_cng, stdec->lspold_cng, M );
            Copy( lsp_cng, stdec->lspold_cng, M);

            lsp2lsf_fx( lsp_cng, stdec->lsf_cng, M, stdec->sr_core );
            stdec->plcBackgroundNoiseUpdated = 1;
            move16();
        }
        BREAK;

    case SID_FRAME:

        st->flag_dtx_mode = 1;
        move16();
        /* no break */

    case ZERO_FRAME:

        test();
        IF(stdec!=NULL && stdec->cng_type_fx==LP_CNG)
        {
            /* Perform noise estimation on inactive phase at the decoder */
            perform_noise_estimation_dec(timeDomainInput, Q, st
                                        );

            /* Update the shaping parameters */
            scalebands(st->msNoiseEst, st->part_shaping, st->nFFTpart_shaping, st->midband_shaping, st->nFFTpart_shaping, st->hFdCngCom->stopFFTbin-st->hFdCngCom->startBand, st->bandNoiseShape, 1);
            st->bandNoiseShape_exp = st->msNoiseEst_exp;
            move16();
            /* This sets the new CNG levels until a SID update overwrites it */
            Copy32(st->bandNoiseShape, cngNoiseLevel, st->hFdCngCom->stopFFTbin-st->hFdCngCom->startBand); /* This sets the new CNG levels until a SID update overwrites it */
            *cngNoiseLevel_exp = st->bandNoiseShape_exp;
            move16();
            BREAK;
        }
        st->hFdCngCom->inactive_frame_counter = add(st->hFdCngCom->inactive_frame_counter,1);
        move16();

        /*************************************
        * SID_FRAME or ZERO_FRAME at DECODER *
        *************************************/

        /* Detect first non-active frame */
        IF ( sub(st->hFdCngCom->inactive_frame_counter,1) == 0 )
        {
            /* Compute the fine spectral structure of the comfort noise shape using the decoder-side noise estimates */
            bandcombinepow (
                st->bandNoiseShape,
                st->bandNoiseShape_exp,
                nBins,
                st->hFdCngCom->part,
                st->hFdCngCom->nFFTpart,
                st->hFdCngCom->psize_inv,
                st->partNoiseShape,
                &st->partNoiseShape_exp
            );
        }

        IF ( sub(m_frame_type,SID_FRAME) == 0 )
        {
            IF ( L_sub(st->hFdCngCom->msFrCnt_init_counter,L_deposit_l(st->hFdCngCom->msFrCnt_init_thresh)) < 0 )
            {
                /* At initialization, interpolate the bin/band-wise levels from the partition levels */
                scalebands (
                    st->hFdCngCom->sidNoiseEst,
                    st->hFdCngCom->part,
                    st->hFdCngCom->npart,
                    st->hFdCngCom->midband,
                    st->hFdCngCom->nFFTpart,
                    sub(st->hFdCngCom->stopBand,st->hFdCngCom->startBand),
                    cngNoiseLevel,
                    1
                );
                *cngNoiseLevel_exp = st->hFdCngCom->sidNoiseEstExp;
                move16();
            }
            ELSE
            {
                /* Interpolate the CLDFB band levels from the SID (partition) levels */
                IF ( sub( st->hFdCngCom->regularStopBand, st->hFdCngCom->numCoreBands ) > 0 )
                {
                    scalebands (
                        st->hFdCngCom->sidNoiseEst,
                        st->hFdCngCom->part,
                        st->hFdCngCom->npart,
                        st->hFdCngCom->midband,
                        st->hFdCngCom->nFFTpart,
                        sub(st->hFdCngCom->stopBand,st->hFdCngCom->startBand),
                        cngNoiseLevel,
                        0
                    );

                    *cngNoiseLevel_exp = st->hFdCngCom->sidNoiseEstExp;
                    move16();
                }

                s2 = -(WORD32_BITS-1);
                move16();
                /* Shape the SID noise levels in each FFT bin */
                j = 0;
                move16();
                FOR (k=0; k < st->hFdCngCom->nFFTpart ; k++)
                {
                    assert(st->partNoiseShape[k]>=0);

                    /* add DELTA as it is done in FLC version, in order to avoid num > denom */
                    facTab[k]    = 0;
                    move16();
                    IF ( st->partNoiseShape[k] != 0)
                    {
                        s1        = norm_l(st->hFdCngCom->sidNoiseEst[k]);
                        L_tmp     = L_shl(st->hFdCngCom->sidNoiseEst[k], s1);
                        L_tmp_exp = sub(st->hFdCngCom->sidNoiseEstExp, s1);
                        L_tmp     = BASOP_Util_Add_Mant32Exp(st->hFdCngCom->sidNoiseEst[k], st->hFdCngCom->sidNoiseEstExp, DELTA_MANTISSA_W32, DELTA_EXPONENT, &L_tmp_exp);
                        L_tmp     = L_shr(L_tmp, 1);
                        s         = add(L_tmp_exp, 1);
                        num       = extract_h(L_tmp);

                        s1        = norm_l(st->partNoiseShape[k]);
                        L_tmp     = L_shl(st->partNoiseShape[k], s1);
                        L_tmp_exp = sub(st->partNoiseShape_exp, s1);
                        L_tmp     = BASOP_Util_Add_Mant32Exp(L_tmp, L_tmp_exp, DELTA_MANTISSA_W32, DELTA_EXPONENT, &L_tmp_exp);
                        s         = sub(s, L_tmp_exp);
                        denom     = extract_h(L_tmp);

                        facTab[k] = div_s(num,denom);
                        move16();
                        facTabExp[k] = s;
                        move16();
                    }
                    /* Set unique exponent, if mantissa is equal to zero */
                    if (facTab[k] == 0)
                    {
                        facTabExp[k] = -(WORD32_BITS-1);
                        move16();
                    }
                    s2 = s_max(s2,facTabExp[k]);
                }

                FOR (k=0; k < st->hFdCngCom->nFFTpart; k++)
                {
                    s = sub(facTabExp[k],s2);
                    s = s_max(s_min(s,WORD32_BITS-1),-(WORD32_BITS-1));
                    FOR( ; j <= st->hFdCngCom->part[k]; j++)
                    {
                        cngNoiseLevel[j] = L_shl(Mpy_32_16_1(st->bandNoiseShape[j],facTab[k]),s);
                        move32();
                    }
                }
                /* adapt scaling for rest of the buffer */
                s = sub(*cngNoiseLevel_exp,add(st->bandNoiseShape_exp,s2));
                FOR ( ; k < st->hFdCngCom->npart; k++)
                {
                    FOR( ; j <= st->hFdCngCom->part[k]; j++)
                    {
                        cngNoiseLevel[j] = L_shl(cngNoiseLevel[j],s);
                        move32();
                    }
                }
                *cngNoiseLevel_exp = add(st->bandNoiseShape_exp,s2);
                move16();
            }
        }


        IF ( sub(stdec->codec_mode, MODE2) == 0 )
        {
            /* Generate comfort noise during SID or zero frames */
            generate_comfort_noise_dec (
                cldfbBufferReal,
                cldfbBufferImag,
                cldfbBufferScale,
                stdec,
                &(stdec->Q_exc),
                2
            );
        }

        BREAK;

    default:
        return -1;
    }


    return 0;
}


/*
   perform_noise_estimation_dec

    Parameters:

    timeDomainInput,      i:   pointer to time domain input
    bitrate,              i:   bitrate
    st                    i/o: FD_CNG structure containing all buffers and variables

    Function:
    perform noise estimation

    Returns:
    void
*/
void perform_noise_estimation_dec (const Word16 *timeDomainInput, /* i:   pointer to time domain input */
                                   Word16 Q,
                                   HANDLE_FD_CNG_DEC st          /* i/o: FD_CNG structure containing all buffers and variables */
                                  )
{
    Word16  i, tmp_r, tmp_i, fac, fftBuffer_exp;
    Word16  s, len, npart, nFFTpart;
    Word16  startBand, stopFFTbin;

    Word16 *part, *psize_inv, *psize_norm;
    Word32 tmp, *fftBuffer, *periodog, *ptr_per, *ptr_r, *ptr_i;



    /* pointer initialization */
    periodog  = st->hFdCngCom->periodog;
    fftBuffer = st->hFdCngCom->fftBuffer;

    part = st->part_shaping;
    psize_inv = st->psize_inv_shaping;
    psize_norm = st->psize_shaping_norm;

    /* variable initialization */
    startBand  = st->hFdCngCom->startBand;
    move16();
    stopFFTbin = st->hFdCngCom->stopFFTbin;
    move16();

    npart = st->npart_shaping;
    move16();
    nFFTpart = st->nFFTpart_shaping;
    move16();

    /* Perform STFT analysis */
    AnalysisSTFT (
        timeDomainInput,
        Q,
        fftBuffer,
        &fftBuffer_exp,
        st->hFdCngCom
    );

    fftBuffer_exp = add(fftBuffer_exp,WORD16_BITS-1);


    assert(startBand != 0);

    len = sub(stopFFTbin, startBand);

    s = getScaleFactor32 (&fftBuffer[2*startBand], 2*len);
    s = sub(s,1);

    ptr_per = periodog;
    IF ( startBand == 0 )
    {
        /* DC component */
        tmp_r = extract_h(L_shl(fftBuffer[0],s));

        tmp = L_mult(tmp_r, tmp_r);
        *ptr_per = tmp;

        ptr_per++;
        ptr_r = fftBuffer + 2;
        len = sub(len, 1);
    }
    ELSE
    {
        ptr_r = fftBuffer + shl(startBand, 1);
    }

    ptr_i = ptr_r+1;
    FOR (i=0; i < len; i++)
    {
        tmp_r = extract_h(L_shl(*ptr_r,s));
        tmp_i = extract_h(L_shl(*ptr_i,s));

        tmp = L_mac(L_mult(tmp_r, tmp_r),tmp_i, tmp_i);
        *ptr_per = tmp;

        ptr_r += 2;
        ptr_i += 2;
        ptr_per++;
    }

    st->hFdCngCom->periodog_exp = shl(sub(fftBuffer_exp,s),1);

    /* Rescale */
    assert( (st->hFdCngCom->fftlen == 640) || (st->hFdCngCom->fftlen == 512) || (st->hFdCngCom->fftlen == 320) );

    fac = FL2WORD16(0.64);
    move16();
    if( sub(st->hFdCngCom->fftlen,512) == 0 )
    {
        fac = FL2WORD16(0.5);
        move16();
    }

    if ( sub(st->hFdCngCom->fftlen,640) == 0 )
    {
        s = 18;
        move16();
    }
    if ( sub(st->hFdCngCom->fftlen,512) == 0 )
    {
        s = 17;
        move16();
    }
    if ( sub(st->hFdCngCom->fftlen,320) == 0 )
    {
        s = 16;
        move16();
    }

    len = sub(stopFFTbin, startBand);
    FOR (i=0; i < len; i++)
    {
        st->hFdCngCom->periodog[i] = Mpy_32_16_1(st->hFdCngCom->periodog[i],fac);
    }
    st->hFdCngCom->periodog_exp = add(st->hFdCngCom->periodog_exp,sub(2,s));


    /* Adjust to the desired frequency resolution by averaging over spectral partitions for SID transmission */
    bandcombinepow(periodog, st->hFdCngCom->periodog_exp, sub(stopFFTbin,startBand), part, npart, psize_inv, st->msPeriodog, &st->msPeriodog_exp);


    st->msPeriodog_exp_fft = st->msPeriodog_exp;
    move16();
    st->msPeriodog_exp_cldfb = st->msPeriodog_exp;
    move16();


    /* Compress MS inputs */
    compress_range(st->msPeriodog, st->msPeriodog_exp, st->msLogPeriodog, npart);

    /* Call the minimum statistics routine for noise estimation */
    minimum_statistics (
        npart,
        nFFTpart,
        psize_norm,
        st->msLogPeriodog,
        st->msNoiseFloor,
        st->msLogNoiseEst,
        st->msAlpha,
        st->msPsd,
        st->msPsdFirstMoment,
        st->msPsdSecondMoment,
        st->msMinBuf,
        st->msBminWin,
        st->msBminSubWin,
        st->msCurrentMin,
        st->msCurrentMinOut,
        st->msCurrentMinSubWindow,
        st->msLocalMinFlag,
        st->msNewMinFlag,
        st->msPeriodogBuf,
        &(st->msPeriodogBufPtr),
        st->hFdCngCom
    );

    /* Expand MS outputs */
    expand_range(st->msLogNoiseEst, st->msNoiseEst, &st->msNoiseEst_exp, npart);

}



/*
   FdCng_decodeSID

    Parameters:

    st               i/o: FD_CNG structure containing all buffers and variables
    bs_word16        i  : Bitstream
    amrwb_io         i  : amr wideband mode
    preemph_fac      i  : preemphase factor

    Function:
    decode the FD-CNG bitstream

    Returns:
    void
*/
void FdCng_decodeSID (HANDLE_FD_CNG_COM st, Decoder_State_fx *corest)
{
    Word16  i, N, index;
    Word32 *sidNoiseEst;

    Word16 indices[32], v16[32];
    Word32 v[32], gain;

    Word32 tmp, maxVal, E_ExpLd64;
    Word16 sidNoiseEst_Exp;

    Word16 preemph_fac;


    sidNoiseEst = st->sidNoiseEst;
    move16();
    preemph_fac = corest->preemph_fac;
    move16();

    N = st->npart;
    move16();

    st->sid_frame_counter = add(st->sid_frame_counter,1);
    move16();

    /* Read bitstream */
    FOR (i=0; i<stages_37bits; i++)
    {
        indices[i] = get_next_indice_fx(corest, bits_37bits[i]);
        move16();
    }
    index = get_next_indice_fx(corest, 7);

    /* MSVQ decoder */
    msvq_decoder (
        cdk_37bits,
        stages_37bits,
        N,
        maxN_37bits,
        indices,
        v16
    );

    FOR (i=0; i<N; i++)
    {
        v[i] = L_deposit_h(v16[i]);
    }

    /* decode gain, format gain: Q9.23 */
    gain = L_shl(L_deposit_l(index),WORD32_BITS-1-8);
    gain = L_sub(gain,FL2WORD32_SCALE(60.0,8));
    gain = Mpy_32_16_1(gain,FL2WORD16(2.0f/3.0f));


    /* Apply gain and undo log */

    /* calculate worst case for scaling */
    maxVal = L_add(FL2WORD32(-1.0), 0);
    FOR (i=0; i<N; i++)
    {
        maxVal = L_max(maxVal,v[i]);
    }

    maxVal = L_add(maxVal, gain);
    maxVal = L_shl(Mpy_32_16_1(maxVal, FL2WORD16(0.66438561897)), 1);

    sidNoiseEst_Exp = 0;
    move16();
    FOR ( ; maxVal >= 0; maxVal -= FL2WORD32(0.015625))
    {
        sidNoiseEst_Exp = add(sidNoiseEst_Exp,1);
    }
    st->sidNoiseEstExp = sidNoiseEst_Exp;
    move16();
    E_ExpLd64 = L_shl(sidNoiseEst_Exp, WORD32_BITS-1-LD_DATA_SCALE);

    /* format v: Q9.23, format sidNoiseEst: Q6.26, 0.66438561897 = log10(10)/log10(2.0) / 10.0 * 2.0 */
    FOR (i=0; i<N; i++)
    {
        tmp = L_add(v[i], gain);
        tmp = L_shl(Mpy_32_16_1(tmp, FL2WORD16(0.66438561897)), 1);
        tmp = L_sub(tmp, E_ExpLd64);
        assert(tmp < 0);
        st->sidNoiseEst[i] = BASOP_Util_InvLog2(tmp);
        move32();
    }

    /* NB last band energy compensation */
    IF ( st->CngBandwidth == NB )
    {
        st->sidNoiseEst[N-1] = Mpy_32_16_1(st->sidNoiseEst[N-1], NB_LAST_BAND_SCALE);
        move32();
    }

    test();
    if ( st->CngBandwidth == SWB && st->CngBitrate <= ACELP_13k20 )
    {
        st->sidNoiseEst[N-1] = Mpy_32_16_1(st->sidNoiseEst[N-1], SWB_13k2_LAST_BAND_SCALE);
    }


    scalebands (sidNoiseEst, st->part, st->npart, st->midband, st->nFFTpart, st->stopBand-st->startBand, st->cngNoiseLevel, 1);
    st->cngNoiseLevelExp = st->sidNoiseEstExp;
    move16();


    lpc_from_spectrum (st->cngNoiseLevel, st->cngNoiseLevelExp, st->startBand, st->stopFFTbin, st->fftlen, st->A_cng, M, preemph_fac);


}


/*
    noisy_speech_detection

    Parameters:

    vad                 i  : VAD decision
    Etot                i  : total channel E
    Etot_exp            i  : exponent for total channel E
    totalNoise          i  : noise estimate over all critical bands
    totalNoise_exp      i  : exponent for noise estimate over all critical bands
    lp_noise            i/o: pointer to long term total Noise energy average
    lp_speech           i/o: pointer to long term active speech energy average

    Function:
    detector for noisy speech, lp_noise and lp_speech are scaled by LD_DATA_SCALE+2 bits

    Returns: flag, that indicates whether noisy speech has been detected

    void
*/
void noisy_speech_detection (const Word16 vad,
                             const Word16 *ftimeInPtr,    /* i  : input time-domain frame                  */
                             const Word16 frameSize,      /* i  : frame size                               */
                             const Word16 Q,
                             const Word32 *msNoiseEst,    /* i  : noise estimate over all critical bands */
                             const Word16 msNoiseEst_exp, /* i  : exponent for noise estimate over all critical bands */
                             const Word16 *psize_norm,
                             const Word16 psize_norm_exp,
                             const Word16 nFFTpart,       /* i  : Number of partitions taken into account  */
                             Word32 *lp_noise,            /* i/o: pointer to long term total Noise energy average */
                             Word32 *lp_speech,           /* i/o: pointer to long term active speech energy average */
                             Word16 *flag_noisy_speech
                            )
{
    Word16 i;
    Word32 tmp;
    Word32 Etot;
    Word16 Etot_exp;
    Word32 logEtot;
    Word32 logEtotExp;
    Word32 totalNoise;
    Word16 totalNoise_exp;
    Word32 logTotalNoise;
    Word32 logTotalNoiseExp;


    IF ( vad == 0 )
    {
        totalNoise = dotWord32_16_Mant32Exp(msNoiseEst, msNoiseEst_exp, psize_norm, psize_norm_exp, nFFTpart, &totalNoise_exp);

        /*
          - logTotalNoise is scaled by LD_DATA_SCALE+2
          - logTotalNoise = 10.0 * log10(totalNoise + DELTA);
          - constant: -0.78125 = 10.0*log10(DELTA)/(1<<(LD_DATA_SCALE+2))
          - constant:  0.75257498916 = 10.0 * log10(2.0)/log10(10.0)/(1<<2)
        */
        IF ( totalNoise == 0 )
        {
            logTotalNoise = L_add(FL2WORD32(-0.78125), 0);
        }
        ELSE
        {
            logTotalNoise = BASOP_Util_Log2(totalNoise);
            logTotalNoiseExp = L_shl(L_deposit_l(totalNoise_exp),WORD32_BITS-1-LD_DATA_SCALE);
            logTotalNoise = Mpy_32_16_1(L_add(logTotalNoise,logTotalNoiseExp),FL2WORD16(0.75257498916));
        }

        *lp_noise = L_add(Mpy_32_16_1(*lp_noise,FL2WORD16(0.995)),L_shr(Mpy_32_16_1(logTotalNoise,FL2WORD16(0.64)),7));
        move32();
    }
    ELSE
    {
        Etot = 0;
        Etot_exp = 31;
        FOR( i = 0; i < frameSize ; i++ )
        {
            tmp = L_shr_r( L_mult0( ftimeInPtr[i], ftimeInPtr[i] ), sub( Etot_exp, 31 ) );
            IF( L_sub( maxWord32, tmp ) < Etot )
            {
                Etot_exp = add( Etot_exp, 1 );
                Etot = L_shr_r( Etot, 1 );
                tmp = L_shr_r( tmp, 1 );
            }
            Etot = L_add( Etot, tmp );
        }
        Etot_exp = sub( Etot_exp, shl( Q, 1 ) );

        /*
          - logEtot is scaled by LD_DATA_SCALE+2
          - logEtot = 10.0 * log10(totalNoise + DELTA);
          - constant: -0.78125 = 10.0*log10(DELTA)/(1<<(LD_DATA_SCALE+2))
          - constant:  0.75257498916 = 10.0 * log10(2.0)/log10(10.0)/(1<<2)
        */
        IF ( Etot == 0 )
        {
            logEtot = L_add(FL2WORD32(-0.78125), 0);
        }
        ELSE
        {
            logEtot = BASOP_Util_Log2(Etot);
            logEtotExp = L_shl(L_deposit_l(Etot_exp),WORD32_BITS-1-LD_DATA_SCALE);
            logEtot = Mpy_32_16_1(L_add(logEtot,logEtotExp),FL2WORD16(0.75257498916));
            IF ( sub( frameSize, L_FRAME16k ) == 0 )
            {
                logEtot = L_add( logEtot, FL2WORD32(-0.086098436822497) );
            }
            ELSE
            {
                logEtot = L_add( logEtot, FL2WORD32(-0.082312889439370) );
            }
        }

        *lp_speech = L_add(Mpy_32_16_1(*lp_speech,FL2WORD16(0.995)),L_shr(Mpy_32_16_1(logEtot,FL2WORD16(0.64)),7));
        move32();
    }

    tmp = L_sub(*lp_speech,FL2WORD32_SCALE(45.0,LD_DATA_SCALE+2));

    if ( L_sub(*lp_noise,tmp) < 0 )
    {
        *lp_noise = tmp;
        move32();
    }

    *flag_noisy_speech = 0;
    move16();
    if ( L_sub(L_sub(*lp_speech,*lp_noise),FL2WORD32_SCALE(28.0,(LD_DATA_SCALE+2))) < 0 )
    {
        *flag_noisy_speech = 1;
        move16();
    }


    return;
}


void
generate_comfort_noise_dec (Word32 **bufferReal,         /* o   : matrix to real part of input bands */
                            Word32 **bufferImag,         /* o   : matrix to imaginary part of input bands */
                            Word16  *bufferScale,        /* o   : pointer to scalefactor for real and imaginary part of input bands */
                            Decoder_State_fx *stdec,
                            Word16 *Q_new,
                            Word16 gen_exc
                           )
{
    Word16  i, j, s, sc, sn, cnt;
    Word16  startBand2;
    Word16  stopFFTbin2;
    Word16  scaleCLDFB;
    Word16  preemph_fac;
    Word32  sqrtNoiseLevel;
    Word16  randGaussExp;
    Word16  fftBufferExp;
    Word16  cngNoiseLevelExp;
    Word16 *seed;
    Word16 *timeDomainOutput;
    Word32 *ptr_r, *ptr_i;
    Word32 *cngNoiseLevel;
    Word32 *ptr_level;
    Word32 *fftBuffer;
    Word16  old_syn_pe_tmp[16];
    Word16 tcx_transition = 0;
    HANDLE_FD_CNG_DEC std = stdec->hFdCngDec_fx;
    HANDLE_FD_CNG_COM st = std->hFdCngCom;



    /* Warning fix */
    s = 0;

    /* pointer initialization */

    cngNoiseLevel = st->cngNoiseLevel;
    cngNoiseLevelExp = st->cngNoiseLevelExp;
    ptr_level = cngNoiseLevel;
    seed = &(st->seed);
    fftBuffer = st->fftBuffer;
    timeDomainOutput = st->timeDomainBuffer;

    /* scaleCLDFB: CLDFBinvScalingFactor_EXP + 1 */
    scaleCLDFB = mult(st->invScalingFactor,CLDFB_SCALING);

    /*
      Generate Gaussian random noise in real and imaginary parts of the FFT bins
      Amplitudes are adjusted to the estimated noise level cngNoiseLevel in each bin
      scaling Gaussian random noise: format Q3.29
    */
    sn = 0;
    move16();
    IF ( s_and(cngNoiseLevelExp,1) != 0 )
    {
        sn = add(sn,1);
        cngNoiseLevelExp = add(cngNoiseLevelExp,sn);
        move16();
    }

    randGaussExp = CNG_RAND_GAUSS_SHIFT;
    move16();
    cnt = sub(st->stopFFTbin, st->startBand);
    IF ( st->startBand == 0 )
    {
        /* DC component in FFT */
        s = 0;
        move16();
        sqrtNoiseLevel = Sqrt32(L_shr(*ptr_level,sn), &s);

        fftBuffer[0] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
        move32();

        /* Nyquist frequency is discarded */
        fftBuffer[1] = L_deposit_l(0);

        ptr_level = ptr_level + 1;
        ptr_r = fftBuffer + 2;
        cnt = sub(cnt, 1);
    }
    ELSE
    {
        startBand2 = shl(st->startBand,1);
        set32_fx(fftBuffer, 0, startBand2);
        ptr_r = fftBuffer + startBand2;
    }

    sn = add(sn,1);
    ptr_i = ptr_r + 1;
    FOR (i=0; i < cnt; i++)
    {
        s = 0;
        move16();
        sqrtNoiseLevel = Sqrt32(L_shr(*ptr_level,sn), &s);

        /* Real part in FFT bins */
        *ptr_r = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
        move32();

        /* Imaginary part in FFT bins */
        *ptr_i = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
        move32();

        ptr_r = ptr_r + 2;
        ptr_i = ptr_i + 2;
        ptr_level = ptr_level + 1;
    }

    /* Remaining FFT bins are set to zero */
    stopFFTbin2 = shl(st->stopFFTbin,1);
    set32_fx(fftBuffer+stopFFTbin2, 0, sub(st->fftlen,stopFFTbin2));

    fftBufferExp = add(shr(cngNoiseLevelExp,1),randGaussExp);

    /* If previous frame is active, reset the overlap-add buffer */
    IF ( st->frame_type_previous == ACTIVE_FRAME )
    {
        set16_fx(st->olapBufferSynth, 0, st->fftlen);
        test();
        test();
        if ( ( stdec->last_core_bfi > ACELP_CORE && stdec->codec_mode == MODE2 ) || stdec->codec_mode == MODE1 )
        {
            tcx_transition = 1;
            move16();
        }
    }

    /* Perform STFT synthesis */
    SynthesisSTFT (fftBuffer, fftBufferExp, timeDomainOutput, st->olapBufferSynth, st->olapWinSyn,
                   tcx_transition,
                   st, gen_exc, Q_new);

    {
        Word32 Lener, att;
        Word16 exp;
        /* update CNG excitation energy for LP_CNG */

        /* calculate the residual signal energy */
        /*enr = dotp( st->exc_cng, st->exc_cng, st->frameSize ) / st->frameSize;*/
        Lener = Dot_productSq16HQ(1,st->exc_cng,stdec->L_frame_fx,&exp);
        exp = add(sub(shl(sub(15,*Q_new),1),8),exp);     /*8 = log2(256)*/

        /* convert log2 of residual signal energy */
        /*(float)log10( enr + 0.1f ) / (float)log10( 2.0f );*/
        Lener = BASOP_Util_Log2(Lener);
        Lener = L_add(Lener,L_shl(L_deposit_l(exp),WORD32_BITS-1-LD_DATA_SCALE)); /*Q25*/
        if(stdec->L_frame_fx == L_FRAME16k)
        {
            Lener = L_sub(Lener, FL2WORD32_SCALE(0.3219280949f, LD_DATA_SCALE)); /*log2(320) = 8.3219280949f*/
        }
        /* decrease the energy in case of WB input */
        IF( sub(stdec->bwidth_fx, NB) != 0 )
        {
            IF( sub(stdec->bwidth_fx,WB) == 0 )
            {
                IF( stdec->CNG_mode_fx >= 0 )
                {
                    /* Bitrate adapted attenuation */
                    att = L_shl(L_deposit_l(ENR_ATT_fx[stdec->CNG_mode_fx]),17);
                }
                ELSE
                {
                    /* Use least attenuation for higher bitrates */
                    att = L_shl(L_deposit_l(ENR_ATT_fx[4]),17);
                }
            }
            ELSE
            {
                att = 384<<17;
                move16();/*1.5 Q8<<17=Q25*/
            }
            Lener = L_sub(Lener, att );
        }
        /*stdec->lp_ener = 0.8f * stcod->lp_ener + 0.2f * pow( 2.0f, enr );*/
        Lener = BASOP_util_Pow2(Lener, 6, &exp);
        Lener = Mult_32_16(Lener, FL2WORD16(0.2f));
        exp = sub(25,exp);
        Lener = L_shr(Lener, exp);  /*Q6*/
        stdec->lp_ener_fx = L_add(Mult_32_16(stdec->lp_ener_fx, FL2WORD16(0.8f)), Lener);  /*Q6*/
    }

    /*
      Generate Gaussian random noise in real and imaginary parts of the CLDFB bands
      Amplitudes are adjusted to the estimated noise level cngNoiseLevel in each band
    */
    test();
    IF ( bufferReal!=NULL && (sub(st->numCoreBands,st->regularStopBand) < 0) )
    {

        sn = sub(sn,1);
        sc = add(shr(add(cngNoiseLevelExp,CLDFBinvScalingFactor_EXP+1-1),1),randGaussExp);
        move16();
        assert( ((cngNoiseLevelExp+CLDFBinvScalingFactor_EXP+1-1)&1) == 0);

        FOR (j=st->numCoreBands; j<st->regularStopBand; j++)
        {
            /* scaleCLDFB:  CLDFBinvScalingFactor_EXP + 1 */
            s = 0;
            move16();
            sqrtNoiseLevel = Sqrt32(L_shr(Mpy_32_16_1(*ptr_level,scaleCLDFB),sn), &s);

            FOR (i=0; i<st->numSlots; i++)
            {
                /* Real part in CLDFB band */
                bufferReal[i][j] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
                move32();
                /*fprintf(pFile,"%13.10f\n",WORD322FL_SCALE(bufferReal[i][j],sc));*/

                /* Imaginary part in CLDFB band */
                bufferImag[i][j] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
                move32();
                /*fprintf(pFile,"%13.10f\n",WORD322FL_SCALE(bufferImag[i][j],sc));*/
            }
            ptr_level = ptr_level + 1;
        }
        *bufferScale = sub(sc,15);
        move16();
    }

    /* Overlap-add when previous frame is active */
    test();
    IF ( st->frame_type_previous == ACTIVE_FRAME && stdec->codec_mode == MODE2 )
    {
        Word32 old_exc_ener, gain, noise32;
        Word16 seed, lpcorder, old_syn, tmp, gain16, N, N2, N4, N8;
        Word16 old_exc_ener_exp, gain_exp;
        Word16 normFacE, normShiftE, normShiftEM1;
        Word16 normFacG, normShiftG, normShiftGM1;
        Word16 noiseExp, *old_exc, *old_Aq, *old_syn_pe;
        Word16 noise[640], normShiftP2;
        Word16 Q_exc, Q_syn;


        assert(st->frameSize <= 640);

        seed = st->seed;
        move16();
        N = st->frameSize;
        move16();
        N2 = shr(st->frameSize,1);

        IF ( stdec->last_core_bfi > ACELP_CORE )
        {
            tcx_windowing_synthesis_current_frame(  timeDomainOutput,
                                                    stdec->tcx_cfg.tcx_mdct_window, /*Keep sine windows for limiting Time modulation*/
                                                    stdec->tcx_cfg.tcx_mdct_window_half,
                                                    stdec->tcx_cfg.tcx_mdct_window_minimum,
                                                    stdec->tcx_cfg.tcx_mdct_window_length,
                                                    stdec->tcx_cfg.tcx_mdct_window_half_length,
                                                    stdec->tcx_cfg.tcx_mdct_window_min_length,
                                                    0,
                                                    stdec->tcx_cfg.tcx_last_overlap_mode==ALDO_WINDOW?FULL_OVERLAP:stdec->tcx_cfg.tcx_last_overlap_mode,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    N/2,
                                                    stdec->tcx_cfg.tcx_offset<0?-stdec->tcx_cfg.tcx_offset:0,
                                                    0,
                                                    1,
                                                    0
                                                    ,0
                                                 );

            IF (stdec->tcx_cfg.last_aldo != 0)
            {
                FOR (i=0; i<st->frameSize; i++)
                {
                    timeDomainOutput[i] = add(timeDomainOutput[i], shr_r(stdec->old_out_LB_fx[i+NS2SA(stdec->sr_core, N_ZERO_MDCT_NS)],stdec->Q_old_wtda_LB));
                }
            }
            ELSE
            {
                tcx_windowing_synthesis_past_frame( stdec->old_syn_Overl,
                stdec->tcx_cfg.tcx_aldo_window_1_trunc,
                stdec->tcx_cfg.tcx_mdct_window_half,
                stdec->tcx_cfg.tcx_mdct_window_minimum,
                stdec->tcx_cfg.tcx_mdct_window_length,
                stdec->tcx_cfg.tcx_mdct_window_half_length,
                stdec->tcx_cfg.tcx_mdct_window_min_length,
                stdec->tcx_cfg.tcx_last_overlap_mode
                                                  );

                FOR (i=0; i<N2; i++)
                {
                    timeDomainOutput[i] += shl(stdec->old_syn_Overl[i],TCX_IMDCT_HEADROOM);
                }
            }
        }
        ELSE
        {

            /*
              - the scaling of the LPCs (e.g. old_Aq) is always Q12 (encoder or decoder)

              - the scaling of the deemphasized signals (e.g. old_syn) is always Q0 (encoder or decoder)

              - the scaling of the excitation signals in the encoder (e.g. old_exc) is Q_new
              - the scaling of the preemphasized signals in the encoder (e.g. old_syn_pe) is Q_new-1

              - the scaling of the excitation signals in the decoder (e.g. old_exc) is Q_exc (or stdec->Q_exc)
              - the scaling of the preemphasized signals in the decoder (e.g. old_syn_pe) is Q_syn (or stdec->Q_syn)
            */

            lpcorder = M;
            move16();
            old_Aq = stdec->old_Aq_12_8_fx;
            old_exc = stdec->old_exc_fx+sub(L_EXC_MEM_DEC,N2);
            old_syn_pe = stdec->mem_syn2_fx;
            old_syn = stdec->syn[lpcorder];
            move16();
            preemph_fac = stdec->preemph_fac;
            move16();
            Q_exc = stdec->Q_exc;
            move16();
            Q_syn = stdec->Q_syn;
            move16();

            /* shift to be in the range of values supported by getNormReciprocalWord16() */
            N8 = shr(N2, CNG_NORM_RECIPROCAL_RANGE_SHIFT);

            assert( N2 == (N8<<CNG_NORM_RECIPROCAL_RANGE_SHIFT) );

            normFacE = getNormReciprocalWord16(N8);
            normShiftE = BASOP_util_norm_s_bands2shift(N8);
            normShiftEM1 = sub(normShiftE, 1);
            normShiftP2 = add(normShiftE, CNG_NORM_RECIPROCAL_RANGE_SHIFT);

            old_exc_ener = L_shr(L_mult(old_exc[0],old_exc[0]),normShiftP2);
            FOR (i=1; i<N2; i++)
            {
                old_exc_ener = L_add(old_exc_ener, L_shr(L_mult(old_exc[i],old_exc[i]),normShiftP2));
            }
            old_exc_ener = L_shl(Mpy_32_16_1(old_exc_ener, shl(normFacE, normShiftEM1)),1);

            old_exc_ener_exp = 0;
            move16();
            old_exc_ener = Sqrt32(old_exc_ener, &old_exc_ener_exp);
            old_exc_ener_exp = add(old_exc_ener_exp,(sub(15,Q_exc)));

            /* shift to be in the range of values supported by getNormReciprocalWord16() */
            N4 = shr(N, CNG_NORM_RECIPROCAL_RANGE_SHIFT);

            assert( N == (N4<<CNG_NORM_RECIPROCAL_RANGE_SHIFT) );

            normFacG = getNormReciprocalWord16(N4);
            normShiftG = BASOP_util_norm_s_bands2shift(N4);
            normShiftGM1 = sub(normShiftG, 1);
            normShiftP2 = add(normShiftG, CNG_NORM_RECIPROCAL_RANGE_SHIFT);

            gain = L_deposit_l(0);
            FOR (i=0; i<N; i++)
            {
                noise32 = rand_gauss(&seed);
                noise[i] = extract_h(noise32);
                gain = L_add(gain, L_shr(L_mult(noise[i],noise[i]),normShiftP2));
            }
            gain = L_shl(Mpy_32_16_1(gain, shl(normFacG, normShiftGM1)),1);

            gain_exp = 2*CNG_RAND_GAUSS_SHIFT;
            move16();
            gain = ISqrt32(gain, &gain_exp);

            gain = Mpy_32_32(old_exc_ener, gain);
            gain16 = extract_h(gain);

            gain_exp = add(old_exc_ener_exp,gain_exp);
            noiseExp = add(CNG_RAND_GAUSS_SHIFT,gain_exp);

            s = sub(15-NOISE_HEADROOM,noiseExp);
            FOR (i=0; i<N; i++)
            {
                noise[i] = shr(mult(noise[i],gain16),s);
                move16();
            }

            assert(lpcorder <= 16);

            s = sub(15-NOISE_HEADROOM,(sub(15,Q_syn)));
            FOR (i=0; i<lpcorder; i++)
            {
                old_syn_pe_tmp[i] = shr(old_syn_pe[i],s);
                move16();
            }

            E_UTIL_synthesis (
                0,               /* i  : scaling to apply for a[0]                 Q0   */
                old_Aq,          /* i  : LP filter coefficients                    Q12  */
                noise,           /* i  : input signal                              Qx   */
                noise,           /* o  : output signal                             Qx-s */
                N,               /* i  : size of filtering                         Q0   */
                old_syn_pe_tmp,  /* i/o: memory associated with this filtering.    Q0 */
                0,               /* i  : 0=no update, 1=update of memory.          Q0   */
                lpcorder         /* i  : order of LP filter                        Q0   */
            );

            tmp = old_syn;
            move16();

            E_UTIL_deemph2 (
                NOISE_HEADROOM,
                noise,        /* I/O: signal			  Qx */
                preemph_fac,  /* I: deemphasis factor	  Qx */
                N,            /* I: vector size              */
                &tmp          /* I/O: memory (signal[-1]) Qx */
            );

            FOR (i=0; i<N4; i++)
            {
                tmp = mult(noise[i],st->olapWinSyn[i].v.re);
                timeDomainOutput[i] = add(timeDomainOutput[i],tmp);
                move16();
                tmp = mult(noise[i+N4],st->olapWinSyn[N4-1-i].v.im);
                timeDomainOutput[i+N4] = add(timeDomainOutput[i+N4],tmp);
                move16();
            }
        }
    }

}


void
generate_comfort_noise_dec_hf (Word32 **bufferReal,         /* o   : matrix to real part of input bands */
                               Word32 **bufferImag,         /* o   : matrix to imaginary part of input bands */
                               Word16  *bufferScale,        /* o   : pointer to scalefactor for real and imaginary part of input bands */
                               Decoder_State_fx *stdec
                              )
{
    Word16  i, j, s, sc, sn;
    Word16  scaleCLDFB;
    Word32  sqrtNoiseLevel;
    Word16  randGaussExp;
    Word16  cngNoiseLevelExp;
    Word16 *seed;
    Word32 *cngNoiseLevel;
    Word32 *ptr_level;
    HANDLE_FD_CNG_COM st = stdec->hFdCngDec_fx->hFdCngCom;

    cngNoiseLevel = st->cngNoiseLevel+st->stopFFTbin-st->startBand;
    cngNoiseLevelExp = st->cngNoiseLevelExp;
    ptr_level = cngNoiseLevel;
    seed = &(st->seed);

    /* scaleCLDFB: CLDFBinvScalingFactor_EXP + 1 */
    scaleCLDFB = mult(st->invScalingFactor,CLDFB_SCALING);

    sn = 0;
    move16();
    IF ( s_and(cngNoiseLevelExp,1) != 0 )
    {
        sn = add(sn,1);
        cngNoiseLevelExp = add(cngNoiseLevelExp,sn);
        move16();
    }

    randGaussExp = CNG_RAND_GAUSS_SHIFT;
    move16();

    IF ( sub(st->numCoreBands,st->regularStopBand) < 0 )
    {

        sc = add(shr(add(cngNoiseLevelExp,CLDFBinvScalingFactor_EXP+1-1),1),randGaussExp);
        move16();
        assert( ((cngNoiseLevelExp+CLDFBinvScalingFactor_EXP+1-1)&1) == 0);

        FOR (j=st->numCoreBands; j<st->regularStopBand; j++)
        {
            /* scaleCLDFB:  CLDFBinvScalingFactor_EXP + 1 */
            s = 0;
            move16();
            sqrtNoiseLevel = Sqrt32(L_shr(Mpy_32_16_1(*ptr_level,scaleCLDFB),sn), &s);

            FOR (i=0; i<st->numSlots; i++)
            {
                /* Real part in CLDFB band */
                bufferReal[i][j] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
                move32();
                /*fprintf(pFile,"%13.10f\n",WORD322FL_SCALE(bufferReal[i][j],sc));*/

                /* Imaginary part in CLDFB band */
                bufferImag[i][j] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),s);
                move32();
                /*fprintf(pFile,"%13.10f\n",WORD322FL_SCALE(bufferImag[i][j],sc));*/
            }
            ptr_level = ptr_level + 1;
        }
        *bufferScale = sub(sc,15);
        move16();
    }
}


/*
    generate_masking_noise

    Parameters:

    timeDomainBuffer       i/o : pointer to time domain output buffer 15Q0
    st                     i/o : pointer to FD_CNG_COM structure
    bitrate                i   : bitrate

    Function:
    Generate additional comfort noise (kind of noise filling)

    Returns: none

    void
*/
void generate_masking_noise (Word16 *timeDomainBuffer, /* i/o : pointer to time domain output buffer 15Q0 */
                             Word16 Q,
                             HANDLE_FD_CNG_COM st     /* i/o : pointer to FD_CNG_COM structure */
                             ,Word16 length
                             ,Word16 core
                            )
{
    Word16  i, s, s1, s2, sq, cnt, startBand2, stopFFTbin2;
    Word16  scaleExp,fftBufferExp,cngNoiseLevelExp;
    Word16  scale, scaleTableSize;
    Word16  maskingNoise[L_FRAME16k];
    Word32  sqrtNoiseLevel;
    Word32 *cngNoiseLevel;
    Word32 *fftBuffer;
    Word16 *seed;



    /* pointer initializations */
    cngNoiseLevel = st->cngNoiseLevel;
    fftBuffer = st->fftBuffer;
    seed = &(st->seed);

    /* Compute additional CN level */
    cngNoiseLevelExp = st->cngNoiseLevelExp;
    move16();

    IF(core!=AMR_WB_CORE)
    {
        scaleTableSize = 18;
        move16();
        assert( scaleTableSize == (sizeof (scaleTable_cn_only) / sizeof (scaleTable_cn_only[0])) );

        scale = -1;
        move16();
        FOR (i=0; i < scaleTableSize; i++)
        {
            test();
            test();
            IF (   ( sub(st->CngBandwidth,scaleTable_cn_only[i].bwmode) == 0 )
                   && ( L_sub(st->CngBitrate,scaleTable_cn_only[i].bitrateFrom) >= 0 )
                   && ( L_sub(st->CngBitrate,scaleTable_cn_only[i].bitrateTo) < 0 )
               )
            {
                scale = scaleTable_cn_only[i].scale;
                move16();
                BREAK;
            }
        }
        assert(scale >= 0);
    }
    ELSE
    {
        scaleTableSize = 3;
        move16();
        assert( scaleTableSize == (sizeof (scaleTable_cn_only_amrwbio) / sizeof (scaleTable_cn_only_amrwbio[0])) );

        scale = 0;
        move16();
        FOR (i=0; i < scaleTableSize; i++)
        {
            IF ( L_sub(st->CngBitrate,scaleTable_cn_only_amrwbio[i][0]) >= 0 )
            {
                scale = scaleTable_cn_only_amrwbio[i][1];
                move16();
                BREAK;
            }
        }
    }

    /* Exclude clean speech */

    s1 = norm_s(scale);
    s2 = norm_s(st->likelihood_noisy_speech);

    /* scaleTable_cn_only[i].scale is scaled by 1 bit */
    scaleExp = sub(1,add(s1,s2));
    scale = mult_r(shl(scale,s1),shl(st->likelihood_noisy_speech,s2));

    {
        /* add exponent of scale and cngNoiseLevel */
        fftBufferExp = add(scaleExp,cngNoiseLevelExp);

        /* even scalefactor needed for sqrt calculation */
        s = s_and(fftBufferExp,1);
        fftBufferExp = add(fftBufferExp,s);

        /* sqrt calculation => shift exponent */
        fftBufferExp = shr(fftBufferExp,1);

        /* consider scaling of random noise */
        fftBufferExp = add(fftBufferExp,CNG_RAND_GAUSS_SHIFT);

        cnt = sub(st->stopFFTbin,st->startBand);
        /*
          Generate Gaussian random noise in real and imaginary parts of the FFT bins
          Amplitudes are adjusted to the estimated noise level cngNoiseLevel in each bin
        */
        IF ( st->startBand == 0 )
        {
            /* random noise is scaled by CNG_RAND_GAUSS_SHIFT bits */

            /* DC component in FFT */

            /* -s => consider scalefactor adaptation for sqrt calculation */
            sq = sub(0,s);
            sqrtNoiseLevel = Sqrt32(Mpy_32_16_1(*cngNoiseLevel,scale),&sq);
            st->fftBuffer[0] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),sq);
            move32();
            st->fftBuffer[1] = 0;
            move32();

            fftBuffer = st->fftBuffer + 2;
            cngNoiseLevel++;

            cnt = sub(cnt,1);
        }
        ELSE
        {
            startBand2 = shl(st->startBand,1);
            set32_fx(st->fftBuffer,0,startBand2);
            fftBuffer = st->fftBuffer + startBand2;
        }

        FOR (i=0; i<cnt; i++)
        {
            /* -1 => weighting with 0.5, -s => consider scalefactor adaptation for sqrt calculation */
            sq = sub(-1,s);
            sqrtNoiseLevel = Sqrt32(Mpy_32_16_1(*cngNoiseLevel,scale),&sq);

            /* real part in FFT bins */

            /* random noise is scaled by CNG_RAND_GAUSS_SHIFT bits */
            *fftBuffer = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),sq);
            move32();
            fftBuffer++;

            /* imaginary part in FFT bins */

            /* random noise is scaled by CNG_RAND_GAUSS_SHIFT bits */
            *fftBuffer = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),sq);
            move32();
            fftBuffer++;

            cngNoiseLevel++;
        }

        /* remaining FFT bins are set to zero */
        stopFFTbin2 = shl(st->stopFFTbin,1);
        set32_fx(st->fftBuffer+stopFFTbin2, 0, sub(st->fftlen,stopFFTbin2));


        /* perform STFT synthesis */
        assert(st->olapBufferSynth2 != NULL);
        SynthesisSTFT(st->fftBuffer, fftBufferExp, maskingNoise, st->olapBufferSynth2, st->olapWinSyn,
                      0,
                      st, 0, NULL);


        /* add some comfort noise on top of decoded signal */
        IF ( st->frameSize > length )
        {
            FOR (i=0; i<length; i++)
            {
                timeDomainBuffer[i] = add(timeDomainBuffer[i],shr_r(maskingNoise[i],-Q));
                move16();
            }
        }
        ELSE
        {
            FOR (i=0; i<st->frameSize; i++)
            {
                timeDomainBuffer[i] = add(timeDomainBuffer[i],shr_r(maskingNoise[i],-Q));
                move16();
            }
        }
    }

}

/************************************************************
* Generate additional comfort noise (kind of noise filling) *
************************************************************/
void generate_masking_noise_mdct (Word32 *mdctBuffer,        /* i/o: time-domain signal */
                                  Word16 *mdctBuffer_e,      /* i/o: exponent time-domain signal */
                                  HANDLE_FD_CNG_COM st       /* i/o: FD_CNG structure containing all buffers and variables */
                                  ,Word16 L_frame
                                 )
{
    Word16  i, s, s1, s2, sq, cnt;
    Word16  scaleExp, maskingNoiseExp, cngNoiseLevelExp;
    Word16  scale, scaleTableSize;
    Word32  noise;
    Word32  sqrtNoiseLevel;
    Word32  maskingNoise[2*L_FRAME16k];
    Word32 *pMaskingNoise;
    Word32 *cngNoiseLevel;
    Word16 *seed;


    /* pointer initializations */
    cngNoiseLevel = st->cngNoiseLevel;
    seed = &(st->seed);

    /* Compute additional CN level */
    cngNoiseLevelExp = st->cngNoiseLevelExp;
    move16();

    /* Compute additional CN level */
    scaleTableSize = 18;
    move16();
    assert( scaleTableSize == (sizeof (scaleTable_cn_only) / sizeof (scaleTable_cn_only[0])) );

    scale = -1;
    move16();
    FOR (i=0; i < scaleTableSize; i++)
    {
        test();
        test();
        IF (   ( sub(st->CngBandwidth,scaleTable_cn_only[i].bwmode) == 0 )
               && ( L_sub(st->CngBitrate,scaleTable_cn_only[i].bitrateFrom) >= 0 )
               && ( L_sub(st->CngBitrate,scaleTable_cn_only[i].bitrateTo) < 0 )
           )
        {
            scale = scaleTable_cn_only[i].scale;
            move16();
            BREAK;
        }
    }
    assert(scale >= 0);

    /* Exclude clean speech */
    s1 = norm_s(scale);
    s2 = norm_s(st->likelihood_noisy_speech);

    /* scaleTable_cn_only[i].scale is scaled by 1 bit */
    scaleExp = sub(1,add(s1,s2));
    scale = mult_r(shl(scale,s1),shl(st->likelihood_noisy_speech,s2));

    /* add exponent of scale and cngNoiseLevel */
    maskingNoiseExp = add(scaleExp,cngNoiseLevelExp);

    /* even scalefactor needed for sqrt calculation */
    s = s_and(maskingNoiseExp,1);
    maskingNoiseExp = add(maskingNoiseExp,s);

    /* sqrt calculation => shift exponent */
    maskingNoiseExp = shr(maskingNoiseExp,1);

    /* consider scaling of random noise */
    maskingNoiseExp = add(maskingNoiseExp,CNG_RAND_GAUSS_SHIFT);

    cnt = sub(st->stopFFTbin,st->startBand);

    /*
      Generate Gaussian random noise in real and imaginary parts of the FFT bins
      Amplitudes are adjusted to the estimated noise level cngNoiseLevel in each bin
    */
    IF ( st->startBand == 0 )
    {
        /* random noise is scaled by CNG_RAND_GAUSS_SHIFT bits */

        /* DC component in FFT */

        /* -1 => weighting with 0.5, -s => consider scalefactor adaptation for sqrt calculation */
        sq = sub(-1,s);
        sqrtNoiseLevel = Sqrt32(Mpy_32_16_1(*cngNoiseLevel,scale),&sq);
        maskingNoise[0] = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),sq);
        move32();

        pMaskingNoise = &maskingNoise[1];
        cngNoiseLevel++;

        cnt = sub(cnt,1);
    }
    ELSE
    {
        set32_fx(maskingNoise,0,st->startBand);
        pMaskingNoise = maskingNoise + st->startBand;
    }

    FOR (i=0; i<cnt; i++)
    {
        /* -1 => weighting with 0.5, -s => consider scalefactor adaptation for sqrt calculation */
        sq = sub(-1,s);
        sqrtNoiseLevel = Sqrt32(Mpy_32_16_1(*cngNoiseLevel,scale),&sq);

        /* real part in FFT bins */

        /* random noise is scaled by CNG_RAND_GAUSS_SHIFT bits */
        *pMaskingNoise = L_shl(Mpy_32_32(rand_gauss(seed),sqrtNoiseLevel),sq);
        move32();
        pMaskingNoise++;

        cngNoiseLevel++;
    }

    /* re-normalization of energy level
       16 * 0.79056941504 = sqrt(NORM_MDCT_FACTOR)
    */
    assert( NORM_MDCT_FACTOR == 160 );

    /* do weighting with factor 0.79056941504 later */
    maskingNoiseExp = add(maskingNoiseExp,4);

    s = s_max(*mdctBuffer_e,maskingNoiseExp);
    s1 = sub(s,*mdctBuffer_e);
    s2 = sub(s,maskingNoiseExp);

    /* avoid rescaling of mdct samples if no comfort noise is added */
    IF ( scale != 0 )
    {
        /* Add some comfort noise on top of decoded signal */
        IF ( s1 == 0 )
        {
            FOR (i=0; i < st->stopFFTbin; i++)
            {
                /* If shifting negative noise values the lowest result is -1 but never 0.
                   Shift positive noise values to avoid unwanted amplification of these small values later */
                noise = L_shr(Mpy_32_16_1(L_abs(maskingNoise[i]),FL2WORD16(0.79056941504)),s2);

                if ( maskingNoise[i] < 0 )
                {
                    noise = L_negate(noise);
                }

                mdctBuffer[i] = L_add(mdctBuffer[i], noise);
                move32();
            }
        }
        ELSE
        {
            FOR (i=0; i < st->stopFFTbin; i++)
            {
                mdctBuffer[i] = L_add(L_shr(mdctBuffer[i],s1),
                Mpy_32_16_1(maskingNoise[i],FL2WORD16(0.79056941504)));
                move32();
            }
            FOR (i=st->stopFFTbin; i < L_frame; i++)
            {
                mdctBuffer[i] = L_shr(mdctBuffer[i],s1);
                move32();
            }
            *mdctBuffer_e = s;
            move16();
        }
    }

}
