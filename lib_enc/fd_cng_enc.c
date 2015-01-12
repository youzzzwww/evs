/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "stl.h"
#include "options.h"
#include "stl.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"


/********************************
* External tables               *
********************************/

extern Word16 preemphCompensation[NB_BANDS];


/********************************
* External functions            *
********************************/

#define swap(x,y,type) {type u__p; u__p=x; x=y; y=u__p;}


/*************************************
* Create an instance of type FD_CNG *
*************************************/
void createFdCngEnc(HANDLE_FD_CNG_ENC* hFdCngEnc)
{
    HANDLE_FD_CNG_ENC hs;

    /* Allocate memory */
    hs = (HANDLE_FD_CNG_ENC) calloc(1, sizeof (FD_CNG_ENC));
    move16();


    createFdCngCom(&(hs->hFdCngCom));
    *hFdCngEnc = hs;
    move16();

    return;
}

void initFdCngEnc(HANDLE_FD_CNG_ENC hsEnc, Word32 input_Fs, Word16 scale)
{
    Word16 j;
    HANDLE_FD_CNG_COM hsCom = hsEnc->hFdCngCom;

    /* Initialize common */

    initFdCngCom( hsCom, scale );

    /* Configure the Noise Estimator */

    hsCom->numSlots          = 16;
    move16();
    hsCom->numCoreBands      = 16;
    move16();
    hsCom->regularStopBand   = idiv1616U( extract_l( L_shr( input_Fs, 5 ) ), 25 );
    if ( sub( hsCom->regularStopBand, 40 ) > 0 )
    {
        hsCom->regularStopBand = 40;
        move16();
    }

    hsCom->startBand = 2;
    move16();
    IF ( sub( hsCom->regularStopBand, 10 ) == 0 )
    {
        hsCom->stopFFTbin = 160;
        move16();
        hsCom->stopBand = 160;
        move16();
        hsCom->nFFTpart = 17;
        move16();
    }
    ELSE
    {
        hsCom->stopFFTbin = 256;
        move16();
        hsCom->stopBand = add( sub( hsCom->regularStopBand, hsCom->numCoreBands ), hsCom->stopFFTbin );
        hsCom->nFFTpart = 20;
        move16();
    }

    initPartitions( sidparts_encoder_noise_est, 24, hsCom->startBand, hsCom->stopBand, hsCom->part, &hsCom->npart, hsCom->midband, hsCom->psize, hsCom->psize_norm, &hsCom->psize_norm_exp, hsCom->psize_inv, 0);

    hsCom->nCLDFBpart = sub( hsCom->npart, hsCom->nFFTpart );
    FOR(j=0; j<hsCom->nCLDFBpart; j++)
    {
        hsCom->CLDFBpart[j] = sub( hsCom->part[j+hsCom->nFFTpart], sub( 256, hsCom->startBand ) );
        hsCom->CLDFBpsize_inv[j] = hsCom->psize_inv[j+hsCom->nFFTpart];
    }

    /* Initialize noise estimation algorithm */
    set32_fx( hsEnc->msPeriodog, 0, NPART );
    hsEnc->msPeriodog_exp_fft = 0;
    move16();
    hsEnc->msPeriodog_exp_cldfb = 0;
    move16();
    set32_fx( hsEnc->msAlpha, 0, NPART );
    set32_fx( hsEnc->msBminWin, 0, NPART );
    set32_fx( hsEnc->msBminSubWin, 0, NPART );

    set32_fx( hsEnc->msNoiseEst, 0, NPART );
    hsEnc->msNoiseEst_exp = 0;
    move16();
    set32_fx( hsEnc->energy_ho, 0, NPART );
    set32_fx( hsEnc->msNoiseEst_old, 0, NPART );

    set16_fx( hsEnc->msLogPeriodog, 0, NPART );
    set16_fx( hsEnc->msLogNoiseEst, 0, NPART );
    set16_fx( hsEnc->msPsd, 0, NPART );
    set16_fx( hsEnc->msNoiseFloor, 0, NPART );
    set32_fx( hsEnc->msMinBuf, FL2WORD32(1.0), MSNUMSUBFR*NPART );
    set32_fx( hsEnc->msCurrentMin, FL2WORD32(1.0), NPART );
    set32_fx( hsEnc->msCurrentMinOut, FL2WORD32(1.0), NPART );
    set32_fx( hsEnc->msCurrentMinSubWindow, FL2WORD32(1.0), NPART );
    set16_fx( hsEnc->msPsdFirstMoment, 0, NPART );
    set16_fx( hsEnc->msPeriodogBuf, 0, MSBUFLEN*NPART );

    set16_fx( hsEnc->msLocalMinFlag, 0, NPART );
    set16_fx( hsEnc->msNewMinFlag, 0, NPART );
    hsEnc->msPeriodogBufPtr = 0;
    move16();
    set32_fx( hsEnc->msPsdSecondMoment, 0, NPART );

    return;
}

/************************************
* Configure FD_CNG                 *
************************************/
void configureFdCngEnc(HANDLE_FD_CNG_ENC hsEnc,   /* i/o: Contains the variables related to the FD-based CNG process */
                       Word16 bandwidth,        /* i:   bandwidth */
                       Word32 bitrate
                      )
{
    HANDLE_FD_CNG_COM hsCom = hsEnc->hFdCngCom;

    hsCom->CngBandwidth = bandwidth;
    move16();
    IF ( sub( hsCom->CngBandwidth, FB ) == 0 )
    {
        hsCom->CngBandwidth = SWB;
    }
    hsCom->CngBitrate = bitrate;
    move32();

    /* NB configuration */
    IF ( sub(bandwidth,NB) == 0 )
    {
        hsCom->FdCngSetup = FdCngSetup_nb;    /* PTR assignation -> no move needed*/
    }

    /* WB configuration */
    ELSE IF ( sub(bandwidth,WB) == 0 )
    {
        /* FFT 6.4kHz, no CLDFB */
        IF ( L_sub(bitrate,ACELP_8k00) <= 0 )
        {
            hsCom->FdCngSetup = FdCngSetup_wb1;
        }
        /* FFT 6.4kHz, CLDFB 8.0kHz */
        ELSE IF ( L_sub(bitrate,ACELP_13k20) <= 0 )
        {
            hsCom->FdCngSetup = FdCngSetup_wb2;
        }
        /* FFT 8.0kHz, no CLDFB */
        ELSE
        {
            hsCom->FdCngSetup = FdCngSetup_wb3;
        }
    }

    /* SWB/FB configuration */
    ELSE
    {
        /* FFT 6.4kHz, CLDFB 14kHz */
        IF ( L_sub(bitrate,ACELP_13k20) <= 0 )
        {
            hsCom->FdCngSetup = FdCngSetup_swb1;
        }
        /* FFT 8.0kHz, CLDFB 16kHz */
        ELSE
        {
            hsCom->FdCngSetup = FdCngSetup_swb2;
        }
    }
    hsCom->fftlen = hsCom->FdCngSetup.fftlen;
    move16();
    hsEnc->stopFFTbinDec = hsCom->FdCngSetup.stopFFTbin;
    move16();

    /* Configure the SID quantizer and the Confort Noise Generator */

    hsEnc->startBandDec = hsCom->startBand;
    move16();
    hsEnc->stopBandDec = add( hsCom->FdCngSetup.sidPartitions[hsCom->FdCngSetup.numPartitions-1], 1 );
    move16();
    initPartitions( hsCom->FdCngSetup.sidPartitions,
                    hsCom->FdCngSetup.numPartitions,
                    hsEnc->startBandDec,
                    hsEnc->stopBandDec,
                    hsEnc->partDec,
                    &hsEnc->npartDec,
                    hsEnc->midbandDec,
                    hsEnc->psizeDec,
                    hsEnc->psizeDec_norm,
                    &hsEnc->psizeDec_norm_exp,
                    hsEnc->psize_invDec,
                    0
                  );
    IF ( sub(hsEnc->stopFFTbinDec,160) == 0 )
    {
        hsEnc->nFFTpartDec = 17;
        move16();
    }
    ELSE IF ( sub(hsEnc->stopFFTbinDec,256) == 0 )
    {
        hsEnc->nFFTpartDec = 20;
        move16();
    }
    ELSE
    {
        hsEnc->nFFTpartDec = 21;
        move16();
    }

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
    hsCom->frameSize = shr( hsCom->fftlen, 1 );

}

/**************************************
* Delete the instance of type FD_CNG *
**************************************/
void deleteFdCngEnc(HANDLE_FD_CNG_ENC * hFdCngEnc)
{

    HANDLE_FD_CNG_ENC hsEnc;
    hsEnc = *hFdCngEnc;
    move16();
    IF (hsEnc != NULL)
    {
        deleteFdCngCom(&(hsEnc->hFdCngCom));
        free(hsEnc);
        *hFdCngEnc = NULL;
        move16();
    }
}


void resetFdCngEnc(
    Encoder_State_fx * st
)
{
    Word16 tmpTest;
    Word16 n;
    Word16 totalNoiseIncrease;
    Word16 thresh = 5 * 256;                    /*  5.0 in Q8 */

    /* st->totalNoise_fx;   Q8 Noise estimator - total noise energy */

    /* Detect fast increase of totalNoise */
    totalNoiseIncrease = sub(st->totalNoise_fx, st->last_totalNoise_fx);
    st->last_totalNoise_fx = st->totalNoise_fx;
    move16();
    IF ( totalNoiseIncrease > 0 )
    {
        IF ( sub(st->totalNoise_increase_len_fx,TOTALNOISE_HIST_SIZE)  == 0 )
        {
            FOR ( n = 0; n < TOTALNOISE_HIST_SIZE-1; n++ )
            {
                st->totalNoise_increase_hist_fx[n] = st->totalNoise_increase_hist_fx[n+1];
                move16();
            }
            st->totalNoise_increase_hist_fx[TOTALNOISE_HIST_SIZE-1] = totalNoiseIncrease;
            move16();
        }
        ELSE
        {
            st->totalNoise_increase_hist_fx[st->totalNoise_increase_len_fx] = totalNoiseIncrease;
            move16();
            st->totalNoise_increase_len_fx = add(st->totalNoise_increase_len_fx, 1);
        }
    }
    ELSE
    {
        st->totalNoise_increase_len_fx = 0;
        move16();
    }
    totalNoiseIncrease = 0;
    move16();
    FOR ( n = 0; n < st->totalNoise_increase_len_fx; n++ )
    {
        totalNoiseIncrease = add(totalNoiseIncrease, st->totalNoise_increase_hist_fx[n]);
    }

    test();
    test();
    tmpTest = ((sub (totalNoiseIncrease,thresh) > 0) && (sub(st->totalNoise_increase_len_fx,TOTALNOISE_HIST_SIZE)==0) && (sub(st->ini_frame_fx,150)>0) );

    test();
    IF ( tmpTest || ( sub (st->input_bwidth_fx,st->last_input_bwidth_fx) > 0 ) || sub(st->last_core_fx,AMR_WB_CORE) == 0 )
    {
        st->fd_cng_reset_flag = 1;
        move16();
        st->hFdCngEnc_fx->hFdCngCom->msFrCnt_init_counter = 0;
        move16();
        st->hFdCngEnc_fx->hFdCngCom->init_old = 32767;
        move16();
    }
    ELSE IF ( s_and((st->fd_cng_reset_flag > 0),(sub(st->fd_cng_reset_flag,10) < 0)) )
    {
        st->fd_cng_reset_flag = add(st->fd_cng_reset_flag,1);
    }
    ELSE
    {
        st->fd_cng_reset_flag = 0;
        move16();
    }
}

/*
   perform_noise_estimation_enc

    Parameters:

    band_energies        i:   energy in critical bands without minimum noise floor MODE2_E_MIN
    band_energies_exp    i:   exponent for energy in critical bands without minimum noise floor MODE2_E_MIN
    cldfbBufferReal,     i:   real part of the CLDFB buffer
    cldfbBufferImag,     i:   imaginary part of the CLDFB buffer
    cldfbBufferExp,      i:   exponent for CLDFB buffer
    bitrate              i:   bitrate
    st                   i/o: FD_CNG structure containing all buffers and variables

    Function:
    Perform noise estimation

    Returns:
    void
*/
void perform_noise_estimation_enc (Word32  *band_energies,       /* i: energy in critical bands without minimum noise floor MODE2_E_MIN */
                                   Word16   band_energies_exp,   /* i: exponent for energy in critical bands without minimum noise floor MODE2_E_MIN */
                                   Word32   *enerBuffer,
                                   Word16   enerBuffer_exp,
                                   HANDLE_FD_CNG_ENC st         /* i/o: FD_CNG structure containing all buffers and variables */
                                  )
{
    Word16  i, j, s, s1, s2;
    Word16  nFFTpart;
    Word16  nCLDFBpart;
    Word16  numBands;
    Word16  numCoreBands;
    Word16  regularStopBand;
    Word16  numSlots;
    Word32  tmp;
    Word32 *periodog;
    Word32 *ptr_per;
    Word32 *msPeriodog;



    nFFTpart = st->hFdCngCom->nFFTpart;
    move16();
    nCLDFBpart = st->hFdCngCom->nCLDFBpart;
    move16();
    numCoreBands = st->hFdCngCom->numCoreBands;
    move16();
    regularStopBand = st->hFdCngCom->regularStopBand;
    move16();
    numSlots = st->hFdCngCom->numSlots;
    move16();
    periodog = st->hFdCngCom->periodog;
    move16();
    ptr_per = periodog;
    move16();
    msPeriodog = st->msPeriodog;
    move16();

    assert(numSlots == 16);

    /* preemphasis compensation and grouping of per bin energies into msPeriodog */
    FOR (i=0; i < nFFTpart; i++)
    {
        tmp = L_add(L_shr(band_energies[i], 1), L_shr(band_energies[i+NB_BANDS], 1));
        msPeriodog[i] = Mpy_32_16_1(tmp, preemphCompensation[i]);
        move32();
    }

    /* exponent for fft part of msPeriodog */
    st->msPeriodog_exp_fft = add(band_energies_exp, PREEMPH_COMPENSATION_EXP);
    move16();

    numBands = sub(regularStopBand, numCoreBands);

    IF ( numBands > 0 )
    {
        /* Adjust to the desired time resolution by averaging the periodograms over the CLDFB time slots */

        FOR (j=numCoreBands; j < regularStopBand; j++)
        {
            *ptr_per = Mpy_32_16_1(enerBuffer[j], st->hFdCngCom->scalingFactor);
            move32();

            ptr_per++;
        }

        /* exponent for cldfb part of msPeriodog */
        st->hFdCngCom->exp_cldfb_periodog = add( sub(enerBuffer_exp, 4), CLDFBscalingFactor_EXP );

        /* Adjust CLDFB filterbank to the desired frequency resolution by averaging over spectral partitions for SID transmission */
        bandcombinepow (
            periodog,
            st->hFdCngCom->exp_cldfb_periodog,
            numBands,
            st->hFdCngCom->CLDFBpart,
            st->hFdCngCom->nCLDFBpart,
            st->hFdCngCom->CLDFBpsize_inv,
            &msPeriodog[nFFTpart],
            &st->msPeriodog_exp_cldfb
        );

        /* find common exponent for fft part and cldfb part of msperiodog */
        s1 = getScaleFactor32 (msPeriodog,nFFTpart);
        s2 = getScaleFactor32 (&msPeriodog[nFFTpart],nCLDFBpart);

        s  = s_max(sub(st->msPeriodog_exp_fft,s1), sub(st->msPeriodog_exp_cldfb,s2));
        s1 = sub(s,st->msPeriodog_exp_fft);
        s2 = sub(s,st->msPeriodog_exp_cldfb);

        st->msPeriodog_exp_fft = s;
        move16();
        st->msPeriodog_exp_cldfb = s;
        move16();

        FOR (i=0; i < nFFTpart; i++)
        {
            msPeriodog[i] = L_shr(msPeriodog[i],s1);
            move32();
        }

        FOR (i=0; i < nCLDFBpart; i++)
        {
            msPeriodog[nFFTpart+i] = L_shr(msPeriodog[nFFTpart+i],s_min(31,s2));
            move32();
        }
    }

    /* exponent for entire msPeriodog vector */
    st->msPeriodog_exp = st->msPeriodog_exp_fft;
    move16();

    /* Compress MS inputs */
    compress_range(st->msPeriodog, st->msPeriodog_exp, st->msLogPeriodog, st->hFdCngCom->npart);

    /* Call the minimum statistics routine for noise estimation */
    minimum_statistics (
        st->hFdCngCom->npart,
        st->hFdCngCom->nFFTpart,
        st->hFdCngCom->psize_norm,
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
    expand_range(st->msLogNoiseEst, st->msNoiseEst, &st->msNoiseEst_exp, st->hFdCngCom->npart);


}

/*
   AdjustFirstSID

    Parameters:

    npart                   i    : number of parts
    msPeriodog              i    : pointer to periodog vector
    msPeriodog_exp          i    : exponent of periodog vector
    energy_ho               i/o  : pointer to energy
    energy_ho_exp           i/o  : pointer to exponent of energy
    msNoiseEst              i/o  : pointer to estimated noise
    msNoiseEst_exp          i/o  : pointer to exponent of estimated noise
    msNoiseEst_old          i/o  : pointer to old estimated noise
    msNoiseEst_old_exp      i/o  : pointer to exponent of old estimated noise
    active_frame_counter    i/o  : pointer to active frame counter
    stcod                   i    : pointer to Coder_State structure

    Function:
    Adjust the noise estimator at the beginning of each CNG phase (encoder-side)

    Returns:
    void
*/
Word16
AdjustFirstSID (Word16  npart,                   /* i    : number of parts */
                Word32 *msPeriodog,              /* i    : pointer to periodog vector */
                Word16  msPeriodog_exp,          /* i    : exponent of periodog vector */
                Word32 *energy_ho,               /* i/o  : pointer to energy */
                Word16 *energy_ho_exp,           /* i/o  : pointer to exponent of energy */
                Word32 *msNoiseEst,              /* i/o  : pointer to estimated noise */
                Word16 *msNoiseEst_exp,          /* i/o  : pointer to exponent of estimated noise */
                Word32 *msNoiseEst_old,          /* i/o  : pointer to old estimated noise */
                Word16 *msNoiseEst_old_exp,      /* i/o  : pointer to exponent of old estimated noise */
                Word16 *active_frame_counter,    /* i/o  : pointer to active frame counter */
                Encoder_State_fx *stcod          /* i    : pointer to Coder_State_Plus structure */
               )
{
    Word16 i, sc, s1, s2, lambda, lambdaM1, invFac;
    Word32 tmp32, energy_ho_local, msNoiseEst_local;


    test();
    IF ( sub(stcod->cnt_SID_fx,1) == 0 && L_sub(stcod->last_core_brate_fx,SID_2k40) > 0 )
    {
        /* Detect the hangover period and the first SID frame at the beginning of each CNG phase */

        /* First hangover frame */
        Copy32 (msPeriodog, energy_ho, npart);
        *energy_ho_exp = msPeriodog_exp;
        move16();

        /* Set first SID to current input level but add some smoothing */
        IF ( sub(*active_frame_counter,254) >= 0 )
        {
            lambda = 0;
            move16();
            lambdaM1 = 0x7FFF;
            move16();
        }
        ELSE
        {
            /* -0.94229902485 = 1024.0*log10(0.96)/log10(2.0)/64.0 */
            /* active_frame_counter scaled by (1/1024.0) for compensation */
            tmp32 = L_shl(L_deposit_l(add(*active_frame_counter, 1)), WORD32_BITS-1-10);
            tmp32 = BASOP_Util_InvLog2(Mpy_32_16_1(tmp32, FL2WORD16(-0.94229902485)));
            lambda = extract_h(tmp32);
            lambdaM1 = extract_h(L_sub(0x7FFFFFFF, tmp32));
        }

        invFac = getNormReciprocalWord16(1);

        /* one bit headroom for addition */
        sc = add(s_max(*msNoiseEst_old_exp, *energy_ho_exp), 1);
        s1 = limitScale32(sub(sc, *msNoiseEst_old_exp));
        s2 = limitScale32(sub(sc, *energy_ho_exp));
        *energy_ho_exp = sc;
        move16();

        FOR (i=0; i<npart; i++)
        {
            msNoiseEst_old[i] = Mpy_32_16_1(msNoiseEst_old[i], lambda);
            move32();
            tmp32 = Mpy_32_16_1(Mpy_32_16_1(energy_ho[i], invFac), lambdaM1);
            energy_ho[i] = L_add(L_shr(msNoiseEst_old[i],s1),L_shr(tmp32,s2));
            move32();
        }

        sc = s_max(*msNoiseEst_exp, *energy_ho_exp);
        s1 = limitScale32(sub(sc, *msNoiseEst_exp));
        s2 = limitScale32(sub(sc, *energy_ho_exp));
        *msNoiseEst_exp = sc;
        move16();

        tmp32 = L_add(0,0);
        FOR (i=0; i<npart; i++)
        {
            msNoiseEst_local = L_shr(msNoiseEst[i],s1);
            energy_ho_local = L_shr(energy_ho[i],s2);
            IF ( L_sub(msNoiseEst_local,energy_ho_local) > 0 )
            {
                msNoiseEst[i] = energy_ho_local;
                move32();
            }
            ELSE
            {
                msNoiseEst[i] = msNoiseEst_local;
                move32();
            }
            if ( msNoiseEst[i] > 0 )
            {
                tmp32 = L_add(0,1);
            }
        }
        /* Set exponent to zero if msNoiseEst is zero */
        if ( tmp32==0 )
        {
            *msNoiseEst_exp = 0;
            move16();
        }

        *active_frame_counter = 0;
        move16();
    }
    test();
    IF ( L_sub(stcod->core_brate_fx,SID_2k40) != 0 && L_sub(stcod->core_brate_fx,FRAME_NO_DATA) != 0 )
    {
        /* Count the number of active frames in a row */
        *active_frame_counter = add(*active_frame_counter, 1);
        move16();
    }
    ELSE
    {
        /* Store the noise estimate obtained in the CNG phases */
        Copy32 (msNoiseEst, msNoiseEst_old, npart);
        *msNoiseEst_old_exp = *msNoiseEst_exp;
        move16();

    }


    return 0;
}


/*
   msvq_encoder

    Parameters:

    cb               i  : Codebook (indexed cb[stages][levels][p]) format Q9.7
    u[]              i  : Vector to be encoded (prediction and mean removed) format Q9.7
    levels           i  : Number of levels in each stage
    maxC             i  : Tree search size
    stages           i  : Number of stages
    N                i  : Vector dimension
    maxN             i  : Codebook vector dimension
    Idx              o  : Indices


    Function:
    multi stage vector quantisation

    Returns:
    void
*/
static void msvq_encoder (const Word16 * const cb[], /* i  : Codebook (indexed cb[*stages][levels][p]) scaled with 8 bits */
                          Word16 u[],          /* i  : Vector to be encoded (prediction and mean removed)           */
                          Word16 *levels,      /* i  : Number of levels in each stage                               */
                          Word16 maxC,         /* i  : Tree search size                                             */
                          Word16 stages,       /* i  : Number of stages                                             */
                          Word16 N,            /* i  : Vector dimension                                             */
                          Word16 maxN,         /* i  : Codebook vector dimension                                    */
                          Word16 Idx[]         /* o  : Indices                                                      */
                         )
{
    Word32 *dist[2];
    Word32 t1, en, ss2, tmp;
    const Word16 *cbp, *cb_stage, *p2;
    Word16 *p1, *pTmp;
    Word16 *indices[2], *resid[2], Tmp[M_MAX];
    Word16 i, j, m, s, c, c2, p_max;
    Word16 parents[MBEST_MAX];
    Word32 dist_buf[2*MBEST_MAX];
    Word16 resid_buf[2*MBEST_MAX*M_MAX];
    Word16 idx_buf[2*MBEST_MAX*NSTAGES_MAX];



    /*----------------------------------------------------------------*
    * Allocate memory for previous (parent) and current nodes.
    *   Parent node is indexed [0], current node is indexed [1].
    *----------------------------------------------------------------*/

    indices[0] = idx_buf;
    indices[1] = idx_buf + maxC*stages;
    set16_fx(idx_buf, 0, 2*stages*maxC);

    resid[0] = resid_buf;
    resid[1] = resid_buf + maxC*N;

    dist[0] = dist_buf;
    dist[1] = dist_buf + maxC;

    set16_fx(parents, 0, maxC);

    /*----------------------------------------------------------------*
    * ISF weights are normalized, so it is always better to multiply it first
    * Set up inital distance vector
    *----------------------------------------------------------------*/

    ss2 = L_mult(u[0], u[0]);
    FOR (j=1; j < N; j++)
    {
        ss2 = L_mac(ss2, u[j], u[j]);
    }

    FOR (j=0; j < maxC; j++)
    {
        dist[1][j] = ss2;
        move32();
    }

    /* Set up inital error (residual) vectors */
    pTmp = resid[1];
    FOR (c=0; c<maxC; c++ )
    {
        FOR (j=0; j < N; j++)
        {
            *pTmp++ = u[j];
            move16();
        }
    }

    /* Loop over all stages */
    m = 1;
    move16();
    FOR (s=0; s<stages; s++)
    {
        cbp = cb[s];

        /* Save pointer to beginning of current stage */
        cb_stage = cbp;

        /* Set up pointers to parent and current nodes */
        swap(indices[0], indices[1], Word16*);
        swap(resid[0], resid[1], Word16*);
        swap(dist[0], dist[1], Word32*);

        /* p_max points to maximum distortion node (worst of best) */
        p_max = 0;
        move16();

        /* Set distortions to a large value */
        FOR (j=0; j < maxC; j++)
        {
            dist[1][j] = MAXVAL_WORD32;
            move32();
        }

        FOR (j=0; j<levels[s]; j++)
        {
            /* Compute weighted codebook element and its energy */
            Tmp[0] = cbp[0];
            move16();
            en = L_mult(cbp[0],cbp[0]);
            FOR (i=1; i<N ; i++)
            {
                Tmp[i] = cbp[i];
                move16();
                en = L_mac(en, cbp[i], cbp[i]);
            }

            cbp += maxN;

            /* Iterate over all parent nodes */
            FOR (c=0; c<m; c++)
            {
                pTmp = &resid[0][c*N];

                t1 = L_mult(pTmp[0], Tmp[0]);
                FOR (i=1; i<N ; i++)
                {
                    t1 = L_mac(t1, pTmp[i], Tmp[i]);
                }

                tmp = L_add(dist[0][c], L_sub(en, L_shl(t1, 1)));

                BASOP_SATURATE_WARNING_OFF
                t1 = L_sub(tmp,dist[1][p_max]);
                BASOP_SATURATE_WARNING_ON
                IF ( t1 <= 0 )
                /* IF (L_sub(L_shr(tmp,1), L_shr(dist[1][p_max],1) ) <= 0 ) */
                {
                    /* Replace worst */
                    dist[1][p_max] = tmp;
                    move32();
                    indices[1][p_max*stages+s] = j;
                    move16();
                    parents[p_max] = c;
                    move16();

                    p_max = 0;
                    move16();

                    FOR (i=1; i < maxC; i++)
                    {
                        if (L_sub(dist[1][i],dist[1][p_max]) > 0)
                        {
                            p_max = add(i,0);
                        }
                    }
                }
            }
        } /* FOR (j=0; j<levels[s]; j++) */

        /*------------------------------------------------------------*
        * Compute error vectors for each node
        *------------------------------------------------------------*/
        pTmp = resid[1];

        FOR (c=0; c<maxC; c++)
        {
            /* Subtract codebook entry from residual vector of parent node and multiply with scale factor */
            p1 = resid[0]+parents[c]*N;
            p2 = cb_stage+(indices[1][c*stages+s])*maxN;

            FOR (j=0; j<N; j++)
            {
                pTmp[j] = sub(p1[j], p2[j]);
                move16();
            }
            pTmp += N;

            /* Get indices that were used for parent node */
            Copy(indices[0]+parents[c]*stages, indices[1]+c*stages, s);
        }
        m = maxC;
        move16();
    }  /* FOR (s=0; s<stages; s++) */

    /* Find the optimum candidate (search for minimum) */
    c2 = 0;
    move16();
    FOR (i=1; i < maxC; i++)
    {
        if ( L_sub(dist[1][i], dist[1][c2]) < 0 )
        {
            c2 = i;
            move16();
        }
    }

    Copy(indices[1]+c2*stages, Idx, stages);

}



/*
   FdCng_encodeSID

    Parameters:

    stenc               i/o: pointer to FD_CNG structure containing all buffers and variables
    bitstream           o  : pointer to bitstream
    total_nbbits        o  : pointer to total number of encoded bits
    bitrate             i  : bitrate
    amrwb_io            i  : amr wideband mode
    preemph_fac         i  : preemphase factor


    Function:
    Generate a bitstream out of the partition levels

    Returns:
    void
*/
void FdCng_encodeSID (HANDLE_FD_CNG_ENC stenc,  /* i/o: pointer to FD_CNG structure containing all buffers and variables */
                      Encoder_State_fx *corest,
                      Word16  preemph_fac        /* i  : preemphase factor */
                     )
{
    Word16 i, index, N;
    Word16 E_Exp, normFacN, normShiftN;
    Word16 normFacGain, normShiftGain, sidNoiseEst_Exp;

    Word32 tmp, gain, e, maxVal;
    Word32 *E, E_ExpLd64;
    Word32 v[32];

    Word16 indices[32];
    Word16 v16[32];

    HANDLE_FD_CNG_COM st;




    /* Init */
    st = stenc->hFdCngCom;

    E_Exp = stenc->msNoiseEst_exp;
    move16();
    E_ExpLd64 = L_shl(E_Exp, WORD32_BITS-1-LD_DATA_SCALE);
    E = stenc->msNoiseEst;

    N = stenc->npartDec;
    move16();

    normFacN = getNormReciprocalWord16(N);
    normShiftN = BASOP_util_norm_s_bands2shift(N);

    normFacGain = getNormReciprocalWord16(N_GAIN_MAX-N_GAIN_MIN);
    normShiftGain = BASOP_util_norm_s_bands2shift(N_GAIN_MAX-N_GAIN_MIN);

    /* Convert to LOG */

    /* e: Q14.23 format, v: Q9.23 format */
    e = L_deposit_l(0);
    FOR (i=0; i<N; i++)
    {
        /* assert( E[i] != 0 ); */
        /* constant: 0.75257498916 = 10.0 * log10(2.0)/log10(10.0) * 0.25 */
        v[i] = Mpy_32_16_1(L_add(BASOP_Util_Log2(L_add(E[i],L_max(1,FL2WORD32_SCALE(1e-4f,E_Exp)))), E_ExpLd64), FL2WORD16(0.75257498916));
        move32();
        e = L_add(e, L_shr(v[i], normShiftN));
    }
    e = L_shl(Mpy_32_16_1(e, shl(normFacN, sub(normShiftN, 1))),1);


    /* Normalize MSVQ input */

    /* gain: Q9.23 format */
    gain = L_deposit_l(0);
    FOR (i=N_GAIN_MIN; i<N_GAIN_MAX; i++)
    {
        gain = L_add(gain, L_shr(v[i], normShiftGain));
    }
    gain = L_shl(Mpy_32_16_1(gain, shl(normFacGain, sub(normShiftGain, 1))),1);

    FOR (i=0; i<N; i++)
    {
        v16[i] = extract_h(L_sub(v[i], gain));
    }


    /* MSVQ encoder */
    msvq_encoder (cdk_37bits,
                  v16,
                  levels_37bits,
                  maxC_37bits,
                  stages_37bits,
                  N,
                  maxN_37bits,
                  indices
                 );

    /* MSVQ decoder */
    msvq_decoder (cdk_37bits,
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


    /* Compute gain, Q9.23 format */
    gain = 0;
    FOR (i=0; i<N; i++)
    {
        gain = L_add(gain, L_shr(v[i], normShiftN));
    }
    gain = L_sub(e, L_shl(Mpy_32_16_1(gain, shl(normFacN, sub(normShiftN, 1))),1));


    /* Apply bitrate-dependant scale */
    apply_scale( &gain, st->CngBandwidth, st->CngBitrate );


    /* Quantize gain, Q14.23 format */
    gain = L_add(gain, L_shr(gain,1));
    gain = L_add(gain, FL2WORD32_SCALE(60.5,8));
    index = extract_l(L_shr(gain,WORD32_BITS-1-8));


    if ( index < 0 )
    {
        index = 0;
        move16();
    }

    if ( sub(index,127) > 0)
    {
        index = 127;
        move16();
    }

    /* gain Q14.23 format */
    gain = L_shl(L_deposit_l(index), WORD32_BITS-1-8);
    gain = L_sub(gain, FL2WORD32_SCALE(60.0,8));
    gain = Mpy_32_16_1(gain, FL2WORD16(2.0f/3.0f));


    /* Apply gain and undo log */

    /* sidNoiseEst: format Q6.26, 0.66438561897 = log10(10)/log10(2.0) / 10.0 * 2.0 */

    /* calculate worst case for scaling */
    maxVal = FL2WORD32(-1.0);
    move32();
    FOR (i=0; i<N; i++)
    {
        maxVal = L_max(maxVal,v[i]);
    }

    maxVal = L_add(maxVal, gain);
    maxVal = L_shl(Mpy_32_16_1(maxVal, FL2WORD16(0.66438561897)), 1);

    sidNoiseEst_Exp = 0;
    move16();
    WHILE (maxVal >= 0)
    {
        maxVal = L_sub(maxVal, FL2WORD32(0.015625));
        sidNoiseEst_Exp = add(sidNoiseEst_Exp,1);
    }
    st->sidNoiseEstExp = sidNoiseEst_Exp;
    move16();
    E_ExpLd64 = L_shl(sidNoiseEst_Exp, WORD32_BITS-1-LD_DATA_SCALE);

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
    IF ( sub(st->CngBandwidth,NB) == 0 )
    {
        st->sidNoiseEst[N-1] = Mpy_32_16_1(st->sidNoiseEst[N-1], NB_LAST_BAND_SCALE);
        move32();
    }

    test();
    if (sub( st->CngBandwidth,SWB) == 0 && L_sub(st->CngBitrate,ACELP_13k20) <= 0 )
    {
        st->sidNoiseEst[N-1] = Mpy_32_16_1(st->sidNoiseEst[N-1], SWB_13k2_LAST_BAND_SCALE);
        move32();
    }


    /* Write bitstream */
    IF ( sub(corest->codec_mode, MODE2) == 0 )
    {
        FOR (i=0; i<stages_37bits; i++)
        {
            push_next_indice_fx(corest, indices[i], bits_37bits[i]);
        }
        push_next_indice_fx(corest, index, 7);
    }
    ELSE
    {
        push_indice_fx( corest, IND_SID_TYPE, 1, 1 );
        push_indice_fx( corest, IND_ACELP_16KHZ, corest->bwidth_fx, 2 );
        push_indice_fx( corest, IND_ACELP_16KHZ, corest->L_frame_fx == L_FRAME16k ? 1 : 0, 1 );
        FOR (i=0; i<stages_37bits; i++)
        {
            push_indice_fx( corest, IND_LSF, indices[i], bits_37bits[i] );
        }
        push_indice_fx( corest, IND_ENERGY, index, 7 );
    }

    /* Interpolate the bin/band-wise levels from the partition levels */
    /* sidNoiseEst: Q6.26 format => cngNoiseLevel: Q6.26 format */
    scalebands(st->sidNoiseEst, stenc->partDec, stenc->npartDec, stenc->midbandDec, stenc->nFFTpartDec, sub(stenc->stopBandDec,stenc->startBandDec), st->cngNoiseLevel, 1);
    st->cngNoiseLevelExp = st->sidNoiseEstExp;
    move16();


    lpc_from_spectrum(st->cngNoiseLevel, st->cngNoiseLevelExp, stenc->startBandDec, stenc->stopFFTbinDec, st->fftlen, st->A_cng, M, preemph_fac );


}


void generate_comfort_noise_enc (Encoder_State_fx *stcod,
                                 Word16 Q_new,
                                 Word16 gen_exc
                                )
{
    Word16  i, s, sn, cnt;
    Word16  startBand2;
    Word16  stopFFTbin2;
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
    HANDLE_FD_CNG_ENC stenc = stcod->hFdCngEnc_fx;
    HANDLE_FD_CNG_COM st = stenc->hFdCngCom;



    /* Warning fix */
    s = 0;

    /* pointer initialization */

    cngNoiseLevel = st->cngNoiseLevel;
    cngNoiseLevelExp = st->cngNoiseLevelExp;
    ptr_level = cngNoiseLevel;
    seed = &(st->seed);
    fftBuffer = st->fftBuffer;
    timeDomainOutput = st->timeDomainBuffer;

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
    cnt = sub(stenc->stopFFTbinDec, stenc->startBandDec);
    IF ( stenc->startBandDec == 0 )
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
        startBand2 = shl(stenc->startBandDec,1);
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
    stopFFTbin2 = shl(stenc->stopFFTbinDec,1);
    set32_fx(fftBuffer+stopFFTbin2, 0, sub(st->fftlen,stopFFTbin2));

    fftBufferExp = add(shr(cngNoiseLevelExp,1),randGaussExp);

    /* If previous frame is active, reset the overlap-add buffer */
    IF ( L_sub(stcod->last_core_brate_fx,SID_2k40) > 0 )
    {
        set16_fx(st->olapBufferSynth, 0, st->fftlen);
        test();
        test();
        IF ( (L_sub(stcod->last_core_fx,ACELP_CORE) > 0 && sub(stcod->codec_mode,MODE2) == 0) || sub(stcod->codec_mode,MODE1) == 0 )
        {
            tcx_transition = 1;
            move16();
        }
    }

    /* Perform STFT synthesis */
    SynthesisSTFT (fftBuffer, fftBufferExp, timeDomainOutput, st->olapBufferSynth, st->olapWinSyn,
                   tcx_transition,
                   st, gen_exc, &Q_new);
    {
        Word32 Lener, att;
        Word16 exp;
        /* update CNG excitation energy for LP_CNG */

        /* calculate the residual signal energy */
        /*enr = dotp( st->exc_cng, st->exc_cng, st->frameSize ) / st->frameSize;*/
        Lener = Dot_productSq16HQ(1,st->exc_cng,stcod->L_frame_fx,&exp);
        exp = add(sub(shl(sub(15,Q_new),1),8),exp);     /*8 = log2(256)*/

        /* convert log2 of residual signal energy */
        /*(float)log10( enr + 0.1f ) / (float)log10( 2.0f );*/
        Lener = BASOP_Util_Log2(Lener);
        Lener = L_add(Lener,L_shl(L_deposit_l(exp),WORD32_BITS-1-LD_DATA_SCALE)); /*Q25*/
        if(sub(stcod->L_frame_fx,L_FRAME16k) == 0)
        {
            Lener = L_sub(Lener, FL2WORD32_SCALE(0.3219280949f, LD_DATA_SCALE)); /*log2(320) = 8.3219280949f*/
        }
        /* decrease the energy in case of WB input */
        IF( sub(stcod->bwidth_fx, NB) != 0 )
        {
            IF( sub(stcod->bwidth_fx,WB) == 0 )
            {
                IF( stcod->CNG_mode_fx >= 0 )
                {
                    /* Bitrate adapted attenuation */
                    att = L_shl(L_deposit_l(ENR_ATT_fx[stcod->CNG_mode_fx]),17);
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
                move32();/*1.5 Q8<<17=Q25*/
            }
            Lener = L_sub(Lener, att );
        }
        /*stdec->lp_ener = 0.8f * stcod->lp_ener + 0.2f * pow( 2.0f, enr );*/
        Lener = BASOP_util_Pow2(Lener, 6, &exp);
        Lener = Mult_32_16(Lener, FL2WORD16(0.2f));
        exp = sub(25,exp);
        Lener = L_shr(Lener, exp);  /*Q6*/
        stcod->lp_ener_fx = L_add(Mult_32_16(stcod->lp_ener_fx, FL2WORD16(0.8f)), Lener);  /*Q6*/
    }

    /* Overlap-add when previous frame is active */
    test();
    IF ( ( L_sub(stcod->last_core_brate_fx,SID_2k40) > 0 ) && ( sub(stcod->codec_mode,MODE2) == 0 ) )
    {
        Word32 old_exc_ener, gain, noise32;
        Word16 seed, lpcorder, old_syn, tmp, gain16, N, N2, N4, N8;
        Word16 old_exc_ener_exp, gain_exp;
        Word16 normFacE, normShiftE, normShiftEM1;
        Word16 normFacG, normShiftG, normShiftGM1;
        Word16 noiseExp, *old_exc, old_Aq[M+1], *old_syn_pe;
        Word16 noise[640], normShiftP2;
        Word16 Q_exc, Q_syn;


        assert(st->frameSize <= 640);

        seed = st->seed;
        move16();
        N = st->frameSize;
        move16();
        N2 = shr(st->frameSize,1);

        IF ( sub(stcod->last_core_fx,ACELP_CORE) > 0 )
        {
            tcx_windowing_synthesis_current_frame(  timeDomainOutput,
                                                    stcod->tcx_cfg.tcx_mdct_window, /*Keep sine windows for limiting Time modulation*/
                                                    stcod->tcx_cfg.tcx_mdct_window_half,
                                                    stcod->tcx_cfg.tcx_mdct_window_minimum,
                                                    stcod->tcx_cfg.tcx_mdct_window_length,
                                                    stcod->tcx_cfg.tcx_mdct_window_half_length,
                                                    stcod->tcx_cfg.tcx_mdct_window_min_length,
                                                    0,
                                                    stcod->tcx_cfg.tcx_last_overlap_mode==ALDO_WINDOW?FULL_OVERLAP:stcod->tcx_cfg.tcx_last_overlap_mode,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    NULL,
                                                    N/2,
                                                    stcod->tcx_cfg.tcx_offset<0?-stcod->tcx_cfg.tcx_offset:0,
                                                    0,
                                                    1,
                                                    0
                                                    ,0
                                                 );

            IF (stcod->tcx_cfg.last_aldo != 0)
            {
                FOR (i=0; i<st->frameSize; i++)
                {
                    timeDomainOutput[i] = add(timeDomainOutput[i], shr_r(stcod->old_out_fx[i+NS2SA(stcod->sr_core, N_ZERO_MDCT_NS)],stcod->Q_old_out));
                    move16();
                }
            }
            ELSE
            {
                tcx_windowing_synthesis_past_frame( stcod->LPDmem.Txnq,
                stcod->tcx_cfg.tcx_aldo_window_1_trunc,
                stcod->tcx_cfg.tcx_mdct_window_half,
                stcod->tcx_cfg.tcx_mdct_window_minimum,
                stcod->tcx_cfg.tcx_mdct_window_length,
                stcod->tcx_cfg.tcx_mdct_window_half_length,
                stcod->tcx_cfg.tcx_mdct_window_min_length,
                stcod->tcx_cfg.tcx_last_overlap_mode
                                                  );

                FOR (i=0; i<N2; i++)
                {
                    timeDomainOutput[i] = add(timeDomainOutput[i],shl(stcod->LPDmem.Txnq[i],TCX_IMDCT_HEADROOM));
                    move16();
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
            E_LPC_f_lsp_a_conversion(stcod->lsp_old_fx, old_Aq, M);
            old_exc = stcod->LPDmem.old_exc+sub(L_EXC_MEM,N2);
            old_syn_pe = stcod->LPDmem.mem_syn2;
            old_syn = stcod->LPDmem.syn[lpcorder];
            move16();
            preemph_fac = stcod->preemph_fac;
            move16();
            Q_exc = Q_new;
            Q_syn = sub(Q_new,1);

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
                noise32 = rand_gauss (&seed);
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
                noise,        /* I/O: signal              Qx */
                preemph_fac,  /* I: deemphasis factor      Qx */
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

