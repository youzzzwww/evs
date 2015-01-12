/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "stl.h"
#include "basop_util.h"
#include "options.h"
#include "stl.h"
#include "rom_basop_util.h"
#include "rom_com_fx.h"
#include "prot_fx.h"

#define DELTA_SHIFT         2
#define DELTA_SHIFT_LD64    FL2WORD32(DELTA_SHIFT/64.0)



/*****************************************
* Create an instance of type FD_CNG_COM *
*****************************************/
void createFdCngCom(HANDLE_FD_CNG_COM * hFdCngCom)
{
    HANDLE_FD_CNG_COM hs;

    /* Allocate memory */
    hs = (HANDLE_FD_CNG_COM) calloc(1, sizeof (FD_CNG_COM));
    move16();


    *hFdCngCom = hs;
    move16();
    return;
}

void initFdCngCom(HANDLE_FD_CNG_COM hs, Word16 scale)
{
    /* Calculate CLDFB scaling factor */
    /* shl(i_mult2(scale, scale), 3) does not fit in 16 bit */
    /*hs->scalingFactor = div_s(1, shl(i_mult2(scale, scale), 3));*/
    assert(FL2WORD16(1.0/(1<<4)) < mult(scale, scale));
    /* Exponent invScalingFactor: -16 = -(2*7 (scale) + 2 (8.0) */
    hs->invScalingFactor = shl(mult(scale,scale),1);
    /* Exponent scalingFactor: -15 = -(2*7 (scale) + 2 (8.0) - 1 (1.0)) */
    hs->scalingFactor = div_s(0x4000,hs->invScalingFactor);

    /* Initialize the overlap-add */
    set16_fx( hs->timeDomainBuffer, 0, L_FRAME16k );
    hs->olapBufferAna = NULL;
    move16();
    set16_fx( hs->olapBufferSynth, 0, FFTLEN );
    hs->olapBufferSynth2 = NULL;
    move16();

    /* Initialize the comfort noise generation */
    set32_fx( hs->fftBuffer, 0, FFTLEN );
    set32_fx( hs->cngNoiseLevel, 0, FFTCLDFBLEN );

    /* Initialize quantizer */
    set32_fx( hs->sidNoiseEst, 0, NPART );
    set16_fx( hs->A_cng, 0, M+1 );
    hs->A_cng[0] = FL2WORD16_SCALE(1.f, 3);   /* 3Q12 */                                  move16();

    /* Set some counters and flags */
    hs->inactive_frame_counter  = 0; /* Either SID or zero frames */                  move16();
    hs->active_frame_counter    = 0;
    move16();
    hs->frame_type_previous     = ACTIVE_FRAME;
    move16();
    hs->flag_noisy_speech       = 0;
    move16();
    hs->likelihood_noisy_speech = 0;
    move16();

    /* Initialize noise estimation algorithm */
    set32_fx( hs->periodog, 0, FFTCLDFBLEN );
    mhvals(MSNUMSUBFR*MSSUBFRLEN, &(hs->msM_win));
    mhvals(MSSUBFRLEN, &(hs->msM_subwin));
    set32_fx( hs->msPeriodogSum, 0, 2 );
    hs->msPeriodogSum_exp[0] = 0;
    move16();
    hs->msPeriodogSum_exp[1] = 0;
    move16();
    set32_fx( hs->msPsdSum, 0, 2 );
    set16_fx( hs->msSlope, 0, 2 );
    set32_fx( hs->msQeqInvAv, 0, 2 );
    hs->msQeqInvAv_exp[0] = 0;
    move16();
    hs->msQeqInvAv_exp[1] = 0;
    move16();
    hs->msFrCnt_init_counter = 0;
    move16();
    hs->msFrCnt_init_thresh  = 1;
    move16();
    hs->init_old             = 0;
    move16();
    hs->offsetflag           = 0;
    move16();
    hs->msFrCnt              = MSSUBFRLEN;
    move16();
    hs->msMinBufferPtr       = 0;
    move16();
    hs->msAlphaCor[0] = FL2WORD32(0.3f);
    move16();
    hs->msAlphaCor[1] = FL2WORD32(0.3f);
    move16();

    /* Initialize exponents */
    hs->exp_cldfb_periodog = 0;
    move16();

    return;
}

/*****************************************
* Delete an instance of type FD_CNG_COM *
*****************************************/
void deleteFdCngCom(HANDLE_FD_CNG_COM * hFdCngCom) /* i/o: Contains the variables related to the CLDFB-based CNG process */
{
    HANDLE_FD_CNG_COM hsCom;
    hsCom = *hFdCngCom;
    move16();
    IF (hsCom != NULL)
    {
        free(hsCom);
        *hFdCngCom = NULL;
        move16();
    }
}

/***************************************
* Initialize the spectral partitioning *
****************************************/
void initPartitions( const Word16* part_in,
                     Word16  npart_in,
                     Word16  startBand,
                     Word16  stopBand,
                     Word16* part_out,
                     Word16* npart_out,
                     Word16* midband,
                     Word16* psize,
                     Word16* psize_norm,
                     Word16* psize_norm_exp,
                     Word16* psize_inv,
                     Word16  stopBandFR
                   )
{
    Word16 i, j, len_out, tmp16;


    IF (part_in != NULL)
    {
        len_out = 0;
        move16();
        IF (sub(stopBandFR, startBand) > 0)
        {
            len_out = sub(stopBandFR, startBand);
            FOR(i = 0 ; i < len_out; i++)
            {
                part_out[i] = i;
                move16();
            }
        }
        FOR(j=0 ; j < npart_in; j++)
        {
            IF (sub(part_in[j], stopBand) >= 0)
            {
                BREAK;
            }
            tmp16 = sub(part_in[j],startBand);
            test();
            if (sub(part_in[j],stopBandFR) >= 0 && tmp16 >= 0)
            {
                part_out[len_out++] = tmp16;
                move16();
            }
        }
    }
    ELSE
    {
        len_out = sub(stopBand, startBand);
        FOR (i = 0 ; i < len_out; i++)
        {
            part_out[i] = i;
            move16();
        }
    }

    *npart_out = len_out;
    move16();
    getmidbands(part_out, len_out, midband, psize, psize_norm, psize_norm_exp, psize_inv);

}

#define CNG_HS     4    /* 4 bit headroom for dot product */
#define CNG_S      6    /* 1 sign bit, 6 bit integer part, 9 bit frational part for input and output data */




/********************************************************
* Apply some dynamic range compression based on the log *
********************************************************/
void compress_range(
    Word32 *in,
    Word16  in_exp,
    Word16 *out,
    Word16  len
)
{
    Word16 i;
    Word32 in_s;
    Word32 one_s;
    Word32 in_exp32;
    Word32 L_tmp;


    /* out = log2( 1 + in ) */
    IF ( in_exp >= 0 )
    {
        one_s = L_shr(FL2WORD32(0.5),in_exp);
        in_exp32 = L_shl(L_deposit_l(add(in_exp,1)),WORD32_BITS-1-LD_DATA_SCALE);
        FOR (i=0; i < len; i++)
        {
            in_s = L_add(L_shr(in[i],1),one_s);
            L_tmp = L_add(BASOP_Util_Log2(in_s),in_exp32);
            if (in_s == 0)
            {
                out[i] = 0;
                move16();
            }
            if (in_s != 0)
            {
                out[i] = extract_h(L_tmp);
            }

        }
    }
    ELSE
    {
        in_exp = sub(in_exp,1);
        in_exp32 = L_shl(L_deposit_l(1),WORD32_BITS-1-LD_DATA_SCALE);
        FOR (i=0; i < len; i++)
        {
            L_tmp = L_add(BASOP_Util_Log2(L_add(L_shl(in[i],in_exp),FL2WORD32(0.5))),in_exp32);
            if (in[i] == 0)
            {
                out[i] = 0;
                move16();
            }
            if (in[i] != 0)
            {
                out[i] = extract_h(L_tmp);
            }
        }
    }

}

/*************************************************************
* Apply some dynamic range expansion to undo the compression *
*************************************************************/
void expand_range(
    Word16 *in,
    Word32 *out,
    Word16 *out_exp,
    Word16  len
)
{
    Word16 i;
    Word16 s;
    Word16 tmp;
    Word32 one_s, tmp32;
    Word16 maxVal;
    Word16 maxVal2;


    maxVal = 0;
    move16();
    FOR (i=0; i < len; i++)
    {
        maxVal = s_max(maxVal,in[i]);
    }

    maxVal2 = maxVal;
    move16();
    s = 0;
    move16();
    WHILE ( maxVal >= 0 )
    {
        maxVal = sub(maxVal,FL2WORD16(0.015625));
        s = add(s,1);
    }
    tmp = sub(maxVal2,maxVal);

    one_s = L_shr(FL2WORD32(0.5),sub(s,1));
    tmp32 = L_shr(FL2WORD32(0.0003385080526823181),s);
    /* out = (2^(in) - 1) */
    FOR (i=0; i < len; i++)
    {
        out[i] = L_sub(BASOP_Util_InvLog2(L_deposit_h(sub(in[i],tmp))),one_s);
        move32();
        if ( out[i] == 0 )
        {
            out[i] = tmp32;
            move32();
        }
    }
    *out_exp = s;
    move16();

}

/*************************************************
* Noise estimation using Minimum Statistics (MS) *
*************************************************/
void minimum_statistics (
    Word16  len,                    /* i  : Total number of partitions (CLDFB or FFT)                   */
    Word16  lenFFT,                 /* i  : Number of FFT partitions                                  */
    Word16 *psize,                  /* i  : Partition sizes, fractional                               */
    Word16 *msPeriodog,             /* i  : Periodogram (energies)                                    */
    Word16 *msNoiseFloor,           /* i/o: Noise floors (energies)                                   */
    Word16 *msNoiseEst,             /* i/o: Noise estimates (energies)                                */
    Word32 *msAlpha,                /* i/o: Forgetting factors                                        */
    Word16 *msPsd,                  /* i/o: Power Spectral Density (smoothed periodogram => energies) */
    Word16 *msPsdFirstMoment,       /* i/o: PSD statistics of 1st order (energy means)                */
    Word32 *msPsdSecondMoment,      /* i/o: PSD statistics of 2nd order (energy variances)            */
    Word32 *msMinBuf,               /* i/o: Buffer of minima (energies)                               */
    Word32 *msBminWin,              /* o  : Bias correction factors                                   */
    Word32 *msBminSubWin,           /* o  : Bias correction factors                                   */
    Word32 *msCurrentMin,           /* i/o: Local minima (energies)                                   */
    Word32 *msCurrentMinOut,        /* i/o: Local minima (energies)                                   */
    Word32 *msCurrentMinSubWindow,  /* i/o: Local minima (energies)                                   */
    Word16 *msLocalMinFlag,         /* i  : Binary flag                                               */
    Word16 *msNewMinFlag,           /* i  : Binary flag                                               */
    Word16 *msPeriodogBuf,          /* i/o: Buffer of periodograms (energies)                         */
    Word16 *msPeriodogBufPtr,       /* i/o: Counter                                                   */
    HANDLE_FD_CNG_COM st            /* i/o: FD_CNG structure containing buffers and variables         */
)
{
    Word16 i,j,k,s,s1,s2,s3;
    Word16 len2;
    Word16 current_len;
    Word16 start, stop, cnt;
    Word16 totsize;
    Word16 inv_totsize;

    Word32  tmp, tmp0, tmp1;
    Word32  scalar, scalar2, scalar3;
    Word32  snr;
    Word32  msAlphaHatMin2;
    Word32  BminCorr;
    Word32  QeqInvAv;
    Word32 *ptr;
    Word32 *msPsdSum;
    Word32 *msPeriodogSum;

    Word16 tmp16, tmp16_1;
    Word16 beta;
    Word16 slope;
    Word16 QeqInv;
    Word16 scalar16;
    Word16 scalar216;
    Word16 scalar316;
    Word16 msM_win;
    Word16 msM_subwin;
    Word16 msAlphaCorAlpha;
    Word16 msAlphaCorAlpha2;
    Word16 msPeriodogSum16;
    Word16 msNoiseFloor16;


    len2 = i_mult(MSNUMSUBFR,len);

    msM_win = st->msM_win;
    move16();
    msM_subwin = st->msM_subwin;
    move16();

    msAlphaCorAlpha = MSALPHACORALPHA;
    move16();
    msAlphaCorAlpha2 = MSALPHACORALPHA2;
    move16();

    msPsdSum = st->msPsdSum;
    msPeriodogSum = st->msPeriodogSum;

    /* No minimum statistics at initialization */
    IF ( sub(st->msFrCnt_init_counter,st->msFrCnt_init_thresh) < 0 )
    {
        Copy(msPeriodog, msPsd, len);            /* 6Q9 */
        Copy(msPeriodog, msNoiseFloor, len);     /* 6Q9 */
        Copy(msPeriodog, msNoiseEst, len);       /* 6Q9 */
        Copy(msPeriodog, msPsdFirstMoment, len); /* 6Q9 */

        set32_fx(msPsdSecondMoment, FL2WORD32(0.0), len);
        msPeriodogSum[0] = dotp_s_fx(msPeriodog, psize, lenFFT, CNG_HS);
        move32();
        msPsdSum[0] = msPeriodogSum[0];   /* 16Q15 */                                                            move32();

        IF ( sub(lenFFT,len) < 0 )
        {
            msPeriodogSum[1] = dotp_s_fx(msPeriodog+lenFFT, psize+lenFFT, sub(len,lenFFT), CNG_HS);
            move32();
            msPsdSum[1] = msPeriodogSum[1]; /* 16Q15 */                                                            move32();
        }

        /* Increment frame counter at initialization */
        /* Some frames are sometimes zero at initialization => ignore them */
        IF ( sub(msPeriodog[0],st->init_old) < 0 )
        {
            set32_fx(msCurrentMinOut, FL2WORD32(1.0), len);        /* 16Q15 */
            set32_fx(msCurrentMin, FL2WORD32(1.0), len);           /* 16Q15 */
            set32_fx(msMinBuf, FL2WORD32(1.0), len2);              /* 16Q15 */
            set32_fx(msCurrentMinSubWindow, FL2WORD32(1.0), len);  /* 16Q15 */

            st->msFrCnt_init_counter = add(st->msFrCnt_init_counter,1);
            move16();
        }
        st->init_old = msPeriodog[0];                       /* 6Q9 */                                            move16();
    }
    ELSE
    {
        /* Consider the FFT and CLDFB bands separately
           - first iteration for FFT bins,
           - second one for CLDFB bands in SWB mode */
        cnt = 0;
        move16();
        start = 0;
        move16();
        stop = lenFFT;
        move16();
        totsize = sub(st->stopFFTbin,st->startBand);
        WHILE ( sub(stop,start) > 0 )
        {
            current_len = sub(stop,start);

            /* Compute smoothed correction factor for PSD smoothing */

            /* msPeriodogSum[cnt] with format 16Q15 */
            msPeriodogSum[cnt] = dotp_s_fx(msPeriodog+start, psize+start, current_len, CNG_HS);
            move32();

            IF ( msPeriodogSum[cnt] == 0 )
            {
                st->msAlphaCor[cnt] = Mpy_32_16_1(st->msAlphaCor[cnt], msAlphaCorAlpha);
                move32();
            }
            ELSE
            {
                /* calculate scalar with normalized msPeriodogSum[cnt], exponent -2*s1 */
                s1 = norm_l(msPeriodogSum[cnt]);
                msPeriodogSum16 = round_fx(L_shl(msPeriodogSum[cnt],s1));
                scalar = L_mult(msPeriodogSum16, msPeriodogSum16);

                /* calculate difference, both elements in 16Q15 format, use absolute value
                   to avoid -1.0 x -1.0 multiplications later */
                scalar2 = L_abs(L_sub(msPsdSum[cnt], msPeriodogSum[cnt]));

                s2 = WORD32_BITS-1;
                move16();
                if ( scalar2 != 0 )
                {
                    /* use absolute value to avoid -1.0 x -1.0 multiplications */
                    s2 = norm_l(scalar2);
                }
                scalar216 = round_fx(L_shl(scalar2,s2));
                scalar2 = L_mult(scalar216, scalar216);

                /* find common exponent */
                tmp16_1 = sub(s1,s2);
                tmp16 = s_min(shl(abs_s(tmp16_1),1),WORD32_BITS-1);
                if ( tmp16_1 < 0 )
                {
                    scalar2 = L_shr(scalar2,tmp16);
                }
                if(tmp16_1 > 0)
                {
                    scalar = L_shr(scalar,tmp16);
                }


                /* add scalar and scalar2, avoid overflows */
                scalar  = L_shr(scalar,1);
                scalar2 = L_shr(scalar2,1);
                scalar3 = L_add(scalar,scalar2);

                /* calculate division */
                scalar16 = BASOP_Util_Divide3232_uu_1616_Scale(scalar, scalar3, &s3);
                s3 = max(s3,-(WORD16_BITS-1));
                scalar16 = shl(scalar16,s3);
                scalar16 = s_max(scalar16, MSALPHACORMAX);

                st->msAlphaCor[cnt] = L_add(Mpy_32_16_1(st->msAlphaCor[cnt], msAlphaCorAlpha),
                L_mult(scalar16, msAlphaCorAlpha2));
                move32();
            }

            /* Compute SNR */

            /* msPeriodogSum[cnt] with format 16Q15 */
            snr = dotp_s_fx(msNoiseFloor+start, psize+start, current_len, CNG_HS);

            IF ( L_sub(L_shr(Mpy_32_16_1(msPsdSum[cnt],FL2WORD16(0.56246299817)),13),snr) > 0 )
            {
                tmp0 = BASOP_Util_Log2(msPsdSum[cnt]);
                tmp1 = BASOP_Util_Log2(snr);
                tmp1 = L_sub(tmp0, tmp1);
                tmp1 = Mpy_32_16_1(tmp1, MSSNREXP);
                msAlphaHatMin2 = BASOP_Util_InvLog2(tmp1);
            }
            ELSE
            {
                msAlphaHatMin2 = MSALPHAHATMIN;
                move32();
            }
            scalar = Mpy_32_16_1(st->msAlphaCor[cnt], MSALPHAMAX);

            FOR (j=start; j<stop; j++)
            {
                /* Compute optimal smoothing parameter for PSD estimation */                                         test();
                test();
                IF ( (scalar == 0) || (msNoiseFloor[j] == 0) )
                {
                    msAlpha[j] = msAlphaHatMin2;
                    move32();
                }
                ELSE
                {
                    /* calculate scalar2 with normalized msNoiseFloor[j], exponent -2*s1 */
                    s1 = WORD16_BITS-1;
                    move16();
                    if ( msNoiseFloor[j] != 0 )
                    {
                        s1 = norm_s(msNoiseFloor[j]);
                    }
                    msNoiseFloor16 = shl(msNoiseFloor[j],s1);
                    scalar2 = L_mult(msNoiseFloor16, msNoiseFloor16);

                    /* calculate difference, both elements in 6Q9 format, use absolute value
                       to avoid -1.0 x -1.0 multiplications later */
                    scalar316 = abs_s(sub(msPsd[j],msNoiseFloor[j]));

                    s2 = WORD16_BITS-1;
                    move16();
                    if ( scalar316 != 0 )
                    {
                        /* use absolute value to avoid -1.0 x -1.0 multiplications */
                        s2 = norm_s(scalar316);
                    }
                    scalar316 = shl(scalar316,s2);
                    scalar3 = L_mult(scalar316, scalar316);

                    /* find common exponent */
                    tmp16_1 = sub(s1,s2);
                    tmp16 = s_min(shl(abs_s(tmp16_1),1),WORD32_BITS-1);
                    if ( tmp16_1 < 0 )
                    {
                        scalar3 = L_shr(scalar3,tmp16);
                    }
                    if(tmp16_1 > 0)
                    {
                        scalar2 = L_shr(scalar2,tmp16);
                    }

                    /* add scalar2 and scalar3, avoid overflows */
                    scalar2 = L_shr(scalar2,1);
                    scalar3 = L_shr(scalar3,1);
                    scalar3 = L_add(scalar2,scalar3);

                    /* calculate division */
                    tmp16 = BASOP_Util_Divide3232_uu_1616_Scale(scalar2, scalar3, &s3);
                    scalar2 = Mpy_32_16_1(scalar, tmp16);
                    s3 = s_max(s3,-(WORD32_BITS-1));
                    scalar2 = L_shl(scalar2,s3);
                    msAlpha[j] = L_max(scalar2, msAlphaHatMin2);
                    move32();
                }

                /* Compute the PSD (smoothed periodogram) in each band */
                msPsd[j] = round_fx(L_add(Mpy_32_16_1(msAlpha[j], msPsd[j]),
                                          Mpy_32_16_1(L_sub(FL2WORD32(1.0),msAlpha[j]), msPeriodog[j])));
            }
            msPsdSum[cnt] = dotp_s_fx(msPsd+start, psize+start, current_len, CNG_HS);
            move32();

            QeqInvAv = FL2WORD32(0.0);
            move32();

            /* scalar: 7Q24 format */
            tmp = FL2WORD32_SCALE((float)(MSNUMSUBFR*MSSUBFRLEN)-1.0,7);
            move32();
            scalar = L_sub(tmp, L_mult(round_fx(tmp), msM_win));

            /* scalar2: 4Q27 format */
            tmp = FL2WORD32_SCALE((float)MSSUBFRLEN-1.0, 4);
            move32();
            scalar2 = L_sub(tmp, L_mult(round_fx(tmp), msM_subwin));

            FOR (j=start; j<stop; j++)
            {
                /* Compute variance of PSD */
                tmp = L_min(msAlpha[j], MSBETAMAX_SQRT);

                s1 = WORD32_BITS-1;
                move16();
                if ( tmp != 0 )
                {
                    s1 = norm_l(tmp);
                }
                s2 = shl(s1,1);
                s3 = shl(s2,1);

                s2 = s_min(s2,WORD32_BITS-1);
                s3 = s_min(s3,WORD32_BITS-1);

                /* beta: scaled by s2 */
                tmp16 = round_fx(L_shl(tmp, s1));
                beta = mult_r(tmp16, tmp16);

                /* scalar3: scaled by s2 */
                scalar3 = L_sub(msPsd[j], msPsdFirstMoment[j]);

                /* msPsdFirstMoment: 6Q9   */
                tmp = L_msu(L_mult(beta, msPsdFirstMoment[j]), beta, msPsd[j]);
                msPsdFirstMoment[j] = add(round_fx(L_shr(tmp, s2)), msPsd[j]);
                move16();

                /* msPsdSecondMoment: 12Q19   */
                tmp0 = L_shr(Mpy_32_16_1(msPsdSecondMoment[j], beta), s2);
                tmp1 = Mpy_32_32(scalar3, scalar3);
                tmp1 = L_shr(L_sub(tmp1, L_shr(Mpy_32_16_1(tmp1, beta), s2)), s3);
                msPsdSecondMoment[j] = L_add(tmp0, tmp1);
                move32();

                /* Compute inverse of amount of degrees of freedom */
                QeqInv = MSQEQINVMAX;
                move16();

                IF ( msNoiseFloor[j] != FL2WORD16(0.0) )
                {
                    tmp = L_mult(msNoiseFloor[j], msNoiseFloor[j]);
                    tmp16 = BASOP_Util_Divide3232_uu_1616_Scale(msPsdSecondMoment[j], tmp, &s);
                    /* consider factor of 2 */
                    s = s_min(s_max(sub(s,1),-(WORD16_BITS-1)),(WORD16_BITS-1));
                    if ( s < 0 )
                    {
                        QeqInv = shl(tmp16, s);
                    }
                    QeqInv = s_min(QeqInv, MSQEQINVMAX);
                }
                QeqInvAv = L_add(QeqInvAv, L_mult(QeqInv, psize[j]));

                /* Compute bias correction Bmin */
                tmp0 = Mpy_32_16_1(scalar, QeqInv);
                tmp1 = L_sub(FL2WORD32_SCALE(0.5,0), L_mult(msM_win, QeqInv));
                tmp16 = BASOP_Util_Divide3232_uu_1616_Scale(tmp0, tmp1, &s);
                msBminWin[j] = L_add(FL2WORD32_SCALE(1.0,4), L_shl(L_deposit_h(tmp16),add(s,7-4)));
                move32();

                tmp0 = Mpy_32_16_1(scalar2, QeqInv);
                tmp1 = L_sub(FL2WORD32(0.5), L_mult(msM_subwin, QeqInv));
                tmp16 = BASOP_Util_Divide3232_uu_1616_Scale(tmp0, tmp1, &s);
                msBminSubWin[j] = L_add(FL2WORD32_SCALE(1.0,4), L_shl(L_deposit_h(tmp16),s));
                move32();
            }

            inv_totsize = BASOP_Util_Divide3232_uu_1616_Scale(1, totsize, &s);
            QeqInvAv = Mpy_32_16_1(QeqInvAv, inv_totsize);
            QeqInvAv = L_shl(QeqInvAv, s);
            st->msQeqInvAv[cnt] = QeqInvAv;
            move32();

            /* New minimum? */

            /* exponent QeqInvAv: CNG_S, exponent MSAV: (4>>1) */
            s = CNG_S+2*MSAV_EXP;
            move16();
            BminCorr = Mpy_32_16_1(Sqrt32(QeqInvAv, &s), MSAV);
            BminCorr = L_shl(BminCorr,sub(s,1));

            /* exponent BminCorr: 1 */
            BminCorr = L_add(BminCorr, FL2WORD32(0.5));

            FOR (j=start; j<stop; j++)
            {
                /* exponent scalar: CNG_S+1 */
                scalar = Mpy_32_16_1(BminCorr, msPsd[j]);

                /* exponent scalar2: CNG_S+1+4 */
                scalar2 = Mpy_32_32(scalar, msBminWin[j]);

                msNewMinFlag[j] = 0;
                move16();
                IF ( L_sub(scalar2,msCurrentMin[j]) < FL2WORD32(0.0) )
                {
                    msNewMinFlag[j] = 1;
                    move16();
                    /* exponent msCurrentMin[j]: CNG_S+1+4 */
                    msCurrentMin[j] = scalar2;
                    move32();
                    /* exponent msCurrentMinSubWindow[j]: CNG_S */
                    BASOP_SATURATE_WARNING_OFF;
                    msCurrentMinSubWindow[j] = L_shl(Mpy_32_32(scalar, msBminSubWin[j]),5);
                    move32();
                    BASOP_SATURATE_WARNING_ON;
                }
            }

            /* This is used later to identify local minima */
            IF ( sub(st->msFrCnt,MSSUBFRLEN) >= 0 )
            {
                FOR ( i = 0; i < 3; i++ )
                {
                    IF ( L_sub(st->msQeqInvAv[cnt],L_shr(L_deposit_h(msQeqInvAv_thresh[i]),CNG_S)) < FL2WORD32(0.0) )
                    {
                        BREAK;
                    }
                }
                /* format 1Q30 */
                st->msSlope[cnt] = msNoiseSlopeMax[i];
                move32();
            }

            /* Consider the FFT and CLDFB bands separately */
            start = stop;
            move16();
            stop  = len;
            move16();
            totsize = sub(st->stopBand,st->stopFFTbin);
            cnt = add(cnt,1);
        } /*while (stop > start)*/

        /* Update minimum between sub windows */
        test();
        IF ( sub(st->msFrCnt,1) > 0 && sub(st->msFrCnt,MSSUBFRLEN) < 0 )
        {
            FOR (j=0; j<len; j++)
            {
                if ( msNewMinFlag[j] > 0 )
                {
                    msLocalMinFlag[j] = 1;
                    move16();
                }
                if ( L_sub(msCurrentMinSubWindow[j],msCurrentMinOut[j]) < FL2WORD32(0.0) )
                {
                    /* msCurrentMinOut[j] scaled with CNG_S */
                    msCurrentMinOut[j] = msCurrentMinSubWindow[j];
                    move32();
                }
            }
            /* Get the current noise floor */
            Copy_Scale_sig_32_16(msCurrentMinOut, msNoiseFloor, len, -16);
        }
        ELSE /* sub window complete */
        {
            IF ( sub(st->msFrCnt,MSSUBFRLEN) >= 0 )
            {
                /* Collect buffers */
                Copy32(msCurrentMinSubWindow, msMinBuf+len*st->msMinBufferPtr, len);

                /* Compute minimum among all buffers */
                Copy32(msMinBuf, msCurrentMinOut, len);
                ptr = msMinBuf + len;
                FOR (i=1; i<MSNUMSUBFR; i++)
                {
                    FOR (j=0; j<len; j++)
                    {
                        if ( L_sub(*ptr,msCurrentMinOut[j]) < FL2WORD32(0.0) )
                        {
                            msCurrentMinOut[j] = *ptr;
                            move32();
                        }
                        ptr++;
                    }
                }

                /* Take over local minima */
                slope = st->msSlope[0];
                move16();
                FOR (j=0; j<len; j++)
                {
                    if ( sub(j,lenFFT) == 0 )
                    {
                        slope = st->msSlope[1];
                        move16();
                    }
                    test();
                    test();
                    test();
                    IF (   ( msLocalMinFlag[j] != 0 )
                           && ( msNewMinFlag[j] == 0 )
                           && ( L_sub(L_shr(msCurrentMinSubWindow[j],1),Mpy_32_16_1(msCurrentMinOut[j],slope)) < FL2WORD32(0.0) )
                           && ( L_sub(msCurrentMinSubWindow[j],msCurrentMinOut[j]) > FL2WORD32(0.0) )
                       )
                    {
                        msCurrentMinOut[j] = msCurrentMinSubWindow[j];
                        move32();
                        i = j;
                        move16();
                        FOR (k=0; k<MSNUMSUBFR; k++)
                        {
                            msMinBuf[i] = msCurrentMinOut[j];
                            move32();
                            i = add(i,len);
                        }
                    }
                }

                /* Reset */
                set16_fx(msLocalMinFlag, 0, len);
                set32_fx(msCurrentMin, FL2WORD32(1.0), len);

                /* Get the current noise floor */
                Copy_Scale_sig_32_16(msCurrentMinOut, msNoiseFloor, len, -16);
            }
        }


        /* Detect sudden offsets based on the FFT bins (core bandwidth) */
        IF ( L_sub(Mpy_32_16_1(msPsdSum[0],FL2WORD16(0.02)), msPeriodogSum[0]) > FL2WORD32(0.0) )
        {
            IF ( st->offsetflag > 0 )
            {
                Copy(msPeriodog, msPsd, len);
                FOR (j=0; j < len; j++)
                {
                    msCurrentMinOut[j] = L_deposit_h(msPeriodog[j]);
                }
                set32_fx(st->msAlphaCor, FL2WORD32(1.0), cnt);
                set32_fx(msAlpha, FL2WORD32(0.0), len);
                Copy(msPeriodog, msPsdFirstMoment, len);
                set32_fx(msPsdSecondMoment, FL2WORD32(0.0), len);

                msPsdSum[0] = dotp_s_fx(msPeriodog, psize, lenFFT, CNG_HS);
                move32();
                IF ( sub(lenFFT,len) < 0 )
                {
                    msPsdSum[1] = dotp_s_fx(msPeriodog+lenFFT, psize+lenFFT, sub(len,lenFFT), CNG_HS);
                    move32();
                }
            }
            st->offsetflag = 1;
            move16();
        }
        ELSE
        {
            st->offsetflag = 0;
            move16();
        }


        /* Increment frame counter */
        IF ( sub(st->msFrCnt,MSSUBFRLEN) == 0)
        {
            st->msFrCnt = 1;
            move16();
            st->msMinBufferPtr = add(st->msMinBufferPtr,1);
            move16();
            if ( sub(st->msMinBufferPtr,MSNUMSUBFR) == 0 )
            {
                st->msMinBufferPtr = 0;
                move16();
            }
        }
        ELSE
        {
            st->msFrCnt = add(st->msFrCnt,1);
        }

        /* Smooth noise estimate during CNG phases */
        FOR (j=0; j<len; j++)
        {
            msNoiseEst[j] = round_fx(L_mac(L_mult(FL2WORD16(0.95), msNoiseEst[j]), FL2WORD16(0.05), msNoiseFloor[j]));
        }
    }

    /* Collect buffers */
    Copy(msPeriodog, msPeriodogBuf+len*(*msPeriodogBufPtr), len);

    *msPeriodogBufPtr = add(*msPeriodogBufPtr,1);
    move16();
    if ( sub(*msPeriodogBufPtr,MSBUFLEN) == 0 )
    {
        (*msPeriodogBufPtr) = 0;
        move16();
    }

    /* Upper limit the noise floors with the averaged input energy */
    FOR (j=0; j<len; j++)
    {
        scalar = L_mult(msPeriodogBuf[j],FL2WORD16(1.0/MSBUFLEN));

        FOR (i=j+len; i<MSBUFLEN*len; i+=len)
        {
            scalar = L_mac(scalar, msPeriodogBuf[i], FL2WORD16(1.0/MSBUFLEN));
        }
        scalar16 = round_fx(scalar);
        if ( sub(msNoiseEst[j],scalar16) > FL2WORD16(0.0) )
        {
            msNoiseEst[j] = scalar16;
            move16();
        }

        assert(msNoiseEst[j] >= FL2WORD16(0.0));
    }

}


/***********************************
* Apply bitrate-dependant scale    *
***********************************/
void apply_scale(Word32 *scale, Word16 bwmode, Word32 bitrate)
{
    Word16 i;
    Word16 scaleTableSize = sizeof (scaleTable) / sizeof (scaleTable[0]);



    FOR (i=0; i < scaleTableSize; i++)
    {
        cast16();
        IF ( s_and( sub(bwmode, (Word16)scaleTable[i].bwmode) == 0,
                    s_and( L_sub(bitrate,scaleTable[i].bitrateFrom) >= 0,
                           L_sub(bitrate,scaleTable[i].bitrateTo) < 0))
           )
        {
            BREAK;
        }
    }

    {
        *scale = L_add(*scale, L_deposit_h(scaleTable[i].scale));
    }

}


/***************************************
* Compute the power for each partition *
***************************************/
void bandcombinepow(Word32* bandpow,                           /* i  : Power for each band */
                    Word16  exp_bandpow,                       /* i  : exponent of bandpow */
                    Word16  nband,                             /* i  : Number of bands */
                    Word16* part,                              /* i  : Partition upper boundaries (band indices starting from 0) */
                    Word16  npart,                             /* i  : Number of partitions */
                    Word16* psize_inv,                         /* i  : Inverse partition sizes */
                    Word32* partpow,                           /* o  : Power for each partition */
                    Word16* exp_partpow)
{

    Word16 i, p;
    Word32 temp;
    Word16 smin, len, prev_part;
    Word16 facTabExp[NPART_SHAPING];



    IF (sub(nband, npart) == 0)
    {
        Copy32(bandpow, partpow, nband);
        smin = 0;
        move16();
    }
    ELSE
    {
        /* Compute the power in each partition */
        prev_part = -1;
        move16();
        FOR (p=0; p < npart; p++)
        {
            len = sub(part[p],prev_part);
            facTabExp[p] = getScaleFactor32 (&bandpow[prev_part+1], len);
            move16();
            prev_part = part[p];
            move16();
        }

        smin = WORD32_BITS-1;
        move16();
        FOR (p=0; p < npart; p++)
        {
            smin = s_min(smin,facTabExp[p]);
        }

        i = 0;
        move16();
        FOR (p = 0; p < npart; p++)
        {
            /* Arithmetic averaging of power for all bins in partition */
            temp = L_add(0,0);
            FOR ( ; i <= part[p]; i++)
            {
                temp = L_add(temp, Mpy_32_16_1(L_shl(bandpow[i],facTabExp[p]),psize_inv[p]));
            }
            partpow[p] = L_shr(temp,sub(facTabExp[p],smin));
            move32();
        }

    }

    *exp_partpow = sub(exp_bandpow,smin);
    move16();
}


/************************************
* Scale partitions (with smoothing) *
************************************/
void scalebands (Word32 *partpow,           /* i  : Power for each partition */
                 Word16 *part,              /* i  : Partition upper boundaries (band indices starting from 0) */
                 Word16  npart,             /* i  : Number of partitions */
                 Word16 *midband,           /* i  : Central band of each partition */
                 Word16  nFFTpart,          /* i  : Number of FFT partitions */
                 Word16  nband,             /* i  : Number of bands */
                 Word32 *bandpow,           /* o  : Power for each band */
                 Word16  flag_fft_en
                )
{
    Word16 i, j, s, s1, nint, delta, delta_cmp, delta_s;
    Word16 startBand, startPart, stopPart, stopPartM1;
    Word32 tmp, val, partpowLD64, partpowLD64M1;



    j = 0;
    move16();
    delta = 0;
    move16();
    partpowLD64M1 = 0L;   /* to avoid compilation warnings */

    /* Interpolate the bin/band-wise levels from the partition levels */
    IF ( sub(nband, npart) == 0 )
    {
        Copy32(partpow, bandpow, npart);
    }
    ELSE
    {
        startBand  = 0;
        move16();
        startPart  = 0;
        move16();
        stopPart   = nFFTpart;
        move16();

        WHILE ( sub(startBand,nband) < 0 )
        {
            stopPartM1 = sub(stopPart, 1);
            test();
            IF ( (flag_fft_en != 0) || (sub(startPart,nFFTpart) >= 0) )
            {
                /* first half partition */
                j = startPart;
                move16();

                FOR (i=startBand; i <= midband[j]; i++)
                {
                    bandpow[i] = partpow[j];
                    move32();
                }
                j = add(j, 1);

                /* inner partitions */
                IF (j < stopPart)
                {
                    partpowLD64M1 = BASOP_Util_Log2(partpow[j-1]);
                }

                /* Debug values to check this variable is set. */
                delta = 0x4000;
                move16();
                delta_cmp = 0x4000;
                move16();
                s1 = 1;
                move16();
                s = 1;
                move16();

                FOR ( ; j < stopPart; j++)
                {
                    nint = sub(midband[j], midband[j-1]);

                    /* log-linear interpolation */
                    partpowLD64 = BASOP_Util_Log2(partpow[j]);
                    tmp = L_sub(partpowLD64, partpowLD64M1);
                    tmp = Mpy_32_16_1(tmp, getNormReciprocalWord16(nint));

                    /* scale logarithmic value */
                    tmp = L_sub(tmp, DELTA_SHIFT_LD64);
                    delta_s = DELTA_SHIFT;
                    move16();

                    WHILE (tmp > 0)
                    {
                        tmp = L_sub(tmp,FL2WORD32(0.015625));
                        delta_s = add(delta_s,1);
                    }
                    delta_cmp = shl(1, s_max(-15, sub(WORD16_BITS-1,delta_s)));

                    tmp = BASOP_Util_InvLog2(tmp);
                    s = norm_l(tmp);
                    s1 = sub(delta_s, s);

                    delta = round_fx(L_shl(tmp, s));

                    /* Choose scale such that the interpolation start and end point both are representable and add 1 additional bit hr. */
                    delta_s = sub(s_min(norm_l(partpow[j-1]), norm_l(partpow[j])), 1);
                    val = L_shl(partpow[j-1], delta_s);
                    FOR ( ; i < midband[j]; i++)
                    {
                        val = L_shl(Mpy_32_16_1(val, delta), s1);
                        bandpow[i] = L_shr(val, delta_s);
                        move32();
                    }
                    bandpow[i++]  = partpow[j];
                    move32();
                    partpowLD64M1 = L_add(0,partpowLD64);
                }

                IF ( sub(shr(delta, s), delta_cmp) > 0 )
                {
                    delta = 0x4000;
                    move16();
                    s1 = 1;
                    move16();
                }

                /* last half partition */
                val = L_add(0,partpow[stopPartM1]);
                FOR ( ; i <= part[stopPartM1]; i++)
                {
                    val = L_shl(Mpy_32_16_1(val,delta), s1);
                    bandpow[i] = val;
                    move32();
                }

            }
            startBand = add(part[stopPartM1], 1);
            startPart = stopPart;
            move16();
            stopPart  = npart;
            move16();
        }
    }

}


/**************************************
* Get central band for each partition *
**************************************/
void getmidbands(Word16* part,              /* i  : Partition upper boundaries (band indices starting from 0) */
                 Word16  npart,             /* i  : Number of partitions */
                 Word16* midband,           /* o  : Central band of each partition */
                 Word16* psize,             /* o  : Partition sizes */
                 Word16* psize_norm,        /* o  : Partition sizes, fractional values */
                 Word16* psize_norm_exp,    /* o  : Exponent for fractional partition sizes */
                 Word16* psize_inv)         /* o  : Inverse of partition sizes */
{
    Word16   j, max_psize, shift;


    max_psize = psize[0];
    move16();
    /* first half partition */                                                        move16();
    midband[0] = part[0];
    psize[0] = add(part[0], 1);
    move16();
    psize_inv[0] = getNormReciprocalWord16(psize[0]);
    move16();
    /* inner partitions */
    FOR (j = 1; j < npart; j++)
    {
        midband[j] = shr(add(add(part[j-1], 1), part[j]), 1);
        move16();
        psize[j]   = sub(part[j], part[j-1]);
        move16();
        psize_inv[j] = getNormReciprocalWord16(psize[j]);
        move16();
        if(sub(psize[j], max_psize) > 0)
        {
            max_psize = psize[j];
            move16();
        }
    }

    shift = 9;
    move16();
    *psize_norm_exp = sub(15, shift);
    move16();
    FOR(j=0; j < npart; j++)
    {
        psize_norm[j] = shl(psize[j], shift);
        move16();
    }
    /* minimum_statistics needs fixed exponent of 6 */
    assert(norm_s(-max_psize) >= 9 );
}


/*
   AnalysisSTFT

    Parameters:

    timeDomainInput,      i  : pointer to time signal
    fftBuffer,            o  : FFT bins
    fftBufferExp,         i  : exponent of FFT bins
    st                    i/o: FD_CNG structure containing all buffers and variables

    Function:
    STFT analysis filterbank

    Returns:
    void
*/
void AnalysisSTFT (const Word16 *timeDomainInput, /* i  : pointer to time signal */
                   Word16 Q,
                   Word32 *fftBuffer,             /* o  : FFT bins */
                   Word16 *fftBuffer_exp,         /* i  : exponent of FFT bins */
                   HANDLE_FD_CNG_COM st          /* i/o: FD_CNG structure containing all buffers and variables */
                  )
{
    Word16  i, len;
    Word16  len2;
    const PWord16 *olapWin;
    Word16 *olapBuffer;



    assert( (st->fftlen>>1) == st->frameSize);

    /* pointer inititialization */
    assert(st->olapBufferAna != NULL);
    olapBuffer = st->olapBufferAna;
    olapWin = st->olapWinAna;

    /* olapWin factor is scaled with one bit */
    *fftBuffer_exp = 1;
    move16();
    len = sub(st->fftlen,st->frameSize);
    assert(len <= 320); /* see size of olapBuffer */

    /* Window the signal */
    len2 = shr(len,1);
    FOR (i=0; i < len2; i++)
    {
        move32();
        move32();
        fftBuffer[i]      = L_mult(olapBuffer[i],      mult_r(olapWin[i].v.im, FL2WORD16_SCALE(1.4142135623730950488016887242097, 1)));
        fftBuffer[i+len2] = L_mult(olapBuffer[i+len2], mult_r(olapWin[len2-1-i].v.re, FL2WORD16_SCALE(1.4142135623730950488016887242097, 1)));
    }
    len2 = shr(st->frameSize,1);
    FOR (i=0; i <len2 ; i++)
    {
        move32();
        move32();
        fftBuffer[i+len]      = L_mult(shr(timeDomainInput[i],Q),      mult_r(olapWin[i].v.re, FL2WORD16_SCALE(1.4142135623730950488016887242097, 1)));
        fftBuffer[i+len+len2] = L_mult(shr(timeDomainInput[i+len2],Q), mult_r(olapWin[len2-1-i].v.im, FL2WORD16_SCALE(1.4142135623730950488016887242097, 1)));
    }

    /* Perform FFT */
    BASOP_rfft(fftBuffer, st->fftlen, fftBuffer_exp, -1);

    FOR (i=0; i <len ; i++)
    {
        olapBuffer[i] = shr( timeDomainInput[sub(st->frameSize,len)+i], Q );
        move16();
    }

}


/*
   SynthesisSTFT

    Parameters:

    fftBuffer                 i    : pointer to FFT bins
    fftBufferExp              i    : exponent of FFT bins
    timeDomainOutput          o    : pointer to time domain signal
    timeDomainOutputExp       o    : pointer to exponent of time domain output
    olapBuffer                i/o  : pointer to overlap buffer
    olapWin                   i    : pointer to overlap window
    st                        i/o  : pointer to FD_CNG structure containing all buffers and variables

    Function:
    STFT synthesis filterbank

    Returns:
    void
*/
void
SynthesisSTFT (Word32 *fftBuffer,           /* i    : pointer to FFT bins */
               Word16  fftBufferExp,        /* i    : exponent of FFT bins */
               Word16 *timeDomainOutput,    /* o    : pointer to time domain signal */
               Word16 *olapBuffer,          /* i/o  : pointer to overlap buffer */
               const  PWord16 *olapWin,     /* i    : pointer to overlap window */
               Word16 tcx_transition,
               HANDLE_FD_CNG_COM st,       /* i/o  : pointer to FD_CNG structure containing all buffers and variables */
               Word16 gen_exc,
               Word16 *Q_new
              )
{
    Word16 i, len, scale, tmp;
    Word16  len2, len3, len4;
    Word16 buf[M+1+L_FRAME16k];


    /* Perform IFFT */
    scale = 0;
    BASOP_rfft(fftBuffer, st->fftlen, &scale, 1);
    fftBufferExp = add(fftBufferExp, scale);
    fftBufferExp = add(fftBufferExp, st->fftlenShift);

    /* Perform overlap-add */
    Copy(olapBuffer+st->frameSize, olapBuffer, st->frameSize);
    set16_fx(olapBuffer+st->frameSize, 0, st->frameSize);

    len2 = shr(st->fftlen,2);
    len4 = shr(st->fftlen,3);
    len3 = add(len2,len4);
    len = add(st->frameSize,len4);
    IF ( tcx_transition )
    {
        FOR (i=0; i < len; i++)
        {
            olapBuffer[i] = round_fx(L_shl(fftBuffer[i],fftBufferExp-15));
        }
    }
    ELSE
    {
        FOR (i=0; i < len4; i++)
        {
            olapBuffer[i+1*len4] = add(olapBuffer[i+1*len4], mult_r(round_fx(L_shl(fftBuffer[i+1*len4],fftBufferExp-15)),olapWin[i].v.im));
            move16();
            olapBuffer[i+2*len4] = add(olapBuffer[i+2*len4], mult_r(round_fx(L_shl(fftBuffer[i+2*len4],fftBufferExp-15)),olapWin[len4-1-i].v.re));
            move16();
        }
        FOR (i=len3; i < len; i++)
        {
            olapBuffer[i] = round_fx(L_shl(fftBuffer[i],fftBufferExp-15));
        }
    }

    FOR (i=0; i < len4; i++)
    {
        olapBuffer[i+5*len4] = mult_r(round_fx(L_shl(fftBuffer[i+5*len4],fftBufferExp-15)),olapWin[i].v.re);
        move16();
        olapBuffer[i+6*len4] = mult_r(round_fx(L_shl(fftBuffer[i+6*len4],fftBufferExp-15)),olapWin[len4-1-i].v.im);
        move16();
    }

    len = add( len, len2 );
    FOR (i=len; i < st->fftlen ; i++)
    {
        olapBuffer[i] = 0;
        move16();
    }

    /* Get time-domain signal */
    FOR (i=0; i < st->frameSize; i++)
    {
        timeDomainOutput[i] = mult_r( olapBuffer[i+len4], st->fftlenFac );
        move16();
    }

    /* Generate excitation */
    IF ( sub( gen_exc, 1 ) == 0 )
    {
        FOR (i=0; i < M+1+st->frameSize; i++)
        {
            buf[i] = mult_r( olapBuffer[i+len4-M-1], st->fftlenFac );
            move16();
        }
        tmp = buf[0];
        E_UTIL_f_preemph2( *Q_new-1, buf+1, PREEMPH_FAC, M+st->frameSize, &tmp );
        Residu3_fx( st->A_cng, buf+1+M, st->exc_cng, st->frameSize, 1 );
    }
    IF ( sub( gen_exc, 2 ) == 0 )
    {
        FOR (i=0; i < M+1+st->frameSize; i++)
        {
            buf[i] = mult_r( olapBuffer[i+len4-M-1], st->fftlenFac );
            move16();
        }
        tmp = buf[0];
        *Q_new = E_UTIL_f_preemph3( buf+1, PREEMPH_FAC, M+st->frameSize, &tmp, 1 );
        Residu3_fx( st->A_cng, buf+1+M, st->exc_cng, st->frameSize, 1 );
    }

}


/**************************************************************************************
* Compute some values used in the bias correction of the minimum statistics algorithm *
**************************************************************************************/
void mhvals(Word16 d,
            Word16 * m /*, float * h*/
           )
{
    Word16 i, j;
    Word16 len = sizeof(d_array)/sizeof(Word16);


    assert( d==72 || d==12); /* function only tested for d==72 and d==12) */
    i = 0;
    move16();
    FOR (i=0 ; i < len ; i++)
    {
        IF (sub(d,d_array[i]) <= 0)
        {
            BREAK;
        }
    }
    IF (sub(i, len) == 0)
    {
        i = sub(len, 1);
        j = i;
        move16();
    }
    ELSE
    {
        j = sub(i, 1);
    }
    IF (sub(d, d_array[i]) == 0)
    {
        *m = m_array[i];
        move16();
    }
    ELSE
    {
        Word32 qi_m, qj_m, q_m, tmp1_m, tmp2_m;
        Word16 qi_e, qj_e, q_e, tmp1_e, tmp2_e, tmp1_w16_m, tmp1_w16_e, shift;


        /* d_array has exponent 15 */
        qj_e    = 15;
        move16();
        qj_m    = L_deposit_h(d_array[i-1]);

        qi_e    = 15;
        move16();
        qi_m    = L_deposit_h(d_array[i]);

        q_e     = 15;
        move16();
        q_m     = L_deposit_h(d);

        qj_m = Sqrt32(qj_m, &qj_e);
        qi_m = Sqrt32(qi_m, &qi_e);
        q_m  = Sqrt32(q_m, &q_e);

        tmp1_m = Mpy_32_32(qi_m, qj_m);
        tmp1_e = add(qi_e, qj_e);
        tmp1_m = L_deposit_h(BASOP_Util_Divide3232_Scale(tmp1_m, q_m, &shift));
        tmp1_e = sub(tmp1_e, q_e);
        tmp1_e = add(tmp1_e, shift);
        tmp1_m = BASOP_Util_Add_Mant32Exp(tmp1_m, tmp1_e, L_negate(qj_m), qj_e, &tmp1_e);

        tmp2_m     = BASOP_Util_Add_Mant32Exp (qi_m, qi_e, L_negate(qj_m), qj_e, &tmp2_e);
        tmp1_w16_m = round_fx(tmp2_m);
        tmp1_w16_e = tmp2_e;
        move16();
        BASOP_Util_Divide_MantExp(sub(m_array[j], m_array[i]), 0, tmp1_w16_m, tmp1_w16_e, &tmp1_w16_m, &tmp1_w16_e);

        tmp2_m = Mpy_32_16_1(tmp1_m, tmp1_w16_m);
        tmp2_e = add(tmp1_e, tmp1_w16_e);

        tmp2_m = BASOP_Util_Add_Mant32Exp (tmp2_m, tmp2_e, L_deposit_h(m_array[i]), 0, &tmp2_e);
        assert(tmp2_e == 0);
        *m = extract_h(tmp2_m);
    }
}


/*
   rand_gauss

    Parameters:

    seed               i/o   : pointer to seed

    Function:
    Random generator with Gaussian distribution with mean 0 and std 1

    Returns:
    random signal format Q3.29
*/
Word32 rand_gauss (Word16 *seed)
{
    Word32 temp;
    Word16 loc_seed;



    /* This unrolled version reduces the cycles from 17 to 10 */
    loc_seed = extract_l(L_mac0(13849, *seed, 31821));
    temp = L_deposit_l(loc_seed);

    loc_seed = extract_l(L_mac0(13849, loc_seed, 31821));
    temp = L_msu0(temp,loc_seed,-1);

    loc_seed = extract_l(L_mac0(13849, loc_seed, 31821));
    temp = L_msu0(temp,loc_seed,-1);

    *seed = loc_seed;
    move16();
    return L_shl(temp,WORD16_BITS-CNG_RAND_GAUSS_SHIFT);
}


/*
   lpc_from_spectrum

    Parameters:

     powspec       i  : pointer to noise levels format Q5.27
     start         i  : start band
     stop          i  : stop band
     fftlen        i  : size of fft
     A             o  : lpc coefficients format Q3.12
     s      i  : lpc order
     preemph_fac   i  : preemphase factor format Q1.15


    Function:
    calculate lpc coefficients from the spectrum

    Returns:
    void
*/
void lpc_from_spectrum (Word32 *powspec,
                        Word16  powspec_exp,
                        Word16  start,
                        Word16  stop,
                        Word16  fftlen,
                        Word16 *A,
                        Word16  lpcorder,
                        Word16  preemph_fac
                       )
{
    Word16 i, s1, s2, s3, fftlen2, scale, fftlen4, fftlen8, len, step, preemph_fac2;
    Word32 maxVal, r[32], fftBuffer[FFTLEN], *ptr, *pti, nf;
    Word16 tmp, r_h[32], r_l[32];
    const PWord16 *table;



    scale = 0;
    move16();
    fftlen2 = shr(fftlen,1);
    fftlen4 = shr(fftlen,2);
    fftlen8 = shr(fftlen,3);

    /* Power Spectrum */
    maxVal = L_add(0,0);
    len = sub(stop, start);
    FOR (i=0; i < len; i++)
    {
        maxVal = L_max(maxVal, L_abs(powspec[i]));
    }
    s1 = norm_l(maxVal);
    nf = L_shr_r(FL2WORD32_SCALE(1e-3f,-9),add(sub(powspec_exp,s1),9));

    ptr = fftBuffer;
    pti = fftBuffer+1;

    FOR (i=0; i < start; i++)
    {
        *ptr = nf;
        move32();
        *pti = L_deposit_l(0);
        ptr += 2;
        pti += 2;
    }

    FOR ( ; i < stop; i++ )
    {
        *ptr = L_max( nf, L_shl(powspec[i-start], s1) );
        move32();
        *pti = L_deposit_l(0);
        ptr += 2;
        pti += 2;
    }

    FOR ( ; i < fftlen2; i++ )
    {
        *ptr = nf;
        move32();
        *pti = L_deposit_l(0);
        ptr += 2;
        pti += 2;
    }

    fftBuffer[1] = nf;
    move32();

    /* Pre-emphasis */

    BASOP_getTables(&table, NULL, &step, fftlen4);
    tmp = round_fx(L_shr(L_add(0x40000000, L_mult0(preemph_fac, preemph_fac)),1));
    preemph_fac2 = shr(preemph_fac,1);
    ptr = fftBuffer;
    *ptr = Mpy_32_16_1( *ptr, sub( tmp, preemph_fac2 ) );
    move32();
    ptr += 2;
    FOR ( i = 1; i < fftlen8; i++ )
    {
        move32();
        *ptr = Mpy_32_16_1( *ptr, sub( tmp, mult_r(preemph_fac2,add(shr(table[i-1].v.re,1),shr(table[i].v.re,1)) ) ) );
        ptr += 2;
    }
    move32();
    *ptr = Mpy_32_16_1( *ptr, sub( tmp, mult_r(preemph_fac2,add(shr(table[fftlen8-1].v.re,1),shr(table[fftlen8-1].v.im,1)) ) ) );
    ptr += 2;
    FOR ( i = 1; i < fftlen8; i++ )
    {
        move32();
        *ptr = Mpy_32_16_1( *ptr, sub( tmp, mult_r(preemph_fac2,add(shr(table[fftlen8-i-1].v.im,1),shr(table[fftlen8-i].v.im,1)) ) ) );
        ptr += 2;
    }
    move32();
    *ptr = Mpy_32_16_1( *ptr, tmp );
    ptr += 2;
    FOR ( i = 1; i < fftlen8; i++ )
    {
        move32();
        *ptr = Mpy_32_16_1( *ptr, add( tmp, mult_r(preemph_fac2,add(shr(table[i-1].v.im,1),shr(table[i].v.im,1)) ) ) );
        ptr += 2;
    }
    move32();
    *ptr = Mpy_32_16_1( *ptr, add( tmp, mult_r(preemph_fac2,add(shr(table[fftlen8-1].v.re,1),shr(table[fftlen8-1].v.im,1)) ) ) );
    ptr += 2;
    FOR ( i = 1; i < fftlen8; i++ )
    {
        move32();
        *ptr = Mpy_32_16_1( *ptr, add( tmp, mult_r(preemph_fac2,add(shr(table[fftlen8-i-1].v.re,1),shr(table[fftlen8-i].v.re,1)) ) ) );
        ptr += 2;
    }
    move32();
    fftBuffer[1] = Mpy_32_16_1( fftBuffer[1], add( tmp, preemph_fac2 ) );
    maxVal = L_add(0,0);
    FOR (i=0; i < fftlen; i++)
    {
        maxVal = L_max(maxVal, L_abs(fftBuffer[i]));
    }
    s2 = norm_l(maxVal);
    FOR (i=0; i < fftlen; i++)
    {
        fftBuffer[i] = L_shl( fftBuffer[i], s2 );
        move32();
    }

    /* Autocorrelation */

    BASOP_rfft(fftBuffer, fftlen, &scale, 1);

    s3 = getScaleFactor32(fftBuffer, add(lpcorder,1));

    FOR (i=0; i <= lpcorder; i++ )
    {
        r[i] = L_shl(fftBuffer[i], s3);
        move32();
    }

    r[0] = Mpy_32_32( r[0], FL2WORD32_SCALE(1.0005f,1) );
    move32();
    FOR (i=1; i <= lpcorder; i++ )
    {
        r[i] = Mpy_32_32( r[i], FL2WORD32_SCALE(1.f,1) );
        move32();
    }
    s3 = getScaleFactor32(r, add(lpcorder,1));

    FOR (i=0; i <= lpcorder; i++ )
    {
        r[i] = L_shl(r[i], s3);
        move32();
    }

    FOR (i=0; i <= lpcorder; i++ )
    {
        L_Extract(r[i], &r_h[i], &r_l[i]);
    }

    /* LPC */

    E_LPC_lev_dur(r_h, r_l, A, NULL, lpcorder, NULL);

}

/*
   msvq_decoder

    Parameters:

    cb               i  : Codebook (indexed cb[stages][levels][p]) format Q9.7
    stages           i  : Number of stages
    N                i  : Vector dimension
    maxN             i  : Codebook vector dimension
    Idx              o  : Indices
    uq[]             i  : Quantized vector format Q9.7


    Function:
    multi stage vector dequantisation

    Returns:
    void
*/
void msvq_decoder (const Word16 *const cb[],       /* i  : Codebook (indexed cb[*stages][levels][p]) */
                   Word16 stages,     /* i  : Number of stages                          */
                   Word16 N,          /* i  : Vector dimension                          */
                   Word16 maxN,       /* i  : Codebook vector dimension                 */
                   Word16 Idx[],      /* i  : Indices                                   */
                   Word16 *uq         /* o  : quantized vector                          */
                  )
{
    Word16 s, i, offset;



    offset = i_mult(Idx[0], maxN);
    FOR (i=0; i<N; i++)
    {
        uq[i] = cb[0][offset+i];
        move16();
    }

    FOR (s=1; s<stages; s++)
    {
        offset = i_mult(Idx[s], maxN);

        FOR (i=0; i<N; i++)
        {
            uq[i] = add(uq[i],cb[s][offset+i]);
            move16();
        }
    }

}

void FdCng_exc(
    HANDLE_FD_CNG_COM hs,
    Word16 *CNG_mode,
    Word16 L_frame,
    Word16 *lsp_old,
    Word16 first_CNG,
    Word16 *lspCNG,
    Word16 *Aq,                    /* o:   LPC coeffs */
    Word16 *lsp_new,               /* o:   lsp  */
    Word16 *lsf_new,               /* o:   lsf  */
    Word16 *exc,                   /* o:   LP excitation   */
    Word16 *exc2,                  /* o:   LP excitation   */
    Word16 *bwe_exc                /* o:   LP excitation for BWE */
)
{
    Word16 i;

    *CNG_mode = -1;

    FOR(i=0; i<L_frame/L_SUBFR; i++)
    {
        Copy( hs->A_cng, Aq+i*(M+1), M+1 );
    }

    E_LPC_a_lsp_conversion( Aq, lsp_new, lsp_old, M );

    IF( first_CNG == 0 )
    {
        Copy( lsp_old, lspCNG, M );
    }
    FOR( i=0; i<M; i++ )
    {
        /* AR low-pass filter  */
        lspCNG[i] = mac_r(L_mult(CNG_ISF_FACT_FX,lspCNG[i]),32768-CNG_ISF_FACT_FX,lsp_new[i]);
        move16(); /* Q15 (15+15+1-16) */
    }

    IF(sub(L_frame, L_FRAME16k)== 0)
    {
        lsp2lsf_fx( lsp_new, lsf_new, M, INT_FS_16k_FX );
    }
    ELSE
    {
        E_LPC_lsp_lsf_conversion( lsp_new, lsf_new, M );
    }
    Copy( hs->exc_cng, exc, L_frame );
    Copy( hs->exc_cng, exc2, L_frame );

    IF( sub(L_frame,L_FRAME) == 0 )
    {
        interp_code_5over2_fx( exc2, bwe_exc, L_frame );
    }
    ELSE
    {
        interp_code_4over2_fx( exc2, bwe_exc, L_frame );
    }
}

