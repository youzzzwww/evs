/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! \file jbm_jb4sb.c Jitter Buffer Management Interface */

/* system headers */
#include <assert.h>
#include <stdlib.h>
/* instrumentation headers */
#include "stl.h"
#include "options.h"
#include "basop_util.h"
#include "basop_util_jbm.h"
#include "cnst_fx.h"
/* local headers */
#include "jbm_jb4_circularbuffer.h"
#include "jbm_jb4_inputbuffer.h"
#include "jbm_jb4_jmf.h"
#include "jbm_jb4sb.h"
#include "prot_fx.h"


static Word16 idiv3216(Word32 x, Word16 y)
{
    Word16 z, divScaleFac;

    z = BASOP_Util_Divide3216_Scale(x, y, &divScaleFac);
    z = shl(z, add(divScaleFac,1));
    return z;
}

#define MAXOFFSET 10

/*! Calculates the difference between two RTP timestamps - the diff is positive, if B 'later', negative otherwise */
static Word32 JB4_rtpTimeStampDiff( Word32 tsA, Word32 tsB );
/* function to calculate different options for the target playout delay */
static void JB4_targetPlayoutDelay( const JB4_HANDLE h, Word32 *targetMin,
                                    Word32 *targetMax, Word32 *targetDtx, Word32 *targetStartUp );
/*! function to do playout adaptation before playing the next data unit */
/*! In case of time shrinking, data units will be dropped before the next data unit to play is returned and
 *  in case of time stretching a empty data unit is returned and the frame should be concealed.
 *  @param[in] now current system time
 *  @param[out] dataUnit the next data unit to play
 *  @param[out] scale the scale in percent used as target for time scaling of the returned data unit
 *  @param[out] maxScaling the maximum allowed external time scaling */
static Word16 JB4_adaptPlayout( JB4_HANDLE h, Word32 sysTime, Word32 extBufferedTime,
                                JB4_DATAUNIT_HANDLE *pDataUnit, Word16 *scale, Word16 *maxScaling );
/*! function to do playout adaptation before playing the first data unit */
/*! @param[in] now current system time
 *  @param[out] prebuffer true, if the data unit should be prebuffered */
static void JB4_adaptFirstPlayout( JB4_HANDLE h, Word32 sysTime, Word16 *prebuffer );
/*! function for playout adaptation while active (no DTX) */
static void JB4_adaptActivePlayout( JB4_HANDLE h, Word32 sysTime, Word32 extBufferedTime,
                                    Word16 *scale, Word16 *maxScaling );
/*! function for playout adaptation while DTX */
static void JB4_adaptDtxPlayout( JB4_HANDLE h, Word32 sysTime, Word16 *stretchTime );
/*! function to look into the buffer and check if it makes sense to drop a data unit */
/*! @param[out] dropEarly true, if a data unit could be dropped early
 *  @param[out] buffered the buffered time span in timeScale units
 *  @return true, if a data unit could be dropped */
static Word16 JB4_inspectBufferForDropping( const JB4_HANDLE h, Word16 *dropEarly, Word32 *buffered );
/* function to look into the buffer and check if it makes sense to drop a data unit during DTX */
static Word16 JB4_checkDtxDropping( const JB4_HANDLE h );
/*! function to estimate the short term jitter  */
static void JB4_estimateShortTermJitter( JB4_HANDLE h, Word32 rcvTime, Word32 rtpTimeStamp );
/*! function to pop a data unit from the buffer */
static void JB4_popFromBuffer( JB4_HANDLE h, Word32 sysTime, JB4_DATAUNIT_HANDLE *pDataUnit );
/*! function to drop a data unit from the buffer - updates nShrinked */
static void JB4_dropFromBuffer( JB4_HANDLE h, Word32 sysTime );
/*! function to calculate the playout delay based on the current jitter */
/*! @param[in] playTime the system time when the data unit will be played
 *  @param[in] timeStamp the time stamp of the data unit to played
 *  @param[out] delay the calculated playout delay */
static Word16 JB4_playoutDelay( const JB4_HANDLE h, Word32 playTime,
                                Word32 rtpTimeStamp, Word32 *delay );
/*! function to update lastPlayoutDelay and lastTargetTime after popFromBuffer() */
static void JB4_updateLastTimingMembers( JB4_HANDLE h, Word32 playTime, Word32 rtpTimeStamp );
/*! function to compare the RTP time stamps of two data units: newElement==arrayElement ? 0 : (newElement>arrayElement ? +1 : -1) */
static Word32 JB4_inputBufferCompareFunction( const JB4_INPUTBUFFER_ELEMENT newElement,
        const JB4_INPUTBUFFER_ELEMENT arrayElement, Word16 *replaceWithNewElementIfEqual );


/*! Jitter Buffer Management Interface */
struct JB4
{
    /*! @name statistics for user */
    /*@{ */
    /*! the number of late lost data units */
    Word32                     nLateLost;
    /*! the number of data units that were available (not NULL) at playout time */
    Word32                     nAvailablePopped;
    /*! the number of data units that were not available (NULL) at playout time */
    Word32                     nUnavailablePopped;
    /*! the number of unavailable pops since the last available one - used as temp value for nLost and nStretched */
    Word32                     nLostOrStretched;
    /*! the number of data units that were lost at playout time */
    Word32                     nLost;
    /*! the number of empty data units inserted for playout adaptation */
    Word32                     nStretched;
    /*! the number of data units dropped for playout adaptation */
    /*! This function counts all time shrinking events, no matter if a dropped data unit was actually available. */
    Word32                     nShrinked;
    /*! the number of data units that were returned to create comfort noice (including NULL) */
    Word32                     nComfortNoice;
    /*! the number of jitter induced concealment operations (as defined in 3GPP TS 26.114) */
    Word32                     jitterInducedConcealments;
    /*! the target playout delay of the last returned data unit */
    Word32                     targetPlayoutDelay;
    /*! the target playout time of the last returned data unit */
    Word32                     lastTargetTime;
    /*@} */
    /*! @name internal configuration values - do not change!!! */
    /*@{ */
    /*! internal time scale for all calculations */
    Word16                     timeScale;
    /*! internal frame duration in timeScale units */
    Word32                     frameDuration;
    /*@} */
    /*! @name jitter buffer configuration values */
    /*@{ */
    /*! the allowed delay reserve in addition to network jitter to reduce late-loss [milliseconds] */
    Word32                     safetyMargin;
    /*@} */
    /*! @name data for short term jitter estimation */
    /*@{ */
    /*! short term jitter measure FIFO */
    JB4_JMF_HANDLE             stJmf;
    /*! FIFO of short term jitter values */
    JB4_CIRCULARBUFFER_HANDLE  stJitterFifo;
    /*! FIFO of RTP time stamps for the values stored in stJitterFifo */
    JB4_CIRCULARBUFFER_HANDLE  stTimeStampFifo;
    /*! short term jitter */
    Word32                     stJitter;
    /*@} */
    /*! @name jitter buffer data */
    /*@{ */
    /*! true, if a data unit was already popped from the buffer */
    Word16                     firstDataUnitPopped;
    /*! system time of the previous JB4_PopDataUnit() call */
    Word32                     prevPopSysTime;
    /*! RTP timestamp of the last played/dropped data unit that was actually available */
    Word32                     lastReturnedTs;
    /*! true, if the last popped data unit contained no active signal, i.e. silence -> hint for DTX */
    Word16                     lastPoppedWasSilence;
    /*! the playout time minus the minimum offset of the last played data unit in microseconds */
    Word32                     lastPlayoutOffset;
    /*! RTP time stamp of the next data unit that is expected to be fetched from the buffer */
    Word32                     nextExpectedTs;
    Word16                     rfOffset2Active;
    Word16                     rfOffset3Active;
    Word16                     rfOffset5Active;
    Word16                     rfOffset7Active;
    Word32                     rfDelay;
    /*! long term jitter measure FIFO */
    JB4_JMF_HANDLE             ltJmf;
    /*@} */
    /*! @name members to store the data units */
    /*@{ */
    /*! the data unit buffer */
    JB4_INPUTBUFFER_HANDLE     inputBuffer;
    Word16                     pre_partial_frame;

    Word32  FecOffWinLen;
    Word32  FecOffWin[10];
    Word32  optimum_offset;
    Word32  totWin;
    Word32  netLossRate;
    /*! the number of partial copies decoded instead of PLC */
    Word32  nPartialCopiesUsed;
    Word32 last_nLost;
    Word32 last_ntot;


    /*@} */
}; /* JB4 */


Word16 JB4_Create( JB4_HANDLE *ph )
{
    JB4_HANDLE h = calloc( 1, sizeof( struct JB4 ) );

    Word16 iter;

    /* statistics for user */
    h->nLateLost                 = L_deposit_l(0);
    h->nAvailablePopped          = L_deposit_l(0);
    h->nUnavailablePopped        = L_deposit_l(0);
    h->nLostOrStretched          = L_deposit_l(0);
    h->nLost                     = L_deposit_l(0);
    h->nStretched                = L_deposit_l(0);
    h->nShrinked                 = L_deposit_l(0);
    h->nComfortNoice             = L_deposit_l(0);
    h->jitterInducedConcealments = L_deposit_l(0);
    h->targetPlayoutDelay        = L_deposit_l(0);
    h->lastTargetTime            = L_deposit_l(0);
    /* internal configuration values - do not change!!! */
    h->timeScale                 = 0;
    move16();
    h->frameDuration             = L_deposit_l(0);
    /* jitter buffer configuration values: done in JB4_Init() */
    /* short term jitter evaluation */
    JB4_JMF_Create( &h->stJmf );
    JB4_CIRCULARBUFFER_Create( &h->stJitterFifo );
    JB4_CIRCULARBUFFER_Create( &h->stTimeStampFifo );
    h->stJitter                  = L_deposit_l(0);
    /* jitter buffer data */
    h->firstDataUnitPopped       = false;
    move16();
    h->prevPopSysTime            = L_deposit_l(0);
    h->lastReturnedTs            = L_deposit_l(0);
    h->lastPoppedWasSilence      = false;
    move16();
    h->lastPlayoutOffset         = L_deposit_l(0);
    h->nextExpectedTs            = L_deposit_l(0);
    h->rfOffset2Active           = 0;
    move16();
    h->rfOffset3Active           = 0;
    move16();
    h->rfOffset5Active           = 0;
    move16();
    h->rfOffset7Active           = 0;
    move16();
    h->rfDelay                   = L_deposit_l(0);
    JB4_JMF_Create( &h->ltJmf );
    h->pre_partial_frame         = 0;

    h->FecOffWinLen               = 0;
    move32();
    FOR (iter = 0; iter < 10; iter++ )
    {
        h->FecOffWin[iter] = 0;
        move32();
    }
    h->optimum_offset             = 3;
    move32();
    h->totWin                     = 0;
    move32();
    h->netLossRate                = 0;
    move32();
    h->nPartialCopiesUsed         = 0;
    move32();
    h->last_nLost                 = 0;
    move32();
    h->last_ntot                  = 0;
    move32();
    /* members to store the data units */
    JB4_INPUTBUFFER_Create( &h->inputBuffer );
    *ph = h;
    move16();
    return 0;
}

void JB4_Destroy( JB4_HANDLE *ph )
{
    JB4_HANDLE h;

    IF( !ph )
    {
        return;
    }
    h = *ph;
    move16();

    IF( !h )
    {
        return;
    }

    JB4_JMF_Destroy( &h->stJmf );
    JB4_CIRCULARBUFFER_Destroy( &h->stJitterFifo );
    JB4_CIRCULARBUFFER_Destroy( &h->stTimeStampFifo );
    JB4_JMF_Destroy( &h->ltJmf );
    JB4_INPUTBUFFER_Destroy( &h->inputBuffer );

    free( h );
    *ph = NULL;
    move16();
}

Word16 JB4_Init( JB4_HANDLE h, Word16 safetyMargin )
{
    Word16 ltJmfSize, stFifoSize, stJmfSize, stJmfAllowedLateLoss;
    Word16 inputBufferCapacity;
    Word16 maxTimeToBuffer;

    /* internal timescale is 1000, frame duration is 20ms */
    h->timeScale            = 1000; /* ms */                                                          move16();
    h->frameDuration        = L_deposit_l(20);   /* ms */

    /* jitter buffer configuration values */
    h->safetyMargin         = L_deposit_l(safetyMargin);
    /* the upper limit of the buffer size in milliseconds */
    maxTimeToBuffer         = 3000;
    move16();

    /* long term jitter measure FIFO: 500 frames and 10s */
    ltJmfSize = 10000;
    move16();
    JB4_JMF_Init( h->ltJmf, h->timeScale, ltJmfSize / 20, ltJmfSize, 1000 );
    /* short term jitter evaluation */
    stFifoSize           = 200;
    move16();
    stJmfSize            = 50;
    move16();
    stJmfAllowedLateLoss = 940; /* (1000 - 60) = 6%, e.g. ignore three packets out of 50 */           move16();
    JB4_CIRCULARBUFFER_Init( h->stJitterFifo, stFifoSize );
    JB4_CIRCULARBUFFER_Init( h->stTimeStampFifo, stFifoSize );
    JB4_JMF_Init( h->stJmf, h->timeScale, stJmfSize, h->timeScale /* 1s */, stJmfAllowedLateLoss );

    /* 50 packets per second plus some reserve */
    inputBufferCapacity = add( idiv1616U( maxTimeToBuffer, extract_l( h->frameDuration ) ), 2 );
    assert( inputBufferCapacity == 152 );
    JB4_INPUTBUFFER_Init( h->inputBuffer, inputBufferCapacity, JB4_inputBufferCompareFunction );
    return 0;
}

Word16 JB4_PushDataUnit( JB4_HANDLE h, JB4_DATAUNIT_HANDLE dataUnit, Word32 rcvTime )
{
    assert( dataUnit->duration  == h->frameDuration );
    assert( dataUnit->timeScale == h->timeScale );

    /* do statistics on partial copy offset using active primary copies to
     * avoid unexpected resets because RF_NO_DATA partial copies are dropped before JBM */
    IF(dataUnit->silenceIndicator == 0 && dataUnit->partial_frame == 0)
    {
        IF(sub(dataUnit->partialCopyOffset, 0) == 0)
        {
            h->rfOffset2Active = s_max(sub(h->rfOffset2Active, 1), 0);
            h->rfOffset3Active = s_max(sub(h->rfOffset3Active, 1), 0);
            h->rfOffset5Active = s_max(sub(h->rfOffset5Active, 1), 0);
            h->rfOffset7Active = s_max(sub(h->rfOffset7Active, 1), 0);
        }
        ELSE IF(sub(dataUnit->partialCopyOffset, 2) == 0)
        {
            h->rfOffset2Active = 100;
            move16();
            h->rfOffset3Active = 0;
            move16();
            h->rfOffset5Active = 0;
            move16();
            h->rfOffset7Active = 0;
            move16();
        }
        ELSE IF(sub(dataUnit->partialCopyOffset, 3) == 0)
        {
            h->rfOffset2Active = 0;
            move16();
            h->rfOffset3Active = 100;
            move16();
            h->rfOffset5Active = 0;
            move16();
            h->rfOffset7Active = 0;
            move16();
        }
        ELSE IF(sub(dataUnit->partialCopyOffset, 5) == 0)
        {
            h->rfOffset2Active = 0;
            move16();
            h->rfOffset3Active = 0;
            move16();
            h->rfOffset5Active = 100;
            move16();
            h->rfOffset7Active = 0;
            move16();
        }
        ELSE IF(sub(dataUnit->partialCopyOffset, 7) == 0)
        {
            h->rfOffset2Active = 0;
            move16();
            h->rfOffset3Active = 0;
            move16();
            h->rfOffset5Active = 0;
            move16();
            h->rfOffset7Active = 100;
            move16();
        }
    }

    IF(dataUnit->partial_frame != 0)
    {
        /* check for "real" late loss: a frame with higher/same timestamp was already returned to be fed into decoder */    test();
        IF( h->firstDataUnitPopped && JB4_rtpTimeStampDiff( h->lastReturnedTs, dataUnit->timeStamp ) <= 0 )
        {
            return 0;
        }

        /* drop partial copy if the missing frame was already concealed */
        IF( h->firstDataUnitPopped )
        {
            IF( sub(dataUnit->partialCopyOffset, 3) <= 0 && JB4_rtpTimeStampDiff( h->nextExpectedTs, dataUnit->timeStamp ) < 0)
            {
                return 0;
            }
            ELSE IF( sub(dataUnit->partialCopyOffset, 5) == 0 && L_add(JB4_rtpTimeStampDiff( h->nextExpectedTs, dataUnit->timeStamp ), 40) < 0)
            {
                return 0;
            }
            ELSE IF( sub(dataUnit->partialCopyOffset, 7) == 0 && L_add(JB4_rtpTimeStampDiff( h->nextExpectedTs, dataUnit->timeStamp ), 80) < 0)
            {
                return 0;
            }
        }

        /* try to store partial copy - will be dropped if primary copy already available */
        IF(JB4_INPUTBUFFER_Enque( h->inputBuffer, dataUnit ) == 0)
        {
            /* partial copy is useful, consider it in long-term jitter estimation */
            IF( sub(dataUnit->partialCopyOffset, 3) <= 0 )
            {
                JB4_JMF_PushPacket( h->ltJmf, rcvTime, dataUnit->timeStamp );
            }
        }
    }
    ELSE
    {
        /* calculate jitter */
        JB4_JMF_PushPacket( h->ltJmf, rcvTime, dataUnit->timeStamp );
        JB4_estimateShortTermJitter( h, rcvTime, dataUnit->timeStamp );
        /* check for "real" late loss: a frame with higher/same timestamp was already returned to be fed into decoder */    test();
        IF( h->firstDataUnitPopped && JB4_rtpTimeStampDiff( h->lastReturnedTs, dataUnit->timeStamp ) <= 0 )
        {
            IF( !dataUnit->silenceIndicator )
            {
                h->nLateLost = L_add(h->nLateLost, 1);
                /* deletion of a speech frame because it arrived at the JBM too late */
                h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
            }
            return 0;
        }
        /* store data unit */
        JB4_INPUTBUFFER_Enque( h->inputBuffer, dataUnit );
    }
    return 0;
}

Word16 JB4_PopDataUnit( JB4_HANDLE h, Word32 sysTime, Word32 extBufferedTime,
                        JB4_DATAUNIT_HANDLE *pDataUnit, Word16 *scale, Word16 *maxScaling )
{
    Word16 ret;

    assert( sysTime >= h->prevPopSysTime );
    if( L_sub(sysTime, L_add(h->prevPopSysTime, h->frameDuration)) > 0 )
    {
        h->lastPlayoutOffset = rtpTs_add(h->lastPlayoutOffset, h->frameDuration);
    }
    h->prevPopSysTime = sysTime;
    move32();

    ret = JB4_adaptPlayout( h, sysTime, extBufferedTime, pDataUnit, scale, maxScaling );
    return ret;
}

/* Calculates the difference between two RTP timestamps - the diff is positive, if B 'later', negative otherwise */
static Word32 JB4_rtpTimeStampDiff( Word32 tsA, Word32 tsB )
{
    Word32 ret;
    /* no saturation wanted! */
    ret = rtpTs_sub(tsB, tsA);
    assert( ret == (Word32)(tsB - tsA) );
    return ret;
}

/* function to get the number of data units contained in the buffer */
Word16 JB4_bufferedDataUnits( const JB4_HANDLE h )
{
    return JB4_INPUTBUFFER_Size( h->inputBuffer );
}


Word16 JB4_getFECoffset(JB4_HANDLE h)
{
    return (Word16)h->optimum_offset;
}

Word16 JB4_FECoffset(JB4_HANDLE h)
{
    IF ( L_sub( h->netLossRate,  1634) < 0 )
    {
        return (Word16)0;
    }
    ELSE
    {
        return (Word16)1;
    }
}


/*****************************************************************************
 **************************** private functions ******************************
 *****************************************************************************/


/* function to calculate different options for the target playout delay */
static void JB4_targetPlayoutDelay( const JB4_HANDLE h, Word32 *targetMin,
                                    Word32 *targetMax, Word32 *targetDtx, Word32 *targetStartUp )
{
    Word32 ltJitter, extraDelayReserve;
    /* adapt target delay to partial copy offset */
    extraDelayReserve = L_deposit_l(0);
    h->rfDelay = L_deposit_l(0);
    IF(h->rfOffset7Active != 0)
    {
        h->rfDelay = L_deposit_l(140);
    }
    ELSE IF(h->rfOffset5Active != 0)
    {
        h->rfDelay = L_deposit_l(100);
    }
    ELSE IF(h->rfOffset2Active == 0 && h->rfOffset3Active == 0)
    {
        /* keep some delay reserve for RF-off */
        extraDelayReserve = L_deposit_l(15);
    }

    /* get estimated long term jitter */
    IF( JB4_JMF_Jitter( h->ltJmf, &ltJitter ) == 0 )
    {
        /* combine long term and short term jitter to calculate target delay values */
        *targetMax     = L_add(h->stJitter, L_add(h->safetyMargin, h->rfDelay));
        *targetMin     = L_min(L_add(L_add(L_add(ltJitter, 20), h->rfDelay), extraDelayReserve), *targetMax );
        *targetDtx     = L_min(L_add(ltJitter, extraDelayReserve), h->stJitter );
        *targetStartUp = L_shr(L_add(L_add(*targetMin, *targetMax), L_shr(extraDelayReserve, 2)), 1);
    }
    ELSE
    {
        /* combine long term and short term jitter to calculate target delay values */
        *targetMax     = h->safetyMargin;
        move32();
        *targetMin     = L_min( 20, *targetMax );
        *targetDtx     = 0;
        move16();
        *targetStartUp = L_shr(L_add(*targetMin, *targetMax), 1);
    }
}

/* function to do playout adaptation before playing the next data unit */
static Word16 JB4_adaptPlayout( JB4_HANDLE h, Word32 sysTime, Word32 extBufferedTime,
                                JB4_DATAUNIT_HANDLE *pDataUnit, Word16 *scale, Word16 *maxScaling )
{
    Word16 stretchTime;

    /* reset scale */                                                                                 test();
    IF( scale == NULL || maxScaling == NULL )
    {
        return -1;
    }
    *scale = 100;
    move16();
    *maxScaling = 0;
    move16();
    stretchTime = false;
    move16();

    /* switch type of current playout (first one, active, DTX) */
    IF( !h->firstDataUnitPopped )
    {
        JB4_adaptFirstPlayout( h, sysTime, &stretchTime );
    }
    ELSE IF( h->lastPoppedWasSilence )
    {
        JB4_adaptDtxPlayout( h, sysTime, &stretchTime );
    }
    ELSE
    {
        JB4_adaptActivePlayout( h, sysTime, extBufferedTime, scale, maxScaling );
    }

    /* time shrinking done if needed, now do time stretching or pop data unit to play */
    IF( stretchTime )
    {
        /* return empty data unit */
        *pDataUnit = NULL;
        move16();

        IF( h->firstDataUnitPopped )
        {
            h->nUnavailablePopped = L_add(h->nUnavailablePopped, 1);
            IF( !h->lastPoppedWasSilence )
            {
                h->nStretched = L_add(h->nStretched, 1);
                /* jitter-induced insertion (e.g. buffer underflow) */
                h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
            }
        }
        /* add one frame to last playout delay */
        h->lastPlayoutOffset = rtpTs_add(h->lastPlayoutOffset, h->frameDuration);
    }
    ELSE
    {
        /* return next data unit from buffer */
        JB4_popFromBuffer( h, sysTime, pDataUnit );
    }
    return 0;
}

/* function for playout adaptation while active (no DTX) */
static void JB4_adaptActivePlayout( JB4_HANDLE h, Word32 sysTime, Word32 extBufferedTime,
                                    Word16 *scale, Word16 *maxScaling )
{
    JB4_DATAUNIT_HANDLE nextDataUnit;
    Word16 convertToLateLoss, dropEarly = 0;
    Word32 targetMin, targetMax, targetDtx, targetStartUp, targetMaxStretch;
    Word32 currPlayoutDelay, buffered;
    Word16 gap, rate, dropGapMax, dropRateMin, dropRateMax;
    Word32 minOffTicks, tsDiffToNextDataUnit;
    Word32 delayWithClearedExternalBuffer;
    Word32 tmp32;

    JB4_targetPlayoutDelay( h, &targetMin, &targetMax, &targetDtx, &targetStartUp );
    IF( JB4_JMF_MinOffset( h->ltJmf, &minOffTicks ) != 0 )
    {
        return;
    }
    h->targetPlayoutDelay = L_shr(L_add(targetMin, targetMax ), 1);

    convertToLateLoss = false;
    move16();
    dropEarly         = false;
    move16();
    dropGapMax        = 200;
    move16();
    dropRateMin       = 5;
    move16();
    dropRateMax       = 200; /* 20% */                                                                move16();
    /* calculate current playout delay */
    currPlayoutDelay = rtpTs_add(rtpTs_sub(h->lastPlayoutOffset, minOffTicks), extBufferedTime);
    /* adapt it to time stretching due to empty buffer */
    IF( !JB4_INPUTBUFFER_IsEmpty( h->inputBuffer ) )
    {
        nextDataUnit = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );
        tsDiffToNextDataUnit = JB4_rtpTimeStampDiff( h->nextExpectedTs, nextDataUnit->timeStamp );
        IF( tsDiffToNextDataUnit < 0 )
        {
            convertToLateLoss = true;
            move16();
            {
                /* time stretching is expected -> increase playout delay to allow dropping the late frame */
                currPlayoutDelay = L_sub(currPlayoutDelay, tsDiffToNextDataUnit);
                currPlayoutDelay = L_add(currPlayoutDelay, 1);
            }
        }
    }

    /*  decided between shrinking/stretching */
    IF( L_sub(currPlayoutDelay, targetMax) > 0 )   /* time shrinking */
    {
        gap = extract_l(L_sub(currPlayoutDelay, h->targetPlayoutDelay));
        /* check if gap is positive and dropping is allowed
         * and buffer contains enough time (ignoring one frame) */                                      test();
        test();
        test();
        IF( gap > 0 &&
            JB4_inspectBufferForDropping( h, &dropEarly, &buffered ) == 0 &&
            ( convertToLateLoss ||
              L_sub(L_add(L_add(buffered, h->frameDuration), extBufferedTime), targetMax) > 0 ) )
        {
            IF( convertToLateLoss )
            {
                JB4_dropFromBuffer( h, sysTime );
            }
            ELSE IF( dropEarly )
            {
                JB4_dropFromBuffer( h, sysTime );
            }
            ELSE
            {
                /* limit gap to [gapMin,gapMax] and calculate current drop rate from gap */
                tmp32 = L_mult0(s_min(gap, dropGapMax), sub(dropRateMax, dropRateMin));
                assert( tmp32 == s_min(gap, dropGapMax) * ( dropRateMax - dropRateMin ));
                rate = idiv3216(tmp32, dropGapMax);
                assert( rate == tmp32 / dropGapMax );
                rate = add(rate, dropRateMin);
                *scale = idiv1616U(sub(1000, rate), 10);
                assert( *scale == (1000 - rate) / 10 );
                *maxScaling = extract_l(L_sub(currPlayoutDelay, targetMax));
            }
        }
    }
    ELSE   /* time stretching */
    {
        /* Stretching only makes sense if we win one additional frame in the input buffer.
         * If too much additional delay would be required to do so, then do not scale.
         * Also make sure that the delay doesn't increase too much. */
        delayWithClearedExternalBuffer = L_add(L_sub(currPlayoutDelay, extBufferedTime), h->frameDuration);
        targetMaxStretch = L_sub(targetMax, h->frameDuration);
        IF( L_sub(L_add(delayWithClearedExternalBuffer, h->frameDuration), targetMaxStretch) <= 0 &&
        L_sub(currPlayoutDelay, targetMaxStretch) < 0 && L_sub(currPlayoutDelay, L_add(110, L_shr(h->rfDelay, 2))) < 0)
        {
            *scale = 120;
            move16();
            *maxScaling = extract_l(L_sub(targetMaxStretch, currPlayoutDelay));
        }
    }
}

/* function for playout adaptation while DTX */
static void JB4_adaptDtxPlayout( JB4_HANDLE h, Word32 sysTime, Word16 *stretchTime )
{
    JB4_DATAUNIT_HANDLE firstDu;
    Word32 targetMin, targetMax, targetDtx, targetStartUp;
    Word32 currPlayoutDelay, headRoom;
    Word32 minOffTicks, tsDiffToNextDataUnit;


    JB4_targetPlayoutDelay( h, &targetMin, &targetMax, &targetDtx, &targetStartUp );
    IF( JB4_JMF_MinOffset( h->ltJmf, &minOffTicks ) != 0 )
    {
        return;
    }

    /* calculate current playout delay */
    assert( rtpTs_sub(h->lastPlayoutOffset, minOffTicks) == h->lastPlayoutOffset - minOffTicks );
    currPlayoutDelay = rtpTs_sub(h->lastPlayoutOffset, minOffTicks);

    /* check for startup after DTX */
    IF( !JB4_INPUTBUFFER_IsEmpty( h->inputBuffer ) )
    {
        firstDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );

        tsDiffToNextDataUnit = JB4_rtpTimeStampDiff( h->nextExpectedTs, firstDu->timeStamp );
        /* check if the next available data unit should already be used (time stamp order) */
        if( tsDiffToNextDataUnit > 0 )
        {
            /* time stretching is expected -> increase playout delay */
            currPlayoutDelay = L_add(currPlayoutDelay, tsDiffToNextDataUnit);
        }

        IF( !firstDu->silenceIndicator )
        {
            /* recalculate playout delay based on first buffered data unit */
            JB4_playoutDelay( h, sysTime, firstDu->timeStamp, &currPlayoutDelay );
            /* check if the next available data unit should already be used (time stamp order) */
            if( tsDiffToNextDataUnit > 0 )
            {
                /* time stretching is expected -> increase playout delay */
                currPlayoutDelay = L_add(currPlayoutDelay, tsDiffToNextDataUnit);
            }
            h->targetPlayoutDelay = targetStartUp;
            move32();
            headRoom = L_deposit_l(12); /* 600 * 20 (h->frameDuration) / 1000 */
            /*  decided between shrinking/stretching */
            IF( L_sub(currPlayoutDelay, L_add(targetStartUp, headRoom)) > 0)   /* time shrinking */
            {
                IF( JB4_checkDtxDropping( h ) )
                {
                    JB4_dropFromBuffer( h, sysTime );
                }
            }
            ELSE IF( L_sub(L_add(currPlayoutDelay, headRoom), targetStartUp) < 0 )   /* time stretching */
            {
                *stretchTime = true;
                move16();
            }
            return;
        }
    }

    /* adapt while DTX */
    h->targetPlayoutDelay = targetDtx;
    move32();

    /*  decided between shrinking/stretching */
    IF( L_sub(currPlayoutDelay, L_add(targetDtx, h->frameDuration)) >= 0 )   /* time shrinking */
    {
        IF( JB4_checkDtxDropping( h ) )
        {
            JB4_dropFromBuffer( h, sysTime );
        }
    }
    ELSE IF( L_sub(L_add(currPlayoutDelay, L_shr(h->frameDuration, 1)), targetDtx) < 0 )   /* time stretching */
    {
        *stretchTime = true;
        move16();
    }
}

/* function to do playout adaptation before playing the first data unit */
static void JB4_adaptFirstPlayout( JB4_HANDLE h, Word32 sysTime, Word16 *prebuffer )
{
    Word32 currPlayoutDelay;
    JB4_DATAUNIT_HANDLE firstDu;
    Word32 targetMin, targetMax, targetDtx, targetStartUp;

    /* get target delay */
    IF( JB4_INPUTBUFFER_IsEmpty( h->inputBuffer ) )
    {
        *prebuffer = true;
        move16();
        return;
    }
    JB4_targetPlayoutDelay( h, &targetMin, &targetMax, &targetDtx, &targetStartUp );

    IF(L_sub(targetStartUp, h->frameDuration) < 0)
    {
        return;
    }
    /* calculate delay if first data unit would be played now */
    firstDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );

    IF( JB4_playoutDelay( h, sysTime, firstDu->timeStamp, &currPlayoutDelay ) != 0 )
    {
        *prebuffer = true;
        move16();
        return;
    }

    IF( L_sub(L_add(currPlayoutDelay, L_shr(h->frameDuration, 1)), targetStartUp) < 0 )   /* time stretching */
    {
        *prebuffer = true;
        move16();
    }
    ELSE    /* no adaptation, start playout */
    {
        *prebuffer = false;
        move16();
    }
}

/* function to look into the buffer and check if it makes sense to drop a data unit */
static Word16 JB4_inspectBufferForDropping( const JB4_HANDLE h, Word16 *dropEarly, Word32 *buffered )
{
    Word16 inputBufferSize;
    Word32 tsDiff, bufferedTs, endTs;
    JB4_DATAUNIT_HANDLE firstDu, secondDu, lastDu;

    assert( !h->lastPoppedWasSilence );
    *dropEarly   = false;
    move16();
    *buffered    = L_deposit_l(0);
    inputBufferSize = JB4_INPUTBUFFER_Size( h->inputBuffer );

    IF( inputBufferSize == 0 )
    {
        return -1;
    }

    firstDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );
    tsDiff = L_deposit_l(0);
    /* check for loss: timestamp diff is exactly 0 in the valid case */
    IF( h->firstDataUnitPopped )
    {
        tsDiff = JB4_rtpTimeStampDiff( h->nextExpectedTs, firstDu->timeStamp );
    }

    IF( tsDiff <= 0 )
    {
        /* preview data unit to play after dropping */
        IF( sub(inputBufferSize, 1) <= 0 )
        {
            /* data unit to play missing, avoid drop followed by concealment */
            return -1;
        }
        secondDu = JB4_INPUTBUFFER_Element( h->inputBuffer, 1 );

        IF( rtpTs_sub(rtpTs_add(firstDu->timeStamp, h->frameDuration), secondDu->timeStamp) != 0 )
        {
            /* data unit to play is not available, avoid drop followed by concealment */
            return -1;
        }
        /* calculate buffered time span */
        bufferedTs = L_deposit_l(0);
    }
    ELSE IF( rtpTs_sub(tsDiff, L_mult0(2, extract_l(h->frameDuration))) == 0 )
    {
        /* data unit to play is not available, avoid dropping followed by concealment */
        return -1;
    }
    ELSE   /* seqNoDiff == 1 || seqNoDiff > 2 */
    {
        /* first data unit is not available -> drop it early to avoid concealment
         * This is very aggressive: ignores the maximum drop rate (50% drop and 50% concealment for adjacent lost),
         * but on the other hand, dropping sounds better than concealment. */
        *dropEarly = true;
        move16();
        /* data unit to drop (first one) is lost */
        bufferedTs = L_deposit_l(0);
    }

    /* add time stamp difference of last and first actually buffered data unit */
    IF( sub(inputBufferSize, 1) == 0 )
    {
        bufferedTs = rtpTs_add(bufferedTs, h->frameDuration);
    }
    ELSE
    {
        lastDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Back( h->inputBuffer );
        endTs  = rtpTs_add(lastDu->timeStamp, h->frameDuration);
        /* check for RTP time stamp wrap around */
        /* check if sign changes from negative to positive */                                           test();
        IF( L_and(firstDu->timeStamp, 0x80000000) != 0 &&
        L_and(endTs, 0x80000000) == 0 )
        {
            endTs = rtpTs_add(endTs, 0xFFFFFFFF);
        }
        bufferedTs = rtpTs_add(bufferedTs, rtpTs_sub(endTs, firstDu->timeStamp));
    }

    /* the result should not be negative */
    IF( bufferedTs < 0 )
    {
        return -1;
    }
    *buffered = bufferedTs;
    move32();
    return 0;
}

/* function to look into the buffer and check if it makes sense to drop a data unit */
static Word16 JB4_checkDtxDropping( const JB4_HANDLE h )
{
    Word16 inputBufferSize;
    JB4_DATAUNIT_HANDLE firstDu;
    Word16 droppingAllowed;

    assert( h->firstDataUnitPopped );
    assert( h->lastPoppedWasSilence );
    droppingAllowed = 1;
    move16();
    inputBufferSize = JB4_INPUTBUFFER_Size( h->inputBuffer );

    IF( inputBufferSize > 0 )
    {
        firstDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );
        /* check for loss */
        if( JB4_rtpTimeStampDiff( h->nextExpectedTs, firstDu->timeStamp ) <= 0 )
        {
            /* no not drop first active frame */
            droppingAllowed = 0;
            move16();
        }
    }
    /* else: buffer empty, allow dropping FRAME_NO_DATA */
    return droppingAllowed;
}

/* function to estimate the short term jitter */
static void JB4_estimateShortTermJitter( JB4_HANDLE h, Word32 rcvTime, Word32 rtpTimeStamp )
{
    Word32 stOffset, ltOffset, duration, maxDuration;
    Word32 jitter, minTime, maxTime;
    JB4_CIRCULARBUFFER_ELEMENT maxElement, dequedElement;

    jitter = L_deposit_l(0);
    JB4_JMF_PushPacket( h->stJmf, rcvTime, rtpTimeStamp );
    /* save delta delay */
    IF( JB4_JMF_Jitter( h->stJmf, &jitter ) == 0 )
    {
        /* compensate difference between both offsets */
        JB4_JMF_MinOffset( h->stJmf, &stOffset );
        JB4_JMF_MinOffset( h->ltJmf, &ltOffset );
        jitter = L_add(jitter, L_sub(stOffset, ltOffset));
        assert( jitter >= 0 );

        IF( JB4_CIRCULARBUFFER_IsFull( h->stJitterFifo ) )
        {
            JB4_CIRCULARBUFFER_Deque( h->stJitterFifo, &dequedElement );
            JB4_CIRCULARBUFFER_Deque( h->stTimeStampFifo, &dequedElement );
        }
        JB4_CIRCULARBUFFER_Enque( h->stJitterFifo, jitter );
        JB4_CIRCULARBUFFER_Enque( h->stTimeStampFifo, rtpTimeStamp );

        /* check for duration and discard first entries if too long */
        minTime = JB4_CIRCULARBUFFER_Front( h->stTimeStampFifo );
        maxTime = JB4_CIRCULARBUFFER_Back( h->stTimeStampFifo );

        duration = rtpTs_sub(maxTime, minTime);
        IF( duration > 0 )
        {
            maxDuration = L_mult0(4, h->timeScale);
            WHILE( rtpTs_sub(duration, maxDuration) > 0 )
            {
                JB4_CIRCULARBUFFER_Deque( h->stJitterFifo, &dequedElement );
                JB4_CIRCULARBUFFER_Deque( h->stTimeStampFifo, &dequedElement );
                minTime = JB4_CIRCULARBUFFER_Front( h->stTimeStampFifo );
                duration = rtpTs_sub(maxTime, minTime);
                IF( duration <= 0)
                {
                    BREAK;
                }
            }
        }
    }

    /* update h->stJitter */
    IF( !JB4_CIRCULARBUFFER_IsEmpty( h->stJitterFifo ) )
    {
        JB4_CIRCULARBUFFER_Max( h->stJitterFifo, &maxElement );
        /* round up to full frame duration */
        h->stJitter = L_add(maxElement, L_sub(h->frameDuration, 1));
        h->stJitter = L_mult0(idiv1616(extract_l(h->stJitter), extract_l(h->frameDuration)), extract_l(h->frameDuration));
    }
}

/* function to pop a data unit from the buffer */
static void JB4_popFromBuffer( JB4_HANDLE h, Word32 sysTime, JB4_DATAUNIT_HANDLE *pDataUnit )
{
    JB4_DATAUNIT_HANDLE nextDataUnit;
    Word32 nStretched, tsDiff;

    JB4_DATAUNIT_HANDLE tempDataUnit;
    Word32 readlen ;
    Word16 i;
    Word32 frameoffset;
    Word32 maxval, lost, total_rec ;



    JB4_DATAUNIT_HANDLE partialCopyDu;
    Word16 searchpos, endpos;

    /* check if a data unit is available */
    IF( JB4_INPUTBUFFER_IsEmpty( h->inputBuffer ) )
    {
        /* no data unit available */
        *pDataUnit = NULL;
        move16();
        h->nextExpectedTs = rtpTs_add(h->nextExpectedTs, h->frameDuration);

        IF( h->lastPoppedWasSilence )
        {
            h->nComfortNoice = L_add(h->nComfortNoice, 1);
        }
        ELSE
        {
            h->nUnavailablePopped = L_add(h->nUnavailablePopped, 1);
            h->nLostOrStretched   = L_add(h->nLostOrStretched, 1);
        }
        return;
    }

    /* preview next data unit in sequence order */
    nextDataUnit = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );

    /* check if this is the first data unit */
    IF( !h->firstDataUnitPopped )
    {
        h->firstDataUnitPopped = true;
        move16();
        /* adjust sequence numbers to avoid handling first packet as loss */
        h->nextExpectedTs = nextDataUnit->timeStamp;
        move32();
    }

    /* check if the next available data unit should already be used (time stamp order) */
    tsDiff = JB4_rtpTimeStampDiff( nextDataUnit->timeStamp, h->nextExpectedTs );

    h->totWin += 1;
    move16();
    test();
    IF ( ( L_sub(h->totWin , 3000) > 0) || ( L_sub( h->FecOffWinLen , 100) >0 ) )
    {
        maxval = h->FecOffWin[1];
        move16();
        h->optimum_offset = 1;
        move16();
        FOR( i = 2; i < MAXOFFSET ; i++ )
        {
            IF ( L_sub( h->FecOffWin[i], maxval ) > 0 )
            {
                maxval = h->FecOffWin[i] ;
                move16();
                h->optimum_offset = i ;
                move16();
            }
            h->FecOffWin[i] = 0;
            move16();
        }
        h->FecOffWin[0] = 0;
        move16();
        h->FecOffWin[1] = 0;
        move16();
        h->totWin = 0;
        move16();
        h->FecOffWinLen = 0;
        move16();

        lost = L_sub( L_add(h->nLost, h->nPartialCopiesUsed), h->last_nLost );
        total_rec = L_sub( L_add(h->nAvailablePopped , h->nUnavailablePopped), h->last_ntot );

        IF ( lost != 0 && total_rec != 0 )
        {
            h->netLossRate =  divide3232( lost , total_rec );
        }
        ELSE
        {
            h->netLossRate = 0;
        }
        h->last_nLost = L_add(h->nLost, h->nPartialCopiesUsed);
        h->last_ntot = L_add(h->nAvailablePopped , h->nUnavailablePopped);
    }



    IF( tsDiff < 0 )
    {


        readlen = JB4_INPUTBUFFER_Size( h->inputBuffer );
        move16();
        FOR ( i=0; i < readlen; i++)
        {
            tempDataUnit = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Element( h->inputBuffer, i );

            test();
            IF ( tempDataUnit->partial_frame == 0 && h->lastPoppedWasSilence == 0 )
            {
                frameoffset = Mult_32_16(JB4_rtpTimeStampDiff(  h->nextExpectedTs, tempDataUnit->timeStamp ), 1639 ) ; /* divide by 20 */
                test();
                IF ( frameoffset > 0  && (L_sub( frameoffset, MAXOFFSET) < 0) )
                {
                    h->FecOffWin[frameoffset] = L_add(h->FecOffWin[frameoffset], 1);
                }
            }
        }

        h->FecOffWinLen = L_add (h->FecOffWinLen, 1);


        /* next expected data unit is missing
         * -> conceal network loss, do time stretching or create comfort noice */
        *pDataUnit = NULL;

        /* update statistics */
        h->nextExpectedTs = rtpTs_add(h->nextExpectedTs, h->frameDuration);

        IF( h->lastPoppedWasSilence )
        {
            h->nComfortNoice = L_add(h->nComfortNoice, 1);
        }
        ELSE
        {
            h->nUnavailablePopped = L_add(h->nUnavailablePopped, 1);
            h->nLostOrStretched   = L_add(h->nLostOrStretched, 1);
        }
        return;
    }

    /* fetch the next data unit from buffer */
    *pDataUnit = nextDataUnit;
    move16();
    nextDataUnit->nextCoderType = INACTIVE;
    IF( sub(h->pre_partial_frame,1) == 0 || sub(nextDataUnit->partial_frame,1) == 0 )
    {
        IF( nextDataUnit->partial_frame )
        {
            h->pre_partial_frame = 1;
        }
        ELSE IF( h->pre_partial_frame )
        {
            h->pre_partial_frame = 0;
        }

        endpos = JB4_INPUTBUFFER_Size(h->inputBuffer);
        FOR(searchpos = 0; searchpos < endpos; searchpos++)
        {
            partialCopyDu = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Element(h->inputBuffer, searchpos);
            IF ( L_sub(partialCopyDu->timeStamp,L_add(nextDataUnit->timeStamp,partialCopyDu->duration)) == 0)
            {
                get_NextCoderType_fx( partialCopyDu->data, &nextDataUnit->nextCoderType);
                break;
            }
        }
    }
    JB4_INPUTBUFFER_Deque( h->inputBuffer, (void**)pDataUnit );


    IF ( sub(nextDataUnit->partial_frame,1) == 0 )
    {

        h->nPartialCopiesUsed = L_add(h->nPartialCopiesUsed, 1);
        readlen = JB4_INPUTBUFFER_Size( h->inputBuffer );
        move16();
        FOR ( i=0; i < readlen; i++)
        {
            tempDataUnit = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Element( h->inputBuffer, i );
            test();
            IF ( tempDataUnit->partial_frame == 0 && h->lastPoppedWasSilence == 0 )
            {
                frameoffset = Mult_32_16(JB4_rtpTimeStampDiff(  h->nextExpectedTs, tempDataUnit->timeStamp ), 1639 ) ;
                test();
                IF ( frameoffset > 0  && (L_sub( frameoffset, MAXOFFSET) < 0) )
                {
                    h->FecOffWin[frameoffset] = L_add(h->FecOffWin[frameoffset], 1);
                }
            }
        }

        h->FecOffWinLen = L_add (h->FecOffWinLen, 1);
    }




    /* update statistics */
    IF( h->nLostOrStretched != 0U )
    {
        assert( h->lastPoppedWasSilence == false );
        /* separate concealments since last available pop in lost and stretched */
        nStretched = idiv3216(tsDiff, extract_l(h->frameDuration));
        assert( nStretched == tsDiff / h->frameDuration );
        assert( h->nLostOrStretched >= nStretched );
        h->nLost = L_add(h->nLost, L_sub(h->nLostOrStretched, nStretched));
        /* jitter-induced insertion (e.g. buffer underflow) */
        h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, nStretched);
        h->nStretched = L_add(h->nStretched, nStretched);
        h->nLostOrStretched = L_deposit_l(0);
    }
    h->lastReturnedTs        = nextDataUnit->timeStamp;
    move32();
    JB4_updateLastTimingMembers( h, sysTime, nextDataUnit->timeStamp );
    h->nextExpectedTs        = rtpTs_add(nextDataUnit->timeStamp, h->frameDuration);

    if( nextDataUnit->silenceIndicator )
        h->nComfortNoice = L_add(h->nComfortNoice, 1);
    if( !nextDataUnit->silenceIndicator )
        h->nAvailablePopped = L_add(h->nAvailablePopped, 1);
    h->lastPoppedWasSilence = nextDataUnit->silenceIndicator;
    move16();
}

/* function to drop a data unit from the buffer - updates nShrinked */
static void JB4_dropFromBuffer( JB4_HANDLE h, Word32 sysTime )
{
    JB4_DATAUNIT_HANDLE nextDataUnit, dataUnit;
    Word32 tsDiff, nStretched;
    (void)sysTime;

    /* check if a data unit is available */
    IF( JB4_INPUTBUFFER_IsEmpty( h->inputBuffer ) )
    {
        return;
    }
    /* preview next data unit in sequence order */
    nextDataUnit = (JB4_DATAUNIT_HANDLE)JB4_INPUTBUFFER_Front( h->inputBuffer );

    /* check if this is the first data unit */
    IF( !h->firstDataUnitPopped )
    {
        h->firstDataUnitPopped = true;
        move16();
        /* adjust sequence numbers to avoid handling first packet as loss */
        h->nextExpectedTs = nextDataUnit->timeStamp;
        move32();
    }

    /* check if the next available data unit should already be used (time stamp order) */
    tsDiff = JB4_rtpTimeStampDiff( nextDataUnit->timeStamp, h->nextExpectedTs );

    IF( tsDiff < 0 )
    {
        /* next expected data unit is missing, remember this data unit as popped,
         * but do not count it as lost, because it will not be concealed */
        h->nextExpectedTs = rtpTs_add(h->nextExpectedTs, h->frameDuration);
        /* substract one frame from last playout delay */
        h->lastPlayoutOffset = rtpTs_sub(h->lastPlayoutOffset, h->frameDuration);

        IF( !h->lastPoppedWasSilence )
        {
            h->nShrinked = L_add(h->nShrinked, 1);
            /* modification of the output timeline due to link loss */
            h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
        }

        if( h->lastTargetTime != 0 )
        {
            h->lastTargetTime = L_add(h->lastTargetTime, h->frameDuration);
        }
        return;
    }

    /* fetch the next data unit from buffer */
    JB4_INPUTBUFFER_Deque( h->inputBuffer, (void *)&dataUnit );
    /* update statistics */
    IF( h->nLostOrStretched != 0U )
    {
        assert( h->lastPoppedWasSilence == false );
        /* separate concealments since last available pop in lost and stretched */
        nStretched = idiv3216(tsDiff, extract_l(h->frameDuration));
        assert( nStretched == tsDiff / h->frameDuration );
        assert( h->nLostOrStretched >= nStretched );

        /* convert stretching followed by shrinking to late-loss */
        IF( nStretched > 0 )
        {
            nStretched = L_sub(nStretched, 1);
            h->nLateLost = L_add(h->nLateLost, 1);
            h->nLost = L_add(h->nLost , L_sub(h->nLostOrStretched, nStretched));
            /* jitter-induced insertion (e.g. buffer underflow) */
            h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, nStretched);

            if( !dataUnit->silenceIndicator )
            {
                /* JBM induced removal of a speech frame (intentional frame dropping) */
                h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
            }
            h->nStretched = L_add(h->nStretched, nStretched);
        }
        ELSE
        {
            h->nLost = L_add(h->nLost, h->nLostOrStretched);
            h->nShrinked = L_add(h->nShrinked, 1);
            if( !dataUnit->silenceIndicator )
            {
                /* JBM induced removal of a speech frame (intentional frame dropping) */
                h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
            }
        }
        h->nLostOrStretched = L_deposit_l(0);
    }
    ELSE IF( !dataUnit->silenceIndicator )
    {
        h->nShrinked = L_add(h->nShrinked, 1);
        /* JBM induced removal of a speech frame (intentional frame dropping) */
        h->jitterInducedConcealments = L_add(h->jitterInducedConcealments, 1);
    }

    h->lastReturnedTs        = dataUnit->timeStamp;
    move32();
    h->lastPoppedWasSilence  = dataUnit->silenceIndicator;
    move16();
    h->nextExpectedTs        = rtpTs_add(dataUnit->timeStamp, h->frameDuration);

    /* substract one frame from last playout delay */
    h->lastPlayoutOffset = rtpTs_sub(h->lastPlayoutOffset, h->frameDuration);

    if( h->lastTargetTime != 0 )
        h->lastTargetTime = L_add(h->lastTargetTime, h->frameDuration);
}

/* function to calculate the playout delay based on the current jitter */
static Word16 JB4_playoutDelay( const JB4_HANDLE h, Word32 playTime, Word32 rtpTimeStamp, Word32 *delay )
{
    Word32 minOffTicks;

    IF( JB4_JMF_MinOffset( h->ltJmf, &minOffTicks ) != 0 )
    {
        return -1;
    }

    *delay = rtpTs_sub(rtpTs_sub(playTime, minOffTicks), rtpTimeStamp);
    return 0;
}

/* function to update lastPlayoutDelay and lastTargetTime after popFromBuffer() */
static void JB4_updateLastTimingMembers( JB4_HANDLE h, Word32 playTime, Word32 rtpTimeStamp )
{
    Word32 minOffTicks;

    IF( JB4_JMF_MinOffset( h->ltJmf, &minOffTicks ) != 0 )
    {
        return;
    }

    /* playoutDelay = playTime - minOffset - timeStamp */
    h->lastPlayoutOffset = rtpTs_sub(playTime, rtpTimeStamp);
    /* targetTime = minOffset + timeStamp + targetDelay */
    h->lastTargetTime = rtpTs_add(rtpTs_add(minOffTicks, rtpTimeStamp), h->targetPlayoutDelay);
}

/* function to compare the RTP time stamps of two data units: newElement==arrayElement ? 0 : (newElement>arrayElement ? +1 : -1) */
static Word32 JB4_inputBufferCompareFunction( const JB4_INPUTBUFFER_ELEMENT newElement,
        const JB4_INPUTBUFFER_ELEMENT arrayElement, Word16 *replaceWithNewElementIfEqual )
{
    JB4_DATAUNIT_HANDLE newDataUnit, arrayDataUnit;
    Word32 diff, result;

    *replaceWithNewElementIfEqual = 0;
    move16();
    newDataUnit   = (JB4_DATAUNIT_HANDLE)newElement;
    move16();
    arrayDataUnit = (JB4_DATAUNIT_HANDLE)arrayElement;
    move16();
    diff = JB4_rtpTimeStampDiff( arrayDataUnit->timeStamp, newDataUnit->timeStamp );

    IF( diff > 0 )
    {
        result = L_deposit_l(1);
    }
    ELSE IF( diff < 0 )
    {
        result = L_negate(1);
    }
    ELSE   /* equal timestamps */
    {
        result = 0;
        move32();
        test();
        IF(newDataUnit->partial_frame == 0 && arrayDataUnit->partial_frame != 0)
        {
            /* replace partial copy with primary copy */
            *replaceWithNewElementIfEqual = 1;
            move16();
        }
        ELSE IF(sub(newDataUnit->partial_frame, arrayDataUnit->partial_frame) == 0 &&
        L_sub(newDataUnit->dataSize, arrayDataUnit->dataSize) > 0)
        {
            /* if both are primary or partial: take the one with higher size (e.g. higher bitrate) */
            *replaceWithNewElementIfEqual = 1;
            move16();
        }
    }
    return result;
}

