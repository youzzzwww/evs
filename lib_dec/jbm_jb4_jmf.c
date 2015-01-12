/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/** \file jbm_jb4_jmf.cpp jitter measure fifo - a fifo used for windowed measure of network status */

/* system includes */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
/* instrumentation */
#include "options.h"
#include "stl.h"
#include "basop_util.h"
/* local includes */
#include "jbm_jb4_jmf.h"
#include "jbm_jb4_circularbuffer.h"


/** jitter measure fifo - a fifo used for windowed measure of network status */
struct JB4_JMF
{
    /** scale of system time and RTP time stamps */
    Word16                    timeScale;
    /** the window size of the fifo as time in sysTimeScale */
    Word16                    maxWindowDuration;
    /** considered fraction in 1/1000 units, e.g. 900 ignores 10% of the highest samples */
    Word16                    consideredFraction;

    /** fifo containing the delay entries (ordered by receive time) */
    JB4_CIRCULARBUFFER_HANDLE fifo;
    /** fifo containing the offset entries (ordered by receive time) */
    JB4_CIRCULARBUFFER_HANDLE offsetFifo;
    /** fifo containing the RTP times of the values in offsetFifo (ordered by receive time) */
    JB4_CIRCULARBUFFER_HANDLE timeStampFifo;
    /** flag if the first packet was already pushed */
    Word16                    firstPacketPushed;
    /** last packets system time in microseconds */
    Word32                    lastSysTime;
    /** RTP time stamp of the last pushed packet */
    Word32                    lastRtpTimeStamp;
    /** last packets calculated delay value */
    Word32                    lastDelay;
    /** number of elements to ignore for percentile calculation - value set within init */
    Word16                    nElementsToIgnore;
    /** maximum delay to avoid overflow by clock drift - value set within init */
    Word32                    maxDelay;
};


/** helper function to add an entry at back of the buffer */
static void JB4_JMF_pushBack( JB4_JMF_HANDLE h, Word32 delay, Word32 offset, Word32 time );
/** helper function to remove an entry from the front of the buffer */
static void JB4_JMF_popFront( JB4_JMF_HANDLE h );


Word16 JB4_JMF_Create( JB4_JMF_HANDLE *ph )
{
    JB4_JMF_HANDLE h = malloc( sizeof( struct JB4_JMF ) );

    JB4_CIRCULARBUFFER_Create( &h->fifo );
    JB4_CIRCULARBUFFER_Create( &h->offsetFifo );
    JB4_CIRCULARBUFFER_Create( &h->timeStampFifo );

    h->timeScale             = 1000;
    move16();
    h->maxWindowDuration     = 10000;
    move16();
    h->consideredFraction    = 1000;
    move16();
    h->firstPacketPushed     = 0;
    move16();
    h->lastSysTime           = L_deposit_l(0);
    h->lastRtpTimeStamp      = L_deposit_l(0);
    h->lastDelay             = L_deposit_l(0);
    h->nElementsToIgnore     = 0;
    move16();
    h->maxDelay              = L_deposit_l(0);

    *ph = h;
    move16();
    return 0;
}

void JB4_JMF_Destroy( JB4_JMF_HANDLE *ph )
{
    JB4_JMF_HANDLE h;

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

    JB4_CIRCULARBUFFER_Destroy( &h->fifo );
    JB4_CIRCULARBUFFER_Destroy( &h->offsetFifo );
    JB4_CIRCULARBUFFER_Destroy( &h->timeStampFifo );

    free( h );
    *ph = NULL;
    move16();
}

/* function to set the window size of the fifo and the fraction which will be considered */
Word16 JB4_JMF_Init( JB4_JMF_HANDLE h, Word16 timeScale, Word16 windowSize,
                     Word16 windowDuration, Word16 consideredFraction )
{
    Word16 divScaleFac;

    /* check the following parameters (debug only) */
    assert( windowSize != 0U );
    /* check consideredFraction given is not too low */
    assert( consideredFraction * windowSize / 1000 >= 2 );
    /* check consideredFraction given is not too high */
    assert( consideredFraction <= 1000 );

    /* store values */
    h->timeScale          = timeScale;
    move16();
    h->maxWindowDuration  = windowDuration;
    move16();
    h->consideredFraction = consideredFraction;
    move16();

    JB4_CIRCULARBUFFER_Init( h->fifo, windowSize );
    JB4_CIRCULARBUFFER_Init( h->offsetFifo, windowSize );
    JB4_CIRCULARBUFFER_Init( h->timeStampFifo, windowSize );

    /* calculate nElementsToIgnore so that the multi & divide occur only once, not every jitter calc.
     * to retain accuracy in the division use scaled division and shift back after */
    h->nElementsToIgnore = BASOP_Util_Divide3216_Scale( L_mult0( windowSize, sub( 1000, h->consideredFraction ) ), 1000, &divScaleFac );
    h->nElementsToIgnore = shl( h->nElementsToIgnore, add( divScaleFac,1 ) );
    assert(h->nElementsToIgnore == windowSize * ( 1000 - consideredFraction ) / 1000);
    /* calculate maxDelay here as to not calculate every push packet */
    h->maxDelay = L_mult0( h->timeScale, 60 );
    return 0;
}

/* function to calculate delay for the current packet */
Word16 JB4_JMF_PushPacket( JB4_JMF_HANDLE h, Word32 sysTime, Word32 rtpTimeStamp )
{
    Word32 rtpTimeDiff, sysTimeDiff;
    Word32 offset, delay;

    /* check if this is the first entry */
    IF( h->firstPacketPushed == 0 )
    {
        h->firstPacketPushed = 1;
        move16();
        h->lastSysTime       = sysTime;
        move32();
        h->lastRtpTimeStamp  = rtpTimeStamp;
        move32();
        return 0;
    }

    rtpTimeDiff = L_sub( rtpTimeStamp, h->lastRtpTimeStamp );
    sysTimeDiff = L_sub( sysTime, h->lastSysTime );
    offset      = L_sub( sysTime, rtpTimeStamp );

    /* get the delay (yes, signed!!!!) */
    delay = L_add( L_sub( sysTimeDiff, rtpTimeDiff ), h->lastDelay );

    /* remember old values */
    h->lastSysTime      = sysTime;
    move32();
    h->lastRtpTimeStamp = rtpTimeStamp;
    move32();
    h->lastDelay        = delay;
    move32();

    /* reset delay if absolute value is greater than 60s
     * to avoid overflow caused by clockdrift */                                  test();
    if( L_sub( delay, h->maxDelay ) > 0 || L_add( delay, h->maxDelay ) < 0 )
    {
        h->lastDelay = L_deposit_l(0);
    }
    JB4_JMF_pushBack( h, delay, offset, rtpTimeStamp );
    return 0;
}

/* function to get the current jitter */
Word16 JB4_JMF_Jitter( const JB4_JMF_HANDLE h, Word32 *jitter )
{
    JB4_CIRCULARBUFFER_ELEMENT min, percentile;

    /* sanity check (must not be empty) and return invalid result if there is only one entry */
    IF( sub( JB4_CIRCULARBUFFER_Size( h->fifo ), 2 ) < 0 )
    {
        return -1;
    }

    JB4_CIRCULARBUFFER_MinAndPercentile( h->fifo, h->nElementsToIgnore, &min, &percentile );

    /* return the difference between the highest considered and the smallest value */
    *jitter = L_sub( percentile, min );
    assert( percentile >= min );
    return 0;
}

/* function to get the minimum offset between received time and time stamp of all entries in the fifo */
Word16 JB4_JMF_MinOffset( const JB4_JMF_HANDLE h, Word32 *offset )
{
    JB4_CIRCULARBUFFER_ELEMENT min;

    IF( JB4_CIRCULARBUFFER_IsEmpty( h->offsetFifo ) )
    {
        return -1;
    }

    JB4_CIRCULARBUFFER_Min( h->offsetFifo, &min );

    *offset = min;
    move32();
    return 0;
}


/*****************************************************************************
 **************************** private functions ******************************
 *****************************************************************************/

/* helper function to add entry at back of the buffer */
static void JB4_JMF_pushBack( JB4_JMF_HANDLE h, Word32 delay, Word32 offset, Word32 time )
{
    Word32 minTime, maxTime, duration;

    /* check for size and discard first entry if too big */
    IF( JB4_CIRCULARBUFFER_IsFull( h->fifo ) )
    {
        JB4_JMF_popFront( h );
    }

    /* push back new entry */
    JB4_CIRCULARBUFFER_Enque( h->fifo, delay );
    JB4_CIRCULARBUFFER_Enque( h->offsetFifo, offset );
    JB4_CIRCULARBUFFER_Enque( h->timeStampFifo, time );

    /* check for duration and discard first entries if too long */
    minTime = JB4_CIRCULARBUFFER_Front( h->timeStampFifo );
    maxTime = JB4_CIRCULARBUFFER_Back( h->timeStampFifo );

    duration = L_sub( maxTime, minTime );
    test();
    WHILE( duration > 0 && L_sub( duration, h->maxWindowDuration ) > 0 )
    {
        test();
        JB4_JMF_popFront( h );
        minTime = JB4_CIRCULARBUFFER_Front( h->timeStampFifo );
        duration = L_sub( maxTime, minTime );
    }
}

/* helper function to remove an entry from the front of the buffer */
static void JB4_JMF_popFront( JB4_JMF_HANDLE h )
{
    JB4_CIRCULARBUFFER_ELEMENT tmpElement;

    /* try to remove one element - fails if empty */
    IF( JB4_CIRCULARBUFFER_Deque( h->fifo, &tmpElement ) != 0 )
    {
        return;
    }
    /* also remove offset entry */
    JB4_CIRCULARBUFFER_Deque( h->offsetFifo, &tmpElement );
    JB4_CIRCULARBUFFER_Deque( h->timeStampFifo, &tmpElement );
}

