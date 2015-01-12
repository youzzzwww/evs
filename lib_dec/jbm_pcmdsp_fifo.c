/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_fifo.c Ringbuffer (FIFO) with fixed capacity for audio samples */

/* system headers */
#include <stdlib.h>
/* instrumentation headers */
#include "options.h"
#include "stl.h"
#include "basop_util.h"
/* local headers */
#include "jbm_pcmdsp_fifo.h"


/** Ringbuffer (FIFO) with fixed capacity for audio samples. */
struct PCMDSP_FIFO
{
    /** number of currently stored samples per channel */
    Word16 size;
    /** maximum allowed number of samples per channel */
    Word16 capacity;
    /** sample size in bytes per channel */
    Word16 nBytesPerSampleSet;

    /** begin of the FIFO data (pointer to bytes) */
    UWord8 *dataBegin;
    /** end of the FIFO data (pointer to bytes) */
    UWord8 *dataEnd;
    /** position of next write operation (pointer to bytes) */
    UWord8 *dataWriteIterator;
    /** position of next read operation (pointer to bytes) */
    UWord8 *dataReadIterator;
};


/* Creates a FIFO. */
Word16 pcmdsp_fifo_create( PCMDSP_FIFO_HANDLE *ph )
{
    PCMDSP_FIFO_HANDLE h = malloc( sizeof( struct PCMDSP_FIFO ) );

    h->size               = 0;
    move16();
    h->capacity           = 0;
    move16();
    h->nBytesPerSampleSet = 0;
    move16();
    h->dataBegin          = NULL;
    move16();
    h->dataEnd            = NULL;
    move16();
    h->dataWriteIterator  = NULL;
    move16();
    h->dataReadIterator   = NULL;
    move16();

    *ph = h;
    move16();
    return 0;
}

/* Destroys the FIFO. */
void pcmdsp_fifo_destroy( PCMDSP_FIFO_HANDLE *ph )
{
    PCMDSP_FIFO_HANDLE h;

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

    IF( h->dataBegin )
    {
        free( h->dataBegin );
    }
    free( h );
    *ph = NULL;
    move16();
}

/* Initializes the FIFO with a fixed maximum allowed number audio samples. */
Word16 pcmdsp_fifo_init( PCMDSP_FIFO_HANDLE h, Word16 nSamples,
                         Word16 nChannels, Word16 nBytesPerSample )
{
    Word32 nDataBytes;

    h->capacity           = nSamples;
    move16();
    h->nBytesPerSampleSet = i_mult2( nChannels, nBytesPerSample );
    nDataBytes            = L_mult0( nSamples, h->nBytesPerSampleSet);
    h->dataBegin          = malloc(nDataBytes);
    h->dataEnd            = h->dataBegin + nDataBytes;
    move16();
    h->dataWriteIterator  = h->dataBegin;
    move16();
    h->dataReadIterator   = h->dataBegin;
    move16();
    return 0;
}

/* Writes the given audio data to the FIFO. */
Word16 pcmdsp_fifo_write( PCMDSP_FIFO_HANDLE h, const UWord8 *samples, Word16 nSamplesPerChannel )
{
    Word32 nBytesToWrite, writeIter, bytesOfFirstPart, secondSize;

    /* check for empty input buffer */
    IF( nSamplesPerChannel == 0 )
    {
        return 0;
    }
    /* check, if enough space left */
    IF( sub( nSamplesPerChannel, sub( h->capacity, h->size ) ) > 0 )
    {
        return -1;
    }

    nBytesToWrite = L_mult0( nSamplesPerChannel, h->nBytesPerSampleSet );

    IF( h->dataWriteIterator + nBytesToWrite - h->dataEnd > 0 )
    {
        /* wrap around: writing two parts */
        bytesOfFirstPart = h->dataEnd - h->dataWriteIterator;
        secondSize       = L_sub( nBytesToWrite, bytesOfFirstPart );

        FOR( writeIter = 0; writeIter < bytesOfFirstPart; ++writeIter )
        {
            *h->dataWriteIterator++ = *(samples + writeIter);
            move16();
        }
        h->dataWriteIterator = h->dataBegin;
        move16();
        FOR( writeIter = 0; writeIter < secondSize; ++writeIter )
        {
            *h->dataWriteIterator++ = *(samples + bytesOfFirstPart + writeIter );
            move16();
        }
    }
    ELSE
    {
        /* no wrap around: simple write */
        FOR( writeIter = 0; writeIter < nBytesToWrite; ++writeIter )
        {
            *h->dataWriteIterator++ = *(samples + writeIter);
            move16();
        }
    }
    h->size = add( h->size, nSamplesPerChannel );
    return 0;
}

/* Reads the given number of audio samples from the FIFO. */
Word16 pcmdsp_fifo_read( PCMDSP_FIFO_HANDLE h, Word16 nSamplesPerChannel, UWord8 *samples )
{
    Word32 nBytesToRead, readIter, nBytesOfSecondPart, bytesOfFirstPart;

    /* check for empty output buffer */
    IF( nSamplesPerChannel == 0 )
    {
        return 0;
    }
    /* check, if enough bytes readable */
    IF( L_sub( nSamplesPerChannel, h->size ) > 0 )
    {
        return -1;
    }

    nBytesToRead = L_mult0( nSamplesPerChannel, h->nBytesPerSampleSet );

    IF( h->dataReadIterator + nBytesToRead - h->dataEnd > 0 )
    {
        /* wrap around: reading two parts */
        bytesOfFirstPart   = h->dataEnd - h->dataReadIterator;
        nBytesOfSecondPart = L_sub( nBytesToRead, bytesOfFirstPart );

        FOR( readIter = 0; readIter < bytesOfFirstPart; ++readIter )
        {
            *(samples + readIter) = *h->dataReadIterator++;
            move16();
        }
        h->dataReadIterator = h->dataBegin;
        move16();
        FOR( readIter = 0; readIter < nBytesOfSecondPart; ++readIter )
        {
            *(samples + bytesOfFirstPart + readIter) = *h->dataReadIterator++;
            move16();
        }
    }
    ELSE
    {
        /* no wrap around: simple read */
        FOR( readIter = 0; readIter < nBytesToRead; ++readIter )
        {
            *(samples + readIter) = *h->dataReadIterator++;
            move16();
        }
    }
    h->size = sub( h->size, nSamplesPerChannel );
    return 0;
}

/* Returns the number of samples per channel that can be read (number of currently stored samples). */
Word16 pcmdsp_fifo_nReadableSamples( const PCMDSP_FIFO_HANDLE h )
{
    return h->size;
}

