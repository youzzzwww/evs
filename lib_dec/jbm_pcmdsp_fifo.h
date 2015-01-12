/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_fifo.h Ringbuffer (FIFO) with fixed capacity for audio samples. */

#ifndef PCMDSP_FIFO_H
#define PCMDSP_FIFO_H PCMDSP_FIFO_H

/* instrumentation headers */
#include "typedef.h"
/** handle for FIFO with fixed capacity */
typedef struct PCMDSP_FIFO *PCMDSP_FIFO_HANDLE;

/** Creates a FIFO.
 * @param[out] ph pointer to created handle
 * @return 0 if succeeded */
Word16 pcmdsp_fifo_create( PCMDSP_FIFO_HANDLE *ph );
/** Destroys the FIFO. */
void pcmdsp_fifo_destroy( PCMDSP_FIFO_HANDLE *ph );
/** Initializes the FIFO with a fixed maximum allowed number of audio samples.
 * @param[in] nSamples maximum allowed number of samples per channel (capacity)
 * @note Word16 gives max capacity of 34 frames @ 48000 Hz
 * @param[in] nChannels number of audio channels
 * @param[in] nBytesPerSample size in bytes per sample per channel
 * @return 0 if succeeded */
Word16 pcmdsp_fifo_init( PCMDSP_FIFO_HANDLE h, Word16 nSamples,
                         Word16 nChannels, Word16 nBytesPerSample );

/** Writes the given audio data to the FIFO.
 * @param[in] samples pointer to audio samples to append
 * @param[in] nSamplesPerChannel the number of samples per channel to append
 * @return 0 if succeeded */
Word16 pcmdsp_fifo_write( PCMDSP_FIFO_HANDLE h, const UWord8 *samples, Word16 nSamplesPerChannel );
/** Reads the given number of audio samples from the FIFO.
 * @param[in] nSamplesPerChannel the number of samples per channel to read from the FIFO
 * @param[in] samples pointer where the audio samples will be copied to
 * @return 0 if succeeded */
Word16 pcmdsp_fifo_read( PCMDSP_FIFO_HANDLE h, Word16 nSamplesPerChannel, UWord8 *samples );

/** Returns the number of samples per channel that can be read (number of currently stored samples). */
Word16 pcmdsp_fifo_nReadableSamples( const PCMDSP_FIFO_HANDLE h );

#endif /* PCMDSP_FIFO_H */
