/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef EvsRXLIB_H
#define EvsRXLIB_H

/* local headers */
#include "stat_dec_fx.h"

/*
 * ENUMS
 */

/* Receiver error enums */
typedef enum _EVS_RX_ERROR
{
    EVS_RX_NO_ERROR          = 0x0000,
    EVS_RX_MEMORY_ERROR      = 0x0001,
    EVS_RX_WRONG_PARAMS      = 0x0002,
    EVS_RX_INIT_ERROR        = 0x0003,
    EVS_RX_RECEIVER_ERROR    = 0x0004,
    EVS_RX_DECODER_ERROR     = 0x0005,
    EVS_RX_JBM_ERROR         = 0x0006,
    EVS_RX_TIMESCALER_ERROR  = 0x0007,
    EVS_RX_NOT_IMPLEMENTED   = 0x0010

} EVS_RX_ERROR;


/*
 * Structures
 */

typedef struct EVS_RX*     EVS_RX_HANDLE;

/*
 * Functions
 */

/*! Opens the EVS Receiver instance. */
EVS_RX_ERROR
EVS_RX_Open(EVS_RX_HANDLE* phEvsRX,
            Decoder_State_fx *st,
            Word16 jbmSafetyMargin,
			short frames_per_apa);

/*! Sets the name of the JBM trace file which will be created. */
EVS_RX_ERROR
EVS_RX_SetJbmTraceFileName(EVS_RX_HANDLE hEvsRX,
                           const char *jbmTraceFileName);
EVS_RX_ERROR
EVS_RX_SetPcmTraceFileName(EVS_RX_HANDLE hEvsRX,
                           const char *pcm_filename,
						   const char *quality_filename);
/*! Feeds one frame into the receiver. */
EVS_RX_ERROR
EVS_RX_FeedFrame(EVS_RX_HANDLE hEvsRX,
                 UWord8 *au,
                 Word16 auSize,
                 Word16 rtpSequenceNumber,
                 Word32 rtpTimeStamp,
                 Word32 rcvTime_ms);

/*! Retrieves one frame of output PCM data. */
EVS_RX_ERROR
EVS_RX_GetSamples(EVS_RX_HANDLE hEvsRX,
                  Word16 *nOutSamples,
                  Word16 *pcmBuf,
                  Word16 pcmBufSize,
                  Word32 systemTimestamp_ms,
				  FILE* f_mode,
				  const char* input_file,
				  short frames_per_apa) ;

Word16
EVS_RX_Get_FEC_offset( EVS_RX_HANDLE hEvsRX, Word16 *offset, Word16 *FEC_hi);

/* calculate the quality and output to quality file*/
void EVS_RX_getQuality(EVS_RX_HANDLE hEvsRX, const char* input_file, int multi_apa);
/*! Returns 1 if the jitter buffer is empty, otherwise 0. */
/*  Intended for flushing at the end of the main loop but not during normal operation! */
Word8
EVS_RX_IsEmpty(EVS_RX_HANDLE hEvsRX );

/*! Closes the receiver instance. */
EVS_RX_ERROR
EVS_RX_Close(EVS_RX_HANDLE* phEvsRX );

#endif
