/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_apa.h Adaptive Playout for Audio (apa). */

#ifndef PCMDSP_APA_H
#define PCMDSP_APA_H PCMDSP_APA_H

/* instrumentation */
#include "typedef.h"

/*
********************************************************************************
*                         DEFINITION OF CONSTANTS
********************************************************************************
*/

/* size of IO buffers (a_in[], a_out[]) for apa_exec() */
//#define APA_BUF (48000/50*3)
#define APA_BUF (48000/50*8)

/* min/max  scaling [%] */
//#define APA_MIN_SCALE 50
//#define APA_MAX_SCALE 150
#define APA_MIN_SCALE 30
#define APA_MAX_SCALE 170


/*
********************************************************************************
*                         DEFINITION OF DATA TYPES
********************************************************************************
*/

struct apa_state_t;
typedef struct apa_state_t apa_state_t;
/*! handle for APA */
typedef struct apa_state_t* PCMDSP_APA_HANDLE;


/*
********************************************************************************
*                         DECLARATION OF PROTOTYPES
********************************************************************************
*/

/*! Allocates memory for state struct and initializes elements.
 *  @return 0 on success, 1 on failure */
Word8 apa_init(apa_state_t **s);

/*! Sets state variables to initial value. */
void apa_reset(apa_state_t *s);

/*! Sets the audio configuration.
 *  Must be called once before processing can start.
 *  If called again during processing it will reset the state struct!
 *  Will also set a number of other state variables that depend on the sampling rate.
 *  @param[in,out] ps state
 *  @param[in] rate sample rate [Hz]
 *  @param[in] num_channels number of channels
 *  @return 0 on success, 1 on failure */
Word8 apa_set_rate(
    apa_state_t *ps,
    Word32      rate,
    Word16      num_channels,
	short frames_per_sample);

/*! Set scaling.
 *  The scale is given in % and will be valid until changed again.
 *  Must be in range [APA_MIN_SCALE,APA_MAX_SCALE].
 *  @return 0 on success, 1 on failure */
Word8 apa_set_scale(apa_state_t *s, Word16 scale);

Word8 apa_set_quality(
    apa_state_t *s,
    Word32     qualityQ16,
    Word16     qualityred,
    Word16     qualityrise);

Word8 apa_set_complexity_options(
    apa_state_t *s,
    Word16     wss,
    Word16     css);

Word8 apa_exit(
    apa_state_t **s);

Word8 apa_exec(
    apa_state_t   *s,
    const Word16  a_in[],
    Word16        l_in,
    Word16        maxScaling,
    Word16        a_out[],
    Word16        *l_out,
	FILE*         frecord);

#endif /* PCMDSP_APA_H */
