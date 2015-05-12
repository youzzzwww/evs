/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/*! @file pcmdsp_apa.c Adaptive Playout for Audio (apa). */

/* system headers */
#include <assert.h>
#include <stdlib.h>   /* malloc(), free() */
#include <stdio.h>
/* flc header */
#include "stl.h"
/* instrumentation */
/* local headers */
#include "jbm_pcmdsp_apa.h"
#include "jbm_pcmdsp_similarityestimation.h"
#include "jbm_pcmdsp_window.h"
#include "options.h"


/*
********************************************************************************
*                         LOCAL DATA DEFINITIONS AND PROTOTYPES
********************************************************************************
*/

/* maximum number of segments/iterations in extend_frm() */
#define MAXN 10

/* definition of state struct */
struct apa_state_t
{
    /* number of right shifts to be applied to the signal before correlation functions */
    Word16 signalScaleForCorrelation;
    /* scaled input samples for similarity estimation */
    Word16 frmInScaled[2*48000/50];
    /* output buffer */
    Word16 buf_out[APA_BUF];
    Word16 l_buf_out;

    /* Hann window */
    Word16 *win;
    Word16 l_halfwin;
    /* subsample factor used for Hann window
     * used to step over x values for lower sample rates */
    Word16 win_incrementor;

    /* sampling rate [Hz] */
    Word32 rate;

    /* length of a segment [samples] */
    Word16 l_seg;

    /* length of a frame [samples] */
    Word16 l_frm;

    /* total number of processed input samples since apa_reset() */
    Word32 l_in_total;

    /* sum of inserted/removed samples since last apa_set_scale() */
    Word32 diffSinceSetScale;
    /* number of input frames since last apa_set_scale() */
    Word16 nFramesSinceSetScale;

    /* current and previous  scaling ratio [%]. */
    Word16 scale;

    /* minimum pitch length [samples] */
    Word16 p_min;

    /* search length [samples] */
    Word16 l_search;

    Word16 wss;                   /* waveform subsampling */
    Word16 css;                   /* correlation subsampling */

    Word32 targetQualityQ16;     /* Q15.16 */
    Word16 qualityred;           /* quality reduction threshold */
    Word16 qualityrise;          /* quality rising for adaptive quality thresholds */
	int    totalQuality;         /* total quality of all the scaled frames */
	short  total_scaled_count;
	int total_scaled_samples;

    Word16 last_pitch;           /* last pitch/sync position */
    Word16 bad_frame_count;      /* # frames before quality threshold is lowered */
    Word16 good_frame_count;     /* # scaled frames */

    Word16 num_channels;         /* number of input/output channels */
};


/* prototypes for local functions */

/** Converts the correlation energy to dB. */
Word16 apa_corrEnergy2dB(Word32 energy, Word16 energyExp, Word16 corr_len);

/** Increases the calculated quality of signals with low energy. */
Word16 apa_getQualityIncreaseForLowEnergy(Word16 energydB);

static Word8 logarithmic_search(const apa_state_t * ps,
                                const Word16 * signal,
                                Word16 s_start,
                                Word16 inlen,
                                Word16 offset,
                                Word16 fixed_pos,
                                Word16 corr_len,
                                Word16 wss,
                                Word16 css,
                                Word16 * synchpos);

static Word16 find_synch (apa_state_t * ps,
                          const Word16 * in,
                          Word16 l_in,
                          Word16 s_start,
                          Word16 s_len,
                          Word16 fixed_pos,
                          Word16 corr_len,
                          Word16 offset,
                          Word16 * energydBQ8,
                          Word32 * qualityQ16,
                          Word16 * synch_pos);

static Word16 copy_frm (apa_state_t * ps,
                        const Word16 frm_in[],
                        Word16 frm_out[],
                        Word16 * l_frm_out);

static Word16 shrink_frm (apa_state_t * ps,
                          const Word16 frm_in[],
                          Word16 maxScaling,
                          Word16 frm_out[],
                          Word16 * l_frm_out);

static Word16 extend_frm (apa_state_t * ps,
                          const Word16 frm_in[],
                          Word16 frm_out[],
                          Word16 * l_frm_out);


/*
********************************************************************************
*                         PUBLIC PROGRAM CODE
********************************************************************************
*/

/* Allocates memory for state struct and initializes elements. */
Word8 apa_init (apa_state_t ** pps)
{
    apa_state_t *ps;

    ps = NULL;
    move16();

    /* make sure pointer is valid */
    IF(!pps)
    {
        return 1;
    }

    /* allocate state struct */
    ps = (apa_state_t *) malloc (sizeof (apa_state_t));
    IF(!ps)
    {
        return 2;
    }
    apa_reset (ps);
    *pps = ps;
    move16();
    return 0;
}

/* Sets state variables to initial value. */
void apa_reset (apa_state_t * ps)
{
    /* init state struct */
    ps->signalScaleForCorrelation = 0;
    move16();
    ps->l_buf_out = 0;
    move16();
    ps->win = NULL;
    move16();
    ps->l_halfwin = 0;
    move16();
    ps->win_incrementor = 0;
    move16();
    ps->rate = L_deposit_l(0);
    ps->l_seg = 0;
    move16();
    ps->l_frm = 0;
    move16();
    ps->l_in_total = L_deposit_l(0);
    ps->diffSinceSetScale = L_deposit_l(0);
    ps->nFramesSinceSetScale = 0;
    move16();
    ps->scale = 100;
    move16();
    ps->p_min = 0;
    move16();
    ps->l_search = 0;
    move16();
    ps->wss = 1;
    move16();
    ps->css = 1;
    move16();
    ps->targetQualityQ16 = L_deposit_l(0);
    ps->qualityred = 0;
    move16();
    ps->qualityrise = 0;
    move16();
    ps->last_pitch = 0;
    move16();
    ps->bad_frame_count = 0;
    move16();
    ps->good_frame_count = 0;
    move16();
    ps->num_channels = 0;
    move16();
	ps->totalQuality = 0;
    move16();
    ps->total_scaled_count = 0;
    move16();
	ps->total_scaled_samples = 0;
	move16();
}

/* Sets the audio configuration. */
Word8 apa_set_rate( apa_state_t * ps, Word32 rate, Word16 num_channels, short frames_per_sample )
{
    Word16 divScaleFac;

    /* make sure pointer is valid */
    IF( ps == (apa_state_t *) NULL )
    {
        return 1;
    }

    /* assert rate is actually matching one of the supported EVS rates otherwise Hann window is wrong */
    assert( rate == 8000 || rate == 16000 || rate == 24000 || rate == 32000 || rate == 48000 );

    /* reset state struct */
    apa_reset( ps );

    /* copy rate to state struct */
    ps->rate = rate;
    move32();

    /* set number of channels */
    ps->num_channels = num_channels;
    move16();

    /*
     * several other parameters depend on the sampling rate
     * and are set below. Some "magic numbers" are used here
     * which are based on typical values of a "pitch" in
     * human voice. The pitch length is the period of the
     * base frequency and is usually assumed to be 40-240
     * samples at 16 kHz.
     */

    /* set frame size */
    /* set to 320 samples at 16 kHz */
    //ps->l_frm = BASOP_Util_Divide3216_Scale( L_mult0_3216( ps->rate, ps->num_channels ), 50, &divScaleFac );
    //ps->l_frm = shl( ps->l_frm, add( divScaleFac,1 ) );

	//change by youyou
	//ps->l_frm equal to (rate*channels/50)*frames_per_sample
	ps->l_frm = (ps->rate*ps->num_channels/50)*frames_per_sample;

    /* set segment size */
    /* in the order of a pitch, set to 160 samples at 16 kHz */
    /* used for windowing and as the correlation length, i.e., */
    /* the size of the template segment. */
    /* before basop port was originally : ps->l_seg = ( ps->rate * ps->num_channels ) / 100 );
     * but whilst frm_size is still hard-coded the seg_size can be taken from half frm_size */
    //ps->l_seg = shr( ps->l_frm, 1 );
	ps->l_seg = (( ps->rate * ps->num_channels ) / 100 )*frames_per_sample;

    /* init Hann window */
    /* Note: l_win < APA_BUF is required, which is assured */
    /* because APA_MAX_RATE/100 = l_win = 441 < 2048 = APA_BUF */
    /* Length of Hann window is independent of
     * number of channels - same window applied to all channels.
     * sample rates 24k & 48k lookup a Hann window of length of 48000/50=960,
     * where 24k subsamples (skips every second sample) */
    ps->win = pcmdsp_window_hann_640;
    move16();
    ps->l_halfwin = 320;
    move16();
    ps->win_incrementor = 1;
    move16();
    IF(L_sub(ps->rate, 48000) == 0)
    {
        ps->win = pcmdsp_window_hann_960;
        move16();
        ps->l_halfwin = 480;
        move16();
    }
    IF(L_sub(ps->rate, 24000) == 0)
    {
        ps->win = pcmdsp_window_hann_960;
        move16();
        ps->l_halfwin = 480;
        move16();
        ps->win_incrementor = 2;
        move16();
    }
    /* sample rates 8k, 16k & 32k use a Hann window of length of 640,
     * where 8k and 16k subsample */
    if(L_sub(ps->rate, 16000) == 0)
	{
		if(frames_per_sample==1){
			ps->win_incrementor = 2;
		}
		else if(frames_per_sample==2){
			ps->win_incrementor = 1;
		}
		else if(frames_per_sample==3){
			ps->win = pcmdsp_window_hann_960;
			move16();
			ps->l_halfwin = 480;
			move16();
			ps->win_incrementor = 1;
		}
		else if(frames_per_sample==4){
			ps->win = pcmdsp_window_hann_1280;
			move16();
			ps->l_halfwin = 640;
			move16();
			ps->win_incrementor = 1;
		}
	}
    move16();
    if(L_sub(ps->rate, 8000) == 0)
        ps->win_incrementor = 4;
    move16();

    /* set minimum pitch */
    /* set to 40 samples at 16 kHz */
    /* (defines min change in number of samples, i.e., abs(l_in-l_out) >= p_min) */
    /* before basop port was originally: ps->p_min = (ps->rate * ps->num_channels) / 400;
     * but for simplicity can be taken as l_seg / 4 */
    //ps->p_min = shr( ps->l_seg, 2 );
	ps->p_min = (ps->rate * ps->num_channels) / 400;

    /* set search length */
    /* must cover one pitch, set to 200 samples at 16 kHz */
    /* (the resulting maximum pitch is then p_min+l_search = 240 samples at 16 kHz) */
    /* the following is equivalent to: ps->l_search = (ps->rate * ps->num_channels) / 80; */
    //ps->l_search = BASOP_Util_Divide3216_Scale( L_mult0_3216( ps->rate, ps->num_channels ), 80, &divScaleFac );
    //ps->l_search = shl( ps->l_search, add( divScaleFac,1 ) );

	ps->l_search = BASOP_Util_Divide3216_Scale( L_mult0_3216( ps->rate, ps->num_channels ), 80/frames_per_sample, &divScaleFac );
    ps->l_search = shl( ps->l_search, add( divScaleFac,1 ) );

    ps->signalScaleForCorrelation = getSignalScaleForCorrelation(ps->rate);

    return 0;
}

/* Set scaling. */
Word8 apa_set_scale (apa_state_t * ps, Word16 scale)
{
    /* make sure pointer is valid */
    IF( ps == (apa_state_t *) NULL)
    {
        return 1;
    }

    /* check range */
    assert( scale >= APA_MIN_SCALE && scale <= APA_MAX_SCALE );

    /* do nothing if same scale is set multiple times */
    /* (otherwise scale control is confused) */
    IF( sub(ps->scale, scale) == 0 )
    {
        return 0;
    }

    /* copy to state struct */
    ps->scale = scale;
    move16();

    /* reset scaling statistics */
    ps->diffSinceSetScale = L_deposit_l(0);
    ps->nFramesSinceSetScale = 0;
    move16();
    return 0;
}

/*
********************************************************************************
*
*     Function        : apa_set_quality
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Set quality thresholds.
*
*                       quality is lower limit for minimum quality
*                       Range is [-2;2] - where positive values allow
*                       only pasting with same phase information
*                       Negative values would yield cross phased pasting
*
*                       qualityred allows dynamic lowering of lower quality
*                       bound - this gives better results for rhythmic signals
*                       Range is [0;20], meaning 0.1 lowering*qualityred
*
*                       undocumented: qualityrise (same as qualityred - other
*                       direction)
*
********************************************************************************
*/
Word8 apa_set_quality(
    apa_state_t *ps,
    Word32     qualityQ16,
    Word16     qualityred,
    Word16     qualityrise)
{
    assert(ps != (apa_state_t *) NULL);
    assert(L_sub(L_deposit_h(-2), qualityQ16) <= 0 && L_sub(qualityQ16, L_deposit_h(3)) <= 0);
    assert(qualityred > 0 && sub( qualityred, 20 ) <= 0);
    assert(qualityrise > 0 && sub(qualityrise, 20) <= 0);

    ps->targetQualityQ16 = qualityQ16;
    move32();
    ps->qualityred = qualityred;
    move16();
    ps->qualityrise = qualityrise;
    move16();
    ps->bad_frame_count = 0;
    move16();
    ps->good_frame_count = 0;
    move16();
    return 0;
}
double apa_get_averageQuality(apa_state_t * ps)
{
	if(ps->totalQuality)
		return ps->totalQuality/ps->total_scaled_count/6554*0.1;
	else return 0;
}
int apa_get_scaledCount(apa_state_t * ps)
{
	return ps->total_scaled_count;
}
int apa_get_scaledSamples(apa_state_t * ps)
{
	return ps->total_scaled_samples;
}
/*
********************************************************************************
*
*     Function        : apa_set_complexity_options
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Set complexity options
*                       Waveform subsampling computes the correlation function
*                         for certain positions only
*                       Correlation function subsampling computes the maxima
*                         for certain positions only
*
********************************************************************************
*/
Word8 apa_set_complexity_options (apa_state_t * ps, Word16 wss, Word16 css)
{
    /* make sure pointer is valid */
    assert( ps != NULL );
    assert( wss != 0 && wss <= 1000 );
    assert( css != 0 && css <= 1000 );

    ps->wss = wss;
    move16();
    ps->css = css;
    move16();
    return 0;
}

/*
********************************************************************************
*
*     Function        : apa_exit
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : The memory used for storing the state is freed.
*                       The state struct pointer is set to NULL.
*
********************************************************************************
*/
Word8 apa_exit( apa_state_t ** pps )
{
    /* ignore NULL pointer input */
    IF( *pps == (apa_state_t *) NULL )
    {
        return 0;
    }

    /* deallocate state struct */
    free (*pps);
    /* set pointer to NULL */
    *pps = NULL;
    move16();
    return 0;
}

/*
********************************************************************************
*
*     Function        : apa_exec
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Execute adaptive playout for audio, i.e., audio scaling.
*                       Will take l_in input samples from a_in[] and
*                       try to extend/shrink the amount of samples according
*                       to the last scaling set by using apa_set_scale().
*                       The actual amount of samples after scaling may vary
*                       and is given in l_out. The scaled audio samples
*                       are contained in a_out[]. Note that the scaling is
*                       achieved only in average. The input buffer must be
*                       filled with 20ms audio. The output buffer must be
*                       allocated externally and must be at least of size
*                       APA_BUF.
*                       Scaling can only be performed when a sampling rate
*                       is specified using apa_set_rate(). Otherwise,
*                       an error is returned.
*
*                       The amount of scaling is achieved by controlling the
*                       frequency of scaling. Note that the exact amount of
*                       scaling is signal dependent and is an integer
*                       multiple of a pitch. Hence, when we want to achieve
*                       a scaling of e.g. 110% then the APA module will typically
*                       forward several frames without any modification and
*                       then scale one frame by a higher amount, e.g. 143%.
*
********************************************************************************
*/
Word8 apa_exec (apa_state_t * ps, /* i/o: state struct */
                const Word16 a_in[], /* i:   input samples */
                Word16  l_in, /* i:   number of input samples */
                Word16  maxScaling, /* i: allowed number of inserted/removed samples */
                Word16  a_out[], /* o:   output samples */
                Word16 *l_out, /* o:   number of output samples */
				FILE* frecord /*o: pcm record file */)
{
    Word16 i;
    Word16 frm_in[APA_BUF];
    Word16 l_frm_out;
    Word16 l_rem;
    Word32 dl_scaled, dl_copied, l_frm_out_target;
    Word32 actScaling, expScaling;
    Word16 *frm_in_ptr, *buf_out_ptr, *buf_out_ptr1, *buf_out_ptr2;
    Word16 statsResetThreshold, statsResetShift;

    statsResetThreshold = 1637;
    move16();
    statsResetShift     = 2;
    move16();

    /* make sure no invalid output is used */
    *l_out = 0;
    move16();
    l_frm_out = 0;
    move16();

    /* make sure pointer is valid */
    IF( ps == (apa_state_t *) NULL )
    {
        return 1;
    }
    /* check available rate */
    IF( ps->rate == 0 )
    {
        return 2;
    }
    /* check size of input */
    IF( L_sub( l_in, ps->l_frm ) != 0 )
    {
        return 3;
    }

    /* get target length */
    IF(s_or(sub(ps->l_frm, 480) == 0, sub(ps->l_frm, 960) == 0))
    {
        /* decomposite ps->l_frm into 15<<i, e.g. 480=15<<5 */
        i = sub(15-4, norm_s(ps->l_frm));
        /* this only works for 20ms framing */
        assert(ps->l_frm == shl(shr(ps->l_frm, i), i));
        assert(i_mult2(sub(ps->scale, 100), add(ps->nFramesSinceSetScale, 1)) == (ps->scale - 100) * (ps->nFramesSinceSetScale + 1));
        expScaling = L_shr_r(L_mult0(i_mult2(sub(ps->scale, 100), add(ps->nFramesSinceSetScale, 1)), FL2WORD16(15*(1<<2)/100.0)), sub(15+2, i));
    }
    ELSE
    {
        /* decomposite ps->l_frm into 5<<i, e.g. 320=5<<6 */
        i = sub(15-3, norm_s(ps->l_frm));
        /* this only works for 20ms framing */
        assert(ps->l_frm == shl(shr(ps->l_frm, i), i));
        assert(i_mult2(sub(ps->scale, 100), add(ps->nFramesSinceSetScale, 1)) == (ps->scale - 100) * (ps->nFramesSinceSetScale + 1));
        expScaling = L_shr_r(L_mult0(i_mult2(sub(ps->scale, 100), add(ps->nFramesSinceSetScale, 1)), FL2WORD16(5*(1<<3)/100.0)), sub(15+3, i));
    }
    assert( expScaling >= (ps->l_frm * (ps->scale - 100.0f) / 100.0f) * (ps->nFramesSinceSetScale + 1LL) - 3);
    assert( expScaling <= (ps->l_frm * (ps->scale - 100.0f) / 100.0f) * (ps->nFramesSinceSetScale + 1LL) + 3);
    actScaling = L_sub(ps->diffSinceSetScale, L_deposit_l(ps->l_frm));
    assert( actScaling == ps->diffSinceSetScale - ps->l_frm );
    /* target number of samples for output frame */
    l_frm_out_target = L_sub(expScaling, actScaling);
    assert( l_frm_out_target == expScaling - actScaling );

    /* Wait until we have l_frm outputs samples */
    /* (required to search for correlation in the past). */
    /* If we don't have enough samples, simply copy input to output */
    IF( sub( ps->l_buf_out, ps->l_frm ) < 0 )
    {
        FOR( i = 0; i < ps->l_frm; i++ )
        {
            a_out[i] = a_in[i];
            move16();
        }
        l_frm_out = ps->l_frm;
        move16();
    }
    ELSE
    {
        buf_out_ptr = &(ps->buf_out[ sub( ps->l_buf_out, ps->l_frm ) ]);
        move16();
        frm_in_ptr = &(frm_in[ps->l_frm]);
        move16();

        /* fill input frame */
        /* 1st input frame: previous output samples */
        FOR( i = 0; i < ps->l_frm; i++ )
        {
            frm_in[i] = buf_out_ptr[i];
            move16();
        }
        /* 2nd input frame: new input samples */
        FOR( i = 0; i < ps->l_frm; i++ )
        {
            frm_in_ptr[i] = a_in[i];
            move16();
        }
		//record pcm data before time-changed
		//fwrite(frm_in_ptr, ps->l_frm, 2, frecord);
        /* no scaling */
        IF( sub( ps->scale, 100 ) == 0 )
        {
            copy_frm (ps, frm_in, a_out, &l_frm_out);
        }
        /* shrink */
        ELSE IF( sub( ps->scale, 100 ) < 0 )
        {
            shrink_frm (ps, frm_in, maxScaling, a_out, &l_frm_out);
        }
        /* extend */
        ELSE {
            extend_frm (ps, frm_in, a_out, &l_frm_out);
        }

        /* control the amount/frequency of scaling */
        IF( sub( l_frm_out, ps->l_frm ) != 0 )
        {
            test();
            IF( maxScaling != 0U &&
            sub( abs_s( sub( ps->l_frm, l_frm_out) ), maxScaling ) > 0 )
            {
                /* maxScaling exceeded -> discard scaled frame */
                copy_frm (ps, frm_in, a_out, &l_frm_out);
            }
            ELSE IF( L_sub( L_abs( l_frm_out_target ), L_deposit_l(ps->l_frm) ) > 0 )   /* ignore small difference */
            {
                dl_copied = L_sub( l_frm_out_target, L_deposit_l(ps->l_frm) );
                dl_scaled = L_sub( l_frm_out_target, L_deposit_l(l_frm_out) );
                /* discard scaled frame if copied frame is closer to target length */
                IF( L_sub( L_abs( dl_copied ), L_abs( dl_scaled ) ) < 0 )
                {
                    copy_frm (ps, frm_in, a_out, &l_frm_out);
                }
            }
        }
    }

    /* copy output to internal buffer */
    /* avoid buffer overflow: */
    /* discard old samples; always keep at least most recent l_frm samples */
    IF ( sub( add( ps->l_buf_out, l_frm_out), APA_BUF ) > 0)
    {
        buf_out_ptr1 = ps->buf_out;
        move16();

        l_rem = sub( ps->l_frm, l_frm_out );
        if( l_rem < 0 )
        {
            l_rem = 0;
            move16();
        }
        buf_out_ptr2 = &(ps->buf_out[ sub( ps->l_buf_out, l_rem )]);
        move16();

        FOR( i = 0; i < l_rem; i++ )
        {
            buf_out_ptr1[i] = buf_out_ptr2[i];
            move16();
        }
        ps->l_buf_out = l_rem;
        move16();
    }
    /* append new output samples */
    IF( sub( add( ps->l_buf_out, l_frm_out), APA_BUF ) > 0)
    {
        return 5;
    }
    {
        Word16 *buf_out_ptr = &(ps->buf_out[ps->l_buf_out]);
        FOR( i = 0; i < l_frm_out; i++)
        {
            buf_out_ptr[i] = a_out[i];
            move16();
        }
    }
    ps->l_buf_out = add( ps->l_buf_out, l_frm_out );

    /* check variable l_frm_out is non-negative since l_out being returned is unsigned */
    assert( l_frm_out >= 0 );
    *l_out = l_frm_out;
    move16();
    /* update statistics */
    ps->l_in_total = L_add( ps->l_in_total, L_deposit_l( ps->l_frm ) );
    test();
    IF( L_sub(L_abs(ps->diffSinceSetScale),
              L_sub(0x7FFFFF, L_deposit_l(sub(l_frm_out, ps->l_frm)))) < 0 &&
        sub(ps->nFramesSinceSetScale, statsResetThreshold) < 0 )
    {
        ps->diffSinceSetScale    = L_add(ps->diffSinceSetScale, L_deposit_l(sub(l_frm_out, ps->l_frm)));
        ps->nFramesSinceSetScale = add(ps->nFramesSinceSetScale, 1);
    }
    ELSE   /* scale statistics down to avoid overflow */
    {
        ps->diffSinceSetScale    = L_shr(ps->diffSinceSetScale, statsResetShift);
        ps->nFramesSinceSetScale = shr(ps->nFramesSinceSetScale, statsResetShift);
    }
	ps->total_scaled_samples += abs(ps->diffSinceSetScale);
    return 0;
}


/*
********************************************************************************
*                         LOCAL PROGRAM CODE
********************************************************************************
*/

static void get_scaling_quality(const apa_state_t * ps,
                                const Word16 * signal,
                                Word16 s_len,
                                Word16 offset,
                                Word16 corr_len,
                                Word16 pitch,
                                Word16 * energydBQ8,
                                Word32 * qualityQ16)
{
    Word32 energy, maxEnergy;
    Word32 qualityOfMaxEnergy;  /* we measure the quality for all channels and select the one with highest energy */
    Word16 half_pitch_cn;
    Word16 pitch_cn;
    Word16 three_halves_pitch_cn;
    Word16 double_pitch_cn;
    Word32 pitch_energy;
    Word32 half_pitch_energy;
    Word32 three_halves_pitch_energy;
    Word32 double_pitch_energy;
    Word16 i;


    maxEnergy = L_deposit_l(0);
    qualityOfMaxEnergy = L_deposit_l(0);

    FOR( i=0; i < ps->num_channels; i++ )
    {
        offset = 0;
        move16();

        pitch_cn = normalized_cross_correlation_self(signal, add(pitch, offset), offset, corr_len,
                   shl(ps->num_channels, 1), &pitch_energy);
        IF(pitch_cn > 0)
        {
            /* calculate correlation for double pitch */
            IF(sub(add(add(shl(pitch, 1), offset), corr_len), s_len) <= 0)
            {
                double_pitch_cn = normalized_cross_correlation_self(signal, add(shl(pitch, 1), offset),
                                  offset, corr_len, shl(ps->num_channels, 1), &double_pitch_energy);
            }
            ELSE
            {
                double_pitch_cn     = pitch_cn;
                move16();
                double_pitch_energy = L_add(pitch_energy, 0);
            }
            /* calculate correlation for three/half pitch */
            IF(sub(add(add(shr(i_mult2(pitch, 3), 1), offset), corr_len), s_len) <= 0)
            {
                three_halves_pitch_cn = normalized_cross_correlation_self(signal, add(shr(i_mult2(pitch, 3), 1),
                                        offset), offset, corr_len, shl(ps->num_channels, 1), &three_halves_pitch_energy);
            }
            ELSE
            {
                three_halves_pitch_cn     = pitch_cn;
                move16();
                three_halves_pitch_energy = L_add(pitch_energy, 0);
            }
            /* calculate correlation for half pitch */
            IF(sub(add(add(shr(pitch, 1), offset), corr_len), s_len) <= 0)
            {
                half_pitch_cn = normalized_cross_correlation_self(signal, add(shr(pitch, 1), offset),
                                offset, corr_len, shl(ps->num_channels, 1), &half_pitch_energy);
            }
            ELSE
            {
                half_pitch_cn     = pitch_cn;
                move16();
                half_pitch_energy = L_add(pitch_energy, 0);
            }

            /* combine correlation results: Q15.16 */
            *qualityQ16 = L_shr(L_mac0(L_mult0(half_pitch_cn, three_halves_pitch_cn),
                                       pitch_cn, double_pitch_cn), 14);
            BASOP_SATURATE_WARNING_OFF
            energy = L_add(L_add(L_add(pitch_energy, half_pitch_energy), three_halves_pitch_energy), double_pitch_energy);
            BASOP_SATURATE_WARNING_ON
        }
        ELSE
        {
            *qualityQ16 = L_shl(L_deposit_l(pitch_cn), 1); /* value is negative, thus pass it */
            energy = L_add(pitch_energy, 0);
        }

        /* update the quality by the quality of the signal with the highest energy */
        IF(L_sub(energy, maxEnergy) > 0)
        {
            qualityOfMaxEnergy = L_add(*qualityQ16, 0);
            maxEnergy = L_add(energy, 0);
        }

        /* go to next channel */
        ++signal;
    }
    *qualityQ16 = qualityOfMaxEnergy;
    move32();

    /* increase calculated quality of signals with low energy */
    *energydBQ8 = apa_corrEnergy2dB(maxEnergy, shl(ps->signalScaleForCorrelation, 1), corr_len);
    *qualityQ16 = L_add(*qualityQ16, L_shl(L_deposit_l(apa_getQualityIncreaseForLowEnergy(*energydBQ8)), 8));
}

/* Converts the correlation energy to dB. */
Word16 apa_corrEnergy2dB(Word32 energy, Word16 energyExp, Word16 corr_len)
{

    Word16 result, tmpScale;

    /* normalise before dividing */
    tmpScale = norm_l( energy );
    energy = L_shl( energy, tmpScale );
    energyExp = sub( energyExp, tmpScale );

    /* divide energy by corr_len */
    result = BASOP_Util_Divide3216_Scale(energy, corr_len, &tmpScale);
    energyExp = add(energyExp, tmpScale);

    result = BASOP_Util_lin2dB( L_deposit_l( result ), energyExp, 1 );
    return result;
}

/* Increases the calculated quality of signals with low energy. */
Word16 apa_getQualityIncreaseForLowEnergy(Word16 energydBQ8)
{
    Word16 qualIncreaseMinEnergy, qualIncreaseMaxEnergy, qualIncForLowEnergy; /* Q8 */

    qualIncreaseMinEnergy = -65 << 8;
    move16();
    qualIncreaseMaxEnergy = -40 << 8;
    move16();
    qualIncForLowEnergy   = 0;
    move16();

    /* increase calculated quality of signals with low energy */
    IF(sub(energydBQ8, qualIncreaseMaxEnergy) < 0)
    {
        qualIncForLowEnergy = energydBQ8;
        move16();
        if(sub(qualIncForLowEnergy, qualIncreaseMinEnergy) < 0)
        {
            qualIncForLowEnergy = qualIncreaseMinEnergy;
            move16();
        }
        if(sub(qualIncForLowEnergy, qualIncreaseMaxEnergy) > 0)
        {
            qualIncForLowEnergy = qualIncreaseMaxEnergy;
            move16();
        }
        /* -50:   (-50 - -40) / (-65 - -40) * 20
         *      = -10 / -25 * 20
         */
        qualIncForLowEnergy = divide1616(sub(qualIncForLowEnergy, qualIncreaseMaxEnergy),
                                         sub(qualIncreaseMinEnergy, qualIncreaseMaxEnergy));
        /* apply factor 2 and scale back to Q8 */
        assert(qualIncForLowEnergy >= 0);
        qualIncForLowEnergy = shr(qualIncForLowEnergy, 7-1);
        assert(qualIncForLowEnergy >= 0 && qualIncForLowEnergy <= (2 << 8));
    }
    return qualIncForLowEnergy;
}

/*
********************************************************************************
*
*     Function        : logarithmic_search
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Search for best match of a template segment using
*                       hierarchical search method:
*                       Parameter css is used for sampling every css'd correlation
*                       value. The area around the best match so far is used for
*                       further correlation value with half css-value until css=1.
*                       Search area length is always half previous search length.
*                       Parameter wss is passed to the correlation computation
*                       If the search area passes the boundaries, the search
*                       window is reduced so that it's entirely inside the
*                       boundaries.
*
********************************************************************************
*/
static Word8 logarithmic_search(const apa_state_t * ps,
                                const Word16 * signal,
                                Word16 s_start,
                                Word16 inlen,
                                Word16 offset,
                                Word16 fixed_pos,
                                Word16 corr_len,
                                Word16 wss,
                                Word16 css,
                                Word16 * synchpos)
{
    Word16 i;
    Word32 coeff;
    Word32 coeff_max;
    Word16 s_start_old, s_len_old;

    DO
    {
        coeff_max = 0x80000000; /* will always be overwritten with result of first correlation */           move32();

        FOR(i = s_start; i < s_start+inlen; i += css)
        {
            test();
            IF( sub(wss,1) == 0 && sub(ps->num_channels, 1) == 0 )
            {
                coeff = cross_correlation_self(signal, add(i, offset), add(fixed_pos, offset), corr_len);
            }
            ELSE
            {
                coeff = cross_correlation_subsampled_self(signal, add(i, offset), add(fixed_pos, offset),
                corr_len, i_mult2(wss, ps->num_channels));
            }

            /* update max corr */
            IF( sub(ps->scale, 100) < 0 )
            {
                /* shrinking: prefer greater synchpos for equal coeff */
                BASOP_SATURATE_WARNING_OFF
                IF(L_sub(coeff, coeff_max) >= 0)
                {
                    coeff_max = L_add(coeff, 0);
                    *synchpos = i;
                    move16();
                }
                BASOP_SATURATE_WARNING_ON
            }
            ELSE
            {
                /* extending: prefer smaller synchpos for equal coeff */
                BASOP_SATURATE_WARNING_OFF
                IF(L_sub(coeff, coeff_max) > 0)
                {
                    coeff_max = L_add(coeff, 0);
                    *synchpos = i;
                    move16();
                }
                BASOP_SATURATE_WARNING_ON
            }
        }
        /* backup old search range */
        s_start_old = s_start;
        move16();
        s_len_old = inlen;
        move16();

        css = shr( css, 1 );
        inlen = shr( inlen, 1 );
        s_start_old = s_start;
        move16();
        s_start = sub( *synchpos, shr( inlen, 1 ) );

        if( sub(s_start,s_start_old) < 0 )
        {
            s_start = s_start_old;
            move16();
        }

        IF( sub( add(s_start,inlen), add(s_start_old,s_len_old) ) > 0 )
        {
            inlen = add( sub( s_start_old, s_start), s_len_old );
        }
    } WHILE( sub( css, 2 ) > 0 );
    return 0;
}

/*
********************************************************************************
*
*     Function        : find_synch
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Find the best match of an template segment within
*                       a search region by similarity measures.
*
********************************************************************************
*/
static Word16 find_synch (apa_state_t * ps,
                          const Word16 * in,
                          Word16 l_in,
                          Word16 s_start,
                          Word16 s_len,
                          Word16 fixed_pos,
                          Word16 corr_len,
                          Word16 offset,
                          Word16 * energydBQ8,
                          Word32 * qualityQ16,
                          Word16 * synch_pos)
{
    assert( (corr_len - 1 + s_start + s_len - 1 + offset) < l_in );
    assert( (corr_len - 1 + fixed_pos + offset) < l_in);

    /* pass last pitch to search function as prediction value */
    *synch_pos = ps->last_pitch;
    move16();
    logarithmic_search(ps,
                       in,
                       s_start,
                       s_len,
                       offset,
                       fixed_pos,
                       corr_len,
                       ps->wss,
                       i_mult2(ps->css, ps->num_channels),
                       synch_pos);
    /* assert synch_pos is cleanly divisible by number of channels */
    assert( *synch_pos % ps->num_channels == 0 );

    *qualityQ16 = L_deposit_l(0);
    get_scaling_quality(ps, in, l_in, offset, corr_len,
                        abs_s( sub(*synch_pos, fixed_pos) ), energydBQ8, qualityQ16);
    ps->last_pitch = *synch_pos;
    move16();
    return 0;
}


/*
********************************************************************************
*
*     Function        : copy_frm
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Copy an audio.
*
*                       The frame size is fixed to ps->l_frm. The input data
*                       is stored in frm_in[], where the first ps->l_frm samples
*                       shall include the previous output frame and the second
*                       ps->l_frm samples shall contain the current input frame.
*                       The output frame is stored in frm_out[] and contains
*                       l_frm_out = ps->l_frm.
*
*                       The first ps->l_frm input samples are not used by
*                       this function and are only provided for a consistent
*                       function call with shrink_frm() and extend_frm().
*
********************************************************************************
*/
static Word16 copy_frm (apa_state_t * ps,
                        const Word16 frm_in[], Word16 frm_out[], Word16 * l_frm_out)
{
    Word16 i;

    /* only 2nd input frame is used */
    frm_in += ps->l_frm;

    /* copy frame */
    FOR( i = 0; i < ps->l_frm; i++ )
    {
        frm_out[i] = frm_in[i];
        move16();
    }

    /* set output length */
    *l_frm_out = ps->l_frm;
    move16();
    return 0;
}

/*
********************************************************************************
*
*     Function        : shrink_frm
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Shrink the length of an audio frame using the WSOLA
*                       algorithm.
*
*                       The frame size is fixed to ps->l_frm. The input data
*                       is stored in frm_in[], where the first ps->l_frm samples
*                       shall include the previous output frame and the second
*                       ps->l_frm samples shall contain the current input frame.
*                       The output frame is stored in frm_out[] and contains
*                       l_frm_out samples. The amount of shrinking is signal
*                       dependent.
*
*                       The first ps->l_frm input samples are not used by
*                       this function and are only provided for a consistent
*                       function call with extend_frm().
*
********************************************************************************
*/
static Word16 shrink_frm (apa_state_t * ps,
                          const Word16 frm_in[], Word16 maxScaling, Word16 frm_out[], Word16 * l_frm_out)
{
    Word16 findSynchResult;
    Word16 xtract, l_rem, s_start, s_end, l_frm, l_seg;
    Word16 i;
    Word16 over;
    Word16 energyQ8;
    Word32 qualityQ16;

    findSynchResult = 0;
    move16();
    l_frm = ps->l_frm;
    move16();
    l_seg = ps->l_seg;
    move16();

    /* only 2nd input frame is used */
    frm_in += l_frm;

    /* set search range */
    s_start = ps->p_min;
    move16();
    /* assumption made that number of channels limited to 2 for basop port */
    assert( ps->num_channels <= 2 );
    /* pre-basop conversion was: s_start = (s_start / nChans) * nChans; */
    IF( ps->num_channels == 2 )
    {
        s_start = shl( shr( s_start, 1 ), 1 );
    }
    s_end = add( s_start, ps->l_search );

    if( sub( add( s_end, l_seg ), l_frm ) >= 0 )
    {
        s_end = sub( l_frm, l_seg );
    }

    /* calculate overlap position */
    IF( isSilence( frm_in, l_seg, 10 ) )
    {
        /* maximum scaling */
        energyQ8   = -65 << 8;
        move16();
        qualityQ16 = L_deposit_h(5);

        /* set to last valid element (i.e. element[len - 1] but note for stereo last element is last pair of samples) */
        xtract = sub( s_end, ps->num_channels );
        test();
        if( maxScaling != 0U && sub( s_end, add( maxScaling, 1 ) ) > 0 )
        {
            xtract = maxScaling;
            move16();
        }
    }
    ELSE
    {
        /* find synch */
        assert(sizeof(ps->frmInScaled)/sizeof(ps->frmInScaled[0]) >= (size_t)l_frm);
        scaleSignal16( frm_in, ps->frmInScaled, l_frm, ps->signalScaleForCorrelation );
        findSynchResult = find_synch (ps, ps->frmInScaled, l_frm,
        s_start, sub(s_end, s_start), 0, l_seg, 0, &energyQ8, &qualityQ16, &xtract);
    }
    /* assert synch_pos is cleanly divisible by number of channels */
    assert( xtract % ps->num_channels == 0 );

    /* set frame overlappable - reset if necessary */
    over = 1;
    move16();

    /* test whether frame has sufficient quality */
    /* 6554=0.1 in Q15.16; 13107=0.2 in Q15.16 */
    IF(L_sub(qualityQ16, L_add(L_sub(ps->targetQualityQ16,
                                     L_mult0(ps->bad_frame_count, 6554)),
                               L_mult0(ps->good_frame_count, 13107))) < 0)
    {
        /* not sufficient */
        over = 0;
        move16();

        if( sub( ps->bad_frame_count, ps->qualityred ) < 0 )
        {
            ps->bad_frame_count = add( ps->bad_frame_count, 1 );
        }
        if( ps->good_frame_count > 0 )
        {
            ps->good_frame_count = sub( ps->good_frame_count, 1 );
        }
    }
    ELSE
    {
        /* sufficient quality */
        if( ps->bad_frame_count > 0 )
        {
            ps->bad_frame_count = sub( ps->bad_frame_count, 1 );
        }
        if( sub( ps->good_frame_count, ps->qualityrise ) < 0 )
        {
            ps->good_frame_count = add( ps->good_frame_count, 1 );
        }
		ps->totalQuality += qualityQ16;
		ps->total_scaled_count++;
    }

    /* Calculate output data */                                                 test();
    IF( over != 0 && xtract != 0 )
    {
        IF( sub( findSynchResult, 1) == 0 )
        {
            return 1;
        }
        overlapAdd(frm_in, frm_in + xtract, frm_out, l_seg, ps->num_channels,
                   ps->win + ps->l_halfwin, ps->win, ps->win_incrementor );
    }
    ELSE
    {
        xtract = 0;
        move16();
        FOR( i = 0; i < l_seg; i++ )
        {
            frm_out[i] = frm_in[i];
            move16();
        }
    }

    /* append remaining samples */
    l_rem = sub( sub( l_frm, xtract ), l_seg );
    FOR( i = 0; i < l_rem; i++ )
    {
        frm_out[l_seg + i] = frm_in[l_frm - l_rem + i];
        move16();
    }

    /* set output length */
    *l_frm_out = add( l_seg, l_rem );
    return 0;
}

/*
********************************************************************************
*
*     Function        : extend_frm
*     Tables          : <none>
*     Compile Defines : <none>
*     Return          : 0 on success, 1 on failure
*     Information     : Extend the length of an audio frame using the WSOLA
*                       algorithm.
*
*                       The frame size is fixed to ps->l_frm. The input data
*                       is stored in frm_in[], where the first ps->l_frm samples
*                       shall include the previous output frame and the second
*                       ps->l_frm samples shall contain the current input frame.
*                       The output frame is stored in frm_out[] and contains
*                       l_frm_out samples. The amount of extension is signal
*                       dependent.
*
********************************************************************************
*/
static Word16 extend_frm (apa_state_t * ps,
                          const Word16 frm_in[], Word16 frm_out[], Word16 * l_frm_out)
{
    Word16 findSynchResult;
    Word16 l_frm_out_target;
    Word16 N, n, i;
    Word16 s[MAXN + 2], s_max, s_min;
    Word16 xtract[MAXN + 2], sync_start, s_end;
    Word16 over[MAXN + 2];
    Word16 l_frm, l_seg;
    Word16 s_start, l_rem;
    Word16 sn_plus_search, sync_start_sub_pmin;
    Word16 divScaleFac;
    Word16 energyQ8;
    Word32 qualityQ16;
    const Word16 *fadeOut, *fadeIn;
    Word16 *frmInScaled, *out;

    findSynchResult = 0;
    move16();
    s_start = 0;
    move16();
    qualityQ16 = L_deposit_l(0);
    l_frm = ps->l_frm;
    move16();
    l_seg = ps->l_seg;
    move16();
    frmInScaled = NULL;
    move16();

    /* number of segments/iterations */
    /* equivalent to l_frm_out_target = l_frm * 1.5; */
    l_frm_out_target = add( l_frm, shr( l_frm, 1 ) );
    /* equivalent to (l_frm_out_target / l_seg) - 1 */
    N = BASOP_Util_Divide3216_Scale( l_frm_out_target, l_seg, &divScaleFac );
    N = sub( shl( N, add(divScaleFac,1) ), 1 );

    assert( (l_frm_out_target / l_seg) - 1 == N );

    if( sub(N, 1) < 0 )
    {
        N = 1;
        move16();
    }

    assert( N <= MAXN );

    /* calculate equally spaced search regions */
    /* s[n] are given relative to 2nd frame and point to the start of */
    /* the search region. The first segment (n=1) will not be moved. */
    /* Hence, the iterations will start with n=2. */
    s_min = sub( negate(ps->l_search), ps->p_min );
    /* (make sure not to exceed array dimension) */
    if( add( l_frm, s_min ) < 0)
    {
        s_min = negate( l_frm );
    }
    s_max = sub( sub( l_frm, shl( l_seg, 1 ) ), ps->l_search );
    if( sub( s_max, s_min ) < 0 )
    {
        N = 1;
        move16();
    }
    /* for just one segment start at s_min */
    s[2] = s_min;
    move16();
    /* else, spread linear in between s_min and s_max */
    /* (including s_min and s_max) */
    IF( sub( N, 1 ) != 0 )
    {
        FOR( n = 2; n <= (N + 1); n++ )
        {
            s[n] = BASOP_Util_Divide3216_Scale( L_mult0( sub( s_max, s_min ), sub(n,2) ), sub(N,1), &divScaleFac );
            s[n] = add( shl( s[n], add(divScaleFac,1) ), s_min );
            move16();
        }
    }

    /*
     *  Planning Phase
     */

    xtract[1] = negate(l_seg); /* make sync_start=0 in 1st iteration */         move16();
    n = 2;
    move16();
    {
        /* define synch segment (to be correlated with search region) */
        sync_start = add( xtract[sub(n,1)], l_seg );
        over[n] = 1; /* will be reset if overlap is not required */               move16();

        /* added basop port - stored results for repeated calculations */
        sn_plus_search = add( s[n], ps->l_search );
        sync_start_sub_pmin = sub( sync_start, ps->p_min );

        /* check end of search region: should be at least p_min */
        /* samples on the left of synch_start */
        /* removed basop (overwritten instead): if( sub( sn_plus_search, sub( sync_start, ps->p_min) ) < 0  */
        s_start = s[n];
        move16();
        s_end = add( s_start, ps->l_search );

        IF( sub( sn_plus_search, sync_start_sub_pmin ) >= 0 )
        {
            /* shrink search region to enforce minimum shift */
            s_end = sync_start_sub_pmin;
            move16();

            IF( sub( sn_plus_search, sync_start ) < 0 )
            {
                s_start = s[n]; /* just do it with normal start position */           move16();
            }
            ELSE IF( sub( n, add(N,1) ) == 0 )   /* move search region left for last segment */
            {
                s_start = sub( s_end, sub( ps->l_search, ps->p_min ) );
            }
            ELSE
            {
                over[n] = 0; /* don't search/overlap (just copy down) */              move16();
            }
        }

        IF( over[n] != 0 )
        {
            /* calculate overlap position */
            IF( isSilence( frm_in, l_seg, 10 ) )
            {
                /* maximum scaling */
                energyQ8   = -65 << 8;
                move16();
                qualityQ16 = L_deposit_h(5);
                xtract[n] = add( s_start, ps->num_channels );
                move16();
            }
            ELSE
            {
                /* find synch */
                IF( frmInScaled == NULL )
                {
                    frmInScaled = ps->frmInScaled;
                    move16();
                    assert(sizeof(ps->frmInScaled)/sizeof(ps->frmInScaled[0]) >= 2 * (size_t)l_frm);
                    scaleSignal16( frm_in, frmInScaled, shl( l_frm, 1 ), ps->signalScaleForCorrelation );
                }
                findSynchResult = find_synch( ps, frmInScaled, shl( l_frm, 1 ),
                s_start, sub( s_end, s_start ), sync_start,
                l_seg, l_frm, &energyQ8, &qualityQ16, &xtract[n] );
            }
            /* assert synch_pos is cleanly divisible by number of channels */
            assert( xtract[n] % ps->num_channels == 0 );

            /* test for sufficient quality */
            /* 6554=0.1 in Q15.16; 13107=0.2 in Q15.16 */
            IF(L_sub(qualityQ16, L_add(L_sub(ps->targetQualityQ16,
                                             L_mult0(ps->bad_frame_count, 6554)),
                                       L_mult0(ps->good_frame_count, 13107))) < 0)
            {
                /* not sufficient */
                over[n] = 0;
                move16();
                xtract[n] = sync_start;
                move16();

                if( sub( ps->bad_frame_count, ps->qualityred ) < 0 )
                {
                    ps->bad_frame_count = add( ps->bad_frame_count, 1 );
                }
                if( ps->good_frame_count > 0 )
                {
                    ps->good_frame_count = sub( ps->good_frame_count, 1 );
                }
            }
            ELSE
            {
                /* sufficient quality */
                if( ps->bad_frame_count > 0 )
                {
                    ps->bad_frame_count = sub( ps->bad_frame_count, 1 );
                }
                if(sub(ps->good_frame_count, ps->qualityrise) < 0)
                {
                    ps->good_frame_count = add( ps->good_frame_count, 1 );
                }
				ps->totalQuality += qualityQ16;
				ps->total_scaled_count++;
            }

            IF( findSynchResult > 0 )
            {
                return 1;
            }
        }
        ELSE
        {
            xtract[n] = sync_start;
            move16();
        }

    }

    /* Calculate output data */
    FOR( n = 2; n <= N; n++ )
    {
        test();
        IF( over[n] != 0 && sub( add( xtract[sub(n,1)], l_seg ), xtract[n] ) != 0 )
        {
            /* mix 2nd half of previous segment with 1st half of current segment */
            fadeOut = frm_in + l_frm + xtract[n - 1] + l_seg;
            fadeIn  = frm_in + l_frm + xtract[n];
            out     = frm_out + i_mult2(sub(n, 2), l_seg);
            overlapAdd(fadeOut, fadeIn, out, l_seg, ps->num_channels,
                       ps->win + ps->l_halfwin, ps->win, ps->win_incrementor );
        }
        ELSE
        {
            /* just copy down 1st half of current segment (= 2nd half of previous segment) */
            Word16       *frm_out_ptr = &(frm_out[ i_mult2( sub(n,2), l_seg ) ]);
            const Word16 *frm_in_ptr  = &(frm_in[ add( l_frm, xtract[n] ) ]);
            FOR(i = 0; i < l_seg; i++)
            {
                frm_out_ptr[i] = frm_in_ptr[i];
                move16();
            }
        }
    }

    /* append remaining samples */
    l_rem = sub( l_frm, add( xtract[N], l_seg ) );
    FOR( i = 0; i < l_rem; i++ )
    {
        frm_out[ add( i_mult2( sub(N,1), l_seg), i ) ] =
            frm_in[ add( sub( shl( l_frm,1 ), l_rem ), i ) ];
        move16();
    }

    /* set output length */
    *l_frm_out = add( i_mult2( sub(N,1), l_seg ), l_rem );
    return 0;
}

