/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "options.h"
#include "prot_fx.h"
#include "EvsRXlib.h"
#include "jbm_jb4sb.h"
#include "jbm_pcmdsp_apa.h"
#include "jbm_pcmdsp_fifo.h"

#include "stl.h"
#include "basop_util.h"
#include "basop_util_jbm.h"

extern struct EVS_RX
{
    Decoder_State_fx        *st;
    JB4_HANDLE               hJBM;
    Word16                   lastDecodedWasActive;
    struct JB4_DATAUNIT      dataUnit [MAX_JBM_SLOTS];
    Word16                   nextDataUnitSlot;
    Word16                   samplesPerMs;              /* sampleRate / 1000 */
    PCMDSP_APA_HANDLE        hTimeScaler;
    PCMDSP_FIFO_HANDLE       hFifoAfterTimeScaler;
    FILE                    *jbmTraceFile;
	FILE*                    pcmRecordFile;            //store pcm data before shrink-extend
	FILE*                    qualityFile;
};

/* function to check if a frame contains a SID */
static Word16 isSidFrame( Word16 size );


/* Opens the EVS Receiver instance. */
EVS_RX_ERROR EVS_RX_Open(EVS_RX_HANDLE* phEvsRX,
                         Decoder_State_fx *st,
                         Word16 jbmSafetyMargin,
						 short frames_per_apa)
{
    EVS_RX_HANDLE hEvsRX;
    Word16 i, divScaleFac;
    Word16 wss, css;

    *phEvsRX = NULL;
    move16();

    /* Create EVS Receiver handle */
    *phEvsRX = (EVS_RX_HANDLE) calloc(1, sizeof(struct EVS_RX) );
    move16();
    IF( phEvsRX == NULL )
    {
        return EVS_RX_MEMORY_ERROR;
    }
    hEvsRX = *phEvsRX;
    move16();

    hEvsRX->st = st;
    move16();
    /* do not use codec for time stretching (PLC) before initialization with first received frame */
    st->codec_mode = 0;
    move16();

    /* open JBM */
    hEvsRX->hJBM = 0;
    move16();
    IF( JB4_Create(&(hEvsRX->hJBM)) != 0)
    {
        return EVS_RX_INIT_ERROR;
    }

    /* init JBM */
    IF(JB4_Init(hEvsRX->hJBM, jbmSafetyMargin) != 0)
    {
        return EVS_RX_INIT_ERROR;
    }

    FOR(i = 0; i < MAX_JBM_SLOTS; i++)
    {
        hEvsRX->dataUnit[i].data = malloc(MAX_AU_SIZE);
        move16();
    }
    hEvsRX->nextDataUnitSlot = 0;
    move16();

    hEvsRX->lastDecodedWasActive = 0;
    move16();
    hEvsRX->samplesPerMs  = BASOP_Util_Divide3216_Scale(st->output_Fs_fx, 1000, &divScaleFac);
    hEvsRX->samplesPerMs  = shl(hEvsRX->samplesPerMs, add(divScaleFac, 1));
    assert(hEvsRX->samplesPerMs  == st->output_Fs_fx / 1000);

    IF(L_sub(st->output_Fs_fx, 8000) == 0)
    {
        wss = 1;
        move16();
        css = 1;
        move16();
    }
    ELSE IF(L_sub(st->output_Fs_fx, 16000) == 0)
    {
        wss = 2;
        move16();
        css = 1;
        move16();
    }
    ELSE IF(L_sub(st->output_Fs_fx, 32000) == 0)
    {
        wss = 4;
        move16();
        css = 2;
        move16();
    }
    ELSE IF(L_sub(st->output_Fs_fx, 48000) == 0)
    {
        wss = 6;
        move16();
        css = 3;
        move16();
    }
    ELSE
    {
        assert(0 || "unknown sample rate!");
        wss = css = 1; /* just to avoid compiler warning */
    }

    /* initialize time scaler and FIFO after time scaler */                     test();
    test();
    test();
    test();
    test();
    IF( apa_init( &hEvsRX->hTimeScaler ) != 0 ||
        apa_set_rate( hEvsRX->hTimeScaler, st->output_Fs_fx, 1, frames_per_apa) != 0 ||
        apa_set_complexity_options( hEvsRX->hTimeScaler, wss, css ) != 0 ||
        apa_set_quality( hEvsRX->hTimeScaler, L_deposit_h(1), 4, 4 ) != 0 ||
        pcmdsp_fifo_create( &hEvsRX->hFifoAfterTimeScaler ) != 0 ||
        pcmdsp_fifo_init( hEvsRX->hFifoAfterTimeScaler, i_mult2(8, st->output_frame_fx) /* 4 frames */, 1, 2 /* Word16 */ ) != 0 )
    {
        return EVS_RX_TIMESCALER_ERROR;
    }

    return EVS_RX_NO_ERROR;
}

/* Sets the name of the JBM trace file which will be created. */
EVS_RX_ERROR
EVS_RX_SetJbmTraceFileName(EVS_RX_HANDLE hEvsRX,
                           const char *jbmTraceFileName)
{
    /* JBM trace file writing is only done for EVS testing and is not instrumented. */
    if( hEvsRX->jbmTraceFile )
        fclose( hEvsRX->jbmTraceFile );
    if( jbmTraceFileName != NULL )
    {
        hEvsRX->jbmTraceFile = fopen( jbmTraceFileName, "w" );
        if( !hEvsRX->jbmTraceFile )
        {
            return EVS_RX_WRONG_PARAMS;
        }
        fprintf( hEvsRX->jbmTraceFile, "#rtpSeqNo;rtpTs;rcvTime;playTime;active\n" );
    }
    return EVS_RX_NO_ERROR;
}
/* Sets the name of the PCM trace file to store origin decoded data. */
EVS_RX_ERROR
EVS_RX_SetPcmTraceFileName(EVS_RX_HANDLE hEvsRX,
                           const char *pcm_filename,
						   const char *quality_filename)
{
    /* JBM trace file writing is only done for EVS testing and is not instrumented. */
    if( hEvsRX->pcmRecordFile )
        fclose( hEvsRX->pcmRecordFile );
    if( pcm_filename != NULL )
    {
        hEvsRX->pcmRecordFile = fopen( pcm_filename, "wb+" );
        if( !hEvsRX->pcmRecordFile )
        {
            return EVS_RX_WRONG_PARAMS;
        }
    }

	if( hEvsRX->qualityFile )
    fclose( hEvsRX->qualityFile );
    if( quality_filename != NULL )
    {
        hEvsRX->qualityFile = fopen( quality_filename, "a+" );
        if( !hEvsRX->qualityFile )
        {
            return EVS_RX_WRONG_PARAMS;
        }
    }
    return EVS_RX_NO_ERROR;
}

/* Feeds one frame into the receiver. */
EVS_RX_ERROR
EVS_RX_FeedFrame(EVS_RX_HANDLE hEvsRX,
                 UWord8 *au,
                 Word16 auSize,
                 Word16 rtpSequenceNumber,
                 Word32 rtpTimeStamp,
                 Word32 rcvTime_ms)
{
    Word16 dataUnitSlot;
    JB4_DATAUNIT_HANDLE dataUnit;
    Word16 partialCopyFrameType, partialCopyOffset;
    Word16 result;

    assert( auSize != 0 );
    assert( (auSize + 7) / 8 <= MAX_AU_SIZE );

    /* check if frame contains a partial copy and get its offset */
    evs_dec_previewFrame(au, auSize, &partialCopyFrameType, &partialCopyOffset);

    /* create data unit for primary copy in the frame */
    dataUnitSlot = hEvsRX->nextDataUnitSlot;
    move16();
    hEvsRX->nextDataUnitSlot = add(hEvsRX->nextDataUnitSlot, 1);
    if( sub(hEvsRX->nextDataUnitSlot, MAX_JBM_SLOTS) == 0 )
    {
        hEvsRX->nextDataUnitSlot = 0;
        move16();
    }
    dataUnit = &hEvsRX->dataUnit[dataUnitSlot];
    copyWord8((Word8*)au, (Word8*)dataUnit->data, shr(add(auSize, 7), 3) );
    dataUnit->dataSize = auSize;
    move16();
    dataUnit->duration = 20;
    move32();
    dataUnit->sequenceNumber = rtpSequenceNumber;
    move16();
    dataUnit->silenceIndicator = isSidFrame( dataUnit->dataSize );
    dataUnit->timeScale = 1000;
    move32();
    dataUnit->rcvTime = rcvTime_ms;
    move32();
    dataUnit->timeStamp = rtpTimeStamp;
    move32();
    dataUnit->partial_frame = 0;
    move16();
    dataUnit->partialCopyOffset = partialCopyOffset;
    move16();
    /* add the frame to the JBM */
    result = JB4_PushDataUnit(hEvsRX->hJBM, dataUnit, rcvTime_ms);
    IF(result != 0)
    {
        return EVS_RX_JBM_ERROR;
    }
    test();
    IF(sub(partialCopyFrameType, RF_NO_DATA) != 0 && partialCopyOffset != 0)
    {
        /* create data unit for partial copy in the frame */
        dataUnitSlot = hEvsRX->nextDataUnitSlot;
        move16();
        hEvsRX->nextDataUnitSlot = add(hEvsRX->nextDataUnitSlot, 1);
        if( sub(hEvsRX->nextDataUnitSlot, MAX_JBM_SLOTS) == 0 )
        {
            hEvsRX->nextDataUnitSlot = 0;
            move16();
        }
        dataUnit = &hEvsRX->dataUnit[dataUnitSlot];
        copyWord8((Word8*)au, (Word8*)dataUnit->data, shr(add(auSize, 7), 3) );
        dataUnit->dataSize = auSize;
        move16();
        dataUnit->duration = 20;
        move32();
        dataUnit->sequenceNumber = rtpSequenceNumber;
        move16();
        dataUnit->silenceIndicator = 0; /* there are no partial copies for SID frames */ move16();
        dataUnit->timeScale = 1000;
        move32();
        dataUnit->rcvTime = rcvTime_ms;
        move32();
        dataUnit->timeStamp = rtpTs_sub(rtpTimeStamp, L_mult0(partialCopyOffset, 20));
        assert(dataUnit->timeStamp == rtpTimeStamp - partialCopyOffset * dataUnit->duration);
        dataUnit->partial_frame = 1;
        move16();
        dataUnit->partialCopyOffset = partialCopyOffset;
        move16();
        /* add the frame to the JBM */
        result = JB4_PushDataUnit(hEvsRX->hJBM, dataUnit, rcvTime_ms);
        IF(result != 0)
        {
            return EVS_RX_JBM_ERROR;
        }
    }
    return EVS_RX_NO_ERROR;
}

int getNumPerScaledFromFile(FILE* fin)
{
	int mode=-1,i=0;
	fscanf(fin, "%d \n", &mode);
	if(mode == 3)
	{
		while(mode>0)
		{
			fscanf(fin, "%d \n", &i);
			mode--;
		}
		return 4;
	}
	else if(mode ==0 || mode==1 || mode==2)
		return 1;
	else return -1;
}
/* Retrieves one frame of output PCM data. */
EVS_RX_ERROR
EVS_RX_GetSamples(EVS_RX_HANDLE hEvsRX,
                  Word16 *nOutSamples,
                  Word16 *pcmBuf,
                  Word16 pcmBufSize,
                  Word32 systemTimestamp_ms,
				  FILE *f_mode,
				  const char* input_file,
				  short frames_per_scaled)
{
    Decoder_State_fx *st;
    Word16 soundCardFrameSize, extBufferedSamples;
    Word32 extBufferedTime_ms;
    Word16 scale, maxScaling, nTimeScalerOutSamples;
    Word16 timeScalingDone, result;
    JB4_DATAUNIT_HANDLE dataUnit;
    Word16 tmp;
	//add by youyou to decode frames_per_apa once time
	//short frames_per_apa = frames_per_scaled;
	static short frames_per_apa=1;
	short i=0, mode=0;
	int previous_frame_length=0;
	short* current_pcmBuf=NULL;
	short average_scaled = 0,max_scaling_sum=0;

    assert(hEvsRX->st->output_frame_fx <= pcmBufSize);
    assert(hEvsRX->st->output_frame_fx <= APA_BUF);

    st = hEvsRX->st;
    move16();
    soundCardFrameSize = st->output_frame_fx;
    move16();
    timeScalingDone = 0;
    move16();

    /* make sure that the FIFO after decoder/scaler contains at least one sound card frame (i.e. 20ms) */
    WHILE(sub(pcmdsp_fifo_nReadableSamples(hEvsRX->hFifoAfterTimeScaler), soundCardFrameSize) < 0)
    {
        extBufferedSamples = pcmdsp_fifo_nReadableSamples( hEvsRX->hFifoAfterTimeScaler );
        extBufferedTime_ms = L_deposit_l(idiv1616U(extBufferedSamples, hEvsRX->samplesPerMs));
        dataUnit = NULL;
        move16();

		if(f_mode)
		{
			mode = getNumPerScaledFromFile(f_mode);
			if(mode == -1)
			{
				EVS_RX_getQuality(hEvsRX, input_file, 0);
				return EVS_RX_DECODER_ERROR;
			}
			if( mode != frames_per_apa)
				apa_set_rate( hEvsRX->hTimeScaler, hEvsRX->samplesPerMs*1000, 1, mode);
			frames_per_apa = mode;
		}
		average_scaled = 0;
		max_scaling_sum = 0;
		current_pcmBuf = pcmBuf;
		for(i=0; i<frames_per_apa; i++)
		{
			/* pop one access unit from the jitter buffer */
			result = JB4_PopDataUnit(hEvsRX->hJBM, systemTimestamp_ms, extBufferedTime_ms, &dataUnit, &scale, &maxScaling);
			IF(result != 0)
			{
				return EVS_RX_JBM_ERROR;
			}
			maxScaling = i_mult2(maxScaling, hEvsRX->samplesPerMs);
			/* add by youyou to summarize scaled parameters */
			average_scaled += scale;
			max_scaling_sum += maxScaling;
			/* avoid time scaling multiple times in one sound card slot */
			IF(sub(scale, 100) != 0)
			{
				if( timeScalingDone != 0 )
				{
					scale = 100;
					move16();
				}
				timeScalingDone = 1;
				move16();
			}		

			/* copy bitstream into decoder state */
			IF(dataUnit)
			{
				IF( sub(st->codec_mode,0) != 0 )
				{
					tmp = 0;
					if (sub(dataUnit->partial_frame,1)==0)
					{
						tmp = 1;
					}
					read_indices_from_djb_fx( st, dataUnit->data, dataUnit->dataSize, tmp, dataUnit->nextCoderType );

					IF(dataUnit->partial_frame != 0)
					{
						st->codec_mode = MODE2;
						st->use_partial_copy = 1;
					}
				}
				ELSE /* initialize decoder with first received frame */
				{
					/* initialize, since this is needed within read_indices_from_djb, to correctly set st->last_codec_mode */
					st->ini_frame_fx = 0;
					/* initialize st->last_codec_mode, since this is needed for init_decoder() */
					read_indices_from_djb_fx( st, dataUnit->data, dataUnit->dataSize, 0, 0 );
					assert(st->codec_mode != 0);
					init_decoder_fx( st );

					/* parse frame again because init_decoder() overwrites st->total_brate_fx */
					read_indices_from_djb_fx( st, dataUnit->data, dataUnit->dataSize, 0, 0 );
				}

			}
			ELSE IF( st->codec_mode != 0 )
			{
				read_indices_from_djb_fx( st, NULL, 0, 0, 0 );
			}

			/* run the main decoding routine */
			current_pcmBuf += previous_frame_length;
			SUB_WMOPS_INIT("evs_dec");
			IF( sub(st->codec_mode, MODE1) == 0 )
			{
				IF( st->Opt_AMR_WB_fx )
				{
					amr_wb_dec_fx( current_pcmBuf, st );
				}
				ELSE
				{
					evs_dec_fx( st, current_pcmBuf, FRAMEMODE_NORMAL );
				}
			}
			ELSE IF( sub(st->codec_mode, MODE2) == 0 )
			{
				IF(st->bfi_fx == 0)
				{
					evs_dec_fx(st, current_pcmBuf, FRAMEMODE_NORMAL);
				}
				ELSE IF ( sub(st->bfi_fx,2) == 0 )
				{
					evs_dec_fx(st, current_pcmBuf, FRAMEMODE_FUTURE);   /* FRAMEMODE_FUTURE */
				}
				ELSE /* conceal */
				{
					evs_dec_fx(st, current_pcmBuf, FRAMEMODE_MISSING);
				}
			}
			END_SUB_WMOPS;
			test();
			IF( sub(st->codec_mode, MODE1) == 0 || sub(st->codec_mode, MODE2) == 0 )
			{
				/* increase the counter of initialization frames */
				if( sub(st->ini_frame_fx, MAX_FRAME_COUNTER) < 0 )
				{
					st->ini_frame_fx = add(st->ini_frame_fx, 1);
				}
			}
			ELSE /* codec mode to use not known yet */
			{
				set16_fx( current_pcmBuf, 0, st->output_frame_fx );
			}

			IF(dataUnit != NULL)
			{
				IF(dataUnit->partial_frame != 0)
				{
					hEvsRX->lastDecodedWasActive = 1;
					move16();
				}
				ELSE
				{
					hEvsRX->lastDecodedWasActive = s_xor(dataUnit->silenceIndicator, 1);
				}
			}
			previous_frame_length = st->output_frame_fx;
			//fwrite(current_pcmBuf,previous_frame_length,2,hEvsRX->pcmRecordFile);
		}

		scale = average_scaled/frames_per_apa;
		maxScaling = max_scaling_sum;
        /* limit scale to range supported by time scaler */
        if(sub(scale, APA_MIN_SCALE) < 0)
            scale = APA_MIN_SCALE;
        move16();
        if(sub(scale, APA_MAX_SCALE) > 0)
            scale = APA_MAX_SCALE;
        move16();
        /* apply time scaling on decoded/concealed samples */
        IF( apa_set_scale( hEvsRX->hTimeScaler, scale ) != 0 )
        {
            return EVS_RX_TIMESCALER_ERROR;
        }

		//fwrite(pcmBuf,st->output_frame_fx*frames_per_apa,2,hEvsRX->pcmRecordFile);
		result = apa_exec( hEvsRX->hTimeScaler, pcmBuf, st->output_frame_fx*frames_per_apa,
                           maxScaling, pcmBuf, &nTimeScalerOutSamples, hEvsRX->pcmRecordFile );
        IF( result != 0 )
        {
            return EVS_RX_TIMESCALER_ERROR;
        }
        assert(nTimeScalerOutSamples <= pcmBufSize);
        assert(nTimeScalerOutSamples <= APA_BUF);
        /* append scaled samples to FIFO */
        IF( pcmdsp_fifo_write( hEvsRX->hFifoAfterTimeScaler,
                               (UWord8*)pcmBuf, nTimeScalerOutSamples ) != 0 )
        {
            return EVS_RX_TIMESCALER_ERROR;
        }
        /* write JBM trace file entry */
        /* JBM trace file writing is only done for EVS testing and is not instrumented. */
        if( hEvsRX->jbmTraceFile )
        {
            /* the first sample of the decoded/concealed frame will be played after the samples in the ring buffer */
            double playTime = systemTimestamp_ms + extBufferedSamples * 1000.0 / st->output_Fs_fx;
            /* rtpSeqNo;rtpTs;rcvTime;playTime;active\n */
            if( dataUnit )
            {
                if(dataUnit->partial_frame == 1)
                {
                    fprintf( hEvsRX->jbmTraceFile, "%d;%d;%d;%f;%d;%d\n",
                             -1, -1, -1, playTime, (int)hEvsRX->lastDecodedWasActive, dataUnit->partialCopyOffset );
                }
                else
                {
                    fprintf( hEvsRX->jbmTraceFile, "%u;%u;%u;%f;%d\n",
                             dataUnit->sequenceNumber, dataUnit->timeStamp, dataUnit->rcvTime,
                             playTime, (int)hEvsRX->lastDecodedWasActive );
                }

            }
            else
            {
                fprintf( hEvsRX->jbmTraceFile, "%d;%d;%d;%f;%d\n",
                         -1, -1, -1,
                         playTime, (int)hEvsRX->lastDecodedWasActive );
            }
        }
    }

    /* fetch one frame for the sound card from FIFO */
    *nOutSamples = soundCardFrameSize;
    move16();
    IF( pcmdsp_fifo_read( hEvsRX->hFifoAfterTimeScaler, *nOutSamples, (UWord8*)pcmBuf ) != 0 )
    {
        return EVS_RX_TIMESCALER_ERROR;
    }
    return EVS_RX_NO_ERROR;
}

Word16
EVS_RX_Get_FEC_offset( EVS_RX_HANDLE hEvsRX, Word16 *offset, Word16 *FEC_hi)
{
    *offset = (Word16)JB4_getFECoffset(hEvsRX->hJBM);
    move16();
    *FEC_hi = (Word16)JB4_FECoffset(hEvsRX->hJBM);
    move16();

    return 0;
}


/* Returns 1 if the jitter buffer is empty, otherwise 0. */
Word8
EVS_RX_IsEmpty(EVS_RX_HANDLE hEvsRX )
{
    Word8 isEmpty;

    isEmpty = 0;
    move16();
    if(JB4_bufferedDataUnits(hEvsRX->hJBM) == 0 )
    {
        isEmpty = 1;
        move16();
    }
    return isEmpty;
}

void EVS_RX_getQuality(EVS_RX_HANDLE hEvsRX,const char* input_file,int multi_apa)
{
	fprintf(hEvsRX->qualityFile, "%s, %d, %f, %d \n",
		input_file,
		multi_apa,
		apa_get_averageQuality(hEvsRX->hTimeScaler),
		apa_get_scaledSamples(hEvsRX->hTimeScaler)/hEvsRX->samplesPerMs
		);
}
/* Closes the receiver instance. */
EVS_RX_ERROR
EVS_RX_Close(EVS_RX_HANDLE* phRX )
{
    Word16 i;

    /* Free all memory */
    test();
    IF( phRX == NULL || *phRX == NULL )
    {
        return EVS_RX_NO_ERROR;
    }

    destroy_decoder( (*phRX)->st );

    IF( (*phRX)->hJBM )
    {
        JB4_Destroy( &(*phRX)->hJBM );
    }

    FOR(i = 0; i < MAX_JBM_SLOTS; i++)
    {
        free((*phRX)->dataUnit[i].data);
    }

    IF( (*phRX)->hTimeScaler )
    {
        apa_exit( &(*phRX)->hTimeScaler );
    }

    IF( (*phRX)->hFifoAfterTimeScaler )
    {
        pcmdsp_fifo_destroy( &(*phRX)->hFifoAfterTimeScaler );
    }

    if( (*phRX)->jbmTraceFile )
        fclose( (*phRX)->jbmTraceFile );
	if( (*phRX)->pcmRecordFile )
        fclose( (*phRX)->pcmRecordFile );
	if( (*phRX)->qualityFile )
		fclose( (*phRX)->qualityFile );
    free( *phRX );
    *phRX = NULL;
    move16();
    phRX = NULL;
    move16();
    return EVS_RX_NO_ERROR;
}

/* function to check if a frame contains a SID */
static Word16 isSidFrame( Word16 size )
{
    Word16 ret;

    ret = 0;
    move16();
    if(sub(size, SID_1k75 / 50) == 0)
    {
        ret = 1; /* AMR-WB SID */                                               move16();
    }
    if(sub(size, SID_2k40 / 50) == 0)
    {
        ret = 1; /* EVS SID */                                                  move16();
    }
    return ret;
}

